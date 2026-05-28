/**
 * local_planner.c — Live-LiDAR replan + stall recovery.
 *
 * LP_MODE_PURE_PURSUIT
 *   Follow the global A* path.
 *   • LiDAR ≥ 3 returns within LP_DETECT_DIST_MM (750 mm from sensor = 350 mm
 *       clearance ahead of bumper): request one A* replan, keep driving.
 *       A* finishes in < 300 ms; car reaches bumper-contact in 2+ s → smooth.
 *   • LiDAR ≥ 3 returns within LP_STOP_DIST_MM (550 mm from sensor = 150 mm
 *       clearance ahead of bumper): too close for smooth reroute →
 *       LP_MODE_WAIT_CLEAR + replan.
 *   • Encoder displacement < LP_STALL_DISP_MM_ODOM for LP_STALL_CYCLES_ODOM
 *       odom ticks (~100 Hz): hidden obstacle — → LP_MODE_REVERSING.
 *
 * LP_MODE_REVERSING
 *   Reverse LP_REVERSE_TARGET_MM (1 m) by odom, then → LP_MODE_STALL_TURN.
 *
 * LP_MODE_STALL_TURN
 *   Drive forward LP_TURN_FWD_MM with LP_MAX_STEER_RAD (50°) hard steer,
 *   alternating direction each recovery, then → LP_MODE_WAIT_CLEAR + replan.
 *
 * LP_MODE_WAIT_CLEAR
 *   Zero-speed hold.  Exits via local_planner_reset_waypoint() when a fresh
 *   A* path has been activated by task_path_exec.
 */

#include "local_planner.h"
#include "command_gen.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#define LP_PI  3.14159265f

/* ════════════════════════════════════════════════════════════════════════════
 * Tuning
 * ════════════════════════════════════════════════════════════════════════════ */

/* LiDAR is mounted LP_LIDAR_FWD_OFFSET_MM behind the car's front bumper.
 * All obstacle distances are specified as clearance ahead of the bumper;
 * the actual LiDAR threshold = clearance + offset. */
#define LP_LIDAR_FWD_OFFSET_MM  400.0f

/* 350 mm clearance ahead of bumper → 750 mm from sensor.
 * Triggers a replan; car keeps moving. */
#define LP_DETECT_DIST_MM     750.0f
/* 150 mm clearance ahead of bumper → 550 mm from sensor.
 * Too close — stop and wait for new path. */
#define LP_STOP_DIST_MM       550.0f
/* Minimum returns inside the corridor to confirm a real obstacle (noise gate). */
#define LP_OBSTACLE_MIN_PTS     3

/* Car body width = 280 mm; at max (hard) steer the swept envelope widens to
 * ~350 mm → half = 175 mm.  Use the wider value so the corridor covers every
 * reachable path. */
#define LP_CORRIDOR_HALF_WIDTH_MM 175.0f

/* Waypoint tracking */
#define LP_WP_REACH_MM        200.0f

/* Stall: driven by local_planner_odom_tick() at odom rate (~100 Hz, 10 ms/pkt).
 * At 100 Hz: 1 mm/pkt ≈ 100 mm/s.  Car truly stuck → < 0.3 mm/pkt. */
#define LP_STALL_DISP_MM_ODOM   1.0f   /* mm per 10 ms odom packet */
#define LP_STALL_CYCLES_ODOM   80      /* 0.8 s at 100 Hz */

/* Stall recovery */
#define LP_REVERSE_SPEED_MM_S  120.0f
#define LP_REVERSE_TARGET_MM  1000.0f  /* 1 m */
#define LP_TURN_SPEED_MM_S     100.0f
#define LP_TURN_FWD_MM         300.0f  /* forward arc distance during turn */
#define LP_MAX_STEER_RAD         0.873f /* 50° hard steer */

/* ════════════════════════════════════════════════════════════════════════════
 * State
 * ════════════════════════════════════════════════════════════════════════════ */

typedef struct {
    lp_mode_t mode;
    uint8_t   current_wp_idx;

    /* Stall detection — updated by local_planner_odom_tick() at 100 Hz */
    uint8_t   stall_cycles;
    uint8_t   stall_count;        /* total recoveries; odd/even sets turn direction */

    /* Recovery phases — distance accumulated by local_planner_odom_tick() */
    float     recovery_accum_mm;
    float     reverse_heading;    /* backward direction locked at stall entry */
    float     turn_steer_rad;     /* ±LP_MAX_STEER_RAD, set at STALL_TURN entry */

    bool      replan_requested;
    bool      replan_pending;     /* blocks duplicate requests while A* runs */
    bool      enabled;            /* false until local_planner_enable() */

    /* Cached by local_planner_update(); read by local_planner_odom_tick(). */
    pose_t    last_pose;          /* most recent scan-cycle pose */
    bool      path_was_active;    /* true when a path was loaded last scan cycle */
} lp_state_t;

static lp_state_t s; /* zero-initialised by BSS */

/* ════════════════════════════════════════════════════════════════════════════
 * Helpers
 * ════════════════════════════════════════════════════════════════════════════ */

static float lp_norm(float a)
{
    while (a >  LP_PI) a -= 2.0f * LP_PI;
    while (a < -LP_PI) a += 2.0f * LP_PI;
    return a;
}

/* Count LiDAR hits inside a forward rectangular corridor.
 * Converts polar → Cartesian: fwd = r·cos(θ), lat = r·sin(θ).
 * Only counts points with fwd > 0 (ahead of sensor) and |lat| ≤ half_width_mm.
 * fwd_limit_mm already includes LP_LIDAR_FWD_OFFSET_MM so the range is
 * measured from the sensor, not the bumper. */
static int lp_count_path_hits(const lidar_scan_t *scan,
                               float fwd_limit_mm, float half_width_mm)
{
    int hits = 0;
    for (uint16_t i = 0; i < scan->count; i++) {
        float r = scan->points[i].r_mm;
        if (r < 10.0f || r > fwd_limit_mm) continue;
        float a   = scan->points[i].theta_deg * (LP_PI / 180.0f);
        float fwd = r * cosf(a);
        if (fwd <= 0.0f) continue;
        if (fabsf(r * sinf(a)) > half_width_mm) continue;
        hits++;
    }
    return hits;
}

/* Count LiDAR returns in a polar sector — used only for the rear-obstacle
 * check during REVERSING.  sector_center / sector_half in radians. */
static int lp_count_hits(const lidar_scan_t *scan, float dist_mm,
                         float sector_center_rad, float sector_half_rad)
{
    int hits = 0;
    for (uint16_t i = 0; i < scan->count; i++) {
        float r = scan->points[i].r_mm;
        if (r < 10.0f || r > dist_mm) continue;
        float a = scan->points[i].theta_deg * (LP_PI / 180.0f);
        if (fabsf(lp_norm(a - sector_center_rad)) > sector_half_rad) continue;
        hits++;
    }
    return hits;
}

/* Request one replan (guarded by replan_pending). */
static void lp_request_replan(const char *reason)
{
    if (!s.replan_pending) {
        s.replan_requested = true;
        s.replan_pending   = true;
        printf("[LP] replan: %s\n", reason);
    }
}

/* ════════════════════════════════════════════════════════════════════════════
 * Pure Pursuit stub
 * ════════════════════════════════════════════════════════════════════════════ */

static control_frame_t lp_pure_pursuit(const path_t *path, const pose_t *pose)
{
    control_frame_t cmd;
    memset(&cmd, 0, sizeof(cmd));
    if (!path || path->length == 0) return cmd;

    while (s.current_wp_idx + 1u < path->length) {
        float dx = path->waypoints[s.current_wp_idx].x - pose->x;
        float dy = path->waypoints[s.current_wp_idx].y - pose->y;
        if (sqrtf(dx * dx + dy * dy) < LP_WP_REACH_MM)
            s.current_wp_idx++;
        else
            break;
    }
    if (s.current_wp_idx >= path->length)
        s.current_wp_idx = (uint8_t)(path->length - 1u);

    return command_gen_compute(pose, &path->waypoints[s.current_wp_idx]);
}

/* ════════════════════════════════════════════════════════════════════════════
 * Public API
 * ════════════════════════════════════════════════════════════════════════════ */

void local_planner_init(float robot_radius_mm)
{
    (void)robot_radius_mm;
    memset(&s, 0, sizeof(s));
    s.mode = LP_MODE_PURE_PURSUIT;
}

void local_planner_enable(void)
{
    s.enabled = true;
    printf("[LP] enabled\n");
}

void local_planner_reset_waypoint(void)
{
    s.current_wp_idx   = 0;
    s.stall_cycles     = 0;
    s.recovery_accum_mm = 0.0f;
    s.replan_pending   = false;
    if (s.mode == LP_MODE_WAIT_CLEAR)
        s.mode = LP_MODE_PURE_PURSUIT;
}

lp_mode_t local_planner_get_mode(void)      { return s.mode; }
bool      local_planner_replan_needed(void) { return s.replan_requested; }
void      local_planner_clear_replan(void)  { s.replan_requested = false; }


bool local_planner_update(const quadtree_map_t *map,
                          const pose_t         *raw_pose,
                          const path_t         *global_path,
                          bool                  override_flag,
                          const lidar_scan_t   *live_scan,
                          control_frame_t      *out_cmd)
{
    (void)map;

    if (!raw_pose || !out_cmd) return false;
    if (!s.enabled)            return false;
    if (override_flag)         return false;

    /* Cache pose and path state for local_planner_odom_tick(). */
    s.last_pose       = *raw_pose;
    s.path_was_active = global_path && global_path->length > 0;

    memset(out_cmd, 0, sizeof(*out_cmd));
    out_cmd->t_heading = raw_pose->theta;

    /* ── WAIT_CLEAR: hold position until new A* path arrives ─────────────── */
    if (s.mode == LP_MODE_WAIT_CLEAR)
        return true;

    /* ── REVERSING: odom_tick accumulates distance and transitions.
     *    Only abort early here if a rear obstacle appears. ─────────────────── */
    if (s.mode == LP_MODE_REVERSING) {
        bool rear_blocked = live_scan && live_scan->count > 0
            && lp_count_hits(live_scan, LP_STOP_DIST_MM,
                             LP_PI, LP_PI / 2.0f) >= LP_OBSTACLE_MIN_PTS;
        if (rear_blocked) {
            printf("[LP] rear obstacle — abort reverse, stall turn\n");
            s.mode              = LP_MODE_STALL_TURN;
            s.recovery_accum_mm = 0.0f;
            s.turn_steer_rad    = (s.stall_count % 2u == 0u)
                                  ? +LP_MAX_STEER_RAD : -LP_MAX_STEER_RAD;
            /* fall through to STALL_TURN */
        } else {
            out_cmd->tx        = LP_REVERSE_SPEED_MM_S * cosf(s.reverse_heading);
            out_cmd->ty        = LP_REVERSE_SPEED_MM_S * sinf(s.reverse_heading);
            out_cmd->t_heading = s.reverse_heading;
            out_cmd->t_speed   = LP_REVERSE_SPEED_MM_S;
            return true;
        }
    }

    /* ── STALL_TURN: odom_tick accumulates distance and transitions.
     *    Just output the steering command here. ───────────────────────────── */
    if (s.mode == LP_MODE_STALL_TURN) {
        float arc_hdg      = lp_norm(s.last_pose.theta + s.turn_steer_rad);
        out_cmd->tx        = LP_TURN_SPEED_MM_S * cosf(arc_hdg);
        out_cmd->ty        = LP_TURN_SPEED_MM_S * sinf(arc_hdg);
        out_cmd->t_heading = arc_hdg;
        out_cmd->t_speed   = LP_TURN_SPEED_MM_S;
        return true;
    }

    /* ── PURE_PURSUIT ────────────────────────────────────────────────────── */

    /* Live-LiDAR scan checks */
    if (live_scan && live_scan->count > 0) {
        int close_hits  = lp_count_path_hits(live_scan, LP_STOP_DIST_MM,   LP_CORRIDOR_HALF_WIDTH_MM);
        int detect_hits = lp_count_path_hits(live_scan, LP_DETECT_DIST_MM, LP_CORRIDOR_HALF_WIDTH_MM);

        if (close_hits >= LP_OBSTACLE_MIN_PTS) {
            printf("[LP] obstacle < %.0f mm — WAIT_CLEAR\n", (double)LP_STOP_DIST_MM);
            s.mode         = LP_MODE_WAIT_CLEAR;
            s.stall_cycles = 0;
            lp_request_replan("obstacle inside stop distance");
            return true;
        }
        if (detect_hits >= LP_OBSTACLE_MIN_PTS)
            lp_request_replan("obstacle at detection range");
    }

    /* Stall detection is handled by local_planner_odom_tick() at 100 Hz. */

    if (!global_path || global_path->length == 0)
        return true;

    *out_cmd = lp_pure_pursuit(global_path, raw_pose);
    return true;
}

void local_planner_odom_tick(float linear_disp_mm)
{
    if (!s.enabled) return;

    float disp = fabsf(linear_disp_mm);

    switch (s.mode) {

    case LP_MODE_PURE_PURSUIT:
        /* Stall: path loaded and barely moving for LP_STALL_CYCLES_ODOM ticks. */
        if (s.path_was_active && disp < LP_STALL_DISP_MM_ODOM) {
            if (++s.stall_cycles >= LP_STALL_CYCLES_ODOM) {
                printf("[LP] stall (%u odom ticks, disp=%.2f mm) — reversing 1 m\n",
                       LP_STALL_CYCLES_ODOM, (double)disp);
                s.mode              = LP_MODE_REVERSING;
                s.stall_cycles      = 0;
                s.recovery_accum_mm = 0.0f;
                s.stall_count++;
                s.reverse_heading   = lp_norm(s.last_pose.theta + LP_PI);
            }
        } else {
            s.stall_cycles = 0;
        }
        break;

    case LP_MODE_REVERSING:
        s.recovery_accum_mm += disp;
        if (s.recovery_accum_mm >= LP_REVERSE_TARGET_MM) {
            printf("[LP] reverse %.0f mm done — stall turn\n",
                   (double)s.recovery_accum_mm);
            s.mode              = LP_MODE_STALL_TURN;
            s.recovery_accum_mm = 0.0f;
            s.turn_steer_rad    = (s.stall_count % 2u == 0u)
                                  ? +LP_MAX_STEER_RAD : -LP_MAX_STEER_RAD;
        }
        break;

    case LP_MODE_STALL_TURN:
        s.recovery_accum_mm += disp;
        if (s.recovery_accum_mm >= LP_TURN_FWD_MM) {
            printf("[LP] stall turn done — WAIT_CLEAR, replanning\n");
            s.mode              = LP_MODE_WAIT_CLEAR;
            s.recovery_accum_mm = 0.0f;
            lp_request_replan("stall recovery complete");
        }
        break;

    default:
        break;
    }
}
