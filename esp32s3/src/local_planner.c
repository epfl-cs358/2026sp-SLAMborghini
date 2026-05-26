/**
 * local_planner.c
 * Module: Local planner — 100 ms reactive navigation layer.
 * Board: ESP32-S3
 *
 * Runs on top of the Hybrid A* global path.
 * Call local_planner_update() every 100 ms from the navigation task.
 *
 * No dynamic allocation.  All state lives in the single static lp_state_t s.
 */

#include "local_planner.h"
#include "command_gen.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stddef.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define LP_PI   3.14159265f
#define LP_2PI  6.28318530f

/* ════════════════════════════════════════════════════════════════════════════
 * Physical / tuning constants
 * ════════════════════════════════════════════════════════════════════════════ */

/* Robot geometry */
/* Half-diagonal of car bounding box: sqrt((295/2)^2 + (394/2)^2) = 246 mm */
#define LP_ROBOT_RADIUS_DEFAULT_MM  246.0f
#define LP_WHEELBASE_MM             260.0f
#define LP_MAX_STEER_RAD            1.134f   /* 65° — physical steering limit */
#define LP_BASE_SPEED_MM_S          150.0f
#define LP_REVERSE_SPEED_MM_S        80.0f

/* Pose filter */
#define LP_THETA_ALPHA               0.3f    /* weight on new raw sample (EMA) */
#define LP_SIGMA_CLAMP_MAX           0.05f   /* max sigma change per cycle (rad) */
#define LP_SIGMA_RECOVER_THRESH      0.25f   /* sigma above this → RECOVER mode */
#define LP_THETA_HIST_MAX            10      /* ring buffer size (used in RECOVER) */
#define LP_THETA_HIST_NORMAL          5      /* samples in normal mode */

/* Local window */
#define LP_FWD_ANGLES                 5
#define LP_FWD_DISTANCES              3
#define LP_FWD_TOTAL                 15      /* 5 × 3 forward cells */
#define LP_FOOT_CELLS                 4
#define LP_FWD_NEAR_MM              150.0f
#define LP_FWD_MID_MM               200.0f
#define LP_FWD_FAR_MM               300.0f  /* 30 cm — max detection distance */
#define LP_CLUSTER_THRESH             3      /* 3 occupied cells in the forward arc
                                             * triggers STOPPED; a flat wall at 300 mm
                                             * hits 3+ beams (0°, ±22.5°) → triggers.
                                             * Corridor side-walls at ±45° give ≤2
                                             * occupied cells → no false trigger. */

/* Reactive */
#define LP_CANDS                      5
#define LP_NARROW_MARGIN_MM          50.0f
#define LP_FORBIDDEN_HALF_MAX        0.611f  /* 35° cap on forbidden sector */
#define LP_FORBIDDEN_BUFFER_RAD      0.0f    /* rollout is the real clearance check */
#define LP_ROLLOUT_TOTAL_MM         550.0f
#define LP_ROLLOUT_NEAR_MM          200.0f
#define LP_ROLLOUT_NEAR_STEP_MM      20.0f   /* 2 cm */
#define LP_ROLLOUT_FAR_STEP_MM       80.0f   /* 8 cm */
#define LP_ROLLOUT_MAX_STEPS         20
#define LP_OCCUPIED_HARD_THRESH       0      /* any positive log-odds → hard collision */
#define LP_DIFFICULT_CELL_COST        0.45f  /* (unused with threshold=0, kept for reference) */
#define LP_UNKNOWN_CELL_COST          0.35f  /* log-odds == 0: penalise unknown near walls */
#define LP_MAX_DIFFICULT_PENALTY      1.50f  /* cumulative rollout penalty → reject */
#define LP_PATH_CTE_NORM_MM         500.0f
#define LP_HEADING_ERR_NORM_RAD       1.571f /* π/2 */
#define LP_SCORE_W_PATH               0.60f
#define LP_SCORE_W_JERK               0.40f
#define LP_JERK_NORM_RAD              0.873f /* 50° */
#define LP_SCORE_HYSTERESIS           0.10f
#define LP_SCORE_TIE_THRESH           0.10f
#define LP_COLLISION_SCORE            1.0e9f
#define LP_REACTIVE_FWD_MM          100.0f   /* command step distance in reactive */

/* Stall detection */
#define LP_STALL_DISP_MM              5.0f  /* <5 mm per 100 ms cycle = stalled */
#define LP_STALL_CYCLES               8     /* 8 consecutive stall cycles (~800 ms) → force ESCAPE */

/* Blockage & ESCAPE */
#define LP_BLOCKED_THRESH            10
#define LP_ESCAPE_TIMEOUT_CYCLES     30      /* 3 s @ 100 ms */
#define LP_ESCAPE_ROT_STEP_RAD        0.262f /* 15° per cycle */
#define LP_ESCAPE_FULL_ROT_RAD        6.283f /* 360° — full rotation = failure */
#define LP_ESCAPE_REVERSE_CYCLES      4      /* 4 cycles of reverse for FP escape */
#define LP_ESCAPE_REVERSE_MM         60.0f   /* commanded backward offset */
#define LP_ESCAPE_MIN_ROT_RAD         1.571f /* must rotate 90° before testing reactive */

/* STOPPED: stop and wait when obstacle cluster detected (dynamic obstacle assumed) */
#define LP_WAIT_CLEAR_MAX_CYCLES     30      /* 3 s in STOPPED before treating as static → replan */
#define LP_WAIT_CLEAR_CLEAR_STREAK    2      /* consecutive clear cycles to resume from STOPPED */
/* WAIT_CLEAR: stop after stall escape, wait for path, then replan */
#define LP_WAIT_CLEAR_TIMEOUT_CYCLES 50      /* 5 s max wait before forced replan */
/* Stall escape: 1 m reverse then 50° reorient */
#define LP_ESCAPE_STALL_REV_CYCLES   17      /* 17 × 60 mm ≈ 1020 mm ≈ 1 m */
#define LP_ESCAPE_STEER_MAX_RAD       0.873f /* 50° cap on stall-escape rotation */

/* Speed scaling */
#define LP_SPEED_SCALE_UNKNOWN        0.60f
#define LP_SPEED_SCALE_NARROW         0.70f
#define LP_SPEED_SCALE_RECOVER        0.50f
#define LP_UNK_SPEED_THRESH           5      /* unknown fwd cells to trigger scaling */

/* RECOVER */
#define LP_RECOVER_STABLE_THRESH      5      /* stable cycles below sigma to exit */

/* Waypoint tracking */
#define LP_WP_REACH_MM              200.0f

/* ════════════════════════════════════════════════════════════════════════════
 * Internal types
 * ════════════════════════════════════════════════════════════════════════════ */

typedef struct {
    bool    footprint_occupied;
    uint8_t occ_count;
    uint8_t unk_count;
    uint8_t free_count;
    float   occ_bearings[LP_FWD_TOTAL]; /* robot-local frame, radians */
    uint8_t occ_bearing_count;
    float   free_width;                 /* lateral clearance estimate (mm) */
} lp_window_t;

typedef struct {
    float score;
    float min_clearance;
    bool  valid;
    float steer_rad;
} lp_cand_t;

typedef struct {
    float     robot_radius;
    float     inflate_radius;
    float     sigma;

    /* Pose filter */
    float     theta_filtered;
    float     theta_history[LP_THETA_HIST_MAX]; /* raw sample ring buffer */
    uint8_t   theta_hist_idx;
    uint8_t   theta_hist_count;

    /* State machine */
    lp_mode_t mode;
    uint8_t   current_wp_idx;

    /* Reactive hysteresis */
    float     prev_steer;
    float     prev_score;

    /* ESCAPE */
    uint8_t   escape_phase;      /* 0 = reversing, 1 = rotating */
    uint8_t   escape_rev_cycles;
    float     escape_heading;    /* heading at ESCAPE entry */
    float     escape_rot_acc;    /* total rotation accumulated (rad) */
    uint32_t  escape_start_cyc;

    /* RECOVER */
    uint8_t   recover_stable;

    /* Blockage counter */
    uint8_t   blocked_cycles;

    /* Footprint hysteresis — require 2 consecutive hits before entering ESCAPE */
    uint8_t   fp_occ_streak;

    /* Stall detection */
    float     prev_x, prev_y;   /* pose snapshot from previous cycle */
    float     prev_cmd_speed;   /* commanded t_speed from previous cycle (mm/s) */
    uint8_t   stall_cycles;     /* consecutive cycles: path active, no displacement */

    /* Path rejoin blend — TODO: activate when Pure Pursuit is implemented */
    uint8_t   blend_cycles;

    int8_t    committed_side;   /* 0=none, +1=right steer, -1=left steer */
    uint8_t   pp_stable_count;  /* consecutive PP cycles since last obstacle */
    bool      replan_requested;
    uint32_t  cycle_count;

    /* STOPPED / WAIT_CLEAR */
    uint8_t   wait_clear_cycles;    /* cycles spent in STOPPED or WAIT_CLEAR */
    uint8_t   stopped_clear_streak; /* consecutive clear cycles in STOPPED (hysteresis) */
    bool      escape_is_stall;      /* true = stall escape (1 m back + 50°); false = FP escape */

    /* Stall-loop obstacle injection */
    uint8_t   stall_escape_count;        /* incremented each time a stall triggers ESCAPE */
    bool      inject_obstacle_requested;
    float     inject_x, inject_y;       /* world coords for virtual obstacle */
} lp_state_t;

static lp_state_t s; /* zero-initialised by BSS */


/* ════════════════════════════════════════════════════════════════════════════
 * Angle helpers
 * ════════════════════════════════════════════════════════════════════════════ */

static float lp_norm(float a)
{
    if (a >  LP_PI) return a - LP_2PI;
    if (a < -LP_PI) return a + LP_2PI;
    return a;
}


/* ════════════════════════════════════════════════════════════════════════════
 * Map query helpers
 * (qt_query_const is static in quadtree_map.c — accessed via the public header)
 * ════════════════════════════════════════════════════════════════════════════ */

static int8_t lp_cell(const quadtree_map_t *map, float x, float y)
{
    return qt_query_const(map, x, y);
}

static bool lp_occupied(const quadtree_map_t *map, float x, float y)
{
    return qt_query_const(map, x, y) > 0;
}


/* ════════════════════════════════════════════════════════════════════════════
 * Step 1 — Pose filter
 * theta_f = 0.7 * theta_prev + 0.3 * theta_raw  (with wrap-safe delta)
 * Variance computed from raw ring buffer; sigma change clamped per cycle.
 * ════════════════════════════════════════════════════════════════════════════ */

static float lp_filter_theta(float raw)
{
    float delta = lp_norm(raw - s.theta_filtered);
    s.theta_filtered = lp_norm(s.theta_filtered + LP_THETA_ALPHA * delta);

    s.theta_history[s.theta_hist_idx] = raw;
    s.theta_hist_idx = (s.theta_hist_idx + 1u) % LP_THETA_HIST_MAX;
    if (s.theta_hist_count < LP_THETA_HIST_MAX)
        s.theta_hist_count++;

    return s.theta_filtered;
}

static float lp_compute_sigma(void)
{
    uint8_t n = (s.mode == LP_MODE_RECOVER) ? LP_THETA_HIST_MAX : LP_THETA_HIST_NORMAL;
    if (n > s.theta_hist_count) n = s.theta_hist_count;
    if (n < 2) return s.sigma;

    /* Circular mean of raw samples */
    float sin_s = 0.0f, cos_s = 0.0f;
    for (uint8_t i = 0; i < n; i++) {
        uint8_t idx = (uint8_t)((s.theta_hist_idx + LP_THETA_HIST_MAX - n + i)
                                 % LP_THETA_HIST_MAX);
        sin_s += sinf(s.theta_history[idx]);
        cos_s += cosf(s.theta_history[idx]);
    }
    float mean = atan2f(sin_s / n, cos_s / n);

    float var = 0.0f;
    for (uint8_t i = 0; i < n; i++) {
        uint8_t idx = (uint8_t)((s.theta_hist_idx + LP_THETA_HIST_MAX - n + i)
                                 % LP_THETA_HIST_MAX);
        float d = lp_norm(s.theta_history[idx] - mean);
        var += d * d;
    }
    float new_sig = sqrtf(var / n);

    /* Clamp sigma change to suppress spike from single bad scan */
    float diff = new_sig - s.sigma;
    if (diff >  LP_SIGMA_CLAMP_MAX) new_sig = s.sigma + LP_SIGMA_CLAMP_MAX;
    if (diff < -LP_SIGMA_CLAMP_MAX) new_sig = s.sigma - LP_SIGMA_CLAMP_MAX;

    s.sigma = new_sig;
    return new_sig;
}


/* ════════════════════════════════════════════════════════════════════════════
 * Step 9 — RECOVER mode transitions
 * ════════════════════════════════════════════════════════════════════════════ */

static void lp_update_recover(float sigma)
{
    if (sigma > LP_SIGMA_RECOVER_THRESH) {
        if (s.mode != LP_MODE_ESCAPE &&
            s.mode != LP_MODE_STOPPED &&
            s.mode != LP_MODE_WAIT_CLEAR)
            s.mode = LP_MODE_RECOVER;
        s.recover_stable = 0;
    } else if (s.mode == LP_MODE_RECOVER) {
        s.recover_stable++;
        if (s.recover_stable >= LP_RECOVER_STABLE_THRESH)
            s.mode = LP_MODE_PURE_PURSUIT;
    }
}


/* ════════════════════════════════════════════════════════════════════════════
 * Step 3 — Local window query (~19 fixed cells, no full-path scan)
 * ════════════════════════════════════════════════════════════════════════════ */

static void lp_query_window(const quadtree_map_t *map,
                            float rx, float ry, float rtheta,
                            float inflate_r,
                            lp_window_t *w)
{
    /* A. Footprint cells at 4 cardinal directions */
    w->footprint_occupied = false;
    float fp_r = inflate_r * 0.65f;
    const float fp_off[LP_FOOT_CELLS] = { 0.0f, 1.5708f, 3.1416f, 4.7124f };
    for (int i = 0; i < LP_FOOT_CELLS; i++) {
        float a = rtheta + fp_off[i];
        if (lp_occupied(map, rx + fp_r * cosf(a), ry + fp_r * sinf(a))) {
            w->footprint_occupied = true;
            return;
        }
    }

    /* B. Forward arc: 5 angles × 4 distances = 20 cells */
    w->occ_count         = 0;
    w->unk_count         = 0;
    w->free_count        = 0;
    w->occ_bearing_count = 0;

    static const float ang[LP_FWD_ANGLES] = {
        -0.7854f, -0.3927f, 0.0f, 0.3927f, 0.7854f  /* ±45°, ±22.5°, 0° */
    };
    static const float dist[LP_FWD_DISTANCES] = {
        LP_FWD_NEAR_MM, LP_FWD_MID_MM, LP_FWD_FAR_MM
    };

    for (int ai = 0; ai < LP_FWD_ANGLES; ai++) {
        float a = rtheta + ang[ai];
        for (int di = 0; di < LP_FWD_DISTANCES; di++) {
            float cx = rx + dist[di] * cosf(a);
            float cy = ry + dist[di] * sinf(a);
            int8_t v = lp_cell(map, cx, cy);
            if (v > 0) {
                w->occ_count++;
                if (w->occ_bearing_count < LP_FWD_TOTAL) {
                    w->occ_bearings[w->occ_bearing_count++] =
                        lp_norm(atan2f(cy - ry, cx - rx) - rtheta);
                }
            } else if (v == 0) {
                w->unk_count++;
            } else {
                w->free_count++;
            }
        }
    }

    /* Lateral free_width at LP_FWD_NEAR_MM forward (used for narrow-mode check) */
    float fw_x = rx + LP_FWD_NEAR_MM * cosf(rtheta);
    float fw_y = ry + LP_FWD_NEAR_MM * sinf(rtheta);
    float lx   = -sinf(rtheta);   /* unit left-perpendicular */
    float ly   =  cosf(rtheta);
    float lw = 0.0f, rw = 0.0f;

    for (float d = 50.0f; d <= 400.0f; d += 50.0f) {
        if (lp_occupied(map, fw_x + d * lx, fw_y + d * ly)) break;
        lw = d;
    }
    for (float d = 50.0f; d <= 400.0f; d += 50.0f) {
        if (lp_occupied(map, fw_x - d * lx, fw_y - d * ly)) break;
        rw = d;
    }
    w->free_width = lw + rw;
}


/* ════════════════════════════════════════════════════════════════════════════
 * Path error helpers (cross-track + heading)
 * ════════════════════════════════════════════════════════════════════════════ */

static float lp_path_cte(const path_t *path,
                         float x, float y, float theta,
                         float *out_head_err)
{
    if (!path || path->length == 0) {
        if (out_head_err) *out_head_err = 0.0f;
        return 0.0f;
    }

    int   best  = 0;
    float bestd = 1.0e9f;
    for (int i = 0; i < path->length; i++) {
        float dx = path->waypoints[i].x - x;
        float dy = path->waypoints[i].y - y;
        float d  = sqrtf(dx * dx + dy * dy);
        if (d < bestd) { bestd = d; best = i; }
    }

    float cte       = bestd;
    float seg_theta = path->waypoints[best].theta;

    if (best + 1 < path->length) {
        float ax = path->waypoints[best].x,      ay = path->waypoints[best].y;
        float bx = path->waypoints[best + 1].x,  by = path->waypoints[best + 1].y;
        float sdx = bx - ax, sdy = by - ay;
        float len = sqrtf(sdx * sdx + sdy * sdy);
        if (len > 1.0f) {
            cte       = ((x - ax) * sdy - (y - ay) * sdx) / len;
            seg_theta = atan2f(sdy, sdx);
        }
    }

    if (out_head_err)
        *out_head_err = lp_norm(theta - seg_theta);
    return cte;
}


/* ════════════════════════════════════════════════════════════════════════════
 * Step 5 — REACTIVE mode
 * ════════════════════════════════════════════════════════════════════════════ */

/* Hard filter: true → reject this candidate before rollout */
static bool lp_hard_reject(float steer_rad, const lp_window_t *w)
{
    /* Kinematic feasibility */
    if (fabsf(steer_rad) > LP_MAX_STEER_RAD)
        return true;

    /* Forbidden angular sector built from occupied-cell bearings */
    if (w->occ_bearing_count > 0) {
        float mn = w->occ_bearings[0], mx = w->occ_bearings[0];
        for (uint8_t i = 1; i < w->occ_bearing_count; i++) {
            if (w->occ_bearings[i] < mn) mn = w->occ_bearings[i];
            if (w->occ_bearings[i] > mx) mx = w->occ_bearings[i];
        }
        float center = 0.5f * (mn + mx);
        float half   = 0.5f * (mx - mn) + LP_FORBIDDEN_BUFFER_RAD;
        if (half > LP_FORBIDDEN_HALF_MAX)
            half = LP_FORBIDDEN_HALF_MAX;
        if (fabsf(lp_norm(steer_rad - center)) < half)
            return true;
    }

    return false;
}

/*
 * Rollout: simulate bicycle model from (rx, ry, rtheta) with constant steer_rad.
 * Variable density: LP_ROLLOUT_NEAR_STEP_MM near, LP_ROLLOUT_FAR_STEP_MM far.
 * Returns LP_COLLISION_SCORE on collision / excessive difficult-zone penalty.
 */
static lp_cand_t lp_rollout(const quadtree_map_t *map,
                             float rx, float ry, float rtheta,
                             float steer_rad,
                             const path_t *path)
{
    lp_cand_t r;
    r.valid         = false;
    r.score         = LP_COLLISION_SCORE;
    r.min_clearance = 0.0f;
    r.steer_rad     = steer_rad;

    float x = rx, y = ry, heading = rtheta;
    float dist = 0.0f;
    float diff_penalty = 0.0f;
    float min_cl = 1.0e9f;
    int   steps  = 0;

    while (dist < LP_ROLLOUT_TOTAL_MM && steps < LP_ROLLOUT_MAX_STEPS) {
        float step = (dist < LP_ROLLOUT_NEAR_MM)
                     ? LP_ROLLOUT_NEAR_STEP_MM
                     : LP_ROLLOUT_FAR_STEP_MM;

        /* Bicycle model Euler step */
        heading = lp_norm(heading + step * tanf(steer_rad) / LP_WHEELBASE_MM);
        x      += step * cosf(heading);
        y      += step * sinf(heading);
        dist   += step;
        steps++;

        int8_t v = lp_cell(map, x, y);

        if (v > LP_OCCUPIED_HARD_THRESH) {
            return r; /* hard collision */
        }

        if (v > 0) {
            diff_penalty += LP_DIFFICULT_CELL_COST; /* difficult zone */
        } else if (v == 0) {
            diff_penalty += LP_UNKNOWN_CELL_COST;   /* unknown: medium cost */
        }

        if (diff_penalty > LP_MAX_DIFFICULT_PENALTY)
            return r; /* reject after too many difficult cells */

        /* Track clearance: negative log-odds = free; more negative = clearer */
        float cl = (float)(-v);
        if (cl < min_cl) min_cl = cl;
    }

    /* Score at rollout endpoint */
    float head_err;
    float cte = lp_path_cte(path, x, y, heading, &head_err);

    float norm_path = fabsf(cte)      / LP_PATH_CTE_NORM_MM      * 0.5f
                    + fabsf(head_err) / LP_HEADING_ERR_NORM_RAD   * 0.5f;
    if (norm_path > 1.0f) norm_path = 1.0f;

    float jerk = fabsf(steer_rad - s.prev_steer) / LP_JERK_NORM_RAD;
    if (jerk > 1.0f) jerk = 1.0f;

    r.score         = LP_SCORE_W_PATH * norm_path + LP_SCORE_W_JERK * jerk;
    r.min_clearance = min_cl;
    r.valid         = true;
    return r;
}

/*
 * Run REACTIVE evaluation with up to LP_CANDS steering candidates.
 * Returns true and fills out_cmd on success.
 */
static bool lp_reactive(const quadtree_map_t *map,
                        float rx, float ry, float rtheta,
                        float inflate_r,
                        const lp_window_t *w,
                        const path_t *path,
                        control_frame_t *out_cmd)
{
    bool narrow = (w->free_width < 2.0f * inflate_r + LP_NARROW_MARGIN_MM);

    static const float wide_deg[LP_CANDS]   = { -25.0f, -12.0f, 0.0f, 12.0f, 25.0f };
    static const float narrow_deg[LP_CANDS] = { -18.0f,  -9.0f, 0.0f,  9.0f, 18.0f };
    const float *deg_tbl = narrow ? narrow_deg : wide_deg;

    lp_cand_t best;
    best.valid         = false;
    best.score         = LP_COLLISION_SCORE;
    best.steer_rad     = 0.0f;
    best.min_clearance = 0.0f;

    for (int ci = 0; ci < LP_CANDS; ci++) {
        float steer_rad = deg_tbl[ci] * (LP_PI / 180.0f);

        if (lp_hard_reject(steer_rad, w))
            continue;

        lp_cand_t res = lp_rollout(map, rx, ry, rtheta, steer_rad, path);
        if (!res.valid)
            continue;

        /* Side commitment: penalise switching sides mid-avoidance */
        if (s.committed_side != 0 && fabsf(steer_rad) > 0.001f) {
            int8_t side = (steer_rad > 0.0f) ? 1 : -1;
            if (side != s.committed_side)
                res.score += 0.35f;
        }

        bool better = (res.score < best.score - LP_SCORE_HYSTERESIS);

        /* Clearance tie-breaker when scores are close */
        if (!better && best.valid
            && fabsf(res.score - best.score) < LP_SCORE_TIE_THRESH
            && res.min_clearance > best.min_clearance) {
            better = true;
        }

        if (!best.valid || better)
            best = res;
    }

    if (!best.valid)
        return false;

    /* Hysteresis against previous cycle's command: keep old steer if not clearly better */
    float steer     = best.steer_rad;
    bool  kept_prev = false;
    if (s.prev_score < LP_COLLISION_SCORE
        && best.score >= s.prev_score - LP_SCORE_HYSTERESIS) {
        steer     = s.prev_steer;
        kept_prev = true;
    }

    float new_heading  = lp_norm(rtheta + steer);
    out_cmd->tx        = LP_REACTIVE_FWD_MM * cosf(new_heading);
    out_cmd->ty        = LP_REACTIVE_FWD_MM * sinf(new_heading);
    out_cmd->t_heading = new_heading;
    out_cmd->t_speed   = LP_BASE_SPEED_MM_S;

    s.prev_steer = steer;
    if (!kept_prev)
        s.prev_score = best.score;
    if (s.committed_side == 0 && fabsf(steer) > 0.001f)
        s.committed_side = (steer > 0.0f) ? 1 : -1;
    return true;
}


/* ════════════════════════════════════════════════════════════════════════════
 * Step 6 — ESCAPE mode (reverse then rotate)
 * Returns true while recovery is in progress, false on unrecoverable failure.
 * ════════════════════════════════════════════════════════════════════════════ */

static bool lp_escape(const quadtree_map_t *map,
                      float rx, float ry, float rtheta,
                      float inflate_r,
                      const path_t *path,
                      control_frame_t *out_cmd)
{
    /* Timeout: stall escape gets extra time for the longer reverse */
    uint32_t timeout = s.escape_is_stall
                     ? (uint32_t)(LP_ESCAPE_STALL_REV_CYCLES + LP_ESCAPE_TIMEOUT_CYCLES)
                     : LP_ESCAPE_TIMEOUT_CYCLES;
    if (s.cycle_count - s.escape_start_cyc >= timeout) {
        if (!s.escape_is_stall) s.replan_requested = true;
        return false; /* caller decides next mode */
    }

    /* Full rotation exhausted — only applies to FP escape (stall caps at 50°) */
    if (!s.escape_is_stall && s.escape_rot_acc >= LP_ESCAPE_FULL_ROT_RAD) {
        s.replan_requested = true;
        return false;
    }

    /* Phase 0: reverse.  Stall escape reverses ~1 m; FP escape reverses ~240 mm. */
    uint8_t rev_target = s.escape_is_stall ? LP_ESCAPE_STALL_REV_CYCLES
                                           : LP_ESCAPE_REVERSE_CYCLES;
    if (s.escape_phase == 0) {
        float rear_a = rtheta + LP_PI; /* cosf/sinf are periodic, no wrap needed */
        float rear_r = inflate_r * 0.65f;
        bool rear_blocked = lp_occupied(map, rx + rear_r * cosf(rear_a),
                                             ry + rear_r * sinf(rear_a));
        if (!rear_blocked) {
            out_cmd->tx        = LP_ESCAPE_REVERSE_MM * cosf(rear_a);
            out_cmd->ty        = LP_ESCAPE_REVERSE_MM * sinf(rear_a);
            out_cmd->t_heading = rear_a;
            out_cmd->t_speed   = LP_REVERSE_SPEED_MM_S;
            s.escape_rev_cycles++;
            if (s.escape_rev_cycles >= rev_target)
                s.escape_phase = 1;
            return true;
        }
        /* rear wall detected — skip reversal, go straight to rotation */
        s.escape_phase = 1;
        /* fall through to Phase 1 */
    }

    /* Phase 1: arc forward toward the escape target heading, 15°/cycle.
     * Stall escape: cap at 50° then stop (go to WAIT_CLEAR).
     * FP escape:    keep arcing up to 360° until a clear direction is found. */
    s.escape_rot_acc += LP_ESCAPE_ROT_STEP_RAD;
    float test_heading = lp_norm(s.escape_heading + s.escape_rot_acc);

    float steer_arc = lp_norm(test_heading - rtheta);
    if (steer_arc >  LP_MAX_STEER_RAD) steer_arc =  LP_MAX_STEER_RAD;
    if (steer_arc < -LP_MAX_STEER_RAD) steer_arc = -LP_MAX_STEER_RAD;
    float arc_hdg      = lp_norm(rtheta + steer_arc);
    out_cmd->tx        = LP_REACTIVE_FWD_MM * cosf(arc_hdg);
    out_cmd->ty        = LP_REACTIVE_FWD_MM * sinf(arc_hdg);
    out_cmd->t_heading = arc_hdg;
    out_cmd->t_speed   = LP_BASE_SPEED_MM_S * 0.5f;

    /* Stall escape: stop after rotating 50° toward safe area → WAIT_CLEAR */
    if (s.escape_is_stall && s.escape_rot_acc >= LP_ESCAPE_STEER_MAX_RAD) {
        printf("[LP] stall escape: 50° reorient done — entering WAIT_CLEAR\n");
        return false; /* caller sets mode = WAIT_CLEAR */
    }

    /* Test whether the test direction is clear yet */
    lp_window_t test_w;
    lp_query_window(map, rx, ry, test_heading, inflate_r, &test_w);

    if (!test_w.footprint_occupied && test_w.occ_count < LP_CLUSTER_THRESH &&
            s.escape_rot_acc >= LP_ESCAPE_MIN_ROT_RAD) {
        if (s.escape_is_stall) {
            /* Stall: found safe direction before 50° — stop and wait for replan */
            printf("[LP] stall escape: clear direction found — entering WAIT_CLEAR\n");
            return false; /* caller sets mode = WAIT_CLEAR */
        }
        /* FP escape: exit with a reactive command from current heading */
        lp_window_t cur_w;
        lp_query_window(map, rx, ry, rtheta, inflate_r, &cur_w);
        control_frame_t test_cmd;
        if (!cur_w.footprint_occupied &&
            lp_reactive(map, rx, ry, rtheta, inflate_r, &cur_w, path, &test_cmd)) {
            *out_cmd       = test_cmd;
            s.mode         = LP_MODE_PURE_PURSUIT;
            s.blend_cycles = 3;
            return true;
        }
    }

    return true; /* keep arcing */
}


/* ════════════════════════════════════════════════════════════════════════════
 * STOPPED mode — stop in place and wait for obstacle to clear (dynamic object).
 * After LP_WAIT_CLEAR_MAX_CYCLES with obstacle still present → treat as static → replan.
 * ════════════════════════════════════════════════════════════════════════════ */

static bool lp_handle_stopped(const lp_window_t *w, float theta_f,
                               control_frame_t *out_cmd)
{
    out_cmd->tx        = 0.0f;
    out_cmd->ty        = 0.0f;
    out_cmd->t_heading = theta_f;
    out_cmd->t_speed   = 0.0f;

    s.wait_clear_cycles++;

    if (!w->footprint_occupied && w->occ_count < LP_CLUSTER_THRESH) {
        s.stopped_clear_streak++;
        if (s.stopped_clear_streak >= LP_WAIT_CLEAR_CLEAR_STREAK) {
            /* Dynamic obstacle cleared — resume current path, no replan */
            s.mode                 = LP_MODE_PURE_PURSUIT;
            s.wait_clear_cycles    = 0;
            s.stopped_clear_streak = 0;
            printf("[LP] STOPPED: obstacle cleared — resuming PP\n");
        }
    } else {
        s.stopped_clear_streak = 0;
        if (s.wait_clear_cycles >= LP_WAIT_CLEAR_MAX_CYCLES) {
            /* Still blocked after 3 s — treat as static, request replan */
            s.replan_requested  = true;
            s.mode              = LP_MODE_PURE_PURSUIT;
            s.wait_clear_cycles = 0;
            printf("[LP] STOPPED: timeout — treating obstacle as static, replanning\n");
        }
    }

    return true;
}


/* ════════════════════════════════════════════════════════════════════════════
 * WAIT_CLEAR mode — stop after stall escape, wait for forward path to clear.
 * When clear: trigger replan and return to Pure Pursuit.
 * ════════════════════════════════════════════════════════════════════════════ */

static bool lp_handle_wait_clear(const lp_window_t *w, float theta_f,
                                  control_frame_t *out_cmd)
{
    out_cmd->tx        = 0.0f;
    out_cmd->ty        = 0.0f;
    out_cmd->t_heading = theta_f;
    out_cmd->t_speed   = 0.0f;

    s.wait_clear_cycles++;

    if (!w->footprint_occupied && w->occ_count < LP_CLUSTER_THRESH) {
        /* Path clear — replan (stall may have caused odometry drift) */
        s.replan_requested  = true;
        s.mode              = LP_MODE_PURE_PURSUIT;
        s.wait_clear_cycles = 0;
        printf("[LP] WAIT_CLEAR: path clear — replanning\n");
    } else if (s.wait_clear_cycles >= LP_WAIT_CLEAR_TIMEOUT_CYCLES) {
        /* Force replan after 5 s to avoid deadlock */
        s.replan_requested  = true;
        s.mode              = LP_MODE_PURE_PURSUIT;
        s.wait_clear_cycles = 0;
        printf("[LP] WAIT_CLEAR: timeout — forcing replan\n");
    }

    return true;
}


/* ════════════════════════════════════════════════════════════════════════════
 * Step 4 — Pure Pursuit stub
 * TODO: implement full lookahead geometry when Pure Pursuit is ready.
 * For now: advance waypoint index and hand off to command_gen_compute().
 * ════════════════════════════════════════════════════════════════════════════ */

static control_frame_t lp_pure_pursuit_stub(const path_t *path,
                                            const pose_t *raw_pose)
{
    control_frame_t cmd;
    memset(&cmd, 0, sizeof(cmd));

    if (!path || path->length == 0)
        return cmd;

    /* Advance past waypoints already reached */
    while (s.current_wp_idx + 1u < path->length) {
        float dx = path->waypoints[s.current_wp_idx].x - raw_pose->x;
        float dy = path->waypoints[s.current_wp_idx].y - raw_pose->y;
        if (sqrtf(dx * dx + dy * dy) < LP_WP_REACH_MM)
            s.current_wp_idx++;
        else
            break;
    }
    if (s.current_wp_idx >= path->length)
        s.current_wp_idx = (uint8_t)(path->length - 1u);

    return command_gen_compute(raw_pose, &path->waypoints[s.current_wp_idx]);
}


/* ════════════════════════════════════════════════════════════════════════════
 * Step 10 — Speed scaling (applied after steering decision)
 * Rules stack multiplicatively.
 * ════════════════════════════════════════════════════════════════════════════ */

static void lp_scale_speed(control_frame_t *cmd,
                           const lp_window_t *w,
                           float inflate_r,
                           bool in_recover)
{
    float scale = 1.0f;

    if (w->unk_count >= LP_UNK_SPEED_THRESH)
        scale *= LP_SPEED_SCALE_UNKNOWN;

    if (w->free_width < 2.0f * inflate_r + LP_NARROW_MARGIN_MM)
        scale *= LP_SPEED_SCALE_NARROW;

    if (in_recover)
        scale *= LP_SPEED_SCALE_RECOVER;

    if (scale < 1.0f) {
        cmd->t_speed *= scale;
        if (cmd->t_speed < 0.0f) cmd->t_speed = 0.0f;
    }
}


/* ════════════════════════════════════════════════════════════════════════════
 * Public API
 * ════════════════════════════════════════════════════════════════════════════ */

void local_planner_init(float robot_radius_mm)
{
    memset(&s, 0, sizeof(s));
    s.robot_radius   = (robot_radius_mm > 10.0f) ? robot_radius_mm
                                                  : LP_ROBOT_RADIUS_DEFAULT_MM;
    s.inflate_radius = s.robot_radius;
    s.mode           = LP_MODE_PURE_PURSUIT;
    s.prev_score     = LP_COLLISION_SCORE;
}

void local_planner_reset_waypoint(void)
{
    s.current_wp_idx = 0;
    s.prev_steer     = 0.0f;
    s.prev_score     = LP_COLLISION_SCORE;
    s.stall_cycles   = 0;
}

lp_mode_t local_planner_get_mode(void)      { return s.mode; }
bool      local_planner_replan_needed(void) { return s.replan_requested; }
void      local_planner_clear_replan(void)  { s.replan_requested = false; }

bool local_planner_obstacle_inject_needed(float *x, float *y)
{
    if (!s.inject_obstacle_requested) return false;
    if (x) *x = s.inject_x;
    if (y) *y = s.inject_y;
    return true;
}

void local_planner_clear_obstacle_inject(void)
{
    s.inject_obstacle_requested = false;
}


bool local_planner_update(const quadtree_map_t *map,
                          const pose_t         *raw_pose,
                          const path_t         *global_path,
                          bool                  override_flag,
                          control_frame_t      *out_cmd)
{
    if (!map || !raw_pose || !out_cmd) return false;

    /* Step 2: Override — emergency layer has control, skip entirely */
    if (override_flag) return false;

    /* Stall detection — triggers ESCAPE when a path is active but the car has
     * not moved for LP_STALL_CYCLES consecutive cycles.  Catches two failure
     * modes: (1) LP stays in PURE_PURSUIT while physically jammed against a
     * wall whose close-range beams were filtered from the map; (2) lp_reactive
     * keeps returning a "valid" steering direction whose command the stuck car
     * cannot execute, preventing blocked_cycles from ever reaching LP_BLOCKED_THRESH. */
    {
        float dx   = raw_pose->x - s.prev_x;
        float dy   = raw_pose->y - s.prev_y;
        float disp = sqrtf(dx * dx + dy * dy);
        s.prev_x   = raw_pose->x;
        s.prev_y   = raw_pose->y;

        bool path_active = global_path && global_path->length > 0;
        if (path_active && s.mode != LP_MODE_ESCAPE && disp < LP_STALL_DISP_MM
                && s.prev_cmd_speed > 30.0f) {
            s.stall_cycles++;
        } else {
            s.stall_cycles = 0;
        }

        if (s.stall_cycles >= LP_STALL_CYCLES) {
            s.mode              = LP_MODE_ESCAPE;
            s.escape_is_stall   = true;
            s.stall_escape_count++;
            s.committed_side    = 0;
            s.pp_stable_count   = 0;
            s.escape_phase      = 0;
            s.escape_rev_cycles = 0;
            s.escape_heading    = raw_pose->theta;
            s.escape_rot_acc    = 0.0f;
            s.escape_start_cyc  = s.cycle_count;
            s.stall_cycles      = 0;
            /* On the 2nd consecutive stall the obstacle is below LiDAR height —
             * inject a virtual obstacle so A* routes around it next replan. */
            if (s.stall_escape_count >= 2) {
                s.inject_x = raw_pose->x + s.robot_radius * cosf(raw_pose->theta);
                s.inject_y = raw_pose->y + s.robot_radius * sinf(raw_pose->theta);
                s.inject_obstacle_requested = true;
                printf("[LP] stall #%u — injecting obstacle at (%.0f,%.0f)\n",
                       (unsigned)s.stall_escape_count,
                       (double)s.inject_x, (double)s.inject_y);
            }
            printf("[LP] stall detected (%u cycles) — ESCAPE: 1 m reverse + 50° reorient\n",
                   (unsigned)LP_STALL_CYCLES);
        }
    }

    /* Step 1: Pose filter */
    float theta_f   = lp_filter_theta(raw_pose->theta);
    float sigma     = lp_compute_sigma();
    float inflate_r = s.robot_radius + 1.5f * sigma * LP_FWD_NEAR_MM;
    s.inflate_radius = inflate_r;

    /* Step 9: RECOVER transitions (must not interrupt ESCAPE) */
    bool in_recover = (s.mode == LP_MODE_RECOVER);
    lp_update_recover(sigma);
    if (s.mode == LP_MODE_RECOVER) in_recover = true;

    /* Step 3: Query local window */
    lp_window_t w;
    lp_query_window(map, raw_pose->x, raw_pose->y, theta_f, inflate_r, &w);

    bool has_cluster = (w.occ_count >= LP_CLUSTER_THRESH);
    bool cmd_valid   = false;

    /* A. Footprint occupied → ESCAPE (wall is inside car body, must back up).
     * Require 2 consecutive fp_occ=1 cycles (hysteresis) to avoid triggering
     * ESCAPE on single-cell map noise from pose drift.
     * Do NOT use STOPPED here — it has no exit and the car freezes forever. */
    if (w.footprint_occupied) s.fp_occ_streak++;
    else                      s.fp_occ_streak = 0;

    if (w.footprint_occupied && s.fp_occ_streak >= 2) {
        if (s.mode != LP_MODE_ESCAPE) {
            s.mode              = LP_MODE_ESCAPE;
            s.escape_is_stall   = false;
            s.committed_side    = 0;
            s.pp_stable_count   = 0;
            s.escape_phase      = 0;
            s.escape_rev_cycles = 0;
            s.escape_heading    = theta_f;
            s.escape_rot_acc    = 0.0f;
            s.escape_start_cyc  = s.cycle_count;
        }
        cmd_valid = lp_escape(map, raw_pose->x, raw_pose->y, theta_f,
                              inflate_r, global_path, out_cmd);
        if (!cmd_valid) {
            /* FP escape (escape_is_stall = false here): replan already set in lp_escape */
            s.mode             = LP_MODE_PURE_PURSUIT;
            s.replan_requested = true;
            out_cmd->tx        = 0.0f;
            out_cmd->ty        = 0.0f;
            out_cmd->t_heading = theta_f;
            out_cmd->t_speed   = 0.0f;
            cmd_valid          = true;
        }
        printf("[LP] mode=%d phase=%d rtheta=%.0f° cmd_hdg=%.0f° spd=%.0f sigma=%.3f fp_occ=1 streak=%u occ=%u\n",
               (int)s.mode, (int)s.escape_phase,
               (double)(theta_f * 180.0f / LP_PI),
               (double)(out_cmd->t_heading * 180.0f / LP_PI),
               (double)out_cmd->t_speed,
               (double)s.sigma, (unsigned)s.fp_occ_streak, (unsigned)w.occ_count);
        s.prev_cmd_speed = out_cmd->t_speed;
        s.cycle_count++;
        return true;
    }

    /* ── State machine ──────────────────────────────────────────────────── */

    if (s.mode == LP_MODE_ESCAPE) {
        /* Continue ongoing ESCAPE (stall or FP — distinguised by escape_is_stall) */
        cmd_valid = lp_escape(map, raw_pose->x, raw_pose->y, theta_f,
                              inflate_r, global_path, out_cmd);
        if (!cmd_valid) {
            if (s.escape_is_stall) {
                /* Stall escape done: stop and wait for path to clear, then replan */
                s.mode              = LP_MODE_WAIT_CLEAR;
                s.wait_clear_cycles = 0;
            } else {
                /* FP escape failed: replan already requested in lp_escape */
                s.mode = LP_MODE_PURE_PURSUIT;
            }
            out_cmd->tx        = 0.0f;
            out_cmd->ty        = 0.0f;
            out_cmd->t_heading = theta_f;
            out_cmd->t_speed   = 0.0f;
            cmd_valid          = true;
        }

    } else if (s.mode == LP_MODE_STOPPED) {
        /* Waiting for dynamic obstacle to clear (or timeout → replan) */
        cmd_valid = lp_handle_stopped(&w, theta_f, out_cmd);

    } else if (s.mode == LP_MODE_WAIT_CLEAR) {
        /* Post-stall: waiting for path to clear before replanning */
        cmd_valid = lp_handle_wait_clear(&w, theta_f, out_cmd);

    } else if (!has_cluster) {
        /* Pure Pursuit (or RECOVER with reduced speed) */
        if (s.mode != LP_MODE_RECOVER)
            s.mode = LP_MODE_PURE_PURSUIT;
        s.pp_stable_count++;
        if (s.pp_stable_count >= 5)
            s.committed_side = 0;
        if (s.pp_stable_count >= 20)
            s.stall_escape_count = 0;
        *out_cmd  = lp_pure_pursuit_stub(global_path, raw_pose);
        cmd_valid = true;

    } else {
        /* Obstacle cluster in forward arc — stop and wait (assume dynamic object) */
        if (s.mode != LP_MODE_STOPPED) {
            s.mode                 = LP_MODE_STOPPED;
            s.wait_clear_cycles    = 0;
            s.stopped_clear_streak = 0;
            s.committed_side       = 0;
            s.pp_stable_count      = 0;
            printf("[LP] obstacle cluster occ=%u — entering STOPPED\n",
                   (unsigned)w.occ_count);
        }
        cmd_valid = lp_handle_stopped(&w, theta_f, out_cmd);
    }

    /* Step 10: Speed scaling */
    if (cmd_valid && out_cmd->t_speed > 0.0f)
        lp_scale_speed(out_cmd, &w, inflate_r, in_recover);

    if (cmd_valid && s.mode != LP_MODE_PURE_PURSUIT)
        printf("[LP] mode=%d phase=%d rtheta=%.0f° cmd_hdg=%.0f° spd=%.0f sigma=%.3f fp_occ=%d streak=%u occ=%u\n",
               (int)s.mode, (int)s.escape_phase,
               (double)(theta_f * 180.0f / LP_PI),
               (double)(out_cmd->t_heading * 180.0f / LP_PI),
               (double)out_cmd->t_speed,
               (double)s.sigma, (int)w.footprint_occupied,
               (unsigned)s.fp_occ_streak, (unsigned)w.occ_count);

    s.prev_cmd_speed = cmd_valid ? out_cmd->t_speed : 0.0f;
    s.cycle_count++;
    return cmd_valid;
}
