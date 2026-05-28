/**
 * pure_pursuit_controller.c  —  Wemos D1 R32
 *
 * Ring-buffer PP: waypoints arrive as streaming PATH_CHUNK packets from
 * ESP32-S3 and are stored in a fixed PP_RING_CAP circular buffer indexed by
 * their global path index (gi % PP_RING_CAP).  The pursuit_idx global counter
 * plays the role of last_target_index from the original flat-array version.
 */

#include "pure_pursuit_controller.h"

#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

static float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static float dist_sq_2d(float ax, float ay, float bx, float by) {
    float dx = ax - bx, dy = ay - by;
    return dx*dx + dy*dy;
}

/* Robot-frame transform: lateral right = X, forward = Y. */
static waypoint_t to_robot_frame(const pose_t *robot_pose, waypoint_t g) {
    float dx = g.x - robot_pose->x;
    float dy = g.y - robot_pose->y;
    waypoint_t r = g;
    r.x =  dx * sinf(robot_pose->theta) - dy * cosf(robot_pose->theta);
    r.y =  dx * cosf(robot_pose->theta) + dy * sinf(robot_pose->theta);
    return r;
}

/* ── Ring helpers ─────────────────────────────────────────────────────────── */
static inline waypoint_t *ring_at(pure_pursuit_controller_t *pp, uint16_t gi) {
    return &pp->ring[gi % PP_RING_CAP];
}
static inline uint16_t ring_tail(const pure_pursuit_controller_t *pp) {
    return pp->ring_head + pp->ring_count;
}

/* ── Public API ───────────────────────────────────────────────────────────── */

void pp_init(pure_pursuit_controller_t *pp) {
    if (!pp) return;
    memset(pp, 0, sizeof(*pp));
    pp->wheelbase_mm      = 258.0f;
    pp->lookahead_mm      = 300.0f;
    pp->fixed_speed_mm_s  = 200.0f;
    pp->kp                = 1.0f;
    pp->min_steering_rad  = -1.134f;
    pp->max_steering_rad  =  1.134f;
    pp->goal_tolerance_mm = 300.0f;
}

bool pp_append_chunk(pure_pursuit_controller_t *pp, const path_chunk_t *chunk) {
    if (!pp || !chunk || chunk->count == 0) return false;

    if (chunk->path_id != pp->path_id) {
        /* New plan: reset ring to global index 0.  Any chunk with
         * start_index != 0 will immediately trigger a NACK below. */
        pp->ring_head      = 0;
        pp->ring_count     = 0;
        pp->path_id        = chunk->path_id;
        pp->pursuit_idx    = 0;
        pp->final_received = false;
    }

    uint16_t expected = ring_tail(pp);  /* ring_head + ring_count */
    if (chunk->start_index != expected) {
        return false;  /* out-of-order — caller must send NACK */
    }

    if (pp->ring_count + chunk->count > PP_RING_CAP) {
        /* Ring full — NACK so ESP32-S3 retries once PP has consumed space.
         * Returning true here would silently drop final_chunk=true and
         * permanently prevent path completion. */
        return false;
    }

    for (uint8_t i = 0; i < chunk->count; i++) {
        pp->ring[(expected + i) % PP_RING_CAP] = chunk->wp[i];
    }
    pp->ring_count += chunk->count;

    if (chunk->final_chunk) {
        pp->final_received = true;
    }

    return true;
}

uint16_t pp_get_consumed_idx(const pure_pursuit_controller_t *pp) {
    return pp ? pp->pursuit_idx : 0;
}

uint16_t pp_get_path_id(const pure_pursuit_controller_t *pp) {
    return pp ? pp->path_id : 0;
}

uint16_t pp_get_expected_start_idx(const pure_pursuit_controller_t *pp) {
    return pp ? ring_tail(pp) : 0;
}

bool pp_has_path(const pure_pursuit_controller_t *pp) {
    return pp && pp->ring_count > 0;
}

/* ── Lookahead search ─────────────────────────────────────────────────────── */

static waypoint_t find_lookahead_point(pure_pursuit_controller_t *pp,
                                        const pose_t *pose,
                                        float lookahead_mm)
{
    uint16_t tail = ring_tail(pp);

    /* Clamp pursuit_idx into valid range. */
    if (pp->pursuit_idx < pp->ring_head) pp->pursuit_idx = pp->ring_head;
    if (pp->pursuit_idx >= tail)         pp->pursuit_idx = tail - 1u;

    /* With a single waypoint, return it directly. */
    if (pp->ring_count < 2) {
        return *ring_at(pp, pp->pursuit_idx);
    }

    /* ── Step 1: advance pursuit_idx to the nearest segment start ────────── */
    float min_dist_sq = 1e30f;
    uint16_t search_end = tail - 1u;
    if (search_end > pp->pursuit_idx + 50u) search_end = pp->pursuit_idx + 50u;

    for (uint16_t gi = pp->pursuit_idx; gi < search_end; gi++) {
        waypoint_t *s = ring_at(pp, gi);
        waypoint_t *e = ring_at(pp, gi + 1u);

        float dx = e->x - s->x, dy = e->y - s->y;
        float len_sq = dx*dx + dy*dy;
        float t = 0.0f;
        if (len_sq > 1e-6f) {
            t = ((pose->x - s->x)*dx + (pose->y - s->y)*dy) / len_sq;
        }
        t = clampf(t, 0.0f, 1.0f);

        float px = s->x + t*dx, py = s->y + t*dy;
        float d_sq = dist_sq_2d(pose->x, pose->y, px, py);
        if (d_sq < min_dist_sq) {
            min_dist_sq     = d_sq;
            pp->pursuit_idx = gi;
        }
    }

    /* ── Step 2: lookahead circle intersection ───────────────────────────── */
    waypoint_t target = *ring_at(pp, pp->pursuit_idx);
    target.v_target = pp->fixed_speed_mm_s;

    for (uint16_t gi = pp->pursuit_idx; gi < tail - 1u; gi++) {
        waypoint_t *s = ring_at(pp, gi);
        waypoint_t *e = ring_at(pp, gi + 1u);

        float dx = e->x - s->x, dy = e->y - s->y;
        float fx = s->x - pose->x, fy = s->y - pose->y;
        float a  = dx*dx + dy*dy;
        if (a < 1e-6f) continue;

        float b  = 2.0f*(fx*dx + fy*dy);
        float c  = fx*fx + fy*fy - (lookahead_mm * lookahead_mm);
        float disc = b*b - 4.0f*a*c;
        if (disc < 0.0f) continue;

        disc      = sqrtf(disc);
        float t1  = (-b - disc) / (2.0f*a);
        float t2  = (-b + disc) / (2.0f*a);
        float t   = -1.0f;
        if (t2 >= 0.0f && t2 <= 1.0f)      t = t2;
        else if (t1 >= 0.0f && t1 <= 1.0f) t = t1;
        if (t < 0.0f) continue;

        waypoint_t cand;
        cand.x = s->x + t*dx; cand.y = s->y + t*dy;
        cand.theta = 0.0f; cand.v_target = pp->fixed_speed_mm_s;

        /* Reject targets that are far behind the robot. */
        if (to_robot_frame(pose, cand).y <= -300.0f) continue;

        target = cand;
        break;
    }

    return target;
}

/* ── Main command ─────────────────────────────────────────────────────────── */

pp_motion_command_t pp_compute_command(pure_pursuit_controller_t *pp,
                                        const pose_t *pose)
{
    pp_motion_command_t stop = { .speed_mm_s = 0.0f, .steering_deg = 90.0f, .stop = true };

    if (!pp || !pose) return stop;

    uint16_t tail = ring_tail(pp);

    /* Ring underrun: no waypoints but more are expected — halt briefly
     * without sending path_done.  stop=false so the caller does not mistake
     * this for goal completion. */
    if (pp->ring_count == 0 && !pp->final_received) {
        pp_motion_command_t halt = { .speed_mm_s = 0.0f, .steering_deg = 90.0f, .stop = false };
        return halt;
    }
    if (pp->ring_count < 2 && !pp->final_received) {
        pp_motion_command_t halt = { .speed_mm_s = 0.0f, .steering_deg = 90.0f, .stop = false };
        return halt;
    }

    /* No waypoints and final chunk received: path is truly done. */
    if (pp->ring_count == 0) return stop;

    /* Goal reached: within tolerance of the last waypoint when all chunks
     * have arrived and only the last segment (≤2 waypoints) remains.
     * ring_count cannot drop below 2 in steady-state because pursuit_idx
     * is always set to a segment START (gi < tail-1), capping ring_head
     * advancement at tail-2.  Checking <= 2 lets the overshot/tolerance
     * test fire on the final segment instead of never triggering.
     * Also stop if the car has overshot (last wp is now behind the robot) —
     * prevents the car from driving straight forever past the goal. */
    if (pp->final_received && pp->ring_count <= 2) {
        waypoint_t *last = ring_at(pp, tail - 1u);
        float dsq = dist_sq_2d(pose->x, pose->y, last->x, last->y);
        if (dsq < pp->goal_tolerance_mm * pp->goal_tolerance_mm) {
            return stop;
        }
        waypoint_t last_r = to_robot_frame(pose, *last);
        if (last_r.y < 0.0f) {
            return stop;  /* overshot: last waypoint is behind the robot */
        }
    }

    waypoint_t target_g = find_lookahead_point(pp, pose, pp->lookahead_mm);
    waypoint_t target_r = to_robot_frame(pose, target_g);

    if (target_r.y <= -300.0f) {
        pp_motion_command_t straight = { pp->fixed_speed_mm_s, 90.0f, false };
        return straight;
    }

    float ld_sq = target_r.x*target_r.x + target_r.y*target_r.y;
    if (ld_sq < 1.0f) ld_sq = 1.0f;

    float curvature    = pp->kp * (2.0f * target_r.x) / ld_sq;
    float steer_rad    = atanf(curvature * pp->wheelbase_mm);
    steer_rad          = clampf(steer_rad, pp->min_steering_rad, pp->max_steering_rad);
    float steer_deg    = steer_rad * (180.0f / (float)M_PI) + 90.0f;
    steer_deg          = clampf(steer_deg, 25.0f, 155.0f);

    /* Advance ring_head to free consumed slots. */
    if (pp->pursuit_idx > pp->ring_head) {
        uint16_t advance = pp->pursuit_idx - pp->ring_head;
        if (advance <= pp->ring_count) {
            pp->ring_head  += advance;
            pp->ring_count -= advance;
        }
    }

    pp_motion_command_t cmd = { pp->fixed_speed_mm_s, steer_deg, false };
    return cmd;
}
