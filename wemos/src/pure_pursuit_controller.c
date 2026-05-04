/**
 * pure_pursuit_controller.c
 *
 * Pure Pursuit local path-following controller for the Wemos Control Brain.
 */

#include "pure_pursuit_controller.h"

#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

static float clampf(float value, float min_val, float max_val) {
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

static uint16_t min_u16(uint16_t a, uint16_t b) {
    return (a < b) ? a : b;
}

static float dist_sq_pose_waypoint(const pose_t *pose, const waypoint_t *wp) {
    float dx = wp->x - pose->x;
    float dy = wp->y - pose->y;
    return dx * dx + dy * dy;
}

/*
 * Transform a global waypoint into the robot frame.
 *
 * Convention used here:
 * - robot-frame X is lateral/right
 * - robot-frame Y is forward
 * - pose->theta is the robot heading in radians
 */
static waypoint_t to_robot_frame(const pose_t *robot_pose, waypoint_t point_global) {
    float dx = point_global.x - robot_pose->x;
    float dy = point_global.y - robot_pose->y;

    waypoint_t point_robot = point_global;

    point_robot.x = dx * cosf(robot_pose->theta) + dy * sinf(robot_pose->theta);
    point_robot.y = -dx * sinf(robot_pose->theta) + dy * cosf(robot_pose->theta);

    return point_robot;
}

void pp_init(pure_pursuit_controller_t *pp) {
    if (pp == NULL) {
        return;
    }

    memset(pp, 0, sizeof(*pp));

    /*
     * Your project uses millimetres.
     * Initial values are equivalent to:
     * - wheelbase: 0.26 m
     * - lookahead: 0.70 m
     * - speed: 0.20 m/s
     * - goal tolerance: 0.10 m
     */
    pp->wheelbase_mm = 260.0f;
    pp->lookahead_mm = 700.0f;
    pp->fixed_speed_mm_s = 200.0f;
    pp->kp = 1.0f;

    /*
     * Steering limited to ±30 degrees.
     */
    pp->min_steering_rad = -0.523599f;
    pp->max_steering_rad =  0.523599f;

    pp->goal_tolerance_mm = 100.0f;
}

void pp_set_path(
    pure_pursuit_controller_t *pp,
    const waypoint_t *path,
    uint16_t path_length
) {
    if (pp == NULL || path == NULL) {
        return;
    }

    if (path_length > PP_MAX_PATH_LENGTH) {
        path_length = PP_MAX_PATH_LENGTH;
    }

    pp->path_length = path_length;

    if (path_length > 0) {
        memcpy(pp->current_path, path, path_length * sizeof(waypoint_t));
    }

    pp->last_target_index = 0;
}

bool pp_is_path_complete(
    const pure_pursuit_controller_t *pp,
    const pose_t *current_pose
) {
    if (pp == NULL || current_pose == NULL) {
        return true;
    }

    if (pp->path_length == 0) {
        return true;
    }

    const waypoint_t *final_goal = &pp->current_path[pp->path_length - 1];
    float dist_to_goal_sq = dist_sq_pose_waypoint(current_pose, final_goal);

    return dist_to_goal_sq < (pp->goal_tolerance_mm * pp->goal_tolerance_mm);
}

static waypoint_t find_lookahead_point(
    pure_pursuit_controller_t *pp,
    const pose_t *current_pose,
    float lookahead_mm
) {
    if (pp->path_length == 0) {
        waypoint_t zero = {0};
        return zero;
    }

    if (pp->last_target_index >= pp->path_length) {
        pp->last_target_index = pp->path_length - 1;
    }

    if (pp->path_length == 1) {
        return pp->current_path[0];
    }

    /*
     * Step 1:
     * Find the closest point on nearby path segments.
     * This prevents the controller from jumping backwards on the path.
     */
    float min_dist_sq = 1e30f;
    waypoint_t closest_point = pp->current_path[pp->last_target_index];

    uint16_t search_limit = min_u16(
        (uint16_t)(pp->path_length - 1),
        (uint16_t)(pp->last_target_index + 50)
    );

    for (uint16_t i = pp->last_target_index; i < search_limit; ++i) {
        waypoint_t start = pp->current_path[i];
        waypoint_t end = pp->current_path[i + 1];

        float dx = end.x - start.x;
        float dy = end.y - start.y;
        float len_sq = dx * dx + dy * dy;

        float t = 0.0f;
        if (len_sq > 1e-6f) {
            t = ((current_pose->x - start.x) * dx +
                 (current_pose->y - start.y) * dy) / len_sq;
        }

        t = clampf(t, 0.0f, 1.0f);

        float px = start.x + t * dx;
        float py = start.y + t * dy;

        float ex = current_pose->x - px;
        float ey = current_pose->y - py;
        float d_sq = ex * ex + ey * ey;

        if (d_sq < min_dist_sq) {
            min_dist_sq = d_sq;
            closest_point.x = px;
            closest_point.y = py;
            closest_point.theta = 0.0f;
            closest_point.v_target = pp->fixed_speed_mm_s;
            pp->last_target_index = i;
        }
    }

    /*
     * Step 2:
     * Search for the intersection between the path and the lookahead circle.
     * Keep the furthest valid intersection along the path.
     */
    waypoint_t target_point = closest_point;

    for (uint16_t i = pp->last_target_index; i < pp->path_length - 1; ++i) {
        waypoint_t start = pp->current_path[i];
        waypoint_t end = pp->current_path[i + 1];

        float dx = end.x - start.x;
        float dy = end.y - start.y;

        float fx = start.x - current_pose->x;
        float fy = start.y - current_pose->y;

        float a = dx * dx + dy * dy;

        if (a < 1e-6f) {
            continue;
        }

        float b = 2.0f * (fx * dx + fy * dy);
        float c = (fx * fx + fy * fy) - (lookahead_mm * lookahead_mm);

        float discriminant = b * b - 4.0f * a * c;

        if (discriminant >= 0.0f) {
            discriminant = sqrtf(discriminant);

            float t1 = (-b - discriminant) / (2.0f * a);
            float t2 = (-b + discriminant) / (2.0f * a);

            if (t2 >= 0.0f && t2 <= 1.0f) {
                target_point.x = start.x + t2 * dx;
                target_point.y = start.y + t2 * dy;
                target_point.theta = 0.0f;
                target_point.v_target = pp->fixed_speed_mm_s;
            } else if (t1 >= 0.0f && t1 <= 1.0f) {
                target_point.x = start.x + t1 * dx;
                target_point.y = start.y + t1 * dy;
                target_point.theta = 0.0f;
                target_point.v_target = pp->fixed_speed_mm_s;
            }
        }
    }

    return target_point;
}

pp_motion_command_t pp_compute_command(
    pure_pursuit_controller_t *pp,
    const pose_t *current_pose
) {
    pp_motion_command_t stop_cmd;
    stop_cmd.speed_mm_s = 0.0f;
    stop_cmd.steering_deg = 90.0f;
    stop_cmd.stop = true;

    if (pp == NULL || current_pose == NULL) {
        return stop_cmd;
    }

    if (pp->path_length == 0 || pp_is_path_complete(pp, current_pose)) {
        if (pp->path_length > 0) {
            pp->last_target_index = pp->path_length - 1;
        }
        return stop_cmd;
    }

    waypoint_t target_global = find_lookahead_point(
        pp,
        current_pose,
        pp->lookahead_mm
    );

    waypoint_t target_robot = to_robot_frame(current_pose, target_global);

    /*
     * Pure Pursuit:
     * curvature = 2 * lateral_offset / lookahead^2
     */
    float curvature = pp->kp *
        (2.0f * target_robot.x) /
        (pp->lookahead_mm * pp->lookahead_mm);

    float steering_rad = atanf(curvature * pp->wheelbase_mm);

    steering_rad = clampf(
        steering_rad,
        pp->min_steering_rad,
        pp->max_steering_rad
    );

    float steering_deg = steering_rad * (180.0f / M_PI) + 90.0f;

    /*
     * Servo command:
     * 90 = straight
     * 60 = max left/right depending on wiring
     * 120 = max right/left depending on wiring
     */
    steering_deg = clampf(steering_deg, 60.0f, 120.0f);

    pp_motion_command_t cmd;
    cmd.speed_mm_s = pp->fixed_speed_mm_s;
    cmd.steering_deg = steering_deg;
    cmd.stop = false;

    return cmd;
}