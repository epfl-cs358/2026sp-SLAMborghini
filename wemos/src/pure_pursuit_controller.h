#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "../../types.h"

#define PP_MAX_PATH_LENGTH 15

/**
 * Pure Pursuit output command.
 *
 * speed_mm_s: target forward speed in mm/s
 * steering_deg: servo command in degrees, where 90 = straight
 * stop: true when no valid path exists or goal is reached
 */
typedef struct {
    float speed_mm_s;
    float steering_deg;
    bool stop;
} pp_motion_command_t;

/**
 * Pure Pursuit controller state.
 *
 * Your project stores positions in mm, so:
 * - waypoints are in mm
 * - pose is in mm
 * - speed is in mm/s
 * - wheelbase and lookahead are also in mm
 */
typedef struct {
    waypoint_t current_path[PP_MAX_PATH_LENGTH];
    uint16_t path_length;
    uint16_t last_target_index;

    float wheelbase_mm;
    float lookahead_mm;
    float fixed_speed_mm_s;
    float kp;

    float min_steering_rad;
    float max_steering_rad;
    float goal_tolerance_mm;
} pure_pursuit_controller_t;

void pp_init(pure_pursuit_controller_t *pp);

void pp_set_path(
    pure_pursuit_controller_t *pp,
    const waypoint_t *path,
    uint16_t path_length
);

pp_motion_command_t pp_compute_command(
    pure_pursuit_controller_t *pp,
    const pose_t *current_pose
);

bool pp_is_path_complete(
    const pure_pursuit_controller_t *pp,
    const pose_t *current_pose
);