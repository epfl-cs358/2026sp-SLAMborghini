#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "../../types.h"

/* Ring capacity.  Must be > PATH_CHUNK_WP_COUNT (8) × 2 so there is always
 * room for one incoming chunk while a full target fill is already buffered. */
#define PP_RING_CAP 32

typedef struct {
    float speed_mm_s;
    float steering_deg;
    bool  stop;
} pp_motion_command_t;

typedef struct {
    waypoint_t  ring[PP_RING_CAP];
    uint16_t    ring_head;      /* global index of oldest slot currently in ring */
    uint16_t    ring_count;     /* number of valid waypoints in the ring         */
    uint16_t    path_id;        /* path_id from the last accepted chunk          */
    bool        final_received; /* true after a chunk with final_chunk=true      */
    uint16_t    pursuit_idx;    /* global segment-start index (consumed progress)*/
    float       goal_x_mm;
    float       goal_y_mm;

    float wheelbase_mm;
    float lookahead_mm;
    float fixed_speed_mm_s;
    float kp;
    float min_steering_rad;
    float max_steering_rad;
    float goal_tolerance_mm;
} pure_pursuit_controller_t;

void pp_init(pure_pursuit_controller_t *pp);

/**
 * Append one streaming chunk to the ring buffer.
 *
 * A new path_id resets the ring to index 0 so the first chunk must have
 * start_index==0 — any other start_index triggers a NACK.
 *
 * Returns true  → chunk accepted.
 * Returns false → out-of-order; call pp_get_expected_start_idx() and send a
 *                 NACK to ESP32-S3.
 */
bool pp_append_chunk(pure_pursuit_controller_t *pp, const path_chunk_t *chunk);

/* Global waypoint index of the segment we are currently pursuing.
 * Copy into odom_t.consumed_wp_idx every tick. */
uint16_t pp_get_consumed_idx(const pure_pursuit_controller_t *pp);

uint16_t pp_get_path_id(const pure_pursuit_controller_t *pp);

/* Next global index we need from ESP32-S3 (= ring_head + ring_count).
 * Use this value in a NACK payload when pp_append_chunk returns false. */
uint16_t pp_get_expected_start_idx(const pure_pursuit_controller_t *pp);

/* True if the ring has at least one waypoint to pursue. */
bool pp_has_path(const pure_pursuit_controller_t *pp);

pp_motion_command_t pp_compute_command(pure_pursuit_controller_t *pp,
                                        const pose_t *current_pose);
