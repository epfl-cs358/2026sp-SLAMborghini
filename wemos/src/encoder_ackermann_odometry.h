/**
 * encoder_ackermann_odometry.h
 *
 * State and API for local Wemos odometry fusion.  The module integrates
 * encoder distance with Ackermann steering geometry and corrects heading with
 * the IMU when the yaw sample passes jump sanity checks.
 */

#ifndef ENCODER_ACKERMANN_ODOMETRY_H
#define ENCODER_ACKERMANN_ODOMETRY_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    float x;
    float y;
    float theta;
    uint32_t timestamp_ms;
} odom_pose_t;

typedef struct {
    float wheelbase_m;
    float imu_correction_gain;
    float max_delta_dist_m;
    float max_yaw_jump_rad;
} odom_config_t;

typedef struct {
    odom_config_t cfg;
    odom_pose_t pose;

    float last_distance_m;
    float last_imu_yaw_rad;
    bool initialized;
} encoder_ackermann_odom_t;

void encoder_ackermann_odom_init(encoder_ackermann_odom_t *odom,
                                 const odom_config_t *cfg);

void encoder_ackermann_odom_reset(encoder_ackermann_odom_t *odom,
                                  float x0,
                                  float y0,
                                  float theta0);

void encoder_ackermann_odom_update(encoder_ackermann_odom_t *odom,
                                   float distance_m,
                                   float steering_rad,
                                   float imu_yaw_rad,
                                   uint32_t timestamp_ms);

const odom_pose_t *encoder_ackermann_odom_get_pose(const encoder_ackermann_odom_t *odom);

#endif
