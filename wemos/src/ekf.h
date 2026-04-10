/**
 * ekf.h
 * Module: Extended Kalman Filter for wheel odometry and IMU fusion.
 * Board: Wemos D1 R32
 * Fuses encoder-derived linear displacement with IMU yaw rate to produce a
 * smooth, drift-compensated 2-D pose estimate.
 */

#ifndef EKF_H
#define EKF_H

#include "../../types.h"

#ifdef USE_STUBS
#include "../stubs/odom_stub.h"
#endif

/** EKF state: 2-D pose (x, y, theta) with 3x3 covariance matrix. */
typedef struct {
    float x;     /**< X position estimate in mm */
    float y;     /**< Y position estimate in mm */
    float theta; /**< Heading estimate in radians */
    float P[9];  /**< 3x3 covariance matrix (row-major: xx,xy,xt,yx,yy,yt,tx,ty,tt) */
} ekf_state_t;

/**
 * Initialize the EKF state to the world origin with high initial covariance.
 * @param state Pointer to the ekf_state_t to initialize.
 */
void ekf_init(ekf_state_t *state);

/**
 * EKF prediction step: propagate the state estimate using the odometry motion model.
 * @param state Pointer to the current EKF state (modified in place).
 * @param odom  Pointer to the latest odometry measurement.
 */
void ekf_predict(ekf_state_t *state, const odom_t *odom);

/**
 * EKF update (correction) step: incorporate an external pose observation.
 * Can be driven by the SLAM correction received from ESP32-S3.
 * @param state     Pointer to the current EKF state (modified in place).
 * @param obs_x     Observed X position in mm.
 * @param obs_y     Observed Y position in mm.
 * @param obs_theta Observed heading in radians.
 */
void ekf_update(ekf_state_t *state, float obs_x, float obs_y, float obs_theta);

/**
 * Convert the EKF state into a pose_t for use by higher-level modules.
 * @param state Pointer to the current EKF state (const).
 * @return pose_t containing the current position, heading, and a covariance summary.
 */
pose_t ekf_get_pose(const ekf_state_t *state);

#endif /* EKF_H */
