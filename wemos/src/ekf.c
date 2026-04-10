/**
 * ekf.c
 * Module: Extended Kalman Filter for wheel odometry and IMU fusion.
 * Board: Wemos D1 R32
 * Implementation phase: stub (Jacobian and covariance update not yet implemented)
 */

#include "ekf.h"

void ekf_init(ekf_state_t *state)
{
    // TODO: implement
    // Set state->x = state->y = state->theta = 0.
    // Initialize P to a diagonal matrix with large initial variance:
    //   P[0] = P[4] = P[8] = 1e6f (position uncertainty 1000 m^2)
    //   All off-diagonals = 0.
    (void)state;
}

void ekf_predict(ekf_state_t *state, const odom_t *odom)
{
    // TODO: implement
    // State transition model (differential drive):
    //   dt   = odom->dt_ms / 1000.0f
    //   d    = odom->linear_disp_mm
    //   dth  = odom->yaw_rate_imu * dt
    //   state->x     += d * cosf(state->theta + dth / 2.0f)
    //   state->y     += d * sinf(state->theta + dth / 2.0f)
    //   state->theta += dth
    // Compute Jacobian F of the motion model w.r.t. state.
    // Propagate covariance: P = F * P * F^T + Q (Q = process noise).
    (void)state;
    (void)odom;
}

void ekf_update(ekf_state_t *state, float obs_x, float obs_y, float obs_theta)
{
    // TODO: implement
    // Innovation y = [obs_x - state->x, obs_y - state->y, obs_theta - state->theta]
    // Innovation covariance S = H * P * H^T + R (H = identity for direct pose obs)
    // Kalman gain K = P * H^T * S^-1
    // state->x     += K[0] * y[0]
    // state->y     += K[4] * y[1]
    // state->theta += K[8] * y[2]
    // P = (I - K * H) * P
    (void)state;
    (void)obs_x;
    (void)obs_y;
    (void)obs_theta;
}

pose_t ekf_get_pose(const ekf_state_t *state)
{
    // TODO: implement
    // Copy state->x, state->y, state->theta into pose.
    // Map diagonal P entries into pose.cov[0,3,5] (xx, yy, tt).
    (void)state;
    pose_t result = {0};
    return result;
}
