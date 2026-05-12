#include "encoder_ackermann_odometry.h"
#include <math.h>

/* O(1) for any finite float — safe even if imu heading drifts large. */
static float wrap_angle(float a)
{
    a = fmodf(a, 2.0f * (float)M_PI);
    if (a >  (float)M_PI) a -= 2.0f * (float)M_PI;
    if (a < -(float)M_PI) a += 2.0f * (float)M_PI;
    return a;
}

void encoder_ackermann_odom_init(encoder_ackermann_odom_t *odom,
                                 const odom_config_t *cfg)
{
    if (odom == NULL || cfg == NULL) {
        return;
    }

    odom->cfg = *cfg;
    encoder_ackermann_odom_reset(odom, 0.0f, 0.0f, 0.0f);
}

void encoder_ackermann_odom_reset(encoder_ackermann_odom_t *odom,
                                  float x0,
                                  float y0,
                                  float theta0)
{
    if (odom == NULL) {
        return;
    }

    odom->pose.x = x0;
    odom->pose.y = y0;
    odom->pose.theta = theta0;
    odom->pose.timestamp_ms = 0U;

    odom->last_distance_m = 0.0f;
    odom->last_imu_yaw_rad = theta0;
    odom->initialized = false;
}

void encoder_ackermann_odom_update(encoder_ackermann_odom_t *odom,
                                   float distance_m,
                                   float steering_rad,
                                   float imu_yaw_rad,
                                   uint32_t timestamp_ms)
{
    if (odom == NULL) {
        return;
    }

    if (!odom->initialized) {
        odom->last_distance_m = distance_m;
        odom->last_imu_yaw_rad = imu_yaw_rad;
        odom->pose.theta = imu_yaw_rad;
        odom->pose.timestamp_ms = timestamp_ms;
        odom->initialized = true;
        return;
    }

    /* 1. Incremental traveled distance */
    float ds = distance_m - odom->last_distance_m;
    odom->last_distance_m = distance_m;

    /* 2. Reject impossible encoder jump */
    if (fabsf(ds) > odom->cfg.max_delta_dist_m) {
        odom->pose.timestamp_ms = timestamp_ms;
        return;
    }

    /* 3. Ackermann prediction — clamp away from ±π/2 where tanf → ±Inf */
    float dtheta_ack = 0.0f;
    if (fabsf(odom->cfg.wheelbase_m) > 1e-6f) {
        float s = steering_rad;
        if (s >  1.50f) s =  1.50f;
        if (s < -1.50f) s = -1.50f;
        dtheta_ack = (ds / odom->cfg.wheelbase_m) * tanf(s);
    }

    float theta_pred = wrap_angle(odom->pose.theta + dtheta_ack);

    /* 4. IMU sanity check */
    float imu_jump = wrap_angle(imu_yaw_rad - odom->last_imu_yaw_rad);
    odom->last_imu_yaw_rad = imu_yaw_rad;

    float theta_meas = odom->pose.theta;
    if (fabsf(imu_jump) <= odom->cfg.max_yaw_jump_rad) {
        theta_meas = imu_yaw_rad;
    }

    /* 5. Complementary correction */
    float innovation = wrap_angle(theta_meas - theta_pred);
    float theta_new = wrap_angle(theta_pred + odom->cfg.imu_correction_gain * innovation);

    /* 6. Midpoint integration */
    float theta_mid = wrap_angle(odom->pose.theta + 0.5f * wrap_angle(theta_new - odom->pose.theta));

    odom->pose.x += ds * cosf(theta_mid);
    odom->pose.y += ds * sinf(theta_mid);
    odom->pose.theta = theta_new;
    odom->pose.timestamp_ms = timestamp_ms;
}

const odom_pose_t *encoder_ackermann_odom_get_pose(const encoder_ackermann_odom_t *odom)
{
    if (odom == NULL) {
        return NULL;
    }
    return &odom->pose;
}