// STUB — simulates imu_encoder_driver odometry for offline testing

/**
 * odom_stub.c
 * Module: Odometry stub — simulates IMU and encoder odometry for offline tests.
 * Board: Wemos D1 R32 (PC stub, no hardware required)
 * Used by test_ekf.
 */

#include "odom_stub.h"

/*
 * Scenario: robot driving in a perfectly straight line at 1000 mm/s.
 * 100 ms window, no yaw drift — good for testing EKF x-axis integration.
 * Used by: test_ekf
 */
odom_t odom_stub_straight(void)
{
    odom_t o;
    o.linear_disp_mm = 100.0f;
    o.yaw_rate_imu   = 0.0f;
    o.dt_ms          = 100.0f;
    return o;
}

/*
 * Scenario: robot executing a left turn at 500 mm/s with 0.3 rad/s yaw rate.
 * Over 100 ms the robot turns ~1.7 degrees — typical gentle corridor turn.
 * Used by: test_ekf
 */
odom_t odom_stub_turning(void)
{
    odom_t o;
    o.linear_disp_mm = 50.0f;
    o.yaw_rate_imu   = 0.3f;
    o.dt_ms          = 100.0f;
    return o;
}
