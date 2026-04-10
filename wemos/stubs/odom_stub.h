/**
 * odom_stub.h
 * Module: Odometry stub — simulates IMU and encoder odometry for offline tests.
 * Board: Wemos D1 R32 (PC stub, no hardware required)
 * Used by test_ekf.
 */

// STUB — simulates imu_encoder_driver odometry for offline testing

#ifndef ODOM_STUB_H
#define ODOM_STUB_H

#include "../../types.h"

/**
 * Returns an odom_t representing 100 mm straight-line forward motion over 100 ms.
 * Zero yaw rate — robot is driving in a perfectly straight line.
 * Used by: test_ekf
 */
odom_t odom_stub_straight(void);

/**
 * Returns an odom_t representing 50 mm forward motion with a 0.3 rad/s left turn over 100 ms.
 * Simulates the robot executing a gentle left-hand curve.
 * Used by: test_ekf
 */
odom_t odom_stub_turning(void);

#endif /* ODOM_STUB_H */
