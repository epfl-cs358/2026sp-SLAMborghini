/**
 * pose_stub.h
 * Module: Pose stub — simulates RBPF pose estimation output for offline tests.
 * Board: ESP32-S3 (PC stub, no hardware required)
 * Used by test_rbpf and test_scan_matcher.
 */

// STUB — simulates pose estimation for offline testing

#ifndef POSE_STUB_H
#define POSE_STUB_H

#include "../../types.h"

/**
 * Returns a pose at the world origin with zero covariance.
 * Simulates the robot's initial pose before any movement.
 * Used by: test_rbpf, test_scan_matcher
 */
pose_t pose_stub_origin(void);

/**
 * Returns a pose at (500, 200) mm with heading 0.3 rad and zero covariance.
 * Simulates the robot after several seconds of forward motion with a slight left turn.
 * Used by: test_rbpf, test_scan_matcher
 */
pose_t pose_stub_moving(void);

#endif /* POSE_STUB_H */
