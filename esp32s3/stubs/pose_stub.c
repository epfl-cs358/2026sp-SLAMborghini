// STUB — simulates pose estimation for offline testing

/**
 * pose_stub.c
 * Module: Pose stub — simulates RBPF pose estimation output for offline tests.
 * Board: ESP32-S3 (PC stub, no hardware required)
 * Used by test_rbpf and test_scan_matcher.
 */

#include "pose_stub.h"

/*
 * Scenario: robot at world origin (0, 0), facing east (theta = 0).
 * Represents initial power-on state before odometry has accumulated.
 * Used by: test_rbpf, test_scan_matcher
 */
pose_t pose_stub_origin(void)
{
    pose_t p;
    p.x        = 0.0f;
    p.y        = 0.0f;
    p.theta    = 0.0f;
    p.cov[0]   = 0.0f;
    p.cov[1]   = 0.0f;
    p.cov[2]   = 0.0f;
    p.cov[3]   = 0.0f;
    p.cov[4]   = 0.0f;
    p.cov[5]   = 0.0f;
    return p;
}

/*
 * Scenario: robot has moved to (500, 200) mm with heading 0.3 rad (~17 deg left turn).
 * Represents a realistic mid-corridor pose after ~2 s of travel at 300 mm/s.
 * Used by: test_rbpf, test_scan_matcher
 */
pose_t pose_stub_moving(void)
{
    pose_t p;
    p.x        = 500.0f;
    p.y        = 200.0f;
    p.theta    = 0.3f;
    p.cov[0]   = 0.0f;
    p.cov[1]   = 0.0f;
    p.cov[2]   = 0.0f;
    p.cov[3]   = 0.0f;
    p.cov[4]   = 0.0f;
    p.cov[5]   = 0.0f;
    return p;
}
