// STUB — simulates scan_matcher for offline testing

/**
 * scan_match_stub.c
 * Module: Scan matcher stub — simulates ICP scan matching results for offline tests.
 * Board: ESP32-S3 (PC stub, no hardware required)
 * Used by test_rbpf.
 */

#include "scan_match_stub.h"

/*
 * Scenario: two identical scans — robot has not moved between frames.
 * Verifies that the RBPF update step handles a zero-drift correction gracefully.
 * Used by: test_rbpf
 */
pose_correction_t scan_match_stub_perfect(void)
{
    pose_correction_t c;
    c.dx     = 0.0f;
    c.dy     = 0.0f;
    c.dtheta = 0.0f;
    c.score  = 1.0f;
    return c;
}

/*
 * Scenario: robot has drifted 10 mm in X, 5 mm in Y, rotated ~0.01 rad (0.57 deg).
 * Score of 0.85 reflects a good-but-not-perfect ICP convergence.
 * Used by: test_rbpf
 */
pose_correction_t scan_match_stub_slight_drift(void)
{
    pose_correction_t c;
    c.dx     = 10.0f;
    c.dy     = 5.0f;
    c.dtheta = 0.01f;
    c.score  = 0.85f;
    return c;
}
