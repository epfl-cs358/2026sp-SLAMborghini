/**
 * scan_match_stub.h
 * Module: Scan matcher stub — simulates ICP scan matching results for offline tests.
 * Board: ESP32-S3 (PC stub, no hardware required)
 * Used by test_rbpf.
 */

// STUB — simulates scan_matcher for offline testing

#ifndef SCAN_MATCH_STUB_H
#define SCAN_MATCH_STUB_H

#include "../../types.h"

/**
 * Returns a perfect scan match correction: zero displacement, score = 1.0.
 * Simulates the scenario where two consecutive scans are identical (robot stationary).
 * Used by: test_rbpf
 */
pose_correction_t scan_match_stub_perfect(void);

/**
 * Returns a slight drift correction: small dx/dy, dtheta ~0.01 rad, score = 0.85.
 * Simulates the scenario where the robot has moved slightly between scans.
 * Used by: test_rbpf
 */
pose_correction_t scan_match_stub_slight_drift(void);

#endif /* SCAN_MATCH_STUB_H */
