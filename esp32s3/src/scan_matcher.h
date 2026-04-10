/**
 * scan_matcher.h
 * Module: ICP-based scan matching (point-to-point).
 * Board: ESP32-S3
 * Estimates the relative pose correction between two consecutive LiDAR point clouds
 * using an Iterative Closest Point algorithm.
 */

#ifndef SCAN_MATCHER_H
#define SCAN_MATCHER_H

#include <stdint.h>
#include "../../types.h"

#ifdef USE_STUBS
#include "../stubs/scan_match_stub.h"
#endif

/**
 * Compute the pose correction that aligns the current scan to the reference scan
 * using point-to-point ICP.
 * @param ref   Reference point cloud (previous scan).
 * @param ref_n Number of points in the reference cloud.
 * @param cur   Current point cloud (new scan to align).
 * @param cur_n Number of points in the current cloud.
 * @return pose_correction_t with dx, dy, dtheta (mm / rad) and a quality score [0,1].
 */
pose_correction_t scan_matcher_match(const point2f_t *ref, uint16_t ref_n,
                                     const point2f_t *cur, uint16_t cur_n);

/**
 * Set the maximum number of ICP iterations per call to scan_matcher_match().
 * Higher values improve accuracy at the cost of execution time.
 * @param iters Maximum iteration count (recommended: 10-50).
 */
void scan_matcher_set_max_iter(uint8_t iters);

#endif /* SCAN_MATCHER_H */
