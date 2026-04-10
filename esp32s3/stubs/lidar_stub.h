/**
 * lidar_stub.h
 * Module: LiDAR stub — simulates RPLiDAR C1 output for offline testing.
 * Board: ESP32-S3 (PC stub, no hardware required)
 * Used by test_scan_matcher and test_map_updater.
 */

// STUB — simulates RPLiDAR C1 for offline testing

#ifndef LIDAR_STUB_H
#define LIDAR_STUB_H

#include "../../types.h"

/**
 * Returns a synthetic scan of a 4000 mm x 3000 mm rectangular room.
 * Approximately 460 points are evenly distributed around the four walls.
 * Used by: test_scan_matcher, test_map_updater
 */
lidar_scan_t lidar_stub_room_scan(void);

/**
 * Returns a second synthetic scan identical to lidar_stub_room_scan()
 * but with a very small perturbation — simulates a nearly perfect match.
 * Used by: test_scan_matcher
 */
lidar_scan_t lidar_stub_room_scan_perturbed(void);

#endif /* LIDAR_STUB_H */
