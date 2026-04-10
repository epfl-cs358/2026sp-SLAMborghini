/**
 * map_updater.h
 * Module: Map updater — integrates classified point cloud into the quadtree map.
 * Board: ESP32-S3
 * Transforms classified LiDAR points from robot-local coordinates to world coordinates
 * using the current best pose, then inserts them into the quadtree occupancy map.
 */

#ifndef MAP_UPDATER_H
#define MAP_UPDATER_H

#include <stdint.h>
#include "../../types.h"
#include "quadtree_map.h"

/**
 * Transform and insert a set of classified points into the map.
 * Uses the provided pose to convert robot-frame coordinates to world-frame coordinates
 * before calling quadtree_map_insert() for each point.
 * @param map   Pointer to the quadtree map to update.
 * @param pts   Array of classified points in robot-local coordinates.
 * @param count Number of points in the array.
 * @param pose  Current best robot pose (world frame).
 */
void map_updater_update(quadtree_map_t *map, const classified_point_t *pts,
                        uint16_t count, const pose_t *pose);

#endif /* MAP_UPDATER_H */
