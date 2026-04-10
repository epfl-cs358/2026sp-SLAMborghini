/**
 * map_consistency.h
 * Module: Map consistency checker and loop closure candidate detector.
 * Board: ESP32-S3
 * Verifies that the current estimated pose is geometrically consistent with
 * the existing map contents, and marks revisited regions for loop closure.
 */

#ifndef MAP_CONSISTENCY_H
#define MAP_CONSISTENCY_H

#include <stdbool.h>
#include "../../types.h"
#include "quadtree_map.h"

/**
 * Check whether the given pose is consistent with the occupied cells in the map.
 * Returns false if the pose places the robot inside a known obstacle.
 * @param map  Pointer to the current quadtree map (const).
 * @param pose Pointer to the pose to validate.
 * @return true if the pose is consistent with the map, false otherwise.
 */
bool map_consistency_check(const quadtree_map_t *map, const pose_t *pose);

/**
 * Mark the region around the given pose as revisited in the map.
 * Used to trigger loop closure processing when a previously-mapped area is re-entered.
 * @param map  Pointer to the quadtree map to annotate.
 * @param pose Pointer to the current robot pose.
 */
void map_consistency_mark_revisited(quadtree_map_t *map, const pose_t *pose);

#endif /* MAP_CONSISTENCY_H */
