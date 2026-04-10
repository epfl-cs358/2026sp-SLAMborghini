/**
 * path_refiner.h
 * Module: Path refiner — smoothing and shortcutting for Hybrid A* paths.
 * Board: ESP32-S3
 * Post-processes raw planner output to reduce path length and improve curvature
 * continuity before the path is handed to the command generator.
 */

#ifndef PATH_REFINER_H
#define PATH_REFINER_H

#include "../../types.h"
#include "hybrid_astar.h"
#include "quadtree_map.h"

/**
 * Smooth a path in-place using a sliding window average over waypoint positions.
 * Reduces sharp heading changes while preserving start and end waypoints.
 * @param path Pointer to the path to smooth (modified in place).
 */
void path_refiner_smooth(path_t *path);

/**
 * Remove redundant intermediate waypoints where line-of-sight is clear.
 * Iteratively checks if waypoint[i] can be skipped by drawing a straight line
 * from waypoint[i-1] to waypoint[i+1] and collision-checking against the map.
 * @param path Pointer to the path to shortcut (modified in place).
 * @param map  Pointer to the quadtree map for collision checking (const).
 */
void path_refiner_shortcut(path_t *path, const quadtree_map_t *map);

#endif /* PATH_REFINER_H */
