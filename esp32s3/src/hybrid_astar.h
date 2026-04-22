/**
 * Plan a path from start to the given frontier goal.
 * Current implementation builds a sparse adjacency graph from traversable
 * quadtree leaves and runs A* directly on that graph.
 * @param map   Pointer to the current quadtree map.
 * @param start Pointer to the current robot pose.
 * @param goal  Pointer to the target frontier.
 * @return path_t with length > 0 on success, or length == 0 on failure.
 */

#ifndef HYBRID_ASTAR_H
#define HYBRID_ASTAR_H

#include <stdbool.h>
#include "../../types.h"
#include "quadtree_map.h"

#ifdef USE_STUBS
#include "../stubs/astar_stub.h"
#endif

/** A planned path represented as an ordered sequence of waypoints. */
typedef struct {
    waypoint_t waypoints[64]; /**< Ordered list of waypoints */
    uint8_t    length;        /**< Number of valid waypoints in the path */
} path_t;

/**
 * Plan a kinematically feasible path from start to the given frontier goal.
  * Uses the quadtree map as the source occupancy representation and may build
 * an internal planning grid for collision checking and search.
 * @param map   Pointer to the current quadtree map (const, for collision checks).
 * @param start Pointer to the current robot pose.
 * @param goal  Pointer to the target frontier.
 * @return path_t with length > 0 on success, or length == 0 on planning failure.
 */
path_t hybrid_astar_plan(const quadtree_map_t *map, const pose_t *start,
                         const frontier_t *goal);

/**
 * Check whether a path contains at least one valid waypoint.
 * @param path Pointer to the path to validate.
 * @return true if path->length >= 1, false otherwise.
 */
bool hybrid_astar_is_valid(const path_t *path);

#endif /* HYBRID_ASTAR_H */
