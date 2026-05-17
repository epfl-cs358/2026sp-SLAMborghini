#ifndef HYBRID_ASTAR_H
#define HYBRID_ASTAR_H

#include <stdbool.h>
#include <stdint.h>
#include "../../types.h"
#include "quadtree_map.h"

#ifdef USE_STUBS
#include "../stubs/astar_stub.h"
#endif

#ifndef HYBRID_ASTAR_MAX_WAYPOINTS
#define HYBRID_ASTAR_MAX_WAYPOINTS 64
#endif

/** A planned path represented as an ordered sequence of waypoints. */
typedef struct {
    waypoint_t waypoints[HYBRID_ASTAR_MAX_WAYPOINTS];
    uint8_t    length;
} path_t;

/**
 * Plan a kinematically feasible path from start to a frontier goal using
 * Hybrid A*.
 *
 * @param map   Pointer to the current quadtree map.
 * @param start Pointer to the current robot pose.
 * @param goal  Pointer to the target frontier.
 * @return path_t with length > 0 on success, or length == 0 on failure.
 */
path_t hybrid_astar_plan(const quadtree_map_t *map,
                         const pose_t *start,
                         const frontier_t *goal);

/**
 * Check whether a path contains at least one valid waypoint.
 *
 * @param path Pointer to the path to validate.
 * @return true if path->length >= 1, false otherwise.
 */
bool hybrid_astar_is_valid(const path_t *path);

#endif /* HYBRID_ASTAR_H */
