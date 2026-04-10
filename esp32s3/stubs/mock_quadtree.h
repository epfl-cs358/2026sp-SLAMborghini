/**
 * mock_quadtree.h
 * Module: Mock quadtree — flat-grid substitute for offline frontier testing.
 * Board: PC (compiled without hardware, -DUSE_STUBS)
 *
 * Provides the full quadtree_map_t API (quadtree_map_init, quadtree_map_query,
 * quadtree_map_insert, quadtree_map_free) backed by a static flat uint8_t grid
 * instead of a real tree. Allows frontier_detector to run end-to-end without
 * the quadtree implementation being ready.
 *
 * Also exposes get_populated_map() which returns a pre-built mock room:
 *   - 30 x 20 cells at 50 mm resolution  = 1500 mm x 1000 mm
 *   - Perimeter cells: occupied (255)
 *   - Central free zone  (radius ~6 cells around robot start): free (20)
 *   - Everything else: unknown (128)
 *
 * The free/unknown boundary is the frontier the detector must find.
 * Robot starts at the centre cell (15, 10).
 *
 * Compile with this file instead of quadtree_map.c — do NOT link both.
 * Used by: test_frontier_detector
 */

#ifndef MOCK_QUADTREE_H
#define MOCK_QUADTREE_H

#include "../../types.h"
#include "../src/quadtree_map.h"

/* Mock room dimensions */
#define MOCK_GRID_W   30    /* cells along X */
#define MOCK_GRID_H   20    /* cells along Y */
#define MOCK_RES_MM   50.0f
#define MOCK_ROBOT_IX 15    /* robot start cell X */
#define MOCK_ROBOT_IY 10    /* robot start cell Y */

/**
 * Returns a quadtree_map_t backed by a static flat grid representing a simple
 * rectangular room. The map->root pointer holds the flat uint8_t grid array.
 * quadtree_map_query() in this stub reads directly from that array.
 *
 * Room layout:
 *   255 = occupied  (perimeter walls)
 *    20 = free       (central area visible from robot start)
 *   128 = unknown   (everything beyond the first scan bubble)
 */
quadtree_map_t get_populated_map(void);

/**
 * Convenience: return the robot pose for the mock room.
 * Robot starts at the centre cell, heading = 0 (facing +X).
 */
pose_t get_mock_robot_pose(void);

#endif /* MOCK_QUADTREE_H */
