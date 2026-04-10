/**
 * map_stub.h
 * Module: Map stub — simulates quadtree occupancy map for offline tests.
 * Board: ESP32-S3 (PC stub, no hardware required)
 * Used by test_astar and test_map_updater.
 */

// STUB — simulates quadtree_map_t for offline testing

#ifndef MAP_STUB_H
#define MAP_STUB_H

#include "../../types.h"
#include "../src/quadtree_map.h"

/**
 * Returns a zero-initialized quadtree_map_t covering 4000 x 3000 mm at 50 mm resolution.
 * Represents a completely unknown environment (no walls, no obstacles inserted).
 * Used by: test_astar, test_map_updater
 */
quadtree_map_t map_stub_empty(void);

/**
 * Returns a quadtree_map_t pre-populated with wall data for the 4 m x 3 m room.
 * Wall cells are inserted around the perimeter.
 * Used by: test_astar, test_map_updater
 */
quadtree_map_t map_stub_with_walls(void);

#endif /* MAP_STUB_H */
