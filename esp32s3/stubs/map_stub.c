// STUB — simulates quadtree_map_t for offline testing

/**
 * map_stub.c
 * Module: Map stub — simulates quadtree occupancy map for offline tests.
 * Board: ESP32-S3 (PC stub, no hardware required)
 * Used by test_astar and test_map_updater.
 */

#include "map_stub.h"

/*
 * Scenario: completely empty / unknown environment (robot has not yet observed anything).
 * Provides a blank map suitable for testing planners and updaters from a clean state.
 * Used by: test_astar, test_map_updater
 */
quadtree_map_t map_stub_empty(void)
{
    quadtree_map_t map;
    map.root          = (void *)0; /* NULL root — no nodes allocated */
    map.resolution_mm = 50.0f;
    map.width_mm      = 4000.0f;
    map.height_mm     = 3000.0f;
    return map;
}

/*
 * Scenario: 4 m x 3 m rectangular room with walls along the perimeter.
 * Pre-populated so planners encounter obstacles when approaching the boundary.
 * Used by: test_astar, test_map_updater
 * TODO: implement actual wall-cell insertion once quadtree_map_insert() is ready.
 */
quadtree_map_t map_stub_with_walls(void)
{
    /* Return empty map for now; real wall insertion requires a live quadtree. */
    quadtree_map_t map = map_stub_empty();
    /* TODO: call quadtree_map_insert() along perimeter once hardware stub is complete */
    return map;
}
