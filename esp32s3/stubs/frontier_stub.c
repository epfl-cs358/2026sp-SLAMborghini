// STUB — simulates frontier_detector for offline testing

/**
 * frontier_stub.c
 * Module: Frontier stub — simulates frontier detection output for offline tests.
 * Board: ESP32-S3 (PC stub, no hardware required)
 * Used by test_astar.
 */

#include "frontier_stub.h"

/*
 * Scenario: one large unexplored frontier at the room centre (2000, 1500 mm), size 10 cells.
 * Provides a realistic single-target scenario for testing the A* planner.
 * Used by: test_astar
 */
frontier_list_t frontier_stub_single(void)
{
    frontier_list_t list;
    list.count         = 1;
    list.items[0].cx   = 2000.0f;
    list.items[0].cy   = 1500.0f;
    list.items[0].size = 10;
    return list;
}

/*
 * Scenario: no frontiers remaining — exploration is complete.
 * Tests that the planner handles an empty goal list without crashing.
 * Used by: test_astar
 */
frontier_list_t frontier_stub_empty(void)
{
    frontier_list_t list;
    list.count = 0;
    /* items left uninitialised — count=0 means they are ignored */
    return list;
}
