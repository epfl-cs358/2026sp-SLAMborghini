/**
 * frontier_stub.h
 * Module: Frontier stub — simulates frontier detection output for offline tests.
 * Board: ESP32-S3 (PC stub, no hardware required)
 * Used by test_astar.
 */

// STUB — simulates frontier_detector for offline testing

#ifndef FRONTIER_STUB_H
#define FRONTIER_STUB_H

#include "../../types.h"

/**
 * Returns a frontier_list_t containing one frontier at (2000, 1500) with size = 10.
 * Simulates detecting a single large unexplored region at the room centre.
 * Used by: test_astar
 */
frontier_list_t frontier_stub_single(void);

/**
 * Returns a frontier_list_t with count = 0 (no frontiers detected).
 * Simulates a fully-explored environment where no further exploration targets exist.
 * Used by: test_astar
 */
frontier_list_t frontier_stub_empty(void);

#endif /* FRONTIER_STUB_H */
