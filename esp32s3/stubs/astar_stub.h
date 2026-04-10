/**
 * astar_stub.h
 * Module: A* stub — simulates hybrid_astar path planner output for offline tests.
 * Board: ESP32-S3 (PC stub, no hardware required)
 * Used by test_astar.
 */

// STUB — simulates hybrid_astar for offline testing

#ifndef ASTAR_STUB_H
#define ASTAR_STUB_H

#include "../../types.h"
#include "../src/hybrid_astar.h"

/**
 * Returns a path_t with 3 waypoints along the positive X axis:
 *   (0,0,0,0.3 m/s) -> (500,0,0,0.3 m/s) -> (1000,0,0,0.3 m/s)
 * Simulates a straight-line path toward a nearby goal with constant speed.
 * Used by: test_astar
 */
path_t astar_stub_straight(void);

#endif /* ASTAR_STUB_H */
