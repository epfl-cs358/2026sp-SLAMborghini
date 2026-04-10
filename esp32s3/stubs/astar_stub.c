// STUB — simulates hybrid_astar for offline testing

/**
 * astar_stub.c
 * Module: A* stub — simulates hybrid_astar path planner output for offline tests.
 * Board: ESP32-S3 (PC stub, no hardware required)
 * Used by test_astar.
 */

#include "astar_stub.h"

/*
 * Scenario: straight-line path along the X axis at constant speed 300 mm/s.
 * Three waypoints: origin, 500 mm ahead, 1000 mm ahead.
 * Tests that hybrid_astar_is_valid() accepts this path and that command_gen
 * can consume the first waypoint.
 * Used by: test_astar
 */
path_t astar_stub_straight(void)
{
    path_t path;
    path.length = 3;

    /* Waypoint 0: start */
    path.waypoints[0].x        = 0.0f;
    path.waypoints[0].y        = 0.0f;
    path.waypoints[0].theta    = 0.0f;
    path.waypoints[0].v_target = 300.0f;

    /* Waypoint 1: 500 mm ahead */
    path.waypoints[1].x        = 500.0f;
    path.waypoints[1].y        = 0.0f;
    path.waypoints[1].theta    = 0.0f;
    path.waypoints[1].v_target = 300.0f;

    /* Waypoint 2: 1000 mm ahead */
    path.waypoints[2].x        = 1000.0f;
    path.waypoints[2].y        = 0.0f;
    path.waypoints[2].theta    = 0.0f;
    path.waypoints[2].v_target = 300.0f;

    return path;
}
