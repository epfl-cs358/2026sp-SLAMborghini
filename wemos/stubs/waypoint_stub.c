// STUB — simulates path waypoints for offline testing

/**
 * waypoint_stub.c
 * Module: Waypoint stub — simulates path waypoints for offline tests.
 * Board: Wemos D1 R32 (PC stub, no hardware required)
 * Used by test_stanley.
 */

#include "waypoint_stub.h"

/*
 * Scenario: waypoint 500 mm straight ahead along the X axis, heading = 0.
 * With robot at origin facing east, cross-track error = 0 and heading error = 0,
 * so the Stanley controller should return a steering angle ~= 0.
 * Used by: test_stanley
 */
waypoint_t waypoint_stub_ahead(void)
{
    waypoint_t wp;
    wp.x        = 500.0f;
    wp.y        = 0.0f;
    wp.theta    = 0.0f;
    wp.v_target = 300.0f;
    return wp;
}

/*
 * Scenario: waypoint 500 mm to the left (Y direction), heading = pi/2 (facing north).
 * With robot at origin facing east, there is a 90-degree heading error and a large
 * lateral offset — Stanley should return a significant positive (left) steering angle.
 * Used by: test_stanley
 */
waypoint_t waypoint_stub_turn(void)
{
    waypoint_t wp;
    wp.x        = 0.0f;
    wp.y        = 500.0f;
    wp.theta    = 1.57f; /* ~pi/2 */
    wp.v_target = 200.0f;
    return wp;
}
