/**
 * waypoint_stub.h
 * Module: Waypoint stub — simulates path waypoints for offline tests.
 * Board: Wemos D1 R32 (PC stub, no hardware required)
 * Used by test_stanley.
 */

// STUB — simulates path waypoints for offline testing

#ifndef WAYPOINT_STUB_H
#define WAYPOINT_STUB_H

#include "../../types.h"

/**
 * Returns a waypoint 500 mm directly ahead (positive X) with heading 0 and speed 300 mm/s.
 * Simulates a straight corridor waypoint — expected steering output should be ~0.
 * Used by: test_stanley
 */
waypoint_t waypoint_stub_ahead(void);

/**
 * Returns a waypoint 500 mm to the left (positive Y) with heading pi/2 and speed 200 mm/s.
 * Simulates the robot needing to make a 90-degree left turn — expected steering should be nonzero.
 * Used by: test_stanley
 */
waypoint_t waypoint_stub_turn(void);

#endif /* WAYPOINT_STUB_H */
