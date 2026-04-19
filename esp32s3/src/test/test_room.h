/**
 * test_room.h
 * Module: Pre-loaded test room for hardware validation.
 * Board: ESP32-S3
 *
 * Provides build_test_room(), which replaces the empty quadtree_map_init()
 * call during the hardware test phase.  It populates the map with real indoor
 * geometry from the Mexico GFS dataset so that frontier_detector_detect() has
 * something meaningful to work with before the LiDAR driver is implemented.
 *
 * Usage (in app_main):
 *   quadtree_map_t map;
 *   pose_t pose = {0};
 *   build_test_room(&map, &pose);   // replaces quadtree_map_init()
 *
 * After this call:
 *   • map contains all wall cells from the dataset (CLASS_WALL).
 *   • A FREE disk of radius BUILD_FREE_DISK_MM is stamped around pose so
 *     the frontier-detector BFS has a traversable seed region.
 *   • pose is set to the robot's starting position in the dataset.
 *
 * Prerequisite (CRITICAL):
 *   quadtree_map_query() must return 128 (OCC_UNKNOWN) for unvisited cells,
 *   not 0.  The current stub returns 0, which makes frontier_detector see
 *   the entire map as free.  One-line fix in quadtree_map.c:
 *       - return 0;
 *       + return 128;   // OCC_UNKNOWN
 */

#ifndef TEST_ROOM_H
#define TEST_ROOM_H

#include "../../../types.h"
#include "quadtree_map.h"

/** Radius (mm) of the free-space disk stamped around the start pose. */
#define BUILD_FREE_DISK_MM  2000.0f   /* 2 m radius — puts the first frontier
                                       * ~2000 mm from start, reachable in
                                       * ~10 planning cycles at 200 mm/step.
                                       * (was 4000 mm when quadtree was a stub) */

/**
 * Populate map with the pre-parsed Mexico GFS room data and set start pose.
 *
 * Calls only the public quadtree API:
 *   quadtree_map_init()    — called once with the dataset's bounding box
 *   quadtree_map_insert()  — called for each wall cell and each free cell
 *
 * @param map        Uninitialised quadtree_map_t to populate.
 * @param start_pose pose_t to initialise from the first GFS pose.
 */
void build_test_room(quadtree_map_t *map, pose_t *start_pose);

#endif /* TEST_ROOM_H */
