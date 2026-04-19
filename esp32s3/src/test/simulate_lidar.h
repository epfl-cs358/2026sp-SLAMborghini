/**
 * simulate_lidar.h
 * Module: Synthetic LiDAR simulation for test-room exploration.
 * Board: ESP32-S3
 *
 * Enables realistic map-building without real LiDAR hardware:
 *   1. slam_map_init()            — start with only the initial free disk (unknown elsewhere)
 *   2. simulate_and_update_map()  — ray-cast through truth_map from current pose,
 *                                   mark free/occupied cells in slam_map
 *
 * Usage in app_main (Mode B):
 *   quadtree_map_t truth_map;   // full room walls — used as ground truth for rays
 *   quadtree_map_t slam_map;    // progressively revealed — shown on dashboard
 *   build_test_room(&truth_map, &pose);
 *   slam_map_init(&slam_map, &pose, ROOM_WIDTH_MM, ROOM_HEIGHT_MM, 50.0f, 2000.0f);
 *   // each cycle after dead_reckon_pose():
 *   simulate_and_update_map(&truth_map, &slam_map, &pose, 180, 6000.0f, 50.0f);
 *   wifi_dashboard_update(&slam_map, &pose);
 */

#ifndef SIMULATE_LIDAR_H
#define SIMULATE_LIDAR_H

#include <stdint.h>
#include "../../../types.h"
#include "quadtree_map.h"

/**
 * Initialise slam_map as mostly-unknown with a free disk of radius free_r_mm
 * around start_pose.  Everything outside the disk starts at log-odds 0 (unknown).
 *
 * @param slam_map    Uninitialised map to populate.
 * @param start_pose  Robot starting pose — centre of the free disk.
 * @param width_mm    Map width  (must match truth_map, e.g. ROOM_WIDTH_MM).
 * @param height_mm   Map height (must match truth_map, e.g. ROOM_HEIGHT_MM).
 * @param step_mm     Cell resolution (e.g. 50 mm).
 * @param free_r_mm   Radius of the initial free disk around start (e.g. 2000 mm).
 */
void slam_map_init(quadtree_map_t *slam_map,
                   const pose_t   *start_pose,
                   float width_mm, float height_mm,
                   float step_mm,  float free_r_mm);

/**
 * Simulate one 360° LiDAR scan from pose, ray-casting through truth_map.
 * For each beam:
 *   - Cells along the ray with no wall hit → marked free in slam_map (QT_MISS_DEC).
 *   - First cell where truth_map shows a wall → marked occupied in slam_map (QT_HIT_INC).
 *
 * @param truth_map   Full room map (walls pre-loaded via build_test_room).
 * @param slam_map    Map being built — updated in place.
 * @param pose        Current robot pose (world frame, mm).
 * @param n_beams     Number of LiDAR beams (e.g. 180 for 2° resolution).
 * @param max_mm      Maximum beam range in mm (e.g. 6000).
 * @param step_mm     Ray-march step in mm (e.g. 50 — matches cell resolution).
 */
void simulate_and_update_map(const quadtree_map_t *truth_map,
                             quadtree_map_t       *slam_map,
                             const pose_t         *pose,
                             uint16_t              n_beams,
                             float                 max_mm,
                             float                 step_mm);

#endif /* SIMULATE_LIDAR_H */
