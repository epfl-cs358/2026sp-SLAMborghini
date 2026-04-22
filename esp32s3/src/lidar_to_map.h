/**
 * lidar_to_map.h
 * Bridge between raw LiDAR scan and the quadtree occupancy map.
 * Board: ESP32-S3
 */

#ifndef LIDAR_TO_MAP_H
#define LIDAR_TO_MAP_H

#include "../../types.h"
#include "quadtree_map.h"

/**
 * Integrate one LiDAR scan into the occupancy map via ray marching (mm units).
 *
 * For each beam: marks free cells along the ray (QT_MISS_DEC) then marks the
 * endpoint as occupied (QT_HIT_INC).  All coordinates in mm; pose->theta in rad.
 *
 * @param map         Quadtree occupancy map to update.
 * @param scan        Raw scan from lidar_driver_read_scan() — r_mm + theta_deg.
 * @param pose        Robot pose at scan time (x,y in mm, theta in rad).
 * @param max_range_mm Skip beams beyond this distance (mm).
 * @param step_mm     Ray-march step size (mm); match to leaf-cell size for efficiency.
 */
void lidar_to_map(quadtree_map_t     *map,
                  const lidar_scan_t *scan,
                  const pose_t       *pose,
                  float               max_range_mm,
                  float               step_mm);

#endif /* LIDAR_TO_MAP_H */
