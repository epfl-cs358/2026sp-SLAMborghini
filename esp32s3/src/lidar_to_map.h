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
 * World-coordinate bounding box of cells written during one lidar_to_map() call.
 * Populated by lidar_to_map() when out_dirty != NULL.
 * valid=false means no beams were integrated (all filtered out or scan empty).
 */
typedef struct {
    float x_min, y_min;
    float x_max, y_max;
    bool  valid;
} map_dirty_rect_t;

/**
 * Integrate one LiDAR scan into the occupancy map via ray marching (mm units).
 *
 * For each beam: marks free cells along the ray (QT_MISS_DEC) then marks the
 * endpoint as occupied (QT_HIT_INC).  All coordinates in mm; pose->theta in rad.
 *
 * @param map          Quadtree occupancy map to update.
 * @param scan         Raw scan from lidar_driver_read_scan() — r_mm + theta_deg.
 * @param pose         Robot pose at scan time (x,y in mm, theta in rad).
 * @param max_range_mm Skip beams beyond this distance (mm).
 * @param step_mm      Ray-march step size (mm); match to leaf-cell size for efficiency.
 * @param out_dirty    If non-NULL, filled with the world bbox of all written cells.
 *                     Pass NULL to skip tracking (zero overhead).
 */
void lidar_to_map(quadtree_map_t     *map,
                  const lidar_scan_t *scan,
                  const pose_t       *pose,
                  float               max_range_mm,
                  float               step_mm,
                  map_dirty_rect_t   *out_dirty);

#endif /* LIDAR_TO_MAP_H */
