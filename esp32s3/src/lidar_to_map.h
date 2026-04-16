#ifndef LIDAR_TO_MAP_H
#define LIDAR_TO_MAP_H

#include <stdint.h>
#include "quadtree_map.h"

/* 
 * lidar_to_map.h : LiDAR scan → quadtree occupancy update
 *
 * Pipeline (issue #32 scope) :
 *   1. Decode one full LiDAR scan (distance, angle, intensity)
 *   2. Convert polar → local Cartesian, then to world frame
 *   3. Ray-trace each beam :
 *        • crossed cells  →  qt_update(..., QT_MISS_DEC)   (free)
 *        • endpoint cell  →  qt_update(..., QT_HIT_INC)    (occupied)
 * 
 */
typedef struct {
    float   angle_rad;   /* beam angle in the robot frame (radians)   */
    float   distance_m;  /* measured range in metres; 0 = invalid     */
    uint8_t intensity;   /* signal strength (0 = no return)           */
} LidarPoint;

/* Robot pose in the world frame.
 * theta is counter-clockwise from the positive x-axis (east).        */
typedef struct {
    float x;      /* metres */
    float y;      /* metres */
    float theta;  /* radians */
} RobotPose;

/*  Process one full LiDAR scan and update *map

 * map : initialised QuadTreeMap to update
 * scan : array of n_points LidarPoint readings
 * n_points : number of beams in this scan
 * pose : car pose at the time of the scan
 * max_range_m  : beams with distance_m > max_range_m are skipped(trop loin)
 * step_m : ray-march step size in metres
 * Recommended: half the minimum cell size.
 */

void lidar_to_map(QuadTreeMap *map,
                  const LidarPoint *scan,
                  uint16_t n_points,
                  const RobotPose *pose,
                  float max_range_m,
                  float step_m);

#endif 
