/**
 * lidar_to_map.h
 * Bridge between raw LiDAR scan and the quadtree occupancy map.
 * Board: ESP32-S3
 */

#ifndef LIDAR_TO_MAP_H
#define LIDAR_TO_MAP_H

#include "../../types.h"
#include "quadtree_map.h"

/* ── LiDAR extrinsic calibration (sensor origin relative to robot centre) ─── *
 * Set to measured values once the sensor is mounted and measured.             *
 * Positive X_MM = sensor is forward of centre; positive Y_MM = port side.    *
 * THETA_RAD: sensor yaw relative to robot forward axis (positive = CCW).     */
#define LIDAR_OFFSET_X_MM      0.0f
#define LIDAR_OFFSET_Y_MM      0.0f
#define LIDAR_OFFSET_THETA_RAD 0.0f

/* Hard range filter — beams beyond this are silently dropped before any
 * processing.  Set to the sensor's reliable physical limit. */
#define LIDAR_PROCESS_RANGE_MM 3500.0f

/* Free-traversal endpoint guard (mm).
 * The ray march stops this far short of the obstacle so the last FREE step
 * can never land in the same 156 mm quadtree leaf as the HIT endpoint.
 * Required minimum: cell_size + step_mm = 156 + 80 = 236 mm.
 * Set to 260 mm (24 mm above minimum) to reduce the unknown band along wall
 * faces from ~300 mm to ~260 mm.  The narrower band means displaced wall
 * cells (from localisation jitter) are more likely to be reached by MISS
 * sweeps from adjacent beams, clearing ghost obstacles faster.
 * Still 24 mm above the hard minimum so worst-case cell alignment can never
 * cause a MISS to land in the same leaf as the HIT endpoint. */
#define LIDAR_ENDPOINT_GUARD_MM 260.0f

/* Active mapping radius (mm).  Only the disc of this radius around the
 * robot is written to the map each scan.  Beams that return beyond this
 * distance are still useful: they mark free space to the radius boundary
 * but do NOT register an obstacle.  Shrink to reduce cpu/memory per scan;
 * enlarge to map further ahead at planning time. */
#define LIDAR_MAP_RADIUS_MM    3000.0f

/* Angular delta filter threshold (mm).  Applied only to finite-range beams
 * (r > 0).  No-return beams (r == 0) are NEVER filtered — they must sweep
 * MISS updates through free space every scan so ghost HITs from noise spikes
 * get cleared within 1-2 scans rather than persisting indefinitely.
 *
 * Set BELOW the LiDAR shot-to-shot noise floor (~15 mm for RPLiDAR C1) so
 * that only truly static returns (< 8 mm change) skip the MISS march.
 * Beams near the noise floor (8–15 mm variation) now receive full MISS
 * sweeps each scan, clearing displaced ghost cells from localisation jitter
 * and transient reflections 2–3× faster than the old 15 mm threshold.
 * CPU cost is slightly higher but the watchdog caps any single scan to 100 ms. */
#define LIDAR_DELTA_MM          8.0f

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
 * Extrinsic offsets (LIDAR_OFFSET_*) are applied automatically.
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

/**
 * De-skew and integrate a scan using two-point pose interpolation.
 *
 * Each beam is assigned a world pose interpolated between pre_pose (at
 * pre_time_us) and post_pose (at post_time_us) using the per-point timestamp
 * stored by lidar_driver_read_scan().  Extrinsic offsets are applied per beam.
 *
 * @param map           Quadtree occupancy map to update.
 * @param scan          Scan with per-point timestamp_us set by lidar_driver.
 * @param pre_pose      Robot pose just before the scan (mm, rad).
 * @param pre_time_us   esp_timer_get_time() when pre_pose was captured.
 * @param post_pose     Robot pose just after the scan (mm, rad).
 * @param post_time_us  esp_timer_get_time() when post_pose was captured.
 * @param max_range_mm  Skip beams beyond this distance.
 * @param step_mm       Ray-march step size.
 * @param out_dirty     If non-NULL, filled with bbox of all written cells.
 */
void lidar_deskew_and_map(quadtree_map_t     *map,
                           const lidar_scan_t *scan,
                           const pose_t       *pre_pose,
                           int64_t             pre_time_us,
                           const pose_t       *post_pose,
                           int64_t             post_time_us,
                           float               max_range_mm,
                           float               step_mm,
                           map_dirty_rect_t   *out_dirty);

/**
 * Reset the angular delta-filter state.
 *
 * Call this in two situations:
 *   1. After qt_compact() wipes the map: the pool reset clears all free cells,
 *      but s_delta_ref[] still holds pre-compaction ranges.  Without a reset,
 *      stable-beam filtering (same_range == true) skips MISS sweeps on the
 *      next scan, so the restored-but-now-unknown free cells never get
 *      re-marked as free, and the frontier-detector BFS stays locked.
 *
 *   2. After a large scan-matcher correction (|dx|>5 mm, |dy|>5 mm, or
 *      |dθ|>0.05 rad): the corrected pose shifts beam endpoints, but
 *      s_delta_ref[] compares against pre-correction ranges.  When the shift
 *      is < LIDAR_DELTA_MM (8 mm) the filter sees same_range==true and
 *      skips the MISS sweep — ghost cells at the old pose positions persist
 *      indefinitely.  A one-scan full-sweep clears them immediately.
 */
void lidar_to_map_reset_delta_filter(void);

#endif /* LIDAR_TO_MAP_H */
