/**
 * scan_matcher.h — Correlative scan matching against the quadtree occupancy map.
 * Board: ESP32-S3
 *
 * Two-level exhaustive grid search over (dx, dy, dtheta) finds the pose
 * correction that maximises the number of scan endpoints landing on occupied
 * cells.  Typical runtime: ~5 ms on ESP32-S3 @ 240 MHz.
 *
 * Search grid:
 *   Coarse: ±100 mm / 50 mm, ±8° / 4°  → 125 candidates
 *   Fine:   ±20 mm / 10 mm, ±2° / 1°   → 125 candidates centred on coarse best
 *   Sample: every 5th of ~460 beams     → ~90 sample points per candidate
 *
 * Correction is REJECTED when any of these fail:
 *   • hit-rate < 10%                  (map too sparse)
 *   • improvement over baseline < 5   (no real gain over odometry pose)
 *   • |dx| or |dy| > 80 mm           (implausibly large single-scan jump)
 *   • |dθ| > 12°                      (implausibly large single-scan rotation)
 *
 * Call scan_match() BEFORE lidar_to_map() so the corrected pose drives the map.
 * Thread-safety: uses static sample buffers — call from one task only.
 */

#ifndef SCAN_MATCHER_H
#define SCAN_MATCHER_H

#include <stdbool.h>
#include <stdint.h>
#include "quadtree_map.h"
#include "../../types.h"

/* Diagnostics returned by every scan_match() call */
typedef struct {
    float    dx_mm;        /* x correction applied  (corrected − raw odometry) */
    float    dy_mm;        /* y correction applied */
    float    dtheta_rad;   /* theta correction applied */
    int      score;        /* beam endpoints that landed on occupied cells (best candidate) */
    int      baseline;     /* beam endpoints at raw odometry pose (no correction) */
    int      samples;      /* total beams evaluated */
    bool     valid;        /* false → correction not applied */
    uint32_t elapsed_us;   /* wall time inside scan_match() for perf logging */
} scan_match_result_t;

/**
 * Refine odom_pose by correlating the scan against the existing quadtree map.
 *
 * @param map        Quadtree occupancy map (read-only, already integrated).
 * @param scan       Current LiDAR scan (r_mm + theta_cdeg per point).
 * @param odom_pose  Raw odometry pose — starting estimate (x,y mm, theta rad).
 * @param corrected  Output: scan-matched corrected pose.  Set to *odom_pose on failure.
 * @param result     Output: diagnostics (score, deltas, timing).  Always filled.
 * @return           true if a confident correction was found and applied.
 */
bool scan_match(const QuadTreeMap   *map,
                const lidar_scan_t  *scan,
                const pose_t        *odom_pose,
                pose_t              *corrected,
                scan_match_result_t *result);

#endif /* SCAN_MATCHER_H */
