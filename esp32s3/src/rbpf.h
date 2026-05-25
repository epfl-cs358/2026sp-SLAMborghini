/**
 * rbpf.h
 * Module: Rao-Blackwellized Particle Filter (RBPF) for SLAM pose estimation.
 * Board: ESP32-S3
 *
 * ── WHAT THIS IS ──────────────────────────────────────────────────────────
 * GMapping-style RBPF with an "improved proposal distribution":
 *
 *   Classic PF:   propose x from odometry → weight by p(scan | x, map)
 *   GMapping PF:  propose x from odometry + local scan matching
 *                 → weight by p(scan | x_best, map)
 *
 * The scan-matching step moves each particle to its locally-best matching
 * pose BEFORE weighting.  This is why 5–8 particles work as well as 100+ in
 * a naive PF.  It is the key contribution of Hähnel et al. / Grisetti et al.
 *
 * ── WHAT THIS IS NOT ──────────────────────────────────────────────────────
 * True FastSLAM 2.0 gives each particle its own map.  On ESP32-S3, 8 × 48 KB
 * = 384 KB exceeds available DRAM.  This implementation uses a SINGLE shared
 * QuadTreeMap for all particles (map is Rao-Blackwellized / marginalised).
 * The shared map is used read-only during scoring; it is written only ONCE per
 * scan, using `rbpf_get_best_pose()`.
 *
 * Per-particle maps via ancestry-tree sharing (path-copying QTNodes) are the
 * correct next step and would eliminate the residual double-wall problem, but
 * require restructuring quadtree_map.c and are deferred.
 *
 * ── INTEGRATION NOTE ──────────────────────────────────────────────────────
 * Map updates (lidar_deskew_and_map) MUST use rbpf_get_best_pose(), not
 * rbpf_get_mean_pose().  Writing the map at the mean pose accumulates the
 * same blur that RBPF is trying to prevent.  Use rbpf_get_mean_pose() only
 * for control commands and dashboard display.
 *
 * This module is intended to REPLACE the scan_match() + direct s_pose write
 * in task_lidar_slam.  rbpf_update() runs the same coarse+fine correlative
 * search as scan_matcher.c, but independently per particle.
 *
 * ── PIPELINE (one cycle) ──────────────────────────────────────────────────
 *   rbpf_predict(state, odom)            — motion model, spread particles
 *   rbpf_update(state, map, scan)        — local scan-match per particle,
 *                                          weight, normalise, resample
 *   lidar_deskew_and_map(rbpf_get_best_pose())  — map update
 *   rbpf_get_mean_pose()                 — pose for control / dashboard
 */

#ifndef RBPF_H
#define RBPF_H

#include <stdint.h>
#include <stdbool.h>
#include "../../types.h"
#include "quadtree_map.h"

#ifdef USE_STUBS
#include "../stubs/pose_stub.h"
#endif

/* Particle count.  5–8 is sufficient for indoor structured environments when
 * per-particle scan matching is used.  Increasing beyond 16 rarely helps and
 * multiplies the update cost proportionally. */
#define RBPF_MAX_PARTICLES  8

/** RBPF filter state — ~360 bytes total. */
typedef struct {
    pose_t   poses[RBPF_MAX_PARTICLES];    /**< Pose hypothesis per particle */
    float    weights[RBPF_MAX_PARTICLES];  /**< Normalised weights (sum to 1) */
    uint8_t  n;                            /**< Active particle count ≤ RBPF_MAX_PARTICLES */
    uint32_t resample_count;               /**< Total low-variance resamples since init */
} rbpf_state_t;

/** Diagnostics snapshot (no state mutation). */
typedef struct {
    float    n_eff;           /**< Effective particle count = 1/Σwᵢ²  (range: 1..n) */
    float    best_weight;     /**< Highest normalised particle weight */
    uint32_t resample_count;  /**< Total resamples since init */
} rbpf_diag_t;

/**
 * Initialise the filter.
 *
 * @param state         Filter to initialise.
 * @param n_particles   Particle count (1..RBPF_MAX_PARTICLES).  Pass 0 for max.
 * @param initial_pose  All particles start here; NULL → world origin.
 *                      Particles are jittered ±20 mm / ±3° (1-sigma) to cover
 *                      typical initial pose uncertainty.
 */
void rbpf_init(rbpf_state_t *state,
               uint8_t       n_particles,
               const pose_t *initial_pose);

/**
 * Prediction step — advance all particles through the differential-drive
 * motion model and inject per-particle Gaussian noise.
 *
 * Must be called once per odom packet BEFORE rbpf_update().
 *
 * @param state Filter state (modified in place).
 * @param odom  Latest odometry: linear_disp_mm, yaw_rate_imu (rad/s), dt_ms.
 */
void rbpf_predict(rbpf_state_t *state, const odom_t *odom);

/**
 * Update step — GMapping improved-proposal scan matching + weight + resample.
 *
 * For each particle:
 *   1. Runs a coarse (±100 mm/50 mm, ±8°/4°) + fine (±20 mm/10 mm, ±2°/1°)
 *      scan-to-map search centred on the particle's predicted pose.
 *   2. Moves the particle to the locally-best matched pose.
 *   3. Weights it by exp(k × best_score).
 * Then normalises and performs low-variance resampling when N_eff < N/2.
 *
 * Uses one flat 36×36 occupancy grid (1.3 KB) centred on the particle-cloud
 * centroid — built once per call then reused for all per-particle searches.
 * Scores are O(1) grid lookups; beam endpoints are pre-computed once per call.
 *
 * @param state  Filter state (modified in place: poses updated, weights updated).
 * @param map    Shared quadtree map (read-only).  Caller must hold s_map_mutex.
 * @param scan   Current 360° LiDAR scan.
 */
void rbpf_update(rbpf_state_t      *state,
                 const QuadTreeMap *map,
                 const lidar_scan_t *scan);

/**
 * Return the highest-weight particle pose.
 *
 * USE THIS for lidar_deskew_and_map() — inserts scan at the single most-likely
 * pose, avoiding the blur that writing at an averaged position would cause.
 */
pose_t rbpf_get_best_pose(const rbpf_state_t *state);

/**
 * Return the weighted mean pose.
 *
 * Smoother than best-particle for control and display.  Uses the circular mean
 * for heading (atan2 of weighted sin/cos sum) so wrap-around is correct.
 * Do NOT use this for map integration — see integration note above.
 */
pose_t rbpf_get_mean_pose(const rbpf_state_t *state);

/** Fill diagnostics without modifying filter state. */
void rbpf_get_diag(const rbpf_state_t *state, rbpf_diag_t *out);

#endif /* RBPF_H */
