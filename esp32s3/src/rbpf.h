/**
 * rbpf.h
 * Module: Rao-Blackwellized Particle Filter (RBPF) for SLAM pose estimation.
 * Board: ESP32-S3
 * Maintains a set of weighted particles representing the robot pose distribution.
 * Performs prediction via odometry motion model and update via scan matching correction.
 */

#ifndef RBPF_H
#define RBPF_H

#include <stdint.h>
#include "../../types.h"

#ifdef USE_STUBS
#include "../stubs/pose_stub.h"
#endif

/** RBPF particle filter state. Holds up to 64 weighted pose hypotheses. */
typedef struct {
    pose_t  particles[64]; /**< Particle poses */
    float   weights[64];   /**< Normalized particle weights (sum to 1.0) */
    uint8_t n_particles;   /**< Active particle count */
} rbpf_state_t;

/**
 * Initialize the particle filter with n_particles uniform-weight particles at the origin.
 * @param state       Pointer to rbpf_state_t to initialize.
 * @param n_particles Number of particles to use (max 64).
 */
void rbpf_init(rbpf_state_t *state, uint8_t n_particles);

/**
 * Propagate all particles through the odometry motion model (prediction step).
 * Adds Gaussian noise according to the motion model covariance.
 * @param state Pointer to the current RBPF state.
 * @param odom  Pointer to the latest odometry measurement.
 */
void rbpf_predict(rbpf_state_t *state, const odom_t *odom);

/**
 * Weight particles using the scan-matching correction and resample if needed (update step).
 * @param state      Pointer to the current RBPF state.
 * @param correction Pointer to the pose correction from the scan matcher.
 */
void rbpf_update(rbpf_state_t *state, const pose_correction_t *correction);

/**
 * Return the pose of the highest-weight particle as the best estimate.
 * @param state Pointer to the current RBPF state (const, no modification).
 * @return pose_t of the best particle.
 */
pose_t rbpf_get_best_pose(const rbpf_state_t *state);

#endif /* RBPF_H */
