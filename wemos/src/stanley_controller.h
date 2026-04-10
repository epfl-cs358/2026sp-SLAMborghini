/**
 * stanley_controller.h
 * Module: Stanley lateral controller for path tracking.
 * Board: Wemos D1 R32
 * Implements the Stanley method for computing the front-wheel steering angle
 * required to converge to the nearest point on the planned path.
 */

#ifndef STANLEY_CONTROLLER_H
#define STANLEY_CONTROLLER_H

#include "../../types.h"

#ifdef USE_STUBS
#include "../stubs/waypoint_stub.h"
#endif

/**
 * Compute the Stanley steering angle to track a path waypoint.
 * Combines heading error and cross-track error with a gain-scaled correction.
 * @param pose      Pointer to the current robot pose (world frame).
 * @param target    Pointer to the next waypoint on the planned path.
 * @param speed_mms Current forward speed in mm/s (used to scale cross-track term).
 * @return Steering angle in radians (positive = left / counter-clockwise).
 */
float stanley_compute_steer(const pose_t *pose, const waypoint_t *target, float speed_mms);

/**
 * Set the Stanley gain constant k.
 * Higher values give more aggressive cross-track error correction.
 * @param k Gain constant (typical range: 0.5 — 5.0).
 */
void stanley_set_gain(float k);

#endif /* STANLEY_CONTROLLER_H */
