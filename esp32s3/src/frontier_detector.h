/**
 * frontier_detector.h
 * Module: Frontier-based exploration detector.
 * Board: ESP32-S3
 * Identifies boundaries between free (known) and unknown space in the occupancy map.
 * Provides the next exploration target as the nearest large frontier.
 */

#ifndef FRONTIER_DETECTOR_H
#define FRONTIER_DETECTOR_H

#include "../../types.h"
#include "quadtree_map.h"

#ifdef USE_STUBS
#include "../stubs/frontier_stub.h"
#endif

/**
 * Scan the quadtree map for frontier clusters using Wavefront Frontier Detection (WFD).
 * BFS expands outward from the robot pose through free cells only.
 * Any free cell with at least one unknown 4-connected neighbour is a frontier seed.
 * Connected seeds are flood-filled into clusters; each cluster's target point is the
 * cluster cell nearest to the centroid (guaranteed reachable, never inside an obstacle).
 * A safety spiral nudges the target away from any occupied neighbour.
 *
 * @param map        Pointer to the current quadtree map (const).
 * @param robot_pose Current robot pose — BFS starts here.
 * @return frontier_list_t containing up to 32 frontier clusters.
 */
frontier_list_t frontier_detector_detect(const quadtree_map_t *map,
                                          const pose_t *robot_pose);

/**
 * Select the best frontier to explore next: the nearest frontier with the largest size.
 * @param list       Pointer to the frontier list returned by frontier_detector_detect().
 * @param robot_pose Pointer to the current robot pose for distance computation.
 * @return The best frontier_t, or a zero-initialized frontier if the list is empty.
 */
frontier_t frontier_detector_best(const frontier_list_t *list, const pose_t *robot_pose);

#endif /* FRONTIER_DETECTOR_H */
