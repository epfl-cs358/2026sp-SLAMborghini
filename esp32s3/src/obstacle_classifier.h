/**
 * obstacle_classifier.h
 * Module: Obstacle classifier — assigns semantic classes to Cartesian LiDAR points.
 * Board: ESP32-S3
 * Analyses geometric and intensity properties of each point cloud cluster
 * to label points as wall, obstacle, glass, person, or unknown.
 */

#ifndef OBSTACLE_CLASSIFIER_H
#define OBSTACLE_CLASSIFIER_H

#include <stdint.h>
#include "../../types.h"

/**
 * Classify an array of Cartesian LiDAR points into semantic categories.
 * Writes results into out[], which must have capacity >= count.
 * @param pts       Input array of Cartesian points (robot-local frame).
 * @param count     Number of input points.
 * @param out       Output array of classified_point_t (caller-allocated, size >= count).
 * @param out_count Receives the number of classified points written to out.
 */
void obstacle_classifier_classify(const point2f_t *pts, uint16_t count,
                                  classified_point_t *out, uint16_t *out_count);

#endif /* OBSTACLE_CLASSIFIER_H */
