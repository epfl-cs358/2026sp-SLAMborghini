/**
 * obstacle_classifier.c
 * Module: Obstacle classifier — assigns semantic classes to Cartesian LiDAR points.
 * Board: ESP32-S3
 * Implementation phase: stub (classification algorithm not yet implemented)
 */

#include "obstacle_classifier.h"

void obstacle_classifier_classify(const point2f_t *pts, uint16_t count,
                                  classified_point_t *out, uint16_t *out_count)
{
    // TODO: implement
    // 1. Cluster input points using DBSCAN or run-length encoding on sorted angles.
    // 2. For each cluster, compute geometric features (linearity, spread, intensity stats).
    // 3. Apply decision rules to assign semantic_class_t.
    // 4. Write results to out[], set *out_count.
    (void)pts;
    (void)count;
    (void)out;
    if (out_count) {
        *out_count = 0;
    }
}
