/**
 * obstacle_classifier.c
 * Module: Obstacle classifier — assigns semantic classes to Cartesian LiDAR points.
 * Board: ESP32-S3
 *
 * Algorithm:
 *   1. Sort input points by angle (atan2 in robot-local frame).
 *   2. Group into clusters on angular gaps > 5°.
 *   3. Per cluster: check spread and max perpendicular deviation from the
 *      chord connecting the first and last point.
 *      - Spread >= 150 mm AND deviation/spread < 0.15  → CLASS_WALL
 *      - >= 2 points but not wall-like                 → CLASS_OBSTACLE
 *      - single point                                  → CLASS_UNKNOWN
 */

#include "obstacle_classifier.h"
#include <math.h>
#include <string.h>

#define MAX_PTS             460u   /* RPLiDAR C1 max points per scan */
#define MAX_CLUSTERS        64u
#define GAP_THRESH_RAD      0.0873f  /* 5° — new cluster if angular gap exceeds this */
#define MIN_WALL_POINTS     5u
#define MIN_WALL_SPREAD_MM  150.0f
#define WALL_LINEAR_RATIO   0.15f    /* max(deviation)/spread below this → wall */

typedef struct { float angle; uint16_t idx; } sorted_pt_t;
typedef struct { uint16_t start; uint16_t count; } cluster_t;

/* Static BSS — avoids placing 2.7 KB on the 6 KB task stack. */
static sorted_pt_t s_sorted[MAX_PTS];

/* Perpendicular distance from point (px,py) to line through (ax,ay)-(bx,by). */
static float perp_dist(float ax, float ay, float bx, float by,
                       float px, float py)
{
    float dx = bx - ax, dy = by - ay;
    float len = sqrtf(dx * dx + dy * dy);
    if (len < 1.0f)
        return sqrtf((px - ax) * (px - ax) + (py - ay) * (py - ay));
    return fabsf((px - ax) * dy - (py - ay) * dx) / len;
}

void obstacle_classifier_classify(const point2f_t *pts, uint16_t count,
                                  classified_point_t *out, uint16_t *out_count)
{
    if (!pts || count == 0 || !out) {
        if (out_count) *out_count = 0;
        return;
    }

    uint16_t n = (count < MAX_PTS) ? count : (uint16_t)MAX_PTS;

    /* Step 1: build sorted index by angle */
    for (uint16_t i = 0; i < n; i++) {
        s_sorted[i].angle = atan2f(pts[i].y, pts[i].x);
        s_sorted[i].idx   = i;
    }

    /* Insertion sort — clusters are examined in angular order; n <= 460. */
    for (uint16_t i = 1; i < n; i++) {
        sorted_pt_t key = s_sorted[i];
        int16_t j = (int16_t)(i - 1);
        while (j >= 0 && s_sorted[j].angle > key.angle) {
            s_sorted[j + 1] = s_sorted[j];
            j--;
        }
        s_sorted[j + 1] = key;
    }

    /* Step 2: split into clusters on angular gaps */
    cluster_t clusters[MAX_CLUSTERS];
    uint16_t  nc = 0;

    clusters[0].start = 0;
    clusters[0].count = 1;
    nc = 1;

    for (uint16_t i = 1; i < n; i++) {
        float gap = s_sorted[i].angle - s_sorted[i - 1].angle;
        if (gap > GAP_THRESH_RAD && nc < MAX_CLUSTERS) {
            clusters[nc].start = i;
            clusters[nc].count = 1;
            nc++;
        } else {
            clusters[nc - 1].count++;
        }
    }

    /* Step 3: classify clusters and write output */
    uint16_t out_n = 0;

    for (uint16_t ci = 0; ci < nc; ci++) {
        uint16_t cstart = clusters[ci].start;
        uint16_t ccount = clusters[ci].count;

        semantic_class_t cls;

        if (ccount >= MIN_WALL_POINTS) {
            uint16_t ia = s_sorted[cstart].idx;
            uint16_t ib = s_sorted[cstart + ccount - 1u].idx;
            float ax = pts[ia].x, ay = pts[ia].y;
            float bx = pts[ib].x, by = pts[ib].y;
            float spread = sqrtf((bx - ax) * (bx - ax) + (by - ay) * (by - ay));

            if (spread >= MIN_WALL_SPREAD_MM) {
                float max_dev = 0.0f;
                for (uint16_t k = cstart + 1u; k < cstart + ccount - 1u; k++) {
                    float dev = perp_dist(ax, ay, bx, by,
                                          pts[s_sorted[k].idx].x,
                                          pts[s_sorted[k].idx].y);
                    if (dev > max_dev) max_dev = dev;
                }
                cls = (max_dev / spread < WALL_LINEAR_RATIO)
                      ? CLASS_WALL : CLASS_OBSTACLE;
            } else {
                cls = CLASS_OBSTACLE;
            }
        } else if (ccount >= 2u) {
            cls = CLASS_OBSTACLE;
        } else {
            cls = CLASS_UNKNOWN;
        }

        for (uint16_t k = cstart; k < cstart + ccount && out_n < n; k++) {
            uint16_t ik = s_sorted[k].idx;
            out[out_n].x   = pts[ik].x;
            out[out_n].y   = pts[ik].y;
            out[out_n].cls = cls;
            out_n++;
        }
    }

    if (out_count) *out_count = out_n;
}
