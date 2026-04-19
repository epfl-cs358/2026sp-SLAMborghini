/**
 * simulate_lidar.c
 * Module: Synthetic LiDAR simulation for test-room exploration.
 * Board: ESP32-S3
 */

#include "simulate_lidar.h"
#include <math.h>

/* Occupancy threshold — compat layer returns 230 for walls (log-odds > 0).
 * Any value above OCC_UNK_MAX (178) means the cell is occupied in truth_map. */
#define TRUTH_OCC_THRESH 178u

/* Two-PI constant — avoids repeated multiplication in the beam loop. */
#define TWO_PI 6.28318530f


/* ════════════════════════════════════════════════════════════════════════════
 * slam_map_init
 * ════════════════════════════════════════════════════════════════════════════ */
void slam_map_init(quadtree_map_t *slam_map,
                   const pose_t   *start_pose,
                   float width_mm, float height_mm,
                   float step_mm,  float free_r_mm)
{
    if (!slam_map || !start_pose) return;

    quadtree_map_init(slam_map, width_mm, height_mm, step_mm);

    float r2   = free_r_mm * free_r_mm;
    float cx   = start_pose->x;
    float cy   = start_pose->y;

    for (float dy = -free_r_mm; dy <= free_r_mm; dy += step_mm) {
        for (float dx = -free_r_mm; dx <= free_r_mm; dx += step_mm) {
            if (dx * dx + dy * dy <= r2) {
                quadtree_map_insert(slam_map, cx + dx, cy + dy, CLASS_FREE);
            }
        }
    }
}


/* ════════════════════════════════════════════════════════════════════════════
 * simulate_and_update_map
 *
 * Ray-casts n_beams evenly distributed beams from pose through truth_map.
 * Updates slam_map: free along ray, occupied at first wall hit.
 * ════════════════════════════════════════════════════════════════════════════ */
void simulate_and_update_map(const quadtree_map_t *truth_map,
                             quadtree_map_t       *slam_map,
                             const pose_t         *pose,
                             uint16_t              n_beams,
                             float                 max_mm,
                             float                 step_mm)
{
    if (!truth_map || !slam_map || !pose) return;
    if (n_beams == 0 || step_mm <= 0.0f || max_mm <= 0.0f) return;

    const float x0          = pose->x;
    const float y0          = pose->y;
    const float angle_step  = TWO_PI / (float)n_beams;

    const float xmin = truth_map->x_min;
    const float xmax = truth_map->x_max;
    const float ymin = truth_map->y_min;
    const float ymax = truth_map->y_max;

    for (uint16_t b = 0; b < n_beams; b++) {
        float angle = pose->theta + (float)b * angle_step;
        float cos_a = cosf(angle);
        float sin_a = sinf(angle);

        for (float t = step_mm; t <= max_mm; t += step_mm) {
            float wx = x0 + t * cos_a;
            float wy = y0 + t * sin_a;

            /* Stop ray at map boundary */
            if (wx < xmin || wx >= xmax || wy < ymin || wy >= ymax) break;

            uint8_t truth_val = quadtree_map_query(truth_map, wx, wy);

            if (truth_val > TRUTH_OCC_THRESH) {
                /* Wall hit — mark occupied in slam_map and stop beam */
                qt_update(slam_map, wx, wy, QT_HIT_INC);
                break;
            }

            /* Free space — mark traversed */
            qt_update(slam_map, wx, wy, QT_MISS_DEC);
        }
    }
}
