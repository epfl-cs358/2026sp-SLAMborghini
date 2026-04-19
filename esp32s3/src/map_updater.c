/**
 * map_updater.c
 * Module: Map updater — integrates classified point cloud into the quadtree map.
 * Board: ESP32-S3
 * Implementation phase: stub (coordinate transform not yet implemented)
 */

#include "map_updater.h"
#include <math.h>

void map_updater_update(quadtree_map_t *map, const classified_point_t *pts,
                        uint16_t count, const pose_t *pose)
{
    if (!map || !pts || !pose || count == 0) return;

    float cos_t = cosf(pose->theta);
    float sin_t = sinf(pose->theta);

    for (uint16_t i = 0; i < count; i++) {
        float wx = pose->x + pts[i].x * cos_t - pts[i].y * sin_t;
        float wy = pose->y + pts[i].x * sin_t + pts[i].y * cos_t;
        quadtree_map_insert(map, wx, wy, pts[i].cls);
    }
}
