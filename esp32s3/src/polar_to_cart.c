/**
 * polar_to_cart.c
 * Module: Polar-to-Cartesian conversion for LiDAR scan points.
 * Board: ESP32-S3
 */

#include "polar_to_cart.h"
#include <math.h>

void polar_to_cart_convert(const lidar_scan_t *scan,
                           const pose_t       *pose,
                           point2f_t          *out_pts,
                           uint16_t           *out_count)
{
    if (!scan || !out_pts || !out_count || !pose) return;

    const float cos_t = cosf(pose->theta);
    const float sin_t = sinf(pose->theta);

    uint16_t n = 0;
    for (uint16_t i = 0; i < scan->count; i++) {
        float r = lidar_point_range_mm(&scan->points[i]);
        if (r <= 0.0f) continue;

        /* Polar → local Cartesian — LiDAR scans clockwise, trig expects CCW */
        float rad = -lidar_point_theta_deg(&scan->points[i]) * ((float)M_PI / 180.0f);
        float lx  = r * cosf(rad);
        float ly  = r * sinf(rad);

        /* Local → global using robot pose */
        out_pts[n].x         = (int16_t)lrintf(pose->x + lx * cos_t - ly * sin_t);
        out_pts[n].y         = (int16_t)lrintf(pose->y + lx * sin_t + ly * cos_t);
        out_pts[n].intensity = scan->points[i].intensity;
        n++;
    }

    *out_count = n;
}
