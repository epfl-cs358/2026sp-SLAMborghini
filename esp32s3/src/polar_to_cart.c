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
        float r = scan->points[i].r_mm;
        if (r <= 0.0f) continue;

        /* Polar → local Cartesian — LiDAR scans clockwise, trig expects CCW */
        float rad = -scan->points[i].theta_deg * ((float)M_PI / 180.0f);
        float lx  = r * cosf(rad);
        float ly  = r * sinf(rad);

        /* Local → global using robot pose */
        out_pts[n].x         = pose->x + lx * cos_t - ly * sin_t;
        out_pts[n].y         = pose->y + lx * sin_t + ly * cos_t;
        out_pts[n].intensity = scan->points[i].intensity;
        n++;
    }

    *out_count = n;
}