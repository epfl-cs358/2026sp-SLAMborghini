/**
 * polar_to_cart.c
 * Module: Polar-to-Cartesian conversion for LiDAR scan points.
 * Board: ESP32-S3
 * Implementation phase: stub (math not yet implemented)
 */

#include "polar_to_cart.h"
#include <math.h>

void polar_to_cart_convert(const lidar_scan_t *scan, point2f_t *out_pts, uint16_t *out_count)
{
    if (!scan || !out_pts || !out_count) return;

    uint16_t n = 0;
    for (uint16_t i = 0; i < scan->count; i++) {
        float r = scan->points[i].r_mm;
        if (r <= 0.0f) continue;
        float rad = scan->points[i].theta_deg * (3.14159265f / 180.0f);
        out_pts[n].x         = r * cosf(rad);
        out_pts[n].y         = r * sinf(rad);
        out_pts[n].intensity = scan->points[i].intensity;
        n++;
    }
    *out_count = n;
}
