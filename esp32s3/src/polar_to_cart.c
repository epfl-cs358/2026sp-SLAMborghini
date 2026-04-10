/**
 * polar_to_cart.c
 * Module: Polar-to-Cartesian conversion for LiDAR scan points.
 * Board: ESP32-S3
 * Implementation phase: stub (math not yet implemented)
 */

#include "polar_to_cart.h"

void polar_to_cart_convert(const lidar_scan_t *scan, point2f_t *out_pts, uint16_t *out_count)
{
    // TODO: implement
    // For each point in scan->points[i]:
    //   float rad = scan->points[i].theta_deg * (M_PI / 180.0f);
    //   out_pts[i].x = scan->points[i].r_mm * cosf(rad);
    //   out_pts[i].y = scan->points[i].r_mm * sinf(rad);
    //   out_pts[i].intensity = scan->points[i].intensity;
    // Set *out_count = scan->count;
    (void)scan;
    (void)out_pts;
    if (out_count) {
        *out_count = 0;
    }
}
