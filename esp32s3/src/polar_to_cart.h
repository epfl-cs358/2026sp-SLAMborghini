/**
 * polar_to_cart.h
 * Module: Polar-to-Cartesian conversion for LiDAR scan points.
 * Board: ESP32-S3
 * Converts a raw RPLiDAR polar scan into a Cartesian point cloud (point2f_t array).
 */

#ifndef POLAR_TO_CART_H
#define POLAR_TO_CART_H

#include <stdint.h>
#include "../../types.h"

/**
 * Convert a polar LiDAR scan into Cartesian point cloud coordinates.
 * Applies x = r * cos(theta), y = r * sin(theta) for each valid point.
 * @param scan      Pointer to the input polar scan (must not be NULL).
 * @param out_pts   Output array of Cartesian points (caller must provide capacity >= 460).
 * @param out_count Pointer to uint16_t that receives the number of points written.
 */
void polar_to_cart_convert(const lidar_scan_t *scan, point2f_t *out_pts, uint16_t *out_count);

#endif /* POLAR_TO_CART_H */
