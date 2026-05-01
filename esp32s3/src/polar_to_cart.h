#ifndef POLAR_TO_CART_H
#define POLAR_TO_CART_H

#include <stdint.h>
#include "lidar_driver.h"   /* lidar_scan_t */
#include "../../types.h"    /* pose_t, point2f_t */

/* Convert a full LiDAR scan from polar to global Cartesian coordinates.
 * Output points are in the same unit as pose->x/y (mm). */
void polar_to_cart_convert(const lidar_scan_t *scan,
                           const pose_t       *pose,
                           point2f_t          *out_pts,
                           uint16_t           *out_count);

#endif /* POLAR_TO_CART_H */