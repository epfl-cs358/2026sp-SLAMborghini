#include "lidar_to_map.h"
#include <math.h>

// Bridge between raw LiDAR scan and the quadtree occupancy map.
// For each beam: convert polar (angle, distance) to world (x, y)
// then update the map: ray = free, endpoint = occupied.

void lidar_to_map(QuadTreeMap       *map,
                  const LidarPoint  *scan,
                  uint16_t           n_points,
                  const RobotPose   *pose,
                  float              max_range_m,
                  float              step_m)
{
    // sanity checks - avoid crash on null pointer or bad step
    if (!map || !scan || !pose) return;
    if (n_points == 0 || step_m <= 0.0f) return;

    // car position and orientation in the world
    // cos/sin computed once - same for all beams in one scan (ESP32 optimization)
    const float x0    = pose->x;
    const float y0    = pose->y;
    const float cos_t = cosf(pose->theta);
    const float sin_t = sinf(pose->theta);

    for (uint16_t i = 0; i < n_points; i++) {

        // skip invalid beams: no return (0), too far, or negative
        float d = scan[i].distance_m;
        if (d <= 0.0f || d > max_range_m) continue;

        // polar -> local Cartesian (car frame)
        // ?????? i will be pluging polar_to_cart_convert() from #28 once types and units are aligned
        float lx = d * cosf(scan[i].angle_rad);
        float ly = d * sinf(scan[i].angle_rad);

        // local -> world frame (rotate by theta, then translate by car position)
        float ex = x0 + lx * cos_t - ly * sin_t;
        float ey = y0 + lx * sin_t + ly * cos_t;

        // unit vector along the beam
        float dx = (ex - x0) / d;
        float dy = (ey - y0) / d;

        // ray-march: mark every cell along the beam as free
        // stop before endpoint to avoid overwriting it with a free update
        float t = 0.0f;
        float stop = d - step_m;
        while (t < stop) {
            qt_update(map, x0 + dx * t, y0 + dy * t, QT_MISS_DEC);
            t += step_m;
        }

        // mark endpoint as occupied (obstacle confirmed here)
        qt_update(map, ex, ey, QT_HIT_INC);
    }
}