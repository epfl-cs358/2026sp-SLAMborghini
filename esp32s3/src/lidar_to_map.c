#include "lidar_to_map.h"
#include <math.h>

#if defined(ESP_PLATFORM)
#include "esp_timer.h"
#define _now_us() esp_timer_get_time()
#else
#include <time.h>
static inline int64_t _now_us(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000000LL + ts.tv_nsec / 1000LL;
}
#endif

#define LIDAR_TO_MAP_WATCHDOG_US  100000LL   /* 100 ms per scan */
#define LIDAR_TO_MAP_WATCHDOG_CHK 32          /* check timer every N beams */

static inline bool _valid(float v) { return isfinite(v); }

void lidar_to_map(quadtree_map_t     *map,
                  const lidar_scan_t *scan,
                  const pose_t       *pose,
                  float               max_range_mm,
                  float               step_mm)
{
    if (!map || !scan || !pose) return;
    if (scan->count == 0 || step_mm <= 0.0f) return;

    /* Pose sanity — corrupt pose would write garbage all over the map */
    if (!_valid(pose->x) || !_valid(pose->y) || !_valid(pose->theta)) return;

    const float x0    = pose->x;
    const float y0    = pose->y;
    /* Normalise theta to [-π, π] with fmodf (avoids while-loop drift) */
    float theta = fmodf(pose->theta, 2.0f * (float)M_PI);
    if (theta >  (float)M_PI) theta -= 2.0f * (float)M_PI;
    if (theta < -(float)M_PI) theta += 2.0f * (float)M_PI;
    const float cos_t = cosf(theta);
    const float sin_t = sinf(theta);

    const int64_t t_start = _now_us();

    /* Robot's own cell is definitely clear — mark it free so the frontier
     * detector's BFS can start here even after the robot has moved away
     * from the initial free-disk seed. Without this, the cell at the robot
     * position stays unknown (ray marching starts at t=step_mm, not t=0)
     * and the BFS exits immediately. */
    qt_update(map, x0, y0, QT_MISS_DEC);

    for (uint16_t i = 0; i < scan->count; i++) {

        /* Watchdog: bail out if we've spent too long on this scan */
        if ((i & (LIDAR_TO_MAP_WATCHDOG_CHK - 1)) == 0 && i > 0) {
            if (_now_us() - t_start > LIDAR_TO_MAP_WATCHDOG_US) break;
        }

        float r = scan->points[i].r_mm;
        if (r < 100.0f || r > max_range_mm) continue;

        float theta_deg = scan->points[i].theta_deg;
        if (!_valid(theta_deg)) continue;

        /* Quality filter — zero intensity = noisy/invalid return */
        if (scan->points[i].intensity == 0) continue;

        /* Polar (LiDAR frame) → robot frame → world frame, all in mm */
        float rad = theta_deg * ((float)M_PI / 180.0f);
        float lx  = r * cosf(rad);
        float ly  = r * sinf(rad);
        float ex  = x0 + lx * cos_t - ly * sin_t;
        float ey  = y0 + lx * sin_t + ly * cos_t;

        if (!_valid(ex) || !_valid(ey)) continue;

        /* Unit vector along beam */
        float dx = (ex - x0) / r;
        float dy = (ey - y0) / r;

        /* Ray-march: mark free space */
        for (float t = step_mm; t < r - step_mm; t += step_mm)
            qt_update(map, x0 + dx * t, y0 + dy * t, QT_MISS_DEC);

        /* Endpoint: mark obstacle */
        qt_update(map, ex, ey, QT_HIT_INC);
    }
}
