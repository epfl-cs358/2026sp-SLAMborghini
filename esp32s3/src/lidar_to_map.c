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
                  float               step_mm,
                  map_dirty_rect_t   *out_dirty)
{
    if (out_dirty) out_dirty->valid = false;

    if (!map || !scan || !pose) return;
    if (scan->count == 0 || step_mm <= 0.0f) return;

    /* Pose sanity — corrupt pose would write garbage all over the map */
    if (!_valid(pose->x) || !_valid(pose->y) || !_valid(pose->theta)) return;

    /* Normalise theta to [-π, π] with fmodf (avoids while-loop drift) */
    float theta = fmodf(pose->theta, 2.0f * (float)M_PI);
    if (theta >  (float)M_PI) theta -= 2.0f * (float)M_PI;
    if (theta < -(float)M_PI) theta += 2.0f * (float)M_PI;
    const float cos_t = cosf(theta);
    const float sin_t = sinf(theta);

    /* Apply LiDAR extrinsic offset: sensor origin in world frame */
    const float x0 = pose->x + LIDAR_OFFSET_X_MM * cos_t - LIDAR_OFFSET_Y_MM * sin_t;
    const float y0 = pose->y + LIDAR_OFFSET_X_MM * sin_t + LIDAR_OFFSET_Y_MM * cos_t;

    const int64_t t_start = _now_us();

    /* Robot's own cell is definitely clear — mark it free so the frontier
     * detector's BFS can start here even after the robot has moved away
     * from the initial free-disk seed. Without this, the cell at the robot
     * position stays unknown (ray marching starts at t=step_mm, not t=0)
     * and the BFS exits immediately. */
    qt_update(map, x0, y0, QT_MISS_DEC);

    /* Seed dirty rect with robot position */
    if (out_dirty) {
        out_dirty->x_min = x0;
        out_dirty->y_min = y0;
        out_dirty->x_max = x0;
        out_dirty->y_max = y0;
        out_dirty->valid = true;
    }

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

        /* Polar (LiDAR frame) → robot frame → world frame, all in mm.
         * LIDAR_OFFSET_THETA_RAD rotates the beam within the sensor frame. */
        float rad = theta_deg * ((float)M_PI / 180.0f) + LIDAR_OFFSET_THETA_RAD;
        float lx  = r * cosf(rad);
        float ly  = r * sinf(rad);
        float ex  = x0 + lx * cos_t - ly * sin_t;
        float ey  = y0 + lx * sin_t + ly * cos_t;

        if (!_valid(ex) || !_valid(ey)) continue;

        /* Expand dirty rect to include this beam's endpoint.
         * All ray-step cells lie on the segment (x0,y0)→(ex,ey) so
         * the endpoints already bound the entire beam geometrically. */
        if (out_dirty) {
            if (ex < out_dirty->x_min) out_dirty->x_min = ex;
            if (ey < out_dirty->y_min) out_dirty->y_min = ey;
            if (ex > out_dirty->x_max) out_dirty->x_max = ex;
            if (ey > out_dirty->y_max) out_dirty->y_max = ey;
        }

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


/* ── Helper: interpolate a scalar with wrap-around for angles ─────────────── */
static float _lerp_angle(float a, float b, float alpha)
{
    float d = b - a;
    while (d >  (float)M_PI) d -= 2.0f * (float)M_PI;
    while (d < -(float)M_PI) d += 2.0f * (float)M_PI;
    return a + alpha * d;
}


void lidar_deskew_and_map(quadtree_map_t     *map,
                           const lidar_scan_t *scan,
                           const pose_t       *pre_pose,
                           int64_t             pre_time_us,
                           const pose_t       *post_pose,
                           int64_t             post_time_us,
                           float               max_range_mm,
                           float               step_mm,
                           map_dirty_rect_t   *out_dirty)
{
    if (out_dirty) out_dirty->valid = false;
    if (!map || !scan || !pre_pose || !post_pose) return;
    if (scan->count == 0 || step_mm <= 0.0f) return;
    if (!_valid(pre_pose->x)  || !_valid(pre_pose->y)  || !_valid(pre_pose->theta))  return;
    if (!_valid(post_pose->x) || !_valid(post_pose->y) || !_valid(post_pose->theta)) return;

    const float dt_total = (float)(post_time_us - pre_time_us);
    /* If timestamps are degenerate, fall back to static pre_pose */
    const bool has_motion = (dt_total > 1.0f);

    const int64_t t_start = _now_us();

    for (uint16_t i = 0; i < scan->count; i++) {

        if ((i & (LIDAR_TO_MAP_WATCHDOG_CHK - 1)) == 0 && i > 0) {
            if (_now_us() - t_start > LIDAR_TO_MAP_WATCHDOG_US) break;
        }

        float r = scan->points[i].r_mm;
        if (r < 100.0f || r > max_range_mm) continue;
        if (scan->points[i].intensity == 0) continue;

        float theta_deg = scan->points[i].theta_deg;
        if (!_valid(theta_deg)) continue;

        /* Interpolate robot pose at this beam's capture time */
        float alpha = 0.0f;
        if (has_motion) {
            alpha = (float)((int64_t)scan->points[i].timestamp_us - pre_time_us) / dt_total;
            if (alpha < 0.0f) alpha = 0.0f;
            if (alpha > 1.0f) alpha = 1.0f;
        }

        float px    = pre_pose->x     + alpha * (post_pose->x     - pre_pose->x);
        float py    = pre_pose->y     + alpha * (post_pose->y     - pre_pose->y);
        float pth   = _lerp_angle(pre_pose->theta, post_pose->theta, alpha);
        float cos_t = cosf(pth);
        float sin_t = sinf(pth);

        /* Sensor origin in world frame (extrinsic offset) */
        float sx = px + LIDAR_OFFSET_X_MM * cos_t - LIDAR_OFFSET_Y_MM * sin_t;
        float sy = py + LIDAR_OFFSET_X_MM * sin_t + LIDAR_OFFSET_Y_MM * cos_t;

        /* Beam endpoint in world frame */
        float rad = theta_deg * ((float)M_PI / 180.0f) + LIDAR_OFFSET_THETA_RAD;
        float lx  = r * cosf(rad);
        float ly  = r * sinf(rad);
        float ex  = sx + lx * cos_t - ly * sin_t;
        float ey  = sy + lx * sin_t + ly * cos_t;

        if (!_valid(ex) || !_valid(ey)) continue;

        if (out_dirty) {
            if (!out_dirty->valid) {
                out_dirty->x_min = sx; out_dirty->y_min = sy;
                out_dirty->x_max = sx; out_dirty->y_max = sy;
                out_dirty->valid = true;
            }
            if (ex < out_dirty->x_min) out_dirty->x_min = ex;
            if (ey < out_dirty->y_min) out_dirty->y_min = ey;
            if (ex > out_dirty->x_max) out_dirty->x_max = ex;
            if (ey > out_dirty->y_max) out_dirty->y_max = ey;
        }

        /* Mark robot cell free, then ray-march, then mark endpoint */
        qt_update(map, sx, sy, QT_MISS_DEC);
        float dx = (ex - sx) / r;
        float dy = (ey - sy) / r;
        for (float t = step_mm; t < r - step_mm; t += step_mm)
            qt_update(map, sx + dx * t, sy + dy * t, QT_MISS_DEC);
        qt_update(map, ex, ey, QT_HIT_INC);
    }
}
