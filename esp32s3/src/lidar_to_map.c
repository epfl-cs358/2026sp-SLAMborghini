#include "lidar_to_map.h"
#include <math.h>

#if defined(ESP_PLATFORM)
#include "esp_timer.h"
#include "esp_attr.h"   /* IRAM_ATTR */
#define _now_us() esp_timer_get_time()
#else
#define IRAM_ATTR       /* host builds: no-op */
#include <time.h>
static inline int64_t _now_us(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000000LL + ts.tv_nsec / 1000LL;
}
#endif

#define LIDAR_TO_MAP_WATCHDOG_US  100000LL   /* 100 ms per scan */
#define LIDAR_TO_MAP_WATCHDOG_CHK 32          /* check timer every N beams */
#define LIDAR_HIT_SUPPORT_SPAN    3           /* neighbouring beams to check */
#define LIDAR_HIT_SUPPORT_DELTA_MM 280.0f     /* max range difference for same surface */

static inline bool _valid(float v) { return isfinite(v); }

static bool _hit_has_neighbour_support(const lidar_scan_t *scan, uint16_t idx, float r)
{
    for (int off = 1; off <= LIDAR_HIT_SUPPORT_SPAN; off++) {
        if (idx >= (uint16_t)off) {
            float rn = scan->points[idx - (uint16_t)off].r_mm;
            if (rn >= 100.0f && rn <= LIDAR_PROCESS_RANGE_MM &&
                fabsf(rn - r) <= LIDAR_HIT_SUPPORT_DELTA_MM) {
                return true;
            }
        }
        uint16_t j = (uint16_t)(idx + (uint16_t)off);
        if (j < scan->count) {
            float rn = scan->points[j].r_mm;
            if (rn >= 100.0f && rn <= LIDAR_PROCESS_RANGE_MM &&
                fabsf(rn - r) <= LIDAR_HIT_SUPPORT_DELTA_MM) {
                return true;
            }
        }
    }
    return false;
}

IRAM_ATTR void lidar_to_map(quadtree_map_t     *map,
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

    /* Seed dirty rect with robot position; valid is set on the first processed beam */
    if (out_dirty) {
        out_dirty->x_min = x0;
        out_dirty->y_min = y0;
        out_dirty->x_max = x0;
        out_dirty->y_max = y0;
    }

    for (uint16_t i = 0; i < scan->count; i++) {

        /* Watchdog: bail out if we've spent too long on this scan */
        if ((i & (LIDAR_TO_MAP_WATCHDOG_CHK - 1)) == 0 && i > 0) {
            if (_now_us() - t_start > LIDAR_TO_MAP_WATCHDOG_US) break;
        }

        float r = scan->points[i].r_mm;
        /* Self-hit: driver filters these, but be defensive */
        if (r > 0.0f && r < 100.0f) continue;
        /* No return (reflective/absorbing surface) — skip all map updates.
         * Sweeping 3000 mm of MISS for r=0 writes free-space through walls
         * that have not been mapped yet and corrupts cells behind them. */
        if (r == 0.0f) continue;

        float theta_deg = scan->points[i].theta_deg;
        if (!_valid(theta_deg)) continue;

        float rad     = -theta_deg * ((float)M_PI / 180.0f) + LIDAR_OFFSET_THETA_RAD;
        float cos_rad = cosf(rad);
        float sin_rad = sinf(rad);
        /* Unit vector in world frame */
        float ux = cos_rad * cos_t - sin_rad * sin_t;
        float uy = cos_rad * sin_t + sin_rad * cos_t;

        /* Determine march distance and whether this beam ends at an obstacle.
         * r == 0 is already filtered above; only r > 0 reaches here.
         * Two cases:
         *   valid return beyond active zone → free to radius, no obstacle
         *   valid return within active zone → free along ray, obstacle at r */
        float march_to;
        bool  has_obstacle;
        if (r > max_range_mm || r > LIDAR_MAP_RADIUS_MM) {
            march_to     = LIDAR_MAP_RADIUS_MM;
            has_obstacle = false;
        } else {
            march_to     = r;
            has_obstacle = true;
        }
        if (has_obstacle && !_hit_has_neighbour_support(scan, i, r)) {
            has_obstacle = false;
        }

        float ex = x0 + ux * march_to;
        float ey = y0 + uy * march_to;
        if (!_valid(ex) || !_valid(ey)) continue;

        if (out_dirty) {
            if (!out_dirty->valid) out_dirty->valid = true;
            if (ex < out_dirty->x_min) out_dirty->x_min = ex;
            if (ey < out_dirty->y_min) out_dirty->y_min = ey;
            if (ex > out_dirty->x_max) out_dirty->x_max = ex;
            if (ey > out_dirty->y_max) out_dirty->y_max = ey;
        }

        for (float t = step_mm; t < march_to - LIDAR_ENDPOINT_GUARD_MM; t += step_mm)
            qt_update(map, x0 + ux * t, y0 + uy * t, QT_MISS_DEC);

        if (has_obstacle)
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
        if (r > 0.0f && r < 100.0f) continue;
        if (r == 0.0f) continue;  /* no return — skip, same reason as lidar_to_map */

        float theta_deg = scan->points[i].theta_deg;
        if (!_valid(theta_deg)) continue;

        /* Interpolate robot pose at this beam's capture time.
         * Use uint32 subtraction from scan_start_us so the result is correct
         * even after esp_timer wraps at ~71 min (abs int64 comparison breaks). */
        float alpha = 0.0f;
        if (has_motion) {
            uint32_t beam_off  = scan->points[i].timestamp_us - scan->scan_start_us;
            float    scan_span = (scan->rotation_period_us > 0u)
                                 ? (float)scan->rotation_period_us : dt_total;
            alpha = (scan_span > 1.0f) ? ((float)beam_off / scan_span) : 0.0f;
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

        float rad     = -theta_deg * ((float)M_PI / 180.0f) + LIDAR_OFFSET_THETA_RAD;
        float cos_rad = cosf(rad);
        float sin_rad = sinf(rad);
        float ux = cos_rad * cos_t - sin_rad * sin_t;
        float uy = cos_rad * sin_t + sin_rad * cos_t;

        float march_to;
        bool  has_obstacle;
        if (r > max_range_mm || r > LIDAR_MAP_RADIUS_MM) {
            march_to     = LIDAR_MAP_RADIUS_MM;
            has_obstacle = false;
        } else {
            march_to     = r;
            has_obstacle = true;
        }
        if (has_obstacle && !_hit_has_neighbour_support(scan, i, r)) {
            has_obstacle = false;
        }

        float ex = sx + ux * march_to;
        float ey = sy + uy * march_to;
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

        /* Mark the sensor origin free (robot was here when this beam fired) */
        qt_update(map, sx, sy, QT_MISS_DEC);

        for (float t = step_mm; t < march_to - LIDAR_ENDPOINT_GUARD_MM; t += step_mm)
            qt_update(map, sx + ux * t, sy + uy * t, QT_MISS_DEC);

        if (has_obstacle)
            qt_update(map, ex, ey, QT_HIT_INC);
    }
}
