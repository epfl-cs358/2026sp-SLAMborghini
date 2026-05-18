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

static inline bool _valid(float v) { return isfinite(v); }

/* ── Angular delta filter state ────────────────────────────────────────────
 * 360 buckets (1°/bucket).  s_delta_ref[b] = last processed range at that
 * angle in mm; 0.0f = no-return.  s_delta_valid is false until the first
 * complete scan has populated all buckets. */
#define DELTA_BUCKETS 360
static float s_delta_ref[DELTA_BUCKETS];
static bool  s_delta_valid = false;

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

        float theta_deg = scan->points[i].theta_deg;
        if (!_valid(theta_deg)) continue;

        /* Angular delta filter — skip beams whose range barely changed */
        int bucket = (int)(theta_deg + 0.5f) % DELTA_BUCKETS;
        if (bucket < 0) bucket += DELTA_BUCKETS;
        float prev_r = s_delta_valid ? s_delta_ref[bucket] : -1.0f;
        s_delta_ref[bucket] = r;
        if (s_delta_valid) {
            bool same_range = (r > 0.0f && prev_r > 0.0f
                               && fabsf(r - prev_r) < LIDAR_DELTA_MM);
            if (same_range) {
                /* Reinforce stable obstacle cell against no-return MISS erosion.
                 * Skip the MISS march (free space hasn't changed) but still apply
                 * HIT so the cell stays at max value rather than drifting to unknown. */
                float rad2 = -theta_deg * ((float)M_PI / 180.0f) + LIDAR_OFFSET_THETA_RAD;
                float c2 = cosf(rad2), s2 = sinf(rad2);
                float ex2 = x0 + (c2 * cos_t - s2 * sin_t) * r;
                float ey2 = y0 + (c2 * sin_t + s2 * cos_t) * r;
                if (_valid(ex2) && _valid(ey2))
                    qt_update(map, ex2, ey2, QT_HIT_INC);
                continue;
            }
        }

        float rad     = -theta_deg * ((float)M_PI / 180.0f) + LIDAR_OFFSET_THETA_RAD;
        float cos_rad = cosf(rad);
        float sin_rad = sinf(rad);
        /* Unit vector in world frame — valid even when r == 0 */
        float ux = cos_rad * cos_t - sin_rad * sin_t;
        float uy = cos_rad * sin_t + sin_rad * cos_t;

        /* Determine march distance and whether this beam ends at an obstacle.
         * Three cases:
         *   no return / beyond sensor range → free to radius, no obstacle
         *   valid return beyond active zone → free to radius, no obstacle
         *   valid return within active zone → free along ray, obstacle at r */
        float march_to;
        bool  has_obstacle;
        if (r == 0.0f || r > max_range_mm) {
            march_to     = LIDAR_MAP_RADIUS_MM;
            has_obstacle = false;
        } else if (r > LIDAR_MAP_RADIUS_MM) {
            march_to     = LIDAR_MAP_RADIUS_MM;
            has_obstacle = false;
        } else {
            march_to     = r;
            has_obstacle = true;
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

        /* Uniform gentle miss for all beams (−1 vs HIT +30 = 30:1 ratio).
         * Using QT_MISS_DEC (−2) for finite-range beams gave a 15:1 ratio that
         * was insufficient: N adjacent no-return beams erode boundary obstacle
         * cells faster than the direct-beam HIT can reinforce them. */
        for (float t = step_mm; t < march_to - LIDAR_ENDPOINT_GUARD_MM; t += step_mm)
            qt_update(map, x0 + ux * t, y0 + uy * t, (int8_t)(-1));

        if (has_obstacle)
            qt_update(map, ex, ey, QT_HIT_INC);
    }

    s_delta_valid = true;
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

        /* ── Angular delta filter ──────────────────────────────────────────
         * Map the beam to a 1°-wide bucket and compare to the previous scan.
         * Unchanged beams (within LIDAR_DELTA_MM) are skipped entirely — no
         * ray march, no map write.  The reference is always updated so the
         * next scan compares against the freshest reading. */
        int bucket = (int)(theta_deg + 0.5f) % DELTA_BUCKETS;
        if (bucket < 0) bucket += DELTA_BUCKETS;

        float prev_r = s_delta_valid ? s_delta_ref[bucket] : -1.0f;
        s_delta_ref[bucket] = r;   /* update reference regardless of skip */

        if (s_delta_valid) {
            bool same_range = (r > 0.0f && prev_r > 0.0f
                               && fabsf(r - prev_r) < LIDAR_DELTA_MM);
            if (same_range) {
                /* Reinforce stable obstacle cell against no-return MISS erosion.
                 * Skip the MISS march (free space unchanged) but still apply HIT
                 * so the cell stays at max value rather than drifting to unknown. */
                float rad2 = -theta_deg * ((float)M_PI / 180.0f) + LIDAR_OFFSET_THETA_RAD;
                float c2 = cosf(rad2), s2 = sinf(rad2);
                float ex2 = sx + (c2 * cos_t - s2 * sin_t) * r;
                float ey2 = sy + (c2 * sin_t + s2 * cos_t) * r;
                if (_valid(ex2) && _valid(ey2))
                    qt_update(map, ex2, ey2, QT_HIT_INC);
                continue;
            }
        }

        float rad     = -theta_deg * ((float)M_PI / 180.0f) + LIDAR_OFFSET_THETA_RAD;
        float cos_rad = cosf(rad);
        float sin_rad = sinf(rad);
        float ux = cos_rad * cos_t - sin_rad * sin_t;
        float uy = cos_rad * sin_t + sin_rad * cos_t;

        float march_to;
        bool  has_obstacle;
        if (r == 0.0f || r > max_range_mm) {
            march_to     = LIDAR_MAP_RADIUS_MM;
            has_obstacle = false;
        } else if (r > LIDAR_MAP_RADIUS_MM) {
            march_to     = LIDAR_MAP_RADIUS_MM;
            has_obstacle = false;
        } else {
            march_to     = r;
            has_obstacle = true;
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

        qt_update(map, sx, sy, (int8_t)(-1));
        for (float t = step_mm; t < march_to - LIDAR_ENDPOINT_GUARD_MM; t += step_mm)
            qt_update(map, sx + ux * t, sy + uy * t, (int8_t)(-1));
        if (has_obstacle)
            qt_update(map, ex, ey, QT_HIT_INC);
    }

    s_delta_valid = true;
}
