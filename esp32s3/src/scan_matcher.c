/**
 * scan_matcher.c — Correlative scan-to-map matching.
 * Board: ESP32-S3
 *
 * Algorithm:
 *   1. Precompute beam endpoints in robot frame (once per scan, ~90 points).
 *   2. Build a local occupancy grid (36×36 = 1296 cells, 100 mm resolution)
 *      centred on the odometry pose — ONE qt_query_const call per cell.
 *   3. Coarse grid search: 125 (dx,dy,dtheta) candidates scored via O(1)
 *      array lookups in the local grid.
 *   4. Fine grid search: 125 candidates centred on the coarse winner.
 *   5. Two-gate accept/reject:
 *        a) absolute hit-rate ≥ SM_MIN_HIT_RATE     (map sparse guard)
 *        b) improvement over odometry baseline       (drift guard)
 *        c) correction magnitude ≤ SM_MAX_XY_MM/deg (clamp guard)
 *
 * Performance vs old direct-quadtree approach:
 *   Old: 250 candidates × 90 beams = 22,500 qt_query_const calls → ~90 ms
 *   New: 1296 grid-build calls + 22,500 array lookups             → ~5  ms
 *
 * Why a local flat grid instead of direct quadtree queries?
 *   Each qt_query_const traverses a depth-7 pointer chain with random memory
 *   access — ~4 µs on ESP32-S3.  A uint8_t[36][36] fits in 1.3 KB and is
 *   hot in D-cache after the raster build, making lookups ~5 ns each.
 */

#include "scan_matcher.h"
#include "lidar_to_map.h"   /* LIDAR_OFFSET_THETA_RAD, LIDAR_MAP_RADIUS_MM */

#include <math.h>
#include <string.h>

#if defined(ESP_PLATFORM)
#  include "esp_timer.h"
#  define _sm_now_us() ((uint32_t)esp_timer_get_time())
#else
#  include <time.h>
   static inline uint32_t _sm_now_us(void) {
       struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts);
       return (uint32_t)((int64_t)ts.tv_sec * 1000000LL + ts.tv_nsec / 1000LL);
   }
#endif

/* ── Beam sampling ──────────────────────────────────────────────────────── */
#define SM_BEAM_STRIDE   5          /* sample every Nth beam ≈ 92/460 */
#define SM_MAX_SAMPLES   100        /* hard cap on precomputed endpoints */
#define SM_MIN_RANGE_MM  200.0f     /* ignore very close returns (robot body) */

/* ── Accept/reject thresholds ───────────────────────────────────────────── */
#define SM_MIN_HIT_RATE    0.10f    /* ≥10% absolute hit rate (map sparse guard) */
#define SM_MIN_IMPROVEMENT 3        /* must beat odometry baseline by ≥3 hits */
#define SM_MAX_XY_MM       80.0f    /* clamp: reject if |dx| or |dy| > 80 mm (search radius 120 mm) */
#define SM_MAX_DTHETA_DEG  10.0f    /* clamp: reject if |dθ| > 10° */

/* ── Coarse search grid ─────────────────────────────────────────────────── */
static const float k_c_xy[] = { -100.f, -50.f, 0.f, 50.f, 100.f };  /* mm */
static const float k_c_th[] = { -8.f, -4.f, 0.f, 4.f, 8.f };        /* degrees */

/* ── Fine search grid (offsets from coarse best) ────────────────────────── */
static const float k_f_xy[] = { -20.f, -10.f, 0.f, 10.f, 20.f };    /* mm */
static const float k_f_th[] = { -2.f, -1.f, 0.f, 1.f, 2.f };        /* degrees */

#define SM_NC (sizeof(k_c_xy)/sizeof(k_c_xy[0]))   /* 5 */
#define SM_NF (sizeof(k_f_xy)/sizeof(k_f_xy[0]))   /* 5 */

/* ── Local occupancy grid ───────────────────────────────────────────────── *
 * Covers ±GRID_HALF_MM around the odometry pose at GRID_CELL_MM resolution.*
 * Max beam range = 1500 mm + max coarse offset = 100 mm → need ≥ 1600 mm. *
 * 1800 mm half-width gives 200 mm margin on all sides.                     *
 *                                                                           *
 * Grid size: 36×36 = 1296 uint8_t = 1.3 KB  (fits hot in 16 KB D-cache)  */
#define GRID_HALF_MM  1800.0f
#define GRID_CELL_MM   100.0f
#define GRID_CELLS        36        /* 2 * 1800 / 100 */

static uint8_t s_grid[GRID_CELLS][GRID_CELLS];  /* 1296 B in BSS */
static float   s_grid_ox, s_grid_oy;            /* world coord of grid[0][0] centre */

/* ── Beam endpoint buffers (BSS, not re-entrant) ────────────────────────── */
static float s_bx[SM_MAX_SAMPLES];
static float s_by[SM_MAX_SAMPLES];
static int   s_nb = 0;

/* Build the local occupancy grid centred at (cx, cy). */
static void _build_grid(const QuadTreeMap *map, float cx, float cy)
{
    s_grid_ox = cx - GRID_HALF_MM + GRID_CELL_MM * 0.5f;
    s_grid_oy = cy - GRID_HALF_MM + GRID_CELL_MM * 0.5f;
    for (int j = 0; j < GRID_CELLS; j++) {
        float wy = s_grid_oy + j * GRID_CELL_MM;
        for (int i = 0; i < GRID_CELLS; i++) {
            s_grid[j][i] =
                (qt_query_const(map, s_grid_ox + i * GRID_CELL_MM, wy) > 0) ? 1u : 0u;
        }
    }
}

/* Score one candidate pose via O(1) grid lookups. */
static int _score(float cx, float cy, float cos_t, float sin_t)
{
    int hits = 0;
    for (int j = 0; j < s_nb; j++) {
        float wx = cx + s_bx[j] * cos_t - s_by[j] * sin_t;
        float wy = cy + s_bx[j] * sin_t + s_by[j] * cos_t;
        /* Nearest-grid-cell lookup; out-of-bounds cast to large uint → skip. */
        int gi = (int)((wx - s_grid_ox) / GRID_CELL_MM + 0.5f);
        int gj = (int)((wy - s_grid_oy) / GRID_CELL_MM + 0.5f);
        if ((unsigned)gi < (unsigned)GRID_CELLS &&
            (unsigned)gj < (unsigned)GRID_CELLS)
            hits += s_grid[gj][gi];
    }
    return hits;
}

bool scan_match(const QuadTreeMap   *map,
                const lidar_scan_t  *scan,
                const pose_t        *odom_pose,
                pose_t              *corrected,
                scan_match_result_t *result)
{
    memset(result, 0, sizeof(*result));
    *corrected = *odom_pose;

    if (!map || !scan || !odom_pose || !corrected || !result) return false;

    const uint32_t t0  = _sm_now_us();
    const float    d2r = (float)M_PI / 180.0f;

    /* ── 1. Precompute beam endpoints in robot frame ─────────────────────
     * Negate theta_deg: LiDAR angles are clockwise; trig expects CCW.     */
    s_nb = 0;
    for (uint16_t i = 0; i < scan->count && s_nb < SM_MAX_SAMPLES; i += SM_BEAM_STRIDE) {
        float r = scan->points[i].r_mm;
        if (r < SM_MIN_RANGE_MM || r > LIDAR_MAP_RADIUS_MM) continue;
        float rad = -scan->points[i].theta_deg * d2r + LIDAR_OFFSET_THETA_RAD;
        s_bx[s_nb] = r * cosf(rad);
        s_by[s_nb] = r * sinf(rad);
        s_nb++;
    }

    result->samples = s_nb;
    if (s_nb < 5) {
        result->elapsed_us = _sm_now_us() - t0;
        return false;
    }

    /* ── 2. Build local occupancy grid (1296 qt calls, replaces 22500) ── */
    _build_grid(map, odom_pose->x, odom_pose->y);

    /* ── 3. Baseline score at the odometry pose (no correction) ─────────
     * All subsequent candidates must beat this by SM_MIN_IMPROVEMENT.     */
    const float cos_base = cosf(odom_pose->theta);
    const float sin_base = sinf(odom_pose->theta);
    const int   baseline = _score(odom_pose->x, odom_pose->y, cos_base, sin_base);
    result->baseline = baseline;

    /* ── 4. Coarse search ────────────────────────────────────────────────
     * Initialise best = baseline so only genuine improvements are kept.   */
    int   best = baseline;
    float bdx  = 0.f, bdy = 0.f, bdt = 0.f;

    for (int ti = 0; ti < (int)SM_NC; ti++) {
        float dt     = k_c_th[ti] * d2r;
        float ctheta = odom_pose->theta + dt;
        float cos_t  = cosf(ctheta);
        float sin_t  = sinf(ctheta);
        for (int yi = 0; yi < (int)SM_NC; yi++) {
            float cy = odom_pose->y + k_c_xy[yi];
            for (int xi = 0; xi < (int)SM_NC; xi++) {
                int s = _score(odom_pose->x + k_c_xy[xi], cy, cos_t, sin_t);
                if (s > best) {
                    best = s;
                    bdx  = k_c_xy[xi];
                    bdy  = k_c_xy[yi];
                    bdt  = dt;
                }
            }
        }
    }

    /* ── 5. Fine search centred on coarse best ───────────────────────────
     * If nothing beat baseline, fine search refines around (0,0,0).       */
    int   fbest = best;
    float fbdx  = bdx, fbdy = bdy, fbdt = bdt;

    for (int ti = 0; ti < (int)SM_NF; ti++) {
        float dt     = bdt + k_f_th[ti] * d2r;
        float ctheta = odom_pose->theta + dt;
        float cos_t  = cosf(ctheta);
        float sin_t  = sinf(ctheta);
        for (int yi = 0; yi < (int)SM_NF; yi++) {
            float cy = odom_pose->y + bdy + k_f_xy[yi];
            for (int xi = 0; xi < (int)SM_NF; xi++) {
                float cx = odom_pose->x + bdx + k_f_xy[xi];
                int s = _score(cx, cy, cos_t, sin_t);
                if (s > fbest) {
                    fbest = s;
                    fbdx  = bdx + k_f_xy[xi];
                    fbdy  = bdy + k_f_xy[yi];
                    fbdt  = dt;
                }
            }
        }
    }

    result->score      = fbest;
    result->elapsed_us = _sm_now_us() - t0;

    /* ── 6. Three-gate accept / reject ───────────────────────────────────
     * Gate 1: absolute hit-rate (map sparse guard).
     * Gate 2: improvement over baseline (stationary-drift guard).
     * Gate 3: correction magnitude (clamp guard for false local optima).  */
    float hit_rate = (float)fbest / (float)s_nb;
    int   impr     = fbest - baseline;
    float dxy_max  = fabsf(fbdx) > fabsf(fbdy) ? fabsf(fbdx) : fabsf(fbdy);
    float dth_deg  = fabsf(fbdt) / d2r;

    if (hit_rate  < SM_MIN_HIT_RATE  ||
        impr      < SM_MIN_IMPROVEMENT ||
        dxy_max   > SM_MAX_XY_MM      ||
        dth_deg   > SM_MAX_DTHETA_DEG) {
        result->valid = false;
        return false;
    }

    result->dx_mm      = fbdx;
    result->dy_mm      = fbdy;
    result->dtheta_rad = fbdt;
    result->valid      = true;

    corrected->x     = odom_pose->x     + fbdx;
    corrected->y     = odom_pose->y     + fbdy;
    corrected->theta = odom_pose->theta + fbdt;

    /* Wrap theta to [-π, π] */
    while (corrected->theta >  (float)M_PI) corrected->theta -= 2.0f * (float)M_PI;
    while (corrected->theta < -(float)M_PI) corrected->theta += 2.0f * (float)M_PI;

    return true;
}
