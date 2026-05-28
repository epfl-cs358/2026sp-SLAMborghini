/**
 * rbpf.c
 * Module: Rao-Blackwellized Particle Filter for pose estimation.
 * Board: ESP32-S3
 *
 * Architecture note — shared map, not per-particle maps:
 *   True RBPF-SLAM (FastSLAM 2.0) requires each particle to maintain its own
 *   occupancy map.  At 48 KB/map × 16 particles = 768 KB that exceeds ESP32-S3
 *   DRAM.  This implementation shares the single QuadTreeMap across all particles
 *   and uses it only for likelihood scoring.  The result is MCL-style localisation
 *   once the map is seeded.  Per-particle maps can be added later by shrinking
 *   QT_POOL_SIZE and allocating N separate QuadTreeMap instances.
 */

#include "rbpf.h"
#include "lidar_to_map.h"  /* LIDAR_OFFSET_THETA_RAD, LIDAR_MAP_RADIUS_MM */

#include <math.h>
#include <string.h>

#if defined(ESP_PLATFORM)
#  include "esp_random.h"
static inline uint32_t _rand32(void) { return esp_random(); }
#else
#  include <stdlib.h>
static inline uint32_t _rand32(void) { return (uint32_t)rand(); }
#endif


/* ── Uniform [0,1] and standard-normal PRNG ────────────────────────────── */

static inline float _rand_f(void)
{
    /* Drop the sign bit of esp_random() so the cast to float is positive. */
    return (float)(_rand32() >> 1) / (float)0x7FFFFFFFu;
}

/* Box-Muller: produces pairs of standard-normal samples. */
static float _randn(void)
{
    static bool  s_spare_valid = false;
    static float s_spare;
    if (s_spare_valid) { s_spare_valid = false; return s_spare; }
    float u, v, s;
    do {
        u = _rand_f() * 2.0f - 1.0f;
        v = _rand_f() * 2.0f - 1.0f;
        s = u * u + v * v;
    } while (s >= 1.0f || s == 0.0f);
    float mul    = sqrtf(-2.0f * logf(s) / s);
    s_spare       = v * mul;
    s_spare_valid = true;
    return u * mul;
}


/* ── Motion noise model ──────────────────────────────────────────────────
 * Standard four-parameter model (Thrun et al., "Probabilistic Robotics"):
 *
 *   σ_trans = α_tt × |Δd| + α_tr × |Δθ|    (translational noise, mm)
 *   σ_rot   = α_rt × |Δd| + α_rr × |Δθ|    (rotational noise, rad)
 *
 * α_tt  — translation noise from translation (wheel slip, mm per mm)
 * α_tr  — translation noise from rotation    (turning radius error, mm per rad)
 * α_rt  — rotation noise from translation    (encoder misalignment, rad per mm)
 * α_rr  — rotation noise from rotation       (IMU scale factor, rad per rad)
 *
 * Tune α values on hardware by comparing RBPF spread vs. scan-match residual. */
#define ALPHA_TT   0.025f   /* 2.5 mm per 100 mm travel */
#define ALPHA_TR   2.0f     /* 2 mm per radian (50 mm per 25°) */
#define ALPHA_RT   0.004f   /* 0.4° per 100 mm = ~1.4°/m */
#define ALPHA_RR   0.05f    /* 5% angular scale error */

/* Skip noise injection when motion is negligible to avoid drift while static. */
#define MIN_TRANS_MM  1.0f
#define MIN_ROT_RAD   0.002f   /* ~0.1° */


/* ── Flat occupancy grid for particle scoring ────────────────────────────
 * Identical layout to scan_matcher.c — 36×36 cells, 100 mm resolution,
 * covering ±1800 mm around the particle cloud centre.  Built once per
 * rbpf_update() call (1296 qt_query_const calls ≈ 5 ms) then scored per
 * particle with O(n_beams) array lookups.
 *
 * Beam endpoints beyond ±1800 mm of the cloud centre are missed by the grid.
 * This truncates some long-range signal but does not bias relative particle
 * weights — every particle faces the same truncation. */
#define GRID_HALF    1800.0f
#define GRID_CELL     100.0f
#define GRID_N           36    /* 2 × 1800 / 100 */

static uint8_t s_grid[GRID_N][GRID_N];
static float   s_grid_ox, s_grid_oy;

/* Pre-computed beam endpoints in robot frame (same sampling as scan_matcher). */
#define MAX_BEAMS     100
#define BEAM_STRIDE     5
#define MIN_RANGE_MM  200.0f

static int16_t s_bx[MAX_BEAMS]; /* robot-frame beam endpoints, mm */
static int16_t s_by[MAX_BEAMS];
static int   s_nb = 0;

/* Likelihood sharpness: w ∝ exp(k × hits).  k=0.1 → ~20× weight ratio
 * between a particle with 70 hits vs. one with 40 hits on 90 beams. */
#define SCORE_K  0.1f

/* Resample when N_eff < n × this ratio. */
#define RESAMPLE_RATIO  0.5f


/* ── Internal: build flat grid centred at (cx, cy) ───────────────────── */
static void _build_grid(const QuadTreeMap *map, float cx, float cy)
{
    s_grid_ox = cx - GRID_HALF + GRID_CELL * 0.5f;
    s_grid_oy = cy - GRID_HALF + GRID_CELL * 0.5f;
    for (int j = 0; j < GRID_N; j++) {
        float wy = s_grid_oy + (float)j * GRID_CELL;
        for (int i = 0; i < GRID_N; i++) {
            s_grid[j][i] =
                (qt_query_const(map, s_grid_ox + (float)i * GRID_CELL, wy) > 0)
                ? 1u : 0u;
        }
    }
}

/* ── Internal: score one pose against the pre-built grid ─────────────── */
static int _score_pose(float cx, float cy, float cos_t, float sin_t)
{
    int hits = 0;
    for (int j = 0; j < s_nb; j++) {
        float bx = (float)s_bx[j];
        float by = (float)s_by[j];
        float wx = cx + bx * cos_t - by * sin_t;
        float wy = cy + bx * sin_t + by * cos_t;
        int gi = (int)((wx - s_grid_ox) / GRID_CELL + 0.5f);
        int gj = (int)((wy - s_grid_oy) / GRID_CELL + 0.5f);
        if ((unsigned)gi < (unsigned)GRID_N &&
            (unsigned)gj < (unsigned)GRID_N)
            hits += s_grid[gj][gi];
    }
    return hits;
}


/* ════════════════════════════════════════════════════════════════════════
 * Public API
 * ════════════════════════════════════════════════════════════════════════ */

void rbpf_init(rbpf_state_t *state,
               uint8_t       n_particles,
               const pose_t *initial_pose)
{
    if (!state) return;
    if (n_particles == 0 || n_particles > RBPF_MAX_PARTICLES)
        n_particles = RBPF_MAX_PARTICLES;

    state->n              = n_particles;
    state->resample_count = 0;

    pose_t base = {0};
    if (initial_pose) base = *initial_pose;

    float w0 = 1.0f / (float)n_particles;
    for (int i = 0; i < n_particles; i++) {
        /* Spread around initial pose: ~20 mm position spread, ~3° heading spread.
         * Covers typical initial localisation uncertainty without wasting particles
         * on obviously wrong areas. */
        state->poses[i].x     = base.x     + _randn() * 20.0f;
        state->poses[i].y     = base.y     + _randn() * 20.0f;
        state->poses[i].theta = base.theta + _randn() * 0.052f; /* ~3° 1-sigma */
        memcpy(state->poses[i].cov, base.cov, sizeof(base.cov));
        state->weights[i] = w0;
    }
}


void rbpf_predict(rbpf_state_t *state, const odom_t *odom)
{
    if (!state || !odom) return;

    const float delta_d = odom->linear_disp_mm;
    const float delta_r = odom->yaw_rate_imu * (odom->dt_ms * 0.001f);

    const float abs_d = fabsf(delta_d);
    const float abs_r = fabsf(delta_r);
    const bool  moving = (abs_d > MIN_TRANS_MM || abs_r > MIN_ROT_RAD);

    const float sigma_trans = ALPHA_TT * abs_d + ALPHA_TR * abs_r;
    const float sigma_rot   = ALPHA_RT * abs_d + ALPHA_RR * abs_r;

    for (int i = 0; i < state->n; i++) {
        float dd = delta_d;
        float dr = delta_r;

        if (moving) {
            dd += _randn() * sigma_trans;
            dr += _randn() * sigma_rot;
        }

        /* Midpoint Runge-Kutta: rotate half-step, then translate. */
        float mid_theta = state->poses[i].theta + 0.5f * dr;
        state->poses[i].x     += dd * cosf(mid_theta);
        state->poses[i].y     += dd * sinf(mid_theta);
        state->poses[i].theta += dr;

        /* Wrap to [-π, π] with fmodf (matches lidar_to_map.c convention). */
        state->poses[i].theta =
            fmodf(state->poses[i].theta, 2.0f * (float)M_PI);
        if (state->poses[i].theta >  (float)M_PI)
            state->poses[i].theta -= 2.0f * (float)M_PI;
        if (state->poses[i].theta < -(float)M_PI)
            state->poses[i].theta += 2.0f * (float)M_PI;
    }
}


void rbpf_update(rbpf_state_t      *state,
                 const QuadTreeMap *map,
                 const lidar_scan_t *scan)
{
    if (!state || !map || !scan) return;
    if (scan->count < 5) return;

    const float d2r = (float)M_PI / 180.0f;

    /* 1. Pre-compute beam endpoints in robot frame (stride-sampled, capped). */
    s_nb = 0;
    for (uint16_t i = 0; i < scan->count && s_nb < MAX_BEAMS; i += BEAM_STRIDE) {
        float r = lidar_point_range_mm(&scan->points[i]);
        if (r < MIN_RANGE_MM || r > LIDAR_MAP_RADIUS_MM) continue;
        float rad = -lidar_point_theta_deg(&scan->points[i]) * d2r + LIDAR_OFFSET_THETA_RAD;
        s_bx[s_nb] = (int16_t)lrintf(r * cosf(rad));
        s_by[s_nb] = (int16_t)lrintf(r * sinf(rad));
        s_nb++;
    }
    if (s_nb < 3) return;

    /* 2. Build flat grid centred on particle-weighted mean.
     *    Built once; all per-particle searches use the same grid so relative
     *    weights are unbiased despite the shared origin. */
    float mx = 0.0f, my = 0.0f;
    for (int i = 0; i < state->n; i++) {
        mx += state->weights[i] * state->poses[i].x;
        my += state->weights[i] * state->poses[i].y;
    }
    _build_grid(map, mx, my);

    /* Search offsets identical to scan_matcher.c. */
    static const float k_c_xy[] = { -100.f, -50.f,  0.f, 50.f, 100.f }; /* mm */
    static const float k_c_th[] = {   -8.f,  -4.f,  0.f,  4.f,   8.f }; /* deg */
    static const float k_f_xy[] = {  -20.f, -10.f,  0.f, 10.f,  20.f }; /* mm */
    static const float k_f_th[] = {   -2.f,  -1.f,  0.f,  1.f,   2.f }; /* deg */

    /* 3. GMapping improved-proposal: per-particle coarse+fine scan matching.
     *    Each particle is moved to its locally-best matched pose BEFORE weighting.
     *    This concentrates the proposal around high-likelihood regions and lets
     *    N=8 particles achieve accuracy that would require 100+ in naive MCL. */
    float raw_w[RBPF_MAX_PARTICLES];
    float sum_w = 0.0f;

    for (int p = 0; p < state->n; p++) {
        const float px = state->poses[p].x;
        const float py = state->poses[p].y;
        const float pt = state->poses[p].theta;

        /* 3a. Coarse search: 5×5×5 = 125 candidates.
         * cbest=0: ties (all-zero map signal) keep defaults (0,0,0) → predicted
         * pose wins.  cbest=-1 would let the first candidate (offsets -100,-100,-8°)
         * win whenever the map is empty, drifting every particle by ~100 mm/cycle. */
        int   cbest    = 0;
        float cbest_dx = 0.f, cbest_dy = 0.f, cbest_dt = 0.f;
        for (int ti = 0; ti < 5; ti++) {
            float t_cand = pt + k_c_th[ti] * d2r;
            float ct = cosf(t_cand), st = sinf(t_cand);
            for (int yi = 0; yi < 5; yi++) {
                for (int xi = 0; xi < 5; xi++) {
                    int score = _score_pose(px + k_c_xy[xi],
                                           py + k_c_xy[yi], ct, st);
                    if (score > cbest) {
                        cbest    = score;
                        cbest_dx = k_c_xy[xi];
                        cbest_dy = k_c_xy[yi];
                        cbest_dt = k_c_th[ti] * d2r;
                    }
                }
            }
        }

        /* 3b. Fine search: 5×5×5 = 125 candidates centred on coarse winner. */
        float base_x = px + cbest_dx;
        float base_y = py + cbest_dy;
        float base_t = pt + cbest_dt;
        int   fbest   = cbest;
        float fbest_x = base_x, fbest_y = base_y, fbest_t = base_t;

        for (int ti = 0; ti < 5; ti++) {
            float t_cand = base_t + k_f_th[ti] * d2r;
            float ct = cosf(t_cand), st = sinf(t_cand);
            for (int yi = 0; yi < 5; yi++) {
                for (int xi = 0; xi < 5; xi++) {
                    float cx = base_x + k_f_xy[xi];
                    float cy = base_y + k_f_xy[yi];
                    int score = _score_pose(cx, cy, ct, st);
                    if (score > fbest) {
                        fbest   = score;
                        fbest_x = cx;
                        fbest_y = cy;
                        fbest_t = t_cand;
                    }
                }
            }
        }

        /* 3c. Move particle to its best-matched pose (improved proposal). */
        state->poses[p].x     = fbest_x;
        state->poses[p].y     = fbest_y;
        state->poses[p].theta = fbest_t;

        /* Wrap theta to [-π, π] — same convention as rbpf_predict. */
        state->poses[p].theta =
            fmodf(state->poses[p].theta, 2.0f * (float)M_PI);
        if (state->poses[p].theta >  (float)M_PI)
            state->poses[p].theta -= 2.0f * (float)M_PI;
        if (state->poses[p].theta < -(float)M_PI)
            state->poses[p].theta += 2.0f * (float)M_PI;

        /* 3d. Weight = prior × exp(k × best_score). */
        raw_w[p] = state->weights[p] * expf(SCORE_K * (float)fbest);
        sum_w   += raw_w[p];
    }

    /* 4. Normalise — or reset to uniform when map is empty (all scores 0). */
    if (sum_w < 1e-12f) {
        float w0 = 1.0f / (float)state->n;
        for (int i = 0; i < state->n; i++) state->weights[i] = w0;
        return;
    }
    float inv = 1.0f / sum_w;
    for (int i = 0; i < state->n; i++) state->weights[i] = raw_w[i] * inv;

    /* 5. Compute N_eff = 1 / Σ wᵢ². */
    float sum_w2 = 0.0f;
    for (int i = 0; i < state->n; i++)
        sum_w2 += state->weights[i] * state->weights[i];
    float n_eff = (sum_w2 > 1e-12f) ? (1.0f / sum_w2) : (float)state->n;

    /* 6. Low-variance (systematic) resampling when N_eff < N/2.
     *    Thrun et al., "Probabilistic Robotics", Algorithm 4.4. */
    if (n_eff < (float)state->n * RESAMPLE_RATIO) {
        pose_t new_poses[RBPF_MAX_PARTICLES];
        float  w0  = 1.0f / (float)state->n;
        float  r   = _rand_f() * w0;
        float  c   = state->weights[0];
        int    idx = 0;

        for (int m = 0; m < state->n; m++) {
            float U = r + (float)m * w0;
            while (U > c && idx < state->n - 1) {
                idx++;
                c += state->weights[idx];
            }
            new_poses[m] = state->poses[idx];
        }

        memcpy(state->poses, new_poses, (size_t)state->n * sizeof(pose_t));
        for (int i = 0; i < state->n; i++) state->weights[i] = w0;
        state->resample_count++;
    }
}


pose_t rbpf_get_best_pose(const rbpf_state_t *state)
{
    pose_t zero = {0};
    if (!state || state->n == 0) return zero;
    int best = 0;
    for (int i = 1; i < state->n; i++)
        if (state->weights[i] > state->weights[best]) best = i;
    return state->poses[best];
}


pose_t rbpf_get_mean_pose(const rbpf_state_t *state)
{
    pose_t out = {0};
    if (!state || state->n == 0) return out;

    float sx = 0.0f, sy = 0.0f, ss = 0.0f, sc = 0.0f;
    for (int i = 0; i < state->n; i++) {
        float w  = state->weights[i];
        sx += w * state->poses[i].x;
        sy += w * state->poses[i].y;
        ss += w * sinf(state->poses[i].theta);
        sc += w * cosf(state->poses[i].theta);
    }
    out.x     = sx;
    out.y     = sy;
    out.theta = atan2f(ss, sc);   /* circular mean — handles θ wrap correctly */
    return out;
}


void rbpf_get_diag(const rbpf_state_t *state, rbpf_diag_t *out)
{
    if (!state || !out) return;
    float sum_w2 = 0.0f, max_w = 0.0f;
    for (int i = 0; i < state->n; i++) {
        float w = state->weights[i];
        sum_w2 += w * w;
        if (w > max_w) max_w = w;
    }
    out->n_eff          = (sum_w2 > 1e-12f) ? (1.0f / sum_w2) : (float)state->n;
    out->best_weight    = max_w;
    out->resample_count = state->resample_count;
}
