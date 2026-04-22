/**
 * scan_matcher.c
 * Module: ICP-based scan matching (point-to-point).
 * Board: ESP32-S3
 * Written with the help of ChatGPT and Claude.
 *
 * Algorithm overview:
 *   1. For each point in the current (transformed) cloud, find the nearest
 *      neighbour in the reference cloud (brute-force O(n*m), acceptable for
 *      ≤460 pts on ESP32-S3 at ~10 Hz).
 *   2. Compute the optimal rigid transform (rotation + translation) that
 *      minimises the sum of squared distances using the SVD-free closed-form
 *      2-D solution (cross-covariance → atan2).
 *   3. Accumulate the incremental transform across iterations.
 *   4. Derive a quality score from the mean squared error of matched pairs.
 *
 * Coordinate convention: x forward, y left, angles in radians (CCW positive).
 * Units: millimetres for translation, radians for rotation.
 */

#include "scan_matcher.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

/* ── tunables ─────────────────────────────────────────────────────────────── */

/** Default maximum ICP iterations (can be overridden via scan_matcher_set_max_iter). */
#define ICP_MAX_ITER_DEFAULT  20u

/**
 * Distance threshold: correspondences whose squared distance exceeds this
 * value are rejected as outliers.  Unit: mm².  (200 mm ≈ 8 inches.)
 */
#define ICP_REJECT_DIST2  (200.0f * 200.0f)

/**
 * Convergence threshold: if the total incremental displacement in one
 * iteration is smaller than this we stop early.  Unit: mm (rotation
 * contributes 1 mm per 1 mm of effective arm length ≈ just stops it spinning).
 */
#define ICP_CONV_EPS  0.1f

/**
 * Score scaling factor k: score = exp(-k * mean_error²).
 * k = 1/σ² with σ ≈ 30 mm nominal correspondence noise.
 * Tuned so that a perfect match (mean_error → 0) gives score → 1.0
 * and a 30 mm mean error gives score ≈ 0.37.
 */
#define ICP_SCORE_K  (1.0f / (30.0f * 30.0f))

/* ── module state ─────────────────────────────────────────────────────────── */

static uint8_t s_max_iter = ICP_MAX_ITER_DEFAULT;

/* ── internal types ───────────────────────────────────────────────────────── */

/** Accumulated 2-D rigid transform (working unit: mm / rad). */
typedef struct {
    float dx;
    float dy;
    float dtheta;
} Transform2D;

/* ── helpers ──────────────────────────────────────────────────────────────── */

/** Apply a 2-D rigid transform to a single point (in-place). */
static inline void apply_transform(const Transform2D *T,
                                   float *x, float *y)
{
    float c = cosf(T->dtheta);
    float s = sinf(T->dtheta);
    float nx = c * (*x) - s * (*y) + T->dx;
    float ny = s * (*x) + c * (*y) + T->dy;
    *x = nx;
    *y = ny;
}

/**
 * Compose T_total ← T_new ∘ T_total.
 * Both represent "first apply T_total, then T_new".
 */
static void compose_transform(Transform2D *total, const Transform2D *delta)
{
    float c = cosf(delta->dtheta);
    float s = sinf(delta->dtheta);

    float nx = c * total->dx - s * total->dy + delta->dx;
    float ny = s * total->dx + c * total->dy + delta->dy;

    total->dx     = nx;
    total->dy     = ny;
    total->dtheta = total->dtheta + delta->dtheta;

    /* keep dtheta in (-π, π] */
    while (total->dtheta >  3.14159265f) total->dtheta -= 6.28318530f;
    while (total->dtheta < -3.14159265f) total->dtheta += 6.28318530f;
}

/**
 * Find the index of the closest point in the reference cloud to query (qx, qy).
 * Returns UINT16_MAX if ref_n == 0.
 */
static uint16_t nearest_neighbour(const point2f_t *ref, uint16_t ref_n,
                                  float qx, float qy,
                                  float *out_dist2)
{
    float   best_d2  = 1e30f;
    uint16_t best_idx = UINT16_MAX;

    for (uint16_t i = 0; i < ref_n; i++) {
        float ex = ref[i].x - qx;
        float ey = ref[i].y - qy;
        float d2 = ex * ex + ey * ey;
        if (d2 < best_d2) {
            best_d2  = d2;
            best_idx = i;
        }
    }

    *out_dist2 = best_d2;
    return best_idx;
}

/**
 * Closed-form 2-D point-to-point ICP step.
 *
 * Given N matched pairs (src[i], dst[i]), compute the transform that minimises
 *   Σ || dst[i] - (R·src[i] + t) ||²
 *
 * Solution (2-D):
 *   1. Centroid subtraction.
 *   2. Cross-covariance Sxy = Σ (src_i - μ_src) · (dst_i - μ_dst)ᵀ  [scalar x-y products].
 *   3. dθ = atan2(Sxy_yx - Sxy_xy,  Sxy_xx + Sxy_yy)   (sum of cross-products).
 *   4. t = μ_dst - R·μ_src.
 *
 * @param src       Transformed current-scan points.
 * @param dst       Corresponding reference-scan points.
 * @param n         Number of valid pairs.
 * @param out       Output incremental transform.
 */
static void compute_optimal_transform(const float *src_x, const float *src_y,
                                      const float *dst_x, const float *dst_y,
                                      uint16_t n,
                                      Transform2D *out)
{
    if (n == 0) {
        out->dx = out->dy = out->dtheta = 0.0f;
        return;
    }

    float inv_n = 1.0f / (float)n;

    /* centroids */
    float mu_sx = 0.0f, mu_sy = 0.0f;
    float mu_dx = 0.0f, mu_dy = 0.0f;
    for (uint16_t i = 0; i < n; i++) {
        mu_sx += src_x[i]; mu_sy += src_y[i];
        mu_dx += dst_x[i]; mu_dy += dst_y[i];
    }
    mu_sx *= inv_n; mu_sy *= inv_n;
    mu_dx *= inv_n; mu_dy *= inv_n;

    /* cross-covariance (2×2, row-major) */
    float Sxx = 0.0f, Sxy = 0.0f, Syx = 0.0f, Syy = 0.0f;
    for (uint16_t i = 0; i < n; i++) {
        float ax = src_x[i] - mu_sx, ay = src_y[i] - mu_sy;
        float bx = dst_x[i] - mu_dx, by = dst_y[i] - mu_dy;
        Sxx += ax * bx;
        Sxy += ax * by;
        Syx += ay * bx;
        Syy += ay * by;
    }

    /*
     * Optimal rotation angle (Horn 1987 closed-form 2-D solution).
     * Minimises ||dst - (R·src + t)||² → R = argmin.
     * The cross-covariance is S = Σ (src_demean)(dst_demean)ᵀ, so
     *   dθ = atan2(S_xy − S_yx,  S_xx + S_yy)
     * This yields the angle to rotate src onto dst (i.e., the correction).
     */
    float dtheta = atan2f(Sxy - Syx, Sxx + Syy);

    /* optimal translation */
    float c = cosf(dtheta);
    float s = sinf(dtheta);
    float tx = mu_dx - (c * mu_sx - s * mu_sy);
    float ty = mu_dy - (s * mu_sx + c * mu_sy);

    out->dx     = tx;
    out->dy     = ty;
    out->dtheta = dtheta;
}

/* ── public API ───────────────────────────────────────────────────────────── */

void scan_matcher_set_max_iter(uint8_t iters)
{
    if (iters == 0) iters = 1;
    s_max_iter = iters;
}

pose_correction_t scan_matcher_match(const point2f_t *ref, uint16_t ref_n,
                                     const point2f_t *cur, uint16_t cur_n)
{
    pose_correction_t result;
    memset(&result, 0, sizeof(result));
    result.score = 0.0f;

    /* Guard against degenerate inputs */
    if (!ref || !cur || ref_n < 3 || cur_n < 3) {
        return result;
    }

    /* --- working copy of current cloud (gets transformed each iteration) --- */
    /* Maximum 460 points (RPLiDAR C1 spec).  Allocate on the stack.
     * Stack frame on ESP32-S3: 460 × 2 × 4 B = 3 680 B.  Within limits. */
#define SM_MAX_PTS 460u
    if (cur_n > SM_MAX_PTS) cur_n = SM_MAX_PTS;

    float wx[SM_MAX_PTS];   /* working x for current scan */
    float wy[SM_MAX_PTS];   /* working y for current scan */

    for (uint16_t i = 0; i < cur_n; i++) {
        wx[i] = cur[i].x;
        wy[i] = cur[i].y;
    }

    /* Correspondence buffers (at most cur_n valid pairs per iteration) */
    float src_x[SM_MAX_PTS], src_y[SM_MAX_PTS];
    float dst_x[SM_MAX_PTS], dst_y[SM_MAX_PTS];

    Transform2D total = { 0.0f, 0.0f, 0.0f };
    float mean_error2 = 1e30f;

    /* ── ICP iterations ────────────────────────────────────────────────────── */
    for (uint8_t iter = 0; iter < s_max_iter; iter++) {

        /* 1. Find correspondences, reject outliers */
        uint16_t n_pairs = 0;
        float    sum_d2  = 0.0f;

        for (uint16_t i = 0; i < cur_n; i++) {
            float d2;
            uint16_t j = nearest_neighbour(ref, ref_n, wx[i], wy[i], &d2);
            if (j == UINT16_MAX || d2 > ICP_REJECT_DIST2) continue;

            src_x[n_pairs] = wx[i];
            src_y[n_pairs] = wy[i];
            dst_x[n_pairs] = ref[j].x;
            dst_y[n_pairs] = ref[j].y;
            sum_d2 += d2;
            n_pairs++;
        }

        if (n_pairs < 3) break;   /* not enough inliers */

        mean_error2 = sum_d2 / (float)n_pairs;

        /* 2. Compute optimal incremental transform */
        Transform2D delta;
        compute_optimal_transform(src_x, src_y, dst_x, dst_y, n_pairs, &delta);

        /* 3. Apply delta to working cloud */
        for (uint16_t i = 0; i < cur_n; i++) {
            apply_transform(&delta, &wx[i], &wy[i]);
        }

        /* 4. Accumulate total transform */
        compose_transform(&total, &delta);

        /* 5. Check convergence */
        float disp = sqrtf(delta.dx * delta.dx + delta.dy * delta.dy)
                     + fabsf(delta.dtheta) * 100.0f; /* weight rotation */
        if (disp < ICP_CONV_EPS) break;
    }

    /* ── Build result ──────────────────────────────────────────────────────── */
    result.dx     = total.dx;
    result.dy     = total.dy;
    result.dtheta = total.dtheta;

    /*
     * Quality score: exponential decay on mean squared error.
     *   score = exp(-k * mean_error²)
     * Clamped to [0, 1].  For a perfect match mean_error → 0 → score → 1.
     */
    if (mean_error2 < 1e29f) {
        result.score = expf(-ICP_SCORE_K * mean_error2);
    }

    return result;
}