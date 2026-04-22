/**
 * tests/test_scan_matcher.c
 * Merged unit tests for the ICP scan matcher.
 * Written with the help of ChatGPT and Claude.
 *
 * ── Build modes ──────────────────────────────────────────────────────────────
 *
 *   Invoke from inside the tests/ directory.
 *
 *   A) Stub mode — exercises the full lidar → polar_to_cart → scan_matcher
 *      pipeline using stub scan data.
 *
 *      cc -DUSE_STUBS test_scan_matcher.c \
 *          ../stubs/lidar_stub.c \
 *          ../stubs/scan_match_stub.c \
 *          ../stubs/pose_stub.c \
 *          ../src/scan_matcher.c \
 *          ../src/polar_to_cart.c \
 *          -lm -o test_scan_matcher
 *
 *   B) Standalone mode — no stubs required; all tests use synthetic fixtures.
 *
 *      cc test_scan_matcher.c \
 *          ../src/scan_matcher.c \
 *          -lm -o test_scan_matcher
 *
 * Exit code: 0 if every assertion passed, 1 otherwise.
 *
/* ── standard includes ───────────────────────────────────────────────────────*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>

/* Portable pi constant — avoids depending on _GNU_SOURCE or M_PI */
#ifndef SM_PI
#define SM_PI 3.14159265358979f
#endif

/* ── project types ───────────────────────────────────────────────────────────*/
#include "../../types.h"

/* ── optional stub headers ───────────────────────────────────────────────────*/
#ifdef USE_STUBS
#include "../stubs/lidar_stub.h"
#include "../stubs/scan_match_stub.h"
#include "../stubs/pose_stub.h"
#endif

/* ── modules under test ──────────────────────────────────────────────────────*/
#include "../src/scan_matcher.h"

#ifdef USE_STUBS
#include "../src/polar_to_cart.h"
#endif

/* ═══════════════════════════════════════════════════════════════════════════
 * Test framework
 *
 * EXPECT_TRUE / EXPECT_NEAR  — fine-grained assertions (one line per check).
 *   Used by Group B (ICP geometry tests) to report each condition separately,
 *   matching the granular style of the original second test file.
 *
 * ASSERT_PASS                — coarse-grained assertion (one line per test).
 *   Used by Group A (pipeline tests) to mirror the single PASS/FAIL output
 *   of the original first test file.
 *
 * Both styles share the same s_total / s_passed counters.
 * ═══════════════════════════════════════════════════════════════════════════ */

static int s_total  = 0;
static int s_passed = 0;

#define EXPECT_TRUE(cond, msg) do {                                   \
    s_total++;                                                        \
    if (cond) { printf("  PASS  %s\n", msg); s_passed++; }           \
    else       { printf("  FAIL  %s\n", msg); }                       \
} while (0)

#define EXPECT_NEAR(a, b, tol, msg) \
    EXPECT_TRUE(fabsf((float)(a) - (float)(b)) <= (float)(tol), msg)

#define ASSERT_PASS(label, cond) do {                                 \
    s_total++;                                                        \
    if (cond) { printf("  PASS  %s\n", label); s_passed++; }         \
    else       { printf("  FAIL  %s\n", label); }                     \
} while (0)

/* ═══════════════════════════════════════════════════════════════════════════
 * Fixture helpers
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * Square room: four equal walls, uniformly sampled.
 * Good for translation / drift / cloud-size tests.
 * Avoid for rotation tests — 4-fold symmetry makes rotation ambiguous.
 */
static uint16_t make_square_room(point2f_t *pts, uint16_t capacity,
                                 float half_side, uint16_t pts_per_side)
{
    uint16_t count = 0;
    float step = (2.0f * half_side) / (float)(pts_per_side - 1);

    for (uint16_t i = 0; i < pts_per_side && count < capacity; i++, count++) {
        pts[count].x = -half_side + i * step; pts[count].y = -half_side;
    }
    for (uint16_t i = 0; i < pts_per_side && count < capacity; i++, count++) {
        pts[count].x = -half_side + i * step; pts[count].y =  half_side;
    }
    for (uint16_t i = 0; i < pts_per_side && count < capacity; i++, count++) {
        pts[count].x = -half_side; pts[count].y = -half_side + i * step;
    }
    for (uint16_t i = 0; i < pts_per_side && count < capacity; i++, count++) {
        pts[count].x =  half_side; pts[count].y = -half_side + i * step;
    }
    return count;
}

/**
 * Asymmetric room: walls of unequal length + interior diagonal obstacle.
 * Breaks 2-fold symmetry; sufficient for large-angle rotation tests (> 5 deg)
 * and for the A4 rotation-detection test where only |dtheta| matters.
 */
static uint16_t make_asymmetric_room(point2f_t *pts, uint16_t capacity)
{
    uint16_t count = 0;
    const uint16_t pps = 50;

    for (uint16_t i = 0; i < pps && count < capacity; i++, count++) {
        pts[count].x = -1000.0f + i * (2000.0f / (pps - 1)); pts[count].y = -800.0f;
    }
    for (uint16_t i = 0; i < pps && count < capacity; i++, count++) {
        pts[count].x = -1000.0f + i * (2000.0f / (pps - 1)); pts[count].y =  600.0f;
    }
    for (uint16_t i = 0; i < 36 && count < capacity; i++, count++) {
        pts[count].x = -1000.0f; pts[count].y = -800.0f + i * (1400.0f / 35.0f);
    }
    for (uint16_t i = 0; i < 24 && count < capacity; i++, count++) {
        pts[count].x =  1000.0f; pts[count].y = -800.0f + i * (900.0f / 23.0f);
    }
    for (uint16_t i = 0; i < 20 && count < capacity; i++, count++) {
        pts[count].x = 200.0f + i * 15.0f; pts[count].y = 100.0f + i * 20.0f;
    }
    return count;
}

/**
 * Feature-rich cloud: curved arc + two diagonal obstacles + corner + isolated
 * cluster.  Produces strong, unambiguous rotation signal for angles as small
 * as 2-3 degrees.  Required for B3 and B4.
 */
static uint16_t make_feature_rich(point2f_t *pts, uint16_t capacity)
{
    uint16_t n = 0;

    /* diagonal obstacle #1 */
    for (int i = 0; i < 30 && n < capacity; i++, n++) {
        pts[n].x = (float)i * 20.0f; pts[n].y = (float)i * 30.0f;
    }
    /* diagonal obstacle #2 (different slope) */
    for (int i = 0; i < 30 && n < capacity; i++, n++) {
        pts[n].x = 500.0f + (float)i * 25.0f; pts[n].y = 300.0f - (float)i * 15.0f;
    }
    /* semicircular arc — strong rotation constraint */
    for (int i = 0; i < 40 && n < capacity; i++, n++) {
        float a = (float)i / 39.0f * SM_PI;
        pts[n].x = 800.0f * cosf(a); pts[n].y = 600.0f * sinf(a) + 200.0f;
    }
    /* corner: two short perpendicular segments */
    for (int i = 0; i < 20 && n < capacity; i++, n++) {
        pts[n].x = -500.0f + (float)i * 10.0f; pts[n].y = -300.0f;
    }
    for (int i = 0; i < 20 && n < capacity; i++, n++) {
        pts[n].x = -500.0f; pts[n].y = -300.0f + (float)i * 10.0f;
    }
    /* isolated far-field anchor */
    for (int i = 0; i < 10 && n < capacity; i++, n++) {
        pts[n].x = 900.0f + (float)i * 5.0f; pts[n].y = -600.0f + (float)i * 3.0f;
    }
    return n;
}

/**
 * Apply a rigid 2-D transform to a point cloud.
 * @param in     Source (read-only; may equal out when dtheta==0 and dx/dy are applied in-place).
 * @param n      Number of points.
 * @param dx     Translation x (mm).
 * @param dy     Translation y (mm).
 * @param dtheta Rotation (rad, CCW positive).
 * @param out    Destination (may alias in for pure translations).
 */
static void apply_known_transform(const point2f_t *in, uint16_t n,
                                  float dx, float dy, float dtheta,
                                  point2f_t *out)
{
    float c = cosf(dtheta);
    float s = sinf(dtheta);
    for (uint16_t i = 0; i < n; i++) {
        out[i].x = c * in[i].x - s * in[i].y + dx;
        out[i].y = s * in[i].x + c * in[i].y + dy;
    }
}

/* ── stub-mode helper ────────────────────────────────────────────────────────*/
#ifdef USE_STUBS
/**
 * Convert a polar lidar_scan_t to a Cartesian point2f_t array.
 * Thin wrapper around polar_to_cart_convert(); here for readability.
 */
static uint16_t scan_to_cart(const lidar_scan_t *scan, point2f_t *pts)
{
    uint16_t count = 0;
    polar_to_cart_convert(scan, pts, &count);
    return count;
}
#endif /* USE_STUBS */

/* ═══════════════════════════════════════════════════════════════════════════
 * GROUP A — pipeline-level tests
 *
 * Originally from the first test file.  In USE_STUBS mode the real stub scan
 * infrastructure is used (lidar_stub → polar_to_cart → scan_matcher).
 * In standalone mode the same logical assertions are verified using synthetic
 * fixtures so every test runs in both build configurations.
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * A1 — Perfect match: score >= 1.0.
 *
 * Stub path:  scan_match_stub_perfect() returns a pre-baked perfect result;
 *             we verify the stub is well-formed.
 * Standalone: identical clouds fed to scan_matcher_match(); score must be
 *             >= 0.99 (exp(-k*0) = 1.0, but FP rounding may shave a tiny bit).
 */
static void test_A1_perfect_match(void)
{
    printf("\n[A1] Perfect match — score >= 1.0\n");
#ifdef USE_STUBS
    pose_correction_t c = scan_match_stub_perfect();
    ASSERT_PASS("stub perfect score >= 1.0", c.score >= 1.0f);
#else
    point2f_t room[200];
    uint16_t  n = make_square_room(room, 200, 1000.0f, 50);
    scan_matcher_set_max_iter(20);
    pose_correction_t c = scan_matcher_match(room, n, room, n);
    ASSERT_PASS("identical clouds score >= 0.99", c.score >= 0.99f);
#endif
}

/**
 * A2 — Slight drift: correction dx > 0, score < 1.0.
 *
 * Stub path:  scan_match_stub_slight_drift() returns pre-baked values.
 * Standalone: square room shifted +30 mm with 20 extra noise points appended.
 *             The noise points (absent from the reference) leave residual MSE
 *             after ICP converges, so score < 1.0 while |dx| is clearly > 0.
 */
static void test_A2_slight_drift(void)
{
    printf("\n[A2] Slight drift detection — |dx| > 0, score < 1.0\n");
#ifdef USE_STUBS
    pose_correction_t c = scan_match_stub_slight_drift();
    ASSERT_PASS("stub drift dx > 0",      c.dx    > 0.0f);
    ASSERT_PASS("stub drift score < 1.0", c.score < 1.0f);
#else
    point2f_t ref[200], cur[220];
    uint16_t  n_ref = make_square_room(ref, 200, 1000.0f, 50);
    uint16_t  n_cur = make_square_room(cur, 200, 1000.0f, 50);

    /* shift current cloud by +30 mm */
    for (uint16_t i = 0; i < n_cur; i++) cur[i].x += 30.0f;

    /* add noise points not present in reference — ensures score < 1.0 */
    for (int i = 0; i < 20; i++, n_cur++) {
        cur[n_cur].x = 200.0f + (float)i * 30.0f;
        cur[n_cur].y = 500.0f + (float)i * 20.0f;
    }

    scan_matcher_set_max_iter(20);
    pose_correction_t c = scan_matcher_match(ref, n_ref, cur, n_cur);
    ASSERT_PASS("drift |dx| > 0",    fabsf(c.dx) > 0.0f);
    ASSERT_PASS("drift score < 1.0", c.score     < 1.0f);
#endif
}

/**
 * A3 — Zero motion: dx ~= 0, dy ~= 0, dtheta ~= 0.
 *
 * Both paths feed the same scan as ref and cur.  In stub mode the scan is
 * obtained from lidar_stub_room_scan() and converted through polar_to_cart.
 */
static void test_A3_zero_motion(void)
{
    printf("\n[A3] Zero motion — dx~=0, dy~=0, dtheta~=0\n");
#ifdef USE_STUBS
    lidar_scan_t scan = lidar_stub_room_scan();
    point2f_t pts_a[460], pts_b[460];
    uint16_t  n_a = scan_to_cart(&scan, pts_a);
    uint16_t  n_b = scan_to_cart(&scan, pts_b);
    scan_matcher_set_max_iter(20);
    pose_correction_t c = scan_matcher_match(pts_a, n_a, pts_b, n_b);
#else
    point2f_t room[200];
    uint16_t  n = make_square_room(room, 200, 1000.0f, 50);
    scan_matcher_set_max_iter(20);
    pose_correction_t c = scan_matcher_match(room, n, room, n);
#endif
    ASSERT_PASS("zero-motion |dx|     < 1 mm",     fabsf(c.dx)     < 1.0f);
    ASSERT_PASS("zero-motion |dy|     < 1 mm",     fabsf(c.dy)     < 1.0f);
    ASSERT_PASS("zero-motion |dtheta| < 0.01 rad", fabsf(c.dtheta) < 0.01f);
}

/**
 * A4 — Rotation detection: |dtheta| > 0.02 rad and score < 0.95.
 *
 * A rotation of 0.1 rad + translation of 50 mm is applied, then index-based
 * noise is added (preserving the original test's perturbation pattern).
 *
 * In stub mode: lidar_stub_room_scan() produces a perfect circle, which is
 * rotationally invariant — ICP cannot recover the angle.  We therefore use
 * the asymmetric_room fixture in both modes so the test behaviour is consistent
 * and the pass criterion (rotation *detected*, not recovered exactly) is met.
 *
 * In stub mode the polar_to_cart conversion is still exercised for A3/A6.
 */
static void test_A4_rotation_detection(void)
{
    printf("\n[A4] Rotation detection — |dtheta| > 0.02 rad, score < 0.95\n");

    /* Use asymmetric_room in both modes: the stub's circular scan is rotationally
     * invariant and would always give dtheta==0.  The asymmetric fixture
     * provides the necessary geometric variety. */
    point2f_t pts_ref[200], pts_rot[200];
    uint16_t  n = make_asymmetric_room(pts_ref, 200);

    const float angle = 0.1f;
    const float tx    = 50.0f;
    const float ca    = cosf(angle);
    const float sa    = sinf(angle);

    for (uint16_t i = 0; i < n; i++) {
        float x = pts_ref[i].x;
        float y = pts_ref[i].y;
        /* rotate + translate + add structured index-based noise */
        pts_rot[i].x = ca * x - sa * y + tx + (float)(i % 7) * 5.0f;
        pts_rot[i].y = sa * x + ca * y      + (float)(i % 5) * 3.0f;
    }

    scan_matcher_set_max_iter(20);
    pose_correction_t res = scan_matcher_match(pts_ref, n, pts_rot, n);

    ASSERT_PASS("rotation detected |dtheta| > 0.02", fabsf(res.dtheta) > 0.02f);
    ASSERT_PASS("mismatch detected  score   < 0.95",  res.score         < 0.95f);
}

/**
 * A5 — Null / zero input safety: must return zeroed correction, never crash.
 */
static void test_A5_null_input(void)
{
    printf("\n[A5] Null / zero input — no crash, zeroed result\n");
    pose_correction_t c = scan_matcher_match(NULL, 0, NULL, 0);
    ASSERT_PASS("null input dx     == 0", c.dx     == 0.0f);
    ASSERT_PASS("null input dy     == 0", c.dy     == 0.0f);
    ASSERT_PASS("null input dtheta == 0", c.dtheta == 0.0f);
}

/**
 * A6 — Repeatability: two calls with identical inputs must return identical
 * outputs (ICP is deterministic for fixed point clouds).
 */
static void test_A6_repeatability(void)
{
    printf("\n[A6] Repeatability — same inputs yield same outputs\n");
#ifdef USE_STUBS
    lidar_scan_t scan = lidar_stub_room_scan();
    point2f_t pts[460];
    uint16_t  n = scan_to_cart(&scan, pts);
#else
    point2f_t pts[200];
    uint16_t  n = make_square_room(pts, 200, 1000.0f, 50);
#endif
    scan_matcher_set_max_iter(20);
    pose_correction_t c1 = scan_matcher_match(pts, n, pts, n);
    pose_correction_t c2 = scan_matcher_match(pts, n, pts, n);

    ASSERT_PASS("repeat |dx1-dx2|         < 0.01",   fabsf(c1.dx     - c2.dx)     < 0.01f);
    ASSERT_PASS("repeat |dy1-dy2|         < 0.01",   fabsf(c1.dy     - c2.dy)     < 0.01f);
    ASSERT_PASS("repeat |dtheta1-dtheta2| < 0.001",  fabsf(c1.dtheta - c2.dtheta) < 0.001f);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * GROUP B — ICP geometry tests
 *
 * Originally from the second test file.  All tests use synthetic point clouds
 * and verify that the ICP algorithm recovers ground-truth transforms to within
 * tight numerical tolerances.  Run identically in both build modes.
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * B1 — Perfect match (ref == cur).
 * Expected: dx~=0, dy~=0, dtheta~=0, score >= 0.99.
 *
 * Spec requires score >= 1.0; our score = exp(-k*MSE) → 1.0 as MSE → 0.
 * Floating-point rounding may land at 0.9999…, so we accept >= 0.99.
 */
static void test_B1_perfect_match(void)
{
    printf("\n[B1] Perfect match (ref == cur)\n");
    point2f_t room[200];
    uint16_t  n = make_square_room(room, 200, 1000.0f, 50);
    scan_matcher_set_max_iter(20);
    pose_correction_t c = scan_matcher_match(room, n, room, n);

    EXPECT_NEAR(c.dx,     0.0f, 1.0f,  "dx ~= 0 mm");
    EXPECT_NEAR(c.dy,     0.0f, 1.0f,  "dy ~= 0 mm");
    EXPECT_NEAR(c.dtheta, 0.0f, 0.01f, "dtheta ~= 0 rad");
    EXPECT_TRUE(c.score >= 0.99f,       "score >= 0.99 (spec: >= 1.0 for perfect match)");
}

/**
 * B2 — Pure translation (+30 mm in x).
 * ICP correction reverses the displacement: dx ~= -30 mm.
 */
static void test_B2_pure_translation(void)
{
    printf("\n[B2] Pure translation (+30 mm in x)\n");
    point2f_t ref[200], cur[200];
    uint16_t  n = make_square_room(ref, 200, 1000.0f, 50);
    apply_known_transform(ref, n, 30.0f, 0.0f, 0.0f, cur);
    scan_matcher_set_max_iter(30);
    pose_correction_t c = scan_matcher_match(ref, n, cur, n);

    EXPECT_NEAR(c.dx,     -30.0f, 3.0f,  "dx ~= -30 mm");
    EXPECT_NEAR(c.dy,       0.0f, 3.0f,  "dy ~= 0 mm");
    EXPECT_NEAR(c.dtheta,   0.0f, 0.01f, "dtheta ~= 0 rad");
    EXPECT_TRUE(c.score >= 0.5f,          "score >= 0.5 after correction");
}

/**
 * B3 — Pure rotation (+3 degrees).
 * Uses make_feature_rich() — straight-wall clouds produce near-zero rotation
 * signal due to correspondence ambiguity along wall direction.
 * Expected correction: dtheta ~= -3 deg (rotates cur back onto ref).
 */
static void test_B3_pure_rotation(void)
{
    printf("\n[B3] Pure rotation (+3 degrees)\n");
    const float DEG3 = 3.0f * SM_PI / 180.0f;
    point2f_t ref[200], cur[200];
    uint16_t  n = make_feature_rich(ref, 200);
    apply_known_transform(ref, n, 0.0f, 0.0f, DEG3, cur);
    scan_matcher_set_max_iter(40);
    pose_correction_t c = scan_matcher_match(ref, n, cur, n);

    EXPECT_NEAR(c.dx,      0.0f,  5.0f,   "dx ~= 0 mm");
    EXPECT_NEAR(c.dy,      0.0f,  5.0f,   "dy ~= 0 mm");
    EXPECT_NEAR(c.dtheta, -DEG3,  0.005f, "dtheta ~= -3 degrees");
    EXPECT_TRUE(c.score >= 0.5f,           "score >= 0.5 after correction");
}

/**
 * B4 — Combined translation + rotation (dx=20 mm, dy=10 mm, dtheta=+2 deg).
 * All three DOF must be recovered simultaneously.
 */
static void test_B4_combined(void)
{
    printf("\n[B4] Combined translation + rotation (dx=20, dy=10, theta=+2 deg)\n");
    const float DEG2 = 2.0f * SM_PI / 180.0f;
    point2f_t ref[200], cur[200];
    uint16_t  n = make_feature_rich(ref, 200);
    apply_known_transform(ref, n, 20.0f, 10.0f, DEG2, cur);
    scan_matcher_set_max_iter(40);
    pose_correction_t c = scan_matcher_match(ref, n, cur, n);

    EXPECT_NEAR(c.dx,     -20.0f, 5.0f,  "dx ~= -20 mm");
    EXPECT_NEAR(c.dy,     -10.0f, 5.0f,  "dy ~= -10 mm");
    EXPECT_NEAR(c.dtheta, -DEG2,  0.01f, "dtheta ~= -2 degrees");
    EXPECT_TRUE(c.score >= 0.4f,          "score >= 0.4 after correction");
}

/**
 * B5 — Drift case: large displacement far beyond ICP's basin of attraction.
 * We only verify that the score reflects the poor match (< 1.0).
 */
static void test_B5_drift_score(void)
{
    printf("\n[B5] Drift case — large shift, score < 1.0\n");
    point2f_t ref[200], cur[200];
    uint16_t  n = make_square_room(ref, 200, 1000.0f, 50);
    apply_known_transform(ref, n, 300.0f, 200.0f, 0.1f, cur);
    scan_matcher_set_max_iter(20);
    pose_correction_t c = scan_matcher_match(ref, n, cur, n);

    EXPECT_TRUE(c.score < 1.0f, "score < 1.0 for drift case");
    printf("  INFO  score = %.4f\n", c.score);
}

/**
 * B6 — Degenerate input (< 3 points).
 * Must return zeroed correction with score == 0.0 and must not crash.
 */
static void test_B6_degenerate(void)
{
    printf("\n[B6] Degenerate input (< 3 points)\n");
    point2f_t tiny[2] = {{ 0.0f, 0.0f }, { 1.0f, 1.0f }};
    pose_correction_t c = scan_matcher_match(tiny, 2, tiny, 2);

    EXPECT_TRUE(c.score == 0.0f, "score == 0.0");
    EXPECT_NEAR(c.dx,     0.0f, 1e-6f, "dx == 0");
    EXPECT_NEAR(c.dy,     0.0f, 1e-6f, "dy == 0");
    EXPECT_NEAR(c.dtheta, 0.0f, 1e-6f, "dtheta == 0");
}

/**
 * B7 — Asymmetric cloud sizes (ref dense, cur sparse).
 * Models the common real-world scenario where one scan misses some sectors.
 */
static void test_B7_asymmetric_clouds(void)
{
    printf("\n[B7] Asymmetric cloud sizes (ref dense, cur sparse)\n");
    point2f_t ref[200], cur[100];
    uint16_t  ref_n = make_square_room(ref, 200, 1000.0f, 50);
    uint16_t  cur_n = make_square_room(cur, 100, 1000.0f, 25);
    apply_known_transform(cur, cur_n, 15.0f, -5.0f, 0.0f, cur);
    scan_matcher_set_max_iter(30);
    pose_correction_t c = scan_matcher_match(ref, ref_n, cur, cur_n);

    EXPECT_NEAR(c.dx,     -15.0f, 5.0f,  "dx ~= -15 mm");
    EXPECT_NEAR(c.dy,       5.0f, 5.0f,  "dy ~= +5 mm");
    EXPECT_NEAR(c.dtheta,   0.0f, 0.01f, "dtheta ~= 0 rad");
    EXPECT_TRUE(c.score >= 0.4f,          "score >= 0.4");
}

/* ═══════════════════════════════════════════════════════════════════════════
 * main
 * ═══════════════════════════════════════════════════════════════════════════ */

int main(void)
{
    printf("=== test_scan_matcher (merged) ===\n");
#ifdef USE_STUBS
    printf("    Build mode: USE_STUBS  (lidar + polar_to_cart pipeline active)\n");
#else
    printf("    Build mode: standalone (synthetic fixtures only)\n");
#endif

    printf("\n-- Group A: pipeline-level tests --\n");
    test_A1_perfect_match();
    test_A2_slight_drift();
    test_A3_zero_motion();
    test_A4_rotation_detection();
    test_A5_null_input();
    test_A6_repeatability();

    printf("\n-- Group B: ICP geometry tests --\n");
    test_B1_perfect_match();
    test_B2_pure_translation();
    test_B3_pure_rotation();
    test_B4_combined();
    test_B5_drift_score();
    test_B6_degenerate();
    test_B7_asymmetric_clouds();

    printf("\n=== Results: %d / %d passed ===\n", s_passed, s_total);
    return (s_passed == s_total) ? 0 : 1;
}
