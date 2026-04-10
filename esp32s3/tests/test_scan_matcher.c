/**
 * test_scan_matcher.c
 * Tests for the scan_matcher module (ICP-based scan matching).
 * Board: PC (compiled with gcc -DUSE_STUBS)
 * Run: gcc -DUSE_STUBS test_scan_matcher.c ../stubs/lidar_stub.c
 *           ../stubs/scan_match_stub.c ../stubs/pose_stub.c
 *           ../src/scan_matcher.c ../src/polar_to_cart.c -lm -o test_scan_matcher
 */

#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include "../../types.h"

#ifdef USE_STUBS
#include "../stubs/lidar_stub.h"
#include "../stubs/scan_match_stub.h"
#include "../stubs/pose_stub.h"
#endif

#include "../src/scan_matcher.h"
#include "../src/polar_to_cart.h"

/* ------------------------------------------------------------------ */
/* Helper: convert a lidar_scan_t to a point2f_t array                */
/* ------------------------------------------------------------------ */
static uint16_t scan_to_cart(const lidar_scan_t *scan, point2f_t *pts)
{
    uint16_t count = 0;
    polar_to_cart_convert(scan, pts, &count);
    return count;
}

/* ------------------------------------------------------------------ */
/* Test 1: perfect match returns score >= 1.0                          */
/* ------------------------------------------------------------------ */
static int test_perfect_match(void)
{
    printf("Test 1: perfect match returns score >= 1.0 ... ");

#ifdef USE_STUBS
    /* Use stub: inject a known-perfect correction and verify score */
    pose_correction_t c = scan_match_stub_perfect();
    if (c.score >= 1.0f) {
        printf("PASS\n");
        return 1;
    } else {
        printf("FAIL (score=%.3f, expected >= 1.0)\n", c.score);
        return 0;
    }
#else
    /* Real path: get two identical scans, match them, check score */
    lidar_scan_t scan_a = lidar_stub_room_scan();
    lidar_scan_t scan_b = lidar_stub_room_scan(); /* identical */

    point2f_t pts_a[460], pts_b[460];
    uint16_t  n_a = scan_to_cart(&scan_a, pts_a);
    uint16_t  n_b = scan_to_cart(&scan_b, pts_b);

    scan_matcher_set_max_iter(20);
    pose_correction_t c = scan_matcher_match(pts_a, n_a, pts_b, n_b);

    if (c.score >= 1.0f) {
        printf("PASS\n");
        return 1;
    } else {
        printf("FAIL (score=%.3f, expected >= 1.0)\n", c.score);
        return 0;
    }
#endif
}

/* ------------------------------------------------------------------ */
/* Test 2: slight drift detection — dx > 0 and score < 1.0            */
/* ------------------------------------------------------------------ */
static int test_slight_drift(void)
{
    printf("Test 2: slight drift detection (dx > 0, score < 1.0) ... ");

#ifdef USE_STUBS
    pose_correction_t c = scan_match_stub_slight_drift();
    if (c.dx > 0.0f && c.score < 1.0f) {
        printf("PASS\n");
        return 1;
    } else {
        printf("FAIL (dx=%.2f, score=%.3f)\n", c.dx, c.score);
        return 0;
    }
#else
    /* Real path: match a reference scan against a perturbed scan */
    lidar_scan_t scan_ref  = lidar_stub_room_scan();
    lidar_scan_t scan_pert = lidar_stub_room_scan_perturbed();

    point2f_t pts_ref[460], pts_pert[460];
    uint16_t  n_ref  = scan_to_cart(&scan_ref,  pts_ref);
    uint16_t  n_pert = scan_to_cart(&scan_pert, pts_pert);

    scan_matcher_set_max_iter(20);
    pose_correction_t c = scan_matcher_match(pts_ref, n_ref, pts_pert, n_pert);

    if (c.dx > 0.0f && c.score < 1.0f) {
        printf("PASS\n");
        return 1;
    } else {
        printf("FAIL (dx=%.2f, score=%.3f)\n", c.dx, c.score);
        return 0;
    }
#endif
}

/* ------------------------------------------------------------------ */
/* main                                                                 */
/* ------------------------------------------------------------------ */
int main(void)
{
    printf("=== test_scan_matcher ===\n");

    int pass = 0;
    int total = 0;

    total++; pass += test_perfect_match();
    total++; pass += test_slight_drift();

    printf("Result: %d/%d PASS\n", pass, total);
    return (pass == total) ? 0 : 1;
}
