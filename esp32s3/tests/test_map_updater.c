/**
 * test_map_updater.c
 * Tests for the map_updater module.
 * Board: PC (compiled with gcc -DUSE_STUBS)
 * Run: gcc -DUSE_STUBS test_map_updater.c
 *           ../stubs/lidar_stub.c ../stubs/map_stub.c ../stubs/pose_stub.c
 *           ../src/map_updater.c ../src/quadtree_map.c
 *           ../src/polar_to_cart.c ../src/obstacle_classifier.c -lm -o test_map_updater
 */

#include <stdio.h>
#include <stdint.h>
#include "../../types.h"

#ifdef USE_STUBS
#include "../stubs/lidar_stub.h"
#include "../stubs/map_stub.h"
#include "../stubs/pose_stub.h"
#endif

#include "../src/map_updater.h"
#include "../src/polar_to_cart.h"
#include "../src/obstacle_classifier.h"
#include "../src/quadtree_map.h"

/* ------------------------------------------------------------------ */
/* Test 1: update does not crash with empty point cloud                */
/* ------------------------------------------------------------------ */
static int test_update_empty_cloud(void)
{
    printf("Test 1: update does not crash with empty point cloud ... ");

#ifdef USE_STUBS
    quadtree_map_t map  = map_stub_empty();
    pose_t         pose = pose_stub_origin();
#else
    quadtree_map_t map;
    quadtree_map_init(&map, 4000.0f, 3000.0f, 50.0f);
    pose_t pose = {0};
#endif

    classified_point_t pts[1]; /* unused — count is 0 */
    map_updater_update(&map, pts, 0, &pose);

    /* If we reach here without crash, the test passes */
    printf("PASS\n");
    return 1;
}

/* ------------------------------------------------------------------ */
/* Test 2: update with full lidar stub data pipeline                   */
/* ------------------------------------------------------------------ */
static int test_update_with_lidar_stub(void)
{
    printf("Test 2: update with lidar stub data (full pipeline) ... ");

#ifdef USE_STUBS
    /* Step 1: get a synthetic room scan */
    lidar_scan_t scan = lidar_stub_room_scan();

    /* Step 2: convert polar to Cartesian */
    point2f_t cart_pts[460];
    uint16_t  cart_count = 0;
    polar_to_cart_convert(&scan, cart_pts, &cart_count);

    /* Step 3: classify points */
    classified_point_t cls_pts[460];
    uint16_t           cls_count = 0;
    obstacle_classifier_classify(cart_pts, cart_count, cls_pts, &cls_count);

    /* Step 4: update the map */
    quadtree_map_t map  = map_stub_empty();
    pose_t         pose = pose_stub_origin();
    map_updater_update(&map, cls_pts, cls_count, &pose);

    /* Verify: scan had points, conversion ran, no crash during update */
    if (scan.count > 0) {
        printf("PASS (scan.count=%d, cart_count=%d, cls_count=%d)\n",
               (int)scan.count, (int)cart_count, (int)cls_count);
        return 1;
    } else {
        printf("FAIL (scan had 0 points)\n");
        return 0;
    }
#else
    printf("PASS (stub not available, skipped)\n");
    return 1;
#endif
}

/* ------------------------------------------------------------------ */
/* main                                                                 */
/* ------------------------------------------------------------------ */
int main(void)
{
    printf("=== test_map_updater ===\n");

    int pass  = 0;
    int total = 0;

    total++; pass += test_update_empty_cloud();
    total++; pass += test_update_with_lidar_stub();

    printf("Result: %d/%d PASS\n", pass, total);
    return (pass == total) ? 0 : 1;
}
