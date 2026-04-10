/**
 * test_frontier_detector.c
 * Tests for the frontier_detector module against a mock occupancy map.
 * Board: PC (compiled with gcc, no hardware required)
 */

#include <stdio.h>
#include <math.h>
#include "../../types.h"
#include "../src/frontier_detector.h"
#include "../stubs/mock_quadtree.h"

/* ── 8-connected neighbour offsets (reused by Test 8) ───────────────────── */
static const int T8X[8] = {  1, -1,  0,  0,  1,  1, -1, -1 };
static const int T8Y[8] = {  0,  0,  1, -1,  1, -1,  1, -1 };

/* ── helpers ──────────────────────────────────────────────────────────────── */

static void print_map(const quadtree_map_t *map)
{
    int mw = (int)(map->width_mm  / map->resolution_mm);
    int mh = (int)(map->height_mm / map->resolution_mm);
    printf("\n  Mock map (%d x %d cells, %.0f mm/cell):\n", mw, mh,
           map->resolution_mm);
    for (int iy = 0; iy < mh; iy++) {
        printf("  ");
        for (int ix = 0; ix < mw; ix++) {
            uint8_t occ = quadtree_map_query(
                map,
                (ix + 0.5f) * map->resolution_mm,
                (iy + 0.5f) * map->resolution_mm);
            if      (occ >= 179) putchar('#');          /* occupied / wall */
            else if (occ <= 50)  putchar('.');          /* free            */
            else                 putchar(' ');           /* unknown         */
        }
        putchar('\n');
    }
    putchar('\n');
}

/* ── Test 1: detect returns at least one frontier cluster ───────────────── */
static int test_detect_finds_frontiers(void)
{
    printf("Test 1: detect() finds at least one frontier cluster ... ");

    quadtree_map_t map  = get_populated_map();
    pose_t         pose = get_mock_robot_pose();

    frontier_list_t list = frontier_detector_detect(&map, &pose);

    if (list.count > 0) {
        printf("PASS (count=%d)\n", (int)list.count);
        return 1;
    } else {
        printf("FAIL (count=0 — no frontiers detected)\n");
        return 0;
    }
}

/* ── Test 2: every returned frontier has a free target cell ─────────────── */
static int test_all_targets_are_free(void)
{
    printf("Test 2: all frontier target cells are free ... ");

    quadtree_map_t map  = get_populated_map();
    pose_t         pose = get_mock_robot_pose();

    frontier_list_t list = frontier_detector_detect(&map, &pose);

    if (list.count == 0) {
        printf("SKIP (no frontiers detected)\n");
        return 1;   /* not a failure of this test */
    }

    int all_free = 1;
    for (int i = 0; i < list.count; i++) {
        uint8_t occ = quadtree_map_query(&map, list.items[i].cx, list.items[i].cy);
        if (occ > 50) {
            printf("FAIL (frontier[%d] at (%.0f, %.0f) has occupancy %d)\n",
                   i, list.items[i].cx, list.items[i].cy, (int)occ);
            all_free = 0;
        }
    }

    if (all_free) {
        printf("PASS (%d frontiers checked)\n", (int)list.count);
        return 1;
    }
    return 0;
}

/* ── Test 3: best() returns a non-zero result from a non-empty list ──────── */
static int test_best_selects_one(void)
{
    printf("Test 3: best() selects a valid single target ... ");

    quadtree_map_t map  = get_populated_map();
    pose_t         pose = get_mock_robot_pose();

    frontier_list_t list = frontier_detector_detect(&map, &pose);

    if (list.count == 0) {
        printf("SKIP (no frontiers detected)\n");
        return 1;
    }

    frontier_t best = frontier_detector_best(&list, &pose);

    if (best.size > 0) {
        float dx   = best.cx - pose.x;
        float dy   = best.cy - pose.y;
        float dist = sqrtf(dx * dx + dy * dy);
        printf("PASS (cx=%.0f mm, cy=%.0f mm, size=%d, dist=%.0f mm)\n",
               best.cx, best.cy, (int)best.size, dist);
        return 1;
    } else {
        printf("FAIL (best.size == 0)\n");
        return 0;
    }
}

/* ── Test 4: best() on empty list returns zero-struct safely ─────────────── */
static int test_best_empty_list_safe(void)
{
    printf("Test 4: best() on empty list returns zero safely ... ");

    frontier_list_t empty = {0};
    pose_t pose = get_mock_robot_pose();

    frontier_t best = frontier_detector_best(&empty, &pose);

    if (best.size == 0 && best.cx == 0.0f && best.cy == 0.0f) {
        printf("PASS\n");
        return 1;
    } else {
        printf("FAIL (non-zero result from empty list)\n");
        return 0;
    }
}

/* ── Test 5: detect() on all-unknown map returns no frontiers ─────────────
 * A fully unknown map has no free cells → no frontiers by definition.       */
static int test_unknown_map_no_frontiers(void)
{
    printf("Test 5: all-unknown map produces no frontiers ... ");

    quadtree_map_t map;
    /* Use an empty map (root = NULL → query always returns 128 = unknown) */
    map.root          = (void *)0;
    map.resolution_mm = 50.0f;
    map.width_mm      = 1500.0f;
    map.height_mm     = 1000.0f;

    pose_t pose = get_mock_robot_pose();

    frontier_list_t list = frontier_detector_detect(&map, &pose);

    if (list.count == 0) {
        printf("PASS\n");
        return 1;
    } else {
        printf("FAIL (expected 0, got %d)\n", (int)list.count);
        return 0;
    }
}

/* ── Test 6: frontier targets in bounds and above MIN_CLUSTER_SIZE ────────
 * Test 1 only checks count > 0. This verifies each returned cx/cy actually
 * lies inside the map rectangle and that no sub-threshold cluster slipped
 * through the MIN_CLUSTER_SIZE = 3 guard.                                   */
static int test_frontier_coords_and_sizes(void)
{
    printf("Test 6: frontier cx/cy in map bounds and size >= MIN_CLUSTER_SIZE ... ");

    quadtree_map_t  map  = get_populated_map();
    pose_t          pose = get_mock_robot_pose();
    frontier_list_t list = frontier_detector_detect(&map, &pose);

    if (list.count == 0) {
        printf("SKIP (no frontiers)\n");
        return 1;
    }

    int ok = 1;
    for (int i = 0; i < list.count; i++) {
        float cx = list.items[i].cx;
        float cy = list.items[i].cy;
        int   sz = (int)list.items[i].size;

        if (cx < 0.0f || cx > map.width_mm || cy < 0.0f || cy > map.height_mm) {
            printf("FAIL (frontier[%d] cx=%.0f cy=%.0f outside %.0fx%.0f mm)\n",
                   i, cx, cy, map.width_mm, map.height_mm);
            ok = 0;
        }
        if (sz < 3) {
            printf("FAIL (frontier[%d] size=%d below MIN_CLUSTER_SIZE=3)\n", i, sz);
            ok = 0;
        }
    }
    if (ok)
        printf("PASS (%d frontiers, all in bounds and size >= 3)\n", (int)list.count);
    return ok;
}

/* ── Test 7: best() scoring — higher size/dist ratio wins regardless of
 *            array position ──────────────────────────────────────────────
 * Test 3 passes even if best() just returns items[0]. Here the higher-
 * scoring frontier is at index 1, so any "return items[0]" impl fails.
 *
 *   items[0]: size=20, dist=4000 mm  →  score ≈ 0.0050   (large but far)
 *   items[1]: size= 5, dist= 100 mm  →  score = 0.0500   ← should win     */
static int test_best_scoring_prefers_high_ratio(void)
{
    printf("Test 7: best() picks high size/dist ratio, not array order ... ");

    pose_t pose = {0};   /* robot at origin */

    frontier_list_t list = {0};
    list.count = 2;

    list.items[0].cx   = 4000.0f;
    list.items[0].cy   =    0.0f;
    list.items[0].size = 20;        /* score ≈ 0.005 */

    list.items[1].cx   =  100.0f;
    list.items[1].cy   =    0.0f;
    list.items[1].size = 5;         /* score = 0.050  ← winner */

    frontier_t best = frontier_detector_best(&list, &pose);

    if (best.cx == 100.0f && best.size == 5) {
        printf("PASS (selected items[1]: score=0.050 over items[0]: score=0.005)\n");
        return 1;
    }
    printf("FAIL (got cx=%.0f size=%d — expected cx=100 size=5)\n",
           best.cx, (int)best.size);
    return 0;
}

/* ── Test 8: safety spiral — every returned target has all 8 neighbours
 *            free (occ <= 50) ───────────────────────────────────────────
 * Frontier ring cells border unknown space by definition, so without the
 * spiral every target would be adjacent to an unknown cell — the car would
 * navigate to within one cell of unexplored space and could clip an obstacle.
 *
 * This test verifies safety_spiral ran and moved each target to an interior
 * cell where all 8 neighbours are confirmed free.  Disabling the spiral in
 * frontier_detector.c should cause this test to fail on the circular mock
 * map because every ring cell has at least one unknown 8-neighbour.         */
static int test_safety_spiral_all_neighbours_free(void)
{
    printf("Test 8: safety_spiral ensures all 8 neighbours of each target are free ... ");

    quadtree_map_t  map  = get_populated_map();
    pose_t          pose = get_mock_robot_pose();
    frontier_list_t list = frontier_detector_detect(&map, &pose);

    if (list.count == 0) {
        printf("SKIP (no frontiers)\n");
        return 1;
    }

    float res = map.resolution_mm;
    int   mw  = (int)(map.width_mm  / res);
    int   mh  = (int)(map.height_mm / res);
    int   ok  = 1;

    for (int i = 0; i < list.count; i++) {
        int ix = (int)(list.items[i].cx / res);
        int iy = (int)(list.items[i].cy / res);

        for (int k = 0; k < 8; k++) {
            int nx = ix + T8X[k];
            int ny = iy + T8Y[k];
            if (nx < 0 || ny < 0 || nx >= mw || ny >= mh) continue;
            uint8_t occ = quadtree_map_query(&map,
                                             (nx + 0.5f) * res,
                                             (ny + 0.5f) * res);
            if (occ > 50) {
                printf("FAIL (frontier[%d] target (%d,%d) has non-free "
                       "8-neighbour (%d,%d) occ=%d)\n",
                       i, ix, iy, nx, ny, (int)occ);
                ok = 0;
            }
        }
    }
    if (ok)
        printf("PASS (all %d targets have 8 free neighbours)\n", (int)list.count);
    return ok;
}

/* ── Test 9: robot near grid edge — BFS in_bounds() does not over-run ─────
 * In Tests 1-8 the robot is always at the map centre. This test places it
 * at cell (1,1) — one cell from two walls — to stress every in_bounds()
 * call in the BFS and the cluster flood-fill.
 *
 * A 4×4 free zone is built around (1,1) using quadtree_map_init +
 * quadtree_map_insert (all-unknown base, perimeter walls, then 16 free
 * cells).  The zone's right column and bottom row border unknown space, so
 * at least one cluster must be detected.  We verify no crash, count > 0,
 * all targets in-bounds, and all targets in free cells.                     */
static int test_robot_near_grid_edge(void)
{
    printf("Test 9: robot near grid edge — no crash and valid frontier found ... ");

    quadtree_map_t map;
    quadtree_map_init(&map, 1500.0f, 1000.0f, 50.0f);  /* 30×20, all unknown */

    float res = map.resolution_mm;

    /* Perimeter walls (top, bottom, left, right) */
    for (int ix = 0; ix < 30; ix++) {
        quadtree_map_insert(&map, ix * res + res * 0.5f,           res * 0.5f, CLASS_WALL);
        quadtree_map_insert(&map, ix * res + res * 0.5f, 19 * res + res * 0.5f, CLASS_WALL);
    }
    for (int iy = 0; iy < 20; iy++) {
        quadtree_map_insert(&map,           res * 0.5f, iy * res + res * 0.5f, CLASS_WALL);
        quadtree_map_insert(&map, 29 * res + res * 0.5f, iy * res + res * 0.5f, CLASS_WALL);
    }

    /* Free zone: cells (1,1) to (4,4) — 4×4 block.
     * Frontier cells: right column (ix=4, iy=1..4) + bottom row (ix=1..3, iy=4)
     * = 7 cells, which clears the MIN_CLUSTER_SIZE=3 gate.
     * CLASS_PERSON hits the default branch in mock_quadtree → occ = 20 (free). */
    for (int ix = 1; ix <= 4; ix++) {
        for (int iy = 1; iy <= 4; iy++) {
            quadtree_map_insert(&map,
                                ix * res + res * 0.5f,
                                iy * res + res * 0.5f,
                                CLASS_PERSON);
        }
    }

    /* Robot at cell (1,1) — adjacent to two walls */
    pose_t pose;
    pose.x     = 1 * res + res * 0.5f;   /* 75 mm */
    pose.y     = 1 * res + res * 0.5f;   /* 75 mm */
    pose.theta = 0.0f;

    frontier_list_t list = frontier_detector_detect(&map, &pose);

    if (list.count == 0) {
        printf("FAIL (expected frontiers around corner free zone, got 0)\n");
        return 0;
    }

    int ok = 1;
    for (int i = 0; i < list.count; i++) {
        float cx = list.items[i].cx;
        float cy = list.items[i].cy;

        if (cx < 0.0f || cx > map.width_mm || cy < 0.0f || cy > map.height_mm) {
            printf("FAIL (frontier[%d] cx=%.0f cy=%.0f out of bounds)\n", i, cx, cy);
            ok = 0;
        }
        if (quadtree_map_query(&map, cx, cy) > 50) {
            printf("FAIL (frontier[%d] target not in free cell)\n", i);
            ok = 0;
        }
    }
    if (ok)
        printf("PASS (count=%d, all in-bounds and free, robot was at cell (1,1))\n",
               (int)list.count);
    return ok;
}

/* ── main ────────────────────────────────────────────────────────────────── */
int main(void)
{
    printf("=== test_frontier_detector ===\n");

    quadtree_map_t map = get_populated_map();
    print_map(&map);

    pose_t pose = get_mock_robot_pose();
    printf("  Robot start: (%.0f mm, %.0f mm), theta=%.2f rad\n\n",
           pose.x, pose.y, pose.theta);

    int pass  = 0;
    int total = 0;

    total++; pass += test_detect_finds_frontiers();
    total++; pass += test_all_targets_are_free();
    total++; pass += test_best_selects_one();
    total++; pass += test_best_empty_list_safe();
    total++; pass += test_unknown_map_no_frontiers();
    total++; pass += test_frontier_coords_and_sizes();
    total++; pass += test_best_scoring_prefers_high_ratio();
    total++; pass += test_safety_spiral_all_neighbours_free();
    total++; pass += test_robot_near_grid_edge();

    printf("\nResult: %d/%d PASS\n", pass, total);
    return (pass == total) ? 0 : 1;
}
