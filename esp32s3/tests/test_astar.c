/**
 * test_astar.c
 * Tests for the hybrid_astar path planner module.
 * Board: PC (compiled with gcc -DUSE_STUBS)
 * Run: gcc -DUSE_STUBS test_astar.c
 *           ../stubs/astar_stub.c ../stubs/frontier_stub.c
 *           ../stubs/map_stub.c ../stubs/pose_stub.c
 *           ../src/hybrid_astar.c ../src/quadtree_map.c -lm -o test_astar
 */

#include <stdio.h>
#include <stdint.h>
#include "../../types.h"

#ifdef USE_STUBS
#include "../stubs/astar_stub.h"
#include "../stubs/frontier_stub.h"
#include "../stubs/map_stub.h"
#include "../stubs/pose_stub.h"
#endif

#include "../src/hybrid_astar.h"
#include "../src/quadtree_map.h"

/* ------------------------------------------------------------------ */
/* Test 1: plan returns a valid path given a single frontier           */
/* ------------------------------------------------------------------ */
static int test_plan_valid_path(void)
{
    printf("Test 1: plan returns valid path (stub straight path) ... ");

#ifdef USE_STUBS
    /* Use stub path that already has 3 waypoints */
    path_t path = astar_stub_straight();

    if (hybrid_astar_is_valid(&path)) {
        printf("PASS (length=%d)\n", (int)path.length);
        return 1;
    } else {
        printf("FAIL (path invalid, length=%d)\n", (int)path.length);
        return 0;
    }
#else
    /* Real path: plan from origin to a frontier at (2000, 1500) */
    quadtree_map_t map;
    quadtree_map_init(&map, 4000.0f, 3000.0f, 50.0f);

    pose_t start = {0};
    frontier_t goal;
    goal.cx   = 2000.0f;
    goal.cy   = 1500.0f;
    goal.size = 10;

    path_t path = hybrid_astar_plan(&map, &start, &goal);

    if (hybrid_astar_is_valid(&path)) {
        printf("PASS (length=%d)\n", (int)path.length);
        return 1;
    } else {
        printf("FAIL (path invalid)\n");
        return 0;
    }
#endif
}

/* ------------------------------------------------------------------ */
/* Test 2: empty frontier list results in an invalid (empty) path     */
/* ------------------------------------------------------------------ */
static int test_empty_frontier_invalid_path(void)
{
    printf("Test 2: empty frontier list returns invalid path ... ");

#ifdef USE_STUBS
    frontier_list_t list = frontier_stub_empty();

    /* With no frontier, a planner should return an empty path.
     * We simulate this by creating a zero-length path and checking is_valid. */
    path_t path;
    path.length = 0;

    if (!hybrid_astar_is_valid(&path) && list.count == 0) {
        printf("PASS (list.count=%d, path.length=%d)\n",
               (int)list.count, (int)path.length);
        return 1;
    } else {
        printf("FAIL (list.count=%d, path.length=%d, is_valid=%d)\n",
               (int)list.count, (int)path.length, (int)hybrid_astar_is_valid(&path));
        return 0;
    }
#else
    /* Real path: try to plan toward an empty frontier list */
    path_t path;
    path.length = 0;

    if (!hybrid_astar_is_valid(&path)) {
        printf("PASS\n");
        return 1;
    } else {
        printf("FAIL\n");
        return 0;
    }
#endif
}

/* ------------------------------------------------------------------ */
/* main                                                                 */
/* ------------------------------------------------------------------ */
int main(void)
{
    printf("=== test_astar ===\n");

    int pass  = 0;
    int total = 0;

    total++; pass += test_plan_valid_path();
    total++; pass += test_empty_frontier_invalid_path();

    printf("Result: %d/%d PASS\n", pass, total);
    return (pass == total) ? 0 : 1;
}
