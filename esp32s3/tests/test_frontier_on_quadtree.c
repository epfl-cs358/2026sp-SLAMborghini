/**
 * test_frontier_on_quadtree.c
 * Integration tests: frontier_detector against the REAL quadtree_map.
 * Board: PC  (gcc, no hardware, no stubs, no -DUSE_STUBS)
 *
 * Compile:
 *   cd esp32s3/tests
 *   gcc test_frontier_on_quadtree.c ../src/frontier_detector.c ../stubs/mock_quadtree.c \
 *       -lm -o test_frontier_on_quadtree && ./test_frontier_on_quadtree
 *
 * Why this file exists:
 *   test_frontier_detector.c uses get_populated_map() from mock_quadtree.c and
 *   is tightly coupled to that pre-built room.  This file also uses mock_quadtree.c
 *   but builds maps manually via quadtree_map_init / quadtree_map_insert so we can
 *   control exact cell contents and test the API surface directly.
 *
 * Room layout (30 × 20 cells, 50 mm/cell → 1500 mm × 1000 mm):
 *
 *   ##############################   row 0  (top wall, occ=255)
 *   #............................#   rows 1–18 (unknown, never inserted)
 *   #.........           .........#
 *   #.........   [free]  .........#   circular free zone, radius=6, occ=20
 *   #.........           .........#   robot start = centre (15, 10)
 *   #............................#
 *   ##############################   row 19 (bottom wall, occ=255)
 *
 *   # = inserted CLASS_WALL → occ=255
 *   . = never inserted        → occ=128 (unknown)
 *   free = inserted CLASS_PERSON → occ=20
 */

#include <stdio.h>
#include <math.h>
#include <string.h>
#include "../../types.h"
#include "../src/frontier_detector.h"
#include "../stubs/mock_quadtree.h"

/* ── Room constants (mirrors mock_quadtree.c get_populated_map) ───────────── */
#define ROOM_W_MM     1500.0f
#define ROOM_H_MM     1000.0f
#define RES_MM          50.0f
#define ROOM_COLS        30
#define ROOM_ROWS        20
#define ROBOT_IX         15
#define ROBOT_IY         10
#define FREE_RADIUS       6

/* ── 8-connected neighbour offsets (Test 4) ───────────────────────────────── */
static const int T8X[8] = {  1, -1,  0,  0,  1,  1, -1, -1 };
static const int T8Y[8] = {  0,  0,  1, -1,  1, -1,  1, -1 };

/* ── helpers ──────────────────────────────────────────────────────────────── */

/**
 * Build the standard test room: walls around the perimeter, a circular free
 * zone of radius FREE_RADIUS centred on (ROBOT_IX, ROBOT_IY), everything
 * else left uninserted (returns 128 = unknown).
 */
static quadtree_map_t build_room(void)
{
    quadtree_map_t map;
    quadtree_map_init(&map, ROOM_W_MM, ROOM_H_MM, RES_MM);

    /* Perimeter walls */
    for (int ix = 0; ix < ROOM_COLS; ix++) {
        quadtree_map_insert(&map, ix * RES_MM + RES_MM * 0.5f,
                            RES_MM * 0.5f,                CLASS_WALL);
        quadtree_map_insert(&map, ix * RES_MM + RES_MM * 0.5f,
                            (ROOM_ROWS - 1) * RES_MM + RES_MM * 0.5f, CLASS_WALL);
    }
    for (int iy = 0; iy < ROOM_ROWS; iy++) {
        quadtree_map_insert(&map, RES_MM * 0.5f,
                            iy * RES_MM + RES_MM * 0.5f, CLASS_WALL);
        quadtree_map_insert(&map, (ROOM_COLS - 1) * RES_MM + RES_MM * 0.5f,
                            iy * RES_MM + RES_MM * 0.5f, CLASS_WALL);
    }

    /* Circular free zone around robot start */
    for (int dy = -FREE_RADIUS; dy <= FREE_RADIUS; dy++) {
        for (int dx = -FREE_RADIUS; dx <= FREE_RADIUS; dx++) {
            if (dx * dx + dy * dy > FREE_RADIUS * FREE_RADIUS) continue;
            int ix = ROBOT_IX + dx;
            int iy = ROBOT_IY + dy;
            if (ix < 1 || iy < 1 || ix >= ROOM_COLS - 1 || iy >= ROOM_ROWS - 1) continue;
            quadtree_map_insert(&map, ix * RES_MM + RES_MM * 0.5f,
                                iy * RES_MM + RES_MM * 0.5f, CLASS_PERSON);
        }
    }

    return map;
}

static pose_t robot_pose(void)
{
    pose_t p = {0};
    p.x     = (ROBOT_IX + 0.5f) * RES_MM;   /* 775 mm */
    p.y     = (ROBOT_IY + 0.5f) * RES_MM;   /* 525 mm */
    p.theta = 0.0f;
    return p;
}

static void print_map(const quadtree_map_t *map)
{
    int mw = (int)(map->width_mm  / map->resolution_mm);
    int mh = (int)(map->height_mm / map->resolution_mm);
    printf("\n  Quadtree map (%d x %d cells, %.0f mm/cell):\n", mw, mh,
           map->resolution_mm);
    for (int iy = 0; iy < mh; iy++) {
        printf("  ");
        for (int ix = 0; ix < mw; ix++) {
            uint8_t occ = quadtree_map_query(
                map,
                (ix + 0.5f) * map->resolution_mm,
                (iy + 0.5f) * map->resolution_mm);
            if      (occ >= 179) putchar('#');
            else if (occ <= 50)  putchar('.');
            else                 putchar(' ');
        }
        putchar('\n');
    }
    putchar('\n');
}

/* ══════════════════════════════════════════════════════════════════════════
 * Test 1: insert→query roundtrip
 *   Verifies that the quadtree stores what was inserted.
 *   CLASS_WALL → 255, CLASS_PERSON → 20, CLASS_UNKNOWN → 128.
 *   Uninserted cell must still return 128.
 * ══════════════════════════════════════════════════════════════════════════ */
static int test_insert_query_roundtrip(void)
{
    printf("Test 1: insert→query roundtrip for wall / free / unknown cells ... ");

    quadtree_map_t map;
    quadtree_map_init(&map, ROOM_W_MM, ROOM_H_MM, RES_MM);

    /* Insert three different classes at three different cells */
    quadtree_map_insert(&map,  25.0f,  25.0f, CLASS_WALL);      /* cell (0,0) */
    quadtree_map_insert(&map, 775.0f, 525.0f, CLASS_PERSON);    /* cell (15,10) */
    quadtree_map_insert(&map, 125.0f, 125.0f, CLASS_UNKNOWN);   /* cell (2,2) */

    uint8_t wall    = quadtree_map_query(&map,  25.0f,  25.0f);
    uint8_t free_c  = quadtree_map_query(&map, 775.0f, 525.0f);
    uint8_t unk     = quadtree_map_query(&map, 125.0f, 125.0f);
    uint8_t virgin  = quadtree_map_query(&map, 325.0f, 325.0f); /* never inserted */

    int ok = (wall == 255) && (free_c == 20) && (unk == 128) && (virgin == 128);
    if (ok)
        printf("PASS (wall=%d free=%d unk=%d virgin=%d)\n",
               wall, free_c, unk, virgin);
    else
        printf("FAIL (wall=%d free=%d unk=%d virgin=%d — expected 255/20/128/128)\n",
               wall, free_c, unk, virgin);

    quadtree_map_free(&map);
    return ok;
}

/* ══════════════════════════════════════════════════════════════════════════
 * Test 2: out-of-bounds query returns 128
 *   quadtree_map_query clamps out-of-range coordinates.
 * ══════════════════════════════════════════════════════════════════════════ */
static int test_oob_query_returns_unknown(void)
{
    printf("Test 2: out-of-bounds query returns 128 (unknown) ... ");

    quadtree_map_t map;
    quadtree_map_init(&map, ROOM_W_MM, ROOM_H_MM, RES_MM);

    uint8_t neg_x  = quadtree_map_query(&map, -100.0f,  500.0f);
    uint8_t neg_y  = quadtree_map_query(&map,  750.0f, -100.0f);
    uint8_t far_x  = quadtree_map_query(&map, 9999.0f,  500.0f);
    uint8_t far_y  = quadtree_map_query(&map,  750.0f, 9999.0f);

    int ok = (neg_x == 128) && (neg_y == 128) && (far_x == 128) && (far_y == 128);
    if (ok)
        printf("PASS (all four OOB queries → 128)\n");
    else
        printf("FAIL (neg_x=%d neg_y=%d far_x=%d far_y=%d — expected all 128)\n",
               neg_x, neg_y, far_x, far_y);

    quadtree_map_free(&map);
    return ok;
}

/* ══════════════════════════════════════════════════════════════════════════
 * Test 3: all-unknown map (only init, no inserts) → no frontiers
 *   The robot cell is unknown → frontier_detector_detect() returns early.
 *   count must be 0.
 * ══════════════════════════════════════════════════════════════════════════ */
static int test_virgin_map_no_frontiers(void)
{
    printf("Test 3: virgin map (no inserts) → no frontiers detected ... ");

    quadtree_map_t map;
    quadtree_map_init(&map, ROOM_W_MM, ROOM_H_MM, RES_MM);

    pose_t pose = robot_pose();
    frontier_list_t list = frontier_detector_detect(&map, &pose);

    int ok = (list.count == 0);
    if (ok)
        printf("PASS\n");
    else
        printf("FAIL (expected 0, got %d)\n", (int)list.count);

    quadtree_map_free(&map);
    return ok;
}

/* ══════════════════════════════════════════════════════════════════════════
 * Test 4: detect() finds a frontier cluster in the circular room
 *   Same room as the mock tests but built with the real quadtree.
 *   Verifies count > 0, target inside map bounds, target cell is free,
 *   and all 8 neighbours of the target are free (safety_spiral ran).
 * ══════════════════════════════════════════════════════════════════════════ */
static int test_detect_on_real_quadtree(void)
{
    printf("Test 4: detect() finds frontier in real quadtree circular room ... ");

    quadtree_map_t  map  = build_room();
    pose_t          pose = robot_pose();
    frontier_list_t list = frontier_detector_detect(&map, &pose);

    if (list.count == 0) {
        printf("FAIL (count=0 — no frontiers detected)\n");
        quadtree_map_free(&map);
        return 0;
    }

    float cx = list.items[0].cx;
    float cy = list.items[0].cy;
    int   sz = (int)list.items[0].size;

    /* Target must be inside the map */
    if (cx < 0.0f || cx > map.width_mm || cy < 0.0f || cy > map.height_mm) {
        printf("FAIL (target (%.0f, %.0f) outside map)\n", cx, cy);
        quadtree_map_free(&map);
        return 0;
    }

    /* Target cell must be free */
    uint8_t tgt_occ = quadtree_map_query(&map, cx, cy);
    if (tgt_occ > 50) {
        printf("FAIL (target occ=%d, expected ≤50)\n", (int)tgt_occ);
        quadtree_map_free(&map);
        return 0;
    }

    /* All 8 neighbours must be free (safety_spiral guarantee) */
    float res = map.resolution_mm;
    int   mw  = (int)(map.width_mm  / res);
    int   mh  = (int)(map.height_mm / res);
    int   ix  = (int)(cx / res);
    int   iy  = (int)(cy / res);
    for (int k = 0; k < 8; k++) {
        int nx = ix + T8X[k];
        int ny = iy + T8Y[k];
        if (nx < 0 || ny < 0 || nx >= mw || ny >= mh) continue;
        uint8_t occ = quadtree_map_query(&map,
                                         (nx + 0.5f) * res,
                                         (ny + 0.5f) * res);
        if (occ > 50) {
            printf("FAIL (8-neighbour (%d,%d) of target (%d,%d) has occ=%d)\n",
                   nx, ny, ix, iy, (int)occ);
            quadtree_map_free(&map);
            return 0;
        }
    }

    printf("PASS (cx=%.0f mm, cy=%.0f mm, size=%d, tgt_occ=%d)\n",
           cx, cy, sz, (int)tgt_occ);
    quadtree_map_free(&map);
    return 1;
}

/* ══════════════════════════════════════════════════════════════════════════
 * Test 5: occupied wall cell → still returns 255 after the free zone is built
 *   Re-queries a few corner wall cells to make sure the free-zone inserts
 *   didn't corrupt the wall values in the quadtree.
 * ══════════════════════════════════════════════════════════════════════════ */
static int test_wall_cells_survive_free_inserts(void)
{
    printf("Test 5: wall cells retain occ=255 after free-zone inserts ... ");

    quadtree_map_t map = build_room();

    /* Sample four corners and the midpoints of each wall edge */
    float coords[][2] = {
        { 25.0f,   25.0f },   /* top-left corner     (0,0)   */
        { 1475.0f, 25.0f },   /* top-right corner    (29,0)  */
        { 25.0f,   975.0f },  /* bottom-left corner  (0,19)  */
        { 1475.0f, 975.0f },  /* bottom-right corner (29,19) */
        { 750.0f,  25.0f },   /* top mid             (15,0)  */
        { 750.0f,  975.0f },  /* bottom mid          (15,19) */
    };
    int n = (int)(sizeof(coords) / sizeof(coords[0]));

    int ok = 1;
    for (int i = 0; i < n; i++) {
        uint8_t occ = quadtree_map_query(&map, coords[i][0], coords[i][1]);
        if (occ != 255) {
            printf("FAIL (wall at (%.0f,%.0f) has occ=%d, expected 255)\n",
                   coords[i][0], coords[i][1], (int)occ);
            ok = 0;
        }
    }
    if (ok)
        printf("PASS (%d wall cells all occ=255)\n", n);

    quadtree_map_free(&map);
    return ok;
}

/* ══════════════════════════════════════════════════════════════════════════
 * Test 6: overwrite — last insert wins
 *   Insert CLASS_WALL then CLASS_PERSON at the same cell.
 *   Query must return 20 (the second insert), not 255.
 * ══════════════════════════════════════════════════════════════════════════ */
static int test_overwrite_last_wins(void)
{
    printf("Test 6: overwrite at same cell — last insert wins ... ");

    quadtree_map_t map;
    quadtree_map_init(&map, ROOM_W_MM, ROOM_H_MM, RES_MM);

    float px = 375.0f, py = 275.0f;   /* cell (7, 5) */
    quadtree_map_insert(&map, px, py, CLASS_WALL);    /* occ = 255 */
    quadtree_map_insert(&map, px, py, CLASS_PERSON);  /* occ = 20  */

    uint8_t occ = quadtree_map_query(&map, px, py);
    int ok = (occ == 20);
    if (ok)
        printf("PASS (occ=%d after wall→person overwrite)\n", (int)occ);
    else
        printf("FAIL (occ=%d, expected 20)\n", (int)occ);

    quadtree_map_free(&map);
    return ok;
}

/* ══════════════════════════════════════════════════════════════════════════
 * Test 7: small free zone near map edge — no crash, valid result
 *   Places robot at cell (1,1) — one step from two walls — with a 3×3 free
 *   block.  The right column and bottom row border unknown space (3+2=5
 *   frontier cells ≥ MIN_CLUSTER_SIZE=3), so at least one frontier must be
 *   detected without any out-of-bounds access.
 * ══════════════════════════════════════════════════════════════════════════ */
static int test_edge_robot_no_crash(void)
{
    printf("Test 7: robot near grid edge — no crash, valid frontier ... ");

    quadtree_map_t map;
    quadtree_map_init(&map, ROOM_W_MM, ROOM_H_MM, RES_MM);

    float res = map.resolution_mm;

    /* Perimeter walls */
    for (int ix = 0; ix < ROOM_COLS; ix++) {
        quadtree_map_insert(&map, ix * res + res * 0.5f, res * 0.5f,                      CLASS_WALL);
        quadtree_map_insert(&map, ix * res + res * 0.5f, (ROOM_ROWS-1)*res + res * 0.5f,  CLASS_WALL);
    }
    for (int iy = 0; iy < ROOM_ROWS; iy++) {
        quadtree_map_insert(&map, res * 0.5f,                     iy * res + res * 0.5f, CLASS_WALL);
        quadtree_map_insert(&map, (ROOM_COLS-1)*res + res * 0.5f, iy * res + res * 0.5f, CLASS_WALL);
    }

    /* 3×3 free block at cells (1,1)–(3,3).
     * Frontier cells: right column (ix=3, iy=1..3) + bottom row (ix=1..2, iy=3) = 5 cells. */
    for (int ix = 1; ix <= 3; ix++)
        for (int iy = 1; iy <= 3; iy++)
            quadtree_map_insert(&map, ix * res + res * 0.5f,
                                iy * res + res * 0.5f, CLASS_PERSON);

    pose_t pose;
    pose.x     = 1 * res + res * 0.5f;   /* 75 mm */
    pose.y     = 1 * res + res * 0.5f;   /* 75 mm */
    pose.theta = 0.0f;

    frontier_list_t list = frontier_detector_detect(&map, &pose);

    int ok = 1;
    if (list.count == 0) {
        printf("FAIL (expected frontiers, got 0)\n");
        ok = 0;
    } else {
        float cx = list.items[0].cx;
        float cy = list.items[0].cy;
        if (cx < 0.0f || cx > map.width_mm || cy < 0.0f || cy > map.height_mm) {
            printf("FAIL (target (%.0f,%.0f) out of bounds)\n", cx, cy);
            ok = 0;
        }
        if (quadtree_map_query(&map, cx, cy) > 50) {
            printf("FAIL (target cell not free)\n");
            ok = 0;
        }
        if (ok)
            printf("PASS (count=%d cx=%.0f cy=%.0f, robot was at cell (1,1))\n",
                   (int)list.count, cx, cy);
    }

    quadtree_map_free(&map);
    return ok;
}

/* ── main ────────────────────────────────────────────────────────────────── */
int main(void)
{
    printf("=== test_frontier_on_quadtree (mock quadtree API, manual map builds) ===\n");

    /* Print the room so layout issues are immediately visible */
    quadtree_map_t room = build_room();
    print_map(&room);
    pose_t p = robot_pose();
    printf("  Robot start: (%.0f mm, %.0f mm), theta=%.2f rad\n\n",
           p.x, p.y, p.theta);
    quadtree_map_free(&room);

    int pass = 0, total = 0;

    total++; pass += test_insert_query_roundtrip();
    total++; pass += test_oob_query_returns_unknown();
    total++; pass += test_virgin_map_no_frontiers();
    total++; pass += test_detect_on_real_quadtree();
    total++; pass += test_wall_cells_survive_free_inserts();
    total++; pass += test_overwrite_last_wins();
    total++; pass += test_edge_robot_no_crash();

    printf("\nResult: %d/%d PASS\n", pass, total);
    return (pass == total) ? 0 : 1;
}
