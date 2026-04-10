/**
 * mock_quadtree.c
 * Module: Mock quadtree — flat-grid substitute for offline frontier testing.
 * Board: PC (compiled without hardware, -DUSE_STUBS)
 *
 * Implements the full quadtree_map_t public API using a static flat uint8_t
 * array stored in map->root. This lets frontier_detector.c call
 * quadtree_map_query() normally without knowing it is talking to a flat grid.
 *
 * Link this file INSTEAD OF quadtree_map.c for frontier detector tests.
 */

#include "mock_quadtree.h"
#include <string.h>
#include <math.h>

/* ── Flat grid storage ───────────────────────────────────────────────────── */
/* One static grid shared across the process lifetime — fine for unit tests. */
static uint8_t s_grid[MOCK_GRID_H][MOCK_GRID_W];

/* ── quadtree_map public API (mock implementations) ─────────────────────── */

void quadtree_map_init(quadtree_map_t *map,
                       float width_mm, float height_mm, float resolution_mm)
{
    memset(s_grid, 128, sizeof(s_grid));   /* all unknown */
    map->root          = (void *)s_grid;
    map->resolution_mm = resolution_mm;
    map->width_mm      = width_mm;
    map->height_mm     = height_mm;
}

uint8_t quadtree_map_query(const quadtree_map_t *map, float x, float y)
{
    if (!map || !map->root) return 128;    /* unknown if uninitialised */

    int ix = (int)(x / map->resolution_mm);
    int iy = (int)(y / map->resolution_mm);
    int mw = (int)(map->width_mm  / map->resolution_mm);
    int mh = (int)(map->height_mm / map->resolution_mm);

    if (ix < 0 || iy < 0 || ix >= mw || iy >= mh) return 128;

    /* map->root is the flat grid: uint8_t[mh][mw] laid out row-major */
    const uint8_t *grid = (const uint8_t *)map->root;
    return grid[iy * mw + ix];
}

void quadtree_map_insert(quadtree_map_t *map,
                         float x, float y, semantic_class_t cls)
{
    if (!map || !map->root) return;

    int ix = (int)(x / map->resolution_mm);
    int iy = (int)(y / map->resolution_mm);
    int mw = (int)(map->width_mm  / map->resolution_mm);
    int mh = (int)(map->height_mm / map->resolution_mm);

    if (ix < 0 || iy < 0 || ix >= mw || iy >= mh) return;

    uint8_t *grid = (uint8_t *)map->root;
    /* Map semantic class to occupancy value */
    uint8_t occ;
    switch (cls) {
        case CLASS_WALL:
        case CLASS_OBSTACLE: occ = 255; break;
        case CLASS_UNKNOWN:  occ = 128; break;
        default:             occ =  20; break;   /* free-ish */
    }
    grid[iy * mw + ix] = occ;
}

void quadtree_map_free(quadtree_map_t *map)
{
    /* Static grid — nothing to free. Just null the root pointer. */
    if (map) map->root = (void *)0;
}

/* ── get_populated_map ───────────────────────────────────────────────────── */
/*
 * Room layout (30 x 20 cells, 50 mm/cell → 1500 mm x 1000 mm):
 *
 *   ##############################   row 0  (top wall)
 *   #............................#   rows 1-18 (interior)
 *   #............................#
 *   #.........FFFFFFFFFF.........#   ← frontier ring around free zone
 *   #.........FFFFFFFFFF.........#
 *   #.........FF      FF.........#   ← free cells (robot start = centre)
 *   #.........FF  [R] FF.........#
 *   #.........FF      FF.........#
 *   #.........FFFFFFFFFF.........#
 *   #.........FFFFFFFFFF.........#
 *   #............................#
 *   ##############################   row 19 (bottom wall)
 *
 *   # = occupied (255)   wall cells
 *   . = unknown  (128)   robot hasn't observed these yet
 *   F = frontier         (free cell adjacent to unknown → detected by WFD)
 *   space = free (20)    area visible from robot starting position
 *   [R] = robot start cell (15, 10)
 *
 * The frontier ring is produced automatically: free cells whose 4-connected
 * neighbours include at least one unknown cell. No manual annotation needed.
 */
quadtree_map_t get_populated_map(void)
{
    /* Initialise everything to unknown */
    memset(s_grid, 128, sizeof(s_grid));

    /* ── Perimeter walls ─────────────────────────────────────── */
    for (int x = 0; x < MOCK_GRID_W; x++) {
        s_grid[0][x]               = 255;   /* top wall    */
        s_grid[MOCK_GRID_H - 1][x] = 255;   /* bottom wall */
    }
    for (int y = 0; y < MOCK_GRID_H; y++) {
        s_grid[y][0]               = 255;   /* left wall   */
        s_grid[y][MOCK_GRID_W - 1] = 255;   /* right wall  */
    }

    /* ── Free zone around robot start ────────────────────────── */
    /* Circular free area of radius FREE_RADIUS cells centred on robot start.
     * The outermost ring of free cells borders unknown cells → becomes frontier. */
    const int FREE_RADIUS = 6;
    for (int dy = -FREE_RADIUS; dy <= FREE_RADIUS; dy++) {
        for (int dx = -FREE_RADIUS; dx <= FREE_RADIUS; dx++) {
            if (dx * dx + dy * dy > FREE_RADIUS * FREE_RADIUS) continue;
            int ix = MOCK_ROBOT_IX + dx;
            int iy = MOCK_ROBOT_IY + dy;
            if (ix < 1 || iy < 1 ||
                ix >= MOCK_GRID_W - 1 || iy >= MOCK_GRID_H - 1) continue;
            s_grid[iy][ix] = 20;   /* free */
        }
    }

    quadtree_map_t map;
    map.root          = (void *)s_grid;
    map.resolution_mm = MOCK_RES_MM;
    map.width_mm      = MOCK_GRID_W * MOCK_RES_MM;
    map.height_mm     = MOCK_GRID_H * MOCK_RES_MM;
    return map;
}

/* ── get_mock_robot_pose ─────────────────────────────────────────────────── */
pose_t get_mock_robot_pose(void)
{
    pose_t p = {0};
    p.x     = (MOCK_ROBOT_IX + 0.5f) * MOCK_RES_MM;   /* 775 mm */
    p.y     = (MOCK_ROBOT_IY + 0.5f) * MOCK_RES_MM;   /* 525 mm */
    p.theta = 0.0f;
    return p;
}
