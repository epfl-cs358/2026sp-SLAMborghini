/**
 * planning_grid.c
 * Module: Planner-side occupancy grid extracted from the quadtree map.
 * Board: ESP32-S3
 */

#include "planning_grid.h"

#include <stdlib.h>
#include <string.h>
#include <stddef.h>

/* Occupancy thresholds from quadtree_map_query() output:
 * 0   = definitely free
 * 128 = unknown
 * 255 = definitely occupied
 *
 * Conservative first version:
 * - low values => free
 * - middle band => unknown
 * - high values => occupied
 */
#define OCC_THRESHOLD_FREE_MAX      99
#define OCC_THRESHOLD_OCC_MIN       156
#define OCC_UNKNOWN_VALUE           128

static inline int planning_grid_index(const planning_grid_t *grid, int x, int y)
{
    return y * grid->width + x;
}

bool planning_grid_is_inside(const planning_grid_t *grid, int x, int y)
{
    if (grid == NULL) {
        return false;
    }

    return (x >= 0 && x < grid->width && y >= 0 && y < grid->height);
}

planner_cell_t planning_grid_get(const planning_grid_t *grid, int x, int y)
{
    if (!planning_grid_is_inside(grid, x, y)) {
        return CELL_OCCUPIED; /* out of bounds = blocked */
    }

    return (planner_cell_t)grid->cells[planning_grid_index(grid, x, y)];
}

void planning_grid_set(planning_grid_t *grid, int x, int y, planner_cell_t value)
{
    if (!planning_grid_is_inside(grid, x, y)) {
        return;
    }

    grid->cells[planning_grid_index(grid, x, y)] = (uint8_t)value;
}

bool planning_grid_init(planning_grid_t *grid, int width, int height, float cell_size_mm)
{
    int total_cells;

    if (grid == NULL || width <= 0 || height <= 0 || cell_size_mm <= 0.0f) {
        return false;
    }

    total_cells = width * height;

    grid->width = width;
    grid->height = height;
    grid->cell_size_mm = cell_size_mm;
    grid->cells = (uint8_t *)malloc((size_t)total_cells);

    if (grid->cells == NULL) {
        grid->width = 0;
        grid->height = 0;
        grid->cell_size_mm = 0.0f;
        return false;
    }

    /* Initialize conservatively as unknown */
    for (int i = 0; i < total_cells; ++i) {
        grid->cells[i] = (uint8_t)CELL_UNKNOWN;
    }

    return true;
}

void planning_grid_free(planning_grid_t *grid)
{
    if (grid == NULL) {
        return;
    }

    if (grid->cells != NULL) {
        free(grid->cells);
        grid->cells = NULL;
    }

    grid->width = 0;
    grid->height = 0;
    grid->cell_size_mm = 0.0f;
}

static planner_cell_t occupancy_to_planner_cell(uint8_t occ)
{
    if (occ <= OCC_THRESHOLD_FREE_MAX) {
        return CELL_FREE;
    }

    if (occ >= OCC_THRESHOLD_OCC_MIN) {
        return CELL_OCCUPIED;
    }

    return CELL_UNKNOWN;
}

bool planning_grid_build_from_quadtree(
    const quadtree_map_t *map,
    planning_grid_t *grid)
{
    if (map == NULL || grid == NULL || grid->cells == NULL) {
        return false;
    }

    for (int gy = 0; gy < grid->height; ++gy) {
        for (int gx = 0; gx < grid->width; ++gx) {
            /* Sample the center of each planning cell in map coordinates (mm) */
            float x_mm = ((float)gx + 0.5f) * grid->cell_size_mm;
            float y_mm = ((float)gy + 0.5f) * grid->cell_size_mm;

            planner_cell_t cell_value;

            /* If the planning grid exceeds map bounds, mark blocked/unknown */
            if (x_mm < map->x_min || x_mm >= map->x_max ||
                y_mm < map->y_min || y_mm >= map->y_max) {
                cell_value = CELL_OCCUPIED;
            } else {
                uint8_t occ = quadtree_map_query(map, x_mm, y_mm);
                cell_value = occupancy_to_planner_cell(occ);
            }

            planning_grid_set(grid, gx, gy, cell_value);
        }
    }

    return true;
}

void planning_grid_inflate(planning_grid_t *grid, int radius_cells)
{
    uint8_t *original;
    int total_cells;

    if (grid == NULL || grid->cells == NULL || radius_cells <= 0) {
        return;
    }

    total_cells = grid->width * grid->height;

    original = (uint8_t *)malloc((size_t)total_cells);
    if (original == NULL) {
        return;
    }

    memcpy(original, grid->cells, (size_t)total_cells);

    for (int y = 0; y < grid->height; ++y) {
        for (int x = 0; x < grid->width; ++x) {
            planner_cell_t current =
                (planner_cell_t)original[planning_grid_index(grid, x, y)];

            /* Inflate around both occupied and unknown cells for safety */
            if (current == CELL_OCCUPIED || current == CELL_UNKNOWN) {
                for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
                    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
                        int nx = x + dx;
                        int ny = y + dy;

                        if (!planning_grid_is_inside(grid, nx, ny)) {
                            continue;
                        }

                        /* Circular inflation: skip cells outside the radius */
                        if ((dx * dx + dy * dy) > (radius_cells * radius_cells)) {
                            continue;
                        }

                        planner_cell_t neighbor =
                            (planner_cell_t)grid->cells[planning_grid_index(grid, nx, ny)];

                        /* Do not overwrite hard occupied or unknown cells */
                        if (neighbor == CELL_FREE) {
                            grid->cells[planning_grid_index(grid, nx, ny)] = (uint8_t)CELL_INFLATED;
                        }
                    }
                }
            }
        }
    }

    free(original);
}
