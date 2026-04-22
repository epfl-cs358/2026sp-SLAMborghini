#ifndef PLANNING_GRID_H
#define PLANNING_GRID_H

#include <stdint.h>
#include <stdbool.h>
#include "../quadtree_map.h"

typedef enum {
    CELL_FREE = 0,
    CELL_OCCUPIED = 1,
    CELL_UNKNOWN = 2,
    CELL_INFLATED = 3
} planner_cell_t;

typedef struct {
    int width;
    int height;
    float cell_size_mm;
    planner_cell_t *cells;
} planning_grid_t;

bool planning_grid_init(planning_grid_t *grid, int width, int height, float cell_size_mm);
void planning_grid_free(planning_grid_t *grid);
bool planning_grid_build_from_quadtree(const quadtree_map_t *map, planning_grid_t *grid);
void planning_grid_inflate(planning_grid_t *grid, int radius_cells);

planner_cell_t planning_grid_get(const planning_grid_t *grid, int x, int y);
void planning_grid_set(planning_grid_t *grid, int x, int y, planner_cell_t value);
bool planning_grid_is_inside(const planning_grid_t *grid, int x, int y);

#endif /* PLANNING_GRID_H */