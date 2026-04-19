/**
 * hybrid_astar.c
 * Module: Hybrid A* kinematically-feasible path planner.
 * Board: ESP32-S3
 * Current phase: first real search implementation using A* on planning grid.
 * This is a stepping stone toward full Hybrid A*.
 */

#include "hybrid_astar.h"
#include "planning_grid.h"

#include <math.h>
#include <stddef.h>
#include <float.h>

#define PLANNER_CELL_SIZE_MM 100.0f
#define PLANNER_INFLATION_RADIUS_CELLS 2
#define DEFAULT_TARGET_SPEED_MM_S 200.0f

#define ASTAR_MAX_W 128
#define ASTAR_MAX_H 128
#define ASTAR_MAX_CELLS (ASTAR_MAX_W * ASTAR_MAX_H)
#define ASTAR_INF 1.0e30f

typedef struct {
    bool open;
    bool closed;
    float g;
    float f;
    int parent_x;
    int parent_y;
} astar_cell_t;

static path_t hybrid_astar_empty_path(void)
{
    path_t path;
    path.length = 0;
    return path;
}

bool hybrid_astar_is_valid(const path_t *path)
{
    return (path != NULL && path->length > 0);
}

static bool is_pose_inside_map(const quadtree_map_t *map, float x, float y)
{
    if (map == NULL) {
        return false;
    }

    return (x >= map->x_min && x < map->x_max &&
            y >= map->y_min && y < map->y_max);
}

static int world_to_grid_x(const quadtree_map_t *map, const planning_grid_t *grid, float x_mm)
{
    (void)map;
    return (int)((x_mm - map->x_min) / grid->cell_size_mm);
}

static int world_to_grid_y(const quadtree_map_t *map, const planning_grid_t *grid, float y_mm)
{
    (void)map;
    return (int)((y_mm - map->y_min) / grid->cell_size_mm);
}

static float grid_to_world_x(const quadtree_map_t *map, const planning_grid_t *grid, int gx)
{
    return map->x_min + ((float)gx + 0.5f) * grid->cell_size_mm;
}

static float grid_to_world_y(const quadtree_map_t *map, const planning_grid_t *grid, int gy)
{
    return map->y_min + ((float)gy + 0.5f) * grid->cell_size_mm;
}

static bool is_blocked(const planning_grid_t *grid, int gx, int gy)
{
    planner_cell_t cell = planning_grid_get(grid, gx, gy);

    return (cell == CELL_OCCUPIED ||
            cell == CELL_UNKNOWN ||
            cell == CELL_INFLATED);
}

static float heuristic_cost(int x0, int y0, int x1, int y1)
{
    float dx = (float)(x1 - x0);
    float dy = (float)(y1 - y0);
    return sqrtf(dx * dx + dy * dy);
}

static float step_cost(int dx, int dy)
{
    return (dx != 0 && dy != 0) ? 1.41421356f : 1.0f;
}

static int cell_index(const planning_grid_t *grid, int x, int y)
{
    return y * grid->width + x;
}

static bool reconstruct_path(const quadtree_map_t *map,
                             const planning_grid_t *grid,
                             astar_cell_t *cells,
                             int goal_x,
                             int goal_y,
                             path_t *out_path)
{
    int rev_x[64];
    int rev_y[64];
    int count = 0;

    int cx = goal_x;
    int cy = goal_y;

    while (count < 64) {
        rev_x[count] = cx;
        rev_y[count] = cy;
        count++;

        astar_cell_t *c = &cells[cell_index(grid, cx, cy)];
        if (c->parent_x == cx && c->parent_y == cy) {
            break;
        }

        if (c->parent_x < 0 || c->parent_y < 0) {
            break;
        }

        cx = c->parent_x;
        cy = c->parent_y;
    }

    if (count <= 0) {
        return false;
    }

    out_path->length = (uint8_t)count;

    for (int i = 0; i < count; ++i) {
        int src = count - 1 - i;
        float wx = grid_to_world_x(map, grid, rev_x[src]);
        float wy = grid_to_world_y(map, grid, rev_y[src]);

        out_path->waypoints[i].x = wx;
        out_path->waypoints[i].y = wy;
        out_path->waypoints[i].v_target = DEFAULT_TARGET_SPEED_MM_S;

        if (i + 1 < count) {
            float nx = grid_to_world_x(map, grid, rev_x[src - 1]);
            float ny = grid_to_world_y(map, grid, rev_y[src - 1]);
            out_path->waypoints[i].theta = atan2f(ny - wy, nx - wx);
        } else if (i > 0) {
            out_path->waypoints[i].theta = out_path->waypoints[i - 1].theta;
        } else {
            out_path->waypoints[i].theta = 0.0f;
        }
    }

    return true;
}

static bool run_astar(const quadtree_map_t *map,
                      const planning_grid_t *grid,
                      int start_x,
                      int start_y,
                      int goal_x,
                      int goal_y,
                      path_t *out_path)
{
    static astar_cell_t cells[ASTAR_MAX_CELLS];

    const int dirs[8][2] = {
        { 1,  0}, {-1,  0}, {0,  1}, {0, -1},
        { 1,  1}, { 1, -1}, {-1, 1}, {-1,-1}
    };

    int total = grid->width * grid->height;
    if (grid->width > ASTAR_MAX_W || grid->height > ASTAR_MAX_H || total > ASTAR_MAX_CELLS) {
        return false;
    }

    for (int i = 0; i < total; ++i) {
        cells[i].open = false;
        cells[i].closed = false;
        cells[i].g = ASTAR_INF;
        cells[i].f = ASTAR_INF;
        cells[i].parent_x = -1;
        cells[i].parent_y = -1;
    }

    int start_idx = cell_index(grid, start_x, start_y);
    cells[start_idx].g = 0.0f;
    cells[start_idx].f = heuristic_cost(start_x, start_y, goal_x, goal_y);
    cells[start_idx].open = true;
    cells[start_idx].parent_x = start_x;
    cells[start_idx].parent_y = start_y;

    while (1) {
        int best_x = -1;
        int best_y = -1;
        float best_f = ASTAR_INF;

        for (int y = 0; y < grid->height; ++y) {
            for (int x = 0; x < grid->width; ++x) {
                astar_cell_t *c = &cells[cell_index(grid, x, y)];
                if (c->open && !c->closed && c->f < best_f) {
                    best_f = c->f;
                    best_x = x;
                    best_y = y;
                }
            }
        }

        if (best_x < 0 || best_y < 0) {
            return false;
        }

        if (best_x == goal_x && best_y == goal_y) {
            return reconstruct_path(map, grid, cells, goal_x, goal_y, out_path);
        }

        astar_cell_t *current = &cells[cell_index(grid, best_x, best_y)];
        current->closed = true;
        current->open = false;

        for (int i = 0; i < 8; ++i) {
            int nx = best_x + dirs[i][0];
            int ny = best_y + dirs[i][1];

            if (!planning_grid_is_inside(grid, nx, ny)) {
                continue;
            }

            if (is_blocked(grid, nx, ny)) {
                continue;
            }

            astar_cell_t *neighbor = &cells[cell_index(grid, nx, ny)];
            if (neighbor->closed) {
                continue;
            }

            float tentative_g = current->g + step_cost(dirs[i][0], dirs[i][1]);

            if (!neighbor->open || tentative_g < neighbor->g) {
                neighbor->open = true;
                neighbor->g = tentative_g;
                neighbor->f = tentative_g + heuristic_cost(nx, ny, goal_x, goal_y);
                neighbor->parent_x = best_x;
                neighbor->parent_y = best_y;
            }
        }
    }
}

path_t hybrid_astar_plan(const quadtree_map_t *map,
                         const pose_t *start,
                         const frontier_t *goal)
{
    path_t path = hybrid_astar_empty_path();
    planning_grid_t grid;

    if (map == NULL || start == NULL || goal == NULL) {
        return path;
    }

    if (!is_pose_inside_map(map, start->x, start->y) ||
        !is_pose_inside_map(map, goal->cx, goal->cy)) {
        return path;
    }

    float map_width_mm = map->x_max - map->x_min;
    float map_height_mm = map->y_max - map->y_min;

    int grid_w = (int)(map_width_mm / PLANNER_CELL_SIZE_MM);
    int grid_h = (int)(map_height_mm / PLANNER_CELL_SIZE_MM);

    if (grid_w <= 0 || grid_h <= 0) {
        return path;
    }

    if (!planning_grid_init(&grid, grid_w, grid_h, PLANNER_CELL_SIZE_MM)) {
        return path;
    }

    if (!planning_grid_build_from_quadtree(map, &grid)) {
        planning_grid_free(&grid);
        return path;
    }

    planning_grid_inflate(&grid, PLANNER_INFLATION_RADIUS_CELLS);

    int start_x = world_to_grid_x(map, &grid, start->x);
    int start_y = world_to_grid_y(map, &grid, start->y);
    int goal_x = world_to_grid_x(map, &grid, goal->cx);
    int goal_y = world_to_grid_y(map, &grid, goal->cy);

    if (!planning_grid_is_inside(&grid, start_x, start_y) ||
        !planning_grid_is_inside(&grid, goal_x, goal_y) ||
        is_blocked(&grid, start_x, start_y) ||
        is_blocked(&grid, goal_x, goal_y)) {
        planning_grid_free(&grid);
        return path;
    }

    if (!run_astar(map, &grid, start_x, start_y, goal_x, goal_y, &path)) {
        planning_grid_free(&grid);
        return hybrid_astar_empty_path();
    }

    planning_grid_free(&grid);
    return path;
}