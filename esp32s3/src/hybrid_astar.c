/**
 * hybrid_astar.c
 * Module: Hybrid A* kinematically-feasible path planner.
 * Board: ESP32-S3
 * Implementation phase: stub (search algorithm not yet implemented)
 */

#include "hybrid_astar.h"
#include "planning_grid.h"

#include <math.h>
#include <stddef.h>

#define PLANNER_CELL_SIZE_MM 100.0f
#define PLANNER_INFLATION_RADIUS_CELLS 2
#define PATH_MAX_WAYPOINTS 64
#define DEFAULT_TARGET_SPEED_MM_S 200.0f

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

    return (x >= 0.0f && x < map->width_mm &&
            y >= 0.0f && y < map->height_mm);
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

    int grid_w = (int)(map->width_mm / PLANNER_CELL_SIZE_MM);
    int grid_h = (int)(map->height_mm / PLANNER_CELL_SIZE_MM);

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

    /* TODO:
     * Replace this fallback with actual Hybrid A* search.
     * For now, return a minimal path with start and goal.
     */
    path.length = 2;

    path.waypoints[0].x = start->x;
    path.waypoints[0].y = start->y;
    path.waypoints[0].theta = start->theta;
    path.waypoints[0].v_target = DEFAULT_TARGET_SPEED_MM_S;

    float dx = goal->cx - start->x;
    float dy = goal->cy - start->y;
    float goal_theta = atan2f(dy, dx);

    path.waypoints[1].x = goal->cx;
    path.waypoints[1].y = goal->cy;
    path.waypoints[1].theta = goal_theta;
    path.waypoints[1].v_target = DEFAULT_TARGET_SPEED_MM_S;

    planning_grid_free(&grid);
    return path;
}