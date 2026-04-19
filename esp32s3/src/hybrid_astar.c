/**
 * hybrid_astar.c
 * Module: Hybrid A* kinematically-feasible path planner.
 * Board: ESP32-S3
 * Current phase: first heading-aware Hybrid A* implementation on planning grid.
 */

#include "hybrid_astar.h"
#include "planning_grid.h"

#include <math.h>
#include <stddef.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define PLANNER_CELL_SIZE_MM 100.0f
#define PLANNER_INFLATION_RADIUS_CELLS 2
#define DEFAULT_TARGET_SPEED_MM_S 200.0f

#define ASTAR_MAX_W 128
#define ASTAR_MAX_H 128
#define ANGLE_BINS 16
#define HYBRID_MAX_STATES (ASTAR_MAX_W * ASTAR_MAX_H * ANGLE_BINS)
#define ASTAR_INF 1.0e30f

#define PRIMITIVE_STEP_CELLS 1.5f
#define PRIMITIVE_TURN_ANGLE_RAD 0.39269908f   /* ~22.5 deg */
#define COLLISION_SAMPLES 5
#define MAX_PATH_WAYPOINTS 64

typedef struct {
    bool open;
    bool closed;
    float g;
    float f;
    int parent_x;
    int parent_y;
    int parent_theta;
} hybrid_cell_t;

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

static float wrap_angle(float a)
{
    while (a > (float)M_PI) {
        a -= 2.0f * (float)M_PI;
    }
    while (a < -(float)M_PI) {
        a += 2.0f * (float)M_PI;
    }
    return a;
}

static int theta_to_bin(float theta)
{
    float wrapped = wrap_angle(theta);
    float normalized = (wrapped + (float)M_PI) / (2.0f * (float)M_PI);
    int bin = (int)(normalized * (float)ANGLE_BINS);

    if (bin < 0) {
        bin = 0;
    }
    if (bin >= ANGLE_BINS) {
        bin = ANGLE_BINS - 1;
    }

    return bin;
}

static float bin_to_theta(int bin)
{
    float frac = ((float)bin + 0.5f) / (float)ANGLE_BINS;
    return frac * 2.0f * (float)M_PI - (float)M_PI;
}

static int world_to_grid_x(const quadtree_map_t *map, const planning_grid_t *grid, float x_mm)
{
    return (int)((x_mm - map->x_min) / grid->cell_size_mm);
}

static int world_to_grid_y(const quadtree_map_t *map, const planning_grid_t *grid, float y_mm)
{
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

static float heading_difference_cost(int theta_bin, float goal_theta)
{
    float theta = bin_to_theta(theta_bin);
    float d = wrap_angle(goal_theta - theta);
    return fabsf(d);
}

static int hybrid_index(const planning_grid_t *grid, int x, int y, int theta_bin)
{
    return (theta_bin * grid->height + y) * grid->width + x;
}

static bool apply_motion_primitive(const quadtree_map_t *map,
                                   const planning_grid_t *grid,
                                   int x,
                                   int y,
                                   int theta_bin,
                                   float steering_delta,
                                   int *out_x,
                                   int *out_y,
                                   int *out_theta_bin)
{
    float theta = bin_to_theta(theta_bin);
    float new_theta = wrap_angle(theta + steering_delta);

    float wx = grid_to_world_x(map, grid, x);
    float wy = grid_to_world_y(map, grid, y);
    float step_mm = PRIMITIVE_STEP_CELLS * grid->cell_size_mm;

    float new_wx = wx + step_mm * cosf(new_theta);
    float new_wy = wy + step_mm * sinf(new_theta);

    int gx = world_to_grid_x(map, grid, new_wx);
    int gy = world_to_grid_y(map, grid, new_wy);
    int gt = theta_to_bin(new_theta);

    if (!planning_grid_is_inside(grid, gx, gy)) {
        return false;
    }

    *out_x = gx;
    *out_y = gy;
    *out_theta_bin = gt;
    return true;
}

static bool primitive_is_collision_free(const quadtree_map_t *map,
                                        const planning_grid_t *grid,
                                        int x0,
                                        int y0,
                                        int theta0_bin,
                                        int x1,
                                        int y1,
                                        int theta1_bin)
{
    (void)theta0_bin;
    (void)theta1_bin;

    float wx0 = grid_to_world_x(map, grid, x0);
    float wy0 = grid_to_world_y(map, grid, y0);
    float wx1 = grid_to_world_x(map, grid, x1);
    float wy1 = grid_to_world_y(map, grid, y1);

    for (int i = 0; i <= COLLISION_SAMPLES; ++i) {
        float t = (float)i / (float)COLLISION_SAMPLES;
        float wx = wx0 + t * (wx1 - wx0);
        float wy = wy0 + t * (wy1 - wy0);

        int gx = world_to_grid_x(map, grid, wx);
        int gy = world_to_grid_y(map, grid, wy);

        if (!planning_grid_is_inside(grid, gx, gy)) {
            return false;
        }
        if (is_blocked(grid, gx, gy)) {
            return false;
        }
    }

    return true;
}

static bool reconstruct_hybrid_path(const quadtree_map_t *map,
                                    const planning_grid_t *grid,
                                    hybrid_cell_t *states,
                                    int goal_x,
                                    int goal_y,
                                    int goal_theta,
                                    path_t *out_path)
{
    int rev_x[MAX_PATH_WAYPOINTS];
    int rev_y[MAX_PATH_WAYPOINTS];
    int rev_t[MAX_PATH_WAYPOINTS];
    int count = 0;

    int cx = goal_x;
    int cy = goal_y;
    int ct = goal_theta;

    while (count < MAX_PATH_WAYPOINTS) {
        hybrid_cell_t *c = &states[hybrid_index(grid, cx, cy, ct)];

        rev_x[count] = cx;
        rev_y[count] = cy;
        rev_t[count] = ct;
        count++;

        if (c->parent_x == cx && c->parent_y == cy && c->parent_theta == ct) {
            break;
        }

        if (c->parent_x < 0 || c->parent_y < 0 || c->parent_theta < 0) {
            break;
        }

        cx = c->parent_x;
        cy = c->parent_y;
        ct = c->parent_theta;
    }

    if (count <= 0) {
        return false;
    }

    out_path->length = (uint8_t)count;

    for (int i = 0; i < count; ++i) {
        int src = count - 1 - i;

        out_path->waypoints[i].x = grid_to_world_x(map, grid, rev_x[src]);
        out_path->waypoints[i].y = grid_to_world_y(map, grid, rev_y[src]);
        out_path->waypoints[i].theta = bin_to_theta(rev_t[src]);
        out_path->waypoints[i].v_target = DEFAULT_TARGET_SPEED_MM_S;
    }

    return true;
}

static bool run_hybrid_astar(const quadtree_map_t *map,
                             const planning_grid_t *grid,
                             int start_x,
                             int start_y,
                             int start_theta,
                             int goal_x,
                             int goal_y,
                             path_t *out_path)
{
    static hybrid_cell_t states[HYBRID_MAX_STATES];
    const float primitives[3] = {
        -PRIMITIVE_TURN_ANGLE_RAD,
         0.0f,
         PRIMITIVE_TURN_ANGLE_RAD
    };

    int total = grid->width * grid->height * ANGLE_BINS;
    if (grid->width > ASTAR_MAX_W ||
        grid->height > ASTAR_MAX_H ||
        total > HYBRID_MAX_STATES) {
        return false;
    }

    for (int i = 0; i < total; ++i) {
        states[i].open = false;
        states[i].closed = false;
        states[i].g = ASTAR_INF;
        states[i].f = ASTAR_INF;
        states[i].parent_x = -1;
        states[i].parent_y = -1;
        states[i].parent_theta = -1;
    }

    float goal_heading = atan2f((float)(goal_y - start_y), (float)(goal_x - start_x));

    int start_idx = hybrid_index(grid, start_x, start_y, start_theta);
    states[start_idx].g = 0.0f;
    states[start_idx].f = heuristic_cost(start_x, start_y, goal_x, goal_y);
    states[start_idx].open = true;
    states[start_idx].parent_x = start_x;
    states[start_idx].parent_y = start_y;
    states[start_idx].parent_theta = start_theta;

    while (1) {
        int best_x = -1;
        int best_y = -1;
        int best_t = -1;
        float best_f = ASTAR_INF;

        for (int t = 0; t < ANGLE_BINS; ++t) {
            for (int y = 0; y < grid->height; ++y) {
                for (int x = 0; x < grid->width; ++x) {
                    hybrid_cell_t *s = &states[hybrid_index(grid, x, y, t)];
                    if (s->open && !s->closed && s->f < best_f) {
                        best_f = s->f;
                        best_x = x;
                        best_y = y;
                        best_t = t;
                    }
                }
            }
        }

        if (best_x < 0 || best_y < 0 || best_t < 0) {
            return false;
        }

        if (best_x == goal_x && best_y == goal_y) {
            return reconstruct_hybrid_path(map, grid, states, best_x, best_y, best_t, out_path);
        }

        hybrid_cell_t *current = &states[hybrid_index(grid, best_x, best_y, best_t)];
        current->closed = true;
        current->open = false;

        for (int i = 0; i < 3; ++i) {
            int nx = 0;
            int ny = 0;
            int nt = 0;

            if (!apply_motion_primitive(map, grid, best_x, best_y, best_t,
                                        primitives[i], &nx, &ny, &nt)) {
                continue;
            }

            if (is_blocked(grid, nx, ny)) {
                continue;
            }

            if (!primitive_is_collision_free(map, grid, best_x, best_y, best_t, nx, ny, nt)) {
                continue;
            }

            hybrid_cell_t *neighbor = &states[hybrid_index(grid, nx, ny, nt)];
            if (neighbor->closed) {
                continue;
            }

            float steering_penalty = (i == 1) ? 0.0f : 0.2f;
            float tentative_g = current->g + 1.0f + steering_penalty;
            float h = heuristic_cost(nx, ny, goal_x, goal_y) +
                      0.2f * heading_difference_cost(nt, goal_heading);

            if (!neighbor->open || tentative_g < neighbor->g) {
                neighbor->open = true;
                neighbor->g = tentative_g;
                neighbor->f = tentative_g + h;
                neighbor->parent_x = best_x;
                neighbor->parent_y = best_y;
                neighbor->parent_theta = best_t;
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
    int start_theta = theta_to_bin(start->theta);

    int goal_x = world_to_grid_x(map, &grid, goal->cx);
    int goal_y = world_to_grid_y(map, &grid, goal->cy);

    if (!planning_grid_is_inside(&grid, start_x, start_y) ||
        !planning_grid_is_inside(&grid, goal_x, goal_y) ||
        is_blocked(&grid, start_x, start_y) ||
        is_blocked(&grid, goal_x, goal_y)) {
        planning_grid_free(&grid);
        return path;
    }

    if (!run_hybrid_astar(map, &grid, start_x, start_y, start_theta, goal_x, goal_y, &path)) {
        planning_grid_free(&grid);
        return hybrid_astar_empty_path();
    }

    planning_grid_free(&grid);
    return path;
}