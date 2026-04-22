/**
 * hybrid_astar.c
 * Module: Direct-on-quadtree global path planner.
 * Board: ESP32-S3
 *
 * Current phase:
 * - no temporary planning grid
 * - A* runs directly on a sparse graph built from free quadtree leaves
 *
 * Note:
 * This keeps the public hybrid_astar_plan() API unchanged for compatibility.
 * Internally, this is graph-based A* on quadtree leaves, not full
 * car-kinematic Hybrid A* yet.
 */

#include "hybrid_astar.h"

#include <math.h>
#include <stddef.h>
#include <float.h>
#include <stdbool.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define DEFAULT_TARGET_SPEED_MM_S 200.0f
#define ASTAR_INF 1.0e30f
#define MAX_QT_FREE_LEAVES 4096
#define MAX_QT_NEIGHBORS 16
#define MAX_PATH_WAYPOINTS 64

typedef struct {
    float x_min;
    float x_max;
    float y_min;
    float y_max;
    float cx;
    float cy;
    int8_t value;
    uint16_t neighbors[MAX_QT_NEIGHBORS];
    uint8_t neighbor_count;
} qt_graph_node_t;

typedef struct {
    bool open;
    bool closed;
    float g;
    float f;
    int parent;
} astar_state_t;

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

static bool node_has_children(const QTNode *n)
{
    return (n->children[0] != QT_NULL ||
            n->children[1] != QT_NULL ||
            n->children[2] != QT_NULL ||
            n->children[3] != QT_NULL);
}

static void child_bounds(float xmn, float xmx,
                         float ymn, float ymx,
                         int q,
                         float *cxmn, float *cxmx,
                         float *cymn, float *cymx)
{
    float cx = 0.5f * (xmn + xmx);
    float cy = 0.5f * (ymn + ymx);

    switch (q) {
        case 0: /* NW */
            *cxmn = xmn; *cxmx = cx;
            *cymn = cy;  *cymx = ymx;
            break;
        case 1: /* NE */
            *cxmn = cx;  *cxmx = xmx;
            *cymn = cy;  *cymx = ymx;
            break;
        case 2: /* SW */
            *cxmn = xmn; *cxmx = cx;
            *cymn = ymn; *cymx = cy;
            break;
        default: /* SE */
            *cxmn = cx;  *cxmx = xmx;
            *cymn = ymn; *cymx = cy;
            break;
    }
}

static void collect_free_leaves_recursive(const quadtree_map_t *map,
                                          uint16_t idx,
                                          float xmn, float xmx,
                                          float ymn, float ymx,
                                          qt_graph_node_t *nodes,
                                          int *count)
{
    if (map == NULL || map->pool == NULL || idx == QT_NULL || count == NULL) {
        return;
    }

    const QTNode *n = &map->pool[idx];

    /* In the current implementation, meaningful occupancy is stored at leaves.
       Traversable leaves are the ones with negative log-odds (free). */
    if (!node_has_children(n)) {
        if (n->value < 0 && *count < MAX_QT_FREE_LEAVES) {
            qt_graph_node_t *out = &nodes[*count];
            out->x_min = xmn;
            out->x_max = xmx;
            out->y_min = ymn;
            out->y_max = ymx;
            out->cx = 0.5f * (xmn + xmx);
            out->cy = 0.5f * (ymn + ymx);
            out->value = n->value;
            out->neighbor_count = 0;
            (*count)++;
        }
        return;
    }

    for (int q = 0; q < 4; ++q) {
        if (n->children[q] == QT_NULL) {
            continue;
        }

        float cxmn, cxmx, cymn, cymx;
        child_bounds(xmn, xmx, ymn, ymx, q, &cxmn, &cxmx, &cymn, &cymx);

        collect_free_leaves_recursive(map,
                                      n->children[q],
                                      cxmn, cxmx, cymn, cymx,
                                      nodes, count);
    }
}

static int collect_free_leaves(const quadtree_map_t *map, qt_graph_node_t *nodes)
{
    int count = 0;

    if (map == NULL || map->pool == NULL || map->count <= 1) {
        return 0;
    }

    collect_free_leaves_recursive(map,
                                  1,
                                  map->x_min, map->x_max,
                                  map->y_min, map->y_max,
                                  nodes, &count);
    return count;
}

static bool intervals_overlap(float a0, float a1, float b0, float b1)
{
    return (fminf(a1, b1) - fmaxf(a0, b0)) > 0.0f;
}

static bool leaves_are_adjacent(const qt_graph_node_t *a, const qt_graph_node_t *b)
{
    const float eps = 1e-3f;

    bool touch_vertical =
        (fabsf(a->x_max - b->x_min) < eps || fabsf(b->x_max - a->x_min) < eps) &&
        intervals_overlap(a->y_min, a->y_max, b->y_min, b->y_max);

    bool touch_horizontal =
        (fabsf(a->y_max - b->y_min) < eps || fabsf(b->y_max - a->y_min) < eps) &&
        intervals_overlap(a->x_min, a->x_max, b->x_min, b->x_max);

    return touch_vertical || touch_horizontal;
}

static void add_neighbor(qt_graph_node_t *nodes, int from, int to)
{
    if (nodes[from].neighbor_count >= MAX_QT_NEIGHBORS) {
        return;
    }
    nodes[from].neighbors[nodes[from].neighbor_count++] = (uint16_t)to;
}

static void build_adjacency(qt_graph_node_t *nodes, int count)
{
    for (int i = 0; i < count; ++i) {
        nodes[i].neighbor_count = 0;
    }

    for (int i = 0; i < count; ++i) {
        for (int j = i + 1; j < count; ++j) {
            if (leaves_are_adjacent(&nodes[i], &nodes[j])) {
                add_neighbor(nodes, i, j);
                add_neighbor(nodes, j, i);
            }
        }
    }
}

static float node_distance(const qt_graph_node_t *a, const qt_graph_node_t *b)
{
    float dx = a->cx - b->cx;
    float dy = a->cy - b->cy;
    return sqrtf(dx * dx + dy * dy);
}

static int find_containing_free_leaf(const qt_graph_node_t *nodes, int count, float x, float y)
{
    for (int i = 0; i < count; ++i) {
        if (x >= nodes[i].x_min && x < nodes[i].x_max &&
            y >= nodes[i].y_min && y < nodes[i].y_max) {
            return i;
        }
    }
    return -1;
}

static int find_nearest_free_leaf(const qt_graph_node_t *nodes, int count, float x, float y)
{
    int best = -1;
    float best_d2 = FLT_MAX;

    for (int i = 0; i < count; ++i) {
        float dx = nodes[i].cx - x;
        float dy = nodes[i].cy - y;
        float d2 = dx * dx + dy * dy;

        if (d2 < best_d2) {
            best_d2 = d2;
            best = i;
        }
    }

    return best;
}

static float min_leaf_size_mm(const quadtree_map_t *map)
{
    float width = map->x_max - map->x_min;
    float height = map->y_max - map->y_min;
    float side = fminf(width, height);
    float divisions = (float)(1 << (QT_MAX_DEPTH - 1));
    return side / divisions;
}

static bool line_is_collision_free_quadtree(const quadtree_map_t *map,
                                            float x0, float y0,
                                            float x1, float y1)
{
    float dx = x1 - x0;
    float dy = y1 - y0;
    float dist = sqrtf(dx * dx + dy * dy);

    float step = 0.5f * min_leaf_size_mm(map);
    if (step <= 1e-3f) {
        step = 50.0f;
    }

    int samples = (int)(dist / step);
    if (samples < 1) {
        samples = 1;
    }

    for (int i = 0; i <= samples; ++i) {
        float t = (float)i / (float)samples;
        float x = x0 + t * dx;
        float y = y0 + t * dy;

        int8_t v = qt_query_const(map, x, y);

        /* free < 0, unknown == 0, occupied > 0 */
        if (v >= 0) {
            return false;
        }
    }

    return true;
}

static void smooth_path_quadtree(const quadtree_map_t *map, path_t *path)
{
    if (map == NULL || path == NULL || path->length < 3) {
        return;
    }

    waypoint_t smoothed[MAX_PATH_WAYPOINTS];
    uint8_t new_len = 0;
    int i = 0;

    while (i < path->length && new_len < MAX_PATH_WAYPOINTS) {
        smoothed[new_len++] = path->waypoints[i];

        int farthest = i + 1;
        for (int j = path->length - 1; j > i + 1; --j) {
            if (line_is_collision_free_quadtree(map,
                                                path->waypoints[i].x, path->waypoints[i].y,
                                                path->waypoints[j].x, path->waypoints[j].y)) {
                farthest = j;
                break;
            }
        }

        i = farthest;
    }

    if (new_len < MAX_PATH_WAYPOINTS) {
        waypoint_t last = path->waypoints[path->length - 1];
        if (new_len == 0 ||
            smoothed[new_len - 1].x != last.x ||
            smoothed[new_len - 1].y != last.y) {
            smoothed[new_len++] = last;
        }
    }

    path->length = new_len;
    for (int k = 0; k < new_len; ++k) {
        path->waypoints[k] = smoothed[k];
    }

    for (int k = 0; k + 1 < path->length; ++k) {
        float dx = path->waypoints[k + 1].x - path->waypoints[k].x;
        float dy = path->waypoints[k + 1].y - path->waypoints[k].y;
        path->waypoints[k].theta = atan2f(dy, dx);
    }

    if (path->length >= 2) {
        path->waypoints[path->length - 1].theta = path->waypoints[path->length - 2].theta;
    }
}

static bool reconstruct_graph_path(const qt_graph_node_t *nodes,
                                   const astar_state_t *states,
                                   int goal_idx,
                                   path_t *out_path)
{
    int rev[MAX_PATH_WAYPOINTS];
    int count = 0;
    int cur = goal_idx;

    while (cur >= 0 && count < MAX_PATH_WAYPOINTS) {
        rev[count++] = cur;

        if (states[cur].parent == cur) {
            break;
        }

        cur = states[cur].parent;
    }

    if (count <= 0) {
        return false;
    }

    out_path->length = (uint8_t)count;

    for (int i = 0; i < count; ++i) {
        int src = rev[count - 1 - i];

        out_path->waypoints[i].x = nodes[src].cx;
        out_path->waypoints[i].y = nodes[src].cy;
        out_path->waypoints[i].v_target = DEFAULT_TARGET_SPEED_MM_S;

        if (i + 1 < count) {
            int next = rev[count - 2 - i];
            float dx = nodes[next].cx - nodes[src].cx;
            float dy = nodes[next].cy - nodes[src].cy;
            out_path->waypoints[i].theta = atan2f(dy, dx);
        } else if (i > 0) {
            out_path->waypoints[i].theta = out_path->waypoints[i - 1].theta;
        } else {
            out_path->waypoints[i].theta = 0.0f;
        }
    }

    return true;
}

static bool run_quadtree_graph_astar(const qt_graph_node_t *nodes,
                                     int node_count,
                                     int start_idx,
                                     int goal_idx,
                                     path_t *out_path)
{
    static astar_state_t states[MAX_QT_FREE_LEAVES];

    if (node_count <= 0 || node_count > MAX_QT_FREE_LEAVES) {
        return false;
    }

    for (int i = 0; i < node_count; ++i) {
        states[i].open = false;
        states[i].closed = false;
        states[i].g = ASTAR_INF;
        states[i].f = ASTAR_INF;
        states[i].parent = -1;
    }

    states[start_idx].g = 0.0f;
    states[start_idx].f = node_distance(&nodes[start_idx], &nodes[goal_idx]);
    states[start_idx].open = true;
    states[start_idx].parent = start_idx;

    while (1) {
        int best = -1;
        float best_f = ASTAR_INF;

        for (int i = 0; i < node_count; ++i) {
            if (states[i].open && !states[i].closed && states[i].f < best_f) {
                best_f = states[i].f;
                best = i;
            }
        }

        if (best < 0) {
            return false;
        }

        if (best == goal_idx) {
            return reconstruct_graph_path(nodes, states, goal_idx, out_path);
        }

        states[best].open = false;
        states[best].closed = true;

        for (int k = 0; k < nodes[best].neighbor_count; ++k) {
            int nb = (int)nodes[best].neighbors[k];

            if (nb < 0 || nb >= node_count) {
                continue;
            }
            if (states[nb].closed) {
                continue;
            }

            float tentative_g = states[best].g + node_distance(&nodes[best], &nodes[nb]);

            if (!states[nb].open || tentative_g < states[nb].g) {
                states[nb].open = true;
                states[nb].g = tentative_g;
                states[nb].f = tentative_g + node_distance(&nodes[nb], &nodes[goal_idx]);
                states[nb].parent = best;
            }
        }
    }
}

path_t hybrid_astar_plan(const quadtree_map_t *map,
                         const pose_t *start,
                         const frontier_t *goal)
{
    path_t path = hybrid_astar_empty_path();
    qt_graph_node_t nodes[MAX_QT_FREE_LEAVES];

    if (map == NULL || start == NULL || goal == NULL) {
        return path;
    }

    if (!is_pose_inside_map(map, start->x, start->y) ||
        !is_pose_inside_map(map, goal->cx, goal->cy)) {
        return path;
    }

    int node_count = collect_free_leaves(map, nodes);
    if (node_count <= 0) {
        return path;
    }

    build_adjacency(nodes, node_count);

    int start_idx = find_containing_free_leaf(nodes, node_count, start->x, start->y);
    if (start_idx < 0) {
        start_idx = find_nearest_free_leaf(nodes, node_count, start->x, start->y);
    }

    int goal_idx = find_containing_free_leaf(nodes, node_count, goal->cx, goal->cy);
    if (goal_idx < 0) {
        goal_idx = find_nearest_free_leaf(nodes, node_count, goal->cx, goal->cy);
    }

    if (start_idx < 0 || goal_idx < 0) {
        return path;
    }

    if (!run_quadtree_graph_astar(nodes, node_count, start_idx, goal_idx, &path)) {
        return hybrid_astar_empty_path();
    }

    smooth_path_quadtree(map, &path);
    return path;
}