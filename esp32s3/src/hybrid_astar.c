/* ============================================================================
 * hybrid_astar.c
 * ============================================================================
 * Module: Direct-on-quadtree Hybrid A* global path planner.
 * Board:  ESP32-S3
 *
 * Open list: indexed min-heap (g_heap) + hash map (g_hash).
 *   g_hash  maps discretized state (ix, iy, itheta) → node index in g_nodes[].
 *   g_heap  is a min-heap of node INDICES ordered by f-score.
 *   heap_pos inside each node is its current slot in g_heap[] (or -1).
 *   All three are kept in sync on every heap operation.
 *
 * Complexity: O(n log n) vs the previous O(n²) linear-scan implementation.
 *
 * Memory vs previous version (net −41 KB BSS):
 *   MAX_HYBRID_NODES 2048 → 1024  saves 36 KB (g_nodes) + 4 KB (reverse_buf)
 *   MAX_QT_FREE_LEAVES 512 → 256  saves  7 KB
 *   g_heap[1024] + g_hash[2048]  costs  2 KB + 4 KB
 */

#include "hybrid_astar.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <stddef.h>
#include <float.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "esp_log.h"

static const char *TAG = "hybrid_astar";

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef QT_NULL
#error "QT_NULL must be defined by quadtree_map.h"
#endif

/* -------------------------------------------------------------------------- */
/* Tunables                                                                    */
/* -------------------------------------------------------------------------- */

#define DEFAULT_TARGET_SPEED_MM_S       200.0f

#define MAX_HYBRID_NODES                1024
#define MAX_PATH_WAYPOINTS              HYBRID_ASTAR_MAX_WAYPOINTS
#define MAX_QT_FREE_LEAVES              256

#ifndef PLANNER_WHEELBASE_MM
#define PLANNER_WHEELBASE_MM            260.0f
#endif

#ifndef PLANNER_MAX_STEER_RAD
#define PLANNER_MAX_STEER_RAD           1.134f  /* 65° — physical steering limit */
#endif

#ifndef PLANNER_ROBOT_RADIUS_MM
/* Half-width of car (295/2 = 147.5 mm) rounded up — the relevant lateral
 * clearance for forward-moving paths.  The full half-diagonal (246 mm) is
 * only needed for pure rotation and would make A* reject the start position
 * when the car is within 246 mm of any mapped wall. */
#define PLANNER_ROBOT_RADIUS_MM         150.0f
#endif

#ifndef HYBRID_XY_RESOLUTION_MM
#define HYBRID_XY_RESOLUTION_MM         100.0f
#endif

#ifndef HYBRID_THETA_BINS
#define HYBRID_THETA_BINS               24
#endif

#ifndef HYBRID_PRIMITIVE_STEP_MM
#define HYBRID_PRIMITIVE_STEP_MM        120.0f
#endif

#ifndef HYBRID_COLLISION_STEP_MM
#define HYBRID_COLLISION_STEP_MM        30.0f
#endif

#ifndef HYBRID_GOAL_TOLERANCE_MM
#define HYBRID_GOAL_TOLERANCE_MM        140.0f
#endif

#define REVERSE_COST_MULTIPLIER         1.8f
#define STEER_COST_MULTIPLIER           1.05f
#define STEER_CHANGE_COST               15.0f
#define MAX_FRONTIER_APPEND_MM          250.0f

/* -------------------------------------------------------------------------- */
/* Hash table constants                                                        */
/* -------------------------------------------------------------------------- */

/* Power-of-two, >= 2 × MAX_HYBRID_NODES for <= 50% load. */
#define HASH_SIZE   4096u
#define HASH_MASK   (HASH_SIZE - 1u)
#define HASH_EMPTY  0xFFFFu   /* sentinel: slot is unoccupied */

/* -------------------------------------------------------------------------- */
/* Data structures                                                             */
/* -------------------------------------------------------------------------- */

typedef struct {
    float   x_min, x_max, y_min, y_max;
    float   cx, cy;
    int8_t  value;
} qt_free_leaf_t;

typedef struct {
    float   x, y, theta;
    float   g, f;
    int     parent;
    int8_t  steer_id;
    int8_t  direction;
    int16_t ix, iy, itheta;
    bool    closed;      /* true after the node has been expanded */
    int16_t heap_pos;    /* slot in g_heap[], or -1 when not in open set */
} hybrid_node_t;
_Static_assert(sizeof(hybrid_node_t) == 36u,
               "hybrid_node_t alignment changed — recheck 41 KB BSS savings claim");

/* State semantics:
 *   not in g_hash          → never seen
 *   in g_hash, closed=false, heap_pos >= 0  → open
 *   in g_hash, closed=true,  heap_pos == -1 → closed            */

typedef struct { int count; bool truncated; } collect_result_t;

/* -------------------------------------------------------------------------- */
/* Static storage                                                              */
/* -------------------------------------------------------------------------- */

static hybrid_node_t  g_nodes[MAX_HYBRID_NODES];
static qt_free_leaf_t g_free_leaves[MAX_QT_FREE_LEAVES];

/* Open-list structures — reset at the start of every search. */
static uint16_t g_heap[MAX_HYBRID_NODES]; /* min-heap of node indices     */
static int      g_heap_size;
static uint16_t g_hash[HASH_SIZE];        /* state key → node index       */

/* -------------------------------------------------------------------------- */
/* Basic helpers                                                               */
/* -------------------------------------------------------------------------- */

static path_t hybrid_astar_empty_path(void)
{
    path_t p; p.length = 0; return p;
}

bool hybrid_astar_is_valid(const path_t *path)
{
    return (path != NULL && path->length > 0);
}

static float sqr(float x) { return x * x; }

static float dist2_xy(float x0, float y0, float x1, float y1)
{
    return sqr(x1 - x0) + sqr(y1 - y0);
}

static float dist_xy(float x0, float y0, float x1, float y1)
{
    return sqrtf(dist2_xy(x0, y0, x1, y1));
}

static float wrap_pi(float a)
{
    a = fmodf(a + (float)M_PI, 2.0f * (float)M_PI);
    if (a < 0.0f) a += 2.0f * (float)M_PI;
    return a - (float)M_PI;
}

static bool map_is_valid(const quadtree_map_t *map)
{
    return (map && map->pool && map->count > 1 &&
            map->x_max > map->x_min && map->y_max > map->y_min);
}

static bool is_pose_inside_map(const quadtree_map_t *map, float x, float y)
{
    return map_is_valid(map) &&
           x >= map->x_min && x < map->x_max &&
           y >= map->y_min && y < map->y_max;
}

static bool node_has_children(const QTNode *n)
{
    return n && (n->children[0] != QT_NULL || n->children[1] != QT_NULL ||
                 n->children[2] != QT_NULL || n->children[3] != QT_NULL);
}

/* -------------------------------------------------------------------------- */
/* Hash map                                                                    */
/* -------------------------------------------------------------------------- */

/* Pack (ix, iy, itheta) into a 30-bit key: 10 bits each. */
static uint32_t pack_key(int16_t ix, int16_t iy, int16_t itheta)
{
    return ((uint32_t)(uint16_t)itheta << 20) |
           ((uint32_t)(uint16_t)iy     << 10) |
            (uint32_t)(uint16_t)ix;
}

/* Knuth multiplicative hash → 12-bit index for HASH_SIZE=4096. */
static uint32_t hash_slot_for(int16_t ix, int16_t iy, int16_t itheta)
{
    return (pack_key(ix, iy, itheta) * 2654435761u) >> 20u;
}

/* Returns node index in g_nodes[], or -1 if not found. O(1) amortized. */
static int hash_lookup(int16_t ix, int16_t iy, int16_t itheta)
{
    uint32_t s = hash_slot_for(ix, iy, itheta);
    for (uint32_t i = 0; i < HASH_SIZE; i++) {
        uint32_t slot = (s + i) & HASH_MASK;
        uint16_t ni   = g_hash[slot];
        if (ni == HASH_EMPTY) return -1;
        if (g_nodes[ni].ix == ix &&
            g_nodes[ni].iy == iy &&
            g_nodes[ni].itheta == itheta) return (int)ni;
    }
    return -1;
}

/* Node must be fully initialised (ix/iy/itheta written) before calling. */
static void hash_insert(uint16_t node_idx)
{
    uint32_t s = hash_slot_for(g_nodes[node_idx].ix,
                               g_nodes[node_idx].iy,
                               g_nodes[node_idx].itheta);
    for (uint32_t i = 0; i < HASH_SIZE; i++) {
        uint32_t slot = (s + i) & HASH_MASK;
        if (g_hash[slot] == HASH_EMPTY) {
            g_hash[slot] = node_idx;
            return;
        }
    }
    /* Unreachable when HASH_SIZE >= 2 × MAX_HYBRID_NODES. */
}

/* -------------------------------------------------------------------------- */
/* Indexed min-heap                                                            */
/* -------------------------------------------------------------------------- */

static void heap_swap(int a, int b)
{
    uint16_t tmp    = g_heap[a];
    g_heap[a]       = g_heap[b];
    g_heap[b]       = tmp;
    g_nodes[g_heap[a]].heap_pos = (int16_t)a;
    g_nodes[g_heap[b]].heap_pos = (int16_t)b;
}

static void heap_sift_up(int pos)
{
    while (pos > 0) {
        int parent = (pos - 1) / 2;
        if (g_nodes[g_heap[parent]].f <= g_nodes[g_heap[pos]].f) break;
        heap_swap(parent, pos);
        pos = parent;
    }
}

static void heap_sift_down(int pos)
{
    for (;;) {
        int left     = 2 * pos + 1;
        int right    = 2 * pos + 2;
        int smallest = pos;
        if (left  < g_heap_size &&
            g_nodes[g_heap[left ]].f < g_nodes[g_heap[smallest]].f) smallest = left;
        if (right < g_heap_size &&
            g_nodes[g_heap[right]].f < g_nodes[g_heap[smallest]].f) smallest = right;
        if (smallest == pos) break;
        heap_swap(pos, smallest);
        pos = smallest;
    }
}

/* Insert node_idx into the open set. */
static void heap_push(int node_idx)
{
    int pos               = g_heap_size++;
    g_heap[pos]           = (uint16_t)node_idx;
    g_nodes[node_idx].heap_pos = (int16_t)pos;
    heap_sift_up(pos);
}

/* Remove and return the index of the lowest-f node.
 * Caller must set closed=true on the returned node. */
static int heap_pop(void)
{
    if (g_heap_size == 0) return -1;
    int best = (int)g_heap[0];
    g_nodes[best].heap_pos = -1;
    g_heap_size--;
    if (g_heap_size > 0) {
        g_heap[0] = g_heap[g_heap_size];
        g_nodes[g_heap[0]].heap_pos = 0;
        heap_sift_down(0);
    }
    return best;
}

/* -------------------------------------------------------------------------- */
/* Quadtree leaf collection                                                    */
/* -------------------------------------------------------------------------- */

static void child_bounds(float xmn, float xmx, float ymn, float ymx, int q,
                         float *cxmn, float *cxmx, float *cymn, float *cymx)
{
    const float cx = 0.5f * (xmn + xmx);
    const float cy = 0.5f * (ymn + ymx);
    switch (q) {
        case 0: *cxmn=xmn; *cxmx=cx;  *cymn=cy;  *cymx=ymx; break;
        case 1: *cxmn=cx;  *cxmx=xmx; *cymn=cy;  *cymx=ymx; break;
        case 2: *cxmn=xmn; *cxmx=cx;  *cymn=ymn; *cymx=cy;  break;
        default:*cxmn=cx;  *cxmx=xmx; *cymn=ymn; *cymx=cy;  break;
    }
}

static void collect_free_leaves_recursive(const quadtree_map_t *map,
                                          uint16_t idx,
                                          float xmn, float xmx,
                                          float ymn, float ymx,
                                          qt_free_leaf_t *leaves,
                                          collect_result_t *result)
{
    if (!map || !map->pool || !leaves || !result) return;
    if (idx == QT_NULL || idx >= map->count) return;

    const QTNode *n = &map->pool[idx];
    if (!node_has_children(n)) {
        if (n->value < 0) {
            if (result->count < MAX_QT_FREE_LEAVES) {
                qt_free_leaf_t *out = &leaves[result->count++];
                out->x_min = xmn; out->x_max = xmx;
                out->y_min = ymn; out->y_max = ymx;
                out->cx    = 0.5f * (xmn + xmx);
                out->cy    = 0.5f * (ymn + ymx);
                out->value = n->value;
            } else {
                result->truncated = true;
            }
        }
        return;
    }
    for (int q = 0; q < 4; ++q) {
        if (n->children[q] == QT_NULL) continue;
        float cxmn, cxmx, cymn, cymx;
        child_bounds(xmn, xmx, ymn, ymx, q, &cxmn, &cxmx, &cymn, &cymx);
        collect_free_leaves_recursive(map, n->children[q],
                                      cxmn, cxmx, cymn, cymx, leaves, result);
    }
}

static collect_result_t collect_free_leaves(const quadtree_map_t *map,
                                            qt_free_leaf_t *leaves)
{
    collect_result_t r = {0, false};
    if (!map_is_valid(map) || !leaves) return r;
    collect_free_leaves_recursive(map, 1,
                                  map->x_min, map->x_max,
                                  map->y_min, map->y_max,
                                  leaves, &r);
    return r;
}

static int find_nearest_free_leaf(const qt_free_leaf_t *leaves, int count,
                                  float x, float y)
{
    int best = -1;
    float best_d2 = FLT_MAX;
    for (int i = 0; i < count; ++i) {
        const float d2 = dist2_xy(leaves[i].cx, leaves[i].cy, x, y);
        if (d2 < best_d2) { best_d2 = d2; best = i; }
    }
    return best;
}

/* -------------------------------------------------------------------------- */
/* Collision checking                                                          */
/* -------------------------------------------------------------------------- */

static bool point_robot_collision_free(const quadtree_map_t *map, float x, float y)
{
    if (!is_pose_inside_map(map, x, y)) return false;

    static const float dirs[9][2] = {
        { 0.0000f,  0.0000f},
        { 1.0000f,  0.0000f}, {-1.0000f,  0.0000f},
        { 0.0000f,  1.0000f}, { 0.0000f, -1.0000f},
        { 0.7071f,  0.7071f}, {-0.7071f,  0.7071f},
        { 0.7071f, -0.7071f}, {-0.7071f, -0.7071f}
    };
    for (int i = 0; i < 9; ++i) {
        const float sx = x + dirs[i][0] * PLANNER_ROBOT_RADIUS_MM;
        const float sy = y + dirs[i][1] * PLANNER_ROBOT_RADIUS_MM;
        if (!is_pose_inside_map(map, sx, sy)) return false;
        if (qt_query_const(map, sx, sy) > 0)   return false;
    }
    return true;
}

static bool line_is_collision_free_quadtree(const quadtree_map_t *map,
                                            float x0, float y0,
                                            float x1, float y1)
{
    const float dx = x1 - x0, dy = y1 - y0;
    const float d  = sqrtf(dx * dx + dy * dy);
    int samples    = (int)ceilf(d / HYBRID_COLLISION_STEP_MM);
    if (samples < 1) samples = 1;
    for (int i = 0; i <= samples; ++i) {
        const float t = (float)i / (float)samples;
        if (!point_robot_collision_free(map, x0 + t*dx, y0 + t*dy)) return false;
    }
    return true;
}

static bool simulate_primitive(const quadtree_map_t *map,
                               float x0, float y0, float theta0,
                               float steer, int direction,
                               float *x_out, float *y_out, float *theta_out)
{
    if (!map || !x_out || !y_out || !theta_out) return false;
    if (direction != 1 && direction != -1)      return false;

    const int   n  = (int)ceilf(HYBRID_PRIMITIVE_STEP_MM / HYBRID_COLLISION_STEP_MM);
    const int   ns = (n < 1) ? 1 : n;
    const float ds = ((float)direction * HYBRID_PRIMITIVE_STEP_MM) / (float)ns;

    float x = x0, y = y0, theta = theta0;
    if (!point_robot_collision_free(map, x, y)) return false;

    for (int i = 0; i < ns; ++i) {
        if (fabsf(steer) < 1.0e-4f) {
            x += ds * cosf(theta);
            y += ds * sinf(theta);
        } else {
            const float dtheta     = ds * tanf(steer) / PLANNER_WHEELBASE_MM;
            const float r          = PLANNER_WHEELBASE_MM / tanf(steer);
            const float next_theta = theta + dtheta;
            x     += r * (sinf(next_theta) - sinf(theta));
            y     += -r * (cosf(next_theta) - cosf(theta));
            theta  = next_theta;
        }
        theta = wrap_pi(theta);
        if (!point_robot_collision_free(map, x, y)) return false;
    }
    *x_out = x; *y_out = y; *theta_out = wrap_pi(theta);
    return true;
}

/* -------------------------------------------------------------------------- */
/* Discretization and heuristics                                               */
/* -------------------------------------------------------------------------- */

static int16_t xy_to_ix(const quadtree_map_t *map, float x)
{
    return (int16_t)floorf((x - map->x_min) / HYBRID_XY_RESOLUTION_MM);
}

static int16_t xy_to_iy(const quadtree_map_t *map, float y)
{
    return (int16_t)floorf((y - map->y_min) / HYBRID_XY_RESOLUTION_MM);
}

static int16_t theta_to_bin(float theta)
{
    float a = wrap_pi(theta);
    if (a < 0.0f) a += 2.0f * (float)M_PI;
    int bin = (int)floorf(a * (float)HYBRID_THETA_BINS / (2.0f * (float)M_PI));
    if (bin < 0)                 bin = 0;
    if (bin >= HYBRID_THETA_BINS) bin = HYBRID_THETA_BINS - 1;
    return (int16_t)bin;
}

static float heuristic_cost(float x, float y, float gx, float gy)
{
    return dist_xy(x, y, gx, gy);
}

static bool goal_reached(float x, float y, float gx, float gy)
{
    return dist2_xy(x, y, gx, gy) <= sqr(HYBRID_GOAL_TOLERANCE_MM);
}

static float motion_cost(int direction, int steer_id, int parent_steer_id)
{
    float cost = HYBRID_PRIMITIVE_STEP_MM;
    if (direction < 0)               cost *= REVERSE_COST_MULTIPLIER;
    if (steer_id != 0)               cost *= STEER_COST_MULTIPLIER;
    if (parent_steer_id != steer_id) cost += STEER_CHANGE_COST;
    return cost;
}

/* -------------------------------------------------------------------------- */
/* Path reconstruction                                                         */
/* -------------------------------------------------------------------------- */

static bool reconstruct_path(int goal_idx, path_t *out_path)
{
    static int reverse_buf[MAX_HYBRID_NODES];
    int raw_count = 0, cur = goal_idx;

    if (!out_path || goal_idx < 0 || goal_idx >= MAX_HYBRID_NODES) return false;

    while (cur >= 0) {
        if (raw_count >= MAX_HYBRID_NODES) {
            ESP_LOGW(TAG, "path reconstruction exceeded node buffer");
            return false;
        }
        reverse_buf[raw_count++] = cur;
        if (g_nodes[cur].parent == cur) break;
        cur = g_nodes[cur].parent;
    }

    if (raw_count <= 0) return false;

    int out_count = (raw_count < MAX_PATH_WAYPOINTS) ? raw_count : MAX_PATH_WAYPOINTS;
    out_path->length = (uint8_t)out_count;

    for (int i = 0; i < out_count; ++i) {
        int raw_index;
        if (out_count == 1) {
            raw_index = raw_count - 1;
        } else {
            const float alpha   = (float)i / (float)(out_count - 1);
            const int   rev_pos = (int)lroundf((1.0f - alpha) * (float)(raw_count - 1));
            raw_index = reverse_buf[rev_pos];
        }
        out_path->waypoints[i].x        = g_nodes[raw_index].x;
        out_path->waypoints[i].y        = g_nodes[raw_index].y;
        out_path->waypoints[i].theta    = g_nodes[raw_index].theta;
        out_path->waypoints[i].v_target = DEFAULT_TARGET_SPEED_MM_S;
    }
    return true;
}

/* -------------------------------------------------------------------------- */
/* Hybrid A* search                                                            */
/* -------------------------------------------------------------------------- */

static bool run_hybrid_astar(const quadtree_map_t *map,
                             const pose_t *start,
                             float goal_x, float goal_y,
                             path_t *out_path)
{
    if (!map_is_valid(map) || !start || !out_path) return false;

    /* Reset open-list structures. Nodes are initialised on creation so no
     * O(MAX_HYBRID_NODES) init loop is needed. */
    memset(g_hash, 0xFF, sizeof(g_hash));   /* 0xFFFF = HASH_EMPTY per slot */
    g_heap_size = 0;

    /* Start node */
    int node_count   = 1;
    hybrid_node_t *s = &g_nodes[0];
    s->x        = start->x;
    s->y        = start->y;
    s->theta    = wrap_pi(start->theta);
    s->g        = 0.0f;
    s->f        = heuristic_cost(start->x, start->y, goal_x, goal_y);
    s->parent   = 0;
    s->steer_id = 0;
    s->direction = 1;
    s->ix       = xy_to_ix(map, start->x);
    s->iy       = xy_to_iy(map, start->y);
    s->itheta   = theta_to_bin(start->theta);
    s->closed   = false;
    s->heap_pos = -1;

    hash_insert(0);
    heap_push(0);

    static const float  steering_values[3] = {
        -PLANNER_MAX_STEER_RAD, 0.0f, PLANNER_MAX_STEER_RAD
    };
    static const int8_t steering_ids[3]    = {-1, 0, 1};
    static const int    directions[1]      = {1};   /* extend to {1,-1} for reverse */

    while (g_heap_size > 0) {
        const int best = heap_pop();
        g_nodes[best].closed = true;
        if (node_count % 64 == 0) taskYIELD();

        if (goal_reached(g_nodes[best].x, g_nodes[best].y, goal_x, goal_y))
            return reconstruct_path(best, out_path);

        for (int d = 0; d < (int)(sizeof(directions)/sizeof(directions[0])); ++d) {
            for (int si = 0; si < 3; ++si) {
                float nx, ny, nt;
                const int    direction = directions[d];
                const float  steer     = steering_values[si];
                const int8_t steer_id  = steering_ids[si];

                if (!simulate_primitive(map,
                                        g_nodes[best].x,
                                        g_nodes[best].y,
                                        g_nodes[best].theta,
                                        steer, direction,
                                        &nx, &ny, &nt)) continue;

                const int16_t ix = xy_to_ix(map, nx);
                const int16_t iy = xy_to_iy(map, ny);
                const int16_t it = theta_to_bin(nt);
                const float tentative_g =
                    g_nodes[best].g +
                    motion_cost(direction, steer_id, g_nodes[best].steer_id);

                const int existing = hash_lookup(ix, iy, it);   /* O(1) */

                if (existing >= 0) {
                    if (g_nodes[existing].closed &&
                        tentative_g >= g_nodes[existing].g) continue;

                    if (tentative_g < g_nodes[existing].g) {
                        hybrid_node_t *en = &g_nodes[existing];
                        en->x = nx; en->y = ny; en->theta = nt;
                        en->g        = tentative_g;
                        en->f        = tentative_g +
                                       heuristic_cost(nx, ny, goal_x, goal_y);
                        en->parent   = best;
                        en->steer_id = steer_id;
                        en->direction = (int8_t)direction;
                        en->closed   = false;

                        if (en->heap_pos >= 0) {
                            /* Already open: f decreased → sift toward root. */
                            heap_sift_up((int)en->heap_pos);
                        } else {
                            /* Was closed: reopen. */
                            heap_push(existing);
                        }
                    }
                    continue;
                }

                /* New state */
                if (node_count >= MAX_HYBRID_NODES) {
                    ESP_LOGW(TAG, "node cap hit (%d); raise MAX_HYBRID_NODES "
                             "or coarsen resolution", MAX_HYBRID_NODES);
                    return false;
                }

                const int ni     = node_count++;
                hybrid_node_t *nn = &g_nodes[ni];
                nn->x        = nx; nn->y = ny; nn->theta = nt;
                nn->g        = tentative_g;
                nn->f        = tentative_g +
                               heuristic_cost(nx, ny, goal_x, goal_y);
                nn->parent   = best;
                nn->steer_id = steer_id;
                nn->direction = (int8_t)direction;
                nn->ix       = ix; nn->iy = iy; nn->itheta = it;
                nn->closed   = false;
                nn->heap_pos = -1;

                hash_insert((uint16_t)ni);
                heap_push(ni);
            }
        }
    }

    ESP_LOGW(TAG, "Hybrid A* failed: open set exhausted after %d nodes", node_count);
    return false;
}

/* -------------------------------------------------------------------------- */
/* Frontier approach and endpoint handling                                     */
/* -------------------------------------------------------------------------- */

static bool choose_goal_approach_pose(const quadtree_map_t *map,
                                      const frontier_t *goal,
                                      float *goal_x, float *goal_y)
{
    if (!map_is_valid(map) || !goal || !goal_x || !goal_y) return false;
    if (!is_pose_inside_map(map, goal->cx, goal->cy))      return false;

    if (point_robot_collision_free(map, goal->cx, goal->cy)) {
        *goal_x = goal->cx; *goal_y = goal->cy;
        return true;
    }

    const collect_result_t collected = collect_free_leaves(map, g_free_leaves);
    if (collected.truncated)
        ESP_LOGW(TAG, "free-leaf cap hit; nearest approach may be biased");

    const int idx = find_nearest_free_leaf(g_free_leaves, collected.count,
                                           goal->cx, goal->cy);
    if (idx < 0) return false;

    *goal_x = g_free_leaves[idx].cx;
    *goal_y = g_free_leaves[idx].cy;
    return true;
}

static bool append_frontier_if_safe(const quadtree_map_t *map,
                                    const frontier_t *goal,
                                    path_t *path)
{
    if (!map || !goal || !path || path->length == 0) return false;
    if (path->length >= MAX_PATH_WAYPOINTS)          return true;

    waypoint_t *last = &path->waypoints[path->length - 1];
    const float d = dist_xy(last->x, last->y, goal->cx, goal->cy);
    if (d <= 1.0f) return true;

    const bool free_seg =
        line_is_collision_free_quadtree(map, last->x, last->y,
                                        goal->cx, goal->cy);
    if (!free_seg) {
        if (d > MAX_FRONTIER_APPEND_MM)                    return true;
        if (!is_pose_inside_map(map, goal->cx, goal->cy))  return true;
        if (qt_query_const(map, goal->cx, goal->cy) > 0)   return true;
    }

    const float heading = atan2f(goal->cy - last->y, goal->cx - last->x);
    last->theta = heading;
    path->waypoints[path->length].x        = goal->cx;
    path->waypoints[path->length].y        = goal->cy;
    path->waypoints[path->length].theta    = heading;
    path->waypoints[path->length].v_target = DEFAULT_TARGET_SPEED_MM_S;
    path->length++;
    return true;
}

/* -------------------------------------------------------------------------- */
/* Public planner                                                              */
/* -------------------------------------------------------------------------- */

path_t hybrid_astar_plan(const quadtree_map_t *map,
                         const pose_t *start,
                         const frontier_t *goal)
{
    path_t path = hybrid_astar_empty_path();
    float  approach_x, approach_y;

    if (!map_is_valid(map) || !start || !goal)           return path;
    if (!is_pose_inside_map(map, start->x, start->y) ||
        !is_pose_inside_map(map, goal->cx, goal->cy))    return path;
    if (!point_robot_collision_free(map, start->x, start->y)) {
        ESP_LOGW(TAG, "start pose is not in known-free space");
        return path;
    }
    if (!choose_goal_approach_pose(map, goal, &approach_x, &approach_y)) {
        ESP_LOGW(TAG, "failed to choose frontier approach pose");
        return path;
    }
    if (!run_hybrid_astar(map, start, approach_x, approach_y, &path))
        return hybrid_astar_empty_path();

    append_frontier_if_safe(map, goal, &path);
    return path;
}
