/**
 * test_hybrid_astar.c — Comprehensive Hybrid A* profiling.
 *
 * hybrid_astar_plan() has 5 internal stages (all static, not exposed):
 *
 *   Stage 1  collect_free_leaves()        O(N) DFS on quadtree
 *   Stage 2  build_adjacency()            O(N²) pairwise LOS checks  ← main bottleneck
 *   Stage 3  find_containing/nearest()    O(N) start/goal node lookup
 *   Stage 4  run_quadtree_graph_astar()   O(N²) open-set linear scan A*
 *   Stage 5  smooth_path_quadtree()       iterative LOS shortcutting
 *
 * Since these stages are static, this test stresses them indirectly through
 * four scenarios of increasing difficulty, rotated every cycle:
 *
 *   SCENARIO A  Short path   goal  700 mm right    few nodes, fast adjacency
 *   SCENARIO B  Medium path  goal 1400 mm right    moderate complexity
 *   SCENARIO C  Long path    goal near wall edge   max free leaves, hardest
 *   SCENARIO D  Bad goal     goal inside wall      must return length=0 cleanly
 *
 * ── Per-cycle log ────────────────────────────────────────────────────────────
 *   scenario    A/B/C/D
 *   plan_us     total hybrid_astar_plan() wall time
 *   valid       whether a path was found
 *   waypoints   number of waypoints in the path
 *   path_mm     total path length in mm (sum of segment lengths)
 *   step_mm     average inter-waypoint distance
 *
 * ── Per-scenario min/max/avg summary (every 20 cycles) ───────────────────────
 *   plan_us min/max/avg per scenario — shows variance
 *
 * ── Static RAM breakdown (logged once at init) ───────────────────────────────
 *   nodes[512]  qt_graph_node_t × 512  ≈ 30 KB  (.bss, static inside plan())
 *   states[512] astar_state_t   × 512  ≈ 10 KB  (.bss, static inside astar())
 *   s_path      path_t                 ≈  1 KB  (test-local static)
 *   map pool    quadtree_map_t         ≈ 96 KB  (test-local static)
 *
 * ── Memory + stack snapshot (every 10 cycles) ────────────────────────────────
 *
 * ── Perfmon (every 20 cycles, requires -DPROFILER_USE_PERFMON) ───────────────
 *   Measured on scenario B (representative mid-complexity plan)
 *   IPC, D-cache stall%, I-cache stall%, branch mispred%
 *
 * Board: ESP32-S3
 * Rate:  2 Hz (500 ms delay) — A* on dense map can take 50–200 ms
 * Budget warning: >150 ms
 */

#include "../../profiler.h"
#include "../../../esp32s3/src/quadtree_map.h"
#include "../../../esp32s3/src/hybrid_astar.h"
#include "../../../types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include <math.h>
#include <string.h>

static const char *TAG = "TEST_ASTAR";

/* ── Map geometry ────────────────────────────────────────────────────────── */
#define ROBOT_X    5000.0f
#define ROBOT_Y    5000.0f
#define MAP_W     10000.0f
#define MAP_H     10000.0f
#define MAP_STEP    200.0f
#define FREE_R     1600.0f   /* radius of traversable free disk */
#define WALL_R     2500.0f   /* ring of wall obstacles */
#define GRID_STEP   150.0f   /* free-cell seeding step */

/* ── Scenario goals ──────────────────────────────────────────────────────── */
#define GOAL_A_X  (ROBOT_X + 700.0f)   /* short  — inside free disk */
#define GOAL_A_Y   ROBOT_Y

#define GOAL_B_X  (ROBOT_X + 1400.0f)  /* medium — near edge of free disk */
#define GOAL_B_Y   ROBOT_Y

#define GOAL_C_X  (ROBOT_X + 1550.0f)  /* long   — right at free-disk edge (max leaves) */
#define GOAL_C_Y   ROBOT_Y

#define GOAL_D_X  (ROBOT_X + 2600.0f)  /* bad    — outside wall (unreachable) */
#define GOAL_D_Y   ROBOT_Y

/* ── A* internal static sizes (from hybrid_astar.c source) ──────────────── */
#define ASTAR_MAX_LEAVES    512
/* qt_graph_node_t: 6×float + int8 + uint16[16] + uint8 = 58 B → aligned 60 B */
#define SIZEOF_QT_GRAPH_NODE  60u
/* astar_state_t: 2×bool + 3×float + int = 18 B → aligned 20 B */
#define SIZEOF_ASTAR_STATE    20u

static task_profile_t s_profile;
static quadtree_map_t s_map;
static path_t         s_path;   /* 64 × waypoint_t ≈ 1 KB */

/* ── Per-scenario statistics ─────────────────────────────────────────────── */
typedef struct {
    const char *name;
    uint32_t    count;
    uint32_t    plan_us_min;
    uint32_t    plan_us_max;
    float       plan_us_avg;
    uint32_t    fail_count;
} scenario_stat_t;

static scenario_stat_t s_stats[4];

static void stat_update(scenario_stat_t *s, uint32_t us, bool valid)
{
    s->count++;
    if (!valid) { s->fail_count++; return; }
    if (us < s->plan_us_min) s->plan_us_min = us;
    if (us > s->plan_us_max) s->plan_us_max = us;
    s->plan_us_avg += ((float)us - s->plan_us_avg) / (float)s->count;
}

/* ── Compute total path length in mm ────────────────────────────────────── */
static float path_length_mm(const path_t *p)
{
    float total = 0.0f;
    for (int i = 0; i + 1 < p->length; i++) {
        float dx = p->waypoints[i+1].x - p->waypoints[i].x;
        float dy = p->waypoints[i+1].y - p->waypoints[i].y;
        total += sqrtf(dx*dx + dy*dy);
    }
    return total;
}

/* ── Build the test map: free disk + wall ring ───────────────────────────── */
static void build_test_room(quadtree_map_t *map)
{
    for (float dx = -FREE_R; dx <= FREE_R; dx += GRID_STEP)
        for (float dy = -FREE_R; dy <= FREE_R; dy += GRID_STEP)
            if (dx*dx + dy*dy <= FREE_R*FREE_R)
                quadtree_map_insert(map, ROBOT_X+dx, ROBOT_Y+dy, CLASS_FREE);

    for (float a = 0.0f; a < 360.0f; a += 3.0f) {
        float r = a * 3.14159265f / 180.0f;
        quadtree_map_insert(map, ROBOT_X + WALL_R*cosf(r),
                                 ROBOT_Y + WALL_R*sinf(r), CLASS_WALL);
    }
}

#ifdef PROFILER_USE_PERFMON
typedef struct { const quadtree_map_t *map; const pose_t *start; const frontier_t *goal; } plan_work_t;
static void _plan_work(void *arg)
{
    plan_work_t *w = (plan_work_t *)arg;
    hybrid_astar_plan(w->map, w->start, w->goal);
}
#endif

task_profile_t *hybrid_astar_task_get_profile(void) { return &s_profile; }

void hybrid_astar_test_task(void *arg)
{
    (void)arg;

    task_profile_init(&s_profile, "hybrid_astar_test",
                      8192,
                      3,
                      0);   /* core 0 — compute-intensive */

    task_data_profile_init(&s_profile.data_profile,
                           "waypoint_t",
                           sizeof(waypoint_t),
                           false, false, false, NULL);

    /* ── Static RAM breakdown ─────────────────────────────────────────────── */
    ESP_LOGI(TAG, "=== STATIC RAM (hybrid_astar internals) ===");
    ESP_LOGI(TAG, "  nodes[%d] qt_graph_node_t  %u × %u B = %u B  (.bss)",
             ASTAR_MAX_LEAVES, (unsigned)ASTAR_MAX_LEAVES,
             SIZEOF_QT_GRAPH_NODE,
             (unsigned)(ASTAR_MAX_LEAVES * SIZEOF_QT_GRAPH_NODE));
    ESP_LOGI(TAG, "  states[%d] astar_state_t   %u × %u B = %u B  (.bss)",
             ASTAR_MAX_LEAVES, (unsigned)ASTAR_MAX_LEAVES,
             SIZEOF_ASTAR_STATE,
             (unsigned)(ASTAR_MAX_LEAVES * SIZEOF_ASTAR_STATE));
    ESP_LOGI(TAG, "  path_t s_path              %u B  (test static)",
             (unsigned)sizeof(path_t));
    ESP_LOGI(TAG, "  quadtree_map_t pool        ~96 KB (test static)");
    ESP_LOGI(TAG, "  TOTAL internal DRAM        ~%u KB",
             (unsigned)((ASTAR_MAX_LEAVES * SIZEOF_QT_GRAPH_NODE
                       + ASTAR_MAX_LEAVES * SIZEOF_ASTAR_STATE) / 1024u));

    /* ── Heap before map init ────────────────────────────────────────────── */
    uint32_t heap_pre = esp_get_free_heap_size();

    quadtree_map_init(&s_map, MAP_W, MAP_H, MAP_STEP);
    build_test_room(&s_map);

    ESP_LOGI(TAG, "Map built: %u nodes  heap_used=%u B",
             s_map.count,
             (unsigned)(heap_pre - esp_get_free_heap_size()));

    /* ── Scenario definitions ─────────────────────────────────────────────── */
    pose_t     start = { .x = ROBOT_X, .y = ROBOT_Y, .theta = 0.0f };
    frontier_t goals[4] = {
        { .cx = GOAL_A_X, .cy = GOAL_A_Y, .size = 1 },   /* A: short  */
        { .cx = GOAL_B_X, .cy = GOAL_B_Y, .size = 1 },   /* B: medium */
        { .cx = GOAL_C_X, .cy = GOAL_C_Y, .size = 1 },   /* C: long   */
        { .cx = GOAL_D_X, .cy = GOAL_D_Y, .size = 1 },   /* D: bad    */
    };
    const char *scenario_names[4] = { "A(short)", "B(medium)", "C(long)", "D(bad)" };

    /* Init stats */
    for (int i = 0; i < 4; i++) {
        s_stats[i].name        = scenario_names[i];
        s_stats[i].count       = 0;
        s_stats[i].plan_us_min = UINT32_MAX;
        s_stats[i].plan_us_max = 0;
        s_stats[i].plan_us_avg = 0.0f;
        s_stats[i].fail_count  = 0;
    }

#ifdef PROFILER_USE_PERFMON
    task_perfmon_profile_t pm;
    task_perfmon_profile_init(&pm, "astar_plan_B", -1, 1);
    plan_work_t pm_work = { &s_map, &start, &goals[1] };   /* scenario B */
#endif

    uint32_t cycle = 0;

    while (1) {
        int sc = (int)(cycle % 4u);   /* rotate through A→B→C→D */

        task_profile_cycle_begin(&s_profile);

        /* ── Run the planner ─────────────────────────────────────────────── */
        PROFILE_CPU_BEGIN(plan);
        s_path = hybrid_astar_plan(&s_map, &start, &goals[sc]);
        uint32_t plan_us;
        PROFILE_CPU_END(plan, &plan_us);

        bool valid = hybrid_astar_is_valid(&s_path);

        float plen_mm  = valid ? path_length_mm(&s_path) : 0.0f;
        float step_mm  = (valid && s_path.length > 1)
                         ? plen_mm / (float)(s_path.length - 1) : 0.0f;

        task_data_profile_update(&s_profile.data_profile,
                                 valid ? s_path.length : 0u);
        task_profile_cycle_end(&s_profile);

        stat_update(&s_stats[sc], plan_us, valid);

        /* ── Budget warning ──────────────────────────────────────────────── */
        if (plan_us > 150000u) {
            ESP_LOGW(TAG, "[%s] SLOW plan=%" PRIu32 " µs (budget 150 ms)",
                     scenario_names[sc], plan_us);
        }
        if (sc != 3 && !valid) {
            ESP_LOGW(TAG, "[%s] PLANNING FAILED — path not found (expected valid)",
                     scenario_names[sc]);
        }
        if (sc == 3 && valid) {
            ESP_LOGW(TAG, "[D] Path found to unreachable goal — collision check bug?");
        }

        /* ── Per-cycle log ───────────────────────────────────────────────── */
        ESP_LOGI(TAG,
                 "[%s] plan=%5" PRIu32 "µs  valid=%d"
                 "  wpts=%2u  path=%.0fmm  step=%.0fmm",
                 scenario_names[sc], plan_us, (int)valid,
                 valid ? s_path.length : 0u,
                 (double)plen_mm, (double)step_mm);

        /* ── Summary every 20 cycles (5 full A→B→C→D rotations) ─────────── */
        if (cycle > 0 && cycle % 20u == 0u) {
            ESP_LOGI(TAG, "══ SCENARIO SUMMARY (cycle %" PRIu32 ") ══", cycle);
            for (int i = 0; i < 4; i++) {
                scenario_stat_t *s = &s_stats[i];
                if (s->count == 0) continue;
                ESP_LOGI(TAG,
                         "  [%s] n=%u  fails=%u"
                         "  min=%"PRIu32"µs  max=%"PRIu32"µs  avg=%.0fµs",
                         s->name, s->count, s->fail_count,
                         s->plan_us_min == UINT32_MAX ? 0u : s->plan_us_min,
                         s->plan_us_max,
                         (double)s->plan_us_avg);
            }
        }

        /* ── Memory + stack snapshot every 10 cycles ─────────────────────── */
        if (cycle % 10u == 0u) {
            uint32_t heap_free   = esp_get_free_heap_size();
            uint32_t heap_min    = esp_get_minimum_free_heap_size();
            uint32_t largest_blk = heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT);
            uint32_t frag_pct    = (heap_free > 0u)
                                   ? (uint32_t)(100u - largest_blk * 100u / heap_free)
                                   : 0u;
            uint32_t stack_free  = uxTaskGetStackHighWaterMark(NULL) * sizeof(StackType_t);

            ESP_LOGI(TAG,
                     "=MEM= heap=%" PRIu32 "B  wm=%" PRIu32 "B"
                     "  blk=%" PRIu32 "B  frag=%" PRIu32 "%%"
                     "  stack=%" PRIu32 "B  map_nodes=%u",
                     heap_free, heap_min, largest_blk, frag_pct,
                     stack_free, s_map.count);
        }

#ifdef PROFILER_USE_PERFMON
        /* ── Perfmon on scenario B every 20 cycles ───────────────────────── */
        if (cycle % 20u == 0u) {
            task_perfmon_collect(&pm, _plan_work, &pm_work, -1, 1);
            ESP_LOGI(TAG, "=PERFMON hybrid_astar scenario B=");
            task_perfmon_profile_dump(&pm);
        }
#endif

        cycle++;
        vTaskDelay(pdMS_TO_TICKS(500));   /* 2 Hz */
    }
}
