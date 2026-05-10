/**
 * test_hybrid_astar.c — Profiled Hybrid A* path planner test.
 *
 * Calls hybrid_astar_plan() each cycle on a synthetic circular room.
 * Uses the same build_test_room() helper as test_frontier.c.
 * Goal: frontier at (6400, 5000) mm — directly to the robot's right,
 * inside the free disk (r < 1600 mm from robot at (5000,5000)).
 *
 * Board: ESP32-S3
 * Rate:  2 Hz (500 ms delay) — A* on 512 leaf nodes can take 50-150 ms
 * Budget warning: >100 ms
 *
 * path_t (~1 KB) declared static to avoid stack overflow.
 */

#include "../../profiler.h"
#include "../../../esp32s3/src/quadtree_map.h"
#include "../../../esp32s3/src/hybrid_astar.h"
#include "../../../types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <math.h>

static const char *TAG = "TEST_ASTAR";

#define ROBOT_X  5000.0f
#define ROBOT_Y  5000.0f
#define MAP_W   10000.0f
#define MAP_H   10000.0f
#define MAP_STEP  200.0f
#define FREE_R   1600.0f
#define WALL_R   2500.0f
#define GRID_STEP 150.0f
#define GOAL_X   6400.0f   /* 1400 mm to the right — reachable inside free disk */
#define GOAL_Y   5000.0f

static task_profile_t s_profile;
static path_t         s_path;   /* 64 waypoints × ~16 bytes ≈ 1 KB — static */

static void build_test_room(quadtree_map_t *map, float robot_x, float robot_y)
{
    for (float dx = -FREE_R; dx <= FREE_R; dx += GRID_STEP) {
        for (float dy = -FREE_R; dy <= FREE_R; dy += GRID_STEP) {
            if (dx * dx + dy * dy <= FREE_R * FREE_R)
                quadtree_map_insert(map, robot_x + dx, robot_y + dy, CLASS_FREE);
        }
    }
    for (float angle = 0.0f; angle < 360.0f; angle += 5.0f) {
        float rad = angle * 3.14159265f / 180.0f;
        quadtree_map_insert(map,
                            robot_x + WALL_R * cosf(rad),
                            robot_y + WALL_R * sinf(rad),
                            CLASS_WALL);
    }
}

task_profile_t *hybrid_astar_task_get_profile(void) { return &s_profile; }

void hybrid_astar_test_task(void *arg)
{
    (void)arg;

    task_profile_init(&s_profile, "hybrid_astar_test",
                      8192,  /* stack bytes — A* internal state can be large */
                      3,
                      0);    /* core 0 — compute-intensive */

    /* Item = one waypoint in the planned path */
    task_data_profile_init(&s_profile.data_profile,
                           "waypoint_t",
                           sizeof(waypoint_t),
                           /*heap_allocated=*/ false,
                           /*dma_memory=*/     false,
                           /*passes_ownership=*/false,
                           NULL);

    quadtree_map_t map;
    quadtree_map_init(&map, MAP_W, MAP_H, MAP_STEP);
    build_test_room(&map, ROBOT_X, ROBOT_Y);

    pose_t start = { .x = ROBOT_X, .y = ROBOT_Y, .theta = 0.0f };
    frontier_t goal = { .cx = GOAL_X, .cy = GOAL_Y, .size = 1 };

    while (1) {
        task_profile_cycle_begin(&s_profile);

        PROFILE_CPU_BEGIN(plan);
        s_path = hybrid_astar_plan(&map, &start, &goal);
        uint32_t plan_us;
        PROFILE_CPU_END(plan, &plan_us);

        bool valid = hybrid_astar_is_valid(&s_path);
        task_data_profile_update(&s_profile.data_profile,
                                 valid ? s_path.length : 0);

        task_profile_cycle_end(&s_profile);

        uint32_t cycle_us = s_profile.cpu_cycles_last / 240;
        if (cycle_us > 100000) {
            ESP_LOGW(TAG, "cycle %" PRIu32 " µs over 100 ms budget (plan %" PRIu32 " µs)",
                     cycle_us, plan_us);
        }

        ESP_LOGI(TAG, "valid=%d waypoints=%u plan=%" PRIu32 " µs",
                 (int)valid, valid ? s_path.length : 0, plan_us);

        vTaskDelay(pdMS_TO_TICKS(500));   /* 2 Hz */
    }
}
