/**
 * test_frontier.c — Profiled Wavefront Frontier Detection test.
 *
 * Calls the real frontier_detector module on a synthetic circular room.
 * Tests: frontier_detector_detect() + frontier_detector_best() per cycle.
 *
 * Board: ESP32-S3
 * Rate:  5 Hz (200 ms delay) — WFD BFS is ~10-50 ms on a 3.2 m² free area
 * Budget warning: >50 ms cycle time
 *
 * Map layout: robot at (5000, 5000) mm
 *   - Free disk  radius 1600 mm, 150 mm grid step
 *   - Wall ring  radius 2500 mm, 5° step
 *   - Unknown beyond wall ring (never inserted → log-odds = 0)
 */

#include "../../profiler.h"
#include "../../../esp32s3/src/quadtree_map.h"
#include "../../../esp32s3/src/frontier_detector.h"
#include "../../../types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <math.h>

static const char *TAG = "TEST_FRONTIER";

#define ROBOT_X 5000.0f
#define ROBOT_Y 5000.0f
#define MAP_W   10000.0f
#define MAP_H   10000.0f
#define MAP_STEP  200.0f
#define FREE_R  1600.0f
#define WALL_R  2500.0f
#define GRID_STEP 150.0f

static task_profile_t s_profile;

/* ── Build a simple circular test room centred at (robot_x, robot_y) ──────── */
static void build_test_room(quadtree_map_t *map, float robot_x, float robot_y)
{
    /* Free disk */
    for (float dx = -FREE_R; dx <= FREE_R; dx += GRID_STEP) {
        for (float dy = -FREE_R; dy <= FREE_R; dy += GRID_STEP) {
            if (dx * dx + dy * dy <= FREE_R * FREE_R) {
                quadtree_map_insert(map, robot_x + dx, robot_y + dy, CLASS_FREE);
            }
        }
    }
    /* Wall ring — creates the unknown/free boundary that WFD detects as frontiers */
    for (float angle = 0.0f; angle < 360.0f; angle += 5.0f) {
        float rad = angle * 3.14159265f / 180.0f;
        quadtree_map_insert(map,
                            robot_x + WALL_R * cosf(rad),
                            robot_y + WALL_R * sinf(rad),
                            CLASS_WALL);
    }
}

task_profile_t *frontier_task_get_profile(void) { return &s_profile; }

void frontier_test_task(void *arg)
{
    (void)arg;

    task_profile_init(&s_profile, "frontier_test",
                      8192, /* stack bytes — BFS uses internal static buffers ~18.5 KB */
                      3,    /* priority */
                      0);   /* core 0 — compute-intensive */

    /* frontier_list_t returned by value (32 clusters × ~12 bytes = ~384 bytes) */
    task_data_profile_init(&s_profile.data_profile,
                           "frontier_t",
                           sizeof(frontier_t),
                           /*heap_allocated=*/ false,
                           /*dma_memory=*/     false,
                           /*passes_ownership=*/false,
                           NULL);

    /* Allocate map on heap — pool is 8000 nodes × 12 bytes = 96 KB */
    quadtree_map_t map;
    quadtree_map_init(&map, MAP_W, MAP_H, MAP_STEP);
    build_test_room(&map, ROBOT_X, ROBOT_Y);

    pose_t robot_pose = { .x = ROBOT_X, .y = ROBOT_Y, .theta = 0.0f };

    while (1) {
        task_profile_cycle_begin(&s_profile);

        PROFILE_CPU_BEGIN(detect);
        frontier_list_t frontiers = frontier_detector_detect(&map, &robot_pose);
        uint32_t detect_us;
        PROFILE_CPU_END(detect, &detect_us);

        PROFILE_CPU_BEGIN(best);
        frontier_t best = frontier_detector_best(&frontiers, &robot_pose);
        uint32_t best_us;
        PROFILE_CPU_END(best, &best_us);

        task_data_profile_update(&s_profile.data_profile, frontiers.count);

        task_profile_cycle_end(&s_profile);

        if (s_profile.cpu_cycles_last / 240 > 50000) {
            ESP_LOGW(TAG, "cycle %" PRIu32 " µs — over 50 ms budget (detect %" PRIu32
                     " µs, best %" PRIu32 " µs)",
                     s_profile.cpu_cycles_last / 240, detect_us, best_us);
        }

        ESP_LOGI(TAG, "frontiers=%u best=(%.0f,%.0f) detect=%" PRIu32 " µs",
                 frontiers.count, (double)best.cx, (double)best.cy, detect_us);

        vTaskDelay(pdMS_TO_TICKS(200));   /* 5 Hz */
    }
}
