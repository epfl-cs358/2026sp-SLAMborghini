/**
 * test_pipeline_frontier.c — Pipeline Stage 2: Map → Frontier Detection.
 *
 * Reads the shared quadtree map produced by test_pipeline_scan_to_map.c and
 * runs frontier_detector_detect() + frontier_detector_best() on it.
 *
 * Must run TOGETHER with scan_to_map_task (they share g_shared_map via mutex).
 * Runs at 5 Hz — half the scan rate — so the map has time to accumulate
 * several scans between each frontier detection pass.
 *
 * Board: ESP32-S3
 * Rate:  5 Hz (200 ms delay)
 * Budget warning: >50 ms
 *
 * Data flow:
 *   scan_to_map_task  → g_shared_map (mutex) → frontier_task
 *                                               → g_best_frontier (volatile, no mutex:
 *                                                 written once per cycle, 4-byte aligned)
 */

#include "../../profiler.h"
#include "../../../esp32s3/src/quadtree_map.h"
#include "../../../esp32s3/src/frontier_detector.h"
#include "../../../types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"

static const char *TAG = "PIPE_FRONTIER";

/* Shared state written by scan_to_map_task */
extern SemaphoreHandle_t g_map_mutex;
extern quadtree_map_t    g_shared_map;
extern pose_t            g_robot_pose;
extern volatile bool     g_map_ready;

/* Best frontier published for downstream consumers (e.g., a planner task) */
volatile frontier_t g_best_frontier;
volatile bool       g_frontier_valid = false;

static task_profile_t s_profile;

task_profile_t *pipeline_frontier_task_get_profile(void) { return &s_profile; }

void pipeline_frontier_task(void *arg)
{
    (void)arg;

    task_profile_init(&s_profile, "pipe_frontier",
                      8192,  /* frontier detector uses ~18.5 KB internal static buffers */
                      3,
                      0);    /* core 0 — compute; scan_to_map runs on core 1 */

    task_data_profile_init(&s_profile.data_profile,
                           "frontier_t",
                           sizeof(frontier_t),
                           /*heap_allocated=*/ false,
                           /*dma_memory=*/     false,
                           /*passes_ownership=*/false,
                           NULL);

    /* Wait until scan_to_map_task has populated the map at least once */
    while (!g_map_ready) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    while (1) {
        task_profile_cycle_begin(&s_profile);

        /* Lock map for the duration of frontier detection.
         * WFD BFS reads map cells but does not write — lock still required
         * because scan_to_map_task may reset the pool concurrently. */
        xSemaphoreTake(g_map_mutex, portMAX_DELAY);

        PROFILE_CPU_BEGIN(detect);
        frontier_list_t frontiers = frontier_detector_detect(&g_shared_map, &g_robot_pose);
        uint32_t detect_us;
        PROFILE_CPU_END(detect, &detect_us);

        PROFILE_CPU_BEGIN(best);
        frontier_t best = frontier_detector_best(&frontiers, &g_robot_pose);
        uint32_t best_us;
        PROFILE_CPU_END(best, &best_us);

        xSemaphoreGive(g_map_mutex);

        /* Publish for downstream (no extra lock — struct copy is atomic enough
         * for a profiler demo; production code would use a proper handoff) */
        g_best_frontier  = best;
        g_frontier_valid = (frontiers.count > 0);

        task_data_profile_update(&s_profile.data_profile, frontiers.count);

        task_profile_cycle_end(&s_profile);

        uint32_t cycle_us = s_profile.cpu_cycles_last / 240;
        if (cycle_us > 50000) {
            ESP_LOGW(TAG, "cycle %" PRIu32 " µs over 50 ms budget "
                     "(detect %" PRIu32 " µs best %" PRIu32 " µs)",
                     cycle_us, detect_us, best_us);
        }

        ESP_LOGI(TAG, "frontiers=%u best=(%.0f,%.0f) "
                 "detect=%" PRIu32 " µs best=%" PRIu32 " µs",
                 frontiers.count,
                 (double)best.cx, (double)best.cy,
                 detect_us, best_us);

        vTaskDelay(pdMS_TO_TICKS(200));   /* 5 Hz — map accumulates 2 scans per frontier pass */
    }
}
