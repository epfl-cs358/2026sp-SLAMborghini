/**
 * test_pipeline_scan_to_map.c — Pipeline Stage 1: LiDAR → QuadTree → lidar_to_map.
 *
 * Runs the first stage of the ESP32-S3 SLAM pipeline:
 *   1. Acquire a LiDAR scan (real hardware or synthetic 360-pt sweep)
 *   2. Integrate the scan into a shared quadtree map via lidar_to_map()
 *
 * The resulting map is stored in g_shared_map (mutex-protected) and consumed
 * by test_pipeline_frontier.c running as a parallel task at half the rate.
 *
 * Hardware flag (set in CMake or sdkconfig):
 *   -DPIPELINE_USE_REAL_LIDAR=1   use lidar_driver_read_scan()  (needs RPLiDAR C1)
 *   -DPIPELINE_USE_REAL_LIDAR=0   generate a 360-pt synthetic scan (default)
 *
 * Board: ESP32-S3
 * Rate:  10 Hz (real LiDAR drives the rate; 100 ms delay when synthetic)
 * Budget warning: >120 ms (scan read + lidar_to_map)
 */

#include "../../profiler.h"
#include "../../../esp32s3/src/quadtree_map.h"
#include "../../../esp32s3/src/lidar_driver.h"
#include "../../../esp32s3/src/lidar_to_map.h"
#include "../../../types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <math.h>
#include <string.h>

static const char *TAG = "PIPE_SCAN2MAP";

#ifndef PIPELINE_USE_REAL_LIDAR
#  define PIPELINE_USE_REAL_LIDAR 0
#endif

#define MAP_W        10000.0f
#define MAP_H        10000.0f
#define MAP_STEP       200.0f
#define MAX_RANGE_MM  4000.0f
#define RAY_STEP_MM    100.0f
#define ROBOT_X       5000.0f
#define ROBOT_Y       5000.0f

/* ── Shared state (consumed by test_pipeline_frontier.c) ──────────────────── */
SemaphoreHandle_t g_map_mutex;
quadtree_map_t    g_shared_map;
pose_t            g_robot_pose;
volatile bool     g_map_ready = false;

/* ── Task-local ───────────────────────────────────────────────────────────── */
static task_profile_t s_profile;
static lidar_scan_t   s_scan;    /* ~5.5 KB — static */

static void build_synthetic_scan(lidar_scan_t *scan, uint32_t tick)
{
    scan->count = 360;
    scan->scan_start_us      = (uint32_t)esp_timer_get_time();
    scan->rotation_period_us = 100000;
    for (int i = 0; i < 360; i++) {
        scan->points[i].theta_deg = (float)i;
        /* Simulate a slightly irregular room boundary */
        scan->points[i].r_mm = 1800.0f
            + 400.0f * sinf((float)i * 3.14159265f / 60.0f)
            + 100.0f * sinf((float)(i + tick) * 3.14159265f / 15.0f);
        scan->points[i].intensity = 200;
    }
}

task_profile_t *scan_to_map_task_get_profile(void) { return &s_profile; }

void scan_to_map_task(void *arg)
{
    (void)arg;

    task_profile_init(&s_profile, "scan_to_map",
                      8192,
                      5,    /* priority — LiDAR I/O must not be starved */
                      1);   /* core 1 — I/O on core 1 */

    task_data_profile_init(&s_profile.data_profile,
                           "lidar_scan_point_t",
                           sizeof(lidar_scan_point_t),
                           /*heap_allocated=*/ false,
                           /*dma_memory=*/     false,
                           /*passes_ownership=*/false,
                           NULL);

    /* Init shared state */
    g_map_mutex = xSemaphoreCreateMutex();
    configASSERT(g_map_mutex);
    quadtree_map_init(&g_shared_map, MAP_W, MAP_H, MAP_STEP);
    g_robot_pose = (pose_t){ .x = ROBOT_X, .y = ROBOT_Y, .theta = 0.0f };

#if PIPELINE_USE_REAL_LIDAR
    lidar_driver_init();
#endif

    uint32_t tick = 0;

    while (1) {
        task_profile_cycle_begin(&s_profile);

        /* ── Stage 1a: acquire scan ────────────────────────────────────────── */
        PROFILE_CPU_BEGIN(scan_read);
#if PIPELINE_USE_REAL_LIDAR
        bool ok = lidar_driver_read_scan(&s_scan);
        if (!ok) {
            ESP_LOGW(TAG, "lidar_driver_read_scan() failed");
            task_profile_cycle_end(&s_profile);
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
#else
        build_synthetic_scan(&s_scan, tick++);
        bool ok = true;
        (void)ok;
#endif
        uint32_t scan_us;
        PROFILE_CPU_END(scan_read, &scan_us);

        /* ── Stage 1b: integrate into shared map ───────────────────────────── */
        map_dirty_rect_t dirty = {0};

        xSemaphoreTake(g_map_mutex, portMAX_DELAY);

        PROFILE_CPU_BEGIN(lidar_to_map_call);
        lidar_to_map(&g_shared_map, &s_scan, &g_robot_pose,
                     MAX_RANGE_MM, RAY_STEP_MM, &dirty);
        uint32_t l2m_us;
        PROFILE_CPU_END(lidar_to_map_call, &l2m_us);

        g_map_ready = true;
        uint16_t map_nodes = g_shared_map.count;

        xSemaphoreGive(g_map_mutex);

        task_data_profile_update(&s_profile.data_profile, s_scan.count);

        task_profile_cycle_end(&s_profile);

        uint32_t cycle_us = s_profile.cpu_cycles_last / 240;
        if (cycle_us > 120000) {
            ESP_LOGW(TAG, "cycle %" PRIu32 " µs over 120 ms budget "
                     "(scan %" PRIu32 " µs l2m %" PRIu32 " µs)",
                     cycle_us, scan_us, l2m_us);
        }

        ESP_LOGI(TAG, "pts=%u scan=%" PRIu32 " µs l2m=%" PRIu32 " µs "
                 "nodes=%u dirty=(%.0f,%.0f)-(%.0f,%.0f)",
                 s_scan.count, scan_us, l2m_us,
                 map_nodes,
                 (double)dirty.x_min, (double)dirty.y_min,
                 (double)dirty.x_max, (double)dirty.y_max);

        /* Reset map when pool approaches saturation */
        if (qt_is_pool_full(&g_shared_map)) {
            xSemaphoreTake(g_map_mutex, portMAX_DELAY);
            ESP_LOGW(TAG, "map pool full (%u nodes) — resetting", g_shared_map.count);
            qt_free(&g_shared_map);
            quadtree_map_init(&g_shared_map, MAP_W, MAP_H, MAP_STEP);
            g_map_ready = false;
            xSemaphoreGive(g_map_mutex);
        }

#if !PIPELINE_USE_REAL_LIDAR
        vTaskDelay(pdMS_TO_TICKS(100));   /* 10 Hz when synthetic */
#endif
    }
}
