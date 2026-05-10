/**
 * test_lidar_to_map.c — Profiled lidar_to_map ray-marching integration test.
 *
 * Calls lidar_to_map() with a synthetic 360-point scan on a fresh map each cycle.
 * Tests how long the ray-marching occupancy update takes as a function of point count.
 *
 * Board: ESP32-S3
 * Rate:  10 Hz (100 ms delay)
 * Budget warning: >50 ms per cycle
 *
 * lidar_scan_t (~5.5 KB) and QuadTreeMap pool (96 KB) declared static.
 * Map is re-initialised each cycle to prevent pool saturation.
 */

#include "../../profiler.h"
#include "../../../esp32s3/src/quadtree_map.h"
#include "../../../esp32s3/src/lidar_to_map.h"
#include "../../../types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <math.h>
#include <string.h>

static const char *TAG = "TEST_L2M";

#define MAP_W        10000.0f
#define MAP_H        10000.0f
#define MAP_STEP       200.0f
#define MAX_RANGE_MM  4000.0f
#define RAY_STEP_MM    100.0f
#define ROBOT_X       5000.0f
#define ROBOT_Y       5000.0f
#define N_POINTS         360

static task_profile_t s_profile;
static lidar_scan_t   s_scan;     /* ~5.5 KB — static */
static quadtree_map_t s_map;

/* Build a synthetic 360-point scan: uniform radial distances 2000–3500 mm */
static void build_synthetic_scan(lidar_scan_t *scan, uint32_t tick)
{
    scan->count = N_POINTS;
    scan->scan_start_us = (uint32_t)esp_timer_get_time();
    scan->rotation_period_us = 100000; /* 10 Hz */
    for (int i = 0; i < N_POINTS; i++) {
        scan->points[i].theta_deg  = (float)i;
        scan->points[i].r_mm       = 2000.0f + 500.0f * sinf((float)(i + tick) * 0.05f);
        scan->points[i].intensity  = 200;
    }
}

task_profile_t *lidar_to_map_task_get_profile(void) { return &s_profile; }

void lidar_to_map_test_task(void *arg)
{
    (void)arg;

    task_profile_init(&s_profile, "lidar_to_map_test",
                      6144,
                      3,
                      0);   /* core 0 — compute-intensive */

    /* Item = one ray-marched scan point */
    task_data_profile_init(&s_profile.data_profile,
                           "lidar_scan_point_t",
                           sizeof(lidar_scan_point_t),
                           /*heap_allocated=*/ false,
                           /*dma_memory=*/     false,
                           /*passes_ownership=*/false,
                           NULL);

    quadtree_map_init(&s_map, MAP_W, MAP_H, MAP_STEP);

    pose_t robot_pose = { .x = ROBOT_X, .y = ROBOT_Y, .theta = 0.0f };
    uint32_t tick = 0;

    while (1) {
        task_profile_cycle_begin(&s_profile);

        build_synthetic_scan(&s_scan, tick++);

        map_dirty_rect_t dirty = {0};

        PROFILE_CPU_BEGIN(lidar_to_map_call);
        lidar_to_map(&s_map, &s_scan, &robot_pose, MAX_RANGE_MM, RAY_STEP_MM, &dirty);
        uint32_t l2m_us;
        PROFILE_CPU_END(lidar_to_map_call, &l2m_us);

        task_data_profile_update(&s_profile.data_profile, s_scan.count);

        task_profile_cycle_end(&s_profile);

        uint32_t cycle_us = s_profile.cpu_cycles_last / 240;
        if (cycle_us > 50000) {
            ESP_LOGW(TAG, "cycle %" PRIu32 " µs over 50 ms budget (l2m %" PRIu32 " µs)",
                     cycle_us, l2m_us);
        }

        ESP_LOGI(TAG, "pts=%d l2m=%" PRIu32 " µs dirty=(%.0f,%.0f)-(%.0f,%.0f) nodes=%u",
                 N_POINTS, l2m_us,
                 (double)dirty.x_min, (double)dirty.y_min,
                 (double)dirty.x_max, (double)dirty.y_max,
                 s_map.count);

        /* Reset map each cycle to keep pool from saturating */
        qt_free(&s_map);
        quadtree_map_init(&s_map, MAP_W, MAP_H, MAP_STEP);

        vTaskDelay(pdMS_TO_TICKS(100));   /* 10 Hz */
    }
}
