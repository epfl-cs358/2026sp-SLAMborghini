/**
 * test_wifi_dashboard.c — Profiled WiFi dashboard test.
 *
 * Calls wifi_dashboard_init() then exercises broadcast_state(),
 * broadcast_scan(), and monitors internal queue backpressure each cycle.
 *
 * REQUIRES HARDWARE: WiFi AP credentials must be set in sdkconfig.
 * Without a real AP, wifi_dashboard_init() will fail after ~10 s timeout.
 *
 * Board: ESP32-S3
 * Rate:  10 Hz (100 ms delay)
 * Queue depth warning: >12 (internal 24-slot drop queue is at >50% capacity)
 *
 * lidar_scan_t (~5.5 KB) and quadtree_map_t (96 KB pool) declared static.
 */

#include "../../profiler.h"
#include "../../../esp32s3/src/wifi_dashboard.h"
#include "../../../esp32s3/src/quadtree_map.h"
#include "../../../types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <math.h>
#include <string.h>

static const char *TAG = "TEST_WIFI";

/* WiFi credentials — override at build time via -DTEST_WIFI_SSID and -DTEST_WIFI_PASS */
#ifndef TEST_WIFI_SSID
#  define TEST_WIFI_SSID "SLAMborghini_AP"
#endif
#ifndef TEST_WIFI_PASS
#  define TEST_WIFI_PASS "slamcar2026"
#endif

#define MAP_W    10000.0f
#define MAP_H    10000.0f
#define MAP_STEP   200.0f
#define ROBOT_X   5000.0f
#define ROBOT_Y   5000.0f
#define N_POINTS    360

static task_profile_t s_profile;
static lidar_scan_t   s_scan;     /* ~5.5 KB — static */
static quadtree_map_t s_map;

static void build_synthetic_scan(lidar_scan_t *scan, uint32_t tick)
{
    scan->count = N_POINTS;
    scan->scan_start_us = (uint32_t)esp_timer_get_time();
    scan->rotation_period_us = 100000;
    for (int i = 0; i < N_POINTS; i++) {
        scan->points[i].theta_deg = (float)i;
        scan->points[i].r_mm      = 2000.0f + 300.0f * sinf((float)(i + tick) * 0.04f);
        scan->points[i].intensity = 180;
    }
}

task_profile_t *wifi_dashboard_task_get_profile(void) { return &s_profile; }

void wifi_dashboard_test_task(void *arg)
{
    (void)arg;

    task_profile_init(&s_profile, "wifi_dashboard_test",
                      8192,
                      2,    /* priority — lower than SLAM, higher than idle */
                      0);   /* core 0 — WiFi stack uses core 0 */

    /* Item = one dashboard broadcast message queued */
    task_data_profile_init(&s_profile.data_profile,
                           "dashboard_msg",
                           sizeof(pose_t),   /* representative frame size */
                           /*heap_allocated=*/ false,
                           /*dma_memory=*/     false,
                           /*passes_ownership=*/false,
                           NULL);

    quadtree_map_init(&s_map, MAP_W, MAP_H, MAP_STEP);

    /* Seed a minimal map so the dashboard has something to render */
    pose_t robot_pose = { .x = ROBOT_X, .y = ROBOT_Y, .theta = 0.0f };
    for (float dx = -800.0f; dx <= 800.0f; dx += 200.0f)
        for (float dy = -800.0f; dy <= 800.0f; dy += 200.0f)
            quadtree_map_insert(&s_map, ROBOT_X + dx, ROBOT_Y + dy, CLASS_FREE);

    wifi_dashboard_init(TEST_WIFI_SSID, TEST_WIFI_PASS);

    uint32_t tick = 0;

    while (1) {
        task_profile_cycle_begin(&s_profile);

        build_synthetic_scan(&s_scan, tick);

        float sim_fx = ROBOT_X + 1200.0f * cosf((float)tick * 0.1f);
        float sim_fy = ROBOT_Y + 1200.0f * sinf((float)tick * 0.1f);

        PROFILE_CPU_BEGIN(broadcast);
        wifi_dashboard_broadcast_state(&robot_pose, sim_fx, sim_fy,
                                       /*has_frontier=*/true,
                                       /*scan_idx=*/(uint16_t)(tick % 65536));
        wifi_dashboard_broadcast_scan(&s_scan, &robot_pose);
        uint32_t broadcast_us;
        PROFILE_CPU_END(broadcast, &broadcast_us);

        uint8_t qdepth = wifi_dashboard_queue_depth();

        /* 2 messages queued per cycle (state + scan) */
        task_data_profile_update(&s_profile.data_profile, 2);

        task_profile_cycle_end(&s_profile);

        if (qdepth > 12) {
            ESP_LOGW(TAG, "dashboard queue depth=%u — backpressure! "
                     "dash_task lagging behind", qdepth);
        }

        ESP_LOGI(TAG, "qdepth=%u broadcast=%" PRIu32 " µs tick=%" PRIu32,
                 qdepth, broadcast_us, tick);

        tick++;
        vTaskDelay(pdMS_TO_TICKS(100));   /* 10 Hz */
    }
}
