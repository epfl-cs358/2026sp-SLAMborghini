/**
 * main.c — ESP32-S3 profiler example entry point.
 *
 * Three test modes — enable at most ONE pipeline group at a time:
 *
 *  UNIT TESTS (hardware-independent by default):
 *   ENABLE_LIDAR_DRIVER    — single-module: lidar_driver (needs RPLiDAR C1)
 *   ENABLE_QUADTREE        — single-module: quadtree stress test
 *   ENABLE_LIDAR_TO_MAP    — single-module: lidar_to_map with synthetic scan
 *   ENABLE_FRONTIER        — single-module: frontier detection on seeded map
 *   ENABLE_HYBRID_ASTAR    — single-module: A* on seeded map
 *   ENABLE_UART_BRIDGE     — single-module: UART send+recv (needs Wemos)
 *   ENABLE_WIFI_DASHBOARD  — single-module: WiFi broadcast (needs AP)
 *
 *  PIPELINE GROUP A — two connected tasks sharing a live map:
 *   ENABLE_PIPELINE_STAGES — Task 1: scan→map  +  Task 2: map→frontier
 *                            Uses one shared quadtree pool (96 KB)
 *
 *  PIPELINE GROUP B — full SLAM loop in one task:
 *   ENABLE_PIPELINE_FULL   — single task: scan→map→frontier→astar
 *                            Uses its own quadtree pool (96 KB)
 *
 *  WARNING: Do NOT enable both ENABLE_PIPELINE_STAGES and ENABLE_PIPELINE_FULL
 *  at the same time — two quadtree pools = 192 KB DRAM simultaneously.
 */

#include "../../profiler.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

/* ── Unit test enable switches ─────────────────────────────────────────────── */
#ifndef ENABLE_LIDAR_DRIVER
#  define ENABLE_LIDAR_DRIVER    0
#endif
#ifndef ENABLE_QUADTREE
#  define ENABLE_QUADTREE        1
#endif
#ifndef ENABLE_LIDAR_TO_MAP
#  define ENABLE_LIDAR_TO_MAP    1
#endif
#ifndef ENABLE_FRONTIER
#  define ENABLE_FRONTIER        1
#endif
#ifndef ENABLE_HYBRID_ASTAR
#  define ENABLE_HYBRID_ASTAR    1
#endif
#ifndef ENABLE_UART_BRIDGE
#  define ENABLE_UART_BRIDGE     0
#endif
#ifndef ENABLE_WIFI_DASHBOARD
#  define ENABLE_WIFI_DASHBOARD  0
#endif

/* ── Pipeline test enable switches ─────────────────────────────────────────── */
#ifndef ENABLE_PIPELINE_STAGES
#  define ENABLE_PIPELINE_STAGES     0   /* Task 1 (scan→map) + Task 2 (map→frontier) */
#endif
#ifndef ENABLE_PIPELINE_FULL
#  define ENABLE_PIPELINE_FULL       0   /* Full SLAM: scan→map→frontier→astar */
#endif
#ifndef ENABLE_PIPELINE_LIDAR_WIFI
#  define ENABLE_PIPELINE_LIDAR_WIFI 0   /* Real LiDAR → map → WiFi dashboard */
#endif

static const char *TAG = "PROFILER_MAIN";

/* ── Unit test declarations ─────────────────────────────────────────────────── */
#if ENABLE_LIDAR_DRIVER
extern void lidar_driver_test_task(void *arg);
extern task_profile_t *lidar_driver_task_get_profile(void);
#endif
#if ENABLE_QUADTREE
extern void quadtree_test_task(void *arg);
extern task_profile_t *quadtree_task_get_profile(void);
#endif
#if ENABLE_LIDAR_TO_MAP
extern void lidar_to_map_test_task(void *arg);
extern task_profile_t *lidar_to_map_task_get_profile(void);
#endif
#if ENABLE_FRONTIER
extern void frontier_test_task(void *arg);
extern task_profile_t *frontier_task_get_profile(void);
#endif
#if ENABLE_HYBRID_ASTAR
extern void hybrid_astar_test_task(void *arg);
extern task_profile_t *hybrid_astar_task_get_profile(void);
#endif
#if ENABLE_UART_BRIDGE
extern void uart_bridge_test_task(void *arg);
extern task_profile_t *uart_bridge_task_get_profile(void);
#endif
#if ENABLE_WIFI_DASHBOARD
extern void wifi_dashboard_test_task(void *arg);
extern task_profile_t *wifi_dashboard_task_get_profile(void);
#endif

/* ── Pipeline declarations ───────────────────────────────────────────────────── */
#if ENABLE_PIPELINE_STAGES
extern void scan_to_map_task(void *arg);
extern task_profile_t *scan_to_map_task_get_profile(void);
extern void pipeline_frontier_task(void *arg);
extern task_profile_t *pipeline_frontier_task_get_profile(void);
#endif
#if ENABLE_PIPELINE_FULL
extern void full_slam_task(void *arg);
extern task_profile_t *full_slam_task_get_profile(void);
#endif
#if ENABLE_PIPELINE_LIDAR_WIFI
extern void lidar_wifi_scan_task(void *arg);
extern task_profile_t *lidar_wifi_scan_task_get_profile(void);
extern void lidar_wifi_dash_task(void *arg);
extern task_profile_t *lidar_wifi_dash_task_get_profile(void);
#endif

/* ── Profile registry ───────────────────────────────────────────────────────── */
#define MAX_PROFILES 12
static task_profile_t *s_profiles[MAX_PROFILES];
static int             s_profile_count = 0;

static void register_profile(task_profile_t *p)
{
    if (s_profile_count < MAX_PROFILES)
        s_profiles[s_profile_count++] = p;
}

/* ── Monitor: dump all profiles every 10 s ──────────────────────────────────── */
static void monitor_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(5000));
    while (1) {
        ESP_LOGI(TAG, "══════════════ Profiler report ══════════════");
        for (int i = 0; i < s_profile_count; i++) {
            task_profile_dump(s_profiles[i]);
        }
        ESP_LOGI(TAG, "═════════════════════════════════════════════");
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "SLAMborghini profiler examples starting");

    /* ── Unit tests ──────────────────────────────────────────────────────── */
#if ENABLE_LIDAR_DRIVER
    xTaskCreatePinnedToCore(lidar_driver_test_task, "lidar_drv_test",
                            8192, NULL, 5, NULL, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
    register_profile(lidar_driver_task_get_profile());
#endif

#if ENABLE_QUADTREE
    xTaskCreatePinnedToCore(quadtree_test_task, "quadtree_test",
                            6144, NULL, 3, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    register_profile(quadtree_task_get_profile());
#endif

#if ENABLE_LIDAR_TO_MAP
    xTaskCreatePinnedToCore(lidar_to_map_test_task, "l2m_test",
                            6144, NULL, 3, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    register_profile(lidar_to_map_task_get_profile());
#endif

#if ENABLE_FRONTIER
    xTaskCreatePinnedToCore(frontier_test_task, "frontier_test",
                            8192, NULL, 3, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    register_profile(frontier_task_get_profile());
#endif

#if ENABLE_HYBRID_ASTAR
    xTaskCreatePinnedToCore(hybrid_astar_test_task, "astar_test",
                            8192, NULL, 3, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    register_profile(hybrid_astar_task_get_profile());
#endif

#if ENABLE_UART_BRIDGE
    xTaskCreatePinnedToCore(uart_bridge_test_task, "uart_test",
                            4096, NULL, 4, NULL, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
    register_profile(uart_bridge_task_get_profile());
#endif

#if ENABLE_WIFI_DASHBOARD
    xTaskCreatePinnedToCore(wifi_dashboard_test_task, "wifi_test",
                            8192, NULL, 2, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    register_profile(wifi_dashboard_task_get_profile());
#endif

    /* ── Pipeline Group A: scan→map (core 1) + map→frontier (core 0) ──── */
#if ENABLE_PIPELINE_STAGES
    xTaskCreatePinnedToCore(scan_to_map_task, "scan_to_map",
                            8192, NULL, 5, NULL, 1);
    vTaskDelay(pdMS_TO_TICKS(200));   /* let scan_to_map init the mutex before frontier starts */
    register_profile(scan_to_map_task_get_profile());

    xTaskCreatePinnedToCore(pipeline_frontier_task, "pipe_frontier",
                            8192, NULL, 3, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    register_profile(pipeline_frontier_task_get_profile());
#endif

    /* ── Pipeline Group B: full SLAM in one task ─────────────────────── */
#if ENABLE_PIPELINE_FULL
    xTaskCreatePinnedToCore(full_slam_task, "full_slam",
                            12288, NULL, 4, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    register_profile(full_slam_task_get_profile());
#endif

    /* ── Pipeline Group C: real LiDAR → map → WiFi dashboard ────────── */
#if ENABLE_PIPELINE_LIDAR_WIFI
    xTaskCreatePinnedToCore(lidar_wifi_scan_task, "lw_scan",
                            10240, NULL, 5, NULL, 1);
    vTaskDelay(pdMS_TO_TICKS(200));   /* let scan_task init WiFi+LiDAR first */
    register_profile(lidar_wifi_scan_task_get_profile());

    xTaskCreatePinnedToCore(lidar_wifi_dash_task, "lw_dash",
                            4096, NULL, 2, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    register_profile(lidar_wifi_dash_task_get_profile());
#endif

    xTaskCreate(monitor_task, "profiler_mon", 4096, NULL, 1, NULL);
}
