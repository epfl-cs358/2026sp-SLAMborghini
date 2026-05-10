/**
 * test_pipeline_full_slam.c — Full ESP32-S3 SLAM pipeline integration test.
 *
 * Runs the complete SLAM loop in a single profiled task:
 *   Stage 1  — LiDAR scan acquisition (real or synthetic)
 *   Stage 2  — lidar_to_map() ray-march into quadtree
 *   Stage 3  — frontier_detector_detect() + frontier_detector_best()
 *   Stage 4  — hybrid_astar_plan() from robot pose to best frontier
 *
 * Per-stage timing is measured with PROFILE_CPU_BEGIN/END.
 * The task_profile captures total cycle time, stack, and heap for the full loop.
 *
 * This task is SELF-CONTAINED — it does NOT share g_shared_map.
 * Do NOT enable both ENABLE_PIPELINE_STAGES and ENABLE_PIPELINE_FULL at once
 * (two quadtree pools would consume ~192 KB of the S3's DRAM simultaneously).
 *
 * Hardware flag:
 *   -DPIPELINE_USE_REAL_LIDAR=1   use lidar_driver_read_scan()  (needs RPLiDAR C1)
 *   -DPIPELINE_USE_REAL_LIDAR=0   generate synthetic 360-pt scan (default)
 *
 * Board: ESP32-S3
 * Rate:  driven by lidar_driver (real) or 2 Hz (synthetic, to show A* budget)
 * Budget warning: >300 ms full cycle
 */

#include "../../profiler.h"
#include "../../../esp32s3/src/quadtree_map.h"
#include "../../../esp32s3/src/lidar_driver.h"
#include "../../../esp32s3/src/lidar_to_map.h"
#include "../../../esp32s3/src/frontier_detector.h"
#include "../../../esp32s3/src/hybrid_astar.h"
#include "../../../types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <math.h>
#include <string.h>

static const char *TAG = "PIPE_FULL_SLAM";

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

/* All large state is static — nothing on the stack */
static task_profile_t s_profile;
static lidar_scan_t   s_scan;      /* ~5.5 KB */
static quadtree_map_t s_map;
static path_t         s_path;      /* ~1 KB */

/* Scan counter — used to vary synthetic scan pattern */
static uint32_t s_tick = 0;

static void build_synthetic_scan(lidar_scan_t *scan, uint32_t tick)
{
    scan->count              = 360;
    scan->scan_start_us      = (uint32_t)esp_timer_get_time();
    scan->rotation_period_us = 100000;
    for (int i = 0; i < 360; i++) {
        scan->points[i].theta_deg = (float)i;
        scan->points[i].r_mm = 1800.0f
            + 400.0f * sinf((float)i * 3.14159265f / 60.0f)
            + 100.0f * sinf((float)(i + tick) * 3.14159265f / 15.0f);
        scan->points[i].intensity = 200;
    }
}

task_profile_t *full_slam_task_get_profile(void) { return &s_profile; }

void full_slam_task(void *arg)
{
    (void)arg;

    task_profile_init(&s_profile, "full_slam",
                      12288, /* generous stack — all 4 modules share this task's stack */
                      4,
                      0);    /* core 0 — compute-intensive */

    /* Item = one waypoint in the planned path (end product of the full pipeline) */
    task_data_profile_init(&s_profile.data_profile,
                           "waypoint_t",
                           sizeof(waypoint_t),
                           /*heap_allocated=*/ false,
                           /*dma_memory=*/     false,
                           /*passes_ownership=*/false,
                           NULL);

    quadtree_map_init(&s_map, MAP_W, MAP_H, MAP_STEP);

    pose_t robot_pose = { .x = ROBOT_X, .y = ROBOT_Y, .theta = 0.0f };

#if PIPELINE_USE_REAL_LIDAR
    lidar_driver_init();
#endif

    uint32_t stage_scan_us, stage_l2m_us, stage_frontier_us, stage_astar_us;

    while (1) {
        task_profile_cycle_begin(&s_profile);

        /* ── Stage 1: LiDAR scan ──────────────────────────────────────────── */
        PROFILE_CPU_BEGIN(stage1_scan);
#if PIPELINE_USE_REAL_LIDAR
        bool scan_ok = lidar_driver_read_scan(&s_scan);
        if (!scan_ok) {
            ESP_LOGW(TAG, "scan failed — skipping cycle");
            task_profile_cycle_end(&s_profile);
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
#else
        build_synthetic_scan(&s_scan, s_tick++);
#endif
        PROFILE_CPU_END(stage1_scan, &stage_scan_us);

        /* ── Stage 2: LiDAR → Map ─────────────────────────────────────────── */
        map_dirty_rect_t dirty = {0};

        PROFILE_CPU_BEGIN(stage2_l2m);
        lidar_to_map(&s_map, &s_scan, &robot_pose, MAX_RANGE_MM, RAY_STEP_MM, &dirty);
        PROFILE_CPU_END(stage2_l2m, &stage_l2m_us);

        /* ── Stage 3: Frontier Detection ──────────────────────────────────── */
        PROFILE_CPU_BEGIN(stage3_frontier);
        frontier_list_t frontiers = frontier_detector_detect(&s_map, &robot_pose);
        frontier_t      best      = frontier_detector_best(&frontiers, &robot_pose);
        PROFILE_CPU_END(stage3_frontier, &stage_frontier_us);

        /* ── Stage 4: Hybrid A* Path Planning ────────────────────────────── */
        bool path_valid = false;

        PROFILE_CPU_BEGIN(stage4_astar);
        if (frontiers.count > 0) {
            s_path = hybrid_astar_plan(&s_map, &robot_pose, &best);
            path_valid = hybrid_astar_is_valid(&s_path);
        }
        PROFILE_CPU_END(stage4_astar, &stage_astar_us);

        /* Update data profile with final pipeline output: waypoint count */
        task_data_profile_update(&s_profile.data_profile,
                                 path_valid ? s_path.length : 0);

        task_profile_cycle_end(&s_profile);

        /* ── Reporting ────────────────────────────────────────────────────── */
        uint32_t total_us = s_profile.cpu_cycles_last / 240;

        if (total_us > 300000) {
            ESP_LOGW(TAG, "full cycle %" PRIu32 " ms over 300 ms budget", total_us / 1000);
        }

        ESP_LOGI(TAG,
                 "─ full SLAM cycle #%" PRIu32 " | total %" PRIu32 " ms ─\n"
                 "  scan:     %" PRIu32 " µs  pts=%u\n"
                 "  l2m:      %" PRIu32 " µs  nodes=%u  dirty=(%.0f,%.0f)-(%.0f,%.0f)\n"
                 "  frontier: %" PRIu32 " µs  count=%u  best=(%.0f,%.0f)\n"
                 "  astar:    %" PRIu32 " µs  wpts=%u  valid=%d",
                 s_profile.cycle_count, total_us / 1000,
                 stage_scan_us, s_scan.count,
                 stage_l2m_us, s_map.count,
                 (double)dirty.x_min, (double)dirty.y_min,
                 (double)dirty.x_max, (double)dirty.y_max,
                 stage_frontier_us, frontiers.count,
                 (double)best.cx, (double)best.cy,
                 stage_astar_us,
                 path_valid ? s_path.length : 0,
                 (int)path_valid);

        /* Reset map when pool is full so subsequent cycles are not starved */
        if (qt_is_pool_full(&s_map)) {
            ESP_LOGW(TAG, "map pool full (%u nodes) — resetting", s_map.count);
            qt_free(&s_map);
            quadtree_map_init(&s_map, MAP_W, MAP_H, MAP_STEP);
        }

#if !PIPELINE_USE_REAL_LIDAR
        vTaskDelay(pdMS_TO_TICKS(500));   /* 2 Hz when synthetic — A* budget is visible */
#endif
    }
}
