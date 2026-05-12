/**
 * test_lidar_to_map.c — Profiled lidar_to_map integration test with real LiDAR.
 *
 * Reads a full 360-degree scan from the RPLiDAR C1, then calls lidar_to_map()
 * and measures only the ray-marching occupancy update time.
 *
 * REQUIRES HARDWARE: RPLiDAR C1 on UART (GPIO14 RX, GPIO13 TX, 460800 baud).
 * Without hardware, lidar_driver_read_scan() will block for up to 5000 ms
 * then return false — the profile will report a 5000 ms cycle and 0 items.
 *
 * Board: ESP32-S3
 * Rate:  driven by RPLiDAR motor RPM (~7–10 Hz); lidar_driver_read_scan blocks.
 * Budget warning: >50 ms per lidar_to_map() call.
 *
 * lidar_scan_t (~5.5 KB) and QuadTreeMap pool (96 KB) declared static.
 * Map is re-initialised each cycle to prevent pool saturation.
 */

#include "../../profiler.h"
#include "../../../esp32s3/src/lidar_driver.h"
#include "../../../esp32s3/src/quadtree_map.h"
#include "../../../esp32s3/src/lidar_to_map.h"
#include "../../../types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "TEST_L2M";

#define MAP_W        10000.0f
#define MAP_H        10000.0f
#define MAP_STEP       200.0f
#define MAX_RANGE_MM  4000.0f
#define RAY_STEP_MM    100.0f
#define ROBOT_X       5000.0f
#define ROBOT_Y       5000.0f

static task_profile_t s_profile;
static lidar_scan_t   s_scan;   /* ~5.5 KB — static to avoid stack overflow */
static quadtree_map_t s_map;

task_profile_t *lidar_to_map_task_get_profile(void) { return &s_profile; }

void lidar_to_map_test_task(void *arg)
{
    (void)arg;

    task_profile_init(&s_profile, "lidar_to_map_test",
                      8192,
                      5,     /* priority — high to keep pace with sensor */
                      1);    /* core 1 — I/O tasks on core 1 */

    /* Item = one ray-marched scan point */
    task_data_profile_init(&s_profile.data_profile,
                           "lidar_scan_point_t",
                           sizeof(lidar_scan_point_t),
                           /*heap_allocated=*/ false,
                           /*dma_memory=*/     false,
                           /*passes_ownership=*/false,
                           NULL);

    lidar_driver_init();
    quadtree_map_init(&s_map, MAP_W, MAP_H, MAP_STEP);

    pose_t robot_pose = { .x = ROBOT_X, .y = ROBOT_Y, .theta = 0.0f };

    while (1) {
        task_profile_cycle_begin(&s_profile);

        /* Blocking read — ~132 ms per scan; not included in the l2m timing below */
        bool ok = lidar_driver_read_scan(&s_scan);

        if (!ok) {
            task_data_profile_update(&s_profile.data_profile, 0);
            task_profile_cycle_end(&s_profile);
            ESP_LOGW(TAG, "lidar_driver_read_scan() failed — no hardware?");
            continue;
        }

        map_dirty_rect_t dirty = {0};

        PROFILE_CPU_BEGIN(lidar_to_map_call);
        lidar_to_map(&s_map, &s_scan, &robot_pose, MAX_RANGE_MM, RAY_STEP_MM, &dirty);
        uint32_t l2m_us;
        PROFILE_CPU_END(lidar_to_map_call, &l2m_us);

        task_data_profile_update(&s_profile.data_profile, s_scan.count);
        task_profile_cycle_end(&s_profile);

        if (l2m_us > 50000) {
            ESP_LOGW(TAG, "l2m %" PRIu32 " µs over 50 ms budget", l2m_us);
        }

        ESP_LOGI(TAG, "pts=%u l2m=%" PRIu32 " µs (~%.1f Hz) dirty=(%.0f,%.0f)-(%.0f,%.0f) nodes=%u",
                 s_scan.count, l2m_us,
                 l2m_us > 0 ? 1e6f / (float)l2m_us : 0.0f,
                 (double)dirty.x_min, (double)dirty.y_min,
                 (double)dirty.x_max, (double)dirty.y_max,
                 s_map.count);

        /* Reset map each cycle to keep pool from saturating */
        qt_free(&s_map);
        quadtree_map_init(&s_map, MAP_W, MAP_H, MAP_STEP);

        /* No explicit delay — lidar_driver_read_scan() is blocking */
    }
}
