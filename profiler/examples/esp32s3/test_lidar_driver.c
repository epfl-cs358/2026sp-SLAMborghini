/**
 * test_lidar_driver.c — Profiled RPLiDAR C1 driver test.
 *
 * Calls the real lidar_driver module and measures per-scan timing.
 * REQUIRES HARDWARE: RPLiDAR C1 on UART (GPIO14 RX, GPIO13 TX, 460800 baud).
 * Without hardware, lidar_driver_read_scan() will block for up to 5000 ms
 * then return false — the profile will report an 5000 ms cycle and 0 items.
 *
 * Board: ESP32-S3
 * Rate:  ~10 Hz nominal (driven by RPLiDAR motor RPM; lidar_driver_read_scan blocks)
 * Budget warning: >200 ms per scan (RPLiDAR C1 target is ~100 ms/scan)
 *
 * lidar_scan_t is ~5.5 KB — declared static to avoid stack overflow.
 */

#include "../../profiler.h"
#include "../../../esp32s3/src/lidar_driver.h"
#include "../../../types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "TEST_LIDAR";

static task_profile_t s_profile;
static lidar_scan_t   s_scan;   /* ~5.5 KB — static to avoid stack overflow */

task_profile_t *lidar_driver_task_get_profile(void) { return &s_profile; }

void lidar_driver_test_task(void *arg)
{
    (void)arg;

    task_profile_init(&s_profile, "lidar_driver_test",
                      8192,  /* stack bytes */
                      5,     /* priority — high to keep pace with sensor */
                      1);    /* core 1 — I/O tasks on core 1 */

    /* Item = one lidar_scan_point_t (12 bytes); count = points per scan */
    task_data_profile_init(&s_profile.data_profile,
                           "lidar_scan_point_t",
                           sizeof(lidar_scan_point_t),
                           /*heap_allocated=*/ false,  /* s_scan is static */
                           /*dma_memory=*/     false,
                           /*passes_ownership=*/false,
                           NULL);

    lidar_driver_init();

    while (1) {
        task_profile_cycle_begin(&s_profile);

        PROFILE_CPU_BEGIN(read_scan);
        bool ok = lidar_driver_read_scan(&s_scan);
        uint32_t read_us;
        PROFILE_CPU_END(read_scan, &read_us);

        if (ok) {
            task_data_profile_update(&s_profile.data_profile, s_scan.count);
        } else {
            task_data_profile_update(&s_profile.data_profile, 0);
            ESP_LOGW(TAG, "lidar_driver_read_scan() failed — no hardware?");
        }

        task_profile_cycle_end(&s_profile);

        uint32_t cycle_us = s_profile.cpu_cycles_last / 240;
        if (cycle_us > 200000) {
            ESP_LOGW(TAG, "cycle %" PRIu32 " µs over 200 ms budget", cycle_us);
        }

        if (ok) {
            ESP_LOGI(TAG, "scan pts=%u read=%" PRIu32 " µs rotation=%" PRIu32 " µs",
                     s_scan.count, read_us, s_scan.rotation_period_us);
        }

        /* No explicit delay — lidar_driver_read_scan() is blocking */
    }
}
