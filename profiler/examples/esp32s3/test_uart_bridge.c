/**
 * test_uart_bridge.c — Profiled UART bridge (ESP32-S3 side) test.
 *
 * Sends control frames to Wemos and tries to receive odometry frames.
 * REQUIRES HARDWARE: Wemos D1 R32 connected on:
 *   S3 TX GPIO17 → Wemos RX GPIO16
 *   S3 RX GPIO16 ← Wemos TX GPIO17
 *   115200 baud
 *
 * Without hardware, uart_bridge_recv_odom() always returns false and
 * uart_bridge_send_control() returns false (UART not ready).
 *
 * Board: ESP32-S3
 * Rate:  10 Hz (100 ms delay)
 * Budget warning: none — UART send is fire-and-forget (<1 ms); recv is non-blocking
 */

#include "../../profiler.h"
#include "../../../esp32s3/src/uart_bridge.h"
#include "../../../types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "TEST_UART";

static task_profile_t s_profile;
static uint32_t       s_send_count;
static uint32_t       s_recv_count;

task_profile_t *uart_bridge_task_get_profile(void) { return &s_profile; }

void uart_bridge_test_task(void *arg)
{
    (void)arg;

    task_profile_init(&s_profile, "uart_bridge_test",
                      4096,
                      4,    /* priority — above SLAM tasks, below LiDAR */
                      1);   /* core 1 — UART I/O on core 1 */

    /* Item = one control_frame_t sent + one odom_t received (count separately below) */
    task_data_profile_init(&s_profile.data_profile,
                           "control_frame_t",
                           sizeof(control_frame_t),
                           /*heap_allocated=*/ false,
                           /*dma_memory=*/     false,
                           /*passes_ownership=*/false,
                           "wemos_uart");

    uart_bridge_init();

    control_frame_t ctrl = {
        .tx       = 0.5f,   /* m/s forward */
        .ty       = 0.0f,
        .t_heading = 0.0f,
        .t_speed  = 0.5f,
    };
    odom_t odom = {0};

    while (1) {
        task_profile_cycle_begin(&s_profile);

        PROFILE_CPU_BEGIN(send_ctrl);
        bool sent = uart_bridge_send_control(&ctrl);
        uint32_t send_us;
        PROFILE_CPU_END(send_ctrl, &send_us);

        if (sent) s_send_count++;

        PROFILE_CPU_BEGIN(recv_odom);
        bool got_odom = uart_bridge_recv_odom(&odom);
        uint32_t recv_us;
        PROFILE_CPU_END(recv_odom, &recv_us);

        if (got_odom) s_recv_count++;

        /* Count frames processed this cycle (0 or 1 of each) */
        task_data_profile_update(&s_profile.data_profile,
                                 (uint32_t)sent + (uint32_t)got_odom);

        task_profile_cycle_end(&s_profile);

        ESP_LOGI(TAG, "sent=%d recv=%d (total s=%" PRIu32 " r=%" PRIu32 ") "
                 "send=%" PRIu32 " µs recv=%" PRIu32 " µs",
                 (int)sent, (int)got_odom,
                 s_send_count, s_recv_count,
                 send_us, recv_us);

        if (!sent) {
            ESP_LOGW(TAG, "uart_bridge_send_control() failed — no Wemos hardware?");
        }

        vTaskDelay(pdMS_TO_TICKS(100));   /* 10 Hz */
    }
}
