/**
 * test_uart_echo.c — Wemos UART echo/sink for the ESP32-S3 stress test.
 *
 * Mirrors the production Wemos UART task:
 *   1. drain_pending_packets() via uart_bridge_recv_control() + recv_path()
 *   2. For every control frame received: send odom_t back immediately
 *   3. For every path frame received: send path_ack + path_done
 *
 * REQUIRES HARDWARE: ESP32-S3 running the uart_bridge stress test
 *   Wemos RX GPIO16 ← S3 TX GPIO17
 *   Wemos TX GPIO17 → S3 RX GPIO16
 *   GND shared — 115200 baud
 *
 * ── Per-cycle log (every 500 ticks = 5 s) ────────────────────────────────────
 *   ctrl_rx    control frames received in last window
 *   path_rx    path frames received in last window
 *   odom_tx    odom frames sent back
 *   ctrl_us    avg drain+recv time per tick (µs)
 *   send_us    avg odom send time per tick (µs)
 *
 * ── Memory snapshot (every 5000 ticks = 50 s) ────────────────────────────────
 *   heap_free, watermark, largest block, frag%, stack HWM
 *
 * Rate: 1 kHz tick (1 ms vTaskDelay) — fast enough to drain at 500 Hz stress
 *
 * Board: Wemos D1 R32 (ESP32)
 */

#include "../../profiler.h"
#include "../../../wemos/src/uart_bridge.h"
#include "../../../types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include <string.h>

static const char *TAG = "UART_ECHO";

static task_profile_t s_profile;

task_profile_t *uart_echo_task_get_profile(void) { return &s_profile; }

void uart_echo_test_task(void *arg)
{
    (void)arg;

    task_profile_init(&s_profile, "uart_echo",
                      4096,
                      5,    /* high priority — drain RX before overflow */
                      1);   /* core 1 — UART I/O */

    task_data_profile_init(&s_profile.data_profile,
                           "odom_t",
                           sizeof(odom_t),
                           false, false, false,
                           "s3_uart");

    /* ── Static RAM breakdown ────────────────────────────────────────────── */
    ESP_LOGI(TAG, "=== STATIC RAM ===");
    ESP_LOGI(TAG, "  control_frame_t  %u B", (unsigned)sizeof(control_frame_t));
    ESP_LOGI(TAG, "  path_frame_t     %u B", (unsigned)sizeof(path_frame_t));
    ESP_LOGI(TAG, "  odom_t           %u B", (unsigned)sizeof(odom_t));
    ESP_LOGI(TAG, "  Wemos RX buf:    1024 B → ~%u ctrl pkts before overflow",
             (unsigned)(1024u / (sizeof(control_frame_t) + 5u)));

    uart_bridge_init();
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGI(TAG, "UART echo ready — waiting for S3 stress test");

    /* ── Synthetic odom to send back (static, updated with seq) ─────────── */
    odom_t odom_reply = {
        .linear_disp_mm = 1.0f,
        .yaw_rate_imu   = 0.01f,
        .dt_ms          = 1.0f,
        .seq            = 0u,
    };

    /* ── Per-window counters ─────────────────────────────────────────────── */
    uint32_t window_ctrl_rx  = 0u;
    uint32_t window_path_rx  = 0u;
    uint32_t window_odom_tx  = 0u;
    uint32_t window_ctrl_us_total = 0u;
    uint32_t window_send_us_total = 0u;

    /* ── Session totals ──────────────────────────────────────────────────── */
    uint32_t total_ctrl_rx = 0u;
    uint32_t total_path_rx = 0u;
    uint32_t total_odom_tx = 0u;
    uint32_t total_odom_fail = 0u;

    /* Min/max send latency across session */
    uint32_t send_us_min = UINT32_MAX;
    uint32_t send_us_max = 0u;

    uint32_t tick = 0u;

    while (1) {
        task_profile_cycle_begin(&s_profile);

        /* ── Drain all control frames ─────────────────────────────────────── */
        control_frame_t ctrl;
        PROFILE_CPU_BEGIN(recv_ctrl);
        uint32_t ctrl_this_tick = 0u;
        while (uart_bridge_recv_control(&ctrl)) {
            ctrl_this_tick++;
        }
        uint32_t recv_ctrl_us;
        PROFILE_CPU_END(recv_ctrl, &recv_ctrl_us);

        window_ctrl_rx  += ctrl_this_tick;
        total_ctrl_rx   += ctrl_this_tick;
        window_ctrl_us_total += recv_ctrl_us;

        /* ── Send odom reply for each control received ────────────────────── */
        uint32_t send_this_tick = 0u;
        uint32_t send_us_this_tick = 0u;

        for (uint32_t i = 0u; i < ctrl_this_tick; i++) {
            odom_reply.seq = total_odom_tx;

            PROFILE_CPU_BEGIN(odom_send);
            bool sent = uart_bridge_send_odom(&odom_reply);
            uint32_t send_us;
            PROFILE_CPU_END(odom_send, &send_us);

            if (sent) {
                send_this_tick++;
                total_odom_tx++;
                if (send_us < send_us_min) send_us_min = send_us;
                if (send_us > send_us_max) send_us_max = send_us;
            } else {
                total_odom_fail++;
                if (i == 0u) {   /* only warn on first fail per tick */
                    ESP_LOGW(TAG, "odom send FAILED — total_fail=%" PRIu32,
                             total_odom_fail);
                }
            }
            send_us_this_tick += send_us;
        }

        window_odom_tx        += send_this_tick;
        window_send_us_total  += send_us_this_tick;

        /* ── Drain path frames → send path_ack + path_done ──────────────── */
        path_frame_t pf;
        while (uart_bridge_recv_path(&pf)) {
            window_path_rx++;
            total_path_rx++;
            uart_bridge_send_path_ack(pf.length);
            uart_bridge_send_path_done();
        }

        task_data_profile_update(&s_profile.data_profile, send_this_tick);
        task_profile_cycle_end(&s_profile);

        /* ── Per-window log every 500 ticks (≈5 s at 1kHz) ──────────────── */
        if (tick > 0u && tick % 500u == 0u) {
            float avg_ctrl_us = (window_ctrl_rx > 0u)
                ? (float)window_ctrl_us_total / (float)window_ctrl_rx : 0.0f;
            float avg_send_us = (window_odom_tx > 0u)
                ? (float)window_send_us_total / (float)window_odom_tx : 0.0f;

            /* Inferred recv rate: ctrl_rx / 500 ticks × 1000 ms/s = ctrl_rx × 2 */
            uint32_t ctrl_hz = window_ctrl_rx * 2u;

            ESP_LOGI(TAG,
                     "── tick=%-6"PRIu32" ──────────────────────────────────",
                     tick);
            ESP_LOGI(TAG,
                     "  window ctrl_rx=%-4"PRIu32"  path_rx=%-4"PRIu32
                     "  odom_tx=%-4"PRIu32"  rate=~%"PRIu32"Hz",
                     window_ctrl_rx, window_path_rx, window_odom_tx, ctrl_hz);
            ESP_LOGI(TAG,
                     "  avg recv_us=%.1f  avg send_us=%.1f"
                     "  send_us_range=[%"PRIu32",%"PRIu32"]",
                     (double)avg_ctrl_us, (double)avg_send_us,
                     send_us_min == UINT32_MAX ? 0u : send_us_min,
                     send_us_max);
            ESP_LOGI(TAG,
                     "  totals ctrl_rx=%"PRIu32"  path_rx=%"PRIu32
                     "  odom_tx=%"PRIu32"  odom_fail=%"PRIu32,
                     total_ctrl_rx, total_path_rx,
                     total_odom_tx, total_odom_fail);

            window_ctrl_rx       = 0u;
            window_path_rx       = 0u;
            window_odom_tx       = 0u;
            window_ctrl_us_total = 0u;
            window_send_us_total = 0u;
        }

        /* ── Memory snapshot every 5000 ticks (≈50 s) ───────────────────── */
        if (tick % 5000u == 0u) {
            uint32_t heap_free   = esp_get_free_heap_size();
            uint32_t heap_min    = esp_get_minimum_free_heap_size();
            uint32_t largest_blk = heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT);
            uint32_t frag_pct    = heap_free > 0u
                ? (uint32_t)(100u - largest_blk * 100u / heap_free) : 0u;
            uint32_t stack_free  = uxTaskGetStackHighWaterMark(NULL) * sizeof(StackType_t);

            ESP_LOGI(TAG,
                     "=MEM= heap=%" PRIu32 "B  wm=%" PRIu32 "B"
                     "  blk=%" PRIu32 "B  frag=%" PRIu32 "%%  stack=%" PRIu32 "B",
                     heap_free, heap_min, largest_blk, frag_pct, stack_free);

            if (s_profile.heap_delta_last != 0) {
                ESP_LOGE(TAG, "HEAP DELTA %" PRId32 " B — unexpected alloc in echo loop!",
                         s_profile.heap_delta_last);
            }
        }

        tick++;
        vTaskDelay(pdMS_TO_TICKS(1));   /* 1 kHz drain loop */
    }
}
