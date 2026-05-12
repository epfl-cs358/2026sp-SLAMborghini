/**
 * main.c — Wemos D1 R32 profiler example entry point.
 *
 * Enable at most one test group at a time:
 *
 *  ENABLE_ODOMETRY — real-hardware odometry profiling (ICM-20948 + AS5600)
 *  ENABLE_UART_ECHO — UART echo/sink for ESP32-S3 stress test
 *                     (pair with ENABLE_UART_BRIDGE on S3 side)
 */

#include "../../profiler.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "PROFILER_WEMOS";

#ifndef ENABLE_ODOMETRY
#  define ENABLE_ODOMETRY    1
#endif
#ifndef ENABLE_UART_ECHO
#  define ENABLE_UART_ECHO   0
#endif

/* ── Task declarations ──────────────────────────────────────────────────── */
#if ENABLE_ODOMETRY
extern void odometry_test_task(void *arg);
extern task_profile_t *odometry_task_get_profile(void);
#endif

#if ENABLE_UART_ECHO
extern void uart_echo_test_task(void *arg);
extern task_profile_t *uart_echo_task_get_profile(void);
#endif

/* ── Profile registry ───────────────────────────────────────────────────── */
#define MAX_PROFILES 4
static task_profile_t *s_profiles[MAX_PROFILES];
static int             s_profile_count = 0;

static void register_profile(task_profile_t *p)
{
    if (s_profile_count < MAX_PROFILES)
        s_profiles[s_profile_count++] = p;
}

static void monitor_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(5000));
    while (1) {
        ESP_LOGI(TAG, "─── Profiler report ───────────────────────────────");
        for (int i = 0; i < s_profile_count; i++)
            task_profile_dump(s_profiles[i]);
        ESP_LOGI(TAG, "────────────────────────────────────────────────────");
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "SLAMborghini Wemos profiler examples starting");

#if ENABLE_ODOMETRY
    xTaskCreatePinnedToCore(odometry_test_task, "odom_test",
                            4096, NULL, 5, NULL, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
    register_profile(odometry_task_get_profile());
#endif

#if ENABLE_UART_ECHO
    xTaskCreatePinnedToCore(uart_echo_test_task, "uart_echo",
                            4096, NULL, 5, NULL, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
    register_profile(uart_echo_task_get_profile());
#endif

    xTaskCreate(monitor_task, "profiler_mon", 4096, NULL, 1, NULL);
}
