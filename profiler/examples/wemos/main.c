/**
 * main.c — Wemos D1 R32 profiler example entry point.
 *
 * Creates the profiled odometry test task and a monitor task.
 * Tests encoder_ackermann_odom_update() with synthetic cumulative distance
 * and sinusoidal steering — no hardware required.
 */

#include "../../profiler.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "PROFILER_WEMOS";

extern void odometry_test_task(void *arg);
extern task_profile_t *odometry_task_get_profile(void);

static void monitor_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(5000));   /* wait for odometry to warm up */
    while (1) {
        ESP_LOGI(TAG, "─── Profiler report ───────────────────────────────");
        task_profile_dump(odometry_task_get_profile());
        ESP_LOGI(TAG, "────────────────────────────────────────────────────");
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "SLAMborghini Wemos profiler examples starting");

    xTaskCreatePinnedToCore(odometry_test_task, "odom_test",
                            4096, NULL, 5, NULL, 1);
    vTaskDelay(pdMS_TO_TICKS(100));

    xTaskCreate(monitor_task, "profiler_mon", 4096, NULL, 1, NULL);
}
