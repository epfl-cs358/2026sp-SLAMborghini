/**
 * test_odometry.c — Profiled encoder_ackermann_odom integration test.
 *
 * Calls encoder_ackermann_odom_update() directly with synthetic inputs.
 * Simulates a 1 Hz sinusoidal steering sweep at constant forward speed.
 * Tests encoder_ackermann_odom_update() latency and heap invariant (must be 0).
 *
 * NOTE: This tests encoder_ackermann_odom directly, NOT task_odometry().
 * task_odometry() requires hardware (encoder ISR, IMU I2C); this task is
 * hardware-independent and can run on the Wemos in isolation.
 *
 * Board: Wemos D1 R32 (ESP32)
 * Rate:  100 Hz (10 ms delay)
 * Budget warning: >1 ms per update
 *
 * Config matches wemos project:
 *   wheelbase_m=0.258, imu_correction_gain=1.0,
 *   max_delta_dist_m=0.08, max_yaw_jump_rad=0.35
 */

#include "../../profiler.h"
#include "../../../wemos/src/encoder_ackermann_odometry.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <math.h>

static const char *TAG = "TEST_ODOM";

/* Wemos project configuration */
static const odom_config_t k_cfg = {
    .wheelbase_m         = 0.258f,
    .imu_correction_gain = 1.0f,
    .max_delta_dist_m    = 0.08f,
    .max_yaw_jump_rad    = 0.35f,
};

static task_profile_t          s_profile;
static encoder_ackermann_odom_t s_odom;

task_profile_t *odometry_task_get_profile(void) { return &s_profile; }

void odometry_test_task(void *arg)
{
    (void)arg;

    task_profile_init(&s_profile, "odometry_test",
                      4096,
                      5,    /* priority — 100 Hz loop, must be responsive */
                      1);   /* core 1 — I/O core on Wemos */

    /* Item = one odom_pose_t produced per update */
    task_data_profile_init(&s_profile.data_profile,
                           "odom_pose_t",
                           sizeof(odom_pose_t),
                           /*heap_allocated=*/ false,
                           /*dma_memory=*/     false,
                           /*passes_ownership=*/false,
                           NULL);

    encoder_ackermann_odom_init(&s_odom, &k_cfg);
    encoder_ackermann_odom_reset(&s_odom, 0.0f, 0.0f, 0.0f);

    /* Simulate cumulative encoder distance: 0.3 m/s forward */
    float cumulative_dist_m = 0.0f;
    float imu_yaw_rad = 0.0f;

    uint32_t tick = 0;

    while (1) {
        task_profile_cycle_begin(&s_profile);

        /* Advance simulated state: 0.3 m/s → 3 mm per 10 ms tick */
        cumulative_dist_m += 0.003f;

        /* Sinusoidal steering: ±15° at 1 Hz, 100 tick period */
        float steering_rad = 0.261799f * sinf((float)tick * 2.0f * 3.14159265f / 100.0f);

        /* Integrate simulated IMU yaw at ackermann geometry rate */
        float ds = 0.003f;
        imu_yaw_rad += (ds / k_cfg.wheelbase_m) * tanf(steering_rad);

        uint32_t timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000);

        PROFILE_CPU_BEGIN(odom_update);
        encoder_ackermann_odom_update(&s_odom,
                                      cumulative_dist_m,
                                      steering_rad,
                                      imu_yaw_rad,
                                      timestamp_ms);
        uint32_t update_us;
        PROFILE_CPU_END(odom_update, &update_us);

        const odom_pose_t *pose = encoder_ackermann_odom_get_pose(&s_odom);
        task_data_profile_update(&s_profile.data_profile, 1);

        task_profile_cycle_end(&s_profile);

        uint32_t cycle_us = s_profile.cpu_cycles_last / 240;
        if (cycle_us > 1000) {
            ESP_LOGW(TAG, "cycle %" PRIu32 " µs over 1 ms budget (update %" PRIu32 " µs)",
                     cycle_us, update_us);
        }

        /* Fully static path — heap delta must always be zero */
        if (s_profile.cycle_count > 5 && s_profile.heap_delta_last != 0) {
            ESP_LOGE(TAG, "heap delta %" PRId32 " B — unexpected alloc in odometry!",
                     s_profile.heap_delta_last);
        }

        if (tick % 100 == 0) {
            ESP_LOGI(TAG, "pose x=%.3f y=%.3f th=%.3f | update=%" PRIu32 " µs",
                     (double)pose->x, (double)pose->y, (double)pose->theta, update_us);
        }

        tick++;
        vTaskDelay(pdMS_TO_TICKS(10));   /* 100 Hz */
    }
}
