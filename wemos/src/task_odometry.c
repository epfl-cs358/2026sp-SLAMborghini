#include "task_odometry.h"

#include "encoder_ackermann_odometry.h"
#include "imu_encoder_driver.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_timer.h"

#include <stdint.h>

static const char *TAG = "task_odometry";

/* Shared odometry state — written by task, read by main loop */
static encoder_ackermann_odom_t s_odom;

/* Ackermann config — TODO: measure real wheelbase */
static const odom_config_t k_cfg = {
    .wheelbase_m         = 0.26f,   /* distance between front and rear axle */
    .imu_correction_gain = 0.05f,   /* blending weight: 0=encoder only, 1=IMU only */
    .max_delta_dist_m    = 0.08f,   /* max plausible distance per 10 ms cycle */
    .max_yaw_jump_rad    = 0.35f    /* max plausible yaw jump (~20 deg) per cycle */
};

/* Returns latest pose — safe to call from main loop (read only) */
const odom_pose_t *task_odometry_get_pose(void)
{
    return encoder_ackermann_odom_get_pose(&s_odom);
}

/* Steering angle source — replace with real servo feedback when available */
static float get_steering_angle_rad(void)
{
    return 0.0f;
}

void task_odometry(void *pvParameters)
{
    (void)pvParameters;

    encoder_ackermann_odom_init(&s_odom, &k_cfg);

    int64_t last_log_us = 0;

    for (;;) {
        /* Read IMU + encoder */
        imu_encoder_driver_update();

        float distance_m   = imu_encoder_driver_get_distance_m();
        float yaw_rad      = imu_encoder_driver_get_yaw_rad();
        float steering_rad = get_steering_angle_rad();

        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);

        encoder_ackermann_odom_update(&s_odom,
                                      distance_m,
                                      steering_rad,
                                      yaw_rad,
                                      now_ms);

        /* Log pose once per second */
        int64_t now_us = esp_timer_get_time();
        if ((now_us - last_log_us) > 1000000LL) {
            const odom_pose_t *pose = encoder_ackermann_odom_get_pose(&s_odom);
            if (pose != NULL) {
                ESP_LOGI(TAG,
                         "pose: x=%.3f y=%.3f th=%.3f dist=%.3f steer=%.3f",
                         pose->x, pose->y, pose->theta,
                         distance_m, steering_rad);
            }
            last_log_us = now_us;
        }

        vTaskDelay(pdMS_TO_TICKS(10));  /* 100 Hz */
    }
