#include "imu_encoder_driver.h"
#include "imu_gyro.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "driver/i2c.h"

#include <math.h>
#include <stdint.h>

static const char *TAG = "imu_encoder_driver";

/* AS5600 I2C config — shares bus with IMU (GPIO 21/22) */
#define AS5600_I2C_PORT     I2C_NUM_0
#define AS5600_ADDR         0x36
#define AS5600_REG_ANGLE_H  0x0E
#define AS5600_REG_ANGLE_L  0x0F
#define AS5600_RESOLUTION   4096        /* 12-bit absolute encoder */
#define AS5600_TIMEOUT_MS   10

#define GEAR_RATIO  10.69f   /* calibrated */
#define WHEEL_CIRCUMFERENCE_M  0.308f   /* measured — diameter 9.8 cm */

/* Internal state */
static imu_encoder_sample_t s_sample = {
    .distance_m   = 0.0f,
    .yaw_rad      = 0.0f,
    .timestamp_ms = 0U
};

static float   s_yaw_integrated  = 0.0f;
static int64_t s_last_time_us    = 0;
static float   s_speed_ms        = 0.0f;
static float   s_prev_distance   = 0.0f;

/* Encoder rollover tracking */
static int32_t s_cumulative_ticks = 0;
static int16_t s_last_angle_raw   = -1;   /* -1 = not yet initialized */

/* Read 12-bit angle from AS5600 over I2C */
static esp_err_t as5600_read_angle(uint16_t *angle_out)
{
    uint8_t reg = AS5600_REG_ANGLE_H;
    uint8_t buf[2] = {0};

    esp_err_t err = i2c_master_write_read_device(
        AS5600_I2C_PORT, AS5600_ADDR,
        &reg, 1,
        buf, 2,
        pdMS_TO_TICKS(AS5600_TIMEOUT_MS)
    );

    if (err != ESP_OK) return err;

    *angle_out = (uint16_t)(((buf[0] & 0x0F) << 8) | buf[1]);
    return ESP_OK;
}

/* ── Init ───────────────────────────────────────────────────────────────── */
esp_err_t imu_encoder_driver_init(void)
{
    /* Init IMU — I2C bus already configured by imu_gyro_init */
    if (!imu_gyro_init()) {
        ESP_LOGW(TAG, "IMU not found — yaw will stay at 0");
    }

    /* Check AS5600 is responding */
    uint16_t angle = 0;
    if (as5600_read_angle(&angle) != ESP_OK) {
        ESP_LOGW(TAG, "AS5600 not found — distance will stay at 0");
    } else {
        s_last_angle_raw = (int16_t)angle;
        ESP_LOGI(TAG, "AS5600 ready, initial angle = %d", angle);
    }

    /* Reset internal state */
    s_sample.distance_m   = 0.0f;
    s_sample.yaw_rad      = 0.0f;
    s_yaw_integrated      = 0.0f;
    s_cumulative_ticks    = 0;
    s_speed_ms            = 0.0f;
    s_prev_distance       = 0.0f;
    s_last_time_us        = esp_timer_get_time();
    s_sample.timestamp_ms = (uint32_t)(s_last_time_us / 1000ULL);

    ESP_LOGI(TAG, "imu_encoder_driver initialized");
    return ESP_OK;
}

/* ── Update — called every 10 ms by task_odometry ──────────────────────── */
esp_err_t imu_encoder_driver_update(void)
{
    int64_t now_us = esp_timer_get_time();
    float   dt_s   = (float)(now_us - s_last_time_us) * 1e-6f;
    s_last_time_us = now_us;

    /* 1. Yaw — integrate gyro Z */
    float gz = imu_gyro_read_z();
    s_yaw_integrated += gz * dt_s;

    while (s_yaw_integrated >  (float)M_PI) s_yaw_integrated -= 2.0f * (float)M_PI;
    while (s_yaw_integrated < -(float)M_PI) s_yaw_integrated += 2.0f * (float)M_PI;

    s_sample.yaw_rad = s_yaw_integrated;

    /* 2. Distance — read AS5600 angle and handle rollover */
    uint16_t raw_angle = 0;
    if (as5600_read_angle(&raw_angle) == ESP_OK && s_last_angle_raw >= 0) {

        int16_t current = (int16_t)raw_angle;
        int16_t delta   = current - s_last_angle_raw;

        /* Rollover: 4095 -> 0 (forward) or 0 -> 4095 (backward) */
        if (delta >  (AS5600_RESOLUTION / 2)) delta -= AS5600_RESOLUTION;
        if (delta < -(AS5600_RESOLUTION / 2)) delta += AS5600_RESOLUTION;

        s_cumulative_ticks += delta;
        s_last_angle_raw    = current;

        /* Apply gear ratio — encoder is on motor shaft not wheel */
        s_sample.distance_m = (float)s_cumulative_ticks
                              / (float)AS5600_RESOLUTION
                              / GEAR_RATIO
                              * WHEEL_CIRCUMFERENCE_M;
    }

    /* 3. Speed — finite difference on distance */
    if (dt_s > 0.0f) {
        float delta_dist = s_sample.distance_m - s_prev_distance;
        s_speed_ms       = delta_dist / dt_s;
        s_prev_distance  = s_sample.distance_m;
    }

    s_sample.timestamp_ms = (uint32_t)(now_us / 1000ULL);
    return ESP_OK;
}

/* ── Getters ────────────────────────────────────────────────────────────── */
imu_encoder_sample_t imu_encoder_driver_get_sample(void)
{
    return s_sample;
}

float imu_encoder_driver_get_distance_m(void)
{
    return s_sample.distance_m;
}

float imu_encoder_driver_get_yaw_rad(void)
{
    return s_sample.yaw_rad;
}

float imu_encoder_driver_get_speed_ms(void)
{
    return s_speed_ms;
}