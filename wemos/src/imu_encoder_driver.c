/**
 * imu_encoder_driver.c
 * Module: AS5600 magnetic encoder + IMU yaw for task_odometry.
 * Board: Wemos D1 R32
 *
 * The AS5600 gives a 12-bit absolute shaft angle [0, 4095] per revolution.
 * Successive reads are differenced and accumulated into cumulative distance.
 * Wrap-around (0 ↔ 4096) is handled by clamping the delta to ±2048 ticks.
 *
 * Calibration knobs (adjust at the top of this file):
 *   WHEEL_DIAMETER_M — measure the driven wheel's outer diameter.
 *   ENCODER_SIGN     — flip to -1.0f if increasing angle = backward.
 */

#include "imu_encoder_driver.h"
#include "imu_gyro.h"
#include "hardware_pins.h"

#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

static const char *ENC_TAG = "encoder";

/* ── AS5600 hardware constants ───────────────────────────────────────────── */
#define AS5600_ADDR         0x36u
#define AS5600_REG_ANGLE_H  0x0Eu   /* high nibble [11:8] then low byte [7:0] */
#define AS5600_TICKS_REV    4096.0f /* ticks per full shaft revolution         */
#define AS5600_TIMEOUT_MS   20u  /* must be ≥ 1 FreeRTOS tick (10 ms at 100 Hz); 5 ms rounded to 0 ticks and timed out instantly */

/* ── Wheel geometry — measure and set these before first run ─────────────── */
#define WHEEL_DIAMETER_M    0.065f    /* 65 mm driven wheel outer diameter     */
#define ENCODER_SIGN        1.0f      /* -1.0f if forward motion reads negative */

/* Motor-to-wheel gear reduction ratio.
 *
 * If the AS5600 is mounted on the MOTOR SHAFT (not the wheel axle), it spins
 * GEAR_RATIO times per wheel revolution.  Set this to the number of motor
 * shaft turns per wheel turn.
 *
 * Calibration procedure (one-time, takes ~2 minutes):
 *   1. Mark the robot's start position on the floor.
 *   2. Flash with USE_REAL_LIDAR=1, let the S3 send one path and drive one
 *      PP sub-cycle (~380 ms).  Read the printed line:
 *        [ODOM-SEND] disp=XXXX.X mm ...
 *   3. Measure the physical distance the robot actually traveled (ruler/tape).
 *   4. Set GEAR_RATIO = reported_mm / actual_mm.
 *
 * Example: log shows 1284 mm, ruler shows 160 mm → GEAR_RATIO = 8.025 → use 8.0
 *
 * If the encoder IS on the wheel axle (no gear reduction visible), set 1.0.
 */
#define GEAR_RATIO          16.0f

#define WHEEL_CIRC_M        ((float)M_PI * WHEEL_DIAMETER_M / GEAR_RATIO)

/* ── Module state ─────────────────────────────────────────────────────────── */
static uint16_t s_prev_raw     = 0u;
static float    s_dist_m       = 0.0f;
static bool     s_initialized  = false;
static uint32_t s_fail_count   = 0u;   /* consecutive AS5600 I2C failures     */

/* ── Internal: read 12-bit angle from AS5600 ─────────────────────────────── */
static bool as5600_read_raw(uint16_t *out)
{
    uint8_t reg = AS5600_REG_ANGLE_H;
    uint8_t buf[2] = {0u};
    esp_err_t r = i2c_master_write_read_device(
        IMU_I2C_PORT, AS5600_ADDR,
        &reg, 1u, buf, 2u,
        pdMS_TO_TICKS(AS5600_TIMEOUT_MS));
    if (r != ESP_OK) return false;
    *out = (uint16_t)(((uint16_t)(buf[0] & 0x0Fu) << 8u) | buf[1]);
    return true;
}

/* ── task_odometry API ───────────────────────────────────────────────────── */

void imu_encoder_driver_update(void)
{
    uint16_t raw = 0u;
    if (!as5600_read_raw(&raw)) {
        s_fail_count++;
        /* Log at 1 Hz (100 Hz task → every 100 failures) */
        if (s_fail_count % 100u == 1u)
            ESP_LOGW(ENC_TAG, "AS5600 I2C read failed (%lu times) — dist frozen at %.3f m",
                     (unsigned long)s_fail_count, (double)s_dist_m);
        return;
    }
    s_fail_count = 0u;

    if (!s_initialized) {
        s_prev_raw    = raw;
        s_initialized = true;
        return;
    }

    /* Signed delta with wrap-around handling (AS5600 is 12-bit absolute). */
    int16_t delta = (int16_t)raw - (int16_t)s_prev_raw;
    if (delta >  2048) delta -= 4096;
    if (delta < -2048) delta += 4096;

    s_dist_m  += ENCODER_SIGN * ((float)delta / AS5600_TICKS_REV) * WHEEL_CIRC_M;
    s_prev_raw = raw;
}

float imu_encoder_driver_get_distance_m(void)
{
    return s_dist_m;
}

float imu_encoder_driver_get_yaw_rad(void)
{
    return imu_gyro_get_heading();
}

/* ── Legacy EKF stubs (not used in bridge-slave mode) ───────────────────── */

void imu_encoder_init(void) { /* I2C init handled by imu_gyro_init() */ }

odom_t imu_encoder_read(void)
{
    odom_t result = {0};
    return result;
}
