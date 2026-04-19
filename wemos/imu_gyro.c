/**
 * imu_gyro.c
 * Minimal ICM-20948 gyro-Z driver for ESP-IDF.
 *
 * Wiring (from 2025fa-SLAMurai icm_control/config.hpp):
 *   SDA = GPIO 21,  SCL = GPIO 22,  I2C addr = 0x68
 *
 * Only gyro Z is used — we integrate it during each drive to get the
 * actual heading change and feed it back into dead_reckon_pose().
 */

#include "imu_gyro.h"

#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <math.h>

/* ── I2C config (from SLAMurai) ─────────────────────────────────────────── */
#define IMU_I2C_PORT    I2C_NUM_0
#define IMU_SDA_PIN     21
#define IMU_SCL_PIN     22
#define IMU_ADDR        0x68
#define IMU_TIMEOUT_MS  10

/* ── ICM-20948 registers (User Bank 0 unless noted) ─────────────────────── */
#define REG_WHO_AM_I    0x00    /* expected: 0xEA */
#define REG_PWR_MGMT_1  0x06
#define REG_PWR_MGMT_2  0x07
#define REG_GYRO_ZOUT_H 0x37
#define REG_BANK_SEL    0x7F

/* Bank 2 */
#define REG_GYRO_CFG1   0x01   /* GYRO_CONFIG_1 */

/* ── Gyro sensitivity at 250 DPS: 131 LSB / (°/s) ──────────────────────── */
#define GYRO_SENS       131.0f

/* ── Bias from SLAMurai calibration (rad/s) ─────────────────────────────── */
#define GYRO_BIAS_Z     0.00858f

/* ── Integration poll interval ──────────────────────────────────────────── */
#define POLL_MS         10u    /* 100 Hz */

static const char *TAG = "imu_gyro";
static bool (*s_stop_check)(void) = NULL;

/* ── Low-level I2C helpers ───────────────────────────────────────────────── */
static esp_err_t imu_write(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_write_to_device(IMU_I2C_PORT, IMU_ADDR,
                                      buf, 2, pdMS_TO_TICKS(IMU_TIMEOUT_MS));
}

static esp_err_t imu_read(uint8_t reg, uint8_t *out, size_t len)
{
    return i2c_master_write_read_device(IMU_I2C_PORT, IMU_ADDR,
                                        &reg, 1, out, len,
                                        pdMS_TO_TICKS(IMU_TIMEOUT_MS));
}

static void imu_select_bank(uint8_t bank)
{
    imu_write(REG_BANK_SEL, (uint8_t)(bank << 4));
}


/* ════════════════════════════════════════════════════════════════════════════
 * imu_gyro_set_stop_check
 * ════════════════════════════════════════════════════════════════════════════ */
void imu_gyro_set_stop_check(bool (*fn)(void))
{
    s_stop_check = fn;
}


/* ════════════════════════════════════════════════════════════════════════════
 * imu_gyro_init
 * ════════════════════════════════════════════════════════════════════════════ */
bool imu_gyro_init(void)
{
    /* ── I2C master init ─────────────────────────────────────────────────── */
    i2c_config_t cfg = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = IMU_SDA_PIN,
        .scl_io_num       = IMU_SCL_PIN,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000,
    };
    i2c_param_config(IMU_I2C_PORT, &cfg);
    i2c_driver_install(IMU_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);

    /* ── Wake up ICM-20948, auto-select clock ────────────────────────────── */
    imu_select_bank(0);
    imu_write(REG_PWR_MGMT_1, 0x01);   /* CLKSEL = auto */
    vTaskDelay(pdMS_TO_TICKS(50));
    imu_write(REG_PWR_MGMT_2, 0x00);   /* enable accel + gyro */

    /* ── Set gyro full-scale to 250 DPS (matches SLAMurai) ──────────────── */
    imu_select_bank(2);
    imu_write(REG_GYRO_CFG1, 0x01);    /* FS_SEL=00 (250 DPS), DLPF on */
    imu_select_bank(0);

    /* ── I2C bus scan — logs every responding address ───────────────────── */
    ESP_LOGI(TAG, "Scanning I2C bus...");
    for (uint8_t addr = 1; addr < 127; addr++) {
        uint8_t dummy;
        if (i2c_master_read_from_device(IMU_I2C_PORT, addr, &dummy, 1,
                                         pdMS_TO_TICKS(IMU_TIMEOUT_MS)) == ESP_OK) {
            ESP_LOGI(TAG, "  Found device at 0x%02X", addr);
        }
    }

    /* ── Verify WHO_AM_I ─────────────────────────────────────────────────── */
    uint8_t who = 0;
    imu_read(REG_WHO_AM_I, &who, 1);
    if (who != 0xEA) {
        ESP_LOGW(TAG, "ICM-20948 not found (WHO_AM_I=0x%02X, expected 0xEA)", who);
        return false;
    }
    ESP_LOGI(TAG, "ICM-20948 ready");
    return true;
}


/* ════════════════════════════════════════════════════════════════════════════
 * imu_gyro_read_z
 * ════════════════════════════════════════════════════════════════════════════ */
float imu_gyro_read_z(void)
{
    uint8_t buf[2] = {0};
    if (imu_read(REG_GYRO_ZOUT_H, buf, 2) != ESP_OK) return 0.0f;

    int16_t raw = (int16_t)((buf[0] << 8) | buf[1]);
    float gz_deg_s = (float)raw / GYRO_SENS;
    float gz_rad_s = gz_deg_s * ((float)M_PI / 180.0f);
    return gz_rad_s - GYRO_BIAS_Z;
}


/* ════════════════════════════════════════════════════════════════════════════
 * imu_drive_and_track
 * Drives forward for drive_ms while integrating gyro Z at POLL_MS intervals.
 * Called by drive_for_cmd() — motors must already have servo set before this.
 * Returns the actual heading after the drive.
 * ════════════════════════════════════════════════════════════════════════════ */
float imu_drive_and_track(float start_heading_rad, uint32_t drive_ms)
{
    float heading = start_heading_rad;
    uint32_t elapsed = 0;

    while (elapsed < drive_ms) {
        if (s_stop_check && s_stop_check()) break;   /* emergency stop */
        uint32_t step = (drive_ms - elapsed < POLL_MS) ? (drive_ms - elapsed) : POLL_MS;
        float gz = imu_gyro_read_z();
        heading += gz * ((float)step / 1000.0f);
        vTaskDelay(pdMS_TO_TICKS(step));
        elapsed += step;
    }

    /* Normalise to (-π, π] */
    while (heading >  (float)M_PI) heading -= 2.0f * (float)M_PI;
    while (heading < -(float)M_PI) heading += 2.0f * (float)M_PI;

    return heading;
}
