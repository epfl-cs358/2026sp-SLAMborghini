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
#define REG_WHO_AM_I    0x00
#define REG_PWR_MGMT_1  0x06
#define REG_PWR_MGMT_2  0x07
#define REG_GYRO_ZOUT_H 0x37
#define REG_BANK_SEL    0x7F

/* Bank 2 */
#define REG_GYRO_CFG1   0x01

#define GYRO_SENS       131.0f
#define GYRO_BIAS_Z     0.00635f
#define POLL_MS         10u

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

void imu_gyro_set_stop_check(bool (*fn)(void))
{
    s_stop_check = fn;
}

bool imu_gyro_init(void)
{
    i2c_config_t cfg = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = IMU_SDA_PIN,
        .scl_io_num       = IMU_SCL_PIN,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        /* 100 kHz — 400 kHz failed on long wires (WHO_AM_I returned 0x00
         * even with correct wiring, signal integrity issue on long runs) */
        .master.clk_speed = 100000,
    };
    esp_err_t r;
    r = i2c_param_config(IMU_I2C_PORT, &cfg);
    ESP_LOGI(TAG, "i2c_param_config: 0x%x", r);

    r = i2c_driver_install(IMU_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
    if (r == ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "I2C driver already installed, reinstalling");
        i2c_driver_delete(IMU_I2C_PORT);
        r = i2c_driver_install(IMU_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
    }
    ESP_LOGI(TAG, "i2c_driver_install: 0x%x", r);
    if (r != ESP_OK) {
        ESP_LOGE(TAG, "I2C init failed — IMU unavailable");
        return false;
    }

    vTaskDelay(pdMS_TO_TICKS(100));

    /* ── I2C bus scan — temporary debug, remove after capteurs confirmed ── */
    ESP_LOGI(TAG, "Scanning I2C bus...");
    for (uint8_t addr = 1; addr < 127; addr++) {
        uint8_t dummy;
        if (i2c_master_read_from_device(IMU_I2C_PORT, addr, &dummy, 1,
                                         pdMS_TO_TICKS(10)) == ESP_OK) {
            ESP_LOGI(TAG, "  Found device at 0x%02X", addr);
        }
    }

    /* ── Wake up ICM-20948 ───────────────────────────────────────────────── */
    imu_select_bank(0);
    imu_write(REG_PWR_MGMT_1, 0x01);
    vTaskDelay(pdMS_TO_TICKS(50));
    imu_write(REG_PWR_MGMT_2, 0x00);

    /* ── Set gyro full-scale to 250 DPS ─────────────────────────────────── */
    imu_select_bank(2);
    imu_write(REG_GYRO_CFG1, 0x01);
    imu_select_bank(0);

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

float imu_gyro_read_z(void)
{
    uint8_t buf[2] = {0};
    if (imu_read(REG_GYRO_ZOUT_H, buf, 2) != ESP_OK) return 0.0f;

    int16_t raw = (int16_t)((buf[0] << 8) | buf[1]);
    float gz_deg_s = (float)raw / GYRO_SENS;
    float gz_rad_s = gz_deg_s * ((float)M_PI / 180.0f);
    return gz_rad_s - GYRO_BIAS_Z;
}

float imu_drive_and_track(float start_heading_rad, uint32_t drive_ms)
{
    float heading = start_heading_rad;
    uint32_t elapsed = 0;

    while (elapsed < drive_ms) {
        if (s_stop_check && s_stop_check()) break;
        uint32_t step = (drive_ms - elapsed < POLL_MS) ? (drive_ms - elapsed) : POLL_MS;
        float gz = imu_gyro_read_z();
        heading += gz * ((float)step / 1000.0f);
        vTaskDelay(pdMS_TO_TICKS(step));
        elapsed += step;
    }

    while (heading >  (float)M_PI) heading -= 2.0f * (float)M_PI;
    while (heading < -(float)M_PI) heading += 2.0f * (float)M_PI;

    return heading;
}