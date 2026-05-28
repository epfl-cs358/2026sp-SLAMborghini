/**
 * imu_gyro.c
 * ICM-20948 / MPU-6050 gyro-Z driver for ESP-IDF with auto-detection.
 *
 * Boot sequence:
 *   1. Initialise I2C bus (also needed by the AS5600 encoder at 0x36).
 *   2. Try ICM-20948 at 0x68 — WHO_AM_I reg 0x00, expect 0xEA.
 *   3. If not found, try MPU-6050 at 0x68 — WHO_AM_I reg 0x75, expect 0x68.
 *   4. If neither responds, log a warning and run without IMU (Ackermann yaw).
 *
 * All public functions are no-ops / return 0.0 when s_imu_ok == false, so
 * the rest of the firmware never needs to check the chip type.
 *
 * SDA=GPIO21, SCL=GPIO22 — see hardware_pins.h.
 */

#include "imu_gyro.h"
#include "hardware_pins.h"

#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <math.h>

#define IMU_TIMEOUT_MS  10

/* ── ICM-20948 registers (Bank 0 unless noted) ──────────────────────────── */
#define ICM_ADDR            0x68
#define ICM_REG_WHO_AM_I    0x00    /* expected: 0xEA */
#define ICM_WHO_AM_I_VAL    0xEA
#define ICM_REG_PWR_MGMT_1  0x06
#define ICM_REG_PWR_MGMT_2  0x07
#define ICM_REG_GYRO_ZOUT_H 0x37
#define ICM_REG_BANK_SEL    0x7F
#define ICM_REG_GYRO_CFG1   0x01   /* Bank 2 */

/* ── MPU-6050 registers ─────────────────────────────────────────────────── */
#define MPU_ADDR            0x68
#define MPU_REG_WHO_AM_I    0x75    /* expected: 0x68 */
#define MPU_WHO_AM_I_VAL    0x68
#define MPU_REG_PWR_MGMT_1  0x6B
#define MPU_REG_GYRO_CFG    0x1B
#define MPU_REG_ACCEL_CFG   0x1C
#define MPU_REG_GYRO_ZOUT_H 0x47

/* ── Gyro sensitivity at ±250 DPS: 131 LSB/(°/s) — same for both chips ─── */
#define GYRO_SENS       131.0f

/* ── Detected chip state ─────────────────────────────────────────────────── */
typedef enum { IMU_CHIP_NONE, IMU_CHIP_ICM20948, IMU_CHIP_MPU6050 } imu_chip_t;
static imu_chip_t s_chip       = IMU_CHIP_NONE;
static uint8_t    s_imu_addr   = 0x68;
static uint8_t    s_gyro_z_reg = ICM_REG_GYRO_ZOUT_H;
static bool       s_imu_ok     = false;

/* ── Bias: starts at 0, refined by calibrate_bias() and ZUPT ────────────── */
static float s_bias_z = 0.0f;

/* ── ZUPT parameters ────────────────────────────────────────────────────── */
#define ZUPT_GAIN               0.005f
#define ZUPT_GYRO_THRESH_RAD_S  0.05f

/* ── Integration poll interval ──────────────────────────────────────────── */
#define POLL_MS  10u   /* 100 Hz */

static const char *TAG = "imu_gyro";
static bool (*s_stop_check)(void) = NULL;
static volatile float s_live_heading = 0.0f;

/* ════════════════════════════════════════════════════════════════════════════
 * Low-level I2C helpers
 * ════════════════════════════════════════════════════════════════════════════ */

static esp_err_t imu_write(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_write_to_device(IMU_I2C_PORT, s_imu_addr,
                                      buf, 2, pdMS_TO_TICKS(IMU_TIMEOUT_MS));
}

static esp_err_t imu_read(uint8_t reg, uint8_t *out, size_t len)
{
    return i2c_master_write_read_device(IMU_I2C_PORT, s_imu_addr,
                                        &reg, 1, out, len,
                                        pdMS_TO_TICKS(IMU_TIMEOUT_MS));
}

/* Probe a specific address/register during auto-detection (before s_imu_addr is set). */
static esp_err_t imu_probe_read(uint8_t addr, uint8_t reg, uint8_t *out)
{
    return i2c_master_write_read_device(IMU_I2C_PORT, addr,
                                        &reg, 1, out, 1,
                                        pdMS_TO_TICKS(IMU_TIMEOUT_MS));
}

/* ICM-20948 register bank select — only called when s_chip == IMU_CHIP_ICM20948. */
static void imu_select_bank(uint8_t bank)
{
    imu_write(ICM_REG_BANK_SEL, (uint8_t)(bank << 4));
}


/* ════════════════════════════════════════════════════════════════════════════
 * imu_gyro_set_stop_check
 * ════════════════════════════════════════════════════════════════════════════ */
void imu_gyro_set_stop_check(bool (*fn)(void))
{
    s_stop_check = fn;
}


/* ════════════════════════════════════════════════════════════════════════════
 * imu_gyro_get_heading
 * ════════════════════════════════════════════════════════════════════════════ */
float imu_gyro_get_heading(void)
{
    if (!s_imu_ok) return 0.0f;
    return s_live_heading;
}


/* ════════════════════════════════════════════════════════════════════════════
 * imu_gyro_update
 * ════════════════════════════════════════════════════════════════════════════ */
void imu_gyro_update(float dt_s)
{
    if (!s_imu_ok) return;
    float gz = imu_gyro_read_z();
    s_live_heading += gz * dt_s;
    while (s_live_heading >  (float)M_PI) s_live_heading -= 2.0f * (float)M_PI;
    while (s_live_heading < -(float)M_PI) s_live_heading += 2.0f * (float)M_PI;
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
        /* 100 kHz standard mode — 400 kHz fails silently on long wire runs
         * (ACK never arrives, read buffer stays zeroed).                    */
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

    vTaskDelay(pdMS_TO_TICKS(100));   /* let bus settle after driver start */

    /* ── Auto-detect: try ICM-20948 first ───────────────────────────────── */
    uint8_t who = 0;
    if (imu_probe_read(ICM_ADDR, ICM_REG_WHO_AM_I, &who) == ESP_OK
        && who == ICM_WHO_AM_I_VAL) {

        s_chip       = IMU_CHIP_ICM20948;
        s_imu_addr   = ICM_ADDR;
        s_gyro_z_reg = ICM_REG_GYRO_ZOUT_H;

        imu_select_bank(0);
        imu_write(ICM_REG_PWR_MGMT_1, 0x01);   /* CLKSEL = auto */
        vTaskDelay(pdMS_TO_TICKS(50));
        imu_write(ICM_REG_PWR_MGMT_2, 0x00);   /* enable accel + gyro */

        imu_select_bank(2);
        imu_write(ICM_REG_GYRO_CFG1, 0x01);    /* FS_SEL=00 (250 DPS), DLPF on */
        imu_select_bank(0);

        ESP_LOGI(TAG, "ICM-20948 ready at 0x%02X", s_imu_addr);
        s_imu_ok = true;
        return true;
    }

    /* ── Auto-detect: try MPU-6050 ──────────────────────────────────────── */
    who = 0;
    if (imu_probe_read(MPU_ADDR, MPU_REG_WHO_AM_I, &who) == ESP_OK
        && who == MPU_WHO_AM_I_VAL) {

        s_chip       = IMU_CHIP_MPU6050;
        s_imu_addr   = MPU_ADDR;
        s_gyro_z_reg = MPU_REG_GYRO_ZOUT_H;

        imu_write(MPU_REG_PWR_MGMT_1, 0x00);   /* wake up (exit sleep mode) */
        vTaskDelay(pdMS_TO_TICKS(50));
        imu_write(MPU_REG_GYRO_CFG,  0x00);    /* ±250 DPS */
        imu_write(MPU_REG_ACCEL_CFG, 0x00);    /* ±2g */

        ESP_LOGI(TAG, "MPU-6050 ready at 0x%02X", s_imu_addr);
        s_imu_ok = true;
        return true;
    }

    /* ── No IMU found ────────────────────────────────────────────────────── */
    ESP_LOGW(TAG, "No IMU found (tried ICM-20948 WHO_AM_I=0x%02X, MPU-6050 WHO_AM_I=0x%02X)"
             " — yaw from Ackermann model only", who, who);
    s_imu_ok = false;
    /* Return true: I2C bus is up; AS5600 encoder at 0x36 can still be used. */
    return true;
}


/* ════════════════════════════════════════════════════════════════════════════
 * imu_gyro_read_z
 * ════════════════════════════════════════════════════════════════════════════ */
float imu_gyro_read_z(void)
{
    if (!s_imu_ok) return 0.0f;
    uint8_t buf[2] = {0};
    if (imu_read(s_gyro_z_reg, buf, 2) != ESP_OK) return 0.0f;

    int16_t raw = (int16_t)((buf[0] << 8) | buf[1]);
    float gz_deg_s = (float)raw / GYRO_SENS;
    float gz_rad_s = gz_deg_s * ((float)M_PI / 180.0f);
    return -(gz_rad_s - s_bias_z);   /* negate: chip mounts CW-positive, code expects CCW-positive */
}


/* ════════════════════════════════════════════════════════════════════════════
 * imu_gyro_calibrate_bias
 * ════════════════════════════════════════════════════════════════════════════ */
float imu_gyro_calibrate_bias(int samples)
{
    if (!s_imu_ok) return 0.0f;
    double acc = 0.0;
    int    n   = 0;
    for (int i = 0; i < samples; i++) {
        uint8_t buf[2] = {0};
        if (imu_read(s_gyro_z_reg, buf, 2) == ESP_OK) {
            int16_t raw    = (int16_t)((buf[0] << 8) | buf[1]);
            float gz_deg_s = (float)raw / GYRO_SENS;
            acc += (double)(gz_deg_s * ((float)M_PI / 180.0f));
            n++;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (n > 0) s_bias_z = (float)(acc / n);
    ESP_LOGI(TAG, "Gyro bias calibrated: %.5f rad/s  (%d samples)", (double)s_bias_z, n);
    return s_bias_z;
}


/* ════════════════════════════════════════════════════════════════════════════
 * imu_gyro_zupt_update
 * ════════════════════════════════════════════════════════════════════════════ */
void imu_gyro_zupt_update(void)
{
    if (!s_imu_ok) return;
    uint8_t buf[2] = {0};
    if (imu_read(s_gyro_z_reg, buf, 2) != ESP_OK) return;
    int16_t raw    = (int16_t)((buf[0] << 8) | buf[1]);
    float gz_deg_s = (float)raw / GYRO_SENS;
    float gz_raw   = gz_deg_s * ((float)M_PI / 180.0f);
    float gz_corrected = -(gz_raw - s_bias_z);
    if (fabsf(gz_corrected) > ZUPT_GYRO_THRESH_RAD_S) return;
    s_bias_z += ZUPT_GAIN * (gz_raw - s_bias_z);
}


/* ════════════════════════════════════════════════════════════════════════════
 * imu_drive_and_track
 * ════════════════════════════════════════════════════════════════════════════ */
float imu_drive_and_track(float start_heading_rad, uint32_t drive_ms)
{
    float heading = start_heading_rad;
    s_live_heading = heading;
    uint32_t elapsed = 0;

    while (elapsed < drive_ms) {
        if (s_stop_check && s_stop_check()) break;
        uint32_t step = (drive_ms - elapsed < POLL_MS) ? (drive_ms - elapsed) : POLL_MS;
        float gz = imu_gyro_read_z();
        heading += gz * ((float)step / 1000.0f);
        s_live_heading = heading;
        vTaskDelay(pdMS_TO_TICKS(step));
        elapsed += step;
    }

    while (heading >  (float)M_PI) heading -= 2.0f * (float)M_PI;
    while (heading < -(float)M_PI) heading += 2.0f * (float)M_PI;

    s_live_heading = heading;
    return heading;
}
