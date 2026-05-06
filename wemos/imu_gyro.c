/**
 * imu_gyro.c
 * Minimal ICM-20948 gyro-Z driver for ESP-IDF.
 *
 * In the two-board layout the ICM-20948 is wired to the ESP32-S3 — see
 * hardware_pins.h for the current pin assignments (SDA=GPIO8, SCL=GPIO9).
 */

#include "imu_gyro.h"
#include "hardware_pins.h"

#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <math.h>

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

/* ── Bias: warm-start from SLAMurai calibration; refined at runtime ─────── */
static float s_bias_z = 0.00635f;

/* ── ZUPT EMA gain: ~200 stationary calls to converge 50 % ──────────────── */
#define ZUPT_GAIN 0.005f

/* ── Integration poll interval ──────────────────────────────────────────── */
#define POLL_MS         10u    /* 100 Hz */

static const char *TAG = "imu_gyro";
static bool  (*s_stop_check)(void) = NULL;
static volatile float s_live_heading = 0.0f;

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
 * imu_gyro_get_heading
 * ════════════════════════════════════════════════════════════════════════════ */
float imu_gyro_get_heading(void)
{
    return s_live_heading;
}


/* ════════════════════════════════════════════════════════════════════════════
 * imu_gyro_update
 * Integrate gyro Z into s_live_heading over dt_s seconds.
 * Called every 10 ms by task_odometry so heading stays current between drives.
 * ════════════════════════════════════════════════════════════════════════════ */
void imu_gyro_update(float dt_s)
{
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
        /* I2C clock speed: 400 kHz failed silently on the Wemos (WHO_AM_I
         * returned 0x00, I2C scan found nothing) even though multimeter
         * confirmed all four wires were continuous.  Root cause: the soldered
         * wire runs are long enough that 400 kHz edge times violate the I2C
         * spec — the ESP-IDF driver returns ESP_OK but the ACK never arrives,
         * leaving the read buffer zeroed.  Diagnosed by adding return-value
         * logging to i2c_param_config / i2c_driver_install (both 0x0 = OK),
         * which ruled out a driver init failure and pointed to a signal-
         * integrity problem.  Fixed by dropping to 100 kHz standard mode,
         * which passes comfortably on the same wires (scan finds 0x68,
         * WHO_AM_I = 0xEA). */
        .master.clk_speed = 100000,
    };
    esp_err_t r;
    r = i2c_param_config(IMU_I2C_PORT, &cfg);
    ESP_LOGI(TAG, "i2c_param_config: 0x%x", r);

    r = i2c_driver_install(IMU_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
    if (r == ESP_ERR_INVALID_STATE) {
        /* Driver already installed (e.g. previous boot) — reinstall */
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

    /* ── Wake up ICM-20948, auto-select clock ────────────────────────────── */
    imu_select_bank(0);
    imu_write(REG_PWR_MGMT_1, 0x01);   /* CLKSEL = auto */
    vTaskDelay(pdMS_TO_TICKS(50));
    imu_write(REG_PWR_MGMT_2, 0x00);   /* enable accel + gyro */

    /* ── Set gyro full-scale to 250 DPS (matches SLAMurai) ──────────────── */
    imu_select_bank(2);
    imu_write(REG_GYRO_CFG1, 0x01);    /* FS_SEL=00 (250 DPS), DLPF on */
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
    return gz_rad_s - s_bias_z;
}


/* ════════════════════════════════════════════════════════════════════════════
 * imu_gyro_calibrate_bias
 * Average raw gyro Z over `samples` ticks (10 ms each) to set s_bias_z.
 * ════════════════════════════════════════════════════════════════════════════ */
float imu_gyro_calibrate_bias(int samples)
{
    double acc = 0.0;
    int    n   = 0;
    for (int i = 0; i < samples; i++) {
        uint8_t buf[2] = {0};
        if (imu_read(REG_GYRO_ZOUT_H, buf, 2) == ESP_OK) {
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
 * EMA-refine s_bias_z from raw gyro reading when car is stationary.
 * ════════════════════════════════════════════════════════════════════════════ */
void imu_gyro_zupt_update(void)
{
    uint8_t buf[2] = {0};
    if (imu_read(REG_GYRO_ZOUT_H, buf, 2) != ESP_OK) return;
    int16_t raw    = (int16_t)((buf[0] << 8) | buf[1]);
    float gz_deg_s = (float)raw / GYRO_SENS;
    float gz_raw   = gz_deg_s * ((float)M_PI / 180.0f);
    s_bias_z += ZUPT_GAIN * (gz_raw - s_bias_z);
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
    s_live_heading = heading;   /* publish before first tick so scan_task sees it */
    uint32_t elapsed = 0;

    while (elapsed < drive_ms) {
        if (s_stop_check && s_stop_check()) break;   /* emergency stop */
        uint32_t step = (drive_ms - elapsed < POLL_MS) ? (drive_ms - elapsed) : POLL_MS;
        float gz = imu_gyro_read_z();
        heading += gz * ((float)step / 1000.0f);
        s_live_heading = heading;   /* publish each integration step (~100 Hz) */
        vTaskDelay(pdMS_TO_TICKS(step));
        elapsed += step;
    }

    /* Normalise to (-π, π] */
    while (heading >  (float)M_PI) heading -= 2.0f * (float)M_PI;
    while (heading < -(float)M_PI) heading += 2.0f * (float)M_PI;

    s_live_heading = heading;   /* publish normalised final value */
    return heading;
}
