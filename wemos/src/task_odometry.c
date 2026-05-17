#include "task_odometry.h"

#include "encoder_ackermann_odometry.h"
#include "imu_encoder_driver.h"
#include "imu_gyro.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_timer.h"
#include <math.h>
#include <stdint.h>

static const char *TAG = "task_odometry";

/* Shared odometry state — written by task_odometry, internal use only */
static encoder_ackermann_odom_t s_odom;

/* Published pose snapshot — written under lock by task_odometry,
 * read by task_pure_pursuit via task_odometry_copy_pose(). */
static odom_pose_t s_published_pose;

/* Protects s_published_pose between writer (task_odometry) and reader
 * (task_pure_pursuit).  portMUX critical section is ~100 ns — safe for
 * the 16-byte struct copy without disabling the watchdog. */
static portMUX_TYPE s_pose_mux = portMUX_INITIALIZER_UNLOCKED;

/* I2C bus mutex — both imu_gyro_update() (ICM-20948) and
 * imu_encoder_driver_update() (AS5600) share the same I2C bus.
 * Initialised in task_odometry() before any I2C call.
 * Exposed via task_odometry_get_i2c_mutex() so any future I2C user can
 * participate in the same exclusion without a global extern. */
static SemaphoreHandle_t s_i2c_mutex = NULL;

/* Current commanded servo steering angle in radians (0 = straight).
 * Written by task_pure_pursuit via task_odometry_set_steering_rad().
 * Single 32-bit float write is atomic on Xtensa LX7 — no lock needed. */
static volatile float s_current_steering_rad = 0.0f;

/* ZUPT: count consecutive ticks with encoder delta < 1 mm */
static int s_stationary_ticks = 0;
#define ZUPT_STATIONARY_TICKS  20       /* 200 ms at 100 Hz */
#define ZUPT_DIST_THRESHOLD_M  0.001f   /* 1 mm */

/* Ackermann / complementary filter config.
 *
 * All three sensors are fused every 10 ms:
 *   AS5600 encoder   → cumulative distance (mm), differenced per tick
 *   ICM-20948 IMU    → yaw heading (rad), bias-corrected, ZUPT-refined
 *   Servo command    → steering_rad, used for Ackermann fallback heading
 *
 * imu_correction_gain = 1.0 means IMU is primary; Ackermann heading is the
 * fallback when the IMU jumps more than max_yaw_jump_rad in one tick.
 * Set to 0.7 once the Ackermann steering geometry is validated in hardware.
 */
static const odom_config_t k_cfg = {
    .wheelbase_m         = 0.258f,   /* measured front-axle to rear-axle */
    .imu_correction_gain = 1.0f,     /* 1.0 = IMU primary */
    .max_delta_dist_m    = 0.08f,    /* max plausible encoder step per 10 ms */
    .max_yaw_jump_rad    = 0.35f,    /* ~20°/tick — above this IMU is distrusted */
};

/* ── Public API ─────────────────────────────────────────────────────────────── */

void task_odometry_set_steering_rad(float steering_rad)
{
    s_current_steering_rad = steering_rad;
}

void task_odometry_copy_pose(odom_pose_t *out)
{
    taskENTER_CRITICAL(&s_pose_mux);
    *out = s_published_pose;
    taskEXIT_CRITICAL(&s_pose_mux);
}

const odom_pose_t *task_odometry_get_pose(void)
{
    return encoder_ackermann_odom_get_pose(&s_odom);
}

SemaphoreHandle_t task_odometry_get_i2c_mutex(void)
{
    return s_i2c_mutex;
}

/* ── Task ───────────────────────────────────────────────────────────────────── */

void task_odometry(void *pvParameters)
{
    (void)pvParameters;

    /* I2C mutex — created here, before any I2C access.
     * Both the AS5600 and ICM-20948 share the same I2C bus, so all reads
     * must be serialised.  Any other future I2C user should acquire this
     * mutex via task_odometry_get_i2c_mutex() rather than a raw extern. */
    s_i2c_mutex = xSemaphoreCreateMutex();
    configASSERT(s_i2c_mutex);

    encoder_ackermann_odom_init(&s_odom, &k_cfg);

    int64_t last_log_us  = 0;
    int64_t last_tick_us = esp_timer_get_time();

    static float s_prev_distance_m = 0.0f;

    for (;;) {
        /* ── 1. Measure real dt ───────────────────────────────────────────
         * Capture time BEFORE I2C so dt includes the full tick period,
         * not just the scheduling delay.  Clamped to [5, 50] ms to guard
         * against first-tick and scheduler-pause edge cases.              */
        int64_t now_us = esp_timer_get_time();
        float dt_s = (float)(now_us - last_tick_us) * 1e-6f;
        last_tick_us = now_us;
        if (dt_s < 0.005f) dt_s = 0.005f;
        if (dt_s > 0.050f) dt_s = 0.050f;

        /* ── 2. Read both I2C sensors under the shared bus mutex ─────────
         * imu_gyro_update()          → ICM-20948 at 0x68
         * imu_encoder_driver_update() → AS5600    at 0x36
         * Both share the same I2C bus; interleaving would corrupt reads. */
        xSemaphoreTake(s_i2c_mutex, portMAX_DELAY);
        imu_gyro_update(dt_s);
        imu_encoder_driver_update();
        xSemaphoreGive(s_i2c_mutex);

        float distance_m   = imu_encoder_driver_get_distance_m();
        float yaw_rad      = imu_encoder_driver_get_yaw_rad();
        float steering_rad = s_current_steering_rad;

        /* ── 3. ZUPT — zero-velocity update ──────────────────────────────
         * When encoder delta < 1 mm for 200 ms the car is stationary.
         * imu_gyro_zupt_update() corrects accumulated gyro bias in-place,
         * keeping heading accurate across stop-and-go manoeuvres.        */
        float delta_dist_m = fabsf(distance_m - s_prev_distance_m);
        s_prev_distance_m  = distance_m;

        if (delta_dist_m < ZUPT_DIST_THRESHOLD_M) {
            if (++s_stationary_ticks >= ZUPT_STATIONARY_TICKS)
                imu_gyro_zupt_update();
        } else {
            s_stationary_ticks = 0;
        }

        /* ── 4. Ackermann + IMU fusion ────────────────────────────────────
         * encoder_ackermann_odom_update() blends:
         *   - cumulative encoder distance (arc length)
         *   - IMU heading (primary, bias-corrected)
         *   - Ackermann heading prediction (fallback on IMU fault)
         * Trig (cosf/sinf) runs outside the portMUX critical section —
         * the watchdog must not be suspended for >15 ms.               */
        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
        encoder_ackermann_odom_update(&s_odom, distance_m, steering_rad, yaw_rad, now_ms);

        /* Publish atomically — 16-byte struct copy under spinlock. */
        taskENTER_CRITICAL(&s_pose_mux);
        s_published_pose = s_odom.pose;
        taskEXIT_CRITICAL(&s_pose_mux);

        /* ── 5. 1 Hz diagnostic log ──────────────────────────────────────
         * Printed values feed directly into odom_t sent to the ESP32-S3.
         * x/y/theta here ARE what the S3 integrates into its world pose. */
        now_us = esp_timer_get_time();
        if ((now_us - last_log_us) > 1000000LL) {
            const odom_pose_t *p = task_odometry_get_pose();
            if (p)
                ESP_LOGI(TAG,
                         "x=%.3f m  y=%.3f m  θ=%.2f°  dist=%.3f m"
                         "  steer=%.1f°  zupt=%d  hwm=%u B",
                         (double)p->x, (double)p->y,
                         (double)(p->theta * 180.0f / (float)M_PI),
                         (double)distance_m,
                         (double)(steering_rad * 180.0f / (float)M_PI),
                         s_stationary_ticks >= ZUPT_STATIONARY_TICKS,
                         (unsigned)(uxTaskGetStackHighWaterMark(NULL) * sizeof(StackType_t)));
            last_log_us = now_us;
        }

        vTaskDelay(pdMS_TO_TICKS(10));   /* 100 Hz */
    }
}
