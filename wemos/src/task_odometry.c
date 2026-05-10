#include "task_odometry.h"

#include "encoder_ackermann_odometry.h"
#include "imu_encoder_driver.h"
#include "imu_gyro.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_timer.h"

#include <math.h>
#include <stdint.h>

static const char *TAG = "task_odometry";

/* Shared odometry state — written by task_odometry, read via task_odometry_copy_pose() */
static encoder_ackermann_odom_t s_odom;

/* Protects s_odom.pose between writer (task_odometry) and reader (bridge_slave_task).
 * Critical section keeps it to ~100 ns on the read side — safe for UART ISRs. */
static portMUX_TYPE s_pose_mux = portMUX_INITIALIZER_UNLOCKED;

/* Current commanded servo steering angle in radians (0 = straight).
 * Written by bridge_slave_task via task_odometry_set_steering_rad().
 * Single 32-bit float write is atomic on Xtensa LX7 — no lock needed. */
static volatile float s_current_steering_rad = 0.0f;

/* ZUPT: count consecutive ticks with encoder delta < 1 mm */
static int s_stationary_ticks = 0;
#define ZUPT_STATIONARY_TICKS  20       /* 200 ms at 100 Hz */
#define ZUPT_DIST_THRESHOLD_M  0.001f   /* 1 mm */

/* Ackermann / complementary filter config.
 *
 * All three sensors are now fused:
 *   - AS5600 encoder   → ds (incremental distance per 10 ms tick)
 *   - ICM-20948 IMU    → yaw_rad (heading, bias-corrected, ZUPT-refined)
 *   - Servo command    → steering_rad (Ackermann heading prediction)
 *
 * imu_correction_gain controls the blend:
 *   1.0 → heading = IMU reading (Ackermann is sanity-check fallback on IMU fault)
 *   0.0 → heading = Ackermann prediction only
 *   0.7 → recommended blend once steering geometry is validated on hardware
 *
 * With gain = 1.0 the Ackermann heading (steering_rad) still contributes when the
 * IMU reading jumps more than max_yaw_jump_rad in a single 10 ms tick: in that case
 * the IMU is treated as unreliable and the Ackermann prediction carries the heading
 * forward.  Without a real steering angle that fallback was always "go straight".
 */
static const odom_config_t k_cfg = {
    .wheelbase_m         = 0.258f,   /* measured — front-axle to rear-axle */
    .imu_correction_gain = 1.0f,     /* 1.0 = IMU primary; lower to blend Ackermann */
    .max_delta_dist_m    = 0.08f,    /* max plausible encoder step per 10 ms */
    .max_yaw_jump_rad    = 0.35f     /* ~20 deg/tick — above this the IMU is distrusted */
};

/* ── Public API ─────────────────────────────────────────────────────────────── */

/* Set the commanded servo steering angle.
 * Call from bridge_slave_task immediately after writing the servo duty. */
void task_odometry_set_steering_rad(float steering_rad)
{
    s_current_steering_rad = steering_rad;
}

/* Atomically copy the latest odometry pose into *out.
 * The critical section (~100 ns) prevents the FreeRTOS tick ISR from switching
 * to task_odometry mid-copy, ensuring x, y, theta, timestamp_ms are from the
 * same integration step. */
void task_odometry_copy_pose(odom_pose_t *out)
{
    taskENTER_CRITICAL(&s_pose_mux);
    *out = s_odom.pose;
    taskEXIT_CRITICAL(&s_pose_mux);
}

/* Internal accessor used only by the task itself (no lock needed — same task). */
const odom_pose_t *task_odometry_get_pose(void)
{
    return encoder_ackermann_odom_get_pose(&s_odom);
}

/* ── Internal helpers ───────────────────────────────────────────────────────── */

static float get_steering_angle_rad(void)
{
    return s_current_steering_rad;
}

/* ── Task ───────────────────────────────────────────────────────────────────── */

void task_odometry(void *pvParameters)
{
    (void)pvParameters;

    encoder_ackermann_odom_init(&s_odom, &k_cfg);

    int64_t last_log_us = 0;

    for (;;) {
        /* 1. Integrate gyro Z → fresh heading available before encoder update */
        imu_gyro_update(0.01f);

        /* 2. Read AS5600 encoder tick (cumulative distance) */
        imu_encoder_driver_update();

        float distance_m   = imu_encoder_driver_get_distance_m();  /* cumulative, metres */
        float yaw_rad      = imu_encoder_driver_get_yaw_rad();      /* IMU heading, rad   */
        float steering_rad = get_steering_angle_rad();               /* servo command, rad */

        /* 3. ZUPT — refine gyro bias when stationary (encoder delta < 1 mm) */
        static float s_prev_distance_m = 0.0f;
        float delta_dist_m = fabsf(distance_m - s_prev_distance_m);
        s_prev_distance_m  = distance_m;

        if (delta_dist_m < ZUPT_DIST_THRESHOLD_M) {
            if (++s_stationary_ticks >= ZUPT_STATIONARY_TICKS)
                imu_gyro_zupt_update();
        } else {
            s_stationary_ticks = 0;
        }

        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);

        /* 4. Fuse encoder + steering (Ackermann) + IMU into pose.
         *    Lock only during the struct write so bridge_slave_task always gets
         *    a coherent snapshot from task_odometry_copy_pose(). */
        taskENTER_CRITICAL(&s_pose_mux);
        encoder_ackermann_odom_update(&s_odom,
                                      distance_m,
                                      steering_rad,
                                      yaw_rad,
                                      now_ms);
        taskEXIT_CRITICAL(&s_pose_mux);

        /* 5. Log once per second */
        int64_t now_us = esp_timer_get_time();
        if ((now_us - last_log_us) > 1000000LL) {
            const odom_pose_t *p = task_odometry_get_pose();
            if (p)
                ESP_LOGI(TAG,
                         "x=%.3f m  y=%.3f m  θ=%.2f°  dist=%.3f m  steer=%.1f°",
                         p->x, p->y,
                         (double)(p->theta * 180.0f / (float)M_PI),
                         distance_m,
                         (double)(steering_rad * 180.0f / (float)M_PI));
            last_log_us = now_us;
        }

        vTaskDelay(pdMS_TO_TICKS(10));   /* 100 Hz */
    }
}
