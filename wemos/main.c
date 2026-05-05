/**
 * wemos/main.c
 * Board: Wemos D1 R32 (ESP32) — standalone SLAM brain + motor controller.
 *
 * Normal two-board architecture:
 *   ESP32-S3 (SLAM brain) ──UART──► Wemos D1 R32 (motor controller)
 *
 * This file collapses both roles onto the Wemos for use when the ESP32-S3 is
 * unavailable.  The SLAM modules from esp32s3/src/ are compiled in unchanged.
 * Motor control is wired directly here — no UART relay.
 *
 * ⚠  In the current two-board circuit revision the LiDAR and IMU are wired
 *    to the ESP32-S3, not the Wemos.  This standalone build compiles but
 *    requires re-wiring those sensors back to the Wemos to run correctly.
 *    Pin assignments are centralised in hardware_pins.h.
 *
 * Two-task architecture (solves synchronisation):
 *   scan_task  (priority 5)  — reads LiDAR + updates map continuously,
 *                              even while the robot is driving.
 *   plan_task  (priority 3)  — frontier detection, dashboard push, drive.
 *
 * Shared state (s_map, s_pose) is protected by s_mtx:
 *   scan_task  holds s_mtx only during lidar_to_map() — typically 5–15 ms.
 *   plan_task  holds s_mtx only during frontier_detector_detect() + pose
 *              update — releases it before drive_for_cmd() so scan_task
 *              can continue updating the map while the car is moving.
 *
 * Odometry integration (added):
 *   task_odometry runs at 100 Hz on Core 0, fusing AS5600 encoder + ICM-20948
 *   gyro via Ackermann complementary filter. update_pose_from_odometry()
 *   replaces dead_reckon_pose() in the exploration loop and idle loop.
 *   Hardware pin parity with 2025fa-SLAMurai Wemos:
 *     Motor F/B   — GPIO 13 / 12, LEDC ch 3 / 2
 *     Servo       — GPIO 23
 *     LiDAR UART  — RX 17, TX 16, 460800 baud
 *     IMU I2C     — SDA 21, SCL 22
 */

/* ── Wi-Fi credentials ──────────────────────────────────────────────────── */
#define WIFI_SSID      "SPOT-iot"
#define WIFI_PASSWORD  "RacailleSalutaireMigration8052"

/* ── Heading test mode — uncomment to re-enable, comment out for exploration ─ */
// #define HEADING_TEST_MODE

/* ── Hardware test modes — uncomment exactly one; all others must be off ─── */
//#define TEST_MOTOR       /* drive forward 30 cm then stop                   */
// #define TEST_SERVO       /* sweep: centre → left → centre → right → centre  */
// #define TEST_IMU         /* WHO_AM_I check + live gyro-Z / heading read     */
// #define TEST_AS5600      /* I2C scan + continuous angle read (AS5600 @ 0x36)*/
// #define TEST_BRIDGE_RX   /* receive and decode UART bridge frames from ESP32-S3 */
// #define TEST_BRIDGE_PONG /* interactive: press ENTER → send "hey2", print replies */
#define BRIDGE_SLAVE     /* receive drive commands from ESP32-S3 and execute them */

#include "hardware_pins.h"

/* ── SLAM brain modules ─────────────────────────────────────────────────── */
#include "quadtree_map.h"
#include "frontier_detector.h"
#include "command_gen.h"
#include "wifi_dashboard.h"
#include "lidar_driver.h"
#include "lidar_to_map.h"
#include "imu_gyro.h"
#include "uart_bridge.h"
#include "pure_pursuit_controller.h"

/* ── Odometry modules (added) ───────────────────────────────────────────── */
#include "motor_pid.h"
#include "imu_encoder_driver.h"
#include "task_odometry.h"

/* ── ESP-IDF / FreeRTOS ─────────────────────────────────────────────────── */
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include <string.h>

/* ── Map dimensions ──────────────────────────────────────────────────────── */
#define MAP_SIZE_MM    10000.0f
#define MAP_STEP_MM    50.0f
#define START_X_MM     5000.0f
#define START_Y_MM     5000.0f
#define INIT_FREE_R_MM 500.0f

/* ── Scan integration ────────────────────────────────────────────────────── */
#define SCAN_MAX_MM   5500.0f   /* match simulator LIDAR_RANGE; walls up to 9 m visible */
#define SCAN_STEP_MM   150.0f   /* matches depth-7 leaf cell (~156 mm) */

/* ── Planning loop constants ─────────────────────────────────────────────── */
#define DEBUG_FORWARD_MM    200.0f
#define DEBUG_SPEED_MM_S    100.0f
#define MAX_DRIVE_MS        1200u
#define CYCLE_DELAY_MS      50

#define TURN_ARC_THRESH_RAD  0.785f   /* π/4 */
#define TURN_ARC_MS          800u

/* ════════════════════════════════════════════════════════════════════════════
 * Motor config  (pins from hardware_pins.h)
 * ════════════════════════════════════════════════════════════════════════════ */
#define MOTOR_PWM_FREQ_HZ  1000
#define MOTOR_PWM_RES      LEDC_TIMER_8_BIT
#define MOTOR_DUTY_FWD     35    /* calibrated — was 25, increased for our car */
#define MOTOR_DUTY_STOP    0
#define MOTOR_SPEED_MM_S   346.0f  /* calibrated — was 360, measured on our car */
#define CH_FWD   LEDC_CHANNEL_3
#define CH_BWD   LEDC_CHANNEL_2

/* ════════════════════════════════════════════════════════════════════════════
 * Servo config  (pin from hardware_pins.h)
 * ════════════════════════════════════════════════════════════════════════════ */
#define SERVO_PWM_FREQ_HZ  50
#define SERVO_PWM_RES      LEDC_TIMER_16_BIT
#define SERVO_CH           LEDC_CHANNEL_0
#define SERVO_DUTY_CENTER  4987u
#define SERVO_DUTY_LEFT    4442u
#define SERVO_DUTY_RIGHT   5533u
#define SERVO_STEER_GAIN   1042.0f

/* ════════════════════════════════════════════════════════════════════════════
 * Shared state — protected by s_mtx
 * ════════════════════════════════════════════════════════════════════════════ */
static quadtree_map_t    s_map;
static pose_t            s_pose;
static SemaphoreHandle_t s_mtx;

/* ── Drive interpolation state ───────────────────────────────────────────────
 * plan_task arms this before calling drive_for_cmd(); scan_task reads it to
 * compute a live interpolated pose instead of using the stale s_pose.
 * ──────────────────────────────────────────────────────────────────────────── */
typedef struct {
    float      x0;
    float      y0;
    float      speed;
    TickType_t t0;
    bool       active;
} drive_info_t;
static drive_info_t s_drv;

/* ════════════════════════════════════════════════════════════════════════════
 * update_pose_from_odometry  (added)
 * Reads from task_odometry and updates s_pose in mm for the map.
 * Must be called with s_mtx held.
 * ════════════════════════════════════════════════════════════════════════════ */
static void update_pose_from_odometry(void)
{
    const odom_pose_t *odom = task_odometry_get_pose();
    if (odom != NULL) {
        s_pose.x     = START_X_MM + odom->x * 1000.0f;
        s_pose.y     = START_Y_MM + odom->y * 1000.0f;
        s_pose.theta = odom->theta;
    }
}

/* ════════════════════════════════════════════════════════════════════════════
 * Motor helpers
 * ════════════════════════════════════════════════════════════════════════════ */
static void motor_init(void)
{
    ledc_timer_config_t timer = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = MOTOR_PWM_RES,
        .timer_num       = LEDC_TIMER_0,
        .freq_hz         = MOTOR_PWM_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&timer);

    ledc_channel_config_t ch[] = {
        { .gpio_num=MOTOR_F_PIN, .speed_mode=LEDC_LOW_SPEED_MODE,
          .channel=CH_FWD, .timer_sel=LEDC_TIMER_0, .duty=0, .hpoint=0 },
        { .gpio_num=MOTOR_B_PIN, .speed_mode=LEDC_LOW_SPEED_MODE,
          .channel=CH_BWD, .timer_sel=LEDC_TIMER_0, .duty=0, .hpoint=0 },
    };
    for (int i = 0; i < 2; i++) ledc_channel_config(&ch[i]);

    ledc_timer_config_t servo_timer = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = SERVO_PWM_RES,
        .timer_num       = LEDC_TIMER_1,
        .freq_hz         = SERVO_PWM_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&servo_timer);

    ledc_channel_config_t servo_ch = {
        .gpio_num   = SERVO_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = SERVO_CH,
        .timer_sel  = LEDC_TIMER_1,
        .duty       = SERVO_DUTY_CENTER,
        .hpoint     = 0,
    };
    ledc_channel_config(&servo_ch);
}

static void set_duty(ledc_channel_t ch, uint32_t duty)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, ch, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, ch);
}

static void motors_stop(void)
{
    set_duty(CH_FWD, MOTOR_DUTY_STOP);
    set_duty(CH_BWD, MOTOR_DUTY_STOP);
}

static uint32_t servo_deg_to_duty(float servo_deg)
{
    if (servo_deg < 60.0f)  servo_deg = 60.0f;
    if (servo_deg > 120.0f) servo_deg = 120.0f;

    float ratio = (servo_deg - 90.0f) / 30.0f;

    int32_t duty = (int32_t)SERVO_DUTY_CENTER +
                   (int32_t)(ratio * (float)(SERVO_DUTY_RIGHT - SERVO_DUTY_CENTER));

    if (duty < (int32_t)SERVO_DUTY_LEFT)  duty = SERVO_DUTY_LEFT;
    if (duty > (int32_t)SERVO_DUTY_RIGHT) duty = SERVO_DUTY_RIGHT;

    return (uint32_t)duty;
}

/* ════════════════════════════════════════════════════════════════════════════
 * drive_for_cmd — steers then drives; returns actual heading from gyro.
 * ════════════════════════════════════════════════════════════════════════════ */
static float drive_for_cmd(const control_frame_t *cmd, float current_heading)
{
    float err = cmd->t_heading - current_heading;
    while (err >  (float)M_PI) err -= 2.0f * (float)M_PI;
    while (err < -(float)M_PI) err += 2.0f * (float)M_PI;

    int32_t duty = (int32_t)SERVO_DUTY_CENTER + (int32_t)(err * SERVO_STEER_GAIN);
    if (duty < (int32_t)SERVO_DUTY_LEFT)  duty = (int32_t)SERVO_DUTY_LEFT;
    if (duty > (int32_t)SERVO_DUTY_RIGHT) duty = (int32_t)SERVO_DUTY_RIGHT;
    set_duty(SERVO_CH, (uint32_t)duty);
    vTaskDelay(pdMS_TO_TICKS(50));

    float actual_heading = current_heading;

    if (fabsf(err) > TURN_ARC_THRESH_RAD) {
        char tbuf[64];
        snprintf(tbuf, sizeof(tbuf), "Turning: err=%.1f° (%ums)",
                 err * 180.0f / (float)M_PI, (unsigned)TURN_ARC_MS);
        wifi_dashboard_log(tbuf);

        set_duty(CH_FWD, MOTOR_DUTY_FWD);
        set_duty(CH_BWD, MOTOR_DUTY_STOP);
        actual_heading = imu_drive_and_track(actual_heading, TURN_ARC_MS);
        motors_stop();

        snprintf(tbuf, sizeof(tbuf), "Turn done: heading=%.1f° stop=%d",
                 actual_heading * 180.0f / (float)M_PI,
                 (int)wifi_dashboard_stop_peek());
        wifi_dashboard_log(tbuf);

        float err2 = cmd->t_heading - actual_heading;
        while (err2 >  (float)M_PI) err2 -= 2.0f * (float)M_PI;
        while (err2 < -(float)M_PI) err2 += 2.0f * (float)M_PI;
        int32_t duty2 = (int32_t)SERVO_DUTY_CENTER + (int32_t)(err2 * SERVO_STEER_GAIN);
        if (duty2 < (int32_t)SERVO_DUTY_LEFT)  duty2 = (int32_t)SERVO_DUTY_LEFT;
        if (duty2 > (int32_t)SERVO_DUTY_RIGHT) duty2 = (int32_t)SERVO_DUTY_RIGHT;
        set_duty(SERVO_CH, (uint32_t)duty2);
        vTaskDelay(pdMS_TO_TICKS(30));
    }

    float dist_mm = sqrtf(cmd->tx * cmd->tx + cmd->ty * cmd->ty);
    if (dist_mm < 50.0f) dist_mm = DEBUG_FORWARD_MM;

    float    speed    = (cmd->t_speed > 10.0f) ? cmd->t_speed : DEBUG_SPEED_MM_S;
    uint32_t drive_ms = (uint32_t)((dist_mm / speed) * 1000.0f);
    if (drive_ms > MAX_DRIVE_MS) drive_ms = MAX_DRIVE_MS;
    if (drive_ms < 50)           drive_ms = 50;

    {
        char dbuf[64];
        snprintf(dbuf, sizeof(dbuf), "Straight: dist=%.0fmm t=%ums",
                 dist_mm, (unsigned)drive_ms);
        wifi_dashboard_log(dbuf);
    }

    /* Start PID speed control for straight drive (added) */
    motor_pid_set_target(MOTOR_SPEED_MM_S / 1000.0f);
    set_duty(CH_FWD, MOTOR_DUTY_FWD);
    set_duty(CH_BWD, MOTOR_DUTY_STOP);
    actual_heading = imu_drive_and_track(actual_heading, drive_ms);
    motor_pid_stop();   /* added */
    motors_stop();

    {
        char dbuf[64];
        snprintf(dbuf, sizeof(dbuf), "Straight done: heading=%.1f° stop=%d",
                 actual_heading * 180.0f / (float)M_PI,
                 (int)wifi_dashboard_stop_peek());
        wifi_dashboard_log(dbuf);
    }

    return actual_heading;
}

/* ════════════════════════════════════════════════════════════════════════════
 * dead_reckon_pose — fallback if odometry unavailable
 * ════════════════════════════════════════════════════════════════════════════ */
static void dead_reckon_pose(pose_t *pose, const control_frame_t *cmd)
{
    float dist_mm = sqrtf(cmd->tx * cmd->tx + cmd->ty * cmd->ty);
    if (dist_mm < 1.0f) return;

    float speed = (cmd->t_speed > 10.0f) ? cmd->t_speed : DEBUG_SPEED_MM_S;
    float dt_s  = dist_mm / speed;
    float max_s = MAX_DRIVE_MS / 1000.0f;
    if (dt_s > max_s) dt_s = max_s;

    float traveled = speed * dt_s;
    pose->x    += traveled * cosf(cmd->t_heading);
    pose->y    += traveled * sinf(cmd->t_heading);
    pose->theta = cmd->t_heading;
}


#ifndef HEADING_TEST_MODE
/* ════════════════════════════════════════════════════════════════════════════
 * scan_task  (priority 5)
 * ════════════════════════════════════════════════════════════════════════════ */
static void scan_task(void *arg)
{
    (void)arg;
    static lidar_scan_t scan;
    TickType_t last_scan_broadcast = 0;

    while (1) {
        if (!lidar_driver_read_scan(&scan) || scan.count <= 10) { vTaskDelay(pdMS_TO_TICKS(1)); continue; }

        xSemaphoreTake(s_mtx, portMAX_DELAY);
        pose_t local_pose;
        if (s_drv.active) {
            float elapsed_s  = (float)(xTaskGetTickCount() - s_drv.t0)
                               / (float)configTICK_RATE_HZ;
            float live_theta = imu_gyro_get_heading();
            local_pose.theta = live_theta;
            local_pose.x     = s_drv.x0 + s_drv.speed * elapsed_s * cosf(live_theta);
            local_pose.y     = s_drv.y0 + s_drv.speed * elapsed_s * sinf(live_theta);
        } else {
            local_pose = s_pose;
        }
        xSemaphoreGive(s_mtx);

        map_dirty_rect_t dr;
        xSemaphoreTake(s_mtx, portMAX_DELAY);
        lidar_to_map(&s_map, &scan, &local_pose, SCAN_MAX_MM, SCAN_STEP_MM, &dr);
        xSemaphoreGive(s_mtx);
        wifi_dashboard_mark_dirty(&dr);

        TickType_t now = xTaskGetTickCount();
        if ((now - last_scan_broadcast) >= pdMS_TO_TICKS(500)) {
            wifi_dashboard_broadcast_scan(&scan, &local_pose);
            last_scan_broadcast = now;
        }
    }
}


/* ════════════════════════════════════════════════════════════════════════════
 * plan_task  (priority 3)
 * ════════════════════════════════════════════════════════════════════════════ */
static void plan_task(void *arg)
{
    (void)arg;

    for (;;) {

        printf("[SLAMborghini] Waiting for dashboard Start...\n");

        int idle_tick = 0;
        while (!wifi_dashboard_exploration_requested()) {
            pose_t p;
            xSemaphoreTake(s_mtx, portMAX_DELAY);
            update_pose_from_odometry();   /* added — live pose in idle */
            p = s_pose;
            xSemaphoreGive(s_mtx);
            wifi_dashboard_update(&s_map, &p);
            wifi_dashboard_broadcast_state(&p, 0.0f, 0.0f, false, 0);
            if (idle_tick % 25 == 0)
                wifi_dashboard_log("IMU: ready | LIDAR: running | Waiting for Start...");
            idle_tick++;
            vTaskDelay(pdMS_TO_TICKS(200));
        }
        wifi_dashboard_stop_requested();
        printf("[SLAMborghini] Exploration started.\n");

        int cycle = 0;
        while (1) {
            cycle++;
            {
                char cbuf[32];
                snprintf(cbuf, sizeof(cbuf), "-- Cycle %d --", cycle);
                wifi_dashboard_log(cbuf);
            }

            if (wifi_dashboard_stop_requested()) {
                motors_stop();
                motor_pid_stop();   /* added */
                wifi_dashboard_log("STOP requested — press Start to resume");
                printf("[SLAMborghini] Stopped.\n");
                break;
            }

            pose_t local_pose;
            frontier_list_t fronts;

            xSemaphoreTake(s_mtx, portMAX_DELAY);
            update_pose_from_odometry();   /* added — use real odometry */
            local_pose = s_pose;
            fronts     = frontier_detector_detect(&s_map, &local_pose);
            xSemaphoreGive(s_mtx);

            wifi_dashboard_update(&s_map, &local_pose);
            wifi_dashboard_broadcast_quadtree(&s_map);   /* added */

            frontier_t best   = frontier_detector_best(&fronts, &local_pose);
            bool has_frontier = (best.size > 0);
            wifi_dashboard_broadcast_state(&local_pose,
                                           best.cx, best.cy, has_frontier, 0);

            if (!has_frontier) {
                wifi_dashboard_log("No frontier — exploration complete");
                printf("[SLAMborghini] No frontier — exploration complete.\n");
                motors_stop();
                break;
            }

            waypoint_t wp = {
                .x        = best.cx,
                .y        = best.cy,
                .theta    = 0.0f,
                .v_target = MOTOR_SPEED_MM_S,
            };
            control_frame_t cmd = command_gen_compute(&local_pose, &wp);
            cmd.t_speed = MOTOR_SPEED_MM_S;

            {
                char buf[80];
                snprintf(buf, sizeof(buf), "Drive → (%.0f, %.0f) fronts=%u",
                         best.cx, best.cy, (unsigned)fronts.count);
                wifi_dashboard_log(buf);
            }

            xSemaphoreTake(s_mtx, portMAX_DELAY);
            s_drv.x0     = local_pose.x;
            s_drv.y0     = local_pose.y;
            s_drv.speed  = cmd.t_speed;
            s_drv.t0     = xTaskGetTickCount();
            s_drv.active = true;
            xSemaphoreGive(s_mtx);

            float new_heading = drive_for_cmd(&cmd, local_pose.theta);

            /* Update pose — odometry first, dead reckoning as fallback */
            xSemaphoreTake(s_mtx, portMAX_DELAY);
            s_drv.active = false;
            const odom_pose_t *odom = task_odometry_get_pose();
            if (odom != NULL) {
                s_pose.x     = START_X_MM + odom->x * 1000.0f;
                s_pose.y     = START_Y_MM + odom->y * 1000.0f;
                s_pose.theta = odom->theta;
            } else {
                dead_reckon_pose(&s_pose, &cmd);
                s_pose.theta = new_heading;
            }
            pose_t updated_pose = s_pose;
            xSemaphoreGive(s_mtx);

            {
                char pbuf[72];
                snprintf(pbuf, sizeof(pbuf), "Pose: (%.0f, %.0f) θ=%.1f° stop=%d",
                         updated_pose.x, updated_pose.y,
                         updated_pose.theta * 180.0f / (float)M_PI,
                         (int)wifi_dashboard_stop_peek());
                wifi_dashboard_log(pbuf);
            }

            vTaskDelay(pdMS_TO_TICKS(CYCLE_DELAY_MS));
        }
    }
}
#endif /* !HEADING_TEST_MODE */


/* ════════════════════════════════════════════════════════════════════════════
 * heading_test_task
 * ════════════════════════════════════════════════════════════════════════════ */
#ifdef HEADING_TEST_MODE
#define HT_TARGET_DEG   20.0f
#define HT_TARGET_RAD   (HT_TARGET_DEG * (float)M_PI / 180.0f)
#define HT_DRIVE_MS     800u

static void heading_test_task(void *arg)
{
    (void)arg;
    static float current_heading = 0.0f;
    int run = 0;

    for (;;) {
        printf("\n[HeadingTest] Press ENTER in this terminal to run test (run %d)...\n", run + 1);
        while (getchar() == EOF)
            vTaskDelay(pdMS_TO_TICKS(20));
        vTaskDelay(pdMS_TO_TICKS(50));
        while (getchar() != EOF);

        run++;
        float theta0 = current_heading;
        float target = theta0 + HT_TARGET_RAD;

        float err = HT_TARGET_RAD;
        int32_t duty = (int32_t)SERVO_DUTY_CENTER + (int32_t)(err * SERVO_STEER_GAIN);
        if (duty < (int32_t)SERVO_DUTY_LEFT)  duty = (int32_t)SERVO_DUTY_LEFT;
        if (duty > (int32_t)SERVO_DUTY_RIGHT) duty = (int32_t)SERVO_DUTY_RIGHT;

        printf("\n=== HEADING TEST START (run %d) ===\n", run);
        printf("  target_heading  = %.2f deg\n", target * 180.0f / (float)M_PI);
        printf("  initial_heading = %.2f deg\n", theta0 * 180.0f / (float)M_PI);
        printf("  servo_command   = %ld (center=%u)\n", (long)duty, (unsigned)SERVO_DUTY_CENTER);

        set_duty(SERVO_CH, (uint32_t)duty);
        vTaskDelay(pdMS_TO_TICKS(50));

        set_duty(CH_FWD, MOTOR_DUTY_FWD);
        set_duty(CH_BWD, MOTOR_DUTY_STOP);
        float theta1 = imu_drive_and_track(theta0, HT_DRIVE_MS);

        set_duty(CH_FWD, MOTOR_DUTY_STOP);
        set_duty(CH_BWD, 20);
        vTaskDelay(pdMS_TO_TICKS(80));
        set_duty(CH_BWD, MOTOR_DUTY_STOP);
        vTaskDelay(pdMS_TO_TICKS(300));

        set_duty(SERVO_CH, SERVO_DUTY_CENTER);

        float delta = theta1 - theta0;
        float herr  = HT_TARGET_RAD - delta;

        printf("  final_heading   = %.2f deg\n", theta1 * 180.0f / (float)M_PI);
        printf("  delta_heading   = %.2f deg  (target %.1f deg)\n",
               delta * 180.0f / (float)M_PI, HT_TARGET_DEG);
        printf("  heading_error   = %.2f deg\n", herr * 180.0f / (float)M_PI);
        printf("=== HEADING TEST END — press ENTER to run again ===\n");

        current_heading = theta1;
    }
}
#endif /* HEADING_TEST_MODE */


/* ════════════════════════════════════════════════════════════════════════════
 * TEST_MOTOR
 * ════════════════════════════════════════════════════════════════════════════ */
#ifdef TEST_MOTOR
static void test_motor_task(void *arg)
{
    (void)arg;
    printf("[TEST_MOTOR] LEDC PWM  FWD=GPIO%d  BWD=GPIO%d  duty=80/255\n",
           MOTOR_F_PIN, MOTOR_B_PIN);
    printf("[TEST_MOTOR] Press any key to drive 2 s (repeat as needed)\n");

    for (;;) {
        while (getchar() == EOF) vTaskDelay(pdMS_TO_TICKS(20));
        while (getchar() != EOF);

        printf("[TEST_MOTOR] GO\n");
        set_duty(CH_FWD, MOTOR_DUTY_FWD);
        set_duty(CH_BWD, 0);
        vTaskDelay(pdMS_TO_TICKS(2000));
        set_duty(CH_FWD, 0);
        set_duty(CH_BWD, 0);
        printf("[TEST_MOTOR] STOP — press any key to go again\n");
    }
}
#endif /* TEST_MOTOR */


/* ════════════════════════════════════════════════════════════════════════════
 * TEST_IMU
 * ════════════════════════════════════════════════════════════════════════════ */
#ifdef TEST_IMU
static void test_imu_task(void *arg)
{
    (void)arg;
    bool ok = imu_gyro_init();
    printf("[TEST_IMU] SDA=GPIO%d  SCL=GPIO%d  addr=0x68\n",
           IMU_SDA_PIN, IMU_SCL_PIN);
    printf("[TEST_IMU] WHO_AM_I: %s\n",
           ok ? "0xEA OK — ICM-20948 found"
              : "FAIL — check wiring and 3V3 supply");
    if (!ok) { vTaskDelete(NULL); return; }

    printf("[TEST_IMU] Reading gyro Z at 5 Hz — rotate robot to see heading change...\n");
    float heading = 0.0f;
    for (;;) {
        float gz = imu_gyro_read_z();
        heading += gz * 0.2f;
        printf("[TEST_IMU] gz=% .4f rad/s   heading=% 7.2f deg\n",
               (double)gz,
               (double)(heading * 180.0f / (float)M_PI));
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
#endif /* TEST_IMU */


/* ════════════════════════════════════════════════════════════════════════════
 * TEST_SERVO
 * ════════════════════════════════════════════════════════════════════════════ */
#ifdef TEST_SERVO
static void test_servo_task(void *arg)
{
    (void)arg;
    printf("[TEST_SERVO] Servo sweep in 3 s...\n");
    vTaskDelay(pdMS_TO_TICKS(3000));

    set_duty(SERVO_CH, SERVO_DUTY_CENTER);
    vTaskDelay(pdMS_TO_TICKS(1000));
    set_duty(SERVO_CH, SERVO_DUTY_LEFT);
    vTaskDelay(pdMS_TO_TICKS(1000));
    set_duty(SERVO_CH, SERVO_DUTY_CENTER);
    vTaskDelay(pdMS_TO_TICKS(1000));
    set_duty(SERVO_CH, SERVO_DUTY_RIGHT);
    vTaskDelay(pdMS_TO_TICKS(1000));
    set_duty(SERVO_CH, SERVO_DUTY_CENTER);
    vTaskDelay(pdMS_TO_TICKS(500));
    set_duty(SERVO_CH, 0);
    printf("[TEST_SERVO] Done.\n");
    vTaskDelete(NULL);
}
#endif /* TEST_SERVO */


/* ════════════════════════════════════════════════════════════════════════════
 * TEST_AS5600
 * ════════════════════════════════════════════════════════════════════════════ */
#ifdef TEST_AS5600
#define AS5600_ADDR        0x36
#define AS5600_REG_ANGLE_H 0x0E
#define AS5600_REG_ANGLE_L 0x0F

static void test_as5600_task(void *arg)
{
    (void)arg;

    i2c_config_t cfg = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = ENCODER_SDA_PIN,
        .scl_io_num       = ENCODER_SCL_PIN,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
    };
    i2c_param_config(I2C_NUM_0, &cfg);
    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
    vTaskDelay(pdMS_TO_TICKS(100));

    printf("[TEST_AS5600] I2C scan on SDA=GPIO%d SCL=GPIO%d...\n",
           ENCODER_SDA_PIN, ENCODER_SCL_PIN);
    bool found = false;
    for (uint8_t addr = 1; addr < 127; addr++) {
        uint8_t dummy = 0;
        if (i2c_master_write_to_device(I2C_NUM_0, addr, &dummy, 0,
                                        pdMS_TO_TICKS(10)) == ESP_OK) {
            printf("  0x%02X%s\n", addr,
                   addr == AS5600_ADDR ? " <- AS5600" : "");
            if (addr == AS5600_ADDR) found = true;
        }
    }
    if (!found)
        printf("  AS5600 not found — check power and SDA/SCL wiring\n");

    printf("[TEST_AS5600] Reading angle (rotate wheel to test)...\n");
    while (1) {
        uint8_t reg = AS5600_REG_ANGLE_H;
        uint8_t buf[2] = {0};
        esp_err_t r = i2c_master_write_read_device(
            I2C_NUM_0, AS5600_ADDR, &reg, 1, buf, 2, pdMS_TO_TICKS(10));
        if (r == ESP_OK) {
            uint16_t raw = ((uint16_t)(buf[0] & 0x0F) << 8) | buf[1];
            printf("[TEST_AS5600] %6.2f deg  (raw=%4u)\n",
                   (double)(raw * 360.0f / 4096.0f), (unsigned)raw);
        } else {
            printf("[TEST_AS5600] Read error 0x%x\n", (unsigned)r);
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
#endif /* TEST_AS5600 */


/* ════════════════════════════════════════════════════════════════════════════
 * TEST_BRIDGE_RX
 * ════════════════════════════════════════════════════════════════════════════ */
#ifdef TEST_BRIDGE_RX
static void test_bridge_rx_task(void *arg)
{
    (void)arg;

    uart_config_t cfg = {
        .baud_rate  = WEMOS_BRIDGE_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_param_config(WEMOS_BRIDGE_UART_PORT, &cfg);
    uart_set_pin(WEMOS_BRIDGE_UART_PORT,
                 WEMOS_BRIDGE_TX_PIN, WEMOS_BRIDGE_RX_PIN,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(WEMOS_BRIDGE_UART_PORT, 512, 0, 0, NULL, 0);

    printf("[TEST_BRIDGE_RX] Listening on UART%d  RX=GPIO%d  TX=GPIO%d  %d baud\n",
           (int)WEMOS_BRIDGE_UART_PORT,
           WEMOS_BRIDGE_RX_PIN, WEMOS_BRIDGE_TX_PIN,
           WEMOS_BRIDGE_BAUD);

    uint32_t frames_ok = 0, frames_err = 0;
    uint8_t b;

    while (1) {
        if (uart_read_bytes(WEMOS_BRIDGE_UART_PORT, &b, 1,
                            pdMS_TO_TICKS(500)) != 1) continue;
        if (b != 0xAAu) continue;
        if (uart_read_bytes(WEMOS_BRIDGE_UART_PORT, &b, 1,
                            pdMS_TO_TICKS(50)) != 1) continue;
        if (b != 0xBBu) continue;

        uint8_t msg_type;
        if (uart_read_bytes(WEMOS_BRIDGE_UART_PORT, &msg_type, 1,
                            pdMS_TO_TICKS(50)) != 1) continue;
        if (msg_type != 0x01u) { frames_err++; continue; }

        uint8_t payload_len;
        if (uart_read_bytes(WEMOS_BRIDGE_UART_PORT, &payload_len, 1,
                            pdMS_TO_TICKS(50)) != 1) continue;
        if (payload_len != sizeof(control_frame_t) || payload_len > 128u) {
            frames_err++;
            continue;
        }

        uint8_t buf[sizeof(control_frame_t) + 1];
        if (uart_read_bytes(WEMOS_BRIDGE_UART_PORT, buf,
                            (int)(payload_len + 1u),
                            pdMS_TO_TICKS(100)) != (int)(payload_len + 1u)) {
            frames_err++;
            continue;
        }

        uint8_t ck = msg_type ^ payload_len;
        for (int i = 0; i < (int)payload_len; i++) ck ^= buf[i];
        if (ck != buf[payload_len]) {
            frames_err++;
            continue;
        }

        frames_ok++;
        control_frame_t cmd;
        memcpy(&cmd, buf, sizeof(cmd));
        printf("[TEST_BRIDGE_RX] #%u  tx=%.0f mm  ty=%.0f mm  hdg=%.2f rad"
               "  spd=%.0f mm/s  (err=%u)\n",
               (unsigned)frames_ok,
               (double)cmd.tx, (double)cmd.ty,
               (double)cmd.t_heading, (double)cmd.t_speed,
               (unsigned)frames_err);
    }
}
#endif /* TEST_BRIDGE_RX */


/* ════════════════════════════════════════════════════════════════════════════
 * TEST_BRIDGE_PONG
 * ════════════════════════════════════════════════════════════════════════════ */
#ifdef TEST_BRIDGE_PONG
static void test_bridge_pong_task(void *arg)
{
    (void)arg;

    uart_config_t cfg = {
        .baud_rate  = WEMOS_BRIDGE_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_param_config(WEMOS_BRIDGE_UART_PORT, &cfg);
    uart_set_pin(WEMOS_BRIDGE_UART_PORT,
                 WEMOS_BRIDGE_TX_PIN, WEMOS_BRIDGE_RX_PIN,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(WEMOS_BRIDGE_UART_PORT, 512, 512, 0, NULL, 0);

    printf("[Wemos] Pong test on UART%d  RX=GPIO%d  TX=GPIO%d  %d baud\n",
           (int)WEMOS_BRIDGE_UART_PORT,
           WEMOS_BRIDGE_RX_PIN, WEMOS_BRIDGE_TX_PIN,
           WEMOS_BRIDGE_BAUD);

    uint32_t pong_n = 0;
    while (1) {
        size_t avail = 0;
        uart_get_buffered_data_len(WEMOS_BRIDGE_UART_PORT, &avail);
        if (avail > 0) {
            uint8_t rbuf[64] = {0};
            int got = uart_read_bytes(WEMOS_BRIDGE_UART_PORT, rbuf,
                                      avail < 63 ? (int)avail : 63, 0);
            if (got > 0) {
                if (rbuf[got - 1] == '\n') rbuf[got - 1] = '\0';
                printf("[Wemos] Received: %s\n", (char *)rbuf);
            }
        }

        int c = getchar();
        if (c != EOF && c != '\r' && c != '\n') {
            while (getchar() != EOF);
            pong_n++;
            char msg[32];
            int mlen = snprintf(msg, sizeof(msg), "hey2 #%u\n", (unsigned)pong_n);
            uart_write_bytes(WEMOS_BRIDGE_UART_PORT, msg, mlen);
            printf("[Wemos] Sent: hey2 #%u\n", (unsigned)pong_n);
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
#endif /* TEST_BRIDGE_PONG */


/* ════════════════════════════════════════════════════════════════════════════
 * BRIDGE_SLAVE — receive drive commands from ESP32-S3 and execute them.
 * ════════════════════════════════════════════════════════════════════════════ */
#ifdef BRIDGE_SLAVE
static void bridge_slave_task(void *arg)
{
    (void)arg;

    uart_bridge_init();
    printf("[SLAVE] Ready — waiting for drive commands from ESP32-S3...\n");

    pure_pursuit_controller_t pp;
    pp_init(&pp);

    pose_t local_pose = {
        .x = 0.0f,
        .y = 0.0f,
        .theta = 0.0f,
        .cov = {0}
    };

    for (;;) {
        control_frame_t cmd = {0};
        bool have_command = false;

        path_frame_t path_frame;

        if (uart_bridge_recv_path(&path_frame)) {
            printf("[SLAVE] path received: length=%u\n",
                   (unsigned)path_frame.length);

            pp_set_path(&pp, path_frame.waypoints, path_frame.length);

            waypoint_t final_wp = path_frame.waypoints[path_frame.length - 1];

            cmd.tx = final_wp.x - local_pose.x;
            cmd.ty = final_wp.y - local_pose.y;
            cmd.t_heading = final_wp.theta;
            cmd.t_speed = final_wp.v_target;

            if (cmd.t_speed <= 10.0f) cmd.t_speed = 150.0f;

            have_command = true;
        }

        if (!have_command && uart_bridge_recv_control(&cmd)) {
            printf("[SLAVE] cmd: hdg=%.2f rad  spd=%.0f mm/s  tx=%.0f  ty=%.0f\n",
                   (double)cmd.t_heading, (double)cmd.t_speed,
                   (double)cmd.tx, (double)cmd.ty);

            waypoint_t path[2];
            path[0].x = local_pose.x;
            path[0].y = local_pose.y;
            path[0].theta = local_pose.theta;
            path[0].v_target = cmd.t_speed;
            path[1].x = local_pose.x + cmd.tx;
            path[1].y = local_pose.y + cmd.ty;
            path[1].theta = cmd.t_heading;
            path[1].v_target = cmd.t_speed;

            pp_set_path(&pp, path, 2);
            have_command = true;
        }

        if (!have_command) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        pp_motion_command_t pp_cmd = pp_compute_command(&pp, &local_pose);

        printf("[PP] speed=%.0f mm/s  servo=%.1f deg  stop=%d\n",
               (double)pp_cmd.speed_mm_s,
               (double)pp_cmd.steering_deg,
               (int)pp_cmd.stop);

        if (pp_cmd.stop) {
            motors_stop();
            set_duty(SERVO_CH, SERVO_DUTY_CENTER);
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        uint32_t servo_duty = servo_deg_to_duty(pp_cmd.steering_deg);
        set_duty(SERVO_CH, servo_duty);
        vTaskDelay(pdMS_TO_TICKS(50));

        float dist_mm = sqrtf(cmd.tx * cmd.tx + cmd.ty * cmd.ty);
        if (dist_mm < 50.0f) dist_mm = DEBUG_FORWARD_MM;

        float speed = pp_cmd.speed_mm_s;
        if (cmd.t_speed > 10.0f) speed = cmd.t_speed;

        uint32_t drive_ms = 800;

        /* Use PID speed control in BRIDGE_SLAVE too (added) */
        motor_pid_set_target(speed / 1000.0f);
        set_duty(CH_FWD, MOTOR_DUTY_FWD);
        set_duty(CH_BWD, MOTOR_DUTY_STOP);

        float new_heading = imu_drive_and_track(local_pose.theta, drive_ms);

        motor_pid_stop();   /* added */
        motors_stop();

        /* Update local pose from odometry if available, else dead reckon */
        const odom_pose_t *odom = task_odometry_get_pose();
        if (odom != NULL) {
            local_pose.x     = odom->x * 1000.0f;
            local_pose.y     = odom->y * 1000.0f;
            local_pose.theta = odom->theta;
        } else {
            float moved_mm = speed * ((float)drive_ms / 1000.0f);
            local_pose.theta = new_heading;
            local_pose.x += moved_mm * cosf(local_pose.theta);
            local_pose.y += moved_mm * sinf(local_pose.theta);
        }

        printf("[PP] updated pose: x=%.0f y=%.0f theta=%.2f rad\n",
               (double)local_pose.x,
               (double)local_pose.y,
               (double)local_pose.theta);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
#endif /* BRIDGE_SLAVE */


/* ════════════════════════════════════════════════════════════════════════════
 * app_main — hardware init, map init, task creation
 * ════════════════════════════════════════════════════════════════════════════ */
void app_main(void)
{
#if defined(TEST_MOTOR)
    motor_init();
    motors_stop();
    set_duty(SERVO_CH, SERVO_DUTY_CENTER);
    xTaskCreate(test_motor_task, "t_motor", 2048, NULL, 3, NULL);

#elif defined(TEST_SERVO)
    motor_init();
    set_duty(SERVO_CH, SERVO_DUTY_CENTER);
    xTaskCreate(test_servo_task, "t_servo", 2048, NULL, 3, NULL);

#elif defined(TEST_IMU)
    xTaskCreate(test_imu_task, "t_imu", 3072, NULL, 3, NULL);

#elif defined(TEST_AS5600)
    xTaskCreate(test_as5600_task, "t_enc", 3072, NULL, 3, NULL);

#elif defined(TEST_BRIDGE_RX)
    xTaskCreate(test_bridge_rx_task, "t_brx", 3072, NULL, 3, NULL);

#elif defined(TEST_BRIDGE_PONG)
    xTaskCreate(test_bridge_pong_task, "t_pong", 3072, NULL, 3, NULL);

#elif defined(BRIDGE_SLAVE)
    motor_init();
    motors_stop();
    set_duty(SERVO_CH, SERVO_DUTY_CENTER);
    vTaskDelay(pdMS_TO_TICKS(500));
    imu_encoder_driver_init();   /* added — init encoder + IMU */
    xTaskCreate(bridge_slave_task, "slave", 4096, NULL, 3, NULL);
    xTaskCreate(task_odometry, "odom", 4096, NULL, 4, NULL);   /* added */

#elif defined(HEADING_TEST_MODE)
    motor_init();
    motors_stop();
    set_duty(SERVO_CH, SERVO_DUTY_CENTER);
    vTaskDelay(pdMS_TO_TICKS(500));
    imu_gyro_init();
    printf("[SLAMborghini] HEADING_TEST_MODE active — no LiDAR, no WiFi, no map.\n");
    xTaskCreate(heading_test_task, "htest", 4096, NULL, 3, NULL);

#else
    /* ── Normal exploration mode ────────────────────────────────────────── */
    motor_init();
    motors_stop();
    set_duty(SERVO_CH, SERVO_DUTY_CENTER);
    vTaskDelay(pdMS_TO_TICKS(500));
    imu_encoder_driver_init();   /* added — replaces imu_gyro_init() alone */
    lidar_driver_init();

    quadtree_map_init(&s_map, MAP_SIZE_MM, MAP_SIZE_MM, MAP_STEP_MM);

    s_pose = (pose_t){ .x = START_X_MM, .y = START_Y_MM, .theta = 0.0f };

    /* Seed a free disk so the frontier detector has an initial boundary */
    for (float ang = 0.0f; ang < 360.0f; ang += 5.0f) {
        float rad = ang * ((float)M_PI / 180.0f);
        for (float r = MAP_STEP_MM; r <= INIT_FREE_R_MM; r += MAP_STEP_MM) {
            quadtree_map_insert(&s_map,
                                s_pose.x + r * cosf(rad),
                                s_pose.y + r * sinf(rad),
                                CLASS_FREE);
        }
    }

    wifi_dashboard_init(WIFI_SSID, WIFI_PASSWORD);
    wifi_dashboard_log("IMU: ready");
    wifi_dashboard_log("LIDAR: started");
    wifi_dashboard_log("Map: initialized");
    imu_gyro_set_stop_check(wifi_dashboard_stop_peek);

    s_mtx = xSemaphoreCreateMutex();

    xTaskCreate(scan_task,     "scan",  4096, NULL, 5, NULL);
    xTaskCreate(plan_task,     "plan",  8192, NULL, 3, NULL);
    xTaskCreate(task_odometry, "odom",  4096, NULL, 4, NULL);   /* added */
#endif

    /* app_main returns — FreeRTOS scheduler keeps the tasks running */
}