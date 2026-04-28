/**
 * wemos/main.c
 * Board: Wemos D1 R32 (ESP32)  -  temporary standalone SLAM brain + motor controller
 *
 * Normal two-board architecture:
 *   ESP32-S3 (SLAM brain) ──UART──► Wemos D1 R32 (motor controller)
 *
 * This file collapses both roles onto the Wemos for use when the ESP32-S3 is
 * unavailable.  The SLAM modules from esp32s3/src/ are compiled in unchanged.
 * Motor control is wired directly here  -  no UART relay.
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
 * Hardware pin parity with 2025fa-SLAMurai Wemos (same wiring as SLAMurai):
 *   Motor F/B   — GPIO 13 / 12, LEDC ch 3 / 2
 *   Servo       — GPIO 23
 *   LiDAR UART  — RX 17, TX 16, 460800 baud
 *   IMU I2C     — SDA 21, SCL 22
 */

/* ── Wi-Fi credentials ──────────────────────────────────────────────────── */
#define WIFI_SSID      "SPOT-iot"
#define WIFI_PASSWORD  "RacailleSalutaireMigration8052"

/* ── SLAM brain modules (from esp32s3/src/, compiled via CMakeLists.txt) ── */
#include "quadtree_map.h"
#include "frontier_detector.h"
#include "command_gen.h"
#include "wifi_dashboard.h"
#include "lidar_driver.h"
#include "lidar_to_map.h"
#include "imu_gyro.h"
#include "motor_pid.h"           /* added — motor PID speed control */
#include "imu_encoder_driver.h"  /* added — speed measurement for PID */

/* ── ESP-IDF / FreeRTOS ─────────────────────────────────────────────────── */
#include "driver/ledc.h"
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
#define SCAN_MAX_MM   2000.0f
#define SCAN_STEP_MM   150.0f

/* ── Planning loop constants ─────────────────────────────────────────────── */
#define DEBUG_FORWARD_MM    200.0f
#define DEBUG_SPEED_MM_S    100.0f
#define MAX_DRIVE_MS        1200u
#define CYCLE_DELAY_MS      50

#define TURN_ARC_THRESH_RAD  0.785f
#define TURN_ARC_MS          800u

/* ════════════════════════════════════════════════════════════════════════════
 * Motor config
 * ════════════════════════════════════════════════════════════════════════════ */
#define MOTOR_F_PIN        13
#define MOTOR_B_PIN        12
#define MOTOR_PWM_FREQ_HZ  1000
#define MOTOR_PWM_RES      LEDC_TIMER_8_BIT
#define MOTOR_DUTY_FWD     35
#define MOTOR_DUTY_STOP    0
#define MOTOR_SPEED_MM_S   346.0f
#define CH_FWD   LEDC_CHANNEL_3
#define CH_BWD   LEDC_CHANNEL_2

/* ════════════════════════════════════════════════════════════════════════════
 * Servo config
 * ════════════════════════════════════════════════════════════════════════════ */
#define SERVO_PIN          23
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

    /* Start PID speed control for straight drive */
    motor_pid_set_target(MOTOR_SPEED_MM_S / 1000.0f);
    set_duty(CH_FWD, MOTOR_DUTY_FWD);
    set_duty(CH_BWD, MOTOR_DUTY_STOP);
    actual_heading = imu_drive_and_track(actual_heading, drive_ms);
    motor_pid_stop();
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
 * dead_reckon_pose
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


/* ════════════════════════════════════════════════════════════════════════════
 * scan_task  (priority 5)
 * ════════════════════════════════════════════════════════════════════════════ */
static void scan_task(void *arg)
{
    (void)arg;
    static lidar_scan_t scan;
    TickType_t last_scan_broadcast = 0;

    while (1) {
        if (!lidar_driver_read_scan(&scan) || scan.count <= 10) continue;

        xSemaphoreTake(s_mtx, portMAX_DELAY);
        pose_t local_pose = s_pose;
        xSemaphoreGive(s_mtx);

        xSemaphoreTake(s_mtx, portMAX_DELAY);
        lidar_to_map(&s_map, &scan, &local_pose, SCAN_MAX_MM, SCAN_STEP_MM);
        xSemaphoreGive(s_mtx);

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
                wifi_dashboard_log("STOP requested — press Start to resume");
                printf("[SLAMborghini] Stopped.\n");
                break;
            }

            pose_t local_pose;
            frontier_list_t fronts;

            xSemaphoreTake(s_mtx, portMAX_DELAY);
            local_pose = s_pose;
            fronts     = frontier_detector_detect(&s_map, &local_pose);
            xSemaphoreGive(s_mtx);

            wifi_dashboard_update(&s_map, &local_pose);
            wifi_dashboard_broadcast_quadtree(&s_map);

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

            float new_heading = drive_for_cmd(&cmd, local_pose.theta);

            xSemaphoreTake(s_mtx, portMAX_DELAY);
            dead_reckon_pose(&s_pose, &cmd);
            s_pose.theta = new_heading;
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


/* ════════════════════════════════════════════════════════════════════════════
 * app_main — hardware init, map init, task creation
 * ════════════════════════════════════════════════════════════════════════════ */
void app_main(void)
{
    /* ── Hardware init ──────────────────────────────────────────────────── */
    motor_init();
    motors_stop();
    set_duty(SERVO_CH, SERVO_DUTY_CENTER);
    vTaskDelay(pdMS_TO_TICKS(500));
    imu_gyro_init();
    imu_encoder_driver_init();   /* added — init encoder for PID speed measurement */
    lidar_driver_init();

    /* ── Map init ───────────────────────────────────────────────────────── */
    quadtree_map_init(&s_map, MAP_SIZE_MM, MAP_SIZE_MM, MAP_STEP_MM);

    s_pose = (pose_t){ .x = START_X_MM, .y = START_Y_MM, .theta = 0.0f };

    for (float ang = 0.0f; ang < 360.0f; ang += 5.0f) {
        float rad = ang * ((float)M_PI / 180.0f);
        for (float r = MAP_STEP_MM; r <= INIT_FREE_R_MM; r += MAP_STEP_MM) {
            quadtree_map_insert(&s_map,
                                s_pose.x + r * cosf(rad),
                                s_pose.y + r * sinf(rad),
                                CLASS_FREE);
        }
    }

    /* ── Dashboard ──────────────────────────────────────────────────────── */
    wifi_dashboard_init(WIFI_SSID, WIFI_PASSWORD);
    wifi_dashboard_log("IMU: ready");
    wifi_dashboard_log("LIDAR: started");
    wifi_dashboard_log("Map: initialized");
    imu_gyro_set_stop_check(wifi_dashboard_stop_peek);

    /* ── Synchronisation primitive ──────────────────────────────────────── */
    s_mtx = xSemaphoreCreateMutex();

    /* ── Launch tasks ───────────────────────────────────────────────────── */
    xTaskCreate(scan_task, "scan", 4096, NULL, 5, NULL);
    xTaskCreate(plan_task, "plan", 8192, NULL, 3, NULL);

    /* Motor PID task — Core 1, realtime priority */        /* added */
    xTaskCreatePinnedToCore(motor_pid_task, "motor_pid",
                            2048, NULL, 24, NULL, 1);
}