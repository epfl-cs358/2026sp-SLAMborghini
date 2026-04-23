/**
 * wemos/main.c
 * Board: Wemos D1 R32 (ESP32) — temporary standalone SLAM brain + motor controller
 *
 * Normal two-board architecture:
 *   ESP32-S3 (SLAM brain) ──UART──► Wemos D1 R32 (motor controller)
 *
 * This file collapses both roles onto the Wemos for use when the ESP32-S3 is
 * unavailable.  The SLAM modules from esp32s3/src/ are compiled in unchanged.
 * Motor control is wired directly here — no UART relay.
 *
 * What changed vs the two-board setup:
 *   - uart_bridge_init() / uart_bridge_send_control() removed
 *   - drive_for_cmd() replaces uart_bridge_send_control()
 *   - Motor init / PWM identical to the original wemos/main.c
 *   - Everything else (frontier detection, WiFi, WebSocket, odometry)
 *     is identical to esp32s3/main.c
 *
 * When the ESP32-S3 is back:
 *   Flash esp32s3/main.c → ESP32-S3  and  the original wemos/main.c → Wemos.
 *   No module files need to change.
 *
 * Adjust WIFI_SSID / WIFI_PASSWORD and motor GPIO pins before flashing.
 */

/* ── Mode flag ──────────────────────────────────────────────────────────── */
#define USE_FRONTIER_TARGET  1

/* ── Wi-Fi credentials ──────────────────────────────────────────────────── */
#define WIFI_SSID      "SPOT-iot"
#define WIFI_PASSWORD  "RacailleSalutaireMigration8052"

/* ── SLAM brain modules ─────────────────────────────────────────────────── */
#include "quadtree_map.h"
#include "frontier_detector.h"
#include "command_gen.h"
#include "wifi_dashboard.h"
#include "test/test_room.h"
#include "test/room_data.h"
#include "test/simulate_lidar.h"
#include "imu_gyro.h"
#include "task_odometry.h"
#include "imu_encoder_driver.h"

/* ── ESP-IDF / FreeRTOS ─────────────────────────────────────────────────── */
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <stdbool.h>

/* ── Planning loop constants ────────────────────────────────────────────── */
#define DEBUG_FORWARD_MM    200.0f
#define DEBUG_SPEED_MM_S    100.0f
#define MAX_DRIVE_MS        1200u
#define CYCLE_DELAY_MS      50

#define TURN_ARC_THRESH_RAD  0.785f  /* pi/4 — beyond this, arc before main drive */
#define TURN_ARC_MS          800u

/* ── Motor config ───────────────────────────────────────────────────────── */
#define MOTOR_F_PIN        13
#define MOTOR_B_PIN        12
#define MOTOR_PWM_FREQ_HZ  1000
#define MOTOR_PWM_RES      LEDC_TIMER_8_BIT
#define MOTOR_DUTY_FWD     25
#define MOTOR_DUTY_STOP    0
#define CH_FWD             LEDC_CHANNEL_3
#define CH_BWD             LEDC_CHANNEL_2

/* ── Servo config ───────────────────────────────────────────────────────── */
#define SERVO_PIN          23
#define SERVO_PWM_FREQ_HZ  50
#define SERVO_PWM_RES      LEDC_TIMER_16_BIT
#define SERVO_CH           LEDC_CHANNEL_0
#define SERVO_DUTY_CENTER  4987u
#define SERVO_DUTY_LEFT    4442u
#define SERVO_DUTY_RIGHT   5533u
#define SERVO_STEER_GAIN   1042.0f

/* ── Frontier lock thresholds ───────────────────────────────────────────── */
#define FRONTIER_REACHED_MM   500.0f
#define FRONTIER_STALE_MM    1500.0f

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

    /* Separate timer for servo — avoids PWM freq conflict with motors */
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
 * drive_for_cmd
 * Steers toward cmd->t_heading then drives forward for the commanded distance.
 * Odometry is handled by task_odometry running in parallel — no pose update here.
 * ════════════════════════════════════════════════════════════════════════════ */
static void drive_for_cmd(const control_frame_t *cmd, float current_heading)
{
    /* Compute heading error */
    float err = cmd->t_heading - current_heading;
    while (err >  (float)M_PI) err -= 2.0f * (float)M_PI;
    while (err < -(float)M_PI) err += 2.0f * (float)M_PI;

    /* Set servo */
    int32_t duty = (int32_t)SERVO_DUTY_CENTER + (int32_t)(err * SERVO_STEER_GAIN);
    if (duty < (int32_t)SERVO_DUTY_LEFT)  duty = (int32_t)SERVO_DUTY_LEFT;
    if (duty > (int32_t)SERVO_DUTY_RIGHT) duty = (int32_t)SERVO_DUTY_RIGHT;
    set_duty(SERVO_CH, (uint32_t)duty);
    vTaskDelay(pdMS_TO_TICKS(50));  /* let servo reach position */

    /* Pre-drive arc if heading error > 45 deg */
    if (fabsf(err) > TURN_ARC_THRESH_RAD) {
        set_duty(CH_FWD, MOTOR_DUTY_FWD);
        set_duty(CH_BWD, MOTOR_DUTY_STOP);
        vTaskDelay(pdMS_TO_TICKS(TURN_ARC_MS));
        motors_stop();

        /* Recompute servo after arc — task_odometry updated theta already */
        const odom_pose_t *odom = task_odometry_get_pose();
        float heading_after_arc = (odom != NULL) ? odom->theta : current_heading;

        float err2 = cmd->t_heading - heading_after_arc;
        while (err2 >  (float)M_PI) err2 -= 2.0f * (float)M_PI;
        while (err2 < -(float)M_PI) err2 += 2.0f * (float)M_PI;

        int32_t duty2 = (int32_t)SERVO_DUTY_CENTER + (int32_t)(err2 * SERVO_STEER_GAIN);
        if (duty2 < (int32_t)SERVO_DUTY_LEFT)  duty2 = (int32_t)SERVO_DUTY_LEFT;
        if (duty2 > (int32_t)SERVO_DUTY_RIGHT) duty2 = (int32_t)SERVO_DUTY_RIGHT;
        set_duty(SERVO_CH, (uint32_t)duty2);
        vTaskDelay(pdMS_TO_TICKS(30));
    }

    /* Main drive */
    float dist_mm = sqrtf(cmd->tx * cmd->tx + cmd->ty * cmd->ty);
    if (dist_mm < 50.0f) dist_mm = DEBUG_FORWARD_MM;

    float    speed    = (cmd->t_speed > 10.0f) ? cmd->t_speed : DEBUG_SPEED_MM_S;
    uint32_t drive_ms = (uint32_t)((dist_mm / speed) * 1000.0f);
    if (drive_ms > MAX_DRIVE_MS) drive_ms = MAX_DRIVE_MS;
    if (drive_ms < 50)           drive_ms = 50;

    set_duty(CH_FWD, MOTOR_DUTY_FWD);
    set_duty(CH_BWD, MOTOR_DUTY_STOP);
    vTaskDelay(pdMS_TO_TICKS(drive_ms));
    motors_stop();
}

/* ════════════════════════════════════════════════════════════════════════════
 * app_main
 * ════════════════════════════════════════════════════════════════════════════ */
void app_main(void)
{
    /* Init motors and let power rails settle */
    motor_init();
    motors_stop();
    vTaskDelay(pdMS_TO_TICKS(500));

    /* Init IMU + encoder driver */
    imu_encoder_driver_init();

    /* Stop-check callback for emergency stop during drives */
    imu_gyro_set_stop_check(wifi_dashboard_stop_peek);

    /* Start odometry task — runs at 100 Hz, updates pose independently */
    xTaskCreate(task_odometry, "task_odometry", 4096, NULL, 5, NULL);

    /* Allocate maps before Wi-Fi — malloc needs heap before Wi-Fi claims it */
    quadtree_map_t truth_map;
    quadtree_map_t slam_map;
    pose_t pose = {0};

    build_test_room(&truth_map, &pose);
    slam_map_init(&slam_map, &pose,
                  ROOM_WIDTH_MM, ROOM_HEIGHT_MM,
                  50.0f, BUILD_FREE_DISK_MM);

    /* Wi-Fi + WebSocket dashboard */
    wifi_dashboard_init(WIFI_SSID, WIFI_PASSWORD);

    /* Initial LiDAR scan so dashboard shows the room immediately */
    simulate_and_update_map(&truth_map, &slam_map, &pose, 180, 6000.0f, 50.0f);

    /* Wait for "Start Exploration" from browser */
    while (!wifi_dashboard_exploration_requested()) {
        wifi_dashboard_broadcast_state(&pose, 0.0f, 0.0f, false, 0);
        wifi_dashboard_update(&slam_map, &pose);
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    float locked_fx = 0.0f, locked_fy = 0.0f;
    bool  have_lock  = false;

    /* ── Planning loop ───────────────────────────────────────────────────── */
    while (1) {

        /* Emergency stop */
        if (wifi_dashboard_stop_requested()) {
            motors_stop();
            set_duty(SERVO_CH, SERVO_DUTY_CENTER);
            have_lock = false;
            while (!wifi_dashboard_exploration_requested()) {
                wifi_dashboard_broadcast_state(&pose, 0.0f, 0.0f, false, 0);
                vTaskDelay(pdMS_TO_TICKS(500));
            }
        }

        /* Get latest pose from odometry task */
        const odom_pose_t *odom = task_odometry_get_pose();
        if (odom != NULL) {
            pose.x     = odom->x;
            pose.y     = odom->y;
            pose.theta = odom->theta;
        }

        frontier_list_t frontiers = frontier_detector_detect(&slam_map, &pose);

        control_frame_t cmd = {0};
        bool  has_frontier  = false;
        float fx = 0.0f, fy = 0.0f;

#if USE_FRONTIER_TARGET
        if (frontiers.count > 0) {
            if (have_lock) {
                float dx   = locked_fx - pose.x;
                float dy   = locked_fy - pose.y;
                float dist = sqrtf(dx*dx + dy*dy);

                if (dist < FRONTIER_REACHED_MM) have_lock = false;

                if (have_lock) {
                    bool still_exists = false;
                    for (uint8_t i = 0; i < frontiers.count; i++) {
                        float ex = frontiers.items[i].cx - locked_fx;
                        float ey = frontiers.items[i].cy - locked_fy;
                        if (sqrtf(ex*ex + ey*ey) < FRONTIER_STALE_MM) {
                            still_exists = true;
                            break;
                        }
                    }
                    if (!still_exists) have_lock = false;
                }
            }

            if (!have_lock) {
                frontier_t best = frontier_detector_best(&frontiers, &pose);
                locked_fx = best.cx;
                locked_fy = best.cy;
                have_lock = true;
            }

            waypoint_t wp = { .x = locked_fx, .y = locked_fy };
            cmd           = command_gen_compute(&pose, &wp);
            has_frontier  = true;
            fx            = locked_fx;
            fy            = locked_fy;
        } else {
            have_lock     = false;
            cmd.tx        = 0.0f;
            cmd.ty        = DEBUG_FORWARD_MM;
            cmd.t_heading = pose.theta;
            cmd.t_speed   = DEBUG_SPEED_MM_S;
        }
#else
        (void)frontiers;
        cmd.tx        = 0.0f;
        cmd.ty        = DEBUG_FORWARD_MM;
        cmd.t_heading = pose.theta;
        cmd.t_speed   = DEBUG_SPEED_MM_S;
#endif

        wifi_dashboard_broadcast_state(&pose, fx, fy, has_frontier, 0);

        /* Drive — pose update is handled by task_odometry in parallel */
        drive_for_cmd(&cmd, pose.theta);

        /* Simulate LiDAR at new pose, update slam_map */
        simulate_and_update_map(&truth_map, &slam_map, &pose, 180, 6000.0f, 50.0f);

        wifi_dashboard_update(&slam_map, &pose);

        vTaskDelay(pdMS_TO_TICKS(CYCLE_DELAY_MS));
    }
}