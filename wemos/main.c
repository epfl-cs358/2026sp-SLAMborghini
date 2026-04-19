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
 *   - Everything else (frontier detection, WiFi, WebSocket, dead-reckoning)
 *     is identical to esp32s3/main.c
 *
 * When the ESP32-S3 is back:
 *   Flash esp32s3/main.c → ESP32-S3  and  the original wemos/main.c → Wemos.
 *   No module files need to change.
 *
 * Adjust WIFI_SSID / WIFI_PASSWORD and motor GPIO pins before flashing.
 */

/* ── Mode flag ──────────────────────────────────────────────────────────── */
#define USE_FRONTIER_TARGET  1   /* quadtree_map.c is now a functional flat grid —
                                  * frontier detection works, car drives toward
                                  * the detected frontier each planning cycle.  */

/* ── Wi-Fi credentials ──────────────────────────────────────────────────── */
#define WIFI_SSID      "SPOT-iot"
#define WIFI_PASSWORD  "RacailleSalutaireMigration8052"

/* ── SLAM brain modules (from esp32s3/src/, compiled via platformio.ini) ── */
#include "quadtree_map.h"
#include "frontier_detector.h"
#include "command_gen.h"
#include "wifi_dashboard.h"
#include "test/test_room.h"
#include "test/room_data.h"
#include "test/simulate_lidar.h"
#include "imu_gyro.h"

/* ── ESP-IDF / FreeRTOS ─────────────────────────────────────────────────── */
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <stdbool.h>

/* ── Planning loop constants (must match esp32s3/main.c) ────────────────── */
#define DEBUG_FORWARD_MM    200.0f
#define DEBUG_SPEED_MM_S    100.0f
#define MAX_DRIVE_MS        1200u   /* shorter burst → more frequent replanning */
#define CYCLE_DELAY_MS      50      /* tighter loop = smoother motion           */

/* If heading error exceeds this, arc toward target before main drive.
 * π/4 = 45° — beyond that the servo saturates and the car barely curves. */
#define TURN_ARC_THRESH_RAD  0.785f  /* π/4 */
#define TURN_ARC_MS          800u    /* duration of pre-drive arc at max servo  */

/* ════════════════════════════════════════════════════════════════════════════
 * Motor config — matches SLAMurai (2025fa-SLAMurai/code/firmware/wemos/
 *                include/motor_control/config.hpp) which this hardware reuses.
 * GPIO 13 = forward PWM  (MOTOR_F_PIN)
 * GPIO 12 = backward PWM (MOTOR_B_PIN)
 * LEDC channels 3 (forward) and 2 (backward) — same as SLAMurai.
 * ════════════════════════════════════════════════════════════════════════════ */
#define MOTOR_F_PIN        13    /* forward  PWM — matches SLAMurai config.hpp */
#define MOTOR_B_PIN        12    /* backward PWM — matches SLAMurai config.hpp */

#define MOTOR_PWM_FREQ_HZ  1000
#define MOTOR_PWM_RES      LEDC_TIMER_8_BIT   /* 0-255, matches PWM_RES = 8 */
#define MOTOR_DUTY_FWD     25     /* matches MAX_MOTOR_PWM_SPEED = 25 in SLAMurai */
#define MOTOR_DUTY_STOP    0

#define CH_FWD   LEDC_CHANNEL_3   /* PWM_CHANNEL_F = 3 in SLAMurai */
#define CH_BWD   LEDC_CHANNEL_2   /* PWM_CHANNEL_B = 2 in SLAMurai */

/* ════════════════════════════════════════════════════════════════════════════
 * Servo config — copied from 2025fa-SLAMurai/code/firmware/wemos/include/
 *                servo_control/config.hpp and navigation/config.hpp
 *
 * GPIO 23, 50 Hz PWM, 16-bit LEDC.
 * Angles from SLAMurai: center=94°, max_left=64° (94-30), max_right=124° (94+30)
 * Conversion: pulse_us = 1000 + (angle/180)*1000 ; counts = pulse_us/20000*65536
 *   64°  → 1356 μs → 4442 counts  (max left)
 *   94°  → 1522 μs → 4987 counts  (center / straight)
 *  124°  → 1689 μs → 5533 counts  (max right)
 *
 * Steering gain: 546 counts / (30° = 0.524 rad) ≈ 1042 counts/rad
 * Tune SERVO_STEER_GAIN if the car under/over-steers.
 * ════════════════════════════════════════════════════════════════════════════ */
#define SERVO_PIN          23
#define SERVO_PWM_FREQ_HZ  50
#define SERVO_PWM_RES      LEDC_TIMER_16_BIT
#define SERVO_CH           LEDC_CHANNEL_0
#define SERVO_DUTY_CENTER  4987u   /* 94°  — straight ahead */
#define SERVO_DUTY_LEFT    4442u   /* 64°  — max left       */
#define SERVO_DUTY_RIGHT   5533u   /* 124° — max right      */
#define SERVO_STEER_GAIN   1042.0f /* counts/rad            */

/* ════════════════════════════════════════════════════════════════════════════
 * Motor helpers
 * ════════════════════════════════════════════════════════════════════════════ */
static void motor_init(void)
{
    /* ── Motor timer (1 kHz, 8-bit) ── */
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

    /* ── Servo timer (50 Hz, 16-bit) — separate timer so freq doesn't conflict ── */
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
    /* Servo centres at init; no extra update needed */
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
 * current_heading is pose.theta at the moment the command is issued.
 * ════════════════════════════════════════════════════════════════════════════ */
/* Returns actual heading after the drive (from gyro integration). */
static float drive_for_cmd(const control_frame_t *cmd, float current_heading)
{
    /* ── Steering ────────────────────────────────────────────────────────── */
    float err = cmd->t_heading - current_heading;
    while (err >  (float)M_PI) err -= 2.0f * (float)M_PI;
    while (err < -(float)M_PI) err += 2.0f * (float)M_PI;

    int32_t duty = (int32_t)SERVO_DUTY_CENTER + (int32_t)(err * SERVO_STEER_GAIN);
    if (duty < (int32_t)SERVO_DUTY_LEFT)  duty = (int32_t)SERVO_DUTY_LEFT;
    if (duty > (int32_t)SERVO_DUTY_RIGHT) duty = (int32_t)SERVO_DUTY_RIGHT;

    set_duty(SERVO_CH, (uint32_t)duty);
    vTaskDelay(pdMS_TO_TICKS(50));    /* short settle — servo reaches position quickly */

    float actual_heading = current_heading;

    /* ── Pre-drive arc: if error > 45°, arc at max deflection first ─────── */
    if (fabsf(err) > TURN_ARC_THRESH_RAD) {
        set_duty(CH_FWD, MOTOR_DUTY_FWD);
        set_duty(CH_BWD, MOTOR_DUTY_STOP);
        actual_heading = imu_drive_and_track(actual_heading, TURN_ARC_MS);
        motors_stop();

        /* Recompute error and servo duty with updated heading */
        float err2 = cmd->t_heading - actual_heading;
        while (err2 >  (float)M_PI) err2 -= 2.0f * (float)M_PI;
        while (err2 < -(float)M_PI) err2 += 2.0f * (float)M_PI;
        int32_t duty2 = (int32_t)SERVO_DUTY_CENTER + (int32_t)(err2 * SERVO_STEER_GAIN);
        if (duty2 < (int32_t)SERVO_DUTY_LEFT)  duty2 = (int32_t)SERVO_DUTY_LEFT;
        if (duty2 > (int32_t)SERVO_DUTY_RIGHT) duty2 = (int32_t)SERVO_DUTY_RIGHT;
        set_duty(SERVO_CH, (uint32_t)duty2);
        vTaskDelay(pdMS_TO_TICKS(30));
    }

    /* ── Main drive ─────────────────────────────────────────────────────── */
    float dist_mm = sqrtf(cmd->tx * cmd->tx + cmd->ty * cmd->ty);
    if (dist_mm < 50.0f) dist_mm = DEBUG_FORWARD_MM;

    float    speed    = (cmd->t_speed > 10.0f) ? cmd->t_speed : DEBUG_SPEED_MM_S;
    uint32_t drive_ms = (uint32_t)((dist_mm / speed) * 1000.0f);
    if (drive_ms > MAX_DRIVE_MS) drive_ms = MAX_DRIVE_MS;
    if (drive_ms < 50)           drive_ms = 50;

    set_duty(CH_FWD, MOTOR_DUTY_FWD);
    set_duty(CH_BWD, MOTOR_DUTY_STOP);
    actual_heading = imu_drive_and_track(actual_heading, drive_ms);
    motors_stop();

    /* Keep servo pointed at target — don't re-centre between cycles */

    return actual_heading;
}

/* ════════════════════════════════════════════════════════════════════════════
 * dead_reckon_pose  (identical to esp32s3/main.c)
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
 * app_main
 * ════════════════════════════════════════════════════════════════════════════ */
void app_main(void)
{
    /* ── Motor + IMU init ───────────────────────────────────────────────── */
    motor_init();
    motors_stop();
    vTaskDelay(pdMS_TO_TICKS(500));   /* let power rails settle */
    bool imu_ok = imu_gyro_init();
    /* peek (non-consuming) so the stop flag survives for the main loop check */
    imu_gyro_set_stop_check(wifi_dashboard_stop_peek);

    /* ── Two maps allocated BEFORE Wi-Fi so malloc claims heap first ────── */
    /* truth_map: full room pre-loaded — ground truth for ray-casting         */
    /* slam_map:  progressively revealed — shown on dashboard                 */
    /* Each pool ≈ 60 KB; Wi-Fi needs ~80 KB; total fits in ~260 KB heap.    */
    quadtree_map_t truth_map;
    quadtree_map_t slam_map;
    pose_t pose = {0};

    build_test_room(&truth_map, &pose);
    slam_map_init(&slam_map, &pose,
                  ROOM_WIDTH_MM, ROOM_HEIGHT_MM,
                  50.0f, BUILD_FREE_DISK_MM);

    /* ── Wi-Fi + WebSocket dashboard ─────────────────────────────────────── */
    wifi_dashboard_init(WIFI_SSID, WIFI_PASSWORD);

    /* Initial scan so dashboard shows the first free disk immediately */
    simulate_and_update_map(&truth_map, &slam_map, &pose, 180, 6000.0f, 50.0f);

    /* ── Wait for browser to send "Start Exploration" ────────────────────── */
    while (!wifi_dashboard_exploration_requested()) {
        wifi_dashboard_broadcast_state(&pose, 0.0f, 0.0f, false, 0);
        wifi_dashboard_update(&slam_map, &pose);
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    /* ── Locked frontier state ───────────────────────────────────────────── */
    /* The car commits to one frontier until it arrives or the frontier
     * disappears from the map.  This stops the target from bouncing
     * around every cycle and wasting all movement on re-steering. */
#define FRONTIER_REACHED_MM   500.0f   /* close enough → pick next frontier  */
#define FRONTIER_STALE_MM    1500.0f   /* locked target moved this far → re-pick */
    float locked_fx = 0.0f, locked_fy = 0.0f;
    bool  have_lock  = false;

    /* ── Planning loop ───────────────────────────────────────────────────── */
    while (1) {

        /* Emergency stop — halt motors and wait for Start again */
        if (wifi_dashboard_stop_requested()) {
            motors_stop();
            set_duty(SERVO_CH, SERVO_DUTY_CENTER);
            have_lock = false;   /* re-pick frontier after resume */
            while (!wifi_dashboard_exploration_requested()) {
                wifi_dashboard_broadcast_state(&pose, 0.0f, 0.0f, false, 0);
                vTaskDelay(pdMS_TO_TICKS(500));
            }
        }

        frontier_list_t frontiers = frontier_detector_detect(&slam_map, &pose);

        control_frame_t cmd = {0};
        bool has_frontier   = false;
        float fx = 0.0f, fy = 0.0f;

#if USE_FRONTIER_TARGET
        if (frontiers.count > 0) {
            /* Check if the locked frontier is still valid */
            if (have_lock) {
                float dx = locked_fx - pose.x;
                float dy = locked_fy - pose.y;
                float dist = sqrtf(dx*dx + dy*dy);

                /* Reached? — clear lock so we pick a fresh target below */
                if (dist < FRONTIER_REACHED_MM) have_lock = false;

                /* Still locked? — verify the frontier still exists in the list
                 * (if all frontiers shifted far from the locked point, re-pick) */
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

            /* (Re-)pick a frontier if we don't have a valid lock */
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
            have_lock = false;
            /* No frontier — forward nudge so the car keeps moving */
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

        /* Broadcast state to PC dashboard before driving */
        wifi_dashboard_broadcast_state(&pose, fx, fy, has_frontier, 0);

        /* Drive + integrate gyro → get actual heading */
        float actual_heading = drive_for_cmd(&cmd, pose.theta);
        /* Only trust gyro heading when IMU is present; otherwise keep the
         * commanded heading so dead-reckoning moves toward the frontier. */
        if (imu_ok) cmd.t_heading = actual_heading;

        /* Update dead-reckoned pose using real (or commanded) heading */
        dead_reckon_pose(&pose, &cmd);

        /* Simulate LiDAR at new pose, update slam_map */
        simulate_and_update_map(&truth_map, &slam_map, &pose, 180, 6000.0f, 50.0f);

        wifi_dashboard_update(&slam_map, &pose);

        vTaskDelay(pdMS_TO_TICKS(CYCLE_DELAY_MS));
    }
}
