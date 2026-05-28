/**
 * wemos/main.c
 *
 * Wemos D1 R32 — motion controller + odometry sender.
 *
 * Two-board architecture:
 *   ESP32-S3 (SLAM brain) ──UART──► Wemos D1 R32 (motion controller)
 *
 * Two-task architecture:
 *   task_odometry     priority 5, 100 Hz  — AS5600 + ICM-20948 fusion,
 *                                           publishes odom_pose_t
 *   task_pure_pursuit priority 4,  20 Hz  — receives path_chunk_t from
 *                                           ESP32-S3, runs PP controller,
 *                                           drives servo + motor, sends
 *                                           odom_t back to ESP32-S3
 *
 * Coordinate-frame note:
 *   The ESP32-S3 generates paths in its own world frame (origin at map
 *   bottom-centre, 5000 mm, 500 mm, heading π/2).  The Wemos odometry
 *   starts at (0, 0, 0).  On path receipt, waypoints are rigidly
 *   transformed so that wp[0] aligns with the current Wemos pose.
 */

#include "hardware_pins.h"
#include "imu_gyro.h"
#include "src/uart_bridge.h"
#include "src/pure_pursuit_controller.h"
#include "src/task_odometry.h"
#include "src/imu_encoder_driver.h"
#include "src/encoder_ackermann_odometry.h"
#include "src/hcsr04_driver.h"

#include "driver/ledc.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

#include <math.h>
#include <string.h>

static const char *TAG = "wemos_main";

/* ── Motor ────────────────────────────────────────────────────────────────── */
#define MOTOR_PWM_FREQ_HZ   25000     /* above hearing range — eliminates audible tone */
#define MOTOR_PWM_RES       LEDC_TIMER_8_BIT
#define MOTOR_DUTY_FWD      28        /* slow enough for PP to track corners cleanly */
#define CH_FWD              LEDC_CHANNEL_3
#define CH_BWD              LEDC_CHANNEL_2

/* ── Servo — calibrated values ────────────────────────────────────────────── */
#define SERVO_PWM_FREQ_HZ   50
#define SERVO_PWM_RES       LEDC_TIMER_16_BIT
#define SERVO_CH            LEDC_CHANNEL_0
#define SERVO_DUTY_CENTER   4700u     /* measured true straight */
#define SERVO_STEER_GAIN    1042.0f   /* LEDC counts per radian */
#define SERVO_MAX_STEER_RAD 1.134f    /* 65° — physical steering limit */
/* Derived from center ± (max_steer_rad × gain): 4700 ± (1.134 × 1042) ≈ 4700 ± 1182 */
#define SERVO_DUTY_LEFT     (SERVO_DUTY_CENTER - (uint32_t)(SERVO_MAX_STEER_RAD * SERVO_STEER_GAIN))
#define SERVO_DUTY_RIGHT    (SERVO_DUTY_CENTER + (uint32_t)(SERVO_MAX_STEER_RAD * SERVO_STEER_GAIN))

/* ── PP loop rate ─────────────────────────────────────────────────────────── */
#define PP_PERIOD_MS    50u   /* 20 Hz */

#define SONAR_STOP_MM          100u
#define SONAR_CLEAR_MM         180u
#define SONAR_HIT_STREAK         2u
#define SONAR_CLEAR_STREAK       5u
#define SONAR_REPORT_PERIOD_US 200000LL

/* ════════════════════════════════════════════════════════════════════════════
 * Actuator helpers
 * ════════════════════════════════════════════════════════════════════════════ */

static void motor_set(float speed_mm_s)
{
    if (speed_mm_s > 0.0f) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_FWD, (uint32_t)MOTOR_DUTY_FWD);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_BWD, 0u);
    } else if (speed_mm_s < -10.0f) {
        /* Reverse: drive CH_BWD; CH_FWD must be zero to avoid H-bridge shoot-through. */
        ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_FWD, 0u);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_BWD, (uint32_t)MOTOR_DUTY_FWD);
    } else {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_FWD, 0u);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_BWD, 0u);
    }
    ledc_update_duty(LEDC_LOW_SPEED_MODE, CH_FWD);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, CH_BWD);
}

static void servo_set_deg(float steering_deg)
{
    float rad = (steering_deg - 90.0f) * ((float)M_PI / 180.0f);
    int32_t duty = (int32_t)SERVO_DUTY_CENTER + (int32_t)(rad * SERVO_STEER_GAIN);
    if (duty < (int32_t)SERVO_DUTY_LEFT)  duty = (int32_t)SERVO_DUTY_LEFT;
    if (duty > (int32_t)SERVO_DUTY_RIGHT) duty = (int32_t)SERVO_DUTY_RIGHT;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, SERVO_CH, (uint32_t)duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, SERVO_CH);
}

static float wrap_rad(float a)
{
    a = fmodf(a, 2.0f * (float)M_PI);
    if (a >  (float)M_PI) a -= 2.0f * (float)M_PI;
    if (a < -(float)M_PI) a += 2.0f * (float)M_PI;
    return a;
}

/* ════════════════════════════════════════════════════════════════════════════
 * task_pure_pursuit  —  priority 4, 20 Hz
 * ════════════════════════════════════════════════════════════════════════════ */

/* SE(2) transform state: aligns S3-world waypoints to Wemos local frame.
 * Computed once per path_id from the first chunk (start_index == 0).
 * Stored here (not inside the PP struct) so it survives between chunks. */
static uint16_t s_tf_path_id = 0xFFFFu;  /* UINT16_MAX = no valid transform */
static float    s_tf_c, s_tf_sv;          /* cos(dth), sin(dth) */
static float    s_tf_dth = 0.0f;          /* heading offset: Wemos_theta - S3_theta */
static float    s_tf_base_x, s_tf_base_y, s_tf_base_th;
static float    s_tf_wx, s_tf_wy, s_tf_wth;

static void apply_transform_to_chunk(path_chunk_t *chunk)
{
    for (uint8_t i = 0; i < chunk->count; i++) {
        float rx = chunk->wp[i].x - s_tf_base_x;
        float ry = chunk->wp[i].y - s_tf_base_y;
        chunk->wp[i].x     = (int16_t)lrintf(s_tf_wx + (s_tf_c * rx - s_tf_sv * ry));
        chunk->wp[i].y     = (int16_t)lrintf(s_tf_wy + (s_tf_sv * rx + s_tf_c  * ry));
        chunk->wp[i].theta = wrap_rad(chunk->wp[i].theta - s_tf_base_th + s_tf_wth);
    }
}

static void task_pure_pursuit(void *pvParameters)
{
    (void)pvParameters;

    pure_pursuit_controller_t pp;
    pp_init(&pp);

    float    prev_dist_m     = imu_encoder_driver_get_distance_m();
    float    prev_heading    = imu_encoder_driver_get_yaw_rad();
    int64_t  prev_us         = esp_timer_get_time();
    uint32_t odom_seq        = 0;
    bool     path_active     = false;
    int64_t  override_last_us = 0;   /* timestamp of last valid LP override */
    control_frame_t last_override = {0};
    bool     front_hazard = false;
    bool     front_hazard_last_sent = false;
    uint8_t  front_hazard_hits = 0;
    uint8_t  front_hazard_clears = 0;
    uint8_t  front_hazard_seq = 0;
    int64_t  front_hazard_last_us = 0;
    uint16_t front_hazard_old_path_id = 0xFFFFu;
#define OVERRIDE_TIMEOUT_US 300000LL /* revert to PP if no override for 300 ms */

    for (;;) {
        int64_t now_us = esp_timer_get_time();
        float dt_s = (float)(now_us - prev_us) * 1e-6f;
        if (dt_s < 0.005f) dt_s = 0.005f;
        if (dt_s > 0.200f) dt_s = 0.200f;
        prev_us = now_us;

        /* ── 1. Accept streaming chunk from ESP32-S3 ────────────────────── */
        path_chunk_t chunk;
        if (uart_bridge_recv_path_chunk(&chunk)) {
            if (front_hazard && chunk.path_id == front_hazard_old_path_id) {
                ESP_LOGW(TAG, "discard stale path id=%u during sonar brake",
                         (unsigned)chunk.path_id);
            } else {
            /* First chunk of a new plan: capture SE(2) transform so that all
             * subsequent chunks of this plan are expressed in Wemos local frame.
             * The transform aligns wp[0] (S3's estimate of robot position) with
             * the current Wemos odometry pose. */
            if (chunk.start_index == 0 && chunk.path_id != s_tf_path_id) {
                odom_pose_t op;
                task_odometry_copy_pose(&op);
                s_tf_wx      = op.x * 1000.0f;
                s_tf_wy      = op.y * 1000.0f;
                s_tf_wth     = op.theta;
                s_tf_base_x  = chunk.wp[0].x;
                s_tf_base_y  = chunk.wp[0].y;
                s_tf_base_th = chunk.wp[0].theta;
                float dth    = wrap_rad(s_tf_wth - s_tf_base_th);
                s_tf_c       = cosf(dth);
                s_tf_sv      = sinf(dth);
                s_tf_dth     = dth;
                s_tf_path_id = chunk.path_id;
                path_active  = true;
                ESP_LOGI(TAG, "new path id=%u  local_start=(%.0f, %.0f)",
                         (unsigned)chunk.path_id,
                         (double)s_tf_wx, (double)s_tf_wy);
            }

            /* Apply stored transform and append to PP ring. */
            if (chunk.path_id == s_tf_path_id) {
                apply_transform_to_chunk(&chunk);
                if (!pp_append_chunk(&pp, &chunk)) {
                    /* Out-of-order chunk — tell ESP32-S3 what index we need. */
                    uart_bridge_send_chunk_nack(chunk.path_id,
                                                pp_get_expected_start_idx(&pp));
                    ESP_LOGW(TAG, "NACK path=%u expected=%u got=%u",
                             (unsigned)chunk.path_id,
                             (unsigned)pp_get_expected_start_idx(&pp),
                             (unsigned)chunk.start_index);
                }
            } else {
                /* chunk.path_id unknown (transform not yet computed because
                 * start_index != 0 arrived first) — NACK to force resend from 0. */
                uart_bridge_send_chunk_nack(chunk.path_id, 0u);
            }
            }
        }

        /* ── 2. Current fused pose (metres → mm) ───────────────────────── */
        odom_pose_t op;
        task_odometry_copy_pose(&op);
        pose_t pose = {
            .x     = op.x * 1000.0f,
            .y     = op.y * 1000.0f,
            .theta = op.theta,
        };

        uint16_t sonar_mm = 0u;
        bool sonar_valid = hcsr04_read_u16_mm(&sonar_mm);
        bool was_front_hazard = front_hazard;

        if (sonar_valid && sonar_mm < SONAR_STOP_MM) {
            if (front_hazard_hits < 255u) front_hazard_hits++;
            front_hazard_clears = 0;
            if (front_hazard_hits >= SONAR_HIT_STREAK)
                front_hazard = true;
        } else if (!sonar_valid || sonar_mm > SONAR_CLEAR_MM) {
            if (front_hazard_clears < 255u) front_hazard_clears++;
            front_hazard_hits = 0;
            if (front_hazard_clears >= SONAR_CLEAR_STREAK)
                front_hazard = false;
        }

        if (!was_front_hazard && front_hazard) {
            motor_set(0.0f);
            servo_set_deg(90.0f);
            task_odometry_set_steering_rad(0.0f);
            front_hazard_old_path_id = s_tf_path_id;
            path_active = false;
            pp_init(&pp);
            override_last_us = 0;
            ESP_LOGW(TAG, "front sonar brake latched at %u mm; old path discarded",
                     (unsigned)sonar_mm);
        } else if (was_front_hazard && !front_hazard) {
            if (s_tf_path_id == front_hazard_old_path_id) {
                path_active = false;
                pp_init(&pp);
                s_tf_path_id = 0xFFFFu;
            }
            ESP_LOGI(TAG, "front sonar brake cleared at %u mm",
                     (unsigned)sonar_mm);
        }

        bool send_front_hazard =
            (front_hazard != front_hazard_last_sent) ||
            (front_hazard && now_us - front_hazard_last_us >= SONAR_REPORT_PERIOD_US);
        if (send_front_hazard) {
            front_hazard_t hazard = {
                .distance_mm = sonar_valid ? sonar_mm : 0u,
                .active      = front_hazard ? 1u : 0u,
                .seq         = front_hazard_seq++,
            };
            uart_bridge_send_front_hazard(&hazard);
            front_hazard_last_sent = front_hazard;
            front_hazard_last_us   = now_us;
        }

        /* ── 3 & 4. Override check, then PP command ──────────────────────── *
         * IMPORTANT: pp_compute_command() must NOT be called during an       *
         * override.  That function advances pursuit_idx (and frees ring      *
         * slots) based on the robot's current position.  While the Wemos is  *
         * executing a reactive/escape manoeuvre the robot is off-path, so    *
         * pursuit_idx would skip forward to a later segment.  When the       *
         * override expires PP would then resume at the wrong waypoint,        *
         * silently skipping the section of the A* path the car never drove.  */

        float applied_steer_deg = 90.0f;

        control_frame_t lp_override;
        bool have_override = uart_bridge_recv_control_override(&lp_override);
        if (have_override) {
            last_override    = lp_override;
            override_last_us = now_us;
        }

        if (front_hazard) {
            memset(&last_override, 0, sizeof(last_override));
            override_last_us = 0;
        }

        bool override_active = have_override ||
                               (now_us - override_last_us < OVERRIDE_TIMEOUT_US);

        if (front_hazard) {
            /* HC-SR04 owns the car only as a hard emergency brake. While it is
             * latched, ignore local-planner drive/reverse overrides entirely. */
            motor_set(0.0f);
            servo_set_deg(90.0f);
            task_odometry_set_steering_rad(0.0f);
            applied_steer_deg = 90.0f;
        } else if (override_active) {
            lp_override = last_override;
            /* Apply local-planner command.
             * t_speed == 0 → full stop (LP_MODE_STOPPED / footprint occupied).
             * |heading_err| > 90° means the planner wants to reverse (ESCAPE). */
            if (lp_override.t_speed <= 0.0f) {
                motor_set(0.0f);
                servo_set_deg(90.0f);
                task_odometry_set_steering_rad(0.0f);
            } else {
                /* Convert S3 world-frame heading to Wemos local frame using the
                 * stored SE(2) angular offset (captured at last path receipt).
                 * Without this, the π/2 initial heading difference between frames
                 * makes forward reactive/escape commands trigger reverse. */
                float t_hdg = (s_tf_path_id != 0xFFFFu)
                              ? wrap_rad(lp_override.t_heading + s_tf_dth)
                              : lp_override.t_heading;
                float heading_err = wrap_rad(t_hdg - pose.theta);
                bool  is_reverse  = (heading_err >  (float)(M_PI / 2.0) ||
                                     heading_err < -(float)(M_PI / 2.0));
                float steer_rad;
                if (is_reverse) {
                    steer_rad = wrap_rad(heading_err - (float)M_PI);
                    if (steer_rad >  SERVO_MAX_STEER_RAD) steer_rad =  SERVO_MAX_STEER_RAD;
                    if (steer_rad < -SERVO_MAX_STEER_RAD) steer_rad = -SERVO_MAX_STEER_RAD;
                    ESP_LOGI(TAG, "REVERSE  t_hdg=%.0f° pose.th=%.0f° err=%.0f° spd=%.0f override_age=%lld us",
                             (double)(t_hdg * 180.0f / (float)M_PI),
                             (double)(pose.theta * 180.0f / (float)M_PI),
                             (double)(heading_err * 180.0f / (float)M_PI),
                             (double)lp_override.t_speed,
                             (long long)(now_us - override_last_us));
                    motor_set(-lp_override.t_speed);
                } else {
                    steer_rad = heading_err;
                    if (steer_rad >  SERVO_MAX_STEER_RAD) steer_rad =  SERVO_MAX_STEER_RAD;
                    if (steer_rad < -SERVO_MAX_STEER_RAD) steer_rad = -SERVO_MAX_STEER_RAD;
                    motor_set(lp_override.t_speed);
                }
                float steer_deg = 90.0f + steer_rad * (180.0f / (float)M_PI);
                applied_steer_deg = steer_deg;
                servo_set_deg(steer_deg);
                task_odometry_set_steering_rad(steer_rad);
            }
        } else if (path_active) {
            /* Only advance pursuit_idx when PP is actually in control */
            pp_motion_command_t cmd = pp_compute_command(&pp, &pose);
            if (cmd.stop) {
                motor_set(0.0f);
                servo_set_deg(90.0f);
                task_odometry_set_steering_rad(0.0f);
                uart_bridge_send_path_done();
                path_active = false;
                ESP_LOGI(TAG, "path complete  pose=(%.0f mm, %.0f mm, %.1f°)",
                         (double)pose.x, (double)pose.y,
                         (double)(pose.theta * 180.0f / (float)M_PI));
            } else {
                float steer_rad = (cmd.steering_deg - 90.0f) * ((float)M_PI / 180.0f);
                applied_steer_deg = cmd.steering_deg;
                servo_set_deg(cmd.steering_deg);
                motor_set(cmd.speed_mm_s);
                task_odometry_set_steering_rad(steer_rad);
            }
        } else {
            motor_set(0.0f);
            servo_set_deg(90.0f);
            task_odometry_set_steering_rad(0.0f);
        }

        /* ── 5. Send odom_t to ESP32-S3 ────────────────────────────────── */
        float cur_dist    = imu_encoder_driver_get_distance_m();
        float cur_heading = imu_encoder_driver_get_yaw_rad();

        float disp_mm  = (cur_dist - prev_dist_m) * 1000.0f;
        float dheading = wrap_rad(cur_heading - prev_heading);

        prev_dist_m  = cur_dist;
        prev_heading = cur_heading;

        odom_t pkt = {
            .linear_disp_mm   = disp_mm,
            .yaw_rate_imu     = dheading / dt_s,
            .dt_ms            = dt_s * 1000.0f,
            .seq              = odom_seq++,
            .consumed_wp_idx  = pp_get_consumed_idx(&pp),
            .consumed_path_id = pp_get_path_id(&pp),
        };
        uart_bridge_send_odom(&pkt);

        ESP_LOGD(TAG,
                 "odom: disp=%.2f mm  rate=%.3f r/s  steer=%.1f°  ring=%u  active=%d",
                 (double)pkt.linear_disp_mm, (double)pkt.yaw_rate_imu,
                 (double)applied_steer_deg,
                 (unsigned)pp_get_expected_start_idx(&pp), (int)path_active);

        vTaskDelay(pdMS_TO_TICKS(PP_PERIOD_MS));
    }
}

/* ════════════════════════════════════════════════════════════════════════════
 * Hardware init
 * ════════════════════════════════════════════════════════════════════════════ */

static void motor_ledc_init(void)
{
    ledc_timer_config_t tmr = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = MOTOR_PWM_RES,
        .timer_num       = LEDC_TIMER_1,
        .freq_hz         = MOTOR_PWM_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&tmr);

    ledc_channel_config_t fwd = {
        .gpio_num = MOTOR_F_PIN, .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel  = CH_FWD,      .timer_sel  = LEDC_TIMER_1,
        .duty = 0, .hpoint = 0,
    };
    ledc_channel_config(&fwd);

    ledc_channel_config_t bwd = {
        .gpio_num = MOTOR_B_PIN, .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel  = CH_BWD,      .timer_sel  = LEDC_TIMER_1,
        .duty = 0, .hpoint = 0,
    };
    ledc_channel_config(&bwd);
}

static void servo_ledc_init(void)
{
    ledc_timer_config_t tmr = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = SERVO_PWM_RES,
        .timer_num       = LEDC_TIMER_0,
        .freq_hz         = SERVO_PWM_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&tmr);

    ledc_channel_config_t ch = {
        .gpio_num = SERVO_PIN,  .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel  = SERVO_CH,   .timer_sel  = LEDC_TIMER_0,
        .duty = SERVO_DUTY_CENTER, .hpoint = 0,
    };
    ledc_channel_config(&ch);
}

/* ════════════════════════════════════════════════════════════════════════════
 * app_main
 * ════════════════════════════════════════════════════════════════════════════ */
void app_main(void)
{
    uart_bridge_init();

    if (!imu_gyro_init()) {
        ESP_LOGE(TAG, "IMU init failed — check I2C SDA=%d SCL=%d",
                 WEMOS_I2C_SDA, WEMOS_I2C_SCL);
        for (;;) vTaskDelay(pdMS_TO_TICKS(1000));
    }

    float bias = imu_gyro_calibrate_bias(300);
    ESP_LOGI(TAG, "gyro bias = %.5f rad/s", (double)bias);

    motor_ledc_init();
    servo_ledc_init();
    hcsr04_init(SONAR_B_TRIG_PIN, SONAR_B_ECHO_PIN);
    servo_set_deg(90.0f);

    xTaskCreate(task_odometry,     "odom", 4096, NULL, 5, NULL);
    xTaskCreate(task_pure_pursuit, "pp",   4096, NULL, 4, NULL);
}
