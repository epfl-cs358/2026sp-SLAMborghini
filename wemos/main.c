/**
 * wemos/main.c
 * Wemos D1 R32 application entry point — SLAMborghini debug phase.
 * Board: Wemos D1 R32 (Control Brain)
 *
 * Debug phase behaviour:
 *   - Wait for a control_frame_t from the ESP32-S3 over UART.
 *   - Compute travel distance from the target position in the frame.
 *   - Drive both motors straight forward for the computed duration.
 *   - Stop. Done.
 *
 * No PID, no EKF, no Stanley — those are implemented next.
 * Motor pins and PWM duty MUST be adjusted to match your schematic.
 *
 * Once motor_pid, ekf, and stanley_controller are implemented, replace the
 * debug motor block with the full control loop in the comment at the bottom.
 */

#include "src/imu_encoder_driver.h"
#include "src/ekf.h"
#include "src/stanley_controller.h"
#include "src/motor_pid.h"
#include "src/hcsr04_driver.h"
#include "src/distress_detection.h"
#include "src/uart_bridge.h"

#include "driver/ledc.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

/* ════════════════════════════════════════════════════════════════════════════
 * DEBUG MOTOR CONFIG
 * Adjust GPIO pin numbers and PWM duty to match your hardware schematic.
 * IBT-4 driver: RPWM = forward, LPWM = backward. Both EN pins held HIGH.
 * ════════════════════════════════════════════════════════════════════════════ */
#define MOTOR_L_RPWM_PIN    25    /* left motor  forward  PWM */
#define MOTOR_L_LPWM_PIN    26    /* left motor  backward PWM */
#define MOTOR_R_RPWM_PIN    32    /* right motor forward  PWM */
#define MOTOR_R_LPWM_PIN    33    /* right motor backward PWM */

#define MOTOR_PWM_FREQ_HZ   1000
#define MOTOR_PWM_RES       LEDC_TIMER_8_BIT   /* 0-255 */
#define MOTOR_DUTY_FWD      76    /* ~30% duty — slow, safe for first test    */
#define MOTOR_DUTY_STOP     0

/* ── LEDC channel assignments ───────────────────────────────────────────── */
#define CH_L_FWD    LEDC_CHANNEL_0
#define CH_L_BWD    LEDC_CHANNEL_1
#define CH_R_FWD    LEDC_CHANNEL_2
#define CH_R_BWD    LEDC_CHANNEL_3

/* ── Maximum safe travel duration (caps any miscalculated command) ─────── */
#define MAX_DRIVE_MS        3000u

/* ════════════════════════════════════════════════════════════════════════════
 * debug_motor_init
 * Configure four LEDC channels for IBT-4 motor driver PWM.
 * ════════════════════════════════════════════════════════════════════════════ */
static void debug_motor_init(void)
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
        { .gpio_num=MOTOR_L_RPWM_PIN, .speed_mode=LEDC_LOW_SPEED_MODE,
          .channel=CH_L_FWD, .timer_sel=LEDC_TIMER_0, .duty=0, .hpoint=0 },
        { .gpio_num=MOTOR_L_LPWM_PIN, .speed_mode=LEDC_LOW_SPEED_MODE,
          .channel=CH_L_BWD, .timer_sel=LEDC_TIMER_0, .duty=0, .hpoint=0 },
        { .gpio_num=MOTOR_R_RPWM_PIN, .speed_mode=LEDC_LOW_SPEED_MODE,
          .channel=CH_R_FWD, .timer_sel=LEDC_TIMER_0, .duty=0, .hpoint=0 },
        { .gpio_num=MOTOR_R_LPWM_PIN, .speed_mode=LEDC_LOW_SPEED_MODE,
          .channel=CH_R_BWD, .timer_sel=LEDC_TIMER_0, .duty=0, .hpoint=0 },
    };
    for (int i = 0; i < 4; i++) ledc_channel_config(&ch[i]);
}

/* ── Set PWM duty on a channel ──────────────────────────────────────────── */
static void set_duty(ledc_channel_t ch, uint32_t duty)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, ch, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, ch);
}

/* ── Stop all motors ────────────────────────────────────────────────────── */
static void motors_stop(void)
{
    set_duty(CH_L_FWD, MOTOR_DUTY_STOP);
    set_duty(CH_L_BWD, MOTOR_DUTY_STOP);
    set_duty(CH_R_FWD, MOTOR_DUTY_STOP);
    set_duty(CH_R_BWD, MOTOR_DUTY_STOP);
}

/* ════════════════════════════════════════════════════════════════════════════
 * debug_drive_forward
 * Run both motors forward at MOTOR_DUTY_FWD for duration_ms milliseconds,
 * then stop. Caps at MAX_DRIVE_MS for safety.
 * ════════════════════════════════════════════════════════════════════════════ */
static void debug_drive_forward(uint32_t duration_ms)
{
    if (duration_ms == 0)            return;
    if (duration_ms > MAX_DRIVE_MS)  duration_ms = MAX_DRIVE_MS;

    /* Backward channels stay at 0, forward channels get duty */
    set_duty(CH_L_FWD, MOTOR_DUTY_FWD);
    set_duty(CH_R_FWD, MOTOR_DUTY_FWD);

    vTaskDelay(pdMS_TO_TICKS(duration_ms));

    motors_stop();
}

/* ════════════════════════════════════════════════════════════════════════════
 * app_main
 * ════════════════════════════════════════════════════════════════════════════ */
void app_main(void)
{
    uart_bridge_init();
    debug_motor_init();
    motors_stop();   /* ensure motors are off at boot */

    while (1) {
        control_frame_t cmd;

        if (uart_bridge_recv_control(&cmd)) {
            /* ── Compute travel distance from the target position ─────────
             * The ESP32-S3 sends the target relative to the robot's origin.
             * dist = sqrt(tx² + ty²). For the 200 mm fallback: dist = 200 mm.
             * ──────────────────────────────────────────────────────────── */
            float dist_mm = sqrtf(cmd.tx * cmd.tx + cmd.ty * cmd.ty);

            /* Guard: minimum meaningful distance */
            if (dist_mm < 50.0f) dist_mm = 200.0f;

            float speed = (cmd.t_speed > 10.0f) ? cmd.t_speed : 100.0f;

            /* duration = distance / speed  (both in mm and mm/s → result in s) */
            uint32_t duration_ms = (uint32_t)((dist_mm / speed) * 1000.0f);

            debug_drive_forward(duration_ms);

            /* Command executed — wait for the next one */
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    /* ════════════════════════════════════════════════════════════════════════
     * Full control loop (uncomment once all modules are implemented):
     *
     * while (1) {
     *     odom_t odom = imu_encoder_read();
     *     ekf_predict(&ekf, &odom);
     *     uart_bridge_send_odom(&odom);
     *
     *     control_frame_t cmd;
     *     if (uart_bridge_recv_control(&cmd)) {
     *         pose_t pose = ekf_get_pose(&ekf);
     *         waypoint_t wp = { cmd.tx, cmd.ty, cmd.t_heading, cmd.t_speed };
     *
     *         float steer      = stanley_compute_steer(&pose, &wp, cmd.t_speed);
     *         float left_pwm   = motor_pid_step(&pid_left,  cmd.t_speed,
     *                                            odom.linear_disp_mm, odom.dt_ms);
     *         float right_pwm  = motor_pid_step(&pid_right, cmd.t_speed,
     *                                            odom.linear_disp_mm, odom.dt_ms);
     *         // apply steer + pwm to motors
     *     }
     *
     *     float dist_mm = hcsr04_read_mm();
     *     if (distress_detect(dist_mm, 0.0f)) { /* alert ESP32-S3 */ }
     * }
     * ════════════════════════════════════════════════════════════════════ */
}
