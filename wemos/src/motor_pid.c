/**
 * motor_pid.c
 *
 * Closed-loop speed controller for the Wemos drive motor.  The PID task reads
 * measured wheel speed from the encoder driver and adjusts PWM duty so the
 * commanded speed is less sensitive to battery voltage and surface friction.
 */

#include "motor_pid.h"
#include "imu_encoder_driver.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"

/*
 * PID speed controller for the drive motor.
 *
 * The motor was previously driven at a fixed duty cycle (MOTOR_DUTY_FWD = 35),
 * which assumed the car always travels at exactly MOTOR_SPEED_MM_S = 346 mm/s.
 * In reality, speed varies with battery level, surface friction, and load.
 * This error propagates directly into dead_reckon_pose() — if the assumed speed
 * is wrong, the estimated position drifts from the real one every cycle.
 *
 * The PID closes the loop: it reads the actual speed from the AS5600 encoder
 * via imu_encoder_driver_get_speed_ms(), compares it to the target, and adjusts
 * the PWM duty cycle to keep the real speed close to the commanded value.
 */

static float s_integral  = 0.0f;
static float s_prev_err  = 0.0f;
static volatile float s_target_ms   = 0.0f;
static volatile bool  s_pid_running = false;

void motor_pid_reset(void)
{
    s_integral = 0.0f;
    s_prev_err = 0.0f;
}

/*
 * Anti-windup clamp on the integral term.
 *
 * Without this, if the motor is stalled or the encoder reads zero for several
 * cycles (e.g. at startup before the wheel starts moving), the integral
 * accumulates a very large value. When the motor finally spins up, the PID
 * output overshoots violently. Clamping to [-2, +2] bounds the worst-case
 * overshoot to a predictable range regardless of how long the motor was stalled.
 */
uint32_t motor_pid_update(float target_ms, float measured_ms, float dt_s)
{
    if (dt_s <= 0.0f) return MOTOR_DUTY_MIN;

    float err = target_ms - measured_ms;

    s_integral += err * dt_s;
    if (s_integral >  2.0f) s_integral =  2.0f;
    if (s_integral < -2.0f) s_integral = -2.0f;

    float deriv  = (err - s_prev_err) / dt_s;
    s_prev_err   = err;

    float output = 35.0f
                   + MOTOR_PID_KP * err
                   + MOTOR_PID_KI * s_integral
                   + MOTOR_PID_KD * deriv;

    if (output < (float)MOTOR_DUTY_MIN) output = (float)MOTOR_DUTY_MIN;
    if (output > (float)MOTOR_DUTY_MAX) output = (float)MOTOR_DUTY_MAX;

    return (uint32_t)output;
}

void motor_pid_set_target(float target_ms)
{
    s_target_ms   = target_ms;
    s_pid_running = (target_ms > 0.0f);
}

void motor_pid_stop(void)
{
    s_target_ms   = 0.0f;
    s_pid_running = false;
}

/*
 * Why a dedicated FreeRTOS task at 1 kHz instead of calling the PID inline?
 *
 * The original drive loop called imu_drive_and_track() which blocks for the
 * entire drive_ms duration — up to 1200 ms. Embedding the PID inside that
 * blocking call would tie it to the IMU polling interval (10 ms = 100 Hz),
 * which is too slow for stable speed regulation on a brushed DC motor.
 *
 * Running the PID as a pinned task on Core 1 at priority 24 gives it a
 * deterministic 1 ms period independent of what plan_task or scan_task are
 * doing on Core 0.
 */
void motor_pid_task(void *pvParameters)
{
    (void)pvParameters;
    motor_pid_reset();

    for (;;) {
        if (s_pid_running) {
            float measured = imu_encoder_driver_get_speed_ms();
            uint32_t duty  = motor_pid_update(s_target_ms, measured, 0.001f);

            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, duty);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3);
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
