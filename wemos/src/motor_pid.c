#include "motor_pid.h"
#include <stdint.h>

static float s_integral = 0.0f;
static float s_prev_err = 0.0f;

void motor_pid_reset(void)
{
    s_integral = 0.0f;
    s_prev_err = 0.0f;
}

uint32_t motor_pid_update(float target_ms, float measured_ms, float dt_s)
{
    if (dt_s <= 0.0f) return MOTOR_DUTY_MIN;

    float err = target_ms - measured_ms;

    /* Integral with anti-windup */
    s_integral += err * dt_s;
    if (s_integral >  2.0f) s_integral =  2.0f;
    if (s_integral < -2.0f) s_integral = -2.0f;

    /* Derivative */
    float deriv = (err - s_prev_err) / dt_s;
    s_prev_err  = err;

    /* PID output — base duty + correction */
    float output = MOTOR_DUTY_FWD
                   + MOTOR_PID_KP * err
                   + MOTOR_PID_KI * s_integral
                   + MOTOR_PID_KD * deriv;

    /* Clamp */
    if (output < (float)MOTOR_DUTY_MIN) output = (float)MOTOR_DUTY_MIN;
    if (output > (float)MOTOR_DUTY_MAX) output = (float)MOTOR_DUTY_MAX;

    return (uint32_t)output;
}