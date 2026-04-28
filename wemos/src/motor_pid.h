#ifndef MOTOR_PID_H
#define MOTOR_PID_H

#include <stdint.h>

/* PID gains — tune experimentally */
#define MOTOR_PID_KP   8.0f
#define MOTOR_PID_KI   1.0f
#define MOTOR_PID_KD   0.05f

/* Duty cycle limits (8-bit PWM, 0-255) */
#define MOTOR_DUTY_MIN  10u
#define MOTOR_DUTY_MAX  60u

/* Reset internal PID state — call before each new drive */
void motor_pid_reset(void);

/* Compute corrected duty cycle.
 * target_ms  : desired speed in m/s
 * measured_ms: actual speed from AS5600 in m/s
 * dt_s       : time since last call in seconds
 * returns    : duty cycle to apply (clamped between MIN and MAX) */
uint32_t motor_pid_update(float target_ms, float measured_ms, float dt_s);

#endif