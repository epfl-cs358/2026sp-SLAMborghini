/**
 * motor_pid.h
 * Module: PID speed controller for left and right DC motors.
 * Board: Wemos D1 R32
 * Computes PWM duty cycle corrections for each motor independently,
 * based on the difference between a target speed and measured encoder speed.
 */

#ifndef MOTOR_PID_H
#define MOTOR_PID_H

#include "../../types.h"

/** PID controller state for a single motor channel. */
typedef struct {
    float kp;         /**< Proportional gain */
    float ki;         /**< Integral gain */
    float kd;         /**< Derivative gain */
    float integral;   /**< Accumulated integral term */
    float prev_error; /**< Error from the previous step (for derivative) */
} pid_state_t;

/**
 * Initialize a PID state with the given gains. Sets integral and prev_error to zero.
 * @param pid Pointer to the pid_state_t to initialize.
 * @param kp  Proportional gain.
 * @param ki  Integral gain.
 * @param kd  Derivative gain.
 */
void motor_pid_init(pid_state_t *pid, float kp, float ki, float kd);

/**
 * Compute one PID step and return the PWM duty cycle output.
 * @param pid      Pointer to the PID state (modified in place).
 * @param setpoint Desired speed (mm/s or encoder ticks/s).
 * @param measured Measured speed (same units as setpoint).
 * @param dt       Time step in milliseconds since the last call.
 * @return PWM duty cycle in range [-1.0, 1.0] (negative = reverse).
 */
float motor_pid_step(pid_state_t *pid, float setpoint, float measured, float dt);

/**
 * Reset the integral accumulator and previous error to zero.
 * Call this after a stop or direction reversal to avoid integral windup.
 * @param pid Pointer to the PID state to reset.
 */
void motor_pid_reset(pid_state_t *pid);

#endif /* MOTOR_PID_H */
