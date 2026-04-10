/**
 * motor_pid.c
 * Module: PID speed controller for left and right DC motors.
 * Board: Wemos D1 R32
 * Implementation phase: stub (PID update equation not yet implemented)
 */

#include "motor_pid.h"

void motor_pid_init(pid_state_t *pid, float kp, float ki, float kd)
{
    // TODO: implement
    // Set pid->kp, ki, kd from arguments.
    // Zero out pid->integral and pid->prev_error.
    (void)pid;
    (void)kp;
    (void)ki;
    (void)kd;
}

float motor_pid_step(pid_state_t *pid, float setpoint, float measured, float dt)
{
    // TODO: implement
    // error      = setpoint - measured
    // pid->integral += error * (dt / 1000.0f)
    // derivative = (error - pid->prev_error) / (dt / 1000.0f)
    // output     = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative
    // clamp output to [-1.0, 1.0]
    // pid->prev_error = error
    (void)pid;
    (void)setpoint;
    (void)measured;
    (void)dt;
    return 0;
}

void motor_pid_reset(pid_state_t *pid)
{
    // TODO: implement
    // Zero out pid->integral and pid->prev_error.
    (void)pid;
}
