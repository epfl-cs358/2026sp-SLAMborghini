/**
 * test_motor_pid.c
 * Tests for the motor_pid module (PID speed controller).
 * Board: PC (compiled with gcc -DUSE_STUBS)
 * Run: gcc -DUSE_STUBS test_motor_pid.c ../src/motor_pid.c -lm -o test_motor_pid
 */

#include <stdio.h>
#include <math.h>
#include "../../types.h"
#include "../src/motor_pid.h"

/* Tolerance for "approximately zero" output */
#define OUT_EPS 1e-4f

/* ------------------------------------------------------------------ */
/* Test 1: zero error gives zero (or near-zero) output                 */
/* ------------------------------------------------------------------ */
static int test_zero_error_zero_output(void)
{
    printf("Test 1: zero error (setpoint == measured) gives output ~= 0 ... ");

    pid_state_t pid;
    motor_pid_init(&pid, 1.5f, 0.05f, 0.01f);

    float output = motor_pid_step(&pid, 100.0f, 100.0f, 100.0f);

    if (fabsf(output) <= OUT_EPS) {
        printf("PASS (output=%.6f)\n", output);
        return 1;
    } else {
        printf("FAIL (output=%.6f, expected ~0)\n", output);
        return 0;
    }
}

/* ------------------------------------------------------------------ */
/* Test 2: positive error gives positive output                        */
/* ------------------------------------------------------------------ */
static int test_positive_error_positive_output(void)
{
    printf("Test 2: positive error (setpoint > measured) gives output > 0 ... ");

    pid_state_t pid;
    motor_pid_init(&pid, 1.5f, 0.05f, 0.01f);

    /* setpoint=200, measured=100 => error=100 => output should be > 0 */
    float output = motor_pid_step(&pid, 200.0f, 100.0f, 100.0f);

    if (output > 0.0f) {
        printf("PASS (output=%.4f)\n", output);
        return 1;
    } else {
        printf("FAIL (output=%.6f, expected > 0 for positive error)\n", output);
        return 0;
    }
}

/* ------------------------------------------------------------------ */
/* main                                                                 */
/* ------------------------------------------------------------------ */
int main(void)
{
    printf("=== test_motor_pid ===\n");

    int pass  = 0;
    int total = 0;

    total++; pass += test_zero_error_zero_output();
    total++; pass += test_positive_error_positive_output();

    printf("Result: %d/%d PASS\n", pass, total);
    return (pass == total) ? 0 : 1;
}
