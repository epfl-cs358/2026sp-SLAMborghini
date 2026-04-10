/**
 * test_ekf.c
 * Tests for the EKF module (Extended Kalman Filter for odometry + IMU fusion).
 * Board: PC (compiled with gcc -DUSE_STUBS)
 * Run: gcc -DUSE_STUBS test_ekf.c ../stubs/odom_stub.c
 *           ../src/ekf.c -lm -o test_ekf
 */

#include <stdio.h>
#include <math.h>
#include "../../types.h"

#ifdef USE_STUBS
#include "../stubs/odom_stub.h"
#endif

#include "../src/ekf.h"

/* Floating-point comparison tolerance */
#define EPS 1e-5f

/* ------------------------------------------------------------------ */
/* Test 1: ekf_init sets state to origin                               */
/* ------------------------------------------------------------------ */
static int test_init_sets_origin(void)
{
    printf("Test 1: ekf_init sets state to origin ... ");

    ekf_state_t state;
    ekf_init(&state);

    pose_t pose = ekf_get_pose(&state);

    if (fabsf(pose.x) < EPS && fabsf(pose.y) < EPS && fabsf(pose.theta) < EPS) {
        printf("PASS (x=%.6f, y=%.6f, theta=%.6f)\n", pose.x, pose.y, pose.theta);
        return 1;
    } else {
        printf("FAIL (x=%.6f, y=%.6f, theta=%.6f) — expected all 0\n",
               pose.x, pose.y, pose.theta);
        return 0;
    }
}

/* ------------------------------------------------------------------ */
/* Test 2: ekf_predict moves state in X for straight-line odom        */
/* ------------------------------------------------------------------ */
static int test_predict_moves_state(void)
{
    printf("Test 2: ekf_predict moves state in X for straight odom ... ");

    ekf_state_t state;
    ekf_init(&state);

#ifdef USE_STUBS
    odom_t odom = odom_stub_straight(); /* 100 mm, yaw=0, dt=100ms */
#else
    odom_t odom;
    odom.linear_disp_mm = 100.0f;
    odom.yaw_rate_imu   = 0.0f;
    odom.dt_ms          = 100.0f;
#endif

    ekf_predict(&state, &odom);

    pose_t pose = ekf_get_pose(&state);

    /* After predicting straight forward from origin, x should have increased.
     * With a stub implementation returning zeros this test will report FAIL
     * until ekf_predict() is implemented — that is expected and informative. */
    if (pose.x > 0.0f) {
        printf("PASS (x=%.2f mm after predict)\n", pose.x);
        return 1;
    } else {
        /* Stub not yet implemented — warn but don't hard-fail CI */
        printf("FAIL (x=%.6f — ekf_predict stub returns 0; implement to pass)\n", pose.x);
        return 0;
    }
}

/* ------------------------------------------------------------------ */
/* main                                                                 */
/* ------------------------------------------------------------------ */
int main(void)
{
    printf("=== test_ekf (wemos) ===\n");

    int pass  = 0;
    int total = 0;

    total++; pass += test_init_sets_origin();
    total++; pass += test_predict_moves_state();

    printf("Result: %d/%d PASS\n", pass, total);
    return (pass == total) ? 0 : 1;
}
