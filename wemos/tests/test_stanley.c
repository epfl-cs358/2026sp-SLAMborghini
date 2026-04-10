/**
 * test_stanley.c
 * Tests for the stanley_controller module (Stanley lateral controller).
 * Board: PC (compiled with gcc -DUSE_STUBS)
 * Run: gcc -DUSE_STUBS test_stanley.c ../stubs/waypoint_stub.c
 *           ../src/stanley_controller.c -lm -o test_stanley
 */

#include <stdio.h>
#include <math.h>
#include "../../types.h"

#ifdef USE_STUBS
#include "../stubs/waypoint_stub.h"
#endif

#include "../src/stanley_controller.h"

/* Tolerance for "approximately zero" steering angle */
#define STEER_EPS 0.05f  /* 0.05 rad ~ 2.9 degrees */

/* ------------------------------------------------------------------ */
/* Test 1: waypoint directly ahead gives near-zero steering            */
/* ------------------------------------------------------------------ */
static int test_ahead_waypoint_zero_steer(void)
{
    printf("Test 1: ahead waypoint gives steer ~= 0 ... ");

    pose_t pose;
    pose.x     = 0.0f;
    pose.y     = 0.0f;
    pose.theta = 0.0f;
    pose.cov[0] = pose.cov[1] = pose.cov[2] = 0.0f;
    pose.cov[3] = pose.cov[4] = pose.cov[5] = 0.0f;

    stanley_set_gain(2.0f);

#ifdef USE_STUBS
    waypoint_t wp = waypoint_stub_ahead(); /* (500, 0, 0, 300) */
#else
    waypoint_t wp;
    wp.x = 500.0f; wp.y = 0.0f; wp.theta = 0.0f; wp.v_target = 300.0f;
#endif

    float steer = stanley_compute_steer(&pose, &wp, wp.v_target);

    if (fabsf(steer) <= STEER_EPS) {
        printf("PASS (steer=%.4f rad)\n", steer);
        return 1;
    } else {
        printf("FAIL (steer=%.4f rad, expected |steer| <= %.4f)\n", steer, STEER_EPS);
        return 0;
    }
}

/* ------------------------------------------------------------------ */
/* Test 2: lateral offset waypoint gives nonzero steering              */
/* ------------------------------------------------------------------ */
static int test_lateral_offset_nonzero_steer(void)
{
    printf("Test 2: lateral offset waypoint gives nonzero steer ... ");

    pose_t pose;
    pose.x     = 0.0f;
    pose.y     = 0.0f;
    pose.theta = 0.0f;
    pose.cov[0] = pose.cov[1] = pose.cov[2] = 0.0f;
    pose.cov[3] = pose.cov[4] = pose.cov[5] = 0.0f;

    stanley_set_gain(2.0f);

#ifdef USE_STUBS
    waypoint_t wp = waypoint_stub_turn(); /* (0, 500, pi/2, 200) */
#else
    waypoint_t wp;
    wp.x = 0.0f; wp.y = 500.0f; wp.theta = 1.57f; wp.v_target = 200.0f;
#endif

    float steer = stanley_compute_steer(&pose, &wp, wp.v_target);

    if (fabsf(steer) > STEER_EPS) {
        printf("PASS (steer=%.4f rad — nonzero as expected)\n", steer);
        return 1;
    } else {
        printf("FAIL (steer=%.4f rad — expected |steer| > %.4f for lateral offset)\n",
               steer, STEER_EPS);
        return 0;
    }
}

/* ------------------------------------------------------------------ */
/* main                                                                 */
/* ------------------------------------------------------------------ */
int main(void)
{
    printf("=== test_stanley ===\n");

    int pass  = 0;
    int total = 0;

    total++; pass += test_ahead_waypoint_zero_steer();
    total++; pass += test_lateral_offset_nonzero_steer();

    printf("Result: %d/%d PASS\n", pass, total);
    return (pass == total) ? 0 : 1;
}
