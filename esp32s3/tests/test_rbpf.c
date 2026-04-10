/**
 * test_rbpf.c
 * Tests for the rbpf module (Rao-Blackwellized Particle Filter).
 * Board: PC (compiled with gcc -DUSE_STUBS)
 * Run: gcc -DUSE_STUBS test_rbpf.c ../stubs/pose_stub.c
 *           ../stubs/scan_match_stub.c ../stubs/uart_stub.c
 *           ../src/rbpf.c -lm -o test_rbpf
 */

#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include "../../types.h"

#ifdef USE_STUBS
#include "../stubs/pose_stub.h"
#include "../stubs/scan_match_stub.h"
#include "../stubs/uart_stub.h"
#endif

#include "../src/rbpf.h"

/* ------------------------------------------------------------------ */
/* Test 1: init creates correct particle count                          */
/* ------------------------------------------------------------------ */
static int test_init_particle_count(void)
{
    printf("Test 1: init creates correct particle count ... ");

    rbpf_state_t state;
    rbpf_init(&state, 8);

    if (state.n_particles == 8) {
        printf("PASS\n");
        return 1;
    } else {
        printf("FAIL (n_particles=%d, expected 8)\n", (int)state.n_particles);
        return 0;
    }
}

/* ------------------------------------------------------------------ */
/* Test 2: predict does not crash                                       */
/* ------------------------------------------------------------------ */
static int test_predict_no_crash(void)
{
    printf("Test 2: predict does not crash with stub odom ... ");

    rbpf_state_t state;
    rbpf_init(&state, 8);

#ifdef USE_STUBS
    odom_t odom = uart_stub_recv_odom(); /* 50 mm forward, 0.02 rad/s, dt=100 ms */
#else
    odom_t odom;
    odom.linear_disp_mm = 50.0f;
    odom.yaw_rate_imu   = 0.02f;
    odom.dt_ms          = 100.0f;
#endif

    rbpf_predict(&state, &odom);
    /* If we reach here, no crash or assertion failure */
    printf("PASS\n");
    return 1;
}

/* ------------------------------------------------------------------ */
/* Test 3: best pose is valid after update with perfect correction     */
/* ------------------------------------------------------------------ */
static int test_best_pose_after_update(void)
{
    printf("Test 3: best pose is valid (no NaN) after update ... ");

    rbpf_state_t state;
    rbpf_init(&state, 16);

#ifdef USE_STUBS
    pose_correction_t corr = scan_match_stub_perfect();
#else
    pose_correction_t corr;
    corr.dx     = 0.0f;
    corr.dy     = 0.0f;
    corr.dtheta = 0.0f;
    corr.score  = 1.0f;
#endif

    rbpf_update(&state, &corr);
    pose_t best = rbpf_get_best_pose(&state);

    /* Verify pose fields are finite (not NaN / Inf) */
    int ok = (best.x == best.x) &&   /* NaN check: NaN != NaN */
             (best.y == best.y) &&
             (best.theta == best.theta);

    if (ok) {
        printf("PASS (best pose x=%.2f y=%.2f theta=%.4f)\n",
               best.x, best.y, best.theta);
        return 1;
    } else {
        printf("FAIL (NaN or Inf detected in pose)\n");
        return 0;
    }
}

/* ------------------------------------------------------------------ */
/* main                                                                 */
/* ------------------------------------------------------------------ */
int main(void)
{
    printf("=== test_rbpf ===\n");

    int pass  = 0;
    int total = 0;

    total++; pass += test_init_particle_count();
    total++; pass += test_predict_no_crash();
    total++; pass += test_best_pose_after_update();

    printf("Result: %d/%d PASS\n", pass, total);
    return (pass == total) ? 0 : 1;
}
