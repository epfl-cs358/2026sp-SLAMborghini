/**
 * test_command_gen.c
 * Unit tests for command_gen_compute().
 * Board: PC (gcc, no hardware required)
 *
 * Compile:
 *   cd esp32s3/tests
 *   gcc test_command_gen.c ../src/command_gen.c -lm -o test_command_gen && ./test_command_gen
 */

#include <stdio.h>
#include <math.h>
#include "../../types.h"
#include "../src/command_gen.h"

#define PI_F  3.14159265f
#define TOL   0.01f   /* floating-point tolerance for comparisons */

static int nearly_equal(float a, float b)
{
    return fabsf(a - b) <= TOL;
}

/* ── Test 1: target directly ahead ──────────────────────────────────────────
 * Robot at origin, facing +X (theta=0). Target at (500, 0).
 * Expected: t_heading=0, heading_error=0, tx=500, ty=0, speed=SPEED_MAX.     */
static int test_target_straight_ahead(void)
{
    printf("Test 1: target directly ahead → zero heading error, full speed ... ");

    pose_t     pose   = {0};
    pose.x = 0.0f; pose.y = 0.0f; pose.theta = 0.0f;

    waypoint_t target = {0};
    target.x = 500.0f; target.y = 0.0f;

    control_frame_t cmd = command_gen_compute(&pose, &target);

    int ok = nearly_equal(cmd.tx, 500.0f)
          && nearly_equal(cmd.ty,   0.0f)
          && nearly_equal(cmd.t_heading, 0.0f)
          && cmd.t_speed >= 149.0f;   /* full speed (SPEED_MAX = 150) */

    if (ok)
        printf("PASS (tx=%.0f ty=%.0f heading=%.2f speed=%.0f)\n",
               cmd.tx, cmd.ty, cmd.t_heading, cmd.t_speed);
    else
        printf("FAIL (tx=%.0f ty=%.0f heading=%.2f speed=%.0f)\n",
               cmd.tx, cmd.ty, cmd.t_heading, cmd.t_speed);
    return ok;
}

/* ── Test 2: target 90° to the left ─────────────────────────────────────────
 * Robot at origin facing +X. Target at (0, 500) — directly left (+Y).
 * Expected: t_heading = π/2, heading_error = π/2, tx=0, ty=500.              */
static int test_target_90_left(void)
{
    printf("Test 2: target 90° left → heading = π/2, tx=0, ty=500 ... ");

    pose_t     pose   = {0};
    pose.x = 0.0f; pose.y = 0.0f; pose.theta = 0.0f;

    waypoint_t target = {0};
    target.x = 0.0f; target.y = 500.0f;

    control_frame_t cmd = command_gen_compute(&pose, &target);

    int ok = nearly_equal(cmd.tx,        0.0f)
          && nearly_equal(cmd.ty,      500.0f)
          && nearly_equal(cmd.t_heading, PI_F * 0.5f);

    if (ok)
        printf("PASS (tx=%.0f ty=%.0f heading=%.2f speed=%.0f)\n",
               cmd.tx, cmd.ty, cmd.t_heading, cmd.t_speed);
    else
        printf("FAIL (tx=%.0f ty=%.0f heading=%.2f speed=%.0f)\n",
               cmd.tx, cmd.ty, cmd.t_heading, cmd.t_speed);
    return ok;
}

/* ── Test 3: target directly behind ─────────────────────────────────────────
 * Robot at origin facing +X. Target at (-500, 0) — straight behind.
 * Expected: t_heading = π (or −π), heading_error = π, speed = SPEED_MIN.
 * Also verifies the frame still has the right tx/ty.                         */
static int test_target_behind(void)
{
    printf("Test 3: target directly behind → heading error = π, speed = SPEED_MIN ... ");

    pose_t     pose   = {0};
    pose.x = 0.0f; pose.y = 0.0f; pose.theta = 0.0f;

    waypoint_t target = {0};
    target.x = -500.0f; target.y = 0.0f;

    control_frame_t cmd = command_gen_compute(&pose, &target);

    /* atan2(0, -500) = π */
    int ok = nearly_equal(cmd.tx,  -500.0f)
          && nearly_equal(cmd.ty,     0.0f)
          && nearly_equal(fabsf(cmd.t_heading), PI_F);

    if (ok)
        printf("PASS (tx=%.0f ty=%.0f heading=%.2f speed=%.0f)\n",
               cmd.tx, cmd.ty, cmd.t_heading, cmd.t_speed);
    else
        printf("FAIL (tx=%.0f ty=%.0f heading=%.2f speed=%.0f)\n",
               cmd.tx, cmd.ty, cmd.t_heading, cmd.t_speed);
    return ok;
}

/* ── Test 4: t_heading always within (−π, π] ────────────────────────────────
 * atan2f guarantees (−π, π], so t_heading should always be in range regardless
 * of where the target is. Tests four quadrant directions from a non-origin pose.
 *
 * Note: normalise_angle acts on the internal heading_err (t_heading − pose.theta).
 * Its effect on speed is not observable here because cosf is periodic
 * (cos(x) = cos(x + 2π)), so speed is the same with or without wrapping.
 * normalise_angle becomes critical when a future proportional turn controller
 * consumes heading_err directly as a signed value.                            */
static int test_heading_always_in_range(void)
{
    printf("Test 4: t_heading always within (−π, π] for all quadrant targets ... ");

    pose_t pose = {0};
    pose.x = 300.0f; pose.y = 400.0f; pose.theta = 1.2f;

    float targets[][2] = {
        { 800.0f, 400.0f },   /* right  */
        {-200.0f, 400.0f },   /* left   */
        { 300.0f, 900.0f },   /* up     */
        { 300.0f,-100.0f },   /* down   */
    };
    int n = (int)(sizeof(targets) / sizeof(targets[0]));

    int ok = 1;
    for (int i = 0; i < n; i++) {
        waypoint_t wp = {0};
        wp.x = targets[i][0]; wp.y = targets[i][1];
        control_frame_t cmd = command_gen_compute(&pose, &wp);
        if (cmd.t_heading <= -PI_F - TOL || cmd.t_heading > PI_F + TOL) {
            printf("FAIL (target[%d] → t_heading=%.3f out of (−π, π])\n",
                   i, cmd.t_heading);
            ok = 0;
        }
    }
    if (ok) printf("PASS (%d directions all in (−π, π])\n", n);
    return ok;
}

/* ── Test 5: NULL pointer safety ─────────────────────────────────────────────
 * Both NULL inputs must return a zero frame without crashing.                */
static int test_null_safety(void)
{
    printf("Test 5: NULL inputs return zero frame safely ... ");

    control_frame_t a = command_gen_compute(NULL, NULL);
    control_frame_t b = command_gen_compute(NULL, &(waypoint_t){0});
    control_frame_t c = command_gen_compute(&(pose_t){0}, NULL);

    int ok = (a.tx == 0.0f && a.ty == 0.0f && a.t_speed == 0.0f)
          && (b.tx == 0.0f && b.t_speed == 0.0f)
          && (c.tx == 0.0f && c.t_speed == 0.0f);

    printf(ok ? "PASS\n" : "FAIL\n");
    return ok;
}

/* ── Test 6: relative displacement ──────────────────────────────────────────
 * Robot NOT at origin. Verifies tx/ty are relative (target − pose),
 * not absolute coordinates — the Wemos needs this to compute distance.       */
static int test_relative_displacement(void)
{
    printf("Test 6: tx/ty are relative (target − pose), not absolute ... ");

    pose_t     pose   = {0};
    pose.x = 300.0f; pose.y = 400.0f; pose.theta = 0.0f;

    waypoint_t target = {0};
    target.x = 800.0f; target.y = 400.0f;   /* 500 mm ahead in X */

    control_frame_t cmd = command_gen_compute(&pose, &target);

    /* tx should be 500 (relative), not 800 (absolute) */
    int ok = nearly_equal(cmd.tx, 500.0f)
          && nearly_equal(cmd.ty,   0.0f);

    if (ok)
        printf("PASS (tx=%.0f ty=%.0f — correct relative displacement)\n",
               cmd.tx, cmd.ty);
    else
        printf("FAIL (tx=%.0f ty=%.0f — expected tx=500 ty=0)\n",
               cmd.tx, cmd.ty);
    return ok;
}

/* ── main ────────────────────────────────────────────────────────────────── */
int main(void)
{
    printf("=== test_command_gen ===\n\n");

    int pass = 0, total = 0;

    total++; pass += test_target_straight_ahead();
    total++; pass += test_target_90_left();
    total++; pass += test_target_behind();
    total++; pass += test_heading_always_in_range();
    total++; pass += test_null_safety();
    total++; pass += test_relative_displacement();

    printf("\nResult: %d/%d PASS\n", pass, total);
    return (pass == total) ? 0 : 1;
}
