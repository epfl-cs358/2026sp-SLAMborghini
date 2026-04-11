/**
 * command_gen.c
 * Module: Command generator : converts pose and frontier target into a control_frame_t.
 * Board: ESP32-S3
 *
 * Pipeline position: frontier_detector_best() → command_gen_compute() → uart_bridge_send_control()
 *
 */

#include "command_gen.h"
#include <math.h>

/* ── Speed limits ─────────────────────────────────────────────────────────── */
#define SPEED_MAX   150.0f   /* mm/s — straight ahead                         */
#define SPEED_MIN    50.0f   /* mm/s — sharp turn (|heading_error| >= π/2)    */

/* ── Compile-time float constants ────────────────────────────────────────── */
#define PI_F        3.14159265f
#define TWO_PI_F    6.28318530f

/* ── Angle normalisation helper ──────────────────────────────────────────── */
/* Both operands (atan2f result and pose.theta) are already in (−π, π], so
 * their difference is in (−2π, 2π). One conditional add/subtract is enough —
 * no fmodf needed. Avoids a software remainder call on the ESP32-S3 FPU. */
static float normalise_angle(float a)
{
    if (a >  PI_F) return a - TWO_PI_F;
    if (a < -PI_F) return a + TWO_PI_F;
    return a;
}

/* ══════════════════════════════════════════════════════════════════════════
 * command_gen_compute
 * ══════════════════════════════════════════════════════════════════════════ */
control_frame_t command_gen_compute(const pose_t *pose, const waypoint_t *target)
{
    control_frame_t frame = {0};
    if (!pose || !target) return frame;

    /* ── Displacement from current pose to target (relative, mm) ─────────── */
    float dx = target->x - pose->x;
    float dy = target->y - pose->y;

    /* ── 1. Target heading ────────────────────────────────────────────────── */
    float t_heading = atan2f(dy, dx);          /* range: (−π, π] from atan2  */

    /* ── 2. Heading error, normalised to [−π, π] ─────────────────────────── */
    float heading_err = normalise_angle(t_heading - pose->theta);

    /* ── 3. Speed scaling ────────────────────────────────────────────────────
     * speed = SPEED_MIN + (SPEED_MAX - SPEED_MIN) * cos(|heading_error|)
     *
     * Motivation: the component of velocity making progress toward the target
     * is v * cos(heading_error) — a dot product, not a modelling choice.
     * Scaling speed by cos keeps that contribution proportional to the maximum
     * possible at every heading error.
     *
     * Smoothness: cos has zero derivative at 0°, so small heading wobbles near
     * straight-ahead cause no speed change. Linear scaling lacks this property
     * (constant slope → micro-jitter for any heading perturbation).
     *
     *   |err| = 0°   → SPEED_MAX (150 mm/s)  cos(0)   = 1.0
     *   |err| = 60°  → ~100 mm/s             cos(60°) = 0.5
     *   |err| = 90°  → SPEED_MIN ( 50 mm/s)  cos(90°) = 0.0  (clamped to MIN)
     *   |err| > 90°  → SPEED_MIN             cos > 90° < 0   (clamped to MIN)
     * ──────────────────────────────────────────────────────────────────────── */
    float abs_err = fabsf(heading_err);

    /* ── Fixed speed for turn testing (active) ──────────────────────────────
     * Once turns are verified on hardware, swap to the dynamic block below. */
    float speed = SPEED_MAX;
    (void)abs_err;

    /* ── Dynamic speed (restore when turns are verified) ────────────────────
     * float speed = SPEED_MIN + (SPEED_MAX - SPEED_MIN) * cosf(abs_err);
     * if (speed < SPEED_MIN) speed = SPEED_MIN; */

    /* ── 4. Motion mode: forward only ─────────────────────────────────────── */
    /* TODO: add reverse logic here when needed.
     * Condition for reverse: target is behind the robot AND turning to face it
     * costs more than backing up (e.g. |heading_error| > 2π/3). In that case
     * negate dx/dy and flip t_heading by π before packing the frame. */

    /* ── Pack frame ──────────────────────────────────────────────────────────
     * tx/ty are relative so the Wemos can compute dist = sqrt(tx²+ty²)
     * without needing world-frame origin knowledge. */
    frame.tx        = dx;
    frame.ty        = dy;
    frame.t_heading = t_heading;
    frame.t_speed   = speed;

    return frame;
}
