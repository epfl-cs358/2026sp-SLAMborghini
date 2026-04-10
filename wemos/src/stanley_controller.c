/**
 * stanley_controller.c
 * Module: Stanley lateral controller for path tracking.
 * Board: Wemos D1 R32
 * Implementation phase: stub (Stanley formula not yet implemented)
 */

#include "stanley_controller.h"

float stanley_compute_steer(const pose_t *pose, const waypoint_t *target, float speed_mms)
{
    // TODO: implement
    // 1. Heading error: psi_e = target->theta - pose->theta (wrap to [-pi, pi])
    // 2. Cross-track error: e = perpendicular distance from pose to target segment
    //      dx = target->x - pose->x
    //      dy = target->y - pose->y
    //      e  = -dx * sin(target->theta) + dy * cos(target->theta)
    // 3. Stanley formula:
    //      delta = psi_e + atan2(k * e, speed_mms + epsilon)
    //      clamp delta to [-max_steer, +max_steer]
    (void)pose;
    (void)target;
    (void)speed_mms;
    return 0;
}

void stanley_set_gain(float k)
{
    // TODO: implement
    // Store k in a module-level static variable used by stanley_compute_steer().
    (void)k;
}
