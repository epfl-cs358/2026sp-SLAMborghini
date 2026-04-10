/**
 * command_gen.c
 * Module: Command generator — converts pose and waypoint into a control_frame_t.
 * Board: ESP32-S3
 * Implementation phase: stub (frame computation not yet implemented)
 */

#include "command_gen.h"

control_frame_t command_gen_compute(const pose_t *pose, const waypoint_t *target)
{
    // TODO: implement
    // Compute relative displacement from pose to target:
    //   dx = target->x - pose->x
    //   dy = target->y - pose->y
    // Pack into control_frame_t:
    //   frame.tx        = target->x
    //   frame.ty        = target->y
    //   frame.t_heading = target->theta
    //   frame.t_speed   = target->v_target
    (void)pose;
    (void)target;
    control_frame_t result = {0};
    return result;
}
