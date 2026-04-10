/**
 * command_gen.h
 * Module: Command generator — converts pose and waypoint into a control_frame_t.
 * Board: ESP32-S3
 * Bridges the path planner and the UART bridge: takes the next waypoint and the
 * current robot pose and produces a control_frame_t ready for transmission to the
 * Wemos D1 R32 control board.
 */

#ifndef COMMAND_GEN_H
#define COMMAND_GEN_H

#include "../../types.h"

/**
 * Compute a control frame directing the robot toward the given target waypoint.
 * The resulting frame encodes the target position, heading, and speed.
 * @param pose   Pointer to the current robot pose (world frame).
 * @param target Pointer to the next waypoint on the planned path.
 * @return control_frame_t ready for transmission via uart_bridge_send_control().
 */
control_frame_t command_gen_compute(const pose_t *pose, const waypoint_t *target);

#endif /* COMMAND_GEN_H */
