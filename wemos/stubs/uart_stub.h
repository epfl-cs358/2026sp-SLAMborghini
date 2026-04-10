/**
 * uart_stub.h
 * Module: UART stub — simulates Wemos UART bridge for offline tests.
 * Board: Wemos D1 R32 (PC stub, no hardware required)
 * Used by integration tests.
 */

// STUB — simulates uart_bridge (Wemos side) for offline testing

#ifndef UART_STUB_H
#define UART_STUB_H

#include <stdbool.h>
#include "../../types.h"

/**
 * Simulates receiving a control_frame_t from the ESP32-S3.
 * Returns a frame commanding the robot to move straight at 300 mm/s.
 * tx=0, ty=0, t_heading=0.0, t_speed=300 mm/s.
 * Used by: integration tests
 */
control_frame_t uart_stub_recv_control(void);

/**
 * Simulates a successful UART send: always returns true.
 * Used to verify that odom transmission paths do not block on hardware.
 * Used by: integration tests
 */
bool uart_stub_send_ok(void);

#endif /* UART_STUB_H */
