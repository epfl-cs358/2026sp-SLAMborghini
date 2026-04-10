// STUB — simulates uart_bridge (Wemos side) for offline testing

/**
 * uart_stub.c
 * Module: UART stub — simulates Wemos UART bridge for offline tests.
 * Board: Wemos D1 R32 (PC stub, no hardware required)
 * Used by integration tests.
 */

#include "uart_stub.h"

/*
 * Scenario: ESP32-S3 commands the robot to drive straight at 300 mm/s.
 * t_heading=0.0 means no heading correction needed — straight ahead.
 * Used by: integration tests
 */
control_frame_t uart_stub_recv_control(void)
{
    control_frame_t frame;
    frame.tx        = 0.0f;
    frame.ty        = 0.0f;
    frame.t_heading = 0.0f;
    frame.t_speed   = 300.0f;
    return frame;
}

/*
 * Scenario: odom UART send always succeeds (no hardware errors).
 * Allows isolating test logic from real UART availability.
 * Used by: integration tests
 */
bool uart_stub_send_ok(void)
{
    return true;
}
