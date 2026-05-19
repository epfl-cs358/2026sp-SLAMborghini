#ifndef UART_BRIDGE_H
#define UART_BRIDGE_H

#include <stdbool.h>
#include <stdint.h>
#include "../../types.h"

void uart_bridge_init(void);

/* Send odometry feedback to ESP32-S3 (compact 10-byte wire encoding). */
bool uart_bridge_send_odom(const odom_t *odom);

/* Receive a streaming path chunk from ESP32-S3.
 * Returns true (once per chunk) when a valid path_chunk_t is available. */
bool uart_bridge_recv_path_chunk(path_chunk_t *out);

/* Notify ESP32-S3 that the current path has been fully executed. */
bool uart_bridge_send_path_done(void);

/* Send a NACK back to ESP32-S3 when a chunk arrives out of order.
 * path_id identifies the active plan; expected_start is the next
 * global index we need so ESP32-S3 can rewind and resend. */
bool uart_bridge_send_chunk_nack(uint16_t path_id, uint16_t expected_start);

/* Receive a local-planner override command from ESP32-S3.
 * Returns true (once per packet) when a valid control_frame_t is available.
 * The override is sent only in REACTIVE / ESCAPE / STOPPED modes; callers
 * should check esp_timer_get_time() against a 300 ms timeout and revert to
 * normal path following when no fresh override arrives. */
bool uart_bridge_recv_control_override(control_frame_t *out);

#endif /* UART_BRIDGE_H */
