/**
 * uart_bridge.h
 *
 * ESP32-S3 UART bridge API for the two-board architecture.
 * This side sends control/path packets to the Wemos and receives odometry,
 * path completion, and chunk NACK feedback.
 */

#ifndef UART_BRIDGE_H
#define UART_BRIDGE_H

#include <stdbool.h>
#include <stdint.h>
#include "../../types.h"

void uart_bridge_init(void);

bool uart_bridge_send_control(const control_frame_t *frame);

/* Send one 8-waypoint chunk from the active path plan. */
bool uart_bridge_send_path_chunk(const path_chunk_t *chunk);

/* Decode the next incoming odom packet; also drains MSG_PATH_DONE and
 * MSG_CHUNK_NACK messages as a side effect. Returns true when an odom
 * packet was decoded into *out. */
bool uart_bridge_recv_odom(odom_t *out);

/* Returns true (once) when Wemos signals the current path is complete. */
bool uart_bridge_recv_path_done(void);

/* Returns true (once) when Wemos NACKs a chunk.
 * Fills *out_path_id and *out_expected_start so the streamer can rewind. */
bool uart_bridge_recv_chunk_nack(uint16_t *out_path_id,
                                  uint16_t *out_expected_start);

#endif /* UART_BRIDGE_H */
