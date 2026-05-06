#ifndef UART_BRIDGE_H
#define UART_BRIDGE_H

#include <stdbool.h>
#include <stdint.h>
#include "../../types.h"

void uart_bridge_init(void);

bool uart_bridge_send_control(const control_frame_t *frame);

bool uart_bridge_send_path(const path_frame_t *path_frame);

bool uart_bridge_recv_odom(odom_t *out);

/* Returns true (once) when Wemos signals that the current path is complete. */
bool uart_bridge_recv_path_done(void);

#endif /* UART_BRIDGE_H */