#ifndef PATH_STREAMER_H
#define PATH_STREAMER_H

#include <stdbool.h>
#include <stdint.h>
#include "../../types.h"
#include "hybrid_astar.h"

/**
 * path_streamer — owns the full path_t and feeds Wemos's ring buffer in
 * PATH_CHUNK_WP_COUNT-waypoint chunks using a watermark refill strategy.
 *
 * Call path_streamer_set_path() when a new plan arrives.
 * Call path_streamer_update() every time a new odom_t is received so consumed
 * progress is tracked and the ring is topped up.
 * Call path_streamer_handle_nack() when uart_bridge_recv_chunk_nack() fires.
 */

void path_streamer_init(void);

/* Load a new plan. Increments the internal path_id and begins streaming
 * immediately so the first chunks arrive before the first odom reply. */
void path_streamer_set_path(const path_t *path);

/* Advance the consumed pointer and send more chunks if the ring is low.
 * consumed_path_id must match the current path_id or the call is a no-op
 * (stale odom from a superseded plan). */
void path_streamer_update(uint16_t consumed_wp_idx, uint16_t consumed_path_id);

/* Rewind the send pointer to expected_start and retransmit.
 * No-op if path_id doesn't match the active plan. */
void path_streamer_handle_nack(uint16_t path_id, uint16_t expected_start);

bool     path_streamer_is_active(void);
void     path_streamer_clear(void);
uint16_t path_streamer_current_path_id(void);

#endif /* PATH_STREAMER_H */
