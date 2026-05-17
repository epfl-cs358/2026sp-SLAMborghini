#include "path_streamer.h"
#include "uart_bridge.h"

#include <string.h>

#ifdef ESP_PLATFORM
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"
static const char *TAG = "path_streamer";
#endif

/* Keep the Wemos ring buffer roughly 3/4 full.
 * PP_RING_CAP = 32 (in pure_pursuit_controller.h); fill target = 24.
 * Max in-flight = fill_target + one chunk in transit = 24 + 8 = 32 = ring cap. */
#define PS_RING_CAPACITY    32u
#define PS_RING_FILL_TARGET 24u

static path_t    s_path;
static uint16_t  s_path_id       = 0;
static uint16_t  s_next_send_idx = 0;   /* next waypoint index to send       */
static uint16_t  s_consumed_idx  = 0;   /* last index Wemos confirmed done   */
static bool      s_active        = false;

#ifdef ESP_PLATFORM
static SemaphoreHandle_t s_mutex = NULL;
#endif

void path_streamer_init(void)
{
#ifdef ESP_PLATFORM
    s_mutex = xSemaphoreCreateMutex();
#endif
    memset(&s_path, 0, sizeof(s_path));
    s_path_id       = 0;
    s_next_send_idx = 0;
    s_consumed_idx  = 0;
    s_active        = false;
}

/* Send chunks until the in-flight window reaches PS_RING_FILL_TARGET or the
 * path is exhausted. Must be called with the mutex held (or before tasks start). */
static void send_chunks_to_fill(void)
{
    while (s_active) {
        uint16_t in_flight = s_next_send_idx - s_consumed_idx;
        if (in_flight >= PS_RING_FILL_TARGET) break;
        if (s_next_send_idx >= s_path.length)  break;

        path_chunk_t chunk;
        chunk.path_id     = s_path_id;
        chunk.start_index = s_next_send_idx;
        chunk.count       = 0;
        chunk.final_chunk = false;

        for (uint8_t i = 0; i < PATH_CHUNK_WP_COUNT; i++) {
            uint16_t wi = s_next_send_idx + i;
            if (wi >= s_path.length) break;
            chunk.wp[i] = s_path.waypoints[wi];
            chunk.count++;
        }

        s_next_send_idx += chunk.count;

        if (s_next_send_idx >= s_path.length) {
            chunk.final_chunk = true;
        }

#ifdef ESP_PLATFORM
        if (!uart_bridge_send_path_chunk(&chunk)) {
            ESP_LOGW(TAG, "chunk send failed start=%u", (unsigned)chunk.start_index);
            /* Rewind so we retry next update cycle. */
            s_next_send_idx = chunk.start_index;
            break;
        }
        ESP_LOGD(TAG, "sent chunk start=%u count=%u final=%d",
                 (unsigned)chunk.start_index, (unsigned)chunk.count,
                 (int)chunk.final_chunk);
#else
        uart_bridge_send_path_chunk(&chunk);
#endif
    }
}

void path_streamer_set_path(const path_t *path)
{
    if (!path || path->length == 0) return;

#ifdef ESP_PLATFORM
    xSemaphoreTake(s_mutex, portMAX_DELAY);
#endif

    memcpy(&s_path, path, sizeof(path_t));
    s_path_id++;
    s_next_send_idx = 0;
    s_consumed_idx  = 0;
    s_active        = true;

#ifdef ESP_PLATFORM
    ESP_LOGI(TAG, "new path id=%u len=%u", (unsigned)s_path_id, (unsigned)s_path.length);
#endif

    send_chunks_to_fill();

#ifdef ESP_PLATFORM
    xSemaphoreGive(s_mutex);
#endif
}

void path_streamer_update(uint16_t consumed_wp_idx, uint16_t consumed_path_id)
{
#ifdef ESP_PLATFORM
    xSemaphoreTake(s_mutex, portMAX_DELAY);
#endif

    if (!s_active || consumed_path_id != s_path_id) {
        /* Stale odom from a superseded plan — ignore. */
#ifdef ESP_PLATFORM
        xSemaphoreGive(s_mutex);
#endif
        return;
    }

    /* Only advance — never go backwards. */
    if (consumed_wp_idx > s_consumed_idx) {
        s_consumed_idx = consumed_wp_idx;
    }

    send_chunks_to_fill();

#ifdef ESP_PLATFORM
    xSemaphoreGive(s_mutex);
#endif
}

void path_streamer_handle_nack(uint16_t path_id, uint16_t expected_start)
{
#ifdef ESP_PLATFORM
    xSemaphoreTake(s_mutex, portMAX_DELAY);
#endif

    if (!s_active || path_id != s_path_id) {
#ifdef ESP_PLATFORM
        xSemaphoreGive(s_mutex);
#endif
        return;
    }

#ifdef ESP_PLATFORM
    ESP_LOGW(TAG, "NACK path_id=%u expected_start=%u (was %u)",
             (unsigned)path_id, (unsigned)expected_start,
             (unsigned)s_next_send_idx);
#endif

    s_next_send_idx = expected_start;
    send_chunks_to_fill();

#ifdef ESP_PLATFORM
    xSemaphoreGive(s_mutex);
#endif
}

bool path_streamer_is_active(void)
{
    return s_active;
}

void path_streamer_clear(void)
{
#ifdef ESP_PLATFORM
    xSemaphoreTake(s_mutex, portMAX_DELAY);
#endif
    s_active = false;
#ifdef ESP_PLATFORM
    xSemaphoreGive(s_mutex);
#endif
}

uint16_t path_streamer_current_path_id(void)
{
    return s_path_id;
}
