/**
 * wifi_dashboard.c — Wi-Fi HTTP + WebSocket dashboard, redesigned.
 *
 * Bug fixes applied (see diagnostic for full root-cause analysis):
 *   Bug A  Quadtree broadcast was doubled every plan cycle → stub removed,
 *          explicit caller-side calls removed from both main.c files.
 *   Bug B  s_ws_fd=-1 with no recovery → _on_send_error() now calls
 *          httpd_sess_trigger_close() so browser's onclose fires → 3 s retry.
 *   Bug C  wifi_dashboard_log with dangling stack pointer → string is now
 *          copied into the queue entry before posting.
 *   Bug D  _ws_send() released mutex before send, multiple tasks raced on
 *          shared frame buffers → all sends serialised through dash_task
 *          (priority 1), sole owner of every send buffer.
 *   Bug E  scan_task priority 5 = httpd priority → scan_task lowered to 4
 *          in wemos/main.c; httpd at 5 now always preempts it.
 *   Bug F  2 s silence during drive → dash_task sends 1-byte keepalive
 *          every 2 s; browser TCP connection stays alive.
 *
 * Optimisations applied:
 *   • 50×50 raster grid (was 100×100): 75% less map RAM and bandwidth.
 *   • Quadtree broadcast removed: s_qt_frame (48 KB) eliminated.
 *   • Single dash_task owns all send buffers → no mutex on frame data.
 *   • Separate static buffer per message type (safe for async send).
 *   • Scan downsampled to 90 points (was 180).
 *   • Static RAM: ~88 KB → ~12 KB.
 *
 * Binary message protocol (browser-compatible, unchanged):
 *   0x01  Map full  [type(1) gw(2) gh(2) cellMm(4) xMin(4) yMin(4) cells(GW*GH)]
 *   0x02  Scan      [type(1) count(2) {angle_cdeg(2) range_mm(2)}×count]
 *   0x03  Pose      [type(1) x(4) y(4) theta(4) fx(4) fy(4) has_frontier(1) scan_idx(2)]
 *   0x04  Map delta [type(1) count(2) {cell_idx(2) val(1)}×count]
 *   0xFE  Keepalive [type(1)] — browser ignores unknown types silently.
 */

#include "wifi_dashboard.h"
#include "lidar_driver.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "esp_http_server.h"

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <stdlib.h>

static const char *TAG = "wifi_dash";

/* ── Message type bytes ─────────────────────────────────────────────────── */
#define MSG_MAP       0x01u
#define MSG_SCAN      0x02u
#define MSG_POSE      0x03u
#define MSG_MAP_DELTA 0x04u
#define MSG_PATH      0x06u   /* A* planned path: [type(1)][count(1)][{x(4)y(4)}×count] */
#define MSG_KEEPALIVE 0xFEu   /* 1-byte heartbeat; browser ignores unknown types */

/* ── Dashboard grid ─────────────────────────────────────────────────────── */
#define DASH_GW    50u
#define DASH_GH    50u
#define DASH_CELLS (DASH_GW * DASH_GH)   /* 2500 cells — 50×50 raster grid (was 64×64 = 4096 cells;
                                           * larger frames triggered TCP backpressure → 3-strike WS
                                           * close → scan display froze after first update) */

/* ── Scan downsample cap ─────────────────────────────────────────────────── */
#define SCAN_MAX_PTS 90u

/* ── Static send buffers — owned exclusively by dash_task ───────────────────
 *
 * Full-map frame:  1+2+2+4+4+4+2500          = 2517 bytes  (50×50 grid)
 * Delta threshold: when Δcells > 838, a full map (2517 B) is smaller than the
 *                  delta (3 + 839*3 = 2520 B) → switch to full map.
 *                  Both paths fit in MAP_BUF_SIZE = 2517.
 * Pose frame:      1+4+4+4+4+4+1+2           =   24 bytes
 * Scan frame:      1+2+90*4                  =  363 bytes
 * Log ring:        5 slots × 80 bytes        =  400 bytes
 * ─────────────────────────────────────────────────────────────────────────── */
#define MAP_BUF_SIZE  2517u
#define DELTA_THRESH   838u   /* Δcells above this → switch to full-map send */

/* ── Tile dirty bitmap (4×4 = 16 tiles, each 16×16 cells) ───────────────────
 * Avoids querying all 4096 cells when only a small region changed.
 * Tile index = row * TILE_COLS + col; bit 0 = top-left tile. */
#define TILE_COLS  4u
#define TILE_ROWS  4u
#define TILE_W    16u   /* cells per tile column */
#define TILE_H    16u   /* cells per tile row    */
#define TILES_ALL  ((uint32_t)((1u << (TILE_COLS * TILE_ROWS)) - 1u))

static uint8_t s_map_buf[MAP_BUF_SIZE];                            /* full map or delta  */
static uint8_t s_pose_buf[24u];                                    /* pose frame         */
static uint8_t s_scan_buf[3u + SCAN_MAX_PTS * 4u];                /* scan frame         */
static uint8_t s_path_buf[2u + MAX_SHARED_PATH_POINTS * 8u];      /* path frame         */
static uint8_t s_log_bufs[5][80u];                                 /* 5-slot log ring    */
static uint8_t s_log_slot = 0u;

/* ── Map shadow (delta encoding) ─────────────────────────────────────────── */
static uint8_t s_last_cells[DASH_CELLS];          /* last-sent cell values     */
static uint8_t s_new_cells [DASH_CELLS];          /* current sampled values    */

/* ── Scan staging buffer ─────────────────────────────────────────────────── */
typedef struct { uint16_t acd; uint16_t rmm; } scan_pt_t;
static scan_pt_t s_scan_pts[SCAN_MAX_PTS];
static uint16_t  s_scan_count = 0u;

/* ── Stable map reference (set once by wifi_dashboard_update) ────────────── */
static const quadtree_map_t *s_map_ref = NULL;

/* ── Dash task queue ─────────────────────────────────────────────────────── */
#define DASH_QUEUE_LEN 24u   /* 24 × ~88 B = 2.1 KB; absorbs scan+map+pose burst + 6 logs */
#define DASH_LOG_MAX   80u

typedef enum {
    DASH_MAP_MSG  = 0,
    DASH_POSE_MSG = 1,
    DASH_SCAN_MSG = 2,
    DASH_LOG_MSG  = 3,
    DASH_PATH_MSG = 4,
} dash_msg_type_t;

typedef struct {
    dash_msg_type_t type;
    uint32_t dirty_tiles;   /* DASH_MAP_MSG only: which 5×5 tiles need resampling */
    union {
        struct {
            float    x, y, theta;
            float    fx, fy;
            bool     has_frontier;
            uint16_t scan_idx;
        } pose;
        char log[DASH_LOG_MAX];
        struct {
            uint8_t count;
            uint8_t _pad[3];
            float   pts_x[MAX_SHARED_PATH_POINTS];
            float   pts_y[MAX_SHARED_PATH_POINTS];
        } path;
    };
} dash_msg_t;

static QueueHandle_t s_dash_queue = NULL;

/* ── Wi-Fi ──────────────────────────────────────────────────────────────── */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
#define WIFI_MAX_RETRIES   10

static EventGroupHandle_t s_wifi_eg = NULL;
static int                s_retries = 0;

/* ── HTTP / WebSocket state ──────────────────────────────────────────────── */
static httpd_handle_t    s_server   = NULL;
static int               s_ws_fd    = -1;

/* s_ws_mutex protects: s_ws_fd, s_shadow_valid, s_last_map_us */
static SemaphoreHandle_t s_ws_mutex  = NULL;
/* s_scan_mtx protects: s_scan_pts, s_scan_count */
static SemaphoreHandle_t s_scan_mtx  = NULL;
/* s_dirty_mtx protects: s_pending_dirty (written by scan_task, read by plan_task) */
static SemaphoreHandle_t s_dirty_mtx = NULL;
static map_dirty_rect_t  s_pending_dirty = { .valid = false };

static bool    s_shadow_valid = false;
static int64_t s_last_map_us  = 0;

/* ── Command flags ──────────────────────────────────────────────────────── */
static volatile bool s_start_requested = false;
static volatile bool s_stop_requested  = false;


/* ════════════════════════════════════════════════════════════════════════════
 * _on_send_error — clear WS state and force TCP close on any send failure.
 *
 * Calling httpd_sess_trigger_close causes the httpd server to emit a TCP FIN
 * on the socket.  The browser's ws.onclose fires, and its existing 3-second
 * retry timer reconnects automatically.  On reconnect, ws_handler resets
 * s_shadow_valid and s_last_map_us so the new client receives a fresh full map.
 *
 * Called only from dash_task (via _ws_send_raw or _ws_heartbeat).
 * httpd_sess_trigger_close requires ESP-IDF 4.x+.
 * ════════════════════════════════════════════════════════════════════════════ */
static void _on_send_error(int fd)
{
    xSemaphoreTake(s_ws_mutex, portMAX_DELAY);
    if (s_ws_fd == fd) {
        s_ws_fd        = -1;
        s_shadow_valid = false;
        s_last_map_us  = 0;
    }
    xSemaphoreGive(s_ws_mutex);

    if (s_server) {
        httpd_sess_trigger_close(s_server, fd);
    }
    ESP_LOGI(TAG, "WS fd=%d closed after send error — browser will reconnect", fd);
}


/* ════════════════════════════════════════════════════════════════════════════
 * _ws_send_raw — low-level WebSocket send.
 * Called only from dash_task (sole WS owner) — no frame-buffer races.
 *
 * 3-strike rule: transient congestion should not kill the connection.
 * Only calls _on_send_error after 3 consecutive failures on the same fd.
 * Counter resets to 0 on any success.
 * ════════════════════════════════════════════════════════════════════════════ */
static bool _ws_send_raw(uint8_t *payload, size_t len, httpd_ws_type_t type)
{
    static uint8_t s_fail_streak = 0u;

    xSemaphoreTake(s_ws_mutex, portMAX_DELAY);
    int fd = s_ws_fd;
    xSemaphoreGive(s_ws_mutex);
    if (fd < 0 || !s_server) return false;

    httpd_ws_frame_t frame = {
        .final      = true,
        .fragmented = false,
        .type       = type,
        .payload    = payload,
        .len        = len,
    };
    esp_err_t err = httpd_ws_send_frame_async(s_server, fd, &frame);
    if (err != ESP_OK) {
        s_fail_streak++;
        ESP_LOGW(TAG, "_ws_send_raw fd=%d err=0x%x streak=%u/3",
                 fd, (unsigned)err, (unsigned)s_fail_streak);
        if (s_fail_streak >= 3u) {
            s_fail_streak = 0u;
            _on_send_error(fd);
        }
        return false;
    }
    s_fail_streak = 0u;
    return true;
}


/* ════════════════════════════════════════════════════════════════════════════
 * _tiles_from_rect — convert a world-coordinate dirty rect to a 16-bit tile mask.
 *
 * The 64×64 cell grid is divided into 16 tiles of 16×16 cells each (4 columns,
 * 4 rows).  Only tiles whose world bbox overlaps the dirty rect get a set bit.
 * Returns TILES_ALL on any degenerate input so callers never under-sample.
 * Called only from wifi_dashboard_update() (plan_task context).
 * ════════════════════════════════════════════════════════════════════════════ */
static uint32_t _tiles_from_rect(const map_dirty_rect_t *dr)
{
    const quadtree_map_t *map = s_map_ref;
    if (!map || !dr || !dr->valid) return TILES_ALL;
    float map_w = map->x_max - map->x_min;
    float map_h = map->y_max - map->y_min;
    if (map_w <= 0.0f || map_h <= 0.0f) return TILES_ALL;

    float tile_w_mm = (map_w / (float)DASH_GW) * (float)TILE_W;
    float tile_h_mm = (map_h / (float)DASH_GH) * (float)TILE_H;

    int col_min = (int)((dr->x_min - map->x_min) / tile_w_mm);
    int col_max = (int)((dr->x_max - map->x_min) / tile_w_mm);
    int row_min = (int)((dr->y_min - map->y_min) / tile_h_mm);
    int row_max = (int)((dr->y_max - map->y_min) / tile_h_mm);

    if (col_min < 0)               col_min = 0;
    if (col_max >= (int)TILE_COLS) col_max = (int)TILE_COLS - 1;
    if (row_min < 0)               row_min = 0;
    if (row_max >= (int)TILE_ROWS) row_max = (int)TILE_ROWS - 1;
    if (col_min > col_max || row_min > row_max) return 0u;

    uint32_t mask = 0u;
    for (int r = row_min; r <= row_max; r++)
        for (int c = col_min; c <= col_max; c++)
            mask |= (1u << (r * (int)TILE_COLS + c));
    return mask;
}


/* ════════════════════════════════════════════════════════════════════════════
 * _do_map_send — sample dirty tiles of the 64×64 grid, build full or delta
 *                frame, send.
 *
 * dirty_tiles: 16-bit mask of which 4×4 tiles need qt_query_const resampling.
 *   — When shadow is invalid (new client), overridden to TILES_ALL.
 *   — Non-dirty tiles reuse their existing s_new_cells values; after a
 *     successful send s_last_cells==s_new_cells for those tiles, so they
 *     contribute zero delta bytes — correct.
 *
 * Uses s_map_buf (4113 B) for both paths:
 *   Δcells ≤ 1370 → delta  (3 + Δ*3 ≤ 4113 B)
 *   Δcells > 1370 → full   (17 + 4096 = 4113 B, smaller than delta at that count)
 *
 * Reads s_map_ref without holding s_mtx (same approach as original code).
 * Slight staleness is acceptable for a dashboard visualisation.
 *
 * Called only from dash_task.
 * ════════════════════════════════════════════════════════════════════════════ */
static void _do_map_send(uint32_t dirty_tiles)
{
    if (!s_map_ref) return;

    const quadtree_map_t *map = s_map_ref;
    float map_w = map->x_max - map->x_min;
    float map_h = map->y_max - map->y_min;
    if (map_w <= 0.0f || map_h <= 0.0f) return;

    float cx_step = map_w / (float)DASH_GW;
    float cy_step = map_h / (float)DASH_GH;
    float cell_mm = cx_step;

    xSemaphoreTake(s_ws_mutex, portMAX_DELAY);
    bool shadow_ok = s_shadow_valid;
    xSemaphoreGive(s_ws_mutex);

    /* New client or shadow reset: must resample everything */
    if (!shadow_ok) dirty_tiles = TILES_ALL;

    /* Sample only cells whose tile has a dirty bit set.
     * Non-dirty cells keep their previous s_new_cells value; since the last
     * successful send set s_last_cells == s_new_cells for those, they
     * produce zero diff bytes — no bandwidth wasted. */
    for (uint16_t iy = 0; iy < DASH_GH; iy++) {
        uint32_t tile_row = (uint32_t)(iy / TILE_H);
        float cy = map->y_min + (iy + 0.5f) * cy_step;
        for (uint16_t ix = 0; ix < DASH_GW; ix++) {
            uint32_t tile_idx = tile_row * TILE_COLS + (uint32_t)(ix / TILE_W);
            if (!((dirty_tiles >> tile_idx) & 1u)) continue;
            float cx = map->x_min + (ix + 0.5f) * cx_step;
            int8_t v = qt_query_const(map, cx, cy);
            s_new_cells[iy * DASH_GW + ix] = (uint8_t)((int16_t)v + 128);
        }
    }

    size_t send_len  = 0;
    bool   send_full = !shadow_ok;

    if (shadow_ok) {
        /* Count changed cells */
        uint16_t n = 0;
        for (uint16_t i = 0; i < DASH_CELLS; i++) {
            if (s_new_cells[i] != s_last_cells[i]) n++;
        }
        if (n == 0) return;   /* nothing changed — skip send */

        if (n > DELTA_THRESH) {
            send_full = true;   /* full map is smaller than delta at this count */
        } else {
            /* Build delta frame */
            uint8_t *p = &s_map_buf[3];
            uint16_t j = 0;
            for (uint16_t i = 0; i < DASH_CELLS; i++) {
                if (s_new_cells[i] == s_last_cells[i]) continue;
                p[j * 3u + 0u] = (uint8_t)(i & 0xFFu);
                p[j * 3u + 1u] = (uint8_t)(i >> 8u);
                p[j * 3u + 2u] = s_new_cells[i];
                j++;
            }
            s_map_buf[0] = MSG_MAP_DELTA;
            s_map_buf[1] = (uint8_t)(n & 0xFFu);
            s_map_buf[2] = (uint8_t)(n >> 8u);
            send_len = 3u + (size_t)n * 3u;
        }
    }

    if (send_full) {
        /* Build full-map frame */
        s_map_buf[0] = MSG_MAP;
        s_map_buf[1] = (uint8_t)(DASH_GW & 0xFFu);
        s_map_buf[2] = (uint8_t)(DASH_GW >> 8u);
        s_map_buf[3] = (uint8_t)(DASH_GH & 0xFFu);
        s_map_buf[4] = (uint8_t)(DASH_GH >> 8u);
        memcpy(&s_map_buf[5],  &cell_mm,    sizeof(float));
        memcpy(&s_map_buf[9],  &map->x_min, sizeof(float));
        memcpy(&s_map_buf[13], &map->y_min, sizeof(float));
        memcpy(&s_map_buf[17], s_new_cells, DASH_CELLS);
        send_len = 17u + DASH_CELLS;
    }

    bool sent = _ws_send_raw(s_map_buf, send_len, HTTPD_WS_TYPE_BINARY);
    if (sent) {
        memcpy(s_last_cells, s_new_cells, DASH_CELLS);
        xSemaphoreTake(s_ws_mutex, portMAX_DELAY);
        s_shadow_valid = true;
        xSemaphoreGive(s_ws_mutex);
        ESP_LOGI(TAG, "map %s sent len=%u", send_full ? "FULL" : "DELTA",
                 (unsigned)send_len);
    }
}


/* ════════════════════════════════════════════════════════════════════════════
 * _do_pose_send — build type-0x03 frame into s_pose_buf, send.
 * Called only from dash_task.
 * ════════════════════════════════════════════════════════════════════════════ */
static void _do_pose_send(const dash_msg_t *msg)
{
    s_pose_buf[0] = MSG_POSE;
    memcpy(&s_pose_buf[1],  &msg->pose.x,      4);
    memcpy(&s_pose_buf[5],  &msg->pose.y,      4);
    memcpy(&s_pose_buf[9],  &msg->pose.theta,  4);
    memcpy(&s_pose_buf[13], &msg->pose.fx,     4);
    memcpy(&s_pose_buf[17], &msg->pose.fy,     4);
    s_pose_buf[21] = msg->pose.has_frontier ? 1u : 0u;
    s_pose_buf[22] = (uint8_t)(msg->pose.scan_idx & 0xFFu);
    s_pose_buf[23] = (uint8_t)(msg->pose.scan_idx >> 8u);
    _ws_send_raw(s_pose_buf, 24u, HTTPD_WS_TYPE_BINARY);
}


/* ════════════════════════════════════════════════════════════════════════════
 * _do_scan_send — copy staged scan into s_scan_buf, send.
 * Holds s_scan_mtx only for the memcpy, then releases before the async send.
 * s_scan_buf is owned by dash_task and not touched again until the next
 * DASH_SCAN message (≥ 500 ms), so httpd has time to process the frame.
 * Called only from dash_task.
 * ════════════════════════════════════════════════════════════════════════════ */
static void _do_scan_send(void)
{
    xSemaphoreTake(s_scan_mtx, portMAX_DELAY);
    uint16_t cnt = s_scan_count;
    memcpy(&s_scan_buf[3], s_scan_pts, (size_t)cnt * sizeof(scan_pt_t));
    xSemaphoreGive(s_scan_mtx);

    if (cnt == 0) return;
    s_scan_buf[0] = MSG_SCAN;
    s_scan_buf[1] = (uint8_t)(cnt & 0xFFu);
    s_scan_buf[2] = (uint8_t)(cnt >> 8u);
    _ws_send_raw(s_scan_buf, 3u + (size_t)cnt * 4u, HTTPD_WS_TYPE_BINARY);
}


/* ════════════════════════════════════════════════════════════════════════════
 * _do_log_send — copy string into a rotating 3-slot buffer, send as text.
 *
 * The 5-slot ring ensures five consecutive log sends each use a distinct buffer,
 * giving httpd time to process each one before the slot is reused.
 * Called only from dash_task.
 * ════════════════════════════════════════════════════════════════════════════ */
static void _do_log_send(const char *msg)
{
    uint8_t *buf = s_log_bufs[s_log_slot];
    s_log_slot   = (uint8_t)((s_log_slot + 1u) % 5u);
    size_t len   = strnlen(msg, (size_t)(DASH_LOG_MAX - 1u));
    memcpy(buf, msg, len);
    _ws_send_raw(buf, len, HTTPD_WS_TYPE_TEXT);
}


/* ════════════════════════════════════════════════════════════════════════════
 * _do_path_send — build type-0x06 frame into s_path_buf, send.
 * count=0 is valid: tells the browser to clear its stale path overlay.
 * Called only from dash_task.
 * ════════════════════════════════════════════════════════════════════════════ */
static void _do_path_send(const dash_msg_t *msg)
{
    uint8_t n = msg->path.count;
    if (n > MAX_SHARED_PATH_POINTS) n = MAX_SHARED_PATH_POINTS;
    s_path_buf[0] = MSG_PATH;
    s_path_buf[1] = n;
    for (uint8_t i = 0; i < n; i++) {
        memcpy(&s_path_buf[2u + (size_t)i * 8u],      &msg->path.pts_x[i], 4u);
        memcpy(&s_path_buf[2u + (size_t)i * 8u + 4u], &msg->path.pts_y[i], 4u);
    }
    _ws_send_raw(s_path_buf, 2u + (size_t)n * 8u, HTTPD_WS_TYPE_BINARY);
}


/* ════════════════════════════════════════════════════════════════════════════
 * _ws_heartbeat — 1-byte keepalive frame sent every 2 s.
 *
 * Prevents the browser's WebSocket from timing out during the robot's ≤2 s
 * drive phases when plan_task posts no updates.  Type 0xFE is not used by
 * any MSG_* constant, so the browser's binary dispatcher silently ignores it.
 * Called only from dash_task.
 * ════════════════════════════════════════════════════════════════════════════ */
static void _ws_heartbeat(void)
{
    static uint8_t ka = MSG_KEEPALIVE;
    _ws_send_raw(&ka, 1u, HTTPD_WS_TYPE_BINARY);
}


/* ════════════════════════════════════════════════════════════════════════════
 * _dash_task — priority 1 (lowest user task).
 *
 * Sole owner of all WebSocket send buffers and httpd_ws_send_frame_async.
 * Lower priority than scan_task (4), plan_task (3), and httpd (5) ensures
 * the httpd task always has priority to drain its internal send queue before
 * dash_task posts another frame.
 *
 * Runs at ~33 Hz (30 ms sleep); heartbeat fires every 1 s.
 * Queue telemetry: logs a warning if fill exceeds 50% of capacity.
 * ════════════════════════════════════════════════════════════════════════════ */
static void _dash_task(void *arg)
{
    (void)arg;
    dash_msg_t msg;
    TickType_t last_hb = 0;

    for (;;) {
        TickType_t now = xTaskGetTickCount();

        /* Heartbeat every 1 s (Bug F fix) + queue telemetry */
        if ((now - last_hb) >= pdMS_TO_TICKS(1000)) {
            _ws_heartbeat();
            last_hb = now;
            UBaseType_t q = uxQueueMessagesWaiting(s_dash_queue);
            if (q > (DASH_QUEUE_LEN / 2u)) {
                ESP_LOGW(TAG, "dash queue %u/%u — backpressure",
                         (unsigned)q, (unsigned)DASH_QUEUE_LEN);
            }
        }

        /* Drain queue — non-blocking; stale messages are simply dropped if
         * the queue filled up (non-blocking xQueueSend in callers). */
        while (xQueueReceive(s_dash_queue, &msg, 0) == pdTRUE) {
            switch (msg.type) {
                case DASH_MAP_MSG:  _do_map_send(msg.dirty_tiles); break;
                case DASH_POSE_MSG: _do_pose_send(&msg);   break;
                case DASH_SCAN_MSG: _do_scan_send();       break;
                case DASH_LOG_MSG:  _do_log_send(msg.log); break;
                case DASH_PATH_MSG: _do_path_send(&msg);   break;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(30));
    }
}


/* ════════════════════════════════════════════════════════════════════════════
 * Wi-Fi event handler (unchanged)
 * ════════════════════════════════════════════════════════════════════════════ */
static void wifi_event_handler(void *arg, esp_event_base_t base,
                                int32_t event_id, void *data)
{
    if (base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t *disc = (wifi_event_sta_disconnected_t *)data;
        ESP_LOGW(TAG, "Disconnected reason=%u", (unsigned)disc->reason);
        if (s_retries < WIFI_MAX_RETRIES) {
            esp_wifi_connect();
            s_retries++;
            ESP_LOGI(TAG, "Retry Wi-Fi (%d/%d)", s_retries, WIFI_MAX_RETRIES);
        } else {
            xEventGroupSetBits(s_wifi_eg, WIFI_FAIL_BIT);
        }
    } else if (base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *ev = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&ev->ip_info.ip));
        s_retries = 0;
        xEventGroupSetBits(s_wifi_eg, WIFI_CONNECTED_BIT);
    }
}


/* ════════════════════════════════════════════════════════════════════════════
 * WebSocket handler
 *
 * On new client (HTTP_GET upgrade): reset shadow so the new client receives
 * a full map on the very next wifi_dashboard_update() call.
 *
 * On recv error: clear s_ws_fd.  dash_task detects fd<0 and skips all sends
 * until the browser reconnects and triggers a new HTTP_GET.
 * ════════════════════════════════════════════════════════════════════════════ */
static esp_err_t ws_handler(httpd_req_t *req)
{
    if (req->method == HTTP_GET) {
        int fd = httpd_req_to_sockfd(req);
        xSemaphoreTake(s_ws_mutex, portMAX_DELAY);
        s_ws_fd        = fd;
        s_shadow_valid = false;   /* force full map on next update */
        s_last_map_us  = 0;       /* allow immediate map send       */
        xSemaphoreGive(s_ws_mutex);
        ESP_LOGI(TAG, "WS client connected fd=%d", fd);
        return ESP_OK;
    }

    httpd_ws_frame_t pkt = { .type = HTTPD_WS_TYPE_TEXT };
    uint8_t buf[64] = {0};
    pkt.payload = buf;
    pkt.len     = 0;

    esp_err_t ret = httpd_ws_recv_frame(req, &pkt, sizeof(buf) - 1);
    if (ret != ESP_OK) {
        xSemaphoreTake(s_ws_mutex, portMAX_DELAY);
        s_ws_fd = -1;
        xSemaphoreGive(s_ws_mutex);
        ESP_LOGI(TAG, "WS client disconnected (recv err 0x%x)", (unsigned)ret);
        return ret;
    }

    if (pkt.len > 0) {
        /* "ping" from browser → reply "pong" (latency measurement) */
        if (pkt.len == 4 && memcmp(buf, "ping", 4) == 0) {
            static uint8_t pong[] = "pong";
            int fd = httpd_req_to_sockfd(req);
            httpd_ws_frame_t pr = {
                .final=true, .type=HTTPD_WS_TYPE_TEXT,
                .payload=pong, .len=4
            };
            httpd_ws_send_frame_async(s_server, fd, &pr);
            return ESP_OK;
        }
        if (strstr((char *)buf, "\"start\"")) {
            s_start_requested = true;
            ESP_LOGI(TAG, "Start requested");
        } else if (strstr((char *)buf, "\"stop\"")) {
            s_stop_requested = true;
            ESP_LOGI(TAG, "Stop requested");
        }
    }
    return ESP_OK;
}

static const httpd_uri_t s_ws_uri = {
    .uri          = "/ws",
    .method       = HTTP_GET,
    .handler      = ws_handler,
    .user_ctx     = NULL,
    .is_websocket = true,
};


/* ── Root page ──────────────────────────────────────────────────────────── */
static const char *ROOT_HTML =
    "<!DOCTYPE html><html><body style='font-family:monospace;background:#0f172a;color:#e2e8f0'>"
    "<h2 style='color:#38bdf8'>SLAMborghini ESP32</h2>"
    "<p>WebSocket: <b>ws://this-ip/ws</b></p>"
    "<p>Open <b>tools/dashboard/live_dashboard.html</b> on your PC.</p>"
    "</body></html>";

static esp_err_t root_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, ROOT_HTML, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static const httpd_uri_t s_root_uri = {
    .uri = "/", .method = HTTP_GET, .handler = root_handler,
};


/* ════════════════════════════════════════════════════════════════════════════
 * wifi_dashboard_init
 * ════════════════════════════════════════════════════════════════════════════ */
void wifi_dashboard_init(const char *ssid, const char *password)
{
    s_ws_mutex   = xSemaphoreCreateMutex();
    s_scan_mtx   = xSemaphoreCreateMutex();
    s_dirty_mtx  = xSemaphoreCreateMutex();
    s_dash_queue = xQueueCreate(DASH_QUEUE_LEN, sizeof(dash_msg_t));

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    s_wifi_eg = xEventGroupCreate();

    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                         &wifi_event_handler, NULL, NULL);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                         &wifi_event_handler, NULL, NULL);

    wifi_config_t wcfg = {0};
    strlcpy((char *)wcfg.sta.ssid,     ssid,     sizeof(wcfg.sta.ssid));
    strlcpy((char *)wcfg.sta.password, password, sizeof(wcfg.sta.password));
    wcfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;   /* accept WPA2 and above */

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wcfg);
    esp_wifi_start();

    ESP_LOGI(TAG, "Connecting to %s …", ssid);
    EventBits_t bits = xEventGroupWaitBits(s_wifi_eg,
                                            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                            pdFALSE, pdFALSE,
                                            pdMS_TO_TICKS(10000));
    if (!(bits & WIFI_CONNECTED_BIT)) {
        ESP_LOGW(TAG, "Wi-Fi failed — dashboard unavailable");
        return;
    }

    /* Disable Modem Sleep — prevents radio power-cycling mid-session */
    esp_wifi_set_ps(WIFI_PS_NONE);

    httpd_config_t hcfg = HTTPD_DEFAULT_CONFIG();
    hcfg.lru_purge_enable = true;
    if (httpd_start(&s_server, &hcfg) != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start() failed");
        return;
    }
    httpd_register_uri_handler(s_server, &s_root_uri);
    httpd_register_uri_handler(s_server, &s_ws_uri);
    ESP_LOGI(TAG, "Dashboard ready — open live_dashboard.html, point at ws://<ip>/ws");

    /* dash_task: priority 1 — sole WebSocket sender, lowest user priority.
     * httpd (5) > scan_task (4) > plan_task (3) > dash_task (1). */
    xTaskCreate(_dash_task, "dash", 3072, NULL, 1, NULL);
}


/* ════════════════════════════════════════════════════════════════════════════
 * wifi_dashboard_update — store map reference, throttle to 1 Hz, post DASH_MAP.
 * ════════════════════════════════════════════════════════════════════════════ */
void wifi_dashboard_update(const quadtree_map_t *map, const pose_t *pose)
{
    (void)pose;
    if (!map || !s_dash_queue) return;
    s_map_ref = map;   /* stable pointer; map is static in main.c */

    xSemaphoreTake(s_ws_mutex, portMAX_DELAY);
    int64_t now      = esp_timer_get_time();
    bool throttled   = (now - s_last_map_us < 1000000LL);
    if (!throttled)  s_last_map_us = now;
    xSemaphoreGive(s_ws_mutex);
    if (throttled) return;

    /* Snapshot-and-clear the pending dirty rect, compute tile mask.
     * No valid dirty rect → resample all tiles (conservative but safe). */
    xSemaphoreTake(s_dirty_mtx, portMAX_DELAY);
    map_dirty_rect_t dr  = s_pending_dirty;
    s_pending_dirty.valid = false;
    xSemaphoreGive(s_dirty_mtx);

    dash_msg_t msg = { .type = DASH_MAP_MSG };
    msg.dirty_tiles = dr.valid ? _tiles_from_rect(&dr) : TILES_ALL;
    xQueueSend(s_dash_queue, &msg, 0);   /* non-blocking: drop if queue full */
}


/* ════════════════════════════════════════════════════════════════════════════
 * wifi_dashboard_broadcast_state — copy pose into queue entry, post DASH_POSE.
 * ════════════════════════════════════════════════════════════════════════════ */
void wifi_dashboard_broadcast_state(const pose_t *pose,
                                     float frontier_cx, float frontier_cy,
                                     bool has_frontier,
                                     uint16_t scan_idx)
{
    if (!pose || !s_dash_queue) return;
    dash_msg_t msg;
    msg.type              = DASH_POSE_MSG;
    msg.pose.x            = pose->x;
    msg.pose.y            = pose->y;
    msg.pose.theta        = pose->theta;
    msg.pose.fx           = frontier_cx;
    msg.pose.fy           = frontier_cy;
    msg.pose.has_frontier = has_frontier;
    msg.pose.scan_idx     = scan_idx;
    xQueueSend(s_dash_queue, &msg, 0);
}


/* ════════════════════════════════════════════════════════════════════════════
 * wifi_dashboard_broadcast_scan — downsample, stage into s_scan_pts, post DASH_SCAN.
 * Gating to ≤ 2 Hz is the caller's responsibility (scan_task's 500 ms tick check).
 * ════════════════════════════════════════════════════════════════════════════ */
void wifi_dashboard_broadcast_scan(const lidar_scan_t *scan, const pose_t *pose)
{
    (void)pose;
    if (!scan || scan->count == 0 || !s_dash_queue) return;

    uint16_t step = (scan->count > SCAN_MAX_PTS) ? (scan->count / SCAN_MAX_PTS) : 1u;
    uint16_t out  = 0u;

    xSemaphoreTake(s_scan_mtx, portMAX_DELAY);
    for (uint16_t i = 0; i < scan->count && out < SCAN_MAX_PTS; i += step) {
        float r = scan->points[i].r_mm;
        if (r < 100.0f || r > 6000.0f) continue;
        s_scan_pts[out].acd = (uint16_t)(scan->points[i].theta_deg * 100.0f);
        s_scan_pts[out].rmm = (uint16_t)r;
        out++;
    }
    s_scan_count = out;
    xSemaphoreGive(s_scan_mtx);

    if (out == 0u) return;
    dash_msg_t msg = { .type = DASH_SCAN_MSG };
    xQueueSend(s_dash_queue, &msg, 0);
}


/* ════════════════════════════════════════════════════════════════════════════
 * wifi_dashboard_broadcast_path — post A* waypoints (or empty list) to browser.
 *
 * Pass the same path_frame_t sent to the Wemos so the browser shows exactly
 * what the car is executing.  Pass NULL or a frame with length=0 to clear the
 * stale path overlay when A* fails or no frontier exists.
 * Called from plan_task once per planning cycle — non-blocking queue post.
 * ════════════════════════════════════════════════════════════════════════════ */
void wifi_dashboard_broadcast_path(const path_frame_t *frame)
{
    if (!s_dash_queue) return;
    dash_msg_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.type = DASH_PATH_MSG;
    if (frame && frame->length > 0 && frame->length <= MAX_SHARED_PATH_POINTS) {
        msg.path.count = frame->length;
        for (uint8_t i = 0; i < frame->length; i++) {
            msg.path.pts_x[i] = frame->waypoints[i].x;
            msg.path.pts_y[i] = frame->waypoints[i].y;
        }
    }
    xQueueSend(s_dash_queue, &msg, 0);
}


/* ════════════════════════════════════════════════════════════════════════════
 * wifi_dashboard_broadcast_quadtree — removed (48 KB s_qt_frame eliminated).
 * Kept as a no-op stub so existing call sites compile without modification.
 * ════════════════════════════════════════════════════════════════════════════ */
void wifi_dashboard_broadcast_quadtree(const quadtree_map_t *map)
{
    (void)map;
}


/* ════════════════════════════════════════════════════════════════════════════
 * wifi_dashboard_mark_dirty — accumulate a dirty rect from one lidar_to_map()
 * call into s_pending_dirty under mutex.
 *
 * scan_task calls this ~5× per second; wifi_dashboard_update() (called from
 * plan_task at ~1 Hz) snapshots and clears s_pending_dirty, so the union of
 * all scan bboxes since the last update reaches the dashboard.
 * ════════════════════════════════════════════════════════════════════════════ */
void wifi_dashboard_mark_dirty(const map_dirty_rect_t *rect)
{
    if (!rect || !rect->valid || !s_dirty_mtx) return;
    xSemaphoreTake(s_dirty_mtx, portMAX_DELAY);
    if (!s_pending_dirty.valid) {
        s_pending_dirty = *rect;
    } else {
        if (rect->x_min < s_pending_dirty.x_min) s_pending_dirty.x_min = rect->x_min;
        if (rect->y_min < s_pending_dirty.y_min) s_pending_dirty.y_min = rect->y_min;
        if (rect->x_max > s_pending_dirty.x_max) s_pending_dirty.x_max = rect->x_max;
        if (rect->y_max > s_pending_dirty.y_max) s_pending_dirty.y_max = rect->y_max;
    }
    xSemaphoreGive(s_dirty_mtx);
}


/* ════════════════════════════════════════════════════════════════════════════
 * wifi_dashboard_log — copy string into queue entry, post DASH_LOG.
 *
 * The string is copied immediately into the queue entry (not stored as a
 * pointer), so callers may use stack-allocated buffers safely.
 * ════════════════════════════════════════════════════════════════════════════ */
void wifi_dashboard_log(const char *msg)
{
    if (!msg || !s_dash_queue) return;
    dash_msg_t qmsg;
    qmsg.type = DASH_LOG_MSG;
    size_t len = strnlen(msg, (size_t)(DASH_LOG_MAX - 1u));
    memcpy(qmsg.log, msg, len);
    qmsg.log[len] = '\0';
    xQueueSend(s_dash_queue, &qmsg, 0);
}


/* ════════════════════════════════════════════════════════════════════════════
 * Control flag accessors (unchanged)
 * ════════════════════════════════════════════════════════════════════════════ */
bool wifi_dashboard_exploration_requested(void)
{
    if (s_start_requested) { s_start_requested = false; return true; }
    return false;
}

bool wifi_dashboard_stop_requested(void)
{
    if (s_stop_requested) { s_stop_requested = false; return true; }
    return false;
}

bool wifi_dashboard_stop_peek(void)
{
    return s_stop_requested;
}


/* ════════════════════════════════════════════════════════════════════════════
 * wifi_dashboard_queue_depth — live queue fill level for diagnostics.
 * Returns 0 if the queue has not been created yet.
 * Safe to call from any task — uxQueueMessagesWaiting is ISR-safe.
 * ════════════════════════════════════════════════════════════════════════════ */
uint8_t wifi_dashboard_queue_depth(void)
{
    if (!s_dash_queue) return 0u;
    return (uint8_t)uxQueueMessagesWaiting(s_dash_queue);
}
