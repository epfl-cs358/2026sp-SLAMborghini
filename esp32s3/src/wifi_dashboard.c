/**
 * wifi_dashboard.c
 * Wi-Fi HTTP + WebSocket dashboard — live map, pose, scan.
 *
 * Binary message protocol (first byte = type):
 *   0x01  Map full  — [type(1), gw(2), gh(2), cellMm(4), xMin(4), yMin(4), cells(gw*gh)]
 *                     cells: uint8 log-odds+128 {<118=free, 128=unknown, >138=wall}
 *   0x02  Scan      — [type(1), count(2), {angle_cdeg(2), range_mm(2)}×count]
 *   0x03  Pose      — [type(1), x(4), y(4), theta(4), fx(4), fy(4), has_frontier(1), scan_idx(2)]
 *   0x04  Map delta — [type(1), count(2), {cell_idx(2), val(1)}×count]
 *                     cell_idx = iy*gw + ix; only cells that changed since last send.
 *                     Browser patches its rawGrid then rebuilds the bitmap.
 * Text messages:
 *   "ping" from browser → MCU replies "pong"  (latency measurement)
 *   {"cmd":"start"/"stop"} from browser        (exploration control)
 *
 * Rates: map ≤ 1 Hz (internal throttle), scan ≤ 10 Hz (caller), pose ≤ 20 Hz (caller).
 */

#include "wifi_dashboard.h"

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
#define MSG_QUADTREE  0x05u

/* ── Wi-Fi ──────────────────────────────────────────────────────────────── */
#define WIFI_CONNECTED_BIT  BIT0
#define WIFI_FAIL_BIT       BIT1
#define WIFI_MAX_RETRIES    10

static EventGroupHandle_t s_wifi_eg = NULL;
static int                s_retries = 0;

/* ── HTTP server ────────────────────────────────────────────────────────── */
static httpd_handle_t s_server   = NULL;
static int            s_ws_fd    = -1;
static SemaphoreHandle_t s_ws_mutex = NULL;

/* ── Command flags ──────────────────────────────────────────────────────── */
static volatile bool s_start_requested = false;
static volatile bool s_stop_requested  = false;

/* ── Static message buffers (no heap allocation after init) ─────────────── */
#define DASH_GW 100u
#define DASH_GH 100u
#define DASH_CELLS (DASH_GW * DASH_GH)

/* Full map: type(1) + gw(2) + gh(2) + cellMm(4) + xMin(4) + yMin(4) + cells */
static uint8_t s_map_frame[1u + 16u + DASH_CELLS];

/* Delta map: type(1) + count(2) + up to DASH_CELLS × {idx(2)+val(1)} */
static uint8_t s_delta_frame[3u + DASH_CELLS * 3u];

/* Shadow of the last sent cell values — 0xFF = never sent (forces full map) */
static uint8_t s_last_cells[DASH_CELLS];
static bool    s_shadow_valid = false;   /* false → send full map next time */

/* Scan: type(1) + count(2) + up to 180 points × 4 bytes */
static uint8_t s_scan_frame[3u + 180u * 4u];

/* Pose: type(1) + x(4) + y(4) + theta(4) + fx(4) + fy(4) + has_frontier(1) + scan_idx(2) */
static uint8_t s_pose_frame[24u];

/* Quadtree: type(1) + count(2) + up to 1500 nodes × 8 bytes each */
#define QT_BCAST_MAX 1500u
static uint8_t s_qt_frame[3u + QT_BCAST_MAX * 8u];

/* Map throttle — send at most once per second */
static int64_t s_last_map_us = 0;


/* ════════════════════════════════════════════════════════════════════════════
 * Internal send helper — takes mutex, checks fd, sends frame.
 * Returns true on success.
 * ════════════════════════════════════════════════════════════════════════════ */
static bool _ws_send(uint8_t *payload, size_t len, httpd_ws_type_t type)
{
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
        ESP_LOGW(TAG, "_ws_send type=0x%02x fd=%d err=0x%x", type, fd, (unsigned)err);
        if (err == ESP_ERR_INVALID_STATE) {
            xSemaphoreTake(s_ws_mutex, portMAX_DELAY);
            s_ws_fd = -1;
            xSemaphoreGive(s_ws_mutex);
        }
        return false;
    }
    return true;
}


/* ════════════════════════════════════════════════════════════════════════════
 * Wi-Fi event handler
 * ════════════════════════════════════════════════════════════════════════════ */
static void wifi_event_handler(void *arg, esp_event_base_t base,
                                int32_t event_id, void *data)
{
    if (base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
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
 * ════════════════════════════════════════════════════════════════════════════ */
static esp_err_t ws_handler(httpd_req_t *req)
{
    if (req->method == HTTP_GET) {
        int fd = httpd_req_to_sockfd(req);
        xSemaphoreTake(s_ws_mutex, portMAX_DELAY);
        s_ws_fd = fd;
        xSemaphoreGive(s_ws_mutex);
        /* New client — force a full map on the next wifi_dashboard_update() */
        s_shadow_valid  = false;
        s_last_map_us   = 0;
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
        /* Ping → pong (latency measurement) */
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
        /* Exploration commands */
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


/* ════════════════════════════════════════════════════════════════════════════
 * Root page
 * ════════════════════════════════════════════════════════════════════════════ */
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
    s_ws_mutex = xSemaphoreCreateMutex();

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

    /* Disable Modem Sleep — prevents the radio from power-cycling mid-session
     * which causes the TCP stack to drop WebSocket connections. */
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
}


/* ════════════════════════════════════════════════════════════════════════════
 * wifi_dashboard_update  — throttled to 1 Hz
 *
 * Sends MSG_MAP (0x01, full grid) when a new client connects, then switches
 * to MSG_MAP_DELTA (0x04, changed cells only) for subsequent updates.
 * Delta payload: [type(1), count(2), {idx(2), val(1)}×count]
 * Break-even vs full map: delta wins below ~3338 changed cells per cycle.
 * ════════════════════════════════════════════════════════════════════════════ */
void wifi_dashboard_update(const quadtree_map_t *map, const pose_t *pose)
{
    (void)pose;
    if (!map || !s_server) return;

    /* 1 Hz throttle */
    int64_t now = esp_timer_get_time();
    if (now - s_last_map_us < 1000000LL) return;
    s_last_map_us = now;

    xSemaphoreTake(s_ws_mutex, portMAX_DELAY);
    int fd = s_ws_fd;
    xSemaphoreGive(s_ws_mutex);
    if (fd < 0) {
        ESP_LOGW(TAG, "map: no WS client (fd=-1)");
        return;
    }

    float map_w = map->x_max - map->x_min;
    float map_h = map->y_max - map->y_min;
    if (map_w <= 0.0f || map_h <= 0.0f) {
        ESP_LOGE(TAG, "map: bad bounds x[%.0f,%.0f] y[%.0f,%.0f]",
                 map->x_min, map->x_max, map->y_min, map->y_max);
        return;
    }

    float cy_step = map_h / (float)DASH_GH;
    float cx_step = map_w / (float)DASH_GW;
    float cell_mm = map_w / (float)DASH_GW;

    /* ── Sample the current grid into a temporary new_cells array ── */
    /* Static: 10 KB on the stack would overflow FreeRTOS tasks. */
    static uint8_t new_cells[DASH_CELLS];
    for (uint16_t iy = 0; iy < DASH_GH; iy++) {
        float cy = map->y_min + (iy + 0.5f) * cy_step;
        for (uint16_t ix = 0; ix < DASH_GW; ix++) {
            float cx = map->x_min + (ix + 0.5f) * cx_step;
            int8_t v = qt_query_const(map, cx, cy);
            new_cells[iy * DASH_GW + ix] = (uint8_t)((int16_t)v + 128);
        }
    }

    esp_err_t err;

    if (!s_shadow_valid) {
        /* ── Full map (initial sync) ──────────────────────────────────── */
        s_map_frame[0] = MSG_MAP;
        s_map_frame[1] = (uint8_t)(DASH_GW & 0xFFu);
        s_map_frame[2] = (uint8_t)(DASH_GW >> 8u);
        s_map_frame[3] = (uint8_t)(DASH_GH & 0xFFu);
        s_map_frame[4] = (uint8_t)(DASH_GH >> 8u);
        memcpy(&s_map_frame[5],  &cell_mm,    sizeof(float));
        memcpy(&s_map_frame[9],  &map->x_min, sizeof(float));
        memcpy(&s_map_frame[13], &map->y_min, sizeof(float));
        memcpy(&s_map_frame[17], new_cells, DASH_CELLS);

        httpd_ws_frame_t frame = {
            .final=true, .fragmented=false, .type=HTTPD_WS_TYPE_BINARY,
            .payload=s_map_frame, .len=17u + DASH_CELLS,
        };
        err = httpd_ws_send_frame_async(s_server, fd, &frame);
        if (err == ESP_OK) {
            memcpy(s_last_cells, new_cells, DASH_CELLS);
            s_shadow_valid = true;
            ESP_LOGI(TAG, "map FULL sent fd=%d cells=%u", fd, DASH_CELLS);
        }
    } else {
        /* ── Delta map (changed cells only) ──────────────────────────── */
        uint16_t n = 0;
        uint8_t *p = &s_delta_frame[3];
        for (uint16_t i = 0; i < DASH_CELLS; i++) {
            if (new_cells[i] == s_last_cells[i]) continue;
            p[n * 3u + 0u] = (uint8_t)(i & 0xFFu);
            p[n * 3u + 1u] = (uint8_t)(i >> 8u);
            p[n * 3u + 2u] = new_cells[i];
            n++;
        }

        if (n == 0) return;   /* nothing changed — skip send */

        s_delta_frame[0] = MSG_MAP_DELTA;
        s_delta_frame[1] = (uint8_t)(n & 0xFFu);
        s_delta_frame[2] = (uint8_t)(n >> 8u);

        httpd_ws_frame_t frame = {
            .final=true, .fragmented=false, .type=HTTPD_WS_TYPE_BINARY,
            .payload=s_delta_frame, .len=3u + (size_t)n * 3u,
        };
        err = httpd_ws_send_frame_async(s_server, fd, &frame);
        if (err == ESP_OK) {
            memcpy(s_last_cells, new_cells, DASH_CELLS);
            ESP_LOGI(TAG, "map DELTA sent fd=%d changed=%u bytes=%u",
                     fd, n, (unsigned)(3u + n * 3u));
        }
    }

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "map send fd=%d err=0x%x", fd, (unsigned)err);
        if (err == ESP_ERR_INVALID_STATE) {
            xSemaphoreTake(s_ws_mutex, portMAX_DELAY);
            s_ws_fd = -1;
            xSemaphoreGive(s_ws_mutex);
            s_shadow_valid = false;
        }
        s_last_map_us = 0;   /* retry next cycle */
    }
}


/* ════════════════════════════════════════════════════════════════════════════
 * wifi_dashboard_broadcast_state  — type 0x03, call at 20 Hz
 * ════════════════════════════════════════════════════════════════════════════ */
void wifi_dashboard_broadcast_state(const pose_t *pose,
                                     float frontier_cx, float frontier_cy,
                                     bool has_frontier,
                                     uint16_t scan_idx)
{
    if (!pose) return;

    s_pose_frame[0] = MSG_POSE;
    memcpy(&s_pose_frame[1],  &pose->x,      4);
    memcpy(&s_pose_frame[5],  &pose->y,      4);
    memcpy(&s_pose_frame[9],  &pose->theta,  4);
    memcpy(&s_pose_frame[13], &frontier_cx,  4);
    memcpy(&s_pose_frame[17], &frontier_cy,  4);
    s_pose_frame[21] = has_frontier ? 1u : 0u;
    s_pose_frame[22] = (uint8_t)(scan_idx & 0xFFu);
    s_pose_frame[23] = (uint8_t)(scan_idx >> 8u);

    _ws_send(s_pose_frame, 24u, HTTPD_WS_TYPE_BINARY);
}


/* ════════════════════════════════════════════════════════════════════════════
 * wifi_dashboard_broadcast_scan  — type 0x02, call at up to 10 Hz
 * Downsamples to at most 180 points before sending.
 * ════════════════════════════════════════════════════════════════════════════ */
void wifi_dashboard_broadcast_scan(const lidar_scan_t *scan, const pose_t *pose)
{
    (void)pose;   /* pose is embedded in broadcast_state; browser uses latest */
    if (!scan || scan->count == 0) return;

    uint16_t step = (scan->count > 180u) ? (scan->count / 180u) : 1u;
    uint16_t out  = 0u;
    uint8_t *p    = &s_scan_frame[3];

    for (uint16_t i = 0; i < scan->count && out < 180u; i += step) {
        float r = scan->points[i].r_mm;
        if (r < 100.0f || r > 6000.0f) continue;
        uint16_t acd = (uint16_t)(scan->points[i].theta_deg * 100.0f);
        uint16_t rmm = (uint16_t)r;
        p[out * 4u + 0u] = (uint8_t)(acd & 0xFFu);
        p[out * 4u + 1u] = (uint8_t)(acd >> 8u);
        p[out * 4u + 2u] = (uint8_t)(rmm & 0xFFu);
        p[out * 4u + 3u] = (uint8_t)(rmm >> 8u);
        out++;
    }

    s_scan_frame[0] = MSG_SCAN;
    s_scan_frame[1] = (uint8_t)(out & 0xFFu);
    s_scan_frame[2] = (uint8_t)(out >> 8u);

    _ws_send(s_scan_frame, 3u + (size_t)out * 4u, HTTPD_WS_TYPE_BINARY);
}


/* ════════════════════════════════════════════════════════════════════════════
 * Control flag accessors
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
 * wifi_dashboard_broadcast_quadtree — type 0x05, call at ~1 Hz for debugging
 *
 * Frame layout (little-endian):
 *   [type:u8][count:u16][node × count]
 *   node: [x_min:u16][y_min:u16][size:u16][depth:u8][value:i8]  → 8 bytes
 *
 * Traverses the live QuadTreeMap via DFS and emits every allocated node so
 * the browser can reconstruct the full recursive wireframe grid.
 * Capped at QT_BCAST_MAX (1500) nodes — shallow nodes are emitted first so
 * truncation only drops deep leaves, keeping the structural skeleton intact.
 * ════════════════════════════════════════════════════════════════════════════ */

static int _pack_qt(const QuadTreeMap *m, uint16_t idx,
                    float xmn, float xmx, float ymn, float ymx,
                    uint8_t *out, int rem)
{
    if (idx == QT_NULL || rem < 8) return 0;
    const QTNode *n = &m->pool[idx];

    uint16_t x0 = (uint16_t)(xmn + 0.5f);
    uint16_t y0 = (uint16_t)(ymn + 0.5f);
    uint16_t sz = (uint16_t)((xmx - xmn) + 0.5f);
    out[0] = (uint8_t)(x0 & 0xFFu);  out[1] = (uint8_t)(x0 >> 8u);
    out[2] = (uint8_t)(y0 & 0xFFu);  out[3] = (uint8_t)(y0 >> 8u);
    out[4] = (uint8_t)(sz & 0xFFu);  out[5] = (uint8_t)(sz >> 8u);
    out[6] = n->depth;
    out[7] = (uint8_t)n->value;
    int w = 8;

    float cx = 0.5f * (xmn + xmx);
    float cy = 0.5f * (ymn + ymx);
    /* Quadrant order matches _child_bounds in quadtree_map.c:
     * 0=NW(xmn,cx,cy,ymx)  1=NE(cx,xmx,cy,ymx)
     * 2=SW(xmn,cx,ymn,cy)  3=SE(cx,xmx,ymn,cy) */
    if (n->children[0] != QT_NULL) w += _pack_qt(m, n->children[0], xmn, cx,  cy,  ymx, out+w, rem-w);
    if (n->children[1] != QT_NULL) w += _pack_qt(m, n->children[1], cx,  xmx, cy,  ymx, out+w, rem-w);
    if (n->children[2] != QT_NULL) w += _pack_qt(m, n->children[2], xmn, cx,  ymn, cy,  out+w, rem-w);
    if (n->children[3] != QT_NULL) w += _pack_qt(m, n->children[3], cx,  xmx, ymn, cy,  out+w, rem-w);
    return w;
}

void wifi_dashboard_broadcast_quadtree(const quadtree_map_t *map)
{
    if (!map) return;
    int packed = _pack_qt(map, 1u /*root*/,
                          map->x_min, map->x_max,
                          map->y_min, map->y_max,
                          s_qt_frame + 3, (int)sizeof(s_qt_frame) - 3);
    uint16_t nc  = (uint16_t)(packed / 8);
    s_qt_frame[0] = MSG_QUADTREE;
    s_qt_frame[1] = (uint8_t)(nc & 0xFFu);
    s_qt_frame[2] = (uint8_t)(nc >> 8u);
    _ws_send(s_qt_frame, (size_t)(3 + packed), HTTPD_WS_TYPE_BINARY);
}


/* ════════════════════════════════════════════════════════════════════════════
 * wifi_dashboard_log — send a plain-text line to the dashboard log box
 * ════════════════════════════════════════════════════════════════════════════ */
void wifi_dashboard_log(const char *msg)
{
    if (!msg || !s_server) return;
    xSemaphoreTake(s_ws_mutex, portMAX_DELAY);
    int fd = s_ws_fd;
    xSemaphoreGive(s_ws_mutex);
    if (fd < 0) return;

    httpd_ws_frame_t frame = {
        .final      = true,
        .fragmented = false,
        .type       = HTTPD_WS_TYPE_TEXT,
        .payload    = (uint8_t *)msg,
        .len        = strlen(msg),
    };
    httpd_ws_send_frame_async(s_server, fd, &frame);
}
