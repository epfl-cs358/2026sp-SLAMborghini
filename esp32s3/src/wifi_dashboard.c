/**
 * wifi_dashboard.c
 * Module: Wi-Fi HTTP + WebSocket dashboard.
 * Board: ESP32-S3
 *
 * Implements:
 *   wifi_dashboard_init()           — STA Wi-Fi + httpd server startup.
 *   wifi_dashboard_update()         — map/pose snapshot for HTTP.
 *   wifi_dashboard_broadcast_state() — WebSocket JSON push to live dashboard.
 *
 * WebSocket endpoint: /ws  (ws://<ip>/ws)
 * One active client is supported (single-client hardware test scenario).
 * The client fd is captured on handshake and released on error/close.
 */

#include "wifi_dashboard.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
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

/* ── Wi-Fi connection event group ───────────────────────────────────────── */
#define WIFI_CONNECTED_BIT  BIT0
#define WIFI_FAIL_BIT       BIT1
#define WIFI_MAX_RETRIES    10

static EventGroupHandle_t s_wifi_eg   = NULL;
static int                s_retries   = 0;

/* ── HTTP server handle ─────────────────────────────────────────────────── */
static httpd_handle_t s_server        = NULL;

/* ── Active WebSocket client file-descriptor (-1 = none) ───────────────── */
static int            s_ws_fd         = -1;
static SemaphoreHandle_t s_ws_mutex   = NULL;

/* ── Command flags — set when browser sends the matching cmd ────────────── */
static volatile bool  s_start_requested = false;
static volatile bool  s_stop_requested  = false;


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
            ESP_LOGI(TAG, "Retry Wi-Fi connection (%d/%d)", s_retries, WIFI_MAX_RETRIES);
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
 * WebSocket URI handler
 * ════════════════════════════════════════════════════════════════════════════ */
static esp_err_t ws_handler(httpd_req_t *req)
{
    if (req->method == HTTP_GET) {
        /* WebSocket upgrade handshake — capture the client fd */
        int fd = httpd_req_to_sockfd(req);
        xSemaphoreTake(s_ws_mutex, portMAX_DELAY);
        s_ws_fd = fd;
        xSemaphoreGive(s_ws_mutex);
        ESP_LOGI(TAG, "WebSocket client connected (fd=%d)", fd);
        return ESP_OK;
    }

    /* Handle incoming frames — browser may send {"cmd":"start"} */
    httpd_ws_frame_t pkt = { .type = HTTPD_WS_TYPE_TEXT };
    uint8_t buf[64] = {0};
    pkt.payload = buf;
    pkt.len     = 0;

    esp_err_t ret = httpd_ws_recv_frame(req, &pkt, sizeof(buf) - 1);
    if (ret != ESP_OK) {
        /* Client disconnected or error */
        xSemaphoreTake(s_ws_mutex, portMAX_DELAY);
        s_ws_fd = -1;
        xSemaphoreGive(s_ws_mutex);
        ESP_LOGI(TAG, "WebSocket client disconnected");
        return ret;
    }

    if (pkt.len > 0) {
        if (strstr((char *)buf, "\"start\"")) {
            s_start_requested = true;
            ESP_LOGI(TAG, "Exploration start requested");
        } else if (strstr((char *)buf, "\"stop\"")) {
            s_stop_requested = true;
            ESP_LOGI(TAG, "Emergency stop requested");
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
 * Minimal root page — tells the user to open live_dashboard.html instead
 * ════════════════════════════════════════════════════════════════════════════ */
static const char *ROOT_HTML =
    "<!DOCTYPE html><html><body style='font-family:monospace;background:#1a1a2e;color:#e0e0e0'>"
    "<h2 style='color:#00d4ff'>SLAMborghini ESP32-S3</h2>"
    "<p>WebSocket endpoint: <b>ws://this-ip/ws</b></p>"
    "<p>Open <b>tools/dashboard/live_dashboard.html</b> on your PC and point it at this IP.</p>"
    "</body></html>";

static esp_err_t root_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, ROOT_HTML, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static const httpd_uri_t s_root_uri = {
    .uri     = "/",
    .method  = HTTP_GET,
    .handler = root_handler,
};


/* ════════════════════════════════════════════════════════════════════════════
 * wifi_dashboard_init
 * ════════════════════════════════════════════════════════════════════════════ */
void wifi_dashboard_init(const char *ssid, const char *password)
{
    s_ws_mutex = xSemaphoreCreateMutex();

    /* ── NVS ──────────────────────────────────────────────────────────────── */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    /* ── TCP/IP + event loop ──────────────────────────────────────────────── */
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

    /* ── Configure STA ───────────────────────────────────────────────────── */
    wifi_config_t wcfg = {0};
    strlcpy((char *)wcfg.sta.ssid,     ssid,     sizeof(wcfg.sta.ssid));
    strlcpy((char *)wcfg.sta.password, password, sizeof(wcfg.sta.password));

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wcfg);
    esp_wifi_start();

    ESP_LOGI(TAG, "Connecting to SSID: %s …", ssid);

    /* Block until connected or failure */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_eg,
                                            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                            pdFALSE, pdFALSE,
                                            pdMS_TO_TICKS(10000));
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Wi-Fi connected");
    } else {
        ESP_LOGW(TAG, "Wi-Fi connection failed — dashboard unavailable");
        return;
    }

    /* ── Start httpd ─────────────────────────────────────────────────────── */
    httpd_config_t hcfg = HTTPD_DEFAULT_CONFIG();
    hcfg.lru_purge_enable = true;

    if (httpd_start(&s_server, &hcfg) != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start() failed");
        return;
    }

    httpd_register_uri_handler(s_server, &s_root_uri);
    httpd_register_uri_handler(s_server, &s_ws_uri);

    ESP_LOGI(TAG, "HTTP server started.  Connect live_dashboard.html to ws://<ip>/ws");
}


/* ════════════════════════════════════════════════════════════════════════════
 * wifi_dashboard_update
 * Serialise the quadtree occupancy grid to a binary WebSocket frame and push
 * it to the connected dashboard client.
 *
 * Frame layout (little-endian):
 *   [0..1]  uint16  gw      — grid width  in cells (x-axis)
 *   [2..3]  uint16  gh      — grid height in cells (y-axis)
 *   [4..7]  float32 cellMm  — cell size in mm
 *   [8..]   uint8   cells[] — gw*gh occupancy bytes, row-major, iy=0 = y_min
 *
 * Each cell byte mirrors quadtree_map_query():
 *   20  = free / traversed   (log-odds < 0)
 *   128 = unknown             (never observed)
 *   230 = occupied / wall     (log-odds > 0)
 *
 * Grid is sampled at 200×200 to match the frontier-detector resolution
 * (50 mm/cell for a 10 000 mm map).
 * ════════════════════════════════════════════════════════════════════════════ */
#define DASH_GW 100u
#define DASH_GH 100u

/* 10 008 bytes in BSS — fits comfortably on Wemos (320 KB) and ESP32-S3. */
static uint8_t s_map_frame[8u + DASH_GW * DASH_GH];

void wifi_dashboard_update(const quadtree_map_t *map, const pose_t *pose)
{
    (void)pose;
    if (!map || !s_server) return;

    xSemaphoreTake(s_ws_mutex, portMAX_DELAY);
    int fd = s_ws_fd;
    xSemaphoreGive(s_ws_mutex);

    if (fd < 0) return;

    float map_w = map->x_max - map->x_min;
    float map_h = map->y_max - map->y_min;
    if (map_w <= 0.0f || map_h <= 0.0f) return;

    float cell_mm = map_w / (float)DASH_GW;

    /* Header (little-endian; ESP32 is LE so plain memcpy is correct) */
    s_map_frame[0] = (uint8_t)(DASH_GW & 0xFFu);
    s_map_frame[1] = (uint8_t)(DASH_GW >> 8u);
    s_map_frame[2] = (uint8_t)(DASH_GH & 0xFFu);
    s_map_frame[3] = (uint8_t)(DASH_GH >> 8u);
    memcpy(&s_map_frame[4], &cell_mm, sizeof(float));

    /* Cell values sampled at each cell centre */
    float cy_step = map_h / (float)DASH_GH;
    float cx_step = map_w / (float)DASH_GW;
    for (uint16_t iy = 0; iy < DASH_GH; iy++) {
        float cy = map->y_min + (iy + 0.5f) * cy_step;
        for (uint16_t ix = 0; ix < DASH_GW; ix++) {
            float cx = map->x_min + (ix + 0.5f) * cx_step;
            s_map_frame[8u + iy * DASH_GW + ix] = quadtree_map_query(map, cx, cy);
        }
    }

    httpd_ws_frame_t frame = {
        .final      = true,
        .fragmented = false,
        .type       = HTTPD_WS_TYPE_BINARY,
        .payload    = s_map_frame,
        .len        = 8u + DASH_GW * DASH_GH,
    };

    esp_err_t err = httpd_ws_send_frame_async(s_server, fd, &frame);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "WS map send failed (fd=%d, err=0x%x)", fd, err);
        /* Only clear fd when the socket is actually closed, not on transient OOM */
        if (err == ESP_ERR_INVALID_STATE) {
            xSemaphoreTake(s_ws_mutex, portMAX_DELAY);
            s_ws_fd = -1;
            xSemaphoreGive(s_ws_mutex);
        }
    }
}


/* ════════════════════════════════════════════════════════════════════════════
 * wifi_dashboard_broadcast_state
 * Push a JSON frame over WebSocket to the live dashboard client.
 *
 * JSON format:
 *   {"x":<mm>,"y":<mm>,"theta":<rad>,"fx":<mm>,"fy":<mm>,"has_frontier":<0|1>,"si":<scan_idx>}
 *
 * "si" (scan index) is used by playback_dashboard.html to drive the live
 * map render: the browser processes room.log scans 0..si as the car moves.
 * In frontier mode pass scan_idx=0 (browser ignores map updates).
 *
 * Non-blocking.  Clears s_ws_fd on send error (client disconnected).
 * ════════════════════════════════════════════════════════════════════════════ */
void wifi_dashboard_broadcast_state(const pose_t *pose,
                                     float frontier_cx, float frontier_cy,
                                     bool has_frontier,
                                     uint16_t scan_idx)
{
    if (!s_server || !pose) return;

    xSemaphoreTake(s_ws_mutex, portMAX_DELAY);
    int fd = s_ws_fd;
    xSemaphoreGive(s_ws_mutex);

    if (fd < 0) return;   /* no client connected */

    char buf[192];
    int len = snprintf(buf, sizeof(buf),
        "{\"x\":%.1f,\"y\":%.1f,\"theta\":%.5f,"
        "\"fx\":%.1f,\"fy\":%.1f,\"has_frontier\":%d,\"si\":%u}",
        pose->x, pose->y, pose->theta,
        frontier_cx, frontier_cy,
        has_frontier ? 1 : 0,
        (unsigned)scan_idx);

    httpd_ws_frame_t frame = {
        .final      = true,
        .fragmented = false,
        .type       = HTTPD_WS_TYPE_TEXT,
        .payload    = (uint8_t *)buf,
        .len        = (size_t)len,
    };

    esp_err_t err = httpd_ws_send_frame_async(s_server, fd, &frame);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "WS state send failed (fd=%d, err=0x%x)", fd, err);
        if (err == ESP_ERR_INVALID_STATE) {
            xSemaphoreTake(s_ws_mutex, portMAX_DELAY);
            s_ws_fd = -1;
            xSemaphoreGive(s_ws_mutex);
        }
    }
}


/* ════════════════════════════════════════════════════════════════════════════
 * wifi_dashboard_exploration_requested
 * Returns true once after the browser sends {"cmd":"start"}, then resets.
 * ════════════════════════════════════════════════════════════════════════════ */
bool wifi_dashboard_exploration_requested(void)
{
    if (s_start_requested) {
        s_start_requested = false;
        return true;
    }
    return false;
}


bool wifi_dashboard_stop_requested(void)
{
    if (s_stop_requested) {
        s_stop_requested = false;
        return true;
    }
    return false;
}


bool wifi_dashboard_stop_peek(void)
{
    return s_stop_requested;   /* read without clearing */
}
