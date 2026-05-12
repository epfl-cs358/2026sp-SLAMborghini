/**
 * Board: ESP32-S3 (SLAM Brain)
 *
 * ── Mode flags ───────────────────────────────────────────────────────────────
 * USE_REAL_LIDAR         1  →  Real hardware: RPLiDAR C1 → quadtree → dashboard.
 *                              Set USE_WAYPOINT_PLAYBACK and USE_FRONTIER_TARGET
 *                              to 0 when using this mode.
 *                              Open tools/dashboard/live_dashboard.html, enter
 *                              ESP32 IP, click "Start Exploration".
 *
 * USE_WAYPOINT_PLAYBACK  1  →  Follow room.log trajectory waypoints physically.
 *                              The car drives through the pre-recorded path;
 *                              the browser (playback_dashboard.html) builds the
 *                              map scan-by-scan from room_scan_data.js, synced
 *                              to the broadcast scan index "si".
 *                              ⚠ Set USE_FRONTIER_TARGET to 0 in this mode.
 *
 * USE_WAYPOINT_PLAYBACK  0,
 * USE_FRONTIER_TARGET    1  →  Real frontier exploration on the pre-loaded
 *                              test room (previous behaviour).
 *
 * USE_WAYPOINT_PLAYBACK  0,
 * USE_FRONTIER_TARGET    0  →  UART + motor smoke test (hardcoded 200 mm fwd).
 * ─────────────────────────────────────────────────────────────────────────── */
/* ── Hardware test modes — uncomment exactly one; all USE_* flags are ignored ── */
// #define TEST_LIDAR           /* read scans and print point count + samples   */
// #define TEST_BRIDGE_TX       /* send test control frames to Wemos every 2 s  */
// #define TEST_BRIDGE_PING     /* interactive: press ENTER → send "hey1", print replies */
/* NOTE: TEST_IMU lives in wemos/main.c — IMU is on the Wemos I2C bus.      */

#define USE_REAL_LIDAR         1   /* ← SET TO 1 FOR REAL LIDAR HARDWARE        */
#define USE_WAYPOINT_PLAYBACK  0   /* ← SET TO 1 FOR ROOM.LOG PLAYBACK          */
#define USE_FRONTIER_TARGET    0   /* ← SET TO 1 FOR FRONTIER EXPLORATION        */
#define USE_LOCAL_PLANNER      0   /* disabled — pipeline is lidar→map→frontier→A*→PP */

/* ── Wi-Fi credentials — fill in before flashing ───────────────────────── */
#define WIFI_SSID      "SPOT-iot"
#define WIFI_PASSWORD  "RacailleSalutaireMigration8052"

#include "../hardware_pins.h"
#include "src/lidar_driver.h"
#include "src/lidar_to_map.h"
#include "src/quadtree_map.h"
#include "src/map_updater.h"
#include "src/obstacle_classifier.h"
#include "src/map_consistency.h"
#include "src/frontier_detector.h"
#include "src/hybrid_astar.h"
#include "src/command_gen.h"
#include "src/uart_bridge.h"
#include "src/wifi_dashboard.h"
#include "src/test/test_room.h"         /* build_test_room() */
#include "src/test/room_data.h"         /* ROOM_WIDTH_MM, ROOM_HEIGHT_MM */
#include "src/test/simulate_lidar.h"    /* slam_map_init(), simulate_and_update_map() */

#if USE_WAYPOINT_PLAYBACK
#include "src/test/room_waypoints.h"    /* ROOM_WAYPOINTS[], ROOM_WP_START_POSE */
#endif

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include <math.h>
#include <stdbool.h>

static void slam_main_task(void *arg);

/* ── Debug fallback ──────────────────────────────────────────────────────── */
#define DEBUG_FORWARD_MM   200.0f
#define DEBUG_SPEED_MM_S   100.0f

/* ── Planning loop thresholds ────────────────────────────────────────────── */
#define TARGET_REACHED_MM   150.0f
#define REPLAN_TIMEOUT_MS  5000
#define MAX_DRIVE_MS        3000u   /* must match wemos/main.c */
#define CYCLE_DELAY_MS      200     /* fallback drive extra wait (ms) */
#define SCAN_POLL_MS        50u     /* LiDAR poll interval while tracking (ms) */
#define PATH_TIMEOUT_MS     8000u   /* replan if Wemos never sends path-done */
#define QT_BCAST_PERIOD     5       /* broadcast quadtree every N planning cycles */



/* ════════════════════════════════════════════════════════════════════════════
 * USE_REAL_LIDAR — multi-task shared state and task functions
 * ════════════════════════════════════════════════════════════════════════════ */
#if USE_REAL_LIDAR

#define SCAN_NBUF 3

typedef struct {
    uint8_t  idx;
    pose_t   pre_pose;
    int64_t  pre_time_us;
    pose_t   post_pose;
    int64_t  post_time_us;
} scan_pkt_t;

static lidar_scan_t      s_scan_buf[SCAN_NBUF];
static quadtree_map_t    s_map;
static pose_t            s_pose;
static SemaphoreHandle_t s_pose_mtx;
static SemaphoreHandle_t s_map_mtx;
static SemaphoreHandle_t s_plan_trigger;
static QueueHandle_t     q_raw_scan;
static QueueHandle_t     q_frontier_for_astar;
static QueueHandle_t     q_path_tx;
static volatile bool     s_stop_requested;
static uint32_t          s_odom_last_seq;

typedef struct {
    float        fx, fy;
    bool         has_frontier;
    path_frame_t last_path;
} dash_state_t;
static dash_state_t      s_dash;
static SemaphoreHandle_t s_dash_mtx;

/* ── Core0 prio 7 ────────────────────────────────────────────────────────── */
static void task_lidar_scan(void *arg)
{
    (void)arg;
    uint8_t buf_idx = 0;
    for (;;) {
        pose_t pre_pose;
        xSemaphoreTake(s_pose_mtx, portMAX_DELAY);
        pre_pose = s_pose;
        xSemaphoreGive(s_pose_mtx);
        int64_t pre_time_us = esp_timer_get_time();

        lidar_scan_t *scan = &s_scan_buf[buf_idx];
        if (!lidar_driver_read_scan(scan) || scan->count <= 10) {
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        int64_t post_time_us = esp_timer_get_time();
        pose_t post_pose;
        xSemaphoreTake(s_pose_mtx, portMAX_DELAY);
        post_pose = s_pose;
        xSemaphoreGive(s_pose_mtx);

        scan_pkt_t pkt = {
            .idx          = buf_idx,
            .pre_pose     = pre_pose,
            .pre_time_us  = pre_time_us,
            .post_pose    = post_pose,
            .post_time_us = post_time_us,
        };
        xQueueOverwrite(q_raw_scan, &pkt);
        buf_idx = (uint8_t)((buf_idx + 1u) % SCAN_NBUF);
    }
}

/* ── Core0 prio 6 ────────────────────────────────────────────────────────── */
static void task_uart_bridge(void *arg)
{
    (void)arg;
    bool     path_active  = false;
    uint32_t path_sent_ms = 0;
    uint32_t odom_log_ctr = 0;

    for (;;) {
        /* Drain all pending odom packets → update shared pose */
        {
            odom_t odom;
            while (uart_bridge_recv_odom(&odom)) {
                if (odom.dt_ms > 0.0f) {
                    xSemaphoreTake(s_pose_mtx, portMAX_DELAY);
                    if (odom.seq != 0 && odom.seq != s_odom_last_seq)
                        printf("[ODOM] seq gap: expected %lu got %lu (integrating)\n",
                               (unsigned long)s_odom_last_seq,
                               (unsigned long)odom.seq);
                    s_odom_last_seq = odom.seq + 1;
                    float dtheta    = odom.yaw_rate_imu * (odom.dt_ms / 1000.0f);
                    float mid_theta = s_pose.theta + dtheta * 0.5f;
                    s_pose.x       += odom.linear_disp_mm * cosf(mid_theta);
                    s_pose.y       += odom.linear_disp_mm * sinf(mid_theta);
                    s_pose.theta   += dtheta;
                    while (s_pose.theta >  (float)M_PI) s_pose.theta -= 2.0f * (float)M_PI;
                    while (s_pose.theta < -(float)M_PI) s_pose.theta += 2.0f * (float)M_PI;
                    pose_t log_pose = s_pose;
                    xSemaphoreGive(s_pose_mtx);
                    if (++odom_log_ctr % 5 == 0)
                        printf("[S3 ODOM] seq=%lu disp=%.0f mm yaw=%.3f rad/s"
                               "  pose=(%.0f,%.0f,%.2f)\n",
                               (unsigned long)odom.seq,
                               (double)odom.linear_disp_mm,
                               (double)odom.yaw_rate_imu,
                               (double)log_pose.x, (double)log_pose.y,
                               (double)log_pose.theta);
                }
            }
        }

        /* Send a newly planned path to Wemos (with up to 3 ACK retries) */
        if (!path_active) {
            path_frame_t pf;
            if (xQueueReceive(q_path_tx, &pf, 0) == pdTRUE) {
                bool got_ack = false;
                for (int r = 0; r < 3 && !got_ack; r++) {
                    uart_bridge_send_path(&pf);
                    uint32_t t0 = (uint32_t)(esp_timer_get_time() / 1000ULL);
                    while ((uint32_t)(esp_timer_get_time() / 1000ULL) - t0 < 500u) {
                        odom_t _d;
                        while (uart_bridge_recv_odom(&_d)) { /* drain only */ }
                        if (uart_bridge_recv_path_ack()) { got_ack = true; break; }
                        vTaskDelay(pdMS_TO_TICKS(10));
                    }
                    if (!got_ack && r < 2)
                        wifi_dashboard_log("WARN: no path ACK — retrying...");
                }
                printf("[S3] path: %u wp  ack=%s\n",
                       (unsigned)pf.length, got_ack ? "OK" : "FAIL");
                if (got_ack) {
                    path_active  = true;
                    path_sent_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
                    xSemaphoreTake(s_dash_mtx, portMAX_DELAY);
                    s_dash.last_path = pf;
                    xSemaphoreGive(s_dash_mtx);
                    (void)uart_bridge_recv_path_done();   /* discard stale flag */
                } else {
                    wifi_dashboard_log("WARN: path send failed — replanning");
                    xSemaphoreGive(s_plan_trigger);
                }
            }
        }

        /* Check for path completion or timeout → trigger replanning */
        if (path_active) {
            if (uart_bridge_recv_path_done()) {
                wifi_dashboard_log("Path complete — replanning");
                path_active = false;
                xSemaphoreGive(s_plan_trigger);
            } else {
                uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
                if ((now_ms - path_sent_ms) >= PATH_TIMEOUT_MS) {
                    wifi_dashboard_log("Path timeout — replanning");
                    path_active = false;
                    xSemaphoreGive(s_plan_trigger);
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/* ── Core0 prio 5 ────────────────────────────────────────────────────────── */
static void task_map_update(void *arg)
{
    (void)arg;
    scan_pkt_t pkt;
    for (;;) {
        if (xQueueReceive(q_raw_scan, &pkt, portMAX_DELAY) != pdTRUE) continue;
        lidar_scan_t    *scan = &s_scan_buf[pkt.idx];
        map_dirty_rect_t dr;
        xSemaphoreTake(s_map_mtx, portMAX_DELAY);
        lidar_deskew_and_map(&s_map, scan,
                             &pkt.pre_pose,  pkt.pre_time_us,
                             &pkt.post_pose, pkt.post_time_us,
                             LIDAR_PROCESS_RANGE_MM, 150.0f, &dr);
        xSemaphoreGive(s_map_mtx);
        wifi_dashboard_mark_dirty(&dr);
        wifi_dashboard_broadcast_scan(scan, &pkt.post_pose);
    }
}

/* ── Core0 prio 3 ────────────────────────────────────────────────────────── */
static void task_wifi_ws(void *arg)
{
    (void)arg;
    uint8_t     qt_ctr     = 0;
    static bool pool_warned = false;

    for (;;) {
        if (wifi_dashboard_stop_requested()) {
            s_stop_requested = true;
            wifi_dashboard_log("Exploration stopped.");
        }
        if (s_stop_requested) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        pose_t local_pose;
        xSemaphoreTake(s_pose_mtx, portMAX_DELAY);
        local_pose = s_pose;
        xSemaphoreGive(s_pose_mtx);

        dash_state_t ds;
        xSemaphoreTake(s_dash_mtx, portMAX_DELAY);
        ds = s_dash;
        xSemaphoreGive(s_dash_mtx);

        xSemaphoreTake(s_map_mtx, portMAX_DELAY);
        bool pool_full = !pool_warned && qt_is_pool_full(&s_map);
        wifi_dashboard_update(&s_map, &local_pose);
        if (++qt_ctr >= QT_BCAST_PERIOD) {
            qt_ctr = 0;
            wifi_dashboard_broadcast_quadtree(&s_map);
        }
        xSemaphoreGive(s_map_mtx);

        if (pool_full) {
            pool_warned = true;
            wifi_dashboard_log("WARN: map pool full — map frozen");
        }

        wifi_dashboard_broadcast_state(&local_pose, ds.fx, ds.fy, ds.has_frontier, 0);
        wifi_dashboard_broadcast_path(&ds.last_path);

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

/* ── Core1 prio 6 ────────────────────────────────────────────────────────── */
static void task_hybrid_astar(void *arg)
{
    (void)arg;
    frontier_t goal;
    for (;;) {
        if (xQueueReceive(q_frontier_for_astar, &goal, portMAX_DELAY) != pdTRUE) continue;
        if (s_stop_requested) continue;

        pose_t local_pose;
        xSemaphoreTake(s_pose_mtx, portMAX_DELAY);
        local_pose = s_pose;
        xSemaphoreGive(s_pose_mtx);

        xSemaphoreTake(s_map_mtx, portMAX_DELAY);
        path_t astar_path = hybrid_astar_plan(&s_map, &local_pose, &goal);
        xSemaphoreGive(s_map_mtx);

        if (!hybrid_astar_is_valid(&astar_path)) {
            wifi_dashboard_log("A* failed — waiting");
            xSemaphoreGive(s_plan_trigger);
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        char lbuf[64];
        snprintf(lbuf, sizeof(lbuf), "A*: %u wp  target=(%.0f,%.0f)",
                 (unsigned)astar_path.length, (double)goal.cx, (double)goal.cy);
        wifi_dashboard_log(lbuf);

        uint8_t n = (astar_path.length > MAX_SHARED_PATH_POINTS)
                    ? MAX_SHARED_PATH_POINTS : (uint8_t)astar_path.length;
        path_frame_t pf = {0};
        pf.length = n;
        for (uint8_t i = 0; i < n; i++) {
            pf.waypoints[i] = astar_path.waypoints[i];
            if (pf.waypoints[i].v_target <= 10.0f)
                pf.waypoints[i].v_target = DEBUG_SPEED_MM_S;
        }
        printf("[ASTAR] pose=(%.0f,%.0f) frontier=(%.0f,%.0f)"
               "  wp[0]=(%.0f,%.0f)  wp[%u]=(%.0f,%.0f)\n",
               (double)local_pose.x, (double)local_pose.y,
               (double)goal.cx, (double)goal.cy,
               (double)pf.waypoints[0].x, (double)pf.waypoints[0].y,
               (unsigned)(n - 1u),
               (double)pf.waypoints[n - 1u].x, (double)pf.waypoints[n - 1u].y);

        xSemaphoreTake(s_dash_mtx, portMAX_DELAY);
        s_dash.fx           = goal.cx;
        s_dash.fy           = goal.cy;
        s_dash.has_frontier = true;
        xSemaphoreGive(s_dash_mtx);

        xQueueOverwrite(q_path_tx, &pf);
    }
}

/* ── Core1 prio 5 ────────────────────────────────────────────────────────── */
static void task_frontier(void *arg)
{
    (void)arg;
    for (;;) {
        xSemaphoreTake(s_plan_trigger, portMAX_DELAY);
        if (s_stop_requested) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        pose_t local_pose;
        xSemaphoreTake(s_pose_mtx, portMAX_DELAY);
        local_pose = s_pose;
        xSemaphoreGive(s_pose_mtx);

        xSemaphoreTake(s_map_mtx, portMAX_DELAY);
        frontier_list_t fl = frontier_detector_detect(&s_map, &local_pose);
        xSemaphoreGive(s_map_mtx);

        if (fl.count == 0) {
            wifi_dashboard_log("No frontiers — stopped");
            xSemaphoreTake(s_dash_mtx, portMAX_DELAY);
            s_dash.has_frontier = false;
            xSemaphoreGive(s_dash_mtx);
            vTaskDelay(pdMS_TO_TICKS(1000));
            xSemaphoreGive(s_plan_trigger);   /* retry after delay */
            continue;
        }

        frontier_t best = frontier_detector_best(&fl, &local_pose);

        xSemaphoreTake(s_dash_mtx, portMAX_DELAY);
        s_dash.has_frontier = true;
        s_dash.fx           = best.cx;
        s_dash.fy           = best.cy;
        xSemaphoreGive(s_dash_mtx);

        xQueueOverwrite(q_frontier_for_astar, &best);
    }
}

#endif /* USE_REAL_LIDAR (multi-task declarations) */


/* ════════════════════════════════════════════════════════════════════════════
 * app_main — minimal entry point; real work runs in slam_main_task (16 KB stack)
 * ════════════════════════════════════════════════════════════════════════════ */
void app_main(void)
{
    xTaskCreate(slam_main_task, "slam_main", 16384, NULL, 5, NULL);
}

static void slam_main_task(void *arg)
{
    (void)arg;
/* ════════════════════════════════════════════════════════════════════════════
 * TEST_LIDAR — read scans in a loop and print stats to serial.
 * ════════════════════════════════════════════════════════════════════════════ */
#if defined(TEST_LIDAR)

    lidar_driver_init();
    printf("[TEST_LIDAR] Waiting for scans on UART%d RX=GPIO%d TX=GPIO%d...\n",
           (int)LIDAR_UART_PORT, LIDAR_UART_RX, LIDAR_UART_TX);

    static lidar_scan_t scan;   /* ~4 KB — too large for stack, put in BSS */
    while (1) {
        if (!lidar_driver_read_scan(&scan) || scan.count == 0) {
            printf("[TEST_LIDAR] No scan — LiDAR not responding\n");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        float min_d = 99999.0f, max_d = 0.0f;
        for (int i = 0; i < (int)scan.count; i++) {
            if (scan.points[i].r_mm < min_d) min_d = scan.points[i].r_mm;
            if (scan.points[i].r_mm > max_d) max_d = scan.points[i].r_mm;
        }
        printf("[TEST_LIDAR] pts=%u  min=%.0f mm  max=%.0f mm\n",
               (unsigned)scan.count, (double)min_d, (double)max_d);
        for (int s = 0; s < 5; s++) {
            int idx = (int)scan.count * s / 5;
            printf("  [%3d] %6.1f deg  %6.0f mm\n",
                   idx,
                   (double)scan.points[idx].theta_deg,
                   (double)scan.points[idx].r_mm);
        }
    }


/* ════════════════════════════════════════════════════════════════════════════
 * TEST_BRIDGE_TX — send test control frames to Wemos every 2 s.
 * Flash Wemos with TEST_BRIDGE_RX to see the decoded frames on its serial.
 * ════════════════════════════════════════════════════════════════════════════ */
#elif defined(TEST_BRIDGE_TX)

    uart_bridge_init();
    printf("[TEST_BRIDGE_TX] Sending curved path every 500 ms on UART%d"
           "  TX=GPIO%d  RX=GPIO%d  %d baud\n",
           (int)BRIDGE_UART_PORT,
           (int)BRIDGE_TX_PIN, (int)BRIDGE_RX_PIN,
           BRIDGE_BAUD);

       uint32_t n = 0;

    while (1) {
        path_frame_t path_frame = {0};

        path_frame.length = 5;

        path_frame.waypoints[0].x = 0.0f;
        path_frame.waypoints[0].y = 0.0f;
        path_frame.waypoints[0].theta = 0.0f;
        path_frame.waypoints[0].v_target = 150.0f;

        path_frame.waypoints[1].x = 300.0f;
        path_frame.waypoints[1].y = 0.0f;
        path_frame.waypoints[1].theta = 0.0f;
        path_frame.waypoints[1].v_target = 150.0f;

        path_frame.waypoints[2].x = 600.0f;
        path_frame.waypoints[2].y = 150.0f;
        path_frame.waypoints[2].theta = 0.2f;
        path_frame.waypoints[2].v_target = 150.0f;

        path_frame.waypoints[3].x = 850.0f;
        path_frame.waypoints[3].y = 350.0f;
        path_frame.waypoints[3].theta = 0.4f;
        path_frame.waypoints[3].v_target = 150.0f;

        path_frame.waypoints[4].x = 1000.0f;
        path_frame.waypoints[4].y = 600.0f;
        path_frame.waypoints[4].theta = 0.6f;
        path_frame.waypoints[4].v_target = 150.0f;

        bool ok = uart_bridge_send_path(&path_frame);

        printf("[TEST_BRIDGE_TX] Path #%u  length=%u  %s\n",
               (unsigned)++n,
               (unsigned)path_frame.length,
               ok ? "sent" : "UART FAIL");

        vTaskDelay(pdMS_TO_TICKS(500));
    }


/* ════════════════════════════════════════════════════════════════════════════
 * TEST_BRIDGE_PING — interactive bidirectional raw UART test.
 * Press ENTER here → sends "hey1 #N" to Wemos.
 * Anything received from Wemos is printed immediately.
 * Flash Wemos with TEST_BRIDGE_PONG to close the loop.
 * ════════════════════════════════════════════════════════════════════════════ */
#elif defined(TEST_BRIDGE_PING)

    uart_config_t ping_cfg = {
        .baud_rate  = BRIDGE_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_param_config(BRIDGE_UART_PORT, &ping_cfg);
    uart_set_pin(BRIDGE_UART_PORT, BRIDGE_TX_PIN, BRIDGE_RX_PIN,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(BRIDGE_UART_PORT, 512, 512, 0, NULL, 0);

    printf("[ESP32-S3] Ping test on UART%d  TX=GPIO%d  RX=GPIO%d  %d baud\n",
           (int)BRIDGE_UART_PORT, (int)BRIDGE_TX_PIN, (int)BRIDGE_RX_PIN,
           BRIDGE_BAUD);
    printf("[ESP32-S3] Press ENTER to send 'hey1' to Wemos\n");

    uint32_t ping_n = 0;
    while (1) {
        /* Print anything that arrived from Wemos */
        size_t avail = 0;
        uart_get_buffered_data_len(BRIDGE_UART_PORT, &avail);
        if (avail > 0) {
            uint8_t rbuf[64] = {0};
            int got = uart_read_bytes(BRIDGE_UART_PORT, rbuf,
                                      avail < 63 ? (int)avail : 63, 0);
            if (got > 0) {
                if (rbuf[got - 1] == '\n') rbuf[got - 1] = '\0';
                printf("[ESP32-S3] Received: %s\n", (char *)rbuf);
            }
        }

        /* Send on ENTER keypress */
        int c = getchar();
        if (c != EOF && c != '\r' && c != '\n') {
            while (getchar() != EOF);
            ping_n++;
            char msg[32];
            int mlen = snprintf(msg, sizeof(msg), "hey1 #%u\n", (unsigned)ping_n);
            uart_write_bytes(BRIDGE_UART_PORT, msg, mlen);
            printf("[ESP32-S3] Sent: hey1 #%u\n", (unsigned)ping_n);
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }


#else  /* ── Normal SLAM operation ───────────────────────────────────────── */

    /* ── Hardware init ───────────────────────────────────────────────────── */
    uart_bridge_init();
    vTaskDelay(pdMS_TO_TICKS(1000));   /* let Wemos boot */

    /* ── Wi-Fi + WebSocket dashboard ─────────────────────────────────────── */
    wifi_dashboard_init(WIFI_SSID, WIFI_PASSWORD);


/* ════════════════════════════════════════════════════════════════════════════
 * MODE R — Real LiDAR hardware: RPLiDAR C1 → quadtree map → live dashboard.
 *
 * To use:
 *   1. Wire RPLiDAR C1 to ESP32-S3 UART (lidar_driver.h for pin config).
 *   2. Flash with USE_REAL_LIDAR=1, USE_WAYPOINT_PLAYBACK=0.
 *   3. Open tools/dashboard/live_dashboard.html in your browser.
 *   4. Enter ESP32 IP, click "Start Exploration".
 * ════════════════════════════════════════════════════════════════════════════ */
#if USE_REAL_LIDAR

    lidar_driver_init();

    /* 10 m × 10 m map, robot starts at centre (5000, 5000) mm */
    quadtree_map_init(&s_map, 10000.0f, 10000.0f, 50.0f);
    s_pose = (pose_t){ .x = 5000.0f, .y = 5000.0f, .theta = 0.0f };

    s_pose_mtx           = xSemaphoreCreateMutex();
    s_map_mtx            = xSemaphoreCreateMutex();
    s_plan_trigger       = xSemaphoreCreateBinary();
    s_dash_mtx           = xSemaphoreCreateMutex();
    q_raw_scan           = xQueueCreate(1, sizeof(scan_pkt_t));
    q_frontier_for_astar = xQueueCreate(1, sizeof(frontier_t));
    q_path_tx            = xQueueCreate(1, sizeof(path_frame_t));

    /* Launch all tasks immediately so the LiDAR scans, the map builds, and
     * the WebSocket dashboard stays alive while we wait for Start.
     * task_frontier and task_hybrid_astar block on s_plan_trigger /
     * q_frontier_for_astar — they are harmlessly idle until Start is pressed. */
    xTaskCreatePinnedToCore(task_lidar_scan,   "lscan",    4096, NULL, 7, NULL, 0);
    xTaskCreatePinnedToCore(task_uart_bridge,  "uart_br",  4096, NULL, 6, NULL, 0);
    xTaskCreatePinnedToCore(task_map_update,   "mapupd",   6144, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(task_wifi_ws,      "wifi_ws",  6144, NULL, 3, NULL, 0);
    xTaskCreatePinnedToCore(task_hybrid_astar, "astar",    8192, NULL, 6, NULL, 1);
    xTaskCreatePinnedToCore(task_frontier,     "frontier", 6144, NULL, 5, NULL, 1);

    /* Wait for the browser Start button — yield every 200 ms so dash_task
     * (priority 1) gets CPU time and the WebSocket stays connected. */
    while (!wifi_dashboard_exploration_requested()) {
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    /* Start pressed — release frontier detector and exit. */
    xSemaphoreGive(s_plan_trigger);
    vTaskDelete(NULL);


/* ════════════════════════════════════════════════════════════════════════════
 * MODE A — Waypoint playback: follow room.log trajectory physically.
 *
 * The car drives through ROOM_WAYPOINTS[] derived from the recorded poses.
 * No frontier detection — the map is built in the browser (playback_dashboard)
 * from room_scan_data.js, synchronised to the "si" field broadcast each cycle.
 *
 * To use:
 *   1. Run: python3 tools/parse_room/generate_lidar_map.py
 *   2. Flash with USE_WAYPOINT_PLAYBACK=1
 *   3. Open tools/dashboard/playback_dashboard.html in your browser
 *   4. Enter ESP32 IP and connect
 * ════════════════════════════════════════════════════════════════════════════ */
#elif USE_WAYPOINT_PLAYBACK

    /* Dummy map (not used for planning; browser handles map display) */
    quadtree_map_t map;
    quadtree_map_init(&map, 10000.0f, 10000.0f, 50.0f);

    /* Starting pose matches ROOM_WAYPOINTS[0] */
    pose_t   pose   = ROOM_WP_START_POSE;
    uint16_t wp_idx = 0;

    while (1) {

        /* ── Trajectory complete ─────────────────────────────────────────── */
        if (wp_idx >= ROOM_WAYPOINT_COUNT) {
            /* Broadcast final position so dashboard shows completed path */
            wifi_dashboard_broadcast_state(
                &pose, pose.x, pose.y, false,
                ROOM_WAYPOINT_SCAN_IDX[ROOM_WAYPOINT_COUNT - 1u]);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        const waypoint_t *wp = &ROOM_WAYPOINTS[wp_idx];

        /* ── Check proximity to current waypoint ─────────────────────────── */
        float dx   = wp->x - pose.x;
        float dy   = wp->y - pose.y;
        float dist = sqrtf(dx * dx + dy * dy);

        if (dist < TARGET_REACHED_MM) {
            /* Advance to next waypoint without sending a motor command */
            wp_idx++;
            continue;
        }

        /* ── Compute command toward current waypoint ─────────────────────── */
        control_frame_t cmd = command_gen_compute(&pose, wp);

        /* ── Transmit to Wemos ───────────────────────────────────────────── */
        uart_bridge_send_control(&cmd);

        /* ── Broadcast state: pose + next-waypoint target + current scan ─── */
        uint16_t scan_idx = ROOM_WAYPOINT_SCAN_IDX[wp_idx];
        wifi_dashboard_broadcast_state(&pose, wp->x, wp->y, true, scan_idx);

        /* ── Dead-reckon pose ────────────────────────────────────────────── */
        dead_reckon_pose(&pose, &cmd);

        /* ── Wait for Wemos to execute the command ───────────────────────── */
        float dist_cmd = sqrtf(cmd.tx * cmd.tx + cmd.ty * cmd.ty);
        float speed    = (cmd.t_speed > 10.0f) ? cmd.t_speed : DEBUG_SPEED_MM_S;
        uint32_t drive_ms = (uint32_t)((dist_cmd / speed) * 1000.0f);
        if (drive_ms > MAX_DRIVE_MS) drive_ms = MAX_DRIVE_MS;
        if (drive_ms < 50)           drive_ms = 50;
        vTaskDelay(pdMS_TO_TICKS(drive_ms + CYCLE_DELAY_MS));
    }


/* ════════════════════════════════════════════════════════════════════════════
 * MODE B/C — Frontier exploration OR UART smoke test (simulated test room)
 * ════════════════════════════════════════════════════════════════════════════ */
#else   /* !USE_REAL_LIDAR && !USE_WAYPOINT_PLAYBACK */

    /* ── Two maps for realistic simulation ──────────────────────────────── */
    /* truth_map: full room (all walls pre-loaded) — used only for ray-casting */
    /* slam_map:  what the car has actually seen   — shown on dashboard        */
    quadtree_map_t truth_map;
    quadtree_map_t slam_map;
    pose_t pose = {0};

    build_test_room(&truth_map, &pose);
    slam_map_init(&slam_map, &pose,
                  ROOM_WIDTH_MM, ROOM_HEIGHT_MM,
                  50.0f, BUILD_FREE_DISK_MM);

    /* Run an initial scan at the start pose so the dashboard shows something */
    simulate_and_update_map(&truth_map, &slam_map, &pose, 180, 6000.0f, 50.0f);

    /* ── Wait for browser to send {"cmd":"start"} ────────────────────────── */
    while (!wifi_dashboard_exploration_requested()) {
        wifi_dashboard_broadcast_state(&pose, 0.0f, 0.0f, false, 0);
        wifi_dashboard_update(&slam_map, &pose);
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    /* ── Planning loop ───────────────────────────────────────────────────── */
    while (1) {

        /* ── Frontier detection on slam_map (what the car actually knows) ── */
        frontier_list_t frontiers = frontier_detector_detect(&slam_map, &pose);

        control_frame_t cmd = {0};
        bool has_frontier   = false;
        float fx = 0.0f, fy = 0.0f;

#if USE_FRONTIER_TARGET

        /* ── Real mode: frontier → command_gen ──────────────────────────── */
        if (frontiers.count > 0) {
            frontier_t best = frontier_detector_best(&frontiers, &pose);
            waypoint_t wp   = {0};
            wp.x            = best.cx;
            wp.y            = best.cy;
            cmd             = command_gen_compute(&pose, &wp);
            has_frontier    = true;
            fx              = best.cx;
            fy              = best.cy;
        } else {
            cmd.tx        = 0.0f;
            cmd.ty        = DEBUG_FORWARD_MM;
            cmd.t_heading = pose.theta;
            cmd.t_speed   = DEBUG_SPEED_MM_S;
        }

#else   /* USE_FRONTIER_TARGET == 0 — UART + motor smoke test */

        (void)frontiers;
        cmd.tx        = 0.0f;
        cmd.ty        = DEBUG_FORWARD_MM;
        cmd.t_heading = pose.theta;
        cmd.t_speed   = DEBUG_SPEED_MM_S;

#endif  /* USE_FRONTIER_TARGET */

        /* ── Transmit command to Wemos ───────────────────────────────────── */
        uart_bridge_send_control(&cmd);

        /* ── Broadcast state (scan_idx=0 in non-playback modes) ─────────── */
        wifi_dashboard_broadcast_state(&pose, fx, fy, has_frontier, 0);

        /* ── Dead-reckon pose ────────────────────────────────────────────── */
        dead_reckon_pose(&pose, &cmd);

        /* ── Wait for Wemos to finish driving ────────────────────────────── */
        float dist_mm = sqrtf(cmd.tx * cmd.tx + cmd.ty * cmd.ty);
        float speed   = (cmd.t_speed > 10.0f) ? cmd.t_speed : DEBUG_SPEED_MM_S;
        uint32_t drive_ms = (uint32_t)((dist_mm / speed) * 1000.0f);
        if (drive_ms > MAX_DRIVE_MS) drive_ms = MAX_DRIVE_MS;
        if (drive_ms < 50)           drive_ms = 50;
        vTaskDelay(pdMS_TO_TICKS(drive_ms + CYCLE_DELAY_MS));

        /* ── Simulate LiDAR at new pose, update slam_map ─────────────────── */
        simulate_and_update_map(&truth_map, &slam_map, &pose, 180, 6000.0f, 50.0f);

        /* ── Push updated slam_map to dashboard ──────────────────────────── */
        wifi_dashboard_update(&slam_map, &pose);
    }

#endif  /* USE_REAL_LIDAR / USE_WAYPOINT_PLAYBACK */

#endif  /* TEST_LIDAR / TEST_IMU / TEST_BRIDGE_TX / else (normal SLAM) */

    vTaskDelete(NULL);
}
