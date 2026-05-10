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
 * dead_reckon_pose
 * Update pose estimate from the command just sent.  Since we have no odometry
 * yet, we assume the car traveled min(dist_mm/speed * 1s, MAX_DRIVE_MS ms)
 * in the commanded direction.
 * ════════════════════════════════════════════════════════════════════════════ */
static void dead_reckon_pose(pose_t *pose, const control_frame_t *cmd)
{
    pose->theta = cmd->t_heading;
    float dist_mm = sqrtf(cmd->tx * cmd->tx + cmd->ty * cmd->ty);
    if (dist_mm < 1.0f) return;

    float speed = (cmd->t_speed > 10.0f) ? cmd->t_speed : DEBUG_SPEED_MM_S;
    float dt_s  = dist_mm / speed;
    float max_s = MAX_DRIVE_MS / 1000.0f;
    if (dt_s > max_s) dt_s = max_s;

    float traveled = speed * dt_s;
    pose->x += traveled * cosf(cmd->t_heading);
    pose->y += traveled * sinf(cmd->t_heading);
}


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
    quadtree_map_t slam_map;
    quadtree_map_init(&slam_map, 10000.0f, 10000.0f, 50.0f);

    pose_t pose = { .x = 5000.0f, .y = 5000.0f, .theta = 0.0f };
    static lidar_scan_t scan;   /* ~4 KB — too large for stack, put in BSS */

    /* Wait for browser to send {"cmd":"start"} before driving.
     * Keep scanning so the map populates live even while stationary. */
    while (!wifi_dashboard_exploration_requested()) {
        /* Drain idle heading updates from Wemos so the car icon and scan
         * projection stay correct while the user rotates the car by hand. */
        {
            odom_t odom;
            while (uart_bridge_recv_odom(&odom)) {
                if (odom.dt_ms > 0.0f) {
                    float dtheta    = odom.yaw_rate_imu * (odom.dt_ms / 1000.0f);
                    float mid_theta = pose.theta + dtheta * 0.5f;
                    pose.x    += odom.linear_disp_mm * cosf(mid_theta);
                    pose.y    += odom.linear_disp_mm * sinf(mid_theta);
                    pose.theta += dtheta;
                    while (pose.theta >  (float)M_PI) pose.theta -= 2.0f * (float)M_PI;
                    while (pose.theta < -(float)M_PI) pose.theta += 2.0f * (float)M_PI;
                }
            }
        }
        if (lidar_driver_read_scan(&scan) && scan.count > 10) {
            /* Drain odom that arrived during the ~200 ms blocking scan so the
             * map projection uses the heading at scan-end, not scan-start. */
            {
                odom_t odom;
                while (uart_bridge_recv_odom(&odom)) {
                    if (odom.dt_ms > 0.0f) {
                        float dtheta    = odom.yaw_rate_imu * (odom.dt_ms / 1000.0f);
                        float mid_theta = pose.theta + dtheta * 0.5f;
                        pose.x    += odom.linear_disp_mm * cosf(mid_theta);
                        pose.y    += odom.linear_disp_mm * sinf(mid_theta);
                        pose.theta += dtheta;
                        while (pose.theta >  (float)M_PI) pose.theta -= 2.0f * (float)M_PI;
                        while (pose.theta < -(float)M_PI) pose.theta += 2.0f * (float)M_PI;
                    }
                }
            }
            map_dirty_rect_t dr;
            lidar_to_map(&slam_map, &scan, &pose, 6000.0f, 150.0f, &dr);
            wifi_dashboard_mark_dirty(&dr);
            wifi_dashboard_broadcast_scan(&scan, &pose);
        }
        wifi_dashboard_update(&slam_map, &pose);
        wifi_dashboard_broadcast_state(&pose, 0.0f, 0.0f, false, 0);
    }

    uint8_t         qt_bcast_ctr    = 0;
    path_t          astar_path      = {0};
    bool            path_active     = false;
    uint32_t        path_sent_ms    = 0;
    float           goal_x          = 0.0f;
    float           goal_y          = 0.0f;
    path_frame_t    last_path       = {0};
    /* Odom seq tracking — shared by both drain sites so gaps aren't
     * falsely reported when packets arrive during the blocking LiDAR scan. */
    uint32_t        odom_last_seq   = 0;
    uint32_t        odom_log_ctr    = 0;

    while (1) {

        /* ── Emergency stop from dashboard ───────────────────────────────── */
        if (wifi_dashboard_stop_requested()) {
            wifi_dashboard_log("Exploration stopped.");
            break;
        }

        /* ── Pose update — drain ALL pending odom packets ──────────────────── */
        {
            odom_t odom;
            while (uart_bridge_recv_odom(&odom)) {
                if (odom.dt_ms > 0.0f) {
                    if (odom.seq != 0 && odom.seq != odom_last_seq)
                        printf("[ODOM] seq gap: expected %lu got %lu (integrating)\n",
                               (unsigned long)odom_last_seq,
                               (unsigned long)odom.seq);
                    odom_last_seq = odom.seq + 1;
                    float dtheta    = odom.yaw_rate_imu * (odom.dt_ms / 1000.0f);
                    float mid_theta = pose.theta + dtheta * 0.5f;
                    pose.x    += odom.linear_disp_mm * cosf(mid_theta);
                    pose.y    += odom.linear_disp_mm * sinf(mid_theta);
                    pose.theta += dtheta;
                    while (pose.theta >  (float)M_PI) pose.theta -= 2.0f * (float)M_PI;
                    while (pose.theta < -(float)M_PI) pose.theta += 2.0f * (float)M_PI;
                    /* Log every 5th odom packet so serial isn't flooded */
                    if (++odom_log_ctr % 5 == 0)
                        printf("[S3 ODOM] seq=%lu disp=%.0f mm yaw=%.3f rad/s"
                               "  pose=(%.0f,%.0f,%.2f)\n",
                               (unsigned long)odom.seq,
                               (double)odom.linear_disp_mm,
                               (double)odom.yaw_rate_imu,
                               (double)pose.x, (double)pose.y, (double)pose.theta);
                }
            }
        }

        /* ── LiDAR scan → quadtree map update (with de-skewing) ─────────── */
        {
            pose_t  pre_scan_pose  = pose;
            int64_t pre_scan_time  = esp_timer_get_time();

            if (lidar_driver_read_scan(&scan) && scan.count > 10) {

                /* Drain odom that arrived during the blocking scan (~200 ms).
                 * Advances odom_last_seq so the next main drain doesn't log
                 * false seq-gap warnings for packets consumed here. */
                {
                    odom_t odom;
                    while (uart_bridge_recv_odom(&odom)) {
                        if (odom.dt_ms > 0.0f) {
                            odom_last_seq = odom.seq + 1;
                            float dtheta    = odom.yaw_rate_imu * (odom.dt_ms / 1000.0f);
                            float mid_theta = pose.theta + dtheta * 0.5f;
                            pose.x    += odom.linear_disp_mm * cosf(mid_theta);
                            pose.y    += odom.linear_disp_mm * sinf(mid_theta);
                            pose.theta += dtheta;
                            while (pose.theta >  (float)M_PI) pose.theta -= 2.0f * (float)M_PI;
                            while (pose.theta < -(float)M_PI) pose.theta += 2.0f * (float)M_PI;
                        }
                    }
                }

                pose_t  post_scan_pose = pose;
                int64_t post_scan_time = esp_timer_get_time();

                map_dirty_rect_t dr;
                lidar_deskew_and_map(&slam_map, &scan,
                                     &pre_scan_pose,  pre_scan_time,
                                     &post_scan_pose, post_scan_time,
                                     6000.0f, 150.0f, &dr);
                wifi_dashboard_mark_dirty(&dr);
                wifi_dashboard_broadcast_scan(&scan, &pose);
            }
        }

        /* ── Dashboard map push; periodic quadtree frame ─────────────────── */
        wifi_dashboard_update(&slam_map, &pose);
        if (++qt_bcast_ctr >= QT_BCAST_PERIOD) {
            qt_bcast_ctr = 0;
            wifi_dashboard_broadcast_quadtree(&slam_map);
        }

        /* ── Tracking mode — wait for Wemos to complete the current path ─── *
         * Frontier detection and A* are NOT run until Wemos signals path done.*
         * LiDAR scanning and map updates continue every SCAN_POLL_MS so the  *
         * FIFO never overflows during the drive phase.                        */
        if (path_active) {
            bool path_done = uart_bridge_recv_path_done();

            if (!path_done) {
                uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
                if ((now_ms - path_sent_ms) < PATH_TIMEOUT_MS) {
                    wifi_dashboard_broadcast_state(&pose, goal_x, goal_y, true, 0);
                    wifi_dashboard_broadcast_path(&last_path);
                    vTaskDelay(pdMS_TO_TICKS(SCAN_POLL_MS));
                    continue;
                }
                wifi_dashboard_log("Path timeout — replanning");
            } else {
                wifi_dashboard_log("Path complete — replanning");
            }
            path_active = false;
        }

        /* ── Planning: frontier detection ────────────────────────────────── */
        frontier_list_t frontiers = frontier_detector_detect(&slam_map, &pose);

        bool has_frontier   = false;
        float fx = 0.0f, fy = 0.0f;

        if (frontiers.count > 0) {
            frontier_t best = frontier_detector_best(&frontiers, &pose);
            has_frontier = true;
            fx = best.cx;
            fy = best.cy;

            astar_path = hybrid_astar_plan(&slam_map, &pose, &best);

            if (hybrid_astar_is_valid(&astar_path)) {
                char lbuf[64];
                snprintf(lbuf, sizeof(lbuf), "A*: %u wp  target=(%.0f,%.0f)",
                         (unsigned)astar_path.length, (double)fx, (double)fy);
                wifi_dashboard_log(lbuf);
            } else {
                wifi_dashboard_log("A* failed — waiting");
            }
        } else {
            wifi_dashboard_log("No frontiers — stopped");
        }

        /* ── Transmit to Wemos ───────────────────────────────────────────── */
        bool sent_path = false;
        path_frame_t path_frame = {0};

        if (has_frontier && hybrid_astar_is_valid(&astar_path)) {
            uint8_t n = (astar_path.length > MAX_SHARED_PATH_POINTS)
                        ? MAX_SHARED_PATH_POINTS
                        : (uint8_t)astar_path.length;
            path_frame.length = n;
            for (uint8_t i = 0; i < n; i++) {
                path_frame.waypoints[i] = astar_path.waypoints[i];
                if (path_frame.waypoints[i].v_target <= 10.0f)
                    path_frame.waypoints[i].v_target = DEBUG_SPEED_MM_S;
            }
            /* Send path and wait for Wemos ACK (MSG_PATH_ACK).
             * Up to 3 attempts × 500 ms window each = 1.5 s worst-case.
             * PATH_TIMEOUT_MS starts only after ACK — no more timing races.
             * Odom packets arriving during the wait are drained but discarded
             * (pre-drive odom is low-value; pose fusion resumes once active). */
            bool got_ack = false;
            for (int _r = 0; _r < 3 && !got_ack; _r++) {
                uart_bridge_send_path(&path_frame);
                uint32_t t0 = (uint32_t)(esp_timer_get_time() / 1000ULL);
                while ((uint32_t)(esp_timer_get_time() / 1000ULL) - t0 < 500u) {
                    odom_t _d;
                    while (uart_bridge_recv_odom(&_d)) { /* drain only */ }
                    if (uart_bridge_recv_path_ack()) { got_ack = true; break; }
                    vTaskDelay(pdMS_TO_TICKS(10));
                }
                if (!got_ack && _r < 2) {
                    wifi_dashboard_log("WARN: no path ACK — retrying...");
                }
            }
            sent_path = got_ack;
            printf("[S3] path: %u wp  frontiers=%u  ack=%s\n",
                   (unsigned)n, (unsigned)frontiers.count,
                   got_ack ? "OK" : "FAIL");
            printf("[ASTAR] pose=(%.0f,%.0f) frontier=(%.0f,%.0f)"
                   "  wp[0]=(%.0f,%.0f)  wp[%u]=(%.0f,%.0f)\n",
                   (double)pose.x, (double)pose.y,
                   (double)fx, (double)fy,
                   (double)path_frame.waypoints[0].x,
                   (double)path_frame.waypoints[0].y,
                   (unsigned)(n - 1u),
                   (double)path_frame.waypoints[n - 1u].x,
                   (double)path_frame.waypoints[n - 1u].y);
        }

        /* ── Broadcast pose + frontier target + path to dashboard ────────── */
        {
            static bool s_pool_warned = false;
            if (!s_pool_warned && qt_is_pool_full(&slam_map)) {
                s_pool_warned = true;
                wifi_dashboard_log("WARN: map pool full — map frozen");
            }
        }
        wifi_dashboard_broadcast_state(&pose, fx, fy, has_frontier, 0);
        wifi_dashboard_broadcast_path(&path_frame);

        /* ── Arm tracking state or yield until next scan ─────────────────── */
        if (sent_path) {
            path_active  = true;
            path_sent_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
            goal_x       = fx;
            goal_y       = fy;
            last_path    = path_frame;
            /* Discard any stale done flag that arrived from the previous path. */
            (void)uart_bridge_recv_path_done();
        } else {
            /* lidar_driver_read_scan() blocks ~200 ms per scan, pacing the loop at
             * ~5 Hz naturally. A longer sleep here would overflow the 5000-byte LiDAR
             * UART FIFO (fills in ≈480 ms at 7 Hz) and corrupt the next scan. */
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }


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
