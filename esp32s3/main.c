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
#define USE_LOCAL_PLANNER      0   /* ← SET TO 1 TO ENABLE HYBRID A* + LOCAL PLANNER */

/* ── Wi-Fi credentials — fill in before flashing ───────────────────────── */
#define WIFI_SSID      "SPOT-iot"
#define WIFI_PASSWORD  "RacailleSalutaireMigration8052"

#include "../hardware_pins.h"
#include "src/lidar_driver.h"
#include "src/lidar_to_map.h"
#include "src/polar_to_cart.h"
#include "src/scan_matcher.h"
#include "src/rbpf.h"
#include "src/quadtree_map.h"
#include "src/map_updater.h"
#include "src/obstacle_classifier.h"
#include "src/map_consistency.h"
#include "src/frontier_detector.h"
#include "src/hybrid_astar.h"
#include "src/path_refiner.h"
#include "src/command_gen.h"
#include "src/uart_bridge.h"
#include "src/wifi_dashboard.h"
#if USE_LOCAL_PLANNER
#include "src/local_planner.h"
#endif
#include "src/test/test_room.h"         /* build_test_room() */
#include "src/test/room_data.h"         /* ROOM_WIDTH_MM, ROOM_HEIGHT_MM */
#include "src/test/simulate_lidar.h"    /* slam_map_init(), simulate_and_update_map() */

#if USE_WAYPOINT_PLAYBACK
#include "src/test/room_waypoints.h"    /* ROOM_WAYPOINTS[], ROOM_WP_START_POSE */
#endif

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <stdbool.h>

/* ── Debug fallback ──────────────────────────────────────────────────────── */
#define DEBUG_FORWARD_MM   200.0f
#define DEBUG_SPEED_MM_S   100.0f

/* ── Planning loop thresholds ────────────────────────────────────────────── */
#define TARGET_REACHED_MM   150.0f
#define REPLAN_TIMEOUT_MS  5000
#define MAX_DRIVE_MS        3000u   /* must match wemos/main.c */
#define CYCLE_DELAY_MS      200     /* pause between planning cycles */


/* ════════════════════════════════════════════════════════════════════════════
 * dead_reckon_pose
 * Update pose estimate from the command just sent.  Since we have no odometry
 * yet, we assume the car traveled min(dist_mm/speed * 1s, MAX_DRIVE_MS ms)
 * in the commanded direction.
 * ════════════════════════════════════════════════════════════════════════════ */
static void dead_reckon_pose(pose_t *pose, const control_frame_t *cmd)
{
    float dist_mm = sqrtf(cmd->tx * cmd->tx + cmd->ty * cmd->ty);
    if (dist_mm < 1.0f) return;

    float speed = (cmd->t_speed > 10.0f) ? cmd->t_speed : DEBUG_SPEED_MM_S;
    float dt_s  = dist_mm / speed;
    float max_s = MAX_DRIVE_MS / 1000.0f;
    if (dt_s > max_s) dt_s = max_s;

    float traveled = speed * dt_s;
    pose->x    += traveled * cosf(cmd->t_heading);
    pose->y    += traveled * sinf(cmd->t_heading);
    pose->theta = cmd->t_heading;
}


/* ════════════════════════════════════════════════════════════════════════════
 * app_main
 * ════════════════════════════════════════════════════════════════════════════ */
void app_main(void)
{
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
    printf("[TEST_BRIDGE_TX] Sending frames every 2 s on UART%d"
           "  TX=GPIO%d  RX=GPIO%d  %d baud\n",
           (int)BRIDGE_UART_PORT,
           (int)BRIDGE_TX_PIN, (int)BRIDGE_RX_PIN,
           BRIDGE_BAUD);

    float heading = 0.0f;
    uint32_t n = 0;
    while (1) {
        control_frame_t cmd = {
            .tx        = 300.0f,
            .ty        = 0.0f,
            .t_heading = heading,
            .t_speed   = 346.0f,
        };
        bool ok = uart_bridge_send_control(&cmd);
        printf("[TEST_BRIDGE_TX] Frame #%u  hdg=%.2f rad  %s\n",
               (unsigned)++n, (double)heading, ok ? "sent" : "UART FAIL");
        heading += 0.5f;
        if (heading > (float)M_PI) heading -= 2.0f * (float)M_PI;
        vTaskDelay(pdMS_TO_TICKS(2000));
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
        if (lidar_driver_read_scan(&scan) && scan.count > 10) {
            map_dirty_rect_t dr;
            lidar_to_map(&slam_map, &scan, &pose, 6000.0f, 150.0f, &dr);
            wifi_dashboard_mark_dirty(&dr);
            wifi_dashboard_broadcast_scan(&scan, &pose);
        }
        wifi_dashboard_update(&slam_map, &pose);
        wifi_dashboard_broadcast_state(&pose, 0.0f, 0.0f, false, 0);
    }

    /* prev_cmd / prev_drv_ms track the command that was just dispatched so
     * we can (a) wait for it to finish at the TOP of the next iteration and
     * (b) dead-reckon the pose BEFORE integrating the fresh scan.
     *
     * Correct ordering mirrors the simulation pipeline:
     *   wait → dead_reckon (post-drive pose) → scan → map → frontier → drive
     *
     * First iteration: prev_drv_ms == 0 (no wait), prev_cmd zeroed (no-op
     * dead_reckon), so the robot's initial pose is used for the first scan. */
    control_frame_t prev_cmd    = {0};
    uint32_t        prev_drv_ms = 0;

#if USE_LOCAL_PLANNER
    path_t lp_path = {0};
    local_planner_init(120.0f);
#endif

    while (1) {

        /* ── Wait for the previous drive to complete ─────────────────────── */
        if (prev_drv_ms > 0)
            vTaskDelay(pdMS_TO_TICKS(prev_drv_ms + CYCLE_DELAY_MS));

        /* ── Dead-reckon to post-drive pose ──────────────────────────────────
         * Done BEFORE lidar_to_map so the scan integrates at the position the
         * robot actually occupies when stationary.  Mirrors the simulation's
         * integrate_scan(qt, scan, scan_rx, scan_ry) call order. */
        dead_reckon_pose(&pose, &prev_cmd);

        /* ── Acquire scan, integrate at post-drive pose ──────────────────── */
        if (lidar_driver_read_scan(&scan) && scan.count > 10) {
            map_dirty_rect_t dr;
            lidar_to_map(&slam_map, &scan, &pose, 6000.0f, 150.0f, &dr);
            wifi_dashboard_mark_dirty(&dr);
        }

        /* ── Push map + quadtree to dashboard ───────────────────────────── */
        wifi_dashboard_update(&slam_map, &pose);

        /* ── Frontier detection ──────────────────────────────────────────── */
        frontier_list_t frontiers = frontier_detector_detect(&slam_map, &pose);

        control_frame_t cmd = {0};
        bool has_frontier   = false;
        float fx = 0.0f, fy = 0.0f;

        if (frontiers.count > 0) {
            frontier_t best = frontier_detector_best(&frontiers, &pose);
            has_frontier    = true;
            fx              = best.cx;
            fy              = best.cy;
#if USE_LOCAL_PLANNER
            /* Replan when requested or path expired */
            if (local_planner_replan_needed() || !hybrid_astar_is_valid(&lp_path)) {
                lp_path = hybrid_astar_plan(&slam_map, &pose, &best);
                if (hybrid_astar_is_valid(&lp_path)) {
                    local_planner_reset_waypoint();
                    local_planner_clear_replan();
                }
            }
            if (!local_planner_update(&slam_map, &pose, &lp_path, false, &cmd)) {
                waypoint_t wp = { .x = best.cx, .y = best.cy };
                cmd           = command_gen_compute(&pose, &wp);
            }
#else
            waypoint_t wp = { .x = best.cx, .y = best.cy };
            cmd           = command_gen_compute(&pose, &wp);
#endif
        } else {
            cmd.tx        = 0.0f;
            cmd.ty        = DEBUG_FORWARD_MM;
            cmd.t_heading = pose.theta;
            cmd.t_speed   = DEBUG_SPEED_MM_S;
        }

        /* ── Transmit command to Wemos ───────────────────────────────────── */
        uart_bridge_send_control(&cmd);

        /* ── Broadcast state ─────────────────────────────────────────────── */
        wifi_dashboard_broadcast_state(&pose, fx, fy, has_frontier, 0);

        /* ── Compute drive duration; save command for next iteration ──────── */
        float    dist_mm = sqrtf(cmd.tx * cmd.tx + cmd.ty * cmd.ty);
        float    spd     = (cmd.t_speed > 10.0f) ? cmd.t_speed : DEBUG_SPEED_MM_S;
        prev_drv_ms      = (uint32_t)((dist_mm / spd) * 1000.0f);
        if (prev_drv_ms > MAX_DRIVE_MS) prev_drv_ms = MAX_DRIVE_MS;
        if (prev_drv_ms < 50u)          prev_drv_ms = 50u;
        prev_cmd = cmd;
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
}
