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
#define USE_REAL_LIDAR         0
#define USE_WAYPOINT_PLAYBACK  0
#define USE_FRONTIER_TARGET    0
#define UART_SMOKE_TEST        1

/* ── Wi-Fi credentials — fill in before flashing ───────────────────────── */
#define WIFI_SSID      "YOUR_SSID"
#define WIFI_PASSWORD  "YOUR_PASSWORD"

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
#include "src/test/test_room.h"         /* build_test_room() */
#include "src/test/room_data.h"         /* ROOM_WIDTH_MM, ROOM_HEIGHT_MM */
#include "src/test/simulate_lidar.h"
#include "src/uart_bridge.h"   /* slam_map_init(), simulate_and_update_map() */

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
    printf("[ESP32-S3] app_main started\n");   // ADD THIS
    uart_bridge_init();
    printf("[ESP32-S3] uart_bridge_init done\n");  // ADD THIS
    vTaskDelay(pdMS_TO_TICKS(1000));

#if UART_SMOKE_TEST
    printf("[ESP32-S3] entering smoke test loop\n");  // ADD THIS
    while (1) {
        control_frame_t test_cmd = {
            .tx = 100.0f,
            .ty = 0.0f,
            .t_heading = 0.0f,
            .t_speed = 100.0f,
        };
        printf("[ESP32-S3] sending control frame\n");  // ADD THIS
        uart_bridge_send_control(&test_cmd);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
#endif

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
    lidar_scan_t scan;

    /* Wait for browser to send {"cmd":"start"} before driving */
    while (!wifi_dashboard_exploration_requested()) {
        wifi_dashboard_update(&slam_map, &pose);
        wifi_dashboard_broadcast_state(&pose, 0.0f, 0.0f, false, 0);
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    while (1) {

        /* ── Acquire scan, integrate into map ───────────────────────────── */
        if (lidar_driver_read_scan(&scan) && scan.count > 10) {
            lidar_to_map(&slam_map, &scan, &pose, 6000.0f, 50.0f);
        }

        /* ── Push updated map to dashboard ──────────────────────────────── */
        wifi_dashboard_update(&slam_map, &pose);

        /* ── Frontier detection ──────────────────────────────────────────── */
        frontier_list_t frontiers = frontier_detector_detect(&slam_map, &pose);

        control_frame_t cmd = {0};
        bool has_frontier   = false;
        float fx = 0.0f, fy = 0.0f;

        if (frontiers.count > 0) {
            frontier_t best = frontier_detector_best(&frontiers, &pose);
            waypoint_t wp   = { .x = best.cx, .y = best.cy };
            cmd             = command_gen_compute(&pose, &wp);
            has_frontier    = true;
            fx              = best.cx;
            fy              = best.cy;
        } else {
            /* No frontier yet — nudge forward slowly */
            cmd.tx        = 0.0f;
            cmd.ty        = DEBUG_FORWARD_MM;
            cmd.t_heading = pose.theta;
            cmd.t_speed   = DEBUG_SPEED_MM_S;
        }

        /* ── Transmit command to Wemos ───────────────────────────────────── */
        uart_bridge_send_control(&cmd);

        /* ── Broadcast state ─────────────────────────────────────────────── */
        wifi_dashboard_broadcast_state(&pose, fx, fy, has_frontier, 0);

        /* ── Dead-reckon pose from command ───────────────────────────────── */
        dead_reckon_pose(&pose, &cmd);

        /* ── Wait for Wemos to finish driving ────────────────────────────── */
        float dist_mm = sqrtf(cmd.tx * cmd.tx + cmd.ty * cmd.ty);
        float speed   = (cmd.t_speed > 10.0f) ? cmd.t_speed : DEBUG_SPEED_MM_S;
        uint32_t drive_ms = (uint32_t)((dist_mm / speed) * 1000.0f);
        if (drive_ms > MAX_DRIVE_MS) drive_ms = MAX_DRIVE_MS;
        if (drive_ms < 50u)          drive_ms = 50u;
        vTaskDelay(pdMS_TO_TICKS(drive_ms + CYCLE_DELAY_MS));
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
}
