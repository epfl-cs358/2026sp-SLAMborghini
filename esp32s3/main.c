/**
 * esp32s3/main.c  —  Phase 2: LiDAR + Odometry → Quadtree → Dashboard
 *
 * ════════════════════════════════════════════════════════════════════════════
 * WHAT THIS DOES
 * ════════════════════════════════════════════════════════════════════════════
 * Two tasks on Core 0 run the sensing + localisation pipeline:
 *
 *   task_lidar_slam  (prio 7)
 *     1. Read one 360° LiDAR scan  (~100 ms)
 *     2. Snapshot the current pose AFTER the scan (most current estimate)
 *     3. Ray-march the scan into the quadtree occupancy map  (~15 ms)
 *     4. Push updated map + scan overlay to the browser dashboard
 *     5. Sleep 100 ms to yield CPU to WiFi / httpd / _dash_task
 *     Repeat.
 *
 *   task_odom  (prio 6)
 *     Drains odom_t packets from the Wemos over UART bridge at 100 Hz.
 *     Each packet carries linear displacement (mm) and IMU yaw rate (rad/s).
 *     Integrates them into s_pose using midpoint Runge-Kutta under mutex.
 *
 * Result on the dashboard:
 *   - Move the car forward → car marker moves across the fixed map
 *   - Turn the car in place → car marker rotates; the MAP stays fixed
 *   The map is always in a fixed world frame. Only the car pose changes.
 *
 * ════════════════════════════════════════════════════════════════════════════
 * TASKS
 * ════════════════════════════════════════════════════════════════════════════
 *   task_lidar_slam   Core 0  prio 7   stack 6 KB
 *   task_odom         Core 0  prio 6   stack 3 KB
 *   task_perf_mon     Core 0  prio 1   stack 3 KB
 *   _dash_task        Core 0  prio 1   stack 4 KB   (inside wifi_dashboard.c)
 *
 * ════════════════════════════════════════════════════════════════════════════
 * MEMORY LAYOUT
 * ════════════════════════════════════════════════════════════════════════════
 *
 *   WHERE    WHAT                           SIZE        HOW ALLOCATED
 *   ───────  ─────────────────────────────  ──────────  ──────────────────────
 *   BSS      s_map  (header only)           24 B        global static
 *   HEAP     s_map.pool  (quadtree nodes)   14.4 KB     quadtree_map_init()
 *            1200 nodes × 12 B/node
 *
 *   BSS      s_scan  (lidar_scan_t)         5.4 KB      static local in task
 *            460 points × 12 B/point + 12 B header
 *            *** declared `static` inside task_lidar_slam so it goes to BSS,
 *                NOT onto the 6 KB task stack.  Without static it would
 *                immediately overflow the stack. ***
 *
 *   HEAP     task stacks
 *            task_lidar_slam                6 KB        xTaskCreatePinnedToCore
 *            task_odom                      3 KB        xTaskCreate
 *            task_perf_mon                  3 KB        xTaskCreate
 *            _dash_task  (wifi_dash)        4 KB        xTaskCreate (inside init)
 *
 *   BSS      wifi_dashboard.c internal      ~12 KB      global statics in that TU
 *
 *   TOTAL HEAP:  ~14 KB pool + ~16 KB task stacks  ≈ 30 KB
 *   TOTAL BSS:   ~17 KB
 *   ESP32-S3 has ~320 KB free DRAM after IDF + WiFi → comfortable margin.
 *
 * ════════════════════════════════════════════════════════════════════════════
 * CPU PRIORITY NOTE
 * ════════════════════════════════════════════════════════════════════════════
 *   IDF WiFi driver tasks run at priority 23 — they will always preempt our
 *   tasks so the radio stays alive.
 *   IDF LwIP/TCP stack runs at priority 18 — also above us.
 *   httpd server runs at priority 5 — BELOW our tasks.
 *   → The 100 ms vTaskDelay after each scan is mandatory: it gives httpd
 *     time to flush TCP ACKs so the WebSocket does not stall.
 */

/* ── Wi-Fi credentials — fill in before flashing ───────────────────────── */
#define WIFI_SSID      "SPOT-iot"
#define WIFI_PASSWORD  "RacailleSalutaireMigration8052"

#include "../hardware_pins.h"
#include "src/lidar_driver.h"
#include "src/lidar_to_map.h"
#include "src/quadtree_map.h"
#include "src/scan_matcher.h"
#include "src/wifi_dashboard.h"
#include "src/uart_bridge.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include <math.h>
#include <stdbool.h>
#include <stdio.h>


/* ════════════════════════════════════════════════════════════════════════════
 * Shared state
 * ════════════════════════════════════════════════════════════════════════════ */

/* Map header in BSS; node pool allocated on heap by quadtree_map_init(). */
static quadtree_map_t s_map;

/* Robot pose in the fixed world frame (mm, rad).
 * Written by task_odom, read by task_lidar_slam — always under s_pose_mutex. */
static pose_t s_pose;

/* Task handles — used by task_perf_mon to read stack watermarks. */
static TaskHandle_t s_h_lidar = NULL;
static TaskHandle_t s_h_odom  = NULL;
static TaskHandle_t s_h_perf  = NULL;

/* Protects s_pose between task_odom (writer) and task_lidar_slam (reader). */
static SemaphoreHandle_t s_pose_mutex = NULL;

/* ── Diagnostic counters ─────────────────────────────────────────────────── *
 * Written by task_lidar_slam, read by task_perf_mon.  No mutex needed —     *
 * perf_mon is allowed to read a value that is one update behind.            */
static volatile uint32_t s_scans_ok   = 0;
static volatile uint32_t s_scans_bad  = 0;

static volatile uint32_t s_read_us_tot = 0;
static volatile uint32_t s_read_us_max = 0;

static volatile uint32_t s_l2m_calls  = 0;
static volatile uint32_t s_l2m_us_tot = 0;
static volatile uint32_t s_l2m_us_max = 0;

/* Scan-matcher diagnostics */
static volatile uint32_t s_sm_calls   = 0;   /* total scan_match() invocations */
static volatile uint32_t s_sm_valid   = 0;   /* corrections accepted */
static volatile uint32_t s_sm_us_tot  = 0;   /* cumulative time in scan_match() */
static volatile uint32_t s_sm_us_max  = 0;   /* worst-case scan_match() time */


/* ════════════════════════════════════════════════════════════════════════════
 * task_lidar_slam  —  Core 0, priority 7
 * ════════════════════════════════════════════════════════════════════════════ */
static void task_lidar_slam(void *arg)
{
    (void)arg;

    /* *** 5.4 KB scan buffer — MUST be static ***
     * If this were a plain local variable it would sit on the 6 KB task stack
     * and leave only ~600 B for the rest of the function — instant overflow.
     * `static` moves it to BSS, completely outside the task stack. */
    static lidar_scan_t scan;

    int64_t last_scan_bcast_us = 0;

    for (;;) {

        /* ── 1. Acquire one full 360° scan ──────────────────────────────── *
         * Blocks until the LiDAR motor completes one rotation (~100 ms).    *
         * If the UART ring buffer already holds a complete buffered scan    *
         * it returns in ~0 ms.                                              */
        int64_t t_read  = esp_timer_get_time();
        bool    read_ok = lidar_driver_read_scan(&scan);
        uint32_t read_us = (uint32_t)(esp_timer_get_time() - t_read);

        if (!read_ok || scan.count < 250) {
            s_scans_bad++;
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        /* Reject scan boundary collisions: the UART ring buffer can hold
         * multiple rotation's worth of data, and the seed mechanism sometimes
         * hits two start-bits in rapid succession, producing a scan with only
         * a handful of points and a sub-millisecond period.
         * period == 0 means the entire scan drained from the buffer instantly
         * (valid full scan); period > 0 but < 1 ms means the end start-bit
         * was also already in the buffer — a fragmented scan, not a real rotation. */
        if (scan.rotation_period_us > 0 && scan.rotation_period_us < 1000u) {
            s_scans_bad++;
            continue;
        }

        s_scans_ok++;
        s_read_us_tot += read_us;
        if (read_us > s_read_us_max) s_read_us_max = read_us;

        /* ── 2. Snapshot raw odometry pose (after 100 ms scan window) ────── *
         * task_odom updates s_pose at 100 Hz. Snapshot here for the most   *
         * current estimate before scan matching and map integration.        */
        pose_t raw_pose;
        xSemaphoreTake(s_pose_mutex, portMAX_DELAY);
        raw_pose = s_pose;
        xSemaphoreGive(s_pose_mutex);

        /* ── 3. Scan matching — correct raw odometry with map correlation ── *
         * Runs BEFORE lidar_to_map so the corrected pose drives map writes. *
         * On the first few scans the map is empty → valid=false → raw_pose  *
         * is used unchanged.  Once enough walls are mapped the matcher kicks *
         * in and applies dx/dy/dtheta corrections up to ±25 mm / ±5°.      */
        pose_t matched_pose;
        scan_match_result_t sm;
        bool sm_ok = scan_match(&s_map, &scan, &raw_pose, &matched_pose, &sm);

        s_sm_calls++;
        s_sm_us_tot += sm.elapsed_us;
        if (sm.elapsed_us > s_sm_us_max) s_sm_us_max = sm.elapsed_us;

        if (sm_ok) {
            s_sm_valid++;
            /* Apply the delta correction back to s_pose so subsequent odom
             * integration starts from the corrected position. */
            xSemaphoreTake(s_pose_mutex, portMAX_DELAY);
            s_pose.x     += sm.dx_mm;
            s_pose.y     += sm.dy_mm;
            s_pose.theta += sm.dtheta_rad;
            xSemaphoreGive(s_pose_mutex);

            printf("[SM] corr  odom=(%.0f,%.0f,%.1f°)  matched=(%.0f,%.0f,%.1f°)"
                   "  delta=(dx=%+.0f dy=%+.0f dθ=%+.1f°)"
                   "  score=%d(+%d)/%d(%.0f%%)  t=%lu us\n",
                   (double)raw_pose.x, (double)raw_pose.y,
                   (double)(raw_pose.theta * 180.0f / (float)M_PI),
                   (double)matched_pose.x, (double)matched_pose.y,
                   (double)(matched_pose.theta * 180.0f / (float)M_PI),
                   (double)sm.dx_mm, (double)sm.dy_mm,
                   (double)(sm.dtheta_rad * 180.0f / (float)M_PI),
                   sm.score, sm.score - sm.baseline, sm.samples,
                   (double)(100.0f * sm.score / sm.samples),
                   (unsigned long)sm.elapsed_us);
        } else {
            matched_pose = raw_pose;
            if (sm.samples > 0)
                printf("[SM] skip  score=%d(+%d)/%d(%.0f%%)  t=%lu us\n",
                       sm.score, sm.score - sm.baseline, sm.samples,
                       (double)(100.0f * sm.score / sm.samples),
                       (unsigned long)sm.elapsed_us);
        }

        /* ── 4. Ray-march scan into quadtree map ────────────────────────── *
         * Uses the scan-matched pose for map integration.                    */
        map_dirty_rect_t dirty;
        int64_t t0 = esp_timer_get_time();
        lidar_to_map(&s_map, &scan, &matched_pose,
                     LIDAR_PROCESS_RANGE_MM,
                     80.0f,
                     &dirty);
        uint32_t elapsed = (uint32_t)(esp_timer_get_time() - t0);

        s_l2m_calls++;
        s_l2m_us_tot += elapsed;
        if (elapsed > s_l2m_us_max) s_l2m_us_max = elapsed;

        /* Proactive compaction at 85% pool usage — fires before the pool
         * freezes.  qt_compact() snapshots all confident walls (value ≥ 10),
         * wipes the pool in-place, then re-inserts the saved cells so the
         * scan matcher and dashboard retain full wall knowledge.
         * Headroom: re-inserting N cells uses ≤ N×7 nodes, so triggering
         * at 85% (3400/4000) leaves ≥ 600 nodes of margin. */
        if (s_map.count > (uint16_t)(QT_POOL_SIZE * 85 / 100)) {
            uint16_t before = s_map.count;
            qt_compact(&s_map, 10);
            printf("[MAP] compact  before=%u  after=%u  freed=%u nodes\n",
                   (unsigned)before, (unsigned)s_map.count,
                   (unsigned)(before - s_map.count));
            /* Force dashboard to resample the full map after compaction */
            map_dirty_rect_t full_dirty = {
                .valid = true,
                .x_min = s_map.x_min, .y_min = s_map.y_min,
                .x_max = s_map.x_max, .y_max = s_map.y_max,
            };
            wifi_dashboard_mark_dirty(&full_dirty);
        }

        /* ── 5. Notify dashboard ─────────────────────────────────────────── *
         * Send raw odometry pose (orange ghost) then corrected pose (blue   *
         * car) so the dashboard can show both and the correction vector.    */
        wifi_dashboard_mark_dirty(&dirty);
        wifi_dashboard_update(&s_map, &matched_pose);

        int64_t now_us = esp_timer_get_time();
        if (now_us - last_scan_bcast_us >= 500000LL) {
            wifi_dashboard_broadcast_scan(&scan, &matched_pose);
            last_scan_bcast_us = now_us;
        }

        wifi_dashboard_broadcast_raw_pose(&raw_pose);
        wifi_dashboard_broadcast_state(&matched_pose, 0.0f, 0.0f, false, 0);

        /* ── 5. Yield — gives httpd CPU to flush TCP ACKs ────────────────── */
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}


/* ════════════════════════════════════════════════════════════════════════════
 * task_perf_mon  —  Core 0, priority 1
 * ════════════════════════════════════════════════════════════════════════════ */
static void task_perf_mon(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(5000));

    uint32_t prev_ok = 0, prev_bad = 0, prev_l2m = 0, prev_us = 0;
    uint32_t prev_read_us = 0;
    uint32_t prev_sm = 0, prev_sm_us = 0, prev_sm_valid = 0;

    for (;;) {
        uint32_t free_now = (uint32_t)esp_get_free_heap_size();
        uint32_t free_min = (uint32_t)esp_get_minimum_free_heap_size();
        uint32_t largest  = (uint32_t)heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT);
        uint32_t frag_pct = free_now ? (100u - largest * 100u / free_now) : 0u;

        printf("[PERF] heap    free=%lu B  min_ever=%lu B"
               "  largest_blk=%lu B  frag=%lu%%\n",
               (unsigned long)free_now,  (unsigned long)free_min,
               (unsigned long)largest,   (unsigned long)frag_pct);

        printf("[PERF] stacks  lidar=%lu B  odom=%lu B  perf=%lu B\n",
               (unsigned long)(uxTaskGetStackHighWaterMark(s_h_lidar) * sizeof(StackType_t)),
               (unsigned long)(uxTaskGetStackHighWaterMark(s_h_odom)  * sizeof(StackType_t)),
               (unsigned long)(uxTaskGetStackHighWaterMark(s_h_perf)  * sizeof(StackType_t)));

        uint32_t now_ok  = s_scans_ok,  now_bad = s_scans_bad;
        uint32_t now_l2m = s_l2m_calls, now_us  = s_l2m_us_tot;

        uint32_t d_ok  = now_ok  - prev_ok;
        uint32_t d_bad = now_bad - prev_bad;
        uint32_t d_l2m = now_l2m - prev_l2m;
        uint32_t d_us  = now_us  - prev_us;
        uint32_t avg   = d_l2m ? d_us / d_l2m : 0u;
        uint32_t total = d_ok + d_bad;

        printf("[PERF] lidar   scans_ok=%lu  scans_bad=%lu  drop=%.0f%%  rate=%.1f/s\n",
               (unsigned long)d_ok,  (unsigned long)d_bad,
               total ? (double)d_bad * 100.0 / total : 0.0,
               (double)d_ok / 10.0);

        uint32_t now_read_us = s_read_us_tot;
        uint32_t d_read_us   = now_read_us - prev_read_us;
        uint32_t avg_read    = d_ok ? d_read_us / d_ok : 0u;

        printf("[PERF] stage  scan_read: avg=%lu us  worst_ever=%lu us\n",
               (unsigned long)avg_read,  (unsigned long)s_read_us_max);
        printf("[PERF] stage  l2m:       avg=%lu us  worst_ever=%lu us  calls=%lu\n",
               (unsigned long)avg, (unsigned long)s_l2m_us_max, (unsigned long)d_l2m);

        uint32_t now_sm       = s_sm_calls;
        uint32_t now_sm_us    = s_sm_us_tot;
        uint32_t now_sm_valid = s_sm_valid;
        uint32_t d_sm         = now_sm       - prev_sm;
        uint32_t d_sm_us      = now_sm_us    - prev_sm_us;
        uint32_t d_sm_valid   = now_sm_valid - prev_sm_valid;
        uint32_t avg_sm       = d_sm ? d_sm_us / d_sm : 0u;
        printf("[PERF] stage  scan_match: avg=%lu us  worst_ever=%lu us"
               "  valid=%lu/%lu(%.0f%%)\n",
               (unsigned long)avg_sm,   (unsigned long)s_sm_us_max,
               (unsigned long)d_sm_valid, (unsigned long)d_sm,
               d_sm ? (double)d_sm_valid * 100.0 / d_sm : 0.0);

        prev_ok      = now_ok;  prev_bad = now_bad;
        prev_l2m     = now_l2m; prev_us  = now_us;
        prev_read_us = now_read_us;
        prev_sm      = now_sm;  prev_sm_us = now_sm_us;
        prev_sm_valid = now_sm_valid;

#ifdef CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS
        static char rt_buf[1024];
        vTaskGetRunTimeStats(rt_buf);
        printf("[PERF] cpu (task / ticks / %%cpu):\n%s\n", rt_buf);
#endif

        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}


/* ════════════════════════════════════════════════════════════════════════════
 * task_odom  —  Core 0, priority 6
 *
 * Receives odom_t packets from the Wemos via UART bridge and integrates them
 * into s_pose (x, y, theta) using midpoint Runge-Kutta integration:
 *
 *   dtheta    = yaw_rate_imu × dt_ms / 1000
 *   theta_mid = theta + 0.5 × dtheta          (heading at midpoint of step)
 *   x        += linear_disp_mm × cos(theta_mid)
 *   y        += linear_disp_mm × sin(theta_mid)
 *   theta     = theta + dtheta                 (wrapped to [-π, π])
 *
 * What this means on the dashboard:
 *   Turn the car in place → theta changes → the car MARKER rotates; map is fixed.
 *   Push the car forward  → x/y changes  → the car MARKER moves; map is fixed.
 * ════════════════════════════════════════════════════════════════════════════ */
static float _wrap_angle(float a)
{
    a = fmodf(a, 2.0f * (float)M_PI);
    if (a >  (float)M_PI) a -= 2.0f * (float)M_PI;
    if (a < -(float)M_PI) a += 2.0f * (float)M_PI;
    return a;
}

static void task_odom(void *arg)
{
    (void)arg;

    uint32_t last_seq    = UINT32_MAX;
    uint32_t total_pkts  = 0;
    uint32_t total_drops = 0;
    uint32_t cycle       = 0;
    int64_t  last_log_us = 0;

    for (;;) {

        /* ── Drain all queued odom packets this tick ────────────────────── */
        odom_t odom;
        while (uart_bridge_recv_odom(&odom)) {

            /* Sequence gap detection */
            if (last_seq != UINT32_MAX) {
                uint8_t expected = (uint8_t)((last_seq + 1u) & 0xFFu);
                if ((uint8_t)(odom.seq & 0xFFu) != expected)
                    total_drops++;
            }
            last_seq = odom.seq;
            total_pkts++;

            /* Reject obviously corrupt packets */
            if (!isfinite(odom.linear_disp_mm) || !isfinite(odom.yaw_rate_imu) ||
                odom.dt_ms <= 0.0f || odom.dt_ms > 200.0f)
                continue;

            float ds     = odom.linear_disp_mm;
            float dtheta = odom.yaw_rate_imu * (odom.dt_ms / 1000.0f);

            /* Integrate under mutex */
            xSemaphoreTake(s_pose_mutex, portMAX_DELAY);
            float theta_mid  = _wrap_angle(s_pose.theta + 0.5f * dtheta);
            s_pose.x        += ds * cosf(theta_mid);
            s_pose.y        += ds * sinf(theta_mid);
            s_pose.theta     = _wrap_angle(s_pose.theta + dtheta);
            float log_x      = s_pose.x;
            float log_y      = s_pose.y;
            float log_theta  = s_pose.theta;
            xSemaphoreGive(s_pose_mutex);

            printf("[ODOM-PKT] seq=%3u  enc=% 7.2f mm  yaw_rate=% 6.3f rad/s"
                   "  dt=%4.1f ms  →  ds=% 6.2f mm  dθ=% 6.3f rad"
                   "  pose=(%.0f, %.0f, %.1f°)\n",
                   (unsigned)(odom.seq & 0xFFu),
                   (double)odom.linear_disp_mm,
                   (double)odom.yaw_rate_imu,
                   (double)odom.dt_ms,
                   (double)ds,
                   (double)dtheta,
                   (double)log_x, (double)log_y,
                   (double)(log_theta * 180.0f / (float)M_PI));
        }

        /* ── 1 Hz pose summary ──────────────────────────────────────────── */
        int64_t now_us = esp_timer_get_time();
        if (now_us - last_log_us >= 1000000LL) {
            last_log_us = now_us;

            xSemaphoreTake(s_pose_mutex, portMAX_DELAY);
            float px = s_pose.x, py = s_pose.y, pth = s_pose.theta;
            xSemaphoreGive(s_pose_mutex);

            printf("[ODOM] x=%.0f mm  y=%.0f mm  theta=%.1f°"
                   "  pkts=%lu  drops=%lu  stack=%lu B\n",
                   (double)px, (double)py,
                   (double)(pth * 180.0f / (float)M_PI),
                   (unsigned long)total_pkts,
                   (unsigned long)total_drops,
                   (unsigned long)(uxTaskGetStackHighWaterMark(NULL) * sizeof(StackType_t)));
        }

        /* ── Every 30 s: heap health ────────────────────────────────────── */
        cycle++;
        if (cycle % 300u == 0u) {
            uint32_t heap_free = (uint32_t)esp_get_free_heap_size();
            uint32_t largest   = (uint32_t)heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT);
            uint32_t frag_pct  = heap_free ? (100u - largest * 100u / heap_free) : 0u;
            printf("[ODOM-MEM] heap=%lu B  largest=%lu B  frag=%lu%%\n",
                   (unsigned long)heap_free, (unsigned long)largest,
                   (unsigned long)frag_pct);
#ifdef CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS
            static char rt_buf[512];
            vTaskGetRunTimeStats(rt_buf);
            printf("[ODOM-CPU]\n%s\n", rt_buf);
#endif
        }

        vTaskDelay(pdMS_TO_TICKS(10));   /* 100 Hz drain rate */
    }
}


/* ════════════════════════════════════════════════════════════════════════════
 * task_path_runner  —  any core, priority 3
 *
 * Defines a hardcoded L-shaped path relative to the robot's pose at startup:
 *   WP0  current position          (start)
 *   WP1  +2000 mm forward          (end of straight)
 *   WP2  +2000 mm left from WP1   (final goal)
 *
 * Sends the path once to the Wemos (which executes it via task_pure_pursuit)
 * and re-broadcasts it to the dashboard every 2 s so any late WebSocket
 * connection sees the reference path immediately.
 *
 * Terminates itself after the Wemos signals path_done.
 * ════════════════════════════════════════════════════════════════════════════ */
static void task_path_runner(void *arg)
{
    (void)arg;

    /* Wait for WiFi to associate and the first few scans to arrive so the
     * dashboard has a map to render before the path overlay appears. */
    vTaskDelay(pdMS_TO_TICKS(5000));

    /* Snapshot the current scan-matched pose as the path origin. */
    xSemaphoreTake(s_pose_mutex, portMAX_DELAY);
    float sx  = s_pose.x;
    float sy  = s_pose.y;
    float sth = s_pose.theta;
    xSemaphoreGive(s_pose_mutex);

    const float cos_th = cosf(sth);
    const float sin_th = sinf(sth);

    /* L-path in world mm:
     *   forward direction  = (cos_th,  sin_th)
     *   left    direction  = (-sin_th, cos_th)   (θ + 90°) */
    path_frame_t path;
    path.length     = 3;
    path.reserved   = 0;
    path.waypoints[0] = (waypoint_t){
        .x = sx, .y = sy,
        .theta = sth, .v_target = 200.0f
    };
    path.waypoints[1] = (waypoint_t){
        .x = sx + 2000.0f * cos_th,
        .y = sy + 2000.0f * sin_th,
        .theta = sth, .v_target = 200.0f
    };
    path.waypoints[2] = (waypoint_t){
        .x = sx + 2000.0f * cos_th - 2000.0f * sin_th,
        .y = sy + 2000.0f * sin_th + 2000.0f * cos_th,
        .theta = sth + (float)M_PI / 2.0f, .v_target = 0.0f
    };

    uart_bridge_send_path(&path);
    printf("[PATH] L-path sent:"
           "  (%.0f,%.0f) → (%.0f,%.0f) → (%.0f,%.0f)"
           "  heading=%.1f°\n",
           (double)path.waypoints[0].x, (double)path.waypoints[0].y,
           (double)path.waypoints[1].x, (double)path.waypoints[1].y,
           (double)path.waypoints[2].x, (double)path.waypoints[2].y,
           (double)(sth * 180.0f / (float)M_PI));

    /* Re-send reference path to dashboard every 2 s until the robot is done.
     * Handles late WebSocket connections: the browser always sees the path
     * overlay regardless of when it opened the dashboard page. */
    for (;;) {
        wifi_dashboard_broadcast_path(&path);

        if (uart_bridge_recv_path_done()) {
            printf("[PATH] complete — robot reached goal\n");
            /* Clear the path overlay on the dashboard. */
            path_frame_t empty = { .length = 0, .reserved = 0 };
            wifi_dashboard_broadcast_path(&empty);
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    vTaskDelete(NULL);
}


/* ════════════════════════════════════════════════════════════════════════════
 * app_main
 * ════════════════════════════════════════════════════════════════════════════ */
void app_main(void)
{
    lidar_driver_init();

    /* 10 m × 10 m arena.  Allocates the node pool on the heap.
     * Actual leaf cell = 10000/64 ≈ 156 mm (QT_MAX_DEPTH=7); step_mm is
     * passed for API compatibility but ignored by the compat wrapper. */
    quadtree_map_init(&s_map, 10000.0f, 10000.0f, 156.0f);

    /* Robot starts at map centre facing +X.  task_odom updates this from here. */
    s_pose = (pose_t){ .x = 5000.0f, .y = 5000.0f, .theta = 0.0f };

    s_pose_mutex = xSemaphoreCreateMutex();
    configASSERT(s_pose_mutex);

    /* WiFi + httpd + _dash_task.  Must be called after quadtree_map_init. */
    wifi_dashboard_init(WIFI_SSID, WIFI_PASSWORD);

    /* UART bridge to Wemos: receives encoder + IMU odometry packets. */
    uart_bridge_init();

    xTaskCreatePinnedToCore(task_lidar_slam, "lscan",    6144, NULL, 7, &s_h_lidar, 0);
    xTaskCreate(             task_odom,      "odom",     3072, NULL, 6, &s_h_odom);
    xTaskCreate(             task_perf_mon,  "perf_mon", 3072, NULL, 1, &s_h_perf);
    xTaskCreate(             task_path_runner,"path_run", 3072, NULL, 3, NULL);
}
