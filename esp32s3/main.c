/**
 * Board: ESP32-S3 (SLAM Brain)
 *
 * ── Debug flag ───────────────────────────────────────────────────────────────
 * USE_FRONTIER_TARGET  0  →  Hardware smoke test: always send a hardcoded 200 mm
 *                            straight-forward command regardless of what frontier
 *                            detection returns. Proves the UART + motor chain
 *                            works before path following is wired up.
 *
 * USE_FRONTIER_TARGET  1  →  Real mode: run frontier detection, pass the best
 *                            target through command_gen, send the computed frame.
 *                            Switch to this once UART + motors are verified.
 * ─────────────────────────────────────────────────────────────────────────── */
#define USE_FRONTIER_TARGET  0

#include "src/lidar_driver.h"
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

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

/* ── Debug fallback ──────────────────────────────────────────────────────── */
#define DEBUG_FORWARD_MM   200.0f   /* hardcoded straight-forward distance    */
#define DEBUG_SPEED_MM_S   100.0f   /* speed sent with the fallback command   */

/* ── Event-driven replanning thresholds (used in full loop below) ─────────── */
#define TARGET_REACHED_MM   150.0f  /* declare target reached within this radius */
#define REPLAN_TIMEOUT_MS  5000     /* force replan if no progress for 5 s       */

void app_main(void)
{
    /* ── Hardware initialisation ─────────────────────────────────────────── */
    uart_bridge_init();

    /* Give the Wemos time to boot and start listening before we send */
    vTaskDelay(pdMS_TO_TICKS(1000));

    /* ── Map and pose initialisation ─────────────────────────────────────── */
    quadtree_map_t map;
    quadtree_map_init(&map, 10000.0f, 10000.0f, 50.0f);

    pose_t pose = {0};
    pose.x     = 0.0f;
    pose.y     = 0.0f;
    pose.theta = 0.0f;

    /* ── Frontier detection ──────────────────────────────────────────────── */
    frontier_list_t frontiers = frontier_detector_detect(&map, &pose);

    /* ── Build and send command ──────────────────────────────────────────── */
    control_frame_t cmd = {0};

#if USE_FRONTIER_TARGET

    /* ── Real mode: use command_gen output ─────────────────────────────────
     * frontier_list_t.items[0] is already the nearest valid cluster (BFS
     * order). Build a waypoint from it and let command_gen compute heading
     * and speed-scaled frame. Fall back to 200 mm forward if no frontier. */
    if (frontiers.count > 0) {
        frontier_t best = frontier_detector_best(&frontiers, &pose);
        waypoint_t wp   = {0};
        wp.x            = best.cx;
        wp.y            = best.cy;
        cmd             = command_gen_compute(&pose, &wp);
    } else {
        cmd.tx        = 0.0f;
        cmd.ty        = DEBUG_FORWARD_MM;
        cmd.t_heading = pose.theta;
        cmd.t_speed   = DEBUG_SPEED_MM_S;
    }

#else  /* USE_FRONTIER_TARGET == 0 */

    /* ── Debug mode: UART + motor smoke test ───────────────────────────────
     * Frontier detection ran above (crash-checks the module on real hardware)
     * but its result is ignored. Send a fixed 200 mm forward command so the
     * Wemos drives straight and we can confirm the full communication chain. */
    (void)frontiers;
    cmd.tx        = 0.0f;
    cmd.ty        = DEBUG_FORWARD_MM;
    cmd.t_heading = pose.theta;
    cmd.t_speed   = DEBUG_SPEED_MM_S;

#endif /* USE_FRONTIER_TARGET */

    uart_bridge_send_control(&cmd);

    /* ── Main loop (keep alive; full SLAM loop goes here later) ─────────── */
    /*
     * Full loop (uncomment once all modules are implemented):
     *
     * Replanning is event-driven — frontier_detector_detect() is only called
     * when one of three conditions fires, not every scan cycle:
     *   1. No target yet (startup).
     *   2. Robot reached the current target (within TARGET_REACHED_MM).
     *   3. Current target cell became occupied (obstacle appeared there).
     *   4. REPLAN_TIMEOUT_MS elapsed without reaching the target (stuck).
     *
     * frontier_detector_detect() returns at most 1 frontier (early-exit BFS),
     * so frontier_detector_best() is not needed — items[0] is already the
     * nearest valid cluster by BFS order.
     *
     * pose_t    current_target = {0};
     * bool      has_target     = false;
     * uint32_t  last_plan_ms   = 0;
     *
     * while (1) {
     *     lidar_scan_t scan;
     *     lidar_driver_read_scan(&scan);
     *
     *     point2f_t cart_pts[460]; uint16_t cart_count = 0;
     *     polar_to_cart_convert(&scan, cart_pts, &cart_count);
     *
     *     classified_point_t cls_pts[460]; uint16_t cls_count = 0;
     *     obstacle_classifier_classify(cart_pts, cart_count, cls_pts, &cls_count);
     *
     *     rbpf_update(&rbpf, &scan);
     *     pose = rbpf_get_best_pose(&rbpf);
     *     map_updater_update(&map, cls_pts, cls_count, &pose);
     *
     *     // ── Event-driven replan check ──────────────────────────────────
     *     uint32_t now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
     *     bool need_replan = !has_target;
     *
     *     if (has_target) {
     *         float dx   = pose.x - current_target.x;
     *         float dy   = pose.y - current_target.y;
     *         float dist = sqrtf(dx * dx + dy * dy);
     *
     *         if (dist < TARGET_REACHED_MM)                              need_replan = true;
     *         if (quadtree_map_query(&map, current_target.x,
     *                                current_target.y) > 50)             need_replan = true;
     *         if (now_ms - last_plan_ms > REPLAN_TIMEOUT_MS)             need_replan = true;
     *     }
     *
     *     if (need_replan) {
     *         frontier_list_t frontiers = frontier_detector_detect(&map, &pose);
     *         if (frontiers.count > 0) {
     *             waypoint_t wp = {0};
     *             wp.x = frontiers.items[0].cx;
     *             wp.y = frontiers.items[0].cy;
     *             control_frame_t cmd = command_gen_compute(&pose, &wp);
     *             uart_bridge_send_control(&cmd);
     *             current_target.x = wp.x;
     *             current_target.y = wp.y;
     *             has_target   = true;
     *             last_plan_ms = now_ms;
     *         } else {
     *             has_target = false;   // fully explored or map not ready
     *         }
     *     }
     *
     *     odom_t odom;
     *     if (uart_bridge_recv_odom(&odom)) {
     *         rbpf_predict(&rbpf, &odom);
     *     }
     *
     *     wifi_dashboard_update(&map, &pose);
     * }
     */
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
