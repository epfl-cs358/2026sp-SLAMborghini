/**
 * Board: ESP32-S3 (SLAM Brain)
 *
 * Testing before full first implementation:
 *   1. Run frontier detection on the current map.
 *   2. If a frontier is found → send its position as the control target.
 *   3. If no frontier (map still a stub → all cells return 0 = free,
 *      no unknown neighbours) → fall back to a hardcoded 200 mm straight
 *      forward command so the car still moves and the UART link is verified.
 *
 * Once quadtree_map, rbpf, and hybrid_astar are implemented,we should replace the
 * commented-out full loop below and remove the debug fallback.
 */

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

/* ── Debug fallback distance ─────────────────────────────────────────────── */
#define DEBUG_FORWARD_MM   200.0f   /* straight forward distance (mm)        */
#define DEBUG_SPEED_MM_S   100.0f   /* target speed sent in control frame     */

/* ── Event-driven replanning thresholds ─────────────────────────────────── */
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

    /* Robot starts at the origin, heading = 0 (facing +X).
     * RBPF will provide this once implemented; use (0,0,0) for debug. */
    pose_t pose = {0};
    pose.x     = 0.0f;
    pose.y     = 0.0f;
    pose.theta = 0.0f;

    /* ── Frontier detection ───────────────────────────────────────────────
     * With the quadtree stub (returns 0 = free for all cells), there are
     * no unknown cells, so no frontiers will be detected. The fallback
     * below handles this case. Once the quadtree is populated by the
     * lidar→classifier→map_updater pipeline, this will return real frontiers.
     * ──────────────────────────────────────────────────────────────────── */
    frontier_list_t frontiers = frontier_detector_detect(&map, &pose);

    control_frame_t cmd = {0};

    if (frontiers.count > 0) {
        /* ── Frontier found: send it as the navigation target ────────────
         * Full implementation: pass to hybrid_astar_plan() then command_gen.
         * Debug simplification: send frontier position directly at fixed speed.
         * The Wemos will drive toward it as a straight-line approximation.
         * ──────────────────────────────────────────────────────────────── */
        frontier_t best = frontier_detector_best(&frontiers, &pose);
        cmd.tx        = best.cx;
        cmd.ty        = best.cy;
        cmd.t_heading = pose.theta;
        cmd.t_speed   = DEBUG_SPEED_MM_S;
    } else {
        /* ── Debug fallback: no frontier yet (map is stub) ───────────────
         * Send a command to drive straight forward 200 mm.
         * tx=0, ty=200 means "target is 200 mm ahead along current heading".
         * The Wemos uses sqrt(tx²+ty²) as the distance to travel.
         * ──────────────────────────────────────────────────────────────── */
        cmd.tx        = 0.0f;
        cmd.ty        = DEBUG_FORWARD_MM;
        cmd.t_heading = 0.0f;
        cmd.t_speed   = DEBUG_SPEED_MM_S;
    }

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
     * frontier_detector_detect() now returns at most 1 frontier (early-exit
     * BFS), so frontier_detector_best() is no longer needed — items[0] is
     * already the nearest valid cluster by BFS order.
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
     *             current_target.x = frontiers.items[0].cx;
     *             current_target.y = frontiers.items[0].cy;
     *             has_target   = true;
     *             last_plan_ms = now_ms;
     *         } else {
     *             has_target = false;   // fully explored or map not ready
     *         }
     *     }
     *
     *     // ── Navigate toward current target ─────────────────────────────
     *     if (has_target) {
     *         path_t path = hybrid_astar_plan(&map, &pose, &current_target);
     *         path_refiner_smooth(&path);
     *         path_refiner_shortcut(&path, &map);
     *
     *         if (hybrid_astar_is_valid(&path)) {
     *             control_frame_t cmd = command_gen_compute(&pose, &path.waypoints[0]);
     *             uart_bridge_send_control(&cmd);
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
