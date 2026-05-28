/**
 * wifi_dashboard.h
 * Module: Wi-Fi HTTP + WebSocket dashboard — live map, pose, and frontier.
 * Board: ESP32-S3
 *
 * Two entry points:
 *   wifi_dashboard_init()          — connect to Wi-Fi and start the server.
 *   wifi_dashboard_update()        — snapshot the map for the HTTP page.
 *   wifi_dashboard_broadcast_state() — push pose + frontier over WebSocket to
 *                                      the live_dashboard.html client.
 *
 * WebSocket endpoint: ws://<esp32-ip>/ws
 * JSON frame (text):  {"x":<mm>,"y":<mm>,"theta":<rad>,"fx":<mm>,"fy":<mm>,"has_frontier":<0|1>,"si":<scan_idx>}
 */

#ifndef WIFI_DASHBOARD_H
#define WIFI_DASHBOARD_H

#include "../../types.h"
#include "quadtree_map.h"
#include "lidar_to_map.h"

typedef enum {
    DASH_CTRL_ASTAR = 0,
    DASH_CTRL_REACTIVE,
    DASH_CTRL_STOPPED,
    DASH_CTRL_STALL_REVERSE,
    DASH_CTRL_SONAR_EMERGENCY,
    DASH_CTRL_RECOVER
} dash_control_layer_t;

/**
 * Connect to Wi-Fi and start the HTTP + WebSocket server.
 * Blocks until connected or until a 10-second timeout.
 * @param ssid     Wi-Fi SSID (null-terminated).
 * @param password Wi-Fi password (null-terminated).
 */
void wifi_dashboard_init(const char *ssid, const char *password);

/**
 * Snapshot map + pose for the HTTP tile endpoint (served on request).
 * Thread-safe via internal mutex.
 * @param map  Current quadtree map.
 * @param pose Current best robot pose.
 */
void wifi_dashboard_update(const quadtree_map_t *map, const pose_t *pose);

/**
 * Broadcast robot state to the live dashboard WebSocket client.
 * Non-blocking — silently drops the frame if no client is connected.
 * Call once per planning cycle after command_gen_compute().
 *
 * @param pose         Current robot pose (x, y in mm, theta in radians).
 * @param frontier_cx  Selected frontier / next-waypoint X in mm (0 if none).
 * @param frontier_cy  Selected frontier / next-waypoint Y in mm (0 if none).
 * @param has_frontier true if a frontier / waypoint target is valid.
 * @param scan_idx     room.log scan index the car has reached (0 in frontier
 *                     mode). Sent as "si" in JSON — browser uses it to advance
 *                     the live map render in playback_dashboard.html.
 */
void wifi_dashboard_broadcast_state(const pose_t *pose,
                                     float frontier_cx, float frontier_cy,
                                     bool has_frontier,
                                     uint16_t scan_idx);

void wifi_dashboard_set_control_layer(dash_control_layer_t layer);

/**
 * Returns true (and clears the flag) if the browser sent a {"cmd":"start"}
 * WebSocket message since the last call.  Poll this from app_main to gate
 * the start of the exploration loop.
 */
bool wifi_dashboard_exploration_requested(void);

/**
 * Returns true (and clears the flag) if the browser sent a {"cmd":"stop"}
 * WebSocket message.  Poll this to implement emergency stop.
 */
bool wifi_dashboard_stop_requested(void);

/**
 * Returns true if the stop flag is set, WITHOUT clearing it.
 * Use this as the imu_gyro_set_stop_check() callback so the flag
 * survives for the main loop's wifi_dashboard_stop_requested() check.
 */
bool wifi_dashboard_stop_peek(void);

/**
 * Broadcast a downsampled LiDAR scan as a binary type-0x02 frame.
 * Call after each scan at up to 10 Hz. Silently drops if no client connected.
 * @param scan  Raw LiDAR scan (up to 460 points; downsampled to ≤180 on send).
 * @param pose  Robot pose at scan time (used by browser for world projection).
 */
void wifi_dashboard_broadcast_scan(const lidar_scan_t *scan, const pose_t *pose);

/**
 * Broadcast the raw (pre-scan-match) odometry pose as a type-0x07 frame.
 * The dashboard renders it as a ghost outline in orange so you can see the
 * correction applied by scan matching vs the raw odometry estimate.
 * Call once per scan cycle, right before wifi_dashboard_broadcast_state().
 * @param raw_pose  Raw odometry pose before scan-match correction.
 */
void wifi_dashboard_broadcast_raw_pose(const pose_t *raw_pose);

/**
 * Send a plain-text log line to the dashboard log box.
 * Silently drops if no client is connected.
 * @param msg Null-terminated string (max ~200 chars).
 */
void wifi_dashboard_log(const char *msg);

/**
 * Returns the number of messages currently waiting in the dash task queue
 * (0–24).  Use this to verify architecture health at runtime.
 * Values consistently above 12 indicate backpressure — reduce caller rate.
 * Safe to call from any task.
 */
uint8_t wifi_dashboard_queue_depth(void);

/**
 * Accumulate a dirty bounding box from one lidar_to_map() call into the
 * pending dirty rect.  Thread-safe (mutex-protected).  Call immediately
 * after lidar_to_map() from scan_task; wifi_dashboard_update() snapshots
 * and clears the accumulator when it queues the next map frame.
 * @param rect  Dirty rect filled by lidar_to_map(); ignored if rect->valid == false.
 */
void wifi_dashboard_mark_dirty(const map_dirty_rect_t *rect);

/**
 * Broadcast the A* planned path to the live dashboard as a type-0x06 frame.
 * Pass the path_frame_t that was just sent to the Wemos so the overlay matches
 * what the car is executing.  Pass NULL or length=0 to clear a stale overlay.
 * Non-blocking — silently drops if no client connected.
 * @param frame  Path frame (may be NULL or have length=0 to clear overlay).
 */
void wifi_dashboard_broadcast_path(const path_frame_t *frame);

/**
 * Broadcast the full quadtree structure as a type-0x05 binary frame.
 * Call at ~1 Hz from the planning loop for live debug visualization.
 * Frame: [type(1)][count(2)][{x_min(2),y_min(2),size(2),depth(1),value(1)}×N]
 * All coordinates in mm (little-endian uint16). value is raw log-odds int8.
 * Capped at 1500 nodes; DFS order ensures shallow nodes are never truncated.
 * @param map  Current quadtree map (read-only; no mutex needed from plan_task).
 */
void wifi_dashboard_broadcast_quadtree(const quadtree_map_t *map);

#endif /* WIFI_DASHBOARD_H */
