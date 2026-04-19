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

#endif /* WIFI_DASHBOARD_H */
