/**
 * wifi_dashboard.h
 * Module: Wi-Fi HTTP dashboard — serves live map and pose visualization.
 * Board: ESP32-S3
 * Connects to a Wi-Fi network and starts a lightweight HTTP server that renders
 * the current occupancy map and robot pose as a simple web page.
 */

#ifndef WIFI_DASHBOARD_H
#define WIFI_DASHBOARD_H

#include "../../types.h"
#include "quadtree_map.h"

/**
 * Connect to the specified Wi-Fi network and start the HTTP server.
 * Blocks until a Wi-Fi connection is established or a timeout occurs.
 * @param ssid     Wi-Fi network name (null-terminated string).
 * @param password Wi-Fi password (null-terminated string).
 */
void wifi_dashboard_init(const char *ssid, const char *password);

/**
 * Update the dashboard's internal snapshot of the map and pose.
 * Should be called once per SLAM cycle. The HTTP server will serve the latest snapshot.
 * @param map  Pointer to the current quadtree map.
 * @param pose Pointer to the current best robot pose.
 */
void wifi_dashboard_update(const quadtree_map_t *map, const pose_t *pose);

#endif /* WIFI_DASHBOARD_H */
