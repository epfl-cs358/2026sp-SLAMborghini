/**
 * wifi_dashboard.c
 * Module: Wi-Fi HTTP dashboard — serves live map and pose visualization.
 * Board: ESP32-S3
 * Implementation phase: stub (HTTP server and Wi-Fi stack not yet integrated)
 */

#include "wifi_dashboard.h"

void wifi_dashboard_init(const char *ssid, const char *password)
{
    // TODO: implement
    // 1. Initialize NVS flash (nvs_flash_init).
    // 2. Initialize the TCP/IP stack and Wi-Fi driver (esp_wifi_init).
    // 3. Configure station mode with the provided ssid and password.
    // 4. Start the ESP-IDF HTTP server (esp_http_server).
    // 5. Register URI handlers for "/" (map render) and "/pose" (JSON pose).
    (void)ssid;
    (void)password;
}

void wifi_dashboard_update(const quadtree_map_t *map, const pose_t *pose)
{
    // TODO: implement
    // Copy map and pose into a module-level snapshot buffer (thread-safe with mutex).
    // The HTTP handler will read from the snapshot buffer when serving requests.
    (void)map;
    (void)pose;
}
