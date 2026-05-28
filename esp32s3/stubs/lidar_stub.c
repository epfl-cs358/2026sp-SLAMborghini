// STUB — simulates RPLiDAR C1 for offline testing

/**
 * lidar_stub.c
 * Module: LiDAR stub — simulates RPLiDAR C1 output for offline testing.
 * Board: ESP32-S3 (PC stub, no hardware required)
 * Used by test_scan_matcher and test_map_updater.
 */

#include "lidar_stub.h"
#include <math.h>
#include <stdint.h>

/* Room dimensions: 4000 mm wide, 3000 mm tall, robot at centre (2000, 1500). */
#define ROOM_W_MM  4000.0f
#define ROOM_H_MM  3000.0f
#define ROBOT_X_MM 2000.0f
#define ROBOT_Y_MM 1500.0f
#define NUM_POINTS 460

/*
 * Scenario: robot sitting at the centre of a 4 m x 3 m rectangular room.
 * The scan models rays bouncing off the four walls.
 * Used by: test_scan_matcher, test_map_updater
 */
lidar_scan_t lidar_stub_room_scan(void)
{
    lidar_scan_t scan;
    scan.count = NUM_POINTS;

    for (int i = 0; i < NUM_POINTS; i++) {
        float theta_deg = (360.0f / NUM_POINTS) * i;
        float theta_rad = theta_deg * (3.14159265f / 180.0f);

        float cos_t = cosf(theta_rad);
        float sin_t = sinf(theta_rad);

        /* Find the closest wall intersection along this ray */
        float r = 1e9f;

        /* Right wall: x = ROOM_W_MM/2 relative to robot */
        if (cos_t > 1e-6f) {
            float t = (ROOM_W_MM / 2.0f) / cos_t;
            if (t < r) r = t;
        }
        /* Left wall: x = -ROOM_W_MM/2 relative to robot */
        if (cos_t < -1e-6f) {
            float t = (-ROOM_W_MM / 2.0f) / cos_t;
            if (t < r) r = t;
        }
        /* Top wall: y = ROOM_H_MM/2 relative to robot */
        if (sin_t > 1e-6f) {
            float t = (ROOM_H_MM / 2.0f) / sin_t;
            if (t < r) r = t;
        }
        /* Bottom wall: y = -ROOM_H_MM/2 relative to robot */
        if (sin_t < -1e-6f) {
            float t = (-ROOM_H_MM / 2.0f) / sin_t;
            if (t < r) r = t;
        }

        scan.points[i].r_mm       = (uint16_t)lrintf(r);
        scan.points[i].theta_cdeg = (uint16_t)lrintf(theta_deg * 100.0f);
        scan.points[i].intensity = 200; /* Strong wall return */
    }

    return scan;
}

/*
 * Scenario: same room scan with a tiny (2 mm, 0.01 deg) perturbation per ray.
 * Simulates the robot has moved very slightly — used to test near-perfect ICP alignment.
 * Used by: test_scan_matcher
 */
lidar_scan_t lidar_stub_room_scan_perturbed(void)
{
    lidar_scan_t scan = lidar_stub_room_scan();

    for (int i = 0; i < scan.count; i++) {
        /* Add a tiny, deterministic offset */
        scan.points[i].r_mm       += 2u;
        scan.points[i].theta_cdeg += 1u;
    }

    return scan;
}
