/**
 * types.h
 * Shared type definitions for SLAMborghini project.
 * Board-agnostic — no hardware-specific includes.
 * Used by ESP32-S3, Wemos D1 R32, and ESP32-CAM modules.
 */

#ifndef TYPES_H
#define TYPES_H

#include <stdint.h>
#include <stdbool.h>

/** A single point from the RPLiDAR C1 scan in polar coordinates. */
typedef struct {
    uint16_t r_mm;          /**< Range in millimetres. 0 = no return / clear ray */
    uint16_t theta_cdeg;    /**< Angle in centidegrees (degrees x100, 0..35999) */
    uint8_t  intensity;     /**< Return signal intensity (0-255) */
    uint8_t  _pad[3];
    uint32_t timestamp_us;  /**< Beam capture time (µs, esp_timer epoch); set by lidar_driver */
} lidar_scan_point_t;
_Static_assert(sizeof(lidar_scan_point_t) == 12u,
               "lidar_scan_point_t should stay compact: range mm + angle cdeg");

static inline float lidar_point_range_mm(const lidar_scan_point_t *p)
{
    return p ? (float)p->r_mm : 0.0f;
}

static inline float lidar_point_theta_deg(const lidar_scan_point_t *p)
{
    return p ? (float)p->theta_cdeg * 0.01f : 0.0f;
}

/** One full 360-degree LiDAR scan. */
typedef struct {
    lidar_scan_point_t points[460];       /**< Array of scan points */
    uint16_t           count;             /**< Number of valid points in this scan */
    uint16_t           _pad;
    uint32_t           scan_start_us;     /**< esp_timer_get_time() at first start-bit */
    uint32_t           rotation_period_us;/**< Time between consecutive start-bits (µs) */
} lidar_scan_t;

/** Odometry measurement: linear displacement + IMU yaw rate over dt. */
typedef struct {
    float    linear_disp_mm;    /**< Forward displacement in mm since last frame */
    float    yaw_rate_imu;      /**< Yaw rate from IMU in rad/s */
    float    dt_ms;             /**< Time delta in milliseconds */
    uint32_t seq;               /**< Monotonic counter — gaps indicate dropped packets */
    uint16_t consumed_wp_idx;   /**< Global path index last consumed by PP (streaming) */
    uint16_t consumed_path_id;  /**< Path ID that consumed_wp_idx belongs to */
} odom_t;

/** 2-D car pose with uncertainty. */
typedef struct {
    float x;      /**< X position in mm */
    float y;      /**< Y position in mm */
    float theta;  /**< Heading in radians */
    float cov[6]; /**< Upper-triangular covariance (xx, xy, xt, yy, yt, tt) */
} pose_t;

/** 2-D Cartesian point with intensity (output of polar-to-Cartesian conversion). */
typedef struct {
    int16_t x;         /**< X coordinate in millimetres */
    int16_t y;         /**< Y coordinate in millimetres */
    uint8_t intensity; /**< Scan intensity (0-255) */
} point2f_t;
_Static_assert(sizeof(point2f_t) == 6u,
               "point2f_t should stay compact: int16 mm coordinates");

/** Relative pose correction returned by scan matcher. */
typedef struct {
    float dx;     /**< Correction in X (mm) */
    float dy;     /**< Correction in Y (mm) */
    float dtheta; /**< Correction in heading (radians) */
    float score;  /**< Match quality score (0 = bad, 1 = perfect) */
} pose_correction_t;

/** A single navigation waypoint on a planned path. */
typedef struct {
    int16_t  x;        /**< X position in millimetres */
    int16_t  y;        /**< Y position in millimetres */
    float    theta;    /**< Desired heading at waypoint (radians) */
    uint16_t v_target; /**< Target speed in mm/s */
} waypoint_t;
_Static_assert(sizeof(waypoint_t) == 12u,
               "waypoint_t should stay compact: int16 mm coords + float heading");

/** Maximum number of waypoints sent over UART in one path frame. */
#define MAX_SHARED_PATH_POINTS 15

/** A short navigation path transmitted from ESP32-S3 to Wemos. */
typedef struct {
    uint8_t length;      /**< Number of valid waypoints */
    uint8_t reserved;    /**< Padding/reserved for alignment */
    waypoint_t waypoints[MAX_SHARED_PATH_POINTS];
} path_frame_t;

/** Waypoints per path chunk in the streaming protocol. */
#define PATH_CHUNK_WP_COUNT 8

/**
 * Streaming path chunk transmitted from ESP32-S3 to Wemos.
 * ESP32-S3 keeps the full path_t and refills Wemos's ring buffer in 8-wp chunks.
 * path_id lets Wemos discard stale chunks after a replan.
 * start_index is the global waypoint index of wp[0] in this chunk.
 */
typedef struct {
    uint16_t   path_id;                     /**< Incremented on every new plan */
    uint16_t   start_index;                 /**< Global index of first wp in this chunk */
    uint8_t    count;                       /**< Valid entries in wp[] (1–PATH_CHUNK_WP_COUNT) */
    bool       final_chunk;                 /**< True if this chunk contains the last waypoint */
    uint8_t    _pad[2];
    waypoint_t wp[PATH_CHUNK_WP_COUNT];     /**< Waypoint data */
} path_chunk_t;
/* path_chunk_t: 2+2+1+1+2 header + 8 x 12-byte waypoints = 104 bytes. */
_Static_assert(sizeof(path_chunk_t) == 104u,
               "path_chunk_t size mismatch — check waypoint_t and padding");

/** Control command transmitted from ESP32-S3 to Wemos D1 R32. */
typedef struct {
    float tx;        /**< Target X in mm (absolute or relative) */
    float ty;        /**< Target Y in mm (absolute or relative) */
    float t_heading; /**< Target heading in radians */
    float t_speed;   /**< Target speed in mm/s */
} control_frame_t;

/** Front ultrasonic emergency-brake event from Wemos to ESP32-S3. */
typedef struct {
    uint16_t distance_mm; /**< Latest front HC-SR04 distance; 0 if invalid */
    uint8_t  active;      /**< 1 = brake latched, 0 = cleared */
    uint8_t  seq;         /**< Low-rate sequence counter */
} front_hazard_t;

/** A single exploration frontier cell. */
typedef struct {
    float   cx;   /**< Centroid X in mm */
    float   cy;   /**< Centroid Y in mm */
    uint8_t size; /**< Open-space clearance score: free cells within
                   *   CLEARANCE_RADIUS of the target (0-49 range).
                   *   Higher = more navigable space around the target. */
} frontier_t;

/** List of detected exploration frontiers. */
typedef struct {
    frontier_t items[32]; /**< Array of frontiers */
    uint8_t    count;     /**< Number of valid frontiers */
} frontier_list_t;

/** Semantic class labels for LiDAR points. */
typedef enum {
    CLASS_UNKNOWN  = 0,
    CLASS_WALL     = 1,
    CLASS_OBSTACLE = 2,
    CLASS_GLASS    = 3,
    CLASS_PERSON   = 4,
    CLASS_FREE     = 5  /**< Explicitly free (traversed) space — used by
                         *   build_test_room() when pre-seeding free cells.
                         *   quadtree_map_insert(CLASS_FREE) must set leaf
                         *   occupancy to ~20 (OCC_FREE ≤ 50). */
} semantic_class_t;

/** A Cartesian point annotated with a semantic class. */
typedef struct {
    int16_t x;   /**< X coordinate in millimetres */
    int16_t y;   /**< Y coordinate in millimetres */
    uint8_t cls; /**< semantic_class_t stored as uint8_t */
} classified_point_t;
_Static_assert(sizeof(classified_point_t) == 6u,
               "classified_point_t should stay compact: int16 mm coordinates");

#endif /* TYPES_H */
