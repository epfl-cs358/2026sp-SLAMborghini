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
    float   r_mm;       /**< Range in millimetres */
    float   theta_deg;  /**< Angle in degrees (0-360) */
    uint8_t intensity;  /**< Return signal intensity (0-255) */
} lidar_scan_point_t;

/** One full 360-degree LiDAR scan. */
typedef struct {
    lidar_scan_point_t points[460]; /**< Array of scan points */
    uint16_t           count;       /**< Number of valid points in this scan */
} lidar_scan_t;

/** Odometry measurement: linear displacement + IMU yaw rate over dt. */
typedef struct {
    float linear_disp_mm; /**< Forward displacement in mm since last frame */
    float yaw_rate_imu;   /**< Yaw rate from IMU in rad/s */
    float dt_ms;          /**< Time delta in milliseconds */
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
    float   x;         /**< X coordinate in mm */
    float   y;         /**< Y coordinate in mm */
    uint8_t intensity; /**< Scan intensity (0-255) */
} point2f_t;

/** Relative pose correction returned by scan matcher. */
typedef struct {
    float dx;     /**< Correction in X (mm) */
    float dy;     /**< Correction in Y (mm) */
    float dtheta; /**< Correction in heading (radians) */
    float score;  /**< Match quality score (0 = bad, 1 = perfect) */
} pose_correction_t;

/** A single navigation waypoint on a planned path. */
typedef struct {
    float x;        /**< X position in mm */
    float y;        /**< Y position in mm */
    float theta;    /**< Desired heading at waypoint (radians) */
    float v_target; /**< Target speed at waypoint (mm/s) */
} waypoint_t;

/** Control command transmitted from ESP32-S3 to Wemos D1 R32. */
typedef struct {
    float tx;        /**< Target X in mm (absolute or relative) */
    float ty;        /**< Target Y in mm (absolute or relative) */
    float t_heading; /**< Target heading in radians */
    float t_speed;   /**< Target speed in mm/s */
} control_frame_t;

/** A single exploration frontier cell. */
typedef struct {
    float   cx;   /**< Centroid X in mm */
    float   cy;   /**< Centroid Y in mm */
    uint8_t size; /**< Frontier size (number of cells) */
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
    CLASS_PERSON   = 4
} semantic_class_t;

/** A Cartesian point annotated with a semantic class. */
typedef struct {
    float           x;   /**< X coordinate in mm */
    float           y;   /**< Y coordinate in mm */
    semantic_class_t cls; /**< Semantic classification */
} classified_point_t;

#endif /* TYPES_H */
