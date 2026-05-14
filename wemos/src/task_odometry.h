#ifndef TASK_ODOMETRY_H
#define TASK_ODOMETRY_H

#include "encoder_ackermann_odometry.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

/* FreeRTOS task — call xTaskCreate(task_odometry, ...) once in app_main */
void task_odometry(void *pvParameters);

/* Atomically copy the latest Ackermann-fused pose into *out.
 * Safe to call from any task.  This is the pose that feeds odom_t packets
 * to the ESP32-S3 via task_pure_pursuit → uart_bridge_send_odom(). */
void task_odometry_copy_pose(odom_pose_t *out);

/* Internal accessor — same task only, no lock needed. */
const odom_pose_t *task_odometry_get_pose(void);

/* Set the current servo steering angle in radians (0 = straight).
 * Call from task_pure_pursuit immediately after writing the servo duty. */
void task_odometry_set_steering_rad(float steering_rad);

/* Return the shared I2C bus mutex.  Any future task that needs to access
 * the AS5600 or ICM-20948 must acquire this before touching the bus.
 * Returns NULL before task_odometry() has started. */
SemaphoreHandle_t task_odometry_get_i2c_mutex(void);

#endif