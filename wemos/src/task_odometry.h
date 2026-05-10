#ifndef TASK_ODOMETRY_H
#define TASK_ODOMETRY_H

#include "encoder_ackermann_odometry.h"

/* FreeRTOS task — call xTaskCreate(task_odometry, ...) once in app_main */
void task_odometry(void *pvParameters);

/* Get latest pose computed by the odometry task (internal use — same task only) */
const odom_pose_t *task_odometry_get_pose(void);

/* Atomically copy the latest pose into *out — safe to call from any task */
void task_odometry_copy_pose(odom_pose_t *out);

/* Set the current servo steering angle in radians (0 = straight).
 * Call from bridge_slave_task immediately after writing the servo duty. */
void task_odometry_set_steering_rad(float steering_rad);

#endif