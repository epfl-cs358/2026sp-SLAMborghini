#ifndef TASK_ODOMETRY_H
#define TASK_ODOMETRY_H

#include "encoder_ackermann_odometry.h"

/* FreeRTOS task — call xTaskCreate(task_odometry, ...) once in app_main */
void task_odometry(void *pvParameters);

/* Get latest pose computed by the odometry task */
const odom_pose_t *task_odometry_get_pose(void);

#endif