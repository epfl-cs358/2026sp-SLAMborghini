#ifndef IMU_ENCODER_DRIVER_H
#define IMU_ENCODER_DRIVER_H

#include "esp_err.h"
#include <stdint.h>

/* Latest sensor sample */
typedef struct {
    float    distance_m;    /* cumulative traveled distance in meters */
    float    yaw_rad;       /* current yaw in radians */
    uint32_t timestamp_ms;
} imu_encoder_sample_t;

/* Initialize IMU and AS5600 encoder */
esp_err_t imu_encoder_driver_init(void);

/* Read IMU and encoder, update internal sample — call every 10 ms */
esp_err_t imu_encoder_driver_update(void);

/* Get full latest sample */
imu_encoder_sample_t imu_encoder_driver_get_sample(void);

/* Get cumulative distance in meters */
float imu_encoder_driver_get_distance_m(void);

/* Get current yaw in radians */
float imu_encoder_driver_get_yaw_rad(void);

#endif /* IMU_ENCODER_DRIVER_H */