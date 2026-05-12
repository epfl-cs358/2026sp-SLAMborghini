/**
 * hardware_pins.h
 * Central pin assignments for the SLAMborghini two-board hardware.
 *
 * Two-board architecture:
 *   ESP32-S3  (SLAM brain)       — LiDAR + IMU + SLAM + Wi-Fi dashboard
 *   Wemos D1 R32 (ESP32)         — motors + servo + AS5600 encoder + HC-SR04
 *
 * UART cross-connections:
 *   ESP32-S3 GPIO17 (BRIDGE_TX_PIN)  → Wemos GPIO16  (WEMOS_BRIDGE_RX_PIN)
 *   Wemos    GPIO17 (WEMOS_BRIDGE_TX_PIN) → ESP32-S3 GPIO16 (BRIDGE_RX_PIN)
 *
 * Edit only this file when rewiring.
 */

#ifndef HARDWARE_PINS_H
#define HARDWARE_PINS_H

#include "driver/uart.h"
#include "driver/gpio.h"

/* ══════════════════════════════════════════════════════════════════════════
 * ESP32-S3 — RPLiDAR C1 UART  (UART_NUM_1)
 * ══════════════════════════════════════════════════════════════════════════ */
#define LIDAR_UART_PORT   UART_NUM_1
#define LIDAR_UART_RX     14     /* LiDAR TX (yellow) → ESP32-S3 GPIO14    */
#define LIDAR_UART_TX     13     /* ESP32-S3 GPIO13 → LiDAR RX (green)     */
#define LIDAR_BAUD        460800


/* ══════════════════════════════════════════════════════════════════════════
 * ESP32-S3 → Wemos UART bridge  (UART_NUM_2)
 * ══════════════════════════════════════════════════════════════════════════ */
#define BRIDGE_UART_PORT  UART_NUM_2
#define BRIDGE_TX_PIN     GPIO_NUM_17  /* ESP32-S3 GPIO17 → Wemos GPIO16   */
#define BRIDGE_RX_PIN     GPIO_NUM_16  /* Wemos GPIO17    → ESP32-S3 GPIO16 */
#define BRIDGE_BAUD       921600

/* ══════════════════════════════════════════════════════════════════════════
 * Wemos D1 R32 — IBT-4 motor driver
 * ══════════════════════════════════════════════════════════════════════════ */
#define MOTOR_F_PIN       19     /* IO19 → IBT-4 IN1 (forward)             */
#define MOTOR_B_PIN       13     /* IO13 → IBT-4 IN2 (backward)            */

/* ══════════════════════════════════════════════════════════════════════════
 * Wemos D1 R32 — steering servo (PWM)
 * IO2 on the analog header of the Wemos D1 R32 = GPIO2.
 * ══════════════════════════════════════════════════════════════════════════ */
#define SERVO_PIN         2

/* ══════════════════════════════════════════════════════════════════════════
 * Wemos D1 R32 — shared I2C bus  (I2C_NUM_0)
 * Both the ICM-20948 IMU (addr 0x68) and the AS5600 encoder (addr 0x36)
 * share this bus.  Initialise I2C once; then address each device separately.
 * ══════════════════════════════════════════════════════════════════════════ */
#define WEMOS_I2C_SDA     21     /* shared SDA for IMU + encoder            */
#define WEMOS_I2C_SCL     22     /* shared SCL for IMU + encoder            */

/* Aliases used by imu_gyro.c */
#define IMU_I2C_PORT      I2C_NUM_0
#define IMU_SDA_PIN       WEMOS_I2C_SDA
#define IMU_SCL_PIN       WEMOS_I2C_SCL

/* Aliases used by AS5600 test / future encoder driver */
#define ENCODER_SDA_PIN   WEMOS_I2C_SDA
#define ENCODER_SCL_PIN   WEMOS_I2C_SCL

/* ══════════════════════════════════════════════════════════════════════════
 * Wemos D1 R32 — HC-SR04 ultrasonic sensors
 * Two sensors wired on four GPIOs.
 * TODO: verify which of each pair is TRIG vs ECHO.
 * ══════════════════════════════════════════════════════════════════════════ */
#define SONAR_A_TRIG_PIN  14
#define SONAR_A_ECHO_PIN  27
#define SONAR_B_TRIG_PIN  25
#define SONAR_B_ECHO_PIN  26

/* ══════════════════════════════════════════════════════════════════════════
 * Wemos D1 R32 — UART from ESP32-S3  (UART_NUM_2)
 * GPIO16/17 freed when LiDAR moved to the ESP32-S3.
 * ══════════════════════════════════════════════════════════════════════════ */
#define WEMOS_BRIDGE_UART_PORT  UART_NUM_2
#define WEMOS_BRIDGE_RX_PIN     16   /* from ESP32-S3 GPIO17                */
#define WEMOS_BRIDGE_TX_PIN     17   /* to   ESP32-S3 GPIO16                */
#define WEMOS_BRIDGE_BAUD       921600

#endif /* HARDWARE_PINS_H */
