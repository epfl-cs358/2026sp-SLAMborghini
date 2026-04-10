/**
 * hcsr04_driver.h
 * Module: HC-SR04 ultrasonic distance sensor driver.
 * Board: Wemos D1 R32
 * Controls the HC-SR04 TRIG/ECHO GPIO interface and converts the echo pulse
 * duration to a distance measurement in millimetres.
 */

#ifndef HCSR04_DRIVER_H
#define HCSR04_DRIVER_H

#include <stdint.h>

/**
 * Configure the TRIG and ECHO GPIO pins for the HC-SR04.
 * Sets TRIG as output (initially low) and ECHO as input.
 * @param trig_pin GPIO pin number connected to HC-SR04 TRIG.
 * @param echo_pin GPIO pin number connected to HC-SR04 ECHO.
 */
void hcsr04_init(uint8_t trig_pin, uint8_t echo_pin);

/**
 * Trigger a single ultrasonic measurement and return the measured distance.
 * Sends a 10 µs TRIG pulse, waits for the ECHO rising edge, times the echo pulse,
 * and converts pulse duration to distance using the speed of sound.
 * @return Distance in millimetres, or 0.0f on timeout (no echo received).
 */
float hcsr04_read_mm(void);

#endif /* HCSR04_DRIVER_H */
