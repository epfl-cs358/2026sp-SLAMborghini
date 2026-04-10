/**
 * hcsr04_driver.c
 * Module: HC-SR04 ultrasonic distance sensor driver.
 * Board: Wemos D1 R32
 * Implementation phase: stub (GPIO timing not yet implemented)
 */

#include "hcsr04_driver.h"

void hcsr04_init(uint8_t trig_pin, uint8_t echo_pin)
{
    // TODO: implement
    // Store trig_pin and echo_pin in module-level static variables.
    // gpio_set_direction(trig_pin, GPIO_MODE_OUTPUT)
    // gpio_set_level(trig_pin, 0)
    // gpio_set_direction(echo_pin, GPIO_MODE_INPUT)
    (void)trig_pin;
    (void)echo_pin;
}

float hcsr04_read_mm(void)
{
    // TODO: implement
    // 1. Set TRIG high for 10 µs, then set low.
    // 2. Wait for ECHO to go high (timeout = 30 ms).
    // 3. Record start time (esp_timer_get_time()).
    // 4. Wait for ECHO to go low.
    // 5. duration_us = stop_time - start_time
    // 6. distance_mm = duration_us * 0.1715f  (speed of sound / 2 in mm/us)
    // 7. Return 0 on timeout.
    return 0;
}
