/**
 * esp32cam/main.c
 * ESP32-CAM application entry point for SLAMborghini.
 * Board: ESP32-CAM (Vision Brain)
 * Initializes the camera and runs the posture classifier in a loop.
 */

#include "src/posture_classifier.h"

void app_main(void)
{
    // TODO: implement full vision loop

    /* --- Module initialization --- */
    posture_classifier_init();

    // TODO: implement main vision loop
    // while (1) {
    //     posture_class_t cls = posture_classifier_run();
    //     const char *name = posture_class_name(cls);
    //
    //     if (cls == POSTURE_FALLEN || cls == POSTURE_CROUCHING) {
    //         /* TODO: send distress alert to ESP32-S3 via UART or Wi-Fi */
    //     }
    //
    //     /* vTaskDelay(pdMS_TO_TICKS(200)); */
    // }
}
