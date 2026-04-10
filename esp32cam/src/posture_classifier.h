/**
 * posture_classifier.h
 * Module: Human posture classifier from camera frames.
 * Board: ESP32-CAM
 * Captures JPEG frames from the OV2640 camera module and runs a lightweight
 * inference model to classify the human posture as fallen, crouching, or standing.
 */

#ifndef POSTURE_CLASSIFIER_H
#define POSTURE_CLASSIFIER_H

#include "../../types.h"

/** Posture classification labels. */
typedef enum {
    POSTURE_UNKNOWN   = 0,
    POSTURE_STANDING  = 1,
    POSTURE_CROUCHING = 2,
    POSTURE_FALLEN    = 3
} posture_class_t;

/**
 * Initialize the ESP32-CAM camera peripheral and load model weights.
 * Configures the OV2640 sensor for QVGA (320x240) JPEG output.
 * Must be called once before posture_classifier_run().
 */
void posture_classifier_init(void);

/**
 * Capture one camera frame and run the posture classification model.
 * Blocking: waits for a fresh frame before running inference.
 * @return posture_class_t label for the detected human posture,
 *         or POSTURE_UNKNOWN if no human is visible.
 */
posture_class_t posture_classifier_run(void);

/**
 * Convert a posture_class_t label to a human-readable string.
 * @param cls The posture class to name.
 * @return Null-terminated string: "unknown", "standing", "crouching", or "fallen".
 */
const char *posture_class_name(posture_class_t cls);

#endif /* POSTURE_CLASSIFIER_H */
