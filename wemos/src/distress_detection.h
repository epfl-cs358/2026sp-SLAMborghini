/**
 * distress_detection.h
 * Module: Human distress detector using proximity and audio signals.
 * Board: Wemos D1 R32
 * Fuses HC-SR04 proximity distance and an audio loudness level to detect whether
 * a human in the sensor's field of view may be in distress (e.g. fallen, calling for help).
 */

#ifndef DISTRESS_DETECTION_H
#define DISTRESS_DETECTION_H

#include <stdbool.h>

/**
 * Evaluate whether a distress condition is detected based on sensor inputs.
 * Returns true if the distance is below the proximity threshold AND the audio
 * level exceeds the audio threshold simultaneously.
 * @param distance_mm  Distance measured by HC-SR04 in millimetres.
 * @param audio_level  Normalized audio loudness level in range [0.0, 1.0].
 * @return true if a distress pattern is detected, false otherwise.
 */
bool distress_detect(float distance_mm, float audio_level);

/**
 * Configure the detection thresholds for proximity and audio.
 * @param dist_mm     Proximity threshold in millimetres (trigger if distance < dist_mm).
 * @param audio_thresh Audio threshold in [0.0, 1.0] (trigger if audio > audio_thresh).
 */
void distress_set_threshold(float dist_mm, float audio_thresh);

#endif /* DISTRESS_DETECTION_H */
