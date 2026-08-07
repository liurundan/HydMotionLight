#ifndef OPEN10203040_MEASUREMENT_REFERENCE_H
#define OPEN10203040_MEASUREMENT_REFERENCE_H

/*
 * Provenance: derived from the versioned repository-root
 * open10203040-positive.csv fixture. The source columns are timestamp,
 * feedback-pressure, motor-speed, original-pressure, target-pressure,
 * set-rpm, feedback-torque, and feedback-angle. Every sequential timestamp
 * delta is +1 ms and target pressure is zero throughout the four full
 * set-rpm segments.
 *
 * These are measurement-reference facts only. They are not acceptance
 * targets for the uncalibrated pressure model.
 */
typedef struct {
    float command_rpm;
    int timestamp_start_ms;
    int timestamp_end_ms;
    float mean_feedback_rpm;
    float mean_feedback_pressure_bar;
    float tail_pressure_peak_to_peak_bar;
    float angle_synchronous_order13_amplitude_bar;
} Open10203040MeasurementReference;

static const Open10203040MeasurementReference kOpen10203040MeasurementReference[] = {
    {10.0f, -112067, -91596, 9.968689f, 211.975381f, 53.0f, 12.8f},
    {20.0f, -85652, -60342, 19.956185f, 546.623563f, 73.0f, 19.1f},
    {30.0f, -52607, -26227, 29.912399f, 902.754255f, 97.0f, 22.6f},
    {40.0f, -19128, -8052, 39.717794f, 1230.676537f, 112.0f, 21.7f}
};

enum {
    OPEN10203040_MEASUREMENT_REFERENCE_COUNT =
        (int)(sizeof(kOpen10203040MeasurementReference) /
              sizeof(kOpen10203040MeasurementReference[0])),
    OPEN10203040_MEASUREMENT_TIMESTAMP_DELTA_MS = 1,
    OPEN10203040_MEASUREMENT_TARGET_PRESSURE_BAR = 0
};

#endif /* OPEN10203040_MEASUREMENT_REFERENCE_H */
