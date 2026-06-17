#ifndef PRESSURE_MODEL_OPEN_LOOP_REFERENCE_H
#define PRESSURE_MODEL_OPEN_LOOP_REFERENCE_H

typedef struct {
    float command_rpm;
    int sample_count;
    float head_pressure_bar;
    float tail_pressure_bar;
    float head_motor_rpm;
    float tail_motor_rpm;
    float tail_tooth_span_bar;
    float tail_tooth_min_phase;
    float tail_torque_trend;
} PressureModelOpenLoopReference;

static const PressureModelOpenLoopReference kPressureModelOpenLoopReference[] = {
    {10.0f, 20472, 17.5155f, 21.4452f, 9.6740f, 10.0500f, 3.4294f, 0.596154f, 2796.4850f},
    {20.0f, 25311, 44.1764f, 54.4452f, 19.3135f, 20.0320f, 4.9955f, 0.673077f, 6229.6050f},
    {30.0f, 26381, 73.7124f, 88.8450f, 28.9025f, 29.9760f, 5.5772f, 0.711538f, 9707.6300f},
    {40.0f, 11077, 102.1228f, 125.4012f, 38.5440f, 39.9270f, 5.4218f, 0.711538f, 13550.1800f}
};

enum {
    PRESSURE_MODEL_OPEN_LOOP_REFERENCE_COUNT =
        (int)(sizeof(kPressureModelOpenLoopReference) / sizeof(kPressureModelOpenLoopReference[0]))
};

#endif /* PRESSURE_MODEL_OPEN_LOOP_REFERENCE_H */
