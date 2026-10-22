#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "pressure_model.h"

typedef struct {
    float rpm;
    float pressure_at_1s_bar;
    float pressure_at_5s_bar;
    float tail_pressure_bar;
    float tail_ripple_p2p_bar;
    float tail_torque_percent;
} PressureReference;

static const PressureReference kPressureReference[] = {
    {8.0f, 15.956f, 34.858f, 47.899f, 4.423f, -57.02f},
    {10.0f, 34.624f, 54.375f, 56.137f, 4.483f, -63.89f},
    {12.0f, 49.282f, 70.408f, 75.377f, 5.851f, -82.12f},
    {15.0f, 47.682f, 86.084f, 95.570f, 7.417f, -101.38f},
    {20.0f, 79.629f, 122.808f, 122.822f, 12.168f, -126.18f}
};

static void step_seconds(const PressureModelParams *params,
                         PressureModelState *state,
                         float rpm,
                         int seconds,
                         float *at_1s,
                         float *at_5s,
                         float *tail_mean,
                         float *tail_p2p,
                         float *tail_torque,
                         PressureModelOutput *out) {
    float min_pressure = 1.0e30f;
    float max_pressure = -1.0e30f;
    float pressure_sum = 0.0f;
    float torque_sum = 0.0f;
    int total_steps = seconds * 1000;
    int tail_start = total_steps - 2000;

    for (int step = 0; step < total_steps; ++step) {
        PressureModel_Step(params, state, rpm, 0.001f, out);
        if (step == 999) *at_1s = out->measured_pressure_bar;
        if (step == 4999) *at_5s = out->measured_pressure_bar;
        if (step >= tail_start) {
            pressure_sum += out->measured_pressure_bar;
            torque_sum += out->pumpFeedback.torquePermille;
            if (out->measured_pressure_bar < min_pressure) min_pressure = out->measured_pressure_bar;
            if (out->measured_pressure_bar > max_pressure) max_pressure = out->measured_pressure_bar;
        }
    }

    *tail_mean = pressure_sum / 2000.0f;
    *tail_p2p = max_pressure - min_pressure;
    *tail_torque = torque_sum / 2000.0f;
}

static void test_open_loop_data_fit(void) {
    PressureModelParams params;

    PressureModel_InitParams(&params);
    params.model_type = PRESSURE_MODEL_TYPE_PHYSICAL;
    params.enable_sensor_noise = 0u;
    params.enable_motor_noise = 0u;
    params.enable_process_noise = 0u;

    for (size_t i = 0; i < sizeof(kPressureReference) / sizeof(kPressureReference[0]); ++i) {
        PressureModelState state;
        PressureModelOutput out;
        float at_1s = 0.0f;
        float at_5s = 0.0f;
        float tail_mean = 0.0f;
        float tail_p2p = 0.0f;
        float tail_torque = 0.0f;
        const PressureReference *ref = &kPressureReference[i];

        PressureModel_Reset(&state, 0x5100u + (uint32_t)i);
        step_seconds(&params, &state, ref->rpm, 40,
                     &at_1s, &at_5s, &tail_mean, &tail_p2p, &tail_torque, &out);

        assert(fabsf(at_1s - ref->pressure_at_1s_bar) < 18.0f);
        assert(fabsf(at_5s - ref->pressure_at_5s_bar) < 18.0f);
        assert(fabsf(tail_mean - ref->tail_pressure_bar) < 16.0f);
        assert(tail_p2p > ref->tail_ripple_p2p_bar * 0.45f);
        assert(tail_p2p < ref->tail_ripple_p2p_bar * 1.80f);
        assert(fabsf(tail_torque - ref->tail_torque_percent) < 25.0f);
    }
}

static void test_zero_speed_releases_trapped_pressure_without_reset(void) {
    PressureModelParams params;
    PressureModelState state;
    PressureModelOutput out;
    float charged_pressure;

    PressureModel_InitParams(&params);
    params.model_type = PRESSURE_MODEL_TYPE_PHYSICAL;
    params.enable_sensor_noise = 0u;
    params.enable_motor_noise = 0u;
    params.enable_process_noise = 0u;
    PressureModel_Reset(&state, 0x5200u);

    for (int i = 0; i < 10000; ++i) {
        PressureModel_Step(&params, &state, 20.0f, 0.001f, &out);
    }
    charged_pressure = out.real_pressure_bar;
    for (int i = 0; i < 3000; ++i) {
        PressureModel_Step(&params, &state, 0.0f, 0.001f, &out);
    }

    assert(charged_pressure > 50.0f);
    assert(out.real_pressure_bar < charged_pressure * 0.25f);
    assert(out.real_pressure_bar >= 0.0f);
}

int main(void) {
    test_open_loop_data_fit();
    test_zero_speed_releases_trapped_pressure_without_reset();
    puts("PASS test_pressure_model_data_fit");
    return 0;
}
