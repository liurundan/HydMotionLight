#include "pressure_model.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define PRESSURE_MODEL_DEFAULT_DT_S 0.001f

static float pressure_model_clampf(float value, float min_value, float max_value) {
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static float pressure_model_maxf(float a, float b) {
    return (a > b) ? a : b;
}

static uint32_t pressure_model_seed(uint32_t seed) {
    return seed == 0u ? 0xA341316Cu : seed;
}

void PressureModel_InitParams(PressureModelParams *params) {
    if (params == NULL) {
        return;
    }

    memset(params, 0, sizeof(*params));
    params->pump_displacement_m3_rev = 20.0e-6f;
    params->bulk_modulus_pa = 1.6e9f;
    params->chamber_volume_m3 = 5.0e-4f;
    params->leak_coeff_m3_pa_s =
        (params->pump_displacement_m3_rev * (10.0f / 60.0f)) / (40.0f * 1.0e5f);
    params->relief_set_pa = 250.0f * 1.0e5f;
    params->relief_coeff_m3_pa_s = 1.2e-9f;
    params->sensor_range_bar = 250.0f;
    params->sensor_noise_std_bar = 0.4f;
    params->sensor_bias_bar = 0.0f;
    params->motor_tau_s = 0.05f;
    params->motor_noise_std_rpm = 2.0f;
    params->process_noise_std_m3_s = 0.0f;
    params->flow_ripple_ratio = 0.10f;
    params->tooth_drop_depth_ratio = 0.05f;
    params->tooth_drop_width_ratio = 0.05f;
    params->min_rpm = -100.0f;
    params->max_rpm = 2000.0f;
    params->enable_sensor_noise = 1u;
    params->enable_motor_noise = 1u;
    params->enable_process_noise = 0u;
}

void PressureModel_Reset(PressureModelState *state, uint32_t seed) {
    if (state == NULL) {
        return;
    }

    memset(state, 0, sizeof(*state));
    state->rng_state = pressure_model_seed(seed);
}

void PressureModel_Step(const PressureModelParams *params,
                        PressureModelState *state,
                        float target_rpm,
                        float dt_s,
                        PressureModelOutput *out) {
    float dt;
    float clamped_target;
    float alpha;
    float q_pump;
    float q_leak;
    float q_relief;
    float q_net;
    float d_pressure;

    if (params == NULL || state == NULL || out == NULL) {
        return;
    }

    dt = dt_s > 0.0f ? dt_s : PRESSURE_MODEL_DEFAULT_DT_S;
    clamped_target = pressure_model_clampf(target_rpm, params->min_rpm, params->max_rpm);
    alpha = dt / (params->motor_tau_s + dt);

    state->motor_rpm += alpha * (clamped_target - state->motor_rpm);
    state->motor_rpm = pressure_model_clampf(state->motor_rpm, params->min_rpm, params->max_rpm);

    q_pump = params->pump_displacement_m3_rev * (state->motor_rpm / 60.0f);
    q_leak = params->leak_coeff_m3_pa_s * state->pressure_pa;
    q_relief = 0.0f;
    if (state->pressure_pa > params->relief_set_pa) {
        q_relief = params->relief_coeff_m3_pa_s * (state->pressure_pa - params->relief_set_pa);
    }

    q_net = q_pump - q_leak - q_relief;
    d_pressure = (params->bulk_modulus_pa / params->chamber_volume_m3) * q_net * dt;
    state->pressure_pa = pressure_model_maxf(0.0f, state->pressure_pa + d_pressure);

    out->actual_motor_rpm = state->motor_rpm;
    out->real_pressure_bar = state->pressure_pa * 1.0e-5f;
    out->measured_pressure_bar = pressure_model_clampf(out->real_pressure_bar,
                                                       0.0f,
                                                       params->sensor_range_bar);
    out->pump_flow_m3_s = q_pump;
    out->net_flow_m3_s = q_net;
    out->relief_active = (q_relief > 0.0f) ? 1 : 0;
}

float pressure_update(float target_rpm,
                      float t,
                      float *P_state,
                      float *real_P,
                      float *actual_motor_rpm) {
    static int initialized = 0;
    static PressureModelParams params;
    static PressureModelState state;
    PressureModelOutput out;

    (void)t;

    if (!initialized) {
        PressureModel_InitParams(&params);
        PressureModel_Reset(&state, 0x2468ace1u);
        initialized = 1;
    }

    if (P_state != NULL) {
        state.pressure_pa = *P_state;
    }

    PressureModel_Step(&params, &state, target_rpm, PRESSURE_MODEL_DEFAULT_DT_S, &out);

    if (P_state != NULL) {
        *P_state = state.pressure_pa;
    }
    if (real_P != NULL) {
        *real_P = out.real_pressure_bar;
    }
    if (actual_motor_rpm != NULL) {
        *actual_motor_rpm = out.actual_motor_rpm;
    }

    return out.measured_pressure_bar;
}
