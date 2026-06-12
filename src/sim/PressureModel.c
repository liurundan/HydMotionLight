#include "pressure_model.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define PRESSURE_MODEL_DEFAULT_DT_S 0.001f
#define PRESSURE_MODEL_PI 3.14159265358979323846f

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

static float pressure_model_wrap_unit(float value) {
    while (value >= 1.0f) {
        value -= 1.0f;
    }
    while (value < 0.0f) {
        value += 1.0f;
    }
    return value;
}

static uint32_t pressure_model_seed(uint32_t seed) {
    return seed == 0u ? 0xA341316Cu : seed;
}

static uint32_t pressure_model_next_u32(PressureModelState *state) {
    uint32_t x = pressure_model_seed(state->rng_state);

    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    state->rng_state = pressure_model_seed(x);
    return state->rng_state;
}

static float pressure_model_uniform01(PressureModelState *state) {
    return (pressure_model_next_u32(state) & 0x00ffffffu) / 16777216.0f;
}

static float pressure_model_gaussian(PressureModelState *state, float stddev) {
    float u1;
    float u2;
    float mag;

    if (stddev <= 0.0f) {
        return 0.0f;
    }

    if (state->has_spare_gauss) {
        state->has_spare_gauss = 0;
        return stddev * state->spare_gauss;
    }

    u1 = pressure_model_uniform01(state);
    u2 = pressure_model_uniform01(state);
    if (u1 < 1.0e-7f) {
        u1 = 1.0e-7f;
    }

    mag = sqrtf(-2.0f * logf(u1));
    state->spare_gauss = mag * sinf(2.0f * PRESSURE_MODEL_PI * u2);
    state->has_spare_gauss = 1;
    return stddev * (mag * cosf(2.0f * PRESSURE_MODEL_PI * u2));
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
    params->tooth_drop_depth_ratio = 0.10f;
    params->tooth_drop_width_ratio = 0.25f;
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
    float motor_noise;
    float process_noise;
    float sensor_noise;
    float q_pump;
    float q_base;
    float q_leak;
    float q_relief;
    float q_net;
    float d_pressure;
    float tooth_phase;
    float visible_pressure_pa;

    if (params == NULL || state == NULL || out == NULL) {
        return;
    }

    dt = dt_s > 0.0f ? dt_s : PRESSURE_MODEL_DEFAULT_DT_S;
    clamped_target = pressure_model_clampf(target_rpm, params->min_rpm, params->max_rpm);
    alpha = dt / (params->motor_tau_s + dt);
    motor_noise = 0.0f;
    process_noise = 0.0f;
    sensor_noise = 0.0f;

    if (params->enable_motor_noise) {
        motor_noise = pressure_model_gaussian(state, params->motor_noise_std_rpm);
    }

    state->motor_rpm += alpha * (clamped_target - state->motor_rpm) + motor_noise;
    state->motor_rpm = pressure_model_clampf(state->motor_rpm, params->min_rpm, params->max_rpm);
    state->pump_phase_rev = pressure_model_wrap_unit(
        state->pump_phase_rev + (state->motor_rpm * dt / 60.0f));

    q_base = params->pump_displacement_m3_rev * (state->motor_rpm / 60.0f);
    q_pump = q_base;
    if (state->motor_rpm > 0.01f) {
        q_pump *= 1.0f + params->flow_ripple_ratio *
                  sinf(2.0f * PRESSURE_MODEL_PI * 13.0f * state->pump_phase_rev);
    }

    if (params->enable_process_noise) {
        process_noise = pressure_model_gaussian(state, params->process_noise_std_m3_s);
    }

    q_leak = params->leak_coeff_m3_pa_s * state->pressure_pa;
    q_relief = 0.0f;
    if (state->pressure_pa > params->relief_set_pa) {
        q_relief = params->relief_coeff_m3_pa_s * (state->pressure_pa - params->relief_set_pa);
    }

    q_net = q_pump - q_leak - q_relief + process_noise;
    d_pressure = (params->bulk_modulus_pa / params->chamber_volume_m3) * q_net * dt;
    state->pressure_pa = pressure_model_maxf(0.0f, state->pressure_pa + d_pressure);

    visible_pressure_pa = state->pressure_pa;
    if (state->motor_rpm > 0.01f) {
        tooth_phase = pressure_model_wrap_unit(13.0f * state->pump_phase_rev);
        if (tooth_phase < params->tooth_drop_width_ratio) {
            float window = 0.5f * (1.0f + cosf((2.0f * PRESSURE_MODEL_PI * tooth_phase) /
                                               params->tooth_drop_width_ratio));
            float gain = 1.0f - params->tooth_drop_depth_ratio * window;
            visible_pressure_pa *= gain;
        }
    }

    if (params->enable_sensor_noise) {
        sensor_noise = pressure_model_gaussian(state, params->sensor_noise_std_bar);
    }

    out->actual_motor_rpm = state->motor_rpm;
    out->real_pressure_bar = state->pressure_pa * 1.0e-5f;
    out->measured_pressure_bar = pressure_model_clampf((visible_pressure_pa * 1.0e-5f) +
                                                       params->sensor_bias_bar +
                                                       sensor_noise,
                                                       0.0f,
                                                       params->sensor_range_bar);
    out->pump_flow_m3_s = q_pump;
    out->net_flow_m3_s = q_net;
    out->relief_active = (q_relief > 0.0f || state->pressure_pa > params->relief_set_pa) ? 1 : 0;
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
