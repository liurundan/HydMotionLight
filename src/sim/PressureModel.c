#include "pressure_model.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define PRESSURE_MODEL_DEFAULT_DT_S 0.001f
#define PRESSURE_MODEL_PI 3.14159265358979323846f
#define PRESSURE_MODEL_DEFAULT_PUMP_DISPLACEMENT_M3_REV 20.0e-6f
#define PRESSURE_MODEL_DEFAULT_VEFF_BASE_M3 5.0e-4f
#define PRESSURE_MODEL_DEFAULT_LEAK_BASE_M3_PA_S \
    ((PRESSURE_MODEL_DEFAULT_PUMP_DISPLACEMENT_M3_REV * (10.0f / 60.0f)) / (40.0f * 1.0e5f))
#define PRESSURE_MODEL_DEFAULT_TOOTH_DROP_DEPTH_BASE 0.10f

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

static float pressure_model_absf(float value) {
    return (value < 0.0f) ? -value : value;
}

static float pressure_model_interp3(const float values[3], float abs_rpm) {
    if (abs_rpm <= 20.0f) {
        float t = abs_rpm / 20.0f;
        return values[0] + (values[1] - values[0]) * t;
    }
    if (abs_rpm >= 40.0f) {
        return values[2];
    }
    {
        float t = (abs_rpm - 20.0f) / 20.0f;
        return values[1] + (values[2] - values[1]) * t;
    }
}

static float pressure_model_effective_volume(const PressureModelParams *params, float motor_rpm) {
    float legacy_scale = 1.0f;

    if (PRESSURE_MODEL_DEFAULT_VEFF_BASE_M3 > 0.0f) {
        legacy_scale = params->chamber_volume_m3 / PRESSURE_MODEL_DEFAULT_VEFF_BASE_M3;
    }
    return params->veff_base_m3 *
           pressure_model_interp3(params->veff_speed_scale, pressure_model_absf(motor_rpm)) *
           legacy_scale;
}

static float pressure_model_leak_coeff(const PressureModelParams *params, float motor_rpm) {
    float legacy_scale = 1.0f;

    if (PRESSURE_MODEL_DEFAULT_LEAK_BASE_M3_PA_S > 0.0f) {
        legacy_scale = params->leak_coeff_m3_pa_s / PRESSURE_MODEL_DEFAULT_LEAK_BASE_M3_PA_S;
    }
    return params->leak_base_m3_pa_s *
           pressure_model_interp3(params->leak_speed_scale, pressure_model_absf(motor_rpm)) *
           legacy_scale;
}

static float pressure_model_tooth_drop_depth(const PressureModelParams *params, float motor_rpm) {
    float legacy_scale = 1.0f;

    if (PRESSURE_MODEL_DEFAULT_TOOTH_DROP_DEPTH_BASE > 0.0f) {
        legacy_scale = params->tooth_drop_depth_ratio / PRESSURE_MODEL_DEFAULT_TOOTH_DROP_DEPTH_BASE;
    }
    return params->tooth_drop_depth_base *
           pressure_model_interp3(params->drop_depth_scale, pressure_model_absf(motor_rpm)) *
           legacy_scale;
}

static float pressure_model_tooth_drop_phase(const PressureModelParams *params, float motor_rpm) {
    return params->tooth_drop_phase_base +
           pressure_model_interp3(params->drop_phase_offset, pressure_model_absf(motor_rpm));
}

static float pressure_model_wrap_unit(float value) {
    if (!isfinite(value)) {
        return NAN;
    }
    value = fmodf(value, 1.0f);
    if (value < 0.0f) {
        value += 1.0f;
    }
    return value;
}

static void pressure_model_fill_feedback(const PressureModelState *state,
                                         PressureModelOutput *out) {
    float angle;
    float phase_degrees;
    int angle_valid;

    if (state == NULL || out == NULL) return;
    angle = 0.0f;
    angle_valid = 0;
    phase_degrees = state->pump_phase_rev * 360.0f;
    if (isfinite(phase_degrees)) {
        angle = fmodf(phase_degrees, 360.0f);
        if (angle < 0.0f) angle += 360.0f;
        angle_valid = isfinite(angle);
    }
    memset(&out->pumpFeedback, 0, sizeof(out->pumpFeedback));
    out->pumpFeedback.torquePermille = 0.0f;
    if (isfinite(out->actual_motor_rpm)) {
        out->pumpFeedback.rpm = out->actual_motor_rpm;
        out->pumpFeedback.validFlags |= HYD_PUMP_FEEDBACK_VALID_RPM;
    }
    if (angle_valid) {
        out->pumpFeedback.angleDeg = angle;
        out->pumpFeedback.validFlags |= HYD_PUMP_FEEDBACK_VALID_ANGLE;
    }
    if (isfinite(state->timestamp_s)) {
        out->pumpFeedback.timestamp = state->timestamp_s;
        out->pumpFeedback.validFlags |= HYD_PUMP_FEEDBACK_VALID_TIMESTAMP;
    }
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
    params->pump_displacement_m3_rev = PRESSURE_MODEL_DEFAULT_PUMP_DISPLACEMENT_M3_REV;
    params->bulk_modulus_pa = 1.6e9f;
    params->chamber_volume_m3 = PRESSURE_MODEL_DEFAULT_VEFF_BASE_M3;
    params->leak_coeff_m3_pa_s = PRESSURE_MODEL_DEFAULT_LEAK_BASE_M3_PA_S;
    /* Task 2 activates the fitted pressure skeleton while leaving legacy knobs available as compatibility multipliers. */
    params->veff_base_m3 = 4.4e-4f;
    params->leak_base_m3_pa_s = 1.2245e-12f;
    params->relief_set_pa = 250.0f * 1.0e5f;
    params->relief_coeff_m3_pa_s = 1.2e-9f;
    params->sensor_range_bar = 250.0f;
    params->sensor_noise_std_bar = 0.4f;
    params->sensor_bias_bar = 0.0f;
    params->motor_tau_s = 0.06f;
    params->motor_noise_std_rpm = 2.0f;
    params->process_noise_std_m3_s = 0.0f;
    params->flow_ripple_ratio = 0.08f;
    params->tooth_drop_depth_ratio = PRESSURE_MODEL_DEFAULT_TOOTH_DROP_DEPTH_BASE;
    params->tooth_drop_depth_base = 0.16f;
    params->tooth_drop_width_ratio = 0.20f;
    params->tooth_drop_phase_base = 0.58f;
    params->veff_speed_scale[0] = 1.18f;
    params->veff_speed_scale[1] = 1.0f;
    params->veff_speed_scale[2] = 0.86f;
    params->leak_speed_scale[0] = 1.27f;
    params->leak_speed_scale[1] = 1.0f;
    params->leak_speed_scale[2] = 0.875f;
    params->drop_depth_scale[0] = 1.00f;
    params->drop_depth_scale[1] = 0.62f;
    params->drop_depth_scale[2] = 0.30f;
    params->drop_phase_offset[0] = 0.00f;
    params->drop_phase_offset[1] = 0.07f;
    params->drop_phase_offset[2] = 0.11f;
    params->torque_bias = 400.0f;
    params->torque_from_pressure_gain = 110.0f;
    params->torque_from_speed_gain = 8.0f;
    params->model_type = PRESSURE_MODEL_TYPE_FIRST_ORDER;
    params->first_order_k_bar_per_rpm = 5.4f;
    params->first_order_tau_s = 1.0f;
    params->first_order_delay_s = 0.0f;
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
    state->active_model_type = PRESSURE_MODEL_TYPE_PHYSICAL;
    state->first_order_prev_pressure_bar = 0.0f;
    state->first_order_buffer_index = 0;
}

static unsigned char pressure_model_normalize_type(unsigned char model_type) {
    return (model_type == PRESSURE_MODEL_TYPE_FIRST_ORDER)
               ? PRESSURE_MODEL_TYPE_FIRST_ORDER
               : PRESSURE_MODEL_TYPE_PHYSICAL;
}

static int pressure_model_first_order_delay_steps(float delay_s, float dt_s) {
    float clamped_delay = pressure_model_clampf(delay_s, 0.0f, 1.0f);
    float ratio = clamped_delay / dt_s;
    int steps = (int)ratio;

    if (steps < 0) {
        steps = 0;
    }
    if (steps >= PRESSURE_MODEL_FIRST_ORDER_MAX_DELAY_STEPS) {
        steps = PRESSURE_MODEL_FIRST_ORDER_MAX_DELAY_STEPS - 1;
    }
    return steps;
}

static void pressure_model_fill_first_order_history(PressureModelState *state, float pressure_bar) {
    int i;

    for (i = 0; i < PRESSURE_MODEL_FIRST_ORDER_MAX_DELAY_STEPS; ++i) {
        state->first_order_delay_buffer[i] = pressure_bar;
    }
    state->first_order_buffer_index = 0;
}

static void pressure_model_step_first_order(const PressureModelParams *params,
                                            PressureModelState *state,
                                            float dt,
                                            float abs_motor_rpm,
                                            PressureModelOutput *out) {
    float gain = pressure_model_maxf(0.0f, params->first_order_k_bar_per_rpm);
    float tau = pressure_model_maxf(0.0f, params->first_order_tau_s);
    float undelayed_bar;
    float delayed_bar;
    int delay_steps;

    if (tau > 0.0f) {
        undelayed_bar = ((gain * state->motor_rpm * dt) +
                         (tau * state->first_order_prev_pressure_bar)) /
                        (tau + dt);
    } else {
        undelayed_bar = gain * state->motor_rpm;
    }

    undelayed_bar = pressure_model_clampf(undelayed_bar, 0.0f, 250.0f);
    delay_steps = pressure_model_first_order_delay_steps(params->first_order_delay_s, dt);

    state->first_order_delay_buffer[state->first_order_buffer_index] = undelayed_bar;
    if (delay_steps == 0) {
        delayed_bar = undelayed_bar;
    } else {
        int read_index = (state->first_order_buffer_index +
                          PRESSURE_MODEL_FIRST_ORDER_MAX_DELAY_STEPS -
                          delay_steps) %
                         PRESSURE_MODEL_FIRST_ORDER_MAX_DELAY_STEPS;
        delayed_bar = state->first_order_delay_buffer[read_index];
    }

    state->first_order_buffer_index =
        (state->first_order_buffer_index + 1) % PRESSURE_MODEL_FIRST_ORDER_MAX_DELAY_STEPS;
    state->first_order_prev_pressure_bar = undelayed_bar;
    state->pressure_pa = delayed_bar * 1.0e5f;

    out->actual_motor_rpm = state->motor_rpm;
    out->real_pressure_bar = delayed_bar;
    out->measured_pressure_bar = delayed_bar;
    out->pump_flow_m3_s = 0.0f;
    out->net_flow_m3_s = 0.0f;
    out->relief_active = (undelayed_bar >= 250.0f) ? 1 : 0;
    out->estimated_torque_trend = pressure_model_maxf(
        0.0f,
        params->torque_bias +
            params->torque_from_pressure_gain * out->measured_pressure_bar +
            params->torque_from_speed_gain * abs_motor_rpm);
    pressure_model_fill_feedback(state, out);
}

static void pressure_model_write_switch_hold_output(const PressureModelParams *params,
                                                    PressureModelState *state,
                                                    float abs_motor_rpm,
                                                    float preserved_pressure_bar,
                                                    int first_order_mode,
                                                    PressureModelOutput *out) {
    if (first_order_mode) {
        state->first_order_prev_pressure_bar = preserved_pressure_bar;
        pressure_model_fill_first_order_history(state, preserved_pressure_bar);
    }

    state->pressure_pa = preserved_pressure_bar * 1.0e5f;
    out->actual_motor_rpm = state->motor_rpm;
    out->real_pressure_bar = preserved_pressure_bar;
    out->measured_pressure_bar = preserved_pressure_bar;
    out->pump_flow_m3_s = 0.0f;
    out->net_flow_m3_s = 0.0f;
    out->relief_active = 0;
    out->estimated_torque_trend = pressure_model_maxf(
        0.0f,
        params->torque_bias +
            params->torque_from_pressure_gain * out->measured_pressure_bar +
            params->torque_from_speed_gain * abs_motor_rpm);
    pressure_model_fill_feedback(state, out);
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
    float abs_motor_rpm;

    if (params == NULL || state == NULL || out == NULL) {
        return;
    }

    dt = (isfinite(dt_s) && dt_s > 0.0f) ? dt_s : PRESSURE_MODEL_DEFAULT_DT_S;
    state->timestamp_s += dt;
    clamped_target = isfinite(target_rpm)
        ? pressure_model_clampf(target_rpm, params->min_rpm, params->max_rpm)
        : 0.0f;
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
    abs_motor_rpm = pressure_model_absf(state->motor_rpm);

    {
        unsigned char requested_type = pressure_model_normalize_type(params->model_type);
        float preserved_pressure_bar = state->pressure_pa * 1.0e-5f;

        if (requested_type != state->active_model_type) {
            if (requested_type == PRESSURE_MODEL_TYPE_FIRST_ORDER &&
                state->active_model_type == PRESSURE_MODEL_TYPE_PHYSICAL &&
                preserved_pressure_bar <= 0.0f) {
                state->first_order_prev_pressure_bar = preserved_pressure_bar;
                pressure_model_fill_first_order_history(state, preserved_pressure_bar);
            } else {
                pressure_model_write_switch_hold_output(params,
                                                        state,
                                                        abs_motor_rpm,
                                                        preserved_pressure_bar,
                                                        requested_type == PRESSURE_MODEL_TYPE_FIRST_ORDER,
                                                        out);
                state->active_model_type = requested_type;
                return;
            }
        }

        if (requested_type == PRESSURE_MODEL_TYPE_FIRST_ORDER) {
            pressure_model_step_first_order(params, state, dt, abs_motor_rpm, out);
            state->active_model_type = PRESSURE_MODEL_TYPE_FIRST_ORDER;
            return;
        }
    }

    q_base = params->pump_displacement_m3_rev * (state->motor_rpm / 60.0f);
    q_pump = q_base;
    if (state->motor_rpm > 0.01f) {
        q_pump *= 1.0f + params->flow_ripple_ratio *
                  sinf(2.0f * PRESSURE_MODEL_PI * 13.0f * state->pump_phase_rev);
    }

    if (params->enable_process_noise) {
        process_noise = pressure_model_gaussian(state, params->process_noise_std_m3_s);
    }

    q_leak = pressure_model_leak_coeff(params, state->motor_rpm) * state->pressure_pa;
    q_relief = 0.0f;
    if (state->pressure_pa > params->relief_set_pa) {
        q_relief = params->relief_coeff_m3_pa_s * (state->pressure_pa - params->relief_set_pa);
    }

    q_net = q_pump - q_leak - q_relief + process_noise;
    d_pressure = (params->bulk_modulus_pa /
                  pressure_model_effective_volume(params, state->motor_rpm)) *
                 q_net * dt;
    state->pressure_pa = pressure_model_maxf(0.0f, state->pressure_pa + d_pressure);

    visible_pressure_pa = state->pressure_pa;
    if (state->motor_rpm > 0.01f) {
        float depth = pressure_model_tooth_drop_depth(params, state->motor_rpm);
        float phase = pressure_model_tooth_drop_phase(params, state->motor_rpm);

        tooth_phase = pressure_model_wrap_unit(13.0f * state->pump_phase_rev - phase);
        if (tooth_phase < params->tooth_drop_width_ratio) {
            float window = 0.5f * (1.0f + cosf((PRESSURE_MODEL_PI * tooth_phase) /
                                               params->tooth_drop_width_ratio));
            float gain = 1.0f - depth * window;
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
    out->estimated_torque_trend = pressure_model_maxf(
        0.0f,
        params->torque_bias +
            params->torque_from_pressure_gain * out->measured_pressure_bar +
            params->torque_from_speed_gain * abs_motor_rpm);
    out->relief_active = (q_relief > 0.0f || state->pressure_pa > params->relief_set_pa) ? 1 : 0;
    state->active_model_type = PRESSURE_MODEL_TYPE_PHYSICAL;
    pressure_model_fill_feedback(state, out);
}

float pressure_update(float target_rpm,
                      float t,
                      float *P_state,
                      float *real_P,
                      float *actual_motor_rpm) {
    static int initialized = 0;
    static PressureModelParams params;
    static PressureModelState state;
    static float last_time_s = 0.0f;
    PressureModelOutput out;
    float dt_s;

    if (!initialized) {
        PressureModel_InitParams(&params);
        PressureModel_Reset(&state, 0x2468ace1u);
        last_time_s = 0.0f;
        initialized = 1;
    }

    if (P_state != NULL) {
        state.pressure_pa = *P_state;
    }

    if (t > last_time_s) {
        dt_s = t - last_time_s;
    } else {
        dt_s = PRESSURE_MODEL_DEFAULT_DT_S;
    }
    last_time_s = t;

    PressureModel_Step(&params, &state, target_rpm, dt_s, &out);

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
