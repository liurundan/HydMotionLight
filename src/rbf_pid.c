#include "rbf_pid.h"
#include "hyd_config.h"

#include <math.h>
#include <string.h>

static const float RBF_PID_ERROR_DEADBAND = 0.0005f;
static const float RBF_PID_SOFT_CAP_RATIO = 1.05f;
static const float RBF_PID_LEARNING_RATIO_TIGHT = 0.01f;
static const float RBF_PID_LEARNING_RATIO_NEAR = 0.05f;
static const float RBF_PID_LEARNING_RATIO_MID = 0.10f;
static const float RBF_PID_LEARNING_SCALE_TIGHT = 0.02f;
static const float RBF_PID_LEARNING_SCALE_NEAR = 0.10f;
static const float RBF_PID_LEARNING_SCALE_MID = 0.25f;
static const float RBF_PID_NEAR_TARGET_RATIO = 0.02f;
static const float RBF_PID_ACCEL_FF_GAIN = -0.15f;
static const float RBF_PID_DYNAMIC_FF_GAIN = 0.001f;
static const float RBF_PID_WEIGHT_LIMIT = 5.0f;

static float rbf_pid_compute_soft_flow_cap(const RBF_PID_Handle *pid);

static float clampf(float min_value, float value, float max_value) {
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static float clamp_positive_or_default(float value, float fallback) {
    return value > 0.0f ? value : fallback;
}

static float clamp_finite(float min_value, float value, float max_value, float fallback) {
    if (!isfinite(value)) {
        value = fallback;
    }
    return clampf(min_value, value, max_value);
}

static float finite_or_default(float value, float fallback) {
    return isfinite(value) ? value : fallback;
}

static float rbf_pid_clamp_adaptive_value(const RBF_PID_Handle *pid,
                                          float min_value,
                                          float value,
                                          float max_value,
                                          float fallback) {
    if (pid->control_mode == RBF_PID_CONTROL_MODE_PI) {
        return clamp_finite(min_value, value, max_value, fallback);
    }
    return clampf(min_value, value, max_value);
}

static float rbf_pid_max_flow_output(const RBF_PID_Handle *pid) {
    float max_output = pid->fMaxFlow * pid->fFlowRateLimit;

    return max_output > 0.0f ? max_output : 90.0f;
}

static float rbf_pid_min_flow_output(const RBF_PID_Handle *pid) {
    return pid->output_min_flow;
}

static float rbf_pid_output_lower_bound(const RBF_PID_Handle *pid) {
    float upper = rbf_pid_max_flow_output(pid);
    float lower = rbf_pid_min_flow_output(pid);

    if (lower > upper) {
        lower = upper;
    }

    return lower;
}

static float rbf_pid_output_upper_bound(const RBF_PID_Handle *pid) {
    float upper = pid->output_max_flow;

    if (upper <= 0.0f) {
        upper = rbf_pid_max_flow_output(pid);
    }

    if (upper < rbf_pid_output_lower_bound(pid)) {
        upper = rbf_pid_output_lower_bound(pid);
    }

    return upper;
}

static void sort_pair(float *low, float *high) {
    if (*low > *high) {
        float temp = *low;
        *low = *high;
        *high = temp;
    }
}

static void rbf_pid_apply_default_limits(RBF_PID_Handle *pid) {
    pid->min_KP = PID_MIN_KP;
    pid->max_KP = PID_MAX_KP;
    pid->min_KI = PID_MIN_KI;
    pid->max_KI = PID_MAX_KI;
    pid->min_KD = PID_MIN_KD;
    pid->max_KD = PID_MAX_KD;
}

static void rbf_pid_apply_default_learning_rates(RBF_PID_Handle *pid) {
    pid->eta_w = HYD_DEFAULT_RBF_W_LEARNING_RATE;
    pid->eta_c = HYD_DEFAULT_RBF_C_LEARNING_RATE;
    pid->eta_b = HYD_DEFAULT_RBF_B_LEARNING_RATE;
    pid->eta_p = HYD_DEFAULT_PID_P_LEARNING_RATE;
    pid->eta_i = HYD_DEFAULT_PID_I_LEARNING_RATE;
    pid->eta_d = HYD_DEFAULT_PID_D_LEARNING_RATE;
}

static void rbf_pid_apply_default_gains(RBF_PID_Handle *pid) {
    pid->KP = 0.04f;
    pid->KI = PID_MIN_KI;
    pid->KD = 0.020f;
}

static void rbf_pid_refresh_gain_compensation(RBF_PID_Handle *pid) {
    pid->gain_compensation_enabled = (pid->K > 0.0f);
    pid->gain_compensation_factor = 1.0f;
    pid->fGainCompensation = pid->K;
}

static void rbf_pid_init_network(RBF_PID_Handle *pid) {
    for (int i = 0; i < RBF_HNUM; ++i) {
        for (int j = 0; j < RBF_INPUT_DIM; ++j) {
            pid->c[i][j] = 0.5f * (float)(i + j + 1) / 12.0f;
            pid->ci_1[i][j] = pid->c[i][j];
            pid->ci_2[i][j] = pid->c[i][j];
        }
        pid->b_rbf[i] = 0.8f;
        pid->bi_1[i] = pid->b_rbf[i];
        pid->bi_2[i] = pid->b_rbf[i];
        pid->w[i] = 0.05f * ((float)i - 2.5f) / 2.5f;
        pid->w_1[i] = pid->w[i];
        pid->w_2[i] = pid->w[i];
    }
}

static float rbf_pid_apply_deadband(float error) {
    if (fabsf(error) <= RBF_PID_ERROR_DEADBAND) {
        return 0.0f;
    }

    return error > 0.0f ? error - RBF_PID_ERROR_DEADBAND : error + RBF_PID_ERROR_DEADBAND;
}

static float rbf_pid_effective_flow_scale(const RBF_PID_Handle *pid) {
    return clamp_positive_or_default(pid->flow_normalization_scale,
                                     rbf_pid_max_flow_output(pid));
}

static float rbf_pid_effective_pressure_scale(const RBF_PID_Handle *pid) {
    return clamp_positive_or_default(pid->pressure_normalization_scale,
                                     MAX_PRESSURE);
}

static RBF_PID_ControlState rbf_pid_resolve_control_state(const RBF_PID_Handle *pid,
                                                          float raw_error) {
    float setpoint_scale = clamp_positive_or_default(fabsf(pid->P_set), 1.0f);
    float abs_error_ratio = fabsf(raw_error) / setpoint_scale;

    if (pid->P_set < 0.1f && pid->P_actual < 0.5f) {
        return RBF_PID_CONTROL_STATE_INIT;
    }
    if (raw_error < -0.01f * setpoint_scale) {
        return RBF_PID_CONTROL_STATE_RELIEF;
    }
    if (abs_error_ratio > 0.02f) {
        return RBF_PID_CONTROL_STATE_BOOST;
    }
    return RBF_PID_CONTROL_STATE_HOLD;
}

static void rbf_pid_enforce_control_mode(RBF_PID_Handle *pid) {
    if (pid->control_mode == RBF_PID_CONTROL_MODE_PI) {
        pid->KD = 0.0f;
        pid->eta_d = 0.0f;
        pid->prev_d_term = 0.0f;
        pid->pressure_accel_ff_enabled = false;
    }
}

static void rbf_pid_sanitize_network(RBF_PID_Handle *pid) {
    int i;

    for (i = 0; i < RBF_HNUM; ++i) {
        int j;

        pid->w[i] = clamp_finite(-RBF_PID_WEIGHT_LIMIT, pid->w[i],
                                 RBF_PID_WEIGHT_LIMIT, 0.0f);
        pid->w_1[i] = clamp_finite(-RBF_PID_WEIGHT_LIMIT, pid->w_1[i],
                                   RBF_PID_WEIGHT_LIMIT, pid->w[i]);
        pid->w_2[i] = clamp_finite(-RBF_PID_WEIGHT_LIMIT, pid->w_2[i],
                                   RBF_PID_WEIGHT_LIMIT, pid->w_1[i]);
        pid->b_rbf[i] = clamp_finite(0.2f, pid->b_rbf[i], 5.0f, 0.8f);
        pid->bi_1[i] = clamp_finite(0.2f, pid->bi_1[i], 5.0f, pid->b_rbf[i]);
        pid->bi_2[i] = clamp_finite(0.2f, pid->bi_2[i], 5.0f, pid->bi_1[i]);

        for (j = 0; j < RBF_INPUT_DIM; ++j) {
            pid->c[i][j] = clamp_finite(-2.0f, pid->c[i][j], 2.0f, 0.0f);
            pid->ci_1[i][j] = clamp_finite(-2.0f, pid->ci_1[i][j], 2.0f,
                                           pid->c[i][j]);
            pid->ci_2[i][j] = clamp_finite(-2.0f, pid->ci_2[i][j], 2.0f,
                                           pid->ci_1[i][j]);
        }
    }
}

static void rbf_pid_sanitize_runtime_state(RBF_PID_Handle *pid) {
    float output_min = rbf_pid_output_lower_bound(pid);
    float output_max = rbf_pid_output_upper_bound(pid);
    float history_limit = rbf_pid_max_flow_output(pid);

    pid->u_prev = clamp_finite(output_min, pid->u_prev, output_max, 0.0f);
    pid->du_prev = clamp_finite(-history_limit, pid->du_prev, history_limit, 0.0f);
    pid->e_prev1 = finite_or_default(pid->e_prev1, 0.0f);
    pid->e_prev2 = finite_or_default(pid->e_prev2, 0.0f);
    pid->y_prev1 = finite_or_default(pid->y_prev1, 0.0f);
    pid->y_prev2 = finite_or_default(pid->y_prev2, 0.0f);
    pid->fLastActPress = finite_or_default(pid->fLastActPress, 0.0f);
    pid->fLastActPress2 = finite_or_default(pid->fLastActPress2, 0.0f);
    pid->last_ref = finite_or_default(pid->last_ref, 0.0f);
}

static bool rbf_pid_same_direction_saturation(const RBF_PID_Handle *pid, float error) {
    float output_min;
    float output_max;
    float soft_cap;

    if (!pid->output_saturated) {
        return false;
    }

    output_min = rbf_pid_output_lower_bound(pid);
    soft_cap = rbf_pid_compute_soft_flow_cap(pid);
    if (pid->control_mode == RBF_PID_CONTROL_MODE_PI) {
        output_max = rbf_pid_output_upper_bound(pid);
        if (soft_cap < output_max) {
            output_max = soft_cap;
        }
    } else {
        float hard_limit = rbf_pid_max_flow_output(pid);
        output_max = (soft_cap < hard_limit) ? soft_cap : hard_limit;
    }

    return (pid->Output >= output_max - 1.0e-6f && error > 0.0f) ||
        (pid->Output <= output_min + 1.0e-6f && error < 0.0f);
}

static void rbf_pid_step_rbf_nn(RBF_PID_Handle *pid) {
    float h[RBF_HNUM];
    float flow_scale = rbf_pid_effective_flow_scale(pid);
    float pressure_scale = rbf_pid_effective_pressure_scale(pid);
    float x[RBF_INPUT_DIM] = {
        pid->du_prev / flow_scale,
        pid->y_prev1 / pressure_scale,
        pid->y_prev2 / pressure_scale
    };
    float y_n = pid->P_actual / pressure_scale;
    float y_hat_n = 0.0f;
    float jacobian_n = 0.0f;
    float error_rbf_n;
    int i;

    if (pid->control_mode == RBF_PID_CONTROL_MODE_PI) {
        rbf_pid_sanitize_network(pid);
        for (i = 0; i < RBF_INPUT_DIM; ++i) {
            if (!isfinite(x[i])) {
                x[i] = 0.0f;
            }
        }
    }
    memcpy(pid->last_rbf_input, x, sizeof(x));

    pid->Jacobian = 0.0f;

    for (i = 0; i < RBF_HNUM; ++i) {
        float norm_val = 0.0f;
        int j;

        for (j = 0; j < RBF_INPUT_DIM; ++j) {
            float diff = x[j] - pid->c[i][j];
            norm_val += diff * diff;
        }

        h[i] = expf(-norm_val / (2.0f * pid->b_rbf[i] * pid->b_rbf[i]));
        y_hat_n += pid->w[i] * h[i];
        jacobian_n += pid->w[i] * h[i] * (pid->c[i][0] - x[0]) /
            (pid->b_rbf[i] * pid->b_rbf[i]);
    }

    pid->Jacobian = rbf_pid_clamp_adaptive_value(pid, -5.0f,
        (pressure_scale / flow_scale) * jacobian_n,
        50.0f,
        0.0f);
    error_rbf_n = y_n - y_hat_n;

    if (pid->control_mode == RBF_PID_CONTROL_MODE_PI &&
        rbf_pid_same_direction_saturation(pid, pid->Error)) {
        return;
    }

    for (i = 0; i < RBF_HNUM; ++i) {
        float delta_w = pid->eta_w * error_rbf_n * h[i] +
            pid->alpha * (pid->w[i] - pid->w_1[i]);
        float width = pid->b_rbf[i];
        float width_sq = width * width;
        float width_cu = width_sq * width;
        float norm_val = 0.0f;
        int j;

        if (pid->control_mode == RBF_PID_CONTROL_MODE_PI) {
            pid->w[i] = clamp_finite(-RBF_PID_WEIGHT_LIMIT,
                                     pid->w[i] + delta_w,
                                     RBF_PID_WEIGHT_LIMIT,
                                     pid->w[i]);
        } else {
            pid->w[i] += delta_w;
        }

        for (j = 0; j < RBF_INPUT_DIM; ++j) {
            float delta_center = pid->eta_c * error_rbf_n * pid->w[i] * h[i] *
                (x[j] - pid->c[i][j]) / width_sq +
                pid->alpha * (pid->ci_1[i][j] - pid->ci_2[i][j]);
            pid->c[i][j] = rbf_pid_clamp_adaptive_value(
                pid, -2.0f, pid->c[i][j] + delta_center, 2.0f,
                pid->c[i][j]);
        }

        for (j = 0; j < RBF_INPUT_DIM; ++j) {
            float diff = x[j] - pid->c[i][j];
            norm_val += diff * diff;
        }

        pid->b_rbf[i] = rbf_pid_clamp_adaptive_value(pid, 0.2f,
            pid->b_rbf[i] + pid->eta_b * error_rbf_n * pid->w[i] * h[i] *
            norm_val / width_cu +
            pid->alpha * (pid->bi_1[i] - pid->bi_2[i]),
            5.0f,
            pid->b_rbf[i]);
    }

    for (i = 0; i < RBF_HNUM; ++i) {
        int j;

        for (j = 0; j < RBF_INPUT_DIM; ++j) {
            pid->ci_2[i][j] = pid->ci_1[i][j];
            pid->ci_1[i][j] = pid->c[i][j];
        }

        pid->bi_2[i] = pid->bi_1[i];
        pid->bi_1[i] = pid->b_rbf[i];
        pid->w_2[i] = pid->w_1[i];
        pid->w_1[i] = pid->w[i];
    }
}

static float rbf_pid_compute_soft_flow_cap(const RBF_PID_Handle *pid) {
    float hard_limit = rbf_pid_max_flow_output(pid);

    if (pid->K <= 0.0f || pid->P_set <= 0.0f) {
        return hard_limit;
    }

    return clampf(0.0f, (pid->P_set * RBF_PID_SOFT_CAP_RATIO) / pid->K, hard_limit);
}

static float rbf_pid_target_relative_learning_scale(const RBF_PID_Handle *pid, float raw_error) {
    float setpoint_scale = clamp_positive_or_default(fabsf(pid->P_set), 1.0f);
    float error_ratio = fabsf(raw_error) / setpoint_scale;

    if (error_ratio <= RBF_PID_LEARNING_RATIO_TIGHT) {
        return RBF_PID_LEARNING_SCALE_TIGHT;
    }
    if (error_ratio <= RBF_PID_LEARNING_RATIO_NEAR) {
        return RBF_PID_LEARNING_SCALE_NEAR;
    }
    if (error_ratio <= RBF_PID_LEARNING_RATIO_MID) {
        return RBF_PID_LEARNING_SCALE_MID;
    }
    return 1.0f;
}

static void rbf_pid_step_adaptive_gains(RBF_PID_Handle *pid, float error, float raw_error) {
    float de = error - pid->e_prev1;
    float dde = de - (pid->e_prev1 - pid->e_prev2);
    float learning_scale = rbf_pid_target_relative_learning_scale(pid, raw_error);

    if (rbf_pid_same_direction_saturation(pid, error)) {
        return;
    }

    switch (pid->control_state) {
        case RBF_PID_CONTROL_STATE_INIT:
            learning_scale *= 0.10f;
            break;
        case RBF_PID_CONTROL_STATE_HOLD:
            learning_scale *= 0.25f;
            break;
        case RBF_PID_CONTROL_STATE_RELIEF:
            learning_scale *= 0.50f;
            break;
        case RBF_PID_CONTROL_STATE_BOOST:
        default:
            break;
    }

    pid->KP = rbf_pid_clamp_adaptive_value(pid, pid->min_KP,
        pid->KP + learning_scale * pid->eta_p * error * pid->Jacobian * de,
        pid->max_KP,
        pid->min_KP);
    pid->KI = rbf_pid_clamp_adaptive_value(pid, pid->min_KI,
        pid->KI + learning_scale * pid->eta_i * error * pid->Jacobian * error,
        pid->max_KI,
        pid->min_KI);
    if (pid->control_mode == RBF_PID_CONTROL_MODE_PI) {
        pid->KD = 0.0f;
    } else {
        pid->KD = rbf_pid_clamp_adaptive_value(pid, pid->min_KD,
            pid->KD + learning_scale * pid->eta_d * error * pid->Jacobian * dde,
            pid->max_KD,
            pid->min_KD);
    }
}

static void rbf_pid_step_incremental_output(RBF_PID_Handle *pid, float error, float raw_error) {
    float hard_limit = rbf_pid_max_flow_output(pid);
    float flow_cap = rbf_pid_compute_soft_flow_cap(pid);
    float output_min = rbf_pid_output_lower_bound(pid);
    float output_max = rbf_pid_output_upper_bound(pid);
    float soft_output_max = (flow_cap < hard_limit) ? flow_cap : hard_limit;

    if (soft_output_max > output_max) {
        soft_output_max = output_max;
    }

    float d_term = 0.0f;
    float du;

    if (pid->control_mode == RBF_PID_CONTROL_MODE_PI) {
        pid->prev_d_term = 0.0f;
    } else {
        float raw_d_term = (error - 2.0f * pid->e_prev1 + pid->e_prev2);
        float flt_alpha = HYD_THRESH_RBF_DERIV_FILTER_ALPHA;
        d_term = flt_alpha * raw_d_term + (1.0f - flt_alpha) * pid->prev_d_term;
        pid->prev_d_term = d_term;
    }

    du = pid->KP * (error - pid->e_prev1) + pid->KI * error + pid->KD * d_term;

    float actual_press = pid->P_actual;
    float setpoint_scale = clamp_positive_or_default(fabsf(pid->P_set), 1.0f);
    float pressure_scale = rbf_pid_effective_pressure_scale(pid);
    float actual_press_n = actual_press / pressure_scale;
    float last_press_n = pid->fLastActPress / pressure_scale;
    float last_press2_n = pid->fLastActPress2 / pressure_scale;
    float f_delta_press = actual_press_n - last_press_n;
    float f_dd_press = f_delta_press - (last_press_n - last_press2_n);
    bool near_target = fabsf(raw_error) <= RBF_PID_NEAR_TARGET_RATIO * setpoint_scale;
    bool boost_or_relief = (pid->control_state == RBF_PID_CONTROL_STATE_BOOST) ||
        (pid->control_state == RBF_PID_CONTROL_STATE_RELIEF);
    float f_uff = (pid->pressure_accel_ff_enabled &&
                   boost_or_relief &&
                   !near_target &&
                   fabsf(f_dd_press) > HYD_THRESH_RBF_FF_ACCEL_DEAD_BAND)
        ? (RBF_PID_ACCEL_FF_GAIN * f_dd_press) : 0.0f;

    float ref_change = pid->P_set - pid->last_ref;
    float ref_rate = clampf(-10.0f, ref_change, 10.0f);
    float dynamic_ff = (pid->control_state == RBF_PID_CONTROL_STATE_HOLD)
        ? 0.0f
        : (RBF_PID_DYNAMIC_FF_GAIN * ref_rate);

    du += dynamic_ff + f_uff;

    pid->du = (pid->control_mode == RBF_PID_CONTROL_MODE_PI && !isfinite(du))
        ? 0.0f : du;
    pid->Output = clampf(output_min, pid->u_prev + pid->du, soft_output_max);
    pid->output_saturated = (pid->Output <= output_min + 1.0e-6f) ||
        (pid->Output >= soft_output_max - 1.0e-6f);
    pid->n_out = pid->Output;
    if (pid->P_set < 0.1f && actual_press < 0.5f) {
        pid->Output = 0.0f;
        pid->n_out = 0.0f;
        pid->output_saturated = false;
    }

    pid->fLastActPress2 = pid->fLastActPress;
    pid->fLastActPress = actual_press;
    pid->last_ref = pid->P_set;
}

static void rbf_pid_step_steady_state(RBF_PID_Handle *pid) {
    const float e_steady = 5.0f;
    const float t_steady = 0.2f;
    const float delta_u_steady = 2.0f;
    int n_steady = (int)(t_steady / pid->sampling_period);
    float error = pid->P_set - pid->P_actual;
    bool condition1;
    bool condition2;

    if (n_steady < 5) {
        n_steady = 5;
    }

    condition1 = fabsf(error) <= e_steady;
    condition2 = fabsf(pid->du) <= delta_u_steady;

    if (condition1 && condition2) {
        if (pid->steady_count < n_steady) {
            pid->steady_count++;
        }
    } else {
        pid->steady_count = 0;
    }

    pid->steady_state = condition1 && condition2 &&
        pid->steady_count >= n_steady &&
        fabsf(pid->P_set) > 5.0f;
}

void RBF_PID_Init(RBF_PID_Handle *pid, float sampling_period,
                  float max_flow_lmin, float flow_rate_limit_pct) {
    memset(pid, 0, sizeof(*pid));
    pid->sampling_period = clamp_positive_or_default(sampling_period, 0.001f);
    pid->fMaxFlow = clamp_positive_or_default(max_flow_lmin, 0.0f);
    pid->fFlowRateLimit = clampf(0.0f, flow_rate_limit_pct, 1.0f);
    pid->output_min_flow = MIN_OUTPUT;
    pid->output_max_flow = 0.0f;
    pid->pressure_normalization_scale = 250.0f;
    pid->flow_normalization_scale = (pid->fMaxFlow > 0.0f) ? pid->fMaxFlow : 90.0f;
    pid->output_saturated = false;
    memset(pid->last_rbf_input, 0, sizeof(pid->last_rbf_input));
    pid->Status = 1;
    pid->TuneResult = 66;
    pid->alpha = 0.05f;
    pid->flowToPumpSpeedGain = 20.0f;
    pid->pressure_accel_ff_enabled = true;
    pid->control_mode = RBF_PID_CONTROL_MODE_PID;
    rbf_pid_apply_default_limits(pid);
    rbf_pid_apply_default_learning_rates(pid);
    rbf_pid_apply_default_gains(pid);
    pid->pid_mode_kd = pid->KD;
    pid->pid_mode_eta_d = pid->eta_d;
    pid->pressure_accel_ff_requested = true;
    rbf_pid_refresh_gain_compensation(pid);
    rbf_pid_init_network(pid);
}

float RBF_PID_Update(RBF_PID_Handle *pid, float setpoint, float feedback) {
    float raw_error;
    float error;

    if (pid->control_mode == RBF_PID_CONTROL_MODE_PI) {
        pid->P_set = isfinite(setpoint) ? setpoint : 0.0f;
        pid->P_actual = isfinite(feedback) ? feedback : 0.0f;
        rbf_pid_sanitize_runtime_state(pid);
    } else {
        pid->P_set = setpoint;
        pid->P_actual = feedback;
    }
    rbf_pid_enforce_control_mode(pid);
    raw_error = pid->P_set - pid->P_actual;
    error = rbf_pid_apply_deadband(raw_error);
    pid->Error = error;
    pid->control_state = rbf_pid_resolve_control_state(pid, raw_error);

    rbf_pid_step_rbf_nn(pid);
    rbf_pid_step_adaptive_gains(pid, error, raw_error);
    rbf_pid_step_incremental_output(pid, error, raw_error);

    pid->y_prev2 = pid->y_prev1;
    pid->y_prev1 = pid->P_actual;
    pid->e_prev2 = pid->e_prev1;
    pid->e_prev1 = error;
    pid->u_prev = pid->Output;
    pid->du_prev = pid->du;

    rbf_pid_step_steady_state(pid);
    pid->Status = pid->steady_state ? 3 : 2;

    return pid->Output;
}

void RBF_PID_Reset(RBF_PID_Handle *pid) {
    float sampling_period = pid->sampling_period;
    float max_flow = pid->fMaxFlow;
    float flow_limit = pid->fFlowRateLimit;
    RBF_PID_Init(pid, sampling_period, max_flow, flow_limit);
}

void RBF_PID_SetParamLimits(RBF_PID_Handle *pid,
    float min_kp, float max_kp, float min_ki, float max_ki,
    float min_kd, float max_kd) {
    pid->min_KP = isfinite(min_kp) ? min_kp : PID_MIN_KP;
    pid->max_KP = isfinite(max_kp) ? max_kp : PID_MAX_KP;
    pid->min_KI = isfinite(min_ki) ? min_ki : PID_MIN_KI;
    pid->max_KI = isfinite(max_ki) ? max_ki : PID_MAX_KI;
    pid->min_KD = isfinite(min_kd) ? min_kd : PID_MIN_KD;
    pid->max_KD = isfinite(max_kd) ? max_kd : PID_MAX_KD;
    sort_pair(&pid->min_KP, &pid->max_KP);
    sort_pair(&pid->min_KI, &pid->max_KI);
    sort_pair(&pid->min_KD, &pid->max_KD);
    pid->pid_mode_kd = clampf(pid->min_KD, pid->pid_mode_kd, pid->max_KD);
}

void RBF_PID_SetLearningRates(RBF_PID_Handle *pid,
    float eta_w, float eta_c, float eta_b,
    float eta_p, float eta_i, float eta_d) {
    pid->eta_w = clamp_finite(0.0f, eta_w, 10.0f, 0.0f);
    pid->eta_c = clamp_finite(0.0f, eta_c, 10.0f, 0.0f);
    pid->eta_b = clamp_finite(0.0f, eta_b, 10.0f, 0.0f);
    pid->eta_p = clamp_finite(0.0f, eta_p, 10.0f, 0.0f);
    pid->eta_i = clamp_finite(0.0f, eta_i, 10.0f, 0.0f);
    pid->pid_mode_eta_d = clamp_finite(0.0f, eta_d, 10.0f, 0.0f);
    pid->eta_d = pid->pid_mode_eta_d;
    rbf_pid_enforce_control_mode(pid);
}

void RBF_PID_SetPressureNormalization(RBF_PID_Handle *pid, float scale) {
    pid->pressure_normalization_scale = scale > 0.0f ? scale : MAX_PRESSURE;
    rbf_pid_refresh_gain_compensation(pid);
}

void RBF_PID_SetFlowNormalization(RBF_PID_Handle *pid, float scale) {
    if (pid == NULL) {
        return;
    }

    pid->flow_normalization_scale = clamp_positive_or_default(
        scale,
        (pid->fMaxFlow > 0.0f) ? pid->fMaxFlow : 90.0f);
}

void RBF_PID_SetGainCompensation(RBF_PID_Handle *pid, float systemGain) {
    if (pid == NULL) {
        return;
    }

    pid->K = (systemGain > 0.0f) ? systemGain : 0.0f;
    rbf_pid_refresh_gain_compensation(pid);
}

void RBF_PID_SetPressureAccelFeedforwardEnabled(RBF_PID_Handle *pid, bool enabled) {
    if (pid == NULL) {
        return;
    }

    pid->pressure_accel_ff_requested = enabled;
    pid->pressure_accel_ff_enabled =
        (pid->control_mode == RBF_PID_CONTROL_MODE_PID) &&
        pid->pressure_accel_ff_requested;
}

void RBF_PID_SetControlMode(RBF_PID_Handle *pid, RBF_PID_ControlMode mode) {
    RBF_PID_ControlMode requestedMode;

    if (pid == NULL) {
        return;
    }

    requestedMode = (mode == RBF_PID_CONTROL_MODE_PI)
        ? RBF_PID_CONTROL_MODE_PI
        : RBF_PID_CONTROL_MODE_PID;

    if (pid->control_mode == requestedMode) {
        rbf_pid_enforce_control_mode(pid);
        return;
    }

    if (pid->control_mode == RBF_PID_CONTROL_MODE_PID &&
        requestedMode == RBF_PID_CONTROL_MODE_PI) {
        pid->pid_mode_kd = pid->KD;
        pid->pid_mode_eta_d = pid->eta_d;
        pid->pressure_accel_ff_requested = pid->pressure_accel_ff_enabled;
    }

    pid->control_mode = requestedMode;
    if (requestedMode == RBF_PID_CONTROL_MODE_PID) {
        pid->KD = clampf(pid->min_KD, pid->pid_mode_kd, pid->max_KD);
        pid->eta_d = pid->pid_mode_eta_d;
        pid->pressure_accel_ff_enabled = pid->pressure_accel_ff_requested;
        return;
    }
    rbf_pid_enforce_control_mode(pid);
}

void RBF_PID_SetSeed(RBF_PID_Handle *pid, uint32_t seed) {
    pid->network_seed = seed;
}
