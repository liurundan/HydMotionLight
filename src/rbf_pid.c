#include "rbf_pid.h"

#include <math.h>
#include <string.h>

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
    pid->eta_w = 0.02f;
    pid->eta_c = 0.02f;
    pid->eta_b = 0.02f;
    pid->eta_p = 0.1f;
    pid->eta_i = 0.1f;
    pid->eta_d = 0.1f;
}

static void rbf_pid_apply_default_gains(RBF_PID_Handle *pid) {
    pid->KP = 1.02f;
    pid->KI = 0.02f;
    pid->KD = 1.02f;
}

static void rbf_pid_refresh_gain_compensation(RBF_PID_Handle *pid) {
    if (pid->K > 0.0f && pid->fMaxFlow > 0.0f) {
        pid->gain_compensation_enabled = true;
        pid->gain_compensation_factor =
            pid->pressure_normalization_scale / (pid->K * pid->fMaxFlow);
    } else {
        pid->gain_compensation_enabled = false;
        pid->gain_compensation_factor = 1.0f;
    }

    pid->fGainCompensation = pid->gain_compensation_factor;
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
    const float deadband = 0.05f;

    if (fabsf(error) <= deadband) {
        return 0.0f;
    }

    return error > 0.0f ? error - deadband : error + deadband;
}

static void rbf_pid_step_rbf_nn(RBF_PID_Handle *pid, float error) {
    float h[RBF_HNUM];
    float x[RBF_INPUT_DIM] = {
        pid->du_prev / pid->pressure_normalization_scale,
        pid->P_actual / pid->pressure_normalization_scale,
        pid->y_prev1 / pid->pressure_normalization_scale,
        error / pid->pressure_normalization_scale
    };
    float error_rbf;
    float y_hat = 0.0f;

    pid->Jacobian = 0.0f;

    for (int i = 0; i < RBF_HNUM; ++i) {
        float norm_val = 0.0f;
        for (int j = 0; j < RBF_INPUT_DIM; ++j) {
            float diff = x[j] - pid->c[i][j];
            norm_val += diff * diff;
        }

        h[i] = expf(-norm_val / (2.0f * pid->b_rbf[i] * pid->b_rbf[i]));
        y_hat += pid->w[i] * h[i];
        pid->Jacobian += pid->w[i] * h[i] * (pid->c[i][0] - x[0]) /
            (pid->b_rbf[i] * pid->b_rbf[i]);
    }

    pid->Jacobian = clampf(-5.0f, pid->Jacobian / pid->pressure_normalization_scale, 50.0f);

    error_rbf = pid->P_actual - y_hat;

    for (int i = 0; i < RBF_HNUM; ++i) {
        float delta_w = pid->eta_w * error_rbf * h[i] +
            pid->alpha * (pid->w[i] - pid->w_1[i]);
        pid->w[i] += delta_w;
    }

    for (int i = 0; i < RBF_HNUM; ++i) {
        float width = pid->b_rbf[i];
        float width_sq = width * width;
        float width_cu = width_sq * width;
        float norm_val = 0.0f;

        for (int j = 0; j < RBF_INPUT_DIM; ++j) {
            float delta_center = pid->eta_c * error_rbf * pid->w[i] * h[i] *
                (x[j] - pid->c[i][j]) / width_sq +
                pid->alpha * (pid->ci_1[i][j] - pid->ci_2[i][j]);
            pid->c[i][j] = clampf(-20.0f, pid->c[i][j] + delta_center, 30.0f);
        }

        for (int j = 0; j < RBF_INPUT_DIM; ++j) {
            float diff = x[j] - pid->c[i][j];
            norm_val += diff * diff;
        }

        pid->b_rbf[i] = clampf(5.0f,
            pid->b_rbf[i] + pid->eta_b * error_rbf * pid->w[i] * h[i] *
            norm_val / width_cu +
            pid->alpha * (pid->bi_1[i] - pid->bi_2[i]),
            50.5f);
    }

    for (int i = 0; i < RBF_HNUM; ++i) {
        for (int j = 0; j < RBF_INPUT_DIM; ++j) {
            pid->ci_2[i][j] = pid->ci_1[i][j];
            pid->ci_1[i][j] = pid->c[i][j];
        }

        pid->bi_2[i] = pid->bi_1[i];
        pid->bi_1[i] = pid->b_rbf[i];
        pid->w_2[i] = pid->w_1[i];
        pid->w_1[i] = pid->w[i];
    }
}

static void rbf_pid_step_adaptive_gains(RBF_PID_Handle *pid, float error) {
    float de = error - pid->e_prev1;
    float dde = de - (pid->e_prev1 - pid->e_prev2);

    pid->KP = clampf(pid->min_KP,
        pid->KP + pid->eta_p * error * pid->Jacobian * de,
        pid->max_KP);
    pid->KI = clampf(pid->min_KI,
        pid->KI + pid->eta_i * error * pid->Jacobian * error,
        pid->max_KI);
    pid->KD = clampf(pid->min_KD,
        pid->KD + pid->eta_d * error * pid->Jacobian * dde,
        pid->max_KD);
}

static void rbf_pid_step_incremental_output(RBF_PID_Handle *pid, float error) {
    float raw_d_term = error - 2.0f * pid->e_prev1 + pid->e_prev2;
    float du = pid->KP * (error - pid->e_prev1) + pid->KI * error + pid->KD * raw_d_term;
    float actual_press = pid->P_set - error;
    float f_delta_press = actual_press - pid->fLastActPress;
    float f_dd_press = f_delta_press - (pid->fLastActPress - pid->fLastActPress2);
    float f_uff = -0.5f * f_dd_press;
    float ref_change = pid->P_set - pid->last_ref;
    float ref_rate = clampf(-10.0f, ref_change, 10.0f);
    float dynamic_ff = 0.02f * ref_rate;

    du += dynamic_ff + f_uff;

    pid->du = du;
    pid->Output = clampf(MIN_OUTPUT, pid->u_prev + du, 500.0f);
    if (pid->P_set < 0.1f && actual_press < 0.5f) {
        pid->Output = 0.0f;
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
    pid->pressure_normalization_scale = 250.0f;
    pid->Status = 1;
    pid->TuneResult = 66;
    pid->alpha = 0.05f;
    pid->flowToPumpSpeedGain = 20.0f;
    rbf_pid_apply_default_limits(pid);
    rbf_pid_apply_default_learning_rates(pid);
    rbf_pid_apply_default_gains(pid);
    rbf_pid_refresh_gain_compensation(pid);
    rbf_pid_init_network(pid);
}

float RBF_PID_Update(RBF_PID_Handle *pid, float setpoint, float feedback) {
    float error;
    float raw_output;

    pid->P_set = setpoint;
    pid->P_actual = feedback;
    error = rbf_pid_apply_deadband(setpoint - feedback);
    pid->Error = error;

    rbf_pid_step_rbf_nn(pid, error);
    rbf_pid_step_adaptive_gains(pid, error);
    rbf_pid_step_incremental_output(pid, error);
    raw_output = pid->Output;

    if (pid->gain_compensation_enabled) {
        pid->Output = raw_output * pid->gain_compensation_factor;
    }
    pid->Output = clampf(MIN_OUTPUT, pid->Output, 500.0f);
    pid->n_out = pid->Output;
    pid->y_prev1 = pid->P_actual;
    pid->e_prev2 = pid->e_prev1;
    pid->e_prev1 = error;
    pid->u_prev = raw_output;
    pid->du_prev = pid->du;

    rbf_pid_step_steady_state(pid);
    pid->Status = pid->steady_state ? 3 : 2;
    pid->Output = pid->Output / pid->flowToPumpSpeedGain; // motor speed transfer to flow request
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
    pid->min_KP = min_kp;
    pid->max_KP = max_kp;
    pid->min_KI = min_ki;
    pid->max_KI = max_ki;
    pid->min_KD = min_kd;
    pid->max_KD = max_kd;
    sort_pair(&pid->min_KP, &pid->max_KP);
    sort_pair(&pid->min_KI, &pid->max_KI);
    sort_pair(&pid->min_KD, &pid->max_KD);
}

void RBF_PID_SetLearningRates(RBF_PID_Handle *pid,
    float eta_w, float eta_c, float eta_b,
    float eta_p, float eta_i, float eta_d) {
    pid->eta_w = clampf(0.0f, eta_w, 10.0f);
    pid->eta_c = clampf(0.0f, eta_c, 10.0f);
    pid->eta_b = clampf(0.0f, eta_b, 10.0f);
    pid->eta_p = clampf(0.0f, eta_p, 10.0f);
    pid->eta_i = clampf(0.0f, eta_i, 10.0f);
    pid->eta_d = clampf(0.0f, eta_d, 10.0f);
}

void RBF_PID_SetPressureNormalization(RBF_PID_Handle *pid, float scale) {
    pid->pressure_normalization_scale = scale > 0.0f ? scale : MAX_PRESSURE;
    rbf_pid_refresh_gain_compensation(pid);
}

void RBF_PID_SetGainCompensation(RBF_PID_Handle *pid, float systemGain) {
    pid->K = systemGain;
    rbf_pid_refresh_gain_compensation(pid);
}

void RBF_PID_SetSeed(RBF_PID_Handle *pid, uint32_t seed) {
    pid->network_seed = seed;
}
