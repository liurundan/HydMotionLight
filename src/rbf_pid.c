#include "rbf_pid.h"
#include "hyd_config.h"

#include <math.h>
#include <string.h>

#define EPS 1e-6f

static const float RBF_PID_ERROR_DEADBAND = 0.005f;
static const float RBF_PID_SOFT_CAP_RATIO = 1.05f;
static const float RBF_PID_DYNAMIC_FF_GAIN = 0.001f;
/* The legacy implementation used -0.15 * delta_pressure per sample.  The
 * rate form below is the same nominal damping at the fixed 1 ms scan while
 * making the units explicit: [L/min] / [bar/s]. */
static const float RBF_PID_PRESSURE_RATE_DAMPING_GAIN = 0.00015f;
static const float RBF_PID_PRESSURE_RATE_DAMPING_LIMIT = 0.5f;
/* Minimum measured pressure speed that is allowed to move the output.  This
 * must exceed the filtered sensor/ADC noise expressed in bar/s. */
static const float RBF_PID_PRESSURE_RATE_DEADBAND = 0.5f;
static const float RBF_PID_WEIGHT_LIMIT = 5.0f;

/* Steady-state criteria for a 0-250 bar pressure loop.  The error and rate
 * limits scale with target pressure but retain a noise floor. */
static const float RBF_PID_STEADY_ERROR_MIN_BAR = 0.5f;
static const float RBF_PID_STEADY_ERROR_RATIO = 0.01f;
static const float RBF_PID_STEADY_RATE_MIN_BAR_S = 0.25f;
static const float RBF_PID_STEADY_RATE_RATIO = 0.005f;
static const float RBF_PID_STEADY_TIME_S = 0.2f;
/* Learning freeze is intentionally looser than the reported steady-state
 * flag.  It prevents parameter drift near a target without claiming that the
 * physical pressure has fully settled. */
static const float RBF_PID_LEARNING_ERROR_MIN_BAR = 1.0f;
static const float RBF_PID_LEARNING_ERROR_RATIO = 0.06f;
static const float RBF_PID_LEARNING_ERROR_MAX_BAR = 5.0f;
static const float RBF_PID_LEARNING_DU_LIMIT = 2.0f;

static float rbf_pid_max_flow_output(const RBF_PID_Handle *pid);
static float rbf_pid_compute_soft_flow_cap(const RBF_PID_Handle *pid);

static float sign(float x)
{
    if (x > EPS)
        return 1.0f;
    else if (x < -EPS)
        return -1.0f;
    else
        return 0.0f;
}

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

static float rbf_pid_effective_sampling_period(const RBF_PID_Handle *pid) {
    if (pid != NULL && isfinite(pid->sampling_period) &&
        pid->sampling_period > 1.0e-6f) {
        return pid->sampling_period;
    }
    return RBF_PID_FIXED_SAMPLING_PERIOD;
}

static float rbf_pid_pressure_rate(const RBF_PID_Handle *pid) {
    /* The outer pressure controller supplies a filtered pressure signal.  A
     * local rate deadband below removes the remaining quantization chatter. */
    float dt = rbf_pid_effective_sampling_period(pid);
    float current_delta = pid->P_actual - pid->fLastActPress;
    float rate = current_delta / dt;

    return isfinite(rate) ? rate : 0.0f;
}

static float rbf_pid_setpoint_rate(const RBF_PID_Handle *pid) {
    float dt = rbf_pid_effective_sampling_period(pid);
    float rate = (pid->P_set - pid->last_ref) / dt;

    return isfinite(rate) ? rate : 0.0f;
}

static float rbf_pid_steady_error_limit(const RBF_PID_Handle *pid) {
    float target = fabsf(pid->P_set);
    float limit = RBF_PID_STEADY_ERROR_RATIO * target;

    return (limit > RBF_PID_STEADY_ERROR_MIN_BAR)
        ? limit : RBF_PID_STEADY_ERROR_MIN_BAR;
}

static float rbf_pid_steady_rate_limit(const RBF_PID_Handle *pid) {
    float target = fabsf(pid->P_set);
    float limit = RBF_PID_STEADY_RATE_RATIO * target;

    return (limit > RBF_PID_STEADY_RATE_MIN_BAR_S)
        ? limit : RBF_PID_STEADY_RATE_MIN_BAR_S;
}

static float rbf_pid_pressure_rate_damping_limit(const RBF_PID_Handle *pid) {
    /* Scale the per-scan flow increment with the available flow range so a
     * 30 L/min and a 120 L/min machine do not receive the same absolute kick. */
    return clampf(0.1f,
                  0.005f * rbf_pid_max_flow_output(pid),
                  RBF_PID_PRESSURE_RATE_DAMPING_LIMIT);
}

static float rbf_pid_learning_error_limit(const RBF_PID_Handle *pid) {
    float target = fabsf(pid->P_set);
    float limit = RBF_PID_LEARNING_ERROR_RATIO * target;

    limit = (limit > RBF_PID_LEARNING_ERROR_MIN_BAR)
        ? limit : RBF_PID_LEARNING_ERROR_MIN_BAR;
    return clampf(RBF_PID_LEARNING_ERROR_MIN_BAR,
                  limit,
                  RBF_PID_LEARNING_ERROR_MAX_BAR);
}

static bool rbf_pid_learning_freeze_candidate(const RBF_PID_Handle *pid,
                                               float error) {
    float rate_limit = rbf_pid_steady_rate_limit(pid);
    float setpoint_rate = rbf_pid_setpoint_rate(pid);

    /* PI mode has no pressure-rate damping path.  Preserve its established
     * conservative adaptation hold so this optimization does not change the
     * plant behavior of the derivative-free controller. */
    if (pid->control_mode == RBF_PID_CONTROL_MODE_PI) {
        float de = error - pid->e_prev1;
        return fabsf(error) < 10.0f && fabsf(de) < 10.0f;
    }

    return fabsf(error) <= rbf_pid_learning_error_limit(pid) &&
        fabsf(pid->du) <= RBF_PID_LEARNING_DU_LIMIT &&
        fabsf(setpoint_rate) <= rate_limit &&
        !pid->output_saturated;
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
    pid->KP = PID_MIN_KP;
    pid->KI = PID_MIN_KI;
    pid->KD = PID_MIN_KD;
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

static int rbf_pid_step_rbf_nn(RBF_PID_Handle *pid,float error) {
    float h[RBF_HNUM];
    float pressure_scale = rbf_pid_effective_pressure_scale(pid);
    float flow_scale = clamp_positive_or_default(pid->flow_normalization_scale,
                                                 pid->fMaxFlow > 0.0f ? pid->fMaxFlow : 90.0f);
    float x[RBF_INPUT_DIM] = {
        pid->u_prev / flow_scale,
        pid->y_prev1 / pressure_scale,
        (pid->y_prev1 - pid->y_prev2) / pressure_scale
    };
    float y_n = pid->P_actual / pressure_scale;
    float y_hat_n = 0.0f;
    float jacobian_n = 0.0f;
    float error_rbf_n;
    int freeze_learning = rbf_pid_learning_freeze_candidate(pid, error);
    int i;

    /* P0-1修复：统一数值防护，模式无关 */
    rbf_pid_sanitize_network(pid);
    for (i = 0; i < RBF_INPUT_DIM; ++i) {
        if (!isfinite(x[i])) {
            x[i] = 0.0f;
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

    pid->Jacobian = rbf_pid_clamp_adaptive_value(pid, -500.0f,
        (pressure_scale / flow_scale) * jacobian_n,
        500.0f,
        0.0f);
    if (pid->ksys_valid) {
        pid->Jacobian = clampf(0.5f * pid->K,
            pid->Jacobian, 2.0f * pid->K);
    }

	if (!freeze_learning) {
		error_rbf_n = y_n - y_hat_n;

		/* P0-2修复：权重饱和抑制应模式无关 */
        bool skip_learning = rbf_pid_same_direction_saturation(pid, pid->Error);

		if (!skip_learning) {
			for (i = 0; i < RBF_HNUM; ++i) {
				float w_old = pid->w[i];
				float delta_w = pid->eta_w * error_rbf_n * h[i]
						+ pid->alpha * (pid->w[i] - pid->w_1[i]);
				float width = pid->b_rbf[i];
				float width_sq = width * width;
				float width_cu = width_sq * width;
				int j;

				/* Step 1: Compute norm_val BEFORE updating c[i][j] */
				float norm_val = 0.0f;
				for (j = 0; j < RBF_INPUT_DIM; ++j) {
					float diff = x[j] - pid->c[i][j];
					norm_val += diff * diff;
				}

				/* Step 2: Update c[i][j] using w_old */
				for (j = 0; j < RBF_INPUT_DIM; ++j) {
					float delta_center = pid->eta_c * error_rbf_n * w_old * h[i]
							* (x[j] - pid->c[i][j]) / width_sq
							+ pid->alpha * (pid->ci_1[i][j] - pid->ci_2[i][j]);
					pid->c[i][j] = rbf_pid_clamp_adaptive_value(pid, -2.0f,
							pid->c[i][j] + delta_center, 2.0f, pid->c[i][j]);
				}

				/* Step 3: Update b_rbf[i] using norm_val computed with OLD centers */
				pid->b_rbf[i] = rbf_pid_clamp_adaptive_value(pid, 0.2f,
						pid->b_rbf[i]
								+ pid->eta_b * error_rbf_n * w_old * h[i]
										* norm_val / width_cu
								+ pid->alpha * (pid->bi_1[i] - pid->bi_2[i]), 5.0f,
						pid->b_rbf[i]);

				/* Step 4: Update w[i] with WEIGHT_LIMIT clamping (P0-2修复：统一钳位) */
				pid->w[i] = clamp_finite(-RBF_PID_WEIGHT_LIMIT,
						pid->w[i] + delta_w, RBF_PID_WEIGHT_LIMIT, pid->w[i]);
			}
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

    return freeze_learning;
}

static float rbf_pid_compute_soft_flow_cap(const RBF_PID_Handle *pid) {
    float hard_limit = rbf_pid_max_flow_output(pid);

    if (pid->P_set <= 0.0f) {
        return hard_limit;
    }

    if (pid->ksys_valid && pid->control_state != RBF_PID_CONTROL_STATE_HOLD) {
        return hard_limit;
    }

    if (!pid->ksys_valid && pid->K <= 0.0f) {
        return hard_limit;
    }

    {
        float process_gain = pid->K;
        if (process_gain <= 0.0f) {
            return hard_limit;
        }
        return clampf(0.0f,
            (pid->P_set * RBF_PID_SOFT_CAP_RATIO) / process_gain,
            hard_limit);
    }
}

#define ETA_KD_BOOST 0.5f // 微分强制唤醒系数
#define LAMBDA_KI    0.0005f // 积分惩罚系数
#define KI_CENTER    0.0f   // 积分中心值

static void rbf_pid_step_adaptive_gains(RBF_PID_Handle *pid, float error, float raw_error)
{
	float de  = error - pid->e_prev1;
	// ---------- 5. PID 参数在线整定（带抗饱和 & 微分唤醒） ----------
	float abs_Jac = fabsf(pid->Jacobian);
	if (abs_Jac < 1e-6f) {
		abs_Jac = 1e-6f;   // 避免除零
	}
	// 5.3 比例增益 Kp 更新（常规梯度）
	//    公式：ΔKp = ηp * e * Jac * Δe
    float grad_Kp = pid->eta_p * error * sign(pid->Jacobian) * abs_Jac * de;
    float kp_step_limit = 0.01f * (pid->max_KP - pid->min_KP);
    float ki_step_limit = 0.01f * (pid->max_KI - pid->min_KI);
    pid->KP += clampf(-kp_step_limit, grad_Kp, kp_step_limit);
    pid->KP = clampf(pid->min_KP, pid->KP, pid->max_KP);

	// 5.1 积分增益 Ki 更新（带L2惩罚，防止积分饱和）
    float grad_Ki = pid->eta_i * error * sign(pid->Jacobian) * abs_Jac * error;
    float decay_Ki = LAMBDA_KI * (pid->KI - KI_CENTER);
    float delta_Ki = clampf(-ki_step_limit, grad_Ki - decay_Ki, ki_step_limit);
	pid->KI += delta_Ki;
	pid->KI = clampf(pid->min_KI, pid->KI, pid->max_KI);

	if (pid->control_mode == RBF_PID_CONTROL_MODE_PI) {
		pid->KD = 0.0f;
    } else {
        /* Keep D fixed in the first industrial rollout; measured pressure is
         * already filtered upstream and a second-difference learner is noisy. */
        pid->KD = clampf(pid->min_KD, pid->pid_mode_kd, pid->max_KD);
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
        //float flt_alpha = HYD_THRESH_RBF_DERIV_FILTER_ALPHA;
        //d_term = flt_alpha * raw_d_term + (1.0f - flt_alpha) * pid->prev_d_term;
        d_term = raw_d_term;
        pid->prev_d_term = d_term;
    }
    float interf_term = pid->KI * error;
    interf_term = clampf(-0.08, interf_term, 0.08);
    du = pid->KP * (error - pid->e_prev1) + interf_term + pid->KD * d_term;

    float actual_press = pid->P_actual;
    float pressure_rate = rbf_pid_pressure_rate(pid);

    float f_velfb = 0.0f;
    if (pid->pressure_accel_ff_enabled && !pid->steady_state &&
        fabsf(pressure_rate) > RBF_PID_PRESSURE_RATE_DEADBAND) {
        float damping_limit = rbf_pid_pressure_rate_damping_limit(pid);
        f_velfb = clampf(-damping_limit,
            -RBF_PID_PRESSURE_RATE_DAMPING_GAIN * pressure_rate,
            damping_limit);
    }

    float vel_ref  = pid->P_set - pid->last_ref;
    vel_ref  = clampf( -10.0f, vel_ref, 10.0f );
    float f_du_ff  = RBF_PID_DYNAMIC_FF_GAIN * ( vel_ref -  pid->v_ref_k1 );

   // du = clampf( -0.5, du, 0.5 );
//    printf("output_min: %.3f, output_max: %.3f,  kp:%.3f,k:%.3f,kd:%.3f,du:%.6f\n", output_min, output_max,
//    		pid->KP, pid->KI, pid->KD, du);

    pid->du = !isfinite(du) ? 0.0f : du;

    pid->Output = pid->u_prev + pid->du + f_du_ff + f_velfb +
        (pid->feedforward_flow - pid->feedforward_flow_prev);

    pid->Output =  clampf( output_min, pid->Output, soft_output_max );
    pid->output_saturated = (pid->Output <= output_min + 1.0e-6f) || (pid->Output >= soft_output_max - 1.0e-6f);
    if (pid->P_set < 0.1f && actual_press < 0.5f) {
        pid->Output = 0.0f;
        pid->output_saturated = false;
    }

    pid->fLastActPress2 = pid->fLastActPress;
    pid->fLastActPress = actual_press;
    pid->last_ref = pid->P_set;
    pid->v_ref_k1 = vel_ref;
    pid->feedforward_flow_prev = pid->feedforward_flow;
}

static void rbf_pid_step_steady_state(RBF_PID_Handle *pid) {
    float error_limit = rbf_pid_steady_error_limit(pid);
    float rate_limit = rbf_pid_steady_rate_limit(pid);
    float dt = rbf_pid_effective_sampling_period(pid);
    float pressure_rate = rbf_pid_pressure_rate(pid);
    float setpoint_rate = rbf_pid_setpoint_rate(pid);
    int n_steady = (int)ceilf(RBF_PID_STEADY_TIME_S / dt);
    float error = pid->P_set - pid->P_actual;
    bool condition_error;
    bool condition_pressure_rate;
    bool condition_setpoint_rate;
    bool condition_not_saturated;
    bool candidate;

    if (n_steady < 5) {
        n_steady = 5;
    }

    condition_error = fabsf(error) <= error_limit;
    condition_pressure_rate = fabsf(pressure_rate) <= rate_limit;
    condition_setpoint_rate = fabsf(setpoint_rate) <= rate_limit;
    condition_not_saturated = !pid->output_saturated;
    candidate = condition_error && condition_pressure_rate &&
        condition_setpoint_rate && condition_not_saturated;

    if (candidate) {
        if (pid->steady_count < n_steady) {
            pid->steady_count++;
        }
    } else {
        pid->steady_count = 0;
    }

    pid->steady_state = candidate && pid->steady_count >= n_steady;
}

void RBF_PID_Init(RBF_PID_Handle *pid, float sampling_period,
                  float max_flow_lmin, float flow_rate_limit_pct) {
    memset(pid, 0, sizeof(*pid));
    (void)sampling_period;
    pid->sampling_period = RBF_PID_FIXED_SAMPLING_PERIOD;
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
    pid->f_dd_press_prev = 5.0f;
    pid->pressure_accel_ff_enabled = true;
    pid->control_mode = RBF_PID_CONTROL_MODE_PID;
    rbf_pid_apply_default_limits(pid);
    rbf_pid_apply_default_learning_rates(pid);
    rbf_pid_apply_default_gains(pid);
    pid->pid_mode_kd = pid->KD;
    pid->pid_mode_eta_d = pid->eta_d;
    pid->pressure_accel_ff_requested = true;
    pid->ksys_valid = false;
    rbf_pid_refresh_gain_compensation(pid);
    rbf_pid_init_network(pid);
}

float RBF_PID_Update(RBF_PID_Handle *pid, float setpoint, float feedback) {
    float raw_error;
    float error;

    pid->P_set = isfinite(setpoint) ? setpoint : 0.0f;
    pid->P_actual = isfinite(feedback) ? feedback : 0.0f;

    /* P0-1修复：统一数值防护，移到enforce之前，模式无关 */
    rbf_pid_sanitize_runtime_state(pid);

    rbf_pid_enforce_control_mode(pid);
    raw_error = pid->P_set - pid->P_actual;
    error = rbf_pid_apply_deadband(raw_error);
    pid->Error = error;
    pid->control_state = rbf_pid_resolve_control_state(pid, raw_error);

    rbf_pid_step_steady_state(pid);

    int freeze_learning = rbf_pid_step_rbf_nn(pid,error);

    rbf_pid_step_incremental_output(pid, error, raw_error);

    pid->y_prev2 = pid->y_prev1;
    pid->y_prev1 = pid->P_actual;
    pid->u_prev = pid->Output;
    pid->du_prev = pid->du;

    if (!freeze_learning) {
    	rbf_pid_step_adaptive_gains(pid, error, raw_error);
    }

    pid->e_prev2 = pid->e_prev1;
    pid->e_prev1 = error;

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
    pid->KP = clampf(pid->min_KP, pid->KP, pid->max_KP);
    pid->KI = clampf(pid->min_KI, pid->KI, pid->max_KI);
    pid->KD = clampf(pid->min_KD, pid->KD, pid->max_KD);
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

void RBF_PID_SetDuNormalization(RBF_PID_Handle *pid, float scale) {
    if (pid == NULL) {
        return;
    }

    pid->f_dd_press_prev = clamp_positive_or_default(scale, 5.0f);
}

void RBF_PID_SetGainCompensation(RBF_PID_Handle *pid, float systemGain) {
    if (pid == NULL) {
        return;
    }

    pid->K = (systemGain > 0.0f) ? systemGain : 0.0f;
    rbf_pid_refresh_gain_compensation(pid);
}

void RBF_PID_SetKsysBarPerRpm(RBF_PID_Handle *pid,
                              float ksys_bar_per_rpm,
                              float flow_to_pump_speed_gain) {
    if (pid == NULL) {
        return;
    }

    pid->ksys_valid = false;
    if (isfinite(ksys_bar_per_rpm) && isfinite(flow_to_pump_speed_gain) &&
        ksys_bar_per_rpm > 0.0f && flow_to_pump_speed_gain > 0.0f) {
        float process_gain = ksys_bar_per_rpm * flow_to_pump_speed_gain;
        if (isfinite(process_gain) && process_gain > 0.0f) {
            pid->K = process_gain;
            pid->ksys_valid = true;
        }
    }
}

void RBF_PID_SetFeedforwardFlow(RBF_PID_Handle *pid, float feedforward_flow) {
    if (pid == NULL) {
        return;
    }
    pid->feedforward_flow = isfinite(feedforward_flow) ? feedforward_flow : 0.0f;
}

void RBF_PID_TrackOutput(RBF_PID_Handle *pid,
                         float output_flow,
                         float setpoint,
                         float feedback) {
    if (pid == NULL) {
        return;
    }
    pid->Output = finite_or_default(output_flow, 0.0f);
    pid->u_prev = pid->Output;
    pid->P_set = finite_or_default(setpoint, 0.0f);
    pid->P_actual = finite_or_default(feedback, 0.0f);
    pid->Error = pid->P_set - pid->P_actual;
    pid->e_prev1 = pid->Error;
    pid->e_prev2 = pid->Error;
    pid->y_prev1 = pid->P_actual;
    pid->y_prev2 = pid->P_actual;
    pid->du = 0.0f;
    pid->du_prev = 0.0f;
    pid->feedforward_flow_prev = pid->feedforward_flow;
    pid->steady_count = 0;
    pid->steady_state = false;
    pid->output_saturated = false;
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
