#include "rbf_pid.h"
#include "hyd_config.h"

#include <math.h>
#include <string.h>

#define EPS 1e-6f

static const float RBF_PID_ERROR_DEADBAND = 0.005f;
static const float RBF_PID_SOFT_CAP_RATIO = 1.05f;  /* 保持原语义: 稳态流量的 1.05 倍 */
/* v6: 超调驱动倍率增益 — 允许的流量上限 = 稳态维持流量·(1 + GAIN·|e|/P_set)。
 * 误差越大(离目标越远)允许越多超调驱动；接近目标时平滑收口到稳态流量。 */
static const float RBF_PID_OVERDRIVE_GAIN = 40.0f;
/* v9: 超调驱动授权闸门（相对误差幅值）——替代 v6 的"峰值进展闸门"。
 *
 * 【为什么替换】v6 用 `P_actual >= overdrive_peak_press - HYST` 判定"仍在爬升"，
 * 该条件在压力冲高回落后立即关闭，把上限收回到软上限 1.05*Q_ss。此时误差仍有
 * 14~24 bar（P_set=150bar），超调驱动本应授权到 ~4.4~5.0 L/min，却被撤销到
 * 0.79 L/min —— 恢复段以约 1/6 的流量爬行，ts 由 ~0.5s 恶化到 1.5~2.1s。
 * 真实链实测（25cc/1700rpm, K=200, 管路 3.5m）：
 *   P_set  50/100/150/200 bar -> ts 1799 / 2088 / 1670 / 1469 ms（目标 <=500ms）
 *
 * 【为什么 v6 闸门本来就是多余的】rbf_pid_overdrive_cap() 在 rel_err -> 0 时
 * 本就退化为软上限（(1 + GAIN*0) * 1.05 = 1.05），因此它对"保压纹波"没有任何
 * 额外保护作用，唯一实际效果就是掐掉超调恢复段的驱动。
 * 实测旁证：把该闸门阈值扫为 -1 / 0.005 / 0.01 / 0.02，结果完全相同
 * （582 / 625 / 561 / 528 ms）—— 恰好证明它在保压段恒等。
 *
 * 【新判据】直接用相对误差幅值授权：rel_err > GATE 才允许超调驱动。
 *   升压段 rel_err ~ 1.0，恢复段 rel_err ~ 0.1~0.2，均远大于 GATE；
 *   保压段 rel_err < GATE -> 回到软上限（纹波不被调制）。
 * 阈值与 RBF_PID_OVERDRIVE_MIN_REL_ERR 取同值是有意的：在 rel_err == GATE 处
 * 超调驱动上限恰好等于软上限（见 rbf_pid_overdrive_cap 的死区处理），
 * 因此两个分支在阈值点取值连续，切换不产生流量跳变。
 * 实测阈值 >= 5% 反而劣化（ts 升到 901~1377ms），故不要放大死区。 */
static const float RBF_PID_OVERDRIVE_REL_ERR_GATE = 0.02f;
/* v6: 超调驱动死区 — 归一化误差小于该值时不放宽上限。
 * 保压段传感器噪声(σ=0.4bar)造成的误差波动约 0.4%·P_set，若不加死区，
 * 噪声每拍都会顶开超调闸门→流量指令被噪声调制→稳态纹波放大
 * （实测 σ_ss 由 0.326 恶化到 0.585 bar）。 */
static const float RBF_PID_OVERDRIVE_MIN_REL_ERR = 0.02f;
static const float RBF_PID_DYNAMIC_FF_GAIN = 0.001f;
static const float RBF_PID_WEIGHT_LIMIT = 5.0f;
/* v8: 升压限流默认制动窗口（占目标压力比例）。0.5 = 剩余误差降到 50%·P_set
 * 时开始收口。选值依据（25cc/1700rpm, K=200 bar/(L/min), Q_boost≈10.6 L/min）：
 * 在 e = 0.5*P_set 处最大建压速率 = (K*Q_boost - P)/(tau) ≈ 2 bar/ms，
 * 与 1ms 控制周期匹配 → 离散化超调 < 3 bar（<2%）。 */
static const float RBF_PID_BOOST_BRAKE_FRAC_DEFAULT = 0.5f;
static const float RBF_PID_BOOST_BRAKE_FRAC_MIN = 0.02f;
static const float RBF_PID_BOOST_BRAKE_FRAC_MAX = 10.0f;

static float rbf_pid_compute_soft_flow_cap(const RBF_PID_Handle *pid);
static bool rbf_pid_in_boost_phase(const RBF_PID_Handle *pid, float error);
static float rbf_pid_overdrive_cap(const RBF_PID_Handle *pid, float error);
static float rbf_pid_boost_flow_cap(const RBF_PID_Handle *pid, float error);
static float rbf_pid_effective_soft_cap(const RBF_PID_Handle *pid, float error);
static float clampf(float min_value, float value, float max_value);
static float rbf_pid_effective_du_scale(const RBF_PID_Handle *pid);

static float rbf_pid_discrete_jacobian_min(const RBF_PID_Handle *pid) {
    float tau = pid->process_time_constant_s;
    float dt = HYD_DEFAULT_RBF_PID_SAMPLING_PERIOD;
    float nominal;

    if (!(pid->K > 0.0f) || !(tau > 0.0f) || !isfinite(tau)) {
        return 0.01f;  /* 从0.005提升到0.01 — 修复K=1.5时稳态误差 */
    }
    nominal = pid->K * dt / fmaxf(tau, 0.1f);
    return clampf(0.01f, 0.5f * nominal, 0.1f);  /* 下界从0.005→0.01，上界从0.5→0.1 */
}

static float rbf_pid_discrete_jacobian_max(const RBF_PID_Handle *pid) {
    float tau = pid->process_time_constant_s;
    float dt = HYD_DEFAULT_RBF_PID_SAMPLING_PERIOD;
    float nominal;

    if (!(pid->K > 0.0f) || !(tau > 0.0f) || !isfinite(tau)) {
        return 2.0f;
    }
    nominal = pid->K * dt / fmaxf(tau, 0.1f);
    return fmaxf(rbf_pid_discrete_jacobian_min(pid),
                 clampf(0.05f, 4.0f * nominal, 2.0f));
}

static void rbf_pid_refresh_adaptation_gate(RBF_PID_Handle *pid) {
    float excitation_floor;

    if (pid == NULL) {
        return;
    }
    excitation_floor = 0.02f * rbf_pid_effective_du_scale(pid);
    pid->adaptation_frozen = !pid->dt_valid ||
        !isfinite(pid->P_set) || !isfinite(pid->P_actual) ||
        !isfinite(pid->du_prev) || fabsf(pid->du_prev) < excitation_floor ||
        pid->output_saturated;
}

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

static float rbf_pid_effective_du_scale(const RBF_PID_Handle *pid) {
    return clamp_positive_or_default(pid->f_dd_press_prev, 5.0f);
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
    /* v6: 用"有效"软上限（升压段超调驱动窗口内解除），与实际输出限幅保持一致，
     * 否则会出现"输出已经放开到硬限幅、但饱和判定仍按稳态上限"的自相矛盾，
     * 导致升压段积分被误冻结在 P_set/K 附近（这正是 K=5.0 时 tr 恶化的直接原因）。 */
    soft_cap = rbf_pid_effective_soft_cap(pid, error);
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

// ========== 稳态判定参数（可调） ==========
#define STEADY_DEAD_ZONE     2.0f   /* 误差死区（bar），从10.0降到2.0 — 提高响应灵敏度 */
#define STEADY_DE_RATIO      0.5f   /* 变化率死区系数，从1.0降到0.5 */

static int rbf_pid_step_rbf_nn(RBF_PID_Handle *pid,float error) {
    float h[RBF_HNUM];
    float du_scale = rbf_pid_effective_du_scale(pid);
    float pressure_scale = rbf_pid_effective_pressure_scale(pid);
    float x[RBF_INPUT_DIM] = {
        pid->du_prev / du_scale,
        pid->y_prev1 / pressure_scale,
        pid->y_prev2 / pressure_scale
    };
    float y_n = pid->P_actual / pressure_scale;
    float y_hat_n = 0.0f;
    float jacobian_n = 0.0f;
    float error_rbf_n;
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

    pid->Jacobian = (pressure_scale / du_scale) * jacobian_n;

    /*
     * x0 is du_prev / du_scale, so this is the one-sample discrete
     * sensitivity g_du, not the steady-state process gain K.  For a
     * calibrated first-order plant its nominal value is K*dt/tau.
     * Keep the shadow-only range when no process calibration is available.
     */
    {
        float jac_lo = rbf_pid_discrete_jacobian_min(pid);
        float jac_hi = rbf_pid_discrete_jacobian_max(pid);
        float jac = pid->Jacobian;

        if (!isfinite(jac) || jac <= 0.0f) {
            jac = (pid->K > 0.0f) ? pid->K *
                HYD_DEFAULT_RBF_PID_SAMPLING_PERIOD /
                fmaxf(pid->process_time_constant_s, 0.1f) : jac_lo;
        }
        pid->Jacobian = clampf(jac_lo, jac, jac_hi);
    }

    // ---------- 4. 稳态判定（冻结条件） ----------
        // 当误差和误差变化率都很小时，认为系统进入稳态
    float de  = error - pid->e_prev1;
    int is_steady = (fabsf(error) < STEADY_DEAD_ZONE) &&
                        (fabsf(de) < STEADY_DEAD_ZONE * STEADY_DE_RATIO);

	if (!is_steady && !pid->adaptation_frozen && pid->dt_valid) {
		error_rbf_n = y_n - y_hat_n;

		/* P0-2修复：权重饱和抑制 — PI 模式冻结学习防 Jacobian 偏估 */
		bool skip_learning = false;
		if (pid->control_mode == RBF_PID_CONTROL_MODE_PI) {
			skip_learning = rbf_pid_same_direction_saturation(pid, pid->Error);
		}

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

	return is_steady;
}

static float rbf_pid_compute_soft_flow_cap(const RBF_PID_Handle *pid) {
    float hard_limit = rbf_pid_max_flow_output(pid);

    if (!(pid->K > HYD_MIN_SAFE_SYSTEM_GAIN) || !(pid->P_set > 0.0f)) {
        return hard_limit;
    }

    return clampf(0.0f, (pid->P_set * RBF_PID_SOFT_CAP_RATIO) / pid->K, hard_limit);
}

/* 升压段判定：|e| > 20%·P_set（下限 5bar 防止小设定值下误判） */
static bool rbf_pid_in_boost_phase(const RBF_PID_Handle *pid, float error) {
    float boost_threshold = fabsf(pid->P_set) * 0.20f;
    if (boost_threshold < 5.0f) {
        boost_threshold = 5.0f;
    }
    return fabsf(error) > boost_threshold;
}

/* v6: 有效软上限 = 稳态维持上限，升压段且压力确实在上行时解除
 *
 * 实测依据（tests/test_openloop_gain_probe 开环探针，PressureModel 默认参数）：
 *   n=2..40rpm 段 P_ss = 5.400·n 完全线性 → Ksys = 5.400 bar/rpm 存在且可测；
 *   n≥80rpm 被 250bar 溢流阀截顶 → 线性段之外 K 失效。
 *   τ=1.0s（t90 = 2303ms = 2.303τ，各转速点完全相同，一阶特征确认）。
 *
 * 问题：稳态维持上限 P_set·1.05/K 只是"维持"该压力所需流量，不是"建立"它所需流量。
 *   P_set=150, K=5.0 → 上限 31.5 → 可达稳态仅 170bar → t90 = -τ·ln(1-135/170) = 1584ms。
 *   实测 tr = 1584ms，与理论完全吻合（K=0 无上限时 tr = 371ms）。
 * 解除后：指令 90 → 等效稳态 486bar（被 250bar 量程截顶）→ t90 ≈ 325ms。
 *
 * 安全闸门：必须观测到压力确实在上行（P_actual > fLastActPress）才放宽。
 *   泵故障 / 管路堵塞 / 传感器卡死时压力不上升 → 保持稳态上限，
 *   避免闭环在"打空"工况下持续满量程输出（这是原固定上限真实具有的保护作用）。
 * 仅升压方向放宽（error > 0）：泄压段不需要超调驱动，保持上限不影响负流量泄压。
 *
 * 放宽幅度与归一化误差成正比（不是全开也不是全关）：
 *   实测全开（升压段直接放开到硬限幅）tr=852ms——因为升压段阈值一过(|e|<20%·P_set)
 *   上限立刻收回，最后 15bar 只能靠 157bar 等效稳态爬（占 500ms）。
 *   改为随误差平滑收口后，接近目标时上限才收敛到稳态流量，全程保持足够驱动压差。 */
/* 超调驱动上限（不带安全闸门）：= min(硬限幅, 稳态维持流量·(1+GAIN·|e|/P_set)·1.05)
 * 用于"设定值刚跳变"时的 u_prev 播种——跳变本身就是超调驱动的授权。 */
static float rbf_pid_overdrive_cap(const RBF_PID_Handle *pid, float error) {
    float hard_limit = rbf_pid_max_flow_output(pid);
    float cap;

    if (pid->K <= 0.0f || pid->P_set <= 0.0f || error <= 0.0f) {
        return hard_limit;
    }

    cap = rbf_pid_compute_soft_flow_cap(pid);
    {
        float rel_err = fabsf(error) / fabsf(pid->P_set);
        if (rel_err > 1.0f) {
            rel_err = 1.0f;
        }
        rel_err -= RBF_PID_OVERDRIVE_MIN_REL_ERR;   /* 死区：近目标/噪声区不放宽 */
        if (rel_err < 0.0f) {
            rel_err = 0.0f;
        }
        float overdrive_cap = (fabsf(pid->P_set) / pid->K) *
                              (1.0f + RBF_PID_OVERDRIVE_GAIN * rel_err) *
                              RBF_PID_SOFT_CAP_RATIO;   /* 只放宽，不收紧 */
        if (overdrive_cap > cap) {
            cap = overdrive_cap;
        }
    }

    return (cap < hard_limit) ? cap : hard_limit;
}

float RBF_PID_OverdriveFlowCap(const RBF_PID_Handle *pid, float error) {
    if (pid == NULL) {
        return 0.0f;
    }
    if (pid->effective_upper_cap_valid) {
        return clampf(rbf_pid_output_lower_bound(pid),
                      pid->effective_upper_cap,
                      rbf_pid_output_upper_bound(pid));
    }
    return rbf_pid_overdrive_cap(pid, error);
}

/* v8: 升压段实验限流上限（制动包络）
 *
 * 【问题 — 25cc/rev + 1700rpm 实测】
 *   泵额定 42.5 L/min，flowToPumpSpeedGain = 40 rpm/(L/min)，
 *   整机增益 K = Ksys/(D/1000) = 5.0 bar/rpm / 0.025 L/rev = 200 bar/(L/min)。
 *   满流量等效建压 K*Qmax = 200*42.5 = 8500 bar，相对 150 bar 目标有 57 倍的
 *   "流量权限"。即控制器的驱动能力远超需求，超调与否完全由流量上限决定
 *   —— 这正是"升压段用实验限流把超调钉住"的物理依据。
 *
 * 【重要更正 — 勿再引用旧数字】
 *   早期版本此处写"不限流时 Mp = 66.7%（撞 250bar 溢流阀）"。该数字来自
 *   **非物理管路参数**（R = 2.0e11 Pa*s/m^3，让泵出口与容腔解耦），
 *   按 3.5m/12mm 真实几何标定后，同一条控制链不限流的 Mp 只有 3.69%。
 *   因此"升压限流"不是补救措施，而是**给超调上保险的调试旋钮**（默认关闭）。
 *   详见 docs/压力闭环控制链再评估-2026-09-16.md §11.7 / §12.1(C)。
 *
 * 【解法 — 制动包络，不是硬砍】
 *   被控对象（简化一阶）：dP/dt = (K*Q - P)/tau，Q 为控制器输出流量。
 *   令制动窗口 e_b = P_set * brake_frac，升压段允许流量
 *       Q_allow(e) = Q_ss + (Q_boost - Q_ss) * min(1, e/e_b),  e > 0
 *       Q_ss = P_set/K（维持目标压力的稳态流量）
 *   代入对象方程：
 *       dP/dt = (e/tau) * [1 + (K*Q_boost - P_set)/e_b]
 *   即闭环仍是"指数逼近目标"，时间常数
 *       tau_eff = tau / [1 + (K*Q_boost - P_set)/e_b]  <<  tau
 *   且当 e → 0 时 Q → Q_ss，等效平衡压力恰好 = P_set。
 *   ⇒ 无稳态超调、无极限环；这正是位置规划器 sqrt(2*a*s) 制动律在压力域的
 *     对应形式（位置域"速度-位移"制动 = 压力域"流量-压差"制动）。
 *
 * 【为什么单调安全】
 *   返回的上限永不低于 Q_ss（e > 0 时），也永不超过硬限幅；e <= 0 时返回硬限幅
 *   （不收紧）。因此叠加在既有 soft cap / overdrive cap 之上只会让流量更小，
 *   不会凭空放大，既有回归用例在未调用 SetBoostFlowLimit 时行为完全不变。 */
static float rbf_pid_boost_flow_cap(const RBF_PID_Handle *pid, float error) {
    float hard_limit = rbf_pid_max_flow_output(pid);
    float q_ss;
    float q_boost;
    float e_brake;
    float frac;

    if (!(pid->boost_flow_limit_lmin > 0.0f) ||
        !(pid->K > HYD_MIN_SAFE_SYSTEM_GAIN) ||
        !(pid->P_set > 0.0f)) {
        return hard_limit;   /* 未配置限流：不改变既有行为 */
    }
    if (error <= 0.0f) {
        /* 已达/超过目标：升压限流不收紧（泄压/回退交给 PID 与下界处理） */
        return hard_limit;
    }

    q_boost = clampf(0.0f, pid->boost_flow_limit_lmin, hard_limit);
    q_ss = (pid->P_set * RBF_PID_SOFT_CAP_RATIO) / pid->K;
    if (q_ss > q_boost) {
        /* v9 修正 D11【严重·潜伏】：限流值低于"维持流量"时目标压力永不可达。
         *
         * 旧写法 `q_ss = q_boost;` 把制动包络退化为常量 Q_lim，稳态压力被钉在
         * K*Q_lim，永远达不到 P_set，且无任何诊断输出。误差有闭式规律
         *     ess = K * Q_lim - P_set
         * 实测（真实链 K=200）完全吻合：
         *   Q_lim=0.5 L/min: ess@100bar = -0.50, ess@150 = -50.56, ess@200 = -100.56
         *   Q_lim=1.0 L/min: ess@200bar = -0.84, ess@250 = -50.82
         *   Q_lim=2.0 L/min（> 250bar 的下界 1.25）: 全部收敛，ess -0.09~-0.57
         * 即"可达性下界"= P_set/K：Q_lim 必须大于它。
         *
         * 本机 K=200 时下界仅 1.25 L/min，现场不易踩到；但 K 是标定量——
         * 若换成 Ksys=1 bar/rpm + 25cc 的机型，K=40，250bar 的下界升到 6.25 L/min，
         * 现场按"经验值 5 L/min"限流就会静默失效（压力永远打不到还不报警）。
         *
         * 修正方式：把"压低维持流量"改为"抬高低限流"。限流的作用是限制
         * 升压瞬态流量，而不是把流量压到维持流量以下——后者等价于让目标不可达。
         * 抬到 q_ss 后包络恒为 q_ss（够达目标、无超调），超调仍由包络与
         * 硬限幅共同守住；可达性由构造保证。 */
        q_boost = clampf(0.0f, q_ss, hard_limit);
    }

    e_brake = fabsf(pid->P_set) * pid->boost_flow_brake_frac;
    if (!(e_brake > 0.0f)) {
        e_brake = fabsf(pid->P_set) * RBF_PID_BOOST_BRAKE_FRAC_DEFAULT;
    }
    if (e_brake < 1.0f) {
        e_brake = 1.0f;
    }

    frac = error / e_brake;
    if (frac > 1.0f) {
        frac = 1.0f;
    }

    return q_ss + (q_boost - q_ss) * frac;
}

static float rbf_pid_intrinsic_soft_cap(const RBF_PID_Handle *pid, float error) {
    float cap;
    float boost_cap;

    if (pid->K <= 0.0f || pid->P_set <= 0.0f) {
        return rbf_pid_max_flow_output(pid);
    }
    /* v9: 超调驱动授权闸门 = 误差幅值（替代 v6 的"峰值进展 + 停滞"双闸门）。
     * 理由与实测证据见 RBF_PID_OVERDRIVE_REL_ERR_GATE 的注释。
     * 语义：误差足够大（无论压力是在爬升还是刚从超调回落）就授权超调驱动；
     * 误差小到噪声量级才收回到软上限。 */
    {
        float rel_err = (pid->P_set > 0.0f) ? (error / pid->P_set) : 0.0f;
        if (error > 0.0f && rel_err > RBF_PID_OVERDRIVE_REL_ERR_GATE) {
            cap = rbf_pid_overdrive_cap(pid, error);
        } else {
            cap = rbf_pid_compute_soft_flow_cap(pid);
        }
    }

    /* v8: 升压限流是"附加的、只能收紧的上限"——既有两条上限语义保持不变，
     * 只有在显式配置了限流值之后才可能更低。 */
    boost_cap = rbf_pid_boost_flow_cap(pid, error);
    if (boost_cap < cap) {
        cap = boost_cap;
    }
    return cap;
}

static float rbf_pid_effective_soft_cap(const RBF_PID_Handle *pid, float error) {
    if (pid->effective_upper_cap_valid) {
        return clampf(rbf_pid_output_lower_bound(pid),
                      pid->effective_upper_cap,
                      rbf_pid_output_upper_bound(pid));
    }
    return rbf_pid_intrinsic_soft_cap(pid, error);
}

#define ETA_KD_BOOST 0.5f // 微分强制唤醒系数
#define LAMBDA_KI    0.0005f // 积分惩罚系数
#define KI_CENTER    0.0f   // 积分中心值

static void rbf_pid_step_adaptive_gains(RBF_PID_Handle *pid, float error, float raw_error)
{
	float de  = error - pid->e_prev1;
	float dde = de - (pid->e_prev1 - pid->e_prev2);
	// ---------- 5. PID 参数在线整定（带抗饱和 & 微分唤醒） ----------
	float abs_Jac = fabsf(pid->Jacobian);
	if (abs_Jac < 1e-6f) {
		abs_Jac = 1e-6f;   // 避免除零
	}
	// 5.3 比例增益 Kp 更新（常规梯度）
	//    公式：ΔKp = ηp * e * Jac * Δe
	float grad_Kp = pid->eta_p * error * sign(pid->Jacobian) * abs_Jac * de;
	if (grad_Kp > 0.005f) grad_Kp = 0.005f;
	if (grad_Kp < -0.005f) grad_Kp = -0.005f;
	pid->KP += grad_Kp;
	pid->KP = clampf(pid->min_KP, pid->KP, pid->max_KP);

	// 5.1 积分增益 Ki 更新（带L2惩罚，防止积分饱和）
	// 修复稳态误差：移除 abs_Jac * error 项，避免误差小时更新停滞
	float grad_Ki = pid->eta_i * error * sign(pid->Jacobian);
	float decay_Ki = LAMBDA_KI * (pid->KI - KI_CENTER);
	float delta_Ki = grad_Ki - decay_Ki;
	if (delta_Ki > 0.0001f) delta_Ki = 0.0001f;   // 放宽从0.00005→0.0001
	if (delta_Ki < -0.0001f) delta_Ki = -0.0001f;
	pid->KI += delta_Ki;
	pid->KI = clampf(pid->min_KI, pid->KI, pid->max_KI);

	if (pid->control_mode == RBF_PID_CONTROL_MODE_PI) {
		pid->KD = 0.0f;
	} else {

		// 5.2 微分增益 Kd 更新（带"强制唤醒"机制）
		float delta_Kd = 0.0f;

		// 判断：误差是否在发散且Kd处于低位？
		int error_diverging = (fabsf(error) > fabsf(pid->e_prev1)) && (fabsf(error) > 0.01f);
		int kd_at_floor = (pid->KD <= pid->min_KD * 1.1f);

		if (error_diverging && kd_at_floor) {
			// 【强制唤醒】放弃纯梯度，强行提升Kd
			float boost = ETA_KD_BOOST * fabsf(de) * sign(pid->Jacobian);
			delta_Kd = fmaxf(boost, 0.0f);
		} else {
			// 正常情况：使用"绝对值整流"防止负向累积
			float grad_Kd_base = pid->eta_d * error * sign(pid->Jacobian) * fabsf(dde);
			// 再加一点"趋势预测"：如果误差正在减小，保持Kd不掉太快
			if (error * de < 0) {
				// 误差在收拢，微分项已经起效，不要过度衰减
				delta_Kd = fmaxf(grad_Kd_base, 0.0f);
			} else {
				delta_Kd = grad_Kd_base;
			}
		}
		if (delta_Kd > 0.0002f) delta_Kd = 0.0002f;
		if (delta_Kd < -0.0002f) delta_Kd = -0.0002f;
		pid->KD += delta_Kd;
		pid->KD = clampf(pid->min_KD, pid->KD, pid->max_KD);

	}
}

static void rbf_pid_step_incremental_output(RBF_PID_Handle *pid, float error, float raw_error) {
    float hard_limit = rbf_pid_max_flow_output(pid);
    float output_min = rbf_pid_output_lower_bound(pid);
    float output_max = rbf_pid_output_upper_bound(pid);

    /* 相位判定：升压段(|e| > 20%·P_set) vs 逼近段。提到最前以复用给软上限。 */
    bool in_boost_phase = rbf_pid_in_boost_phase(pid, error);

    /* v6: 有效软上限（升压段超调驱动窗口内解除，见函数注释中的实测依据） */
    float flow_cap = rbf_pid_effective_soft_cap(pid, error);
    float soft_output_max = (flow_cap < hard_limit) ? flow_cap : hard_limit;

    if (soft_output_max > output_max) {
        soft_output_max = output_max;
    }

    /* v10: 泵实测可达流量钳位（back-calculation anti-windup）
     *
     * 由 pressure_controller 从伺服泵实测转速换算（rpm / flowToPumpSpeedGain），
     * 仅在实测转速贴上 IEC 配置的转速上限、且 IEC 使能了该功能时才置位。
     *
     * 与 v8 升压限流的区别：升压限流是"开环、按设计值猜"的预防性上限；
     * 这里是"闭环、按实测"的可行性上限 —— 泵真的到不了那么大流量时才收紧。
     * 只收紧上限，不触碰下限，因此负流量泄压路径完全不受影响。 */
    if (pid->external_flow_cap_valid && pid->external_flow_cap < soft_output_max) {
        soft_output_max = pid->external_flow_cap;
    }

    float d_term = 0.0f;
    float du;

    if (pid->control_mode == RBF_PID_CONTROL_MODE_PI) {
        pid->prev_d_term = 0.0f;
    } else {
        float raw_d_term = (error - 2.0f * pid->e_prev1 + pid->e_prev2);
        const float flt_alpha = HYD_DEFAULT_RBF_D_FILTER_ALPHA;
        d_term = flt_alpha * raw_d_term +
            (1.0f - flt_alpha) * pid->prev_d_term;
        pid->prev_d_term = d_term;
    }
    /* v5: 相位分离积分 — 升压段大钳位快速建压，逼近段小钳位让P项主导制动，
     *   窄带内中钳位消稳态误差。boost_threshold/in_boost_phase 已在函数开头计算。 */
    float integral_zone = fabsf(pid->P_set) * 0.05f;
    if (integral_zone < 5.0f) integral_zone = 5.0f;
    bool in_integral_zone = (!in_boost_phase && fabsf(error) < integral_zone);

    float ki_eff = pid->KI;
    float interf_term = ki_eff * error;

    float interf_clamp;
    if (in_boost_phase) {
        interf_clamp = 0.02f * rbf_pid_max_flow_output(pid);  /* 升压段 ±1.8 */
    } else if (in_integral_zone) {
        interf_clamp = 0.005f * rbf_pid_max_flow_output(pid);  /* 窄带 ±0.45 */
    } else {
        interf_clamp = 0.0008f * rbf_pid_max_flow_output(pid);  /* 逼近段 ±0.072 */
    }

    if (rbf_pid_same_direction_saturation(pid, error)) {
        interf_term = 0.0f;
    }
    interf_term = clampf(-interf_clamp, interf_term, interf_clamp);

    /* v4 升压段增益调度：boost 段 KP×2 加速建压 */
    float kp_eff = in_boost_phase ? (pid->KP * 2.0f) : pid->KP;
    du = kp_eff * (error - pid->e_prev1) + interf_term + pid->KD * d_term;

    float actual_press = pid->P_actual;

    float f_delta_press = actual_press - pid->fLastActPress;

    /* v7: 删除 f_velfb（压力加速度前馈），恒为 0。
     *
     * 【撤销依据 — A/B 对照矩阵 tests/test_pressure_ab_matrix.c】
     *   带传感器噪声(σ=0.4bar)+量化(0.25bar)、前置滤波 α=0.1（与生产链路一致）：
     *     f_velfb ON  (V0)：ess = 2.556 bar  → 超出 1.0 bar 目标，FAIL
     *     f_velfb OFF (V1)：ess = -0.002 bar，tr 330ms，ts 380ms，Mp 0.56% → PASS
     *   关掉后 tr 366→330ms、ts 454→380ms 同时改善，Mp 由 0.00% 升到 0.56%（仍远低于 5%）。
     *
     * 【根因】v5 为了"不阻碍泄压恢复"把 f_velfb 做成非对称：只在 ΔP>0（升压）时制动，
     *   ΔP<0 时不干预。这在无噪声时成立（稳态 ΔP≡0，f_velfb≡0）；
     *   但**随机噪声下 ΔP 的符号是随机的，整流后不再零均值** → 产生系统性负偏置
     *   → 稳态压力被压低 2.56 bar。这是原实现在无噪声仿真里永远暴露不了的缺陷。
     *
     * 【为何不改成对称】对称化后 f_velfb = -kv·ΔP 求和 = -kv·(P(k)-P(0))，
     *   是一个有界比例项，与 KP·Δe 功能完全重复（见评估报告 9.2），
     *   只会多出 4 个调参量。按"不过度设计"原则整体删除。
     *
     * 保留 pressure_accel_ff_enabled 字段与 SetPressureAccelFeedforwardEnabled() 仅为
     * 兼容既有调用方；该开关自 v7 起不再影响任何计算，属待清理的废弃接口。 */
    float f_velfb = 0.0f;
    (void)f_delta_press;
    (void)actual_press;

    float vel_ref  = pid->P_set - pid->last_ref;
    vel_ref  = clampf( -10.0f, vel_ref, 10.0f );
    float f_du_ff  = RBF_PID_DYNAMIC_FF_GAIN * ( vel_ref -  pid->v_ref_k1 );

   // du = clampf( -0.5, du, 0.5 );
//    printf("output_min: %.3f, output_max: %.3f,  kp:%.3f,k:%.3f,kd:%.3f,du:%.6f\n", output_min, output_max,
//    		pid->KP, pid->KI, pid->KD, du);

    pid->du = !isfinite(du) ? 0.0f : du;

    pid->Output = pid->u_prev + pid->du + f_du_ff + f_velfb;

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

    /* v6: 压力停滞计时 — 用于超调驱动安全闸门。
     * 压力有可见上升(>0.01bar/拍)则清零；否则累加采样周期。
     * 泻压方向(error<0)不计停滞：那是正常工况，不应触发保护。 */
    if (actual_press > pid->fLastActPress2 + 0.01f || error < 0.0f) {
        pid->press_stuck_time_s = 0.0f;
    } else {
        pid->press_stuck_time_s += pid->sampling_period;
    }

    /* v6: 峰值进展闸门参考点更新（本轮升压达到的最高压力） */
    if (actual_press > pid->overdrive_peak_press) {
        pid->overdrive_peak_press = actual_press;
    }
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
    pid->process_time_constant_s = 1.0f;
    pid->dt_valid = true;
    pid->adaptation_frozen = false;
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
    /* v8: 升压限流默认关闭（0 = 关闭），制动窗口给默认比例。
     * 关闭是有意的：限流值必须由现场实验确定（泵排量/管路/模具各不相同），
     * 库不替用户猜；开启后行为单调收紧，因此默认关闭不改变既有回归基线。 */
    pid->boost_flow_limit_lmin = 0.0f;
    pid->boost_flow_brake_frac = RBF_PID_BOOST_BRAKE_FRAC_DEFAULT;
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
    bool target_jump;

    pid->P_set = isfinite(setpoint) ? setpoint : 0.0f;
    pid->P_actual = isfinite(feedback) ? feedback : 0.0f;
    target_jump = fabsf(pid->P_set - pid->last_ref) > 5.0f;

    if (!pid->dt_valid) {
        float output_min = rbf_pid_output_lower_bound(pid);
        float output_max = rbf_pid_output_upper_bound(pid);

        pid->adaptation_frozen = true;
        pid->prev_d_term = 0.0f;
        pid->Error = pid->P_set - pid->P_actual;
        pid->e_prev1 = pid->Error;
        pid->e_prev2 = pid->Error;
        pid->y_prev1 = pid->P_actual;
        pid->y_prev2 = pid->P_actual;
        pid->Output = clampf(output_min, pid->Output, output_max);
        pid->u_prev = pid->Output;
        pid->du = 0.0f;
        return pid->Output;
    }

    /* v6: 稳态前馈播种 — 设定值显著跳变时用 P_set/K 初始化 u_prev
     * 仅在跳变(|ΔP_set| > 5bar)时触发；ramp 渐进(每拍变化小)不触发→
     * 不会每拍重置 u_prev 与 PID 打架。last_ref 在步进末尾更新,
     * 故此处读到的是上拍设定值。
     * 泄压保护: P_set/K 恒为正, 若实测压力已高于设定值(需负流量泄压),
     *   前馈会把输出顶在正流量上阻碍泄压(rbf_pid_test 负输出断言实测)。
     *
     * v6b: 播种值改为"超调驱动上限"而非"稳态维持流量"。
     *   实测(调试轨迹): 播种在稳态流量 29.17 后, 输出只能靠 KI·e≈0.84/拍 爬升,
     *   从 29→90 需约 77ms, 这段完全浪费(输出未饱和, 压力升得慢)→tr=414ms。
     *   播种到超调驱动上限(误差大时≈硬限幅 90)后, 起始即为满流量,
     *   tr 应逼近一阶对象的物理下限 325ms。
     *   安全性: 上限本身由 K 与归一化误差决定, K 大(系统刚性强)时上限自动收窄,
     *   不会出现"不知 K 就满量程冲"的危险。 */
    if (pid->K > 0.0f && pid->P_set > 0.0f) {
        float sp_delta = fabsf(pid->P_set - pid->last_ref);
        bool need_relief = (pid->P_actual > pid->P_set + 1.0f);
        if (sp_delta > 5.0f && !need_relief) {
            float ff_flow = rbf_pid_overdrive_cap(pid, pid->P_set - pid->P_actual);
            ff_flow = clampf(rbf_pid_output_lower_bound(pid), ff_flow,
                             rbf_pid_output_upper_bound(pid));
            pid->u_prev = ff_flow;
            pid->Output = ff_flow;
            pid->press_stuck_time_s = 0.0f;  /* 新工况重新给超调驱动一个观察窗口 */
            pid->overdrive_peak_press = pid->P_actual;  /* 峰值闸门从当前压力重新起算 */
        }
    }

    /* P0-1修复：统一数值防护，移到enforce之前，模式无关 */
    rbf_pid_sanitize_runtime_state(pid);

    rbf_pid_refresh_adaptation_gate(pid);

    rbf_pid_enforce_control_mode(pid);
    raw_error = pid->P_set - pid->P_actual;
    error = rbf_pid_apply_deadband(raw_error);
    pid->Error = error;
    if (target_jump) {
        /* Seed the causal history on standalone target jumps as well as outer
         * controller soft resets; this prevents a one-sample D kick. */
        pid->e_prev1 = error;
        pid->e_prev2 = error;
        pid->prev_d_term = 0.0f;
    }
    pid->control_state = rbf_pid_resolve_control_state(pid, raw_error);

    int is_steady = rbf_pid_step_rbf_nn(pid,error);

    rbf_pid_step_incremental_output(pid, error, raw_error);

    pid->y_prev2 = pid->y_prev1;
    pid->y_prev1 = pid->P_actual;
    pid->u_prev = pid->Output;
    pid->du_prev = pid->du;

    if (!is_steady && !pid->adaptation_frozen && pid->dt_valid) {
    	rbf_pid_step_adaptive_gains(pid, error, raw_error);
    }

    pid->e_prev2 = pid->e_prev1;
    pid->e_prev1 = error;

    rbf_pid_step_steady_state(pid);
    pid->Status = pid->steady_state ? 3 : 2;

    return pid->Output;
}

void RBF_PID_Reset(RBF_PID_Handle *pid) {
    float sampling_period = pid->sampling_period;
    float max_flow = pid->fMaxFlow;
    float flow_limit = pid->fFlowRateLimit;
    /* v8: 升压限流是"现场配置量"，与量程/控制周期同属配置而非运行状态，
     * 复位时必须保留，否则每次复位都会静默退回不限流（安全隐患）。 */
    float boost_flow_limit = pid->boost_flow_limit_lmin;
    float boost_brake_frac = pid->boost_flow_brake_frac;

    RBF_PID_Init(pid, sampling_period, max_flow, flow_limit);
    pid->boost_flow_limit_lmin = boost_flow_limit;
    pid->boost_flow_brake_frac = boost_brake_frac;
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

void RBF_PID_SetProcessTimeConstant(RBF_PID_Handle *pid, float tau_s) {
    if (pid == NULL) {
        return;
    }
    pid->process_time_constant_s =
        (isfinite(tau_s) && tau_s > 0.0f) ? tau_s : 1.0f;
}

void RBF_PID_SetDtValid(RBF_PID_Handle *pid, bool valid) {
    if (pid == NULL) {
        return;
    }
    pid->dt_valid = valid;
    pid->adaptation_frozen = !valid;
    if (!valid) {
        /* Seed the D state before the next valid sample to avoid a kick. */
        pid->prev_d_term = 0.0f;
        pid->e_prev2 = pid->e_prev1;
    }
}

void RBF_PID_SetExternalFlowCap(RBF_PID_Handle *pid, float cap_lmin, bool enable) {
    if (pid == NULL) {
        return;
    }

    /* 负值（泵反转泄压）不是"可达上限"，按 0 处理：
     * 此时不允许再朝正向加流，但下限（负流量泄压）保持不受影响。 */
    if (!isfinite(cap_lmin) || cap_lmin < 0.0f) {
        cap_lmin = 0.0f;
    }

    pid->external_flow_cap = cap_lmin;
    pid->external_flow_cap_valid = enable;
    pid->external_saturated = enable;
}

void RBF_PID_SetEffectiveUpperCap(RBF_PID_Handle *pid, float cap_lmin, bool valid) {
    if (pid == NULL) {
        return;
    }

    pid->effective_upper_cap = (isfinite(cap_lmin) && cap_lmin >= 0.0f) ? cap_lmin : 0.0f;
    pid->effective_upper_cap_valid = valid && isfinite(cap_lmin) && cap_lmin >= 0.0f;
}

float RBF_PID_GetIntrinsicUpperCap(const RBF_PID_Handle *pid, float error) {
    float cap;
    if (pid == NULL) {
        return 0.0f;
    }
    cap = rbf_pid_intrinsic_soft_cap(pid, error);
    if (pid->external_flow_cap_valid && pid->external_flow_cap < cap) {
        cap = pid->external_flow_cap;
    }
    return clampf(rbf_pid_output_lower_bound(pid), cap,
                  rbf_pid_output_upper_bound(pid));
}

void RBF_PID_ShadowUpdate(RBF_PID_ShadowState *shadow,
                          const RBF_PID_Handle *pid,
                          float setpoint,
                          float feedback,
                          float measured_flow,
                          float dt,
                          bool dt_valid) {
    const float residual_limit = 2.0f;
    const float g_du_min = 0.005f;
    const float g_du_max = 2.0f;
    float delta_flow;
    float predicted_feedback;
    float innovation;
    float excitation_floor;
    bool quality_ok;

    if (shadow == NULL) {
        return;
    }

    shadow->last_dt = dt;
    shadow->valid = false;
    shadow->quality_valid = false;
    if (pid != NULL && dt_valid && isfinite(dt) &&
        fabsf(dt - HYD_DEFAULT_RBF_PID_SAMPLING_PERIOD) <=
            HYD_RBF_VALID_DT_TOLERANCE &&
        isfinite(setpoint) && isfinite(feedback) && isfinite(measured_flow)) {
        delta_flow = shadow->history_valid ?
            (measured_flow - shadow->previous_flow) : 0.0f;
        predicted_feedback = shadow->history_valid
            ? shadow->previous_feedback + pid->Jacobian * delta_flow
            : feedback;
        innovation = feedback - predicted_feedback;
        shadow->residual = innovation;
        shadow->g_du = pid->Jacobian;
        if (!shadow->history_valid) {
            shadow->residual_rms = fabsf(innovation);
        } else {
            shadow->residual_rms = sqrtf(0.98f * shadow->residual_rms *
                                         shadow->residual_rms +
                                         0.02f * innovation * innovation);
        }
        excitation_floor = 0.02f *
            ((pid->flow_normalization_scale > 0.0f)
                ? pid->flow_normalization_scale : 90.0f);
        quality_ok = shadow->g_du >= g_du_min &&
                     shadow->g_du <= g_du_max &&
                     fabsf(delta_flow) >= excitation_floor &&
                     shadow->residual_rms <= residual_limit &&
                     !pid->output_saturated &&
                     !pid->adaptation_frozen;
        shadow->valid_sample_count++;
        if (quality_ok) {
            shadow->confidence_sample_count++;
            shadow->quality_valid = true;
        } else {
            shadow->confidence_sample_count = 0U;
        }
        shadow->previous_feedback = feedback;
        shadow->previous_flow = measured_flow;
        shadow->history_valid = true;
        shadow->valid = true;
    } else {
        shadow->invalid_sample_count++;
        shadow->confidence_sample_count = 0U;
    }
}

void RBF_PID_SetBoostFlowLimit(RBF_PID_Handle *pid, float limit_lmin) {
    if (pid == NULL) {
        return;
    }
    /* 非有限值或负值 -> 关闭限流。上限不在此处按硬限幅截断，
     * 因为 rbf_pid_boost_flow_cap 每次调用都会重新 clamp 到当前限幅，
     * 这样后续改 fMaxFlow 也不需要重新配置。 */
    pid->boost_flow_limit_lmin = (isfinite(limit_lmin) && limit_lmin > 0.0f)
                                     ? limit_lmin
                                     : 0.0f;
}

void RBF_PID_SetBoostBrakeFrac(RBF_PID_Handle *pid, float brake_frac) {
    if (pid == NULL) {
        return;
    }
    if (!isfinite(brake_frac)) {
        brake_frac = RBF_PID_BOOST_BRAKE_FRAC_DEFAULT;
    }
    pid->boost_flow_brake_frac = clampf(RBF_PID_BOOST_BRAKE_FRAC_MIN,
                                        brake_frac,
                                        RBF_PID_BOOST_BRAKE_FRAC_MAX);
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
