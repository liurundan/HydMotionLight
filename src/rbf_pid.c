/**
 * @file rbf_pid.c
 * @brief RBF神经网络自适应PID控制器 - 嵌入式C实现
 *
 * 修订 v3.2 要点（P0 级修复）：
 * - F2 微分作用对象统一为滤波后测量：d_term = filt(Δ²y)，输出取 −Kd·d_term，
 *   设定值阶跃不再产生微分踢；
 * - F3 双重死区治理：external_deadband_enabled 置位时内层误差死区旁路；
 * - F4 辨识器输入修正：x[0] = u_prev / flow_norm_scale（绝对输出，激励充分），
 *   Jacobian 语义 = ∂P/∂u [bar/(L/min)]，换算 pressure_scale/flow_scale，
 *   与软限幅/增益补偿共用 K 语义；c[i][0] 初始化覆盖 [0,1]。
 * - F1 前馈链路：ff_delta 透传逻辑不变，feedforward_flow 由外层每拍写入。
 */
#include "rbf_pid.h"
#include "hyd_config.h"
#include <math.h>
#include <string.h>

#define EPS 1e-6f

/* 微分滤波系数，可由 hyd_config.h 覆盖 */
#ifndef HYD_THRESH_RBF_DERIV_FILTER_ALPHA
#define HYD_THRESH_RBF_DERIV_FILTER_ALPHA 0.3f
#endif

/* ==================== 具名常量（含单位） ==================== */
static const float RBF_PID_ERROR_DEADBAND_RATIO = 0.0004f;  /* 误差死区 = 0.04%*压力量程 */
static const float RBF_PID_ERROR_DEADBAND_MIN   = 0.05f;    /* 误差死区下限 [bar] */
static const float RBF_PID_SOFT_CAP_RATIO       = 1.05f;    /* 软限幅 = P_set*1.05/K */
static const float RBF_PID_NEAR_TARGET_RATIO    = 0.08f;    /* 近目标判定: |e|<=8%*|P_set| */
static const float RBF_PID_WEIGHT_LIMIT         = 5.0f;     /* RBF 权重钳位 */
static const float RBF_PID_STEADY_DEADZONE_RATIO= 0.005f;   /* RBF冻结死区 = 0.5%*量程 */
static const float RBF_PID_STEADY_DE_RATIO      = 1.0f;     /* 误差变化率死区系数 */
static const float RBF_PID_INCR_I_LIMIT         = 0.08f;    /* 增量I项单步限幅 [L/min] */
static const float RBF_PID_VEL_DAMP_LIMIT       = 1.0f;     /* 速度阻尼项限幅 [L/min] */

/* 增益单步步长限幅（防梯度异常导致参数突跳） */
static const float RBF_PID_KP_STEP_LIMIT = 0.01f;
static const float RBF_PID_KI_STEP_LIMIT = 0.0002f;
static const float RBF_PID_KD_STEP_LIMIT = 0.002f;

/* Jacobian 符号监督：连续为负拍数阈值（100 拍 = 100ms @1ms） */
#define RBF_PID_JACOBIAN_FAULT_COUNT 100

/* Kd 强制唤醒 / Ki L2 惩罚 */
#define ETA_KD_BOOST 0.5f
#define LAMBDA_KI 0.0005f
#define KI_CENTER 0.0f

/* 稳态判定（外层 Status 用，单位明确） */
static const float RBF_PID_STEADY_E_LIMIT      = 5.0f;   /* [bar] */
static const float RBF_PID_STEADY_T_LIMIT      = 0.2f;   /* [s] */
static const float RBF_PID_STEADY_DU_LIMIT     = 2.0f;   /* [L/min] */
static const float RBF_PID_STEADY_MIN_SETPOINT = 5.0f;   /* [bar] */

/* 速度阻尼方向门控阈值 [bar/step]：单步压力上升超过该值才启用阻尼。 */
static const float RBF_PID_VEL_DAMP_RISE_THRESH = 0.31f;

/* ==================== 基础工具 ==================== */
static float sign(float x)
{
    if (x > EPS)  return 1.0f;
    if (x < -EPS) return -1.0f;
    return 0.0f;
}

static float clampf(float min_value, float value, float max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

static float clamp_positive_or_default(float value, float fallback)
{
    return (value > 0.0f && isfinite(value)) ? value : fallback;
}

static float clamp_finite(float min_value, float value, float max_value, float fallback)
{
    if (!isfinite(value)) value = fallback;
    return clampf(min_value, value, max_value);
}

static float finite_or_default(float value, float fallback)
{
    return isfinite(value) ? value : fallback;
}

static void sort_pair(float *low, float *high)
{
    if (*low > *high) {
        float temp = *low;
        *low = *high;
        *high = temp;
    }
}

/* ==================== 内部解析 ==================== */

/* v3.2：流量归一化标尺（辨识器 x[0] = u / flow_scale） */
static float rbf_pid_effective_flow_scale(const RBF_PID_Handle *pid)
{
    float fallback = (pid->fMaxFlow > 0.0f) ? pid->fMaxFlow : 90.0f;
    return clamp_positive_or_default(pid->flow_normalization_scale, fallback);
}

static float rbf_pid_effective_pressure_scale(const RBF_PID_Handle *pid)
{
    return clamp_positive_or_default(pid->pressure_normalization_scale, MAX_PRESSURE);
}

/* 误差死区：量程百分比 + 绝对下限（仅内层死区未旁路时使用） */
static float rbf_pid_error_deadband(const RBF_PID_Handle *pid)
{
    float db = RBF_PID_ERROR_DEADBAND_RATIO * rbf_pid_effective_pressure_scale(pid);
    return (db < RBF_PID_ERROR_DEADBAND_MIN) ? RBF_PID_ERROR_DEADBAND_MIN : db;
}

static float rbf_pid_apply_deadband(float error, float deadband)
{
    if (fabsf(error) <= deadband) return 0.0f;
    return (error > 0.0f) ? (error - deadband) : (error + deadband);
}

static float rbf_pid_max_flow_output(const RBF_PID_Handle *pid)
{
    float max_output = pid->fMaxFlow * pid->fFlowRateLimit;
    return (max_output > 0.0f) ? max_output : 90.0f;
}

static float rbf_pid_output_lower_bound(const RBF_PID_Handle *pid)
{
    float upper = rbf_pid_max_flow_output(pid);
    float lower = pid->output_min_flow;
    return (lower > upper) ? upper : lower;
}

static float rbf_pid_output_upper_bound(const RBF_PID_Handle *pid)
{
    float upper = pid->output_max_flow;
    if (upper <= 0.0f) upper = rbf_pid_max_flow_output(pid);
    if (upper < rbf_pid_output_lower_bound(pid)) upper = rbf_pid_output_lower_bound(pid);
    return upper;
}

static float rbf_pid_compute_soft_flow_cap(const RBF_PID_Handle *pid)
{
    float hard_limit = rbf_pid_max_flow_output(pid);
    if (pid->K <= 0.0f || pid->P_set <= 0.0f) return hard_limit;
    return clampf(0.0f, (pid->P_set * RBF_PID_SOFT_CAP_RATIO) / pid->K, hard_limit);
}

/* 近目标判定：|误差| <= NEAR_TARGET_RATIO * |P_set| */
static bool rbf_pid_near_target(const RBF_PID_Handle *pid)
{
    float sp = (fabsf(pid->P_set) > 1.0f) ? fabsf(pid->P_set) : 1.0f;
    return fabsf(pid->Error) <= RBF_PID_NEAR_TARGET_RATIO * sp;
}

/*
 * 有效输出上限（P1-4）：
 * - 近目标段（|e| <= 8%*|P_set|）：软限幅与配置上限取小，抑制到位超调；
 * - 动态段：仅硬限幅，避免软限幅与增益辨识误差耦合导致压力打不到设定。
 */
static float rbf_pid_effective_output_max(const RBF_PID_Handle *pid)
{
    float hard_limit = rbf_pid_max_flow_output(pid);
    float cap = hard_limit;
    float upper = rbf_pid_output_upper_bound(pid);

    if (rbf_pid_near_target(pid)) {
        float flow_cap = rbf_pid_compute_soft_flow_cap(pid);
        if (flow_cap < cap) cap = flow_cap;
    }
    if (cap > upper) cap = upper;
    return cap;
}

/*
 * 饱和同向判定（P0-4：模式无关）。
 */
static bool rbf_pid_same_direction_saturation(const RBF_PID_Handle *pid)
{
    float output_min;
    float output_max;

    if (!pid->output_saturated) return false;

    output_min = rbf_pid_output_lower_bound(pid);
    output_max = rbf_pid_effective_output_max(pid);

    return (pid->Output >= output_max - 1.0e-6f && pid->Error > 0.0f) ||
           (pid->Output <= output_min + 1.0e-6f && pid->Error < 0.0f);
}

/* ==================== 默认配置 ==================== */
static void rbf_pid_apply_default_limits(RBF_PID_Handle *pid)
{
    pid->min_KP = PID_MIN_KP;
    pid->max_KP = PID_MAX_KP;
    pid->min_KI = PID_MIN_KI;
    pid->max_KI = PID_MAX_KI;
    pid->min_KD = PID_MIN_KD;
    pid->max_KD = PID_MAX_KD;
}

static void rbf_pid_apply_default_learning_rates(RBF_PID_Handle *pid)
{
    pid->eta_w = HYD_DEFAULT_RBF_W_LEARNING_RATE;
    pid->eta_c = HYD_DEFAULT_RBF_C_LEARNING_RATE;
    pid->eta_b = HYD_DEFAULT_RBF_B_LEARNING_RATE;
    pid->eta_p = HYD_DEFAULT_PID_P_LEARNING_RATE;
    pid->eta_i = HYD_DEFAULT_PID_I_LEARNING_RATE;
    pid->eta_d = HYD_DEFAULT_PID_D_LEARNING_RATE;
}

static void rbf_pid_apply_default_gains(RBF_PID_Handle *pid)
{
    pid->KP = PID_MIN_KP;
    pid->KI = PID_MIN_KI;
    pid->KD = PID_MIN_KD;
}

/*
 * 网络初始化（v3.2 修正 F4）：
 * x[0] = u/flow_scale 为绝对输出归一化，典型工作带覆盖 [0,1]，
 * 中心均匀展开 c[i][0] = 0.2*i，宽度 0.4 保证邻域核响应充分；
 * x[1]=x[2]=y/pressure_scale 仍按保压典型带 0.55~1.05 展开。
 */
static void rbf_pid_init_network(RBF_PID_Handle *pid)
{
    int i, j;

    for (i = 0; i < RBF_HNUM; ++i) {
        pid->c[i][0] = 0.2f * (float)i;             /* u_n ∈ [0,1] 均匀覆盖 */
        pid->c[i][1] = 0.55f + 0.1f * (float)i;
        pid->c[i][2] = pid->c[i][1];
        pid->b_rbf[i] = 0.4f;
        pid->w[i] = 0.05f * ((float)i - 2.5f) / 2.5f;

        for (j = 0; j < RBF_INPUT_DIM; ++j) {
            pid->ci_1[i][j] = pid->c[i][j];
            pid->ci_2[i][j] = pid->c[i][j];
        }
        pid->bi_1[i] = pid->b_rbf[i];
        pid->bi_2[i] = pid->b_rbf[i];
        pid->w_1[i] = pid->w[i];
        pid->w_2[i] = pid->w[i];
    }
}

/* 网络参数钳位（P1-7：仅在初始化/复位/学习发生后调用） */
static float rbf_pid_sanitize_network_value(float value, float min_value,
                                            float max_value, float fallback,
                                            bool *changed)
{
    if (!isfinite(value) || value < min_value || value > max_value) {
        *changed = true;
    }
    return clamp_finite(min_value, value, max_value, fallback);
}

static bool rbf_pid_sanitize_network(RBF_PID_Handle *pid)
{
    int i, j;
    bool changed = false;

    for (i = 0; i < RBF_HNUM; ++i) {
        pid->w[i] = rbf_pid_sanitize_network_value(
            pid->w[i], -RBF_PID_WEIGHT_LIMIT, RBF_PID_WEIGHT_LIMIT, 0.0f, &changed);
        pid->w_1[i] = rbf_pid_sanitize_network_value(
            pid->w_1[i], -RBF_PID_WEIGHT_LIMIT, RBF_PID_WEIGHT_LIMIT, pid->w[i], &changed);
        pid->w_2[i] = rbf_pid_sanitize_network_value(
            pid->w_2[i], -RBF_PID_WEIGHT_LIMIT, RBF_PID_WEIGHT_LIMIT, pid->w_1[i], &changed);

        pid->b_rbf[i] = rbf_pid_sanitize_network_value(
            pid->b_rbf[i], 0.2f, 5.0f, 0.4f, &changed);
        pid->bi_1[i] = rbf_pid_sanitize_network_value(
            pid->bi_1[i], 0.2f, 5.0f, pid->b_rbf[i], &changed);
        pid->bi_2[i] = rbf_pid_sanitize_network_value(
            pid->bi_2[i], 0.2f, 5.0f, pid->bi_1[i], &changed);

        for (j = 0; j < RBF_INPUT_DIM; ++j) {
            pid->c[i][j] = rbf_pid_sanitize_network_value(
                pid->c[i][j], -2.0f, 2.0f, 0.0f, &changed);
            pid->ci_1[i][j] = rbf_pid_sanitize_network_value(
                pid->ci_1[i][j], -2.0f, 2.0f, pid->c[i][j], &changed);
            pid->ci_2[i][j] = rbf_pid_sanitize_network_value(
                pid->ci_2[i][j], -2.0f, 2.0f, pid->ci_1[i][j], &changed);
        }
    }

    return changed;
}

static void rbf_pid_sanitize_runtime_state(RBF_PID_Handle *pid)
{
    float output_min = rbf_pid_output_lower_bound(pid);
    float output_max = rbf_pid_output_upper_bound(pid);
    float history_limit = rbf_pid_max_flow_output(pid);

    pid->u_prev  = clamp_finite(output_min, pid->u_prev, output_max, 0.0f);
    pid->du_prev = clamp_finite(-history_limit, pid->du_prev, history_limit, 0.0f);

    pid->e_prev1 = finite_or_default(pid->e_prev1, 0.0f);
    pid->e_prev2 = finite_or_default(pid->e_prev2, 0.0f);
    pid->y_prev1 = finite_or_default(pid->y_prev1, 0.0f);
    pid->y_prev2 = finite_or_default(pid->y_prev2, 0.0f);
    pid->fLastActPress  = finite_or_default(pid->fLastActPress, 0.0f);
    pid->fLastActPress2 = finite_or_default(pid->fLastActPress2, 0.0f);
    pid->last_ref = finite_or_default(pid->last_ref, 0.0f);
    pid->feedforward_flow      = finite_or_default(pid->feedforward_flow, 0.0f);
    pid->feedforward_flow_prev = finite_or_default(pid->feedforward_flow_prev, 0.0f);
}

static void rbf_pid_enforce_control_mode(RBF_PID_Handle *pid)
{
    if (pid->control_mode == RBF_PID_CONTROL_MODE_PI) {
        pid->KD = 0.0f;
        pid->eta_d = 0.0f;
        pid->prev_d_term = 0.0f;
        /* P1-5：速度阻尼与控制模式无关，PI 模式下同样保留 */
    }
}

/* ==================== RBF 网络单步 ==================== */

/*
 * 返回 1 表示处于稳态死区（本拍冻结学习）。
 * 稳态死区 = 0.5% * 压力量程。
 */
static int rbf_pid_step_rbf_nn(RBF_PID_Handle *pid, float error)
{
    float h[RBF_HNUM];
    float flow_scale = rbf_pid_effective_flow_scale(pid);
    float pressure_scale = rbf_pid_effective_pressure_scale(pid);
    float x[RBF_INPUT_DIM];
    float y_n = pid->P_actual / pressure_scale;
    float y_hat_n = 0.0f;
    float jacobian_n = 0.0f;
    float error_rbf_n = 0.0f;
    float deadzone;
    float de;
    int i, j;
    bool learned = false;

    /* v3.2（F4）：x[0] 改为绝对输出归一化，激励充分，
       Jacobian 语义 = ∂P/∂u，与软限幅/增益补偿的 K 一致 */
    x[0] = pid->u_prev / flow_scale;
    x[1] = pid->y_prev1 / pressure_scale;
    x[2] = pid->y_prev2 / pressure_scale;

    for (i = 0; i < RBF_INPUT_DIM; ++i) {
        if (!isfinite(x[i])) x[i] = 0.0f;
    }
    memcpy(pid->last_rbf_input, x, sizeof(x));

    pid->Jacobian = 0.0f;

    for (i = 0; i < RBF_HNUM; ++i) {
        float norm_val = 0.0f;
        for (j = 0; j < RBF_INPUT_DIM; ++j) {
            float diff = x[j] - pid->c[i][j];
            norm_val += diff * diff;
        }
        h[i] = expf(-norm_val / (2.0f * pid->b_rbf[i] * pid->b_rbf[i]));
        y_hat_n += pid->w[i] * h[i];
        jacobian_n += pid->w[i] * h[i] * (pid->c[i][0] - x[0]) / (pid->b_rbf[i] * pid->b_rbf[i]);
    }

    /* v3.2：换算回物理量纲 dP/du = (pressure_scale/flow_scale) * dŷ_n/dx0
       [bar/(L/min)]，与 K = dP/dQ 同量纲 */
    pid->Jacobian = clampf(-5.0f, (pressure_scale / flow_scale) * jacobian_n, 50.0f);

    /* ---- Jacobian 符号监督（P1-3） ---- */
    if (pid->Jacobian < 0.0f) {
        pid->jac_neg_count++;
        if (pid->jac_neg_count >= RBF_PID_JACOBIAN_FAULT_COUNT) {
            pid->fault_flags |= RBF_PID_FAULT_JACOBIAN_SIGN;
        }
    } else {
        pid->jac_neg_count = 0;
        pid->fault_flags &= ~(uint32_t)RBF_PID_FAULT_JACOBIAN_SIGN;
    }

    /* ---- 稳态判定（冻结条件，P0-2） ---- */
    deadzone = RBF_PID_STEADY_DEADZONE_RATIO * pressure_scale;
    de = error - pid->e_prev1;

    {
        int is_steady = (fabsf(error) < deadzone) &&
                        (fabsf(de) < deadzone * RBF_PID_STEADY_DE_RATIO);

        if (!is_steady &&
            pid->learning_enabled &&
            !(pid->fault_flags & RBF_PID_FAULT_JACOBIAN_SIGN) &&
            !rbf_pid_same_direction_saturation(pid)) {

            error_rbf_n = y_n - y_hat_n;

            for (i = 0; i < RBF_HNUM; ++i) {
                float w_old = pid->w[i];
                float delta_w = pid->eta_w * error_rbf_n * h[i] +
                                pid->alpha * (pid->w[i] - pid->w_1[i]);
                float width = pid->b_rbf[i];
                float width_sq = width * width;
                float width_cu = width_sq * width;
                float norm_val = 0.0f;

                /* Step 1: 用更新前的中心计算 norm_val */
                for (j = 0; j < RBF_INPUT_DIM; ++j) {
                    float diff = x[j] - pid->c[i][j];
                    norm_val += diff * diff;
                }

                /* Step 2: 用 w_old 更新中心 */
                for (j = 0; j < RBF_INPUT_DIM; ++j) {
                    float delta_center = pid->eta_c * error_rbf_n * w_old * h[i] *
                                         (x[j] - pid->c[i][j]) / width_sq +
                                         pid->alpha * (pid->ci_1[i][j] - pid->ci_2[i][j]);
                    pid->c[i][j] = clamp_finite(-2.0f, pid->c[i][j] + delta_center, 2.0f, pid->c[i][j]);
                }

                /* Step 3: 用旧中心 norm_val 更新宽度 */
                pid->b_rbf[i] = clamp_finite(0.2f,
                    pid->b_rbf[i] + pid->eta_b * error_rbf_n * w_old * h[i] * norm_val / width_cu +
                    pid->alpha * (pid->bi_1[i] - pid->bi_2[i]),
                    5.0f, pid->b_rbf[i]);

                /* Step 4: 权重更新并钳位 */
                pid->w[i] = clamp_finite(-RBF_PID_WEIGHT_LIMIT,
                    pid->w[i] + delta_w, RBF_PID_WEIGHT_LIMIT, pid->w[i]);
            }

            /* 历史快照滚动 */
            for (i = 0; i < RBF_HNUM; ++i) {
                for (j = 0; j < RBF_INPUT_DIM; ++j) {
                    pid->ci_2[i][j] = pid->ci_1[i][j];
                    pid->ci_1[i][j] = pid->c[i][j];
                }
                pid->bi_2[i] = pid->bi_1[i];
                pid->bi_1[i] = pid->b_rbf[i];
                pid->w_2[i] = pid->w_1[i];
                pid->w_1[i] = pid->w[i];
            }

            learned = true;
        }

        if (learned) {
            (void)rbf_pid_sanitize_network(pid);  /* P1-7：仅学习后钳位 */
        }

        return is_steady;
    }
}

/* ==================== PID 增益在线整定 ==================== */

/*
 * P1-1/P1-2：梯度全部使用压力量程归一化量 e_n/de_n/dde_n，
 * 并施加单步步长限幅。v3.2：Jacobian = ∂P/∂u [bar/(L/min)]。
 */
static void rbf_pid_step_adaptive_gains(RBF_PID_Handle *pid, float error)
{
    float pressure_scale;
    float de, dde;
    float e_n, de_n, dde_n;
    float grad, delta;

    if (!pid->learning_enabled) return;
    if (pid->fault_flags & (RBF_PID_FAULT_JACOBIAN_SIGN |
                            RBF_PID_FAULT_NETWORK)) return;
    if (rbf_pid_same_direction_saturation(pid)) return;

    pressure_scale = rbf_pid_effective_pressure_scale(pid);
    de  = error - pid->e_prev1;
    dde = de - (pid->e_prev1 - pid->e_prev2);
    e_n   = error / pressure_scale;
    de_n  = de / pressure_scale;
    dde_n = dde / pressure_scale;

    /* ---- Kp：ΔKp = ηp * e_n * J * de_n ---- */
    grad = pid->eta_p * e_n * pid->Jacobian * de_n;
    grad = clampf(-RBF_PID_KP_STEP_LIMIT, grad, RBF_PID_KP_STEP_LIMIT);
    pid->KP = clampf(pid->min_KP, pid->KP + grad, pid->max_KP);

    /* ---- Ki：ΔKi = ηi * e_n * J * e_n − λ(KI−center) ---- */
    grad = pid->eta_i * e_n * pid->Jacobian * e_n - LAMBDA_KI * (pid->KI - KI_CENTER);
    delta = clampf(-RBF_PID_KI_STEP_LIMIT, grad, RBF_PID_KI_STEP_LIMIT);
    pid->KI = clampf(pid->min_KI, pid->KI + delta, pid->max_KI);

    /* ---- Kd：带强制唤醒；PI 模式下恒为 0 ---- */
    if (pid->control_mode == RBF_PID_CONTROL_MODE_PI) {
        pid->KD = 0.0f;
    } else {
        int error_diverging = (fabsf(e_n) > fabsf(pid->e_prev1 / pressure_scale)) &&
                              (fabsf(e_n) > 0.01f);
        int kd_at_floor = (pid->KD <= pid->min_KD * 1.1f);

        if (error_diverging && kd_at_floor) {
            delta = fmaxf(ETA_KD_BOOST * fabsf(de_n) * sign(pid->Jacobian), 0.0f);
        } else {
            grad = pid->eta_d * e_n * pid->Jacobian * fabsf(dde_n);
            if (error * de < 0.0f) {
                delta = fmaxf(grad, 0.0f);
            } else {
                delta = grad;
            }
        }

        delta = clampf(-RBF_PID_KD_STEP_LIMIT, delta, RBF_PID_KD_STEP_LIMIT);
        pid->KD = clampf(pid->min_KD, pid->KD + delta, pid->max_KD);
    }
}

/* ==================== 增量输出计算 ==================== */

/*
 * v3.2（F2）：微分项统一作用于滤波后测量（二阶差分 Δ²y + 一阶低通），
 * 输出取 −Kd·d_term：
 *   - 设定值恒定时与旧的 +Kd·Δ²e 数学等价；
 *   - 设定值阶跃时 Δ²y = 0，不再产生微分踢；
 *   - 与外层非 RBF 分支的 −kd·filteredPressureRate 语义一致。
 */
static void rbf_pid_step_incremental_output(RBF_PID_Handle *pid, float error)
{
    float output_min = rbf_pid_output_lower_bound(pid);
    float output_max = rbf_pid_effective_output_max(pid);
    float d_term;
    float du;

    /* ---- 微分项：测量二阶差分 + 一阶低通（P0-5 滤波保留） ---- */
    if (pid->control_mode == RBF_PID_CONTROL_MODE_PI) {
        pid->prev_d_term = 0.0f;
        d_term = 0.0f;
    } else {
        float raw_d_term = (pid->P_actual - 2.0f * pid->y_prev1 + pid->y_prev2);
        float flt_alpha = (float)HYD_THRESH_RBF_DERIV_FILTER_ALPHA;
        d_term = flt_alpha * raw_d_term + (1.0f - flt_alpha) * pid->prev_d_term;
        pid->prev_d_term = d_term;
    }

    {
        float interf_term = pid->KI * error;
        interf_term = clampf(-RBF_PID_INCR_I_LIMIT, interf_term, RBF_PID_INCR_I_LIMIT);
        du = pid->KP * (error - pid->e_prev1) + interf_term - pid->KD * d_term;
    }

    /* ---- 压力速度阻尼（P1-5，方向化门控） ----
     * f_velfb = -gain * ΔP_actual，仅当压力正在上升时生效；
     * 卸压段（ΔP <= 阈值）：阻尼完全旁路。 */
    /* ---- 前馈并入（v3.1 修复 #3 / v3.2 F1 链路接通） ----
     * ff_delta = ff_k − ff_{k−1}：外层每拍写入 feedforward_flow 后生效；
     * 前馈恒定时 ff_delta = 0，不改变既有行为。 */
    {
        float f_delta_press = pid->P_actual - pid->fLastActPress;
        float f_velfb = 0.0f;
        float ff_delta = 0.0f;

        if (pid->pressure_vel_damp_enabled && isfinite(f_delta_press) &&
            (f_delta_press > RBF_PID_VEL_DAMP_RISE_THRESH)) {
            f_velfb = -pid->pressure_vel_damp_gain * f_delta_press;
            f_velfb = clampf(-RBF_PID_VEL_DAMP_LIMIT, f_velfb, RBF_PID_VEL_DAMP_LIMIT);
        }
        /* f_delta_press <= 阈值时 f_velfb 保持 0：卸压方向零干预 */

        if (isfinite(pid->feedforward_flow) && isfinite(pid->feedforward_flow_prev)) {
            ff_delta = pid->feedforward_flow - pid->feedforward_flow_prev;
        }
        pid->feedforward_flow_prev = pid->feedforward_flow;

        pid->du = !isfinite(du) ? 0.0f : du;
        pid->Output = pid->u_prev + pid->du + f_velfb + ff_delta;
    }

    pid->Output = clampf(output_min, pid->Output, output_max);
    pid->output_saturated = (pid->Output <= output_min + 1.0e-6f) ||
                            (pid->Output >= output_max - 1.0e-6f);

    /* 低压安全：设定与反馈均进入低压区时强制零输出 */
    if (pid->P_set < 0.1f && pid->P_actual < 0.5f) {
        pid->Output = 0.0f;
        pid->output_saturated = false;
    }

    pid->fLastActPress2 = pid->fLastActPress;
    pid->fLastActPress = pid->P_actual;
    pid->last_ref = pid->P_set;
}

/* ==================== 外层稳态判定（Status 用） ==================== */
static void rbf_pid_step_steady_state(RBF_PID_Handle *pid)
{
    int n_steady = (int)(RBF_PID_STEADY_T_LIMIT / RBF_PID_FIXED_SAMPLING_PERIOD);
    float error = pid->P_set - pid->P_actual;
    bool condition1 = fabsf(error) <= RBF_PID_STEADY_E_LIMIT;   /* [bar] */
    bool condition2 = fabsf(pid->du) <= RBF_PID_STEADY_DU_LIMIT; /* [L/min] */

    if (n_steady < 5) n_steady = 5;

    if (condition1 && condition2) {
        if (pid->steady_count < n_steady) pid->steady_count++;
    } else {
        pid->steady_count = 0;
    }

    pid->steady_state = condition1 && condition2 &&
                        (pid->steady_count >= n_steady) &&
                        (fabsf(pid->P_set) > RBF_PID_STEADY_MIN_SETPOINT);
}

/* ==================== 公开 API ==================== */

void RBF_PID_Init(RBF_PID_Handle *pid, float sampling_period,
                  float max_flow_lmin, float flow_rate_limit_pct)
{
    if (pid == NULL) return;

    memset(pid, 0, sizeof(*pid));
    (void)sampling_period;
    pid->sampling_period = RBF_PID_FIXED_SAMPLING_PERIOD;
    pid->fMaxFlow = clamp_positive_or_default(max_flow_lmin, 0.0f);
    pid->fFlowRateLimit = clampf(0.0f, flow_rate_limit_pct, 1.0f);
    pid->output_min_flow = MIN_OUTPUT;
    pid->output_max_flow = 0.0f;
    pid->pressure_normalization_scale = MAX_PRESSURE;
    pid->flow_normalization_scale = (pid->fMaxFlow > 0.0f) ? pid->fMaxFlow : 90.0f;
    pid->output_saturated = false;
    memset(pid->last_rbf_input, 0, sizeof(pid->last_rbf_input));

    pid->Status = 1;
    pid->TuneResult = 66;
    pid->alpha = 0.05f;
    pid->pressure_vel_damp_gain = RBF_PID_DEFAULT_PRESSURE_VEL_DAMP_GAIN;
    pid->pressure_vel_damp_enabled = true;
    pid->learning_enabled = true;
    pid->external_deadband_enabled = false;   /* v3.2：默认内层死区生效，向后兼容 */
    pid->jac_neg_count = 0;
    pid->fault_flags = RBF_PID_FAULT_NONE;
    pid->control_mode = RBF_PID_CONTROL_MODE_PID;
    pid->feedforward_flow = 0.0f;       /* v3.1：前馈状态显式清零 */
    pid->feedforward_flow_prev = 0.0f;
    pid->ksys_valid = false;

    rbf_pid_apply_default_limits(pid);
    rbf_pid_apply_default_learning_rates(pid);
    rbf_pid_apply_default_gains(pid);
    pid->pid_mode_kd = pid->KD;
    pid->pid_mode_eta_d = pid->eta_d;

    rbf_pid_init_network(pid);
    (void)rbf_pid_sanitize_network(pid);
}

static float rbf_pid_hold_safe_output(RBF_PID_Handle *pid)
{
    float output_min;
    float output_max;
    float safe_output;

    pid->sampling_period = RBF_PID_FIXED_SAMPLING_PERIOD;
    rbf_pid_sanitize_runtime_state(pid);
    output_min = rbf_pid_output_lower_bound(pid);
    output_max = rbf_pid_effective_output_max(pid);
    safe_output = isfinite(pid->Output) ? pid->Output : pid->u_prev;
    if (!isfinite(safe_output)) safe_output = 0.0f;
    pid->Output = clampf(output_min, safe_output, output_max);
    pid->du = 0.0f;
    pid->output_saturated = (pid->Output <= output_min + 1.0e-6f) ||
                            (pid->Output >= output_max - 1.0e-6f);
    return pid->Output;
}

float RBF_PID_Update(RBF_PID_Handle *pid, float setpoint, float feedback)
{
    float raw_error;
    float error;
    int is_steady;
    bool network_changed;

    if (pid == NULL) return 0.0f;

    pid->sampling_period = RBF_PID_FIXED_SAMPLING_PERIOD;
    if (!isfinite(setpoint) || !isfinite(feedback)) {
        pid->fault_flags |= RBF_PID_FAULT_INPUT_INVALID;
        return rbf_pid_hold_safe_output(pid);
    }

    pid->fault_flags &= ~(uint32_t)RBF_PID_FAULT_INPUT_INVALID;
    pid->P_set = setpoint;
    pid->P_actual = feedback;

    rbf_pid_sanitize_runtime_state(pid);
    rbf_pid_enforce_control_mode(pid);
    network_changed = rbf_pid_sanitize_network(pid);
    if (network_changed) {
        pid->fault_flags |= RBF_PID_FAULT_NETWORK;
    } else {
        pid->fault_flags &= ~(uint32_t)RBF_PID_FAULT_NETWORK;
    }

    /*
     * v3.2（F3）：双重死区治理。
     * external_deadband_enabled = true 时内层误差死区旁路，
     * 由外层（段配置死区 HYD_ApplyPressureDeadband）做唯一一级死区；
     * 默认 false 时保留内层死区（独立使用场景向后兼容）。
     */
    raw_error = pid->P_set - pid->P_actual;
    if (pid->external_deadband_enabled) {
        error = raw_error;
    } else {
        error = rbf_pid_apply_deadband(raw_error, rbf_pid_error_deadband(pid));
    }
    pid->Error = error;

    is_steady = rbf_pid_step_rbf_nn(pid, error);
    rbf_pid_step_incremental_output(pid, error);

    /* 历史滚动 */
    pid->y_prev2 = pid->y_prev1;
    pid->y_prev1 = pid->P_actual;
    pid->u_prev = pid->Output;
    pid->du_prev = pid->du;

    /* 增益整定（冻结/开关/故障判定在函数内部） */
    rbf_pid_step_adaptive_gains(pid, error);

    pid->e_prev2 = pid->e_prev1;
    pid->e_prev1 = error;

    rbf_pid_step_steady_state(pid);
    pid->Status = pid->steady_state ? 3 : 2;

    return pid->Output;
}

void RBF_PID_Reset(RBF_PID_Handle *pid)
{
    float sampling_period, max_flow, flow_limit;
    if (pid == NULL) return;

    sampling_period = pid->sampling_period;
    max_flow = pid->fMaxFlow;
    flow_limit = pid->fFlowRateLimit;
    RBF_PID_Init(pid, sampling_period, max_flow, flow_limit);
}

void RBF_PID_SoftReset(RBF_PID_Handle *pid)
{
    if (pid == NULL) return;

    /* 仅复位控制器运行状态；网络权值、自适应增益、参数窗口与配置全部保留 */
    pid->Output = 0.0f;
    pid->du = 0.0f;
    pid->du_prev = 0.0f;
    pid->u_prev = 0.0f;
    pid->e_prev1 = 0.0f;
    pid->e_prev2 = 0.0f;
    pid->y_prev1 = 0.0f;
    pid->y_prev2 = 0.0f;
    pid->fLastActPress = 0.0f;
    pid->fLastActPress2 = 0.0f;
    pid->last_ref = 0.0f;
    pid->prev_d_term = 0.0f;
    pid->steady_count = 0;
    pid->steady_state = false;
    pid->output_saturated = false;
    memset(pid->last_rbf_input, 0, sizeof(pid->last_rbf_input));
    pid->jac_neg_count = 0;
    pid->fault_flags = RBF_PID_FAULT_NONE;
    pid->Error = 0.0f;
    pid->Jacobian = 0.0f;
    pid->feedforward_flow = 0.0f;       /* v3.1：前馈状态清零，外层 Synchronize 后重新种子 */
    pid->feedforward_flow_prev = 0.0f;
    pid->Status = 1;
    pid->TuneResult = 0;
}

void RBF_PID_SetParamLimits(RBF_PID_Handle *pid, float min_kp, float max_kp,
                            float min_ki, float max_ki, float min_kd, float max_kd)
{
    if (pid == NULL) return;

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

void RBF_PID_SetLearningRates(RBF_PID_Handle *pid, float eta_w, float eta_c,
                              float eta_b, float eta_p, float eta_i, float eta_d)
{
    if (pid == NULL) return;

    pid->eta_w = clamp_finite(0.0f, eta_w, 10.0f, 0.0f);
    pid->eta_c = clamp_finite(0.0f, eta_c, 10.0f, 0.0f);
    pid->eta_b = clamp_finite(0.0f, eta_b, 10.0f, 0.0f);
    pid->eta_p = clamp_finite(0.0f, eta_p, 10.0f, 0.0f);
    pid->eta_i = clamp_finite(0.0f, eta_i, 10.0f, 0.0f);
    pid->pid_mode_eta_d = clamp_finite(0.0f, eta_d, 10.0f, 0.0f);
    pid->eta_d = pid->pid_mode_eta_d;
    rbf_pid_enforce_control_mode(pid);
}

void RBF_PID_SetPressureNormalization(RBF_PID_Handle *pid, float scale)
{
    if (pid == NULL) return;
    pid->pressure_normalization_scale =
        (scale > 0.0f && isfinite(scale)) ? scale : MAX_PRESSURE;
}

void RBF_PID_SetFlowNormalization(RBF_PID_Handle *pid, float scale)
{
    if (pid == NULL) return;
    pid->flow_normalization_scale = clamp_positive_or_default(
        scale, (pid->fMaxFlow > 0.0f) ? pid->fMaxFlow : 90.0f);
}

void RBF_PID_SetGainCompensation(RBF_PID_Handle *pid, float systemGain)
{
    if (pid == NULL) return;
    pid->K = (systemGain > 0.0f && isfinite(systemGain)) ? systemGain : 0.0f;
    pid->ksys_valid = (pid->K > 0.0f);
}

void RBF_PID_SetPressureVelocityDamping(RBF_PID_Handle *pid, float gain)
{
    if (pid == NULL) return;
    if (gain > 0.0f && isfinite(gain)) {
        pid->pressure_vel_damp_gain = clampf(0.0f, gain, 10.0f);
        pid->pressure_vel_damp_enabled = true;
    } else {
        pid->pressure_vel_damp_enabled = false;  /* gain <= 0 关闭阻尼 */
    }
}

void RBF_PID_SetAdaptiveEnabled(RBF_PID_Handle *pid, bool enabled)
{
    if (pid == NULL) return;
    pid->learning_enabled = enabled;
}

void RBF_PID_SetControlMode(RBF_PID_Handle *pid, RBF_PID_ControlMode mode)
{
    RBF_PID_ControlMode requestedMode;
    if (pid == NULL) return;

    requestedMode = (mode == RBF_PID_CONTROL_MODE_PI) ?
                    RBF_PID_CONTROL_MODE_PI : RBF_PID_CONTROL_MODE_PID;

    if (pid->control_mode == requestedMode) {
        rbf_pid_enforce_control_mode(pid);
        return;
    }

    /* PID -> PI：保存微分相关状态 */
    if (pid->control_mode == RBF_PID_CONTROL_MODE_PID &&
        requestedMode == RBF_PID_CONTROL_MODE_PI) {
        pid->pid_mode_kd = pid->KD;
        pid->pid_mode_eta_d = pid->eta_d;
    }

    pid->control_mode = requestedMode;

    if (requestedMode == RBF_PID_CONTROL_MODE_PID) {
        /* PI -> PID：恢复微分状态 */
        pid->KD = clampf(pid->min_KD, pid->pid_mode_kd, pid->max_KD);
        pid->eta_d = pid->pid_mode_eta_d;
        return;
    }

    rbf_pid_enforce_control_mode(pid);
}

void RBF_PID_SetExternalDeadbandEnabled(RBF_PID_Handle *pid, bool enabled)
{
    if (pid == NULL) return;
    pid->external_deadband_enabled = enabled;
}
