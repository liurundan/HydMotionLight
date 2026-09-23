/* rbf_preset_configs.c - RBF-PID预设参数推导
 *
 * 将147个RBF内部参数简化为4套应用场景预设：
 *   1. THIN_WALL     - 薄壁件射胶（激进学习，快速适应熔体剪切稀化）
 *   2. THICK_WALL    - 厚壁件保压（保守学习，平稳跟踪冷却收缩）
 *   3. STANDARD      - 标准通用（中庸配置，平衡响应与稳定性）
 *   4. PLASTICATION  - 储料背压（批次自学习，应对MFI差异）
 *
 * 版本: v14 (2026-09-22)
 * 作者: Claude Opus 4.8
 */

#include "rbf_pid.h"
#include "common_types.h"
#include "hyd_config.h"
#include <math.h>
#include <string.h>

/* 预设参数表：学习率和增益范围 */
typedef struct {
    float eta_w, eta_c, eta_b;       /* 网络学习率 */
    float eta_p, eta_i, eta_d;       /* PID学习率 */
    float min_KP, max_KP;            /* Kp范围 */
    float min_KI, max_KI;            /* Ki范围 */
    float min_KD, max_KD;            /* Kd范围 */
    float error_deadband;            /* 误差死区 */
    bool disable_ff_accel;           /* 禁用前馈加速度 */
} RBF_PresetParams;

/* 薄壁件射胶预设 */
static const RBF_PresetParams PRESET_THIN_WALL = {
    .eta_w = 0.20f, .eta_c = 0.15f, .eta_b = 0.15f,
    .eta_p = 0.15f, .eta_i = 0.10f, .eta_d = 0.08f,
    .min_KP = 0.01f, .max_KP = 0.50f,
    .min_KI = 0.001f, .max_KI = 0.20f,
    .min_KD = 0.0f, .max_KD = 0.05f,
    .error_deadband = 0.005f,
    .disable_ff_accel = false
};

/* 厚壁件保压预设 */
static const RBF_PresetParams PRESET_THICK_WALL = {
    .eta_w = 0.10f, .eta_c = 0.08f, .eta_b = 0.08f,
    .eta_p = 0.05f, .eta_i = 0.03f, .eta_d = 0.02f,
    .min_KP = 0.01f, .max_KP = 0.30f,
    .min_KI = 0.001f, .max_KI = 0.10f,
    .min_KD = 0.0f, .max_KD = 0.03f,
    .error_deadband = 0.01f,
    .disable_ff_accel = true
};

/* 标准配置预设 */
static const RBF_PresetParams PRESET_STANDARD = {
    .eta_w = 0.15f, .eta_c = 0.10f, .eta_b = 0.10f,
    .eta_p = 0.10f, .eta_i = 0.05f, .eta_d = 0.05f,
    .min_KP = 0.01f, .max_KP = 0.40f,
    .min_KI = 0.001f, .max_KI = 0.15f,
    .min_KD = 0.0f, .max_KD = 0.04f,
    .error_deadband = 0.005f,
    .disable_ff_accel = false
};

/* 储料背压预设 */
static const RBF_PresetParams PRESET_PLASTICATION = {
    .eta_w = 0.18f, .eta_c = 0.12f, .eta_b = 0.12f,
    .eta_p = 0.12f, .eta_i = 0.08f, .eta_d = 0.05f,
    .min_KP = 0.01f, .max_KP = 0.45f,
    .min_KI = 0.001f, .max_KI = 0.18f,
    .min_KD = 0.0f, .max_KD = 0.05f,
    .error_deadband = 0.008f,
    .disable_ff_accel = false
};

/* 辅助函数：限制浮点数范围 */
static inline float clampf_local(float min, float value, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

/**
 * @brief 从预设模式推导RBF-PID完整配置
 *
 * @param preset           预设模式枚举
 * @param systemGain       系统增益 K [bar/(L/min)]
 * @param plantTau         对象时间常数 τ [s]
 * @param maxFlow          最大流量 [L/min]
 * @param aggressiveness   响应激进度 [0.5-2.0]，默认1.0
 * @param pid              输出：RBF_PID_Handle指针（已初始化，仅覆盖学习参数）
 */
void HYD_DeriveRbfConfigFromPreset(
    HYD_RbfPreset preset,
    HYD_REAL systemGain,
    HYD_REAL plantTau,
    HYD_REAL maxFlow,
    HYD_REAL aggressiveness,
    RBF_PID_Handle* pid)
{
    const RBF_PresetParams* params;
    float eta_scale;
    float K_crit_approx;

    if (pid == NULL) {
        return;
    }

    /* Step 1: 选择基础预设 */
    switch (preset) {
    case HYD_RBF_PRESET_THIN_WALL:
        params = &PRESET_THIN_WALL;
        break;
    case HYD_RBF_PRESET_THICK_WALL:
        params = &PRESET_THICK_WALL;
        break;
    case HYD_RBF_PRESET_PLASTICATION:
        params = &PRESET_PLASTICATION;
        break;
    case HYD_RBF_PRESET_STANDARD:
    case HYD_RBF_PRESET_CUSTOM:
    default:
        if (preset == HYD_RBF_PRESET_CUSTOM) {
            return;  /* CUSTOM模式不推导，保持用户手动设置 */
        }
        params = &PRESET_STANDARD;
        break;
    }

    /* Step 2: 应用激进度缩放 */
    eta_scale = clampf_local(0.5f, aggressiveness, 2.0f);

    pid->eta_w = clampf_local(0.005f, params->eta_w * eta_scale, 0.50f);
    pid->eta_c = clampf_local(0.005f, params->eta_c * eta_scale, 0.40f);
    pid->eta_b = clampf_local(0.005f, params->eta_b * eta_scale, 0.40f);
    pid->eta_p = clampf_local(0.01f, params->eta_p * eta_scale, 0.30f);
    pid->eta_i = clampf_local(0.005f, params->eta_i * eta_scale, 0.20f);
    pid->eta_d = clampf_local(0.001f, params->eta_d * eta_scale, 0.15f);

    /* Step 3: 根据对象时间常数调整增益范围 */
    pid->min_KP = params->min_KP;
    pid->max_KP = params->max_KP;
    pid->min_KI = params->min_KI;
    pid->max_KI = params->max_KI;
    pid->min_KD = params->min_KD;
    pid->max_KD = params->max_KD;

    if (plantTau > 0.0f && systemGain > HYD_MIN_SAFE_SYSTEM_GAIN) {
        /* 一阶系统临界增益: K_crit ≈ τ / (K * T_sample)
         * 增益上界取临界增益的50%（保守） */
        K_crit_approx = plantTau / (systemGain * HYD_DEFAULT_RBF_PID_SAMPLING_PERIOD);

        pid->max_KP = fminf(pid->max_KP, 0.5f * K_crit_approx);
        pid->max_KI = fminf(pid->max_KI, 0.3f * K_crit_approx);
        pid->max_KD = fminf(pid->max_KD, 0.1f * K_crit_approx);
    }

    /* Step 4: 填充系统参数 */
    pid->K = (float)systemGain;
    pid->process_time_constant_s = (float)plantTau;
    /* max_output_lmin 由外层在 RBF_PID_Init 中设置 */

    /* Step 5: 前馈加速度配置 */
    pid->pressure_accel_ff_enabled = !params->disable_ff_accel;
    pid->pressure_accel_ff_requested = pid->pressure_accel_ff_enabled;
}

