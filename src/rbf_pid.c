/**
 * @file rbf_pid.c
 * @brief RBF神经网络自适应PID控制器实现
 * @note 完全遵循原ST代码逻辑，使用定点化友好的浮点运算
 */

#include "rbf_pid.h"
#include "hyd_config.h"
#include <math.h>
#include <string.h>

/* 内部辅助宏 */
#define LIMIT(min, x, max)  ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))
#define ABS(x)              ((x) >= 0 ? (x) : -(x))
#define MAX(a,b)            ((a) > (b) ? (a) : (b))
#define MIN(a,b)            ((a) < (b) ? (a) : (b))

/* 简单的线性同余随机数生成器(用于初始化中心) */
static uint32_t lcg_rand(uint32_t *seed) {
    *seed = *seed * 1664525UL + 1013904223UL;
    return *seed;
}

/* 生成[0,1)范围浮点数 */
static float lcg_rand_float(uint32_t *seed) {
    uint32_t val = lcg_rand(seed);
    return (float)val / 4294967295.0f;
}

/* 初始化RBF网络参数(中心、宽度、权重) */
static void rbf_init_network(RBF_PID_Handle *pid) {
    uint32_t seed = pid->network_seed;
    int i, j;

    for (i = 0; i < RBF_HNUM; i++) {
        for (j = 0; j < RBF_INPUT_DIM; j++) {
            float rand_val = lcg_rand_float(&seed);
            if (j == 0) {
                pid->c[i][j] = rand_val * 2.0f - 1.0f;
            } else {
                pid->c[i][j] = rand_val;
            }
        }
        pid->b_rbf[i] = 1.5f;
        pid->w[i] = 0.3f + lcg_rand_float(&seed) * 0.4f;
    }

    /* 复制到历史数组 */
    memcpy(pid->ci_1, pid->c, sizeof(pid->c));
    memcpy(pid->ci_2, pid->c, sizeof(pid->c));
    memcpy(pid->ci_3, pid->c, sizeof(pid->c));
    memcpy(pid->bi_1, pid->b_rbf, sizeof(pid->b_rbf));
    memcpy(pid->bi_2, pid->b_rbf, sizeof(pid->b_rbf));
    memcpy(pid->bi_3, pid->b_rbf, sizeof(pid->b_rbf));
    memcpy(pid->w_1, pid->w, sizeof(pid->w));
    memcpy(pid->w_2, pid->w, sizeof(pid->w));
    memcpy(pid->w_3, pid->w, sizeof(pid->w));
}

/* 初始化控制器历史状态 */
static void controller_reset_state(RBF_PID_Handle *pid) {
    pid->u_prev = 0.0f;
    pid->e_prev1 = 0.0f;
    pid->e_prev2 = 0.0f;
    pid->du_prev = 0.0f;
    pid->du = 0.0f;
    pid->y_prev1 = 0.0f;
    pid->Setpoint = 0.0f;
    pid->Feedback = 0.0f;
    pid->Error = 0.0f;
    pid->delta_temp_prev = 0.0f;
    pid->fLastActPress = 0.0f;
    pid->fLastActPress2 = 0.0f;
    pid->fUffAcc = 0.0f;
    pid->last_ref = 0.0f;
}

void RBF_PID_Init(RBF_PID_Handle *pid, float sampling_period,
                  float max_flow_lmin, float flow_rate_limit_pct) {
    /* 清空整个结构体为0 */
    memset(pid, 0, sizeof(RBF_PID_Handle));

    pid->pressure_normalization_scale = 0.0f;  /* 0 -> fall back to MAX_PRESSURE */

    /* 配置基本参数 */
    pid->sampling_period = sampling_period;
    pid->fMaxFlow = max_flow_lmin;
    pid->fFlowRateLimit = flow_rate_limit_pct;

    /* 默认学习率 (与原ST一致) */
    pid->eta_w = 0.25f;
    pid->eta_c = 0.25f;
    pid->eta_b = 0.25f;
    pid->eta_p = 0.25f;
    pid->eta_i = 0.25f;
    pid->eta_d = 0.25f;

    /* 默认网络种子 */
    pid->network_seed = 12345UL;

    /* 动量因子 */
    pid->alpha = 0.05f;
    pid->belte = 0.01f;

    /* PID初始参数（取新默认窗的合理工作点）*/
//    pid->KP = 0.8f;   /* mid of [0.5, 1.2] */
//    pid->KI = 0.020f; /* near typical hydraulic-loop integral value */
//    pid->KD = 1.0f;   /* mid of [0.5, 2.0] */

    pid->KP = 0.03f;   /* mid of [0.5, 1.2] */
    pid->KI = 0.02f; /* near typical hydraulic-loop integral value */
    pid->KD = 0.03f;   /* mid of [0.5, 2.0] */

    pid->min_KP = PID_MIN_KP;
    pid->max_KP = PID_MAX_KP;
    pid->min_KI = PID_MIN_KI;
    pid->max_KI = PID_MAX_KI;
    pid->min_KD = PID_MIN_KD;
    pid->max_KD = PID_MAX_KD;

    /* 前馈参数 */
    pid->EnableFF = true;
    pid->fKff_a_pos = 0.7f;
    pid->fKff_a_neg = 0.32f;
    pid->fKSetpoint_pos = 0.3f;
    pid->fKSetpoint_neg = 0.1f;
    pid->fBaseBias = 0.00001f;

    /* 初始化网络与历史状态 */
    rbf_init_network(pid);
    controller_reset_state(pid);

    /* 其他标志 */
    pid->FirstScan = true;
    pid->Status = 0;
    pid->TuneResult = 0;
}

void RBF_PID_Reset(RBF_PID_Handle *pid) {
    /* 重新初始化网络参数(与FirstScan相同) */
    rbf_init_network(pid);
    controller_reset_state(pid);

    /* 重置PID参数为初始值 */
    pid->KP = 0.8f;
    pid->KI = 0.020f;
    pid->KD = 1.0f;

    pid->FirstScan = false;
    pid->Status = 0;
}

/* 更新RBF网络参数(带动量项) */
static void rbf_update_network(RBF_PID_Handle *pid, float x_norm[RBF_INPUT_DIM],
                               const float h[RBF_HNUM], float error_rbf,
                               float eta_scale) {
    int i, j;
    float norm_val, b_delta, c_delta;
    float eta_w_eff = pid->eta_w * eta_scale;
    float eta_c_eff = pid->eta_c * eta_scale;
    float eta_b_eff = pid->eta_b * eta_scale;

    /* 更新权重 w (动量 + 学习率) */
    for (i = 0; i < RBF_HNUM; i++) {
        float delta_w = eta_w_eff * error_rbf * h[i];
        pid->w[i] = pid->w_1[i] + delta_w
                    + pid->alpha * (pid->w_1[i] - pid->w_2[i])
                    + pid->belte * (pid->w_2[i] - pid->w_3[i]);
    }

    /* 更新宽度和中心 */
    for (i = 0; i < RBF_HNUM; i++) {
        /* 计算范数平方 */
        norm_val = 0.0f;
        for (j = 0; j < RBF_INPUT_DIM; j++) {
            float diff = x_norm[j] - pid->c[i][j];
            norm_val += diff * diff;
        }

        /* 宽度更新 */
        b_delta = eta_b_eff * error_rbf * pid->w[i] * h[i] * norm_val
                  / (pid->b_rbf[i] * pid->b_rbf[i] * pid->b_rbf[i]);
        pid->b_rbf[i] = pid->bi_1[i] + b_delta
                        + pid->alpha * (pid->bi_1[i] - pid->bi_2[i])
                        + pid->belte * (pid->bi_2[i] - pid->bi_3[i]);
        /* 宽度限幅(原ST未限幅，但建议加一个下界防止除零) */
        if (pid->b_rbf[i] < 0.1f) pid->b_rbf[i] = 0.1f;

        /* 中心更新 */
        for (j = 0; j < RBF_INPUT_DIM; j++) {
            c_delta = eta_c_eff * error_rbf * pid->w[i] * h[i]
                      * (x_norm[j] - pid->c[i][j]) / (pid->b_rbf[i] * pid->b_rbf[i]);
            pid->c[i][j] = pid->ci_1[i][j] + c_delta
                           + pid->alpha * (pid->ci_1[i][j] - pid->ci_2[i][j])
                           + pid->belte * (pid->ci_2[i][j] - pid->ci_3[i][j]);
            /* 中心限幅 (原ST限幅到[-1,1]) */
            pid->c[i][j] = LIMIT(-1.0f, pid->c[i][j], 1.0f);
        }
    }

    /* 保存历史数据到下次迭代 */
    memcpy(pid->ci_3, pid->ci_2, sizeof(pid->ci_2));
    memcpy(pid->ci_2, pid->ci_1, sizeof(pid->ci_1));
    memcpy(pid->ci_1, pid->c, sizeof(pid->c));

    memcpy(pid->bi_3, pid->bi_2, sizeof(pid->bi_2));
    memcpy(pid->bi_2, pid->bi_1, sizeof(pid->bi_1));
    memcpy(pid->bi_1, pid->b_rbf, sizeof(pid->b_rbf));

    memcpy(pid->w_3, pid->w_2, sizeof(pid->w_2));
    memcpy(pid->w_2, pid->w_1, sizeof(pid->w_1));
    memcpy(pid->w_1, pid->w, sizeof(pid->w));
}

float RBF_PID_Update(RBF_PID_Handle *pid, float setpoint, float feedback) {
    float error;
    float x_normalized[3];
    float y_hat, sum_jacobian, error_rbf;
    float du, raw_de, delta_temp;
    float fActPress, fDeltaPress, fddPress;
    float ref_change, ref_rate, dynamic_ff;
    float output_total;

    /* 使能检查 */
    if (!pid->enable) {
        pid->Status = -1;
        return 0.0f;
    }

    /* 复位逻辑 (每次调用时检测) */
    if (pid->FirstScan || pid->Reset) {
        rbf_init_network(pid);
        controller_reset_state(pid);
        pid->KP = 0.8f;
        pid->KI = 0.020f;
        pid->KD = 1.0f;
        pid->Status = 0;
        pid->FirstScan = false;
        pid->Reset = false;   // 清除复位标志
    }

    /* 归一化设定值和反馈值 */
    {
        float scale = (pid->pressure_normalization_scale > 0.0f)
                      ? pid->pressure_normalization_scale
                      : MAX_PRESSURE;
        pid->Setpoint = setpoint / scale;
        pid->Feedback = feedback / scale;
    }
    /* Ts 直接使用 sampling_period (但本算法未显式使用，仅用于外部) */
    error = pid->Setpoint - pid->Feedback;
    pid->Error = error;

    /* ----- RBF网络前向计算 ----- */
    x_normalized[0] = pid->du_prev;      // 上一时刻控制增量
    x_normalized[1] = pid->Feedback;     // 当前反馈
    x_normalized[2] = pid->y_prev1;      // 上一时刻反馈

    /* 计算隐含层输出 h[i] */
    for (int i = 0; i < RBF_HNUM; i++) {
        float norm_sq = 0.0f;
        for (int j = 0; j < RBF_INPUT_DIM; j++) {
            float diff = x_normalized[j] - pid->c[i][j];
            norm_sq += diff * diff;
        }
        pid->h[i] = expf(-norm_sq / (2.0f * pid->b_rbf[i] * pid->b_rbf[i]));
    }

    /* 网络输出 y_hat */
    y_hat = 0.0f;
    for (int i = 0; i < RBF_HNUM; i++) {
        y_hat += pid->w[i] * pid->h[i];
    }
    pid->y_hat = y_hat;

    /* 计算Jacobian: ∂y_hat/∂du_prev */
    sum_jacobian = 0.0f;
    for (int i = 0; i < RBF_HNUM; i++) {
        sum_jacobian += pid->w[i] * pid->h[i] *
                        (pid->c[i][0] - x_normalized[0]) / (pid->b_rbf[i] * pid->b_rbf[i]);
    }
    pid->Jacobian = LIMIT(-1.0f, sum_jacobian, 1.0f);

    /* ----- 自适应学习率: 误差大时高学习率(快速跟踪)，误差小时低学习率(稳定保持) ----- */
    {
        float error_norm = ABS(error) / (ABS(pid->Setpoint) + 1e-6f);
        pid->eta_scale = LIMIT(HYD_THRESH_RBF_ETA_SCALE_MIN, error_norm * HYD_THRESH_RBF_ETA_SCALE_GAIN, 1.0f);
    }

    /* ----- RBF网络参数更新 (在线学习) ----- */
    error_rbf = pid->Feedback - y_hat;
    rbf_update_network(pid, x_normalized, pid->h, error_rbf, pid->eta_scale);

    /* ----- PID参数自适应更新 (基于梯度下降) ----- */
    float delta_kp = pid->eta_p * pid->eta_scale * error * pid->Jacobian * (error - pid->e_prev1);
    float delta_ki = pid->eta_i * pid->eta_scale * error * pid->Jacobian * error;
    float delta_kd = pid->eta_d * pid->eta_scale * error * pid->Jacobian * (error - 2*pid->e_prev1 + pid->e_prev2);

    pid->KP += delta_kp;
    pid->KI += delta_ki;
    pid->KD += delta_kd;

    /* 参数限幅 */
    pid->KP = LIMIT(pid->min_KP, pid->KP, pid->max_KP);
    pid->KI = LIMIT(pid->min_KI, pid->KI, pid->max_KI);
    pid->KD = LIMIT(pid->min_KD, pid->KD, pid->max_KD);

    /* ----- 增量式PID计算 ----- */
    raw_de = error - 2*pid->e_prev1 + pid->e_prev2;
    /* 一阶低通滤波微分项 */
    delta_temp = (1.0f - HYD_THRESH_RBF_DERIV_FILTER_ALPHA) * pid->delta_temp_prev + HYD_THRESH_RBF_DERIV_FILTER_ALPHA * raw_de;
    pid->delta_temp_prev = delta_temp;

    du = pid->KP * (error - pid->e_prev1) + pid->KI * error + pid->KD * raw_de;
    pid->du = du;   // 用于下次的du_prev

    /* ----- 前馈控制: 压力加速度补偿 ----- */
    fActPress = pid->Feedback;
    fDeltaPress = fActPress - pid->fLastActPress;
    fddPress = fDeltaPress - (pid->fLastActPress - pid->fLastActPress2);

    pid->fLastActPress2 = pid->fLastActPress;
    pid->fLastActPress = fActPress;

    /* 根据加速度方向选择系数 */
    float fUffAccCompensation = 0.0f;
    float errorThresholdForFF = 0.0005f;  // 设定一个误差阈值，只有当误差小时才启用前馈补偿
    if (ABS(error) < errorThresholdForFF) {
        if (fddPress < -HYD_THRESH_RBF_FF_ACCEL_DEAD_BAND) {
        	fUffAccCompensation = pid->fKff_a_neg * fddPress;
        } else if (fddPress > HYD_THRESH_RBF_FF_ACCEL_DEAD_BAND) {
        	fUffAccCompensation = pid->fKff_a_pos * fddPress;
        } else {
        	fUffAccCompensation = 0.0f;
        }
    }
    pid->fUffAcc = fUffAccCompensation;

    /* 综合控制增量 (减去加速度前馈) */
    du = du + pid->fBaseBias - pid->fUffAcc;
    pid->Output = pid->u_prev + du;

    /* 输出限幅: max_out = fMaxFlow * fFlowRateLimit [L/min] */
    float max_out = pid->fFlowRateLimit;
    pid->Output = LIMIT(MIN_OUTPUT, pid->Output, max_out);
    if (ABS(pid->Setpoint) < HYD_THRESH_RBF_SETPOINT_ZERO_EPS && pid->Feedback < HYD_THRESH_RBF_FEEDBACK_ZERO_BAND) {
        pid->Output = 0.0f;
    }

    output_total = pid->Output;

    /* 设定值变化率前馈 */
    if ( pid->EnableFF ) {
        ref_change = pid->Setpoint - pid->last_ref;
        float max_change_rate = 0.01f;  // 设定一个最大变化率，防止过大前馈
        ref_rate = LIMIT(-max_change_rate, ref_change, max_change_rate);
        float kSetpoint = (ref_rate >= 0.0f) ? pid->fKSetpoint_pos : pid->fKSetpoint_neg;

        dynamic_ff = kSetpoint * ref_rate;
        output_total += dynamic_ff;
        /* 注：原ST中 fUffAcc 未被加入 output_total，仅被用于减 du，保持一致性 */
    }
    pid->last_ref = pid->Setpoint;

    /* 最终限幅 */
    output_total = LIMIT(MIN_OUTPUT, output_total, max_out);
    if (ABS(pid->Setpoint) < HYD_THRESH_RBF_SETPOINT_ZERO_EPS && pid->Feedback < HYD_THRESH_RBF_FEEDBACK_ZERO_BAND) {
        output_total = 0.0f;
    }

    /* 输出流量 [L/min] */
    pid->n_out = output_total;

    /* ----- 更新历史状态 ----- */
    pid->u_prev  = pid->Output;         // 注意：u_prev存储限幅前的Output? 原ST用的是pid->Output
    pid->du_prev = du;
    pid->e_prev2 = pid->e_prev1;
    pid->e_prev1 = error;
    pid->y_prev1 = pid->Feedback;

    pid->Status = 1;
    pid->TuneResult = 66;   // 运行状态码
    float flow_request = pid->fMaxFlow * output_total;

    return flow_request;
}

/* 可选：设置PID限幅范围 */
void RBF_PID_SetParamLimits(RBF_PID_Handle *pid,
    float min_kp, float max_kp, float min_ki, float max_ki,
    float min_kd, float max_kd) {
    if (pid == NULL) {
        return;
    }
    if (min_kp <= max_kp) {
        pid->min_KP = min_kp;
        pid->max_KP = max_kp;
    }
    if (min_ki <= max_ki) {
        pid->min_KI = min_ki;
        pid->max_KI = max_ki;
    }
    if (min_kd <= max_kd) {
        pid->min_KD = min_kd;
        pid->max_KD = max_kd;
    }
}

void RBF_PID_SetLearningRates(RBF_PID_Handle *pid,
    float eta_w, float eta_c, float eta_b,
    float eta_p, float eta_i, float eta_d) {
    pid->eta_w = eta_w;
    pid->eta_c = eta_c;
    pid->eta_b = eta_b;
    pid->eta_p = eta_p;
    pid->eta_i = eta_i;
    pid->eta_d = eta_d;
}

void RBF_PID_SetPressureNormalization(RBF_PID_Handle *pid, float scale) {
    if (pid == NULL) {
        return;
    }
    /* 0 or negative -> clear to default (RBF_PID_Update falls back to MAX_PRESSURE). */
    pid->pressure_normalization_scale = (scale > 0.0f) ? scale : 0.0f;
}

void RBF_PID_SetSeed(RBF_PID_Handle *pid, uint32_t seed) {
    if (pid == NULL) {
        return;
    }
    pid->network_seed = seed;
    rbf_init_network(pid);
    controller_reset_state(pid);
}
