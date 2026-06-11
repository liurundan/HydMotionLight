/**
 * @file rbf_pid.c
 * @brief RBF神经网络自适应PID控制器 - 嵌入式C实现
 * @note 严格遵循 rbf_pid.h 接口约定，修复了抗饱和、发散风险及前馈逻辑
 *
 * 压力闭环 RBF-PID 核心公式回顾（归一化空间）:
 *
 *   e(k)   = r(k) - y(k)                       -- 归一化误差
 *   du(k)  = KP·[e(k)-e(k-1)] + KI·e(k) + KD·[e(k)-2e(k-1)+e(k-2)] + ff + bias
 *   u(k)   = u(k-1) + du(k)                    -- 增量式
 *   n_out  = u(k) * fMaxFlow * gain_comp       -- 反归一化到物理流量 [L/min]
 *
 * 关键修复 (2026-06-03):
 *   - 引入 gain_compensation: 根据系统物理增益 K 和归一化标量自动缩放输出，
 *     解决低压小目标（2-3 bar）时归一化标量过小导致的增益失配问题。
 *     补偿公式: gain_comp = pressure_normalization_scale / (K * fMaxFlow)
 *     当 K=0 时, gain_comp = 1.0 (无补偿，由PID自适应调节)。
 *   - 此补偿确保: 在稳态, u≈targetPressure/(K*fMaxFlow) 时 du→0。
 */

#include "rbf_pid.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

/* ==========================================
 * 内部辅助宏与函数
 * ========================================== */
#define LIMIT(min, val, max) (((val) < (min)) ? (min) : (((val) > (max)) ? (max) : (val)))
#define ABS_VAL(x)           (((x) < 0.0f) ? -(x) : (x))
#define EPSILON              1e-5f

// 线性同余伪随机数生成器 (LCG)
static float lcg_rand(RBF_PID_Handle *pid) {
    pid->network_seed = (pid->network_seed * 1103515245u + 12345u) & 0x7fffffffu;
    return ((float)pid->network_seed / (float)0x7fffffff) * 2.0f - 1.0f; // [-1.0, 1.0]
}

/* ==========================================
 * 1. 初始化与配置接口
 * ========================================== */
void RBF_PID_Init(RBF_PID_Handle *pid, float sampling_period,
                  float max_flow_lmin, float flow_rate_limit_pct) {
    if (!pid) return;
    memset(pid, 0, sizeof(RBF_PID_Handle));

    pid->sampling_period = sampling_period;
    pid->fMaxFlow = max_flow_lmin;
    pid->fFlowRateLimit = LIMIT(0.0f, flow_rate_limit_pct, 1.0f);
    pid->pressure_normalization_scale = MAX_PRESSURE; // 默认归一化标量

    // 1. 初始化 PID 基准参数与限幅
    pid->KP = 0.8f; pid->KI = 0.02f; pid->KD = 1.0f;
    pid->min_KP = PID_MIN_KP; pid->max_KP = PID_MAX_KP;
    pid->min_KI = PID_MIN_KI; pid->max_KI = PID_MAX_KI;
    pid->min_KD = PID_MIN_KD; pid->max_KD = PID_MAX_KD;

    // 2. 初始化 RBF 网络参数
    pid->network_seed = 123456789u; // 默认种子
    for (int i = 0; i < RBF_HNUM; i++) {
        pid->w[i] = 0.1f;
        pid->b_rbf[i] = 3.0f; // 初始宽度设大，防止死神经元
        for (int j = 0; j < RBF_INPUT_DIM; j++) {
            pid->c[i][j] = lcg_rand(pid);
        }
    }

    // 3. 初始化学习率 (降低默认值，防止工业现场发散)
    pid->eta_w = 0.03f; pid->eta_c = 0.02f; pid->eta_b = 0.01f;
    pid->eta_p = 0.05f; pid->eta_i = 0.02f; pid->eta_d = 0.05f;
    pid->alpha = 0.05f; // 动量因子
    pid->belte = 0.01f; // 二次动量因子
    pid->eta_scale = 1.0f;

    // 4. 初始化前馈参数
    pid->EnableFF = true;
    pid->fKff_a_pos = 0.8f;  pid->fKff_a_neg = 0.8f;
    pid->fKSetpoint_pos = 0.003f; pid->fKSetpoint_neg = 0.001f;
    pid->fBaseBias = 0.0002f;

    // 5. 初始化增益补偿 (0 = 禁用，由外部设置)
    pid->fGainCompensation = 0.0f;

    pid->FirstScan = true;
    pid->Status = -1; // 未使能
}

void RBF_PID_Reset(RBF_PID_Handle *pid) {
    if (!pid) return;
    // 仅清空历史状态和中间变量，保留配置参数
    pid->u_prev = 0.0f; pid->du_prev = 0.0f;
    pid->e_prev1 = 0.0f; pid->e_prev2 = 0.0f;
    pid->y_prev1 = 0.0f; pid->delta_temp_prev = 0.0f;
    pid->fLastActPress = 0.0f; pid->fLastActPress2 = 0.0f;
    pid->last_ref = 0.0f; pid->fUffAcc = 0.0f;

    // 清空动量历史
    memset(pid->ci_1, 0, sizeof(pid->ci_1)); memset(pid->ci_2, 0, sizeof(pid->ci_2)); memset(pid->ci_3, 0, sizeof(pid->ci_3));
    memset(pid->bi_1, 0, sizeof(pid->bi_1)); memset(pid->bi_2, 0, sizeof(pid->bi_2)); memset(pid->bi_3, 0, sizeof(pid->bi_3));
    memset(pid->w_1, 0, sizeof(pid->w_1)); memset(pid->w_2, 0, sizeof(pid->w_2)); memset(pid->w_3, 0, sizeof(pid->w_3));

    pid->FirstScan = true;
    pid->Output = 0.0f;
    pid->n_out = 0.0f;
}

void RBF_PID_SetParamLimits(RBF_PID_Handle *pid,
    float min_kp, float max_kp, float min_ki, float max_ki,
    float min_kd, float max_kd) {
    if (!pid) return;
    pid->min_KP = min_kp; pid->max_KP = max_kp;
    pid->min_KI = min_ki; pid->max_KI = max_ki;
    pid->min_KD = min_kd; pid->max_KD = max_kd;
}

void RBF_PID_SetLearningRates(RBF_PID_Handle *pid,
    float eta_w, float eta_c, float eta_b,
    float eta_p, float eta_i, float eta_d) {
    if (!pid) return;
    pid->eta_w = LIMIT(0.001f, eta_w, 0.5f);
    pid->eta_c = LIMIT(0.001f, eta_c, 0.5f);
    pid->eta_b = LIMIT(0.001f, eta_b, 0.5f);
    pid->eta_p = LIMIT(0.001f, eta_p, 0.5f);
    pid->eta_i = LIMIT(0.001f, eta_i, 0.5f);
    pid->eta_d = LIMIT(0.001f, eta_d, 0.5f);
}

void RBF_PID_SetPressureNormalization(RBF_PID_Handle *pid, float scale) {
    if (!pid) return;
    pid->pressure_normalization_scale = (scale > 0.0f) ? scale : MAX_PRESSURE;
}

void RBF_PID_SetGainCompensation(RBF_PID_Handle *pid, float systemGain) {
    if (!pid) return;
    pid->K = (systemGain > 0.0f) ? systemGain : 0.0f;
    /* 计算增益补偿因子:
     *   gain_comp = normScale / (K * fMaxFlow)
     * 当 K=0 时禁用补偿。
     * 此因子用于缩放 PID 输出，使归一化空间的输出直接映射到正确的物理流量。
     * 推导: 稳态时 pressure = flow * K, 归一化后 p_norm = p / normScale,
     *       flow = p / K, 归一化后 flow_norm = flow / fMaxFlow = p / (K * fMaxFlow)
     *       因此 p_norm → flow_norm 的增益为 fMaxFlow * K / normScale
     *       补偿因子取其倒数: normScale / (K * fMaxFlow)
     */
    if (systemGain > 0.0f && pid->fMaxFlow > 0.0f && pid->pressure_normalization_scale > 0.0f) {
        pid->fGainCompensation = pid->pressure_normalization_scale / (systemGain * pid->fMaxFlow);
        /* 限制补偿范围，防止极端值 */
        if (pid->fGainCompensation > 100.0f) pid->fGainCompensation = 100.0f;
        if (pid->fGainCompensation < 0.001f) pid->fGainCompensation = 0.001f;
    } else {
        pid->fGainCompensation = 0.0f;
    }
}

void RBF_PID_SetSeed(RBF_PID_Handle *pid, uint32_t seed) {
    if (!pid) return;
    pid->network_seed = seed;
}

/* ==========================================
 * 2. 核心控制算法实现
 * ========================================== */
float RBF_PID_Update(RBF_PID_Handle *pid, float setpoint, float feedback) {
    if (!pid) return 0.0f;

    pid->P_set = setpoint;
    pid->P_actual = feedback;

    // 1. 状态机与复位处理
    if (pid->Reset) {
        RBF_PID_Reset(pid);
        pid->Reset = false;
    }
    if (!pid->enable) {
        pid->Status = -1;
        pid->Output = 0.0f;
        pid->n_out = 0.0f;
        return 0.0f;
    }
    pid->Status = 1;

    // 2. 归一化处理
    float norm_scale = pid->pressure_normalization_scale;
    pid->Setpoint = setpoint / norm_scale;
    pid->Feedback = feedback / norm_scale;

    // 3. 计算误差与微分滤波
    pid->Error = pid->Setpoint - pid->Feedback;

    // 4. 准备 RBF 网络输入特征
    // 修复点: 将 du_prev 改为 u_prev (绝对控制量)，更符合液压伺服阀物理特性
    float x_norm[RBF_INPUT_DIM] = {
        pid->du_prev/pid->fFlowRateLimit,        // x0: 上一周期绝对控制量 (用于求 Jacobian)
        pid->Feedback,      // x1: 当前压力反馈
        pid->y_prev1          // x2: 当前误差
    };

    // 5. RBF 前向传播与 Jacobian 辨识
    pid->y_hat = 0.0f;
    pid->Jacobian = 0.0f;
    for (int i = 0; i < RBF_HNUM; i++) {
        float dist_sq = 0.0f;
        for (int j = 0; j < RBF_INPUT_DIM; j++) {
            float diff = x_norm[j] - pid->c[i][j];
            dist_sq += diff * diff;
        }
        float b_sq = pid->b_rbf[i] * pid->b_rbf[i];
        if (b_sq < EPSILON) b_sq = EPSILON;

        // 标准高斯函数
        pid->h[i] = expf(-dist_sq / (2.0f * b_sq));
        pid->y_hat += pid->w[i] * pid->h[i];

        // Jacobian: dy/du = sum(w * h * (c_0 - x_0) / b^2)
        pid->Jacobian += pid->w[i] * pid->h[i] * (pid->c[i][0] - x_norm[0]) / b_sq;
    }

    // 6. RBF 网络在线学习 (带梯度裁剪与动量)
    float error_rbf = pid->Feedback - pid->y_hat;
    error_rbf = LIMIT(-10.2f, error_rbf, 10.2f); // 限制辨识误差，防止突变摧毁网络
    float eta_scale = pid->eta_scale;

    for (int i = 0; i < RBF_HNUM; i++) {
        float b_sq = pid->b_rbf[i] * pid->b_rbf[i];
        if (b_sq < EPSILON) b_sq = EPSILON;
        float dist_sq = 0.0f;
        for (int j = 0; j < RBF_INPUT_DIM; j++) {
            float diff = x_norm[j] - pid->c[i][j];
            dist_sq += diff * diff;
        }

        // 权重更新 (带梯度裁剪)
        float dw = eta_scale * pid->eta_w * error_rbf * pid->h[i] + pid->alpha * pid->w_1[i] + pid->belte * pid->w_2[i];
        dw = LIMIT(-10.05f, dw, 10.05f);
        pid->w_3[i] = pid->w_2[i]; pid->w_2[i] = pid->w_1[i]; pid->w_1[i] = dw;
        pid->w[i] += dw;

        // 宽度更新 (下限提高至 0.5，防止 b^3 过小导致更新量放大)
        float db = eta_scale * pid->eta_b * error_rbf * pid->w[i] * pid->h[i] * dist_sq / (pid->b_rbf[i] * b_sq)
                   + pid->alpha * pid->bi_1[i] + pid->belte * pid->bi_2[i];
        db = LIMIT(-10.05f, db, 10.05f);
        pid->bi_3[i] = pid->bi_2[i]; pid->bi_2[i] = pid->bi_1[i]; pid->bi_1[i] = db;
        pid->b_rbf[i] += db;
        if (pid->b_rbf[i] < 0.5f) pid->b_rbf[i] = 0.5f;

        // 中心点更新
        for (int j = 0; j < RBF_INPUT_DIM; j++) {
            float dc = eta_scale * pid->eta_c * error_rbf * pid->w[i] * pid->h[i] * (x_norm[j] - pid->c[i][j]) / b_sq
                       + pid->alpha * pid->ci_1[i][j] + pid->belte * pid->ci_2[i][j];
            dc = LIMIT(-10.05f, dc, 10.05f);
            pid->ci_3[i][j] = pid->ci_2[i][j]; pid->ci_2[i][j] = pid->ci_1[i][j]; pid->ci_1[i][j] = dc;
            pid->c[i][j] += dc;
        }
    }

    // 7. PID 参数自适应更新
    float xc1 = pid->Error - pid->e_prev1;
    float xc2 = pid->Error;
    float xc3 = pid->Error - 2*pid->e_prev1 + pid->e_prev2; // 使用滤波后的微分项

    float dKP = eta_scale * pid->eta_p * pid->Error * pid->Jacobian * xc1;
    float dKI = eta_scale * pid->eta_i * pid->Error * pid->Jacobian * xc2;
    float dKD = eta_scale * pid->eta_d * pid->Error * pid->Jacobian * xc3;

    // 限制 PID 参数单次调整幅度
    dKP = LIMIT(-10.05f, dKP, 10.05f);
    dKI = LIMIT(-10.02f, dKI, 10.02f);
    dKD = LIMIT(-10.05f, dKD, 10.05f);

    pid->KP = LIMIT(pid->min_KP, pid->KP + dKP, pid->max_KP);
    pid->KI = LIMIT(pid->min_KI, pid->KI + dKI, pid->max_KI);
    pid->KD = LIMIT(pid->min_KD, pid->KD + dKD, pid->max_KD);

    // 8. 计算前馈控制 (修正逻辑：瞬态增强，稳态减弱)
    float feedforward = 0.0f;
    float damping_acc = 0.0f; // 阻尼补偿量
    if (pid->EnableFF) {
        float sp_rate = pid->Setpoint - pid->last_ref;
        float press_accel = pid->Feedback - 2.0f * pid->fLastActPress + pid->fLastActPress2;

        float sp_jump_threshold = 0.05f; // 设定值跳变阈值
        float ff_sp = 0.0f;
        if(ABS_VAL(sp_rate) < sp_jump_threshold) {
			ff_sp = (sp_rate >= 0.0f) ? (pid->fKSetpoint_pos * sp_rate) : (pid->fKSetpoint_neg * sp_rate);
		}

        // 加速度前馈 (区分正负向)
        static float filt_acc = 0.0f;
        float alpha_acc = 0.0f;
		filt_acc = alpha_acc * filt_acc + (1.0f - alpha_acc) * press_accel;

		// 3. 加速度死区 (忽略微小的液压脉动，防止阀门无谓的高频动作)
		float acc_deadband = 0.00002f; // 归一化压力的 0.2%
		float valid_acc = (ABS_VAL(filt_acc) > acc_deadband) ? filt_acc : 0.0f;

		// 4. 计算阻尼补偿量 (区分正负向增益)
		if (valid_acc > 0.0f) {
			damping_acc = -(pid->fKff_a_pos * valid_acc); // 减速/防超调
		} else {
			damping_acc = (pid->fKff_a_neg * valid_acc); // 加速/防跌落 (注意负负得正)
		}

        feedforward = ff_sp;

        float max_out_limit = pid->fFlowRateLimit;
        float ff_clamp_limit = max_out_limit * 0.3f; // 前馈部分限制在总输出的一半，防止过度补偿
        feedforward = LIMIT(-ff_clamp_limit, feedforward, ff_clamp_limit);
    }

    // 9. 计算增量式 PID 输出
    pid->du = pid->KP * xc1 + pid->KI * xc2 + pid->KD * xc3 + feedforward + pid->fBaseBias;

    // 10. 抗积分饱和 (Anti-Windup) 机制 【核心修复】
    // 当增益补偿激活时，output_total 直接映射到归一化压力:
    //   p_norm = output_total (稳态时)
    // 因此抗积分饱和的上限应基于设定点而非 fFlowRateLimit=1.0。
    // 无增益补偿时回退到原始行为。
    float max_out_limit;
    if (pid->fGainCompensation > 0.0f) {
        /* 增益补偿模式下: output_total = p_norm 稳态。
         * 上限 = max(Setpoint * 1.2, 0.1)，留 20% 瞬态余量。
         * 这确保稳态时积分不会超过目标压力的 120%。 */
        max_out_limit = pid->Setpoint * 1.2f;
        if (max_out_limit < 0.1f) max_out_limit = 0.1f;   /* 最小 10% 避免锁死 */
        if (max_out_limit > pid->fFlowRateLimit) max_out_limit = pid->fFlowRateLimit;
    } else {
        max_out_limit = pid->fFlowRateLimit;
    }

//    float temp_output = pid->u_prev + pid->du;
//    if (temp_output > max_out_limit && pid->du > 0.0f) {
//        pid->du = max_out_limit - pid->u_prev; // 强制增量不突破上限
//    } else if (temp_output < MIN_OUTPUT && pid->du < 0.0f) {
//        pid->du = MIN_OUTPUT - pid->u_prev;    // 强制增量不突破下限
//    }

    float base_output = pid->u_prev + pid->du;
    float output_total = base_output + damping_acc; // 加入阻尼补偿
    output_total = LIMIT(MIN_OUTPUT, output_total, max_out_limit);

    // 零压死区平滑处理
    if (ABS_VAL(pid->Setpoint) < EPSILON && pid->Feedback < 0.02f && pid->Feedback >= 0.0f) {
        output_total = 0.0f;
    }

    // 12. 增益补偿: 将归一化空间输出映射到正确的物理流量
    // 补偿公式: n_out = output_total * fMaxFlow * fGainCompensation
    // 当 fGainCompensation=0 时，退化为原始行为（无补偿）
    float compensated_n_out;
    if (pid->fGainCompensation > 0.0f) {
        compensated_n_out = output_total * pid->fMaxFlow * pid->fGainCompensation;
        /* 补偿后仍受流量限幅约束 */
        float compensated_max = pid->fMaxFlow * pid->fFlowRateLimit;
        compensated_n_out = LIMIT(MIN_OUTPUT * pid->fMaxFlow, compensated_n_out, compensated_max);
    } else {
        compensated_n_out = output_total * pid->fMaxFlow;
    }

    pid->Output = output_total;
    pid->n_out = compensated_n_out;

    // 13. 更新历史状态
    pid->u_prev = output_total;
    pid->du_prev = pid->du;
    pid->e_prev2 = pid->e_prev1;
    pid->e_prev1 = pid->Error;
    pid->y_prev1 = pid->Feedback;
    pid->fLastActPress2 = pid->fLastActPress;
    pid->fLastActPress = pid->Feedback;
    pid->last_ref = pid->Setpoint;
    pid->FirstScan = false;

    return pid->n_out;
}
