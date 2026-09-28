/**
 * @file rbf_pid.h
 * @brief RBF神经网络自适应PID控制器 - 嵌入式C实现
 * @note 基于ST代码转换，适用于ARM Cortex-M等平台
 *
 * 工程化修订 v3：
 *  - P0-1 新增 RBF_PID_SoftReset()：跨段复位控制器状态但保留网络权值与增益，
 *          支持跨模次学习保持；RBF_PID_Reset() 仍为完全复位。
 *  - P0-2/3 稳态死区与误差死区统一为压力量程百分比，量程可移植。
 *  - P0-4 输出饱和同向抑制学习，模式无关。
 *  - P0-5 微分项一阶滤波恢复（HYD_THRESH_RBF_DERIV_FILTER_ALPHA）。
 *  - P1-1 增益整定梯度按压力量程归一化，并加单步步长限幅。
 *  - P1-3 Jacobian 符号监督：连续为负触发故障位并冻结自适应。
 *  - P1-5 删除压力加速度前馈，改为实际压力速度阻尼项。
 *  - P1-6/8/10 删除死代码与冗余兼容槽（破坏性变更见文件尾注释）。
 */
#ifndef RBF_PID_H
#define RBF_PID_H

#include <stdbool.h>
#include <stdint.h>

/* ---------- 网络结构常数 ---------- */
#define RBF_HNUM            6       /* 隐含层节点数 */
#define RBF_INPUT_DIM       3       /* 输入维度: [Δu_n, y_n-1, y_n-2] */
#define RBF_MOMENTUM_STEPS  2       /* 在线更新使用的历史快照数 */

/* ---------- PID 参数限幅内置窗（初始化/复位恢复到该窗口） ---------- */
#define PID_MIN_KP  0.4f
#define PID_MAX_KP  0.9f
#define PID_MIN_KI  0.0013f         /* 0.0008 */
#define PID_MAX_KI  0.0046f
#define PID_MIN_KD  0.015f
#define PID_MAX_KD  0.035f

/* ---------- 学习率默认值 ---------- */
#define HYD_DEFAULT_RBF_W_LEARNING_RATE       0.002f
#define HYD_DEFAULT_RBF_C_LEARNING_RATE       0.002f
#define HYD_DEFAULT_RBF_B_LEARNING_RATE       0.002f
#define HYD_DEFAULT_PID_P_LEARNING_RATE       0.01f
#define HYD_DEFAULT_PID_I_LEARNING_RATE       0.00025f
#define HYD_DEFAULT_PID_D_LEARNING_RATE       0.00025f
#define HYD_DEFAULT_RBF_PID_SAMPLING_PERIOD   0.001f
#define RBF_PID_FIXED_SAMPLING_PERIOD  HYD_DEFAULT_RBF_PID_SAMPLING_PERIOD
/* ---------- 增量输出限幅与压力量程默认值 ---------- */
#define MIN_OUTPUT   -25.0f          /* 输出下限 [L/min] */
#define MAX_PRESSURE 250.0f         /* 压力量程默认值 [bar] */

/* ---------- 压力速度阻尼默认增益 ---------- */
/* 单位 [L/min / (bar/step)]，按 1ms 采样周期标定；
   含义：实际压力单步变化 1 bar，输出增量被反向压制 0.15 L/min */
#define RBF_PID_DEFAULT_PRESSURE_VEL_DAMP_GAIN  0.15f

/* ---------- 故障标志位（fault_flags） ---------- */
#define RBF_PID_FAULT_NONE              0x0000u
#define RBF_PID_FAULT_JACOBIAN_SIGN     0x0001u  /* Jacobian 持续为负，自适应已冻结 */

/**
 * @brief 反馈控制模式
 * @note PID 为默认。PI 模式保留 RBF 自适应与增量控制器，禁用全部微分行为。
 */
typedef enum {
    RBF_PID_CONTROL_MODE_PID = 0,
    RBF_PID_CONTROL_MODE_PI
} RBF_PID_ControlMode;

/**
 * @brief RBF-PID控制器状态结构体
 * @note 所有持久状态均内聚在此，支持多实例静态分配
 */
typedef struct {
    /* ---- 运行输入与基础配置 ---- */
    float P_set;                        /* 压力设定值 [bar] */
    float P_actual;                     /* 压力反馈值 [bar]（前端已滤波） */
    float sampling_period;              /* 采样时间 [s] */
    float fMaxFlow;                     /* 最大泵流量 [L/min] */
    float fFlowRateLimit;               /* 流量限幅比例 [0,1] */
    float output_min_flow;              /* 输出下限 [L/min] */
    float output_max_flow;              /* 输出上限 [L/min]，<=0 按 fMaxFlow*fFlowRateLimit 推导 */

    /* ---- 归一化标量 ---- */
    float pressure_normalization_scale; /* 压力量程 [bar]，<=0 回落 MAX_PRESSURE */
    float flow_normalization_scale;     /* 流量量程 [L/min] */

    /* ---- 物理增益（用于软限幅与增益补偿配置） ---- */
    float K;                            /* 系统稳态增益 [bar/(L/min)] */

    /* ---- 最近一次控制结果 ---- */
    float Output;                       /* 最后输出的流量指令 [L/min] */
    float KP;
    float KI;
    float KD;
    float du;                           /* 本拍增量输出 [L/min] */
    float Error;                        /* 死区处理后的误差 [bar] */
    float Jacobian;                     /* 归一化输入下的 dP/dΔu 估计（已换算物理量纲） */

    /* ---- 参数窗口 ---- */
    float min_KP;  float max_KP;
    float min_KI;  float max_KI;
    float min_KD;  float max_KD;

    int32_t Status;                     /* 1=就绪 2=运行 3=稳态 */
    int32_t TuneResult;

    /* ---- RBF 网络参数 ---- */
    float c[RBF_HNUM][RBF_INPUT_DIM];   /* 中心向量（归一化域） */
    float b_rbf[RBF_HNUM];              /* 宽度 */
    float w[RBF_HNUM];                  /* 权重 */

    /* ---- 学习率 ---- */
    float eta_w;  float eta_c;  float eta_b;
    float eta_p;  float eta_i;  float eta_d;

    /* ---- 动量因子 ---- */
    float alpha;                        /* 动量因子(0.05) */

    /* ---- 历史快照（动量更新） ---- */
    float ci_1[RBF_HNUM][RBF_INPUT_DIM];
    float ci_2[RBF_HNUM][RBF_INPUT_DIM];
    float bi_1[RBF_HNUM];
    float bi_2[RBF_HNUM];
    float w_1[RBF_HNUM];
    float w_2[RBF_HNUM];

    /* ---- 控制器运行状态（SoftReset 复位区） ---- */
    float u_prev;
    float e_prev1;
    float e_prev2;
    float du_prev;
    int32_t steady_count;
    int32_t jac_neg_count;              /* Jacobian 连续为负计数 */
    bool steady_state;
    bool output_saturated;
    bool learning_enabled;              /* 自适应总开关（网络+增益整定） */
    float y_prev1;
    float y_prev2;
    float last_rbf_input[RBF_INPUT_DIM];

    /* ---- 压力反馈历史 / 设定历史（诊断用） ---- */
    float fLastActPress;                /* 上一次压力反馈 [bar] */
    float fLastActPress2;               /* 上上次压力反馈 [bar] */
    float last_ref;                     /* 上一次设定值 [bar]，仅诊断 */
    float prev_d_term;                  /* 上一次滤波后微分项 */

    /* ---- 压力速度阻尼 ---- */
    float pressure_vel_damp_gain;       /* 阻尼增益 [L/min/(bar/step)] */
    bool  pressure_vel_damp_enabled;

    /* ---- 模式切换保存槽 ---- */
    float pid_mode_kd;
    float pid_mode_eta_d;
    RBF_PID_ControlMode control_mode;

    /* ---- 兼容存储槽：当前用于 Δu 归一化标尺 ---- */
    float f_dd_press_prev;
    float feedforward_flow_prev;
    float feedforward_flow;
    bool  ksys_valid;
    /* ---- 故障标志（RBF_PID_FAULT_xxx 位或） ---- */
    uint32_t fault_flags;
} RBF_PID_Handle;

/* ========================= API ========================= */

/**
 * @brief 初始化RBF-PID控制器（完全初始化，网络回种子）
 * @param pid RBF_PID句柄指针
 * @param sampling_period 采样时间(s)
 * @param max_flow_lmin 最大泵流量 [L/min]
 * @param flow_rate_limit_pct 流量限幅比例 [0~1]
 */
void RBF_PID_Init(RBF_PID_Handle *pid, float sampling_period,
                  float max_flow_lmin, float flow_rate_limit_pct);

/**
 * @brief 执行RBF-PID控制计算（固定1ms调用一次）
 * @param pid RBF_PID句柄指针
 * @param setpoint 设定值(压力 [bar])
 * @param feedback 反馈值(压力 [bar]，前端已滤波)
 * @return 控制器输出流量 [L/min]，不做 flow -> rpm 转换
 */
float RBF_PID_Update(RBF_PID_Handle *pid, float setpoint, float feedback);

/**
 * @brief 完全复位（等价重新 Init）：网络回种子、增益回默认窗、配置回内置默认
 * @note 仅在整机复位/模式彻底切换时使用；段间切换请用 RBF_PID_SoftReset
 */
void RBF_PID_Reset(RBF_PID_Handle *pid);

/**
 * @brief 软复位：仅复位控制器运行状态（误差/输出历史、饱和标志、稳态计数、
 *        故障计数），保留 RBF 网络权值、自适应增益、参数窗口与全部配置。
 * @note 用于压力段切换 / 模次开始，实现跨段、跨模次学习保持。
 */
void RBF_PID_SoftReset(RBF_PID_Handle *pid);

/** @brief 设置PID参数限幅（上下界颠倒会在运行时自动整理为 [min, max]） */
void RBF_PID_SetParamLimits(RBF_PID_Handle *pid, float min_kp, float max_kp,
                            float min_ki, float max_ki, float min_kd, float max_kd);

/** @brief 设置学习率，全部钳位 [0, 10] */
void RBF_PID_SetLearningRates(RBF_PID_Handle *pid, float eta_w, float eta_c, float eta_b,
                              float eta_p, float eta_i, float eta_d);

/**
 * @brief 配置压力归一化标量 [bar]
 * @note 推荐在每段 Resolve 时调用一次；影响误差死区、稳态死区、RBF 输入分布
 *       与梯度归一化，运行中改变会导致基准跳变。
 */
void RBF_PID_SetPressureNormalization(RBF_PID_Handle *pid, float scale);

/** @brief 配置流量归一化标量 [L/min] */
void RBF_PID_SetFlowNormalization(RBF_PID_Handle *pid, float scale);

/**
 * @brief 配置 Δu 归一化标尺 [L/min]，默认 5.0；传 0 或负值回落默认
 * @note 标定目标：单步 Δu/尺度 大致落在 ±0.5 以内。
 */
void RBF_PID_SetDuNormalization(RBF_PID_Handle *pid, float scale);

/**
 * @brief 设置系统物理增益 K = dP/dQ [bar/(L/min)]
 * @note K>0 时近目标段启用软限幅 P_set*1.05/K；<=0 时软限幅退化为硬限幅。
 */
void RBF_PID_SetGainCompensation(RBF_PID_Handle *pid, float systemGain);

/**
 * @brief 配置压力速度阻尼（方向化：仅阻尼压力上升方向）
 * @param gain 阻尼增益 [L/min/(bar/step)]，默认 0.15；传 0 或负值关闭阻尼
 * @note f_velfb = -gain * max(ΔP_actual, 0)，叠加到单步增量 du 上：
 *       仅当实际压力正在上升（ΔP_actual > 0）时产生反向压制，抑制建压
 *       超调与压力振荡；压力下降（卸压）时阻尼完全旁路，不拖慢卸压速度。
 *       依据：建压方向对象为积分型+传播滞后，需阻尼；卸压方向对象自身
 *       为快一阶过程，天然无超调，阻尼反而形成有害的"液压缓冲"。
 *       与 PI/PID 模式无关；增益按 1ms 周期标定，周期变化需按比例重标。
 */
void RBF_PID_SetPressureVelocityDamping(RBF_PID_Handle *pid, float gain);

/**
 * @brief 自适应总开关（RBF 网络学习 + PID 增益在线整定）
 * @param enabled false 时退化为参数固定的增量PID（网络与增益保持当前值）
 */
void RBF_PID_SetAdaptiveEnabled(RBF_PID_Handle *pid, bool enabled);

/** @brief 选择反馈控制模式（PI 模式禁用全部微分行为，KD/eta_d 保存于槽位） */
void RBF_PID_SetControlMode(RBF_PID_Handle *pid, RBF_PID_ControlMode mode);

#endif /* RBF_PID_H */

/*
 * ================= 破坏性变更清单（v3） =================
 * 1. 删除 RBF_PID_ControlState 枚举、Handle.control_state 字段
 *    （原状态机无消费者，属死代码）。
 * 2. 删除 RBF_PID_SetPressureAccelFeedforwardEnabled()，
 *    替换为 RBF_PID_SetPressureVelocityDamping()。
 * 3. 删除 RBF_PID_SetSeed() 与 Handle.network_seed（种子从未接入初始化）。
 * 4. 删除字段：fGainCompensation、gain_compensation_enabled、
 *    gain_compensation_factor、flowToPumpSpeedGain、v_ref_k1、
 *    pressure_accel_ff_enabled、pressure_accel_ff_requested。
 * 5. 新增字段：pressure_vel_damp_gain/enabled、learning_enabled、
 *    jac_neg_count、fault_flags。
 */
