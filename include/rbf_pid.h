/**
 * @file rbf_pid.h
 * @brief RBF神经网络自适应PID控制器 - 嵌入式C实现
 * @note 基于ST代码转换，适用于ARM Cortex-M等平台
 *
 * 评审修复 v2：补充 Δu 单独归一化接口；为保持嵌入式结构体大小，复用
 * 既有兼容状态槽保存该标尺。压力滤波和死区状态由外层控制器维护。
 */

#ifndef RBF_PID_H
#define RBF_PID_H

#include <stdbool.h>
#include <stdint.h>

/* 常数定义 */
#define RBF_HNUM            6       // 隐含层节点数
#define RBF_INPUT_DIM       3       // 输入维度
#define RBF_MOMENTUM_STEPS  2       // 在线更新使用的历史快照数

/* PID 参数限幅默认窗。
 * Task 2 要求初始化/复位恢复到确定性的内置窗口，后续可通过
 * RBF_PID_SetParamLimits() 覆盖。 */
#define PID_MIN_KP          0.4f
#define PID_MAX_KP          0.9f
#define PID_MIN_KI          0.0013f // 0.0008
#define PID_MAX_KI          0.0056f
#define PID_MIN_KD          0.015f
#define PID_MAX_KD          0.035f

#define HYD_DEFAULT_RBF_W_LEARNING_RATE 0.002f
#define HYD_DEFAULT_RBF_C_LEARNING_RATE 0.002f
#define HYD_DEFAULT_RBF_B_LEARNING_RATE 0.002f
#define HYD_DEFAULT_PID_P_LEARNING_RATE 0.01f
#define HYD_DEFAULT_PID_I_LEARNING_RATE 0.00025f
#define HYD_DEFAULT_PID_D_LEARNING_RATE 0.00025f

#define HYD_DEFAULT_RBF_PID_SAMPLING_PERIOD 0.001


/* Task 3 增量控制输出限幅 */
#define MIN_OUTPUT          -5.0f
#define MAX_PRESSURE        250.0f

typedef enum {
    RBF_PID_CONTROL_STATE_INIT = 0,
    RBF_PID_CONTROL_STATE_BOOST,
    RBF_PID_CONTROL_STATE_HOLD,
    RBF_PID_CONTROL_STATE_RELIEF
} RBF_PID_ControlState;

typedef enum {
    RBF_PID_CONTROL_MODE_PID = 0,
    RBF_PID_CONTROL_MODE_PI
} RBF_PID_ControlMode;

/**
 * @brief RBF-PID控制器状态结构体
 * @note 所有持久状态均内聚在此，支持多实例静态分配
 */
typedef struct {
    /* 运行输入与基础配置 */
    float P_set;                    // 压力设定值(原始单位)
    float P_actual;                 // 压力反馈值(原始单位)
    float sampling_period;          // 采样时间(s)
    float fMaxFlow;                 // 最大泵流量 [L/min]
    float fFlowRateLimit;           // 兼容配置：保留流量限幅比例 [0,1]
    float output_min_flow;          // 输出下限 [L/min]，0 表示使用物理默认值
    float output_max_flow;          // 输出上限 [L/min]，0 表示按 fMaxFlow/fFlowRateLimit 推导

    /* 压力归一化标量（与设定/反馈一致的压力单位，当前为 bar）.
     * 0 或负值 -> 落回内置默认 MAX_PRESSURE. 调用 RBF_PID_SetPressureNormalization()
     * 配置；推荐由 pressure_controller.c 在每段 Resolve 时根据段配置写入。 */
    float pressure_normalization_scale;
    float flow_normalization_scale;
    float flowToPumpSpeedGain;      /* retained for outer integration only */

    /* 兼容配置 */
    float K;                        // 系统增益 (bar per L/min, 稳态压力/流量比)
    float fGainCompensation;        // 兼容字段：保留最近一次计算的补偿因子
    bool gain_compensation_enabled; // 是否在输出末端应用兼容增益补偿
    bool pressure_accel_ff_enabled;
    float gain_compensation_factor; // 输出补偿因子，默认 1.0

    /* 最近一次控制结果 */
    float Output;                   /* last commanded flow [L/min] */
    float KP;
    float KI;
    float KD;
    float du;
    float Error;
    float Jacobian;
    float min_KP;
    float max_KP;
    float min_KI;
    float max_KI;
    float min_KD;
    float max_KD;
    int32_t Status;
    int32_t TuneResult;

    /* RBF神经网络参数 */
    float c[RBF_HNUM][RBF_INPUT_DIM];   // 中心向量
    float b_rbf[RBF_HNUM];              // 宽度
    float w[RBF_HNUM];                  // 权重

    /* 学习率参数 */
    float eta_w;                    // 权重学习率
    float eta_c;                    // 中心学习率
    float eta_b;                    // 宽度学习率
    float eta_p;                    // P参数学习率
    float eta_i;                    // I参数学习率
    float eta_d;                    // D参数学习率

    /* 动量因子 */
    float alpha;                    // 动量因子(0.05)

    /* 历史数据存储(动量更新) */
    float ci_1[RBF_HNUM][RBF_INPUT_DIM];
    float ci_2[RBF_HNUM][RBF_INPUT_DIM];
    float bi_1[RBF_HNUM];
    float bi_2[RBF_HNUM];
    float w_1[RBF_HNUM];
    float w_2[RBF_HNUM];

    /* 控制器状态变量 */
    float u_prev;
    float e_prev1;
    float e_prev2;
    float du_prev;
    int32_t steady_count;
    bool steady_state;
    bool output_saturated;
    RBF_PID_ControlState control_state;
    float y_prev1;
    float y_prev2;
    float last_rbf_input[RBF_INPUT_DIM];

    /* 前馈控制相关 */
    float fLastActPress;            // 上一次压力反馈
    float fLastActPress2;           // 上上次压力反馈
    float last_ref;                 // 上一次设定值
    float prev_d_term;                // 上一次微分项
    /* 网络初始化种子 */
    uint32_t network_seed;          // 兼容保留字段：当前仅存储，尚未接入网络初始化流程
    float pid_mode_kd;
    float pid_mode_eta_d;
    bool pressure_accel_ff_requested;
    RBF_PID_ControlMode control_mode; /* Appended to preserve existing field offsets. */

    float f_dd_press_prev;                /* 兼容存储槽：当前用于 Δu 归一化标尺 */
    float v_ref_k1;

} RBF_PID_Handle;

/**
 * @brief 初始化RBF-PID控制器
 * @param pid RBF_PID句柄指针
 * @param sampling_period 采样时间(s)
 * @param max_flow_lmin 最大泵流量 [L/min]
 * @param flow_rate_limit_pct 流量限幅比例 [0~1]
 * @return 无
 */
void RBF_PID_Init(RBF_PID_Handle *pid, float sampling_period,
                  float max_flow_lmin, float flow_rate_limit_pct);

/**
 * @brief 执行RBF-PID控制计算
 * @param pid RBF_PID句柄指针
 * @param setpoint 设定值(压力, 原始单位)
 * @param feedback 反馈值(压力, 原始单位)
 * @return 控制器输出流量 [L/min], 不在此函数内执行 flow -> rpm 转换
 */
float RBF_PID_Update(RBF_PID_Handle *pid, float setpoint, float feedback);

/**
 * @brief 复位控制器(清空所有历史状态)
 * @param pid RBF_PID句柄指针
 * @note 复位后恢复到确定性的内置默认配置，并保留初始化入参对应的基础量程。
 */
void RBF_PID_Reset(RBF_PID_Handle *pid);

/**
 * @brief 设置PID参数限幅(可选，默认使用内部宏)
 * @param pid RBF_PID句柄指针
 * @param min_kp, max_kp, min_ki, max_ki, min_kd, max_kd
 * @note 若传入上下界顺序颠倒，会在运行时自动整理为 [min, max]。
 */
void RBF_PID_SetParamLimits(RBF_PID_Handle *pid,
    float min_kp, float max_kp, float min_ki, float max_ki,
    float min_kd, float max_kd);

/**
 * @brief 设置学习率
 * @param pid 句柄
 * @param eta_w, eta_c, eta_b, eta_p, eta_i, eta_d
 * @note 所有学习率均会被限制在 [0, 10]。
 */
void RBF_PID_SetLearningRates(RBF_PID_Handle *pid,
    float eta_w, float eta_c, float eta_b,
    float eta_p, float eta_i, float eta_d);

/**
 * @brief 配置压力归一化标量
 * @param pid RBF_PID句柄指针
 * @param scale 满量程标量（单位与 setpoint/feedback 相同，当前为 bar）
 *              传 0 或负值会清回内部默认 MAX_PRESSURE.
 * @note 推荐在每段开始时调用一次；运行中改变会导致归一化基准跳变。
 */
void RBF_PID_SetPressureNormalization(RBF_PID_Handle *pid, float scale);
void RBF_PID_SetFlowNormalization(RBF_PID_Handle *pid, float scale);

/**
 * @brief 配置 Δu 归一化标量（评审修复 v2 新增）
 * @param pid RBF_PID句柄指针
 * @param scale Δu 归一化尺度 [L/min]，默认 5.0；传 0 或负值回落默认。
 * @note 标定目标：使单步 Δu/尺度 大致落在 ±0.5 以内。
 *       若采样周期相对 1ms 变化超过 5 倍，建议按实际 Δu 幅度重新标定。
 */
void RBF_PID_SetDuNormalization(RBF_PID_Handle *pid, float scale);

/**
 * @brief 设置系统物理增益兼容参数
 * @param pid RBF_PID句柄指针
 * @param systemGain 系统稳态增益 K = deltaPressure / deltaFlow [bar/(L/min)]
 * @note 当 systemGain 和 fMaxFlow 均为正值时，在 Update() 输出末端应用兼容补偿。
 *       补偿因子会随压力归一化标量变化而同步刷新，避免 setter 调用顺序造成陈旧状态。
 */
void RBF_PID_SetGainCompensation(RBF_PID_Handle *pid, float systemGain);

void RBF_PID_SetPressureAccelFeedforwardEnabled(RBF_PID_Handle *pid, bool enabled);

/**
 * @brief Select the adaptive feedback mode.
 * @note PID is the default. PI mode keeps the same RBF adaptation and
 *       incremental controller while disabling all derivative behavior.
 */
void RBF_PID_SetControlMode(RBF_PID_Handle *pid, RBF_PID_ControlMode mode);

/**
 * @brief 设置网络初始化种子兼容字段
 * @param pid RBF_PID句柄指针
 * @param seed 保留值；当前版本仅存储，不影响网络初始化或复位重现性
 */
void RBF_PID_SetSeed(RBF_PID_Handle *pid, uint32_t seed);

#endif /* RBF_PID_H */
