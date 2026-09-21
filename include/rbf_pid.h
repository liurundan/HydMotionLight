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

    /* v10: 泵实测可达流量钳位（back-calculation anti-windup）
     *
     * 由 pressure_controller 从伺服泵实测转速换算：
     *   actual_flow = rpm / flowToPumpSpeedGain
     * 当实测转速贴上 IEC 配置的转速上限时，把软上限收缩到实测可达流量，
     * 使积分不再朝"执行器无法实现的方向"累积。
     *
     * external_flow_cap_valid = false 时行为与 v9 逐位一致（默认关闭）；
     * 只有 IEC 使能 HYD_PARAM_PUMP_FEEDBACK_ANTI_WINDUP 才会置位。 */
    float external_flow_cap;              /* 实测可达流量上限 [L/min]，>= 0 */
    bool external_flow_cap_valid;         /* 是否参与钳位 */
    bool external_saturated;              /* 诊断：实测转速是否贴限 */

    /* v6: 本轮升压已达到的压力峰值(bar) — HMI 诊断量（"本轮最高压力"）。
     *
     * 历史：v6~v8 曾用作"峰值进展闸门"的判据（压力回落后收回超调驱动上限）。
     * v9 已废除该判据 —— 实测该闸门在保压段恒等（无额外保护），却把超调恢复段
     * 的驱动从 ~4.4 L/min 砍到 0.79 L/min，使 ts 从 ~0.5s 恶化到 1.5~2.1s
     * （见 rbf_pid.c 中 RBF_PID_OVERDRIVE_REL_ERR_GATE 的说明与
     * docs/压力闭环控制链再评估-2026-09-16.md §12.2）。
     * 字段保留仅作可观测量，不再参与控制决策。设定值跳变 / 段切换 / 复位时
     * 重置为当前实测压力。 */
    float overdrive_peak_press;
    /* v6: 压力停滞计时(s) — HMI 诊断量。
     *
     * 历史：v6~v8 曾作为超调驱动"停滞闸门"（压力持续不上升超过 0.25s 即收回
     * 上限）。v9 已废除该判据，理由同上（与峰值闸门同组，实测其在保压段恒等）。
     * 字段保留仅作可观测量：现场可通过它判断"泵在转但压力不动"
     * （泵故障 / 管路堵塞 / 传感器卡死），不再参与控制决策。 */
    float press_stuck_time_s;

    /* v8: 升压段实验限流（用户 2026-09-16 要求：升压段限流，给超调上保险）。
     *
     * 背景：25cc/rev、1700rpm 的伺服泵在 flowToPumpSpeedGain = 40 rpm/(L/min)
     * 下最大流量 42.5 L/min，折合等效建压 K*Q = 200*42.5 = 8500 bar，相对
     * 150 bar 目标有 57 倍流量权限 —— 驱动能力远超需求，超调与否完全由
     * 流量上限决定，这是"用实验限流把超调钉住"的物理依据。
     *
     * 注意（勿再引用旧数字）：早期此处写"不限流时超调 66.7%、直抵 250 bar
     * 溢流阀"，该数字来自**非物理管路参数**（R = 2.0e11，使泵出口与容腔解耦）。
     * 按 3.5m/12mm 真实几何标定后，同一条链不限流的 Mp 只有 3.69%。
     * 因此限流是**安全/调试旋钮**而非补救措施，默认关闭。
     * 详见 docs/压力闭环控制链再评估-2026-09-16.md §11.7 / §12.1(C)。
     *
     * boost_flow_limit_lmin <= 0 表示关闭（保持 v7 及以前的旧行为，既有回归
     * 用例不受影响）；> 0 时在升压段（error > 0）把输出上限收紧到
     *   Q_allow(e) = Q_ss + (Q_boost - Q_ss) * min(1, e / (P_set*brake_frac))
     * 随误差线性收口，逼近目标时回到稳态维持流量 Q_ss = P_set/K，使等效平衡
     * 压力恰好等于 P_set（零稳态超调），是位置规划器 sqrt(2*a*s) 制动包络在
     * 压力域的对应形式。该上限只会收紧、不会放宽，因此单调安全。
     *
     * v9 可达性约束（修正 D11）：限流值若低于维持流量 P_set/K，则目标压力
     * **永不可达**（实测 ess = K*Q_lim - P_set 闭式成立，无任何诊断输出）。
     * 现在内部会把有效限流抬到不低于 Q_ss —— 即可达性由构造保证。
     * 选值时仍应满足 Q_lim > P_set_max/K（换机型后 K 会变，务必重新核算）。 */
    float boost_flow_limit_lmin;
    float boost_flow_brake_frac;
    float effective_upper_cap;          /* shadow cap [L/min], not consumed in Gate 0 */
    bool effective_upper_cap_valid;     /* shadow cap validity */

} RBF_PID_Handle;

typedef struct {
    float residual;                     /* shadow residual [bar] */
    float g_du;                         /* shadow sensitivity [bar/(L/min)] */
    float last_dt;
    uint32_t valid_sample_count;
    uint32_t invalid_sample_count;
    bool valid;
} RBF_PID_ShadowState;

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

/**
 * @brief v10: 设置"执行器实测可达流量上限"（back-calculation anti-windup）
 * @param pid RBF_PID句柄指针
 * @param cap_lmin 实测可达流量上限 [L/min]，由 pumpSpeedFeedbackRpm / flowToPumpSpeedGain 换算；
 *                 负值（泵反转泄压）按 0 处理
 * @param enable false = 完全退回 v9 行为（默认）；true = 参与软上限钳位
 * @return 无
 *
 * @note 用途：伺服泵实测转速贴上 IEC 配置的转速上限时，PID 若仍要求更大流量，
 *       增量式积分会朝"执行器无法实现的方向"持续累积（windup），
 *       在目标压力回落时表现为长尾超调。把软上限收缩到实测可达流量即可闭合该回路。
 *
 * @note 为何是可达流量而不是"堵转/超时"：可达流量是直接可测的物理边界，
 *       不需要额外的超时/停滞启发式参数（v6 的峰值闸门正是因为靠启发式
 *       而实测失效，见 rbf_pid.c 中 RBF_PID_OVERDRIVE_REL_ERR_GATE 注释）。
 *
 * @note 安全性：本函数只收紧**上限**，不影响下限（负流量泄压路径不受影响）；
 *       enable=false 时不改变任何既有回归基线。
 */
void RBF_PID_SetExternalFlowCap(RBF_PID_Handle *pid, float cap_lmin, bool enable);

/**
 * @brief Publish the effective upper cap for diagnostics/shadow consumers.
 * @note Gate 0 stores the cap only; production output behavior is unchanged.
 */
void RBF_PID_SetEffectiveUpperCap(RBF_PID_Handle *pid, float cap_lmin, bool valid);

/**
 * @brief Update a side-effect-free shadow observation state.
 * @note The const PID handle is never modified and production control state is untouched.
 */
void RBF_PID_ShadowUpdate(RBF_PID_ShadowState *shadow,
                          const RBF_PID_Handle *pid,
                          float setpoint,
                          float feedback,
                          float measured_flow,
                          float dt,
                          bool dt_valid);

/**
 * @brief v8: 设置升压段实验限流上限（升压段必须限流，否则容易超调）
 * @param pid RBF_PID句柄指针
 * @param limit_lmin 升压段允许的最大流量 [L/min]；<= 0 表示关闭限流（旧行为）
 * @return 无
 *
 * @note 为什么需要它（25cc/rev + 1700rpm 实测）：
 *   泵最大流量 42.5 L/min，flowToPumpSpeedGain = 1700/42.5 = 40 rpm/(L/min)，
 *   整机增益 K = 200 bar/(L/min)，故满流量对应的等效建压为 K*Qmax = 8500 bar。
 *   目标 150 bar 时若不加限制，压力以 ~20 bar/ms 冲过目标直到 250 bar 溢流阀
 *   动作（实测峰值 250 bar，超调 66.7%）。限流值应取"实验确定的建压流量"。
 *
 * @note 限流不是单纯硬砍：本实现把上限做成随误差线性收口的制动包络
 *         Q_allow(e) = Q_ss + (Q_boost - Q_ss)*min(1, e/(P_set*brake_frac))
 *       逼近目标时自动回到稳态维持流量 Q_ss = P_set/K，等效平衡压力 = P_set，
 *       因此既不拖慢建压、也不产生稳态超调。它是位置规划器 sqrt(2*a*s)
 *       制动律在压力域的对应形式。
 */
void RBF_PID_SetBoostFlowLimit(RBF_PID_Handle *pid, float limit_lmin);

/**
 * @brief v8: 设置升压限流的制动窗口（占目标压力的比例）
 * @param pid RBF_PID句柄指针
 * @param brake_frac 制动窗口 = brake_frac * P_set，默认 0.5，限制在 [0.02, 10]
 * @return 无
 *
 * @note 窗口越大，越早收口、建压越平缓（超调越小、上升越慢）；
 *       窗口越小，越接近硬限流（建压快但剩余误差靠稳态流量慢慢收）。
 *       物理含义：以目标压力的比例表达"开始刹车"的剩余误差。
 */
void RBF_PID_SetBoostBrakeFrac(RBF_PID_Handle *pid, float brake_frac);

/**
 * @brief v6: 超调驱动流量上限 — 设定值跳变时用于播种 u_prev / Output
 * @param pid RBF_PID句柄指针
 * @param error 当前误差 (P_set - P_actual)，只在其 >0（需升压）时放宽
 * @return min(硬限幅, P_set/K · (1 + GAIN·|e|/P_set) · 1.05)
 *
 * @note 语义：P_set/K 是"维持"该压力所需的稳态流量，不是"建立"它所需流量。
 *       升压过程需要远高于稳态流量的驱动压差，本函数按归一化误差放宽该上限：
 *         远离目标(|e|→P_set)  → 接近硬限幅（满流量快速建压）
 *         接近目标(|e|→0)      → 收敛到稳态维持流量（防积分饱和/防超调）
 *       K<=0（未标定）或 error<=0（泄压方向）时返回硬限幅，保持 v5 及以前的行为。
 *       安全性：上限由 K 决定，K 越大（系统刚性越强）上限越窄，不会盲目满量程冲击。
 */
float RBF_PID_OverdriveFlowCap(const RBF_PID_Handle *pid, float error);

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
