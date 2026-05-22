/**
 * @file rbf_pid.h
 * @brief RBF神经网络自适应PID控制器 - 嵌入式C实现
 * @note 基于ST代码转换，适用于ARM Cortex-M等平台
 */

#ifndef RBF_PID_H
#define RBF_PID_H

#include <stdbool.h>
#include <stdint.h>

/* 常数定义 */
#define RBF_HNUM            6       // 隐含层节点数
#define RBF_INPUT_DIM       3       // 输入维度：du_prev, Feedback, y_prev1
#define RBF_MOMENTUM_STEPS  3       // 动量历史步数

/* PID参数限幅（与ST代码一致） */
#define PID_MIN_KP          0.8f
#define PID_MAX_KP          0.85f
#define PID_MIN_KI          0.018f
#define PID_MAX_KI          0.03f
#define PID_MIN_KD          1.2f
#define PID_MAX_KD          1.5f

/* 输出限幅 */
#define MIN_OUTPUT          -0.05f
#define MAX_PRESSURE        250.0f
#define MAX_MOTOR_SPEED     2000.0f

/**
 * @brief RBF-PID控制器状态结构体
 * @note 所有状态变量均内聚在此，支持多实例
 */
typedef struct {
    /* 输入参数（用户配置） */
    bool enable;                    // 使能信号
    float P_set;                    // 压力设定值(原始单位)
    float P_actual;                 // 压力反馈值(原始单位)
    bool auto_tune;                 // 自动调谐标志(未完全实现)
    float sampling_period;          // 采样时间(s)
    float fMaxMotorSpeed;           // 电机最大转速
    float fMaxFlowRate;             // 最大流量(0-1)
    bool Reset;                     // 复位信号

    /* 压力归一化标量（MPa 等设定/反馈单位的满量程）.
     * 0 或负值 -> 落回内置默认 MAX_PRESSURE. 调用 RBF_PID_SetPressureNormalization()
     * 配置；推荐由 pressure_controller.c 在每段 Resolve 时根据段配置写入。 */
    float pressure_normalization_scale;

    /* 系统模型参数(用于未来扩展) */
    float T_d;                      // 纯时滞时间
    float K;                        // 系统增益
    float T;                        // 惯性时间常数

    /* 输出变量 */
    float Output;                   // 控制器原始输出
    float Setpoint;                 // 归一化设定值
    float Feedback;                 // 归一化反馈值
    float KP;                       // 比例系数
    float KI;                       // 积分系数
    float KD;                       // 微分系数
    float du;                       // 本次控制增量
    float Error;                    // 当前误差
    float Jacobian;                 // Jacobian估计值
    float min_KP;                   // 比例系数下限
    float max_KP;                   // 比例系数上限
    float min_KI;                   // 积分系数下限
    float max_KI;                   // 积分系数上限
    float min_KD;                   // 微分系数下限
    float max_KD;                   // 微分系数上限
    int32_t Status;                 // 状态代码(1:运行中, -1:未使能)
    int32_t TuneResult;             // 调谐结果标志
    float n_out;                    // 输出*最大转速

    /* RBF神经网络参数 */
    float c[RBF_HNUM][RBF_INPUT_DIM];   // 中心向量
    float b_rbf[RBF_HNUM];              // 宽度
    float w[RBF_HNUM];                  // 权重

    /* 学习率参数 */
    float eta_w;                    // 权重学习率(0.25)
    float eta_c;                    // 中心学习率(0.25)
    float eta_b;                    // 宽度学习率(0.25)
    float eta_p;                    // P参数学习率(0.25)
    float eta_i;                    // I参数学习率(0.25)
    float eta_d;                    // D参数学习率(0.25)

    /* 动量因子 */
    float alpha;                    // 动量因子(0.05)
    float belte;                    // 二次动量因子(0.01)

    /* 历史数据存储(动量更新) */
    float ci_1[RBF_HNUM][RBF_INPUT_DIM];
    float ci_2[RBF_HNUM][RBF_INPUT_DIM];
    float ci_3[RBF_HNUM][RBF_INPUT_DIM];
    float bi_1[RBF_HNUM];
    float bi_2[RBF_HNUM];
    float bi_3[RBF_HNUM];
    float w_1[RBF_HNUM];
    float w_2[RBF_HNUM];
    float w_3[RBF_HNUM];

    /* 控制器状态变量 */
    float u_prev;                   // 上一次控制输出
    float e_prev1;                  // 前一次误差
    float e_prev2;                  // 前两次误差
    float du_prev;                  // 上一次控制增量
    float y_prev1;                  // 前一次反馈值
    float delta_temp_prev;          // 微分滤波前值

    /* 中间计算变量(避免重复定义) */
    float h[RBF_HNUM];              // 隐含层输出
    float y_hat;                    // RBF预测输出

    /* 前馈控制相关 */
    float fLastActPress;            // 上一次压力反馈
    float fLastActPress2;           // 上上次压力反馈
    float fUffAcc;                  // 加速度前馈量
    bool EnableFF;                  // 前馈使能
    float last_ref;                 // 上一次设定值
    float fKff_a_pos;               // 正向加速度前馈系数(0.7)
    float fKff_a_neg;               // 负向加速度前馈系数(0.32)
    float fKSetpoint;               // 设定值变化率前馈增益(0.3)
    float fBaseBias;                // 静态偏置(0.00001)

    /* 自适应学习率 */
    float eta_scale;                // 误差驱动的学习率缩放因子 [0.01, 1.0]

    /* 网络初始化种子 */
    uint32_t network_seed;          // 可配置的RBF网络初始化种子

    /* 初始化标志 */
    bool FirstScan;

    /* 内部临时变量(非持久化，但为方便而放于结构体) */
    float rand_seed;                // 随机数种子(实际使用线性同余)
} RBF_PID_Handle;

/**
 * @brief 初始化RBF-PID控制器
 * @param pid RBF_PID句柄指针
 * @param sampling_period 采样时间(s)
 * @param max_motor_speed 最大电机转速
 * @param max_flow_rate 最大流量(0~1)
 * @return 无
 */
void RBF_PID_Init(RBF_PID_Handle *pid, float sampling_period, 
                  float max_motor_speed, float max_flow_rate);

/**
 * @brief 执行RBF-PID控制计算
 * @param pid RBF_PID句柄指针
 * @param setpoint 设定值(压力, 原始单位)
 * @param feedback 反馈值(压力, 原始单位)
 * @return 控制输出(归一化流量, 范围[-0.05, max_flow_rate])
 */
float RBF_PID_Update(RBF_PID_Handle *pid, float setpoint, float feedback);

/**
 * @brief 复位控制器(清空所有历史状态)
 * @param pid RBF_PID句柄指针
 */
void RBF_PID_Reset(RBF_PID_Handle *pid);

/**
 * @brief 设置PID参数限幅(可选，默认使用内部宏)
 * @param pid RBF_PID句柄指针
 * @param min_kp, max_kp, min_ki, max_ki, min_kd, max_kd
 */
void RBF_PID_SetParamLimits(RBF_PID_Handle *pid,
    float min_kp, float max_kp, float min_ki, float max_ki,
    float min_kd, float max_kd);

/**
 * @brief 设置学习率
 * @param pid 句柄
 * @param eta_w, eta_c, eta_b, eta_p, eta_i, eta_d
 */
void RBF_PID_SetLearningRates(RBF_PID_Handle *pid,
    float eta_w, float eta_c, float eta_b,
    float eta_p, float eta_i, float eta_d);

/**
 * @brief 配置压力归一化标量
 * @param pid RBF_PID句柄指针
 * @param scale 满量程标量（单位与 setpoint/feedback 相同，例如 MPa）
 *              传 0 或负值会清回内部默认 MAX_PRESSURE.
 * @note 推荐在每段开始时调用一次；运行中改变会导致归一化基准跳变。
 */
void RBF_PID_SetPressureNormalization(RBF_PID_Handle *pid, float scale);

void RBF_PID_SetSeed(RBF_PID_Handle *pid, uint32_t seed);

#endif /* RBF_PID_H */