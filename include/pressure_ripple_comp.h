#ifndef HYD_PRESSURE_RIPPLE_COMP_H
#define HYD_PRESSURE_RIPPLE_COMP_H

#include "common_types.h"

/* 齿轮泵齿数（与 PressureModel.c 的 13 一致） */
#define HYD_PUMP_TEETH 13u
/* LUT 刷新节拍（每 N 周期刷新一次，低频；仅此处做 atan2f 求幅相） */
#define HYD_RIPPLE_REFRESH_TICKS 256u
/* EMA 复解调权重：~1/α 样本为有效平均窗长（α=0.002 => ~500 样本=~4.5 个脉动周期）。
   必须让平均窗 >> 脉动周期（实测 0.02 时窗长仅~50 样本<110 样本/周期，2 倍频解调分量
   仅被衰减到~18%，残留在 i1/q1 中污染幅相，导致残余脉动过大）。调小 α 使 2×(2πZθ)
   分量被充分低通（α=0.002 时 2 倍频增益≈1.8%，幅相估计近似无偏）。
   稳态增益=1，无有限窗频谱泄漏。 */
#define HYD_RIPPLE_EMA_W 0.002f
/* EMA 收敛所需最少稳态样本数（不足则 GetFF 返回 0，避免未学习就补偿） */
#define HYD_RIPPLE_MIN_SAMPLES 200u
#define HYD_RIPPLE_TWO_PI 6.2831853f
#define HYD_RIPPLE_MAX_FF_FLOW 5.0f

/* 通用稳态闸门状态（FF 微调与脉动校准共用，零 RAM 窗口） */
typedef struct {
    HYD_UINT16 counter;       /* 连续满足误差阈值的采样数 */
    HYD_UINT16 required;      /* 触发所需连续样本数 */
    HYD_REAL errThresh;       /* 误差阈值 [bar] */
} HYD_PressureSteadyGateState;

/* 脉动补偿状态（静态预分配，极小 RAM：仅 4 个复解调累加器 + 已学基波幅相）。
 * 设计取舍：早期版本用 208 项 LUT（≈832B）做热路径查表以避免 sinf，但在 STM32H7
 * 内存预算（FB 结构体 <=~3.2KB）下不可接受。改用解析式——EMA 复解调已学到齿频基波
 * 的幅值 a1 与相位 phi1，GetFF 直接用单一 sinf 还原前馈；对单频正弦是精确的，
 * 且状态仅 ~40B，热路径一次 sinf 在 H7 FPU 上开销可忽略。 */
typedef struct {
    HYD_REAL theta;                          /* 泵轴归一化转角 [0,1) */
    /* 复解调累加器（稳态学习期累积，刷新期求幅相；i2/q2 为 2 次谐波备用） */
    HYD_REAL i1, q1;                         /* 基波 Z */
    HYD_REAL i2, q2;                         /* 2 次谐波 2Z */
    HYD_REAL a1;                             /* 学习到的基波幅值 [bar] */
    HYD_REAL phi1;                           /* 学习到的基波相位 [rad] */
    HYD_REAL a2;                             /* 学习到的二次谐波幅值 [bar] */
    HYD_REAL phi2;                           /* 学习到的二次谐波相位 [rad] */
    HYD_REAL dcEstimate;                     /* slow pressure-error baseline */
    HYD_BOOL dcInitialized;
    HYD_UINT16 sampleCount;                  /* 当前累计样本数 */
    HYD_REAL ffGain;                         /* = 1/systemGain，压力->流量换算 */
    HYD_UINT16 refreshTick;                  /* 刷新节拍计数（>255，须 uint16） */
    HYD_UINT8 enabled;                       /* 段级开关镜像 */
} HYD_PressureRippleCompState;

void HYD_PressureSteadyGate_Reset(HYD_PressureSteadyGateState* s, HYD_UINT16 required, HYD_REAL errThresh);
HYD_UINT8 HYD_PressureSteadyGate_Update(HYD_PressureSteadyGateState* s, HYD_REAL error);

void HYD_PressureRippleComp_Reset(HYD_PressureRippleCompState* s);
void HYD_PressureRippleComp_SetEnabled(HYD_PressureRippleCompState* s, HYD_UINT8 enabled);
void HYD_PressureRippleComp_SetGain(HYD_PressureRippleCompState* s, HYD_REAL gain);
/* 每周期调用：推进相位、稳态期复解调累加脉动、低频刷新基波幅相(a1,phi1)。
   eP: 压力误差 [bar]；pumpAngleRev: 编码器角度[rev]（useEncoder=1 时使用）；
   commandedRpm: 上周期泵转速[rpm]（回退累加用）；dt: 采样周期[s]；
   useEncoder: 1=用编码器角度，0=转速累加；steadyGate: 1=稳态可学习 */
void HYD_PressureRippleComp_Update(HYD_PressureRippleCompState* s,
                                   HYD_REAL eP, HYD_REAL pumpAngleRev,
                                   HYD_REAL commandedRpm, HYD_REAL dt,
                                   HYD_UINT8 useEncoder, HYD_UINT8 steadyGate);
/* 热路径：按当前 theta 以单一 sinf 还原基波前馈 [L/min] = -ffGain·a1·sin(2π·Z·θ+φ₁)；
   关闭、或样本不足(MIN_SAMPLES)时返回 0。pumpAngleRev/commandedRpm/dt/useEncoder 仅作
   接口兼容保留（相位已由 Update 推进并存入 s->theta）。 */
HYD_REAL HYD_PressureRippleComp_GetFF(const HYD_PressureRippleCompState* s,
                                      HYD_REAL pumpAngleRev, HYD_REAL commandedRpm,
                                      HYD_REAL dt, HYD_UINT8 useEncoder);

#endif /* HYD_PRESSURE_RIPPLE_COMP_H */
