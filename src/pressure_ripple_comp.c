#include "pressure_ripple_comp.h"
#include <math.h>

void HYD_PressureSteadyGate_Reset(HYD_PressureSteadyGateState* s, HYD_UINT16 required, HYD_REAL errThresh) {
    if (s == NULL) return;
    s->counter = 0u;
    s->required = required;
    s->errThresh = errThresh;
}

HYD_UINT8 HYD_PressureSteadyGate_Update(HYD_PressureSteadyGateState* s, HYD_REAL error) {
    if (s == NULL) return 0u;
    if (fabsf((float)error) <= (float)s->errThresh) {
        if (s->counter < 65535u) ++s->counter;
    } else {
        s->counter = 0u;
    }
    return (s->counter >= s->required) ? 1u : 0u;
}

void HYD_PressureRippleComp_Reset(HYD_PressureRippleCompState* s) {
    if (s == NULL) return;
    s->theta = 0.0f;
    s->i1 = 0.0f; s->q1 = 0.0f;
    s->i2 = 0.0f; s->q2 = 0.0f;
    s->a1 = 0.0f; s->phi1 = 0.0f;
    s->sampleCount = 0u;
    s->ffGain = 0.0f;
    s->refreshTick = 0u;
    s->enabled = 1u;
}

void HYD_PressureRippleComp_SetEnabled(HYD_PressureRippleCompState* s, HYD_UINT8 enabled) {
    if (s == NULL) return;
    s->enabled = enabled ? 1u : 0u;
}

void HYD_PressureRippleComp_SetGain(HYD_PressureRippleCompState* s, HYD_REAL gain) {
    if (s == NULL) return;
    s->ffGain = (float)gain;
}

/* 内部：推进归一化转角 theta */
static void HYD_PressureRippleComp_AdvanceTheta(HYD_PressureRippleCompState* s,
                                                HYD_REAL pumpAngleRev, HYD_REAL commandedRpm,
                                                HYD_REAL dt, HYD_UINT8 useEncoder) {
    if (useEncoder) {
        s->theta = (float)pumpAngleRev;
    } else {
        s->theta += (HYD_REAL)HYD_PUMP_TEETH * (float)commandedRpm * (float)dt / 60.0f;
    }
    s->theta -= (HYD_REAL)(int)s->theta;   /* wrap to [0,1) */
    if (s->theta < 0.0f) s->theta += 1.0f;
}

/* 内部：由复解调累加器求基波幅值/相位（低频，每 REFRESH_TICKS 一次）。
 * EMA 稳态：i1_ss=(A/2)sinφ, q1_ss=(A/2)cosφ => a1=2√(i1²+q1²)=A，与样本数/相位无关，
 * 无频谱泄漏；EMA 累加器不在刷新时清零，持续跟踪漂移。2 次谐波(i2,q2)暂仅累加备用。 */
static void HYD_PressureRippleComp_Refresh(HYD_PressureRippleCompState* s) {
    if (s->sampleCount < (HYD_UINT16)HYD_RIPPLE_MIN_SAMPLES) return;
    s->a1 = 2.0f * sqrtf(s->i1 * s->i1 + s->q1 * s->q1);
    s->phi1 = atan2f(s->i1, s->q1);
}

void HYD_PressureRippleComp_Update(HYD_PressureRippleCompState* s,
                                   HYD_REAL eP, HYD_REAL pumpAngleRev,
                                   HYD_REAL commandedRpm, HYD_REAL dt,
                                   HYD_UINT8 useEncoder, HYD_UINT8 steadyGate) {
    if (s == NULL) return;
    if (!s->enabled) return;   /* 关闭：零计算 */

    HYD_PressureRippleComp_AdvanceTheta(s, pumpAngleRev, commandedRpm, dt, useEncoder);

    if (steadyGate) {
        /* EMA 复解调：相位 θ = Z·theta（基波），2Z·theta（2 次谐波）。
           用 theta 的整圈归一化相位，避免 long-run 相位溢出。
           i += α(eP·cosθ_i - i) 为一阶 IIR 窄带锁定，稳态增益 1，无窗口泄漏。 */
        HYD_REAL ph1 = HYD_RIPPLE_TWO_PI * (HYD_REAL)HYD_PUMP_TEETH * s->theta;
        HYD_REAL ph2 = 2.0f * ph1;
        HYD_REAL w = (HYD_REAL)HYD_RIPPLE_EMA_W;
        HYD_REAL c1 = cosf(ph1), s1 = sinf(ph1);
        HYD_REAL c2 = cosf(ph2), s2 = sinf(ph2);
        s->i1 += w * ((float)eP * c1 - s->i1);
        s->q1 += w * ((float)eP * s1 - s->q1);
        s->i2 += w * ((float)eP * c2 - s->i2);
        s->q2 += w * ((float)eP * s2 - s->q2);
        if (s->sampleCount < 65535u) ++s->sampleCount;
    }

    /* 低频刷新基波幅相（仅此处有 atan2f，刷新频率极低） */
    if (++s->refreshTick >= (HYD_UINT16)HYD_RIPPLE_REFRESH_TICKS) {
        s->refreshTick = 0u;
        HYD_PressureRippleComp_Refresh(s);
    }
}

HYD_REAL HYD_PressureRippleComp_GetFF(const HYD_PressureRippleCompState* s,
                                      HYD_REAL pumpAngleRev, HYD_REAL commandedRpm,
                                      HYD_REAL dt, HYD_UINT8 useEncoder) {
    (void)pumpAngleRev; (void)commandedRpm; (void)dt; (void)useEncoder;
    if (s == NULL || !s->enabled) return 0.0f;            /* 关闭：返回 0 */
    if (s->sampleCount < (HYD_UINT16)HYD_RIPPLE_MIN_SAMPLES) return 0.0f;  /* 未学习 */
    /* 解析式还原基波前馈：ff = -ffGain·a1·sin(2π·Z·θ + φ₁)。
       反相抵消：系统压力误差中的齿频分量 ≈ a1·sin(2πZθ+φ₁)，经 ffGain(bar->L/min)
       换向后由泵产生等量反向压力脉动，抵消之。热路径仅一次 sinf（STM32H7 FPU 可忽略）。 */
    HYD_REAL ph = HYD_RIPPLE_TWO_PI * (HYD_REAL)HYD_PUMP_TEETH * s->theta + s->phi1;
    return -s->ffGain * s->a1 * sinf(ph);
}
