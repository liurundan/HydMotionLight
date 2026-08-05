#include "pressure_ripple_comp.h"
#include <math.h>

#define HYD_RIPPLE_DC_EMA_W 0.005f

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
    s->a2 = 0.0f; s->phi2 = 0.0f;
    s->dcEstimate = 0.0f;
    s->dcInitialized = false;
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
    s->ffGain = (isfinite((float)gain) && gain > 0.0f) ? (float)gain : 0.0f;
}

/* 内部：推进归一化转角 theta */
static void HYD_PressureRippleComp_AdvanceTheta(HYD_PressureRippleCompState* s,
                                                HYD_REAL pumpAngleRev, HYD_REAL commandedRpm,
                                                HYD_REAL dt, HYD_UINT8 useEncoder) {
    if (useEncoder && isfinite((float)pumpAngleRev)) {
        s->theta = (HYD_REAL)fmod((double)pumpAngleRev, 1.0);
        if (s->theta < 0.0f) s->theta += 1.0f;
    } else {
        HYD_REAL safeRpm = isfinite((float)commandedRpm) ? commandedRpm : 0.0f;
        HYD_REAL safeDt = (isfinite((float)dt) && dt > 0.0f) ? dt : 0.0f;
        /* theta is the mechanical shaft revolution fraction. The tooth
         * count is applied once below when constructing the tooth phase. */
        s->theta += safeRpm * safeDt / 60.0f;
    }
    if (!isfinite((float)s->theta)) s->theta = 0.0f;
    s->theta = (HYD_REAL)fmod((double)s->theta, 1.0);
    if (s->theta < 0.0f) s->theta += 1.0f;
}

/* 内部：由复解调累加器求基波幅值/相位（低频，每 REFRESH_TICKS 一次）。
 * EMA 稳态：i1_ss=(A/2)sinφ, q1_ss=(A/2)cosφ => a1=2√(i1²+q1²)=A，与样本数/相位无关，
 * 无频谱泄漏；EMA 累加器不在刷新时清零，持续跟踪漂移。2 次谐波(i2,q2)暂仅累加备用。 */
static void HYD_PressureRippleComp_Refresh(HYD_PressureRippleCompState* s) {
    if (s->sampleCount < (HYD_UINT16)HYD_RIPPLE_MIN_SAMPLES) return;
    s->a1 = 2.0f * sqrtf(s->i1 * s->i1 + s->q1 * s->q1);
    s->phi1 = atan2f(s->i1, s->q1);
    s->a2 = 2.0f * sqrtf(s->i2 * s->i2 + s->q2 * s->q2);
    s->phi2 = atan2f(s->i2, s->q2);
}

void HYD_PressureRippleComp_Update(HYD_PressureRippleCompState* s,
                                   HYD_REAL eP, HYD_REAL pumpAngleRev,
                                   HYD_REAL commandedRpm, HYD_REAL dt,
                                   HYD_UINT8 useEncoder, HYD_UINT8 steadyGate) {
    if (s == NULL) return;
    if (!s->enabled) return;   /* 关闭：零计算 */

    HYD_PressureRippleComp_AdvanceTheta(s, pumpAngleRev, commandedRpm, dt, useEncoder);

    if (!isfinite((float)eP)) {
        eP = 0.0f;
        steadyGate = 0u;
    }

    if (steadyGate) {
        HYD_REAL residual;

        /* Remove slow pressure bias before synchronous demodulation so load
         * error and gain mismatch are not learned as a pump harmonic. */
        if (!s->dcInitialized || !isfinite((float)s->dcEstimate)) {
            s->dcEstimate = eP;
            s->dcInitialized = true;
        } else {
            s->dcEstimate += (HYD_REAL)HYD_RIPPLE_DC_EMA_W *
                             (eP - s->dcEstimate);
        }
        residual = eP - s->dcEstimate;

        /* EMA 复解调：相位 θ = Z·theta（基波），2Z·theta（2 次谐波）。
           用 theta 的整圈归一化相位，避免 long-run 相位溢出。
           i += α(eP·cosθ_i - i) 为一阶 IIR 窄带锁定，稳态增益 1，无窗口泄漏。 */
        HYD_REAL ph1 = HYD_RIPPLE_TWO_PI * (HYD_REAL)HYD_PUMP_TEETH * s->theta;
        HYD_REAL ph2 = 2.0f * ph1;
        HYD_REAL w = (HYD_REAL)HYD_RIPPLE_EMA_W;
        HYD_REAL c1 = cosf(ph1), s1 = sinf(ph1);
        HYD_REAL c2 = cosf(ph2), s2 = sinf(ph2);
        s->i1 += w * ((float)residual * c1 - s->i1);
        s->q1 += w * ((float)residual * s1 - s->q1);
        s->i2 += w * ((float)residual * c2 - s->i2);
        s->q2 += w * ((float)residual * s2 - s->q2);
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
    HYD_REAL ff;

    (void)pumpAngleRev; (void)commandedRpm; (void)dt; (void)useEncoder;
    if (s == NULL || !s->enabled) return 0.0f;            /* 关闭：返回 0 */
    if (s->sampleCount < (HYD_UINT16)HYD_RIPPLE_MIN_SAMPLES) return 0.0f;  /* 未学习 */
    /* 解析式还原基波前馈：ff = -ffGain·a1·sin(2π·Z·θ + φ₁)。
       反相抵消：系统压力误差中的齿频分量 ≈ a1·sin(2πZθ+φ₁)，经 ffGain(bar->L/min)
       换向后由泵产生等量反向压力脉动，抵消之。热路径仅一次 sinf（STM32H7 FPU 可忽略）。 */
    if (!isfinite((float)s->theta) || !isfinite((float)s->a1) ||
        !isfinite((float)s->phi1) || !isfinite((float)s->a2) ||
        !isfinite((float)s->phi2) || !isfinite((float)s->ffGain)) {
        return 0.0f;
    }
    {
        HYD_REAL basePhase = HYD_RIPPLE_TWO_PI * (HYD_REAL)HYD_PUMP_TEETH * s->theta;
        HYD_REAL ph1 = basePhase + s->phi1;
        HYD_REAL ph2 = 2.0f * basePhase + s->phi2;
        /* The second harmonic is useful for the pump tooth-drop waveform but
         * is intentionally weighted down to keep noisy phase estimates from
         * injecting a large command during convergence. */
        ff = -s->ffGain * (s->a1 * sinf(ph1) + 0.5f * s->a2 * sinf(ph2));
    }
    if (!isfinite((float)ff)) return 0.0f;
    return HYD_ClampReal(ff, -HYD_RIPPLE_MAX_FF_FLOW, HYD_RIPPLE_MAX_FF_FLOW);
}
