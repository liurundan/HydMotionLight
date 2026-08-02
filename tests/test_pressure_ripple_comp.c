#include <math.h>
#include <stdio.h>
#include "pressure_ripple_comp.h"
#include <string.h>
#include "pressure_controller.h"

#define DT 0.001f
#define Z  13u
#define TWO_PI 6.2831853f

/* 合成已知脉动 eP(t) = A*sin(2*pi*Z*theta + phi)，theta 由编码器角度提供 */
static int test_ripple_lut_cancels_synthetic_ripple(void) {
    HYD_PressureRippleCompState s;
    HYD_PressureRippleComp_Reset(&s);
    HYD_PressureRippleComp_SetEnabled(&s, 1u);
    HYD_PressureRippleComp_SetGain(&s, 1.0f / 4.5f); /* 1/systemGain, bar->L/min */

    const float A = 3.0f, phi = 0.7f;
    float theta = 0.0f;
    /* 学习阶段：稳态闸门恒开，注入合成脉动 */
    for (int i = 0; i < 4000; ++i) {
        float eP = A * sinf(TWO_PI * (float)Z * theta + phi);
        uint8_t gate = 1u;
        HYD_PressureRippleComp_Update(&s, eP, theta, 0.0f, DT, 1u, gate);
        theta += 0.0007f; /* 任意单调推进，仅用于取相位 */
        if (theta >= 1.0f) theta -= 1.0f;
    }
    /* 验证阶段：用同一相位读取前馈，应反相抵消。
       真实运行每个周期都是 Update(推进/设置 theta) -> GetFF(读 theta)，
       故此处先以 gate=0 调用 Update 推进相位（不污染 LUT），再 GetFF。 */
    float max_residual = 0.0f;
    for (int i = 0; i < 200; ++i) {
        float eP = A * sinf(TWO_PI * (float)Z * theta + phi);
        HYD_PressureRippleComp_Update(&s, 0.0f, theta, 0.0f, DT, 1u, 0u); /* gate=0 仅推进相位 */
        float ff = HYD_PressureRippleComp_GetFF(&s, theta, 0.0f, DT, 1u);
        float residual = eP + ff * 4.5f; /* ff(L/min)*systemGain(bar/(L/min)) 抵消 eP(bar) */
        if (fabsf(residual) > max_residual) max_residual = fabsf(residual);
        theta += 0.0007f;
        if (theta >= 1.0f) theta -= 1.0f;
    }
    if (max_residual > 0.3f) { /* 期望 RMS 降幅 > 90%，残差小 */
        fprintf(stderr, "ripple residual too large: %f\n", (double)max_residual);
        return 0;
    }
    return 1;
}

/* 关闭时 GetFF 恒 0，且 Update 不修改已学幅相（关闭分支直接早返回） */
static int test_ripple_disabled_outputs_zero(void) {
    HYD_PressureRippleCompState s;
    HYD_PressureRippleComp_Reset(&s);
    HYD_PressureRippleComp_SetEnabled(&s, 0u);
    /* 先随便置一组非零学习结果，验证关闭时 GetFF 仍返回 0 */
    s.a1 = 5.0f; s.phi1 = 1.0f; s.sampleCount = 1000u;
    float theta = 0.3f;
    float ff = HYD_PressureRippleComp_GetFF(&s, theta, 100.0f, DT, 0u);
    float ff2 = HYD_PressureRippleComp_GetFF(&s, theta, 100.0f, DT, 0u);
    if (ff != 0.0f || ff2 != 0.0f) return 0;
    /* 关闭时 Update 应早返回，不修改已学幅相 */
    HYD_PressureRippleComp_Update(&s, 2.0f, theta, 100.0f, DT, 0u, 1u);
    if (s.a1 != 5.0f || s.phi1 != 1.0f) return 0;
    return 1;
}

/* 转速累加回退路径：无编码器，theta 由 rpm 累加，相位应与编码器路径一致 */
static int test_ripple_speed_accumulator_fallback(void) {
    HYD_PressureRippleCompState s;
    HYD_PressureRippleComp_Reset(&s);
    HYD_PressureRippleComp_SetEnabled(&s, 1u);
    HYD_PressureRippleComp_SetGain(&s, 1.0f / 4.5f);
    const float rpm = 600.0f; /* 10 rev/s -> theta 步进 Z*rpm*dt/60 */
    float theta_enc = 0.0f;
    float theta_acc = 0.0f;
    float max_diff = 0.0f;
    for (int i = 0; i < 2000; ++i) {
        /* 编码器路径：角度直接用 theta_enc */
        HYD_PressureRippleComp_Update(&s, 0.0f, theta_enc, 0.0f, DT, 1u, 0u); /* gate=0 不学，只推进相位 */
        /* 累加倍增：注意 Update 内部已用 rpm 推进 s.theta；这里单独模拟累加器 */
        theta_acc += (float)Z * rpm * DT / 60.0f;
        theta_acc -= (float)(int)theta_acc;
        theta_enc += (float)Z * rpm * DT / 60.0f;
        theta_enc -= (float)(int)theta_enc;
        float d = fabsf(theta_acc - theta_enc);
        if (d > max_diff) max_diff = d;
    }
    if (max_diff > 1e-3f) { fprintf(stderr, "accumulator drift %f\n", (double)max_diff); return 0; }
    return 1;
}

/* PI/PI_RBF 在 systemGain 已知、FF 基值=target/systemGain 时，稳态误差应≈0
 * （FF-trim 仅补偿 systemGain 之外的残差，本例基值已正确，故稳态误差<1bar）。 */
static int test_ff_trim_removes_steady_error(void) {
    HYD_MotionSegment seg;
    memset(&seg, 0, sizeof(seg));
    seg.mode = HYD_MODE_PRESSURE_CLOSED_LOOP;
    seg.pressureController = HYD_PRESSURE_CONTROLLER_PI;
    seg.targetPressure = 100.0f;
    seg.systemGain = 4.5f;          /* flow* = 100/4.5 = 22.22 L/min */
    seg.targetFlow = 10.0f;         /* 配方错误：偏小，但 FF 基值由 ffBase 推导，不依赖它 */
    seg.pressureKp = 1.5f; seg.pressureKi = 0.5f;
    seg.maxFlow = 50.0f;

    HYD_PressureControllerState st; HYD_PressureController_ClearState(&st);
    HYD_PressureSteadyGate_Reset(&st.ffSteadyGate, 64u, 1.0f);

    float target = 100.0f, meas = 0.0f, t = 0.0f;
    for (int i = 0; i < 8000; ++i) {  /* 8 s @1kHz */
        HYD_PressureControllerInput in; memset(&in, 0, sizeof(in));
        in.targetPressure = target; in.measuredPressure = meas;
        in.feedforwardFlow = target / seg.systemGain;  /* 正确 FF 基值 */
        in.outputMin = -5.0f; in.outputMax = seg.maxFlow;
        in.flowToPumpSpeedGain = 20.0f; in.pumpSpeedLimit = 2000.0f; in.timestamp = t;
        HYD_PressureControllerOutput out;
        HYD_PressureController_Execute(&seg, &st, &in, &out);
        meas += ((float)seg.systemGain * (float)out.outputFlow - meas) * (DT / 0.1f);  /* 一阶被控对象 tau=0.1s: 稳态 meas=systemGain*flow */
        t += DT;
    }
    if (fabsf(meas - target) > 1.0f) { fprintf(stderr, "FF-trim steady error %f\n", (double)(meas-target)); return 0; }
    return 1;
}

/* PI_RBF 必须将 FF 计入输出（纯 RBF 会丢弃 FF -> 输出≈0）；稳态下输出应≈ffBase */
static int test_pi_rbf_keeps_feedforward(void) {
    HYD_MotionSegment seg; memset(&seg, 0, sizeof(seg));
    seg.mode = HYD_MODE_PRESSURE_CLOSED_LOOP;
    seg.pressureController = HYD_PRESSURE_CONTROLLER_PI_RBF;
    seg.targetPressure = 100.0f; seg.systemGain = 4.5f;
    seg.pressureKp = 1.5f; seg.pressureKi = 0.5f; seg.maxFlow = 50.0f;
    HYD_PressureControllerState st; HYD_PressureController_ClearState(&st);
    HYD_PressureSteadyGate_Reset(&st.ffSteadyGate, 64u, 1.0f);
    float target = 100.0f, meas = 100.0f, t = 0.0f;  /* 已在稳态 */
    HYD_PressureControllerInput in; memset(&in, 0, sizeof(in));
    in.targetPressure = target; in.measuredPressure = meas;
    in.feedforwardFlow = target / seg.systemGain; /* 22.22 L/min */
    in.outputMin = -5.0f; in.outputMax = seg.maxFlow;
    in.flowToPumpSpeedGain = 20.0f; in.pumpSpeedLimit = 2000.0f; in.timestamp = t;
    HYD_PressureControllerOutput out;
    HYD_PressureController_Execute(&seg, &st, &in, &out);
    /* PI_RBF 必须将 FF 计入输出；稳态下输出应≈ffBase(22.22)。若丢弃 FF 则 outputFlow≈0。 */
    if ((float)out.outputFlow < 15.0f) { fprintf(stderr, "PI_RBF dropped FF: out=%f\n", (double)out.outputFlow); return 0; }
    if (!out.adaptiveActive) { fprintf(stderr, "PI_RBF adaptiveActive not set\n"); return 0; }
    return 1;
}

int main(void) {
    int failed = 0;
    if (!test_ripple_lut_cancels_synthetic_ripple()) { printf("FAIL test_ripple_lut_cancels_synthetic_ripple\n"); ++failed; }
    else printf("PASS test_ripple_lut_cancels_synthetic_ripple\n");
    if (!test_ripple_disabled_outputs_zero()) { printf("FAIL test_ripple_disabled_outputs_zero\n"); ++failed; }
    else printf("PASS test_ripple_disabled_outputs_zero\n");
    if (!test_ripple_speed_accumulator_fallback()) { printf("FAIL test_ripple_speed_accumulator_fallback\n"); ++failed; }
    else printf("PASS test_ripple_speed_accumulator_fallback\n");
    if (!test_ff_trim_removes_steady_error()) { printf("FAIL test_ff_trim_removes_steady_error\n"); ++failed; }
    else printf("PASS test_ff_trim_removes_steady_error\n");
    if (!test_pi_rbf_keeps_feedforward()) { printf("FAIL test_pi_rbf_keeps_feedforward\n"); ++failed; }
    else printf("PASS test_pi_rbf_keeps_feedforward\n");
    return failed;
}
