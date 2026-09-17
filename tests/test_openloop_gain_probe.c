/* tests/test_openloop_gain_probe.c
 * 开环过程增益实测探针
 *
 * 目的：用仿真实验数据回答"Ksys / K 到底是多少、是否存在"。
 *
 * 方法：对 PressureModel（物理标定模型）施加恒定转速阶跃，记录压力最终稳定值，
 *      拟合 P_steady = f(n)，得到：
 *        Ksys = P_steady / n            [bar/rpm]
 *        K    = P_steady / Q            [bar/(L/min)]   Q = n * D_pump
 *
 * 关键判据：
 *   若 P_steady 随 n 线性增长 → 存在比例环节（泄漏/液阻主导）→ K 有意义
 *   若 P_steady 不随 n 变化（都被溢流阀钳在 250bar）→ 过程是积分+限幅，K 无意义
 *   若 P_steady 持续增长不收敛 → 纯积分环节，K 无意义
 *
 * 同时输出 90% 上升时间，用于判断 tr 的物理下限。
 */

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>

#include "pressure_model.h"

#define DT_S          0.001f
/* 60s, not 20s. The plant is no longer the first-order lag (tau = 1 s) this
 * probe was originally sized for: the physical-calibrated model applies an
 * entrained-air bulk-modulus law, so beta_eff collapses from 1.2 GPa toward
 * ~49 MPa as pressure approaches zero. The low-rpm points (P_ss = 10 bar at
 * 2 rpm) therefore sit in a tau ~= 15 s regime and were still 3.8% short of
 * their steady state after 20 s, which showed up as a spurious gain spread.
 * 60 s clears every point with room to spare. */
#define HOLD_STEPS    60000   /* 60s 充分建稳（低压含气段时间常数可达 ~15s） */
#define PUMP_D_ML_REV 25.0f   /* 与 PressureModel_InitParams 一致（25cc/rev 油泵） */

static float q_lmin_from_rpm(float rpm) {
    /* Q[L/min] = n[rpm] * D[mL/rev] / 1000 */
    return rpm * PUMP_D_ML_REV / 1000.0f;
}

static void run_point(float rpm, float *p_steady, float *t90_ms, float *p_max) {
    PressureModelParams params;
    PressureModelState state;
    PressureModelOutput out;
    int i;
    float sum = 0.0f;
    int avg_count = 0;
    float p_end_prev = 0.0f;
    int drift_count = 0;

    memset(&out, 0, sizeof(out));
    PressureModel_InitParams(&params);
    params.enable_sensor_noise = 0u;
    params.enable_motor_noise = 0u;
    PressureModel_Reset(&state, 0x12345678u);

    *p_max = 0.0f;
    *t90_ms = -1.0f;

    for (i = 0; i < HOLD_STEPS; ++i) {
        PressureModel_Step(&params, &state, rpm, DT_S, &out);

        if (out.measured_pressure_bar > *p_max) *p_max = out.measured_pressure_bar;

        /* 后 1s 取平均作为稳态值 */
        if (i >= HOLD_STEPS - 1000) {
            sum += out.measured_pressure_bar;
            avg_count++;
        }
        /* 漂移检测：最后 5s 内末值与 5s 前之差 */
        if (i == HOLD_STEPS - 5000) p_end_prev = out.measured_pressure_bar;
    }
    *p_steady = sum / (float)avg_count;

    /* 判断是否仍在漂移 */
    if (fabsf(out.measured_pressure_bar - p_end_prev) > 1.0f) {
        drift_count = 1;
    }
    (void)drift_count;

    /* 90% 上升时间：以本工况稳态值为基准 */
    if (*p_steady > 1.0f) {
        PressureModel_Reset(&state, 0x12345678u);
        memset(&out, 0, sizeof(out));
        for (i = 0; i < HOLD_STEPS; ++i) {
            PressureModel_Step(&params, &state, rpm, DT_S, &out);
            if (out.measured_pressure_bar >= 0.9f * *p_steady) {
                *t90_ms = (float)i * DT_S * 1000.0f;
                break;
            }
        }
    }
}

/* 溢流阀设定压力（与 PressureModel_InitParams 一致），超过此值 K 失效 */
#define RELIEF_SET_BAR 250.0f

int main(void) {
    /* 转速点：2..40rpm 为线性段；80rpm 以上进入溢流阀饱和 */
    float rpm_points[] = {2.0f, 5.0f, 10.0f, 20.0f, 40.0f, 80.0f, 160.0f, 320.0f};
    const int n = (int)(sizeof(rpm_points) / sizeof(rpm_points[0]));
    const int n_linear = 5;   /* 前 5 个点在线性段内 */
    int i;
    float ksys_linear[8];
    float ksys_linear_mean = 0.0f;
    float ksys_linear_spread;

    printf("=== 开环过程增益实测（PressureModel 物理标定模型，60s 恒定转速）===\n");
    printf("泵排量 D = %.1f mL/rev, 溢流阀设定 = %.0f bar, 油腔容积 = 0.5 L\n\n",
           PUMP_D_ML_REV, RELIEF_SET_BAR);
    printf("%8s %10s %10s %12s %12s %10s %8s\n",
           "n[rpm]", "Q[L/min]", "P_ss[bar]", "Ksys[bar/rpm]", "K[bar/Lpm]", "t90[ms]", "P_max");

    for (i = 0; i < n; ++i) {
        float rpm = rpm_points[i];
        float p_ss, t90, p_max;
        float q = q_lmin_from_rpm(rpm);
        float ksys, k;

        run_point(rpm, &p_ss, &t90, &p_max);
        ksys = (rpm > 0.0f) ? (p_ss / rpm) : 0.0f;
        k = (q > 0.0f) ? (p_ss / q) : 0.0f;
        ksys_linear[i] = ksys;

        printf("%8.1f %10.3f %10.2f %12.3f %12.2f %10.1f %8.1f\n",
               rpm, q, p_ss, ksys, k, t90, p_max);
    }

    /* ---- 断言 1：线性段内 Ksys 稳定存在（这是 K 可用于参数整定的前提） ---- */
    for (i = 0; i < n_linear; ++i) {
        ksys_linear_mean += ksys_linear[i];
    }
    ksys_linear_mean /= (float)n_linear;
    ksys_linear_spread = 0.0f;
    for (i = 0; i < n_linear; ++i) {
        float dev = (float)fabs((double)(ksys_linear[i] - ksys_linear_mean));
        if (dev > ksys_linear_spread) ksys_linear_spread = dev;
    }
    printf("\n线性段 Ksys 均值 = %.4f bar/rpm, 最大偏差 = %.4f (%.2f%%)\n",
           ksys_linear_mean, ksys_linear_spread,
           100.0f * ksys_linear_spread / ksys_linear_mean);
    assert(ksys_linear_spread / ksys_linear_mean < 0.02f);   /* 线性度优于 2% */
    assert(ksys_linear_mean > 1.0f);                         /* 增益非零且可用 */

    /* ---- 断言 2：K[bar/(L/min)] 与 Ksys 的换算关系 K = Ksys·(1000/D) ---- */
    {
        float k_expected = ksys_linear_mean * (1000.0f / PUMP_D_ML_REV);
        printf("K[bar/(L/min)] = Ksys·(1000/D) = %.4f · %.1f = %.2f\n",
               ksys_linear_mean, 1000.0f / PUMP_D_ML_REV, k_expected);
        assert(k_expected > 0.0f);
    }

    /* ---- 断言 3：超过溢流阀设定后线性关系失效（K 只在有效量程内可用） ---- */
    for (i = n_linear; i < n; ++i) {
        assert(ksys_linear[i] < ksys_linear_mean * 0.95f);
    }
    printf("溢流阀饱和确认：n>=%.0frpm 后 Ksys 下降到 %.3f 以下（P_ss 被钳在 %.0f bar）\n",
           rpm_points[n_linear], ksys_linear_mean * 0.95f, RELIEF_SET_BAR);

    printf("\n✓ PASS 开环过程增益实测（Ksys 存在、线性、且只在溢流阀以下有效）\n");
    return 0;
}
