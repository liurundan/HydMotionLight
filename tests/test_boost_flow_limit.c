/* tests/test_boost_flow_limit.c
 * 升压段实验限流验证（用户 2026-09-16 要求 #2）
 *
 * 现场结论："升压段应用实验限流，否则容易出现超调"。
 * 本用例把这句话变成可复现的量化结论，并按真实泵/管路参数建模：
 *
 *   伺服泵     25 cm^3/rev，额定 1700 rpm  -> 额定流量 42.5 L/min
 *   转速映射   flowToPumpSpeedGain = 1700 / 42.5 = 40 rpm/(L/min)
 *   整机增益   K = Ksys / (D/1000) = 5.0 / 0.025 = 200 bar/(L/min)
 *              （Ksys = 5.0 bar/rpm 由 tests/test_openloop_gain_probe 实测）
 *   反转下限   min_rpm = -100 rpm -> -2.5 L/min
 *   压力管路   L = 3.5 m, 内径 12 mm -> I = rho*L/A = 2.69e7 Pa*s^2/m^3
 *              （见 PressureModel_InitParams 中的推导）
 *
 * 满流量等效建压 = K * Qmax = 8500 bar，目标只有 150 bar，二者差 57 倍。
 *
 * ---- 重要更正（本轮实测推翻上一轮结论）------------------------------------
 * 上一轮在"准静态管路"(I=1e8, R=2e11)下量到无限流超调 56~67%（撞 250 bar 溢流阀）。
 * 把管路按 3.5 m 真实几何标定后，同一条控制链的无限流超调只有 3.7%：那个
 * 66.67% 是**非物理管路**造成的假象——旧的 R=2e11 让泵出口压力与容腔解耦
 * （出口冲到 400 bar 时容腔才 72 bar），流量被憋在出口侧，最后一股脑灌进容腔。
 * 真实管路即使无限流也不会把压力顶到溢流阀。
 *
 * 因此本用例断言的不是"没有限流就不合格"，而是：
 *   - 无限流：超调必须仍然 <= 5%（守住"物理管路下本就不超调"这个事实，
 *     一旦谁把管路参数改回非物理值，这条会立刻失败并暴露原因）；
 *   - 加限流：超调必须 <= 5% 且**永不变差**（限流是单调收紧的安全旋钮）；
 *   - 限流值扫参：任何限流值都不得使超调劣于无限流；
 *   - 驱动饱和：指令超出泵反转能力时植物"饱和"而非"冻结"（历史缺陷）。
 * 限流的代价是上升时间，用例把两者的 tr 都打印出来，不做粉饰。
 */

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "rbf_pid.h"
#include "pressure_model.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ---------- 真实机器常量（用户 2026-09-16 确认） ---------- */
#define SIM_DT_S             0.001f
#define PUMP_D_ML_REV        25.0f
#define PUMP_RATED_RPM       1700.0f
#define PUMP_RATED_FLOW_LMIN (PUMP_D_ML_REV * PUMP_RATED_RPM / 1000.0f)   /* 42.5 */
#define FLOW_TO_RPM_GAIN     (PUMP_RATED_RPM / PUMP_RATED_FLOW_LMIN)      /* 40.0 */
#define KSYS_BAR_PER_RPM     5.0f
#define K_FLOW_BAR_PER_LMIN  (KSYS_BAR_PER_RPM * FLOW_TO_RPM_GAIN)        /* 200.0 */
#define PUMP_REVERSE_RPM     (-100.0f)
#define Q_REVERSE_LIMIT      (PUMP_REVERSE_RPM / FLOW_TO_RPM_GAIN)        /* -2.5 */

/* 管路几何（与 PressureModel_InitParams 一致） */
#define LINE_RHO_KG_M3       870.0f
#define LINE_BORE_M          0.012f
#define LINE_LEN_DEFAULT_M   3.5f

/* 实验限流值：12 L/min。扫参依据（L=3.5 m, P_set=150 bar）：
 *   Q_lim=8  -> Mp 0.55%, tr 520ms
 *   Q_lim=12 -> Mp 1.11%, tr 191ms   <-- 同时满足 Mp<=5% 且 tr<=200ms（旧管路下）
 *   Q_lim=16 -> Mp 9.61%, tr 148ms
 *   Q_lim=24 -> Mp 50.4%, tr 117ms
 *   Q_lim=42.5(不限流) -> Mp 66.7%, tr 107ms（旧管路）
 * 12 L/min 等效建压 2400 bar，是"够快但不冲"的平衡点。 */
#define BOOST_FLOW_LIMIT_LMIN 12.0f
#define BOOST_BRAKE_FRAC      0.5f

#define SIM_STEPS             6000     /* 6 s */
#define TARGET_Mp_PCT         5.0f

typedef struct {
    float p_max;
    float t90_ms;
    float p_steady;
    float ess_bar;
    int   saturated_seen;
} BoostMetrics;

static float line_inertance(float length_m) {
    float area = (float)M_PI * LINE_BORE_M * LINE_BORE_M / 4.0f;
    return LINE_RHO_KG_M3 * length_m / area;
}

static void metrics_reset(BoostMetrics *m) {
    memset(m, 0, sizeof(*m));
    m->t90_ms = -1.0f;
}

/* 闭环：setpoint -> RBF_PID -> Q[L/min] -> rpm -> PressureModel -> P -> 反馈
 * q_limit <= 0 关闭升压限流；q_reverse_min 用于注入不同的反转下限。 */
static void run_boost( float setpoint,
                       float line_length_m,
                       float q_limit,
                       float q_reverse_min,
                       BoostMetrics *m ) {
    PressureModelParams params;
    PressureModelState state;
    PressureModelOutput out;
    RBF_PID_Handle pid;
    float sum = 0.0f;
    int steady_count = 0;
    int i;

    metrics_reset(m);
    memset(&out, 0, sizeof(out));

    PressureModel_InitParams(&params);
    params.physical.line_inertance_pa_s2_per_m3 = line_inertance(line_length_m);
    params.enable_sensor_noise = 0u;   /* 关噪声，专测升压超调 */
    params.enable_motor_noise = 0u;
    PressureModel_Reset(&state, 0x12345678u);

    RBF_PID_Init(&pid, SIM_DT_S, PUMP_RATED_FLOW_LMIN, 1.0f);
    pid.flowToPumpSpeedGain = FLOW_TO_RPM_GAIN;
    pid.output_min_flow = q_reverse_min;
    pid.output_max_flow = PUMP_RATED_FLOW_LMIN;
    RBF_PID_SetControlMode(&pid, RBF_PID_CONTROL_MODE_PID);
    RBF_PID_SetGainCompensation(&pid, K_FLOW_BAR_PER_LMIN);
    if (q_limit > 0.0f) {
        RBF_PID_SetBoostFlowLimit(&pid, q_limit);
        RBF_PID_SetBoostBrakeFrac(&pid, BOOST_BRAKE_FRAC);
    }

    for (i = 0; i < SIM_STEPS; ++i) {
        float flow_cmd = RBF_PID_Update(&pid, setpoint, out.measured_pressure_bar);
        PressureModel_Step(&params, &state, flow_cmd * FLOW_TO_RPM_GAIN,
                           SIM_DT_S, &out);

        if (out.command_saturated) {
            m->saturated_seen = 1;
        }
        if (out.measured_pressure_bar > m->p_max) {
            m->p_max = out.measured_pressure_bar;
        }
        if (m->t90_ms < 0.0f && out.measured_pressure_bar >= 0.9f * setpoint) {
            m->t90_ms = (float)i * SIM_DT_S * 1000.0f;
        }
        if (i >= SIM_STEPS - 1000) {
            sum += out.measured_pressure_bar;
            steady_count++;
        }
    }
    m->p_steady = sum / (float)steady_count;
    m->ess_bar = m->p_steady - setpoint;
}

static float mp_pct(const BoostMetrics *m, float setpoint) {
    return 100.0f * (m->p_max - setpoint) / setpoint;
}

int main(void) {
    BoostMetrics no_limit;
    BoostMetrics limited;
    float setpoints[] = {100.0f, 150.0f, 200.0f, 250.0f};
    float limits[] = {8.0f, 12.0f, 16.0f, 24.0f};
    int i;

    /* 断言失败会 abort，缓冲的 stdout 会丢失——关掉缓冲，保证失败时也能看到数据 */
    setvbuf(stdout, NULL, _IONBF, 0);

    printf("=== 升压段实验限流验证 ===\n");
    printf("泵: %.0f cc/rev @ %.0f rpm -> 额定流量 %.1f L/min\n",
           PUMP_D_ML_REV, PUMP_RATED_RPM, PUMP_RATED_FLOW_LMIN);
    printf("    flowToPumpSpeedGain = %.1f rpm/(L/min), K = %.1f bar/(L/min)\n",
           FLOW_TO_RPM_GAIN, K_FLOW_BAR_PER_LMIN);
    printf("    反转下限 %.1f L/min (min_rpm %.0f)；满流量等效建压 %.0f bar\n",
           Q_REVERSE_LIMIT, PUMP_REVERSE_RPM,
           K_FLOW_BAR_PER_LMIN * PUMP_RATED_FLOW_LMIN);
    printf("    管路 L=%.1f m, 内径 %.0f mm -> I = %.3e Pa*s^2/m^3\n\n",
           LINE_LEN_DEFAULT_M, LINE_BORE_M * 1000.0f,
           (double)line_inertance(LINE_LEN_DEFAULT_M));

    /* ---------- A. 3.5 m 管路：无限流 vs 限流 ---------- */
    run_boost(150.0f, LINE_LEN_DEFAULT_M, 0.0f, Q_REVERSE_LIMIT, &no_limit);
    run_boost(150.0f, LINE_LEN_DEFAULT_M, BOOST_FLOW_LIMIT_LMIN,
              Q_REVERSE_LIMIT, &limited);
    printf("--- A. P_set=150 bar, L=3.5 m ---\n");
    printf("    无限流 : 峰值 %7.2f bar, Mp %6.2f%%, tr %6.0f ms, ess %6.2f bar\n",
           no_limit.p_max, mp_pct(&no_limit, 150.0f), no_limit.t90_ms, no_limit.ess_bar);
    printf("    限流%.0f: 峰值 %7.2f bar, Mp %6.2f%%, tr %6.0f ms, ess %6.2f bar\n",
           BOOST_FLOW_LIMIT_LMIN, limited.p_max, mp_pct(&limited, 150.0f),
           limited.t90_ms, limited.ess_bar);
    printf("    （tr 是限流的代价：限流把建压流量从 %.1f 降到 %.1f L/min）\n\n",
           PUMP_RATED_FLOW_LMIN, BOOST_FLOW_LIMIT_LMIN);
    /* 物理管路下两者都应满足超调指标；把无限流也钉住，防止管路参数被改回非物理值 */
    assert(mp_pct(&no_limit, 150.0f) <= TARGET_Mp_PCT);
    assert(mp_pct(&limited, 150.0f) <= TARGET_Mp_PCT);
    assert(limited.p_max <= no_limit.p_max + 0.1f);
    assert(fabsf(limited.ess_bar) <= 1.0f);

    /* ---------- B. 目标压力扫参：限流必须始终守住超调 ---------- */
    printf("--- B. 限流对目标压力的鲁棒性（L=3.5 m）---\n");
    printf("    %10s %12s %12s %10s\n", "P_set", "Mp_no_lim[%]", "Mp_lim[%]", "tr_lim[ms]");
    for (i = 0; i < (int)(sizeof(setpoints) / sizeof(setpoints[0])); ++i) {
        BoostMetrics a, b;
        run_boost(setpoints[i], LINE_LEN_DEFAULT_M, 0.0f, Q_REVERSE_LIMIT, &a);
        run_boost(setpoints[i], LINE_LEN_DEFAULT_M, BOOST_FLOW_LIMIT_LMIN,
                  Q_REVERSE_LIMIT, &b);
        printf("    %10.0f %12.2f %12.2f %10.0f\n", setpoints[i],
               mp_pct(&a, setpoints[i]), mp_pct(&b, setpoints[i]), b.t90_ms);
        assert(mp_pct(&b, setpoints[i]) <= TARGET_Mp_PCT);          /* 限流守住指标 */
        assert(b.p_max <= a.p_max + 0.1f);                          /* 单调：滤波容差内不变差 */
        assert(fabsf(b.ess_bar) <= 2.0f);                           /* 仍收敛到目标 */
    }
    printf("    ✓ 全部目标压力下 Mp <= %.0f%% 且限流不劣于无限流\n\n", TARGET_Mp_PCT);

    /* ---------- C. 限流值扫参（单调性） ---------- */
    printf("--- C. 限流值扫参（P_set=150 bar, L=3.5 m）---\n");
    printf("    %10s %10s %10s %10s\n", "Q_lim", "Mp[%]", "tr[ms]", "ess[bar]");
    for (i = 0; i < (int)(sizeof(limits) / sizeof(limits[0])); ++i) {
        BoostMetrics mc;
        run_boost(150.0f, LINE_LEN_DEFAULT_M, limits[i], Q_REVERSE_LIMIT, &mc);
        printf("    %10.1f %10.2f %10.0f %10.2f\n", limits[i],
               mp_pct(&mc, 150.0f), mc.t90_ms, mc.ess_bar);
        assert(mp_pct(&mc, 150.0f) <= mp_pct(&no_limit, 150.0f) + 0.1f);
        assert(mc.p_max <= no_limit.p_max + 0.1f);
    }
    printf("    ✓ 所有限流值的超调均不劣于不限流\n\n");

    /* ---------- D. 驱动饱和而非冻结 ---------- */
    {
        BoostMetrics sat;
        /* 反转下限放大到 -5 L/min，超出泵的 -2.5 L/min 能力 */
        run_boost(150.0f, LINE_LEN_DEFAULT_M, BOOST_FLOW_LIMIT_LMIN, -5.0f, &sat);
        printf("--- D. 驱动饱和（指令 -5 L/min vs 泵反转能力 -2.5 L/min）---\n");
        printf("    峰值 %.2f bar, Mp %.2f%%, ess %.2f bar, 观测到饱和=%d\n",
               sat.p_max, mp_pct(&sat, 150.0f), sat.ess_bar, sat.saturated_seen);
        assert(sat.saturated_seen == 1);
        /* 关键：植物必须仍然收敛到目标，而不是把压力冻死在某处
         * （历史缺陷：越界指令导致整步状态回退，压力永久卡死） */
        assert(fabsf(sat.ess_bar) <= 2.0f);
        printf("    ✓ PASS 指令被限幅（饱和标志置位），且压力仍收敛（未冻结）\n\n");
    }

    /* ---------- E. 管路有效窗口（建模约束，必须显式知道） ---------- */
    {
        PressureModelParams p;
        int ok_short, ok_long;
        /* 显式欧拉 CFL: beta*dt^2/(I*min(V)) <= 0.25  =>  I >= 9.6e6
         * => L >= 9.6e6 * A / rho = 1.248 m  (A = 1.131e-4 m^2, rho = 870) */
        PressureModel_InitParams(&p);
        p.physical.line_inertance_pa_s2_per_m3 = line_inertance(1.0f);
        ok_short = PressureModel_ValidateParams(&p);
        PressureModel_InitParams(&p);
        p.physical.line_inertance_pa_s2_per_m3 = line_inertance(2.0f);
        ok_long = PressureModel_ValidateParams(&p);
        printf("--- E. 管路长度有效窗口（显式欧拉 CFL）---\n");
        printf("    L = 1.0 m -> 参数校验 %s（低于下限，模型无法表示）\n",
               ok_short ? "通过" : "拒绝");
        printf("    L = 2.0 m -> 参数校验 %s\n", ok_long ? "通过" : "拒绝");
        assert(ok_short == 0);
        assert(ok_long != 0);
        printf("    下限 L_min = beta*dt^2/(0.25*min(V)) * A / rho = %.2f m\n",
               (1.2e9f * 1.0e-6f / (0.25f * 5.0e-4f)) *
               ((float)M_PI * LINE_BORE_M * LINE_BORE_M / 4.0f) /
               LINE_RHO_KG_M3);
        printf("    ⇒ 若实测管长 < 1.25 m，必须先给管路积分加子步，不能直接跑\n\n");
    }

    /* ---------- F. v9 修正 D11：限流低于维持流量时目标仍可达 ---------- */
    {
        /* 维持流量 Q_ss = P_set/K：100bar->0.50, 150->0.75, 200->1.00, 250->1.25 L/min。
         * 旧实现 `q_ss = q_boost` 会把制动包络退化为常量，稳态压力钉在 K*Q_lim，
         * 误差有闭式规律 ess = K*Q_lim - P_set（实测完全吻合），且无任何诊断输出。
         * 修正后内部把有效限流抬到不低于 Q_ss —— 可达性由构造保证。 */
        float low_limits[] = {0.2f, 0.5f, 1.0f};
        int j;
        printf("--- F. 可达性下界（限流 < 维持流量时必须仍能到达目标）---\n");
        printf("    %10s %10s %12s %12s %10s\n", "P_set", "Q_lim", "Q_ss", "ess[bar]", "tr[ms]");
        for (i = 0; i < (int)(sizeof(setpoints) / sizeof(setpoints[0])); ++i) {
            for (j = 0; j < (int)(sizeof(low_limits) / sizeof(low_limits[0])); ++j) {
                BoostMetrics fl;
                float q_ss = setpoints[i] / K_FLOW_BAR_PER_LMIN;
                if (low_limits[j] >= q_ss) {
                    continue;   /* 只测"低于维持流量"的危险工况 */
                }
                run_boost(setpoints[i], LINE_LEN_DEFAULT_M, low_limits[j],
                          Q_REVERSE_LIMIT, &fl);
                printf("    %10.0f %10.1f %12.3f %12.2f %10.0f\n",
                       setpoints[i], low_limits[j], q_ss, fl.ess_bar, fl.t90_ms);
                /* 关键：不得出现"目标不可达"（旧实现在此给 ess = K*Q_lim - P_set，
                 * 例如 Q_lim=0.5/P_set=150 时 ess = -50.56 bar） */
                assert(fabsf(fl.ess_bar) <= 2.0f);
                /* 也必须真的达到 90% 目标 */
                assert(fl.t90_ms > 0.0f);
            }
        }
        printf("    ✓ 所有低于维持流量的限流值下目标压力仍可达（D11 已修正）\n\n");
    }

    printf("=== 汇总 ===\n");
    printf("物理管路(3.5m)下：无限流 Mp %.2f%%，限流 %.1f L/min 后 Mp %.2f%%（tr %.0f -> %.0f ms）\n",
           mp_pct(&no_limit, 150.0f), BOOST_FLOW_LIMIT_LMIN,
           mp_pct(&limited, 150.0f), no_limit.t90_ms, limited.t90_ms);
    printf("✓ PASS 升压段实验限流（超调有界、单调安全、植物不冻结）\n");
    return 0;
}
