/* tests/test_soft_reset_switch.c
 * 软复位 vs 无复位：150→30bar 工况切换对比验证
 *
 * 用 HYD_PressureController_Execute（真实路径，含软复位逻辑）
 * + PressureModel 物理植物（非线性：βe含气塌缩/泄漏随压力变化）
 *
 * 对比：目标切换后的过渡段冲击、收敛速度、振荡
 */

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "pressure_controller.h"
#include "pressure_model.h"
#include "common_types.h"

#define SIM_DT_S         0.001f
#define HOLD_150_STEPS   3000    /* 3s 稳定在 150bar */
#define SWITCH_STEPS     5000    /* 5s 切换+收敛到 30bar */
#define FLOW_TO_RPM_GAIN 1.0f

/* 构造一个 RBF-PID 压力闭环段 */
static void make_rbf_segment(HYD_MotionSegment *seg) {
    memset(seg, 0, sizeof(*seg));
    seg->mode = HYD_MODE_PRESSURE_CLOSED_LOOP;
    seg->pressureController = HYD_PRESSURE_CONTROLLER_RBF_PID;
    seg->targetPressure = 150.0;
    seg->maxFlow = 90.0;
    seg->targetFlow = 0.0;
    seg->pressureKp = 0.0;   /* 0 → 用库默认 1.5 */
    seg->pressureKi = 0.0;   /* 0 → RBF 内部自适应 */
    seg->pressureKd = 0.0;
    seg->pressureDeadband = 0.0;
    seg->pressureFilterAlpha = 0.0;  /* 0 → 用库默认 0.1 */
    seg->pressureRampRate = 0.0;    /* 0 = 立即跳变（不走 ramp） */
}

/* 跑一次切换场景，采集过渡段指标 */
typedef struct {
    float p_at_switch;        /* 切换瞬间压力 */
    float p_min_after_switch; /* 切换后最低压力（下冲） */
    float p_max_after_switch; /* 切换后最高压力（超调） */
    float settle_time_ms;     /* 进入 30±2bar 后不再退出 */
    float ess_mean;           /* 末 2s 均值误差 */
    float sigma_rms;          /* 末 2s 纹波 */
    int   soft_reset_fired;   /* 软复位是否触发 */
} SwitchMetrics;

static void run_switch_scenario(SwitchMetrics *m) {
    PressureModelParams plant_params;
    PressureModelState plant_state;
    PressureModelOutput plant_out;
    HYD_MotionSegment seg;
    HYD_PressureControllerState ctrl_state;
    HYD_PressureControllerInput ctrl_input = {0};
    HYD_PressureControllerOutput ctrl_output;
    int i;
    float p_sum = 0.0f, p_sq_sum = 0.0f;
    int n_steady = 0;
    int settled_start = -1;
    float band = 2.0f;  /* 30±2 bar */

    memset(m, 0, sizeof(*m));
    m->p_min_after_switch = 1e9f;
    m->p_max_after_switch = -1e9f;
    m->settle_time_ms = -1.0f;

    /* 植物：物理模型，经典默认参数 */
    PressureModel_InitParams(&plant_params);
    plant_params.enable_sensor_noise = 0u;
    plant_params.enable_motor_noise = 0u;
    PressureModel_Reset(&plant_state, 0xAABBCCDDu);
    memset(&plant_out, 0, sizeof(plant_out));

    /* 控制器：RBF-PID */
    make_rbf_segment(&seg);
    HYD_PressureController_ClearState(&ctrl_state);
    memset(&ctrl_input, 0, sizeof(ctrl_input));
    memset(&ctrl_output, 0, sizeof(ctrl_output));

    /* --- 阶段1：150bar 保持 3s，让 RBF 学习 + 稳定 --- */
    ctrl_input.outputMin = -5.0;
    ctrl_input.outputMax = 90.0;
    ctrl_input.feedforwardFlow = 0.0;
    ctrl_input.flowToPumpSpeedGain = FLOW_TO_RPM_GAIN;
    ctrl_input.pumpSpeedLimit = 2000.0;

    for (i = 0; i < HOLD_150_STEPS; ++i) {
        ctrl_input.targetPressure = 150.0;
        ctrl_input.measuredPressure = plant_out.measured_pressure_bar;
        ctrl_input.timestamp = (HYD_TIME)(i * SIM_DT_S);

        HYD_PressureController_Execute(&seg, &ctrl_state, &ctrl_input, &ctrl_output);

        float rpm = ctrl_output.outputFlow * FLOW_TO_RPM_GAIN;
        PressureModel_Step(&plant_params, &plant_state, rpm, SIM_DT_S, &plant_out);
    }

    /* --- 切换瞬间 --- */
    m->p_at_switch = plant_out.measured_pressure_bar;

    /* --- 阶段2：切到 30bar，5s --- */
    for (i = 0; i < SWITCH_STEPS; ++i) {
        ctrl_input.targetPressure = 30.0;
        ctrl_input.measuredPressure = plant_out.measured_pressure_bar;
        ctrl_input.timestamp = (HYD_TIME)((HOLD_150_STEPS + i) * SIM_DT_S);

        HYD_PressureController_Execute(&seg, &ctrl_state, &ctrl_input, &ctrl_output);

        if (ctrl_output.trackingApplied) {
            m->soft_reset_fired = 1;
        }

        float rpm = ctrl_output.outputFlow * FLOW_TO_RPM_GAIN;
        PressureModel_Step(&plant_params, &plant_state, rpm, SIM_DT_S, &plant_out);

        float p = plant_out.measured_pressure_bar;
        if (p < m->p_min_after_switch) m->p_min_after_switch = p;
        if (p > m->p_max_after_switch) m->p_max_after_switch = p;

        /* 建稳判定：进入 30±2bar 后不再退出 */
        if (fabsf(p - 30.0f) <= band) {
            if (settled_start < 0) settled_start = i;
            m->settle_time_ms = (float)settled_start * SIM_DT_S * 1000.0f;
        } else {
            settled_start = -1;
            m->settle_time_ms = -1.0f;
        }

        /* 末 2s 稳态统计 */
        if (i >= SWITCH_STEPS - 2000) {
            p_sum += p;
            p_sq_sum += p * p;
            ++n_steady;
        }
    }

    if (n_steady > 0) {
        float mean = p_sum / n_steady;
        m->ess_mean = 30.0f - mean;
        m->sigma_rms = sqrtf(fabsf(p_sq_sum / n_steady - mean * mean));
    }
}

int main(void) {
    SwitchMetrics m;

    printf("=== 软复位验证：150→30bar 工况切换 ===\n");
    printf("植物: PressureModel 物理模型（经典默认参数，无噪声）\n");
    printf("控制器: RBF-PID via HYD_PressureController_Execute（含软复位逻辑）\n");
    printf("流程: 150bar 保持 3s → 切 30bar 跑 5s\n\n");

    run_switch_scenario(&m);

    printf("--- 软复位路径（HYD_PressureController_Execute 含软复位）---\n");
    printf("  切换时压力     : %.2f bar\n", m.p_at_switch);
    printf("  软复位触发     : %s\n", m.soft_reset_fired ? "是 ✓" : "否 ✗");
    printf("  切换后最低     : %.2f bar\n", m.p_min_after_switch);
    printf("  建稳时间 ts    : %.0f ms\n", m.settle_time_ms);
    printf("  稳态误差 ess   : %.3f bar\n", m.ess_mean);
    printf("  稳态纹波 σ_ss  : %.3f bar RMS\n\n", m.sigma_rms);

    /* 对照：无复位路径（RBF_PID_Update 直接调用，绕过 pressure_controller，无软复位）*/
    {
        SwitchMetrics nb;  /* no-reset baseline */
        PressureModelParams pp;
        PressureModelState ps;
        PressureModelOutput po;
        RBF_PID_Handle pid;
        int i;
        float p_sum=0, p_sq_sum=0;
        int n_steady=0, settled_start=-1;
        float band=2.0f;
        int drop_started=0;

        memset(&nb, 0, sizeof(nb));
        nb.p_min_after_switch = 1e9f;
        nb.p_max_after_switch = -1e9f;

        PressureModel_InitParams(&pp);
        pp.enable_sensor_noise = 0u;
        pp.enable_motor_noise = 0u;
        PressureModel_Reset(&ps, 0xAABBCCDDu);
        memset(&po, 0, sizeof(po));
        RBF_PID_Init(&pid, 0.001f, 90.0f, 1.0f);
        pid.flowToPumpSpeedGain = FLOW_TO_RPM_GAIN;
        pid.output_min_flow = -5.0f;
        pid.output_max_flow = 90.0f;
        RBF_PID_SetControlMode(&pid, RBF_PID_CONTROL_MODE_PID);

        /* 150bar 保持 3s */
        for (i = 0; i < HOLD_150_STEPS; ++i) {
            float q = RBF_PID_Update(&pid, 150.0f, po.measured_pressure_bar);
            PressureModel_Step(&pp, &ps, q * FLOW_TO_RPM_GAIN, SIM_DT_S, &po);
        }
        nb.p_at_switch = po.measured_pressure_bar;

        /* 切 30bar 5s */
        for (i = 0; i < SWITCH_STEPS; ++i) {
            float q = RBF_PID_Update(&pid, 30.0f, po.measured_pressure_bar);
            PressureModel_Step(&pp, &ps, q * FLOW_TO_RPM_GAIN, SIM_DT_S, &po);

            float p = po.measured_pressure_bar;
            /* 只在压力开始下降后才计 p_max（排除切换瞬间残留 150bar）*/
            if (p < nb.p_at_switch * 0.9f) drop_started = 1;
            if (drop_started && p > nb.p_max_after_switch) nb.p_max_after_switch = p;
            if (p < nb.p_min_after_switch) nb.p_min_after_switch = p;

            if (fabsf(p - 30.0f) <= band) {
                if (settled_start < 0) settled_start = i;
                nb.settle_time_ms = (float)settled_start * SIM_DT_S * 1000.0f;
            } else { settled_start = -1; nb.settle_time_ms = -1.0f; }

            if (i >= SWITCH_STEPS - 2000) {
                p_sum += p; p_sq_sum += p*p; ++n_steady;
            }
        }
        if (n_steady > 0) {
            float mean = p_sum / n_steady;
            nb.ess_mean = 30.0f - mean;
            nb.sigma_rms = sqrtf(fabsf(p_sq_sum/n_steady - mean*mean));
        }

        printf("--- 无复位对照（RBF_PID_Update 直接调用，无软复位）---\n");
        printf("  切换时压力     : %.2f bar\n", nb.p_at_switch);
        printf("  切换后最低     : %.2f bar\n", nb.p_min_after_switch);
        printf("  建稳时间 ts    : %.0f ms\n", nb.settle_time_ms);
        printf("  稳态误差 ess   : %.3f bar\n", nb.ess_mean);
        printf("  稳态纹波 σ_ss  : %.3f bar RMS\n\n", nb.sigma_rms);

        printf("=== 软复位 vs 无复位 对比 ===\n");
        printf("  建稳时间  : 软复位 %.0fms vs 无复位 %.0fms  Δ%+.0fms\n",
               m.settle_time_ms, nb.settle_time_ms, m.settle_time_ms - nb.settle_time_ms);
        printf("  最低压力  : 软复位 %.2f vs 无复位 %.2f  Δ%+.2f bar\n",
               m.p_min_after_switch, nb.p_min_after_switch,
               m.p_min_after_switch - nb.p_min_after_switch);
        printf("  稳态误差  : 软复位 %.3f vs 无复位 %.3f\n", m.ess_mean, nb.ess_mean);
        printf("  稳态纹波  : 软复位 %.3f vs 无复位 %.3f\n", m.sigma_rms, nb.sigma_rms);
    }

    return 0;
}
