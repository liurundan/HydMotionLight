/* tests/test_pressure_debug_trace.c
 * 调试用：追踪 S1 升压 0→150bar 阶跃响应的关键变量
 * 输出 CSV 格式到 stdout，便于分析超调根因
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "rbf_pid.h"
#include "pressure_model.h"

#define SIM_DT_S         0.001f
#define FLOW_TO_RPM_GAIN 1.0f
#define RBF_FMAX_FLOW    90.0f
#define RBF_OUTPUT_MIN   -5.0f
#define RBF_OUTPUT_MAX   90.0f

int main(void) {
    PressureModelParams params;
    PressureModelState state;
    PressureModelOutput out; memset(&out, 0, sizeof(out));
    RBF_PID_Handle pid;
    float setpoint = 150.0f;
    float feedback, flow_cmd, rpm_cmd;
    int steps = 2000;  /* 2s @ 1ms, 足够看到超调 */
    int i;

    PressureModel_InitParams(&params);
    params.enable_sensor_noise = 0u;
    params.enable_motor_noise = 0u;
    PressureModel_Reset(&state, 0x12345678u);

    RBF_PID_Init(&pid, 0.001f, RBF_FMAX_FLOW, 1.0f);
    pid.flowToPumpSpeedGain = FLOW_TO_RPM_GAIN;
    pid.output_min_flow = RBF_OUTPUT_MIN;
    pid.output_max_flow = RBF_OUTPUT_MAX;
    RBF_PID_SetControlMode(&pid, RBF_PID_CONTROL_MODE_PID);
    RBF_PID_SetGainCompensation(&pid, 5.4f);  /* v6: 与 S1 仿真一致（开环实测 Ksys） */

    /* CSV header */
    printf("step,time_ms,P_actual,error,Output,u_prev,du,KP,KI,KD,output_saturated,steady_state,control_state\n");

    for (i = 0; i < steps; ++i) {
        feedback = out.measured_pressure_bar;
        flow_cmd = RBF_PID_Update(&pid, setpoint, feedback);
        rpm_cmd = flow_cmd * FLOW_TO_RPM_GAIN;
        PressureModel_Step(&params, &state, rpm_cmd, SIM_DT_S, &out);

        float time_ms = (float)i * SIM_DT_S * 1000.0f;
        float err = setpoint - out.measured_pressure_bar;

        /* Print every step for first 50ms, then every 10ms, then every 50ms */
        int print_it = 0;
        if (i < 50) print_it = 1;                           /* 0-50ms: every step */
        else if (i < 200 && i % 10 == 0) print_it = 1;      /* 50-200ms: every 10ms */
        else if (i < 500 && i % 20 == 0) print_it = 1;      /* 200-500ms: every 20ms */
        else if (i < 2000 && i % 50 == 0) print_it = 1;      /* 500-2000ms: every 50ms */

        /* Always print near the crossing point (135-175bar) */
        if (out.measured_pressure_bar > 130.0f && out.measured_pressure_bar < 180.0f) {
            if (i < 500 || i % 10 == 0) print_it = 1;
        }

        if (print_it) {
            printf("%d,%.1f,%.3f,%.3f,%.3f,%.3f,%.3f,%.4f,%.5f,%.4f,%d,%d,%d\n",
                   i, time_ms,
                   out.measured_pressure_bar,
                   err,
                   pid.Output,
                   pid.u_prev,
                   pid.du,
                   pid.KP, pid.KI, pid.KD,
                   pid.output_saturated ? 1 : 0,
                   pid.steady_state ? 1 : 0,
                   (int)pid.control_state);
        }
    }

    fprintf(stderr, "Peak pressure: %.2f bar\n", out.measured_pressure_bar);
    return 0;
}
