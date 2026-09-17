/* out/tmp/scan_env.c — 扫描 FF_PI 的升压包络参数（boost 限流 × 制动窗口）
 * 背景：生产链路前置滤波 α=0.1（时间常数 dt/α = 10ms）。升压若太快（tr≈111ms，
 * dP/dt 达 1300~2500 bar/s），滤波滞后会让控制器"看不见"已上升的压力 → 超调。
 * 因此必须靠包络把上升速度压下来。
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "pressure_controller.h"
#include "pressure_model.h"

#define SIM_DT 0.001f
#define GAIN   (1000.0f / (25.0f * 0.95f))
#define K_NOM  200.0f
#define OMAX   20.0f
#define OMIN   (-2.375f)
#define SP1    150.0f
#define SP2    100.0f

static void plant_init(PressureModelParams *p, PressureModelState *s,
                       int noise, unsigned seed) {
    memset(p, 0, sizeof(*p));
    PressureModel_InitParams(p);
    p->enable_sensor_noise = noise ? 1u : 0u;
    p->enable_motor_noise  = noise ? 1u : 0u;
    PressureModel_Reset(s, seed);
}

typedef struct { float Mp, tr, ts, ess, sig, ddev; } M;

static void run(float wn, float tau, float boost, float brake, M *m) {
    PressureModelParams pp; PressureModelState ps; PressureModelOutput po;
    HYD_MotionSegment seg; HYD_PressureControllerState st;
    int i, n, settled = -1;
    float pmax = 0, sum = 0, sum2 = 0; int nsum = 0;

    memset(m, 0, sizeof(*m)); m->tr = -1; m->ts = -1;

    memset(&seg, 0, sizeof(seg));
    seg.mode = HYD_MODE_PRESSURE_CLOSED_LOOP;
    seg.endCondition = HYD_END_MANUAL;
    seg.direction = HYD_DIRECTION_HOLD;
    seg.targetPressure = SP1;
    seg.maxFlow = 40.375f;
    seg.pressureController = HYD_PRESSURE_CONTROLLER_FF_PI;
    seg.pressureCeiling = 250.0f;
    seg.pressureFilterAlpha = 0.1f;
    seg.systemGain = K_NOM;

    plant_init(&pp, &ps, 0, 0x12345678u);
    memset(&po, 0, sizeof(po));
    HYD_PressureController_InitState(&st, 0.0f, 0.0f, 0.0);
    n = 6000;
    for (i = 0; i < n; ++i) {
        HYD_PressureControllerInput in; HYD_PressureControllerOutput co;
        memset(&in, 0, sizeof(in));
        in.targetPressure = SP1;
        in.measuredPressure = po.measured_pressure_bar;
        in.outputMin = OMIN; in.outputMax = OMAX;
        in.systemGain = K_NOM;
        in.boostFlowLimitLmin = boost;
        in.boostBrakeFrac = brake;
        in.plantTauS = tau; in.loopOmega = wn;
        in.timestamp = (HYD_TIME)i * SIM_DT;
        HYD_PressureController_Execute(&seg, &st, &in, &co);
        PressureModel_Step(&pp, &ps, co.outputFlow * GAIN, SIM_DT, &po);
        if (po.measured_pressure_bar > pmax) pmax = po.measured_pressure_bar;
        if (m->tr < 0 && po.measured_pressure_bar >= 0.9f * SP1) m->tr = (float)i * SIM_DT * 1000.0f;
        if (fabsf(SP1 - po.measured_pressure_bar) <= 0.02f * SP1) { if (settled < 0) settled = i; }
        else settled = -1;
    }
    m->Mp = (pmax - SP1) / SP1 * 100.0f; if (m->Mp < 0) m->Mp = 0;
    m->ts = (settled >= 0) ? (float)settled * SIM_DT * 1000.0f : -1.0f;

    seg.targetPressure = SP2;
    plant_init(&pp, &ps, 1, 0xABCDEF01u);
    memset(&po, 0, sizeof(po));
    HYD_PressureController_InitState(&st, 0.0f, 0.0f, 0.0);
    n = 10000;
    for (i = 0; i < n; ++i) {
        HYD_PressureControllerInput in; HYD_PressureControllerOutput co;
        memset(&in, 0, sizeof(in));
        in.targetPressure = SP2;
        in.measuredPressure = po.measured_pressure_bar;
        in.outputMin = OMIN; in.outputMax = OMAX;
        in.systemGain = K_NOM;
        in.boostFlowLimitLmin = boost;
        in.boostBrakeFrac = brake;
        in.plantTauS = tau; in.loopOmega = wn;
        in.timestamp = (HYD_TIME)i * SIM_DT;
        HYD_PressureController_Execute(&seg, &st, &in, &co);
        PressureModel_Step(&pp, &ps, co.outputFlow * GAIN, SIM_DT, &po);
        if (i >= n - 2000) { float d = po.measured_pressure_bar - SP2; sum += d; sum2 += d * d; nsum++; }
    }
    m->ess = sum / nsum;
    m->sig = sqrtf(sum2 / nsum - m->ess * m->ess); if (m->sig < 0) m->sig = 0;
}

int main(void) {
    float boosts[6]  = {1.5f, 2.0f, 3.0f, 4.0f, 6.0f, 12.11f};
    float brakes[3]  = {0.5f, 1.0f, 2.0f};
    int i, j; M m;

    printf("wn=12 tau=1.0 K=200  (单元格: Mp%% / tr ms / ts ms)\n\n");
    printf("%8s | %22s | %22s | %22s\n", "boost", "brake=0.5", "brake=1.0", "brake=2.0");
    printf("---------+------------------------+------------------------+------------------------\n");
    for (i = 0; i < 6; ++i) {
        printf("%8.2f |", boosts[i]);
        for (j = 0; j < 3; ++j) {
            run(12.0f, 1.0f, boosts[i], brakes[j], &m);
            printf(" %6.2f %5.0f %5.0f %s |", m.Mp, m.tr, m.ts,
                   (m.Mp <= 5.0f && m.tr <= 400.0f && m.ts >= 0 && m.ts <= 500.0f) ? "✓" : " ");
        }
        printf("\n");
    }
    printf("---------+------------------------+------------------------+------------------------\n");
    printf("合格: Mp<=5%%  tr<=400ms  ts<=500ms\n\n");

    printf("=== 选定组合的保压 ess / sigma（100bar，噪声+纹波）===\n");
    {
        float sel[3][2] = {{2.0f, 1.0f}, {3.0f, 1.0f}, {4.0f, 1.0f}};
        for (i = 0; i < 3; ++i) {
            run(12.0f, 1.0f, sel[i][0], sel[i][1], &m);
            printf("  boost=%.2f brake=%.1f -> Mp %5.2f%%  tr %4.0fms  ts %4.0fms  ess %7.3f  sigma %6.3f  %s\n",
                   sel[i][0], sel[i][1], m.Mp, m.tr, m.ts, m.ess, m.sig,
                   (m.Mp <= 5.0f && fabsf(m.ess) <= 1.0f && m.sig <= 1.0f) ? "达标" : "不达标");
        }
    }
    return 0;
}
