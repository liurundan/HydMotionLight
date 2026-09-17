/* out/tmp/scan_omega.c — 在真实 HYD_PressureController_Execute 路径上扫描 FF_PI 的 ωn
 * 目的：为 HYD_DEFAULT_LOOP_OMEGA 选值。必须满足全部四项指标且留裕度。
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "pressure_controller.h"
#include "pressure_model.h"

#define SIM_DT   0.001f
#define GAIN     (1000.0f / (25.0f * 0.95f))
#define K_NOM    200.0f
#define OMAX     20.0f
#define OMIN     (-2.375f)
#define BOOST    12.11f
#define SP1      150.0f
#define SP2      100.0f
#define ALPHA    0.1f

static void plant_init(PressureModelParams *p, PressureModelState *s,
                       int noise, unsigned seed, float leak) {
    memset(p, 0, sizeof(*p));
    PressureModel_InitParams(p);
    p->enable_sensor_noise = noise ? 1u : 0u;
    p->enable_motor_noise  = noise ? 1u : 0u;
    if (leak != 1.0f) {
        p->physical.pump_leak_c0_m3_pa_s           *= leak;
        p->physical.pump_leak_speed_m3_pa_s_per_rpm*= leak;
        p->physical.outlet_leak_m3_pa_s            *= leak;
        p->physical.cylinder_leak_m3_pa_s          *= leak;
    }
    PressureModel_Reset(s, seed);
}

typedef struct { float Mp, tr, ts, ess, sig, ddev, drec; } M;

static void run(float tau, float wn, int noise, float leak, M *m) {
    PressureModelParams pp; PressureModelState ps; PressureModelOutput po;
    HYD_MotionSegment seg; HYD_PressureControllerState st;
    int i, n, settled = -1;
    float pmax = 0, sum = 0, sum2 = 0; int nsum = 0;

    memset(m, 0, sizeof(*m));
    m->tr = -1; m->ts = -1; m->drec = -1;

    memset(&seg, 0, sizeof(seg));
    seg.mode = HYD_MODE_PRESSURE_CLOSED_LOOP;
    seg.endCondition = HYD_END_MANUAL;
    seg.direction = HYD_DIRECTION_HOLD;
    seg.targetPressure = SP1;
    seg.maxFlow = 40.375f;
    seg.pressureController = HYD_PRESSURE_CONTROLLER_FF_PI;
    seg.pressureCeiling = 250.0f;
    seg.pressureFilterAlpha = ALPHA;
    seg.systemGain = K_NOM;

    /* S1 升压 0→150 */
    plant_init(&pp, &ps, 0, 0x12345678u, leak);
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
        in.boostFlowLimitLmin = BOOST;
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

    /* S2 保压 100（噪声+纹波） */
    seg.targetPressure = SP2;
    plant_init(&pp, &ps, 1, 0xABCDEF01u, leak);
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
        in.boostFlowLimitLmin = BOOST;
        in.plantTauS = tau; in.loopOmega = wn;
        in.timestamp = (HYD_TIME)i * SIM_DT;
        HYD_PressureController_Execute(&seg, &st, &in, &co);
        PressureModel_Step(&pp, &ps, co.outputFlow * GAIN, SIM_DT, &po);
        if (i >= n - 2000) { float d = po.measured_pressure_bar - SP2; sum += d; sum2 += d * d; nsum++; }
    }
    m->ess = sum / nsum;
    m->sig = sqrtf(sum2 / nsum - m->ess * m->ess); if (m->sig < 0) m->sig = 0;

    /* S3 负载扰动 0.15 L/min (t=4~6s) */
    plant_init(&pp, &ps, 0, 0x13572468u, leak);
    memset(&po, 0, sizeof(po));
    HYD_PressureController_InitState(&st, 0.0f, 0.0f, 0.0);
    n = 9000;
    {
        float dev = 0, rec = -1;
        for (i = 0; i < n; ++i) {
            HYD_PressureControllerInput in; HYD_PressureControllerOutput co;
            PressureModelInput pin;
            memset(&in, 0, sizeof(in));
            in.targetPressure = SP2;
            in.measuredPressure = po.measured_pressure_bar;
            in.outputMin = OMIN; in.outputMax = OMAX;
            in.systemGain = K_NOM;
            in.boostFlowLimitLmin = BOOST;
            in.plantTauS = tau; in.loopOmega = wn;
            in.timestamp = (HYD_TIME)i * SIM_DT;
            HYD_PressureController_Execute(&seg, &st, &in, &co);
            pin.target_rpm = co.outputFlow * GAIN;
            pin.dt_s = SIM_DT;
            pin.load_flow_m3_s = (i >= 4000 && i < 6000) ? (0.15f / 60000.0f) : 0.0f;
            PressureModel_StepInput(&pp, &ps, &pin, &po);
            if (i >= 4000) {
                float d = fabsf(po.measured_pressure_bar - SP2);
                if (d > dev) dev = d;
                if (i >= 6000 && d < 1.0f && rec < 0) rec = (float)(i - 6000) * SIM_DT * 1000.0f;
            }
        }
        m->ddev = dev; m->drec = rec;
    }
}

int main(void) {
    float wn; M m;
    printf("tau=1.0 (库默认), K=200, boost=12.11, OMAX=20, alpha=0.1\n\n");
    printf("%6s %7s %7s %7s %9s %8s | %8s %8s | %s\n",
           "wn", "Mp%", "tr/ms", "ts/ms", "ess", "sigma", "扰动Δ", "恢复/ms", "判定");
    printf("----------------------------------------------------------------------------------\n");
    for (wn = 4.0f; wn <= 30.0f; wn += 2.0f) {
        int ok;
        run(1.0f, wn, 1, 1.0f, &m);
        ok = (m.Mp <= 5.0f) && (fabsf(m.ess) <= 1.0f) && (m.sig <= 1.0f);
        printf("%6.1f %7.2f %7.0f %7.0f %9.3f %8.3f | %8.2f %8.0f | %s\n",
               wn, m.Mp, m.tr, m.ts, m.ess, m.sig, m.ddev, m.drec,
               ok ? "全部达标" : (m.sig > 1.0f ? "★σ超标" : "★不达标"));
    }
    printf("----------------------------------------------------------------------------------\n");
    printf("合格线: Mp<=5%%  tr<=400ms  ts<=500ms  |ess|<=1bar  sigma<=1bar\n");
    return 0;
}
