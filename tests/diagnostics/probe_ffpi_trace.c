/* out/tmp/probe_ffpi_trace.c — FF_PI 升压轨迹分解，定位 25% 超调的来源 */
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
#define BOOST  12.11f
#define SP     150.0f

typedef struct { float P_term, I_term, D_term, track, ff, q, P; } Row;

int main(void) {
    PressureModelParams pp; PressureModelState ps; PressureModelOutput po;
    HYD_MotionSegment seg; HYD_PressureControllerState st;
    int i;
    float wn = 12.0f, tau = 1.0f;

    memset(&seg, 0, sizeof(seg));
    seg.mode = HYD_MODE_PRESSURE_CLOSED_LOOP;
    seg.endCondition = HYD_END_MANUAL;
    seg.direction = HYD_DIRECTION_HOLD;
    seg.targetPressure = SP;
    seg.maxFlow = 40.375f;
    seg.pressureController = HYD_PRESSURE_CONTROLLER_FF_PI;
    seg.pressureCeiling = 250.0f;
    seg.pressureFilterAlpha = 0.1f;
    seg.systemGain = K_NOM;

    memset(&pp, 0, sizeof(pp)); PressureModel_InitParams(&pp);
    PressureModel_Reset(&ps, 0x12345678u);
    memset(&po, 0, sizeof(po));
    HYD_PressureController_InitState(&st, 0.0f, 0.0f, 0.0);

    printf("wn=%.1f tau=%.1f  K=%.0f  q_ff=%.4f  boost=%.2f\n",
           wn, tau, K_NOM, SP / K_NOM, BOOST);
    printf("%6s %9s %9s %9s %9s %9s %9s %8s %6s\n",
           "t/ms", "P", "Q", "Pterm", "Iterm", "ssFF", "unsat", "capBnd", "track");
    for (i = 0; i < 900; ++i) {
        HYD_PressureControllerInput in; HYD_PressureControllerOutput co;
        memset(&in, 0, sizeof(in));
        in.targetPressure = SP;
        in.measuredPressure = po.measured_pressure_bar;
        in.outputMin = OMIN; in.outputMax = OMAX;
        in.systemGain = K_NOM;
        in.boostFlowLimitLmin = BOOST;
        in.plantTauS = tau; in.loopOmega = wn;
        in.timestamp = (HYD_TIME)i * SIM_DT;
        HYD_PressureController_Execute(&seg, &st, &in, &co);
        PressureModel_Step(&pp, &ps, co.outputFlow * GAIN, SIM_DT, &po);
        if (i < 12 || i % 25 == 0) {
            printf("%6d %9.3f %9.4f %9.4f %9.4f %9.4f %9.4f %8d %6d\n",
                   i, po.measured_pressure_bar, co.outputFlow,
                   co.proportionalTerm, co.integralTerm, co.steadyStateFF,
                   co.unsaturatedOutputFlow, (int)co.capBoundDemand,
                   (int)co.trackingApplied);
        }
    }
    return 0;
}
