/* 生产链路带宽/鲁棒性扫描：τ × ωn → (Mp, tr, ess, σ)
 * 目的：给出"出厂默认够不够 / 现场要不要标 τ / ωn 能开多大"的实证表 */
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "motion_interface.h"
#include "motion_control.h"
#include "pressure_controller.h"
#include "pressure_model.h"

extern HYD_MotionControlFB* __MK_GetPublic_MotionControlFB(int index);
#define IEC_VAL(var) ((var).value)

#define GAIN (1000.0f/(25.0f*0.95f))
#define DT   0.001f

static void wr(int axis, int param, float v) {
    HYD_WRITEPARAMETER wp;
    memset(&wp, 0, sizeof(wp));
    IEC_VAL(wp.EN) = true; IEC_VAL(wp.EXECUTE) = true; IEC_VAL(wp.AXISID) = (IEC_SINT)axis;
    IEC_VAL(wp.PARAMETERNUMBER) = (IEC_DINT)param; IEC_VAL(wp.VALUE) = (IEC_LREAL)v;
    __mcl_cmd_WriteParameter(&wp);
}

typedef struct { float mp, tr, ess, sig; int ok; } Res;

static Res run(float tau, float omega) {
    HYD_CREATEMOTION cm; HYD_PRESSUREHANDLE ph;
    HYD_MotionControlFB* fb;
    PressureModelParams pp; PressureModelState ps; PressureModelOutput po;
    Res r; int i; float pmax = 0; float tr = -1; float sum=0, sq=0; int n=0;

    __HydMotion_framework_Init();
    memset(&cm, 0, sizeof(cm));
    IEC_VAL(cm.EN) = true; IEC_VAL(cm.USE_RECIPE) = false;
    IEC_VAL(cm.FLOW_TO_PUMPSPEED) = GAIN; IEC_VAL(cm.PUMPSPEED_LIMIT) = 1700.0f;
    IEC_VAL(cm.USE_SIMULATION) = false;
    __mcl_cmd_CreateMotion(&cm);
    fb = __MK_GetPublic_MotionControlFB(0);
    wr(0, HYD_PARAM_PUMP_DISPLACEMENT, 25.0f);
    wr(0, HYD_PARAM_PUMP_VOLUMETRIC_EFF, 0.95f);
    wr(0, HYD_PARAM_PUMP_MAX_SPEED, 1700.0f);
    if (tau > 0.0f)   wr(0, HYD_PARAM_PRESSURE_SYSTEM_GAIN, tau);
    

    PressureModel_InitParams(&pp);
    pp.enable_sensor_noise = 0; pp.enable_motor_noise = 0;
    PressureModel_Reset(&ps, 0x1234u);
    memset(&po, 0, sizeof(po));
    fb->AXIS_REF.pressure = 0.0f;
    memset(&ph, 0, sizeof(ph));
    IEC_VAL(ph.EN) = true; IEC_VAL(ph.EXECUTE) = true; IEC_VAL(ph.AXISID) = 0;
    IEC_VAL(ph.PRESSURE) = 150.0f; IEC_VAL(ph.DURATION) = 1000.0f;
    IEC_VAL(ph.FLOWLIMITPERCENT) = 100.0f; IEC_VAL(ph.PRESSURERAMPRATE) = 100000.0f;
    IEC_VAL(ph.CONTINUOUSUPDATE) = true;

    for (i = 0; i < 5000; ++i) {
        float rpm;
        __mcl_cmd_PressureHandle(&ph);
        fb->AXIS_REF.pressure = po.measured_pressure_bar;
        __HydMotion_framework_Publish();
        rpm = (float)fb->_lastCommandedFlow * GAIN;
        PressureModel_Step(&pp, &ps, rpm, DT, &po);
        if (po.measured_pressure_bar > pmax) pmax = po.measured_pressure_bar;
        if (tr < 0 && po.measured_pressure_bar >= 135.0f) tr = (float)i * DT * 1000.0f;
    }
    r.mp = tr > 0 ? (pmax - 150.0f) / 150.0f * 100.0f : -1.0f;
    r.tr = tr;

    /* 保压 100 bar 带噪声 —— 复位手法与 test_production_default_acceptance 一致 */
    PressureModel_Reset(&ps, 0x8765u);
    memset(&po, 0, sizeof(po));
    pp.enable_sensor_noise = 1; pp.enable_motor_noise = 1;
    fb->_activeSegmentValid = false;
    fb->AXIS_REF.timestamp = 0.0;
    memset(&ph, 0, sizeof(ph));
    IEC_VAL(ph.EN) = true; IEC_VAL(ph.EXECUTE) = true; IEC_VAL(ph.AXISID) = 0;
    IEC_VAL(ph.PRESSURE) = 100.0f; IEC_VAL(ph.DURATION) = 1000.0f;
    IEC_VAL(ph.FLOWLIMITPERCENT) = 100.0f; IEC_VAL(ph.PRESSURERAMPRATE) = 100000.0f;
    IEC_VAL(ph.CONTINUOUSUPDATE) = true;
    for (i = 0; i < 2000; ++i) {
        float rpm;
        __mcl_cmd_PressureHandle(&ph);
        fb->AXIS_REF.pressure = po.measured_pressure_bar;
        __HydMotion_framework_Publish();
        rpm = (float)fb->_lastCommandedFlow * GAIN;
        PressureModel_Step(&pp, &ps, rpm, DT, &po);
    }
    for (i = 0; i < 10000; ++i) {
        float rpm;
        __mcl_cmd_PressureHandle(&ph);
        fb->AXIS_REF.pressure = po.measured_pressure_bar;
        __HydMotion_framework_Publish();
        rpm = (float)fb->_lastCommandedFlow * GAIN;
        PressureModel_Step(&pp, &ps, rpm, DT, &po);
        if (i >= 8000) { sum += po.measured_pressure_bar; sq += po.measured_pressure_bar*po.measured_pressure_bar; ++n; }
    }
    { float mean = sum/n; r.ess = 100.0f - mean; r.sig = sqrtf(fabsf(sq/n - mean*mean)); }
    r.ok = (r.mp >= 0 && r.mp <= 5.0f) && (fabsf(r.ess) <= 1.0f) && (r.sig <= 1.0f);
    return r;
}

int main(void) {
    const float taus[] = {100.0f, 200.0f, 300.0f, 380.0f, 500.0f};
    const float omegas[] = {12.0f};
    int i, j;
    setvbuf(stdout, NULL, _IONBF, 0);
    printf("%8s %8s | %8s %8s %8s %8s | %s\n", "K", "omega", "Mp%", "tr_ms", "ess_bar", "sigma", "3/3");
    for (i = 0; i < 5; ++i) {
        for (j = 0; j < 1; ++j) {
            Res r = run(taus[i], omegas[j]);
            printf("%8s %8.0f | %+8.2f %8.0f %+8.3f %8.3f | %s\n",
                   "",
                   (double)omegas[j], (double)r.mp, (double)r.tr,
                   (double)r.ess, (double)r.sig, r.ok ? "PASS" : "FAIL");
        }
    }
    return 0;
}
