/* 量"停滞"与"正常升压末期"的压力速率，为包络停滞解除定阈值 */
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

static void wr(int p, float v) {
    HYD_WRITEPARAMETER wp;
    memset(&wp, 0, sizeof(wp));
    IEC_VAL(wp.EN) = true; IEC_VAL(wp.EXECUTE) = true; IEC_VAL(wp.AXISID) = 0;
    IEC_VAL(wp.PARAMETERNUMBER) = (IEC_DINT)p; IEC_VAL(wp.VALUE) = (IEC_LREAL)v;
    __mcl_cmd_WriteParameter(&wp);
}

static void scenario(float K, const char* tag) {
    HYD_CREATEMOTION cm; HYD_PRESSUREHANDLE ph; HYD_MotionControlFB* fb;
    PressureModelParams pp; PressureModelState ps; PressureModelOutput po;
    int i; float ring[40]; int ri = 0; float pmin_rate = 1e9f; int pmin_at = -1;
    float plast = 0.0f;

    __HydMotion_framework_Init();
    memset(&cm, 0, sizeof(cm));
    IEC_VAL(cm.EN) = true; IEC_VAL(cm.USE_RECIPE) = false;
    IEC_VAL(cm.FLOW_TO_PUMPSPEED) = GAIN; IEC_VAL(cm.PUMPSPEED_LIMIT) = 1700.0f;
    IEC_VAL(cm.USE_SIMULATION) = false;
    __mcl_cmd_CreateMotion(&cm);
    fb = __MK_GetPublic_MotionControlFB(0);
    wr(HYD_PARAM_PUMP_DISPLACEMENT, 25.0f);
    wr(HYD_PARAM_PUMP_VOLUMETRIC_EFF, 0.95f);
    wr(HYD_PARAM_PUMP_MAX_SPEED, 1700.0f);
    if (K > 0.0f) wr(HYD_PARAM_PRESSURE_SYSTEM_GAIN, K);

    PressureModel_InitParams(&pp);
    pp.enable_sensor_noise = 1; pp.enable_motor_noise = 1;   /* 带噪声 */
    PressureModel_Reset(&ps, 0x1234u);
    memset(&po, 0, sizeof(po)); memset(ring, 0, sizeof(ring));
    fb->AXIS_REF.pressure = 0.0f;
    memset(&ph, 0, sizeof(ph));
    IEC_VAL(ph.EN) = true; IEC_VAL(ph.EXECUTE) = true; IEC_VAL(ph.AXISID) = 0;
    IEC_VAL(ph.PRESSURE) = 150.0f; IEC_VAL(ph.DURATION) = 1000.0f;
    IEC_VAL(ph.FLOWLIMITPERCENT) = 100.0f; IEC_VAL(ph.PRESSURERAMPRATE) = 100000.0f;
    IEC_VAL(ph.CONTINUOUSUPDATE) = true;

    printf("[%s K=%.0f] t_ms   P      rate20ms_bar_s\n", tag, (double)K);
    for (i = 0; i < 1500; ++i) {
        float rpm, rate;
        __mcl_cmd_PressureHandle(&ph);
        fb->AXIS_REF.pressure = po.measured_pressure_bar;
        __HydMotion_framework_Publish();
        rpm = (float)fb->_lastCommandedFlow * GAIN;
        PressureModel_Step(&pp, &ps, rpm, DT, &po);
        ring[ri] = po.measured_pressure_bar; ri = (ri + 1) % 40;
        rate = (po.measured_pressure_bar - ring[ri]) / (0.040f);  /* 40 ms 窗口 */
        if (i > 200 && rate < pmin_rate) { pmin_rate = rate; pmin_at = i; }
        if (i % 100 == 0)
            printf("        %6d %7.2f %10.2f\n", i, po.measured_pressure_bar, (double)rate);
    }
    printf("        >>> 200ms 后最小 40ms 窗口速率 = %.2f bar/s @ %d ms, 终值 %.2f bar\n\n",
           (double)pmin_rate, pmin_at, po.measured_pressure_bar);
}

int main(void) {
    setvbuf(stdout, NULL, _IONBF, 0);
    scenario(200.0f, "名义");
    scenario(500.0f, "高估2.5x");
    return 0;
}
