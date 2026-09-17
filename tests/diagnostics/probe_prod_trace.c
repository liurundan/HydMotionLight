/* 追踪生产链路 FF_PI 的内部量：确认 5 L/min legacy 前馈与 Q_ff 是否叠加 */
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

int main(void) {
    HYD_CREATEMOTION cm;
    HYD_PRESSUREHANDLE ph;
    HYD_WRITEPARAMETER wp;
    HYD_MotionControlFB* fb;
    PressureModelParams pp; PressureModelState ps; PressureModelOutput po;
    int i, axisId = 0;

    __HydMotion_framework_Init();
    memset(&cm, 0, sizeof(cm));
    IEC_VAL(cm.EN) = true;
    IEC_VAL(cm.USE_RECIPE) = false;
    IEC_VAL(cm.FLOW_TO_PUMPSPEED) = GAIN;
    IEC_VAL(cm.PUMPSPEED_LIMIT) = 1700.0f;
    IEC_VAL(cm.USE_SIMULATION) = false;
    __mcl_cmd_CreateMotion(&cm);
    fb = __MK_GetPublic_MotionControlFB(axisId);

    { int k;
      const int pn[3] = {HYD_PARAM_PUMP_DISPLACEMENT, HYD_PARAM_PUMP_VOLUMETRIC_EFF, HYD_PARAM_PUMP_MAX_SPEED};
      const float pv[3] = {25.0f, 0.95f, 1700.0f};
      for (k = 0; k < 3; ++k) {
        memset(&wp, 0, sizeof(wp));
        IEC_VAL(wp.EN) = true; IEC_VAL(wp.EXECUTE) = true; IEC_VAL(wp.AXISID) = 0;
        IEC_VAL(wp.PARAMETERNUMBER) = pn[k]; IEC_VAL(wp.VALUE) = pv[k];
        __mcl_cmd_WriteParameter(&wp);
        if (!IEC_VAL(wp.DONE)) printf("WARN write %d failed err=%u\n", k, (unsigned)IEC_VAL(wp.ERRORID));
      } }

    /* 先跑一拍让段解析完成，再打印解析结果 */

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

    for (i = 0; i < 3; ++i) {
        float rpm;
        __mcl_cmd_PressureHandle(&ph);
        fb->AXIS_REF.pressure = po.measured_pressure_bar;
        __HydMotion_framework_Publish();
        rpm = (float)fb->_lastCommandedFlow * GAIN;
        PressureModel_Step(&pp, &ps, rpm, DT, &po);
    }
    po.measured_pressure_bar = 0.0f; fb->AXIS_REF.pressure = 0.0f;
    printf("seg.targetFlow=%.3f  resolvedKp=%.4f resolvedKi=%.4f Qff=%.4f boost=%.4f\n",
           (double)fb->_activeSegment.targetFlow,
           (double)fb->_pressureController.resolvedKp,
           (double)fb->_pressureController.resolvedKi,
           (double)fb->_pressureController.resolvedSteadyStateFF,
           (double)fb->_pressureController.resolvedBoostFlowLimitLmin);
    printf("%6s %9s %9s %9s %9s\n", "t_ms", "P", "Qcmd", "integral", "P+I+FF+Qff");
    for (i = 0; i < 700; ++i) {
        float rpm;
        __mcl_cmd_PressureHandle(&ph);
        fb->AXIS_REF.pressure = po.measured_pressure_bar;
        __HydMotion_framework_Publish();
        rpm = (float)fb->_lastCommandedFlow * GAIN;
        PressureModel_Step(&pp, &ps, rpm, DT, &po);
        if (i % 20 == 0 || (i > 250 && i < 300)) {
            double e = 150.0 - po.measured_pressure_bar;
            double sum = fb->_pressureController.resolvedSteadyStateFF
                       + fb->_activeSegment.targetFlow
                       + fb->_pressureController.resolvedKp * e
                       + fb->_pressureController.integralOutput;
            printf("%6.0f %9.2f %9.3f %9.3f %9.3f\n",
                   (double)i * DT * 1000.0, po.measured_pressure_bar,
                   (double)fb->_lastCommandedFlow,
                   (double)fb->_pressureController.integralOutput, sum);
        }
    }
    return 0;
}
