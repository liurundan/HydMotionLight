#include <assert.h>
#include <math.h>
#include <string.h>

#include "common_types.h"
#include "hydro_hardware.h"
#include "hydro_interfaces.h"
#include "hydro_sim.h"
#include "hydro_sim_fb.h"
#include "motion_control.h"
#include "pressure_controller.h"

extern HYD_HydraulicSimFB* __MK_GetPublic_HydraulicSimFB(int index);

static void test_packet_contract(void) {
    HYD_PumpFeedback feedback;

    memset(&feedback, 0, sizeof(feedback));
    assert(feedback.validFlags == 0u);
    assert(!HYD_PumpFeedback_HasValid(feedback.validFlags,
                                      HYD_PUMP_FEEDBACK_VALID_ANGLE));

    feedback.rpm = 20.0;
    feedback.angleDeg = 359.5;
    feedback.torquePermille = 6242.0;
    feedback.timestamp = 1.000;
    feedback.validFlags = HYD_PUMP_FEEDBACK_VALID_RPM |
                          HYD_PUMP_FEEDBACK_VALID_ANGLE |
                          HYD_PUMP_FEEDBACK_VALID_TORQUE;
    assert(HYD_PumpFeedback_HasValid(feedback.validFlags,
                                     HYD_PUMP_FEEDBACK_VALID_ANGLE));
    assert(feedback.torquePermille == 6242.0);
}

static void test_packet_is_embedded_at_each_boundary(void) {
    AxisFeedback axis_feedback;
    HydroPump pump;
    HYD_PressureControllerInput pressure_input;
    HYD_AxisRef axis_ref;

    memset(&axis_feedback, 0, sizeof(axis_feedback));
    memset(&pump, 0, sizeof(pump));
    memset(&pressure_input, 0, sizeof(pressure_input));
    memset(&axis_ref, 0, sizeof(axis_ref));

    axis_feedback.pumpFeedback.rpm = 12.0;
    pump.feedback = axis_feedback.pumpFeedback;
    pressure_input.pumpFeedback = pump.feedback;
    axis_ref.pumpFeedback = pressure_input.pumpFeedback;

    assert(axis_ref.pumpFeedback.rpm == 12.0);
}

static void test_native_setter_clears_nonfinite_fields(void) {
    HYD_MotionControlFB fb;
    HYD_PumpFeedback feedback;

    memset(&fb, 0, sizeof(fb));
    HYD_MotionControlFB_Init(&fb);

    memset(&feedback, 0, sizeof(feedback));
    feedback.rpm = 10.0;
    feedback.angleDeg = NAN;
    feedback.torquePermille = INFINITY;
    feedback.timestamp = 2.0;
    feedback.validFlags = HYD_PUMP_FEEDBACK_VALID_RPM |
                          HYD_PUMP_FEEDBACK_VALID_ANGLE |
                          HYD_PUMP_FEEDBACK_VALID_TORQUE |
                          HYD_PUMP_FEEDBACK_VALID_TIMESTAMP;

    HYD_MotionControlFB_SetPumpFeedback(&fb, &feedback);
    assert(fb.AXIS_REF.pumpFeedback.rpm == 10.0);
    assert(!HYD_PumpFeedback_HasValid(fb.AXIS_REF.pumpFeedback.validFlags,
                                      HYD_PUMP_FEEDBACK_VALID_ANGLE));
    assert(!HYD_PumpFeedback_HasValid(fb.AXIS_REF.pumpFeedback.validFlags,
                                      HYD_PUMP_FEEDBACK_VALID_TORQUE));
    assert(HYD_PumpFeedback_HasValid(fb.AXIS_REF.pumpFeedback.validFlags,
                                     HYD_PUMP_FEEDBACK_VALID_RPM));

    HYD_MotionControlFB_SetPumpFeedback(&fb, NULL);
    assert(fb.AXIS_REF.pumpFeedback.validFlags == 0u);
}

static void test_simulator_outputs_pump_feedback(void) {
    HydraulicSimEnv env;
    AxisFeedback feedback;
    HydroPump pump;

    HydraulicSim_Init(&env);
    assert(HydraulicSim_RegisterAxis(&env, 0, SIM_AXIS_CLAMP) == 1);
    assert(HydraulicSim_SetAxisCommand(&env, 0, true, 120.0f, 1) == 1);
    HydraulicSim_Step(&env, 0.01f);
    assert(HydraulicSim_ReadAxis(&env, 0, &feedback) == 1);
    assert(HYD_PumpFeedback_HasValid(feedback.pumpFeedback.validFlags,
                                     HYD_PUMP_FEEDBACK_VALID_RPM |
                                     HYD_PUMP_FEEDBACK_VALID_TIMESTAMP));
    assert(feedback.pumpFeedback.rpm == 120.0f);
    assert(feedback.pumpFeedback.timestamp == 0.01f);

    memset(&pump, 0, sizeof(pump));
    env.axes[0].backend.write_pump(env.axes[0].backend.ctx, &pump);
    assert(pump.feedback_rpm == 120.0f);
    assert(pump.feedback.rpm == 120.0f);

    {
        HYD_CREATESIMAXIS create_cmd;
        HYD_MOVESIMAXIS move_cmd;
        HYD_HydraulicSimFB *handle;

        __HydSimulator_framework_Init();
        memset(&create_cmd, 0, sizeof(create_cmd));
        create_cmd.AXISTYPE.value = SIM_AXIS_CLAMP;
        create_cmd.MAXVEL.value = 100.0;
        create_cmd.MAXACC.value = 100.0;
        create_cmd.MAXDEC.value = 100.0;
        __mcl_cmd_createSimAxis(&create_cmd);
        memset(&move_cmd, 0, sizeof(move_cmd));
        move_cmd.ENABLE.value = true;
        move_cmd.AXISID.value = create_cmd.AXISID.value;
        move_cmd.CMD_RPM.value = 80.0;
        move_cmd.DIRECTION.value = 1;
        __mcl_cmd_moveSimAxis(&move_cmd);
        __HydSimulator_framework_Publish();
        handle = __MK_GetPublic_HydraulicSimFB(create_cmd.AXISID.value);
        assert(handle != NULL);
        assert(HYD_PumpFeedback_HasValid(handle->pumpFeedback.validFlags,
                                         HYD_PUMP_FEEDBACK_VALID_RPM));
        assert(handle->pumpFeedback.rpm == 80.0f);
    }
}

static void test_pressure_model_packet_survives_simulator_refresh(void) {
    HYD_CREATESIMAXIS create_cmd;
    HYD_PRESSUREMODEL model_cmd;
    HYD_HydraulicSimFB *handle;
    HYD_PumpFeedback model_feedback;

    __HydSimulator_framework_Init();
    memset(&create_cmd, 0, sizeof(create_cmd));
    create_cmd.AXISTYPE.value = SIM_AXIS_CLAMP;
    create_cmd.MAXVEL.value = 100.0;
    create_cmd.MAXACC.value = 100.0;
    create_cmd.MAXDEC.value = 100.0;
    __mcl_cmd_createSimAxis(&create_cmd);

    memset(&model_cmd, 0, sizeof(model_cmd));
    model_cmd.ENABLE.value = true;
    model_cmd.MOTOR_RPM.value = 1000.0;
    model_cmd.TIME_S.value = 0.1;
    __mcl_cmd_updatePressureModel(&model_cmd);

    handle = __MK_GetPublic_HydraulicSimFB(create_cmd.AXISID.value);
    assert(handle != NULL);
    assert(HYD_PumpFeedback_HasValid(handle->pumpFeedback.validFlags,
                                     HYD_PUMP_FEEDBACK_VALID_RPM |
                                     HYD_PUMP_FEEDBACK_VALID_ANGLE |
                                     HYD_PUMP_FEEDBACK_VALID_TORQUE |
                                     HYD_PUMP_FEEDBACK_VALID_TIMESTAMP));
    model_feedback = handle->pumpFeedback;
    __HydSimulator_framework_Publish();
    HYD_HydraulicSimFB_Cycle(handle);

    assert(handle->pumpFeedback.rpm == model_feedback.rpm);
    assert(handle->pumpFeedback.angleDeg == model_feedback.angleDeg);
    assert(handle->pumpFeedback.torquePermille == model_feedback.torquePermille);
    assert(handle->pumpFeedback.timestamp == model_feedback.timestamp);
}

int main(void) {
    test_packet_contract();
    test_packet_is_embedded_at_each_boundary();
    test_native_setter_clears_nonfinite_fields();
    test_simulator_outputs_pump_feedback();
    test_pressure_model_packet_survives_simulator_refresh();
    return 0;
}
