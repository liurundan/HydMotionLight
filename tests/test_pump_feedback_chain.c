#include <assert.h>
#include <math.h>
#include <string.h>

#include "common_types.h"
#include "hydro_hardware.h"
#include "hydro_sim.h"
#include "hydro_sim_fb.h"
#include "pressure_model.h"

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

static void assert_valid_feedback_fields_are_finite(const HYD_PumpFeedback* feedback) {
    assert(feedback != NULL);
    if (HYD_PumpFeedback_HasValid(feedback->validFlags,
                                  HYD_PUMP_FEEDBACK_VALID_RPM)) {
        assert(isfinite(feedback->rpm));
    }
    if (HYD_PumpFeedback_HasValid(feedback->validFlags,
                                  HYD_PUMP_FEEDBACK_VALID_ANGLE)) {
        assert(isfinite(feedback->angleDeg));
    }
    if (HYD_PumpFeedback_HasValid(feedback->validFlags,
                                  HYD_PUMP_FEEDBACK_VALID_TORQUE)) {
        assert(isfinite(feedback->torquePermille));
    }
    if (HYD_PumpFeedback_HasValid(feedback->validFlags,
                                  HYD_PUMP_FEEDBACK_VALID_TIMESTAMP)) {
        assert(isfinite(feedback->timestamp));
    }
}

static void test_pressure_model_nonfinite_phase_is_not_valid(void) {
    PressureModelParams params;
    PressureModelState state;
    PressureModelOutput output;

    PressureModel_InitParams(&params);
    PressureModel_Reset(&state, 0x1234u);
    state.pump_phase_rev = NAN;
    memset(&output, 0, sizeof(output));
    PressureModel_Step(&params, &state, 100.0f, 0.001f, &output);
    assert(!HYD_PumpFeedback_HasValid(output.pumpFeedback.validFlags,
                                      HYD_PUMP_FEEDBACK_VALID_ANGLE));
    assert_valid_feedback_fields_are_finite(&output.pumpFeedback);

    PressureModel_Reset(&state, 0x1234u);
    state.pump_phase_rev = INFINITY;
    memset(&output, 0, sizeof(output));
    PressureModel_Step(&params, &state, 100.0f, 0.001f, &output);
    assert(!HYD_PumpFeedback_HasValid(output.pumpFeedback.validFlags,
                                      HYD_PUMP_FEEDBACK_VALID_ANGLE));
    assert_valid_feedback_fields_are_finite(&output.pumpFeedback);
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
    assert(HydraulicSim_SetAxisCommand(&env, 0, true, 60.0f, 1) == 1);
    HydraulicSim_Step(&env, 0.01f);
    assert(HydraulicSim_ReadAxis(&env, 0, &feedback) == 1);
    assert(feedback.pumpFeedback.rpm == 60.0f);
    assert(feedback.pumpFeedback.timestamp > 0.01f);
    assert(HydraulicSim_SetAxisCommand(&env, 0, false, 0.0f, 0) == 1);
    HydraulicSim_Step(&env, 0.01f);
    assert(HydraulicSim_ReadAxis(&env, 0, &feedback) == 1);
    assert(feedback.pumpFeedback.rpm == 0.0f);
    assert(feedback.pumpFeedback.timestamp > 0.02f);

    memset(&pump, 0, sizeof(pump));
    env.axes[0].backend.write_pump(env.axes[0].backend.ctx, &pump);
    assert(pump.feedback_rpm == 0.0f);
    assert(pump.feedback.rpm == 0.0f);

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
                                     HYD_PUMP_FEEDBACK_VALID_TIMESTAMP));
    assert(!HYD_PumpFeedback_HasValid(handle->pumpFeedback.validFlags,
                                      HYD_PUMP_FEEDBACK_VALID_TORQUE));
    model_feedback = handle->pumpFeedback;
    {
        HYD_MOVESIMAXIS move_cmd;

        memset(&move_cmd, 0, sizeof(move_cmd));
        move_cmd.ENABLE.value = true;
        move_cmd.AXISID.value = create_cmd.AXISID.value;
        move_cmd.CMD_RPM.value = 50.0;
        move_cmd.DIRECTION.value = 1;
        __mcl_cmd_moveSimAxis(&move_cmd);
        assert(HYD_PumpFeedback_HasValid(handle->pumpFeedback.validFlags,
                                         HYD_PUMP_FEEDBACK_VALID_RPM));
        assert(!HYD_PumpFeedback_HasValid(handle->pumpFeedback.validFlags,
                                          HYD_PUMP_FEEDBACK_VALID_ANGLE |
                                          HYD_PUMP_FEEDBACK_VALID_TIMESTAMP));
        assert(handle->pumpFeedback.rpm == 50.0f);
    }
    __mcl_cmd_updatePressureModel(&model_cmd);
    model_feedback = handle->pumpFeedback;
    __HydSimulator_framework_Publish();
    HYD_HydraulicSimFB_Cycle(handle);

    assert(handle->pumpFeedback.rpm == model_feedback.rpm);
    assert(handle->pumpFeedback.angleDeg == model_feedback.angleDeg);
    assert(handle->pumpFeedback.torquePermille == model_feedback.torquePermille);
    assert(handle->pumpFeedback.timestamp == model_feedback.timestamp);

    model_cmd.ENABLE.value = false;
    __mcl_cmd_updatePressureModel(&model_cmd);
    assert(!HYD_PumpFeedback_HasValid(handle->pumpFeedback.validFlags,
                                      HYD_PUMP_FEEDBACK_VALID_ANGLE));
    assert(HYD_PumpFeedback_HasValid(handle->pumpFeedback.validFlags,
                                     HYD_PUMP_FEEDBACK_VALID_RPM));
    assert(!HYD_PumpFeedback_HasValid(handle->pumpFeedback.validFlags,
                                      HYD_PUMP_FEEDBACK_VALID_TIMESTAMP));
    assert(handle->pumpFeedback.rpm == 50.0f);
    __HydSimulator_framework_Publish();
    HYD_HydraulicSimFB_Cycle(handle);
    assert(!HYD_PumpFeedback_HasValid(handle->pumpFeedback.validFlags,
                                      HYD_PUMP_FEEDBACK_VALID_ANGLE));
    assert(HYD_PumpFeedback_HasValid(handle->pumpFeedback.validFlags,
                                     HYD_PUMP_FEEDBACK_VALID_RPM |
                                     HYD_PUMP_FEEDBACK_VALID_TIMESTAMP));
    assert(handle->pumpFeedback.rpm == 50.0f);

    {
        HYD_MOVESIMAXIS move_cmd;

        memset(&move_cmd, 0, sizeof(move_cmd));
        move_cmd.ENABLE.value = true;
        move_cmd.AXISID.value = create_cmd.AXISID.value;
        move_cmd.CMD_RPM.value = 40.0;
        move_cmd.DIRECTION.value = 1;
        __mcl_cmd_moveSimAxis(&move_cmd);
        __mcl_cmd_updatePressureModel(&model_cmd);
        assert(handle->pumpFeedback.rpm == 40.0f);
        assert(!HYD_PumpFeedback_HasValid(handle->pumpFeedback.validFlags,
                                          HYD_PUMP_FEEDBACK_VALID_ANGLE));
    }

    model_cmd.ENABLE.value = true;
    model_cmd.MOTOR_RPM.value = NAN;
    model_cmd.TIME_S.value = 0.2;
    __mcl_cmd_updatePressureModel(&model_cmd);
    assert(!HYD_PumpFeedback_HasValid(handle->pumpFeedback.validFlags,
                                      HYD_PUMP_FEEDBACK_VALID_TORQUE));
    if (HYD_PumpFeedback_HasValid(handle->pumpFeedback.validFlags,
                                  HYD_PUMP_FEEDBACK_VALID_RPM)) {
        assert(isfinite(handle->pumpFeedback.rpm));
    }
    if (HYD_PumpFeedback_HasValid(handle->pumpFeedback.validFlags,
                                  HYD_PUMP_FEEDBACK_VALID_ANGLE)) {
        assert(isfinite(handle->pumpFeedback.angleDeg));
    }
    if (HYD_PumpFeedback_HasValid(handle->pumpFeedback.validFlags,
                                  HYD_PUMP_FEEDBACK_VALID_TIMESTAMP)) {
        assert(isfinite(handle->pumpFeedback.timestamp));
    }
}

int main(void) {
    test_packet_contract();
    test_pressure_model_nonfinite_phase_is_not_valid();
    test_simulator_outputs_pump_feedback();
    test_pressure_model_packet_survives_simulator_refresh();
    return 0;
}
