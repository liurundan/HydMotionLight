#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "common_types.h"
#include "hydro_hardware.h"
#include "hydro_sim.h"
#include "hydro_sim_fb.h"
#include "motion_interface.h"
#include "motion_control.h"
#include "pressure_controller.h"
#include "pressure_model.h"
#include "ripple_compensator.h"

extern HYD_HydraulicSimFB* __MK_GetPublic_HydraulicSimFB(int index);
extern HYD_MotionControlFB* __MK_GetPublic_MotionControlFB(int index);
extern void __mcl_cmd_SetPumpFeedback(HYD_SETPUMPFEEDBACK *data__);

static HYD_MotionSegment make_rbf_pi_pressure_segment(void);
static HYD_MotionControlFB* create_physical_rbf_pi_axis(void);
static void submit_valid_iec_pump_feedback(HYD_SETPUMPFEEDBACK* command);

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

static void test_iec_set_pump_feedback_contract(void) {
    HYD_SETPUMPFEEDBACK command;

    memset(&command, 0, sizeof(command));
    __SET_VAR(command., ENABLE, , true);
    __SET_VAR(command., ACT_RPM, , 1200.0);
    __SET_VAR(command., ACT_ANGLE_DEG, , 42.0);
    __SET_VAR(command., ACT_TORQUE_PERMILLE, , 375.0);
    __SET_VAR(command., VALID_FLAGS, , (IEC_DWORD)(
        HYD_PUMP_FEEDBACK_VALID_RPM |
        HYD_PUMP_FEEDBACK_VALID_ANGLE |
        HYD_PUMP_FEEDBACK_VALID_TORQUE));

    __mcl_cmd_SetPumpFeedback(&command);

    assert(sizeof(command.VALID_FLAGS.value) == sizeof(IEC_DWORD));
    assert(__GET_VAR(command.ENO));
    assert(__GET_VAR(command.DONE));
    assert(!__GET_VAR(command.BUSY));
    assert(!__GET_VAR(command.ERROR));
    assert(__GET_VAR(command.ERRORID) == HYD_DIAG_CODE_NONE);
}

static void test_iec_set_pump_feedback_sanitizes_nonfinite_fields(void) {
    HYD_SETPUMPFEEDBACK command;
    HYD_MotionControlFB* fb = create_physical_rbf_pi_axis();
    RBF_PID_Handle before;

    submit_valid_iec_pump_feedback(&command);
    __HydMotion_framework_Publish();
    before = fb->_pressureController.rbfPid;

    memset(&command, 0, sizeof(command));
    __SET_VAR(command., ENABLE, , true);
    __SET_VAR(command., ACT_RPM, , NAN);
    __SET_VAR(command., ACT_ANGLE_DEG, , INFINITY);
    __SET_VAR(command., ACT_TORQUE_PERMILLE, , -INFINITY);
    __SET_VAR(command., VALID_FLAGS, , (IEC_DWORD)(
        HYD_PUMP_FEEDBACK_VALID_RPM |
        HYD_PUMP_FEEDBACK_VALID_ANGLE |
        HYD_PUMP_FEEDBACK_VALID_TORQUE));

    __mcl_cmd_SetPumpFeedback(&command);

    assert(__GET_VAR(command.DONE));
    assert(!__GET_VAR(command.ERROR));
    __HydMotion_framework_Publish();
    assert(fb->_pressureController.rbfPid.KP == before.KP);
    assert(fb->_pressureController.rbfPid.KI == before.KI);
    assert(fb->_pressureController.rbfPid.Jacobian == before.Jacobian);
}

static void test_iec_set_pump_feedback_is_fresh_for_one_cycle(void) {
    HYD_SETPUMPFEEDBACK command;
    HYD_MotionControlFB* fb = create_physical_rbf_pi_axis();
    RBF_PID_Handle before;

    submit_valid_iec_pump_feedback(&command);
    __HydMotion_framework_Publish();
    before = fb->_pressureController.rbfPid;

    submit_valid_iec_pump_feedback(&command);
    __SET_VAR(command., ENABLE, , false);
    __mcl_cmd_SetPumpFeedback(&command);
    assert(__GET_VAR(command.DONE));
    __HydMotion_framework_Publish();
    assert(fb->_pressureController.rbfPid.KP == before.KP);
    assert(fb->_pressureController.rbfPid.KI == before.KI);
    assert(fb->_pressureController.rbfPid.Jacobian == before.Jacobian);
}

static HYD_MotionControlFB* create_physical_rbf_pi_axis(void) {
    HYD_CREATEMOTION create_command;
    HYD_MotionControlFB* fb;
    HYD_MotionSegment segment;

    __HydMotion_framework_Init();
    memset(&create_command, 0, sizeof(create_command));
    __SET_VAR(create_command., EN, , true);
    __SET_VAR(create_command., USE_RECIPE, , false);
    __SET_VAR(create_command., FLOW_TO_PUMPSPEED, , 20.0);
    __SET_VAR(create_command., PUMPSPEED_LIMIT, , 1800.0);
    __SET_VAR(create_command., USE_SIMULATION, , false);
    __mcl_cmd_CreateMotion(&create_command);
    assert(__GET_VAR(create_command.DONE));

    fb = __MK_GetPublic_MotionControlFB(__GET_VAR(create_command.AXISID));
    assert(fb != NULL);
    for (int i = 0; i < 3; ++i) {
        __HydMotion_framework_Publish();
    }
    fb->AXIS_REF.pressure = 10.0;
    segment = make_rbf_pi_pressure_segment();
    assert(HYD_MotionControlFB_LoadDirectSegment(fb, &segment));
    assert(HYD_MotionControlFB_StartSegment(fb, 0U, 0.0));
    return fb;
}

static void submit_valid_iec_pump_feedback(HYD_SETPUMPFEEDBACK* command) {
    memset(command, 0, sizeof(*command));
    __SET_VAR(command->, ENABLE, , true);
    __SET_VAR(command->, ACT_RPM, , 1000.0);
    __SET_VAR(command->, ACT_ANGLE_DEG, , 10.0);
    __SET_VAR(command->, ACT_TORQUE_PERMILLE, , 250.0);
    __SET_VAR(command->, VALID_FLAGS, , (IEC_DWORD)(
        HYD_PUMP_FEEDBACK_VALID_RPM |
        HYD_PUMP_FEEDBACK_VALID_ANGLE |
        HYD_PUMP_FEEDBACK_VALID_TORQUE));
    __mcl_cmd_SetPumpFeedback(command);
    assert(__GET_VAR(command->DONE));
}

static void test_iec_pump_feedback_reaches_publish_once_with_control_time(void) {
    HYD_SETPUMPFEEDBACK command;
    HYD_MotionControlFB* fb = create_physical_rbf_pi_axis();
    RBF_PID_Handle before;

    submit_valid_iec_pump_feedback(&command);
    __HydMotion_framework_Publish();
    before = fb->_pressureController.rbfPid;

    submit_valid_iec_pump_feedback(&command);
    __HydMotion_framework_Publish();
    assert(fb->_pressureController.rbfPid.KP != before.KP ||
           fb->_pressureController.rbfPid.KI != before.KI ||
           fb->_pressureController.rbfPid.Jacobian != before.Jacobian);

    /* Segment time is now 3 ms; a retained zero timestamp exceeds the gate. */
    before = fb->_pressureController.rbfPid;
    submit_valid_iec_pump_feedback(&command);
    __HydMotion_framework_Publish();
    assert(fb->_pressureController.rbfPid.KP != before.KP ||
           fb->_pressureController.rbfPid.KI != before.KI ||
           fb->_pressureController.rbfPid.Jacobian != before.Jacobian);

    before = fb->_pressureController.rbfPid;
    __HydMotion_framework_Publish();
    assert(fb->_pressureController.rbfPid.KP == before.KP);
    assert(fb->_pressureController.rbfPid.KI == before.KI);
    assert(fb->_pressureController.rbfPid.Jacobian == before.Jacobian);
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
    assert(HYD_PumpFeedback_HasValid(handle->pumpFeedback.validFlags,
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

static HYD_MotionSegment make_rbf_pi_pressure_segment(void) {
    HYD_MotionSegment segment;

    memset(&segment, 0, sizeof(segment));
    segment.segmentTag = 1;
    segment.segmentType = HYD_SEGMENT_TYPE_HOLDING;
    segment.planner = HYD_PLANNER_TIME_BASED;
    segment.mode = HYD_MODE_PRESSURE_CLOSED_LOOP;
    segment.endCondition = HYD_END_TIME;
    segment.direction = HYD_DIRECTION_HOLD;
    segment.targetPressure = 20.0;
    segment.targetFlow = 0.0;
    segment.maxFlow = 20.0;
    segment.duration = 1.0;
    segment.pressureTolerance = 0.5;
    segment.pressureRampRate = 1000.0;
    segment.pressureController = HYD_PRESSURE_CONTROLLER_RBF_PI;
    segment.pressureRbfConfig.minKp = 0.1;
    segment.pressureRbfConfig.maxKp = 0.5;
    segment.pressureRbfConfig.minKi = 0.001;
    segment.pressureRbfConfig.maxKi = 0.01;
    segment.pressureIntegralLimit = 20.0;
    return segment;
}

static void test_producer_packet_bridges_to_transient_motion_scan(void) {
    PressureModelParams params;
    PressureModelState model_state;
    PressureModelOutput model_output;
    HYD_MotionControlFB fb;
    HYD_MotionSegment segment;
    HYD_RippleCompState ripple_state;

    printf("Testing pressure-model feedback bridge into transient motion scan...\n");
    PressureModel_InitParams(&params);
    params.model_type = PRESSURE_MODEL_TYPE_PHYSICAL_CALIBRATED;
    PressureModel_Reset(&model_state, 17u);
    memset(&model_output, 0, sizeof(model_output));
    PressureModel_Step(&params, &model_state, 100.0f, 0.001f, &model_output);
    assert(HYD_PumpFeedback_HasValid(model_output.pumpFeedback.validFlags,
                                     HYD_PUMP_FEEDBACK_VALID_RPM |
                                     HYD_PUMP_FEEDBACK_VALID_ANGLE |
                                     HYD_PUMP_FEEDBACK_VALID_TORQUE |
                                     HYD_PUMP_FEEDBACK_VALID_TIMESTAMP));

    HYD_MotionControlFB_Init(&fb);
    fb.FLOW_TO_PUMP_SPEED_GAIN = 20.0;
    fb.PUMP_SPEED_LIMIT = 1800.0;
    fb.AXIS_REF.pressure = 10.0;
    fb.AXIS_REF.timestamp = model_output.pumpFeedback.timestamp;
    segment = make_rbf_pi_pressure_segment();
    assert(HYD_MotionControlFB_LoadDirectSegment(&fb, &segment));
    assert(HYD_MotionControlFB_StartSegment(&fb, 0U, 0.0));

    HYD_RippleComp_Reset(&ripple_state);
    HYD_MotionControlFB_ScanWithPumpFeedback(&fb, &model_output.pumpFeedback,
                                              &ripple_state);
    assert(isfinite(fb.PUMP_SPEED));
    assert(fb.FB_STATE == HYD_FB_STATE_RUNNING ||
           fb.FB_STATE == HYD_FB_STATE_STARTING);
    assert(sizeof(HYD_MotionControlFB) <= 3208U);

    /* Legacy scan has no packet/state ownership and stays callable. */
    fb.AXIS_REF.timestamp += 0.001;
    HYD_MotionControlFB_Scan(&fb);
    assert(isfinite(fb.PUMP_SPEED));
    printf("PASS transient motion feedback bridge test\n");
}

static void test_generated_control_timestamp_reaches_rbf_pi_feedback_gate(void) {
    HYD_MotionControlFB fb;
    HYD_MotionSegment segment;
    HYD_PressureControllerState state;
    HYD_PressureControllerInput input;
    HYD_PressureControllerOutput output;
    HYD_PumpFeedback feedback;
    RBF_PID_Handle before;

    HYD_MotionControlFB_Init(&fb);
    fb.AXIS_REF.timestamp = 0.001;
    segment = make_rbf_pi_pressure_segment();
    memset(&state, 0, sizeof(state));
    memset(&input, 0, sizeof(input));
    memset(&output, 0, sizeof(output));
    memset(&feedback, 0, sizeof(feedback));

    input.targetPressure = 20.0;
    input.measuredPressure = 10.0;
    input.timestamp = fb.AXIS_REF.timestamp;
    feedback.rpm = 1000.0;
    feedback.angleDeg = 10.0;
    feedback.torquePermille = 250.0;
    feedback.timestamp = input.timestamp;
    feedback.validFlags = HYD_PUMP_FEEDBACK_VALID_RPM |
                          HYD_PUMP_FEEDBACK_VALID_ANGLE |
                          HYD_PUMP_FEEDBACK_VALID_TORQUE |
                          HYD_PUMP_FEEDBACK_VALID_TIMESTAMP;

    HYD_PressureController_ExecuteWithPumpFeedback(
        &segment, &state, &input, &feedback, &output);
    before = state.rbfPid;

    input.timestamp = 0.002;
    feedback.timestamp = input.timestamp;
    HYD_PressureController_ExecuteWithPumpFeedback(
        &segment, &state, &input, &feedback, &output);
    assert(state.rbfPid.KP != before.KP ||
           state.rbfPid.KI != before.KI ||
           state.rbfPid.Jacobian != before.Jacobian);

    before = state.rbfPid;
    input.timestamp = 0.003;
    feedback.validFlags = 0u;
    HYD_PressureController_ExecuteWithPumpFeedback(
        &segment, &state, &input, &feedback, &output);
    assert(state.rbfPid.KP == before.KP);
    assert(state.rbfPid.KI == before.KI);
    assert(state.rbfPid.Jacobian == before.Jacobian);
}

static void test_invalid_sample_resets_ripple_phase_state(void) {
    HYD_RippleCompState ripple_state;
    HYD_RippleCompOutput ripple_output;
    HYD_PressureRippleEntry entry = {1000.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    HYD_RippleCompTable table = {&entry, 1U, true};
    HYD_PumpFeedback feedback;

    memset(&feedback, 0, sizeof(feedback));
    feedback.rpm = 1000.0;
    feedback.angleDeg = 0.0;
    feedback.timestamp = 0.001;
    feedback.validFlags = HYD_PUMP_FEEDBACK_VALID_RPM |
                          HYD_PUMP_FEEDBACK_VALID_ANGLE |
                          HYD_PUMP_FEEDBACK_VALID_TIMESTAMP;
    HYD_RippleComp_Reset(&ripple_state);
    (void)HYD_RippleComp_Scan(&feedback, &table, &ripple_state,
                              1000.0, 1800.0, &ripple_output);
    assert(ripple_state.initialized);

    memset(&feedback, 0, sizeof(feedback));
    (void)HYD_RippleComp_Scan(&feedback, &table, &ripple_state,
                              1000.0, 1800.0, &ripple_output);
    assert(!ripple_state.initialized);
    assert(!ripple_output.active);
    assert(ripple_output.deltaRpm == 0.0);

    feedback.rpm = 1000.0;
    feedback.angleDeg = 0.0;
    feedback.timestamp = 0.002;
    feedback.validFlags = HYD_PUMP_FEEDBACK_VALID_RPM |
                          HYD_PUMP_FEEDBACK_VALID_ANGLE |
                          HYD_PUMP_FEEDBACK_VALID_TIMESTAMP;
    (void)HYD_RippleComp_Scan(&feedback, &table, &ripple_state,
                              1000.0, 1800.0, &ripple_output);
    assert(ripple_state.initialized);
}

int main(void) {
    test_packet_contract();
    test_iec_set_pump_feedback_contract();
    test_iec_set_pump_feedback_sanitizes_nonfinite_fields();
    test_iec_set_pump_feedback_is_fresh_for_one_cycle();
    test_iec_pump_feedback_reaches_publish_once_with_control_time();
    test_pressure_model_nonfinite_phase_is_not_valid();
    test_simulator_outputs_pump_feedback();
    test_pressure_model_packet_survives_simulator_refresh();
    test_producer_packet_bridges_to_transient_motion_scan();
    test_generated_control_timestamp_reaches_rbf_pi_feedback_gate();
    test_invalid_sample_resets_ripple_phase_state();
    return 0;
}
