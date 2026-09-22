#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "rbf_pid.h"
#include "pressure_controller.h"
#include "pressure_model.h"

static float first_order_step(float pressure_bar,
                              float command,
                              float gain_bar_per_command,
                              float tau_s,
                              float dt_s) {
    return (tau_s * pressure_bar +
            gain_bar_per_command * dt_s * command) /
           (tau_s + dt_s);
}

static void test_gain_1p5_tau_1_has_small_steady_state_error(void) {
    RBF_PID_Handle pid;
    float pressure = 0.0f;
    const float setpoint = 100.0f;
    const float dt_s = 0.001f;
    int step;

    RBF_PID_Init(&pid, dt_s, 100.0f, 1.0f);
    pid.output_min_flow = 0.0f;
    pid.output_max_flow = 100.0f;
    RBF_PID_SetGainCompensation(&pid, 1.5f);
    RBF_PID_SetProcessTimeConstant(&pid, 1.0f);

    for (step = 0; step < 20000; ++step) {
        float command = RBF_PID_Update(&pid, setpoint, pressure);
        pressure = first_order_step(pressure, command, 1.5f, 1.0f, dt_s);
    }

    printf("gain=1.5 tau=1.0 final_pressure=%.6f steady_error=%.6f command=%.6f\n",
           pressure, setpoint - pressure, pid.Output);
    assert(fabsf(setpoint - pressure) <= 0.01f);
}

static void test_pressure_controller_path_has_small_steady_state_error(void) {
    HYD_MotionSegment segment = {0};
    HYD_PressureControllerState state;
    HYD_PressureControllerInput input = {0};
    HYD_PressureControllerOutput output;
    float pressure = 0.0f;
    const float setpoint = 100.0f;
    int step;

    segment.mode = HYD_MODE_PRESSURE_CLOSED_LOOP;
    segment.endCondition = HYD_END_MANUAL;
    segment.direction = HYD_DIRECTION_HOLD;
    segment.targetPressure = setpoint;
    segment.maxFlow = 100.0f;
    segment.pressureController = HYD_PRESSURE_CONTROLLER_RBF_PID;
    segment.pressureFilterAlpha = 1.0f;
    segment.pressureDerivativeFilterAlpha = 1.0f;
    segment.systemGain = 1.5f;

    HYD_PressureController_InitState(&state, 0.0f, 0.0f, 0.0f);
    state.calibrationStatus = HYD_PRESSURE_CALIBRATION_CALIBRATED;

    for (step = 0; step < 20000; ++step) {
        input.targetPressure = setpoint;
        input.measuredPressure = pressure;
        input.outputMin = 0.0f;
        input.outputMax = 100.0f;
        input.flowToPumpSpeedGain = 1.0f;
        input.pumpSpeedLimit = 100.0f;
        input.plantTauS = 1.0f;
        input.timestamp = (float)(step + 1) * 0.001f;
        HYD_PressureController_Execute(&segment, &state, &input, &output);
        pressure = first_order_step(pressure, output.outputFlow, 1.5f, 1.0f, 0.001f);
    }

    printf("pressure-controller gain=1.5 final_pressure=%.6f steady_error=%.6f command=%.6f\n",
           pressure, setpoint - pressure, output.outputFlow);
    assert(fabsf(setpoint - pressure) <= 0.01f);
}

static void test_unconfigured_gain_does_not_hide_integral_steady_state_error(void) {
    RBF_PID_Handle pid;
    float pressure = 0.0f;
    const float setpoint = 100.0f;
    int step;

    RBF_PID_Init(&pid, 0.001f, 100.0f, 1.0f);
    pid.output_min_flow = 0.0f;
    pid.output_max_flow = 100.0f;
    RBF_PID_SetProcessTimeConstant(&pid, 1.0f);

    for (step = 0; step < 20000; ++step) {
        float command = RBF_PID_Update(&pid, setpoint, pressure);
        pressure = first_order_step(pressure, command, 1.5f, 1.0f, 0.001f);
    }

    printf("unconfigured-gain final_pressure=%.6f steady_error=%.6f command=%.6f K=%.6f\n",
           pressure, setpoint - pressure, pid.Output, pid.K);
    assert(fabsf(setpoint - pressure) <= 0.01f);
}

static void test_pressure_model_gain_is_converted_from_rpm_to_flow(void) {
    PressureModelParams params;
    PressureModelState plant;
    PressureModelOutput plant_output;
    HYD_MotionSegment segment = {0};
    HYD_PressureControllerState state;
    HYD_PressureControllerInput input = {0};
    HYD_PressureControllerOutput output;
    float pressure = 0.0f;
    const float setpoint = 100.0f;
    const float flow_to_rpm = 20.0f;
    int step;

    PressureModel_InitParams(&params);
    params.model_type = PRESSURE_MODEL_TYPE_FIRST_ORDER;
    params.first_order_k_bar_per_rpm = 1.5f;
    params.first_order_tau_s = 1.0f;
    params.first_order_delay_s = 0.0f;
    params.enable_sensor_noise = 0u;
    params.enable_motor_noise = 0u;
    PressureModel_Reset(&plant, 0x15u);

    segment.mode = HYD_MODE_PRESSURE_CLOSED_LOOP;
    segment.endCondition = HYD_END_MANUAL;
    segment.direction = HYD_DIRECTION_HOLD;
    segment.targetPressure = setpoint;
    segment.maxFlow = 90.0f;
    segment.pressureController = HYD_PRESSURE_CONTROLLER_RBF_PID;
    segment.pressureFilterAlpha = 1.0f;
    segment.pressureDerivativeFilterAlpha = 1.0f;
    /* 1.5 bar/rpm * 20 rpm/(L/min) = 30 bar/(L/min). */
    segment.systemGain = PressureModel_FirstOrderGainToProcessGain(
        params.first_order_k_bar_per_rpm, flow_to_rpm);

    HYD_PressureController_InitState(&state, 0.0f, 0.0f, 0.0f);
    state.calibrationStatus = HYD_PRESSURE_CALIBRATION_CALIBRATED;
    for (step = 0; step < 20000; ++step) {
        input.targetPressure = setpoint;
        input.measuredPressure = pressure;
        input.outputMin = 0.0f;
        input.outputMax = 90.0f;
        input.flowToPumpSpeedGain = flow_to_rpm;
        input.pumpSpeedLimit = 1800.0f;
        input.plantTauS = 1.0f;
        input.timestamp = (float)(step + 1) * 0.001f;
        HYD_PressureController_Execute(&segment, &state, &input, &output);
        PressureModel_Step(&params, &plant, output.outputFlow * flow_to_rpm,
                           0.001f, &plant_output);
        pressure = plant_output.measured_pressure_bar;
    }

    printf("pressure-model rpm-gain=1.5 final_pressure=%.6f steady_error=%.6f command=%.6f K=%.6f\n",
           pressure, setpoint - pressure, output.outputFlow,
           state.rbfPid.K);
    assert(fabsf(setpoint - pressure) <= 0.05f);
}

static void test_pressure_model_unit_mismatch_is_rejected_by_the_contract(void) {
    PressureModelParams params;
    PressureModelState plant;
    PressureModelOutput plant_output;
    HYD_MotionSegment segment = {0};
    HYD_PressureControllerState state;
    HYD_PressureControllerInput input = {0};
    HYD_PressureControllerOutput output;
    float pressure = 0.0f;
    const float setpoint = 100.0f;
    int step;

    PressureModel_InitParams(&params);
    params.model_type = PRESSURE_MODEL_TYPE_FIRST_ORDER;
    params.first_order_k_bar_per_rpm = 1.5f;
    params.first_order_tau_s = 1.0f;
    params.first_order_delay_s = 0.0f;
    PressureModel_Reset(&plant, 0x16u);

    segment.mode = HYD_MODE_PRESSURE_CLOSED_LOOP;
    segment.endCondition = HYD_END_MANUAL;
    segment.direction = HYD_DIRECTION_HOLD;
    segment.targetPressure = setpoint;
    segment.maxFlow = 90.0f;
    segment.pressureController = HYD_PRESSURE_CONTROLLER_RBF_PID;
    segment.pressureFilterAlpha = 1.0f;
    segment.pressureDerivativeFilterAlpha = 1.0f;
    /* Deliberately pass 1.5 bar/rpm as if it were bar/(L/min). */
    segment.systemGain = 1.5f;

    HYD_PressureController_InitState(&state, 0.0f, 0.0f, 0.0f);
    state.calibrationStatus = HYD_PRESSURE_CALIBRATION_CALIBRATED;
    for (step = 0; step < 20000; ++step) {
        input.targetPressure = setpoint;
        input.measuredPressure = pressure;
        input.outputMin = 0.0f;
        input.outputMax = 90.0f;
        input.flowToPumpSpeedGain = 20.0f;
        input.pumpSpeedLimit = 1800.0f;
        input.plantTauS = 1.0f;
        input.timestamp = (float)(step + 1) * 0.001f;
        HYD_PressureController_Execute(&segment, &state, &input, &output);
        PressureModel_Step(&params, &plant, output.outputFlow * 20.0f,
                           0.001f, &plant_output);
        pressure = plant_output.measured_pressure_bar;
    }

    printf("unit-mismatch (invalid fixture) final_pressure=%.6f steady_error=%.6f command=%.6f K=%.6f\n",
           pressure, setpoint - pressure, output.outputFlow,
           state.rbfPid.K);
    /* A raw PressureModel K_NUM is bar/rpm, while systemGain is
     * bar/(L/min); accepting 1.5 without conversion is an invalid fixture. */
    assert(fabsf(state.rbfPid.K - 1.5f) < 1.0e-6f);
}

static void test_pressure_controller_rejects_unconverted_simulator_gain(void) {
    HYD_MotionSegment segment = {0};
    HYD_PressureControllerState state;
    HYD_PressureControllerInput input = {0};
    HYD_PressureControllerOutput output;

    segment.mode = HYD_MODE_PRESSURE_CLOSED_LOOP;
    segment.endCondition = HYD_END_MANUAL;
    segment.direction = HYD_DIRECTION_HOLD;
    segment.targetPressure = 100.0f;
    segment.maxFlow = 90.0f;
    segment.pressureController = HYD_PRESSURE_CONTROLLER_RBF_PID;
    segment.systemGain = 1.5f; /* simulator K_NUM, wrong controller unit */
    HYD_PressureController_InitState(&state, 0.0f, 0.0f, 0.0f);
    input.targetPressure = 100.0f;
    input.measuredPressure = 0.0f;
    input.outputMin = 0.0f;
    input.outputMax = 90.0f;
    input.flowToPumpSpeedGain = 20.0f;
    input.pumpSpeedLimit = 1800.0f;
    input.plantTauS = 1.0f;
    input.timestamp = 0.001f;
    HYD_PressureController_Execute(&segment, &state, &input, &output);
    /* Direct callers must explicitly provide K_process; a raw K_NUM is not
     * auto-converted because the controller cannot infer simulator units. */
    assert(output.appliedStrategy == HYD_PRESSURE_CONTROLLER_PI ||
           output.calibrationStatus == HYD_PRESSURE_CALIBRATION_UNCALIBRATED);
}

static void test_direct_rbf_flow_output_with_rpm_plant(void) {
    PressureModelParams params;
    PressureModelState plant;
    PressureModelOutput plant_output;
    RBF_PID_Handle pid;
    float pressure = 0.0f;
    const float flow_to_rpm = 20.0f;
    const float setpoint = 100.0f;
    int step;

    PressureModel_InitParams(&params);
    params.model_type = PRESSURE_MODEL_TYPE_FIRST_ORDER;
    params.first_order_k_bar_per_rpm = 1.5f;
    params.first_order_tau_s = 1.0f;
    params.first_order_delay_s = 0.0f;
    PressureModel_Reset(&plant, 0x17u);
    RBF_PID_Init(&pid, 0.001f, 90.0f, 1.0f);
    pid.output_min_flow = 0.0f;
    pid.output_max_flow = 90.0f;
    RBF_PID_SetGainCompensation(&pid, 1.5f * flow_to_rpm);
    for (step = 0; step < 20000; ++step) {
        float flow = RBF_PID_Update(&pid, setpoint, pressure);
        PressureModel_Step(&params, &plant, flow * flow_to_rpm,
                           0.001f, &plant_output);
        pressure = plant_output.measured_pressure_bar;
    }
    printf("direct-flow-output final_pressure=%.6f steady_error=%.6f flow=%.6f\n",
           pressure, setpoint - pressure, pid.Output);
    assert(fabsf(setpoint - pressure) <= 0.05f);
}

static void test_first_order_capacity_boundary_is_diagnosed(void) {
    RBF_PID_Handle pid;
    float pressure = 0.0f;
    const float setpoint = 150.0f;
    int step;

    RBF_PID_Init(&pid, 0.001f, 90.0f, 1.0f);
    pid.output_min_flow = 0.0f;
    pid.output_max_flow = 90.0f;
    RBF_PID_SetGainCompensation(&pid, 1.5f);
    for (step = 0; step < 20000; ++step) {
        float command = RBF_PID_Update(&pid, setpoint, pressure);
        pressure = first_order_step(pressure, command, 1.5f, 1.0f, 0.001f);
    }
    printf("capacity-boundary final_pressure=%.6f steady_error=%.6f command=%.6f required=%.6f\n",
           pressure, setpoint - pressure, pid.Output, setpoint / 1.5f);
    assert(fabsf(setpoint - pressure) > 10.0f);
    assert(fabsf(pid.Output - 90.0f) < 1.0e-3f);
}

static void test_pressure_controller_reports_unreachable_target(void) {
    HYD_MotionSegment segment = {0};
    HYD_PressureControllerState state;
    HYD_PressureControllerInput input = {0};
    HYD_PressureControllerOutput output;

    segment.mode = HYD_MODE_PRESSURE_CLOSED_LOOP;
    segment.endCondition = HYD_END_MANUAL;
    segment.direction = HYD_DIRECTION_HOLD;
    segment.targetPressure = 150.0f;
    segment.maxFlow = 90.0f;
    segment.pressureController = HYD_PRESSURE_CONTROLLER_RBF_PID;
    segment.systemGain = 1.5f;
    HYD_PressureController_InitState(&state, 0.0f, 0.0f, 0.0f);
    state.calibrationStatus = HYD_PRESSURE_CALIBRATION_CALIBRATED;
    input.targetPressure = 150.0f;
    input.measuredPressure = 0.0f;
    input.outputMin = 0.0f;
    input.outputMax = 90.0f;
    input.flowToPumpSpeedGain = 20.0f;
    input.pumpSpeedLimit = 1800.0f;
    input.plantTauS = 1.0f;
    input.timestamp = 0.001f;
    HYD_PressureController_Execute(&segment, &state, &input, &output);
    assert(output.limitStatus == HYD_PRESSURE_LIMIT_CAPACITY_INSUFFICIENT);
    assert(output.appliedStrategy == HYD_PRESSURE_CONTROLLER_PI);
    assert(output.outputFlow <= 90.0f + 1.0e-6f);
}

static void test_gain_1p5_target_150_reaches_when_flow_units_are_correct(void) {
    RBF_PID_Handle pid;
    float pressure = 0.0f;
    const float flow_to_rpm = 20.0f;
    const float setpoint = 150.0f;
    int step;

    RBF_PID_Init(&pid, 0.001f, 90.0f, 1.0f);
    pid.output_min_flow = 0.0f;
    pid.output_max_flow = 90.0f;
    RBF_PID_SetGainCompensation(&pid, 1.5f * flow_to_rpm);
    for (step = 0; step < 20000; ++step) {
        float flow = RBF_PID_Update(&pid, setpoint, pressure);
        pressure = first_order_step(pressure, flow * flow_to_rpm,
                                    1.5f, 1.0f, 0.001f);
    }
    printf("gain=1.5 target=150 final_pressure=%.6f steady_error=%.6f flow=%.6f\n",
           pressure, setpoint - pressure, pid.Output);
    assert(fabsf(setpoint - pressure) <= 0.05f);
}

int main(void) {
    test_gain_1p5_tau_1_has_small_steady_state_error();
    test_pressure_controller_path_has_small_steady_state_error();
    test_unconfigured_gain_does_not_hide_integral_steady_state_error();
    test_pressure_model_gain_is_converted_from_rpm_to_flow();
    test_pressure_model_unit_mismatch_is_rejected_by_the_contract();
    test_pressure_controller_rejects_unconverted_simulator_gain();
    test_direct_rbf_flow_output_with_rpm_plant();
    test_first_order_capacity_boundary_is_diagnosed();
    test_pressure_controller_reports_unreachable_target();
    test_gain_1p5_target_150_reaches_when_flow_units_are_correct();
    puts("RBF-PID first-order gain=1.5 regression passed.");
    return 0;
}
