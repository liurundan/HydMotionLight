#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "motion_control.h"
#include "toggle_mechanism_pool.h"

static void assert_near(HYD_REAL actual, HYD_REAL expected,
                        HYD_REAL tolerance)
{
    assert(fabsf(actual - expected) <= tolerance);
}

static HYD_TogglePreparedConfig prepare_default_toggle(void)
{
    HYD_ToggleGeometryConfig raw = HYD_ToggleKinematics_DefaultConfig();
    HYD_TogglePreparedConfig prepared;
    HYD_ToggleError error = HYD_TOGGLE_ERROR_NONE;

    assert(HYD_ToggleKinematics_ValidateBlocking(&raw, &prepared, &error));
    return prepared;
}

static void bind_toggle(HYD_MotionControlFB *fb,
                        const HYD_TogglePreparedConfig *prepared)
{
    HYD_UINT8 slot = HYD_TOGGLE_SLOT_NONE;

    HYD_ToggleMechanismPool_Reset();
    assert(HYD_ToggleMechanismPool_Reserve(0u, &slot));
    assert(HYD_ToggleMechanismPool_Commit(slot, prepared, true));
    fb->mechanismType = (HYD_UINT8)HYD_MECHANISM_FIVE_POINT_TOGGLE;
    fb->mechanismSlot = slot;
    fb->STATE.mechanismType = fb->mechanismType;
    fb->STATE.mechanismConfigVersion =
        HYD_ToggleMechanismPool_GetVersion(slot);
}

static HYD_MotionSegment make_speed_segment(void)
{
    HYD_MotionSegment segment;

    memset(&segment, 0, sizeof(segment));
    segment.segmentType = HYD_SEGMENT_TYPE_OTHER;
    segment.planner = HYD_PLANNER_TIME_BASED;
    segment.mode = HYD_MODE_SPEED_RAMP;
    segment.endCondition = HYD_END_MANUAL;
    segment.direction = HYD_DIRECTION_EXTEND;
    segment.maxVelocity = 20.0f;
    segment.maxAcceleration = 100.0f;
    segment.maxDeceleration = 100.0f;
    segment.maxFlow = 100.0f;
    segment.velocityToFlowGain = 2.0f;
    return segment;
}

static HYD_MotionSegment make_position_segment(void)
{
    HYD_MotionSegment segment = make_speed_segment();

    segment.planner = HYD_PLANNER_POSITION_BASED;
    segment.mode = HYD_MODE_POSITION;
    segment.endCondition = HYD_END_POSITION;
    segment.targetPosition = 100.0f;
    segment.maxAcceleration = 10.0f;
    segment.maxDeceleration = 10.0f;
    segment.positionTolerance = 0.01f;
    return segment;
}

static void run_first_control_cycle(HYD_MotionControlFB *fb,
                                    const HYD_MotionSegment *segment,
                                    HYD_REAL position)
{
    HYD_MotionControlFB_Init(fb);
    fb->USE_RECIPE = false;
    fb->FLOW_TO_PUMP_SPEED_GAIN = 10.0f;
    fb->PUMP_SPEED_LIMIT = 3000.0f;
    fb->AXIS_REF.position = position;
    fb->AXIS_REF.velocity = 0.0f;
    fb->AXIS_REF.flow = 0.0f;
    fb->AXIS_REF.pressure = 20.0f;
    fb->AXIS_REF.timestamp = 0.0f;

    assert(HYD_MotionControlFB_LoadDirectSegment(fb, segment));
    assert(HYD_MotionControlFB_StartSegment(fb, 0u, 0.0f));
    HYD_MotionControlFB_Scan(fb);
    fb->AXIS_REF.timestamp = 0.1f;
    HYD_MotionControlFB_Scan(fb);
}

static void test_direct_position_and_velocity_flow_are_unchanged(void)
{
    HYD_MotionControlFB position_fb;
    HYD_MotionControlFB velocity_fb;
    HYD_MotionSegment position_segment = make_position_segment();
    HYD_MotionSegment velocity_segment = make_speed_segment();

    run_first_control_cycle(&position_fb, &position_segment, 0.0f);
    assert_near(position_fb.STATE.plannedVelocity, 1.0f, 0.001f);
    assert_near(position_fb.STATE.plannedFlow, 2.0f, 0.001f);
    assert_near(position_fb.PUMP_SPEED, 20.0f, 0.001f);

    run_first_control_cycle(&velocity_fb, &velocity_segment, 0.0f);
    assert_near(velocity_fb.STATE.plannedVelocity, 10.0f, 0.001f);
    assert_near(velocity_fb.STATE.plannedFlow, 20.0f, 0.001f);
    assert_near(velocity_fb.PUMP_SPEED, 200.0f, 0.001f);
}

static void run_toggle_speed_cycle(HYD_MotionControlFB *fb,
                                   const HYD_TogglePreparedConfig *prepared,
                                   HYD_REAL position)
{
    HYD_MotionSegment segment = make_speed_segment();

    run_first_control_cycle(fb, &segment, position);
    bind_toggle(fb, prepared);

    fb->AXIS_REF.timestamp = 0.2f;
    HYD_MotionControlFB_Scan(fb);
}

static void test_toggle_mapping_varies_with_position_in_actuator_space(void)
{
    HYD_TogglePreparedConfig prepared = prepare_default_toggle();
    HYD_MotionControlFB near_open;
    HYD_MotionControlFB near_closed;
    HYD_ToggleSolution open_solution;
    HYD_ToggleSolution closed_solution;
    HYD_ToggleError error = HYD_TOGGLE_ERROR_NONE;
    HYD_REAL expected_open_flow;
    HYD_REAL expected_closed_flow;

    run_toggle_speed_cycle(&near_open, &prepared, 101.0f);
    assert(HYD_ToggleKinematics_SolveOnline(
        &prepared, 101.0f, near_open.STATE.plannedVelocity,
        &open_solution, &error));
    expected_open_flow = fabsf(open_solution.vs) * 2.0f;

    run_toggle_speed_cycle(&near_closed, &prepared, 180.0f);
    assert(HYD_ToggleKinematics_SolveOnline(
        &prepared, 180.0f, near_closed.STATE.plannedVelocity,
        &closed_solution, &error));
    expected_closed_flow = fabsf(closed_solution.vs) * 2.0f;

    assert_near(near_open.STATE.plannedVelocity, 20.0f, 0.001f);
    assert_near(near_closed.STATE.plannedVelocity, 20.0f, 0.001f);
    assert_near(near_open.STATE.plannedFlow, expected_open_flow, 0.002f);
    assert_near(near_closed.STATE.plannedFlow, expected_closed_flow, 0.002f);
    assert(fabsf(near_open.STATE.plannedFlow -
                 near_closed.STATE.plannedFlow) > 0.1f);
#if HYD_ENABLE_MECHANISM_TELEMETRY
    assert_near(near_open.STATE.actuatorPosition, open_solution.xs, 0.002f);
    assert_near(near_open.STATE.actuatorVelocityCommand,
                open_solution.vs, 0.002f);
    assert_near(near_open.STATE.velocityRatio,
                open_solution.velocityRatio, 0.0002f);
#endif
    assert(near_open.STATE.actuatorDirection == HYD_DIRECTION_RETRACT);
    assert(near_open.STATE.mechanismConfigVersion == 1u);
}

static HYD_MotionSegment make_pressure_segment(void)
{
    HYD_MotionSegment segment;

    memset(&segment, 0, sizeof(segment));
    segment.segmentType = HYD_SEGMENT_TYPE_HOLDING;
    segment.planner = HYD_PLANNER_TIME_BASED;
    segment.mode = HYD_MODE_PRESSURE_CLOSED_LOOP;
    segment.endCondition = HYD_END_MANUAL;
    segment.direction = HYD_DIRECTION_HOLD;
    segment.targetPressure = 20.0f;
    segment.targetFlow = 5.0f;
    segment.maxFlow = 20.0f;
    segment.pressureRampRate = 100.0f;
    segment.pressureController = HYD_PRESSURE_CONTROLLER_P;
    segment.pressureKp = 0.5f;
    segment.pressureDeadband = 0.01f;
    segment.pressureFilterAlpha = 1.0f;
    return segment;
}

static void test_pressure_flow_bypasses_toggle_velocity_mapping(void)
{
    HYD_TogglePreparedConfig prepared = prepare_default_toggle();
    HYD_MotionSegment segment = make_pressure_segment();
    HYD_MotionControlFB direct;
    HYD_MotionControlFB toggle;

    run_first_control_cycle(&direct, &segment, 101.0f);
    run_first_control_cycle(&toggle, &segment, 101.0f);
    bind_toggle(&toggle, &prepared);
    toggle.AXIS_REF.timestamp = 0.2f;
    HYD_MotionControlFB_Scan(&toggle);

    assert_near(toggle.STATE.plannedFlow, direct.STATE.plannedFlow, 0.0001f);
    assert_near(toggle.STATE.plannedFlow, 5.0f, 0.0001f);
}

static void start_bound_toggle_speed(
    HYD_MotionControlFB *fb,
    const HYD_TogglePreparedConfig *prepared,
    HYD_REAL position)
{
    HYD_MotionSegment segment = make_speed_segment();

    HYD_MotionControlFB_Init(fb);
    fb->USE_RECIPE = false;
    fb->FLOW_TO_PUMP_SPEED_GAIN = 10.0f;
    fb->PUMP_SPEED_LIMIT = 3000.0f;
    fb->AXIS_REF.position = position;
    fb->AXIS_REF.pressure = 20.0f;
    bind_toggle(fb, prepared);
    assert(HYD_MotionControlFB_LoadDirectSegment(fb, &segment));
    assert(HYD_MotionControlFB_StartSegment(fb, 0u, 0.0f));
    HYD_MotionControlFB_Scan(fb);
    fb->AXIS_REF.timestamp = 0.1f;
    HYD_MotionControlFB_Scan(fb);
}

static void assert_toggle_stop_flow_at_position(HYD_REAL position)
{
    HYD_TogglePreparedConfig prepared = prepare_default_toggle();
    HYD_MotionControlFB fb;
    HYD_ToggleSolution solution;
    HYD_ToggleError error = HYD_TOGGLE_ERROR_NONE;

    start_bound_toggle_speed(&fb, &prepared, position);
    fb.AXIS_REF.velocity = fb.STATE.plannedVelocity;
    assert(HYD_MotionControlFB_Stop(&fb, 0.1f, 20.0f));
    fb.AXIS_REF.timestamp = 0.2f;
    HYD_MotionControlFB_Scan(&fb);

    assert(HYD_ToggleKinematics_SolveOnline(
        &prepared, position, fb.STATE.plannedVelocity, &solution, &error));
    assert_near(fb.STATE.plannedFlow, fabsf(solution.vs) * 2.0f, 0.003f);
    assert_near(fb.PUMP_SPEED, fb.STATE.plannedFlow * 10.0f, 0.003f);
}

static void test_stop_deceleration_uses_dynamic_gain_at_each_position(void)
{
    assert_toggle_stop_flow_at_position(101.0f);
    assert_toggle_stop_flow_at_position(180.0f);
}

static void test_runtime_mapping_failures_zero_outputs_in_same_cycle(void)
{
    HYD_TogglePreparedConfig prepared = prepare_default_toggle();
    HYD_MotionControlFB fb;

    start_bound_toggle_speed(&fb, &prepared, 101.0f);
    assert(fb.PUMP_SPEED > 0.0f);

    fb.AXIS_REF.position = prepared.raw.sm + 1.0f;
    fb.AXIS_REF.timestamp = 0.2f;
    HYD_MotionControlFB_Scan(&fb);
    assert(fb.FB_STATE == HYD_FB_STATE_FAULT);
    assert(fb.DIAGNOSTIC.code ==
           HYD_DIAG_CODE_KINEMATICS_POSITION_OUT_OF_RANGE);
    assert_near(fb.STATE.plannedFlow, 0.0f, 0.0f);
    assert_near(fb.PUMP_SPEED, 0.0f, 0.0f);
    assert_near(fb._lastCommandedFlow, 0.0f, 0.0f);

    HYD_MotionControlFB_Init(&fb);
    start_bound_toggle_speed(&fb, &prepared, 101.0f);
    HYD_ToggleMechanismPool_Release(fb.mechanismSlot);
    fb.AXIS_REF.timestamp = 0.2f;
    HYD_MotionControlFB_Scan(&fb);
    assert(fb.FB_STATE == HYD_FB_STATE_FAULT);
    assert(fb.DIAGNOSTIC.code == HYD_DIAG_CODE_KINEMATICS_RUNTIME_INVALID);
    assert_near(fb.STATE.plannedFlow, 0.0f, 0.0f);
    assert_near(fb.PUMP_SPEED, 0.0f, 0.0f);
    assert_near(fb._lastCommandedFlow, 0.0f, 0.0f);
}

static void test_soft_reset_preserves_toggle_binding(void)
{
    HYD_TogglePreparedConfig prepared = prepare_default_toggle();
    HYD_MotionControlFB fb;
    HYD_UINT8 slot;

    HYD_MotionControlFB_Init(&fb);
    bind_toggle(&fb, &prepared);
    slot = fb.mechanismSlot;
    HYD_MotionControlFB_SoftReset(&fb);

    assert(fb.mechanismType ==
           (HYD_UINT8)HYD_MECHANISM_FIVE_POINT_TOGGLE);
    assert(fb.mechanismSlot == slot);
    assert(fb.STATE.mechanismType == fb.mechanismType);
    assert(fb.STATE.mechanismConfigVersion == 1u);
    assert(HYD_ToggleMechanismPool_GetPrepared(slot) != NULL);
}

static void test_hold_resume_preserves_template_position(void)
{
    HYD_TogglePreparedConfig prepared = prepare_default_toggle();
    HYD_MotionControlFB fb;
    HYD_REAL heldPosition;

    start_bound_toggle_speed(&fb, &prepared, 101.0f);
    heldPosition = fb.AXIS_REF.position;
    assert(HYD_MotionControlFB_Hold(&fb));
    fb.AXIS_REF.timestamp = 0.2f;
    HYD_MotionControlFB_Scan(&fb);
    assert(fb.FB_STATE == HYD_FB_STATE_HOLD);
    assert_near(fb.AXIS_REF.position, heldPosition, 0.0f);
    assert_near(fb.PUMP_SPEED, 0.0f, 0.0f);
    assert(fb.STATE.actuatorDirection == HYD_DIRECTION_HOLD);
#if HYD_ENABLE_MECHANISM_TELEMETRY
    assert_near(fb.STATE.actuatorVelocityCommand, 0.0f, 0.0f);
#endif

    assert(HYD_MotionControlFB_Resume(&fb));
    fb.AXIS_REF.timestamp = 0.3f;
    HYD_MotionControlFB_Scan(&fb);
    fb.AXIS_REF.timestamp = 0.4f;
    HYD_MotionControlFB_Scan(&fb);
    assert_near(fb.AXIS_REF.position, heldPosition, 0.0f);
    assert(fb.STATE.mechanismType ==
           (HYD_UINT8)HYD_MECHANISM_FIVE_POINT_TOGGLE);
    assert(fabsf(fb.STATE.plannedVelocity) <= 20.0f);
}

static void finish_pressure_segment(HYD_MotionControlFB *fb,
                                    const HYD_TogglePreparedConfig *prepared,
                                    HYD_REAL position)
{
    HYD_MotionSegment segment = make_pressure_segment();
    unsigned int cycle;

    segment.endCondition = HYD_END_TIME;
    segment.duration = 0.02f;
    HYD_MotionControlFB_Init(fb);
    fb->USE_RECIPE = false;
    fb->FLOW_TO_PUMP_SPEED_GAIN = 10.0f;
    fb->PUMP_SPEED_LIMIT = 3000.0f;
    fb->AXIS_REF.position = position;
    fb->AXIS_REF.pressure = 20.0f;
    bind_toggle(fb, prepared);
    assert(HYD_MotionControlFB_LoadDirectSegment(fb, &segment));
    assert(HYD_MotionControlFB_StartSegment(fb, 0U, 0.0f));
    HYD_MotionControlFB_Scan(fb);
    for (cycle = 1U; cycle <= 10U && !HYD_MotionControlFB_IsDone(fb);
         ++cycle) {
        fb->AXIS_REF.timestamp = (HYD_REAL)cycle * 0.01f;
        HYD_MotionControlFB_Scan(fb);
    }
    assert(HYD_MotionControlFB_IsDone(fb));
    assert(fb->_lastCommandedFlow > 0.0f);
}

static void test_pressure_to_velocity_uses_dynamic_inverse_and_faults_safely(void)
{
    HYD_TogglePreparedConfig prepared = prepare_default_toggle();
    HYD_MotionSegment speed = make_speed_segment();
    HYD_MotionControlFB fb;
    HYD_ActuationMapperInput input;
    HYD_ToggleError error = HYD_TOGGLE_ERROR_NONE;
    HYD_REAL expectedVelocity = 0.0f;
    HYD_REAL carriedFlow;

    finish_pressure_segment(&fb, &prepared, 101.0f);
    carriedFlow = fb._lastCommandedFlow;
    memset(&input, 0, sizeof(input));
    input.mechanismType = HYD_MECHANISM_FIVE_POINT_TOGGLE;
    input.toggleConfig = &prepared;
    input.templatePosition = 101.0f;
    input.cylinderConfig = &fb.cylinderConfig;
    input.fallbackCylinderVelocityToFlowGain = speed.velocityToFlowGain;
    assert(HYD_ActuationMapper_FlowToTemplateVelocity(
        &input, carriedFlow, &expectedVelocity, &error));

    assert(HYD_MotionControlFB_LoadDirectSegment(&fb, &speed));
    assert(HYD_MotionControlFB_StartSegment(
        &fb, 0U, fb.AXIS_REF.timestamp));
    HYD_MotionControlFB_Scan(&fb);
    assert(fb.FB_STATE == HYD_FB_STATE_RUNNING);
    assert(fb._plannerState.initialized);
    assert_near(fb._plannerState.lastTargetVelocity,
                expectedVelocity, 0.002f);
    assert_near(fb._plannerState.lastTargetFlow, carriedFlow, 0.002f);

    finish_pressure_segment(&fb, &prepared, 101.0f);
    fb.AXIS_REF.position = prepared.raw.sm + 1.0f;
    assert(HYD_MotionControlFB_StartDirectCommand(
               &fb, HYD_DIRECT_CMD_MOVE_VELOCITY, &speed, NULL,
               HYD_BUFFER_MODE_ABORT, fb.AXIS_REF.timestamp) ==
           HYD_DIRECT_START_REJECTED);
    assert(fb.FB_STATE == HYD_FB_STATE_FAULT);
    assert(fb.DIAGNOSTIC.code ==
           HYD_DIAG_CODE_KINEMATICS_POSITION_OUT_OF_RANGE);
    assert_near(fb.PUMP_SPEED, 0.0f, 0.0f);
    assert_near(fb._lastCommandedFlow, 0.0f, 0.0f);
}

static HYD_MotionSegment make_position_command(HYD_REAL targetPosition)
{
    HYD_MotionSegment segment = make_position_segment();

    segment.targetPosition = targetPosition;
    segment.maxVelocity = 20.0f;
    segment.maxAcceleration = 100.0f;
    segment.maxDeceleration = 100.0f;
    segment.maxFlow = 100.0f;
    segment.velocityToFlowGain = 2.0f;
    return segment;
}

static void advance_template_plant(HYD_MotionControlFB *fb, HYD_REAL dt)
{
    fb->AXIS_REF.position += fb->STATE.plannedVelocity * dt;
    fb->AXIS_REF.velocity = fb->STATE.plannedVelocity;
    fb->AXIS_REF.flow = fb->STATE.plannedFlow;
    fb->AXIS_REF.timestamp += dt;
    HYD_MotionControlFB_Cycle(fb);
}

static void initialize_bound_toggle_core(
    HYD_MotionControlFB *fb,
    const HYD_TogglePreparedConfig *prepared)
{
    HYD_MotionControlFB_Init(fb);
    fb->FLOW_TO_PUMP_SPEED_GAIN = 10.0f;
    fb->PUMP_SPEED_LIMIT = 3000.0f;
    fb->AXIS_REF.position = 101.0f;
    fb->AXIS_REF.pressure = 20.0f;
    bind_toggle(fb, prepared);
}

static void test_continuous_absolute_transition_stays_in_template_space(void)
{
    HYD_TogglePreparedConfig prepared = prepare_default_toggle();
    HYD_MotionControlFB fb;
    HYD_MotionSegment approach = make_position_command(125.0f);
    HYD_ContinuousAbsoluteContext context;
    unsigned int cycle;

    initialize_bound_toggle_core(&fb, &prepared);
    memset(&context, 0, sizeof(context));
    context.valid = true;
    context.phase = HYD_CONTABS_PHASE_APPROACH;
    context.targetPosition = approach.targetPosition;
    context.crossingVelocity = 10.0f;
    context.sustainVelocity = 10.0f;
    context.approachDirection = HYD_DIRECTION_EXTEND;
    context.sustainDirection = HYD_DIRECTION_EXTEND;
    assert(HYD_MotionControlFB_StartDirectCommand(
               &fb, HYD_DIRECT_CMD_MOVE_CONTINUOUS_ABSOLUTE,
               &approach, &context, HYD_BUFFER_MODE_ABORT, 0.0f) ==
           HYD_DIRECT_START_STARTED);

    for (cycle = 0U; cycle < 1000U &&
         fb._directContinuousAbsolute.phase != HYD_CONTABS_PHASE_SUSTAIN;
         ++cycle) {
        advance_template_plant(&fb, 0.01f);
        assert(fb.FB_STATE != HYD_FB_STATE_FAULT);
        assert_near(fb._simFeedback.targetVelocity,
                    fb.STATE.plannedVelocity, 0.0f);
    }
    assert(fb._directContinuousAbsolute.phase == HYD_CONTABS_PHASE_SUSTAIN);
    assert(fb.STATE.mechanismType ==
           (HYD_UINT8)HYD_MECHANISM_FIVE_POINT_TOGGLE);
#if HYD_ENABLE_MECHANISM_TELEMETRY
    assert(fabsf(fb.STATE.actuatorVelocityCommand -
                 fb.STATE.plannedVelocity) > 0.1f);
#endif
}

static void test_buffered_blend_recomputes_toggle_flow_after_cutover(void)
{
    HYD_TogglePreparedConfig prepared = prepare_default_toggle();
    HYD_MotionControlFB fb;
    HYD_MotionSegment first = make_position_command(130.0f);
    HYD_MotionSegment second = make_position_command(180.0f);
    HYD_ActuationMapperOutput expected;
    HYD_DiagnosticCode code = HYD_DIAG_CODE_NONE;
    unsigned int cycle;

    initialize_bound_toggle_core(&fb, &prepared);
    assert(HYD_MotionControlFB_StartDirectCommand(
               &fb, HYD_DIRECT_CMD_MOVE_ABSOLUTE, &first, NULL,
               HYD_BUFFER_MODE_ABORT, 0.0f) == HYD_DIRECT_START_STARTED);
    advance_template_plant(&fb, 0.01f);
    assert(HYD_MotionControlFB_StartDirectCommand(
               &fb, HYD_DIRECT_CMD_MOVE_ABSOLUTE, &second, NULL,
               HYD_BUFFER_MODE_BLENDING_NEXT, fb.AXIS_REF.timestamp) ==
           HYD_DIRECT_START_QUEUED);

    for (cycle = 0U; cycle < 1000U && fb._directPendingValid; ++cycle) {
        advance_template_plant(&fb, 0.01f);
        assert(fb.FB_STATE != HYD_FB_STATE_FAULT);
    }
    assert(!fb._directPendingValid);
    assert_near(fb._activeSegment.targetPosition, 180.0f, 0.0f);
    advance_template_plant(&fb, 0.01f);
    assert(HYD_MotionControlFB_MapTemplateVelocity(
        &fb, &fb._activeSegment, fb.STATE.plannedVelocity,
        fb._activeSegment.maxFlow, &expected, &code));
    assert_near(fb.STATE.plannedFlow, expected.requestedFlow, 0.003f);
}

int main(void)
{
    test_direct_position_and_velocity_flow_are_unchanged();
    test_toggle_mapping_varies_with_position_in_actuator_space();
    test_pressure_flow_bypasses_toggle_velocity_mapping();
    test_stop_deceleration_uses_dynamic_gain_at_each_position();
    test_runtime_mapping_failures_zero_outputs_in_same_cycle();
    test_soft_reset_preserves_toggle_binding();
    test_hold_resume_preserves_template_position();
    test_pressure_to_velocity_uses_dynamic_inverse_and_faults_safely();
    test_continuous_absolute_transition_stays_in_template_space();
    test_buffered_blend_recomputes_toggle_flow_after_cutover();
    printf("toggle motion integration tests passed\n");
    return 0;
}
