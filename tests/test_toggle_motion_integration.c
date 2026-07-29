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

int main(void)
{
    test_direct_position_and_velocity_flow_are_unchanged();
    test_toggle_mapping_varies_with_position_in_actuator_space();
    test_pressure_flow_bypasses_toggle_velocity_mapping();
    printf("toggle motion integration tests passed\n");
    return 0;
}
