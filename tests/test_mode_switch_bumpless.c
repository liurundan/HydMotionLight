#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "motion_control.h"
#include "segment_limits.h"

static HYD_MotionSegment make_position_segment(void) {
    HYD_MotionSegment segment;
    memset(&segment, 0, sizeof(segment));
    segment.segmentType = HYD_SEGMENT_TYPE_OTHER;
    segment.mode = HYD_MODE_POSITION;
    segment.endCondition = HYD_END_POSITION;
    segment.planner = HYD_PLANNER_POSITION_BASED;
    segment.direction = HYD_DIRECTION_EXTEND;
    segment.targetPosition = 100.0;
    segment.maxVelocity = 20.0;
    segment.maxAcceleration = 100.0;
    segment.maxDeceleration = 100.0;
    segment.maxFlow = 50.0;
    segment.velocityToFlowGain = 1.0;
    segment.positionTolerance = 0.1;
    return segment;
}

static HYD_MotionSegment make_speed_segment(void) {
    HYD_MotionSegment segment;
    memset(&segment, 0, sizeof(segment));
    segment.segmentType = HYD_SEGMENT_TYPE_OTHER;
    segment.mode = HYD_MODE_SPEED_RAMP;
    segment.endCondition = HYD_END_MANUAL;
    segment.planner = HYD_PLANNER_TIME_BASED;
    segment.direction = HYD_DIRECTION_EXTEND;
    segment.maxVelocity = 20.0;
    segment.maxAcceleration = 100.0;
    segment.maxDeceleration = 100.0;
    segment.maxFlow = 50.0;
    segment.velocityToFlowGain = 1.0;
    return segment;
}

static HYD_MotionSegment make_pressure_segment(void) {
    HYD_MotionSegment segment;
    memset(&segment, 0, sizeof(segment));
    segment.segmentType = HYD_SEGMENT_TYPE_HOLDING;
    segment.mode = HYD_MODE_PRESSURE_CLOSED_LOOP;
    segment.endCondition = HYD_END_MANUAL;
    segment.direction = HYD_DIRECTION_HOLD;
    segment.targetPressure = 10.0;
    segment.targetFlow = 2.0;
    segment.maxFlow = 50.0;
    segment.pressureController = HYD_PRESSURE_CONTROLLER_P;
    segment.pressureKp = 1.0;
    return segment;
}

static void seed_active_segment(HYD_MotionControlFB* fb,
                                HYD_MotionSegment segment,
                                HYD_DirectCommandKind ownerKind) {
    HYD_MotionControlFB_Init(fb);
    fb->USE_RECIPE = false;
    fb->FLOW_TO_PUMP_SPEED_GAIN = 100.0;
    fb->PUMP_SPEED_LIMIT = 3000.0;
    fb->AXIS_REF.position = 20.0;
    fb->AXIS_REF.velocity = 5.0;
    fb->AXIS_REF.flow = 5.0;
    fb->AXIS_REF.pressure = 10.0;
    fb->AXIS_REF.timestamp = 1.0;
    fb->_activeSegment = segment;
    fb->_activeSegmentValid = true;
    fb->_activeSegmentSource = HYD_SEGMENT_SOURCE_DIRECT;
    fb->STATE.active = true;
    fb->FB_STATE = HYD_FB_STATE_RUNNING;
    fb->_directOwnerKind = ownerKind;
    fb->_directOwnerTicket = 1U;
    fb->_executionId = 1U;
    fb->PUMP_SPEED = 500.0;
    fb->STATE.commandedPumpSpeed = 500.0;
    fb->STATE.plannedFlow = 5.0;
    fb->STATE.plannedVelocity = 5.0;
    fb->_lastCommandedFlow = 5.0;
    fb->_plannerState.initialized = true;
    fb->_plannerState.lastTargetVelocity = 5.0;
    fb->_plannerState.lastTargetFlow = 5.0;
}

static void assert_handover(HYD_ControlMode sourceMode,
                            HYD_ControlMode successorMode,
                            HYD_DirectCommandKind sourceKind,
                            HYD_DirectCommandKind successorKind,
                            HYD_MotionSegment successor) {
    HYD_MotionControlFB fb;
    HYD_DirectStartResult result;

    if (sourceMode == HYD_MODE_POSITION) {
        seed_active_segment(&fb, make_position_segment(), sourceKind);
    } else if (sourceMode == HYD_MODE_SPEED_RAMP) {
        seed_active_segment(&fb, make_speed_segment(), sourceKind);
    } else {
        seed_active_segment(&fb, make_pressure_segment(), sourceKind);
    }

    result = HYD_MotionControlFB_StartDirectCommand(
        &fb, successorKind, &successor, NULL, HYD_BUFFER_MODE_ABORT, 1.001);
    assert(result == HYD_DIRECT_START_STARTED);
    assert(fb._activeSegment.mode == successorMode);
    assert(fb.PUMP_SPEED > 0.0);
    assert(fabs(fb.PUMP_SPEED - 500.0) < 0.001);
    assert(fb._lastCommandedFlow > 0.0);

    fb.AXIS_REF.timestamp = 1.002;
    HYD_MotionControlFB_Cycle(&fb);
    assert(fb.PUMP_SPEED > 0.0);
    assert(fb.PUMP_SPEED <= 520.001);
}

static void test_position_pressure_handover(void) {
    assert_handover(HYD_MODE_POSITION,
                    HYD_MODE_PRESSURE_CLOSED_LOOP,
                    HYD_DIRECT_CMD_MOVE_ABSOLUTE,
                    HYD_DIRECT_CMD_PRESSURE_HANDLE,
                    make_pressure_segment());
}

static void test_speed_pressure_handover(void) {
    assert_handover(HYD_MODE_SPEED_RAMP,
                    HYD_MODE_PRESSURE_CLOSED_LOOP,
                    HYD_DIRECT_CMD_MOVE_VELOCITY,
                    HYD_DIRECT_CMD_PRESSURE_HANDLE,
                    make_pressure_segment());
}

static void test_pressure_position_handover(void) {
    assert_handover(HYD_MODE_PRESSURE_CLOSED_LOOP,
                    HYD_MODE_POSITION,
                    HYD_DIRECT_CMD_PRESSURE_HANDLE,
                    HYD_DIRECT_CMD_MOVE_ABSOLUTE,
                    make_position_segment());
}

static void test_pressure_speed_handover(void) {
    assert_handover(HYD_MODE_PRESSURE_CLOSED_LOOP,
                    HYD_MODE_SPEED_RAMP,
                    HYD_DIRECT_CMD_PRESSURE_HANDLE,
                    HYD_DIRECT_CMD_MOVE_VELOCITY,
                    make_speed_segment());
}

static void assert_motion_handover(HYD_MotionSegment source,
                                   HYD_DirectCommandKind sourceKind,
                                   HYD_MotionSegment successor,
                                   HYD_DirectCommandKind successorKind) {
    HYD_MotionControlFB fb;

    seed_active_segment(&fb, source, sourceKind);
    fb.AXIS_REF.velocity = 1.0;
    fb._plannerState.transitionTargetVelocity = 10.0;
    fb._plannerState.transitionRate = 50.0;
    fb._plannerState.transitionTimeMs = 100.0;

    assert(HYD_MotionControlFB_StartDirectCommand(
        &fb, successorKind, &successor, NULL, HYD_BUFFER_MODE_ABORT, 1.001) ==
        HYD_DIRECT_START_STARTED);
    assert(fabs(fb._plannerState.lastTargetVelocity - 5.0) < 0.001);
    assert(fabs(fb._plannerState.transitionTargetVelocity - 10.0) < 0.001);
    assert(fabs(fb._plannerState.transitionRate - 50.0) < 0.001);

    fb.AXIS_REF.timestamp = 1.002;
    HYD_MotionControlFB_Cycle(&fb);
    assert(fabs(fb.STATE.plannedVelocity - 5.05) < 0.02);
}

static void test_position_to_position_motion_handover(void) {
    HYD_MotionSegment successor = make_position_segment();
    successor.targetPosition = 200.0;
    successor.maxVelocity = 10.0;
    assert_motion_handover(make_position_segment(),
                           HYD_DIRECT_CMD_MOVE_ABSOLUTE,
                           successor,
                           HYD_DIRECT_CMD_MOVE_ABSOLUTE);
}

static void test_speed_to_speed_motion_handover(void) {
    HYD_MotionSegment successor = make_speed_segment();
    successor.maxVelocity = 10.0;
    assert_motion_handover(make_speed_segment(),
                           HYD_DIRECT_CMD_MOVE_VELOCITY,
                           successor,
                           HYD_DIRECT_CMD_MOVE_VELOCITY);
}

static void test_position_to_speed_motion_handover(void) {
    HYD_MotionSegment successor = make_speed_segment();
    successor.maxVelocity = 10.0;
    assert_motion_handover(make_position_segment(),
                           HYD_DIRECT_CMD_MOVE_ABSOLUTE,
                           successor,
                           HYD_DIRECT_CMD_MOVE_VELOCITY);
}

static void test_speed_to_position_motion_handover(void) {
    HYD_MotionSegment successor = make_position_segment();
    successor.targetPosition = 200.0;
    successor.maxVelocity = 10.0;
    assert_motion_handover(make_speed_segment(),
                           HYD_DIRECT_CMD_MOVE_VELOCITY,
                           successor,
                           HYD_DIRECT_CMD_MOVE_ABSOLUTE);
}

static void test_motion_handover_does_not_add_second_pump_slew(void) {
    HYD_MotionControlFB fb;
    HYD_MotionSegment source = make_speed_segment();
    HYD_MotionSegment successor = make_position_segment();

    source.maxVelocity = 100.0;
    successor.maxVelocity = 10.0;
    successor.targetPosition = 200.0;

    seed_active_segment(&fb, source, HYD_DIRECT_CMD_MOVE_VELOCITY);
    fb.FLOW_TO_PUMP_SPEED_GAIN = 10.0;
    fb.PUMP_SPEED_LIMIT = 3000.0;
    fb.PUMP_SPEED = 1000.0;
    fb.STATE.commandedPumpSpeed = 1000.0;
    fb.STATE.plannedFlow = 100.0;
    fb._lastCommandedFlow = 100.0;
    fb._plannerState.lastTargetVelocity = 100.0;
    fb._plannerState.lastTargetFlow = 100.0;

    assert(HYD_MotionControlFB_StartDirectCommand(
        &fb, HYD_DIRECT_CMD_MOVE_ABSOLUTE, &successor, NULL,
        HYD_BUFFER_MODE_ABORT, 1.001) == HYD_DIRECT_START_STARTED);

    for (int cycle = 1; cycle <= 10; ++cycle) {
        fb.AXIS_REF.timestamp = 1.001 + cycle * 0.001;
        HYD_MotionControlFB_Cycle(&fb);
    }

    /* The planner output should continue from the carried command; the final
     * pump output must follow it directly instead of being held near the old
     * pump speed by a second slew. */
    assert(fabs(fb.STATE.plannedVelocity - 91.0) < 0.02);
    assert(fabs(fb.PUMP_SPEED - fb.STATE.plannedFlow * 10.0) < 0.01);
    assert(fb.PUMP_SPEED < 950.0);
}

static void test_pressure_handover_does_not_add_second_pump_slew(void) {
    HYD_MotionControlFB fb;
    HYD_MotionSegment source = make_position_segment();
    HYD_MotionSegment successor = make_pressure_segment();

    source.maxVelocity = 50.0;
    seed_active_segment(&fb, source, HYD_DIRECT_CMD_MOVE_ABSOLUTE);
    fb.FLOW_TO_PUMP_SPEED_GAIN = 10.0;
    fb.PUMP_SPEED_LIMIT = 3000.0;
    fb.PUMP_SPEED = 1000.0;
    fb.STATE.commandedPumpSpeed = 1000.0;
    fb.STATE.plannedFlow = 50.0;
    fb._lastCommandedFlow = 50.0;
    fb._plannerState.lastTargetVelocity = 50.0;
    fb._plannerState.lastTargetFlow = 50.0;
    successor.targetPressure = 20.0;
    successor.targetFlow = 2.0;

    assert(HYD_MotionControlFB_StartDirectCommand(
        &fb, HYD_DIRECT_CMD_PRESSURE_HANDLE, &successor, NULL,
        HYD_BUFFER_MODE_ABORT, 1.001) == HYD_DIRECT_START_STARTED);

    fb.AXIS_REF.timestamp = 1.002;
    HYD_MotionControlFB_Cycle(&fb);

    /* Pressure control owns its requested output; a motion handover slew must
     * not add a second, unrelated rpm ramp on top of it. */
    assert(fabs(fb.PUMP_SPEED - fb.STATE.commandedPumpSpeed) < 0.01);
    assert(fabs(fb.PUMP_SPEED - 500.0) < 0.01);
}

/* spec 2026-11-26 §4.2 / T3：限压中换段（限压 100 → 140，P = 124）不得一拍恢复到新段的
 * 速度流量。复现 vofa 20261125 8879 ms：217 rpm → 868 rpm。 */
static void test_pressure_limited_handover_continues_from_executed_flow(void) {
    HYD_MotionControlFB fb;
    HYD_MotionSegment source = make_speed_segment();
    HYD_MotionSegment successor = make_speed_segment();

    source.maxVelocity = 43.4;
    source.maxPressure = 100.0;
    successor.maxVelocity = 43.4;
    successor.maxPressure = 140.0;
    seed_active_segment(&fb, source, HYD_DIRECT_CMD_MOVE_VELOCITY);
    fb.FLOW_TO_PUMP_SPEED_GAIN = 20.0;
    fb.PUMP_SPEED_LIMIT = 1700.0;
    fb.AXIS_REF.pressure = 124.0;
    fb.PUMP_SPEED = 217.0;
    fb.STATE.commandedPumpSpeed = 217.0;
    fb._lastCommandedFlow = 10.85;               /* 限压后实际执行 217 rpm */
    fb._plannerState.lastTargetVelocity = 43.4;  /* 规划器仍按 868 rpm 走 */
    fb._plannerState.lastTargetFlow = 43.4;
    fb.STATE.plannedFlow = 43.4;
    fb.STATE.plannedVelocity = 43.4;

    assert(HYD_MotionControlFB_StartDirectCommand(
        &fb, HYD_DIRECT_CMD_MOVE_VELOCITY, &successor, NULL,
        HYD_BUFFER_MODE_ABORT, 1.001) == HYD_DIRECT_START_STARTED);

    fb.AXIS_REF.timestamp = 1.002;
    HYD_MotionControlFB_Cycle(&fb);

    /* Q = 10.85 + Kp·(140 − 124) = 26.85 L/min → 537 rpm（默认 Kp 1.0） */
    assert(fabs(fb.PUMP_SPEED - 537.0) < 1.0);
    assert((fb.STATE.limitFlags & HYD_LIMIT_FLAG_PRESSURE) != 0u);
}

/* spec 2026-11-26 §4.5 / T5：限压中转保压，保压 PI 初值取实际执行流量（7.7 L/min），
 * 不取限压前的规划流量（52 L/min）。 */
static void test_pressure_handover_seeds_from_executed_flow(void) {
    HYD_MotionControlFB fb;
    HYD_MotionSegment source = make_speed_segment();
    HYD_MotionSegment successor = make_pressure_segment();

    source.maxVelocity = 52.0;
    source.maxPressure = 140.0;
    seed_active_segment(&fb, source, HYD_DIRECT_CMD_MOVE_VELOCITY);
    fb.FLOW_TO_PUMP_SPEED_GAIN = 20.0;
    fb.PUMP_SPEED_LIMIT = 1700.0;
    fb.AXIS_REF.pressure = 140.0;
    fb.AXIS_REF.flow = 0.0;
    fb.PUMP_SPEED = 154.0;
    fb.STATE.commandedPumpSpeed = 154.0;
    fb._lastCommandedFlow = 7.7;
    fb._plannerState.lastTargetVelocity = 52.0;
    fb._plannerState.lastTargetFlow = 52.0;
    fb.STATE.plannedFlow = 52.0;
    successor.targetPressure = 100.0;
    successor.maxFlow = 85.0;

    assert(HYD_MotionControlFB_StartDirectCommand(
        &fb, HYD_DIRECT_CMD_PRESSURE_HANDLE, &successor, NULL,
        HYD_BUFFER_MODE_ABORT, 1.001) == HYD_DIRECT_START_STARTED);
    assert(fabs(fb._pressureController.previousOutput - 7.7) < 1e-3);
}

/* spec 2026-11-26 §4.2 / T3c：接管前限压已把实际执行流量压到 0，
 * 接管后首拍从 0 续接，不退回规划流量（868 rpm）。 */
static void seed_limited_to_zero(HYD_MotionControlFB* fb,
                                 HYD_MotionSegment source,
                                 HYD_REAL pressure) {
    seed_active_segment(fb, source, HYD_DIRECT_CMD_MOVE_VELOCITY);
    fb->FLOW_TO_PUMP_SPEED_GAIN = 20.0;
    fb->PUMP_SPEED_LIMIT = 1700.0;
    fb->AXIS_REF.pressure = pressure;
    fb->PUMP_SPEED = 0.0;
    fb->STATE.commandedPumpSpeed = 0.0;
    fb->_lastCommandedFlow = 0.0;                 /* 限压 PI 已把实际流量压到 0 */
    fb->_limiterState.pressureLimitActive = true;
    fb->_plannerState.lastTargetVelocity = 43.4;  /* 规划器仍按 868 rpm 走 */
    fb->_plannerState.lastTargetFlow = 43.4;
    fb->STATE.plannedFlow = 43.4;
    fb->STATE.plannedVelocity = 43.4;
}

static void test_takeover_while_limited_to_zero_keeps_zero_flow(void) {
    HYD_MotionControlFB fb;
    HYD_MotionSegment source = make_speed_segment();
    HYD_MotionSegment successor = make_speed_segment();

    source.maxVelocity = 43.4;
    source.maxPressure = 100.0;
    successor.maxVelocity = 43.4;
    successor.maxPressure = 140.0;

    /* P = 150 高于新 Plim 140 */
    seed_limited_to_zero(&fb, source, 150.0);
    assert(HYD_MotionControlFB_StartDirectCommand(
        &fb, HYD_DIRECT_CMD_MOVE_VELOCITY, &successor, NULL,
        HYD_BUFFER_MODE_ABORT, 1.001) == HYD_DIRECT_START_STARTED);
    assert(fb.PUMP_SPEED < 1.0);

    fb.AXIS_REF.timestamp = 1.002;
    HYD_MotionControlFB_Cycle(&fb);

    /* Q = 0 + Kp·(140 − 150) → 限到 0 */
    assert(fb.PUMP_SPEED < 1.0);
    assert((fb.STATE.limitFlags & HYD_LIMIT_FLAG_PRESSURE) != 0u);

    /* P = 124 低于新 Plim 140 */
    seed_limited_to_zero(&fb, source, 124.0);
    assert(HYD_MotionControlFB_StartDirectCommand(
        &fb, HYD_DIRECT_CMD_MOVE_VELOCITY, &successor, NULL,
        HYD_BUFFER_MODE_ABORT, 1.001) == HYD_DIRECT_START_STARTED);

    fb.AXIS_REF.timestamp = 1.002;
    HYD_MotionControlFB_Cycle(&fb);

    /* Q = 0 + Kp·(140 − 124) = 16 L/min → 320 rpm（默认 Kp 1.0） */
    assert(fabs(fb.PUMP_SPEED - 320.0) < 1.0);
    assert((fb.STATE.limitFlags & HYD_LIMIT_FLAG_PRESSURE) != 0u);
}

/* spec 2026-11-26 §4.2 注 / T3b：限压稳态 P = 旧 Plim 100，实际流量约为泄漏量，
 * 换段把 Plim 上调到 140。 */
static void test_raised_limit_with_margin_releases_toward_velocity_flow(void) {
    HYD_MotionControlFB fb;
    HYD_MotionSegment source = make_speed_segment();
    HYD_MotionSegment successor = make_speed_segment();

    source.maxVelocity = 43.4;
    source.maxPressure = 100.0;
    successor.maxVelocity = 43.4;
    successor.maxPressure = 140.0;
    seed_active_segment(&fb, source, HYD_DIRECT_CMD_MOVE_VELOCITY);
    fb.FLOW_TO_PUMP_SPEED_GAIN = 20.0;
    fb.PUMP_SPEED_LIMIT = 1700.0;
    fb.AXIS_REF.pressure = 100.0;
    fb.PUMP_SPEED = 20.0;
    fb.STATE.commandedPumpSpeed = 20.0;
    fb._lastCommandedFlow = 1.0;                 /* 约等于泄漏量 */
    fb._plannerState.lastTargetVelocity = 43.4;
    fb._plannerState.lastTargetFlow = 43.4;
    fb.STATE.plannedFlow = 43.4;
    fb.STATE.plannedVelocity = 43.4;

    assert(HYD_MotionControlFB_StartDirectCommand(
        &fb, HYD_DIRECT_CMD_MOVE_VELOCITY, &successor, NULL,
        HYD_BUFFER_MODE_ABORT, 1.001) == HYD_DIRECT_START_STARTED);

    fb.AXIS_REF.timestamp = 1.002;
    HYD_MotionControlFB_Cycle(&fb);

    /* Q_p = 1.0 + 1.0·(140 − 100) = 41 L/min < Q_v 43.4 → 820 rpm。
     * 离新 Plim 有压力余量时，压力环一拍内就让流量回到接近 Q_v，
     * 到新 Plim 附近才重新接管（spec §4.2 注、§9 上机注意第 1 条）。 */
    assert(fabs(fb.PUMP_SPEED - 820.0) < 1.0);
}

static void test_reverse_position_handover_keeps_old_direction(void) {
    HYD_MotionSegment successor = make_position_segment();
    HYD_MotionControlFB fb;
    int cycle;

    successor.direction = HYD_DIRECTION_RETRACT;
    successor.targetPosition = 0.0;
    successor.maxVelocity = 10.0;
    seed_active_segment(&fb, make_position_segment(),
                        HYD_DIRECT_CMD_MOVE_ABSOLUTE);
    fb.AXIS_REF.velocity = 1.0;
    fb._plannerState.transitionTargetVelocity = 10.0;
    fb._plannerState.transitionRate = 50.0;
    fb._plannerState.transitionTimeMs = 100.0;

    assert(HYD_MotionControlFB_StartDirectCommand(
        &fb, HYD_DIRECT_CMD_MOVE_ABSOLUTE, &successor, NULL,
        HYD_BUFFER_MODE_ABORT, 1.001) == HYD_DIRECT_START_STARTED);
    for (cycle = 1; cycle <= 100; ++cycle) {
        fb.AXIS_REF.timestamp = 1.001 + cycle * 0.001;
        HYD_MotionControlFB_Cycle(&fb);
        assert(fb.STATE.plannedVelocity > 0.0);
        assert(fb.STATE.plannedDirection == HYD_DIRECTION_EXTEND);
        assert(fb.PUMP_SPEED >= 0.0);
    }

    fb.AXIS_REF.timestamp += 0.001;
    HYD_MotionControlFB_Cycle(&fb);
    assert(fabs(fb.STATE.plannedVelocity) < 0.001);
    assert(fb.STATE.plannedDirection == HYD_DIRECTION_EXTEND);
    assert(fb.PUMP_SPEED >= 0.0);

    fb.AXIS_REF.timestamp += 0.001;
    HYD_MotionControlFB_Cycle(&fb);
    assert(fb.STATE.plannedVelocity < 0.0);
    assert(fb.STATE.plannedDirection == HYD_DIRECTION_RETRACT);
}

static void test_pressure_handover_keeps_initialized_zero_feedback_fallback(void) {
    HYD_MotionControlFB fb;
    HYD_MotionSegment pressure = make_pressure_segment();

    seed_active_segment(&fb, make_position_segment(),
                        HYD_DIRECT_CMD_MOVE_ABSOLUTE);
    fb._plannerState.lastTargetVelocity = 0.0;
    fb.STATE.plannedVelocity = 0.0;
    fb._simFeedback.targetVelocity = 0.0;
    fb.AXIS_REF.velocity = 5.0;

    assert(HYD_MotionControlFB_StartDirectCommand(
        &fb, HYD_DIRECT_CMD_PRESSURE_HANDLE, &pressure, NULL,
        HYD_BUFFER_MODE_ABORT, 1.001) == HYD_DIRECT_START_STARTED);
    assert(fabs(fb._plannerState.lastTargetVelocity - 5.0) < 0.001);
    assert(fabs(fb.STATE.plannedVelocity - 5.0) < 0.001);
}

static void test_initialized_zero_planner_state_ignores_feedback_velocity(void) {
    HYD_MotionControlFB fb;
    HYD_MotionSegment successor = make_speed_segment();

    seed_active_segment(&fb, make_speed_segment(),
                        HYD_DIRECT_CMD_MOVE_VELOCITY);
    fb._plannerState.lastTargetVelocity = 0.0;
    fb._plannerState.transitionTargetVelocity = 0.0;
    fb._plannerState.transitionRate = 0.0;
    fb._plannerState.transitionTimeMs = 100.0;
    fb.STATE.plannedVelocity = 0.0;
    fb.AXIS_REF.velocity = 5.0;
    successor.maxVelocity = 10.0;

    assert(HYD_MotionControlFB_StartDirectCommand(
        &fb, HYD_DIRECT_CMD_MOVE_VELOCITY, &successor, NULL,
        HYD_BUFFER_MODE_ABORT, 1.001) == HYD_DIRECT_START_STARTED);
    assert(fabs(fb._plannerState.lastTargetVelocity) < 0.001);

    fb.AXIS_REF.timestamp = 1.002;
    HYD_MotionControlFB_Cycle(&fb);
    assert(fabs(fb.STATE.plannedVelocity - 0.1) < 0.02);
}

static void test_explicit_abort_still_zeros_outputs(void) {
    HYD_MotionControlFB fb;

    seed_active_segment(&fb, make_speed_segment(), HYD_DIRECT_CMD_MOVE_VELOCITY);
    assert(HYD_MotionControlFB_Abort(&fb));
    HYD_MotionControlFB_Execute(&fb);
    assert(fb.PUMP_SPEED == 0.0);
    assert(fb.STATE.commandedPumpSpeed == 0.0);
    assert(fb._lastCommandedFlow == 0.0);
}

static void test_pressure_handover_does_not_timeout(void) {
    HYD_MotionControlFB fb;
    HYD_MotionSegment position = make_position_segment();
    HYD_MotionSegment pressure = make_pressure_segment();
    HYD_TIME timestamp;

    /* A timeout configured for position motion must not leak into pressure hold. */
    position.timeoutLimit = 0.05;
    pressure.timeoutLimit = 0.05;

    HYD_MotionControlFB_Init(&fb);
    fb.USE_RECIPE = false;
    fb.FLOW_TO_PUMP_SPEED_GAIN = 100.0;
    fb.PUMP_SPEED_LIMIT = 3000.0;
    fb.AXIS_REF.position = 0.0;
    fb.AXIS_REF.velocity = 0.0;
    fb.AXIS_REF.flow = 0.0;
    fb.AXIS_REF.pressure = 10.0;
    fb.AXIS_REF.timestamp = 0.0;

    assert(HYD_MotionControlFB_LoadDirectSegment(&fb, &position));
    assert(HYD_MotionControlFB_StartSegment(&fb, 0U, 0.0));
    HYD_MotionControlFB_Cycle(&fb);

    /* Switch while the position segment is active, as the process layer does. */
    assert(HYD_MotionControlFB_StartDirectCommand(
        &fb,
        HYD_DIRECT_CMD_PRESSURE_HANDLE,
        &pressure,
        NULL,
        HYD_BUFFER_MODE_ABORT,
        0.1));

    for (timestamp = 0.2; timestamp <= 0.5; timestamp += 0.01) {
        fb.AXIS_REF.timestamp = timestamp;
        fb.AXIS_REF.pressure = 10.0;
        HYD_MotionControlFB_Cycle(&fb);
        assert(!fb.STATE.faultActive);
        assert(fb.DIAGNOSTIC.code != HYD_DIAG_CODE_TIMEOUT);
    }

    assert(fb._activeSegment.mode == HYD_MODE_PRESSURE_CLOSED_LOOP);
    assert(HYD_Segment_GetTimeoutLimit(&pressure) == 0.0);
}

static void test_position_segment_timeout_remains_enabled(void) {
    HYD_MotionControlFB fb;
    HYD_MotionSegment position = make_position_segment();

    position.timeoutLimit = 0.05;

    HYD_MotionControlFB_Init(&fb);
    fb.USE_RECIPE = false;
    fb.FLOW_TO_PUMP_SPEED_GAIN = 100.0;
    fb.PUMP_SPEED_LIMIT = 3000.0;
    fb.AXIS_REF.position = 0.0;
    fb.AXIS_REF.velocity = 0.0;
    fb.AXIS_REF.flow = 0.0;
    fb.AXIS_REF.pressure = 10.0;
    fb.AXIS_REF.timestamp = 0.0;

    assert(HYD_MotionControlFB_LoadDirectSegment(&fb, &position));
    assert(HYD_MotionControlFB_StartSegment(&fb, 0U, 0.0));
    HYD_MotionControlFB_Cycle(&fb);

    fb.AXIS_REF.timestamp = 0.1;
    HYD_MotionControlFB_Cycle(&fb);

    assert(fb.STATE.faultActive);
    assert(fb.DIAGNOSTIC.code == HYD_DIAG_CODE_TIMEOUT);
}

int main(void) {
    printf("planner_state=%zu bytes; motion_control_fb=%zu bytes\n",
           sizeof(HYD_MotionPlannerState), sizeof(HYD_MotionControlFB));
#if HYD_REAL_PRECISION_FLOAT
    assert(sizeof(HYD_MotionPlannerState) <= 28U);
    /* +8: _params.responseDelayMs / zeroEndSettleVelocity (zero-end stop).
     * +16: 两份 HYD_ContinuousAbsoluteContext 各加 flowPercent / flowRampRate。
     * +104: 压力环二自由度（参考延迟线、轨迹状态、参数 37–39、纹波补偿系数）。
     * +8: 限压 PI 参数 40/41（限幅状态 pressureScale/LastTime 换成积分，−4，被对齐吃掉）。 */
    assert(sizeof(HYD_MotionControlFB) <= 3440U);
#endif
    printf("Running bumpless direct mode handover tests...\n");
    test_position_pressure_handover();
    test_speed_pressure_handover();
    test_pressure_position_handover();
    test_pressure_speed_handover();
    test_position_to_position_motion_handover();
    test_speed_to_speed_motion_handover();
    test_position_to_speed_motion_handover();
    test_speed_to_position_motion_handover();
    test_motion_handover_does_not_add_second_pump_slew();
    test_pressure_handover_does_not_add_second_pump_slew();
    test_reverse_position_handover_keeps_old_direction();
    test_pressure_limited_handover_continues_from_executed_flow();
    test_pressure_handover_seeds_from_executed_flow();
    test_takeover_while_limited_to_zero_keeps_zero_flow();
    test_raised_limit_with_margin_releases_toward_velocity_flow();
    test_pressure_handover_keeps_initialized_zero_feedback_fallback();
    test_initialized_zero_planner_state_ignores_feedback_velocity();
    test_explicit_abort_still_zeros_outputs();
    test_pressure_handover_does_not_timeout();
    test_position_segment_timeout_remains_enabled();
    printf("All bumpless direct mode handover tests passed.\n");
    return 0;
}
