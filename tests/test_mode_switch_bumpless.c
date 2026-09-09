#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "motion_control.h"

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

static void test_explicit_abort_still_zeros_outputs(void) {
    HYD_MotionControlFB fb;

    seed_active_segment(&fb, make_speed_segment(), HYD_DIRECT_CMD_MOVE_VELOCITY);
    assert(HYD_MotionControlFB_Abort(&fb));
    HYD_MotionControlFB_Execute(&fb);
    assert(fb.PUMP_SPEED == 0.0);
    assert(fb.STATE.commandedPumpSpeed == 0.0);
    assert(fb._lastCommandedFlow == 0.0);
}

int main(void) {
    printf("Running bumpless direct mode handover tests...\n");
    test_position_pressure_handover();
    test_speed_pressure_handover();
    test_pressure_position_handover();
    test_pressure_speed_handover();
    test_explicit_abort_still_zeros_outputs();
    printf("All bumpless direct mode handover tests passed.\n");
    return 0;
}
