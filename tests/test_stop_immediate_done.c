/**
 * Minimal reproduction: MoveAbsolute + Stop → Stop.DONE immediately true?
 *
 * Scenario A: Stop in SAME scan as MoveAbsolute (before Publish).
 * Scenario B: Stop in LATER scan after motion is running (with Publish in between).
 * Scenario C: Stop without simulation (AXIS_REF.velocity stays at zero).
 */

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdbool.h>

#include "motion_interface.h"
#include "motion_control.h"

extern HYD_MotionControlFB* __MK_GetPublic_MotionControlFB(int index);

#define IEC_VAL(var) ((var).value)

static int tests_run = 0;
static int tests_passed = 0;

#define CHECK(cond, msg) do { \
    tests_run++; \
    if (cond) { tests_passed++; printf("  PASS: %s\n", msg); } \
    else { printf("  FAIL: %s\n", msg); } \
} while (0)

static int create_axis(bool useSimulation) {
    HYD_CREATEMOTION cm;
    memset(&cm, 0, sizeof(cm));
    IEC_VAL(cm.EN) = true;
    IEC_VAL(cm.USE_RECIPE) = false;
    IEC_VAL(cm.FLOW_TO_PUMPSPEED) = 1.0f;
    IEC_VAL(cm.PUMPSPEED_LIMIT) = 3000.0f;
    IEC_VAL(cm.USE_SIMULATION) = useSimulation;
    __mcl_cmd_CreateMotion(&cm);
    return (int)IEC_VAL(cm.AXISID);
}

static int create_sim_axis(void) {
    return create_axis(true);
}

/* ==================================================================
 * Scenario A: Stop in same scan as MoveAbsolute (before Publish)
 *
 * Current direct-mode semantics on an idle axis:
 *   MoveAbsolute(EXECUTE:=TRUE) starts immediately on the same scan
 *   Stop(EXECUTE:=TRUE)         must still take over and stay BUSY first
 * ================================================================== */
static void test_stop_same_scan_as_moveabsolute(void) {
    HYD_MOVEABSOLUTE ma;
    HYD_STOP stop;
    int axisId;

    printf("\n--- Scenario A: Stop same-scan as MoveAbsolute ---\n");
    __HydMotion_framework_Init();
    axisId = create_sim_axis();
    CHECK(axisId >= 0, "Create sim axis");

    /* MoveAbsolute execRising (no Publish in between!) */
    memset(&ma, 0, sizeof(ma));
    IEC_VAL(ma.EN) = true;
    IEC_VAL(ma.AXISID) = axisId;
    IEC_VAL(ma.POSITION) = 100.0f;
    IEC_VAL(ma.VELOCITY) = 50.0f;
    IEC_VAL(ma.ACCELERATION) = 200.0f;
    IEC_VAL(ma.DIRECTION) = 1;
    IEC_VAL(ma.BUFFERMODE) = 1;  /* HYD_BUFFER_MODE_BUFFER, avoid implicit ABORT=0 */
    IEC_VAL(ma.EXECUTE) = true;
    ma.EXECUTE0.value = false;
    __mcl_cmd_MoveAbsolute(&ma);

    /* Verify MoveAbsolute already started on the idle axis */
    {
        HYD_MotionControlFB* fb = __MK_GetPublic_MotionControlFB(axisId);
        printf("  After Move: FB_STATE=%d, _pendingCommand=%d\n",
               fb->FB_STATE, fb->_pendingCommand);
        CHECK(fb->_pendingCommand == HYD_CMD_NONE,
              "Idle-axis MoveAbsolute should not leave a pending START behind");
        CHECK(fb->FB_STATE == HYD_FB_STATE_STARTING,
              "MoveAbsolute should already enter STARTING before same-scan Stop");
    }

    /* Stop execRising (same scan, before any Publish) */
    memset(&stop, 0, sizeof(stop));
    IEC_VAL(stop.EN) = true;
    IEC_VAL(stop.EXECUTE) = true;
    stop.EXECUTE0.value = false;
    IEC_VAL(stop.AXISID) = axisId;
    __mcl_cmd_Stop(&stop);

    printf("  Stop.DONE=%d, Stop.BUSY=%d, Stop.ERROR=%d\n",
           (int)IEC_VAL(stop.DONE),
           (int)IEC_VAL(stop.BUSY),
           (int)IEC_VAL(stop.ERROR));

    /* Stop should still take over the just-started motion before reporting DONE */
    CHECK(IEC_VAL(stop.DONE) == false,
          "Stop.DONE should NOT be true immediately when motion started in the same scan");
    CHECK(IEC_VAL(stop.BUSY) == true,
          "Stop.BUSY should be true while same-scan Stop takes over the motion");
}

/* ==================================================================
 * Scenario B: Stop in later scan after motion is running
 *
 * MoveAbsolute → Publish (motion begins) → loop to build velocity →
 * Stop → verify deceleration takes time, not immediate
 * ================================================================== */
static void test_stop_after_motion_running(void) {
    HYD_MOVEABSOLUTE ma;
    HYD_STOP stop;
    int axisId, step;
    HYD_REAL velocityBeforeStop;
    int stopDoneStep = 0;

    printf("\n--- Scenario B: Stop after motion is running ---\n");
    __HydMotion_framework_Init();
    axisId = create_sim_axis();
    CHECK(axisId >= 0, "Create sim axis");

    /* Start MoveAbsolute */
    memset(&ma, 0, sizeof(ma));
    IEC_VAL(ma.EN) = true;
    IEC_VAL(ma.AXISID) = axisId;
    IEC_VAL(ma.POSITION) = 100.0f;
    IEC_VAL(ma.VELOCITY) = 50.0f;
    IEC_VAL(ma.ACCELERATION) = 200.0f;
    IEC_VAL(ma.DIRECTION) = 1;
    IEC_VAL(ma.BUFFERMODE) = 1;  /* HYD_BUFFER_MODE_BUFFER */

    IEC_VAL(ma.EXECUTE) = true;
    ma.EXECUTE0.value = false;
    __mcl_cmd_MoveAbsolute(&ma);
    __HydMotion_framework_Publish();

    IEC_VAL(ma.EXECUTE) = true;
    ma.EXECUTE0.value = true;
    __mcl_cmd_MoveAbsolute(&ma);

    /* Run several cycles to build up simulated velocity */
    for (step = 0; step < 50; step++) {
        __HydMotion_framework_Publish();
        IEC_VAL(ma.EXECUTE) = true;
        ma.EXECUTE0.value = true;
        __mcl_cmd_MoveAbsolute(&ma);
    }

    /* Capture velocity before Stop */
    {
        HYD_MotionControlFB* fb = __MK_GetPublic_MotionControlFB(axisId);
        velocityBeforeStop = fb->AXIS_REF.velocity;
        printf("  Velocity BEFORE stop: %.4f mm/s\n", (double)velocityBeforeStop);
        CHECK(velocityBeforeStop > 1.0,
              "Axis should be moving BEFORE stop (velocity > 1.0 mm/s)");
    }

    /* Call Stop */
    memset(&stop, 0, sizeof(stop));
    IEC_VAL(stop.EN) = true;
    IEC_VAL(stop.EXECUTE) = true;
    stop.EXECUTE0.value = false;
    IEC_VAL(stop.AXISID) = axisId;
    __mcl_cmd_Stop(&stop);

    /* First Stop call should NOT have DONE=true */
    CHECK(IEC_VAL(stop.DONE) == false,
          "First Stop call should have DONE=false (deceleration not started yet inside this call)");

    /* Now loop Publish → Stop until DONE */
    for (step = 0; step < 1000; step++) {
        __HydMotion_framework_Publish();
        IEC_VAL(stop.EXECUTE) = true;
        stop.EXECUTE0.value = true;
        __mcl_cmd_Stop(&stop);

        if (IEC_VAL(stop.DONE)) {
            stopDoneStep = step + 1;
            break;
        }
    }

    printf("  Stop.DONE reached after %d Publish cycles\n", stopDoneStep);
    printf("  Velocity at Stop.DONE: ");
    {
        HYD_MotionControlFB* fb = __MK_GetPublic_MotionControlFB(axisId);
        printf("%.4f mm/s\n", (double)fb->AXIS_REF.velocity);
    }

    /* DONE should NOT be immediate -- it should take significant steps to decelerate */
    CHECK(stopDoneStep > 5,
          "Stop.DONE should take > 5 cycles (deceleration should take time)");
    CHECK(IEC_VAL(stop.DONE) == true,
          "Stop.DONE should eventually be true");
    CHECK(IEC_VAL(stop.BUSY) == false,
          "Stop.BUSY should be false after DONE");
    CHECK(IEC_VAL(stop.ERROR) == false,
          "Stop should not set ERROR");

    /* Verify velocity is zero after DONE */
    {
        HYD_MotionControlFB* fb = __MK_GetPublic_MotionControlFB(axisId);
        CHECK(fabs(fb->AXIS_REF.velocity) < 0.01,
              "Velocity should be ~0 after Stop.DONE");
        CHECK(fb->FB_STATE == HYD_FB_STATE_DONE,
              "FB_STATE should be DONE after Stop");
    }
}

/* ==================================================================
 * Scenario C: Stop.DECELERATION should affect the stop duration
 *
 * Using a 20 mm/s motion and Stop.DECELERATION=200 ms, the stop should
 * take roughly 200 cycles, not an immediate stop.
 * ================================================================== */
static void test_stop_deceleration_input_is_honored(void) {
    HYD_MOVEABSOLUTE ma;
    HYD_STOP stop;
    int axisId, step;
    int stopDoneStep = 0;

    printf("\n--- Scenario C: Stop deceleration input is honored ---\n");
    __HydMotion_framework_Init();
    axisId = create_sim_axis();
    CHECK(axisId >= 0, "Create sim axis");

    memset(&ma, 0, sizeof(ma));
    IEC_VAL(ma.EN) = true;
    IEC_VAL(ma.AXISID) = axisId;
    IEC_VAL(ma.POSITION) = 200.0f;
    IEC_VAL(ma.VELOCITY) = 20.0f;
    IEC_VAL(ma.ACCELERATION) = 200.0f;
    IEC_VAL(ma.DIRECTION) = 1;
    IEC_VAL(ma.BUFFERMODE) = 1;

    IEC_VAL(ma.EXECUTE) = true;
    ma.EXECUTE0.value = false;
    __mcl_cmd_MoveAbsolute(&ma);
    __HydMotion_framework_Publish();

    IEC_VAL(ma.EXECUTE) = true;
    ma.EXECUTE0.value = true;
    __mcl_cmd_MoveAbsolute(&ma);

    for (step = 0; step < 100; step++) {
        __HydMotion_framework_Publish();
        IEC_VAL(ma.EXECUTE) = true;
        ma.EXECUTE0.value = true;
        __mcl_cmd_MoveAbsolute(&ma);
    }

    memset(&stop, 0, sizeof(stop));
    IEC_VAL(stop.EN) = true;
    IEC_VAL(stop.EXECUTE) = true;
    stop.EXECUTE0.value = false;
    IEC_VAL(stop.AXISID) = axisId;
    IEC_VAL(stop.DECELERATION) = 200.0f;
    __mcl_cmd_Stop(&stop);

    for (step = 0; step < 1000; step++) {
        __HydMotion_framework_Publish();
        IEC_VAL(stop.EXECUTE) = true;
        stop.EXECUTE0.value = true;
        __mcl_cmd_Stop(&stop);
        if (IEC_VAL(stop.DONE)) {
            stopDoneStep = step + 1;
            break;
        }
    }

    CHECK(stopDoneStep >= 150,
          "Stop.DECELERATION should extend the stop duration");
}

/* ==================================================================
 * Scenario D: Stop should decelerate from the planner command when feedback
 * velocity is zero.
 *
 * AXIS_REF.velocity remains zero here, but MoveAbsolute has already produced
 * a nonzero planned velocity command.
 * ================================================================== */
static void test_stop_uses_planned_velocity_when_feedback_velocity_is_zero(void) {
    HYD_MOVEABSOLUTE ma;
    HYD_STOP stop;
    HYD_MotionControlFB* fb;
    int axisId, step;
    int stopDoneStep = 0;
    HYD_REAL plannedBeforeStop;

    printf("\n--- Scenario D: Stop uses planner command when feedback velocity is zero ---\n");
    __HydMotion_framework_Init();
    axisId = create_axis(false);
    CHECK(axisId >= 0, "Create non-sim axis");
    fb = __MK_GetPublic_MotionControlFB(axisId);

    memset(&ma, 0, sizeof(ma));
    IEC_VAL(ma.EN) = true;
    IEC_VAL(ma.AXISID) = axisId;
    IEC_VAL(ma.POSITION) = 200.0f;
    IEC_VAL(ma.VELOCITY) = 20.0f;
    IEC_VAL(ma.ACCELERATION) = 200.0f;
    IEC_VAL(ma.DIRECTION) = 1;
    IEC_VAL(ma.BUFFERMODE) = 1;

    IEC_VAL(ma.EXECUTE) = true;
    ma.EXECUTE0.value = false;
    __mcl_cmd_MoveAbsolute(&ma);

    for (step = 0; step < 30; step++) {
        fb->AXIS_REF.timestamp += 0.001f;
        __HydMotion_framework_Publish();
        IEC_VAL(ma.EXECUTE) = true;
        ma.EXECUTE0.value = true;
        __mcl_cmd_MoveAbsolute(&ma);
    }

    plannedBeforeStop = fb->STATE.plannedVelocity;
    printf("  Feedback velocity: %.4f, planned velocity: %.4f\n",
           (double)fb->AXIS_REF.velocity,
           (double)plannedBeforeStop);
    CHECK(fabs(fb->AXIS_REF.velocity) < 0.001,
          "Feedback velocity stays zero in this harness");
    CHECK(fabs(plannedBeforeStop) > 1.0,
          "MoveAbsolute should have nonzero planned velocity before Stop");

    memset(&stop, 0, sizeof(stop));
    IEC_VAL(stop.EN) = true;
    IEC_VAL(stop.EXECUTE) = true;
    stop.EXECUTE0.value = false;
    IEC_VAL(stop.AXISID) = axisId;
    IEC_VAL(stop.DECELERATION) = 200.0f;
    __mcl_cmd_Stop(&stop);

    CHECK(IEC_VAL(stop.DONE) == false,
          "Stop.DONE should not be true on first call when planned velocity is nonzero");

    for (step = 0; step < 1000; step++) {
        fb->AXIS_REF.timestamp += 0.001f;
        __HydMotion_framework_Publish();
        IEC_VAL(stop.EXECUTE) = true;
        stop.EXECUTE0.value = true;
        __mcl_cmd_Stop(&stop);
        if (IEC_VAL(stop.DONE)) {
            stopDoneStep = step + 1;
            break;
        }
    }

    printf("  Stop.DONE reached after %d Publish cycles\n", stopDoneStep);
    CHECK(stopDoneStep > 5,
          "Stop.DONE should take multiple cycles when planned velocity must decelerate");
    CHECK(IEC_VAL(stop.ERROR) == false,
          "Stop should not set ERROR while decelerating from planned velocity");
}

static HYD_MotionSegment make_stop_test_segment(HYD_ControlMode mode) {
    HYD_MotionSegment segment;

    memset(&segment, 0, sizeof(segment));
    segment.segmentType = HYD_SEGMENT_TYPE_OTHER;
    segment.mode = mode;
    segment.direction = HYD_DIRECTION_EXTEND;
    segment.maxVelocity = 20.0f;
    segment.maxAcceleration = 100.0f;
    segment.maxDeceleration = 100.0f;
    segment.maxFlow = 100.0f;
    segment.velocityToFlowGain = 2.0f;
    if (mode == HYD_MODE_POSITION) {
        segment.planner = HYD_PLANNER_POSITION_BASED;
        segment.endCondition = HYD_END_POSITION;
        segment.targetPosition = 200.0f;
        segment.positionTolerance = 0.01f;
    } else {
        segment.planner = HYD_PLANNER_TIME_BASED;
        segment.endCondition = HYD_END_MANUAL;
    }
    return segment;
}

static void assert_stop_latches_planned_command(HYD_ControlMode mode,
                                                const char* modeName) {
    HYD_MotionControlFB fb;
    HYD_MotionSegment segment = make_stop_test_segment(mode);
    HYD_REAL plannedBeforeStop;
    HYD_REAL feedbackVelocity;
    HYD_REAL previousMagnitude;
    int step;

    HYD_MotionControlFB_Init(&fb);
    fb.USE_RECIPE = false;
    fb.FLOW_TO_PUMP_SPEED_GAIN = 10.0f;
    fb.PUMP_SPEED_LIMIT = 3000.0f;
    fb.AXIS_REF.position = 0.0f;
    fb.AXIS_REF.timestamp = 0.0f;
    CHECK(HYD_MotionControlFB_LoadDirectSegment(&fb, &segment),
          "Load direct stop test segment");
    CHECK(HYD_MotionControlFB_StartSegment(&fb, 0u, 0.0f),
          "Start direct stop test segment");
    HYD_MotionControlFB_Cycle(&fb);
    fb.AXIS_REF.timestamp = 0.05f;
    HYD_MotionControlFB_Cycle(&fb);

    plannedBeforeStop = fb.STATE.plannedVelocity;
    CHECK(fabs(plannedBeforeStop) > 0.1f,
          "Motion planner has issued a nonzero velocity command");
    feedbackVelocity = plannedBeforeStop * 0.25f;
    fb.AXIS_REF.velocity = feedbackVelocity;

    CHECK(HYD_MotionControlFB_Stop(&fb, fb.AXIS_REF.timestamp, 200.0f),
          "Queue Stop while motion is running");
    fb.AXIS_REF.timestamp += 0.001f;
    HYD_MotionControlFB_Cycle(&fb);

    printf("  %s Stop start: planned=%.4f feedback=%.4f latched=%.4f\n",
           modeName, (double)plannedBeforeStop, (double)feedbackVelocity,
           (double)fb._stopStartVel);
    CHECK(fabs(fb._stopStartVel - plannedBeforeStop) < 0.001f,
          "Stop starts from the last planner command, not measured velocity");

    previousMagnitude = fabs(fb.STATE.plannedVelocity);
    for (step = 0; step < 4; ++step) {
        fb.AXIS_REF.timestamp += 0.01f;
        HYD_MotionControlFB_Cycle(&fb);
        CHECK(fabs(fb.STATE.plannedVelocity) <= previousMagnitude + 0.001f,
              "Stop planner command magnitude decreases monotonically");
        CHECK(fb.STATE.plannedVelocity == 0.0f ||
                  fb.STATE.plannedVelocity * plannedBeforeStop > 0.0f,
              "Stop planner command does not reverse direction");
        previousMagnitude = fabs(fb.STATE.plannedVelocity);
    }
}

static void test_stop_latches_planned_command_for_position_and_speed(void) {
    printf("\n--- Scenario E: Stop latches planner command in position and speed modes ---\n");
    assert_stop_latches_planned_command(HYD_MODE_POSITION, "Position");
    assert_stop_latches_planned_command(HYD_MODE_SPEED_RAMP, "Speed");
}

static void assert_stop_does_not_fallback_from_zero_planner(HYD_ControlMode mode) {
    HYD_MotionControlFB fb;
    HYD_MotionSegment segment = make_stop_test_segment(mode);
    HYD_REAL feedbackVelocity;

    HYD_MotionControlFB_Init(&fb);
    fb.USE_RECIPE = false;
    fb.FLOW_TO_PUMP_SPEED_GAIN = 10.0f;
    fb.PUMP_SPEED_LIMIT = 3000.0f;
    fb.AXIS_REF.timestamp = 0.0f;
    CHECK(HYD_MotionControlFB_LoadDirectSegment(&fb, &segment),
          "Load segment for zero planner command");
    CHECK(HYD_MotionControlFB_StartSegment(&fb, 0u, 0.0f),
          "Start segment for zero planner command");
    HYD_MotionControlFB_Cycle(&fb);
    fb.AXIS_REF.timestamp = 0.05f;
    HYD_MotionControlFB_Cycle(&fb);

    feedbackVelocity = fb.STATE.plannedVelocity;
    CHECK(fabs(feedbackVelocity) > 0.1f,
          "Axis feedback scenario starts with nonzero motion");
    fb.AXIS_REF.velocity = feedbackVelocity;
    fb.STATE.plannedVelocity = 0.0f;
    fb.STATE.plannedFlow = 0.0f;
    fb.STATE.commandedPumpSpeed = 0.0f;
    fb.PUMP_SPEED = 0.0f;
    fb._plannerState.lastTargetVelocity = 0.0f;
    fb._plannerState.lastTargetFlow = 0.0f;
    fb._lastCommandedFlow = 0.0f;

    CHECK(HYD_MotionControlFB_Stop(&fb, fb.AXIS_REF.timestamp, 200.0f),
          "Queue Stop with zero planner command and nonzero feedback");
    fb.AXIS_REF.timestamp += 0.001f;
    HYD_MotionControlFB_Cycle(&fb);

    CHECK(fb._stopStartVel == 0.0f,
          "A zero planner command must not fall back to measured velocity");
    CHECK(fb.PUMP_SPEED == 0.0f,
          "Stop must not raise pump command when the last planner command was zero");
}

static void test_stop_keeps_zero_planner_command_in_position_and_speed(void) {
    printf("\n--- Scenario F: Stop does not fall back from zero planner command ---\n");
    assert_stop_does_not_fallback_from_zero_planner(HYD_MODE_POSITION);
    assert_stop_does_not_fallback_from_zero_planner(HYD_MODE_SPEED_RAMP);
}

static void test_stop_output_respects_soft_limit(void) {
    HYD_MotionControlFB fb;
    HYD_MotionSegment segment = make_stop_test_segment(HYD_MODE_SPEED_RAMP);

    printf("\n--- Scenario G: HYD_STOP output respects the soft limit ---\n");
    HYD_MotionControlFB_Init(&fb);
    fb.USE_RECIPE = false;
    fb.FLOW_TO_PUMP_SPEED_GAIN = 10.0f;
    fb.PUMP_SPEED_LIMIT = 3000.0f;
    fb.cylinderConfig.areaExtendMm2 = 10000.0f;
    fb.cylinderConfig.areaRetractMm2 = 6000.0f;
    fb.cylinderConfig.strokeMm = 100.0f;
    fb.cylinderConfig.softLimitBandMm = 20.0f;
    fb.AXIS_REF.timestamp = 0.0f;
    CHECK(HYD_MotionControlFB_LoadDirectSegment(&fb, &segment),
          "Load direct speed segment for soft-limit test");
    CHECK(HYD_MotionControlFB_StartSegment(&fb, 0u, 0.0f),
          "Start direct speed segment for soft-limit test");
    HYD_MotionControlFB_Cycle(&fb);
    fb.AXIS_REF.timestamp = 0.05f;
    HYD_MotionControlFB_Cycle(&fb);
    CHECK(fb.PUMP_SPEED > 0.0f,
          "Speed motion has nonzero pump command before protection");

    fb.AXIS_REF.position = 101.0f;
    CHECK(HYD_MotionControlFB_Stop(&fb, fb.AXIS_REF.timestamp, 200.0f),
          "Queue HYD_STOP as the axis reaches the soft limit");
    fb.AXIS_REF.timestamp += 0.001f;
    HYD_MotionControlFB_Cycle(&fb);

    CHECK(fb.PUMP_SPEED == 0.0f,
          "Soft limit must constrain the decelerating HYD_STOP output");
    CHECK((fb.STATE.limitFlags & HYD_LIMIT_FLAG_SOFT) != 0u,
          "Stop cycle must report that the soft-limit clamp was applied");
}

int main(void) {
    printf("=== Stop Immediate DONE Reproduction ===\n");

    __HydMotion_framework_Init();
    test_stop_same_scan_as_moveabsolute();

    __HydMotion_framework_Init();
    test_stop_after_motion_running();

    __HydMotion_framework_Init();
    test_stop_deceleration_input_is_honored();

    __HydMotion_framework_Init();
    test_stop_uses_planned_velocity_when_feedback_velocity_is_zero();

    test_stop_latches_planned_command_for_position_and_speed();
    test_stop_keeps_zero_planner_command_in_position_and_speed();
    test_stop_output_respects_soft_limit();

    printf("\n=== Results: %d/%d passed ===\n", tests_passed, tests_run);
    return (tests_passed == tests_run) ? 0 : 1;
}
