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
 * Using a 20 mm/s motion and Stop.DECELERATION=50, the stop should
 * take roughly 400 cycles, not 100 cycles from the motion acceleration.
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
    IEC_VAL(stop.DECELERATION) = 50.0f;
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

    CHECK(stopDoneStep >= 250,
          "Stop.DECELERATION should extend the stop duration");
}

/* ==================================================================
 * Scenario D: Stop should decelerate from planned velocity when feedback
 * velocity is not being closed by the test harness / hardware adapter yet.
 *
 * AXIS_REF.velocity remains zero here, but MoveAbsolute has already produced
 * a nonzero planned velocity. Stop must not treat that as already stopped.
 * ================================================================== */
static void test_stop_uses_planned_velocity_when_feedback_velocity_is_zero(void) {
    HYD_MOVEABSOLUTE ma;
    HYD_STOP stop;
    HYD_MotionControlFB* fb;
    int axisId, step;
    int stopDoneStep = 0;
    HYD_REAL plannedBeforeStop;

    printf("\n--- Scenario D: Stop uses planned velocity when feedback velocity is zero ---\n");
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
    IEC_VAL(stop.DECELERATION) = 50.0f;
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

    printf("\n=== Results: %d/%d passed ===\n", tests_passed, tests_run);
    return (tests_passed == tests_run) ? 0 : 1;
}
