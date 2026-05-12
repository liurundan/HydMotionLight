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

static int create_sim_axis(void) {
    HYD_CREATEMOTION cm;
    memset(&cm, 0, sizeof(cm));
    IEC_VAL(cm.EN) = true;
    IEC_VAL(cm.USE_RECIPE) = false;
    IEC_VAL(cm.FLOW_TO_PUMPSPEED) = 1.0f;
    IEC_VAL(cm.PUMPSPEED_LIMIT) = 3000.0f;
    IEC_VAL(cm.USE_SIMULATION) = true;
    __mcl_cmd_CreateMotion(&cm);
    return (int)IEC_VAL(cm.AXISID);
}

/* ==================================================================
 * Scenario A: Stop in same scan as MoveAbsolute (before Publish)
 *
 * This is the most likely scenario in a real PLC:
 *   MoveAbsolute(EXECUTE:=TRUE) → queues START (FB_STATE stays IDLE/READY)
 *   Stop(EXECUTE:=TRUE)         → sees IDLE → DONE=true immediately!
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

    /* Verify MoveAbsolute queued the START */
    {
        HYD_MotionControlFB* fb = __MK_GetPublic_MotionControlFB(axisId);
        printf("  After Move: FB_STATE=%d, _pendingCommand=%d\n",
               fb->FB_STATE, fb->_pendingCommand);
        CHECK(fb->_pendingCommand == HYD_CMD_START,
              "START command should be queued after MoveAbsolute");
        CHECK(fb->FB_STATE == HYD_FB_STATE_READY ||
              fb->FB_STATE == HYD_FB_STATE_IDLE,
              "FB_STATE not yet STARTING (no Publish)");
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

    /* BUG: Stop should NOT be DONE immediately - there's a pending START */
    CHECK(IEC_VAL(stop.DONE) == false,
          "Stop.DONE should NOT be true immediately when START is pending");
    CHECK(IEC_VAL(stop.BUSY) == true,
          "Stop.BUSY should be true (should process pending START then stop)");
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

int main(void) {
    printf("=== Stop Immediate DONE Reproduction ===\n");

    __HydMotion_framework_Init();
    test_stop_same_scan_as_moveabsolute();

    __HydMotion_framework_Init();
    test_stop_after_motion_running();

    printf("\n=== Results: %d/%d passed ===\n", tests_passed, tests_run);
    return (tests_passed == tests_run) ? 0 : 1;
}
