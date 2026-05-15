#include <stdio.h>
#include <assert.h>
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
    else { printf("  FAIL: %s [line %d]\n", msg, __LINE__); } \
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

static void test_moveabsolute_stop_loop(void) {
    HYD_MOVEABSOLUTE ma;
    HYD_STOP stop;
    HYD_MotionControlFB* fb;
    int axisId, step;
    int stopDoneCycle = 0;

    __HydMotion_framework_Init();
    axisId = create_sim_axis();
    CHECK(axisId >= 0, "CreateMotion should allocate a simulation axis");
    fb = __MK_GetPublic_MotionControlFB(axisId);
    CHECK(fb != NULL, "Should expose allocated motion FB");

    memset(&ma, 0, sizeof(ma));
    IEC_VAL(ma.EN) = true;
    IEC_VAL(ma.AXISID) = axisId;
    IEC_VAL(ma.POSITION) = 400.0f;
    IEC_VAL(ma.VELOCITY) = 20.0f;
    IEC_VAL(ma.ACCELERATION) = 200.0f;
    IEC_VAL(ma.DECELERATION) = 200.0f;
    IEC_VAL(ma.DIRECTION) = 1;
    IEC_VAL(ma.BUFFERMODE) = 0;
    IEC_VAL(ma.EXECUTE) = true;
    ma.EXECUTE0.value = false;
    __mcl_cmd_MoveAbsolute(&ma);

    for (step = 0; step < 80; step++) {
        __HydMotion_framework_Publish();
        IEC_VAL(ma.EXECUTE) = true;
        ma.EXECUTE0.value = true;
        __mcl_cmd_MoveAbsolute(&ma);
    }

    CHECK(fabs(fb->AXIS_REF.velocity) > 1.0f, "MoveAbsolute should build non-zero velocity before Stop");

    memset(&stop, 0, sizeof(stop));
    IEC_VAL(stop.EN) = true;
    IEC_VAL(stop.EXECUTE) = true;
    stop.EXECUTE0.value = false;
    IEC_VAL(stop.AXISID) = axisId;
    IEC_VAL(stop.DECELERATION) = 50.0f;
    __mcl_cmd_Stop(&stop);

    CHECK(IEC_VAL(stop.DONE) == false, "Stop should not be done on the trigger call");

    for (step = 0; step < 5000; step++) {
        __HydMotion_framework_Publish();

        IEC_VAL(ma.EXECUTE) = true;
        ma.EXECUTE0.value = true;
        __mcl_cmd_MoveAbsolute(&ma);

        IEC_VAL(stop.EXECUTE) = true;
        stop.EXECUTE0.value = true;
        __mcl_cmd_Stop(&stop);

        if (IEC_VAL(stop.DONE)) {
            stopDoneCycle = step + 1;
            break;
        }
    }

    CHECK(stopDoneCycle > 5, "Stop should require multiple publish cycles to decelerate");
    CHECK(IEC_VAL(ma.COMMANDABORTED) == true, "MoveAbsolute should report COMMANDABORTED after Stop takeover");
    CHECK(fabs(fb->AXIS_REF.velocity) < 0.01f, "Velocity should be near zero when Stop.DONE becomes true");
}

static void test_stop_without_deceleration_uses_segment_max_deceleration(void) {
    HYD_MotionControlFB fb;
    HYD_MotionSegment segment;
    HYD_REAL firstStopVelocity;

    HYD_MotionControlFB_Init(&fb);
    fb.USE_RECIPE = false;
    fb.FLOW_TO_PUMP_SPEED_GAIN = 10.0;
    fb.PUMP_SPEED_LIMIT = 3000.0;
    fb.AXIS_REF.position = 0.0;
    fb.AXIS_REF.velocity = 20.0;
    fb.AXIS_REF.flow = 20.0;
    fb.AXIS_REF.pressure = 5.0;
    fb.AXIS_REF.timestamp = 0.0;

    memset(&segment, 0, sizeof(segment));
    segment.planner = HYD_PLANNER_TIME_BASED;
    segment.mode = HYD_MODE_SPEED_RAMP;
    segment.endCondition = HYD_END_MANUAL;
    segment.direction = HYD_DIRECTION_EXTEND;
    segment.maxVelocity = 50.0;
    segment.maxAcceleration = 100.0;
    segment.maxDeceleration = 5.0;
    segment.maxFlow = 100.0;
    segment.velocityToFlowGain = 1.0;

    assert(HYD_MotionControlFB_LoadDirectSegment(&fb, &segment));
    assert(HYD_MotionControlFB_StartSegment(&fb, 0, 0.0));

    fb.AXIS_REF.timestamp = 0.1;
    HYD_MotionControlFB_Cycle(&fb);

    assert(HYD_MotionControlFB_Stop(&fb, 0.1, 0.0));

    fb.AXIS_REF.timestamp = 0.2;
    fb.AXIS_REF.velocity = 20.0;
    HYD_MotionControlFB_Cycle(&fb);
    firstStopVelocity = fabs(fb.STATE.plannedVelocity);

    CHECK(fabs(firstStopVelocity - 19.5) < 0.01,
          "Stop without DECELERATION should use segment maxDeceleration");
}

int main(void) {
    printf("=== MoveAbsolute + Stop Integration ===\n");
    test_moveabsolute_stop_loop();
    test_stop_without_deceleration_uses_segment_max_deceleration();
    printf("=== Results: %d/%d passed ===\n", tests_passed, tests_run);
    return (tests_passed == tests_run) ? 0 : 1;
}
