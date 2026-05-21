/**
 * @file test_axismotion_setpoint_isolation.c
 * @brief HYD_AXISMOTION setpoint/actual 半区隔离测试 (Sprint 0 spec C-1)
 *
 * 验证 runtime 永远不会覆盖 PLC 拥有的 setpoint 字段。
 *
 * Setpoint 半区 (PLC -> runtime, runtime read-only):
 *   SEGMENTTAG, SEGMENTTYPE, PLANNER, MODE, ENDCONDITION, DIRECTION,
 *   SETPOSITION, SETVELOCITY, SETFLOW, SETPRESSURE, ACCELERATION,
 *   DECELERATION, DURATION, PRESSURERAMPRATE.
 *
 * Actual 半区 (runtime -> PLC, PLC read-only):
 *   ACTPOSITION, ACTVELOCITY, ACTFLOW, ACTPRESSURE, TIMESTAMP.
 *
 * 多 FB 共用同一物理 HYD_AXISMOTION 时，runtime 的反向回写会静默覆盖
 * 另一个 FB 排队的 setpoint。本测试用 sentinel 值锁住 runtime 不允许
 * 触碰 Setpoint 半区。
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

#define ASSERT_TRUE(cond, msg) do { \
    tests_run++; \
    if (cond) { tests_passed++; } \
    else { printf("  FAIL: %s\n", msg); } \
} while (0)

#define ASSERT_REAL_EQ(actual, expected, msg) do { \
    tests_run++; \
    if (fabs((double)(actual) - (double)(expected)) < 1e-5) { tests_passed++; } \
    else { printf("  FAIL: %s (actual=%g expected=%g)\n", \
                  msg, (double)(actual), (double)(expected)); } \
} while (0)

#define ASSERT_INT_EQ(actual, expected, msg) do { \
    tests_run++; \
    if ((long)(actual) == (long)(expected)) { tests_passed++; } \
    else { printf("  FAIL: %s (actual=%ld expected=%ld)\n", \
                  msg, (long)(actual), (long)(expected)); } \
} while (0)

/*
 * Test 1: PLC writes Setpoint half with sentinel values. After MoveProfile
 * runs two scan cycles (rising edge + steady-state), runtime MUST NOT have
 * overwritten any Setpoint field.
 *
 * Pre-fix expected: FAIL (writeMotionFromSegment writes SET*, ACCELERATION,
 * DECELERATION, DURATION, PRESSURERAMPRATE, SEGMENTTAG, SEGMENTTYPE,
 * PLANNER, MODE, ENDCONDITION, DIRECTION back from the active segment).
 *
 * Post-fix expected: PASS (writeMotionFromSegment is gone / restricted to
 * Actual half).
 */
static void test_runtime_does_not_overwrite_setpoint_fields(void) {
    HYD_CREATEMOTION cm;
    HYD_MOVEPROFILE mp;
    HYD_AXISMOTION sentinel;
    HYD_AXISMOTION readback;

    printf("Running: test_runtime_does_not_overwrite_setpoint_fields\n");

    __HydMotion_framework_Init();

    /* Allocate one recipe-mode axis, non-simulation. In non-sim mode the
     * runtime only reads MOTION.ACT* once per cycle and never writes them
     * back -- the only writes to MOTION come from writeMotionFromSegment. */
    memset(&cm, 0, sizeof(cm));
    IEC_VAL(cm.EN) = true;
    IEC_VAL(cm.USE_RECIPE) = true;
    IEC_VAL(cm.FLOW_TO_PUMPSPEED) = 1.0f;
    IEC_VAL(cm.PUMPSPEED_LIMIT) = 3000.0f;
    IEC_VAL(cm.USE_SIMULATION) = false;
    __mcl_cmd_CreateMotion(&cm);

    /* PLC stages a segment via MOTION setpoint fields with sentinel values
     * that the runtime should never produce on its own. */
    memset(&sentinel, 0, sizeof(sentinel));
    sentinel.SEGMENTTAG = 0xA5;
    sentinel.SEGMENTTYPE = HYD_SEGMENT_TYPE_INJECTION;
    sentinel.PLANNER = HYD_PLANNER_POSITION_BASED;
    sentinel.MODE = HYD_MODE_POSITION;
    sentinel.ENDCONDITION = HYD_END_POSITION;
    sentinel.DIRECTION = HYD_DIRECTION_EXTEND;
    sentinel.SETPOSITION = 123.456f;
    sentinel.SETVELOCITY = 78.9f;
    sentinel.SETFLOW = 12.34f;
    sentinel.SETPRESSURE = 9.876f;
    sentinel.ACCELERATION = 234.5f;
    sentinel.DECELERATION = 345.6f;
    sentinel.DURATION = 56.78f;
    sentinel.PRESSURERAMPRATE = 0.4321f;
    sentinel.ACTPOSITION = 5.0f;   /* Actual; runtime may overwrite */
    sentinel.ACTVELOCITY = 0.0f;
    sentinel.ACTFLOW = 0.0f;
    sentinel.ACTPRESSURE = 0.0f;
    sentinel.TIMESTAMP = 0.001f;

    memset(&mp, 0, sizeof(mp));
    IEC_VAL(mp.EN) = true;
    IEC_VAL(mp.EXECUTE) = true;
    mp.EXECUTE0.value = false;  /* rising edge */
    IEC_VAL(mp.AXISID) = IEC_VAL(cm.AXISID);
    IEC_VAL(mp.BUFFERMODE) = HYD_BUFFER_MODE_ABORT;
    __SET_VAR(mp., MOTION, , sentinel);

    /* Cycle 1: rising edge -- runtime builds segment from sentinel, starts. */
    __mcl_cmd_MoveProfile(&mp);
    __HydMotion_framework_Publish();

    /* Cycle 2: steady state -- this is when writeMotionFromSegment runs
     * (line 752-756 of motion_interface.c pre-fix). */
    IEC_VAL(mp.EXECUTE) = true;
    mp.EXECUTE0.value = true;
    __mcl_cmd_MoveProfile(&mp);

    /* PLC reads back -- Setpoint half MUST equal sentinel exactly. */
    readback = __GET_VAR(mp.MOTION);

    ASSERT_REAL_EQ(readback.SETPOSITION, 123.456f,
                   "SETPOSITION must not be overwritten by runtime");
    ASSERT_REAL_EQ(readback.SETVELOCITY, 78.9f,
                   "SETVELOCITY must not be overwritten by runtime");
    ASSERT_REAL_EQ(readback.SETFLOW, 12.34f,
                   "SETFLOW must not be overwritten by runtime");
    ASSERT_REAL_EQ(readback.SETPRESSURE, 9.876f,
                   "SETPRESSURE must not be overwritten by runtime");
    ASSERT_REAL_EQ(readback.ACCELERATION, 234.5f,
                   "ACCELERATION must not be overwritten by runtime");
    ASSERT_REAL_EQ(readback.DECELERATION, 345.6f,
                   "DECELERATION must not be overwritten by runtime");
    ASSERT_REAL_EQ(readback.DURATION, 56.78f,
                   "DURATION must not be overwritten by runtime");
    ASSERT_REAL_EQ(readback.PRESSURERAMPRATE, 0.4321f,
                   "PRESSURERAMPRATE must not be overwritten by runtime");
    ASSERT_INT_EQ(readback.SEGMENTTAG, 0xA5,
                  "SEGMENTTAG must not be overwritten by runtime");
    ASSERT_INT_EQ(readback.SEGMENTTYPE, HYD_SEGMENT_TYPE_INJECTION,
                  "SEGMENTTYPE must not be overwritten by runtime");
    ASSERT_INT_EQ(readback.PLANNER, HYD_PLANNER_POSITION_BASED,
                  "PLANNER must not be overwritten by runtime");
    ASSERT_INT_EQ(readback.MODE, HYD_MODE_POSITION,
                  "MODE must not be overwritten by runtime");
    ASSERT_INT_EQ(readback.ENDCONDITION, HYD_END_POSITION,
                  "ENDCONDITION must not be overwritten by runtime");
    ASSERT_INT_EQ(readback.DIRECTION, HYD_DIRECTION_EXTEND,
                  "DIRECTION must not be overwritten by runtime");
}

/*
 * Test 2: PLC stages a second segment via MOTION before the first has
 * completed (the classic multi-FB-per-axis race). Even though only one
 * FB is plumbed here, this verifies that calling MoveProfile again with
 * brand-new setpoint values does not see the previous values resurrected
 * by the runtime.
 */
static void test_runtime_preserves_pending_setpoint_across_cycles(void) {
    HYD_CREATEMOTION cm;
    HYD_MOVEPROFILE mp;
    HYD_AXISMOTION first;
    HYD_AXISMOTION second_stage;
    HYD_AXISMOTION readback;

    printf("Running: test_runtime_preserves_pending_setpoint_across_cycles\n");

    __HydMotion_framework_Init();

    memset(&cm, 0, sizeof(cm));
    IEC_VAL(cm.EN) = true;
    IEC_VAL(cm.USE_RECIPE) = true;
    IEC_VAL(cm.FLOW_TO_PUMPSPEED) = 1.0f;
    IEC_VAL(cm.PUMPSPEED_LIMIT) = 3000.0f;
    IEC_VAL(cm.USE_SIMULATION) = false;
    __mcl_cmd_CreateMotion(&cm);

    /* First segment is launched with one set of setpoint values. */
    memset(&first, 0, sizeof(first));
    first.MODE = HYD_MODE_POSITION;
    first.ENDCONDITION = HYD_END_POSITION;
    first.DIRECTION = HYD_DIRECTION_EXTEND;
    first.SETPOSITION = 100.0f;
    first.SETVELOCITY = 40.0f;
    first.SETFLOW = 10.0f;
    first.ACCELERATION = 150.0f;
    first.DECELERATION = 150.0f;
    first.TIMESTAMP = 0.0f;

    memset(&mp, 0, sizeof(mp));
    IEC_VAL(mp.EN) = true;
    IEC_VAL(mp.EXECUTE) = true;
    mp.EXECUTE0.value = false;
    IEC_VAL(mp.AXISID) = IEC_VAL(cm.AXISID);
    IEC_VAL(mp.BUFFERMODE) = HYD_BUFFER_MODE_ABORT;
    __SET_VAR(mp., MOTION, , first);

    __mcl_cmd_MoveProfile(&mp);
    __HydMotion_framework_Publish();

    /* PLC now stages a different second segment by writing fresh values
     * into MOTION between cycles. The runtime should leave these alone. */
    memset(&second_stage, 0, sizeof(second_stage));
    second_stage.MODE = HYD_MODE_SPEED_RAMP;
    second_stage.ENDCONDITION = HYD_END_TIME;
    second_stage.DIRECTION = HYD_DIRECTION_RETRACT;
    second_stage.SETPOSITION = 999.0f;
    second_stage.SETVELOCITY = 22.0f;
    second_stage.SETFLOW = 7.5f;
    second_stage.SETPRESSURE = 4.25f;
    second_stage.ACCELERATION = 999.0f;
    second_stage.DECELERATION = 888.0f;
    second_stage.DURATION = 1.5f;
    second_stage.PRESSURERAMPRATE = 2.5f;
    second_stage.SEGMENTTAG = 0x42;
    second_stage.SEGMENTTYPE = HYD_SEGMENT_TYPE_HOLDING;
    second_stage.PLANNER = HYD_PLANNER_TIME_BASED;
    second_stage.TIMESTAMP = 0.002f;
    __SET_VAR(mp., MOTION, , second_stage);

    /* Continue running -- runtime should not touch the staged values. */
    IEC_VAL(mp.EXECUTE) = true;
    mp.EXECUTE0.value = true;
    __mcl_cmd_MoveProfile(&mp);

    readback = __GET_VAR(mp.MOTION);

    ASSERT_REAL_EQ(readback.SETPOSITION, 999.0f,
                   "Staged SETPOSITION must survive a runtime cycle");
    ASSERT_REAL_EQ(readback.SETVELOCITY, 22.0f,
                   "Staged SETVELOCITY must survive a runtime cycle");
    ASSERT_REAL_EQ(readback.SETFLOW, 7.5f,
                   "Staged SETFLOW must survive a runtime cycle");
    ASSERT_REAL_EQ(readback.SETPRESSURE, 4.25f,
                   "Staged SETPRESSURE must survive a runtime cycle");
    ASSERT_REAL_EQ(readback.ACCELERATION, 999.0f,
                   "Staged ACCELERATION must survive a runtime cycle");
    ASSERT_REAL_EQ(readback.DECELERATION, 888.0f,
                   "Staged DECELERATION must survive a runtime cycle");
    ASSERT_REAL_EQ(readback.DURATION, 1.5f,
                   "Staged DURATION must survive a runtime cycle");
    ASSERT_REAL_EQ(readback.PRESSURERAMPRATE, 2.5f,
                   "Staged PRESSURERAMPRATE must survive a runtime cycle");
    ASSERT_INT_EQ(readback.SEGMENTTAG, 0x42,
                  "Staged SEGMENTTAG must survive a runtime cycle");
    ASSERT_INT_EQ(readback.SEGMENTTYPE, HYD_SEGMENT_TYPE_HOLDING,
                  "Staged SEGMENTTYPE must survive a runtime cycle");
    ASSERT_INT_EQ(readback.MODE, HYD_MODE_SPEED_RAMP,
                  "Staged MODE must survive a runtime cycle");
    ASSERT_INT_EQ(readback.ENDCONDITION, HYD_END_TIME,
                  "Staged ENDCONDITION must survive a runtime cycle");
    ASSERT_INT_EQ(readback.DIRECTION, HYD_DIRECTION_RETRACT,
                  "Staged DIRECTION must survive a runtime cycle");
}

int main(void) {
    test_runtime_does_not_overwrite_setpoint_fields();
    test_runtime_preserves_pending_setpoint_across_cycles();

    printf("\n[axismotion_setpoint_isolation] %d/%d assertions passed\n",
           tests_passed, tests_run);
    return (tests_run == tests_passed) ? 0 : 1;
}
