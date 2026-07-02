/**
 * @file test_motion_interface_arbitration.c
 * @brief IEC FB接口层命令仲裁测试
 *
 * 目标：验证基于 executionId 的多命令抢占与
 *       COMMANDABORTED 检测机制，包括跨 FB 抢占、多轴隔离和
 *       所有权边界条件。
 *
 * 仲裁规则:
 *   - MoveProfile (Recipe模式) 与 Direct模式命令互斥
 *   - 新命令可以接管正在执行的旧命令
 *   - 被接管的命令输出 COMMANDABORTED=true, BUSY=false
 *   - 不同轴之间的命令互不影响
 */

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdbool.h>

#include "motion_interface.h"
#include "motion_control.h"
#include "state_reporter.h"
#include "test_recipe_rejection_helpers.h"

extern HYD_MotionControlFB* __MK_GetPublic_MotionControlFB(int index);

#define IEC_VAL(var) ((var).value)

#define STOP_WAIT_BUDGET 5000

static int tests_run = 0;
static int tests_passed = 0;

#define ASSERT_TRUE(cond, msg) do { \
    tests_run++; \
    if (cond) { tests_passed++; } \
    else { printf("  FAIL: %s\n", msg); } \
} while (0)

/* 辅助: 通过CreateMotion分配指定数量的轴 */
static void ensure_axes_allocated(int count) {
    for (int i = 0; i < count; i++) {
        HYD_CREATEMOTION cm;
        memset(&cm, 0, sizeof(cm));
        IEC_VAL(cm.EN) = true;
        IEC_VAL(cm.USE_RECIPE) = false;
        IEC_VAL(cm.FLOW_TO_PUMPSPEED) = 1.2f;
        IEC_VAL(cm.PUMPSPEED_LIMIT) = 3000.0f;
        IEC_VAL(cm.USE_SIMULATION) = false;
        __mcl_cmd_CreateMotion(&cm);
    }
}

static void advance_non_sim_feedback(int axisIndex, HYD_REAL deltaTime) {
    HYD_MotionControlFB* fb = __MK_GetPublic_MotionControlFB(axisIndex);

    if (fb == NULL) {
        return;
    }

    fb->AXIS_REF.timestamp += deltaTime;
    if (fb->_simFeedback.valid) {
        fb->AXIS_REF.velocity = fb->_simFeedback.targetVelocity;
        fb->AXIS_REF.position += fb->AXIS_REF.velocity * deltaTime;
        fb->AXIS_REF.flow = fb->_simFeedback.targetFlow;
        fb->AXIS_REF.pressure = fb->_simFeedback.targetPressure;
    }
}

/* 辅助: 在指定轴上启动 MoveAbsolute 并经过一次Publish */
static void start_moveabsolute_on_axis(int axisIndex, HYD_MOVEABSOLUTE* ma) {
    memset(ma, 0, sizeof(*ma));
    IEC_VAL(ma->EN) = true;
    IEC_VAL(ma->EXECUTE) = true;
    ma->EXECUTE0.value = false;  /* 上升沿 */
    IEC_VAL(ma->AXISID) = axisIndex;
    IEC_VAL(ma->POSITION) = 100.0f;
    IEC_VAL(ma->VELOCITY) = 50.0f;
    IEC_VAL(ma->ACCELERATION) = 200.0f;
    IEC_VAL(ma->DIRECTION) = 1;

    __mcl_cmd_MoveAbsolute(ma);
    __HydMotion_framework_Publish();

    /* 下一周期: EXECUTE保持true, 非上升沿 */
    IEC_VAL(ma->EXECUTE) = true;
    ma->EXECUTE0.value = true;
    __mcl_cmd_MoveAbsolute(ma);

    for (int step = 0; step < 5; step++) {
        advance_non_sim_feedback(axisIndex, 0.01f);
        __HydMotion_framework_Publish();
        IEC_VAL(ma->EXECUTE) = true;
        ma->EXECUTE0.value = true;
        __mcl_cmd_MoveAbsolute(ma);
    }
}

/* 辅助: 在指定轴上启动 MoveVelocity 并经过一次Publish */
static void start_movevelocity_on_axis(int axisIndex, HYD_MOVEVELOCITY* mv) {
    memset(mv, 0, sizeof(*mv));
    IEC_VAL(mv->EN) = true;
    IEC_VAL(mv->EXECUTE) = true;
    mv->EXECUTE0.value = false;  /* 上升沿 */
    IEC_VAL(mv->AXISID) = axisIndex;
    IEC_VAL(mv->VELOCITY) = 30.0f;
    IEC_VAL(mv->ACCELERATION) = 150.0f;
    IEC_VAL(mv->DIRECTION) = 1;

    __mcl_cmd_MoveVelocity(mv);
    __HydMotion_framework_Publish();

    /* 下一周期 */
    IEC_VAL(mv->EXECUTE) = true;
    mv->EXECUTE0.value = true;
    __mcl_cmd_MoveVelocity(mv);

    for (int step = 0; step < 5; step++) {
        advance_non_sim_feedback(axisIndex, 0.01f);
        __HydMotion_framework_Publish();
        IEC_VAL(mv->EXECUTE) = true;
        mv->EXECUTE0.value = true;
        __mcl_cmd_MoveVelocity(mv);
    }
}

static HYD_MotionControlFB* start_blend_pair(HYD_BufferMode mode,
                                             HYD_REAL firstVelocity,
                                             HYD_REAL secondVelocity) {
    HYD_MOVEABSOLUTE first;
    HYD_MOVEABSOLUTE second;
    HYD_MotionControlFB* fb;

    __HydMotion_framework_Init();
    ensure_axes_allocated(1);
    fb = __MK_GetPublic_MotionControlFB(0);
    ASSERT_TRUE(fb != NULL, "Axis 0 control FB should exist");

    memset(&first, 0, sizeof(first));
    IEC_VAL(first.EN) = true;
    IEC_VAL(first.EXECUTE) = true;
    first.EXECUTE0.value = false;
    IEC_VAL(first.AXISID) = 0;
    IEC_VAL(first.POSITION) = 100.0f;
    IEC_VAL(first.VELOCITY) = firstVelocity;
    IEC_VAL(first.ACCELERATION) = 100.0f;
    IEC_VAL(first.DECELERATION) = 100.0f;
    IEC_VAL(first.DIRECTION) = 1;
    IEC_VAL(first.BUFFERMODE) = HYD_BUFFER_MODE_ABORT;
    __mcl_cmd_MoveAbsolute(&first);
    __HydMotion_framework_Publish();

    memset(&second, 0, sizeof(second));
    IEC_VAL(second.EN) = true;
    IEC_VAL(second.EXECUTE) = true;
    second.EXECUTE0.value = false;
    IEC_VAL(second.AXISID) = 0;
    IEC_VAL(second.POSITION) = 200.0f;
    IEC_VAL(second.VELOCITY) = secondVelocity;
    IEC_VAL(second.ACCELERATION) = 100.0f;
    IEC_VAL(second.DECELERATION) = 100.0f;
    IEC_VAL(second.DIRECTION) = 1;
    IEC_VAL(second.BUFFERMODE) = mode;
    __mcl_cmd_MoveAbsolute(&second);

    return fb;
}

static int drive_stop_until_done(int axisIndex, HYD_STOP* stop, int maxSteps) {
    HYD_MotionControlFB* fb = __MK_GetPublic_MotionControlFB(axisIndex);

    if (fb == NULL) {
        return -1;
    }

    for (int step = 0; step < maxSteps; step++) {
        advance_non_sim_feedback(axisIndex, 0.01f);

        __HydMotion_framework_Publish();
        IEC_VAL(stop->EXECUTE) = true;
        stop->EXECUTE0.value = true;
        __mcl_cmd_Stop(stop);

        if (IEC_VAL(stop->DONE)) {
            return step + 1;
        }
    }

    return -1;
}

/* ==================================================================
 * Test 1: MoveAbsolute 被 Stop 抢占 → COMMANDABORTED
 * ================================================================== */
static void test_moveabsolute_preempted_by_stop(void) {
    HYD_MOVEABSOLUTE ma;
    HYD_STOP stop;
    int stopDoneStep = -1;

    __HydMotion_framework_Init();
    ensure_axes_allocated(2);

    /* Step 1: 启动 MoveAbsolute */
    start_moveabsolute_on_axis(0, &ma);
    ASSERT_TRUE(IEC_VAL(ma.ACTIVE) || IEC_VAL(ma.BUSY),
               "MoveAbsolute should be active before preemption");

    /* Step 2: Stop 接管同一轴 */
    memset(&stop, 0, sizeof(stop));
    IEC_VAL(stop.EN) = true;
    IEC_VAL(stop.EXECUTE) = true;
    stop.EXECUTE0.value = false;
    IEC_VAL(stop.AXISID) = 0;
    __mcl_cmd_Stop(&stop);
    ASSERT_TRUE(IEC_VAL(stop.DONE) == false,
               "Stop should not complete on the trigger call");
    advance_non_sim_feedback(0, 0.01f);
    __HydMotion_framework_Publish();

    /* 下一周期Stop继续 */
    IEC_VAL(stop.EXECUTE) = true;
    stop.EXECUTE0.value = true;
    __mcl_cmd_Stop(&stop);
    ASSERT_TRUE(IEC_VAL(stop.DONE) == false,
               "Stop should still be pending while decelerating active motion");

    /* Step 3: MoveAbsolute 应检测到被抢占 */
    IEC_VAL(ma.EXECUTE) = true;
    ma.EXECUTE0.value = true;
    __mcl_cmd_MoveAbsolute(&ma);

    ASSERT_TRUE(IEC_VAL(ma.COMMANDABORTED) == true,
               "MoveAbsolute should raise COMMANDABORTED when preempted by Stop");
    ASSERT_TRUE(IEC_VAL(ma.BUSY) == false,
               "MoveAbsolute BUSY should be false after preemption");

    stopDoneStep = drive_stop_until_done(0, &stop, STOP_WAIT_BUDGET);

    ASSERT_TRUE(stopDoneStep >= 1,
               "Stop should complete after the already-verified pending cycle");
    ASSERT_TRUE(IEC_VAL(stop.BUSY) == false,
               "Stop BUSY should be false once DONE");
}

static void test_buffered_moveabsolute_reports_busy_but_not_active_while_waiting(void) {
    HYD_MOVEABSOLUTE first;
    HYD_MOVEABSOLUTE second;

    __HydMotion_framework_Init();
    ensure_axes_allocated(1);

    start_moveabsolute_on_axis(0, &first);
    ASSERT_TRUE(IEC_VAL(first.ACTIVE) || IEC_VAL(first.BUSY),
               "First MoveAbsolute should be running before the buffered follower starts");

    memset(&second, 0, sizeof(second));
    IEC_VAL(second.EN) = true;
    IEC_VAL(second.EXECUTE) = true;
    second.EXECUTE0.value = false;
    IEC_VAL(second.AXISID) = 0;
    IEC_VAL(second.POSITION) = 200.0f;
    IEC_VAL(second.VELOCITY) = 40.0f;
    IEC_VAL(second.ACCELERATION) = 100.0f;
    IEC_VAL(second.DECELERATION) = 100.0f;
    IEC_VAL(second.DIRECTION) = 1;
    IEC_VAL(second.BUFFERMODE) = HYD_BUFFER_MODE_BLENDING_HIGH;
    __mcl_cmd_MoveAbsolute(&second);

    ASSERT_TRUE(IEC_VAL(second.BUSY) == true,
               "Buffered MoveAbsolute should be BUSY while waiting");
    ASSERT_TRUE(IEC_VAL(second.ACTIVE) == false,
               "Buffered MoveAbsolute must not be ACTIVE while another MoveAbsolute owns the axis");
    ASSERT_TRUE(IEC_VAL(second.DONE) == false,
               "Buffered MoveAbsolute should not report DONE while waiting");
    ASSERT_TRUE(IEC_VAL(second.COMMANDABORTED) == false,
               "Buffered MoveAbsolute should not report COMMANDABORTED while waiting");
}

static void test_third_same_axis_moveabsolute_is_rejected_when_one_active_and_one_pending_exist(void) {
    HYD_MOVEABSOLUTE first;
    HYD_MOVEABSOLUTE second;
    HYD_MOVEABSOLUTE third;
    HYD_MotionControlFB* fb;

    __HydMotion_framework_Init();
    ensure_axes_allocated(1);

    start_moveabsolute_on_axis(0, &first);

    memset(&second, 0, sizeof(second));
    IEC_VAL(second.EN) = true;
    IEC_VAL(second.EXECUTE) = true;
    second.EXECUTE0.value = false;
    IEC_VAL(second.AXISID) = 0;
    IEC_VAL(second.POSITION) = 200.0f;
    IEC_VAL(second.VELOCITY) = 40.0f;
    IEC_VAL(second.ACCELERATION) = 100.0f;
    IEC_VAL(second.DECELERATION) = 100.0f;
    IEC_VAL(second.DIRECTION) = 1;
    IEC_VAL(second.BUFFERMODE) = HYD_BUFFER_MODE_BLENDING_HIGH;
    __mcl_cmd_MoveAbsolute(&second);

    ASSERT_TRUE(IEC_VAL(second.ERROR) == false,
               "Second MoveAbsolute should occupy the single pending slot");

    memset(&third, 0, sizeof(third));
    IEC_VAL(third.EN) = true;
    IEC_VAL(third.EXECUTE) = true;
    third.EXECUTE0.value = false;
    IEC_VAL(third.AXISID) = 0;
    IEC_VAL(third.POSITION) = 300.0f;
    IEC_VAL(third.VELOCITY) = 60.0f;
    IEC_VAL(third.ACCELERATION) = 100.0f;
    IEC_VAL(third.DECELERATION) = 100.0f;
    IEC_VAL(third.DIRECTION) = 1;
    IEC_VAL(third.BUFFERMODE) = HYD_BUFFER_MODE_BLENDING_LOW;
    __mcl_cmd_MoveAbsolute(&third);

    ASSERT_TRUE(IEC_VAL(third.ERROR) == true,
               "Third same-axis MoveAbsolute should be rejected when one active and one pending command already exist");
    ASSERT_TRUE(IEC_VAL(third.BUSY) == false,
               "Rejected third MoveAbsolute should not enter BUSY");
    ASSERT_TRUE(IEC_VAL(third.ACTIVE) == false,
               "Rejected third MoveAbsolute should not enter ACTIVE");

    fb = __MK_GetPublic_MotionControlFB(0);
    ASSERT_TRUE(fb != NULL, "Axis 0 control FB should exist for pending-slot invariant checks");
    if (fb != NULL) {
        ASSERT_TRUE(fb->_directPendingValid == true,
                   "Rejected third MoveAbsolute should leave the single pending slot occupied");
        ASSERT_TRUE(fabs(fb->_directPendingSegment.targetPosition - 200.0f) < 0.001f,
                   "Rejected third MoveAbsolute should not overwrite the existing pending target");
        ASSERT_TRUE(fb->_directBlendContext.active == true,
                   "Rejected third MoveAbsolute should not clear the active blend context");
    }

    ASSERT_TRUE(IEC_VAL(third.ERRORID) == (IEC_WORD)HYD_DIAG_CODE_BUFFER_FULL,
               "Rejected third MoveAbsolute should report BUFFER_FULL");
    ASSERT_TRUE(IEC_VAL(third.COMMANDABORTED) == false,
               "Rejected third MoveAbsolute should not report COMMANDABORTED");
    ASSERT_TRUE(IEC_VAL(second.COMMANDABORTED) == false,
               "Buffered second MoveAbsolute should remain non-aborted after rejecting the third");
}

static void test_rejected_third_moveabsolute_stays_local_under_persistent_execute_high(void) {
    HYD_MOVEABSOLUTE first;
    HYD_MOVEABSOLUTE second;
    HYD_MOVEABSOLUTE third;
    HYD_MotionControlFB* fb;

    __HydMotion_framework_Init();
    ensure_axes_allocated(1);

    start_moveabsolute_on_axis(0, &first);

    memset(&second, 0, sizeof(second));
    IEC_VAL(second.EN) = true;
    IEC_VAL(second.EXECUTE) = true;
    second.EXECUTE0.value = false;
    IEC_VAL(second.AXISID) = 0;
    IEC_VAL(second.POSITION) = 200.0f;
    IEC_VAL(second.VELOCITY) = 40.0f;
    IEC_VAL(second.ACCELERATION) = 100.0f;
    IEC_VAL(second.DECELERATION) = 100.0f;
    IEC_VAL(second.DIRECTION) = 1;
    IEC_VAL(second.BUFFERMODE) = HYD_BUFFER_MODE_BLENDING_HIGH;
    __mcl_cmd_MoveAbsolute(&second);

    memset(&third, 0, sizeof(third));
    IEC_VAL(third.EN) = true;
    IEC_VAL(third.EXECUTE) = true;
    third.EXECUTE0.value = false;
    IEC_VAL(third.AXISID) = 0;
    IEC_VAL(third.POSITION) = 300.0f;
    IEC_VAL(third.VELOCITY) = 60.0f;
    IEC_VAL(third.ACCELERATION) = 100.0f;
    IEC_VAL(third.DECELERATION) = 100.0f;
    IEC_VAL(third.DIRECTION) = 1;
    IEC_VAL(third.BUFFERMODE) = HYD_BUFFER_MODE_BLENDING_HIGH;
    __mcl_cmd_MoveAbsolute(&third);

    fb = __MK_GetPublic_MotionControlFB(0);
    ASSERT_TRUE(fb != NULL, "Axis 0 control FB should exist for persistent-high rejection checks");
    if (fb == NULL) {
        return;
    }

    for (int step = 0; step < 3; step++) {
        IEC_VAL(first.EXECUTE) = true;
        first.EXECUTE0.value = true;
        __mcl_cmd_MoveAbsolute(&first);

        IEC_VAL(second.EXECUTE) = true;
        second.EXECUTE0.value = true;
        __mcl_cmd_MoveAbsolute(&second);

        IEC_VAL(third.EXECUTE) = true;
        third.EXECUTE0.value = true;
        __mcl_cmd_MoveAbsolute(&third);

        ASSERT_TRUE(IEC_VAL(third.BUSY) == false,
                   "Rejected third MoveAbsolute should remain non-busy while EXECUTE stays high");
        ASSERT_TRUE(IEC_VAL(third.ACTIVE) == false,
                   "Rejected third MoveAbsolute should remain inactive while EXECUTE stays high");
        ASSERT_TRUE(IEC_VAL(third.ERROR) == true,
                   "Rejected third MoveAbsolute should keep ERROR latched while EXECUTE stays high");
        ASSERT_TRUE(IEC_VAL(third.ERRORID) == (IEC_WORD)HYD_DIAG_CODE_BUFFER_FULL,
                   "Rejected third MoveAbsolute should keep BUFFER_FULL latched while EXECUTE stays high");
        ASSERT_TRUE(IEC_VAL(third.COMMANDABORTED) == false,
                   "Rejected third MoveAbsolute should not mutate into COMMANDABORTED on later scans");
        ASSERT_TRUE(fb->_directPendingValid == true,
                   "Rejected third MoveAbsolute should not dislodge the accepted pending command on later scans");

        advance_non_sim_feedback(0, 0.01f);
        __HydMotion_framework_Publish();
    }
}

/* ==================================================================
 * Test 2: MoveAbsolute 被 MoveVelocity 抢占 → COMMANDABORTED
 * ================================================================== */
static void test_moveabsolute_preempted_by_movevelocity(void) {
    HYD_MOVEABSOLUTE ma;
    HYD_MOVEVELOCITY mv;

    __HydMotion_framework_Init();
    ensure_axes_allocated(2);

    /* Step 1: 启动 MoveAbsolute */
    start_moveabsolute_on_axis(0, &ma);
    ASSERT_TRUE(IEC_VAL(ma.ACTIVE) || IEC_VAL(ma.BUSY),
               "MoveAbsolute should be active before preemption");

    /* Step 2: MoveVelocity 接管同一轴 */
    memset(&mv, 0, sizeof(mv));
    IEC_VAL(mv.EN) = true;
    IEC_VAL(mv.EXECUTE) = true;
    mv.EXECUTE0.value = false;
    IEC_VAL(mv.AXISID) = 0;
    IEC_VAL(mv.VELOCITY) = 30.0f;
    IEC_VAL(mv.ACCELERATION) = 150.0f;
    IEC_VAL(mv.DIRECTION) = 1;
    __mcl_cmd_MoveVelocity(&mv);
    __HydMotion_framework_Publish();

    /* Step 3: MoveAbsolute 应检测到被抢占 */
    IEC_VAL(ma.EXECUTE) = true;
    ma.EXECUTE0.value = true;
    __mcl_cmd_MoveAbsolute(&ma);

    ASSERT_TRUE(IEC_VAL(ma.COMMANDABORTED) == true,
               "MoveAbsolute should raise COMMANDABORTED when preempted by MoveVelocity");
    ASSERT_TRUE(IEC_VAL(ma.ACTIVE) == false,
               "MoveAbsolute ACTIVE should be false after preemption");
    ASSERT_TRUE(IEC_VAL(ma.DONE) == false,
               "MoveAbsolute DONE should NOT be true after preemption (not normal completion)");
}

/* ==================================================================
 * Test 3: MoveVelocity 被 MoveAbsolute 抢占 → COMMANDABORTED
 * ================================================================== */
static void test_movevelocity_preempted_by_moveabsolute(void) {
    HYD_MOVEVELOCITY mv;
    HYD_MOVEABSOLUTE ma;

    __HydMotion_framework_Init();
    ensure_axes_allocated(2);

    /* Step 1: 启动 MoveVelocity */
    start_movevelocity_on_axis(0, &mv);
    ASSERT_TRUE(IEC_VAL(mv.ACTIVE) || IEC_VAL(mv.BUSY),
               "MoveVelocity should be active before preemption");

    /* Step 2: MoveAbsolute 接管同一轴 */
    memset(&ma, 0, sizeof(ma));
    IEC_VAL(ma.EN) = true;
    IEC_VAL(ma.EXECUTE) = true;
    ma.EXECUTE0.value = false;
    IEC_VAL(ma.AXISID) = 0;
    IEC_VAL(ma.POSITION) = 150.0f;
    IEC_VAL(ma.VELOCITY) = 60.0f;
    IEC_VAL(ma.ACCELERATION) = 200.0f;
    IEC_VAL(ma.DIRECTION) = 1;
    __mcl_cmd_MoveAbsolute(&ma);
    __HydMotion_framework_Publish();

    /* Step 3: MoveVelocity 应检测到被抢占 */
    IEC_VAL(mv.EXECUTE) = true;
    mv.EXECUTE0.value = true;
    __mcl_cmd_MoveVelocity(&mv);

    ASSERT_TRUE(IEC_VAL(mv.COMMANDABORTED) == true,
               "MoveVelocity should raise COMMANDABORTED when preempted by MoveAbsolute");
    ASSERT_TRUE(IEC_VAL(mv.BUSY) == false,
               "MoveVelocity BUSY should be false after preemption");
    ASSERT_TRUE(IEC_VAL(mv.INVELOCITY) == false,
               "MoveVelocity INVELOCITY should be false after preemption");
}

/* ==================================================================
 * Test 4: PressureHandle 被 Stop 抢占 → COMMANDABORTED
 * ================================================================== */
static void test_pressurehandle_preempted_by_stop(void) {
    HYD_PRESSUREHANDLE ph;
    HYD_STOP stop;

    __HydMotion_framework_Init();
    ensure_axes_allocated(2);

    /* Step 1: 启动 PressureHandle */
    memset(&ph, 0, sizeof(ph));
    IEC_VAL(ph.EN) = true;
    IEC_VAL(ph.EXECUTE) = true;
    ph.EXECUTE0.value = false;
    IEC_VAL(ph.AXISID) = 0;
    IEC_VAL(ph.PRESSURE) = 10.0f;
    IEC_VAL(ph.PRESSURERAMPRATE) = 2.0f;
    __mcl_cmd_PressureHandle(&ph);
    __HydMotion_framework_Publish();

    IEC_VAL(ph.EXECUTE) = true;
    ph.EXECUTE0.value = true;
    __mcl_cmd_PressureHandle(&ph);
    ASSERT_TRUE(IEC_VAL(ph.ACTIVE) || IEC_VAL(ph.BUSY),
               "PressureHandle should be active before preemption");

    /* Step 2: Stop 接管 */
    memset(&stop, 0, sizeof(stop));
    IEC_VAL(stop.EN) = true;
    IEC_VAL(stop.EXECUTE) = true;
    stop.EXECUTE0.value = false;
    IEC_VAL(stop.AXISID) = 0;
    __mcl_cmd_Stop(&stop);
    ASSERT_TRUE(IEC_VAL(stop.DONE) == false,
               "Axis 0 Stop should not complete on the trigger call");
    __HydMotion_framework_Publish();

    /* Step 3: PressureHandle 应检测到被抢占 */
    IEC_VAL(ph.EXECUTE) = true;
    ph.EXECUTE0.value = true;
    __mcl_cmd_PressureHandle(&ph);

    ASSERT_TRUE(IEC_VAL(ph.COMMANDABORTED) == true,
               "PressureHandle should raise COMMANDABORTED when preempted by Stop");
    ASSERT_TRUE(IEC_VAL(ph.ACTIVE) == false,
               "PressureHandle ACTIVE should be false after preemption");
    ASSERT_TRUE(IEC_VAL(ph.INPRESSURE) == false,
               "PressureHandle INPRESSURE should clear after Stop takeover");
}

/* ==================================================================
 * Test 5: 多轴隔离 — 轴0的命令不影响轴1
 * ================================================================== */
static void test_movevelocity_preempted_by_pressurehandle(void) {
    HYD_MOVEVELOCITY mv;
    HYD_PRESSUREHANDLE ph;

    __HydMotion_framework_Init();
    ensure_axes_allocated(2);

    memset(&mv, 0, sizeof(mv));
    IEC_VAL(mv.EN) = true;
    IEC_VAL(mv.EXECUTE) = true;
    mv.EXECUTE0.value = false;
    IEC_VAL(mv.AXISID) = 0;
    IEC_VAL(mv.VELOCITY) = 30.0f;
    IEC_VAL(mv.ACCELERATION) = 150.0f;
    IEC_VAL(mv.DIRECTION) = 1;
    __mcl_cmd_MoveVelocity(&mv);
    __HydMotion_framework_Publish();

    IEC_VAL(mv.EXECUTE) = true;
    mv.EXECUTE0.value = true;
    __mcl_cmd_MoveVelocity(&mv);

    memset(&ph, 0, sizeof(ph));
    IEC_VAL(ph.EN) = true;
    IEC_VAL(ph.EXECUTE) = true;
    ph.EXECUTE0.value = false;
    IEC_VAL(ph.AXISID) = 0;
    IEC_VAL(ph.PRESSURE) = 5.0f;
    IEC_VAL(ph.PRESSURERAMPRATE) = 10.0f;
    IEC_VAL(ph.DURATION) = 0.5f;
    __mcl_cmd_PressureHandle(&ph);
    __HydMotion_framework_Publish();

    IEC_VAL(mv.EXECUTE) = true;
    mv.EXECUTE0.value = true;
    __mcl_cmd_MoveVelocity(&mv);

    ASSERT_TRUE(IEC_VAL(mv.COMMANDABORTED) == true,
               "MoveVelocity should raise COMMANDABORTED when preempted by PressureHandle");
    ASSERT_TRUE(IEC_VAL(mv.INVELOCITY) == false,
               "MoveVelocity INVELOCITY should clear after PressureHandle takeover");
}

/* ==================================================================
 * Test 5: 多轴隔离 — 轴0的命令不影响轴1
 * ================================================================== */
static void test_multi_axis_isolation(void) {
    HYD_MOVEABSOLUTE ma0, ma1;
    HYD_STOP stop;

    __HydMotion_framework_Init();
    ensure_axes_allocated(2);

    /* 轴0: 启动 MoveAbsolute */
    memset(&ma0, 0, sizeof(ma0));
    IEC_VAL(ma0.EN) = true;
    IEC_VAL(ma0.EXECUTE) = true;
    ma0.EXECUTE0.value = false;
    IEC_VAL(ma0.AXISID) = 0;
    IEC_VAL(ma0.POSITION) = 100.0f;
    IEC_VAL(ma0.VELOCITY) = 50.0f;
    IEC_VAL(ma0.ACCELERATION) = 200.0f;
    IEC_VAL(ma0.DIRECTION) = 1;
    __mcl_cmd_MoveAbsolute(&ma0);
    __HydMotion_framework_Publish();

    IEC_VAL(ma0.EXECUTE) = true;
    ma0.EXECUTE0.value = true;
    __mcl_cmd_MoveAbsolute(&ma0);
    ASSERT_TRUE(IEC_VAL(ma0.ACTIVE) || IEC_VAL(ma0.BUSY),
               "Axis 0 MoveAbsolute should be active");

    /* 轴1: 独立启动 MoveAbsolute */
    memset(&ma1, 0, sizeof(ma1));
    IEC_VAL(ma1.EN) = true;
    IEC_VAL(ma1.EXECUTE) = true;
    ma1.EXECUTE0.value = false;
    IEC_VAL(ma1.AXISID) = 1;
    IEC_VAL(ma1.POSITION) = 200.0f;
    IEC_VAL(ma1.VELOCITY) = 30.0f;
    IEC_VAL(ma1.ACCELERATION) = 150.0f;
    IEC_VAL(ma1.DIRECTION) = 1;
    __mcl_cmd_MoveAbsolute(&ma1);
    __HydMotion_framework_Publish();

    IEC_VAL(ma1.EXECUTE) = true;
    ma1.EXECUTE0.value = true;
    __mcl_cmd_MoveAbsolute(&ma1);

    /* 轴0 不应被轴1 影响 */
    IEC_VAL(ma0.EXECUTE) = true;
    ma0.EXECUTE0.value = true;
    __mcl_cmd_MoveAbsolute(&ma0);

    ASSERT_TRUE(IEC_VAL(ma0.COMMANDABORTED) == false,
               "Axis 0 should NOT get COMMANDABORTED from Axis 1 activity");

    /* 轴1 应该正常运行 */
    ASSERT_TRUE(IEC_VAL(ma1.COMMANDABORTED) == false,
               "Axis 1 should NOT get COMMANDABORTED from its own command");

    /* 现在在轴0上执行Stop, 轴1不应受影响 */
    memset(&stop, 0, sizeof(stop));
    IEC_VAL(stop.EN) = true;
    IEC_VAL(stop.EXECUTE) = true;
    stop.EXECUTE0.value = false;
    IEC_VAL(stop.AXISID) = 0;
    __mcl_cmd_Stop(&stop);
    advance_non_sim_feedback(0, 0.01f);
    __HydMotion_framework_Publish();

    /* 轴0被抢占 */
    IEC_VAL(ma0.EXECUTE) = true;
    ma0.EXECUTE0.value = true;
    __mcl_cmd_MoveAbsolute(&ma0);
    ASSERT_TRUE(IEC_VAL(ma0.COMMANDABORTED) == true,
               "Axis 0 should get COMMANDABORTED from Axis 0 Stop");

    /* 轴1不应受影响 */
    IEC_VAL(ma1.EXECUTE) = true;
    ma1.EXECUTE0.value = true;
    __mcl_cmd_MoveAbsolute(&ma1);
    ASSERT_TRUE(IEC_VAL(ma1.COMMANDABORTED) == false,
               "Axis 1 should NOT get COMMANDABORTED from Axis 0 Stop");
    ASSERT_TRUE(IEC_VAL(ma1.ACTIVE) || IEC_VAL(ma1.BUSY),
               "Axis 1 should still be active after Axis 0 Stop");
}

/* ==================================================================
 * Test 6: Stop 成功停止运动后, 新命令可以启动
 * ================================================================== */
static void test_stop_success_then_new_command_starts(void) {
    HYD_MOVEABSOLUTE ma;
    HYD_STOP stop;

    __HydMotion_framework_Init();
    ensure_axes_allocated(2);

    /* Step 1: 先启动 MoveAbsolute */
    start_moveabsolute_on_axis(0, &ma);
    ASSERT_TRUE(IEC_VAL(ma.ACTIVE) || IEC_VAL(ma.BUSY),
               "MoveAbsolute should be running");

    /* Step 2: Stop 接管 */
    memset(&stop, 0, sizeof(stop));
    IEC_VAL(stop.EN) = true;
    IEC_VAL(stop.EXECUTE) = true;
    stop.EXECUTE0.value = false;
    IEC_VAL(stop.AXISID) = 0;
    __mcl_cmd_Stop(&stop);
    advance_non_sim_feedback(0, 0.01f);
    __HydMotion_framework_Publish();

    {
        int stopDoneStep = drive_stop_until_done(0, &stop, STOP_WAIT_BUDGET);
        ASSERT_TRUE(stopDoneStep > 1,
                   "Stop should require multiple cycles before axis is stopped");
    }
    ASSERT_TRUE(IEC_VAL(stop.DONE) == true,
               "Stop should report DONE after axis is stopped");
    ASSERT_TRUE(IEC_VAL(stop.BUSY) == false,
               "Stop BUSY should be false after DONE");

    /* Step 3: 新的 MoveAbsolute 在停止后的轴上启动 */
    memset(&ma, 0, sizeof(ma));
    IEC_VAL(ma.EN) = true;
    IEC_VAL(ma.EXECUTE) = true;
    ma.EXECUTE0.value = false;
    IEC_VAL(ma.AXISID) = 0;
    IEC_VAL(ma.POSITION) = 200.0f;
    IEC_VAL(ma.VELOCITY) = 60.0f;
    IEC_VAL(ma.ACCELERATION) = 200.0f;
    IEC_VAL(ma.DIRECTION) = 1;  /* EXTEND: 从0到200 */
    __mcl_cmd_MoveAbsolute(&ma);
    __HydMotion_framework_Publish();

    /* 新的 MoveAbsolute 应正常运行 */
    IEC_VAL(ma.EXECUTE) = true;
    ma.EXECUTE0.value = true;
    __mcl_cmd_MoveAbsolute(&ma);
    ASSERT_TRUE(IEC_VAL(ma.ACTIVE) || IEC_VAL(ma.BUSY),
               "New MoveAbsolute should be active after Stop");
    ASSERT_TRUE(IEC_VAL(ma.COMMANDABORTED) == false,
               "New command should not be COMMANDABORTED");
}

/* ==================================================================
 * Test 7: 自抢占 — 同一FB连续两次EXECUTE上升沿
 * ================================================================== */
static void test_self_preemption_same_fb_twice(void) {
    HYD_MOVEABSOLUTE ma;

    __HydMotion_framework_Init();
    ensure_axes_allocated(2);

    /* 第一次启动 */
    memset(&ma, 0, sizeof(ma));
    IEC_VAL(ma.EN) = true;
    IEC_VAL(ma.EXECUTE) = true;
    ma.EXECUTE0.value = false;
    IEC_VAL(ma.AXISID) = 0;
    IEC_VAL(ma.POSITION) = 100.0f;
    IEC_VAL(ma.VELOCITY) = 50.0f;
    IEC_VAL(ma.ACCELERATION) = 200.0f;
    IEC_VAL(ma.DIRECTION) = 1;
    __mcl_cmd_MoveAbsolute(&ma);
    __HydMotion_framework_Publish();

    /* 第二次启动 (同一个FB, 新的参数) — 模拟EXECUTE下降再上升 */
    IEC_VAL(ma.EXECUTE) = false;
    ma.EXECUTE0.value = true;  /* 上一次是true → 下降沿 */
    __mcl_cmd_MoveAbsolute(&ma);

    IEC_VAL(ma.EXECUTE) = true;
    ma.EXECUTE0.value = false;  /* 新上升沿 */
    IEC_VAL(ma.POSITION) = 200.0f;  /* 新目标位置 */
    IEC_VAL(ma.VELOCITY) = 80.0f;
    __mcl_cmd_MoveAbsolute(&ma);
    __HydMotion_framework_Publish();

    /* 第二次启动应该成功 */
    IEC_VAL(ma.EXECUTE) = true;
    ma.EXECUTE0.value = true;
    __mcl_cmd_MoveAbsolute(&ma);

    ASSERT_TRUE(IEC_VAL(ma.COMMANDABORTED) == false,
               "Same FB re-trigger should not COMMANDABORTED itself");
    ASSERT_TRUE(IEC_VAL(ma.BUSY) == true || IEC_VAL(ma.ACTIVE) == true,
               "Same FB should be active after re-trigger");
}

static void test_buffered_moveabsolute_waits_without_preempting_active_owner(void) {
    HYD_MOVEABSOLUTE first;
    HYD_MOVEABSOLUTE second;

    __HydMotion_framework_Init();
    ensure_axes_allocated(1);

    start_moveabsolute_on_axis(0, &first);
    ASSERT_TRUE(IEC_VAL(first.ACTIVE) || IEC_VAL(first.BUSY),
               "First MoveAbsolute should be active before buffered command");

    memset(&second, 0, sizeof(second));
    IEC_VAL(second.EN) = true;
    IEC_VAL(second.EXECUTE) = true;
    second.EXECUTE0.value = false;
    IEC_VAL(second.AXISID) = 0;
    IEC_VAL(second.POSITION) = 200.0f;
    IEC_VAL(second.VELOCITY) = 40.0f;
    IEC_VAL(second.ACCELERATION) = 100.0f;
    IEC_VAL(second.DIRECTION) = 1;
    IEC_VAL(second.BUFFERMODE) = HYD_BUFFER_MODE_BUFFER;
    __mcl_cmd_MoveAbsolute(&second);

    ASSERT_TRUE(IEC_VAL(second.ERROR) == false,
               "Buffered MoveAbsolute should be accepted while another direct command is active");
    ASSERT_TRUE(IEC_VAL(second.BUSY) == true,
               "Buffered MoveAbsolute should report BUSY while waiting for ownership");

    __HydMotion_framework_Publish();

    IEC_VAL(first.EXECUTE) = true;
    first.EXECUTE0.value = true;
    __mcl_cmd_MoveAbsolute(&first);
    ASSERT_TRUE(IEC_VAL(first.COMMANDABORTED) == false,
               "Buffered command should not preempt the active MoveAbsolute owner");
    ASSERT_TRUE(IEC_VAL(first.ACTIVE) || IEC_VAL(first.BUSY),
               "Active MoveAbsolute should keep ownership while buffered command waits");

    IEC_VAL(second.EXECUTE) = true;
    second.EXECUTE0.value = true;
    __mcl_cmd_MoveAbsolute(&second);
    ASSERT_TRUE(IEC_VAL(second.COMMANDABORTED) == false,
               "Waiting buffered MoveAbsolute should not report COMMANDABORTED");
}

static void test_buffered_endless_movevelocity_degrades_to_abort_takeover(void) {
    HYD_MOVEVELOCITY first;
    HYD_MOVEABSOLUTE second;

    __HydMotion_framework_Init();
    ensure_axes_allocated(1);

    start_movevelocity_on_axis(0, &first);
    ASSERT_TRUE(IEC_VAL(first.ACTIVE) || IEC_VAL(first.BUSY),
               "MoveVelocity should be active before buffered takeover");

    memset(&second, 0, sizeof(second));
    IEC_VAL(second.EN) = true;
    IEC_VAL(second.EXECUTE) = true;
    second.EXECUTE0.value = false;
    IEC_VAL(second.AXISID) = 0;
    IEC_VAL(second.POSITION) = 120.0f;
    IEC_VAL(second.VELOCITY) = 45.0f;
    IEC_VAL(second.ACCELERATION) = 100.0f;
    IEC_VAL(second.DIRECTION) = 1;
    IEC_VAL(second.BUFFERMODE) = HYD_BUFFER_MODE_BUFFER;
    __mcl_cmd_MoveAbsolute(&second);
    __HydMotion_framework_Publish();

    IEC_VAL(first.EXECUTE) = true;
    first.EXECUTE0.value = true;
    __mcl_cmd_MoveVelocity(&first);

    ASSERT_TRUE(IEC_VAL(first.COMMANDABORTED) == true,
               "Buffered command after endless MoveVelocity should degrade to abort takeover");

    IEC_VAL(second.EXECUTE) = true;
    second.EXECUTE0.value = true;
    __mcl_cmd_MoveAbsolute(&second);
    ASSERT_TRUE(IEC_VAL(second.ACTIVE) || IEC_VAL(second.BUSY),
               "Buffered command should become active after endless MoveVelocity takeover");
}

static void test_blending_modes_select_distinct_through_velocities(void) {
    HYD_MotionControlFB* fb;

    fb = start_blend_pair(HYD_BUFFER_MODE_BLENDING_LOW, 20.0f, 8.0f);
    ASSERT_TRUE(fb->_directBlendContext.active,
               "BlendingLow should create an active direct blend context");
    ASSERT_TRUE(fabs(fb->_directBlendContext.blendVelocity - 8.0f) < 0.001f,
               "BlendingLow should use the lower velocity");

    fb = start_blend_pair(HYD_BUFFER_MODE_BLENDING_PREVIOUS, 20.0f, 8.0f);
    ASSERT_TRUE(fb->_directBlendContext.active,
               "BlendingPrevious should create an active direct blend context");
    ASSERT_TRUE(fabs(fb->_directBlendContext.blendVelocity - 20.0f) < 0.001f,
               "BlendingPrevious should use the previous velocity");

    fb = start_blend_pair(HYD_BUFFER_MODE_BLENDING_NEXT, 20.0f, 8.0f);
    ASSERT_TRUE(fb->_directBlendContext.active,
               "BlendingNext should create an active direct blend context");
    ASSERT_TRUE(fabs(fb->_directBlendContext.blendVelocity - 8.0f) < 0.001f,
               "BlendingNext should use the next velocity");

    fb = start_blend_pair(HYD_BUFFER_MODE_BLENDING_HIGH, 20.0f, 8.0f);
    ASSERT_TRUE(fb->_directBlendContext.active,
               "BlendingHigh should create an active direct blend context");
    ASSERT_TRUE(fabs(fb->_directBlendContext.blendVelocity - 20.0f) < 0.001f,
               "BlendingHigh should use the higher velocity");
}

static void test_blended_front_segment_keeps_nonzero_velocity_near_switch(void) {
    HYD_MotionControlFB* fb;

    fb = start_blend_pair(HYD_BUFFER_MODE_BLENDING_NEXT, 20.0f, 8.0f);
    ASSERT_TRUE(fb != NULL, "Axis 0 control FB should exist for blend output test");
    ASSERT_TRUE(fb->_directBlendContext.active,
               "Blend context should be active before near-switch cycle");

    fb->_activeSegment.positionTolerance = 0.05f;
    fb->_directBlendContext.switchTolerance = 0.05f;
    fb->_directPendingValid = false;
    fb->AXIS_REF.position = 99.98f;
    fb->AXIS_REF.velocity = 8.0f;
    fb->_plannerState.initialized = true;
    fb->_plannerState.lastTargetVelocity = 8.0f;
    fb->AXIS_REF.timestamp += 0.1f;

    __HydMotion_framework_Publish();

    ASSERT_TRUE(fabs(fb->STATE.references.velocityReference - 8.0f) < 0.001f,
               "Blended front segment should hold the selected through velocity inside switch tolerance");
}

static void test_blended_cutover_preserves_planner_state(void) {
    HYD_MotionControlFB* fb;
    HYD_MOVEABSOLUTE first;
    HYD_MOVEABSOLUTE second;
    IEC_WORD firstExecId;

    __HydMotion_framework_Init();
    ensure_axes_allocated(1);
    fb = __MK_GetPublic_MotionControlFB(0);
    ASSERT_TRUE(fb != NULL, "Axis 0 control FB should exist for cutover test");

    memset(&first, 0, sizeof(first));
    IEC_VAL(first.EN) = true;
    IEC_VAL(first.EXECUTE) = true;
    first.EXECUTE0.value = false;
    IEC_VAL(first.AXISID) = 0;
    IEC_VAL(first.POSITION) = 100.0f;
    IEC_VAL(first.VELOCITY) = 20.0f;
    IEC_VAL(first.ACCELERATION) = 100.0f;
    IEC_VAL(first.DECELERATION) = 100.0f;
    IEC_VAL(first.DIRECTION) = 1;
    IEC_VAL(first.BUFFERMODE) = HYD_BUFFER_MODE_ABORT;
    __mcl_cmd_MoveAbsolute(&first);
    __HydMotion_framework_Publish();

    IEC_VAL(first.EXECUTE) = true;
    first.EXECUTE0.value = true;
    __mcl_cmd_MoveAbsolute(&first);
    ASSERT_TRUE(IEC_VAL(first._EXEC_ID) != 0,
               "Front MoveAbsolute should latch direct ownership before blend cutover");
    firstExecId = IEC_VAL(first._EXEC_ID);

    memset(&second, 0, sizeof(second));
    IEC_VAL(second.EN) = true;
    IEC_VAL(second.EXECUTE) = true;
    second.EXECUTE0.value = false;
    IEC_VAL(second.AXISID) = 0;
    IEC_VAL(second.POSITION) = 200.0f;
    IEC_VAL(second.VELOCITY) = 8.0f;
    IEC_VAL(second.ACCELERATION) = 100.0f;
    IEC_VAL(second.DECELERATION) = 100.0f;
    IEC_VAL(second.DIRECTION) = 1;
    IEC_VAL(second.BUFFERMODE) = HYD_BUFFER_MODE_BLENDING_NEXT;
    __mcl_cmd_MoveAbsolute(&second);
    ASSERT_TRUE(fb->_directPendingValid,
               "Pending direct command should be present before cutover");
    ASSERT_TRUE(IEC_VAL(second.BUSY),
               "Buffered MoveAbsolute should wait busy before blend cutover");

    fb->AXIS_REF.position = 100.0f;
    fb->AXIS_REF.velocity = 8.0f;
    fb->AXIS_REF.timestamp += 0.1f;
    fb->_plannerState.initialized = true;
    fb->_plannerState.lastTargetVelocity = 8.0f;

    __HydMotion_framework_Publish();

    IEC_VAL(first.EXECUTE) = true;
    first.EXECUTE0.value = true;
    __mcl_cmd_MoveAbsolute(&first);
    ASSERT_TRUE(IEC_VAL(first.DONE) == true,
               "Front MoveAbsolute should report DONE after blended cutover");
    ASSERT_TRUE(IEC_VAL(first.COMMANDABORTED) == false,
               "Front MoveAbsolute should not report COMMANDABORTED after blended cutover");
    ASSERT_TRUE(IEC_VAL(first.BUSY) == false,
               "Front MoveAbsolute should clear BUSY after blended cutover");
    ASSERT_TRUE(IEC_VAL(first.ACTIVE) == false,
               "Front MoveAbsolute should clear ACTIVE after blended cutover");
    ASSERT_TRUE(!HYD_MotionControlFB_ConsumeDirectTicketCompleted(fb,
                                                                   (uint16_t)firstExecId,
                                                                   HYD_DIRECT_CMD_MOVE_ABSOLUTE),
               "Front MoveAbsolute completion marker should be consumed after reporting DONE");

    IEC_VAL(second.EXECUTE) = true;
    second.EXECUTE0.value = true;
    __mcl_cmd_MoveAbsolute(&second);
    ASSERT_TRUE(IEC_VAL(second.COMMANDABORTED) == false,
               "Second MoveAbsolute should not report COMMANDABORTED after blended cutover");
    ASSERT_TRUE(IEC_VAL(second.BUSY) || IEC_VAL(second.ACTIVE),
               "Second MoveAbsolute should be busy or active after blended cutover");
    ASSERT_TRUE(IEC_VAL(second._EXEC_ID) != 0,
               "Second MoveAbsolute should latch direct ownership after blended cutover");

    ASSERT_TRUE(!fb->_directPendingValid,
               "Pending direct command should be consumed by blended cutover");
    ASSERT_TRUE(!fb->_directBlendContext.active,
               "Blend context should be cleared after blended cutover");
    ASSERT_TRUE(fabs(fb->_activeSegment.targetPosition - 200.0f) < 0.001f,
               "Pending MoveAbsolute should become active segment after cutover");
    ASSERT_TRUE(fb->_plannerState.initialized,
               "Planner state should remain initialized across blended cutover");
    ASSERT_TRUE(fabs(fb->_plannerState.lastTargetVelocity) > 0.1f,
               "Planner velocity should remain nonzero across blended cutover");
}

static void test_blended_cutover_keeps_nonzero_output_velocity_same_scan(void) {
    HYD_MotionControlFB* fb;
    HYD_MOVEABSOLUTE first;
    HYD_MOVEABSOLUTE second;

    __HydMotion_framework_Init();
    ensure_axes_allocated(1);
    fb = __MK_GetPublic_MotionControlFB(0);
    ASSERT_TRUE(fb != NULL, "Axis 0 control FB should exist for same-scan cutover output test");
    if (fb == NULL) {
        return;
    }

    memset(&first, 0, sizeof(first));
    IEC_VAL(first.EN) = true;
    IEC_VAL(first.EXECUTE) = true;
    first.EXECUTE0.value = false;
    IEC_VAL(first.AXISID) = 0;
    IEC_VAL(first.POSITION) = 100.0f;
    IEC_VAL(first.VELOCITY) = 5.0f;
    IEC_VAL(first.ACCELERATION) = 100.0f;
    IEC_VAL(first.DECELERATION) = 100.0f;
    IEC_VAL(first.DIRECTION) = 1;
    IEC_VAL(first.BUFFERMODE) = HYD_BUFFER_MODE_ABORT;
    __mcl_cmd_MoveAbsolute(&first);
    __HydMotion_framework_Publish();

    IEC_VAL(first.EXECUTE) = true;
    first.EXECUTE0.value = true;
    __mcl_cmd_MoveAbsolute(&first);

    memset(&second, 0, sizeof(second));
    IEC_VAL(second.EN) = true;
    IEC_VAL(second.EXECUTE) = true;
    second.EXECUTE0.value = false;
    IEC_VAL(second.AXISID) = 0;
    IEC_VAL(second.POSITION) = 200.0f;
    IEC_VAL(second.VELOCITY) = 20.0f;
    IEC_VAL(second.ACCELERATION) = 100.0f;
    IEC_VAL(second.DECELERATION) = 100.0f;
    IEC_VAL(second.DIRECTION) = 1;
    IEC_VAL(second.BUFFERMODE) = HYD_BUFFER_MODE_BLENDING_HIGH;
    __mcl_cmd_MoveAbsolute(&second);

    fb->AXIS_REF.position = 100.0f;
    fb->AXIS_REF.velocity = 5.0f;
    fb->_plannerState.initialized = true;
    fb->_plannerState.lastTargetVelocity = 5.0f;
    fb->_plannerState.lastTargetFlow = 1.0f;
    advance_non_sim_feedback(0, 0.1f);

    __HydMotion_framework_Publish();

    ASSERT_TRUE(fabs(fb->_plannerState.lastTargetVelocity) > 0.1f,
               "Planner velocity should remain nonzero on the cutover scan");
    ASSERT_TRUE(fabs(fb->STATE.plannedVelocity) > 0.1f,
               "State plannedVelocity should remain nonzero on the cutover scan");
    ASSERT_TRUE(fabs(fb->_simFeedback.targetVelocity) > 0.1f,
               "Simulation feedback velocity should remain nonzero on the cutover scan");
    ASSERT_TRUE(fabs(fb->AXIS_REF.velocity) > 0.05f,
               "Axis velocity should not drop to zero on the cutover scan");
    ASSERT_TRUE(fb->_activeSegmentValid,
               "Active segment should stay valid on the cutover scan");
    ASSERT_TRUE(fabs(fb->_activeSegment.targetPosition - 200.0f) < 0.001f,
               "Second MoveAbsolute should own the active segment on the cutover scan");
}

static void test_moveabsolute_direct_start_latches_ownership_on_rising_edge(void) {
    HYD_MOVEABSOLUTE ma;

    __HydMotion_framework_Init();
    ensure_axes_allocated(1);

    memset(&ma, 0, sizeof(ma));
    IEC_VAL(ma.EN) = true;
    IEC_VAL(ma.EXECUTE) = true;
    ma.EXECUTE0.value = false;
    IEC_VAL(ma.AXISID) = 0;
    IEC_VAL(ma.POSITION) = 100.0f;
    IEC_VAL(ma.VELOCITY) = 50.0f;
    IEC_VAL(ma.ACCELERATION) = 200.0f;
    IEC_VAL(ma.DECELERATION) = 200.0f;
    IEC_VAL(ma.DIRECTION) = 1;
    IEC_VAL(ma.BUFFERMODE) = HYD_BUFFER_MODE_ABORT;

    __mcl_cmd_MoveAbsolute(&ma);

    ASSERT_TRUE(IEC_VAL(ma._PENDING) == false,
               "MoveAbsolute should not remain pending on same-scan direct start");
    ASSERT_TRUE(IEC_VAL(ma._EXEC_ID) != 0,
               "MoveAbsolute should latch execution ownership on the rising-edge call");
    ASSERT_TRUE(IEC_VAL(ma.BUSY) == true,
               "MoveAbsolute should be BUSY on the rising-edge call");
    ASSERT_TRUE(IEC_VAL(ma.ACTIVE) == true,
               "MoveAbsolute should be ACTIVE on the rising-edge call");
    ASSERT_TRUE(IEC_VAL(ma.DONE) == false,
               "MoveAbsolute should not be DONE on the rising-edge call");
    ASSERT_TRUE(IEC_VAL(ma.COMMANDABORTED) == false,
               "MoveAbsolute should not be COMMANDABORTED on the rising-edge call");
}

static HYD_MotionControlFB* start_active_moveabsolute_for_blend_fallback_test(void) {
    HYD_MOVEABSOLUTE first;
    HYD_MotionControlFB* fb;

    __HydMotion_framework_Init();
    ensure_axes_allocated(1);
    fb = __MK_GetPublic_MotionControlFB(0);
    ASSERT_TRUE(fb != NULL, "Axis 0 control FB should exist for blend fallback test");

    memset(&first, 0, sizeof(first));
    IEC_VAL(first.EN) = true;
    IEC_VAL(first.EXECUTE) = true;
    first.EXECUTE0.value = false;
    IEC_VAL(first.AXISID) = 0;
    IEC_VAL(first.POSITION) = 100.0f;
    IEC_VAL(first.VELOCITY) = 20.0f;
    IEC_VAL(first.ACCELERATION) = 100.0f;
    IEC_VAL(first.DECELERATION) = 100.0f;
    IEC_VAL(first.DIRECTION) = 1;
    IEC_VAL(first.BUFFERMODE) = HYD_BUFFER_MODE_ABORT;
    __mcl_cmd_MoveAbsolute(&first);
    __HydMotion_framework_Publish();

    fb->AXIS_REF.position = 10.0f;

    return fb;
}

static void queue_pending_moveabsolute_for_blend_fallback_test(HYD_REAL targetPosition,
                                                              IEC_SINT direction) {
    HYD_MOVEABSOLUTE second;

    memset(&second, 0, sizeof(second));
    IEC_VAL(second.EN) = true;
    IEC_VAL(second.EXECUTE) = true;
    second.EXECUTE0.value = false;
    IEC_VAL(second.AXISID) = 0;
    IEC_VAL(second.POSITION) = targetPosition;
    IEC_VAL(second.VELOCITY) = 8.0f;
    IEC_VAL(second.ACCELERATION) = 100.0f;
    IEC_VAL(second.DECELERATION) = 100.0f;
    IEC_VAL(second.DIRECTION) = direction;
    IEC_VAL(second.BUFFERMODE) = HYD_BUFFER_MODE_BLENDING_HIGH;
    __mcl_cmd_MoveAbsolute(&second);
}

static void test_blend_context_requires_compatible_moveabsolute_direction(void) {
    HYD_MotionControlFB* fb;

    fb = start_active_moveabsolute_for_blend_fallback_test();
    queue_pending_moveabsolute_for_blend_fallback_test(200.0f, 1);
    ASSERT_TRUE(fb->_directPendingValid,
               "Forward MoveAbsolute should be accepted into pending slot");
    ASSERT_TRUE(fb->_directBlendContext.active,
               "Forward MoveAbsolute should create a direct blend context");

    fb = start_active_moveabsolute_for_blend_fallback_test();
    queue_pending_moveabsolute_for_blend_fallback_test(0.0f, 2)  /* NEGATIVE */;

    ASSERT_TRUE(fb->_directPendingValid,
               "Reverse MoveAbsolute should still be accepted into pending slot");
    ASSERT_TRUE(!fb->_directBlendContext.active,
               "Reverse MoveAbsolute should not create a nonzero blend context");
}

/* ==================================================================
 * Test 8: Reset 后旧命令失去所有权
 * ================================================================== */
static void test_previous_command_loses_ownership_after_reset(void) {
    HYD_MOVEABSOLUTE ma;
    HYD_RESET reset;

    __HydMotion_framework_Init();
    ensure_axes_allocated(2);

    /* 启动 MoveAbsolute */
    start_moveabsolute_on_axis(0, &ma);
    ASSERT_TRUE(IEC_VAL(ma.ACTIVE) || IEC_VAL(ma.BUSY),
               "MoveAbsolute should be active before reset");

    /* Reset 轴 */
    memset(&reset, 0, sizeof(reset));
    IEC_VAL(reset.EN) = true;
    IEC_VAL(reset.EXECUTE) = true;
    reset.EXECUTE0.value = false;
    IEC_VAL(reset.AXISID) = 0;
    __mcl_cmd_Reset(&reset);
    __HydMotion_framework_Publish();

    /* MoveAbsolute 应检测到失去所有权 (executionId不匹配, Reset归零了_executionId) */
    IEC_VAL(ma.EXECUTE) = true;
    ma.EXECUTE0.value = true;
    __mcl_cmd_MoveAbsolute(&ma);

    ASSERT_TRUE(IEC_VAL(ma.COMMANDABORTED) == true,
               "MoveAbsolute should raise COMMANDABORTED after axis reset");
}

/* ==================================================================
 * Test 9: 抢占链 — MoveAbsolute → MoveVelocity → Stop
 * ================================================================== */
static void test_preemption_chain_three_commands(void) {
    HYD_MOVEABSOLUTE ma;
    HYD_MOVEVELOCITY mv;
    HYD_STOP stop;

    __HydMotion_framework_Init();
    ensure_axes_allocated(2);

    /* Command 1: MoveAbsolute */
    start_moveabsolute_on_axis(0, &ma);
    ASSERT_TRUE(IEC_VAL(ma.ACTIVE) || IEC_VAL(ma.BUSY), "Cmd1 (MA) should be active");

    /* Command 2: MoveVelocity 抢占 */
    memset(&mv, 0, sizeof(mv));
    IEC_VAL(mv.EN) = true;
    IEC_VAL(mv.EXECUTE) = true;
    mv.EXECUTE0.value = false;
    IEC_VAL(mv.AXISID) = 0;
    IEC_VAL(mv.VELOCITY) = 30.0f;
    IEC_VAL(mv.ACCELERATION) = 150.0f;
    IEC_VAL(mv.DIRECTION) = 1;
    __mcl_cmd_MoveVelocity(&mv);
    __HydMotion_framework_Publish();

    /* Command 1 被抢占 */
    IEC_VAL(ma.EXECUTE) = true;
    ma.EXECUTE0.value = true;
    __mcl_cmd_MoveAbsolute(&ma);
    ASSERT_TRUE(IEC_VAL(ma.COMMANDABORTED) == true, "Cmd1 (MA) should be COMMANDABORTED");

    /* Command 2 继续运行 */
    IEC_VAL(mv.EXECUTE) = true;
    mv.EXECUTE0.value = true;
    __mcl_cmd_MoveVelocity(&mv);
    ASSERT_TRUE(IEC_VAL(mv.ACTIVE) || IEC_VAL(mv.BUSY), "Cmd2 (MV) should be active");
    ASSERT_TRUE(IEC_VAL(mv.COMMANDABORTED) == false, "Cmd2 should not be aborted yet");

    /* Command 3: Stop 抢占 */
    memset(&stop, 0, sizeof(stop));
    IEC_VAL(stop.EN) = true;
    IEC_VAL(stop.EXECUTE) = true;
    stop.EXECUTE0.value = false;
    IEC_VAL(stop.AXISID) = 0;
    __mcl_cmd_Stop(&stop);
    __HydMotion_framework_Publish();

    /* Command 2 被抢占 */
    IEC_VAL(mv.EXECUTE) = true;
    mv.EXECUTE0.value = true;
    __mcl_cmd_MoveVelocity(&mv);
    ASSERT_TRUE(IEC_VAL(mv.COMMANDABORTED) == true,
               "Cmd2 (MV) should be COMMANDABORTED after Cmd3 (Stop)");
}

/* ==================================================================
 * Test 10: 空转生成 — 从未激活的FB不应误报COMMANDABORTED
 * ================================================================== */
static void test_never_activated_fb_no_false_commandaborted(void) {
    HYD_MOVEABSOLUTE ma;

    __HydMotion_framework_Init();
    ensure_axes_allocated(2);
    memset(&ma, 0, sizeof(ma));

    /* 不启动MoveAbsolute, 只设置EN并调用 */
    IEC_VAL(ma.EN) = true;
    IEC_VAL(ma.EXECUTE) = false;
    ma.EXECUTE0.value = false;
    IEC_VAL(ma.AXISID) = 0;
    IEC_VAL(ma.POSITION) = 100.0f;
    IEC_VAL(ma.VELOCITY) = 50.0f;
    IEC_VAL(ma.ACCELERATION) = 200.0f;

    /* 直接非上升沿调用 (从未激活) */
    __mcl_cmd_MoveAbsolute(&ma);

    ASSERT_TRUE(IEC_VAL(ma.COMMANDABORTED) == false,
               "Never-activated FB should not get COMMANDABORTED");
    ASSERT_TRUE(IEC_VAL(ma.BUSY) == false,
               "Never-activated FB should not be BUSY");
}

static void test_direct_command_preempts_moveprofile(void) {
    HYD_MOVEPROFILE mp;
    HYD_MOVEABSOLUTE ma;
    HYD_CREATEMOTION cm;
    HYD_AXISMOTION motion;

    __HydMotion_framework_Init();

    memset(&cm, 0, sizeof(cm));
    IEC_VAL(cm.EN) = true;
    IEC_VAL(cm.USE_RECIPE) = true;
    IEC_VAL(cm.FLOW_TO_PUMPSPEED) = 1.2f;
    IEC_VAL(cm.PUMPSPEED_LIMIT) = 3000.0f;
    IEC_VAL(cm.USE_SIMULATION) = false;
    __mcl_cmd_CreateMotion(&cm);

    memset(&mp, 0, sizeof(mp));
    memset(&motion, 0, sizeof(motion));
    IEC_VAL(mp.EN) = true;
    IEC_VAL(mp.EXECUTE) = true;
    mp.EXECUTE0.value = false;
    IEC_VAL(mp.AXISID) = IEC_VAL(cm.AXISID);
    motion.MODE = HYD_MODE_POSITION;
    motion.ENDCONDITION = HYD_END_POSITION;
    motion.DIRECTION = HYD_DIRECTION_EXTEND;
    motion.SETPOSITION = 100.0f;
    motion.SETVELOCITY = 40.0f;
    motion.SETFLOW = 10.0f;
    motion.ACCELERATION = 150.0f;
    motion.TIMESTAMP = 0.0f;
    __SET_VAR(mp., MOTION, , motion);
    __mcl_cmd_MoveProfile(&mp);
    __HydMotion_framework_Publish();

    memset(&ma, 0, sizeof(ma));
    IEC_VAL(ma.EN) = true;
    IEC_VAL(ma.EXECUTE) = true;
    ma.EXECUTE0.value = false;
    IEC_VAL(ma.AXISID) = IEC_VAL(cm.AXISID);
    IEC_VAL(ma.POSITION) = 50.0f;
    IEC_VAL(ma.VELOCITY) = 30.0f;
    IEC_VAL(ma.ACCELERATION) = 120.0f;
    IEC_VAL(ma.DIRECTION) = 1;
    __mcl_cmd_MoveAbsolute(&ma);
    __HydMotion_framework_Publish();

    IEC_VAL(mp.EXECUTE) = true;
    mp.EXECUTE0.value = true;
    __mcl_cmd_MoveProfile(&mp);

    ASSERT_TRUE(IEC_VAL(mp.ACTIVE) == false,
               "MoveProfile should clear ACTIVE when a direct command takes over");
    ASSERT_TRUE(IEC_VAL(mp.BUSY) == false,
               "MoveProfile should clear BUSY when a direct command takes over");
}

static void test_moveprofile_loses_activity_when_direct_command_takes_over(void) {
    HYD_CREATEMOTION cm;
    HYD_MOVEPROFILE mp;
    HYD_AXISMOTION motion;
    HYD_MOVEABSOLUTE ma;

    __HydMotion_framework_Init();

    memset(&cm, 0, sizeof(cm));
    IEC_VAL(cm.EN) = true;
    IEC_VAL(cm.USE_RECIPE) = true;
    IEC_VAL(cm.FLOW_TO_PUMPSPEED) = 1.2f;
    IEC_VAL(cm.PUMPSPEED_LIMIT) = 3000.0f;
    IEC_VAL(cm.USE_SIMULATION) = false;
    __mcl_cmd_CreateMotion(&cm);

    memset(&mp, 0, sizeof(mp));
    memset(&motion, 0, sizeof(motion));
    IEC_VAL(mp.EN) = true;
    IEC_VAL(mp.EXECUTE) = true;
    mp.EXECUTE0.value = false;
    IEC_VAL(mp.AXISID) = IEC_VAL(cm.AXISID);

    motion.MODE = HYD_MODE_POSITION;
    motion.ENDCONDITION = HYD_END_POSITION;
    motion.DIRECTION = HYD_DIRECTION_EXTEND;
    motion.SETPOSITION = 100.0f;
    motion.SETVELOCITY = 40.0f;
    motion.SETFLOW = 10.0f;
    motion.ACCELERATION = 150.0f;
    motion.TIMESTAMP = 0.0f;
    __SET_VAR(mp., MOTION, , motion);

    __mcl_cmd_MoveProfile(&mp);
    __HydMotion_framework_Publish();
    IEC_VAL(mp.EXECUTE) = true;
    mp.EXECUTE0.value = true;
    __mcl_cmd_MoveProfile(&mp);
    ASSERT_TRUE(IEC_VAL(mp.ACTIVE) || IEC_VAL(mp.BUSY),
               "MoveProfile should be active before direct takeover");

    memset(&ma, 0, sizeof(ma));
    IEC_VAL(ma.EN) = true;
    IEC_VAL(ma.EXECUTE) = true;
    ma.EXECUTE0.value = false;
    IEC_VAL(ma.AXISID) = IEC_VAL(cm.AXISID);
    IEC_VAL(ma.POSITION) = 50.0f;
    IEC_VAL(ma.VELOCITY) = 20.0f;
    IEC_VAL(ma.ACCELERATION) = 100.0f;
    IEC_VAL(ma.DIRECTION) = 1;
    __mcl_cmd_MoveAbsolute(&ma);
    __HydMotion_framework_Publish();

    IEC_VAL(mp.EXECUTE) = true;
    mp.EXECUTE0.value = true;
    __mcl_cmd_MoveProfile(&mp);

    ASSERT_TRUE(IEC_VAL(mp.COMMANDABORTED) == true,
               "MoveProfile should raise COMMANDABORTED when direct motion takes over");
    ASSERT_TRUE(IEC_VAL(mp.ACTIVE) == false,
               "MoveProfile should clear ACTIVE after direct takeover");
    ASSERT_TRUE(IEC_VAL(mp.BUSY) == false,
               "MoveProfile should clear BUSY after direct takeover");
    ASSERT_TRUE(IEC_VAL(mp.DONE) == false,
               "MoveProfile should not report DONE when displaced by takeover");
}

static void test_moveprofile_loses_activity_after_reset_takeover(void) {
    HYD_CREATEMOTION cm;
    HYD_MOVEPROFILE mp;
    HYD_AXISMOTION motion;
    HYD_RESET reset;

    __HydMotion_framework_Init();

    memset(&cm, 0, sizeof(cm));
    IEC_VAL(cm.EN) = true;
    IEC_VAL(cm.USE_RECIPE) = true;
    IEC_VAL(cm.FLOW_TO_PUMPSPEED) = 1.2f;
    IEC_VAL(cm.PUMPSPEED_LIMIT) = 3000.0f;
    IEC_VAL(cm.USE_SIMULATION) = false;
    __mcl_cmd_CreateMotion(&cm);

    memset(&mp, 0, sizeof(mp));
    memset(&motion, 0, sizeof(motion));
    IEC_VAL(mp.EN) = true;
    IEC_VAL(mp.EXECUTE) = true;
    mp.EXECUTE0.value = false;
    IEC_VAL(mp.AXISID) = IEC_VAL(cm.AXISID);

    motion.MODE = HYD_MODE_POSITION;
    motion.ENDCONDITION = HYD_END_POSITION;
    motion.DIRECTION = HYD_DIRECTION_EXTEND;
    motion.SETPOSITION = 80.0f;
    motion.SETVELOCITY = 30.0f;
    motion.SETFLOW = 8.0f;
    motion.ACCELERATION = 120.0f;
    motion.TIMESTAMP = 0.0f;
    __SET_VAR(mp., MOTION, , motion);

    __mcl_cmd_MoveProfile(&mp);
    __HydMotion_framework_Publish();
    IEC_VAL(mp.EXECUTE) = true;
    mp.EXECUTE0.value = true;
    __mcl_cmd_MoveProfile(&mp);

    memset(&reset, 0, sizeof(reset));
    IEC_VAL(reset.EN) = true;
    IEC_VAL(reset.EXECUTE) = true;
    reset.EXECUTE0.value = false;
    IEC_VAL(reset.AXISID) = IEC_VAL(cm.AXISID);
    __mcl_cmd_Reset(&reset);
    ASSERT_TRUE(IEC_VAL(reset.DONE) == true,
               "Reset should complete immediately on the recipe axis");

    IEC_VAL(mp.EXECUTE) = true;
    mp.EXECUTE0.value = true;
    __mcl_cmd_MoveProfile(&mp);

    ASSERT_TRUE(IEC_VAL(mp.COMMANDABORTED) == true,
               "MoveProfile should raise COMMANDABORTED after reset takeover");
    ASSERT_TRUE(IEC_VAL(mp.ACTIVE) == false,
               "MoveProfile should clear ACTIVE after reset takeover");
    ASSERT_TRUE(IEC_VAL(mp.BUSY) == false,
               "MoveProfile should clear BUSY after reset takeover");
    ASSERT_TRUE(IEC_VAL(mp.DONE) == false,
               "MoveProfile should not report DONE after reset takeover");
}

static void test_direct_command_starts_cleanly_after_recipe_done(void) {
    HYD_CREATEMOTION cm;
    HYD_MOVEPROFILE mp;
    HYD_AXISMOTION motion;
    HYD_MOVEABSOLUTE ma;
    HYD_MotionControlFB* fb;
    int step;

    __HydMotion_framework_Init();

    memset(&cm, 0, sizeof(cm));
    IEC_VAL(cm.EN) = true;
    IEC_VAL(cm.USE_RECIPE) = true;
    IEC_VAL(cm.FLOW_TO_PUMPSPEED) = 1.0f;
    IEC_VAL(cm.PUMPSPEED_LIMIT) = 3000.0f;
    IEC_VAL(cm.USE_SIMULATION) = true;
    __mcl_cmd_CreateMotion(&cm);
    fb = __MK_GetPublic_MotionControlFB((int)IEC_VAL(cm.AXISID));
    ASSERT_TRUE(fb != NULL, "Recipe axis should expose an FB for simulation-backed MoveProfile");

    memset(&mp, 0, sizeof(mp));
    memset(&motion, 0, sizeof(motion));
    IEC_VAL(mp.EN) = true;
    IEC_VAL(mp.EXECUTE) = true;
    mp.EXECUTE0.value = false;
    IEC_VAL(mp.AXISID) = IEC_VAL(cm.AXISID);

    motion.MODE = HYD_MODE_POSITION;
    motion.ENDCONDITION = HYD_END_POSITION;
    motion.DIRECTION = HYD_DIRECTION_EXTEND;
    motion.SETPOSITION = 20.0f;
    motion.SETVELOCITY = 10.0f;
    motion.SETFLOW = 2.0f;
    motion.ACCELERATION = 50.0f;
    motion.TIMESTAMP = 0.0f;
    __SET_VAR(mp., MOTION, , motion);

    __mcl_cmd_MoveProfile(&mp);
    __HydMotion_framework_Publish();
    IEC_VAL(mp.EXECUTE) = true;
    mp.EXECUTE0.value = true;
    __mcl_cmd_MoveProfile(&mp);

    for (step = 0; step < 4000; step++) {
        __HydMotion_framework_Publish();
        motion = __GET_VAR(mp.MOTION);
        motion.ACTPOSITION = (REAL)fb->AXIS_REF.position;
        motion.ACTVELOCITY = (REAL)fb->AXIS_REF.velocity;
        motion.ACTFLOW = (REAL)fb->AXIS_REF.flow;
        motion.ACTPRESSURE = (REAL)fb->AXIS_REF.pressure;
        motion.TIMESTAMP = (REAL)fb->AXIS_REF.timestamp;
        __SET_VAR(mp., MOTION, , motion);
        IEC_VAL(mp.EXECUTE) = true;
        mp.EXECUTE0.value = true;
        __mcl_cmd_MoveProfile(&mp);
        if (IEC_VAL(mp.DONE)) {
            break;
        }
    }

    ASSERT_TRUE(IEC_VAL(mp.DONE) == true,
               "MoveProfile should reach DONE on the single recipe segment");

    IEC_VAL(mp.EXECUTE) = false;
    mp.EXECUTE0.value = true;
    __mcl_cmd_MoveProfile(&mp);
    __HydMotion_framework_Publish();

    memset(&ma, 0, sizeof(ma));
    IEC_VAL(ma.EN) = true;
    IEC_VAL(ma.EXECUTE) = true;
    ma.EXECUTE0.value = false;
    IEC_VAL(ma.AXISID) = IEC_VAL(cm.AXISID);
    IEC_VAL(ma.POSITION) = 0.0f;
    IEC_VAL(ma.VELOCITY) = 15.0f;
    IEC_VAL(ma.ACCELERATION) = 70.0f;
    IEC_VAL(ma.DIRECTION) = 2;  /* NEGATIVE */
    __mcl_cmd_MoveAbsolute(&ma);
    __HydMotion_framework_Publish();
    IEC_VAL(ma.EXECUTE) = true;
    ma.EXECUTE0.value = true;
    __mcl_cmd_MoveAbsolute(&ma);

    ASSERT_TRUE(IEC_VAL(ma.ERROR) == false,
               "Direct command should start cleanly after recipe DONE");
    ASSERT_TRUE(IEC_VAL(ma.COMMANDABORTED) == false,
               "Direct command should not inherit an aborted lifecycle after recipe DONE");
}

static void test_moveprofile_rejects_oversized_recipe_load(void) {
    HYD_CREATEMOTION cm;
    HYD_MotionControlFB* fb;
    HYD_MotionSegment recipe[2];

    __HydMotion_framework_Init();

    memset(&cm, 0, sizeof(cm));
    IEC_VAL(cm.EN) = true;
    IEC_VAL(cm.USE_RECIPE) = true;
    IEC_VAL(cm.FLOW_TO_PUMPSPEED) = 1.0f;
    IEC_VAL(cm.PUMPSPEED_LIMIT) = 3000.0f;
    IEC_VAL(cm.USE_SIMULATION) = false;
    __mcl_cmd_CreateMotion(&cm);
    fb = __MK_GetPublic_MotionControlFB((int)IEC_VAL(cm.AXISID));
    ASSERT_TRUE(fb != NULL, "Recipe axis should expose an FB");

    memset(recipe, 0, sizeof(recipe));
    recipe[0].segmentTag = 1;
    recipe[0].segmentType = HYD_SEGMENT_TYPE_HOLDING;
    recipe[0].mode = HYD_MODE_PRESSURE_CLOSED_LOOP;
    recipe[0].endCondition = HYD_END_TIME;
    recipe[0].direction = HYD_DIRECTION_HOLD;
    recipe[0].targetPressure = 5.0f;
    recipe[0].targetFlow = 1.0f;
    recipe[0].maxFlow = 5.0f;
    recipe[0].duration = 0.002f;
    recipe[0].pressureController = HYD_PRESSURE_CONTROLLER_P;
    recipe[0].pressureKp = 0.5f;
    recipe[0].pressureTolerance = 0.5f;
    recipe[0].timeoutLimit = 1.0f;

    recipe[1] = recipe[0];
    recipe[1].segmentTag = 2;

    assert_oversized_recipe_load_rejected(fb, recipe, 2U);
    ASSERT_TRUE(fb->DIAGNOSTIC.code == HYD_DIAG_CODE_RECIPE_TOO_LARGE,
               "Oversized recipe load should report RECIPE_TOO_LARGE");
}

static void test_reverse_blend_pending_completes_as_buffered(void) {
    HYD_MotionControlFB* fb;
    HYD_MOVEABSOLUTE second;

    fb = start_active_moveabsolute_for_blend_fallback_test();
    queue_pending_moveabsolute_for_blend_fallback_test(0.0f, 2)  /* NEGATIVE */;

    ASSERT_TRUE(fb->_directPendingValid,
               "Reverse MoveAbsolute should still occupy the pending slot");
    ASSERT_TRUE(!fb->_directBlendContext.active,
               "Reverse MoveAbsolute should not create a blend context");

    fb->AXIS_REF.position = 100.0f;
    fb->AXIS_REF.velocity = 0.0f;
    fb->AXIS_REF.timestamp += 0.5f;
    __HydMotion_framework_Publish();

    ASSERT_TRUE(!fb->_directPendingValid,
               "Reverse pending should be consumed via buffered completion path");
    ASSERT_TRUE(fb->_activeSegmentValid,
               "Active segment should be the reverse MoveAbsolute after buffered cutover");
    ASSERT_TRUE(fabs(fb->_activeSegment.targetPosition - 0.0f) < 0.001f,
               "Pending reverse MoveAbsolute should become active segment");
    ASSERT_TRUE(!fb->_directBlendContext.active,
               "Blend context should remain inactive across reverse buffered completion");

    memset(&second, 0, sizeof(second));
    IEC_VAL(second.EN) = true;
    IEC_VAL(second.EXECUTE) = true;
    second.EXECUTE0.value = true;
    IEC_VAL(second.AXISID) = 0;
    IEC_VAL(second.POSITION) = 0.0f;
    IEC_VAL(second.VELOCITY) = 8.0f;
    IEC_VAL(second.ACCELERATION) = 100.0f;
    IEC_VAL(second.DECELERATION) = 100.0f;
    IEC_VAL(second.DIRECTION) = 2;  /* NEGATIVE */
    IEC_VAL(second.BUFFERMODE) = HYD_BUFFER_MODE_BLENDING_HIGH;
    __mcl_cmd_MoveAbsolute(&second);
    ASSERT_TRUE(IEC_VAL(second.COMMANDABORTED) == false,
               "Reverse buffered completion should not raise COMMANDABORTED for the second command");
}

static void test_live_update_refreshes_blend_context(void) {
    HYD_MotionControlFB* fb;
    HYD_LiveUpdateRequest request;

    fb = start_blend_pair(HYD_BUFFER_MODE_BLENDING_PREVIOUS, 20.0f, 8.0f);
    ASSERT_TRUE(fb != NULL, "Axis 0 control FB should exist for live-update refresh test");
    ASSERT_TRUE(fb->_directBlendContext.active,
               "Blend context should be active before live update");
    ASSERT_TRUE(fabs(fb->_directBlendContext.switchPosition - 100.0f) < 0.001f,
               "Initial blend switch position should match active target");
    ASSERT_TRUE(fabs(fb->_directBlendContext.blendVelocity - 20.0f) < 0.001f,
               "Initial blend velocity should match BlendingPrevious selection");

    memset(&request, 0, sizeof(request));
    request.flags = HYD_LIVE_UPDATE_TARGET_POSITION |
                    HYD_LIVE_UPDATE_MAX_VELOCITY |
                    HYD_LIVE_UPDATE_ACCELERATION |
                    HYD_LIVE_UPDATE_DECELERATION;
    request.ownerKind = HYD_DIRECT_CMD_MOVE_ABSOLUTE;
    request.ownerTicket = fb->_directOwnerTicket;
    request.targetPosition = 140.0f;
    request.maxVelocity = 12.0f;
    request.maxAcceleration = 100.0f;
    request.maxDeceleration = 100.0f;

    ASSERT_TRUE(HYD_MotionControlFB_ApplyLiveUpdate(fb, &request),
               "Live update should succeed for active direct MoveAbsolute");

    ASSERT_TRUE(fb->_directBlendContext.active,
               "Blend context should remain active after live update");
    ASSERT_TRUE(fabs(fb->_directBlendContext.switchPosition - 140.0f) < 0.001f,
               "Blend switch position should follow updated target position");
    ASSERT_TRUE(fabs(fb->_directBlendContext.blendVelocity - 12.0f) < 0.001f,
               "BlendingPrevious blend velocity should follow updated active maxVelocity");
}

static void test_blend_pending_rejected_while_stopping(void) {
    HYD_MotionControlFB* fb;
    HYD_MOVEABSOLUTE first;
    HYD_MOVEABSOLUTE second;

    __HydMotion_framework_Init();
    ensure_axes_allocated(1);
    fb = __MK_GetPublic_MotionControlFB(0);
    ASSERT_TRUE(fb != NULL, "Axis 0 control FB should exist for stopping-reject test");

    memset(&first, 0, sizeof(first));
    IEC_VAL(first.EN) = true;
    IEC_VAL(first.EXECUTE) = true;
    first.EXECUTE0.value = false;
    IEC_VAL(first.AXISID) = 0;
    IEC_VAL(first.POSITION) = 100.0f;
    IEC_VAL(first.VELOCITY) = 20.0f;
    IEC_VAL(first.ACCELERATION) = 100.0f;
    IEC_VAL(first.DECELERATION) = 100.0f;
    IEC_VAL(first.DIRECTION) = 1;
    IEC_VAL(first.BUFFERMODE) = HYD_BUFFER_MODE_ABORT;
    __mcl_cmd_MoveAbsolute(&first);
    __HydMotion_framework_Publish();

    /* Isolate the submission-side stopping gate by forcing the flag directly.
     * The normal Stop -> _isStopping transition is exercised by other tests. */
    fb->_isStopping = true;

    memset(&second, 0, sizeof(second));
    IEC_VAL(second.EN) = true;
    IEC_VAL(second.EXECUTE) = true;
    second.EXECUTE0.value = false;
    IEC_VAL(second.AXISID) = 0;
    IEC_VAL(second.POSITION) = 200.0f;
    IEC_VAL(second.VELOCITY) = 8.0f;
    IEC_VAL(second.ACCELERATION) = 100.0f;
    IEC_VAL(second.DECELERATION) = 100.0f;
    IEC_VAL(second.DIRECTION) = 1;
    IEC_VAL(second.BUFFERMODE) = HYD_BUFFER_MODE_BLENDING_NEXT;
    __mcl_cmd_MoveAbsolute(&second);

    ASSERT_TRUE(IEC_VAL(second.ERROR) == true,
               "Blend MoveAbsolute should report ERROR when submitted during Stopping");
    ASSERT_TRUE(!fb->_directPendingValid,
               "Blend MoveAbsolute should not occupy pending slot during Stopping");
    ASSERT_TRUE(!fb->_directBlendContext.active,
               "Blend context should remain cleared when submission is rejected during Stopping");
}

static void test_runtime_fault_clears_blend_pending_slot(void) {
    HYD_MotionControlFB* fb;

    fb = start_blend_pair(HYD_BUFFER_MODE_BLENDING_NEXT, 20.0f, 8.0f);
    ASSERT_TRUE(fb != NULL, "Axis 0 control FB should exist for fault-teardown test");
    ASSERT_TRUE(fb->_directPendingValid,
               "Pending blend command should be present before runtime fault");
    ASSERT_TRUE(fb->_directBlendContext.active,
               "Blend context should be active before runtime fault");

    HYD_StateReporter_ReportFault(fb,
                                  HYD_DIAG_CODE_SENSOR_FAULT,
                                  fb->AXIS_REF.timestamp,
                                  &fb->_activeSegment,
                                  &fb->STATE.references);

    ASSERT_TRUE(fb->FB_STATE == HYD_FB_STATE_FAULT,
               "Runtime fault entry should drive FB_STATE to FAULT");
    ASSERT_TRUE(!fb->_directPendingValid,
               "Runtime fault entry should clear pending direct slot");
    ASSERT_TRUE(!fb->_directBlendContext.active,
               "Runtime fault entry should clear blend context");
}

static void test_stop_completion_clears_blend_pending_slot(void) {
    HYD_MotionControlFB* fb;

    fb = start_blend_pair(HYD_BUFFER_MODE_BLENDING_NEXT, 20.0f, 8.0f);
    ASSERT_TRUE(fb != NULL, "Axis 0 control FB should exist for stop-teardown test");
    ASSERT_TRUE(fb->_directPendingValid,
               "Pending blend command should be present before Stop");
    ASSERT_TRUE(fb->_directBlendContext.active,
               "Blend context should be active before Stop");

    ASSERT_TRUE(HYD_MotionControlFB_Stop(fb, fb->AXIS_REF.timestamp, 200.0),
               "Stop request should be accepted while blended pending is queued");

    fb->AXIS_REF.velocity = 0.0f;
    fb->AXIS_REF.timestamp += 1.0f;
    __HydMotion_framework_Publish();
    fb->AXIS_REF.timestamp += 1.0f;
    __HydMotion_framework_Publish();

    ASSERT_TRUE(fb->FB_STATE == HYD_FB_STATE_DONE,
               "Stop completion should reach DONE");
    ASSERT_TRUE(!fb->_directPendingValid,
               "Stop completion should clear pending direct slot");
    ASSERT_TRUE(!fb->_directBlendContext.active,
               "Stop completion should clear blend context");
}

int main(void) {
    printf("=== Motion Interface Arbitration Tests ===\n\n");

    test_moveabsolute_preempted_by_stop();
    test_buffered_moveabsolute_reports_busy_but_not_active_while_waiting();
    test_third_same_axis_moveabsolute_is_rejected_when_one_active_and_one_pending_exist();
    test_rejected_third_moveabsolute_stays_local_under_persistent_execute_high();
    test_moveabsolute_preempted_by_movevelocity();
    test_movevelocity_preempted_by_moveabsolute();
    test_pressurehandle_preempted_by_stop();
    test_movevelocity_preempted_by_pressurehandle();
    test_multi_axis_isolation();
    test_stop_success_then_new_command_starts();
    test_self_preemption_same_fb_twice();
    test_buffered_moveabsolute_waits_without_preempting_active_owner();
    test_buffered_endless_movevelocity_degrades_to_abort_takeover();
    test_blending_modes_select_distinct_through_velocities();
    test_blended_front_segment_keeps_nonzero_velocity_near_switch();
    test_blended_cutover_preserves_planner_state();
    test_blended_cutover_keeps_nonzero_output_velocity_same_scan();
    test_moveabsolute_direct_start_latches_ownership_on_rising_edge();
    test_blend_context_requires_compatible_moveabsolute_direction();
    test_previous_command_loses_ownership_after_reset();
    test_preemption_chain_three_commands();
    test_never_activated_fb_no_false_commandaborted();
    test_direct_command_preempts_moveprofile();
    test_moveprofile_loses_activity_when_direct_command_takes_over();
    test_moveprofile_loses_activity_after_reset_takeover();
    test_direct_command_starts_cleanly_after_recipe_done();
    test_moveprofile_rejects_oversized_recipe_load();
    test_stop_completion_clears_blend_pending_slot();
    test_runtime_fault_clears_blend_pending_slot();
    test_blend_pending_rejected_while_stopping();
    test_live_update_refreshes_blend_context();
    test_reverse_blend_pending_completes_as_buffered();

    printf("\n=== Results: %d/%d passed ===\n", tests_passed, tests_run);
    return (tests_passed == tests_run) ? 0 : 1;
}
