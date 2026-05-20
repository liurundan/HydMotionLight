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
}

static int drive_stop_until_done(int axisIndex, HYD_STOP* stop, int maxSteps) {
    HYD_MotionControlFB* fb = __MK_GetPublic_MotionControlFB(axisIndex);

    if (fb == NULL) {
        return -1;
    }

    for (int step = 0; step < maxSteps; step++) {
        fb->AXIS_REF.timestamp += 0.01f;
        if (fb->_simFeedback.valid) {
            fb->AXIS_REF.velocity = fb->_simFeedback.targetVelocity;
            fb->AXIS_REF.position += fb->AXIS_REF.velocity * 0.01f;
            fb->AXIS_REF.flow = fb->_simFeedback.targetFlow;
            fb->AXIS_REF.pressure = fb->_simFeedback.targetPressure;
        }

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

    ASSERT_TRUE(stopDoneStep > 1,
               "Stop should require multiple cycles before DONE after taking over");
    ASSERT_TRUE(IEC_VAL(stop.BUSY) == false,
               "Stop BUSY should be false once DONE");
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
    IEC_VAL(ma.DIRECTION) = -1;
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

static void test_moveprofile_does_not_self_abort_on_recipe_nextsegment(void) {
    HYD_CREATEMOTION cm;
    HYD_MOVEPROFILE mp;
    HYD_AXISMOTION motion;
    HYD_MotionControlFB* fb;
    HYD_MotionSegment recipe[2];
    int step;

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

    ASSERT_TRUE(HYD_MotionControlFB_LoadRecipe(fb, recipe, 2U),
               "Two-step recipe should preload successfully");

    memset(&mp, 0, sizeof(mp));
    memset(&motion, 0, sizeof(motion));
    IEC_VAL(mp.EN) = true;
    IEC_VAL(mp.EXECUTE) = true;
    mp.EXECUTE0.value = false;
    IEC_VAL(mp.AXISID) = IEC_VAL(cm.AXISID);
    __SET_VAR(mp., MOTION, , motion);

    __mcl_cmd_MoveProfile(&mp);
    __HydMotion_framework_Publish();
    IEC_VAL(mp.EXECUTE) = true;
    mp.EXECUTE0.value = true;
    __mcl_cmd_MoveProfile(&mp);

    for (step = 0; step < 100; step++) {
        motion = __GET_VAR(mp.MOTION);
        motion.TIMESTAMP += 0.001f;
        __SET_VAR(mp., MOTION, , motion);
        __HydMotion_framework_Publish();
        IEC_VAL(mp.EXECUTE) = true;
        mp.EXECUTE0.value = true;
        __mcl_cmd_MoveProfile(&mp);
        if (fb->SEGMENT_COMPLETED) {
            break;
        }
    }

    ASSERT_TRUE(fb->SEGMENT_COMPLETED == true,
               "First recipe segment should complete before NextSegment");
    ASSERT_TRUE(IEC_VAL(mp.COMMANDABORTED) == false,
               "MoveProfile should not self-abort on normal first-segment completion");

    ASSERT_TRUE(HYD_MotionControlFB_NextSegment(fb, fb->AXIS_REF.timestamp),
               "NextSegment should be accepted for the completed recipe");
    __HydMotion_framework_Publish();

    motion = __GET_VAR(mp.MOTION);
    motion.TIMESTAMP += 0.001f;
    __SET_VAR(mp., MOTION, , motion);
    IEC_VAL(mp.EXECUTE) = true;
    mp.EXECUTE0.value = true;
    __mcl_cmd_MoveProfile(&mp);

    ASSERT_TRUE(IEC_VAL(mp.COMMANDABORTED) == false,
               "MoveProfile should not raise COMMANDABORTED on normal recipe NextSegment");
    ASSERT_TRUE(IEC_VAL(mp.BUSY) == true || IEC_VAL(mp.ACTIVE) == true,
               "MoveProfile should remain in a live lifecycle on the next recipe segment");
}

int main(void) {
    printf("=== Motion Interface Arbitration Tests ===\n\n");

    test_moveabsolute_preempted_by_stop();
    test_moveabsolute_preempted_by_movevelocity();
    test_movevelocity_preempted_by_moveabsolute();
    test_pressurehandle_preempted_by_stop();
    test_movevelocity_preempted_by_pressurehandle();
    test_multi_axis_isolation();
    test_stop_success_then_new_command_starts();
    test_self_preemption_same_fb_twice();
    test_buffered_moveabsolute_waits_without_preempting_active_owner();
    test_buffered_endless_movevelocity_degrades_to_abort_takeover();
    test_previous_command_loses_ownership_after_reset();
    test_preemption_chain_three_commands();
    test_never_activated_fb_no_false_commandaborted();
    test_direct_command_preempts_moveprofile();
    test_moveprofile_loses_activity_when_direct_command_takes_over();
    test_moveprofile_loses_activity_after_reset_takeover();
    test_direct_command_starts_cleanly_after_recipe_done();
    test_moveprofile_does_not_self_abort_on_recipe_nextsegment();

    printf("\n=== Results: %d/%d passed ===\n", tests_passed, tests_run);
    return (tests_passed == tests_run) ? 0 : 1;
}
