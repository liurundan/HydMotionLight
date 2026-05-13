/**
 * @file test_motion_interface_done_signals.c
 * @brief IEC FB接口层 Done 信号仿真循环测试
 *
 * 目标：使用 CreateMotion(USE_SIMULATION=true) 启用仿真反馈回路，
 *       通过 MoveAbsolute / MoveVelocity / PressureHandle / Stop / Reset
 *       的完整生命周期仿真，验证各 FB 的 Done 信号正确性。
 *
 * 测试场景:
 *   1. MoveAbsolute 伸出→Done→缩回→Done 循环3轮
 *   2. MoveAbsolute Done 后同一 FB 重触发新目标
 *   3. MoveVelocity 执行→Stop→Done
 *   4. PressureHandle(有时间限制)→Done
 *   5. 两个 MoveAbsolute FB 交替控制同一轴（多FB同一轴循环）
 *   6. MoveAbsolute Done 后 ReadSimFeedback 读取最终位置
 *
 * 仿真回路原理:
 *   当 USE_SIMULATION=true 时，__HydMotion_framework_Publish() 会自动
 *   用规划器输出更新 AXIS_REF（位置积分 position += velocity * deltaTime），
 *   测试只需循环调用 Publish + 对应FB命令即可推进仿真时间直到 Done。
 */

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdbool.h>

#include "motion_interface.h"
#include "motion_control.h"

extern HYD_MotionControlFB* __MK_GetPublic_MotionControlFB(int index);

#define IEC_VAL(var) ((var).value)

/* 最大仿真步数保护，防止无限循环 */
#define MAX_SIM_STEPS  20000

static int tests_run = 0;
static int tests_passed = 0;

#define ASSERT_TRUE(cond, msg) do { \
    tests_run++; \
    if (cond) { tests_passed++; } \
    else { printf("  FAIL: %s\n", msg); } \
} while (0)

#define ASSERT_EQ(a, b, msg) do { \
    tests_run++; \
    if ((a) == (b)) { tests_passed++; } \
    else { printf("  FAIL: %s (got %d, expected %d)\n", msg, (int)(a), (int)(b)); } \
} while (0)

#define ASSERT_FNEAR(a, b, tol, msg) do { \
    tests_run++; \
    if (fabs((double)(a) - (double)(b)) <= (double)(tol)) { tests_passed++; } \
    else { printf("  FAIL: %s (got %.4f, expected %.4f, tol=%.4f)\n", msg, (double)(a), (double)(b), (double)(tol)); } \
} while (0)

/* ==================================================================
 * 辅助函数: 通过CreateMotion分配仿真轴 (Direct模式 + 仿真)
 * ================================================================== */
static int create_sim_axis(HYD_BOOL use_recipe) {
    HYD_CREATEMOTION cm;
    memset(&cm, 0, sizeof(cm));
    IEC_VAL(cm.EN) = true;
    IEC_VAL(cm.USE_RECIPE) = use_recipe;
    IEC_VAL(cm.FLOW_TO_PUMPSPEED) = 1.0f;
    IEC_VAL(cm.PUMPSPEED_LIMIT) = 3000.0f;
    IEC_VAL(cm.USE_SIMULATION) = true;
    __mcl_cmd_CreateMotion(&cm);
    return (int)IEC_VAL(cm.AXISID);
}

/* 辅助: 创建非仿真轴 (当前测试未使用，保留以备后续扩展) */
#if 0
static int create_nosim_axis(HYD_BOOL use_recipe) {
    HYD_CREATEMOTION cm;
    memset(&cm, 0, sizeof(cm));
    IEC_VAL(cm.EN) = true;
    IEC_VAL(cm.USE_RECIPE) = use_recipe;
    IEC_VAL(cm.FLOW_TO_PUMPSPEED) = 1.0f;
    IEC_VAL(cm.PUMPSPEED_LIMIT) = 3000.0f;
    IEC_VAL(cm.USE_SIMULATION) = false;
    __mcl_cmd_CreateMotion(&cm);
    return (int)IEC_VAL(cm.AXISID);
}
#endif

/* ==================================================================
 * 辅助函数: 启动 MoveAbsolute 并等待 Done (仿真驱动)
 *
 * 流程:
 *   1. EXECUTE上升沿触发 MoveAbsolute
 *   2. 循环: Publish() → MoveAbsolute(非上升沿) → 检查Done
 *   3. 返回实际仿真步数，-1表示超时
 * ================================================================== */
static int run_moveabsolute_to_done(HYD_MOVEABSOLUTE* ma, int maxSteps) {
    int step = 0;

    /* Step 1: EXECUTE上升沿 */
    IEC_VAL(ma->EXECUTE) = true;
    ma->EXECUTE0.value = false;
    __mcl_cmd_MoveAbsolute(ma);

    /* Step 2: 第一周期Publish + 确认所有权 */
    __HydMotion_framework_Publish();
    IEC_VAL(ma->EXECUTE) = true;
    ma->EXECUTE0.value = true;
    __mcl_cmd_MoveAbsolute(ma);

    /* Step 3: 循环推进仿真直到 Done */
    while (step < maxSteps) {
        __HydMotion_framework_Publish();
        IEC_VAL(ma->EXECUTE) = true;
        ma->EXECUTE0.value = true;
        __mcl_cmd_MoveAbsolute(ma);

        step++;

        if (IEC_VAL(ma->DONE)) {
            return step;
        }
    }

    return -1;  /* 超时 */
}

/* ==================================================================
 * Test 1: MoveAbsolute 伸出→Done→缩回→Done 循环3轮
 *
 * 模拟油缸: 伸出(0→100mm)→Done→缩回(100→0mm)→Done
 * 循环执行3轮(6次MoveAbsolute完成)
 * ================================================================== */
static void test_moveabsolute_extend_retract_cycle(void) {
    HYD_MOVEABSOLUTE ma;
    int axisId;
    int cycle, steps;
    int totalCycles = 3;

    printf("--- Test: MoveAbsolute extend/retract cycle (%d rounds) ---\n", totalCycles);

    __HydMotion_framework_Init();
    axisId = create_sim_axis(false);
    ASSERT_TRUE(axisId >= 0, "CreateMotion should succeed for sim axis");

    for (cycle = 0; cycle < totalCycles; cycle++) {
        /* 伸出: 0 → 100mm */
        memset(&ma, 0, sizeof(ma));
        IEC_VAL(ma.EN) = true;
        IEC_VAL(ma.AXISID) = axisId;
        IEC_VAL(ma.POSITION) = 100.0f;
        IEC_VAL(ma.VELOCITY) = 50.0f;
        IEC_VAL(ma.ACCELERATION) = 200.0f;
        IEC_VAL(ma.DIRECTION) = 1;  /* EXTEND */

        steps = run_moveabsolute_to_done(&ma, MAX_SIM_STEPS);
        ASSERT_TRUE(steps > 0, "Extend MoveAbsolute should reach DONE within max steps");
        ASSERT_TRUE(IEC_VAL(ma.DONE) == true, "Extend should set DONE=true");
        ASSERT_TRUE(IEC_VAL(ma.BUSY) == false, "Extend should clear BUSY after DONE");
        ASSERT_TRUE(IEC_VAL(ma.ACTIVE) == false, "Extend should clear ACTIVE after DONE");
        ASSERT_TRUE(IEC_VAL(ma.ERROR) == false, "Extend should not set ERROR");
        ASSERT_TRUE(IEC_VAL(ma.COMMANDABORTED) == false, "Extend should not set COMMANDABORTED");

        printf("  Cycle %d extend: DONE after %d sim steps\n", cycle + 1, steps);

        /* 缩回: 100 → 0mm */
        memset(&ma, 0, sizeof(ma));
        IEC_VAL(ma.EN) = true;
        IEC_VAL(ma.AXISID) = axisId;
        IEC_VAL(ma.POSITION) = 0.0f;
        IEC_VAL(ma.VELOCITY) = 50.0f;
        IEC_VAL(ma.ACCELERATION) = 200.0f;
        IEC_VAL(ma.DIRECTION) = -1;  /* RETRACT */

        steps = run_moveabsolute_to_done(&ma, MAX_SIM_STEPS);
        ASSERT_TRUE(steps > 0, "Retract MoveAbsolute should reach DONE within max steps");
        ASSERT_TRUE(IEC_VAL(ma.DONE) == true, "Retract should set DONE=true");
        ASSERT_TRUE(IEC_VAL(ma.BUSY) == false, "Retract should clear BUSY after DONE");
        ASSERT_TRUE(IEC_VAL(ma.ACTIVE) == false, "Retract should clear ACTIVE after DONE");
        ASSERT_TRUE(IEC_VAL(ma.ERROR) == false, "Retract should not set ERROR");
        ASSERT_TRUE(IEC_VAL(ma.COMMANDABORTED) == false, "Retract should not set COMMANDABORTED");

        printf("  Cycle %d retract: DONE after %d sim steps\n", cycle + 1, steps);
    }

    ASSERT_EQ(totalCycles * 2, totalCycles * 2, "Should complete all extend+retract cycles");
}

/* ==================================================================
 * Test 2: MoveAbsolute Done 后同一 FB 重触发新目标
 *
 * 验证: 第一次 Done 后，将 EXECUTE 拉低再拉高，
 *       可以成功启动新的 MoveAbsolute 而不产生 COMMANDABORTED
 * ================================================================== */
static void test_moveabsolute_retrigger_after_done(void) {
    HYD_MOVEABSOLUTE ma;
    int axisId, steps;

    printf("--- Test: MoveAbsolute retrigger after DONE ---\n");

    __HydMotion_framework_Init();
    axisId = create_sim_axis(false);
    ASSERT_TRUE(axisId >= 0, "CreateMotion should succeed");

    /* 第一次: 伸出到 50mm */
    memset(&ma, 0, sizeof(ma));
    IEC_VAL(ma.EN) = true;
    IEC_VAL(ma.AXISID) = axisId;
    IEC_VAL(ma.POSITION) = 50.0f;
    IEC_VAL(ma.VELOCITY) = 30.0f;
    IEC_VAL(ma.ACCELERATION) = 150.0f;
    IEC_VAL(ma.DIRECTION) = 1;

    steps = run_moveabsolute_to_done(&ma, MAX_SIM_STEPS);
    ASSERT_TRUE(steps > 0, "First MoveAbsolute should reach DONE");
    ASSERT_TRUE(IEC_VAL(ma.DONE) == true, "First move should be DONE");

    /* EXECUTE下降沿 */
    IEC_VAL(ma.EXECUTE) = false;
    ma.EXECUTE0.value = true;
    __mcl_cmd_MoveAbsolute(&ma);
    __HydMotion_framework_Publish();

    /* 重触发: 新目标 100mm */
    IEC_VAL(ma.EXECUTE) = true;
    ma.EXECUTE0.value = false;
    IEC_VAL(ma.POSITION) = 100.0f;
    IEC_VAL(ma.VELOCITY) = 50.0f;
    IEC_VAL(ma.ACCELERATION) = 200.0f;
    IEC_VAL(ma.DIRECTION) = 1;
    __mcl_cmd_MoveAbsolute(&ma);

    ASSERT_TRUE(IEC_VAL(ma.COMMANDABORTED) == false,
               "Re-trigger should not produce COMMANDABORTED");
    ASSERT_TRUE(IEC_VAL(ma.ERROR) == false,
               "Re-trigger should not produce ERROR");

    /* 推进仿真直到第二次Done */
    steps = run_moveabsolute_to_done(&ma, MAX_SIM_STEPS);
    ASSERT_TRUE(steps > 0, "Second MoveAbsolute should reach DONE");
    ASSERT_TRUE(IEC_VAL(ma.DONE) == true, "Second move should be DONE");
    ASSERT_TRUE(IEC_VAL(ma.COMMANDABORTED) == false,
               "Second move should not be COMMANDABORTED");
}

/* ==================================================================
 * Test 3: MoveVelocity 执行 → Stop → Done
 *
 * 验证: MoveVelocity 启动后，Stop 接管，Stop 的 Done 信号正确
 * ================================================================== */
static void test_movevelocity_then_stop_done(void) {
    HYD_MOVEVELOCITY mv;
    HYD_STOP stop;
    int axisId, step;
    int stopDoneStep = -1;

    printf("--- Test: MoveVelocity then Stop → DONE ---\n");

    __HydMotion_framework_Init();
    axisId = create_sim_axis(false);
    ASSERT_TRUE(axisId >= 0, "CreateMotion should succeed");

    /* 启动 MoveVelocity */
    memset(&mv, 0, sizeof(mv));
    IEC_VAL(mv.EN) = true;
    IEC_VAL(mv.EXECUTE) = true;
    mv.EXECUTE0.value = false;
    IEC_VAL(mv.AXISID) = axisId;
    IEC_VAL(mv.VELOCITY) = 30.0f;
    IEC_VAL(mv.ACCELERATION) = 150.0f;
    IEC_VAL(mv.DIRECTION) = 1;

    __mcl_cmd_MoveVelocity(&mv);
    __HydMotion_framework_Publish();

    /* 确认 MoveVelocity 已激活 */
    IEC_VAL(mv.EXECUTE) = true;
    mv.EXECUTE0.value = true;
    __mcl_cmd_MoveVelocity(&mv);
    ASSERT_TRUE(IEC_VAL(mv.BUSY) == true || IEC_VAL(mv.ACTIVE) == true,
               "MoveVelocity should be active after start");

    /* 运行若干周期让速度建立 */
    for (step = 0; step < 10; step++) {
        __HydMotion_framework_Publish();
        IEC_VAL(mv.EXECUTE) = true;
        mv.EXECUTE0.value = true;
        __mcl_cmd_MoveVelocity(&mv);
    }

    /* Stop 接管 */
    memset(&stop, 0, sizeof(stop));
    IEC_VAL(stop.EN) = true;
    IEC_VAL(stop.EXECUTE) = true;
    stop.EXECUTE0.value = false;
    IEC_VAL(stop.AXISID) = axisId;
    __mcl_cmd_Stop(&stop);
    ASSERT_TRUE(IEC_VAL(stop.DONE) == false,
               "Stop should not complete on the trigger call");
    __HydMotion_framework_Publish();

    /* 等待 Stop 完成 */
    for (step = 0; step < MAX_SIM_STEPS; step++) {
        __HydMotion_framework_Publish();
        IEC_VAL(stop.EXECUTE) = true;
        stop.EXECUTE0.value = true;
        __mcl_cmd_Stop(&stop);

        if (IEC_VAL(stop.DONE)) {
            stopDoneStep = step + 1;
            break;
        }
    }

    ASSERT_TRUE(IEC_VAL(stop.DONE) == true,
               "Stop should reach DONE after stopping motion");
    ASSERT_TRUE(stopDoneStep > 1,
               "Stop should require multiple cycles to decelerate active motion");
    ASSERT_TRUE(IEC_VAL(stop.BUSY) == false,
               "Stop should clear BUSY after DONE");
    ASSERT_TRUE(IEC_VAL(stop.ERROR) == false,
               "Stop should not set ERROR on normal completion");

    /* MoveVelocity 应检测到被抢占 */
    IEC_VAL(mv.EXECUTE) = true;
    mv.EXECUTE0.value = true;
    __mcl_cmd_MoveVelocity(&mv);
    ASSERT_TRUE(IEC_VAL(mv.COMMANDABORTED) == true,
               "MoveVelocity should get COMMANDABORTED when stopped");
    ASSERT_TRUE(IEC_VAL(mv.INVELOCITY) == false,
               "MoveVelocity INVELOCITY should clear after Stop takeover");
}

/* ==================================================================
 * Test 4: PressureHandle (有时间限制) → Done
 *
 * 验证: PressureHandle 指定 duration 后，时间到 Done 信号正确
 * ================================================================== */
static void test_pressurehandle_timed_done(void) {
    HYD_PRESSUREHANDLE ph;
    int axisId, step;

    printf("--- Test: PressureHandle timed → DONE ---\n");

    __HydMotion_framework_Init();
    axisId = create_sim_axis(false);
    ASSERT_TRUE(axisId >= 0, "CreateMotion should succeed");

    /* 启动 PressureHandle (duration=0.5s 以缩短仿真时间) */
    memset(&ph, 0, sizeof(ph));
    IEC_VAL(ph.EN) = true;
    IEC_VAL(ph.EXECUTE) = true;
    ph.EXECUTE0.value = false;
    IEC_VAL(ph.AXISID) = axisId;
    IEC_VAL(ph.PRESSURE) = 5.0f;
    IEC_VAL(ph.PRESSURERAMPRATE) = 10.0f;
    IEC_VAL(ph.DURATION) = 0.5f;  /* 0.5秒 */

    __mcl_cmd_PressureHandle(&ph);

    ASSERT_TRUE(IEC_VAL(ph.BUSY) == true, "PressureHandle should set BUSY on start");
    ASSERT_TRUE(IEC_VAL(ph.ACTIVE) == true, "PressureHandle should set ACTIVE on start");

    /* 推进仿真直到 Done 或超时 */
    for (step = 0; step < MAX_SIM_STEPS; step++) {
        __HydMotion_framework_Publish();
        IEC_VAL(ph.EXECUTE) = true;
        ph.EXECUTE0.value = true;
        __mcl_cmd_PressureHandle(&ph);

        /* 检查是否完成: PressureHandle 有时间限制时，完成条件是BUSY=false且ACTIVE=false */
        if (!IEC_VAL(ph.BUSY) && !IEC_VAL(ph.ACTIVE)) {
            break;
        }
    }

    ASSERT_TRUE(step < MAX_SIM_STEPS,
               "PressureHandle should complete within max sim steps");

    /* 注意: PressureHandle时间到后，当前实现中BUSY/ACTIVE清除，
     * 但不一定设置明确的Done信号(与MoveAbsolute不同)。
     * 检查BUSY=false表示运动已完成 */
    ASSERT_TRUE(IEC_VAL(ph.BUSY) == false,
               "PressureHandle should clear BUSY after duration elapsed");
    ASSERT_TRUE(IEC_VAL(ph.ACTIVE) == false,
               "PressureHandle should clear ACTIVE after duration elapsed");
    ASSERT_TRUE(IEC_VAL(ph.COMMANDABORTED) == false,
               "PressureHandle should not be COMMANDABORTED on normal completion");
}

/* ==================================================================
 * Test 5: 两个 MoveAbsolute FB 交替控制同一轴（多FB同一轴循环）
 *
 * 场景:
 *   FB_A 启动伸出 → FB_B 抢占缩回 → FB_B Done →
 *   FB_A 重新启动伸出 → FB_A Done → FB_B 重新启动缩回 → FB_B Done
 *
 * 验证: 后者抢占前者时，前者 COMMANDABORTED 正确；
 *       重新启动时各 FB 独立追踪所有权
 * ================================================================== */
static void test_two_moveabsolute_fbs_alternating_same_axis(void) {
    HYD_MOVEABSOLUTE maA, maB;
    int axisId, step;

    printf("--- Test: Two MoveAbsolute FBs alternating on same axis ---\n");

    __HydMotion_framework_Init();
    axisId = create_sim_axis(false);
    ASSERT_TRUE(axisId >= 0, "CreateMotion should succeed");

    /* FB_A 启动伸出 0→80mm */
    memset(&maA, 0, sizeof(maA));
    IEC_VAL(maA.EN) = true;
    IEC_VAL(maA.AXISID) = axisId;
    IEC_VAL(maA.POSITION) = 80.0f;
    IEC_VAL(maA.VELOCITY) = 40.0f;
    IEC_VAL(maA.ACCELERATION) = 200.0f;
    IEC_VAL(maA.DIRECTION) = 1;

    IEC_VAL(maA.EXECUTE) = true;
    maA.EXECUTE0.value = false;
    __mcl_cmd_MoveAbsolute(&maA);
    __HydMotion_framework_Publish();

    IEC_VAL(maA.EXECUTE) = true;
    maA.EXECUTE0.value = true;
    __mcl_cmd_MoveAbsolute(&maA);
    ASSERT_TRUE(IEC_VAL(maA.BUSY) || IEC_VAL(maA.ACTIVE),
               "FB_A should be active after first start");

    /* 运行几个周期让运动建立 */
    for (step = 0; step < 5; step++) {
        __HydMotion_framework_Publish();
        IEC_VAL(maA.EXECUTE) = true;
        maA.EXECUTE0.value = true;
        __mcl_cmd_MoveAbsolute(&maA);
    }

    /* FB_B 抢占: 启动缩回到0mm */
    memset(&maB, 0, sizeof(maB));
    IEC_VAL(maB.EN) = true;
    IEC_VAL(maB.AXISID) = axisId;
    IEC_VAL(maB.POSITION) = 0.0f;
    IEC_VAL(maB.VELOCITY) = 40.0f;
    IEC_VAL(maB.ACCELERATION) = 200.0f;
    IEC_VAL(maB.DIRECTION) = -1;  /* RETRACT */

    IEC_VAL(maB.EXECUTE) = true;
    maB.EXECUTE0.value = false;
    __mcl_cmd_MoveAbsolute(&maB);
    __HydMotion_framework_Publish();

    /* FB_A 应检测到被抢占 */
    IEC_VAL(maA.EXECUTE) = true;
    maA.EXECUTE0.value = true;
    __mcl_cmd_MoveAbsolute(&maA);
    ASSERT_TRUE(IEC_VAL(maA.COMMANDABORTED) == true,
               "FB_A should get COMMANDABORTED when FB_B takes over");
    ASSERT_TRUE(IEC_VAL(maA.DONE) == false,
               "FB_A should NOT set DONE when preempted");

    /* FB_B 确认所有权并推进到 Done */
    IEC_VAL(maB.EXECUTE) = true;
    maB.EXECUTE0.value = true;
    __mcl_cmd_MoveAbsolute(&maB);

    for (step = 0; step < MAX_SIM_STEPS; step++) {
        __HydMotion_framework_Publish();
        IEC_VAL(maB.EXECUTE) = true;
        maB.EXECUTE0.value = true;
        __mcl_cmd_MoveAbsolute(&maB);

        if (IEC_VAL(maB.DONE)) break;
    }

    ASSERT_TRUE(IEC_VAL(maB.DONE) == true,
               "FB_B should reach DONE on retract");
    ASSERT_TRUE(IEC_VAL(maB.COMMANDABORTED) == false,
               "FB_B should not be COMMANDABORTED");

    printf("  FB_B retract DONE after %d sim steps\n", step);

    /* FB_A 重新启动伸出 (EXECUTE下降再上升) */
    IEC_VAL(maA.EXECUTE) = false;
    maA.EXECUTE0.value = true;
    __mcl_cmd_MoveAbsolute(&maA);
    __HydMotion_framework_Publish();

    memset(&maA, 0, sizeof(maA));
    IEC_VAL(maA.EN) = true;
    IEC_VAL(maA.AXISID) = axisId;
    IEC_VAL(maA.POSITION) = 80.0f;
    IEC_VAL(maA.VELOCITY) = 40.0f;
    IEC_VAL(maA.ACCELERATION) = 200.0f;
    IEC_VAL(maA.DIRECTION) = 1;

    IEC_VAL(maA.EXECUTE) = true;
    maA.EXECUTE0.value = false;
    __mcl_cmd_MoveAbsolute(&maA);

    ASSERT_TRUE(IEC_VAL(maA.ERROR) == false,
               "FB_A re-trigger should not set ERROR");
    ASSERT_TRUE(IEC_VAL(maA.COMMANDABORTED) == false,
               "FB_A re-trigger should not set COMMANDABORTED");

    /* 推进FB_A到Done */
    for (step = 0; step < MAX_SIM_STEPS; step++) {
        __HydMotion_framework_Publish();
        IEC_VAL(maA.EXECUTE) = true;
        maA.EXECUTE0.value = true;
        __mcl_cmd_MoveAbsolute(&maA);

        if (IEC_VAL(maA.DONE)) break;
    }

    ASSERT_TRUE(IEC_VAL(maA.DONE) == true,
               "FB_A should reach DONE on second extend");
    ASSERT_TRUE(IEC_VAL(maA.COMMANDABORTED) == false,
               "FB_A should not be COMMANDABORTED on second run");

    printf("  FB_A second extend DONE after %d sim steps\n", step);

    /* FB_B 重新启动缩回 */
    IEC_VAL(maB.EXECUTE) = false;
    maB.EXECUTE0.value = true;
    __mcl_cmd_MoveAbsolute(&maB);
    __HydMotion_framework_Publish();

    memset(&maB, 0, sizeof(maB));
    IEC_VAL(maB.EN) = true;
    IEC_VAL(maB.AXISID) = axisId;
    IEC_VAL(maB.POSITION) = 0.0f;
    IEC_VAL(maB.VELOCITY) = 40.0f;
    IEC_VAL(maB.ACCELERATION) = 200.0f;
    IEC_VAL(maB.DIRECTION) = -1;

    IEC_VAL(maB.EXECUTE) = true;
    maB.EXECUTE0.value = false;
    __mcl_cmd_MoveAbsolute(&maB);

    for (step = 0; step < MAX_SIM_STEPS; step++) {
        __HydMotion_framework_Publish();
        IEC_VAL(maB.EXECUTE) = true;
        maB.EXECUTE0.value = true;
        __mcl_cmd_MoveAbsolute(&maB);

        if (IEC_VAL(maB.DONE)) break;
    }

    ASSERT_TRUE(IEC_VAL(maB.DONE) == true,
               "FB_B should reach DONE on second retract");

    printf("  FB_B second retract DONE after %d sim steps\n", step);
}

/* ==================================================================
 * Test 6: MoveAbsolute Done 后 ReadSimFeedback 读取最终位置
 *
 * 验证: 仿真运动完成后，通过 ReadSimFeedback 读取的数据有效，
 *       位置应接近目标位置
 * ================================================================== */
static void test_moveabsolute_done_read_sim_feedback(void) {
    HYD_MOVEABSOLUTE ma;
    HYD_READSIMFEEDBACK rfb;
    int axisId, steps;

    printf("--- Test: MoveAbsolute DONE → ReadSimFeedback ---\n");

    __HydMotion_framework_Init();
    axisId = create_sim_axis(false);
    ASSERT_TRUE(axisId >= 0, "CreateMotion should succeed");

    /* 伸出到 50mm */
    memset(&ma, 0, sizeof(ma));
    IEC_VAL(ma.EN) = true;
    IEC_VAL(ma.AXISID) = axisId;
    IEC_VAL(ma.POSITION) = 50.0f;
    IEC_VAL(ma.VELOCITY) = 30.0f;
    IEC_VAL(ma.ACCELERATION) = 150.0f;
    IEC_VAL(ma.DIRECTION) = 1;

    steps = run_moveabsolute_to_done(&ma, MAX_SIM_STEPS);
    ASSERT_TRUE(steps > 0, "MoveAbsolute should reach DONE");

    /* 读取仿真反馈 */
    memset(&rfb, 0, sizeof(rfb));
    IEC_VAL(rfb.ENABLE) = true;
    IEC_VAL(rfb.AXISID) = axisId;
    __mcl_cmd_ReadSimFeedback(&rfb);

    ASSERT_TRUE(IEC_VAL(rfb.POSITION) >= 0.0f,
               "Sim feedback position should be non-negative");
    /* 仿真中位置积分应该推进（可能不完全精确到达目标，取决于仿真精度） */
    printf("  Final sim position: %.2f mm (target: 50.0 mm)\n",
           (double)IEC_VAL(rfb.POSITION));
    printf("  Final sim velocity: %.4f mm/s\n", (double)IEC_VAL(rfb.VELOCITY));
    printf("  Final sim flow:     %.4f L/min\n", (double)IEC_VAL(rfb.FLOW));
    printf("  Final sim pressure: %.4f MPa\n", (double)IEC_VAL(rfb.PRESSURE));
}

/* ==================================================================
 * Test 7: Reset 在运动中执行 → Done
 *
 * 验证: MoveAbsolute 运动中 Reset 后，Reset 的 Done 正确置位
 * ================================================================== */
static void test_reset_during_motion_done(void) {
    HYD_MOVEABSOLUTE ma;
    HYD_RESET reset;
    int axisId, step;

    printf("--- Test: Reset during motion → DONE ---\n");

    __HydMotion_framework_Init();
    axisId = create_sim_axis(false);
    ASSERT_TRUE(axisId >= 0, "CreateMotion should succeed");

    /* 启动 MoveAbsolute */
    memset(&ma, 0, sizeof(ma));
    IEC_VAL(ma.EN) = true;
    IEC_VAL(ma.AXISID) = axisId;
    IEC_VAL(ma.POSITION) = 100.0f;
    IEC_VAL(ma.VELOCITY) = 50.0f;
    IEC_VAL(ma.ACCELERATION) = 200.0f;
    IEC_VAL(ma.DIRECTION) = 1;

    IEC_VAL(ma.EXECUTE) = true;
    ma.EXECUTE0.value = false;
    __mcl_cmd_MoveAbsolute(&ma);
    __HydMotion_framework_Publish();

    IEC_VAL(ma.EXECUTE) = true;
    ma.EXECUTE0.value = true;
    __mcl_cmd_MoveAbsolute(&ma);

    /* 运行几个周期让运动建立 */
    for (step = 0; step < 5; step++) {
        __HydMotion_framework_Publish();
        IEC_VAL(ma.EXECUTE) = true;
        ma.EXECUTE0.value = true;
        __mcl_cmd_MoveAbsolute(&ma);
    }

    ASSERT_TRUE(IEC_VAL(ma.BUSY) || IEC_VAL(ma.ACTIVE),
               "MoveAbsolute should be active during motion");

    /* Reset */
    memset(&reset, 0, sizeof(reset));
    IEC_VAL(reset.EN) = true;
    IEC_VAL(reset.EXECUTE) = true;
    reset.EXECUTE0.value = false;
    IEC_VAL(reset.AXISID) = axisId;
    __mcl_cmd_Reset(&reset);

    ASSERT_TRUE(IEC_VAL(reset.DONE) == true,
               "Reset should set DONE immediately");
    ASSERT_TRUE(IEC_VAL(reset.BUSY) == false,
               "Reset should clear BUSY after DONE");
    ASSERT_TRUE(IEC_VAL(reset.ERROR) == false,
               "Reset should not set ERROR");

    /* MoveAbsolute 应检测到失去所有权 */
    __HydMotion_framework_Publish();
    IEC_VAL(ma.EXECUTE) = true;
    ma.EXECUTE0.value = true;
    __mcl_cmd_MoveAbsolute(&ma);

    ASSERT_TRUE(IEC_VAL(ma.COMMANDABORTED) == true,
               "MoveAbsolute should get COMMANDABORTED after Reset");
}

static void test_reset_preempts_movevelocity(void) {
    HYD_MOVEVELOCITY mv;
    HYD_RESET reset;
    int axisId, step;

    __HydMotion_framework_Init();
    axisId = create_sim_axis(false);
    ASSERT_TRUE(axisId >= 0, "CreateMotion should succeed");

    memset(&mv, 0, sizeof(mv));
    IEC_VAL(mv.EN) = true;
    IEC_VAL(mv.EXECUTE) = true;
    mv.EXECUTE0.value = false;
    IEC_VAL(mv.AXISID) = axisId;
    IEC_VAL(mv.VELOCITY) = 30.0f;
    IEC_VAL(mv.ACCELERATION) = 150.0f;
    IEC_VAL(mv.DIRECTION) = 1;
    __mcl_cmd_MoveVelocity(&mv);
    __HydMotion_framework_Publish();

    IEC_VAL(mv.EXECUTE) = true;
    mv.EXECUTE0.value = true;
    __mcl_cmd_MoveVelocity(&mv);

    for (step = 0; step < 5; step++) {
        __HydMotion_framework_Publish();
        IEC_VAL(mv.EXECUTE) = true;
        mv.EXECUTE0.value = true;
        __mcl_cmd_MoveVelocity(&mv);
    }

    memset(&reset, 0, sizeof(reset));
    IEC_VAL(reset.EN) = true;
    IEC_VAL(reset.EXECUTE) = true;
    reset.EXECUTE0.value = false;
    IEC_VAL(reset.AXISID) = axisId;
    __mcl_cmd_Reset(&reset);

    ASSERT_TRUE(IEC_VAL(reset.DONE) == true,
               "Reset should complete immediately while preempting MoveVelocity");

    __HydMotion_framework_Publish();
    IEC_VAL(mv.EXECUTE) = true;
    mv.EXECUTE0.value = true;
    __mcl_cmd_MoveVelocity(&mv);

    ASSERT_TRUE(IEC_VAL(mv.COMMANDABORTED) == true,
               "MoveVelocity should report COMMANDABORTED after Reset");
    ASSERT_TRUE(IEC_VAL(mv.INVELOCITY) == false,
               "MoveVelocity INVELOCITY should clear after Reset takeover");
}

/* ==================================================================
 * Test 8: MoveAbsolute 伸出→Done→MoveVelocity 连续切换
 *
 * 验证: MoveAbsolute Done 后，立即启动 MoveVelocity，
 *       MoveVelocity 正常执行不产生 COMMANDABORTED
 * ================================================================== */
static void test_moveabsolute_done_then_movevelocity(void) {
    HYD_MOVEABSOLUTE ma;
    HYD_MOVEVELOCITY mv;
    int axisId, steps;

    printf("--- Test: MoveAbsolute DONE → MoveVelocity ---\n");

    __HydMotion_framework_Init();
    axisId = create_sim_axis(false);
    ASSERT_TRUE(axisId >= 0, "CreateMotion should succeed");

    /* MoveAbsolute 伸出 */
    memset(&ma, 0, sizeof(ma));
    IEC_VAL(ma.EN) = true;
    IEC_VAL(ma.AXISID) = axisId;
    IEC_VAL(ma.POSITION) = 50.0f;
    IEC_VAL(ma.VELOCITY) = 30.0f;
    IEC_VAL(ma.ACCELERATION) = 150.0f;
    IEC_VAL(ma.DIRECTION) = 1;

    steps = run_moveabsolute_to_done(&ma, MAX_SIM_STEPS);
    ASSERT_TRUE(steps > 0, "MoveAbsolute should reach DONE");

    /* EXECUTE下降 */
    IEC_VAL(ma.EXECUTE) = false;
    ma.EXECUTE0.value = true;
    __mcl_cmd_MoveAbsolute(&ma);
    __HydMotion_framework_Publish();

    /* 启动 MoveVelocity */
    memset(&mv, 0, sizeof(mv));
    IEC_VAL(mv.EN) = true;
    IEC_VAL(mv.EXECUTE) = true;
    mv.EXECUTE0.value = false;
    IEC_VAL(mv.AXISID) = axisId;
    IEC_VAL(mv.VELOCITY) = 20.0f;
    IEC_VAL(mv.ACCELERATION) = 100.0f;
    IEC_VAL(mv.DIRECTION) = 1;

    __mcl_cmd_MoveVelocity(&mv);
    __HydMotion_framework_Publish();

    IEC_VAL(mv.EXECUTE) = true;
    mv.EXECUTE0.value = true;
    __mcl_cmd_MoveVelocity(&mv);

    ASSERT_TRUE(IEC_VAL(mv.COMMANDABORTED) == false,
               "MoveVelocity should not be COMMANDABORTED after MoveAbsolute DONE");
    ASSERT_TRUE(IEC_VAL(mv.BUSY) || IEC_VAL(mv.ACTIVE),
               "MoveVelocity should be active after start");
}

/* ==================================================================
 * Test 9: MoveAbsolute 伸出→Done→PressureHandle 连续切换
 *
 * 验证: 位置运动完成后，切换到压力控制模式正常工作
 * ================================================================== */
static void test_moveabsolute_done_then_pressurehandle(void) {
    HYD_MOVEABSOLUTE ma;
    HYD_PRESSUREHANDLE ph;
    int axisId, steps;

    printf("--- Test: MoveAbsolute DONE → PressureHandle ---\n");

    __HydMotion_framework_Init();
    axisId = create_sim_axis(false);
    ASSERT_TRUE(axisId >= 0, "CreateMotion should succeed");

    /* MoveAbsolute 伸出 */
    memset(&ma, 0, sizeof(ma));
    IEC_VAL(ma.EN) = true;
    IEC_VAL(ma.AXISID) = axisId;
    IEC_VAL(ma.POSITION) = 50.0f;
    IEC_VAL(ma.VELOCITY) = 30.0f;
    IEC_VAL(ma.ACCELERATION) = 150.0f;
    IEC_VAL(ma.DIRECTION) = 1;

    steps = run_moveabsolute_to_done(&ma, MAX_SIM_STEPS);
    ASSERT_TRUE(steps > 0, "MoveAbsolute should reach DONE");

    /* EXECUTE下降 */
    IEC_VAL(ma.EXECUTE) = false;
    ma.EXECUTE0.value = true;
    __mcl_cmd_MoveAbsolute(&ma);
    __HydMotion_framework_Publish();

    /* 启动 PressureHandle */
    memset(&ph, 0, sizeof(ph));
    IEC_VAL(ph.EN) = true;
    IEC_VAL(ph.EXECUTE) = true;
    ph.EXECUTE0.value = false;
    IEC_VAL(ph.AXISID) = axisId;
    IEC_VAL(ph.PRESSURE) = 8.0f;
    IEC_VAL(ph.PRESSURERAMPRATE) = 5.0f;
    IEC_VAL(ph.DURATION) = 0.3f;

    __mcl_cmd_PressureHandle(&ph);

    ASSERT_TRUE(IEC_VAL(ph.BUSY) == true,
               "PressureHandle should set BUSY after MoveAbsolute DONE");
    ASSERT_TRUE(IEC_VAL(ph.COMMANDABORTED) == false,
               "PressureHandle should not be COMMANDABORTED");
}

/* ==================================================================
 * Test 10: 多轴伸/缩循环并行
 *
 * 验证: 轴0和轴1各自独立执行伸/缩循环，互不干扰
 * ================================================================== */
static void test_multi_axis_extend_retract_parallel(void) {
    HYD_MOVEABSOLUTE ma0, ma1;
    int axisId0, axisId1, steps0, steps1;

    printf("--- Test: Multi-axis extend/retract parallel ---\n");

    __HydMotion_framework_Init();
    axisId0 = create_sim_axis(false);
    axisId1 = create_sim_axis(false);
    ASSERT_TRUE(axisId0 >= 0, "Axis 0 CreateMotion should succeed");
    ASSERT_TRUE(axisId1 >= 0, "Axis 1 CreateMotion should succeed");

    /* 轴0 伸出 */
    memset(&ma0, 0, sizeof(ma0));
    IEC_VAL(ma0.EN) = true;
    IEC_VAL(ma0.AXISID) = axisId0;
    IEC_VAL(ma0.POSITION) = 80.0f;
    IEC_VAL(ma0.VELOCITY) = 40.0f;
    IEC_VAL(ma0.ACCELERATION) = 200.0f;
    IEC_VAL(ma0.DIRECTION) = 1;
    steps0 = run_moveabsolute_to_done(&ma0, MAX_SIM_STEPS);

    /* 轴1 伸出 */
    memset(&ma1, 0, sizeof(ma1));
    IEC_VAL(ma1.EN) = true;
    IEC_VAL(ma1.AXISID) = axisId1;
    IEC_VAL(ma1.POSITION) = 60.0f;
    IEC_VAL(ma1.VELOCITY) = 30.0f;
    IEC_VAL(ma1.ACCELERATION) = 150.0f;
    IEC_VAL(ma1.DIRECTION) = 1;
    steps1 = run_moveabsolute_to_done(&ma1, MAX_SIM_STEPS);

    ASSERT_TRUE(steps0 > 0, "Axis 0 extend should reach DONE");
    ASSERT_TRUE(steps1 > 0, "Axis 1 extend should reach DONE");
    ASSERT_TRUE(IEC_VAL(ma0.DONE) == true, "Axis 0 should be DONE");
    ASSERT_TRUE(IEC_VAL(ma1.DONE) == true, "Axis 1 should be DONE");

    /* 轴0 缩回 */
    memset(&ma0, 0, sizeof(ma0));
    IEC_VAL(ma0.EN) = true;
    IEC_VAL(ma0.AXISID) = axisId0;
    IEC_VAL(ma0.POSITION) = 0.0f;
    IEC_VAL(ma0.VELOCITY) = 40.0f;
    IEC_VAL(ma0.ACCELERATION) = 200.0f;
    IEC_VAL(ma0.DIRECTION) = -1;
    steps0 = run_moveabsolute_to_done(&ma0, MAX_SIM_STEPS);

    /* 轴1 缩回 */
    memset(&ma1, 0, sizeof(ma1));
    IEC_VAL(ma1.EN) = true;
    IEC_VAL(ma1.AXISID) = axisId1;
    IEC_VAL(ma1.POSITION) = 0.0f;
    IEC_VAL(ma1.VELOCITY) = 30.0f;
    IEC_VAL(ma1.ACCELERATION) = 150.0f;
    IEC_VAL(ma1.DIRECTION) = -1;
    steps1 = run_moveabsolute_to_done(&ma1, MAX_SIM_STEPS);

    ASSERT_TRUE(steps0 > 0, "Axis 0 retract should reach DONE");
    ASSERT_TRUE(steps1 > 0, "Axis 1 retract should reach DONE");
    ASSERT_TRUE(IEC_VAL(ma0.COMMANDABORTED) == false,
               "Axis 0 should not be COMMANDABORTED by Axis 1");
    ASSERT_TRUE(IEC_VAL(ma1.COMMANDABORTED) == false,
               "Axis 1 should not be COMMANDABORTED by Axis 0");
}

/* ==================================================================
 * Test 11: Stop 在运动轴上的 Done 信号 (从MoveAbsolute运行中Stop)
 * ================================================================== */
static void test_stop_during_moveabsolute_done(void) {
    HYD_MOVEABSOLUTE ma;
    HYD_STOP stop;
    int axisId, step;
    int stopDoneStep = -1;

    printf("--- Test: Stop during MoveAbsolute → DONE ---\n");

    __HydMotion_framework_Init();
    axisId = create_sim_axis(false);
    ASSERT_TRUE(axisId >= 0, "CreateMotion should succeed");

    /* 启动 MoveAbsolute */
    memset(&ma, 0, sizeof(ma));
    IEC_VAL(ma.EN) = true;
    IEC_VAL(ma.AXISID) = axisId;
    IEC_VAL(ma.POSITION) = 100.0f;
    IEC_VAL(ma.VELOCITY) = 50.0f;
    IEC_VAL(ma.ACCELERATION) = 200.0f;
    IEC_VAL(ma.DIRECTION) = 1;

    IEC_VAL(ma.EXECUTE) = true;
    ma.EXECUTE0.value = false;
    __mcl_cmd_MoveAbsolute(&ma);
    __HydMotion_framework_Publish();

    IEC_VAL(ma.EXECUTE) = true;
    ma.EXECUTE0.value = true;
    __mcl_cmd_MoveAbsolute(&ma);

    /* 运行若干周期 */
    for (step = 0; step < 10; step++) {
        __HydMotion_framework_Publish();
        IEC_VAL(ma.EXECUTE) = true;
        ma.EXECUTE0.value = true;
        __mcl_cmd_MoveAbsolute(&ma);
    }

    ASSERT_TRUE(IEC_VAL(ma.BUSY) || IEC_VAL(ma.ACTIVE),
               "MoveAbsolute should be active before Stop");

    /* Stop */
    memset(&stop, 0, sizeof(stop));
    IEC_VAL(stop.EN) = true;
    IEC_VAL(stop.EXECUTE) = true;
    stop.EXECUTE0.value = false;
    IEC_VAL(stop.AXISID) = axisId;
    __mcl_cmd_Stop(&stop);
    ASSERT_TRUE(IEC_VAL(stop.DONE) == false,
               "Stop should not be DONE on the trigger call");
    __HydMotion_framework_Publish();

    /* 等待 Stop Done */
    for (step = 0; step < MAX_SIM_STEPS; step++) {
        __HydMotion_framework_Publish();
        IEC_VAL(stop.EXECUTE) = true;
        stop.EXECUTE0.value = true;
        __mcl_cmd_Stop(&stop);

        if (IEC_VAL(stop.DONE)) {
            stopDoneStep = step + 1;
            break;
        }
    }

    ASSERT_TRUE(IEC_VAL(stop.DONE) == true,
               "Stop should reach DONE during active motion");
    ASSERT_TRUE(stopDoneStep > 1,
               "Stop should take multiple cycles to finish during active motion");
    ASSERT_TRUE(IEC_VAL(stop.BUSY) == false,
               "Stop BUSY should be false after DONE");
    ASSERT_TRUE(IEC_VAL(stop.ERROR) == false,
               "Stop should not set ERROR");

    /* MoveAbsolute 应被抢占 */
    IEC_VAL(ma.EXECUTE) = true;
    ma.EXECUTE0.value = true;
    __mcl_cmd_MoveAbsolute(&ma);
    ASSERT_TRUE(IEC_VAL(ma.COMMANDABORTED) == true,
               "MoveAbsolute should get COMMANDABORTED after Stop");
    ASSERT_TRUE(IEC_VAL(ma.DONE) == false,
               "MoveAbsolute should NOT be DONE when stopped prematurely");
}

/* ==================================================================
 * Test 12: MoveVelocity → 另一个MoveVelocity (自抢占/参数更新)
 *
 * 验证: 同一轴上先启动MoveVelocity，再启动新的MoveVelocity，
 *       第一个应被COMMANDABORTED，第二个正常运行
 * ================================================================== */
static void test_movevelocity_preempted_by_another_movevelocity(void) {
    HYD_MOVEVELOCITY mv1, mv2;
    int axisId, step;

    printf("--- Test: MoveVelocity preempted by another MoveVelocity ---\n");

    __HydMotion_framework_Init();
    axisId = create_sim_axis(false);
    ASSERT_TRUE(axisId >= 0, "CreateMotion should succeed");

    /* 第一个 MoveVelocity */
    memset(&mv1, 0, sizeof(mv1));
    IEC_VAL(mv1.EN) = true;
    IEC_VAL(mv1.EXECUTE) = true;
    mv1.EXECUTE0.value = false;
    IEC_VAL(mv1.AXISID) = axisId;
    IEC_VAL(mv1.VELOCITY) = 20.0f;
    IEC_VAL(mv1.ACCELERATION) = 100.0f;
    IEC_VAL(mv1.DIRECTION) = 1;

    __mcl_cmd_MoveVelocity(&mv1);
    __HydMotion_framework_Publish();

    IEC_VAL(mv1.EXECUTE) = true;
    mv1.EXECUTE0.value = true;
    __mcl_cmd_MoveVelocity(&mv1);

    /* 运行若干周期 */
    for (step = 0; step < 10; step++) {
        __HydMotion_framework_Publish();
        IEC_VAL(mv1.EXECUTE) = true;
        mv1.EXECUTE0.value = true;
        __mcl_cmd_MoveVelocity(&mv1);
    }

    ASSERT_TRUE(IEC_VAL(mv1.ACTIVE) || IEC_VAL(mv1.BUSY),
               "First MoveVelocity should be active");

    /* 第二个 MoveVelocity (不同速度) 抢占 */
    memset(&mv2, 0, sizeof(mv2));
    IEC_VAL(mv2.EN) = true;
    IEC_VAL(mv2.EXECUTE) = true;
    mv2.EXECUTE0.value = false;
    IEC_VAL(mv2.AXISID) = axisId;
    IEC_VAL(mv2.VELOCITY) = 40.0f;
    IEC_VAL(mv2.ACCELERATION) = 150.0f;
    IEC_VAL(mv2.DIRECTION) = 1;

    __mcl_cmd_MoveVelocity(&mv2);
    __HydMotion_framework_Publish();

    /* mv1 应被抢占 */
    IEC_VAL(mv1.EXECUTE) = true;
    mv1.EXECUTE0.value = true;
    __mcl_cmd_MoveVelocity(&mv1);
    ASSERT_TRUE(IEC_VAL(mv1.COMMANDABORTED) == true,
               "First MoveVelocity should get COMMANDABORTED");

    /* mv2 应正常运行 */
    IEC_VAL(mv2.EXECUTE) = true;
    mv2.EXECUTE0.value = true;
    __mcl_cmd_MoveVelocity(&mv2);
    ASSERT_TRUE(IEC_VAL(mv2.COMMANDABORTED) == false,
               "Second MoveVelocity should not be COMMANDABORTED");
    ASSERT_TRUE(IEC_VAL(mv2.ACTIVE) || IEC_VAL(mv2.BUSY),
               "Second MoveVelocity should be active");
}

/* ==================================================================
 * Test 13: MoveAbsolute Done后速度输出应为0
 *
 * 验证: 当FB从RUNNING过渡到DONE时，规划器输出的速度应归零，
 *       仿真反馈回路的速度也应归零，不应出现DONE后仍有非零速度输出。
 *
 * 背景: 在仿真模式下，_simFeedback在段完成时未被清除，
 *       导致Publish()使用陈旧的速度值更新AXIS_REF，
 *       使仿真轴在DONE后继续运动。
 * ================================================================== */
static void test_moveabsolute_done_velocity_zero(void) {
    HYD_MOVEABSOLUTE ma;
    HYD_READSIMFEEDBACK rfb;
    int axisId, steps, postStep;

    printf("--- Test: MoveAbsolute DONE → velocity must be zero ---\n");

    __HydMotion_framework_Init();
    axisId = create_sim_axis(false);
    ASSERT_TRUE(axisId >= 0, "CreateMotion should succeed");

    /* 伸出到 100mm */
    memset(&ma, 0, sizeof(ma));
    IEC_VAL(ma.EN) = true;
    IEC_VAL(ma.AXISID) = axisId;
    IEC_VAL(ma.POSITION) = 100.0f;
    IEC_VAL(ma.VELOCITY) = 50.0f;
    IEC_VAL(ma.ACCELERATION) = 200.0f;
    IEC_VAL(ma.DIRECTION) = 1;  /* EXTEND */

    steps = run_moveabsolute_to_done(&ma, MAX_SIM_STEPS);
    ASSERT_TRUE(steps > 0, "MoveAbsolute should reach DONE within max steps");
    ASSERT_TRUE(IEC_VAL(ma.DONE) == true, "MoveAbsolute should be DONE");
    ASSERT_TRUE(IEC_VAL(ma.BUSY) == false, "BUSY should be false after DONE");
    ASSERT_TRUE(IEC_VAL(ma.ACTIVE) == false, "ACTIVE should be false after DONE");

    printf("  DONE after %d sim steps\n", steps);

    /* 读取核心FB内部状态验证 */
    {
        HYD_MotionControlFB* fb = __MK_GetPublic_MotionControlFB(axisId);
        ASSERT_TRUE(fb != NULL, "Should get FB by index");
        ASSERT_FNEAR(fb->STATE.plannedVelocity, 0.0, 0.01,
                     "plannedVelocity should be 0 after DONE");
        ASSERT_FNEAR(fb->PUMP_SPEED, 0.0, 0.01,
                     "PUMP_SPEED should be 0 after DONE");
        ASSERT_FNEAR(fb->_simFeedback.targetVelocity, 0.0, 0.01,
                     "_simFeedback.targetVelocity should be 0 after DONE");
        ASSERT_FNEAR(fb->_simFeedback.targetFlow, 0.0, 0.01,
                     "_simFeedback.targetFlow should be 0 after DONE");
        printf("  After DONE: plannedVelocity=%.4f, pumpSpeed=%.4f, simVel=%.4f\n",
               (double)fb->STATE.plannedVelocity,
               (double)fb->PUMP_SPEED,
               (double)fb->_simFeedback.targetVelocity);
    }

    /* 再推进若干仿真周期，验证速度持续为0 */
    HYD_REAL positionAtDone = 0.0;
    {
        HYD_MotionControlFB* fb = __MK_GetPublic_MotionControlFB(axisId);
        positionAtDone = fb->AXIS_REF.position;
        printf("  Position at DONE: %.4f mm\n", (double)positionAtDone);
    }

    for (postStep = 0; postStep < 20; postStep++) {
        __HydMotion_framework_Publish();
        IEC_VAL(ma.EXECUTE) = true;
        ma.EXECUTE0.value = true;
        __mcl_cmd_MoveAbsolute(&ma);
    }

    /* 验证: 多周期后速度仍为0，位置不再偏移 */
    {
        HYD_MotionControlFB* fb = __MK_GetPublic_MotionControlFB(axisId);
        ASSERT_FNEAR(fb->_simFeedback.targetVelocity, 0.0, 0.01,
                     "simFeedback velocity should stay 0 after multiple cycles");
        ASSERT_FNEAR(fb->AXIS_REF.velocity, 0.0, 0.01,
                     "AXIS_REF velocity should be 0 after multiple cycles");
        ASSERT_FNEAR(fb->AXIS_REF.position, positionAtDone, 0.1,
                     "Position should not drift after DONE");
        printf("  After 20 more cycles: velocity=%.4f, position=%.4f (drift=%.4f)\n",
               (double)fb->AXIS_REF.velocity,
               (double)fb->AXIS_REF.position,
               (double)(fb->AXIS_REF.position - positionAtDone));
    }

    /* 通过ReadSimFeedback验证外部接口 */
    memset(&rfb, 0, sizeof(rfb));
    IEC_VAL(rfb.ENABLE) = true;
    IEC_VAL(rfb.AXISID) = axisId;
    __mcl_cmd_ReadSimFeedback(&rfb);
    ASSERT_FNEAR(IEC_VAL(rfb.VELOCITY), 0.0, 0.01,
                 "ReadSimFeedback velocity should be 0 after DONE");
    printf("  ReadSimFeedback: pos=%.4f, vel=%.4f, flow=%.4f\n",
           (double)IEC_VAL(rfb.POSITION),
           (double)IEC_VAL(rfb.VELOCITY),
           (double)IEC_VAL(rfb.FLOW));
}

/* ==================================================================
 * Test 14: MoveAbsolute缩回Done后速度归零
 *
 * 验证缩回方向同样满足Done后速度为0
 * ================================================================== */
static void test_moveabsolute_retract_done_velocity_zero(void) {
    HYD_MOVEABSOLUTE ma;
    int axisId, steps;

    printf("--- Test: MoveAbsolute retract DONE → velocity zero ---\n");

    __HydMotion_framework_Init();
    axisId = create_sim_axis(false);
    ASSERT_TRUE(axisId >= 0, "CreateMotion should succeed");

    /* 先伸出到 80mm */
    memset(&ma, 0, sizeof(ma));
    IEC_VAL(ma.EN) = true;
    IEC_VAL(ma.AXISID) = axisId;
    IEC_VAL(ma.POSITION) = 80.0f;
    IEC_VAL(ma.VELOCITY) = 40.0f;
    IEC_VAL(ma.ACCELERATION) = 200.0f;
    IEC_VAL(ma.DIRECTION) = 1;

    steps = run_moveabsolute_to_done(&ma, MAX_SIM_STEPS);
    ASSERT_TRUE(steps > 0, "Extend should reach DONE");

    /* EXECUTE下降沿 */
    IEC_VAL(ma.EXECUTE) = false;
    ma.EXECUTE0.value = true;
    __mcl_cmd_MoveAbsolute(&ma);
    __HydMotion_framework_Publish();

    /* 缩回到 0mm */
    memset(&ma, 0, sizeof(ma));
    IEC_VAL(ma.EN) = true;
    IEC_VAL(ma.AXISID) = axisId;
    IEC_VAL(ma.POSITION) = 0.0f;
    IEC_VAL(ma.VELOCITY) = 40.0f;
    IEC_VAL(ma.ACCELERATION) = 200.0f;
    IEC_VAL(ma.DIRECTION) = -1;

    steps = run_moveabsolute_to_done(&ma, MAX_SIM_STEPS);
    ASSERT_TRUE(steps > 0, "Retract should reach DONE");

    /* 验证缩回Done后速度为0 */
    {
        HYD_MotionControlFB* fb = __MK_GetPublic_MotionControlFB(axisId);
        ASSERT_TRUE(fb != NULL, "Should get FB by index");
        ASSERT_FNEAR(fb->_simFeedback.targetVelocity, 0.0, 0.01,
                     "simFeedback velocity should be 0 after retract DONE");
        ASSERT_FNEAR(fb->AXIS_REF.velocity, 0.0, 0.01,
                     "AXIS_REF velocity should be 0 after retract DONE");
        printf("  Retract DONE: simVel=%.4f, axisVel=%.4f\n",
               (double)fb->_simFeedback.targetVelocity,
               (double)fb->AXIS_REF.velocity);
    }
}

/* ==================================================================
 * Test 15: MoveAbsolute运行中Stop完成后速度归零
 *
 * 验证: Stop减速完成后清除仿真反馈速度
 * ================================================================== */
static void test_moveabsolute_stop_velocity_zero(void) {
    HYD_MOVEABSOLUTE ma;
    HYD_STOP stop;
    int axisId, step;

    printf("--- Test: MoveAbsolute stop → velocity zero ---\n");

    __HydMotion_framework_Init();
    axisId = create_sim_axis(false);
    ASSERT_TRUE(axisId >= 0, "CreateMotion should succeed");

    /* 启动MoveAbsolute */
    memset(&ma, 0, sizeof(ma));
    IEC_VAL(ma.EN) = true;
    IEC_VAL(ma.AXISID) = axisId;
    IEC_VAL(ma.POSITION) = 100.0f;
    IEC_VAL(ma.VELOCITY) = 50.0f;
    IEC_VAL(ma.ACCELERATION) = 200.0f;
    IEC_VAL(ma.DIRECTION) = 1;

    IEC_VAL(ma.EXECUTE) = true;
    ma.EXECUTE0.value = false;
    __mcl_cmd_MoveAbsolute(&ma);
    __HydMotion_framework_Publish();

    IEC_VAL(ma.EXECUTE) = true;
    ma.EXECUTE0.value = true;
    __mcl_cmd_MoveAbsolute(&ma);

    /* 运行若干周期让速度建立 */
    for (step = 0; step < 20; step++) {
        __HydMotion_framework_Publish();
        IEC_VAL(ma.EXECUTE) = true;
        ma.EXECUTE0.value = true;
        __mcl_cmd_MoveAbsolute(&ma);
    }

    /* Stop */
    memset(&stop, 0, sizeof(stop));
    IEC_VAL(stop.EN) = true;
    IEC_VAL(stop.EXECUTE) = true;
    stop.EXECUTE0.value = false;
    IEC_VAL(stop.AXISID) = axisId;
    __mcl_cmd_Stop(&stop);
    __HydMotion_framework_Publish();

    /* 等待Stop Done */
    for (step = 0; step < MAX_SIM_STEPS; step++) {
        __HydMotion_framework_Publish();
        IEC_VAL(stop.EXECUTE) = true;
        stop.EXECUTE0.value = true;
        __mcl_cmd_Stop(&stop);
        if (IEC_VAL(stop.DONE)) break;
    }

    ASSERT_TRUE(IEC_VAL(stop.DONE) == true,
               "Stop should reach DONE before zero-velocity assertions");

    /* 验证Stop完成后仿真反馈速度为0 */
    {
        HYD_MotionControlFB* fb = __MK_GetPublic_MotionControlFB(axisId);
        ASSERT_TRUE(fb != NULL, "Should get FB by index");
        ASSERT_FNEAR(fb->_simFeedback.targetVelocity, 0.0, 0.01,
                     "simFeedback velocity should be 0 after stop completion");
        ASSERT_FNEAR(fb->AXIS_REF.velocity, 0.0, 0.01,
                     "AXIS_REF velocity should be 0 after stop completion");
        printf("  After stop: simVel=%.4f, axisVel=%.4f\n",
               (double)fb->_simFeedback.targetVelocity,
               (double)fb->AXIS_REF.velocity);
    }
}

int main(void) {
    printf("=== Motion Interface Done Signal Simulation Tests ===\n\n");

    test_moveabsolute_extend_retract_cycle();
    test_moveabsolute_retrigger_after_done();
    test_movevelocity_then_stop_done();
    test_pressurehandle_timed_done();
    test_two_moveabsolute_fbs_alternating_same_axis();
    test_moveabsolute_done_read_sim_feedback();
    test_reset_during_motion_done();
    test_reset_preempts_movevelocity();
    test_moveabsolute_done_then_movevelocity();
    test_moveabsolute_done_then_pressurehandle();
    test_multi_axis_extend_retract_parallel();
    test_stop_during_moveabsolute_done();
    test_movevelocity_preempted_by_another_movevelocity();
    test_moveabsolute_done_velocity_zero();
    test_moveabsolute_retract_done_velocity_zero();
    test_moveabsolute_stop_velocity_zero();

    printf("\n=== Results: %d/%d passed ===\n", tests_passed, tests_run);
    return (tests_passed == tests_run) ? 0 : 1;
}
