/**
 * @file test_motion_interface_unit.c
 * @brief IEC FB接口层单元测试
 *
 * 目标：验证6个PLCopen FB（MoveProfile/Stop/MoveAbsolute/MoveVelocity/
 *       Reset/PressureHandle）的生命周期、信号输出、参数校验和
 *       EN/EXECUTE行为。
 *
 * 注意: MoveProfile使用FB池分配器(allocMotionControlFB),
 *        Direct命令(Stop/MoveAbsolute/MoveVelocity/PressureHandle)
 *        通过ensureFbInitialized直接访问FB实例数组。
 */

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdbool.h>

#include "motion_interface.h"
#include "motion_control.h"
#include "recipe_validator.h"

extern HYD_MotionControlFB* __MK_GetPublic_MotionControlFB(int index);

#define IEC_VAL(var) ((var).value)

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

/* 辅助: 通过CreateMotion分配指定数量的轴 (Direct模式) */
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

/* 辅助: 通过CreateMotion分配指定数量的轴 (Recipe模式) */
static void ensure_recipe_axes_allocated(int count) {
    for (int i = 0; i < count; i++) {
        HYD_CREATEMOTION cm;
        memset(&cm, 0, sizeof(cm));
        IEC_VAL(cm.EN) = true;
        IEC_VAL(cm.USE_RECIPE) = true;
        IEC_VAL(cm.FLOW_TO_PUMPSPEED) = 1.2f;
        IEC_VAL(cm.PUMPSPEED_LIMIT) = 3000.0f;
        IEC_VAL(cm.USE_SIMULATION) = false;
        __mcl_cmd_CreateMotion(&cm);
    }
}

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
 * Test 1: Framework Init 归零FB池与分配器
 * ================================================================== */
static void test_framework_init_resets_pool(void) {
    HYD_MotionControlFB* fb;

    __HydMotion_framework_Init();

    /* 初始化后没有FB实例可以访问 */
    fb = __MK_GetPublic_MotionControlFB(0);
    ASSERT_TRUE(fb == NULL, "No FB instance should exist immediately after framework init");

    /* 重新初始化应该仍然是干净的 */
    __HydMotion_framework_Init();
    fb = __MK_GetPublic_MotionControlFB(0);
    ASSERT_TRUE(fb == NULL, "FB pool should still be clean after re-init");
}

static void test_publish_advances_simulation_feedback_time(void) {
    HYD_MOVEABSOLUTE ma;
    HYD_MotionControlFB* fb;
    HYD_REAL firstTimestamp;
    HYD_REAL firstPosition;
    int axisId;

    __HydMotion_framework_Init();
    axisId = create_sim_axis();
    fb = __MK_GetPublic_MotionControlFB(axisId);

    ASSERT_TRUE(axisId >= 0, "CreateMotion should allocate a simulation axis");
    ASSERT_TRUE(fb != NULL, "Simulation axis should expose a public FB instance");

    memset(&ma, 0, sizeof(ma));
    IEC_VAL(ma.EN) = true;
    IEC_VAL(ma.EXECUTE) = true;
    ma.EXECUTE0.value = false;
    IEC_VAL(ma.AXISID) = axisId;
    IEC_VAL(ma.POSITION) = 100.0f;
    IEC_VAL(ma.VELOCITY) = 20.0f;
    IEC_VAL(ma.ACCELERATION) = 200.0f;
    IEC_VAL(ma.DECELERATION) = 200.0f;
    IEC_VAL(ma.DIRECTION) = 1;

    __mcl_cmd_MoveAbsolute(&ma);
    __HydMotion_framework_Publish();

    IEC_VAL(ma.EXECUTE) = true;
    ma.EXECUTE0.value = true;
    __mcl_cmd_MoveAbsolute(&ma);

    firstTimestamp = fb->AXIS_REF.timestamp;
    firstPosition = fb->AXIS_REF.position;

    ASSERT_TRUE(firstTimestamp > 0.0,
                "Publish should advance simulation timestamp on the first cycle");

    __HydMotion_framework_Publish();
    __mcl_cmd_MoveAbsolute(&ma);

    ASSERT_TRUE(fb->AXIS_REF.timestamp > firstTimestamp,
                "Publish should keep advancing simulation timestamp");
    ASSERT_TRUE(fabs(fb->AXIS_REF.velocity) > 0.0,
                "Publish should apply simulated velocity feedback");
    ASSERT_TRUE(fb->AXIS_REF.position != firstPosition,
                "Publish should integrate simulated position when velocity is non-zero");
}

static void test_simulation_velocity_ramp_uses_fixed_step_after_large_timestamp(void) {
    HYD_MOVEVELOCITY mv;
    HYD_MotionControlFB* fb;
    int axisId;

    __HydMotion_framework_Init();
    axisId = create_sim_axis();
    fb = __MK_GetPublic_MotionControlFB(axisId);

    ASSERT_TRUE(axisId >= 0, "CreateMotion should allocate a simulation axis");
    ASSERT_TRUE(fb != NULL, "Simulation axis should expose a public FB instance");

    memset(&mv, 0, sizeof(mv));
    IEC_VAL(mv.EN) = true;
    IEC_VAL(mv.EXECUTE) = true;
    mv.EXECUTE0.value = false;
    IEC_VAL(mv.AXISID) = axisId;
    IEC_VAL(mv.VELOCITY) = 100.0f;
    IEC_VAL(mv.ACCELERATION) = 200.0f;
    IEC_VAL(mv.DECELERATION) = 200.0f;
    IEC_VAL(mv.DIRECTION) = 1;

    __mcl_cmd_MoveVelocity(&mv);
    __HydMotion_framework_Publish();

    IEC_VAL(mv.EXECUTE) = true;
    mv.EXECUTE0.value = true;
    __mcl_cmd_MoveVelocity(&mv);
    ASSERT_TRUE(fb->_activeSegmentValid, "MoveVelocity should have an active segment");

    /* Force the visible timestamp into the float precision-loss region.
     * The simulation control loop should still step by the fixed 1 ms period. */
    fb->AXIS_REF.timestamp = 28800.0f;
    fb->_lastFeedbackTimestamp = fb->AXIS_REF.timestamp;
    fb->_plannerState.initialized = true;
    fb->_plannerState.lastTargetVelocity = 0.0f;
    fb->_plannerState.lastTargetFlow = 0.0f;
    fb->AXIS_REF.velocity = 0.0f;
    fb->AXIS_REF.flow = 0.0f;

    __HydMotion_framework_Publish();
    __mcl_cmd_MoveVelocity(&mv);
    ASSERT_TRUE(fabsf(fb->AXIS_REF.velocity - 0.2f) < 0.05f,
                "Simulation velocity ramp should advance by acceleration * 1 ms even after large absolute timestamps");
}

static void test_real_axis_velocity_ramp_uses_fixed_step_after_large_timestamp(void) {
    HYD_MOVEVELOCITY mv;
    HYD_MotionControlFB* fb;
    int axisId;

    __HydMotion_framework_Init();
    ensure_axes_allocated(1);
    axisId = 0;
    fb = __MK_GetPublic_MotionControlFB(axisId);

    ASSERT_TRUE(fb != NULL, "Real axis should expose a public FB instance");

    fb->AXIS_REF.position = 0.0f;
    fb->AXIS_REF.velocity = 0.0f;
    fb->AXIS_REF.flow = 0.0f;
    fb->AXIS_REF.pressure = 0.0f;
    fb->AXIS_REF.timestamp = 0.0f;

    memset(&mv, 0, sizeof(mv));
    IEC_VAL(mv.EN) = true;
    IEC_VAL(mv.EXECUTE) = true;
    mv.EXECUTE0.value = false;
    IEC_VAL(mv.AXISID) = axisId;
    IEC_VAL(mv.VELOCITY) = 100.0f;
    IEC_VAL(mv.ACCELERATION) = 200.0f;
    IEC_VAL(mv.DECELERATION) = 200.0f;
    IEC_VAL(mv.DIRECTION) = 1;

    __mcl_cmd_MoveVelocity(&mv);
    __HydMotion_framework_Publish();

    IEC_VAL(mv.EXECUTE) = true;
    mv.EXECUTE0.value = true;
    __mcl_cmd_MoveVelocity(&mv);
    ASSERT_TRUE(fb->_activeSegmentValid, "MoveVelocity should have an active segment");

    /* Force the visible timestamp into the float precision-loss region and
     * keep feeding the same large absolute sensor timestamp. The control loop
     * should still advance by the fixed 1 ms period. */
    fb->AXIS_REF.timestamp = 28800.0f;
    fb->_lastFeedbackTimestamp = fb->AXIS_REF.timestamp;
    fb->_plannerState.initialized = true;
    fb->_plannerState.lastTargetVelocity = 0.0f;
    fb->_plannerState.lastTargetFlow = 0.0f;
    fb->AXIS_REF.velocity = 0.0f;
    fb->AXIS_REF.flow = 0.0f;

    __HydMotion_framework_Publish();
    __mcl_cmd_MoveVelocity(&mv);
    ASSERT_TRUE(fabsf(fb->STATE.references.velocityReference - 0.2f) < 0.05f,
                "Real-axis control reference should advance by acceleration * 1 ms even after large absolute timestamps");
}

/* ==================================================================
 * Test 2: MoveProfile INIT 分配FB并设置Recipe模式
 * ================================================================== */
static void test_moveprofile_init_allocates_fb_with_recipe_mode(void) {
    HYD_MOVEPROFILE mp;
    HYD_MotionControlFB* fb;
    HYD_CREATEMOTION cm;

    __HydMotion_framework_Init();

    /* CreateMotion with USE_RECIPE=true — recipe-mode axis */
    memset(&cm, 0, sizeof(cm));
    IEC_VAL(cm.EN) = true;
    IEC_VAL(cm.USE_RECIPE) = true;
    IEC_VAL(cm.FLOW_TO_PUMPSPEED) = 1.2f;
    IEC_VAL(cm.PUMPSPEED_LIMIT) = 3000.0f;
    IEC_VAL(cm.USE_SIMULATION) = false;
    __mcl_cmd_CreateMotion(&cm);

    memset(&mp, 0, sizeof(mp));
    IEC_VAL(mp.AXISID) = IEC_VAL(cm.AXISID);
    __mcl_cmd_MoveProfile(&mp);

    fb = __MK_GetPublic_MotionControlFB(0);
    ASSERT_TRUE(fb != NULL, "FB instance should be retrievable via AXISID after CreateMotion");
    ASSERT_TRUE(fb->USE_RECIPE == true, "CreateMotion should set USE_RECIPE=true for recipe-mode axis");

    /* 二次调用不应重新分配 */
    __mcl_cmd_MoveProfile(&mp);
    ASSERT_EQ(IEC_VAL(mp.AXISID), IEC_VAL(cm.AXISID), "Axis index should remain stable across calls");
}

/* ==================================================================
 * Test 3: MoveProfile 无 EXECUTE 不触发运动
 * ================================================================== */
static void test_moveprofile_no_execute_does_not_start(void) {
    HYD_MOVEPROFILE mp;

    __HydMotion_framework_Init();
    ensure_recipe_axes_allocated(2);
    memset(&mp, 0, sizeof(mp));

    /* INIT */
    __mcl_cmd_MoveProfile(&mp);

    /* 设置 EXECUTE=false (未触发) */
    IEC_VAL(mp.EN) = true;
    IEC_VAL(mp.EXECUTE) = false;
    mp.EXECUTE0.value = false;

    __mcl_cmd_MoveProfile(&mp);

    ASSERT_TRUE(IEC_VAL(mp.ACTIVE) == false, "ACTIVE should be false without EXECUTE trigger");
    ASSERT_TRUE(IEC_VAL(mp.BUSY) == false, "BUSY should be false without EXECUTE trigger");
}

/* ==================================================================
 * Test 4: MoveProfile EXECUTE 上升沿触发运动
 * ================================================================== */
static void test_moveprofile_execute_rising_triggers_motion(void) {
    HYD_MOVEPROFILE mp;
    HYD_AXISMOTION motion;

    __HydMotion_framework_Init();
    ensure_recipe_axes_allocated(2);
    memset(&mp, 0, sizeof(mp));
    memset(&motion, 0, sizeof(motion));

    /* INIT */
    __mcl_cmd_MoveProfile(&mp);

    /* 配置 MOTION 参数: 位置控制模式 */
    motion.MODE = HYD_MODE_POSITION;
    motion.ENDCONDITION = HYD_END_POSITION;
    motion.DIRECTION = HYD_DIRECTION_EXTEND;
    motion.SETPOSITION = 50.0f;
    motion.SETVELOCITY = 20.0f;
    motion.SETFLOW = 10.0f;
    motion.ACCELERATION = 100.0f;
    motion.TIMESTAMP = 0.0f;

    /* 准备 EXECUTE 上升沿 */
    IEC_VAL(mp.EN) = true;
    IEC_VAL(mp.EXECUTE) = true;
    mp.EXECUTE0.value = false;
    __SET_VAR(mp., MOTION, , motion);

    __mcl_cmd_MoveProfile(&mp);

    HYD_MotionControlFB* fb = __MK_GetPublic_MotionControlFB(0);
    ASSERT_TRUE(fb != NULL, "FB should still exist after execute");

    /* 启动成功后 _PENDING 应为 true, _EXEC_ID 应已清零准备写入 */
    ASSERT_TRUE(IEC_VAL(mp._PENDING) == true,
               "_PENDING should be set after EXECUTE starts");
    ASSERT_TRUE(IEC_VAL(mp._EXEC_ID) == 0,
               "_EXEC_ID should be 0 until ownership is confirmed");

    /* 处理后第二周期确认所有权 */
    __HydMotion_framework_Publish();
    IEC_VAL(mp.EXECUTE) = true;
    mp.EXECUTE0.value = true;
    __mcl_cmd_MoveProfile(&mp);
    ASSERT_TRUE(IEC_VAL(mp._PENDING) == false,
               "_PENDING should be cleared after ownership is resolved");
}

/* ==================================================================
 * Test 5: MoveAbsolute EXECUTE 上升沿 正常生命周期
 * ================================================================== */
static void test_moveabsolute_execute_rising_sets_busy_active(void) {
    HYD_MOVEABSOLUTE ma;

    __HydMotion_framework_Init();
    ensure_axes_allocated(2);
    memset(&ma, 0, sizeof(ma));

    /* 准备命令 */
    IEC_VAL(ma.EN) = true;
    IEC_VAL(ma.EXECUTE) = true;
    ma.EXECUTE0.value = false;  /* 上升沿 */
    IEC_VAL(ma.AXISID) = 0;
    IEC_VAL(ma.POSITION) = 100.0f;
    IEC_VAL(ma.VELOCITY) = 50.0f;
    IEC_VAL(ma.ACCELERATION) = 200.0f;
    IEC_VAL(ma.DIRECTION) = 1;  /* EXTEND */

    __mcl_cmd_MoveAbsolute(&ma);

    ASSERT_TRUE(IEC_VAL(ma.BUSY) == true, "MoveAbsolute should set BUSY on execRising");
    ASSERT_TRUE(IEC_VAL(ma.ACTIVE) == true, "MoveAbsolute should set ACTIVE on execRising");
    ASSERT_TRUE(IEC_VAL(ma.DONE) == false, "MoveAbsolute DONE should be false initially");
    ASSERT_TRUE(IEC_VAL(ma.COMMANDABORTED) == false, "COMMANDABORTED should be false initially");
    ASSERT_TRUE(IEC_VAL(ma.ERROR) == false, "ERROR should be false on normal start");
    /* ENO由PLC IEC层处理，不在C库层测试 */
}

/* ==================================================================
 * Test 6: MoveAbsolute 持续调用保持 BUSY/ACTIVE
 * ================================================================== */
static void test_moveabsolute_sustains_busy_active_across_calls(void) {
    HYD_MOVEABSOLUTE ma;

    __HydMotion_framework_Init();
    ensure_axes_allocated(2);
    memset(&ma, 0, sizeof(ma));

    /* 上升沿触发 */
    IEC_VAL(ma.EN) = true;
    IEC_VAL(ma.EXECUTE) = true;
    ma.EXECUTE0.value = false;
    IEC_VAL(ma.AXISID) = 0;
    IEC_VAL(ma.POSITION) = 100.0f;
    IEC_VAL(ma.VELOCITY) = 50.0f;
    IEC_VAL(ma.ACCELERATION) = 200.0f;
    IEC_VAL(ma.DIRECTION) = 1;

    __mcl_cmd_MoveAbsolute(&ma);
    ASSERT_TRUE(IEC_VAL(ma.BUSY) == true, "BUSY should be true after execRising");

    /* 下一周期: EXECUTE仍为true但不再是上升沿 */
    IEC_VAL(ma.EXECUTE) = true;
    ma.EXECUTE0.value = true;
    __HydMotion_framework_Publish();  /* 处理待处理命令 */

    __mcl_cmd_MoveAbsolute(&ma);

    /* 所有权应已确认, 仍是owner */
    ASSERT_TRUE(IEC_VAL(ma.BUSY) == true || IEC_VAL(ma.ACTIVE) == true,
               "BUSY or ACTIVE should be sustained for current owner");

    /* 不应触发 COMMANDABORTED */
    ASSERT_TRUE(IEC_VAL(ma.COMMANDABORTED) == false,
               "COMMANDABORTED should not fire for current owner");
}

/* ==================================================================
 * Test 7: MoveAbsolute 持有执行权时 fault 应映射到 ERROR/ERRORID
 * ================================================================== */
static void test_moveabsolute_owned_fault_sets_error_outputs(void) {
    HYD_MOVEABSOLUTE ma;
    HYD_MotionControlFB* fb;

    __HydMotion_framework_Init();
    ensure_axes_allocated(2);
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

    IEC_VAL(ma.EXECUTE) = true;
    ma.EXECUTE0.value = true;
    __mcl_cmd_MoveAbsolute(&ma);

    fb = __MK_GetPublic_MotionControlFB(0);
    ASSERT_TRUE(fb != NULL, "FB instance should exist for owned fault test");

    fb->STATE.faultActive = true;
    fb->FB_STATE = HYD_FB_STATE_FAULT;
    fb->ERROR_ID = HYD_DIAG_CODE_SENSOR_FAULT;
    fb->DIAGNOSTIC.severity = HYD_DIAG_SEVERITY_FAULT;

    __mcl_cmd_MoveAbsolute(&ma);

    ASSERT_TRUE(IEC_VAL(ma.ERROR) == true,
               "MoveAbsolute should surface ERROR while owned execution is in fault");
    ASSERT_TRUE(IEC_VAL(ma.ERRORID) == HYD_DIAG_CODE_SENSOR_FAULT,
               "MoveAbsolute should surface the active fault code");
    ASSERT_TRUE(IEC_VAL(ma.BUSY) == false,
               "MoveAbsolute BUSY should clear when owned execution faults");
    ASSERT_TRUE(IEC_VAL(ma.ACTIVE) == false,
               "MoveAbsolute ACTIVE should clear when owned execution faults");
}


/* ==================================================================
 * Test 8: MoveAbsolute 非法 AXISID 返回错误
 * ================================================================== */
static void test_moveabsolute_rejects_invalid_axis_index(void) {
    HYD_MOVEABSOLUTE ma;

    __HydMotion_framework_Init();
    ensure_axes_allocated(2);
    memset(&ma, 0, sizeof(ma));

    IEC_VAL(ma.EN) = true;
    IEC_VAL(ma.EXECUTE) = true;
    ma.EXECUTE0.value = false;
    IEC_VAL(ma.AXISID) = -1;  /* 非法 */
    IEC_VAL(ma.POSITION) = 100.0f;
    IEC_VAL(ma.VELOCITY) = 50.0f;
    IEC_VAL(ma.ACCELERATION) = 200.0f;

    __mcl_cmd_MoveAbsolute(&ma);

    ASSERT_TRUE(IEC_VAL(ma.ERROR) == true, "ERROR should be true for invalid AXISID");
    ASSERT_TRUE(IEC_VAL(ma.ERRORID) != 0, "ERRORID should be non-zero for invalid AXISID");
    ASSERT_TRUE(IEC_VAL(ma.BUSY) == false, "BUSY should not be set for invalid AXISID");
}

static void test_moveabsolute_maps_deceleration_independently(void) {
    HYD_MOVEABSOLUTE ma;
    HYD_MotionControlFB* fb;

    __HydMotion_framework_Init();
    ensure_axes_allocated(1);
    memset(&ma, 0, sizeof(ma));

    IEC_VAL(ma.EN) = true;
    IEC_VAL(ma.EXECUTE) = true;
    ma.EXECUTE0.value = false;
    IEC_VAL(ma.AXISID) = 0;
    IEC_VAL(ma.POSITION) = 100.0f;
    IEC_VAL(ma.VELOCITY) = 20.0f;
    IEC_VAL(ma.ACCELERATION) = 50.0f;
    IEC_VAL(ma.DECELERATION) = 7.0f;
    IEC_VAL(ma.DIRECTION) = 1;

    __mcl_cmd_MoveAbsolute(&ma);

    fb = __MK_GetPublic_MotionControlFB(0);
    ASSERT_TRUE(fb != NULL, "MoveAbsolute deceleration test should resolve an FB");
    ASSERT_TRUE(fb->DIRECT_SEGMENT_VALID == true,
               "MoveAbsolute should load a direct segment");
    ASSERT_TRUE(fb->DIRECT_SEGMENT.maxAcceleration == 50.0f,
               "MoveAbsolute should preserve ACCELERATION");
    ASSERT_TRUE(fb->DIRECT_SEGMENT.maxDeceleration == 7.0f,
               "MoveAbsolute should map DECELERATION independently");
}

static void test_loadprofile_preloads_single_recipe_segment(void) {
    HYD_CREATEMOTION cm;
    HYD_LOADPROFILE lp;
    HYD_MotionControlFB* fb;
    HYD_AXISMOTION motion;

    __HydMotion_framework_Init();

    memset(&cm, 0, sizeof(cm));
    IEC_VAL(cm.EN) = true;
    IEC_VAL(cm.USE_RECIPE) = true;
    IEC_VAL(cm.FLOW_TO_PUMPSPEED) = 1.2f;
    IEC_VAL(cm.PUMPSPEED_LIMIT) = 3000.0f;
    IEC_VAL(cm.USE_SIMULATION) = false;
    __mcl_cmd_CreateMotion(&cm);

    memset(&lp, 0, sizeof(lp));
    memset(&motion, 0, sizeof(motion));
    IEC_VAL(lp.EN) = true;
    IEC_VAL(lp.EXECUTE) = true;
    lp.EXECUTE0.value = false;
    IEC_VAL(lp.AXISID) = IEC_VAL(cm.AXISID);

    motion.SEGMENTTAG = 7;
    motion.PLANNER = HYD_PLANNER_TIME_BASED;
    motion.MODE = HYD_MODE_POSITION;
    motion.ENDCONDITION = HYD_END_POSITION;
    motion.DIRECTION = HYD_DIRECTION_EXTEND;
    motion.SETPOSITION = 42.0f;
    motion.SETVELOCITY = 12.0f;
    motion.SETFLOW = 4.0f;
    motion.SETPRESSURE = 0.0f;
    motion.ACCELERATION = 55.0f;
    motion.DURATION = 0.0f;
    motion.PRESSURERAMPRATE = 0.0f;
    __SET_VAR(lp., MOTION, , motion);

    __mcl_cmd_LoadProfile(&lp);

    fb = __MK_GetPublic_MotionControlFB((int)IEC_VAL(cm.AXISID));
    ASSERT_TRUE(fb != NULL, "LoadProfile recipe axis should resolve an FB");
    ASSERT_TRUE(IEC_VAL(lp.DONE) == true, "LoadProfile should set DONE after preload");
    ASSERT_TRUE(fb->RECIPE_SIZE == 1U, "LoadProfile should preload one recipe segment");
    ASSERT_TRUE(fb->DIRECT_SEGMENT_VALID == false,
               "Recipe-axis LoadProfile should not populate DIRECT_SEGMENT");
    ASSERT_TRUE(fb->STATE.active == false,
               "LoadProfile should preload only, not start execution");
}

static void test_loadprofile_keeps_segment_tag_and_type_separate(void) {
    HYD_CREATEMOTION cm;
    HYD_LOADPROFILE lp;
    HYD_MotionControlFB* fb;
    HYD_AXISMOTION motion;

    __HydMotion_framework_Init();

    memset(&cm, 0, sizeof(cm));
    IEC_VAL(cm.EN) = true;
    IEC_VAL(cm.USE_RECIPE) = true;
    IEC_VAL(cm.FLOW_TO_PUMPSPEED) = 1.2f;
    IEC_VAL(cm.PUMPSPEED_LIMIT) = 3000.0f;
    IEC_VAL(cm.USE_SIMULATION) = false;
    __mcl_cmd_CreateMotion(&cm);

    memset(&lp, 0, sizeof(lp));
    memset(&motion, 0, sizeof(motion));
    IEC_VAL(lp.EN) = true;
    IEC_VAL(lp.EXECUTE) = true;
    lp.EXECUTE0.value = false;
    IEC_VAL(lp.AXISID) = IEC_VAL(cm.AXISID);

    motion.SEGMENTTAG = 99;
    motion.SEGMENTTYPE = HYD_SEGMENT_TYPE_HOLDING;
    motion.PLANNER = HYD_PLANNER_TIME_BASED;
    motion.MODE = HYD_MODE_PRESSURE_CLOSED_LOOP;
    motion.ENDCONDITION = HYD_END_TIME;
    motion.DIRECTION = HYD_DIRECTION_HOLD;
    motion.SETFLOW = 4.0f;
    motion.SETPRESSURE = 15.0f;
    motion.ACCELERATION = 55.0f;
    motion.DURATION = 0.5f;
    __SET_VAR(lp., MOTION, , motion);

    __mcl_cmd_LoadProfile(&lp);

    fb = __MK_GetPublic_MotionControlFB((int)IEC_VAL(cm.AXISID));
    ASSERT_TRUE(fb != NULL, "LoadProfile recipe axis should resolve an FB");
    ASSERT_TRUE(IEC_VAL(lp.DONE) == true, "LoadProfile should complete for independent tag/type fields");
    ASSERT_TRUE(fb->RECIPE_SIZE == 1U, "LoadProfile should preload one recipe segment");
    ASSERT_TRUE(fb->RECIPE[0].segmentTag == 99,
               "SEGMENTTAG should remain the opaque process-layer tag");
    ASSERT_TRUE(fb->RECIPE[0].segmentType == HYD_SEGMENT_TYPE_HOLDING,
               "SEGMENTTYPE should define the domain segment type independently");
}

static void test_loadprofile_preserves_independent_accel_and_decel(void) {
    HYD_CREATEMOTION cm;
    HYD_LOADPROFILE lp;
    HYD_MotionControlFB* fb;
    HYD_AXISMOTION motion;

    __HydMotion_framework_Init();

    memset(&cm, 0, sizeof(cm));
    IEC_VAL(cm.EN) = true;
    IEC_VAL(cm.USE_RECIPE) = true;
    IEC_VAL(cm.FLOW_TO_PUMPSPEED) = 1.2f;
    IEC_VAL(cm.PUMPSPEED_LIMIT) = 3000.0f;
    IEC_VAL(cm.USE_SIMULATION) = false;
    __mcl_cmd_CreateMotion(&cm);

    memset(&lp, 0, sizeof(lp));
    memset(&motion, 0, sizeof(motion));
    IEC_VAL(lp.EN) = true;
    IEC_VAL(lp.EXECUTE) = true;
    lp.EXECUTE0.value = false;
    IEC_VAL(lp.AXISID) = IEC_VAL(cm.AXISID);

    motion.SEGMENTTAG = 10;
    motion.MODE = HYD_MODE_POSITION;
    motion.ENDCONDITION = HYD_END_POSITION;
    motion.DIRECTION = HYD_DIRECTION_EXTEND;
    motion.SETPOSITION = 50.0f;
    motion.SETVELOCITY = 12.0f;
    motion.SETFLOW = 5.0f;
    motion.ACCELERATION = 30.0f;
    motion.DECELERATION = 4.0f;
    __SET_VAR(lp., MOTION, , motion);

    __mcl_cmd_LoadProfile(&lp);

    fb = __MK_GetPublic_MotionControlFB((int)IEC_VAL(cm.AXISID));
    ASSERT_TRUE(fb != NULL, "LoadProfile accel/decel test should resolve an FB");
    ASSERT_TRUE(fb->RECIPE_SIZE == 1U, "LoadProfile should preload one recipe segment");
    ASSERT_TRUE(fb->RECIPE[0].maxAcceleration == 30.0f,
               "LoadProfile should preserve ACCELERATION");
    ASSERT_TRUE(fb->RECIPE[0].maxDeceleration == 4.0f,
               "LoadProfile should preserve DECELERATION independently");
}

static void test_loadprofile_preloads_direct_segment_on_direct_axis(void) {
    HYD_CREATEMOTION cm;
    HYD_LOADPROFILE lp;
    HYD_MotionControlFB* fb;
    HYD_AXISMOTION motion;

    __HydMotion_framework_Init();

    memset(&cm, 0, sizeof(cm));
    IEC_VAL(cm.EN) = true;
    IEC_VAL(cm.USE_RECIPE) = false;
    IEC_VAL(cm.FLOW_TO_PUMPSPEED) = 1.2f;
    IEC_VAL(cm.PUMPSPEED_LIMIT) = 3000.0f;
    IEC_VAL(cm.USE_SIMULATION) = false;
    __mcl_cmd_CreateMotion(&cm);

    memset(&lp, 0, sizeof(lp));
    memset(&motion, 0, sizeof(motion));
    IEC_VAL(lp.EN) = true;
    IEC_VAL(lp.EXECUTE) = true;
    lp.EXECUTE0.value = false;
    IEC_VAL(lp.AXISID) = IEC_VAL(cm.AXISID);

    motion.SEGMENTTAG = 8;
    motion.PLANNER = HYD_PLANNER_TIME_BASED;
    motion.MODE = HYD_MODE_SPEED_RAMP;
    motion.ENDCONDITION = HYD_END_TIME;
    motion.DIRECTION = HYD_DIRECTION_EXTEND;
    motion.SETVELOCITY = 15.0f;
    motion.SETFLOW = 6.0f;
    motion.ACCELERATION = 80.0f;
    motion.DURATION = 0.5f;
    __SET_VAR(lp., MOTION, , motion);

    __mcl_cmd_LoadProfile(&lp);

    fb = __MK_GetPublic_MotionControlFB((int)IEC_VAL(cm.AXISID));
    ASSERT_TRUE(fb != NULL, "LoadProfile direct axis should resolve an FB");
    ASSERT_TRUE(IEC_VAL(lp.DONE) == true, "LoadProfile should set DONE on direct preload");
    ASSERT_TRUE(fb->DIRECT_SEGMENT_VALID == true,
               "Direct-axis LoadProfile should populate DIRECT_SEGMENT");
    ASSERT_TRUE(fb->RECIPE_SIZE == 0U,
               "Direct-axis LoadProfile should not populate RECIPE");
    ASSERT_TRUE(fb->STATE.active == false,
               "Direct-axis LoadProfile should preload only, not start execution");
}

static void test_loadprofile_keeps_recipe_preload_target_after_direct_override(void) {
    HYD_CREATEMOTION cm;
    HYD_LOADPROFILE lp;
    HYD_AXISMOTION motion;
    HYD_MOVEABSOLUTE ma;
    HYD_MotionControlFB* fb;

    __HydMotion_framework_Init();

    memset(&cm, 0, sizeof(cm));
    IEC_VAL(cm.EN) = true;
    IEC_VAL(cm.USE_RECIPE) = true;
    IEC_VAL(cm.FLOW_TO_PUMPSPEED) = 1.2f;
    IEC_VAL(cm.PUMPSPEED_LIMIT) = 3000.0f;
    IEC_VAL(cm.USE_SIMULATION) = false;
    __mcl_cmd_CreateMotion(&cm);
    fb = __MK_GetPublic_MotionControlFB((int)IEC_VAL(cm.AXISID));
    ASSERT_TRUE(fb != NULL, "Recipe axis should expose an FB");

    memset(&ma, 0, sizeof(ma));
    IEC_VAL(ma.EN) = true;
    IEC_VAL(ma.EXECUTE) = true;
    ma.EXECUTE0.value = false;
    IEC_VAL(ma.AXISID) = IEC_VAL(cm.AXISID);
    IEC_VAL(ma.POSITION) = 10.0f;
    IEC_VAL(ma.VELOCITY) = 5.0f;
    IEC_VAL(ma.ACCELERATION) = 20.0f;
    IEC_VAL(ma.DIRECTION) = 1;
    __mcl_cmd_MoveAbsolute(&ma);

    ASSERT_TRUE(fb->USE_RECIPE == false,
               "Direct command may temporarily switch the start selector to direct");

    memset(&lp, 0, sizeof(lp));
    memset(&motion, 0, sizeof(motion));
    IEC_VAL(lp.EN) = true;
    IEC_VAL(lp.EXECUTE) = true;
    lp.EXECUTE0.value = false;
    IEC_VAL(lp.AXISID) = IEC_VAL(cm.AXISID);

    motion.SEGMENTTAG = 9;
    motion.PLANNER = HYD_PLANNER_TIME_BASED;
    motion.MODE = HYD_MODE_POSITION;
    motion.ENDCONDITION = HYD_END_POSITION;
    motion.DIRECTION = HYD_DIRECTION_EXTEND;
    motion.SETPOSITION = 60.0f;
    motion.SETVELOCITY = 12.0f;
    motion.SETFLOW = 5.0f;
    motion.ACCELERATION = 70.0f;
    __SET_VAR(lp., MOTION, , motion);

    __mcl_cmd_LoadProfile(&lp);

    ASSERT_TRUE(IEC_VAL(lp.DONE) == true,
               "LoadProfile should still complete on the recipe-configured axis");
    ASSERT_TRUE(fb->RECIPE_SIZE == 1U,
               "Recipe-configured axis should still preload RECIPE after direct override");
    ASSERT_TRUE(fb->RECIPE[0].targetPosition == 60.0f,
               "Recipe-configured axis should store the new preloaded recipe segment");
}

static void test_moveabsolute_rejects_nonzero_jerk_until_supported(void) {
    HYD_MOVEABSOLUTE ma;

    __HydMotion_framework_Init();
    ensure_axes_allocated(1);
    memset(&ma, 0, sizeof(ma));

    IEC_VAL(ma.EN) = true;
    IEC_VAL(ma.EXECUTE) = true;
    ma.EXECUTE0.value = false;
    IEC_VAL(ma.AXISID) = 0;
    IEC_VAL(ma.POSITION) = 100.0f;
    IEC_VAL(ma.VELOCITY) = 20.0f;
    IEC_VAL(ma.ACCELERATION) = 50.0f;
    IEC_VAL(ma.JERK) = 1.0f;

    __mcl_cmd_MoveAbsolute(&ma);

    ASSERT_TRUE(IEC_VAL(ma.ERROR) == true,
               "MoveAbsolute should reject unsupported nonzero JERK");
    ASSERT_TRUE(IEC_VAL(ma.ERRORID) == HYD_DIAG_CODE_COMMAND_NOT_ALLOWED,
               "Unsupported JERK should surface COMMAND_NOT_ALLOWED");
}

static void test_movevelocity_accepts_continuousupdate_and_updates_active_target(void) {
    HYD_MOVEVELOCITY mv;
    HYD_MotionControlFB* fb;

    __HydMotion_framework_Init();
    ensure_axes_allocated(1);
    memset(&mv, 0, sizeof(mv));

    IEC_VAL(mv.EN) = true;
    IEC_VAL(mv.EXECUTE) = true;
    mv.EXECUTE0.value = false;
    IEC_VAL(mv.AXISID) = 0;
    IEC_VAL(mv.VELOCITY) = 25.0f;
    IEC_VAL(mv.ACCELERATION) = 100.0f;
    IEC_VAL(mv.DIRECTION) = 1;
    IEC_VAL(mv.CONTINUOUSUPDATE) = true;
    IEC_VAL(mv.PRESSURELIMIT) = 140.0f;

    __mcl_cmd_MoveVelocity(&mv);

    ASSERT_TRUE(IEC_VAL(mv.ERROR) == false,
               "MoveVelocity should accept supported CONTINUOUSUPDATE");

    __HydMotion_framework_Publish();
    mv.EXECUTE0.value = true;
    __mcl_cmd_MoveVelocity(&mv);

    fb = __MK_GetPublic_MotionControlFB(0);
    ASSERT_TRUE(fb != NULL, "Allocated FB should exist");
    ASSERT_TRUE(fb->_activeSegmentValid, "MoveVelocity should have an active segment");
    ASSERT_TRUE(fabs(fb->_activeSegment.maxVelocity - 25.0f) < 0.001f,
               "MoveVelocity should latch initial velocity target");
    ASSERT_TRUE(fabsf(fb->_activeSegment.maxPressure - 140.0f) <= 1e-6f,
               "MoveVelocity should latch the initial pressure limit");

    IEC_VAL(mv.VELOCITY) = 35.0f;
    IEC_VAL(mv.PRESSURELIMIT) = 110.0f;
    __mcl_cmd_MoveVelocity(&mv);

    ASSERT_TRUE(IEC_VAL(mv.ERROR) == false,
               "MoveVelocity continuous update should not raise ERROR");
    ASSERT_TRUE(fabs(fb->_activeSegment.maxVelocity - 35.0f) < 0.001f,
               "MoveVelocity continuous update should update active maxVelocity");
    ASSERT_TRUE(fabsf(fb->_activeSegment.maxPressure - 110.0f) <= 1e-6f,
               "MoveVelocity continuous update should update the active pressure limit");

    fb->PRESSURE_LIMIT = 90.0f;
    IEC_VAL(mv.PRESSURELIMIT) = 0.0f;
    __mcl_cmd_MoveVelocity(&mv);

    ASSERT_TRUE(fabsf(fb->_activeSegment.maxPressure - 90.0f) <= 1e-6f,
               "MoveVelocity continuous update should apply the axis pressure default");
}

static void test_movevelocity_continuousupdate_lower_target_preserves_ramp(void) {
    HYD_MOVEVELOCITY mv;
    HYD_MotionControlFB* fb;
    HYD_REAL previousVelocity;
    HYD_REAL previousFlow;
    int axisId;
    int step;

    __HydMotion_framework_Init();
    axisId = create_sim_axis();
    fb = __MK_GetPublic_MotionControlFB(axisId);
    memset(&mv, 0, sizeof(mv));

    IEC_VAL(mv.EN) = true;
    IEC_VAL(mv.EXECUTE) = true;
    mv.EXECUTE0.value = false;
    IEC_VAL(mv.AXISID) = axisId;
    IEC_VAL(mv.VELOCITY) = 60.0f;
    IEC_VAL(mv.ACCELERATION) = 1000.0f;
    IEC_VAL(mv.DECELERATION) = 10.0f;
    IEC_VAL(mv.DIRECTION) = 1;
    IEC_VAL(mv.CONTINUOUSUPDATE) = true;

    __mcl_cmd_MoveVelocity(&mv);
    __HydMotion_framework_Publish();
    mv.EXECUTE0.value = true;
    __mcl_cmd_MoveVelocity(&mv);

    for (step = 0; step < 200; ++step) {
        __HydMotion_framework_Publish();
        __mcl_cmd_MoveVelocity(&mv);
        if (fb->STATE.plannedVelocity >= 55.0f) {
            break;
        }
    }

    ASSERT_TRUE(step < 200,
               "MoveVelocity should accelerate near the original target before live update");

    previousVelocity = fb->STATE.plannedVelocity;
    previousFlow = fb->STATE.plannedFlow;

    IEC_VAL(mv.VELOCITY) = 30.0f;
    __mcl_cmd_MoveVelocity(&mv);

    __HydMotion_framework_Publish();
    __mcl_cmd_MoveVelocity(&mv);

    ASSERT_TRUE(fb->STATE.plannedVelocity < previousVelocity,
               "MoveVelocity lower target should begin decelerating");
    ASSERT_TRUE(fb->STATE.plannedVelocity > 50.0f,
               "MoveVelocity lower target should stay above the new target on the first decel cycle");
    ASSERT_TRUE(fb->STATE.plannedFlow > 10.0f,
               "MoveVelocity lower target should not clamp flow to the new steady-state value immediately");
    ASSERT_TRUE(fb->STATE.plannedFlow < previousFlow,
               "MoveVelocity lower target should still reduce flow");
}

static void test_movevelocity_continuousupdate_rejects_nonfinite_pressure_limit(void) {
    HYD_MOVEVELOCITY mv;
    HYD_MotionControlFB* fb;

    __HydMotion_framework_Init();
    ensure_axes_allocated(1);
    fb = __MK_GetPublic_MotionControlFB(0);
    memset(&mv, 0, sizeof(mv));

    IEC_VAL(mv.EN) = true;
    IEC_VAL(mv.EXECUTE) = true;
    mv.EXECUTE0.value = false;
    IEC_VAL(mv.AXISID) = 0;
    IEC_VAL(mv.VELOCITY) = 25.0f;
    IEC_VAL(mv.ACCELERATION) = 100.0f;
    IEC_VAL(mv.DIRECTION) = 1;
    IEC_VAL(mv.CONTINUOUSUPDATE) = true;
    IEC_VAL(mv.PRESSURELIMIT) = 140.0f;

    __mcl_cmd_MoveVelocity(&mv);
    __HydMotion_framework_Publish();
    mv.EXECUTE0.value = true;
    __mcl_cmd_MoveVelocity(&mv);

    ASSERT_TRUE(fb != NULL && fb->_activeSegmentValid,
               "MoveVelocity nonfinite live-update test should start an active segment");
    if (fb == NULL || !fb->_activeSegmentValid) {
        return;
    }

    IEC_VAL(mv.PRESSURELIMIT) = NAN;
    __mcl_cmd_MoveVelocity(&mv);

    ASSERT_TRUE(IEC_VAL(mv.ERROR) == true,
               "MoveVelocity continuous update should reject a nonfinite PRESSURELIMIT");
    ASSERT_TRUE(IEC_VAL(mv.ERRORID) == HYD_DIAG_CODE_COMMAND_NOT_ALLOWED,
               "Nonfinite live PRESSURELIMIT should surface COMMAND_NOT_ALLOWED");
    ASSERT_TRUE(fabsf(fb->_activeSegment.maxPressure - 140.0f) <= 1e-6f,
               "Rejected live PRESSURELIMIT should not modify the active segment");
}

static void test_movevelocity_pressure_limit_stays_latched_without_continuousupdate(void) {
    HYD_MOVEVELOCITY mv;
    HYD_MotionControlFB* fb;

    __HydMotion_framework_Init();
    ensure_axes_allocated(1);
    fb = __MK_GetPublic_MotionControlFB(0);
    memset(&mv, 0, sizeof(mv));

    IEC_VAL(mv.EN) = true;
    IEC_VAL(mv.EXECUTE) = true;
    mv.EXECUTE0.value = false;
    IEC_VAL(mv.AXISID) = 0;
    IEC_VAL(mv.VELOCITY) = 25.0f;
    IEC_VAL(mv.ACCELERATION) = 100.0f;
    IEC_VAL(mv.DIRECTION) = 1;
    IEC_VAL(mv.CONTINUOUSUPDATE) = false;
    IEC_VAL(mv.PRESSURELIMIT) = 140.0f;

    __mcl_cmd_MoveVelocity(&mv);
    __HydMotion_framework_Publish();
    mv.EXECUTE0.value = true;
    __mcl_cmd_MoveVelocity(&mv);

    ASSERT_TRUE(fb != NULL && fb->_activeSegmentValid,
               "MoveVelocity latch test should start an active segment");
    if (fb == NULL || !fb->_activeSegmentValid) {
        return;
    }

    IEC_VAL(mv.PRESSURELIMIT) = 110.0f;
    __mcl_cmd_MoveVelocity(&mv);

    ASSERT_TRUE(fabsf(fb->_activeSegment.maxPressure - 140.0f) <= 1e-6f,
               "MoveVelocity should keep PRESSURELIMIT latched when CONTINUOUSUPDATE is false");
}

static void test_moveabsolute_continuousupdate_lower_velocity_preserves_ramp(void) {
    HYD_MOVEABSOLUTE ma;
    HYD_MotionControlFB* fb;
    HYD_REAL previousVelocity;
    HYD_REAL previousFlow;
    int axisId;
    int step;

    __HydMotion_framework_Init();
    axisId = create_sim_axis();
    fb = __MK_GetPublic_MotionControlFB(axisId);
    memset(&ma, 0, sizeof(ma));

    IEC_VAL(ma.EN) = true;
    IEC_VAL(ma.EXECUTE) = true;
    ma.EXECUTE0.value = false;
    IEC_VAL(ma.AXISID) = axisId;
    IEC_VAL(ma.POSITION) = 10000.0f;
    IEC_VAL(ma.VELOCITY) = 60.0f;
    IEC_VAL(ma.ACCELERATION) = 1000.0f;
    IEC_VAL(ma.DECELERATION) = 10.0f;
    IEC_VAL(ma.DIRECTION) = 1;
    IEC_VAL(ma.CONTINUOUSUPDATE) = true;

    __mcl_cmd_MoveAbsolute(&ma);
    __HydMotion_framework_Publish();
    ma.EXECUTE0.value = true;
    __mcl_cmd_MoveAbsolute(&ma);

    for (step = 0; step < 200; ++step) {
        __HydMotion_framework_Publish();
        __mcl_cmd_MoveAbsolute(&ma);
        if (fb->STATE.plannedVelocity >= 55.0f) {
            break;
        }
    }

    ASSERT_TRUE(step < 200,
               "MoveAbsolute should accelerate near the original target before live update");

    previousVelocity = fb->STATE.plannedVelocity;
    previousFlow = fb->STATE.plannedFlow;

    IEC_VAL(ma.VELOCITY) = 30.0f;
    __mcl_cmd_MoveAbsolute(&ma);

    __HydMotion_framework_Publish();
    __mcl_cmd_MoveAbsolute(&ma);

    ASSERT_TRUE(fb->STATE.plannedVelocity < previousVelocity,
               "MoveAbsolute lower velocity should begin decelerating");
    ASSERT_TRUE(fb->STATE.plannedVelocity > 50.0f,
               "MoveAbsolute lower velocity should stay above the new target on the first decel cycle");
    ASSERT_TRUE(fb->STATE.plannedFlow > 10.0f,
               "MoveAbsolute lower velocity should not clamp flow to the new steady-state value immediately");
    ASSERT_TRUE(fb->STATE.plannedFlow < previousFlow,
               "MoveAbsolute lower velocity should still reduce flow");
}

static void test_moveabsolute_accepts_beckhoff_buffer_modes(void) {
    HYD_MOVEABSOLUTE ma;
    int mode;

    for (mode = 0; mode <= 5; ++mode) {
        __HydMotion_framework_Init();
        ensure_axes_allocated(1);
        memset(&ma, 0, sizeof(ma));

        IEC_VAL(ma.EN) = true;
        IEC_VAL(ma.EXECUTE) = true;
        ma.EXECUTE0.value = false;
        IEC_VAL(ma.AXISID) = 0;
        IEC_VAL(ma.POSITION) = 50.0f;
        IEC_VAL(ma.VELOCITY) = 10.0f;
        IEC_VAL(ma.ACCELERATION) = 40.0f;
        IEC_VAL(ma.DIRECTION) = 1;
        IEC_VAL(ma.BUFFERMODE) = mode;

        __mcl_cmd_MoveAbsolute(&ma);

        ASSERT_TRUE(IEC_VAL(ma.ERROR) == false,
                   "MoveAbsolute should accept Beckhoff BUFFERMODE values 0..5");
    }

    __HydMotion_framework_Init();
    ensure_axes_allocated(1);
    memset(&ma, 0, sizeof(ma));
    IEC_VAL(ma.EN) = true;
    IEC_VAL(ma.EXECUTE) = true;
    ma.EXECUTE0.value = false;
    IEC_VAL(ma.AXISID) = 0;
    IEC_VAL(ma.POSITION) = 50.0f;
    IEC_VAL(ma.VELOCITY) = 10.0f;
    IEC_VAL(ma.ACCELERATION) = 40.0f;
    IEC_VAL(ma.DIRECTION) = 1;
    IEC_VAL(ma.BUFFERMODE) = 6;

    __mcl_cmd_MoveAbsolute(&ma);
    ASSERT_TRUE(IEC_VAL(ma.ERROR) == true,
               "MoveAbsolute should reject BUFFERMODE values outside 0..5");
    ASSERT_TRUE(IEC_VAL(ma.ERRORID) == HYD_DIAG_CODE_COMMAND_NOT_ALLOWED,
               "Invalid BUFFERMODE should surface COMMAND_NOT_ALLOWED");
}

/* ==================================================================
 * Test 9: MoveVelocity EXECUTE 上升沿 启动速度控制
 * ================================================================== */
static void test_movevelocity_execute_rising_starts_velocity_control(void) {
    HYD_MOVEVELOCITY mv;

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

    ASSERT_TRUE(IEC_VAL(mv.BUSY) == true, "MoveVelocity should set BUSY on execRising");
    ASSERT_TRUE(IEC_VAL(mv.ACTIVE) == true, "MoveVelocity should set ACTIVE on execRising");
    ASSERT_TRUE(IEC_VAL(mv.INVELOCITY) == false, "INVELOCITY should be false initially");
    ASSERT_TRUE(IEC_VAL(mv.COMMANDABORTED) == false, "COMMANDABORTED should be false initially");
}


/* ==================================================================
 * Test 11: MoveVelocity 非法 AXISID
 * ================================================================== */
static void test_movevelocity_rejects_invalid_axis_index(void) {
    HYD_MOVEVELOCITY mv;

    __HydMotion_framework_Init();
    ensure_axes_allocated(2);
    memset(&mv, 0, sizeof(mv));

    IEC_VAL(mv.EN) = true;
    IEC_VAL(mv.EXECUTE) = true;
    mv.EXECUTE0.value = false;
    IEC_VAL(mv.AXISID) = HYD_MAX_AXIS_MOTION + 1;  /* 超出范围 */
    IEC_VAL(mv.VELOCITY) = 30.0f;

    __mcl_cmd_MoveVelocity(&mv);

    ASSERT_TRUE(IEC_VAL(mv.ERROR) == true, "ERROR should be true for invalid AXISID");
    ASSERT_TRUE(IEC_VAL(mv.ERRORID) == HYD_DIAG_CODE_START_CONTEXT_INVALID,
               "ERRORID should be START_CONTEXT_INVALID");
}

static void test_movevelocity_maps_deceleration_independently(void) {
    HYD_MOVEVELOCITY mv;
    HYD_MotionControlFB* fb;

    __HydMotion_framework_Init();
    ensure_axes_allocated(1);
    memset(&mv, 0, sizeof(mv));

    IEC_VAL(mv.EN) = true;
    IEC_VAL(mv.EXECUTE) = true;
    mv.EXECUTE0.value = false;
    IEC_VAL(mv.AXISID) = 0;
    IEC_VAL(mv.VELOCITY) = 25.0f;
    IEC_VAL(mv.ACCELERATION) = 100.0f;
    IEC_VAL(mv.DECELERATION) = 6.0f;
    IEC_VAL(mv.DIRECTION) = 1;

    __mcl_cmd_MoveVelocity(&mv);

    fb = __MK_GetPublic_MotionControlFB(0);
    ASSERT_TRUE(fb != NULL, "MoveVelocity deceleration test should resolve an FB");
    ASSERT_TRUE(fb->DIRECT_SEGMENT_VALID == true,
               "MoveVelocity should load a direct segment");
    ASSERT_TRUE(fb->DIRECT_SEGMENT.maxAcceleration == 100.0f,
               "MoveVelocity should preserve ACCELERATION");
    ASSERT_TRUE(fb->DIRECT_SEGMENT.maxDeceleration == 6.0f,
               "MoveVelocity should map DECELERATION independently");
}

static void test_movevelocity_maps_explicit_pressure_limit_to_direct_segment(void) {
    HYD_MOVEVELOCITY mv;
    HYD_MotionControlFB* fb;

    __HydMotion_framework_Init();
    ensure_axes_allocated(1);
    memset(&mv, 0, sizeof(mv));

    IEC_VAL(mv.EN) = true;
    IEC_VAL(mv.EXECUTE) = true;
    mv.EXECUTE0.value = false;
    IEC_VAL(mv.AXISID) = 0;
    IEC_VAL(mv.VELOCITY) = 25.0f;
    IEC_VAL(mv.ACCELERATION) = 100.0f;
    IEC_VAL(mv.DECELERATION) = 6.0f;
    IEC_VAL(mv.DIRECTION) = 1;
    IEC_VAL(mv.PRESSURELIMIT) = 120.0f;

    __mcl_cmd_MoveVelocity(&mv);

    fb = __MK_GetPublic_MotionControlFB(0);
    ASSERT_TRUE(fb != NULL, "MoveVelocity pressure-limit test should resolve an FB");
    ASSERT_TRUE(IEC_VAL(mv.ERROR) == false,
               "MoveVelocity should accept a finite PRESSURELIMIT");
    ASSERT_TRUE(fb->DIRECT_SEGMENT_VALID == true,
               "MoveVelocity should load a direct segment for pressure limiting");
    ASSERT_TRUE(fabsf(fb->DIRECT_SEGMENT.maxPressure - 120.0f) <= 1e-6f,
               "MoveVelocity PRESSURELIMIT should enter segment.maxPressure in bar");
}

static void test_movevelocity_nonpositive_pressure_limit_uses_axis_default(void) {
    const HYD_REAL inputs[] = {0.0f, -1.0f};

    for (size_t i = 0; i < sizeof(inputs) / sizeof(inputs[0]); ++i) {
        HYD_MOVEVELOCITY mv;
        HYD_MotionControlFB* fb;

        __HydMotion_framework_Init();
        ensure_axes_allocated(1);
        fb = __MK_GetPublic_MotionControlFB(0);
        memset(&mv, 0, sizeof(mv));

        ASSERT_TRUE(fb != NULL, "MoveVelocity fallback test should resolve an FB");
        if (fb == NULL) {
            continue;
        }
        fb->PRESSURE_LIMIT = 180.0f;

        IEC_VAL(mv.EN) = true;
        IEC_VAL(mv.EXECUTE) = true;
        mv.EXECUTE0.value = false;
        IEC_VAL(mv.AXISID) = 0;
        IEC_VAL(mv.VELOCITY) = 25.0f;
        IEC_VAL(mv.ACCELERATION) = 100.0f;
        IEC_VAL(mv.DECELERATION) = 6.0f;
        IEC_VAL(mv.DIRECTION) = 1;
        IEC_VAL(mv.PRESSURELIMIT) = inputs[i];

        __mcl_cmd_MoveVelocity(&mv);

        ASSERT_TRUE(IEC_VAL(mv.ERROR) == false,
                   "MoveVelocity should accept a nonpositive PRESSURELIMIT as fallback");
        ASSERT_TRUE(fabsf(fb->DIRECT_SEGMENT.maxPressure - 180.0f) <= 1e-6f,
                   "MoveVelocity nonpositive PRESSURELIMIT should use the axis default");
    }
}

static void test_movevelocity_rejects_nonfinite_pressure_limit(void) {
    const HYD_REAL inputs[] = {NAN, INFINITY, -INFINITY};

    for (size_t i = 0; i < sizeof(inputs) / sizeof(inputs[0]); ++i) {
        HYD_MOVEVELOCITY mv;
        HYD_MotionControlFB* fb;

        __HydMotion_framework_Init();
        ensure_axes_allocated(1);
        fb = __MK_GetPublic_MotionControlFB(0);
        memset(&mv, 0, sizeof(mv));

        IEC_VAL(mv.EN) = true;
        IEC_VAL(mv.EXECUTE) = true;
        mv.EXECUTE0.value = false;
        IEC_VAL(mv.AXISID) = 0;
        IEC_VAL(mv.VELOCITY) = 25.0f;
        IEC_VAL(mv.ACCELERATION) = 100.0f;
        IEC_VAL(mv.DECELERATION) = 6.0f;
        IEC_VAL(mv.DIRECTION) = 1;
        IEC_VAL(mv.PRESSURELIMIT) = inputs[i];

        __mcl_cmd_MoveVelocity(&mv);

        ASSERT_TRUE(IEC_VAL(mv.ERROR) == true,
                   "MoveVelocity should reject a nonfinite PRESSURELIMIT");
        ASSERT_TRUE(IEC_VAL(mv.ERRORID) == HYD_DIAG_CODE_COMMAND_NOT_ALLOWED,
                   "Nonfinite MoveVelocity PRESSURELIMIT should surface COMMAND_NOT_ALLOWED");
        ASSERT_TRUE(fb != NULL && fb->DIRECT_SEGMENT_VALID == false,
                   "Nonfinite MoveVelocity PRESSURELIMIT should not start a direct segment");
    }
}

/* ==================================================================
 * Test 12: Stop 在空闲轴上立即完成
 * ================================================================== */
static void test_stop_on_idle_axis_immediate_done(void) {
    HYD_STOP stop;

    __HydMotion_framework_Init();
    ensure_axes_allocated(2);
    memset(&stop, 0, sizeof(stop));

    IEC_VAL(stop.EN) = true;
    IEC_VAL(stop.EXECUTE) = true;
    stop.EXECUTE0.value = false;
    IEC_VAL(stop.AXISID) = 0;

    __mcl_cmd_Stop(&stop);
    __HydMotion_framework_Publish();

    /* 下一周期确认 */
    IEC_VAL(stop.EXECUTE) = true;
    stop.EXECUTE0.value = true;
    __mcl_cmd_Stop(&stop);

    /* Stop应该已经完成 (轴从未激活) */
    ASSERT_TRUE(IEC_VAL(stop.DONE) == true, "Stop on idle axis should set DONE immediately");
    ASSERT_TRUE(IEC_VAL(stop.BUSY) == false, "Stop BUSY should be false after DONE");
    ASSERT_TRUE(IEC_VAL(stop.COMMANDABORTED) == false,
               "COMMANDABORTED should be false for successful Stop");
}



/* ==================================================================
 * Test 14: Stop 非法 AXISID
 * ================================================================== */
static void test_stop_rejects_invalid_axis_index(void) {
    HYD_STOP stop;

    __HydMotion_framework_Init();
    ensure_axes_allocated(2);
    memset(&stop, 0, sizeof(stop));

    IEC_VAL(stop.EN) = true;
    IEC_VAL(stop.EXECUTE) = true;
    stop.EXECUTE0.value = false;
    IEC_VAL(stop.AXISID) = -1;

    __mcl_cmd_Stop(&stop);

    ASSERT_TRUE(IEC_VAL(stop.ERROR) == true, "Stop should set ERROR for invalid AXISID");
    ASSERT_TRUE(IEC_VAL(stop.ERRORID) != 0, "Stop should set non-zero ERRORID for invalid input");
}

/* ==================================================================
 * Test 15: Reset 在已初始化轴上立即完成
 * ================================================================== */
static void test_reset_immediate_done_on_initialized_axis(void) {
    HYD_MOVEABSOLUTE ma;
    HYD_RESET reset;

    __HydMotion_framework_Init();
    ensure_axes_allocated(2);
    memset(&ma, 0, sizeof(ma));

    /* 先用 MoveAbsolute 初始化轴 */
    IEC_VAL(ma.EN) = true;
    IEC_VAL(ma.EXECUTE) = true;
    ma.EXECUTE0.value = false;
    IEC_VAL(ma.AXISID) = 0;
    IEC_VAL(ma.POSITION) = 100.0f;
    IEC_VAL(ma.VELOCITY) = 50.0f;
    IEC_VAL(ma.ACCELERATION) = 200.0f;
    __mcl_cmd_MoveAbsolute(&ma);

    /* Reset */
    memset(&reset, 0, sizeof(reset));
    IEC_VAL(reset.EN) = true;
    IEC_VAL(reset.EXECUTE) = true;
    reset.EXECUTE0.value = false;
    IEC_VAL(reset.AXISID) = 0;

    __mcl_cmd_Reset(&reset);

    ASSERT_TRUE(IEC_VAL(reset.DONE) == true, "Reset should set DONE immediately");
    ASSERT_TRUE(IEC_VAL(reset.BUSY) == false, "Reset BUSY should be false after DONE");
}

/* ==================================================================
 * Test 16: Reset 在未初始化轴上立即返回DONE
 * ================================================================== */
static void test_reset_immediate_done_on_uninitialized_axis(void) {
    HYD_RESET reset;

    __HydMotion_framework_Init();
    ensure_axes_allocated(2);
    memset(&reset, 0, sizeof(reset));

    IEC_VAL(reset.EN) = true;
    IEC_VAL(reset.EXECUTE) = true;
    reset.EXECUTE0.value = false;
    IEC_VAL(reset.AXISID) = 5;  /* 未通过CreateMotion分配的轴 */

    __mcl_cmd_Reset(&reset);

    ASSERT_TRUE(IEC_VAL(reset.ERROR) == true,
               "Reset on unallocated axis should return ERROR");
    ASSERT_TRUE(IEC_VAL(reset.BUSY) == false,
               "Reset BUSY should be false on unallocated axis");
}

static void test_reset_preserves_direct_segment_configuration(void) {
    HYD_MOVEABSOLUTE ma;
    HYD_RESET reset;
    HYD_MotionControlFB* fb;

    __HydMotion_framework_Init();
    ensure_axes_allocated(2);
    memset(&ma, 0, sizeof(ma));

    IEC_VAL(ma.EN) = true;
    IEC_VAL(ma.EXECUTE) = true;
    ma.EXECUTE0.value = false;
    IEC_VAL(ma.AXISID) = 0;
    IEC_VAL(ma.POSITION) = 123.0f;
    IEC_VAL(ma.VELOCITY) = 40.0f;
    IEC_VAL(ma.ACCELERATION) = 200.0f;
    IEC_VAL(ma.DIRECTION) = 1;
    __mcl_cmd_MoveAbsolute(&ma);
    __HydMotion_framework_Publish();

    fb = __MK_GetPublic_MotionControlFB(0);
    ASSERT_TRUE(fb != NULL, "FB instance should exist");
    ASSERT_TRUE(fb->DIRECT_SEGMENT_VALID == true,
               "Direct segment should be loaded before reset");
    fb->PRESSURE_LIMIT = 180.0f;

    memset(&reset, 0, sizeof(reset));
    IEC_VAL(reset.EN) = true;
    IEC_VAL(reset.EXECUTE) = true;
    reset.EXECUTE0.value = false;
    IEC_VAL(reset.AXISID) = 0;
    __mcl_cmd_Reset(&reset);

    ASSERT_TRUE(fb->DIRECT_SEGMENT_VALID == true,
               "SoftReset should preserve the direct segment");
    ASSERT_TRUE(fb->PRESSURE_LIMIT == 180.0f,
               "SoftReset should preserve the axis pressure limit");
    ASSERT_TRUE(fb->STATE.active == false,
               "SoftReset should clear active execution state");
}

/* ==================================================================
 * Test 17: PressureHandle EXECUTE 上升沿 启动压力控制
 * ================================================================== */
static void test_pressurehandle_execute_rising_starts_pressure_control(void) {
    HYD_PRESSUREHANDLE ph;

    __HydMotion_framework_Init();
    ensure_axes_allocated(2);
    memset(&ph, 0, sizeof(ph));

    IEC_VAL(ph.EN) = true;
    IEC_VAL(ph.EXECUTE) = true;
    ph.EXECUTE0.value = false;
    IEC_VAL(ph.AXISID) = 0;
    IEC_VAL(ph.PRESSURE) = 10.0f;
    IEC_VAL(ph.PRESSURERAMPRATE) = 2.0f;
    IEC_VAL(ph.DURATION) = 5.0f;

    __mcl_cmd_PressureHandle(&ph);

    ASSERT_TRUE(IEC_VAL(ph.BUSY) == true, "PressureHandle should set BUSY on execRising");
    ASSERT_TRUE(IEC_VAL(ph.ACTIVE) == true, "PressureHandle should set ACTIVE on execRising");
    ASSERT_TRUE(IEC_VAL(ph.INPRESSURE) == false,
               "INPRESSURE should be false initially (pressure=0)");
    ASSERT_TRUE(IEC_VAL(ph.COMMANDABORTED) == false,
               "COMMANDABORTED should be false initially");
}

static void test_pressurehandle_accepts_continuousupdate_and_updates_active_target(void) {
    HYD_PRESSUREHANDLE ph;
    HYD_MotionControlFB* fb;

    __HydMotion_framework_Init();
    ensure_axes_allocated(1);
    memset(&ph, 0, sizeof(ph));

    IEC_VAL(ph.EN) = true;
    IEC_VAL(ph.EXECUTE) = true;
    ph.EXECUTE0.value = false;
    IEC_VAL(ph.AXISID) = 0;
    IEC_VAL(ph.PRESSURE) = 8.0f;
    IEC_VAL(ph.PRESSURERAMPRATE) = 2.0f;
    IEC_VAL(ph.DURATION) = 1.0f;
    IEC_VAL(ph.CONTINUOUSUPDATE) = true;

    __mcl_cmd_PressureHandle(&ph);

    ASSERT_TRUE(IEC_VAL(ph.ERROR) == false,
               "PressureHandle should expose and accept CONTINUOUSUPDATE");

    __HydMotion_framework_Publish();
    ph.EXECUTE0.value = true;
    __mcl_cmd_PressureHandle(&ph);

    fb = __MK_GetPublic_MotionControlFB(0);
    ASSERT_TRUE(fb != NULL, "PressureHandle continuous update test should resolve FB");
    ASSERT_TRUE(fb->_activeSegmentValid, "PressureHandle should have an active segment before update");

    IEC_VAL(ph.PRESSURE) = 12.0f;
    IEC_VAL(ph.PRESSURERAMPRATE) = 4.0f;
    __mcl_cmd_PressureHandle(&ph);

    ASSERT_TRUE(IEC_VAL(ph.ERROR) == false,
               "PressureHandle continuous update should not raise ERROR");
    ASSERT_TRUE(fabs(fb->_activeSegment.targetPressure - 12.0f) < 0.001f,
               "PressureHandle continuous update should update active targetPressure");
    ASSERT_TRUE(fabs(fb->_activeSegment.pressureRampRate - 4.0f) < 0.001f,
               "PressureHandle continuous update should update active pressureRampRate");
}

/* ==================================================================
 * Test 18: PressureHandle EN=false 清除输出
 * ================================================================== */
static void test_pressurehandle_en_false_clears_outputs(void) {
    HYD_PRESSUREHANDLE ph;

    __HydMotion_framework_Init();
    ensure_axes_allocated(2);
    memset(&ph, 0, sizeof(ph));

    /* 启动 */
    IEC_VAL(ph.EN) = true;
    IEC_VAL(ph.EXECUTE) = true;
    ph.EXECUTE0.value = false;
    IEC_VAL(ph.AXISID) = 0;
    IEC_VAL(ph.PRESSURE) = 10.0f;

    __mcl_cmd_PressureHandle(&ph);

    /* EN=false */
    IEC_VAL(ph.EN) = false;
    __mcl_cmd_PressureHandle(&ph);

    ASSERT_TRUE(IEC_VAL(ph.INPRESSURE) == false, "INPRESSURE should be cleared when EN=false");
    ASSERT_TRUE(IEC_VAL(ph.DONE) == false, "DONE should be cleared when EN=false");
    ASSERT_TRUE(IEC_VAL(ph.BUSY) == false, "BUSY should be cleared when EN=false");
    ASSERT_TRUE(IEC_VAL(ph.ACTIVE) == false, "ACTIVE should be cleared when EN=false");
}

/* ==================================================================
 * Test 19: PressureHandle 非法 AXISID
 * ================================================================== */
static void test_pressurehandle_rejects_invalid_axis_index(void) {
    HYD_PRESSUREHANDLE ph;

    __HydMotion_framework_Init();
    ensure_axes_allocated(2);
    memset(&ph, 0, sizeof(ph));

    IEC_VAL(ph.EN) = true;
    IEC_VAL(ph.EXECUTE) = true;
    ph.EXECUTE0.value = false;
    IEC_VAL(ph.AXISID) = -5;

    __mcl_cmd_PressureHandle(&ph);

    ASSERT_TRUE(IEC_VAL(ph.ERROR) == true, "PressureHandle should set ERROR for invalid AXISID");
}

/* ==================================================================
 * Test 19b: PressureHandle 正常完成时保持当前完成语义
 *
 * 当前接口没有 DONE 引脚，因此外部完成语义是:
 * 时间到后清除 BUSY/ACTIVE，且不报 COMMANDABORTED。
 * 这是 direct FB 包装层重构时必须保持不变的行为。
 * ================================================================== */
static void test_pressurehandle_completion_keeps_completion_semantics(void) {
    HYD_PRESSUREHANDLE ph;
    HYD_CREATEMOTION cm;

    __HydMotion_framework_Init();

    memset(&cm, 0, sizeof(cm));
    IEC_VAL(cm.EN) = true;
    IEC_VAL(cm.USE_RECIPE) = false;
    IEC_VAL(cm.FLOW_TO_PUMPSPEED) = 1.0f;
    IEC_VAL(cm.PUMPSPEED_LIMIT) = 3000.0f;
    IEC_VAL(cm.USE_SIMULATION) = true;
    __mcl_cmd_CreateMotion(&cm);

    memset(&ph, 0, sizeof(ph));
    IEC_VAL(ph.EN) = true;
    IEC_VAL(ph.EXECUTE) = true;
    ph.EXECUTE0.value = false;
    IEC_VAL(ph.AXISID) = IEC_VAL(cm.AXISID);
    IEC_VAL(ph.PRESSURE) = 10.0f;
    IEC_VAL(ph.PRESSURERAMPRATE) = 2.0f;
    IEC_VAL(ph.DURATION) = 0.05f;

    __mcl_cmd_PressureHandle(&ph);
    __HydMotion_framework_Publish();

    IEC_VAL(ph.EXECUTE) = true;
    ph.EXECUTE0.value = true;

    for (int step = 0; step < 500; step++) {
        __mcl_cmd_PressureHandle(&ph);
        if (!IEC_VAL(ph.BUSY) && !IEC_VAL(ph.ACTIVE)) {
            break;
        }
        __HydMotion_framework_Publish();
    }

    ASSERT_TRUE(IEC_VAL(ph.BUSY) == false,
               "PressureHandle should clear BUSY after timed completion");
    ASSERT_TRUE(IEC_VAL(ph.ACTIVE) == false,
               "PressureHandle should clear ACTIVE after timed completion");
    ASSERT_TRUE(IEC_VAL(ph.COMMANDABORTED) == false,
               "PressureHandle should not report COMMANDABORTED on normal completion");
    ASSERT_TRUE(IEC_VAL(ph.DONE) == true,
               "PressureHandle should set DONE on completion");
}

static void test_hold_resume_surface_transitions_active_motion(void) {
    HYD_MOVEABSOLUTE ma;
    HYD_HOLD hold;
    HYD_RESUME resume;
    HYD_MotionControlFB* fb;

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
    IEC_VAL(ma.DIRECTION) = 1;
    __mcl_cmd_MoveAbsolute(&ma);
    __HydMotion_framework_Publish();

    IEC_VAL(ma.EXECUTE) = true;
    ma.EXECUTE0.value = true;
    __mcl_cmd_MoveAbsolute(&ma);

    fb = __MK_GetPublic_MotionControlFB(0);
    ASSERT_TRUE(fb != NULL, "FB instance should exist before Hold");
    ASSERT_TRUE(fb->FB_STATE == HYD_FB_STATE_RUNNING || fb->FB_STATE == HYD_FB_STATE_STARTING,
               "MoveAbsolute should be executing before Hold");

    memset(&hold, 0, sizeof(hold));
    IEC_VAL(hold.EN) = true;
    IEC_VAL(hold.EXECUTE) = true;
    hold.EXECUTE0.value = false;
    IEC_VAL(hold.AXISID) = 0;
    __mcl_cmd_Hold(&hold);

    ASSERT_TRUE(IEC_VAL(hold.DONE) == false, "Hold should not report DONE on the trigger call");
    ASSERT_TRUE(IEC_VAL(hold.BUSY) == true, "Hold should report BUSY while pending");

    __HydMotion_framework_Publish();
    IEC_VAL(hold.EXECUTE) = true;
    hold.EXECUTE0.value = true;
    __mcl_cmd_Hold(&hold);

    ASSERT_TRUE(fb->FB_STATE == HYD_FB_STATE_HOLD, "Hold should transition runtime to HOLD");
    ASSERT_TRUE(IEC_VAL(hold.DONE) == true, "Hold should report DONE when HOLD state is reached");
    ASSERT_TRUE(IEC_VAL(hold.BUSY) == false, "Hold BUSY should clear after DONE");
    ASSERT_TRUE(IEC_VAL(hold.ERROR) == false, "Hold should not report ERROR on success");

    IEC_VAL(ma.EXECUTE) = true;
    ma.EXECUTE0.value = true;
    __mcl_cmd_MoveAbsolute(&ma);

    ASSERT_TRUE(IEC_VAL(ma.BUSY) == true, "Held MoveAbsolute owner should remain BUSY");
    ASSERT_TRUE(IEC_VAL(ma.ACTIVE) == false, "Held MoveAbsolute owner should clear ACTIVE");
    ASSERT_TRUE(IEC_VAL(ma.COMMANDABORTED) == false,
               "Held MoveAbsolute owner should not report COMMANDABORTED");

    memset(&resume, 0, sizeof(resume));
    IEC_VAL(resume.EN) = true;
    IEC_VAL(resume.EXECUTE) = true;
    resume.EXECUTE0.value = false;
    IEC_VAL(resume.AXISID) = 0;
    __mcl_cmd_Resume(&resume);

    ASSERT_TRUE(IEC_VAL(resume.DONE) == false, "Resume should not report DONE on the trigger call");
    ASSERT_TRUE(IEC_VAL(resume.BUSY) == true, "Resume should report BUSY while pending");

    __HydMotion_framework_Publish();
    IEC_VAL(resume.EXECUTE) = true;
    resume.EXECUTE0.value = true;
    __mcl_cmd_Resume(&resume);

    ASSERT_TRUE(fb->FB_STATE == HYD_FB_STATE_RUNNING || fb->FB_STATE == HYD_FB_STATE_STARTING,
               "Resume should return runtime to execution state");
    ASSERT_TRUE(IEC_VAL(resume.DONE) == true, "Resume should report DONE once HOLD is left");
    ASSERT_TRUE(IEC_VAL(resume.BUSY) == false, "Resume BUSY should clear after DONE");
    ASSERT_TRUE(IEC_VAL(resume.ERROR) == false, "Resume should not report ERROR on success");
}

static void test_hold_resume_reject_invalid_axis_index(void) {
    HYD_HOLD hold;
    HYD_RESUME resume;

    __HydMotion_framework_Init();
    ensure_axes_allocated(1);

    memset(&hold, 0, sizeof(hold));
    IEC_VAL(hold.EN) = true;
    IEC_VAL(hold.EXECUTE) = true;
    hold.EXECUTE0.value = false;
    IEC_VAL(hold.AXISID) = -1;
    __mcl_cmd_Hold(&hold);

    ASSERT_TRUE(IEC_VAL(hold.ERROR) == true, "Hold should set ERROR for invalid AXISID");
    ASSERT_TRUE(IEC_VAL(hold.ERRORID) == HYD_DIAG_CODE_START_CONTEXT_INVALID,
               "Hold ERRORID should be START_CONTEXT_INVALID");
    ASSERT_TRUE(IEC_VAL(hold.BUSY) == false, "Hold BUSY should be false after invalid AXISID");

    memset(&resume, 0, sizeof(resume));
    IEC_VAL(resume.EN) = true;
    IEC_VAL(resume.EXECUTE) = true;
    resume.EXECUTE0.value = false;
    IEC_VAL(resume.AXISID) = 2;
    __mcl_cmd_Resume(&resume);

    ASSERT_TRUE(IEC_VAL(resume.ERROR) == true, "Resume should set ERROR for unallocated AXISID");
    ASSERT_TRUE(IEC_VAL(resume.ERRORID) == HYD_DIAG_CODE_START_CONTEXT_INVALID,
               "Resume ERRORID should be START_CONTEXT_INVALID");
    ASSERT_TRUE(IEC_VAL(resume.BUSY) == false, "Resume BUSY should be false after invalid AXISID");
}

static void test_hold_resume_reject_invalid_runtime_state(void) {
    HYD_HOLD hold;
    HYD_RESUME resume;

    __HydMotion_framework_Init();
    ensure_axes_allocated(1);

    memset(&hold, 0, sizeof(hold));
    IEC_VAL(hold.EN) = true;
    IEC_VAL(hold.EXECUTE) = true;
    hold.EXECUTE0.value = false;
    IEC_VAL(hold.AXISID) = 0;
    __mcl_cmd_Hold(&hold);

    ASSERT_TRUE(IEC_VAL(hold.ERROR) == true, "Hold should reject an idle axis");
    ASSERT_TRUE(IEC_VAL(hold.ERRORID) != 0, "Hold should report non-zero ERRORID for invalid state");
    ASSERT_TRUE(IEC_VAL(hold.BUSY) == false, "Hold BUSY should stay false for invalid state");

    memset(&resume, 0, sizeof(resume));
    IEC_VAL(resume.EN) = true;
    IEC_VAL(resume.EXECUTE) = true;
    resume.EXECUTE0.value = false;
    IEC_VAL(resume.AXISID) = 0;
    __mcl_cmd_Resume(&resume);

    ASSERT_TRUE(IEC_VAL(resume.ERROR) == true, "Resume should reject a non-held axis");
    ASSERT_TRUE(IEC_VAL(resume.ERRORID) != 0, "Resume should report non-zero ERRORID for invalid state");
    ASSERT_TRUE(IEC_VAL(resume.BUSY) == false, "Resume BUSY should stay false for invalid state");
}

/* ==================================================================
 * Test 20: 多个轴独立运行 (轴0和轴1各自执行不同命令)
 * ================================================================== */
static void test_multiple_axes_operate_independently(void) {
    HYD_MOVEABSOLUTE ma0;
    HYD_MOVEVELOCITY mv1;

    __HydMotion_framework_Init();
    ensure_axes_allocated(2);
    memset(&ma0, 0, sizeof(ma0));
    memset(&mv1, 0, sizeof(mv1));

    /* 轴0: MoveAbsolute execRising */
    IEC_VAL(ma0.EN) = true;
    IEC_VAL(ma0.EXECUTE) = true;
    ma0.EXECUTE0.value = false;
    IEC_VAL(ma0.AXISID) = 0;
    IEC_VAL(ma0.POSITION) = 100.0f;
    IEC_VAL(ma0.VELOCITY) = 50.0f;
    IEC_VAL(ma0.ACCELERATION) = 200.0f;
    IEC_VAL(ma0.DIRECTION) = 1;
    __mcl_cmd_MoveAbsolute(&ma0);
    ASSERT_TRUE(IEC_VAL(ma0.BUSY) == true, "Axis 0 MoveAbsolute: BUSY should be true after execRising");

    /* 轴1: MoveVelocity execRising */
    IEC_VAL(mv1.EN) = true;
    IEC_VAL(mv1.EXECUTE) = true;
    mv1.EXECUTE0.value = false;
    IEC_VAL(mv1.AXISID) = 1;
    IEC_VAL(mv1.VELOCITY) = 30.0f;
    IEC_VAL(mv1.ACCELERATION) = 150.0f;
    IEC_VAL(mv1.DIRECTION) = 1;
    __mcl_cmd_MoveVelocity(&mv1);
    ASSERT_TRUE(IEC_VAL(mv1.BUSY) == true, "Axis 1 MoveVelocity: BUSY should be true after execRising");

    /* 处理两个轴的待处理命令 */
    __HydMotion_framework_Publish();

    /* 轴0 确认所有权 */
    IEC_VAL(ma0.EXECUTE) = true;
    ma0.EXECUTE0.value = true;
    __mcl_cmd_MoveAbsolute(&ma0);

    /* 轴1 确认所有权 */
    IEC_VAL(mv1.EXECUTE) = true;
    mv1.EXECUTE0.value = true;
    __mcl_cmd_MoveVelocity(&mv1);

    /* 两轴应各自独立，都不应为 COMMANDABORTED */
    ASSERT_TRUE(IEC_VAL(ma0.COMMANDABORTED) == false,
               "Axis 0 should not get COMMANDABORTED from independent operation");
    ASSERT_TRUE(IEC_VAL(mv1.COMMANDABORTED) == false,
               "Axis 1 should not get COMMANDABORTED from independent operation");
}

/**
 * @brief ApplyLiveUpdate 在段完成后应对同一 owner 的请求静默成功
 *
 * 修复前: 连续更新模式下段完成后，ApplyLiveUpdate 守卫条件失败会报
 * COMMAND_NOT_ALLOWED。修复后应静默返回 true 让 IEC 适配器的 DONE 逻辑触发。
 */
static void test_apply_live_update_tolerates_completed_segment(void)
{
    HYD_MotionControlFB* fb;
    HYD_LiveUpdateRequest request;
    HYD_MotionSegment segment;
    HYD_BOOL result;

    __HydMotion_framework_Init();
    ensure_axes_allocated(1);
    fb = __MK_GetPublic_MotionControlFB(0);

    /* 手动构造一个活跃的 DIRECT 段并设置 owner */
    memset(&segment, 0, sizeof(segment));
    segment.mode = HYD_MODE_POSITION;
    segment.endCondition = HYD_END_POSITION;
    segment.targetPosition = 200.0;
    segment.maxVelocity = 50.0;
    segment.maxAcceleration = 100.0;
    segment.maxDeceleration = 100.0;
    segment.maxFlow = 50.0;
    segment.velocityToFlowGain = 1.0;
    segment.direction = HYD_DIRECTION_EXTEND;

    fb->DIRECT_SEGMENT = segment;
    fb->DIRECT_SEGMENT_VALID = true;
    fb->USE_RECIPE = false;
    fb->AXIS_REF.timestamp = 1.0;

    /* 模拟 BeginSegment 后的状态 */
    fb->_activeSegment = segment;
    fb->_activeSegmentValid = true;
    fb->_activeSegmentSource = HYD_SEGMENT_SOURCE_DIRECT;
    fb->STATE.active = true;
    fb->_directOwnerKind = HYD_DIRECT_CMD_MOVE_ABSOLUTE;
    fb->_directOwnerTicket = 42;
    fb->_executionId = 42;

    /* 构造请求 */
    memset(&request, 0, sizeof(request));
    request.flags = HYD_LIVE_UPDATE_TARGET_POSITION | HYD_LIVE_UPDATE_MAX_VELOCITY;
    request.ownerKind = HYD_DIRECT_CMD_MOVE_ABSOLUTE;
    request.ownerTicket = 42;
    request.targetPosition = 250.0;
    request.maxVelocity = 60.0;

    /* 活跃段 + 正确 owner: ApplyLiveUpdate 应该成功 */
    result = HYD_MotionControlFB_ApplyLiveUpdate(fb, &request);
    ASSERT_TRUE(result == true,
               "ApplyLiveUpdate should succeed on active segment with matching owner");
    ASSERT_TRUE(fabs(fb->_activeSegment.targetPosition - 250.0) < 0.001,
               "ApplyLiveUpdate should update target position on active segment");

    /* 模拟段正常完成 (ApplyIdleState 效果) */
    fb->STATE.finished = true;
    fb->SEGMENT_COMPLETED = true;
    fb->_activeSegmentValid = false;
    fb->STATE.active = false;
    fb->_activeSegmentSource = HYD_SEGMENT_SOURCE_NONE;

    /* 同一 owner 在完成后请求更新: 应静默成功 (不报错) */
    result = HYD_MotionControlFB_ApplyLiveUpdate(fb, &request);
    ASSERT_TRUE(result == true,
               "ApplyLiveUpdate should silently succeed for same owner after completion");

    /* 不同 owner ticket 在完成后请求更新: 仍应拒绝 */
    request.ownerTicket = 999;
    result = HYD_MotionControlFB_ApplyLiveUpdate(fb, &request);
    ASSERT_TRUE(result == false,
               "ApplyLiveUpdate should reject WRONG owner even after completion");

    printf("  OK: test_apply_live_update_tolerates_completed_segment\n");
}

/* Test: Direction enum values survive round-trip through segment direction field.
 * Uses direct recipe loading (no PLCopen lifecycle) to verify the 4-direction mapping. */
static void test_direction_enum_values_round_trip(void) {
    HYD_MotionControlFB fb;
    HYD_MotionSegment seg;

    memset(&fb, 0, sizeof(fb));
    memset(&seg, 0, sizeof(seg));
    seg.mode = HYD_MODE_SPEED_RAMP;
    seg.endCondition = HYD_END_MANUAL;
    seg.direction = HYD_DIRECTION_SHORTEST_WAY;
    seg.planner = HYD_PLANNER_TIME_BASED;

    /* SHORTEST_WAY */
    ASSERT_TRUE(seg.direction == HYD_DIRECTION_SHORTEST_WAY,
                "SHORTEST_WAY enum must be 0");
    ASSERT_TRUE(seg.direction == 0, "SHORTEST_WAY value must be 0");

    /* POSITIVE */
    seg.direction = HYD_DIRECTION_POSITIVE;
    ASSERT_TRUE(seg.direction == HYD_DIRECTION_POSITIVE,
                "POSITIVE enum must be 1");
    ASSERT_TRUE(seg.direction == 1, "POSITIVE value must be 1");

    /* NEGATIVE */
    seg.direction = HYD_DIRECTION_NEGATIVE;
    ASSERT_TRUE(seg.direction == HYD_DIRECTION_NEGATIVE,
                "NEGATIVE enum must be 2");
    ASSERT_TRUE(seg.direction == 2, "NEGATIVE value must be 2");

    /* CURRENT */
    seg.direction = HYD_DIRECTION_CURRENT;
    ASSERT_TRUE(seg.direction == HYD_DIRECTION_CURRENT,
                "CURRENT enum must be 3");
    ASSERT_TRUE(seg.direction == 3, "CURRENT value must be 3");

    /* HOLD */
    seg.direction = HYD_DIRECTION_HOLD;
    ASSERT_TRUE(seg.direction == HYD_DIRECTION_HOLD,
                "HOLD enum must be 4");
    ASSERT_TRUE(seg.direction == 4, "HOLD value must be 4");

    printf("  PASS: Direction enum values (SW=0, POS=1, NEG=2, CUR=3, HOLD=4)\n");
}

/* Test: MoveAbsolute with DIRECTION=0 (SHORTEST_WAY) should be accepted and
 * correctly resolved at runtime. Verifies that recipe_validator does not
 * reject SHORTEST_WAY/CURRENT as invalid directions for position mode. */
static void test_moveabsolute_shortest_way_direction_accepted(void) {
    HYD_MOVEABSOLUTE ma;
    HYD_MotionControlFB* fb;

    __HydMotion_framework_Init();
    ensure_axes_allocated(1);
    memset(&ma, 0, sizeof(ma));

    /* 上升沿触发 MoveAbsolute with SHORTEST_WAY direction, velocity=5, accel=200, target=100 */
    IEC_VAL(ma.EN) = true;
    IEC_VAL(ma.EXECUTE) = true;
    ma.EXECUTE0.value = false;
    IEC_VAL(ma.AXISID) = 0;
    IEC_VAL(ma.POSITION) = 100.0f;
    IEC_VAL(ma.VELOCITY) = 5.0f;
    IEC_VAL(ma.ACCELERATION) = 200.0f;
    IEC_VAL(ma.DIRECTION) = 0;  /* HYD_Shortest_Way */

    __mcl_cmd_MoveAbsolute(&ma);

    ASSERT_TRUE(IEC_VAL(ma.ERROR) == false,
               "MoveAbsolute SHORTEST_WAY should NOT error");
    ASSERT_TRUE(IEC_VAL(ma.BUSY) == true,
               "MoveAbsolute SHORTEST_WAY should set BUSY=true");
    ASSERT_TRUE(IEC_VAL(ma.ACTIVE) == true,
               "MoveAbsolute SHORTEST_WAY should set ACTIVE=true");

    /* 验证底层FB状态：segment方向应为SHORTEST_WAY，运行时会被解析 */
    fb = __MK_GetPublic_MotionControlFB(0);
    ASSERT_TRUE(fb != NULL, "FB should exist after MoveAbsolute SHORTEST_WAY");
    ASSERT_TRUE(fb->DIRECT_SEGMENT.direction == HYD_DIRECTION_SHORTEST_WAY,
               "Segment direction should be SHORTEST_WAY(0)");
    ASSERT_TRUE(fb->DIRECT_SEGMENT.mode == HYD_MODE_POSITION,
               "Segment mode should be POSITION");

    printf("  PASS: MoveAbsolute SHORTEST_WAY direction accepted\n");
}

/* Test: MoveAbsolute with DIRECTION=3 (CURRENT) should also be accepted
 * by the recipe validator for position mode. */
static void test_moveabsolute_current_direction_accepted(void) {
    HYD_MOVEABSOLUTE ma;
    HYD_MotionControlFB* fb;

    __HydMotion_framework_Init();
    ensure_axes_allocated(1);
    memset(&ma, 0, sizeof(ma));

    IEC_VAL(ma.EN) = true;
    IEC_VAL(ma.EXECUTE) = true;
    ma.EXECUTE0.value = false;
    IEC_VAL(ma.AXISID) = 0;
    IEC_VAL(ma.POSITION) = 100.0f;
    IEC_VAL(ma.VELOCITY) = 5.0f;
    IEC_VAL(ma.ACCELERATION) = 200.0f;
    IEC_VAL(ma.DIRECTION) = 3;  /* HYD_Current_Direction */

    __mcl_cmd_MoveAbsolute(&ma);

    ASSERT_TRUE(IEC_VAL(ma.ERROR) == false,
               "MoveAbsolute CURRENT direction should NOT error");
    ASSERT_TRUE(IEC_VAL(ma.BUSY) == true,
               "MoveAbsolute CURRENT direction should set BUSY=true");

    fb = __MK_GetPublic_MotionControlFB(0);
    ASSERT_TRUE(fb != NULL, "FB should exist after MoveAbsolute CURRENT");
    ASSERT_TRUE(fb->DIRECT_SEGMENT.direction == HYD_DIRECTION_CURRENT,
               "Segment direction should be CURRENT(3)");

    printf("  PASS: MoveAbsolute CURRENT direction accepted\n");
}

/* ==================================================================
 * MoveAbsolute 方向参数覆盖测试
 * 验证 POSITIVE / NEGATIVE 强制方向 + velocity=5 accel=200 的
 * 匹配/不匹配场景，以及越界方向值的默认处理。
 * ================================================================== */

/* DIRECTION=1 (POSITIVE), targetPos=100 > currentPos=0 — 方向匹配，应正常启动 */
static void test_moveabsolute_positive_direction_matching_target_starts(void) {
    HYD_MOVEABSOLUTE ma;
    HYD_MotionControlFB* fb;

    __HydMotion_framework_Init();
    ensure_axes_allocated(1);
    memset(&ma, 0, sizeof(ma));

    IEC_VAL(ma.EN) = true;
    IEC_VAL(ma.EXECUTE) = true;
    ma.EXECUTE0.value = false;
    IEC_VAL(ma.AXISID) = 0;
    IEC_VAL(ma.POSITION) = 100.0f;   /* 目标在当前位置前方 */
    IEC_VAL(ma.VELOCITY) = 5.0f;
    IEC_VAL(ma.ACCELERATION) = 200.0f;
    IEC_VAL(ma.DIRECTION) = 1;  /* HYD_Positive_Direction */

    __mcl_cmd_MoveAbsolute(&ma);

    ASSERT_TRUE(IEC_VAL(ma.ERROR) == false,
               "MoveAbsolute POSITIVE with forward target should NOT error");
    ASSERT_TRUE(IEC_VAL(ma.BUSY) == true,
               "MoveAbsolute POSITIVE with forward target should set BUSY=true");
    ASSERT_TRUE(IEC_VAL(ma.ACTIVE) == true,
               "MoveAbsolute POSITIVE with forward target should set ACTIVE=true");
    ASSERT_TRUE(IEC_VAL(ma.COMMANDABORTED) == false,
               "MoveAbsolute POSITIVE with forward target should not abort");

    fb = __MK_GetPublic_MotionControlFB(0);
    ASSERT_TRUE(fb != NULL, "FB should exist after MoveAbsolute POSITIVE");
    ASSERT_TRUE(fb->DIRECT_SEGMENT_VALID == true,
               "DIRECT_SEGMENT_VALID should be true");
    ASSERT_TRUE(fb->DIRECT_SEGMENT.direction == HYD_DIRECTION_POSITIVE,
               "Segment direction should be POSITIVE(1)");
    ASSERT_TRUE(fabs(fb->DIRECT_SEGMENT.maxVelocity - 5.0f) < 0.001f,
               "Velocity should be preserved as 5.0");
    ASSERT_TRUE(fabs(fb->DIRECT_SEGMENT.maxAcceleration - 200.0f) < 0.001f,
               "Acceleration should be preserved as 200.0");

    printf("  PASS: MoveAbsolute POSITIVE direction matching target starts (vel=5, accel=200)\n");
}

/* DIRECTION=1 (POSITIVE), targetPos=-50 < currentPos=0 — 方向冲突，应报 ERROR */
static void test_moveabsolute_positive_direction_rejects_backward_target(void) {
    HYD_MOVEABSOLUTE ma;

    __HydMotion_framework_Init();
    ensure_axes_allocated(1);
    memset(&ma, 0, sizeof(ma));

    IEC_VAL(ma.EN) = true;
    IEC_VAL(ma.EXECUTE) = true;
    ma.EXECUTE0.value = false;
    IEC_VAL(ma.AXISID) = 0;
    IEC_VAL(ma.POSITION) = -50.0f;   /* 目标在当前位置后方 — 与正向冲突 */
    IEC_VAL(ma.VELOCITY) = 5.0f;
    IEC_VAL(ma.ACCELERATION) = 200.0f;
    IEC_VAL(ma.DIRECTION) = 1;  /* HYD_Positive_Direction */

    __mcl_cmd_MoveAbsolute(&ma);

    ASSERT_TRUE(IEC_VAL(ma.ERROR) == true,
               "MoveAbsolute POSITIVE with backward target should set ERROR");
    ASSERT_TRUE(IEC_VAL(ma.ERRORID) == HYD_DIAG_CODE_COMMAND_NOT_ALLOWED,
               "ERRORID should be COMMAND_NOT_ALLOWED for POSITIVE + backward target");
    ASSERT_TRUE(IEC_VAL(ma.BUSY) == false,
               "MoveAbsolute should NOT set BUSY on direction conflict");
    ASSERT_TRUE(IEC_VAL(ma.ACTIVE) == false,
               "MoveAbsolute should NOT set ACTIVE on direction conflict");

    printf("  PASS: MoveAbsolute POSITIVE direction rejects backward target\n");
}

/* DIRECTION=2 (NEGATIVE), targetPos=-100 < currentPos=0 — 方向匹配，应正常启动 */
static void test_moveabsolute_negative_direction_matching_target_starts(void) {
    HYD_MOVEABSOLUTE ma;
    HYD_MotionControlFB* fb;

    __HydMotion_framework_Init();
    ensure_axes_allocated(1);
    memset(&ma, 0, sizeof(ma));

    IEC_VAL(ma.EN) = true;
    IEC_VAL(ma.EXECUTE) = true;
    ma.EXECUTE0.value = false;
    IEC_VAL(ma.AXISID) = 0;
    IEC_VAL(ma.POSITION) = -100.0f;   /* 目标在当前位置后方 */
    IEC_VAL(ma.VELOCITY) = 5.0f;
    IEC_VAL(ma.ACCELERATION) = 200.0f;
    IEC_VAL(ma.DIRECTION) = 2;  /* HYD_Negative_Direction */

    __mcl_cmd_MoveAbsolute(&ma);

    ASSERT_TRUE(IEC_VAL(ma.ERROR) == false,
               "MoveAbsolute NEGATIVE with backward target should NOT error");
    ASSERT_TRUE(IEC_VAL(ma.BUSY) == true,
               "MoveAbsolute NEGATIVE with backward target should set BUSY=true");
    ASSERT_TRUE(IEC_VAL(ma.ACTIVE) == true,
               "MoveAbsolute NEGATIVE with backward target should set ACTIVE=true");
    ASSERT_TRUE(IEC_VAL(ma.COMMANDABORTED) == false,
               "MoveAbsolute NEGATIVE with backward target should not abort");

    fb = __MK_GetPublic_MotionControlFB(0);
    ASSERT_TRUE(fb != NULL, "FB should exist after MoveAbsolute NEGATIVE");
    ASSERT_TRUE(fb->DIRECT_SEGMENT_VALID == true,
               "DIRECT_SEGMENT_VALID should be true");
    ASSERT_TRUE(fb->DIRECT_SEGMENT.direction == HYD_DIRECTION_NEGATIVE,
               "Segment direction should be NEGATIVE(2)");
    ASSERT_TRUE(fabs(fb->DIRECT_SEGMENT.maxVelocity - 5.0f) < 0.001f,
               "Velocity should be 5.0 (abs of input)");
    ASSERT_TRUE(fabs(fb->DIRECT_SEGMENT.maxAcceleration - 200.0f) < 0.001f,
               "Acceleration should be 200.0");

    printf("  PASS: MoveAbsolute NEGATIVE direction matching target starts (vel=5, accel=200)\n");
}

/* DIRECTION=2 (NEGATIVE), targetPos=100 > currentPos=0 — 方向冲突，应报 ERROR */
static void test_moveabsolute_negative_direction_rejects_forward_target(void) {
    HYD_MOVEABSOLUTE ma;

    __HydMotion_framework_Init();
    ensure_axes_allocated(1);
    memset(&ma, 0, sizeof(ma));

    IEC_VAL(ma.EN) = true;
    IEC_VAL(ma.EXECUTE) = true;
    ma.EXECUTE0.value = false;
    IEC_VAL(ma.AXISID) = 0;
    IEC_VAL(ma.POSITION) = 100.0f;   /* 目标在当前位置前方 — 与负向冲突 */
    IEC_VAL(ma.VELOCITY) = 5.0f;
    IEC_VAL(ma.ACCELERATION) = 200.0f;
    IEC_VAL(ma.DIRECTION) = 2;  /* HYD_Negative_Direction */

    __mcl_cmd_MoveAbsolute(&ma);

    ASSERT_TRUE(IEC_VAL(ma.ERROR) == true,
               "MoveAbsolute NEGATIVE with forward target should set ERROR");
    ASSERT_TRUE(IEC_VAL(ma.ERRORID) == HYD_DIAG_CODE_COMMAND_NOT_ALLOWED,
               "ERRORID should be COMMAND_NOT_ALLOWED for NEGATIVE + forward target");
    ASSERT_TRUE(IEC_VAL(ma.BUSY) == false,
               "MoveAbsolute should NOT set BUSY on direction conflict");
    ASSERT_TRUE(IEC_VAL(ma.ACTIVE) == false,
               "MoveAbsolute should NOT set ACTIVE on direction conflict");

    printf("  PASS: MoveAbsolute NEGATIVE direction rejects forward target\n");
}

/* 越界方向值 (>3) 默认映射为 SHORTEST_WAY 并正常启动 */
static void test_moveabsolute_out_of_range_direction_defaults_to_shortest_way(void) {
    HYD_MOVEABSOLUTE ma;
    HYD_MotionControlFB* fb;

    /* 测试 DIRECTION=4, 5, 99, 255 均映射到 SHORTEST_WAY */
    int directions[] = {4, 5, 99, 255};
    int numDirs = sizeof(directions) / sizeof(directions[0]);

    for (int i = 0; i < numDirs; i++) {
        __HydMotion_framework_Init();
        ensure_axes_allocated(1);
        memset(&ma, 0, sizeof(ma));

        IEC_VAL(ma.EN) = true;
        IEC_VAL(ma.EXECUTE) = true;
        ma.EXECUTE0.value = false;
        IEC_VAL(ma.AXISID) = 0;
        IEC_VAL(ma.POSITION) = 100.0f;
        IEC_VAL(ma.VELOCITY) = 5.0f;
        IEC_VAL(ma.ACCELERATION) = 200.0f;
        IEC_VAL(ma.DIRECTION) = directions[i];

        __mcl_cmd_MoveAbsolute(&ma);

        ASSERT_TRUE(IEC_VAL(ma.ERROR) == false,
                   "MoveAbsolute out-of-range direction should NOT error");
        ASSERT_TRUE(IEC_VAL(ma.BUSY) == true,
                   "MoveAbsolute out-of-range direction should set BUSY=true");

        fb = __MK_GetPublic_MotionControlFB(0);
        ASSERT_TRUE(fb != NULL, "FB should exist after MoveAbsolute out-of-range dir");
        ASSERT_TRUE(fb->DIRECT_SEGMENT.direction == HYD_DIRECTION_SHORTEST_WAY,
                   "Out-of-range direction should default to SHORTEST_WAY(0)");
    }

    printf("  PASS: MoveAbsolute out-of-range direction values (4,5,99,255) default to SHORTEST_WAY\n");
}

/* ==================================================================
 * Test: MoveAbsolute CONTINUOUSUPDATE 反向运动到目标点 Done/Busy/Active 稳定性
 *
 * 场景:
 *   1. MoveAbsolute CONTINUOUSUPDATE=1 启动 target=100
 *   2. 仿真驱动前进到 ~100，DONE/BUSY/ACTIVE 正常
 *   3. 改变 POSITION=20（反向），CONTINUOUSUPDATE 触发 target 更新
 *   4. 仿真继续推进到 ~20
 *   5. 验证 DONE/BUSY/ACTIVE 不会反复跳变
 * ================================================================== */
static void test_moveabsolute_continuous_update_reverse_no_oscillation(void) {
    HYD_MOVEABSOLUTE ma;
    HYD_MotionControlFB* fb;
    int axisId, step, doneSteps;
    int oscCount = 0;
    HYD_BOOL prevDone = false;
    HYD_BOOL prevBusy = false;
    HYD_BOOL prevActive = false;
    HYD_REAL finalPosition;

    printf("--- Test: MoveAbsolute CONTINUOUSUPDATE reverse direction no oscillation ---\n");

    /* --- 创建仿真轴 --- */
    __HydMotion_framework_Init();
    {
        HYD_CREATEMOTION cm;
        memset(&cm, 0, sizeof(cm));
        IEC_VAL(cm.EN) = true;
        IEC_VAL(cm.USE_RECIPE) = false;
        IEC_VAL(cm.FLOW_TO_PUMPSPEED) = 1.0f;
        IEC_VAL(cm.PUMPSPEED_LIMIT) = 3000.0f;
        IEC_VAL(cm.USE_SIMULATION) = true;
        __mcl_cmd_CreateMotion(&cm);
        axisId = (int)IEC_VAL(cm.AXISID);
    }
    ASSERT_TRUE(axisId >= 0, "CreateMotion should succeed for reverse-direction test");

    fb = __MK_GetPublic_MotionControlFB(axisId);
    ASSERT_TRUE(fb != NULL, "FB should exist");

    /* --- 第一阶段: MoveAbsolute target=100 (正向), CONTINUOUSUPDATE=1 --- */
    memset(&ma, 0, sizeof(ma));
    IEC_VAL(ma.EN) = true;
    IEC_VAL(ma.AXISID) = axisId;
    IEC_VAL(ma.POSITION) = 100.0f;
    IEC_VAL(ma.VELOCITY) = 50.0f;
    IEC_VAL(ma.ACCELERATION) = 200.0f;
    IEC_VAL(ma.DECELERATION) = 200.0f;
    IEC_VAL(ma.DIRECTION) = 0;   /* SHORTEST_WAY */
    IEC_VAL(ma.CONTINUOUSUPDATE) = true;

    /* EXECUTE 上升沿 */
    IEC_VAL(ma.EXECUTE) = true;
    ma.EXECUTE0.value = false;
    __mcl_cmd_MoveAbsolute(&ma);
    ASSERT_TRUE(IEC_VAL(ma.BUSY) == true, "First EXECUTE: BUSY should be true");
    ASSERT_TRUE(IEC_VAL(ma.ACTIVE) == true, "First EXECUTE: ACTIVE should be true");
    ASSERT_TRUE(IEC_VAL(ma.DONE) == false, "First EXECUTE: DONE should be false");

    /* 第二个周期: 获取 _EXEC_ID ownership */
    __HydMotion_framework_Publish();
    ma.EXECUTE0.value = true;
    __mcl_cmd_MoveAbsolute(&ma);

    /* --- 等待伸出到 100 完成 --- */
    /* v=50, a=200 => 梯形 2.25s = 2250 steps; 使用 3000 步确保完成 */
    doneSteps = -1;
    for (step = 0; step < 3000; step++) {
        __HydMotion_framework_Publish();
        ma.EXECUTE0.value = IEC_VAL(ma.EXECUTE);
        __mcl_cmd_MoveAbsolute(&ma);

        if (IEC_VAL(ma.DONE)) {
            doneSteps = step;
            break;
        }
    }
    ASSERT_TRUE(doneSteps >= 0, "MoveAbsolute 0->100 should reach DONE");
    printf("  Forward 0->100: DONE after %d steps, position=%.3f\n",
           doneSteps, (double)fb->AXIS_REF.position);

    /* --- 第二阶段: 修改 POSITION=20 (反向), CONTINUOUSUPDATE 触发 restart --- */
    /* 注意: 不拉低 EXECUTE, 不创建新 FB, 仅更改 POSITION 输入 */
    IEC_VAL(ma.POSITION) = 20.0f;

    /* 第一周期: POSITION=20 被 applyMoveAbsoluteLiveUpdate 看到 */
    __HydMotion_framework_Publish();
    ma.EXECUTE0.value = IEC_VAL(ma.EXECUTE);
    __mcl_cmd_MoveAbsolute(&ma);

    /* 此时 CONTINUOUSUPDATE 应触发 restart, FB_STATE 从 DONE -> STARTING */
    printf("  After reverse target=20: BUSY=%d ACTIVE=%d DONE=%d\n",
           IEC_VAL(ma.BUSY) ? 1 : 0,
           IEC_VAL(ma.ACTIVE) ? 1 : 0,
           IEC_VAL(ma.DONE) ? 1 : 0);

    /* --- 等待缩回到 20 完成, 同时监控状态是否跳变 --- */
    doneSteps = -1;
    oscCount = 0;
    for (step = 0; step < 6000; step++) {
        __HydMotion_framework_Publish();
        ma.EXECUTE0.value = IEC_VAL(ma.EXECUTE);
        __mcl_cmd_MoveAbsolute(&ma);

        /* 检测状态跳变: 记录从 BUSY->!BUSY 或 DONE->!DONE 的次数 */
        if (step > 0) {
            HYD_BOOL curDone = IEC_VAL(ma.DONE);
            HYD_BOOL curBusy = IEC_VAL(ma.BUSY);
            HYD_BOOL curActive = IEC_VAL(ma.ACTIVE);

            if (curDone != prevDone) oscCount++;
            if (curBusy != prevBusy) oscCount++;
            if (curActive != prevActive) oscCount++;

            prevDone = curDone;
            prevBusy = curBusy;
            prevActive = curActive;
        } else {
            prevDone = IEC_VAL(ma.DONE);
            prevBusy = IEC_VAL(ma.BUSY);
            prevActive = IEC_VAL(ma.ACTIVE);
        }

        if (IEC_VAL(ma.DONE) && doneSteps < 0) {
            doneSteps = step;
            /* 注意: 到达 DONE 后仍需多运行几步检查稳定性 */
        }
    }
    ASSERT_TRUE(doneSteps >= 0, "MoveAbsolute 100->20 reverse should reach DONE");

    finalPosition = fb->AXIS_REF.position;
    printf("  Reverse 100->20: DONE first at step %d of 6000, final position=%.3f\n",
           doneSteps, (double)finalPosition);

    /* 验证位置到达目标 */
    ASSERT_TRUE(fabs((double)finalPosition - 20.0) < 1.0,
               "Position should be near 20 after completion");

    /* 正常状态机应有: BUSY(上升沿)→BUSY(执行中)→过渡到DONE(完成)
     * 期望的状态变化: 上升沿触发 1 次 + PENDING→激活 1 次 + DONE 1 次 = 约 3-4 次
     * 如果 oscCount 过大, 说明存在反复跳变 */
    printf("  State signal transitions (reverse phase): %d\n", oscCount);
    ASSERT_TRUE(oscCount < 12,
               "Done/Busy/Active should NOT oscillate repeatedly during reverse move");

    printf("  PASS: MoveAbsolute CONTINUOUSUPDATE reverse direction no oscillation\n");
}

/* Test: Position feedback drift after DONE should NOT trigger restart.
 *
 * Simulates the exact oscillation scenario: after a MoveAbsolute completes
 * to target 20, we artificially shift AXIS_REF.position outside the position
 * tolerance.  With CONTINUOUSUPDATE=1 and POSITION=20 unchanged, the FB
 * should stay DONE (no restart), because the target INPUT hasn't changed.
 */
static void test_moveabsolute_position_drift_no_restart(void) {
    HYD_MOVEABSOLUTE ma;
    HYD_MotionControlFB* fb;
    int axisId, step, doneSteps;

    printf("--- Test: MoveAbsolute position drift after DONE should not restart ---\n");

    __HydMotion_framework_Init();
    {
        HYD_CREATEMOTION cm;
        memset(&cm, 0, sizeof(cm));
        IEC_VAL(cm.EN) = true;
        IEC_VAL(cm.USE_RECIPE) = false;
        IEC_VAL(cm.FLOW_TO_PUMPSPEED) = 1.0f;
        IEC_VAL(cm.PUMPSPEED_LIMIT) = 3000.0f;
        IEC_VAL(cm.USE_SIMULATION) = true;
        __mcl_cmd_CreateMotion(&cm);
        axisId = (int)IEC_VAL(cm.AXISID);
    }
    ASSERT_TRUE(axisId >= 0, "CreateMotion should succeed for drift test");

    fb = __MK_GetPublic_MotionControlFB(axisId);
    ASSERT_TRUE(fb != NULL, "FB should exist");

    /* Start MoveAbsolute to target=20 */
    memset(&ma, 0, sizeof(ma));
    IEC_VAL(ma.EN) = true;
    IEC_VAL(ma.AXISID) = axisId;
    IEC_VAL(ma.POSITION) = 20.0f;
    IEC_VAL(ma.VELOCITY) = 50.0f;
    IEC_VAL(ma.ACCELERATION) = 200.0f;
    IEC_VAL(ma.DECELERATION) = 200.0f;
    IEC_VAL(ma.DIRECTION) = 0;   /* SHORTEST_WAY */
    IEC_VAL(ma.CONTINUOUSUPDATE) = true;

    IEC_VAL(ma.EXECUTE) = true;
    ma.EXECUTE0.value = false;
    __mcl_cmd_MoveAbsolute(&ma);

    __HydMotion_framework_Publish();
    ma.EXECUTE0.value = true;
    __mcl_cmd_MoveAbsolute(&ma);

    /* Wait for DONE */
    doneSteps = -1;
    for (step = 0; step < 3000; step++) {
        __HydMotion_framework_Publish();
        ma.EXECUTE0.value = IEC_VAL(ma.EXECUTE);
        __mcl_cmd_MoveAbsolute(&ma);
        if (IEC_VAL(ma.DONE)) {
            doneSteps = step;
            break;
        }
    }
    ASSERT_TRUE(doneSteps >= 0, "MoveAbsolute to 20 should reach DONE");
    printf("  MoveAbsolute to 20: DONE after %d steps, position=%.4f\n",
           doneSteps, (double)fb->AXIS_REF.position);

    /* Record DONE state */
    ASSERT_TRUE(IEC_VAL(ma.DONE) == true, "DONE should be true after completion");
    ASSERT_TRUE(IEC_VAL(ma.BUSY) == false, "BUSY should be false after completion");

    /* Simulate position feedback drift: shift position 0.5mm outside tolerance.
     * This mimics real-world axis drift after pump stops. */
    fb->AXIS_REF.position = 20.5f;  /* 0.5mm away from target, outside 0.1 tolerance */

    /* Run several cycles with POSITION unchanged (still 20) and CONTINUOUSUPDATE=1 */
    for (step = 0; step < 50; step++) {
        __HydMotion_framework_Publish();
        ma.EXECUTE0.value = IEC_VAL(ma.EXECUTE);
        __mcl_cmd_MoveAbsolute(&ma);

        /* DONE should remain true despite position drift — the target INPUT
         * hasn't changed, so no restart should be triggered */
        if (!IEC_VAL(ma.DONE)) {
            printf("  FAIL at step %d: DONE went false! BUSY=%d ACTIVE=%d pos=%.3f\n",
                   step, IEC_VAL(ma.BUSY)?1:0, IEC_VAL(ma.ACTIVE)?1:0,
                   (double)fb->AXIS_REF.position);
            ASSERT_TRUE(false,
                       "DONE should remain true when position drifts but POSITION input unchanged");
            return;
        }
    }

    printf("  Position drift (0.5mm outside tol): DONE stayed true for 50 cycles\n");

    /* Now change POSITION input to 30 — this SHOULD trigger a restart */
    IEC_VAL(ma.POSITION) = 30.0f;
    __HydMotion_framework_Publish();
    ma.EXECUTE0.value = IEC_VAL(ma.EXECUTE);
    __mcl_cmd_MoveAbsolute(&ma);

    printf("  After POSITION changed to 30: BUSY=%d ACTIVE=%d DONE=%d\n",
           IEC_VAL(ma.BUSY)?1:0, IEC_VAL(ma.ACTIVE)?1:0, IEC_VAL(ma.DONE)?1:0);
    ASSERT_TRUE(IEC_VAL(ma.BUSY) == true,
               "BUSY should become true when POSITION input changes to new target");
    ASSERT_TRUE(IEC_VAL(ma.DONE) == false,
               "DONE should become false when POSITION input changes to new target");

    printf("  PASS: MoveAbsolute position drift after DONE does not restart\n");
}

/* Test: LiveUpdate request carries CONTINUOUS_UPDATE and DIRECTION flags
 * and successfully updates direction on an active POSITION segment. */
static void test_live_update_request_carries_flags_and_direction(void) {
    HYD_MotionControlFB fb;
    HYD_LiveUpdateRequest req;
    HYD_MotionSegment segment;
    HYD_BOOL result;

    printf("--- Test: LiveUpdate request carries CONTINUOUS_UPDATE and DIRECTION ---\n");

    memset(&fb, 0, sizeof(fb));
    fb.USE_RECIPE = false;
    fb.FB_STATE = HYD_FB_STATE_IDLE;

    /* Set up an active POSITION segment with POSITIVE direction */
    memset(&segment, 0, sizeof(segment));
    segment.mode = HYD_MODE_POSITION;
    segment.endCondition = HYD_END_POSITION;
    segment.targetPosition = 200.0;
    segment.maxVelocity = 50.0;
    segment.maxAcceleration = 100.0;
    segment.maxDeceleration = 100.0;
    segment.maxFlow = 50.0;
    segment.velocityToFlowGain = 1.0;
    segment.direction = HYD_DIRECTION_POSITIVE;

    fb._activeSegment = segment;
    fb._activeSegmentValid = true;
    fb._activeSegmentSource = HYD_SEGMENT_SOURCE_DIRECT;
    fb.STATE.active = true;
    fb._directOwnerKind = HYD_DIRECT_CMD_MOVE_ABSOLUTE;
    fb._directOwnerTicket = 42;
    fb._executionId = 42;
    fb.AXIS_REF.timestamp = 1.0;
    fb.AXIS_REF.position = 100.0;
    fb.DIRECT_SEGMENT = segment;
    fb.DIRECT_SEGMENT_VALID = true;

    /* Apply live update with CONTINUOUS_UPDATE + DIRECTION flip to NEGATIVE */
    memset(&req, 0, sizeof(req));
    req.flags = HYD_LIVE_UPDATE_TARGET_POSITION |
                HYD_LIVE_UPDATE_MAX_VELOCITY |
                HYD_LIVE_UPDATE_CONTINUOUS_UPDATE |
                HYD_LIVE_UPDATE_DIRECTION;
    req.ownerKind = HYD_DIRECT_CMD_MOVE_ABSOLUTE;
    req.ownerTicket = 42;
    req.targetPosition = 50.0;  /* Valid: target < current for NEGATIVE */
    req.maxVelocity = 30.0;
    req.direction = HYD_DIRECTION_NEGATIVE;

    result = HYD_MotionControlFB_ApplyLiveUpdate(&fb, &req);
    ASSERT_TRUE(result, "LiveUpdate with CONTINUOUS_UPDATE + DIRECTION should succeed on active segment");
    ASSERT_TRUE(fb._activeSegment.direction == HYD_DIRECTION_NEGATIVE,
        "Direction should be updated to NEGATIVE");
    ASSERT_TRUE(fabs(fb._activeSegment.targetPosition - 50.0) < 0.001,
        "targetPosition should be updated");

    printf("  PASS: LiveUpdate request carries CONTINUOUS_UPDATE and DIRECTION\n");
}

/* Test: CONTINUOUS_UPDATE flag suppresses diagnostic output in Case 3
 * (unauthorized owner / no active segment). Without CONTINUOUS_UPDATE,
 * Case 3 would write a COMMAND_NOT_ALLOWED diagnostic. */
static void test_live_update_continuous_suppresses_diagnostic(void) {
    HYD_MotionControlFB fb;

    printf("--- Test: CONTINUOUS_UPDATE suppresses diagnostic in Case 3 ---\n");

    memset(&fb, 0, sizeof(fb));
    HYD_MotionControlFB_Init(&fb);
    fb.FB_STATE = HYD_FB_STATE_IDLE;

    /* No active segment, no ownership — this is Case 3 */
    HYD_LiveUpdateRequest req;
    memset(&req, 0, sizeof(req));
    req.flags = HYD_LIVE_UPDATE_TARGET_POSITION |
                HYD_LIVE_UPDATE_CONTINUOUS_UPDATE;
    req.ownerKind = HYD_DIRECT_CMD_MOVE_ABSOLUTE;
    req.ownerTicket = 99;  /* Mismatch with fb._directOwnerTicket / active owner contract */
    req.targetPosition = 100.0;

    /* Save pre-call diagnostic state */
    HYD_DiagnosticCode preCode = fb.DIAGNOSTIC.code;

    HYD_BOOL result = HYD_MotionControlFB_ApplyLiveUpdate(&fb, &req);
    ASSERT_TRUE(!result, "Should return false in Case 3");

    /* CONTINUOUS_UPDATE should suppress diagnostic — code unchanged */
    ASSERT_TRUE(fb.DIAGNOSTIC.code == preCode,
        "Diagnostic should NOT be written when CONTINUOUS_UPDATE is set");

    printf("  PASS: CONTINUOUS_UPDATE suppresses diagnostic in Case 3\n");
}

/* Test: MoveVelocity (SPEED_RAMP mode) direction flip succeeds.
 * Verifies that flipping from POSITIVE to NEGATIVE is accepted
 * on a SPEED_RAMP segment. */
static void test_movevelocity_live_update_direction_flip(void) {
    HYD_MotionControlFB fb;
    HYD_MotionSegment segment;
    HYD_LiveUpdateRequest req;

    printf("--- Test: MoveVelocity LiveUpdate direction flip ---\n");

    memset(&fb, 0, sizeof(fb));
    fb.USE_RECIPE = false;
    fb.FB_STATE = HYD_FB_STATE_IDLE;

    /* Configure cylinder with different EXTEND/RETRACT areas */
    fb.cylinderConfig.areaExtendMm2 = 10000.0;
    fb.cylinderConfig.areaRetractMm2 = 6000.0;

    /* Set up an active SPEED_RAMP segment with POSITIVE direction */
    memset(&segment, 0, sizeof(segment));
    segment.mode = HYD_MODE_SPEED_RAMP;
    segment.endCondition = HYD_END_MANUAL;
    segment.planner = HYD_PLANNER_TIME_BASED;
    segment.maxVelocity = 20.0;
    segment.maxAcceleration = 50.0;
    segment.maxDeceleration = 50.0;
    segment.maxFlow = 50.0;
    segment.velocityToFlowGain = 1.0;
    segment.direction = HYD_DIRECTION_POSITIVE;

    fb._activeSegment = segment;
    fb._activeSegmentValid = true;
    fb._activeSegmentSource = HYD_SEGMENT_SOURCE_DIRECT;
    fb.STATE.active = true;
    fb._directOwnerKind = HYD_DIRECT_CMD_MOVE_VELOCITY;
    fb._directOwnerTicket = 10;
    fb._executionId = 10;
    fb.AXIS_REF.timestamp = 1.0;
    fb.AXIS_REF.position = 0.0;
    fb.AXIS_REF.flow = 0.0;
    fb.AXIS_REF.pressure = 0.0;
    fb.DIRECT_SEGMENT = segment;
    fb.DIRECT_SEGMENT_VALID = true;

    /* Flip direction to RETRACT via live update.
     * Since velocityToFlowGain is > 0, the recalculation condition
     * (gain <= 0) won't fire, but the direction should still flip
     * and planner should be re-primed. */
    memset(&req, 0, sizeof(req));
    req.flags = HYD_LIVE_UPDATE_MAX_VELOCITY |
                HYD_LIVE_UPDATE_CONTINUOUS_UPDATE |
                HYD_LIVE_UPDATE_DIRECTION;
    req.ownerKind = HYD_DIRECT_CMD_MOVE_VELOCITY;
    req.ownerTicket = 10;
    req.maxVelocity = 20.0;
    req.direction = HYD_DIRECTION_NEGATIVE;

    ASSERT_TRUE(HYD_MotionControlFB_ApplyLiveUpdate(&fb, &req),
        "Direction flip on SPEED_RAMP should succeed");

    ASSERT_TRUE(fb._activeSegment.direction == HYD_DIRECTION_NEGATIVE,
        "Direction should be NEGATIVE after flip");

    /* velocityToFlowGain may or may not change depending on whether
     * the recalculation condition fires (gain <= 0). The key assertion
     * is that direction was flipped and the request was accepted. */

    printf("  PASS: MoveVelocity LiveUpdate direction flip\n");
}

/* Test: MoveAbsolute (POSITION mode) direction flip is rejected when
 * target is inconsistent with the requested direction.
 * NEGATIVE direction requires target <= current, but target 200 > current 100. */
static void test_moveabsolute_live_update_direction_rejected(void) {
    HYD_MotionControlFB fb;
    HYD_MotionSegment segment;
    HYD_LiveUpdateRequest req;

    printf("--- Test: MoveAbsolute LiveUpdate direction rejected on consistency ---\n");

    memset(&fb, 0, sizeof(fb));
    fb.USE_RECIPE = false;
    fb.FB_STATE = HYD_FB_STATE_IDLE;

    /* Set up an active POSITION segment at position 100, moving forward to 200 */
    memset(&segment, 0, sizeof(segment));
    segment.mode = HYD_MODE_POSITION;
    segment.endCondition = HYD_END_POSITION;
    segment.targetPosition = 200.0;
    segment.maxVelocity = 20.0;
    segment.maxAcceleration = 50.0;
    segment.maxDeceleration = 50.0;
    segment.maxFlow = 50.0;
    segment.velocityToFlowGain = 1.0;
    segment.direction = HYD_DIRECTION_POSITIVE;
    segment.positionTolerance = 0.5;

    fb._activeSegment = segment;
    fb._activeSegmentValid = true;
    fb._activeSegmentSource = HYD_SEGMENT_SOURCE_DIRECT;
    fb.STATE.active = true;
    fb._directOwnerKind = HYD_DIRECT_CMD_MOVE_ABSOLUTE;
    fb._directOwnerTicket = 42;
    fb._executionId = 42;
    fb.AXIS_REF.timestamp = 1.0;
    fb.AXIS_REF.position = 100.0;
    fb.DIRECT_SEGMENT = segment;
    fb.DIRECT_SEGMENT_VALID = true;

    /* Try to flip to NEGATIVE direction — but target 200 is ahead of current 100.
     * NEGATIVE requires target <= current, so this should be rejected. */
    memset(&req, 0, sizeof(req));
    req.flags = HYD_LIVE_UPDATE_TARGET_POSITION |
                HYD_LIVE_UPDATE_CONTINUOUS_UPDATE |
                HYD_LIVE_UPDATE_DIRECTION;
    req.ownerKind = HYD_DIRECT_CMD_MOVE_ABSOLUTE;
    req.ownerTicket = 42;
    req.targetPosition = 200.0;
    req.direction = HYD_DIRECTION_NEGATIVE;

    HYD_BOOL result = HYD_MotionControlFB_ApplyLiveUpdate(&fb, &req);
    ASSERT_TRUE(!result,
        "Direction flip NEGATIVE with target > current should be rejected");

    ASSERT_TRUE(fb._activeSegment.direction == HYD_DIRECTION_POSITIVE,
        "Direction should remain POSITIVE after rejected update");

    printf("  PASS: MoveAbsolute LiveUpdate direction consistency rejection\n");
}

/* Test: PressureHandle (PRESSURE_CLOSED_LOOP mode) rejects DIRECTION update.
 * DIRECTION is only meaningful for POSITION and SPEED_RAMP modes. */
static void test_pressurehandle_live_update_direction_rejected(void) {
    HYD_MotionControlFB fb;
    HYD_MotionSegment segment;
    HYD_LiveUpdateRequest req;

    printf("--- Test: PressureHandle LiveUpdate DIRECTION rejected ---\n");

    memset(&fb, 0, sizeof(fb));
    fb.USE_RECIPE = false;
    fb.FB_STATE = HYD_FB_STATE_IDLE;

    /* Set up an active PRESSURE_CLOSED_LOOP segment */
    memset(&segment, 0, sizeof(segment));
    segment.mode = HYD_MODE_PRESSURE_CLOSED_LOOP;
    segment.endCondition = HYD_END_PRESSURE;
    segment.targetPressure = 50.0;
    segment.pressureRampRate = 10.0;
    segment.maxVelocity = 20.0;
    segment.maxAcceleration = 50.0;
    segment.maxDeceleration = 50.0;
    segment.maxFlow = 50.0;
    segment.velocityToFlowGain = 1.0;

    fb._activeSegment = segment;
    fb._activeSegmentValid = true;
    fb._activeSegmentSource = HYD_SEGMENT_SOURCE_DIRECT;
    fb.STATE.active = true;
    fb._directOwnerKind = HYD_DIRECT_CMD_PRESSURE_HANDLE;
    fb._directOwnerTicket = 77;
    fb._executionId = 77;
    fb.AXIS_REF.timestamp = 1.0;
    fb.AXIS_REF.position = 0.0;
    fb.DIRECT_SEGMENT = segment;
    fb.DIRECT_SEGMENT_VALID = true;

    /* Try DIRECTION update — should be rejected by HYD_ApplyLiveUpdateOverrides */
    memset(&req, 0, sizeof(req));
    req.flags = HYD_LIVE_UPDATE_TARGET_PRESSURE |
                HYD_LIVE_UPDATE_DIRECTION;
    req.ownerKind = HYD_DIRECT_CMD_PRESSURE_HANDLE;
    req.ownerTicket = 77;
    req.targetPressure = 60.0;
    req.direction = HYD_DIRECTION_POSITIVE;

    HYD_BOOL result = HYD_MotionControlFB_ApplyLiveUpdate(&fb, &req);
    ASSERT_TRUE(!result,
        "DIRECTION update on PRESSURE_CLOSED_LOOP should be rejected");

    printf("  PASS: PressureHandle LiveUpdate DIRECTION rejected\n");
}

/* Test: MoveVelocity live update negative VELOCITY flips direction.
 * When CONTINUOUSUPDATE=1 and DIRECTION=SHORTEST_WAY, setting VELOCITY
 * to a negative value should flip direction to NEGATIVE (not error). */
static void test_movevelocity_live_update_negative_velocity_flips_direction(void)
{
    HYD_MotionControlFB* fb;
    HYD_MOVEVELOCITY mv;

    printf("--- Test: MoveVelocity live update negative VELOCITY flips direction ---\n");

    __HydMotion_framework_Init();
    ensure_axes_allocated(1);

    /* Phase 1: Start MoveVelocity with VELOCITY=5, SHORTEST_WAY */
    memset(&mv, 0, sizeof(mv));
    IEC_VAL(mv.EN) = true;
    IEC_VAL(mv.EXECUTE) = true;
    mv.EXECUTE0.value = false;
    IEC_VAL(mv.AXISID) = 0;
    IEC_VAL(mv.CONTINUOUSUPDATE) = true;
    IEC_VAL(mv.DIRECTION) = 0;  /* SHORTEST_WAY */
    IEC_VAL(mv.VELOCITY) = 5.0;
    IEC_VAL(mv.ACCELERATION) = 50.0;
    IEC_VAL(mv.DECELERATION) = 50.0;
    IEC_VAL(mv.BUFFERMODE) = (IEC_INT)HYD_BUFFER_MODE_ABORT;
    __mcl_cmd_MoveVelocity(&mv);
    ASSERT_TRUE(IEC_VAL(mv.BUSY), "MoveVelocity should be BUSY after execute");
    ASSERT_TRUE(IEC_VAL(mv.ERROR) == false,
        "MoveVelocity should NOT produce ERROR on execRising");

    /* Simulate 1 scan cycle to latch ownership */
    __HydMotion_framework_Publish();
    mv.EXECUTE0.value = true;
    __mcl_cmd_MoveVelocity(&mv);

    fb = __MK_GetPublic_MotionControlFB(0);
    ASSERT_TRUE(fb != NULL, "Allocated FB should exist");
    ASSERT_TRUE(fb->_activeSegmentValid, "MoveVelocity should have an active segment");

    /* Verify initial state: direction POSITIVE, maxVelocity=5 */
    ASSERT_TRUE(fb->_activeSegment.direction == HYD_DIRECTION_POSITIVE,
        "Initial direction should be POSITIVE (derived from +VELOCITY)");
    ASSERT_TRUE(fb->_activeSegment.maxVelocity == 5.0,
        "Initial maxVelocity should be 5.0");

    /* Phase 2: Live update VELOCITY=-5, still SHORTEST_WAY.
     * Expected: direction flips to NEGATIVE, maxVelocity=fabs(-5)=5.0 */
    IEC_VAL(mv.VELOCITY) = -5.0;
    __mcl_cmd_MoveVelocity(&mv);

    ASSERT_TRUE(IEC_VAL(mv.ERROR) == false,
        "Live update with negative VELOCITY should NOT produce ERROR");
    ASSERT_TRUE(fb->_activeSegment.direction == HYD_DIRECTION_NEGATIVE,
        "Direction should flip to NEGATIVE after negative VELOCITY update");
    ASSERT_TRUE(fb->_activeSegment.maxVelocity == 5.0,
        "maxVelocity should be 5.0 (fabs of -5) after negative update");

    printf("  PASS: MoveVelocity live update negative VELOCITY flips direction\n");
}

/* Test: MoveVelocity live update VELOCITY=0 decelerates to stop.
 * Setting VELOCITY=0 should set maxVelocity=0 and keep the last
 * active direction without triggering an ERROR. */
static void test_movevelocity_live_update_zero_velocity_decel_to_stop(void)
{
    HYD_MotionControlFB* fb;
    HYD_MOVEVELOCITY mv;

    printf("--- Test: MoveVelocity live update VELOCITY=0 decelerates to stop ---\n");

    __HydMotion_framework_Init();
    ensure_axes_allocated(1);

    /* Phase 1: Start MoveVelocity with VELOCITY=5, SHORTEST_WAY */
    memset(&mv, 0, sizeof(mv));
    IEC_VAL(mv.EN) = true;
    IEC_VAL(mv.EXECUTE) = true;
    mv.EXECUTE0.value = false;
    IEC_VAL(mv.AXISID) = 0;
    IEC_VAL(mv.CONTINUOUSUPDATE) = true;
    IEC_VAL(mv.DIRECTION) = 0;  /* SHORTEST_WAY */
    IEC_VAL(mv.VELOCITY) = 5.0;
    IEC_VAL(mv.ACCELERATION) = 50.0;
    IEC_VAL(mv.DECELERATION) = 50.0;
    IEC_VAL(mv.BUFFERMODE) = (IEC_INT)HYD_BUFFER_MODE_ABORT;
    __mcl_cmd_MoveVelocity(&mv);
    ASSERT_TRUE(IEC_VAL(mv.BUSY), "MoveVelocity should be BUSY after execute");
    ASSERT_TRUE(IEC_VAL(mv.ERROR) == false,
        "MoveVelocity should NOT produce ERROR on execRising");

    /* Simulate 1 scan cycle to latch ownership */
    __HydMotion_framework_Publish();
    mv.EXECUTE0.value = true;
    __mcl_cmd_MoveVelocity(&mv);

    fb = __MK_GetPublic_MotionControlFB(0);
    ASSERT_TRUE(fb != NULL, "Allocated FB should exist");
    ASSERT_TRUE(fb->_activeSegmentValid, "MoveVelocity should have an active segment");

    /* Phase 2: Live update VELOCITY=0.
     * Expected: no ERROR, maxVelocity=0, direction stays POSITIVE.
     * The planner will return 0.0 velocity for maxVelocity<=0. */
    IEC_VAL(mv.VELOCITY) = 0.0;
    __mcl_cmd_MoveVelocity(&mv);

    ASSERT_TRUE(IEC_VAL(mv.ERROR) == false,
        "Live update with VELOCITY=0 should NOT produce ERROR");
    ASSERT_TRUE(fb->_activeSegment.maxVelocity == 0.0,
        "maxVelocity should be 0.0 after zero VELOCITY update");
    ASSERT_TRUE(fb->_activeSegment.direction == HYD_DIRECTION_POSITIVE,
        "Direction should stay POSITIVE (lastActiveDirection for zero VELOCITY)");

    printf("  PASS: MoveVelocity live update VELOCITY=0 decelerates to stop\n");
}

/* Test: ValidateSegment accepts maxVelocity=0 for SPEED_RAMP mode.
 * VELOCITY=0 is a valid stop request and should not be rejected. */
/* Test: CONTINUOUSUPDATE + forced Direction rejects backward target after
 * segment completion.  Mirrors the isSegmentActive direction-consistency
 * check (Case 1) for the isSegmentCompleted path (Case 2).
 *
 * Scenario:
 *   Direction = POSITIVE, axis completed at position 100.
 *   User changes target to 20 via CONTINUOUSUPDATE.
 *   Target 20 at position 100 with forced POSITIVE is impossible —
 *   must set ERROR/ERRORID, NOT silently restart. */
static void test_moveabsolute_completed_segment_direction_conflict_positive(void)
{
    HYD_MotionControlFB fb;
    HYD_MotionSegment segment;
    HYD_LiveUpdateRequest req;
    HYD_BOOL result;

    printf("--- Test: Completed-segment direction conflict POSITIVE target < current ---\n");

    memset(&fb, 0, sizeof(fb));
    fb.USE_RECIPE = false;
    fb.FB_STATE = HYD_FB_STATE_IDLE;

    /* Construct a completed direct segment at position 100 */
    memset(&segment, 0, sizeof(segment));
    segment.mode = HYD_MODE_POSITION;
    segment.endCondition = HYD_END_POSITION;
    segment.targetPosition = 100.0;
    segment.maxVelocity = 20.0;
    segment.maxAcceleration = 50.0;
    segment.maxDeceleration = 50.0;
    segment.maxFlow = 50.0;
    segment.velocityToFlowGain = 1.0;
    segment.direction = HYD_DIRECTION_POSITIVE;
    segment.positionTolerance = 0.5;

    fb.DIRECT_SEGMENT = segment;
    fb.DIRECT_SEGMENT_VALID = true;
    fb.AXIS_REF.timestamp = 1.0;
    fb.AXIS_REF.position = 100.0;
    fb._directOwnerKind = HYD_DIRECT_CMD_MOVE_ABSOLUTE;
    fb._directOwnerTicket = 42;
    fb._executionId = 42;

    /* Simulate a completed direct session */
    fb.STATE.finished = true;
    fb._activeSegmentValid = false;
    fb.STATE.active = false;
    fb._activeSegmentSource = HYD_SEGMENT_SOURCE_NONE;
    fb._previousSegmentMode = HYD_MODE_POSITION;
    fb._activeSegment = segment; /* stale but needed for mode carry-over */

    /* CONTINUOUSUPDATE with Direction=POSITIVE, target=20 (behind current=100) */
    memset(&req, 0, sizeof(req));
    req.flags = HYD_LIVE_UPDATE_TARGET_POSITION |
                HYD_LIVE_UPDATE_DIRECTION |
                HYD_LIVE_UPDATE_CONTINUOUS_UPDATE;
    req.ownerKind = HYD_DIRECT_CMD_MOVE_ABSOLUTE;
    req.ownerTicket = 42;
    req.targetPosition = 20.0;
    req.direction = HYD_DIRECTION_POSITIVE;

    result = HYD_MotionControlFB_ApplyLiveUpdate(&fb, &req);
    ASSERT_TRUE(result == false,
        "POSITIVE direction with target < current should be REJECTED after completion");
    ASSERT_TRUE(fb.ERROR_ID == HYD_DIAG_CODE_PUMP_DIRECTION_CONFLICT,
        "ERROR_ID should be PUMP_DIRECTION_CONFLICT for POSITIVE + backward target");

    printf("  PASS: Completed-segment direction conflict POSITIVE target < current\n");
}

/* Test: CONTINUOUSUPDATE + forced NEGATIVE Direction rejects forward target
 * after segment completion. */
static void test_moveabsolute_completed_segment_direction_conflict_negative(void)
{
    HYD_MotionControlFB fb;
    HYD_MotionSegment segment;
    HYD_LiveUpdateRequest req;
    HYD_BOOL result;

    printf("--- Test: Completed-segment direction conflict NEGATIVE target > current ---\n");

    memset(&fb, 0, sizeof(fb));
    fb.USE_RECIPE = false;
    fb.FB_STATE = HYD_FB_STATE_IDLE;

    /* Construct a completed direct segment at position 50 */
    memset(&segment, 0, sizeof(segment));
    segment.mode = HYD_MODE_POSITION;
    segment.endCondition = HYD_END_POSITION;
    segment.targetPosition = 50.0;
    segment.maxVelocity = 20.0;
    segment.maxAcceleration = 50.0;
    segment.maxDeceleration = 50.0;
    segment.maxFlow = 50.0;
    segment.velocityToFlowGain = 1.0;
    segment.direction = HYD_DIRECTION_NEGATIVE;
    segment.positionTolerance = 0.5;

    fb.DIRECT_SEGMENT = segment;
    fb.DIRECT_SEGMENT_VALID = true;
    fb.AXIS_REF.timestamp = 1.0;
    fb.AXIS_REF.position = 50.0;
    fb._directOwnerKind = HYD_DIRECT_CMD_MOVE_ABSOLUTE;
    fb._directOwnerTicket = 42;
    fb._executionId = 42;

    /* Simulate a completed direct session */
    fb.STATE.finished = true;
    fb._activeSegmentValid = false;
    fb.STATE.active = false;
    fb._activeSegmentSource = HYD_SEGMENT_SOURCE_NONE;
    fb._previousSegmentMode = HYD_MODE_POSITION;
    fb._activeSegment = segment;

    /* CONTINUOUSUPDATE with Direction=NEGATIVE, target=200 (ahead of current=50) */
    memset(&req, 0, sizeof(req));
    req.flags = HYD_LIVE_UPDATE_TARGET_POSITION |
                HYD_LIVE_UPDATE_DIRECTION |
                HYD_LIVE_UPDATE_CONTINUOUS_UPDATE;
    req.ownerKind = HYD_DIRECT_CMD_MOVE_ABSOLUTE;
    req.ownerTicket = 42;
    req.targetPosition = 200.0;
    req.direction = HYD_DIRECTION_NEGATIVE;

    result = HYD_MotionControlFB_ApplyLiveUpdate(&fb, &req);
    ASSERT_TRUE(result == false,
        "NEGATIVE direction with target > current should be REJECTED after completion");
    ASSERT_TRUE(fb.ERROR_ID == HYD_DIAG_CODE_PUMP_DIRECTION_CONFLICT,
        "ERROR_ID should be PUMP_DIRECTION_CONFLICT for NEGATIVE + forward target");

    printf("  PASS: Completed-segment direction conflict NEGATIVE target > current\n");
}

/* Test: CONTINUOUSUPDATE + forced Direction with compatible target succeeds
 * after segment completion (verifies the check doesn't block valid restarts). */
static void test_moveabsolute_completed_segment_direction_ok(void)
{
    HYD_MotionControlFB fb;
    HYD_MotionSegment segment;
    HYD_LiveUpdateRequest req;
    HYD_BOOL result;

    printf("--- Test: Completed-segment direction compatible target succeeds ---\n");

    memset(&fb, 0, sizeof(fb));
    fb.USE_RECIPE = false;
    fb.FB_STATE = HYD_FB_STATE_IDLE;

    /* Construct a completed direct segment at position 100 */
    memset(&segment, 0, sizeof(segment));
    segment.mode = HYD_MODE_POSITION;
    segment.endCondition = HYD_END_POSITION;
    segment.targetPosition = 100.0;
    segment.maxVelocity = 20.0;
    segment.maxAcceleration = 50.0;
    segment.maxDeceleration = 50.0;
    segment.maxFlow = 50.0;
    segment.velocityToFlowGain = 1.0;
    segment.direction = HYD_DIRECTION_POSITIVE;
    segment.positionTolerance = 0.5;

    fb.DIRECT_SEGMENT = segment;
    fb.DIRECT_SEGMENT_VALID = true;
    fb.AXIS_REF.timestamp = 1.0;
    fb.AXIS_REF.position = 100.0;
    fb._directOwnerKind = HYD_DIRECT_CMD_MOVE_ABSOLUTE;
    fb._directOwnerTicket = 42;
    fb._executionId = 42;

    /* Simulate a completed direct session */
    fb.STATE.finished = true;
    fb._activeSegmentValid = false;
    fb.STATE.active = false;
    fb._activeSegmentSource = HYD_SEGMENT_SOURCE_NONE;
    fb._previousSegmentMode = HYD_MODE_POSITION;
    fb._activeSegment = segment;

    /* CONTINUOUSUPDATE with Direction=POSITIVE, target=200 (ahead of current=100) */
    memset(&req, 0, sizeof(req));
    req.flags = HYD_LIVE_UPDATE_TARGET_POSITION |
                HYD_LIVE_UPDATE_DIRECTION |
                HYD_LIVE_UPDATE_CONTINUOUS_UPDATE;
    req.ownerKind = HYD_DIRECT_CMD_MOVE_ABSOLUTE;
    req.ownerTicket = 42;
    req.targetPosition = 200.0;
    req.direction = HYD_DIRECTION_POSITIVE;

    result = HYD_MotionControlFB_ApplyLiveUpdate(&fb, &req);
    ASSERT_TRUE(result == true,
        "POSITIVE direction with target > current should SUCCEED after completion");
    ASSERT_TRUE(fabs(fb._activeSegment.targetPosition - 200.0) < 0.001,
        "Active segment target should be updated to 200.0 after restart");
    ASSERT_TRUE(fb._activeSegmentValid == true,
        "Segment should be active after successful restart");

    printf("  PASS: Completed-segment direction compatible target succeeds\n");
}

static void test_validate_segment_accepts_zero_maxvelocity_speed_ramp(void)
{
    HYD_MotionSegment seg;
    HYD_DiagnosticCode code = HYD_DIAG_CODE_NONE;

    printf("--- Test: ValidateSegment accepts maxVelocity=0 for SPEED_RAMP ---\n");

    memset(&seg, 0, sizeof(seg));
    seg.segmentTag = HYD_SEGMENT_TYPE_OTHER;
    seg.segmentType = HYD_SEGMENT_TYPE_OTHER;
    seg.mode = HYD_MODE_SPEED_RAMP;
    seg.endCondition = HYD_END_MANUAL;
    seg.direction = HYD_DIRECTION_POSITIVE;
    seg.planner = HYD_PLANNER_TIME_BASED;
    seg.maxVelocity = 0.0;
    seg.maxAcceleration = 50.0;
    seg.maxDeceleration = 50.0;
    seg.velocityToFlowGain = 0.2f;
    seg.maxFlow = 50.0f;
    seg.timeoutLimit = 30.0f;

    HYD_BOOL result = HYD_RecipeValidator_ValidateSegment(&seg, 0, &code, NULL);
    ASSERT_TRUE(result,
        "ValidateSegment should accept SPEED_RAMP with maxVelocity=0");
    ASSERT_TRUE(code == HYD_DIAG_CODE_NONE,
        "Diagnostic code should be NONE for valid maxVelocity=0 segment");

    printf("  PASS: ValidateSegment accepts maxVelocity=0 for SPEED_RAMP\n");
}

int main(void) {
    printf("=== Motion Interface Unit Tests ===\n\n");

    test_framework_init_resets_pool();
    test_publish_advances_simulation_feedback_time();
    test_simulation_velocity_ramp_uses_fixed_step_after_large_timestamp();
    test_real_axis_velocity_ramp_uses_fixed_step_after_large_timestamp();
    test_moveprofile_init_allocates_fb_with_recipe_mode();
    test_moveprofile_no_execute_does_not_start();
    test_moveprofile_execute_rising_triggers_motion();
    test_moveabsolute_execute_rising_sets_busy_active();
    test_moveabsolute_sustains_busy_active_across_calls();
    test_moveabsolute_owned_fault_sets_error_outputs();
    test_moveabsolute_rejects_invalid_axis_index();
    test_moveabsolute_maps_deceleration_independently();
    test_loadprofile_preloads_single_recipe_segment();
    test_loadprofile_keeps_segment_tag_and_type_separate();
    test_loadprofile_preserves_independent_accel_and_decel();
    test_loadprofile_preloads_direct_segment_on_direct_axis();
    test_loadprofile_keeps_recipe_preload_target_after_direct_override();
    test_moveabsolute_rejects_nonzero_jerk_until_supported();
    test_movevelocity_accepts_continuousupdate_and_updates_active_target();
    test_movevelocity_continuousupdate_lower_target_preserves_ramp();
    test_movevelocity_continuousupdate_rejects_nonfinite_pressure_limit();
    test_movevelocity_pressure_limit_stays_latched_without_continuousupdate();
    test_moveabsolute_continuousupdate_lower_velocity_preserves_ramp();
    test_moveabsolute_accepts_beckhoff_buffer_modes();
    test_movevelocity_execute_rising_starts_velocity_control();
    test_movevelocity_rejects_invalid_axis_index();
    test_movevelocity_maps_deceleration_independently();
    test_movevelocity_maps_explicit_pressure_limit_to_direct_segment();
    test_movevelocity_nonpositive_pressure_limit_uses_axis_default();
    test_movevelocity_rejects_nonfinite_pressure_limit();
    test_stop_on_idle_axis_immediate_done();
    test_stop_rejects_invalid_axis_index();
    test_reset_immediate_done_on_initialized_axis();
    test_reset_immediate_done_on_uninitialized_axis();
    test_reset_preserves_direct_segment_configuration();
    test_pressurehandle_execute_rising_starts_pressure_control();
    test_pressurehandle_accepts_continuousupdate_and_updates_active_target();
    test_pressurehandle_en_false_clears_outputs();
    test_pressurehandle_rejects_invalid_axis_index();
    test_pressurehandle_completion_keeps_completion_semantics();
    test_hold_resume_surface_transitions_active_motion();
    test_hold_resume_reject_invalid_axis_index();
    test_hold_resume_reject_invalid_runtime_state();
    test_multiple_axes_operate_independently();
    test_apply_live_update_tolerates_completed_segment();
    test_direction_enum_values_round_trip();
    test_moveabsolute_shortest_way_direction_accepted();
    test_moveabsolute_current_direction_accepted();
    /* 新增: MoveAbsolute 方向参数全覆盖测试 */
    test_moveabsolute_positive_direction_matching_target_starts();
    test_moveabsolute_positive_direction_rejects_backward_target();
    test_moveabsolute_negative_direction_matching_target_starts();
    test_moveabsolute_negative_direction_rejects_forward_target();
    test_moveabsolute_out_of_range_direction_defaults_to_shortest_way();
    test_moveabsolute_continuous_update_reverse_no_oscillation();
    test_moveabsolute_position_drift_no_restart();
    test_live_update_request_carries_flags_and_direction();
    test_live_update_continuous_suppresses_diagnostic();
    test_movevelocity_live_update_direction_flip();
    test_moveabsolute_live_update_direction_rejected();
    test_pressurehandle_live_update_direction_rejected();
    test_movevelocity_live_update_negative_velocity_flips_direction();
    test_movevelocity_live_update_zero_velocity_decel_to_stop();
    test_moveabsolute_completed_segment_direction_conflict_positive();
    test_moveabsolute_completed_segment_direction_conflict_negative();
    test_moveabsolute_completed_segment_direction_ok();
    test_validate_segment_accepts_zero_maxvelocity_speed_ramp();

    printf("\n=== Results: %d/%d passed ===\n", tests_passed, tests_run);
    return (tests_passed == tests_run) ? 0 : 1;
}
