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

    IEC_VAL(mv.VELOCITY) = 35.0f;
    __mcl_cmd_MoveVelocity(&mv);

    ASSERT_TRUE(IEC_VAL(mv.ERROR) == false,
               "MoveVelocity continuous update should not raise ERROR");
    ASSERT_TRUE(fabs(fb->_activeSegment.maxVelocity - 35.0f) < 0.001f,
               "MoveVelocity continuous update should update active maxVelocity");
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

    memset(&reset, 0, sizeof(reset));
    IEC_VAL(reset.EN) = true;
    IEC_VAL(reset.EXECUTE) = true;
    reset.EXECUTE0.value = false;
    IEC_VAL(reset.AXISID) = 0;
    __mcl_cmd_Reset(&reset);

    ASSERT_TRUE(fb->DIRECT_SEGMENT_VALID == true,
               "SoftReset should preserve the direct segment");
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

int main(void) {
    printf("=== Motion Interface Unit Tests ===\n\n");

    test_framework_init_resets_pool();
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
    test_moveabsolute_accepts_beckhoff_buffer_modes();
    test_movevelocity_execute_rising_starts_velocity_control();
    test_movevelocity_rejects_invalid_axis_index();
    test_movevelocity_maps_deceleration_independently();
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

    printf("\n=== Results: %d/%d passed ===\n", tests_passed, tests_run);
    return (tests_passed == tests_run) ? 0 : 1;
}
