/**
 * @file test_motion_interface_negative_speed.c
 * @brief 测试 __mcl_cmd_GetPumpRequest 的正反向转速仲裁逻辑
 *
 * 验证 ALLOW_NEGATIVE 引脚控制下，正向 MAX 仲裁与反向 MIN 仲裁的正确性，
 * 以及反向优先（卸压优先）的策略。
 *
 * 注意：本测试通过 CreateMotion + __MK_GetPublic_MotionControlFB 分配和
 *       访问 FB 实例。CreateMotion 内部已调用 Init，测试只需设置
 *       STATE.active / PUMP_SPEED / STATE.plannedDirection 三个字段。
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "motion_interface.h"
#include "motion_control.h"
#include "hyd_config.h"

/* 访问 MK 导出的 FB 实例获取函数 */
extern HYD_MotionControlFB* __MK_GetPublic_MotionControlFB(int index);

#define IEC_VAL(var)  ((var).value)

static int tests_run = 0;
static int tests_passed = 0;
static int g_next_idx = 0;  /* 追踪已分配的轴索引，每次测试前重置 */

#define CHECK(cond, msg) do { \
    tests_run++; \
    if (cond) { tests_passed++; printf("  PASS: %s\n", msg); } \
    else { printf("  FAIL: %s  (line %d)\n", msg, __LINE__); } \
} while(0)

/* ==================================================================
 * 辅助：重置轴分配计数器（每个测试函数开头调用）
 * ================================================================== */
static void reset_axis_allocator(void)
{
    g_next_idx = 0;
    __HydMotion_framework_Init();
}

/* ==================================================================
 * 辅助：分配一个 Direct 模式轴，返回 FB 指针
 * CreateMotion 内部已调用 HYD_MotionControlFB_Init，无需再次 memset。
 * ================================================================== */
static HYD_MotionControlFB* alloc_one_axis(void)
{
    HYD_CREATEMOTION cm;
    memset(&cm, 0, sizeof(cm));
    IEC_VAL(cm.EN) = TRUE;
    IEC_VAL(cm.USE_RECIPE) = FALSE;
    IEC_VAL(cm.FLOW_TO_PUMPSPEED) = 1.0f;
    IEC_VAL(cm.PUMPSPEED_LIMIT) = 1500.0f;
    IEC_VAL(cm.USE_SIMULATION) = FALSE;
    __mcl_cmd_CreateMotion(&cm);
    return __MK_GetPublic_MotionControlFB(g_next_idx++);
}

/* ==================================================================
 * Test 1: 只有正转速请求时，输出最大值（原行为不变）
 * ================================================================== */
static void test_positive_only(void)
{
    reset_axis_allocator();

    HYD_MotionControlFB* fb1 = alloc_one_axis();
    HYD_MotionControlFB* fb2 = alloc_one_axis();
    HYD_MotionControlFB* fb3 = alloc_one_axis();

    /* CreateMotion 已 Init，只需设置测试所需字段 */
    fb1->STATE.active = TRUE;  fb1->PUMP_SPEED = 100.0f;  fb1->STATE.plannedDirection = HYD_DIRECTION_EXTEND;
    fb2->STATE.active = TRUE;  fb2->PUMP_SPEED = 200.0f;  fb2->STATE.plannedDirection = HYD_DIRECTION_EXTEND;
    fb3->STATE.active = TRUE;  fb3->PUMP_SPEED = 150.0f;  fb3->STATE.plannedDirection = HYD_DIRECTION_EXTEND;

    HYD_GETPUMPREQUEST req;
    memset(&req, 0, sizeof(req));
    IEC_VAL(req.ENABLE) = TRUE;
    IEC_VAL(req.ALLOW_NEGATIVE) = FALSE;

    __mcl_cmd_GetPumpRequest(&req);

    CHECK(fabs(IEC_VAL(req.PUMPSPEED) - 200.0f) < 1e-3f, "positive_only: MAX=200");
    CHECK(IEC_VAL(req.CONFLICT) == FALSE, "positive_only: no conflict");
}

/* ==================================================================
 * Test 2: ALLOW_NEGATIVE=TRUE，有负转速时输出最小值
 * ================================================================== */
static void test_negative_allowed(void)
{
    reset_axis_allocator();

    HYD_MotionControlFB* fb1 = alloc_one_axis();
    HYD_MotionControlFB* fb2 = alloc_one_axis();
    HYD_MotionControlFB* fb3 = alloc_one_axis();

    fb1->STATE.active = TRUE;  fb1->PUMP_SPEED = 100.0f;  fb1->STATE.plannedDirection = HYD_DIRECTION_EXTEND;
    fb2->STATE.active = TRUE;  fb2->PUMP_SPEED = -50.0f;  fb2->STATE.plannedDirection = HYD_DIRECTION_HOLD;
    fb3->STATE.active = TRUE;  fb3->PUMP_SPEED = -30.0f;  fb3->STATE.plannedDirection = HYD_DIRECTION_HOLD;

    HYD_GETPUMPREQUEST req;
    memset(&req, 0, sizeof(req));
    IEC_VAL(req.ENABLE) = TRUE;
    IEC_VAL(req.ALLOW_NEGATIVE) = TRUE;

    __mcl_cmd_GetPumpRequest(&req);

    CHECK(fabs(IEC_VAL(req.PUMPSPEED) - (-50.0f)) < 1e-3f, "negative_allowed: MIN=-50");
}

/* ==================================================================
 * Test 3: ALLOW_NEGATIVE=FALSE，负转速被忽略
 * ================================================================== */
static void test_negative_not_allowed(void)
{
    reset_axis_allocator();

    HYD_MotionControlFB* fb1 = alloc_one_axis();
    HYD_MotionControlFB* fb2 = alloc_one_axis();
    HYD_MotionControlFB* fb3 = alloc_one_axis();

    fb1->STATE.active = TRUE;  fb1->PUMP_SPEED = 100.0f;  fb1->STATE.plannedDirection = HYD_DIRECTION_EXTEND;
    fb2->STATE.active = TRUE;  fb2->PUMP_SPEED = -50.0f;  fb2->STATE.plannedDirection = HYD_DIRECTION_HOLD;
    fb3->STATE.active = TRUE;  fb3->PUMP_SPEED = -30.0f;  fb3->STATE.plannedDirection = HYD_DIRECTION_HOLD;

    HYD_GETPUMPREQUEST req;
    memset(&req, 0, sizeof(req));
    IEC_VAL(req.ENABLE) = TRUE;
    IEC_VAL(req.ALLOW_NEGATIVE) = FALSE;

    __mcl_cmd_GetPumpRequest(&req);

    CHECK(fabs(IEC_VAL(req.PUMPSPEED) - 100.0f) < 1e-3f, "negative_not_allowed: MAX=100 (neg ignored)");
}

/* ==================================================================
 * Test 4: 同时存在正负转速，反向优先（卸压优先）
 * ================================================================== */
static void test_reverse_priority(void)
{
    reset_axis_allocator();

    HYD_MotionControlFB* fb1 = alloc_one_axis();
    HYD_MotionControlFB* fb2 = alloc_one_axis();
    HYD_MotionControlFB* fb3 = alloc_one_axis();
    HYD_MotionControlFB* fb4 = alloc_one_axis();

    fb1->STATE.active = TRUE;  fb1->PUMP_SPEED = 500.0f;  fb1->STATE.plannedDirection = HYD_DIRECTION_EXTEND;
    fb2->STATE.active = TRUE;  fb2->PUMP_SPEED = 300.0f;  fb2->STATE.plannedDirection = HYD_DIRECTION_EXTEND;
    fb3->STATE.active = TRUE;  fb3->PUMP_SPEED = -80.0f;  fb3->STATE.plannedDirection = HYD_DIRECTION_HOLD;
    fb4->STATE.active = TRUE;  fb4->PUMP_SPEED = -20.0f;  fb4->STATE.plannedDirection = HYD_DIRECTION_HOLD;

    HYD_GETPUMPREQUEST req;
    memset(&req, 0, sizeof(req));
    IEC_VAL(req.ENABLE) = TRUE;
    IEC_VAL(req.ALLOW_NEGATIVE) = TRUE;

    __mcl_cmd_GetPumpRequest(&req);

    CHECK(fabs(IEC_VAL(req.PUMPSPEED) - (-80.0f)) < 1e-3f, "reverse_priority: reverse wins (-80)");
}

/* ==================================================================
 * Test 5: 无活跃 FB 时输出 0
 * ================================================================== */
static void test_no_active_fb(void)
{
    reset_axis_allocator();

    HYD_MotionControlFB* fb1 = alloc_one_axis();
    HYD_MotionControlFB* fb2 = alloc_one_axis();

    /* 分配了但设置为不活跃 */
    fb1->STATE.active = FALSE;
    fb2->STATE.active = FALSE;

    HYD_GETPUMPREQUEST req;
    memset(&req, 0, sizeof(req));
    IEC_VAL(req.ENABLE) = TRUE;
    IEC_VAL(req.ALLOW_NEGATIVE) = TRUE;

    __mcl_cmd_GetPumpRequest(&req);

    CHECK(fabs(IEC_VAL(req.PUMPSPEED) - 0.0f) < 1e-3f, "no_active_fb: output=0");
}

/* ==================================================================
 * Test 6: EXTEND 和 RETRACT 同时出现时 CONFLICT=TRUE
 * ================================================================== */
static void test_conflict_flag(void)
{
    reset_axis_allocator();

    HYD_MotionControlFB* fb1 = alloc_one_axis();
    HYD_MotionControlFB* fb2 = alloc_one_axis();

    fb1->STATE.active = TRUE;  fb1->PUMP_SPEED = 100.0f;  fb1->STATE.plannedDirection = HYD_DIRECTION_EXTEND;
    fb2->STATE.active = TRUE;  fb2->PUMP_SPEED =  50.0f;  fb2->STATE.plannedDirection = HYD_DIRECTION_RETRACT;

    HYD_GETPUMPREQUEST req;
    memset(&req, 0, sizeof(req));
    IEC_VAL(req.ENABLE) = TRUE;
    IEC_VAL(req.ALLOW_NEGATIVE) = FALSE;

    __mcl_cmd_GetPumpRequest(&req);

    CHECK(IEC_VAL(req.CONFLICT) == TRUE, "conflict_flag: EXTEND+RETRACT");
}

/* ==================================================================
 * Test 7: ENABLE=FALSE 时输出 0，CONFLICT=FALSE，DONE=TRUE
 * ================================================================== */
static void test_enable_false(void)
{
    reset_axis_allocator();

    HYD_MotionControlFB* fb1 = alloc_one_axis();
    fb1->STATE.active = TRUE;
    fb1->PUMP_SPEED = 100.0f;

    HYD_GETPUMPREQUEST req;
    memset(&req, 0, sizeof(req));
    IEC_VAL(req.ENABLE) = FALSE;

    __mcl_cmd_GetPumpRequest(&req);

    CHECK(fabs(IEC_VAL(req.PUMPSPEED) - 0.0f) < 1e-3f, "enable_false: output=0");
    CHECK(IEC_VAL(req.CONFLICT) == FALSE, "enable_false: no conflict");
    CHECK(IEC_VAL(req.DONE) == TRUE, "enable_false: DONE=TRUE");
}

/* ==================================================================
 * Test 8: 负转速不被钳位到 0（验证 ALLOW_NEGATIVE 生效）
 * pumpSpeedLimit=1500, RATIO=0.05 → 下限=-75，-60 在范围内
 * ================================================================== */
static void test_negative_speed_not_clamped(void)
{
    reset_axis_allocator();

    HYD_MotionControlFB* fb1 = alloc_one_axis();
    fb1->STATE.active = TRUE;
    fb1->PUMP_SPEED = -60.0f;
    fb1->STATE.plannedDirection = HYD_DIRECTION_HOLD;

    HYD_GETPUMPREQUEST req;
    memset(&req, 0, sizeof(req));
    IEC_VAL(req.ENABLE) = TRUE;
    IEC_VAL(req.ALLOW_NEGATIVE) = TRUE;

    __mcl_cmd_GetPumpRequest(&req);

    /* 输出应该是 -60，而不是被钳位到 0 */
    CHECK(fabs(IEC_VAL(req.PUMPSPEED) - (-60.0f)) < 1e-3f,
          "negative_speed_not_clamped: output=-60 (not 0)");
}

/* ================================================================== */
int main(void)
{
    printf("=== test_motion_interface_negative_speed ===\n");

    test_positive_only();
    test_negative_allowed();
    test_negative_not_allowed();
    test_reverse_priority();
    test_no_active_fb();
    test_conflict_flag();
    test_enable_false();
    test_negative_speed_not_clamped();

    printf("\n=== Results: %d/%d passed ===\n", tests_passed, tests_run);
    return (tests_passed == tests_run) ? 0 : 1;
}
