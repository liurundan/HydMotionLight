/**
 * test_stop_moveabsolute_interaction.c
 *
 * 从 IEC ST PLC 程序 TESTMOTION 转换而来的标准 C 测试用例。
 * 重点测试 MoveAbsolute 与 Stop 命令的交互：
 *   - 运动中执行 Stop，验证减速至0后 DONE 信号
 *   - Stop.DONE 后清除 EXECUTE，验证信号正确归零
 *   - Stop 后重新启动 MoveAbsolute，验证可重入
 *
 * 编译: 参照项目 CMakeLists.txt 添加此文件到测试列表
 */
 
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdbool.h>
 
#include "motion_interface.h"
#include "motion_control.h"
 
extern HYD_MotionControlFB* __MK_GetPublic_MotionControlFB(int index);
 
/* ======================================================================
 * IEC 变量访问宏（与 test_stop_immediate_done.c 保持一致）
 * ====================================================================== */
#define IEC_VAL(var) ((var).value)
 
/* ======================================================================
 * 测试框架
 * ====================================================================== */
static int g_tests_run    = 0;
static int g_tests_passed = 0;
 
#define CHECK(cond, msg) do { \
    g_tests_run++; \
    if (cond) { g_tests_passed++; printf("  PASS: %s\n", msg); } \
    else { printf("  FAIL: %s  [line %d]\n", msg, __LINE__); } \
} while (0)
 
#define CYCLE_PERIOD 0.001  /* 1ms PLC 扫描周期 */
 
/* ======================================================================
 * 辅助：创建仿真轴（对应 PLC step 0 的 CreateMotion）
 * ====================================================================== */
static int create_sim_axis(void) {
    HYD_CREATEMOTION cm;
    memset(&cm, 0, sizeof(cm));
    IEC_VAL(cm.EN)            = true;
    IEC_VAL(cm.USE_RECIPE)    = false;        /* Direct 模式 */
    IEC_VAL(cm.FLOW_TO_PUMPSPEED) = 1.0f;
    IEC_VAL(cm.PUMPSPEED_LIMIT)   = 3000.0f;
    IEC_VAL(cm.USE_SIMULATION)    = true;      /* 使用内嵌仿真 */
    __mcl_cmd_CreateMotion(&cm);
    return (int)IEC_VAL(cm.AXISID);
}
 
/* ======================================================================
 * 辅助：读取仿真反馈（对应 PLC 的 ReadSimFeedback）
 * ====================================================================== */
static void read_sim_feedback(int axisId,
                              HYD_REAL *pos, HYD_REAL *vel,
                              HYD_REAL *flow, HYD_REAL *pressure) {
    HYD_READSIMFEEDBACK fb;
    memset(&fb, 0, sizeof(fb));
    IEC_VAL(fb.ENABLE)  = true;
    IEC_VAL(fb.AXISID)  = (IEC_SINT)axisId;
    __mcl_cmd_ReadSimFeedback(&fb);
    if (pos)      *pos      = IEC_VAL(fb.POSITION);
    if (vel)      *vel      = IEC_VAL(fb.VELOCITY);
    if (flow)     *flow     = IEC_VAL(fb.FLOW);
    if (pressure) *pressure = IEC_VAL(fb.PRESSURE);
}
 
/* ======================================================================
 * 辅助：读取轴状态（对应 PLC 的 ReadStatus）
 * ====================================================================== */
static IEC_UINT read_axis_state(int axisId) {
    HYD_READSTATUS fb;
    memset(&fb, 0, sizeof(fb));
    IEC_VAL(fb.ENABLE) = true;
    IEC_VAL(fb.AXISID) = (IEC_SINT)axisId;
    __mcl_cmd_ReadStatus(&fb);
    return IEC_VAL(fb.STATE);
}
 
/* ======================================================================
 * 测试 1：MoveAbsolute → Stop 中断 → 减速至0 → DONE → 清除 EXECUTE
 *
 * 还原 PLC 的 step 1 / step 3 交互：
 *   step 1: MoveAbsolute(400, 20, 200, 方向=1)
 *   运行中途触发 step 3: Stop(DECELERATION=50)
 *   检测 Stop.DONE → 清除 EXECUTE → 验证可重新启动
 * ====================================================================== */
static void test_stop_during_moveabsolute(void) {
    HYD_MOVEABSOLUTE ma;
    HYD_STOP          stop;
    int axisId;
    int step;
    HYD_REAL velBeforeStop = 0.0;
    int  stopDoneCycle      = 0;
    HYD_REAL posAtStopDone  = 0.0;
    HYD_REAL velAtStopDone  = 0.0;
 
    printf("\n=== Test 1: Stop during MoveAbsolute ===\n");
    __HydMotion_framework_Init();
    axisId = create_sim_axis();
    CHECK(axisId >= 0, "Create sim axis");
 
    /* ----------------------------------------------------------
     * Step 1 等价：启动 MoveAbsolute(POSITION=400)
     * ---------------------------------------------------------- */
    memset(&ma, 0, sizeof(ma));
    IEC_VAL(ma.EN)          = true;
    IEC_VAL(ma.AXISID)      = axisId;
    IEC_VAL(ma.POSITION)    = 400.0f;
    IEC_VAL(ma.VELOCITY)    = 20.0f;
    IEC_VAL(ma.ACCELERATION)= 200.0f;
    IEC_VAL(ma.DECELERATION)= 200.0f;
    IEC_VAL(ma.DIRECTION)   = 1;          /* EXTEND */
    IEC_VAL(ma.BUFFERMODE)  = 0;          /* ABORT (与 PLC 一致) */
    IEC_VAL(ma.EXECUTE)     = true;
    ma.EXECUTE0.value       = false;       /* 上升沿 */
    __mcl_cmd_MoveAbsolute(&ma);
 
    /* 等待 MoveAbsolute 进入 active（_PENDING→confirmed） */
    for (step = 0; step < 10; step++) {
        __HydMotion_framework_Publish();
        IEC_VAL(ma.EXECUTE)     = true;
        ma.EXECUTE0.value       = true;
        __mcl_cmd_MoveAbsolute(&ma);
        if (IEC_VAL(ma.ACTIVE)) break;
    }
    CHECK(IEC_VAL(ma.ACTIVE), "MoveAbsolute should become ACTIVE");
 
    /* 继续运行若干周期，积累速度 */
    for (step = 0; step < 80; step++) {
        __HydMotion_framework_Publish();
        IEC_VAL(ma.EXECUTE)     = true;
        ma.EXECUTE0.value       = true;
        __mcl_cmd_MoveAbsolute(&ma);
    }
 
    /* 记录 Stop 前的速度 */
    {
        HYD_MotionControlFB* fb = __MK_GetPublic_MotionControlFB(axisId);
        velBeforeStop = fb->AXIS_REF.velocity;
        printf("  Velocity BEFORE Stop: %.4f mm/s\n", (double)velBeforeStop);
    }
    CHECK(velBeforeStop > 1.0, "Axis should be moving before Stop");
 
    /* ----------------------------------------------------------
     * Step 3 等价：执行 Stop(DECELERATION=50)
     * ---------------------------------------------------------- */
    memset(&stop, 0, sizeof(stop));
    IEC_VAL(stop.EN)          = true;
    IEC_VAL(stop.EXECUTE)     = true;
    stop.EXECUTE0.value       = false;      /* 上升沿 */
    IEC_VAL(stop.AXISID)      = axisId;
    IEC_VAL(stop.DECELERATION)= 50.0f;
    __mcl_cmd_Stop(&stop);
 
    /* Stop 第一次调用不应立即 DONE */
    CHECK(IEC_VAL(stop.DONE) == false,
          "Stop.DONE should be false on first call (deceleration not done)");
 
    /* 循环等待 Stop.DONE，同时保持 MoveAbsolute EXECUTE=true */
    stopDoneCycle = 0;
    for (step = 0; step < 5000; step++) {
        __HydMotion_framework_Publish();
 
        /* MoveAbsolute 保持 EXECUTE=true（PLC 中 step 3 没有清除它） */
        IEC_VAL(ma.EXECUTE)     = true;
        ma.EXECUTE0.value       = true;
        __mcl_cmd_MoveAbsolute(&ma);
 
        /* Stop 保持 EXECUTE=true */
        IEC_VAL(stop.EXECUTE)   = true;
        stop.EXECUTE0.value     = true;
        __mcl_cmd_Stop(&stop);
 
        if (IEC_VAL(stop.DONE)) {
            stopDoneCycle = step + 1;
 
            /* 读取减速结束时的位置和速度 */
            {
                HYD_MotionControlFB* fb = __MK_GetPublic_MotionControlFB(axisId);
                posAtStopDone = fb->AXIS_REF.position;
                velAtStopDone = fb->AXIS_REF.velocity;
            }
            break;
        }
    }
 
    printf("  Stop.DONE reached after %d cycles\n", stopDoneCycle);
    printf("  Position at Stop.DONE:  %.4f mm\n", (double)posAtStopDone);
    printf("  Velocity at Stop.DONE:  %.4f mm/s\n", (double)velAtStopDone);
 
    /* ---- 核心断言 ---- */
    CHECK(stopDoneCycle > 5,
          "Stop.DONE should take >5 cycles (deceleration takes time)");
    CHECK(IEC_VAL(stop.DONE) == true,
          "Stop.DONE should be true after deceleration");
    CHECK(IEC_VAL(stop.BUSY) == false,
          "Stop.BUSY should be false after DONE");
    CHECK(fabs(velAtStopDone) < 0.01,
          "Velocity should be ~0 at Stop.DONE");
    CHECK(IEC_VAL(stop.ERROR) == false,
          "Stop should not set ERROR");
 
    /* ----------------------------------------------------------
     * Stop.DONE 后：清除 EXECUTE（还原 PLC 逻辑）
     *   FBHYD_STOP.EXECUTE := FALSE
     *   FBHYD_MOVEABSOLUTE.EXECUTE := FALSE
     * ---------------------------------------------------------- */
    printf("  --- Clearing EXECUTE after Stop.DONE ---\n");
 
    /* Stop: EXECUTE=FALSE */
    IEC_VAL(stop.EXECUTE)   = false;
    stop.EXECUTE0.value     = true;   /* 下降沿：execute=0, EXECUTE0=1 */
    __mcl_cmd_Stop(&stop);
 
    /* MoveAbsolute: EXECUTE=FALSE */
    IEC_VAL(ma.EXECUTE)     = false;
    ma.EXECUTE0.value       = true;
    __mcl_cmd_MoveAbsolute(&ma);
 
    /* 再运行一个周期让下降沿生效 */
    __HydMotion_framework_Publish();
 
    IEC_VAL(stop.EXECUTE)   = false;
    stop.EXECUTE0.value     = false;
    __mcl_cmd_Stop(&stop);
 
    IEC_VAL(ma.EXECUTE)     = false;
    ma.EXECUTE0.value       = false;
    __mcl_cmd_MoveAbsolute(&ma);
 
    printf("  Stop:      DONE=%d BUSY=%d COMMANDABORTED=%d ERROR=%d\n",
           (int)IEC_VAL(stop.DONE), (int)IEC_VAL(stop.BUSY),
           (int)IEC_VAL(stop.COMMANDABORTED), (int)IEC_VAL(stop.ERROR));
    printf("  MoveAbs:   DONE=%d BUSY=%d ACTIVE=%d COMMANDABORTED=%d ERROR=%d\n",
           (int)IEC_VAL(ma.DONE), (int)IEC_VAL(ma.BUSY),
           (int)IEC_VAL(ma.ACTIVE), (int)IEC_VAL(ma.COMMANDABORTED),
           (int)IEC_VAL(ma.ERROR));
 
    /* EXECUTE 归零后，DONE 信号应被清除 */
    CHECK(IEC_VAL(stop.DONE) == false,
          "Stop.DONE should clear after EXECUTE goes FALSE");
    CHECK(IEC_VAL(stop.BUSY) == false,
          "Stop.BUSY should be false after cleanup");
    CHECK(IEC_VAL(ma.COMMANDABORTED) == false,
          "MoveAbsolute should NOT report COMMANDABORTED after Stop");
 
    /* ----------------------------------------------------------
     * 验证可重入：再次启动 MoveAbsolute
     * ---------------------------------------------------------- */
    printf("  --- Restart MoveAbsolute after Stop ---\n");
 
    memset(&ma, 0, sizeof(ma));
    IEC_VAL(ma.EN)          = true;
    IEC_VAL(ma.AXISID)      = axisId;
    IEC_VAL(ma.POSITION)    = 0.0f;         /* 反向 */
    IEC_VAL(ma.VELOCITY)    = 20.0f;
    IEC_VAL(ma.ACCELERATION)= 200.0f;
    IEC_VAL(ma.DECELERATION)= 200.0f;
    IEC_VAL(ma.DIRECTION)   = -1;           /* RETRACT */
    IEC_VAL(ma.BUFFERMODE)  = 0;
    IEC_VAL(ma.EXECUTE)     = true;
    ma.EXECUTE0.value       = false;        /* 上升沿 */
    __mcl_cmd_MoveAbsolute(&ma);
 
    /* 等待 active */
    HYD_BOOL becameActive = false;
    for (step = 0; step < 100; step++) {
        __HydMotion_framework_Publish();
        IEC_VAL(ma.EXECUTE)     = true;
        ma.EXECUTE0.value       = true;
        __mcl_cmd_MoveAbsolute(&ma);
        if (IEC_VAL(ma.ACTIVE)) {
            becameActive = true;
            break;
        }
    }
    CHECK(becameActive,
          "MoveAbsolute should become ACTIVE after restart (Stop cleared state)");
 
    /* 运行一段时间验证正常运动 */
    for (step = 0; step < 50; step++) {
        __HydMotion_framework_Publish();
        IEC_VAL(ma.EXECUTE)     = true;
        ma.EXECUTE0.value       = true;
        __mcl_cmd_MoveAbsolute(&ma);
    }
    {
        HYD_MotionControlFB* fb = __MK_GetPublic_MotionControlFB(axisId);
        printf("  After restart: Pos=%.2f Vel=%.2f FB_STATE=%d\n",
               (double)fb->AXIS_REF.position,
               (double)fb->AXIS_REF.velocity,
               fb->FB_STATE);
        CHECK(fb->AXIS_REF.velocity < -0.5 || fb->AXIS_REF.velocity > 0.5,
              "Axis should be moving after MoveAbsolute restart");
    }
}
 
/* ======================================================================
 * 测试 2：完整 PLC 步进循环
 *   step 0 → CreateMotion
 *   step 1 → MoveAbsolute(400) + Stop 中断
 *   step 3 → Stop → DONE → 清除 → 回到 step 2
 *   step 2 → MoveAbsolute(0) → 正常完成
 * ====================================================================== */
static void test_full_plc_step_cycle(void) {
    HYD_MOVEABSOLUTE   ma;
    HYD_STOP           stop;
    int axisId = -1;
    int istep = -1;
    int step;
    int stopDoneCycle = 0;
 
    printf("\n=== Test 2: Full PLC step cycle with Stop interrupt ===\n");
    __HydMotion_framework_Init();
 
    /* ---- Step 0: CreateMotion ---- */
    istep = 0;
    {
        HYD_CREATEMOTION cm;
        memset(&cm, 0, sizeof(cm));
        IEC_VAL(cm.USE_RECIPE)       = false;
        IEC_VAL(cm.FLOW_TO_PUMPSPEED)= 1.0f;
        IEC_VAL(cm.USE_SIMULATION)   = true;
        __mcl_cmd_CreateMotion(&cm);
        if (IEC_VAL(cm.DONE)) {
            axisId = (int)IEC_VAL(cm.AXISID);
            istep  = 1;
            printf("  Step 0: CreateMotion done, axisId=%d\n", axisId);
        }
    }
    CHECK(axisId >= 0, "Step 0: CreateMotion should succeed");
 
    /* ---- Step 1: MoveAbsolute(POSITION=400) ---- */
    CHECK(istep == 1, "Should be at step 1");
    {
        memset(&ma, 0, sizeof(ma));
        IEC_VAL(ma.EN)          = true;
        IEC_VAL(ma.AXISID)      = axisId;
        IEC_VAL(ma.POSITION)    = 400.0f;
        IEC_VAL(ma.VELOCITY)    = 20.0f;
        IEC_VAL(ma.ACCELERATION)= 200.0f;
        IEC_VAL(ma.DECELERATION)= 200.0f;
        IEC_VAL(ma.DIRECTION)   = 1;
        IEC_VAL(ma.BUFFERMODE)  = 0;
        IEC_VAL(ma.EXECUTE)     = true;
        ma.EXECUTE0.value       = false;
        __mcl_cmd_MoveAbsolute(&ma);
    }
 
    /* 运行 60 个周期积累速度 */
    for (step = 0; step < 60; step++) {
        __HydMotion_framework_Publish();
        IEC_VAL(ma.EXECUTE)     = true;
        ma.EXECUTE0.value       = true;
        __mcl_cmd_MoveAbsolute(&ma);
    }
 
    /* 模拟外部触发：进入 step 3 (Stop) */
    istep = 3;
    printf("  Switching to step 3 (Stop) at cycle %d\n", step);
 
    /* ---- Step 3: Stop ---- */
    {
        memset(&stop, 0, sizeof(stop));
        IEC_VAL(stop.EN)          = true;
        IEC_VAL(stop.EXECUTE)     = true;
        stop.EXECUTE0.value       = false;
        IEC_VAL(stop.AXISID)      = axisId;
        IEC_VAL(stop.DECELERATION)= 50.0f;
        __mcl_cmd_Stop(&stop);
    }
 
    stopDoneCycle = 0;
    for (step = 0; step < 5000; step++) {
        __HydMotion_framework_Publish();
 
        IEC_VAL(ma.EXECUTE)     = true;
        ma.EXECUTE0.value       = true;
        __mcl_cmd_MoveAbsolute(&ma);
 
        IEC_VAL(stop.EXECUTE)   = true;
        stop.EXECUTE0.value     = true;
        __mcl_cmd_Stop(&stop);
 
        if (IEC_VAL(stop.DONE)) {
            stopDoneCycle = step + 1;
            break;
        }
    }
 
    printf("  Stop.DONE at cycle %d\n", stopDoneCycle);
    CHECK(stopDoneCycle > 0, "Stop should eventually report DONE");
 
    /* Stop.DONE → 清除 EXECUTE（还原 PLC 逻辑） */
    IEC_VAL(ma.EXECUTE)     = false;
    ma.EXECUTE0.value       = true;
    __mcl_cmd_MoveAbsolute(&ma);
 
    IEC_VAL(stop.EXECUTE)   = false;
    stop.EXECUTE0.value     = true;
    __mcl_cmd_Stop(&stop);
 
    __HydMotion_framework_Publish();
 
    IEC_VAL(ma.EXECUTE)     = false;
    ma.EXECUTE0.value       = false;
    __mcl_cmd_MoveAbsolute(&ma);
 
    IEC_VAL(stop.EXECUTE)   = false;
    stop.EXECUTE0.value     = false;
    __mcl_cmd_Stop(&stop);
 
    /* 验证清除后状态干净 */
    CHECK(IEC_VAL(stop.DONE) == false,
          "Step 3: Stop.DONE should clear after EXECUTE=FALSE");
    CHECK(IEC_VAL(ma.COMMANDABORTED) == false,
          "Step 3: MoveAbsolute should NOT be COMMANDABORTED after Stop");
 
    printf("  After cleanup: Stop.DONE=%d MA.DONE=%d MA.ACTIVE=%d MA.CMDABORTED=%d\n",
           (int)IEC_VAL(stop.DONE), (int)IEC_VAL(ma.DONE),
           (int)IEC_VAL(ma.ACTIVE), (int)IEC_VAL(ma.COMMANDABORTED));
 
    /* ---- 进入 Step 2: MoveAbsolute(POSITION=0, DIR=-1) ---- */
    istep = 2;
    printf("  Entering step 2: MoveAbsolute(0, retract)\n");
 
    memset(&ma, 0, sizeof(ma));
    IEC_VAL(ma.EN)          = true;
    IEC_VAL(ma.AXISID)      = axisId;
    IEC_VAL(ma.POSITION)    = 0.0f;
    IEC_VAL(ma.VELOCITY)    = 20.0f;
    IEC_VAL(ma.ACCELERATION)= 200.0f;
    IEC_VAL(ma.DECELERATION)= 200.0f;
    IEC_VAL(ma.DIRECTION)   = -1;
    IEC_VAL(ma.BUFFERMODE)  = 0;
    IEC_VAL(ma.EXECUTE)     = true;
    ma.EXECUTE0.value       = false;
    __mcl_cmd_MoveAbsolute(&ma);
 
    /* 等待 active */
    HYD_BOOL becameActive = false;
    for (step = 0; step < 100; step++) {
        __HydMotion_framework_Publish();
        IEC_VAL(ma.EXECUTE)     = true;
        ma.EXECUTE0.value       = true;
        __mcl_cmd_MoveAbsolute(&ma);
        if (IEC_VAL(ma.ACTIVE)) {
            becameActive = true;
            break;
        }
    }
    CHECK(becameActive,
          "Step 2: MoveAbsolute should become ACTIVE after Stop cleanup");
 
    /* 运行验证正常运动 */
    for (step = 0; step < 30; step++) {
        __HydMotion_framework_Publish();
        IEC_VAL(ma.EXECUTE)     = true;
        ma.EXECUTE0.value       = true;
        __mcl_cmd_MoveAbsolute(&ma);
    }
    {
        HYD_MotionControlFB* fb = __MK_GetPublic_MotionControlFB(axisId);
        printf("  Step 2 running: Pos=%.2f Vel=%.2f FB_STATE=%d\n",
               (double)fb->AXIS_REF.position,
               (double)fb->AXIS_REF.velocity,
               fb->FB_STATE);
        CHECK(fb->FB_STATE == HYD_FB_STATE_RUNNING ||
              fb->FB_STATE == HYD_FB_STATE_STARTING,
              "Step 2: FB should be RUNNING after successful restart");
    }
}
 
/* ======================================================================
 * 测试 3：Stop.DONE 后再次 Stop（验证 Stop FB 可重复使用）
 * ====================================================================== */
static void test_stop_reusable(void) {
    HYD_MOVEABSOLUTE ma;
    HYD_STOP          stop;
    int axisId, step;
    int stopDoneCycle;
 
    printf("\n=== Test 3: Stop FB reusable after DONE + EXECUTE clear ===\n");
    __HydMotion_framework_Init();
    axisId = create_sim_axis();
    CHECK(axisId >= 0, "Create sim axis");
 
    /* 第一次 MoveAbsolute + Stop */
    memset(&ma, 0, sizeof(ma));
    IEC_VAL(ma.EN)          = true;
    IEC_VAL(ma.AXISID)      = axisId;
    IEC_VAL(ma.POSITION)    = 400.0f;
    IEC_VAL(ma.VELOCITY)    = 20.0f;
    IEC_VAL(ma.ACCELERATION)= 200.0f;
    IEC_VAL(ma.DIRECTION)   = 1;
    IEC_VAL(ma.BUFFERMODE)  = 0;
    IEC_VAL(ma.EXECUTE)     = true;
    ma.EXECUTE0.value       = false;
    __mcl_cmd_MoveAbsolute(&ma);
 
    for (step = 0; step < 60; step++) {
        __HydMotion_framework_Publish();
        IEC_VAL(ma.EXECUTE)     = true;
        ma.EXECUTE0.value       = true;
        __mcl_cmd_MoveAbsolute(&ma);
    }
 
    /* 第一次 Stop */
    memset(&stop, 0, sizeof(stop));
    IEC_VAL(stop.EN)          = true;
    IEC_VAL(stop.EXECUTE)     = true;
    stop.EXECUTE0.value       = false;
    IEC_VAL(stop.AXISID)      = axisId;
    IEC_VAL(stop.DECELERATION)= 50.0f;
    __mcl_cmd_Stop(&stop);
 
    stopDoneCycle = 0;
    for (step = 0; step < 5000; step++) {
        __HydMotion_framework_Publish();
        IEC_VAL(ma.EXECUTE)     = true;
        ma.EXECUTE0.value       = true;
        __mcl_cmd_MoveAbsolute(&ma);
        IEC_VAL(stop.EXECUTE)   = true;
        stop.EXECUTE0.value     = true;
        __mcl_cmd_Stop(&stop);
        if (IEC_VAL(stop.DONE)) { stopDoneCycle = step + 1; break; }
    }
    CHECK(stopDoneCycle > 0, "First Stop should reach DONE");
 
    /* 清除 EXECUTE */
    IEC_VAL(ma.EXECUTE)     = false;
    ma.EXECUTE0.value       = true;
    __mcl_cmd_MoveAbsolute(&ma);
    IEC_VAL(stop.EXECUTE)   = false;
    stop.EXECUTE0.value     = true;
    __mcl_cmd_Stop(&stop);
    __HydMotion_framework_Publish();
    IEC_VAL(ma.EXECUTE)     = false;
    ma.EXECUTE0.value       = false;
    __mcl_cmd_MoveAbsolute(&ma);
    IEC_VAL(stop.EXECUTE)   = false;
    stop.EXECUTE0.value     = false;
    __mcl_cmd_Stop(&stop);
 
    CHECK(IEC_VAL(stop.DONE) == false, "Stop.DONE should clear after EXECUTE=0");
 
    /* 第二次 MoveAbsolute + Stop（验证可重入） */
    printf("  --- Second round ---\n");
    memset(&ma, 0, sizeof(ma));
    IEC_VAL(ma.EN)          = true;
    IEC_VAL(ma.AXISID)      = axisId;
    IEC_VAL(ma.POSITION)    = 400.0f;
    IEC_VAL(ma.VELOCITY)    = 20.0f;
    IEC_VAL(ma.ACCELERATION)= 200.0f;
    IEC_VAL(ma.DIRECTION)   = 1;
    IEC_VAL(ma.BUFFERMODE)  = 0;
    IEC_VAL(ma.EXECUTE)     = true;
    ma.EXECUTE0.value       = false;
    __mcl_cmd_MoveAbsolute(&ma);
 
    for (step = 0; step < 60; step++) {
        __HydMotion_framework_Publish();
        IEC_VAL(ma.EXECUTE)     = true;
        ma.EXECUTE0.value       = true;
        __mcl_cmd_MoveAbsolute(&ma);
    }
 
    /* 第二次 Stop */
    memset(&stop, 0, sizeof(stop));
    IEC_VAL(stop.EN)          = true;
    IEC_VAL(stop.EXECUTE)     = true;
    stop.EXECUTE0.value       = false;
    IEC_VAL(stop.AXISID)      = axisId;
    IEC_VAL(stop.DECELERATION)= 50.0f;
    __mcl_cmd_Stop(&stop);
 
    stopDoneCycle = 0;
    for (step = 0; step < 5000; step++) {
        __HydMotion_framework_Publish();
        IEC_VAL(ma.EXECUTE)     = true;
        ma.EXECUTE0.value       = true;
        __mcl_cmd_MoveAbsolute(&ma);
        IEC_VAL(stop.EXECUTE)   = true;
        stop.EXECUTE0.value     = true;
        __mcl_cmd_Stop(&stop);
        if (IEC_VAL(stop.DONE)) { stopDoneCycle = step + 1; break; }
    }
    printf("  Second Stop.DONE at cycle %d\n", stopDoneCycle);
    CHECK(stopDoneCycle > 0, "Second Stop should also reach DONE (FB reusable)");
}
 
/* ======================================================================
 * main
 * ====================================================================== */
int main(void) {
    printf("==========================================================\n");
    printf("  Stop + MoveAbsolute Interaction Test Suite\n");
    printf("  Converted from PLC TESTMOTION step logic\n");
    printf("==========================================================\n");
 
    __HydMotion_framework_Init();
    test_stop_during_moveabsolute();
 
    __HydMotion_framework_Init();
    test_full_plc_step_cycle();
 
    __HydMotion_framework_Init();
    test_stop_reusable();
 
    printf("\n==========================================================\n");
    printf("  Results: %d/%d passed\n", g_tests_passed, g_tests_run);
    printf("==========================================================\n");
 
    return (g_tests_passed == g_tests_run) ? 0 : 1;
}