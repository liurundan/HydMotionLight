/**
 * 精准探针: 对比同扫描触发 vs 延迟触发场景下 FB2.Active / FB2.Done 的行为
 *
 * 场景A: FB1 和 FB2 在同一扫描内先后 Execute:=TRUE (前后脚 – 现场真实情况)
 * 场景B: FB2 在 FB1.Active 变 true 后的下一扫描才 Execute:=TRUE (当前测试的触发方式)
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>

#include "motion_interface.h"
#include "motion_control.h"

extern HYD_MotionControlFB* __MK_GetPublic_MotionControlFB(int index);

#define IEC_VAL(v) ((v).value)
#define MAX_STEPS  60000

static int alloc_axis(void)
{
    HYD_CREATEMOTION cm;
    memset(&cm, 0, sizeof(cm));
    IEC_VAL(cm.EN)              = true;
    IEC_VAL(cm.USE_RECIPE)      = false;
    IEC_VAL(cm.FLOW_TO_PUMPSPEED) = 1.0f;
    IEC_VAL(cm.PUMPSPEED_LIMIT) = 3000.0f;
    IEC_VAL(cm.USE_SIMULATION)  = true;
    __mcl_cmd_CreateMotion(&cm);
    return (int)IEC_VAL(cm.AXISID);
}

static void init_ma(HYD_MOVEABSOLUTE* ma, int ax,
                    float pos, float vel, float acc, int bm)
{
    memset(ma, 0, sizeof(*ma));
    IEC_VAL(ma->EN)          = true;
    IEC_VAL(ma->AXISID)      = ax;
    IEC_VAL(ma->POSITION)    = pos;
    IEC_VAL(ma->VELOCITY)    = vel;
    IEC_VAL(ma->ACCELERATION) = acc;
    IEC_VAL(ma->DECELERATION) = acc;
    IEC_VAL(ma->DIRECTION)   = 1;   /* POSITIVE/EXTEND */
    IEC_VAL(ma->BUFFERMODE)  = bm;
    /* EXECUTE0 初始为 false —— 下一次调用产生 rising edge */
}

/* -----------------------------------------------------------------------
 * 场景A: 同一扫描内 FB1 rising-edge 后紧跟 FB2 rising-edge
 * 这是现场 "前后脚置TRUE" 的精确复现
 * ----------------------------------------------------------------------- */
static void run_scenario_A_same_scan(void)
{
    HYD_MOVEABSOLUTE fb1, fb2;
    int axisId;
    int fb2_active_scan = -1;
    int fb2_done_scan   = -1;

    __HydMotion_framework_Init();
    axisId = alloc_axis();
    if (axisId < 0) { printf("alloc failed\n"); return; }

    init_ma(&fb1, axisId, 100.0f,  5.0f, 50.0f, HYD_BUFFER_MODE_ABORT);
    init_ma(&fb2, axisId, 200.0f, 20.0f, 50.0f, HYD_BUFFER_MODE_BLENDING_HIGH);

    printf("=== 场景A: 同一扫描 FB1+FB2 rising edge ===\n");

    /* 扫描1: FB1 rising edge → FB2 rising edge → Publish */
    IEC_VAL(fb1.EXECUTE) = true;  fb1.EXECUTE0.value = false;
    __mcl_cmd_MoveAbsolute(&fb1);
    fb1.EXECUTE0.value = true;   /* 下次不再触发 */

    IEC_VAL(fb2.EXECUTE) = true;  fb2.EXECUTE0.value = false;
    __mcl_cmd_MoveAbsolute(&fb2);
    fb2.EXECUTE0.value = true;

    __HydMotion_framework_Publish();

    printf("  扫描1后: FB1.Active=%d FB1.Busy=%d  FB2.Active=%d FB2.Busy=%d FB2._PENDING=%d\n",
           IEC_VAL(fb1.ACTIVE), IEC_VAL(fb1.BUSY),
           IEC_VAL(fb2.ACTIVE), IEC_VAL(fb2.BUSY),
           IEC_VAL(fb2._PENDING));

    HYD_MotionControlFB* core = __MK_GetPublic_MotionControlFB(axisId);
    if (core) {
        printf("  Core: STATE.active=%d _directPendingValid=%d _executionId=%u _directOwnerTicket=%u\n",
               core->STATE.active, core->_directPendingValid,
               (unsigned)core->_executionId, (unsigned)core->_directOwnerTicket);
    }

    /* 持续扫描直到 FB2.Done 或超时 */
    for (int s = 0; s < MAX_STEPS; s++) {
        IEC_VAL(fb1.EXECUTE) = true;  fb1.EXECUTE0.value = true;
        __mcl_cmd_MoveAbsolute(&fb1);

        IEC_VAL(fb2.EXECUTE) = true;  fb2.EXECUTE0.value = true;
        __mcl_cmd_MoveAbsolute(&fb2);

        __HydMotion_framework_Publish();

        if (fb2_active_scan < 0 && IEC_VAL(fb2.ACTIVE)) {
            fb2_active_scan = s + 2;
        }
        if (fb2_done_scan < 0 && IEC_VAL(fb2.DONE)) {
            fb2_done_scan = s + 2;
            break;
        }
        if (IEC_VAL(fb2.COMMANDABORTED) || IEC_VAL(fb2.ERROR)) {
            printf("  [扫描%d] FB2 abort=%d error=%d\n",
                   s + 2, IEC_VAL(fb2.COMMANDABORTED), IEC_VAL(fb2.ERROR));
            break;
        }
        /* 每隔 5000 扫描打一行状态 */
        if ((s % 5000) == 4999) {
            printf("  [扫描%d] pos=%.1f FB1.Done=%d FB2.Active=%d FB2.Done=%d FB2._PENDING=%d\n",
                   s+2, core ? core->AXIS_REF.position : -1.0f,
                   IEC_VAL(fb1.DONE), IEC_VAL(fb2.ACTIVE), IEC_VAL(fb2.DONE),
                   IEC_VAL(fb2._PENDING));
        }
    }

    if (fb2_active_scan < 0) {
        printf("  FAIL: FB2.Active 从未变 true\n");
    } else {
        printf("  OK:   FB2.Active 首次 true 在扫描 %d\n", fb2_active_scan);
    }
    if (fb2_done_scan < 0) {
        printf("  FAIL: FB2.Done 从未变 true");
        if (core) printf(" (最终位置 %.2f mm)", core->AXIS_REF.position);
        printf("\n");
    } else {
        printf("  OK:   FB2.Done  首次 true 在扫描 %d\n", fb2_done_scan);
    }
}

/* -----------------------------------------------------------------------
 * 场景B: FB2 在 FB1 变 active 之后的 *下一扫描* 才触发
 * 这是当前 trigger_fb2_when_fb1_active() 的触发方式
 * ----------------------------------------------------------------------- */
static void run_scenario_B_delayed(void)
{
    HYD_MOVEABSOLUTE fb1, fb2;
    int axisId;
    int fb2_active_scan = -1;
    int fb2_done_scan   = -1;

    __HydMotion_framework_Init();
    axisId = alloc_axis();
    if (axisId < 0) { printf("alloc failed\n"); return; }

    init_ma(&fb1, axisId, 100.0f,  5.0f, 50.0f, HYD_BUFFER_MODE_ABORT);

    printf("\n=== 场景B: FB2 在 FB1.Active 后的下一扫描触发 ===\n");

    /* 扫描1: 仅 FB1 rising edge */
    IEC_VAL(fb1.EXECUTE) = true;  fb1.EXECUTE0.value = false;
    __mcl_cmd_MoveAbsolute(&fb1);
    fb1.EXECUTE0.value = true;
    __HydMotion_framework_Publish();
    printf("  扫描1后(仅FB1): FB1.Active=%d\n", IEC_VAL(fb1.ACTIVE));

    /* 扫描2: FB1 steady + FB2 rising edge */
    init_ma(&fb2, axisId, 200.0f, 20.0f, 50.0f, HYD_BUFFER_MODE_BLENDING_HIGH);
    IEC_VAL(fb1.EXECUTE) = true;  fb1.EXECUTE0.value = true;
    __mcl_cmd_MoveAbsolute(&fb1);
    IEC_VAL(fb2.EXECUTE) = true;  fb2.EXECUTE0.value = false;
    __mcl_cmd_MoveAbsolute(&fb2);
    fb2.EXECUTE0.value = true;
    __HydMotion_framework_Publish();
    printf("  扫描2后: FB1.Active=%d  FB2.Active=%d FB2._PENDING=%d\n",
           IEC_VAL(fb1.ACTIVE), IEC_VAL(fb2.ACTIVE), IEC_VAL(fb2._PENDING));

    HYD_MotionControlFB* core = __MK_GetPublic_MotionControlFB(axisId);

    for (int s = 0; s < MAX_STEPS; s++) {
        IEC_VAL(fb1.EXECUTE) = true;  fb1.EXECUTE0.value = true;
        __mcl_cmd_MoveAbsolute(&fb1);

        IEC_VAL(fb2.EXECUTE) = true;  fb2.EXECUTE0.value = true;
        __mcl_cmd_MoveAbsolute(&fb2);

        __HydMotion_framework_Publish();

        if (fb2_active_scan < 0 && IEC_VAL(fb2.ACTIVE)) {
            fb2_active_scan = s + 3;
        }
        if (fb2_done_scan < 0 && IEC_VAL(fb2.DONE)) {
            fb2_done_scan = s + 3;
            break;
        }
        if (IEC_VAL(fb2.COMMANDABORTED) || IEC_VAL(fb2.ERROR)) {
            printf("  [扫描%d] FB2 abort=%d error=%d\n",
                   s + 3, IEC_VAL(fb2.COMMANDABORTED), IEC_VAL(fb2.ERROR));
            break;
        }
    }

    if (fb2_active_scan < 0) {
        printf("  FAIL: FB2.Active 从未变 true\n");
    } else {
        printf("  OK:   FB2.Active 首次 true 在扫描 %d\n", fb2_active_scan);
    }
    if (fb2_done_scan < 0) {
        printf("  FAIL: FB2.Done 从未变 true");
        if (core) printf(" (最终位置 %.2f mm)", core->AXIS_REF.position);
        printf("\n");
    } else {
        printf("  OK:   FB2.Done  首次 true 在扫描 %d\n", fb2_done_scan);
    }
}

int main(void)
{
    printf("=== 探针: MoveAbsolute BLENDING_HIGH FB2 状态输出验证 ===\n\n");
    run_scenario_A_same_scan();
    run_scenario_B_delayed();
    printf("\n完成\n");
    return 0;
}
