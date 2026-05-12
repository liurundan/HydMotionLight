/**
 * Bug reproduction: MoveAbsolute → Stop, Stop cannot achieve deceleration
 *
 * Detailed debugging: trace every step of the Stop deceleration process
 * to find where the logic breaks.
 */

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdbool.h>

#include "motion_interface.h"
#include "motion_control.h"

extern HYD_MotionControlFB* __MK_GetPublic_MotionControlFB(int index);

#define IEC_VAL(var) ((var).value)

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

/**
 * Key test: MoveAbsolute running → Stop
 * Trace every step to find the bug
 */
static void test_stop_trace_detailed(void) {
    HYD_MOVEABSOLUTE ma;
    HYD_STOP stop;
    int axisId, step;

    printf("=== Detailed Stop Deceleration Trace ===\n\n");

    __HydMotion_framework_Init();
    axisId = create_sim_axis();

    /* Start MoveAbsolute */
    memset(&ma, 0, sizeof(ma));
    IEC_VAL(ma.EN) = true;
    IEC_VAL(ma.AXISID) = axisId;
    IEC_VAL(ma.POSITION) = 100.0f;
    IEC_VAL(ma.VELOCITY) = 50.0f;
    IEC_VAL(ma.ACCELERATION) = 200.0f;
    IEC_VAL(ma.DIRECTION) = 1;
    IEC_VAL(ma.BUFFERMODE) = 1;

    IEC_VAL(ma.EXECUTE) = true;
    ma.EXECUTE0.value = false;
    __mcl_cmd_MoveAbsolute(&ma);

    /* Build velocity for 30 cycles */
    for (step = 0; step < 30; step++) {
        __HydMotion_framework_Publish();
        IEC_VAL(ma.EXECUTE) = true;
        ma.EXECUTE0.value = true;
        __mcl_cmd_MoveAbsolute(&ma);
    }

    {
        HYD_MotionControlFB* fb = __MK_GetPublic_MotionControlFB(axisId);
        printf("Before Stop: vel=%.4f, pos=%.4f, FB_STATE=%d\n",
               (double)fb->AXIS_REF.velocity, (double)fb->AXIS_REF.position, fb->FB_STATE);
    }

    /* Issue Stop (rising edge) */
    memset(&stop, 0, sizeof(stop));
    IEC_VAL(stop.EN) = true;
    IEC_VAL(stop.EXECUTE) = true;
    stop.EXECUTE0.value = false;
    IEC_VAL(stop.AXISID) = axisId;
    __mcl_cmd_Stop(&stop);

    {
        HYD_MotionControlFB* fb = __MK_GetPublic_MotionControlFB(axisId);
        printf("After Stop execRising: _isStopping=%d, _stopStartVel=%.4f, FB_STATE=%d, DONE=%d, BUSY=%d\n",
               fb->_isStopping, (double)fb->_stopStartVel, fb->FB_STATE,
               (int)IEC_VAL(stop.DONE), (int)IEC_VAL(stop.BUSY));
    }

    /* Trace deceleration process step by step */
    for (step = 0; step < 500; step++) {
        __HydMotion_framework_Publish();
        IEC_VAL(stop.EXECUTE) = true;
        stop.EXECUTE0.value = true;
        __mcl_cmd_Stop(&stop);

        {
            HYD_MotionControlFB* fb = __MK_GetPublic_MotionControlFB(axisId);
            if (step < 10 || step % 50 == 0 || IEC_VAL(stop.DONE) || fb->FB_STATE == HYD_FB_STATE_DONE) {
                printf("  Step %3d: vel=%.4f, pumpSpeed=%.4f, _isStopping=%d, FB_STATE=%d, Stop.DONE=%d, Stop.BUSY=%d\n",
                       step,
                       (double)fb->AXIS_REF.velocity,
                       (double)fb->PUMP_SPEED,
                       fb->_isStopping,
                       fb->FB_STATE,
                       (int)IEC_VAL(stop.DONE),
                       (int)IEC_VAL(stop.BUSY));
            }

            if (IEC_VAL(stop.DONE)) {
                printf("  >>> Stop.DONE reached at step %d <<<\n", step);
                break;
            }
        }
    }

    /* Final state check */
    {
        HYD_MotionControlFB* fb = __MK_GetPublic_MotionControlFB(axisId);
        printf("\nFinal: vel=%.4f, FB_STATE=%d, _isStopping=%d\n",
               (double)fb->AXIS_REF.velocity, fb->FB_STATE, fb->_isStopping);
    }
}

/**
 * Test: Stop after MoveAbsolute where we also call MoveAbsolute each cycle
 * This simulates the PLC pattern where both FBs are called every scan
 */
static void test_stop_with_ma_also_called(void) {
    HYD_MOVEABSOLUTE ma;
    HYD_STOP stop;
    int axisId, step;

    printf("\n=== Stop with MoveAbsolute also called each cycle ===\n\n");

    __HydMotion_framework_Init();
    axisId = create_sim_axis();

    /* Start MoveAbsolute */
    memset(&ma, 0, sizeof(ma));
    IEC_VAL(ma.EN) = true;
    IEC_VAL(ma.AXISID) = axisId;
    IEC_VAL(ma.POSITION) = 100.0f;
    IEC_VAL(ma.VELOCITY) = 50.0f;
    IEC_VAL(ma.ACCELERATION) = 200.0f;
    IEC_VAL(ma.DIRECTION) = 1;
    IEC_VAL(ma.BUFFERMODE) = 1;

    IEC_VAL(ma.EXECUTE) = true;
    ma.EXECUTE0.value = false;
    __mcl_cmd_MoveAbsolute(&ma);

    for (step = 0; step < 30; step++) {
        __HydMotion_framework_Publish();
        IEC_VAL(ma.EXECUTE) = true;
        ma.EXECUTE0.value = true;
        __mcl_cmd_MoveAbsolute(&ma);
    }

    /* Issue Stop */
    memset(&stop, 0, sizeof(stop));
    IEC_VAL(stop.EN) = true;
    IEC_VAL(stop.EXECUTE) = true;
    stop.EXECUTE0.value = false;
    IEC_VAL(stop.AXISID) = axisId;
    __mcl_cmd_Stop(&stop);

    /* In PLC, both FBs are called each cycle */
    for (step = 0; step < 500; step++) {
        __HydMotion_framework_Publish();

        /* Call MoveAbsolute (still holding EXECUTE=true) */
        IEC_VAL(ma.EXECUTE) = true;
        ma.EXECUTE0.value = true;
        __mcl_cmd_MoveAbsolute(&ma);

        /* Call Stop (still holding EXECUTE=true) */
        IEC_VAL(stop.EXECUTE) = true;
        stop.EXECUTE0.value = true;
        __mcl_cmd_Stop(&stop);

        if (IEC_VAL(stop.DONE)) {
            printf("  Stop.DONE at step %d\n", step);
            printf("  MoveAbsolute: DONE=%d, COMMANDABORTED=%d, BUSY=%d\n",
                   (int)IEC_VAL(ma.DONE),
                   (int)IEC_VAL(ma.COMMANDABORTED),
                   (int)IEC_VAL(ma.BUSY));
            break;
        }
    }
}

/**
 * Test: MoveAbsolute running → Stop → after DONE, clear EXECUTE → re-trigger
 * Full lifecycle with proper PLCopen signal management
 */
static void test_full_lifecycle_retrigger(void) {
    HYD_MOVEABSOLUTE ma;
    HYD_STOP stop;
    int axisId, step;

    printf("\n=== Full Lifecycle: MA → Stop → Clear → Re-trigger ===\n\n");

    __HydMotion_framework_Init();
    axisId = create_sim_axis();

    /* === Round 1: MoveAbsolute → Stop === */
    printf("--- Round 1 ---\n");

    memset(&ma, 0, sizeof(ma));
    IEC_VAL(ma.EN) = true;
    IEC_VAL(ma.AXISID) = axisId;
    IEC_VAL(ma.POSITION) = 100.0f;
    IEC_VAL(ma.VELOCITY) = 50.0f;
    IEC_VAL(ma.ACCELERATION) = 200.0f;
    IEC_VAL(ma.DIRECTION) = 1;
    IEC_VAL(ma.BUFFERMODE) = 1;
    IEC_VAL(ma.EXECUTE) = true;
    ma.EXECUTE0.value = false;
    __mcl_cmd_MoveAbsolute(&ma);

    for (step = 0; step < 30; step++) {
        __HydMotion_framework_Publish();
        IEC_VAL(ma.EXECUTE) = true;
        ma.EXECUTE0.value = true;
        __mcl_cmd_MoveAbsolute(&ma);
    }

    memset(&stop, 0, sizeof(stop));
    IEC_VAL(stop.EN) = true;
    IEC_VAL(stop.EXECUTE) = true;
    stop.EXECUTE0.value = false;
    IEC_VAL(stop.AXISID) = axisId;
    __mcl_cmd_Stop(&stop);

    for (step = 0; step < 5000; step++) {
        __HydMotion_framework_Publish();
        IEC_VAL(stop.EXECUTE) = true;
        stop.EXECUTE0.value = true;
        __mcl_cmd_Stop(&stop);
        IEC_VAL(ma.EXECUTE) = true;
        ma.EXECUTE0.value = true;
        __mcl_cmd_MoveAbsolute(&ma);

        if (IEC_VAL(stop.DONE)) {
            printf("  Round 1: Stop.DONE at step %d\n", step);
            break;
        }
    }

    /* Clear Stop.EXECUTE */
    IEC_VAL(stop.EXECUTE) = false;
    stop.EXECUTE0.value = true;
    __mcl_cmd_Stop(&stop);

    /* Clear MoveAbsolute.EXECUTE */
    IEC_VAL(ma.EXECUTE) = false;
    ma.EXECUTE0.value = true;
    __mcl_cmd_MoveAbsolute(&ma);

    /* Extra cycle to clear signals */
    __HydMotion_framework_Publish();
    IEC_VAL(stop.EXECUTE) = false;
    stop.EXECUTE0.value = false;
    __mcl_cmd_Stop(&stop);
    IEC_VAL(ma.EXECUTE) = false;
    ma.EXECUTE0.value = false;
    __mcl_cmd_MoveAbsolute(&ma);

    printf("  After clearing: Stop.DONE=%d, MA.DONE=%d, MA.COMMANDABORTED=%d\n",
           (int)IEC_VAL(stop.DONE), (int)IEC_VAL(ma.DONE), (int)IEC_VAL(ma.COMMANDABORTED));

    /* === Round 2: Re-trigger MoveAbsolute === */
    printf("--- Round 2 ---\n");

    memset(&ma, 0, sizeof(ma));
    IEC_VAL(ma.EN) = true;
    IEC_VAL(ma.AXISID) = axisId;
    IEC_VAL(ma.POSITION) = 200.0f;
    IEC_VAL(ma.VELOCITY) = 50.0f;
    IEC_VAL(ma.ACCELERATION) = 200.0f;
    IEC_VAL(ma.DIRECTION) = 1;
    IEC_VAL(ma.BUFFERMODE) = 1;
    IEC_VAL(ma.EXECUTE) = true;
    ma.EXECUTE0.value = false;
    __mcl_cmd_MoveAbsolute(&ma);

    {
        HYD_MotionControlFB* fb = __MK_GetPublic_MotionControlFB(axisId);
        printf("  After re-trigger: FB_STATE=%d, _pendingCommand=%d, ERROR=%d, COMMANDABORTED=%d\n",
               fb->FB_STATE, fb->_pendingCommand,
               (int)IEC_VAL(ma.ERROR), (int)IEC_VAL(ma.COMMANDABORTED));
    }

    for (step = 0; step < 30; step++) {
        __HydMotion_framework_Publish();
        IEC_VAL(ma.EXECUTE) = true;
        ma.EXECUTE0.value = true;
        __mcl_cmd_MoveAbsolute(&ma);
    }

    {
        HYD_MotionControlFB* fb = __MK_GetPublic_MotionControlFB(axisId);
        printf("  After 30 cycles: vel=%.4f, FB_STATE=%d\n",
               (double)fb->AXIS_REF.velocity, fb->FB_STATE);
    }
}

int main(void) {
    test_stop_trace_detailed();
    test_stop_with_ma_also_called();
    test_full_lifecycle_retrigger();
    return 0;
}
