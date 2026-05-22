/* tests/test_pump_direction_conflict.c
 * Sprint 3 §3.6 — single-pump direction conflict detection.
 */

#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "motion_interface.h"
#include "motion_control.h"

extern HYD_MotionControlFB* __MK_GetPublic_MotionControlFB(int index);

#define IEC_VAL(var) ((var).value)

static void ensure_axes_allocated(int count) {
    for (int i = 0; i < count; i++) {
        HYD_CREATEMOTION cm;
        memset(&cm, 0, sizeof(cm));
        IEC_VAL(cm.EN) = true;
        IEC_VAL(cm.USE_RECIPE) = false;
        IEC_VAL(cm.FLOW_TO_PUMPSPEED) = 20.0f;
        IEC_VAL(cm.PUMPSPEED_LIMIT) = 3000.0f;
        IEC_VAL(cm.USE_SIMULATION) = false;
        __mcl_cmd_CreateMotion(&cm);
    }
}

static void test_no_conflict_when_directions_match(void) {
    HYD_GETPUMPREQUEST req;
    HYD_MotionControlFB* fb0;
    HYD_MotionControlFB* fb1;

    printf("Testing GetPumpRequest reports no conflict for matching directions...\n");

    __HydMotion_framework_Init();
    ensure_axes_allocated(2);
    fb0 = __MK_GetPublic_MotionControlFB(0);
    fb1 = __MK_GetPublic_MotionControlFB(1);
    assert(fb0 != NULL && fb1 != NULL);

    fb0->STATE.active = true;
    fb0->STATE.plannedDirection = HYD_DIRECTION_EXTEND;
    fb0->PUMP_SPEED = 1000.0;
    fb1->STATE.active = true;
    fb1->STATE.plannedDirection = HYD_DIRECTION_EXTEND;
    fb1->PUMP_SPEED = 1500.0;

    memset(&req, 0, sizeof(req));
    IEC_VAL(req.ENABLE) = true;
    __mcl_cmd_GetPumpRequest(&req);

    assert(IEC_VAL(req.CONFLICT) == false);
    assert(IEC_VAL(req.PUMPSPEED) > 1499.0f);  /* max of two */
    printf("  No-conflict matching-directions test passed\n");
}

static void test_conflict_detected_when_directions_oppose(void) {
    HYD_GETPUMPREQUEST req;
    HYD_MotionControlFB* fb0;
    HYD_MotionControlFB* fb1;

    printf("Testing GetPumpRequest reports conflict for opposing directions...\n");

    __HydMotion_framework_Init();
    ensure_axes_allocated(2);
    fb0 = __MK_GetPublic_MotionControlFB(0);
    fb1 = __MK_GetPublic_MotionControlFB(1);

    fb0->STATE.active = true;
    fb0->STATE.plannedDirection = HYD_DIRECTION_EXTEND;
    fb0->PUMP_SPEED = 1000.0;
    fb1->STATE.active = true;
    fb1->STATE.plannedDirection = HYD_DIRECTION_RETRACT;
    fb1->PUMP_SPEED = 1500.0;

    memset(&req, 0, sizeof(req));
    IEC_VAL(req.ENABLE) = true;
    __mcl_cmd_GetPumpRequest(&req);

    assert(IEC_VAL(req.CONFLICT) == true);
    /* Even on conflict, PUMPSPEED still reports the max — PLC decides action */
    assert(IEC_VAL(req.PUMPSPEED) > 1499.0f);
    printf("  Conflict opposing-directions test passed\n");
}

static void test_no_conflict_when_some_axes_hold(void) {
    HYD_GETPUMPREQUEST req;
    HYD_MotionControlFB* fb0;
    HYD_MotionControlFB* fb1;
    HYD_MotionControlFB* fb2;

    printf("Testing GetPumpRequest ignores HOLD/AUTO axes for conflict...\n");

    __HydMotion_framework_Init();
    ensure_axes_allocated(3);
    fb0 = __MK_GetPublic_MotionControlFB(0);
    fb1 = __MK_GetPublic_MotionControlFB(1);
    fb2 = __MK_GetPublic_MotionControlFB(2);

    fb0->STATE.active = true;
    fb0->STATE.plannedDirection = HYD_DIRECTION_EXTEND;
    fb0->PUMP_SPEED = 800.0;
    fb1->STATE.active = true;
    fb1->STATE.plannedDirection = HYD_DIRECTION_HOLD;
    fb1->PUMP_SPEED = 200.0;
    fb2->STATE.active = true;
    fb2->STATE.plannedDirection = HYD_DIRECTION_AUTO;
    fb2->PUMP_SPEED = 500.0;

    memset(&req, 0, sizeof(req));
    IEC_VAL(req.ENABLE) = true;
    __mcl_cmd_GetPumpRequest(&req);

    assert(IEC_VAL(req.CONFLICT) == false);
    printf("  HOLD/AUTO conflict-ignore test passed\n");
}

int main(void) {
    printf("Running pump-direction-conflict tests...\n\n");
    test_no_conflict_when_directions_match();
    test_conflict_detected_when_directions_oppose();
    test_no_conflict_when_some_axes_hold();
    printf("\nAll pump-direction-conflict tests passed.\n");
    return 0;
}
