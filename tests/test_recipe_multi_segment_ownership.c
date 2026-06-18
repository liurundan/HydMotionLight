/**
 * @file test_recipe_multi_segment_ownership.c
 * @brief HYD_MAX_SEGMENTS=1 guard for recipe rejection and Stop ownership
 *
 * Background:
 *   HYD_MAX_SEGMENTS=1 makes multi-segment recipe ownership coverage
 *   unreachable on this target. This file therefore carries one oversized-
 *   recipe rejection guard plus one surviving single-segment Stop takeover
 *   regression to protect the ownership path that remains reachable.
 *
 * Test 1: build a 3-segment recipe, reject it as RECIPE_TOO_LARGE.
 *
 * Test 2: load one segment, issue Stop, and verify the next MoveProfile scan
 *         still reports COMMANDABORTED for a real takeover.
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <stdbool.h>

#include "motion_interface.h"
#include "motion_control.h"
#include "action_profile.h"
#include "test_recipe_rejection_helpers.h"

extern HYD_MotionControlFB* __MK_GetPublic_MotionControlFB(int index);

#define IEC_VAL(var) ((var).value)

static int tests_run = 0;
static int tests_passed = 0;

#define ASSERT_TRUE(cond, msg) do { \
    tests_run++; \
    if (cond) { tests_passed++; } \
    else { printf("  FAIL: %s\n", msg); } \
} while (0)

static void build_clamp_close_segment(HYD_MotionSegment* seg,
                                      HYD_UINT8 tag,
                                      HYD_REAL targetPos) {
    HYD_MotionFBParams params;
    memset(&params, 0, sizeof(params));
    params.maxFlow = 50.0;
    params.maxVelocity = 80.0;
    params.maxAcceleration = 500.0;
    params.maxDeceleration = 500.0;
    params.velocityToFlowGain = 0.25;
    params.positionTolerance = 0.5;
    /* Action profile requires a usable params struct; result must succeed. */
    if (!HYD_ActionProfile_BuildClampClose(seg, &params, tag, targetPos)) {
        printf("  FATAL: HYD_ActionProfile_BuildClampClose failed\n");
        exit(1);
    }
}

/* ====================================================================
 * Test 1: Oversized recipes must be rejected up front on this platform.
 *
 * The platform limit is HYD_MAX_SEGMENTS=1, so a 3-segment recipe should
 * fail to load and report RECIPE_TOO_LARGE.
 * ==================================================================== */
static void test_moveprofile_rejects_oversized_recipe_load(void) {
    HYD_CREATEMOTION cm;
    HYD_MotionSegment seg[3];
    HYD_MotionControlFB* fb;
    int i;
    int axisIndex;

    printf("Running: test_moveprofile_rejects_oversized_recipe_load\n");

    __HydMotion_framework_Init();

    /* Allocate one simulation axis in recipe mode. */
    memset(&cm, 0, sizeof(cm));
    IEC_VAL(cm.EN) = true;
    IEC_VAL(cm.USE_RECIPE) = true;
    IEC_VAL(cm.FLOW_TO_PUMPSPEED) = 1.0f;
    IEC_VAL(cm.PUMPSPEED_LIMIT) = 5000.0f;
    IEC_VAL(cm.USE_SIMULATION) = true;
    __mcl_cmd_CreateMotion(&cm);
    axisIndex = (int)IEC_VAL(cm.AXISID);
    ASSERT_TRUE(axisIndex >= 0, "CreateMotion should succeed");

    fb = __MK_GetPublic_MotionControlFB(axisIndex);
    ASSERT_TRUE(fb != NULL, "Should fetch axis FB");

    /* Build a 3-segment recipe and confirm the platform rejects it. */
    for (i = 0; i < 3; i++) {
        build_clamp_close_segment(&seg[i], (HYD_UINT8)(i + 1), (HYD_REAL)((i + 1) * 30));
    }
    assert_oversized_recipe_load_rejected(fb, seg, 3U);
}

/* ====================================================================
 * Test 2: Real takeover via Stop must STILL raise COMMANDABORTED on
 *         the outer MoveProfile (regression guard so we don't over-fix).
 * ==================================================================== */
static void test_moveprofile_aborted_by_stop_takeover(void) {
    HYD_CREATEMOTION cm;
    HYD_MOVEPROFILE mp;
    HYD_STOP stop;
    HYD_AXISMOTION motion;
    HYD_MotionSegment seg;
    HYD_MotionControlFB* fb;
    int axisIndex;
    int step;

    printf("Running: test_moveprofile_aborted_by_stop_takeover\n");

    __HydMotion_framework_Init();

    memset(&cm, 0, sizeof(cm));
    IEC_VAL(cm.EN) = true;
    IEC_VAL(cm.USE_RECIPE) = true;
    IEC_VAL(cm.FLOW_TO_PUMPSPEED) = 1.0f;
    IEC_VAL(cm.PUMPSPEED_LIMIT) = 5000.0f;
    IEC_VAL(cm.USE_SIMULATION) = true;
    __mcl_cmd_CreateMotion(&cm);
    axisIndex = (int)IEC_VAL(cm.AXISID);

    fb = __MK_GetPublic_MotionControlFB(axisIndex);
    ASSERT_TRUE(fb != NULL, "Should fetch axis FB");

    build_clamp_close_segment(&seg, 1, 200.0);
    ASSERT_TRUE(HYD_MotionControlFB_LoadRecipe(fb, &seg, 1),
                "LoadRecipe with 1 segment should succeed");

    memset(&mp, 0, sizeof(mp));
    memset(&motion, 0, sizeof(motion));
    IEC_VAL(mp.EN) = true;
    IEC_VAL(mp.EXECUTE) = true;
    mp.EXECUTE0.value = false;
    IEC_VAL(mp.AXISID) = axisIndex;
    IEC_VAL(mp.BUFFERMODE) = HYD_BUFFER_MODE_ABORT;
    motion.TIMESTAMP = 0.0f;
    __SET_VAR(mp., MOTION, , motion);

    __mcl_cmd_MoveProfile(&mp);
    __HydMotion_framework_Publish();

    /* Run a few cycles so the FB enters RUNNING state. */
    for (step = 1; step < 20; step++) {
        IEC_VAL(mp.EXECUTE) = true;
        mp.EXECUTE0.value = true;
        __mcl_cmd_MoveProfile(&mp);
        __HydMotion_framework_Publish();
    }

    ASSERT_TRUE(IEC_VAL(mp.ACTIVE) || IEC_VAL(mp.BUSY),
                "MoveProfile should be active/busy before Stop");

    /* Stop preempts. */
    memset(&stop, 0, sizeof(stop));
    IEC_VAL(stop.EN) = true;
    IEC_VAL(stop.EXECUTE) = true;
    stop.EXECUTE0.value = false;
    IEC_VAL(stop.AXISID) = axisIndex;
    IEC_VAL(stop.DECELERATION) = 200.0f;
    __mcl_cmd_Stop(&stop);
    __HydMotion_framework_Publish();

    /* Next MoveProfile scan must see COMMANDABORTED. */
    IEC_VAL(mp.EXECUTE) = true;
    mp.EXECUTE0.value = true;
    __mcl_cmd_MoveProfile(&mp);

    ASSERT_TRUE(IEC_VAL(mp.COMMANDABORTED) == true,
                "MoveProfile should report COMMANDABORTED after Stop takeover");
    ASSERT_TRUE(IEC_VAL(mp.ACTIVE) == false,
                "MoveProfile should clear ACTIVE after Stop takeover");
    ASSERT_TRUE(IEC_VAL(mp.BUSY) == false,
                "MoveProfile should clear BUSY after Stop takeover");
}

int main(void) {
    test_moveprofile_rejects_oversized_recipe_load();
    test_moveprofile_aborted_by_stop_takeover();

    printf("\n[recipe_multi_segment_ownership] %d/%d assertions passed\n",
           tests_passed, tests_run);
    return (tests_run == tests_passed) ? 0 : 1;
}
