/**
 * @file test_recipe_multi_segment_ownership.c
 * @brief MoveProfile ownership stability across recipe advance (Sprint 0 spec C-2)
 *
 * Background:
 *   The IEC FB ownership tracking uses an execution-id epoch. Previously the
 *   same epoch (_executionId) advanced on every HYD_BeginSegment, including
 *   recipe NextSegment. So the IEC adapter's recipeExecutionLostOwnership()
 *   saw an id mismatch on every recipe advance and falsely raised
 *   COMMANDABORTED on the outer MoveProfile FB instance.
 *
 *   Fix: split ownership into per-segment epoch (_executionId, direct path)
 *   and per-recipe batch epoch (_recipeBatchId, MoveProfile path). Recipe
 *   NextSegment advances _executionId but NOT _recipeBatchId.
 *
 * Test 1: 3-segment recipe; manually call NextSegment after each segment
 *         completes. MoveProfile must NEVER raise COMMANDABORTED. After all
 *         three segments finish, DONE must be true.
 *
 * Test 2: 1-segment recipe; rising-edge MoveProfile, then issue Stop. The
 *         next MoveProfile scan MUST raise COMMANDABORTED (real takeover).
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <stdbool.h>

#include "motion_interface.h"
#include "motion_control.h"
#include "action_profile.h"

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
 * Test 1: Multi-segment recipe — NextSegment must NOT raise COMMANDABORTED
 *         on the outer MoveProfile FB.
 *
 * Setup:
 *   - Create simulation axis with USE_RECIPE=true
 *   - Pre-load a 3-segment recipe (clamp close 30/60/100mm) directly into
 *     the core FB (bypassing IEC) so MoveProfile's rising-edge sees a
 *     multi-segment recipe rather than building one from MOTION.
 *   - Rising edge MoveProfile -> starts segment 0
 *   - Spin Publish + steady-state MoveProfile until SEGMENT_COMPLETE,
 *     then call NextSegment manually. Repeat for segments 1, 2.
 *
 * Pre-fix expected: FAIL (the very first NextSegment bumps _executionId,
 * the adapter sees mismatch and raises COMMANDABORTED).
 * Post-fix expected: PASS.
 * ==================================================================== */
static void test_moveprofile_survives_recipe_advance(void) {
    HYD_CREATEMOTION cm;
    HYD_MOVEPROFILE mp;
    HYD_AXISMOTION motion;
    HYD_MotionSegment seg[3];
    HYD_MotionControlFB* fb;
    int step;
    int i;
    int axisIndex;
    int nextSegmentCount = 0;

    printf("Running: test_moveprofile_survives_recipe_advance\n");

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

    /* Pre-load a 3-segment recipe directly. MoveProfile's execRising path
     * will see RECIPE_SIZE != 0 and skip the 1-segment build-from-MOTION. */
    for (i = 0; i < 3; i++) {
        build_clamp_close_segment(&seg[i], (HYD_UINT8)(i + 1), (HYD_REAL)((i + 1) * 30));
    }
    ASSERT_TRUE(HYD_MotionControlFB_LoadRecipe(fb, seg, 3),
                "LoadRecipe with 3 segments should succeed");

    /* Rising edge: MoveProfile uses the preloaded recipe. */
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

    /* Spin: steady-state MoveProfile + Publish, calling NextSegment when the
     * core FB reports SEGMENT_COMPLETE. Each scan, assert COMMANDABORTED
     * stayed false (this is the bug under test). Bound the loop to 4000
     * iterations to prevent run-away. */
    for (step = 1; step < 4000; step++) {
        IEC_VAL(mp.EXECUTE) = true;
        mp.EXECUTE0.value = true;
        __mcl_cmd_MoveProfile(&mp);

        if (IEC_VAL(mp.COMMANDABORTED)) {
            printf("  FAIL: MoveProfile COMMANDABORTED at step %d, fb->FB_STATE=%d, "
                   "currentSegmentIndex=%u, nextSegmentCount=%d\n",
                   step, (int)fb->FB_STATE,
                   (unsigned)fb->STATE.currentSegmentIndex, nextSegmentCount);
            tests_run++;
            return;
        }

        if (fb->FB_STATE == HYD_FB_STATE_SEGMENT_COMPLETE) {
            HYD_MotionControlFB_NextSegment(fb, fb->AXIS_REF.timestamp);
            nextSegmentCount++;
        }

        if (HYD_MotionControlFB_IsDone(fb) && fb->STATE.finished) {
            break;
        }

        __HydMotion_framework_Publish();
    }

    /* Drive one more scan so MoveProfile latches the DONE state. */
    IEC_VAL(mp.EXECUTE) = true;
    mp.EXECUTE0.value = true;
    __mcl_cmd_MoveProfile(&mp);

    ASSERT_TRUE(IEC_VAL(mp.COMMANDABORTED) == false,
                "MoveProfile must NOT report COMMANDABORTED after multi-segment recipe");
    ASSERT_TRUE(IEC_VAL(mp.DONE) == true,
                "MoveProfile must report DONE after all 3 segments complete");
    ASSERT_TRUE(nextSegmentCount >= 2,
                "Should have invoked NextSegment at least twice (3 segments => 2 advances)");
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
    test_moveprofile_survives_recipe_advance();
    test_moveprofile_aborted_by_stop_takeover();

    printf("\n[recipe_multi_segment_ownership] %d/%d assertions passed\n",
           tests_passed, tests_run);
    return (tests_run == tests_passed) ? 0 : 1;
}
