/**
 * @file test_moveabsolute_blending_done.c
 * @brief 验证两个 MoveAbsolute FB 的速度平滑切换和最后一个 FB 的 Done 信号
 *
 * 测试场景:
 *   FB1: velocity=5, position=100, bufferMode=ABORT
 *   FB2: velocity=20, position=200, bufferMode=BLENDING_HIGH (在FB1运行中提交)
 *
 * 预期行为 (PLCopen BLENDING_HIGH 语义):
 *   1. FB1 在位置~100 处切换时速度不为零（平滑过渡，无顿挫）
 *   2. FB1 在切换后输出 DONE=true，不输出 COMMANDABORTED
 *   3. FB2 在到达位置200后输出 DONE=true，不输出 COMMANDABORTED
 *   4. 以上行为在重复循环中稳定复现
 */

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdbool.h>

#include "motion_interface.h"
#include "motion_control.h"

extern HYD_MotionControlFB* __MK_GetPublic_MotionControlFB(int index);

#define IEC_VAL(var) ((var).value)
#define MAX_SIM_STEPS 50000

static int tests_run    = 0;
static int tests_passed = 0;
static int tests_failed = 0;

#define ASSERT_TRUE(cond, msg) do { \
    tests_run++; \
    if (cond) { tests_passed++; } \
    else { tests_failed++; printf("  FAIL [line %d]: %s\n", __LINE__, msg); } \
} while (0)

/* ── axis allocation ─────────────────────────────────────────────── */
static int alloc_sim_axis(void) {
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

/* ── init a MoveAbsolute struct ──────────────────────────────────── */
static void init_ma(HYD_MOVEABSOLUTE* ma, int axisId,
                    float pos, float vel, float acc, int bufMode) {
    memset(ma, 0, sizeof(*ma));
    IEC_VAL(ma->EN)          = true;
    IEC_VAL(ma->AXISID)      = axisId;
    IEC_VAL(ma->POSITION)    = pos;
    IEC_VAL(ma->VELOCITY)    = vel;
    IEC_VAL(ma->ACCELERATION) = acc;
    IEC_VAL(ma->DECELERATION) = acc;
    IEC_VAL(ma->DIRECTION)   = 1;   /* POSITIVE */
    IEC_VAL(ma->BUFFERMODE)  = bufMode;
}

/* ── trigger EXECUTE rising edge ─────────────────────────────────── */
static void rising_edge(HYD_MOVEABSOLUTE* ma) {
    IEC_VAL(ma->EXECUTE)  = true;
    ma->EXECUTE0.value    = false;
    __mcl_cmd_MoveAbsolute(ma);
    /* next calls: EXECUTE stays true, EXECUTE0 follows */
    ma->EXECUTE0.value = true;
}

static void hold_true_scan(HYD_MOVEABSOLUTE* ma) {
    IEC_VAL(ma->EXECUTE) = true;
    ma->EXECUTE0.value = true;
    __mcl_cmd_MoveAbsolute(ma);
}

static int trigger_fb2_when_fb1_active(HYD_MOVEABSOLUTE* fb1,
                                       HYD_MOVEABSOLUTE* fb2,
                                       int maxWaitScans) {
    for (int step = 0; step < maxWaitScans; step++) {
        __HydMotion_framework_Publish();
        hold_true_scan(fb1);
        if (IEC_VAL(fb1->ACTIVE)) {
            rising_edge(fb2);
            return step + 1;
        }
    }
    return -1;
}

static HYD_BOOL velocity_overwritten_before_cutover(HYD_MotionControlFB* core,
                                                    HYD_REAL switchPosition,
                                                    HYD_REAL forbiddenVelocity,
                                                    HYD_REAL tolerance) {
    if (core == NULL) {
        return false;
    }
    if (core->AXIS_REF.position >= switchPosition - 5.0f) {
        return false;
    }
    return fabs(core->_plannerState.lastTargetVelocity - forbiddenVelocity) <= tolerance;
}

static void assert_pending_blend_does_not_take_over_early(HYD_REAL activeVelocity,
                                                           HYD_REAL pendingVelocity,
                                                           HYD_REAL expectedBlendVelocity,
                                                           int pendingBufferMode,
                                                           const char* modeLabel,
                                                           const char* velocityMessage) {
    HYD_MOVEABSOLUTE fb1, fb2;
    HYD_MotionControlFB* core;
    int axisId;
    int triggerScan;

    __HydMotion_framework_Init();
    axisId = alloc_sim_axis();
    ASSERT_TRUE(axisId >= 0, "alloc_sim_axis should succeed");
    if (axisId < 0) {
        return;
    }

    init_ma(&fb1, axisId, 100.0f, activeVelocity, 50.0f, HYD_BUFFER_MODE_ABORT);
    rising_edge(&fb1);

    init_ma(&fb2, axisId, 200.0f, pendingVelocity, 50.0f, pendingBufferMode);
    triggerScan = trigger_fb2_when_fb1_active(&fb1, &fb2, 20);
    ASSERT_TRUE(triggerScan > 0, "FB2 should be triggered after FB1 becomes ACTIVE");
    if (triggerScan <= 0) {
        return;
    }

    core = __MK_GetPublic_MotionControlFB(axisId);
    ASSERT_TRUE(core != NULL, "Public motion control FB should be available");
    if (core == NULL) {
        return;
    }
    ASSERT_TRUE(IEC_VAL(fb2.BUSY) == true,
               "Buffered FB2 should report BUSY immediately after acceptance");
    ASSERT_TRUE(IEC_VAL(fb2.ACTIVE) == false,
               "Buffered FB2 must not report ACTIVE before cutover");
    ASSERT_TRUE(core->_directBlendContext.active == true,
               "Buffered blend context should be recorded immediately after acceptance");
    ASSERT_TRUE(fabs(core->_directBlendContext.blendVelocity - expectedBlendVelocity) <= 0.001f,
               "Buffered blend context should retain the mode-specific through velocity");
    if (IEC_VAL(fb2.ACTIVE) != false) {
        return;
    }

    for (int step = 0; step < 40; step++) {
        __HydMotion_framework_Publish();
        hold_true_scan(&fb1);
        hold_true_scan(&fb2);

        if (IEC_VAL(fb2.ACTIVE) != false) {
            char message[160];
            snprintf(message,
                     sizeof(message),
                     "FB2 should stay inactive while FB1 is still far from the cutover (%s)",
                     modeLabel);
            ASSERT_TRUE(false, message);
            break;
        }
        if (IEC_VAL(fb1.COMMANDABORTED) != false) {
            char message[160];
            snprintf(message,
                     sizeof(message),
                     "FB1 should not be aborted by buffered %s submission",
                     modeLabel);
            ASSERT_TRUE(false, message);
            break;
        }
        if (velocity_overwritten_before_cutover(core, 100.0f, pendingVelocity, 0.25f)) {
            ASSERT_TRUE(false, velocityMessage);
            break;
        }

        if (core->AXIS_REF.position >= 95.0f) {
            break;
        }
    }
}

/* ── drive both FBs each scan; return scan count or -1 on timeout ── */
static int run_until_fb2_done(HYD_MOVEABSOLUTE* fb1, HYD_MOVEABSOLUTE* fb2,
                               float* vel_at_switch, float switch_pos) {
    *vel_at_switch = -1.0f;

    for (int step = 0; step < MAX_SIM_STEPS; step++) {
        __HydMotion_framework_Publish();

        IEC_VAL(fb1->EXECUTE) = true;
        fb1->EXECUTE0.value   = true;
        __mcl_cmd_MoveAbsolute(fb1);

        IEC_VAL(fb2->EXECUTE) = true;
        fb2->EXECUTE0.value   = true;
        __mcl_cmd_MoveAbsolute(fb2);

        /* Capture velocity just as FB1 hands over (first time DONE fires).
         * Use _plannerState.lastTargetVelocity: ApplySafeOutputs (called during
         * BeginSegment in the blend cutover) zeroes AXIS_REF.velocity in the
         * same Publish() call, so the planner state is the right source here. */
        if (*vel_at_switch < 0.0f && IEC_VAL(fb1->DONE)) {
            HYD_MotionControlFB* core = __MK_GetPublic_MotionControlFB(
                (int)IEC_VAL(fb1->AXISID));
            if (core != NULL) {
                *vel_at_switch = (float)fabs(core->_plannerState.lastTargetVelocity);
            }
        }

        if (IEC_VAL(fb2->DONE)) {
            return step + 1;
        }
        /* Bail out early on any error/abort */
        if (IEC_VAL(fb1->ERROR) || IEC_VAL(fb2->ERROR) ||
            IEC_VAL(fb1->COMMANDABORTED) || IEC_VAL(fb2->COMMANDABORTED)) {
            return -1;
        }
    }
    return -1;
}

/* ================================================================== */
static void test_blending_high_two_moveabsolute_cycles(void) {
    const int CYCLES = 3;
    printf("--- Test: BLENDING_HIGH two MoveAbsolute FBs, %d cycles ---\n", CYCLES);

    __HydMotion_framework_Init();
    int axisId = alloc_sim_axis();
    ASSERT_TRUE(axisId >= 0, "alloc_sim_axis should succeed");

    HYD_MOVEABSOLUTE fb1, fb2;

    for (int cycle = 0; cycle < CYCLES; cycle++) {
        printf("  Cycle %d:\n", cycle + 1);

        /* FB1: v=5, pos=100, ABORT */
        init_ma(&fb1, axisId, 100.0f, 5.0f, 50.0f, HYD_BUFFER_MODE_ABORT);
        rising_edge(&fb1);                /* rising edge + first Publish */
        __HydMotion_framework_Publish();
        IEC_VAL(fb1.EXECUTE) = true;
        fb1.EXECUTE0.value   = true;
        __mcl_cmd_MoveAbsolute(&fb1);

        /* FB2: v=20, pos=200, BLENDING_HIGH — issue while FB1 is running */
        init_ma(&fb2, axisId, 200.0f, 20.0f, 50.0f, HYD_BUFFER_MODE_BLENDING_HIGH);
        rising_edge(&fb2);

        float vel_at_switch = -1.0f;
        int steps = run_until_fb2_done(&fb1, &fb2, &vel_at_switch, 100.0f);

        ASSERT_TRUE(steps > 0,
            "Both FBs should complete without timeout or error/abort");
        ASSERT_TRUE(IEC_VAL(fb1.DONE) == true,
            "FB1 (v=5, pos=100) should output DONE after blended cutover");
        ASSERT_TRUE(IEC_VAL(fb1.COMMANDABORTED) == false,
            "FB1 should NOT output COMMANDABORTED");
        ASSERT_TRUE(IEC_VAL(fb2.DONE) == true,
            "FB2 (v=20, pos=200) should output DONE after reaching pos=200");
        ASSERT_TRUE(IEC_VAL(fb2.COMMANDABORTED) == false,
            "FB2 should NOT output COMMANDABORTED");

        /* Core: velocity at blend switch must be non-zero (smooth transition) */
        ASSERT_TRUE(vel_at_switch > 0.1f,
            "Velocity at blend switch point must be non-zero (no stop/jerk)");

        if (steps > 0) {
            printf("    Done in %d scans, vel@switch=%.3f mm/s\n",
                   steps, vel_at_switch);
        }

        /* Reset FBs for next cycle: lower EXECUTE */
        IEC_VAL(fb1.EXECUTE) = false;
        IEC_VAL(fb2.EXECUTE) = false;
        __mcl_cmd_MoveAbsolute(&fb1);
        __mcl_cmd_MoveAbsolute(&fb2);
        /* Return axis to origin — DIRECTION=0 (SHORTEST_WAY) to handle retract */
        init_ma(&fb1, axisId, 0.0f, 20.0f, 50.0f, HYD_BUFFER_MODE_ABORT);
        IEC_VAL(fb1.DIRECTION) = 0;
        rising_edge(&fb1);
        for (int s = 0; s < MAX_SIM_STEPS; s++) {
            __HydMotion_framework_Publish();
            IEC_VAL(fb1.EXECUTE) = true;
            fb1.EXECUTE0.value   = true;
            __mcl_cmd_MoveAbsolute(&fb1);
            if (IEC_VAL(fb1.DONE)) break;
        }
        IEC_VAL(fb1.EXECUTE) = false;
        __mcl_cmd_MoveAbsolute(&fb1);
    }
}

static void test_blending_high_pending_does_not_take_active_or_overwrite_velocity_early(void) {
    assert_pending_blend_does_not_take_over_early(5,
                                                  20,
                                                  20,
                                                  HYD_BUFFER_MODE_BLENDING_HIGH,
                                                  "BlendingHigh",
                                                  "Before cutover, planner velocity must not jump to FB2 max velocity");
}

static void test_blending_low_pending_does_not_take_active_or_overwrite_velocity_early(void) {
    assert_pending_blend_does_not_take_over_early(20,
                                                  8,
                                                  8,
                                                  HYD_BUFFER_MODE_BLENDING_LOW,
                                                  "BlendingLow",
                                                  "Before cutover, planner velocity must not collapse to FB2 max velocity");
}

static void test_blending_previous_pending_does_not_take_active_or_overwrite_velocity_early(void) {
    assert_pending_blend_does_not_take_over_early(8,
                                                  20,
                                                  8,
                                                  HYD_BUFFER_MODE_BLENDING_PREVIOUS,
                                                  "BlendingPrevious",
                                                  "Before cutover, planner velocity must not jump to FB2 max velocity during BlendingPrevious");
}

static void test_blending_next_pending_does_not_take_active_or_overwrite_velocity_early(void) {
    assert_pending_blend_does_not_take_over_early(20,
                                                  8,
                                                  8,
                                                  HYD_BUFFER_MODE_BLENDING_NEXT,
                                                  "BlendingNext",
                                                  "Before cutover, planner velocity must not collapse to FB2 max velocity during BlendingNext");
}

static void test_three_segment_buffered_chain_reuses_pending_slot_without_early_takeover(void) {
    HYD_MOVEABSOLUTE fb1, fb2, fb3, fb4;
    HYD_MotionControlFB* core;
    int axisId;
    int secondTriggerScan;
    int thirdTriggerScan;
    float velAtSecondSwitch;
    int finishSteps;

    __HydMotion_framework_Init();
    axisId = alloc_sim_axis();
    ASSERT_TRUE(axisId >= 0, "alloc_sim_axis should succeed");
    if (axisId < 0) {
        return;
    }

    init_ma(&fb1, axisId, 100.0f, 5.0f, 50.0f, HYD_BUFFER_MODE_ABORT);
    rising_edge(&fb1);

    init_ma(&fb2, axisId, 200.0f, 20.0f, 50.0f, HYD_BUFFER_MODE_BLENDING_HIGH);
    secondTriggerScan = trigger_fb2_when_fb1_active(&fb1, &fb2, 20);
    ASSERT_TRUE(secondTriggerScan > 0, "FB2 should be triggered after FB1 becomes ACTIVE");
    if (secondTriggerScan <= 0) {
        return;
    }

    core = __MK_GetPublic_MotionControlFB(axisId);
    ASSERT_TRUE(core != NULL, "Public motion control FB should be available");
    if (core == NULL) {
        return;
    }

    init_ma(&fb3, axisId, 300.0f, 8.0f, 50.0f, HYD_BUFFER_MODE_BLENDING_NEXT);
    thirdTriggerScan = -1;
    for (int step = 0; step < MAX_SIM_STEPS; step++) {
        __HydMotion_framework_Publish();
        hold_true_scan(&fb1);
        hold_true_scan(&fb2);

        if (IEC_VAL(fb1.COMMANDABORTED) || IEC_VAL(fb2.COMMANDABORTED)) {
            break;
        }

        if (IEC_VAL(fb1.DONE) && IEC_VAL(fb2.ACTIVE)) {
            ASSERT_TRUE(core->_directPendingValid == false,
                       "Pending slot should be empty immediately after FB2 acquires ownership");
            ASSERT_TRUE(core->_directBlendContext.active == false,
                       "First blend context should be cleared immediately after the first cutover");
            rising_edge(&fb3);
            thirdTriggerScan = step + 1;
            break;
        }
    }

    ASSERT_TRUE(thirdTriggerScan > 0,
               "FB3 should be triggered after FB2 becomes ACTIVE at the first blended cutover");
    if (thirdTriggerScan <= 0) {
        return;
    }
    ASSERT_TRUE(IEC_VAL(fb1.DONE) == true,
               "FB1 should report DONE before FB3 occupies the reused pending slot");
    ASSERT_TRUE(IEC_VAL(fb1.COMMANDABORTED) == false,
               "FB1 should not be aborted during the first blended cutover");
    ASSERT_TRUE(IEC_VAL(fb2.ACTIVE) == true,
               "FB2 should own the axis before FB3 is accepted into the pending slot");
    ASSERT_TRUE(IEC_VAL(fb3.BUSY) == true,
               "FB3 should report BUSY immediately after it reuses the pending slot");
    ASSERT_TRUE(IEC_VAL(fb3.ACTIVE) == false,
               "FB3 must stay inactive until the second cutover");
    ASSERT_TRUE(core->_directPendingValid == true,
               "Pending slot should be reused by FB3 after the first cutover");
    ASSERT_TRUE(core->_directBlendContext.active == true,
               "A second blend context should be recorded for FB2 -> FB3");
    ASSERT_TRUE(fabs(core->_directBlendContext.switchPosition - 200.0f) <= 0.001f,
               "Second blend context should switch at FB2 target position");
    ASSERT_TRUE(fabs(core->_directBlendContext.blendVelocity - 8.0f) <= 0.001f,
               "BlendingNext should use FB3 velocity at the second cutover");

    init_ma(&fb4, axisId, 400.0f, 12.0f, 50.0f, HYD_BUFFER_MODE_BLENDING_HIGH);
    rising_edge(&fb4);
    ASSERT_TRUE(IEC_VAL(fb4.ERROR) == true,
               "A fourth same-axis MoveAbsolute should be rejected while FB2 is active and FB3 is pending");
    ASSERT_TRUE(IEC_VAL(fb4.BUSY) == false,
               "Rejected FB4 should not enter BUSY after pending-slot reuse");
    ASSERT_TRUE(IEC_VAL(fb4.ACTIVE) == false,
               "Rejected FB4 should not enter ACTIVE after pending-slot reuse");
    ASSERT_TRUE(core->_directPendingValid == true,
               "Rejected FB4 should not displace FB3 from the reused pending slot");

    for (int step = 0; step < 40; step++) {
        __HydMotion_framework_Publish();
        hold_true_scan(&fb2);
        hold_true_scan(&fb3);

        if (IEC_VAL(fb3.ACTIVE) != false) {
            ASSERT_TRUE(false,
                       "FB3 should stay inactive while FB2 is still far from the second cutover");
            break;
        }
        if (IEC_VAL(fb2.COMMANDABORTED) != false) {
            ASSERT_TRUE(false,
                       "FB2 should not be aborted by buffered FB3 submission after slot reuse");
            break;
        }
        if (velocity_overwritten_before_cutover(core, 200.0f, 8.0f, 0.25f)) {
            ASSERT_TRUE(false,
                       "Before the second cutover, planner velocity must not collapse to FB3 max velocity");
            break;
        }

        if (core->AXIS_REF.position >= 195.0f) {
            break;
        }
    }

    velAtSecondSwitch = -1.0f;
    finishSteps = run_until_fb2_done(&fb2, &fb3, &velAtSecondSwitch, 200.0f);

    ASSERT_TRUE(finishSteps > 0,
               "FB2 and FB3 should complete without timeout or error after pending-slot reuse");
    ASSERT_TRUE(IEC_VAL(fb2.DONE) == true,
               "FB2 should output DONE after the second blended cutover");
    ASSERT_TRUE(IEC_VAL(fb2.COMMANDABORTED) == false,
               "FB2 should not output COMMANDABORTED during pending-slot reuse");
    ASSERT_TRUE(IEC_VAL(fb3.DONE) == true,
               "FB3 should output DONE after reaching its final target");
    ASSERT_TRUE(IEC_VAL(fb3.COMMANDABORTED) == false,
               "FB3 should not output COMMANDABORTED after slot reuse");
    ASSERT_TRUE(velAtSecondSwitch > 0.1f,
               "Velocity at the second blend switch must remain non-zero");
}

static void test_reverse_direction_pending_falls_back_to_buffered_promotion_after_slot_reuse(void) {
    HYD_MOVEABSOLUTE fb1, fb2, fb3, fb4;
    HYD_MotionControlFB* core;
    int axisId;
    int secondTriggerScan;
    int thirdTriggerScan;
    int promotionScan;
    int finishSteps;

    __HydMotion_framework_Init();
    axisId = alloc_sim_axis();
    ASSERT_TRUE(axisId >= 0, "alloc_sim_axis should succeed");
    if (axisId < 0) {
        return;
    }

    init_ma(&fb1, axisId, 100.0f, 5.0f, 50.0f, HYD_BUFFER_MODE_ABORT);
    rising_edge(&fb1);

    init_ma(&fb2, axisId, 200.0f, 20.0f, 50.0f, HYD_BUFFER_MODE_BLENDING_HIGH);
    secondTriggerScan = trigger_fb2_when_fb1_active(&fb1, &fb2, 20);
    ASSERT_TRUE(secondTriggerScan > 0, "FB2 should be triggered after FB1 becomes ACTIVE");
    if (secondTriggerScan <= 0) {
        return;
    }

    core = __MK_GetPublic_MotionControlFB(axisId);
    ASSERT_TRUE(core != NULL, "Public motion control FB should be available");
    if (core == NULL) {
        return;
    }

    init_ma(&fb3, axisId, 0.0f, 8.0f, 50.0f, HYD_BUFFER_MODE_BLENDING_HIGH);
    IEC_VAL(fb3.DIRECTION) = 2;  /* NEGATIVE */
    thirdTriggerScan = -1;
    for (int step = 0; step < MAX_SIM_STEPS; step++) {
        __HydMotion_framework_Publish();
        hold_true_scan(&fb1);
        hold_true_scan(&fb2);

        if (IEC_VAL(fb1.COMMANDABORTED) || IEC_VAL(fb2.COMMANDABORTED)) {
            break;
        }

        if (IEC_VAL(fb1.DONE) && IEC_VAL(fb2.ACTIVE)) {
            ASSERT_TRUE(core->_directPendingValid == false,
                       "Pending slot should be empty immediately after FB2 acquires ownership");
            ASSERT_TRUE(core->_directBlendContext.active == false,
                       "First blend context should be cleared immediately after the first cutover");
            rising_edge(&fb3);
            thirdTriggerScan = step + 1;
            break;
        }
    }

    ASSERT_TRUE(thirdTriggerScan > 0,
               "FB3 reverse command should be triggered after FB2 becomes ACTIVE");
    if (thirdTriggerScan <= 0) {
        return;
    }
    ASSERT_TRUE(IEC_VAL(fb1.DONE) == true,
               "FB1 should report DONE before reverse FB3 occupies the reused pending slot");
    ASSERT_TRUE(IEC_VAL(fb1.COMMANDABORTED) == false,
               "FB1 should not be aborted during the first blended cutover");
    ASSERT_TRUE(IEC_VAL(fb2.ACTIVE) == true,
               "FB2 should own the axis before reverse FB3 is accepted");
    ASSERT_TRUE(IEC_VAL(fb2.COMMANDABORTED) == false,
               "FB2 should not be aborted when reverse FB3 is submitted");
    ASSERT_TRUE(IEC_VAL(fb3.BUSY) == true,
               "Reverse FB3 should report BUSY immediately after pending-slot reuse");
    ASSERT_TRUE(IEC_VAL(fb3.ACTIVE) == false,
               "Reverse FB3 must stay inactive until buffered promotion");
    ASSERT_TRUE(core->_directPendingValid == true,
               "Pending slot should be reused by reverse FB3 behind active FB2");
    ASSERT_TRUE(core->_directBlendContext.active == false,
               "Incompatible-direction FB2 -> FB3 pair should not create a direct blend context");

    init_ma(&fb4, axisId, 300.0f, 12.0f, 50.0f, HYD_BUFFER_MODE_BLENDING_HIGH);
    rising_edge(&fb4);
    ASSERT_TRUE(IEC_VAL(fb4.ERROR) == true,
               "A fourth same-axis MoveAbsolute should be rejected while FB2 is active and reverse FB3 is pending");
    ASSERT_TRUE(IEC_VAL(fb4.BUSY) == false,
               "Rejected FB4 should not enter BUSY while reverse FB3 holds the pending slot");
    ASSERT_TRUE(IEC_VAL(fb4.ACTIVE) == false,
               "Rejected FB4 should not enter ACTIVE while reverse FB3 holds the pending slot");
    ASSERT_TRUE(core->_directPendingValid == true,
               "Rejected FB4 should not displace reverse FB3 from the pending slot");

    promotionScan = -1;
    for (int step = 0; step < MAX_SIM_STEPS; step++) {
        __HydMotion_framework_Publish();
        hold_true_scan(&fb2);
        hold_true_scan(&fb3);

        if (!IEC_VAL(fb2.DONE)) {
            if (IEC_VAL(fb3.ACTIVE) != false) {
                ASSERT_TRUE(false,
                           "Reverse FB3 should remain pending until FB2 completes");
                break;
            }
            if (IEC_VAL(fb2.COMMANDABORTED) != false) {
                ASSERT_TRUE(false,
                           "FB2 should not be COMMANDABORTED by reverse FB3 submission");
                break;
            }
            if (core->_directBlendContext.active != false) {
                ASSERT_TRUE(false,
                           "Direct blend context must stay inactive for incompatible-direction fallback");
                break;
            }
            continue;
        }

        promotionScan = step + 1;
        break;
    }

    ASSERT_TRUE(promotionScan > 0,
               "FB2 should complete and promote reverse FB3 via the buffered-completion path");
    if (promotionScan <= 0) {
        return;
    }
    ASSERT_TRUE(IEC_VAL(fb2.COMMANDABORTED) == false,
               "FB2 should complete without COMMANDABORTED after reverse FB3 submission");
    ASSERT_TRUE(core->_directPendingValid == false,
               "Pending slot should be cleared after reverse FB3 is promoted");
    ASSERT_TRUE(core->_activeSegmentValid == true,
               "Active segment should be valid after reverse FB3 promotion");
    ASSERT_TRUE(fabs(core->_activeSegment.targetPosition - 0.0f) <= 0.001f,
               "Reverse FB3 target should become the active segment after promotion");
    ASSERT_TRUE(core->_directBlendContext.active == false,
               "Direct blend context should remain inactive after reverse FB3 promotion");
    ASSERT_TRUE(IEC_VAL(fb3._PENDING) == false,
               "Reverse FB3 should clear _PENDING once buffered promotion acquires ownership");
    ASSERT_TRUE(IEC_VAL(fb3._EXEC_ID) != 0,
               "Reverse FB3 should latch direct ownership after buffered promotion");
    ASSERT_TRUE(IEC_VAL(fb3.BUSY) || IEC_VAL(fb3.ACTIVE),
               "Reverse FB3 should be busy or active immediately after buffered promotion");
    ASSERT_TRUE(IEC_VAL(fb3.COMMANDABORTED) == false,
               "Reverse FB3 should not be COMMANDABORTED at promotion");

    __HydMotion_framework_Publish();
    hold_true_scan(&fb3);
    ASSERT_TRUE(IEC_VAL(fb3.ACTIVE) == true,
               "Reverse FB3 should become ACTIVE on the first post-promotion owner scan");
    ASSERT_TRUE(IEC_VAL(fb3.COMMANDABORTED) == false,
               "Reverse FB3 should remain non-aborted after becoming ACTIVE");

    finishSteps = -1;
    for (int step = 0; step < MAX_SIM_STEPS; step++) {
        __HydMotion_framework_Publish();
        hold_true_scan(&fb3);

        if (IEC_VAL(fb3.DONE)) {
            finishSteps = step + 1;
            break;
        }
        if (IEC_VAL(fb3.ERROR) || IEC_VAL(fb3.COMMANDABORTED)) {
            break;
        }
    }

    ASSERT_TRUE(finishSteps > 0,
               "Reverse FB3 should complete after buffered promotion");
    ASSERT_TRUE(IEC_VAL(fb3.DONE) == true,
               "Reverse FB3 should output DONE after reaching its final target");
    ASSERT_TRUE(IEC_VAL(fb3.COMMANDABORTED) == false,
               "Reverse FB3 should complete without COMMANDABORTED");
}

static void test_reverse_then_forward_reuses_pending_slot_without_stale_blend_context(void) {
    HYD_MOVEABSOLUTE fb1, fb2, fb3, fb4, fb5;
    HYD_MotionControlFB* core;
    int axisId;
    int secondTriggerScan;
    int thirdTriggerScan;
    int promotionScan;
    HYD_BOOL observedPreFifthWindow;

    __HydMotion_framework_Init();
    axisId = alloc_sim_axis();
    ASSERT_TRUE(axisId >= 0, "alloc_sim_axis should succeed");
    if (axisId < 0) {
        return;
    }

    init_ma(&fb1, axisId, 100.0f, 5.0f, 50.0f, HYD_BUFFER_MODE_ABORT);
    rising_edge(&fb1);

    init_ma(&fb2, axisId, 200.0f, 20.0f, 50.0f, HYD_BUFFER_MODE_BLENDING_HIGH);
    secondTriggerScan = trigger_fb2_when_fb1_active(&fb1, &fb2, 20);
    ASSERT_TRUE(secondTriggerScan > 0, "FB2 should be triggered after FB1 becomes ACTIVE");
    if (secondTriggerScan <= 0) {
        return;
    }

    core = __MK_GetPublic_MotionControlFB(axisId);
    ASSERT_TRUE(core != NULL, "Public motion control FB should be available");
    if (core == NULL) {
        return;
    }

    init_ma(&fb3, axisId, 0.0f, 8.0f, 50.0f, HYD_BUFFER_MODE_BLENDING_HIGH);
    IEC_VAL(fb3.DIRECTION) = 2;  /* NEGATIVE */
    thirdTriggerScan = -1;
    for (int step = 0; step < MAX_SIM_STEPS; step++) {
        __HydMotion_framework_Publish();
        hold_true_scan(&fb1);
        hold_true_scan(&fb2);

        if (IEC_VAL(fb1.COMMANDABORTED) || IEC_VAL(fb2.COMMANDABORTED)) {
            break;
        }

        if (IEC_VAL(fb1.DONE) && IEC_VAL(fb2.ACTIVE)) {
            ASSERT_TRUE(core->_directPendingValid == false,
                       "Pending slot should be empty immediately after FB2 acquires ownership");
            ASSERT_TRUE(core->_directBlendContext.active == false,
                       "First blend context should be cleared immediately after the first cutover");
            rising_edge(&fb3);
            thirdTriggerScan = step + 1;
            break;
        }
    }

    ASSERT_TRUE(thirdTriggerScan > 0,
               "Reverse FB3 should be triggered after FB2 becomes ACTIVE");
    if (thirdTriggerScan <= 0) {
        return;
    }
    ASSERT_TRUE(IEC_VAL(fb3.BUSY) == true,
               "Reverse FB3 should report BUSY immediately after pending-slot reuse");
    ASSERT_TRUE(IEC_VAL(fb3.ACTIVE) == false,
               "Reverse FB3 must stay inactive until buffered promotion");
    ASSERT_TRUE(core->_directPendingValid == true,
               "Pending slot should be occupied by reverse FB3 before promotion");
    ASSERT_TRUE(core->_directBlendContext.active == false,
               "Forward FB2 -> reverse FB3 should not keep a direct blend context");

    promotionScan = -1;
    for (int step = 0; step < MAX_SIM_STEPS; step++) {
        __HydMotion_framework_Publish();
        hold_true_scan(&fb2);
        hold_true_scan(&fb3);

        if (!IEC_VAL(fb2.DONE)) {
            ASSERT_TRUE(IEC_VAL(fb3.ACTIVE) == false,
                       "Reverse FB3 should remain pending until FB2 completes");
            ASSERT_TRUE(IEC_VAL(fb2.COMMANDABORTED) == false,
                       "FB2 should not be COMMANDABORTED by reverse FB3 submission");
            ASSERT_TRUE(core->_directBlendContext.active == false,
                       "Direct blend context must stay inactive during reverse buffered fallback");
            continue;
        }

        promotionScan = step + 1;
        break;
    }

    ASSERT_TRUE(promotionScan > 0,
               "FB2 should complete and promote reverse FB3 via the buffered-completion path");
    if (promotionScan <= 0) {
        return;
    }

    __HydMotion_framework_Publish();
    hold_true_scan(&fb3);
    ASSERT_TRUE(IEC_VAL(fb3.ACTIVE) == true,
               "Reverse FB3 should become ACTIVE after buffered promotion");
    ASSERT_TRUE(IEC_VAL(fb3.COMMANDABORTED) == false,
               "Reverse FB3 should not be COMMANDABORTED when it becomes ACTIVE");
    ASSERT_TRUE(core->_directPendingValid == false,
               "Pending slot should be empty again after reverse FB3 becomes ACTIVE");
    ASSERT_TRUE(core->_directBlendContext.active == false,
               "No stale blend context should remain after reverse FB3 becomes ACTIVE");

    init_ma(&fb4, axisId, 300.0f, 12.0f, 50.0f, HYD_BUFFER_MODE_BLENDING_HIGH);
    ASSERT_TRUE(core->_directBlendContext.active == false,
               "No stale direct blend context should remain before FB4 is accepted");
    rising_edge(&fb4);

    ASSERT_TRUE(IEC_VAL(fb4.ERROR) == false,
               "FB4 should be accepted into the empty pending slot behind reverse FB3");
    ASSERT_TRUE(IEC_VAL(fb4.BUSY) == true,
               "Accepted FB4 should report BUSY while pending behind reverse FB3");
    ASSERT_TRUE(IEC_VAL(fb4.ACTIVE) == false,
               "Accepted FB4 must stay pending behind reverse FB3");
    ASSERT_TRUE(IEC_VAL(fb4._PENDING) == true,
               "Accepted FB4 should latch the pending state behind reverse FB3");
    ASSERT_TRUE(core->_directPendingValid == true,
               "Pending slot should be reused by FB4 after reverse FB3 becomes ACTIVE");
    ASSERT_TRUE(core->_directBlendContext.active == false,
               "Reverse FB3 -> forward FB4 should not create a direct blend context");
    ASSERT_TRUE(IEC_VAL(fb3.COMMANDABORTED) == false,
               "Active reverse FB3 should not be COMMANDABORTED by pending FB4");

    observedPreFifthWindow = false;
    for (int step = 0; step < 40; step++) {
        __HydMotion_framework_Publish();
        hold_true_scan(&fb3);
        hold_true_scan(&fb4);

        observedPreFifthWindow = true;
        ASSERT_TRUE(IEC_VAL(fb3.ACTIVE) == true,
                   "Reverse FB3 should remain ACTIVE while FB4 is pending");
        ASSERT_TRUE(IEC_VAL(fb4.ACTIVE) == false,
                   "FB4 should remain pending while reverse FB3 is still active");
        ASSERT_TRUE(IEC_VAL(fb3.COMMANDABORTED) == false,
                   "Pending FB4 must not cause reverse FB3 to COMMANDABORT early");
        ASSERT_TRUE(core->_directBlendContext.active == false,
                   "No direct blend context should appear for the reverse->forward pair while FB4 is pending");
        if (core->AXIS_REF.position <= 20.0f) {
            break;
        }
    }
    ASSERT_TRUE(observedPreFifthWindow == true,
               "Pre-FB5 observation window should execute at least once");

    init_ma(&fb5, axisId, 350.0f, 10.0f, 50.0f, HYD_BUFFER_MODE_BLENDING_HIGH);
    rising_edge(&fb5);

    ASSERT_TRUE(IEC_VAL(fb5.ERROR) == true,
               "A fifth same-axis MoveAbsolute should be rejected while reverse FB3 is active and FB4 is pending");
    ASSERT_TRUE(IEC_VAL(fb5.BUSY) == false,
               "Rejected FB5 should not report BUSY while FB4 occupies the pending slot");
    ASSERT_TRUE(IEC_VAL(fb5.ACTIVE) == false,
               "Rejected FB5 should not report ACTIVE while FB4 occupies the pending slot");
    ASSERT_TRUE(core->_directPendingValid == true,
               "Rejected FB5 should not displace FB4 from the reused pending slot");
    ASSERT_TRUE(IEC_VAL(fb4._PENDING) == true,
               "Rejected FB5 should leave FB4 latched as the pending command");
    ASSERT_TRUE(IEC_VAL(fb4.BUSY) == true,
               "Rejected FB5 should leave FB4 BUSY in the pending slot");
    ASSERT_TRUE(IEC_VAL(fb4.ACTIVE) == false,
               "Rejected FB5 should not let FB4 skip directly to ACTIVE");
}

int main(void) {
    printf("=== test_moveabsolute_blending_done ===\n\n");
    test_blending_high_pending_does_not_take_active_or_overwrite_velocity_early();
    test_blending_low_pending_does_not_take_active_or_overwrite_velocity_early();
    test_blending_previous_pending_does_not_take_active_or_overwrite_velocity_early();
    test_blending_next_pending_does_not_take_active_or_overwrite_velocity_early();
    test_three_segment_buffered_chain_reuses_pending_slot_without_early_takeover();
    test_reverse_direction_pending_falls_back_to_buffered_promotion_after_slot_reuse();
    test_reverse_then_forward_reuses_pending_slot_without_stale_blend_context();
    test_blending_high_two_moveabsolute_cycles();
    printf("\n%d/%d tests passed\n", tests_passed, tests_run);
    return (tests_failed > 0) ? 1 : 0;
}
