#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdbool.h>

#include "motion_interface.h"
#include "motion_control.h"

extern HYD_MotionControlFB* __MK_GetPublic_MotionControlFB(int index);

#define IEC_VAL(var) ((var).value)
#define MAX_SIM_STEPS 20000

static int tests_run = 0;
static int tests_passed = 0;
static int tests_failed = 0;

#define ASSERT_TRUE(cond, msg) do { \
    tests_run++; \
    if (cond) { tests_passed++; } \
    else { tests_failed++; printf("  FAIL [line %d]: %s\n", __LINE__, msg); } \
} while (0)

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

static void init_movecontinuousabsolute(HYD_MOVECONTINUOUSABSOLUTE* fb,
                                        int axisId,
                                        HYD_REAL position,
                                        HYD_REAL velocity,
                                        HYD_REAL endVelocity,
                                        IEC_SINT direction,
                                        IEC_SINT endVelocityDirection) {
    memset(fb, 0, sizeof(*fb));
    IEC_VAL(fb->EN) = true;
    IEC_VAL(fb->AXISID) = axisId;
    IEC_VAL(fb->POSITION) = position;
    IEC_VAL(fb->VELOCITY) = velocity;
    IEC_VAL(fb->ENDVELOCITY) = endVelocity;
    IEC_VAL(fb->ENDVELOCITYDIRECTION) = endVelocityDirection;
    IEC_VAL(fb->ACCELERATION) = 100.0f;
    IEC_VAL(fb->DECELERATION) = 100.0f;
    IEC_VAL(fb->DIRECTION) = direction;
}

static void rising_edge_scan(HYD_MOVECONTINUOUSABSOLUTE* fb) {
    IEC_VAL(fb->EXECUTE) = true;
    fb->EXECUTE0.value = false;
    __mcl_cmd_MoveContinuousAbsolute(fb);
    fb->EXECUTE0.value = true;
}

static void hold_true_scan(HYD_MOVECONTINUOUSABSOLUTE* fb) {
    IEC_VAL(fb->EXECUTE) = true;
    fb->EXECUTE0.value = true;
    __mcl_cmd_MoveContinuousAbsolute(fb);
}

static int run_until_position_reached(HYD_MOVECONTINUOUSABSOLUTE* fb) {
    for (int step = 0; step < MAX_SIM_STEPS; step++) {
        __HydMotion_framework_Publish();
        hold_true_scan(fb);
        if (IEC_VAL(fb->POSITIONREACHED)) {
            return step + 1;
        }
    }
    return -1;
}

static int run_until_inendvelocity(HYD_MOVECONTINUOUSABSOLUTE* fb) {
    for (int step = 0; step < MAX_SIM_STEPS; step++) {
        __HydMotion_framework_Publish();
        hold_true_scan(fb);
        if (IEC_VAL(fb->INENDVELOCITY)) {
            return step + 1;
        }
    }
    return -1;
}

static void test_rejects_invalid_end_velocity_direction(void) {
    HYD_MOVECONTINUOUSABSOLUTE cmd;
    int axisId;

    __HydMotion_framework_Init();
    axisId = create_sim_axis();
    ASSERT_TRUE(axisId >= 0, "CreateMotion should allocate a simulation axis");
    if (axisId < 0) {
        return;
    }

    init_movecontinuousabsolute(&cmd, axisId, 100.0f, 20.0f, 8.0f,
                                HYD_DIRECTION_POSITIVE,
                                HYD_DIRECTION_SHORTEST_WAY);
    rising_edge_scan(&cmd);

    ASSERT_TRUE(IEC_VAL(cmd.ERROR) == true,
                "ENDVELOCITYDIRECTION=0 should be rejected");
}

static void test_current_end_velocity_direction_uses_velocity_then_last_active_direction(void) {
    HYD_MOVECONTINUOUSABSOLUTE cmd;
    HYD_MotionControlFB* fb;
    int axisId;

    __HydMotion_framework_Init();
    axisId = create_sim_axis();
    ASSERT_TRUE(axisId >= 0, "CreateMotion should allocate a simulation axis");
    fb = __MK_GetPublic_MotionControlFB(axisId);
    ASSERT_TRUE(fb != NULL, "Allocated simulation axis should expose a motion FB");
    if (axisId < 0 || fb == NULL) {
        return;
    }

    fb->AXIS_REF.velocity = -5.0f;
    fb->_lastActiveDirection = HYD_DIRECTION_POSITIVE;
    init_movecontinuousabsolute(&cmd, axisId, 120.0f, 20.0f, 8.0f,
                                HYD_DIRECTION_POSITIVE,
                                HYD_DIRECTION_CURRENT);
    rising_edge_scan(&cmd);
    ASSERT_TRUE(fb->_directContinuousAbsolute.sustainDirection == HYD_DIRECTION_NEGATIVE,
                "ENDVELOCITYDIRECTION=CURRENT should use the current negative axis velocity first");

    __HydMotion_framework_Init();
    axisId = create_sim_axis();
    ASSERT_TRUE(axisId >= 0, "CreateMotion should allocate a second simulation axis");
    fb = __MK_GetPublic_MotionControlFB(axisId);
    ASSERT_TRUE(fb != NULL, "Second simulation axis should expose a motion FB");
    if (axisId < 0 || fb == NULL) {
        return;
    }

    fb->AXIS_REF.velocity = 0.0f;
    fb->_lastActiveDirection = HYD_DIRECTION_NEGATIVE;
    init_movecontinuousabsolute(&cmd, axisId, 120.0f, 20.0f, 8.0f,
                                HYD_DIRECTION_POSITIVE,
                                HYD_DIRECTION_CURRENT);
    rising_edge_scan(&cmd);
    ASSERT_TRUE(fb->_directContinuousAbsolute.sustainDirection == HYD_DIRECTION_NEGATIVE,
                "ENDVELOCITYDIRECTION=CURRENT should fall back to the last active direction when axis velocity is zero");
}

static void test_same_direction_reaches_position_and_end_velocity(void) {
    HYD_MOVECONTINUOUSABSOLUTE cmd;
    HYD_MotionControlFB* fb;
    int axisId;
    int positionReachedStep;
    int inEndVelocityStep;

    __HydMotion_framework_Init();
    axisId = create_sim_axis();
    ASSERT_TRUE(axisId >= 0, "CreateMotion should allocate a simulation axis");
    fb = __MK_GetPublic_MotionControlFB(axisId);
    ASSERT_TRUE(fb != NULL, "Allocated simulation axis should expose a motion FB");
    if (axisId < 0 || fb == NULL) {
        return;
    }

    init_movecontinuousabsolute(&cmd, axisId, 120.0f, 25.0f, 8.0f,
                                HYD_DIRECTION_POSITIVE,
                                HYD_DIRECTION_POSITIVE);
    rising_edge_scan(&cmd);

    positionReachedStep = run_until_position_reached(&cmd);
    ASSERT_TRUE(positionReachedStep > 0,
                "Same-direction MoveContinuousAbsolute should reach POSITIONREACHED");
    if (positionReachedStep <= 0) {
        return;
    }

    inEndVelocityStep = run_until_inendvelocity(&cmd);
    ASSERT_TRUE(inEndVelocityStep > 0,
                "Same-direction MoveContinuousAbsolute should eventually reach INENDVELOCITY");
    if (inEndVelocityStep <= 0) {
        return;
    }

    ASSERT_TRUE(IEC_VAL(cmd.POSITIONREACHED) == true,
                "POSITIONREACHED should stay latched once the target position is crossed");
    ASSERT_TRUE(IEC_VAL(cmd.INENDVELOCITY) == true,
                "INENDVELOCITY should become true after the sustain velocity settles");
    ASSERT_TRUE(IEC_VAL(cmd.BUSY) == true,
                "MoveContinuousAbsolute should remain BUSY while sustaining the end velocity");
    ASSERT_TRUE(fabs(fb->AXIS_REF.velocity) > 0.01f,
                "Axis velocity should remain non-zero after reaching the same-direction end velocity");
}

static void test_adapt_lowers_crossing_velocity_when_distance_is_too_short_to_accelerate(void) {
    HYD_MOVECONTINUOUSABSOLUTE cmd;
    HYD_MotionControlFB* fb;
    int axisId;

    __HydMotion_framework_Init();
    axisId = create_sim_axis();
    ASSERT_TRUE(axisId >= 0, "CreateMotion should allocate a simulation axis");
    fb = __MK_GetPublic_MotionControlFB(axisId);
    ASSERT_TRUE(fb != NULL, "Allocated simulation axis should expose a motion FB");
    if (axisId < 0 || fb == NULL) {
        return;
    }

    fb->AXIS_REF.velocity = 0.0f;
    init_movecontinuousabsolute(&cmd, axisId, 5.0f, 10.0f, 40.0f,
                                HYD_DIRECTION_POSITIVE,
                                HYD_DIRECTION_POSITIVE);
    IEC_VAL(cmd.ADAPTENDVELTOAVOIDOVERSHOOT) = true;
    rising_edge_scan(&cmd);

    ASSERT_TRUE(fb->_directContinuousAbsolute.crossingVelocity < IEC_VAL(cmd.ENDVELOCITY),
                "Adapt mode should lower crossingVelocity when the distance is too short to accelerate up to the requested end velocity");
}

static void test_adapt_raises_crossing_velocity_when_distance_is_too_short_to_decelerate(void) {
    HYD_MOVECONTINUOUSABSOLUTE cmd;
    HYD_MotionControlFB* fb;
    int axisId;

    __HydMotion_framework_Init();
    axisId = create_sim_axis();
    ASSERT_TRUE(axisId >= 0, "CreateMotion should allocate a simulation axis");
    fb = __MK_GetPublic_MotionControlFB(axisId);
    ASSERT_TRUE(fb != NULL, "Allocated simulation axis should expose a motion FB");
    if (axisId < 0 || fb == NULL) {
        return;
    }

    fb->AXIS_REF.velocity = 35.0f;
    init_movecontinuousabsolute(&cmd, axisId, 6.0f, 35.0f, 5.0f,
                                HYD_DIRECTION_POSITIVE,
                                HYD_DIRECTION_POSITIVE);
    IEC_VAL(cmd.ADAPTENDVELTOAVOIDOVERSHOOT) = true;
    rising_edge_scan(&cmd);

    ASSERT_TRUE(fb->_directContinuousAbsolute.crossingVelocity > IEC_VAL(cmd.ENDVELOCITY),
                "Adapt mode should raise crossingVelocity when the distance is too short to decelerate down to the requested end velocity");
}

static void test_reverse_sustain_delays_inendvelocity(void) {
    HYD_MOVECONTINUOUSABSOLUTE cmd;
    HYD_MotionControlFB* fb;
    int axisId;
    int positionReachedStep = -1;
    int inEndVelocityStep = -1;

    __HydMotion_framework_Init();
    axisId = create_sim_axis();
    ASSERT_TRUE(axisId >= 0, "CreateMotion should allocate a simulation axis");
    fb = __MK_GetPublic_MotionControlFB(axisId);
    ASSERT_TRUE(fb != NULL, "Allocated simulation axis should expose a motion FB");
    if (axisId < 0 || fb == NULL) {
        return;
    }

    init_movecontinuousabsolute(&cmd, axisId, 90.0f, 20.0f, 10.0f,
                                HYD_DIRECTION_POSITIVE,
                                HYD_DIRECTION_NEGATIVE);
    rising_edge_scan(&cmd);

    for (int step = 0; step < MAX_SIM_STEPS; step++) {
        __HydMotion_framework_Publish();
        hold_true_scan(&cmd);

        if (positionReachedStep < 0 && IEC_VAL(cmd.POSITIONREACHED)) {
            positionReachedStep = step + 1;
        }
        if (IEC_VAL(cmd.INENDVELOCITY)) {
            inEndVelocityStep = step + 1;
            break;
        }
    }

    ASSERT_TRUE(positionReachedStep > 0,
                "Reverse sustain should still reach POSITIONREACHED");
    ASSERT_TRUE(inEndVelocityStep > 0,
                "Reverse sustain should eventually reach INENDVELOCITY");
    ASSERT_TRUE(positionReachedStep > 0 && inEndVelocityStep > positionReachedStep,
                "Reverse sustain should latch POSITIONREACHED before INENDVELOCITY");
    ASSERT_TRUE(fb->AXIS_REF.velocity < 0.0f,
                "Final sustained velocity should be negative for reverse sustain");
}

static void test_pressure_limit_can_hold_positionreached_true_while_inendvelocity_stays_false(void) {
    HYD_MOVECONTINUOUSABSOLUTE cmd;
    HYD_MotionControlFB* fb;
    int axisId;
    bool sawSplitState = false;
    bool sawPositionReached = false;

    __HydMotion_framework_Init();
    axisId = create_sim_axis();
    ASSERT_TRUE(axisId >= 0, "CreateMotion should allocate a simulation axis");
    fb = __MK_GetPublic_MotionControlFB(axisId);
    ASSERT_TRUE(fb != NULL, "Allocated simulation axis should expose a motion FB");
    if (axisId < 0 || fb == NULL) {
        return;
    }

    fb->PRESSURE_LIMIT = 2.0f;
    fb->AXIS_REF.pressure = 20.0f;
    init_movecontinuousabsolute(&cmd, axisId, 80.0f, 20.0f, 12.0f,
                                HYD_DIRECTION_POSITIVE,
                                HYD_DIRECTION_POSITIVE);
    IEC_VAL(cmd.PRESSURELIMIT) = 0.0f;
    rising_edge_scan(&cmd);

    for (int step = 0; step < MAX_SIM_STEPS; step++) {
        fb->AXIS_REF.pressure = 20.0f;
        __HydMotion_framework_Publish();
        hold_true_scan(&cmd);

        if (IEC_VAL(cmd.POSITIONREACHED)) {
            sawPositionReached = true;
        }
        if (IEC_VAL(cmd.POSITIONREACHED) && !IEC_VAL(cmd.INENDVELOCITY)) {
            sawSplitState = true;
            break;
        }
    }

    ASSERT_TRUE(sawPositionReached,
                "Pressure-limited run should still reach POSITIONREACHED");
    ASSERT_TRUE(sawSplitState,
                "Pressure limiting should allow a scan where POSITIONREACHED stays true while INENDVELOCITY remains false");
}

static void test_stop_takes_over_and_sets_commandaborted(void) {
    HYD_MOVECONTINUOUSABSOLUTE cmd;
    HYD_STOP stop;
    int axisId;
    int commandAbortedStep = -1;

    __HydMotion_framework_Init();
    axisId = create_sim_axis();
    ASSERT_TRUE(axisId >= 0, "CreateMotion should allocate a simulation axis");
    if (axisId < 0) {
        return;
    }

    init_movecontinuousabsolute(&cmd, axisId, 160.0f, 25.0f, 8.0f,
                                HYD_DIRECTION_POSITIVE,
                                HYD_DIRECTION_POSITIVE);
    rising_edge_scan(&cmd);

    for (int step = 0; step < 40; step++) {
        __HydMotion_framework_Publish();
        hold_true_scan(&cmd);
    }

    memset(&stop, 0, sizeof(stop));
    IEC_VAL(stop.EN) = true;
    IEC_VAL(stop.AXISID) = axisId;
    IEC_VAL(stop.DECELERATION) = 80.0f;
    IEC_VAL(stop.EXECUTE) = true;
    stop.EXECUTE0.value = false;
    __mcl_cmd_Stop(&stop);

    for (int step = 0; step < MAX_SIM_STEPS; step++) {
        __HydMotion_framework_Publish();
        hold_true_scan(&cmd);

        IEC_VAL(stop.EXECUTE) = true;
        stop.EXECUTE0.value = true;
        __mcl_cmd_Stop(&stop);

        if (IEC_VAL(cmd.COMMANDABORTED)) {
            commandAbortedStep = step + 1;
            break;
        }
    }

    ASSERT_TRUE(commandAbortedStep > 0,
                "Stop takeover should eventually set COMMANDABORTED on MoveContinuousAbsolute");
    ASSERT_TRUE(IEC_VAL(cmd.COMMANDABORTED) == true,
                "MoveContinuousAbsolute should report COMMANDABORTED after Stop takeover");
    ASSERT_TRUE(IEC_VAL(cmd.BUSY) == false,
                "MoveContinuousAbsolute BUSY should clear after Stop takeover");
}

int main(void) {
    printf("=== MoveContinuousAbsolute integration ===\n\n");

    test_rejects_invalid_end_velocity_direction();
    test_current_end_velocity_direction_uses_velocity_then_last_active_direction();
    test_same_direction_reaches_position_and_end_velocity();
    test_adapt_lowers_crossing_velocity_when_distance_is_too_short_to_accelerate();
    test_adapt_raises_crossing_velocity_when_distance_is_too_short_to_decelerate();
    test_reverse_sustain_delays_inendvelocity();
    test_pressure_limit_can_hold_positionreached_true_while_inendvelocity_stays_false();
    test_stop_takes_over_and_sets_commandaborted();

    printf("\n=== Results: %d/%d passed ===\n", tests_passed, tests_run);
    return (tests_failed == 0) ? 0 : 1;
}
