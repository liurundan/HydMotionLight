#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdbool.h>

#include "motion_interface.h"
#include "motion_control.h"

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

static int create_axis(bool useSimulation) {
    HYD_CREATEMOTION cm;

    memset(&cm, 0, sizeof(cm));
    IEC_VAL(cm.EN) = true;
    IEC_VAL(cm.USE_RECIPE) = false;
    IEC_VAL(cm.FLOW_TO_PUMPSPEED) = 1.0f;
    IEC_VAL(cm.PUMPSPEED_LIMIT) = 3000.0f;
    IEC_VAL(cm.USE_SIMULATION) = useSimulation;
    __mcl_cmd_CreateMotion(&cm);
    return (int)IEC_VAL(cm.AXISID);
}

static int create_sim_axis(void) {
    return create_axis(true);
}

static int create_manual_axis(void) {
    return create_axis(false);
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

static void hold_movevelocity_scan(HYD_MOVEVELOCITY* fb) {
    IEC_VAL(fb->EXECUTE) = true;
    fb->EXECUTE0.value = true;
    __mcl_cmd_MoveVelocity(fb);
}

static void hold_stop_scan(HYD_STOP* fb) {
    IEC_VAL(fb->EXECUTE) = true;
    fb->EXECUTE0.value = true;
    __mcl_cmd_Stop(fb);
}

static bool read_sim_feedback(int axisId,
                              HYD_REAL* position,
                              HYD_REAL* velocity,
                              HYD_REAL* flow,
                              HYD_REAL* pressure) {
    HYD_READSIMFEEDBACK readback;

    memset(&readback, 0, sizeof(readback));
    IEC_VAL(readback.EN) = true;
    IEC_VAL(readback.AXISID) = axisId;
    IEC_VAL(readback.ENABLE) = true;
    __mcl_cmd_ReadSimFeedback(&readback);

    if (!IEC_VAL(readback.VALID) || IEC_VAL(readback.ERROR)) {
        return false;
    }

    if (position != NULL) {
        *position = IEC_VAL(readback.POSITION);
    }
    if (velocity != NULL) {
        *velocity = IEC_VAL(readback.VELOCITY);
    }
    if (flow != NULL) {
        *flow = IEC_VAL(readback.FLOW);
    }
    if (pressure != NULL) {
        *pressure = IEC_VAL(readback.PRESSURE);
    }
    return true;
}

static bool set_axis_feedback(int axisId,
                              HYD_REAL position,
                              HYD_REAL velocity,
                              HYD_REAL flow,
                              HYD_REAL pressure,
                              HYD_REAL timestamp) {
    HYD_SETAXISFEEDBACK writeback;

    memset(&writeback, 0, sizeof(writeback));
    IEC_VAL(writeback.EN) = true;
    IEC_VAL(writeback.AXISID) = axisId;
    IEC_VAL(writeback.ENABLE) = true;
    IEC_VAL(writeback.ACT_POSITION) = position;
    IEC_VAL(writeback.ACT_VELOCITY) = velocity;
    IEC_VAL(writeback.ACT_FLOW) = flow;
    IEC_VAL(writeback.ACT_PRESSURE) = pressure;
    IEC_VAL(writeback.TIMESTAMP) = timestamp;
    __mcl_cmd_SetAxisFeedback(&writeback);

    return IEC_VAL(writeback.DONE) == true && IEC_VAL(writeback.ERROR) == false;
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

static bool seed_negative_velocity_history(int axisId, bool stopToZero) {
    HYD_MOVEVELOCITY mv;
    HYD_STOP stop;

    memset(&mv, 0, sizeof(mv));
    IEC_VAL(mv.EN) = true;
    IEC_VAL(mv.AXISID) = axisId;
    IEC_VAL(mv.VELOCITY) = 10.0f;
    IEC_VAL(mv.ACCELERATION) = 100.0f;
    IEC_VAL(mv.DECELERATION) = 100.0f;
    IEC_VAL(mv.DIRECTION) = HYD_DIRECTION_NEGATIVE;
    IEC_VAL(mv.EXECUTE) = true;
    mv.EXECUTE0.value = false;
    __mcl_cmd_MoveVelocity(&mv);

    for (int step = 0; step < 40; step++) {
        __HydMotion_framework_Publish();
        hold_movevelocity_scan(&mv);
    }

    if (!stopToZero) {
        return true;
    }

    memset(&stop, 0, sizeof(stop));
    IEC_VAL(stop.EN) = true;
    IEC_VAL(stop.AXISID) = axisId;
    IEC_VAL(stop.DECELERATION) = 100.0f;
    IEC_VAL(stop.EXECUTE) = true;
    stop.EXECUTE0.value = false;
    __mcl_cmd_Stop(&stop);

    for (int step = 0; step < MAX_SIM_STEPS; step++) {
        __HydMotion_framework_Publish();
        hold_movevelocity_scan(&mv);
        hold_stop_scan(&stop);
        if (IEC_VAL(stop.DONE)) {
            return true;
        }
    }

    return false;
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
    int axisId;
    int inEndVelocityStep;
    bool seededNegativeHistory;
    HYD_REAL finalVelocity = 0.0f;

    __HydMotion_framework_Init();
    axisId = create_sim_axis();
    ASSERT_TRUE(axisId >= 0, "CreateMotion should allocate a simulation axis");
    if (axisId < 0) {
        return;
    }

    seededNegativeHistory = seed_negative_velocity_history(axisId, false);
    ASSERT_TRUE(seededNegativeHistory,
                "Public commands should be able to create a negative actual velocity before the current-direction case");
    if (!seededNegativeHistory) {
        return;
    }

    init_movecontinuousabsolute(&cmd, axisId, 120.0f, 20.0f, 8.0f,
                                HYD_DIRECTION_POSITIVE,
                                HYD_DIRECTION_CURRENT);
    rising_edge_scan(&cmd);
    inEndVelocityStep = run_until_inendvelocity(&cmd);
    ASSERT_TRUE(inEndVelocityStep > 0,
                "ENDVELOCITYDIRECTION=CURRENT should eventually reach INENDVELOCITY when starting from a negative actual velocity");
    if (inEndVelocityStep > 0) {
        ASSERT_TRUE(read_sim_feedback(axisId, NULL, &finalVelocity, NULL, NULL),
                    "ReadSimFeedback should expose the sustained velocity in the negative-velocity case");
        ASSERT_TRUE(IEC_VAL(cmd.ERROR) == false,
                    "Current-direction command should not error in the negative-velocity case");
        ASSERT_TRUE(IEC_VAL(cmd.COMMANDABORTED) == false,
                    "Current-direction command should not be aborted in the negative-velocity case");
        ASSERT_TRUE(finalVelocity < 0.0f,
                    "Current-direction command should sustain a negative end velocity when the actual velocity starts negative");
    }

    __HydMotion_framework_Init();
    axisId = create_sim_axis();
    ASSERT_TRUE(axisId >= 0, "CreateMotion should allocate a second simulation axis");
    if (axisId < 0) {
        return;
    }

    seededNegativeHistory = seed_negative_velocity_history(axisId, true);
    ASSERT_TRUE(seededNegativeHistory,
                "Public commands should be able to seed a negative last-active direction before the fallback case");
    if (!seededNegativeHistory) {
        return;
    }

    init_movecontinuousabsolute(&cmd, axisId, 120.0f, 20.0f, 8.0f,
                                HYD_DIRECTION_POSITIVE,
                                HYD_DIRECTION_CURRENT);
    rising_edge_scan(&cmd);
    inEndVelocityStep = run_until_inendvelocity(&cmd);
    ASSERT_TRUE(inEndVelocityStep > 0,
                "ENDVELOCITYDIRECTION=CURRENT should eventually reach INENDVELOCITY in the zero-velocity fallback case");
    if (inEndVelocityStep > 0) {
        ASSERT_TRUE(read_sim_feedback(axisId, NULL, &finalVelocity, NULL, NULL),
                    "ReadSimFeedback should expose the sustained velocity in the zero-velocity fallback case");
        ASSERT_TRUE(IEC_VAL(cmd.ERROR) == false,
                    "Current-direction fallback command should not error after seeding a negative last-active direction");
        ASSERT_TRUE(IEC_VAL(cmd.COMMANDABORTED) == false,
                    "Current-direction fallback command should not be aborted after seeding a negative last-active direction");
        ASSERT_TRUE(finalVelocity < 0.0f,
                    "Current-direction fallback should sustain a negative end velocity after public negative-direction history");
    }
}

static void test_same_direction_reaches_position_and_end_velocity(void) {
    HYD_MOVECONTINUOUSABSOLUTE cmd;
    int axisId;
    int positionReachedStep;
    int inEndVelocityStep;
    HYD_REAL finalVelocity = 0.0f;

    __HydMotion_framework_Init();
    axisId = create_sim_axis();
    ASSERT_TRUE(axisId >= 0, "CreateMotion should allocate a simulation axis");
    if (axisId < 0) {
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
    ASSERT_TRUE(read_sim_feedback(axisId, NULL, &finalVelocity, NULL, NULL),
                "ReadSimFeedback should expose the sustained velocity in the same-direction case");
    ASSERT_TRUE(fabs(finalVelocity) > 0.01f,
                "Axis velocity should remain non-zero after reaching the same-direction end velocity");
}

static void test_adapt_lowers_crossing_velocity_when_distance_is_too_short_to_accelerate(void) {
    HYD_MOVECONTINUOUSABSOLUTE cmd;
    int axisId;
    int positionReachedStep;
    bool reversedBeforeTarget = false;
    HYD_REAL observedVelocity = 0.0f;

    __HydMotion_framework_Init();
    axisId = create_sim_axis();
    ASSERT_TRUE(axisId >= 0, "CreateMotion should allocate a simulation axis");
    if (axisId < 0) {
        return;
    }

    init_movecontinuousabsolute(&cmd, axisId, 5.0f, 10.0f, 40.0f,
                                HYD_DIRECTION_POSITIVE,
                                HYD_DIRECTION_POSITIVE);
    IEC_VAL(cmd.ADAPTENDVELTOAVOIDOVERSHOOT) = true;
    rising_edge_scan(&cmd);

    positionReachedStep = -1;
    for (int step = 0; step < MAX_SIM_STEPS; step++) {
        __HydMotion_framework_Publish();
        hold_true_scan(&cmd);

        if (read_sim_feedback(axisId, NULL, &observedVelocity, NULL, NULL) &&
            observedVelocity < -0.01f) {
            reversedBeforeTarget = true;
        }
        if (IEC_VAL(cmd.POSITIONREACHED)) {
            positionReachedStep = step + 1;
            break;
        }
        if (IEC_VAL(cmd.ERROR) || IEC_VAL(cmd.COMMANDABORTED)) {
            break;
        }
    }

    ASSERT_TRUE(positionReachedStep > 0,
                "Adapt mode should still reach POSITIONREACHED on a short accelerate-up move");
    ASSERT_TRUE(IEC_VAL(cmd.ERROR) == false,
                "Adapt mode should not error on a short accelerate-up move");
    ASSERT_TRUE(IEC_VAL(cmd.COMMANDABORTED) == false,
                "Adapt mode should not be aborted on a short accelerate-up move");
    ASSERT_TRUE(reversedBeforeTarget == false,
                "Adapt mode should not reverse direction before the first target reach on a short accelerate-up move");
}

static void test_adapt_raises_crossing_velocity_when_distance_is_too_short_to_decelerate(void) {
    HYD_MOVECONTINUOUSABSOLUTE cmd;
    int axisId;
    int positionReachedStep;
    bool reversedBeforeTarget = false;
    HYD_REAL observedVelocity = 0.0f;
    HYD_REAL manualPosition = 0.0f;
    HYD_REAL manualVelocity = 35.0f;
    HYD_REAL timestamp = 0.0f;
    const HYD_REAL dt = 0.01f;

    __HydMotion_framework_Init();
    axisId = create_manual_axis();
    ASSERT_TRUE(axisId >= 0, "CreateMotion should allocate a manual-feedback axis");
    if (axisId < 0) {
        return;
    }

    if (!set_axis_feedback(axisId, manualPosition, manualVelocity, manualVelocity, 5.0f, timestamp)) {
        ASSERT_TRUE(false,
                    "SetAxisFeedback should seed a positive high initial velocity for the short decelerate-down case");
        return;
    }
    init_movecontinuousabsolute(&cmd, axisId, 6.0f, 35.0f, 5.0f,
                                HYD_DIRECTION_POSITIVE,
                                HYD_DIRECTION_POSITIVE);
    IEC_VAL(cmd.ADAPTENDVELTOAVOIDOVERSHOOT) = true;
    rising_edge_scan(&cmd);

    positionReachedStep = -1;
    for (int step = 0; step < MAX_SIM_STEPS; step++) {
        timestamp += dt;
        if (!IEC_VAL(cmd.POSITIONREACHED)) {
            manualPosition += manualVelocity * dt;
            if (manualPosition > IEC_VAL(cmd.POSITION)) {
                manualPosition = IEC_VAL(cmd.POSITION);
            }
        }
        if (!set_axis_feedback(axisId, manualPosition, manualVelocity, manualVelocity, 5.0f, timestamp)) {
            ASSERT_TRUE(false,
                        "SetAxisFeedback should sustain the public high-velocity precondition for the short decelerate-down case");
            return;
        }
        __HydMotion_framework_Publish();
        hold_true_scan(&cmd);

        if (read_sim_feedback(axisId, NULL, &observedVelocity, NULL, NULL) &&
            observedVelocity < -0.01f) {
            reversedBeforeTarget = true;
        }
        if (IEC_VAL(cmd.POSITIONREACHED)) {
            positionReachedStep = step + 1;
            break;
        }
        if (IEC_VAL(cmd.ERROR) || IEC_VAL(cmd.COMMANDABORTED)) {
            break;
        }
    }

    ASSERT_TRUE(positionReachedStep > 0,
                "Adapt mode should still reach POSITIONREACHED on a short decelerate-down move");
    ASSERT_TRUE(IEC_VAL(cmd.ERROR) == false,
                "Adapt mode should not error on a short decelerate-down move");
    ASSERT_TRUE(IEC_VAL(cmd.COMMANDABORTED) == false,
                "Adapt mode should not be aborted on a short decelerate-down move");
    ASSERT_TRUE(reversedBeforeTarget == false,
                "Adapt mode should not reverse direction before the first target reach on a short decelerate-down move");
}

static void test_reverse_sustain_delays_inendvelocity(void) {
    HYD_MOVECONTINUOUSABSOLUTE cmd;
    int axisId;
    int positionReachedStep = -1;
    int inEndVelocityStep = -1;
    HYD_REAL finalVelocity = 0.0f;

    __HydMotion_framework_Init();
    axisId = create_sim_axis();
    ASSERT_TRUE(axisId >= 0, "CreateMotion should allocate a simulation axis");
    if (axisId < 0) {
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
    ASSERT_TRUE(read_sim_feedback(axisId, NULL, &finalVelocity, NULL, NULL),
                "ReadSimFeedback should expose the sustained velocity in the reverse case");
    ASSERT_TRUE(finalVelocity < 0.0f,
                "Final sustained velocity should be negative for reverse sustain");
}

static void test_pressure_limit_can_hold_positionreached_true_while_inendvelocity_stays_false(void) {
    HYD_MOVECONTINUOUSABSOLUTE cmd;
    int axisId;
    bool sawSplitState = false;
    bool sawPositionReached = false;
    HYD_REAL manualPosition = 0.0f;
    HYD_REAL manualVelocity = 18.0f;
    HYD_REAL timestamp = 0.0f;
    const HYD_REAL dt = 0.01f;

    __HydMotion_framework_Init();
    axisId = create_manual_axis();
    ASSERT_TRUE(axisId >= 0, "CreateMotion should allocate a manual-feedback axis");
    if (axisId < 0) {
        return;
    }

    ASSERT_TRUE(set_axis_feedback(axisId, manualPosition, manualVelocity, manualVelocity, 20.0f, timestamp),
                "SetAxisFeedback should seed the manual axis before the pressure-limit case");
    init_movecontinuousabsolute(&cmd, axisId, 80.0f, 20.0f, 12.0f,
                                HYD_DIRECTION_POSITIVE,
                                HYD_DIRECTION_POSITIVE);
    IEC_VAL(cmd.PRESSURELIMIT) = 2.0f;
    rising_edge_scan(&cmd);

    for (int step = 0; step < MAX_SIM_STEPS; step++) {
        timestamp += dt;
        if (!IEC_VAL(cmd.POSITIONREACHED)) {
            manualPosition += manualVelocity * dt;
            if (manualPosition > IEC_VAL(cmd.POSITION)) {
                manualPosition = IEC_VAL(cmd.POSITION);
            }
        }
        if (!set_axis_feedback(axisId, manualPosition, manualVelocity, manualVelocity, 20.0f, timestamp)) {
            ASSERT_TRUE(false,
                        "SetAxisFeedback should keep pressure limiting active on the manual axis");
            return;
        }
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
