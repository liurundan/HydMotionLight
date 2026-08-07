#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdbool.h>

#include "motion_interface.h"
#include "motion_control.h"

extern HYD_MotionControlFB* __MK_GetPublic_MotionControlFB(int index);

#define IEC_VAL(var) ((var).value)
#define MAX_SIM_STEPS 20000
#define VELOCITY_EPSILON 0.01f

enum {
    RUN_TIMEOUT = -1,
    RUN_ERROR = -2,
    RUN_ABORTED = -3
};

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
    __mcl_cmd_MoveContinuousAbsolute(fb);
}

static void hold_true_scan(HYD_MOVECONTINUOUSABSOLUTE* fb) {
    IEC_VAL(fb->EXECUTE) = true;
    __mcl_cmd_MoveContinuousAbsolute(fb);
}

static void hold_movevelocity_scan(HYD_MOVEVELOCITY* fb) {
    IEC_VAL(fb->EXECUTE) = true;
    __mcl_cmd_MoveVelocity(fb);
}

static void hold_stop_scan(HYD_STOP* fb) {
    IEC_VAL(fb->EXECUTE) = true;
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

static void test_plc_pressure_feedback_and_movecontinuousabsolute_limit_use_bar(void) {
    HYD_MOVECONTINUOUSABSOLUTE cmd;
    HYD_MotionControlFB* core;
    int axisId;

    __HydMotion_framework_Init();
    axisId = create_manual_axis();
    ASSERT_TRUE(axisId >= 0, "CreateMotion should allocate a manual axis for IEC pressure-unit boundary");
    if (axisId < 0) {
        return;
    }

    core = __MK_GetPublic_MotionControlFB(axisId);
    ASSERT_TRUE(core != NULL, "IEC pressure-unit boundary should expose the public FB");
    if (core == NULL) {
        return;
    }

    ASSERT_TRUE(set_axis_feedback(axisId, 0.0f, 0.0f, 0.0f, 110.0f, 0.0f),
                "SetAxisFeedback should accept bar-valued pressure feedback");
    ASSERT_TRUE(fabsf(core->AXIS_REF.pressure - 110.0f) <= 1e-6f,
                "SetAxisFeedback ACT_PRESSURE should remain in bar in the core axis feedback");

    init_movecontinuousabsolute(&cmd, axisId, 10.0f, 5.0f, 2.0f,
                                HYD_DIRECTION_POSITIVE,
                                HYD_DIRECTION_POSITIVE);
    IEC_VAL(cmd.PRESSURELIMIT) = 100.0f;
    rising_edge_scan(&cmd);

    ASSERT_TRUE(IEC_VAL(cmd.ERROR) == false,
                "MoveContinuousAbsolute should accept a finite bar-valued PRESSURELIMIT");
    ASSERT_TRUE(fabsf(core->DIRECT_SEGMENT.maxPressure - 100.0f) <= 1e-6f,
                "MoveContinuousAbsolute PRESSURELIMIT should enter segment.maxPressure in bar");
    ASSERT_TRUE(fabsf(core->_directContinuousAbsolute.effectivePressureLimit - 100.0f) <= 1e-6f,
                "MoveContinuousAbsolute direct context should retain the bar-valued pressure limit");
}

static int run_until_position_reached(HYD_MOVECONTINUOUSABSOLUTE* fb) {
    for (int step = 0; step < MAX_SIM_STEPS; step++) {
        __HydMotion_framework_Publish();
        hold_true_scan(fb);
        if (IEC_VAL(fb->POSITIONREACHED)) {
            return step + 1;
        }
        if (IEC_VAL(fb->ERROR)) {
            return RUN_ERROR;
        }
        if (IEC_VAL(fb->COMMANDABORTED)) {
            return RUN_ABORTED;
        }
    }
    return RUN_TIMEOUT;
}

static int run_until_inendvelocity(HYD_MOVECONTINUOUSABSOLUTE* fb) {
    for (int step = 0; step < MAX_SIM_STEPS; step++) {
        __HydMotion_framework_Publish();
        hold_true_scan(fb);
        if (IEC_VAL(fb->INENDVELOCITY)) {
            return step + 1;
        }
        if (IEC_VAL(fb->ERROR)) {
            return RUN_ERROR;
        }
        if (IEC_VAL(fb->COMMANDABORTED)) {
            return RUN_ABORTED;
        }
    }
    return RUN_TIMEOUT;
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

static void assert_run_result(int result, const char* successMsg) {
    if (result == RUN_ERROR) {
        ASSERT_TRUE(false,
                    "Motion helper stopped early because ERROR became true");
        return;
    }
    if (result == RUN_ABORTED) {
        ASSERT_TRUE(false,
                    "Motion helper stopped early because COMMANDABORTED became true");
        return;
    }
    if (result == RUN_TIMEOUT) {
        ASSERT_TRUE(false,
                    "Motion helper timed out before reaching the requested state");
        return;
    }

    ASSERT_TRUE(result > 0, successMsg);
}

static void test_zero_end_velocity_direction_defaults_to_current(void) {
    HYD_MOVECONTINUOUSABSOLUTE cmd;
    int axisId;
    int inEndVelocityStep;
    bool seededNegativeHistory;
    HYD_REAL seededVelocity = 0.0f;
    HYD_REAL finalVelocity = 0.0f;

    __HydMotion_framework_Init();
    axisId = create_sim_axis();
    ASSERT_TRUE(axisId >= 0, "CreateMotion should allocate a simulation axis");
    if (axisId < 0) {
        return;
    }

    seededNegativeHistory = seed_negative_velocity_history(axisId, false);
    ASSERT_TRUE(seededNegativeHistory,
                "Public commands should be able to create a negative actual velocity before the default current-direction case");
    if (!seededNegativeHistory) {
        return;
    }
    {
        bool readOk = read_sim_feedback(axisId, NULL, &seededVelocity, NULL, NULL);
        ASSERT_TRUE(readOk,
                    "ReadSimFeedback should expose the seeded negative actual velocity before the default current-direction case");
        if (!readOk) {
            return;
        }
    }
    ASSERT_TRUE(seededVelocity < -VELOCITY_EPSILON,
                "The default current-direction case should start from a proven negative actual velocity");

    init_movecontinuousabsolute(&cmd, axisId, 120.0f, 20.0f, 8.0f,
                                HYD_DIRECTION_POSITIVE,
                                0);
    rising_edge_scan(&cmd);
    inEndVelocityStep = run_until_inendvelocity(&cmd);
    assert_run_result(inEndVelocityStep,
                      "ENDVELOCITYDIRECTION=0 should default to CURRENT and eventually reach INENDVELOCITY");
    if (inEndVelocityStep <= 0) {
        return;
    }

    {
        bool readOk = read_sim_feedback(axisId, NULL, &finalVelocity, NULL, NULL);
        ASSERT_TRUE(readOk,
                    "ReadSimFeedback should expose the sustained velocity in the default current-direction case");
        if (!readOk) {
            return;
        }
    }
    ASSERT_TRUE(IEC_VAL(cmd.ERROR) == false,
                "ENDVELOCITYDIRECTION=0 should not error");
    ASSERT_TRUE(IEC_VAL(cmd.COMMANDABORTED) == false,
                "ENDVELOCITYDIRECTION=0 should not be aborted");
    ASSERT_TRUE(finalVelocity < 0.0f,
                "ENDVELOCITYDIRECTION=0 should sustain a negative end velocity when the actual velocity starts negative");
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
                                4);
    rising_edge_scan(&cmd);

    ASSERT_TRUE(IEC_VAL(cmd.ERROR) == true,
                "ENDVELOCITYDIRECTION values above CURRENT should be rejected");
    ASSERT_TRUE(IEC_VAL(cmd.ERRORID) == HYD_DIAG_CODE_COMMAND_NOT_ALLOWED,
                "Invalid ENDVELOCITYDIRECTION should surface COMMAND_NOT_ALLOWED");
}

static void test_current_end_velocity_direction_uses_velocity_then_last_active_direction(void) {
    HYD_MOVECONTINUOUSABSOLUTE cmd;
    int axisId;
    int inEndVelocityStep;
    bool seededNegativeHistory;
    HYD_REAL seededVelocity = 0.0f;
    HYD_REAL stoppedVelocity = 0.0f;
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
    {
        bool readOk = read_sim_feedback(axisId, NULL, &seededVelocity, NULL, NULL);
        ASSERT_TRUE(readOk,
                    "ReadSimFeedback should expose the seeded negative actual velocity before the current-direction case");
        if (!readOk) {
            return;
        }
    }
    ASSERT_TRUE(seededVelocity < -VELOCITY_EPSILON,
                "The current-direction negative-velocity case should start from a proven negative actual velocity");

    init_movecontinuousabsolute(&cmd, axisId, 120.0f, 20.0f, 8.0f,
                                HYD_DIRECTION_POSITIVE,
                                HYD_DIRECTION_CURRENT);
    rising_edge_scan(&cmd);
    inEndVelocityStep = run_until_inendvelocity(&cmd);
    assert_run_result(inEndVelocityStep,
                      "ENDVELOCITYDIRECTION=CURRENT should eventually reach INENDVELOCITY when starting from a negative actual velocity");
    if (inEndVelocityStep > 0) {
        bool readOk = read_sim_feedback(axisId, NULL, &finalVelocity, NULL, NULL);
        ASSERT_TRUE(readOk,
                    "ReadSimFeedback should expose the sustained velocity in the negative-velocity case");
        if (!readOk) {
            return;
        }
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
    {
        bool readOk = read_sim_feedback(axisId, NULL, &stoppedVelocity, NULL, NULL);
        ASSERT_TRUE(readOk,
                    "ReadSimFeedback should expose the stopped velocity after seeding the negative last-active-direction fallback");
        if (!readOk) {
            return;
        }
    }
    ASSERT_TRUE(fabs(stoppedVelocity) <= VELOCITY_EPSILON,
                "The current-direction fallback case should start from a proven near-zero actual velocity");

    init_movecontinuousabsolute(&cmd, axisId, 120.0f, 20.0f, 8.0f,
                                HYD_DIRECTION_POSITIVE,
                                HYD_DIRECTION_CURRENT);
    rising_edge_scan(&cmd);
    inEndVelocityStep = run_until_inendvelocity(&cmd);
    assert_run_result(inEndVelocityStep,
                      "ENDVELOCITYDIRECTION=CURRENT should eventually reach INENDVELOCITY in the zero-velocity fallback case");
    if (inEndVelocityStep > 0) {
        bool readOk = read_sim_feedback(axisId, NULL, &finalVelocity, NULL, NULL);
        ASSERT_TRUE(readOk,
                    "ReadSimFeedback should expose the sustained velocity in the zero-velocity fallback case");
        if (!readOk) {
            return;
        }
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
    assert_run_result(positionReachedStep,
                      "Same-direction MoveContinuousAbsolute should reach POSITIONREACHED");
    if (positionReachedStep <= 0) {
        return;
    }

    inEndVelocityStep = run_until_inendvelocity(&cmd);
    assert_run_result(inEndVelocityStep,
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
    {
        bool readOk = read_sim_feedback(axisId, NULL, &finalVelocity, NULL, NULL);
        ASSERT_TRUE(readOk,
                "ReadSimFeedback should expose the sustained velocity in the same-direction case");
        if (!readOk) {
            return;
        }
    }
    ASSERT_TRUE(fabs(finalVelocity) > 0.01f,
                "Axis velocity should remain non-zero after reaching the same-direction end velocity");
}

static void test_same_direction_keeps_velocity_across_position_to_sustain_switch(void) {
    HYD_MOVECONTINUOUSABSOLUTE cmd;
    HYD_MotionControlFB* core;
    int axisId;
    int positionReachedStep = RUN_TIMEOUT;
    HYD_REAL velocityAtCrossing = 0.0f;

    __HydMotion_framework_Init();
    axisId = create_sim_axis();
    ASSERT_TRUE(axisId >= 0, "CreateMotion should allocate a simulation axis");
    if (axisId < 0) {
        return;
    }

    core = __MK_GetPublic_MotionControlFB(axisId);
    ASSERT_TRUE(core != NULL, "Public motion FB should be available for sustain continuity test");
    if (core == NULL) {
        return;
    }

    init_movecontinuousabsolute(&cmd, axisId, 20.0f, 5.0f, 5.0f,
                                HYD_DIRECTION_POSITIVE,
                                HYD_DIRECTION_POSITIVE);
    rising_edge_scan(&cmd);

    for (int step = 0; step < MAX_SIM_STEPS; step++) {
        __HydMotion_framework_Publish();
        hold_true_scan(&cmd);

        if (IEC_VAL(cmd.POSITIONREACHED)) {
            positionReachedStep = step + 1;
            velocityAtCrossing = core->AXIS_REF.velocity;
            break;
        }
        if (IEC_VAL(cmd.ERROR)) {
            positionReachedStep = RUN_ERROR;
            break;
        }
        if (IEC_VAL(cmd.COMMANDABORTED)) {
            positionReachedStep = RUN_ABORTED;
            break;
        }
    }

    assert_run_result(positionReachedStep,
                      "Same-direction MoveContinuousAbsolute should reach POSITIONREACHED before sustain continuity is checked");
    if (positionReachedStep <= 0) {
        return;
    }

    __HydMotion_framework_Publish();
    hold_true_scan(&cmd);

    ASSERT_TRUE(velocityAtCrossing > 4.0f,
                "The approach segment should hit the target while already moving near the requested end velocity");
    ASSERT_TRUE(core->STATE.plannedVelocity >= velocityAtCrossing - 0.25f,
                "Position-to-sustain switch should preserve plannedVelocity instead of restarting from zero");
    ASSERT_TRUE(core->_simFeedback.targetVelocity >= velocityAtCrossing - 0.25f,
                "Position-to-sustain switch should preserve simulation feedback velocity instead of restarting from zero");
    ASSERT_TRUE(core->AXIS_REF.velocity >= velocityAtCrossing - 0.25f,
                "Position-to-sustain switch should preserve axis velocity instead of restarting from zero");
}

static void test_omitted_direction_reaches_position_and_end_velocity(void) {
    HYD_MOVECONTINUOUSABSOLUTE cmd;
    HYD_MotionControlFB* core;
    int axisId;
    int positionReachedStep;
    int inEndVelocityStep;
    HYD_REAL finalVelocity = 0.0f;
    bool readOk = false;

    __HydMotion_framework_Init();
    axisId = create_sim_axis();
    ASSERT_TRUE(axisId >= 0, "CreateMotion should allocate a simulation axis");
    if (axisId < 0) {
        return;
    }

    core = __MK_GetPublic_MotionControlFB(axisId);
    ASSERT_TRUE(core != NULL, "Public motion FB should be available for omitted-direction test");
    if (core == NULL) {
        return;
    }

    init_movecontinuousabsolute(&cmd, axisId, 20.0f, 5.0f, 5.0f,
                                HYD_DIRECTION_SHORTEST_WAY,
                                HYD_DIRECTION_POSITIVE);
    rising_edge_scan(&cmd);

    positionReachedStep = run_until_position_reached(&cmd);
    assert_run_result(positionReachedStep,
                      "MoveContinuousAbsolute should reach POSITIONREACHED even when Direction is omitted");
    if (positionReachedStep <= 0) {
        return;
    }

    inEndVelocityStep = run_until_inendvelocity(&cmd);
    assert_run_result(inEndVelocityStep,
                      "MoveContinuousAbsolute should eventually reach INENDVELOCITY even when Direction is omitted");
    if (inEndVelocityStep <= 0) {
        return;
    }

    ASSERT_TRUE(IEC_VAL(cmd.POSITIONREACHED) == true,
                "POSITIONREACHED should stay latched once the target position is crossed with omitted Direction");
    ASSERT_TRUE(IEC_VAL(cmd.INENDVELOCITY) == true,
                "INENDVELOCITY should become true after the sustain velocity settles with omitted Direction");
    ASSERT_TRUE(IEC_VAL(cmd.ERROR) == false,
                "MoveContinuousAbsolute should not error when Direction is omitted");
    ASSERT_TRUE(IEC_VAL(cmd.COMMANDABORTED) == false,
                "MoveContinuousAbsolute should not be aborted when Direction is omitted");

    readOk = read_sim_feedback(axisId, NULL, &finalVelocity, NULL, NULL);
    ASSERT_TRUE(readOk,
                "ReadSimFeedback should expose the sustained velocity in the omitted-direction case");
    if (!readOk) {
        return;
    }
    ASSERT_TRUE(fabsf(finalVelocity) > 0.01f,
                "Axis velocity should remain non-zero after the omitted-direction case reaches sustain");
}

static void test_negative_same_direction_latches_inendvelocity_on_target_crossing(void) {
    HYD_MOVECONTINUOUSABSOLUTE cmd;
    int axisId;
    int positionReachedStep;
    HYD_REAL finalVelocity = 0.0f;
    bool readOk = false;
    HYD_MotionControlFB *core;

    __HydMotion_framework_Init();
    axisId = create_sim_axis();
    ASSERT_TRUE(axisId >= 0, "CreateMotion should allocate a simulation axis");
    if (axisId < 0) {
        return;
    }
    core = __MK_GetPublic_MotionControlFB(axisId);
    ASSERT_TRUE(core != NULL, "CreateMotion should expose the simulation axis");
    if (core == NULL) {
        return;
    }
    HYD_MotionControlFB_SetSimulationPosition(core, 200.0f);

    init_movecontinuousabsolute(&cmd, axisId, 120.0f, 25.0f, 8.0f,
                                HYD_DIRECTION_NEGATIVE,
                                HYD_DIRECTION_NEGATIVE);
    rising_edge_scan(&cmd);

    positionReachedStep = run_until_position_reached(&cmd);
    assert_run_result(positionReachedStep,
                      "Negative same-direction MoveContinuousAbsolute should reach POSITIONREACHED");
    if (positionReachedStep <= 0) {
        return;
    }

    ASSERT_TRUE(IEC_VAL(cmd.INENDVELOCITY) == true,
                "Negative same-direction MoveContinuousAbsolute should latch INENDVELOCITY on the same scan as POSITIONREACHED when adaptation is disabled");
    ASSERT_TRUE(IEC_VAL(cmd.ERROR) == false,
                "Negative same-direction MoveContinuousAbsolute should not error before the target crossing");
    ASSERT_TRUE(IEC_VAL(cmd.COMMANDABORTED) == false,
                "Negative same-direction MoveContinuousAbsolute should not be aborted before the target crossing");

    readOk = read_sim_feedback(axisId, NULL, &finalVelocity, NULL, NULL);
    ASSERT_TRUE(readOk,
                "ReadSimFeedback should expose the sustained velocity in the negative same-direction case");
    if (!readOk) {
        return;
    }
    ASSERT_TRUE(finalVelocity < -VELOCITY_EPSILON,
                "Negative same-direction MoveContinuousAbsolute should sustain a negative end velocity after crossing the target");
}

static void test_adapt_lowers_crossing_velocity_when_distance_is_too_short_to_accelerate(void) {
    HYD_MOVECONTINUOUSABSOLUTE cmd;
    int axisId;
    int positionReachedStep;
    bool reversedBeforeTarget = false;
    HYD_REAL observedVelocity = 0.0f;
    bool sawValidFeedback = false;

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

        if (!read_sim_feedback(axisId, NULL, &observedVelocity, NULL, NULL)) {
            ASSERT_TRUE(false,
                        "ReadSimFeedback should expose observed velocity during the short accelerate-up adapt case");
            return;
        }
        sawValidFeedback = true;
        if (observedVelocity < -VELOCITY_EPSILON) {
            reversedBeforeTarget = true;
        }
        if (IEC_VAL(cmd.POSITIONREACHED)) {
            positionReachedStep = step + 1;
            break;
        }
        if (IEC_VAL(cmd.ERROR)) {
            positionReachedStep = RUN_ERROR;
            break;
        }
        if (IEC_VAL(cmd.COMMANDABORTED)) {
            positionReachedStep = RUN_ABORTED;
            break;
        }
    }

    assert_run_result(positionReachedStep,
                      "Adapt mode should still reach POSITIONREACHED on a short accelerate-up move");
    if (positionReachedStep <= 0) {
        return;
    }
    ASSERT_TRUE(IEC_VAL(cmd.ERROR) == false,
                "Adapt mode should not error on a short accelerate-up move");
    ASSERT_TRUE(IEC_VAL(cmd.COMMANDABORTED) == false,
                "Adapt mode should not be aborted on a short accelerate-up move");
    ASSERT_TRUE(sawValidFeedback == true,
                "Adapt mode should observe at least one valid public velocity sample on the short accelerate-up move");
    ASSERT_TRUE(reversedBeforeTarget == false,
                "Adapt mode should not reverse direction before the first target reach on a short accelerate-up move");
}

static void test_adapt_raises_crossing_velocity_when_distance_is_too_short_to_decelerate(void) {
    HYD_MOVECONTINUOUSABSOLUTE cmd;
    HYD_MOVEVELOCITY mv;
    int axisId;
    int positionReachedStep;
    bool reversedBeforeTarget = false;
    HYD_REAL seededPosition = 0.0f;
    HYD_REAL observedVelocity = 0.0f;
    bool seededHighVelocity = false;
    bool sawValidFeedback = false;
    const HYD_REAL targetPosition = 6.0f;
    const HYD_REAL endVelocity = 5.0f;
    const HYD_REAL deceleration = 100.0f;

    __HydMotion_framework_Init();
    axisId = create_sim_axis();
    ASSERT_TRUE(axisId >= 0, "CreateMotion should allocate a simulation axis");
    if (axisId < 0) {
        return;
    }

    memset(&mv, 0, sizeof(mv));
    IEC_VAL(mv.EN) = true;
    IEC_VAL(mv.AXISID) = axisId;
    IEC_VAL(mv.VELOCITY) = 35.0f;
    IEC_VAL(mv.ACCELERATION) = 100.0f;
    IEC_VAL(mv.DECELERATION) = 100.0f;
    IEC_VAL(mv.DIRECTION) = HYD_DIRECTION_POSITIVE;
    IEC_VAL(mv.EXECUTE) = true;
    __mcl_cmd_MoveVelocity(&mv);

    for (int step = 0; step < MAX_SIM_STEPS; step++) {
        __HydMotion_framework_Publish();
        hold_movevelocity_scan(&mv);
        if (!read_sim_feedback(axisId, &seededPosition, &observedVelocity, NULL, NULL)) {
            ASSERT_TRUE(false,
                        "ReadSimFeedback should expose seeded position and velocity while preparing the short decelerate-down adapt case");
            return;
        }
        if (observedVelocity >= 30.0f) {
            seededHighVelocity = true;
            break;
        }
    }

    ASSERT_TRUE(seededHighVelocity,
                "Public MoveVelocity should seed a proven high positive actual velocity for the short decelerate-down case");
    if (!seededHighVelocity) {
        return;
    }
    ASSERT_TRUE(seededPosition < targetPosition,
                "The short decelerate-down case should still be before the target when the high seed velocity is observed");
    ASSERT_TRUE((targetPosition - seededPosition) <
                ((observedVelocity * observedVelocity) - (endVelocity * endVelocity)) / (2.0f * deceleration),
                "The short decelerate-down case should prove the remaining distance is too short to decelerate from the seeded velocity to the requested end velocity");

    init_movecontinuousabsolute(&cmd, axisId, targetPosition, 35.0f, endVelocity,
                                HYD_DIRECTION_POSITIVE,
                                HYD_DIRECTION_POSITIVE);
    IEC_VAL(cmd.ADAPTENDVELTOAVOIDOVERSHOOT) = true;
    rising_edge_scan(&cmd);

    positionReachedStep = -1;
    for (int step = 0; step < MAX_SIM_STEPS; step++) {
        __HydMotion_framework_Publish();
        hold_true_scan(&cmd);

        if (!read_sim_feedback(axisId, NULL, &observedVelocity, NULL, NULL)) {
            ASSERT_TRUE(false,
                        "ReadSimFeedback should expose observed velocity during the short decelerate-down adapt case");
            return;
        }
        sawValidFeedback = true;
        if (observedVelocity < -VELOCITY_EPSILON) {
            reversedBeforeTarget = true;
        }
        if (IEC_VAL(cmd.POSITIONREACHED)) {
            positionReachedStep = step + 1;
            break;
        }
        if (IEC_VAL(cmd.ERROR)) {
            positionReachedStep = RUN_ERROR;
            break;
        }
        if (IEC_VAL(cmd.COMMANDABORTED)) {
            positionReachedStep = RUN_ABORTED;
            break;
        }
    }

    assert_run_result(positionReachedStep,
                      "Adapt mode should still reach POSITIONREACHED on a short decelerate-down move");
    if (positionReachedStep <= 0) {
        return;
    }
    ASSERT_TRUE(IEC_VAL(cmd.ERROR) == false,
                "Adapt mode should not error on a short decelerate-down move");
    ASSERT_TRUE(IEC_VAL(cmd.COMMANDABORTED) == false,
                "Adapt mode should not be aborted on a short decelerate-down move");
    ASSERT_TRUE(sawValidFeedback == true,
                "Adapt mode should observe at least one valid public velocity sample on the short decelerate-down move");
    ASSERT_TRUE(reversedBeforeTarget == false,
                "Adapt mode should not reverse direction before the first target reach on a short decelerate-down move");
}

static void test_reverse_sustain_delays_inendvelocity(void) {
    HYD_MOVECONTINUOUSABSOLUTE cmd;
    int axisId;
    int positionReachedStep = -1;
    int inEndVelocityStep = -1;
    HYD_REAL finalVelocity = 0.0f;
    bool readOk = false;

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
        if (IEC_VAL(cmd.ERROR)) {
            positionReachedStep = RUN_ERROR;
            inEndVelocityStep = RUN_ERROR;
            break;
        }
        if (IEC_VAL(cmd.COMMANDABORTED)) {
            positionReachedStep = RUN_ABORTED;
            inEndVelocityStep = RUN_ABORTED;
            break;
        }
    }

    assert_run_result(positionReachedStep,
                      "Reverse sustain should still reach POSITIONREACHED");
    assert_run_result(inEndVelocityStep,
                      "Reverse sustain should eventually reach INENDVELOCITY");
    if (positionReachedStep <= 0 || inEndVelocityStep <= 0) {
        return;
    }
    ASSERT_TRUE(positionReachedStep > 0 && inEndVelocityStep > positionReachedStep,
                "Reverse sustain should latch POSITIONREACHED before INENDVELOCITY");
    readOk = read_sim_feedback(axisId, NULL, &finalVelocity, NULL, NULL);
    ASSERT_TRUE(readOk,
                "ReadSimFeedback should expose the sustained velocity in the reverse case");
    if (!readOk) {
        return;
    }
    ASSERT_TRUE(finalVelocity < 0.0f,
                "Final sustained velocity should be negative for reverse sustain");
}

static void test_pressure_limit_can_hold_positionreached_true_while_inendvelocity_stays_false(void) {
    HYD_MOVECONTINUOUSABSOLUTE cmd;
    HYD_MotionControlFB* fb;
    int axisId;
    bool sawSplitState = false;
    bool sawPositionReached = false;
    int splitStateStep = RUN_TIMEOUT;
    HYD_REAL manualPosition = 70.0f;
    HYD_REAL manualVelocity = 18.0f;
    HYD_REAL timestamp = 0.0f;
    const HYD_REAL dt = 0.01f;

    __HydMotion_framework_Init();
    axisId = create_manual_axis();
    ASSERT_TRUE(axisId >= 0, "CreateMotion should allocate a manual-feedback axis");
    if (axisId < 0) {
        return;
    }
    fb = __MK_GetPublic_MotionControlFB(axisId);
    ASSERT_TRUE(fb != NULL, "Pressure-limit split-state test should expose the public FB");
    if (fb == NULL) {
        return;
    }
    fb->PRESSURE_LIMIT = 2.0f;

    ASSERT_TRUE(set_axis_feedback(axisId, manualPosition, manualVelocity, manualVelocity, 20.0f, timestamp),
                "SetAxisFeedback should seed the manual axis before the pressure-limit case");
    init_movecontinuousabsolute(&cmd, axisId, 80.0f, 20.0f, 12.0f,
                                HYD_DIRECTION_POSITIVE,
                                HYD_DIRECTION_POSITIVE);
    IEC_VAL(cmd.PRESSURELIMIT) = 0.0f;
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
            splitStateStep = step + 1;
            break;
        }
        if (IEC_VAL(cmd.ERROR)) {
            splitStateStep = RUN_ERROR;
            break;
        }
        if (IEC_VAL(cmd.COMMANDABORTED)) {
            splitStateStep = RUN_ABORTED;
            break;
        }
    }

    assert_run_result(splitStateStep,
                      "Pressure limiting should allow a scan where POSITIONREACHED stays true while INENDVELOCITY remains false");
    if (splitStateStep <= 0) {
        return;
    }
    ASSERT_TRUE(sawPositionReached,
                "Pressure-limited run should still reach POSITIONREACHED");
    ASSERT_TRUE(sawSplitState,
                "Pressure limiting should allow a scan where POSITIONREACHED stays true while INENDVELOCITY remains false");
}

static void test_pressure_limit_fault_surfaces_as_error(void) {
    HYD_MOVECONTINUOUSABSOLUTE cmd;
    HYD_MotionControlFB* fb;
    int axisId;

    __HydMotion_framework_Init();
    axisId = create_sim_axis();
    ASSERT_TRUE(axisId >= 0, "CreateMotion should allocate a simulation axis");
    if (axisId < 0) {
        return;
    }
    fb = __MK_GetPublic_MotionControlFB(axisId);
    ASSERT_TRUE(fb != NULL, "Fault-escalation test should expose the public FB");
    if (fb == NULL) {
        return;
    }

    fb->PRESSURE_LIMIT = 1.0f;
    init_movecontinuousabsolute(&cmd, axisId, 30.0f, 20.0f, 10.0f,
                                HYD_DIRECTION_POSITIVE,
                                HYD_DIRECTION_POSITIVE);
    IEC_VAL(cmd.ACCELERATION) = 120.0f;
    rising_edge_scan(&cmd);

    for (int step = 0; step < 3000; step++) {
        fb->AXIS_REF.pressure = 40.0f;
        __HydMotion_framework_Publish();
        hold_true_scan(&cmd);
        if (IEC_VAL(cmd.ERROR)) {
            break;
        }
    }

    ASSERT_TRUE(IEC_VAL(cmd.ERROR) == true,
                "Fault-level pressure limiting should surface ERROR on the command");
    ASSERT_TRUE(IEC_VAL(cmd.BUSY) == false,
                "Fault-level pressure limiting should clear Busy on the command");
}

static void test_chained_same_direction_segments_keep_end_velocity_on_takeover_scan(void) {
    HYD_MOVECONTINUOUSABSOLUTE first;
    HYD_MOVECONTINUOUSABSOLUTE second;
    HYD_MotionControlFB* core;
    int axisId;
    int inEndVelocityStep = RUN_TIMEOUT;
    HYD_REAL velocityBeforeTakeover = 0.0f;
    HYD_REAL plannedVelocityAfterTakeover;
    HYD_REAL feedbackVelocityAfterTakeover;
    HYD_REAL observedVelocityAfterTakeover = 0.0f;
    bool readOk;

    __HydMotion_framework_Init();
    axisId = create_sim_axis();
    ASSERT_TRUE(axisId >= 0, "CreateMotion should allocate a simulation axis");
    if (axisId < 0) {
        return;
    }

    core = __MK_GetPublic_MotionControlFB(axisId);
    ASSERT_TRUE(core != NULL, "Public motion FB should be available for chained takeover test");
    if (core == NULL) {
        return;
    }

    init_movecontinuousabsolute(&first, axisId, 20.0f, 5.0f, 5.0f,
                                HYD_DIRECTION_POSITIVE,
                                HYD_DIRECTION_POSITIVE);
    rising_edge_scan(&first);

    for (int step = 0; step < MAX_SIM_STEPS; step++) {
        __HydMotion_framework_Publish();
        hold_true_scan(&first);

        if (IEC_VAL(first.INENDVELOCITY)) {
            inEndVelocityStep = step + 1;
            velocityBeforeTakeover = core->AXIS_REF.velocity;
            break;
        }
        if (IEC_VAL(first.ERROR)) {
            inEndVelocityStep = RUN_ERROR;
            break;
        }
        if (IEC_VAL(first.COMMANDABORTED)) {
            inEndVelocityStep = RUN_ABORTED;
            break;
        }
    }

    assert_run_result(inEndVelocityStep,
                      "The first MoveContinuousAbsolute should reach INENDVELOCITY before chaining");
    if (inEndVelocityStep <= 0) {
        return;
    }

    init_movecontinuousabsolute(&second, axisId, 40.0f, 10.0f, 10.0f,
                                HYD_DIRECTION_POSITIVE,
                                HYD_DIRECTION_POSITIVE);

    /* Reproduce the field scan order:
     *   previous scan observed FB1.INENDVELOCITY = TRUE
     *   current scan executes FB1() -> FB2() -> Publish() -> read outputs
     */
    hold_true_scan(&first);
    rising_edge_scan(&second);
    __HydMotion_framework_Publish();

    plannedVelocityAfterTakeover = core->STATE.plannedVelocity;
    feedbackVelocityAfterTakeover = core->_simFeedback.targetVelocity;
    readOk = read_sim_feedback(axisId, NULL, &observedVelocityAfterTakeover, NULL, NULL);
    ASSERT_TRUE(readOk,
                "ReadSimFeedback should expose the chained takeover velocity");
    if (!readOk) {
        return;
    }

    ASSERT_TRUE(velocityBeforeTakeover > 4.0f,
                "The first segment should already sustain its end velocity before the chained takeover");
    ASSERT_TRUE(plannedVelocityAfterTakeover >= velocityBeforeTakeover - 0.25f,
                "Chained MoveContinuousAbsolute should preserve the previous end velocity in plannedVelocity on the takeover scan");
    ASSERT_TRUE(feedbackVelocityAfterTakeover >= velocityBeforeTakeover - 0.25f,
                "Chained MoveContinuousAbsolute should preserve the previous end velocity in simulation feedback on the takeover scan");
    ASSERT_TRUE(observedVelocityAfterTakeover >= velocityBeforeTakeover - 0.25f,
                "Chained MoveContinuousAbsolute should preserve the previous end velocity in axis feedback on the takeover scan");
}

static void test_low_acceleration_start_does_not_false_trip_position_deviation(void) {
    HYD_MOVECONTINUOUSABSOLUTE cmd;
    HYD_MotionControlFB* core;
    int axisId;
    bool sawSlowStartup = false;
    bool sawPositionDeviation = false;
    int positionDeviationStep = RUN_TIMEOUT;

    __HydMotion_framework_Init();
    axisId = create_sim_axis();
    ASSERT_TRUE(axisId >= 0, "CreateMotion should allocate a simulation axis");
    if (axisId < 0) {
        return;
    }

    core = __MK_GetPublic_MotionControlFB(axisId);
    ASSERT_TRUE(core != NULL, "Public motion FB should be available for low-acceleration startup test");
    if (core == NULL) {
        return;
    }

    init_movecontinuousabsolute(&cmd, axisId, 66.0f, 22.0f, 22.0f,
                                HYD_DIRECTION_POSITIVE,
                                HYD_DIRECTION_POSITIVE);
    IEC_VAL(cmd.ACCELERATION) = 0.056f;
    IEC_VAL(cmd.DECELERATION) = 0.056f;
    rising_edge_scan(&cmd);

    for (int step = 0; step < 1200; step++) {
        __HydMotion_framework_Publish();
        hold_true_scan(&cmd);

        if (core->AXIS_REF.velocity > 0.0f &&
            core->AXIS_REF.velocity < 1.0f &&
            core->AXIS_REF.position < 1.0f) {
            sawSlowStartup = true;
        }

        if (core->DIAGNOSTIC.code == HYD_DIAG_CODE_POSITION_DEVIATION ||
            core->DIAGNOSTIC.positionDeviation) {
            sawPositionDeviation = true;
            positionDeviationStep = step + 1;
            break;
        }

        if (IEC_VAL(cmd.POSITIONREACHED) || IEC_VAL(cmd.INENDVELOCITY)) {
            break;
        }
    }

    ASSERT_TRUE(sawSlowStartup,
                "The low-acceleration test should spend time in a slow startup phase far from the target");
    ASSERT_TRUE(!sawPositionDeviation,
                "Low-acceleration MoveContinuousAbsolute should not report POSITION_DEVIATION during normal startup far from the target");
    if (sawPositionDeviation) {
        printf("  observed false POSITION_DEVIATION at step %d: pos=%.6f vel=%.6f plannedVel=%.6f diag=%d\n",
               positionDeviationStep,
               core->AXIS_REF.position,
               core->AXIS_REF.velocity,
               core->STATE.plannedVelocity,
               core->DIAGNOSTIC.code);
    }
}

static void test_stop_takes_over_and_sets_commandaborted(void) {
    HYD_MOVECONTINUOUSABSOLUTE cmd;
    HYD_STOP stop;
    int axisId;
    int commandAbortedStep = RUN_TIMEOUT;

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
    __mcl_cmd_Stop(&stop);

    for (int step = 0; step < MAX_SIM_STEPS; step++) {
        __HydMotion_framework_Publish();
        hold_true_scan(&cmd);

        IEC_VAL(stop.EXECUTE) = true;
        __mcl_cmd_Stop(&stop);

        if (IEC_VAL(cmd.COMMANDABORTED)) {
            commandAbortedStep = step + 1;
            break;
        }
        if (IEC_VAL(cmd.ERROR)) {
            commandAbortedStep = RUN_ERROR;
            break;
        }
        if (IEC_VAL(stop.ERROR)) {
            commandAbortedStep = RUN_ERROR;
            break;
        }
        if (IEC_VAL(stop.DONE)) {
            ASSERT_TRUE(false,
                        "Stop should not reach DONE before MoveContinuousAbsolute reports COMMANDABORTED");
            return;
        }
    }

    assert_run_result(commandAbortedStep,
                      "Stop takeover should eventually set COMMANDABORTED on MoveContinuousAbsolute");
    if (commandAbortedStep <= 0) {
        return;
    }
    ASSERT_TRUE(IEC_VAL(cmd.COMMANDABORTED) == true,
                "MoveContinuousAbsolute should report COMMANDABORTED after Stop takeover");
    ASSERT_TRUE(IEC_VAL(cmd.BUSY) == false,
                "MoveContinuousAbsolute BUSY should clear after Stop takeover");
}

int main(void) {
    printf("=== MoveContinuousAbsolute integration ===\n\n");

    test_zero_end_velocity_direction_defaults_to_current();
    test_rejects_invalid_end_velocity_direction();
    test_current_end_velocity_direction_uses_velocity_then_last_active_direction();
    test_same_direction_reaches_position_and_end_velocity();
    test_same_direction_keeps_velocity_across_position_to_sustain_switch();
    test_omitted_direction_reaches_position_and_end_velocity();
    test_negative_same_direction_latches_inendvelocity_on_target_crossing();
    test_adapt_lowers_crossing_velocity_when_distance_is_too_short_to_accelerate();
    test_adapt_raises_crossing_velocity_when_distance_is_too_short_to_decelerate();
    test_reverse_sustain_delays_inendvelocity();
    test_plc_pressure_feedback_and_movecontinuousabsolute_limit_use_bar();
    test_pressure_limit_can_hold_positionreached_true_while_inendvelocity_stays_false();
    test_pressure_limit_fault_surfaces_as_error();
    test_chained_same_direction_segments_keep_end_velocity_on_takeover_scan();
    test_low_acceleration_start_does_not_false_trip_position_deviation();
    test_stop_takes_over_and_sets_commandaborted();

    printf("\n=== Results: %d/%d passed ===\n", tests_passed, tests_run);
    return (tests_failed == 0) ? 0 : 1;
}
