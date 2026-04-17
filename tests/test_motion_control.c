#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "diagnostics.h"
#include "motion_control.h"

static void init_controller(HDY_MotionControlFB* fb) {
    HDY_MotionControlFB_Init(fb);
    fb->EN = true;
    fb->FLOW_TO_PUMP_SPEED_GAIN = 100.0;
    fb->PUMP_SPEED_LIMIT = 3000.0;
}

static HDY_MotionSegment make_position_segment(const char* name,
                                               HDY_REAL targetPosition,
                                               HDY_MotionDirection direction) {
    HDY_MotionSegment segment = {0};
    strncpy(segment.name, name, HDY_NAME_MAX - 1);
    segment.name[HDY_NAME_MAX - 1] = '\0';
    segment.type = HDY_SEGMENT_TYPE_CLAMPING;
    segment.planner = HDY_PLANNER_POSITION_BASED;
    segment.mode = HDY_MODE_POSITION;
    segment.endCondition = HDY_END_POSITION;
    segment.direction = direction;
    segment.targetPosition = targetPosition;
    segment.targetFlow = 12.0;
    segment.targetPressure = 8.0;
    segment.maxAcceleration = 20.0;
    segment.maxVelocity = 15.0;
    segment.maxFlow = 18.0;
    segment.duration = 1.0;
    segment.tolerance = 0.0;
    segment.positionTolerance = 0.1;
    segment.pressureTolerance = 0.5;
    segment.flowTolerance = 0.2;
    segment.velocityTolerance = 0.5;
    segment.timeoutLimit = 2.0;
    segment.velocityToFlowGain = 1.0;
    segment.pressureRampRate = 5.0;
    return segment;
}

static HDY_MotionSegment make_time_segment(const char* name,
                                           HDY_TIME duration,
                                           HDY_MotionDirection direction) {
    HDY_MotionSegment segment = {0};
    strncpy(segment.name, name, HDY_NAME_MAX - 1);
    segment.name[HDY_NAME_MAX - 1] = '\0';
    segment.type = HDY_SEGMENT_TYPE_HOLDING;
    segment.planner = HDY_PLANNER_TIME_BASED;
    segment.mode = HDY_MODE_SPEED_RAMP;
    segment.endCondition = HDY_END_TIME;
    segment.direction = direction;
    segment.targetPosition = 10.0;
    segment.targetFlow = 8.0;
    segment.targetPressure = 10.0;
    segment.maxAcceleration = 10.0;
    segment.maxVelocity = 6.0;
    segment.maxFlow = 10.0;
    segment.duration = duration;
    segment.tolerance = 0.0;
    segment.positionTolerance = 0.1;
    segment.pressureTolerance = 0.5;
    segment.flowTolerance = 0.2;
    segment.velocityTolerance = 0.5;
    segment.timeoutLimit = duration + 0.5;
    segment.velocityToFlowGain = 1.0;
    segment.pressureRampRate = 2.0;
    return segment;
}

static HDY_MotionSegment make_pressure_segment(const char* name,
                                               HDY_TIME duration,
                                               HDY_REAL targetPressure,
                                               HDY_REAL targetFlow) {
    HDY_MotionSegment segment = {0};
    strncpy(segment.name, name, HDY_NAME_MAX - 1);
    segment.name[HDY_NAME_MAX - 1] = '\0';
    segment.type = HDY_SEGMENT_TYPE_HOLDING;
    segment.planner = HDY_PLANNER_TIME_BASED;
    segment.mode = HDY_MODE_PRESSURE_CLOSED_LOOP;
    segment.endCondition = HDY_END_TIME;
    segment.direction = HDY_DIRECTION_HOLD;
    segment.targetPosition = 10.0;
    segment.targetFlow = targetFlow;
    segment.targetPressure = targetPressure;
    segment.maxAcceleration = 0.0;
    segment.maxVelocity = 0.0;
    segment.maxFlow = 12.0;
    segment.duration = duration;
    segment.tolerance = 0.0;
    segment.positionTolerance = 0.1;
    segment.pressureTolerance = 0.2;
    segment.flowTolerance = 0.2;
    segment.velocityTolerance = 0.5;
    segment.timeoutLimit = duration + 0.5;
    segment.velocityToFlowGain = 0.0;
    segment.pressureRampRate = 0.0;
    segment.pressureDerivativeFilterAlpha = 1.0;
    return segment;
}

static void test_load_recipe_requires_start_command(void) {
    HDY_MotionControlFB fb;
    HDY_MotionSegment recipe[1];

    printf("Testing LoadRecipe idle semantics...\n");
    init_controller(&fb);
    recipe[0] = make_position_segment("Clamp", 10.0, HDY_DIRECTION_EXTEND);

    assert(HDY_MotionControlFB_LoadRecipe(&fb, recipe, 1));
    assert(fb.RECIPE_SIZE == 1U);
    assert(!fb.ACTIVE);
    assert(!fb.FAULT);
    assert(fb.STATUS == HDY_STATUS_READY);
    assert(fb.FB_STATE == HDY_FB_STATE_READY);
    assert(!fb.STATE.active);
    assert(!fb.FINISHED);
    assert(!fb.SEGMENT_COMPLETED);
    assert(fb.STATE.currentSegmentName[0] == '\0');
    assert(fb.STATE.status == HDY_STATUS_READY);

    fb.AXIS_REF.position = 0.0;
    fb.AXIS_REF.timestamp = 0.0;
    HDY_MotionControlFB_Execute(&fb);
    assert(fb.PUMP_SPEED == 0.0);
    assert(!fb.ACTIVE);
    assert(!fb.SEGMENT_COMPLETED);
    assert(fb.STATUS == HDY_STATUS_READY);
    assert(fb.STATE.plannedDirection == HDY_DIRECTION_HOLD);
    printf("✓ LoadRecipe idle semantics test passed\n");
}

static void test_start_segment_and_segment_changed_pulse(void) {
    HDY_MotionControlFB fb;
    HDY_MotionSegment recipe[1];

    printf("Testing StartSegment execution semantics...\n");
    init_controller(&fb);
    recipe[0] = make_position_segment("Inject", 20.0, HDY_DIRECTION_EXTEND);
    assert(HDY_MotionControlFB_LoadRecipe(&fb, recipe, 1));

    fb.AXIS_REF.position = 0.0;
    fb.AXIS_REF.pressure = 2.0;
    fb.AXIS_REF.timestamp = 0.0;
    assert(HDY_MotionControlFB_StartSegment(&fb, 0, fb.AXIS_REF.timestamp));
    assert(!fb.ACTIVE);
    assert(fb.STATUS == HDY_STATUS_READY);
    assert(fb.FB_STATE == HDY_FB_STATE_READY);

    HDY_MotionControlFB_Execute(&fb);
    assert(fb.ACTIVE);
    assert(!fb.FAULT);
    assert(fb.STATUS == HDY_STATUS_RUNNING);
    assert(fb.FB_STATE == HDY_FB_STATE_RUNNING);
    assert(fb.STATE.status == HDY_STATUS_RUNNING);
    assert(fb.SEGMENT_CHANGED);
    assert(!fb.SEGMENT_COMPLETED);
    assert(fb.PUMP_SPEED > 0.0);
    assert(fb.STATE.plannedVelocity > 0.0);
    assert(fb.STATE.plannedFlow > 0.0);
    assert(fb.STATE.plannedDirection == HDY_DIRECTION_EXTEND);
    assert(strcmp(fb.STATE.currentSegmentName, "Inject") == 0);

    fb.AXIS_REF.timestamp = 0.1;
    HDY_MotionControlFB_Execute(&fb);
    assert(!fb.SEGMENT_CHANGED);
    assert(fb.ACTIVE);
    assert(fb.PUMP_SPEED > 0.0);
    assert(fb.STATUS == HDY_STATUS_RUNNING);
    printf("✓ StartSegment execution semantics test passed\n");
}

static void test_start_segment_command_input(void) {
    HDY_MotionControlFB fb;
    HDY_MotionSegment recipe[1];

    printf("Testing START_SEGMENT command input semantics...\n");
    init_controller(&fb);
    recipe[0] = make_position_segment("CommandStart", 12.0, HDY_DIRECTION_EXTEND);
    assert(HDY_MotionControlFB_LoadRecipe(&fb, recipe, 1));

    fb.AXIS_REF.position = 0.0;
    fb.AXIS_REF.pressure = 1.5;
    fb.AXIS_REF.timestamp = 0.0;
    fb.START_SEGMENT = true;
    fb.START_SEGMENT_INDEX = 0U;

    HDY_MotionControlFB_Execute(&fb);
    assert(!fb.START_SEGMENT);
    assert(fb.ACTIVE);
    assert(!fb.FAULT);
    assert(fb.SEGMENT_CHANGED);
    assert(fb.STATUS == HDY_STATUS_RUNNING);
    assert(fb.STATE.plannedDirection == HDY_DIRECTION_EXTEND);
    assert(strcmp(fb.STATE.currentSegmentName, "CommandStart") == 0);
    printf("✓ START_SEGMENT command input semantics test passed\n");
}

static void test_start_segment_input_uses_rising_edge(void) {
    HDY_MotionControlFB fb;
    HDY_MotionSegment recipe[1];

    printf("Testing START_SEGMENT rising-edge semantics...\n");
    init_controller(&fb);
    recipe[0] = make_position_segment("EdgeStart", 10.0, HDY_DIRECTION_EXTEND);
    assert(HDY_MotionControlFB_LoadRecipe(&fb, recipe, 1));

    fb.AXIS_REF.position = 0.0;
    fb.AXIS_REF.pressure = 1.0;
    fb.AXIS_REF.timestamp = 0.0;
    fb.START_SEGMENT = true;
    fb.START_SEGMENT_INDEX = 0U;
    HDY_MotionControlFB_Scan(&fb);
    assert(fb.ACTIVE);
    assert(fb.SEGMENT_CHANGED);
    assert(fb.FB_STATE == HDY_FB_STATE_RUNNING);

    fb.AXIS_REF.timestamp = 0.1;
    fb.START_SEGMENT = true;
    fb.START_SEGMENT_INDEX = 0U;
    HDY_MotionControlFB_Scan(&fb);
    assert(!fb.SEGMENT_CHANGED);
    assert(fb.ACTIVE);
    assert(fb.STATE.references.elapsedTime > 0.09);

    fb.AXIS_REF.timestamp = 0.2;
    fb.START_SEGMENT = false;
    HDY_MotionControlFB_Scan(&fb);

    fb.AXIS_REF.timestamp = 0.3;
    fb.START_SEGMENT = true;
    fb.START_SEGMENT_INDEX = 0U;
    HDY_MotionControlFB_Scan(&fb);
    assert(!fb.SEGMENT_CHANGED);
    assert(fb.ACTIVE);
    assert(fb.STATE.references.elapsedTime > 0.29);
    printf("✓ START_SEGMENT rising-edge test passed\n");
}

static void test_cycle_does_not_sample_command_inputs_without_scan(void) {
    HDY_MotionControlFB fb;
    HDY_MotionSegment recipe[1];

    printf("Testing Cycle() command/input separation...\n");
    init_controller(&fb);
    recipe[0] = make_position_segment("CycleOnly", 10.0, HDY_DIRECTION_EXTEND);
    assert(HDY_MotionControlFB_LoadRecipe(&fb, recipe, 1));

    fb.AXIS_REF.position = 0.0;
    fb.AXIS_REF.pressure = 1.0;
    fb.AXIS_REF.timestamp = 0.0;
    fb.START_SEGMENT = true;
    fb.START_SEGMENT_INDEX = 0U;

    HDY_MotionControlFB_Cycle(&fb);
    assert(fb.START_SEGMENT);
    assert(!fb.ACTIVE);
    assert(!fb.SEGMENT_CHANGED);
    assert(fb.STATUS == HDY_STATUS_READY);
    assert(fb.FB_STATE == HDY_FB_STATE_READY);
    assert(fb._pendingCommand == HDY_CMD_NONE);

    HDY_MotionControlFB_Scan(&fb);
    assert(!fb.START_SEGMENT);
    assert(fb._pendingCommand == HDY_CMD_NONE);
    assert(fb.ACTIVE);
    assert(fb.SEGMENT_CHANGED);
    assert(fb.STATUS == HDY_STATUS_RUNNING);
    assert(fb.FB_STATE == HDY_FB_STATE_RUNNING);
    printf("✓ Cycle() command/input separation test passed\n");
}

static void test_cycle_consumes_api_queued_start_command(void) {
    HDY_MotionControlFB fb;
    HDY_MotionSegment recipe[1];

    printf("Testing Cycle() consumes queued API commands...\n");
    init_controller(&fb);
    recipe[0] = make_position_segment("CycleQueuedStart", 12.0, HDY_DIRECTION_EXTEND);
    assert(HDY_MotionControlFB_LoadRecipe(&fb, recipe, 1));

    fb.AXIS_REF.position = 0.0;
    fb.AXIS_REF.pressure = 1.0;
    fb.AXIS_REF.timestamp = 0.0;
    assert(HDY_MotionControlFB_StartSegment(&fb, 0, 0.0));
    assert(fb._pendingCommand == HDY_CMD_START);
    assert(!fb.ACTIVE);

    HDY_MotionControlFB_Cycle(&fb);
    assert(fb._pendingCommand == HDY_CMD_NONE);
    assert(fb.ACTIVE);
    assert(fb.SEGMENT_CHANGED);
    assert(fb.STATUS == HDY_STATUS_RUNNING);
    assert(fb.FB_STATE == HDY_FB_STATE_RUNNING);
    assert(strcmp(fb.STATE.currentSegmentName, "CycleQueuedStart") == 0);
    printf("✓ Cycle() queued API command test passed\n");
}

static void test_start_rejected_in_disabled_state(void) {
    HDY_MotionControlFB fb;
    HDY_MotionSegment recipe[1];

    printf("Testing Start command legality in DISABLED state...\n");
    init_controller(&fb);
    recipe[0] = make_position_segment("DisabledStart", 10.0, HDY_DIRECTION_EXTEND);
    assert(HDY_MotionControlFB_LoadRecipe(&fb, recipe, 1));

    fb.EN = false;
    assert(!HDY_MotionControlFB_StartSegment(&fb, 0, 0.0));
    assert(!fb.ACTIVE);
    assert(fb.DIAGNOSTIC.code == HDY_DIAG_CODE_COMMAND_NOT_ALLOWED);
    assert(fb.DIAGNOSTIC.source == HDY_DIAG_SOURCE_COMMAND);
    assert(fb.DIAGNOSTIC.recovery == HDY_DIAG_RECOVERY_CHECK_COMMAND);
    assert(fb.DIAGNOSTIC.protectionAction == HDY_PROTECTION_ACTION_WARNING);
    assert(strstr(fb.DIAGNOSTIC.message, "START") != NULL);
    assert(strstr(fb.DIAGNOSTIC.message, "DISABLED") != NULL);
    printf("✓ Start command legality in DISABLED state test passed\n");
}

static void test_start_command_rejected_while_running(void) {
    HDY_MotionControlFB fb;
    HDY_MotionSegment recipe[1];

    printf("Testing Start command legality while running...\n");
    init_controller(&fb);
    recipe[0] = make_time_segment("BusyStart", 1.0, HDY_DIRECTION_EXTEND);
    assert(HDY_MotionControlFB_LoadRecipe(&fb, recipe, 1));

    fb.AXIS_REF.position = 0.0;
    fb.AXIS_REF.velocity = 0.0;
    fb.AXIS_REF.flow = 0.0;
    fb.AXIS_REF.pressure = recipe[0].targetPressure;
    fb.AXIS_REF.timestamp = 0.0;
    assert(HDY_MotionControlFB_StartSegment(&fb, 0, 0.0));
    HDY_MotionControlFB_Execute(&fb);
    assert(fb.ACTIVE);
    assert(fb.DIAGNOSTIC.code == HDY_DIAG_CODE_NONE);

    assert(!HDY_MotionControlFB_StartSegment(&fb, 0, 0.0));
    assert(fb.ACTIVE);
    assert(fb.STATUS == HDY_STATUS_RUNNING);
    assert(fb.FB_STATE == HDY_FB_STATE_RUNNING);
    assert(fb.DIAGNOSTIC.code == HDY_DIAG_CODE_COMMAND_NOT_ALLOWED);
    assert(fb.DIAGNOSTIC.source == HDY_DIAG_SOURCE_COMMAND);
    assert(fb.DIAGNOSTIC.recovery == HDY_DIAG_RECOVERY_CHECK_COMMAND);
    assert(fb.DIAGNOSTIC.protectionAction == HDY_PROTECTION_ACTION_WARNING);
    assert(strstr(fb.DIAGNOSTIC.message, "START") != NULL);
    assert(strstr(fb.DIAGNOSTIC.message, "RUNNING") != NULL);

    fb.AXIS_REF.timestamp = 0.1;
    HDY_MotionControlFB_Execute(&fb);
    assert(fb.ACTIVE);
    assert(fb.FB_STATE == HDY_FB_STATE_RUNNING);
    printf("✓ Start command legality while running test passed\n");
}

static void test_abort_rejected_in_ready_state(void) {
    HDY_MotionControlFB fb;
    HDY_MotionSegment recipe[1];

    printf("Testing Abort command legality in READY state...\n");
    init_controller(&fb);
    recipe[0] = make_position_segment("AbortReady", 10.0, HDY_DIRECTION_EXTEND);
    assert(HDY_MotionControlFB_LoadRecipe(&fb, recipe, 1));

    assert(!HDY_MotionControlFB_Abort(&fb));
    assert(!fb.ACTIVE);
    assert(!fb.FINISHED);
    assert(fb.STATUS == HDY_STATUS_READY);
    assert(fb.FB_STATE == HDY_FB_STATE_READY);
    assert(fb.DIAGNOSTIC.code == HDY_DIAG_CODE_COMMAND_NOT_ALLOWED);
    assert(fb.DIAGNOSTIC.source == HDY_DIAG_SOURCE_COMMAND);
    assert(fb.DIAGNOSTIC.recovery == HDY_DIAG_RECOVERY_CHECK_COMMAND);
    assert(fb.DIAGNOSTIC.protectionAction == HDY_PROTECTION_ACTION_WARNING);
    assert(strstr(fb.DIAGNOSTIC.message, "ABORT") != NULL);
    assert(strstr(fb.DIAGNOSTIC.message, "READY") != NULL);
    printf("✓ Abort command legality in READY state test passed\n");
}

static void test_acknowledge_rejected_while_running_even_without_live_diagnostic(void) {
    HDY_MotionControlFB fb;
    HDY_MotionSegment recipe[1];

    printf("Testing Ack legality while running...\n");
    init_controller(&fb);
    recipe[0] = make_time_segment("AckRunning", 1.0, HDY_DIRECTION_EXTEND);
    assert(HDY_MotionControlFB_LoadRecipe(&fb, recipe, 1));

    fb.AXIS_REF.position = 0.0;
    fb.AXIS_REF.velocity = 0.0;
    fb.AXIS_REF.flow = 0.0;
    fb.AXIS_REF.pressure = recipe[0].targetPressure;
    fb.AXIS_REF.timestamp = 0.0;
    assert(HDY_MotionControlFB_StartSegment(&fb, 0, 0.0));
    HDY_MotionControlFB_Execute(&fb);
    assert(fb.ACTIVE);
    assert(fb.DIAGNOSTIC.code == HDY_DIAG_CODE_NONE);

    assert(!HDY_MotionControlFB_AcknowledgeDiagnostics(&fb));
    assert(fb.ACTIVE);
    assert(fb.DIAGNOSTIC.code == HDY_DIAG_CODE_NONE);
    assert(fb.DIAGNOSTIC_LATCH.code == HDY_DIAG_CODE_NONE);
    printf("✓ Ack legality while running test passed\n");
}

static void test_next_rejected_in_aborted_state(void) {
    HDY_MotionControlFB fb;
    HDY_MotionSegment recipe[1];

    printf("Testing Next command legality in ABORTED state...\n");
    init_controller(&fb);
    recipe[0] = make_time_segment("AbortNext", 1.0, HDY_DIRECTION_EXTEND);
    assert(HDY_MotionControlFB_LoadRecipe(&fb, recipe, 1));

    fb.AXIS_REF.position = 0.0;
    fb.AXIS_REF.velocity = 0.0;
    fb.AXIS_REF.flow = 0.0;
    fb.AXIS_REF.pressure = recipe[0].targetPressure;
    fb.AXIS_REF.timestamp = 0.0;
    assert(HDY_MotionControlFB_StartSegment(&fb, 0, 0.0));
    HDY_MotionControlFB_Execute(&fb);
    assert(HDY_MotionControlFB_Abort(&fb));

    fb.AXIS_REF.timestamp = 0.1;
    HDY_MotionControlFB_Execute(&fb);
    assert(fb.FINISHED);
    assert(fb.FB_STATE == HDY_FB_STATE_ABORTED);

    assert(!HDY_MotionControlFB_NextSegment(&fb, 0.1));
    assert(fb.FINISHED);
    assert(fb.FB_STATE == HDY_FB_STATE_ABORTED);
    assert(fb.DIAGNOSTIC.code == HDY_DIAG_CODE_COMMAND_NOT_ALLOWED);
    assert(fb.DIAGNOSTIC.source == HDY_DIAG_SOURCE_COMMAND);
    assert(fb.DIAGNOSTIC.recovery == HDY_DIAG_RECOVERY_CHECK_COMMAND);
    assert(fb.DIAGNOSTIC.protectionAction == HDY_PROTECTION_ACTION_WARNING);
    assert(strstr(fb.DIAGNOSTIC.message, "NEXT") != NULL);
    assert(strstr(fb.DIAGNOSTIC.message, "ABORTED") != NULL);
    printf("✓ Next command legality in ABORTED state test passed\n");
}

static void test_abort_rejected_in_done_state(void) {
    HDY_MotionControlFB fb;
    HDY_MotionSegment recipe[1];

    printf("Testing Abort command legality in DONE state...\n");
    init_controller(&fb);
    recipe[0] = make_position_segment("AbortDone", 1.0, HDY_DIRECTION_EXTEND);
    assert(HDY_MotionControlFB_LoadRecipe(&fb, recipe, 1));

    fb.AXIS_REF.position = 0.0;
    fb.AXIS_REF.pressure = recipe[0].targetPressure;
    fb.AXIS_REF.timestamp = 0.0;
    assert(HDY_MotionControlFB_StartSegment(&fb, 0, 0.0));
    HDY_MotionControlFB_Execute(&fb);

    fb.AXIS_REF.position = 1.0;
    fb.AXIS_REF.timestamp = 0.1;
    HDY_MotionControlFB_Execute(&fb);
    assert(fb.FINISHED);
    assert(fb.FB_STATE == HDY_FB_STATE_DONE);

    assert(!HDY_MotionControlFB_Abort(&fb));
    assert(fb.FINISHED);
    assert(!fb.FAULT);
    assert(fb.FB_STATE == HDY_FB_STATE_DONE);
    assert(fb.DIAGNOSTIC.code == HDY_DIAG_CODE_COMMAND_NOT_ALLOWED);
    assert(fb.DIAGNOSTIC.source == HDY_DIAG_SOURCE_COMMAND);
    assert(fb.DIAGNOSTIC.recovery == HDY_DIAG_RECOVERY_CHECK_COMMAND);
    assert(fb.DIAGNOSTIC.protectionAction == HDY_PROTECTION_ACTION_WARNING);
    assert(strstr(fb.DIAGNOSTIC.message, "ABORT") != NULL);
    assert(strstr(fb.DIAGNOSTIC.message, "DONE") != NULL);
    printf("✓ Abort command legality in DONE state test passed\n");
}

static void test_acknowledge_allowed_in_disabled_state(void) {
    HDY_MotionControlFB fb;
    HDY_MotionSegment recipe[1];

    printf("Testing Ack command legality in DISABLED state...\n");
    init_controller(&fb);
    recipe[0] = make_position_segment("AckDisabled", 20.0, HDY_DIRECTION_EXTEND);
    assert(HDY_MotionControlFB_LoadRecipe(&fb, recipe, 1));

    fb.AXIS_REF.position = 0.0;
    fb.AXIS_REF.velocity = 0.0;
    fb.AXIS_REF.flow = 0.0;
    fb.AXIS_REF.pressure = recipe[0].targetPressure;
    fb.AXIS_REF.timestamp = 0.0;
    assert(HDY_MotionControlFB_StartSegment(&fb, 0, 0.0));
    HDY_MotionControlFB_Execute(&fb);
    assert(fb.DIAGNOSTIC.code == HDY_DIAG_CODE_FLOW_DEVIATION);
    assert(fb.DIAGNOSTIC_LATCH.code == HDY_DIAG_CODE_FLOW_DEVIATION);
    assert(fb.DIAGNOSTIC_HISTORY.count == 1U);

    fb.EN = false;
    fb.AXIS_REF.timestamp = 0.1;
    HDY_MotionControlFB_Execute(&fb);
    assert(!fb.ENO);
    assert(fb.FB_STATE == HDY_FB_STATE_DISABLED);
    assert(fb.DIAGNOSTIC.code == HDY_DIAG_CODE_NONE);
    assert(fb.DIAGNOSTIC_LATCH.code == HDY_DIAG_CODE_FLOW_DEVIATION);
    assert(fb.DIAGNOSTIC_HISTORY.count == 1U);

    assert(HDY_MotionControlFB_AcknowledgeDiagnostics(&fb));
    assert(fb.DIAGNOSTIC.code == HDY_DIAG_CODE_NONE);
    assert(fb.DIAGNOSTIC_LATCH.code == HDY_DIAG_CODE_NONE);
    assert(!fb.LAST_DIAGNOSTIC_SNAPSHOT.valid);
    assert(!fb.LAST_FAULT_SNAPSHOT.valid);
    assert(fb.DIAGNOSTIC_HISTORY.count == 0U);
    assert(fb.DIAGNOSTIC_HISTORY.totalRecorded == 0U);
    printf("✓ Ack command legality in DISABLED state test passed\n");
}

static void test_start_allowed_in_aborted_state(void) {
    HDY_MotionControlFB fb;
    HDY_MotionSegment recipe[1];

    printf("Testing Start command legality in ABORTED state...\n");
    init_controller(&fb);
    recipe[0] = make_time_segment("RestartAfterAbort", 1.0, HDY_DIRECTION_EXTEND);
    assert(HDY_MotionControlFB_LoadRecipe(&fb, recipe, 1));

    fb.AXIS_REF.position = 0.0;
    fb.AXIS_REF.velocity = 0.0;
    fb.AXIS_REF.flow = 0.0;
    fb.AXIS_REF.pressure = recipe[0].targetPressure;
    fb.AXIS_REF.timestamp = 0.0;
    assert(HDY_MotionControlFB_StartSegment(&fb, 0, 0.0));
    HDY_MotionControlFB_Cycle(&fb);
    assert(HDY_MotionControlFB_Abort(&fb));

    fb.AXIS_REF.timestamp = 0.1;
    HDY_MotionControlFB_Cycle(&fb);
    assert(fb.FINISHED);
    assert(fb.FB_STATE == HDY_FB_STATE_ABORTED);

    fb.AXIS_REF.timestamp = 0.2;
    fb.AXIS_REF.pressure = recipe[0].targetPressure;
    assert(HDY_MotionControlFB_StartSegment(&fb, 0, 0.2));
    assert(fb._pendingCommand == HDY_CMD_START);
    assert(!fb.ACTIVE);

    HDY_MotionControlFB_Cycle(&fb);
    assert(fb._pendingCommand == HDY_CMD_NONE);
    assert(fb.ACTIVE);
    assert(!fb.FINISHED);
    assert(!fb.FAULT);
    assert(fb.FB_STATE == HDY_FB_STATE_RUNNING);
    assert(strcmp(fb.STATE.currentSegmentName, "RestartAfterAbort") == 0);
    printf("✓ Start command legality in ABORTED state test passed\n");
}

static void test_multiple_instances_are_isolated(void) {
    HDY_MotionControlFB fbA;
    HDY_MotionControlFB fbB;
    HDY_MotionSegment recipeA[1];
    HDY_MotionSegment recipeB[1];
    HDY_DiagnosticCode aDiagnosticCode;
    HDY_UINT aHistoryCount;
    HDY_REAL aPumpSpeed;

    printf("Testing multi-instance isolation...\n");
    init_controller(&fbA);
    init_controller(&fbB);
    recipeA[0] = make_position_segment("IsoA", 20.0, HDY_DIRECTION_EXTEND);
    recipeB[0] = make_time_segment("IsoB", 1.0, HDY_DIRECTION_RETRACT);
    assert(HDY_MotionControlFB_LoadRecipe(&fbA, recipeA, 1));
    assert(HDY_MotionControlFB_LoadRecipe(&fbB, recipeB, 1));

    fbA.AXIS_REF.position = 0.0;
    fbA.AXIS_REF.flow = 0.0;
    fbA.AXIS_REF.pressure = recipeA[0].targetPressure;
    fbA.AXIS_REF.timestamp = 0.0;
    assert(HDY_MotionControlFB_StartSegment(&fbA, 0, 0.0));
    HDY_MotionControlFB_Cycle(&fbA);
    assert(fbA.ACTIVE);
    assert(strcmp(fbA.STATE.currentSegmentName, "IsoA") == 0);
    aDiagnosticCode = fbA.DIAGNOSTIC.code;
    aHistoryCount = fbA.DIAGNOSTIC_HISTORY.count;
    aPumpSpeed = fbA.PUMP_SPEED;

    assert(!fbB.ACTIVE);
    assert(fbB.STATUS == HDY_STATUS_READY);
    assert(fbB.FB_STATE == HDY_FB_STATE_READY);
    assert(fbB.DIAGNOSTIC.code == HDY_DIAG_CODE_NONE);
    assert(fbB.DIAGNOSTIC_HISTORY.count == 0U);

    fbB.AXIS_REF.position = 10.0;
    fbB.AXIS_REF.flow = 0.0;
    fbB.AXIS_REF.pressure = recipeB[0].targetPressure;
    fbB.AXIS_REF.timestamp = 1.0;
    fbB.START_SEGMENT = true;
    fbB.START_SEGMENT_INDEX = 0U;
    HDY_MotionControlFB_Scan(&fbB);
    assert(fbB.ACTIVE);
    assert(strcmp(fbB.STATE.currentSegmentName, "IsoB") == 0);
    assert(fbB.STATE.plannedDirection == HDY_DIRECTION_RETRACT);

    assert(fbA.ACTIVE);
    assert(strcmp(fbA.STATE.currentSegmentName, "IsoA") == 0);
    assert(fbA.DIAGNOSTIC.code == aDiagnosticCode);
    assert(fbA.DIAGNOSTIC_HISTORY.count == aHistoryCount);
    assert(fbA.PUMP_SPEED == aPumpSpeed);

    assert(HDY_MotionControlFB_Abort(&fbB));
    fbB.AXIS_REF.timestamp = 1.1;
    HDY_MotionControlFB_Cycle(&fbB);
    assert(!fbB.ACTIVE);
    assert(fbB.FINISHED);
    assert(fbB.FB_STATE == HDY_FB_STATE_ABORTED);

    fbA.AXIS_REF.timestamp = 0.1;
    HDY_MotionControlFB_Execute(&fbA);
    assert(fbA.ACTIVE);
    assert(fbA.FB_STATE == HDY_FB_STATE_RUNNING);
    assert(strcmp(fbA.STATE.currentSegmentName, "IsoA") == 0);
    assert(fbB.FB_STATE == HDY_FB_STATE_ABORTED);
    assert(strcmp(fbB.STATE.currentSegmentName, "") == 0);
    printf("✓ Multi-instance isolation test passed\n");
}

static void test_segment_completion_and_next_segment(void) {
    HDY_MotionControlFB fb;
    HDY_MotionSegment recipe[2];

    printf("Testing segment completion and NextSegment behavior...\n");
    init_controller(&fb);
    recipe[0] = make_position_segment("SegmentA", 1.0, HDY_DIRECTION_EXTEND);
    recipe[1] = make_time_segment("SegmentB", 0.5, HDY_DIRECTION_EXTEND);
    assert(HDY_MotionControlFB_LoadRecipe(&fb, recipe, 2));

    fb.AXIS_REF.position = 0.0;
    fb.AXIS_REF.timestamp = 0.0;
    assert(HDY_MotionControlFB_StartSegment(&fb, 0, fb.AXIS_REF.timestamp));
    HDY_MotionControlFB_Execute(&fb);

    assert(fb.ACTIVE);
    assert(!fb.SEGMENT_COMPLETED);
    assert(!HDY_MotionControlFB_NextSegment(&fb, fb.AXIS_REF.timestamp));
    assert(fb.ACTIVE);
    assert(!fb.SEGMENT_COMPLETED);
    assert(fb.STATE.currentSegmentIndex == 0U);
    assert(fb.DIAGNOSTIC.code == HDY_DIAG_CODE_SEGMENT_NOT_COMPLETED);
    assert(strstr(fb.DIAGNOSTIC.message, "not completed") != NULL);


    fb.AXIS_REF.position = 1.0;
    HDY_MotionControlFB_Execute(&fb);

    assert(fb.SEGMENT_COMPLETED);
    assert(!fb.ACTIVE);
    assert(!fb.FINISHED);
    assert(!fb.FAULT);
    assert(fb.STATUS == HDY_STATUS_SEGMENT_COMPLETE);
    assert(fb.PUMP_SPEED == 0.0);
    assert(strcmp(fb.STATE.currentSegmentName, "SegmentA") == 0);

    assert(HDY_MotionControlFB_NextSegment(&fb, fb.AXIS_REF.timestamp));
    assert(!fb.ACTIVE);
    assert(fb.SEGMENT_COMPLETED);
    assert(!fb.FINISHED);
    assert(fb.STATUS == HDY_STATUS_SEGMENT_COMPLETE);
    assert(fb.FB_STATE == HDY_FB_STATE_SEGMENT_COMPLETE);
    assert(fb.STATE.currentSegmentIndex == 0U);
    assert(strcmp(fb.STATE.currentSegmentName, "SegmentA") == 0);

    fb.AXIS_REF.timestamp = 0.1;
    HDY_MotionControlFB_Execute(&fb);
    assert(fb.SEGMENT_CHANGED);
    assert(fb.ACTIVE);
    assert(!fb.SEGMENT_COMPLETED);
    assert(fb.STATUS == HDY_STATUS_RUNNING);
    assert(fb.FB_STATE == HDY_FB_STATE_RUNNING);
    assert(fb.STATE.currentSegmentIndex == 1U);
    assert(strcmp(fb.STATE.currentSegmentName, "SegmentB") == 0);

    fb.AXIS_REF.timestamp = 0.6;
    HDY_MotionControlFB_Execute(&fb);
    assert(!fb.ACTIVE);
    assert(fb.SEGMENT_COMPLETED);
    assert(fb.FINISHED);
    assert(!fb.FAULT);
    assert(fb.STATUS == HDY_STATUS_FINISHED);
    assert(fb.PUMP_SPEED == 0.0);
    assert(!HDY_MotionControlFB_NextSegment(&fb, fb.AXIS_REF.timestamp));
    assert(fb.FINISHED);
    assert(fb.SEGMENT_COMPLETED);
    assert(fb.DIAGNOSTIC.code == HDY_DIAG_CODE_RECIPE_ALREADY_FINISHED);
    assert(strstr(fb.DIAGNOSTIC.message, "already finished") != NULL);
    printf("✓ Segment completion and NextSegment behavior test passed\n");
}

static void test_retract_position_directional_planning(void) {
    HDY_MotionControlFB fb;
    HDY_MotionSegment recipe[1];

    printf("Testing retract position directional planning...\n");
    init_controller(&fb);
    recipe[0] = make_position_segment("RetractPosition", 2.0, HDY_DIRECTION_RETRACT);
    assert(HDY_MotionControlFB_LoadRecipe(&fb, recipe, 1));

    fb.AXIS_REF.position = 10.0;
    fb.AXIS_REF.timestamp = 0.0;
    assert(HDY_MotionControlFB_StartSegment(&fb, 0, 0.0));

    HDY_MotionControlFB_Execute(&fb);
    assert(fb.ACTIVE);
    assert(fb.STATE.plannedDirection == HDY_DIRECTION_RETRACT);
    assert(fb.STATE.plannedVelocity < 0.0);
    assert(fb.STATE.plannedFlow > 0.0);
    assert(fb.PUMP_SPEED > 0.0);

    fb.AXIS_REF.position = 2.05;
    fb.AXIS_REF.timestamp = 0.1;
    HDY_MotionControlFB_Execute(&fb);
    assert(fb.SEGMENT_COMPLETED);
    assert(!fb.ACTIVE);
    assert(fb.STATUS == HDY_STATUS_FINISHED);
    printf("✓ Retract position directional planning test passed\n");
}

static void test_speed_ramp_retract_directional_planning(void) {
    HDY_MotionControlFB fb;
    HDY_MotionSegment recipe[1];

    printf("Testing speed-ramp retract directional planning...\n");
    init_controller(&fb);
    recipe[0] = make_time_segment("RetractRamp", 0.5, HDY_DIRECTION_RETRACT);
    assert(HDY_MotionControlFB_LoadRecipe(&fb, recipe, 1));

    fb.AXIS_REF.position = 10.0;
    fb.AXIS_REF.timestamp = 0.0;
    assert(HDY_MotionControlFB_StartSegment(&fb, 0, 0.0));

    fb.AXIS_REF.timestamp = 0.2;
    HDY_MotionControlFB_Execute(&fb);
    assert(fb.ACTIVE);
    assert(fb.STATE.plannedDirection == HDY_DIRECTION_RETRACT);
    assert(fb.STATE.plannedVelocity < 0.0);
    assert(fb.STATE.plannedFlow > 0.0);
    assert(fb.PUMP_SPEED > 0.0);

    fb.AXIS_REF.timestamp = 0.6;
    HDY_MotionControlFB_Execute(&fb);
    assert(fb.SEGMENT_COMPLETED);
    assert(fb.FINISHED);
    assert(fb.STATUS == HDY_STATUS_FINISHED);
    printf("✓ Speed-ramp retract directional planning test passed\n");
}

static void test_pressure_closed_loop_pi_strategy_accumulates_flow(void) {
    HDY_MotionControlFB fb;
    HDY_MotionSegment recipe[1];
    HDY_REAL firstPumpSpeed;

    printf("Testing pressure closed-loop PI strategy integration...\n");
    init_controller(&fb);
    recipe[0] = make_pressure_segment("PressurePI", 2.0, 10.0, 2.0);
    recipe[0].pressureController = HDY_PRESSURE_CONTROLLER_PI;
    recipe[0].pressureKp = 0.2;
    recipe[0].pressureKi = 1.0;
    recipe[0].pressureIntegralLimit = 4.0;
    recipe[0].pressureDeadband = 0.0;
    recipe[0].pressureFilterAlpha = 1.0;
    assert(HDY_MotionControlFB_LoadRecipe(&fb, recipe, 1));

    fb.AXIS_REF.pressure = 6.0;
    fb.AXIS_REF.timestamp = 0.0;
    assert(HDY_MotionControlFB_StartSegment(&fb, 0, 0.0));

    HDY_MotionControlFB_Execute(&fb);
    firstPumpSpeed = fb.PUMP_SPEED;
    assert(fb.ACTIVE);
    assert(fb.STATE.plannedDirection == HDY_DIRECTION_HOLD);
    assert(fb.STATE.plannedVelocity == 0.0);
    assert(fb.STATE.plannedFlow > recipe[0].targetFlow);
    assert(firstPumpSpeed > recipe[0].targetFlow * fb.FLOW_TO_PUMP_SPEED_GAIN);

    fb.AXIS_REF.timestamp = 1.0;
    fb.AXIS_REF.pressure = 6.0;
    HDY_MotionControlFB_Execute(&fb);
    assert(fb.ACTIVE);
    assert(fb.PUMP_SPEED > firstPumpSpeed);
    assert(fb.STATE.plannedFlow > recipe[0].targetFlow + 2.0);
    printf("✓ Pressure closed-loop PI strategy integration test passed\n");
}

static void test_pressure_mode_transition_tracks_existing_flow(void) {
    HDY_MotionControlFB fb;
    HDY_MotionSegment recipe[2];
    HDY_REAL trackedFlow;

    printf("Testing pressure mode transition bumpless flow tracking...\n");
    init_controller(&fb);
    recipe[0] = make_time_segment("VelocityStage", 0.2, HDY_DIRECTION_EXTEND);
    recipe[0].targetFlow = 10.0;
    recipe[0].maxFlow = 10.0;
    recipe[0].maxAcceleration = 50.0;
    recipe[0].maxVelocity = 20.0;
    recipe[1] = make_pressure_segment("PressureTrack", 1.0, 10.0, 2.0);
    recipe[1].pressureController = HDY_PRESSURE_CONTROLLER_PI;
    recipe[1].pressureKp = 0.2;
    recipe[1].pressureKi = 1.0;
    recipe[1].pressureIntegralLimit = 10.0;
    assert(HDY_MotionControlFB_LoadRecipe(&fb, recipe, 2));

    fb.AXIS_REF.pressure = 10.0;
    fb.AXIS_REF.timestamp = 0.0;
    assert(HDY_MotionControlFB_StartSegment(&fb, 0, 0.0));

    fb.AXIS_REF.timestamp = 0.1;
    HDY_MotionControlFB_Execute(&fb);
    trackedFlow = fb.STATE.plannedFlow;
    assert(trackedFlow > 4.0);

    fb.AXIS_REF.flow = trackedFlow;
    fb.AXIS_REF.timestamp = 0.25;
    HDY_MotionControlFB_Execute(&fb);
    assert(fb.SEGMENT_COMPLETED);
    assert(HDY_MotionControlFB_NextSegment(&fb, 0.25));

    fb.AXIS_REF.pressure = 10.0;
    fb.AXIS_REF.flow = trackedFlow;
    HDY_MotionControlFB_Execute(&fb);

    assert(fb.ACTIVE);
    assert(fb.STATE.currentSegmentIndex == 1U);
    assert(fabs(fb.STATE.plannedFlow - trackedFlow) < 0.001);
    assert(fb.STATE.plannedFlow > recipe[1].targetFlow + 1.0);
    printf("✓ Pressure mode transition bumpless flow tracking test passed\n");
}

static void test_runtime_reference_state_exposes_shared_execution_context(void) {
    HDY_MotionControlFB fb;
    HDY_MotionSegment recipe[1];

    printf("Testing runtime reference state exposure...\n");
    init_controller(&fb);
    recipe[0] = make_pressure_segment("PressureReferenceState", 2.0, 15.0, 2.0);
    recipe[0].pressureController = HDY_PRESSURE_CONTROLLER_PI;
    recipe[0].pressureKp = 0.5;
    recipe[0].pressureKi = 1.0;
    recipe[0].pressureIntegralLimit = 8.0;
    recipe[0].pressureRampRate = 4.0;
    assert(HDY_MotionControlFB_LoadRecipe(&fb, recipe, 1));

    fb.AXIS_REF.pressure = 5.0;
    fb.AXIS_REF.flow = 0.0;
    fb.AXIS_REF.timestamp = 0.0;
    assert(HDY_MotionControlFB_StartSegment(&fb, 0, 0.0));

    fb.AXIS_REF.timestamp = 0.5;
    HDY_MotionControlFB_Execute(&fb);
    assert(fb.ACTIVE);
    assert(fabs(fb.STATE.references.elapsedTime - 0.5) < 0.001);
    assert(fabs(fb.STATE.references.pressureReference - 7.0) < 0.001);
    assert(fabs(fb.STATE.references.flowReference - fb.STATE.plannedFlow) < 0.001);
    assert(fabs(fb.STATE.references.velocityReference - fb.STATE.plannedVelocity) < 0.001);
    assert(fb.STATE.pressureControllerApplied == HDY_PRESSURE_CONTROLLER_PI);

    fb.EN = false;
    fb.AXIS_REF.timestamp = 0.6;
    HDY_MotionControlFB_Execute(&fb);
    assert(fb.STATE.references.elapsedTime == 0.0);
    assert(fb.STATE.references.pressureReference == 0.0);
    assert(fb.STATE.references.flowReference == 0.0);
    assert(fb.STATE.references.velocityReference == 0.0);
    assert(fb.STATE.pressureControllerApplied == HDY_PRESSURE_CONTROLLER_NONE);
    printf("✓ Runtime reference state exposure test passed\n");
}

static void test_pressure_loop_state_exposes_adaptive_rbf_telemetry(void) {
    HDY_MotionControlFB fb;
    HDY_MotionSegment recipe[1];

    printf("Testing pressure-loop adaptive telemetry exposure...\n");
    init_controller(&fb);
    recipe[0] = make_pressure_segment("PressureRbfState", 2.0, 18.0, 0.3);
    recipe[0].pressureController = HDY_PRESSURE_CONTROLLER_RBF_PID;
    recipe[0].maxFlow = 1.2;
    recipe[0].pressureRbfConfig.minKp = 0.81;
    recipe[0].pressureRbfConfig.maxKp = 0.82;
    recipe[0].pressureRbfConfig.minKi = 0.019;
    recipe[0].pressureRbfConfig.maxKi = 0.021;
    recipe[0].pressureRbfConfig.minKd = 1.24;
    recipe[0].pressureRbfConfig.maxKd = 1.26;
    recipe[0].pressureRbfConfig.etaP = 0.11;
    recipe[0].pressureRbfConfig.etaI = 0.12;
    recipe[0].pressureRbfConfig.etaD = 0.13;
    assert(HDY_MotionControlFB_LoadRecipe(&fb, recipe, 1));

    fb.AXIS_REF.pressure = 5.0;
    fb.AXIS_REF.flow = 0.2;
    fb.AXIS_REF.timestamp = 0.0;
    assert(HDY_MotionControlFB_StartSegment(&fb, 0, 0.0));

    fb.AXIS_REF.timestamp = 0.02;
    HDY_MotionControlFB_Execute(&fb);
    assert(fb.ACTIVE);
    assert(fb.STATE.pressureControllerApplied == HDY_PRESSURE_CONTROLLER_RBF_PID);
    assert(fb.STATE.pressureLoop.adaptiveActive);
    assert(fabs(fb.STATE.pressureLoop.feedforwardFlow - recipe[0].targetFlow) < 0.001);
    assert(fabs(fb.STATE.pressureLoop.outputFlow - fb.STATE.plannedFlow) < 0.001);
    assert(fabs(fb.STATE.pressureLoop.samplingPeriod - 0.02) < 0.001);
    assert(fb.STATE.pressureLoop.adaptiveKp >= 0.81 - 1e-6);
    assert(fb.STATE.pressureLoop.adaptiveKp <= 0.82 + 1e-6);
    assert(fb.STATE.pressureLoop.adaptiveKi >= 0.019 - 1e-6);
    assert(fb.STATE.pressureLoop.adaptiveKi <= 0.021 + 1e-6);
    assert(fb.STATE.pressureLoop.adaptiveKd >= 1.24 - 1e-6);
    assert(fb.STATE.pressureLoop.adaptiveKd <= 1.26 + 1e-6);

    fb.EN = false;
    fb.AXIS_REF.timestamp = 0.03;
    HDY_MotionControlFB_Execute(&fb);
    assert(fb.STATE.pressureControllerApplied == HDY_PRESSURE_CONTROLLER_NONE);
    assert(!fb.STATE.pressureLoop.adaptiveActive);
    assert(fb.STATE.pressureLoop.outputFlow == 0.0);
    assert(fb.STATE.pressureLoop.filteredPressure == 0.0);
    printf("✓ Pressure-loop adaptive telemetry exposure test passed\n");
}

static void test_execution_diagnostics_promote_degraded_status(void) {
    HDY_MotionControlFB fb;
    HDY_MotionSegment recipe[1];

    printf("Testing execution diagnostics degraded-state behavior...\n");
    init_controller(&fb);
    recipe[0] = make_position_segment("DegradedFlow", 20.0, HDY_DIRECTION_EXTEND);
    assert(HDY_MotionControlFB_LoadRecipe(&fb, recipe, 1));

    fb.AXIS_REF.position = 0.0;
    fb.AXIS_REF.flow = 0.0;
    fb.AXIS_REF.pressure = recipe[0].targetPressure;
    fb.AXIS_REF.timestamp = 0.0;
    assert(HDY_MotionControlFB_StartSegment(&fb, 0, 0.0));
    HDY_MotionControlFB_Execute(&fb);

    assert(fb.ACTIVE);
    assert(fb.STATUS == HDY_STATUS_DEGRADED);
    assert(fb.STATE.status == HDY_STATUS_DEGRADED);
    assert(fb.STATE.protectionAction == HDY_PROTECTION_ACTION_DERATE);
    assert(fb.DIAGNOSTIC.flowDeviation);
    assert(fb.DIAGNOSTIC.code == HDY_DIAG_CODE_FLOW_DEVIATION);
    assert(fb.DIAGNOSTIC.source == HDY_DIAG_SOURCE_EXECUTION);
    assert(fb.DIAGNOSTIC.recovery == HDY_DIAG_RECOVERY_CHECK_COMMAND);
    assert(fb.DIAGNOSTIC.protectionAction == HDY_PROTECTION_ACTION_DERATE);
    printf("✓ Execution diagnostics degraded-state test passed\n");
}

static void test_diagnostic_flags_expose_minimal_embedded_summary(void) {
    HDY_MotionControlFB fb;
    HDY_MotionSegment recipe[1];

    printf("Testing compact diagnostic flags and string helpers...\n");
    init_controller(&fb);
    recipe[0] = make_position_segment("DiagSummary", 20.0, HDY_DIRECTION_EXTEND);
    assert(HDY_MotionControlFB_LoadRecipe(&fb, recipe, 1));

    fb.AXIS_REF.position = 0.0;
    fb.AXIS_REF.flow = 0.0;
    fb.AXIS_REF.pressure = recipe[0].targetPressure;
    fb.AXIS_REF.timestamp = 0.0;
    assert(HDY_MotionControlFB_StartSegment(&fb, 0, 0.0));
    HDY_MotionControlFB_Execute(&fb);

    assert(fb.DIAGNOSTIC.flags == HDY_Diagnostics_GetFlagMask(&fb.DIAGNOSTIC));
    assert(HDY_Diagnostics_HasFlag(&fb.DIAGNOSTIC, HDY_DIAG_FLAG_FLOW_DEVIATION));
    assert(HDY_Diagnostics_HasFlag(&fb.DIAGNOSTIC, HDY_DIAG_FLAG_POSITION_DEVIATION));
    assert(HDY_Diagnostics_HasFlag(&fb.DIAGNOSTIC, HDY_DIAG_FLAG_VELOCITY_DEVIATION));
    assert(!HDY_Diagnostics_HasFlag(&fb.DIAGNOSTIC, HDY_DIAG_FLAG_TIMEOUT));
    assert(strcmp(HDY_Diagnostics_CodeToString(fb.DIAGNOSTIC.code), "FLOW_DEVIATION") == 0);
    assert(strcmp(HDY_Diagnostics_SeverityToString(fb.DIAGNOSTIC.severity), "WARNING") == 0);
    assert(strcmp(HDY_Diagnostics_SourceToString(fb.DIAGNOSTIC.source), "EXECUTION") == 0);
    assert(strcmp(HDY_Diagnostics_RecoveryToString(fb.DIAGNOSTIC.recovery), "CHECK_COMMAND") == 0);
    assert(strcmp(HDY_Diagnostics_ProtectionActionToString(fb.DIAGNOSTIC.protectionAction), "DERATE") == 0);
    printf("✓ Compact diagnostic flags/string helper test passed\n");
}

static void test_diagnostic_latch_and_history_persist_after_live_clear(void) {
    HDY_MotionControlFB fb;
    HDY_MotionSegment recipe[1];

    printf("Testing diagnostic latch/history retention semantics...\n");
    init_controller(&fb);
    recipe[0] = make_position_segment("DiagLatch", 20.0, HDY_DIRECTION_EXTEND);
    assert(HDY_MotionControlFB_LoadRecipe(&fb, recipe, 1));

    fb.AXIS_REF.position = 0.0;
    fb.AXIS_REF.velocity = 0.0;
    fb.AXIS_REF.flow = 0.0;
    fb.AXIS_REF.pressure = recipe[0].targetPressure;
    fb.AXIS_REF.timestamp = 0.0;
    assert(HDY_MotionControlFB_StartSegment(&fb, 0, 0.0));
    HDY_MotionControlFB_Execute(&fb);

    assert(fb.DIAGNOSTIC.code == HDY_DIAG_CODE_FLOW_DEVIATION);
    assert(fb.DIAGNOSTIC_LATCH.code == HDY_DIAG_CODE_FLOW_DEVIATION);
    assert(fb.LAST_DIAGNOSTIC_SNAPSHOT.valid);
    assert(fb.LAST_DIAGNOSTIC_SNAPSHOT.diagnostic.code == HDY_DIAG_CODE_FLOW_DEVIATION);
    assert(fb.LAST_DIAGNOSTIC_SNAPSHOT.segmentIndex == 0U);
    assert(fb.LAST_DIAGNOSTIC_SNAPSHOT.status == HDY_STATUS_DEGRADED);
    assert(strcmp(fb.LAST_DIAGNOSTIC_SNAPSHOT.segmentName, "DiagLatch") == 0);
    assert(fabs(fb.LAST_DIAGNOSTIC_SNAPSHOT.references.flowReference - fb.STATE.references.flowReference) < 0.001);
    assert(fb.DIAGNOSTIC_HISTORY.count == 1U);
    assert(fb.DIAGNOSTIC_HISTORY.totalRecorded == 1U);
    assert(!fb.LAST_FAULT_SNAPSHOT.valid);

    fb.AXIS_REF.position = 20.0;
    fb.AXIS_REF.velocity = 0.0;
    fb.AXIS_REF.flow = 0.0;
    fb.AXIS_REF.pressure = recipe[0].targetPressure;
    fb.AXIS_REF.timestamp = 0.1;
    HDY_MotionControlFB_Execute(&fb);

    assert(fb.DIAGNOSTIC.code == HDY_DIAG_CODE_NONE);
    assert(fb.DIAGNOSTIC_LATCH.code == HDY_DIAG_CODE_FLOW_DEVIATION);
    assert(fb.LAST_DIAGNOSTIC_SNAPSHOT.valid);
    assert(fb.DIAGNOSTIC_HISTORY.count == 1U);
    assert(fb.DIAGNOSTIC_HISTORY.totalRecorded == 1U);
    printf("✓ Diagnostic latch/history retention test passed\n");
}

static void test_diagnostic_history_helpers_preserve_chronological_order(void) {
    HDY_DiagnosticHistory history;
    HDY_DiagnosticInfo diagnostic;
    HDY_DiagnosticSnapshot snapshot;
    HDY_DiagnosticSnapshot entry;
    HDY_AxisRef axisRef = {0};
    HDY_ExecutionReference references = {0};
    const HDY_DiagnosticCode codes[5] = {
        HDY_DIAG_CODE_OVER_PRESSURE,
        HDY_DIAG_CODE_FLOW_DEVIATION,
        HDY_DIAG_CODE_POSITION_DEVIATION,
        HDY_DIAG_CODE_SENSOR_FAULT,
        HDY_DIAG_CODE_TIMEOUT
    };
    const char* names[5] = {"H0", "H1", "H2", "H3", "H4"};
    size_t index;

    printf("Testing diagnostic history helper ordering...\n");
    HDY_DiagnosticsHistory_Clear(&history);

    for (index = 0U; index < 5U; ++index) {
        HDY_Diagnostics_SetEvent(&diagnostic, codes[index], HDY_DIAG_SEVERITY_NONE, NULL);
        HDY_Diagnostics_CaptureSnapshot(&snapshot,
                                        &diagnostic,
                                        &axisRef,
                                        &references,
                                        (HDY_TIME)index,
                                        (HDY_UINT8)index,
                                        names[index],
                                        HDY_STATUS_RUNNING,
                                        true,
                                        false,
                                        false);
        HDY_DiagnosticsHistory_Push(&history, &snapshot);
    }

    assert(history.count == HDY_DIAG_HISTORY_DEPTH);
    assert(history.totalRecorded == 5U);
    assert(history.wrapped);
    assert(!HDY_DiagnosticsHistory_GetEntry(&history, 4U, &entry));

    assert(HDY_DiagnosticsHistory_GetEntry(&history, 0U, &entry));
    assert(entry.diagnostic.code == HDY_DIAG_CODE_FLOW_DEVIATION);
    assert(entry.segmentIndex == 1U);
    assert(strcmp(entry.segmentName, "H1") == 0);

    assert(HDY_DiagnosticsHistory_GetEntry(&history, 1U, &entry));
    assert(entry.diagnostic.code == HDY_DIAG_CODE_POSITION_DEVIATION);
    assert(entry.segmentIndex == 2U);

    assert(HDY_DiagnosticsHistory_GetEntry(&history, 2U, &entry));
    assert(entry.diagnostic.code == HDY_DIAG_CODE_SENSOR_FAULT);
    assert(entry.segmentIndex == 3U);

    assert(HDY_DiagnosticsHistory_GetLatest(&history, &entry));
    assert(entry.diagnostic.code == HDY_DIAG_CODE_TIMEOUT);
    assert(entry.segmentIndex == 4U);
    assert(fabs(entry.eventTimestamp - 4.0) < 0.001);
    assert(strcmp(entry.segmentName, "H4") == 0);
    printf("✓ Diagnostic history helper ordering test passed\n");
}

static void test_acknowledge_clears_retention_and_allows_re_recording(void) {
    HDY_MotionControlFB fb;
    HDY_MotionSegment recipe[1];

    printf("Testing AcknowledgeDiagnostics retention-clear semantics...\n");
    init_controller(&fb);
    recipe[0] = make_position_segment("AckCase", 20.0, HDY_DIRECTION_EXTEND);
    assert(HDY_MotionControlFB_LoadRecipe(&fb, recipe, 1));

    fb.AXIS_REF.position = 0.0;
    fb.AXIS_REF.velocity = 0.0;
    fb.AXIS_REF.flow = 0.0;
    fb.AXIS_REF.pressure = recipe[0].targetPressure;
    fb.AXIS_REF.timestamp = 0.0;
    assert(HDY_MotionControlFB_StartSegment(&fb, 0, 0.0));
    HDY_MotionControlFB_Execute(&fb);

    assert(fb.DIAGNOSTIC.code == HDY_DIAG_CODE_FLOW_DEVIATION);
    assert(!HDY_MotionControlFB_AcknowledgeDiagnostics(&fb));
    assert(fb.DIAGNOSTIC_LATCH.code == HDY_DIAG_CODE_FLOW_DEVIATION);
    assert(fb.DIAGNOSTIC_HISTORY.count == 1U);

    fb.AXIS_REF.position = 20.0;
    fb.AXIS_REF.velocity = 0.0;
    fb.AXIS_REF.flow = 0.0;
    fb.AXIS_REF.pressure = recipe[0].targetPressure;
    fb.AXIS_REF.timestamp = 0.1;
    HDY_MotionControlFB_Execute(&fb);

    assert(fb.DIAGNOSTIC.code == HDY_DIAG_CODE_NONE);
    assert(HDY_MotionControlFB_AcknowledgeDiagnostics(&fb));
    assert(fb.DIAGNOSTIC.code == HDY_DIAG_CODE_NONE);
    assert(fb.DIAGNOSTIC_LATCH.code == HDY_DIAG_CODE_NONE);
    assert(!fb.LAST_DIAGNOSTIC_SNAPSHOT.valid);
    assert(!fb.LAST_FAULT_SNAPSHOT.valid);
    assert(fb.DIAGNOSTIC_HISTORY.count == 0U);
    assert(fb.DIAGNOSTIC_HISTORY.totalRecorded == 0U);

    fb.AXIS_REF.position = 0.0;
    fb.AXIS_REF.velocity = 0.0;
    fb.AXIS_REF.flow = 0.0;
    fb.AXIS_REF.pressure = recipe[0].targetPressure;
    fb.AXIS_REF.timestamp = 0.2;
    assert(HDY_MotionControlFB_StartSegment(&fb, 0, 0.2));
    HDY_MotionControlFB_Execute(&fb);

    assert(fb.DIAGNOSTIC.code == HDY_DIAG_CODE_FLOW_DEVIATION);
    assert(fb.DIAGNOSTIC_LATCH.code == HDY_DIAG_CODE_FLOW_DEVIATION);
    assert(fb.DIAGNOSTIC_HISTORY.count == 1U);
    assert(fb.DIAGNOSTIC_HISTORY.totalRecorded == 1U);
    printf("✓ AcknowledgeDiagnostics retention-clear test passed\n");
}

static void test_acknowledge_rejects_fault_retention(void) {
    HDY_MotionControlFB fb;
    HDY_MotionSegment recipe[1];
    HDY_DiagnosticSnapshot latestSnapshot;

    printf("Testing AcknowledgeDiagnostics fault-retention guard...\n");
    init_controller(&fb);
    recipe[0] = make_position_segment("AckFault", 10.0, HDY_DIRECTION_EXTEND);
    assert(HDY_MotionControlFB_LoadRecipe(&fb, recipe, 1));

    fb.AXIS_REF.position = 0.0;
    fb.AXIS_REF.pressure = 1.0;
    fb.AXIS_REF.timestamp = 0.0;
    assert(HDY_MotionControlFB_StartSegment(&fb, 0, 0.0));
    HDY_MotionControlFB_Execute(&fb);

    fb.AXIS_REF.timestamp = 0.1;
    fb.AXIS_REF.pressure = -0.5;
    HDY_MotionControlFB_Execute(&fb);

    assert(fb.FAULT);
    assert(!HDY_MotionControlFB_AcknowledgeDiagnostics(&fb));
    assert(fb.DIAGNOSTIC.code == HDY_DIAG_CODE_SENSOR_FAULT);
    assert(fb.DIAGNOSTIC_LATCH.code == HDY_DIAG_CODE_SENSOR_FAULT);
    assert(fb.LAST_FAULT_SNAPSHOT.valid);
    assert(fb.DIAGNOSTIC_HISTORY.count >= 1U);
    assert(HDY_DiagnosticsHistory_GetLatest(&fb.DIAGNOSTIC_HISTORY, &latestSnapshot));
    assert(latestSnapshot.diagnostic.code == HDY_DIAG_CODE_SENSOR_FAULT);
    printf("✓ AcknowledgeDiagnostics fault-retention guard test passed\n");
}

static void test_fault_snapshot_captures_protected_stop_context(void) {
    HDY_MotionControlFB fb;
    HDY_MotionSegment recipe[1];

    printf("Testing fault snapshot capture semantics...\n");
    init_controller(&fb);
    recipe[0] = make_time_segment("FaultSnapshot", 1.0, HDY_DIRECTION_EXTEND);
    assert(HDY_MotionControlFB_LoadRecipe(&fb, recipe, 1));

    fb.AXIS_REF.position = 0.0;
    fb.AXIS_REF.velocity = 0.0;
    fb.AXIS_REF.flow = 0.0;
    fb.AXIS_REF.pressure = recipe[0].targetPressure;
    fb.AXIS_REF.timestamp = 0.0;
    assert(HDY_MotionControlFB_StartSegment(&fb, 0, 0.0));
    HDY_MotionControlFB_Execute(&fb);
    assert(fb.DIAGNOSTIC.code == HDY_DIAG_CODE_NONE);
    assert(fb.DIAGNOSTIC_HISTORY.count == 0U);

    fb.AXIS_REF.timestamp = 0.1;
    fb.AXIS_REF.pressure = -0.5;
    HDY_MotionControlFB_Execute(&fb);

    assert(fb.FAULT);
    assert(fb.LAST_FAULT_SNAPSHOT.valid);
    assert(fb.LAST_FAULT_SNAPSHOT.fault);
    assert(fb.LAST_FAULT_SNAPSHOT.status == HDY_STATUS_FAULT);
    assert(fb.LAST_FAULT_SNAPSHOT.segmentIndex == 0U);
    assert(fb.LAST_FAULT_SNAPSHOT.diagnostic.code == HDY_DIAG_CODE_SENSOR_FAULT);
    assert(strcmp(fb.LAST_FAULT_SNAPSHOT.segmentName, "FaultSnapshot") == 0);
    assert(fabs(fb.LAST_FAULT_SNAPSHOT.eventTimestamp - 0.1) < 0.001);
    assert(fb.LAST_FAULT_SNAPSHOT.axisRef.pressure == -0.5);
    assert(fb.DIAGNOSTIC_HISTORY.count == 1U);
    assert(fb.DIAGNOSTIC_HISTORY.totalRecorded == 1U);
    assert(fb.DIAGNOSTIC_HISTORY.entries[0].diagnostic.code == HDY_DIAG_CODE_SENSOR_FAULT);
    printf("✓ Fault snapshot capture test passed\n");
}

static void test_sensor_fault_triggers_protected_stop(void) {
    HDY_MotionControlFB fb;
    HDY_MotionSegment recipe[1];

    printf("Testing invalid sensor feedback protected-stop behavior...\n");
    init_controller(&fb);
    recipe[0] = make_position_segment("SensorFault", 10.0, HDY_DIRECTION_EXTEND);
    assert(HDY_MotionControlFB_LoadRecipe(&fb, recipe, 1));

    fb.AXIS_REF.position = 0.0;
    fb.AXIS_REF.pressure = 1.0;
    fb.AXIS_REF.timestamp = 0.0;
    assert(HDY_MotionControlFB_StartSegment(&fb, 0, 0.0));
    HDY_MotionControlFB_Execute(&fb);
    assert(fb.ACTIVE);

    fb.AXIS_REF.timestamp = 0.1;
    fb.AXIS_REF.pressure = -0.5;
    HDY_MotionControlFB_Execute(&fb);
    assert(!fb.ACTIVE);
    assert(fb.FAULT);
    assert(fb.STATUS == HDY_STATUS_FAULT);
    assert(fb.STATE.protectionAction == HDY_PROTECTION_ACTION_STOP);
    assert(fb.DIAGNOSTIC.code == HDY_DIAG_CODE_SENSOR_FAULT);
    assert(fb.DIAGNOSTIC.severity == HDY_DIAG_SEVERITY_FAULT);
    assert(fb.DIAGNOSTIC.source == HDY_DIAG_SOURCE_SENSOR);
    assert(fb.DIAGNOSTIC.recovery == HDY_DIAG_RECOVERY_CHECK_SENSOR);
    assert(fb.DIAGNOSTIC.protectionAction == HDY_PROTECTION_ACTION_STOP);
    printf("✓ Invalid sensor feedback protected-stop test passed\n");
}

static void test_timestamp_rollback_triggers_protected_stop(void) {
    HDY_MotionControlFB fb;
    HDY_MotionSegment recipe[1];

    printf("Testing timestamp rollback protected-stop behavior...\n");
    init_controller(&fb);
    recipe[0] = make_position_segment("TimestampRollback", 10.0, HDY_DIRECTION_EXTEND);
    assert(HDY_MotionControlFB_LoadRecipe(&fb, recipe, 1));

    fb.AXIS_REF.position = 0.0;
    fb.AXIS_REF.pressure = 1.0;
    fb.AXIS_REF.timestamp = 0.2;
    assert(HDY_MotionControlFB_StartSegment(&fb, 0, 0.2));
    HDY_MotionControlFB_Execute(&fb);
    assert(fb.ACTIVE);

    fb.AXIS_REF.timestamp = 0.1;
    HDY_MotionControlFB_Execute(&fb);
    assert(!fb.ACTIVE);
    assert(fb.FAULT);
    assert(fb.STATUS == HDY_STATUS_FAULT);
    assert(fb.STATE.protectionAction == HDY_PROTECTION_ACTION_STOP);
    assert(fb.DIAGNOSTIC.code == HDY_DIAG_CODE_TIMESTAMP_ROLLBACK);
    assert(fb.DIAGNOSTIC.severity == HDY_DIAG_SEVERITY_FAULT);
    assert(fb.DIAGNOSTIC.source == HDY_DIAG_SOURCE_SENSOR);
    assert(fb.DIAGNOSTIC.recovery == HDY_DIAG_RECOVERY_CHECK_SENSOR);
    assert(fb.DIAGNOSTIC.protectionAction == HDY_PROTECTION_ACTION_STOP);
    printf("✓ Timestamp rollback protected-stop test passed\n");
}

static void test_pressure_segment_timeout_faults_safely(void) {
    HDY_MotionControlFB fb;
    HDY_MotionSegment recipe[1];

    printf("Testing pressure segment timeout safety behavior...\n");
    init_controller(&fb);
    recipe[0] = make_pressure_segment("PressureTimeout", 5.0, 12.0, 3.0);
    recipe[0].pressureController = HDY_PRESSURE_CONTROLLER_PI;
    recipe[0].pressureKp = 0.5;
    recipe[0].pressureKi = 0.8;
    recipe[0].pressureIntegralLimit = 4.0;
    recipe[0].timeoutLimit = 0.25;
    assert(HDY_MotionControlFB_LoadRecipe(&fb, recipe, 1));

    fb.AXIS_REF.pressure = 4.0;
    fb.AXIS_REF.timestamp = 0.0;
    assert(HDY_MotionControlFB_StartSegment(&fb, 0, 0.0));
    HDY_MotionControlFB_Execute(&fb);
    assert(fb.ACTIVE);

    fb.AXIS_REF.timestamp = 0.3;
    HDY_MotionControlFB_Execute(&fb);
    assert(!fb.ACTIVE);
    assert(fb.FAULT);
    assert(fb.STATUS == HDY_STATUS_FAULT);
    assert(fb.PUMP_SPEED == 0.0);
    assert(fb.STATE.protectionAction == HDY_PROTECTION_ACTION_STOP);
    assert(fb.DIAGNOSTIC.timeout);
    assert(fb.DIAGNOSTIC.code == HDY_DIAG_CODE_TIMEOUT);
    assert(fb.DIAGNOSTIC.source == HDY_DIAG_SOURCE_EXECUTION);
    assert(fb.DIAGNOSTIC.recovery == HDY_DIAG_RECOVERY_RESTART_SEGMENT);
    assert(fb.DIAGNOSTIC.protectionAction == HDY_PROTECTION_ACTION_STOP);
    printf("✓ Pressure segment timeout safety test passed\n");
}

static void test_disable_requires_restart(void) {
    HDY_MotionControlFB fb;
    HDY_MotionSegment recipe[1];

    printf("Testing EN disable safety semantics...\n");
    init_controller(&fb);
    recipe[0] = make_position_segment("DisableCase", 5.0, HDY_DIRECTION_EXTEND);
    assert(HDY_MotionControlFB_LoadRecipe(&fb, recipe, 1));

    fb.AXIS_REF.position = 0.0;
    fb.AXIS_REF.timestamp = 0.0;
    assert(HDY_MotionControlFB_StartSegment(&fb, 0, fb.AXIS_REF.timestamp));
    HDY_MotionControlFB_Execute(&fb);
    assert(fb.PUMP_SPEED > 0.0);

    fb.EN = false;
    fb.START_SEGMENT = true;
    fb.START_SEGMENT_INDEX = 0U;
    fb.AXIS_REF.timestamp = 0.1;
    HDY_MotionControlFB_Execute(&fb);
    assert(!fb.ENO);
    assert(!fb.ACTIVE);
    assert(!fb.FAULT);
    assert(fb.STATUS == HDY_STATUS_READY);
    assert(fb.PUMP_SPEED == 0.0);
    assert(!fb.START_SEGMENT);
    assert(fb.STATE.plannedDirection == HDY_DIRECTION_HOLD);

    fb.EN = true;
    fb.AXIS_REF.timestamp = 0.2;
    HDY_MotionControlFB_Execute(&fb);
    assert(fb.ENO);
    assert(!fb.ACTIVE);
    assert(fb.STATUS == HDY_STATUS_READY);
    assert(fb.PUMP_SPEED == 0.0);

    assert(HDY_MotionControlFB_StartSegment(&fb, 0, fb.AXIS_REF.timestamp));
    HDY_MotionControlFB_Execute(&fb);
    assert(fb.ACTIVE);
    assert(fb.PUMP_SPEED > 0.0);
    printf("✓ EN disable safety semantics test passed\n");
}

static void test_abort_forces_safe_outputs(void) {
    HDY_MotionControlFB fb;
    HDY_MotionSegment recipe[1];

    printf("Testing Abort safety semantics...\n");
    init_controller(&fb);
    recipe[0] = make_position_segment("AbortCase", 5.0, HDY_DIRECTION_EXTEND);
    assert(HDY_MotionControlFB_LoadRecipe(&fb, recipe, 1));

    fb.AXIS_REF.position = 0.0;
    fb.AXIS_REF.timestamp = 0.0;
    assert(HDY_MotionControlFB_StartSegment(&fb, 0, fb.AXIS_REF.timestamp));
    HDY_MotionControlFB_Execute(&fb);
    assert(fb.PUMP_SPEED > 0.0);

    fb.START_SEGMENT = true;
    fb.START_SEGMENT_INDEX = 0U;
    assert(HDY_MotionControlFB_Abort(&fb));
    assert(fb.ACTIVE);
    assert(!fb.FINISHED);
    assert(fb.STATUS == HDY_STATUS_RUNNING);

    HDY_MotionControlFB_Execute(&fb);
    assert(!fb.ACTIVE);
    assert(fb.FINISHED);
    assert(!fb.FAULT);
    assert(fb.STATUS == HDY_STATUS_FINISHED);
    assert(fb.FB_STATE == HDY_FB_STATE_ABORTED);
    assert(!fb.SEGMENT_COMPLETED);
    assert(fb.PUMP_SPEED == 0.0);
    assert(!fb.START_SEGMENT);
    assert(fb.STATE.plannedDirection == HDY_DIRECTION_HOLD);
    assert(fb.STATE.currentSegmentName[0] == '\0');
    assert(fb.STATE.protectionAction == HDY_PROTECTION_ACTION_NONE);
    assert(fb.DIAGNOSTIC.code == HDY_DIAG_CODE_ABORTED);
    assert(fb.DIAGNOSTIC.source == HDY_DIAG_SOURCE_COMMAND);
    assert(fb.DIAGNOSTIC.recovery == HDY_DIAG_RECOVERY_NONE);
    assert(fb.DIAGNOSTIC.protectionAction == HDY_PROTECTION_ACTION_NONE);
    assert(strstr(fb.DIAGNOSTIC.message, "Aborted by caller") != NULL);

    HDY_MotionControlFB_Execute(&fb);
    assert(fb.PUMP_SPEED == 0.0);
    assert(!fb.ACTIVE);
    printf("✓ Abort safety semantics test passed\n");
}

static void test_abort_diagnostic_auto_clears_in_finished_hold(void) {
    HDY_MotionControlFB fb;
    HDY_MotionSegment recipe[1];

    printf("Testing Abort diagnostic auto-clear lifecycle...\n");
    init_controller(&fb);
    recipe[0] = make_position_segment("AbortDiag", 5.0, HDY_DIRECTION_EXTEND);
    assert(HDY_MotionControlFB_LoadRecipe(&fb, recipe, 1));

    fb.AXIS_REF.position = 0.0;
    fb.AXIS_REF.timestamp = 0.0;
    assert(HDY_MotionControlFB_StartSegment(&fb, 0, 0.0));
    HDY_MotionControlFB_Execute(&fb);
    assert(HDY_MotionControlFB_Abort(&fb));

    fb.AXIS_REF.timestamp = 0.1;
    HDY_MotionControlFB_Execute(&fb);
    assert(fb.DIAGNOSTIC_LATCH.code == HDY_DIAG_CODE_ABORTED);

    fb.AXIS_REF.timestamp = 0.2;
    HDY_MotionControlFB_Execute(&fb);
    assert(fb.FINISHED);
    assert(!fb.FAULT);
    assert(fb.DIAGNOSTIC.code == HDY_DIAG_CODE_NONE);
    assert(fb.DIAGNOSTIC_LATCH.code == HDY_DIAG_CODE_ABORTED);
    assert(HDY_MotionControlFB_AcknowledgeDiagnostics(&fb));
    assert(fb.DIAGNOSTIC_LATCH.code == HDY_DIAG_CODE_NONE);
    assert(fb.DIAGNOSTIC_HISTORY.count == 0U);
    printf("✓ Abort diagnostic auto-clear test passed\n");
}

static void test_finished_command_diagnostic_auto_clears_in_hold(void) {
    HDY_MotionControlFB fb;
    HDY_MotionSegment recipe[1];

    printf("Testing finished-state command diagnostic auto-clear lifecycle...\n");
    init_controller(&fb);
    recipe[0] = make_position_segment("FinishedDiag", 1.0, HDY_DIRECTION_EXTEND);
    assert(HDY_MotionControlFB_LoadRecipe(&fb, recipe, 1));

    fb.AXIS_REF.position = 0.0;
    fb.AXIS_REF.timestamp = 0.0;
    assert(HDY_MotionControlFB_StartSegment(&fb, 0, 0.0));
    HDY_MotionControlFB_Execute(&fb);

    fb.AXIS_REF.position = 1.0;
    fb.AXIS_REF.timestamp = 0.1;
    HDY_MotionControlFB_Execute(&fb);
    assert(fb.FINISHED);
    assert(fb.SEGMENT_COMPLETED);
    assert(!HDY_MotionControlFB_NextSegment(&fb, 0.1));
    assert(fb.DIAGNOSTIC.code == HDY_DIAG_CODE_RECIPE_ALREADY_FINISHED);
    assert(fb.DIAGNOSTIC_LATCH.code == HDY_DIAG_CODE_RECIPE_ALREADY_FINISHED);

    fb.AXIS_REF.timestamp = 0.2;
    HDY_MotionControlFB_Execute(&fb);
    assert(fb.FINISHED);
    assert(fb.DIAGNOSTIC.code == HDY_DIAG_CODE_NONE);
    assert(fb.DIAGNOSTIC_LATCH.code == HDY_DIAG_CODE_RECIPE_ALREADY_FINISHED);
    assert(HDY_MotionControlFB_AcknowledgeDiagnostics(&fb));
    assert(fb.DIAGNOSTIC_LATCH.code == HDY_DIAG_CODE_NONE);
    printf("✓ Finished-state command diagnostic auto-clear test passed\n");
}

static void test_reset_performs_full_reinitialization(void) {
    HDY_MotionControlFB fb;
    HDY_MotionSegment recipe[1];

    printf("Testing RESET full reinitialization semantics...\n");
    init_controller(&fb);
    recipe[0] = make_position_segment("ResetCase", 6.0, HDY_DIRECTION_EXTEND);
    assert(HDY_MotionControlFB_LoadRecipe(&fb, recipe, 1));
    assert(HDY_MotionControlFB_StartSegment(&fb, 0, 0.0));

    fb.AXIS_REF.timestamp = 0.0;
    HDY_MotionControlFB_Execute(&fb);
    assert(fb.ACTIVE);
    assert(fb.DIAGNOSTIC_LATCH.code != HDY_DIAG_CODE_NONE);
    assert(fb.LAST_DIAGNOSTIC_SNAPSHOT.valid);
    assert(fb.DIAGNOSTIC_HISTORY.count > 0U);

    fb.RESET = true;
    HDY_MotionControlFB_Execute(&fb);
    assert(!fb.EN);
    assert(fb.ENO);
    assert(!fb.RESET);
    assert(fb.RECIPE_SIZE == 0U);
    assert(fb.FLOW_TO_PUMP_SPEED_GAIN == 0.0);
    assert(fb.PUMP_SPEED_LIMIT == 0.0);
    assert(!fb.ACTIVE);
    assert(!fb.FINISHED);
    assert(!fb.FAULT);
    assert(fb.STATUS == HDY_STATUS_IDLE);
    assert(!fb.SEGMENT_COMPLETED);
    assert(fb.STATE.plannedDirection == HDY_DIRECTION_HOLD);
    assert(fb.STATE.currentSegmentName[0] == '\0');
    assert(fb.DIAGNOSTIC.code == HDY_DIAG_CODE_NONE);
    assert(fb.DIAGNOSTIC_LATCH.code == HDY_DIAG_CODE_NONE);
    assert(!fb.LAST_DIAGNOSTIC_SNAPSHOT.valid);
    assert(!fb.LAST_FAULT_SNAPSHOT.valid);
    assert(fb.DIAGNOSTIC_HISTORY.count == 0U);
    assert(fb.DIAGNOSTIC_HISTORY.totalRecorded == 0U);
    printf("✓ RESET full reinitialization semantics test passed\n");
}

static void test_typed_diagnostic_thresholds_override_legacy_tolerance(void) {
    HDY_MotionControlFB fb;
    HDY_MotionSegment recipe[1];

    printf("Testing typed diagnostic thresholds override legacy tolerance...\n");
    init_controller(&fb);
    recipe[0] = make_position_segment("DiagTyped", 20.0, HDY_DIRECTION_EXTEND);
    recipe[0].tolerance = 0.05;
    recipe[0].pressureTolerance = 0.5;
    recipe[0].flowTolerance = 0.2;
    assert(HDY_MotionControlFB_LoadRecipe(&fb, recipe, 1));

    fb.AXIS_REF.position = 0.0;
    fb.AXIS_REF.flow = 11.85;
    fb.AXIS_REF.pressure = 8.4;
    fb.AXIS_REF.timestamp = 0.0;
    assert(HDY_MotionControlFB_StartSegment(&fb, 0, 0.0));

    HDY_MotionControlFB_Execute(&fb);
    assert(fb.ACTIVE);
    assert(!fb.DIAGNOSTIC.overPressure);
    assert(!fb.DIAGNOSTIC.underPressure);
    assert(!fb.DIAGNOSTIC.flowDeviation);
    assert(fb.DIAGNOSTIC.positionDeviation);
    assert(fb.DIAGNOSTIC.velocityDeviation);
    assert(fb.DIAGNOSTIC.code == HDY_DIAG_CODE_POSITION_DEVIATION);
    assert(fb.DIAGNOSTIC.severity == HDY_DIAG_SEVERITY_WARNING);
    assert(fb.DIAGNOSTIC.source == HDY_DIAG_SOURCE_EXECUTION);
    assert(fb.DIAGNOSTIC.protectionAction == HDY_PROTECTION_ACTION_WARNING);
    assert(fb.STATE.protectionAction == HDY_PROTECTION_ACTION_WARNING);
    printf("✓ Typed diagnostic thresholds test passed\n");
}

static void test_timeout_limit_stops_segment_safely(void) {
    HDY_MotionControlFB fb;
    HDY_MotionSegment recipe[1];

    printf("Testing timeoutLimit safe-stop behavior...\n");
    init_controller(&fb);
    recipe[0] = make_position_segment("TimeoutCase", 100.0, HDY_DIRECTION_EXTEND);
    recipe[0].timeoutLimit = 0.25;
    assert(HDY_MotionControlFB_LoadRecipe(&fb, recipe, 1));

    fb.AXIS_REF.position = 0.0;
    fb.AXIS_REF.timestamp = 0.0;
    assert(HDY_MotionControlFB_StartSegment(&fb, 0, 0.0));
    HDY_MotionControlFB_Execute(&fb);
    assert(fb.ACTIVE);
    assert(!fb.DIAGNOSTIC.timeout);

    fb.AXIS_REF.timestamp = 0.3;
    HDY_MotionControlFB_Execute(&fb);
    assert(!fb.ACTIVE);
    assert(!fb.FINISHED);
    assert(!fb.SEGMENT_COMPLETED);
    assert(fb.FAULT);
    assert(fb.STATUS == HDY_STATUS_FAULT);
    assert(fb.STATE.faultActive);
    assert(fb.PUMP_SPEED == 0.0);
    assert(fb.STATE.protectionAction == HDY_PROTECTION_ACTION_STOP);
    assert(fb.DIAGNOSTIC.timeout);
    assert(fb.DIAGNOSTIC.code == HDY_DIAG_CODE_TIMEOUT);
    assert(fb.DIAGNOSTIC.severity == HDY_DIAG_SEVERITY_FAULT);
    assert(fb.DIAGNOSTIC.source == HDY_DIAG_SOURCE_EXECUTION);
    assert(fb.DIAGNOSTIC.recovery == HDY_DIAG_RECOVERY_RESTART_SEGMENT);
    assert(fb.DIAGNOSTIC.protectionAction == HDY_PROTECTION_ACTION_STOP);
    assert(strstr(fb.DIAGNOSTIC.message, "timeout") != NULL);
    printf("✓ timeoutLimit safe-stop test passed\n");
}

static void test_runtime_validation_fault_latches_safe_state(void) {
    HDY_MotionControlFB fb;
    HDY_MotionSegment recipe[1];

    printf("Testing runtime validation fault behavior...\n");
    init_controller(&fb);
    recipe[0] = make_position_segment("RuntimeFault", 20.0, HDY_DIRECTION_EXTEND);
    assert(HDY_MotionControlFB_LoadRecipe(&fb, recipe, 1));

    fb.AXIS_REF.position = 0.0;
    fb.AXIS_REF.timestamp = 0.0;
    assert(HDY_MotionControlFB_StartSegment(&fb, 0, 0.0));
    HDY_MotionControlFB_Execute(&fb);
    assert(fb.ACTIVE);

    fb.FLOW_TO_PUMP_SPEED_GAIN = 0.0;
    fb.AXIS_REF.timestamp = 0.1;
    HDY_MotionControlFB_Execute(&fb);
    assert(!fb.ACTIVE);
    assert(fb.FAULT);
    assert(fb.STATUS == HDY_STATUS_FAULT);
    assert(fb.PUMP_SPEED == 0.0);
    assert(fb.STATE.protectionAction == HDY_PROTECTION_ACTION_STOP);
    assert(fb.DIAGNOSTIC.code == HDY_DIAG_CODE_RUNTIME_CONFIG_INVALID);
    assert(fb.DIAGNOSTIC.severity == HDY_DIAG_SEVERITY_FAULT);
    assert(fb.DIAGNOSTIC.source == HDY_DIAG_SOURCE_RUNTIME);
    assert(fb.DIAGNOSTIC.recovery == HDY_DIAG_RECOVERY_RESET_CONTROLLER);
    assert(fb.DIAGNOSTIC.protectionAction == HDY_PROTECTION_ACTION_STOP);
    assert(strstr(fb.DIAGNOSTIC.message, "FLOW_TO_PUMP_SPEED_GAIN") != NULL);
    printf("✓ Runtime validation fault test passed\n");
}

static void test_parameter_validation(void) {
    HDY_MotionControlFB fb;
    HDY_MotionSegment badRecipe[1];
    HDY_MotionSegment goodRecipe[1];

    printf("Testing recipe and runtime parameter validation...\n");
    init_controller(&fb);

    badRecipe[0] = make_time_segment("BadDirection", 0.5, HDY_DIRECTION_AUTO);
    assert(!HDY_MotionControlFB_LoadRecipe(&fb, badRecipe, 1));
    assert(fb.RECIPE_SIZE == 0U);
    assert(!fb.FAULT);
    assert(fb.STATUS == HDY_STATUS_IDLE);
    assert(fb.DIAGNOSTIC.code == HDY_DIAG_CODE_SEGMENT_INVALID);
    assert(fb.DIAGNOSTIC.source == HDY_DIAG_SOURCE_RECIPE);
    assert(fb.DIAGNOSTIC.recovery == HDY_DIAG_RECOVERY_RELOAD_RECIPE);
    assert(fb.DIAGNOSTIC.protectionAction == HDY_PROTECTION_ACTION_WARNING);
    assert(strstr(fb.DIAGNOSTIC.message, "direction") != NULL);

    badRecipe[0] = make_time_segment("BadPlanner", 0.5, HDY_DIRECTION_EXTEND);
    badRecipe[0].planner = HDY_PLANNER_POSITION_BASED;
    assert(!HDY_MotionControlFB_LoadRecipe(&fb, badRecipe, 1));
    assert(fb.DIAGNOSTIC.code == HDY_DIAG_CODE_SEGMENT_INVALID);
    assert(strstr(fb.DIAGNOSTIC.message, "TIME_BASED") != NULL);

    badRecipe[0] = make_pressure_segment("RbfPressureCfg", 1.0, 10.0, 2.0);
    badRecipe[0].pressureController = HDY_PRESSURE_CONTROLLER_RBF_PID;
    assert(HDY_MotionControlFB_LoadRecipe(&fb, badRecipe, 1));
    assert(fb.RECIPE_SIZE == 1U);
    assert(fb.DIAGNOSTIC.code == HDY_DIAG_CODE_NONE);

    badRecipe[0] = make_pressure_segment("BadRbfProfile", 1.0, 10.0, 2.0);
    badRecipe[0].pressureController = HDY_PRESSURE_CONTROLLER_RBF_PID;
    badRecipe[0].pressureRbfConfig.minKp = 0.9;
    badRecipe[0].pressureRbfConfig.maxKp = 0.85;
    assert(!HDY_MotionControlFB_LoadRecipe(&fb, badRecipe, 1));
    assert(fb.DIAGNOSTIC.code == HDY_DIAG_CODE_SEGMENT_INVALID);
    assert(strstr(fb.DIAGNOSTIC.message, "RBF-PID gain limits") != NULL);

    badRecipe[0] = make_pressure_segment("BadFilter", 1.0, 10.0, 2.0);
    badRecipe[0].pressureController = HDY_PRESSURE_CONTROLLER_PI;
    badRecipe[0].pressureKp = 0.5;
    badRecipe[0].pressureKi = 0.5;
    badRecipe[0].pressureFilterAlpha = 1.5;
    assert(!HDY_MotionControlFB_LoadRecipe(&fb, badRecipe, 1));
    assert(fb.DIAGNOSTIC.code == HDY_DIAG_CODE_SEGMENT_INVALID);
    assert(strstr(fb.DIAGNOSTIC.message, "pressureFilterAlpha") != NULL);

    goodRecipe[0] = make_position_segment("Good", 5.0, HDY_DIRECTION_EXTEND);
    assert(HDY_MotionControlFB_LoadRecipe(&fb, goodRecipe, 1));
    fb.FLOW_TO_PUMP_SPEED_GAIN = 0.0;
    fb.AXIS_REF.position = 0.0;
    assert(!HDY_MotionControlFB_StartSegment(&fb, 0, 0.0));
    assert(!fb.ACTIVE);
    assert(!fb.FAULT);
    assert(fb.DIAGNOSTIC.code == HDY_DIAG_CODE_RUNTIME_CONFIG_INVALID);
    assert(fb.DIAGNOSTIC.source == HDY_DIAG_SOURCE_RUNTIME);
    assert(fb.DIAGNOSTIC.recovery == HDY_DIAG_RECOVERY_RESET_CONTROLLER);
    assert(fb.DIAGNOSTIC.protectionAction == HDY_PROTECTION_ACTION_STOP);
    assert(strstr(fb.DIAGNOSTIC.message, "FLOW_TO_PUMP_SPEED_GAIN") != NULL);

    fb.FLOW_TO_PUMP_SPEED_GAIN = 100.0;
    fb.AXIS_REF.position = 10.0;
    assert(!HDY_MotionControlFB_StartSegment(&fb, 0, 0.0));
    assert(fb.DIAGNOSTIC.code == HDY_DIAG_CODE_START_CONTEXT_INVALID);
    assert(fb.DIAGNOSTIC.source == HDY_DIAG_SOURCE_COMMAND);
    assert(fb.DIAGNOSTIC.recovery == HDY_DIAG_RECOVERY_CHECK_COMMAND);
    assert(fb.DIAGNOSTIC.protectionAction == HDY_PROTECTION_ACTION_WARNING);
    assert(strstr(fb.DIAGNOSTIC.message, "conflicts") != NULL);
    printf("✓ Recipe and runtime parameter validation test passed\n");
}

int main(void) {
    printf("Running MotionControl tests...\n\n");

    test_load_recipe_requires_start_command();
    test_start_segment_and_segment_changed_pulse();
    test_start_segment_command_input();
    test_start_segment_input_uses_rising_edge();
    test_cycle_does_not_sample_command_inputs_without_scan();
    test_cycle_consumes_api_queued_start_command();
    test_start_rejected_in_disabled_state();
    test_start_command_rejected_while_running();
    test_abort_rejected_in_ready_state();
    test_next_rejected_in_aborted_state();
    test_abort_rejected_in_done_state();
    test_acknowledge_rejected_while_running_even_without_live_diagnostic();
    test_acknowledge_allowed_in_disabled_state();
    test_start_allowed_in_aborted_state();
    test_multiple_instances_are_isolated();
    test_segment_completion_and_next_segment();
    test_retract_position_directional_planning();
    test_speed_ramp_retract_directional_planning();
    test_pressure_closed_loop_pi_strategy_accumulates_flow();
    test_pressure_mode_transition_tracks_existing_flow();
    test_runtime_reference_state_exposes_shared_execution_context();
    test_pressure_loop_state_exposes_adaptive_rbf_telemetry();
    test_execution_diagnostics_promote_degraded_status();
    test_diagnostic_flags_expose_minimal_embedded_summary();
    test_diagnostic_latch_and_history_persist_after_live_clear();
    test_diagnostic_history_helpers_preserve_chronological_order();
    test_acknowledge_clears_retention_and_allows_re_recording();
    test_acknowledge_rejects_fault_retention();
    test_fault_snapshot_captures_protected_stop_context();
    test_sensor_fault_triggers_protected_stop();
    test_timestamp_rollback_triggers_protected_stop();
    test_pressure_segment_timeout_faults_safely();
    test_disable_requires_restart();
    test_abort_forces_safe_outputs();
    test_abort_diagnostic_auto_clears_in_finished_hold();
    test_finished_command_diagnostic_auto_clears_in_hold();
    test_reset_performs_full_reinitialization();
    test_typed_diagnostic_thresholds_override_legacy_tolerance();
    test_timeout_limit_stops_segment_safely();
    test_runtime_validation_fault_latches_safe_state();
    test_parameter_validation();

    printf("\n✅ All MotionControl tests passed successfully!\n");
    return 0;
}
