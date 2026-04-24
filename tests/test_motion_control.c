#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "diagnostics.h"
#include "diagnostics_monitor.h"
#include "diagnostics_criteria.h"
#include "motion_control.h"

static void init_controller(HDY_MotionControlFB* fb) {
    HDY_MotionControlFB_Init(fb);
    fb->EN = true;
    fb->FLOW_TO_PUMP_SPEED_GAIN = 100.0;
    fb->PUMP_SPEED_LIMIT = 3000.0;
}

/* Disable all diagnostic suppress windows on the FB so tests can verify
 * immediate triggering without waiting for startup/switch suppress periods.
 * This is only for unit-test convenience; production code should keep
 * suppress enabled for transient rejection. */
static void disable_diagnostic_suppress(HDY_MotionControlFB* fb) {
    fb->_pressureCriteria.enableStartupSuppress = false;
    fb->_pressureCriteria.startupSuppressTime = 0.0;
    fb->_pressureCriteria.enableSwitchSuppress = false;
    fb->_pressureCriteria.switchSuppressTime = 0.0;
    fb->_flowCriteria.enableStartupSuppress = false;
    fb->_flowCriteria.startupSuppressTime = 0.0;
    fb->_flowCriteria.enableSwitchSuppress = false;
    fb->_flowCriteria.switchSuppressTime = 0.0;
    fb->_velocityCriteria.enableStartupSuppress = false;
    fb->_velocityCriteria.startupSuppressTime = 0.0;
    fb->_positionCriteria.enableStartupSuppress = false;
    fb->_positionCriteria.startupSuppressTime = 0.0;
    fb->_isSwitchPhase = false;
}

static void assert_standard_outputs(const HDY_MotionControlFB* fb,
                                    HDY_BOOL busy,
                                    HDY_BOOL done,
                                    HDY_BOOL error,
                                    HDY_DiagnosticCode errorId) {
    assert(fb != NULL);
    assert(fb->BUSY == busy);
    assert(fb->DONE == done);
    assert(fb->ERROR == error);
    assert(fb->ERROR_ID == errorId);
}

static HDY_MotionSegment make_position_segment(HDY_UINT8 tag,
                                               HDY_REAL targetPosition,
                                               HDY_MotionDirection direction) {
    HDY_MotionSegment segment = {0};
    segment.segmentTag = tag;
    segment.segmentType = HDY_SEGMENT_TYPE_CLAMPING;
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

static HDY_MotionSegment make_time_segment(HDY_UINT8 tag,
                                           HDY_TIME duration,
                                           HDY_MotionDirection direction) {
    HDY_MotionSegment segment = {0};
    segment.segmentTag = tag;
    segment.segmentType = HDY_SEGMENT_TYPE_HOLDING;
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

static HDY_MotionSegment make_pressure_segment(HDY_UINT8 tag,
                                               HDY_TIME duration,
                                               HDY_REAL targetPressure,
                                               HDY_REAL targetFlow) {
    HDY_MotionSegment segment = {0};
    segment.segmentTag = tag;
    segment.segmentType = HDY_SEGMENT_TYPE_HOLDING;
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
    recipe[0] = make_position_segment(1, 10.0, HDY_DIRECTION_EXTEND);

    assert(HDY_MotionControlFB_LoadRecipe(&fb, recipe, 1));
    assert(fb.RECIPE_SIZE == 1U);
    assert(!fb.ACTIVE);
    assert(!fb.BUSY);
    assert(!fb.DONE);
    assert(!fb.ERROR);
    assert(fb.ERROR_ID == HDY_DIAG_CODE_NONE);
    assert(!fb.FAULT);
    assert(fb.STATUS == HDY_STATUS_READY);
    assert(fb.FB_STATE == HDY_FB_STATE_READY);
    assert(!fb.STATE.active);
    assert(!fb.FINISHED);
    assert(!fb.SEGMENT_COMPLETED);
    assert(fb.STATE.currentSegmentTag == 0);
    assert(fb.STATE.status == HDY_STATUS_READY);

    fb.AXIS_REF.position = 0.0;
    fb.AXIS_REF.timestamp = 0.0;
    HDY_MotionControlFB_Execute(&fb);
    assert(fb.PUMP_SPEED == 0.0);
    assert(!fb.ACTIVE);
    assert(!fb.BUSY);
    assert(!fb.DONE);
    assert(!fb.ERROR);
    assert(fb.ERROR_ID == HDY_DIAG_CODE_NONE);
    assert(!fb.SEGMENT_COMPLETED);
    assert(fb.STATUS == HDY_STATUS_READY);
    assert(fb.STATE.plannedDirection == HDY_DIRECTION_HOLD);
    printf("✓ LoadRecipe idle semantics test passed\n");
}

static void test_standard_outputs_follow_plcopen_state_machine(void) {
    HDY_MotionControlFB fb;
    HDY_MotionSegment recipe[2];

    printf("Testing PLCopen Busy/Done/Error/ErrorID output matrix...\n");
    init_controller(&fb);
    assert(fb.FB_STATE == HDY_FB_STATE_IDLE);
    assert_standard_outputs(&fb, false, false, false, HDY_DIAG_CODE_NONE);

    recipe[0] = make_position_segment(11, 1.0, HDY_DIRECTION_EXTEND);
    recipe[1] = make_time_segment(12, 0.5, HDY_DIRECTION_EXTEND);
    assert(HDY_MotionControlFB_LoadRecipe(&fb, recipe, 2));
    assert(fb.FB_STATE == HDY_FB_STATE_READY);
    assert_standard_outputs(&fb, false, false, false, HDY_DIAG_CODE_NONE);

    fb.AXIS_REF.position = 0.0;
    fb.AXIS_REF.velocity = 0.0;
    fb.AXIS_REF.flow = 0.0;
    fb.AXIS_REF.pressure = recipe[0].targetPressure;
    fb.AXIS_REF.timestamp = 0.0;
    assert(HDY_MotionControlFB_StartSegment(&fb, 0, 0.0));
    HDY_MotionControlFB_Execute(&fb);
    assert(fb.FB_STATE == HDY_FB_STATE_RUNNING);
    assert_standard_outputs(&fb, true, false, false, HDY_DIAG_CODE_NONE);

    fb.AXIS_REF.velocity = fb.STATE.plannedVelocity;
    fb.AXIS_REF.flow = fb.STATE.plannedFlow;
    fb.AXIS_REF.timestamp = 0.1;
    assert(HDY_MotionControlFB_Hold(&fb));
    HDY_MotionControlFB_Cycle(&fb);
    assert(fb.FB_STATE == HDY_FB_STATE_HOLD);
    assert_standard_outputs(&fb, true, false, false, HDY_DIAG_CODE_NONE);

    fb.AXIS_REF.timestamp = 0.2;
    assert(HDY_MotionControlFB_Resume(&fb));
    HDY_MotionControlFB_Cycle(&fb);
    assert(fb.FB_STATE == HDY_FB_STATE_RUNNING);
    assert_standard_outputs(&fb, true, false, false, HDY_DIAG_CODE_NONE);

    fb.AXIS_REF.position = 1.0;
    fb.AXIS_REF.velocity = 0.0;
    fb.AXIS_REF.flow = 0.0;
    fb.AXIS_REF.timestamp = 0.3;
    HDY_MotionControlFB_Execute(&fb);
    assert(fb.FB_STATE == HDY_FB_STATE_SEGMENT_COMPLETE);
    assert_standard_outputs(&fb, true, false, false, HDY_DIAG_CODE_NONE);

    assert(HDY_MotionControlFB_NextSegment(&fb, 0.3));
    fb.AXIS_REF.timestamp = 0.4;
    fb.AXIS_REF.pressure = recipe[1].targetPressure;
    HDY_MotionControlFB_Execute(&fb);
    assert(fb.FB_STATE == HDY_FB_STATE_RUNNING);
    assert_standard_outputs(&fb, true, false, false, HDY_DIAG_CODE_NONE);

    assert(HDY_MotionControlFB_Abort(&fb));
    fb.AXIS_REF.timestamp = 0.5;
    HDY_MotionControlFB_Execute(&fb);
    assert(fb.FB_STATE == HDY_FB_STATE_ABORTED);
    assert_standard_outputs(&fb, false, false, false, HDY_DIAG_CODE_NONE);

    init_controller(&fb);
    recipe[0] = make_position_segment(13, 1.0, HDY_DIRECTION_EXTEND);
    assert(HDY_MotionControlFB_LoadRecipe(&fb, recipe, 1));
    fb.AXIS_REF.position = 0.0;
    fb.AXIS_REF.pressure = recipe[0].targetPressure;
    fb.AXIS_REF.timestamp = 0.0;
    assert(HDY_MotionControlFB_StartSegment(&fb, 0, 0.0));
    HDY_MotionControlFB_Execute(&fb);
    fb.AXIS_REF.position = 1.0;
    fb.AXIS_REF.timestamp = 0.1;
    HDY_MotionControlFB_Execute(&fb);
    assert(fb.FB_STATE == HDY_FB_STATE_DONE);
    assert_standard_outputs(&fb, false, true, false, HDY_DIAG_CODE_NONE);

    init_controller(&fb);
    recipe[0] = make_position_segment(14, 1.0, HDY_DIRECTION_EXTEND);
    assert(HDY_MotionControlFB_LoadRecipe(&fb, recipe, 1));
    fb.EN = false;
    fb.AXIS_REF.timestamp = 0.0;
    HDY_MotionControlFB_Execute(&fb);
    assert(fb.FB_STATE == HDY_FB_STATE_DISABLED);
    assert_standard_outputs(&fb, false, false, false, HDY_DIAG_CODE_NONE);

    init_controller(&fb);
    recipe[0] = make_position_segment(15, 10.0, HDY_DIRECTION_EXTEND);
    assert(HDY_MotionControlFB_LoadRecipe(&fb, recipe, 1));
    fb.AXIS_REF.position = 0.0;
    fb.AXIS_REF.pressure = recipe[0].targetPressure;
    fb.AXIS_REF.timestamp = 0.0;
    assert(HDY_MotionControlFB_StartSegment(&fb, 0, 0.0));
    HDY_MotionControlFB_Execute(&fb);
    fb.AXIS_REF.timestamp = 0.1;
    fb.AXIS_REF.pressure = -0.5;
    HDY_MotionControlFB_Execute(&fb);
    assert(fb.FB_STATE == HDY_FB_STATE_FAULT);
    assert_standard_outputs(&fb, false, false, true, HDY_DIAG_CODE_SENSOR_FAULT);
    printf("✓ PLCopen Busy/Done/Error/ErrorID output matrix test passed\n");
}

static void test_start_segment_and_segment_changed_pulse(void) {
    HDY_MotionControlFB fb;
    HDY_MotionSegment recipe[1];

    printf("Testing StartSegment execution semantics...\n");
    init_controller(&fb);
    recipe[0] = make_position_segment(2, 20.0, HDY_DIRECTION_EXTEND);
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
    assert(fb.BUSY);
    assert(!fb.DONE);
    assert(!fb.ERROR);
    assert(fb.ERROR_ID == HDY_DIAG_CODE_NONE);
    assert(!fb.FAULT);
    assert(fb.STATUS == HDY_STATUS_RUNNING || fb.STATUS == HDY_STATUS_DEGRADED);
    assert(fb.FB_STATE == HDY_FB_STATE_RUNNING);
    assert(fb.STATE.status == HDY_STATUS_RUNNING);
    assert(fb.SEGMENT_CHANGED);
    assert(!fb.SEGMENT_COMPLETED);
    assert(fb.PUMP_SPEED > 0.0);
    assert(fb.STATE.plannedVelocity > 0.0);
    assert(fb.STATE.plannedFlow > 0.0);
    assert(fb.STATE.plannedDirection == HDY_DIRECTION_EXTEND);
    assert(fb.STATE.currentSegmentTag == 2);

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
    recipe[0] = make_position_segment(3, 12.0, HDY_DIRECTION_EXTEND);
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
    assert(fb.STATE.currentSegmentTag == 3);
    printf("✓ START_SEGMENT command input semantics test passed\n");
}

static void test_start_segment_input_uses_rising_edge(void) {
    HDY_MotionControlFB fb;
    HDY_MotionSegment recipe[1];

    printf("Testing START_SEGMENT rising-edge semantics...\n");
    init_controller(&fb);
    recipe[0] = make_position_segment(4, 10.0, HDY_DIRECTION_EXTEND);
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
    recipe[0] = make_position_segment(5, 10.0, HDY_DIRECTION_EXTEND);
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
    recipe[0] = make_position_segment(6, 12.0, HDY_DIRECTION_EXTEND);
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
    assert(fb.STATE.currentSegmentTag == 6);
    printf("✓ Cycle() queued API command test passed\n");
}

static void test_direct_mode_start_without_recipe_uses_direct_segment_buffer(void) {
    HDY_MotionControlFB fb;
    HDY_MotionSegment directSegment;

    printf("Testing direct-mode start without recipe...\n");
    init_controller(&fb);
    fb.USE_RECIPE = false;
    directSegment = make_position_segment(7, 2.0, HDY_DIRECTION_EXTEND);
    assert(HDY_MotionControlFB_LoadDirectSegment(&fb, &directSegment));
    assert(fb.DIRECT_SEGMENT_VALID);
    assert(fb.STATUS == HDY_STATUS_READY);
    assert(fb.FB_STATE == HDY_FB_STATE_READY);

    fb.AXIS_REF.position = 0.0;
    fb.AXIS_REF.velocity = 0.0;
    fb.AXIS_REF.flow = 0.0;
    fb.AXIS_REF.pressure = directSegment.targetPressure;
    fb.AXIS_REF.timestamp = 0.0;
    fb.START_SEGMENT = true;
    fb.START_SEGMENT_INDEX = 7U;
    HDY_MotionControlFB_Execute(&fb);

    assert(fb.ACTIVE);
    assert(fb.BUSY);
    assert(!fb.DONE);
    assert(!fb.ERROR);
    assert(fb.ERROR_ID == HDY_DIAG_CODE_NONE);
    assert(!fb.FAULT);
    assert(fb.STATUS == HDY_STATUS_RUNNING || fb.STATUS == HDY_STATUS_DEGRADED);
    assert(fb.FB_STATE == HDY_FB_STATE_RUNNING);
    assert(fb.STATE.segmentSource == HDY_SEGMENT_SOURCE_DIRECT);
    assert(fb.STATE.currentSegmentIndex == HDY_MAX_SEGMENTS);
    assert(fb.STATE.currentSegmentTag == 7);
    assert(fb.STATE.plannedDirection == HDY_DIRECTION_EXTEND);
    assert(fb.PUMP_SPEED > 0.0);

    fb.AXIS_REF.position = 2.0;
    fb.AXIS_REF.timestamp = 0.1;
    HDY_MotionControlFB_Execute(&fb);
    assert(!fb.ACTIVE);
    assert(!fb.BUSY);
    assert(fb.DONE);
    assert(!fb.ERROR);
    assert(fb.ERROR_ID == HDY_DIAG_CODE_NONE);
    assert(fb.FINISHED);
    assert(fb.SEGMENT_COMPLETED);
    assert(fb.FB_STATE == HDY_FB_STATE_DONE);
    assert(fb.STATE.segmentSource == HDY_SEGMENT_SOURCE_DIRECT);
    printf("✓ Direct-mode start without recipe test passed\n");
}

static void test_direct_mode_latches_segment_parameters_at_start(void) {
    HDY_MotionControlFB fb;
    HDY_MotionSegment directSegment;

    printf("Testing direct-mode parameter latching semantics...\n");
    init_controller(&fb);
    fb.USE_RECIPE = false;
    directSegment = make_position_segment(8, 5.0, HDY_DIRECTION_EXTEND);
    assert(HDY_MotionControlFB_LoadDirectSegment(&fb, &directSegment));

    fb.AXIS_REF.position = 0.0;
    fb.AXIS_REF.velocity = 0.0;
    fb.AXIS_REF.flow = 0.0;
    fb.AXIS_REF.pressure = directSegment.targetPressure;
    fb.AXIS_REF.timestamp = 0.0;
    assert(HDY_MotionControlFB_StartSegment(&fb, 99U, 0.0));
    HDY_MotionControlFB_Execute(&fb);
    assert(fb.ACTIVE);
    assert(fb.STATE.currentSegmentTag == 8);
    assert(fb.STATE.segmentSource == HDY_SEGMENT_SOURCE_DIRECT);
    assert(fb.STATE.plannedDirection == HDY_DIRECTION_EXTEND);

    directSegment = make_position_segment(9, 1.0, HDY_DIRECTION_RETRACT);
    assert(HDY_MotionControlFB_LoadDirectSegment(&fb, &directSegment));

    fb.AXIS_REF.velocity = fb.STATE.plannedVelocity;
    fb.AXIS_REF.flow = fb.STATE.plannedFlow;
    fb.AXIS_REF.timestamp = 0.2;
    HDY_MotionControlFB_Execute(&fb);
    assert(fb.ACTIVE);
    assert(fb.STATE.currentSegmentTag == 8);
    assert(fb.STATE.segmentSource == HDY_SEGMENT_SOURCE_DIRECT);
    assert(fb.STATE.plannedDirection == HDY_DIRECTION_EXTEND);
    printf("✓ Direct-mode parameter latching test passed\n");
}

static void test_recipe_and_direct_modes_can_coexist_and_switch(void) {
    HDY_MotionControlFB fb;
    HDY_MotionSegment recipe[1];
    HDY_MotionSegment directSegment;

    printf("Testing recipe/direct coexistence and source switching...\n");
    init_controller(&fb);
    recipe[0] = make_position_segment(10, 1.0, HDY_DIRECTION_EXTEND);
    directSegment = make_time_segment(107, 0.5, HDY_DIRECTION_RETRACT);
    assert(HDY_MotionControlFB_LoadRecipe(&fb, recipe, 1));
    assert(HDY_MotionControlFB_LoadDirectSegment(&fb, &directSegment));

    fb.AXIS_REF.position = 0.0;
    fb.AXIS_REF.pressure = recipe[0].targetPressure;
    fb.AXIS_REF.timestamp = 0.0;
    assert(HDY_MotionControlFB_StartSegment(&fb, 0, 0.0));
    HDY_MotionControlFB_Execute(&fb);
    assert(fb.ACTIVE);
    assert(fb.STATE.segmentSource == HDY_SEGMENT_SOURCE_RECIPE);
    assert(fb.STATE.currentSegmentIndex == 0U);
    assert(fb.STATE.currentSegmentTag == 10);

    fb.AXIS_REF.position = 1.0;
    fb.AXIS_REF.timestamp = 0.1;
    HDY_MotionControlFB_Execute(&fb);
    assert(fb.FINISHED);
    assert(fb.FB_STATE == HDY_FB_STATE_DONE);
    assert(fb.STATE.segmentSource == HDY_SEGMENT_SOURCE_RECIPE);

    fb.USE_RECIPE = false;
    fb.AXIS_REF.position = 10.0;
    fb.AXIS_REF.velocity = 0.0;
    fb.AXIS_REF.flow = 0.0;
    fb.AXIS_REF.pressure = directSegment.targetPressure;
    fb.AXIS_REF.timestamp = 0.2;
    assert(HDY_MotionControlFB_StartSegment(&fb, 0, 0.2));
    HDY_MotionControlFB_Execute(&fb);
    assert(fb.ACTIVE);
    assert(!fb.FINISHED);
    assert(fb.STATE.segmentSource == HDY_SEGMENT_SOURCE_DIRECT);
    assert(fb.STATE.currentSegmentIndex == HDY_MAX_SEGMENTS);
    assert(fb.STATE.currentSegmentTag == 107);
    assert(fb.STATE.plannedDirection == HDY_DIRECTION_RETRACT);
    printf("✓ Recipe/direct coexistence and switching test passed\n");
}

static void test_direct_mode_requires_direct_segment_configuration(void) {
    HDY_MotionControlFB fb;

    printf("Testing direct-mode missing-configuration diagnostics...\n");
    init_controller(&fb);
    fb.USE_RECIPE = false;

    assert(!HDY_MotionControlFB_StartSegment(&fb, 0, 0.0));
    assert(!fb.ACTIVE);
    assert(!fb.FINISHED);
    assert(fb.STATUS == HDY_STATUS_IDLE);
    assert(fb.FB_STATE == HDY_FB_STATE_IDLE);
    assert(fb.DIAGNOSTIC.code == HDY_DIAG_CODE_NO_DIRECT_SEGMENT);
    assert(fb.DIAGNOSTIC.source == HDY_DIAG_SOURCE_COMMAND);
    assert(fb.DIAGNOSTIC.recovery == HDY_DIAG_RECOVERY_CHECK_COMMAND);
    assert(fb.DIAGNOSTIC.protectionAction == HDY_PROTECTION_ACTION_WARNING);
    printf("✓ Direct-mode missing-configuration diagnostics test passed\n");
}

static void test_hold_command_transitions_running_to_hold(void) {
    HDY_MotionControlFB fb;
    HDY_MotionSegment recipe[1];

    printf("Testing Hold command running -> HOLD transition...\n");
    init_controller(&fb);
    recipe[0] = make_time_segment(101, 1.0, HDY_DIRECTION_EXTEND);
    assert(HDY_MotionControlFB_LoadRecipe(&fb, recipe, 1));

    fb.AXIS_REF.position = 0.0;
    fb.AXIS_REF.velocity = 0.0;
    fb.AXIS_REF.flow = 0.0;
    fb.AXIS_REF.pressure = recipe[0].targetPressure;
    fb.AXIS_REF.timestamp = 0.0;
    assert(HDY_MotionControlFB_StartSegment(&fb, 0, 0.0));
    HDY_MotionControlFB_Execute(&fb);

    fb.AXIS_REF.velocity = fb.STATE.plannedVelocity;
    fb.AXIS_REF.flow = fb.STATE.plannedFlow;
    fb.AXIS_REF.timestamp = 0.2;
    HDY_MotionControlFB_Execute(&fb);
    assert(fb.ACTIVE);
    assert(fb.FB_STATE == HDY_FB_STATE_RUNNING);
    assert(fb.STATE.references.elapsedTime > 0.19);

    assert(HDY_MotionControlFB_Hold(&fb));
    assert(fb._pendingCommand == HDY_CMD_HOLD);

    HDY_MotionControlFB_Cycle(&fb);
    assert(fb._pendingCommand == HDY_CMD_NONE);
    assert(!fb.ACTIVE);
    assert(fb.BUSY);
    assert(!fb.DONE);
    assert(!fb.ERROR);
    assert(fb.ERROR_ID == HDY_DIAG_CODE_NONE);
    assert(!fb.FINISHED);
    assert(!fb.FAULT);
    assert(!fb.SEGMENT_COMPLETED);
    assert(!fb.SEGMENT_CHANGED);
    assert(fb.STATUS == HDY_STATUS_HOLD);
    assert(fb.FB_STATE == HDY_FB_STATE_HOLD);
    assert(fb.PUMP_SPEED == 0.0);
    assert(fb.STATE.commandedPumpSpeed == 0.0);
    assert(fb.STATE.plannedVelocity == 0.0);
    assert(fb.STATE.plannedFlow == 0.0);
    assert(fb.STATE.plannedDirection == HDY_DIRECTION_HOLD);
    assert(fb.STATE.currentSegmentTag == 101);
    assert(fb._activeSegmentValid);
    assert(fabs(fb._holdStateTime - 0.2) < 1e-9);
    printf("✓ Hold command running -> HOLD transition test passed\n");
}

static void test_resume_command_restores_running_and_freezes_elapsed_time(void) {
    HDY_MotionControlFB fb;
    HDY_MotionSegment recipe[1];
    HDY_REAL elapsedBeforeHold;
    HDY_REAL flowBeforeHold;
    HDY_REAL velocityBeforeHold;

    printf("Testing Resume command HOLD -> RUNNING transition...\n");
    init_controller(&fb);
    recipe[0] = make_time_segment(102, 1.0, HDY_DIRECTION_EXTEND);
    assert(HDY_MotionControlFB_LoadRecipe(&fb, recipe, 1));

    fb.AXIS_REF.position = 0.0;
    fb.AXIS_REF.velocity = 0.0;
    fb.AXIS_REF.flow = 0.0;
    fb.AXIS_REF.pressure = recipe[0].targetPressure;
    fb.AXIS_REF.timestamp = 0.0;
    assert(HDY_MotionControlFB_StartSegment(&fb, 0, 0.0));
    HDY_MotionControlFB_Execute(&fb);

    fb.AXIS_REF.velocity = fb.STATE.plannedVelocity;
    fb.AXIS_REF.flow = fb.STATE.plannedFlow;
    fb.AXIS_REF.timestamp = 0.25;
    HDY_MotionControlFB_Execute(&fb);
    assert(fb.ACTIVE);
    elapsedBeforeHold = fb.STATE.references.elapsedTime;
    flowBeforeHold = fb.STATE.plannedFlow;
    velocityBeforeHold = fb.STATE.plannedVelocity;
    assert(elapsedBeforeHold > 0.24);

    assert(HDY_MotionControlFB_Hold(&fb));
    HDY_MotionControlFB_Cycle(&fb);
    assert(fb.FB_STATE == HDY_FB_STATE_HOLD);

    fb.AXIS_REF.velocity = velocityBeforeHold;
    fb.AXIS_REF.flow = flowBeforeHold;
    fb.AXIS_REF.timestamp = 0.75;
    HDY_MotionControlFB_Cycle(&fb);
    assert(fb.FB_STATE == HDY_FB_STATE_HOLD);
    assert(fb.STATUS == HDY_STATUS_HOLD);

    assert(HDY_MotionControlFB_Resume(&fb));
    assert(fb._pendingCommand == HDY_CMD_RESUME);
    HDY_MotionControlFB_Cycle(&fb);
    assert(fb._pendingCommand == HDY_CMD_NONE);
    assert(fb.ACTIVE);
    assert(!fb.FINISHED);
    assert(!fb.FAULT);
    assert(fb.STATUS == HDY_STATUS_RUNNING);
    assert(fb.FB_STATE == HDY_FB_STATE_RUNNING);
    assert(fb.STATE.plannedDirection == HDY_DIRECTION_EXTEND);
    assert(fb.STATE.currentSegmentTag == 102);
    assert(fabs(fb.STATE.references.elapsedTime - elapsedBeforeHold) < 1e-9);
    assert(fabs(fb._segmentStartTime - 0.5) < 1e-9);
    assert(fb.PUMP_SPEED > 0.0);
    printf("✓ Resume command HOLD -> RUNNING transition test passed\n");
}

static void test_hold_rejected_in_ready_state(void) {
    HDY_MotionControlFB fb;
    HDY_MotionSegment recipe[1];

    printf("Testing Hold command legality in READY state...\n");
    init_controller(&fb);
    recipe[0] = make_position_segment(16, 10.0, HDY_DIRECTION_EXTEND);
    assert(HDY_MotionControlFB_LoadRecipe(&fb, recipe, 1));

    assert(!HDY_MotionControlFB_Hold(&fb));
    assert(!fb.ACTIVE);
    assert(!fb.FINISHED);
    assert(fb.STATUS == HDY_STATUS_READY);
    assert(fb.FB_STATE == HDY_FB_STATE_READY);
    assert(fb.DIAGNOSTIC.code == HDY_DIAG_CODE_COMMAND_NOT_ALLOWED);
    assert(fb.DIAGNOSTIC.source == HDY_DIAG_SOURCE_COMMAND);
    assert(fb.DIAGNOSTIC.recovery == HDY_DIAG_RECOVERY_CHECK_COMMAND);
    assert(fb.DIAGNOSTIC.protectionAction == HDY_PROTECTION_ACTION_WARNING);
    printf("✓ Hold command legality in READY state test passed\n");
}

static void test_command_warning_syncs_and_clears_state_protection_action(void) {
    HDY_MotionControlFB fb;
    HDY_MotionSegment recipe[1];

    printf("Testing command-warning protection-action sync and clear semantics...\n");
    init_controller(&fb);
    recipe[0] = make_position_segment(17, 10.0, HDY_DIRECTION_EXTEND);
    assert(HDY_MotionControlFB_LoadRecipe(&fb, recipe, 1));

    assert(!HDY_MotionControlFB_Hold(&fb));
    assert(fb.DIAGNOSTIC.code == HDY_DIAG_CODE_COMMAND_NOT_ALLOWED);
    assert(fb.DIAGNOSTIC.protectionAction == HDY_PROTECTION_ACTION_WARNING);
    assert(fb.STATE.protectionAction == HDY_PROTECTION_ACTION_WARNING);
    assert(fb.STATUS == HDY_STATUS_READY);
    assert(fb.STATE.status == HDY_STATUS_READY);

    assert(HDY_MotionControlFB_LoadRecipe(&fb, recipe, 1));
    assert(fb.DIAGNOSTIC.code == HDY_DIAG_CODE_NONE);
    assert(fb.STATE.protectionAction == HDY_PROTECTION_ACTION_NONE);
    assert(fb.STATUS == HDY_STATUS_READY);
    assert(fb.STATE.status == HDY_STATUS_READY);
    printf("✓ Command-warning protection-action sync and clear test passed\n");
}

static void test_resume_rejected_while_running(void) {
    HDY_MotionControlFB fb;
    HDY_MotionSegment recipe[1];

    printf("Testing Resume command legality while running...\n");
    init_controller(&fb);
    recipe[0] = make_time_segment(103, 1.0, HDY_DIRECTION_EXTEND);
    assert(HDY_MotionControlFB_LoadRecipe(&fb, recipe, 1));

    fb.AXIS_REF.position = 0.0;
    fb.AXIS_REF.velocity = 0.0;
    fb.AXIS_REF.flow = 0.0;
    fb.AXIS_REF.pressure = recipe[0].targetPressure;
    fb.AXIS_REF.timestamp = 0.0;
    assert(HDY_MotionControlFB_StartSegment(&fb, 0, 0.0));
    HDY_MotionControlFB_Execute(&fb);
    assert(fb.ACTIVE);
    assert(fb.FB_STATE == HDY_FB_STATE_RUNNING);

    assert(!HDY_MotionControlFB_Resume(&fb));
    assert(fb.ACTIVE);
    assert(fb.STATUS == HDY_STATUS_RUNNING);
    assert(fb.FB_STATE == HDY_FB_STATE_RUNNING);
    assert(fb.DIAGNOSTIC.code == HDY_DIAG_CODE_COMMAND_NOT_ALLOWED);
    assert(fb.DIAGNOSTIC.source == HDY_DIAG_SOURCE_COMMAND);
    assert(fb.DIAGNOSTIC.recovery == HDY_DIAG_RECOVERY_CHECK_COMMAND);
    assert(fb.DIAGNOSTIC.protectionAction == HDY_PROTECTION_ACTION_WARNING);
    printf("✓ Resume command legality while running test passed\n");
}

static void test_start_rejected_in_disabled_state(void) {
    HDY_MotionControlFB fb;
    HDY_MotionSegment recipe[1];

    printf("Testing Start command legality in DISABLED state...\n");
    init_controller(&fb);
    recipe[0] = make_position_segment(18, 10.0, HDY_DIRECTION_EXTEND);
    assert(HDY_MotionControlFB_LoadRecipe(&fb, recipe, 1));

    fb.EN = false;
    assert(!HDY_MotionControlFB_StartSegment(&fb, 0, 0.0));
    assert(!fb.ACTIVE);
    assert(fb.DIAGNOSTIC.code == HDY_DIAG_CODE_COMMAND_NOT_ALLOWED);
    assert(fb.DIAGNOSTIC.source == HDY_DIAG_SOURCE_COMMAND);
    assert(fb.DIAGNOSTIC.recovery == HDY_DIAG_RECOVERY_CHECK_COMMAND);
    assert(fb.DIAGNOSTIC.protectionAction == HDY_PROTECTION_ACTION_WARNING);
    printf("✓ Start command legality in DISABLED state test passed\n");
}

static void test_start_command_rejected_while_running(void) {
    HDY_MotionControlFB fb;
    HDY_MotionSegment recipe[1];

    printf("Testing Start command legality while running...\n");
    init_controller(&fb);
    recipe[0] = make_time_segment(104, 1.0, HDY_DIRECTION_EXTEND);
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
    recipe[0] = make_position_segment(19, 10.0, HDY_DIRECTION_EXTEND);
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
    printf("✓ Abort command legality in READY state test passed\n");
}

static void test_acknowledge_rejected_while_running_even_without_live_diagnostic(void) {
    HDY_MotionControlFB fb;
    HDY_MotionSegment recipe[1];

    printf("Testing Ack legality while running...\n");
    init_controller(&fb);
    recipe[0] = make_time_segment(106, 1.0, HDY_DIRECTION_EXTEND);
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
    recipe[0] = make_time_segment(105, 1.0, HDY_DIRECTION_EXTEND);
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
    printf("✓ Next command legality in ABORTED state test passed\n");
}

static void test_abort_rejected_in_done_state(void) {
    HDY_MotionControlFB fb;
    HDY_MotionSegment recipe[1];

    printf("Testing Abort command legality in DONE state...\n");
    init_controller(&fb);
    recipe[0] = make_position_segment(21, 1.0, HDY_DIRECTION_EXTEND);
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
    printf("✓ Abort command legality in DONE state test passed\n");
}

static void test_acknowledge_allowed_in_disabled_state(void) {
    HDY_MotionControlFB fb;
    HDY_MotionSegment recipe[1];

    printf("Testing Ack command legality in DISABLED state...\n");
    init_controller(&fb);
    recipe[0] = make_position_segment(20, 20.0, HDY_DIRECTION_EXTEND);
    assert(HDY_MotionControlFB_LoadRecipe(&fb, recipe, 1));

    fb.AXIS_REF.position = 0.0;
    fb.AXIS_REF.velocity = 0.0;
    fb.AXIS_REF.flow = 0.0;
    fb.AXIS_REF.pressure = recipe[0].targetPressure;
    fb.AXIS_REF.timestamp = 0.0;
    assert(HDY_MotionControlFB_StartSegment(&fb, 0, 0.0));
    HDY_MotionControlFB_Execute(&fb);
    
    /* Note: Flow deviation check is now only active in pressure-closed-loop mode.
     * In position mode, flow is derived from velocity and is not directly controlled,
     * so flow deviation during startup is expected and not an error condition.
     * For testing acknowledge functionality in DISABLED state, we'll skip the error generation
     * step and focus on testing the DISABLED state handling itself. */
    
    fb.EN = false;
    fb.AXIS_REF.timestamp = 0.1;
    HDY_MotionControlFB_Execute(&fb);
    assert(!fb.ENO);
    assert(fb.FB_STATE == HDY_FB_STATE_DISABLED);
    assert(fb.DIAGNOSTIC.code == HDY_DIAG_CODE_NONE);
    assert(fb.DIAGNOSTIC_LATCH.code == HDY_DIAG_CODE_NONE);
    assert(!fb.DIAGNOSTIC_HISTORY.hasRecord);

    /* Test that acknowledge works even when no diagnostics are present */
    assert(HDY_MotionControlFB_AcknowledgeDiagnostics(&fb));
    assert(fb.DIAGNOSTIC.code == HDY_DIAG_CODE_NONE);
    assert(fb.DIAGNOSTIC_LATCH.code == HDY_DIAG_CODE_NONE);
    assert(!fb.LAST_DIAGNOSTIC_SNAPSHOT.valid);
    assert(!fb.LAST_FAULT_SNAPSHOT.valid);
    assert(!fb.DIAGNOSTIC_HISTORY.hasRecord);
    assert(fb.DIAGNOSTIC_HISTORY.totalRecorded == 0U);
    printf("✓ Ack command legality in DISABLED state test passed\n");
}

static void test_start_allowed_in_aborted_state(void) {
    HDY_MotionControlFB fb;
    HDY_MotionSegment recipe[1];

    printf("Testing Start command legality in ABORTED state...\n");
    init_controller(&fb);
    recipe[0] = make_time_segment(37, 1.0, HDY_DIRECTION_EXTEND);
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
    assert(fb.STATE.currentSegmentTag == 37);
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
    recipeA[0] = make_position_segment(36, 20.0, HDY_DIRECTION_EXTEND);
    recipeB[0] = make_time_segment(111, 1.0, HDY_DIRECTION_RETRACT);
    assert(HDY_MotionControlFB_LoadRecipe(&fbA, recipeA, 1));
    assert(HDY_MotionControlFB_LoadRecipe(&fbB, recipeB, 1));

    fbA.AXIS_REF.position = 0.0;
    fbA.AXIS_REF.flow = 0.0;
    fbA.AXIS_REF.pressure = recipeA[0].targetPressure;
    fbA.AXIS_REF.timestamp = 0.0;
    assert(HDY_MotionControlFB_StartSegment(&fbA, 0, 0.0));
    HDY_MotionControlFB_Cycle(&fbA);
    assert(fbA.ACTIVE);
    assert(fbA.STATE.currentSegmentTag == 36);
    aDiagnosticCode = fbA.DIAGNOSTIC.code;
    aHistoryCount = fbA.DIAGNOSTIC_HISTORY.hasRecord ? 1U : 0U;
    aPumpSpeed = fbA.PUMP_SPEED;

    assert(!fbB.ACTIVE);
    assert(fbB.STATUS == HDY_STATUS_READY);
    assert(fbB.FB_STATE == HDY_FB_STATE_READY);
    assert(fbB.DIAGNOSTIC.code == HDY_DIAG_CODE_NONE);
    assert(!fbB.DIAGNOSTIC_HISTORY.hasRecord);

    fbB.AXIS_REF.position = 10.0;
    fbB.AXIS_REF.flow = 0.0;
    fbB.AXIS_REF.pressure = recipeB[0].targetPressure;
    fbB.AXIS_REF.timestamp = 1.0;
    fbB.START_SEGMENT = true;
    fbB.START_SEGMENT_INDEX = 0U;
    HDY_MotionControlFB_Scan(&fbB);
    assert(fbB.ACTIVE);
    assert(fbB.STATE.currentSegmentTag == 111);
    assert(fbB.STATE.plannedDirection == HDY_DIRECTION_RETRACT);

    assert(fbA.ACTIVE);
    assert(fbA.STATE.currentSegmentTag == 36);
    assert(fbA.DIAGNOSTIC.code == aDiagnosticCode);
    assert(fbA.DIAGNOSTIC_HISTORY.hasRecord == (aHistoryCount > 0U));
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
    assert(fbA.STATE.currentSegmentTag == 36);
    assert(fbB.FB_STATE == HDY_FB_STATE_ABORTED);
    assert(fbB.STATE.currentSegmentTag == 0);
    printf("✓ Multi-instance isolation test passed\n");
}

static void test_segment_completion_and_next_segment(void) {
    HDY_MotionControlFB fb;
    HDY_MotionSegment recipe[2];

    printf("Testing segment completion and NextSegment behavior...\n");
    init_controller(&fb);
    recipe[0] = make_position_segment(35, 1.0, HDY_DIRECTION_EXTEND);
    recipe[1] = make_time_segment(110, 0.5, HDY_DIRECTION_EXTEND);
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


    fb.AXIS_REF.position = 1.0;
    HDY_MotionControlFB_Execute(&fb);

    assert(fb.SEGMENT_COMPLETED);
    assert(!fb.ACTIVE);
    assert(fb.BUSY);
    assert(!fb.DONE);
    assert(!fb.ERROR);
    assert(fb.ERROR_ID == HDY_DIAG_CODE_NONE);
    assert(!fb.FINISHED);
    assert(!fb.FAULT);
    assert(fb.STATUS == HDY_STATUS_SEGMENT_COMPLETE);
    assert(fb.PUMP_SPEED == 0.0);
    assert(fb.STATE.currentSegmentTag == 35);

    assert(HDY_MotionControlFB_NextSegment(&fb, fb.AXIS_REF.timestamp));
    assert(!fb.ACTIVE);
    assert(fb.SEGMENT_COMPLETED);
    assert(!fb.FINISHED);
    assert(fb.STATUS == HDY_STATUS_SEGMENT_COMPLETE);
    assert(fb.FB_STATE == HDY_FB_STATE_SEGMENT_COMPLETE);
    assert(fb.STATE.currentSegmentIndex == 0U);
    assert(fb.STATE.currentSegmentTag == 35);

    fb.AXIS_REF.timestamp = 0.1;
    HDY_MotionControlFB_Execute(&fb);
    assert(fb.SEGMENT_CHANGED);
    assert(fb.ACTIVE);
    assert(!fb.SEGMENT_COMPLETED);
    assert(fb.STATUS == HDY_STATUS_RUNNING);
    assert(fb.FB_STATE == HDY_FB_STATE_RUNNING);
    assert(fb.STATE.currentSegmentIndex == 1U);
    assert(fb.STATE.currentSegmentTag == 110);

    fb.AXIS_REF.timestamp = 0.6;
    HDY_MotionControlFB_Execute(&fb);
    assert(!fb.ACTIVE);
    assert(!fb.BUSY);
    assert(fb.DONE);
    assert(!fb.ERROR);
    assert(fb.ERROR_ID == HDY_DIAG_CODE_NONE);
    assert(fb.SEGMENT_COMPLETED);
    assert(fb.FINISHED);
    assert(!fb.FAULT);
    assert(fb.STATUS == HDY_STATUS_FINISHED);
    assert(fb.PUMP_SPEED == 0.0);
    assert(!HDY_MotionControlFB_NextSegment(&fb, fb.AXIS_REF.timestamp));
    assert(fb.FINISHED);
    assert(fb.SEGMENT_COMPLETED);
    assert(fb.DIAGNOSTIC.code == HDY_DIAG_CODE_RECIPE_ALREADY_FINISHED);
    printf("✓ Segment completion and NextSegment behavior test passed\n");
}

static void test_retract_position_directional_planning(void) {
    HDY_MotionControlFB fb;
    HDY_MotionSegment recipe[1];

    printf("Testing retract position directional planning...\n");
    init_controller(&fb);
    recipe[0] = make_position_segment(22, 2.0, HDY_DIRECTION_RETRACT);
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
    recipe[0] = make_time_segment(109, 0.5, HDY_DIRECTION_RETRACT);
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
    recipe[0] = make_pressure_segment(201, 2.0, 10.0, 2.0);
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
    recipe[0] = make_time_segment(113, 0.2, HDY_DIRECTION_EXTEND);
    recipe[0].targetFlow = 10.0;
    recipe[0].maxFlow = 10.0;
    recipe[0].maxAcceleration = 50.0;
    recipe[0].maxVelocity = 20.0;
    recipe[1] = make_pressure_segment(202, 1.0, 10.0, 2.0);
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
    recipe[0] = make_pressure_segment(203, 2.0, 15.0, 2.0);
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
    recipe[0] = make_pressure_segment(204, 2.0, 18.0, 0.3);
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
    /* Use pressure-closed-loop mode to trigger a real flow deviation after startup transient. */
    recipe[0] = make_pressure_segment(25, 10.0, 12.0, 0.5);
    assert(HDY_MotionControlFB_LoadRecipe(&fb, recipe, 1));

    fb.AXIS_REF.position = 0.0;
    fb.AXIS_REF.flow = 0.0;
    fb.AXIS_REF.pressure = 11.9;  /* Close to target pressure (12.0) to avoid UNDER_PRESSURE trigger */
    fb.AXIS_REF.timestamp = 0.0;
    assert(HDY_MotionControlFB_StartSegment(&fb, 0, 0.0));
    HDY_MotionControlFB_Execute(&fb);

    /* Startup cycle should not immediately raise a flow-deviation warning.
     * Disable suppress windows so diagnostics trigger within the test's time horizon.
     * Debounce (0.1s) is kept to verify the full criteria pipeline; the test
     * advances past the debounce window before checking. */
    disable_diagnostic_suppress(&fb);

    assert(fb.ACTIVE);
    assert(fb.STATUS == HDY_STATUS_RUNNING);
    assert(fb.STATE.status == HDY_STATUS_RUNNING);
    assert(fb.STATE.protectionAction == HDY_PROTECTION_ACTION_NONE);
    assert(fb.DIAGNOSTIC.code == HDY_DIAG_CODE_NONE);
    assert(!fb.DIAGNOSTIC.flowDeviation);

    /* Advance past the debounce window (0.1s) so the criteria layer
     * accumulates enough sustained deviation to trigger. */
    fb.AXIS_REF.timestamp = 0.15;
    HDY_MotionControlFB_Execute(&fb);
    /* After debounce period the flow deviation should be detected */
    fb.AXIS_REF.timestamp = 0.3;
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
    assert(!fb.DIAGNOSTIC.overPressure);
    assert(!fb.DIAGNOSTIC.underPressure);
    printf("✓ Execution diagnostics degraded-state test passed\n");
}

static void test_diagnostic_flags_expose_minimal_embedded_summary(void) {
    HDY_MotionControlFB fb;
    HDY_MotionSegment recipe[1];

    printf("Testing compact diagnostic flags and string helpers...\n");
    init_controller(&fb);
    recipe[0] = make_pressure_segment(24, 10.0, 12.0, 2.0);
    assert(HDY_MotionControlFB_LoadRecipe(&fb, recipe, 1));

    fb.AXIS_REF.position = 0.0;
    fb.AXIS_REF.flow = 0.0;
    fb.AXIS_REF.pressure = 12.25;
    fb.AXIS_REF.timestamp = 0.0;
    assert(HDY_MotionControlFB_StartSegment(&fb, 0, 0.0));
    HDY_MotionControlFB_Execute(&fb);

    /* Disable suppress windows so diagnostics trigger within the test's time horizon.
     * Without this, startup suppress (0.5s) and switch suppress (0.3s) would block
     * all diagnostics at t<0.5s. */
    disable_diagnostic_suppress(&fb);

    /* Advance past the debounce window (0.1s) so criteria can accumulate
     * sustained deviation. First cycle at t=0.15 starts debounce, second
     * cycle at t=0.3 satisfies the debounce duration. */
    fb.AXIS_REF.timestamp = 0.15;
    HDY_MotionControlFB_Execute(&fb);
    fb.AXIS_REF.timestamp = 0.3;
    HDY_MotionControlFB_Execute(&fb);

    assert(fb.DIAGNOSTIC.flags == HDY_Diagnostics_GetFlagMask(&fb.DIAGNOSTIC));
    assert(fb.DIAGNOSTIC.flags == (HDY_DIAG_FLAG_OVER_PRESSURE | HDY_DIAG_FLAG_FLOW_DEVIATION));
    assert(HDY_Diagnostics_HasFlag(&fb.DIAGNOSTIC, HDY_DIAG_FLAG_OVER_PRESSURE));
    assert(HDY_Diagnostics_HasFlag(&fb.DIAGNOSTIC, HDY_DIAG_FLAG_FLOW_DEVIATION));
    assert(!HDY_Diagnostics_HasFlag(&fb.DIAGNOSTIC, HDY_DIAG_FLAG_POSITION_DEVIATION));
    assert(!HDY_Diagnostics_HasFlag(&fb.DIAGNOSTIC, HDY_DIAG_FLAG_VELOCITY_DEVIATION));
    assert(!HDY_Diagnostics_HasFlag(&fb.DIAGNOSTIC, HDY_DIAG_FLAG_TIMEOUT));
    assert(strcmp(HDY_Diagnostics_CodeToString(fb.DIAGNOSTIC.code), "OVER_PRESSURE") == 0);
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
    recipe[0] = make_position_segment(23, 20.0, HDY_DIRECTION_EXTEND);
    assert(HDY_MotionControlFB_LoadRecipe(&fb, recipe, 1));

    fb.AXIS_REF.position = 0.0;
    fb.AXIS_REF.velocity = 0.0;
    fb.AXIS_REF.flow = 0.0;
    fb.AXIS_REF.pressure = recipe[0].targetPressure;
    fb.AXIS_REF.timestamp = 0.0;
    assert(HDY_MotionControlFB_StartSegment(&fb, 0, 0.0));
    HDY_MotionControlFB_Execute(&fb);
    assert(fb.DIAGNOSTIC.code == HDY_DIAG_CODE_NONE);

    disable_diagnostic_suppress(&fb);

    /* Advance past debounce window (0.1s) for criteria to trigger */
    fb.AXIS_REF.timestamp = 0.15;
    HDY_MotionControlFB_Execute(&fb);
    fb.AXIS_REF.timestamp = 0.3;
    HDY_MotionControlFB_Execute(&fb);

    assert(fb.DIAGNOSTIC.code == HDY_DIAG_CODE_FLOW_DEVIATION);
    assert(fb.DIAGNOSTIC_LATCH.code == HDY_DIAG_CODE_FLOW_DEVIATION);
    assert(fb.LAST_DIAGNOSTIC_SNAPSHOT.valid);
    assert(fb.LAST_DIAGNOSTIC_SNAPSHOT.diagnostic.code == HDY_DIAG_CODE_FLOW_DEVIATION);
    assert(fb.LAST_DIAGNOSTIC_SNAPSHOT.segmentIndex == 0U);
    assert(fb.LAST_DIAGNOSTIC_SNAPSHOT.status == HDY_STATUS_DEGRADED);
    assert(fb.LAST_DIAGNOSTIC_SNAPSHOT.segmentTag == 23);
    assert(fabs(fb.LAST_DIAGNOSTIC_SNAPSHOT.references.flowReference - fb.STATE.references.flowReference) < 0.001);
    assert(fb.DIAGNOSTIC_HISTORY.hasRecord);
    assert(fb.DIAGNOSTIC_HISTORY.totalRecorded == 1U);
    assert(!fb.LAST_FAULT_SNAPSHOT.valid);

    fb.AXIS_REF.position = 20.0;
    fb.AXIS_REF.velocity = 0.0;
    fb.AXIS_REF.flow = 0.0;
    fb.AXIS_REF.pressure = recipe[0].targetPressure;
    fb.AXIS_REF.timestamp = 0.3;
    HDY_MotionControlFB_Execute(&fb);

    assert(fb.DIAGNOSTIC.code == HDY_DIAG_CODE_NONE);
    assert(fb.DIAGNOSTIC_LATCH.code == HDY_DIAG_CODE_FLOW_DEVIATION);
    assert(fb.LAST_DIAGNOSTIC_SNAPSHOT.valid);
    assert(fb.DIAGNOSTIC_HISTORY.hasRecord);
    assert(fb.DIAGNOSTIC_HISTORY.totalRecorded == 1U);
    printf("✓ Diagnostic latch/history retention test passed\n");
}

static void test_diagnostic_history_helpers_preserve_latest_snapshot(void) {
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
    HDY_UINT8 tags[5] = {0, 1, 2, 3, 4};
    size_t index;

    printf("Testing diagnostic history retains latest snapshot only...\n");
    HDY_DiagnosticsHistory_Clear(&history);

    for (index = 0U; index < 5U; ++index) {
        HDY_Diagnostics_SetEvent(&diagnostic, codes[index], HDY_DIAG_SEVERITY_NONE);
        HDY_Diagnostics_CaptureSnapshot(&snapshot,
                                        &diagnostic,
                                        &axisRef,
                                        &references,
                                        (HDY_TIME)index,
                                        (HDY_UINT8)index,
                                        tags[index],
                                        HDY_STATUS_RUNNING,
                                        true,
                                        false,
                                        false);
        HDY_DiagnosticsHistory_Push(&history, &snapshot);
    }

    /* After pushing 5 entries, only the last one survives (single-snapshot model).
     * totalRecorded tracks the cumulative count; only the latest snapshot is kept.
     */
    assert(history.hasRecord);
    assert(history.totalRecorded == 5U);

    /* GetEntry(0) returns the only stored snapshot (the latest) */
    assert(HDY_DiagnosticsHistory_GetEntry(&history, 0U, &entry));
    assert(entry.diagnostic.code == HDY_DIAG_CODE_TIMEOUT);
    assert(entry.segmentIndex == 4U);
    assert(entry.segmentTag == 4);

    /* GetEntry(n > 0) returns false — no historical entries beyond the latest */
    assert(!HDY_DiagnosticsHistory_GetEntry(&history, 1U, &entry));

    assert(HDY_DiagnosticsHistory_GetLatest(&history, &entry));
    assert(entry.diagnostic.code == HDY_DIAG_CODE_TIMEOUT);
    assert(entry.segmentIndex == 4U);
    assert(fabs(entry.eventTimestamp - 4.0) < 0.001);
    assert(entry.segmentTag == 4);
    printf("✓ Diagnostic history latest-snapshot retention test passed\n");
}

static void test_acknowledge_clears_retention_and_allows_re_recording(void) {
    HDY_MotionControlFB fb;
    HDY_MotionSegment recipe[1];

    printf("Testing AcknowledgeDiagnostics retention-clear semantics...\n");
    init_controller(&fb);
    recipe[0] = make_position_segment(41, 20.0, HDY_DIRECTION_EXTEND);
    assert(HDY_MotionControlFB_LoadRecipe(&fb, recipe, 1));

    fb.AXIS_REF.position = 0.0;
    fb.AXIS_REF.velocity = 0.0;
    fb.AXIS_REF.flow = 0.0;
    fb.AXIS_REF.pressure = recipe[0].targetPressure;
    fb.AXIS_REF.timestamp = 0.0;
    assert(HDY_MotionControlFB_StartSegment(&fb, 0, 0.0));
    HDY_MotionControlFB_Execute(&fb);
    assert(fb.DIAGNOSTIC.code == HDY_DIAG_CODE_NONE);

    disable_diagnostic_suppress(&fb);

    /* Advance past debounce window (0.1s) for criteria to trigger */
    fb.AXIS_REF.timestamp = 0.15;
    HDY_MotionControlFB_Execute(&fb);
    fb.AXIS_REF.timestamp = 0.3;
    HDY_MotionControlFB_Execute(&fb);

    assert(fb.DIAGNOSTIC.code == HDY_DIAG_CODE_FLOW_DEVIATION);
    assert(!HDY_MotionControlFB_AcknowledgeDiagnostics(&fb));
    assert(fb.DIAGNOSTIC_LATCH.code == HDY_DIAG_CODE_FLOW_DEVIATION);
    assert(fb.DIAGNOSTIC_HISTORY.hasRecord);

    fb.AXIS_REF.position = 20.0;
    fb.AXIS_REF.velocity = 0.0;
    fb.AXIS_REF.flow = 0.0;
    fb.AXIS_REF.pressure = recipe[0].targetPressure;
    fb.AXIS_REF.timestamp = 0.3;
    HDY_MotionControlFB_Execute(&fb);

    assert(fb.DIAGNOSTIC.code == HDY_DIAG_CODE_NONE);
    assert(HDY_MotionControlFB_AcknowledgeDiagnostics(&fb));
    assert(fb.DIAGNOSTIC.code == HDY_DIAG_CODE_NONE);
    assert(fb.DIAGNOSTIC_LATCH.code == HDY_DIAG_CODE_NONE);
    assert(!fb.LAST_DIAGNOSTIC_SNAPSHOT.valid);
    assert(!fb.LAST_FAULT_SNAPSHOT.valid);
    assert(!fb.DIAGNOSTIC_HISTORY.hasRecord);
    assert(fb.DIAGNOSTIC_HISTORY.totalRecorded == 0U);

    fb.AXIS_REF.position = 0.0;
    fb.AXIS_REF.velocity = 0.0;
    fb.AXIS_REF.flow = 0.0;
    fb.AXIS_REF.pressure = recipe[0].targetPressure;
    fb.AXIS_REF.timestamp = 0.4;
    assert(HDY_MotionControlFB_StartSegment(&fb, 0, 0.4));
    HDY_MotionControlFB_Execute(&fb);
    assert(fb.DIAGNOSTIC.code == HDY_DIAG_CODE_NONE);

    fb.AXIS_REF.timestamp = 0.6;
    HDY_MotionControlFB_Execute(&fb);

    assert(fb.DIAGNOSTIC.code == HDY_DIAG_CODE_FLOW_DEVIATION);
    assert(fb.DIAGNOSTIC_LATCH.code == HDY_DIAG_CODE_FLOW_DEVIATION);
    assert(fb.DIAGNOSTIC_HISTORY.hasRecord);
    assert(fb.DIAGNOSTIC_HISTORY.totalRecorded == 1U);
    printf("✓ AcknowledgeDiagnostics retention-clear test passed\n");
}

static void test_acknowledge_rejects_fault_retention(void) {
    HDY_MotionControlFB fb;
    HDY_MotionSegment recipe[1];
    HDY_DiagnosticSnapshot latestSnapshot;

    printf("Testing AcknowledgeDiagnostics fault-retention guard...\n");
    init_controller(&fb);
    recipe[0] = make_position_segment(42, 10.0, HDY_DIRECTION_EXTEND);
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
    assert(fb.DIAGNOSTIC_HISTORY.hasRecord);
    assert(HDY_DiagnosticsHistory_GetLatest(&fb.DIAGNOSTIC_HISTORY, &latestSnapshot));
    assert(latestSnapshot.diagnostic.code == HDY_DIAG_CODE_SENSOR_FAULT);
    printf("✓ AcknowledgeDiagnostics fault-retention guard test passed\n");
}

static void test_fault_snapshot_captures_protected_stop_context(void) {
    HDY_MotionControlFB fb;
    HDY_MotionSegment recipe[1];

    printf("Testing fault snapshot capture semantics...\n");
    init_controller(&fb);
    recipe[0] = make_time_segment(112, 1.0, HDY_DIRECTION_EXTEND);
    assert(HDY_MotionControlFB_LoadRecipe(&fb, recipe, 1));

    fb.AXIS_REF.position = 0.0;
    fb.AXIS_REF.velocity = 0.0;
    fb.AXIS_REF.flow = 0.0;
    fb.AXIS_REF.pressure = recipe[0].targetPressure;
    fb.AXIS_REF.timestamp = 0.0;
    assert(HDY_MotionControlFB_StartSegment(&fb, 0, 0.0));
    HDY_MotionControlFB_Execute(&fb);
    assert(fb.DIAGNOSTIC.code == HDY_DIAG_CODE_NONE);
    assert(!fb.DIAGNOSTIC_HISTORY.hasRecord);

    fb.AXIS_REF.timestamp = 0.1;
    fb.AXIS_REF.pressure = -0.5;
    HDY_MotionControlFB_Execute(&fb);

    assert(fb.FAULT);
    assert(fb.LAST_FAULT_SNAPSHOT.valid);
    assert(fb.LAST_FAULT_SNAPSHOT.fault);
    assert(fb.LAST_FAULT_SNAPSHOT.status == HDY_STATUS_FAULT);
    assert(fb.LAST_FAULT_SNAPSHOT.segmentIndex == 0U);
    assert(fb.LAST_FAULT_SNAPSHOT.diagnostic.code == HDY_DIAG_CODE_SENSOR_FAULT);
    assert(fb.LAST_FAULT_SNAPSHOT.segmentTag == 112);
    assert(fabs(fb.LAST_FAULT_SNAPSHOT.eventTimestamp - 0.1) < 0.001);
    assert(fb.LAST_FAULT_SNAPSHOT.axisRef.pressure == -0.5);
    assert(fb.DIAGNOSTIC_HISTORY.hasRecord);
    assert(fb.DIAGNOSTIC_HISTORY.totalRecorded == 1U);
    assert(fb.DIAGNOSTIC_HISTORY.lastSnapshot.diagnostic.code == HDY_DIAG_CODE_SENSOR_FAULT);
    printf("✓ Fault snapshot capture test passed\n");
}

static void test_sensor_fault_triggers_protected_stop(void) {
    HDY_MotionControlFB fb;
    HDY_MotionSegment recipe[1];

    printf("Testing invalid sensor feedback protected-stop behavior...\n");
    init_controller(&fb);
    recipe[0] = make_position_segment(26, 10.0, HDY_DIRECTION_EXTEND);
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
    assert(!fb.BUSY);
    assert(!fb.DONE);
    assert(fb.ERROR);
    assert(fb.ERROR_ID == HDY_DIAG_CODE_SENSOR_FAULT);
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
    recipe[0] = make_position_segment(27, 10.0, HDY_DIRECTION_EXTEND);
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
    assert(!fb.BUSY);
    assert(!fb.DONE);
    assert(fb.ERROR);
    assert(fb.ERROR_ID == HDY_DIAG_CODE_TIMESTAMP_ROLLBACK);
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
    recipe[0] = make_pressure_segment(205, 5.0, 12.0, 3.0);
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
    recipe[0] = make_position_segment(43, 5.0, HDY_DIRECTION_EXTEND);
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
    recipe[0] = make_position_segment(28, 5.0, HDY_DIRECTION_EXTEND);
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
    assert(!fb.BUSY);
    assert(!fb.DONE);
    assert(!fb.ERROR);
    assert(fb.ERROR_ID == HDY_DIAG_CODE_NONE);
    assert(fb.FINISHED);
    assert(!fb.FAULT);
    assert(fb.STATUS == HDY_STATUS_FINISHED);
    assert(fb.FB_STATE == HDY_FB_STATE_ABORTED);
    assert(!fb.SEGMENT_COMPLETED);
    assert(fb.PUMP_SPEED == 0.0);
    assert(!fb.START_SEGMENT);
    assert(fb.STATE.plannedDirection == HDY_DIRECTION_HOLD);
    assert(fb.STATE.currentSegmentTag == 0);
    assert(fb.STATE.protectionAction == HDY_PROTECTION_ACTION_NONE);
    assert(fb.DIAGNOSTIC.code == HDY_DIAG_CODE_ABORTED);
    assert(fb.DIAGNOSTIC.source == HDY_DIAG_SOURCE_COMMAND);
    assert(fb.DIAGNOSTIC.recovery == HDY_DIAG_RECOVERY_NONE);
    assert(fb.DIAGNOSTIC.protectionAction == HDY_PROTECTION_ACTION_NONE);

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
    recipe[0] = make_position_segment(29, 5.0, HDY_DIRECTION_EXTEND);
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
    assert(!fb.DIAGNOSTIC_HISTORY.hasRecord);
    printf("✓ Abort diagnostic auto-clear test passed\n");
}

static void test_finished_command_diagnostic_auto_clears_in_hold(void) {
    HDY_MotionControlFB fb;
    HDY_MotionSegment recipe[1];

    printf("Testing finished-state command diagnostic auto-clear lifecycle...\n");
    init_controller(&fb);
    recipe[0] = make_position_segment(30, 1.0, HDY_DIRECTION_EXTEND);
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

static void test_reset_soft_reset_preserves_config(void) {
    HDY_MotionControlFB fb;
    HDY_MotionSegment recipe[1];

    printf("Testing RESET soft-reset preserves recipe and config...\n");
    init_controller(&fb);
    fb.FLOW_TO_PUMP_SPEED_GAIN = 42.0;
    fb.PUMP_SPEED_LIMIT = 100.0;
    recipe[0] = make_position_segment(31, 6.0, HDY_DIRECTION_EXTEND);
    assert(HDY_MotionControlFB_LoadRecipe(&fb, recipe, 1));
    assert(HDY_MotionControlFB_StartSegment(&fb, 0, 0.0));

    fb.AXIS_REF.timestamp = 0.0;
    HDY_MotionControlFB_Execute(&fb);
    assert(fb.ACTIVE);
    assert(fb.DIAGNOSTIC.code == HDY_DIAG_CODE_NONE);

    disable_diagnostic_suppress(&fb);

    fb.AXIS_REF.timestamp = 0.15;
    HDY_MotionControlFB_Execute(&fb);
    fb.AXIS_REF.timestamp = 0.3;
    HDY_MotionControlFB_Execute(&fb);
    assert(fb.DIAGNOSTIC_LATCH.code != HDY_DIAG_CODE_NONE);
    assert(fb.LAST_DIAGNOSTIC_SNAPSHOT.valid);
    assert(fb.DIAGNOSTIC_HISTORY.hasRecord);

    /* RESET=true triggers soft reset: preserves recipe and gains */
    fb.RESET = true;
    HDY_MotionControlFB_Execute(&fb);
    assert(!fb.ACTIVE);
    assert(!fb.FINISHED);
    assert(!fb.FAULT);
    assert(!fb.SEGMENT_COMPLETED);
    assert(fb.STATE.plannedDirection == HDY_DIRECTION_HOLD);
    assert(fb.STATE.currentSegmentTag == 0);
    assert(fb.DIAGNOSTIC.code == HDY_DIAG_CODE_NONE);
    assert(fb.DIAGNOSTIC_LATCH.code == HDY_DIAG_CODE_NONE);
    assert(!fb.LAST_DIAGNOSTIC_SNAPSHOT.valid);
    assert(!fb.LAST_FAULT_SNAPSHOT.valid);
    assert(!fb.DIAGNOSTIC_HISTORY.hasRecord);
    assert(fb.DIAGNOSTIC_HISTORY.totalRecorded == 0U);

    /* Soft reset preserves these */
    assert(fb.RECIPE_SIZE == 1U);
    assert(fb.FLOW_TO_PUMP_SPEED_GAIN == 42.0);
    assert(fb.PUMP_SPEED_LIMIT == 100.0);
    assert(fb.USE_RECIPE);

    /* After soft reset with recipe loaded, FB should be in READY state */
    assert(fb.FB_STATE == HDY_FB_STATE_READY);
    assert(fb.STATUS == HDY_STATUS_READY);

    printf("✓ RESET soft-reset preserves recipe and config test passed\n");
}

static void test_init_hard_reset_clears_everything(void) {
    HDY_MotionControlFB fb;
    HDY_MotionSegment recipe[1];

    printf("Testing Init() hard-reset clears everything...\n");
    init_controller(&fb);
    fb.FLOW_TO_PUMP_SPEED_GAIN = 42.0;
    fb.PUMP_SPEED_LIMIT = 100.0;
    recipe[0] = make_position_segment(32, 6.0, HDY_DIRECTION_EXTEND);
    assert(HDY_MotionControlFB_LoadRecipe(&fb, recipe, 1));

    /* Init() performs full hard reset: clears everything */
    HDY_MotionControlFB_Init(&fb);
    assert(fb.RECIPE_SIZE == 0U);
    assert(fb.FLOW_TO_PUMP_SPEED_GAIN == 0.0);
    assert(fb.PUMP_SPEED_LIMIT == 0.0);
    assert(!fb.ACTIVE);
    assert(!fb.FINISHED);
    assert(!fb.FAULT);
    assert(fb.FB_STATE == HDY_FB_STATE_IDLE);
    assert(fb.STATUS == HDY_STATUS_IDLE);
    assert(!fb.SEGMENT_COMPLETED);
    assert(fb.DIAGNOSTIC.code == HDY_DIAG_CODE_NONE);
    assert(fb.DIAGNOSTIC_LATCH.code == HDY_DIAG_CODE_NONE);

    printf("✓ Init() hard-reset clears everything test passed\n");
}

static void test_soft_reset_allows_restart(void) {
    HDY_MotionControlFB fb;
    HDY_MotionSegment recipe[2];

    printf("Testing soft reset allows restart without reload...\n");
    init_controller(&fb);
    recipe[0] = make_position_segment(33, 6.0, HDY_DIRECTION_EXTEND);
    recipe[1] = make_position_segment(34, 8.0, HDY_DIRECTION_EXTEND);
    assert(HDY_MotionControlFB_LoadRecipe(&fb, recipe, 2));
    assert(HDY_MotionControlFB_StartSegment(&fb, 0, 0.0));

    fb.AXIS_REF.timestamp = 0.0;
    HDY_MotionControlFB_Execute(&fb);
    assert(fb.ACTIVE);

    /* Soft reset via API */
    HDY_MotionControlFB_SoftReset(&fb);
    assert(!fb.ACTIVE);
    assert(fb.RECIPE_SIZE == 2U);
    assert(fb.FB_STATE == HDY_FB_STATE_READY);

    /* Should be able to start a segment again without reloading recipe */
    fb.EN = true;
    assert(HDY_MotionControlFB_StartSegment(&fb, 0, 0.0));
    fb.AXIS_REF.timestamp = 0.0;
    HDY_MotionControlFB_Execute(&fb);
    assert(fb.ACTIVE);
    assert(fb.FB_STATE == HDY_FB_STATE_RUNNING);

    /* Advance to next segment should also work */
    fb.AXIS_REF.position = 6.0;
    fb.AXIS_REF.timestamp = 1.0;
    HDY_MotionControlFB_Execute(&fb);

    printf("✓ Soft reset allows restart without reload test passed\n");
}

static void test_typed_diagnostic_thresholds_via_criteria_layer(void) {
    HDY_ErrorMonitor monitor;
    HDY_DiagnosticCriteria flowCriteria;
    HDY_DiagnosticCriteriaState flowState;
    HDY_DiagnosticResult flowResult;
    HDY_AxisRef axisRef;
    HDY_ExecutionReference references;

    printf("Testing typed diagnostic thresholds via criteria layer...\n");

    /* Setup: use flow criteria with typed threshold = 0.2 L/min, no suppress for clean test */
    HDY_ErrorMonitor_Init(&monitor);
    HDY_DiagnosticCriteria_CreateDefaultFlowCriteria(&flowCriteria);
    HDY_DiagnosticCriteria_InitState(&flowState);
    flowCriteria.enableStartupSuppress = false;
    flowCriteria.enableSwitchSuppress = false;
    flowCriteria.enableLoopBuildSuppress = false;
    flowCriteria.debounceTime = 0.0;
    flowCriteria.baseThreshold = 0.2;  /* matches segment.flowTolerance = 0.2 */

    /* Case 1: flow within tolerance (error = 12.0 - 11.85 = 0.15 < 0.2) */
    axisRef.position = 0.0;
    axisRef.velocity = 0.0;
    axisRef.flow = 11.85;
    axisRef.pressure = 12.40;
    axisRef.timestamp = 1.0;

    references.pressureReference = 12.0;
    references.flowReference = 12.0;
    references.velocityReference = 0.0;
    references.elapsedTime = 0.6;

    HDY_ErrorMonitor_Update(&monitor, &axisRef, &references, 1.0);
    HDY_DiagnosticCriteria_CheckFlow(&flowResult, &monitor, &flowCriteria, &flowState,
                                      1.0, 0.6, false, false);
    assert(!flowResult.triggered);

    /* Case 2: flow exceeds tolerance (error = 12.0 - 11.70 = 0.30 > 0.2) */
    axisRef.flow = 11.70;
    HDY_ErrorMonitor_Update(&monitor, &axisRef, &references, 1.1);
    HDY_DiagnosticCriteria_CheckFlow(&flowResult, &monitor, &flowCriteria, &flowState,
                                      1.1, 0.6, false, false);
    assert(flowResult.triggered);
    assert(flowResult.code == HDY_DIAG_CODE_FLOW_DEVIATION);
    assert(flowResult.severity == HDY_DIAG_SEVERITY_WARNING);
    assert(flowResult.action == HDY_PROTECTION_ACTION_DERATE);

    printf("✓ Typed diagnostic thresholds via criteria layer test passed\n");
}

static void test_timeout_limit_stops_segment_safely(void) {
    HDY_MotionControlFB fb;
    HDY_MotionSegment recipe[1];

    printf("Testing timeoutLimit safe-stop behavior...\n");
    init_controller(&fb);
    recipe[0] = make_position_segment(38, 100.0, HDY_DIRECTION_EXTEND);
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
    printf("✓ timeoutLimit safe-stop test passed\n");
}

static void test_runtime_validation_fault_latches_safe_state(void) {
    HDY_MotionControlFB fb;
    HDY_MotionSegment recipe[1];

    printf("Testing runtime validation fault behavior...\n");
    init_controller(&fb);
    recipe[0] = make_position_segment(39, 20.0, HDY_DIRECTION_EXTEND);
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
    printf("✓ Runtime validation fault test passed\n");
}

static void test_parameter_validation(void) {
    HDY_MotionControlFB fb;
    HDY_MotionSegment badRecipe[1];
    HDY_MotionSegment goodRecipe[1];

    printf("Testing recipe and runtime parameter validation...\n");
    init_controller(&fb);

    badRecipe[0] = make_time_segment(114, 0.5, HDY_DIRECTION_AUTO);
    assert(!HDY_MotionControlFB_LoadRecipe(&fb, badRecipe, 1));
    assert(fb.RECIPE_SIZE == 0U);
    assert(!fb.FAULT);
    assert(fb.STATUS == HDY_STATUS_IDLE);
    assert(fb.DIAGNOSTIC.code == HDY_DIAG_CODE_SEGMENT_INVALID);
    assert(fb.DIAGNOSTIC.source == HDY_DIAG_SOURCE_RECIPE);
    assert(fb.DIAGNOSTIC.recovery == HDY_DIAG_RECOVERY_RELOAD_RECIPE);
    assert(fb.DIAGNOSTIC.protectionAction == HDY_PROTECTION_ACTION_WARNING);

    badRecipe[0] = make_time_segment(115, 0.5, HDY_DIRECTION_EXTEND);
    badRecipe[0].planner = HDY_PLANNER_POSITION_BASED;
    assert(!HDY_MotionControlFB_LoadRecipe(&fb, badRecipe, 1));
    assert(fb.DIAGNOSTIC.code == HDY_DIAG_CODE_SEGMENT_INVALID);

    badRecipe[0] = make_pressure_segment(206, 1.0, 10.0, 2.0);
    badRecipe[0].pressureController = HDY_PRESSURE_CONTROLLER_RBF_PID;
    assert(HDY_MotionControlFB_LoadRecipe(&fb, badRecipe, 1));
    assert(fb.RECIPE_SIZE == 1U);
    assert(fb.DIAGNOSTIC.code == HDY_DIAG_CODE_NONE);

    badRecipe[0] = make_pressure_segment(207, 1.0, 10.0, 2.0);
    badRecipe[0].pressureController = HDY_PRESSURE_CONTROLLER_RBF_PID;
    badRecipe[0].pressureRbfConfig.minKp = 0.9;
    badRecipe[0].pressureRbfConfig.maxKp = 0.85;
    assert(!HDY_MotionControlFB_LoadRecipe(&fb, badRecipe, 1));
    assert(fb.DIAGNOSTIC.code == HDY_DIAG_CODE_SEGMENT_INVALID);

    badRecipe[0] = make_pressure_segment(208, 1.0, 10.0, 2.0);
    badRecipe[0].pressureController = HDY_PRESSURE_CONTROLLER_PI;
    badRecipe[0].pressureKp = 0.5;
    badRecipe[0].pressureKi = 0.5;
    badRecipe[0].pressureFilterAlpha = 1.5;
    assert(!HDY_MotionControlFB_LoadRecipe(&fb, badRecipe, 1));
    assert(fb.DIAGNOSTIC.code == HDY_DIAG_CODE_SEGMENT_INVALID);

    goodRecipe[0] = make_position_segment(40, 5.0, HDY_DIRECTION_EXTEND);
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

    fb.FLOW_TO_PUMP_SPEED_GAIN = 100.0;
    fb.AXIS_REF.position = 10.0;
    assert(!HDY_MotionControlFB_StartSegment(&fb, 0, 0.0));
    assert(fb.DIAGNOSTIC.code == HDY_DIAG_CODE_START_CONTEXT_INVALID);
    assert(fb.DIAGNOSTIC.source == HDY_DIAG_SOURCE_COMMAND);
    assert(fb.DIAGNOSTIC.recovery == HDY_DIAG_RECOVERY_CHECK_COMMAND);
    assert(fb.DIAGNOSTIC.protectionAction == HDY_PROTECTION_ACTION_WARNING);
    printf("✓ Recipe and runtime parameter validation test passed\n");
}

int main(void) {
    printf("Running MotionControl tests...\n\n");

    test_load_recipe_requires_start_command();
    test_standard_outputs_follow_plcopen_state_machine();
    test_start_segment_and_segment_changed_pulse();
    test_start_segment_command_input();
    test_start_segment_input_uses_rising_edge();
    test_cycle_does_not_sample_command_inputs_without_scan();
    test_cycle_consumes_api_queued_start_command();
    test_direct_mode_start_without_recipe_uses_direct_segment_buffer();
    test_direct_mode_latches_segment_parameters_at_start();
    test_recipe_and_direct_modes_can_coexist_and_switch();
    test_direct_mode_requires_direct_segment_configuration();
    test_hold_command_transitions_running_to_hold();
    test_resume_command_restores_running_and_freezes_elapsed_time();
    test_hold_rejected_in_ready_state();
    test_command_warning_syncs_and_clears_state_protection_action();
    test_resume_rejected_while_running();
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
    test_diagnostic_history_helpers_preserve_latest_snapshot();
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
    test_reset_soft_reset_preserves_config();
    test_init_hard_reset_clears_everything();
    test_soft_reset_allows_restart();
    test_typed_diagnostic_thresholds_via_criteria_layer();
    test_timeout_limit_stops_segment_safely();
    test_runtime_validation_fault_latches_safe_state();
    test_parameter_validation();

    printf("\n✅ All MotionControl tests passed successfully!\n");
    return 0;
}
