#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "motion_control.h"

typedef struct {
    HDY_UINT segmentChangeCount;
    HDY_UINT segmentCompletedCount;
    HDY_REAL maxPumpSpeed;
    HDY_REAL maxPressure;
    HDY_REAL finalPosition;
    HDY_BOOL sawPressureClosedLoop;
    HDY_BOOL sawRetract;
    HDY_BOOL sawDegraded;
} ScenarioStats;

static void init_controller(HDY_MotionControlFB* fb) {
    HDY_MotionControlFB_Init(fb);
    fb->EN = true;
    fb->FLOW_TO_PUMP_SPEED_GAIN = 80.0;
    fb->PUMP_SPEED_LIMIT = 5000.0;
}

static void init_axis(HDY_AxisRef* axis, HDY_REAL position, HDY_REAL pressure) {
    memset(axis, 0, sizeof(*axis));
    axis->position = position;
    axis->pressure = pressure;
    axis->timestamp = 0.0;
}

static HDY_MotionSegment make_linear_segment(const char* name,
                                             HDY_SegmentType type,
                                             HDY_ControlMode mode,
                                             HDY_MotionDirection direction,
                                             HDY_REAL targetPosition,
                                             HDY_REAL targetFlow,
                                             HDY_REAL targetPressure,
                                             HDY_REAL maxAcceleration,
                                             HDY_REAL maxVelocity,
                                             HDY_REAL maxFlow,
                                             HDY_TIME timeoutLimit) {
    HDY_MotionSegment segment = {0};

    strncpy(segment.name, name, HDY_NAME_MAX - 1);
    segment.name[HDY_NAME_MAX - 1] = '\0';
    segment.type = type;
    segment.planner = (mode == HDY_MODE_POSITION)
        ? HDY_PLANNER_POSITION_BASED
        : HDY_PLANNER_TIME_BASED;
    segment.mode = mode;
    segment.endCondition = HDY_END_POSITION;
    segment.direction = direction;
    segment.targetPosition = targetPosition;
    segment.targetFlow = targetFlow;
    segment.targetPressure = targetPressure;
    segment.maxAcceleration = maxAcceleration;
    segment.maxVelocity = maxVelocity;
    segment.maxFlow = maxFlow;
    segment.duration = 0.0;
    segment.tolerance = 0.0;
    segment.positionTolerance = 0.20;
    segment.pressureTolerance = 1.00;
    segment.flowTolerance = maxFlow + 2.0;
    segment.velocityTolerance = maxVelocity + 5.0;
    segment.timeoutLimit = timeoutLimit;
    segment.velocityToFlowGain = 0.30;
    segment.pressureRampRate = 20.0;
    return segment;
}

static HDY_MotionSegment make_pressure_segment(const char* name,
                                               HDY_SegmentType type,
                                               HDY_EndConditionType endCondition,
                                               HDY_REAL targetPressure,
                                               HDY_REAL targetFlow,
                                               HDY_TIME duration,
                                               HDY_TIME timeoutLimit) {
    HDY_MotionSegment segment = {0};

    strncpy(segment.name, name, HDY_NAME_MAX - 1);
    segment.name[HDY_NAME_MAX - 1] = '\0';
    segment.type = type;
    segment.planner = HDY_PLANNER_TIME_BASED;
    segment.mode = HDY_MODE_PRESSURE_CLOSED_LOOP;
    segment.endCondition = endCondition;
    segment.direction = HDY_DIRECTION_HOLD;
    segment.targetPosition = 0.0;
    segment.targetFlow = targetFlow;
    segment.targetPressure = targetPressure;
    segment.maxAcceleration = 0.0;
    segment.maxVelocity = 0.0;
    segment.maxFlow = targetFlow + 8.0;
    segment.duration = duration;
    segment.tolerance = 0.0;
    segment.positionTolerance = 0.20;
    segment.pressureTolerance = 0.30;
    segment.flowTolerance = targetFlow + 10.0;
    segment.velocityTolerance = 5.0;
    segment.timeoutLimit = timeoutLimit;
    segment.velocityToFlowGain = 0.0;
    segment.pressureRampRate = 25.0;
    segment.pressureController = HDY_PRESSURE_CONTROLLER_PI;
    segment.pressureKp = 0.55;
    segment.pressureKi = 1.20;
    segment.pressureIntegralLimit = targetFlow + 5.0;
    segment.pressureDeadband = 0.05;
    segment.pressureFilterAlpha = 0.45;
    segment.pressureDerivativeFilterAlpha = 0.35;
    return segment;
}

static size_t build_full_cycle_recipe(HDY_MotionSegment* recipe) {
    size_t size = 0U;

    recipe[size++] = make_linear_segment("SlowClamp",
                                         HDY_SEGMENT_TYPE_CLAMPING,
                                         HDY_MODE_SPEED_RAMP,
                                         HDY_DIRECTION_EXTEND,
                                         8.0,
                                         5.0,
                                         4.0,
                                         120.0,
                                         30.0,
                                         10.0,
                                         4.0);
    recipe[size++] = make_linear_segment("FastClamp",
                                         HDY_SEGMENT_TYPE_CLAMPING,
                                         HDY_MODE_SPEED_RAMP,
                                         HDY_DIRECTION_EXTEND,
                                         25.0,
                                         12.0,
                                         6.0,
                                         220.0,
                                         70.0,
                                         18.0,
                                         4.0);
    recipe[size++] = make_pressure_segment("LowPressureProtect",
                                           HDY_SEGMENT_TYPE_CLAMPING,
                                           HDY_END_TIME,
                                           8.0,
                                           3.0,
                                           0.30,
                                           2.0);
    recipe[size++] = make_pressure_segment("HighPressureLock",
                                           HDY_SEGMENT_TYPE_CLAMPING,
                                           HDY_END_PRESSURE,
                                           14.0,
                                           5.0,
                                           0.0,
                                           2.0);
    recipe[size++] = make_linear_segment("InjectionStage1",
                                         HDY_SEGMENT_TYPE_INJECTION,
                                         HDY_MODE_SPEED_RAMP,
                                         HDY_DIRECTION_EXTEND,
                                         45.0,
                                         20.0,
                                         5.0,
                                         260.0,
                                         90.0,
                                         26.0,
                                         4.0);
    recipe[size++] = make_linear_segment("InjectionStage2",
                                         HDY_SEGMENT_TYPE_INJECTION,
                                         HDY_MODE_SPEED_RAMP,
                                         HDY_DIRECTION_EXTEND,
                                         70.0,
                                         12.0,
                                         6.0,
                                         200.0,
                                         60.0,
                                         18.0,
                                         4.0);
    recipe[size++] = make_pressure_segment("HoldStage1",
                                           HDY_SEGMENT_TYPE_HOLDING,
                                           HDY_END_TIME,
                                           12.0,
                                           4.0,
                                           0.30,
                                           2.0);
    recipe[size++] = make_pressure_segment("HoldStage2",
                                           HDY_SEGMENT_TYPE_HOLDING,
                                           HDY_END_TIME,
                                           10.0,
                                           3.0,
                                           0.40,
                                           2.0);
    recipe[size++] = make_linear_segment("SuckBack",
                                         HDY_SEGMENT_TYPE_INJECTION,
                                         HDY_MODE_SPEED_RAMP,
                                         HDY_DIRECTION_RETRACT,
                                         66.0,
                                         8.0,
                                         4.0,
                                         180.0,
                                         40.0,
                                         12.0,
                                         4.0);
    recipe[size++] = make_linear_segment("MoldOpen",
                                         HDY_SEGMENT_TYPE_OPENING,
                                         HDY_MODE_SPEED_RAMP,
                                         HDY_DIRECTION_RETRACT,
                                         12.0,
                                         18.0,
                                         4.0,
                                         260.0,
                                         80.0,
                                         24.0,
                                         5.0);
    recipe[size++] = make_linear_segment("EjectorForward",
                                         HDY_SEGMENT_TYPE_EJECTION,
                                         HDY_MODE_POSITION,
                                         HDY_DIRECTION_EXTEND,
                                         16.0,
                                         6.0,
                                         3.0,
                                         140.0,
                                         25.0,
                                         10.0,
                                         4.0);
    recipe[size++] = make_linear_segment("CorePullBack",
                                         HDY_SEGMENT_TYPE_CORE_PULL,
                                         HDY_MODE_POSITION,
                                         HDY_DIRECTION_RETRACT,
                                         6.0,
                                         6.0,
                                         3.0,
                                         140.0,
                                         25.0,
                                         10.0,
                                         4.0);
    return size;
}

static size_t build_boundary_recipe(HDY_MotionSegment* recipe) {
    static const HDY_REAL targetPositions[8] = {2.0, 6.0, 3.0, 7.0, 4.0, 8.0, 5.0, 9.0};
    static const HDY_MotionDirection directions[8] = {
        HDY_DIRECTION_EXTEND,
        HDY_DIRECTION_EXTEND,
        HDY_DIRECTION_RETRACT,
        HDY_DIRECTION_EXTEND,
        HDY_DIRECTION_RETRACT,
        HDY_DIRECTION_EXTEND,
        HDY_DIRECTION_RETRACT,
        HDY_DIRECTION_EXTEND
    };
    size_t pairIndex;
    size_t size = 0U;
    char name[HDY_NAME_MAX];

    for (pairIndex = 0U; pairIndex < 8U; ++pairIndex) {
        snprintf(name, sizeof(name), "BoundaryMotion%02u", (unsigned int)(pairIndex + 1U));
        recipe[size++] = make_linear_segment(name,
                                             (pairIndex % 2U == 0U)
                                                 ? HDY_SEGMENT_TYPE_CLAMPING
                                                 : HDY_SEGMENT_TYPE_OPENING,
                                             HDY_MODE_SPEED_RAMP,
                                             directions[pairIndex],
                                             targetPositions[pairIndex],
                                             5.0 + (HDY_REAL)pairIndex,
                                             4.0,
                                             180.0,
                                             30.0,
                                             12.0,
                                             2.0);

        snprintf(name, sizeof(name), "BoundaryHold%02u", (unsigned int)(pairIndex + 1U));
        recipe[size++] = make_pressure_segment(name,
                                               HDY_SEGMENT_TYPE_HOLDING,
                                               HDY_END_TIME,
                                               6.0 + 0.5 * (HDY_REAL)pairIndex,
                                               2.0 + 0.2 * (HDY_REAL)pairIndex,
                                               0.05,
                                               0.50);
    }

    return size;
}

static void simulate_plant_response(HDY_AxisRef* axis,
                                    const HDY_MotionControlFB* fb,
                                    HDY_REAL dt) {
    const HDY_MotionSegment* segment;
    HDY_REAL targetPressure;

    if (axis == NULL || fb == NULL) {
        return;
    }

    if (!fb->ACTIVE || fb->STATE.currentSegmentIndex >= fb->RECIPE_SIZE) {
        axis->velocity = 0.0;
        axis->flow = 0.0;
        return;
    }

    segment = &fb->RECIPE[fb->STATE.currentSegmentIndex];
    axis->flow = fb->STATE.plannedFlow;

    if (segment->mode == HDY_MODE_PRESSURE_CLOSED_LOOP) {
        targetPressure = fb->STATE.references.pressureReference;
        axis->velocity = 0.0;
        axis->pressure += (targetPressure - axis->pressure) * 0.65;
        if (fabs(axis->pressure - targetPressure) <= segment->pressureTolerance) {
            axis->pressure = targetPressure;
        }
        return;
    }

    axis->velocity = fb->STATE.plannedVelocity;
    axis->position += axis->velocity * dt;
    if (segment->direction == HDY_DIRECTION_EXTEND && axis->position > segment->targetPosition) {
        axis->position = segment->targetPosition;
    }
    if (segment->direction == HDY_DIRECTION_RETRACT && axis->position < segment->targetPosition) {
        axis->position = segment->targetPosition;
    }

    targetPressure = segment->targetPressure;
    axis->pressure += (targetPressure - axis->pressure) * 0.30;
    if (fabs(axis->pressure - targetPressure) <= segment->pressureTolerance) {
        axis->pressure = targetPressure;
    }
}

static ScenarioStats run_recipe_and_collect(const HDY_MotionSegment* recipe,
                                            size_t recipeSize,
                                            HDY_REAL initialPosition,
                                            HDY_REAL initialPressure,
                                            HDY_REAL dt,
                                            size_t maxSteps) {
    HDY_MotionControlFB fb;
    HDY_AxisRef axis;
    ScenarioStats stats;
    size_t step;
    size_t expectedSegmentStart = 0U;
    size_t expectedSegmentComplete = 0U;
    size_t lastObservedSegmentIndex = recipeSize;

    memset(&stats, 0, sizeof(stats));
    init_controller(&fb);
    init_axis(&axis, initialPosition, initialPressure);

    assert(HDY_MotionControlFB_LoadRecipe(&fb, recipe, recipeSize));
    fb.AXIS_REF = axis;
    assert(HDY_MotionControlFB_StartSegment(&fb, 0U, axis.timestamp));

    for (step = 0U; step < maxSteps; ++step) {
        HDY_BOOL completedThisCycle;
        size_t completedIndex;
        const HDY_MotionSegment* activeSegment;

        fb.AXIS_REF = axis;
        HDY_MotionControlFB_Execute(&fb);

        assert(!fb.FAULT);
        assert(fb.DIAGNOSTIC.code != HDY_DIAG_CODE_TIMEOUT);
        stats.maxPumpSpeed = (fb.PUMP_SPEED > stats.maxPumpSpeed) ? fb.PUMP_SPEED : stats.maxPumpSpeed;
        stats.maxPressure = (axis.pressure > stats.maxPressure) ? axis.pressure : stats.maxPressure;
        if (fb.STATUS == HDY_STATUS_DEGRADED) {
            stats.sawDegraded = true;
        }

        if (fb.STATE.currentSegmentIndex < recipeSize &&
            fb.STATE.currentSegmentIndex != lastObservedSegmentIndex &&
            (fb.ACTIVE || fb.SEGMENT_COMPLETED || fb.FINISHED)) {
            assert(expectedSegmentStart < recipeSize);
            assert(fb.STATE.currentSegmentIndex == expectedSegmentStart);
            assert(strcmp(fb.STATE.currentSegmentName, recipe[expectedSegmentStart].name) == 0);
            lastObservedSegmentIndex = fb.STATE.currentSegmentIndex;
            ++stats.segmentChangeCount;
            ++expectedSegmentStart;
        }

        if (fb.SEGMENT_CHANGED) {
            assert(fb.STATE.currentSegmentIndex < recipeSize);
            assert(fb.STATE.currentSegmentIndex == lastObservedSegmentIndex);
        }

        if (fb.ACTIVE) {
            assert(fb.STATE.currentSegmentIndex < recipeSize);
            assert(fb.STATE.currentSegmentName[0] != '\0');
            activeSegment = &recipe[fb.STATE.currentSegmentIndex];
            if (activeSegment->mode == HDY_MODE_PRESSURE_CLOSED_LOOP) {
                assert(fb.STATE.pressureControllerApplied == HDY_PRESSURE_CONTROLLER_PI);
                stats.sawPressureClosedLoop = true;
            } else {
                assert(fb.STATE.pressureControllerApplied == HDY_PRESSURE_CONTROLLER_NONE);
            }

            if (fb.STATE.plannedDirection == HDY_DIRECTION_RETRACT) {
                stats.sawRetract = true;
            }
        }

        completedThisCycle = fb.SEGMENT_COMPLETED;
        completedIndex = fb.STATE.currentSegmentIndex;
        if (completedThisCycle) {
            assert(expectedSegmentComplete < recipeSize);
            assert(completedIndex == expectedSegmentComplete);
            assert(strcmp(fb.STATE.currentSegmentName, recipe[expectedSegmentComplete].name) == 0);
            ++stats.segmentCompletedCount;
            ++expectedSegmentComplete;

            if (!fb.FINISHED) {
                assert(HDY_MotionControlFB_NextSegment(&fb, axis.timestamp));
            }

            axis.velocity = 0.0;
            axis.flow = 0.0;
            axis.timestamp += dt;
            if (fb.FINISHED) {
                break;
            }
            continue;
        }

        simulate_plant_response(&axis, &fb, dt);
        axis.timestamp += dt;
    }

    assert(!fb.FAULT);
    assert(fb.FINISHED);
    assert(fb.SEGMENT_COMPLETED);
    assert(expectedSegmentStart == recipeSize);
    assert(expectedSegmentComplete == recipeSize);
    stats.finalPosition = axis.position;
    return stats;
}

static void test_full_cycle_scenario_matrix(void) {
    HDY_MotionSegment recipe[HDY_MAX_SEGMENTS];
    ScenarioStats stats;
    size_t recipeSize;

    printf("Testing full injection-machine scenario matrix...\n");
    recipeSize = build_full_cycle_recipe(recipe);
    assert(recipeSize == 12U);

    stats = run_recipe_and_collect(recipe, recipeSize, 0.0, 2.0, 0.05, 1000U);

    assert(stats.segmentChangeCount == recipeSize);
    assert(stats.segmentCompletedCount == recipeSize);
    assert(stats.maxPumpSpeed > 0.0);
    assert(stats.maxPressure >= 10.0);
    assert(stats.sawPressureClosedLoop);
    assert(stats.sawRetract);
    assert(fabs(stats.finalPosition - 6.0) <= 0.20);
    printf("✓ Full scenario matrix test passed\n");
}

static void test_long_run_regression_cycles(void) {
    HDY_MotionSegment recipe[HDY_MAX_SEGMENTS];
    ScenarioStats stats;
    size_t recipeSize;
    size_t cycle;
    HDY_UINT totalTransitions = 0U;
    HDY_UINT totalCompletions = 0U;

    printf("Testing long-run regression over repeated full cycles...\n");
    recipeSize = build_full_cycle_recipe(recipe);

    for (cycle = 0U; cycle < 12U; ++cycle) {
        stats = run_recipe_and_collect(recipe, recipeSize, 0.0, 2.0, 0.05, 1000U);
        totalTransitions += stats.segmentChangeCount;
        totalCompletions += stats.segmentCompletedCount;
    }

    assert(totalTransitions == (HDY_UINT)(12U * recipeSize));
    assert(totalCompletions == (HDY_UINT)(12U * recipeSize));
    printf("✓ Long-run repeated-cycle regression test passed\n");
}

static void test_max_segment_boundary_recipe(void) {
    HDY_MotionSegment recipe[HDY_MAX_SEGMENTS];
    ScenarioStats stats;
    size_t recipeSize;

    printf("Testing max-segment boundary recipe execution...\n");
    recipeSize = build_boundary_recipe(recipe);
    assert(recipeSize == HDY_MAX_SEGMENTS);

    stats = run_recipe_and_collect(recipe, recipeSize, 0.0, 2.0, 0.05, 1200U);

    assert(stats.segmentChangeCount == HDY_MAX_SEGMENTS);
    assert(stats.segmentCompletedCount == HDY_MAX_SEGMENTS);
    assert(stats.maxPumpSpeed > 0.0);
    assert(stats.sawPressureClosedLoop);
    assert(stats.sawRetract);
    printf("✓ Max-segment boundary recipe test passed\n");
}

int main(void) {
    printf("Running scenario matrix tests...\n\n");

    test_full_cycle_scenario_matrix();
    test_long_run_regression_cycles();
    test_max_segment_boundary_recipe();

    printf("\n✅ All scenario matrix tests passed successfully!\n");
    return 0;
}
