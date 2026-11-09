#include <math.h>
#include <stdio.h>
#include <string.h>
#include "motion_planner.h"

static int failures;
static void check(int condition, const char *message)
{
    if (!condition) {
        ++failures;
        printf("FAIL: %s\n", message);
    }
}

static void test_speed_target_reduction_rebases_from_planned_velocity(void)
{
    HYD_AxisRef axis = {0};
    HYD_MotionSegment segment = {0};
    HYD_MotionPlannerState state = {0};
    HYD_MotionPlannerInput input = {0};
    HYD_MotionPlannerOutput output = {0};
    HYD_REAL previousPlanned;
    HYD_REAL expected;

    segment.mode = HYD_MODE_SPEED_RAMP;
    segment.planner = HYD_PLANNER_TIME_BASED;
    segment.endCondition = HYD_END_MANUAL;
    segment.direction = HYD_DIRECTION_POSITIVE;
    segment.maxVelocity = 40.0;
    segment.maxAcceleration = 100.0;
    segment.maxDeceleration = 100.0;
    segment.maxFlow = 100.0;
    segment.velocityToFlowGain = 1.0;

    input.axisRef = &axis;
    input.segment = &segment;
    input.state = &state;
    input.deltaTime = 0.001;

    for (int cycle = 0; cycle < 40; ++cycle) {
        input.elapsedTime = (cycle + 1) * 0.001;
        HYD_MotionPlanner_Execute(&input, &output);
    }

    previousPlanned = output.targetVelocity;
    axis.velocity = 1.0;
    segment.maxVelocity = 10.0;
    input.elapsedTime = 0.041;
    HYD_MotionPlanner_Execute(&input, &output);
    expected = previousPlanned + (10.0 - previousPlanned) * 0.001 / 0.1;
    check(fabs(output.targetVelocity - expected) < 0.01,
          "target reduction should rebase from the last planned velocity");

    for (int cycle = 1; cycle < 100; ++cycle) {
        input.elapsedTime = (41 + cycle) * 0.001;
        HYD_MotionPlanner_Execute(&input, &output);
    }
    check(fabs(output.targetVelocity - 10.0) < 0.01,
          "lower target should be reached within its 100 ms transition");
}

static void test_reverse_keeps_old_direction_until_zero(void)
{
    HYD_AxisRef axis = {0};
    HYD_MotionSegment segment = {0};
    HYD_MotionPlannerState state = {0};
    HYD_MotionPlannerInput input = {0};
    HYD_MotionPlannerOutput output = {0};
    int keptOldDirection = 1;

    segment.mode = HYD_MODE_POSITION;
    segment.planner = HYD_PLANNER_POSITION_BASED;
    segment.endCondition = HYD_END_POSITION;
    segment.direction = HYD_DIRECTION_RETRACT;
    segment.targetPosition = 0.0;
    segment.maxVelocity = 20.0;
    segment.maxAcceleration = 100.0;
    segment.maxDeceleration = 100.0;
    segment.maxFlow = 100.0;
    segment.velocityToFlowGain = 1.0;

    axis.position = 100.0;
    state.initialized = true;
    state.lastTargetVelocity = 5.0;

    input.axisRef = &axis;
    input.segment = &segment;
    input.state = &state;

    for (int cycle = 0; cycle < 99; ++cycle) {
        input.deltaTime = 0.001;
        HYD_MotionPlanner_Execute(&input, &output);
        keptOldDirection = keptOldDirection &&
            output.direction == HYD_DIRECTION_EXTEND && output.targetVelocity > 0.0;
    }
    check(keptOldDirection,
          "reverse request should keep the old direction while slowing");

    input.deltaTime = 0.001;
    HYD_MotionPlanner_Execute(&input, &output);
    check(output.direction == HYD_DIRECTION_EXTEND &&
              fabs(output.targetVelocity) < 0.01,
          "zero-speed cycle should still publish the old direction");

    input.deltaTime = 0.001;
    HYD_MotionPlanner_Execute(&input, &output);
    check(output.direction == HYD_DIRECTION_RETRACT &&
              output.targetVelocity < 0.0,
          "new direction should start after the zero-speed cycle");
}

static void test_position_safety_envelope_remains_a_hard_ceiling(void)
{
    HYD_AxisRef axis = {0};
    HYD_MotionSegment segment = {0};
    HYD_MotionPlannerState state = {0};
    HYD_MotionPlannerInput input = {0};
    HYD_MotionPlannerOutput output = {0};
    HYD_REAL brakingAcceleration;
    HYD_REAL expectedSafety;

    segment.mode = HYD_MODE_POSITION;
    segment.planner = HYD_PLANNER_TIME_BASED;
    segment.endCondition = HYD_END_POSITION;
    segment.direction = HYD_DIRECTION_EXTEND;
    segment.targetPosition = 100.0;
    segment.maxVelocity = 20.0;
    segment.maxAcceleration = 100.0;
    segment.maxDeceleration = 100.0;
    segment.maxFlow = 100.0;
    segment.velocityToFlowGain = 1.0;

    axis.position = 99.95;
    axis.velocity = 0.0;
    state.initialized = true;
    state.lastTargetVelocity = 20.0;

    input.axisRef = &axis;
    input.segment = &segment;
    input.state = &state;
    input.deltaTime = 0.001;
    input.elapsedTime = 0.001;
    HYD_MotionPlanner_Execute(&input, &output);

    brakingAcceleration = 20.0 / 0.1;
    expectedSafety = sqrt(pow(brakingAcceleration * 0.001, 2.0) +
                          2.0 * brakingAcceleration * 0.05) -
                      brakingAcceleration * 0.001;
    check(fabs(output.targetVelocity) <= expectedSafety + 0.001,
          "planned velocity must remain below the delay-aware position safety envelope");
}

static void test_position_terminated_speed_ramp_keeps_distance_safety_cap(void)
{
    HYD_AxisRef axis = {0};
    HYD_MotionSegment segment = {0};
    HYD_MotionPlannerState state = {0};
    HYD_MotionPlannerInput input = {0};
    HYD_MotionPlannerOutput output = {0};
    HYD_REAL expectedSafety;

    segment.mode = HYD_MODE_SPEED_RAMP;
    segment.planner = HYD_PLANNER_TIME_BASED;
    segment.endCondition = HYD_END_POSITION;
    segment.direction = HYD_DIRECTION_EXTEND;
    segment.targetPosition = 100.0;
    segment.maxVelocity = 20.0;
    segment.maxAcceleration = 100.0;
    segment.maxDeceleration = 100.0;
    segment.maxFlow = 100.0;
    segment.velocityToFlowGain = 1.0;

    axis.position = 99.95;
    state.initialized = true;
    state.lastTargetVelocity = 20.0;
    input.axisRef = &axis;
    input.segment = &segment;
    input.state = &state;
    input.deltaTime = 0.001;

    HYD_MotionPlanner_Execute(&input, &output);

    expectedSafety = sqrt(2.0 * (20.0 / 0.1) * 0.05);
    check(fabs(output.targetVelocity) <= expectedSafety + 0.001,
          "position-terminated speed ramp should keep the distance-based safety cap");
    check(fabs(output.targetVelocity - expectedSafety) < 0.001,
          "position safety cap should retain its calculated nonzero velocity");
    check(fabs(output.targetFlow - expectedSafety) < 0.001,
          "position-capped speed ramp should recompute flow from planned velocity");
}

static void test_speed_ramp_preserves_elapsed_deceleration_cap(void)
{
    HYD_AxisRef axis = {0};
    HYD_MotionSegment segment = {0};
    HYD_MotionPlannerState state = {0};
    HYD_MotionPlannerInput input = {0};
    HYD_MotionPlannerOutput output = {0};

    segment.mode = HYD_MODE_SPEED_RAMP;
    segment.planner = HYD_PLANNER_TIME_BASED;
    segment.endCondition = HYD_END_MANUAL;
    segment.direction = HYD_DIRECTION_POSITIVE;
    segment.maxVelocity = 20.0;
    segment.maxAcceleration = 100.0;
    segment.maxDeceleration = 100.0;
    segment.maxFlow = 100.0;
    segment.velocityToFlowGain = 1.0;

    state.initialized = true;
    state.lastTargetVelocity = 20.0;
    input.axisRef = &axis;
    input.segment = &segment;
    input.state = &state;
    input.deltaTime = 0.001;
    input.decelStartVel = 20.0;
    input.decelElapsed = 0.05;

    HYD_MotionPlanner_Execute(&input, &output);
    check(output.targetVelocity <= 10.0 + 0.001,
          "elapsed speed-ramp deceleration should cap velocity at 10 mm/s");
    check(fabs(output.targetFlow - 10.0) < 0.001,
          "speed-ramp flow should be recomputed from the deceleration cap");
}

static void test_blend_switch_safety_caps_velocity_at_terminal(void)
{
    HYD_AxisRef axis = {0};
    HYD_MotionSegment segment = {0};
    HYD_MotionPlannerState state = {0};
    HYD_MotionPlannerInput input = {0};
    HYD_MotionPlannerOutput output = {0};
    HYD_MotionBlendContext blend = {0};

    segment.mode = HYD_MODE_POSITION;
    segment.planner = HYD_PLANNER_POSITION_BASED;
    segment.endCondition = HYD_END_POSITION;
    segment.direction = HYD_DIRECTION_EXTEND;
    segment.targetPosition = 200.0;
    segment.maxVelocity = 20.0;
    segment.maxAcceleration = 100.0;
    segment.maxDeceleration = 100.0;
    segment.maxFlow = 100.0;
    segment.velocityToFlowGain = 1.0;

    axis.position = 100.0;
    state.initialized = true;
    state.lastTargetVelocity = 20.0;
    blend.active = true;
    blend.bufferMode = HYD_BUFFER_MODE_BLENDING_LOW;
    blend.blendVelocity = 5.0;
    blend.switchPosition = 100.0;
    blend.switchTolerance = 0.1;

    input.axisRef = &axis;
    input.segment = &segment;
    input.state = &state;
    input.blend = &blend;
    input.deltaTime = 0.001;

    HYD_MotionPlanner_Execute(&input, &output);
    check(output.targetVelocity <= blend.blendVelocity + 0.001,
          "blend switch safety ceiling should not exceed terminal velocity");
}

static void test_blend_terminal_ramp_continues_from_zero_planned_velocity(void)
{
    HYD_AxisRef axis = {0};
    HYD_MotionSegment segment = {0};
    HYD_MotionPlannerState state = {0};
    HYD_MotionPlannerInput input = {0};
    HYD_MotionPlannerOutput output = {0};
    HYD_MotionBlendContext blend = {0};

    segment.mode = HYD_MODE_POSITION;
    segment.planner = HYD_PLANNER_POSITION_BASED;
    segment.endCondition = HYD_END_POSITION;
    segment.direction = HYD_DIRECTION_EXTEND;
    segment.targetPosition = 100.0;
    segment.maxVelocity = 20.0;
    segment.maxAcceleration = 100.0;
    segment.maxDeceleration = 100.0;
    segment.maxFlow = 100.0;
    segment.velocityToFlowGain = 1.0;

    axis.position = 100.0;
    state.initialized = true;
    blend.active = true;
    blend.bufferMode = HYD_BUFFER_MODE_BLENDING_LOW;
    blend.blendVelocity = 5.0;
    blend.switchPosition = 100.0;
    blend.switchTolerance = 0.1;
    input.axisRef = &axis;
    input.segment = &segment;
    input.state = &state;
    input.blend = &blend;
    input.deltaTime = 0.001;

    HYD_MotionPlanner_Execute(&input, &output);
    check(fabs(output.targetVelocity - 0.05) < 0.001,
          "blend terminal speed should ramp from zero planned velocity");
}

static void test_small_target_and_time_changes_keep_cached_rate(void)
{
    HYD_AxisRef axis = {0};
    HYD_MotionSegment segment = {0};
    HYD_MotionPlannerState state = {0};
    HYD_MotionPlannerInput input = {0};
    HYD_MotionPlannerOutput output = {0};

    segment.mode = HYD_MODE_SPEED_RAMP;
    segment.planner = HYD_PLANNER_TIME_BASED;
    segment.endCondition = HYD_END_MANUAL;
    segment.direction = HYD_DIRECTION_POSITIVE;
    segment.maxVelocity = 20.0 + HYD_REAL_EPSILON * 100.0;
    segment.maxAcceleration = 100.0 + HYD_REAL_EPSILON * 1000.0;
    segment.maxDeceleration = 100.0;
    segment.maxFlow = 100.0;
    segment.velocityToFlowGain = 1.0;

    state.initialized = true;
    state.lastTargetVelocity = 1.0;
    state.transitionTargetVelocity = 20.0;
    state.transitionRate = 2.0;
    state.transitionTimeMs = 100.0;
    input.axisRef = &axis;
    input.segment = &segment;
    input.state = &state;
    input.deltaTime = 0.001;

    HYD_MotionPlanner_Execute(&input, &output);
    check(fabs(output.targetVelocity - 1.002) < 0.0001,
          "small target and transition-time noise should retain the cached rate");
}

int main(void)
{
    HYD_AxisRef axis = {0};
    HYD_MotionSegment segment = {0};
    HYD_MotionPlannerState state = {0};
    HYD_MotionPlannerInput input = {0};
    HYD_MotionPlannerOutput output = {0};
    HYD_REAL previous = 0.0;

    segment.mode = HYD_MODE_SPEED_RAMP;
    segment.planner = HYD_PLANNER_TIME_BASED;
    segment.endCondition = HYD_END_MANUAL;
    segment.direction = HYD_DIRECTION_POSITIVE;
    segment.maxVelocity = 20.0;
    segment.maxAcceleration = 100.0;
    segment.maxDeceleration = 100.0;
    segment.maxFlow = 100.0;
    segment.velocityToFlowGain = 1.0;

    input.axisRef = &axis;
    input.segment = &segment;
    input.state = &state;
    input.deltaTime = 0.001;

    for (int cycle = 0; cycle < 100; ++cycle) {
        input.elapsedTime = (cycle + 1) * 0.001;
        HYD_MotionPlanner_Execute(&input, &output);
        check(output.targetVelocity >= previous - 1e-6,
              "speed ramp should not reverse or jump backward while accelerating");
        check(output.targetVelocity - previous <= 0.21,
              "100 ms acceleration time should bound each 1 ms velocity step");
        previous = output.targetVelocity;
    }
    check(fabs(output.targetVelocity - 20.0) < 0.01,
          "100 ms transition should reach the target within one-cycle tolerance");

    segment.maxVelocity = 10.0;
    input.elapsedTime = 0.101;
    HYD_MotionPlanner_Execute(&input, &output);
    check(output.targetVelocity <= previous + 1e-6,
          "online target reduction should not jump above the previous output");
    check(output.targetVelocity >= 9.7,
          "online target reduction should smoothly approach the new target");

    test_speed_target_reduction_rebases_from_planned_velocity();
    test_reverse_keeps_old_direction_until_zero();
    test_position_safety_envelope_remains_a_hard_ceiling();
    test_position_terminated_speed_ramp_keeps_distance_safety_cap();
    test_speed_ramp_preserves_elapsed_deceleration_cap();
    test_blend_switch_safety_caps_velocity_at_terminal();
    test_blend_terminal_ramp_continues_from_zero_planned_velocity();
    test_small_target_and_time_changes_keep_cached_rate();

    printf("motion planner time ramp: %s\n", failures == 0 ? "PASS" : "FAIL");
    return failures == 0 ? 0 : 1;
}
