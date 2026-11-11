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

static void test_position_safety_uses_planned_velocity_for_delta(void)
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
    axis.velocity = 20.0;
    state.initialized = true;
    state.lastTargetVelocity = 10.0;

    input.axisRef = &axis;
    input.segment = &segment;
    input.state = &state;
    input.deltaTime = 0.001;
    input.elapsedTime = 0.001;

    HYD_MotionPlanner_Execute(&input, &output);

    brakingAcceleration = 10.0 / 0.1;
    expectedSafety = sqrt(pow(brakingAcceleration * input.deltaTime, 2.0) +
                          2.0 * brakingAcceleration * 0.05) -
                    brakingAcceleration * input.deltaTime;
    check(fabs(output.targetVelocity - expectedSafety) < 0.001,
          "position safety envelope should derive delta from planned velocity, not feedback velocity");
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

static void run_entry_overspeed_case(HYD_REAL maxVelocity,
                                     HYD_REAL entryVelocity,
                                     HYD_REAL expectedMidpoint)
{
    HYD_AxisRef axis = {0};
    HYD_MotionSegment segment = {0};
    HYD_MotionPlannerState state = {0};
    HYD_MotionPlannerInput input = {0};
    HYD_MotionPlannerOutput output = {0};
    int cycle;

    segment.mode = HYD_MODE_POSITION;
    segment.planner = HYD_PLANNER_POSITION_BASED;
    segment.endCondition = HYD_END_POSITION;
    segment.direction = HYD_DIRECTION_EXTEND;
    segment.targetPosition = 500.0;   /* 足够远，包络 sqrt 距离界不参与 */
    segment.maxVelocity = maxVelocity;
    segment.maxAcceleration = 100.0;  /* ms */
    segment.maxDeceleration = 100.0;  /* ms */
    segment.maxFlow = 1000.0;
    segment.velocityToFlowGain = 1.0;

    axis.position = 0.0;
    axis.velocity = entryVelocity;
    state.initialized = true;
    state.lastTargetVelocity = entryVelocity;  /* 上一段遗留速度 */

    input.axisRef = &axis;
    input.segment = &segment;
    input.state = &state;
    input.deltaTime = 0.001;

    for (cycle = 0; cycle < 102; ++cycle) {
        input.elapsedTime = (cycle + 1) * 0.001;
        HYD_MotionPlanner_Execute(&input, &output);
        axis.position += output.targetVelocity * 0.001; /* 运动学反馈 */
        axis.velocity = output.targetVelocity;
        if (cycle == 49) {
            check(fabs(output.targetVelocity - expectedMidpoint) < 0.5,
                  "entry overspeed must ramp linearly: midpoint at 50 ms");
        }
        check(output.targetVelocity >= maxVelocity - 0.01,
              "entry overspeed ramp must not undershoot the segment velocity");
    }
    /* HYD_REAL 为 float：允许斜坡在第 100~101 个周期落点（±1 周期） */
    check(fabs(output.targetVelocity - maxVelocity) < 0.01,
          "entry overspeed transition must complete in t_dec (+1 cycle)");
}

static void test_entry_overspeed_transitions_in_decel_time(void)
{
    /* 高速段入口（原 11ms 陡降）：100 -> 90，50ms 时应为 95 */
    run_entry_overspeed_case(90.0, 100.0, 95.0);
    /* 低速段入口（原 200ms 慢降）：30 -> 10，50ms 时应为 20 */
    run_entry_overspeed_case(10.0, 30.0, 20.0);
}

static void run_lagged_entry_overspeed_case(HYD_REAL maxVelocity,
                                            HYD_REAL entryVelocity)
{
    HYD_AxisRef axis = {0};
    HYD_MotionSegment segment = {0};
    HYD_MotionPlannerState state = {0};
    HYD_MotionPlannerInput input = {0};
    HYD_MotionPlannerOutput output = {0};
    HYD_REAL history[3];
    int firstReachCycle = -1;
    int tightened = 1;
    int cycle;

    segment.mode = HYD_MODE_POSITION;
    segment.planner = HYD_PLANNER_POSITION_BASED;
    segment.endCondition = HYD_END_POSITION;
    segment.direction = HYD_DIRECTION_EXTEND;
    segment.targetPosition = 500.0;   /* far: sqrt envelope not binding */
    segment.maxVelocity = maxVelocity;
    segment.maxAcceleration = 100.0;
    segment.maxDeceleration = 100.0;
    segment.maxFlow = 1000.0;
    segment.velocityToFlowGain = 1.0;

    history[0] = entryVelocity;
    history[1] = entryVelocity;
    history[2] = entryVelocity;
    state.initialized = true;
    state.lastTargetVelocity = entryVelocity;

    input.axisRef = &axis;
    input.segment = &segment;
    input.state = &state;   /* blend == NULL: MoveAbsolute-style, emergency path armed */
    input.deltaTime = 0.001;

    for (cycle = 1; cycle <= 300; ++cycle) {
        HYD_REAL plannedRamp = entryVelocity +
            (maxVelocity - entryVelocity) * (HYD_REAL)cycle / 100.0;
        if (plannedRamp < maxVelocity) {
            plannedRamp = maxVelocity;
        }
        axis.velocity = history[0];   /* measured = command from 3 cycles ago */
        input.elapsedTime = cycle * 0.001;
        HYD_MotionPlanner_Execute(&input, &output);
        if (output.targetVelocity > plannedRamp + 0.01) {
            tightened = 0;
        }
        if (firstReachCycle < 0 && output.targetVelocity <= maxVelocity + 0.05) {
            firstReachCycle = cycle;
        }
        history[0] = history[1];
        history[1] = history[2];
        history[2] = output.targetVelocity;
        axis.position += axis.velocity * 0.001;
    }
    check(tightened,
          "measured-overspeed branch may only tighten the planned entry ramp");
    check(firstReachCycle > 0 && firstReachCycle <= 102,
          "lagged entry overspeed must never take longer than t_dec");
}

static void test_lagged_entry_overspeed_never_slower_than_decel_time(void)
{
    /* prev > 2*maxV: old emergency rate a_cfg=100/s is slower than the
     * planned 200/s and used to stretch the transition to ~199 ms */
    run_lagged_entry_overspeed_case(10.0, 30.0);
    /* prev < 2*maxV: emergency rate is faster (deferred limitation);
     * the transition must still never exceed t_dec */
    run_lagged_entry_overspeed_case(90.0, 100.0);
}

static void setup_blend_braking_case(HYD_AxisRef* axis,
                                     HYD_MotionSegment* segment,
                                     HYD_MotionBlendContext* blend,
                                     HYD_MotionPlannerState* state,
                                     HYD_MotionPlannerInput* input,
                                     HYD_REAL remainingDistance)
{
    memset(axis, 0, sizeof(*axis));
    memset(segment, 0, sizeof(*segment));
    memset(blend, 0, sizeof(*blend));
    memset(state, 0, sizeof(*state));
    memset(input, 0, sizeof(*input));

    segment->mode = HYD_MODE_POSITION;
    segment->planner = HYD_PLANNER_POSITION_BASED;
    segment->endCondition = HYD_END_POSITION;
    segment->direction = HYD_DIRECTION_EXTEND;
    segment->targetPosition = 100.0;
    segment->maxVelocity = 90.0;
    segment->maxAcceleration = 100.0;
    segment->maxDeceleration = 100.0;
    segment->maxFlow = 1000.0;
    segment->velocityToFlowGain = 1.0;

    blend->active = true;
    blend->bufferMode = HYD_BUFFER_MODE_BLENDING_NEXT;
    blend->blendVelocity = 50.0;            /* 终点速度 vt */
    blend->switchPosition = 100.0;
    blend->switchTolerance = 0.05;

    axis->position = 100.0 - remainingDistance;
    axis->velocity = 90.0;
    state->initialized = true;
    state->lastTargetVelocity = 90.0;       /* 巡航中 */

    input->axisRef = axis;
    input->segment = segment;
    input->state = state;
    input->blend = blend;
    input->deltaTime = 0.001;
    input->elapsedTime = 0.001;
}

static void test_braking_trigger_uses_fixed_time_distance(void)
{
    HYD_AxisRef axis;
    HYD_MotionSegment segment;
    HYD_MotionBlendContext blend;
    HYD_MotionPlannerState state;
    HYD_MotionPlannerInput input;
    HYD_MotionPlannerOutput output = {0};

    /* 新触发距离 = (90+50)/2*0.1 + 40*0.001 = 7.04mm；
     * 旧触发距离 = (90²-50²)/(2*900) + 0.04 = 3.15mm。
     * 6.5mm：新语义已制动（<90），旧语义仍巡航（=90）。 */
    setup_blend_braking_case(&axis, &segment, &blend, &state, &input, 6.5);
    HYD_MotionPlanner_Execute(&input, &output);
    check(output.targetVelocity < 90.0 - 0.2,   /* 新语义首周期 = 89.6 */
          "braking must engage at the fixed-time trigger distance");

    /* 8mm 在新触发距离之外：仍巡航，防止过早制动 */
    setup_blend_braking_case(&axis, &segment, &blend, &state, &input, 8.0);
    HYD_MotionPlanner_Execute(&input, &output);
    check(fabs(output.targetVelocity - 90.0) < 0.01,
          "cruise must continue outside the fixed-time trigger distance");
}

static void test_braking_lands_at_terminal_in_decel_time(void)
{
    HYD_AxisRef axis;
    HYD_MotionSegment segment;
    HYD_MotionBlendContext blend;
    HYD_MotionPlannerState state;
    HYD_MotionPlannerInput input;
    HYD_MotionPlannerOutput output = {0};
    int decelCycles = 0;
    int cycle;

    setup_blend_braking_case(&axis, &segment, &blend, &state, &input, 20.0);
    for (cycle = 0; cycle < 400; ++cycle) {
        input.elapsedTime = (cycle + 1) * 0.001;
        HYD_MotionPlanner_Execute(&input, &output);
        axis.position += output.targetVelocity * 0.001;
        axis.velocity = output.targetVelocity;
        if (output.targetVelocity < 90.0 - 0.05 &&
            output.targetVelocity > 50.0 + 0.05) {
            decelCycles++;
        }
        if (axis.position >= blend.switchPosition) {
            break;
        }
    }
    check(axis.position >= blend.switchPosition,
          "approach must reach the switch position");
    check(fabs(output.targetVelocity - 50.0) < 1.5,
          "velocity at the switch crossing must equal the end velocity");
    check(decelCycles >= 95 && decelCycles <= 105,
          "deceleration from 90 to 50 must take ~t_dec (100 cycles)");
}

static void test_braking_stop_lands_on_target_monotone(void)
{
    HYD_AxisRef axis = {0};
    HYD_MotionSegment segment = {0};
    HYD_MotionPlannerState state = {0};
    HYD_MotionPlannerInput input = {0};
    HYD_MotionPlannerOutput output = {0};
    HYD_REAL previous = 90.0;
    int braking = 0;
    int monotone = 1;
    int decelCycles = 0;
    int cycle;

    segment.mode = HYD_MODE_POSITION;
    segment.planner = HYD_PLANNER_POSITION_BASED;
    segment.endCondition = HYD_END_POSITION;
    segment.direction = HYD_DIRECTION_EXTEND;
    segment.targetPosition = 100.0;
    segment.maxVelocity = 90.0;
    segment.maxAcceleration = 100.0;
    segment.maxDeceleration = 100.0;
    segment.maxFlow = 1000.0;
    segment.velocityToFlowGain = 1.0;
    segment.positionTolerance = 0.0001;   /* IEC default: creep zone 0.001mm */

    axis.position = 80.0;
    axis.velocity = 90.0;
    state.initialized = true;
    state.lastTargetVelocity = 90.0;      /* cruising, END velocity 0 */

    input.axisRef = &axis;
    input.segment = &segment;
    input.state = &state;   /* blend == NULL */
    input.deltaTime = 0.001;

    for (cycle = 0; cycle < 600; ++cycle) {
        input.elapsedTime = (cycle + 1) * 0.001;
        HYD_MotionPlanner_Execute(&input, &output);
        axis.position += output.targetVelocity * 0.001;
        axis.velocity = output.targetVelocity;
        if (braking && output.targetVelocity > previous + 1e-4) {
            monotone = 0;
        }
        if (output.targetVelocity < 90.0 - 0.05) {
            braking = 1;
        }
        if (output.targetVelocity > 0.05 && output.targetVelocity < 90.0 - 0.05) {
            decelCycles++;
        }
        previous = output.targetVelocity;
    }
    check(segment.targetPosition - axis.position <= 0.002,
          "stop must land on the target instead of stopping short");
    check(monotone,
          "stop braking command must never increase (no re-anchor blips)");
    /* delay-aware landing decelerates at a*v/(v+a*dt): ~dt*ln(v0/v_end) (~4 cycles) extra; 105 seen at some phases */
    check(decelCycles >= 97 && decelCycles <= 106,
          "stop from 90 must take ~t_dec (100 cycles)");
}

static void run_entry_overspeed_short_stop_case(HYD_REAL maxVelocity,
                                                HYD_REAL entryVelocity,
                                                HYD_REAL remainingDistance)
{
    HYD_AxisRef axis = {0};
    HYD_MotionSegment segment = {0};
    HYD_MotionPlannerState state = {0};
    HYD_MotionPlannerInput input = {0};
    HYD_MotionPlannerOutput output = {0};
    HYD_REAL previous = entryVelocity;
    int monotone = 1;
    int cycle;

    segment.mode = HYD_MODE_POSITION;
    segment.planner = HYD_PLANNER_POSITION_BASED;
    segment.endCondition = HYD_END_POSITION;
    segment.direction = HYD_DIRECTION_EXTEND;
    segment.targetPosition = 100.0;
    segment.maxVelocity = maxVelocity;
    segment.maxAcceleration = 100.0;
    segment.maxDeceleration = 100.0;
    segment.maxFlow = 1000.0;
    segment.velocityToFlowGain = 1.0;
    segment.positionTolerance = 0.1;

    axis.position = 100.0 - remainingDistance;
    axis.velocity = entryVelocity;
    state.initialized = true;
    state.lastTargetVelocity = entryVelocity;  /* entry overspeed, END velocity 0 */

    input.axisRef = &axis;
    input.segment = &segment;
    input.state = &state;   /* blend == NULL */
    input.deltaTime = 0.001;

    for (cycle = 0; cycle < 600; ++cycle) {
        input.elapsedTime = (cycle + 1) * 0.001;
        HYD_MotionPlanner_Execute(&input, &output);
        axis.position += output.targetVelocity * 0.001;
        axis.velocity = output.targetVelocity;
        if (output.targetVelocity > previous + 1e-4) {
            monotone = 0;
        }
        previous = output.targetVelocity;
    }
    check(monotone,
          "short overspeed stop command must never increase (no re-anchor blips)");
    check(segment.targetPosition - axis.position <= 0.1 + 1e-4,
          "short overspeed stop must land within the position tolerance");
}

static void test_braking_entry_overspeed_short_stop_monotone(void)
{
    /* (a) 30 -> 0, maxV 10, 0.3mm remaining */
    run_entry_overspeed_short_stop_case(10.0, 30.0, 0.3);
    /* (b) 100 -> 0, maxV 90, 5mm remaining */
    run_entry_overspeed_short_stop_case(90.0, 100.0, 5.0);
}

static void test_blend_terminal_stop_keeps_envelope_landing(void)
{
    HYD_AxisRef axis = {0};
    HYD_MotionSegment segment = {0};
    HYD_MotionBlendContext blend = {0};
    HYD_MotionPlannerState state = {0};
    HYD_MotionPlannerInput input = {0};
    HYD_MotionPlannerOutput output = {0};
    HYD_REAL remaining;
    int reachedZero = 0;
    int decelCycles = 0;
    int cycle;

    segment.mode = HYD_MODE_POSITION;
    segment.planner = HYD_PLANNER_POSITION_BASED;
    segment.endCondition = HYD_END_POSITION;
    segment.direction = HYD_DIRECTION_EXTEND;
    segment.targetPosition = 100.0;
    segment.maxVelocity = 90.0;
    segment.maxAcceleration = 100.0;
    segment.maxDeceleration = 100.0;
    segment.maxFlow = 1000.0;
    segment.velocityToFlowGain = 1.0;
    segment.positionTolerance = 0.0001;

    /* MoveContinuousAbsolute END velocity 0 approach context */
    blend.active = true;
    blend.bufferMode = HYD_BUFFER_MODE_BLENDING_NEXT;
    blend.blendVelocity = 0.0;
    blend.switchPosition = 100.0;
    blend.switchTolerance = 0.0001;

    axis.position = 80.0;
    axis.velocity = 90.0;
    state.initialized = true;
    state.lastTargetVelocity = 90.0;      /* cruising */

    input.axisRef = &axis;
    input.segment = &segment;
    input.state = &state;
    input.blend = &blend;
    input.deltaTime = 0.001;

    for (cycle = 0; cycle < 600; ++cycle) {
        input.elapsedTime = (cycle + 1) * 0.001;
        HYD_MotionPlanner_Execute(&input, &output);
        axis.position += output.targetVelocity * 0.001;
        axis.velocity = output.targetVelocity;
        if (output.targetVelocity > 0.05 && output.targetVelocity < 90.0 - 0.05) {
            decelCycles++;
        }
        if (output.targetVelocity <= 0.05) {
            reachedZero = 1;
        }
    }
    remaining = segment.targetPosition - axis.position;
    if (remaining < 0.0) {
        remaining = 0.0;
    }
    check(decelCycles >= 97 && decelCycles <= 101,
          "blend terminal stop must keep the ~t_dec envelope landing");
    check(remaining <= 0.001,
          "blend terminal stop must land on the target");
    check(reachedZero,
          "blend terminal stop command must reach zero");
}

static void test_speed_ramp_decel_uses_delta_v_rate(void)
{
    HYD_AxisRef axis = {0};
    HYD_MotionSegment segment = {0};
    HYD_MotionPlannerInput input = {0};
    HYD_MotionPlannerOutput output = {0};

    segment.mode = HYD_MODE_SPEED_RAMP;
    segment.endCondition = HYD_END_TIME;      /* 避开位置包络路径 */
    segment.direction = HYD_DIRECTION_EXTEND;
    segment.maxVelocity = 200.0;              /* 故意 != decelStartVel */
    segment.maxAcceleration = 100.0;
    segment.maxDeceleration = 100.0;
    segment.maxFlow = 1000.0;
    segment.velocityToFlowGain = 1.0;

    axis.velocity = 100.0;

    input.axisRef = &axis;
    input.segment = &segment;
    input.state = NULL;                        /* 走 HYD_ComputeSpeedRampVelocityMagnitude */
    input.deltaTime = 0.001;
    input.elapsedTime = 1.0;                   /* 加速早已完成 */
    input.decelStartVel = 100.0;
    input.decelElapsed = 0.05;                 /* 减速进行到 50ms */

    HYD_MotionPlanner_Execute(&input, &output);
    /* 语义A：a=100/0.1=1000 → v=100-1000*0.05=50；旧：a=200/0.1=2000 → v=0 */
    check(fabs(fabs(output.targetVelocity) - 50.0) < 0.5,
          "speed ramp deceleration must follow decelStartVel/t_dec");
}

static void test_speed_ramp_stateful_decel_uses_delta_v_rate(void)
{
    HYD_AxisRef axis = {0};
    HYD_MotionSegment segment = {0};
    HYD_MotionPlannerState state = {0};
    HYD_MotionPlannerInput input = {0};
    HYD_MotionPlannerOutput output = {0};

    segment.mode = HYD_MODE_SPEED_RAMP;
    segment.planner = HYD_PLANNER_TIME_BASED;
    segment.endCondition = HYD_END_TIME;
    segment.direction = HYD_DIRECTION_EXTEND;
    segment.maxVelocity = 200.0;              /* deliberately != decelStartVel */
    segment.maxAcceleration = 100.0;
    segment.maxDeceleration = 100.0;
    segment.maxFlow = 1000.0;
    segment.velocityToFlowGain = 1.0;

    axis.velocity = 100.0;
    state.initialized = true;
    state.lastTargetVelocity = 100.0;         /* first deceleration cycle */

    input.axisRef = &axis;
    input.segment = &segment;
    input.state = &state;                     /* production (stateful) path */
    input.deltaTime = 0.001;
    input.elapsedTime = 1.0;
    input.decelStartVel = 100.0;
    input.decelElapsed = 0.001;

    HYD_MotionPlanner_Execute(&input, &output);
    /* semantics A: cap = 100 - (100/0.1)*0.001 = 99, ramp 100->0 also 99.
     * old a_cfg = 200/0.1: cap = 100 - 2000*0.001 = 98. */
    check(fabs(fabs(output.targetVelocity) - 99.0) < 0.1,
          "stateful speed ramp deceleration must follow decelStartVel/t_dec");
}

static void test_short_stroke_caps_peak_and_keeps_ramp_time(void)
{
    HYD_AxisRef axis = {0};
    HYD_MotionSegment segment = {0};
    HYD_MotionPlannerState state = {0};
    HYD_MotionPlannerInput input = {0};
    HYD_MotionPlannerOutput output = {0};
    HYD_REAL peak = 0.0;
    HYD_REAL previous = 0.0;
    HYD_REAL maxStep = 0.0;
    int cycle;

    segment.mode = HYD_MODE_POSITION;
    segment.planner = HYD_PLANNER_POSITION_BASED;
    segment.endCondition = HYD_END_POSITION;
    segment.direction = HYD_DIRECTION_EXTEND;
    segment.targetPosition = 5.0;     /* 短行程 5mm */
    segment.maxVelocity = 90.0;       /* 高目标速度 */
    segment.maxAcceleration = 100.0;
    segment.maxDeceleration = 100.0;
    segment.maxFlow = 1000.0;
    segment.velocityToFlowGain = 1.0;
    state.initialized = true;         /* 从静止出发 */

    input.axisRef = &axis;
    input.segment = &segment;
    input.state = &state;
    input.deltaTime = 0.001;

    for (cycle = 0; cycle < 600; ++cycle) {
        input.elapsedTime = (cycle + 1) * 0.001;
        HYD_MotionPlanner_Execute(&input, &output);
        if (output.targetVelocity - previous > maxStep) {
            maxStep = output.targetVelocity - previous;
        }
        previous = output.targetVelocity;
        axis.position += output.targetVelocity * 0.001;
        axis.velocity = output.targetVelocity;
        if (output.targetVelocity > peak) {
            peak = output.targetVelocity;
        }
    }
    /* vp = 2*5/(0.2+0.002) = 49.5：峰值必须被限制在 ~49.5。
     * 旧行为冲向 90 后被 sqrt 包络压回，峰值约 60~67。 */
    check(peak <= 50.5, "short stroke must cap the peak at the reachable vp");
    check(peak >= 40.0, "short stroke cap must not collapse the motion");
    /* 锚定一次：正向（加速）步长 = vp/T_acc ≈ 0.495/周期，绝不出现 90/0.1=0.9
     * 的旧陡升。只计正向步长：落地尾段负向收紧步长 a_env·dt 是既有包络安全
     * 行为（measured-overspeed 收紧），不在本断言管辖。 */
    check(maxStep <= 0.75, "short stroke acceleration steps must use the anchored vp rate");
    check(axis.position > 4.5, "short stroke move must still converge on target");
}

static void test_long_stroke_keeps_full_cruise(void)
{
    HYD_AxisRef axis = {0};
    HYD_MotionSegment segment = {0};
    HYD_MotionPlannerState state = {0};
    HYD_MotionPlannerInput input = {0};
    HYD_MotionPlannerOutput output = {0};
    int cycle;

    segment.mode = HYD_MODE_POSITION;
    segment.planner = HYD_PLANNER_POSITION_BASED;
    segment.endCondition = HYD_END_POSITION;
    segment.direction = HYD_DIRECTION_EXTEND;
    segment.targetPosition = 500.0;
    segment.maxVelocity = 90.0;
    segment.maxAcceleration = 100.0;
    segment.maxDeceleration = 100.0;
    segment.maxFlow = 1000.0;
    segment.velocityToFlowGain = 1.0;
    state.initialized = true;

    input.axisRef = &axis;
    input.segment = &segment;
    input.state = &state;
    input.deltaTime = 0.001;

    for (cycle = 0; cycle < 102; ++cycle) {
        input.elapsedTime = (cycle + 1) * 0.001;
        HYD_MotionPlanner_Execute(&input, &output);
        axis.position += output.targetVelocity * 0.001;
        axis.velocity = output.targetVelocity;
        if (cycle == 49) {
            check(fabs(output.targetVelocity - 45.0) < 0.5,
                  "long stroke acceleration must stay linear (regression guard)");
        }
    }
    check(fabs(output.targetVelocity - 90.0) < 0.05,
          "long stroke must reach full cruise in t_acc (+1 cycle, regression guard)");
}

static void test_chained_segment_keeps_vp_cap_on_carried_state(void)
{
    HYD_AxisRef axis = {0};
    HYD_MotionSegment segment = {0};
    HYD_MotionPlannerState state = {0};
    HYD_MotionPlannerInput input = {0};
    HYD_MotionPlannerOutput output = {0};
    HYD_REAL peak = 0.0;
    HYD_REAL previous = 0.0;
    HYD_REAL maxStep = 0.0;
    int cycle;

    /* 段1：5mm 短行程跑到命令归零——锚定停在指向 vt=0 的制动斜坡上
     * （FB 段链 preserveMotionPlannerState 会整体携带该锚定）。 */
    segment.mode = HYD_MODE_POSITION;
    segment.planner = HYD_PLANNER_POSITION_BASED;
    segment.endCondition = HYD_END_POSITION;
    segment.direction = HYD_DIRECTION_EXTEND;
    segment.targetPosition = 5.0;
    segment.maxVelocity = 90.0;
    segment.maxAcceleration = 100.0;
    segment.maxDeceleration = 100.0;
    segment.maxFlow = 1000.0;
    segment.velocityToFlowGain = 1.0;
    state.initialized = true;

    input.axisRef = &axis;
    input.segment = &segment;
    input.state = &state;
    input.deltaTime = 0.001;

    for (cycle = 0; cycle < 600; ++cycle) {
        input.elapsedTime = (cycle + 1) * 0.001;
        HYD_MotionPlanner_Execute(&input, &output);
        axis.position += output.targetVelocity * 0.001;
        axis.velocity = output.targetVelocity;
        if (output.targetVelocity <= 0.0) {
            break;   /* 段完成：命令归零，携带锚定进入段链 */
        }
    }
    check(output.targetVelocity <= 0.0,
          "chained segment 1 must complete (command reaches zero)");

    /* 段2：同一 state（含携带锚定）再跑 5mm 短行程（目标 10mm）。
     * 旧行为：锚定制动斜坡判据误触发跳过，第二段冲向 maxV，
     * peak=59.4、加速 +0.9/周期。 */
    segment.targetPosition = 10.0;

    for (cycle = 0; cycle < 600; ++cycle) {
        input.elapsedTime = (cycle + 1) * 0.001;
        HYD_MotionPlanner_Execute(&input, &output);
        if (output.targetVelocity - previous > maxStep) {
            maxStep = output.targetVelocity - previous;
        }
        previous = output.targetVelocity;
        axis.position += output.targetVelocity * 0.001;
        axis.velocity = output.targetVelocity;
        if (output.targetVelocity > peak) {
            peak = output.targetVelocity;
        }
    }
    check(peak <= 50.5, "chained short stroke must keep the reachable vp cap");
    check(maxStep <= 0.75, "chained short stroke acceleration must use the vp rate");
    check(axis.position > 9.5, "chained short stroke must converge on target");
}

static void test_mold_open_sequence_transitions_take_decel_time(void)
{
    static const HYD_REAL TARGET[4] = { 50.0, 60.0, 70.0, 90.0 };
    static const HYD_REAL VELOCITY[4] = { 18.0, 90.0, 54.0, 18.0 };
    HYD_AxisRef axis = {0};
    HYD_MotionPlannerState state = {0};
    HYD_MotionPlannerOutput output = {0};
    int seg;

    state.initialized = true;
    axis.position = 2.0;                      /* 实验中开模起点约 2mm */

    for (seg = 0; seg < 4; ++seg) {
        HYD_MotionSegment segment = {0};
        HYD_MotionBlendContext blend = {0};
        HYD_MotionPlannerInput input = {0};
        HYD_REAL entryVelocity = fabs(state.lastTargetVelocity);
        int transitionCycles = 0;
        int cycle;

        segment.mode = HYD_MODE_POSITION;
        segment.planner = HYD_PLANNER_POSITION_BASED;
        segment.endCondition = HYD_END_POSITION;
        segment.direction = HYD_DIRECTION_EXTEND;
        segment.targetPosition = TARGET[seg];
        segment.maxVelocity = VELOCITY[seg];
        segment.maxAcceleration = 100.0;
        segment.maxDeceleration = 100.0;
        segment.maxFlow = 1000.0;
        segment.velocityToFlowGain = 1.0;

        blend.active = true;
        blend.bufferMode = HYD_BUFFER_MODE_BLENDING_NEXT;
        blend.blendVelocity = VELOCITY[seg];  /* 终速 = 本段速度 */
        blend.switchPosition = TARGET[seg];
        blend.switchTolerance = 0.05;

        input.axisRef = &axis;
        input.segment = &segment;
        input.state = &state;
        input.blend = &blend;
        input.deltaTime = 0.001;

        for (cycle = 0; cycle < 20000; ++cycle) {
            HYD_REAL magnitude;

            input.elapsedTime = (cycle + 1) * 0.001;
            HYD_MotionPlanner_Execute(&input, &output);
            axis.position += output.targetVelocity * 0.001;
            axis.velocity = output.targetVelocity;
            magnitude = fabs(output.targetVelocity);
            if (fabs(magnitude - segment.maxVelocity) > 0.05 &&
                fabs(magnitude - entryVelocity) > 0.05) {
                transitionCycles++;   /* 处于段入口过渡斜坡上 */
            }
            if (axis.position >= segment.targetPosition - blend.switchTolerance) {
                break;
            }
        }
        check(axis.position >= segment.targetPosition - 0.1,
              "sequence segment must reach its switch position");
        if (seg > 0 && fabs(entryVelocity - segment.maxVelocity) > 1.0) {
            /* 实验值：90→54 用 67ms，54→18 用 200ms；修复后都应 ~100ms */
            check(transitionCycles >= 95 && transitionCycles <= 105,
                  "inter-segment velocity transition must take ~t_dec");
        }
    }
}

/* ---------------------------------------------------------------------------
 * 合模序列（实验曲线 docs/data_deg/vofa_waveform_20261110_082428_443.csv，
 * t = 6474..7280 ms，位置 130mm -> 5mm，RETRACT）。
 *
 * 段  目标mm  uiSpd/END   v(mm/s)  END(mm/s)  行程(mm)
 * 1   120     100/100     180      180        130->120 (10)
 * 2   110      90/ 90     162      162        120->110 (10)
 * 3    70     100/100     180      180        110-> 70 (40)
 * 4     5      90/ 50     162       90         70->  5 (65)
 * 单位：uiSpd x 1.8 mm/s；t_acc=t_dec=100ms（88ms 经 100ms 下限限幅）。
 * 段5（5->0，END=0）是低压护模/保压段：实测速度瞬间降到 ~1mm/s 后由压力
 * 闭环接管，不属于位置规划序列，不纳入本测试。
 *
 * 旧语义实测（修复前 CSV）：段2/段4 入口 180->162 用 12ms @ -1500~-1620
 * mm/s²；段4 END 162->90 用 44ms @ -1607 mm/s²（a_cfg = maxV/t_dec = 1620，
 * 即 1.62 mm/s per 1ms 周期）。语义A 期望恒时间 100ms、斜率 = Δv/t：
 *   段1 +1.800、段2 -0.180、段3 +0.180、
 *   段4 入口 -0.180、段4 END -0.720（mm/s per 1ms 周期）。
 * 段2 的 10mm 行程 < 恒时间减速所需 (180+162)/2*0.1=17.1mm，过渡会被段
 * 边界截断、尾部由 sqrt 包络兜底（Task 6 刹距不足 WARNING 的工艺场景），
 * 因此段2 只校验入口斜坡斜率与"未按旧语义提前到位"。
 * ------------------------------------------------------------------------- */
typedef struct {
    int start;
    int cycles;
    HYD_REAL from;
    HYD_REAL to;
    HYD_REAL meanStep;
    HYD_REAL maxStep;
    int rising;
} HYD_TraceRamp;

static int collect_trace_ramps(const HYD_REAL* trace, int count,
                               HYD_TraceRamp* ramps, int capacity)
{
    const HYD_REAL eps = 0.005;
    int k = 0;
    int found = 0;

    while (k < count - 1 && found < capacity) {
        HYD_REAL dv = trace[k + 1] - trace[k];
        int sign;
        int start;
        int steps = 0;
        HYD_REAL sum = 0.0;
        HYD_REAL maximum = 0.0;

        if (fabs(dv) <= eps) {
            ++k;
            continue;
        }
        sign = (dv > 0.0) ? 1 : -1;
        start = k;
        while (k < count - 1 &&
               (trace[k + 1] - trace[k]) * sign > eps) {
            HYD_REAL step = (HYD_REAL)fabs(trace[k + 1] - trace[k]);
            sum += step;
            if (step > maximum) {
                maximum = step;
            }
            ++steps;
            ++k;
        }
        if (steps >= 5) {
            ramps[found].start = start;
            ramps[found].cycles = steps;
            ramps[found].from = trace[start];
            ramps[found].to = trace[k];
            ramps[found].meanStep = sum / steps;
            ramps[found].maxStep = maximum;
            ramps[found].rising = (sign > 0);
            ++found;
        }
    }
    return found;
}

static void test_mold_close_sequence_transition_slopes(void)
{
    static const HYD_REAL TARGET[4] = { 120.0, 110.0, 70.0, 5.0 };
    static const HYD_REAL VELOCITY[4] = { 180.0, 162.0, 180.0, 162.0 };
    static const HYD_REAL END_VELOCITY[4] = { 180.0, 162.0, 180.0, 90.0 };
    static HYD_REAL trace[2000];
    HYD_AxisRef axis = {0};
    HYD_MotionPlannerState state = {0};
    HYD_MotionPlannerOutput output = {0};
    HYD_REAL slope[4] = {0.0, 0.0, 0.0, 0.0};
    HYD_REAL endSlope = 0.0;
    HYD_REAL seg2EntryMean = 0.0;
    HYD_REAL seg2EntryMax = 0.0;
    int cyclesCount[4] = {0, 0, 0, 0};
    int endCycles = 0;
    int sawNegative = 0;
    int seg;

    state.initialized = true;
    axis.position = 130.0;

    for (seg = 0; seg < 4; ++seg) {
        HYD_MotionSegment segment = {0};
        HYD_MotionBlendContext blend = {0};
        HYD_MotionPlannerInput input = {0};
        HYD_TraceRamp ramps[4];
        int rampCount;
        int n = 0;
        int cycle;

        segment.mode = HYD_MODE_POSITION;
        segment.planner = HYD_PLANNER_POSITION_BASED;
        segment.endCondition = HYD_END_POSITION;
        segment.direction = HYD_DIRECTION_RETRACT;
        segment.targetPosition = TARGET[seg];
        segment.maxVelocity = VELOCITY[seg];
        segment.maxAcceleration = 100.0;
        segment.maxDeceleration = 100.0;
        segment.maxFlow = 1000.0;
        segment.velocityToFlowGain = 1.0;

        blend.active = true;
        blend.bufferMode = HYD_BUFFER_MODE_BLENDING_NEXT;
        blend.blendVelocity = END_VELOCITY[seg];
        blend.switchPosition = TARGET[seg];
        blend.switchTolerance = 0.05;

        input.axisRef = &axis;
        input.segment = &segment;
        input.state = &state;
        input.blend = &blend;
        input.deltaTime = 0.001;

        for (cycle = 0; cycle < 2000; ++cycle) {
            input.elapsedTime = (cycle + 1) * 0.001;
            HYD_MotionPlanner_Execute(&input, &output);
            if (output.targetVelocity < -0.001) {
                sawNegative = 1;
            }
            trace[n++] = (HYD_REAL)fabs(output.targetVelocity);
            axis.position += output.targetVelocity * 0.001;
            axis.velocity = output.targetVelocity;
            if (axis.position <= TARGET[seg] + blend.switchTolerance) {
                break;
            }
        }
        check(n < 2000, "mold close segment must finish within the trace buffer");
        check(fabs(axis.position - TARGET[seg]) < 0.15,
              "mold close segment must reach its switch position");

        rampCount = collect_trace_ramps(trace, n, ramps, 4);
        check(rampCount >= 1, "mold close segment must show a transition ramp");

        if (seg == 1) {
            /* 10mm 行程不足 17.1mm 恒时间刹距：入口斜坡按 Δv/t_dec 起步，
             * 段边界截断后尾部由包络兜底。旧语义会在 ~12ms 内以
             * 1.62 mm/s/ms 冲到 162 并巡航到段末。 */
            int k;
            for (k = 0; k < 29; ++k) {
                HYD_REAL step = (HYD_REAL)fabs(trace[k + 1] - trace[k]);
                seg2EntryMean += step;
                if (step > seg2EntryMax) {
                    seg2EntryMax = step;
                }
            }
            seg2EntryMean /= 29.0;
            check(ramps[0].cycles >= 40,
                  "short close segment transition must not collapse to a_cfg timing");
            check(fabs(seg2EntryMean - 0.18) < 0.03,
                  "short close segment entry slope must be delta-v/t_dec");
            check(seg2EntryMax < 0.25,
                  "short close segment entry must not use the old a_cfg step");
            check(fabs(trace[n - 1] - 162.0) > 0.2,
                  "short close segment must stay on the truncated ramp at switch");
            check(ramps[0].maxStep <= 1.7,
                  "short close segment tail must stay within the envelope step");
            slope[seg] = seg2EntryMean;
            cyclesCount[seg] = ramps[0].cycles;
            continue;
        }

        if (rampCount >= 1) {
            const HYD_TraceRamp* first = &ramps[0];
            HYD_REAL expected = (seg == 0) ? 1.8 : 0.18;
            slope[seg] = first->meanStep;
            cyclesCount[seg] = first->cycles;
            check(first->cycles >= 95 && first->cycles <= 105,
                  "mold close transition must take t_dec/t_acc (95..105 cycles)");
            check(fabs(first->meanStep - expected) <= 0.03,
                  "mold close transition slope must equal delta-v/t");
            check(first->maxStep <= expected * 1.2,
                  "mold close transition must not contain an old a_cfg step");
            check(first->rising == (seg == 0 || seg == 2),
                  "mold close transition direction must match the setpoint change");
        }

        if (seg == 3) {
            /* 段4 的 END_SPEED=90 是下一段速度（段内前瞻写法）：段内还有
             * 一条 162->90 的 END 制动斜坡，旧语义只用 44ms @ a_cfg。 */
            if (rampCount >= 2) {
                const HYD_TraceRamp* last = &ramps[rampCount - 1];
                endSlope = last->meanStep;
                endCycles = last->cycles;
                check(last->rising == 0, "mold close END braking must decelerate");
                check(fabs(last->from - 162.0) < 0.5 &&
                      fabs(last->to - 90.0) < 0.5,
                      "mold close END braking must span 162 to 90 mm/s");
                check(last->cycles >= 95 && last->cycles <= 105,
                      "mold close END braking must take t_dec");
                check(fabs(last->meanStep - 0.72) <= 0.05,
                      "mold close END braking slope must equal delta-v/t_dec");
                check(last->maxStep <= 0.9,
                      "mold close END braking must not use the old a_cfg step");
            } else {
                check(0, "mold close segment 4 must have an END braking ramp");
            }
        }
    }

    check(sawNegative, "mold close sequence must command the RETRACT direction");
    printf("mold close slopes (mm/s per ms): s1 +%.3f/%dms  s2 -%.3f/%dms(truncated)  "
           "s3 +%.3f/%dms  s4 -%.3f/%dms  end -%.3f/%dms\n",
           slope[0], cyclesCount[0], slope[1], cyclesCount[1],
           slope[2], cyclesCount[2], slope[3], cyclesCount[3],
           endSlope, endCycles);
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
    test_position_safety_uses_planned_velocity_for_delta();
    test_position_terminated_speed_ramp_keeps_distance_safety_cap();
    test_speed_ramp_preserves_elapsed_deceleration_cap();
    test_blend_switch_safety_caps_velocity_at_terminal();
    test_blend_terminal_ramp_continues_from_zero_planned_velocity();
    test_small_target_and_time_changes_keep_cached_rate();
    test_entry_overspeed_transitions_in_decel_time();
    test_lagged_entry_overspeed_never_slower_than_decel_time();
    test_braking_trigger_uses_fixed_time_distance();
    test_braking_lands_at_terminal_in_decel_time();
    test_braking_stop_lands_on_target_monotone();
    test_braking_entry_overspeed_short_stop_monotone();
    test_blend_terminal_stop_keeps_envelope_landing();
    test_speed_ramp_decel_uses_delta_v_rate();
    test_speed_ramp_stateful_decel_uses_delta_v_rate();
    test_short_stroke_caps_peak_and_keeps_ramp_time();
    test_long_stroke_keeps_full_cruise();
    test_chained_segment_keeps_vp_cap_on_carried_state();
    test_mold_open_sequence_transitions_take_decel_time();
    test_mold_close_sequence_transition_slopes();

    printf("motion planner time ramp: %s\n", failures == 0 ? "PASS" : "FAIL");
    return failures == 0 ? 0 : 1;
}
