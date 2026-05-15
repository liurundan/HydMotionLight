# Online Trapezoid Position Planner Design

Date: 2026-05-15

## Goal

Implement an online trapezoid velocity planner for `HYD_MODE_POSITION` so servo-pump hydraulic cylinder position moves no longer command step changes in velocity, flow, or pump speed at segment start.

The planner must keep the current runtime architecture:

- the motion library owns trajectory math, velocity/flow references, pump conversion, diagnostics, and completion signals
- the PLC process layer owns valves, interlocks, and machine phase sequencing
- no heap allocation, no dynamic profile buffers, and no IEC interface change in this phase

## Problem

The current `HYD_PLANNER_POSITION_BASED` path is a position braking law:

```c
v = sqrt(2 * deceleration * remainingDistance)
```

That law is useful near the target, but far from the target it immediately saturates at `maxVelocity`. On a servo-pump hydraulic axis this can produce a command step:

```text
targetVelocity: 0 -> maxVelocity
targetFlow:     0 -> maxVelocity * velocityToFlowGain
pumpSpeed:      0 -> targetFlow * FLOW_TO_PUMP_SPEED_GAIN
```

This is a direct source of hydraulic shock because the pump-flow command changes faster than the oil column, cylinder, load, and pressure dynamics can absorb.

The project already has `HYD_PlanTrapezoid()` and `HYD_EvalTrapezoid()`, but those functions are offline profile helpers. They are tested, but they are not connected to `HYD_MotionPlanner_Execute()`.

## Online vs Offline Trapezoid Planning

### Offline Trapezoid

An offline trapezoid profile is computed once from total move distance:

```text
distance, maxVelocity, acceleration -> tAcc, tConst, tDec
```

Runtime then evaluates the planned profile by elapsed time. This is appropriate when the actuator follows the planned profile closely.

Current `HYD_PlanTrapezoid()` is offline:

- assumes start velocity is zero
- assumes end velocity is zero
- uses one acceleration value for both acceleration and deceleration
- does not account for live feedback error during execution
- does not re-plan when load, pressure, or actual position deviates from the plan

### Online Trapezoid

An online trapezoid planner runs every control cycle. It uses live feedback and the previous target velocity:

```text
actualPosition, targetPosition, lastTargetVelocity, dt,
maxVelocity, maxAcceleration, maxDeceleration
```

Each cycle decides whether to accelerate, cruise, or decelerate based on the remaining distance and current commanded velocity. This is better for hydraulic position control because the cylinder may lag or lead the reference due to oil compressibility, pump delay, friction, load changes, and pressure buildup.

## Decision

Use an online trapezoid velocity planner as the default runtime behavior for `HYD_MODE_POSITION`.

Keep `HYD_PlanTrapezoid()` and `HYD_EvalTrapezoid()` as offline utility functions for:

- test references
- commissioning estimates
- future offline profile support
- simulation or documentation

Do not route production hydraulic position control through the current offline profile in this phase.

## Planner Behavior

For each cycle, resolve the segment direction as today:

```c
direction = HYD_Segment_ResolveDirection(segment, axisRef);
```

Then compute the unsigned remaining distance:

```text
EXTEND:  targetPosition - actualPosition
RETRACT: actualPosition - targetPosition
```

Clamp negative remaining distance to zero.

Let:

```text
vPrev = abs(state->lastTargetVelocity)
dt = input->deltaTime
a = segment->maxAcceleration
d = segment->maxDeceleration > 0 ? segment->maxDeceleration : segment->maxAcceleration
vMax = segment->maxVelocity
```

The online trapezoid decision is:

```text
if dt <= 0:
    vNext = vPrev
else if remaining <= positionTolerance:
    vNext = 0
else:
    brakeDistance = vPrev * vPrev / (2 * d)
    if remaining <= brakeDistance:
        vNext = max(0, vPrev - d * dt)
    else:
        vNext = min(vMax, vPrev + a * dt)

    safetyVelocity = sqrt(2 * d * remaining)
    vNext = min(vNext, safetyVelocity)
```

The signed output remains:

```text
targetVelocity = vNext * directionSign
targetFlow = clamp(vNext * velocityToFlowGain, 0, flowLimit)
```

This gives the desired T-shaped behavior online:

- start: velocity rises by no more than `maxAcceleration * dt`
- mid-travel: velocity is capped by `maxVelocity`
- target approach: velocity falls by no more than `maxDeceleration * dt`
- short travel: motion naturally becomes triangular
- disturbed feedback: the next cycle adapts to the new remaining distance

## Safety And Hydraulic Constraints

The braking velocity remains a hard safety cap:

```text
safetyVelocity = sqrt(2 * d * remaining)
```

This prevents the commanded velocity from exceeding what can be stopped within the current remaining distance, even if the acceleration branch would otherwise continue increasing speed.

Target completion is not decided by the planner. It remains owned by `segment_completion.c` using position tolerance, stable window, and stable velocity limits.

When remaining distance is within position tolerance, the planner outputs zero velocity and zero flow for position mode. Completion still requires the existing stable completion logic.

## Runtime Integration

Modify only the planner layer and tests in the implementation phase:

- `include/motion_planner.h`
- `src/motion_planner.c`
- `tests/test_motion_planner.c`
- targeted runtime tests if needed, such as `tests/test_velocity_controller.c` or an integration test that observes `plannedFlow` / `PUMP_SPEED`

No IEC surface changes are required.

No new public segment fields are required in the first version. Existing fields are sufficient:

- `targetPosition`
- `maxVelocity`
- `maxAcceleration`
- `maxDeceleration`
- `velocityToFlowGain`
- `maxFlow`
- `targetFlow`
- typed position tolerance
- `HYD_MotionPlannerState.lastTargetVelocity`

## Planner Type Semantics

The implementation should preserve the enum values but clarify their behavior:

- `HYD_PLANNER_POSITION_BASED`
  - use online trapezoid position planning with braking safety cap
  - this becomes the preferred position-control path for servo-pump hydraulic cylinders

- `HYD_PLANNER_TIME_BASED`
  - keep the current elapsed-time ramp behavior plus braking safety cap
  - keep existing compatibility for recipes that intentionally use elapsed-time velocity buildup

The old pure braking-law behavior should not remain the primary behavior for `HYD_PLANNER_POSITION_BASED`.

## Error Handling

The planner should fail safely by outputting hold/zero when:

- input, segment, or axis reference is null
- direction resolves to `HYD_DIRECTION_HOLD`
- `maxVelocity <= 0`
- `maxAcceleration <= 0`
- resolved deceleration <= 0
- remaining distance is zero or within tolerance

Timestamp rollback remains handled by `motion_control.c`; the planner only treats `deltaTime <= 0` as a no-advance cycle.

## Testing Requirements

Add or update tests to prove:

1. `HYD_PLANNER_POSITION_BASED` no longer jumps to `maxVelocity` on the first active cycle.
2. Velocity increase is bounded by `maxAcceleration * deltaTime`.
3. Velocity decrease is bounded by `maxDeceleration * deltaTime`.
4. Remaining-distance braking cap is still enforced.
5. Short moves become triangular and never demand an unreachable velocity.
6. RETRACT direction produces negative signed velocity and positive flow magnitude.
7. `maxDeceleration` is used independently from `maxAcceleration`.
8. Position tolerance produces zero target velocity and zero flow.
9. Runtime integration shows `STATE.plannedVelocity`, `STATE.plannedFlow`, and `PUMP_SPEED` ramp smoothly for position segments.

Existing offline trapezoid tests for `HYD_PlanTrapezoid()` and `HYD_EvalTrapezoid()` should remain. They should be treated as offline utility coverage, not proof that runtime position control is using an offline profile.

## Out Of Scope

This design does not implement:

- S-curve / jerk-limited planning
- continuous update of an active target position
- blend modes beyond the current supported buffer-mode subset
- pressure-limited position planning
- valve sequencing
- automatic machine phase transitions
- changes to IEC POU layouts

S-curve planning is the logical next improvement after online T planning. It should be a separate design because it introduces jerk limits, additional state, and more tuning choices.

## Acceptance Criteria

The implementation is accepted when:

- all existing tests pass
- new planner tests pass
- position-mode startup velocity is continuous from zero
- commanded flow and pump speed ramp instead of stepping to their maximum values
- near-target deceleration still prevents overshoot
- no public IEC interface changes are required
- offline trapezoid helper functions remain available and tested

## Implementation Status

Implemented after this design:

- `HYD_PLANNER_POSITION_BASED` uses an online trapezoid velocity planner for `HYD_MODE_POSITION`.
- The offline `HYD_PlanTrapezoid()` / `HYD_EvalTrapezoid()` helpers remain available and tested as utility functions.
- Unit tests cover acceleration limiting, deceleration limiting, braking safety cap, short triangular moves, retract direction, position tolerance zero-output, and independent max deceleration.
- Runtime integration coverage verifies smooth `STATE.plannedVelocity`, `STATE.plannedFlow`, and `PUMP_SPEED` ramping.
