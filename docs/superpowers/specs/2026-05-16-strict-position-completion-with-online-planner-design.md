# Strict Position Completion With Online Planner Design

Date: 2026-05-16

## Goal

Make position-move completion consistent with the online trapezoid position planner.

After adopting online velocity planning, a position move must not complete merely because the
axis has entered `positionTolerance`. Completion should mean the axis has reached the position
window and the motion has settled.

The target completion rule is:

```text
position reached
AND planned velocity settled
AND actual velocity settled
AND optional stable window satisfied
```

## Problem

`segment_completion.c` currently treats `HYD_END_POSITION` primarily as a position-window check:

```text
EXTEND:  axisRef.position >= targetPosition - positionTolerance
RETRACT: axisRef.position <= targetPosition + positionTolerance
AUTO/HOLD fallback: abs(position error) <= positionTolerance
```

The existing `stableVelocityLimit` and `stableWindow` can add a velocity-settled gate, but only
when the segment explicitly configures them. In normal direct positioning, `MoveAbsolute` builds
segments from `_params.positionTolerance` and `_params.velocityTolerance`, but it does not set
`stableVelocityLimit` or `stableWindow`.

That creates a mismatch:

- the online planner may still be decelerating smoothly inside the position window
- completion can end the segment as soon as the position enters the tolerance band
- the state reporter and protection layer can then clear outputs
- the observed velocity curve can jump from a small non-zero value to zero

For servo-pump hydraulic position control this is not acceptable. A position window is only a
candidate for completion; it is not a safe completion condition by itself.

## Decision

Upgrade `HYD_END_POSITION` completion to require both position and velocity settlement.

The velocity-settled check should use both:

- the planner/reference velocity: `abs(context->references->velocityReference)`
- the measured axis velocity: `abs(context->axisRef->velocity)`

Both must be below the resolved settled-velocity tolerance before the segment may complete.

This gives two protections:

- the controller will not end a segment while it is still commanding deceleration
- the controller will not report done while the hydraulic axis is still physically moving

## Completion Rule

For `HYD_END_POSITION`, compute `positionReached` using the existing direction semantics.

Then resolve:

```text
settledVelocityTolerance =
    segment.stableVelocityLimit > 0
        ? segment.stableVelocityLimit
        : segment.velocityTolerance > 0
            ? segment.velocityTolerance
            : HYD_DEFAULT_POSITION_SETTLED_VELOCITY_TOLERANCE
```

Use `HYD_DEFAULT_POSITION_SETTLED_VELOCITY_TOLERANCE = 1.0 mm/s` for this phase.

The raw completion condition becomes:

```text
rawComplete =
    positionReached
    && abs(velocityReference) <= settledVelocityTolerance
    && abs(axisRef.velocity) <= settledVelocityTolerance
```

Then pass `rawComplete` through the existing stable-window logic:

```text
complete = ApplyStableWindow(context, rawComplete)
```

The stable-window layer remains optional:

- `stableWindow <= 0` means complete immediately once position and velocity are settled
- `stableWindow > 0` requires the settled condition to remain true for the configured duration

Velocity must be checked before the stable window starts. If either actual or planned velocity
exceeds the tolerance, the stable-window candidate must reset.

## Reference Velocity Fallback

`HYD_SegmentCompletion_CheckWithContext()` may be called with `context->references == NULL`.

When no execution reference is available, use this fallback:

```text
velocityReference = 0
```

This preserves existing standalone tests and utility usage. Runtime motion-control calls already
provide `executionReference`, so production execution will use the planner's current velocity
reference.

## Scope

Modify the completion layer, not the planner, for this behavior.

Primary implementation targets:

- `src/segment_completion.c`
- `tests/segment_completion_test.c`
- targeted integration tests that observe `MoveAbsolute` or direct position segments near done

No public IEC interface change is required.

No new public struct field is required in this phase. Existing fields remain sufficient:

- `positionTolerance`
- `velocityTolerance`
- `stableVelocityLimit`
- `stableWindow`
- `HYD_ExecutionReference.velocityReference`

## Compatibility

The behavior changes only for `HYD_END_POSITION`.

Unchanged:

- `HYD_END_TIME`
- `HYD_END_PRESSURE`
- `HYD_END_FLOW`
- `HYD_END_MANUAL`
- pressure-hold behavior
- explicit `HYD_DIRECTION_HOLD` planner output behavior

Existing segments that already configure `stableVelocityLimit` keep using that value. Segments
that only configure `velocityTolerance` now gain a meaningful default velocity-settled gate.
Segments that configure neither use the library default of `1.0 mm/s`.

This may delay `DONE` compared with the old behavior. That is intentional: the old behavior could
report done while motion was still decelerating.

## Interaction With Online Trapezoid Planning

The planner owns velocity continuity. Completion owns lifecycle state.

The online planner may continue producing a small non-zero target velocity inside
`positionTolerance` while it decelerates according to `maxDeceleration`. Completion must wait for
that planned velocity to settle.

This corrects the older design note in the online-planner spec that said position tolerance should
produce zero target velocity. With online hydraulic positioning, entering the position tolerance
band should not force a command step to zero. It should permit controlled deceleration, and
completion should occur only after the command and feedback velocities settle.

## Tests

Add or update tests for:

1. Position reached but planner velocity non-zero: not complete.
2. Position reached and planner velocity settled but actual velocity non-zero: not complete.
3. Position reached and both velocities settled: complete.
4. `stableVelocityLimit` overrides `velocityTolerance`.
5. If neither velocity field is configured, the default `1.0 mm/s` threshold is used.
6. Existing `stableWindow` still delays completion and resets when either velocity becomes
   unsettled.
7. Non-position end conditions are unchanged.
8. Runtime position integration does not report done before planned velocity reaches the settled
   threshold.

## Out Of Scope

- Changing the IEC function block API
- Adding jerk-limited S-curve planning
- Changing the `HYD_MotionPlanner_Execute()` online trapezoid algorithm
- Refactoring recipe validation
- Changing `HYD_MAX_SEGMENTS`
