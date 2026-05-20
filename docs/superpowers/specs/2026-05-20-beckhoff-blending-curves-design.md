# Beckhoff Blending Curves Design

Date: 2026-05-20

## Goal

Implement real Beckhoff-style `BlendingLow`, `BlendingPrevious`, `BlendingNext`, and `BlendingHigh` transition behavior for finite direct `MoveAbsolute -> MoveAbsolute` command chains.

The existing buffer-live-update work already accepts Beckhoff-compatible `BufferMode` values and stores one pending direct command. This design completes the missing physical behavior: blend modes must produce distinct through-velocity constraints instead of behaving like plain buffered execution.

## Official Documentation Basis

This design follows the Beckhoff / PLCopen MC2 `BufferMode` interpretation used by TwinCAT motion libraries:

- `Aborting`: interrupt current motion and start the new command.
- `Buffered`: execute the new command after the current command ends.
- `BlendingLow`: blend using the lower velocity at the transition.
- `BlendingPrevious`: blend using the previous command's velocity at the transition.
- `BlendingNext`: blend using the next command's velocity at the transition.
- `BlendingHigh`: blend using the higher velocity at the transition.

The relevant Beckhoff documentation families for this project are:

- `TF8560_Plastic_Technology_Functions`, for injection-molding process function context.
- `TF5810_TC3_Hydraulic_Positioning`, for hydraulic axis positioning and online setpoint generation context.
- Beckhoff PLCopen MC2 `MC_BufferMode` / `MC_BUFFER_MODE` documentation, for the `BufferMode` enum semantics.

This library remains hydraulic-control oriented: it uses current feedback, remaining path, acceleration, and deceleration each scan rather than relying on a precomputed offline motion table.

## Scope

First implementation scope:

- Support real blend curves only for finite direct `MoveAbsolute -> MoveAbsolute`.
- Keep the direct buffer capacity at "active command plus one pending command".
- Preserve current `MoveVelocity` and no-duration `PressureHandle` fallback behavior because endless commands do not have a natural finite blend point.
- Do not implement geometric arcs, splines, multi-command look-ahead, jerk-limited S-curves, or multi-axis coordinated path blending.

Unsupported command-type combinations do not receive real blend curves in this phase. They keep the current safe fallback behavior: abort takeover for endless commands, or plain buffered lifecycle where appropriate.

## Architecture

Responsibilities stay separated:

- `motion_interface.c` validates IEC pins and submits direct commands with `BufferMode`.
- `motion_control.c` owns command lifecycle, one-slot pending state, blend eligibility, blend context creation, cutover timing, and cleanup on Stop/Abort/Reset/Fault.
- `motion_planner.c` owns the actual velocity curve. It receives a blend context and computes a continuous online trapezoid reference.
- `segment_completion.c` remains the normal completion authority. Blend cutover is handled by `motion_control.c` before a blended front segment is allowed to behave like a stop-and-Done segment.

The core rule is:

**A blended front segment targets a nonzero terminal velocity at the first segment's target point instead of targeting zero velocity.**

The following segment then inherits the existing planner state, so the first scan of the next segment continues from the actual through velocity instead of restarting from zero.

## Blend Context

Add a compact planner-facing context in `include/motion_planner.h`:

```c
typedef struct {
    HYD_BOOL active;
    HYD_BufferMode bufferMode;
    HYD_REAL blendVelocity;
    HYD_REAL switchPosition;
    HYD_REAL switchTolerance;
} HYD_MotionBlendContext;
```

Extend `HYD_MotionPlannerInput`:

```c
const HYD_MotionBlendContext* blend;
```

`motion_control.c` stores the runtime blend state in `HYD_MotionControlFB`, using the same fields or equivalent compact internal fields. The stored context is valid only while the current active direct segment is the front segment of an eligible `MoveAbsolute -> MoveAbsolute` blend.

## Blend Velocity Selection

When a pending direct `MoveAbsolute` arrives while another finite direct `MoveAbsolute` is active and `BufferMode` is one of the four blending modes, the core computes a through velocity magnitude:

```text
vPrevious = activeSegment.maxVelocity
vNext     = pendingSegment.maxVelocity

BlendingLow      -> min(vPrevious, vNext)
BlendingPrevious -> vPrevious
BlendingNext     -> vNext
BlendingHigh     -> max(vPrevious, vNext)
```

The selected value is a target through velocity, not an unconditional output. The planner still applies:

- `maxAcceleration`
- `maxDeceleration`
- remaining-distance safety cap
- `maxFlow`
- `targetFlow` cap for position/speed modes
- direction consistency

If the selected through velocity is unreachable because the second command arrived too late, the planner transitions as smoothly as possible from the current state without creating a velocity step.

## Planner Curve

The current online position planner effectively plans toward a terminal velocity of zero at the target:

```text
s = distance_to_target
vStopCap = sqrt(2 * decel * s)
vNext = min(rate_limited_velocity, vStopCap)
```

For an active blend front segment, replace the stop cap with a terminal-velocity cap:

```text
s = distance_to_switch_position
vTerminal = blendVelocity
vBlendCap = sqrt(vTerminal^2 + 2 * decel * s)
vNext = min(rate_limited_velocity, vBlendCap)
```

At the switch point, this permits a nonzero through velocity. Away from the switch point, it still prevents an unreachable velocity profile because the cap is based on the distance remaining before the transition.

For ordinary non-blended position moves:

```text
vTerminal = 0
```

So existing stop-at-target behavior remains unchanged.

## Cutover State Machine

`motion_control.c` starts the pending blended segment when all of these are true:

- active segment source is direct
- active segment kind is `MoveAbsolute`
- pending direct segment kind is `MoveAbsolute`
- pending buffer mode is one of `BlendingLow..BlendingHigh`
- both segments are finite position segments
- directions are compatible for a nonzero through transition
- the active segment has entered its switch tolerance or crossed `switchPosition`

On cutover:

- copy pending segment into `DIRECT_SEGMENT`
- clear the pending slot
- begin the next segment
- do not reset `_plannerState`
- keep the current velocity reference continuous
- set normal segment-change reporting
- avoid a front-segment stop-and-Done pause

The front command may complete from the IEC user's perspective according to the existing direct-session output mapping, but the runtime motion must not command a zero-velocity pause between the two segments.

## Direction And Fallback Rules

True blending requires a physically continuous 1D direction through the transition point.

If the pending position segment would reverse direction at the transition, real blend is not applied. The command falls back to the existing safe buffered behavior so the axis can settle before the reverse move.

Other fallback cases:

- active or pending segment is not `HYD_MODE_POSITION`
- active or pending segment is not finite position-ended motion
- invalid acceleration, deceleration, velocity, or tolerance values
- pending command arrives after the active segment is already complete or stopping
- Stop, Abort, Reset, or Fault occurs

Fallback must never silently produce an unsafe through-speed. It either buffers safely or rejects according to the current one-slot direct buffer contract.

## Files

Modify:

- `include/motion_planner.h`: define `HYD_MotionBlendContext` and add it to `HYD_MotionPlannerInput`.
- `src/motion_planner.c`: generalize position braking from terminal velocity zero to terminal velocity `blendVelocity`.
- `include/motion_control.h`: add compact internal blend state to `HYD_MotionControlFB`.
- `src/motion_control.c`: create/clear blend context, pass it to the planner, and cut over to pending blended segments without resetting planner state.
- `tests/test_motion_planner.c`: add direct planner tests for nonzero terminal velocity and safety caps.
- `tests/test_motion_interface_arbitration.c`: add runtime tests for four blend modes and cutover continuity.

No IEC POU surface change is required beyond the existing Beckhoff-compatible `BufferMode` work.

## Tests

Planner tests:

1. A blended front position segment near target does not decelerate to zero when `blendVelocity > 0`.
2. The blend cap follows `sqrt(vBlend^2 + 2*d*s)`.
3. Acceleration toward the through velocity remains limited by `maxAcceleration * dt`.
4. Deceleration toward the through velocity remains limited by `maxDeceleration * dt`.
5. Ordinary non-blended position moves still stop at target.

Runtime tests:

1. `BlendingLow`, `BlendingPrevious`, `BlendingNext`, and `BlendingHigh` produce distinct through-velocity constraints when previous and next velocity limits differ.
2. `MoveAbsolute -> MoveAbsolute` cutover does not clear `_plannerState.lastTargetVelocity`.
3. `Buffered` still waits for the front command to complete normally before starting the pending command.
4. Direction reversal does not perform a nonzero through blend.
5. Stop/Abort/Reset/Fault clears the blend context and pending slot.

Full verification:

```bash
cmake --build build -j2
./build/test_motion_planner
./build/test_motion_interface_arbitration
./build/test_motion_interface_unit
python3 tests/test_interface_layout_consistency.py
ctest --test-dir build --output-on-failure
```

## Acceptance Criteria

The implementation is accepted when:

- Four Beckhoff blend modes are observably distinct for finite `MoveAbsolute -> MoveAbsolute`.
- A blended front segment uses nonzero terminal velocity at the transition point.
- The following segment starts without resetting planner state.
- Unsupported command combinations retain safe existing behavior.
- All existing buffer-live-update, direct arbitration, stop/reset/fault, and planner tests pass.
- Full CTest passes.

## Explicit Non-Goals

- Multi-segment look-ahead.
- More than one pending direct command.
- Jerk-limited or S-curve planning.
- Multi-axis path geometry.
- Real blend behavior for endless `MoveVelocity` or no-duration `PressureHandle`.
- IEC surface additions beyond the current `BufferMode` and `ContinuousUpdate` contract.
