# HYD_MoveContinuousAbsolute Design

Date: 2026-07-02
Status: Approved for planning

Related code:
- `pousHydMotion.xml`
- `include/motion_interface.h`
- `src/motion_interface.c`
- `include/motion_control.h`
- `src/motion_control.c`
- `include/motion_planner.h`
- `src/motion_planner.c`
- `src/segment_completion.c`
- `src/output_limiter.c`

Related repo docs:
- `docs/architecture/motion-runtime-contract.md`
- `docs/superpowers/specs/2026-05-16-motion-control-continuous-command-roadmap-design.md`
- `docs/superpowers/specs/2026-05-26-pressure-limit-soft-position-limit-design.md`
- `docs/superpowers/specs/2026-06-27-direct-moveabsolute-blending-ownership-design.md`
- `docs/superpowers/specs/2026-07-01-moveabsolute-three-fb-terminal-state-design.md`

External references:
- Beckhoff general rules for MC function blocks: <https://infosys.beckhoff.com/content/1033/tcplclib_tc2_mc2/70043531.html>
- Beckhoff `MC_BufferMode`: <https://infosys.beckhoff.com/content/1033/tcplclib_tc2_mc2/70147595.html>
- Beckhoff `MC_MoveContinuousAbsolute`: <https://infosys.beckhoff.com/content/1033/tcplclib_tc2_mc2/70103947.html>

## Goal

Add a new IEC function block, `HYD_MoveContinuousAbsolute`, for absolute positioning with a maintained terminal velocity.

Unlike the existing `HYD_MoveAbsolute`, this command does not finish by converging to zero velocity at the target position. It drives to `Position`, then continues running with a programmed end velocity until another motion command takes ownership.

The target use case is continuous via-style hydraulic motion, especially open/close mold sequences where the process should pass a position boundary without stopping.

## Design summary

The approved design is:

1. Add a new dedicated IEC FB instead of extending `HYD_MoveAbsolute`.
2. Reuse the repository's direct-command ownership, ticket, pending-slot, and preemption model.
3. Implement the new command as one direct owner with two internal phases:
   - `APPROACH`: finite absolute positioning with a nonzero crossing-velocity target
   - `SUSTAIN`: endless velocity-hold after first target reach
4. Add `PressureLimit : REAL` as a new FB input.
5. Keep the runtime's existing pressure-limit chain as the only enforcement mechanism by mapping the command input into the existing `segment.maxPressure` path.

This keeps old `MoveAbsolute` behavior stable while giving the new FB a lifecycle that matches continuous hydraulic motion.

## Scope

In scope:
- New IEC FB type and XML POU entry
- New direct-command kind in runtime
- New two-phase continuous-absolute runtime behavior
- New `PressureLimit` input with default fallback
- Validation and test coverage for lifecycle, pressure-limit interaction, and overshoot-prevention behavior

Out of scope:
- Changing public semantics of `HYD_MoveAbsolute`
- Recipe-side `MoveProfile` support for continuous absolute in this round
- Jerk-limited implementation
- Multi-axis synchronization
- Full physical blending between two `HYD_MoveContinuousAbsolute` commands in the first version
- New pressure-window inputs that duplicate `pressureCeiling`

## Compatibility posture

This design is intentionally close to PLCopen / Beckhoff continuous-motion conventions, but it is not a byte-for-byte clone.

Aligned behaviors:
- rising-edge `Execute` start
- `Busy` remains true while the command is still the active lifecycle
- `CommandAborted` means takeover by another motion command
- `Execute` falling edge clears the FB-local visible state but does not stop the axis by itself
- `BufferMode` remains a second-FB command-flow feature, not an in-place retarget feature

Intentional repo-local extensions:
- add `PositionReached` output in addition to `InEndVelocity`
- define `InEndVelocity` more strictly than Beckhoff: in this repo it means "target position reached and programmed end velocity achieved", not merely "target position reached"
- omit a public `Active` output to match the existing repository's leaner IEC surfaces for some direct commands

This is best described as a PLCopen-compatible repository-specific extension, not a strict standard clone.

## Public FB contract

### Inputs

| Name | Type | Semantics |
| --- | --- | --- |
| `AXISID` | `SINT` | Axis index |
| `EXECUTE` | `BOOL` | Rising edge starts the command |
| `POSITION` | `REAL` | Absolute target position |
| `VELOCITY` | `REAL` | Maximum approach velocity magnitude |
| `ENDVELOCITY` | `REAL` | Desired sustained velocity magnitude after target reach |
| `ENDVELOCITYDIRECTION` | `SINT` | `positive`, `negative`, or `current` |
| `ACCELERATION` | `REAL` | Positive acceleration limit |
| `DECELERATION` | `REAL` | Positive deceleration limit; nonpositive falls back to `ACCELERATION` |
| `JERK` | `REAL` | Reserved for compatibility; nonzero rejected in this round |
| `DIRECTION` | `SINT` | `positive`, `negative`, `current`, or `shortest`; same meaning as `HYD_MoveAbsolute` |
| `ADAPTENDVELTOAVOIDOVERSHOOT` | `BOOL` | Enables overshoot-suppression adjustment of the crossing velocity |
| `PRESSURELIMIT` | `REAL` | Per-command maximum pressure limit; nonpositive means "not configured, use axis default" |

### Outputs

| Name | Type | Semantics |
| --- | --- | --- |
| `INENDVELOCITY` | `BOOL` | True only after target position was reached and the programmed end velocity was actually achieved |
| `POSITIONREACHED` | `BOOL` | True from the first target-position reach onward |
| `BUSY` | `BOOL` | True from acceptance until takeover, error, or local reset |
| `COMMANDABORTED` | `BOOL` | True when this FB loses ownership to another motion command |
| `ERROR` | `BOOL` | True on start rejection or runtime error surfaced to this FB |
| `ERRORID` | `WORD` | Repository diagnostic code |

### No `Done`

`HYD_MoveContinuousAbsolute` does not expose `Done`.

The normal success state is not a terminal completion. The intended steady state is:

- `Busy = TRUE`
- `PositionReached = TRUE`
- `InEndVelocity = TRUE`

and it remains there until another motion command interrupts it.

## Output semantics

### `Busy`

`Busy` becomes true as soon as the command is accepted. It stays true during both internal phases:

- `APPROACH`
- `SUSTAIN`

It becomes false only when:
- another motion command takes ownership
- the runtime faults this command
- the command is rejected on start
- `Execute` falls and the FB-local visible state is cleared

### `PositionReached`

`PositionReached` means the axis has reached the programmed `Position` for the first time under the command's target-reach rule.

It is latched true for the rest of this FB lifecycle. It does not drop back to false just because the axis keeps moving through the target during the sustain phase.

### `InEndVelocity`

`InEndVelocity` is stricter than Beckhoff `MC_MoveContinuousAbsolute`.

In this repository it means:

1. the target position has been reached
2. the actual motion has reached the programmed end velocity that the command intends to sustain

This output is also latched true for the rest of the FB lifecycle.

If pressure limiting or later decel/accel shaping prevents the axis from ever reaching the programmed end velocity, `InEndVelocity` remains false even if `PositionReached` is already true.

### `CommandAborted`

`CommandAborted` is the takeover output:
- another `MoveAbsolute`
- another `MoveContinuousAbsolute`
- `MoveVelocity`
- `PressureHandle`
- `Stop`
- reset-like ownership invalidation paths already treated as command takeover in the existing direct-command model

Once takeover is detected:
- `CommandAborted = TRUE`
- `Busy = FALSE`
- `PositionReached = FALSE`
- `InEndVelocity = FALSE`

### `Error`

`Error` covers:
- invalid axis
- unsupported direction enum for `EndVelocityDirection`
- nonfinite or nonpositive motion parameters where positive values are required
- unsupported nonzero `Jerk`
- runtime start rejection
- runtime fault or protection escalation surfaced as a command error

Once `Error` is set:
- `Busy = FALSE`
- `CommandAborted = FALSE`

## Execution model

### New direct-command kind

Add a new direct-command kind:

- `HYD_DIRECT_CMD_MOVE_CONTINUOUS_ABSOLUTE`

This command participates in the same direct-command arbitration framework as the existing direct FBs:
- one active owner
- one pending follower
- accepted ticket identity
- preempted history
- completed record where applicable

### One owner, two internal phases

The command is not modeled as a normal one-shot position segment that completes and returns idle. Instead it is one direct owner with two internal phases.

#### Phase 1: `APPROACH`

The runtime executes a finite position-mode segment toward `Position`.

This phase uses:
- the requested maximum velocity
- the resolved move direction
- the computed crossing velocity target
- the configured acceleration/deceleration limits

#### Phase 2: `SUSTAIN`

At the first target reach:
- `PositionReached` is latched
- ownership does not end
- the runtime immediately switches the same direct owner into an endless speed-ramp segment

This phase drives the axis toward the programmed sustained end velocity and then holds it until takeover.

No completed direct ticket is recorded when the command enters `SUSTAIN`, because the command is still logically running.

## Motion model

### Two internal velocity concepts

The runtime must distinguish:

- `CrossingVelocity`: the target velocity at first target reach
- `SustainVelocity`: the final velocity that should be maintained after target reach

These are often equal, but they are not always equal when overshoot suppression is enabled or when the sustain direction reverses after target reach.

### Direction resolution

#### Approach direction

`DIRECTION` follows the repository's current `MoveAbsolute` contract:
- `positive`
- `negative`
- `current`
- `shortest`

The approach direction is derived from the target-position delta (`Position - CurrentPosition`) after applying the existing repository direction-resolution rules. It is not inferred from current velocity sign or acceleration sign.

#### Sustain direction

`ENDVELOCITYDIRECTION` accepts only:
- `positive`
- `negative`
- `current`

Unsupported values are rejected.

`current` resolves in this order:
1. current measured motion direction if actual velocity is above a small direction threshold
2. runtime `_lastActiveDirection`
3. approach direction fallback

### Sustain velocity

Let:
- `sustainMagnitude = abs(ENDVELOCITY)`
- `sustainDirection = resolved(ENDVELOCITYDIRECTION)`

Then:
- `SustainVelocity = sign(sustainDirection) * sustainMagnitude`

`SustainVelocity` represents the user's programmed post-target intent. Pressure limiting does not rewrite this programmed value.

### Crossing velocity without overshoot adaptation

If `ADAPTENDVELTOAVOIDOVERSHOOT = FALSE`:

- if sustain direction matches approach direction:
  - `CrossingVelocity = abs(SustainVelocity)` in the approach direction
- if sustain direction opposes approach direction:
  - `CrossingVelocity = 0`
  - after target reach, the sustain phase accelerates in the opposite direction

This means the repository intentionally resolves the physically inconsistent case conservatively. The command does not overshoot the target just to establish reverse-direction end velocity at the first reach instant.

### Crossing velocity with overshoot adaptation

If `ADAPTENDVELTOAVOIDOVERSHOOT = TRUE`, the runtime may modify `CrossingVelocity` to avoid positional overshoot.

Definitions:
- `d = abs(Position - CurrentPosition)` along the commanded approach direction
- `v0 = current projected speed magnitude along the approach direction`
- `a_up = Acceleration`
- `a_down = resolved deceleration`
- `v_req = abs(SustainVelocity)`

Rules:

1. sustain direction opposite to approach direction:
   - `CrossingVelocity = 0`

2. sustain direction same as approach direction and `v0 < v_req`:
   - if the remaining distance is too short to raise velocity smoothly to `v_req`, reduce crossing velocity to the reachable value

3. sustain direction same as approach direction and `v0 > v_req`:
   - if the remaining distance is too short to decelerate smoothly to `v_req`, raise crossing velocity to the reachable floor

This preserves the approved interpretation:
- too little travel to accelerate -> lower the actual crossing velocity
- too little travel to decelerate -> raise the actual crossing velocity
- reverse sustain direction -> zero crossing velocity

### Phase switch and signal timing

At first target reach:
- `PositionReached` is latched true
- the runtime transitions from `APPROACH` to `SUSTAIN`

Then:
- if `CrossingVelocity == SustainVelocity`, `PositionReached` and `InEndVelocity` may become true in the same cycle
- if they differ, `PositionReached` may become true first and `InEndVelocity` follows only after sustain velocity is reached

This is the approved explanation for both:
- `AdaptEndVelToAvoidOvershoot = TRUE`
- reverse-direction sustain after target reach

## Parameter validation

Validation follows the existing direct-command style and adds explicit rules for the new inputs.

### Required positive or finite rules

- invalid `AXISID` -> `HYD_DIAG_CODE_START_CONTEXT_INVALID`
- `VELOCITY`:
  - internal magnitude is `abs(VELOCITY)`
  - must be finite and greater than zero
- `ENDVELOCITY`:
  - internal magnitude is `abs(ENDVELOCITY)`
  - must be finite
  - zero is allowed
- `ACCELERATION`:
  - must be finite and greater than zero
- `DECELERATION`:
  - if positive, use it
  - otherwise fall back to `ACCELERATION`
- `JERK`:
  - nonzero rejected in this round
- `ENDVELOCITYDIRECTION`:
  - only `positive`, `negative`, and `current` accepted
  - `shortest`, `fastest`, and out-of-range values are rejected
- `PRESSURELIMIT`:
  - must be finite
  - positive means explicitly configured
  - zero or negative means "not configured for this command"

## Pressure-limit contract

### Default fallback

The new `PRESSURELIMIT` input behaves as follows:

- `PRESSURELIMIT > 0`:
  - explicit per-command pressure limit for this invocation
- `PRESSURELIMIT <= 0`:
  - not configured for this invocation
  - use the axis default stored in `fb.PRESSURE_LIMIT`

The input does not write back to axis defaults.

### Alignment with existing pressure-limit path

The new input does not create a new protection subsystem.

Instead:
- the command resolves one per-command max-pressure value
- that value is written into the internal continuous-absolute segment's `maxPressure`
- the runtime continues to use the existing `segment.maxPressure` plus `fb.PRESSURE_LIMIT` merge logic

Therefore the effective limit is:

- the minimum nonzero value among:
  - command input `PressureLimit` if configured
  - axis default `fb.PRESSURE_LIMIT` if configured
  - any per-phase segment max-pressure field used by the command

In practice, both internal phases use the same resolved command pressure limit so that the constraint applies for the full lifecycle.

### No mapping to `pressureCeiling`

`PressureLimit` maps only to the repository's full-path maximum pressure guard (`maxPressure` path).

It does not map to:
- `pressureCeiling`
- `pressureCeilingPositionStart`
- `pressureCeilingPositionEnd`

Reason:
- `maxPressure` is a whole-command pressure envelope
- `pressureCeiling` is a position-window mold-protect primitive

Keeping them separate avoids semantic overlap and preserves the current runtime layering.

### Interaction with outputs

When pressure limiting is active:
- `Busy` stays true unless the protection escalates to a runtime fault
- `PositionReached` may still become true
- `InEndVelocity` becomes true only if the actual sustained speed truly reaches the programmed target

If the pressure-limit path escalates to fault and runtime stop:
- `Error = TRUE`
- `Busy = FALSE`
- `CommandAborted = FALSE`

## IEC adapter responsibilities

`__mcl_cmd_MoveContinuousAbsolute()` in `src/motion_interface.c` should:

1. detect rising edge
2. validate public inputs
3. resolve direction enums
4. normalize magnitudes
5. resolve the per-command pressure-limit value
6. build a continuous-absolute start request
7. start or queue the direct command
8. map runtime phase and ticket state back to the IEC outputs

The IEC adapter should not own the detailed motion math. It should not compute overshoot-safe crossing velocity itself.

## Core runtime responsibilities

The runtime owns:
- phase tracking
- crossing-velocity computation
- sustain-velocity tracking
- pressure-limit propagation into both internal phases
- target-reach latching
- end-velocity latching
- takeover and fault handling

The minimum private context that should exist on `HYD_MotionControlFB` for this command is:
- active/inactive
- owner ticket
- current phase
- programmed sustain velocity
- resolved crossing velocity
- resolved effective pressure limit
- `positionReachedLatched`
- `inEndVelocityLatched`
- `adaptEndVelEnabled`

The exact storage layout may vary, but these facts must exist somewhere stable in runtime state.

## BufferMode and queueing

The command participates in the existing direct owner + one pending slot contract.

First-version rules:
- `ABORT` -> immediate takeover
- `BUFFER` -> allowed when the single pending slot is free
- `BLENDING_*` -> accepted only as plain buffered command-flow behavior in the first version unless the runtime explicitly proves a safe physical transition path

This means the first version does not promise full motion-profile blending between two `HYD_MoveContinuousAbsolute` commands. It only promises correct queueing and takeover semantics.

This is intentionally conservative because the command already has an internal two-phase motion model, and command-to-command continuous blending would significantly widen the state surface.

## Testing and acceptance

### A. Start and validation

- valid start succeeds
- invalid axis rejects with start-context error
- unsupported nonzero jerk rejected
- unsupported `EndVelocityDirection` rejected
- `PressureLimit <= 0` falls back to axis default

### B. Same-direction continuous motion

- command reaches target position
- `PositionReached` and `InEndVelocity` can assert in the same cycle when crossing and sustain velocity are equal
- `Busy` remains true after target reach
- actual target velocity does not collapse to zero at the transition

### C. Overshoot adaptation

- no-adaptation same-direction path preserves requested crossing target
- adaptation lowers crossing velocity when travel is too short to accelerate enough
- adaptation raises crossing velocity when travel is too short to decelerate enough
- reverse sustain direction with adaptation forces zero crossing velocity

### D. Reverse sustain

- target position is reached first
- command then accelerates in the opposite direction during sustain
- `PositionReached` asserts before `InEndVelocity`

### E. Pressure-limit interaction

- warning/derate does not clear `Busy`
- `PositionReached = TRUE` while `InEndVelocity = FALSE` is possible under limiting
- fault escalation surfaces as `Error` and clears `Busy`

### F. Ownership and takeover

- preemption by `Stop`
- preemption by `MoveAbsolute`
- preemption by `MoveVelocity`
- preemption by `PressureHandle`
- preemption by another `MoveContinuousAbsolute`
- `Execute = FALSE` clears this FB's visible outputs without actively stopping the axis

### G. Queueing

- one active + one pending works
- third same-axis direct command is rejected when queue is full
- buffered follower activates correctly after front command transition or takeover

## Rejected alternatives

### Rejected: extend `HYD_MoveAbsolute`

Rejected because current `MoveAbsolute` semantics, completion logic, and tests assume target reach with near-zero velocity. Mixing the two behaviors into one FB would create unnecessary regression risk.

### Rejected: treat the command as ordinary `MoveAbsolute` completion followed by user-side `MoveVelocity`

Rejected because the new FB must preserve one command lifecycle, one owner contract, and one set of outputs across the position-to-sustain transition.

### Rejected: map `PressureLimit` to `pressureCeiling`

Rejected because `pressureCeiling` is windowed mold-protect logic, not a full-lifecycle maximum pressure envelope.

## Implementation impact

Expected files to change in the implementation phase:
- `pousHydMotion.xml`
- `include/motion_interface.h`
- `src/motion_interface.c`
- `include/motion_control.h`
- `src/motion_control.c`
- `include/motion_planner.h`
- `src/motion_planner.c`
- `src/segment_completion.c`
- relevant unit and integration tests under `tests/`

The implementation phase should preserve all existing `HYD_MoveAbsolute` tests and add focused new tests for the new command.
