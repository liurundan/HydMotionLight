# MoveAbsolute + Stop Integration Design

Date: 2026-05-12
Branch: `feat-moveabsolute-stop-integration`
Scope: Direct command session unification for `MoveAbsolute` and `Stop`

## Goal

Stabilize the integrated use of `MoveAbsolute` and `Stop` for the hydraulic motion library so that:

- `MoveAbsolute` can start a position move in direct mode.
- `Stop` can preempt an active `MoveAbsolute` command.
- `Stop.DONE` becomes `1` only after controlled deceleration reaches zero velocity.
- The preempted `MoveAbsolute` reports `COMMANDABORTED = 1`.
- After both FBs see `EXECUTE = 0`, their latched outputs clear and the next cycle can start a new motion/stop loop cleanly.

This design keeps the library aligned with PLCopen FB semantics, IEC scan-cycle expectations, and the existing explicit simulation publish model.

## Confirmed Constraints

- `Stop` preempting `MoveAbsolute` must produce `MoveAbsolute.COMMANDABORTED = 1`.
- `USE_SIMULATION = true` will continue to require explicit `__HydMotion_framework_Publish()` calls.
- `Stop.DONE` must not become `1` on the trigger cycle during active motion.
- The implementation focus is the direct-command path first, not recipe/multi-segment process orchestration.

## Problem Summary

The current behavior is spread across core and IEC layers:

- core stop state is partially represented by `_isStopping`
- IEC output mapping is split across per-FB code paths
- command ownership is inferred indirectly from shared FB state and `_executionId`
- simulation progression depends on explicit `Publish()`, but some existing assumptions treat `Stop` as if it can complete immediately

This creates three unstable outcomes:

1. `Stop.DONE` can be evaluated too early.
2. The preempted `MoveAbsolute` may be judged from shared axis state instead of its own command ownership.
3. Existing integration logic can mix command lifecycle with simulation-time progression.

## Recommended Approach

Adopt a unified direct-command session model in the core, starting with `MoveAbsolute` and `Stop`.

Why this approach:

- It makes command ownership explicit.
- It moves `Stop` completion semantics into the core instead of relying on IEC-side guesses.
- It preserves the existing explicit `Publish()` simulation contract.
- It gives a clean path to later unify `MoveVelocity` and `PressureHandle` without redesigning the public FB contract again.

## Simplicity and Extensibility Guardrails

This design is only acceptable if it remains simple enough to reason about in one pass and narrow enough to extend without rewriting the stop path again.

Guardrails:

- reuse the existing `_executionId` concept instead of inventing a second command identity mechanism
- add only the minimum direct-session metadata needed for ownership, session kind, session state, and preemption reason
- keep the core as the single source of truth for session lifecycle; IEC code may map outputs, but may not reconstruct lifecycle by combining shared FB fields
- do not push direct-session semantics down into `motion_planner.c`; planning remains a service to the owning session
- keep `USE_SIMULATION` explicit and external to command lifecycle; publish drives time, commands drive ownership
- structure the first pass so that `MoveVelocity` and `PressureHandle` can reuse the same ownership query path later without requiring a second architecture rewrite

## Alternative Approaches Considered

### 1. Minimal patch

Patch `__mcl_cmd_Stop`, `__mcl_cmd_MoveAbsolute`, and the `_isStopping` path only.

Trade-off:

- small and low-risk
- leaves ownership and terminal-state semantics fragmented
- likely to regress when additional direct FBs are aligned later

### 2. Mid-size stop-session cleanup

Keep the current architecture but formalize stop behavior a bit more.

Trade-off:

- better than a one-off fix
- still leaves direct-command lifecycle knowledge split across layers

### 3. Unified direct-command session model

Introduce a consistent ownership model and explicit session-state interpretation in the core.

Trade-off:

- more design work now
- best long-term clarity and least ambiguity for PLCopen mappings

This spec chooses option 3.

## Core Session Model

### Session ownership

At any moment, one axis may have at most one active direct-command owner.

Each direct command session has:

- command kind
- owner execution id
- session state
- completion/preemption reason

The first implementation pass only needs this model to be authoritative for `MoveAbsolute` and `Stop`.

### Session kinds

- `MOVE_ABSOLUTE`
- `STOP`

Future-compatible kinds may later include `MOVE_VELOCITY` and `PRESSURE_HANDLE`, but they are outside this implementation pass.

### Session states

- `IDLE`
- `RUNNING`
- `STOPPING`
- `DONE`
- `ABORTED`
- `FAULT`

### Ownership rules

- `MoveAbsolute` rising edge creates a motion session and becomes the direct-command owner.
- `Stop` rising edge creates a stop session and preempts the current motion owner.
- A preempted motion session transitions to `ABORTED`.
- `Stop` transitions to `DONE` only when the controlled deceleration target has reached zero and, after publish, `AXIS_REF.velocity` is within the stop-zero tolerance.

## PLCopen/IEC Semantics

### `MoveAbsolute` when running normally

- `BUSY = 1`
- `ACTIVE = 1`
- `DONE = 0`
- `COMMANDABORTED = 0`

### `MoveAbsolute` after preemption by `Stop`

- `BUSY = 0`
- `ACTIVE = 0`
- `DONE = 0`
- `COMMANDABORTED = 1`

This is required behavior, not an optional mapping.

### `MoveAbsolute` after normal completion

- `BUSY = 0`
- `ACTIVE = 0`
- `DONE = 1`
- `COMMANDABORTED = 0`

### `Stop` after successful takeover

- `BUSY = 1`
- `DONE = 0`
- `COMMANDABORTED = 0`

### `Stop` during deceleration

- `BUSY = 1`
- `DONE = 0`

### `Stop` after zero velocity is reached

- `BUSY = 0`
- `DONE = 1`

### `Stop` on an idle axis

Immediate `DONE = 1` remains allowed, but only if there is no active direct-command owner. This path must not be reused for the active-motion stop path.

### `EXECUTE` falling edge behavior

For both FBs:

- falling edge clears the FB's latched outputs
- falling edge does not retroactively alter the core session history
- next rising edge starts a new command lifecycle

## Simulation Contract

The simulation model remains explicit.

### Required cycle model

Each logical scan uses this order:

1. IEC FBs write command inputs.
2. Command FB calls sample requests.
3. `__HydMotion_framework_Publish()` advances core execution and simulation time.
4. Next scan reads updated outputs and feedback.

### `USE_SIMULATION = true`

When simulation is enabled:

- `__HydMotion_framework_Publish()` is the only valid time-advance point
- `_simFeedback` is applied to `AXIS_REF` only during publish
- timestamp progression remains bound to publish

### Consequence for `Stop`

`Stop.DONE` is tied to deceleration over published cycles, not to the trigger scan alone.
The intended acceptance condition is:

- stop target velocity has decayed to zero in the core
- a publish cycle has applied the zeroed simulation output
- observed `AXIS_REF.velocity` is within the stop-zero tolerance

This must be explicit in tests and implementation comments because it is the main reason "immediate done" assumptions become incorrect in simulation.

## Layer Responsibilities

### Core layer

Files:

- `src/motion_control.c`

Responsibilities:

- own direct-command session state
- own direct-command takeover/preemption rules
- determine when stop deceleration is complete
- expose stable command/session facts for IEC mapping

The core must be the single source of truth for:

- current direct owner
- whether a previous owner was preempted
- whether the stop session is still decelerating
- whether the stop session has actually completed

### IEC interface layer

Files:

- `src/motion_interface.c`

Responsibilities:

- detect `EXECUTE` rising/falling edges
- submit command requests to core
- map core session results to PLCopen outputs
- clear per-FB latched outputs on `EXECUTE = 0`

The IEC layer must not infer private command terminal states by combining shared axis fields such as `FB_STATE`, `SEGMENT_COMPLETED`, and `STATE.finished` without command ownership context.

### Planner layer

Files:

- `src/motion_planner.c`

Responsibilities in this scope:

- no algorithm redesign
- preserve current trajectory/deceleration math behavior unless a session-state integration point must be adjusted

The planner is not a redesign target in this pass.

## Integration Test Matrix

### Primary loop test

Add a dedicated integration test for this exact cycle:

1. `CreateMotion(USE_SIMULATION = true)`
2. `MoveAbsolute.EXECUTE` rising edge
3. several cycles of `Publish() -> MoveAbsolute`
4. `Stop.EXECUTE` rising edge during motion
5. several cycles of `Publish() -> MoveAbsolute -> Stop`
6. verify:
   - `MoveAbsolute.COMMANDABORTED = 1`
   - `Stop.DONE = 0` until deceleration completes
   - velocity is near zero when `Stop.DONE = 1`
7. lower `MoveAbsolute.EXECUTE` and `Stop.EXECUTE`
8. run one or two cleanup cycles
9. verify:
   - `Stop.DONE = 0`
   - `MoveAbsolute.COMMANDABORTED = 0`
   - `BUSY/ACTIVE` outputs are clear
10. start a new `MoveAbsolute`
11. stop it again
12. verify the loop is repeatable

### Edge tests

- stop on idle axis: immediate `DONE`
- move queued then same-scan stop: must not incorrectly take idle-axis path
- varying `Stop.DECELERATION`: completion cycle count changes accordingly
- `EXECUTE` remains high after done: done latch may remain until falling edge
- preempted `MoveAbsolute` must never show `DONE = 1` together with `COMMANDABORTED = 1`
- simulation without `Publish()`: no time progression, no fake completion

### Existing test updates

Existing arbitration tests that assume "stop completes next cycle" must be updated to "stop completes after deceleration."

## Implementation Boundary

This design intentionally does not include:

- recipe-mode redesign
- multi-segment process sequencing
- mold/ejector/carriage/injection/plasticizing process orchestration
- changing simulation from explicit publish to implicit progression
- a full redesign of all direct commands in one pass
- planner algorithm replacement

## Expected Outcome

After implementation:

- `MoveAbsolute` and `Stop` can be used together repeatedly in a PLC cycle loop
- `Stop` completes only after real deceleration-to-zero behavior
- the preempted `MoveAbsolute` reports `COMMANDABORTED`
- output latches clear cleanly after `EXECUTE = 0`
- the same axis can be reused in the next command cycle without leaking previous session state

## Acceptance Criteria

- all existing baseline tests remain green, except tests intentionally updated for corrected stop timing semantics
- new integration tests cover the looped `MoveAbsolute -> Stop -> clear -> restart` flow
- active-motion stop no longer produces immediate done
- stop completion is tied to zero velocity in simulation
- `MoveAbsolute` preempted by `Stop` reports `COMMANDABORTED = 1`
