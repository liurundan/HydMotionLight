# MoveVelocity + PressureHandle Direct Session Design

Date: 2026-05-13
Branch: `feat-moveabsolute-stop-integration`
Scope: Extend direct-command session semantics from `MoveAbsolute + Stop` to `MoveVelocity` and `PressureHandle`

## Goal

Extend the existing direct-command session architecture so that `MoveVelocity` and `PressureHandle` use the same ownership, preemption, terminal-state, and latch-clearing model already established for `MoveAbsolute + Stop`.

After this extension:

- all direct commands share one owner model
- all direct-command preemptions report `COMMANDABORTED` consistently
- `DONE`, `INVELOCITY`, and `INPRESSURE` are derived from a unified owner/session model
- `Stop` can take over any direct command using the same lifecycle semantics

## Confirmed Constraints

- This round must extend both preemption semantics and completion semantics.
- `MoveVelocity` and `PressureHandle` should be unified into the same direct-session model, not handled by a parallel IEC-only workaround.
- The explicit `__HydMotion_framework_Publish()` simulation contract remains unchanged.
- Recipe-mode and process-layer orchestration remain out of scope.

## Recommended Approach

Use the existing direct-session architecture and extend it minimally:

- add direct owner kinds for `MOVE_VELOCITY` and `PRESSURE_HANDLE`
- set owner kind in the core when those direct segments start
- keep session state vocabulary unchanged
- update IEC mapping for `MoveVelocity` and `PressureHandle` to use direct-session facts just like `MoveAbsolute`

This keeps the architecture cohesive and avoids adding a second command-lifecycle path inside `motion_interface.c`.

## Alternative Approaches Considered

### 1. IEC-only unification

Keep the core mostly unchanged and rewrite `MoveVelocity` / `PressureHandle` outputs only in `motion_interface.c`.

Trade-off:

- lower immediate code churn
- pushes more lifecycle guessing into IEC mapping
- drifts away from the single-source-of-truth architecture already established

### 2. Extend current direct-session model

Reuse the current core session metadata and expand owner kinds and mappings.

Trade-off:

- moderate code change
- preserves architecture
- easiest to reason about and extend further

### 3. Full direct-session extraction

Create a dedicated core submodule for direct-command lifecycle.

Trade-off:

- clean long-term direction
- too large for this phase

This spec chooses option 2.

## Session Model Extension

### Direct command kinds

The core direct owner kinds must become:

- `HYD_DIRECT_CMD_NONE`
- `HYD_DIRECT_CMD_MOVE_ABSOLUTE`
- `HYD_DIRECT_CMD_MOVE_VELOCITY`
- `HYD_DIRECT_CMD_PRESSURE_HANDLE`
- `HYD_DIRECT_CMD_STOP`

### Session states

No new session states are needed. Reuse:

- `IDLE`
- `RUNNING`
- `STOPPING`
- `DONE`
- `ABORTED`
- `FAULT`

### Ownership rules

- starting a direct velocity segment sets owner kind to `MOVE_VELOCITY`
- starting a direct pressure segment sets owner kind to `PRESSURE_HANDLE`
- any new direct command can preempt the current direct owner
- `Stop` preempting any direct owner uses the same stop takeover lifecycle
- recipe execution must not be misreported as a direct owner

## Completion Semantics

### Core rule

Separate:

1. command lifecycle terminal state
2. in-band target acquisition state

`DONE` is a lifecycle terminal signal.
`INVELOCITY` and `INPRESSURE` are in-band acquisition signals.

### MoveVelocity

`MoveVelocity` remains a sustained command by default.

Meaning:

- `DONE` is generally not the normal terminal path for an unconstrained velocity hold
- `INVELOCITY` means the active owner is `MOVE_VELOCITY` and actual velocity is inside the accepted band
- if ownership is lost, `INVELOCITY` must clear immediately
- if the command is preempted, `COMMANDABORTED = 1`, `DONE = 0`, `INVELOCITY = 0`
- if the command faults, `ERROR = 1`, `DONE = 0`, `INVELOCITY = 0`

### PressureHandle

`PressureHandle` has two semantic modes:

- duration-limited command: may naturally reach `DONE`
- persistent/hold-like command: may remain in `RUNNING` while only asserting `INPRESSURE`

For this implementation round, preserve current duration-based completion behavior:

- `DONE` is allowed only for the duration-limited path when the owner is still `PRESSURE_HANDLE`
- `INPRESSURE` means the active owner is `PRESSURE_HANDLE` and the actual pressure is inside the accepted band
- if ownership is lost, `INPRESSURE` must clear immediately
- if preempted, `COMMANDABORTED = 1`, `DONE = 0`, `INPRESSURE = 0`
- if faulted, `ERROR = 1`, `DONE = 0`, `INPRESSURE = 0`

## IEC Output Mapping

### MoveVelocity

IEC mapping must be driven in this order:

1. if the execution was preempted as `MOVE_VELOCITY`, report:
   - `COMMANDABORTED = 1`
   - `BUSY = 0`
   - `ACTIVE = 0`
   - `INVELOCITY = 0`
2. else if the owner is still this `MOVE_VELOCITY` execution:
   - if faulted: `ERROR = 1`
   - else: `BUSY = 1`, `ACTIVE = 1`, and compute `INVELOCITY`
3. else if ownership was lost to another command, report `COMMANDABORTED = 1`

### PressureHandle

IEC mapping must be driven in this order:

1. if preempted as `PRESSURE_HANDLE`, report:
   - `COMMANDABORTED = 1`
   - `BUSY = 0`
   - `ACTIVE = 0`
   - `INPRESSURE = 0`
2. else if still owned by this `PRESSURE_HANDLE` execution:
   - if faulted: `ERROR = 1`
   - else if duration-limited and complete: normal terminal completion
   - else: `BUSY = 1`, `ACTIVE = 1`, and compute `INPRESSURE`
3. else if ownership was lost, report `COMMANDABORTED = 1`

### Latch clearing

For both FBs:

- `EXECUTE = 0` clears latched outputs
- latch clearing does not alter core command history
- retrigger must behave like a new lifecycle

## Core Layer Responsibilities

### `motion_control.c`

Add only the minimum new core semantics:

- infer direct owner kind for velocity segments
- infer direct owner kind for pressure segments
- keep recipe segments outside direct-owner reporting
- preserve current stop lifecycle
- preserve current direct-session state transitions

The core should not gain a second stop path or a second owner model.

### `motion_interface.c`

Refactor only the IEC mapping for:

- `__mcl_cmd_MoveVelocity`
- `__mcl_cmd_PressureHandle`

These should follow the same mapping strategy now used by `MoveAbsolute`.

## Testing Strategy

### 1. MoveVelocity lifecycle tests

Verify:

- owner acquisition
- `INVELOCITY` assertion during sustained running
- `Stop` takeover yields:
  - no immediate `DONE`
  - multi-cycle stop completion
  - `COMMANDABORTED = 1`
  - `INVELOCITY = 0`
- retrigger works after latch clearing

### 2. PressureHandle lifecycle tests

Verify:

- owner acquisition
- `INPRESSURE` assertion while pressure is in band
- duration-limited completion still produces `DONE`
- `Stop` takeover yields:
  - `COMMANDABORTED = 1`
  - `INPRESSURE = 0`
  - stop still completes through the same multi-cycle stop lifecycle

### 3. Direct-command preemption matrix

Verify at least:

- `MoveVelocity -> MoveAbsolute`
- `MoveVelocity -> PressureHandle`
- `PressureHandle -> MoveVelocity`
- `PressureHandle -> Stop`

For each:

- old owner gets `COMMANDABORTED = 1`
- old owner’s in-band signal clears
- new owner does not inherit stale done/band signals

### 4. Regression coverage

Keep and preserve:

- `MoveAbsolute + Stop` dedicated integration test
- arbitration tests for direct-command takeover
- owned-fault regression for `MoveAbsolute`
- multi-axis isolation coverage

## Files Expected to Change

- `include/motion_control.h`
- `src/motion_control.c`
- `src/motion_interface.c`
- `tests/test_motion_interface_done_signals.c`
- `tests/test_motion_interface_arbitration.c`

A new dedicated integration test is optional for this phase. The preferred path is to reuse and strengthen the existing done-signal and arbitration suites unless a gap remains that cannot be expressed cleanly there.

## Non-Goals

- no recipe-mode redesign
- no process-layer sequencing changes
- no simulation-driver model change
- no planner algorithm change
- no full module extraction of direct-session logic

## Acceptance Criteria

- `MoveVelocity` and `PressureHandle` use the same direct owner/preemption model as `MoveAbsolute`
- `Stop` can preempt either command with the same multi-cycle stop completion semantics
- `INVELOCITY` and `INPRESSURE` clear immediately when ownership is lost
- `COMMANDABORTED` is reported consistently from core preemption facts
- current `MoveAbsolute + Stop` behavior remains green
- full CTest suite remains green after the extension
