# Reset + Recipe/Direct Boundary Design

Date: 2026-05-13
Branch: `feat-moveabsolute-stop-integration`
Scope: Unify `Reset` semantics and recipe/direct ownership boundaries under a thin axis-execution dispatcher model

## Goal

Make `Reset` and recipe/direct command mixing follow one consistent ownership model so that:

- one axis has exactly one active execution source at a time
- one axis has exactly one active owner at a time
- preemption between recipe and direct commands is reported consistently
- `Reset` has a single, explicit system-level meaning
- `DONE`, `COMMANDABORTED`, `ERROR`, and in-band signals are derived from one set of state-transition rules

## Confirmed Constraints

- The user chose the thin-dispatcher architecture direction.
- The implementation should stay simple and avoid introducing a second large state machine.
- `SoftReset` should continue to preserve loaded configuration (`RECIPE`, `DIRECT_SEGMENT`, parameters) while clearing execution/runtime state.
- `Stop` remains a direct-command owner, not the global system preemption command.
- Recipe-mode and direct-mode execution may both remain configured in memory, but not active simultaneously.

## Recommended Approach

Introduce a thin axis-dispatcher model in the core with two explicit dimensions:

1. execution source
2. active owner

Then derive all IEC output semantics from owner transitions, not from ad hoc combinations of `FB_STATE`, `USE_RECIPE`, `_executionId`, and command-local assumptions.

This keeps the design small while solving both:

- `Reset` behavior
- recipe/direct mixed-command ownership

## Alternative Approaches Considered

### 1. IEC-only patching

Keep the core mostly as-is and keep adding special-case logic in `motion_interface.c`.

Trade-off:

- smaller immediate diff
- ownership rules remain fragmented
- high long-term ambiguity

### 2. Thin dispatcher in the core

Add a small owner/source abstraction in `motion_control.c` and let IEC map outputs from it.

Trade-off:

- modest core changes
- much clearer semantics
- easiest to extend without making the codebase heavier

### 3. Full scheduler module extraction

Create a dedicated runtime scheduler module for all recipe/direct/system ownership.

Trade-off:

- clean long-term direction
- too large for this phase

This spec chooses option 2.

## Dispatcher Model

### Execution source

The axis-level execution source must be explicit:

- `HYD_EXEC_SRC_NONE`
- `HYD_EXEC_SRC_RECIPE`
- `HYD_EXEC_SRC_DIRECT`
- `HYD_EXEC_SRC_SYSTEM`

Meaning:

- `NONE`: no active execution owns the axis
- `RECIPE`: a recipe-driven command currently owns execution
- `DIRECT`: a direct command currently owns execution
- `SYSTEM`: a system-level command such as `Reset` is currently taking control

### Owner kinds

The owner kind must also be explicit:

- `HYD_OWNER_NONE`
- `HYD_OWNER_MOVE_PROFILE`
- `HYD_OWNER_MOVE_ABSOLUTE`
- `HYD_OWNER_MOVE_VELOCITY`
- `HYD_OWNER_PRESSURE_HANDLE`
- `HYD_OWNER_STOP`
- `HYD_OWNER_RESET`

This owner model replaces the need for each IEC FB to infer “who displaced me” from global side effects.

## Ownership Rules

### General rule

At any moment:

- one axis has at most one execution source
- one axis has at most one active owner

### Direct command takeover

If a direct command is accepted while another direct command is active:

- old owner loses ownership
- old owner is recorded as preempted
- execution source remains `DIRECT`
- new owner becomes the accepted direct command

### Direct takes over recipe

If a direct command is accepted while recipe execution is active:

- old owner (`MOVE_PROFILE`) loses ownership
- execution source switches `RECIPE -> DIRECT`
- `MOVE_PROFILE` is reported as preempted

### Recipe takes over direct

If `MoveProfile` is accepted while a direct command is active:

- old direct owner loses ownership
- execution source switches `DIRECT -> RECIPE`
- the direct owner is reported as preempted

### Reset takes over anything

If `Reset` is accepted while any owner is active:

- old owner loses ownership immediately
- execution source switches to `SYSTEM` for the reset transition
- reset clears runtime ownership/execution state
- after reset completes, execution source returns to `NONE`

`Reset` does not wait for normal stop, segment completion, or recipe completion.

## Reset Semantics

### What Reset clears

`SoftReset` should clear:

- active owner
- execution source
- pending command
- pending ownership transition
- controller runtime state
- active execution latches
- live fault/execution state

### What Reset preserves

`SoftReset` should preserve:

- `RECIPE`
- `RECIPE_SIZE`
- `DIRECT_SEGMENT`
- `DIRECT_SEGMENT_VALID`
- parameter defaults/tuning
- mode-selection configuration such as `USE_RECIPE`

This keeps the axis configured but not executing.

### Reset output behavior

`Reset` is a system command with immediate terminal behavior:

- accepted reset → `DONE = 1`
- it does not expose a long-running active phase like motion commands
- it clears the previous owner’s lifecycle in a single ownership transition

## Unified Output Semantics

### Global rule

- `DONE` means normal completion for the current command’s intended lifecycle
- `COMMANDABORTED` means owner was displaced by another accepted owner
- `ERROR` means the command faulted while still owning execution
- in-band signals (`INVELOCITY`, `INPRESSURE`) are valid only while the command still owns execution

### MoveProfile

- normal recipe completion → `DONE = 1`
- displaced by direct command or reset → `COMMANDABORTED = 1`
- fault while owning recipe execution → `ERROR = 1`

### Direct commands

- `MoveAbsolute`: normal position completion → `DONE = 1`
- `MoveVelocity`: generally sustained, so `DONE` is not the common terminal path
- `PressureHandle`: duration-limited path may `DONE`; hold-like path may remain active with only `INPRESSURE`
- `Stop`: `DONE` only after deceleration-to-zero lifecycle completes

For all direct commands:

- ownership loss → `COMMANDABORTED = 1`
- fault while still owner → `ERROR = 1`
- `Reset` takeover counts as ownership loss

### Reset

- accepted reset → `DONE = 1`
- no prolonged running lifecycle
- system-level clear of previous owner state

## IEC Mapping Responsibilities

### `motion_interface.c`

The IEC layer should:

- detect command edges
- submit command requests
- query execution source + owner + preemption facts from the core
- map outputs from those facts

It should not:

- infer ownership from `USE_RECIPE` + `FB_STATE` alone
- infer preemption from bare `_executionId` mismatch alone
- encode different reset/preemption semantics per FB

### Mapping order

For all IEC FBs, output mapping should use a consistent order:

1. invalid input / early parameter error
2. `EXECUTE = 0` latch clearing
3. pending ownership acquisition
4. explicit preemption fact
5. current owner match
6. fault while still current owner
7. fallback ownership loss / no-owner state

## Core Layer Responsibilities

### `motion_control.c`

The core should hold:

- current execution source
- current owner kind
- current owner execution id
- last preempted owner kind / execution id
- reset transition semantics

It should not:

- absorb IEC latch behavior
- duplicate per-command UI/output policy

## Testing Strategy

### 1. Reset on active direct owner

Verify:

- `MoveAbsolute`, `MoveVelocity`, `PressureHandle`, and `Stop` all lose ownership under reset
- previous owner reports `COMMANDABORTED = 1`
- reset reports `DONE = 1`
- runtime ownership state is cleared

### 2. Reset on active recipe owner

Verify:

- `MoveProfile` running recipe is preempted by reset
- recipe command reports `COMMANDABORTED = 1`
- recipe configuration remains loaded after reset
- axis returns to configured-but-idle state

### 3. Recipe/direct cross-preemption

Verify:

- direct command can take over recipe execution
- `MoveProfile` gets `COMMANDABORTED = 1`
- recipe command can take over active direct execution
- displaced direct owner gets `COMMANDABORTED = 1`

### 4. Configuration retention after reset

Verify:

- `RECIPE_SIZE` still present after reset
- `DIRECT_SEGMENT_VALID` still reflects the stored segment after reset
- parameters remain preserved
- no active execution state survives

### 5. Regression protection

Keep green:

- `MoveAbsolute + Stop` integration
- direct-session `MoveVelocity` / `PressureHandle` tests
- current arbitration and unit suites

## Files Expected to Change

- `include/motion_control.h`
- `src/motion_control.c`
- `src/motion_interface.c`
- `tests/test_motion_interface_unit.c`
- `tests/test_motion_interface_done_signals.c`
- `tests/test_motion_interface_arbitration.c`

If recipe/direct mixed-boundary scenarios become too crowded in existing tests, adding one focused integration test is acceptable, but reuse of current suites is preferred first.

## Non-Goals

- no planner rewrite
- no simulation-driver redesign
- no process-layer mold/ejector/carriage/injection sequencing
- no large scheduler-module extraction

## Acceptance Criteria

- `Reset` has one explicit meaning across recipe and direct execution
- recipe/direct takeover rules are symmetric and testable
- previous owners consistently report `COMMANDABORTED`
- `SoftReset` preserves configuration but clears execution state
- direct and recipe commands no longer rely on mixed implicit ownership heuristics
- full CTest suite remains green after implementation
