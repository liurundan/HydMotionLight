# MoveAbsolute Third-Command Buffer-Full Design

Date: 2026-06-30

Related specs:
- `docs/superpowers/specs/2026-06-27-direct-moveabsolute-blending-ownership-design.md`
- `docs/superpowers/specs/2026-06-29-moveabsolute-buffered-blending-activation-design.md`
- `docs/superpowers/specs/2026-06-30-moveabsolute-field-scan-order-alignment-design.md`

External references:
- PLCopen Function Blocks for Motion Control Version 2.0 compliance appendix: <https://www.plcopen.org/download_file/8050474b-a8b9-44d4-98b5-51a9f88ca76f/488/>
- PLCopen Function Blocks for Motion Control Part 6 - Fluid Power Extensions: <https://www.plcopen.org/download_file/force/9d64ccb7-4b78-4af3-b477-14d2df272256/342/>
- Beckhoff general rules for MC function blocks: <https://infosys.beckhoff.com/content/1033/tcplclib_tc2_mc2/70043531.html>
- Beckhoff `MC_BufferMode`: <https://infosys.beckhoff.com/content/1033/tcplclib_tc2_mc2/70147595.html>
- Beckhoff `MC_MoveAbsolute`: <https://infosys.beckhoff.com/content/1033/tcplclib_tc2_mc2/70094731.html>

Code references:
- `src/motion_interface.c`
- `src/motion_control.c`
- `tests/test_moveabsolute_blending_done.c`
- `tests/test_motion_interface_arbitration.c`
- `tests/test_motion_interface_done_signals.c`

## Goal

Define the required PLC-visible behavior for a three-FB same-axis `HydMoveAbsolute` scenario under persistent `Execute = TRUE`, with strict `1 active + 1 pending` direct-command capacity and PLCopen-style blending completion semantics.

The target field scan order is:

`FB1() -> FB2() -> FB3() -> Publish() -> read outputs`

This round is a contract-definition and validation-design round. It does not redesign the planner or introduce deeper buffering.

## Scenario Under Design

All three IEC FB instances stay resident and keep `Execute := TRUE` after their rising edge.

- `FB1 = MoveAbsolute(position=100, velocity=5, bufferMode=ABORT)`
- `FB2 = MoveAbsolute(position=200, velocity=20, bufferMode=BLENDING_HIGH)`
- `FB3 = MoveAbsolute(position=10, velocity=10, bufferMode=BLENDING_HIGH)`

Expected high-level outcome:

1. `FB1` starts and becomes active.
2. `FB2` is accepted as the only buffered successor.
3. `FB3` is rejected in the same scan in which it is submitted.
4. `FB1` reaches a valid blend cutover and reports `Done`, not `CommandAborted`.
5. `FB2` becomes active after cutover and later reports `Done`.
6. `FB3` must not disturb `FB1` or `FB2`.

## Confirmed Constraints

### 1. Queue depth is fixed

For same-axis direct commands, the runtime may hold at most:

- `1` active direct owner
- `1` pending direct successor

No hidden third slot, deferred overflow queue, or implicit replacement is allowed.

### 2. Blending completion is cutover-based

For a valid `MoveAbsolute -> MoveAbsolute` blend:

- the front command completes at the valid cutover
- completion does not require velocity to reach zero
- the successor becomes the new active owner after the cutover

This is the required behavior for `FB1 -> FB2` in this scenario.

### 3. Submission failure is not command abort

If a same-axis command is rejected because the active slot and pending slot are already occupied, that failure is a submission error, not a previously accepted command being aborted.

Therefore `FB3` must report `Error`, not `CommandAborted`.

## External Semantics Baseline

The design is grounded in the following external rules:

- PLCopen and Beckhoff both model `Busy`, `Done`, `Active`, `CommandAborted`, and `Error` as distinct output concepts.
- Beckhoff documents that `Busy`, `Done`, `Error`, and `CommandAborted` are mutually exclusive command-state outputs, while `Active` describes whether the command currently controls the axis.
- Beckhoff documents that a buffered command is accepted before it becomes active, and that only one buffered command is available in the normal command queueing model.
- Beckhoff `MC_BufferMode` defines blending relative to the transition between the active command and the accepted successor, not as early takeover of the whole first segment.
- PLCopen Part 6 is relevant because this repository implements hydraulic motion control; this design therefore keeps the command-state contract aligned with PLCopen motion semantics while using the repository's hydraulic planner and runtime.

This design uses Beckhoff's documentation as the concrete executable interpretation surface for PLCopen-style behavior.

## Problem Statement

The repository already has the core ingredients for this behavior:

- one active direct owner
- one pending direct slot
- direct tickets for lifecycle identity
- direct blend context for smooth `MoveAbsolute -> MoveAbsolute` cutover

However, the required three-FB scenario needs an explicit contract because it combines several edge conditions at once:

- multiple same-axis IEC FB instances
- persistent `Execute = TRUE`
- one legal blended successor
- one illegal third submission
- distinction between field-visible `Error` and `CommandAborted`
- requirement that a rejected third command has zero side effects on the accepted two-command chain

Without an explicit contract, it is easy to produce superficially plausible behavior that is still wrong for PLC logic:

- letting `FB3` steal or replace the pending slot
- reporting `FB3.CommandAborted` instead of `FB3.Error`
- delaying `FB2.Active` too long after cutover
- treating `FB1` cutover as an abort instead of a normal blended completion

## Design

### 1. Field-visible state model

For direct `MoveAbsolute`, the accepted lifecycle states in this scenario are:

- `accepted -> active -> done`
- `accepted -> pending -> active -> done`
- `rejected at submission`

The third branch is intentionally separate from accepted-command lifecycle tracking.

`FB3` must never appear to PLC logic as a command that was first accepted and then aborted. It was never accepted into the axis command chain.

### 2. Scan-order contract

The field-visible order remains:

`FB1() -> FB2() -> FB3() -> Publish() -> read outputs`

Responsibilities are split as follows:

- FB call:
  - detect rising edges
  - validate command admissibility
  - accept or reject the command
  - refresh PLC-visible outputs for that FB instance
- `Publish()`:
  - advance the core motion runtime
  - execute blend cutover
  - record completion
  - promote a pending command when allowed

This means ownership transition may occur internally during `Publish()`, but PLC-visible state must become observable on the next FB calls without an extra artificial delay.

### 3. Slot-capacity rule

When the axis is in the state:

- `active slot = occupied by FB1`
- `pending slot = occupied by FB2`

then `FB3` must be rejected immediately on its submission scan.

Required consequences:

- `FB3.Error = TRUE`
- `FB3.ErrorID` identifies the submission-capacity failure (`buffer full`, `pending slot occupied`, or equivalent command-not-allowed diagnostic for this runtime)
- `FB3.Busy = FALSE`
- `FB3.Active = FALSE`
- `FB3.Done = FALSE`
- `FB3.CommandAborted = FALSE`

Required non-consequences:

- `FB2` remains in the pending slot
- the direct blend context remains defined by `FB1 -> FB2`
- `FB1` remains the active owner
- no preemption record is created for `FB1` or `FB2`
- no completion marker is created for `FB3`

### 4. Output mapping rules

#### `Busy`

`Busy` means the command was accepted and has not yet terminated.

Therefore:

- `FB1.Busy = TRUE` after immediate start
- `FB2.Busy = TRUE` while pending and while active
- `FB3.Busy = FALSE` because its submission failed

#### `Active`

`Active` means the command currently owns control of the axis.

Therefore:

- `FB1.Active = TRUE` while it is the active owner
- `FB2.Active = FALSE` while only pending
- `FB2.Active = TRUE` only after ownership transfers at the valid cutover
- `FB3.Active = FALSE` always in this scenario

#### `Done`

`Done` means normal completion of the command's own lifecycle.

Therefore:

- `FB1.Done = TRUE` at valid blend cutover
- `FB1.Done` does not wait for velocity to drop to zero
- `FB2.Done = TRUE` only after its own target-position completion
- `FB3.Done = FALSE` because rejected commands do not complete

#### `CommandAborted`

`CommandAborted` means a command had already entered the accepted lifecycle and later lost it due to preemption, clear, or runtime interruption.

Therefore:

- `FB1.CommandAborted = FALSE` during legal blended handoff
- `FB2.CommandAborted = FALSE` during normal promotion and completion
- `FB3.CommandAborted = FALSE` because rejection at submission is not post-acceptance abort

#### `Error`

`Error` means the submission itself failed.

Therefore:

- `FB3.Error = TRUE` on the same scan in which the full-slot condition is detected
- `FB3.ErrorID` must resolve to the runtime's slot-capacity / command-not-allowed diagnostic rather than to an unrelated motion, planner, or ownership code
- `FB1.Error = FALSE` in the normal scenario
- `FB2.Error = FALSE` in the normal scenario

### 5. Cutover contract for `FB1 -> FB2`

The valid `BLENDING_HIGH` cutover must satisfy all of the following:

- `FB1` remains the active owner until the cutover is actually reached
- the blend context may shape the terminal through-velocity of `FB1`
- the cutover does not zero the axis velocity
- at the cutover:
  - `FB1` is recorded as normally completed
  - `FB2` is promoted from pending to active owner
  - the pending slot is cleared
  - the old blend context is cleared

Field-visible consequences on subsequent FB calls:

- `FB1.Done = TRUE`
- `FB1.CommandAborted = FALSE`
- `FB2.Active = TRUE`

### 6. Ticket and runtime bookkeeping rules

The runtime must preserve the distinction between:

- accepted owner
- accepted pending command
- rejected submission

This implies:

- `FB1` gets a stable direct ticket when started
- `FB2` gets a stable direct ticket when accepted into the pending slot
- `FB3` must not create a ticket entry that participates in owner, completed, or preempted tracking

If an implementation allocates a transient identifier during validation, that identifier must not leak into accepted-command bookkeeping.

The purpose is to keep the PLC-visible outputs derivable from unambiguous accepted-command state instead of from best-effort inference.

### 7. Persistent-`Execute` stability rules

Since all three FB instances keep `Execute = TRUE`, the runtime must remain stable under repeated cyclic calls:

- `FB3` must not repeatedly rewrite axis state after its initial rejection
- `FB1.Done` must not be overwritten to `CommandAborted` on later scans just because `Execute` stays high
- `FB2.Done` must remain a normal completion result and must not re-enter active ownership
- the accepted tickets for `FB1` and `FB2` must remain authoritative until consumed by their normal lifecycle transitions

This is required so ordinary PLC code that leaves FB instances resident across scans does not see output flapping.

## Recommended Core Shape

This design is compatible with the repository's current direction and recommends preserving these per-axis structures:

- one active direct slot
- one pending direct slot
- one direct blend context
- stable direct tickets for accepted commands
- separate rejected-submission handling

The critical architectural rule is:

`rejected submission` must stay outside the accepted-command lifecycle path.

That single separation protects the correctness of:

- `Error` versus `CommandAborted`
- pending-slot integrity
- blend-context integrity
- `Done` reporting for the two accepted commands

## Non-Goals

This design does not:

- add third-level buffering
- add implicit deferred queueing for `FB3`
- redesign planner curves
- introduce jerk-limited blending
- broaden the queueing model to recipe or mixed-command orchestration
- change the accepted `1 active + 1 pending` direct-command depth

## Validation Plan

### 1. Primary three-FB scenario

Add or adapt a regression that drives:

`FB1() -> FB2() -> FB3() -> Publish() -> read outputs`

with the exact command set from this design.

The test must prove:

- `FB1` becomes active first
- `FB2` becomes `Busy = TRUE, Active = FALSE` as the accepted pending command
- `FB3` reports submission-time `Error`
- `FB3` does not disturb the accepted `FB1 -> FB2` chain
- `FB1.Done` occurs at nonzero cutover velocity
- `FB2.Active` is observed after cutover
- `FB2.Done` is observed at final completion

### 2. Core-state invariants

Validate directly against core runtime state:

- the pending slot belongs to `FB2` before cutover
- `FB3` rejection does not overwrite the pending slot
- `FB3` rejection does not overwrite the direct blend context
- after cutover, `FB2` becomes active owner and the pending slot is clear

### 3. Output-semantic separation

Add assertions that explicitly distinguish:

- submission error (`FB3.Error`)
- normal blended completion (`FB1.Done`)
- normal successor completion (`FB2.Done`)
- absence of false aborts (`FB1.CommandAborted = FALSE`, `FB2.CommandAborted = FALSE`, `FB3.CommandAborted = FALSE`)

### 4. Persistent-high stability

Hold all three `Execute` inputs high across repeated scans and verify:

- no output flapping
- no repeated side effects from the rejected third command
- no stale-ticket ownership confusion after `Done`

## Acceptance Criteria

This design is satisfied only when all of the following are true:

1. Same-axis direct-command depth remains exactly `1 active + 1 pending`.
2. The third same-axis `MoveAbsolute` is rejected in the submission scan with `Error`, not `CommandAborted`.
3. Rejection of the third command leaves the accepted `FB1 -> FB2` command chain unchanged.
4. `FB1` reports `Done` at legal blended cutover and never reports `CommandAborted` for that handoff.
5. `FB2` becomes `Active` only after the cutover and later reports normal `Done`.
6. Persistent `Execute = TRUE` does not destabilize outputs or accepted-command ownership.
