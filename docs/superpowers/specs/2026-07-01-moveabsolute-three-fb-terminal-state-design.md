# MoveAbsolute Three-FB Terminal State Design

Date: 2026-07-01

Related specs:
- `docs/superpowers/specs/2026-06-27-direct-moveabsolute-blending-ownership-design.md`
- `docs/superpowers/specs/2026-06-29-moveabsolute-buffered-blending-activation-design.md`
- `docs/superpowers/specs/2026-06-30-moveabsolute-field-scan-order-alignment-design.md`
- `docs/superpowers/specs/2026-06-30-moveabsolute-third-command-buffer-full-design.md`

References:
- Beckhoff general rules for MC function blocks: <https://infosys.beckhoff.com/content/1033/tcplclib_tc2_mc2/70043531.html>
- Beckhoff `MC_BufferMode`: <https://infosys.beckhoff.com/content/1033/tcplclib_tc2_mc2/70147595.html>
- Beckhoff `MC_MoveAbsolute`: <https://infosys.beckhoff.com/content/1033/tcplclib_tc2_mc2/70094731.html>
- PLCopen Function Blocks for Motion Control Part 6 - Fluid Power Extensions: <https://www.plcopen.org/download_file/force/9d64ccb7-4b78-4af3-b477-14d2df272256/342/>

Code references:
- `src/motion_interface.c`
- `src/motion_control.c`
- `include/motion_interface.h`
- `tests/test_moveabsolute_blending_done.c`
- `tests/test_motion_interface_arbitration.c`
- `tests/test_motion_interface_done_signals.c`
- `tests/test_moveabsolute_stop_integration.c`

## Goal

Fix the same-axis three-FB `MoveAbsolute` field scenario where the first blended command can lose its visible `Done` state after cutover, which then prevents PLC logic from advancing to the second command as intended.

The target scenario is:

- `FB1 = MoveAbsolute(position=100, velocity=5, bufferMode=ABORT)`
- `FB2 = MoveAbsolute(position=200, velocity=20, bufferMode=BLENDING_HIGH)`
- `FB3 = MoveAbsolute(position=10, velocity=10, bufferMode=BLENDING_HIGH)`
- all three FB instances stay resident
- all three keep `Execute = TRUE` after their rising edge
- field scan order is:
  - `FB1() -> FB2() -> FB3() -> Publish() -> read outputs`

Expected outcome:

1. `FB1` starts, becomes active, and at valid blend cutover reports `Done = TRUE`.
2. `FB2` is buffered first, then becomes active after cutover, then later reports `Done = TRUE`.
3. `FB3` is rejected because the direct queue is already full (`1 active + 1 pending`), but it must not disturb `FB1` or `FB2`.
4. `FB3.Error` remains latched while `FB3.Execute = TRUE`, and clears only when `FB3.Execute = FALSE`.

## Confirmed External Semantics

This design uses Beckhoff documentation as the concrete executable interpretation surface for PLCopen-style behavior:

- `Busy`, `Done`, `Error`, and `CommandAborted` are mutually exclusive.
- `Active` indicates that the command currently executes the axis motion.
- `Busy = TRUE` and `Active = FALSE` is valid for a buffered command that has been accepted but is not yet executing.
- only one buffered command is available while another command executes
- if another buffered command is triggered while the buffered slot is already occupied, the new buffered command is rejected with an error
- `MC_MoveAbsolute.Active` becomes `TRUE` only after the current command completes
- a blended follow-on command changes the velocity at the transition point, not ownership of the first segment before the transition

These rules are consistent with PLCopen Part 6 fluid-power motion behavior and with the repository's hydraulic direct-command model.

## Problem Statement

The repository already has the following machinery:

- one active direct owner
- one pending direct slot
- direct command tickets
- blended `MoveAbsolute -> MoveAbsolute` cutover
- same-scan post-acquisition owner mapping in the `MoveAbsolute` IEC adapter

However, the three-FB field scenario still needs an explicit terminal-state contract because the visible FB outputs are currently derived too directly from live owner/pending facts.

That creates a failure mode where:

- `Publish()` performs a correct `FB1 -> FB2` cutover in the core runtime
- `FB1` is no longer the owner on subsequent scans
- `FB1` can lose its visible `Done` state because the IEC adapter keeps recomputing status from current runtime ownership instead of latching the already-earned terminal result

In the same scenario, the third command needs a separate rule:

- `FB3` is not an accepted command that later gets displaced
- `FB3` is a submission failure and must therefore latch `Error`, not `CommandAborted`

## Design

### 1. Field-visible lifecycle contract

For `MoveAbsolute`, the accepted field-visible lifecycles in this round are:

- `accepted -> active -> done`
- `accepted -> pending -> active -> done`
- `accepted -> active/pending -> commandaborted`
- `rejected at submission -> error`

The last branch is not part of the accepted-command ownership lifecycle. A rejected command must never be reinterpreted later as a preempted command.

### 2. Queue-depth contract remains unchanged

Direct-command capacity stays fixed at:

- `1` active direct owner
- `1` pending direct follower

This round does not add a third slot, deferred queue, or pending replacement policy.

### 3. `MoveAbsolute` must latch its own terminal outcome

The IEC adapter for `MoveAbsolute` must stop deriving terminal outputs only from the current axis owner relationship.

Instead, each `HYD_MOVEABSOLUTE` instance must maintain a private local terminal state that survives while `Execute = TRUE`.

Required local states:

- `IDLE`
- `RUNNING_OWNER`
- `RUNNING_PENDING`
- `DONE_LATCHED`
- `ERROR_LATCHED`
- `ABORTED_LATCHED`

This may be implemented through explicit enum storage or through an equivalent compact representation in private FB fields. The important contract is behavioral, not the exact storage shape.

### 4. Core runtime remains responsible only for axis-level facts

The core runtime continues to own:

- current direct owner ticket/kind
- pending direct slot contents
- completed direct ticket record
- preempted direct ticket history
- submission rejection result for a new rising-edge request

This round does not move terminal latching into `motion_control.c`. The latch belongs to the IEC FB instance because `Done`, `Error`, and `CommandAborted` are FB-visible outputs tied to that instance's own lifecycle.

### 5. IEC `MoveAbsolute` consumes core facts and latches one terminal result

The IEC adapter must translate runtime facts into one stable FB-local terminal state.

Rules:

- if the rising edge is rejected:
  - enter `ERROR_LATCHED`
  - set `Error = TRUE`
  - set the rejection `ErrorID`
  - keep `Busy = FALSE`
  - keep `Active = FALSE`
  - keep `Done = FALSE`
  - keep `CommandAborted = FALSE`
- if the command is accepted and starts immediately:
  - enter `RUNNING_OWNER`
- if the command is accepted into the pending slot:
  - enter `RUNNING_PENDING`
- if a pending command later acquires ownership:
  - transition `RUNNING_PENDING -> RUNNING_OWNER`
- if the command's ticket is observed in the completed record:
  - enter `DONE_LATCHED`
- if the command's ticket is observed in the preempted record:
  - enter `ABORTED_LATCHED`

### 6. Terminal latches have priority over live ownership recomputation

Once a `HYD_MOVEABSOLUTE` instance enters one of these terminal states:

- `DONE_LATCHED`
- `ERROR_LATCHED`
- `ABORTED_LATCHED`

the FB must continue to present that terminal output while `Execute = TRUE`.

It must not re-enter running state simply because:

- another command is now the current owner
- the pending slot has changed
- the old ticket is no longer current

This rule is what preserves `FB1.Done = TRUE` after a legal blended cutover and preserves `FB3.Error = TRUE` after a full-slot rejection.

### 7. Clear condition is only the FB's own `Execute` falling edge

For `MoveAbsolute`, the following outputs and private local terminal state reset only when that same FB instance sees `Execute = FALSE`:

- `Done`
- `Error`
- `ErrorID`
- `CommandAborted`
- `Busy`
- `Active`
- local terminal/running state
- `_PENDING`
- `_EXEC_ID`

No other command's completion, promotion, or rejection may clear a different FB instance's terminal outcome.

This matches Beckhoff's documented rule that outputs are reset by the `Execute` falling edge, while command execution itself is unaffected by dropping `Execute`.

### 8. `FB1 -> FB2` blended cutover contract

The existing blended cutover semantics stay in place:

- `FB1` remains the active owner until the valid cutover point
- `FB2` stays buffered until that cutover
- at cutover:
  - the runtime records `FB1`'s direct ticket as completed
  - the runtime promotes `FB2` from pending to owner
  - planner continuity remains nonzero

Field-visible consequences on later FB calls:

- `FB1` consumes the completed ticket and enters `DONE_LATCHED`
- `FB1.CommandAborted = FALSE`
- `FB2` consumes acquired ownership and enters `RUNNING_OWNER`
- `FB2.Active = TRUE`

This round does not reinterpret a legal blended cutover as an abort.

### 9. `FB3` full-slot rejection contract

When `FB3` rises while:

- `FB1` owns the active slot
- `FB2` already occupies the pending slot

then `FB3` must be rejected immediately.

Required visible mapping:

- `FB3.Error = TRUE`
- `FB3.ErrorID = HYD_DIAG_CODE_COMMAND_NOT_ALLOWED` or the repository's precise full-slot rejection code if a more specific code already exists
- `FB3.Busy = FALSE`
- `FB3.Active = FALSE`
- `FB3.Done = FALSE`
- `FB3.CommandAborted = FALSE`

Required persistence:

- while `FB3.Execute = TRUE`, the above error state remains latched

Required non-effects:

- `FB2` remains the pending follower
- the blend context remains `FB1 -> FB2`
- `FB1` remains eligible to complete normally at cutover
- no completion/preemption record is created for `FB3`

### 10. Scope boundary for other function blocks

This round is intentionally scoped to `MoveAbsolute` behavior.

It must not change the documented public semantics of:

- `MoveVelocity`
- `PressureHandle`
- `Stop`
- `MoveProfile`

If shared helpers or direct-ticket readers are touched, their effects must be verified by regression tests. No new terminal-state behavior may be silently imposed on other FB types in this round.

## Implementation Scope

Expected code touch points:

- `src/motion_interface.c`
  - implement latched terminal-state behavior for `__mcl_cmd_MoveAbsolute()`
  - ensure terminal-output priority is above live ownership recomputation
- `include/motion_interface.h`
  - extend `HYD_MOVEABSOLUTE` private storage only if needed for an explicit local terminal/running state
- `tests/test_moveabsolute_blending_done.c`
  - add the exact three-FB field reproducer
  - assert `FB1.Done`, `FB2.Active`, `FB2.Done`, and sticky `FB3.Error`
- `tests/test_motion_interface_arbitration.c`
  - verify the third buffered `MoveAbsolute` is still rejected and does not disturb the accepted pair
- `tests/test_motion_interface_done_signals.c`
  - verify terminal outputs remain latched until `Execute = FALSE`
- `tests/test_moveabsolute_stop_integration.c`
  - verify `MoveAbsolute` still reports `CommandAborted` when genuinely preempted by `Stop`

## Required Regression Matrix

### 1. Main three-FB field reproducer

Under scan order:

- `FB1() -> FB2() -> FB3() -> Publish() -> read outputs`

and with all three `Execute` signals held `TRUE`:

- `FB1` starts and becomes active
- `FB2` is accepted as pending and remains `Busy = TRUE`, `Active = FALSE`
- `FB3` is rejected with `Error = TRUE`
- `FB3.Error` remains `TRUE` while `FB3.Execute = TRUE`
- `FB1` reaches valid blended cutover and reports `Done = TRUE`
- `FB2` becomes `Active = TRUE` after cutover
- `FB2` later reports `Done = TRUE`
- `FB3` does not disturb `FB1`/`FB2` completion

### 2. Existing `MoveAbsolute` same-family coverage must remain green

The round must preserve:

- two-FB blended nonzero-velocity cutover behavior
- pending command `Busy = TRUE`, `Active = FALSE` while waiting
- same-axis third-command rejection behavior
- retrigger-after-done behavior
- falling-edge output reset behavior

### 3. Cross-FB combination coverage must remain green

The round must verify:

- `MoveAbsolute` preempted by `Stop` still yields `CommandAborted`, not `Done`
- `MoveVelocity -> Stop` done behavior remains unchanged
- `PressureHandle` done/error paths remain unchanged
- multi-axis isolation remains unchanged

### 4. Validation command set

At minimum, run:

- `ctest --test-dir out/build/unixgcc --output-on-failure -R test_moveabsolute_blending_done`
- `ctest --test-dir out/build/unixgcc --output-on-failure -R test_motion_interface_arbitration`
- `ctest --test-dir out/build/unixgcc --output-on-failure -R test_motion_interface_done_signals`
- `ctest --test-dir out/build/unixgcc --output-on-failure -R test_moveabsolute_stop_integration`

If implementation touches shared direct-command helpers, run the broader affected direct-command subset as well before claiming completion.

## Non-Goals

This round does not:

- redesign planner blend math
- change the cutover position rule
- add a third buffered command slot
- generalize the terminal-state latch to every other FB type in the same change without explicit need
- change external PLC pins
- change `MoveVelocity` or `PressureHandle` user-visible semantics

## Acceptance Criteria

The design is satisfied when:

1. `FB1` in the three-FB scenario visibly reaches `Done = TRUE` and stays there while its `Execute = TRUE`.
2. `FB2` still acquires ownership and later reaches `Done = TRUE`.
3. `FB3` reports sticky submission `Error`, not `CommandAborted`, while its `Execute = TRUE`.
4. `FB3` has zero side effects on the accepted `FB1 -> FB2` command chain.
5. Existing `MoveAbsolute` blending behavior remains correct.
6. The specified cross-FB regressions remain green so the fix does not introduce new state or execution errors in other function-block combinations.
