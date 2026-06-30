# MoveAbsolute Field Scan Order Alignment Design

Date: 2026-06-30

Related specs:
- `docs/superpowers/specs/2026-06-27-direct-moveabsolute-blending-ownership-design.md`
- `docs/superpowers/specs/2026-06-29-moveabsolute-buffered-blending-activation-design.md`

References:
- `src/motion_interface.c`
- `src/motion_control.c`
- `tests/test_moveabsolute_blending_done.c`
- `tests/test_motion_interface_arbitration.c`
- `tests/test_motion_interface_done_signals.c`

## Goal

Fix the direct `MoveAbsolute -> MoveAbsolute` buffered blending case where the second IEC FB instance does not expose the expected `ACTIVE` and `DONE` outputs under the real PLC scan order used in the field.

The target field scan order is confirmed as:

`FB1() -> FB2() -> Publish() -> read outputs`

This round is not a planner redesign. The blending cutover math and one-active-one-pending direct-command contract already exist. The gap is the IEC adapter's timing contract relative to the field scan order.

## Confirmed Requirements

- Two IEC `MoveAbsolute` FB instances may bind to the same axis.
- `FB1` starts first with `BufferMode = ABORT`.
- `FB2` is accepted later with `BufferMode = BLENDING_HIGH`.
- Both FB instances may keep `EXECUTE = TRUE` across scans.
- The visible field behavior must be judged against the field scan order:
  - call both FBs
  - run `Publish()`
  - then read FB outputs
- After blended cutover:
  - `FB1` must report `DONE = TRUE`
  - `FB1` must not report `COMMANDABORTED = TRUE`
  - `FB2` must become the direct owner
  - `FB2` must report `ACTIVE = TRUE` on the first post-cutover FB call that sees acquired ownership
- After `FB2` finishes:
  - `FB2` must report `DONE = TRUE`
  - `FB2` must not report `COMMANDABORTED = TRUE`

## Problem Statement

The current repository already passes the existing blending regression suite, but those tests primarily drive and sample the system with a different timing model:

`Publish() -> FB1() -> FB2() -> read outputs`

That timing hides a field-order contract gap. In the real field order, buffered ownership transfer happens during `Publish()`, while the IEC output mapping for `MoveAbsolute` is only refreshed when the next FB call runs. The current `MoveAbsolute` pending-acquired branch then returns too early and leaves stale outputs visible for that scan.

As a result, a field user can see the second FB stay `ACTIVE = FALSE` when ownership has already transferred in the runtime, and can also miss the expected `DONE` observation window for the second command.

## Root Cause Findings

### 1. Cutover happens inside `Publish()`

The runtime performs blend cutover in the motion-control scan path called from `__HydMotion_framework_Publish()`. When the front segment reaches the valid cutover point:

- the front direct ticket is recorded as completed
- the pending direct slot is promoted
- the new direct owner is installed in core runtime state

This is correct and should stay in place.

### 2. `MoveAbsolute` suppresses same-call state refresh after pending acquisition

In `src/motion_interface.c`, `__mcl_cmd_MoveAbsolute()` handles `_PENDING` ownership resolution before the normal owner-state mapping.

When `resolveDirectPendingOwnership(...)` returns `HYD_DIRECT_PENDING_ACQUIRED`, the function currently:

- writes `_EXEC_ID`
- clears `_PENDING`
- returns immediately

That early return means the same call does not continue into the owner-state logic that sets:

- `ACTIVE`
- `BUSY`
- `DONE`
- `COMMANDABORTED`

Under field scan order, this creates a stale-output scan immediately after cutover.

### 3. Existing blending tests under-represent the field timing model

`tests/test_moveabsolute_blending_done.c` currently drives its main loops with `Publish()` before the repeated FB calls. That validates core blending behavior, but it does not reproduce the field scan order that exposed the issue.

So the current green tests do not prove the field-visible `ACTIVE` and `DONE` timing contract.

## Design

### 1. Field-Order Visibility Contract

For the field scan order

`FB1() -> FB2() -> Publish() -> read outputs`

the runtime is allowed to transfer ownership only during `Publish()`. Therefore the IEC adapter must guarantee:

- once a later `MoveAbsolute()` call observes that its buffered command has acquired ownership, that same FB call must publish owner-state outputs immediately
- once a later `MoveAbsolute()` call observes normal completion for its ticket, that same FB call must publish `DONE` immediately

This keeps the field-visible contract aligned with PLC scan semantics rather than with internal runtime event timing.

### 2. `MoveAbsolute` Pending-Acquired Path Must Fall Through Into Owner Mapping

The `MoveAbsolute` adapter must keep the existing ownership-resolution step, but it must stop treating acquisition as a terminal branch.

Required behavior:

- if pending ownership is still waiting:
  - keep `BUSY = TRUE`
  - keep `ACTIVE = FALSE`
  - leave `_PENDING = TRUE`
- if pending ownership is aborted:
  - set `COMMANDABORTED = TRUE`
  - clear `BUSY`
  - clear `ACTIVE`
  - clear `_PENDING`
  - return
- if pending ownership is acquired:
  - write `_EXEC_ID`
  - clear `_PENDING`
  - continue through the normal direct-owner evaluation in the same call

The critical rule is that acquisition must not require one extra scan before `ACTIVE` can become visible.

### 3. Keep Completion and Preemption Semantics in the Existing Owner Path

This design does not move completion logic into the pending branch. Normal completion and preemption decisions should stay in the existing owner-state section that already checks:

- consumed completion marker
- preempted ticket history
- current owner identity
- lost ownership

The change is only that a newly acquired buffered command must reach that section in the same call that clears `_PENDING`.

### 4. Do Not Change Planner or Cutover Math in This Round

No change is required to:

- cutover position detection
- planner continuity carry-over
- blend velocity selection
- one-active-one-pending buffer depth
- direct ticket bookkeeping model

The problem is not that the wrong command becomes owner. The problem is that the IEC adapter does not publish the ownership transition promptly under the field scan order.

### 5. Test Timing Alignment Policy

`tests/test_moveabsolute_blending_done.c` must be updated so its main execution loops match the field timing model:

`FB1() -> FB2() -> Publish() -> read outputs`

This applies to the main reproducer and the main blending progression helpers. The purpose is to make the regression suite prove the field-visible contract, not just the internal runtime contract.

When inspecting related tests:

- if a test asserts field-visible post-scan state, prefer the field timing model
- if a test intentionally checks pre-`Publish()` or same-call transitional state, it may keep its existing timing, but the reason must be documented in a short comment

The default posture for blending integration tests should be field-order timing.

### 6. Regression Audit Scope

This round must inspect and adjust timing assumptions in:

- `tests/test_moveabsolute_blending_done.c`
- direct buffered/blended assertions in `tests/test_motion_interface_arbitration.c`
- direct-command done-signal assertions in `tests/test_motion_interface_done_signals.c`

The goal is not a bulk rewrite of all tests. The goal is to align tests that claim to validate field-visible post-scan behavior.

## Non-Goals

This design does not:

- generalize the fix to `MoveVelocity` or `PressureHandle` in the same round unless inspection proves the same field-order defect is already covered by the requested test-alignment work
- change PLCopen command acceptance policy
- change buffer depth
- add new IEC FB pins
- redesign planner math or hydraulic simulation timing

## Implementation Scope

Expected code touch points:

- `src/motion_interface.c`
  - update `__mcl_cmd_MoveAbsolute()` so `HYD_DIRECT_PENDING_ACQUIRED` falls through into owner-state mapping
- `tests/test_moveabsolute_blending_done.c`
  - change main drive loops and helper loops to field timing order
  - add or adjust assertions so `FB2.ACTIVE` and `FB2.DONE` are verified under field timing
- `tests/test_motion_interface_arbitration.c`
  - inspect buffered/blended state assertions and align timing where the test claims post-scan behavior
- `tests/test_motion_interface_done_signals.c`
  - inspect direct `MoveAbsolute` done-signal loops for timing assumptions and align the relevant field-visible cases

## Acceptance Tests

### 1. Field-order blended activation reproducer

Using:

- `FB1 = MoveAbsolute(position=100, velocity=5, bufferMode=ABORT)`
- `FB2 = MoveAbsolute(position=200, velocity=20, bufferMode=BLENDING_HIGH)`
- both FBs held at `EXECUTE = TRUE`
- scan order `FB1() -> FB2() -> Publish() -> read outputs`

the test must prove:

- `FB1` becomes active first
- `FB2` stays `BUSY = TRUE, ACTIVE = FALSE` while still pending
- after cutover, `FB2` becomes visible as `ACTIVE = TRUE`

### 2. Field-order second-command completion

Using the same scan order, the test must prove:

- `FB2` eventually reaches `DONE = TRUE`
- `FB2` does not report `COMMANDABORTED`

### 3. Existing smooth-cutover behavior remains intact

The field-order-aligned blending tests must still prove:

- nonzero velocity across valid cutover
- `FB1` reports `DONE`, not `COMMANDABORTED`
- active segment ownership transfers to the second target

## Risks and Mitigations

- Risk: changing the pending-acquired path could accidentally change abort handling.
  - Mitigation: keep the aborted-pending branch as an immediate return and verify `COMMANDABORTED` regressions in arbitration tests.
- Risk: bulk timing edits could hide tests that intentionally validate pre-`Publish()` state.
  - Mitigation: only change tests whose purpose is post-scan field-visible behavior, and annotate any deliberate exceptions.
- Risk: the field bug could appear fixed in one reproducer but remain untested elsewhere.
  - Mitigation: inspect the main direct blending and done-signal suites, not just the single reproducer.

## Success Criteria

This work is complete when:

- `MoveAbsolute` publishes correct owner-state outputs immediately after buffered ownership acquisition under field scan order
- `test_moveabsolute_blending_done.c` validates the field scan order by default
- relevant post-scan blending and done-signal tests are aligned with field timing or explicitly documented as exceptions
- existing blending smoothness and arbitration regressions remain green
