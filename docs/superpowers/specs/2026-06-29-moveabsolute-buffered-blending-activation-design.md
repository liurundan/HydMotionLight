# MoveAbsolute Buffered Blending Activation Design

Date: 2026-06-29

Related specs:
- `docs/superpowers/specs/2026-05-20-beckhoff-blending-curves-design.md`
- `docs/superpowers/specs/2026-06-27-direct-moveabsolute-blending-ownership-design.md`

References:
- Beckhoff `MC_BufferMode`: <https://infosys.beckhoff.com/content/1033/tcplclib_tc2_mc2/70147595.html>
- Beckhoff motion command behavior notes: <https://infosys.beckhoff.com/content/1033/tcplclib_tc2_mc2/70043531.html>

## Goal

Fix the direct `MoveAbsolute -> MoveAbsolute` buffered blending bug where a second FB instance can affect the first segment too early and make the axis run with the second segment's velocity before the first target-position cutover.

The target behavior is TwinCAT-style direct buffering:

- one active direct command on the axis
- one pending direct command on the axis
- no hidden third-slot queue
- `BLENDING_LOW` / `BLENDING_HIGH` only shape the cutover, not the whole first segment

## Confirmed Requirements

- Two different IEC `MoveAbsolute` FB instances may bind to the same axis.
- `FB1` starts with `BufferMode = Aborting`.
- `FB2` is triggered later with `BufferMode = BlendingHigh` or `BlendingLow`.
- Real deployment timing matches the early-trigger case: `FB2` may rise very soon after `FB1`, even while `FB1` has only just become `ACTIVE`.
- Even when PLC logic uses `FB1.ACTIVE` to trigger `FB2.EXECUTE`, the runtime must not let `FB2` take over the first segment early.
- While `FB2` is accepted but not yet promoted:
  - `BUSY = TRUE`
  - `ACTIVE = FALSE`
  - `DONE = FALSE`
  - `COMMANDABORTED = FALSE`
- At the cutover point:
  - `FB1` reports `DONE = TRUE`
  - `FB1` does not report `COMMANDABORTED = TRUE`
  - `FB2` becomes the active owner
- Queue depth stays exactly `1 running + 1 pending`.

## Current Findings

The current runtime already contains the core pieces for one-slot buffered direct commands:

- one pending direct slot in `HYD_MotionControlFB`
- a direct blend context
- cutover logic that can promote the pending slot without zeroing planner state
- planner support for nonzero terminal velocity at blend cutover

The bug is not "no buffering exists". The bug is that buffering and blending are exposed too early.

Confirmed implementation conflicts:

1. `__mcl_cmd_MoveAbsolute(...)` currently marks a newly accepted command `BUSY=true, ACTIVE=true` on the rising edge even when the command may only be pending behind another active owner.
2. The direct blend context is created as soon as the second command is accepted into the pending slot.
3. The planner consumes that blend context whenever it is active, so the pending command can influence the current active segment before the cutover window.

This matches the observed field symptom: before the first target position is reached, the axis can accelerate or run as if the second segment velocity already replaced the first segment velocity.

## Problem Statement

The runtime currently mixes three different states:

- command accepted
- command owns the active direct segment
- command is eligible to affect cutover velocity

TwinCAT-style semantics require those states to stay separate.

For the buffered command:

- acceptance must only mean "queued"
- ownership must begin only after promotion at cutover
- blend influence must begin only in a bounded activation window near the first segment's target

Without that separation, PLC code sees a false `ACTIVE`, and the active segment planner can respond to a future segment too early.

## Design

### 1. Behavioral Contract

Define the direct `MoveAbsolute` buffered lifecycle as:

`accepted -> pending -> active -> done`

or

`accepted -> pending -> commandaborted`

Never:

`accepted -> active` when another direct command still owns the axis.

For buffered/blended commands:

- `BUSY` means the command has been accepted and has not yet terminated.
- `ACTIVE` means the command is the current direct owner of the axis.
- `DONE` means the command completed as the owner or, for the front blended segment, completed at the valid cutover.
- `COMMANDABORTED` means the command was displaced, cleared, or rejected after acceptance.

### 2. Queue Depth Contract

Direct `MoveAbsolute` buffering stays limited to:

- `1` active direct command
- `1` pending direct command

If a third same-axis direct command arrives while one command is active and one is pending, the new command must be rejected. It must not silently replace the pending command.

### 3. Submission Result Must Distinguish Start vs Pending

The core direct-command submission path must return more than a boolean success value.

The caller needs to know which of these happened:

- rejected
- accepted and started immediately
- accepted into the pending slot

This distinction is required so the IEC adapter can map `ACTIVE` correctly on the `EXECUTE` rising edge.

The existing "success only" shape is insufficient because the IEC layer cannot tell whether a successful submission became the owner or merely entered the waiting slot.

### 4. IEC FB State Mapping

For `MoveAbsolute`:

- immediate-start acceptance:
  - `BUSY = TRUE`
  - `ACTIVE = TRUE`
- pending acceptance:
  - `BUSY = TRUE`
  - `ACTIVE = FALSE`

The FB must stay in the pending state until ownership actually transfers to its command.

This change is required both for PLC-facing contract correctness and to make `FB1.ACTIVE` a dependable trigger source in process-layer code.

### 5. Blend Context Must Be Recorded Early but Armed Late

The second command may still be accepted early and stored in the pending slot. That is correct.

What must change is when that future command is allowed to influence the current planner.

Split direct blending into two concepts:

- **blend recorded**
  - the pending command is eligible for future blending
  - the runtime has stored the pending segment and selected blend mode / candidate through-speed
- **blend armed**
  - the current active segment is close enough to its cutover region that the blend context may shape terminal velocity

Only the **armed** state may affect the active segment's planner input.

Before the blend is armed:

- the first segment runs under its own velocity constraints
- the second segment does not replace the first segment's overall velocity plan

### 6. Blend Arming Rule

The runtime must use a bounded activation window near the first segment's target position before applying the blend terminal-velocity constraint.

This design fixes the activation rule as:

- compute the active segment's remaining distance to `switchPosition`
- compute the braking distance required to reduce the active segment from its unconstrained target velocity down to the selected blend terminal velocity using the active segment braking limit
- arm the blend only when:
  - `remainingDistance <= brakingDistance + switchTolerance`

This makes the arming point explicit and keeps the blend dormant until the first segment is physically close enough that terminal-velocity shaping is relevant.

Operationally:

- outside the activation window, the pending segment is ignored by the planner
- inside the activation window, the pending segment may shape the front segment's terminal velocity according to the selected blend mode

This keeps the current physical blend math but narrows when it is legal to participate.

### 7. Blend Mode Meaning

For the supported direct finite `MoveAbsolute -> MoveAbsolute` case:

- `BlendingLow`: cutover speed uses the lower of the two segment velocity limits
- `BlendingHigh`: cutover speed uses the higher of the two segment velocity limits

These modes do not authorize early full-segment takeover.

They only define the allowed cutover speed at the transition region.

### 8. Cutover Contract

When the first segment reaches the cutover region:

- the first segment is recorded as completed
- the pending segment is promoted into the active slot
- planner continuity is preserved across the promotion
- the first FB reports `DONE`
- the second FB becomes `ACTIVE`

The first FB must not be mapped to `COMMANDABORTED` during a valid blended cutover.

### 9. Non-Goals

This design does not add:

- more than one pending command
- recipe path blending
- jerk-limited blend redesign
- generalized multi-segment look-ahead
- new FB pins

This is a contract and timing fix for direct `MoveAbsolute` buffering and blending only.

## Root Cause Hypothesis

The defect is most likely produced by the combination of:

1. pending acceptance immediately setting `ACTIVE=true` in the IEC adapter
2. pending blend context immediately reaching planner input
3. early-trigger PLC timing where the second FB is accepted before the first segment has meaningfully progressed toward its cutover region

That combination makes "future command accepted" behave too much like "future command active now".

## Implementation Scope

Expected code touch points:

- `src/motion_interface.c`
  - fix `MoveAbsolute` rising-edge state mapping for pending acceptance
- `include/motion_control.h`
  - extend direct-command start result so callers know whether the command started immediately or queued
- `src/motion_control.c`
  - separate "blend recorded" from "blend armed"
  - restrict planner exposure to armed blends only
  - preserve existing one-active-one-pending direct buffer contract
- `tests/test_moveabsolute_blending_done.c`
  - add reproduction coverage for early-trigger buffered blending
- `tests/test_motion_interface_arbitration.c`
  - add state-contract and queue-depth tests

## Validation Order

Validation must happen in this sequence:

1. add a failing regression test that reproduces the early-trigger bug under the current code
2. verify the current code fails that test
3. implement the runtime changes
4. verify the new regression turns green
5. run the pre-existing blending and arbitration tests to ensure no regression in already-working happy paths

This order is mandatory. The bug must be reproduced before the fix is accepted.

## Required Regression Tests

### 1. Early-trigger reproduction

Add a regression where:

- `FB1` starts with `Aborting`
- `FB2` is triggered very early with `BlendingHigh`
- the trigger path must include the PLC-style case where `FB2.EXECUTE` rises because `FB1.ACTIVE` became true
- both FBs keep `Execute = TRUE`
- before `FB1` reaches its target neighborhood:
  - `FB1` remains owner
  - `FB2.ACTIVE == FALSE`
  - the commanded/planned velocity does not jump to the second segment's full max velocity

This test should fail on the current implementation.

### 2. Same reproduction with `BlendingLow`

The same early-trigger sequence must also be covered for `BlendingLow`, because the bug is about activation timing, not only about the selected through-speed value.

### 3. Pending FB state contract

For an accepted buffered command before cutover:

- `BUSY == TRUE`
- `ACTIVE == FALSE`
- `DONE == FALSE`
- `COMMANDABORTED == FALSE`

### 4. Valid cutover completion

At the cutover:

- `FB1.DONE == TRUE`
- `FB1.COMMANDABORTED == FALSE`
- `FB2.ACTIVE == TRUE`
- the planner does not reset to zero-velocity pause

### 5. Queue depth enforcement

With one active and one pending direct `MoveAbsolute` already present:

- a third same-axis command must be rejected
- it must not replace the pending command implicitly

### 6. Existing happy-path preservation

Keep the current end-to-end blend completion coverage and existing arbitration tests green after the fix.

## Acceptance Criteria

The design is accepted when all of the following are true:

- the early-trigger bug is reproduced by a regression test before the code change
- buffered `MoveAbsolute` no longer reports false `ACTIVE`
- a pending blended command cannot drive the first segment with the second segment's full velocity before the cutover region
- valid `MoveAbsolute -> MoveAbsolute` cutover still completes without a zero-speed pause
- queue depth remains `1 running + 1 pending`
- no new public IEC parameter or pin surface is introduced
