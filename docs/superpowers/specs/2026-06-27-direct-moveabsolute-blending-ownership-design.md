# Direct MoveAbsolute Blending Ownership Design

Date: 2026-06-27

Related specs:
- `docs/superpowers/specs/2026-05-20-beckhoff-blending-curves-design.md`
- `docs/superpowers/specs/2026-05-13-movevelocity-pressurehandle-direct-session-design.md`

## Goal

Close the remaining ownership-contract gap so multiple IEC `MoveAbsolute` FB instances can keep `EXECUTE=TRUE` across scans and still get correct `BufferMode` / blending semantics.

This round does not introduce new motion math. The planner already has the required nonzero terminal-velocity behavior. The missing piece is reliable direct-command lifecycle identity under multi-FB polling.

## Confirmed Decisions

- Real speed smoothing is implemented only for finite direct `MoveAbsolute -> MoveAbsolute`.
- Direct buffer capacity remains one active direct command plus one pending direct command.
- `MoveVelocity` and no-duration `PressureHandle` receiving `BLENDING_*` degrade to abort takeover.
- Finite non-`MoveAbsolute` direct commands may still use buffered lifecycle, but they do not gain nonzero through-speed blending.
- `MoveProfile` stays out of scope in this round.
- Acceptance priority is PLC multi-FB behavior first, planner continuity second.

## Problem

The current code already contains the physical blending pieces:

- `motion_planner.c` supports nonzero terminal velocity.
- `motion_control.c` supports one pending direct slot, blend context, and cutover without planner reset.

The remaining gap is in direct pending ownership.

Today the IEC adapter can infer "pending acquired" from the fact that some direct command is active on the axis. That is too weak for the real PLC usage pattern where:

1. one `MoveAbsolute` FB is active
2. a second `MoveAbsolute` FB is accepted into the pending slot
3. both FB instances keep being called every scan

Under that pattern, the second FB can clear `_PENDING` before its command actually becomes the owner. The ambiguity becomes worse when a third same-kind command clears the pending slot before cutover. The result is incorrect `BUSY` / `ACTIVE` / `DONE` / `COMMANDABORTED` sequencing even though the core blend math itself is present.

## Design

### 1. Split Segment Epoch From Direct Command Ticket

Keep `_executionId` as the internal per-segment runtime epoch. It remains core-private and continues to advance on each successful `HYD_BeginSegment()`.

Add a separate direct command ticket:

- assigned when a direct command is accepted
- stable for that command's entire lifecycle
- valid across pending, active, done, and command-aborted phases

The direct ticket is the lifecycle identity seen by the IEC adapter. `_executionId` is no longer used for direct ownership matching in `motion_interface.c`.

### 2. Direct Ticket Lifecycle

One accepted direct command always has exactly one ticket.

- Immediate abort takeover:
  - new command gets a new ticket
  - ticket becomes owner immediately
- Buffered / blending acceptance:
  - new command gets a new ticket
  - ticket stays pending until the command is actually promoted into the active slot
- Pending cancellation:
  - cancelled pending ticket is treated as `COMMANDABORTED` from the waiting FB's point of view
- Normal direct completion:
  - completed owner ticket is recorded once and consumed by the matching IEC FB

This makes the state transition explicit:

`accepted -> pending -> owner -> done`

or

`accepted -> pending -> commandaborted`

or

`accepted -> owner -> commandaborted`

### 3. Core State Additions

Add bounded direct-ticket state to `HYD_MotionControlFB`:

- monotonic `uint16_t _directTicketCounter`
- `uint16_t _directOwnerTicket`
- `uint16_t _directPendingTicket`
- one completed direct-ticket record for the last normally completed owner
- aborted direct-ticket history with capacity `2`

The aborted-ticket capacity must be `2` because one scan can invalidate both:

- the currently active direct owner
- the queued pending direct command

This happens on paths such as Stop / Abort / Reset / Fault or a new abort-style takeover while a pending command is queued.

### 4. Core API Contract

The core direct-command API changes from "accepted or rejected" only to "accepted or rejected, plus accepted ticket".

Required contract changes:

- `HYD_MotionControlFB_StartDirectCommand(...)` returns the accepted direct ticket via out-parameter
- `HYD_LiveUpdateRequest.ownerExecutionId` is replaced by `ownerTicket`
- direct-owner accessors expose ticket-based queries
- completed / aborted direct-command queries consume ticket records, not `_executionId`

No IEC public pin surface changes are required. These are core / adapter contract changes only.

### 5. IEC Adapter Mapping

`motion_interface.c` direct FBs keep the two-phase adapter shape, but the identity they track changes.

Rules:

- on `EXECUTE` rising edge, the adapter stores the accepted direct ticket into `_EXEC_ID` immediately
- `_EXEC_ID` becomes a private direct command ticket, not a mirror of `_executionId`
- `_PENDING` stays true until `currentOwnerTicket == myTicket`
- pending acquisition no longer keys off "any direct command is active"
- pending abort checks the aborted-ticket history for `myTicket` and kind
- done checks the completed-ticket record for `myTicket` and kind
- live-update authorization also uses `ownerTicket == myTicket`

This removes the current false-positive acquisition path for the second `MoveAbsolute`.

### 6. BufferMode Rules By Command Type

#### Real blending

Real nonzero through-speed blending is allowed only when all of the following are true:

- active command is direct `MoveAbsolute`
- pending command is direct `MoveAbsolute`
- both moves are finite position-ended moves
- directions are compatible
- `BufferMode` is one of `BLENDING_LOW`, `BLENDING_PREVIOUS`, `BLENDING_NEXT`, `BLENDING_HIGH`

#### Plain buffered lifecycle

If a direct command is finite but not eligible for real blending, it may still wait in the pending slot and start after the front command completes. This includes duration-limited `PressureHandle`.

In that case:

- no nonzero through-speed is generated
- no special planner math is added
- `BLENDING_*` behaves like buffered sequencing, not like physical blending

#### Abort-takeover fallback

If the active command is endless and therefore has no natural finite cutover point, `BUFFER` and `BLENDING_*` degrade to abort takeover.

This applies to:

- active `MoveVelocity`
- active no-duration `PressureHandle`

The runtime must not fake a pending blend for these commands.

#### Reverse-direction fallback

If a pending `MoveAbsolute` reverses direction relative to the active `MoveAbsolute`, the command may still be accepted into the pending slot, but it uses plain buffered lifecycle rather than nonzero through-speed blending.

### 7. Pending-Slot Teardown

Stop / Abort / Reset / Fault must clear all state tied to the pending direct command:

- pending segment slot
- blend context
- pending ticket
- aborted-ticket bookkeeping for the discarded pending command

This teardown is part of the ownership contract, not a best-effort cleanup step.

### 8. No Planner Redesign In This Round

`motion_planner.c` already contains the necessary nonzero terminal-velocity behavior. This design does not add:

- new curve families
- jerk-limited logic
- multi-command look-ahead
- recipe-side path blending

Planner changes in this round, if any, are limited to regression-safe adjustments that are independent of the new ticket model.

## Implementation Impact

Expected file impact:

- `include/motion_control.h`
  - direct ticket fields
  - bounded aborted-ticket history
  - `HYD_LiveUpdateRequest.ownerTicket`
  - direct-start API out-ticket
- `src/motion_control.c`
  - ticket allocation
  - pending-ticket promotion
  - pending-ticket cancellation
  - ticket completion / abort bookkeeping
- `src/motion_interface.c`
  - direct FB `_PENDING` / `_EXEC_ID` mapping switched from `_executionId` inference to exact ticket matching
- `tests/test_motion_interface_arbitration.c`
  - multi-FB same-kind waiting
  - exact cutover acquisition
  - pending cancellation
  - same-kind third-command preemption
- `tests/test_motion_interface_unit.c`
  - live-update authorization by ticket
  - fallback behavior for endless commands

No IEC XML or public FB pin additions are needed.

## Acceptance Tests

### PLC multi-FB behavior

1. Two `MoveAbsolute` FBs with `BLENDING_*`
   - first FB remains owner until actual cutover
   - second FB stays `_PENDING/BUSY` and not `ACTIVE`
   - cutover scan gives first FB `DONE` and second FB ownership
   - no zero-velocity pause is introduced at cutover

2. Same-kind third command while second `MoveAbsolute` is pending
   - pending second FB reports `COMMANDABORTED`
   - pending second FB never reports false ownership

3. Stop / Abort / Reset / Fault while one direct owner and one pending direct command coexist
   - active owner and pending command both receive correct terminal mapping
   - pending slot and blend context are cleared

### Contract fallback behavior

4. `MoveVelocity` and no-duration `PressureHandle` with `BLENDING_*`
   - runtime does not expose fake pending blend semantics
   - behavior degrades to abort takeover

5. Duration-limited `PressureHandle` with `BLENDING_*`
   - command may wait as plain buffered lifecycle
   - command does not gain nonzero through-speed semantics

### Existing physical continuity checks

6. Existing planner blend tests remain green
   - nonzero terminal velocity remains intact
   - cutover still preserves planner state

## Non-Goals

- `MoveProfile` blending or recipe-side ownership redesign
- multi-command look-ahead
- jerk-limited curves
- fake smoothing for endless commands
- direct buffer depth greater than one pending slot

## Assumptions

- current target still behaves as a one-segment recipe platform, so direct commands are the practical blending surface in this round
- PLC programs may call multiple direct FB instances on the same axis every scan and keep `EXECUTE=TRUE` high while waiting
- `_EXEC_ID` is adapter-private state; changing its internal meaning does not change the external IEC contract
