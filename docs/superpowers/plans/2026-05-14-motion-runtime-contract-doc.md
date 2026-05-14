# Motion Runtime Contract Document Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Produce the first formal architecture document, `docs/architecture/motion-runtime-contract.md`, so the motion-execution layer has a stable, reviewable runtime contract for library developers and PLC process-layer consumers.

**Architecture:** This work is documentation-first. The document must describe current intended runtime semantics clearly enough that code and tests can be compared against it later. It should not invent machine-process logic, valve behavior, or unsupported PLCopen semantics; it should only formalize the motion runtime layer that already exists or is explicitly intended.

**Tech Stack:** Markdown, existing design/spec docs in `docs/superpowers/specs`, current C headers and runtime/IEC implementation in `include/` and `src/`, existing tests in `tests/`.

---

## File Map

- Create: `docs/architecture/motion-runtime-contract.md`
  First formal runtime contract document for the motion-execution layer.
- Reference: `docs/superpowers/specs/2026-05-14-motion-control-layer-docs-design.md`
  Approved design document that defines the documentation set and recommended order.
- Reference: `include/motion_control.h`
  Canonical source for core runtime states, commands, control modes, and comments on signal semantics.
- Reference: `include/common_types.h`
  Canonical source for shared enums, diagnostic types, control modes, and segment semantics.
- Reference: `src/motion_control.c`
  Current runtime behavior source for command queueing, ownership, stop/hold/resume/reset, diagnostics, and completion flow.
- Reference: `src/motion_interface.c`
  IEC-facing mapping behavior for `DONE/BUSY/ACTIVE/COMMANDABORTED/ERROR/INVELOCITY/INPRESSURE`.
- Reference: `tests/test_motion_interface_unit.c`
  Focused expectations for adapter/runtime signal behavior.
- Reference: `tests/test_motion_interface_done_signals.c`
  Integration expectations for done/in-band semantics.
- Reference: `tests/test_motion_interface_arbitration.c`
  Ownership takeover and preemption expectations.
- Reference: `tests/test_moveabsolute_stop_integration.c`
  Stop-to-zero completion expectations.

## Task 1: Collect Runtime Semantics Inputs

**Files:**
- Reference: `docs/superpowers/specs/2026-05-14-motion-control-layer-docs-design.md`
- Reference: `include/motion_control.h`
- Reference: `include/common_types.h`
- Reference: `src/motion_control.c`
- Reference: `src/motion_interface.c`
- Reference: `tests/test_motion_interface_unit.c`
- Reference: `tests/test_motion_interface_done_signals.c`
- Reference: `tests/test_motion_interface_arbitration.c`
- Reference: `tests/test_moveabsolute_stop_integration.c`

- [x] **Step 1: Re-read the approved design doc and extract the required sections**

Read:

```bash
sed -n '1,260p' docs/superpowers/specs/2026-05-14-motion-control-layer-docs-design.md
```

Expected extraction list:

- purpose and scope
- runtime objects
- state model
- execution ownership model
- command contract
- output signal contract
- completion semantics
- recipe vs direct boundary
- error and protection contract
- PLC dependency rules
- worked examples
- not supported

- [x] **Step 2: Capture the state and command vocabulary from `include/motion_control.h`**

Read:

```bash
sed -n '1,260p' include/motion_control.h
```

Record these exact contract items for later use:

- `HYD_FbCommand`
- `HYD_FbState`
- `HYD_DirectCommandKind`
- `HYD_DirectSessionState`
- comments describing `STATE.active`, `IsBusy`, `IsDone`, `IsError`, `SEGMENT_COMPLETED`, `DIAGNOSTIC`, and `USE_RECIPE`

- [x] **Step 3: Capture the shared type semantics from `include/common_types.h`**

Read:

```bash
sed -n '1,260p' include/common_types.h
```

Record these exact contract items for later use:

- `HYD_ControlMode`
- `HYD_EndConditionType`
- `HYD_MotionDirection`
- `HYD_SegmentSource`
- `HYD_ControllerStatus`
- `HYD_DiagnosticSeverity`
- `HYD_DiagnosticCode`
- `HYD_BufferMode`
- comments describing segment and mode semantics

- [x] **Step 4: Extract current runtime command and completion behavior from `src/motion_control.c`**

Read:

```bash
sed -n '1,1715p' src/motion_control.c
```

Capture the actual current behavior for:

- command legality matrix
- pending-command queueing
- begin-segment ownership setup
- stop takeover and deceleration
- hold/resume
- abort
- reset preservation model
- runtime fault entry
- segment-complete to done path

- [x] **Step 5: Extract IEC-facing signal behavior from `src/motion_interface.c`**

Read:

```bash
sed -n '1,1100p' src/motion_interface.c
```

Capture the actual current adapter behavior for:

- `MoveProfile`
- `MoveAbsolute`
- `MoveVelocity`
- `PressureHandle`
- `Stop`
- `Reset`

Especially record:

- how `EXECUTE` rising edges are handled
- when `_PENDING` is cleared
- when `DONE` is raised
- when `COMMANDABORTED` is raised
- when `INVELOCITY` and `INPRESSURE` are asserted

- [x] **Step 6: Cross-check tests so the document reflects externally relied-on behavior**

Read:

```bash
sed -n '1,260p' tests/test_motion_interface_done_signals.c
sed -n '1,260p' tests/test_motion_interface_arbitration.c
sed -n '1,260p' tests/test_moveabsolute_stop_integration.c
sed -n '1,260p' tests/test_motion_interface_unit.c
```

Extract externally visible guarantees already enforced by tests:

- stop requires multiple cycles to decelerate before `DONE`
- preempted owners report `COMMANDABORTED`
- `PressureHandle` completion clears `BUSY/ACTIVE` without a `DONE` pin
- `Reset` preserves direct configuration while clearing active execution

- [x] **Step 7: Summarize mismatches between interface appearance and actual supported semantics**

Make a working note with at least these items if still true during review:

- `DECELERATION/JERK/CONTINUOUSUPDATE` are exposed on some IEC FBs but not fully implemented semantically
- `LoadProfile` is implemented as preload-only and is not an execution lifecycle owner
- blending modes beyond `ABORT/BUFFER` are not supported

- [x] **Step 8: Commit the notes-free context collection checkpoint**

```bash
git status --short
```

Expected:

- no documentation output yet, only analysis complete in working memory

No commit in this task; commit only after the document is created and reviewed.

## Task 2: Draft `motion-runtime-contract.md`

**Files:**
- Create: `docs/architecture/motion-runtime-contract.md`

- [x] **Step 1: Write the document header and scope section**

Create the file with this opening structure:

```md
# Motion Runtime Contract

Date: 2026-05-14

## Purpose

This document defines the formal runtime contract of the motion-execution layer in HydroMotionLib.

It specifies:

- runtime state semantics
- execution ownership semantics
- command semantics
- output signal semantics
- completion and termination semantics
- error and protection semantics
- the PLC-visible fields that upper layers may safely depend on

It does not specify:

- valve control
- machine process sequencing
- V/P transfer decision logic
- machine-specific hydraulic interlocks
- product-specific molding process rules

## Scope

This contract applies to:

- the motion runtime core in `src/motion_control.c`
- shared runtime state in `include/motion_control.h`
- IEC adapter mappings in `src/motion_interface.c`
- PLC process-layer consumers of the official runtime outputs
```

- [x] **Step 2: Add the runtime objects section**

Write a `## Runtime Objects` section with a table containing at least these rows:

```md
| Object | Meaning | Maintained By | PLC-Stable? |
| --- | --- | --- | --- |
| `FB_STATE` | Framework-level runtime state machine | motion runtime core | Yes |
| `STATE.active` | Whether a segment is currently executing | motion runtime core | Yes |
| `STATE.finished` | Whether execution reached a terminal completed state | motion runtime core | Yes |
| `STATE.faultActive` | Whether execution is in a faulted/protected-stop condition | motion runtime core | Yes |
| `STATE.status` | Aggregated controller status for upper layers | motion runtime core | Yes |
| `SEGMENT_COMPLETED` | Whether the active segment reached its end condition | motion runtime core | Yes |
| `DIAGNOSTIC` | Live current-cycle/current-command diagnostic output | motion runtime core | Yes |
| `_executionId` | Internal execution identity used for ownership tracking | runtime core / IEC adapter | No |
| `_pendingCommand` | Internal queued command | motion runtime core | No |
| `_isStopping` | Internal stop-deceleration flag | motion runtime core | No |
```

- [x] **Step 3: Add the state-model section with an explicit state table**

Write `## State Model` and include:

```md
### FB State Table

| `FB_STATE` | Meaning | `BUSY` | `ACTIVE` | Typical Entry Condition |
| --- | --- | --- | --- | --- |
| `HYD_FB_STATE_DISABLED` | Runtime disabled or unavailable | No | No | Disabled controller context |
| `HYD_FB_STATE_IDLE` | No active execution and no ready source selected | No | No | Fresh init or cleared runtime with no startable source |
| `HYD_FB_STATE_READY` | No active execution, but a recipe/direct source is available to start | No | No | Post-reset or post-load ready state |
| `HYD_FB_STATE_STARTING` | Execution has started and startup processing is underway | Yes | Yes | Successful segment start |
| `HYD_FB_STATE_RUNNING` | Active execution is underway | Yes | Yes | Runtime settled into normal execution |
| `HYD_FB_STATE_SEGMENT_COMPLETE` | A segment completed but recipe execution is not fully terminal | Yes | No | Mid-recipe completion waiting for `Next` |
| `HYD_FB_STATE_HOLD` | Execution is paused while preserving segment context | Yes | No | `Hold` consumed |
| `HYD_FB_STATE_DONE` | Execution terminated normally | No | No | Final segment done or stop-to-zero done |
| `HYD_FB_STATE_ABORTED` | Execution was terminated by abort semantics | No | No | `Abort` consumed |
| `HYD_FB_STATE_FAULT` | Execution entered fault handling | No | No | Fault/protected-stop path |
```
```

Then add short text explicitly distinguishing:

- `FB_STATE` vs `STATE.status`
- `STATE.finished` vs `SEGMENT_COMPLETED`
- `DONE` vs “reached control band”

- [x] **Step 4: Add the execution ownership model section**

Write `## Execution Ownership Model` with:

- a `source_kind` table using `NONE / RECIPE / DIRECT`
- an `owner_kind` table using currently supported direct owners
- a `session_state` table using `IDLE / RUNNING / STOPPING / DONE / ABORTED / FAULT`

Then add these exact rules in prose:

```md
1. At most one execution owner exists per axis at a time.
2. Starting a new direct command may preempt the previous active owner on the same axis.
3. `Stop` is modeled as a distinct direct execution owner, not as an immediate abort.
4. `Reset` clears active execution state but is not a normal-completion event.
```

- [x] **Step 5: Add the command contract section**

Write `## Command Contract` and add subsections for:

- `Start`
- `Next`
- `Stop`
- `Hold`
- `Resume`
- `Abort`
- `Reset`
- `Ack`

For each command, use this mini-template:

```md
### `Stop`

- Trigger: command request or IEC FB rising edge
- Allowed runtime states: `STARTING`, `RUNNING`
- Immediate effect: stop request is queued and may take ownership
- Completion condition: commanded deceleration reaches zero and runtime marks stop completion
- Not equivalent to: `Abort`
- PLC note: `DONE` must not be assumed on the trigger cycle during active motion
```

- [x] **Step 6: Add the output signal contract table**

Write `## Output Signal Contract` and include a table with at least:

```md
| Signal | Meaning | Assert Condition | Clear Condition | Terminal? |
| --- | --- | --- | --- | --- |
| `DONE` | Normal terminal completion | Runtime enters a normal done path for FBs that expose `DONE` | `EXECUTE` falls or a new command lifecycle begins | Yes |
| `BUSY` | Execution context still in-progress/owned by the FB | Active execution or command lifecycle in progress | Completion, abort, reset, fault, or idle clear | No |
| `ACTIVE` | The FB currently owns active execution | Owner currently maps to active execution | Ownership lost or execution ends | No |
| `COMMANDABORTED` | Previous owner lost execution to a takeover | Preemption detected by the adapter/runtime contract | `EXECUTE` falls or a new lifecycle begins | Yes-ish lifecycle latch |
| `ERROR` | Runtime/adapter fault or invalid command state | Runtime fault or adapter-reported invalid usage | Reset/new lifecycle per FB semantics | Yes-ish lifecycle latch |
| `INVELOCITY` | Velocity-control target is within band | `MoveVelocity` owner active and band condition met | Out of band, preempted, stopped, reset, or execute clear | No |
| `INPRESSURE` | Pressure-control target is within band | `PressureHandle` owner active and band condition met | Out of band, preempted, completed, reset, or execute clear | No |
```

Add text explicitly stating:

- `DONE` is not a universal “target reached” signal for all FBs.
- `INVELOCITY` and `INPRESSURE` are in-band achievement signals, not universal terminal signals.

- [x] **Step 7: Add a terminal-semantics matrix**

Write `## Completion Semantics` with a matrix like:

```md
| Termination Type | `DONE` | `BUSY` | `ACTIVE` | `COMMANDABORTED` | `ERROR` |
| --- | --- | --- | --- | --- | --- |
| Normal segment completion | Yes for FBs that expose terminal done | No | No | No | No |
| Stop-to-zero completion | Yes for `Stop`; preempted motion owner sees `COMMANDABORTED` | No | No | Depends on owner | No |
| Abort termination | No | No | No | Consumer may observe takeover/abort semantics | No unless fault path also present |
| Reset termination | No for the preempted command; `Reset` reports its own completion | No | No | Preempted owner may observe `COMMANDABORTED` | No unless invalid reset request |
| Fault termination | No | No | No | Not required | Yes |
```

- [x] **Step 8: Add recipe/direct boundary and PLC dependency sections**

Write:

- `## Recipe vs Direct Boundary`
- `## PLC Dependency Rules`

In `PLC Dependency Rules`, include an explicit table:

```md
| Field Class | Safe for PLC Logic? | Examples |
| --- | --- | --- |
| Official FB outputs | Yes | `DONE`, `BUSY`, `ACTIVE`, `ERROR`, `COMMANDABORTED` |
| Reported runtime state | Yes | `FB_STATE`, `STATE.status`, `SEGMENT_COMPLETED` |
| Internal bookkeeping | No | `_executionId`, `_pendingCommand`, `_isStopping` |
```

- [x] **Step 9: Add three worked examples**

Write `## Worked Examples` with these subsections:

1. `MoveAbsolute` normal completion
2. `MoveAbsolute` preempted by `Stop`
3. `MoveVelocity` preempted by `PressureHandle` or another direct owner

Each example should include:

- trigger sequence
- expected ownership transition
- expected external output changes
- what the PLC layer should consume

- [x] **Step 10: Add `Not Supported`**

Write `## Not Supported` with at least:

- direct valve control
- machine phase sequencing
- V/P transfer decision logic
- machine-specific interlocks
- blending modes beyond the currently supported buffer model subset
- any IEC FB pins whose semantics are not currently implemented end-to-end

## Task 3: Review the Draft Against Current Code and Tests

**Files:**
- Modify: `docs/architecture/motion-runtime-contract.md`

- [x] **Step 1: Read the draft and compare each section against current runtime code**

Run:

```bash
sed -n '1,260p' docs/architecture/motion-runtime-contract.md
sed -n '1,1715p' src/motion_control.c
sed -n '1,1100p' src/motion_interface.c
```

Expected review outcome:

- every document claim matches either code or explicit intended current behavior already enforced in tests
- no machine-process sequencing language appears in the runtime document

- [x] **Step 2: Check the document against test-enforced behavior**

Run:

```bash
sed -n '1,260p' tests/test_motion_interface_done_signals.c
sed -n '1,260p' tests/test_motion_interface_arbitration.c
sed -n '1,260p' tests/test_moveabsolute_stop_integration.c
```

Expected review outcome:

- stop-to-zero semantics are documented correctly
- preemption behavior is documented correctly
- `PressureHandle` completion semantics are documented correctly

- [x] **Step 3: Remove or soften any statement that over-promises unsupported semantics**

Specifically verify the document does not imply:

- jerk support
- continuous update semantics
- full PLCopen blending support
- machine-specific hydraulic sequencing support

- [x] **Step 4: Run a placeholder and ambiguity scan**

Run:

```bash
rg -n "TODO|TBD|maybe|might|probably|future|later" docs/architecture/motion-runtime-contract.md
```

Expected:

- either no matches
- or only deliberate mentions inside `Not Supported`

- [x] **Step 5: Commit the runtime-contract document**

```bash
git add docs/architecture/motion-runtime-contract.md docs/superpowers/specs/2026-05-14-motion-control-layer-docs-design.md docs/superpowers/plans/2026-05-14-motion-runtime-contract-doc.md
git commit -m "docs: add motion runtime contract plan and design"
```

## Self-Review

Spec coverage check:

- This plan covers the first deliverable from the approved documentation-design spec: `motion-runtime-contract.md`.
- It does not yet implement the boundary guide, PLC integration guide, or archetype guide. That is intentional to keep scope to one focused deliverable.

Placeholder scan:

- No `TODO/TBD/implement later` placeholders are used as task content.
- Every task includes exact files and exact commands.

Type consistency:

- The plan uses the same contract vocabulary already present in the codebase: `FB_STATE`, `STATE`, `SEGMENT_COMPLETED`, `DONE`, `BUSY`, `ACTIVE`, `COMMANDABORTED`, `ERROR`, `INVELOCITY`, `INPRESSURE`.

## Execution Handoff

Plan complete and saved to `docs/superpowers/plans/2026-05-14-motion-runtime-contract-doc.md`. Two execution options:

**1. Subagent-Driven (recommended)** - I dispatch a fresh subagent per task, review between tasks, fast iteration

**2. Inline Execution** - Execute tasks in this session using executing-plans, batch execution with checkpoints

Which approach?
