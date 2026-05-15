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

- the motion runtime core in [src/motion_control.c](/home/dan/project/hdy-motion-light/src/motion_control.c)
- shared runtime state in [motion_control.h](/home/dan/project/hdy-motion-light/include/motion_control.h)
- IEC adapter mappings in [motion_interface.c](/home/dan/project/hdy-motion-light/src/motion_interface.c)
- PLC process-layer consumers of the official runtime outputs

It is a contract for the current motion-execution layer, not a complete machine-process specification.

## Runtime Objects

| Object | Meaning | Maintained By | PLC-Stable? |
| --- | --- | --- | --- |
| `FB_STATE` | Framework-level runtime state machine | motion runtime core | Yes |
| `STATE.active` | Whether a segment is currently executing | motion runtime core | Yes |
| `STATE.finished` | Whether execution reached a normal terminal completion state | motion runtime core | Yes |
| `STATE.faultActive` | Whether execution is in a faulted/protected-stop condition | motion runtime core | Yes |
| `STATE.status` | Aggregated controller status for upper layers | motion runtime core | Yes |
| `SEGMENT_COMPLETED` | Whether the active segment reached its end condition | motion runtime core | Yes |
| `DIAGNOSTIC` | Live current-cycle/current-command diagnostic output | motion runtime core | Yes |
| `LAST_FAULT_SNAPSHOT` | Most recent retained fault snapshot for review/service | motion runtime core | Yes |
| `DIAGNOSTIC_HISTORY` | Retained diagnostic history/snapshot output | motion runtime core | Yes |
| `execution_id` | Runtime execution identity associated with the current started execution lifecycle | motion runtime core | No |

PLC logic should depend on official outputs and documented runtime state, not on underscored bookkeeping fields or adapter bookkeeping.

## State Model

### FB State Table

| `FB_STATE` | Meaning | `BUSY` | `ACTIVE` | Typical Entry Condition |
| --- | --- | --- | --- | --- |
| `HYD_FB_STATE_DISABLED` | Runtime disabled or unavailable | No | No | Disabled controller context |
| `HYD_FB_STATE_IDLE` | No active execution and no startable source selected | No | No | Fresh init or runtime cleared without recipe/direct source |
| `HYD_FB_STATE_READY` | No active execution, but a startable source exists | No | No | Post-load or post-reset ready state |
| `HYD_FB_STATE_STARTING` | Execution has started and startup processing is underway | Yes | Yes | Successful segment start |
| `HYD_FB_STATE_RUNNING` | Active execution is underway | Yes | Yes | Normal running execution |
| `HYD_FB_STATE_SEGMENT_COMPLETE` | A segment completed but execution is not yet fully terminal | Yes | No | Mid-recipe completion awaiting `Next` |
| `HYD_FB_STATE_HOLD` | Execution is paused while preserving segment context | Yes | No | `Hold` consumed |
| `HYD_FB_STATE_DONE` | Execution terminated normally | No | No | Final segment complete or stop-to-zero complete |
| `HYD_FB_STATE_ABORTED` | Execution terminated by abort semantics | No | No | `Abort` consumed |
| `HYD_FB_STATE_FAULT` | Execution entered fault handling | No | No | Fault/protected-stop path |

### Derived Runtime Fields

| Field | Meaning | Notes |
| --- | --- | --- |
| `STATE.active` | True only while a segment is actively executing | More precise than `BUSY`; `HOLD` is not active |
| `STATE.finished` | Normal-completion marker | Separate from fault and abort semantics |
| `STATE.faultActive` | Runtime fault/protected-stop marker | Drives `ERROR` semantics |
| `STATE.status` | Aggregated status for upper layers | Mirrors controller-level view, not raw ownership |
| `SEGMENT_COMPLETED` | End condition met for current segment | May be true before full recipe completion |

### State Rules

1. `FB_STATE` is the framework-layer state machine.
2. `STATE.status` is an aggregated controller status projection.
3. `STATE.finished` is not equivalent to `SEGMENT_COMPLETED`.
4. `DONE` is derived from terminal completion semantics, not from generic in-band tracking.

## Execution Ownership Model

### Source Kind

| Source | Meaning |
| --- | --- |
| `HYD_SEGMENT_SOURCE_NONE` | No active source |
| `HYD_SEGMENT_SOURCE_RECIPE` | Runtime is executing a recipe segment |
| `HYD_SEGMENT_SOURCE_DIRECT` | Runtime is executing a direct single segment |

### Execution Identity

| Concept | Meaning |
| --- | --- |
| `execution_id` | A runtime execution identity assigned when a segment start succeeds |
| Lifecycle | Advances on successful start; also advances when runtime ownership changes in ways that invalidate a previous direct observer, such as abort and stop takeover |
| Purpose | Distinguishes the current execution lifecycle from a previous one on the same axis |
| PLC visibility | Not a stable PLC field; documented here only as a runtime concept behind ownership semantics |

### Owner Kind

| Owner | Meaning |
| --- | --- |
| `HYD_DIRECT_CMD_NONE` | No direct owner |
| `HYD_DIRECT_CMD_MOVE_ABSOLUTE` | Direct position move owner |
| `HYD_DIRECT_CMD_MOVE_VELOCITY` | Direct velocity move owner |
| `HYD_DIRECT_CMD_PRESSURE_HANDLE` | Direct pressure-control owner |
| `HYD_DIRECT_CMD_STOP` | Direct controlled-stop owner |

Recipe execution does not currently expose a separate recipe-side owner-kind signal. It is still part of the same single-owner runtime model, but recipe-side observation is expressed mainly through runtime state and completion behavior rather than through a recipe-specific takeover signal.

### Direct Session State

| Session State | Meaning |
| --- | --- |
| `HYD_DIRECT_SESSION_IDLE` | No active direct session |
| `HYD_DIRECT_SESSION_RUNNING` | Direct owner is running normally |
| `HYD_DIRECT_SESSION_STOPPING` | `Stop` owns the session and is decelerating to zero |
| `HYD_DIRECT_SESSION_DONE` | Direct session reached normal terminal completion |
| `HYD_DIRECT_SESSION_ABORTED` | Direct session terminated by abort semantics |
| `HYD_DIRECT_SESSION_FAULT` | Direct session entered fault/protected-stop handling |

### Ownership Rules

1. At most one execution owner exists per axis at a time.
2. Every successful execution start creates or advances the current `execution_id`.
3. Starting a new direct command may preempt the previous active owner on the same axis.
4. `Stop` is modeled as a distinct direct execution owner, not as an immediate abort.
5. `Reset` clears active execution state but is not a normal-completion event for the preempted owner.
6. Recipe and direct execution both participate in the same single-owner runtime model, even if recipe-side ownership is surfaced less explicitly at the IEC level.

## Command Contract

### `Start`

- Trigger: start request reaches the runtime
- Allowed runtime states: `HYD_FB_STATE_IDLE`, `HYD_FB_STATE_READY`, `HYD_FB_STATE_SEGMENT_COMPLETE`, `HYD_FB_STATE_DONE`, `HYD_FB_STATE_ABORTED`
- Immediate effect: start command is queued and consumed by the runtime on a following scan
- Completion condition: segment starts successfully and runtime enters `STARTING`
- Failure condition: invalid source, invalid segment, invalid state, or invalid start context

### `Next`

- Trigger: next-segment request reaches the runtime
- Allowed runtime states: `HYD_FB_STATE_SEGMENT_COMPLETE`
- Immediate effect: requests advance to the next recipe segment
- Completion condition: next segment starts, or recipe ends normally
- Failure condition: no recipe, invalid state, or current segment not completed

### `Stop`

- Trigger: stop request reaches the runtime
- Allowed runtime states: `HYD_FB_STATE_STARTING`, `HYD_FB_STATE_RUNNING`
- Immediate effect: stop request is queued; on consumption it may take ownership as `HYD_DIRECT_CMD_STOP`
- Completion condition: deceleration reaches zero and runtime marks `HYD_DIRECT_SESSION_DONE` / `HYD_FB_STATE_DONE`
- Failure condition: invalid state, invalid context, or runtime fault
- Deceleration source: a positive caller-supplied stop deceleration is used directly. If the caller supplies zero or a negative value, the runtime falls back to the active segment's `maxDeceleration`; if that field is zero, it falls back to `maxAcceleration` for legacy recipes.

### `Hold`

- Trigger: command request
- Allowed runtime states: `HYD_FB_STATE_STARTING`, `HYD_FB_STATE_RUNNING`
- Immediate effect: runtime enters paused hold state while preserving segment context
- Completion condition: none; `Hold` is a state transition, not a terminal completion
- Failure condition: invalid state or invalid active-segment context

### `Resume`

- Trigger: command request
- Allowed runtime states: `HYD_FB_STATE_HOLD`
- Immediate effect: preserved segment resumes from current feedback context
- Completion condition: runtime re-enters execution flow
- Failure condition: invalid state, invalid active-segment context, or feedback validity failure

### `Abort`

- Trigger: command request
- Allowed runtime states: `HYD_FB_STATE_IDLE`, `HYD_FB_STATE_READY`, `HYD_FB_STATE_STARTING`, `HYD_FB_STATE_RUNNING`, `HYD_FB_STATE_SEGMENT_COMPLETE`, `HYD_FB_STATE_DONE`, `HYD_FB_STATE_ABORTED`, `HYD_FB_STATE_HOLD`
- Immediate effect: runtime clears active execution and enters abort semantics immediately on command consumption
- Completion condition: runtime reaches `HYD_FB_STATE_ABORTED`
- Failure condition: invalid command context or fault suppression path

### `Reset`

- Trigger: reset request reaches the runtime
- Allowed runtime states: handled via reset path rather than normal command legality table
- Immediate effect: runtime execution state is cleared while configuration, recipe/direct buffers, and tunable criteria are preserved
- Completion condition: runtime returns to `READY` or `IDLE` depending on preserved source availability
- Failure condition: invalid axis/context at adapter level

### `Ack`

- Trigger: diagnostic acknowledgement request
- Allowed runtime states: `HYD_FB_STATE_DISABLED`, `HYD_FB_STATE_IDLE`, `HYD_FB_STATE_READY`, `HYD_FB_STATE_SEGMENT_COMPLETE`, `HYD_FB_STATE_HOLD`, `HYD_FB_STATE_DONE`, `HYD_FB_STATE_ABORTED`
- Immediate effect: clears clearable retained diagnostics after the live event has cleared
- Completion condition: diagnostics acknowledged
- Failure condition: invalid state or still-active fault conditions

## Output Signal Contract

| Signal | Meaning | Assert Condition | Clear Condition | Terminal? |
| --- | --- | --- | --- | --- |
| `DONE` | Normal terminal completion | Runtime enters a normal done path for FBs that expose `DONE` | FB lifecycle reset or a new lifecycle begins | Yes |
| `BUSY` | Execution context is still in progress/owned by the FB | Active execution or command lifecycle still in progress | Completion, abort, reset, fault, or idle clear | No |
| `ACTIVE` | The FB currently owns active execution | Owner currently maps to active execution | Ownership lost or execution ends | No |
| `COMMANDABORTED` | The FB lost execution to a takeover/preemption | Adapter detects preemption or lost ownership | FB lifecycle reset or a new lifecycle begins | Lifecycle latch |
| `ERROR` | Faulted or invalid execution state as surfaced to the consumer | Runtime fault or externally surfaced invalid-usage condition | Reset/new lifecycle per-FB semantics | Lifecycle latch |
| `INVELOCITY` | Velocity-control target is within band | `MoveVelocity` owner active and band condition met | Out of band, preempted, stopped, reset, or lifecycle clear | No |
| `INPRESSURE` | Pressure-control target is within band | `PressureHandle` owner active and band condition met | Out of band, preempted, completed, reset, or lifecycle clear | No |

### Signal Rules

1. `DONE` is not a universal “target reached” signal for all FBs.
2. `INVELOCITY` and `INPRESSURE` are in-band achievement signals, not universal terminal signals.
3. `COMMANDABORTED` indicates loss of execution ownership, not necessarily a fault.
4. `ERROR` indicates fault or invalid-command handling, not normal preemption.
5. Recipe-side observation of takeover is exposed through the existing `MoveProfile.COMMANDABORTED` IEC output when ownership is lost.

## Completion Semantics

| Termination Type | `DONE` | `BUSY` | `ACTIVE` | `COMMANDABORTED` | `ERROR` |
| --- | --- | --- | --- | --- | --- |
| Intermediate recipe segment completion (`HYD_FB_STATE_SEGMENT_COMPLETE`) | No | Yes | No | No | No |
| Final normal execution completion (`HYD_FB_STATE_DONE`) | Yes for FBs that expose terminal `DONE` | No | No | No | No |
| Stop-to-zero completion | Yes for `Stop`; preempted motion owner sees `COMMANDABORTED` | No | No | Depends on observer | No |
| Abort termination | No | No | No | Consumer may observe takeover/abort semantics | No unless fault path also exists |
| Reset termination | No for the preempted command; `Reset` reports its own completion | No | No | Preempted owner may observe `COMMANDABORTED` | No unless invalid reset request |
| Fault termination | No | No | No | Not required | Yes |

`MoveVelocity` and `PressureHandle` do not both use terminal `DONE` semantics in the same way as `MoveAbsolute` and `Stop`.

## Recipe vs Direct Boundary

1. Recipe and direct segment buffers may coexist in memory.
2. The active source is latched at segment start time.
3. `USE_RECIPE=true` makes `Start` consume `RECIPE[index]`.
4. `USE_RECIPE=false` makes `Start` consume `DIRECT_SEGMENT`.
5. `NextSegment()` is meaningful only for recipe execution.
6. Direct execution is modeled as a single direct-run action owned by the current direct command.
7. Switching `USE_RECIPE` or editing `DIRECT_SEGMENT` while a segment is active does not retroactively change the current active segment.
8. `HYD_AXISMOTION.SEGMENTTAG` is the opaque process-layer segment identifier.
9. `HYD_AXISMOTION.SEGMENTTYPE` is the domain segment type and is mapped independently from `SEGMENTTAG`.
10. If a direct command or `Reset` takes over a recipe execution, the displaced `MoveProfile` lifecycle may observe `COMMANDABORTED=true` while the runtime transitions ownership.

### `HYD_LOADPROFILE`

`HYD_LOADPROFILE` is a preload-only IEC adapter FB. On a valid `EXECUTE` rising edge:

- recipe-configured axes load one segment into `RECIPE[0]`
- direct-configured axes load one segment into `DIRECT_SEGMENT`
- `DONE` reports successful preload completion
- the runtime does not start execution, take ownership, or set `STATE.active`

Execution still begins through `MoveProfile`/`Start` or through direct command FBs.

## Error and Protection Contract

### Error Semantics

- `ERROR` is derived from runtime fault state, `FB_STATE`, and diagnostic severity.
- `ERROR_ID` mirrors the active fault diagnostic code when `ERROR` is true.
- Integration layers may project invalid command/context usage onto `ERROR`, but that projection is outside the core runtime semantics defined here.

### Diagnostic Semantics

- `DIAGNOSTIC` is the live current-cycle/current-command diagnostic output.
- `LAST_FAULT_SNAPSHOT` is the most recent retained fault snapshot intended for review and service.
- `DIAGNOSTIC_HISTORY` and `LAST_FAULT_SNAPSHOT` retain diagnostic information for review and service.
- Warning diagnostics may exist without full terminal fault semantics.

### Protection Semantics

- Protection actions are reported by the runtime as part of diagnostics.
- A protected stop/fault path may terminate execution without producing `DONE`.
- `HYD_PROTECTION_ACTION_STOP` is a fault/protected-stop semantic, distinct from a user-requested `Stop` command.

## PLC Dependency Rules

| Field Class | Safe for PLC Logic? | Examples |
| --- | --- | --- |
| Official FB outputs | Yes | `DONE`, `BUSY`, `ACTIVE`, `ERROR`, `COMMANDABORTED`, `INVELOCITY`, `INPRESSURE` |
| Reported runtime state | Yes | `FB_STATE`, `STATE.status`, `SEGMENT_COMPLETED`, `DIAGNOSTIC`, `DIAGNOSTIC_HISTORY`, `LAST_FAULT_SNAPSHOT` |
| Internal bookkeeping | No | underscored ownership, pending-command, stop, and controller-state fields |

PLC logic should consume formal outputs and documented runtime state. It should not inspect underscored internal fields to infer completion, preemption, or stop status.

## Worked Examples

### `MoveAbsolute` Normal Completion

1. A direct position execution is started successfully.
2. The runtime owns active execution for that axis.
3. When the segment reaches its end condition, the runtime reaches normal completion.
4. Upper layers should observe:
   - `DONE=true`
   - `BUSY=false`
   - `ACTIVE=false`
   - `COMMANDABORTED=false`

### `MoveAbsolute` Preempted by `Stop`

1. An active direct position execution is running.
2. A stop request is accepted and ownership transfers to `HYD_DIRECT_CMD_STOP`.
3. Stop decelerates over multiple cycles until velocity reaches zero.
4. Upper layers should observe:
   - on the preempted move owner: `COMMANDABORTED=true`, `DONE=false`
   - on the stop owner during deceleration: no terminal completion yet
   - on the stop owner after deceleration: `DONE=true`, `BUSY=false`

### `MoveVelocity` Preempted by Another Direct Owner

1. A direct velocity execution is running.
2. Another direct owner takes over the same axis.
3. Runtime transfers direct ownership to the new command.
4. Upper layers should observe on the old `MoveVelocity` FB:
   - `COMMANDABORTED=true`
   - `BUSY=false`
   - `ACTIVE=false`
   - `INVELOCITY=false`

## Not Supported

The current runtime contract does not define or fully support:

- direct valve control
- machine phase sequencing
- V/P transfer decision logic
- machine-specific hydraulic interlocks
- PLCopen blending modes beyond the currently supported buffer-mode subset
- full end-to-end semantics for all exposed IEC pins that are present only for compatibility or extension

Known examples to verify against implementation and documentation cleanup:

- `JERK` and `CONTINUOUSUPDATE` on some IEC FBs are still reserved compatibility pins and do not yet imply full runtime semantics
- `DECELERATION` is independently consumed by `MoveAbsolute`, `MoveVelocity`, and `HYD_AXISMOTION`, but broader PLCopen motion-profile semantics are still not claimed beyond the current braking/deceleration behavior
- `HYD_LOADPROFILE` is implemented as preload-only and is not an execution lifecycle owner
- `PressureHandle` timed completion currently clears `BUSY/ACTIVE` without exposing a `DONE` pin like `MoveAbsolute` or `Stop`
