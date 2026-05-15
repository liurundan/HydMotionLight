# PLC Process-Layer Integration Guide

Date: 2026-05-14

## Purpose

This document explains how a PLC process layer should integrate with HydroMotionLib.

It is intended for PLC engineers and process-layer developers who:

- call the IEC-facing function blocks
- consume runtime outputs
- orchestrate clamp, injection, hold, eject, carriage, and related machine phases

This document does not define a complete machine process sequence. It defines the recommended way to connect the PLC process layer to the motion-execution layer.

## Integration Model

HydroMotionLib is intended to sit between:

- machine-specific PLC process logic
- the reusable motion-execution runtime

The recommended split is:

- The library calculates motion execution, pressure/velocity/position control, diagnostics, protection state, and pump-side requests.
- The PLC process layer decides when machine phases begin/end, when a motion FB is triggered, how valve states are controlled, and how interlocks and process conditions are evaluated.

In practice:

1. The PLC process layer determines the current machine phase.
2. The PLC process layer selects the appropriate motion FB and parameters.
3. The motion layer executes the commanded segment and reports runtime state.
4. The PLC process layer consumes those outputs and decides the next machine step.

## Recommended Scan-Cycle Pattern

The PLC process layer should follow a stable cyclic pattern.

### Per-Cycle Sequence

1. Read measured machine feedback.
2. Update the library's axis/runtime feedback inputs.
3. Execute the PLC process state machine.
4. Decide whether to trigger or maintain motion FB commands.
5. Call the relevant motion FBs.
6. Read motion-layer outputs.
7. Drive valve outputs and other machine outputs based on:
   - current process phase
   - direction semantics
   - interlocks
   - machine state
8. Advance to the next PLC cycle.

### Reference Pseudocode

```text
Read axis/process feedback
Update library feedback inputs
Run PLC process state machine
Set EXECUTE / command parameters
Call motion FBs
Read DONE/BUSY/ACTIVE/ERROR/COMMANDABORTED/INVELOCITY/INPRESSURE
Drive valves and machine outputs
Advance cycle
```

## `EXECUTE` Usage Rules

The IEC FB surface is edge-sensitive. The PLC process layer should treat `EXECUTE` as a lifecycle control signal, not as a casual command bit.

### General Rules

1. Start-type FBs are triggered by an `EXECUTE` rising edge.
2. After a command has started, the PLC process layer should keep the FB lifecycle consistent until the command reaches its defined termination or takeover outcome.
3. After terminal completion or command invalidation, the PLC process layer should explicitly drop `EXECUTE` to prepare for the next command lifecycle.
4. The PLC process layer must not assume that all FBs use `DONE` in the same way.

### Recommended Lifecycle by FB

The formal signal semantics are defined in [motion-runtime-contract.md](/home/dan/project/hdy-motion-light/docs/architecture/motion-runtime-contract.md).

This table only summarizes what PLC code should typically watch during integration.

| FB | Trigger | While Running | Typical Runtime Observations | Lifecycle Reset Guideline |
| --- | --- | --- | --- | --- |
| `MoveAbsolute` | `EXECUTE` rising edge | keep lifecycle stable while owned | observe `DONE` or `COMMANDABORTED`/`ERROR` | drop `EXECUTE` after completion or takeover |
| `MoveVelocity` | `EXECUTE` rising edge | keep lifecycle stable while owned | observe `INVELOCITY`, `COMMANDABORTED`, and `ERROR` | drop `EXECUTE` when the surrounding action wrapper closes the lifecycle |
| `PressureHandle` | `EXECUTE` rising edge | keep lifecycle stable while owned | observe `INPRESSURE`, `BUSY/ACTIVE` clear, `COMMANDABORTED`, and `ERROR` | drop `EXECUTE` when the surrounding action wrapper closes the lifecycle |
| `Stop` | `EXECUTE` rising edge | keep lifecycle stable while decelerating | observe `DONE` after stop-to-zero | drop `EXECUTE` after `DONE` |
| `Hold` | `EXECUTE` rising edge | hold current runtime context | observe `DONE` after hold state is reached or `ERROR` | drop `EXECUTE` after transition is acknowledged |
| `Resume` | `EXECUTE` rising edge | resume a held runtime context | observe `DONE` after running state is restored or `ERROR` | drop `EXECUTE` after transition is acknowledged |
| `Reset` | `EXECUTE` rising edge | typically immediate lifecycle | observe `DONE` or `ERROR` | drop `EXECUTE` after completion |
| `MoveProfile` | `EXECUTE` rising edge | keep lifecycle stable while recipe execution is owned | observe `DONE`, `COMMANDABORTED`, or `ERROR` | drop `EXECUTE` after completion, takeover, or lifecycle invalidation |

## Recommended Usage by FB

### `MoveAbsolute`

Recommended for:

- position-driven clamp/open strokes
- eject forward/back strokes
- carriage position moves

Use it when:

- the process layer wants a terminal position move
- terminal completion should be observed with `DONE`

Do not use `MoveAbsolute` as:

- a complete machine-phase controller
- a substitute for process-layer decision logic

### `MoveVelocity`

Recommended for:

- fill/injection-style phases
- speed-ramp behavior
- phases where the machine layer decides when to leave the velocity phase

Use it when:

- the process layer wants a velocity-governed execution
- the process layer cares about in-band velocity tracking via `INVELOCITY`

Do not assume:

- `INVELOCITY` means the process phase is complete
- `MoveVelocity` will always terminate with `DONE`

### `PressureHandle`

Recommended for:

- hold-pressure phases
- pressure-maintenance phases
- phases that require pressure-in-band tracking

Use it when:

- the process layer wants pressure closed-loop control
- the process layer wants `INPRESSURE` tracking

Do not assume:

- `PressureHandle` behaves exactly like `MoveAbsolute` with respect to terminal `DONE`
- the library decides when hold phase starts or ends

### `Stop`

Recommended for:

- controlled stop insertion into an active motion lifecycle
- process-layer stop requests that require deceleration to zero

Use it when:

- the process layer wants a controlled stop
- an active direct owner must be decelerated to zero

Do not assume:

- `Stop` is equivalent to immediate `Abort`
- `DONE` is valid on the trigger cycle during active motion

### `Hold` / `Resume`

Recommended for:

- temporarily pausing an active runtime context without declaring terminal completion
- resuming the same runtime context after a process-layer pause condition clears

Use them when:

- the process layer wants runtime pause/resume semantics rather than a controlled stop
- the active segment context should be preserved

Do not assume:

- `Hold` or `Resume` is a machine phase transition
- `Hold.DONE` means the action completed normally
- pause/resume replaces valve safety logic or interlocks

### `Reset`

Recommended for:

- clearing execution state
- recovering to a reusable motion context while preserving configured source data

Use it when:

- the process layer wants to terminate current execution semantics and return the axis to a reusable state

Do not assume:

- `Reset` is a normal completion of the previously running command

### `MoveProfile`

Recommended for:

- recipe-driven multi-segment motion
- process-layer controlled sequencing where recipe semantics are desired

Use it when:

- the process layer wants recipe ownership and segment progression

Do not assume:

- recipe execution can only terminate through normal `DONE`
- recipe takeover should be inferred by recomputing ownership from unrelated runtime fields

## How to Consume Runtime Signals

The PLC process layer should consume each signal according to the formal contract defined in [motion-runtime-contract.md](/home/dan/project/hdy-motion-light/docs/architecture/motion-runtime-contract.md).

This section does not redefine signal semantics. It only translates those semantics into usage guidance.

| Signal | Use It For | Do Not Use It For |
| --- | --- | --- |
| `DONE` | observing terminal completion of FBs that expose terminal `DONE` | assuming every control phase is complete just because a target band is reached |
| `BUSY` | knowing that a lifecycle is still in progress | inferring that a machine phase should always continue |
| `ACTIVE` | knowing the FB currently owns active execution | treating it as proof of target achievement |
| `COMMANDABORTED` | detecting takeover or ownership loss | treating it as a fault condition |
| `ERROR` | fault/invalid lifecycle handling | treating it as equivalent to preemption |
| `INVELOCITY` | velocity-in-band observation | treating it as universal terminal completion |
| `INPRESSURE` | pressure-in-band observation | treating it as universal terminal completion |
| `STATE.vpTransferReady` | observing a library-computed injection fill transfer recommendation | allowing the runtime to switch phases or valves automatically |
| `STATE.vpTransferReason` | identifying which transfer criterion raised the recommendation | using it without process interlock validation |

### Signal Consumption Rules

1. Treat `DONE` as a terminal-completion observation only where the runtime contract says it applies.
2. Treat `INVELOCITY` and `INPRESSURE` as in-band control observations, not as universal phase-completion signals.
3. Treat `COMMANDABORTED` as an ownership-loss branch.
4. Treat `ERROR` as an error/recovery branch.

## Recommended PLC-Side Abstraction Pattern

The PLC process layer should not scatter raw FB calls across the top-level machine sequence.

Recommended PLC-side structure:

1. `Motion FB Layer`
   - direct use of library FBs
2. `Action Wrapper Layer`
   - project-level action wrappers around library FBs
3. `Process Sequence Layer`
   - full machine workflow and phase transitions

The action wrapper layer should:

- own FB triggering patterns
- own action-local `EXECUTE` lifecycle control
- expose clean action-level outputs to the top-level process sequence

The top-level process sequence should:

- decide when an action starts
- decide when the machine transitions to another action
- own interlocks and valve logic

## Typical Integration Scenarios

### Clamp Close

- PLC decides clamp-close phase entry
- PLC configures a position move
- Library executes the move and reports status
- PLC uses runtime outputs plus machine interlocks to decide the next phase
- PLC controls low-pressure/high-pressure valve logic itself

### Injection Fill

- PLC decides fill-phase entry
- PLC triggers `MoveVelocity`
- Library executes the speed-ramp behavior
- Library may report `STATE.vpTransferReady` and `STATE.vpTransferReason`
- PLC treats V/P readiness as a transfer recommendation
- PLC validates interlocks, stops or closes the fill lifecycle, commands holding pressure, and switches valves explicitly

### Hold Pressure

- PLC decides hold-phase entry
- PLC triggers `PressureHandle`
- Library executes pressure control and reports `INPRESSURE`, `BUSY`, `ACTIVE`, diagnostics
- PLC decides hold-phase exit

### Controlled Stop

- PLC triggers `Stop`
- Library executes controlled deceleration
- PLC waits for `Stop.DONE`
- PLC then clears the stop lifecycle and determines the next machine action

### Hold and Resume

- PLC detects a process condition that requires a temporary pause
- PLC triggers `Hold`
- Library enters hold state and clears active actuation outputs
- PLC keeps valve and machine outputs in a process-safe state
- PLC triggers `Resume` only after interlocks and process conditions allow the motion to continue

## Error and Recovery Handling

The PLC process layer should keep recovery policy outside the library, while relying on official runtime semantics to detect what happened.

### Recommended PLC Reaction Categories

| Runtime Observation | PLC-Side Meaning | Recommended PLC Response |
| --- | --- | --- |
| `COMMANDABORTED` | command lost ownership | treat as takeover/lifecycle branch, not as a fault |
| `ERROR` | runtime or command fault surfaced | enter recovery/error branch |
| `Stop.DONE` | controlled stop complete | decide next machine-safe phase |
| `Reset.DONE` | execution state cleared | rebuild action lifecycle as needed |
| `BUSY/ACTIVE` clear on nonterminal FBs | phase may have completed or been invalidated | check the owning action logic and diagnostics |

## Common Mistakes

Avoid these integration errors:

1. Treating every FB as if `DONE` meant the same thing.
2. Assuming `Stop` is complete on the trigger cycle.
3. Recomputing ownership/preemption logic in the PLC process layer.
4. Reading internal runtime fields instead of documented outputs.
5. Letting the top-level machine state machine call raw FBs everywhere without an action-wrapper layer.
6. Treating `COMMANDABORTED` as equivalent to `ERROR`.
7. Letting the library decide V/P transfer or machine phase transitions.
8. Treating `STATE.vpTransferReady` as permission to bypass machine interlocks.

## Must Remain in PLC

The following concerns must remain in the PLC process layer:

- valve commands
- machine phase sequencing
- V/P transfer decisions
- mold-protect decisions
- eject repeat-count logic
- carriage/nozzle contact decisions
- machine-specific interlocks
- alarm workflow and recovery policy

## Recommended Next Step

Use this guide together with:

- [motion-runtime-contract.md](/home/dan/project/hdy-motion-light/docs/architecture/motion-runtime-contract.md)
- [control-layer-boundary.md](/home/dan/project/hdy-motion-light/docs/architecture/control-layer-boundary.md)

The runtime contract defines signal and ownership semantics.
The boundary document defines responsibility ownership.
This guide explains how the PLC process layer should consume both. 
