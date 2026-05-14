# Control Layer Boundary

Date: 2026-05-14

## Purpose

This document defines the responsibility boundary between:

- the HydroMotionLib motion-control library
- the PLC process layer that uses the library

The goal is to preserve the project's intended middle-layer architecture:

- the library owns motion-execution semantics, mathematical control, diagnostics, protection behavior, and pump-side command outputs
- the PLC process layer owns hydraulic mechanism logic, valve control, machine workflow, interlocks, and process-phase decision logic

This document is a boundary contract. It exists to prevent responsibility drift, duplicated logic, and accidental promotion of machine-specific process logic into the library.

## Layer Overview

| Layer | Responsibility | Inputs | Outputs | Must Not Do |
| --- | --- | --- | --- | --- |
| Algorithm Layer | Pure control/math primitives | segment parameters, measured feedback, gains | planned velocity/flow/pressure terms, controller outputs | own machine workflow or PLC semantics |
| Motion Runtime Layer | Segment execution, ownership, completion semantics, protection, diagnostics | validated commands, selected segments, feedback | runtime state, pump-side requests, diagnostics, protection state | own valve logic or machine process sequencing |
| IEC Adapter Layer | PLCopen-style FB mapping and lifecycle projection | IEC FB inputs, runtime state | IEC-facing outputs and lifecycle signals | become a second runtime or a process state machine |
| PLC Process Layer | Machine phases, valve actions, interlocks, sequencing, machine-specific decisions | runtime outputs, machine I/O, process recipes | stage transitions, valve commands, high-level action orchestration | reimplement runtime completion, ownership, or control math |

## Library Responsibilities

The library is responsible for the control-layer behavior that should be common across machine variants and PLC applications.

### 1. Motion-Execution Responsibilities

- Position control
- Velocity/flow control
- Pressure closed-loop control
- Direct segment execution
- Recipe segment execution
- Controlled stop semantics
- Hold/resume semantics
- Reset and abort semantics

### 2. Runtime Semantics Responsibilities

- Command legality by runtime state
- Pending-command consumption
- Execution ownership
- Preemption/takeover behavior
- Segment completion semantics
- Final completion semantics
- Protection-state transitions

### 3. Diagnostics and Protection Responsibilities

- Runtime deviation detection
- Timeout detection
- Fault escalation
- Protection-action reporting
- Fault snapshot retention
- Live and retained diagnostic output

### 4. Output Responsibilities

- Pump-speed / pump-side command output
- Planned direction semantics
- Runtime status output
- Diagnostic and protection outputs

## PLC Process-Layer Responsibilities

The PLC process layer is responsible for machine-specific behavior, hydraulic mechanism logic, and process decisions.

### 1. Machine Workflow Responsibilities

- Clamp close/open workflow
- Injection fill workflow
- Hold-phase workflow
- Plasticizing / storage workflow
- Ejector workflow
- Carriage forward/back workflow
- Full molding-cycle sequencing

### 2. Hydraulic Mechanism Responsibilities

- Solenoid-valve control
- Valve-group coordination
- Hydraulic circuit switching
- Regeneration / unloading logic
- Back-pressure and decompression mechanism handling

### 3. Process Decision Responsibilities

- V/P transfer criteria
- Mold-contact / mold-protect decisions
- Cushion / material-end decisions
- Eject-count and repeat-stroke decisions
- Stage-to-stage transition criteria

### 4. Machine Interlock Responsibilities

- Safety interlocks
- Axis-to-axis process coordination
- Machine-specific permission logic
- Alarm reaction policy
- Shared-resource orchestration at the machine workflow level

## Explicit Library Non-Goals

The library must not directly own the following:

- direct solenoid-valve control
- machine phase sequencing
- V/P transfer decision logic
- mold-protect strategy decisions
- ejector cycle counting logic
- carriage/nozzle contact logic
- machine-specific hydraulic interlocks
- product-specific molding-process decisions

If a feature requires product, mold, machine topology, or hydraulic-circuit knowledge to decide process flow, it belongs outside the library.

## Shared-Boundary Rules

Some concerns are shared across the boundary, but the ownership split must still remain explicit.

### Direction

- The library owns direction semantics for execution (`EXTEND`, `RETRACT`, `HOLD`, `AUTO`).
- The PLC process layer owns how those semantics map to actual valve actions.

### Segment Templates

- The library may provide reusable segment or axis archetypes.
- The PLC process layer owns when those templates are selected and how they are sequenced.

### Diagnostics

- The library owns detection and reporting.
- The PLC process layer owns machine response and alarm workflow.

### Protection

- The library owns protection-state generation and reporting.
- The PLC process layer owns what the rest of the machine does in response.

## Boundary Rules

1. The PLC process layer must not reimplement runtime completion or ownership semantics that are already provided by the library.
2. The library must not decide machine process-phase transitions.
3. Valve actions must not be encoded in the motion runtime layer.
4. Machine-specific hydraulic sequencing must not be encoded in generic segment-runtime behavior.
5. If a rule depends on mold, product, machine topology, or circuit layout to decide flow, the rule belongs in PLC logic.
6. If a rule is about execution ownership, segment completion, protection, or control-mode semantics, it belongs in the library.

## Positive Examples

### Example 1: Clamp Close with Mold Protect

- Library responsibility:
  - execute the selected close segment
  - report runtime state, motion completion, diagnostics, and pump request
- PLC responsibility:
  - decide when clamp close begins
  - control valves for low-pressure / high-pressure hydraulic modes
  - decide when mold-protect logic causes a transition or alarm

### Example 2: Injection Fill to Hold

- Library responsibility:
  - execute injection speed segment
  - execute hold-pressure control segments once requested
  - report completion, deviations, and protection state
- PLC responsibility:
  - decide when fill starts
  - decide the V/P transfer condition
  - switch process phase from fill to hold
  - decide when the hold phase begins and ends

### Example 3: Ejector Repeat Strokes

- Library responsibility:
  - execute each eject forward/back segment
  - report completion and preemption/fault state
- PLC responsibility:
  - decide stroke count
  - decide when the repeat loop ends
  - coordinate with mold-open permissions and interlocks

## Negative Examples

### Anti-Example 1

Incorrect:

- the library automatically decides “pressure reached, now enter hold phase”

Reason:

- that is a process-phase decision and belongs in PLC logic

### Anti-Example 2

Incorrect:

- the PLC process layer infers stop-to-zero by directly reading internal stop bookkeeping instead of consuming official runtime semantics

Reason:

- that duplicates runtime ownership/completion logic and breaks the contract boundary

### Anti-Example 3

Incorrect:

- the motion runtime directly emits machine-specific valve patterns for one injection-machine hydraulic circuit

Reason:

- that collapses machine mechanism logic into a supposedly reusable control library

## Extension Policy

When adding a new feature, place it by the following rules:

| New Capability Type | Layer |
| --- | --- |
| Pure mathematical control logic | Algorithm Layer |
| Execution ownership / completion / stop-reset semantics | Motion Runtime Layer |
| PLC FB input/output mapping | IEC Adapter Layer |
| Stage logic / valve logic / machine interlocks | PLC Process Layer |
| Reusable control parameter templates | Library, as profile/archetype support |

When in doubt:

- if the feature changes how execution itself behaves, it is library/runtime territory
- if the feature changes when the machine should move or which hydraulic path should be enabled, it is PLC territory

## Review Checklist

Use this checklist when reviewing future changes:

- Does this change introduce valve logic into the library?
- Does this change introduce machine phase sequencing into the runtime?
- Does this change make PLC logic recompute runtime semantics already owned by the library?
- Does this change encode a machine-specific hydraulic mechanism into generic motion behavior?
- Does this change belong in a reusable archetype/template instead of hard-coded runtime behavior?
