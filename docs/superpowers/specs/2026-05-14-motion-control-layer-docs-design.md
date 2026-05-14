# Motion Control Layer Documentation Design

Date: 2026-05-14

## Goal

Stabilize the project's intended `middle-layer` architecture:

- The PLC process layer owns hydraulic mechanism logic, valve control, phase switching, interlocks, and machine workflow.
- This library owns motion-execution semantics, mathematical control, pump-side command outputs, diagnostics, and reusable IEC function blocks.

The immediate deliverable is not new runtime behavior. It is a documentation set that formalizes boundaries, runtime contracts, PLC integration rules, and reusable motion-profile archetypes so future development stays aligned with the intended separation of responsibilities.

## Why This Matters

The current codebase already reflects much of this boundary in practice:

- `motion_control.*` acts as the motion runtime and control core.
- `motion_interface.*` acts as the IEC/PLCopen-facing adapter layer.
- valve actions and process sequencing are intentionally left outside the library.

However, the contract is still spread across code comments, design notes, tests, and implementation details. That creates three risks:

1. PLC engineers may consume unstable or implicit runtime behavior.
2. Future feature work may push machine/process logic down into the library.
3. The repo may be described as an injection-molding motion library while behaving more like a generic hydraulic motion kernel.

The documentation set below is meant to close that gap.

## Architecture Position

This project should keep the following long-term identity:

- Not a pure mathematical toolbox.
- Not a full machine-sequence controller.
- A motion-execution kernel that is friendly to PLC process-layer integration.

That means:

- The library standardizes motion semantics, execution ownership, completion behavior, diagnostics, and pump-side outputs.
- The PLC process layer standardizes machine actions such as clamp close/open, injection fill, hold pressure, eject, carriage motion, and machine-specific hydraulic sequencing.

## Recommended Documentation Set

### 1. `docs/architecture/motion-runtime-contract.md`

Purpose:

- Define the formal runtime contract of the motion-execution layer.

Must cover:

- `FB_STATE`, `STATE.*`, `SEGMENT_COMPLETED`
- recipe/direct execution ownership
- `owner_kind`, `session_state`, `execution_id`
- semantics of `Start/Next/Stop/Hold/Resume/Abort/Reset/Ack`
- `DONE/BUSY/ACTIVE/COMMANDABORTED/ERROR/INVELOCITY/INPRESSURE`
- terminal-completion matrix
- fields safe for PLC consumption vs internal-only fields
- current unsupported semantics

This is the highest-priority document because the other three depend on it.

### 2. `docs/architecture/control-layer-boundary.md`

Purpose:

- Lock down what belongs to the library and what belongs to the PLC process layer.

Must cover:

- 4-layer view: algorithm, runtime, IEC adapter, PLC process layer
- library responsibilities
- PLC process-layer responsibilities
- explicit non-goals for the library
- positive and negative examples
- extension rules for future features

This document prevents architecture drift.

### 3. `docs/integration/plc-process-layer-integration-guide.md`

Purpose:

- Show PLC engineers how to use the motion layer correctly.

Must cover:

- scan-cycle calling pattern
- `EXECUTE` edge-handling rules
- recommended per-FB usage
- how to consume runtime signals
- recommended PLC-side action wrapping
- common integration mistakes
- what still must be implemented in PLC

This document translates architecture and contracts into practical usage guidance.

### 4. `docs/architecture/motion-profile-archetypes.md`

Purpose:

- Provide standardized control-layer templates without taking over process logic.

Must cover:

- axis archetypes: clamp, injection, hold, ejector, carriage
- segment archetypes: clamp close/open, injection fill, hold pressure, eject, carriage
- recommended mode/planner/end-condition choices
- parameter grouping guidance
- override rules
- unsupported/deferred semantics

This document improves reuse and reduces parameter drift in PLC projects.

## Recommended Writing Order

1. `motion-runtime-contract.md`
2. `control-layer-boundary.md`
3. `plc-process-layer-integration-guide.md`
4. `motion-profile-archetypes.md`

Reason:

- First define what the library guarantees.
- Then define what the library must not absorb.
- Then explain how PLC code should integrate with it.
- Finally standardize reusable control templates.

## Detailed Outline: Runtime Contract

The first document should use a specification style and include at least:

1. Purpose and scope
2. Runtime objects
3. State model
4. Execution ownership model
5. Command contract
6. Output signal contract
7. Completion semantics
8. Recipe vs direct boundary
9. Error and protection contract
10. PLC dependency rules
11. Worked examples
12. Not supported

Required artifacts inside the document:

- state table
- session-state table
- signal-contract table
- terminal-semantic matrix
- worked examples for:
  - normal move completion
  - stop deceleration completion
  - command takeover/preemption

## Detailed Outline: Control Boundary

The second document should include:

1. Purpose
2. Layer overview
3. Library responsibilities
4. PLC process-layer responsibilities
5. Explicit library non-goals
6. Shared-boundary rules
7. Boundary rules
8. Positive examples
9. Negative examples
10. Extension policy
11. Review checklist

## Detailed Outline: PLC Integration Guide

The third document should include:

1. Purpose
2. Integration model
3. Scan-cycle call pattern
4. `EXECUTE` usage rules
5. Recommended usage by FB
6. Recommended process-layer wrapping pattern
7. How to consume runtime signals
8. Typical integration scenarios
9. Error and recovery handling
10. Common mistakes
11. Recommended PLC-side abstractions
12. Not supported / must remain in PLC

## Detailed Outline: Motion Profile Archetypes

The fourth document should include:

1. Purpose
2. Archetype design principles
3. Axis archetypes
4. Segment archetypes
5. Parameter recommendations
6. Mapping between machine actions and control modes
7. Override rules
8. Unsupported or deferred semantics
9. Example template definitions

## Constraints

The documentation set must preserve the intended architecture:

- The library does not directly control valves.
- The library does not own machine phase sequencing.
- The library does not decide V/P transfer conditions.
- The library does not encode machine-specific interlocks.
- The PLC process layer must not reimplement motion-runtime completion semantics.

## Recommended Near-Term Follow-Up

After the documentation set is approved, the next implementation planning step should be:

1. write `motion-runtime-contract.md`
2. compare the current code and tests against that contract
3. identify semantic mismatches
4. plan only the smallest runtime/interface adjustments needed to make implementation and contract converge

This keeps the project on the intended `middle-layer` trajectory instead of drifting either toward a thin math-only utility or toward a machine-sequence controller.
