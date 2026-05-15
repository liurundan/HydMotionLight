# Motion Profile Archetypes

Date: 2026-05-14

## Purpose

This document defines recommended motion-profile archetypes for HydroMotionLib.

These archetypes are intended to:

- standardize how common machine actions map onto the library's control modes
- reduce parameter drift across PLC projects
- provide reusable control-layer defaults without taking over machine workflow decisions

This document does not define:

- machine phase sequencing
- V/P transfer decisions or automatic transfer execution
- solenoid-valve behavior
- machine-specific interlocks
- product- or mold-specific process decisions

## Archetype Design Principles

1. Archetypes define control-parameter structure, not machine workflow.
2. Archetypes must stay compatible with the existing motion runtime contract.
3. Archetypes should prefer the library's current supported semantics over exposing unimplemented behavior.
4. PLC process logic may override archetype values, but should preserve the intended control meaning of the archetype.
5. If an archetype starts to encode machine workflow, it no longer belongs in this document.

## Axis Archetypes

Axis archetypes define default control expectations by machine axis role.

### 1. Clamp Axis

Typical uses:

- clamp close
- clamp open
- mold protection approach

Recommended control emphasis:

- position accuracy
- controlled deceleration near target
- explicit timeout protection

Recommended defaults:

| Field | Recommendation |
| --- | --- |
| Primary mode | `HYD_MODE_POSITION` |
| Common end condition | `HYD_END_POSITION` |
| Typical direction use | `EXTEND` for close, `RETRACT` for open |
| Planner preference | `HYD_PLANNER_POSITION_BASED` or `HYD_PLANNER_TIME_BASED` with position braking protection |
| Key protections | position tolerance, timeout |

### 2. Injection Axis

Typical uses:

- injection fill
- injection continuation under time or position constraint

Recommended control emphasis:

- speed/flow behavior
- smooth acceleration
- controlled response to stop or takeover

Recommended defaults:

| Field | Recommendation |
| --- | --- |
| Primary mode | `HYD_MODE_SPEED_RAMP` |
| Common end condition | `HYD_END_POSITION`, `HYD_END_TIME`, or PLC-driven phase switch |
| Typical direction use | `EXTEND` |
| Planner preference | `HYD_PLANNER_TIME_BASED` |
| Key protections | velocity tolerance, flow tolerance, timeout where applicable |

### 3. Hold-Pressure Axis Role

Typical uses:

- hold pressure
- pressure maintenance

Recommended control emphasis:

- pressure closed-loop stability
- feedforward flow bias
- pressure ramping into target

Recommended defaults:

| Field | Recommendation |
| --- | --- |
| Primary mode | `HYD_MODE_PRESSURE_CLOSED_LOOP` |
| Common end condition | `HYD_END_TIME` or `HYD_END_MANUAL` |
| Typical direction use | `HOLD` |
| Planner preference | not planner-driven in the same way as position/velocity modes |
| Key protections | pressure tolerance, flow tolerance, timeout |

### 4. Ejector Axis

Typical uses:

- eject forward
- eject retract
- repeated eject strokes orchestrated by PLC

Recommended control emphasis:

- short-stroke repeatability
- clean position completion semantics

Recommended defaults:

| Field | Recommendation |
| --- | --- |
| Primary mode | `HYD_MODE_POSITION` |
| Common end condition | `HYD_END_POSITION` |
| Typical direction use | `EXTEND` forward, `RETRACT` backward |
| Planner preference | `HYD_PLANNER_POSITION_BASED` |
| Key protections | position tolerance, timeout |

### 5. Carriage Axis

Typical uses:

- carriage forward
- carriage backward

Recommended control emphasis:

- deterministic position movement
- explicit timeout protection
- process-layer-controlled phase transitions

Recommended defaults:

| Field | Recommendation |
| --- | --- |
| Primary mode | `HYD_MODE_POSITION` |
| Common end condition | `HYD_END_POSITION` |
| Typical direction use | `EXTEND` or `RETRACT` depending on machine convention |
| Planner preference | `HYD_PLANNER_POSITION_BASED` |
| Key protections | position tolerance, timeout |

## Segment Archetypes

Segment archetypes provide reusable control patterns. They are not complete process phases.

### `ClampCloseSegment`

Recommended use:

- main clamp-close motion under position control

Recommended structure:

| Field | Recommendation |
| --- | --- |
| `mode` | `HYD_MODE_POSITION` |
| `endCondition` | `HYD_END_POSITION` |
| `direction` | `HYD_DIRECTION_EXTEND` |
| `planner` | `HYD_PLANNER_POSITION_BASED` or `HYD_PLANNER_TIME_BASED` |
| Required values | `targetPosition`, `maxVelocity`, `maxAcceleration` |
| Recommended safeguards | `positionTolerance`, `timeoutLimit` |

### `ClampOpenSegment`

Recommended use:

- clamp-open motion under position control

Recommended structure:

| Field | Recommendation |
| --- | --- |
| `mode` | `HYD_MODE_POSITION` |
| `endCondition` | `HYD_END_POSITION` |
| `direction` | `HYD_DIRECTION_RETRACT` |
| `planner` | `HYD_PLANNER_POSITION_BASED` |
| Required values | `targetPosition`, `maxVelocity`, `maxAcceleration` |
| Recommended safeguards | `positionTolerance`, `timeoutLimit` |

### `InjectionFillSegment`

Recommended use:

- fill/injection motion under velocity-governed behavior

Recommended structure:

| Field | Recommendation |
| --- | --- |
| `mode` | `HYD_MODE_SPEED_RAMP` |
| `direction` | `HYD_DIRECTION_EXTEND` |
| `planner` | `HYD_PLANNER_TIME_BASED` |
| Common end conditions | `HYD_END_POSITION`, `HYD_END_TIME`, or process-layer phase switch |
| Required values | `maxVelocity`, `maxAcceleration`, `maxFlow`, `velocityToFlowGain` |
| Optional values | `targetPosition`, `targetFlow`, `duration`, V/P observation thresholds |
| Recommended safeguards | `velocityTolerance`, `flowTolerance`, `timeoutLimit` |

V/P transfer thresholds on this segment are observation criteria only. They may set `STATE.vpTransferReady` during runtime, but the PLC process layer still decides when to close the fill lifecycle, start holding pressure, and switch valves.

### `HoldPressureSegment`

Recommended use:

- pressure-maintenance / hold-pressure control

Recommended structure:

| Field | Recommendation |
| --- | --- |
| `mode` | `HYD_MODE_PRESSURE_CLOSED_LOOP` |
| `direction` | `HYD_DIRECTION_HOLD` |
| Common end conditions | `HYD_END_TIME`, `HYD_END_MANUAL` |
| Required values | `targetPressure`, `maxFlow` |
| Recommended support values | `targetFlow`, `pressureRampRate`, `pressureController` |
| Recommended safeguards | `pressureTolerance`, `flowTolerance`, `timeoutLimit` |

### `EjectAdvanceSegment`

Recommended use:

- forward eject stroke

Recommended structure:

| Field | Recommendation |
| --- | --- |
| `mode` | `HYD_MODE_POSITION` |
| `endCondition` | `HYD_END_POSITION` |
| `direction` | `HYD_DIRECTION_EXTEND` |
| Required values | `targetPosition`, `maxVelocity`, `maxAcceleration` |
| Recommended safeguards | `positionTolerance`, `timeoutLimit` |

### `EjectRetractSegment`

Recommended use:

- return eject stroke

Recommended structure:

| Field | Recommendation |
| --- | --- |
| `mode` | `HYD_MODE_POSITION` |
| `endCondition` | `HYD_END_POSITION` |
| `direction` | `HYD_DIRECTION_RETRACT` |
| Required values | `targetPosition`, `maxVelocity`, `maxAcceleration` |
| Recommended safeguards | `positionTolerance`, `timeoutLimit` |

### `CarriageForwardSegment`

Recommended use:

- carriage approach / forward position move

Recommended structure:

| Field | Recommendation |
| --- | --- |
| `mode` | `HYD_MODE_POSITION` |
| `endCondition` | `HYD_END_POSITION` |
| `direction` | machine-specific `EXTEND`/`RETRACT`, but explicit |
| Required values | `targetPosition`, `maxVelocity`, `maxAcceleration` |
| Recommended safeguards | `positionTolerance`, `timeoutLimit` |

### `CarriageBackwardSegment`

Recommended use:

- carriage retract / backward position move

Recommended structure:

| Field | Recommendation |
| --- | --- |
| `mode` | `HYD_MODE_POSITION` |
| `endCondition` | `HYD_END_POSITION` |
| `direction` | opposite of the machine's forward convention |
| Required values | `targetPosition`, `maxVelocity`, `maxAcceleration` |
| Recommended safeguards | `positionTolerance`, `timeoutLimit` |

## Parameter Recommendations

Archetypes should define parameter groups, not fixed machine values.

### Core Parameter Groups

| Group | Typical Fields | Notes |
| --- | --- | --- |
| Position-governed | `targetPosition`, `maxVelocity`, `maxAcceleration`, `positionTolerance`, `timeoutLimit` | Used for clamp, eject, carriage moves |
| Velocity-governed | `maxVelocity`, `maxAcceleration`, `maxFlow`, `velocityToFlowGain`, optional `targetFlow`, optional `duration` | Used for fill/injection-style segments |
| Pressure-governed | `targetPressure`, `targetFlow`, `maxFlow`, `pressureRampRate`, controller gains, `pressureTolerance`, `timeoutLimit` | Used for hold-pressure control |

### Tuning Guidance

1. `positionTolerance` should be chosen for motion accuracy, not for process-phase switching.
2. `velocityTolerance` and `flowTolerance` should support in-band tracking, not replace process decisions.
3. `timeoutLimit` should protect execution semantics, not implement machine workflow timers.
4. `targetFlow` in pressure mode should be treated as a feedforward bias, not as a valve command.
5. Direction must always be explicit when the segment is motion-bearing.

## Mapping Between Machine Actions and Control Modes

| Machine Action | Recommended Axis Archetype | Recommended Segment Archetype | Control Mode |
| --- | --- | --- | --- |
| Clamp close | Clamp Axis | `ClampCloseSegment` | Position |
| Clamp open | Clamp Axis | `ClampOpenSegment` | Position |
| Injection fill | Injection Axis | `InjectionFillSegment` | Speed ramp |
| Hold pressure | Hold-Pressure Axis Role | `HoldPressureSegment` | Pressure closed-loop |
| Eject advance | Ejector Axis | `EjectAdvanceSegment` | Position |
| Eject retract | Ejector Axis | `EjectRetractSegment` | Position |
| Carriage forward/back | Carriage Axis | `CarriageForwardSegment` / `CarriageBackwardSegment` | Position |

Process-phase ownership, valve logic, interlocks, and transfer criteria remain defined by the boundary and integration documents, not by this archetype mapping.

## Runtime Action Profile Helpers

The runtime provides helper builders for common injection-machine motion segment defaults:

- `HYD_ActionProfile_BuildClampClose`
- `HYD_ActionProfile_BuildClampOpen`
- `HYD_ActionProfile_BuildInjectionFill`
- `HYD_ActionProfile_BuildHoldingPressure`
- `HYD_ActionProfile_BuildEjectAdvance`
- `HYD_ActionProfile_BuildEjectRetract`
- `HYD_ActionProfile_BuildCarriageMove`

These helpers populate `HYD_MotionSegment` defaults only. They do not start motion, switch valves, decide V/P transfer, or own machine sequencing. PLC process logic may still override generated segment values before loading the recipe.

If a project configures V/P observation fields on an injection-fill segment, the helper output remains a segment template. The observation result is a runtime recommendation, not a process transition.

## Override Rules

1. PLC projects may override archetype parameter values.
2. Overrides should preserve the intended control meaning of the archetype.
3. If a move requires fundamentally different control semantics, create a new archetype rather than distorting an unrelated one.
4. Archetypes are starting points for standardization, not replacements for machine tuning.

## Unsupported or Deferred Semantics

This archetype set does not currently define:

- jerk-shaped control archetypes
- continuous-update archetype semantics
- true blending-mode archetypes beyond the currently supported runtime subset
- automatic V/P transfer or holding-pressure transition archetypes
- valve-pattern archetypes
- machine-sequence archetypes

## Example Template Definitions

### Example: Injection Fill Template

```text
mode = HYD_MODE_SPEED_RAMP
direction = HYD_DIRECTION_EXTEND
planner = HYD_PLANNER_TIME_BASED
endCondition = HYD_END_POSITION or HYD_END_TIME
required = maxVelocity, maxAcceleration, maxFlow, velocityToFlowGain
optional = targetPosition, targetFlow, duration
optional_observation = vpTransferPosition, vpTransferPressure, vpTransferMinTime, vpTransferVelocityDrop
```

### Example: Hold Pressure Template

```text
mode = HYD_MODE_PRESSURE_CLOSED_LOOP
direction = HYD_DIRECTION_HOLD
endCondition = HYD_END_TIME or HYD_END_MANUAL
required = targetPressure, maxFlow
recommended = targetFlow, pressureRampRate, controller gains, pressureTolerance
```

### Example: Clamp Close Template

```text
mode = HYD_MODE_POSITION
direction = HYD_DIRECTION_EXTEND
endCondition = HYD_END_POSITION
planner = HYD_PLANNER_POSITION_BASED or HYD_PLANNER_TIME_BASED
required = targetPosition, maxVelocity, maxAcceleration
recommended = positionTolerance, timeoutLimit
```

## Relationship to Other Documents

Use this document together with:

- [motion-runtime-contract.md](/home/dan/project/hdy-motion-light/docs/architecture/motion-runtime-contract.md)
- [control-layer-boundary.md](/home/dan/project/hdy-motion-light/docs/architecture/control-layer-boundary.md)
- [plc-process-layer-integration-guide.md](/home/dan/project/hdy-motion-light/docs/integration/plc-process-layer-integration-guide.md)

Those documents define:

- what the runtime guarantees
- what belongs in the library vs PLC
- how the PLC process layer should integrate with the library

This document only standardizes reusable control templates on top of those contracts. 
