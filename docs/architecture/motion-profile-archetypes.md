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

#### 低压护膜变体 (Low-Pressure Mold Protect)

> Introduced in Sprint 1.

合模段进入护膜窗口后,允许"实测压力 > 段配置 ceiling + tolerance"立即触发 DERATE,
持续超限超过 fault-escalation 阈值后升级为 STOP。适用场景:模具内异物、模板平行度异常、
顶针未完全回位、嵌件未到位等。

**推荐 builder:**

```c
HYD_ActionProfile_BuildClampCloseWithMoldProtect(
    &seg, &params, tag,
    /* targetPosition */ 100.0,    /* mm, 全合位置 */
    /* protectWindowStart */ 70.0, /* mm, 低速护膜窗口起点 */
    /* pressureCeiling */ 5.0,     /* MPa, 护膜段压力上限 */
    /* pressureCeilingTolerance */ 0.0,  /* 0 = 沿用 params->pressureTolerance */
    /* derateRatio */ 0.2);        /* 触发 ceiling-exceeded 时减速到 20% */
```

**激活窗口语义:**

- `pressureCeilingPositionStart < pressureCeilingPositionEnd` -> 窗口内激活
- `pressureCeilingPositionStart >= pressureCeilingPositionEnd`(含 0,0)-> 整段激活
- `pressureCeiling = 0` -> 整段禁用 ceiling 检查(默认状态)

**诊断码:**

| Code | Severity | Protection Action |
|---|---|---|
| `HYD_DIAG_CODE_PRESSURE_CEILING_EXCEEDED` | WARNING | DERATE |
| `HYD_DIAG_CODE_PRESSURE_CEILING_VIOLATED` | FAULT | STOP |

EXCEEDED -> VIOLATED 的升级由 `_pressureCeilingCriteria.faultEscalationTime`
控制(默认 300 ms)。PLC 在 WARNING 阶段会观察到 PUMP_SPEED 按 derateRatio 缩放;
在 FAULT 阶段会观察到 FB_STATE = FAULT、PUMP_SPEED = 0,需要通过 Abort+重启或
Reset 恢复。

**与 OVER_PRESSURE 的区别:**

- `OVER_PRESSURE`(legacy)只在 PRESSURE_CLOSED_LOOP mode 评估,语义是"实测压力偏离
  closed-loop reference 超过 pressureTolerance",用于压力 servo 跟踪。
- `PRESSURE_CEILING_EXCEEDED`(Sprint 1)在 POSITION / SPEED_RAMP /
  PRESSURE_CLOSED_LOOP 任意 mode 都评估,语义是"实测压力超过段配置的固定软上限",
  用于安全保护。两者可以共存:一个 PRESSURE_CLOSED_LOOP 段同时配置
  `pressureTolerance`(跟踪误差告警)和 `pressureCeiling`(安全上限保护)是合法且推荐
  的工艺配置。

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
