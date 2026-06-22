# RBF-PID Flow-Domain Migration Design

Date: 2026-06-22

## Goal

Refactor the current RBF-PID pressure controller so that the adaptive controller operates entirely in flow space (`L/min`) from the pressure-loop entry point through RBF identification, incremental PID output, compensation, and limiting. The only remaining `flow -> pump speed` conversion must happen in `src/pump_converter.c`.

The change also corrects two identified algorithm deviations in `src/rbf_pid.c`:

1. the RBF input timing must use causal history rather than mixing current-step signals
2. the RBF input vector must not include current pressure feedback

The implementation must be validated against the existing pressure-control simulation chain and must demonstrate pressure overshoot below `5%`.

## User-Confirmed Scope

- Modify the RBF-PID path rooted in:
  - `src/rbf_pid.c`
  - `include/rbf_pid.h`
  - `src/pressure_controller.c`
  - related tests and simulation-facing validation
- Change controller parameter semantics so `KP / KI / KD` become new flow-domain coefficients
- Re-tune default gains, adaptive gain windows, and learning rates as needed to keep the loop stable
- Validate with the simulation model already used by the repository
- Closed-loop performance target for this phase: overshoot `< 5%`

## Explicit Non-Goals

- do not implement online `K` identification in this phase
- do not implement a Smith predictor in this phase
- do not redesign the entire pressure model
- do not replace RBF-PID with another control strategy
- do not preserve old `KP / KI / KD` numeric values as-is

## Problem Summary

The current implementation mixes two incompatible control domains:

- the adaptive controller internally carries motor-speed-domain assumptions
- the surrounding pressure controller and segment configuration expose flow-domain limits and commands

This leaks into several places:

- `flowToPumpSpeedGain` participates inside `src/rbf_pid.c` control output, compensation, and limiting
- the RBF input uses `du_prev` and pressure feedback normalized by the same pressure scalar
- current pressure is present in the RBF input path, which weakens the causal identification structure
- the existing parameter defaults implicitly assume the old domain and become too aggressive when moved directly into flow space

The result is a controller that is harder to reason about physically and harder to stabilize consistently.

## Options Considered

### Option A: Direct flow-domain migration with local RBF-PID refactor

Move the full adaptive controller to `L/min`, fix the RBF input timing and normalization contract, re-tune gains and learning rates, and keep `pump_converter` as the sole actuator mapping.

Pros:

- aligns the implementation with the intended physical control variable
- removes repeated unit conversion logic from the adaptive loop
- gives a clean basis for later adaptive enhancements

Cons:

- requires retuning defaults and updating tests
- changes the meaning of existing segment-level `KP / KI / KD` values

### Option B: Keep the old internal domain and wrap it with conversions

Leave the current RBF-PID structure mostly intact and add conversions before and after the adaptive update.

Pros:

- smaller superficial code change

Cons:

- preserves the domain-mixing problem
- leaves the input-timing issue only partially corrected
- increases long-term maintenance cost

### Option C: Broader architectural split of identification and control

Rebuild the adaptive path as separate identification and control modules before migrating to flow space.

Pros:

- cleanest long-term architecture

Cons:

- too large for this phase
- unnecessarily broad validation surface

## Recommendation

Use **Option A**.

This is the smallest approach that actually fixes the two approved deviations, establishes a consistent physical control domain, and gives a realistic path to the `< 5%` overshoot target without adding unrelated architecture work.

## Target Architecture

The pressure-control chain for the adaptive branch must become:

```text
HYD_PressureController_Execute
    -> RBF_PID_Update              // full adaptive loop in L/min
    -> flow-domain limiting
    -> HYD_PumpConverter_Execute   // only L/min -> rpm mapping
    -> plant / simulation / hardware response
```

### Module Responsibilities

#### `src/rbf_pid.c`

Owns:

- RBF identification state
- Jacobian estimation
- adaptive `KP / KI / KD`
- incremental PID computation in `L/min`
- flow-domain compensation and flow-domain limiting

Must not own:

- motor-speed-domain output state
- `L/min -> rpm` conversion logic

#### `src/pressure_controller.c`

Owns:

- segment-level configuration resolution
- filtered pressure and filtered pressure rate
- selection of strategy
- output min/max handoff in `L/min`
- synchronization and reset logic around segment changes

#### `src/pump_converter.c`

Owns:

- the only mapping from requested flow to pump speed
- pump-speed limit enforcement derived from physical pump configuration

## Flow-Domain State Contract

After the migration, the following `RBF_PID_Handle` fields must all represent flow-space quantities:

- `u_prev`
- `du`
- `du_prev`
- `Output`
- `n_out` if retained

If `n_out` remains in the struct for compatibility, it should represent the final commanded flow, not a hidden motor-speed-domain output.

`flowToPumpSpeedGain` may still exist in outer layers for initialization and for `pump_converter`, but it must no longer shape the adaptive controller's internal control law.

## RBF Input Timing and Identification Contract

The identification path must use a strictly causal historical input vector:

```text
x(k) = [
  Δq(k-1),
  y(k-1),
  y(k-2),
  e(k-1)
]
```

Where:

- `q(k)` is the absolute commanded flow in `L/min`
- `Δq(k) = q(k) - q(k-1)`
- `y(k)` is measured pressure
- `e(k) = r(k) - y(k)`

### Required Changes

- remove current pressure `y(k)` from the RBF input vector
- do not feed current-step error `e(k)` into the RBF input if that path would indirectly reintroduce `y(k)`
- preserve the current control-step ordering:
  1. read current pressure
  2. compute current pressure error
  3. evaluate RBF using historical input vector
  4. estimate Jacobian
  5. update adaptive gains
  6. compute `Δq(k)`
  7. apply compensation and flow-domain limits
  8. update stored history for the next cycle

### Identification Meaning

The RBF network should estimate:

```text
ŷ(k) = f(Δq(k-1), y(k-1), y(k-2), e(k-1))
```

This keeps the identification structure causal and avoids target leakage from current-step pressure.

## Normalization Design

The current logic normalizes `du_prev` and pressure quantities by the same pressure scalar. That is not dimensionally sound and must be changed.

### Required Separation

Use two normalization scales:

- `flow_normalization_scale`
- `pressure_normalization_scale`

The normalized input vector becomes:

```text
x0_n = Δq(k-1) / flow_scale
x1_n = y(k-1)  / pressure_scale
x2_n = y(k-2)  / pressure_scale
x3_n = e(k-1)  / pressure_scale
```

The normalized prediction target becomes:

```text
y_n = y(k) / pressure_scale
ŷ_n = Σ w_j h_j
e_rbf_n = y_n - ŷ_n
```

### Jacobian Recovery

The internal neural derivative is computed in normalized space:

```text
J_n = ∂ŷ_n / ∂x0_n
```

The physical Jacobian used by the adaptive PID update must be:

```text
J = (pressure_scale / flow_scale) * J_n
```

This gives the intended physical meaning:

```text
J = ∂y / ∂Δq
```

with pressure per flow-increment units.

### Recommended Scale Sources

- `flow_scale`: use the effective segment-level max flow, with a stable positive fallback
- `pressure_scale`: use the effective pressure ceiling if available, otherwise a stable positive full-scale fallback

The implementation must guard against zero or invalid scales before any normalization.

## Flow-Domain PID Law

The adaptive controller must use the incremental PID law directly in `L/min`:

```text
Δq(k) = Kp·[e(k)-e(k-1)]
      + Ki·e(k)
      + Kd·[e(k)-2e(k-1)+e(k-2)]

q(k) = q(k-1) + Δq(k)
```

All adaptive parameter updates remain based on:

```text
ΔKp = ηp · e(k) · J · [e(k)-e(k-1)]
ΔKi = ηi · e(k) · J · e(k)
ΔKd = ηd · e(k) · J · [e(k)-2e(k-1)+e(k-2)]
```

but now `J` is the flow-domain Jacobian `∂y/∂Δq`.

## Parameter Semantics and Migration Policy

### New Meaning of `KP / KI / KD`

After this change, segment-level and default `KP / KI / KD` values must be interpreted as flow-domain discrete incremental PID coefficients.

The implementation must not claim numeric backward compatibility with the old values.

### Migration Starting Point

For initial retuning and for any temporary translation helpers, a reasonable seed is:

```text
K_new ≈ K_old / flowToPumpSpeedGain
```

for `Kp`, `Ki`, and `Kd`.

This is only a starting point, not the final tuning policy.

### Learning-Rate Rescaling

Because the Jacobian moves to `∂y/∂Δq`, the adaptive update magnitude also changes. A safe migration starting point is:

```text
η_new ≈ η_old / flowToPumpSpeedGain²
```

for:

- `eta_p`
- `eta_i`
- `eta_d`

Final values should be chosen by simulation-backed tuning, not fixed only by formula.

## Compensation and Limiting Policy

### Compensation

The current output-end multiplication by `gain_compensation_factor` should not survive in its current form.

In the flow-domain design:

- `systemGain` may still be used as plant information
- `systemGain` should no longer multiply the final controller output directly

Recommended uses of `systemGain` in this phase:

- derive a soft pressure-based flow ceiling
- influence default gain seeds or segment-level tuning templates
- support diagnostics and future adaptive enhancements

### Limiting

All output limiting in the adaptive loop must happen in `L/min`:

```text
q_raw = q_prev + Δq + q_ff
q_cmd = clamp(q_raw, outputMin, outputMax)
q_cmd = clamp(q_cmd, outputMin, q_cap_if_enabled)
```

Where:

- `outputMin` and `outputMax` are the resolved segment/controller flow bounds
- `q_cap_if_enabled` is an optional pressure-aware soft cap derived from `systemGain`

Negative-flow gating must also be evaluated in `L/min`, not in a hidden motor-speed domain.

## Stability Protection Rules

To reach the required overshoot target with acceptable robustness, the flow-domain controller should include two stabilization rules in this phase:

1. **saturation-aware adaptation restraint**
   - when the command is saturated and the error would drive it deeper into saturation, freeze or weaken integral growth and adaptive gain updates

2. **small-error learning reduction**
   - once pressure error enters a narrow steady-state band, reduce adaptive learning aggressiveness to avoid parameter wandering during pressure hold

These rules are part of the migration design because unit-domain cleanup alone is unlikely to guarantee `< 5%` overshoot under the current adaptive structure.

## Testing and Validation Plan

Validation must cover both correctness of the unit-domain migration and closed-loop behavior.

### Required Test Surfaces

1. `test_rbf_pid`
2. `test_pressure_controller`
3. `test_rbf_pid_hil`
4. new or updated plant-model simulation tests for the flow-domain adaptive path

### Unit and Causality Assertions

Add or update tests so they prove:

- `RBF_PID_Update()` returns flow in `L/min`
- internal adaptive-output state is flow-domain state
- `flowToPumpSpeedGain` is not used inside the adaptive control law for output scaling
- `pump_converter` is the only `flow -> rpm` conversion point
- the RBF input vector uses historical values only
- current pressure is not part of the RBF input vector
- flow normalization and pressure normalization are distinct

### Closed-Loop Performance Tests

Use the existing simulation chain:

```text
pressure_controller -> rbf_pid -> pump_converter -> plant model
```

The plant may remain an RPM-input plant model, but the pump-speed input must come from `pump_converter`, not from bypassing the flow-domain controller output.

#### Single-setpoint tests

At minimum, preserve the existing target family:

- `50 bar`
- `80 bar`
- `100 bar`

Acceptance:

- overshoot `< 5%`
- final steady-state error `<= 2%`
- no sustained oscillation in the tail window

#### Setpoint-switching tests

At minimum, validate:

- `50 -> 80 -> 100`
- `100 -> 50`

Acceptance:

- no unstable spike on target decrease
- no abnormal positive flow surge after a downward step
- bounded gains throughout the sequence

#### Saturation and negative-flow tests

Validate:

- bounded behavior when `outputMax` is active
- no runaway adaptation while saturated
- if negative flow is enabled, zero-crossing and relief behavior remain smooth

## Acceptance Criteria

This design is considered successfully implemented only when all of the following are true:

1. the adaptive pressure-control path operates entirely in `L/min` until `pump_converter`
2. the RBF input vector uses causal history and excludes current pressure feedback
3. flow and pressure normalization are separated
4. the output-end gain-compensation multiplier is removed or replaced by a pure flow-domain policy
5. the updated simulation-backed tests pass
6. measured overshoot in the required validation scenarios is below `5%`

## Implementation Sequence

Recommended execution order:

1. update `RBF_PID_Handle` state semantics and comments
2. change `src/rbf_pid.c` to pure flow-domain output and pure historical RBF input
3. separate flow and pressure normalization
4. remove the internal `flowToPumpSpeedGain`-based output scaling path
5. replace end-of-output gain multiplication with flow-domain limiting logic
6. retune default gains, gain windows, and learning rates
7. update synchronization logic in `src/pressure_controller.c`
8. update and extend tests
9. run simulation-backed tuning until the overshoot criterion is met

## Stop Condition

This design phase ends when the implementation plan can be written against a stable, user-approved spec with:

- no unresolved unit-domain ambiguity
- no unresolved RBF input-timing ambiguity
- a concrete validation target for `< 5%` overshoot
- explicit confirmation that online `K` and Smith compensation remain deferred
