# Pressure Model First-Order Switch Design

Date: 2026-06-22

## Goal

Extend `src/sim/PressureModel.c` and the PLC-facing `HYD_PRESSUREMODEL` FB so the pressure simulator can switch online between:

- the current physical pressure model
- a new first-order inertial pressure model with optional pure delay

The design shall:

- preserve the current physical model as the default behavior
- expose model selection and first-order parameters all the way to `HYD_PRESSUREMODEL`
- allow online switching without forcing pressure to drop to zero
- preserve current pressure value across model switches
- make the first-order model use `MOTOR_RPM` as input, interpreted through `K_NUM` in `bar/rpm`
- keep first-order output bounded to `0 .. 250 bar`
- keep the existing reset behavior when `ENABLE = false`
- add focused regression tests so the default physical-model path does not regress

The primary deliverable is a design for a shared `PressureModel` API that supports two pressure-generation branches behind one stateful interface. No implementation is included in this task.

## Scope and Non-Goals

### In scope

- `include/pressure_model.h`
- `src/sim/PressureModel.c`
- `include/hydro_sim_fb.h`
- `src/sim/hydro_sim_fb.c`
- `tests/test_pressure_model.c`
- `tests/test_hydro_sim_fb.c`

### Out of scope

- changing the existing physical pressure equations unless required to wrap them behind model selection
- exposing the new model through unrelated simulator APIs
- adding visual, HMI, or recipe-layer configuration outside `HYD_PRESSUREMODEL`
- matching a measured waveform beyond the agreed first-order formula and delay semantics
- introducing new dependencies or calibration tooling

## Current Context

The current pressure model already exposes a stateful C API through `PressureModel_InitParams`, `PressureModel_Reset`, and `PressureModel_Step`. The model is integrated into the PLC adapter through `__mcl_cmd_updatePressureModel(...)`, which currently accepts only:

- `ENABLE`
- `MOTOR_RPM`
- `TIME_S`

The current implementation has three important constraints for this task:

1. the physical model is already the repository default and is covered by unit and FB-level regression tests
2. `HYD_PRESSUREMODEL` does not yet expose any model-selection or first-order tuning inputs
3. switching models at runtime must preserve current pressure rather than resetting the simulation

This means the safest design is to preserve the current physical path intact by default and add the first-order path as an explicitly selected branch.

## Recommended Design

Use one shared `PressureModel` API and extend its params/state to support both the current physical model and a first-order inertial model with pure delay.

This is preferred over a separate first-order API because:

- the repository already treats `PressureModel_Step(...)` as the single pressure-plant boundary
- C-level and FB-level tests can validate one shared state machine instead of two loosely coupled ones
- online switching semantics are easier to keep consistent when both branches share one state carrier
- default physical-model behavior remains the unmodified fallback for invalid or omitted model selection

## Architecture

The architecture remains one public API with two internal pressure-generation branches:

```text
HYD_PRESSUREMODEL inputs
  -> PressureModelParams update
  -> PressureModel_Step(...)
     -> shared motor/time preprocessing
     -> model switch handling
     -> physical branch OR first-order branch
     -> shared output packaging
```

### Public interface changes

`PressureModelParams` gains four public fields:

- `model_type`
- `first_order_k_bar_per_rpm`
- `first_order_tau_s`
- `first_order_delay_s`

`PressureModelState` keeps the existing physical-model state and adds first-order branch state:

- `active_model_type`
- `first_order_prev_pressure_bar`
- `first_order_buffer_index`
- `first_order_delay_buffer[1000]`

`HYD_PRESSUREMODEL` gains matching FB inputs:

- `MODEL_TYPE`
- `K_NUM`
- `TTAU`
- `DELAYTIME`

### Default behavior rule

The physical model remains the default in every entry path:

- default `PressureModel_InitParams(...)` values choose `physical`
- `pressure_update(...)` continues to use `physical`
- invalid `MODEL_TYPE` values fall back to `physical`

So existing callers keep the current plant unless they explicitly opt into the first-order branch.

## Model Types and Routing

The design assumes two explicit model types:

- `physical`
- `first_order`

The exact constant names may follow local enum style, but the routing rule is fixed:

- `physical` runs the current pressure plant
- `first_order` runs the new delayed first-order pressure model
- any unknown value is treated as `physical`

This keeps compatibility conservative and makes runtime failure modes obvious.

## State Model

The state is split conceptually into shared state and branch-specific state.

### Shared state

Shared state continues to exist regardless of active model:

- actual motor speed
- pressure in `Pa`
- pump phase in revolutions
- deterministic random state

This allows the simulator to preserve scan-to-scan continuity and makes switching back into the physical model straightforward.

### First-order branch state

The first-order branch adds:

- previous undelayed pressure output in `bar`
- circular delay-buffer write index
- fixed-size delay history buffer for up to `1.0 s` at `1 ms`
- last active model type

The fixed-size buffer avoids dynamic allocation and keeps the implementation consistent with the rest of the C99 codebase.

## Online Switching Semantics

The switch contract is:

- online switching is allowed while `ENABLE = true`
- current pressure is preserved across switches
- the newly activated branch rebuilds only the state it strictly needs
- `ENABLE = false` still resets everything

### Physical to first-order

When switching from `physical` to `first_order`:

- take the current pressure from the shared plant state
- convert it from `Pa` to `bar`
- write that value into `first_order_prev_pressure_bar`
- fill the full delay buffer with that same pressure
- keep motor state and phase state continuous

This prevents the delay line from emitting zero immediately after a switch.

### First-order to physical

When switching from `first_order` to `physical`:

- take the current first-order pressure output in `bar`
- convert it into `Pa`
- write it back into the shared pressure state
- keep shared motor state and phase state continuous

This lets the physical model resume from the currently visible pressure rather than restarting from zero.

## Per-Step Data Flow

Each call to `PressureModel_Step(...)` follows three layers:

1. shared preprocessing
2. branch execution
3. shared output packaging

### Shared preprocessing

The common preprocessing rules are:

- if `dt_s <= 0`, use the existing fallback `1 ms`
- clamp commanded motor speed with the existing min/max limits
- update `state->motor_rpm` using the existing motor first-order lag
- update `state->pump_phase_rev` continuously

The first-order pressure branch therefore uses filtered actual motor speed, not raw commanded speed.

### Branch dispatch

After preprocessing:

- compare `params->model_type` with `state->active_model_type`
- if different, apply the switch-state synchronization rules
- dispatch to either the physical or first-order branch

### Shared output packaging

Both branches write the same `PressureModelOutput` struct so callers keep one stable integration path.

## First-Order Branch Mathematics

The first-order branch uses the agreed formula with optional pure delay.

### Input interpretation

- input signal: actual motor speed in `rpm`
- static gain: `K_NUM` in `bar/rpm`
- time constant: `TAU_S` in `s`
- pure delay: `DELAY_S` in `s`

### Parameter limiting

To keep runtime behavior stable:

- `K_NUM` is clamped to `>= 0`
- `TAU_S` is clamped to `>= 0`
- `DELAY_S` is clamped to `[0, 1.0]`

Delay depth is computed as:

```text
delay_steps = floor(DELAY_S / dt_s)
delay_steps = clamp(delay_steps, 0, 999)
```

### Undelayed first-order response

If `TAU_S > 0`:

```text
y(k) = [K * u * dt + tau * y(k-1)] / [tau + dt]
```

If `TAU_S == 0`:

```text
y(k) = K * u
```

The undelayed output is then limited to `0 .. 250 bar`.

### Delayed output

After calculating the undelayed output:

- write it into the circular delay buffer
- if `delay_steps == 0`, output the current value
- otherwise read from the delayed index

The delayed output becomes the first-order branch pressure result.

## Output Semantics

### First-order branch

For `first_order` mode:

- `REAL_PRESSURE_BAR = delayed_output_bar`
- `MEASURED_PRESSURE_BAR = delayed_output_bar`
- `ACTUAL_MOTOR_RPM = state->motor_rpm`
- `pump_flow_m3_s = 0`
- `net_flow_m3_s = 0`
- `relief_active = 1` only when the first-order result is capped by the `250 bar` limit
- `estimated_torque_trend` continues to use the existing speed-and-pressure-derived formula

The first-order branch does not simulate a separate measurement chain. The user-approved contract is that both pressure outputs are identical in this mode.

### Physical branch

For `physical` mode:

- keep the current physical-model output semantics unchanged
- keep the current noise, ripple, relief, and visible-pressure behavior unchanged

This protects existing simulator expectations and regression tests.

## PLC FB Integration

`HYD_PRESSUREMODEL` becomes the runtime control surface for the new branch.

### New FB inputs

The FB gains four new online-settable inputs:

- `MODEL_TYPE`
- `K_NUM`
- `TTAU`
- `DELAYTIME`

Each scan:

- the adapter copies these into `g_pressure_model_params`
- `TIME_S` still determines `dt_s`
- the adapter calls `PressureModel_Step(...)`

### FB reset semantics

When `ENABLE = false`:

- reset model state
- clear outputs
- clear scan-time history

This remains unchanged from the current FB behavior.

## Compatibility Rules

The design intentionally minimizes behavior changes:

- existing callers that do not set `MODEL_TYPE` continue to get the physical model
- `pressure_update(...)` remains a compatibility wrapper over the default physical branch
- first-order fields exist but are inert until `MODEL_TYPE = first_order`
- invalid model values do not produce a third runtime path; they fall back to `physical`

## Testing Strategy

Testing is split into C API tests and PLC FB tests.

### `tests/test_pressure_model.c`

Add or extend tests for:

- default physical-model behavior still matching current expectations
- first-order zero-input output staying at zero
- first-order static gain reaching the expected steady state
- first-order `TAU_S` affecting transient response
- first-order `DELAY_S` delaying visible output by the expected number of scans
- `TAU_S = 0` degenerating to direct gain
- online `physical -> first_order` switch preserving pressure continuity
- online `first_order -> physical` switch preserving pressure continuity
- invalid model type falling back to physical behavior

### `tests/test_hydro_sim_fb.c`

Add or extend tests for:

- new FB inputs defaulting to behavior compatible with the current model
- online `MODEL_TYPE` change keeping `ACTIVE` asserted while pressure stays continuous
- online `K_NUM`, `TTAU`, and `DELAYTIME` updates affecting subsequent scans
- `ENABLE = false` still clearing outputs and state

## Acceptance Criteria

The task is complete when all of the following are true:

- default physical-model tests still pass without requiring callers to opt in
- first-order branch tests pass at the C API layer
- `HYD_PRESSUREMODEL` exposes online switching and first-order parameter inputs
- pressure does not jump to zero when switching models online
- `REAL_PRESSURE_BAR == MEASURED_PRESSURE_BAR` in first-order mode
- current reset semantics on `ENABLE = false` remain intact

## Risks and Tradeoffs

### Fixed delay buffer size

Using a fixed `1000`-sample buffer is simple and allocation-free, but ties the design to the agreed `1.0 s` maximum delay target. That is acceptable for this task because the user explicitly chose that bound.

### Shared motor state across branches

Keeping motor state continuous across both models improves switch behavior and keeps `ACTUAL_MOTOR_RPM` meaningful, but it means the first-order branch is not a fully isolated black-box transfer function. This tradeoff is intentional because runtime continuity matters more than strict branch independence here.

### Physical path preservation

The design favors wrapping the current physical branch rather than reshaping it. This keeps regression risk down, but requires discipline so the new routing logic does not accidentally alter the default path.

## Implementation Notes for Planning

The eventual implementation plan should keep the work split into small verifiable steps:

1. extend public structs and FB inputs
2. add first-order state scaffolding with no behavior change to physical default path
3. add branch dispatch and switch-state synchronization
4. add focused unit tests for first-order response and switching
5. add FB regression tests for online control and reset semantics

The implementation should prefer targeted tests before broad full-suite runs, then finish with the normal pressure-model and FB regression commands used in this repository.
