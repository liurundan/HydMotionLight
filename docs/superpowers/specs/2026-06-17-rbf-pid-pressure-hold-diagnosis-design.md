# RBF-PID Pressure Hold Diagnosis Design

Date: 2026-06-17

## Goal

Diagnose why `src/sim/PressureModel.c` plus `src/rbf_pid.c` produces about `+-17 bar` steady-state error around a `100 bar` dead-head pressure-hold target, while the real machine reports about `+-5 bar`.

The design must start from two directions:

1. review the pressure simulation model structure and parameters
2. review the RBF-PID algorithm and its surrounding closed-loop wiring

The diagnosis must preserve the current `10 / 20 / 30 / 40 rpm` open-loop fit as much as possible. Closed-loop improvement is required, but it must not come from casually breaking the validated open-loop behavior.

## User-Confirmed Scope

- Closed-loop target: `100 bar`
- Error metric is based on `measured_pressure_bar`
- Real-machine reference is based on the filtered pressure used by the controller
- Sim-side observation has been checked both on raw `measured_pressure_bar` and on a sliding-mean version; both are still too large
- Operating condition is dead-head pressure hold: target reached, valves effectively closed, no motion
- Controller configuration is intended to match the real machine
- Hard constraint: keep the current open-loop fit quality for `10 / 20 / 30 / 40 rpm`
- Additional requirement: the issue must be validated in test cases, not only by manual trend inspection

## Problem Statement

The current mismatch is likely not caused by one isolated bug.

The codebase currently combines:

- an open-loop-fitted pressure model whose recent tuning target is the `10 / 20 / 30 / 40 rpm` envelope
- a measurement layer that adds tooth-synchronous visible pressure valleys and noise on top of the real pressure state
- an RBF-PID controller that runs with adaptive gains, gain compensation, internal deadband, and pressure-difference-based feedforward terms

Because of that structure, the most likely failure mode is combined mismatch:

- the model may generate too much visible pressure ripple for a `100 bar` hold condition
- the controller may be too sensitive to that ripple and amplify it into output oscillation

## Relevant Code Context

### Pressure model

- `src/sim/PressureModel.c` maintains a real pressure state and then derives `measured_pressure_bar`
- `measured_pressure_bar` is not only the physical pressure state; it includes:
  - tooth-synchronous visible pressure drop shaping
  - optional sensor noise
  - optional sensor bias
- the model also injects motor-speed noise directly into the motor state update
- the current defaults were tuned to match open-loop measured data, not specifically `100 bar` closed-loop hold behavior

### Pressure controller and RBF-PID

- `src/pressure_controller.c` feeds the pressure controller with `AXIS_REF.pressure`
- the RBF branch uses `filteredPressure`, but current defaults often make filtering effectively minimal for the RBF path
- `src/rbf_pid.c` layers several behaviors:
  - RBF network Jacobian estimation
  - adaptive `KP / KI / KD`
  - output gain compensation through `systemGain`
  - internal deadband
  - pressure-difference-based feedforward terms

### Unit caution

The repository contains both `bar` and `MPa` language in different areas. The diagnosis must treat unit consistency as an explicit check item, especially for:

- `targetPressure`
- `pressureCeiling`
- `systemGain`
- controller normalization scale
- pressure model outputs

The design does not assume a unit bug exists, but it must be ruled out early because it would distort all later conclusions.

## Constraints

- Do not start by rewriting the model or replacing the controller
- Do not sacrifice current open-loop fit quality just to make one closed-loop case look better
- Do not use manual visual inspection as the only validation path
- Keep the diagnosis narrow and evidence-driven
- Preserve existing APIs unless evidence shows the API surface itself is part of the problem

## Recommended Approaches

### Approach A: Model-first diagnosis with minimal controller disturbance

Start by isolating how much of the `+-17 bar` comes from:

- real pressure dynamics
- visible measurement shaping
- motor/sensor noise

Then make only the controller changes that remain necessary.

Pros:

- best fit for the user's open-loop preservation constraint
- easier to prove whether the model is overstating visible ripple
- lower risk of masking plant-model problems with controller tuning

Cons:

- may take longer to reach the final closed-loop number

### Approach B: Controller-first desensitization

Keep the model mostly unchanged and reduce the controller's sensitivity to high-frequency pressure ripple by tuning:

- input filtering
- gain compensation
- feedforward terms
- adaptive gain windows

Pros:

- fastest path to smaller hold error

Cons:

- can hide model-side overstatement
- may not generalize to later scenarios

### Approach C: Full joint retuning

Retune both the model and the controller together against a new `100 bar` hold benchmark.

Pros:

- strongest eventual match if done well

Cons:

- highest scope and regression risk
- most likely to damage the validated open-loop fit

## Recommendation

Use **Approach A as the main lane**, use **Approach B only as an isolation tool**, and use a small **Approach C** pass only at the end if needed.

Recommended sequence:

1. add closed-loop diagnosis coverage without removing current open-loop coverage
2. isolate model-side contribution first
3. isolate controller-side amplification second
4. only then tune the smallest set of parameters or terms needed

This gives the cleanest root-cause signal while respecting the open-loop constraint.

## Diagnosis Architecture

The diagnosis must separate four signal layers in the same `100 bar` dead-head hold scenario:

1. `real_pressure_bar`
2. `measured_pressure_bar`
3. controller `filteredPressure`
4. controller output command, at least `outputFlow`, and preferably the downstream pump-speed equivalent

These four signals are enough to classify the fault source:

- if `real_pressure_bar` is already large and unstable, the pressure state dynamics are too active
- if `real_pressure_bar` is reasonable but `measured_pressure_bar` is too large, the measurement layer is overstated
- if `filteredPressure` remains too large relative to `measured_pressure_bar`, the controller-side filtering is insufficient
- if pressure ripple is moderate but output command oscillation is excessive, the controller is amplifying the ripple

## Diagnosis Matrix

### Model-side isolation checks

Run the same closed-loop hold scenario while selectively disabling or reducing individual model features.

Priority checks:

1. disable `tooth_drop` visibility effect
2. disable motor noise
3. disable sensor noise
4. compare `real_pressure_bar` against `measured_pressure_bar`

Interpretation:

- if disabling `tooth_drop` removes most of the error, the visible valley model is too strong for hold conditions
- if disabling motor noise removes most of the error, current motor-noise injection is too aggressive for a `1 ms` simulation loop
- if `real_pressure_bar` stays tight while `measured_pressure_bar` swings wide, the main pressure state is acceptable and the measurement layer is the problem

### Controller-side isolation checks

Keep the model fixed and selectively reduce controller sensitivity.

Priority checks:

1. disable or neutralize `systemGain` compensation
2. disable or reduce the pressure second-difference feedforward term
3. increase effective measurement filtering for the RBF path
4. shrink adaptive `KP / KI / KD` windows

Interpretation:

- if removing gain compensation helps, `systemGain` does not match the tuned plant
- if removing the second-difference pressure feedforward helps, the controller is reacting to tooth ripple and noise as if they were useful dynamics
- if stronger filtering helps, the controller is tracking tooth valleys instead of average hold pressure
- if tighter gain windows help, adaptation freedom is too large for this plant-plus-measurement combination

## Most Suspicious Model Parameters

The first model parameters to inspect are:

1. `motor_noise_std_rpm`
2. `tooth_drop_depth_base`
3. `tooth_drop_width_ratio`
4. `flow_ripple_ratio`

Why these first:

- they directly affect high-frequency visible pressure behavior
- they can strongly change hold ripple without rewriting the main pressure-state equation
- they are the most likely cause of a model that still passes open-loop envelope tests but overstates closed-loop hold fluctuation

Secondary model parameters to inspect only if needed:

1. `veff_base_m3`
2. `leak_base_m3_pa_s`
3. speed-dependent scale arrays for volume and leakage

These affect the pressure-state gain and damping, so they have higher risk of damaging the validated open-loop fit.

## Most Suspicious Controller Parameters and Terms

The first controller items to inspect are:

1. effective measurement filtering in the RBF path
2. `systemGain`
3. pressure-difference-based feedforward term
4. adaptive gain windows

Why these first:

- they can amplify ripple without any bug in the plant state equation
- they can usually be isolated without changing the controller architecture
- they are directly involved in how closed-loop pressure-hold reacts to visible tooth valleys

Secondary controller items:

1. internal deadband size
2. normalization scale policy
3. reset and reseeding behavior when targets drop or segments switch

## Test Strategy

The issue must be validated by test cases.

### Existing tests to preserve

Current `10 / 20 / 30 / 40 rpm` open-loop pressure-model tests remain the regression baseline. They must keep passing unless a design-approved tolerance update is explicitly justified.

### New tests to add

Add dedicated closed-loop diagnosis tests that make the current failure measurable and repeatable.

Minimum new test surface:

1. **`100 bar` hold baseline test**
   - dead-head hold
   - fixed `1 ms` step
   - fixed simulation duration long enough to include a settle window and a steady-state window
   - records:
     - `real_pressure_bar`
     - `measured_pressure_bar`
     - `filteredPressure`
     - `outputFlow`
   - computes steady-state metrics over the hold window

2. **Model-side ablation tests**
   - same hold case with:
     - tooth-drop disabled
     - motor noise disabled
     - sensor noise disabled
   - these are diagnosis tests, not final acceptance tests
   - they must prove which model feature contributes how much

3. **Controller-side ablation tests**
   - same hold case with:
     - gain compensation disabled
     - high-frequency feedforward disabled or reduced
     - stronger filtering
     - narrower adaptive gain windows
   - these must prove whether the controller is amplifying the measured ripple

### Metrics to compute in tests

Each hold test should compute at least:

1. steady-state peak-to-peak range of `real_pressure_bar`
2. steady-state peak-to-peak range of `measured_pressure_bar`
3. steady-state peak-to-peak range of `filteredPressure`
4. steady-state mean absolute error versus the `100 bar` target
5. steady-state output oscillation range

These metrics are more useful than a single instantaneous error assertion because the issue is specifically about steady-state hold fluctuation.

## Validation Stages

### Stage 1: Root-cause classification

Success means the test suite can show which bucket dominates:

1. main pressure-state dynamics
2. model measurement layer
3. controller high-frequency amplification
4. combined model-controller mismatch

This stage is about explanation quality, not yet final tuning.

### Stage 2: Optimization validation

Success means:

1. current open-loop tests still pass
2. the new `100 bar` hold tests show clear reduction from the current sim-side behavior
3. `filteredPressure` steady-state fluctuation moves materially closer to the real-machine `+-5 bar` reference
4. controller output does not show sustained large-amplitude hunting in the hold window

The design intentionally does not freeze one final numeric threshold yet, because the first task is to classify where the excess ripple comes from. Once the baseline and ablation tests are added, the exact acceptance budget can be set from measured evidence instead of guesswork.

## Implementation Guidance

When this design moves into implementation, use the smallest change set that can prove or remove each hypothesis.

Recommended execution order:

1. add the new closed-loop diagnosis tests first
2. make the tests expose the current failure
3. add ablation hooks or test-only parameter overrides
4. identify the dominant contribution
5. tune model-side parameters first if the main issue is in measurement shaping or noise
6. tune controller-side sensitivity second if amplification remains
7. rerun all open-loop and closed-loop regressions after every meaningful adjustment

## Non-Goals

- redesigning the entire pressure model
- replacing RBF-PID with another control law
- making one closed-loop case look good by overfitting and damaging open-loop behavior
- using only manual plots as proof

## Stop Condition

The diagnosis phase is complete when all of the following are true:

1. the dominant source of the excessive `100 bar` hold error is identified with test-backed evidence
2. the repo contains repeatable tests that expose the current issue and future regressions
3. the next step can be expressed as a bounded implementation plan rather than another round of open-ended discussion
