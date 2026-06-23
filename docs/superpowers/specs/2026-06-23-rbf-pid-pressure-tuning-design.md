# RBF-PID Pressure Tuning Design

Date: 2026-06-23

## Goal

Review the existing RBF neural-network PID tuning logic in:

- `src/rbf_pid.c`
- `src/sim/PressureModel.c`

and tune the closed-loop pressure-control parameters so the simulation satisfies injection-machine pressure-control expectations for a pure first-order pressure object.

The required outcome is:

- scenario-aware algorithm review, tied to injection-machine pressure control rather than generic adaptive-control commentary
- tuned RBF-PID parameter recommendations with direct implementation mapping
- simulation validation against three pressure steps: `50 / 100 / 200 bar`
- each target must satisfy:
  - overshoot `<= 5%`
  - steady-state pressure error `<= 1%`

## User-Confirmed Scope

- Main verification object: pure first-order pressure model only
- Pressure-model boundary: keep the repository's existing static `flow -> rpm` conversion, but do not include the motor first-order dynamic link; the controlled object for tuning is only the first-order pressure branch
- Pressure-model parameters:
  - gain `K = 5.4`
  - time constant `tau = 1.0 s`
  - dead time `L = 0`
- Target points: `50 bar`, `100 bar`, `200 bar`
- Final output format must be split into:
  1. `算法评审结论`
  2. `优化后参数`
  3. `仿真验证说明`

## Existing Code Context

### RBF-PID controller

The current adaptive controller already exists in `src/rbf_pid.c` and is wired through `src/pressure_controller.c`.

Relevant characteristics:

- 4-dimensional RBF input:
  - previous incremental output `du_prev`
  - previous measured pressure `y_prev1`
  - second previous measured pressure `y_prev2`
  - previous error `e_prev1`
- 6 hidden nodes
- online Jacobian estimation through the RBF network
- online adaptation of:
  - RBF weights / centers / widths
  - PID gains `KP / KI / KD`
- incremental PID output form
- additional pressure-acceleration feedforward term
- bounded gain windows and bounded learning rates exposed through `pressureRbfConfig`

### Pressure model

`src/sim/PressureModel.c` already contains a first-order branch.

The current first-order discrete update is:

```text
P(k+1) = (K * rpm(k) * Ts + tau * P(k)) / (tau + Ts)
```

with internal clamping to the physical pressure range.

### Existing tests

The repository already contains a near-target validation surface in
`tests/test_pressure_controller.c`.

That test harness already:

- uses a pure first-order plant
- runs RBF-PID in closed loop
- verifies bounded overshoot
- verifies final convergence

The main gap is that the existing acceptance shape is not yet aligned with this task:

- current targets are not `50 / 100 / 200 bar`
- current steady-state acceptance is looser than `<= 1%`
- current output does not explicitly explain the tuning mechanism in injection-machine terms

## Problem Statement

The task is not to invent a new controller. It is to judge whether the current RBF-PID structure is correct and suitable for injection-machine pressure control, then tighten the tuning and validation envelope so it behaves like a practical pressure-building and pressure-holding controller.

For this plant, the limiting factor is not actuator authority.

Given:

- motor max speed `2000 RPM`
- pump displacement `20 cc/rev`
- flow command range `0-90 L/min`

and the current flow-to-speed mapping used by the repository, even the `200 bar` target requires only a small steady-state flow command relative to the available output ceiling. That means excessive overshoot is more likely to come from:

- aggressive incremental `KP / KI / KD`
- overly fast online adaptation
- Jacobian fluctuation
- feedforward overreaction
- adaptation continuing too actively near saturation or near target

This is directly relevant to injection pressure control, where the practical requirement is:

- build pressure quickly enough to avoid sluggish cycle response
- avoid pressure strike and overshoot at high pressure
- hold pressure steadily under small disturbances and measurement ripple

## Constraints

- Stay inside the current RBF-PID architecture; do not replace it with a different controller
- Keep the work narrowly tied to:
  - `src/rbf_pid.c`
  - `src/sim/PressureModel.c`
  - directly related test coverage
- Tune against injection-machine pressure-control needs, not generic adaptive-control theory
- The tuning explanation must state why each parameter change affects:
  - overshoot
  - steady-state error
  - robustness
- Validation must be testable and repeatable in-repo

## Approaches Considered

### Approach A: Keep the current structure and only tighten parameter windows

Adjust gain bounds and learning rates without changing the validation method.

Pros:

- smallest code change
- lowest delivery risk
- directly reuses current test harness

Cons:

- still leaves the controller acting as both plant estimator and aggressive online tuner
- weaker explanation for why the tuned result is reliable at high pressure

### Approach B: Baseline PID stability first, then allow RBF to make bounded micro-adjustments

Treat the existing controller as a two-layer system:

1. a stable bounded PID envelope matched to the first-order pressure object
2. a slower RBF adaptation layer allowed only to fine-tune within that envelope

Pros:

- best fit for injection-machine pressure control
- more defensible high-pressure behavior
- easier to explain why overshoot drops and steady-state error tightens
- preserves the value of RBF adaptation without letting it dominate the loop

Cons:

- requires more deliberate validation of the adaptive layer

### Approach C: Reduce or disable adaptation and fall back toward fixed PID

Pros:

- easiest path to passing the numeric target

Cons:

- weak answer to the actual task
- does not properly review or optimize the RBF-PID method itself

## Recommendation

Use **Approach B**.

That means:

- keep the existing RBF-PID architecture
- establish a conservative PID operating window matched to the first-order plant
- reduce learning aggressiveness so adaptation is slower than plant dynamics
- let the RBF layer refine the loop inside bounded limits instead of driving large control swings

This recommendation best matches injection-machine pressure control, where practical control quality is defined less by theoretical adaptability and more by:

- no pressure strike during build-up
- no visible pressure pumping during hold
- reliable behavior from low pressure to high pressure

## Design

### 1. Review focus

The algorithm review will explicitly evaluate the following aspects of the current implementation.

#### 1.1 Input and normalization suitability

Review whether the current RBF inputs and normalization strategy are suitable for a pressure loop with:

- pressure range `0-250 bar`
- flow output range `0-90 L/min`

The review must check whether the controller is using a physically sensible scale for:

- pressure error
- incremental output
- recent pressure trend

Expected finding direction:

- the structure is broadly correct for online local Jacobian estimation
- but stable injection pressure control depends on tighter scaling discipline than the current defaults alone guarantee

#### 1.2 Adaptive gain update suitability

Review whether the online updates of `KP / KI / KD` are too aggressive for a `tau = 1.0 s` first-order object.

Expected finding direction:

- the adaptive form is acceptable
- but the current tuning freedom is likely too large relative to the plant time constant, especially at higher pressure targets

#### 1.3 Feedforward suitability

Review whether the pressure-acceleration feedforward term helps or harms this object.

Expected finding direction:

- for a pure first-order plant with no dead time, pressure second-difference feedforward is not the main source of control authority
- it may still help transient shaping, but it must remain subordinate to the bounded PID loop

#### 1.4 Injection-machine applicability

The review must judge suitability against actual injection pressure-control needs:

- pressure rise should be prompt
- high-pressure overshoot must stay low
- steady-state pressure error must stay small
- adaptation must not create extra jitter near hold

The review should therefore not reward adaptation speed by itself. It should reward controlled, repeatable pressure response.

### 2. Tuning strategy

The tuning strategy will follow a bounded two-layer rule.

#### 2.1 Tighten the base gain window

`KP / KI / KD` windows should be narrowed so the controller cannot jump into high-gain combinations that are unnecessary for this plant.

Reason:

- the `200 bar` point is still far from output saturation in steady state
- excessive overshoot is therefore mainly a tuning issue, not an actuator-limit issue

Expected effect:

- lower transient strike at `200 bar`
- lower gain drift near the target
- easier convergence across all three targets

#### 2.2 Slow the adaptation layer

Reduce:

- `etaP`
- `etaI`
- `etaD`
- and, if needed, `etaW / etaC / etaB`

Reason:

- the plant pressure evolves on the order of `1 s`
- adaptation should not move PID gains on a timescale comparable to or faster than the pressure transient itself

Expected effect:

- smaller gain chasing during rise
- lower Jacobian-induced control jitter
- smaller final steady-state ripple

#### 2.3 Keep RBF adaptation bounded, not disabled

Do not disable the RBF layer by default.

Reason:

- the task requires reviewing and optimizing the RBF-PID method
- bounded adaptation is the correct compromise between practical stability and algorithm intent

Expected effect:

- preserve adaptive capability
- avoid turning the controller into a nominally adaptive but effectively unstable tuner

#### 2.4 Re-evaluate acceleration feedforward conservatively

Keep the pressure-acceleration feedforward only if it helps all three targets without increasing high-pressure overshoot.

Reason:

- on a dead-time-free first-order plant, aggressive second-difference pressure feedforward can amplify transient correction rather than improve it

Expected effect:

- if retained with conservative tuning, it may improve rise shaping
- if it raises overshoot at `200 bar`, it should be reduced or disabled

### 3. Parameterization target

The implementation work should produce a recommended tuned profile covering:

- bounded `KP / KI / KD` window
- bounded `etaW / etaC / etaB`
- bounded `etaP / etaI / etaD`
- explicit recommendation on whether pressure-acceleration feedforward remains enabled
- any normalization-related adjustments necessary for the `0-250 bar` and `0-90 L/min` operating ranges

The final tuned profile should be suitable for three representative injection-machine pressure levels:

- `50 bar`: low-pressure response must still converge cleanly, without adaptation becoming too weak to remove residual error
- `100 bar`: nominal working point
- `200 bar`: high-pressure point where overshoot control is most safety-relevant

## Verification Design

Verification will use the pure first-order closed-loop harness as the main proof surface.

### Step scenarios

Run three step tests:

- `0 -> 50 bar`
- `0 -> 100 bar`
- `0 -> 200 bar`

### Metrics

Collect, per target:

- peak pressure
- overshoot ratio
- final `1 s` window average absolute error
- final `1 s` window maximum absolute error
- time to enter and remain within the `+-1%` band
- final adaptive gains
- optional Jacobian and output-flow variation summary if needed to explain tuning behavior

### Pass criteria

For each of `50 / 100 / 200 bar`:

- overshoot `<= 5%`
- final `1 s` average absolute error `<= 1% * target`
- final `1 s` maximum absolute error `<= 1% * target`

The maximum-absolute-error gate is required so the result cannot pass on averaging alone while still oscillating visibly near the target.

### Mechanism validation

The final explanation must include at least one before/after comparison showing why the tuning worked.

Recommended evidence:

- before/after peak overshoot at `200 bar`
- before/after final error at `50 bar` and `100 bar`
- before/after final `KP / KI / KD` behavior or output-flow variation

The point is not just to report that the numbers improved. It is to show which tuning direction reduced overshoot and which tuning direction tightened steady-state error.

## Deliverable Shape

The final user-facing output must be organized into exactly these three sections.

### 算法评审结论

This section should state:

- whether the current RBF-PID structure is basically correct
- which parts are suitable for injection-machine pressure control
- which parts need tightening or restraint
- the direct engineering reason for each recommendation

### 优化后参数

This section should provide:

- tuned or recommended values for:
  - `KP / KI / KD`
  - `etaW / etaC / etaB`
  - `etaP / etaI / etaD`
  - pressure-acceleration feedforward enable/disable decision
- where in the current code or segment config these values map

### 仿真验证说明

This section should provide:

- the three step-test results
- whether each target met:
  - overshoot `<= 5%`
  - steady-state error `<= 1%`
- a concise mechanism explanation tying parameter changes to:
  - overshoot behavior
  - steady-state error behavior
  - robustness in the injection pressure-control scenario

## Implementation Guidance

The implementation phase should stay narrow:

- prefer extending the existing first-order plant test in `tests/test_pressure_controller.c`
- only touch `src/rbf_pid.c` and `src/sim/PressureModel.c` where the tuning evidence points
- avoid unrelated refactoring

The implementation is complete only when:

- all three targets pass the numeric criteria
- the test results are reproducible
- the final explanation is grounded in the observed behavior of the tuned loop
