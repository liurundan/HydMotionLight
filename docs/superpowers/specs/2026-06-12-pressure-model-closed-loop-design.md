# Pressure Model Closed-Loop Simulation Design

Date: 2026-06-12

## Goal

Upgrade `src/sim/PressureModel.c` into a testable hydraulic pressure plant model for injection-molding pressure-loop simulation.

The model shall:

- accept target motor speed in the range `-100 .. 2000 rpm`
- output hydraulic pressure in the range `0 .. 250 bar`
- simulate the dead-head condition only (all servo valves closed)
- include the 13-tooth gear pump signature, especially the once-per-tooth pressure drop behavior
- include representative noise sources
- support small negative speed for fast depressurization

The primary deliverable is a standalone pressure model plus focused tests. A secondary deliverable is the minimum PLC FB state cleanup required so the model can run scan-to-scan without resetting its internal dynamics.

## Scope and Non-Goals

### In scope

- `src/sim/PressureModel.c`
- one dedicated model header for explicit params/state/output types
- the pressure-model call site in `src/sim/hydro_sim_fb.c`
- dedicated pressure-model tests
- minimal PLC FB state persistence fixes

### Out of scope

- valve-open simulation
- extending `src/sim/hydro_sim.c` multi-axis motion physics
- adding empirical multi-point pressure-speed fitting beyond the given `10 rpm -> 40 bar` calibration point
- changing the general `HydraulicSimEnv` pump definition, which currently uses a different displacement

## Current Context

`PressureModel.c` already contains the beginnings of a physical model: motor lag, leak flow, relief flow, tooth ripple, pressure drop shaping, and sensor noise.

The current implementation is not yet a reliable plant model because:

- motor dynamic state is recreated inside each call instead of being preserved across calls
- pressure state is partly hidden in FB-static storage
- the model has no direct unit tests
- the physics and measurement layers are not separated cleanly enough for deterministic validation

The repo also has a pressure-control test surface already, but it currently validates controller behavior more than this new plant model.

## Recommended Design

Use a minimal physical dead-head pressure model, calibrated only by the known open-loop point `10 rpm -> 40 bar`, with explicit state and deterministic test mode.

This is preferred over a lookup or piecewise empirical curve because only one calibration point is available. The single measured point should be used to identify a physical parameter, not to justify a synthetic pressure-speed map.

## Architecture

Introduce an explicit stateful model API:

```c
typedef struct PressureModelParams PressureModelParams;
typedef struct PressureModelState PressureModelState;
typedef struct PressureModelOutput PressureModelOutput;

void PressureModel_InitParams(PressureModelParams* params);
void PressureModel_Reset(PressureModelState* state, unsigned int seed);
void PressureModel_Step(const PressureModelParams* params,
                        PressureModelState* state,
                        float target_rpm,
                        float dt_s,
                        PressureModelOutput* out);
```

### Responsibilities

- `PressureModelParams`
  - constant machine and model parameters
  - noise amplitudes and toggles
  - limits and calibration constants

- `PressureModelState`
  - actual motor speed
  - real chamber pressure
  - pump rotational phase
  - deterministic random-generator state

- `PressureModelOutput`
  - measured pressure in bar
  - real pressure in bar
  - actual motor speed in rpm
  - optional diagnostic values such as pump flow and relief-active flag

### Integration rule

Keep `src/sim/hydro_sim.c` unchanged. The pressure model remains an isolated plant-model utility used through the PLC FB adapter path in `src/sim/hydro_sim_fb.c`.

This keeps the diff narrow and avoids mixing this task's required `20 cc/rev` pressure model with the current general simulator defaults.

## Mathematical Model

The model is split into a physical plant layer and a measurement layer.

### 1. Motor speed dynamics

Target speed is limited first:

```text
n_ref = sat[-100, 2000](target_rpm)
```

Actual motor speed follows a first-order lag plus speed disturbance:

```text
alpha = dt / (tau_m + dt)
n_act(k+1) = sat[-100, 2000](
    n_act(k) + alpha * (n_ref - n_act(k)) + w_n(k)
)
```

Where:

- `n_act` is in `rpm`
- `tau_m` is motor time constant
- `w_n` is motor-speed noise, modeled as zero-mean Gaussian noise

### 2. Pump flow

Pump displacement is fixed by the task:

```text
D = 20 cc/rev = 20e-6 m^3/rev
```

Base pump flow:

```text
Q_base = D * n_act / 60
```

Positive rotation includes 13-tooth flow ripple:

```text
theta_rev(k+1) = frac(theta_rev(k) + n_act * dt / 60)
Q_pump = Q_base * [1 + A_q * sin(2*pi*13*theta_rev)]
```

Rules:

- positive speed: apply ripple
- negative speed: allow negative flow for depressurization
- near zero speed: flow approaches zero smoothly

### 3. Dead-head pressure build-up

All servo valves are closed. The chamber is modeled as a fixed effective volume with compression, leakage, and relief:

```text
Q_leak   = C_leak * P
Q_relief = max(0, C_relief * (P - P_relief))
dP/dt    = (beta_e / V) * (Q_pump - Q_leak - Q_relief + w_q)
P(k+1)   = max(0, P(k) + dt * dP/dt)
```

Where:

- `P` is real chamber pressure in `Pa`
- `beta_e` is effective bulk modulus
- `V` is effective trapped volume
- `w_q` is optional low-frequency process disturbance

### 4. Single-point calibration

Use the given open-loop point `10 rpm -> 40 bar` as the only steady-state calibration point.

At steady state below relief pressure:

```text
Q_pump = Q_leak
D * (10 / 60) = C_leak * 40e5
```

So:

```text
C_leak = D * (10 / 60) / (40e5)
```

This identifies the leakage coefficient from the given machine data without inventing additional empirical curve points.

### 5. Thirteen-tooth pressure signature

Represent the tooth signature with two separate effects:

1. process-side flow ripple at `13x` shaft frequency
2. measurement-side local pressure drop window once per tooth

Measurement-side drop shaping:

```text
tooth_phase = frac(13 * theta_rev)

if tooth_phase < w_d:
    drop_window = 0.5 * [1 + cos(2*pi*tooth_phase / w_d)]
    drop_gain   = 1 - A_d * drop_window
    P_vis       = P * drop_gain
else:
    P_vis       = P
```

This preserves the idea that the trapped chamber filters part of the flow ripple, while the sensor can still observe distinct tooth-related pressure valleys.

### 6. Measurement output

Measured pressure is derived from the visible pressure plus sensor effects:

```text
P_meas_bar = sat[0, 250](P_vis_bar + b_s + w_s)
```

Where:

- `b_s` is optional static sensor bias
- `w_s` is sensor noise
- `P_vis_bar = P_vis * 1e-5`

## Noise Assumptions

The design shall model only common noise sources that fit the stated task.

### Required noise sources

1. pressure sensor noise
2. motor speed fluctuation noise

### Optional but allowed disturbance

3. low-frequency process disturbance on net flow or leakage

### Sensor assumption

The specified pressure sensor is `Gefran KS-N-E-E-B25D-M-V`, range `0 .. 250 bar`.

If no exact accuracy sheet is introduced into the repo during implementation, assume a typical full-scale accuracy of `+-0.5% FS`, which is `+-1.25 bar`, and document that it is an engineering assumption rather than source-backed device characterization.

### Testability rule

Tests shall be able to:

- disable all noise for deterministic assertions
- or run with a fixed seed for reproducible stochastic outputs

## Parameter Defaults

The implementation should keep explicit defaults close to the existing draft unless a test requires adjustment:

- motor power: `5.5 kW` (documented machine parameter, not directly required in the equations yet)
- rated speed: `2000 rpm`
- pump displacement: `20 cc/rev`
- gear count: `13`
- speed range: `-100 .. 2000 rpm`
- pressure output range: `0 .. 250 bar`
- dead-head chamber effective volume: use the current draft value unless refactoring reveals a unit mistake
- effective bulk modulus: use the current draft value unless refactoring reveals a unit mistake
- relief pressure: `250 bar`

The model should remain parameterized so these defaults are explicit and test-visible.

## PLC FB Integration

The FB integration change should be minimal.

### Required behavior

- `ENABLE = 0`
  - reset model state
  - reset exposed outputs to zero

- `ENABLE = 1`
  - preserve model state across scans
  - call `PressureModel_Step(...)` once per scan
  - publish:
    - real pressure
    - measured pressure
    - actual motor speed

### State ownership

The FB adapter may keep one static `PressureModelState` instance for the existing single pressure-model FB surface.

The adapter must not:

- keep separate hidden motor state inside the update function
- rebuild dynamic state from scratch every scan

### Backward compatibility

If the current `pressure_update(...)` symbol is already consumed externally, implementation may keep it as a thin compatibility wrapper over the new API. The wrapper must not become the primary source of state.

## Test Design

Add a dedicated test target, for example `tests/test_pressure_model.c`, and keep it focused on plant-model behavior.

### Required model tests

1. zero-speed hold
   - input `0 rpm`
   - noise disabled
   - pressure remains near `0 bar`

2. calibration-point convergence
   - input `10 rpm`
   - noise disabled
   - steady-state pressure converges near `40 bar`

3. negative-speed depressurization
   - first build nonzero pressure
   - then command a small negative speed
   - pressure decreases faster than passive leakage-only decay

4. 13-tooth observability
   - fixed positive speed
   - noise disabled
   - measured pressure shows repeating tooth valleys
   - real pressure is smoother than measured pressure
   - one shaft revolution contains 13 observable tooth events

5. relief / saturation protection
   - high-speed command for sufficient time
   - measured output never exceeds `250 bar`
   - relief path becomes active near ceiling

6. motor-state continuity
   - speed step command
   - actual motor speed ramps continuously instead of restarting from zero each step

7. noise control
   - fixed seed gives repeatable output
   - disabled noise yields deterministic output

### Required FB-facing test

Extend the existing FB-side test surface with one narrow scenario:

- repeated `ENABLE=1` updates accumulate state across scans
- `ENABLE=0` clears state and output

This test belongs with the existing PLC adapter tests because it validates scan persistence rather than plant physics.

## Acceptance Criteria

The design is complete when implementation can satisfy all of the following:

1. `PressureModel.c` exposes explicit params/state/output boundaries
2. the physical model covers motor lag, pump flow, dead-head compression, leakage, relief, 13-tooth signature, and noise
3. no valve-open path is introduced
4. the known `10 rpm -> 40 bar` point is matched through physical calibration rather than a fitted curve table
5. negative rpm supports fast depressurization without producing negative pressure
6. dedicated pressure-model tests exist and pass
7. FB-side scan persistence is tested

## Risks and Mitigations

### Risk: tooth ripple is too weak or too strong

Mitigation:

- keep ripple amplitude and drop depth parameterized
- test for presence and boundedness, not a brittle exact waveform sample

### Risk: stochastic tests become flaky

Mitigation:

- default tests to noise-off or fixed-seed mode

### Risk: scope drifts into the general multi-axis simulator

Mitigation:

- keep all motion-simulator core files unchanged unless a compile boundary requires a header include

## Implementation Notes

- prefer deterministic math helper functions over hidden globals
- preserve the narrow scope around the pressure-model path
- keep units explicit in names and comments where the equations are easy to misuse
- avoid adding new dependencies
