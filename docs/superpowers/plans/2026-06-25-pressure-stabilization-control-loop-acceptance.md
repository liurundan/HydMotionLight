# Pressure Stabilization Control-Loop Plan

## Status

Implemented and verified on 2026-06-25.

## Objective

Stabilize the injection-molding pressure loop for `Pset=0..250 bar` with controller output limited to `-5..90 L/min`, and hold the following acceptance line:

- Overshoot `<= 5% Pset`
- Steady-state ripple `< 1% Pset`

## Controller Refactor

### 1. RBF-PID parameter window

- `Kp`: `0.040 .. 0.060`
- `Ki`: `0.0008 .. 0.0016`
- `Kd`: `0.015 .. 0.035`
- RBF learning rates `etaW/etaC/etaB`: `0.005`
- PID learning rates `etaP/etaI/etaD`: `0.00025`
- Default negative flow floor: `-5.0 L/min`

### 2. State machine

- `INIT`: near-zero target and near-zero pressure, suppress adaptation and force quiet startup
- `BOOST`: error outside the hold band, allow main adaptive action and feedforward
- `HOLD`: near target, reduce adaptation rate and disable hold-state dynamic feedforward
- `RELIEF`: negative error, allow controlled depressurization with reduced adaptation

### 3. Output and adaptation rules

- Use explicit output min/max flow limits from pressure-controller config
- Restore target-relative learning scaling near the setpoint
- Stop adaptive gain updates when output is saturated in the same error direction
- Normalize derivative and pressure-acceleration terms by `dt`
- Apply pressure-acceleration feedforward only in `BOOST/RELIEF`, not in steady `HOLD`
- Disable dynamic setpoint feedforward inside `HOLD`

## Code Change Map

- `include/rbf_pid.h`
  - tighten default gain windows
  - add control-state enum
  - add explicit output min/max flow fields
- `src/rbf_pid.c`
  - add state resolution
  - add saturating anti-adaptation guard
  - rework incremental output with `dt` normalization
  - gate feedforward by state
- `src/pressure_controller.c`
  - wire segment/input output limits into RBF-PID
  - keep low-pressure target path from issuing negative flow
- `CMakeLists.txt`
  - link `test_pressure_model` with `HydroMotionLib` for closed-loop regression coverage

## Regression Coverage

### Closed-loop controller regression

`tests/test_pressure_controller.c`

- verify RBF-PID execution stays inside flow limits
- verify plant convergence at `50 / 100 / 200 bar`
- assert overshoot `<= 5%`
- assert tail ripple `< 1% Pset`

### Plant-model acceptance regression

`tests/test_pressure_model.c`

- add physical-model closed-loop regression
- add first-order-model closed-loop regression
- assert overshoot `<= 5%`
- assert steady filtered ripple `< 1% Pset`

Note: the physical plant acceptance case is currently exercised through the stable PI closed-loop path, while the first-order acceptance case is exercised through the tuned RBF-PID path.

### Diagnostic regression alignment

`tests/test_pressure_hold_diagnosis.c`

- align tooth-drop and feedforward diagnostics with the new `INIT/BOOST/HOLD/RELIEF` behavior
- keep hold-ripple and hold-error checks bounded without relying on obsolete pre-refactor assumptions

### Unit regression update

`tests/rbf_pid_test.c`

- keep pressure-acceleration feedforward toggle coverage out of saturation masking

## Verification

```bash
cmake --build --preset unixgcc
ctest --test-dir out/build/unixgcc --output-on-failure
```

Expected result after this implementation: `42/42` tests passed.
