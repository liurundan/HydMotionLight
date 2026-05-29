# RBF PID Pressure Controller Plant-Model Test Design

**Date:** 2026-05-29
**Scope:** Add plant-model convergence tests for `HYD_PressureController_Execute` using `HYD_PRESSURE_CONTROLLER_RBF_PID`, with `pump_converter` in the loop.

## Goal

Validate that the RBF PID adaptive pressure controller converges stably against a first-order inertia plant model, with pump_converter flow->RPM transformation verified at each step. All units are engineering units (bar).

## Plant Model

### Transfer function

```
G(s) = K / (T*s + 1)     K = 5.4 bar/RPM    T = 1.0 s
```

Input: pumpSpeed (RPM), Output: pressure (bar).

### Backward difference discretization (Ts = 0.001 s)

```
pressure_bar(k) = (T * pressure_bar(k-1) + K * Ts * pumpSpeed(k)) / (T + Ts)
                = (pressure_bar(k-1) + 0.0054 * pumpSpeed(k)) / 1.001
```

### Steady-state

At steady state, pressure = K * pumpSpeed. E.g., 100 bar target requires ~18.52 RPM steady-state pump speed, which with flowToPumpSpeedGain=20 implies ~0.93 L/min steady-state flow.

## Data Chain (single step)

```
1. pressure_controller.Execute(target_bar, feedback_bar)
   -> outputFlow (L/min)

2. pumpSpeed = clamp(outputFlow * flowToPumpSpeedGain, 0, pumpSpeedLimit)
   -> pump_converter transformation verified at each step

3. pressure_bar(k+1) = (pressure_bar(k) + 5.4 * 0.001 * pumpSpeed(k)) / 1.001
   -> plant model update

4. feedback_bar = pressure_bar(k+1)
   -> fed back to step 1 on next iteration
```

## Controller Configuration

| Parameter | Value |
|-----------|-------|
| pressureController | `HYD_PRESSURE_CONTROLLER_RBF_PID` |
| targetFlow (feedforward) | 0.0 |
| maxFlow | 90.0 (= pumpSpeedLimit / flowToPumpSpeedGain = 1800/20) |
| flowToPumpSpeedGain | 20.0 |
| pumpSpeedLimit | 1800.0 |
| pressureCeiling | targetPressure * 3 (for proper normalization) |
| All other params | defaults (no segment-level RBF override) |

## Test 1 -- Single-Setpoint Convergence

Run 3 separate sub-tests, one per target: 50 bar, 80 bar, 100 bar.

**Procedure** (per target):
1. Init pressure controller state with `initialPressure=0`, `initialOutputFlow=0`
2. Configure segment with RBF PID, target, maxFlow=90, pressureCeiling=target*3
3. Run 5000 iterations (5s simulated) with Ts=0.001s
4. Feed pumpSpeed through plant model, feed pressure_bar back as measuredPressure

**Assertions** (per target):
- Final pressure within +-2% of target after 5000 steps
- Peak pressure does not exceed target + 5% (overshoot < 5%)
- `pumpSpeed == outputFlow * 20.0` at each step (within floating-point tolerance)
- `pumpSpeed >= 0` and `pumpSpeed <= 1800` at each step
- `outputFlow >= 0` and `outputFlow <= 90` at each step
- PID gains (KP, KI, KD) remain within default bounds throughout
- No oscillation in the last 1000 steps: `|outputFlow(k) - outputFlow(k-1)| < 1.0` for all k in [4000, 4999]

## Test 2 -- Setpoint Switching

**Procedure**:
1. Init state as in Test 1, start at 50 bar
2. Every 2000 steps (2s), switch target: 50 -> 80 -> 100 -> 50 bar
3. Reconfigure segment with new target and pressureCeiling on each switch
4. Total: 8000 steps (8s)

**Assertions**:
- No oscillation divergence during or after any switch
- Each target reached within +-5% by end of its 2000-step window
- First step after each switch does not produce a flow spike > 50% above the previous step's flow
- pump_converter relationship holds at every step
- PID gains remain bounded throughout

## Location

Add to `tests/test_pressure_controller.c` (2 new test functions).

## Acceptance Criteria

1. All assertions pass with default RBF PID parameters (no manual tuning)
2. `ctest --test-dir out/build/unixgcc -R '^test_pressure_controller$' --output-on-failure` passes
3. Tests complete in reasonable time (< 1 second)
