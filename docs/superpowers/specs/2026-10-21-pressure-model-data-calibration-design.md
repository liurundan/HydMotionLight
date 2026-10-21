# 1 ms Dead-Head Pressure Model Calibration Design

## Goal

Calibrate `PressureModel.c` against the supplied 1 ms open-loop speed records and
closed-loop RBF-PID records so offline pressure-control tests have the measured pressure
rise, leakage platform, negative-load behavior, tooth-synchronous ripple, sensor noise,
and resisting torque sign.

## Evidence and scope

The supplied open-loop records contain positive-speed sections at 8, 10, 12, 15, and
20 RPM. Their final 2000-sample filtered-pressure means are 47.90, 56.14, 75.38,
95.57, and 122.82 bar; the corresponding peak-to-peak envelopes are 4.42, 4.48,
5.85, 7.42, and 12.17 bar. The 1 s and 5 s samples show that pressure establishes
within the first few seconds. The closed-loop record holds 50, 80, 100, 120, and
140 bar with final-window peak-to-peak values from 2.12 to 5.63 bar.

These records identify the dead-head pressure state and visible disturbance envelope.
They do not independently identify valve-open line dynamics, temperature, or cylinder
load, so those parameters remain explicit but are not overfit.

## Model boundary

The existing `PressureModel_StepInput` boundary and first-order/integrator compatibility
branches remain unchanged. The calibrated physical branch keeps the second-order motor
response, pump displacement, volumetric loss, effective bulk modulus, relief latch,
negative-speed depressurization, and deterministic RNG. Because every supplied test has
the oil valves closed, the physical branch uses one effective trapped-oil pressure state.
The outlet pressure and line-flow state are still populated for diagnostics and ABI
compatibility; an unobserved line resonance is not inserted into the calibrated dead-head
response.

The measured layer is separated from the real pressure state:

```text
target RPM
  -> motor dynamics and pump phase
  -> pump flow, leakage, compressibility, relief
  -> real trapped pressure
  -> 13/26/39 order pressure envelope + sensor delay/quantization/noise
  -> raw_pressure_bar / measured_pressure_bar
```

The periodic pressure disturbance is relative to real pressure and is gated by the
existing process-ripple switch. Setting `flow_ripple_ratio = 0` disables it for
deterministic controller tests while retaining the physical pressure equation.

## Embedded parameters

- Effective outlet/chamber volume: `7.0e-4 m^3`.
- Entrained gas fraction: `2.0e-5`, selected from the one-second pressure-rise evidence.
- Pump, outlet, and cylinder leakage: `3.4e-13`, `1.7e-13`, and `1.7e-13 m^3/(Pa s)`.
- Sensor-visible periodic pressure ratio: `0.05`, with 13th/26th/39th order weights
  `0.82 / 0.13 / 0.05`.
- Resisting motor torque uses the negative sign observed in the recordings and remains
  a derived trend from pressure differential and pump displacement.

## Verification

`tests/test_pressure_model_data_fit.c` replays the five open-loop speeds at a fixed 1 ms
period and checks pressure at 1 s and 5 s, tail pressure, tail ripple, torque trend, and
zero-speed pressure release. Existing `test_pressure_model`, `test_pressure_hold_diagnosis`,
and `test_hydro_sim_fb` retain the legacy branch, controller-hold, relief, switching, and
FB lifecycle contracts. `scripts/analyze_pressure_model_data.py` regenerates
`docs/pressure-model-calibration.md` directly from the supplied CSV files.

## Limits

The calibrated profile is intended for the supplied dead-head and pressure-hold cases.
It is not evidence for valve-open flow, thermal drift, or hardware timing; those require
separate measurements before production claims are made.
