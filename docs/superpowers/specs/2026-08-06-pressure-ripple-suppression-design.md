# 13 Tooth Gear-Pump Pressure Ripple Suppression Design

- Date: 2026-08-06
- Status: Approved design; implementation plan review is next
- Scope: Vertical injection-molding machine, servo motor driven 13-tooth external gear pump
- Control period: 1 ms

## 1. Objective

Reduce pressure ripple and pressure-step overshoot in the hydraulic pressure loop without
breaking existing P/PI/PID recipes or exceeding the real-time resources of an STM32H7.

The production pressure loop receives target pressure and measured pressure in bar and
ultimately commands servo-pump speed in RPM. The internal pressure-controller output remains
flow in L/min until the pump converter. The servo drive directly receives the final RPM command.

The approved production architecture is:

> Tracking positional RBF-PI core + incremental, rate-limited actuator output + fixed
> angle-synchronous 13th/26th-order RPM feedforward.

RBF-PID remains a compatible, experimental strategy. It is not the default pressure-hold
strategy and its adaptive D term is not enabled in the first production implementation.

## 2. Evidence and Current-System Findings

### 2.1 Measurement contract

The supplied open-loop recording has a fixed timestamp interval of 1 ms:

| Signal | Unit | Meaning |
|---|---:|---|
| Feedback pressure and target pressure | bar | Hydraulic pressure signals |
| Set speed and feedback speed | RPM | Servo-motor speed command and feedback |
| Feedback torque | 0.1% rated torque | Servo torque feedback, also called permille here |
| Feedback angle | deg modulo 360 | Electrical/mechanical feedback angle after modulo operation |

`open10203040-positive.csv` is an **open-loop speed test**, not a closed-loop pressure
step test. The pressure setpoint column is zero and the motor was directly commanded to
10, 20, 30, and 40 RPM. It can identify speed-loop and pump-ripple behavior, but cannot
prove a closed-loop overshoot result.

### 2.2 Open-loop observations

After removing the initial transition and evaluating stable windows synchronously against
the motor angle, the following approximate values were observed:

| Set RPM | Mean feedback RPM | Mean pressure (bar) | Tail pressure p-p (bar) | 13th-order pressure amplitude (bar) |
|---:|---:|---:|---:|---:|
| 10 | 9.97 | 212 | 53 | 12.8 |
| 20 | 19.96 | 547 | 73 | 19.1 |
| 30 | 29.91 | 903 | 97 | 22.6 |
| 40 | 39.72 | 1231 | 112 | 21.7 |

The dominant tooth-order frequency is `13 * RPM / 60`, or about 2.16, 4.32, 6.48, and
8.61 Hz in these four windows. The 26th and 39th orders are material and must not be
discarded without a calibrated residual-energy check. The speed feedback also shows a
fast oscillatory response after speed changes, so the existing fixed 60 ms first-order
motor model is not sufficient.

The pressure mean drifts over long open-loop windows. Identification must therefore use
separate train/validation windows and detrend or model the slow thermal/leakage drift;
a single stationary fit across the full CSV is invalid.

### 2.3 Current code issues

1. `src/rbf_pid.c` uses an incremental PID update:

   ```text
   du = KP * (e - e_prev) + KI * e + KD * second_difference(e)
   Output = clamp(u_prev + du)
   ```

   `KI` is applied per sample, while public pressure tuning describes continuous-time
   units. At a 1 ms period this creates an ambiguous, sample-rate-dependent gain meaning.

2. The current D term is the second difference of pressure error. It reacts strongly to
   sensor noise and gear-pump ripple; the optional pressure-acceleration feedforward has
   the same high-frequency sensitivity.

3. The RBF network currently identifies from `[du_prev, y_prev1, y_prev2]`. Replacing
   `du_prev` with `u_prev` is a model-contract change, not a one-line fix: old weights
   cannot be reused without retraining or deterministic reinitialization.

4. The RBF path has no explicit integral state. `u_prev` simultaneously represents the
   applied command, hidden integral accumulation, and bumpless-transfer state. It cannot
   correctly back-calculate after the final RPM limiter acts.

5. `systemGain` in the current RBF implementation primarily bounds a soft flow cap. It
   is not a calibrated inverse plant feedforward gain and must not be used as one.

6. The library initialization and the pressure-controller default RBF gain limits use
   different Kp scales. The resolved limits and the initialized gain must be made
   dimensionally consistent before tuning conclusions are drawn.

7. `src/sim/PressureModel.c` currently uses a constant bulk modulus, a sinusoidal ripple,
   a first-order motor model, mostly linear leakage, and a pressure-only tooth drop. The
   model does not yet represent the pump, line, and sensor mechanisms needed to validate
   ripple cancellation.

8. Existing `AxisFeedback`, `HydroPump`, and `HYD_PressureControllerInput` do not provide
   a complete angle/torque feedback chain. `motion_control.c` constructs a local pressure
   input without clearing it, so adding fields without updating every producer would be
   unsafe.

9. Existing controller tests mostly use ideal first-order, non-ripple plants. Passing
   them establishes API regression compatibility, not machine-level ripple suppression.

## 3. Constraints and Acceptance Criteria

### 3.1 Hard constraints

- C99, no dynamic allocation, deterministic execution in the 1 ms task.
- The control algorithm must run on STM32H7 hardware. CPU and memory claims must be
  measured on the target, not inferred from desktop execution.
- All production additions must have a complete producer -> consumer -> observable-effect
  chain. No placeholder fields, unused state, or one-way telemetry-only additions.
- Internal physical-model calculations use SI units. Boundary interfaces use bar, RPM,
  deg, and torque in 0.1% rated torque exactly as defined above.
- Existing P/PI/PID and legacy simulation profiles stay available unless an explicit
  migration is approved.

### 3.2 Closed-loop acceptance targets

For the same target pressure, load condition, speed range, and evaluation window:

- Pressure overshoot is at most 5% of the setpoint.
- 13th-order pressure amplitude is reduced by at least 50%.
- Total steady-state pressure peak-to-peak value is reduced by at least 35%.

The report must additionally show settling time, steady-state error, IAE, command
saturation ratio, RPM slew rate, and worst-case task execution time. At very low
setpoints, the report also records an absolute-bar value so a percentage alone does not
hide sensor-resolution effects.

## 4. Complete Feedback and Command Data Chain

A single feedback structure is required rather than scattered optional arguments. Task 1
ends at external producer and transport boundaries: `AxisFeedback`, `HydroPump`,
`PressureModelOutput`, simulator state, and simulator handles. It must not retain the
packet in `HYD_AxisRef`, diagnostics, or `HYD_PressureControllerInput`, because those
core snapshots are resource-constrained and have no consumer before ripple compensation.
Task 5 adds the first core consumer atomically through a transient per-cycle ingress.

The fields and their first consumer are fixed here:

| Field | Producer | First consumer | Required behavior |
|---|---|---|---|
| `measured_rpm` | Servo/HAL | ripple scheduler | Gain/phase table selection and low-speed gate |
| `motor_angle_deg` | Servo/HAL | phase tracker | 13th/26th tooth phase calculation |
| `torque_permille` | Servo/HAL | RBF learning gate and diagnostic | Freeze learning on invalid/inconsistent torque; record load consistency |
| `valid_flags` | Servo/HAL | controller safety gate | Bypass synchronized compensation or freeze learning |
| `timestamp` | control task | phase tracker | Validate period and angle increment |
| final applied RPM | pump converter/actuator mapper | PI tracking | Back-calculate after all limits and slew constraints |

The chain is:

```text
IEC/HAL -> HYD_PumpFeedback -> AxisFeedback/HydroPump/PressureModelOutput
        -> simulator state and public handle -> Task 5 transient motion ingress
        -> ripple compensator -> pump converter -> final applied RPM
        -> PI tracking state and diagnostics
```

Every stack instance of `HYD_PressureControllerInput` must be zero-initialized before its
fields are filled. Each simulator, IEC adapter, test fixture, and hardware adapter must
populate the same transport contract. If torque cannot drive the specified learning gate
in the first core consumer, it remains invalid rather than copied into unused core state.
Until Task 2 implements the calibrated physical torque equation, PressureModel packets
set torque to zero and clear `HYD_PUMP_FEEDBACK_VALID_TORQUE`; the legacy
`estimated_torque_trend` remains a compatibility diagnostic only.

No runtime temperature field is added in this plan because there is no confirmed sensor
chain. Temperature remains an offline calibration parameter until that chain exists.

## 5. Angle and Phase Handling

Feedback angle is already modulo 360 degrees. The phase tracker must not assume an
unbounded monotonically increasing angle.

For each valid 1 ms sample:

```text
dtheta = angle_deg(k) - angle_deg(k-1)
if dtheta >  180 deg: dtheta -= 360 deg
if dtheta < -180 deg: dtheta += 360 deg

shaft_phase = wrap_2pi(shaft_phase + dtheta * 2*pi / 360)
phase13 = wrap_2pi(13 * shaft_phase)
phase26 = wrap_2pi(26 * shaft_phase)
phase39 = wrap_2pi(39 * shaft_phase)
```

The tracker verifies timestamp monotonicity, a physical maximum angle increment derived
from the configured pump RPM limit, and agreement between angle direction and RPM sign.
A zero angle increment is valid when the pump is stopped; it is not by itself a fault.
An invalid angle disables synchronized compensation and preserves the base pressure loop.
There is no synthetic phase integration from speed alone for a claim of phase-locked
suppression.

`phase13` is the existing compensator's `tooth_phase`; `phase26` and `phase39` are
derived from the same shaft phase rather than independently integrated. A 13th-order
term is always evaluated as `sin(phase13 + phi13)`, never as `sin(13 * phase13 + phi13)`.

At the configured maximum RPM, the implementation must verify that the 13th order is
observable at 1 ms. If the order approaches the Nyquist limit or has too few samples per
cycle, the relevant compensator is disabled and a diagnostic is raised.

## 6. Production Pressure Controller

### 6.1 Controller selection

Pressure hold, back-pressure control, and the pressure-controlled side of V/P transfer
use RBF-PI as the production default. This matches the hydraulic object's leakage,
compressibility, sensor delay, and periodic pump disturbance more safely than adaptive
error-derivative control.

RBF-PID is retained for recipe compatibility and A/B tests. It may only use a fixed,
filtered measurement-rate damping path after model and closed-loop evidence show a
benefit. It does not use online adaptive KD in the first production release.

### 6.2 Tracking positional RBF-PI core

The core has an explicit position-form integral state, while the actuator interface keeps
an incremental command and slew limit for continuity:

```text
e_control = p_ref - p_filtered
p_term    = Kp * (beta * p_ref - p_filtered)
i_candidate = i_state + Ki * Ts * e_control

u_rate    = clamp(-Krate * LPF(dp_filtered/dt), -u_rate_max, u_rate_max)
u_raw     = u_ff + p_term + i_candidate + u_rate
u_base    = flow_limit_and_slew(u_raw)
```

`Ki` has the continuous unit L/min/(bar*s), `i_state` has the unit L/min, and `Ts` is
the measured control period. An alternative discrete `Ki_d` is allowed only if it has a
separate type/name and cannot be confused with the continuous parameter.

`beta` is a setpoint weight. It is chosen below one during a pressure rise if validation
requires lower setpoint kick. `u_rate` is a bounded, low-pass measurement-pressure-rate
damping term. It is not an error derivative, does not differentiate the setpoint, and is
normally zero in pressure hold.

After flow-to-RPM conversion, phase feedforward and all final RPM limits act. The base
PI state tracks the equivalent finally applied base flow using back calculation:

```text
n_base      = FlowToRPM(u_base)
n_command   = limit_rpm_slew(n_base + delta_n_ripple)
u_applied_base = RPMToFlowEquivalent(n_command - delta_n_ripple)
i_state_next = i_candidate + Kaw * Ts * (u_applied_base - u_raw)
du = u_applied_base - u_applied_base_previous
```

The exact converter inverse uses the existing pump conversion contract or a documented
equivalent. The known ripple injection is removed before anti-windup so the integrator
does not absorb the periodic compensation. Controller strategy changes, feedforward
changes, and gain changes initialize `i_state` from the current applied base output for
bumpless transfer.

### 6.3 Machine-phase scheduling

| Machine phase | Base controller | Measurement-rate damping | RBF learning | Ripple feedforward |
|---|---|---|---|---|
| Fill / injection velocity control | fixed PI or low-rate RBF-PI pressure limiter | only near a pressure ceiling | slow or frozen | normally off |
| V/P transfer and pressure rise | RBF-PI with setpoint ramp | bounded and optional | medium, no saturation | gated |
| Pressure hold | RBF-PI | off by default | slow, order-clean learning only | 13th/26th fixed tables |
| Relief / decompression | fixed PI or RBF-PI | off | frozen | off |

Machine phase is supplied by the motion state. The existing internal `BOOST`, `HOLD`,
and `RELIEF` error-ratio states are safeguards, not the sole source of scheduling truth.

### 6.4 RBF responsibilities and learning safety

The RBF network estimates local plant behavior and schedules only Kp and Ki. It does not
directly command a separate hidden output and does not adapt KD in the first release.

- `e_control` remains the pressure-loop error and is never overwritten for learning.
- `e_learning` is a separately filtered/order-clean residual used to gate or scale gain
  updates. Prediction residual for the RBF model remains separate as well.
- The intended RBF input contract is `[u_prev, p_prev1, p_prev2]` in normalized flow and
  pressure units. Changing from `du_prev` requires reidentification or reset of the
  network; it cannot silently retain old state.
- Learning freezes when angle, pressure, RPM, torque, or timestamp validity fails; when
  output is saturated in the same error direction; during relief; and while the output
  slew limiter is active beyond its allowed tracking threshold.
- Gain changes are rate limited and bumpless. RBF weight/gain adaptation can be decimated
  to every 4 to 10 control samples while the PI output still runs every 1 ms.

## 7. Angle-Synchronous Gear-Pump Ripple Feedforward

The first production release uses fixed calibrated feedforward, not a direct online
FxLMS implementation. This avoids relying on an unidentified secondary path and avoids
the sign and unit errors in the previous draft.

```text
delta_n_ripple = A13(|n|) * sin(tooth_phase + phi13(|n|))
               + A26(|n|) * sin(2 * tooth_phase + phi26(|n|))
```

`A` is in RPM and `phi` is in radians or degrees with one documented convention. The
sign is established by calibration against the physical plant, not hard-coded as a
generic subtraction. The table is indexed by absolute RPM and direction is handled
explicitly. A small pressure-dependent schedule is introduced only if validation proves
that a speed-only table leaves an unacceptable residual; it is not added preemptively.

The 39th order is measured and reported. It is included only when a 13th/26th controller
passes the resource gate but cannot meet the total p-p target. All compensation is
bounded, rate limited by the final RPM limiter, and disabled on invalid phase feedback.

Torque is a learning/diagnostic signal in this release, not a substitute for pressure
residual and not a fake source for an unimplemented Goertzel controller.

Online adaptive harmonic cancellation, if justified by residual data, is a later plan.
It requires an identified secondary path, normalized filtered-x update, coefficient
leakage, saturation gates, phase-direction testing, and a safe rollback condition.

## 8. Fluid-Equation Simulation Model

### 8.1 Profiles

`PRESSURE_MODEL_TYPE_FIRST_ORDER` remains a legacy regression profile. A new explicit
gear-pump physical profile is selected by one model-type enum, not by several conflicting
boolean switches. `PRESSURE_MODEL_TYPE_PHYSICAL_CALIBRATED` is retained as the canonical
source-compatible enum name, but default physical parameters are **uncalibrated** until
versioned raw-data identification and held-out validation succeed. Existing first-order
tests keep their legacy behavior.

The physical profile has two execution targets:

- Offline high-fidelity calibration: fixed small substeps and semi-implicit Euler or RK2.
- Embedded-equivalent HIL: one to four deterministic fixed substeps, no allocation, no
  optimizer, and table-based nonlinear functions where required.

Both profiles use the same parameter definitions and equations. The production pressure
controller does not execute this model in its 1 ms path. The standalone plant accepts a
transient input `{ target_rpm, load_flow_m3_s, dt_s }`; `PressureModel_Step()` remains the
source-compatible zero-load wrapper. `load_flow_m3_s` represents cylinder/load volume
consumption at the chamber and is not stored in a model parameter or PLC pin. A later HIL
integration may source it from an actual cylinder-flow trace only after the standalone
model passes validation.

### 8.2 Required equations and nonlinearities

All internal pressures below are in Pa and flows in m3/s. Outlet and chamber state
pressures are gauge pressures. `Patm` and `Psuction_abs` are explicit absolute-pressure
parameters, so `Pabs = Patm + max(Pgauge, 0)` is used wherever gas compressibility or
pump pressure differential is evaluated.

```text
Qideal = D * n / 60
DeltaPpump = max(Ps_abs - Psuction_abs, 0)
Qleak_pump = (C0 + Cn * |n|) * DeltaPpump
eta_v = clamp(1 - Qleak_pump / (D * |n| / 60 + Qepsilon), eta_v_min, 1)
Qleak_outlet = Coutlet * DeltaPpump
Qleak_cylinder = Ccylinder * max(Pc_abs - Psuction_abs, 0)

Qpump = sign(n) * D * |n| / 60 * eta_v
        * [1 + r13(Ps, |n|) * w13(phase13)
             + r26(Ps, |n|) * w26(phase26)
             + r39(Ps, |n|) * w39(phase39)]

dPs/dt = beta_e(Ps_abs) / Vs * (Qpump - Qline - Qleak_outlet - Qrelief)
Lline * dQline/dt = Ps - Pc - Rlinear * Qline - Rquadratic * Qline * |Qline|
dPc/dt = beta_e(Pc_abs) / Vc * (Qline - Qload - Qleak_cylinder)
```

The effective bulk modulus represents entrained gas with physically bounded parameters:

```text
alpha_g(Pabs) = clamp(alpha_g0 * Ptransition / max(Pabs, Ptransition), 0, alpha_g0)
1 / beta_e = (1 - alpha_g(Pabsolute)) / beta_o + alpha_g(Pabsolute) / Pabsolute
```

`alpha_g` may depend on pressure and is bounded. Temperature is not a runtime state in
this plan; any temperature effect is a calibrated offline parameter.

Pump ripple begins with separately gated, unit-peak sinusoidal 13th, 26th, and 39th
components. Their amplitudes are relative-to-mean **peak** flow amplitudes, not pressure
p-p. A calibrated asymmetric trapped-volume or relief-window waveform may be introduced
only by expanding it into explicit independently gated Fourier terms; otherwise a disabled
26th or 39th component can leak through an enabled 13th waveform and alias at 1 ms. The
total delivered-flow multiplier is clamped to a positive bounded interval. Pump leakage is
accounted for once through volumetric efficiency; outlet and cylinder leakage are separate
pressure-difference-driven paths. The model must not multiply a cosmetic tooth drop only
onto visible pressure.

The servo speed model has a measured delay, a second-order or otherwise identified speed
response, and acceleration limiting. It must not retain an arbitrary fixed 60 ms
first-order constant after the CSV fit rejects it. `motor_torque_limit_permille` is a
feedback/telemetry clamp in this phase; it does not constrain speed dynamics until a
measured torque-to-acceleration relation and rotor-equivalent inertia are introduced.
The model provides a true torque signal when torque is evaluated:

```text
eta_m = clamp(eta_m_nominal - eta_m_pressure_loss_per_pa * DeltaPpump
               - eta_m_speed_loss_per_rpm * |n|, eta_m_min, 1)
Tdc_nm = sign(n) * DeltaPpump * D / (2 * pi * eta_m)
torque_permille = clamp(1000 * Tdc_nm *
                        [1 + torque_ripple13_peak *
                         sin(phase13 + torque_ripple13_phase_rad)] /
                        rated_motor_torque_nm,
                        -torque_limit_permille, torque_limit_permille)
```

The torque-valid bit is set only when the computation and rated torque are finite and
positive. The existing `estimated_torque_trend` is not treated as this harmonic measurement.

Relief flow uses a nonlinear deadband/orifice relation with hysteresis where data
supports it. Its setpoint, deadband, and hysteresis are gauge-pressure thresholds.
Sensor delay, quantization, bias, and bounded noise are added only as
model states that are consumed by the measured-pressure output.

The pressure observation point is fixed for this plan:

```text
real_pressure_bar     = Pchamber_gauge / 1e5
measured_pressure_bar = delayed_quantized_sensor(Pchamber_gauge) / 1e5
controller input      = measured_pressure_bar
```

Outlet/manifold pressure remains an internal pump/line/relief diagnostic. Moving the
modelled sensor upstream requires a new documented sensor-placement contract and a new
validation fixture.

The physical model does not add actuator-position or temperature states unless a real
motion/temperature input consumes them in the current phase.

### 8.3 Fixed-step and order-resolution policy

The embedded-equivalent path accepts only an exact integer multiple of the 1 ms task
period: 1, 2, 3, or 4 ms. It uses that many 1 ms deterministic substeps; nonfinite,
nonpositive, nonintegral, or over-limit intervals use one 1 ms safe fallback step. Each
substep updates delayed motor command, motor speed, line flow, then outlet and chamber
pressure semi-implicitly. Delay queues therefore retain their fixed 64 ms meaning and
have no dynamic allocation.

At the observable 1 ms sampling interval, an `m`th tooth order is enabled only when
`m * abs(rpm) / 60 < 0.45 / dt`. This gives a 10% guard below the nominal 1 ms Nyquist
limits of approximately 1154 RPM for 26th and 769 RPM for 39th. The implemented guard
thresholds are approximately 1038 RPM for 26th and 692 RPM for 39th, rather than silently
aliasing either flow wave.
An unresolved order is omitted from embedded-equivalent output and reported by replay;
it is not turned into a fictitious low-frequency pressure component.

The calibrated physical profile owns only the pump-to-chamber fluid chain. The existing
`HydraulicSim` kinematic cylinder remains a separate simulator in this plan. Coupling it
requires a later explicit adapter that derives `load_flow_m3_s` from a real cylinder state
and is prohibited from being claimed as closed-loop plant validation before then.

### 8.4 Parameter Admissibility and Replay Calibration

`PressureModel_ValidatePhysicalParams()` is the intrinsic deterministic C99 physical-block
contract. It rejects nonfinite values; nonpositive volumes, inertance, oil modulus, or rated
torque; `beta_min_pa > beta_oil_pa`; efficiencies outside `(0, 1]`; negative
leakage/resistance/ripple limits; delays outside `[0, 64 ms]`; and a 1 ms hydraulic stiffness ratio
`beta_oil_pa * (0.001 s)^2 / (line_inertance * min(outlet_volume, chamber_volume)) > 0.25`.
`PressureModel_ValidateParams()` is the full runtime-parameter contract used by physical
tests and host replay before a calibrated run. It rejects pump displacement above
`50e-6 m3/rev`, configured RPM ranges outside `[-2000, 2000]`, motor natural frequency
above `50 Hz`, motor damping above `2`, or motor acceleration limit above `100000 RPM/s`.
`PressureModel_ValidateInput()` validates each physical scan and rejects nonfinite target
or load flow, target RPM outside the configured `[min_rpm, max_rpm]` range, and load-flow
magnitude above `1.666667e-3 m3/s`. It intentionally does not reject an irregular `dt_s`,
because `PressureModel_StepInput()` normalizes only that case to one safe 1 ms substep;
physical target RPM is rejected rather than normalized. The line update uses its semi-implicit
resistance denominator; a nonpositive denominator is also invalid. Invalid physical
settings do not receive broad silent clamping: replay fails, and the physical step holds
its last finite chamber pressure with zero flow and an invalid torque packet.

Before each physical integration, runtime admission also evaluates the combined worst-case
1 ms pressure increment using `beta_oil`, the smaller outlet/chamber volume, the maximum
configured displacement and RPM, the `1.8` peak ripple multiplier, and the maximum
opposing load. The result must not exceed `25 MPa` per scan. This rejects finite but
physically/numerically unusable parameter combinations that can pass the hydraulic
stiffness-ratio test alone. A defensive post-step check restores the previous finite state
and emits a zero-flow invalid-torque safe hold if an unforeseen arithmetic failure occurs;
it never clamps an overflowed pressure trajectory into a plausible result. Invalid current
physical state is rejected before delay-ring access and preserved unchanged except for the
normal finite scan timestamp advance.

For a valid multi-substep physical scan, integration is atomic: if any later 1 ms substep
cannot complete, the model restores the state captured at scan entry, advances only the finite
requested scan timestamp, and emits hold output. This prevents a failed 4 ms scan from exposing
a partially integrated plant state.

Task 2 ends with an explicitly uncalibrated deterministic `pressure_model_replay` baseline.
Task 3 exclusively owns KV-calibrated replay, including its strict versioned
`identified_params.kv` parser, verifier, load admission, and calibrated output labeling. The
Task 3 calibrated invocation is:

```text
pressure_model_replay physical <rpm> <samples> <identified_params.kv>
```

`identified_params.kv` is emitted beside `identified_params.json`, contains every consumed
physical parameter plus `schema_version` and a Python-computed SHA-256 `calibration_id`.
Its strict file parser belongs only to the Task 3 test/replay binary, never the 1 ms
production path. The validation script verifies the JSON/KV calibration ID match and
records that ID in its report and replay CSV header before comparing held-out data.

## 9. Identification and Validation Gate

### 9.1 Identification order

1. Fit the servo speed loop from command RPM and feedback RPM.
2. Fit mean pressure, pump leakage, cylinder/system leakage, and volumetric efficiency.
3. Fit 13th, 26th, and measured 39th order amplitude/phase by angle-synchronous
   demodulation, not a fixed-frequency FFT across varying RPM.
4. Fit bulk modulus, line inertia/damping, relief behavior, and sensor delay from
   transients. Parameters not identifiable from the constant-speed hold windows must be
   supplied by a versioned hardware/transient manifest with source and acquisition window;
   absent provenance yields "model not calibrated".
5. Hold out independent windows with different initial phase and pressure level for
   validation.

### 9.2 Model acceptance before control claims

The calibrated physical model must meet all of the following on held-out open-loop data:

- Mean-pressure error no more than `max(5%, 5 bar)`.
- 13th-order amplitude error no more than 20% and phase error no more than 15 degrees.
- 26th-order amplitude error no more than 30%.
- Speed-response peak, settling time, and steady-state error match the held-out speed
  steps within limits recorded in the identification report.

If these gates fail, the result is "model not calibrated". No controller architecture is
declared ready for machine use from that model.

### 9.3 Controller validation matrix

The validated model runs the following comparisons at representative low, mid, and high
production pressure levels, with different initial tooth phases and modeled load changes:

1. Existing PI baseline.
2. Existing PID/RBF-PID baseline.
3. Tracking positional RBF-PI with no ripple feedforward.
4. Tracking positional RBF-PI with calibrated 13th feedforward.
5. Tracking positional RBF-PI with calibrated 13th/26th feedforward.

Closed-loop pressure targets must be nonzero recorded targets or an approved synthetic
test profile. The supplied open-loop CSV cannot be reused as evidence for overshoot.
The successful candidate then requires new closed-loop machine data before enabling it
outside a controlled validation mode.

Unit and integration coverage must include:

- 359-to-0 and 0-to-359 angle wrap, direction reversal, invalid timestamps, and lost angle;
- pressure/RPM/torque validity gates and fallback behavior;
- continuous Ki units at 1 ms and controlled behavior under permitted period jitter;
- controller tracking after final RPM saturation and slew limiting;
- no compensation when the pump is stopped or phase is invalid;
- physical versus first-order model profile selection and legacy-profile regression;
- gain-change and strategy-change bumpless transfer;
- 13th/26th amplitude and phase evaluation on deterministic simulated data.

## 10. STM32H7 Resource and Safety Gate

- No dynamic allocation, recursion, FFT, optimizer, or filesystem activity in the 1 ms
  production task.
- The ripple generator uses a bounded sine/cosine lookup table with interpolation, not
  per-cycle trigonometric calls.
- The existing six-node, three-input RBF network is not enlarged in this plan.
- RBF adaptation may be decimated; the PI, limiter, and phase gate execute every cycle.
- Static RAM and flash deltas are inspected in the link map. Each new state member is
  accounted for by one of the data-chain consumers above.
- Worst-case control-task execution time is measured with the STM32H7 DWT cycle counter
  under pressure control, diagnostics, and communications load. The provisional gate is
  less than 20% of the 1 ms period; the exact board clock and scheduling budget are
  recorded with the test evidence.
- Nonfinite inputs, invalid phase, invalid torque, and output saturation produce a
  defined fallback: base PI/RBF-PI remains active, synchronized compensation is bypassed,
  and RBF learning is frozen. Fault recovery is bumpless.

## 11. Scope and Deferred Work

### Current implementation plan

The next implementation plan covers the complete feedback chain, calibrated physical
simulation profile, tracking positional RBF-PI core, final-output tracking, fixed 13th/
26th feedforward, diagnostics, and the validation tests above. It removes or corrects
the prior draft's incorrect angle, sign, unit, and model assumptions.

### Explicitly deferred to a later approved plan

- Online FxLMS/filtered-x harmonic adaptation.
- Temperature-scheduled runtime compensation.
- A pressure-and-speed two-dimensional feedforward table if a speed-only table fails the
  validated residual criterion.
- Additional 39th-order compensation.
- Extra actuator mechanical states when a real position/load interface needs them.

No public data member or embedded state for these deferred functions is added beforehand.

## 12. Corrections to the Previous Draft

This specification supersedes the earlier draft assumptions that:

- feedback angle is unbounded and only needs floating-point accumulation protection;
- the 13th order contains 90% of all relevant ripple and 26th/higher order may be ignored;
- a single-frequency direct LMS update is safe without secondary-path identification;
- `a += mu * pressure_error * cos` and a generic subtraction have a universally correct
  hydraulic sign and unit;
- torque trend can serve as a real torque harmonic;
- a constant leakage coefficient is a valid `eta_v(P, n)` model;
- first-order simulation defaults can validate a physical-pressure implementation;
- a new interface field can be introduced without updating all construction and consumer
  sites.

## 13. Decision Record

The machine pressure loop will be optimized in the following order:

1. Validate the physical model against open-loop speed/angle/pressure data.
2. Establish a stable tracking positional RBF-PI base loop with final-output anti-windup.
3. Add fixed calibrated 13th/26th RPM feedforward and prove the stated ripple targets.
4. Obtain closed-loop machine evidence for overshoot and hold behavior.
5. Consider adaptive harmonic cancellation only if fixed compensation cannot meet the
   residual target and the secondary path is identified.

This order prevents the system from using adaptive control to compensate for an
unidentified pump, an invalid phase signal, or a hidden actuator saturation.
