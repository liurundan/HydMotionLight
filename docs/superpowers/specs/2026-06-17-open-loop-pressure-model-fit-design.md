# Open-Loop Pressure Model Fit Design

Date: 2026-06-17

## Goal

Upgrade `src/sim/PressureModel.c` from a generic dead-head pressure draft into an open-loop plant model that can approximate the measured data in `open10203040-positive.csv` and still remain useful for later closed-loop controller development.

The design shall:

- fit the positive-speed open-loop pressure data at `10 / 20 / 30 / 40 rpm`
- match both the pressure build-up transient and the steady-state pressure platform as closely as practical
- preserve the 13-tooth gear-pump signature visible in the measured pressure ripple
- keep a unified physical main model instead of degrading into one independent curve per speed
- allow only a very small set of weak speed-dependent correction terms
- preserve framework support for negative-speed depressurization / back-drive behavior
- treat torque as a derived trend output rather than a hard-fitting state target

The primary deliverable is a design for a tuned `PressureModel.c` implementation with embedded parameters and correction logic. No separate calibration tool is required for this task.

## Scope and Non-Goals

### In scope

- open-loop analysis of `open10203040-positive.csv`
- the structure and parameterization of `src/sim/PressureModel.c`
- fixed parameter embedding in code
- a modeling strategy that explains how to approximate both transient and steady-state behavior
- a derived torque-trend output strategy

### Out of scope

- valve-open hydraulic behavior
- a per-speed lookup-table model
- a standalone offline replay / optimizer tool
- hard quantitative fitting of torque sample-by-sample
- changing the broader multi-axis simulator architecture in `src/sim/hydro_sim.c`

## Measured Data Summary

The measured CSV is encoded in `gb18030` and contains these columns:

- `时间戳`
- `反馈压力`
- `电机转速`
- `原始压力`
- `设置压力`
- `设定速度`
- `反馈转矩`
- `反馈角度`

Units and engineering interpretation for this task:

- input speed command: `rpm`
- pressure feedback: `0.1 bar`
- torque feedback: `1e-4` engineering units
- timestamp period: `1 ms`
- motor feedback angle: `deg`

### Segment structure

The measured sequence is not steady-state-only. It alternates between `0 rpm` dwell sections and positive-speed build-up sections:

- `10 rpm`: `20472` samples
- `20 rpm`: `25311` samples
- `30 rpm`: `26381` samples
- `40 rpm`: `11077` samples

Each positive-speed section starts near zero pressure and includes the full pressure establishment process up to its steady-state region.

### Observed mean values

Measured section-wide average pressure and torque trend:

| Command speed (rpm) | Mean pressure (0.1 bar) | Mean pressure (bar) | Mean speed (rpm) | Mean torque |
| --- | ---: | ---: | ---: | ---: |
| 10 | 211.975 | 21.198 | 9.969 | 2761.874 |
| 20 | 546.624 | 54.662 | 19.956 | 6242.144 |
| 30 | 902.754 | 90.275 | 29.912 | 9845.566 |
| 40 | 1230.677 | 123.068 | 39.718 | 13238.207 |

Tail-region pressure averages over the last `2000` samples show the approximate steady-state platforms:

- `10 rpm`: about `21.4 bar`
- `20 rpm`: about `54.4 bar`
- `30 rpm`: about `88.8 bar`
- `40 rpm`: about `125.4 bar`

Head-region averages over the first `2000` samples show that the transient matters:

- `10 rpm`: about `17.5 bar`
- `20 rpm`: about `44.2 bar`
- `30 rpm`: about `73.7 bar`
- `40 rpm`: about `102.1 bar`

So the fitting target must include both the full build-up transient and the steady-state platform.

### Tooth-synchronous pressure signature

When pressure is aligned by:

```text
feedback_angle mod (360 / 13)
```

each steady-state section shows a stable pressure valley at nearly fixed tooth phase. The phase-aligned pressure span is approximately:

- `10 rpm`: `34.29` counts, about `3.43 bar`
- `20 rpm`: `49.95` counts, about `5.00 bar`
- `30 rpm`: `55.77` counts, about `5.58 bar`
- `40 rpm`: `54.22` counts, about `5.42 bar`

This is strong evidence that the dominant ripple is related to the pump's `13` teeth and is not just random sensor noise.

## Recommended Design

Use a unified physical main model with a very small number of weak speed-dependent correction terms.

This is preferred over per-speed empirical fitting because:

- the model needs to remain interpretable for later closed-loop controller development
- negative-speed behavior should fall out of the same state equations
- the measured data already supports a physical explanation: motor lag, compressibility, leakage, and tooth-synchronous ripple
- the non-ideal speed dependence is modest enough to handle with low-degree correction terms rather than independent segment curves

## Model Architecture

The model shall remain split into four layers:

1. motor-speed dynamics
2. pump flow generation
3. dead-head pressure state dynamics
4. measurement / visible-pressure layer

### Inputs

- `target_rpm`
- `dt_s`

### Core states

- `n_act`: actual motor speed in `rpm`
- `theta_rev`: accumulated pump phase in revolutions, wrapped to `[0, 1)`
- `P_real`: real trapped-oil pressure in `Pa`
- `rng_state`: deterministic random generator state

### Outputs

- `measured_pressure_bar`
- `real_pressure_bar`
- `actual_motor_rpm`
- `estimated_torque_trend`

### Architecture rule

The implementation must keep one unified main plant structure:

```text
target_rpm
  -> motor lag model
  -> pump base flow + 13-tooth phase
  -> compression / leakage / relief / negative-speed depressurization
  -> visible tooth valley shaping
  -> sensor output
```

The model must not branch into a separate pressure curve for `10`, `20`, `30`, or `40 rpm`.

## Mathematical Model

### 1. Motor-speed dynamics

```text
n_ref = sat(target_rpm)
alpha = dt / (tau_m + dt)
n_act(k+1) = sat(n_act(k) + alpha * (n_ref - n_act(k)) + w_n)
```

Where:

- `tau_m` is the motor time constant
- `w_n` is optional motor-speed noise
- the saturation keeps the model within the valid motor-speed range

This state captures the measured fact that section-head speed is slightly below section-tail speed before converging to the commanded level.

### 2. Pump phase and base flow

```text
theta_rev(k+1) = frac(theta_rev(k) + n_act * dt / 60)
Q_base = D * n_act / 60
```

Where:

- `D` is pump displacement in `m^3 / rev`
- `theta_rev` carries the angular state needed for tooth-synchronous effects

### 3. Tooth-related flow ripple

```text
Q_pump = Q_base * [1 + A_ripple * sin(2*pi*(13*theta_rev + phi_ripple))]
```

This process-side ripple gives the model a physically meaningful `13x` shaft-frequency component before the sensor layer adds the visible tooth valley.

### 4. Dead-head pressure dynamics

The trapped chamber remains the main pressure state:

```text
dP/dt = beta_e / Veff(n) * [Q_pump - Cleak(n) * P_real - Q_relief(P_real) + w_q]
P_real(k+1) = max(0, P_real(k) + dP/dt * dt)
```

Where:

- `beta_e` is effective bulk modulus
- `Veff(n)` is the effective trapped volume with weak speed dependence
- `Cleak(n)` is the effective leakage coefficient with weak speed dependence
- `Q_relief(P_real)` is the pressure-relief branch
- `w_q` is optional low-frequency process disturbance

This equation is responsible for the full-section build-up transient and the steady-state platform.

### 5. Tooth-synchronous visible pressure valley

The sensor-visible valley is modeled separately from the bulk pressure state:

```text
tooth_phase = frac(13*theta_rev + Phidrop(n))
```

If `tooth_phase` lies inside a finite tooth-drop window:

```text
P_vis = P_real * [1 - Adrop(n) * window(tooth_phase)]
```

otherwise:

```text
P_vis = P_real
```

The valley phase and amplitude are allowed only weak speed dependence. This matches the measured fixed-phase pressure dips without forcing the full chamber pressure state to oscillate unrealistically.

### 6. Sensor output

```text
P_meas_bar = clamp(P_vis * 1e-5 + bias + w_s, 0, sensor_range_bar)
```

Where:

- `bias` is optional sensor bias
- `w_s` is sensor noise

### 7. Negative-speed branch

The same pressure-state equation must remain valid for negative speed:

- `n_act < 0` makes `Q_base` and therefore `Q_pump` negative
- the same compression / leakage state equation then depressurizes the chamber
- `P_real` remains clamped to `>= 0`

If future measured negative-speed data shows systematic mismatch, one unified depressurization branch may be added later, but the current design must not introduce negative-speed lookup logic.

## Allowed Speed-Dependent Corrections

The model may use only this minimal correction set:

- `Veff(n)`
- `Cleak(n)`
- `Adrop(n)`
- `Phidrop(n)`

Their meanings are:

- `Veff(n)`: adjusts build-up slope and transient lag
- `Cleak(n)`: adjusts steady-state platform level
- `Adrop(n)`: adjusts tooth-valley depth
- `Phidrop(n)`: adjusts tooth-valley phase alignment

No other speed-dependent correction terms should be added unless the implementation fails verification with this minimal set.

### Parameterization rule

All four corrections shall be low-degree functions of `|n_act|`, not per-speed custom branches.

Recommended normalization:

```text
s = clamp(|n_act| / 40, 0, 1)
```

Recommended implementation form:

- three monotonic nodes at `0 rpm`, `20 rpm`, and `40 rpm`
- linear interpolation between nodes

This deliberately forces `10 rpm` and `30 rpm` to be explained by interpolation rather than dedicated independent parameters.

## Fixed Parameter Layout in `PressureModel.c`

The implementation should keep parameters grouped into five categories.

### 1. Base physical parameters

- `pump_displacement_m3_rev`
- `bulk_modulus_pa`
- `relief_set_pa`
- `sensor_range_bar`

### 2. Motor dynamic parameters

- `motor_tau_s`
- `motor_noise_std_rpm`

### 3. Main pressure parameters

- `veff_base_m3`
- `leak_base_m3_pa_s`
- `relief_coeff_m3_pa_s`

### 4. Tooth-signature parameters

- `flow_ripple_ratio`
- `tooth_drop_depth_base`
- `tooth_drop_width_ratio`
- `tooth_drop_phase_base`

### 5. Speed-correction arrays

- `veff_speed_scale[3]`
- `leak_speed_scale[3]`
- `drop_depth_scale[3]`
- `drop_phase_offset[3]`

The three correction nodes correspond to:

- `0 rpm`
- `20 rpm`
- `40 rpm`

This is intentionally lower freedom than a four-node `10 / 20 / 30 / 40 rpm` scheme.

## Parameter-to-Feature Mapping

Each tuning parameter exists for one measured effect first and only secondarily influences other effects.

### Motor lag

- parameter: `motor_tau_s`
- measured feature: section-head motor speed is slightly below section-tail motor speed before convergence

### Pressure build-up slope

- parameters: `veff_base_m3`, `veff_speed_scale`
- measured feature: each positive-speed section includes a non-instantaneous pressure establishment process

### Steady-state platform

- parameters: `leak_base_m3_pa_s`, `leak_speed_scale`
- measured feature: tail pressure levels at `10 / 20 / 30 / 40 rpm`

### Tooth-valley depth

- parameters: `tooth_drop_depth_base`, `drop_depth_scale`
- measured feature: about `3.4 .. 5.6 bar` phase-aligned pressure span

### Tooth-valley phase

- parameters: `tooth_drop_phase_base`, `drop_phase_offset`
- measured feature: tooth-synchronous pressure minima appear at nearly fixed phase, with a small speed-related drift

### Residual noise

- parameters: `sensor_noise_std_bar`, optional `process_noise_std`
- measured feature: non-synchronous jitter remaining after the deterministic trend and tooth signature are explained

## Torque Trend Output

Torque is a derived trend output, not a hard-fit state target.

Recommended form:

```text
T_est = k0 + k1 * P_meas + k2 * |n_act|
```

This is sufficient because the measured torque already shows the desired trend:

- higher speed -> higher mean torque
- higher pressure -> higher mean torque

The implementation only needs to preserve that monotonic relationship and approximate the section-average trend.

## Fitting Sequence

The fitting order is part of the design. Parameters must not be tuned all at once.

### Step 1: Disable stochastic noise

Temporarily disable:

- sensor noise
- motor noise
- process noise

First fit the deterministic structure.

### Step 2: Tune motor dynamics

Tune `motor_tau_s` against measured motor speed:

- startup region should lag slightly below the final value
- steady-state speed should converge near measured section means

### Step 3: Tune main pressure skeleton

Tune:

- `veff_base_m3`
- `veff_speed_scale`
- `leak_base_m3_pa_s`
- `leak_speed_scale`

Target:

- full-section pressure build-up shape
- tail steady-state pressure platform at each speed

### Step 4: Tune tooth signature

Tune:

- `flow_ripple_ratio`
- `tooth_drop_depth_base`
- `tooth_drop_width_ratio`
- `tooth_drop_phase_base`
- `drop_depth_scale`
- `drop_phase_offset`

Order:

1. phase alignment
2. valley depth
3. valley width

### Step 5: Reintroduce noise

After the deterministic structure matches:

- restore sensor noise
- restore motor noise
- add only minimal process noise if still needed

Noise must not be used to hide model-structure errors.

### Step 6: Tune torque-trend output

Finally tune the simple torque estimator coefficients:

- `k0`
- `k1`
- `k2`

Only average trend agreement is required.

## Acceptance Criteria

The model is considered good enough for this task when all of the following hold.

### A. Pressure transient

- each positive-speed section builds from near zero toward its measured platform
- the build-up shape is qualitatively and quantitatively close enough to the measured section
- the transient ordering is sensible: higher command speed must not create a slower or visibly inverted build-up shape without physical reason

### B. Pressure platform

Tail-region steady-state pressure should be close to measured levels:

- `10 rpm`: about `21.4 bar`
- `20 rpm`: about `54.4 bar`
- `30 rpm`: about `88.8 bar`
- `40 rpm`: about `125.4 bar`

### C. Tooth-synchronous feature

- clear `13`-tooth related ripple exists in the measured-pressure output
- valley phase stays near a stable angular location
- valley depth remains in the same order as the measured `3.4 .. 5.6 bar` span

### D. Speed response

- actual simulated motor speed must converge near the measured command section means
- no obvious long-term speed deficit is allowed at steady state

### E. Torque trend

- mean torque rises monotonically with speed and pressure
- average per-section torque trend matches the measured ordering

### F. Extrapolation rule

- the model structure must remain valid for negative-speed depressurization
- no implementation choice may lock the model to only the four measured positive-speed sections

## Explicit Rejections

The design explicitly rejects:

- independent `10 / 20 / 30 / 40 rpm` pressure curves
- dense speed-segment lookup logic
- fitting tooth ripple before fitting the main pressure skeleton
- using large random noise to mask deterministic mismatch
- feeding the torque estimate back into the pressure-state equation

## Implementation Guidance

The implementation pass should keep `PressureModel.c` small and testable:

- preserve one main stepping function
- isolate helper functions for clamping, interpolation, phase wrapping, and noise generation
- compute speed corrections through one shared interpolation helper
- keep torque estimation downstream from pressure simulation

The implementation should continue to support the existing `pressure_update(...)` compatibility surface unless a separate integration change is intentionally planned.

## Validation Guidance

No standalone calibration service is required.

If visual comparison is needed during development, any temporary service output should bind to `0.0.0.0:61479`, but the production implementation is expected to ship as embedded constants and logic inside `PressureModel.c`.
