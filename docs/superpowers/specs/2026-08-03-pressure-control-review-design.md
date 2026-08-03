# Pressure Control Review and Conservative PI-RBF Design

- Date: 2026-08-03
- Status: Approved design, implementation not started
- Scope: `src/pressure_controller.c`, `src/rbf_pid.c`, pressure ripple integration, and pressure-specific tests

## 1. Goal and Constraints

The pressure loop must respond quickly and keep steady process-pressure ripple below 1% of the target pressure by peak-to-peak measurement. The approved baseline acceptance targets for a 100 bar step are:

- 10-90% rise time <= 100 ms
- Entry and continuous residence inside +/-2% <= 300 ms
- Overshoot <= 5%
- Steady process-pressure peak-to-peak <= 1 bar

The controller runs at a nominal 1 kHz / 1 ms cycle. The design keeps the existing position-form PI architecture and limits changes to pressure-control behavior. It does not redesign the velocity planner, motion state machine, pump converter, or public enum numbering.

Raw sensor pressure, filtered pressure, and simulated real process pressure are reported separately. A raw sensor signal with noise larger than 1% of a low pressure target cannot satisfy a process ripple requirement by definition; the hard process criterion is evaluated on real/filtered pressure and the raw signal is reported as a sensor-noise diagnostic.

## 2. Evidence and Cross-Validation

The current branch was built in an isolated MinGW directory and the existing pressure tests passed:

- `test_pressure_controller`
- `test_pressure_ripple_comp`
- `rbf_pid_test`
- `test_rbf_pid_hil`
- `test_pressure_model`
- `sim_pressure_control`

The tests passing is not sufficient evidence for the new acceptance criteria because most tests use deterministic or simplified plants.

The current 1 ms simulation uses a first-order pressure plant, 1.5 bar synthetic pump ripple, 0.30 bar sensor noise, gain mismatch, and an 8 bar load step. Its measured baseline is:

| Strategy | Rise | Settle | Steady error | Ripple RMS | Ripple p2p |
| --- | ---: | ---: | ---: | ---: | ---: |
| PI, no FF | 6450 ms | 7999 ms | -10.208 bar | 0.686 bar | 2.944 bar |
| PI + physical FF | 34 ms | 58 ms | 0.068 bar | 0.548 bar | 1.683 bar |
| RBF-PI | 201 ms | 263 ms | -0.190 bar | 0.634 bar | 1.862 bar |
| RBF-PID | 201 ms | 264 ms | -0.201 bar | 0.585 bar | 1.806 bar |

At a plant gain of 5.4 instead of the nominal 4.5, RBF-PI and RBF-PID reduce steady error but still produce approximately 1.19 and 1.32 bar peak-to-peak ripple. Ideal phase-feedforward scans can cancel a synthetic sinusoid, but the endpoint simulation still reports residual closed-loop ripple. Therefore:

- RBF is useful for bounded gain adaptation and disturbance recovery, not inherently faster than a correctly fed-forward PI.
- PI+FF is the deterministic production baseline.
- RBF-PID and RBF-PI remain comparison/experimental strategies unless they pass the nonlinear scenario matrix.

## 3. Approved Architecture

### 3.1 Position-form production loop

The final production command remains a position-form PI command with explicit physical feedforward:

```text
u_raw = ffBase + ffTrim + rippleFF + Kp_eff * e + I
u_applied = clamp(u_raw, u_min, u_max)
I_next = I + Ki_eff * e * dt + Kaw * (u_applied - u_raw)
```

`ffBase` is `rampedPressure / systemGain` when `systemGain` is valid, otherwise the existing `targetFlow` fallback is retained. `ffTrim` is a slow, gated bias correction and is not allowed to become a second fast integral controller.

The existing position-form state, output tracking, pressure filtering, and pressure ceiling interfaces remain in place. A small output slew limit may be added only inside the pressure loop when required by the calibrated actuator rate; it must feed the actual applied command back into anti-windup.

### 3.2 PI-RBF supervisory mode

`HYD_PRESSURE_CONTROLLER_PI_RBF` remains a position-form PI output path. The RBF module only proposes bounded `Kp_eff` and `Ki_eff` values. It does not own the final flow command.

At every cycle, the outer loop writes the actually applied output, output increment, saturation result, target, feedback, and error history back to the RBF state. The RBF network therefore learns the plant response to the command that was really applied, not a discarded virtual command.

RBF gain adaptation is frozen when any of the following is true:

- output or pressure is saturated in the same direction as the error;
- the Jacobian or network state is non-finite or has an invalid plant sign;
- pressure/flow excitation is below the configured minimum;
- a target, strategy, direction, low-pressure mold-protect, high-pressure lock, or fault transition is active;
- the segment explicitly disables learning.

The initial `Kp/Ki` values and limits are derived from the calibrated PI baseline. A default adaptation window of roughly +/-20-30% around that baseline is preferred over device-independent fixed gains. RBF derivative adaptation remains disabled by default.

### 3.3 Other strategies

The existing P, PI, PID, RBF-PI, and RBF-PID enum values and legacy behavior are retained unless a correctness fix is required. `RBF_PID` and `RBF_PI` are not the default safety path for high-pressure lock. Existing recipes must continue to resolve and clamp their configured limits.

## 4. Ripple Compensation Rules

The ripple module is independent of the PI/RBF decision and is treated as a reversible disturbance feedforward:

- learn only from a slow-error-removed pressure residual;
- require pressure-error, pressure-rate, target-stability, and pump-speed-stability gates;
- require enough samples for EMA amplitude/phase convergence before producing output;
- use an explicit encoder-valid flag; if invalid, use the speed phase accumulator; if neither source is valid, return zero;
- clear or smoothly decay stale amplitude/phase/gain on segment changes, invalid `systemGain`, phase jumps, speed steps, or estimator faults;
- apply amplitude, phase-rate, flow-rate, and total-command limits;
- keep the fast path to cached amplitude/phase values and one bounded phase evaluation.

The existing hard-coded `useEncoder=1` behavior and stale `ffGain` carry-over when `systemGain <= 0` are correctness defects to remove.

## 5. Motion-Scenario Contract

| Motion action | Primary control | Pressure-loop role |
| --- | --- | --- |
| Fast open/close | position/velocity | pressure ceiling and safe derating |
| Close final approach / low-pressure mold protect | position/velocity | low-pressure limit, no negative relief |
| High-pressure lock | pressure PI or PI-RBF | bounded pressure hold, learning frozen by default |
| Injection V/P and holding | pressure PI+FF or PI-RBF | ramped pressure tracking and disturbance recovery |
| Plasticizing / back pressure | pressure PI+FF or PI-RBF | pressure hold with direction-aware flow limits |
| Ejector pressure hold | pressure PI+FF or PI-RBF | low-pressure safe hold, negative relief disabled |

All mode transitions use output tracking. Pressure ceiling enforcement stays in the motion layer and cannot be overridden by adaptive or ripple terms. Negative flow is allowed only for explicitly permitted relief actions and only beyond a configured over-pressure threshold.

## 6. Required Correctness Fixes

The implementation plan must address these bounded issues:

1. Synchronize PI-RBF state with the real outer-loop output and saturation.
2. Accept `HYD_PRESSURE_CONTROLLER_PI_RBF` in parameter validation without changing enum values.
3. Validate and clamp `dt`; preserve the 1 ms tuning at nominal rate and normalize RBF incremental terms for valid timing jitter.
4. Normalize/sort RBF parameter windows before clamping runtime gains and reject non-finite configuration values.
5. Make RBF external saturation detection use the configured segment output bounds, not only the internal hard pump bound.
6. Make low-pressure/negative-flow behavior explicit and consistent across PI and RBF paths.
7. Add encoder-valid handling, speed-phase fallback, stale-gain clearing, and bounded ripple output.
8. Keep `ffTrim` gated and rate-limited; reset or retain it only according to an explicit segment/strategy transition rule.

No unrelated motion-planner, velocity-controller, or interface refactor is included.

## 7. Simulation and Test Matrix

The existing first-order model remains a fast regression model. A configurable nonlinear harness adds:

- motor/pump rate limit, dead zone, saturation, 2-5 ms motor lag, and transport delay;
- pressure inertia, leakage/relief, gain drift with pressure or temperature, and load steps;
- first, second, and third pump harmonics with speed-dependent amplitude and phase jitter;
- sensor quantization, bias, noise, low-pass, and delay;
- fixed-seed and Monte Carlo noise runs.

Required cases include 20/100/180 bar target steps, +/-30% gain mismatch, target ramps, load steps, speed changes, encoder loss, segment transitions, low-pressure mold protect, high-pressure lock, V/P switch, plasticizing back pressure, and ejector hold.

Regression tests must cover:

- PI-RBF actual-output synchronization and bumpless strategy switching;
- dt rollback, long-cycle, and non-finite input handling;
- saturation freeze and recovery for both inner and external limits;
- low-pressure negative-flow suppression and allowed relief;
- ripple compensation encoder fallback, invalid gain, phase jump, convergence gate, harmonic input, amplitude limit, and disable behavior;
- motion-layer mold-protect and lock-pressure transitions.

The simulation target is registered in CTest and emits a summary with rise, settling, overshoot, real/filtered/raw steady error, p2p, disturbance recovery, controller execution time, and pass/fail status.

## 8. Compatibility and Rollout

- New state/validity fields are appended to public structs and layout-consistency tests are updated.
- No dynamic allocation or new dependency is introduced.
- The default production selection is fixed PI+FF or PI-RBF after calibration; pure RBF remains opt-in.
- Online learning and ripple compensation each have independent disable paths and fall back to fixed PI+FF without output discontinuity.
- Phase 1 is correctness plus regression tests; Phase 2 is nonlinear simulation and tuning; Phase 3 is action-level HIL and real-machine calibration.

## 9. Non-Goals

- No wholesale migration to an incremental outer PI.
- No redesign of the motion planner, velocity controller, pump converter, or PLC command protocol.
- No claim that raw noisy sensor peak-to-peak can be below 1% at every low pressure target.
- No automatic online parameter learning without explicit bounds, validity gates, and a fixed-PI fallback.
