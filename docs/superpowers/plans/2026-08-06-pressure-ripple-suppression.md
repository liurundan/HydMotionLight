# Pressure Ripple Suppression Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement and validate a calibrated 13-tooth gear-pump pressure-control path with a tracking positional RBF-PI core, fixed 13th/26th-order RPM feedforward, and a fluid-equation simulation model that is approved before machine activation.

**Architecture:** Keep the existing pressure-controller flow domain and public P/PI/PID recipe compatibility. Add one unified pump-feedback packet, a tracking positional RBF-PI core with an incremental/rate-limited actuator boundary, and an angle-synchronous RPM compensator after flow-to-RPM conversion. Extend `PressureModel` with an explicit calibrated physical profile while preserving the legacy first-order profile; use the same physical parameter meanings in host analysis and embedded-equivalent HIL.

**Tech Stack:** C99, CMake preset `unixgcc`, existing C test binaries, Python 3 standard library (`csv`, `math`, `json`, `statistics`) for deterministic data analysis, STM32H7 target build with DWT cycle measurement supplied by the board integration.

**Spec:** `docs/superpowers/specs/2026-08-06-pressure-ripple-suppression-design.md`

---

## Execution Gates

1. The implementation branch is created before any C, header, test, or script implementation:

   ```bash
   git switch -c feature/pressure-ripple-suppression
   git branch --show-current
   ```

   The second command must print `feature/pressure-ripple-suppression`.

2. The branch is based on the committed specification (`015e716`) and a clean baseline build/test is recorded before the first code edit.

3. Every implementation task ends with a focused test and a Lore-protocol commit. Do not stage `.omx/metrics.json` or unrelated worktree changes.

4. No merge to `master` is performed after host tests alone. The merge gate requires:
   - held-out physical-model validation;
   - STM32H7 worst-case 1 ms task timing and memory evidence;
   - closed-loop machine data showing overshoot <= 5%, 13th-order amplitude reduction >= 50%, and total pressure p-p reduction >= 35%.

5. Only after the hardware evidence is reviewed may the branch be merged:

   ```bash
   git switch master
   git merge --no-ff feature/pressure-ripple-suppression
   git log --oneline -2
   ```

## File Map

| File | Responsibility in this plan |
|---|---|
| `include/common_types.h` | `HYD_PumpFeedback` and validity bits; core axis snapshots remain unchanged |
| `include/hydro_hardware.h` | Unified pump feedback in the hardware pump object while retaining legacy RPM compatibility |
| `include/hydro_interfaces.h` | Sensor-backend feedback packet propagation |
| `include/pressure_controller.h` | Existing pressure input and applied-output tracking API; no retained packet until Task 5 |
| `include/rbf_pid.h`, `src/rbf_pid.c` | Tracking positional RBF-PI state, continuous Ki semantics, learning gates |
| `include/ripple_compensator.h`, `src/ripple_compensator.c` | Modulo-360 phase tracker and fixed 13th/26th RPM compensation |
| `include/pressure_model.h`, `src/sim/PressureModel.c` | Physical pressure model parameters, states, outputs, and legacy profile compatibility |
| `include/hydro_sim.h`, `src/sim/hydro_sim.c`, `src/sim/hydro_sim_fb.c` | Simulator pump feedback and pressure-model output propagation |
| `include/motion_control.h`, `src/motion_control.c` | Existing motion core; Task 5 adds transient per-cycle feedback ingress with the first consumer |
| `src/motion_interface.c` | Preserve PLC axis feedback behavior without adding packet IEC pins |
| `tests/test_pump_feedback_chain.c` | Feedback packet validity and propagation tests |
| `tests/test_ripple_compensator.c` | Angle wrap, phase, gate, table, sign, and limit tests |
| `tests/test_pressure_model.c`, `tests/test_hydro_sim_fb.c` | Physical-model equations, profile switching, and simulator output tests |
| `tests/pressure_model_replay.c` | Fixed-step C replay stream consumed by held-out validation scripts |
| `tests/test_pressure_controller.c`, `tests/rbf_pid_test.c` | RBF-PI tracking, Ki/Ts, anti-windup, and bumpless-transition tests |
| `tests/test_pressure_closed_loop_ripple.c` | Deterministic physical-model closed-loop metrics and controller comparison |
| `tests/fixtures/open10203040_measurement_reference.h` | Measured open-loop summary used only for model validation |
| `scripts/ripple/analyze_open_loop.py` | Standard-library CSV parsing and angle-synchronous order metrics |
| `scripts/ripple/fit_physical_model.py` | Bounded deterministic parameter fitting and held-out validation report |
| `scripts/ripple/export_ripple_table.py` | Calibration-table export after model/order gates pass |
| `docs/ripple-analysis/` | Reproducible calibration reports and STM32H7 resource evidence |
| `CMakeLists.txt` | New source/test targets and CTest registrations |

## Task 0: Create the Implementation Branch and Capture the Baseline

**Files:** None modified.

- [ ] **Step 1: Confirm the specification commit and worktree boundary**

Run:

```bash
git log --oneline -- docs/superpowers/specs/2026-08-06-pressure-ripple-suppression-design.md docs/superpowers/plans/2026-08-06-pressure-ripple-suppression.md
git status --short
```

Expected: the specification commit `015e716` and plan commit `898d327` are present. Only the unrelated `.omx/metrics.json` may remain modified; it is never staged for implementation.

- [ ] **Step 2: Create and verify the implementation branch**

Run:

```bash
git switch -c feature/pressure-ripple-suppression
git branch --show-current
```

Expected: the current branch is exactly `feature/pressure-ripple-suppression`. If the branch already exists, verify it contains both `015e716` and `898d327` before using `git switch feature/pressure-ripple-suppression`.

- [ ] **Step 3: Configure and run the unmodified baseline**

Run:

```bash
mkdir -p docs/ripple-analysis
{
  cmake --preset unixgcc
  cmake --build --preset unixgcc --target test_pressure_controller test_pressure_model test_hydro_sim_fb rbf_pid_test
  ctest --test-dir out/build/unixgcc --output-on-failure
} 2>&1 | tee docs/ripple-analysis/baseline-host-tests.txt
```

Expected: configuration succeeds, the four focused binaries build, and the existing CTest suite is green; the complete transcript is captured in `docs/ripple-analysis/baseline-host-tests.txt` on the implementation branch.

- [ ] **Step 4: Commit the baseline evidence**

```bash
git add docs/ripple-analysis/baseline-host-tests.txt
git commit -m "记录压力控制改造前的主机基线" -m "保存物理模型和RBF控制器改造前的可重复测试结果。\n\nConstraint: 代码改造必须能区分既有回归和新增行为\nRejected: 直接在未记录基线的情况下修改控制器 | 无法判断行为回归\nConfidence: high\nScope-risk: narrow\nDirective: 后续任务保留此基线，不修改旧测试以掩盖失败\nTested: cmake --preset unixgcc; cmake --build --preset unixgcc --target test_pressure_controller test_pressure_model test_hydro_sim_fb rbf_pid_test; ctest --test-dir out/build/unixgcc --output-on-failure\nNot-tested: STM32H7 target timing is not available in the host baseline"
```

## Task 1: Define and Propagate the Unified Pump-Feedback Packet

**Files:**
- Modify: `include/common_types.h`
- Modify: `include/hydro_hardware.h`
- Modify: `include/hydro_interfaces.h`
- Modify: `include/pressure_controller.h`
- Modify: `include/motion_control.h`
- Modify: `src/motion_control.c`
- Modify: `src/motion_interface.c`
- Modify: `src/sim/hydro_sim.c`
- Modify: `src/sim/hydro_sim_fb.c`
- Create: `tests/test_pump_feedback_chain.c`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Write the failing feedback-contract test**

Create `tests/test_pump_feedback_chain.c` with these assertions:

```c
HYD_PumpFeedback feedback;
memset(&feedback, 0, sizeof(feedback));
assert(feedback.validFlags == 0u);
assert(!HYD_PumpFeedback_HasValid(feedback.validFlags,
                                  HYD_PUMP_FEEDBACK_VALID_ANGLE));

feedback.rpm = 20.0;
feedback.angleDeg = 359.5;
feedback.torquePermille = 6242.0;
feedback.timestamp = 1.000;
feedback.validFlags = HYD_PUMP_FEEDBACK_VALID_RPM |
                      HYD_PUMP_FEEDBACK_VALID_ANGLE |
                      HYD_PUMP_FEEDBACK_VALID_TORQUE;
assert(HYD_PumpFeedback_HasValid(feedback.validFlags,
                                 HYD_PUMP_FEEDBACK_VALID_ANGLE));
assert(feedback.torquePermille == 6242.0);
```

Register it as `test_pump_feedback_chain` in `CMakeLists.txt`, then run:

```bash
cmake --build --preset unixgcc --target test_pump_feedback_chain
```

Expected: the new test fails to compile because the unified packet and validity helper do not yet exist.

- [ ] **Step 2: Add the packet and validity bits**

Add the following C99-compatible contract to `include/common_types.h`:

```c
enum {
    HYD_PUMP_FEEDBACK_VALID_RPM = 1u << 0,
    HYD_PUMP_FEEDBACK_VALID_ANGLE = 1u << 1,
    HYD_PUMP_FEEDBACK_VALID_TORQUE = 1u << 2,
    HYD_PUMP_FEEDBACK_VALID_TIMESTAMP = 1u << 3
};

typedef struct {
    HYD_REAL rpm;
    HYD_REAL angleDeg;
    HYD_REAL torquePermille;
    HYD_TIME timestamp;
    uint32_t validFlags;
} HYD_PumpFeedback;

static inline HYD_BOOL HYD_PumpFeedback_HasValid(uint32_t flags, uint32_t required) {
    return (flags & required) == required;
}
```

Add `HYD_PumpFeedback feedback` to `HydroPump` while keeping the legacy `feedback_rpm`
field populated for existing hardware adapters. Do not add the packet to `HYD_AxisRef`,
`HYD_PressureControllerInput`, diagnostics, or another retained core snapshot before an
actual core consumer exists. This preserves the hard motion-FB resource cap and prevents
unused state from becoming an ABI obligation.

- [ ] **Step 3: Propagate simulator feedback without PLC layout changes**

Extend `AxisFeedback` and the simulator environment with one `HYD_PumpFeedback` packet.
`HydraulicSim_ReadAxis()` returns it, `Hyd_CopyAxisFeedbackToHandle()` copies it to the
simulation handle, and `__mcl_cmd_updatePressureModel()` copies the model output packet.
Do not add new IEC pins to `HYD_PRESSUREMODEL`. Task 1 ends at these producer/transport
boundaries; Task 5 introduces the first core consumer through a transient per-cycle
feedback ingress, rather than storing the packet in `AXIS_REF`.

- [ ] **Step 6: Run the focused test and commit**

```bash
cmake --preset unixgcc
cmake --build --preset unixgcc --target test_pump_feedback_chain test_toggle_mechanism_pool test_hydro_sim_fb test_pressure_model
ctest --test-dir out/build/unixgcc -R '^(test_pump_feedback_chain|test_toggle_mechanism_pool|test_hydro_sim_fb|test_pressure_model)$' --output-on-failure
git add include/common_types.h include/hydro_hardware.h include/hydro_interfaces.h include/hydro_sim.h include/hydro_sim_fb.h include/pressure_model.h src/sim/PressureModel.c src/sim/hydro_sim.c src/sim/hydro_sim_fb.c tests/test_pump_feedback_chain.c CMakeLists.txt
git commit -m "建立伺服泵反馈生产和传输边界" -m "让统一反馈包从仿真与压力模型生产者稳定传播到公开观察点，而不在没有消费者的运动核心快照中持久化。\n\nConstraint: 保持HYD_MotionControlFB资源上限和PLC引脚布局\nRejected: 预先将packet写入AXIS_REF或压力输入 | 引入无消费的长期RAM和ABI负担\nConfidence: high\nScope-risk: narrow\nDirective: Task 5必须通过瞬态每周期入口原子引入第一个核心消费者\nTested: test_pump_feedback_chain; test_toggle_mechanism_pool; test_hydro_sim_fb; test_pressure_model\nNot-tested: STM32H7目标板资源测量"
```

## Task 2: Add the Uncalibrated Fluid-Equation Pressure Model Profile

`PRESSURE_MODEL_TYPE_PHYSICAL_CALIBRATED` remains the source-compatible canonical enum
name, but Task 2 defaults are explicitly **uncalibrated**. They verify equation signs,
fixed-step causality, and transport contracts only; they are not calibration evidence.

**Files:**
- Modify: `include/pressure_model.h`
- Modify: `src/sim/PressureModel.c`
- Modify: `tests/test_pressure_model.c`
- Modify: `tests/fixtures/pressure_model_open_loop_reference.h`
- Create: `tests/fixtures/open10203040_measurement_reference.h`
- Create: `tests/pressure_model_replay.c`
- Modify: `CMakeLists.txt`
- Modify: `src/sim/hydro_sim_fb.c`

- [ ] **Step 1: Add physical-model regression tests before implementation**

Extend `tests/test_pressure_model.c` with deterministic tests for the new explicit physical
profile. The moving-load path is explicit rather than hidden in a parameter: add
`PressureModelInput` and exercise `PressureModel_StepInput()` with both zero and positive
`load_flow_m3_s`. Keep `PressureModel_Step()` as the compatibility wrapper that supplies
zero load flow.

```c
params.model_type = PRESSURE_MODEL_TYPE_PHYSICAL_CALIBRATED;
params.physical.gas_fraction = 0.002f;
params.physical.pump_leak_c0_m3_pa_s = 1.0e-12f;
params.physical.pump_leak_speed_m3_pa_s_per_rpm = 1.0e-14f;
params.physical.ripple13_peak = 0.12f;
params.physical.ripple26_peak = 0.04f;

PressureModel_Reset(&state, 0x2468ace1u);
run_steps(&params, &state, 20.0f, 20000, 0.001f, &out);
assert(out.real_pressure_bar > 0.0f);
assert(out.actual_motor_rpm > 0.0f);
assert(out.pumpFeedback.angleDeg >= 0.0f && out.pumpFeedback.angleDeg < 360.0f);
assert((out.pumpFeedback.validFlags & HYD_PUMP_FEEDBACK_VALID_RPM) != 0u);
assert((out.pumpFeedback.validFlags & HYD_PUMP_FEEDBACK_VALID_ANGLE) != 0u);
assert((out.pumpFeedback.validFlags & HYD_PUMP_FEEDBACK_VALID_TORQUE) != 0u);
```

Add separate assertions that increasing outlet pressure increases pump leakage, low
pressure has lower effective bulk modulus than high pressure, relief flow is monotonic
after its deadband, positive load flow lowers chamber pressure relative to the same
zero-load run, negative speed reduces pressure, and torque contains a deterministic
13th-order component when enabled. Cover a finite/bounded 1 ms step, a delayed sensor
sample, quantization, and the order-resolution gate. Run the test before implementation
and record the expected compile or assertion failure.

- [ ] **Step 2: Introduce a nested physical-parameter block and state variables**

Add `PressureModelPhysicalParams` inside `PressureModelParams` with only parameters used
by the equations below. `atmospheric_pressure_pa` and `suction_pressure_pa` are **absolute**
pressures. The outlet and chamber states remain gauge pressures, which avoids using a
gauge pressure in the gas-compressibility denominator.

```c
typedef struct {
    float atmospheric_pressure_pa;
    float suction_pressure_pa;
    float outlet_volume_m3;
    float chamber_volume_m3;
    float line_inertance_pa_s2_per_m3;
    float line_resistance_pa_s_per_m3;
    float line_quadratic_resistance_pa_s2_per_m6;
    float beta_oil_pa;
    float gas_fraction;
    float gas_transition_pa;
    float beta_min_pa;
    float pump_leak_c0_m3_pa_s;
    float pump_leak_speed_m3_pa_s_per_rpm;
    float outlet_leak_m3_pa_s;
    float cylinder_leak_m3_pa_s;
    float eta_v_min;
    float eta_m_nominal;
    float eta_m_pressure_loss_per_pa;
    float eta_m_speed_loss_per_rpm;
    float eta_m_min;
    float rated_motor_torque_nm;
    float torque_ripple13_peak;
    float torque_ripple13_phase_rad;
    float ripple13_peak;
    float ripple26_peak;
    float ripple39_peak;
    float ripple13_phase_rad;
    float ripple26_phase_rad;
    float ripple39_phase_rad;
    float motor_natural_freq_hz;
    float motor_damping;
    float motor_delay_s;
    float motor_accel_limit_rpm_s;
    float motor_torque_limit_permille;
    float relief_set_pa;
    float relief_deadband_pa;
    float relief_orifice_coeff_m3_s_sqrt_pa;
    float relief_hysteresis_pa;
    float sensor_delay_s;
    float sensor_quantization_bar;
} PressureModelPhysicalParams;

typedef struct {
    float target_rpm;
    float load_flow_m3_s;
    float dt_s;
} PressureModelInput;
```

Use fixed-size `PRESSURE_MODEL_PHYSICAL_MAX_DELAY_STEPS` rings for delayed motor command
and sensor pressure (bounded to 64 ms), together with motor acceleration, outlet gauge
pressure, line flow, relief-latch state, and the existing chamber pressure, phase, and
timestamp fields. Do not duplicate chamber pressure: `PressureModelState.pressure_pa`
remains the chamber gauge-pressure state for both profiles. Preserve the legacy
first-order fields and delay buffer unchanged for the old profile. Every new state member
must be consumed by one physical equation or output path.

The state stores shaft phase in the existing `pump_phase_rev`; derive `phase13`, `phase26`,
and `phase39` as 13, 26, and 39 times that shaft phase. Do not store three independently
integrated tooth phases. `PressureModelOutput.real_pressure_bar` is chamber gauge pressure;
`measured_pressure_bar` is the delayed/quantized/bias/noise sensor image of that chamber
pressure. Outlet pressure is only a pump/line/relief diagnostic in this task.

Expose `PressureModel_ValidatePhysicalParams()` for physical tests and host replay. It
must reject invalid/nonfinite calibrated parameters rather than quietly clamping them:
positive finite volumes/inertance/modulus/rated torque, `beta_min <= beta_oil`, efficiencies
in `(0, 1]`, nonnegative leak/resistance/ripple values, delays no greater than 64 ms, and
the fixed-1-ms hydraulic stiffness ratio must satisfy the documented stability bound.

Retain and complete the `PressureModelOutput.pumpFeedback` packet introduced by Task 1;
keep `estimated_torque_trend` as a compatibility diagnostic only. Legacy and first-order
profiles keep torque at zero with `HYD_PUMP_FEEDBACK_VALID_TORQUE` clear. The calibrated
physical profile adds the rated-torque conversion and is the first profile whose
regression test asserts the torque valid bit, together with RPM, modulo-360 angle, and
timestamp.

- [ ] **Step 3: Implement the bounded nonlinear equations**

Implement static helpers in `src/sim/PressureModel.c` for:

```text
Pout_abs = Patm + max(Pout_gauge, 0)
Pchamber_abs = Patm + max(Pchamber_gauge, 0)
DeltaPpump = max(Pout_abs - Psuction_abs, 0)
Qleak_pump = (C0 + Cn * abs(rpm)) * DeltaPpump
eta_v = clamp(1 - Qleak_pump / (D * abs(rpm) / 60 + epsilon), eta_v_min, 1)
Qleak_outlet = Coutlet * DeltaPpump
Qleak_cylinder = Ccylinder * max(Pchamber_abs - Psuction_abs, 0)

alpha_g(Pabsolute) = clamp(gas_fraction * gas_transition_pa /
                           max(Pabsolute, gas_transition_pa), 0, gas_fraction)
1 / beta_e = (1 - alpha_g(Pabsolute)) / beta_oil
             + alpha_g(Pabsolute) / max(Pabsolute, 1 Pa)
beta_e = clamp(beta_e, beta_min_pa, beta_oil_pa)

phase13 = 2 * pi * 13 * pump_phase_rev
phase26 = 2 * pi * 26 * pump_phase_rev
phase39 = 2 * pi * 39 * pump_phase_rev
Qpump = sign(rpm) * D * abs(rpm) / 60 * eta_v *
        clamp(1 + r13 * w13(phase13) + r26 * w26(phase26) + r39 * w39(phase39))
Lline * Qline_dot = Pout_gauge - Pchamber_gauge
                   - Rlinear * Qline - Rquadratic * Qline * abs(Qline)
Pout_dot = beta_e(Pout_abs) / Vout * (Qpump - Qline - Qleak_outlet - Qrelief)
Pchamber_dot = beta_e(Pchamber_abs) / Vchamber * (Qline - Qload - Qleak_cylinder)
```

Use a bounded asymmetric lookup/Fourier waveform for 13th, 26th, and optional measured
39th order. `ripple_waveform` is a multiplier clamped to a positive bounded interval and
is applied to delivered pump flow, never to visible pressure after integration. The pump
leakage is already represented by `eta_v`, so it is not subtracted a second time from the
outlet balance. Use a nonlinear relief deadband/orifice relation and a second-order/delayed
motor response with an acceleration limit. `motor_torque_limit_permille` limits the
reported torque packet only in this phase; it cannot be called a motor-dynamics limit until
measured torque-to-acceleration data supports a rotor-inertia extension. Do not multiply a
cosmetic pressure drop after the fluid integration.

Use semi-implicit Euler for the embedded-equivalent path: update delayed motor command,
motor speed, line flow, then pressure states with the newest upstream values. A physical
call accepts only exact integer multiples of 1 ms up to 4 ms and uses that many 1 ms fixed
substeps. Nonfinite, nonpositive, nonintegral, or over-limit intervals fall back to one
1 ms step. This preserves the fixed 64 ms delay-ring meaning and is a safe HIL fallback,
not permission to run an arbitrary slow plant.
The host calibration path may use the same fixed substeps or RK2, but it must share
parameter names and physical signs with C.

At a 1 ms output interval, enable an order only when
`order * abs(rpm) / 60 < 0.45 / dt_s`; otherwise disable that unresolved component in the
embedded-equivalent model and record it in the replay report. This gates 26th order before
the nominal 500 Hz Nyquist boundaries (about 1154 RPM for 26th and 769 RPM for 39th) with
a 10% guard band; a model must not silently alias them into an apparently calibrated
pressure wave.

Generate true torque feedback as:

```text
eta_m = clamp(eta_m_nominal - eta_m_pressure_loss_per_pa * DeltaPpump
               - eta_m_speed_loss_per_rpm * abs(rpm), eta_m_min, 1)
Tdc_nm = sign(rpm) * DeltaPpump * D / (2 * pi * eta_m)
Tnm = Tdc_nm * [1 + torque_ripple13_peak *
                 sin(phase13 + torque_ripple13_phase_rad)]
torque_permille = clamp(1000 * Tnm / rated_motor_torque_nm,
                        -torque_limit_permille, torque_limit_permille)
```

Set `HYD_PUMP_FEEDBACK_VALID_TORQUE` only when every torque operand is finite and
`rated_motor_torque_nm > 0`. Use SI Pa/m3/s/Nm internally and convert only at
`PressureModelOutput` boundaries.
`relief_set_pa`, `relief_deadband_pa`, and `relief_hysteresis_pa` are gauge-pressure
thresholds, so relief does not accidentally open at atmospheric pressure.

- [ ] **Step 4: Preserve model-profile switching and legacy tests**

Define `PRESSURE_MODEL_TYPE_PHYSICAL_CALIBRATED` as the canonical calibrated enum value
and retain `PRESSURE_MODEL_TYPE_PHYSICAL` as a source-compatible alias with the same
numeric value for existing callers. Keep `PRESSURE_MODEL_TYPE_FIRST_ORDER` as the default
for legacy tests. On a
profile switch, hold the current pressure and reset only states that cannot be mapped; do
not silently enable physical parameters while the first-order profile is selected.

Update `src/sim/hydro_sim_fb.c` to copy `out.pumpFeedback` into the native feedback packet.
Add `tests/pressure_model_replay.c` and the CMake target `pressure_model_replay`. In Task 2
it accepts `physical|first_order`, an RPM value, and a sample count and emits only the
deterministic `calibration_id=uncalibrated` skeleton. A supplied `identified_params.kv`
must fail clearly as uncalibrated; Task 2 must not load or validate held-out calibration
data. The replay writes one CSV row per 1 ms step with actual RPM, chamber real/measured
pressure, angle, torque, timestamp, validity bits, active-order mask, and a
calibration-status header. Task 3 owns the strict host-only parser and calibrated invocation:

```bash
./out/build/unixgcc/pressure_model_replay physical 20 20000 docs/ripple-analysis/identified_params.kv > docs/ripple-analysis/replay-20rpm.csv
```

Do not change IEC `HYD_PRESSUREMODEL` field order or add PLC pins in this task.
`PressureModel_Step()` supplies `load_flow_m3_s = 0` for this legacy PLC path. A future
motion-coupled HIL task may provide the instantaneous cylinder/load-flow trace through
`PressureModelInput`; it must not add a retained PLC field or pretend that the standalone
plant is already coupled to `HydraulicSim`.

- [ ] **Step 5: Update fixtures and run physical-model tests**

Create `tests/fixtures/open10203040_measurement_reference.h` only from the checked-in raw
open-loop measurement and record its source window. The expected summary is command RPM
10/20/30/40, 1 ms samples, actual-RPM means 9.97/19.96/29.91/39.72, tail pressure p-p
53/73/97/112 bar, and angle-synchronous 13th amplitudes 12.8/19.1/22.6/21.7 bar. Keep the
old fixture for legacy regression and label its values as model baseline rather than
machine data. Without the raw source and held-out windows, the fixture is a test target,
not calibration evidence.

Run:

```bash
cmake --build --preset unixgcc --target test_pressure_model test_hydro_sim_fb
ctest --test-dir out/build/unixgcc -R '^(test_pressure_model|test_hydro_sim_fb)$' --output-on-failure
```

Expected: legacy profile tests and the new deterministic physical-equation tests pass.
This demonstrates equation consistency only; the model remains uncalibrated until Task 3
loads a versioned parameter file and later held-out raw-data validation passes.

- [ ] **Step 6: Commit the model profile**

```bash
git add include/pressure_model.h src/sim/PressureModel.c src/sim/hydro_sim_fb.c tests/pressure_model_replay.c tests/test_pressure_model.c tests/fixtures/pressure_model_open_loop_reference.h tests/fixtures/open10203040_measurement_reference.h CMakeLists.txt
git commit -m "建立13齿齿轮泵物理压力模型" -m "在保留一阶回归模型的同时加入压缩性、泄漏、容积效率、管路、溢流和真实转矩/角度输出。\n\nConstraint: 仿真模型必须可在C99固定步长下复现并可映射到STM32H7 HIL\nRejected: 用可见压力乘齿槽压降继续伪造纹波 | 无法验证流体闭环控制\nConfidence: medium\nScope-risk: broad\nDirective: 新物理参数只能被校准模型或嵌入式等价模型消费\nTested: test_pressure_model; test_hydro_sim_fb\nNot-tested: held-out CSV calibration is performed in Task 6"
```

## Task 3: Build Reproducible Open-Loop Identification and Calibration Tools

**Files:**
- Create: `scripts/ripple/analyze_open_loop.py`
- Create: `scripts/ripple/fit_physical_model.py`
- Create: `scripts/ripple/export_ripple_table.py`
- Create: `docs/ripple-analysis/`
- Modify: `tests/pressure_model_replay.c` to add the strict host-only
  `identified_params.kv` parser/loader after the file schema is defined

Task 3 is the first task allowed to load `identified_params.kv` into host replay, verify
its schema/version/calibration ID, and label a replay calibrated. Task 2's replay skeleton
must reject supplied KV files and remain explicitly uncalibrated.

- [ ] **Step 1: Implement standard-library CSV parsing and data-contract checks**

`analyze_open_loop.py` must read `open10203040-positive.csv` using `utf-8-sig` then
`gbk`, map columns by position to `ts, feedback_pressure_bar, feedback_rpm,
raw_pressure_bar, target_pressure_bar, set_rpm, torque_permille, angle_deg`, and reject
any timestamp delta other than exactly 1 ms. It must reject a nonzero target-pressure
column when labeling the file as open-loop.

- [ ] **Step 2: Implement angle-synchronous order metrics without external packages**

For each constant `set_rpm` segment, discard the first 2 seconds, use the final 5,000
samples for tail metrics, and solve the three-parameter normal equations for
`dc + c*cos(m*angle) + s*sin(m*angle)` at `m=13,26,39`. Report:

```text
amplitude = sqrt(c*c + s*s)
phase_for_A_sin(theta + phi) = atan2(c, s)
```

Write `docs/ripple-analysis/open_loop_summary.json` with sample counts, means, p-p,
order amplitude/phase, RPM standard deviation, torque mean, and validation-window IDs.
The script must produce the measured values recorded in
`tests/fixtures/open10203040_measurement_reference.h` within the fixture tolerances.

- [ ] **Step 3: Implement bounded deterministic physical-parameter fitting**

`fit_physical_model.py` must use only `csv`, `math`, `json`, `statistics`, `pathlib`, and
`hashlib` from the Python standard library. It runs a fixed coordinate-descent search over the
parameters observable in the
recorded speed/pressure/angle/torque windows:

```text
motor_natural_freq_hz [1.0, 200.0]
motor_damping         [0.2, 2.0]
motor_delay_s     [0.000, 0.050]
outlet_volume_m3  [1e-5, 5e-3]
gas_fraction      [1e-5, 2e-2]
pump_leak_c0     [1e-14, 1e-9]
pump_leak_speed   [1e-16, 1e-11]
cylinder_leak    [1e-14, 1e-9]
outlet_leak       [1e-14, 1e-9]
ripple13_peak    [0.00, 0.30]
ripple26_peak    [0.00, 0.15]
ripple39_peak    [0.00, 0.10]
torque_ripple13_peak [0.00, 0.30]
```

Use 20 coordinate rounds, halve a parameter step when neither direction improves the
weighted objective, and stop only when all step sizes are below 1% of their parameter
range. The objective weights held-out pressure RMSE, RPM RMSE, and 13/26 order amplitude
errors; it must not fit only the mean pressure. Emit a canonical `identified_params.json`
and strict `identified_params.kv` with the same `schema_version` and SHA-256
`calibration_id`.
Each contains parameter bounds, training windows, validation windows, seed, objective, and
pass/fail gates.

Create `docs/ripple-analysis/physical_parameter_manifest.json` for all remaining consumed
parameters that these windows cannot identify: atmospheric/suction pressure, chamber volume,
line inertance/resistance, bulk-modulus minimum/transition, volumetric/mechanical efficiency
bounds and coefficients, motor acceleration limit, relief curve/hysteresis, sensor
delay/quantization, rated torque/torque limit, and ripple phases. Every entry records a
value, unit, source (datasheet/nameplate/bench transient), acquisition window where
applicable, and the same calibration ID. Missing or unprovenanced entries make the fitter
emit `model not calibrated`.

- [ ] **Step 4: Export only a validated fixed RPM compensation table**

`export_ripple_table.py` reads `open_loop_summary.json` and
`identified_params.json`. It refuses to write a header unless the model gates are passed:
mean pressure error <= `max(5%, 5 bar)`, 13th amplitude error <= 20%, 13th phase error
<= 15 degrees, and 26th amplitude error <= 30%. The generated header is
`include/pressure_ripple_table.h` and contains speed breakpoints, 13th/26th amplitudes
in RPM, phases in radians, and the table count. It does not generate a 39th-order table
unless the total-p-p residual test in Task 7 requires it.

- [ ] **Step 5: Run the analysis and commit reproducible calibration artifacts**

Run:

```bash
python3 scripts/ripple/analyze_open_loop.py open10203040-positive.csv docs/ripple-analysis/open_loop_summary.json
python3 scripts/ripple/fit_physical_model.py open10203040-positive.csv docs/ripple-analysis/open_loop_summary.json docs/ripple-analysis/identified_params.json
python3 scripts/ripple/export_ripple_table.py docs/ripple-analysis/open_loop_summary.json docs/ripple-analysis/identified_params.json include/pressure_ripple_table.h
```

Expected: the first script succeeds with exact 1 ms timing, the second writes a held-out
fit report and matching KV/manifest artifacts, and the third either writes the table after
all model gates pass or exits
nonzero with `model not calibrated` and does not create a production table.

Commit only the scripts, reports, and a table that passed the gates:

```bash
git add scripts/ripple docs/ripple-analysis
git add include/pressure_ripple_table.h
git commit -m "增加开环阶次辨识和标定表生成" -m "用角度同步指标和有界物理参数拟合为固定13/26阶RPM补偿提供可复现数据。\n\nConstraint: 不引入嵌入式运行时Python依赖，模型未通过门禁不能生成生产表\nRejected: 整段固定频率FFT和未验证的在线LMS | 变速和次级通道会使结论失真\nConfidence: medium\nScope-risk: moderate\nDirective: 任何标定表必须保留训练/验证窗口和参数边界\nTested: analyze_open_loop.py; fit_physical_model.py; export_ripple_table.py\nNot-tested: controller metrics are covered in Task 7"
```

## Task 4: Refactor RBF-PID into a Tracking Positional RBF-PI Core

**Files:**
- Modify: `include/rbf_pid.h`
- Modify: `src/rbf_pid.c`
- Modify: `include/pressure_controller.h`
- Modify: `src/pressure_controller.c`
- Modify: `tests/rbf_pid_test.c`
- Modify: `tests/test_pressure_controller.c`

- [ ] **Step 1: Add failing tests for positional integral and tracking**

Extend `tests/rbf_pid_test.c` and `tests/test_pressure_controller.c` with these cases:

```c
RBF_PID_SetControlMode(&pid, RBF_PID_CONTROL_MODE_PI);
RBF_PID_SetContinuousGains(&pid, 0.05f, 0.005f);
RBF_PID_Update(&pid, 100.0f, 0.0f);
assert(fabsf(pid.integral_state - 0.0005f) < 1.0e-7f); /* 0.005*0.001*100 */

RBF_PID_TrackAppliedFlow(&pid, 0.0f);
RBF_PID_Update(&pid, 100.0f, 0.0f);
assert(pid.output_saturated);
assert(pid.integral_state <= pid.integral_limit + 1.0e-6f);

RBF_PID_SetControlMode(&pid, RBF_PID_CONTROL_MODE_PI);
RBF_PID_TrackAppliedFlow(&pid, 4.0f);
RBF_PID_Update(&pid, 50.0f, 50.0f);
assert(fabsf(pid.Output - 4.0f) < 1.0e-4f); /* bumpless hold */
```

The pre-change test must fail because the explicit integral state and continuous-gain
API do not yet exist.

- [ ] **Step 2: Add explicit PI state and compatibility APIs**

Append fields to `RBF_PID_Handle` without deleting existing fields:

```c
float integral_state;       /* L/min */
float integral_limit;       /* L/min */
float antiwindup_gain;      /* 1/s */
float raw_output;           /* L/min before final limits */
float applied_output;       /* L/min after final limits */
float feedforward_flow;     /* L/min, set by the outer pressure controller */
float learning_error;       /* bar, separate from control Error */
float max_delta_flow;       /* L/min per sample, <= 0 disables slew */
```

Add:

```c
void RBF_PID_SetContinuousGains(RBF_PID_Handle *pid, float kp, float ki);
void RBF_PID_SetAntiWindup(RBF_PID_Handle *pid, float kaw, float integral_limit);
void RBF_PID_SetOutputSlew(RBF_PID_Handle *pid, float max_delta_flow);
void RBF_PID_SetFeedforwardFlow(RBF_PID_Handle *pid, float feedforward_flow);
void RBF_PID_TrackAppliedFlow(RBF_PID_Handle *pid, float applied_flow);
```

Keep `RBF_PID_Update()` and `RBF_PID_SetControlMode()` names for recipe/API compatibility.
`SetContinuousGains()` stores Kp in L/min/bar and Ki in L/min/(bar*s); the PI update
always multiplies Ki by the measured `sampling_period`.

- [ ] **Step 3: Implement the positional PI calculation and anti-windup**

In PI mode, replace the current `u_prev + du` accumulation with:

```c
float control_error = setpoint - feedback;
float proportional = pid->KP * control_error;
float integral_candidate = pid->integral_state +
    pid->KI * pid->sampling_period * control_error;
float raw_output = pid->feedforward_flow + proportional + integral_candidate;
float limited_output = clamp_with_slew(raw_output,
                                       pid->applied_output,
                                       pid->output_min_flow,
                                       pid->output_max_flow,
                                       pid->max_delta_flow);

pid->integral_state = integral_candidate + pid->antiwindup_gain *
    pid->sampling_period * (limited_output - raw_output);
pid->Output = limited_output;
pid->du = limited_output - pid->applied_output;
pid->applied_output = limited_output;
```

Before each pressure-controller update, `HYD_PressureController_Execute()` calls
`RBF_PID_SetFeedforwardFlow(&state->rbfPid, input->feedforwardFlow)` so the configured
flow feedforward is consumed exactly once by the PI calculation.

Keep the existing PID calculation only for the compatibility/experimental
`RBF_PID_CONTROL_MODE_PID` path, but disable pressure-acceleration feedforward in PI mode.
The RBF model and gain adaptation use `u_prev` as the model input only after the network
is reinitialized or calibrated for that contract. `e_control` is never overwritten with
the filtered learning residual.

- [ ] **Step 4: Synchronize the outer pressure-controller state**

Add:

```c
void HYD_PressureController_TrackAppliedFlow(HYD_PressureControllerState *state,
                                             HYD_REAL appliedBaseFlow);
```

Call it after the pump converter and ripple limiter have produced the final command. The
function updates `rbfPid.applied_output`, `rbfPid.u_prev`, and `state->previousOutput`
without adding a second integral. `HYD_PressureController_RequestTracking()` uses the same
path for strategy changes and segment transitions.

Unify the RBF and classic negative-flow deadband thresholds in one helper. Preserve the
existing rule that low-pressure startup forbids negative flow; relief behavior remains
explicit and tested.

- [ ] **Step 5: Run focused controller tests and commit**

```bash
cmake --build --preset unixgcc --target rbf_pid_test test_pressure_controller
ctest --test-dir out/build/unixgcc -R '^(test_rbf_pid|test_pressure_controller)$' --output-on-failure
```

Expected: all existing tests pass, new PI tests pass, `adaptiveKd` is zero in RBF-PI,
and the positional integral remains bounded during output saturation.

```bash
git add include/rbf_pid.h src/rbf_pid.c include/pressure_controller.h src/pressure_controller.c tests/rbf_pid_test.c tests/test_pressure_controller.c
git commit -m "将RBF压力回路改为跟踪型位置式PI" -m "显式保存积分状态并按1ms采样周期解释Ki，同时将最终施加流量回传到anti-windup。\n\nConstraint: 保留RBF-PID和现有配方接口兼容性\nRejected: 继续用u_prev隐式承载积分 | 无法正确处理最终RPM限幅和无扰切换\nConfidence: high\nScope-risk: moderate\nDirective: PI模式不得重新启用自适应KD或压力加速度前馈\nTested: rbf_pid_test; test_pressure_controller\nNot-tested: physical ripple plant is covered in Task 7"
```

## Task 5: Implement the Fixed Angle-Synchronous Ripple Compensator

**Files:**
- Create: `include/ripple_compensator.h`
- Create: `src/ripple_compensator.c`
- Create: `tests/test_ripple_compensator.c`
- Modify: `include/motion_control.h`
- Modify: `src/motion_control.c`
- Modify: `CMakeLists.txt`
- Include: `include/pressure_ripple_table.h` generated by Task 3

- [ ] **Step 1: Write failing phase and safety tests**

Create tests for:

```c
HYD_PumpFeedback fb = {
    .rpm = 20.0,
    .angleDeg = 359.5,
    .torquePermille = 6242.0,
    .timestamp = 0.001,
    .validFlags = HYD_PUMP_FEEDBACK_VALID_RPM |
                  HYD_PUMP_FEEDBACK_VALID_ANGLE |
                  HYD_PUMP_FEEDBACK_VALID_TORQUE |
                  HYD_PUMP_FEEDBACK_VALID_TIMESTAMP
};
HYD_RippleComp_Execute(&state, &fb, 400.0f, &out);

fb.angleDeg = 0.5;
fb.timestamp = 0.002;
HYD_RippleComp_Execute(&state, &fb, 400.0f, &out);
assert(out.phaseValid);
assert(out.deltaRpm == out.deltaRpm); /* finite */

fb.validFlags &= ~HYD_PUMP_FEEDBACK_VALID_ANGLE;
HYD_RippleComp_Execute(&state, &fb, 400.0f, &out);
assert(!out.phaseValid);
assert(out.deltaRpm == 0.0f);
```

Add tests for reverse wrap, zero RPM with zero angle increment, timestamp rollback,
angle jumps beyond the configured physical limit, correct sign from a calibrated table,
13/26 table interpolation, and final RPM compensation limits.

- [ ] **Step 2: Define the fixed-table API**

Use these C99 types. `include/ripple_compensator.h` includes `<stddef.h>` for `size_t`
and `<stdbool.h>` for `bool`, in addition to the project common-types header:

```c
typedef struct {
    float rpm;
    float amplitude13_rpm;
    float phase13_rad;
    float amplitude26_rpm;
    float phase26_rad;
} HYD_RippleTablePoint;

typedef struct {
    double toothPhaseTurns; /* always wrapped to [0, 1) */
    float previousAngleDeg;
    const HYD_RippleTablePoint *table;
    size_t tableCount;
    float pumpSpeedLimitRpm;
    HYD_TIME previousTimestamp;
    float previousRpm;
    bool initialized;
    bool phaseValid;
} HYD_RippleCompState;

typedef struct {
    float deltaRpm;
    bool phaseValid;
    bool compensationActive;
    bool speedLimitActive;
} HYD_RippleCompOutput;

void HYD_RippleComp_Init(HYD_RippleCompState *state,
                         const HYD_RippleTablePoint *table,
                         size_t tableCount,
                         float pumpSpeedLimitRpm);
void HYD_RippleComp_Execute(HYD_RippleCompState *state,
                            const HYD_PumpFeedback *feedback,
                            float baseRpm,
                            HYD_RippleCompOutput *output);
```

The state uses `double` only for phase integration and wraps `toothPhaseTurns` to
`[0, 1)` after every update; the public angle remains modulo-360 degrees. The implementation folds angle deltas into [-180, 180), rejects
increments above the physical RPM-derived limit, and treats zero delta as valid at zero
RPM. Use a bounded lookup table for sine/cosine and no adaptive coefficient state.

- [ ] **Step 3: Implement calibrated compensation with explicit units and sign**

For valid feedback, calculate the table waveform with the bounded sine/cosine lookup
already used by the implementation:

```c
float delta_rpm = amplitude13 * lookup_sin(tooth_phase + phase13) +
                  amplitude26 * lookup_sin(2.0f * tooth_phase + phase26);
delta_rpm = clampf(-0.30f * pumpSpeedLimitRpm,
                  delta_rpm,
                  0.30f * pumpSpeedLimitRpm);
```

Use the table's measured sign; do not prepend a generic negative sign. Disable output
when the pump is stopped, speed is outside the calibrated range, phase is invalid, or
the 13th order is not observable at the configured 1 ms period.

- [ ] **Step 4: Apply compensation at the RPM boundary and track the base output**

Store `HYD_RippleCompState` in `HYD_MotionControlFB`. In
`HYD_ExecuteActiveSegmentControl()`, obtain `feedbackIngress` from the producer route
for the current cycle, validate it, and pass it directly to the compensator. Do not
retain it in `AXIS_REF`, diagnostics, or `HYD_PressureControllerInput`:

```c
base_rpm = pumpOutput->pumpSpeed;
HYD_RippleComp_Execute(&fb->_rippleComp,
                       feedbackIngress,
                       (float)base_rpm,
                       &rippleOutput);
final_rpm = clamp_slew(base_rpm + rippleOutput.deltaRpm,
                       fb->_previousAppliedPumpSpeed,
                       pumpInput.pumpSpeedLimit);
pumpOutput->pumpSpeed = final_rpm;
applied_base_flow = (final_rpm - rippleOutput.deltaRpm) /
                    pumpInput.flowToPumpSpeedGain;
HYD_PressureController_TrackAppliedFlow(&fb->_pressureController,
                                        applied_base_flow);
```

The exact local variable names may follow the existing function, but the sequence must
remain converter -> ripple -> final limiter -> PI tracking. Reset the compensator on
segment changes, invalid feedback, and controller clear; preserve the base PI path when
the phase gate is closed.

- [ ] **Step 5: Run tests and commit**

```bash
cmake --preset unixgcc
cmake --build --preset unixgcc --target test_ripple_compensator test_pressure_controller
ctest --test-dir out/build/unixgcc -R '^(test_ripple_compensator|test_pressure_controller)$' --output-on-failure
```

```bash
git add include/ripple_compensator.h src/ripple_compensator.c include/motion_control.h src/motion_control.c tests/test_ripple_compensator.c CMakeLists.txt
git commit -m "加入角度同步13阶和26阶RPM补偿" -m "在泵转换器之后注入经过标定的齿轮泵阶次补偿，并将最终基础流量回传到PI跟踪状态。\n\nConstraint: 1ms STM32H7路径无在线LMS、无动态内存、角度失效必须旁路\nRejected: 未辨识次级通道的直接FxLMS | 符号、单位和饱和行为无法保证\nConfidence: medium\nScope-risk: moderate\nDirective: 补偿符号只能来自标定表，不得恢复通用u_pi-k_ff*u_c假设\nTested: test_ripple_compensator; test_pressure_controller\nNot-tested: reduction metrics require Task 7 physical closed-loop test"
```

## Task 6: Complete Physical-Model Calibration and Held-Out Validation

**Files:**
- Modify: `scripts/ripple/fit_physical_model.py`
- Modify: `scripts/ripple/analyze_open_loop.py`
- Create: `scripts/ripple/validate_physical_model.py`
- Modify: `docs/ripple-analysis/identified_params.json`
- Create: `docs/ripple-analysis/model_validation.json`

- [ ] **Step 1: Replay all four measured speed windows through the C model**

Add a deterministic replay mode to `validate_physical_model.py` that first verifies the
matching `identified_params.json`, `identified_params.kv`, and physical-parameter manifest
share one `schema_version` and `calibration_id`. It then invokes the built
`pressure_model_replay` executable, for example
`./out/build/unixgcc/pressure_model_replay physical 20 20000 docs/ripple-analysis/identified_params.kv`,
and consumes its CSV header/output. Use the measured command-RPM segments, not the pressure
column as a setpoint. Compare feedback RPM, mean pressure, p-p, and 13/26/39
angle-synchronous amplitudes.

- [ ] **Step 2: Separate training and validation windows**

Use the first half of each stable speed segment for fitting and the final half for
validation, while preserving a separate 2-second transition window for speed-loop
identification. Record the exact timestamp ranges and sample counts in
`model_validation.json`.

- [ ] **Step 3: Enforce the model gates**

The script exits with status 0 only when all held-out gates pass:

```text
mean pressure error <= max(5%, 5 bar)
13th amplitude error <= 20%
13th phase error <= 15 degrees
26th amplitude error <= 30%
speed peak/settling/steady-state errors within the recorded fit limits
```

When a gate fails, print `model not calibrated`, write the error report, and return a
nonzero status. Do not export or commit `include/pressure_ripple_table.h` from a failed
fit.

- [ ] **Step 4: Commit only a passing calibration**

```bash
python3 scripts/ripple/validate_physical_model.py open10203040-positive.csv docs/ripple-analysis/identified_params.json docs/ripple-analysis/model_validation.json
git add scripts/ripple docs/ripple-analysis
git commit -m "通过物理压力模型独立窗口校验" -m "确认速度环、平均压力、泄漏压缩性和13/26阶纹波参数可复现开环数据。\n\nConstraint: 未通过留出窗口不能进入闭环控制结论\nRejected: 只用训练窗口或旧一阶模型作验收 | 会掩盖模型失配\nConfidence: medium\nScope-risk: moderate\nDirective: 任何参数修改都必须重新生成留出窗口报告\nTested: validate_physical_model.py\nNot-tested: closed-loop overshoot is measured in Task 7"
```

## Task 7: Add Deterministic Physical Closed-Loop Comparison Tests

**Files:**
- Create: `tests/test_pressure_closed_loop_ripple.c`
- Modify: `tests/test_pressure_model.c`
- Modify: `tests/test_pressure_controller.c`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Write the comparison harness before enabling assertions**

Create a harness that runs the same `PressureModelParams`, seed, initial tooth phase,
target-pressure trajectory, and load trajectory for:

```text
PI baseline
legacy RBF-PID baseline
tracking positional RBF-PI without ripple compensation
tracking positional RBF-PI with 13th compensation
tracking positional RBF-PI with 13th + 26th compensation
```

The loop is one control step per 1 ms. At each step it passes measured pressure and the
unified feedback packet to the pressure controller, converts flow to RPM, applies the
compensator, tracks the final base flow, advances the physical model, and records
pressure, RPM, torque, angle, saturation, and execution counters.

- [ ] **Step 2: Implement identical metrics for every controller**

For each nonzero target profile, calculate:

```text
overshoot = (peak_pressure - target_pressure) / target_pressure
13th_amp = angle-synchronous amplitude in the final hold window
total_pp = max(pressure) - min(pressure) in the same final hold window
settling_time = first time after which pressure remains within the configured band
IAE = sum(abs(target_pressure - measured_pressure) * 0.001)
```

Use at least three production-representative pressure levels and multiple initial tooth
phases. The default deterministic fixture uses 50, 100, and 200 bar only when those
levels are within the configured machine pressure range; otherwise the fixture loads the
three nearest approved levels from `identified_params.json`.

- [ ] **Step 3: Enforce the approved controller gates**

Assert for the selected production candidate:

```c
assert(metrics.overshoot_ratio <= 0.05f);
assert(metrics.ripple13_amplitude <= 0.50f * baseline.ripple13_amplitude);
assert(metrics.total_pressure_pp <= 0.65f * baseline.total_pressure_pp);
assert(metrics.saturation_ratio <= baseline.saturation_ratio + 0.05f);
```

The test must fail with a report containing all controller rows if any assertion fails.
Do not weaken assertions to make an uncalibrated model pass.

- [ ] **Step 4: Add CTest registration and run the comparison**

```bash
cmake --preset unixgcc
cmake --build --preset unixgcc --target test_pressure_closed_loop_ripple test_pressure_model test_pressure_controller rbf_pid_test
ctest --test-dir out/build/unixgcc -R '^(test_pressure_closed_loop_ripple|test_pressure_model|test_pressure_controller|test_rbf_pid)$' --output-on-failure
```

Expected: the physical model and controller comparison produce a metrics table; no
production conclusion is recorded if Task 6's model gate is not present.

- [ ] **Step 5: Commit the model-in-the-loop evidence**

```bash
git add tests/test_pressure_closed_loop_ripple.c tests/test_pressure_model.c tests/test_pressure_controller.c CMakeLists.txt
git commit -m "增加物理模型闭环压力指标测试" -m "在同一流体模型、相位和负载下比较PI、RBF-PID、RBF-PI及13/26阶补偿，直接验证超调和纹波门槛。\n\nConstraint: 控制器结论必须建立在已通过留出窗口的模型上\nRejected: 继续使用理想一阶对象作为唯一闭环测试 | 无法覆盖泵纹波和泄漏非线性\nConfidence: medium\nScope-risk: moderate\nDirective: 输出完整指标表，不得只保留通过/失败布尔值\nTested: test_pressure_closed_loop_ripple; test_pressure_model; test_pressure_controller; rbf_pid_test\nNot-tested: machine closed-loop data remains required before merge"
```

## Task 8: Measure STM32H7 Runtime and Freeze the Safety Fallbacks

**Files:**
- Modify: `tests/benchmark_performance.c`
- Create: `tests/test_pressure_resource_budget.c`
- Create: `docs/ripple-analysis/stm32h7-runtime-budget.md`
- Modify: `include/ripple_compensator.h`
- Modify: `src/ripple_compensator.c`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Add host-visible state-size and boundedness checks**

`test_pressure_resource_budget.c` reports `sizeof(HYD_PumpFeedback)`,
`sizeof(HYD_RippleCompState)`, `sizeof(RBF_PID_Handle)`, and the total added state. It
also runs one million fixed-step compensator calls and asserts that every output is finite
and within the configured RPM limit.

- [ ] **Step 2: Add a benchmark entry point for the production path**

Extend `benchmark_performance.c` with a pressure-loop benchmark that executes the PI,
RBF update, phase tracker, fixed-table compensation, and final tracking path for 100,000
cycles. Report cycles per call and host wall time; this is a regression signal, not the
STM32 acceptance result.

- [ ] **Step 3: Measure the target with DWT and record the result**

In the STM32H7 firmware integration, wrap the existing 1 ms control task with the board's
DWT cycle counter. Record clock frequency, worst cycles, average cycles, stack high-water
mark, static RAM delta, and flash delta in `docs/ripple-analysis/stm32h7-runtime-budget.md`.
The release gate is worst-case pressure-task time below 20% of 1 ms and no task overrun.
Do not add a fake host-only DWT API to the production controller.

- [ ] **Step 4: Verify all safety fallbacks**

Run tests for nonfinite pressure/RPM/torque, timestamp rollback, invalid angle, stopped
pump, same-direction saturation, negative-flow startup, and compensation table range
misses. Expected behavior is base PI/RBF-PI active, `deltaRpm == 0`, RBF learning frozen,
and no nonfinite output.

- [ ] **Step 5: Commit the resource and fallback evidence**

```bash
cmake --build --preset unixgcc --target benchmark_performance test_pressure_resource_budget
./out/build/unixgcc/benchmark_performance
ctest --test-dir out/build/unixgcc -R '^(test_pressure_resource_budget|test_pressure_closed_loop_ripple)$' --output-on-failure
git add tests/benchmark_performance.c tests/test_pressure_resource_budget.c docs/ripple-analysis/stm32h7-runtime-budget.md include/ripple_compensator.h src/ripple_compensator.c CMakeLists.txt
git commit -m "锁定压力控制资源和故障旁路" -m "验证RBF-PI与同步补偿的有界输出，并记录STM32H7目标板最坏执行时间和内存证据。\n\nConstraint: 1ms实时任务必须先过资源门禁\nRejected: 以桌面运行时间替代目标板测量 | Cortex-M7调度和编译选项不同\nConfidence: high\nScope-risk: narrow\nDirective: 资源报告缺失时不得进入上机验证\nTested: benchmark_performance; test_pressure_resource_budget; test_pressure_closed_loop_ripple\nNot-tested: exact target numbers depend on the STM32H7 firmware build"
```

## Task 9: Full Verification, Machine Trial, and Merge Gate

**Files:**
- Create: `docs/ripple-analysis/closed_loop_machine_validation.md`
- Modify: `docs/superpowers/specs/2026-08-06-pressure-ripple-suppression-design.md` only for measured evidence links

- [ ] **Step 1: Run the full host verification on the feature branch**

```bash
cmake --preset unixgcc
cmake --build --preset unixgcc
ctest --test-dir out/build/unixgcc --output-on-failure
python3 tests/test_interface_layout_consistency.py
git diff --check
```

Expected: all CTest tests, the interface layout check, and whitespace validation pass.

- [ ] **Step 2: Build the production STM32H7 image**

Use the repository's target firmware build with `HYD_PRESSURE_CONTROLLER_RBF_PI` selected
for the validated pressure recipe. Confirm that the compiler map contains one ripple
state per motion-control instance, no dynamic allocation, and no unresolved feedback
symbols. Save the target build identifier and map summary in
`docs/ripple-analysis/stm32h7-runtime-budget.md`.

- [ ] **Step 3: Run controlled machine tests before normal production use**

For each approved pressure level and speed range:

1. Run the existing open-loop speed sweep to confirm feedback angle, torque, and pressure
   channels are valid.
2. Run a nonzero closed-loop pressure rise with PI baseline.
3. Run tracking RBF-PI without ripple compensation.
4. Run tracking RBF-PI with fixed 13th/26th compensation.
5. Keep the same target pressure, load, initial angle, sample interval, and evaluation
   windows for every comparison.

Record raw pressure, filtered pressure, setpoint, requested/applied RPM, actual RPM,
angle, torque, compensation delta, saturation flags, and timestamps at 1 ms. Evaluate
overshoot, 13th amplitude, total p-p, settling time, IAE, saturation, and CPU load.

- [ ] **Step 4: Write the machine validation decision**

`docs/ripple-analysis/closed_loop_machine_validation.md` must state PASS or FAIL for all
three approved thresholds and include the raw-data file names, calibration-table version,
model-validation report, target firmware identifier, and resource report. A FAIL leaves
the branch unmerged and identifies the next bounded parameter/model task.

- [ ] **Step 5: Merge only after the hardware gate passes**

After the validation document states PASS and the user approves the release:

```bash
git status --short
git switch master
git merge --no-ff feature/pressure-ripple-suppression
git log --oneline -3
```

Expected: `master` contains the feature branch commits and the validation evidence. If
any gate is missing or failed, do not run the merge command.

## Self-Review Checklist

- [ ] Every spec section has at least one implementation or validation task.
- [ ] The branch is created before the first code edit.
- [ ] The plan contains no Windows-only paths or direct `.git/refs` writes.
- [ ] `HYD_PumpFeedback` fields all have a producer, first consumer, and observable effect.
- [ ] No temperature, Goertzel, online FxLMS, or 39th-order runtime state is added before
  its data/validation gate exists.
- [ ] Physical model and controller tests use held-out windows and deterministic seeds.
- [ ] Ki units include the 1 ms sampling period exactly once.
- [ ] Final RPM limiting is fed back to the PI/RBF tracking state.
- [ ] Model failure blocks table export and controller claims.
- [ ] STM32H7 timing/memory evidence and closed-loop machine data are required before merge.
