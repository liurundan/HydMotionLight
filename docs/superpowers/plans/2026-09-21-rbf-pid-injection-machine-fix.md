# RBF-PID 注塑机应用修复计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在固定 1 ms 控制周期下，把 RBF 增量 PID 从“网络输出被错误钳位、增益窗口失配、近目标过早冻结”的状态修到可在注塑机压力环上安全验证的工程版本。

**Architecture:** 保留现有增量 PID 与 RBF 一步预测器结构，不更换控制器接口。用离散对象灵敏度 `b = K·(1-exp(-Ts/τ))` 替代稳态增益 K 作为 Jacobian 量纲基准；默认 PID 增益窗口切换到实测稳定区，并把学习冻结阈值改成与目标压力相关的相对阈值。验收测试直接按峰峰值纹波和 `tr <= 800 ms` 做硬判定。

**Tech Stack:** C99, CMake, MinGW-w64 GCC 12.2, CTest, 1 ms 压力闭环仿真。

**Spec:** `docs/RBF-PID压力闭环算法工程实用性评估-2026-09-17.md` 与本轮外部评审结论。

## Global Constraints

- 控制周期固定为 1 ms；不引入运行时可变采样周期，也不改变现有增量 PID 的 `Δu` 语义。
- RBF 网络第一输入继续表示上一拍 `Δu(k-1)`，Jacobian 的物理单位固定为 `bar/(L/min)` 的一步离散灵敏度。
- 默认增益必须落在当前注塑机实测稳定区：`KP <= 0.31`、`KI <= 0.0029`；PID 模式的默认 D 窗口保持保守。
- 生产验收必须硬检查：超调 `<= 5%`、稳态峰峰值纹波 `<= 1 bar`、90% 上升时间 `<= 800 ms`。
- 不新增第三方依赖；保持公开结构体的既有字段顺序，仅在尾部追加状态字段（如确有需要）。

---

### Task 1: 先写回归基线与失败验收

**Files:**
- Modify: `tests/rbf_pid_test.c`
- Modify: `tests/test_pressure_target_verification.c`
- Modify: `tests/test_production_default_acceptance.c`

**Interfaces:**
- Consumes: 现有 `RBF_PID_Handle`、压力仿真输出和 1 ms `SIM_DT`。
- Produces: 能暴露 Jacobian 量纲错误、过早冻结、峰峰值/上升时间验收缺口的失败测试。

- [ ] **Step 1: Add a Jacobian unit regression test**

  Use `K=200 bar/(L/min)`, `τ=1.862 s`, `Ts=0.001 s`, and assert the reported Jacobian stays in a discrete-sensitivity window around `b≈0.107 bar/(L/min)` instead of `[0.2K,5K]`.

- [ ] **Step 2: Add a near-target learning-window regression test**

  Drive a 150 bar target through an 8 bar error and assert adaptation is not frozen solely because `|e|<10 bar`; then drive a 1 bar error and assert the steady-state freeze still restrains drift.

- [ ] **Step 3: Replace RMS-only ripple acceptance with peak-to-peak**

  Track `steady_min` and `steady_max` in the final 2 s window, expose `ripple_p2p_bar = steady_max - steady_min`, keep RMS as diagnostic only, and make `ripple_p2p_bar <= 1.0f` the hard assertion.

- [ ] **Step 4: Make rise time a hard acceptance condition**

  Define `TARGET_TR_MS = 800.0f` and assert `p90 > 0 && p90 <= TARGET_TR_MS` in the production-default test and the pressure-target verification path.

- [ ] **Step 5: Run the focused tests and record the expected failures**

  Run the MinGW-built test binaries before algorithm edits; the new tests must fail for the current implementation, proving the regressions are sensitive to the reported defects.

### Task 2: Correct the RBF Jacobian to discrete sensitivity units

**Files:**
- Modify: `include/rbf_pid.h`
- Modify: `src/rbf_pid.c`
- Modify: `tests/rbf_pid_test.c`

**Interfaces:**
- Consumes: `K`, fixed `sampling_period=0.001 s`, optional calibrated plant time constant.
- Produces: `RBF_PID_SetProcessTimeConstant()` and a bounded positive Jacobian used by adaptive gain updates.

- [ ] **Step 1: Append the calibrated process time constant field and setter**

  Add `process_time_constant_s` at the tail of `RBF_PID_Handle`, initialize it to the safe default `1.0 s`, and expose `RBF_PID_SetProcessTimeConstant(pid, tau_s)` with finite-positive validation.

- [ ] **Step 2: Replace `[0.2K,5K]` with a discrete-sensitivity window**

  Compute `b_nom = K * (1 - exp(-Ts / tau))`, then clamp the magnitude to `max(0.25*b_nom, 0.02*Ts*K)` and `min(4.0*b_nom, 5.0*Ts*K)` with finite fallbacks. Do not use the steady-state K directly as a Jacobian bound.

- [ ] **Step 3: Enforce the known positive hydraulic sign**

  If the network gradient is non-finite or non-positive, publish `Jacobian=0` for that sample and skip adaptive PID gain updates for that sample; never let a negative noisy gradient reverse the tuning direction.

- [ ] **Step 4: Add a direct unit test for the 1 ms / τ=1.862 s case**

  Assert the Jacobian is not pinned at `0.2K`, remains finite and positive during informative excitation, and leaves the old 40 bar/(L/min) boundary by at least one order of magnitude.

### Task 3: Move default adaptive gains into the measured stable window

**Files:**
- Modify: `include/rbf_pid.h`
- Modify: `src/rbf_pid.c`
- Modify: `tests/rbf_pid_test.c`

**Interfaces:**
- Consumes: existing `RBF_PID_SetParamLimits()` override path.
- Produces: safe library defaults that remain overrideable per segment.

- [ ] **Step 1: Set conservative defaults for the injection-machine plant**

  Use `KP=[0.05,0.15]`, `KI=[0.0003,0.0025]`, and `KD=[0.0,0.01]`; initialize at the lower bound and keep all existing clamp/sort behavior.

- [ ] **Step 2: Preserve explicit per-segment overrides**

  Keep `RBF_PID_SetParamLimits()` authoritative after initialization; do not rewrite caller-supplied windows during `SetGainCompensation()` or reset.

- [ ] **Step 3: Verify the defaults never enter the old unstable window**

  Extend the default-window test to assert `max_KP <= 0.31`, `max_KI <= 0.0029`, `min_KD >= 0`, and that the adaptive values remain inside the configured window during a 1 ms closed-loop drive.

### Task 4: Parameterize the learning freeze zone by target pressure

**Files:**
- Modify: `src/rbf_pid.c`
- Modify: `tests/rbf_pid_test.c`

**Interfaces:**
- Consumes: `P_set`, `e`, and `Δe` already available in `rbf_pid_step_rbf_nn()`.
- Produces: target-relative freeze thresholds with no new runtime dependency.

- [ ] **Step 1: Replace the fixed 10 bar threshold**

  Use `error_threshold = max(0.5 bar, 0.02*abs(P_set))` and `delta_error_threshold = max(0.5 bar, 0.01*abs(P_set))`.

- [ ] **Step 2: Retain noise protection near the final target**

  Keep learning frozen once both thresholds are satisfied; do not re-enable continuous steady-state learning that would learn pump ripple as plant dynamics.

- [ ] **Step 3: Verify both sides of the threshold**

  Assert adaptation is active at 8 bar error for a 150 bar target and restrained at 1 bar error.

### Task 5: Integrate calibration and verify MinGW delivery

**Files:**
- Modify: `src/pressure_controller.c`
- Modify: `tests/test_rbf_pid_contract.c`
- Modify: `CMakeLists.txt` only if a new test target is needed.

**Interfaces:**
- Consumes: existing `HYD_PressureControllerInput.plantTauS` and `systemGain` calibration values.
- Produces: RBF path receives the same calibrated τ used by FF_PI without changing the 1 ms loop period.

- [ ] **Step 1: Pass plant τ into the RBF handle**

  Resolve `plantTauS` for all pressure strategies and call `RBF_PID_SetProcessTimeConstant()` during RBF configuration; leave zero/invalid τ on the 1.0 s safe default.

- [ ] **Step 2: Add a contract assertion for calibration propagation**

  Configure τ=1.862 s and K=210.55 in the contract test, execute one informative RBF update, and assert the Jacobian is in the discrete window implied by those values.

- [ ] **Step 3: Configure, build, and test with the repository MinGW preset**

  Run:

  ```powershell
  cmake --preset mingw-w64
  cmake --build --preset mingw-w64 --target rbf_pid_test test_rbf_pid_contract test_pressure_target_verification test_production_default_acceptance
  ctest --test-dir out/build/mingw-w64 -R "rbf_pid|pressure_target_verification|production_default_acceptance" --output-on-failure
  ```

- [ ] **Step 4: Run the complete CTest suite and classify unrelated failures**

  Do not waive a changed RBF or pressure-acceptance failure; report only pre-existing failures that are outside the touched paths.

### Stop condition

Stop this iteration only when the focused MinGW build succeeds, the RBF unit/contract tests pass, the pressure acceptance tests enforce (and meet) the three user metrics, and no touched-path test reports `#REF`, NaN/Inf, or a stale RMS-only pass condition. FF_PI low-K robustness remains a separate follow-up item unless the focused acceptance run demonstrates it is required for the RBF path.
