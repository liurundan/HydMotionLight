# Pressure Control Review Fixes Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Complete the pressure-controller runtime path for `HYD_PRESSURE_CONTROLLER_PI_RBF` and apply the bounded correctness fixes identified by the nonlinear pressure-loop review without changing the position-form PI architecture or unrelated motion behavior.

**Architecture:** Keep PI, PI+FF, RBF-PI, RBF-PID, and PI-RBF as runtime enum-selected strategies. PI-RBF remains a position-form PI output whose `Kp/Ki` are bounded proposals from the RBF supervisor; the outer loop owns saturation, feed-forward, anti-windup, and the actual command fed back to RBF. Validation and IEC surfaces use the same enum values, with the new value appended for numeric compatibility.

**Tech Stack:** C99, CMake/CTest, MinGW or Unix GCC, existing pressure model and standalone regression tests, matiec-generated IEC headers.

---

### Task 1: Close the PI-RBF selection and IEC contract

**Files:**
- Modify: `src/motion_control.c:4012-4035`
- Modify: `src/recipe_validator.c:41-57,244-247`
- Modify: `tests/plcdemo/POUS.h:664-670` (generated IEC enum mirror)
- Modify: `docs/液压库参数读写接口设计文档.md:96`
- Test: `tests/test_parameter_access.c`, `tests/test_parameter_iec.c`, and the existing recipe-validator test target

- [ ] **Step 1: Add failing regression assertions** for writing value `HYD_PRESSURE_CONTROLLER_PI_RBF`, reading it back, validating a PI-RBF pressure segment, and exposing it through `PRESSURECONTROLLERAPPLIED`.
- [ ] **Step 2: Run the focused tests and confirm failure** because parameter and recipe validation currently stop at `HYD_PRESSURE_CONTROLLER_RBF_PI`.
- [ ] **Step 3: Add `HYD_PRESSURE_CONTROLLER_PI_RBF` to parameter and recipe validation** while preserving the existing enum numbering and active-segment latching behavior.
- [ ] **Step 4: Add the missing generated IEC enum item and correct the parameter mapping documentation** to list `NONE=0, P=1, PI=2, PID=3, RBF_PID=4, RBF_PI=5, PI_RBF=6`.
- [ ] **Step 5: Run the focused parameter, IEC, recipe, and layout tests.**

### Task 2: Make PI-RBF learning use the actual applied command

**Files:**
- Modify: `src/pressure_controller.c:536-610`
- Modify: `include/pressure_controller.h` only if an appended state field is required
- Test: `tests/test_pressure_ripple_comp.c` or a focused pressure-controller test

- [ ] **Step 1: Add a failing test** that constrains the outer PI output, executes PI-RBF, and asserts RBF `Output`, `u_prev`, and `n_out` track the clamped command rather than the discarded virtual RBF command.
- [ ] **Step 2: Run the focused pressure test and confirm the mismatch.**
- [ ] **Step 3: Synchronize PI-RBF RBF state after the final PI clamp and anti-windup path, set saturation from both internal and segment limits, and freeze adaptation when the error drives the active limit.**
- [ ] **Step 4: Run pressure-controller and ripple-compensation tests.**

### Task 3: Harden timing, RBF configuration, and low-pressure behavior

**Files:**
- Modify: `src/pressure_controller.c`
- Modify: `src/rbf_pid.c` only where timing normalization or non-finite guards are required
- Test: `tests/test_rbf_pid_hil.c`, `tests/test_pressure_controller.c`, and new focused assertions in `tests/test_pressure_ripple_comp.c`

- [ ] **Step 1: Add failing tests** for non-finite/rollback/long-cycle `dt`, reversed or non-finite RBF windows, external output saturation, and negative-flow suppression near a low-pressure setpoint.
- [ ] **Step 2: Run the focused tests and record the expected failures.**
- [ ] **Step 3: Clamp valid `dt`, reject invalid timing/configuration, normalize/sort RBF windows before runtime clamping, use configured segment limits for saturation, and keep negative relief disabled unless the segment explicitly permits it and over-pressure is present.**
- [ ] **Step 4: Run all pressure and RBF tests.**

### Task 4: Harden ripple-compensation state transitions

**Files:**
- Modify: `src/pressure_ripple_comp.c`
- Modify: `include/pressure_ripple_comp.h` if an encoder-valid input is needed
- Modify: `src/motion_control.c` only for the existing pressure-input wiring
- Test: `tests/test_pressure_ripple_comp.c`

- [ ] **Step 1: Add failing tests** for invalid encoder phase, speed-phase fallback, invalid `systemGain`, stale gain clearing, phase jumps, convergence gating, and bounded ripple output.
- [ ] **Step 2: Run the focused ripple tests and confirm failures.**
- [ ] **Step 3: Implement explicit phase validity/fallback, clear stale gain on invalid transitions, gate learning until convergence, and enforce amplitude/phase-rate/total-command bounds.**
- [ ] **Step 4: Run ripple, pressure-controller, and motion-interface regression tests.**

### Task 5: Nonlinear simulation and final verification

**Files:**
- Modify: `tests/sim_pressure_control.c` only if the existing harness lacks a required scenario
- Modify: `CMakeLists.txt` only if a new focused test target must be registered
- Update: `docs/superpowers/specs/2026-08-03-pressure-control-review-design.md` with final evidence

- [ ] **Step 1: Run the existing baseline and nonlinear simulations** at 20/100/180 bar, +/-30% gain mismatch, load steps, speed changes, encoder loss, segment transitions, low-pressure mold protection, high-pressure lock, holding, plasticizing/back pressure, and ejector hold.
- [ ] **Step 2: Verify acceptance metrics:** rise <=100 ms, settle inside +/-2% <=300 ms, overshoot <=5%, and real/filtered process ripple <=1% at the calibrated target where sensor noise does not mathematically dominate.
- [ ] **Step 3: Run the full CTest suite and static checks.**
- [ ] **Step 4: Record residual risks** where the physical plant or sensor noise prevents the hard ripple criterion from being claimed.

