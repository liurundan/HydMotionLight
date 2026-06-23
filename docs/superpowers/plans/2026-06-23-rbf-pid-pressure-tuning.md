# RBF-PID Pressure Tuning Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Tune the existing RBF-PID pressure loop so the pure first-order pressure object meets `<= 5%` overshoot and `<= 1%` steady-state error at `50 / 100 / 200 bar`, while keeping the pressure-model boundary explicit and preserving bounded adaptive behavior.

**Architecture:** Keep `tests/test_pressure_controller.c` as the primary acceptance surface and tighten it first so the failure is measurable. Then align unresolved RBF defaults in `src/pressure_controller.c` with the conservative library profile, reduce aggressive adaptation in `src/rbf_pid.c`, and finish by encoding the tuned segment-level `pressureRbfConfig` profile that the first-order plant tests will use as the reference simulation recipe. `tests/test_pressure_model.c` stays as the guardrail for the first-order plant boundary: static `flow -> rpm`, first-order pressure branch only, no motor dynamic validation in this task.

**Tech Stack:** C99, HydroMotionLib, HydroSimLib, `tests/test_pressure_controller.c`, `tests/test_pressure_model.c`, `tests/rbf_pid_test.c`, CMake/CTest, `libm`

---

## File Structure

| File | Action | Responsibility |
| --- | --- | --- |
| `tests/test_pressure_controller.c` | Modify | Tighten the pure first-order plant acceptance contract to `50 / 100 / 200 bar`, collect last-`1s` error metrics, and encode the tuned RBF segment profile used for simulation verification |
| `tests/test_pressure_model.c` | Modify | Lock the first-order plant boundary so the tuning task stays on `K = 5.4`, `tau = 1.0`, `delay = 0`, with no motor dynamic branch in scope |
| `src/pressure_controller.c` | Modify | Align unresolved default RBF learning rates with the conservative bounded profile used by the controller library |
| `tests/rbf_pid_test.c` | Modify | Tighten the soft-cap and near-target adaptation tests so `src/rbf_pid.c` changes are forced by unit tests before plant-level validation |
| `src/rbf_pid.c` | Modify | Reduce aggressive adaptive behavior, tighten the system-gain soft cap, and make near-target learning more conservative |
| `src/sim/PressureModel.c` | Preserve unless Task 1 fails | The first-order defaults already match the requested object; only touch this file if the boundary-lock test reveals drift |

## Boundary Rules

- Do not replace the RBF-PID controller with a fixed PID controller.
- Do not add a second plant model or a second `flow -> rpm` path.
- Do not pull the motor first-order dynamic link back into the acceptance test for this task.
- Do not retune the physical pressure branch in `src/sim/PressureModel.c`.
- Do not broaden this work into HIL or open-loop model-fit redesign.
- Keep the final tuned simulation recipe encoded through existing `pressureRbfConfig` and `systemGain` fields.

### Task 1: Lock the first-order plant acceptance contract before tuning

**Files:**
- Modify: `tests/test_pressure_controller.c`
- Modify: `tests/test_pressure_model.c`

- [ ] **Step 1: Rewrite the first-order boundary test and add explicit step-response metrics**

In `tests/test_pressure_model.c`, replace `test_init_params_default_to_physical_mode()` with:

```c
static int test_init_params_default_to_first_order_tuning_contract(void) {
    PressureModelParams params;
    PressureModelState state;
    PressureModelOutput out;

    memset(&out, 0, sizeof(out));
    PressureModel_InitParams(&params);
    PressureModel_Reset(&state, 0x51515151u);

    ASSERT_TRUE(params.model_type == PRESSURE_MODEL_TYPE_FIRST_ORDER);
    ASSERT_NEAR(params.first_order_k_bar_per_rpm, 5.4f, 1e-6f);
    ASSERT_NEAR(params.first_order_tau_s, 1.0f, 1e-6f);
    ASSERT_NEAR(params.first_order_delay_s, 0.0f, 1e-6f);
    ASSERT_TRUE(state.active_model_type == PRESSURE_MODEL_TYPE_PHYSICAL);
    ASSERT_NEAR(state.first_order_prev_pressure_bar, 0.0f, 1e-6f);
    ASSERT_TRUE(state.first_order_buffer_index == 0);

    PressureModel_Step(&params, &state, 100.0f, DT_S, &out);

    ASSERT_TRUE(state.active_model_type == PRESSURE_MODEL_TYPE_FIRST_ORDER);
    ASSERT_NEAR(out.measured_pressure_bar, out.real_pressure_bar, 1e-6f);
    ASSERT_TRUE(out.real_pressure_bar > 0.0f);

    return 1;
}
```

Update the call site in `main()`:

```c
    if (test_init_params_default_to_first_order_tuning_contract()) {
        ++passed;
        printf("PASS test_init_params_default_to_first_order_tuning_contract\n");
    } else {
        ++failed;
        printf("FAIL test_init_params_default_to_first_order_tuning_contract\n");
    }
```

In `tests/test_pressure_controller.c`, insert these definitions immediately above `plant_model_step(...)`:

```c
typedef struct {
    HYD_REAL peak_pressure_bar;
    HYD_REAL tail_abs_error_sum;
    HYD_REAL tail_abs_error_max;
    int tail_samples;
    int settle_step;
    HYD_BOOL settled;
} PlantStepMetrics;

static void plant_step_metrics_init(PlantStepMetrics *metrics) {
    memset(metrics, 0, sizeof(*metrics));
    metrics->settle_step = -1;
}

static void plant_step_metrics_record(PlantStepMetrics *metrics,
                                      HYD_REAL target_bar,
                                      HYD_REAL pressure_bar,
                                      int step,
                                      int tail_start_step) {
    HYD_REAL abs_error = fabs(pressure_bar - target_bar);
    HYD_REAL band = target_bar * 0.01;

    if (pressure_bar > metrics->peak_pressure_bar) {
        metrics->peak_pressure_bar = pressure_bar;
    }

    if (step >= tail_start_step) {
        metrics->tail_abs_error_sum += abs_error;
        if (abs_error > metrics->tail_abs_error_max) {
            metrics->tail_abs_error_max = abs_error;
        }
        metrics->tail_samples++;
    }

    if (!metrics->settled && abs_error <= band) {
        metrics->settled = 1;
        metrics->settle_step = step;
    } else if (metrics->settled && abs_error > band) {
        metrics->settled = 0;
        metrics->settle_step = -1;
    }
}

static HYD_REAL plant_step_metrics_tail_mae(const PlantStepMetrics *metrics) {
    return (metrics->tail_samples > 0)
        ? (metrics->tail_abs_error_sum / (HYD_REAL)metrics->tail_samples)
        : 0.0;
}
```

- [ ] **Step 2: Tighten the plant convergence test so it fails against the current tuning**

In `tests/test_pressure_controller.c`, replace `test_rbf_pid_single_setpoint_plant_convergence()` with:

```c
static void test_rbf_pid_single_setpoint_plant_convergence(void) {
    HYD_REAL targets[] = {50.0, 100.0, 200.0};
    int num_targets = 3;
    int ti;

    printf("Testing RBF-PID single-setpoint convergence against plant model...\n");

    for (ti = 0; ti < num_targets; ti++) {
        HYD_REAL target = targets[ti];
        HYD_MotionSegment segment = make_rbf_pid_segment(target);
        HYD_PressureControllerState state;
        HYD_PressureControllerInput input;
        HYD_PressureControllerOutput output;
        HYD_REAL pressure_bar = 0.0;
        HYD_REAL pump_speed;
        HYD_BOOL oscillating = false;
        HYD_REAL prev_flow = 0.0;
        PlantStepMetrics metrics;
        int k;
        int steps = 8000;
        int tail_start_step = steps - 1000;

        plant_step_metrics_init(&metrics);
        HYD_PressureController_InitState(&state, 0.0, 0.0, 0.0);

        for (k = 0; k < steps; k++) {
            input.targetPressure = target;
            input.measuredPressure = pressure_bar;
            input.feedforwardFlow = 0.0;
            input.outputMin = 0.0;
            input.outputMax = segment.maxFlow;
            input.flowToPumpSpeedGain = PLANT_GAIN;
            input.pumpSpeedLimit = PLANT_PUMP_LIMIT;
            input.timestamp = (HYD_REAL)(k + 1) * PLANT_TS;

            HYD_PressureController_Execute(&segment, &state, &input, &output);

            pump_speed = pump_convert(output.outputFlow);
            assert(pump_speed >= -1e-6);
            assert(pump_speed <= PLANT_PUMP_LIMIT + 1e-6);
            assert(output.outputFlow >= -1e-6);
            assert(output.outputFlow <= segment.maxFlow + 1e-6);

            pressure_bar = plant_model_step(pressure_bar, pump_speed);
            plant_step_metrics_record(&metrics, target, pressure_bar, k, tail_start_step);

            if (k >= tail_start_step) {
                if (k > tail_start_step && fabs(output.outputFlow - prev_flow) >= 1.0) {
                    oscillating = true;
                }
                prev_flow = output.outputFlow;
            }
        }

        printf("  Target=%.0f bar: final=%.2f peak=%.2f overshoot=%.2f%% tail_mae=%.3f tail_max=%.3f settle_step=%d\n",
               (double)target,
               (double)pressure_bar,
               (double)metrics.peak_pressure_bar,
               (double)((metrics.peak_pressure_bar - target) / target * 100.0),
               (double)plant_step_metrics_tail_mae(&metrics),
               (double)metrics.tail_abs_error_max,
               metrics.settle_step);

        assert(metrics.peak_pressure_bar <= target * 1.05);
        assert(plant_step_metrics_tail_mae(&metrics) <= target * 0.01);
        assert(metrics.tail_abs_error_max <= target * 0.01);
        assert(metrics.settle_step >= 0);
        assert(!oscillating);
    }

    printf("✓ RBF-PID single-setpoint plant convergence test passed\n");
}
```

- [ ] **Step 3: Run the focused regression and confirm the stricter plant test fails**

Run:

```bash
cmake --build --preset unixgcc --target test_pressure_model test_pressure_controller
ctest --test-dir out/build/unixgcc -R '^(test_pressure_model|test_pressure_controller)$' --output-on-failure
```

Expected:

- `test_pressure_model` passes the first-order boundary contract
- `test_pressure_controller` fails at least one of:
  - `peak_pressure_bar <= target * 1.05`
  - `tail_mae <= target * 0.01`
  - `tail_abs_error_max <= target * 0.01`

- [ ] **Step 4: Commit the stricter acceptance surface**

```bash
git add tests/test_pressure_model.c tests/test_pressure_controller.c
git commit -m "Make the RBF-PID pressure target measurable at 50/100/200 bar" -m "Constraint: Validation must stay on the pure first-order pressure object with a static flow-to-rpm conversion and 1 second tail-error metrics
Rejected: Keeping the older 50/80/100 bar and +/-2% convergence gate | It does not prove the user-requested 200 bar and <=1% steady-state requirement
Confidence: high
Scope-risk: narrow
Directive: Keep plant acceptance logic in test_pressure_controller.c; do not move it into production code
Tested: ctest --test-dir out/build/unixgcc -R '^(test_pressure_model|test_pressure_controller)$' --output-on-failure
Not-tested: Full suite not run in this slice"
```

### Task 2: Align unresolved RBF defaults with the conservative library profile

**Files:**
- Modify: `tests/test_pressure_controller.c`
- Modify: `src/pressure_controller.c`

- [ ] **Step 1: Add a failing test that proves unresolved segment defaults are too aggressive**

In `tests/test_pressure_controller.c`, add this test above `test_rbf_pid_strategy_uses_segment_level_tuning_profile()`:

```c
static void test_rbf_pid_strategy_uses_library_default_tuning_profile(void) {
    HYD_MotionSegment segment;
    HYD_PressureControllerState state;
    HYD_PressureControllerInput input;
    HYD_PressureControllerOutput output;

    printf("Testing RBF-PID library default tuning profile mapping...\n");
    segment = make_pressure_segment();
    segment.pressureController = HYD_PRESSURE_CONTROLLER_RBF_PID;
    memset(&segment.pressureRbfConfig, 0, sizeof(segment.pressureRbfConfig));

    HYD_PressureController_InitState(&state, 5.0, segment.targetFlow, 0.0);

    input.targetPressure = 20.0;
    input.measuredPressure = 5.0;
    input.feedforwardFlow = segment.targetFlow;
    input.outputMin = 0.0;
    input.outputMax = segment.maxFlow;
    input.flowToPumpSpeedGain = 20.0;
    input.pumpSpeedLimit = 1800.0;
    input.timestamp = 0.02;

    HYD_PressureController_Execute(&segment, &state, &input, &output);

    assert(fabsf(state.rbfPid.eta_w - 0.005f) < 1e-6f);
    assert(fabsf(state.rbfPid.eta_c - 0.005f) < 1e-6f);
    assert(fabsf(state.rbfPid.eta_b - 0.005f) < 1e-6f);
    assert(fabsf(state.rbfPid.eta_p - 0.00025f) < 1e-6f);
    assert(fabsf(state.rbfPid.eta_i - 0.00025f) < 1e-6f);
    assert(fabsf(state.rbfPid.eta_d - 0.00025f) < 1e-6f);
    assert(output.adaptiveActive);
    printf("✓ RBF-PID library default tuning profile test passed\n");
}
```

Call it in `main()` immediately before `test_rbf_pid_strategy_uses_segment_level_tuning_profile();`:

```c
    test_rbf_pid_strategy_uses_library_default_tuning_profile();
```

- [ ] **Step 2: Run the focused controller test and confirm the default-profile test fails**

Run:

```bash
cmake --build --preset unixgcc --target test_pressure_controller
ctest --test-dir out/build/unixgcc -R '^test_pressure_controller$' --output-on-failure
```

Expected: `test_rbf_pid_strategy_uses_library_default_tuning_profile()` fails because the unresolved defaults still come from `0.02 / 0.1` in `src/pressure_controller.c`.

- [ ] **Step 3: Lower the unresolved controller defaults to the conservative library profile**

In `src/pressure_controller.c`, replace the default learning-rate macros:

```c
#define HYD_DEFAULT_RBF_LEARNING_RATE 0.005
#define HYD_DEFAULT_PID_LEARNING_RATE 0.00025
```

Keep `HYD_ResolveRbfPidConfig(...)` unchanged apart from using these lower defaults. Do not change the segment override behavior.

- [ ] **Step 4: Re-run the controller regression and confirm only plant-performance failures remain**

Run:

```bash
cmake --build --preset unixgcc --target test_pressure_controller
ctest --test-dir out/build/unixgcc -R '^test_pressure_controller$' --output-on-failure
```

Expected:

- `test_rbf_pid_strategy_uses_library_default_tuning_profile()` passes
- the strict `50 / 100 / 200 bar` plant test still fails until `src/rbf_pid.c` and the plant segment profile are tuned

- [ ] **Step 5: Commit the conservative unresolved-default slice**

```bash
git add src/pressure_controller.c tests/test_pressure_controller.c
git commit -m "Make unresolved RBF defaults match the conservative library profile" -m "Constraint: A segment with zeroed pressureRbfConfig must inherit the same restrained learning rates the RBF library expects internally
Rejected: Keeping 0.02 and 0.1 as resolver defaults | That makes unset segment tuning far more aggressive than the bounded RBF profile used elsewhere
Confidence: high
Scope-risk: narrow
Directive: Keep segment-level pressureRbfConfig overrides authoritative; this slice only fixes the fallback profile
Tested: ctest --test-dir out/build/unixgcc -R '^test_pressure_controller$' --output-on-failure
Not-tested: Full suite not run in this slice"
```

### Task 3: Tighten RBF soft-cap and near-target adaptation before plant-level retuning

**Files:**
- Modify: `tests/rbf_pid_test.c`
- Modify: `src/rbf_pid.c`

- [ ] **Step 1: Tighten the unit tests so the current adaptive logic fails fast**

In `tests/rbf_pid_test.c`, change the soft-cap assertion inside `test_flow_normalization_and_system_gain_soft_cap_are_configurable()`:

```c
    output = RBF_PID_Update(&pid, 120.0f, 0.0f);
    assert(output <= (120.0f * 1.05f / 60.0f) + 1e-3f);
```

Add this new test above `main()`:

```c
static void test_target_relative_small_error_reduces_gain_drift(void) {
    RBF_PID_Handle pid;
    float kp_before;
    float ki_before;
    float kd_before;
    int step;

    printf("Testing target-relative small-error learning restraint...\n");
    RBF_PID_Init(&pid, 0.001f, 90.0f, 1.0f);
    RBF_PID_SetParamLimits(&pid, 0.030f, 0.090f, 0.0005f, 0.0040f, 0.010f, 0.080f);

    for (step = 0; step < 100; ++step) {
        (void)RBF_PID_Update(&pid, 200.0f, 150.0f + (float)step * 0.4f);
    }

    kp_before = pid.KP;
    ki_before = pid.KI;
    kd_before = pid.KD;

    for (step = 0; step < 200; ++step) {
        (void)RBF_PID_Update(&pid, 200.0f, 198.5f);
    }

    assert(fabsf(pid.KP - kp_before) < 0.0025f);
    assert(fabsf(pid.KI - ki_before) < 0.0003f);
    assert(fabsf(pid.KD - kd_before) < 0.0030f);
    printf("PASS target-relative small-error learning restraint test\n");
}
```

Add the declaration near the top:

```c
static void test_target_relative_small_error_reduces_gain_drift(void);
```

And call it in `main()`:

```c
    test_target_relative_small_error_reduces_gain_drift();
```

- [ ] **Step 2: Run the focused RBF unit target and confirm it fails**

Run:

```bash
cmake --build --preset unixgcc --target rbf_pid_test
ctest --test-dir out/build/unixgcc -R '^test_rbf_pid$' --output-on-failure
```

Expected:

- the stricter soft-cap assertion fails because `src/rbf_pid.c` still uses `1.10`
- the new near-target drift test fails because the current small-error learning restraint is too loose

- [ ] **Step 3: Refine `src/rbf_pid.c` so soft-cap and near-target learning are conservative**

In `src/rbf_pid.c`, replace `rbf_pid_apply_default_gains(...)` with:

```c
static void rbf_pid_apply_default_gains(RBF_PID_Handle *pid) {
    pid->KP = 0.048f;
    pid->KI = 0.0008f;
    pid->KD = 0.020f;
}
```

Add this helper above `rbf_pid_step_adaptive_gains(...)`:

```c
static float rbf_pid_target_relative_learning_scale(const RBF_PID_Handle *pid, float error) {
    float setpoint_scale = clamp_positive_or_default(fabsf(pid->P_set), 1.0f);
    float error_ratio = fabsf(error) / setpoint_scale;

    if (error_ratio <= 0.01f) {
        return 0.02f;
    }
    if (error_ratio <= 0.05f) {
        return 0.10f;
    }
    if (error_ratio <= 0.10f) {
        return 0.25f;
    }
    return 1.0f;
}
```

Change `rbf_pid_compute_soft_flow_cap(...)` to:

```c
static float rbf_pid_compute_soft_flow_cap(const RBF_PID_Handle *pid) {
    float hard_limit = rbf_pid_max_flow_output(pid);

    if (pid->K <= 0.0f || pid->P_set <= 0.0f) {
        return hard_limit;
    }

    return clampf(0.0f, (pid->P_set * 1.05f) / pid->K, hard_limit);
}
```

Update `rbf_pid_step_adaptive_gains(...)`:

```c
static void rbf_pid_step_adaptive_gains(RBF_PID_Handle *pid, float error) {
    float de = error - pid->e_prev1;
    float dde = de - (pid->e_prev1 - pid->e_prev2);
    float learning_scale = rbf_pid_target_relative_learning_scale(pid, error);

    if (pid->output_saturated &&
        ((pid->Output >= rbf_pid_compute_soft_flow_cap(pid) - 1.0e-6f && error > 0.0f) ||
         (pid->Output <= MIN_OUTPUT + 1.0e-6f && error < 0.0f))) {
        return;
    }

    pid->KP = clampf(pid->min_KP,
        pid->KP + learning_scale * pid->eta_p * error * pid->Jacobian * de,
        pid->max_KP);
    pid->KI = clampf(pid->min_KI,
        pid->KI + learning_scale * pid->eta_i * error * pid->Jacobian * error,
        pid->max_KI);
    pid->KD = clampf(pid->min_KD,
        pid->KD + learning_scale * pid->eta_d * error * pid->Jacobian * dde,
        pid->max_KD);
}
```

Update the feedforward block inside `rbf_pid_step_incremental_output(...)`:

```c
    float setpoint_scale = clamp_positive_or_default(fabsf(pid->P_set), 1.0f);
    float near_target = fabsf(error) <= 0.02f * setpoint_scale;
    float f_uff = (pid->pressure_accel_ff_enabled && !near_target) ? (-0.15f * f_dd_press) : 0.0f;
    float ref_change = pid->P_set - pid->last_ref;
    float ref_rate = clampf(-10.0f, ref_change, 10.0f);
    float dynamic_ff = near_target ? 0.0f : (0.01f * ref_rate);
```

- [ ] **Step 4: Re-run the RBF unit target and confirm the unit-level contract passes**

Run:

```bash
cmake --build --preset unixgcc --target rbf_pid_test
ctest --test-dir out/build/unixgcc -R '^test_rbf_pid$' --output-on-failure
```

Expected: `test_rbf_pid` passes, but the strict plant convergence test may still fail until the segment profile is tuned.

- [ ] **Step 5: Commit the adaptive-restraint slice**

```bash
git add src/rbf_pid.c tests/rbf_pid_test.c
git commit -m "Restrain RBF-PID learning near the pressure target" -m "Constraint: The controller must still adapt, but it cannot keep chasing gain changes at the same pace as a 1 second first-order pressure transient
Rejected: Disabling adaptation entirely | That would satisfy the number more easily but would not complete the requested RBF-PID optimization
Confidence: medium
Scope-risk: moderate
Directive: Keep target-relative learning restraint simple and local to rbf_pid.c; do not add external tuning surfaces unless the current bounded profile proves insufficient
Tested: ctest --test-dir out/build/unixgcc -R '^test_rbf_pid$' --output-on-failure
Not-tested: Full suite not run in this slice"
```

### Task 4: Encode the tuned segment profile and close the `50 / 100 / 200 bar` acceptance loop

**Files:**
- Modify: `tests/test_pressure_controller.c`

- [ ] **Step 1: Replace the plant test segment helper with the tuned profile**

In `tests/test_pressure_controller.c`, replace `make_rbf_pid_segment(...)` with:

```c
static HYD_MotionSegment make_rbf_pid_segment(HYD_REAL target_bar) {
    HYD_MotionSegment seg;
    memset(&seg, 0, sizeof(seg));
    seg.mode = HYD_MODE_PRESSURE_CLOSED_LOOP;
    seg.endCondition = HYD_END_TIME;
    seg.direction = HYD_DIRECTION_HOLD;
    seg.targetPressure = target_bar;
    seg.maxFlow = PLANT_PUMP_LIMIT / PLANT_GAIN;
    seg.duration = 10.0;
    seg.pressureController = HYD_PRESSURE_CONTROLLER_RBF_PID;
    seg.pressureCeiling = target_bar * 3.0;
    seg.pressureFilterAlpha = 1.0;
    seg.pressureDerivativeFilterAlpha = 1.0;
    seg.systemGain = PLANT_K * PLANT_GAIN;
    seg.pressureRbfConfig.minKp = 0.040;
    seg.pressureRbfConfig.maxKp = 0.060;
    seg.pressureRbfConfig.minKi = 0.0008;
    seg.pressureRbfConfig.maxKi = 0.0016;
    seg.pressureRbfConfig.minKd = 0.015;
    seg.pressureRbfConfig.maxKd = 0.035;
    seg.pressureRbfConfig.etaW = 0.0020;
    seg.pressureRbfConfig.etaC = 0.0020;
    seg.pressureRbfConfig.etaB = 0.0010;
    seg.pressureRbfConfig.etaP = 0.00010;
    seg.pressureRbfConfig.etaI = 0.00005;
    seg.pressureRbfConfig.etaD = 0.00010;
    seg.pressureRbfConfig.disablePressureAccelFeedforward = 1.0;
    return seg;
}
```

- [ ] **Step 2: Extend the segment-profile mapping test so the tuned recipe is visible in code review**

In `tests/test_pressure_controller.c`, append this assertion block to `test_rbf_pid_strategy_uses_segment_level_tuning_profile()`:

```c
    segment.pressureRbfConfig.disablePressureAccelFeedforward = 1.0;
```

and after the existing learning-rate assertions add:

```c
    assert(!state.rbfPid.pressure_accel_ff_enabled);
```

- [ ] **Step 3: Run the focused regression and confirm the strict plant criteria now pass**

Run:

```bash
cmake --build --preset unixgcc --target test_pressure_controller test_pressure_model rbf_pid_test
ctest --test-dir out/build/unixgcc -R '^(test_pressure_controller|test_pressure_model|test_rbf_pid)$' --output-on-failure
```

Expected:

- `test_pressure_model` passes the first-order boundary lock
- `test_rbf_pid` passes the conservative adaptive-unit checks
- `test_pressure_controller` reports passing `50 / 100 / 200 bar` results with:
  - overshoot `<= 5%`
  - last `1s` average absolute error `<= 1%`
  - last `1s` maximum absolute error `<= 1%`

- [ ] **Step 4: Commit the tuned simulation recipe**

```bash
git add tests/test_pressure_controller.c
git commit -m "Tune the RBF pressure recipe for the first-order plant targets" -m "Constraint: The tuned profile must satisfy 50/100/200 bar with <=5% overshoot and <=1% steady-state error using the existing segment-level pressureRbfConfig surface
Rejected: Hiding the tuned values only in the final write-up | The acceptance recipe must live in the regression test that proves it
Confidence: medium
Scope-risk: moderate
Directive: Treat this helper profile as the reference simulation recipe for the first-order plant; do not silently widen the bounds without re-running the strict plant test
Tested: ctest --test-dir out/build/unixgcc -R '^(test_pressure_controller|test_pressure_model|test_rbf_pid)$' --output-on-failure
Not-tested: HIL and full-suite regressions not run in this slice"
```

### Task 5: Run the relevant regression envelope before closing the task

**Files:**
- Modify: none

- [ ] **Step 1: Run the pressure-control regression envelope**

Run:

```bash
ctest --test-dir out/build/unixgcc -R '^(test_pressure_controller|test_pressure_model|test_rbf_pid|test_rbf_pid_hil|test_pressure_hold_diagnosis)$' --output-on-failure
```

Expected:

- all five targets pass
- the stricter first-order plant tests remain green
- the RBF unit suite remains green
- the existing HIL and pressure-hold diagnosis suites do not regress because of the new conservative defaults and adaptive restraint

- [ ] **Step 2: Record the tuned parameter set from the passing regression output for the final answer**

Capture these values from the committed code and test output:

```text
Segment profile:
  Kp window: 0.040 - 0.060
  Ki window: 0.0008 - 0.0016
  Kd window: 0.015 - 0.035
  etaW / etaC / etaB: 0.0020 / 0.0020 / 0.0010
  etaP / etaI / etaD: 0.00010 / 0.00005 / 0.00010
  pressure accel feedforward: disabled

Library fallback profile:
  etaW / etaC / etaB: 0.005 / 0.005 / 0.005
  etaP / etaI / etaD: 0.00025 / 0.00025 / 0.00025
```

Use those exact numbers in the final response section `优化后参数`, and use the three-step test output in `仿真验证说明`.

## Self-Review Checklist

- Spec coverage:
  - algorithm review is implemented through explicit bounded-profile and adaptive-restraint changes in `src/pressure_controller.c` and `src/rbf_pid.c`
  - `50 / 100 / 200 bar` acceptance is implemented in `tests/test_pressure_controller.c`
  - pure first-order boundary is locked in `tests/test_pressure_model.c`
  - final output parameters are explicitly captured in Task 5
- Placeholder scan:
  - no unfinished-marker or deferred-detail phrases remain
- Type consistency:
  - all function and field names match the current codebase:
    - `HYD_PressureController_Execute`
    - `pressureRbfConfig`
    - `pressure_accel_ff_enabled`
    - `PressureModel_InitParams`
    - `RBF_PID_Update`
