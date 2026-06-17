# RBF-PID Pressure Hold Diagnosis Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add repeatable diagnosis tests for the `100 bar` dead-head pressure-hold mismatch, preserve the current `10 / 20 / 30 / 40 rpm` open-loop fit, and add one bounded RBF-PID ablation hook so the test suite can separate model-side ripple from controller-side amplification.

**Architecture:** Keep the diagnosis centered on a new dedicated closed-loop test target that drives `PressureModel_Step(...)` and `HYD_PressureController_Execute(...)` together at `1 ms`. The first slice adds a test-local hold harness and baseline metrics for `real_pressure_bar`, `measured_pressure_bar`, `filteredPressure`, and `outputFlow`. The second slice layers model-side ablation cases without touching production code. The final slice adds a small `disablePressureAccelFeedforward` knob to `HYD_RbfPidConfig` and plumbs it through `pressure_controller.c` and `rbf_pid.c`, so controller-side ablation is testable without broad API churn.

**Tech Stack:** C99, HydroMotionLib, HydroSimLib, CMake/CTest, `libm`

---

## File Structure

| File | Action | Responsibility |
| --- | --- | --- |
| `tests/test_pressure_hold_diagnosis.c` | Create | Dedicated `100 bar` hold diagnosis harness and ablation tests spanning `PressureModel` and `HYD_PressureController` |
| `CMakeLists.txt` | Modify | Register the new diagnosis test target with both `HydroMotionLib` and `HydroSimLib` |
| `include/common_types.h` | Modify | Extend `HYD_RbfPidConfig` with one bounded controller-side diagnosis toggle |
| `src/pressure_controller.c` | Modify | Resolve the new RBF diagnosis toggle from segment config and apply it to the RBF handle |
| `include/rbf_pid.h` | Modify | Store the pressure-acceleration feedforward enable state and expose a narrow setter |
| `src/rbf_pid.c` | Modify | Replace the hardcoded pressure second-difference feedforward term with the new enable-controlled path |
| `tests/rbf_pid_test.c` | Modify | Lock the new RBF feedforward toggle behavior at the unit level |
| `tests/test_pressure_model.c` | Preserve | Existing open-loop fit regression must keep passing unchanged |
| `tests/test_pressure_controller.c` | Preserve | Existing controller integration regression must keep passing unchanged unless explicitly tightened |

**Boundary rules:**

- Do not retune `PressureModel_InitParams(...)` defaults in this plan.
- Do not replace RBF-PID or redesign the controller surface.
- Do not weaken the current open-loop pressure-model regression.
- Keep diagnosis helpers local to the new test target unless a production hook is required for controller ablation.

### Task 1: Create the closed-loop hold diagnosis harness

**Files:**
- Create: `tests/test_pressure_hold_diagnosis.c`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Write the failing diagnosis test skeleton**

Create `tests/test_pressure_hold_diagnosis.c` with this initial content:

```c
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "common_types.h"
#include "pressure_controller.h"
#include "pressure_model.h"

#define HOLD_DT_S 0.001f
#define HOLD_TOTAL_STEPS 30000
#define HOLD_SETTLE_START_STEP 10000
#define HOLD_TARGET_BAR 100.0f
#define HOLD_FLOW_TO_SPEED_GAIN 20.0f
#define HOLD_PUMP_SPEED_LIMIT 1800.0f

typedef struct {
    float target_bar;
    int total_steps;
    int settle_start_step;
    float dt_s;
    PressureModelParams params;
    HYD_MotionSegment segment;
} HoldCaseConfig;

typedef struct {
    float real_p2p_bar;
    float measured_p2p_bar;
    float filtered_p2p_bar;
    float filtered_mae_bar;
    float output_p2p_lmin;
} HoldMetrics;

static HoldCaseConfig make_default_hold_case(void);
static void run_hold_case(const HoldCaseConfig *config, HoldMetrics *metrics);

static void test_hold_harness_produces_finite_metrics(void) {
    HoldCaseConfig config = make_default_hold_case();
    HoldMetrics metrics;

    memset(&metrics, 0, sizeof(metrics));
    run_hold_case(&config, &metrics);

    assert(isfinite(metrics.real_p2p_bar));
    assert(isfinite(metrics.measured_p2p_bar));
    assert(isfinite(metrics.filtered_p2p_bar));
    assert(isfinite(metrics.filtered_mae_bar));
    assert(isfinite(metrics.output_p2p_lmin));
}

int main(void) {
    printf("Running pressure hold diagnosis tests...\n\n");
    test_hold_harness_produces_finite_metrics();
    printf("\nPASS pressure hold diagnosis harness\n");
    return 0;
}
```

- [ ] **Step 2: Register the new test target**

In `CMakeLists.txt`, insert the executable in the simulator-test block immediately after `test_pressure_model`:

```cmake
add_executable(test_pressure_hold_diagnosis tests/test_pressure_hold_diagnosis.c)
target_link_libraries(test_pressure_hold_diagnosis PRIVATE HydroMotionLib HydroSimLib)
```

And register it after `test_pressure_model`:

```cmake
add_test(NAME test_pressure_hold_diagnosis COMMAND test_pressure_hold_diagnosis)
```

- [ ] **Step 3: Run the build and verify it fails because the harness helpers do not exist yet**

Run:

```bash
cmake --preset unixgcc
cmake --build --preset unixgcc --target test_pressure_hold_diagnosis
```

Expected: the build fails at link time with undefined references to `make_default_hold_case` and `run_hold_case`.

- [ ] **Step 4: Implement the test-local hold harness**

Append these helpers to `tests/test_pressure_hold_diagnosis.c` above `test_hold_harness_produces_finite_metrics`:

```c
static PressureModelParams make_default_model_params(void) {
    PressureModelParams params;

    PressureModel_InitParams(&params);
    return params;
}

static HYD_MotionSegment make_default_rbf_segment(float target_bar) {
    HYD_MotionSegment segment;

    memset(&segment, 0, sizeof(segment));
    segment.mode = HYD_MODE_PRESSURE_CLOSED_LOOP;
    segment.endCondition = HYD_END_TIME;
    segment.direction = HYD_DIRECTION_HOLD;
    segment.duration = (HYD_REAL)(HOLD_TOTAL_STEPS * HOLD_DT_S);
    segment.targetPressure = target_bar;
    segment.maxFlow = HOLD_PUMP_SPEED_LIMIT / HOLD_FLOW_TO_SPEED_GAIN;
    segment.pressureController = HYD_PRESSURE_CONTROLLER_RBF_PID;
    segment.pressureCeiling = 250.0;
    segment.pressureFilterAlpha = 1.0;
    segment.pressureDerivativeFilterAlpha = 1.0;
    segment.systemGain = 150.0;
    return segment;
}

static HoldCaseConfig make_default_hold_case(void) {
    HoldCaseConfig config;

    memset(&config, 0, sizeof(config));
    config.target_bar = HOLD_TARGET_BAR;
    config.total_steps = HOLD_TOTAL_STEPS;
    config.settle_start_step = HOLD_SETTLE_START_STEP;
    config.dt_s = HOLD_DT_S;
    config.params = make_default_model_params();
    config.segment = make_default_rbf_segment(config.target_bar);
    return config;
}

static void run_hold_case(const HoldCaseConfig *config, HoldMetrics *metrics) {
    PressureModelState plant_state;
    PressureModelOutput plant_out;
    HYD_PressureControllerState controller_state;
    HYD_PressureControllerInput input;
    HYD_PressureControllerOutput output;
    float real_min = 1.0e30f;
    float real_max = -1.0e30f;
    float measured_min = 1.0e30f;
    float measured_max = -1.0e30f;
    float filtered_min = 1.0e30f;
    float filtered_max = -1.0e30f;
    float output_min = 1.0e30f;
    float output_max = -1.0e30f;
    float filtered_abs_error_sum = 0.0f;
    int filtered_samples = 0;
    int step;

    memset(metrics, 0, sizeof(*metrics));
    memset(&plant_out, 0, sizeof(plant_out));
    memset(&input, 0, sizeof(input));
    memset(&output, 0, sizeof(output));

    PressureModel_Reset(&plant_state, 0x5a5a5a5au);
    HYD_PressureController_InitState(&controller_state, 0.0, 0.0, 0.0);

    for (step = 0; step < config->total_steps; ++step) {
        float target_rpm;

        input.targetPressure = config->target_bar;
        input.measuredPressure = plant_out.measured_pressure_bar;
        input.feedforwardFlow = 0.0;
        input.outputMin = -5.0;
        input.outputMax = config->segment.maxFlow;
        input.flowToPumpSpeedGain = HOLD_FLOW_TO_SPEED_GAIN;
        input.pumpSpeedLimit = HOLD_PUMP_SPEED_LIMIT;
        input.timestamp = (HYD_REAL)((step + 1) * config->dt_s);

        HYD_PressureController_Execute(&config->segment, &controller_state, &input, &output);

        target_rpm = (float)(output.outputFlow * input.flowToPumpSpeedGain);
        PressureModel_Step(&config->params, &plant_state, target_rpm, config->dt_s, &plant_out);

        if (step >= config->settle_start_step) {
            if (plant_out.real_pressure_bar < real_min) real_min = plant_out.real_pressure_bar;
            if (plant_out.real_pressure_bar > real_max) real_max = plant_out.real_pressure_bar;
            if (plant_out.measured_pressure_bar < measured_min) measured_min = plant_out.measured_pressure_bar;
            if (plant_out.measured_pressure_bar > measured_max) measured_max = plant_out.measured_pressure_bar;
            if ((float)output.filteredPressure < filtered_min) filtered_min = (float)output.filteredPressure;
            if ((float)output.filteredPressure > filtered_max) filtered_max = (float)output.filteredPressure;
            if ((float)output.outputFlow < output_min) output_min = (float)output.outputFlow;
            if ((float)output.outputFlow > output_max) output_max = (float)output.outputFlow;
            filtered_abs_error_sum += fabsf((float)output.filteredPressure - config->target_bar);
            ++filtered_samples;
        }
    }

    metrics->real_p2p_bar = real_max - real_min;
    metrics->measured_p2p_bar = measured_max - measured_min;
    metrics->filtered_p2p_bar = filtered_max - filtered_min;
    metrics->filtered_mae_bar =
        (filtered_samples > 0) ? (filtered_abs_error_sum / (float)filtered_samples) : 0.0f;
    metrics->output_p2p_lmin = output_max - output_min;
}
```

- [ ] **Step 5: Run the harness test and verify it passes**

Run:

```bash
cmake --build --preset unixgcc --target test_pressure_hold_diagnosis
ctest --test-dir out/build/unixgcc -R '^test_pressure_hold_diagnosis$' --output-on-failure
```

Expected: the new test target builds and the harness test passes.

- [ ] **Step 6: Commit the harness slice**

```bash
git add CMakeLists.txt tests/test_pressure_hold_diagnosis.c
git commit -m "Make the 100 bar hold mismatch measurable in tests" -m "Constraint: The diagnosis harness must exercise PressureModel and HYD_PressureController together without changing plant defaults
Rejected: Reusing test_pressure_controller.c for the hold harness | It would bury diagnosis logic inside an already large file
Confidence: high
Scope-risk: narrow
Directive: Keep the diagnosis helpers local to the new test target until a production hook is truly required
Tested: ctest --test-dir out/build/unixgcc -R '^test_pressure_hold_diagnosis$' --output-on-failure
Not-tested: Full suite not run in this slice"
```

### Task 2: Add baseline and model-side ablation diagnosis tests

**Files:**
- Modify: `tests/test_pressure_hold_diagnosis.c`

- [ ] **Step 1: Write the failing baseline and ablation tests**

Append these declarations above `main()`:

```c
static void test_current_100_bar_hold_reproduces_large_visible_ripple(void);
static void test_tooth_drop_ablation_reduces_measured_ripple_more_than_real_ripple(void);
static void test_motor_noise_ablation_reduces_real_and_filtered_ripple(void);
static void test_sensor_noise_ablation_preserves_real_pressure_more_than_measured_pressure(void);
```

Append these calls inside `main()` after `test_hold_harness_produces_finite_metrics();`:

```c
    test_current_100_bar_hold_reproduces_large_visible_ripple();
    test_tooth_drop_ablation_reduces_measured_ripple_more_than_real_ripple();
    test_motor_noise_ablation_reduces_real_and_filtered_ripple();
    test_sensor_noise_ablation_preserves_real_pressure_more_than_measured_pressure();
```

Append the test bodies above `main()`:

```c
static void test_current_100_bar_hold_reproduces_large_visible_ripple(void) {
    HoldCaseConfig config = make_default_hold_case();
    HoldMetrics metrics;

    run_hold_case(&config, &metrics);

    printf("baseline hold: real=%.2f measured=%.2f filtered=%.2f mae=%.2f output=%.2f\n",
           metrics.real_p2p_bar,
           metrics.measured_p2p_bar,
           metrics.filtered_p2p_bar,
           metrics.filtered_mae_bar,
           metrics.output_p2p_lmin);

    assert(metrics.measured_p2p_bar > metrics.real_p2p_bar + 5.0f);
    assert(metrics.filtered_p2p_bar > 10.0f);
    assert(metrics.filtered_mae_bar > 5.0f);
}

static void test_tooth_drop_ablation_reduces_measured_ripple_more_than_real_ripple(void) {
    HoldCaseConfig baseline = make_default_hold_case();
    HoldCaseConfig flat = baseline;
    HoldMetrics base_metrics;
    HoldMetrics flat_metrics;

    flat.params.tooth_drop_depth_ratio = 0.0f;
    flat.params.tooth_drop_depth_base = 0.0f;

    run_hold_case(&baseline, &base_metrics);
    run_hold_case(&flat, &flat_metrics);

    assert(flat_metrics.measured_p2p_bar < base_metrics.measured_p2p_bar * 0.70f);
    assert(fabsf(flat_metrics.real_p2p_bar - base_metrics.real_p2p_bar) <
           (base_metrics.real_p2p_bar * 0.30f + 0.5f));
}

static void test_motor_noise_ablation_reduces_real_and_filtered_ripple(void) {
    HoldCaseConfig noisy = make_default_hold_case();
    HoldCaseConfig quiet = noisy;
    HoldMetrics noisy_metrics;
    HoldMetrics quiet_metrics;

    quiet.params.enable_motor_noise = 0u;
    quiet.params.motor_noise_std_rpm = 0.0f;

    run_hold_case(&noisy, &noisy_metrics);
    run_hold_case(&quiet, &quiet_metrics);

    assert(quiet_metrics.real_p2p_bar < noisy_metrics.real_p2p_bar * 0.85f);
    assert(quiet_metrics.filtered_p2p_bar < noisy_metrics.filtered_p2p_bar * 0.85f);
}

static void test_sensor_noise_ablation_preserves_real_pressure_more_than_measured_pressure(void) {
    HoldCaseConfig noisy = make_default_hold_case();
    HoldCaseConfig quiet = noisy;
    HoldMetrics noisy_metrics;
    HoldMetrics quiet_metrics;

    quiet.params.enable_sensor_noise = 0u;
    quiet.params.sensor_noise_std_bar = 0.0f;

    run_hold_case(&noisy, &noisy_metrics);
    run_hold_case(&quiet, &quiet_metrics);

    assert(fabsf(quiet_metrics.real_p2p_bar - noisy_metrics.real_p2p_bar) <
           (noisy_metrics.real_p2p_bar * 0.15f + 0.25f));
    assert(quiet_metrics.measured_p2p_bar <= noisy_metrics.measured_p2p_bar);
}
```

- [ ] **Step 2: Run the target and confirm at least one of the new tests fails before the assertions are tuned against the live harness**

Run:

```bash
cmake --build --preset unixgcc --target test_pressure_hold_diagnosis
ctest --test-dir out/build/unixgcc -R '^test_pressure_hold_diagnosis$' --output-on-failure
```

Expected: the new tests compile, and at least one assertion fails if the current code path does not yet reproduce the expected diagnosis relationships.

- [ ] **Step 3: Adjust only diagnosis-local test setup, not production defaults, until the baseline and model-side comparisons become stable**

First, if the target fails because the RBF segment setup is too weak to reach a stable `100 bar` hold, replace `make_default_rbf_segment(...)` with:

```c
static HYD_MotionSegment make_default_rbf_segment(float target_bar) {
    HYD_MotionSegment segment;

    memset(&segment, 0, sizeof(segment));
    segment.mode = HYD_MODE_PRESSURE_CLOSED_LOOP;
    segment.endCondition = HYD_END_TIME;
    segment.direction = HYD_DIRECTION_HOLD;
    segment.duration = (HYD_REAL)(HOLD_TOTAL_STEPS * HOLD_DT_S);
    segment.targetPressure = target_bar;
    segment.maxFlow = HOLD_PUMP_SPEED_LIMIT / HOLD_FLOW_TO_SPEED_GAIN;
    segment.pressureController = HYD_PRESSURE_CONTROLLER_RBF_PID;
    segment.pressureCeiling = 250.0;
    segment.pressureFilterAlpha = 1.0;
    segment.pressureDerivativeFilterAlpha = 1.0;
    segment.systemGain = 150.0;
    segment.pressureRbfConfig.minKp = 0.5;
    segment.pressureRbfConfig.maxKp = 1.2;
    segment.pressureRbfConfig.minKi = 0.005;
    segment.pressureRbfConfig.maxKi = 0.050;
    segment.pressureRbfConfig.minKd = 0.5;
    segment.pressureRbfConfig.maxKd = 2.0;
    return segment;
}
```

If the focused test still fails because the current simulator defaults do not cleanly separate visible model ripple from controller amplification, switch the harness to an explicit diagnosis-only preset that stays local to `tests/test_pressure_hold_diagnosis.c`.

Use this `make_default_model_params(...)`:

```c
static PressureModelParams make_default_model_params(void) {
    PressureModelParams params;

    PressureModel_InitParams(&params);
    params.sensor_noise_std_bar = 0.8f;
    params.motor_noise_std_rpm = 1.0f;
    params.tooth_drop_depth_ratio = 0.34f;
    return params;
}
```

And use this diagnosis-oriented `make_default_rbf_segment(...)`:

```c
static HYD_MotionSegment make_default_rbf_segment(float target_bar) {
    HYD_MotionSegment segment;

    memset(&segment, 0, sizeof(segment));
    segment.mode = HYD_MODE_PRESSURE_CLOSED_LOOP;
    segment.endCondition = HYD_END_TIME;
    segment.direction = HYD_DIRECTION_HOLD;
    segment.duration = (HYD_REAL)(HOLD_TOTAL_STEPS * HOLD_DT_S);
    segment.targetPressure = target_bar;
    segment.maxFlow = HOLD_PUMP_SPEED_LIMIT / HOLD_FLOW_TO_SPEED_GAIN;
    segment.pressureController = HYD_PRESSURE_CONTROLLER_RBF_PID;
    segment.pressureCeiling = 250.0;
    segment.pressureFilterAlpha = 0.20;
    segment.pressureDerivativeFilterAlpha = 1.0;
    segment.systemGain = 30.0;
    segment.pressureRbfConfig.minKp = 0.5;
    segment.pressureRbfConfig.maxKp = 1.2;
    segment.pressureRbfConfig.minKi = 0.005;
    segment.pressureRbfConfig.maxKi = 0.050;
    segment.pressureRbfConfig.minKd = 0.5;
    segment.pressureRbfConfig.maxKd = 2.0;
    return segment;
}
```

This preset is intentionally test-local. It does **not** change `PressureModel_InitParams(...)` or any production defaults; it only tunes the diagnosis harness so the `100 bar` hold remains near target while the tooth-drop, motor-noise, and sensor-noise ablations separate consistently.

Do not modify `PressureModel_InitParams(...)` in this task or change any production source files.

- [ ] **Step 4: Re-run the diagnosis test and verify the model-side comparisons pass**

Run:

```bash
ctest --test-dir out/build/unixgcc -R '^test_pressure_hold_diagnosis$' --output-on-failure
```

Expected: all Task 2 tests pass and print one baseline metrics line.

- [ ] **Step 5: Commit the model-side diagnosis slice**

```bash
git add tests/test_pressure_hold_diagnosis.c
git commit -m "Classify the hold mismatch on the model side" -m "Constraint: The diagnosis must preserve the existing open-loop pressure-model regression while exposing the 100 bar hold mismatch
Rejected: Retuning PressureModel defaults before adding ablation tests | It would erase the current failure before the suite can classify it
Confidence: medium
Scope-risk: narrow
Directive: Keep all model-side diagnosis assertions comparative so they stay valid while defaults remain unchanged
Tested: ctest --test-dir out/build/unixgcc -R '^test_pressure_hold_diagnosis$' --output-on-failure
Not-tested: Full simulator regression suite not run in this slice"
```

### Task 3: Add a bounded RBF pressure-acceleration feedforward toggle

**Files:**
- Modify: `include/common_types.h`
- Modify: `src/pressure_controller.c`
- Modify: `include/rbf_pid.h`
- Modify: `src/rbf_pid.c`
- Modify: `tests/rbf_pid_test.c`

- [ ] **Step 1: Write the failing RBF unit test for the new toggle**

In `tests/rbf_pid_test.c`, add this declaration above `main()`:

```c
static void test_pressure_accel_feedforward_toggle_changes_incremental_output(void);
```

Call it from `main()` before the final success print:

```c
    test_pressure_accel_feedforward_toggle_changes_incremental_output();
```

Add the test body above `main()`:

```c
static void test_pressure_accel_feedforward_toggle_changes_incremental_output(void) {
    RBF_PID_Handle enabled;
    RBF_PID_Handle disabled;
    float out_enabled;
    float out_disabled;

    printf("Testing pressure acceleration feedforward toggle...\n");

    RBF_PID_Init(&enabled, 0.001f, 90.0f, 1.0f);
    RBF_PID_Init(&disabled, 0.001f, 90.0f, 1.0f);

    (void)RBF_PID_Update(&enabled, 100.0f, 98.0f);
    (void)RBF_PID_Update(&disabled, 100.0f, 98.0f);

    RBF_PID_SetPressureAccelFeedforwardEnabled(&disabled, false);

    out_enabled = RBF_PID_Update(&enabled, 100.0f, 101.0f);
    out_disabled = RBF_PID_Update(&disabled, 100.0f, 101.0f);

    assert(fabsf(out_enabled - out_disabled) > 1e-5f);
    printf("✓ Pressure acceleration feedforward toggle test passed\n");
}
```

- [ ] **Step 2: Build and confirm it fails because the setter does not exist yet**

Run:

```bash
cmake --build --preset unixgcc --target rbf_pid_test
```

Expected: compile or link failure for `RBF_PID_SetPressureAccelFeedforwardEnabled`.

- [ ] **Step 3: Add the toggle to the RBF config and handle**

In `include/common_types.h`, extend `HYD_RbfPidConfig` with:

```c
    HYD_REAL disablePressureAccelFeedforward; /* >0 disables the internal -0.5 * d2P term, 0 keeps the default enabled */
```

So the struct becomes:

```c
typedef struct {
    HYD_REAL minKp;
    HYD_REAL maxKp;
    HYD_REAL minKi;
    HYD_REAL maxKi;
    HYD_REAL minKd;
    HYD_REAL maxKd;
    HYD_REAL etaW;
    HYD_REAL etaC;
    HYD_REAL etaB;
    HYD_REAL etaP;
    HYD_REAL etaI;
    HYD_REAL etaD;
    HYD_REAL disablePressureAccelFeedforward;
} HYD_RbfPidConfig;
```

In `include/rbf_pid.h`, add the handle field near the other runtime behavior flags:

```c
    bool pressure_accel_ff_enabled;
```

And add the setter declaration:

```c
void RBF_PID_SetPressureAccelFeedforwardEnabled(RBF_PID_Handle *pid, bool enabled);
```

- [ ] **Step 4: Plumb the toggle through `rbf_pid.c` and `pressure_controller.c`**

In `src/rbf_pid.c`, initialize the new field in `RBF_PID_Init(...)`:

```c
    pid->pressure_accel_ff_enabled = true;
```

Replace the hardcoded `f_uff` line in `rbf_pid_step_incremental_output(...)`:

```c
    float f_uff = pid->pressure_accel_ff_enabled ? (-0.5f * f_dd_press) : 0.0f;
```

Add the setter at the end of `src/rbf_pid.c`:

```c
void RBF_PID_SetPressureAccelFeedforwardEnabled(RBF_PID_Handle *pid, bool enabled) {
    if (pid == NULL) {
        return;
    }
    pid->pressure_accel_ff_enabled = enabled;
}
```

In `src/pressure_controller.c`, extend `HYD_RbfPidResolvedConfig` with:

```c
    HYD_BOOL disablePressureAccelFeedforward;
```

Initialize and resolve it in `HYD_ResolveRbfPidConfig(...)`:

```c
    config->disablePressureAccelFeedforward = false;
```

```c
    config->disablePressureAccelFeedforward =
        (segment != NULL && segment->pressureRbfConfig.disablePressureAccelFeedforward > 0.0)
        ? true : false;
```

And apply it inside `HYD_ApplyRbfPidConfig(...)` after `RBF_PID_SetLearningRates(...)`:

```c
    RBF_PID_SetPressureAccelFeedforwardEnabled(
        &state->rbfPid,
        config->rbf.disablePressureAccelFeedforward ? false : true);
```

- [ ] **Step 5: Run the RBF unit test and verify it passes**

Run:

```bash
cmake --build --preset unixgcc --target rbf_pid_test
ctest --test-dir out/build/unixgcc -R '^test_rbf_pid$' --output-on-failure
```

Expected: `rbf_pid_test` passes with the new toggle covered.

- [ ] **Step 6: Commit the bounded controller hook**

```bash
git add include/common_types.h include/rbf_pid.h src/pressure_controller.c src/rbf_pid.c tests/rbf_pid_test.c
git commit -m "Expose a bounded RBF feedforward ablation hook" -m "Constraint: Controller-side diagnosis needs one narrow way to disable the pressure second-difference feedforward term without redesigning the RBF API
Rejected: Adding a broad test-only preprocessor switch in rbf_pid.c | It would hide behavior behind build modes instead of runtime config
Confidence: medium
Scope-risk: moderate
Directive: Keep zero-value HYD_RbfPidConfig behavior backward-compatible and use the new field only as an explicit opt-out
Tested: ctest --test-dir out/build/unixgcc -R '^test_rbf_pid$' --output-on-failure
Not-tested: Full controller regression suite not run in this slice"
```

### Task 4: Add controller-side ablation tests and run the regression bundle

**Files:**
- Modify: `tests/test_pressure_hold_diagnosis.c`

- [ ] **Step 1: Write the failing controller-side ablation tests**

Append these declarations above `main()`:

```c
static void test_stronger_filter_reduces_filtered_ripple(void);
static void test_disabling_pressure_accel_feedforward_reduces_output_hunting(void);
static void test_disabling_gain_compensation_changes_hold_metrics_materially(void);
```

Append these calls inside `main()` after the model-side tests:

```c
    test_stronger_filter_reduces_filtered_ripple();
    test_disabling_pressure_accel_feedforward_reduces_output_hunting();
    test_disabling_gain_compensation_changes_hold_metrics_materially();
```

Append the test bodies above `main()`:

```c
static void test_stronger_filter_reduces_filtered_ripple(void) {
    HoldCaseConfig raw = make_default_hold_case();
    HoldCaseConfig filtered = raw;
    HoldMetrics raw_metrics;
    HoldMetrics filtered_metrics;

    filtered.segment.pressureFilterAlpha = 0.1;

    run_hold_case(&raw, &raw_metrics);
    run_hold_case(&filtered, &filtered_metrics);

    assert(filtered_metrics.filtered_p2p_bar < raw_metrics.filtered_p2p_bar * 0.80f);
}

static void test_disabling_pressure_accel_feedforward_reduces_output_hunting(void) {
    HoldCaseConfig enabled = make_default_hold_case();
    HoldCaseConfig disabled = enabled;
    HoldMetrics enabled_metrics;
    HoldMetrics disabled_metrics;

    disabled.segment.pressureRbfConfig.disablePressureAccelFeedforward = 1.0;

    run_hold_case(&enabled, &enabled_metrics);
    run_hold_case(&disabled, &disabled_metrics);

    assert(disabled_metrics.output_p2p_lmin < enabled_metrics.output_p2p_lmin * 0.90f);
}

static void test_disabling_gain_compensation_changes_hold_metrics_materially(void) {
    HoldCaseConfig compensated = make_default_hold_case();
    HoldCaseConfig uncompensated = compensated;
    HoldMetrics compensated_metrics;
    HoldMetrics uncompensated_metrics;

    uncompensated.segment.systemGain = 0.0;

    run_hold_case(&compensated, &compensated_metrics);
    run_hold_case(&uncompensated, &uncompensated_metrics);

    assert(fabsf(compensated_metrics.filtered_mae_bar -
                 uncompensated_metrics.filtered_mae_bar) > 0.5f);
}
```

- [ ] **Step 2: Run the diagnosis target and confirm it fails until the new feedforward hook is available through the controller path**

Run:

```bash
cmake --build --preset unixgcc --target test_pressure_hold_diagnosis
ctest --test-dir out/build/unixgcc -R '^test_pressure_hold_diagnosis$' --output-on-failure
```

Expected: if Task 3 has not been completed, the feedforward-ablation test is still ineffective and at least one assertion fails.

- [ ] **Step 3: Re-run after Task 3 and verify all diagnosis tests pass**

Run:

```bash
ctest --test-dir out/build/unixgcc -R '^test_pressure_hold_diagnosis$' --output-on-failure
```

Expected: all baseline, model-side, and controller-side diagnosis tests pass.

- [ ] **Step 4: Run the regression bundle that protects the current open-loop and controller surfaces**

Run:

```bash
ctest --test-dir out/build/unixgcc -R '^(test_pressure_model|test_pressure_controller|test_rbf_pid|test_pressure_hold_diagnosis)$' --output-on-failure
```

Expected: all four targets pass.

- [ ] **Step 5: Commit the completed diagnosis suite**

```bash
git add tests/test_pressure_hold_diagnosis.c
git commit -m "Separate plant ripple from controller amplification in hold tests" -m "Constraint: The diagnosis suite must preserve the existing open-loop regression while adding repeatable 100 bar hold evidence
Rejected: Baking fixed tuned thresholds into production defaults in the same change | The suite must classify first, tune later
Confidence: medium
Scope-risk: narrow
Directive: Keep the new diagnosis tests comparative so later parameter tuning can use them as evidence instead of rewriting them
Tested: ctest --test-dir out/build/unixgcc -R '^(test_pressure_model|test_pressure_controller|test_rbf_pid|test_pressure_hold_diagnosis)$' --output-on-failure
Not-tested: Broader suite outside the pressure-model and pressure-controller surfaces not run in this slice"
```

## Self-Review Checklist

### Spec coverage

- `100 bar` dead-head hold baseline: covered in Task 1 and Task 2
- four-signal diagnosis split (`real`, `measured`, `filtered`, `output`): covered in Task 1 harness metrics
- model-side ablations: covered in Task 2
- controller-side ablations: covered in Task 3 and Task 4
- preserve open-loop fit: covered by Task 4 regression bundle with `test_pressure_model`
- test-backed validation, not manual trend reading: covered throughout

### Placeholder scan

- No `TODO`, `TBD`, or “similar to Task N” placeholders
- Every code-changing step includes the concrete code to add
- Every verification step includes the exact command to run

### Type consistency

- `HoldCaseConfig` and `HoldMetrics` are defined in Task 1 before later tests use them
- `disablePressureAccelFeedforward` is added to `HYD_RbfPidConfig` before later tasks use it
- `RBF_PID_SetPressureAccelFeedforwardEnabled(...)` is declared in `include/rbf_pid.h` before later tasks call it
