# RBF-PID Flow-Domain Migration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Move the adaptive pressure controller to a pure flow-domain (`L/min`) implementation, fix the RBF causal input contract, keep `src/pump_converter.c` as the only `flow -> rpm` conversion point, and verify the plant-model overshoot stays below `5%`.

**Architecture:** Keep the public pressure-loop entry point in `src/pressure_controller.c`, but make `RBF_PID_Update()` operate only on flow-domain state and flow-domain limits. The RBF input vector becomes strictly historical: `[du(k-1), y(k-1), y(k-2), e(k-1)]`, with separate flow and pressure normalization scales so `du_prev` and `y_prev1` are no longer divided by the same scalar. `systemGain` remains available as plant metadata for a flow-domain soft cap, but it must not multiply the controller output.

**Tech Stack:** C99, HydroMotionLib, HydroSimLib, CMake, CTest, `libm`

---

## File Structure

| File | Action | Responsibility |
| --- | --- | --- |
| `include/rbf_pid.h` | Modify | Declare the flow-domain state contract, split normalization fields, causal-history storage, and the new flow-normalization setter |
| `src/rbf_pid.c` | Modify | Implement causal RBF input handling, normalized Jacobian recovery, pure flow-domain output, flow-domain soft cap, and conservative adaptive-learning restraint |
| `src/pressure_controller.c` | Modify | Resolve per-segment normalization scales, seed/synchronize the RBF state in flow space, and clear old speed-domain compensation assumptions |
| `tests/rbf_pid_test.c` | Modify | Lock the new unit-domain, causal-input, and soft-cap behavior with deterministic unit tests |
| `tests/test_pressure_controller.c` | Modify | Validate the migrated controller against the plant model using the real pump converter path and enforce `< 5%` overshoot |
| `tests/test_rbf_pid_hil.c` | Modify | Verify the full motion-control stack still routes pressure-loop flow through `HYD_PumpConverter_Execute(...)` and remains bounded in the simulator |
| `src/pump_converter.c` | Preserve | Remains the sole `flow -> rpm` conversion point |
| `src/motion_interface.c` | Preserve | Continues passing segment gains and pump parameters through without new unit conversion logic |

**Boundary rules:**

- Do not implement online `K` identification in this plan.
- Do not implement a Smith predictor in this plan.
- Do not add a second `flow -> rpm` conversion path anywhere outside `src/pump_converter.c`.
- Do not keep the old output-end multiplication/division path inside `src/rbf_pid.c`.
- Keep all RBF, PID, compensation, and limiting logic in `L/min`.

### Task 1: Lock the flow-domain and causal-input contract in tests and headers

**Files:**
- Modify: `tests/rbf_pid_test.c`
- Modify: `include/rbf_pid.h`

- [ ] **Step 1: Rewrite the RBF unit tests so they describe the new contract before implementation**

In `tests/rbf_pid_test.c`, replace the old normalization / gain-compensation declaration with these declarations near the top of the file:

```c
static void test_flow_normalization_and_system_gain_soft_cap_are_configurable(void);
static void test_flow_domain_output_is_independent_from_pump_gain(void);
static void test_rbf_input_uses_causal_history_and_split_normalization(void);
```

Replace the old call site block in `main()`:

```c
    test_flow_normalization_and_system_gain_soft_cap_are_configurable();
    test_flow_domain_output_is_independent_from_pump_gain();
    test_rbf_input_uses_causal_history_and_split_normalization();
```

Replace the old `test_pressure_normalization_and_gain_compensation_are_configurable(...)` body with:

```c
static void test_flow_normalization_and_system_gain_soft_cap_are_configurable(void) {
    RBF_PID_Handle pid;
    float output;

    printf("Testing flow normalization and system-gain soft cap configuration...\n");
    RBF_PID_Init(&pid, 0.01f, 90.0f, 1.0f);
    RBF_PID_SetFlowNormalization(&pid, 45.0f);
    RBF_PID_SetPressureNormalization(&pid, 180.0f);
    RBF_PID_SetGainCompensation(&pid, 60.0f);
    RBF_PID_SetLearningRates(&pid, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

    assert(fabsf(pid.flow_normalization_scale - 45.0f) < 1e-6f);
    assert(fabsf(pid.pressure_normalization_scale - 180.0f) < 1e-6f);
    assert(pid.gain_compensation_enabled);
    assert(fabsf(pid.gain_compensation_factor - 1.0f) < 1e-6f);

    output = RBF_PID_Update(&pid, 120.0f, 0.0f);
    assert(output <= (120.0f * 1.10f / 60.0f) + 1e-3f);

    RBF_PID_SetGainCompensation(&pid, 0.0f);
    assert(!pid.gain_compensation_enabled);
    assert(fabsf(pid.gain_compensation_factor - 1.0f) < 1e-6f);
    printf("PASS flow normalization / system-gain soft cap test\n");
}
```

Append these two new tests above `main()`:

```c
static void test_flow_domain_output_is_independent_from_pump_gain(void) {
    RBF_PID_Handle base;
    RBF_PID_Handle altered;
    float out_base;
    float out_altered;

    printf("Testing flow-domain controller independence from pump-speed gain...\n");
    RBF_PID_Init(&base, 0.001f, 90.0f, 1.0f);
    RBF_PID_Init(&altered, 0.001f, 90.0f, 1.0f);
    RBF_PID_SetLearningRates(&base, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    RBF_PID_SetLearningRates(&altered, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

    altered.flowToPumpSpeedGain = 37.0f;

    (void)RBF_PID_Update(&base, 80.0f, 20.0f);
    (void)RBF_PID_Update(&altered, 80.0f, 20.0f);

    out_base = RBF_PID_Update(&base, 80.0f, 25.0f);
    out_altered = RBF_PID_Update(&altered, 80.0f, 25.0f);

    assert(fabsf(out_base - out_altered) < 1e-6f);
    assert(fabsf(base.n_out - out_base) < 1e-6f);
    assert(fabsf(altered.n_out - out_altered) < 1e-6f);
    printf("PASS flow-domain controller independence test\n");
}

static void test_rbf_input_uses_causal_history_and_split_normalization(void) {
    RBF_PID_Handle pid;
    float prev_du;

    printf("Testing RBF causal input vector and split normalization...\n");
    RBF_PID_Init(&pid, 0.001f, 90.0f, 1.0f);
    RBF_PID_SetFlowNormalization(&pid, 45.0f);
    RBF_PID_SetPressureNormalization(&pid, 200.0f);
    RBF_PID_SetLearningRates(&pid, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

    (void)RBF_PID_Update(&pid, 100.0f, 40.0f);
    (void)RBF_PID_Update(&pid, 100.0f, 55.0f);
    prev_du = pid.du_prev;
    (void)RBF_PID_Update(&pid, 100.0f, 60.0f);

    assert(fabsf(pid.last_rbf_input[0] - (prev_du / 45.0f)) < 1e-6f);
    assert(fabsf(pid.last_rbf_input[1] - (55.0f / 200.0f)) < 1e-6f);
    assert(fabsf(pid.last_rbf_input[2] - (40.0f / 200.0f)) < 1e-6f);
    assert(fabsf(pid.last_rbf_input[3] - ((100.0f - 55.0f) / 200.0f)) < 1e-6f);
    assert(fabsf(pid.last_rbf_input[0] - (prev_du / 200.0f)) > 1e-4f);
    printf("PASS RBF causal input vector test\n");
}
```

- [ ] **Step 2: Run the RBF unit target and confirm it fails on missing flow-normalization / telemetry support**

Run:

```bash
cmake --build --preset unixgcc --target rbf_pid_test
```

Expected: build failure for `RBF_PID_SetFlowNormalization`, `flow_normalization_scale`, or `last_rbf_input`, plus at least one old assumption about `n_out` still behaving like a motor-speed-domain quantity.

- [ ] **Step 3: Extend the public RBF handle and API so the test contract can compile**

In `include/rbf_pid.h`, update the runtime state section to:

```c
    float pressure_normalization_scale;
    float flow_normalization_scale;
    float flowToPumpSpeedGain;      /* retained for outer integration only */
```

Update the output/state section to:

```c
    float Output;                   /* last commanded flow [L/min] */
    float KP;
    float KI;
    float KD;
    float du;
    float Error;
    float Jacobian;
    float min_KP;
    float max_KP;
    float min_KI;
    float max_KI;
    float min_KD;
    float max_KD;
    int32_t Status;
    int32_t TuneResult;
    float n_out;                    /* mirrored flow-domain command [L/min] */
```

Extend the historical state and test telemetry section to:

```c
    float u_prev;
    float e_prev1;
    float e_prev2;
    float du_prev;
    int32_t steady_count;
    bool steady_state;
    bool output_saturated;
    float y_prev1;
    float y_prev2;
    float last_rbf_input[RBF_INPUT_DIM];
```

Add the setter declaration next to `RBF_PID_SetPressureNormalization(...)`:

```c
void RBF_PID_SetFlowNormalization(RBF_PID_Handle *pid, float scale);
```

Update the `RBF_PID_Update(...)` comment to keep the contract explicit:

```c
 * @return 控制器输出流量 [L/min], 不在此函数内执行 flow -> rpm 转换
```

- [ ] **Step 4: Rebuild and confirm the tests now compile but still fail against the old speed-domain implementation**

Run:

```bash
cmake --build --preset unixgcc --target rbf_pid_test
ctest --test-dir out/build/unixgcc -R '^test_rbf_pid$' --output-on-failure
```

Expected: the target builds, but `test_flow_domain_output_is_independent_from_pump_gain(...)` and `test_rbf_input_uses_causal_history_and_split_normalization(...)` fail until `src/rbf_pid.c` is refactored.

- [ ] **Step 5: Commit the contract and failing-test slice**

```bash
git add include/rbf_pid.h tests/rbf_pid_test.c
git commit -m "Define the flow-domain RBF-PID contract before migration" -m "Constraint: The adaptive controller must end this phase entirely in L/min and expose enough history to prove the causal input contract
Rejected: Leaving the unit-domain expectations implicit in pressure_controller integration tests | It would hide the core migration contract behind a larger failure surface
Confidence: high
Scope-risk: narrow
Directive: Keep last_rbf_input as a narrow diagnostic aid for unit tests; do not add more debug-only public state unless a test cannot be expressed otherwise
Tested: cmake --build --preset unixgcc --target rbf_pid_test; ctest --test-dir out/build/unixgcc -R '^test_rbf_pid$' --output-on-failure
Not-tested: Full suite not run in this slice"
```

### Task 2: Refactor `src/rbf_pid.c` to causal, split-normalized, pure flow-domain control

**Files:**
- Modify: `include/rbf_pid.h`
- Modify: `src/rbf_pid.c`
- Modify: `tests/rbf_pid_test.c`

- [ ] **Step 1: Move the built-in defaults to flow-domain values and add the flow-normalization setter**

In `include/rbf_pid.h`, replace the default adaptive window macros with:

```c
#define PID_MIN_KP          0.030f
#define PID_MAX_KP          0.090f
#define PID_MIN_KI          0.0005f
#define PID_MAX_KI          0.0040f
#define PID_MIN_KD          0.010f
#define PID_MAX_KD          0.080f
```

In `src/rbf_pid.c`, replace the default gain and learning-rate helpers with:

```c
static void rbf_pid_apply_default_learning_rates(RBF_PID_Handle *pid) {
    pid->eta_w = 0.005f;
    pid->eta_c = 0.005f;
    pid->eta_b = 0.005f;
    pid->eta_p = 0.00025f;
    pid->eta_i = 0.00025f;
    pid->eta_d = 0.00025f;
}

static void rbf_pid_apply_default_gains(RBF_PID_Handle *pid) {
    pid->KP = 0.051f;
    pid->KI = 0.0010f;
    pid->KD = 0.030f;
}
```

Initialize the new scales and diagnostics in `RBF_PID_Init(...)`:

```c
    pid->pressure_normalization_scale = 250.0f;
    pid->flow_normalization_scale = (pid->fMaxFlow > 0.0f) ? pid->fMaxFlow : 90.0f;
    pid->output_saturated = false;
    memset(pid->last_rbf_input, 0, sizeof(pid->last_rbf_input));
```

Add the setter implementation near `RBF_PID_SetPressureNormalization(...)`:

```c
void RBF_PID_SetFlowNormalization(RBF_PID_Handle *pid, float scale) {
    if (pid == NULL) {
        return;
    }
    pid->flow_normalization_scale = clamp_positive_or_default(
        scale,
        (pid->fMaxFlow > 0.0f) ? pid->fMaxFlow : 90.0f);
}
```

- [ ] **Step 2: Replace the current RBF input path with a historical vector and recover the physical Jacobian**

In `src/rbf_pid.c`, add these helpers above `rbf_pid_step_rbf_nn(...)`:

```c
static float rbf_pid_max_flow_output(const RBF_PID_Handle *pid) {
    float max_output = pid->fMaxFlow * pid->fFlowRateLimit;
    return max_output > 0.0f ? max_output : 90.0f;
}

static float rbf_pid_effective_flow_scale(const RBF_PID_Handle *pid) {
    return clamp_positive_or_default(pid->flow_normalization_scale,
                                     rbf_pid_max_flow_output(pid));
}

static float rbf_pid_effective_pressure_scale(const RBF_PID_Handle *pid) {
    return clamp_positive_or_default(pid->pressure_normalization_scale,
                                     MAX_PRESSURE);
}
```

Replace `rbf_pid_step_rbf_nn(...)` with:

```c
static void rbf_pid_step_rbf_nn(RBF_PID_Handle *pid) {
    float h[RBF_HNUM];
    float flow_scale = rbf_pid_effective_flow_scale(pid);
    float pressure_scale = rbf_pid_effective_pressure_scale(pid);
    float x[RBF_INPUT_DIM] = {
        pid->du_prev / flow_scale,
        pid->y_prev1 / pressure_scale,
        pid->y_prev2 / pressure_scale,
        pid->e_prev1 / pressure_scale
    };
    float y_n = pid->P_actual / pressure_scale;
    float y_hat_n = 0.0f;
    float jacobian_n = 0.0f;
    float error_rbf_n;
    int i;

    memcpy(pid->last_rbf_input, x, sizeof(x));
    pid->Jacobian = 0.0f;

    for (i = 0; i < RBF_HNUM; ++i) {
        float norm_val = 0.0f;
        int j;

        for (j = 0; j < RBF_INPUT_DIM; ++j) {
            float diff = x[j] - pid->c[i][j];
            norm_val += diff * diff;
        }

        h[i] = expf(-norm_val / (2.0f * pid->b_rbf[i] * pid->b_rbf[i]));
        y_hat_n += pid->w[i] * h[i];
        jacobian_n += pid->w[i] * h[i] * (pid->c[i][0] - x[0]) /
            (pid->b_rbf[i] * pid->b_rbf[i]);
    }

    pid->Jacobian = clampf(-5.0f,
        (pressure_scale / flow_scale) * jacobian_n,
        50.0f);
    error_rbf_n = y_n - y_hat_n;

    for (i = 0; i < RBF_HNUM; ++i) {
        float delta_w = pid->eta_w * error_rbf_n * h[i] +
            pid->alpha * (pid->w[i] - pid->w_1[i]);
        float width = pid->b_rbf[i];
        float width_sq = width * width;
        float width_cu = width_sq * width;
        float norm_val = 0.0f;
        int j;

        pid->w[i] += delta_w;

        for (j = 0; j < RBF_INPUT_DIM; ++j) {
            float delta_center = pid->eta_c * error_rbf_n * pid->w[i] * h[i] *
                (x[j] - pid->c[i][j]) / width_sq +
                pid->alpha * (pid->ci_1[i][j] - pid->ci_2[i][j]);
            pid->c[i][j] = clampf(-2.0f, pid->c[i][j] + delta_center, 2.0f);
        }

        for (j = 0; j < RBF_INPUT_DIM; ++j) {
            float diff = x[j] - pid->c[i][j];
            norm_val += diff * diff;
        }

        pid->b_rbf[i] = clampf(0.2f,
            pid->b_rbf[i] + pid->eta_b * error_rbf_n * pid->w[i] * h[i] *
            norm_val / width_cu +
            pid->alpha * (pid->bi_1[i] - pid->bi_2[i]),
            5.0f);
    }

    for (i = 0; i < RBF_HNUM; ++i) {
        int j;

        for (j = 0; j < RBF_INPUT_DIM; ++j) {
            pid->ci_2[i][j] = pid->ci_1[i][j];
            pid->ci_1[i][j] = pid->c[i][j];
        }
        pid->bi_2[i] = pid->bi_1[i];
        pid->bi_1[i] = pid->b_rbf[i];
        pid->w_2[i] = pid->w_1[i];
        pid->w_1[i] = pid->w[i];
    }
}
```

Update the call site in `RBF_PID_Update(...)` from:

```c
    rbf_pid_step_rbf_nn(pid, error);
```

to:

```c
    rbf_pid_step_rbf_nn(pid);
```

- [ ] **Step 3: Remove the speed-domain output path, replace it with a flow soft cap, and apply conservative learning restraint**

In `src/rbf_pid.c`, add this helper above `rbf_pid_step_incremental_output(...)`:

```c
static float rbf_pid_compute_soft_flow_cap(const RBF_PID_Handle *pid) {
    float hard_limit = rbf_pid_max_flow_output(pid);

    if (pid->K <= 0.0f || pid->P_set <= 0.0f) {
        return hard_limit;
    }

    return clampf(0.0f, (pid->P_set * 1.10f) / pid->K, hard_limit);
}
```

Replace `rbf_pid_step_adaptive_gains(...)` with:

```c
static void rbf_pid_step_adaptive_gains(RBF_PID_Handle *pid, float error) {
    float de = error - pid->e_prev1;
    float dde = de - (pid->e_prev1 - pid->e_prev2);
    float learning_scale = (fabsf(error) <= 1.0f) ? 0.20f : 1.0f;

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

Replace `rbf_pid_step_incremental_output(...)` with:

```c
static void rbf_pid_step_incremental_output(RBF_PID_Handle *pid, float error) {
    float hard_limit = rbf_pid_max_flow_output(pid);
    float flow_cap = rbf_pid_compute_soft_flow_cap(pid);
    float output_limit = (flow_cap < hard_limit) ? flow_cap : hard_limit;
    float raw_d_term = error - 2.0f * pid->e_prev1 + pid->e_prev2;
    float du = pid->KP * (error - pid->e_prev1) + pid->KI * error + pid->KD * raw_d_term;
    float actual_press = pid->P_set - error;
    float f_delta_press = actual_press - pid->fLastActPress;
    float f_dd_press = f_delta_press - (pid->fLastActPress - pid->fLastActPress2);
    float f_uff = pid->pressure_accel_ff_enabled ? (-0.5f * f_dd_press) : 0.0f;
    float ref_change = pid->P_set - pid->last_ref;
    float ref_rate = clampf(-10.0f, ref_change, 10.0f);
    float dynamic_ff = 0.02f * ref_rate;

    du += dynamic_ff + f_uff;

    pid->du = du;
    pid->Output = clampf(MIN_OUTPUT, pid->u_prev + du, output_limit);
    pid->output_saturated = (pid->Output <= MIN_OUTPUT + 1.0e-6f) ||
        (pid->Output >= output_limit - 1.0e-6f);
    pid->n_out = pid->Output;

    if (pid->P_set < 0.1f && actual_press < 0.5f) {
        pid->Output = 0.0f;
        pid->n_out = 0.0f;
        pid->output_saturated = false;
    }

    pid->fLastActPress2 = pid->fLastActPress;
    pid->fLastActPress = actual_press;
    pid->last_ref = pid->P_set;
}
```

Replace the end of `RBF_PID_Update(...)` with:

```c
    rbf_pid_step_rbf_nn(pid);
    rbf_pid_step_adaptive_gains(pid, error);
    rbf_pid_step_incremental_output(pid, error);

    pid->y_prev2 = pid->y_prev1;
    pid->y_prev1 = pid->P_actual;
    pid->e_prev2 = pid->e_prev1;
    pid->e_prev1 = error;
    pid->u_prev = pid->Output;
    pid->du_prev = pid->du;

    rbf_pid_step_steady_state(pid);
    pid->Status = pid->steady_state ? 3 : 2;
    return pid->Output;
```

Replace `rbf_pid_refresh_gain_compensation(...)` and `RBF_PID_SetGainCompensation(...)` with:

```c
static void rbf_pid_refresh_gain_compensation(RBF_PID_Handle *pid) {
    pid->gain_compensation_enabled = (pid->K > 0.0f);
    pid->gain_compensation_factor = 1.0f;
    pid->fGainCompensation = pid->K;
}

void RBF_PID_SetGainCompensation(RBF_PID_Handle *pid, float systemGain) {
    if (pid == NULL) {
        return;
    }

    pid->K = (systemGain > 0.0f) ? systemGain : 0.0f;
    rbf_pid_refresh_gain_compensation(pid);
}
```

- [ ] **Step 4: Run the unit-level migration checks until `test_rbf_pid` is green**

Run:

```bash
cmake --build --preset unixgcc --target rbf_pid_test
ctest --test-dir out/build/unixgcc -R '^test_rbf_pid$' --output-on-failure
```

Expected: `test_rbf_pid` passes with `n_out == Output == returned flow`, the causal-input telemetry assertions pass, and changing `flowToPumpSpeedGain` no longer changes the flow command.

- [ ] **Step 5: Commit the pure flow-domain RBF slice**

```bash
git add include/rbf_pid.h src/rbf_pid.c tests/rbf_pid_test.c
git commit -m "Move the adaptive pressure core fully into flow space" -m "Constraint: The RBF network, PID law, compensation, and limit handling must all stay in L/min until pump_converter
Rejected: Wrapping the old speed-domain controller with extra conversions | It would preserve the unit-mixing bug and keep the causal-input fix ambiguous
Confidence: medium
Scope-risk: moderate
Directive: Keep flowToPumpSpeedGain as outer-layer metadata only; do not let future edits reintroduce it into the control law
Tested: cmake --build --preset unixgcc --target rbf_pid_test; ctest --test-dir out/build/unixgcc -R '^test_rbf_pid$' --output-on-failure
Not-tested: pressure_controller and HIL targets not yet rerun in this slice"
```

### Task 3: Rewire `pressure_controller.c` around the flow-domain RBF contract

**Files:**
- Modify: `src/pressure_controller.c`
- Modify: `tests/test_pressure_controller.c`

- [ ] **Step 1: Tighten the integration test so it fails if `pressure_controller` still seeds or reads speed-domain state**

In `tests/test_pressure_controller.c`, add `pump_converter.h` to the includes:

```c
#include "pressure_controller.h"
#include "pump_converter.h"
```

Replace the local `pump_convert(...)` helper with:

```c
static HYD_REAL pump_convert(HYD_REAL flow_lmin) {
    HYD_PumpConverterInput input;
    HYD_PumpConverterOutput output;

    memset(&input, 0, sizeof(input));
    memset(&output, 0, sizeof(output));
    input.requestedFlow = flow_lmin;
    input.flowToPumpSpeedGain = PLANT_GAIN;
    input.pumpSpeedLimit = PLANT_PUMP_LIMIT;
    input.direction = HYD_DIRECTION_HOLD;

    HYD_PumpConverter_Execute(&input, &output);
    assert(fabs(output.pumpSpeed - output.commandFlow * PLANT_GAIN) < 1e-6);
    return output.pumpSpeed;
}
```

In `test_rbf_pid_strategy_executes_within_limits_and_adapts(...)`, append these assertions inside the loop after `HYD_PressureController_Execute(...)`:

```c
        assert(fabs((double)state.rbfPid.Output - (double)output.outputFlow) < 1e-6);
        assert(fabs((double)state.rbfPid.n_out - (double)output.outputFlow) < 1e-6);
        assert(fabs((double)state.rbfPid.u_prev - (double)output.outputFlow) < 1.0);
        assert(state.rbfPid.flow_normalization_scale > 0.0f);
        assert(state.rbfPid.pressure_normalization_scale > 0.0f);
```

In `test_rbf_pid_strategy_switch_tracks_previous_output_bumplessly(...)`, append:

```c
    assert(fabs((double)state.rbfPid.Output - (double)output1.outputFlow) < 0.05);
    assert(fabs((double)state.rbfPid.n_out - (double)output1.outputFlow) < 0.05);
```

- [ ] **Step 2: Run the pressure-controller target and confirm it still fails while synchronization seeds motor-domain values**

Run:

```bash
cmake --build --preset unixgcc --target test_pressure_controller
ctest --test-dir out/build/unixgcc -R '^test_pressure_controller$' --output-on-failure
```

Expected: the new `Output / n_out / u_prev` assertions fail until `HYD_SynchronizeRbfPidState(...)` and `HYD_ApplyRbfPidConfig(...)` are updated.

- [ ] **Step 3: Update the RBF integration path to seed and clamp only in flow space**

In `src/pressure_controller.c`, keep `HYD_EnsureRbfPidInitialized(...)` for `fMaxFlow` derivation, but after the existing `RBF_PID_SetPressureAccelFeedforwardEnabled(...)` call in `HYD_ApplyRbfPidConfig(...)`, add:

```c
    RBF_PID_SetFlowNormalization(
        &state->rbfPid,
        (float)HYD_ResolvePositiveOrDefault(config->outputMax,
                                            (HYD_REAL)state->rbfPid.fMaxFlow));
```

Replace the pressure-normalization block in `HYD_ApplyRbfPidConfig(...)` with:

```c
    {
        HYD_REAL pressureScale = 0.0;

        if (segment != NULL && segment->pressureCeiling > 0.0) {
            pressureScale = segment->pressureCeiling;
        } else if (segment != NULL && segment->targetPressure > 0.0) {
            HYD_REAL candidate = segment->targetPressure * 3.0;
            pressureScale = (candidate > (HYD_REAL)MAX_PRESSURE) ?
                candidate : (HYD_REAL)MAX_PRESSURE;
        }

        RBF_PID_SetPressureNormalization(&state->rbfPid, (float)pressureScale);
    }
```

Replace the `systemGain` block with:

```c
    if (segment != NULL && segment->systemGain > 0.0) {
        RBF_PID_SetGainCompensation(&state->rbfPid, (float)segment->systemGain);
    } else {
        RBF_PID_SetGainCompensation(&state->rbfPid, 0.0f);
    }
```

Replace `HYD_SynchronizeRbfPidState(...)` with a pure flow-domain seed:

```c
static void HYD_SynchronizeRbfPidState(HYD_PressureControllerState* state,
                                       HYD_REAL trackedOutputFlow,
                                       HYD_REAL targetPressure,
                                       HYD_REAL measuredPressure,
                                       const HYD_PressureResolvedConfig* config,
                                       const HYD_MotionSegment* segment,
                                       HYD_REAL flowToPumpSpeedGain,
                                       HYD_REAL pumpSpeedLimit) {
    HYD_REAL seededFlow;
    HYD_REAL error;

    if (state == NULL || config == NULL) {
        return;
    }

    RBF_PID_Reset(&state->rbfPid);
    HYD_ApplyRbfPidConfig(state, config, segment,
                          flowToPumpSpeedGain, pumpSpeedLimit);

    seededFlow = HYD_ClampReal(trackedOutputFlow, config->outputMin, config->outputMax);
    error = targetPressure - measuredPressure;

    state->rbfPid.Output = (float)seededFlow;
    state->rbfPid.u_prev = (float)seededFlow;
    state->rbfPid.n_out = (float)seededFlow;
    state->rbfPid.P_set = (float)targetPressure;
    state->rbfPid.P_actual = (float)measuredPressure;
    state->rbfPid.Error = (float)error;
    state->rbfPid.du = 0.0f;
    state->rbfPid.du_prev = 0.0f;
    state->rbfPid.e_prev1 = (float)error;
    state->rbfPid.e_prev2 = (float)error;
    state->rbfPid.y_prev1 = (float)measuredPressure;
    state->rbfPid.y_prev2 = (float)measuredPressure;
    state->rbfPid.fLastActPress = (float)measuredPressure;
    state->rbfPid.fLastActPress2 = (float)measuredPressure;
    state->rbfPid.last_ref = (float)targetPressure;
    state->rbfPid.output_saturated = false;
    state->rbfPid.Status = 1;
    state->rbfPid.TuneResult = 0;
}
```

Keep the outer safety clamp in `HYD_PressureController_Execute(...)`, but do not add any new `flow -> rpm` conversion there.

- [ ] **Step 4: Re-run the integrated pressure-loop targets**

Run:

```bash
cmake --build --preset unixgcc --target rbf_pid_test test_pressure_controller
ctest --test-dir out/build/unixgcc -R '^(test_rbf_pid|test_pressure_controller)$' --output-on-failure
```

Expected: unit tests and integration tests now agree that the RBF branch is carrying only flow-domain state.

- [ ] **Step 5: Commit the pressure-controller integration slice**

```bash
git add src/pressure_controller.c tests/test_pressure_controller.c
git commit -m "Align pressure_controller with the flow-domain RBF contract" -m "Constraint: The pressure loop must still derive max flow from pump metadata while keeping the adaptive state fully in L/min
Rejected: Moving pump conversion into pressure_controller | It would split the actuator mapping across two modules and break the single-conversion rule
Confidence: medium
Scope-risk: moderate
Directive: Keep the outer clamp in pressure_controller as a guard, but treat the primary RBF limit as an internal flow-domain responsibility
Tested: cmake --build --preset unixgcc --target rbf_pid_test test_pressure_controller; ctest --test-dir out/build/unixgcc -R '^(test_rbf_pid|test_pressure_controller)$' --output-on-failure
Not-tested: HIL path not yet rerun in this slice"
```

### Task 4: Retune the plant-model scenarios and verify the end-to-end converter path

**Files:**
- Modify: `tests/test_pressure_controller.c`
- Modify: `tests/test_rbf_pid_hil.c`

- [ ] **Step 1: Move the plant-model RBF tuning profile to flow-domain values**

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
    seg.pressureRbfConfig.minKp = 0.030;
    seg.pressureRbfConfig.maxKp = 0.090;
    seg.pressureRbfConfig.minKi = 0.0005;
    seg.pressureRbfConfig.maxKi = 0.0040;
    seg.pressureRbfConfig.minKd = 0.010;
    seg.pressureRbfConfig.maxKd = 0.080;
    seg.pressureRbfConfig.etaW = 0.005;
    seg.pressureRbfConfig.etaC = 0.005;
    seg.pressureRbfConfig.etaB = 0.005;
    seg.pressureRbfConfig.etaP = 0.00025;
    seg.pressureRbfConfig.etaI = 0.00025;
    seg.pressureRbfConfig.etaD = 0.00025;
    return seg;
}
```

Keep the existing acceptance checks in `test_rbf_pid_single_setpoint_plant_convergence(...)`:

```c
        assert(pressure_bar >= target * 0.98);
        assert(pressure_bar <= target * 1.02);
        assert(peak_pressure <= target * 1.05);
        assert(!oscillating);
```

and in `test_rbf_pid_setpoint_switching_plant(...)`:

```c
        assert(pressure_bar >= target * 0.95);
        assert(pressure_bar <= target * 1.10);
        assert(!spike_detected);
```

- [ ] **Step 2: Update the HIL test so the full stack proves it is using `HYD_PumpConverter_Execute(...)`**

In `tests/test_rbf_pid_hil.c`, add:

```c
#include "pump_converter.h"
```

Inside `hil_step_once(...)`, immediately after `HYD_MotionControlFB_Execute(fb);`, add:

```c
    if (fb->STATE.pressureLoop.adaptiveActive) {
        HYD_PumpConverterInput pump_input;
        HYD_PumpConverterOutput pump_output;

        memset(&pump_input, 0, sizeof(pump_input));
        memset(&pump_output, 0, sizeof(pump_output));
        pump_input.requestedFlow = fb->STATE.pressureLoop.outputFlow;
        pump_input.flowToPumpSpeedGain = fb->FLOW_TO_PUMP_SPEED_GAIN;
        pump_input.pumpSpeedLimit = fb->PUMP_SPEED_LIMIT;
        pump_input.direction = fb->STATE.plannedDirection;

        HYD_PumpConverter_Execute(&pump_input, &pump_output);
        assert(fabs((double)fb->PUMP_SPEED - (double)pump_output.pumpSpeed) < 1e-6);
    }
```

Keep Scenario A as a bounded/no-fault HIL gate rather than the `< 5%` tuning gate; the quantitative overshoot requirement stays on the plant-model test where the pressure dynamics are explicit and repeatable.

- [ ] **Step 3: Run the quantitative validation and tune only the constants from Step 1 if the acceptance gates miss**

Run:

```bash
cmake --build --preset unixgcc --target test_pressure_controller test_rbf_pid_hil
./out/build/unixgcc/test_pressure_controller
ctest --test-dir out/build/unixgcc -R '^test_rbf_pid_hil$' --output-on-failure
```

Expected: `test_pressure_controller` prints overshoot lines for `50 / 80 / 100 bar`, each at or below `5%`, and `test_rbf_pid_hil` remains fault-free while its pump-speed assertion proves the single conversion path.

If `test_rbf_pid_single_setpoint_plant_convergence(...)` still shows overshoot above `5%`, apply only this narrower damping patch and rerun:

```c
    seg.pressureRbfConfig.maxKp = 0.075;
    seg.pressureRbfConfig.maxKd = 0.060;
```

If the overshoot is already below `5%` but the final pressure is still below `98%` of target, apply only this narrower integral patch and rerun:

```c
    seg.pressureRbfConfig.maxKi = 0.0060;
```

and in `src/rbf_pid.c`:

```c
    pid->KI = 0.0015f;
```

Do not widen more than one branch at a time; rerun the same validation command after each adjustment so the overshoot and steady-state tradeoff stays attributable.

- [ ] **Step 4: Run the focused regression suite that must stay green before handoff**

Run:

```bash
cmake --build --preset unixgcc --target rbf_pid_test test_pressure_controller test_rbf_pid_hil test_pump_converter
ctest --test-dir out/build/unixgcc -R '^(test_rbf_pid|test_pressure_controller|test_rbf_pid_hil|test_pump_converter)$' --output-on-failure
```

Expected: all four focused targets pass, `test_pressure_controller` has already shown `< 5%` overshoot in its direct output, and no end-to-end path bypasses `HYD_PumpConverter_Execute(...)`.

- [ ] **Step 5: Commit the tuning and end-to-end verification slice**

```bash
git add tests/test_pressure_controller.c tests/test_rbf_pid_hil.c src/rbf_pid.c
git commit -m "Validate the flow-domain migration against plant and HIL loops" -m "Constraint: This phase closes only when the plant-model overshoot is below 5 percent and the pump-speed path is still centralized in pump_converter
Rejected: Treating the HIL sim as the primary overshoot oracle | Its pressure physics are intentionally looser than the focused plant-model regression
Confidence: medium
Scope-risk: moderate
Directive: Keep overshoot tuning changes confined to the flow-domain gain windows and learning rates unless a later phase explicitly changes the plant model
Tested: cmake --build --preset unixgcc --target rbf_pid_test test_pressure_controller test_rbf_pid_hil test_pump_converter; ./out/build/unixgcc/test_pressure_controller; ctest --test-dir out/build/unixgcc -R '^(test_rbf_pid|test_pressure_controller|test_rbf_pid_hil|test_pump_converter)$' --output-on-failure
Not-tested: Full repository suite not run in this slice"
```

## Self-Review

**Spec coverage**

- Pure flow-domain adaptive path: Task 2 Step 3 and Task 3 Step 3
- Causal RBF input and removal of current pressure: Task 2 Step 2
- Split flow / pressure normalization, including the `du_prev` vs `y_prev1` concern: Task 1 Step 1 and Task 2 Step 2
- Single `flow -> rpm` conversion point in `pump_converter`: Task 3 Step 1 and Task 4 Step 2
- Retuned `KP / KI / KD` semantics in flow space: Task 2 Step 1 and Task 4 Step 1
- Overshoot `< 5%`: Task 4 Step 3
- Deferred online `K` / Smith predictor: enforced by boundary rules, no task implements either feature

**Placeholder scan**

- No placeholder markers remain in the plan body.
- Every code-touching step includes a concrete code block.
- Every verification step includes an exact command and an expected result.

**Type consistency**

- The new public setter name is consistently `RBF_PID_SetFlowNormalization(...)`.
- The causal telemetry field is consistently `last_rbf_input`.
- Flow-domain state references use `Output`, `u_prev`, and `n_out` as `L/min` across the plan.

Plan complete and saved to `docs/superpowers/plans/2026-06-22-rbf-pid-flow-domain-migration.md`. Two execution options:

**1. Subagent-Driven (recommended)** - I dispatch a fresh subagent per task, review between tasks, fast iteration

**2. Inline Execution** - Execute tasks in this session using executing-plans, batch execution with checkpoints

**Which approach?**
