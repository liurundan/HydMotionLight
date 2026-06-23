# Pressure Model First-Order Switch Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Extend the pressure simulator so `HYD_PRESSUREMODEL` can switch online between the existing physical pressure model and a first-order delayed model without regressing the default physical path.

**Architecture:** Keep `PressureModel_Step(...)` as the single stateful plant boundary and add a model-type branch inside it. Preserve shared motor/time preprocessing for both models, store first-order delay history inside `PressureModelState`, and make runtime switching explicit by synchronizing pressure state before activating the new branch. Wire the new parameters through `HYD_PRESSUREMODEL` each scan so unit tests and PLC-adapter tests exercise the same code path.

**Tech Stack:** C99, HydroSimLib (`src/sim/*.c`), `tests/test_pressure_model.c`, `tests/test_hydro_sim_fb.c`, CMake/CTest, GNU `libm`

---

## File Map

| File | Action | Responsibility |
| --- | --- | --- |
| `include/pressure_model.h` | Modify | Add model-type enum, first-order parameters, and first-order state storage to the public pressure-model API |
| `src/sim/PressureModel.c` | Modify | Keep shared preprocessing, preserve the current physical branch, add the first-order branch, implement switch-state synchronization, and keep `pressure_update(...)` compatibility |
| `include/hydro_sim_fb.h` | Modify | Expose `MODEL_TYPE`, `K_NUM`, `TTAU`, and `DELAYTIME` on `HYD_PRESSUREMODEL` |
| `src/sim/hydro_sim_fb.c` | Modify | Copy the new FB inputs into `g_pressure_model_params` each scan and keep reset semantics unchanged |
| `tests/test_pressure_model.c` | Modify | Add first-order, fallback, and switch-continuity unit tests on the shared C API |
| `tests/test_hydro_sim_fb.c` | Modify | Add PLC-adapter tests for online switching, hot parameter updates, and reset behavior with the new inputs |

## Boundary Rules

- Do not change `CMakeLists.txt`; the relevant test targets already exist.
- Do not create a second pressure-model API. The new branch must live behind `PressureModel_Step(...)`.
- Do not change the `pressure_update(...)` signature.
- Do not change the physical-model math except to extract it behind helpers or wrap it behind model-type dispatch.
- Do not add dynamic allocation. The first-order delay buffer must be fixed-size.

---

### Task 1: Extend the public API and FB surface without changing default behavior

**Files:**
- Modify: `include/pressure_model.h`
- Modify: `include/hydro_sim_fb.h`
- Modify: `src/sim/PressureModel.c`
- Modify: `tests/test_pressure_model.c`
- Modify: `tests/test_hydro_sim_fb.c`

- [ ] **Step 1: Add failing tests that reference the new public fields**

In `tests/test_pressure_model.c`, insert this helper after `make_deterministic_params()`:

```c
static PressureModelParams make_first_order_params(float gain, float tau_s, float delay_s) {
    PressureModelParams params = make_deterministic_params();

    params.model_type = PRESSURE_MODEL_TYPE_FIRST_ORDER;
    params.first_order_k_bar_per_rpm = gain;
    params.first_order_tau_s = tau_s;
    params.first_order_delay_s = delay_s;
    params.sensor_range_bar = 10000.0f;
    params.motor_tau_s = 0.0f;

    return params;
}
```

Add this test after `test_zero_speed_holds_zero_pressure()`:

```c
static int test_init_params_default_to_physical_mode(void) {
    PressureModelParams params;
    PressureModelState state;

    PressureModel_InitParams(&params);
    PressureModel_Reset(&state, 0x51515151u);

    ASSERT_TRUE(params.model_type == PRESSURE_MODEL_TYPE_PHYSICAL);
    ASSERT_NEAR(params.first_order_k_bar_per_rpm, 0.0f, 1e-6f);
    ASSERT_NEAR(params.first_order_tau_s, 0.2f, 1e-6f);
    ASSERT_NEAR(params.first_order_delay_s, 0.0f, 1e-6f);
    ASSERT_TRUE(state.active_model_type == PRESSURE_MODEL_TYPE_PHYSICAL);
    ASSERT_NEAR(state.first_order_prev_pressure_bar, 0.0f, 1e-6f);
    ASSERT_TRUE(state.first_order_buffer_index == 0);

    return 1;
}
```

Wire it into `main()` immediately after `test_zero_speed_holds_zero_pressure()`:

```c
    if (test_init_params_default_to_physical_mode()) {
        ++passed;
        printf("PASS test_init_params_default_to_physical_mode\n");
    } else {
        ++failed;
        printf("FAIL test_init_params_default_to_physical_mode\n");
    }
```

In `tests/test_hydro_sim_fb.c`, add this test after `test_pressure_model_fb_persists_state_and_resets_on_disable()`:

```c
static void test_pressure_model_fb_exposes_first_order_inputs(void) {
    HYD_PRESSUREMODEL cmd;

    memset(&cmd, 0, sizeof(cmd));
    cmd.MODEL_TYPE.value = PRESSURE_MODEL_TYPE_FIRST_ORDER;
    cmd.K_NUM.value = 0.25;
    cmd.TTAU.value = 0.2;
    cmd.DELAYTIME.value = 0.01;

    ASSERT_TRUE(cmd.MODEL_TYPE.value == PRESSURE_MODEL_TYPE_FIRST_ORDER,
                "PressureModel FB should expose MODEL_TYPE");
    ASSERT_NEAR(cmd.K_NUM.value, 0.25, TOLERANCE,
                "PressureModel FB should expose K_NUM");
    ASSERT_NEAR(cmd.TTAU.value, 0.2, TOLERANCE,
                "PressureModel FB should expose TTAU");
    ASSERT_NEAR(cmd.DELAYTIME.value, 0.01, TOLERANCE,
                "PressureModel FB should expose DELAYTIME");
}
```

Call it in `main()` before `test_pressure_model_fb_negative_speed_depressurizes_without_reset();`:

```c
    test_pressure_model_fb_exposes_first_order_inputs();
```

- [ ] **Step 2: Run the focused build and confirm it fails on the missing API**

Run:

```bash
cmake --preset unixgcc
cmake --build --preset unixgcc --target test_pressure_model test_hydro_sim_fb
```

Expected: compilation fails with unknown identifiers such as `PRESSURE_MODEL_TYPE_FIRST_ORDER`, missing `model_type`, missing `first_order_k_bar_per_rpm`, and missing `MODEL_TYPE` / `K_NUM` / `TTAU` / `DELAYTIME`.

- [ ] **Step 3: Add the model-type enum, first-order params, first-order state, and FB inputs**

In `include/pressure_model.h`, add these definitions above `PressureModelParams`:

```c
#define PRESSURE_MODEL_FIRST_ORDER_MAX_DELAY_STEPS 1000

typedef enum {
    PRESSURE_MODEL_TYPE_PHYSICAL = 0u,
    PRESSURE_MODEL_TYPE_FIRST_ORDER = 1u
} PressureModelType;
```

Replace `PressureModelParams` with:

```c
typedef struct {
    float pump_displacement_m3_rev;
    float bulk_modulus_pa;
    float chamber_volume_m3;
    float leak_coeff_m3_pa_s;
    float relief_set_pa;
    float relief_coeff_m3_pa_s;
    float sensor_range_bar;
    float sensor_noise_std_bar;
    float sensor_bias_bar;
    float motor_tau_s;
    float motor_noise_std_rpm;
    float process_noise_std_m3_s;
    float flow_ripple_ratio;
    float tooth_drop_depth_ratio;
    float tooth_drop_width_ratio;
    float min_rpm;
    float max_rpm;
    unsigned char enable_sensor_noise;
    unsigned char enable_motor_noise;
    unsigned char enable_process_noise;
    float veff_base_m3;
    float leak_base_m3_pa_s;
    float tooth_drop_depth_base;
    float tooth_drop_phase_base;
    float veff_speed_scale[3];
    float leak_speed_scale[3];
    float drop_depth_scale[3];
    float drop_phase_offset[3];
    float torque_bias;
    float torque_from_pressure_gain;
    float torque_from_speed_gain;
    unsigned char model_type;
    float first_order_k_bar_per_rpm;
    float first_order_tau_s;
    float first_order_delay_s;
} PressureModelParams;
```

Replace `PressureModelState` with:

```c
typedef struct {
    float motor_rpm;
    float pressure_pa;
    float pump_phase_rev;
    uint32_t rng_state;
    int has_spare_gauss;
    float spare_gauss;
    unsigned char active_model_type;
    float first_order_prev_pressure_bar;
    int first_order_buffer_index;
    float first_order_delay_buffer[PRESSURE_MODEL_FIRST_ORDER_MAX_DELAY_STEPS];
} PressureModelState;
```

In `include/hydro_sim_fb.h`, extend `HYD_PRESSUREMODEL` like this:

```c
typedef struct {
    __DECLARE_VAR(BOOL,EN)
    __DECLARE_VAR(BOOL,ENO)
    __DECLARE_VAR(BOOL,ENABLE)
    __DECLARE_VAR(REAL,MOTOR_RPM)
    __DECLARE_VAR(REAL,TIME_S)
    __DECLARE_VAR(USINT,MODEL_TYPE)
    __DECLARE_VAR(REAL,K_NUM)
    __DECLARE_VAR(REAL,TTAU)
    __DECLARE_VAR(REAL,DELAYTIME)

    __DECLARE_VAR(BOOL,ACTIVE)
    __DECLARE_VAR(REAL,MEASURED_PRESSURE_BAR)
    __DECLARE_VAR(REAL,REAL_PRESSURE_BAR)
    __DECLARE_VAR(REAL,ACTUAL_MOTOR_RPM)
} HYD_PRESSUREMODEL;
```

- [ ] **Step 4: Initialize and reset the new fields without changing the default branch**

In `src/sim/PressureModel.c`, inside `PressureModel_InitParams(...)`, add:

```c
    params->model_type = PRESSURE_MODEL_TYPE_PHYSICAL;
    params->first_order_k_bar_per_rpm = 0.0f;
    params->first_order_tau_s = 0.2f;
    params->first_order_delay_s = 0.0f;
```

In `PressureModel_Reset(...)`, keep the existing `memset` and seed logic, then add:

```c
    state->active_model_type = PRESSURE_MODEL_TYPE_PHYSICAL;
    state->first_order_prev_pressure_bar = 0.0f;
    state->first_order_buffer_index = 0;
```

No behavior dispatch belongs in this task. The physical path must remain the only active branch after reset.

- [ ] **Step 5: Rebuild and run the targeted tests until the interface layer passes**

Run:

```bash
cmake --build --preset unixgcc --target test_pressure_model test_hydro_sim_fb
ctest --test-dir out/build/unixgcc -R '^(test_pressure_model|test_hydro_sim_fb)$' --output-on-failure
```

Expected: the new interface tests pass, and the existing physical-model tests still pass because `model_type` defaults to `PRESSURE_MODEL_TYPE_PHYSICAL`.

- [ ] **Step 6: Commit the interface slice**

```bash
git add include/pressure_model.h include/hydro_sim_fb.h src/sim/PressureModel.c tests/test_pressure_model.c tests/test_hydro_sim_fb.c
git commit -m "Expose first-order pressure-model controls without changing the default path

Constraint: HYD_PRESSUREMODEL needs online model-selection inputs while the physical model remains the default runtime branch
Rejected: Adding a second pressure-model API beside PressureModel_Step | It would split state ownership and make switch behavior harder to verify
Confidence: high
Scope-risk: narrow
Directive: Keep all first-order fields additive until the new branch has dedicated tests
Tested: ctest --test-dir out/build/unixgcc -R '^(test_pressure_model|test_hydro_sim_fb)$' --output-on-failure
Not-tested: Full repository ctest suite"
```

---

### Task 2: Add first-order branch tests from reset and implement the branch math

**Files:**
- Modify: `tests/test_pressure_model.c`
- Modify: `src/sim/PressureModel.c`

- [ ] **Step 1: Add failing first-order unit tests for zero input, direct gain, delay, and output semantics**

In `tests/test_pressure_model.c`, add these tests after `test_motor_state_is_continuous_across_steps()`:

```c
static int test_first_order_zero_input_holds_zero_pressure(void) {
    PressureModelParams params = make_first_order_params(0.5f, 0.2f, 0.0f);
    PressureModelState state;
    PressureModelOutput out;

    memset(&out, 0, sizeof(out));
    PressureModel_Reset(&state, 0x78787878u);

    run_steps(&params, &state, 0.0f, 200, DT_S, &out);

    ASSERT_NEAR(out.real_pressure_bar, 0.0f, PRESSURE_EPS);
    ASSERT_NEAR(out.measured_pressure_bar, 0.0f, PRESSURE_EPS);
    ASSERT_NEAR(out.actual_motor_rpm, 0.0f, RPM_EPS);

    return 1;
}

static int test_first_order_tau_zero_matches_gain_times_actual_rpm(void) {
    PressureModelParams params = make_first_order_params(0.25f, 0.0f, 0.0f);
    PressureModelState state;
    PressureModelOutput out;

    memset(&out, 0, sizeof(out));
    PressureModel_Reset(&state, 0x79797979u);
    PressureModel_Step(&params, &state, 120.0f, DT_S, &out);

    ASSERT_NEAR(out.real_pressure_bar,
                params.first_order_k_bar_per_rpm * out.actual_motor_rpm,
                1e-4f);
    ASSERT_NEAR(out.measured_pressure_bar, out.real_pressure_bar, 1e-6f);

    return 1;
}

static int test_first_order_delay_defers_visible_output(void) {
    PressureModelParams params = make_first_order_params(0.5f, 0.0f, 0.003f);
    PressureModelState state;
    PressureModelOutput out;

    memset(&out, 0, sizeof(out));
    PressureModel_Reset(&state, 0x7a7a7a7au);

    PressureModel_Step(&params, &state, 100.0f, DT_S, &out);
    ASSERT_NEAR(out.real_pressure_bar, 0.0f, 1e-6f);
    PressureModel_Step(&params, &state, 100.0f, DT_S, &out);
    ASSERT_NEAR(out.real_pressure_bar, 0.0f, 1e-6f);
    PressureModel_Step(&params, &state, 100.0f, DT_S, &out);
    ASSERT_NEAR(out.real_pressure_bar, 0.0f, 1e-6f);
    PressureModel_Step(&params, &state, 100.0f, DT_S, &out);
    ASSERT_TRUE(out.real_pressure_bar > 0.0f);

    return 1;
}

static int test_first_order_outputs_measured_equal_real_and_zero_flow_terms(void) {
    PressureModelParams params = make_first_order_params(0.1f, 0.2f, 0.0f);
    PressureModelState state;
    PressureModelOutput out;

    memset(&out, 0, sizeof(out));
    PressureModel_Reset(&state, 0x7b7b7b7bu);
    run_steps(&params, &state, 250.0f, 50, DT_S, &out);

    ASSERT_NEAR(out.measured_pressure_bar, out.real_pressure_bar, 1e-6f);
    ASSERT_NEAR(out.pump_flow_m3_s, 0.0f, 1e-6f);
    ASSERT_NEAR(out.net_flow_m3_s, 0.0f, 1e-6f);

    return 1;
}
```

Wire them into `main()` after `test_motor_state_is_continuous_across_steps()`:

```c
    if (test_first_order_zero_input_holds_zero_pressure()) {
        ++passed;
        printf("PASS test_first_order_zero_input_holds_zero_pressure\n");
    } else {
        ++failed;
        printf("FAIL test_first_order_zero_input_holds_zero_pressure\n");
    }

    if (test_first_order_tau_zero_matches_gain_times_actual_rpm()) {
        ++passed;
        printf("PASS test_first_order_tau_zero_matches_gain_times_actual_rpm\n");
    } else {
        ++failed;
        printf("FAIL test_first_order_tau_zero_matches_gain_times_actual_rpm\n");
    }

    if (test_first_order_delay_defers_visible_output()) {
        ++passed;
        printf("PASS test_first_order_delay_defers_visible_output\n");
    } else {
        ++failed;
        printf("FAIL test_first_order_delay_defers_visible_output\n");
    }

    if (test_first_order_outputs_measured_equal_real_and_zero_flow_terms()) {
        ++passed;
        printf("PASS test_first_order_outputs_measured_equal_real_and_zero_flow_terms\n");
    } else {
        ++failed;
        printf("FAIL test_first_order_outputs_measured_equal_real_and_zero_flow_terms\n");
    }
```

- [ ] **Step 2: Run the pressure-model test target and confirm the new behavior tests fail**

Run:

```bash
cmake --build --preset unixgcc --target test_pressure_model
ctest --test-dir out/build/unixgcc -R '^test_pressure_model$' --output-on-failure
```

Expected: the new first-order tests fail because `PressureModel_Step(...)` still executes only the physical branch.

- [ ] **Step 3: Add helper functions for model normalization, delay steps, history fill, and first-order execution**

In `src/sim/PressureModel.c`, add these helpers above `PressureModel_Step(...)`:

```c
static unsigned char pressure_model_normalize_type(unsigned char model_type) {
    return (model_type == PRESSURE_MODEL_TYPE_FIRST_ORDER)
               ? PRESSURE_MODEL_TYPE_FIRST_ORDER
               : PRESSURE_MODEL_TYPE_PHYSICAL;
}

static void pressure_model_fill_first_order_history(PressureModelState *state, float pressure_bar) {
    int i;

    for (i = 0; i < PRESSURE_MODEL_FIRST_ORDER_MAX_DELAY_STEPS; ++i) {
        state->first_order_delay_buffer[i] = pressure_bar;
    }
    state->first_order_buffer_index = 0;
}

static int pressure_model_first_order_delay_steps(float delay_s, float dt_s) {
    float clamped_delay = pressure_model_clampf(delay_s, 0.0f, 1.0f);
    float ratio = clamped_delay / dt_s;
    int steps = (int)ratio;

    if (steps < 0) {
        steps = 0;
    }
    if (steps >= PRESSURE_MODEL_FIRST_ORDER_MAX_DELAY_STEPS) {
        steps = PRESSURE_MODEL_FIRST_ORDER_MAX_DELAY_STEPS - 1;
    }
    return steps;
}

static void pressure_model_step_first_order(const PressureModelParams *params,
                                            PressureModelState *state,
                                            float dt,
                                            float abs_motor_rpm,
                                            PressureModelOutput *out) {
    float gain = pressure_model_maxf(0.0f, params->first_order_k_bar_per_rpm);
    float tau = pressure_model_maxf(0.0f, params->first_order_tau_s);
    float undelayed_bar;
    float delayed_bar;
    int delay_steps;

    if (tau > 0.0f) {
        undelayed_bar = ((gain * state->motor_rpm * dt) +
                         (tau * state->first_order_prev_pressure_bar)) /
                        (tau + dt);
    } else {
        undelayed_bar = gain * state->motor_rpm;
    }

    undelayed_bar = pressure_model_clampf(undelayed_bar, 0.0f, 250.0f);
    delay_steps = pressure_model_first_order_delay_steps(params->first_order_delay_s, dt);

    state->first_order_delay_buffer[state->first_order_buffer_index] = undelayed_bar;
    if (delay_steps == 0) {
        delayed_bar = undelayed_bar;
    } else {
        int read_index = (state->first_order_buffer_index +
                          PRESSURE_MODEL_FIRST_ORDER_MAX_DELAY_STEPS -
                          delay_steps) %
                         PRESSURE_MODEL_FIRST_ORDER_MAX_DELAY_STEPS;
        delayed_bar = state->first_order_delay_buffer[read_index];
    }

    state->first_order_buffer_index =
        (state->first_order_buffer_index + 1) % PRESSURE_MODEL_FIRST_ORDER_MAX_DELAY_STEPS;
    state->first_order_prev_pressure_bar = undelayed_bar;
    state->pressure_pa = delayed_bar * 1.0e5f;

    out->actual_motor_rpm = state->motor_rpm;
    out->real_pressure_bar = delayed_bar;
    out->measured_pressure_bar = delayed_bar;
    out->pump_flow_m3_s = 0.0f;
    out->net_flow_m3_s = 0.0f;
    out->relief_active = (undelayed_bar >= 250.0f) ? 1 : 0;
    out->estimated_torque_trend = pressure_model_maxf(
        0.0f,
        params->torque_bias +
            params->torque_from_pressure_gain * out->measured_pressure_bar +
            params->torque_from_speed_gain * abs_motor_rpm);
}
```

- [ ] **Step 4: Branch `PressureModel_Step(...)` into physical or first-order execution**

In `PressureModel_Step(...)`, after shared preprocessing and `abs_motor_rpm` calculation, insert this early branch:

```c
    if (pressure_model_normalize_type(params->model_type) == PRESSURE_MODEL_TYPE_FIRST_ORDER) {
        pressure_model_step_first_order(params, state, dt, abs_motor_rpm, out);
        state->active_model_type = PRESSURE_MODEL_TYPE_FIRST_ORDER;
        return;
    }
```

Leave the existing physical-model body in place after this branch. At the end of the physical path, set:

```c
    state->active_model_type = PRESSURE_MODEL_TYPE_PHYSICAL;
```

This task does not implement switch continuity yet. It only makes the first-order branch work from reset.

- [ ] **Step 5: Run the pressure-model target until the first-order-from-reset tests pass**

Run:

```bash
cmake --build --preset unixgcc --target test_pressure_model
ctest --test-dir out/build/unixgcc -R '^test_pressure_model$' --output-on-failure
```

Expected: the new first-order tests pass, and existing physical tests continue to pass because `model_type` still defaults to `PRESSURE_MODEL_TYPE_PHYSICAL`.

- [ ] **Step 6: Commit the first-order branch slice**

```bash
git add src/sim/PressureModel.c tests/test_pressure_model.c
git commit -m "Add the first-order pressure branch behind the shared model API

Constraint: The first-order branch must live behind PressureModel_Step and use fixed-size delay history
Rejected: Implementing the branch in HYD_PRESSUREMODEL only | It would bypass the C API and weaken unit-test coverage
Confidence: high
Scope-risk: moderate
Directive: Keep switch synchronization separate from branch math so continuity logic stays reviewable
Tested: ctest --test-dir out/build/unixgcc -R '^test_pressure_model$' --output-on-failure
Not-tested: HYD_PRESSUREMODEL adapter path"
```

---

### Task 3: Add switch continuity and invalid-model fallback tests, then implement switch-state synchronization

**Files:**
- Modify: `tests/test_pressure_model.c`
- Modify: `src/sim/PressureModel.c`

- [ ] **Step 1: Add failing unit tests for fallback and online switch continuity**

In `tests/test_pressure_model.c`, add these tests after `test_first_order_outputs_measured_equal_real_and_zero_flow_terms()`:

```c
static int test_invalid_model_type_matches_physical_branch(void) {
    PressureModelParams physical_params = make_deterministic_params();
    PressureModelParams invalid_params = physical_params;
    PressureModelState physical_state;
    PressureModelState invalid_state;
    PressureModelOutput physical_out;
    PressureModelOutput invalid_out;
    int i;

    invalid_params.model_type = 99u;
    memset(&physical_out, 0, sizeof(physical_out));
    memset(&invalid_out, 0, sizeof(invalid_out));
    PressureModel_Reset(&physical_state, 0x7c7c7c7cu);
    PressureModel_Reset(&invalid_state, 0x7c7c7c7cu);

    for (i = 0; i < 500; ++i) {
        PressureModel_Step(&physical_params, &physical_state, 40.0f, DT_S, &physical_out);
        PressureModel_Step(&invalid_params, &invalid_state, 40.0f, DT_S, &invalid_out);
    }

    ASSERT_NEAR(invalid_out.real_pressure_bar, physical_out.real_pressure_bar, 1e-6f);
    ASSERT_NEAR(invalid_out.measured_pressure_bar, physical_out.measured_pressure_bar, 1e-6f);
    ASSERT_NEAR(invalid_out.actual_motor_rpm, physical_out.actual_motor_rpm, 1e-6f);

    return 1;
}

static int test_switch_from_physical_to_first_order_preserves_pressure(void) {
    PressureModelParams params = make_deterministic_params();
    PressureModelState state;
    PressureModelOutput out;
    float charged_pressure;

    memset(&out, 0, sizeof(out));
    PressureModel_Reset(&state, 0x7d7d7d7du);
    run_steps(&params, &state, 40.0f, 12000, DT_S, &out);
    charged_pressure = out.real_pressure_bar;

    params.model_type = PRESSURE_MODEL_TYPE_FIRST_ORDER;
    params.first_order_k_bar_per_rpm = 1.0f;
    params.first_order_tau_s = 0.2f;
    params.first_order_delay_s = 0.0f;
    PressureModel_Step(&params, &state, 40.0f, DT_S, &out);

    ASSERT_NEAR(out.real_pressure_bar, charged_pressure, 1e-6f);
    ASSERT_NEAR(out.measured_pressure_bar, charged_pressure, 1e-6f);
    ASSERT_TRUE(state.active_model_type == PRESSURE_MODEL_TYPE_FIRST_ORDER);

    return 1;
}

static int test_switch_from_first_order_to_physical_preserves_pressure(void) {
    PressureModelParams params = make_first_order_params(0.5f, 0.2f, 0.0f);
    PressureModelState state;
    PressureModelOutput out;
    float charged_pressure;

    memset(&out, 0, sizeof(out));
    PressureModel_Reset(&state, 0x7e7e7e7eu);
    run_steps(&params, &state, 200.0f, 400, DT_S, &out);
    charged_pressure = out.real_pressure_bar;

    params = make_deterministic_params();
    PressureModel_Step(&params, &state, 200.0f, DT_S, &out);

    ASSERT_NEAR(out.real_pressure_bar, charged_pressure, 1e-6f);
    ASSERT_NEAR(out.measured_pressure_bar, charged_pressure, 1e-6f);
    ASSERT_TRUE(state.active_model_type == PRESSURE_MODEL_TYPE_PHYSICAL);

    return 1;
}
```

Wire them into `main()` after `test_first_order_outputs_measured_equal_real_and_zero_flow_terms()`:

```c
    if (test_invalid_model_type_matches_physical_branch()) {
        ++passed;
        printf("PASS test_invalid_model_type_matches_physical_branch\n");
    } else {
        ++failed;
        printf("FAIL test_invalid_model_type_matches_physical_branch\n");
    }

    if (test_switch_from_physical_to_first_order_preserves_pressure()) {
        ++passed;
        printf("PASS test_switch_from_physical_to_first_order_preserves_pressure\n");
    } else {
        ++failed;
        printf("FAIL test_switch_from_physical_to_first_order_preserves_pressure\n");
    }

    if (test_switch_from_first_order_to_physical_preserves_pressure()) {
        ++passed;
        printf("PASS test_switch_from_first_order_to_physical_preserves_pressure\n");
    } else {
        ++failed;
        printf("FAIL test_switch_from_first_order_to_physical_preserves_pressure\n");
    }
```

- [ ] **Step 2: Run the pressure-model target and confirm the continuity tests fail**

Run:

```bash
cmake --build --preset unixgcc --target test_pressure_model
ctest --test-dir out/build/unixgcc -R '^test_pressure_model$' --output-on-failure
```

Expected: the invalid-model test may already pass if normalization falls back to physical, but the two switch-continuity tests should fail because the switching scan still executes the new branch immediately instead of preserving the current pressure observation.

- [ ] **Step 3: Add explicit switch helpers and a one-scan continuity hold**

In `src/sim/PressureModel.c`, add this helper near the first-order helpers:

```c
static void pressure_model_write_switch_hold_output(const PressureModelParams *params,
                                                    PressureModelState *state,
                                                    float abs_motor_rpm,
                                                    float preserved_pressure_bar,
                                                    int first_order_mode,
                                                    PressureModelOutput *out) {
    if (first_order_mode) {
        state->first_order_prev_pressure_bar = preserved_pressure_bar;
        pressure_model_fill_first_order_history(state, preserved_pressure_bar);
    }

    state->pressure_pa = preserved_pressure_bar * 1.0e5f;
    out->actual_motor_rpm = state->motor_rpm;
    out->real_pressure_bar = preserved_pressure_bar;
    out->measured_pressure_bar = preserved_pressure_bar;
    out->pump_flow_m3_s = 0.0f;
    out->net_flow_m3_s = 0.0f;
    out->relief_active = 0;
    out->estimated_torque_trend = pressure_model_maxf(
        0.0f,
        params->torque_bias +
            params->torque_from_pressure_gain * out->measured_pressure_bar +
            params->torque_from_speed_gain * abs_motor_rpm);
}
```

- [ ] **Step 4: Normalize model type once per scan and return preserved pressure on the switching scan**

In `PressureModel_Step(...)`, after shared preprocessing and `abs_motor_rpm`, replace the direct branch check with:

```c
    {
        unsigned char requested_type = pressure_model_normalize_type(params->model_type);
        float preserved_pressure_bar = state->pressure_pa * 1.0e-5f;

        if (requested_type != state->active_model_type) {
            pressure_model_write_switch_hold_output(params,
                                                    state,
                                                    abs_motor_rpm,
                                                    preserved_pressure_bar,
                                                    requested_type == PRESSURE_MODEL_TYPE_FIRST_ORDER,
                                                    out);
            state->active_model_type = requested_type;
            return;
        }

        if (requested_type == PRESSURE_MODEL_TYPE_FIRST_ORDER) {
            pressure_model_step_first_order(params, state, dt, abs_motor_rpm, out);
            state->active_model_type = PRESSURE_MODEL_TYPE_FIRST_ORDER;
            return;
        }
    }
```

This makes the switching scan a pressure-hold scan. The next scan uses the newly selected branch dynamics.

- [ ] **Step 5: Run the pressure-model target until fallback and switch continuity pass**

Run:

```bash
cmake --build --preset unixgcc --target test_pressure_model
ctest --test-dir out/build/unixgcc -R '^test_pressure_model$' --output-on-failure
```

Expected: all new first-order, fallback, and switch tests pass together with the pre-existing physical-model regressions.

- [ ] **Step 6: Commit the switch-synchronization slice**

```bash
git add src/sim/PressureModel.c tests/test_pressure_model.c
git commit -m "Preserve pressure continuity when switching pressure-model branches

Constraint: Online model changes must not drop pressure to zero or fork a third fallback path
Rejected: Recomputing the new branch on the switch scan | It breaks the approved continuity contract
Confidence: high
Scope-risk: moderate
Directive: Keep switch handling explicit and preserve the current pressure observation on the transition scan
Tested: ctest --test-dir out/build/unixgcc -R '^test_pressure_model$' --output-on-failure
Not-tested: HYD_PRESSUREMODEL adapter path"
```

---

### Task 4: Wire the new FB inputs into the shared API and lock online adapter behavior with tests

**Files:**
- Modify: `src/sim/hydro_sim_fb.c`
- Modify: `tests/test_hydro_sim_fb.c`

- [ ] **Step 1: Add failing FB tests for first-order mode, online switch continuity, and hot parameter updates**

In `tests/test_hydro_sim_fb.c`, add this helper above the pressure-model FB tests:

```c
static void reset_pressure_model_cmd(HYD_PRESSUREMODEL *cmd) {
    memset(cmd, 0, sizeof(*cmd));
    cmd->MODEL_TYPE.value = PRESSURE_MODEL_TYPE_PHYSICAL;
    cmd->K_NUM.value = 0.0;
    cmd->TTAU.value = 0.2;
    cmd->DELAYTIME.value = 0.0;
}
```

Replace the start of `test_pressure_model_fb_persists_state_and_resets_on_disable()`:

```c
    reset_pressure_model_cmd(&cmd);
```

Add these new tests after `test_pressure_model_fb_negative_speed_depressurizes_without_reset()`:

```c
static void test_pressure_model_fb_first_order_mode_outputs_equal_real_and_measured(void) {
    HYD_PRESSUREMODEL cmd;
    int i;

    reset_pressure_model_cmd(&cmd);
    cmd.ENABLE.value = false;
    cmd.TIME_S.value = 0.0;
    __mcl_cmd_updatePressureModel(&cmd);

    for (i = 0; i < 300; ++i) {
        cmd.ENABLE.value = true;
        cmd.MODEL_TYPE.value = PRESSURE_MODEL_TYPE_FIRST_ORDER;
        cmd.K_NUM.value = 0.20;
        cmd.TTAU.value = 0.0;
        cmd.DELAYTIME.value = 0.0;
        cmd.MOTOR_RPM.value = 120.0;
        cmd.TIME_S.value = 0.001 * (double)i;
        __mcl_cmd_updatePressureModel(&cmd);
    }

    ASSERT_TRUE(cmd.ACTIVE.value,
                "First-order PressureModel FB should stay active while enabled");
    ASSERT_TRUE(cmd.REAL_PRESSURE_BAR.value > 0.0,
                "First-order PressureModel FB should build pressure for positive rpm");
    ASSERT_NEAR(cmd.REAL_PRESSURE_BAR.value, cmd.MEASURED_PRESSURE_BAR.value, TOLERANCE,
                "First-order PressureModel FB should report equal real and measured pressure");
}

static void test_pressure_model_fb_online_model_switch_preserves_pressure(void) {
    HYD_PRESSUREMODEL cmd;
    double charged_pressure;
    int i;

    reset_pressure_model_cmd(&cmd);
    cmd.ENABLE.value = false;
    cmd.TIME_S.value = 0.0;
    __mcl_cmd_updatePressureModel(&cmd);

    for (i = 0; i < 12000; ++i) {
        cmd.ENABLE.value = true;
        cmd.MODEL_TYPE.value = PRESSURE_MODEL_TYPE_PHYSICAL;
        cmd.MOTOR_RPM.value = 40.0;
        cmd.TIME_S.value = 0.001 * (double)i;
        __mcl_cmd_updatePressureModel(&cmd);
    }
    charged_pressure = cmd.REAL_PRESSURE_BAR.value;

    cmd.MODEL_TYPE.value = PRESSURE_MODEL_TYPE_FIRST_ORDER;
    cmd.K_NUM.value = 1.0;
    cmd.TTAU.value = 0.2;
    cmd.DELAYTIME.value = 0.0;
    cmd.TIME_S.value = 12.000;
    __mcl_cmd_updatePressureModel(&cmd);

    ASSERT_TRUE(cmd.ACTIVE.value,
                "Switching model type online should keep the PressureModel FB active");
    ASSERT_NEAR(cmd.REAL_PRESSURE_BAR.value, charged_pressure, 1e-6,
                "Switching to first-order mode should preserve the current real pressure");
    ASSERT_NEAR(cmd.MEASURED_PRESSURE_BAR.value, charged_pressure, 1e-6,
                "Switching to first-order mode should preserve the current measured pressure");
}

static void test_pressure_model_fb_hot_updates_first_order_parameters(void) {
    HYD_PRESSUREMODEL cmd;
    double low_gain_pressure;
    int i;

    reset_pressure_model_cmd(&cmd);
    cmd.ENABLE.value = false;
    cmd.TIME_S.value = 0.0;
    __mcl_cmd_updatePressureModel(&cmd);

    for (i = 0; i < 500; ++i) {
        cmd.ENABLE.value = true;
        cmd.MODEL_TYPE.value = PRESSURE_MODEL_TYPE_FIRST_ORDER;
        cmd.K_NUM.value = 0.05;
        cmd.TTAU.value = 0.0;
        cmd.DELAYTIME.value = 0.0;
        cmd.MOTOR_RPM.value = 200.0;
        cmd.TIME_S.value = 0.001 * (double)i;
        __mcl_cmd_updatePressureModel(&cmd);
    }
    low_gain_pressure = cmd.REAL_PRESSURE_BAR.value;

    for (i = 500; i < 1000; ++i) {
        cmd.K_NUM.value = 0.10;
        cmd.TIME_S.value = 0.001 * (double)i;
        __mcl_cmd_updatePressureModel(&cmd);
    }

    ASSERT_TRUE(cmd.REAL_PRESSURE_BAR.value > low_gain_pressure + 5.0,
                "Increasing K_NUM online should raise the first-order pressure output");
}
```

Call them in `main()` after `test_pressure_model_fb_negative_speed_depressurizes_without_reset();`:

```c
    test_pressure_model_fb_first_order_mode_outputs_equal_real_and_measured();
    test_pressure_model_fb_online_model_switch_preserves_pressure();
    test_pressure_model_fb_hot_updates_first_order_parameters();
```

- [ ] **Step 2: Run the FB target and confirm these new tests fail**

Run:

```bash
cmake --build --preset unixgcc --target test_hydro_sim_fb
ctest --test-dir out/build/unixgcc -R '^test_hydro_sim_fb$' --output-on-failure
```

Expected: the new tests fail because `__mcl_cmd_updatePressureModel(...)` still ignores `MODEL_TYPE`, `K_NUM`, `TTAU`, and `DELAYTIME`.

- [ ] **Step 3: Copy the new FB inputs into `g_pressure_model_params` each scan**

In `src/sim/hydro_sim_fb.c`, inside `__mcl_cmd_updatePressureModel(...)`, after reading `current_time` and `target_motor_speed`, add:

```c
    g_pressure_model_params.model_type = (unsigned char)__GET_VAR(data__->MODEL_TYPE);
    g_pressure_model_params.first_order_k_bar_per_rpm = (float)__GET_VAR(data__->K_NUM);
    g_pressure_model_params.first_order_tau_s = (float)__GET_VAR(data__->TTAU);
    g_pressure_model_params.first_order_delay_s = (float)__GET_VAR(data__->DELAYTIME);
```

Do not change `PressureModelFb_ResetState()` or `PressureModelFb_ResetOutputs(...)`. Reset semantics stay the same.

- [ ] **Step 4: Run the adapter tests until the new FB behaviors pass**

Run:

```bash
cmake --build --preset unixgcc --target test_hydro_sim_fb test_pressure_model
ctest --test-dir out/build/unixgcc -R '^(test_hydro_sim_fb|test_pressure_model)$' --output-on-failure
```

Expected: FB tests now pass, and pressure-model unit tests continue to pass because both surfaces exercise the same shared model code.

- [ ] **Step 5: Commit the FB-wiring slice**

```bash
git add include/hydro_sim_fb.h src/sim/hydro_sim_fb.c tests/test_hydro_sim_fb.c
git commit -m "Wire HYD_PRESSUREMODEL inputs into the shared first-order pressure branch

Constraint: PLC callers must control model selection and first-order tuning online without changing reset semantics
Rejected: Hiding first-order tuning inside g_pressure_model_params defaults | It would make online switching untestable from the PLC surface
Confidence: high
Scope-risk: narrow
Directive: Keep HYD_PRESSUREMODEL as a thin adapter that copies inputs and delegates all branch logic to PressureModel_Step
Tested: ctest --test-dir out/build/unixgcc -R '^(test_hydro_sim_fb|test_pressure_model)$' --output-on-failure
Not-tested: Broader HydroSimLib regressions"
```

---

### Task 5: Run the relevant regressions and finish with a clean verification pass

**Files:**
- Modify: none unless verification exposes a real bug

- [ ] **Step 1: Rebuild the relevant simulator targets from scratch**

Run:

```bash
cmake --build --preset unixgcc --target test_pressure_model test_hydro_sim_fb test_pressure_hold_diagnosis
```

Expected: all three targets build successfully.

- [ ] **Step 2: Run the focused regression set for the changed behavior surface**

Run:

```bash
ctest --test-dir out/build/unixgcc -R '^(test_pressure_model|test_hydro_sim_fb|test_pressure_hold_diagnosis)$' --output-on-failure
```

Expected: all targeted tests pass, covering the shared pressure-model API, the PLC adapter, and the hold-diagnosis integration that depends on the pressure-model contract.

- [ ] **Step 3: Run the full repository CTest suite**

Run:

```bash
ctest --test-dir out/build/unixgcc --output-on-failure
```

Expected: the full suite passes. If a failure appears, fix it in a new commit rather than amending earlier commits.

- [ ] **Step 4: Record the final verification evidence**

Capture these command results in the implementation handoff or final summary:

```text
cmake --build --preset unixgcc --target test_pressure_model test_hydro_sim_fb test_pressure_hold_diagnosis
ctest --test-dir out/build/unixgcc -R '^(test_pressure_model|test_hydro_sim_fb|test_pressure_hold_diagnosis)$' --output-on-failure
ctest --test-dir out/build/unixgcc --output-on-failure
```

Use the exact test names in the final summary so future reviewers can match the verification to the changed behavior surface.
