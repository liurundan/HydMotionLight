# Open-Loop Pressure Model Fit Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Retune `src/sim/PressureModel.c` so it approximates the measured `10 / 20 / 30 / 40 rpm` open-loop pressure data, preserves the 13-tooth ripple signature, keeps negative-speed depressurization support, and remains usable for later closed-loop controller work.

**Architecture:** Keep one stateful physical pressure model and add only four low-degree speed-dependent correction hooks: effective volume, leakage, tooth-drop depth, and tooth-drop phase. Distill the measured CSV into compact test fixtures inside `tests/fixtures/` so implementation and regression tests do not depend on the raw untracked CSV file. Preserve the existing `PressureModel_Step(...)`, `pressure_update(...)`, and `HYD_PRESSUREMODEL` integration surface; only the model internals and unit-test expectations change.

**Tech Stack:** C99, HydroSimLib (`src/sim/*.c`), existing `tests/test_pressure_model.c` and `tests/test_hydro_sim_fb.c`, CMake/CTest, GNU `libm`

---

## File Map

| File | Action | Responsibility |
| --- | --- | --- |
| `include/pressure_model.h` | Modify | Expand parameter/output structs with the tuning hooks and torque-trend output required by the approved spec |
| `src/sim/PressureModel.c` | Modify | Implement interpolation-based speed corrections, tuned defaults, tooth-phase shaping, torque trend, and keep `pressure_update(...)` compatibility |
| `tests/fixtures/pressure_model_open_loop_reference.h` | Create | Store compact reference metrics extracted from `open10203040-positive.csv` so tests stay repo-local and deterministic |
| `tests/test_pressure_model.c` | Modify | Replace the outdated `10 rpm -> 40 bar` expectation with measured open-loop envelope, tooth-phase, and torque-trend regressions |
| `tests/test_hydro_sim_fb.c` | Modify | Lock FB-level persistence, reset, and negative-speed depressurization behavior against the tuned model |

## Boundary Rules

- Do not add a standalone calibration tool.
- Do not make `PressureModel.c` branch into independent per-speed pressure curves.
- Do not change `src/sim/hydro_sim.c`.
- Do not add new dependencies.
- Keep `pressure_update(...)` callable with its current signature.

---

### Task 1: Extend the public pressure-model surface for fitted open-loop tuning

**Files:**
- Modify: `include/pressure_model.h`
- Modify: `src/sim/PressureModel.c`
- Modify: `tests/test_pressure_model.c`

- [ ] **Step 1: Write the failing test that demands the new tuning hooks and torque output**

In `tests/test_pressure_model.c`, add this test right after `test_zero_speed_holds_zero_pressure()`:

```c
static int test_init_params_expose_open_loop_fit_knobs(void) {
    PressureModelParams params;
    PressureModelState state;
    PressureModelOutput out;

    memset(&out, 0, sizeof(out));
    PressureModel_InitParams(&params);
    PressureModel_Reset(&state, 0x51515151u);
    PressureModel_Step(&params, &state, 0.0f, DT_S, &out);

    ASSERT_NEAR(params.veff_speed_scale[0], 1.0f, 1e-6f);
    ASSERT_NEAR(params.veff_speed_scale[1], 1.0f, 1e-6f);
    ASSERT_NEAR(params.veff_speed_scale[2], 1.0f, 1e-6f);
    ASSERT_NEAR(params.leak_speed_scale[0], 1.0f, 1e-6f);
    ASSERT_NEAR(params.drop_depth_scale[0], 1.0f, 1e-6f);
    ASSERT_NEAR(params.drop_phase_offset[0], 0.0f, 1e-6f);
    ASSERT_NEAR(out.estimated_torque_trend, 0.0f, 1e-4f);

    return 1;
}
```

Wire it into `main()` before `test_ten_rpm_converges_to_forty_bar()`:

```c
    if (test_init_params_expose_open_loop_fit_knobs()) {
        ++passed;
        printf("PASS test_init_params_expose_open_loop_fit_knobs\n");
    } else {
        ++failed;
        printf("FAIL test_init_params_expose_open_loop_fit_knobs\n");
    }
```

- [ ] **Step 2: Run the targeted build and confirm it fails because the new fields do not exist yet**

Run:

```bash
cmake --preset unixgcc
cmake --build --preset unixgcc --target test_pressure_model
```

Expected: compilation fails with unknown members such as `veff_speed_scale`, `leak_speed_scale`, `drop_depth_scale`, `drop_phase_offset`, and `estimated_torque_trend`.

- [ ] **Step 3: Expand `PressureModelParams` and `PressureModelOutput` in `include/pressure_model.h`**

Replace the existing `PressureModelParams` and `PressureModelOutput` definitions with:

```c
typedef struct {
    float pump_displacement_m3_rev;
    float bulk_modulus_pa;
    float veff_base_m3;
    float leak_base_m3_pa_s;
    float relief_set_pa;
    float relief_coeff_m3_pa_s;
    float sensor_range_bar;
    float sensor_noise_std_bar;
    float sensor_bias_bar;
    float motor_tau_s;
    float motor_noise_std_rpm;
    float process_noise_std_m3_s;
    float flow_ripple_ratio;
    float tooth_drop_depth_base;
    float tooth_drop_width_ratio;
    float tooth_drop_phase_base;
    float veff_speed_scale[3];
    float leak_speed_scale[3];
    float drop_depth_scale[3];
    float drop_phase_offset[3];
    float torque_bias;
    float torque_from_pressure_gain;
    float torque_from_speed_gain;
    float min_rpm;
    float max_rpm;
    unsigned char enable_sensor_noise;
    unsigned char enable_motor_noise;
    unsigned char enable_process_noise;
} PressureModelParams;

typedef struct {
    float measured_pressure_bar;
    float real_pressure_bar;
    float actual_motor_rpm;
    float pump_flow_m3_s;
    float net_flow_m3_s;
    float estimated_torque_trend;
    int relief_active;
} PressureModelOutput;
```

- [ ] **Step 4: Add the additive defaults and zero-speed torque output in `src/sim/PressureModel.c`**

Add these helpers near the top of `src/sim/PressureModel.c`, after `pressure_model_maxf(...)`:

```c
static float pressure_model_absf(float value) {
    return (value < 0.0f) ? -value : value;
}

static float pressure_model_interp3(const float values[3], float abs_rpm) {
    if (abs_rpm <= 20.0f) {
        float t = abs_rpm / 20.0f;
        return values[0] + (values[1] - values[0]) * t;
    }
    if (abs_rpm >= 40.0f) {
        return values[2];
    }
    {
        float t = (abs_rpm - 20.0f) / 20.0f;
        return values[1] + (values[2] - values[1]) * t;
    }
}
```

Update `PressureModel_InitParams(...)` to initialize the new additive fields while keeping behavior unchanged for now:

```c
    params->veff_base_m3 = 5.0e-4f;
    params->leak_base_m3_pa_s =
        (params->pump_displacement_m3_rev * (10.0f / 60.0f)) / (40.0f * 1.0e5f);
    params->tooth_drop_depth_base = 0.10f;
    params->tooth_drop_phase_base = 0.0f;

    params->veff_speed_scale[0] = 1.0f;
    params->veff_speed_scale[1] = 1.0f;
    params->veff_speed_scale[2] = 1.0f;
    params->leak_speed_scale[0] = 1.0f;
    params->leak_speed_scale[1] = 1.0f;
    params->leak_speed_scale[2] = 1.0f;
    params->drop_depth_scale[0] = 1.0f;
    params->drop_depth_scale[1] = 1.0f;
    params->drop_depth_scale[2] = 1.0f;
    params->drop_phase_offset[0] = 0.0f;
    params->drop_phase_offset[1] = 0.0f;
    params->drop_phase_offset[2] = 0.0f;
    params->torque_bias = 0.0f;
    params->torque_from_pressure_gain = 0.0f;
    params->torque_from_speed_gain = 0.0f;
```

In `PressureModel_Step(...)`, switch the old field names to the new ones without changing the underlying behavior yet:

```c
    q_leak = params->leak_base_m3_pa_s * state->pressure_pa;
    d_pressure = (params->bulk_modulus_pa / params->veff_base_m3) * q_net * dt;
```

Replace the tooth-drop use site:

```c
            float gain = 1.0f - params->tooth_drop_depth_base * window;
```

After `out->net_flow_m3_s = q_net;`, add:

```c
    out->estimated_torque_trend = pressure_model_maxf(
        0.0f,
        params->torque_bias +
        params->torque_from_pressure_gain * out->measured_pressure_bar +
        params->torque_from_speed_gain * pressure_model_absf(state->motor_rpm));
```

- [ ] **Step 5: Run the unit test until the API-extension test passes without changing the old fit behavior yet**

Run:

```bash
cmake --build --preset unixgcc --target test_pressure_model
ctest --test-dir out/build/unixgcc -R '^test_pressure_model$' --output-on-failure
```

Expected: `test_init_params_expose_open_loop_fit_knobs` passes. `test_ten_rpm_converges_to_forty_bar` should still pass at this point because tuning has not changed yet.

- [ ] **Step 6: Commit the API-extension slice**

```bash
git add include/pressure_model.h src/sim/PressureModel.c tests/test_pressure_model.c
git commit -m "Expose the tuning knobs needed for open-loop pressure fitting" -m "Constraint: The fitted model needs low-degree speed corrections and a torque-trend output without breaking existing call sites
Rejected: A second parallel params struct for fitted models | Duplicate plumbing and higher maintenance cost
Confidence: high
Scope-risk: narrow
Directive: Keep all new fields additive and preserve pressure_update(...)
Tested: ctest --test-dir out/build/unixgcc -R '^test_pressure_model$' --output-on-failure
Not-tested: Full suite not run in this task"
```

---

### Task 2: Replace the outdated 10 rpm -> 40 bar contract with measured open-loop envelope regression

**Files:**
- Create: `tests/fixtures/pressure_model_open_loop_reference.h`
- Modify: `tests/test_pressure_model.c`
- Modify: `src/sim/PressureModel.c`

- [ ] **Step 1: Add a compact measured-data fixture derived from `open10203040-positive.csv`**

Create `tests/fixtures/pressure_model_open_loop_reference.h` with:

```c
#ifndef PRESSURE_MODEL_OPEN_LOOP_REFERENCE_H
#define PRESSURE_MODEL_OPEN_LOOP_REFERENCE_H

typedef struct {
    float command_rpm;
    int sample_count;
    float head_pressure_bar;
    float tail_pressure_bar;
    float head_motor_rpm;
    float tail_motor_rpm;
    float tail_tooth_span_bar;
    float tail_tooth_min_phase;
    float tail_torque_trend;
} PressureModelOpenLoopReference;

static const PressureModelOpenLoopReference kPressureModelOpenLoopReference[] = {
    {10.0f, 20472, 17.5155f, 21.4452f,  9.6740f, 10.0500f, 3.4294f, 0.596154f,  2796.4850f},
    {20.0f, 25311, 44.1764f, 54.4452f, 19.3135f, 20.0320f, 4.9955f, 0.673077f,  6229.6050f},
    {30.0f, 26381, 73.7124f, 88.8450f, 28.9025f, 29.9760f, 5.5772f, 0.711538f,  9707.6300f},
    {40.0f, 11077,102.1228f,125.4012f, 38.5440f, 39.9270f, 5.4218f, 0.711538f, 13550.1800f}
};

enum {
    PRESSURE_MODEL_OPEN_LOOP_REFERENCE_COUNT =
        (int)(sizeof(kPressureModelOpenLoopReference) / sizeof(kPressureModelOpenLoopReference[0]))
};

#endif /* PRESSURE_MODEL_OPEN_LOOP_REFERENCE_H */
```

- [ ] **Step 2: Replace the old 10 rpm steady-state test with measured section-summary helpers and a failing envelope regression**

In `tests/test_pressure_model.c`, add this include near the top:

```c
#include "fixtures/pressure_model_open_loop_reference.h"
```

Add these helpers after `run_steps(...)`:

```c
typedef struct {
    float head_pressure_bar;
    float tail_pressure_bar;
    float head_motor_rpm;
    float tail_motor_rpm;
    float tail_torque_trend;
} PressureModelSectionSummary;

static void summarize_open_loop_section(const PressureModelParams *params,
                                        const PressureModelOpenLoopReference *reference,
                                        PressureModelSectionSummary *summary) {
    PressureModelState state;
    PressureModelOutput out;
    float head_pressure_sum = 0.0f;
    float tail_pressure_sum = 0.0f;
    float head_motor_sum = 0.0f;
    float tail_motor_sum = 0.0f;
    float tail_torque_sum = 0.0f;
    int i;

    memset(&out, 0, sizeof(out));
    memset(summary, 0, sizeof(*summary));
    PressureModel_Reset(&state, 0x61616161u + (unsigned int)reference->command_rpm);

    for (i = 0; i < reference->sample_count; ++i) {
        PressureModel_Step(params, &state, reference->command_rpm, DT_S, &out);
        if (i < 2000) {
            head_pressure_sum += out.measured_pressure_bar;
            head_motor_sum += out.actual_motor_rpm;
        }
        if (i >= reference->sample_count - 2000) {
            tail_pressure_sum += out.measured_pressure_bar;
            tail_motor_sum += out.actual_motor_rpm;
            tail_torque_sum += out.estimated_torque_trend;
        }
    }

    summary->head_pressure_bar = head_pressure_sum / 2000.0f;
    summary->tail_pressure_bar = tail_pressure_sum / 2000.0f;
    summary->head_motor_rpm = head_motor_sum / 2000.0f;
    summary->tail_motor_rpm = tail_motor_sum / 2000.0f;
    summary->tail_torque_trend = tail_torque_sum / 2000.0f;
}
```

Delete `test_ten_rpm_converges_to_forty_bar()` and replace it with:

```c
static int test_open_loop_sections_match_measured_pressure_envelope(void) {
    PressureModelParams params = make_deterministic_params();
    int i;

    for (i = 0; i < PRESSURE_MODEL_OPEN_LOOP_REFERENCE_COUNT; ++i) {
        PressureModelSectionSummary summary;
        const PressureModelOpenLoopReference *reference = &kPressureModelOpenLoopReference[i];

        summarize_open_loop_section(&params, reference, &summary);

        ASSERT_NEAR(summary.head_pressure_bar, reference->head_pressure_bar, 4.0f);
        ASSERT_NEAR(summary.tail_pressure_bar, reference->tail_pressure_bar, 3.0f);
        ASSERT_NEAR(summary.head_motor_rpm, reference->head_motor_rpm, 0.8f);
        ASSERT_NEAR(summary.tail_motor_rpm, reference->tail_motor_rpm, 0.4f);
    }

    return 1;
}
```

Wire the new test into `main()` in the same slot where `test_ten_rpm_converges_to_forty_bar()` used to be:

```c
    if (test_open_loop_sections_match_measured_pressure_envelope()) {
        ++passed;
        printf("PASS test_open_loop_sections_match_measured_pressure_envelope\n");
    } else {
        ++failed;
        printf("FAIL test_open_loop_sections_match_measured_pressure_envelope\n");
    }
```

- [ ] **Step 3: Run the targeted test and confirm the current defaults fail against the measured data**

Run:

```bash
cmake --build --preset unixgcc --target test_pressure_model
ctest --test-dir out/build/unixgcc -R '^test_pressure_model$' --output-on-failure
```

Expected: `test_open_loop_sections_match_measured_pressure_envelope` fails because the current model still targets roughly `40 bar` at `10 rpm`.

- [ ] **Step 4: Tune the main pressure skeleton with low-degree effective-volume and leakage corrections**

Add these helpers in `src/sim/PressureModel.c`, right after `pressure_model_interp3(...)`:

```c
static float pressure_model_effective_volume(const PressureModelParams *params, float motor_rpm) {
    return params->veff_base_m3 *
           pressure_model_interp3(params->veff_speed_scale, pressure_model_absf(motor_rpm));
}

static float pressure_model_leak_coeff(const PressureModelParams *params, float motor_rpm) {
    return params->leak_base_m3_pa_s *
           pressure_model_interp3(params->leak_speed_scale, pressure_model_absf(motor_rpm));
}
```

Update the tuned defaults in `PressureModel_InitParams(...)`:

```c
    params->veff_base_m3 = 4.4e-4f;
    params->leak_base_m3_pa_s = 1.2245e-12f;

    params->veff_speed_scale[0] = 1.18f;
    params->veff_speed_scale[1] = 1.00f;
    params->veff_speed_scale[2] = 0.86f;

    params->leak_speed_scale[0] = 1.27f;
    params->leak_speed_scale[1] = 1.00f;
    params->leak_speed_scale[2] = 0.87f;
```

Change the pressure-state calculation in `PressureModel_Step(...)` to:

```c
    q_leak = pressure_model_leak_coeff(params, state->motor_rpm) * state->pressure_pa;
    d_pressure = (params->bulk_modulus_pa /
                  pressure_model_effective_volume(params, state->motor_rpm)) *
                 q_net * dt;
```

- [ ] **Step 5: Re-run the measured-envelope regression until the main pressure skeleton passes**

Run:

```bash
cmake --build --preset unixgcc --target test_pressure_model
ctest --test-dir out/build/unixgcc -R '^test_pressure_model$' --output-on-failure
```

Expected: `test_open_loop_sections_match_measured_pressure_envelope` passes, along with the pre-existing zero-speed, continuity, reverse-depressurization, relief, and legacy-wrapper tests.

- [ ] **Step 6: Commit the measured-envelope fitting slice**

```bash
git add tests/fixtures/pressure_model_open_loop_reference.h tests/test_pressure_model.c src/sim/PressureModel.c
git commit -m "Tune the main pressure state to the measured open-loop envelope" -m "Constraint: The fit must match 10/20/30/40 rpm data while staying one physical model
Rejected: Independent per-speed pressure curves | Good short-term fit, poor extrapolation and lower control-design value
Confidence: medium
Scope-risk: moderate
Directive: Keep speed dependence limited to effective volume and leakage in this task
Tested: ctest --test-dir out/build/unixgcc -R '^test_pressure_model$' --output-on-failure
Not-tested: Full CTest suite not run in this task"
```

---

### Task 3: Fit the tooth-synchronous pressure signature and the derived torque trend

**Files:**
- Modify: `src/sim/PressureModel.c`
- Modify: `tests/test_pressure_model.c`

- [ ] **Step 1: Extend the section-summary helper with tooth-span and phase measurements, then write the failing test**

In `tests/test_pressure_model.c`, expand `PressureModelSectionSummary` to:

```c
typedef struct {
    float head_pressure_bar;
    float tail_pressure_bar;
    float head_motor_rpm;
    float tail_motor_rpm;
    float tail_tooth_span_bar;
    float tail_tooth_min_phase;
    float tail_torque_trend;
} PressureModelSectionSummary;
```

Replace `summarize_open_loop_section(...)` with this version:

```c
static void summarize_open_loop_section(const PressureModelParams *params,
                                        const PressureModelOpenLoopReference *reference,
                                        PressureModelSectionSummary *summary) {
    PressureModelState state;
    PressureModelOutput out;
    float head_pressure_sum = 0.0f;
    float tail_pressure_sum = 0.0f;
    float head_motor_sum = 0.0f;
    float tail_motor_sum = 0.0f;
    float tail_torque_sum = 0.0f;
    float bins[26];
    int counts[26];
    int i;

    memset(&out, 0, sizeof(out));
    memset(summary, 0, sizeof(*summary));
    memset(bins, 0, sizeof(bins));
    memset(counts, 0, sizeof(counts));
    PressureModel_Reset(&state, 0x61616161u + (unsigned int)reference->command_rpm);

    for (i = 0; i < reference->sample_count; ++i) {
        float tooth_phase;
        int bin_index;

        PressureModel_Step(params, &state, reference->command_rpm, DT_S, &out);
        if (i < 2000) {
            head_pressure_sum += out.measured_pressure_bar;
            head_motor_sum += out.actual_motor_rpm;
        }
        if (i >= reference->sample_count - 2000) {
            tail_pressure_sum += out.measured_pressure_bar;
            tail_motor_sum += out.actual_motor_rpm;
            tail_torque_sum += out.estimated_torque_trend;
        }
        if (i >= reference->sample_count - 5000) {
            tooth_phase = fmodf(13.0f * state.pump_phase_rev, 1.0f);
            if (tooth_phase < 0.0f) {
                tooth_phase += 1.0f;
            }
            bin_index = (int)(tooth_phase * 26.0f);
            if (bin_index > 25) {
                bin_index = 25;
            }
            bins[bin_index] += out.measured_pressure_bar;
            counts[bin_index] += 1;
        }
    }

    summary->head_pressure_bar = head_pressure_sum / 2000.0f;
    summary->tail_pressure_bar = tail_pressure_sum / 2000.0f;
    summary->head_motor_rpm = head_motor_sum / 2000.0f;
    summary->tail_motor_rpm = tail_motor_sum / 2000.0f;
    summary->tail_torque_trend = tail_torque_sum / 2000.0f;

    {
        float min_value = 1.0e30f;
        float max_value = -1.0e30f;
        int min_index = 0;

        for (i = 0; i < 26; ++i) {
            float mean_value;
            if (counts[i] == 0) {
                continue;
            }
            mean_value = bins[i] / (float)counts[i];
            if (mean_value < min_value) {
                min_value = mean_value;
                min_index = i;
            }
            if (mean_value > max_value) {
                max_value = mean_value;
            }
        }

        summary->tail_tooth_span_bar = max_value - min_value;
        summary->tail_tooth_min_phase = ((float)min_index + 0.5f) / 26.0f;
    }
}
```

Add this new regression test after `test_open_loop_sections_match_measured_pressure_envelope()`:

```c
static int test_open_loop_sections_match_tooth_phase_and_torque_trend(void) {
    PressureModelParams params = make_deterministic_params();
    float previous_tail_torque = -1.0f;
    int i;

    for (i = 0; i < PRESSURE_MODEL_OPEN_LOOP_REFERENCE_COUNT; ++i) {
        PressureModelSectionSummary summary;
        const PressureModelOpenLoopReference *reference = &kPressureModelOpenLoopReference[i];

        summarize_open_loop_section(&params, reference, &summary);

        ASSERT_NEAR(summary.tail_tooth_span_bar, reference->tail_tooth_span_bar, 1.5f);
        ASSERT_NEAR(summary.tail_tooth_min_phase, reference->tail_tooth_min_phase, 0.08f);
        ASSERT_TRUE(summary.tail_torque_trend > previous_tail_torque);
        ASSERT_TRUE(summary.tail_torque_trend > reference->tail_torque_trend * 0.80f);
        ASSERT_TRUE(summary.tail_torque_trend < reference->tail_torque_trend * 1.20f);
        previous_tail_torque = summary.tail_torque_trend;
    }

    return 1;
}
```

Wire it into `main()` after the pressure-envelope test:

```c
    if (test_open_loop_sections_match_tooth_phase_and_torque_trend()) {
        ++passed;
        printf("PASS test_open_loop_sections_match_tooth_phase_and_torque_trend\n");
    } else {
        ++failed;
        printf("FAIL test_open_loop_sections_match_tooth_phase_and_torque_trend\n");
    }
```

- [ ] **Step 2: Run the pressure-model test and confirm the current tooth-phase and torque behavior are still wrong**

Run:

```bash
cmake --build --preset unixgcc --target test_pressure_model
ctest --test-dir out/build/unixgcc -R '^test_pressure_model$' --output-on-failure
```

Expected: the new tooth-phase / torque-trend regression fails even if the main pressure-envelope regression passes.

- [ ] **Step 3: Implement speed-dependent tooth-depth, tooth-phase, and torque-trend tuning**

In `PressureModel_InitParams(...)`, replace the provisional tooth and torque defaults with:

```c
    params->flow_ripple_ratio = 0.08f;
    params->tooth_drop_depth_base = 0.16f;
    params->tooth_drop_width_ratio = 0.20f;
    params->tooth_drop_phase_base = 0.58f;

    params->drop_depth_scale[0] = 1.00f;
    params->drop_depth_scale[1] = 0.62f;
    params->drop_depth_scale[2] = 0.30f;

    params->drop_phase_offset[0] = 0.00f;
    params->drop_phase_offset[1] = 0.07f;
    params->drop_phase_offset[2] = 0.11f;

    params->torque_bias = 400.0f;
    params->torque_from_pressure_gain = 110.0f;
    params->torque_from_speed_gain = 8.0f;
```

Add these helpers below `pressure_model_leak_coeff(...)`:

```c
static float pressure_model_tooth_drop_depth(const PressureModelParams *params, float motor_rpm) {
    return params->tooth_drop_depth_base *
           pressure_model_interp3(params->drop_depth_scale, pressure_model_absf(motor_rpm));
}

static float pressure_model_tooth_drop_phase(const PressureModelParams *params, float motor_rpm) {
    return params->tooth_drop_phase_base +
           pressure_model_interp3(params->drop_phase_offset, pressure_model_absf(motor_rpm));
}
```

Then replace the visible-pressure tooth-drop block in `PressureModel_Step(...)` with:

```c
    visible_pressure_pa = state->pressure_pa;
    if (state->motor_rpm > 0.01f) {
        float depth = pressure_model_tooth_drop_depth(params, state->motor_rpm);
        float phase = pressure_model_tooth_drop_phase(params, state->motor_rpm);

        tooth_phase = pressure_model_wrap_unit(13.0f * state->pump_phase_rev + phase);
        if (tooth_phase < params->tooth_drop_width_ratio) {
            float window = 0.5f * (1.0f + cosf((2.0f * PRESSURE_MODEL_PI * tooth_phase) /
                                               params->tooth_drop_width_ratio));
            float gain = 1.0f - depth * window;
            visible_pressure_pa *= gain;
        }
    }
```

Leave the torque expression added in Task 1 in place; it will now use the tuned gains.

- [ ] **Step 4: Re-run the model regressions until pressure, tooth-phase, and torque trend all pass together**

Run:

```bash
cmake --build --preset unixgcc --target test_pressure_model
ctest --test-dir out/build/unixgcc -R '^test_pressure_model$' --output-on-failure
```

Expected: all `test_pressure_model` cases pass, including the measured pressure envelope, tooth-phase, relief cap, deterministic noise, negative-speed depressurization, and legacy wrapper continuity checks.

- [ ] **Step 5: Commit the ripple and torque-trend slice**

```bash
git add src/sim/PressureModel.c tests/test_pressure_model.c
git commit -m "Lock the tooth-synchronous pressure signature to the measured ripple" -m "Constraint: Ripple fitting must preserve one pressure-state model and only add weak speed-dependent tooth corrections
Rejected: Encoding a separate steady-state waveform per speed | Too empirical and fragile outside the measured points
Confidence: medium
Scope-risk: moderate
Directive: Keep torque as a derived output and do not feed it back into the pressure state
Tested: ctest --test-dir out/build/unixgcc -R '^test_pressure_model$' --output-on-failure
Not-tested: Full CTest suite not run in this task"
```

---

### Task 4: Lock FB-level negative-speed behavior and run the verification matrix

**Files:**
- Modify: `tests/test_hydro_sim_fb.c`

- [ ] **Step 1: Add an FB-level regression for negative-speed depressurization without reset**

In `tests/test_hydro_sim_fb.c`, add this test after `test_pressure_model_fb_persists_state_and_resets_on_disable()`:

```c
static void test_pressure_model_fb_negative_speed_depressurizes_without_reset(void) {
    HYD_PRESSUREMODEL cmd;
    double charged_pressure;
    int i;

    memset(&cmd, 0, sizeof(cmd));
    cmd.ENABLE.value = true;
    cmd.MOTOR_RPM.value = 40.0;

    for (i = 0; i < 12000; ++i) {
        cmd.TIME_S.value = 0.001 * (double)i;
        __mcl_cmd_updatePressureModel(&cmd);
    }

    charged_pressure = cmd.REAL_PRESSURE_BAR.value;
    ASSERT_TRUE(charged_pressure > 100.0,
                "Positive-speed pressure build should charge the FB before reverse command");

    cmd.MOTOR_RPM.value = -40.0;
    for (i = 12000; i < 15000; ++i) {
        cmd.TIME_S.value = 0.001 * (double)i;
        __mcl_cmd_updatePressureModel(&cmd);
    }

    ASSERT_TRUE(cmd.ACTIVE.value,
                "Negative-speed depressurization should keep the PressureModel FB active");
    ASSERT_TRUE(cmd.REAL_PRESSURE_BAR.value < charged_pressure,
                "Negative-speed command should reduce the real pressure without a reset");
    ASSERT_TRUE(cmd.REAL_PRESSURE_BAR.value >= 0.0,
                "Negative-speed command must not drive real pressure below zero");
}
```

Wire it into `main()` after `test_pressure_model_fb_persists_state_and_resets_on_disable();`:

```c
    test_pressure_model_fb_negative_speed_depressurizes_without_reset();
```

- [ ] **Step 2: Run the focused simulator regressions**

Run:

```bash
cmake --build --preset unixgcc --target test_pressure_model test_hydro_sim_fb
ctest --test-dir out/build/unixgcc -R '^(test_pressure_model|test_hydro_sim_fb)$' --output-on-failure
```

Expected: both `test_pressure_model` and `test_hydro_sim_fb` pass together.

- [ ] **Step 3: Run the full verification matrix required by the repo contract**

Run:

```bash
ctest --test-dir out/build/unixgcc --output-on-failure
```

Expected: full CTest suite passes. There is no separate lint or typecheck target in this repo, so full CTest is the verification bar for this implementation plan.

- [ ] **Step 4: Commit the FB regression and final verification slice**

```bash
git add tests/test_hydro_sim_fb.c
git commit -m "Guard FB-level reverse-pressure behavior for the tuned model" -m "Constraint: The tuned PressureModel must keep legacy FB persistence and reset behavior while supporting negative-speed depressurization
Rejected: Leaving reverse-pressure behavior covered only at the unit-test level | Misses integration regressions through __mcl_cmd_updatePressureModel
Confidence: high
Scope-risk: narrow
Directive: Keep HYD_PRESSUREMODEL semantics stable unless a future spec changes the FB surface
Tested: ctest --test-dir out/build/unixgcc -R '^(test_pressure_model|test_hydro_sim_fb)$' --output-on-failure; ctest --test-dir out/build/unixgcc --output-on-failure
Not-tested: No manual visual comparison against the raw CSV in this task"
```
