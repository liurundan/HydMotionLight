# Pressure Model Closed-Loop Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将 `src/sim/PressureModel.c` 重构为可测试的死挡（伺服阀全关）液压压力对象模型，补齐 13 齿压力特征、负转速卸压、噪声控制和 PLC FB 跨扫描状态保持，并新增独立测试与 FB 回归测试。

**Architecture:** 新建 `include/pressure_model.h` 作为显式参数/状态/输出 API，保留 `src/sim/PressureModel.c` 作为唯一物理模型实现。`src/sim/hydro_sim_fb.c` 不再依赖 `pressure_update()` 内部隐式状态，而是持有一份静态 `PressureModelParams/State`，根据 `TIME_S` 计算 `dt_s` 后逐拍调用 `PressureModel_Step(...)`。测试分两层：`tests/test_pressure_model.c` 直接验证物理模型，`tests/test_hydro_sim_fb.c` 只验证 FB 状态保持与复位。

**Tech Stack:** C99, HydroSimLib (`src/sim/*.c`), CMake/CTest, GNU libm

---

## File Structure

| 文件 | 角色 | 操作 |
|------|------|------|
| `include/pressure_model.h` | 新的压力模型公开 API：参数、状态、输出、Step/Reset 接口 | Create |
| `src/sim/PressureModel.c` | 压力模型唯一实现，保留兼容 `pressure_update()` 包装层 | Modify |
| `src/sim/hydro_sim_fb.c` | `HYD_PRESSUREMODEL` FB 适配层，持有显式状态并按 `TIME_S` 增量步进 | Modify |
| `tests/test_pressure_model.c` | 压力模型独立单元测试：稳态、动态、齿脉动、噪声、限压、负转速 | Create |
| `tests/test_hydro_sim_fb.c` | FB 级回归测试：跨拍状态保持、`ENABLE=0` 复位、同种子重启一致性 | Modify |
| `CMakeLists.txt` | `test_pressure_model` 目标与 CTest 注册 | Modify |

**Boundary rules:**

- 不改 `src/sim/hydro_sim.c`
- 不添加阀门开启态仿真
- 不引入新依赖
- `10 rpm -> 40 bar` 只用于泄漏系数校准，不做经验曲线拟合

---

### Task 1: 建立显式 API 和确定性零输入基线

**Files:**
- Create: `include/pressure_model.h`
- Create: `tests/test_pressure_model.c`
- Modify: `src/sim/PressureModel.c`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: 写第一个失败测试，只锁定“零转速保持零压”的最小行为**

在 `tests/test_pressure_model.c` 新建以下内容：

```c
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "pressure_model.h"

#define DT_S 0.001f
#define PRESSURE_EPS 1e-4f
#define RPM_EPS 1e-3f

static int tests_run = 0;
static int tests_passed = 0;

#define ASSERT_TRUE(cond, msg) do { \
    ++tests_run; \
    if (cond) { \
        ++tests_passed; \
    } else { \
        printf("FAIL: %s\n", msg); \
    } \
} while (0)

#define ASSERT_NEAR(actual, expected, tol, msg) do { \
    ++tests_run; \
    if (fabs((double)((actual) - (expected))) <= (double)(tol)) { \
        ++tests_passed; \
    } else { \
        printf("FAIL: %s (got=%g expected=%g tol=%g)\n", \
               msg, (double)(actual), (double)(expected), (double)(tol)); \
    } \
} while (0)

static PressureModelParams make_deterministic_params(void) {
    PressureModelParams params;

    PressureModel_InitParams(&params);
    params.enable_sensor_noise = 0;
    params.enable_motor_noise = 0;
    params.enable_process_noise = 0;
    params.sensor_noise_std_bar = 0.0f;
    params.motor_noise_std_rpm = 0.0f;
    params.process_noise_std_m3_s = 0.0f;
    params.sensor_bias_bar = 0.0f;
    return params;
}

static void run_steps(const PressureModelParams* params,
                      PressureModelState* state,
                      float target_rpm,
                      int cycles,
                      float dt_s,
                      PressureModelOutput* out) {
    int i;

    for (i = 0; i < cycles; ++i) {
        PressureModel_Step(params, state, target_rpm, dt_s, out);
    }
}

static void test_zero_speed_holds_zero_pressure(void) {
    PressureModelParams params = make_deterministic_params();
    PressureModelState state;
    PressureModelOutput out;

    memset(&out, 0, sizeof(out));
    PressureModel_Reset(&state, 0x12345678u);

    run_steps(&params, &state, 0.0f, 2000, DT_S, &out);

    ASSERT_NEAR(out.actual_motor_rpm, 0.0f, RPM_EPS,
                "zero target should keep actual motor speed at zero");
    ASSERT_NEAR(out.real_pressure_bar, 0.0f, PRESSURE_EPS,
                "zero target should keep real pressure at zero");
    ASSERT_NEAR(out.measured_pressure_bar, 0.0f, PRESSURE_EPS,
                "zero target should keep measured pressure at zero");
}

int main(void) {
    printf("=== PressureModel Tests ===\n");
    test_zero_speed_holds_zero_pressure();
    printf("=== Results: %d/%d passed ===\n", tests_passed, tests_run);
    return (tests_passed == tests_run) ? 0 : 1;
}
```

- [ ] **Step 2: 在 `CMakeLists.txt` 注册独立测试目标**

在仿真测试区域追加：

```cmake
add_executable(test_pressure_model tests/test_pressure_model.c)
target_link_libraries(test_pressure_model PRIVATE HydroSimLib)
add_test(NAME test_pressure_model COMMAND test_pressure_model)
```

建议放在现有 `test_hydro_sim_fb` 附近：

```cmake
# ==================================================================
# 仿真相关测试
# ==================================================================
add_executable(test_hydro_sim_fb tests/test_hydro_sim_fb.c)
target_link_libraries(test_hydro_sim_fb PRIVATE HydroSimLib)

add_executable(test_pressure_model tests/test_pressure_model.c)
target_link_libraries(test_pressure_model PRIVATE HydroSimLib)
```

并在 `enable_testing()` 之后追加：

```cmake
add_test(NAME test_pressure_model COMMAND test_pressure_model)
```

- [ ] **Step 3: 先跑构建，确认测试因 API 缺失而失败**

Run:

```bash
cmake --preset unixgcc
cmake --build --preset unixgcc --target test_pressure_model
```

Expected: build fails with missing `pressure_model.h` and/or undefined `PressureModel_InitParams`, `PressureModel_Reset`, `PressureModel_Step`.

- [ ] **Step 4: 创建显式头文件 `include/pressure_model.h`**

写入完整头文件：

```c
#ifndef PRESSURE_MODEL_H
#define PRESSURE_MODEL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

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
} PressureModelParams;

typedef struct {
    float motor_rpm;
    float pressure_pa;
    float pump_phase_rev;
    uint32_t rng_state;
    int has_spare_gauss;
    float spare_gauss;
} PressureModelState;

typedef struct {
    float measured_pressure_bar;
    float real_pressure_bar;
    float actual_motor_rpm;
    float pump_flow_m3_s;
    float net_flow_m3_s;
    int relief_active;
} PressureModelOutput;

void PressureModel_InitParams(PressureModelParams* params);
void PressureModel_Reset(PressureModelState* state, uint32_t seed);
void PressureModel_Step(const PressureModelParams* params,
                        PressureModelState* state,
                        float target_rpm,
                        float dt_s,
                        PressureModelOutput* out);
float pressure_update(float target_rpm,
                      float t,
                      float *P_state,
                      float *real_P,
                      float *actual_motor_rpm);

#ifdef __cplusplus
}
#endif

#endif /* PRESSURE_MODEL_H */
```

- [ ] **Step 5: 用最小实现重写 `src/sim/PressureModel.c`，先让零输入基线通过**

把文件顶层替换为以下最小版本。此版本先只提供参数、状态、零输入安全行为和可编译的旧接口包装层，后续任务再加完整物理过程：

```c
#include "pressure_model.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define PRESSURE_MODEL_DEFAULT_DT_S 0.001f

static float pressure_model_clampf(float value, float min_value, float max_value) {
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

static uint32_t pressure_model_seed(uint32_t seed) {
    return (seed != 0u) ? seed : 1u;
}

void PressureModel_InitParams(PressureModelParams* params) {
    if (params == NULL) return;

    memset(params, 0, sizeof(*params));
    params->pump_displacement_m3_rev = 20.0e-6f;
    params->bulk_modulus_pa = 1.6e9f;
    params->chamber_volume_m3 = 5.0e-4f;
    params->leak_coeff_m3_pa_s =
        (params->pump_displacement_m3_rev * (10.0f / 60.0f)) / (40.0f * 1.0e5f);
    params->relief_set_pa = 250.0f * 1.0e5f;
    params->relief_coeff_m3_pa_s = 1.2e-9f;
    params->sensor_range_bar = 250.0f;
    params->sensor_noise_std_bar = 0.4f;
    params->sensor_bias_bar = 0.0f;
    params->motor_tau_s = 0.05f;
    params->motor_noise_std_rpm = 2.0f;
    params->process_noise_std_m3_s = 0.0f;
    params->flow_ripple_ratio = 0.10f;
    params->tooth_drop_depth_ratio = 0.05f;
    params->tooth_drop_width_ratio = 0.05f;
    params->min_rpm = -100.0f;
    params->max_rpm = 2000.0f;
    params->enable_sensor_noise = 1u;
    params->enable_motor_noise = 1u;
    params->enable_process_noise = 0u;
}

void PressureModel_Reset(PressureModelState* state, uint32_t seed) {
    if (state == NULL) return;

    memset(state, 0, sizeof(*state));
    state->rng_state = pressure_model_seed(seed);
}

void PressureModel_Step(const PressureModelParams* params,
                        PressureModelState* state,
                        float target_rpm,
                        float dt_s,
                        PressureModelOutput* out) {
    float dt;
    float clamped_target;
    float alpha;

    if (params == NULL || state == NULL || out == NULL) return;

    dt = (dt_s > 0.0f) ? dt_s : PRESSURE_MODEL_DEFAULT_DT_S;
    clamped_target = pressure_model_clampf(target_rpm, params->min_rpm, params->max_rpm);
    alpha = dt / (params->motor_tau_s + dt);
    state->motor_rpm += alpha * (clamped_target - state->motor_rpm);
    if (state->pressure_pa < 0.0f) {
        state->pressure_pa = 0.0f;
    }

    out->actual_motor_rpm = state->motor_rpm;
    out->real_pressure_bar = state->pressure_pa * 1.0e-5f;
    out->measured_pressure_bar = out->real_pressure_bar;
    out->pump_flow_m3_s = 0.0f;
    out->net_flow_m3_s = 0.0f;
    out->relief_active = 0;
}

float pressure_update(float target_rpm,
                      float t,
                      float *P_state,
                      float *real_P,
                      float *actual_motor_rpm) {
    static PressureModelParams params;
    static PressureModelState state;
    static int initialized = 0;
    PressureModelOutput out;

    (void)t;

    if (!initialized) {
        PressureModel_InitParams(&params);
        PressureModel_Reset(&state, 0x2468ace1u);
        initialized = 1;
    }

    if (P_state != NULL) {
        state.pressure_pa = *P_state;
    }

    PressureModel_Step(&params, &state, target_rpm, PRESSURE_MODEL_DEFAULT_DT_S, &out);

    if (P_state != NULL) {
        *P_state = state.pressure_pa;
    }
    if (real_P != NULL) {
        *real_P = out.real_pressure_bar;
    }
    if (actual_motor_rpm != NULL) {
        *actual_motor_rpm = out.actual_motor_rpm;
    }

    return out.measured_pressure_bar;
}
```

- [ ] **Step 6: 跑目标测试，确认最小行为已通过**

Run:

```bash
cmake --build --preset unixgcc --target test_pressure_model
ctest --test-dir out/build/unixgcc -R '^test_pressure_model$' --output-on-failure
```

Expected: `test_pressure_model` passes the zero-speed baseline.

- [ ] **Step 7: Commit**

```bash
git add include/pressure_model.h src/sim/PressureModel.c tests/test_pressure_model.c CMakeLists.txt
git commit --no-gpg-sign -m "Create a testable pressure-model API before tuning plant physics" -m "Introduces a public pressure-model header, a dedicated unit-test target, and a minimal deterministic implementation so the plant can be evolved behind tests.

Constraint: The first task only locks zero-input behavior and compile surfaces; full dead-head physics lands incrementally in later commits
Rejected: Reworking hydro_sim.c together with PressureModel | The approved scope keeps the pressure model isolated
Confidence: high
Scope-risk: narrow
Directive: Keep all new pressure-model behavior reachable through PressureModel_Step rather than hidden FB-only state
Tested: cmake --build --preset unixgcc --target test_pressure_model; ctest --test-dir out/build/unixgcc -R '^test_pressure_model$' --output-on-failure
Not-tested: Calibration, tooth ripple, relief behavior, and FB integration are not covered yet"
```

---

### Task 2: 实现死挡稳态校准和电机连续动态

**Files:**
- Modify: `src/sim/PressureModel.c`
- Modify: `tests/test_pressure_model.c`

- [ ] **Step 1: 补两个失败测试：10 rpm 稳态到 40 bar，电机转速跨步连续**

在 `tests/test_pressure_model.c` 中追加：

```c
static void test_ten_rpm_converges_to_forty_bar(void) {
    PressureModelParams params = make_deterministic_params();
    PressureModelState state;
    PressureModelOutput out;

    memset(&out, 0, sizeof(out));
    PressureModel_Reset(&state, 0x12345678u);

    run_steps(&params, &state, 10.0f, 15000, DT_S, &out);

    ASSERT_NEAR(out.real_pressure_bar, 40.0f, 1.5f,
                "10 rpm should converge near the calibrated 40 bar steady-state point");
    ASSERT_NEAR(out.measured_pressure_bar, out.real_pressure_bar, 1e-3f,
                "with noise disabled measured pressure should match real pressure");
}

static void test_motor_state_is_continuous_across_steps(void) {
    PressureModelParams params = make_deterministic_params();
    PressureModelState state;
    PressureModelOutput out0;
    PressureModelOutput out1;

    memset(&out0, 0, sizeof(out0));
    memset(&out1, 0, sizeof(out1));
    PressureModel_Reset(&state, 0x12345678u);

    PressureModel_Step(&params, &state, 1000.0f, DT_S, &out0);
    PressureModel_Step(&params, &state, 1000.0f, DT_S, &out1);

    ASSERT_TRUE(out0.actual_motor_rpm > 0.0f,
                "first step should accelerate the motor above zero");
    ASSERT_TRUE(out0.actual_motor_rpm < 1000.0f,
                "first step should not jump straight to the target rpm");
    ASSERT_TRUE(out1.actual_motor_rpm > out0.actual_motor_rpm,
                "second step should continue from prior motor state instead of restarting");
}
```

并在 `main()` 中调用：

```c
    test_ten_rpm_converges_to_forty_bar();
    test_motor_state_is_continuous_across_steps();
```

- [ ] **Step 2: 跑测试，确认当前最小实现失败**

Run:

```bash
cmake --build --preset unixgcc --target test_pressure_model
ctest --test-dir out/build/unixgcc -R '^test_pressure_model$' --output-on-failure
```

Expected: `test_ten_rpm_converges_to_forty_bar` fails because pressure never建立；`test_motor_state_is_continuous_across_steps` may still pass or partially pass, but the suite overall must fail before implementation proceeds.

- [ ] **Step 3: 给 `PressureModel_Step` 加上基础死挡物理方程和稳定校准**

在 `src/sim/PressureModel.c` 中追加基础辅助函数：

```c
static float pressure_model_maxf(float a, float b) {
    return (a > b) ? a : b;
}
```

然后把 `PressureModel_Step(...)` 替换为以下版本。此阶段先实现电机一阶动态、基础泵流量、泄漏、压缩积分和 250 bar 前的真实压力建立；13 齿脉动、齿谷和噪声留到下一任务：

```c
void PressureModel_Step(const PressureModelParams* params,
                        PressureModelState* state,
                        float target_rpm,
                        float dt_s,
                        PressureModelOutput* out) {
    float dt;
    float clamped_target;
    float alpha;
    float q_pump;
    float q_leak;
    float q_relief;
    float q_net;
    float d_pressure;

    if (params == NULL || state == NULL || out == NULL) return;

    dt = (dt_s > 0.0f) ? dt_s : PRESSURE_MODEL_DEFAULT_DT_S;
    clamped_target = pressure_model_clampf(target_rpm, params->min_rpm, params->max_rpm);
    alpha = dt / (params->motor_tau_s + dt);
    state->motor_rpm += alpha * (clamped_target - state->motor_rpm);
    state->motor_rpm = pressure_model_clampf(state->motor_rpm, params->min_rpm, params->max_rpm);

    q_pump = params->pump_displacement_m3_rev * (state->motor_rpm / 60.0f);
    q_leak = params->leak_coeff_m3_pa_s * state->pressure_pa;
    q_relief = 0.0f;
    if (state->pressure_pa > params->relief_set_pa) {
        q_relief = params->relief_coeff_m3_pa_s * (state->pressure_pa - params->relief_set_pa);
    }

    q_net = q_pump - q_leak - q_relief;
    d_pressure = (params->bulk_modulus_pa / params->chamber_volume_m3) * q_net * dt;
    state->pressure_pa = pressure_model_maxf(0.0f, state->pressure_pa + d_pressure);

    out->actual_motor_rpm = state->motor_rpm;
    out->real_pressure_bar = state->pressure_pa * 1.0e-5f;
    out->measured_pressure_bar = pressure_model_clampf(out->real_pressure_bar,
                                                       0.0f,
                                                       params->sensor_range_bar);
    out->pump_flow_m3_s = q_pump;
    out->net_flow_m3_s = q_net;
    out->relief_active = (q_relief > 0.0f) ? 1 : 0;
}
```

- [ ] **Step 4: 重新跑测试，确认稳态校准和连续动态都通过**

Run:

```bash
cmake --build --preset unixgcc --target test_pressure_model
ctest --test-dir out/build/unixgcc -R '^test_pressure_model$' --output-on-failure
```

Expected: zero-speed、10 rpm 稳态、motor continuity 三个测试全部通过。

- [ ] **Step 5: Commit**

```bash
git add src/sim/PressureModel.c tests/test_pressure_model.c
git commit --no-gpg-sign -m "Calibrate the dead-head pressure core around the known 10 rpm point" -m "Adds the first physically meaningful plant dynamics: motor lag, pump flow, leakage balance, dead-head compression, and relief onset.

Constraint: Only the measured 10 rpm -> 40 bar point may be used for static calibration
Rejected: Fitting a multi-point rpm-pressure curve | The repo does not contain enough measured data to justify it
Confidence: high
Scope-risk: narrow
Directive: Keep measured pressure derivation separate from the real chamber pressure state
Tested: cmake --build --preset unixgcc --target test_pressure_model; ctest --test-dir out/build/unixgcc -R '^test_pressure_model$' --output-on-failure
Not-tested: Tooth ripple, visible tooth drops, explicit noise behavior, and FB scan persistence are not covered yet"
```

---

### Task 3: 加入 13 齿特征、负转速卸压、限压观测和可控噪声

**Files:**
- Modify: `include/pressure_model.h`
- Modify: `src/sim/PressureModel.c`
- Modify: `tests/test_pressure_model.c`

- [ ] **Step 1: 补四个失败测试，覆盖齿谷、负转速卸压、250 bar 限压、固定种子可复现**

在 `tests/test_pressure_model.c` 中追加辅助函数和测试：

```c
static int count_visible_tooth_valleys(const float* measured,
                                       const float* real,
                                       int samples,
                                       float min_gap_bar) {
    int i;
    int valleys = 0;

    for (i = 1; i + 1 < samples; ++i) {
        if (measured[i] < measured[i - 1] &&
            measured[i] <= measured[i + 1] &&
            (real[i] - measured[i]) >= min_gap_bar) {
            ++valleys;
        }
    }
    return valleys;
}

static void test_negative_speed_depressurizes_faster_than_passive_leak(void) {
    PressureModelParams params = make_deterministic_params();
    PressureModelState charged_state;
    PressureModelState leak_only_state;
    PressureModelState reverse_state;
    PressureModelOutput out;

    memset(&out, 0, sizeof(out));
    PressureModel_Reset(&charged_state, 0x11111111u);
    run_steps(&params, &charged_state, 10.0f, 15000, DT_S, &out);

    leak_only_state = charged_state;
    reverse_state = charged_state;

    run_steps(&params, &leak_only_state, 0.0f, 2000, DT_S, &out);
    run_steps(&params, &reverse_state, -50.0f, 2000, DT_S, &out);

    ASSERT_TRUE(reverse_state.pressure_pa < leak_only_state.pressure_pa,
                "small negative rpm should release pressure faster than passive leakage only");
    ASSERT_TRUE(reverse_state.pressure_pa >= 0.0f,
                "reverse depressurization must not drive real pressure below zero");
}

static void test_tooth_drop_is_visible_once_per_tooth(void) {
    PressureModelParams params = make_deterministic_params();
    PressureModelState state;
    PressureModelOutput out;
    float measured[100];
    float real[100];
    int i;
    int valleys;

    memset(&out, 0, sizeof(out));
    PressureModel_Reset(&state, 0x22222222u);

    run_steps(&params, &state, 600.0f, 2000, DT_S, &out);
    for (i = 0; i < 100; ++i) {
        PressureModel_Step(&params, &state, 600.0f, DT_S, &out);
        measured[i] = out.measured_pressure_bar;
        real[i] = out.real_pressure_bar;
    }

    valleys = count_visible_tooth_valleys(measured, real, 100, 0.05f);

    ASSERT_TRUE(valleys >= 10 && valleys <= 16,
                "one shaft revolution at 600 rpm should expose about 13 visible tooth valleys");
}

static void test_relief_caps_measured_output_at_two_hundred_fifty_bar(void) {
    PressureModelParams params = make_deterministic_params();
    PressureModelState state;
    PressureModelOutput out;

    memset(&out, 0, sizeof(out));
    PressureModel_Reset(&state, 0x33333333u);

    run_steps(&params, &state, 2000.0f, 30000, DT_S, &out);

    ASSERT_TRUE(out.measured_pressure_bar <= 250.0f + 1e-3f,
                "measured pressure must stay within the 250 bar sensor range");
    ASSERT_TRUE(out.relief_active,
                "high-speed dead-head run should activate relief flow near the ceiling");
}

static void test_noise_control_is_repeatable_with_fixed_seed(void) {
    PressureModelParams params;
    PressureModelState state_a;
    PressureModelState state_b;
    PressureModelOutput out_a;
    PressureModelOutput out_b;
    int i;

    PressureModel_InitParams(&params);
    params.enable_sensor_noise = 1;
    params.enable_motor_noise = 1;
    params.enable_process_noise = 1;
    params.process_noise_std_m3_s = 1.0e-7f;

    memset(&out_a, 0, sizeof(out_a));
    memset(&out_b, 0, sizeof(out_b));
    PressureModel_Reset(&state_a, 0x44444444u);
    PressureModel_Reset(&state_b, 0x44444444u);

    for (i = 0; i < 500; ++i) {
        PressureModel_Step(&params, &state_a, 800.0f, DT_S, &out_a);
        PressureModel_Step(&params, &state_b, 800.0f, DT_S, &out_b);
        ASSERT_NEAR(out_a.measured_pressure_bar, out_b.measured_pressure_bar, 1e-6f,
                    "fixed seed should reproduce the same measured-pressure trace");
        ASSERT_NEAR(out_a.actual_motor_rpm, out_b.actual_motor_rpm, 1e-6f,
                    "fixed seed should reproduce the same motor-speed trace");
    }
}
```

并在 `main()` 中追加：

```c
    test_negative_speed_depressurizes_faster_than_passive_leak();
    test_tooth_drop_is_visible_once_per_tooth();
    test_relief_caps_measured_output_at_two_hundred_fifty_bar();
    test_noise_control_is_repeatable_with_fixed_seed();
```

- [ ] **Step 2: 跑测试，确认当前基础模型对这些要求仍然失败**

Run:

```bash
cmake --build --preset unixgcc --target test_pressure_model
ctest --test-dir out/build/unixgcc -R '^test_pressure_model$' --output-on-failure
```

Expected: suite fails on tooth valleys, repeatable noise, or reverse depressurization behavior.

- [ ] **Step 3: 在头文件和实现里补齐噪声/RNG/13 齿显示逻辑**

先确认 `include/pressure_model.h` 已包含以下字段；如果 Task 1 已写入则不需再改：

```c
    float process_noise_std_m3_s;
    float flow_ripple_ratio;
    float tooth_drop_depth_ratio;
    float tooth_drop_width_ratio;
    unsigned char enable_sensor_noise;
    unsigned char enable_motor_noise;
    unsigned char enable_process_noise;
```

然后在 `src/sim/PressureModel.c` 中加入随机数和相位辅助函数：

```c
#define PRESSURE_MODEL_PI 3.14159265358979323846f

static float pressure_model_wrap_unit(float value) {
    while (value >= 1.0f) value -= 1.0f;
    while (value < 0.0f) value += 1.0f;
    return value;
}

static uint32_t pressure_model_next_u32(PressureModelState* state) {
    uint32_t x = pressure_model_seed(state->rng_state);

    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    state->rng_state = pressure_model_seed(x);
    return state->rng_state;
}

static float pressure_model_uniform01(PressureModelState* state) {
    return (pressure_model_next_u32(state) & 0x00ffffffu) / 16777216.0f;
}

static float pressure_model_gaussian(PressureModelState* state, float stddev) {
    float u1;
    float u2;
    float mag;

    if (stddev <= 0.0f) return 0.0f;

    if (state->has_spare_gauss) {
        state->has_spare_gauss = 0;
        return stddev * state->spare_gauss;
    }

    u1 = pressure_model_uniform01(state);
    u2 = pressure_model_uniform01(state);
    if (u1 < 1.0e-7f) u1 = 1.0e-7f;

    mag = sqrtf(-2.0f * logf(u1));
    state->spare_gauss = mag * sinf(2.0f * PRESSURE_MODEL_PI * u2);
    state->has_spare_gauss = 1;
    return stddev * (mag * cosf(2.0f * PRESSURE_MODEL_PI * u2));
}
```

再把 `PressureModel_Step(...)` 更新为完整版本：

```c
void PressureModel_Step(const PressureModelParams* params,
                        PressureModelState* state,
                        float target_rpm,
                        float dt_s,
                        PressureModelOutput* out) {
    float dt;
    float clamped_target;
    float alpha;
    float motor_noise = 0.0f;
    float process_noise = 0.0f;
    float sensor_noise = 0.0f;
    float q_base;
    float q_pump;
    float q_leak;
    float q_relief = 0.0f;
    float q_net;
    float d_pressure;
    float tooth_phase;
    float visible_pressure_pa;

    if (params == NULL || state == NULL || out == NULL) return;

    dt = (dt_s > 0.0f) ? dt_s : PRESSURE_MODEL_DEFAULT_DT_S;
    clamped_target = pressure_model_clampf(target_rpm, params->min_rpm, params->max_rpm);
    alpha = dt / (params->motor_tau_s + dt);
    if (params->enable_motor_noise) {
        motor_noise = pressure_model_gaussian(state, params->motor_noise_std_rpm);
    }

    state->motor_rpm += alpha * (clamped_target - state->motor_rpm) + motor_noise;
    state->motor_rpm = pressure_model_clampf(state->motor_rpm, params->min_rpm, params->max_rpm);
    state->pump_phase_rev = pressure_model_wrap_unit(
        state->pump_phase_rev + (state->motor_rpm * dt / 60.0f));

    q_base = params->pump_displacement_m3_rev * (state->motor_rpm / 60.0f);
    q_pump = q_base;
    if (state->motor_rpm > 0.01f) {
        q_pump *= 1.0f + params->flow_ripple_ratio *
                  sinf(2.0f * PRESSURE_MODEL_PI * 13.0f * state->pump_phase_rev);
    }

    if (params->enable_process_noise) {
        process_noise = pressure_model_gaussian(state, params->process_noise_std_m3_s);
    }

    q_leak = params->leak_coeff_m3_pa_s * state->pressure_pa;
    if (state->pressure_pa > params->relief_set_pa) {
        q_relief = params->relief_coeff_m3_pa_s * (state->pressure_pa - params->relief_set_pa);
    }

    q_net = q_pump - q_leak - q_relief + process_noise;
    d_pressure = (params->bulk_modulus_pa / params->chamber_volume_m3) * q_net * dt;
    state->pressure_pa = pressure_model_maxf(0.0f, state->pressure_pa + d_pressure);

    visible_pressure_pa = state->pressure_pa;
    if (state->motor_rpm > 0.01f) {
        tooth_phase = pressure_model_wrap_unit(13.0f * state->pump_phase_rev);
        if (tooth_phase < params->tooth_drop_width_ratio) {
            float window = 0.5f * (1.0f + cosf((2.0f * PRESSURE_MODEL_PI * tooth_phase) /
                                               params->tooth_drop_width_ratio));
            float gain = 1.0f - params->tooth_drop_depth_ratio * window;
            visible_pressure_pa *= gain;
        }
    }

    if (params->enable_sensor_noise) {
        sensor_noise = pressure_model_gaussian(state, params->sensor_noise_std_bar);
    }

    out->actual_motor_rpm = state->motor_rpm;
    out->real_pressure_bar = state->pressure_pa * 1.0e-5f;
    out->measured_pressure_bar = pressure_model_clampf((visible_pressure_pa * 1.0e-5f) +
                                                       params->sensor_bias_bar + sensor_noise,
                                                       0.0f,
                                                       params->sensor_range_bar);
    out->pump_flow_m3_s = q_pump;
    out->net_flow_m3_s = q_net;
    out->relief_active = (q_relief > 0.0f) ? 1 : 0;
}
```

- [ ] **Step 4: 跑独立对象模型测试，确认全部通过**

Run:

```bash
cmake --build --preset unixgcc --target test_pressure_model
ctest --test-dir out/build/unixgcc -R '^test_pressure_model$' --output-on-failure
```

Expected: `test_pressure_model` passes all plant-model checks.

- [ ] **Step 5: Commit**

```bash
git add include/pressure_model.h src/sim/PressureModel.c tests/test_pressure_model.c
git commit --no-gpg-sign -m "Model the 13-tooth pressure signature and deterministic noise paths" -m "Completes the approved plant behavior with positive-speed ripple, per-tooth visible pressure drops, controlled noise sources, reverse depressurization, and relief-limited measurement output.

Constraint: The pressure model must stay deterministic under fixed-seed test mode
Rejected: Hiding tooth drops entirely inside the real-pressure state | The approved design separates chamber pressure from the visible sensor signature
Confidence: high
Scope-risk: moderate
Directive: Add future noise sources only behind explicit params/state controls so tests remain reproducible
Tested: cmake --build --preset unixgcc --target test_pressure_model; ctest --test-dir out/build/unixgcc -R '^test_pressure_model$' --output-on-failure
Not-tested: PLC FB scan persistence and legacy pressure_update wrapper behavior are not verified yet"
```

---

### Task 4: 将 PLC FB 改为显式状态持有，并补 FB 回归测试

**Files:**
- Modify: `src/sim/hydro_sim_fb.c`
- Modify: `tests/test_hydro_sim_fb.c`

- [ ] **Step 1: 先写 FB 级失败测试，锁定“跨拍累积 + disable 清零 + 同种子重启一致”**

在 `tests/test_hydro_sim_fb.c` 中追加：

```c
static void test_pressure_model_fb_persists_state_and_resets_on_disable(void) {
    HYD_PRESSUREMODEL cmd;
    double first_step_rpm;
    double first_step_real_pressure;

    memset(&cmd, 0, sizeof(cmd));

    cmd.ENABLE.value = true;
    cmd.MOTOR_RPM.value = 1000.0;
    cmd.TIME_S.value = 0.000;
    __mcl_cmd_updatePressureModel(&cmd);

    first_step_rpm = cmd.ACTUAL_MOTOR_RPM.value;
    first_step_real_pressure = cmd.REAL_PRESSURE_BAR.value;

    ASSERT_TRUE(cmd.ACTIVE.value, "PressureModel FB should become active when enabled");
    ASSERT_TRUE(first_step_rpm > 0.0, "First enabled step should accelerate the motor");

    cmd.TIME_S.value = 0.001;
    __mcl_cmd_updatePressureModel(&cmd);

    ASSERT_TRUE(cmd.ACTUAL_MOTOR_RPM.value > first_step_rpm,
                "Second enabled step should continue from prior motor state");
    ASSERT_TRUE(cmd.REAL_PRESSURE_BAR.value >= first_step_real_pressure,
                "Second enabled step should not restart the pressure state");

    cmd.ENABLE.value = false;
    __mcl_cmd_updatePressureModel(&cmd);

    ASSERT_TRUE(!cmd.ACTIVE.value, "Disabling PressureModel FB should clear ACTIVE");
    ASSERT_NEAR(cmd.REAL_PRESSURE_BAR.value, 0.0, TOLERANCE,
                "Disabling PressureModel FB should reset real pressure output");
    ASSERT_NEAR(cmd.MEASURED_PRESSURE_BAR.value, 0.0, TOLERANCE,
                "Disabling PressureModel FB should reset measured pressure output");
    ASSERT_NEAR(cmd.ACTUAL_MOTOR_RPM.value, 0.0, TOLERANCE,
                "Disabling PressureModel FB should reset actual motor rpm output");

    cmd.ENABLE.value = true;
    cmd.MOTOR_RPM.value = 1000.0;
    cmd.TIME_S.value = 0.000;
    __mcl_cmd_updatePressureModel(&cmd);

    ASSERT_NEAR(cmd.ACTUAL_MOTOR_RPM.value, first_step_rpm, 1e-6,
                "Re-enable after reset should replay the same first-step motor rpm");
    ASSERT_NEAR(cmd.REAL_PRESSURE_BAR.value, first_step_real_pressure, 1e-6,
                "Re-enable after reset should replay the same first-step real pressure");
}
```

并在 `main()` 中追加调用：

```c
    test_pressure_model_fb_persists_state_and_resets_on_disable();
```

- [ ] **Step 2: 跑 FB 测试，确认它先失败**

Run:

```bash
cmake --build --preset unixgcc --target test_hydro_sim_fb
ctest --test-dir out/build/unixgcc -R '^test_hydro_sim_fb$' --output-on-failure
```

Expected: current implementation fails because it only keeps `last_pressure`,每拍重建 `motor_state`，且没有可重复的 reset/reseed 逻辑。

- [ ] **Step 3: 在 `src/sim/hydro_sim_fb.c` 中改为持有显式 params/state，并按 `TIME_S` 求 `dt_s`**

在文件顶部加入新 include 和静态状态：

```c
#include "pressure_model.h"

static PressureModelParams g_pressure_model_params;
static PressureModelState g_pressure_model_state;
static int g_pressure_model_initialized = 0;
static int g_pressure_model_have_time = 0;
static float g_pressure_model_last_time_s = 0.0f;
static const unsigned int kPressureModelSeed = 0x13572468u;

static void PressureModelFb_ResetOutputs(HYD_PRESSUREMODEL *data__) {
    __SET_VAR(data__->, REAL_PRESSURE_BAR,, 0.0f);
    __SET_VAR(data__->, MEASURED_PRESSURE_BAR,, 0.0f);
    __SET_VAR(data__->, ACTUAL_MOTOR_RPM,, 0.0f);
    __SET_VAR(data__->, ACTIVE,, 0);
}

static void PressureModelFb_ResetState(void) {
    PressureModel_Reset(&g_pressure_model_state, kPressureModelSeed);
    g_pressure_model_have_time = 0;
    g_pressure_model_last_time_s = 0.0f;
}

static void PressureModelFb_EnsureInitialized(void) {
    if (g_pressure_model_initialized) return;

    PressureModel_InitParams(&g_pressure_model_params);
    PressureModelFb_ResetState();
    g_pressure_model_initialized = 1;
}
```

然后把 `__mcl_cmd_updatePressureModel(...)` 替换为：

```c
void __mcl_cmd_updatePressureModel(HYD_PRESSUREMODEL *data__)
{
    float current_time;
    float dt_s;
    float target_motor_speed;
    PressureModelOutput out;

    if (data__ == NULL) return;

    PressureModelFb_EnsureInitialized();

    if (!__GET_VAR(data__->ENABLE)) {
        PressureModelFb_ResetState();
        PressureModelFb_ResetOutputs(data__);
        return;
    }

    current_time = __GET_VAR(data__->TIME_S);
    target_motor_speed = __GET_VAR(data__->MOTOR_RPM);
    if (g_pressure_model_have_time && current_time > g_pressure_model_last_time_s) {
        dt_s = current_time - g_pressure_model_last_time_s;
    } else {
        dt_s = 0.001f;
    }
    g_pressure_model_last_time_s = current_time;
    g_pressure_model_have_time = 1;

    memset(&out, 0, sizeof(out));
    PressureModel_Step(&g_pressure_model_params,
                       &g_pressure_model_state,
                       target_motor_speed,
                       dt_s,
                       &out);

    __SET_VAR(data__->, REAL_PRESSURE_BAR,, out.real_pressure_bar);
    __SET_VAR(data__->, MEASURED_PRESSURE_BAR,, out.measured_pressure_bar);
    __SET_VAR(data__->, ACTUAL_MOTOR_RPM,, out.actual_motor_rpm);
    __SET_VAR(data__->, ACTIVE,, 1);
}
```

- [ ] **Step 4: 重新跑 FB 测试，确认状态保持和 disable reset 都通过**

Run:

```bash
cmake --build --preset unixgcc --target test_hydro_sim_fb
ctest --test-dir out/build/unixgcc -R '^test_hydro_sim_fb$' --output-on-failure
```

Expected: `test_hydro_sim_fb` passes, including the new pressure-model FB regression.

- [ ] **Step 5: Commit**

```bash
git add src/sim/hydro_sim_fb.c tests/test_hydro_sim_fb.c
git commit --no-gpg-sign -m "Stop rebuilding pressure-model dynamics inside the FB adapter" -m "Moves the HYD_PRESSUREMODEL FB onto explicit params/state storage with deterministic reset semantics and TIME_S-based time increments.

Constraint: The FB surface stays single-instance and scan-driven; no expansion into the general simulator core
Rejected: Continuing to route the FB through pressure_update hidden statics | That path cannot preserve all required model state cleanly
Confidence: high
Scope-risk: narrow
Directive: Any future FB-side tuning must happen through PressureModelParams rather than ad hoc local statics
Tested: cmake --build --preset unixgcc --target test_hydro_sim_fb; ctest --test-dir out/build/unixgcc -R '^test_hydro_sim_fb$' --output-on-failure
Not-tested: Legacy pressure_update compatibility behavior is still only build-covered"
```

---

### Task 5: 补 legacy 包装层烟雾测试并做最终验证

**Files:**
- Modify: `src/sim/PressureModel.c`
- Modify: `tests/test_pressure_model.c`

- [ ] **Step 1: 给旧接口 `pressure_update(...)` 加一个失败烟雾测试**

在 `tests/test_pressure_model.c` 中追加：

```c
static void test_legacy_pressure_update_keeps_motor_state_between_calls(void) {
    float pressure_state = 0.0f;
    float real_pressure0 = 0.0f;
    float real_pressure1 = 0.0f;
    float rpm0 = 0.0f;
    float rpm1 = 0.0f;
    float measured0;
    float measured1;

    measured0 = pressure_update(1000.0f, 0.000f, &pressure_state, &real_pressure0, &rpm0);
    measured1 = pressure_update(1000.0f, 0.001f, &pressure_state, &real_pressure1, &rpm1);

    ASSERT_TRUE(rpm1 > rpm0,
                "legacy pressure_update should preserve motor state between calls");
    ASSERT_TRUE(real_pressure1 >= real_pressure0,
                "legacy pressure_update should preserve pressure state between calls");
    ASSERT_TRUE(measured0 >= 0.0f && measured1 >= 0.0f,
                "legacy pressure_update should keep measured pressure non-negative");
}
```

并在 `main()` 中追加：

```c
    test_legacy_pressure_update_keeps_motor_state_between_calls();
```

- [ ] **Step 2: 跑独立测试，确认旧包装层仍然失败或不够稳**

Run:

```bash
cmake --build --preset unixgcc --target test_pressure_model
ctest --test-dir out/build/unixgcc -R '^test_pressure_model$' --output-on-failure
```

Expected: the new wrapper smoke test fails until `pressure_update(...)` tracks `dt_s` and keeps its own compatibility state.

- [ ] **Step 3: 把 `pressure_update(...)` 改为基于显式兼容状态的薄包装**

在 `src/sim/PressureModel.c` 中用下面的实现替换 Task 1 的临时包装层：

```c
float pressure_update(float target_rpm,
                      float t,
                      float *P_state,
                      float *real_P,
                      float *actual_motor_rpm) {
    static PressureModelParams params;
    static PressureModelState state;
    static float last_time_s = 0.0f;
    static int initialized = 0;
    PressureModelOutput out;
    float dt_s;

    if (!initialized) {
        PressureModel_InitParams(&params);
        PressureModel_Reset(&state, 0x2468ace1u);
        initialized = 1;
    }

    if (P_state != NULL) {
        state.pressure_pa = *P_state;
    }

    if (t > last_time_s) {
        dt_s = t - last_time_s;
    } else {
        dt_s = PRESSURE_MODEL_DEFAULT_DT_S;
    }
    last_time_s = t;

    memset(&out, 0, sizeof(out));
    PressureModel_Step(&params, &state, target_rpm, dt_s, &out);

    if (P_state != NULL) {
        *P_state = state.pressure_pa;
    }
    if (real_P != NULL) {
        *real_P = out.real_pressure_bar;
    }
    if (actual_motor_rpm != NULL) {
        *actual_motor_rpm = out.actual_motor_rpm;
    }

    return out.measured_pressure_bar;
}
```

- [ ] **Step 4: 运行最终针对性验证**

Run:

```bash
cmake --build --preset unixgcc --target test_pressure_model test_hydro_sim_fb test_rbf_pid_hil
ctest --test-dir out/build/unixgcc -R '^(test_pressure_model|test_hydro_sim_fb|test_rbf_pid_hil)$' --output-on-failure
```

Expected: all three tests pass. `test_rbf_pid_hil` serves as a HydroSimLib smoke check after touching sim sources.

- [ ] **Step 5: 运行更宽的回归检查**

Run:

```bash
cmake --build --preset unixgcc
ctest --test-dir out/build/unixgcc --output-on-failure
```

Expected: full suite passes. If the full suite is too slow for the current execution lane, record that and at minimum keep Step 4 green before stopping.

- [ ] **Step 6: Commit**

```bash
git add src/sim/PressureModel.c tests/test_pressure_model.c
git commit --no-gpg-sign -m "Preserve the legacy pressure-model entry point while finishing verification" -m "Finalizes the compatibility wrapper around PressureModel_Step and closes the verification loop with direct plant tests, FB tests, and a HydroSimLib smoke run.

Constraint: pressure_update must remain compatible enough for older callers while no longer being the primary model surface
Rejected: Deleting pressure_update immediately | The current repo still exports the symbol through hydro_sim.h
Confidence: medium
Scope-risk: narrow
Directive: Remove the legacy wrapper only after all in-repo callers migrate to pressure_model.h
Tested: cmake --build --preset unixgcc --target test_pressure_model test_hydro_sim_fb test_rbf_pid_hil; ctest --test-dir out/build/unixgcc -R '^(test_pressure_model|test_hydro_sim_fb|test_rbf_pid_hil)$' --output-on-failure; cmake --build --preset unixgcc; ctest --test-dir out/build/unixgcc --output-on-failure
Not-tested: No external, out-of-repo callers of pressure_update were exercised"
```
