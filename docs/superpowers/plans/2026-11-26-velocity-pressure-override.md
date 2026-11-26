# 速度段限压改为压力环取小（pQ override）Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在位置/速度段（MoveAbsolute、MoveVelocity、MoveContinuousAbsolute、Stop 减速）中，把输出限幅器里的比例缩放限压改为"速度流量与限压 PI 取小"，使实际压力稳定在 PRESSURELIMIT（±2 bar），负载下降后无扰回到速度控制。

**Architecture:**
- 限压 PI 写在 `src/output_limiter.c` 内，约 60 行，不复用 `HYD_PressureController`。
- 未选中时，积分跟踪上一拍实际执行流量 `fb->_lastCommandedFlow`（外部复位反馈）。
- 增益走新参数 40/41，单位与保压 PI 参数 11/14 相同。
- 另修一处：转保压时，保压 PI 的初值改用实际执行流量。
- PLC 侧只增加参数写入和一个状态输出。

**Tech Stack:** C99（`HYD_REAL` = float），CMake + CTest，IEC 61131-3 ST（matiec），Python 3 标准库脚本。

**Spec:** `docs/superpowers/specs/2026-11-26-velocity-pressure-override-design.md`

## Global Constraints

- 纯 C99；新代码只用 `HYD_REAL` 和带 `f` 后缀的常量，不能有隐式 double 提升（`-Wdouble-promotion` 零告警）；不使用 malloc。
- 调参类浮点阈值放在 `include/hyd_config.h` §14B，不写死在 `.c` 里。
- **行尾**：
  - `src/motion_control.c` 和 `iec/FB_HydAxis.st` 是 CRLF，编辑后必须仍然全部是 CRLF；
  - 其余要改的文件是 LF。
  - 用 Python 改时按二进制读写，或者先归一再恢复 CRLF。
- **WSL 运行方式**：仓库在 WSL Ubuntu-22.04 的 `~/project/hdy-motion-light`。从 Git Bash 调 `wsl.exe` 时 `$VAR`、heredoc、awk `$1` 会丢失；多于一条的命令写成 `.sh` 文件再用 `bash` 执行。
- **基线**：开工前 `ctest` 为 62/64。`test_parameter_iec` 和 `test_motion_interface_unit` 已经失败，原因是 b3c78ff 把 PressureHandle 基准流量 20 改成了 100 L/min，与本计划无关。
  - 不修这两个测试；
  - 每个任务结束时，这两个测试的失败断言集合必须和基线完全相同；
  - 其余 62 个测试必须全部通过。
- **参数定义**：
  - 40 = `HYD_PARAM_PRESSURE_LIMIT_KP`，单位 L/min/bar；
  - 41 = `HYD_PARAM_PRESSURE_LIMIT_KI`，单位 L/min/(bar·s)；
  - 0 表示使用 §14B 默认值 1.0 / 16；
  - 写入负数或非有限值时拒绝；
  - `HYD_PARAM_COUNT` = 42。
- 压力闭环段（`HYD_MODE_PRESSURE_CLOSED_LOOP`）、RBF、2DOF、纹波补偿的代码路径不改，相关测试数值必须不变。
- 限压期间运动块保持 Busy，不出 Done、CommandAborted、Error，不产生库诊断码；状态只通过 `HYD_LIMIT_FLAG_PRESSURE` / `HYD_ReadStatus.PRESSURELIMITACTIVE` 输出。
- 每个任务单独提交，提交信息用中文，末尾加：
  `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`

**常用命令**（仓库根目录，WSL 内执行）：

```bash
cmake --preset unixgcc                                  # 仅在新增 .c 文件后需要
cmake --build --preset unixgcc -j8
ctest --test-dir out/build/unixgcc --output-on-failure
ctest --test-dir out/build/unixgcc -R '^test_output_limiter$' --output-on-failure
```

**原型验证**：本计划的全部代码都在仓库外的临时副本中做过原型，未入库。
- 全量 ctest：62/64，失败的 2 个与基线完全相同。
- T3、T5 在回退各自的修复后会失败，恢复后通过。
- 实测 FB 尺寸 3440 B，限幅状态 20 B。
- `-Wdouble-promotion` 零告警。
- FB 级堵转仿真（Kq 1.2–3.2，伺服纯滞后 5 ms，Plim 140 bar）：所有工况稳态 140.0 bar，FB 保持 Busy、无 Error。

---

## File Structure

| 文件 | 职责 | 任务 |
|---|---|---|
| `include/common_types.h` | 参数枚举 40/41，`HYD_MotionFBParams` 两个新字段 | 1 |
| `src/motion_control.c`（CRLF） | 参数初始化、读写；限幅器输入接线；转保压初值 | 1, 2, 3 |
| `include/hyd_config.h` | 删除比例缩放常量，新增 PI 默认增益 | 2 |
| `include/output_limiter.h` | 限幅器输入、状态结构 | 2 |
| `src/output_limiter.c` | 限压 PI 取小，替换比例缩放、回差、双速恢复 | 2 |
| `tests/test_parameter_access.c` | 参数 40/41 读写 | 1 |
| `tests/test_mode_switch_bumpless.c` | FB 尺寸上限；T3 换段连续性；T5 转保压初值 | 1, 2, 3 |
| `tests/test_toggle_mechanism_pool.c` | FB 尺寸基线 | 1 |
| `tests/test_output_limiter.c` | 限压 PI 单元测试，替换比例缩放测试 | 2 |
| `tests/test_movevelocity_stop_reverse_restart.c` | IEC 级限压断言改为 PI 语义 | 2 |
| `scripts/identify_pressure_plant.py` | 列名选项、螺杆静止过滤、参数 40/41 建议值 | 4 |
| `iec/ST_HydAxisRef.st`、`iec/FB_HydAxis.st`（CRLF） | 写入 40/41，输出 `bPressureLimited` | 5 |

---

### Task 1: 参数 40/41（限压 PI 增益）

**Files:**
- Modify: `include/common_types.h`（`HYD_ParameterNumber` 末尾，约 604 行；`HYD_MotionFBParams` 末尾，约 642 行）
- Modify: `src/motion_control.c`（Init 约 4121 行；ReadParameter 约 4750 行；WriteParameter 约 4849 行）——CRLF
- Modify: `tests/test_parameter_access.c`
- Modify: `tests/test_mode_switch_bumpless.c:450`，`tests/test_toggle_mechanism_pool.c:7`

**Interfaces:**
- Produces: `HYD_PARAM_PRESSURE_LIMIT_KP` (= 40)、`HYD_PARAM_PRESSURE_LIMIT_KI` (= 41)，`fb->_params.pressureLimitKp`、`fb->_params.pressureLimitKi`（`HYD_REAL`，Init 后为 0）。Task 2 读这两个字段。

- [ ] **Step 1: 写失败测试**

在 `tests/test_parameter_access.c` 的 `static void test_pressure_controller_type_validation(void) {` 前插入：

```c
static void test_pressure_limit_pi_parameters_roundtrip_and_validation(void) {
    HYD_MotionControlFB fb;
    HYD_REAL value = -1.0f;
    HYD_BOOL ok;

    HYD_MotionControlFB_Init(&fb);

    ASSERT_EQ((int)HYD_PARAM_PRESSURE_LIMIT_KP, 40, "Pressure-limit Kp is parameter 40");
    ASSERT_EQ((int)HYD_PARAM_PRESSURE_LIMIT_KI, 41, "Pressure-limit Ki is parameter 41");
    ASSERT_EQ((int)HYD_PARAM_COUNT, 42, "Parameter count follows parameter 41");

    ok = HYD_MotionControlFB_ReadParameter(&fb, HYD_PARAM_PRESSURE_LIMIT_KP, &value);
    ASSERT_TRUE(ok, "Pressure-limit Kp should be readable");
    ASSERT_FLOAT_EQ(value, 0.0f, 0.0f, "Pressure-limit Kp should default to zero (built-in default)");
    ok = HYD_MotionControlFB_ReadParameter(&fb, HYD_PARAM_PRESSURE_LIMIT_KI, &value);
    ASSERT_TRUE(ok, "Pressure-limit Ki should be readable");
    ASSERT_FLOAT_EQ(value, 0.0f, 0.0f, "Pressure-limit Ki should default to zero (built-in default)");

    ok = HYD_MotionControlFB_WriteParameter(&fb, HYD_PARAM_PRESSURE_LIMIT_KP, 2.2f);
    ASSERT_TRUE(ok, "Positive pressure-limit Kp should be writable");
    ok = HYD_MotionControlFB_ReadParameter(&fb, HYD_PARAM_PRESSURE_LIMIT_KP, &value);
    ASSERT_FLOAT_EQ(value, 2.2f, 0.0001f, "Pressure-limit Kp should round-trip in L/min/bar");
    ok = HYD_MotionControlFB_WriteParameter(&fb, HYD_PARAM_PRESSURE_LIMIT_KI, 35.0f);
    ASSERT_TRUE(ok, "Positive pressure-limit Ki should be writable");
    ok = HYD_MotionControlFB_ReadParameter(&fb, HYD_PARAM_PRESSURE_LIMIT_KI, &value);
    ASSERT_FLOAT_EQ(value, 35.0f, 0.0001f, "Pressure-limit Ki should round-trip in L/min/(bar*s)");

    ASSERT_TRUE(!HYD_MotionControlFB_WriteParameter(&fb, HYD_PARAM_PRESSURE_LIMIT_KP, -1.0f),
                "Negative pressure-limit Kp should be rejected");
    ASSERT_TRUE(!HYD_MotionControlFB_WriteParameter(&fb, HYD_PARAM_PRESSURE_LIMIT_KI, NAN),
                "Non-finite pressure-limit Ki should be rejected");
    ok = HYD_MotionControlFB_ReadParameter(&fb, HYD_PARAM_PRESSURE_LIMIT_KP, &value);
    ASSERT_FLOAT_EQ(value, 2.2f, 0.0001f, "Rejected writes keep the last accepted Kp");
    ASSERT_TRUE(HYD_MotionControlFB_WriteParameter(&fb, HYD_PARAM_PRESSURE_LIMIT_KP, 0.0f),
                "Zero pressure-limit Kp should select the built-in default");
}
```

在 `main()` 中 `test_pressure_model_parameters_roundtrip_and_validation();` 的下一行加：

```c
    test_pressure_limit_pi_parameters_roundtrip_and_validation();
```

- [ ] **Step 2: 运行，确认失败**

Run: `cmake --build --preset unixgcc -j8 2>&1 | grep -m3 error`
Expected: `‘HYD_PARAM_PRESSURE_LIMIT_KP’ undeclared`

- [ ] **Step 3: 实现**

`include/common_types.h`，把

```c
    HYD_PARAM_PUMP_RIPPLE_ORDER,        /* ripples per pump revolution (integer); 0 disables compensation */
    HYD_PARAM_COUNT
```

改为

```c
    HYD_PARAM_PUMP_RIPPLE_ORDER,        /* ripples per pump revolution (integer); 0 disables compensation */
    HYD_PARAM_PRESSURE_LIMIT_KP,        /* L/min per bar; motion-segment pressure-limit PI, 0 uses HYD_THRESH_PRESSURE_LIMIT_PI_KP */
    HYD_PARAM_PRESSURE_LIMIT_KI,        /* L/min per (bar*s); motion-segment pressure-limit PI, 0 uses HYD_THRESH_PRESSURE_LIMIT_PI_KI */
    HYD_PARAM_COUNT
```

再把

```c
    HYD_REAL pumpRippleOrder;            /* ripples per pump revolution (integer); 0 disables */
} HYD_MotionFBParams;
```

改为

```c
    HYD_REAL pumpRippleOrder;            /* ripples per pump revolution (integer); 0 disables */
    HYD_REAL pressureLimitKp;            /* L/min per bar; pressure-limit PI, 0 uses the built-in default */
    HYD_REAL pressureLimitKi;            /* L/min per (bar*s); pressure-limit PI, 0 uses the built-in default */
} HYD_MotionFBParams;
```

`src/motion_control.c` 共三处，保持 CRLF：

```c
    fb->_params.pumpRippleOrder = 0.0f;
    fb->_params.pressureLimitKp = 0.0f;
    fb->_params.pressureLimitKi = 0.0f;
```

（Init 中，紧跟在已有的 `fb->_params.pumpRippleOrder = 0.0f;` 之后。）

```c
        case HYD_PARAM_PUMP_RIPPLE_ORDER:              *value = fb->_params.pumpRippleOrder; break;
        case HYD_PARAM_PRESSURE_LIMIT_KP:              *value = fb->_params.pressureLimitKp; break;
        case HYD_PARAM_PRESSURE_LIMIT_KI:              *value = fb->_params.pressureLimitKi; break;
```

（ReadParameter 的 switch。）

```c
            fb->_params.pumpRippleOrder = value;
            break;
        case HYD_PARAM_PRESSURE_LIMIT_KP:
            if (!isfinite(value) || value < 0.0) return false;
            fb->_params.pressureLimitKp = value;
            break;
        case HYD_PARAM_PRESSURE_LIMIT_KI:
            if (!isfinite(value) || value < 0.0) return false;
            fb->_params.pressureLimitKi = value;
            break;
```

（WriteParameter 的 switch，接在 `HYD_PARAM_PUMP_RIPPLE_ORDER` 分支的 `break;` 之后、`default:` 之前。）

FB 每轴增加 8 B，同步更新两处尺寸上限：

`tests/test_mode_switch_bumpless.c` 改为

```c
     * +104: 压力环二自由度（参考延迟线、轨迹状态、参数 37–39、纹波补偿系数）。
     * +8: 限压 PI 参数 40/41（限幅状态 pressureScale/LastTime 换成积分，−4，被对齐吃掉）。 */
    assert(sizeof(HYD_MotionControlFB) <= 3440U);
```

`tests/test_toggle_mechanism_pool.c:7` 改为

```c
#define HYD_BASELINE_MOTION_FB_BYTES 3392U /* host ABI without toggle telemetry; +104 for the 2-DOF pressure loop (2026-11-23); +8 pressure-limit PI params 40/41 (2026-11-26) */
```

- [ ] **Step 4: 运行，确认通过**

Run:
```bash
cmake --build --preset unixgcc -j8 && ctest --test-dir out/build/unixgcc -R '^(test_parameter_access|test_mode_switch_bumpless|test_toggle_mechanism_pool)$' --output-on-failure
```
Expected: 3/3 passed；`test_parameter_access` 输出 `125/125 passed`；`test_mode_switch_bumpless` 打印 `motion_control_fb=3440 bytes`。

Run: `ctest --test-dir out/build/unixgcc 2>&1 | tail -4`
Expected: `97% tests passed, 2 tests failed out of 64`，失败的只有 `test_parameter_iec` 和 `test_motion_interface_unit`。

Run: `grep -c $'\r$' src/motion_control.c; wc -l < src/motion_control.c`
Expected: 两个数相等。

- [ ] **Step 5: 提交**

```bash
git add include/common_types.h src/motion_control.c tests/test_parameter_access.c tests/test_mode_switch_bumpless.c tests/test_toggle_mechanism_pool.c
git commit -m "参数 40/41：速度段限压 PI 的 Kp、Ki（spec 2026-11-26 §4.4）" -m "单位 L/min/bar 与 L/min/(bar·s)，与参数 11/14 同；0 用 §14B 默认值，负数和非有限值拒绝写入。FB 每轴 +8 B（3440 B）。" -m "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 2: 限压 PI 取小，替换比例缩放（含换段连续性 T3）

**Files:**
- Modify: `include/hyd_config.h:438-464`（压力限制保护阈值段）
- Modify: `include/output_limiter.h`
- Modify: `src/output_limiter.c`（`ResetState`；比例缩放与状态函数整体替换；`ExecuteWithProtection` 的第 2–5 步）
- Modify: `src/motion_control.c`（`HYD_MotionControlFB_RunRunningState` 中 `limiterInput.actualPressure = fb->AXIS_REF.pressure;` 之后，约 3453 行）——CRLF
- Modify: `tests/test_output_limiter.c`、`tests/test_movevelocity_stop_reverse_restart.c`、`tests/test_mode_switch_bumpless.c`

**Interfaces:**
- Consumes: `fb->_params.pressureLimitKp/Ki`（Task 1）。
- Produces:
  - `HYD_OutputLimiterInput` 新增 `pressureLimitKp`、`pressureLimitKi`、`previousCommandFlow`、`deltaTime`（均为 `HYD_REAL`）；
  - `HYD_OutputLimiterState` 删除 `pressureScale`、`pressureScaleLastTime`，新增 `HYD_REAL pressureLimitIntegral`；
  - `HYD_OutputLimiterOutput.pressureLimitActive` 的含义改为"本拍压力环被选中"；
  - 常量 `HYD_THRESH_PRESSURE_LIMIT_PI_KP` (1.0f)、`HYD_THRESH_PRESSURE_LIMIT_PI_KI` (16.0f)。

- [ ] **Step 1: 写失败测试（限幅器单元）**

在 `tests/test_output_limiter.c` 中删除以下 5 个函数及它们在 `main()` 中的调用，它们测的是旧的比例缩放和回差：
- `test_pressure_limit_proportional_reduction`
- `test_pressure_limit_zero_scale_clamp`
- `test_protection_takes_min_scale`
- `test_pressure_limit_stays_non_fault_and_hysteretic`
- `test_pressure_scale_recovers_fast_below_release_threshold`

在 `int main(void)` 前插入：

```c
/* 限压 PI（spec 2026-11-26 §4.1）公共输入：G = 20 rpm/(L/min)，泵 1700 rpm → Qmax 85 L/min，
 * Q_v = 50 L/min，Plim = 140 bar，参数 40/41 = 0 → 默认 Kp 1.0、Ki 16，dt = 1 ms。 */
static void init_pressure_limit_input(HYD_OutputLimiterInput* input) {
    memset(input, 0, sizeof(*input));
    input->requestedFlow = 50.0f;
    input->requestedPumpSpeed = 1000.0f;
    input->flowToPumpSpeedGain = 20.0f;
    input->pumpSpeedLimit = 1700.0f;
    input->protectionAction = HYD_PROTECTION_ACTION_NONE;
    input->effectiveMaxPressure = 140.0f;
    input->previousCommandFlow = 50.0f;
    input->deltaTime = 0.001f;
    input->currentTime = 1.0f;
}

static void test_pressure_limit_not_selected_below_limit(void) {
    HYD_OutputLimiterInput input;
    HYD_OutputLimiterState state;
    HYD_OutputLimiterOutput output;

    init_pressure_limit_input(&input);
    HYD_OutputLimiter_ResetState(&state);
    input.actualPressure = 100.0f;   /* Q_p = 1.0·40 + 50 = 90 > Q_v */

    HYD_OutputLimiter_ExecuteWithProtection(&input, &state, &output);

    assert(output.pressureLimitActive == false);
    assert(output.derated == false);
    assert_real_eq(output.commandFlow, 50.0f, 1e-4f, "below limit keeps Q_v");
    assert_real_eq(output.pumpSpeed, 1000.0f, 1e-3f, "below limit keeps pump speed");
    printf("test_pressure_limit_not_selected_below_limit PASSED\n");
}

static void test_pressure_limit_takes_over_before_limit_when_flow_rises(void) {
    HYD_OutputLimiterInput input;
    HYD_OutputLimiterState state;
    HYD_OutputLimiterOutput output;

    init_pressure_limit_input(&input);
    HYD_OutputLimiter_ResetState(&state);
    input.previousCommandFlow = 30.0f;  /* 上一拍 30，本拍 Q_v 50 */
    input.actualPressure = 135.0f;      /* Q_p = 1.0·5 + 30 = 35 < 50 */

    HYD_OutputLimiter_ExecuteWithProtection(&input, &state, &output);

    assert(output.pressureLimitActive == true);
    assert_real_eq(output.commandFlow, 35.0f, 1e-4f, "rise limited to previous flow + Kp*e");
    assert_real_eq(output.pumpSpeed, 700.0f, 1e-3f, "pump speed follows limited flow");
    assert_real_eq(state.pressureLimitIntegral, 30.08f, 1e-4f, "integral = 30 + 16*5*0.001");
    printf("test_pressure_limit_takes_over_before_limit_when_flow_rises PASSED\n");
}

static void test_pressure_limit_over_limit_reduces_flow_with_integral(void) {
    HYD_OutputLimiterInput input;
    HYD_OutputLimiterState state;
    HYD_OutputLimiterOutput output;

    init_pressure_limit_input(&input);
    HYD_OutputLimiter_ResetState(&state);
    input.actualPressure = 150.0f;      /* Q_p = −10 + 50 = 40 */

    HYD_OutputLimiter_ExecuteWithProtection(&input, &state, &output);
    assert(output.pressureLimitActive == true);
    assert(output.diagnosticCode == HYD_DIAG_CODE_NONE);
    assert_real_eq(output.commandFlow, 40.0f, 1e-4f, "first over-limit scan");
    assert_real_eq(state.pressureLimitIntegral, 49.84f, 1e-4f, "integral = 50 − 16*10*0.001");

    /* 第二拍：沿用自身积分，不再取 previousCommandFlow */
    input.previousCommandFlow = 40.0f;
    input.currentTime = 1.001f;
    HYD_OutputLimiter_ExecuteWithProtection(&input, &state, &output);
    assert_real_eq(output.commandFlow, 39.84f, 1e-4f, "second scan uses own integral");
    printf("test_pressure_limit_over_limit_reduces_flow_with_integral PASSED\n");
}

static void test_pressure_limit_output_never_negative(void) {
    HYD_OutputLimiterInput input;
    HYD_OutputLimiterState state;
    HYD_OutputLimiterOutput output;

    init_pressure_limit_input(&input);
    HYD_OutputLimiter_ResetState(&state);
    input.actualPressure = 300.0f;      /* Q_p = −160 + 50 = −110 */

    HYD_OutputLimiter_ExecuteWithProtection(&input, &state, &output);

    assert(output.pressureLimitActive == true);
    assert_real_eq(output.commandFlow, 0.0f, 1e-6f, "motion segment flow never negative");
    assert_real_eq(output.pumpSpeed, 0.0f, 1e-6f, "motion segment pump speed never negative");
    assert_real_eq(state.pressureLimitIntegral, 47.44f, 1e-4f, "integral = 50 − 16*160*0.001");
    printf("test_pressure_limit_output_never_negative PASSED\n");
}

static void test_pressure_limit_uses_configured_gains(void) {
    HYD_OutputLimiterInput input;
    HYD_OutputLimiterState state;
    HYD_OutputLimiterOutput output;

    init_pressure_limit_input(&input);
    HYD_OutputLimiter_ResetState(&state);
    input.pressureLimitKp = 2.0f;
    input.pressureLimitKi = 32.0f;
    input.actualPressure = 150.0f;      /* Q_p = 2·(−10) + 50 = 30 */

    HYD_OutputLimiter_ExecuteWithProtection(&input, &state, &output);

    assert_real_eq(output.commandFlow, 30.0f, 1e-4f, "configured Kp");
    assert_real_eq(state.pressureLimitIntegral, 49.68f, 1e-4f, "configured Ki");
    printf("test_pressure_limit_uses_configured_gains PASSED\n");
}

/* 积分对象 dP/dt = Kq_f·(Q − P/R_f)，Kq_f = 64 bar/s/(L/min)，R_f = 104 bar/(L/min)，
 * 执行流量滞后一拍。Q_v = 10 L/min 堵转（油阀全关），默认增益。 */
static void test_pressure_limit_closed_loop_settles_at_limit_and_releases(void) {
    HYD_OutputLimiterInput input;
    HYD_OutputLimiterState state;
    HYD_OutputLimiterOutput output;
    HYD_REAL pressure = 60.0f;
    HYD_REAL applied = 10.0f;
    HYD_REAL peak = 0.0f;
    HYD_REAL previous;
    int step;

    init_pressure_limit_input(&input);
    HYD_OutputLimiter_ResetState(&state);
    input.requestedFlow = 10.0f;
    input.requestedPumpSpeed = 200.0f;
    input.previousCommandFlow = 10.0f;

    for (step = 0; step < 2000; ++step) {
        input.actualPressure = pressure;
        input.currentTime = 1.0f + 0.001f * (HYD_REAL)step;
        HYD_OutputLimiter_ExecuteWithProtection(&input, &state, &output);
        pressure += 64.0f * (applied - pressure / 104.0f) * 0.001f;
        applied = output.commandFlow;
        input.previousCommandFlow = output.commandFlow;
        if (pressure > peak) {
            peak = pressure;
        }
    }
    assert(output.pressureLimitActive == true);
    assert(fabs(pressure - 140.0f) < 0.5f);
    assert(peak < 154.0f);

    /* 负载下降：压力以 200 bar/s 降到 100 bar 并保持，流量连续回升到 Q_v，
     * 单拍增量不超过 Kp·|ΔP| + Ki·e·dt。 */
    previous = output.commandFlow;
    for (step = 0; step < 300; ++step) {
        HYD_REAL lastPressure = pressure;
        pressure = (pressure - 0.2f > 100.0f) ? pressure - 0.2f : 100.0f;
        input.actualPressure = pressure;
        HYD_OutputLimiter_ExecuteWithProtection(&input, &state, &output);
        assert(output.commandFlow >= previous - 1e-5f);
        assert(output.commandFlow - previous <=
               1.0f * (lastPressure - pressure) + 16.0f * (140.0f - pressure) * 0.001f + 1e-4f);
        previous = output.commandFlow;
        input.previousCommandFlow = output.commandFlow;
    }
    assert(output.pressureLimitActive == false);
    assert_real_eq(output.commandFlow, 10.0f, 1e-4f, "released back to Q_v");
    printf("test_pressure_limit_closed_loop_settles_at_limit_and_releases PASSED (peak %.2f bar)\n",
           (double)peak);
}

static void test_pressure_limit_and_soft_limit_take_minimum(void) {
    HYD_OutputLimiterInput input;
    HYD_OutputLimiterState state;
    HYD_OutputLimiterOutput output;

    /* 软限位：remaining 2 / band 5 → Q_v = 50 × 0.4 = 20 */
    init_pressure_limit_input(&input);
    HYD_OutputLimiter_ResetState(&state);
    input.actualPosition = 98.0f;
    input.strokeMm = 100.0f;
    input.softLimitBandMm = 5.0f;
    input.direction = HYD_DIRECTION_EXTEND;

    /* 压力 145：Q_p = −5 + 50 = 45 > 20 → 软限位决定输出 */
    input.actualPressure = 145.0f;
    HYD_OutputLimiter_ExecuteWithProtection(&input, &state, &output);
    assert(output.softLimitActive == true);
    assert(output.pressureLimitActive == false);
    assert_real_eq(output.commandFlow, 20.0f, 1e-4f, "soft limit wins");

    /* 压力 180：Q_p = −40 + 50 = 10 < 20 → 限压决定输出 */
    HYD_OutputLimiter_ResetState(&state);
    input.actualPressure = 180.0f;
    HYD_OutputLimiter_ExecuteWithProtection(&input, &state, &output);
    assert(output.softLimitActive == false);
    assert(output.pressureLimitActive == true);
    assert_real_eq(output.commandFlow, 10.0f, 1e-4f, "pressure limit wins");
    printf("test_pressure_limit_and_soft_limit_take_minimum PASSED\n");
}
```

`main()` 中原先调用 `test_pressure_limit_proportional_reduction();`、`test_pressure_limit_zero_scale_clamp();` 的位置换成：

```c
    test_pressure_limit_not_selected_below_limit();
    test_pressure_limit_takes_over_before_limit_when_flow_rises();
    test_pressure_limit_over_limit_reduces_flow_with_integral();
    test_pressure_limit_output_never_negative();
    test_pressure_limit_uses_configured_gains();
    test_pressure_limit_closed_loop_settles_at_limit_and_releases();
    test_pressure_limit_and_soft_limit_take_minimum();
```

保留的测试不动：`test_soft_limit_extend_deceleration`、`test_soft_limit_does_not_block_retract`、`test_no_protection_when_disabled`，以及 derate、negative-flow 系列。其中 `test_no_protection_when_disabled` 证明 `effectiveMaxPressure = 0` 时行为与以前逐位一致。

- [ ] **Step 2: 写失败测试（T3 换段连续性，FB 级）**

在 `tests/test_mode_switch_bumpless.c` 的 `static void test_reverse_position_handover_keeps_old_direction(void) {` 前插入：

```c
/* spec 2026-11-26 §4.2 / T3：限压中换段（限压 100 → 140，P = 124）不得一拍恢复到新段的
 * 速度流量。复现 vofa 20261125 8879 ms：217 rpm → 868 rpm。 */
static void test_pressure_limited_handover_continues_from_executed_flow(void) {
    HYD_MotionControlFB fb;
    HYD_MotionSegment source = make_speed_segment();
    HYD_MotionSegment successor = make_speed_segment();

    source.maxVelocity = 43.4;
    source.maxPressure = 100.0;
    successor.maxVelocity = 43.4;
    successor.maxPressure = 140.0;
    seed_active_segment(&fb, source, HYD_DIRECT_CMD_MOVE_VELOCITY);
    fb.FLOW_TO_PUMP_SPEED_GAIN = 20.0;
    fb.PUMP_SPEED_LIMIT = 1700.0;
    fb.AXIS_REF.pressure = 124.0;
    fb.PUMP_SPEED = 217.0;
    fb.STATE.commandedPumpSpeed = 217.0;
    fb._lastCommandedFlow = 10.85;               /* 限压后实际执行 217 rpm */
    fb._plannerState.lastTargetVelocity = 43.4;  /* 规划器仍按 868 rpm 走 */
    fb._plannerState.lastTargetFlow = 43.4;
    fb.STATE.plannedFlow = 43.4;
    fb.STATE.plannedVelocity = 43.4;

    assert(HYD_MotionControlFB_StartDirectCommand(
        &fb, HYD_DIRECT_CMD_MOVE_VELOCITY, &successor, NULL,
        HYD_BUFFER_MODE_ABORT, 1.001) == HYD_DIRECT_START_STARTED);

    fb.AXIS_REF.timestamp = 1.002;
    HYD_MotionControlFB_Cycle(&fb);

    /* Q = 10.85 + Kp·(140 − 124) = 26.85 L/min → 537 rpm（默认 Kp 1.0） */
    assert(fabs(fb.PUMP_SPEED - 537.0) < 1.0);
    assert((fb.STATE.limitFlags & HYD_LIMIT_FLAG_PRESSURE) != 0u);
}
```

在 `main()` 中 `test_reverse_position_handover_keeps_old_direction();` 的下一行加：

```c
    test_pressure_limited_handover_continues_from_executed_flow();
```

- [ ] **Step 3: 运行，确认失败**

Run: `cmake --build --preset unixgcc -j8 2>&1 | grep -m3 error`
Expected: `‘HYD_OutputLimiterInput’ has no member named ‘previousCommandFlow’`（限幅器测试编译失败）。

原型核对结果：如果只加 T3 而 `src/` 保持原样，`test_mode_switch_bumpless` 会在 `assert(fabs(fb.PUMP_SPEED - 537.0) < 1.0)` 处失败，原因是限幅状态被清零后流量恢复到 868 rpm。

- [ ] **Step 4: 实现——常量与接口**

`include/hyd_config.h`：把从 `/* 压力限制比例增益：超压比例 overRatio 时的输出缩减系数。` 到 `#define HYD_THRESH_PRESSURE_LIMIT_SCALE_FAST_RELEASE_RATE 10.0f` 的整段（5 个 `#define` 及其注释）替换为：

```c
/* 速度段限压 PI（spec 2026-11-26 §4–§5）：Q = min(Q_v, Kp·(Plim − P) + I)，
 * 压力环未被选中时 I 跟踪上一拍实际执行流量。参数 40/41 为 0 时用下列默认值。
 * 按油阀全关 Kq = 3.2 bar/s/rpm、G = 20 rpm/(L/min)、fc = 10 Hz 设计：
 * Kp = 2π·fc/(Kq·G) ≈ 1.0，Ki = Kp·2π·fc/4 ≈ 16（Ti ≈ 64 ms）。
 * 同一泵上任何轴的 Kq 都不高于油阀全关值，所以默认值对所有轴稳定。 */
#define HYD_THRESH_PRESSURE_LIMIT_PI_KP          1.0f   /* L/min per bar */
#define HYD_THRESH_PRESSURE_LIMIT_PI_KI          16.0f  /* L/min per (bar·s) */
```

`HYD_THRESH_PRESSURE_LIMIT_DEBOUNCE_S` 和 `HYD_THRESH_PRESSURE_LIMIT_FAULT_ESCALATION_S` 保留不动。被删除的 5 个常量已确认只有 `src/output_limiter.c` 使用。

`include/output_limiter.h` 有四处修改。

在 `HYD_OutputLimiterInput` 中 `effectiveMaxPressure` 一行之后加：

```c
    HYD_REAL pressureLimitKp;        /* 限压 PI 比例增益 [L/min/bar]，<= 0 用默认值 */
    HYD_REAL pressureLimitKi;        /* 限压 PI 积分增益 [L/min/(bar·s)]，<= 0 用默认值 */
    HYD_REAL previousCommandFlow;    /* 上一拍实际执行流量 [L/min]，PI 未选中时的积分跟踪源 */
    HYD_REAL deltaTime;              /* 本拍周期 [s]，非正或非有限时按 1 ms */
```

`HYD_OutputLimiterOutput` 中改为：

```c
    HYD_BOOL pressureLimitActive;    /* 限压 PI 本拍被选中（Q_p < Q_v） */
```

`HYD_OutputLimiterState` 中把

```c
    /* 压力限制运行状态。位置/速度段使用滞回保持限压状态，
     * 不在这里升级为库级压力故障。 */
    HYD_BOOL pressureLimitActive;
    HYD_REAL pressureScale;
    HYD_TIME pressureScaleLastTime;
```

改为

```c
    /* 限压 PI：上一拍是否被选中、积分 [L/min]。位置/速度段不升级为库级故障。 */
    HYD_BOOL pressureLimitActive;
    HYD_REAL pressureLimitIntegral;
```

`ExecuteWithProtection` 声明上方的注释改为 `/* 带状态的扩展版本（限压 PI 取小 + 软限位诊断） */`。

- [ ] **Step 5: 实现——`src/output_limiter.c`**

(a) `HYD_OutputLimiter_ResetState` 中把

```c
    state->pressureScale = 1.0;
    state->pressureScaleLastTime = 0.0;
```

改为

```c
    state->pressureLimitIntegral = 0.0f;
```

(b) 删除比例缩放实现，范围是从注释头 `/* ===...\n * 压力限制比例限速算法` 开始，一直到 `HYD_OutputLimiter_UpdatePressureLimitState` 函数结束。删除的内容包括：
- `HYD_OutputLimiter_CalcPressureScale`
- `HYD_OutputLimiter_ResolvePressureHysteresis`
- `HYD_OutputLimiter_UpdatePressureLimitState`

在原位置、`/* ===...\n * 软限位减速算法` 之前插入：

```c
/* ============================================================================
 * 速度段限压 PI（spec 2026-11-26 §4.1）
 * ----------------------------------------------------------------------------
 * 速度链路流量 Q_v 与限压 PI 输出 Q_p 取小：
 *   e   = Plim − P（P 不滤波）
 *   Q_p = Kp·e + I
 *   选中（Q_p < Q_v）：输出 Q_p，然后 I ← clamp(I + Ki·e·dt, 0, Qmax)
 *   未选中：输出 Q_v；下一拍 I 取上一拍实际执行流量（外部复位反馈）
 * 压力越接近 Plim，Q_p 比上一拍流量高出 Kp·e 越少，到 Plim 时接管；
 * 负载下降后 Q_p 连续升过 Q_v，自动回到速度控制。
 * actualPressure 与 effectiveMaxPressure 同为 bar。
 * 返回本拍流量上限；*selected 为 true 表示压力环被选中。
 * ============================================================================ */
static HYD_REAL HYD_OutputLimiter_ApplyPressureLimit(
    const HYD_OutputLimiterInput* input,
    HYD_OutputLimiterState* state,
    HYD_REAL velocityFlow,
    HYD_REAL maxFlow,
    HYD_BOOL* selected)
{
    HYD_REAL kp;
    HYD_REAL ki;
    HYD_REAL dt;
    HYD_REAL error;
    HYD_REAL integral;
    HYD_REAL pressureFlow;

    *selected = false;
    if (!(input->effectiveMaxPressure > 0.0f) ||
        !HYD_OutputLimiter_IsFinite(input->effectiveMaxPressure) ||
        !HYD_OutputLimiter_IsFinite(input->actualPressure)) {
        state->pressureLimitActive = false;
        return velocityFlow;
    }

    kp = (input->pressureLimitKp > 0.0f &&
          HYD_OutputLimiter_IsFinite(input->pressureLimitKp))
        ? input->pressureLimitKp : HYD_THRESH_PRESSURE_LIMIT_PI_KP;
    ki = (input->pressureLimitKi > 0.0f &&
          HYD_OutputLimiter_IsFinite(input->pressureLimitKi))
        ? input->pressureLimitKi : HYD_THRESH_PRESSURE_LIMIT_PI_KI;
    dt = input->deltaTime;
    if (!HYD_OutputLimiter_IsFinite(dt) || dt <= 0.0f) {
        dt = 0.001f;
    } else if (dt > 0.1f) {
        dt = 0.1f;
    }

    if (state->pressureLimitActive) {
        integral = state->pressureLimitIntegral;
    } else {
        integral = HYD_OutputLimiter_IsFinite(input->previousCommandFlow)
            ? input->previousCommandFlow : 0.0f;
        if (integral < 0.0f) {
            integral = 0.0f;
        } else if (integral > maxFlow) {
            integral = maxFlow;
        }
    }

    error = input->effectiveMaxPressure - input->actualPressure;
    pressureFlow = kp * error + integral;
    if (pressureFlow < velocityFlow) {
        *selected = true;
        integral += ki * error * dt;
        if (integral < 0.0f) {
            integral = 0.0f;
        } else if (integral > maxFlow) {
            integral = maxFlow;
        }
    }

    state->pressureLimitActive = *selected;
    state->pressureLimitIntegral = integral;
    return *selected ? pressureFlow : velocityFlow;
}
```

(c) `HYD_OutputLimiter_ExecuteWithProtection` 的局部变量声明中，把

```c
    HYD_REAL ratio;
    HYD_REAL pressureScale;
    HYD_REAL softLimitScale;
    HYD_REAL finalScale;
    HYD_DiagnosticCode softLimitDiag;
    HYD_BOOL pressureLimitActive;
```

改为

```c
    HYD_REAL ratio;
    HYD_REAL softLimitScale;
    HYD_DiagnosticCode softLimitDiag;
```

(d) 同一函数中，把从 `    /* --- 2. 计算压力限制 scale（独立于软限位） --- */` 到 `    /* --- 6. 软限位诊断；段压力限幅本身不是库级故障 --- */` 之前的部分（原第 2–5 步）整体替换为：

```c
    /* --- 2. 软限位减速 --- */
    softLimitScale = HYD_OutputLimiter_CalcSoftLimitScale(
        input->actualPosition, input->strokeMm,
        input->softLimitRetractMm, input->softLimitBandMm,
        input->direction);
    if (softLimitScale < 1.0f) {
        commandFlow *= softLimitScale;
        pumpSpeed *= softLimitScale;
        output->softLimitActive = true;
        output->derated = true;
    }

    /* --- 3. 限压 PI 与速度链路流量取小（spec 2026-11-26 §4.1） --- */
    {
        HYD_OutputLimiterState scratch;
        HYD_OutputLimiterState* piState = state;
        HYD_BOOL pressureSelected;
        HYD_REAL limitedFlow;

        if (piState == NULL) {
            HYD_OutputLimiter_ResetState(&scratch);
            piState = &scratch;
        }
        limitedFlow = HYD_OutputLimiter_ApplyPressureLimit(
            input, piState, commandFlow,
            input->pumpSpeedLimit / input->flowToPumpSpeedGain,
            &pressureSelected);
        if (pressureSelected) {
            commandFlow = (limitedFlow > minFlow) ? limitedFlow : minFlow;
            pumpSpeed = commandFlow * input->flowToPumpSpeedGain;
            output->pressureLimitActive = true;
            output->softLimitActive = false;
            output->derated = true;
        }
    }

```

然后把后面的两处注释编号改成连续的：
- `/* --- 6. 软限位诊断；段压力限幅本身不是库级故障 --- */` 改为 `/* --- 4. ... --- */`；
- `/* --- 7. pumpSpeedLimit 硬裁剪（最终兜底） --- */` 改为 `/* --- 5. ... --- */`。

函数体其余部分不动。

- [ ] **Step 6: 实现——`src/motion_control.c` 接线（CRLF）**

在 `HYD_MotionControlFB_RunRunningState` 中，把

```c
    limiterInput.actualPressure = fb->AXIS_REF.pressure;
```

改为

```c
    limiterInput.actualPressure = fb->AXIS_REF.pressure;
    limiterInput.pressureLimitKp = fb->_params.pressureLimitKp;
    limiterInput.pressureLimitKi = fb->_params.pressureLimitKi;
    limiterInput.previousCommandFlow = fb->_lastCommandedFlow;
    limiterInput.deltaTime = deltaTime;
```

`deltaTime` 是同一函数中已有的局部变量，由 `HYD_GetRunningDeltaTime(fb)` 计算。Stop 减速路径 `HYD_RunRunningStateStopping` 复用这份 `limiterInput`，不需要再改。

- [ ] **Step 7: 更新 IEC 级回归断言**

`tests/test_movevelocity_stop_reverse_restart.c` 的 `test_pressure_limit_derates_output_without_fault` 中，把从 `    fb->AXIS_REF.pressure = 110.0f;`（第一次出现）到 `"The lower axis pressure limit should override the command pressure limit");` 的部分替换为：

```c
    /* 限压 PI（spec 2026-11-26）：0.2 bar 超限时 Q = 上一拍流量 − Kp·0.2，
     * 默认 Kp 1.0 L/min/bar、增益 100 rpm/(L/min) → 泵转速比上一拍低 20 rpm。 */
    fb->AXIS_REF.pressure = 100.2f;
    __HydMotion_framework_Publish();
    hold_movevelocity_scan(&mv);
    limited_pump_speed = fb->PUMP_SPEED;

    ASSERT_TRUE(unrestricted_pump_speed > 90.0f,
                "MoveVelocity should reach steady unrestricted pump speed before pressure limiting");
    ASSERT_TRUE(fabsf(limited_pump_speed - (unrestricted_pump_speed - 20.0f)) < 0.5f,
                "A 0.2 bar excess should lower pump speed by Kp*0.2*gain = 20 rpm");
    ASSERT_TRUE(IEC_VAL(mv.BUSY) == true && IEC_VAL(mv.ACTIVE) == true,
                "Pressure limiting should keep MoveVelocity busy and active");

    /* 88 bar 低于指令限压 100、高于轴限压 80：只有轴限压能让压力环接管。 */
    fb->PRESSURE_LIMIT = 80.0f;
    fb->AXIS_REF.pressure = 88.0f;
    __HydMotion_framework_Publish();
    hold_movevelocity_scan(&mv);
    axis_limited_pump_speed = fb->PUMP_SPEED;

    ASSERT_TRUE(axis_limited_pump_speed < limited_pump_speed - 1.0f &&
                (fb->STATE.limitFlags & HYD_LIMIT_FLAG_PRESSURE) != 0u,
                "The lower axis pressure limit should override the command pressure limit");
```

同一函数开头的 `printf` 改为 `"--- Test: MoveVelocity pressure limit PI should limit without fault ---\n"`。

后半段不改，即持续 110 bar 运行 2000 拍、断言 Busy/Active、无 Error/CommandAborted、`HYD_LIMIT_FLAG_PRESSURE` 置位、无诊断码。这部分加上 `test_movecontinuousabsolute_integration.c` 中的 `test_pressure_limit_stays_running_without_error`（不改），覆盖 spec T4。

- [ ] **Step 8: 运行，确认通过**

Run:
```bash
cmake --build --preset unixgcc -j8 && ctest --test-dir out/build/unixgcc -R '^(test_output_limiter|test_mode_switch_bumpless|test_movevelocity_stop_reverse_restart|test_movecontinuousabsolute_integration)$' --output-on-failure
```
Expected: 4/4 passed；`test_output_limiter` 打印 `test_pressure_limit_closed_loop_settles_at_limit_and_releases PASSED (peak 146.58 bar)`。

Run: `ctest --test-dir out/build/unixgcc 2>&1 | tail -4`
Expected: `2 tests failed out of 64`，只有 `test_parameter_iec` 和 `test_motion_interface_unit`。

Run: `gcc -std=c99 -Wall -Wextra -Wdouble-promotion -Iinclude -c src/output_limiter.c -o /tmp/ol.o && echo clean`
Expected: 没有任何 warning，输出 `clean`。

Run: `grep -rn "HYD_THRESH_PRESSURE_LIMIT_\(KP\|MIN_SCALE\|HYSTERESIS_RATIO\|SCALE_RELEASE_RATE\|SCALE_FAST_RELEASE_RATE\)\b\|pressureScale" src include tests`
Expected: 无输出。

CRLF 检查同 Task 1 Step 4。

- [ ] **Step 9: 提交**

```bash
git add include/hyd_config.h include/output_limiter.h src/output_limiter.c src/motion_control.c tests/test_output_limiter.c tests/test_movevelocity_stop_reverse_restart.c tests/test_mode_switch_bumpless.c
git commit -m "速度段限压改为压力环取小：限压 PI 替换比例缩放（spec 2026-11-26 §4.1–§4.2）" -m "Q = min(Q_v, Kp·(Plim − P) + I)，未选中时积分跟踪上一拍实际执行流量；删除比例缩放、回差、双速恢复。换段时从实际流量续接（复现 vofa 20261125 8879 ms 217→868 rpm 跳变的测试）。" -m "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 3: 转保压时保压 PI 初值取实际执行流量（T5）

**Files:**
- Modify: `src/motion_control.c`（`HYD_PrimeSegmentControllers` 的"V->P and Position->P carryover"分支，约 1820–1830 行）——CRLF
- Modify: `tests/test_mode_switch_bumpless.c`

**Interfaces:**
- Consumes: 无新接口。
- Produces: 行为变化。速度/位置段转压力段续接时，`carriedFlow` 优先取 `fb->_lastCommandedFlow`，为 0 时才退回 `fb->_plannerState.lastTargetFlow`。

- [ ] **Step 1: 写失败测试**

在 `tests/test_mode_switch_bumpless.c` 中 Task 2 加的 `test_pressure_limited_handover_continues_from_executed_flow` 之后插入：

```c
/* spec 2026-11-26 §4.5 / T5：限压中转保压，保压 PI 初值取实际执行流量（7.7 L/min），
 * 不取限压前的规划流量（52 L/min）。 */
static void test_pressure_handover_seeds_from_executed_flow(void) {
    HYD_MotionControlFB fb;
    HYD_MotionSegment source = make_speed_segment();
    HYD_MotionSegment successor = make_pressure_segment();

    source.maxVelocity = 52.0;
    source.maxPressure = 140.0;
    seed_active_segment(&fb, source, HYD_DIRECT_CMD_MOVE_VELOCITY);
    fb.FLOW_TO_PUMP_SPEED_GAIN = 20.0;
    fb.PUMP_SPEED_LIMIT = 1700.0;
    fb.AXIS_REF.pressure = 140.0;
    fb.AXIS_REF.flow = 0.0;
    fb.PUMP_SPEED = 154.0;
    fb.STATE.commandedPumpSpeed = 154.0;
    fb._lastCommandedFlow = 7.7;
    fb._plannerState.lastTargetVelocity = 52.0;
    fb._plannerState.lastTargetFlow = 52.0;
    fb.STATE.plannedFlow = 52.0;
    successor.targetPressure = 100.0;
    successor.maxFlow = 85.0;

    assert(HYD_MotionControlFB_StartDirectCommand(
        &fb, HYD_DIRECT_CMD_PRESSURE_HANDLE, &successor, NULL,
        HYD_BUFFER_MODE_ABORT, 1.001) == HYD_DIRECT_START_STARTED);
    assert(fabs(fb._pressureController.previousOutput - 7.7) < 1e-3);
}
```

在 `main()` 中 `test_pressure_limited_handover_continues_from_executed_flow();` 的下一行加：

```c
    test_pressure_handover_seeds_from_executed_flow();
```

- [ ] **Step 2: 运行，确认失败**

Run: `cmake --build --preset unixgcc -j8 && ./out/build/unixgcc/test_mode_switch_bumpless`
Expected: `Assertion 'fabs(fb._pressureController.previousOutput - 7.7) < 1e-3' failed`，此时初值为 52。

- [ ] **Step 3: 实现（CRLF）**

在 `HYD_PrimeSegmentControllers` 中，把

```c
                /* Preserve flow from motion planning state */
                carriedFlow = fb->_plannerState.lastTargetFlow;
                if (carriedFlow <= 0.0) {
                    /* Fallback: use last commanded pump flow */
                    carriedFlow = fb->_lastCommandedFlow;
                }
```

改为

```c
                /* 限压生效时规划流量高于实际执行流量（spec 2026-11-26 §4.5）：
                 * 优先取上一拍实际执行流量，未记录时退回规划流量。 */
                carriedFlow = fb->_lastCommandedFlow;
                if (carriedFlow <= 0.0) {
                    carriedFlow = fb->_plannerState.lastTargetFlow;
                }
```

这段只在 `segment->mode == HYD_MODE_PRESSURE_CLOSED_LOOP` 且上一段是 SPEED_RAMP 或 POSITION 时执行。不限压时两个值相等，行为不变：已有的 `test_position_pressure_handover`、`test_speed_pressure_handover`、`test_pressure_handover_does_not_add_second_pump_slew` 会验证这一点。

- [ ] **Step 4: 运行，确认通过**

Run: `cmake --build --preset unixgcc -j8 && ctest --test-dir out/build/unixgcc --output-on-failure 2>&1 | tail -4`
Expected: `2 tests failed out of 64`，只有那 2 个已知失败；`test_mode_switch_bumpless`、`test_pressure_2dof_closed_loop`、`test_pressure_controller` 通过。

CRLF 检查同 Task 1 Step 4。

- [ ] **Step 5: 提交**

```bash
git add src/motion_control.c tests/test_mode_switch_bumpless.c
git commit -m "转保压初值改用实际执行流量（spec 2026-11-26 §4.5）" -m "限压生效时规划流量（约 1040 rpm）远高于实际执行流量（约 150 rpm），V→P 续接改为优先取 _lastCommandedFlow。" -m "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 4: 辨识脚本——射出轴列名、螺杆静止过滤、参数 40/41 建议值

**Files:**
- Modify: `scripts/identify_pressure_plant.py`（LF，只用标准库）

**Interfaces:**
- Produces:
  - CLI：`identify_pressure_plant.py [--pressure-col C] [--angle-col C] [--speed-col C] [--speed-scale K] [--pos-col C] [--max-pos-rate R] [--gain G] FILE...`；
  - 默认值 `ActPressure` / `rActPumpPosition` / 无 / 1.0 / 无 / 0.5 / 20.0；
  - 新增一行输出：`parameter 40 = … L/min/bar, parameter 41 = … L/min/(bar*s) (…)`。

- [ ] **Step 1: 记录现有输出作为基准**

```bash
for f in docs/data_deg/hor1121/vofa_waveform_20261121_close100.csv docs/data_deg/hor1121/vofa_waveform_20261121_close140.csv; do python3 scripts/identify_pressure_plant.py "$f"; done > /tmp/id_before.txt
cat /tmp/id_before.txt
```

Expected（原型实测）：

```
== docs/data_deg/hor1121/vofa_waveform_20261121_close100.csv
   Kq   = 2.859 bar/s/rpm   ksys = 5.344 bar/rpm   (lag 4 ms, R2 0.915, 11990 samples)
   Kq by pressure band: 80-100 bar 2.93, 100-120 bar 3.56
   parameter 34 = 5.34, parameter 37 = 2.57 (0.9 x lowest Kq)
== docs/data_deg/hor1121/vofa_waveform_20261121_close140.csv
   Kq   = 3.010 bar/s/rpm   ksys = 5.034 bar/rpm   (lag 4 ms, R2 0.877, 14635 samples)
   Kq by pressure band: 120-140 bar 3.74, 140-160 bar 4.52
   parameter 34 = 5.03, parameter 37 = 2.71 (0.9 x lowest Kq)
```

- [ ] **Step 2: 实现**

(a) 文档字符串末尾，把

```python
Standard library only. Usage:
    python3 scripts/identify_pressure_plant.py FILE.csv [FILE.csv ...]
"""
import csv
import math
import sys
```

改为

```python
Pressure-limit PI (spec 2026-11-26 §10, parameters 40/41) uses the HIGHEST
band Kq: Kp = 2*pi*fc / (Kq_max * G), Ki = Kp * 2*pi*fc / 4, fc = 10 Hz.

Records without a pump angle (e.g. injection-axis records) pass the column
names; --speed-col reads the servo speed directly (times --speed-scale, the
VOFA records log 10 rpm per count), --pos-col with --max-pos-rate keeps only
samples where the screw is stationary. Ripple orders need the pump angle.

Standard library only. Usage:
    python3 scripts/identify_pressure_plant.py FILE.csv [FILE.csv ...]
    python3 scripts/identify_pressure_plant.py --pressure-col fbPressureSensor.rActualValue \
        --speed-col fbServoControl.ActSpeedRPM --speed-scale 10 \
        --pos-col G_Inject.udiInjElecRulerVal --max-pos-rate 0.5 FILE.csv
"""
import argparse
import csv
import math
import sys
```

(b) 把 `MAX_ORDER = 30` 以及后面整个 `def load(path):` 函数替换为：

```python
MAX_ORDER = 30
LIMIT_FC = 10.0        # Hz, pressure-limit PI crossover (spec 2026-11-26 §5.1)


def load(path, args):
    with open(path, encoding="utf-8-sig", newline="") as handle:
        rows = list(csv.DictReader(handle))
    pressure = [float(r[args.pressure_col]) for r in rows]
    angle = None
    speed = None
    if args.speed_col:
        speed = [float(r[args.speed_col]) * args.speed_scale for r in rows]
    else:
        angle = [float(r[args.angle_col]) for r in rows]
    position = [float(r[args.pos_col]) for r in rows] if args.pos_col else None
    return pressure, angle, speed, position


def valid_samples(pressure, position, max_rate):
    if position is None:
        return [p >= MIN_PRESSURE for p in pressure]
    rate = gradient(smooth(position))
    return [p >= MIN_PRESSURE and abs(r) <= max_rate for p, r in zip(pressure, rate)]
```

（`gradient` 和 `smooth` 定义在后面，运行时才调用，不受定义顺序影响。）

(c) `fit_plant` 和 `kq_bands` 各增加最后一个参数 `valid`，循环里的过滤条件由

```python
            if pressure[i] < MIN_PRESSURE:
                continue
```

改为

```python
            if not valid[i]:
                continue
```

函数签名改为 `def fit_plant(pressure, speed, rate, valid):` 和 `def kq_bands(pressure, speed, rate, lag, ksys, valid):`。

(d) 把 `main` 的开头

```python
def main(paths):
    for path in paths:
        raw_pressure, raw_angle = load(path)
        pressure = smooth(raw_pressure)
        speed = [v * 60.0 / 360.0 for v in gradient(smooth(unwrap_deg(raw_angle)))]
        rate = gradient(pressure)
        print(f"== {path}")
        fit = fit_plant(pressure, speed, rate)
```

改为

```python
def main(args):
    for path in args.files:
        raw_pressure, raw_angle, raw_speed, position = load(path, args)
        pressure = smooth(raw_pressure)
        if raw_speed is not None:
            speed = smooth(raw_speed)
        else:
            speed = [v * 60.0 / 360.0 for v in gradient(smooth(unwrap_deg(raw_angle)))]
        rate = gradient(pressure)
        valid = valid_samples(pressure, position, args.max_pos_rate)
        print(f"== {path}")
        fit = fit_plant(pressure, speed, rate, valid)
```

(e) 把 `main` 中从 `        bands = kq_bands(pressure, speed, rate, fit["lag"], ksys)` 到 `        window = ripple_orders(raw_pressure, raw_angle, longest_hold(raw_pressure))` 的部分替换为：

```python
        bands = kq_bands(pressure, speed, rate, fit["lag"], ksys, valid)
        if bands:
            print("   Kq by pressure band: " +
                  ", ".join(f"{p:.0f}-{p + BAND:.0f} bar {k:.2f}" for p, k, _ in bands))
            lowest = min(k for _, k, _ in bands)
            highest = max(k for _, k, _ in bands)
        else:
            lowest = highest = fit["kq"]
        print(f"   parameter 34 = {ksys:.2f}, parameter 37 = {0.9 * min(lowest, fit['kq']):.2f}"
              f" (0.9 x lowest Kq)")
        highest = max(highest, fit["kq"])
        if fit["r2"] < MIN_R2 or not bands or min(k for _, k, _ in bands) <= 0.0:
            print("   parameter 40/41: not identified (poor fit), keep the built-in defaults")
        else:
            omega = 2.0 * math.pi * LIMIT_FC
            kp = omega / (highest * args.gain)
            print(f"   parameter 40 = {kp:.2f} L/min/bar, parameter 41 = {kp * omega / 4.0:.1f} L/min/(bar*s)"
                  f" (pressure-limit PI, fc {LIMIT_FC:.0f} Hz, highest Kq {highest:.2f}, G {args.gain:g})")
        if raw_angle is None:
            continue
        window = ripple_orders(raw_pressure, raw_angle, longest_hold(raw_pressure))
```

**拟合质量门槛必须保留。**原型中没有阶跃的 1125 记录拟合 R² 只有 0.005；如果去掉门槛，脚本会建议 Kp = 574，写进去会让回路失稳。

(f) 把文件末尾的

```python
if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(2)
    main(sys.argv[1:])
```

改为

```python
def parse_args(argv):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("files", nargs="+")
    parser.add_argument("--pressure-col", default="ActPressure")
    parser.add_argument("--angle-col", default="rActPumpPosition")
    parser.add_argument("--speed-col", default=None,
                        help="servo speed column; replaces the pump-angle derivative")
    parser.add_argument("--speed-scale", type=float, default=1.0,
                        help="rpm per logged count of --speed-col (VOFA records: 10)")
    parser.add_argument("--pos-col", default=None,
                        help="actuator position column [mm] for the stationary filter")
    parser.add_argument("--max-pos-rate", type=float, default=0.5,
                        help="mm/s; samples moving faster are skipped (needs --pos-col)")
    parser.add_argument("--gain", type=float, default=20.0,
                        help="flowToPumpSpeedGain G [rpm per L/min] for parameters 40/41")
    return parser.parse_args(argv)


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(2)
    main(parse_args(sys.argv[1:]))
```

- [ ] **Step 3: 验证——旧记录输出不变，并多出参数 40/41 一行**

```bash
for f in docs/data_deg/hor1121/vofa_waveform_20261121_close100.csv docs/data_deg/hor1121/vofa_waveform_20261121_close140.csv; do python3 scripts/identify_pressure_plant.py "$f"; done > /tmp/id_after.txt
diff <(grep -v "parameter 40" /tmp/id_after.txt) /tmp/id_before.txt && echo "legacy output unchanged"
grep "parameter 40" /tmp/id_after.txt
```

Expected：

```
legacy output unchanged
   parameter 40 = 0.88 L/min/bar, parameter 41 = 13.9 L/min/(bar*s) (pressure-limit PI, fc 10 Hz, highest Kq 3.56, G 20)
   parameter 40 = 0.69 L/min/bar, parameter 41 = 10.9 L/min/(bar*s) (pressure-limit PI, fc 10 Hz, highest Kq 4.52, G 20)
```

- [ ] **Step 4: 验证——射出记录的列名选项能跑通，且拟合差时拒绝给出建议值**

`docs/data_deg/vofa_waveform_20261125_115208_697.csv` 在工作区里未跟踪，不要提交它。

```bash
python3 scripts/identify_pressure_plant.py --pressure-col fbPressureSensor.rActualValue \
    --speed-col fbServoControl.ActSpeedRPM --speed-scale 10 \
    --pos-col G_Inject.udiInjElecRulerVal --max-pos-rate 0.5 \
    docs/data_deg/vofa_waveform_20261125_115208_697.csv
```

Expected：输出 R² 告警，最后一行是 `   parameter 40/41: not identified (poor fit), keep the built-in defaults`。这份记录没有螺杆静止的压力阶跃，现场需要按 spec §10.2 重新录一份。

- [ ] **Step 5: 提交**

```bash
git add scripts/identify_pressure_plant.py
git commit -m "辨识脚本：射出轴列名选项、螺杆静止过滤、参数 40/41 建议值（spec 2026-11-26 §10）" -m "--speed-col/--speed-scale 直接读伺服转速（VOFA 记录 10 rpm/计数），--pos-col/--max-pos-rate 只取螺杆静止样本；按最高段 Kq、fc 10 Hz 给出限压 PI 增益，拟合 R² 不足时不给建议值。旧记录输出逐行不变。" -m "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 5: PLC 侧——写入参数 40/41，输出 `bPressureLimited`

**Files:**
- Modify: `iec/ST_HydAxisRef.st`（LF）
- Modify: `iec/FB_HydAxis.st`（CRLF）

**Interfaces:**
- Consumes: 参数 40/41（Task 1）；`HYD_ReadStatus.PRESSURELIMITACTIVE`（已有）。
- Produces:
  - `ST_HydAxisRef.rPressureLimitKp`、`rPressureLimitKi`（`REAL := 0`）；
  - `FB_HydAxis` 新输出 `bPressureLimited : BOOL`。

- [ ] **Step 1: 修改 `iec/ST_HydAxisRef.st`**

把

```
  rPressureKi            : REAL := 0;
END_STRUCT;
```

改为

```
  rPressureKi            : REAL := 0;
  rPressureLimitKp       : REAL := 0; // 速度段限压 PI 比例增益 L/min/bar（参数 40），0 = 库默认 1.0
  rPressureLimitKi       : REAL := 0; // 速度段限压 PI 积分增益 L/min/(bar·s)（参数 41），0 = 库默认 16
END_STRUCT;
```

- [ ] **Step 2: 修改 `iec/FB_HydAxis.st`（保持 CRLF）**

(a) `VAR_OUTPUT` 中，把

```
    bInPressure : BOOL;
    bAlarm : BOOL;
```

改为

```
    bInPressure : BOOL;
    bPressureLimited : BOOL;   // 速度类段限压 PI 正在接管（实际压力达到 uiPres），工艺层据此判堵转
    bAlarm : BOOL;
```

(b) 第 0 步初始化中，在

```
              IF stHydAxisRef.rPressureKi > 0.0 THEN
                SetParameter(14, stHydAxisRef.rPressureKi);
              END_IF;
```

之后插入

```
              // 速度段限压 PI（spec 2026-11-26 §5）：参数 40 Kp L/min/bar、41 Ki L/min/(bar·s)；0 = 不写，保持库默认
              IF stHydAxisRef.rPressureLimitKp > 0.0 THEN
                SetParameter(40, stHydAxisRef.rPressureLimitKp);
              END_IF;
              IF stHydAxisRef.rPressureLimitKi > 0.0 THEN
                SetParameter(41, stHydAxisRef.rPressureLimitKi);
              END_IF;
```

(c) 在已有的 `fbHYD_ReadStatus(...)` 调用行之后插入

```
  bPressureLimited := fbHYD_ReadStatus.VALID AND fbHYD_ReadStatus.PRESSURELIMITACTIVE AND (iStep = 100 OR iStep = 105);
```

- [ ] **Step 3: 验证**

```bash
grep -n "rPressureLimitK" iec/ST_HydAxisRef.st iec/FB_HydAxis.st
grep -n "bPressureLimited" iec/FB_HydAxis.st
f=iec/FB_HydAxis.st; echo "lines $(wc -l < $f) crlf $(grep -c $'\r$' $f)"
f=iec/ST_HydAxisRef.st; echo "crlf $(grep -c $'\r$' $f)"
```

Expected：
- `ST_HydAxisRef.st` 有 2 处定义；`FB_HydAxis.st` 有 4 处引用（2 个 IF、2 个 SetParameter）；
- `bPressureLimited` 出现 2 次（声明和赋值）；
- `FB_HydAxis.st` 的 lines 与 crlf 相等（原型为 644/644）；`ST_HydAxisRef.st` 的 crlf 为 0。

ST 文件不在 ctest 范围内，编译验证放到 PLC 工程（Beremiz/matiec）下载前进行，见下文"上机交接"。

- [ ] **Step 4: 提交**

```bash
git add iec/ST_HydAxisRef.st iec/FB_HydAxis.st
git commit -m "FB_HydAxis：写入限压 PI 参数 40/41，输出 bPressureLimited（spec 2026-11-26 §9）" -m "rPressureLimitKp/Ki 大于 0 才写入，否则保持库默认值 1.0/16；bPressureLimited 取 HYD_ReadStatus.PRESSURELIMITACTIVE，仅在运行步有效。" -m "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

## 最终验证（全部任务完成后）

```bash
cmake --build --preset unixgcc -j8
ctest --test-dir out/build/unixgcc 2>&1 | tail -4      # 期望：2 failed out of 64，仅 test_parameter_iec、test_motion_interface_unit
./out/build/unixgcc/test_mode_switch_bumpless | head -1   # 期望：motion_control_fb=3440 bytes
for f in src/motion_control.c iec/FB_HydAxis.st; do echo "$f $(wc -l < $f) $(grep -c $'\r$' $f)"; done   # 两列相等
git log --oneline -5
```

## 上机交接（不在 ctest 范围内，由用户在现场执行，对应 spec §10）

1. PLC 工程重新编译下载，确认 `FB_HydAxis` 编译通过。参数 40/41 先不写，使用库默认值 1.0/16。
2. 确认工艺层"最大射出时间 → 自动转保压"已经生效（用户已确认具备）。
3. 按 spec §10.2 录保压阶跃记录（闭环 60→100→140→60 bar，或开环恒速 150/250/350 rpm），用 Task 4 的命令行（带 `--speed-scale 10 --pos-col ... --max-pos-rate 0.5`）得到参数 40/41 建议值，写入 `rPressureLimitKp/Ki`。
4. 按 spec §10.4 做到位前堵转试验，限压依次设 80 → 120 → 140 bar，每档 3 模。判据：
   - 稳态 |P − Plim| ≤ 2 bar；
   - 峰值 ≤ 1.1 Plim；
   - 无振荡；
   - `bPressureLimited` = TRUE。
5. 另见 spec §9"上机注意"四条：段间上调 Plim、库诊断保持关闭、`bPressureLimited` 去抖、完整下载。
