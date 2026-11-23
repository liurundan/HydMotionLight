# 压力闭环二自由度结构与泵轴角同步纹波补偿 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 压力段在参数 37（Kq）> 0、策略不是 RBF 时，用“带速率限制和 √ 制动的参考轨迹 + 模型前馈 + 延迟参考 PI + 泵轴角 N/2N 阶纹波补偿”替代 20 ms 时间斜坡 + PI，使超调 < 1.5 bar、保压纹波峰峰值 < 2 bar；Kq = 0 时与现版本逐拍一致。

**Architecture:** `ramp_controller` 新增 profiled 执行函数，输出参考 r 和斜率 ṙ。`pressure_controller` 的非 RBF 分支新增“模型路径”：
- 用 Qff = ṙ/Kq_f + r/R_f 替换原前馈；
- 反馈误差与延迟 12 拍的参考比较；
- 卸压状态机加 ṙ 条件；
- 调用新模块 `pressure_ripple_comp`（泵轴角同步 LMS），并把补偿流量加进未饱和输出。

`motion_control` 只负责接线：参数 37/38/39 → 轨迹速率上限 → 控制器输入。纹波系数存放在 FB 里，每段开始时不清零。

**Tech Stack:** C99（HydroMotionLib）、CMake/CTest（`unixgcc` preset，在 WSL Ubuntu-22.04 中构建）、`assert` 测试、Python 3 标准库脚本、IEC 61131-3 ST。

**Spec:** `docs/superpowers/specs/2026-11-23-pressure-loop-2dof-ripple-comp-design.md`（提交 0a8afa4，用户 2026-11-23 审阅通过）。
- 本计划实现 spec §4.2–§4.7、§6 T1–T8、§7 的参数写入、§8.1 的辨识脚本。
- 与 spec 不同的地方见下文“与 spec 的差异”，每条都给出了验证依据。
- 计划里的全部代码已在仓库的临时副本中完整实现并跑通：64/64 通过；Kq = 0 路径与 0a8afa4 逐位一致。

## Global Constraints

- 纯 C99、`HYD_` 前缀、静态内存、无 malloc（CLAUDE.md 约束 5）。
- 调参型浮点常数放进 `include/hyd_config.h` §14B，不写在 `.c` 里（CLAUDE.md 约束 7）。本计划新增：

  | 常数 | 值 |
  |---|---|
  | `HYD_THRESH_PRESSURE_PROFILE_RATE_MARGIN` | 0.6 |
  | `HYD_THRESH_PRESSURE_PROFILE_MIN_FLOW_RATIO` | 0.1 |
  | `HYD_THRESH_PRESSURE_PROFILE_BRAKE_TIME_S` | 0.12 |
  | `HYD_MAX_PRESSURE_BRAKE_TIME_MS` | 2000 |
  | `HYD_PRESSURE_REF_DELAY_SAMPLES` | 12 |
  | `HYD_THRESH_PRESSURE_RIPPLE_MU` | 4.0 |
  | `HYD_THRESH_PRESSURE_RIPPLE_PATH_DELAY_S` | 0.006 |
  | `HYD_THRESH_PRESSURE_RIPPLE_MIN_RPM` | 3.0 |
  | `HYD_THRESH_PRESSURE_RIPPLE_MAX_ERROR` | 3.0 |
  | `HYD_THRESH_PRESSURE_RIPPLE_HOLD_S` | 0.05 |
  | `HYD_THRESH_PRESSURE_RIPPLE_FADE_IN_S` / `_FADE_OUT_S` | 0.05 / 0.02 |
  | `HYD_THRESH_PRESSURE_RIPPLE_MAX_FLOW` | 1.5 |
  | `HYD_THRESH_PRESSURE_RIPPLE_MAX_FREQ_HZ` | 100 |
  | `HYD_MAX_PUMP_RIPPLE_ORDER` | 64 |

- 参数编号：现有编号不变，新参数追加在 36 之后：

  | 编号 | 名称 | 单位 | 写入校验 |
  |---|---|---|---|
  | 37 | `HYD_PARAM_KQ_BAR_PER_S_RPM` | bar/s/rpm | 有限且 ≥ 0 |
  | 38 | `HYD_PARAM_PRESSURE_BRAKE_TIME` | ms | 0–2000 |
  | 39 | `HYD_PARAM_PUMP_RIPPLE_ORDER` | — | 整数 0–64 |

  三个参数默认都是 0。
- 模型路径只在参数 37 > 0、策略不是 RBF_PID/RBF_PI、`flowToPumpSpeedGain` > 0 时启用。参数 37 = 0 时：
  - 轨迹、前馈、卸压、调试通道全部走旧代码；
  - 由 Task 1 的逐位基准（0a8afa4 上采集）守护。
- 不改 `src/rbf_pid.c`、`include/rbf_pid.h`，也不改 `pressure_controller.c` 的 RBF 分支。
- 换行符保持原样，不要整文件转换：
  - `src/motion_control.c`、`iec/FB_HydAxis.st` 是 CRLF；
  - 本计划涉及的其他文件都是 LF。
- FB 尺寸：`sizeof(HYD_MotionControlFB)` 由 3328 变为 3432 字节（+104）。
- 基线 62 个 CTest 通过，完成后 64 个。
- 每次 `git add` 只加本任务列出的文件。
  - 禁止 `git add -A`、`git add .`、`git commit -a`。
  - 不提交用户已有的 `scripts/coverage.sh`、`scripts/deploy_embedded_prod.sh` 修改，也不提交未跟踪的 `docs/data_deg/hor1121/`。
- 在 WSL 中提交，使用仓库已配置的身份，分支 `feature/rbf-pid-industrialization`。提交信息末尾加：`Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`。

## 构建与运行

仓库在 WSL 里。所有命令都在 WSL 中、仓库根目录执行：

```bash
wsl.exe -d Ubuntu-22.04 bash -lc 'cd ~/project/hdy-motion-light && <命令>'
```

- **Shell 注意**：从 Git Bash 经 `wsl.exe` 传入的命令里，`$VAR`、heredoc 和带引号的内联 C 代码经常被吞掉或改坏。超过一行的内容先用编辑工具写成文件，再执行。
- **构建**：`cmake --build --preset unixgcc 2>&1 | grep -E "error|warning: "`。现有告警只出现在 `rbf_pid.c`、`state_reporter.c`、`test_diagnostic_criteria.c`，不得新增其他告警。
- **新增 `src/*.c` 或在 CMakeLists 注册新测试后**，先 `cmake --preset unixgcc`（CLAUDE.md 约束 6）。
- **全量测试**：`ctest --test-dir out/build/unixgcc --output-on-failure 2>&1 | tail -5`。

## 与 spec 的差异（均已验证）

“oracle”指与本计划 C 代码逐行对应的 Python 仿真；“T6”指本计划的 C 闭环回归。

| # | spec 原文 | 本计划 | 依据 |
|---|---|---|---|
| D1 | 反馈参考延迟 8 拍 | 12 拍（`HYD_PRESSURE_REF_DELAY_SAMPLES`） | oracle，开纹波补偿、Kq 配置准确：<br>• 伺服延迟 7 ms 时，最大超调 8 拍 1.81 bar、12 拍 0.90 bar<br>• 真实 Kq 为配置值 1.1–1.25 倍时，8 拍 1.60–1.67 bar、12 拍 ≤ 1.05 bar<br>• 代价：t98 慢约 10 ms |
| D2 | 反向改目标时速度连续转向 | 参考先停（ṙ 置 0），再反向加速 | oracle：140→60 中途反向时，spec 原式让参考越过旧目标冲到 191 bar；先停只到 107 bar |
| D3 | 纹波系数放在 `HYD_PressureControllerState` 子结构里，`InitState` 先保存后恢复 | 放在 FB 的 `_pressureRippleComp`，通过 `HYD_PressureControllerInput.rippleComp` 指针传入 | 效果相同，`InitState` 不用改。禁用和故障时由 `safety_state_manager` 清零；`Init` 与 `SoftReset` 的整体 memset 也会清零 |
| D4 | 静止 50 ms 后学习和淡入同时开始 | 淡入完成（fade = 1）后才学习 | 淡入期间学习会过学。再次进入同一压力时，0.25–0.5 s 的 12 阶残余由 0.24 bar 降到 0.15 bar（T5 守护） |
| D5 | 卸压判据用延迟参考误差 e | 卸压判据仍用 r(k) − P，只在原函数上加 ṙ 条件 | `HYD_UpdateReliefState` 改动最小；oracle 与 T6 均按此验证 |
| D6 | `rPumpMaxRPM` 默认值 1800 → 1700 | 不改 | `FB_HydAxis.st` 调用处已显式传入 1700，改默认值没有运行效果（见下文“需要告知用户”） |
| D7 | T6 不开纹波补偿时保压峰峰 ≤ 2.6 bar | ≤ 3.0 bar | 参考延迟 12 拍时，T6 实测 140 bar 保压为 2.68 bar。只做 B1 本来就达不到 < 2 bar，由 B2 承担（B2 门限 ≤ 1.0 bar 不变） |
| D8 | `HYD_PressureControllerInput` 增加 `pressureBrakeTimeMs` | 不加；改为增加 `rippleComp` 指针 | 制动时间只在 `motion_control` 生成轨迹时使用 |
| D9 | 参数 37 初值 2.9 | 建议 2.5 | 辨识脚本对 hor1121 五个闭环记录的整体拟合 Kq 为 2.77–3.01 bar/s/rpm，最低 2.77（50 bar），× 0.9 ≈ 2.5。T6 仍按 spec 用“配置 2.9 / 对象 3.2” |
| D10 | 只做 Kq 比值 1.0–2.0 的鲁棒性验证 | 补充：Kq 比值 2.0 时超调 1.5–2.4 bar | 配置值不能低于真实值的一半；§8.1 的“×0.9”远在此范围内 |
| D11 | T8 经 PressureHandle 做 FB 级接线测试 | T6/T8 直接驱动 `HYD_MotionControlFB`（`LoadDirectSegment` + `ApplyLiveUpdate`） | 新参数从 `fb->_params` 读取，`motion_interface.c` 的 `buildPressureSegment` 不经手；IEC 适配层无改动，现有接口测试已覆盖 |

## 文件结构

| 文件 | 改动 | 职责 |
|---|---|---|
| `include/hyd_config.h` | 改 | §14B 新常数 |
| `include/ramp_controller.h`、`src/ramp_controller.c` | 改 | 新增 `HYD_RampProfileLimits`、`HYD_RampController_ExecuteProfiledWithDt`、输出 `rampedPressureRate`；旧函数报告斜率 0 |
| `include/pressure_ripple_comp.h`、`src/pressure_ripple_comp.c` | 新 | N/2N 阶泵轴角同步 LMS，含门控、淡入淡出、限幅、触限冻结 |
| `include/pressure_controller.h`、`src/pressure_controller.c` | 改 | 模型路径：速率上限辅助函数、模型前馈、延迟参考、卸压 ṙ 条件、纹波补偿接入、调试通道 0–7 |
| `include/common_types.h` | 改 | 参数枚举 37/38/39，`HYD_MotionFBParams` 加 3 个字段 |
| `include/motion_control.h` | 改 | FB 增加 `_pressureRippleComp` |
| `src/motion_control.c`（CRLF） | 改 | 参数默认值、读写；压力分支接线 |
| `src/safety_state_manager.c` | 改 | 禁用和故障时清零纹波系数 |
| `iec/FB_HydAxis.st`（CRLF） | 改 | 第 0 步写参数 34/37/38/39/11/14 |
| `scripts/identify_pressure_plant.py` | 新 | 从 VOFA CSV 辨识 ksys、Kq、纹波阶次 |
| `tests/test_pressure_controller.c` | 改 | T4 逐位基准；T2/T3 |
| `tests/test_ramp_controller.c` | 改 | T1 |
| `tests/test_pressure_ripple_comp.c` | 新 | T5 |
| `tests/test_parameter_access.c` | 改 | T7 |
| `tests/test_pressure_2dof_closed_loop.c` | 新 | T6/T8：真实 FB 加 hor1121 对象的闭环回归 |
| `tests/test_negative_flow.c`、`tests/test_sprint_b_integration.c` | 改 | 未初始化的 `HYD_PressureControllerInput` 改为 `= {0}`，避免新字段里的随机值误开模型路径 |
| `tests/test_mode_switch_bumpless.c`、`tests/test_toggle_mechanism_pool.c` | 改 | FB 尺寸预算 |
| `tests/test_hydaxis_p0_contract.py` | 改 | PLC 参数写入契约 |
| `CMakeLists.txt` | 改 | 注册 2 个新测试 |

---
### Task 1: 旧路径逐位基准（T4）

**Files:**
- Modify: `tests/test_pressure_controller.c`

**Interfaces:**
- Consumes：现有 `HYD_PressureController_*`、`HYD_MotionControlFB_*` API。
- Produces：`test_legacy_path_is_bit_identical()`。后续任务每次跑全量测试都会执行它，用来保证参数 37 = 0 的路径不变。

这是特征化测试，在未修改的代码上就应通过，不是“先失败”的测试。基准值在 0a8afa4 上用同一段代码采集（十六进制浮点，逐位比较）。

- [ ] **Step 1: 加测试**

`tests/test_pressure_controller.c`（第 6 行），找到：

```c
#include "pressure_controller.h"
```

替换为：

```c
#include "motion_control.h"
#include "pressure_controller.h"
```

`tests/test_pressure_controller.c` 中，在 `int main(void) {` 之前插入：

```c
/* ===================== Legacy characterization (spec 2026-11-23 §6 T4) =====================
 * Golden values were captured on commit 0a8afa4, before the 2-DOF pressure
 * loop existed. Kq = 0 must keep the non-RBF controller and the FB pressure
 * path bit-identical. If a later, intentional change to the legacy path moves
 * these values, re-capture them in that change. */

static HYD_REAL legacy_trace_pressure(int k) {
    HYD_REAL p = 100.0f * (1.0f - expf(-(HYD_REAL)k / 300.0f)) +
                 3.0f * sinf((HYD_REAL)k * 0.05f);
    if (k >= 1500 && k < 1700) {
        p += 15.0f;
    }
    return p;
}

static void legacy_controller_trace(HYD_PressureControllerType strategy,
                                    double* sumOut, HYD_REAL* lastOut) {
    HYD_MotionSegment segment;
    HYD_PressureControllerState state;
    HYD_PressureControllerInput input;
    HYD_PressureControllerOutput output;
    int k;

    memset(&segment, 0, sizeof(segment));
    segment.mode = HYD_MODE_PRESSURE_CLOSED_LOOP;
    segment.endCondition = HYD_END_MANUAL;
    segment.direction = HYD_DIRECTION_HOLD;
    segment.targetPressure = 100.0f;
    segment.maxFlow = 20.0f;
    segment.pressureController = strategy;
    segment.pressureKp = 0.5f;
    segment.pressureKi = 4.0f;
    segment.pressureKd = 0.01f;
    segment.pressureIntegralLimit = 10.0f;
    segment.pressureDeadband = 0.005f;
    segment.pressureDerivativeFilterAlpha = 0.5f;

    memset(&input, 0, sizeof(input));
    input.outputMin = -7.2f;
    input.outputMax = 20.0f;
    input.flowToPumpSpeedGain = 20.0f;
    input.pumpSpeedLimit = 1800.0f;
    input.feedforwardFlow = 0.5f;

    HYD_PressureController_InitState(&state, 0.0f, 0.0f, 0.0f);
    *sumOut = 0.0;
    *lastOut = 0.0f;
    for (k = 1; k <= 2000; ++k) {
        input.targetPressure = (k < 1000) ? 100.0f : 60.0f;
        input.measuredPressure = legacy_trace_pressure(k);
        input.timestamp = (HYD_TIME)k * 0.001f;
        HYD_PressureController_Execute(&segment, &state, &input, &output);
        *sumOut += (double)output.outputFlow;
        *lastOut = output.outputFlow;
    }
}

static void legacy_fb_trace(double* sumSpeed, HYD_REAL* lastSpeed) {
    static HYD_MotionControlFB fb;
    HYD_MotionSegment segment;
    HYD_REAL pressure = 0.0f;
    int k;

    HYD_MotionControlFB_Init(&fb);
    fb.USE_RECIPE = false;
    memset(&segment, 0, sizeof(segment));
    segment.segmentType = HYD_SEGMENT_TYPE_HOLDING;
    segment.mode = HYD_MODE_PRESSURE_CLOSED_LOOP;
    segment.endCondition = HYD_END_MANUAL;
    segment.direction = HYD_DIRECTION_HOLD;
    segment.targetPressure = 100.0f;
    segment.maxFlow = 20.0f;
    segment.pressureRampRate = 20.0f;
    segment.pressureController = HYD_PRESSURE_CONTROLLER_PI;
    segment.pressureKp = 0.5f;
    segment.pressureKi = 4.0f;
    segment.pressureIntegralLimit = 10.0f;
    segment.pressureDeadband = 0.005f;

    fb.AXIS_REF.pressure = 0.0f;
    fb.AXIS_REF.timestamp = 0.0f;
    assert(HYD_MotionControlFB_LoadDirectSegment(&fb, &segment));
    assert(HYD_MotionControlFB_StartSegment(&fb, 0U, 0.0f));
    *sumSpeed = 0.0;
    *lastSpeed = 0.0f;
    for (k = 1; k <= 1500; ++k) {
        fb.AXIS_REF.timestamp = (HYD_TIME)k * 0.001f;
        fb.AXIS_REF.pressure = pressure;
        HYD_MotionControlFB_Cycle(&fb);
        pressure += 64.0f * (fb.PUMP_SPEED / 20.0f - pressure / 104.0f) * 0.001f;
        *sumSpeed += (double)fb.PUMP_SPEED;
        *lastSpeed = fb.PUMP_SPEED;
    }
}

static void test_legacy_path_is_bit_identical(void) {
    double sum;
    HYD_REAL last;

    legacy_controller_trace(HYD_PRESSURE_CONTROLLER_P, &sum, &last);
    assert(sum == 0x1.0168b5c18p+12 && last == -0x1.ccccccp+2f);
    legacy_controller_trace(HYD_PRESSURE_CONTROLLER_PI, &sum, &last);
    assert(sum == 0x1.424019228p+13 && last == -0x1.ccccccp+2f);
    legacy_controller_trace(HYD_PRESSURE_CONTROLLER_PID, &sum, &last);
    assert(sum == 0x1.3f431ec84p+13 && last == -0x1.ccccccp+2f);
    legacy_fb_trace(&sum, &last);
    assert(sum == 0x1.cfb7e910cp+15 && last == 0x1.33b09cp+4f);
}
```

`tests/test_pressure_controller.c`（`main` 中），找到：

```c
    test_soft_reset_preserves_network();
```

替换为：

```c
    test_soft_reset_preserves_network();
    test_legacy_path_is_bit_identical();
```

- [ ] **Step 2: 运行，确认通过**

```bash
wsl.exe -d Ubuntu-22.04 bash -lc 'cd ~/project/hdy-motion-light && cmake --build --preset unixgcc 2>&1 | grep -E "error|warning: " | grep test_pressure_controller; ./out/build/unixgcc/test_pressure_controller'
```

Expected：没有新告警，输出 `Pressure controller tests passed.`

如果断言失败，说明工作区和 0a8afa4 不一致，停下检查，不要改基准值。

- [ ] **Step 3: 提交**

```bash
wsl.exe -d Ubuntu-22.04 bash -lc 'cd ~/project/hdy-motion-light && git add tests/test_pressure_controller.c && git commit -q -m "测试：压力控制器旧路径逐位基准（spec 2026-11-23 T4）" -m "P/PI/PID 控制器轨迹和 FB 压力段 PI 轨迹的十六进制基准，采集于 0a8afa4。Kq = 0 必须保持逐位一致。" -m "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>" && git log --oneline -1 && git status --short'
```

Expected：新提交；`git status --short` 只剩未跟踪的 `docs/data_deg/hor1121/`（以及用户原有的 `scripts/*.sh` 修改，如果有）。

---

### Task 2: §14B 常数与 profiled 参考轨迹（T1）

**Files:**
- Modify: `include/hyd_config.h`（§14B，`HYD_THRESH_PRESSURE_LIMIT_FAULT_ESCALATION_S` 之后）
- Modify: `include/ramp_controller.h`、`src/ramp_controller.c`（整文件替换）
- Modify: `tests/test_ramp_controller.c`
- Modify: `tests/test_mode_switch_bumpless.c`、`tests/test_toggle_mechanism_pool.c`（FB 尺寸预算一次性改到最终值）

**Interfaces:**
- Produces（后续任务使用）：
  - `typedef struct { HYD_REAL riseRate; HYD_REAL fallRate; HYD_REAL brakeTime; } HYD_RampProfileLimits;`（bar/s、bar/s、s）
  - `void HYD_RampController_ExecuteProfiledWithDt(HYD_RampController*, const HYD_RampControllerInput*, const HYD_RampProfileLimits*, HYD_TIME deltaTime, HYD_RampControllerOutput*);`
  - `HYD_RampControllerOutput.rampedPressureRate`（bar/s）：旧函数恒为 0。
  - `HYD_RampController` 新字段：`profileRate`、`profileRateLimit`、`profileDecel`、`profileLatched`。
  - §14B 全部新常数（Task 3–5 使用）。

轨迹规则（spec §4.2，含 D2）：
- **锁存**：目标或过渡时间变化时锁存
  - `v_lim = min(v_auto, |Pt − r| / T)`；
  - `a = v_auto / Tb`。
- **每拍**：`v_t = s·min(v_lim, √(2a|rem|))`。
  - 符号与当前速度相反时，先把速度置 0；
  - 需要加速时按 `a·dt` 限幅，减速时直接取 `v_t`；
  - 越过目标时令 `r = Pt`、`v = 0`。
- **退化**：`v_lim` 或 `a` 不大于 0 时直接跳到目标。

- [ ] **Step 1: 写失败测试**

`tests/test_ramp_controller.c`（第 8–9 行），找到：

```c
#include <stdio.h>
#include <math.h>
```

替换为：

```c
#include <assert.h>
#include <stdio.h>
#include <math.h>
```

`tests/test_ramp_controller.c` 中，在 `int main(void) {` 之前插入：

```c
/* ===================== Profiled reference (spec 2026-11-23 §4.2, T1) ===================== */

typedef struct {
    HYD_REAL maxRef;
    HYD_REAL minRef;
    HYD_REAL maxAbsRate;
    HYD_REAL maxAbsRateStep;
    int arrivalStep; /* first step with ref == target and rate == 0, -1 if never */
} ProfileRun;

static const HYD_RampProfileLimits kProfileLimits = {600.0f, 300.0f, 0.12f};

static void run_profile(HYD_RampController* controller, HYD_REAL target, HYD_REAL rampMs,
                        const HYD_RampProfileLimits* limits, int steps, ProfileRun* run) {
    HYD_RampControllerInput input;
    HYD_RampControllerOutput output;
    HYD_REAL previousRate = controller->profileRate;
    int k;

    run->maxRef = controller->rampedPressure;
    run->minRef = controller->rampedPressure;
    run->maxAbsRate = 0.0f;
    run->maxAbsRateStep = 0.0f;
    run->arrivalStep = -1;
    input.targetPressure = target;
    input.rampRate = rampMs;
    for (k = 1; k <= steps; ++k) {
        input.currentTime = controller->lastTimestamp + 0.001f;
        HYD_RampController_ExecuteProfiledWithDt(controller, &input, limits, 0.001f, &output);
        if (output.rampedPressure > run->maxRef) run->maxRef = output.rampedPressure;
        if (output.rampedPressure < run->minRef) run->minRef = output.rampedPressure;
        if (fabs(output.rampedPressureRate) > run->maxAbsRate) {
            run->maxAbsRate = (HYD_REAL)fabs(output.rampedPressureRate);
        }
        if (fabs(output.rampedPressureRate - previousRate) > run->maxAbsRateStep) {
            run->maxAbsRateStep = (HYD_REAL)fabs(output.rampedPressureRate - previousRate);
        }
        previousRate = output.rampedPressureRate;
        if (run->arrivalStep < 0 && output.rampedPressure == target &&
            output.rampedPressureRate == 0.0f) {
            run->arrivalStep = k;
        }
    }
}

static void test_profile_rise_is_bounded_and_lands_exactly(void) {
    HYD_RampController controller;
    ProfileRun run;
    /* a = 600 / 0.12 = 5000 bar/s^2: accelerate 120 ms (36 bar), cruise
     * 28 bar at 600 bar/s (47 ms), brake 120 ms (36 bar) -> about 287 ms. */
    HYD_RampController_Init(&controller, 0.0f, 0.0f);
    run_profile(&controller, 100.0f, 20.0f, &kProfileLimits, 1000, &run);

    assert(run.maxRef <= 100.0f);
    assert(run.maxAbsRate <= 600.0f + 1.0e-3f);
    assert(run.maxAbsRateStep <= 3.0f * 5000.0f * 0.001f);
    assert(run.arrivalStep >= 270 && run.arrivalStep <= 300);
    assert(controller.rampedPressure == 100.0f);
    printf("✓ profile rise: arrival %d ms, max slope %.1f bar/s\n",
           run.arrivalStep, (double)run.maxAbsRate);
}

static void test_profile_fall_uses_fall_bound(void) {
    HYD_RampController controller;
    ProfileRun run;

    HYD_RampController_Init(&controller, 140.0f, 0.0f);
    run_profile(&controller, 80.0f, 20.0f, &kProfileLimits, 1000, &run);

    assert(run.minRef >= 80.0f);
    assert(run.maxAbsRate <= 300.0f + 1.0e-3f);
    assert(run.maxAbsRate >= 290.0f);
    assert(run.arrivalStep > 0);
    assert(controller.rampedPressure == 80.0f);
    printf("✓ profile fall: arrival %d ms, max slope %.1f bar/s\n",
           run.arrivalStep, (double)run.maxAbsRate);
}

static void test_profile_slower_user_time_wins(void) {
    HYD_RampController controller;
    ProfileRun run;

    HYD_RampController_Init(&controller, 0.0f, 0.0f);
    run_profile(&controller, 100.0f, 1000.0f, &kProfileLimits, 1500, &run);

    assert(run.maxAbsRate <= 100.0f + 1.0e-3f);
    assert(run.arrivalStep >= 1000 && run.arrivalStep <= 1100);
    printf("✓ profile 1000 ms transition: arrival %d ms\n", run.arrivalStep);
}

static void test_profile_reversal_stops_reference(void) {
    HYD_RampController controller;
    ProfileRun run;
    HYD_REAL atSwitch;

    HYD_RampController_Init(&controller, 0.0f, 0.0f);
    run_profile(&controller, 140.0f, 20.0f, &kProfileLimits, 200, &run);
    atSwitch = controller.rampedPressure;
    assert(atSwitch > 70.0f);
    assert(controller.profileRate > 100.0f);

    run_profile(&controller, 60.0f, 20.0f, &kProfileLimits, 1000, &run);
    assert(run.maxRef <= atSwitch);
    assert(run.minRef >= 60.0f);
    assert(controller.rampedPressure == 60.0f);
    printf("✓ profile reversal: stopped at %.2f bar, landed on 60\n", (double)atSwitch);
}

static void test_profile_retarget_same_direction_is_continuous(void) {
    HYD_RampController controller;
    ProfileRun run;

    HYD_RampController_Init(&controller, 0.0f, 0.0f);
    run_profile(&controller, 100.0f, 20.0f, &kProfileLimits, 100, &run);
    run_profile(&controller, 120.0f, 20.0f, &kProfileLimits, 1000, &run);

    assert(run.maxAbsRateStep <= 3.0f * 5000.0f * 0.001f);
    assert(run.maxRef <= 120.0f);
    assert(controller.rampedPressure == 120.0f);
    printf("✓ profile retarget 100 -> 120 while rising is continuous\n");
}

static void test_profile_zero_bounds_snap_to_target(void) {
    HYD_RampController controller;
    HYD_RampProfileLimits zero = {0.0f, 0.0f, 0.12f};
    ProfileRun run;

    HYD_RampController_Init(&controller, 10.0f, 0.0f);
    run_profile(&controller, 50.0f, 20.0f, &zero, 1, &run);
    assert(controller.rampedPressure == 50.0f);
    assert(run.maxAbsRate == 0.0f);
    printf("✓ profile with zero bounds snaps to the target\n");
}

static void test_legacy_reports_zero_rate(void) {
    HYD_RampController controller;
    HYD_RampControllerInput input;
    HYD_RampControllerOutput output;

    HYD_RampController_Init(&controller, 0.0f, 0.0f);
    input.targetPressure = 100.0f;
    input.rampRate = 100.0f;
    input.currentTime = 0.001f;
    output.rampedPressureRate = 123.0f;
    HYD_RampController_ExecuteWithDt(&controller, &input, 0.001f, &output);
    assert(output.rampedPressure > 0.0f && output.rampedPressure < 100.0f);
    assert(output.rampedPressureRate == 0.0f);
    printf("✓ legacy ramp reports zero slope\n");
}
```

`tests/test_ramp_controller.c`（`main` 中），找到：

```c
    test_zero_ramp_rate();
```

替换为：

```c
    test_zero_ramp_rate();
    test_profile_rise_is_bounded_and_lands_exactly();
    test_profile_fall_uses_fall_bound();
    test_profile_slower_user_time_wins();
    test_profile_reversal_stops_reference();
    test_profile_retarget_same_direction_is_continuous();
    test_profile_zero_bounds_snap_to_target();
    test_legacy_reports_zero_rate();
```

- [ ] **Step 2: 运行，确认失败**

```bash
wsl.exe -d Ubuntu-22.04 bash -lc 'cd ~/project/hdy-motion-light && cmake --build out/build/unixgcc --target test_ramp_controller 2>&1 | grep -E "error" | head -5'
```

Expected：编译错误，提示 `HYD_RampProfileLimits` 未知、`rampedPressureRate` 不是成员、`HYD_RampController_ExecuteProfiledWithDt` 隐式声明等。

- [ ] **Step 3: 加 §14B 常数**

`include/hyd_config.h`（约第 469–471 行），找到：

```c
#define HYD_THRESH_PRESSURE_LIMIT_FAULT_ESCALATION_S  1.0f

/* --- 软限位保护阈值（src/output_limiter.c） --- */
```

替换为：

```c
#define HYD_THRESH_PRESSURE_LIMIT_FAULT_ESCALATION_S  1.0f

/* --- 压力闭环二自由度结构（src/ramp_controller.c、src/pressure_controller.c、
 *     src/motion_control.c；spec 2026-11-23 §4.2–§4.4） --- */

/* η：参考轨迹速率上限只占可用流量的这一比例，余量留给反馈。
 * v_rise = η·Kq_f·max(Qmax − Pt/R_f, r·Qmax)，v_fall = η·Kq_f·max(|Qrev| + Pt/R_f, r·Qmax)。
 * 单位：无量纲。 */
#ifndef HYD_THRESH_PRESSURE_PROFILE_RATE_MARGIN
    #define HYD_THRESH_PRESSURE_PROFILE_RATE_MARGIN 0.6f
#endif

/* 上式中的 r：保压流量接近 Qmax 时，速率上限不低于 r·Qmax 对应的速率。
 * 单位：无量纲（相对 Qmax）。 */
#ifndef HYD_THRESH_PRESSURE_PROFILE_MIN_FLOW_RATIO
    #define HYD_THRESH_PRESSURE_PROFILE_MIN_FLOW_RATIO 0.1f
#endif

/* 参数 38 PRESSURE_BRAKE_TIME 为 0 时的制动时间：减速度 = 速率上限 / 该时间。
 * 单位：s。 */
#ifndef HYD_THRESH_PRESSURE_PROFILE_BRAKE_TIME_S
    #define HYD_THRESH_PRESSURE_PROFILE_BRAKE_TIME_S 0.12f
#endif

/* 参数 38 允许写入的上限。单位：ms。 */
#ifndef HYD_MAX_PRESSURE_BRAKE_TIME_MS
    #define HYD_MAX_PRESSURE_BRAKE_TIME_MS 2000.0f
#endif

/* 反馈误差用的参考延迟拍数（1 拍 = 1 ms），与“MA3 + 伺服起步”的回路延迟匹配，
 * 前馈正常起效时反馈不抢油。整数，数组维度。 */
#ifndef HYD_PRESSURE_REF_DELAY_SAMPLES
    #define HYD_PRESSURE_REF_DELAY_SAMPLES 12
#endif

/* --- 泵轴角同步纹波补偿（src/pressure_ripple_comp.c；spec 2026-11-23 §4.5） --- */

/* LMS 学习速率。单位：1/s。 */
#ifndef HYD_THRESH_PRESSURE_RIPPLE_MU
    #define HYD_THRESH_PRESSURE_RIPPLE_MU 4.0f
#endif

/* 次级通道模型里的纯延迟。单位：s。 */
#ifndef HYD_THRESH_PRESSURE_RIPPLE_PATH_DELAY_S
    #define HYD_THRESH_PRESSURE_RIPPLE_PATH_DELAY_S 0.006f
#endif

/* 低于该泵转速不学习。单位：rpm。 */
#ifndef HYD_THRESH_PRESSURE_RIPPLE_MIN_RPM
    #define HYD_THRESH_PRESSURE_RIPPLE_MIN_RPM 3.0f
#endif

/* |误差| 不小于该值时不学习。单位：bar。 */
#ifndef HYD_THRESH_PRESSURE_RIPPLE_MAX_ERROR
    #define HYD_THRESH_PRESSURE_RIPPLE_MAX_ERROR 3.0f
#endif

/* 参考静止并且反馈有效持续该时间后，才开始学习和淡入。单位：s。 */
#ifndef HYD_THRESH_PRESSURE_RIPPLE_HOLD_S
    #define HYD_THRESH_PRESSURE_RIPPLE_HOLD_S 0.05f
#endif

/* 输出淡入 / 淡出时间。单位：s。 */
#ifndef HYD_THRESH_PRESSURE_RIPPLE_FADE_IN_S
    #define HYD_THRESH_PRESSURE_RIPPLE_FADE_IN_S 0.05f
#endif
#ifndef HYD_THRESH_PRESSURE_RIPPLE_FADE_OUT_S
    #define HYD_THRESH_PRESSURE_RIPPLE_FADE_OUT_S 0.02f
#endif

/* 每阶补偿流量幅值上限。单位：L/min。 */
#ifndef HYD_THRESH_PRESSURE_RIPPLE_MAX_FLOW
    #define HYD_THRESH_PRESSURE_RIPPLE_MAX_FLOW 1.5f
#endif

/* 2N 阶频率不低于该值时淡出且不学习（采样 1 kHz，相位模型失真）。单位：Hz。 */
#ifndef HYD_THRESH_PRESSURE_RIPPLE_MAX_FREQ_HZ
    #define HYD_THRESH_PRESSURE_RIPPLE_MAX_FREQ_HZ 100.0f
#endif

/* 参数 39 PUMP_RIPPLE_ORDER 允许写入的上限（整数）。 */
#ifndef HYD_MAX_PUMP_RIPPLE_ORDER
    #define HYD_MAX_PUMP_RIPPLE_ORDER 64
#endif

/* --- 软限位保护阈值（src/output_limiter.c） --- */
```

- [ ] **Step 4: 实现轨迹**

`include/ramp_controller.h` 整文件替换为：

```c
#ifndef HYD_RAMP_CONTROLLER_H
#define HYD_RAMP_CONTROLLER_H

#include "common_types.h"

typedef struct {
    HYD_REAL rampedPressure;
    HYD_TIME lastTimestamp;
    HYD_REAL transitionTargetPressure;
    HYD_REAL transitionRate;
    HYD_TIME transitionDuration;
    HYD_REAL profileRate;      /* bar/s, profiled reference slope */
    HYD_REAL profileRateLimit; /* bar/s, latched when the target or time changes */
    HYD_REAL profileDecel;     /* bar/s^2, latched together with profileRateLimit */
    HYD_BOOL profileLatched;
} HYD_RampController;

typedef struct {
    HYD_REAL targetPressure;
    HYD_REAL rampRate; /* ms pressure transition time */
    HYD_TIME currentTime;
} HYD_RampControllerInput;

/* Profiled reference limits (spec 2026-11-23 §4.2). */
typedef struct {
    HYD_REAL riseRate;  /* bar/s, slope bound when the target is above the reference */
    HYD_REAL fallRate;  /* bar/s, slope bound when the target is below the reference */
    HYD_REAL brakeTime; /* s, deceleration = slope bound / brakeTime */
} HYD_RampProfileLimits;

typedef struct {
    HYD_REAL rampedPressure;
    HYD_REAL rampedPressureRate; /* bar/s; the legacy time-based functions always report 0 */
} HYD_RampControllerOutput;

void HYD_RampController_Init(HYD_RampController* controller, HYD_REAL initialPressure, HYD_TIME initialTime);
void HYD_RampController_Execute(HYD_RampController* controller, const HYD_RampControllerInput* input, HYD_RampControllerOutput* output);
void HYD_RampController_ExecuteWithDt(HYD_RampController* controller,
                                      const HYD_RampControllerInput* input,
                                      HYD_TIME deltaTime,
                                      HYD_RampControllerOutput* output);
/* Rate-limited reference with sqrt braking. input->rampRate becomes the
 * minimum transition time: the slower of |target - reference| / time and the
 * latched slope bound wins. Never passes the target; reports the slope. */
void HYD_RampController_ExecuteProfiledWithDt(HYD_RampController* controller,
                                              const HYD_RampControllerInput* input,
                                              const HYD_RampProfileLimits* limits,
                                              HYD_TIME deltaTime,
                                              HYD_RampControllerOutput* output);

#endif /* HYD_RAMP_CONTROLLER_H */
```

`src/ramp_controller.c` 整文件替换为（`Init` 增加 4 个字段的清零；`ExecuteWithDt` 末尾增加 `output->rampedPressureRate = 0.0;`；末尾新增 profiled 函数；其余逐字不变）：

```c
#include "ramp_controller.h"
#include "motion_time.h"

void HYD_RampController_Init(HYD_RampController* controller, HYD_REAL initialPressure, HYD_TIME initialTime) {
    if (controller == NULL) {
        return;
    }
    controller->rampedPressure = initialPressure;
    controller->lastTimestamp = initialTime;
    controller->transitionTargetPressure = initialPressure;
    controller->transitionRate = 0.0;
    controller->transitionDuration = 0.0;
    controller->profileRate = 0.0;
    controller->profileRateLimit = 0.0;
    controller->profileDecel = 0.0;
    controller->profileLatched = false;
}

void HYD_RampController_ExecuteWithDt(HYD_RampController* controller,
                                      const HYD_RampControllerInput* input,
                                      HYD_TIME deltaTime,
                                      HYD_RampControllerOutput* output) {
    if (controller == NULL || input == NULL || output == NULL) {
        return;
    }

    if (deltaTime < 0.0) {
        deltaTime = 0.0;
    }
    controller->lastTimestamp = input->currentTime;

    {
        HYD_TIME transitionTime = HYD_PressureTimeMsToSeconds(input->rampRate);
        if (input->targetPressure != controller->transitionTargetPressure ||
            transitionTime != controller->transitionDuration) {
            controller->transitionTargetPressure = input->targetPressure;
            controller->transitionDuration = transitionTime;
            controller->transitionRate = (transitionTime > 0.0)
                ? (input->targetPressure - controller->rampedPressure) / transitionTime
                : 0.0;
        }

        if (fabs(input->targetPressure - controller->rampedPressure) <=
                fabs(controller->transitionRate * deltaTime) + 1e-6f ||
            transitionTime <= 0.0) {
            controller->rampedPressure = input->targetPressure;
        } else {
            controller->rampedPressure += controller->transitionRate * deltaTime;
        }
    }

    output->rampedPressure = controller->rampedPressure;
    output->rampedPressureRate = 0.0;
}

void HYD_RampController_Execute(HYD_RampController* controller, const HYD_RampControllerInput* input, HYD_RampControllerOutput* output) {
    HYD_TIME deltaTime;

    if (controller == NULL || input == NULL || output == NULL) {
        return;
    }

    deltaTime = input->currentTime - controller->lastTimestamp;
    if (deltaTime < 0.0) {
        deltaTime = 0.0;
    }
    HYD_RampController_ExecuteWithDt(controller, input, deltaTime, output);
}

void HYD_RampController_ExecuteProfiledWithDt(HYD_RampController* controller,
                                              const HYD_RampControllerInput* input,
                                              const HYD_RampProfileLimits* limits,
                                              HYD_TIME deltaTime,
                                              HYD_RampControllerOutput* output) {
    HYD_TIME transitionTime;
    HYD_REAL remaining;
    HYD_REAL direction;
    HYD_REAL slopeTarget;
    HYD_REAL brakingSlope;
    HYD_REAL maxStep;

    if (controller == NULL || input == NULL || limits == NULL || output == NULL) {
        return;
    }

    if (deltaTime < 0.0) {
        deltaTime = 0.0;
    }
    controller->lastTimestamp = input->currentTime;

    /* Latch the slope bound and the deceleration when the target or the
     * transition time changes (including CONTINUOUSUPDATE live updates).
     * The deceleration depends only on the automatic bound, so braking does
     * not get weaker when the user asks for a slow transition. */
    transitionTime = HYD_PressureTimeMsToSeconds(input->rampRate);
    if (!controller->profileLatched ||
        input->targetPressure != controller->transitionTargetPressure ||
        transitionTime != controller->transitionDuration) {
        HYD_REAL distance = input->targetPressure - controller->rampedPressure;
        HYD_REAL slopeBound = (distance >= 0.0) ? limits->riseRate : limits->fallRate;

        controller->transitionTargetPressure = input->targetPressure;
        controller->transitionDuration = transitionTime;
        controller->profileRateLimit = slopeBound;
        if (transitionTime > 0.0 && fabs(distance) / transitionTime < slopeBound) {
            controller->profileRateLimit = fabs(distance) / transitionTime;
        }
        controller->profileDecel = (limits->brakeTime > 0.0)
            ? slopeBound / limits->brakeTime : 0.0;
        controller->profileLatched = true;
    }

    if (controller->profileRateLimit <= 0.0 || controller->profileDecel <= 0.0) {
        controller->rampedPressure = input->targetPressure;
        controller->profileRate = 0.0;
    } else {
        remaining = input->targetPressure - controller->rampedPressure;
        direction = (remaining >= 0.0) ? 1.0 : -1.0;
        brakingSlope = sqrt(2.0 * controller->profileDecel * fabs(remaining));
        slopeTarget = direction * ((brakingSlope < controller->profileRateLimit)
            ? brakingSlope : controller->profileRateLimit);

        /* Reversal: stop the reference first, never coast past the old
         * target while turning around. */
        if (slopeTarget * controller->profileRate < 0.0) {
            controller->profileRate = 0.0;
        }
        if (fabs(slopeTarget) > fabs(controller->profileRate)) {
            maxStep = controller->profileDecel * deltaTime;
            controller->profileRate += HYD_ClampReal(slopeTarget - controller->profileRate,
                                                     -maxStep, maxStep);
        } else {
            controller->profileRate = slopeTarget;
        }

        controller->rampedPressure += controller->profileRate * deltaTime;
        if ((input->targetPressure - controller->rampedPressure) * direction <= 0.0) {
            controller->rampedPressure = input->targetPressure;
            controller->profileRate = 0.0;
        }
    }

    output->rampedPressure = controller->rampedPressure;
    output->rampedPressureRate = controller->profileRate;
}
```

- [ ] **Step 5: FB 尺寸预算改到最终值**

本特性完成后 FB 共增加 104 字节：
- 轨迹状态 +16；
- 控制器延迟线 +52；
- 参数 +12；
- 纹波系数 +28；
- 合计 108，其中 4 字节被对齐吸收。

两处预算这里一次改到最终值，中间各任务就不用反复改。

`tests/test_mode_switch_bumpless.c`（约第 447–449 行），找到：

```c
     * +16: 两份 HYD_ContinuousAbsoluteContext 各加 flowPercent / flowRampRate。 */
    assert(sizeof(HYD_MotionControlFB) <= 3328U);
```

替换为：

```c
     * +16: 两份 HYD_ContinuousAbsoluteContext 各加 flowPercent / flowRampRate。
     * +104: 压力环二自由度（参考延迟线、轨迹状态、参数 37–39、纹波补偿系数）。 */
    assert(sizeof(HYD_MotionControlFB) <= 3432U);
```

`tests/test_toggle_mechanism_pool.c`（第 7 行），找到：

```c
#define HYD_BASELINE_MOTION_FB_BYTES 3280U /* host ABI at current pre-feature baseline */
```

替换为：

```c
#define HYD_BASELINE_MOTION_FB_BYTES 3384U /* host ABI without toggle telemetry; +104 for the 2-DOF pressure loop (2026-11-23) */
```

- [ ] **Step 6: 运行，确认通过**

```bash
wsl.exe -d Ubuntu-22.04 bash -lc 'cd ~/project/hdy-motion-light && cmake --build --preset unixgcc 2>&1 | grep -E "error|warning: " | grep -v "rbf_pid.c\|state_reporter.c\|test_diagnostic_criteria.c"; ./out/build/unixgcc/test_ramp_controller | grep -E "^✓ (profile|legacy)"; ./out/build/unixgcc/test_ramp_controller > /dev/null; echo exit=$?'
```

Expected：没有新告警，输出：

```
✓ profile rise: arrival 284 ms, max slope 600.0 bar/s
✓ profile fall: arrival 317 ms, max slope 300.0 bar/s
✓ profile 1000 ms transition: arrival 1018 ms
✓ profile reversal: stopped at 84.30 bar, landed on 60
✓ profile retarget 100 -> 120 while rising is continuous
✓ profile with zero bounds snaps to the target
✓ legacy ramp reports zero slope
exit=0
```

文件里原有的 5 个旧测试会照常打印 11 行 `✗ FAIL`。这是原有现象：旧测试把 `rampRate` 当成 bar/s，而它实际是 ms。这些测试不做断言，退出码为 0。不要修它们。

- [ ] **Step 7: 全量回归**

```bash
wsl.exe -d Ubuntu-22.04 bash -lc 'cd ~/project/hdy-motion-light && ctest --test-dir out/build/unixgcc --output-on-failure 2>&1 | tail -3'
```

Expected：`100% tests passed, 0 tests failed out of 62`（Task 1 的逐位基准仍通过，旧路径不受影响）。

- [ ] **Step 8: 提交**

```bash
wsl.exe -d Ubuntu-22.04 bash -lc 'cd ~/project/hdy-motion-light && git add include/hyd_config.h include/ramp_controller.h src/ramp_controller.c tests/test_ramp_controller.c tests/test_mode_switch_bumpless.c tests/test_toggle_mechanism_pool.c && git commit -q -m "压力参考轨迹：速率限制 + 平方根制动的 profiled 执行函数（spec 2026-11-23 §4.2）" -m "新增 HYD_RampController_ExecuteProfiledWithDt，输出 r 与斜率；反向改目标时先停再反向。旧函数行为不变，斜率报 0。§14B 加入本特性全部常数；FB 尺寸预算改到 3432。" -m "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>" && git log --oneline -1 && git status --short'
```

---

### Task 3: 泵轴角同步纹波补偿模块（T5）

**Files:**
- Create: `include/pressure_ripple_comp.h`、`src/pressure_ripple_comp.c`
- Create: `tests/test_pressure_ripple_comp.c`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes：Task 2 的 `HYD_THRESH_PRESSURE_RIPPLE_*` 常数；`HYD_PumpFeedback`、`HYD_PumpFeedback_HasValid`（`common_types.h`）。
- Produces（Task 4、6 使用）：
  - `HYD_PressureRippleCompState`：`coeffRe[2]`、`coeffIm[2]`、`fade`、`holdTime`、`order`、`lastClipped`。`lastClipped` 由调用方每拍写入。
  - `HYD_PressureRippleCompInput`：`order`、`pumpFeedback`、`referenceMoving`、`error`、`kqFlow`、`leakResistanceFlow`、`kp`、`ki`、`dt`。
  - `HYD_PressureRippleCompOutput`：`flow`、`amplitude[2]`、`learning`。
  - `void HYD_PressureRippleComp_Clear(HYD_PressureRippleCompState*);`
  - `void HYD_PressureRippleComp_Execute(HYD_PressureRippleCompState*, const HYD_PressureRippleCompInput*, HYD_PressureRippleCompOutput*);`

算法（spec §4.5，含 D4）：
- **系数**：每阶一个复系数 c_k，按每 rpm 存储。
- **输出**：u = fade·n·Σ Re{c_k·e^(jkNθ)}，每阶幅值 ≤ 1.5 L/min。
- **学习律**：c_k += μ·dt·2·e·e^(−jkNθ)/(Ph_k·n)，其中
  - 1/Ph = (1/R_f + jω/Kq_f)·e^(jωTd) + Kp + Ki/(jω)；
  - 2N 阶用倍角公式，每拍只算一对 sin/cos 加一对延迟相位。
- **门控**：静止 ≥ 50 ms 且反馈有效、2N·n/60 < 100 Hz 时淡入，否则淡出；学习还要求以下全部满足：
  - fade = 1；
  - n ≥ 3 rpm；
  - |e| < 3 bar；
  - 上一拍输出未触限。
- **阶次切换**：`order` 改变时清零系数。

- [ ] **Step 1: 写失败测试**

新建 `tests/test_pressure_ripple_comp.c`：

```c
/* Pump-angle-synchronous ripple compensation (spec 2026-11-23 §4.5, T5). */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "pressure_ripple_comp.h"
#include "pressure_controller.h"

#define DT 0.001
#define TWO_PI 6.283185307179586
#define KQ_FLOW 64.0          /* bar/s per L/min */
#define R_FLOW 104.0          /* bar per L/min */
#define PUMP_RPM 20.0
#define LOOP_DELAY 5          /* scans from command to flow */
#define TARGET 100.0

typedef struct {
    HYD_PressureRippleCompState comp;
    double pressure;
    double integral;
    double angle;             /* rad */
    double delayLine[LOOP_DELAY];
    int delayIndex;
    double kp;
    double ki;
} HoldLoop;

static void hold_loop_init(HoldLoop* loop) {
    int i;
    memset(loop, 0, sizeof(*loop));
    HYD_PressureRippleComp_Clear(&loop->comp);
    loop->pressure = TARGET;
    loop->kp = TWO_PI * 5.0 / KQ_FLOW;
    loop->ki = loop->kp * TWO_PI * 5.0 / 4.0;
    for (i = 0; i < LOOP_DELAY; ++i) loop->delayLine[i] = TARGET / R_FLOW;
}

/* Runs the hold loop; returns the order-12 pressure amplitude over the last
 * `windowSteps` steps (use whole ripple periods: 250 steps at 20 rpm).
 * `order` 0 disables compensation. */
static double hold_loop_run(HoldLoop* loop, HYD_UINT8 order, int steps, int windowSteps,
                            HYD_BOOL feedbackValid, HYD_BOOL moving, double* lastFlow) {
    double sumCos = 0.0;
    double sumSin = 0.0;
    int k;

    for (k = 0; k < steps; ++k) {
        HYD_PressureRippleCompInput input;
        HYD_PressureRippleCompOutput output;
        double error = TARGET - loop->pressure;
        double command;
        double applied;
        double w12;
        double ripple;

        memset(&input, 0, sizeof(input));
        input.order = order;
        input.pumpFeedback.rpm = PUMP_RPM;
        input.pumpFeedback.angleDeg = fmod(loop->angle, TWO_PI) * 360.0 / TWO_PI;
        input.pumpFeedback.validFlags = feedbackValid
            ? (HYD_PUMP_FEEDBACK_VALID_RPM | HYD_PUMP_FEEDBACK_VALID_ANGLE) : 0u;
        input.referenceMoving = moving;
        input.error = (HYD_REAL)error;
        input.kqFlow = KQ_FLOW;
        input.leakResistanceFlow = R_FLOW;
        input.kp = (HYD_REAL)loop->kp;
        input.ki = (HYD_REAL)loop->ki;
        input.dt = DT;
        HYD_PressureRippleComp_Execute(&loop->comp, &input, &output);
        if (lastFlow != NULL) *lastFlow = output.flow;

        loop->integral += loop->ki * error * DT;
        command = TARGET / R_FLOW + loop->kp * error + loop->integral + output.flow;
        loop->comp.lastClipped = HYD_FALSE;

        applied = loop->delayLine[loop->delayIndex];
        loop->delayLine[loop->delayIndex] = command;
        loop->delayIndex = (loop->delayIndex + 1) % LOOP_DELAY;

        loop->angle += PUMP_RPM / 60.0 * TWO_PI * DT;
        w12 = 12.0 * PUMP_RPM / 60.0 * TWO_PI;
        ripple = 1.2 * w12 / KQ_FLOW * sin(12.0 * loop->angle + 0.7) +
                 0.36 * w12 / KQ_FLOW * sin(24.0 * loop->angle + 1.9);
        loop->pressure += KQ_FLOW * (applied + ripple - loop->pressure / R_FLOW) * DT;

        if (k >= steps - windowSteps) {
            sumCos += (loop->pressure - TARGET) * cos(12.0 * loop->angle);
            sumSin += (loop->pressure - TARGET) * sin(12.0 * loop->angle);
        }
    }
    return 2.0 * sqrt(sumCos * sumCos + sumSin * sumSin) / (double)windowSteps;
}

static void test_order12_amplitude_drops_80_percent_in_3s(void) {
    HoldLoop loop;
    double baseline;
    double compensated;

    hold_loop_init(&loop);
    baseline = hold_loop_run(&loop, 0U, 1000, 500, HYD_TRUE, HYD_FALSE, NULL);

    hold_loop_init(&loop);
    compensated = hold_loop_run(&loop, 12U, 3000, 500, HYD_TRUE, HYD_FALSE, NULL);

    printf("order-12 amplitude: %.3f bar -> %.3f bar\n", baseline, compensated);
    assert(baseline > 0.5);
    assert(compensated < 0.2 * baseline);
}

static void test_coefficients_survive_and_converge_faster_on_reentry(void) {
    HoldLoop loop;
    HYD_PressureRippleCompState learned;
    double fresh;
    double reentry;

    hold_loop_init(&loop);
    fresh = hold_loop_run(&loop, 12U, 500, 250, HYD_TRUE, HYD_FALSE, NULL);

    hold_loop_init(&loop);
    (void)hold_loop_run(&loop, 12U, 3000, 500, HYD_TRUE, HYD_FALSE, NULL);
    learned = loop.comp;

    hold_loop_init(&loop);
    loop.comp = learned;
    loop.comp.fade = 0.0;
    loop.comp.holdTime = 0.0;
    reentry = hold_loop_run(&loop, 12U, 500, 250, HYD_TRUE, HYD_FALSE, NULL);
    printf("amplitude 0.25-0.5 s: fresh %.3f bar, re-entry %.3f bar\n", fresh, reentry);
    assert(reentry < 0.5 * fresh);
    assert(reentry < 0.2);
}

static void test_moving_reference_neither_learns_nor_outputs(void) {
    HoldLoop loop;
    double lastFlow = 1.0;

    hold_loop_init(&loop);
    (void)hold_loop_run(&loop, 12U, 500, 100, HYD_TRUE, HYD_TRUE, &lastFlow);
    assert(loop.comp.coeffRe[0] == 0.0 && loop.comp.coeffIm[0] == 0.0);
    assert(lastFlow == 0.0);
    assert(loop.comp.fade == 0.0);
}

static void test_invalid_feedback_fades_out_within_20ms(void) {
    HoldLoop loop;
    double lastFlow = 1.0;
    HYD_REAL coeff;

    hold_loop_init(&loop);
    (void)hold_loop_run(&loop, 12U, 1000, 100, HYD_TRUE, HYD_FALSE, NULL);
    assert(loop.comp.fade == 1.0);
    coeff = loop.comp.coeffRe[0];

    (void)hold_loop_run(&loop, 12U, 20, 10, HYD_FALSE, HYD_FALSE, &lastFlow);
    assert(loop.comp.fade < 1.0e-6);
    assert(fabs(lastFlow) < 1.0e-6);
    assert(loop.comp.coeffRe[0] == coeff);
}

static void test_clipped_output_freezes_learning(void) {
    HYD_PressureRippleCompState comp;
    HYD_PressureRippleCompInput input;
    HYD_PressureRippleCompOutput output;
    int k;

    HYD_PressureRippleComp_Clear(&comp);
    memset(&input, 0, sizeof(input));
    input.order = 12U;
    input.pumpFeedback.rpm = PUMP_RPM;
    input.pumpFeedback.validFlags = HYD_PUMP_FEEDBACK_VALID_RPM | HYD_PUMP_FEEDBACK_VALID_ANGLE;
    input.error = 1.0;
    input.kqFlow = KQ_FLOW;
    input.leakResistanceFlow = R_FLOW;
    input.kp = 0.5;
    input.ki = 4.0;
    input.dt = DT;
    for (k = 0; k < 200; ++k) {
        input.pumpFeedback.angleDeg = (HYD_REAL)k * 0.12;
        comp.lastClipped = HYD_TRUE;
        HYD_PressureRippleComp_Execute(&comp, &input, &output);
        assert(!output.learning);
    }
    assert(comp.coeffRe[0] == 0.0 && comp.coeffIm[0] == 0.0);
    assert(comp.fade > 0.0);   /* output gating is independent of the freeze */

    comp.lastClipped = HYD_FALSE;
    HYD_PressureRippleComp_Execute(&comp, &input, &output);
    assert(output.learning);
    assert(comp.coeffRe[0] != 0.0 || comp.coeffIm[0] != 0.0);
}

static void test_amplitude_is_capped(void) {
    HYD_PressureRippleCompState comp;
    HYD_PressureRippleCompInput input;
    HYD_PressureRippleCompOutput output;

    HYD_PressureRippleComp_Clear(&comp);
    comp.order = 12U;
    comp.coeffRe[0] = 1.0;     /* 20 L/min at 20 rpm, far above the cap */
    comp.fade = 1.0;
    comp.holdTime = 1.0;
    comp.lastClipped = HYD_TRUE;
    memset(&input, 0, sizeof(input));
    input.order = 12U;
    input.pumpFeedback.rpm = PUMP_RPM;
    input.pumpFeedback.angleDeg = 0.0;
    input.pumpFeedback.validFlags = HYD_PUMP_FEEDBACK_VALID_RPM | HYD_PUMP_FEEDBACK_VALID_ANGLE;
    input.kqFlow = KQ_FLOW;
    input.dt = DT;
    HYD_PressureRippleComp_Execute(&comp, &input, &output);
    assert(fabs(output.amplitude[0] - HYD_THRESH_PRESSURE_RIPPLE_MAX_FLOW) < 1.0e-5);
    assert(fabs(output.flow) <= HYD_THRESH_PRESSURE_RIPPLE_MAX_FLOW + 1.0e-5);
}

static void test_order_zero_and_order_change(void) {
    HYD_PressureRippleCompState comp;
    HYD_PressureRippleCompInput input;
    HYD_PressureRippleCompOutput output;

    HYD_PressureRippleComp_Clear(&comp);
    comp.order = 12U;
    comp.coeffRe[0] = 0.01;
    comp.fade = 1.0;
    memset(&input, 0, sizeof(input));
    input.order = 0U;
    input.pumpFeedback.rpm = PUMP_RPM;
    input.pumpFeedback.validFlags = HYD_PUMP_FEEDBACK_VALID_RPM | HYD_PUMP_FEEDBACK_VALID_ANGLE;
    input.dt = DT;
    HYD_PressureRippleComp_Execute(&comp, &input, &output);
    assert(output.flow == 0.0);
    assert(comp.coeffRe[0] == 0.0);   /* order changed 12 -> 0: coefficients cleared */
}

static void test_controller_init_keeps_and_clear_drops_coefficients(void) {
    HYD_PressureRippleCompState comp;
    HYD_PressureControllerState state;

    HYD_PressureRippleComp_Clear(&comp);
    comp.order = 12U;
    comp.coeffRe[1] = 0.02f;
    /* The coefficients live outside the controller state, so the segment
     * start memset in InitState cannot touch them. */
    HYD_PressureController_InitState(&state, 50.0, 0.0, 0.0);
    assert(comp.coeffRe[1] == 0.02f);
    HYD_PressureRippleComp_Clear(&comp);
    assert(comp.coeffRe[1] == 0.0 && comp.order == 0U);
}

int main(void) {
    test_order12_amplitude_drops_80_percent_in_3s();
    test_coefficients_survive_and_converge_faster_on_reentry();
    test_moving_reference_neither_learns_nor_outputs();
    test_invalid_feedback_fades_out_within_20ms();
    test_clipped_output_freezes_learning();
    test_amplitude_is_capped();
    test_order_zero_and_order_change();
    test_controller_init_keeps_and_clear_drops_coefficients();
    puts("Pressure ripple compensation tests passed.");
    return 0;
}
```

`CMakeLists.txt`（约第 50–51 行），找到：

```cmake
add_executable(test_pressure_ramp_time tests/test_pressure_ramp_time.c)
target_link_libraries(test_pressure_ramp_time PRIVATE HydroMotionLib)
```

替换为：

```cmake
add_executable(test_pressure_ramp_time tests/test_pressure_ramp_time.c)
target_link_libraries(test_pressure_ramp_time PRIVATE HydroMotionLib)

add_executable(test_pressure_ripple_comp tests/test_pressure_ripple_comp.c)
target_link_libraries(test_pressure_ripple_comp PRIVATE HydroMotionLib)
```

`CMakeLists.txt`（`enable_testing()` 之后），找到：

```cmake
add_test(NAME test_pressure_ramp_time COMMAND test_pressure_ramp_time)
```

替换为：

```cmake
add_test(NAME test_pressure_ramp_time COMMAND test_pressure_ramp_time)
add_test(NAME test_pressure_ripple_comp COMMAND test_pressure_ripple_comp)
```

- [ ] **Step 2: 运行，确认失败**

```bash
wsl.exe -d Ubuntu-22.04 bash -lc 'cd ~/project/hdy-motion-light && cmake --preset unixgcc > /dev/null && cmake --build out/build/unixgcc --target test_pressure_ripple_comp 2>&1 | grep -E "error" | head -3'
```

Expected：`fatal error: pressure_ripple_comp.h: No such file or directory`。

- [ ] **Step 3: 实现模块**

新建 `include/pressure_ripple_comp.h`：

```c
#ifndef HYD_PRESSURE_RIPPLE_COMP_H
#define HYD_PRESSURE_RIPPLE_COMP_H

#include "common_types.h"

/* Pump-angle-synchronous compensation of the pressure ripple at orders N and
 * 2N of the pump shaft (spec 2026-11-23 §4.5).
 *
 * One complex flow coefficient per order, stored per rpm, learned with an LMS
 * that divides by a model of the closed-loop secondary path
 * Ph = G / (1 + C·G), G = Kq_f·e^(-jωTd) / (jω + Kq_f/R_f), C = Kp + Ki/(jω).
 * Learning and output are limited to pressure hold; the state is owned by the
 * caller so that it survives segment switches. */

#define HYD_PRESSURE_RIPPLE_HARMONICS 2

typedef struct {
    HYD_REAL coeffRe[HYD_PRESSURE_RIPPLE_HARMONICS]; /* L/min per rpm, orders N and 2N */
    HYD_REAL coeffIm[HYD_PRESSURE_RIPPLE_HARMONICS]; /* L/min per rpm */
    HYD_REAL fade;        /* 0..1 output weight */
    HYD_REAL holdTime;    /* s the reference has been still with valid feedback (capped) */
    HYD_UINT8 order;      /* N the coefficients belong to; another N clears them */
    HYD_BOOL lastClipped; /* written by the caller: controller output was limited last scan */
} HYD_PressureRippleCompState;

typedef struct {
    HYD_UINT8 order;               /* N, ripples per pump revolution; 0 disables */
    HYD_PumpFeedback pumpFeedback; /* needs RPM and ANGLE valid flags; angle increases with rpm */
    HYD_BOOL referenceMoving;      /* pressure reference slope is not zero */
    HYD_REAL error;                /* bar, controller error */
    HYD_REAL kqFlow;               /* bar/s per L/min */
    HYD_REAL leakResistanceFlow;   /* bar per L/min; 0 drops the leak term */
    HYD_REAL kp;                   /* L/min per bar */
    HYD_REAL ki;                   /* L/min per (bar·s) */
    HYD_REAL dt;                   /* s */
} HYD_PressureRippleCompInput;

typedef struct {
    HYD_REAL flow;                                     /* L/min, add to the unsaturated output */
    HYD_REAL amplitude[HYD_PRESSURE_RIPPLE_HARMONICS]; /* L/min at the current speed */
    HYD_BOOL learning;
} HYD_PressureRippleCompOutput;

void HYD_PressureRippleComp_Clear(HYD_PressureRippleCompState* state);
void HYD_PressureRippleComp_Execute(HYD_PressureRippleCompState* state,
                                    const HYD_PressureRippleCompInput* input,
                                    HYD_PressureRippleCompOutput* output);

#endif /* HYD_PRESSURE_RIPPLE_COMP_H */
```

新建 `src/pressure_ripple_comp.c`：

```c
#include "pressure_ripple_comp.h"
#include <math.h>
#include <string.h>

#define HYD_RIPPLE_TWO_PI 6.28318530717958647692
#define HYD_RIPPLE_DEG_TO_RAD 0.01745329251994329577

void HYD_PressureRippleComp_Clear(HYD_PressureRippleCompState* state)
{
    if (state == NULL) return;
    memset(state, 0, sizeof(*state));
}

void HYD_PressureRippleComp_Execute(HYD_PressureRippleCompState* state,
                                    const HYD_PressureRippleCompInput* input,
                                    HYD_PressureRippleCompOutput* output)
{
    HYD_REAL speed;
    HYD_REAL phase;
    HYD_REAL cosK[HYD_PRESSURE_RIPPLE_HARMONICS];
    HYD_REAL sinK[HYD_PRESSURE_RIPPLE_HARMONICS];
    HYD_REAL flow = 0.0;
    HYD_BOOL feedbackValid;
    HYD_BOOL active;
    int k;

    if (output == NULL) return;
    memset(output, 0, sizeof(*output));
    if (state == NULL || input == NULL) return;

    if (input->order != state->order) {
        HYD_PressureRippleComp_Clear(state);
        state->order = input->order;
    }
    if (input->order == 0U || !(input->dt > 0.0)) {
        state->fade = 0.0;
        state->holdTime = 0.0;
        return;
    }

    feedbackValid = HYD_PumpFeedback_HasValid(input->pumpFeedback.validFlags,
                                              HYD_PUMP_FEEDBACK_VALID_RPM |
                                              HYD_PUMP_FEEDBACK_VALID_ANGLE) &&
                    isfinite(input->pumpFeedback.rpm) &&
                    isfinite(input->pumpFeedback.angleDeg);
    speed = (feedbackValid && input->pumpFeedback.rpm > 0.0) ? input->pumpFeedback.rpm : 0.0;

    if (input->referenceMoving || !feedbackValid) {
        state->holdTime = 0.0;
    } else if (state->holdTime < HYD_THRESH_PRESSURE_RIPPLE_HOLD_S) {
        state->holdTime += input->dt;
    }
    active = feedbackValid &&
             (2.0 * input->order * speed / 60.0) < HYD_THRESH_PRESSURE_RIPPLE_MAX_FREQ_HZ &&
             state->holdTime >= HYD_THRESH_PRESSURE_RIPPLE_HOLD_S;
    if (active) {
        state->fade += input->dt / HYD_THRESH_PRESSURE_RIPPLE_FADE_IN_S;
        if (state->fade > 1.0) state->fade = 1.0;
    } else {
        state->fade -= input->dt / HYD_THRESH_PRESSURE_RIPPLE_FADE_OUT_S;
        if (state->fade < 0.0) state->fade = 0.0;
    }

    /* N·θ reduced in degrees first keeps float precision at high orders. */
    phase = fmod((HYD_REAL)input->order * input->pumpFeedback.angleDeg, 360.0) *
            HYD_RIPPLE_DEG_TO_RAD;
    cosK[0] = cos(phase);
    sinK[0] = sin(phase);
    cosK[1] = cosK[0] * cosK[0] - sinK[0] * sinK[0];
    sinK[1] = 2.0 * sinK[0] * cosK[0];

    /* Learn only at full output weight: during fade-in the residual still
     * contains the faded-out part and the LMS would over-learn it. */
    output->learning = active && state->fade >= 1.0 && input->kqFlow > 0.0 &&
                       speed >= HYD_THRESH_PRESSURE_RIPPLE_MIN_RPM &&
                       fabs(input->error) < HYD_THRESH_PRESSURE_RIPPLE_MAX_ERROR &&
                       !state->lastClipped;
    if (output->learning) {
        HYD_REAL omega1 = input->order * HYD_RIPPLE_TWO_PI * speed / 60.0;
        HYD_REAL delayCos1 = cos(omega1 * HYD_THRESH_PRESSURE_RIPPLE_PATH_DELAY_S);
        HYD_REAL delaySin1 = sin(omega1 * HYD_THRESH_PRESSURE_RIPPLE_PATH_DELAY_S);
        HYD_REAL delayCos[HYD_PRESSURE_RIPPLE_HARMONICS];
        HYD_REAL delaySin[HYD_PRESSURE_RIPPLE_HARMONICS];
        HYD_REAL leak = (input->leakResistanceFlow > 0.0) ? 1.0 / input->leakResistanceFlow : 0.0;
        HYD_REAL step = HYD_THRESH_PRESSURE_RIPPLE_MU * input->dt * 2.0 * input->error / speed;

        delayCos[0] = delayCos1;
        delaySin[0] = delaySin1;
        delayCos[1] = delayCos1 * delayCos1 - delaySin1 * delaySin1;
        delaySin[1] = 2.0 * delaySin1 * delayCos1;

        for (k = 0; k < HYD_PRESSURE_RIPPLE_HARMONICS; ++k) {
            HYD_REAL omega = omega1 * (HYD_REAL)(k + 1);
            /* 1/Ph = (1/R_f + jω/Kq_f)·e^(jωTd) + Kp + Ki/(jω) */
            HYD_REAL invRe = leak * delayCos[k] - (omega / input->kqFlow) * delaySin[k] + input->kp;
            HYD_REAL invIm = leak * delaySin[k] + (omega / input->kqFlow) * delayCos[k] -
                             input->ki / omega;
            HYD_REAL magnitude;

            /* c += step · e^(-jkθ) / Ph */
            state->coeffRe[k] += step * (cosK[k] * invRe + sinK[k] * invIm);
            state->coeffIm[k] += step * (cosK[k] * invIm - sinK[k] * invRe);
            magnitude = sqrt(state->coeffRe[k] * state->coeffRe[k] +
                             state->coeffIm[k] * state->coeffIm[k]) * speed;
            if (magnitude > HYD_THRESH_PRESSURE_RIPPLE_MAX_FLOW) {
                HYD_REAL scale = HYD_THRESH_PRESSURE_RIPPLE_MAX_FLOW / magnitude;
                state->coeffRe[k] *= scale;
                state->coeffIm[k] *= scale;
            }
        }
    }

    for (k = 0; k < HYD_PRESSURE_RIPPLE_HARMONICS; ++k) {
        HYD_REAL magnitude = sqrt(state->coeffRe[k] * state->coeffRe[k] +
                                  state->coeffIm[k] * state->coeffIm[k]) * speed;
        HYD_REAL scale = (magnitude > HYD_THRESH_PRESSURE_RIPPLE_MAX_FLOW)
            ? HYD_THRESH_PRESSURE_RIPPLE_MAX_FLOW / magnitude : 1.0;

        output->amplitude[k] = magnitude * scale;
        flow += scale * speed * (state->coeffRe[k] * cosK[k] - state->coeffIm[k] * sinK[k]);
    }
    output->flow = state->fade * flow;
}
```

- [ ] **Step 4: 运行，确认通过**

新增了 `src/*.c`，必须重新 configure：

```bash
wsl.exe -d Ubuntu-22.04 bash -lc 'cd ~/project/hdy-motion-light && cmake --preset unixgcc > /dev/null && cmake --build --preset unixgcc 2>&1 | grep -E "error|warning: " | grep -v "rbf_pid.c\|state_reporter.c\|test_diagnostic_criteria.c"; ./out/build/unixgcc/test_pressure_ripple_comp'
```

Expected：没有新告警，输出：

```
order-12 amplitude: 0.929 bar -> 0.000 bar
amplitude 0.25-0.5 s: fresh 0.389 bar, re-entry 0.152 bar
Pressure ripple compensation tests passed.
```

- [ ] **Step 5: 全量回归**

```bash
wsl.exe -d Ubuntu-22.04 bash -lc 'cd ~/project/hdy-motion-light && ctest --test-dir out/build/unixgcc --output-on-failure 2>&1 | tail -3'
```

Expected：`100% tests passed, 0 tests failed out of 63`

- [ ] **Step 6: 提交**

```bash
wsl.exe -d Ubuntu-22.04 bash -lc 'cd ~/project/hdy-motion-light && git add include/pressure_ripple_comp.h src/pressure_ripple_comp.c tests/test_pressure_ripple_comp.c CMakeLists.txt && git commit -q -m "泵轴角同步纹波补偿模块：N/2N 阶 LMS（spec 2026-11-23 §4.5）" -m "带次级通道逆的归一化 LMS，系数按每 rpm 存储；静止 50 ms 后淡入，淡入完成后才学习；触限冻结学习，每阶幅值限 1.5 L/min。新增 T5。" -m "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>" && git log --oneline -1 && git status --short'
```

---

### Task 4: 压力控制器模型路径（T2/T3）

**Files:**
- Modify: `include/pressure_controller.h`、`src/pressure_controller.c`
- Modify: `tests/test_pressure_controller.c`
- Modify: `tests/test_negative_flow.c`、`tests/test_sprint_b_integration.c`
- Modify: `src/motion_control.c`（CRLF）：压力分支 `pressureInput` 整体清零，1 行

**Interfaces:**
- Consumes：
  - Task 2 的 `HYD_PRESSURE_REF_DELAY_SAMPLES`、`HYD_THRESH_PRESSURE_PROFILE_*`；
  - Task 3 的纹波模块；
  - `HYD_Debug_SetValue`（`debug_values.h`）。
- Produces（Task 6 使用）：
  - `HYD_PressureControllerInput` 新字段：
    - `targetPressureRate`（bar/s）；
    - `kqBarPerSecRpm`；
    - `pumpRippleOrder`（`HYD_UINT8`）；
    - `rippleComp`（`HYD_PressureRippleCompState*`，NULL 时不补偿）。
  - `HYD_PressureControllerOutput` 新字段：`referencePressureRate`、`rippleCompFlow`。
  - `HYD_PressureControllerState` 新字段：`referenceDelay[HYD_PRESSURE_REF_DELAY_SAMPLES]`、`referenceDelayIndex`、`referenceDelayPrimed`。
  - `HYD_BOOL HYD_PressureController_ResolveRateLimits(HYD_REAL kqBarPerSecRpm, HYD_REAL ksysBarPerRpm, HYD_REAL flowToPumpSpeedGain, HYD_REAL targetPressure, HYD_REAL outputMax, HYD_REAL reliefFlow, HYD_REAL* riseRate, HYD_REAL* fallRate);`
- 调试通道（仅模型路径写入）：

  | 通道 | 内容 |
  |---|---|
  | 0 | r |
  | 1 | ṙ |
  | 2 | Qff |
  | 3 | P + I（+ D + tracking） |
  | 4 | 纹波补偿流量 u |
  | 5 | N 阶幅值 |
  | 6 | 2N 阶幅值 |
  | 7 | 误差 e |

- [ ] **Step 1: 写失败测试**

`tests/test_pressure_controller.c` 中，在 `int main(void) {` 之前（Task 1 插入的 `test_legacy_path_is_bit_identical` 函数之后）插入：

```c
/* ===================== Model path (spec 2026-11-23 §4.3/§4.4, T2/T3) ===================== */

static HYD_MotionSegment make_model_segment(void) {
    HYD_MotionSegment segment = make_segment(HYD_PRESSURE_CONTROLLER_PI);
    segment.pressureKp = 0.5;
    segment.pressureKi = 4.0;
    segment.pressureIntegralLimit = 10.0;
    segment.pressureDeadband = 0.0;
    return segment;
}

static void fill_model_input(HYD_PressureControllerInput* input, HYD_TIME timestamp) {
    fill_input(input, timestamp);
    input->outputMin = -6.8;
    input->flowToPumpSpeedGain = 20.0;
    input->pumpSpeedLimit = 1700.0;
    input->systemGainBarPerRpm = 5.2;   /* R_f = 104 bar per L/min */
    input->kqBarPerSecRpm = 3.2;        /* Kq_f = 64 bar/s per L/min */
}

static void test_rate_limits_formula(void) {
    HYD_REAL rise = -1.0;
    HYD_REAL fall = -1.0;

    /* Kq_f = 64, R_f = 104, P = 104 bar -> hold flow 1 L/min. */
    assert(HYD_PressureController_ResolveRateLimits(3.2, 5.2, 20.0, 104.0, 20.0, -6.8, &rise, &fall));
    assert(fabs(rise - 0.6 * 64.0 * 19.0) < 1.0e-2);
    assert(fabs(fall - 0.6 * 64.0 * 7.8) < 1.0e-2);

    /* ksys = 0 drops the leak term. */
    assert(HYD_PressureController_ResolveRateLimits(3.2, 0.0, 20.0, 104.0, 20.0, -6.8, &rise, &fall));
    assert(fabs(rise - 0.6 * 64.0 * 20.0) < 1.0e-2);
    assert(fabs(fall - 0.6 * 64.0 * 6.8) < 1.0e-2);

    /* Hold flow above the cap: the 10 % floor keeps the reference moving. */
    assert(HYD_PressureController_ResolveRateLimits(3.2, 5.2, 20.0, 2080.0, 20.0, -6.8, &rise, &fall));
    assert(fabs(rise - 0.6 * 64.0 * 2.0) < 1.0e-2);

    /* The gain converts the rpm-domain parameters: G = 21.05 scales Kq_f. */
    assert(HYD_PressureController_ResolveRateLimits(3.2, 0.0, 21.05, 0.0, 20.0, -6.8, &rise, &fall));
    assert(fabs(rise - 0.6 * 3.2 * 21.05 * 20.0) < 1.0e-2);

    assert(!HYD_PressureController_ResolveRateLimits(0.0, 5.2, 20.0, 100.0, 20.0, -6.8, &rise, &fall));
    assert(rise == 0.0 && fall == 0.0);
    assert(!HYD_PressureController_ResolveRateLimits(3.2, 5.2, 0.0, 100.0, 20.0, -6.8, &rise, &fall));
    assert(!HYD_PressureController_ResolveRateLimits(3.2, 5.2, 20.0, 100.0, 0.0, -6.8, &rise, &fall));
}

static void test_model_feedforward_replaces_legacy_feedforward(void) {
    HYD_MotionSegment segment = make_model_segment();
    HYD_PressureControllerState state;
    HYD_PressureControllerInput input;
    HYD_PressureControllerOutput output;

    HYD_PressureController_InitState(&state, 104.0, 0.0, 0.0);
    fill_model_input(&input, 0.001);
    input.targetPressure = 104.0;
    input.measuredPressure = 104.0;
    input.targetPressureRate = 128.0;   /* 128 / 64 = 2 L/min */
    input.feedforwardFlow = 7.0;        /* ignored on the model path */
    HYD_PressureController_Execute(&segment, &state, &input, &output);

    assert(fabs(output.feedforwardFlow - 3.0) < 1.0e-5);
    assert(fabs(output.outputFlow - 3.0) < 1.0e-5);
    assert(fabs(output.referencePressureRate - 128.0) < 1.0e-5);

    /* ksys = 0: rate term only. */
    HYD_PressureController_InitState(&state, 104.0, 0.0, 0.0);
    input.systemGainBarPerRpm = 0.0;
    HYD_PressureController_Execute(&segment, &state, &input, &output);
    assert(fabs(output.feedforwardFlow - 2.0) < 1.0e-5);
}

static void test_model_error_uses_delayed_reference(void) {
    HYD_MotionSegment segment = make_model_segment();
    HYD_PressureControllerState state;
    HYD_PressureControllerInput input;
    HYD_PressureControllerOutput output;
    int k;

    HYD_PressureController_InitState(&state, 50.0, 0.0, 0.0);
    fill_model_input(&input, 0.0);
    input.measuredPressure = 50.0;
    for (k = 0; k < HYD_PRESSURE_REF_DELAY_SAMPLES + 3; ++k) {
        input.targetPressure = 50.0 + (HYD_REAL)k;
        input.timestamp = (HYD_TIME)(k + 1) * 0.001;
        HYD_PressureController_Execute(&segment, &state, &input, &output);
        if (k < HYD_PRESSURE_REF_DELAY_SAMPLES) {
            /* The line was filled with the first reference (50 bar). */
            assert(fabs(output.controlError) < 1.0e-5);
        } else {
            assert(fabs(output.controlError -
                        (HYD_REAL)(k - HYD_PRESSURE_REF_DELAY_SAMPLES)) < 1.0e-4);
        }
    }

    /* A new segment refills the line from its first reference. */
    HYD_PressureController_InitState(&state, 80.0, 0.0, 0.0);
    input.targetPressure = 90.0;
    input.measuredPressure = 80.0;
    HYD_PressureController_Execute(&segment, &state, &input, &output);
    assert(fabs(output.controlError - 10.0) < 1.0e-5);
}

static void test_model_tracking_is_bumpless(void) {
    HYD_MotionSegment segment = make_model_segment();
    HYD_PressureControllerState state;
    HYD_PressureControllerInput input;
    HYD_PressureControllerOutput output;

    HYD_PressureController_InitState(&state, 60.0, 4.0, 0.0);
    HYD_PressureController_RequestTracking(&state, 4.0);
    fill_model_input(&input, 0.001);
    input.targetPressure = 60.0;
    input.measuredPressure = 60.0;
    input.targetPressureRate = 0.0;
    HYD_PressureController_Execute(&segment, &state, &input, &output);

    assert(output.trackingApplied);
    assert(fabs(output.outputFlow - 4.0) < 1.0e-4);
    /* Integral absorbs everything the model feedforward does not explain. */
    assert(fabs(output.integralTerm - (4.0 - 60.0 / 104.0)) < 1.0e-4);
}

static void test_model_relief_holds_while_reference_falls(void) {
    HYD_MotionSegment segment = make_model_segment();
    HYD_PressureControllerState state;
    HYD_PressureControllerInput input;
    HYD_PressureControllerOutput output;

    HYD_PressureController_InitState(&state, 100.0, 1.0, 0.0);
    fill_model_input(&input, 0.001);
    input.targetPressure = 99.7;
    input.measuredPressure = 100.0;
    input.targetPressureRate = -300.0;
    HYD_PressureController_Execute(&segment, &state, &input, &output);
    assert(output.reliefActive);
    assert(output.outputFlow < 0.0);
    assert(output.effectiveOutputMin < 0.0);

    /* Pressure already at the reference, but the reference is still falling:
     * relief must stay active. */
    input.timestamp = 0.002;
    input.targetPressure = 99.4;
    input.measuredPressure = 99.4;
    HYD_PressureController_Execute(&segment, &state, &input, &output);
    assert(output.reliefActive);

    /* Reference stopped and pressure within 0.5 bar: relief exits. */
    input.timestamp = 0.003;
    input.targetPressureRate = 0.0;
    HYD_PressureController_Execute(&segment, &state, &input, &output);
    assert(!output.reliefActive);
    assert(output.outputFlow >= 0.0);
}

static void test_model_conditional_integration_at_limit(void) {
    HYD_MotionSegment segment = make_model_segment();
    HYD_PressureControllerState state;
    HYD_PressureControllerInput input;
    HYD_PressureControllerOutput output;
    int k;

    HYD_PressureController_InitState(&state, 0.0, 0.0, 0.0);
    fill_model_input(&input, 0.0);
    input.targetPressure = 100.0;
    input.measuredPressure = 20.0;     /* e = 80 -> P term 40 L/min > cap */
    for (k = 1; k <= 50; ++k) {
        input.timestamp = (HYD_TIME)k * 0.001;
        HYD_PressureController_Execute(&segment, &state, &input, &output);
    }
    assert(output.saturated);
    assert(fabs(output.outputFlow - 20.0) < 1.0e-5);
    assert(fabs(output.integralTerm) < 1.0e-6);
}

static void test_model_path_flags_clipped_output_for_ripple(void) {
    HYD_MotionSegment segment = make_model_segment();
    HYD_PressureControllerState state;
    HYD_PressureControllerInput input;
    HYD_PressureControllerOutput output;
    HYD_PressureRippleCompState ripple;

    HYD_PressureRippleComp_Clear(&ripple);
    HYD_PressureController_InitState(&state, 20.0, 0.0, 0.0);
    fill_model_input(&input, 0.001);
    input.pumpRippleOrder = 12U;
    input.rippleComp = &ripple;
    input.targetPressure = 100.0;
    input.measuredPressure = 20.0;
    HYD_PressureController_Execute(&segment, &state, &input, &output);
    assert(output.saturated);
    assert(ripple.lastClipped);
    assert(ripple.order == 12U);

    input.timestamp = 0.002;
    input.measuredPressure = 100.0;
    HYD_PressureController_Execute(&segment, &state, &input, &output);
    assert(!output.saturated);
    assert(!ripple.lastClipped);
}

static void test_rbf_ignores_kq(void) {
    HYD_MotionSegment segment = make_segment(HYD_PRESSURE_CONTROLLER_RBF_PID);
    HYD_PressureControllerState stateA;
    HYD_PressureControllerState stateB;
    HYD_PressureControllerInput input;
    HYD_PressureControllerOutput outputA;
    HYD_PressureControllerOutput outputB;
    int k;

    HYD_PressureController_InitState(&stateA, 0.0, 0.0, 0.0);
    HYD_PressureController_InitState(&stateB, 0.0, 0.0, 0.0);
    fill_input(&input, 0.0);
    for (k = 1; k <= 200; ++k) {
        input.timestamp = (HYD_TIME)k * 0.001;
        input.measuredPressure = (HYD_REAL)k * 0.4;
        input.kqBarPerSecRpm = 0.0;
        HYD_PressureController_Execute(&segment, &stateA, &input, &outputA);
        input.kqBarPerSecRpm = 3.2;
        HYD_PressureController_Execute(&segment, &stateB, &input, &outputB);
        assert(outputA.outputFlow == outputB.outputFlow);
    }
    assert(outputB.referencePressureRate == 0.0);
}
```

`tests/test_pressure_controller.c`（`main` 中，Task 1 之后），找到：

```c
    test_legacy_path_is_bit_identical();
```

替换为：

```c
    test_legacy_path_is_bit_identical();
    test_rate_limits_formula();
    test_model_feedforward_replaces_legacy_feedforward();
    test_model_error_uses_delayed_reference();
    test_model_tracking_is_bumpless();
    test_model_relief_holds_while_reference_falls();
    test_model_conditional_integration_at_limit();
    test_model_path_flags_clipped_output_for_ripple();
    test_rbf_ignores_kq();
```

- [ ] **Step 2: 运行，确认失败**

```bash
wsl.exe -d Ubuntu-22.04 bash -lc 'cd ~/project/hdy-motion-light && cmake --build out/build/unixgcc --target test_pressure_controller 2>&1 | grep -E "error" | head -5'
```

Expected：编译错误，提示 `HYD_PressureControllerInput` 没有 `kqBarPerSecRpm` 等成员、`HYD_PressureController_ResolveRateLimits` 隐式声明。

- [ ] **Step 3: 头文件**

`include/pressure_controller.h`，找到：

```c
#include "rbf_pid.h"
```

替换为：

```c
#include "rbf_pid.h"
#include "pressure_ripple_comp.h"
```

`include/pressure_controller.h`，找到：

```c
    HYD_PumpFeedback pumpFeedback;  /* shared system-level pump feedback snapshot */
} HYD_PressureControllerInput;
```

替换为：

```c
    HYD_PumpFeedback pumpFeedback;  /* shared system-level pump feedback snapshot */
    HYD_REAL targetPressureRate;    /* bar/s, profiled reference slope; 0 with the legacy ramp */
    HYD_REAL kqBarPerSecRpm;        /* bar/s per rpm; > 0 with a non-RBF strategy selects the model path */
    HYD_UINT8 pumpRippleOrder;      /* ripples per pump revolution; 0 disables ripple compensation */
    HYD_PressureRippleCompState* rippleComp; /* learned ripple, owned by the caller; NULL disables it */
} HYD_PressureControllerInput;
```

`include/pressure_controller.h`，找到：

```c
    HYD_PressureControllerType activeStrategy;
    RBF_PID_Handle rbfPid;
} HYD_PressureControllerState;
```

替换为：

```c
    HYD_PressureControllerType activeStrategy;
    RBF_PID_Handle rbfPid;
    HYD_REAL referenceDelay[HYD_PRESSURE_REF_DELAY_SAMPLES]; /* model path: reference history */
    HYD_UINT8 referenceDelayIndex;
    HYD_BOOL referenceDelayPrimed;
} HYD_PressureControllerState;
```

`include/pressure_controller.h`，找到：

```c
    HYD_BOOL reliefActive;
} HYD_PressureControllerOutput;
```

替换为：

```c
    HYD_BOOL reliefActive;
    HYD_REAL referencePressureRate; /* bar/s, model path only */
    HYD_REAL rippleCompFlow;        /* L/min, model path only */
} HYD_PressureControllerOutput;
```

`include/pressure_controller.h`（`HYD_PressureController_Execute` 声明前），找到：

```c
void HYD_PressureController_Execute(
```

替换为：

```c
/* Reference slope bounds for the profiled pressure ramp (spec 2026-11-23 §4.3):
 * rise = η·Kq_f·max(outputMax − P/R_f, r·outputMax),
 * fall = η·Kq_f·max(|reliefFlow| + P/R_f, r·outputMax),
 * Kq_f = kq·gain, R_f = ksys·gain (ksys = 0 drops the leak term).
 * Returns false (and zero bounds) when kq, gain or outputMax is not positive. */
HYD_BOOL HYD_PressureController_ResolveRateLimits(HYD_REAL kqBarPerSecRpm,
                                                  HYD_REAL ksysBarPerRpm,
                                                  HYD_REAL flowToPumpSpeedGain,
                                                  HYD_REAL targetPressure,
                                                  HYD_REAL outputMax,
                                                  HYD_REAL reliefFlow,
                                                  HYD_REAL* riseRate,
                                                  HYD_REAL* fallRate);
void HYD_PressureController_Execute(
```

- [ ] **Step 4: `src/pressure_controller.c`**

4a. include：

`src/pressure_controller.c`，找到：

```c
#include "pressure_controller.h"
#include <math.h>
```

替换为：

```c
#include "pressure_controller.h"
#include "debug_values.h"
#include <math.h>
```

4b. 跟踪积分使用调用方传入的前馈（旧路径传 `input->feedforwardFlow`，数值不变）：

`src/pressure_controller.c`，找到：

```c
static HYD_REAL HYD_ResolveTrackedIntegralOutput(const HYD_MotionSegment* segment,
                                                 const HYD_PressureControllerInput* input,
                                                 HYD_REAL proportionalTerm,
                                                 HYD_REAL derivativeTerm,
                                                 HYD_REAL trackedOutputFlow)
{
    HYD_REAL trackedIntegral;
    HYD_REAL integralLimit;

    if (input == NULL) return 0.0;

    trackedIntegral = trackedOutputFlow - input->feedforwardFlow
                    - proportionalTerm - derivativeTerm;
```

替换为：

```c
static HYD_REAL HYD_ResolveTrackedIntegralOutput(const HYD_MotionSegment* segment,
                                                 const HYD_PressureControllerInput* input,
                                                 HYD_REAL feedforwardFlow,
                                                 HYD_REAL proportionalTerm,
                                                 HYD_REAL derivativeTerm,
                                                 HYD_REAL trackedOutputFlow)
{
    HYD_REAL trackedIntegral;
    HYD_REAL integralLimit;

    if (input == NULL) return 0.0;

    trackedIntegral = trackedOutputFlow - feedforwardFlow
                    - proportionalTerm - derivativeTerm;
```

4c. 卸压状态机加 ṙ 条件（spec §4.4；旧路径 ṙ ≡ 0，逻辑等价）：

`src/pressure_controller.c`（`HYD_UpdateReliefState` 内），找到：

```c
    HYD_REAL pressureError;
    HYD_REAL targetDrop;

    if (state == NULL || input == NULL || !isfinite(input->targetPressure) ||
        !isfinite(input->measuredPressure)) {
        return;
    }

    pressureError = input->targetPressure - input->measuredPressure;
    targetDrop = state->targetPressureInitialized ?
                 state->previousTargetPressure - input->targetPressure : 0.0;

    if (!state->reliefActive) {
        if (input->measuredPressure > HYD_RELIEF_MIN_FEEDBACK_PRESSURE &&
            (pressureError <= -HYD_RELIEF_ENTER_ERROR ||
             targetDrop >= HYD_RELIEF_TARGET_DROP)) {
            state->reliefActive = HYD_TRUE;
        }
    } else if (input->measuredPressure <= HYD_RELIEF_MIN_FEEDBACK_PRESSURE ||
               pressureError >= -HYD_RELIEF_EXIT_ERROR) {
        state->reliefActive = HYD_FALSE;
    }
```

替换为：

```c
    HYD_REAL pressureError;
    HYD_REAL targetDrop;
    HYD_REAL targetRate;

    if (state == NULL || input == NULL || !isfinite(input->targetPressure) ||
        !isfinite(input->measuredPressure)) {
        return;
    }

    pressureError = input->targetPressure - input->measuredPressure;
    targetDrop = state->targetPressureInitialized ?
                 state->previousTargetPressure - input->targetPressure : 0.0;
    /* A profiled reference never drops 2 bar in one scan; a falling slope
     * is what admits reverse relief there. Legacy ramp reports 0. */
    targetRate = isfinite(input->targetPressureRate) ? input->targetPressureRate : 0.0;

    if (!state->reliefActive) {
        if (input->measuredPressure > HYD_RELIEF_MIN_FEEDBACK_PRESSURE &&
            (pressureError <= -HYD_RELIEF_ENTER_ERROR ||
             targetDrop >= HYD_RELIEF_TARGET_DROP ||
             targetRate < 0.0)) {
            state->reliefActive = HYD_TRUE;
        }
    } else if ((input->measuredPressure <= HYD_RELIEF_MIN_FEEDBACK_PRESSURE ||
                pressureError >= -HYD_RELIEF_EXIT_ERROR) &&
               targetRate >= 0.0) {
        state->reliefActive = HYD_FALSE;
    }
```

4d. 在 `HYD_SynchronizeRbfPidState` 函数结束的 `}` 之后、`void HYD_PressureController_ClearState(` 之前插入：

```c
/* ---- 模型路径（spec 2026-11-23 §4.3–§4.5）：Kq > 0 且策略不是 RBF ---- */

static HYD_BOOL HYD_IsModelPathActive(const HYD_PressureResolvedConfig* config,
                                      const HYD_PressureControllerInput* input)
{
    return config->strategy != HYD_PRESSURE_CONTROLLER_RBF_PID &&
           config->strategy != HYD_PRESSURE_CONTROLLER_RBF_PI &&
           isfinite(input->kqBarPerSecRpm) && input->kqBarPerSecRpm > 0.0 &&
           isfinite(input->flowToPumpSpeedGain) && input->flowToPumpSpeedGain > 0.0;
}

/* Qff = ṙ/Kq_f + r/R_f, Kq_f = Kq·G, R_f = ksys·G (ksys = 0 drops the leak term). */
static HYD_REAL HYD_ResolveModelFeedforward(const HYD_PressureControllerInput* input)
{
    HYD_REAL gain = input->flowToPumpSpeedGain;
    HYD_REAL feedforward = input->targetPressureRate / (input->kqBarPerSecRpm * gain);

    if (isfinite(input->systemGainBarPerRpm) && input->systemGainBarPerRpm > 0.0) {
        feedforward += input->targetPressure / (input->systemGainBarPerRpm * gain);
    }
    return feedforward;
}

/* Returns the reference from HYD_PRESSURE_REF_DELAY_SAMPLES scans ago. The
 * first call of a segment fills the line with the current reference. */
static HYD_REAL HYD_DelayReference(HYD_PressureControllerState* state, HYD_REAL reference)
{
    HYD_REAL delayed;
    unsigned int index;

    if (!state->referenceDelayPrimed) {
        for (index = 0U; index < (unsigned int)HYD_PRESSURE_REF_DELAY_SAMPLES; ++index) {
            state->referenceDelay[index] = reference;
        }
        state->referenceDelayIndex = 0U;
        state->referenceDelayPrimed = HYD_TRUE;
    }

    delayed = state->referenceDelay[state->referenceDelayIndex];
    state->referenceDelay[state->referenceDelayIndex] = reference;
    state->referenceDelayIndex = (HYD_UINT8)
        ((state->referenceDelayIndex + 1U) % (unsigned int)HYD_PRESSURE_REF_DELAY_SAMPLES);
    return delayed;
}

HYD_BOOL HYD_PressureController_ResolveRateLimits(HYD_REAL kqBarPerSecRpm,
                                                  HYD_REAL ksysBarPerRpm,
                                                  HYD_REAL flowToPumpSpeedGain,
                                                  HYD_REAL targetPressure,
                                                  HYD_REAL outputMax,
                                                  HYD_REAL reliefFlow,
                                                  HYD_REAL* riseRate,
                                                  HYD_REAL* fallRate)
{
    HYD_REAL kqFlow;
    HYD_REAL holdFlow = 0.0;
    HYD_REAL minimumFlow;
    HYD_REAL riseFlow;
    HYD_REAL fallFlow;

    if (riseRate == NULL || fallRate == NULL) return HYD_FALSE;
    *riseRate = 0.0;
    *fallRate = 0.0;
    if (!isfinite(kqBarPerSecRpm) || kqBarPerSecRpm <= 0.0 ||
        !isfinite(flowToPumpSpeedGain) || flowToPumpSpeedGain <= 0.0 ||
        !isfinite(outputMax) || outputMax <= 0.0) {
        return HYD_FALSE;
    }

    kqFlow = kqBarPerSecRpm * flowToPumpSpeedGain;
    if (isfinite(ksysBarPerRpm) && ksysBarPerRpm > 0.0 &&
        isfinite(targetPressure) && targetPressure > 0.0) {
        holdFlow = targetPressure / (ksysBarPerRpm * flowToPumpSpeedGain);
    }
    minimumFlow = HYD_THRESH_PRESSURE_PROFILE_MIN_FLOW_RATIO * outputMax;
    riseFlow = outputMax - holdFlow;
    fallFlow = (isfinite(reliefFlow) ? fabs(reliefFlow) : 0.0) + holdFlow;
    if (riseFlow < minimumFlow) riseFlow = minimumFlow;
    if (fallFlow < minimumFlow) fallFlow = minimumFlow;

    *riseRate = HYD_THRESH_PRESSURE_PROFILE_RATE_MARGIN * kqFlow * riseFlow;
    *fallRate = HYD_THRESH_PRESSURE_PROFILE_RATE_MARGIN * kqFlow * fallFlow;
    return HYD_TRUE;
}
```

4e. `HYD_PressureController_Execute` 开头：

`src/pressure_controller.c`，找到：

```c
    HYD_REAL trackedOutputFlow;
    HYD_BOOL trackingRequested;

    if (output == NULL) return;

    memset(output, 0, sizeof(*output));
    output->appliedStrategy = HYD_PRESSURE_CONTROLLER_NONE;

    if (segment == NULL || state == NULL || input == NULL) return;

    if (!state->initialized) {
        HYD_PressureController_InitState(state, input->measuredPressure,
            HYD_ClampReal(input->feedforwardFlow, input->outputMin, input->outputMax),
            input->timestamp);
    }

    HYD_UpdateReliefState(state, input);
    HYD_ResolvePressureControllerConfig(segment, state, input, &config);
```

替换为：

```c
    HYD_REAL trackedOutputFlow;
    HYD_BOOL trackingRequested;
    HYD_BOOL modelActive;
    HYD_REAL referencePressure;
    HYD_REAL feedforwardFlow;
    HYD_REAL rippleFlow = 0.0;
    HYD_PressureRippleCompOutput rippleOutput;

    if (output == NULL) return;

    memset(output, 0, sizeof(*output));
    memset(&rippleOutput, 0, sizeof(rippleOutput));
    output->appliedStrategy = HYD_PRESSURE_CONTROLLER_NONE;

    if (segment == NULL || state == NULL || input == NULL) return;

    if (!state->initialized) {
        HYD_PressureController_InitState(state, input->measuredPressure,
            HYD_ClampReal(input->feedforwardFlow, input->outputMin, input->outputMax),
            input->timestamp);
    }

    HYD_UpdateReliefState(state, input);
    HYD_ResolvePressureControllerConfig(segment, state, input, &config);

    /* Model path: the feedback compares against the reference delayed by the
     * loop delay, and the model feedforward replaces input->feedforwardFlow. */
    modelActive = HYD_IsModelPathActive(&config, input);
    referencePressure = modelActive ? HYD_DelayReference(state, input->targetPressure)
                                    : input->targetPressure;
    feedforwardFlow = modelActive ? HYD_ResolveModelFeedforward(input)
                                  : input->feedforwardFlow;
```

4f. 误差与输出记录：

`src/pressure_controller.c`，找到：

```c
    error = HYD_ApplyPressureDeadband(input->targetPressure - filteredPressure, config.deadband);
```

替换为：

```c
    error = HYD_ApplyPressureDeadband(referencePressure - filteredPressure, config.deadband);
```

`src/pressure_controller.c`，找到：

```c
    output->controlError = error;
    output->feedforwardFlow = input->feedforwardFlow;
```

替换为：

```c
    output->controlError = error;
    output->feedforwardFlow = feedforwardFlow;
```

RBF 分支里引用的仍是 `input->feedforwardFlow`、`input->targetPressure`。对 RBF 来说 `modelActive` 恒为假，两者与 `feedforwardFlow`、`referencePressure` 相等，所以 RBF 分支不改。

4g. 非 RBF 分支，从 `if (trackingRequested) {` 到 `output->saturated = ...;` 整段替换：

`src/pressure_controller.c`，找到：

```c
    if (trackingRequested) {
        output->trackingApplied = true;
        if (config.strategySpec->supportsIntegral) {
            integralTerm = HYD_ResolveTrackedIntegralOutput(segment, input,
                                                            proportionalTerm,
                                                            derivativeTerm,
                                                            trackedOutputFlow);
        } else {
            trackingTerm = trackedOutputFlow - input->feedforwardFlow
                         - proportionalTerm - derivativeTerm;
        }
    }

    if (config.strategySpec->supportsIntegral && config.dt > 0.0 && config.ki > 0.0) {
        integralCandidate = integralTerm + config.ki * error * config.dt;
        if (config.integralLimit > 0.0) {
            integralCandidate = HYD_ClampReal(integralCandidate, -config.integralLimit, config.integralLimit);
        }

        unsaturatedOutput = input->feedforwardFlow + proportionalTerm
                          + integralCandidate + derivativeTerm + trackingTerm;

        if ((unsaturatedOutput >= config.outputMin && unsaturatedOutput <= config.outputMax) ||
            (unsaturatedOutput > config.outputMax && error < 0.0) ||
            (unsaturatedOutput < config.outputMin && error > 0.0)) {
            integralTerm = integralCandidate;
        }
    }

    unsaturatedOutput = input->feedforwardFlow + proportionalTerm
                      + integralTerm + derivativeTerm + trackingTerm;
    outputFlow = HYD_ClampReal(unsaturatedOutput, config.outputMin, config.outputMax);

    /* Keep reverse flow semantics identical to the RBF path. */
    if (outputFlow < 0.0 &&
        (!state->reliefActive || HYD_ShouldClampReverseFlow(input))) {
        outputFlow = 0.0;
    }

    output->proportionalTerm = proportionalTerm;
    output->integralTerm = integralTerm;
    output->derivativeTerm = derivativeTerm;
    output->trackingTerm = trackingTerm;
    output->feedbackFlow = proportionalTerm + integralTerm + derivativeTerm + trackingTerm;
    output->unsaturatedOutputFlow = unsaturatedOutput;
    output->outputFlow = outputFlow;
    output->saturated = (outputFlow != unsaturatedOutput);
```

替换为：

```c
    if (trackingRequested) {
        output->trackingApplied = true;
        if (config.strategySpec->supportsIntegral) {
            integralTerm = HYD_ResolveTrackedIntegralOutput(segment, input,
                                                            feedforwardFlow,
                                                            proportionalTerm,
                                                            derivativeTerm,
                                                            trackedOutputFlow);
        } else {
            trackingTerm = trackedOutputFlow - feedforwardFlow
                         - proportionalTerm - derivativeTerm;
        }
    }

    if (modelActive && input->rippleComp != NULL) {
        HYD_PressureRippleCompInput rippleInput;

        memset(&rippleInput, 0, sizeof(rippleInput));
        rippleInput.order = input->pumpRippleOrder;
        rippleInput.pumpFeedback = input->pumpFeedback;
        rippleInput.referenceMoving = (input->targetPressureRate != 0.0);
        rippleInput.error = error;
        rippleInput.kqFlow = input->kqBarPerSecRpm * input->flowToPumpSpeedGain;
        rippleInput.leakResistanceFlow = (input->systemGainBarPerRpm > 0.0)
            ? input->systemGainBarPerRpm * input->flowToPumpSpeedGain : 0.0;
        rippleInput.kp = config.kp;
        rippleInput.ki = config.ki;
        rippleInput.dt = config.dt;
        HYD_PressureRippleComp_Execute(input->rippleComp, &rippleInput, &rippleOutput);
        rippleFlow = rippleOutput.flow;
    }

    if (config.strategySpec->supportsIntegral && config.dt > 0.0 && config.ki > 0.0) {
        integralCandidate = integralTerm + config.ki * error * config.dt;
        if (config.integralLimit > 0.0) {
            integralCandidate = HYD_ClampReal(integralCandidate, -config.integralLimit, config.integralLimit);
        }

        unsaturatedOutput = feedforwardFlow + proportionalTerm
                          + integralCandidate + derivativeTerm + trackingTerm + rippleFlow;

        if ((unsaturatedOutput >= config.outputMin && unsaturatedOutput <= config.outputMax) ||
            (unsaturatedOutput > config.outputMax && error < 0.0) ||
            (unsaturatedOutput < config.outputMin && error > 0.0)) {
            integralTerm = integralCandidate;
        }
    }

    unsaturatedOutput = feedforwardFlow + proportionalTerm
                      + integralTerm + derivativeTerm + trackingTerm + rippleFlow;
    outputFlow = HYD_ClampReal(unsaturatedOutput, config.outputMin, config.outputMax);

    /* Keep reverse flow semantics identical to the RBF path. */
    if (outputFlow < 0.0 &&
        (!state->reliefActive || HYD_ShouldClampReverseFlow(input))) {
        outputFlow = 0.0;
    }

    output->proportionalTerm = proportionalTerm;
    output->integralTerm = integralTerm;
    output->derivativeTerm = derivativeTerm;
    output->trackingTerm = trackingTerm;
    output->feedbackFlow = proportionalTerm + integralTerm + derivativeTerm + trackingTerm;
    output->unsaturatedOutputFlow = unsaturatedOutput;
    output->outputFlow = outputFlow;
    output->saturated = (outputFlow != unsaturatedOutput);

    if (modelActive) {
        if (input->rippleComp != NULL) {
            /* Freeze ripple learning next scan while the output is limited. */
            input->rippleComp->lastClipped = output->saturated;
        }
        output->referencePressureRate = input->targetPressureRate;
        output->rippleCompFlow = rippleFlow;
        HYD_Debug_SetValue(0U, (float)input->targetPressure);
        HYD_Debug_SetValue(1U, (float)input->targetPressureRate);
        HYD_Debug_SetValue(2U, (float)feedforwardFlow);
        HYD_Debug_SetValue(3U, (float)output->feedbackFlow);
        HYD_Debug_SetValue(4U, (float)rippleFlow);
        HYD_Debug_SetValue(5U, (float)rippleOutput.amplitude[0]);
        HYD_Debug_SetValue(6U, (float)rippleOutput.amplitude[1]);
        HYD_Debug_SetValue(7U, (float)error);
    }
```

4h. 未初始化输入的测试改为零初始化。否则 `kqBarPerSecRpm` 等新字段里的栈上随机值可能误开模型路径：
- `tests/test_negative_flow.c` 中 6 处 `    HYD_PressureControllerInput input;`（第 37、67、96、126、156、182 行）全部改为 `    HYD_PressureControllerInput input = {0};`；
- `tests/test_sprint_b_integration.c` 第 115 行同样改。

两个文件都是 LF。

```bash
wsl.exe -d Ubuntu-22.04 bash -lc 'cd ~/project/hdy-motion-light && sed -i "s/^    HYD_PressureControllerInput input;\$/    HYD_PressureControllerInput input = {0};/" tests/test_negative_flow.c tests/test_sprint_b_integration.c && grep -c "input = {0};" tests/test_negative_flow.c tests/test_sprint_b_integration.c'
```

Expected：`tests/test_negative_flow.c:6`、`tests/test_sprint_b_integration.c:1`。

如果 `$` 在传递中被吞掉，sed 会因为匹配到其他行而多改。此时用 `git diff` 检查，必要时改用编辑工具逐处修改。

4i. FB 压力分支的 `pressureInput` 先整体清零（`src/motion_control.c`，CRLF，约第 2564 行）。

这个局部变量原来逐字段赋值、没有清零，本任务新增的 `kqBarPerSecRpm`、`rippleComp` 等字段在 FB 里会是栈上随机值，可能误开模型路径，甚至解引用野指针。其余字段在 Task 6 接线。

`src/motion_control.c`，找到：

```c
        HYD_REAL pumpFlowLimit;

        pressureInput.outputMax = HYD_ResolvePressureOutputMax(fb, segment);
```

替换为：

```c
        HYD_REAL pumpFlowLimit;

        memset(&pressureInput, 0, sizeof(pressureInput));
        pressureInput.outputMax = HYD_ResolvePressureOutputMax(fb, segment);
```

- [ ] **Step 5: 运行，确认通过**

```bash
wsl.exe -d Ubuntu-22.04 bash -lc 'cd ~/project/hdy-motion-light && cmake --build --preset unixgcc 2>&1 | grep -E "error|warning: " | grep -v "rbf_pid.c\|state_reporter.c\|test_diagnostic_criteria.c"; ./out/build/unixgcc/test_pressure_controller && ctest --test-dir out/build/unixgcc --output-on-failure 2>&1 | tail -3'
```

Expected：没有新告警；`Pressure controller tests passed.`；`100% tests passed, 0 tests failed out of 63`。

Task 1 的逐位基准通过，说明参数 37 = 0 的路径没有变化。

- [ ] **Step 6: 提交**

```bash
wsl.exe -d Ubuntu-22.04 bash -lc 'cd ~/project/hdy-motion-light && git add include/pressure_controller.h src/pressure_controller.c src/motion_control.c tests/test_pressure_controller.c tests/test_negative_flow.c tests/test_sprint_b_integration.c && git commit -q -m "压力控制器模型路径：模型前馈、延迟参考、卸压斜率条件、纹波补偿接入（spec 2026-11-23 §4.3–§4.5）" -m "Kq > 0 且策略非 RBF 时：Qff = r'/Kq_f + r/R_f 替换原前馈，误差与 12 拍前的参考比较，参考下降期间允许并保持卸压，纹波补偿进入未饱和输出与条件积分，调试通道 0–7。新增 HYD_PressureController_ResolveRateLimits。Kq = 0 逐位不变。" -m "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>" && git log --oneline -1 && git status --short'
```

---

### Task 5: 参数 37/38/39（T7）

**Files:**
- Modify: `include/common_types.h`
- Modify: `src/motion_control.c`（CRLF）：`Init` 默认值、`ReadParameter`、`WriteParameter`
- Modify: `tests/test_parameter_access.c`

**Interfaces:**
- Consumes：Task 2 的 `HYD_MAX_PRESSURE_BRAKE_TIME_MS`、`HYD_MAX_PUMP_RIPPLE_ORDER`。
- Produces（Task 6、7 使用）：
  - 枚举 `HYD_PARAM_KQ_BAR_PER_S_RPM`（37）、`HYD_PARAM_PRESSURE_BRAKE_TIME`（38）、`HYD_PARAM_PUMP_RIPPLE_ORDER`（39）；
  - `fb->_params.kqBarPerSecRpm`、`fb->_params.pressureBrakeTimeMs`、`fb->_params.pumpRippleOrder`（`HYD_REAL`，整数值）。

- [ ] **Step 1: 写失败测试**

`tests/test_parameter_access.c` 中 `static void test_pressure_controller_type_validation(void) {` 之前插入：

```c
static void test_pressure_model_parameters_roundtrip_and_validation(void) {
    HYD_MotionControlFB fb;
    HYD_REAL value = -1.0f;
    HYD_BOOL ok;

    HYD_MotionControlFB_Init(&fb);

    ok = HYD_MotionControlFB_ReadParameter(&fb, HYD_PARAM_KQ_BAR_PER_S_RPM, &value);
    ASSERT_TRUE(ok, "Kq parameter should be readable");
    ASSERT_FLOAT_EQ(value, 0.0f, 0.0f, "Kq should default to zero (legacy path)");
    ok = HYD_MotionControlFB_ReadParameter(&fb, HYD_PARAM_PRESSURE_BRAKE_TIME, &value);
    ASSERT_TRUE(ok, "Pressure brake time should be readable");
    ASSERT_FLOAT_EQ(value, 0.0f, 0.0f, "Pressure brake time should default to zero");
    ok = HYD_MotionControlFB_ReadParameter(&fb, HYD_PARAM_PUMP_RIPPLE_ORDER, &value);
    ASSERT_TRUE(ok, "Pump ripple order should be readable");
    ASSERT_FLOAT_EQ(value, 0.0f, 0.0f, "Pump ripple order should default to zero");

    ASSERT_EQ((int)HYD_PARAM_KQ_BAR_PER_S_RPM, 37, "Kq is parameter 37");
    ASSERT_EQ((int)HYD_PARAM_PRESSURE_BRAKE_TIME, 38, "Brake time is parameter 38");
    ASSERT_EQ((int)HYD_PARAM_PUMP_RIPPLE_ORDER, 39, "Ripple order is parameter 39");

    ok = HYD_MotionControlFB_WriteParameter(&fb, HYD_PARAM_KQ_BAR_PER_S_RPM, 2.9f);
    ASSERT_TRUE(ok, "Positive Kq should be writable");
    ok = HYD_MotionControlFB_ReadParameter(&fb, HYD_PARAM_KQ_BAR_PER_S_RPM, &value);
    ASSERT_FLOAT_EQ(value, 2.9f, 0.0001f, "Kq should round-trip in bar/s/rpm");
    ASSERT_TRUE(!HYD_MotionControlFB_WriteParameter(&fb, HYD_PARAM_KQ_BAR_PER_S_RPM, -1.0f),
                "Negative Kq should be rejected");
    ASSERT_TRUE(!HYD_MotionControlFB_WriteParameter(&fb, HYD_PARAM_KQ_BAR_PER_S_RPM, NAN),
                "Non-finite Kq should be rejected");
    ASSERT_TRUE(HYD_MotionControlFB_WriteParameter(&fb, HYD_PARAM_KQ_BAR_PER_S_RPM, 0.0f),
                "Zero Kq should select the legacy path");

    ok = HYD_MotionControlFB_WriteParameter(&fb, HYD_PARAM_PRESSURE_BRAKE_TIME, 150.0f);
    ASSERT_TRUE(ok, "A 150 ms brake time should be writable");
    ok = HYD_MotionControlFB_ReadParameter(&fb, HYD_PARAM_PRESSURE_BRAKE_TIME, &value);
    ASSERT_FLOAT_EQ(value, 150.0f, 0.0001f, "Brake time should round-trip in ms");
    ASSERT_TRUE(HYD_MotionControlFB_WriteParameter(&fb, HYD_PARAM_PRESSURE_BRAKE_TIME, 2000.0f),
                "The 2000 ms upper bound should be accepted");
    ASSERT_TRUE(!HYD_MotionControlFB_WriteParameter(&fb, HYD_PARAM_PRESSURE_BRAKE_TIME, 2000.5f),
                "Brake times above 2000 ms should be rejected");
    ASSERT_TRUE(!HYD_MotionControlFB_WriteParameter(&fb, HYD_PARAM_PRESSURE_BRAKE_TIME, -1.0f),
                "Negative brake time should be rejected");

    ok = HYD_MotionControlFB_WriteParameter(&fb, HYD_PARAM_PUMP_RIPPLE_ORDER, 12.0f);
    ASSERT_TRUE(ok, "Ripple order 12 should be writable");
    ok = HYD_MotionControlFB_ReadParameter(&fb, HYD_PARAM_PUMP_RIPPLE_ORDER, &value);
    ASSERT_FLOAT_EQ(value, 12.0f, 0.0f, "Ripple order should round-trip");
    ASSERT_TRUE(!HYD_MotionControlFB_WriteParameter(&fb, HYD_PARAM_PUMP_RIPPLE_ORDER, 12.5f),
                "Non-integer ripple order should be rejected");
    ASSERT_TRUE(!HYD_MotionControlFB_WriteParameter(&fb, HYD_PARAM_PUMP_RIPPLE_ORDER, 65.0f),
                "Ripple order above 64 should be rejected");
    ASSERT_TRUE(!HYD_MotionControlFB_WriteParameter(&fb, HYD_PARAM_PUMP_RIPPLE_ORDER, -1.0f),
                "Negative ripple order should be rejected");
    ok = HYD_MotionControlFB_ReadParameter(&fb, HYD_PARAM_PUMP_RIPPLE_ORDER, &value);
    ASSERT_FLOAT_EQ(value, 12.0f, 0.0f, "Rejected writes keep the last accepted order");
}
```

`tests/test_parameter_access.c`（`main` 中），找到：

```c
    test_hydraulic_response_delay_roundtrip_and_validation();
```

替换为：

```c
    test_hydraulic_response_delay_roundtrip_and_validation();
    test_pressure_model_parameters_roundtrip_and_validation();
```

- [ ] **Step 2: 运行，确认失败**

```bash
wsl.exe -d Ubuntu-22.04 bash -lc 'cd ~/project/hdy-motion-light && cmake --build out/build/unixgcc --target test_parameter_access 2>&1 | grep -E "error" | head -3'
```

Expected：`'HYD_PARAM_KQ_BAR_PER_S_RPM' undeclared` 等编译错误。

- [ ] **Step 3: 实现**

`include/common_types.h`，找到：

```c
    HYD_PARAM_ZERO_END_SETTLE_VELOCITY, /* mm/s; 0 keeps the legacy zero-end settle gate */
    HYD_PARAM_COUNT
```

替换为：

```c
    HYD_PARAM_ZERO_END_SETTLE_VELOCITY, /* mm/s; 0 keeps the legacy zero-end settle gate */
    HYD_PARAM_KQ_BAR_PER_S_RPM,         /* bar/s per rpm; 0 keeps the legacy pressure path */
    HYD_PARAM_PRESSURE_BRAKE_TIME,      /* ms; 0 uses HYD_THRESH_PRESSURE_PROFILE_BRAKE_TIME_S */
    HYD_PARAM_PUMP_RIPPLE_ORDER,        /* ripples per pump revolution (integer); 0 disables compensation */
    HYD_PARAM_COUNT
```

`include/common_types.h`，找到：

```c
    HYD_REAL zeroEndSettleVelocity;      /* mm/s; zero-end settle gate, 0 = legacy */
} HYD_MotionFBParams;
```

替换为：

```c
    HYD_REAL zeroEndSettleVelocity;      /* mm/s; zero-end settle gate, 0 = legacy */
    HYD_REAL kqBarPerSecRpm;             /* bar/s per rpm; 0 keeps the legacy pressure path */
    HYD_REAL pressureBrakeTimeMs;        /* ms; 0 uses the built-in brake time */
    HYD_REAL pumpRippleOrder;            /* ripples per pump revolution (integer); 0 disables */
} HYD_MotionFBParams;
```

`src/motion_control.c` 是 CRLF。下面三处用编辑工具修改，或者按“与原文件相同的 CRLF”插入：

`src/motion_control.c`（`HYD_MotionControlFB_Init`，约第 4063 行），找到：

```c
    fb->_params.zeroEndSettleVelocity = 0.0f;
```

替换为：

```c
    fb->_params.zeroEndSettleVelocity = 0.0f;
    fb->_params.kqBarPerSecRpm = 0.0f;
    fb->_params.pressureBrakeTimeMs = 0.0f;
    fb->_params.pumpRippleOrder = 0.0f;
```

`src/motion_control.c`（`HYD_MotionControlFB_ReadParameter`，约第 4689 行），找到：

```c
        case HYD_PARAM_ZERO_END_SETTLE_VELOCITY:       *value = fb->_params.zeroEndSettleVelocity; break;
```

替换为：

```c
        case HYD_PARAM_ZERO_END_SETTLE_VELOCITY:       *value = fb->_params.zeroEndSettleVelocity; break;
        case HYD_PARAM_KQ_BAR_PER_S_RPM:               *value = fb->_params.kqBarPerSecRpm; break;
        case HYD_PARAM_PRESSURE_BRAKE_TIME:            *value = fb->_params.pressureBrakeTimeMs; break;
        case HYD_PARAM_PUMP_RIPPLE_ORDER:              *value = fb->_params.pumpRippleOrder; break;
```

`src/motion_control.c`（`HYD_MotionControlFB_WriteParameter`，约第 4771 行），找到：

```c
        case HYD_PARAM_ZERO_END_SETTLE_VELOCITY:
            if (!isfinite(value) || value < 0.0) return false;
            fb->_params.zeroEndSettleVelocity = value;
            break;
```

替换为：

```c
        case HYD_PARAM_ZERO_END_SETTLE_VELOCITY:
            if (!isfinite(value) || value < 0.0) return false;
            fb->_params.zeroEndSettleVelocity = value;
            break;
        case HYD_PARAM_KQ_BAR_PER_S_RPM:
            if (!isfinite(value) || value < 0.0) return false;
            fb->_params.kqBarPerSecRpm = value;
            break;
        case HYD_PARAM_PRESSURE_BRAKE_TIME:
            if (!isfinite(value) || value < 0.0 ||
                value > HYD_MAX_PRESSURE_BRAKE_TIME_MS) return false;
            fb->_params.pressureBrakeTimeMs = value;
            break;
        case HYD_PARAM_PUMP_RIPPLE_ORDER:
            if (!isfinite(value) || value < 0.0 ||
                value > (HYD_REAL)HYD_MAX_PUMP_RIPPLE_ORDER ||
                (HYD_REAL)(int)value != value) return false;
            fb->_params.pumpRippleOrder = value;
            break;
```

- [ ] **Step 4: 运行，确认通过**

```bash
wsl.exe -d Ubuntu-22.04 bash -lc 'cd ~/project/hdy-motion-light && cmake --build --preset unixgcc 2>&1 | grep -E "error|warning: " | grep -v "rbf_pid.c\|state_reporter.c\|test_diagnostic_criteria.c"; ./out/build/unixgcc/test_parameter_access | tail -1 && ctest --test-dir out/build/unixgcc --output-on-failure 2>&1 | tail -3 && git diff --stat src/motion_control.c'
```

Expected：
- `Parameter access tests: 110/110 passed`（原 85 个断言，新增 25 个）；
- `100% tests passed, 0 tests failed out of 63`；
- `src/motion_control.c` 约 21 行新增、没有删除。如果显示数千行改动，说明换行符被转换了，先恢复。

- [ ] **Step 5: 提交**

```bash
wsl.exe -d Ubuntu-22.04 bash -lc 'cd ~/project/hdy-motion-light && git add include/common_types.h src/motion_control.c tests/test_parameter_access.c && git commit -q -m "参数 37/38/39：Kq、压力制动时间、泵纹波阶次（spec 2026-11-23 §4.6）" -m "转速域 Kq（bar/s/rpm，0 = 旧路径）、制动时间（ms，0–2000，0 = 120 ms）、泵每转纹波次数（整数 0–64，0 = 关）。新增 T7。" -m "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>" && git log --oneline -1 && git status --short'
```

---

### Task 6: FB 接线与闭环回归（T6/T8）

**Files:**
- Modify: `include/motion_control.h`
- Modify: `src/motion_control.c`（CRLF）：`HYD_ExecuteActiveSegmentControl` 之前新增 2 个 static 函数；在该函数内改轨迹调用和压力分支
- Modify: `src/safety_state_manager.c`
- Create: `tests/test_pressure_2dof_closed_loop.c`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes：
  - Task 2 的 `HYD_RampController_ExecuteProfiledWithDt`、`HYD_RampProfileLimits`；
  - Task 4 的 `HYD_PressureController_ResolveRateLimits` 和新的输入字段；
  - Task 5 的参数字段；
  - Task 3 的 `HYD_PressureRippleComp_Clear`。
- Produces：
  - `HYD_MotionControlFB._pressureRippleComp`；
  - 文件内私有函数 `HYD_GetPressureReliefFlow`、`HYD_ResolvePressureProfileLimits`。

接线规则（spec §4.6）：
- **启用条件**：段为压力闭环、参数 37 > 0、策略不是 RBF 时，用 profiled 轨迹。
- **速率上限**：每拍用 `segment->targetPressure`、`HYD_ResolvePressureOutputMax`、卸压流量 `−0.08 × 泵流量上限` 计算；只在目标变化时被轨迹锁存。
- **制动时间**：参数 38 > 0 时取参数 38，否则取 §14B 默认值。
- **时间步长**：仿真固定步长用 `deltaTime`，否则用 `controllerTime − lastTimestamp`，与旧 `Execute` 一致。
- **控制器输入**：`pressureInput` 已在 Task 4 整体清零。本任务填入 `targetPressureRate`、`kqBarPerSecRpm`、`pumpRippleOrder`、`rippleComp`；参数 37 = 0 时控制器不进入模型路径，这些字段不起作用。
- **纹波系数清零时机**：`ApplyDisabledState`、`ApplyFaultHold`、`EnterFaultStop`，以及 `Init`/`SoftReset` 的整体 memset。段切换、停止、完成都不清零。

- [ ] **Step 1: 写失败测试**

新建 `tests/test_pressure_2dof_closed_loop.c`：

```c
/* Closed-loop regression for the 2-DOF pressure loop (spec 2026-11-23 §5, T6/T8).
 *
 * Runs the real HYD_MotionControlFB (PressureHandle-style direct segments,
 * live target updates) against the valve-closed plant identified from
 * docs/data_deg/hor1121: dP/dt = Kq·(Q − P/R), Kq = 64 (+0.4/bar above
 * 100 bar) bar/s per L/min, R = 104 bar per L/min, servo dead time, MA3 on
 * the feedback, pump-side order 12/24 flow ripple. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "debug_values.h"
#include "motion_control.h"

#define DT 0.001
#define TWO_PI 6.283185307179586
#define GAIN 20.0              /* rpm per L/min (FLOW_TO_PUMPSPEED = 20) */
#define MAX_SERVO_DELAY 8
#define SEGMENT_COUNT 5

typedef struct {
    double servoDelay;         /* scans */
    double kqScale;            /* true Kq / identified Kq (3.2 bar/s/rpm); configured is 2.9 */
    HYD_REAL rippleOrder;      /* parameter 39 */
} PlantCase;

typedef struct {
    double overshoot[SEGMENT_COUNT];
    double holdPeakToPeak[SEGMENT_COUNT];
    int t98[SEGMENT_COUNT];
    double maxDebugAbs[HYD_DEBUG_VALUE_COUNT];
} LoopMetrics;

static const double kTargets[SEGMENT_COUNT + 1] = {0.0, 100.0, 140.0, 80.0, 120.0, 50.0};
static const int kSegmentSteps = 3000;
static const int kLeadSteps = 200;

static HYD_MotionSegment make_hold_segment(double target) {
    HYD_MotionSegment segment;
    memset(&segment, 0, sizeof(segment));
    segment.segmentTag = HYD_SEGMENT_TYPE_HOLDING;
    segment.segmentType = HYD_SEGMENT_TYPE_HOLDING;
    segment.mode = HYD_MODE_PRESSURE_CLOSED_LOOP;
    segment.endCondition = HYD_END_MANUAL;
    segment.direction = HYD_DIRECTION_HOLD;
    segment.targetPressure = (HYD_REAL)target;
    segment.maxFlow = 20.0f;
    segment.pressureRampRate = 20.0f;
    segment.pressureCeiling = MAX_PRESSURE;
    segment.pressureController = HYD_PRESSURE_CONTROLLER_PI;
    /* Kp = 2π·5/(Kq·G), Ki = Kp·2π·5/4 with Kq = 2.9 bar/s/rpm (3.2 × 0.9). */
    segment.pressureKp = (HYD_REAL)(TWO_PI * 5.0 / (2.9 * GAIN));
    segment.pressureKi = (HYD_REAL)(TWO_PI * 5.0 / (2.9 * GAIN) * TWO_PI * 5.0 / 4.0);
    segment.pressureIntegralLimit = 10.0f;
    segment.pressureDeadband = 0.005f;
    return segment;
}

static void configure_fb(HYD_MotionControlFB* fb, HYD_REAL rippleOrder) {
    HYD_MotionControlFB_Init(fb);
    fb->USE_RECIPE = false;
    assert(HYD_MotionControlFB_WriteParameter(fb, HYD_PARAM_FLOW_TO_PUMP_SPEED_GAIN, (HYD_REAL)GAIN));
    assert(HYD_MotionControlFB_WriteParameter(fb, HYD_PARAM_PUMP_SPEED_LIMIT, 1700.0f));
    assert(HYD_MotionControlFB_WriteParameter(fb, HYD_PARAM_KSYS_BAR_PER_RPM, 5.2f));
    assert(HYD_MotionControlFB_WriteParameter(fb, HYD_PARAM_KQ_BAR_PER_S_RPM, 2.9f));
    assert(HYD_MotionControlFB_WriteParameter(fb, HYD_PARAM_PUMP_RIPPLE_ORDER, rippleOrder));
}

static void run_sequence(const PlantCase* plantCase, LoopMetrics* metrics) {
    static HYD_MotionControlFB fb;
    HYD_MotionSegment segment;
    double pressure = 0.0;
    double angle = 0.0;
    double history[3] = {0.0, 0.0, 0.0};
    double measured = 0.0;
    double speedLine[MAX_SERVO_DELAY];
    int delay = (int)plantCase->servoDelay;
    int lineIndex = 0;
    int seg;
    int k;
    int step = 0;

    memset(metrics, 0, sizeof(*metrics));
    memset(speedLine, 0, sizeof(speedLine));
    assert(delay >= 1 && delay <= MAX_SERVO_DELAY);
    HYD_Debug_Reset();
    configure_fb(&fb, plantCase->rippleOrder);

    segment = make_hold_segment(0.0);
    assert(HYD_MotionControlFB_LoadDirectSegment(&fb, &segment));
    fb.AXIS_REF.timestamp = 0.0f;
    assert(HYD_MotionControlFB_StartSegment(&fb, 0U, 0.0f));

    for (seg = 0; seg <= SEGMENT_COUNT; ++seg) {
        int steps = (seg == 0) ? kLeadSteps : kSegmentSteps;
        double target = kTargets[seg];
        double previous = (seg == 0) ? 0.0 : kTargets[seg - 1];
        double maxP = -1.0e9;
        double minP = 1.0e9;
        double holdMax = -1.0e9;
        double holdMin = 1.0e9;
        int t98 = -1;

        if (seg > 0) {
            HYD_LiveUpdateRequest request;
            memset(&request, 0, sizeof(request));
            request.flags = HYD_LIVE_UPDATE_TARGET_PRESSURE;
            request.ownerKind = fb._directOwnerKind;
            request.ownerTicket = fb._directOwnerTicket;
            request.targetPressure = (HYD_REAL)target;
            assert(HYD_MotionControlFB_ApplyLiveUpdate(&fb, &request));
        }

        for (k = 0; k < steps; ++k) {
            double speedCommand;
            double appliedSpeed;
            double kq;
            double w12;
            double ripple;
            int i;

            ++step;
            fb.AXIS_REF.timestamp = (HYD_TIME)(step * DT);
            fb.AXIS_REF.pressure = (HYD_REAL)measured;
            HYD_MotionControl_SetPumpFeedback(&(HYD_PumpFeedback){
                (HYD_REAL)speedLine[lineIndex],
                (HYD_REAL)(fmod(angle, TWO_PI) * 360.0 / TWO_PI),
                0.0f, (HYD_TIME)(step * DT),
                HYD_PUMP_FEEDBACK_VALID_RPM | HYD_PUMP_FEEDBACK_VALID_ANGLE});
            HYD_MotionControlFB_Cycle(&fb);
            assert(!fb.STATE.faultActive);
            speedCommand = fb.PUMP_SPEED;

            appliedSpeed = speedLine[lineIndex];
            speedLine[lineIndex] = speedCommand;
            lineIndex = (lineIndex + 1) % delay;

            angle += appliedSpeed / 60.0 * TWO_PI * DT;
            kq = plantCase->kqScale * (64.0 + 0.4 * fmax(pressure - 100.0, 0.0));
            w12 = 12.0 * fabs(appliedSpeed) / 60.0 * TWO_PI;
            ripple = 1.2 * w12 / 64.0 * sin(12.0 * angle + 0.7) +
                     0.36 * w12 / 64.0 * sin(24.0 * angle + 1.9);
            pressure += kq * (appliedSpeed / GAIN + ripple - pressure / 104.0) * DT;
            if (pressure < 0.0) pressure = 0.0;
            history[0] = history[1];
            history[1] = history[2];
            history[2] = pressure;
            measured = (history[0] + history[1] + history[2]) / 3.0;

            if (seg > 0) {
                if (measured > maxP) maxP = measured;
                if (measured < minP) minP = measured;
                if (t98 < 0 && ((target >= previous && measured >= previous + 0.98 * (target - previous)) ||
                                (target < previous && measured <= previous + 0.98 * (target - previous)))) {
                    t98 = k;
                }
                if (k >= steps - 1000) {
                    if (measured > holdMax) holdMax = measured;
                    if (measured < holdMin) holdMin = measured;
                }
            }
            for (i = 0; i < (int)HYD_DEBUG_VALUE_COUNT; ++i) {
                double v = fabs((double)HYD_Debug_GetValue((HYD_UINT8)i));
                if (v > metrics->maxDebugAbs[i]) metrics->maxDebugAbs[i] = v;
            }
        }

        if (seg > 0) {
            metrics->overshoot[seg - 1] = (target >= previous) ? maxP - target : target - minP;
            metrics->holdPeakToPeak[seg - 1] = holdMax - holdMin;
            metrics->t98[seg - 1] = t98;
        }
    }
}

static void print_metrics(const char* name, const LoopMetrics* m) {
    int i;
    printf("%-26s", name);
    for (i = 0; i < SEGMENT_COUNT; ++i) {
        printf(" | %3.0f->%3.0f ov %.2f t98 %3d pp %.2f", kTargets[i], kTargets[i + 1],
               m->overshoot[i], m->t98[i], m->holdPeakToPeak[i]);
    }
    printf("\n");
}

static double max_of(const double* values) {
    double m = values[0];
    int i;
    for (i = 1; i < SEGMENT_COUNT; ++i) if (values[i] > m) m = values[i];
    return m;
}

static void test_nominal_without_ripple_compensation(void) {
    PlantCase plantCase = {5, 1.0, 0.0f};
    LoopMetrics m;
    run_sequence(&plantCase, &m);
    print_metrics("B1 nominal", &m);
    assert(max_of(m.overshoot) <= 1.5);
    assert(max_of(m.holdPeakToPeak) <= 3.0);
}

static void test_nominal_with_ripple_compensation(void) {
    PlantCase plantCase = {5, 1.0, 12.0f};
    LoopMetrics m;
    int i;
    run_sequence(&plantCase, &m);
    print_metrics("B2 nominal", &m);
    assert(max_of(m.overshoot) <= 1.5);
    assert(max_of(m.holdPeakToPeak) <= 1.0);
    for (i = 0; i < SEGMENT_COUNT; ++i) assert(m.t98[i] > 0);
    /* T8: every debug channel carries a value on the model path. */
    for (i = 0; i < (int)HYD_DEBUG_VALUE_COUNT; ++i) assert(m.maxDebugAbs[i] > 0.0);
}

static void test_robust_to_servo_delay_and_lower_configured_kq(void) {
    const PlantCase cases[] = {
        /* true Kq = configured (no 0.9 margin) and true Kq = 1.5 x configured */
        {3, 2.9 / 3.2, 12.0f}, {7, 2.9 / 3.2, 12.0f},
        {3, 1.5 * 2.9 / 3.2, 12.0f}, {7, 1.5 * 2.9 / 3.2, 12.0f},
    };
    size_t c;
    for (c = 0; c < sizeof(cases) / sizeof(cases[0]); ++c) {
        LoopMetrics m;
        char name[40];
        run_sequence(&cases[c], &m);
        snprintf(name, sizeof(name), "B2 servo %d true/cfg %.2f", (int)cases[c].servoDelay,
                 cases[c].kqScale * 3.2 / 2.9);
        print_metrics(name, &m);
        assert(max_of(m.overshoot) <= 1.5);
    }
}

int main(void) {
    test_nominal_without_ripple_compensation();
    test_nominal_with_ripple_compensation();
    test_robust_to_servo_delay_and_lower_configured_kq();
    puts("2-DOF pressure closed-loop tests passed.");
    return 0;
}
```

`CMakeLists.txt`，找到：

```cmake
add_executable(test_pressure_ripple_comp tests/test_pressure_ripple_comp.c)
target_link_libraries(test_pressure_ripple_comp PRIVATE HydroMotionLib)
```

替换为：

```cmake
add_executable(test_pressure_ripple_comp tests/test_pressure_ripple_comp.c)
target_link_libraries(test_pressure_ripple_comp PRIVATE HydroMotionLib)

add_executable(test_pressure_2dof_closed_loop tests/test_pressure_2dof_closed_loop.c)
target_link_libraries(test_pressure_2dof_closed_loop PRIVATE HydroMotionLib)
```

`CMakeLists.txt`，找到：

```cmake
add_test(NAME test_pressure_ripple_comp COMMAND test_pressure_ripple_comp)
```

替换为：

```cmake
add_test(NAME test_pressure_ripple_comp COMMAND test_pressure_ripple_comp)
add_test(NAME test_pressure_2dof_closed_loop COMMAND test_pressure_2dof_closed_loop)
```

- [ ] **Step 2: 运行，确认失败**

```bash
wsl.exe -d Ubuntu-22.04 bash -lc 'cd ~/project/hdy-motion-light && cmake --preset unixgcc > /dev/null && cmake --build out/build/unixgcc --target test_pressure_2dof_closed_loop 2>&1 | grep -E "error" | head -3; ./out/build/unixgcc/test_pressure_2dof_closed_loop; echo exit=$?'
```

Expected：编译通过；运行时输出 `test_nominal_without_ripple_compensation: Assertion 'max_of(m.overshoot) <= 1.5' failed.`，`exit=134`。

原因：这时 FB 还在走 20 ms 时间斜坡加旧前馈，超调远大于 1.5 bar（2026-11-23 临时副本实测）。如果失败在别处，或者测试通过了，先停下检查接线状态，不要继续。

- [ ] **Step 3: FB 字段与清零**

`include/motion_control.h`（约第 368 行），找到：

```c
    HYD_PressureControllerState _pressureController;
```

替换为：

```c
    HYD_PressureControllerState _pressureController;
    HYD_PressureRippleCompState _pressureRippleComp; /* learned pump ripple; kept across segments */
```

`src/safety_state_manager.c` 三处，各在 `HYD_SafetyStateManager_ResetRuntimeActuation(fb);` 之后加一行：

`src/safety_state_manager.c`（`ApplyDisabledState`），找到：

```c
    HYD_SafetyStateManager_ResetRuntimeActuation(fb);
    fb->_lastCommandedFlow = 0.0;
    HYD_StateReporter_ResetTransitionFlags(fb);
```

替换为：

```c
    HYD_SafetyStateManager_ResetRuntimeActuation(fb);
    HYD_PressureRippleComp_Clear(&fb->_pressureRippleComp);
    fb->_lastCommandedFlow = 0.0;
    HYD_StateReporter_ResetTransitionFlags(fb);
```

`src/safety_state_manager.c`（`ApplyFaultHold`），找到：

```c
    HYD_SafetyStateManager_ResetRuntimeActuation(fb);
    fb->_lastCommandedFlow = 0.0;
    HYD_StateReporter_ApplySafeOutputs(fb);
    HYD_StateReporter_SetProtectionAction(fb, HYD_PROTECTION_ACTION_STOP);
```

替换为：

```c
    HYD_SafetyStateManager_ResetRuntimeActuation(fb);
    HYD_PressureRippleComp_Clear(&fb->_pressureRippleComp);
    fb->_lastCommandedFlow = 0.0;
    HYD_StateReporter_ApplySafeOutputs(fb);
    HYD_StateReporter_SetProtectionAction(fb, HYD_PROTECTION_ACTION_STOP);
```

`src/safety_state_manager.c`（`EnterFaultStop`），找到：

```c
    HYD_SafetyStateManager_ResetRuntimeActuation(fb);
    fb->_lastCommandedFlow = 0.0;
    HYD_ClearDirectPendingSlot(fb);
```

替换为：

```c
    HYD_SafetyStateManager_ResetRuntimeActuation(fb);
    HYD_PressureRippleComp_Clear(&fb->_pressureRippleComp);
    fb->_lastCommandedFlow = 0.0;
    HYD_ClearDirectPendingSlot(fb);
```

`safety_state_manager.c` 已经 include 了 `pressure_controller.h`，后者会带入 `pressure_ripple_comp.h`，不用另加 include。

- [ ] **Step 4: `src/motion_control.c` 接线（CRLF）**

4a. 在 `HYD_ExecuteActiveSegmentControl` 的**定义**（约第 2511 行，不是第 74 行的前向声明）之前插入两个函数，并在其局部变量中加入 `profileLimits`：

`src/motion_control.c`，找到：

```c
static HYD_BOOL HYD_ExecuteActiveSegmentControl(HYD_MotionControlFB* fb,
                                                const HYD_MotionSegment* segment,
                                                HYD_REAL elapsed,
                                                HYD_REAL deltaTime,
                                                HYD_RampControllerOutput* rampOutput,
                                                HYD_MotionPlannerOutput* plannerOutput,
                                                HYD_PressureControllerOutput* pressureOutput,
                                                HYD_PumpConverterOutput* pumpOutput,
                                                HYD_ExecutionReference* executionReference) {
    HYD_RampControllerInput rampInput;
```

替换为：

```c
/* Reverse relief flow offered to the pressure controller (L/min, <= 0). */
static HYD_REAL HYD_GetPressureReliefFlow(const HYD_MotionControlFB* fb) {
    return -HYD_GetPumpFlowLimit(fb) * 0.08f;
}

/* Profiled pressure reference (spec 2026-11-23 §4.2/§4.3): pressure segments
 * with a configured Kq (parameter 37) and a non-RBF strategy only. */
static HYD_BOOL HYD_ResolvePressureProfileLimits(const HYD_MotionControlFB* fb,
                                                 const HYD_MotionSegment* segment,
                                                 HYD_RampProfileLimits* limits) {
    HYD_REAL gain;

    if (segment->mode != HYD_MODE_PRESSURE_CLOSED_LOOP ||
        !(fb->_params.kqBarPerSecRpm > 0.0f) ||
        segment->pressureController == HYD_PRESSURE_CONTROLLER_RBF_PID ||
        segment->pressureController == HYD_PRESSURE_CONTROLLER_RBF_PI) {
        return false;
    }
    gain = HYD_PumpConfig_IsValid(&fb->pumpConfig)
        ? HYD_PumpConfig_GetFlowToSpeedGain(&fb->pumpConfig)
        : fb->FLOW_TO_PUMP_SPEED_GAIN;
    if (!HYD_PressureController_ResolveRateLimits(fb->_params.kqBarPerSecRpm,
                                                  fb->_params.ksysBarPerRpm,
                                                  gain,
                                                  segment->targetPressure,
                                                  HYD_ResolvePressureOutputMax(fb, segment),
                                                  HYD_GetPressureReliefFlow(fb),
                                                  &limits->riseRate,
                                                  &limits->fallRate)) {
        return false;
    }
    limits->brakeTime = (fb->_params.pressureBrakeTimeMs > 0.0f)
        ? fb->_params.pressureBrakeTimeMs * 0.001f
        : HYD_THRESH_PRESSURE_PROFILE_BRAKE_TIME_S;
    return true;
}

static HYD_BOOL HYD_ExecuteActiveSegmentControl(HYD_MotionControlFB* fb,
                                                const HYD_MotionSegment* segment,
                                                HYD_REAL elapsed,
                                                HYD_REAL deltaTime,
                                                HYD_RampControllerOutput* rampOutput,
                                                HYD_MotionPlannerOutput* plannerOutput,
                                                HYD_PressureControllerOutput* pressureOutput,
                                                HYD_PumpConverterOutput* pumpOutput,
                                                HYD_ExecutionReference* executionReference) {
    HYD_RampControllerInput rampInput;
    HYD_RampProfileLimits profileLimits;
```

4b. 轨迹调用：

`src/motion_control.c`，找到：

```c
        rampInput.targetPressure = segment->targetPressure;
        rampInput.rampRate = segment->pressureRampRate;
        rampInput.currentTime = controllerTime;
        if (HYD_UseSimulationFixedStep(fb)) {
            HYD_RampController_ExecuteWithDt(&fb->_rampController, &rampInput, deltaTime, rampOutput);
        } else {
```

替换为：

```c
        rampInput.targetPressure = segment->targetPressure;
        rampInput.rampRate = segment->pressureRampRate;
        rampInput.currentTime = controllerTime;
        if (HYD_ResolvePressureProfileLimits(fb, segment, &profileLimits)) {
            HYD_RampController_ExecuteProfiledWithDt(
                &fb->_rampController, &rampInput, &profileLimits,
                HYD_UseSimulationFixedStep(fb)
                    ? deltaTime
                    : controllerTime - fb->_rampController.lastTimestamp,
                rampOutput);
        } else if (HYD_UseSimulationFixedStep(fb)) {
            HYD_RampController_ExecuteWithDt(&fb->_rampController, &rampInput, deltaTime, rampOutput);
        } else {
```

4c. 压力分支（2 处；`pressureInput` 的整体清零已在 Task 4 完成）：

`src/motion_control.c`，找到：

```c
        pressureInput.outputMin = -pumpFlowLimit * 0.08f;
```

替换为：

```c
        pressureInput.outputMin = HYD_GetPressureReliefFlow(fb);
```

`src/motion_control.c`，找到：

```c
        HYD_MotionControl_GetPumpFeedback(&pressureInput.pumpFeedback);
        pressureInput.timestamp = HYD_GetCurrentSegmentTime(fb);
```

替换为：

```c
        HYD_MotionControl_GetPumpFeedback(&pressureInput.pumpFeedback);
        pressureInput.targetPressureRate = rampOutput->rampedPressureRate;
        pressureInput.kqBarPerSecRpm = fb->_params.kqBarPerSecRpm;
        pressureInput.pumpRippleOrder = (HYD_UINT8)fb->_params.pumpRippleOrder;
        pressureInput.rippleComp = &fb->_pressureRippleComp;
        pressureInput.timestamp = HYD_GetCurrentSegmentTime(fb);
```

`HYD_GetPressureReliefFlow` 与原表达式 `-pumpFlowLimit * 0.08f` 数值相同，`pumpFlowLimit` 变量仍被下面的限位标志使用，不要删除。

- [ ] **Step 5: 运行，确认通过**

```bash
wsl.exe -d Ubuntu-22.04 bash -lc 'cd ~/project/hdy-motion-light && cmake --build --preset unixgcc 2>&1 | grep -E "error|warning: " | grep -v "rbf_pid.c\|state_reporter.c\|test_diagnostic_criteria.c"; ./out/build/unixgcc/test_pressure_2dof_closed_loop; ./out/build/unixgcc/test_mode_switch_bumpless | head -1'
```

Expected：没有新告警，输出如下（末位允许 ±0.01）：

```
B1 nominal                 |   0->100 ov 0.78 t98 257 pp 2.00 | 100->140 ov 1.04 t98 162 pp 2.68 | 140-> 80 ov 1.05 t98 342 pp 1.75 |  80->120 ov 0.91 t98 160 pp 2.33 | 120-> 50 ov 0.72 t98 374 pp 1.24
B2 nominal                 |   0->100 ov 0.41 t98 257 pp 0.15 | 100->140 ov 0.26 t98 161 pp 0.23 | 140-> 80 ov 0.37 t98 341 pp 0.13 |  80->120 ov 0.11 t98 233 pp 0.19 | 120-> 50 ov 0.16 t98 391 pp 0.12
B2 servo 3 true/cfg 1.00   |   0->100 ov 0.77 t98 260 pp 0.08 | 100->140 ov 0.15 t98 180 pp 0.14 | 140-> 80 ov 0.30 t98 338 pp 0.07 |  80->120 ov 0.18 t98 173 pp 0.11 | 120-> 50 ov 0.59 t98 371 pp 0.05
B2 servo 7 true/cfg 1.00   |   0->100 ov 0.74 t98 255 pp 0.17 | 100->140 ov 0.21 t98 169 pp 0.26 | 140-> 80 ov 0.26 t98 333 pp 0.16 |  80->120 ov 0.53 t98 163 pp 0.22 | 120-> 50 ov 0.63 t98 368 pp 0.16
B2 servo 3 true/cfg 1.50   |   0->100 ov 1.09 t98 314 pp 0.14 | 100->140 ov 0.48 t98 367 pp 0.23 | 140-> 80 ov 0.25 t98 412 pp 0.12 |  80->120 ov 0.46 t98 285 pp 0.18 | 120-> 50 ov 0.34 t98 403 pp 0.16
B2 servo 7 true/cfg 1.50   |   0->100 ov 1.23 t98 321 pp 0.31 | 100->140 ov 0.56 t98 370 pp 0.48 | 140-> 80 ov 0.26 t98 414 pp 0.28 |  80->120 ov 0.49 t98 277 pp 0.39 | 120-> 50 ov 0.60 t98 402 pp 0.51
2-DOF pressure closed-loop tests passed.
planner_state=28 bytes; motion_control_fb=3432 bytes
```

- [ ] **Step 6: 全量回归与逐位基准**

```bash
wsl.exe -d Ubuntu-22.04 bash -lc 'cd ~/project/hdy-motion-light && ctest --test-dir out/build/unixgcc --output-on-failure 2>&1 | tail -3 && git diff --stat src/motion_control.c'
```

Expected：
- `100% tests passed, 0 tests failed out of 64`，其中 `test_pressure_controller` 的逐位基准覆盖了 FB 级 PI 压力段；
- `src/motion_control.c` 约 50 行改动（Task 4、5 已提交的部分不计入），几乎都是新增行。如果显示数千行改动，说明换行符被转换了。

- [ ] **Step 7: 提交**

```bash
wsl.exe -d Ubuntu-22.04 bash -lc 'cd ~/project/hdy-motion-light && git add include/motion_control.h src/motion_control.c src/safety_state_manager.c tests/test_pressure_2dof_closed_loop.c CMakeLists.txt && git commit -q -m "压力段接线：Kq > 0 时走参考轨迹 + 模型前馈 + 纹波补偿（spec 2026-11-23 §4.6）" -m "motion_control 按参数 37/38/39 计算轨迹速率上限并传入控制器；纹波系数存于 FB，跨段保留，禁用和故障时清零。新增 T6/T8：真实 FB 对 hor1121 对象的闭环回归，超调 <= 1.5 bar，开补偿保压峰峰 <= 1.0 bar。" -m "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>" && git log --oneline -1 && git status --short'
```

---

### Task 7: PLC 写入参数（spec §7）

**Files:**
- Modify: `iec/FB_HydAxis.st`（CRLF）：第 0 步，参数 35/36 写入之后
- Modify: `tests/test_hydaxis_p0_contract.py`

**Interfaces:**
- Consumes：Task 5 的参数编号；现有方法 `SetParameter(iIndex, rValue)`。
- Produces：`ST_HydAxisRef` 需要新增 6 个 REAL 字段：`rPressureKsys`、`rPressureKq`、`rPressureBrakeTimeMs`、`rPumpRippleOrder`、`rPressureKp`、`rPressureKi`。这个结构体不在仓库里，由用户在 PLC 工程中添加，做法与参数 35/36 相同。

规则：
- 字段 > 0 才写，0 表示保持库默认。压力轴不填就保持旧行为。
- 策略仍由现有的 `SetPidType(iTypePressurePID)` 写参数 25。启用新结构时，`iTypePressurePID` 要设为 2（PI）。

- [ ] **Step 1: 写失败测试**

`tests/test_hydaxis_p0_contract.py`（`test_toggle_flow_percent_mode_contract` 末尾），找到：

```python
        self.assertNotIn("rEndVel := UINT_TO_REAL(uiEndSpd)*0.1*0.01*rMaxSpeed;", SOURCE)
```

替换为：

```python
        self.assertNotIn("rEndVel := UINT_TO_REAL(uiEndSpd)*0.1*0.01*rMaxSpeed;", SOURCE)

    def test_pressure_2dof_parameters_contract(self):
        # spec 2026-11-23 §7
        for number, field in (
            (34, "rPressureKsys"),
            (37, "rPressureKq"),
            (38, "rPressureBrakeTimeMs"),
            (39, "rPumpRippleOrder"),
            (11, "rPressureKp"),
            (14, "rPressureKi"),
        ):
            self.assertIn(f"IF stHydAxisRef.{field} > 0.0 THEN", SOURCE)
            self.assertIn(f"SetParameter({number}, stHydAxisRef.{field});", SOURCE)
```

- [ ] **Step 2: 运行，确认失败**

```bash
wsl.exe -d Ubuntu-22.04 bash -lc 'cd ~/project/hdy-motion-light && python3 tests/test_hydaxis_p0_contract.py 2>&1 | tail -3'
```

Expected：`FAILED (failures=1)`，失败在 `test_pressure_2dof_parameters_contract`。

- [ ] **Step 3: 修改 `iec/FB_HydAxis.st`（保持 CRLF）**

`iec/FB_HydAxis.st`（约第 394–396 行），找到：

```iecst
              IF stHydAxisRef.rZeroEndSettleVelocity > 0.0 THEN
                SetParameter(36, stHydAxisRef.rZeroEndSettleVelocity);
              END_IF;
```

替换为：

```iecst
              IF stHydAxisRef.rZeroEndSettleVelocity > 0.0 THEN
                SetParameter(36, stHydAxisRef.rZeroEndSettleVelocity);
              END_IF;

              // 压力闭环二自由度（spec 2026-11-23 §7）：参数 34 ksys bar/rpm、37 Kq bar/s/rpm、
              // 38 制动时间 ms、39 泵每转纹波次数、11/14 PI 增益；0 = 不写，保持库默认（Kq = 0 走旧路径）
              IF stHydAxisRef.rPressureKsys > 0.0 THEN
                SetParameter(34, stHydAxisRef.rPressureKsys);
              END_IF;
              IF stHydAxisRef.rPressureKq > 0.0 THEN
                SetParameter(37, stHydAxisRef.rPressureKq);
              END_IF;
              IF stHydAxisRef.rPressureBrakeTimeMs > 0.0 THEN
                SetParameter(38, stHydAxisRef.rPressureBrakeTimeMs);
              END_IF;
              IF stHydAxisRef.rPumpRippleOrder > 0.0 THEN
                SetParameter(39, stHydAxisRef.rPumpRippleOrder);
              END_IF;
              IF stHydAxisRef.rPressureKp > 0.0 THEN
                SetParameter(11, stHydAxisRef.rPressureKp);
              END_IF;
              IF stHydAxisRef.rPressureKi > 0.0 THEN
                SetParameter(14, stHydAxisRef.rPressureKi);
              END_IF;
```

- [ ] **Step 4: 运行，确认通过**

```bash
wsl.exe -d Ubuntu-22.04 bash -lc 'cd ~/project/hdy-motion-light && python3 tests/test_hydaxis_p0_contract.py 2>&1 | tail -1 && ctest --test-dir out/build/unixgcc --output-on-failure 2>&1 | tail -3 && git diff --stat iec/FB_HydAxis.st'
```

Expected：`OK`；`100% tests passed, 0 tests failed out of 64`；`iec/FB_HydAxis.st | 20 ++++`（只有新增行，换行符没有被转换）。

- [ ] **Step 5: 提交**

```bash
wsl.exe -d Ubuntu-22.04 bash -lc 'cd ~/project/hdy-motion-light && git add iec/FB_HydAxis.st tests/test_hydaxis_p0_contract.py && git commit -q -m "FB_HydAxis：第 0 步写入压力环参数 34/37/38/39/11/14（spec 2026-11-23 §7）" -m "字段大于 0 才写，0 保持库默认。ST_HydAxisRef 需在 PLC 工程中新增 rPressureKsys、rPressureKq、rPressureBrakeTimeMs、rPumpRippleOrder、rPressureKp、rPressureKi。" -m "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>" && git log --oneline -1 && git status --short'
```

---

### Task 8: 对象辨识脚本（spec §8.1）

**Files:**
- Create: `scripts/identify_pressure_plant.py`

**Interfaces:**
- Consumes：VOFA CSV 列 `ActPressure`、`rActPumpPosition`。泵转速由泵轴角差分得到，不依赖 `rActRawSpeedRPM`。
- Produces：逐文件打印 Kq、ksys、分压力段的 Kq、建议的参数 34/37、保压纹波的主要阶次。

方法：
- **拟合**：对 5 bar 以上的全部样本做最小二乘，模型 dP/dt = a·n(t−lag) − b·P，lag 取 0–8 ms 中最好的一个。
  - Kq = a，ksys = a/b；
  - 再按 20 bar 分段单独拟合 Kq；
  - 参数 37 = 0.9 × 最低值。
- **纹波阶次**：取最长的平稳段（1 s 斜率 < 2 bar/s，且不少于 2 s），计算泵轴角 1–30 阶的幅值。
- **告警**：R² < 0.8 时提示数据不适合辨识。开环低速记录就属于这种情况，因为压力上升太慢，被纹波淹没。

- [ ] **Step 1: 写脚本**

新建 `scripts/identify_pressure_plant.py`（LF，仅用标准库）：

```python
#!/usr/bin/env python3
"""Identify the pressure plant from a VOFA CSV (spec 2026-11-23 §8.1).

Model, rpm domain (parameters 34 and 37):
    dP/dt = Kq * (n(t - lag) - P / ksys)
    Kq    bar/s per rpm   (parameter 37 = 0.9 x the lowest Kq over the working range)
    ksys  bar per rpm     (parameter 34)

Method: pump speed n comes from the unwrapped pump angle (rActPumpPosition),
pressure from ActPressure (after the PLC MA3). Both are smoothed over 11 ms,
then dP/dt = a*n(t - lag) - b*P is fitted by least squares over every sample
above 5 bar, for lag 0..8 ms; the best fit gives Kq = a and ksys = a / b.
Kq is also fitted per 20 bar pressure band (ksys fixed) to expose its rise
with pressure. The ripple orders are pressure amplitudes at orders 1..30 of
the pump angle over the longest flat-pressure stretch.

Standard library only. Usage:
    python3 scripts/identify_pressure_plant.py FILE.csv [FILE.csv ...]
"""
import csv
import math
import sys

DT = 0.001
SMOOTH = 11            # samples
MIN_PRESSURE = 5.0     # bar
BAND = 20.0            # bar, Kq pressure bands
MIN_BAND_SAMPLES = 200
MIN_DRIVE = 2.0        # rpm, n - P/ksys
HOLD_SLOPE = 2.0       # bar/s over 1 s (whole ripple periods at >= 5 rpm)
MIN_R2 = 0.8
MIN_HOLD = 2000        # samples
MAX_ORDER = 30


def load(path):
    with open(path, encoding="utf-8-sig", newline="") as handle:
        rows = list(csv.DictReader(handle))
    pressure = [float(r["ActPressure"]) for r in rows]
    angle = [float(r["rActPumpPosition"]) for r in rows]
    return pressure, angle


def unwrap_deg(angle):
    out = [angle[0]]
    offset = 0.0
    for previous, current in zip(angle, angle[1:]):
        step = current - previous
        if step > 180.0:
            offset -= 360.0
        elif step < -180.0:
            offset += 360.0
        out.append(current + offset)
    return out


def smooth(values, width=SMOOTH):
    half = width // 2
    prefix = [0.0]
    for v in values:
        prefix.append(prefix[-1] + v)
    out = []
    for i in range(len(values)):
        lo = max(0, i - half)
        hi = min(len(values), i + half + 1)
        out.append((prefix[hi] - prefix[lo]) / (hi - lo))
    return out


def gradient(values, span=1):
    out = [0.0] * len(values)
    for i in range(span, len(values) - span):
        out[i] = (values[i + span] - values[i - span]) / (2 * span * DT)
    return out


def fit_plant(pressure, speed, rate):
    best = None
    for lag in range(0, 9):
        sxx = sxy = syy = sxd = syd = sdd = 0.0
        count = 0
        for i in range(lag + SMOOTH, len(pressure) - SMOOTH):
            if pressure[i] < MIN_PRESSURE:
                continue
            x = speed[i - lag]
            y = -pressure[i]
            d = rate[i]
            sxx += x * x; sxy += x * y; syy += y * y
            sxd += x * d; syd += y * d; sdd += d * d
            count += 1
        det = sxx * syy - sxy * sxy
        if count < 500 or det <= 0.0:
            continue
        a = (sxd * syy - syd * sxy) / det
        b = (syd * sxx - sxd * sxy) / det
        residual = sdd - 2 * (a * sxd + b * syd) + a * a * sxx + 2 * a * b * sxy + b * b * syy
        r2 = 1.0 - residual / max(sdd, 1e-12)
        if best is None or r2 > best["r2"]:
            best = {"lag": lag, "kq": a, "b": b, "r2": r2, "count": count}
    return best


def kq_bands(pressure, speed, rate, lag, ksys):
    bands = {}
    for i in range(lag + SMOOTH, len(pressure) - SMOOTH):
        if pressure[i] < MIN_PRESSURE:
            continue
        drive = speed[i - lag] - pressure[i] / ksys
        if drive < MIN_DRIVE:
            continue
        key = int(pressure[i] // BAND)
        num, den, cnt = bands.get(key, (0.0, 0.0, 0))
        bands[key] = (num + rate[i] * drive, den + drive * drive, cnt + 1)
    return [(key * BAND, num / den, cnt) for key, (num, den, cnt) in sorted(bands.items())
            if cnt >= MIN_BAND_SAMPLES and den > 0.0]


def longest_hold(pressure):
    slope = gradient(pressure, span=500)
    best = (0, 0)
    start = None
    for i, s in enumerate(slope + [1e9]):
        flat = i < len(pressure) and abs(s) < HOLD_SLOPE and pressure[i] > 10.0
        if flat and start is None:
            start = i
        elif not flat and start is not None:
            if i - start > best[1] - best[0]:
                best = (start, i)
            start = None
    return best


def ripple_orders(pressure, angle, window):
    start, end = window
    if end - start < MIN_HOLD:
        return []
    mean = sum(pressure[start:end]) / (end - start)
    result = []
    for order in range(1, MAX_ORDER + 1):
        c = s = 0.0
        for i in range(start, end):
            theta = math.radians(angle[i]) * order
            c += (pressure[i] - mean) * math.cos(theta)
            s += (pressure[i] - mean) * math.sin(theta)
        result.append((2.0 * math.hypot(c, s) / (end - start), order))
    return sorted(result, reverse=True)


def main(paths):
    for path in paths:
        raw_pressure, raw_angle = load(path)
        pressure = smooth(raw_pressure)
        speed = [v * 60.0 / 360.0 for v in gradient(smooth(unwrap_deg(raw_angle)))]
        rate = gradient(pressure)
        print(f"== {path}")
        fit = fit_plant(pressure, speed, rate)
        if fit is None or fit["kq"] <= 0.0 or fit["b"] <= 0.0:
            print("   not enough data above 5 bar")
            continue
        ksys = fit["kq"] / fit["b"]
        print(f"   Kq   = {fit['kq']:.3f} bar/s/rpm   ksys = {ksys:.3f} bar/rpm"
              f"   (lag {fit['lag']} ms, R2 {fit['r2']:.3f}, {fit['count']} samples)")
        if fit["r2"] < MIN_R2:
            print(f"   WARNING: R2 < {MIN_R2}: pressure rises too slowly for the ripple;"
                  " identify from a closed-loop step record instead")
        bands = kq_bands(pressure, speed, rate, fit["lag"], ksys)
        if bands:
            print("   Kq by pressure band: " +
                  ", ".join(f"{p:.0f}-{p + BAND:.0f} bar {k:.2f}" for p, k, _ in bands))
            lowest = min(k for _, k, _ in bands)
        else:
            lowest = fit["kq"]
        print(f"   parameter 34 = {ksys:.2f}, parameter 37 = {0.9 * min(lowest, fit['kq']):.2f}"
              f" (0.9 x lowest Kq)")
        window = ripple_orders(raw_pressure, raw_angle, longest_hold(raw_pressure))
        if window:
            print("   hold ripple: " + ", ".join(f"order {k} {a:.2f} bar" for a, k in window[:4]))


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(2)
    main(sys.argv[1:])
```

- [ ] **Step 2: 在 hor1121 数据上运行**

```bash
wsl.exe -d Ubuntu-22.04 bash -lc 'cd ~/project/hdy-motion-light && python3 scripts/identify_pressure_plant.py docs/data_deg/hor1121/vofa_waveform_20261121_close50.csv docs/data_deg/hor1121/vofa_waveform_20261121_close100.csv docs/data_deg/hor1121/vofa_waveform_20261121_open25rpm.csv'
```

Expected（2026-11-23 临时副本实测）：

```
== docs/data_deg/hor1121/vofa_waveform_20261121_close50.csv
   Kq   = 2.770 bar/s/rpm   ksys = 5.384 bar/rpm   (lag 3 ms, R2 0.917, 8578 samples)
   Kq by pressure band: 40-60 bar 3.08
   parameter 34 = 5.38, parameter 37 = 2.49 (0.9 x lowest Kq)
   hold ripple: order 12 0.39 bar, order 24 0.26 bar, order 23 0.05 bar, order 11 0.05 bar
== docs/data_deg/hor1121/vofa_waveform_20261121_close100.csv
   Kq   = 2.859 bar/s/rpm   ksys = 5.344 bar/rpm   (lag 4 ms, R2 0.915, 11990 samples)
   Kq by pressure band: 80-100 bar 2.93, 100-120 bar 3.56
   parameter 34 = 5.34, parameter 37 = 2.57 (0.9 x lowest Kq)
== docs/data_deg/hor1121/vofa_waveform_20261121_open25rpm.csv
   Kq   = 2.767 bar/s/rpm   ksys = 4.900 bar/rpm   (lag 0 ms, R2 0.401, 24227 samples)
   WARNING: R2 < 0.8: pressure rises too slowly for the ripple; identify from a closed-loop step record instead
   Kq by pressure band: 0-20 bar 3.02, 20-40 bar 2.76, 40-60 bar 2.13, 60-80 bar 2.05, 80-100 bar 1.89, 100-120 bar 4.38, 120-140 bar 6.22
   parameter 34 = 4.90, parameter 37 = 1.70 (0.9 x lowest Kq)
   hold ripple: order 12 1.23 bar, order 13 0.34 bar, order 1 0.33 bar, order 8 0.24 bar
```

这些结果与 spec §2.1 一致：ksys ≈ 5.2，泵轴 12 阶纹波为主；闭环数据的 Kq 为 2.8–3.6，随压力升高。

- [ ] **Step 3: 提交**

```bash
wsl.exe -d Ubuntu-22.04 bash -lc 'cd ~/project/hdy-motion-light && git add scripts/identify_pressure_plant.py && git commit -q -m "脚本：从 VOFA 记录辨识压力对象 Kq、ksys 与泵纹波阶次（spec 2026-11-23 §8.1）" -m "仅用标准库。最小二乘拟合 dP/dt = Kq·(n − P/ksys)，按 20 bar 分段给出 Kq，建议参数 37 = 0.9 × 最低值；保压段给出泵轴角 1–30 阶纹波幅值。" -m "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>" && git log --oneline -1 && git status --short'
```

---

### Task 9: 最终核对

- [ ] **Step 1: 全量**

```bash
wsl.exe -d Ubuntu-22.04 bash -lc 'cd ~/project/hdy-motion-light && cmake --preset unixgcc > /dev/null && cmake --build --preset unixgcc 2>&1 | grep -E "error|warning: " | grep -v "rbf_pid.c\|state_reporter.c\|test_diagnostic_criteria.c"; ctest --test-dir out/build/unixgcc --output-on-failure 2>&1 | tail -3; ./out/build/unixgcc/test_mode_switch_bumpless | head -1'
```

Expected：没有新告警；`100% tests passed, 0 tests failed out of 64`；`motion_control_fb=3432 bytes`。

- [ ] **Step 2: 提交范围与换行符**

```bash
wsl.exe -d Ubuntu-22.04 bash -lc 'cd ~/project/hdy-motion-light && git diff --stat 0a8afa4 HEAD && git ls-files --eol src/motion_control.c iec/FB_HydAxis.st src/pressure_ripple_comp.c && git status --short'
```

Expected：
- **改动范围**：只涉及本计划“文件结构”表中的文件（加上本计划文档）。
- **换行符**：
  - `src/motion_control.c`、`iec/FB_HydAxis.st` 为 `i/crlf w/crlf`；
  - `src/pressure_ripple_comp.c` 为 `i/lf`。
- **工作区**：`git status` 只剩未跟踪的 `docs/data_deg/hor1121/`（以及用户原有的 `scripts/*.sh` 修改，如果有）。

- [ ] **Step 3: 未开启时的行为**

参数 37 默认为 0。确认下列测试都没有改断言并且通过：`test_rbf_pid*`、`test_pressure_model`、`test_hydro_sim_fb`、`test_direct_mode*`。

---

## 需要告知用户（不属于代码任务）

1. **`rPumpMaxRPM` 不用改**：调用处已显式传入 1700。
2. **用户已确认（2026-11-23）所有轴 `iMechanismType ≥ 100`**，第 0 步会调用 `SetPumpConfig(50, 1700)`（容积效率固定 0.95）：
   - G = 1000/(50 × 0.95) = 21.05 rpm/(L/min)；
   - 泵流量上限 80.75 L/min，卸压流量 −6.46 L/min。
   - 参数 34/37 在转速域，不受 G 影响；只有 PI 增益（L/min/bar）要按 21.05 换算。
3. **上机前先确认泵配置真的生效**：hor1121 记录里卸压流量是 −7.2 L/min，对应 G = 20、限速 1800 的库默认值，说明那次试验时泵配置没有生效（或试验程序不同）。
   - 上机时用 `HYD_ReadParameter` 读回参数 28/29/30（应为 50/0.95/1700）；或者看保压时流量与转速之比是否为 21.05。
   - 如果读回不对，Kp/Ki 改按 G = 20 计算（比 21.05 大 5%，影响很小）。
4. **上机初值**（油阀全关，G = 21.05）：

   | 参数 | 值 |
   |---|---|
   | `iTypePressurePID` | 2 |
   | `rPressureKsys` | 5.2 |
   | `rPressureKq` | 2.5（D9） |
   | `rPressureBrakeTimeMs` | 0 |
   | `rPumpRippleOrder` | 12 |
   | `rPressureKp` | 0.53 |
   | `rPressureKi` | 4.2 |

   Kp、Ki 用辨识出的 Kq（未乘 0.9）计算，fc 取 5 Hz：
   - Kp = 2π·5 / (2.8 × 21.05) ≈ 0.53；
   - Ki = Kp · 2π·5/4 ≈ 4.2。

   锁模、射出轴按 spec §8 逐轴辨识后再填。
