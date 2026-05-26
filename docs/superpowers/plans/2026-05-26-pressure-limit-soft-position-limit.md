# 压力限制与软限位保护 实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在 output_limiter 中实现通用最大压力保护（比例限速 + 报警 + 故障升级）和基于 cylinderConfig.strokeMm 的运行时软限位保护，作为上机测试前的安全底线功能。

**Architecture:** 扩展 `HYD_OutputLimiterInput` 结构体，将压力反馈/限制值和位置反馈/行程信息传入 output_limiter。在 `HYD_OutputLimiter_Execute` 内部独立计算两个 scale 因子，取较小值（不叠加相乘）作为最终缩减比应用到 commandFlow/pumpSpeed。诊断升级（WARNING→FAULT）通过新增的持续时间计数器在 output_limiter 内部完成。

**Tech Stack:** C99, CMake, 无外部依赖

**用户确认的设计决策：**
1. Kp 取保守值（3.0），避免振荡；关键算法加注释
2. 故障升级时间取 1.0s（适合射胶段，避免瞬态误报）
3. 默认软限位带宽 5.0mm，使用位置电子尺反馈（即 AXIS_REF.position）
4. 两种保护不叠加——独立计算 scale，取 min 值（非相乘）
5. minScale = 0.1（比例限速不完全停泵，完全停泵由 FAULT/STOP 负责）

---

## 文件变更总览

| 文件 | 动作 | 职责 |
|------|------|------|
| `include/common_types.h` | 修改 | 新增 `maxPressure` 到 segment；扩展 `HYD_CylinderConfig`；新增诊断码/标志 |
| `include/motion_control.h` | 修改 | 新增 `PRESSURE_LIMIT` 到 FB INPUT 区 |
| `include/output_limiter.h` | 修改 | 扩展 `HYD_OutputLimiterInput`/`Output`；新增内部状态结构体 |
| `include/hyd_config.h` | 修改 | §14B 新增压力限制/软限位阈值 |
| `src/output_limiter.c` | 修改 | 实现压力比例限速 + 软限位减速 + 诊断升级 |
| `src/motion_control.c` | 修改 | 填充新增的 limiterInput 字段 |
| `src/recipe_validator.c` | 修改 | targetPosition vs strokeMm 校验 |
| `src/diagnostics.c` | 修改 | 注册新诊断码到 spec 表和 CodeToString |
| `tests/test_output_limiter.c` | 修改 | 新增压力限制和软限位测试用例 |
| `tests/test_recipe_validator.c` | 修改 | 新增 targetPosition 越界校验测试 |

---

## Task 1: 新增诊断码与标志位

**Files:**
- Modify: `include/common_types.h:155-175`
- Modify: `src/diagnostics.c` (spec table + CodeToString)

- [ ] **Step 1: 在 HYD_DiagnosticCode 枚举中新增四个码**

在 `include/common_types.h` 的 `HYD_DIAG_CODE_PUMP_DIRECTION_CONFLICT` 之前插入：

```c
    HYD_DIAG_CODE_OVER_PRESSURE_LIMIT,       /* WARNING: 超过最大压力限制（比例限速中） */
    HYD_DIAG_CODE_OVER_PRESSURE_LIMIT_FAULT, /* FAULT: 持续超压，停机 */
    HYD_DIAG_CODE_SOFT_LIMIT_REACHED,        /* WARNING: 到达软限位边界 */
    HYD_DIAG_CODE_SOFT_LIMIT_VIOLATED,       /* FAULT: 超出软限位 */
```

- [ ] **Step 2: 在 HYD_DiagnosticFlag 枚举中新增标志位**

在 `HYD_DIAG_FLAG_PUMP_DIRECTION_CONFLICT = 1U << 10` 之后追加：

```c
    HYD_DIAG_FLAG_OVER_PRESSURE_LIMIT = 1U << 11,
    HYD_DIAG_FLAG_SOFT_LIMIT = 1U << 12
```

- [ ] **Step 3: 在 diagnostics.c 的 HYD_DIAGNOSTIC_SPECS 表中注册新码**

在 `HYD_DIAG_CODE_PUMP_DIRECTION_CONFLICT` 条目之前插入四条：

```c
    /* 压力限制 WARNING — 比例限速激活 */
    {
        HYD_DIAG_CODE_OVER_PRESSURE_LIMIT,
        HYD_DIAG_SEVERITY_WARNING,
        HYD_DIAG_SOURCE_EXECUTION,
        HYD_DIAG_RECOVERY_CHECK_COMMAND,
        HYD_PROTECTION_ACTION_DERATE,
        "Pressure limit active: proportional flow reduction"
    },
    /* 压力限制 FAULT — 持续超压停机 */
    {
        HYD_DIAG_CODE_OVER_PRESSURE_LIMIT_FAULT,
        HYD_DIAG_SEVERITY_FAULT,
        HYD_DIAG_SOURCE_EXECUTION,
        HYD_DIAG_RECOVERY_RESTART_SEGMENT,
        HYD_PROTECTION_ACTION_STOP,
        "Pressure limit violated: sustained over-pressure, emergency stop"
    },
    /* 软限位 WARNING — 到达边界 */
    {
        HYD_DIAG_CODE_SOFT_LIMIT_REACHED,
        HYD_DIAG_SEVERITY_WARNING,
        HYD_DIAG_SOURCE_EXECUTION,
        HYD_DIAG_RECOVERY_CHECK_COMMAND,
        HYD_PROTECTION_ACTION_DERATE,
        "Soft position limit reached: flow reduction active"
    },
    /* 软限位 FAULT — 超出极限 */
    {
        HYD_DIAG_CODE_SOFT_LIMIT_VIOLATED,
        HYD_DIAG_SEVERITY_FAULT,
        HYD_DIAG_SOURCE_EXECUTION,
        HYD_DIAG_RECOVERY_RESTART_SEGMENT,
        HYD_PROTECTION_ACTION_STOP,
        "Soft position limit violated: beyond stroke boundary"
    },
```

- [ ] **Step 4: 在 HYD_Diagnostics_CodeToString 的 switch 中新增 case**

```c
    case HYD_DIAG_CODE_OVER_PRESSURE_LIMIT:       return "OVER_PRESSURE_LIMIT";
    case HYD_DIAG_CODE_OVER_PRESSURE_LIMIT_FAULT: return "OVER_PRESSURE_LIMIT_FAULT";
    case HYD_DIAG_CODE_SOFT_LIMIT_REACHED:        return "SOFT_LIMIT_REACHED";
    case HYD_DIAG_CODE_SOFT_LIMIT_VIOLATED:       return "SOFT_LIMIT_VIOLATED";
```

- [ ] **Step 5: 构建验证**

Run: `cmake --build --preset unixgcc 2>&1 | tail -20`
Expected: BUILD SUCCESSFUL, 无 warning

- [ ] **Step 6: 提交**

```bash
git add include/common_types.h src/diagnostics.c
git commit -m "feat: add diagnostic codes for pressure limit and soft position limit"
```

---

## Task 2: 扩展类型定义（segment + cylinderConfig + FB）

**Files:**
- Modify: `include/common_types.h:263-332` (HYD_MotionSegment)
- Modify: `include/common_types.h:546-550` (HYD_CylinderConfig)
- Modify: `include/motion_control.h:207-235` (HYD_MotionControlFB INPUT)

- [ ] **Step 1: 在 HYD_MotionSegment 中新增 maxPressure 字段**

在 `derateRatio` 字段（line 329）之后、`pressureRbfConfig` 之前插入：

```c
    /* 本段最大压力限制 [MPa]。0 表示使用 FB 级全局 PRESSURE_LIMIT。
     * 当 segment.maxPressure > 0 且 fb.PRESSURE_LIMIT > 0 时，取两者较小值生效。
     * 与 pressureCeiling（位置窗口内低压模保护）独立评估，互不干扰。 */
    HYD_REAL maxPressure;
```

- [ ] **Step 2: 扩展 HYD_CylinderConfig 结构体**

将现有 `HYD_CylinderConfig` 定义替换为：

```c
typedef struct {
    HYD_REAL areaExtendMm2;    /* 无杆侧有效面积 [mm²] */
    HYD_REAL areaRetractMm2;   /* 有杆侧有效面积 [mm²] */
    HYD_REAL strokeMm;         /* 最大行程 [mm], 正向软限位极限 */
    HYD_REAL softLimitBandMm;  /* 减速带宽度 [mm], 0 = 不启用软限位 */
    HYD_REAL softLimitRetractMm; /* 负向软限位位置 [mm], 默认 0（完全缩回） */
} HYD_CylinderConfig;
```

- [ ] **Step 3: 在 HYD_MotionControlFB INPUT 区新增 PRESSURE_LIMIT**

在 `HYD_CylinderConfig cylinderConfig;` 之后插入：

```c
    /* 全局最大压力限制 [MPa]。0 表示不启用。
     * 当 segment.maxPressure 也 > 0 时，取两者较小值生效。
     * 所有控制模式下均评估（位置/速度/压力闭环）。 */
    HYD_REAL PRESSURE_LIMIT;
```

- [ ] **Step 4: 构建验证**

Run: `cmake --build --preset unixgcc 2>&1 | tail -20`
Expected: BUILD SUCCESSFUL

- [ ] **Step 5: 提交**

```bash
git add include/common_types.h include/motion_control.h
git commit -m "feat: add maxPressure/PRESSURE_LIMIT fields and extend CylinderConfig for soft limits"
```

---

## Task 3: 新增 hyd_config.h 阈值

**Files:**
- Modify: `include/hyd_config.h` (§14B section)

- [ ] **Step 1: 在 §14B 的 pressure ceiling 阈值之后、RBF-PID 阈值之前插入新阈值块**

在 `HYD_THRESH_PRESSURE_CEILING_FAULT_ESCALATION_S` 定义（line 350）之后插入：

```c

/* --- 压力限制保护阈值（src/output_limiter.c） --- */

/* 压力限制比例增益：超压比例 overRatio 时的输出缩减系数。
 * scale = 1.0 - Kp * overRatio
 * 例：Kp=3.0, 超压10% → scale = 1.0 - 3.0*0.1 = 0.7（输出降至70%）
 * 取保守值避免振荡，实际整定时可适当增大。
 * 单位：无量纲。 */
#define HYD_THRESH_PRESSURE_LIMIT_KP             3.0f

/* 压力限制最小缩放比：比例限速的下限，不允许完全停泵。
 * 完全停泵由 FAULT/STOP 保护层负责。
 * 单位：无量纲 [0, 1)。 */
#define HYD_THRESH_PRESSURE_LIMIT_MIN_SCALE      0.1f

/* 压力限制 WARNING 触发前的 debounce 时长。
 * 取较宽值避免瞬态压力尖峰误报（如射胶段切换瞬间）。
 * 单位：秒。 */
#define HYD_THRESH_PRESSURE_LIMIT_DEBOUNCE_S     0.20f

/* 压力限制 WARNING → FAULT 升级时长。
 * 射胶段压力波动较大，取 1.0s 避免误停机。
 * 单位：秒。 */
#define HYD_THRESH_PRESSURE_LIMIT_FAULT_ESCALATION_S  1.0f

/* --- 软限位保护阈值（src/output_limiter.c） --- */

/* 软限位默认减速带宽度（当 cylinderConfig.softLimitBandMm 未设置时的参考值）。
 * 实际使用 cylinderConfig.softLimitBandMm 字段，此处仅作文档参考。
 * 单位：mm。 */
#define HYD_THRESH_SOFT_LIMIT_DEFAULT_BAND_MM    5.0f

/* 软限位 FAULT 触发前的 debounce 时长（position 到达极限后）。
 * 单位：秒。 */
#define HYD_THRESH_SOFT_LIMIT_FAULT_DEBOUNCE_S   0.50f
```

- [ ] **Step 2: 构建验证**

Run: `cmake --build --preset unixgcc 2>&1 | tail -20`
Expected: BUILD SUCCESSFUL

- [ ] **Step 3: 提交**

```bash
git add include/hyd_config.h
git commit -m "feat: add pressure limit and soft limit thresholds to hyd_config.h §14B"
```

---

## Task 4: 扩展 output_limiter 接口

**Files:**
- Modify: `include/output_limiter.h`

- [ ] **Step 1: 重写 output_limiter.h 完整内容**

```c
#ifndef HYD_OUTPUT_LIMITER_H
#define HYD_OUTPUT_LIMITER_H

#include "common_types.h"

typedef struct {
    /* --- 原有字段 --- */
    HYD_REAL requestedFlow;
    HYD_REAL requestedPumpSpeed;
    HYD_REAL flowToPumpSpeedGain;
    HYD_REAL pumpSpeedLimit;
    HYD_ProtectionAction protectionAction;
    HYD_REAL derateRatio;

    /* --- 压力限制（新增） --- */
    HYD_REAL actualPressure;         /* 当前压力反馈 [MPa] */
    HYD_REAL effectiveMaxPressure;   /* 生效的最大压力限制 [MPa], 0 = 不启用 */

    /* --- 软限位（新增） --- */
    HYD_REAL actualPosition;         /* 当前位置反馈 [mm]（电子尺） */
    HYD_REAL strokeMm;               /* 正向极限 [mm], 0 = 不启用 */
    HYD_REAL softLimitRetractMm;     /* 负向极限 [mm] */
    HYD_REAL softLimitBandMm;        /* 减速带宽度 [mm], 0 = 不启用 */
    HYD_MotionDirection direction;   /* 当前运动方向 */

    /* --- 时间（新增，用于 debounce/升级计时） --- */
    HYD_TIME currentTime;            /* 当前时间戳 [s] */
} HYD_OutputLimiterInput;

typedef struct {
    HYD_REAL commandFlow;
    HYD_REAL pumpSpeed;
    HYD_BOOL derated;

    /* --- 保护状态输出（新增） --- */
    HYD_BOOL pressureLimitActive;    /* 压力比例限速正在生效 */
    HYD_BOOL softLimitActive;        /* 软限位减速正在生效 */
    HYD_DiagnosticCode diagnosticCode; /* 最高优先级的保护诊断码, NONE = 无 */
} HYD_OutputLimiterOutput;

/* output_limiter 内部持久状态（由调用方持有，每段开始时 reset） */
typedef struct {
    /* 压力限制计时 */
    HYD_BOOL pressureBreachActive;     /* 当前是否处于超压状态 */
    HYD_TIME pressureBreachStartTime;  /* 超压开始时间 */
    HYD_BOOL pressureFaultEscalated;   /* 已升级为 FAULT */

    /* 软限位计时 */
    HYD_BOOL softLimitBreachActive;    /* 当前是否处于越界状态 */
    HYD_TIME softLimitBreachStartTime; /* 越界开始时间 */
    HYD_BOOL softLimitFaultEscalated;  /* 已升级为 FAULT */
} HYD_OutputLimiterState;

/* 原有接口保留（向后兼容，不含保护逻辑） */
void HYD_OutputLimiter_Execute(const HYD_OutputLimiterInput* input,
                               HYD_OutputLimiterOutput* output);

/* 带状态的扩展版本（支持压力限制 + 软限位 + debounce + 故障升级） */
void HYD_OutputLimiter_ExecuteWithProtection(
    const HYD_OutputLimiterInput* input,
    HYD_OutputLimiterState* state,
    HYD_OutputLimiterOutput* output);

/* 重置保护状态（每段开始时调用） */
void HYD_OutputLimiter_ResetState(HYD_OutputLimiterState* state);

#endif /* HYD_OUTPUT_LIMITER_H */
```

- [ ] **Step 2: 构建验证（预期 linker error，因新函数未实现）**

Run: `cmake --build --preset unixgcc 2>&1 | grep -c "undefined reference"`
Expected: 非零（linker error），Task 5 修复

- [ ] **Step 3: 提交**

```bash
git add include/output_limiter.h
git commit -m "feat: extend output_limiter interface for pressure limit and soft limit protection"
```

---

## Task 5: 实现 output_limiter 保护逻辑

**Files:**
- Modify: `src/output_limiter.c`

这是核心实现。关键算法必须加注释说明。

- [ ] **Step 1: 在 output_limiter.c 顶部新增 include 和阈值引用**

在 `#include <math.h>` 之后添加：

```c
#include "hyd_config.h"
```

- [ ] **Step 2: 实现 HYD_OutputLimiter_ResetState**

在文件末尾（`HYD_OutputLimiter_Execute` 之后）添加：

```c
void HYD_OutputLimiter_ResetState(HYD_OutputLimiterState* state) {
    if (state == NULL) return;
    state->pressureBreachActive = false;
    state->pressureBreachStartTime = 0.0;
    state->pressureFaultEscalated = false;
    state->softLimitBreachActive = false;
    state->softLimitBreachStartTime = 0.0;
    state->softLimitFaultEscalated = false;
}
```

- [ ] **Step 3: 实现压力限制 scale 计算（静态辅助函数）**

在 `HYD_OutputLimiter_ResetState` 之后添加：

```c
/* ============================================================================
 * 压力限制比例限速算法
 * ----------------------------------------------------------------------------
 * 当实际压力超过 effectiveMaxPressure 时，按超压比例线性缩减输出流量。
 *
 * 算法：
 *   overRatio = (actualPressure - maxPressure) / maxPressure
 *   scale = 1.0 - Kp * overRatio
 *   scale = clamp(scale, minScale, 1.0)
 *
 * 设计意图：
 * - Kp=3.0（保守值）：超压10%时输出降至70%，避免振荡
 * - minScale=0.1：比例限速不完全停泵，完全停泵由 FAULT/STOP 层负责
 * - 只在 actualPressure > maxPressure 时生效，否则 scale = 1.0
 * ============================================================================ */
static HYD_REAL HYD_OutputLimiter_CalcPressureScale(
    HYD_REAL actualPressure,
    HYD_REAL effectiveMaxPressure)
{
    HYD_REAL overRatio;
    HYD_REAL scale;

    /* 未启用或未超压：不限制 */
    if (effectiveMaxPressure <= 0.0 || actualPressure <= effectiveMaxPressure) {
        return 1.0;
    }

    /* 计算超压比例：(实际 - 限制) / 限制 */
    overRatio = (actualPressure - effectiveMaxPressure) / effectiveMaxPressure;

    /* 比例缩减：Kp 越大响应越快，但振荡风险越高 */
    scale = 1.0 - HYD_THRESH_PRESSURE_LIMIT_KP * overRatio;

    /* 钳位到 [minScale, 1.0]，minScale > 0 保证不完全停泵 */
    if (scale < HYD_THRESH_PRESSURE_LIMIT_MIN_SCALE) {
        scale = HYD_THRESH_PRESSURE_LIMIT_MIN_SCALE;
    }
    if (scale > 1.0) {
        scale = 1.0;
    }

    return scale;
}
```

- [ ] **Step 4: 实现软限位 scale 计算（静态辅助函数）**

```c
/* ============================================================================
 * 软限位减速算法
 * ----------------------------------------------------------------------------
 * 当位置进入减速带（距极限 < softLimitBandMm）时，按剩余距离比例缩减输出。
 *
 * 算法：
 *   remaining = 极限位置 - 当前位置（正向）或 当前位置 - 负向极限（负向）
 *   scale = remaining / softLimitBandMm
 *   scale = clamp(scale, 0.0, 1.0)
 *
 * 关键约束：
 * - 只限制【向极限方向】的运动，不阻止【远离极限方向】的运动（允许退回）
 * - 使用电子尺位置反馈（AXIS_REF.position）
 * - strokeMm > 0 且 softLimitBandMm > 0 时才启用
 * ============================================================================ */
static HYD_REAL HYD_OutputLimiter_CalcSoftLimitScale(
    HYD_REAL actualPosition,
    HYD_REAL strokeMm,
    HYD_REAL softLimitRetractMm,
    HYD_REAL softLimitBandMm,
    HYD_MotionDirection direction)
{
    HYD_REAL remaining;
    HYD_REAL scale;

    /* 未启用软限位：不限制 */
    if (strokeMm <= 0.0 || softLimitBandMm <= 0.0) {
        return 1.0;
    }

    /* 正向运动（EXTEND）：检查是否接近 strokeMm */
    if (direction == HYD_DIRECTION_EXTEND) {
        remaining = strokeMm - actualPosition;
        if (remaining < softLimitBandMm) {
            /* 进入减速带：剩余距离越小，scale 越小 */
            scale = remaining / softLimitBandMm;
            if (scale < 0.0) scale = 0.0;
            if (scale > 1.0) scale = 1.0;
            return scale;
        }
    }

    /* 负向运动（RETRACT）：检查是否接近 softLimitRetractMm */
    if (direction == HYD_DIRECTION_RETRACT) {
        remaining = actualPosition - softLimitRetractMm;
        if (remaining < softLimitBandMm) {
            /* 进入减速带：剩余距离越小，scale 越小 */
            scale = remaining / softLimitBandMm;
            if (scale < 0.0) scale = 0.0;
            if (scale > 1.0) scale = 1.0;
            return scale;
        }
    }

    /* 未进入减速带或方向不匹配：不限制 */
    return 1.0;
}
```

- [ ] **Step 5: 实现压力限制诊断升级（静态辅助函数）**

```c
/* 压力限制诊断升级：debounce → WARNING → FAULT
 * 返回当前应报告的诊断码（NONE 表示无需报告） */
static HYD_DiagnosticCode HYD_OutputLimiter_UpdatePressureDiag(
    HYD_REAL pressureScale,
    HYD_TIME currentTime,
    HYD_OutputLimiterState* state)
{
    HYD_TIME elapsed;

    /* 已升级为 FAULT：锁定不回退 */
    if (state->pressureFaultEscalated) {
        return HYD_DIAG_CODE_OVER_PRESSURE_LIMIT_FAULT;
    }

    if (pressureScale < 1.0) {
        /* 超压中 */
        if (!state->pressureBreachActive) {
            /* 首次进入超压：记录起始时间 */
            state->pressureBreachActive = true;
            state->pressureBreachStartTime = currentTime;
        }
        elapsed = currentTime - state->pressureBreachStartTime;

        /* debounce 通过后报 WARNING */
        if (elapsed >= HYD_THRESH_PRESSURE_LIMIT_DEBOUNCE_S) {
            /* 检查是否升级为 FAULT */
            if (elapsed >= (HYD_THRESH_PRESSURE_LIMIT_DEBOUNCE_S +
                           HYD_THRESH_PRESSURE_LIMIT_FAULT_ESCALATION_S)) {
                state->pressureFaultEscalated = true;
                return HYD_DIAG_CODE_OVER_PRESSURE_LIMIT_FAULT;
            }
            return HYD_DIAG_CODE_OVER_PRESSURE_LIMIT;
        }
    } else {
        /* 压力恢复正常：重置计时（但不重置 FAULT 锁定） */
        state->pressureBreachActive = false;
    }

    return HYD_DIAG_CODE_NONE;
}
```

- [ ] **Step 6: 实现软限位诊断升级（静态辅助函数）**

```c
/* 软限位诊断升级：到达极限 → WARNING → FAULT
 * 返回当前应报告的诊断码 */
static HYD_DiagnosticCode HYD_OutputLimiter_UpdateSoftLimitDiag(
    HYD_REAL softLimitScale,
    HYD_TIME currentTime,
    HYD_OutputLimiterState* state)
{
    HYD_TIME elapsed;

    /* 已升级为 FAULT：锁定 */
    if (state->softLimitFaultEscalated) {
        return HYD_DIAG_CODE_SOFT_LIMIT_VIOLATED;
    }

    /* scale <= 0 表示已到达或超出极限 */
    if (softLimitScale <= 0.0) {
        if (!state->softLimitBreachActive) {
            state->softLimitBreachActive = true;
            state->softLimitBreachStartTime = currentTime;
        }
        elapsed = currentTime - state->softLimitBreachStartTime;

        if (elapsed >= HYD_THRESH_SOFT_LIMIT_FAULT_DEBOUNCE_S) {
            state->softLimitFaultEscalated = true;
            return HYD_DIAG_CODE_SOFT_LIMIT_VIOLATED;
        }
        return HYD_DIAG_CODE_SOFT_LIMIT_REACHED;
    } else if (softLimitScale < 1.0) {
        /* 在减速带内但未到极限：报 WARNING，不计时升级 */
        state->softLimitBreachActive = false;
        return HYD_DIAG_CODE_SOFT_LIMIT_REACHED;
    } else {
        /* 正常区域：重置 */
        state->softLimitBreachActive = false;
        return HYD_DIAG_CODE_NONE;
    }
}
```

- [ ] **Step 7: 实现 HYD_OutputLimiter_ExecuteWithProtection 主函数**

```c
void HYD_OutputLimiter_ExecuteWithProtection(
    const HYD_OutputLimiterInput* input,
    HYD_OutputLimiterState* state,
    HYD_OutputLimiterOutput* output)
{
    HYD_REAL commandFlow;
    HYD_REAL pumpSpeed;
    HYD_REAL ratio;
    HYD_REAL pressureScale;
    HYD_REAL softLimitScale;
    HYD_REAL finalScale;
    HYD_DiagnosticCode pressureDiag;
    HYD_DiagnosticCode softLimitDiag;

    if (output == NULL) return;

    output->commandFlow = 0.0;
    output->pumpSpeed = 0.0;
    output->derated = false;
    output->pressureLimitActive = false;
    output->softLimitActive = false;
    output->diagnosticCode = HYD_DIAG_CODE_NONE;

    if (input == NULL) return;

    /* --- STOP 优先：立即归零 --- */
    if (input->protectionAction == HYD_PROTECTION_ACTION_STOP) {
        return;
    }

    /* --- 输入有效性检查 --- */
    if (!HYD_OutputLimiter_IsFinite(input->requestedFlow) ||
        !HYD_OutputLimiter_IsFinite(input->requestedPumpSpeed) ||
        !HYD_OutputLimiter_IsFinite(input->flowToPumpSpeedGain) ||
        !HYD_OutputLimiter_IsFinite(input->pumpSpeedLimit) ||
        input->flowToPumpSpeedGain <= 0.0 ||
        input->pumpSpeedLimit < 0.0) {
        return;
    }

    commandFlow = HYD_OutputLimiter_AbsReal(input->requestedFlow);
    pumpSpeed = HYD_OutputLimiter_AbsReal(input->requestedPumpSpeed);

    /* --- 1. 现有 DERATE 逻辑（pressureCeiling 等触发） --- */
    if (input->protectionAction == HYD_PROTECTION_ACTION_DERATE) {
        ratio = HYD_OutputLimiter_ResolveDerateRatio(input->derateRatio);
        commandFlow *= ratio;
        pumpSpeed *= ratio;
        output->derated = true;
    }

    /* --- 2. 计算压力限制 scale（独立于软限位） --- */
    pressureScale = HYD_OutputLimiter_CalcPressureScale(
        input->actualPressure, input->effectiveMaxPressure);

    /* --- 3. 计算软限位 scale（独立于压力限制） --- */
    softLimitScale = HYD_OutputLimiter_CalcSoftLimitScale(
        input->actualPosition, input->strokeMm,
        input->softLimitRetractMm, input->softLimitBandMm,
        input->direction);

    /* --- 4. 取两者较小值作为最终 scale（不叠加相乘） ---
     * 设计决策：两种保护独立评估，取更严格的那个生效。
     * 例：pressureScale=0.7, softLimitScale=0.5 → finalScale=0.5 */
    finalScale = (pressureScale < softLimitScale) ? pressureScale : softLimitScale;

    /* --- 5. 应用 finalScale --- */
    if (finalScale < 1.0) {
        commandFlow *= finalScale;
        pumpSpeed *= finalScale;

        if (pressureScale <= softLimitScale) {
            output->pressureLimitActive = true;
        }
        if (softLimitScale <= pressureScale) {
            output->softLimitActive = true;
        }
        output->derated = true;
    }

    /* --- 6. 诊断升级（需要 state） --- */
    if (state != NULL) {
        pressureDiag = HYD_OutputLimiter_UpdatePressureDiag(
            pressureScale, input->currentTime, state);
        softLimitDiag = HYD_OutputLimiter_UpdateSoftLimitDiag(
            softLimitScale, input->currentTime, state);

        /* FAULT 优先于 WARNING */
        if (pressureDiag == HYD_DIAG_CODE_OVER_PRESSURE_LIMIT_FAULT) {
            output->diagnosticCode = HYD_DIAG_CODE_OVER_PRESSURE_LIMIT_FAULT;
        } else if (softLimitDiag == HYD_DIAG_CODE_SOFT_LIMIT_VIOLATED) {
            output->diagnosticCode = HYD_DIAG_CODE_SOFT_LIMIT_VIOLATED;
        } else if (pressureDiag != HYD_DIAG_CODE_NONE) {
            output->diagnosticCode = pressureDiag;
        } else {
            output->diagnosticCode = softLimitDiag;
        }
    }

    /* --- 7. pumpSpeedLimit 硬裁剪（最终兜底） --- */
    if (pumpSpeed > input->pumpSpeedLimit) {
        pumpSpeed = input->pumpSpeedLimit;
        commandFlow = pumpSpeed / input->flowToPumpSpeedGain;
    }

    output->commandFlow = commandFlow;
    output->pumpSpeed = pumpSpeed;
}
```

- [ ] **Step 8: 构建验证**

Run: `cmake --build --preset unixgcc 2>&1 | tail -20`
Expected: BUILD SUCCESSFUL

- [ ] **Step 9: 提交**

```bash
git add src/output_limiter.c
git commit -m "feat: implement pressure limit and soft limit protection in output_limiter"
```

---

## Task 6: 在 motion_control.c 中接入保护逻辑

**Files:**
- Modify: `include/motion_control.h` (新增 `_limiterState` 内部字段)
- Modify: `src/motion_control.c` (填充 limiterInput 新字段 + 切换到 WithProtection 调用)

- [ ] **Step 1: 在 HYD_MotionControlFB 的 INTERNAL 区新增 limiterState**

在 `include/motion_control.h` 的 `HYD_DiagnosticCriteriaState _timeoutCriteriaState;` 之后插入：

```c
    /* --- Output limiter protection state (pressure limit + soft limit) --- */
    HYD_OutputLimiterState _limiterState;
```

同时在文件顶部确保 `#include "output_limiter.h"` 存在（如果不存在则添加）。

- [ ] **Step 2: 在 HYD_PrimeSegmentControllers 中重置 limiterState**

在 `src/motion_control.c` 的 `HYD_PrimeSegmentControllers` 函数中，在现有的 `HYD_DiagnosticCriteria_ResetState(&fb->_timeoutCriteriaState);`（约 line 707）之后添加：

```c
    HYD_OutputLimiter_ResetState(&fb->_limiterState);
```

- [ ] **Step 3: 计算 effectiveMaxPressure 并填充 limiterInput 新字段**

在 `src/motion_control.c` 的 `HYD_MotionControlFB_RunRunningState` 中，将现有的 limiterInput 填充块（约 line 1843-1856）替换为：

```c
    limiterInput.requestedFlow = pumpOutput.commandFlow;
    limiterInput.requestedPumpSpeed = pumpOutput.pumpSpeed;
    if (HYD_PumpConfig_IsValid(&fb->pumpConfig)) {
        limiterInput.flowToPumpSpeedGain = HYD_PumpConfig_GetFlowToSpeedGain(&fb->pumpConfig);
        limiterInput.pumpSpeedLimit = HYD_PumpConfig_GetSpeedLimit(&fb->pumpConfig);
    } else {
        limiterInput.flowToPumpSpeedGain = fb->FLOW_TO_PUMP_SPEED_GAIN;
        limiterInput.pumpSpeedLimit = fb->PUMP_SPEED_LIMIT;
    }
    limiterInput.protectionAction = fb->DIAGNOSTIC.protectionAction;
    limiterInput.derateRatio = HYD_Segment_GetDerateRatio(segment);

    /* --- 压力限制参数 ---
     * effectiveMaxPressure = 取 segment.maxPressure 和 fb.PRESSURE_LIMIT 中较小的非零值。
     * 两者都为 0 时不启用压力限制。 */
    {
        HYD_REAL segMax = segment->maxPressure;
        HYD_REAL fbMax = fb->PRESSURE_LIMIT;
        HYD_REAL effective = 0.0;
        if (segMax > 0.0 && fbMax > 0.0) {
            effective = (segMax < fbMax) ? segMax : fbMax;
        } else if (segMax > 0.0) {
            effective = segMax;
        } else if (fbMax > 0.0) {
            effective = fbMax;
        }
        limiterInput.effectiveMaxPressure = effective;
    }
    limiterInput.actualPressure = fb->AXIS_REF.pressure;

    /* --- 软限位参数（使用电子尺位置反馈） --- */
    limiterInput.actualPosition = fb->AXIS_REF.position;
    limiterInput.strokeMm = fb->cylinderConfig.strokeMm;
    limiterInput.softLimitRetractMm = fb->cylinderConfig.softLimitRetractMm;
    limiterInput.softLimitBandMm = fb->cylinderConfig.softLimitBandMm;
    limiterInput.direction = segment->direction;
    limiterInput.currentTime = fb->AXIS_REF.timestamp;

    /* 使用带保护状态的扩展版本 */
    HYD_OutputLimiter_ExecuteWithProtection(&limiterInput, &fb->_limiterState, &limiterOutput);

    pumpOutput.commandFlow = limiterOutput.commandFlow;
    pumpOutput.pumpSpeed = limiterOutput.pumpSpeed;
    plannerOutput.targetFlow = limiterOutput.commandFlow;
    executionReference.flowReference = limiterOutput.commandFlow;

    /* 如果 limiter 报告了 FAULT 级诊断，升级 protectionAction 为 STOP */
    if (limiterOutput.diagnosticCode == HYD_DIAG_CODE_OVER_PRESSURE_LIMIT_FAULT ||
        limiterOutput.diagnosticCode == HYD_DIAG_CODE_SOFT_LIMIT_VIOLATED) {
        fb->DIAGNOSTIC.protectionAction = HYD_PROTECTION_ACTION_STOP;
        fb->DIAGNOSTIC.code = limiterOutput.diagnosticCode;
        fb->DIAGNOSTIC.severity = HYD_DIAG_SEVERITY_FAULT;
    } else if (limiterOutput.diagnosticCode != HYD_DIAG_CODE_NONE &&
               fb->DIAGNOSTIC.protectionAction < HYD_PROTECTION_ACTION_DERATE) {
        /* WARNING 级：仅在当前无更高优先级保护时设置 */
        fb->DIAGNOSTIC.code = limiterOutput.diagnosticCode;
        fb->DIAGNOSTIC.severity = HYD_DIAG_SEVERITY_WARNING;
    }
```

- [ ] **Step 4: 构建验证**

Run: `cmake --build --preset unixgcc 2>&1 | tail -20`
Expected: BUILD SUCCESSFUL

- [ ] **Step 5: 运行现有测试确保不回归**

Run: `ctest --test-dir out/build/unixgcc --output-on-failure 2>&1 | tail -20`
Expected: 所有现有测试 PASS

- [ ] **Step 6: 提交**

```bash
git add include/motion_control.h src/motion_control.c
git commit -m "feat: wire pressure limit and soft limit into motion_control execution path"
```

---

## Task 7: 启动前校验（recipe_validator）

**Files:**
- Modify: `src/recipe_validator.c`

- [ ] **Step 1: 在 HYD_RecipeValidator_ValidateSegment 中新增 targetPosition 校验**

在现有的 `pressureCeiling` 校验块之后（约 line 335 附近，函数 return true 之前）插入：

```c
    /* --- 软限位启动前校验 ---
     * 当 cylinderConfig.strokeMm > 0 时，拒绝 targetPosition 超出行程范围的段。
     * 这是静态校验，运行时保护由 output_limiter 负责。 */
    if (cylinderConfig != NULL && cylinderConfig->strokeMm > 0.0) {
        if (segment->targetPosition > cylinderConfig->strokeMm) {
            if (code) *code = HYD_DIAG_CODE_SOFT_LIMIT_VIOLATED;
            return false;
        }
        if (segment->targetPosition < cylinderConfig->softLimitRetractMm) {
            if (code) *code = HYD_DIAG_CODE_SOFT_LIMIT_VIOLATED;
            return false;
        }
    }
```

注意：需要检查 `HYD_RecipeValidator_ValidateSegment` 的函数签名是否已接收 `cylinderConfig` 参数。如果没有，需要扩展签名添加 `const HYD_CylinderConfig* cylinderConfig` 参数，并在所有调用点传入 `&fb->cylinderConfig`。

- [ ] **Step 2: 如需扩展函数签名，更新调用点**

在 `src/motion_control.c` 中搜索 `HYD_RecipeValidator_ValidateSegment` 的调用，添加 `&fb->cylinderConfig` 参数。

- [ ] **Step 3: 构建验证**

Run: `cmake --build --preset unixgcc 2>&1 | tail -20`
Expected: BUILD SUCCESSFUL

- [ ] **Step 4: 提交**

```bash
git add src/recipe_validator.c include/recipe_validator.h src/motion_control.c
git commit -m "feat: add targetPosition vs strokeMm validation in recipe_validator"
```

---

## Task 8: 测试——压力限制与软限位

**Files:**
- Modify: `tests/test_output_limiter.c`

- [ ] **Step 1: 新增压力限制比例缩减测试**

在 `test_output_limiter.c` 的 `main()` 之前添加：

```c
static void test_pressure_limit_proportional_reduction(void) {
    HYD_OutputLimiterInput input;
    HYD_OutputLimiterState state;
    HYD_OutputLimiterOutput output;

    memset(&input, 0, sizeof(input));
    memset(&state, 0, sizeof(state));

    input.requestedFlow = 100.0;
    input.requestedPumpSpeed = 1500.0;
    input.flowToPumpSpeedGain = 15.0;
    input.pumpSpeedLimit = 3000.0;
    input.protectionAction = HYD_PROTECTION_ACTION_NONE;
    input.derateRatio = 0.0;

    /* 超压 10%: scale = 1.0 - 3.0*0.1 = 0.7 */
    input.actualPressure = 22.0;
    input.effectiveMaxPressure = 20.0;
    input.strokeMm = 0.0; /* 软限位不启用 */
    input.currentTime = 1.0;

    HYD_OutputLimiter_ExecuteWithProtection(&input, &state, &output);

    assert(output.pressureLimitActive == true);
    assert(output.softLimitActive == false);
    /* 100.0 * 0.7 = 70.0 */
    assert_real_eq(output.commandFlow, 70.0, 0.01, "pressure limit 10% over");
    assert_real_eq(output.pumpSpeed, 1050.0, 0.01, "pump speed 10% over");
    printf("  PASS: test_pressure_limit_proportional_reduction\n");
}
```

- [ ] **Step 2: 新增压力限制 minScale 钳位测试**

```c
static void test_pressure_limit_min_scale_clamp(void) {
    HYD_OutputLimiterInput input;
    HYD_OutputLimiterState state;
    HYD_OutputLimiterOutput output;

    memset(&input, 0, sizeof(input));
    memset(&state, 0, sizeof(state));

    input.requestedFlow = 100.0;
    input.requestedPumpSpeed = 1500.0;
    input.flowToPumpSpeedGain = 15.0;
    input.pumpSpeedLimit = 3000.0;
    input.protectionAction = HYD_PROTECTION_ACTION_NONE;

    /* 超压 50%: scale = 1.0 - 3.0*0.5 = -0.5 → clamp to 0.1 */
    input.actualPressure = 30.0;
    input.effectiveMaxPressure = 20.0;
    input.strokeMm = 0.0;
    input.currentTime = 1.0;

    HYD_OutputLimiter_ExecuteWithProtection(&input, &state, &output);

    /* 100.0 * 0.1 = 10.0 */
    assert_real_eq(output.commandFlow, 10.0, 0.01, "pressure limit min scale");
    printf("  PASS: test_pressure_limit_min_scale_clamp\n");
}
```

- [ ] **Step 3: 新增软限位正向减速测试**

```c
static void test_soft_limit_extend_deceleration(void) {
    HYD_OutputLimiterInput input;
    HYD_OutputLimiterState state;
    HYD_OutputLimiterOutput output;

    memset(&input, 0, sizeof(input));
    memset(&state, 0, sizeof(state));

    input.requestedFlow = 80.0;
    input.requestedPumpSpeed = 1200.0;
    input.flowToPumpSpeedGain = 15.0;
    input.pumpSpeedLimit = 3000.0;
    input.protectionAction = HYD_PROTECTION_ACTION_NONE;

    /* 位置 97mm, 行程 100mm, 减速带 5mm → remaining=3, scale=3/5=0.6 */
    input.actualPosition = 97.0;
    input.strokeMm = 100.0;
    input.softLimitBandMm = 5.0;
    input.softLimitRetractMm = 0.0;
    input.direction = HYD_DIRECTION_EXTEND;
    input.effectiveMaxPressure = 0.0; /* 压力限制不启用 */
    input.currentTime = 1.0;

    HYD_OutputLimiter_ExecuteWithProtection(&input, &state, &output);

    assert(output.softLimitActive == true);
    assert(output.pressureLimitActive == false);
    /* 80.0 * 0.6 = 48.0 */
    assert_real_eq(output.commandFlow, 48.0, 0.01, "soft limit extend");
    printf("  PASS: test_soft_limit_extend_deceleration\n");
}
```

- [ ] **Step 4: 新增软限位不阻止反向运动测试**

```c
static void test_soft_limit_does_not_block_retract(void) {
    HYD_OutputLimiterInput input;
    HYD_OutputLimiterState state;
    HYD_OutputLimiterOutput output;

    memset(&input, 0, sizeof(input));
    memset(&state, 0, sizeof(state));

    input.requestedFlow = 80.0;
    input.requestedPumpSpeed = 1200.0;
    input.flowToPumpSpeedGain = 15.0;
    input.pumpSpeedLimit = 3000.0;
    input.protectionAction = HYD_PROTECTION_ACTION_NONE;

    /* 位置 99mm（接近正向极限），但方向是 RETRACT → 不限制 */
    input.actualPosition = 99.0;
    input.strokeMm = 100.0;
    input.softLimitBandMm = 5.0;
    input.softLimitRetractMm = 0.0;
    input.direction = HYD_DIRECTION_RETRACT;
    input.effectiveMaxPressure = 0.0;
    input.currentTime = 1.0;

    HYD_OutputLimiter_ExecuteWithProtection(&input, &state, &output);

    assert(output.softLimitActive == false);
    assert_real_eq(output.commandFlow, 80.0, 0.01, "retract not blocked");
    printf("  PASS: test_soft_limit_does_not_block_retract\n");
}
```

- [ ] **Step 5: 新增两种保护取 min 测试**

```c
static void test_protection_takes_min_scale(void) {
    HYD_OutputLimiterInput input;
    HYD_OutputLimiterState state;
    HYD_OutputLimiterOutput output;

    memset(&input, 0, sizeof(input));
    memset(&state, 0, sizeof(state));

    input.requestedFlow = 100.0;
    input.requestedPumpSpeed = 1500.0;
    input.flowToPumpSpeedGain = 15.0;
    input.pumpSpeedLimit = 3000.0;
    input.protectionAction = HYD_PROTECTION_ACTION_NONE;

    /* 压力限制: 超压10% → scale=0.7
     * 软限位: remaining=2, band=5 → scale=0.4
     * 最终取 min(0.7, 0.4) = 0.4 */
    input.actualPressure = 22.0;
    input.effectiveMaxPressure = 20.0;
    input.actualPosition = 98.0;
    input.strokeMm = 100.0;
    input.softLimitBandMm = 5.0;
    input.softLimitRetractMm = 0.0;
    input.direction = HYD_DIRECTION_EXTEND;
    input.currentTime = 1.0;

    HYD_OutputLimiter_ExecuteWithProtection(&input, &state, &output);

    assert(output.softLimitActive == true);
    /* 100.0 * 0.4 = 40.0 */
    assert_real_eq(output.commandFlow, 40.0, 0.01, "min of two scales");
    printf("  PASS: test_protection_takes_min_scale\n");
}
```

- [ ] **Step 6: 新增 FAULT 升级测试**

```c
static void test_pressure_limit_fault_escalation(void) {
    HYD_OutputLimiterInput input;
    HYD_OutputLimiterState state;
    HYD_OutputLimiterOutput output;

    memset(&input, 0, sizeof(input));
    HYD_OutputLimiter_ResetState(&state);

    input.requestedFlow = 100.0;
    input.requestedPumpSpeed = 1500.0;
    input.flowToPumpSpeedGain = 15.0;
    input.pumpSpeedLimit = 3000.0;
    input.protectionAction = HYD_PROTECTION_ACTION_NONE;
    input.actualPressure = 25.0;
    input.effectiveMaxPressure = 20.0;
    input.strokeMm = 0.0;

    /* t=0: 首次超压，debounce 未过 → NONE */
    input.currentTime = 0.0;
    HYD_OutputLimiter_ExecuteWithProtection(&input, &state, &output);
    assert(output.diagnosticCode == HYD_DIAG_CODE_NONE);

    /* t=0.25s: debounce(0.2s) 已过 → WARNING */
    input.currentTime = 0.25;
    HYD_OutputLimiter_ExecuteWithProtection(&input, &state, &output);
    assert(output.diagnosticCode == HYD_DIAG_CODE_OVER_PRESSURE_LIMIT);

    /* t=1.3s: debounce(0.2) + escalation(1.0) = 1.2s 已过 → FAULT */
    input.currentTime = 1.3;
    HYD_OutputLimiter_ExecuteWithProtection(&input, &state, &output);
    assert(output.diagnosticCode == HYD_DIAG_CODE_OVER_PRESSURE_LIMIT_FAULT);

    printf("  PASS: test_pressure_limit_fault_escalation\n");
}
```

- [ ] **Step 7: 新增不启用时向后兼容测试**

```c
static void test_no_protection_when_disabled(void) {
    HYD_OutputLimiterInput input;
    HYD_OutputLimiterState state;
    HYD_OutputLimiterOutput output;

    memset(&input, 0, sizeof(input));
    HYD_OutputLimiter_ResetState(&state);

    input.requestedFlow = 100.0;
    input.requestedPumpSpeed = 1500.0;
    input.flowToPumpSpeedGain = 15.0;
    input.pumpSpeedLimit = 3000.0;
    input.protectionAction = HYD_PROTECTION_ACTION_NONE;

    /* 两者都为 0：不启用任何保护 */
    input.effectiveMaxPressure = 0.0;
    input.strokeMm = 0.0;
    input.actualPressure = 999.0; /* 即使压力很高也不触发 */
    input.currentTime = 1.0;

    HYD_OutputLimiter_ExecuteWithProtection(&input, &state, &output);

    assert(output.pressureLimitActive == false);
    assert(output.softLimitActive == false);
    assert_real_eq(output.commandFlow, 100.0, 0.01, "backward compat");
    printf("  PASS: test_no_protection_when_disabled\n");
}
```

- [ ] **Step 8: 在 main() 中注册所有新测试**

```c
    test_pressure_limit_proportional_reduction();
    test_pressure_limit_min_scale_clamp();
    test_soft_limit_extend_deceleration();
    test_soft_limit_does_not_block_retract();
    test_protection_takes_min_scale();
    test_pressure_limit_fault_escalation();
    test_no_protection_when_disabled();
```

- [ ] **Step 9: 构建并运行测试**

Run: `cmake --build --preset unixgcc && ctest --test-dir out/build/unixgcc -R test_output_limiter --output-on-failure`
Expected: ALL PASS

- [ ] **Step 10: 提交**

```bash
git add tests/test_output_limiter.c
git commit -m "test: add pressure limit and soft limit protection tests"
```

---

## Task 9: 测试——recipe_validator targetPosition 校验

**Files:**
- Modify: `tests/test_recipe_validator.c`

- [ ] **Step 1: 新增 targetPosition 超出 strokeMm 被拒绝的测试**

```c
static void test_target_position_exceeds_stroke_rejected(void) {
    HYD_MotionSegment seg = make_valid_segment();
    HYD_CylinderConfig cyl = {0};
    HYD_DiagnosticCode code = HYD_DIAG_CODE_NONE;

    cyl.strokeMm = 100.0;
    cyl.softLimitRetractMm = 0.0;

    /* targetPosition > strokeMm → 拒绝 */
    seg.targetPosition = 105.0;
    assert(HYD_RecipeValidator_ValidateSegment(&seg, &cyl, &code) == false);
    assert(code == HYD_DIAG_CODE_SOFT_LIMIT_VIOLATED);

    /* targetPosition == strokeMm → 通过（边界值） */
    seg.targetPosition = 100.0;
    code = HYD_DIAG_CODE_NONE;
    assert(HYD_RecipeValidator_ValidateSegment(&seg, &cyl, &code) == true);

    printf("  PASS: test_target_position_exceeds_stroke_rejected\n");
}
```

- [ ] **Step 2: 新增 targetPosition 低于 softLimitRetractMm 被拒绝的测试**

```c
static void test_target_position_below_retract_limit_rejected(void) {
    HYD_MotionSegment seg = make_valid_segment();
    HYD_CylinderConfig cyl = {0};
    HYD_DiagnosticCode code = HYD_DIAG_CODE_NONE;

    cyl.strokeMm = 100.0;
    cyl.softLimitRetractMm = 5.0;

    /* targetPosition < softLimitRetractMm → 拒绝 */
    seg.targetPosition = 3.0;
    assert(HYD_RecipeValidator_ValidateSegment(&seg, &cyl, &code) == false);
    assert(code == HYD_DIAG_CODE_SOFT_LIMIT_VIOLATED);

    /* targetPosition == softLimitRetractMm → 通过 */
    seg.targetPosition = 5.0;
    code = HYD_DIAG_CODE_NONE;
    assert(HYD_RecipeValidator_ValidateSegment(&seg, &cyl, &code) == true);

    printf("  PASS: test_target_position_below_retract_limit_rejected\n");
}
```

- [ ] **Step 3: 新增 strokeMm=0 时不校验的测试（向后兼容）**

```c
static void test_stroke_zero_skips_position_validation(void) {
    HYD_MotionSegment seg = make_valid_segment();
    HYD_CylinderConfig cyl = {0};
    HYD_DiagnosticCode code = HYD_DIAG_CODE_NONE;

    cyl.strokeMm = 0.0; /* 不启用 */

    seg.targetPosition = 9999.0; /* 任意大值 */
    assert(HYD_RecipeValidator_ValidateSegment(&seg, &cyl, &code) == true);

    printf("  PASS: test_stroke_zero_skips_position_validation\n");
}
```

- [ ] **Step 4: 在 main() 中注册新测试**

```c
    test_target_position_exceeds_stroke_rejected();
    test_target_position_below_retract_limit_rejected();
    test_stroke_zero_skips_position_validation();
```

- [ ] **Step 5: 构建并运行测试**

Run: `cmake --build --preset unixgcc && ctest --test-dir out/build/unixgcc -R test_recipe_validator --output-on-failure`
Expected: ALL PASS

- [ ] **Step 6: 提交**

```bash
git add tests/test_recipe_validator.c
git commit -m "test: add targetPosition vs strokeMm validation tests"
```

---

## Task 10: 集成验证与全量测试

**Files:** 无新文件

- [ ] **Step 1: 全量构建**

Run: `cmake --preset unixgcc && cmake --build --preset unixgcc`
Expected: BUILD SUCCESSFUL, 0 warnings

- [ ] **Step 2: 全量测试**

Run: `ctest --test-dir out/build/unixgcc --output-on-failure`
Expected: ALL PASS

- [ ] **Step 3: 运行 main 端到端验证**

Run: `./out/build/unixgcc/main`
Expected: 正常完成，无 crash

- [ ] **Step 4: 最终提交（如有遗漏修复）**

如果前面步骤有修复，统一提交：
```bash
git add -A
git commit -m "fix: integration fixes for pressure limit and soft limit"
```
