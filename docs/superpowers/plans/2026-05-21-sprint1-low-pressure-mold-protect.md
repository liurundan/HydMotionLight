# Sprint 1：低压护膜支持 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 把"段内压力软上限 (`pressureCeiling`) + position-window-aware 保护"下沉为 HydroMotionLib 的一级原语。所有 `HYD_ControlMode`（POSITION / SPEED_RAMP / PRESSURE_CLOSED_LOOP）在配置了 ceiling 的段内都会评估"实测压力 > ceiling + tolerance"并先 DERATE、持续超限后升级 FAULT/STOP，PLC 工艺层无需再自己实现合模/低压护膜判据。

**Architecture:**
- 在 `HYD_MotionSegment` 新增 4 字段：`pressureCeiling` / `pressureCeilingTolerance` / `pressureCeilingPositionStart` / `pressureCeilingPositionEnd`。这两个 position 字段定义"激活窗口"（mold-protect zone），零值表示 always-on。
- 新增 2 个诊断码：`HYD_DIAG_CODE_PRESSURE_CEILING_EXCEEDED`（WARNING, DERATE）与 `HYD_DIAG_CODE_PRESSURE_CEILING_VIOLATED`（FAULT, STOP），后者由 ceiling-exceeded 持续超过 fault-escalation 阈值后升级而来。
- 在 `HYD_UpdateExecutionDiagnostics` 中所有 mode 下评估 ceiling（独立于原有 `HYD_DIAG_CODE_OVER_PRESSURE` 的 PRESSURE_CLOSED_LOOP-only 检查），优先级在 timeout 之后、`OVER_PRESSURE` 之前。
- `HYD_OutputLimiter` 的 `derateRatio` 改由段配置（新增段字段 `derateRatio`，0/越界回退默认 0.5），同步替换 `motion_control.c` 中硬编码 `limiterInput.derateRatio = 0.5`。
- 新增 `HYD_ActionProfile_BuildClampCloseWithMoldProtect()` 模板 builder，把"低速护膜段"包装成一句调用，避免每个工艺工程师手抄一遍参数。
- 仿真层无需改动：`HydraulicSimEnv` 已有 `inject_mold_obstacle / obstacle_pos_mm / obstacle_stiffness_N_mm`；clamp 轴已有 `close_pos_mm + tie_bar_stiffness_N_mm`。集成测试直接驱动 `fb->AXIS_REF.pressure` 模拟"模具内异物 → 压力提前升高"即可，不需要复刻完整 sim-FB 联动。

**Tech Stack:** C99（gcc 9+/clang 12+）、CMake 3.16+、ctest、Beremiz/matiec IEC 类型系统、`tests/` 下纯 C 单元测试。

**Spec:** `docs/superpowers/specs/2026-05-21-code-review-and-roadmap-design.md`（Sprint 1 §1.1–§1.7）

**任务编号 → spec 子任务映射：**

| Plan Task | Spec ID | 产出 |
|---|---|---|
| Task 1 | 1.1 | `common_types.h` 增加 4 字段 + segment_limits 访问器 |
| Task 2 | 1.2 | 诊断表追加 2 条 + 枚举追加 2 个码 + flags 位扩展 |
| Task 3 | 1.3 | `motion_control.c` 在所有 mode 下评估 ceiling |
| Task 4 | 1.4 | `output_limiter` 的 derateRatio 改可段配置 |
| Task 5 | 1.5 | `action_profile.c` 新增 `BuildClampCloseWithMoldProtect` builder |
| Task 6 | 1.6 | `tests/test_mold_protect.c` 端到端测试 |
| Task 7 | 1.7 | `motion-profile-archetypes.md` + `motion-runtime-contract.md` 文档同步 |

**前置准备：所有 Task 共用的工作目录与编译命令**

```bash
cd /home/dan/project/hdy-motion-light

cmake --preset unixgcc
cmake --build --preset unixgcc

# 全量回归
ctest --test-dir out/build/unixgcc --output-on-failure

# 单测试
ctest --test-dir out/build/unixgcc -R '<test_name>' --output-on-failure
```

**重要约定：**
- 每次添加新 `tests/*.c` 文件后必须 re-run `cmake --preset unixgcc`（CMakeLists 使用 `file(GLOB_RECURSE ...)`）。
- TDD：先写复现 spec 的失败测试，再修代码，再确认 PASS。
- 每个 Task 末尾独立 commit。
- Sprint 0 已合并到 master（commit 8649a3c）；本 Sprint 假设 Sprint 0 的 7 个修复已生效。如发现 Sprint 0 相关回归，必须先解决再继续本 Sprint。

---

## Task 1: HYD_MotionSegment 增加 4 个 ceiling 字段（spec §1.1）

**目标：** 在 `HYD_MotionSegment` 中新增 4 个字段定义压力软上限与激活窗口，并在 `segment_limits.c/h` 中提供访问器以便上层统一通过 helper 取值（保持与现有 `HYD_Segment_GetPressureTolerance` 一致的风格）。

**Files:**
- Modify: `include/common_types.h`（在 `HYD_MotionSegment` 增加 4 个字段 + derateRatio）
- Modify: `include/segment_limits.h`（新增 5 个访问器声明）
- Modify: `src/segment_limits.c`（新增 5 个访问器实现）
- Modify: `src/recipe_validator.c`（早失败：ceiling > 0 且 window start >= window end 时拒绝段）
- Test: `tests/test_recipe_validator.c`（追加 ceiling 字段的早失败用例）

### Steps

- [ ] **Step 1.1: 阅读 `segment_limits.h` 已有访问器风格，确认命名约定**

```bash
grep -n "HYD_Segment_Get" include/segment_limits.h
```

Expected：列出 `HYD_Segment_GetPressureTolerance` / `HYD_Segment_GetPositionTolerance` / `HYD_Segment_GetFlowTolerance` / `HYD_Segment_GetVelocityTolerance` 等访问器。后续 5 个新访问器需保持 `HYD_Segment_GetXxx(const HYD_MotionSegment*) -> HYD_REAL` 的签名风格。

- [ ] **Step 1.2: 在 `include/common_types.h` 的 `HYD_MotionSegment` 中追加 5 个字段**

定位 `HYD_MotionSegment` 结构体（约 249-300 行）。在 `HYD_RbfPidConfig pressureRbfConfig;` 行**之前**追加：

```c
    /* Pressure ceiling — low-pressure mold-protect primitive.
     * Activates when actual position is within [pressureCeilingPositionStart,
     * pressureCeilingPositionEnd] AND |position end - start| > 0.
     * When both position fields are 0, the ceiling is always-on.
     * Zero pressureCeiling disables the check entirely. */
    HYD_REAL pressureCeiling;                /* MPa, 0 disables ceiling check */
    HYD_REAL pressureCeilingTolerance;       /* MPa, hysteresis above ceiling before DERATE; 0 uses pressureTolerance */
    HYD_REAL pressureCeilingPositionStart;   /* mm, window lower bound; 0 means always-on with End */
    HYD_REAL pressureCeilingPositionEnd;     /* mm, window upper bound; <=Start means always-on */

    /* Per-segment derate ratio for protectionAction = DERATE.
     * Range (0.0, 1.0). Zero or out-of-range falls back to library default 0.5.
     * Replaces the hardcoded limiterInput.derateRatio = 0.5 in motion_control.c. */
    HYD_REAL derateRatio;
```

- [ ] **Step 1.3: 在 `include/segment_limits.h` 新增 5 个访问器声明**

定位已有 `HYD_Segment_GetPressureTolerance` 声明（grep 行号），在其下方按相同风格追加：

```c
HYD_REAL HYD_Segment_GetPressureCeiling(const HYD_MotionSegment* segment);
HYD_REAL HYD_Segment_GetPressureCeilingTolerance(const HYD_MotionSegment* segment);
HYD_REAL HYD_Segment_GetPressureCeilingPositionStart(const HYD_MotionSegment* segment);
HYD_REAL HYD_Segment_GetPressureCeilingPositionEnd(const HYD_MotionSegment* segment);
HYD_REAL HYD_Segment_GetDerateRatio(const HYD_MotionSegment* segment);
HYD_BOOL HYD_Segment_PressureCeilingActiveAt(const HYD_MotionSegment* segment, HYD_REAL actualPosition);
```

- [ ] **Step 1.4: 在 `src/segment_limits.c` 实现 6 个访问器**

在文件末尾追加：

```c
HYD_REAL HYD_Segment_GetPressureCeiling(const HYD_MotionSegment* segment) {
    if (segment == NULL) {
        return 0.0;
    }
    return (segment->pressureCeiling > 0.0) ? segment->pressureCeiling : 0.0;
}

HYD_REAL HYD_Segment_GetPressureCeilingTolerance(const HYD_MotionSegment* segment) {
    if (segment == NULL) {
        return 0.0;
    }
    if (segment->pressureCeilingTolerance > 0.0) {
        return segment->pressureCeilingTolerance;
    }
    /* Fall back to generic pressureTolerance if dedicated value not configured. */
    return HYD_Segment_GetPressureTolerance(segment);
}

HYD_REAL HYD_Segment_GetPressureCeilingPositionStart(const HYD_MotionSegment* segment) {
    if (segment == NULL) {
        return 0.0;
    }
    return segment->pressureCeilingPositionStart;
}

HYD_REAL HYD_Segment_GetPressureCeilingPositionEnd(const HYD_MotionSegment* segment) {
    if (segment == NULL) {
        return 0.0;
    }
    return segment->pressureCeilingPositionEnd;
}

HYD_REAL HYD_Segment_GetDerateRatio(const HYD_MotionSegment* segment) {
    if (segment == NULL) {
        return 0.0;
    }
    if (segment->derateRatio > 0.0 && segment->derateRatio < 1.0) {
        return segment->derateRatio;
    }
    return 0.0;  /* 0 signals "use library default" to HYD_OutputLimiter */
}

HYD_BOOL HYD_Segment_PressureCeilingActiveAt(const HYD_MotionSegment* segment,
                                             HYD_REAL actualPosition) {
    HYD_REAL start;
    HYD_REAL end;

    if (segment == NULL || HYD_Segment_GetPressureCeiling(segment) <= 0.0) {
        return false;
    }

    start = segment->pressureCeilingPositionStart;
    end = segment->pressureCeilingPositionEnd;

    /* Always-on when window is degenerate (end <= start). */
    if (end <= start) {
        return true;
    }

    return (actualPosition >= start) && (actualPosition <= end);
}
```

确保 `src/segment_limits.c` 顶部已 `#include "segment_limits.h"`（已有则跳过）。

- [ ] **Step 1.5: 在 `src/recipe_validator.c` 中追加 ceiling-window 早失败校验**

```bash
grep -n "HYD_RecipeValidator_ValidateSegment" src/recipe_validator.c | head -5
```

打开找到的函数。在现有所有字段校验之后、return true 之前追加：

```c
    /* Pressure ceiling configuration must be coherent. */
    if (segment->pressureCeiling > 0.0) {
        /* Tolerance must be finite and nonnegative (zero is allowed → falls back to pressureTolerance). */
        if (!isfinite(segment->pressureCeilingTolerance) || segment->pressureCeilingTolerance < 0.0) {
            if (code != NULL) *code = HYD_DIAG_CODE_SEGMENT_INVALID;
            return false;
        }
        /* Position window: either degenerate (both 0 / end<=start → always-on)
         * or strictly ordered. Reject NaN / -Inf. */
        if (!isfinite(segment->pressureCeilingPositionStart) ||
            !isfinite(segment->pressureCeilingPositionEnd)) {
            if (code != NULL) *code = HYD_DIAG_CODE_SEGMENT_INVALID;
            return false;
        }
    }
    /* Derate ratio: 0 means default; otherwise must be strictly between 0 and 1. */
    if (segment->derateRatio != 0.0) {
        if (!isfinite(segment->derateRatio) ||
            segment->derateRatio <= 0.0 ||
            segment->derateRatio >= 1.0) {
            if (code != NULL) *code = HYD_DIAG_CODE_SEGMENT_INVALID;
            return false;
        }
    }
```

确保 `src/recipe_validator.c` 顶部已 `#include <math.h>`（用于 `isfinite`）。

- [ ] **Step 1.6: 在 `tests/test_recipe_validator.c` 追加 ceiling 字段早失败用例**

先看现有测试结构：

```bash
grep -n "^static void test_\|^int main" tests/test_recipe_validator.c | head -20
```

在 `main()` 之前、最末一个 `static void test_*` 之后追加：

```c
static void test_invalid_ceiling_tolerance_rejected(void) {
    HYD_MotionSegment seg;
    HYD_DiagnosticCode code = HYD_DIAG_CODE_NONE;
    memset(&seg, 0, sizeof(seg));
    seg.segmentTag = 1;
    seg.segmentType = HYD_SEGMENT_TYPE_CLAMPING;
    seg.mode = HYD_MODE_POSITION;
    seg.planner = HYD_PLANNER_TIME_BASED;
    seg.endCondition = HYD_END_POSITION;
    seg.direction = HYD_DIRECTION_EXTEND;
    seg.targetPosition = 100.0;
    seg.maxVelocity = 50.0;
    seg.maxAcceleration = 200.0;
    seg.maxDeceleration = 200.0;
    seg.maxFlow = 30.0;
    seg.velocityToFlowGain = 0.25;
    seg.positionTolerance = 0.5;

    /* Valid ceiling: passes */
    seg.pressureCeiling = 5.0;
    seg.pressureCeilingTolerance = 0.2;
    seg.pressureCeilingPositionStart = 70.0;
    seg.pressureCeilingPositionEnd = 100.0;
    assert(HYD_RecipeValidator_ValidateSegment(&seg, 0, &code));

    /* Negative tolerance: rejected */
    seg.pressureCeilingTolerance = -0.1;
    code = HYD_DIAG_CODE_NONE;
    assert(!HYD_RecipeValidator_ValidateSegment(&seg, 0, &code));
    assert(code == HYD_DIAG_CODE_SEGMENT_INVALID);

    /* Zero ceiling disables check — other fields irrelevant */
    seg.pressureCeiling = 0.0;
    seg.pressureCeilingTolerance = -1.0;  /* invalid but ignored */
    code = HYD_DIAG_CODE_NONE;
    assert(HYD_RecipeValidator_ValidateSegment(&seg, 0, &code));

    printf("test_invalid_ceiling_tolerance_rejected PASSED\n");
}

static void test_invalid_derate_ratio_rejected(void) {
    HYD_MotionSegment seg;
    HYD_DiagnosticCode code = HYD_DIAG_CODE_NONE;
    memset(&seg, 0, sizeof(seg));
    seg.segmentTag = 1;
    seg.segmentType = HYD_SEGMENT_TYPE_CLAMPING;
    seg.mode = HYD_MODE_POSITION;
    seg.planner = HYD_PLANNER_TIME_BASED;
    seg.endCondition = HYD_END_POSITION;
    seg.direction = HYD_DIRECTION_EXTEND;
    seg.targetPosition = 100.0;
    seg.maxVelocity = 50.0;
    seg.maxAcceleration = 200.0;
    seg.maxDeceleration = 200.0;
    seg.maxFlow = 30.0;
    seg.velocityToFlowGain = 0.25;
    seg.positionTolerance = 0.5;

    /* 0.0 = use default → passes */
    seg.derateRatio = 0.0;
    assert(HYD_RecipeValidator_ValidateSegment(&seg, 0, &code));

    /* Valid range (0,1) → passes */
    seg.derateRatio = 0.3;
    assert(HYD_RecipeValidator_ValidateSegment(&seg, 0, &code));

    /* Out of range → rejected */
    seg.derateRatio = 1.5;
    code = HYD_DIAG_CODE_NONE;
    assert(!HYD_RecipeValidator_ValidateSegment(&seg, 0, &code));
    assert(code == HYD_DIAG_CODE_SEGMENT_INVALID);

    seg.derateRatio = -0.1;
    code = HYD_DIAG_CODE_NONE;
    assert(!HYD_RecipeValidator_ValidateSegment(&seg, 0, &code));
    assert(code == HYD_DIAG_CODE_SEGMENT_INVALID);

    printf("test_invalid_derate_ratio_rejected PASSED\n");
}
```

在 `main()` 中追加调用：

```c
    test_invalid_ceiling_tolerance_rejected();
    test_invalid_derate_ratio_rejected();
```

- [ ] **Step 1.7: 配置 + 构建 + 跑测试**

```bash
cmake --preset unixgcc
cmake --build --preset unixgcc 2>&1 | tail -20
ctest --test-dir out/build/unixgcc -R '^test_recipe_validator$' --output-on-failure
```

Expected：新追加的 2 个用例 PASS。其他既有验证用例不回归。

- [ ] **Step 1.8: 跑全量回归**

```bash
ctest --test-dir out/build/unixgcc --output-on-failure
```

Expected：全部 PASS。新字段只是结构追加，对未配置 ceiling 的现有段透明。

- [ ] **Step 1.9: 提交**

```bash
git add include/common_types.h include/segment_limits.h src/segment_limits.c \
        src/recipe_validator.c tests/test_recipe_validator.c
git commit -m "$(cat <<'EOF'
feat: add HYD_MotionSegment pressure-ceiling and derate fields

Introduces 5 new HYD_MotionSegment fields needed by Sprint 1 low-pressure
mold-protect support:
- pressureCeiling: MPa, 0 disables the soft upper bound
- pressureCeilingTolerance: hysteresis above ceiling before DERATE
- pressureCeilingPositionStart / End: activation window for the ceiling
- derateRatio: per-segment derate factor; 0 falls back to library 0.5

segment_limits.{h,c} exposes typed accessors including
HYD_Segment_PressureCeilingActiveAt() that checks both ceiling > 0
and current position inside the configured window.

recipe_validator early-rejects configurations that pass NaN, negative
tolerance, or out-of-range derate ratios.

Refs: Sprint 1 spec §1.1 (2026-05-21-code-review-and-roadmap-design.md)

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 2: 新增 PRESSURE_CEILING 诊断码（spec §1.2）

**目标：** 给压力软上限超限定义两级诊断：WARNING/DERATE（首次超限）和 FAULT/STOP（持续超限升级）。诊断表 + 枚举 + flag 位 + criteria 配置都要更新，确保现有诊断 pipeline（debounce / 滞回 / startup 抑制 / fault escalation）能直接复用。

**Files:**
- Modify: `include/common_types.h`（`HYD_DiagnosticCode` 枚举追加 2 个 + `HYD_DiagnosticFlag` 追加 2 位 + `HYD_DiagnosticInfo` 追加 2 个 BOOL 字段）
- Modify: `src/diagnostics.c`（`HYD_DIAGNOSTIC_SPECS` 表追加 2 条 + `HYD_Diagnostics_BuildFlagMask` + `HYD_Diagnostics_CodeToString`）
- Modify: `include/motion_control.h`（`HYD_MotionControlFB` 增加 `_pressureCeilingCriteria` + `_pressureCeilingCriteriaState`）
- Modify: `src/motion_control.c`（`HYD_MotionControlFB_Init` 初始化新 criteria + `RESET` 路径清空）
- Test: `tests/test_diagnostics.c`（追加 ceiling 码 spec 查询用例）

### Steps

- [ ] **Step 2.1: 在 `include/common_types.h` 的 `HYD_DiagnosticCode` 枚举末尾追加 2 项**

定位 `HYD_DiagnosticCode` 枚举（约第 124-147 行）。在 `HYD_DIAG_CODE_INTERNAL_ERROR` **之前**插入：

```c
    HYD_DIAG_CODE_PRESSURE_CEILING_EXCEEDED,
    HYD_DIAG_CODE_PRESSURE_CEILING_VIOLATED,
```

（之所以插在 INTERNAL_ERROR 前，是保留 INTERNAL_ERROR 作为最后一个码以方便未来阅读时一眼定位"通用兜底"。）

- [ ] **Step 2.2: 在 `HYD_DiagnosticFlag` 枚举追加 2 个 flag 位**

定位 `HYD_DiagnosticFlag`（约第 151-161 行）。在 `HYD_DIAG_FLAG_TIMESTAMP_ROLLBACK = 1U << 7` 后追加：

```c
,
    HYD_DIAG_FLAG_PRESSURE_CEILING_EXCEEDED = 1U << 8,
    HYD_DIAG_FLAG_PRESSURE_CEILING_VIOLATED = 1U << 9
```

确认 `HYD_DiagnosticFlags` typedef（约第 149 行）依然是 `HYD_UINT16`——16 位足够容纳 10 个 flag，无需扩宽。

- [ ] **Step 2.3: 在 `HYD_DiagnosticInfo` 结构体追加 2 个 BOOL 字段**

定位 `HYD_DiagnosticInfo`（约第 302-322 行）。在 `HYD_BOOL timestampRollback;` 之后追加：

```c
    HYD_BOOL pressureCeilingExceeded;
    HYD_BOOL pressureCeilingViolated;
```

- [ ] **Step 2.4: 在 `src/diagnostics.c` 的 `HYD_DIAGNOSTIC_SPECS` 表追加 2 条**

定位表（src/diagnostics.c 约第 15-148 行）。在 `HYD_DIAG_CODE_TIMESTAMP_ROLLBACK` 那一条之后、`HYD_DIAG_CODE_INTERNAL_ERROR` 之前插入：

```c
    {HYD_DIAG_CODE_PRESSURE_CEILING_EXCEEDED,
     HYD_DIAG_SEVERITY_WARNING,
     HYD_DIAG_SOURCE_EXECUTION,
     HYD_DIAG_RECOVERY_CHECK_COMMAND,
     HYD_PROTECTION_ACTION_DERATE,
     "Pressure exceeded segment soft ceiling"},
    {HYD_DIAG_CODE_PRESSURE_CEILING_VIOLATED,
     HYD_DIAG_SEVERITY_FAULT,
     HYD_DIAG_SOURCE_EXECUTION,
     HYD_DIAG_RECOVERY_RESTART_SEGMENT,
     HYD_PROTECTION_ACTION_STOP,
     "Pressure remained above ceiling beyond fault-escalation window"},
```

- [ ] **Step 2.5: 在 `HYD_Diagnostics_BuildFlagMask` 中追加 2 个 flag 位映射**

定位（src/diagnostics.c 约第 150-183 行）。在 `if (diagnostic->timestampRollback) { ... }` 之后追加：

```c
    if (diagnostic->pressureCeilingExceeded) {
        flags |= HYD_DIAG_FLAG_PRESSURE_CEILING_EXCEEDED;
    }
    if (diagnostic->pressureCeilingViolated) {
        flags |= HYD_DIAG_FLAG_PRESSURE_CEILING_VIOLATED;
    }
```

- [ ] **Step 2.6: 在 `HYD_Diagnostics_CodeToString` 中追加 2 个 case**

```bash
grep -n "HYD_Diagnostics_CodeToString" src/diagnostics.c
```

打开函数（约第 380-410 行的 switch）。在 `case HYD_DIAG_CODE_TIMESTAMP_ROLLBACK:` 之后、`case HYD_DIAG_CODE_INTERNAL_ERROR:` 之前追加：

```c
        case HYD_DIAG_CODE_PRESSURE_CEILING_EXCEEDED:
            return "PRESSURE_CEILING_EXCEEDED";
        case HYD_DIAG_CODE_PRESSURE_CEILING_VIOLATED:
            return "PRESSURE_CEILING_VIOLATED";
```

- [ ] **Step 2.7: 在 `HYD_MotionControlFB` 增加 ceiling-criteria 字段**

```bash
grep -n "_pressureCriteria\|_pressureCriteriaState" include/motion_control.h
```

定位现有 `HYD_DiagnosticCriteria _pressureCriteria;` + `HYD_DiagnosticCriteriaState _pressureCriteriaState;` 行。在它们之后追加同样两行用于 ceiling：

```c
    HYD_DiagnosticCriteria _pressureCeilingCriteria;
    HYD_DiagnosticCriteriaState _pressureCeilingCriteriaState;
```

- [ ] **Step 2.8: 在 `HYD_MotionControlFB_Init` 中初始化 ceiling criteria**

```bash
grep -n "HYD_MotionControlFB_Init\b" src/motion_control.c | head -5
```

打开 `HYD_MotionControlFB_Init`。`memset(fb, 0, sizeof(*fb))` 之后会陆续配置各 criteria。定位 `_pressureCriteria` 的配置位置（grep `_pressureCriteria` 一行附近），在那行下方仿照风格追加：

```c
    /* Pressure ceiling criteria mirrors _pressureCriteria but uses a shorter
     * debounce + faster fault escalation, since ceiling violations are
     * already "above the safe envelope" and should react more promptly. */
    HYD_DiagnosticCriteria_Init(&fb->_pressureCeilingCriteria);
    fb->_pressureCeilingCriteria.debounceTime = 0.05;          /* 50 ms — react faster than normal pressure deviation */
    fb->_pressureCeilingCriteria.startupSuppressTime = 0.10;
    fb->_pressureCeilingCriteria.enableStartupSuppress = true;
    fb->_pressureCeilingCriteria.faultEscalationDuration = 0.30; /* 300 ms above ceiling → escalate to FAULT/STOP */
    fb->_pressureCeilingCriteria.protectionAction = HYD_PROTECTION_ACTION_DERATE;
    HYD_DiagnosticCriteria_ResetState(&fb->_pressureCeilingCriteriaState);
```

如果 `HYD_DiagnosticCriteria_Init` 不存在，使用 `_pressureCriteria` 初始化处的等价模式（直接字段赋值）。先 grep `_pressureCriteria` 看现有初始化代码，把它的形状复制过来即可。

- [ ] **Step 2.9: RESET 路径无需修改**

`HYD_MotionControlFB_Reset` 内部已 `memset` 整个 FB；新增字段会一同清零。Init 在 memset 后重新配置 criteria，故 reset → init 后默认值正确。这一步只需**确认**（不需要写代码），grep 一遍：

```bash
grep -n "memset(fb, 0, sizeof" src/motion_control.c
```

Expected：看到 Init 与 SoftReset 都是 memset，确认无需额外清理。

- [ ] **Step 2.10: 在 `tests/test_diagnostics.c` 追加 ceiling spec 查询用例**

打开 `tests/test_diagnostics.c`（Sprint 0 Task 5 已创建）。在 `main()` 之前追加：

```c
static void test_diag_spec_returns_warning_for_pressure_ceiling_exceeded(void) {
    HYD_DiagnosticInfo info;
    memset(&info, 0, sizeof(info));
    HYD_Diagnostics_SetEvent(&info, HYD_DIAG_CODE_PRESSURE_CEILING_EXCEEDED, HYD_DIAG_SEVERITY_NONE);
    assert(info.code == HYD_DIAG_CODE_PRESSURE_CEILING_EXCEEDED);
    assert(info.severity == HYD_DIAG_SEVERITY_WARNING);
    assert(info.protectionAction == HYD_PROTECTION_ACTION_DERATE);
    printf("test_diag_spec_returns_warning_for_pressure_ceiling_exceeded PASSED\n");
}

static void test_diag_spec_returns_fault_for_pressure_ceiling_violated(void) {
    HYD_DiagnosticInfo info;
    memset(&info, 0, sizeof(info));
    HYD_Diagnostics_SetEvent(&info, HYD_DIAG_CODE_PRESSURE_CEILING_VIOLATED, HYD_DIAG_SEVERITY_NONE);
    assert(info.code == HYD_DIAG_CODE_PRESSURE_CEILING_VIOLATED);
    assert(info.severity == HYD_DIAG_SEVERITY_FAULT);
    assert(info.protectionAction == HYD_PROTECTION_ACTION_STOP);
    printf("test_diag_spec_returns_fault_for_pressure_ceiling_violated PASSED\n");
}

static void test_ceiling_flag_mask_round_trip(void) {
    HYD_DiagnosticInfo info;
    memset(&info, 0, sizeof(info));
    info.pressureCeilingExceeded = true;
    info.flags = HYD_Diagnostics_GetFlagMask(&info);
    assert((info.flags & HYD_DIAG_FLAG_PRESSURE_CEILING_EXCEEDED) != 0U);
    assert((info.flags & HYD_DIAG_FLAG_PRESSURE_CEILING_VIOLATED) == 0U);

    info.pressureCeilingViolated = true;
    info.flags = HYD_Diagnostics_GetFlagMask(&info);
    assert((info.flags & HYD_DIAG_FLAG_PRESSURE_CEILING_VIOLATED) != 0U);
    printf("test_ceiling_flag_mask_round_trip PASSED\n");
}
```

并在 `main()` 中追加：

```c
    test_diag_spec_returns_warning_for_pressure_ceiling_exceeded();
    test_diag_spec_returns_fault_for_pressure_ceiling_violated();
    test_ceiling_flag_mask_round_trip();
```

- [ ] **Step 2.11: 配置 + 构建 + 跑测试**

```bash
cmake --build --preset unixgcc 2>&1 | tail -20
ctest --test-dir out/build/unixgcc -R '^test_diagnostics$' --output-on-failure
```

Expected：3 个新用例 PASS。

- [ ] **Step 2.12: 全量回归**

```bash
ctest --test-dir out/build/unixgcc --output-on-failure
```

Expected：全部 PASS。诊断表追加 + 枚举追加都是非侵入式改动，已有测试不应受影响。

- [ ] **Step 2.13: 提交**

```bash
git add include/common_types.h include/motion_control.h src/diagnostics.c \
        src/motion_control.c tests/test_diagnostics.c
git commit -m "$(cat <<'EOF'
feat: add PRESSURE_CEILING diagnostic codes for low-pressure mold-protect

Introduces two new HYD_DiagnosticCode entries:
- PRESSURE_CEILING_EXCEEDED (WARNING, DERATE) — initial soft-ceiling breach
- PRESSURE_CEILING_VIOLATED (FAULT, STOP)     — escalation after sustained breach

Reuses the existing diagnostic-criteria pipeline (debounce / startup
suppress / fault escalation) by adding _pressureCeilingCriteria and
_pressureCeilingCriteriaState to HYD_MotionControlFB. Ceiling criteria
uses 50 ms debounce + 300 ms fault-escalation window, both faster than
the regular OVER_PRESSURE channel because ceiling already means
"outside the safe envelope".

Adds flag bits HYD_DIAG_FLAG_PRESSURE_CEILING_EXCEEDED / _VIOLATED and
the matching HYD_DiagnosticInfo bool fields; flag mask round-trip is
unit-tested.

Refs: Sprint 1 spec §1.2 (2026-05-21-code-review-and-roadmap-design.md)

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 3: 在所有 mode 下评估 ceiling（spec §1.3）

**目标：** 在 `HYD_UpdateExecutionDiagnostics` 中追加一段"ceiling 判据"，在 POSITION / SPEED_RAMP / PRESSURE_CLOSED_LOOP 三种 mode 下都生效。优先级顺序：timeout > pressureCeilingViolated > overPressure > pressureCeilingExceeded > underPressure > flowDeviation > positionDeviation > velocityDeviation（FAULT 级永远先于 WARNING 级）。

**Files:**
- Modify: `src/motion_control.c`（`HYD_UpdateExecutionDiagnostics` 追加 ceiling 评估 + 优先级修正）
- Test: `tests/test_mold_protect.c`（Task 6 创建，本 Task 在测试中先放一个单 mode 占位用例，验证 POSITION mode 下 ceiling 触发 DERATE，证明本 Task 的核心改动到位）

### Steps

- [ ] **Step 3.1: 阅读现有 `HYD_UpdateExecutionDiagnostics` 优先级块结构**

```bash
grep -n "priorityCode\|prioritySeverity\|HYD_DIAG_CODE_OVER_PRESSURE" src/motion_control.c
```

阅读约 1469-1500 行的优先级构建逻辑。当前顺序为 timeout → overPressure → underPressure → flowDeviation → positionDeviation → velocityDeviation。我们要把 `pressureCeilingViolated`（FAULT 级）插入 timeout 之后、overPressure 之前；把 `pressureCeilingExceeded`（WARNING 级）插入 overPressure 与 underPressure 之间——但 FAULT 级别永远优先 WARNING，所以更精确的次序是：

1. timeout (FAULT)
2. pressureCeilingViolated (FAULT)
3. overPressure(FAULT-escalated) (FAULT or WARNING)
4. pressureCeilingExceeded (WARNING)
5. underPressure (WARNING)
6. flowDeviation
7. positionDeviation
8. velocityDeviation

- [ ] **Step 3.2: 在 `HYD_UpdateExecutionDiagnostics` 顶部新增 ceiling 局部变量**

定位函数开头（约 1259 行）。在 `HYD_BOOL timeout = false;` 之后追加：

```c
    HYD_BOOL pressureCeilingExceeded = false;
    HYD_BOOL pressureCeilingViolated = false;
    HYD_REAL pressureCeilingValue = 0.0;
    HYD_REAL pressureCeilingTolerance = 0.0;
    HYD_DiagnosticResult ceilingResult;
    HYD_BOOL ceilingActive = false;
```

并在 `HYD_REAL velocityError = 0.0;` 之后追加：

```c
    HYD_REAL ceilingErrorValue = 0.0;  /* actual - ceiling, only meaningful when > 0 */
```

- [ ] **Step 3.3: 在"--- Pressure diagnostics ---"块之后追加 ceiling 评估块**

定位 `/* --- Position diagnostics --- */` 行（约 1411 行），在它**之前**插入：

```c
    /* --- Pressure ceiling diagnostics (Sprint 1 §1.3) ---
     * Evaluated under ALL HYD_ControlMode values whenever the segment
     * configures pressureCeiling > 0 AND current position is inside
     * pressureCeilingPositionStart..End window (or window is degenerate). */
    pressureCeilingValue = HYD_Segment_GetPressureCeiling(segment);
    if (pressureCeilingValue > 0.0) {
        pressureCeilingTolerance = HYD_Segment_GetPressureCeilingTolerance(segment);
        ceilingActive = HYD_Segment_PressureCeilingActiveAt(segment, fb->AXIS_REF.position);
        if (ceilingActive) {
            ceilingErrorValue = fb->AXIS_REF.pressure - (pressureCeilingValue + pressureCeilingTolerance);
            if (ceilingErrorValue > 0.0) {
                /* Drive the criteria pipeline manually since the standard
                 * pressure criteria is tied to the closed-loop reference,
                 * not to a fixed ceiling. */
                HYD_DiagnosticCriteriaState* st = &fb->_pressureCeilingCriteriaState;
                HYD_DiagnosticCriteria* cr = &fb->_pressureCeilingCriteria;
                HYD_BOOL isStartupCeiling = HYD_IsStartupSuppressActive(elapsed, cr->startupSuppressTime);
                HYD_BOOL suppressed = (cr->enableStartupSuppress && isStartupCeiling) || isSwitchPhase;

                if (!suppressed) {
                    if (!st->triggered) {
                        st->triggered = true;
                        st->triggerStartTime = fb->AXIS_REF.timestamp;
                        st->elapsedDuration = 0.0;
                    } else {
                        st->elapsedDuration = fb->AXIS_REF.timestamp - st->triggerStartTime;
                    }
                    pressureCeilingExceeded = true;
                    memset(&ceilingResult, 0, sizeof(ceilingResult));
                    ceilingResult.triggered = true;
                    ceilingResult.severity = HYD_DIAG_SEVERITY_WARNING;
                    /* Fault escalation when above ceiling for cr->faultEscalationDuration */
                    if (cr->faultEscalationDuration > 0.0 &&
                        st->elapsedDuration >= cr->faultEscalationDuration) {
                        pressureCeilingViolated = true;
                        ceilingResult.severity = HYD_DIAG_SEVERITY_FAULT;
                    }
                }
            } else {
                /* Pressure dropped back under ceiling: reset the criteria state.
                 * Hysteresis: only reset when the value is strictly below ceiling
                 * (without the tolerance margin) to avoid chatter on jitter. */
                if (fb->AXIS_REF.pressure < pressureCeilingValue) {
                    HYD_DiagnosticCriteria_ResetState(&fb->_pressureCeilingCriteriaState);
                }
            }
        } else {
            /* Outside the configured window: reset criteria so the next entry
             * starts fresh (don't carry duration across activations). */
            HYD_DiagnosticCriteria_ResetState(&fb->_pressureCeilingCriteriaState);
        }
    } else {
        HYD_DiagnosticCriteria_ResetState(&fb->_pressureCeilingCriteriaState);
    }
```

(注：`HYD_DiagnosticCriteriaState` 字段名 `triggered` / `triggerStartTime` / `elapsedDuration` 需要先 `grep -n "typedef struct.*HYD_DiagnosticCriteriaState" include/diagnostics_criteria.h` 确认。如名字不同，按实际结构调整三个字段名后即可。)

- [ ] **Step 3.4: 修改优先级构建块，加入 ceiling**

定位现有优先级 if-else if 链（约 1469-1494 行的 `if (timeout) ... else if (overPressure) ...`）。完整替换为：

```c
    /* Build priority diagnostic from active conditions.
     * Ordering: FAULT-level codes outrank WARNING-level codes; among same
     * severity, ceiling violations outrank legacy pressure deviation. */
    if (timeout) {
        priorityCode = HYD_DIAG_CODE_TIMEOUT;
        prioritySeverity = HYD_DIAG_SEVERITY_FAULT;
    } else if (pressureCeilingViolated) {
        priorityCode = HYD_DIAG_CODE_PRESSURE_CEILING_VIOLATED;
        prioritySeverity = HYD_DIAG_SEVERITY_FAULT;
    } else if (overPressure && pressureResult.severity == HYD_DIAG_SEVERITY_FAULT) {
        priorityCode = HYD_DIAG_CODE_OVER_PRESSURE;
        prioritySeverity = HYD_DIAG_SEVERITY_FAULT;
    } else if (pressureCeilingExceeded) {
        priorityCode = HYD_DIAG_CODE_PRESSURE_CEILING_EXCEEDED;
        prioritySeverity = HYD_DIAG_SEVERITY_WARNING;
    } else if (overPressure) {
        priorityCode = HYD_DIAG_CODE_OVER_PRESSURE;
        prioritySeverity = HYD_DIAG_SEVERITY_WARNING;
    } else if (underPressure) {
        priorityCode = HYD_DIAG_CODE_UNDER_PRESSURE;
        prioritySeverity = (pressureResult.severity == HYD_DIAG_SEVERITY_FAULT)
            ? HYD_DIAG_SEVERITY_FAULT : HYD_DIAG_SEVERITY_WARNING;
    } else if (flowDeviation) {
        priorityCode = HYD_DIAG_CODE_FLOW_DEVIATION;
        prioritySeverity = (flowResult.severity == HYD_DIAG_SEVERITY_FAULT)
            ? HYD_DIAG_SEVERITY_FAULT : HYD_DIAG_SEVERITY_WARNING;
    } else if (positionDeviation) {
        priorityCode = HYD_DIAG_CODE_POSITION_DEVIATION;
        prioritySeverity = (positionResult.severity == HYD_DIAG_SEVERITY_FAULT)
            ? HYD_DIAG_SEVERITY_FAULT : HYD_DIAG_SEVERITY_WARNING;
    } else if (velocityDeviation) {
        priorityCode = HYD_DIAG_CODE_VELOCITY_DEVIATION;
        prioritySeverity = (velocityResult.severity == HYD_DIAG_SEVERITY_FAULT)
            ? HYD_DIAG_SEVERITY_FAULT : HYD_DIAG_SEVERITY_WARNING;
    }
```

- [ ] **Step 3.5: 在 `HYD_UpdateExecutionDiagnostics` 末尾的 `fb->DIAGNOSTIC.xxx = ...` 块追加 ceiling 标志**

定位 `fb->DIAGNOSTIC.timeout = timeout;` 行（约 1507 行）。在其后追加：

```c
    fb->DIAGNOSTIC.pressureCeilingExceeded = pressureCeilingExceeded;
    fb->DIAGNOSTIC.pressureCeilingViolated = pressureCeilingViolated;
```

确保 `fb->DIAGNOSTIC.flags = HYD_Diagnostics_GetFlagMask(&fb->DIAGNOSTIC);` 这一行在所有 BOOL 赋值之后（已是当前结构，无需移动）。

- [ ] **Step 3.6: 加入对应 include**

`src/motion_control.c` 顶部应该已经包含 `segment_limits.h`（Sprint 0 Task 3 加过）。先 grep 确认：

```bash
grep -n "#include \"segment_limits.h\"" src/motion_control.c
```

如果没有，加在 `#include "motion_control.h"` 之后。

- [ ] **Step 3.7: 配置 + 构建（先不跑测试）**

```bash
cmake --preset unixgcc
cmake --build --preset unixgcc 2>&1 | tail -30
```

Expected：构建成功。若 `HYD_DiagnosticCriteriaState` 字段名与 Step 3.3 中的 `triggered` / `triggerStartTime` / `elapsedDuration` 不符，按真实字段名调整。

- [ ] **Step 3.8: 跑全量回归确认未破坏现有测试**

```bash
ctest --test-dir out/build/unixgcc --output-on-failure
```

Expected：全部 PASS。如有失败，最可能是 priority 顺序变化导致某个旧测试断言旧 priorityCode，按实际新 priorityCode 调整测试断言。

- [ ] **Step 3.9: 创建占位测试 `tests/test_mold_protect.c` 覆盖 POSITION mode**

此处仅放一个最小用例验证 Task 3 的"所有 mode 下评估"已生效；Task 6 会扩展为端到端测试。

```c
/* tests/test_mold_protect.c - Sprint 1 low-pressure mold-protect tests */
#include "motion_control.h"
#include "action_profile.h"
#include "segment_limits.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static void prime_clamp_close_with_ceiling(HYD_MotionControlFB* fb,
                                           HYD_REAL ceiling,
                                           HYD_REAL ceilingTol,
                                           HYD_REAL windowStart,
                                           HYD_REAL windowEnd) {
    HYD_MotionFBParams params;
    HYD_MotionSegment seg;

    memset(&params, 0, sizeof(params));
    params.maxFlow = 30.0;
    params.maxVelocity = 50.0;
    params.maxAcceleration = 200.0;
    params.maxDeceleration = 200.0;
    params.velocityToFlowGain = 0.25;
    params.positionTolerance = 0.5;
    params.pressureTolerance = 0.2;

    assert(HYD_ActionProfile_BuildClampClose(&seg, &params, 1, 100.0));
    seg.pressureCeiling = ceiling;
    seg.pressureCeilingTolerance = ceilingTol;
    seg.pressureCeilingPositionStart = windowStart;
    seg.pressureCeilingPositionEnd = windowEnd;

    assert(HYD_MotionControlFB_LoadRecipe(fb, &seg, 1));
    fb->USE_RECIPE = true;
    fb->FLOW_TO_PUMP_SPEED_GAIN = 1.0;
    fb->PUMP_SPEED_LIMIT = 5000.0;
}

/* Task 3 placeholder: prove POSITION-mode ceiling detection raises
 * PRESSURE_CEILING_EXCEEDED with DERATE. Full DERATE→STOP escalation
 * lives in Task 6 test_mold_protect_stop_escalation. */
static void test_ceiling_exceeded_in_position_mode(void) {
    HYD_MotionControlFB fb;
    HYD_MotionControlFB_Init(&fb);
    prime_clamp_close_with_ceiling(&fb, /*ceiling*/ 5.0, /*tol*/ 0.2,
                                   /*windowStart*/ 70.0, /*windowEnd*/ 100.0);

    fb.AXIS_REF.timestamp = 0.0;
    fb.AXIS_REF.position = 0.0;
    fb.AXIS_REF.pressure = 1.0;
    assert(HYD_MotionControlFB_StartSegment(&fb, 0, 0.0));

    /* Step 1: outside the protect window — ceiling not active even if pressure spikes */
    for (int i = 0; i < 10; i++) {
        fb.AXIS_REF.timestamp = (HYD_TIME)(i * 0.01);
        fb.AXIS_REF.position = 30.0;          /* well below windowStart=70 */
        fb.AXIS_REF.pressure = 10.0;          /* way above ceiling */
        HYD_MotionControlFB_Execute(&fb);
        assert(!fb.DIAGNOSTIC.pressureCeilingExceeded);
    }

    /* Step 2: enter window with pressure above ceiling+tol — should trigger after debounce */
    for (int i = 0; i < 30; i++) {
        fb.AXIS_REF.timestamp = (HYD_TIME)(0.10 + i * 0.01);
        fb.AXIS_REF.position = 80.0;          /* inside [70, 100] */
        fb.AXIS_REF.pressure = 6.0;           /* > ceiling(5) + tol(0.2) */
        HYD_MotionControlFB_Execute(&fb);
    }
    assert(fb.DIAGNOSTIC.pressureCeilingExceeded);
    assert(fb.DIAGNOSTIC.code == HYD_DIAG_CODE_PRESSURE_CEILING_EXCEEDED);
    assert(fb.DIAGNOSTIC.severity == HYD_DIAG_SEVERITY_WARNING);
    assert(fb.DIAGNOSTIC.protectionAction == HYD_PROTECTION_ACTION_DERATE);

    printf("test_ceiling_exceeded_in_position_mode PASSED\n");
}

int main(void) {
    test_ceiling_exceeded_in_position_mode();
    return 0;
}
```

- [ ] **Step 3.10: 注册测试**

在 `CMakeLists.txt` 测试注册区（参照 Sprint 0 Task 5 的 `add_executable(test_state_reporter ...)` 块）追加：

```cmake
add_executable(test_mold_protect tests/test_mold_protect.c)
target_link_libraries(test_mold_protect PRIVATE HydroMotionLib)
add_test(NAME test_mold_protect
         COMMAND test_mold_protect
         WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
```

- [ ] **Step 3.11: 构建 + 跑测试**

```bash
cmake --preset unixgcc
cmake --build --preset unixgcc --target test_mold_protect
ctest --test-dir out/build/unixgcc -R '^test_mold_protect$' --output-on-failure
```

Expected：PASS。

如果 FAIL：
- 检查 `_pressureCeilingCriteria.debounceTime = 0.05` 是否被实际 criteria 实现作为"持续超过 0.05 s 才触发"使用，如果是的话需要让步骤循环跑足 6+ 次（已经跑 30 次满足）；
- 检查 `HYD_IsStartupSuppressActive(elapsed, 0.10)` 与外层 `elapsed = timestamp - segmentStartTime`，确认在 t=0.10 之后已经退出 startup 抑制窗。

- [ ] **Step 3.12: 跑全量回归**

```bash
ctest --test-dir out/build/unixgcc --output-on-failure
```

Expected：全部 PASS。

- [ ] **Step 3.13: 提交**

```bash
git add src/motion_control.c tests/test_mold_protect.c CMakeLists.txt
git commit -m "$(cat <<'EOF'
feat: evaluate pressureCeiling diagnostic across all control modes

HYD_UpdateExecutionDiagnostics now evaluates the pressureCeiling channel
under HYD_MODE_POSITION / SPEED_RAMP / PRESSURE_CLOSED_LOOP whenever
segment->pressureCeiling > 0 and AXIS_REF.position lies inside the
configured ceiling window. Previously only the OVER_PRESSURE channel
ran, and only in PRESSURE_CLOSED_LOOP — blocking low-pressure
mold-protect from any clamping segment.

- Driver loop manually advances _pressureCeilingCriteriaState because
  the legacy criteria pipeline references the closed-loop pressure
  reference, not a fixed ceiling
- WARNING (DERATE) on initial breach; FAULT (STOP) after
  cr->faultEscalationDuration of sustained breach
- Hysteresis: reset criteria state only when pressure dips strictly
  below ceiling (not ceiling+tol), to avoid chatter
- Priority order updated so PRESSURE_CEILING_VIOLATED outranks
  OVER_PRESSURE_FAULT but PRESSURE_CEILING_EXCEEDED ranks below
  OVER_PRESSURE_FAULT — FAULT codes always outrank WARNING codes

Placeholder test_mold_protect.c covers POSITION-mode trigger only;
Task 6 will extend it with DERATE→STOP escalation end-to-end.

Refs: Sprint 1 spec §1.3 (2026-05-21-code-review-and-roadmap-design.md)

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 4: derateRatio 从硬编码 0.5 改为段配置（spec §1.4）

**目标：** `HYD_OutputLimiter_Execute` 已支持 `input->derateRatio`（Sprint 0 之前就有），但 `motion_control.c:1628` 硬编码 `limiterInput.derateRatio = 0.5`。改为从段配置读取，0 回退为 0.5。Task 1 已在段结构追加 `derateRatio` 字段并加了 validator——本 Task 把它接通到 limiter。

**Files:**
- Modify: `src/motion_control.c`（删除硬编码 0.5，改为读 `HYD_Segment_GetDerateRatio(segment)`）
- Test: `tests/test_output_limiter.c`（追加 derate ratio 来自段配置的验证）

### Steps

- [ ] **Step 4.1: 阅读现有测试结构**

```bash
grep -n "^static void test_\|^int main" tests/test_output_limiter.c | head -10
grep -n "derateRatio" tests/test_output_limiter.c | head -5
```

Expected：已有测试覆盖默认 0.5 derate 行为。新追加的测试要在外部传入不同 derateRatio 来确认实际生效。

- [ ] **Step 4.2: 修改 `src/motion_control.c` 第 1628 行附近**

定位：

```bash
grep -n "limiterInput.derateRatio = 0.5" src/motion_control.c
```

Expected：1 处（约 1628 行）。完整替换：

```c
    /* Per-segment derate ratio: 0 falls back to library default (0.5) inside
     * HYD_OutputLimiter_Execute. See HYD_Segment_GetDerateRatio + segment->derateRatio. */
    limiterInput.derateRatio = HYD_Segment_GetDerateRatio(segment);
```

- [ ] **Step 4.3: 追加测试 `test_segment_derate_ratio_overrides_default`**

在 `tests/test_output_limiter.c` 中追加：

```c
static void test_segment_derate_ratio_overrides_default(void) {
    HYD_OutputLimiterInput in;
    HYD_OutputLimiterOutput out;

    memset(&in, 0, sizeof(in));
    in.requestedFlow = 20.0;
    in.requestedPumpSpeed = 1000.0;
    in.flowToPumpSpeedGain = 50.0;
    in.pumpSpeedLimit = 5000.0;
    in.protectionAction = HYD_PROTECTION_ACTION_DERATE;

    /* derateRatio = 0 → default 0.5 */
    in.derateRatio = 0.0;
    HYD_OutputLimiter_Execute(&in, &out);
    assert(out.derated);
    assert(fabs(out.commandFlow - 10.0) < 1e-6);
    assert(fabs(out.pumpSpeed - 500.0) < 1e-6);

    /* derateRatio = 0.3 → 30% of requested */
    in.derateRatio = 0.3;
    HYD_OutputLimiter_Execute(&in, &out);
    assert(out.derated);
    assert(fabs(out.commandFlow - 6.0) < 1e-6);
    assert(fabs(out.pumpSpeed - 300.0) < 1e-6);

    /* derateRatio = 0.9 → 90% (gentle derate) */
    in.derateRatio = 0.9;
    HYD_OutputLimiter_Execute(&in, &out);
    assert(out.derated);
    assert(fabs(out.commandFlow - 18.0) < 1e-6);
    assert(fabs(out.pumpSpeed - 900.0) < 1e-6);

    /* derateRatio = 1.5 (out of range) → fall back to default 0.5 */
    in.derateRatio = 1.5;
    HYD_OutputLimiter_Execute(&in, &out);
    assert(out.derated);
    assert(fabs(out.commandFlow - 10.0) < 1e-6);

    printf("test_segment_derate_ratio_overrides_default PASSED\n");
}
```

在 `main()` 中追加调用。确保文件顶部已 `#include <math.h>` 与 `#include <assert.h>`。

- [ ] **Step 4.4: 构建 + 跑测试**

```bash
cmake --build --preset unixgcc 2>&1 | tail -10
ctest --test-dir out/build/unixgcc -R '^test_output_limiter$' --output-on-failure
```

Expected：PASS。

- [ ] **Step 4.5: 跑全量回归**

```bash
ctest --test-dir out/build/unixgcc --output-on-failure
```

Expected：全部 PASS。注意 `test_mold_protect`（Task 3 创建）现在会在 `RunRunningState` 内调用新的 `HYD_Segment_GetDerateRatio`——确认它在 segment 未配置 `derateRatio` 时仍回退到 0.5，因此 Task 3 测试断言不变。

- [ ] **Step 4.6: 提交**

```bash
git add src/motion_control.c tests/test_output_limiter.c
git commit -m "$(cat <<'EOF'
feat: source derateRatio from segment config instead of hardcoded 0.5

motion_control.c previously passed limiterInput.derateRatio = 0.5
unconditionally. Now reads from HYD_Segment_GetDerateRatio(segment),
which returns 0 when the segment does not configure a ratio — and the
limiter already falls back to 0.5 on 0/out-of-range inputs, so default
behavior is preserved.

This is the final wiring needed for mold-protect to slow the platen
to a per-segment-configured fraction (e.g. 0.2 = 20% of normal speed)
during ceiling-exceeded conditions, instead of the one-size-fits-all
50% rate.

Refs: Sprint 1 spec §1.4 (2026-05-21-code-review-and-roadmap-design.md)

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 5: 新增 BuildClampCloseWithMoldProtect builder（spec §1.5）

**目标：** 给工艺工程师一个一句调用的 builder：传入"低速护膜窗口起点 + 压力上限 + DERATE 比例"就构造完整的 clamp-close 段。建立"低压护膜"这一惯例的模板，让所有用户工程使用同一组默认参数。

**Files:**
- Modify: `include/action_profile.h`（追加 `HYD_ActionProfile_BuildClampCloseWithMoldProtect` 声明）
- Modify: `src/action_profile.c`（实现）
- Test: `tests/test_action_profile.c`（追加用例）

### Steps

- [ ] **Step 5.1: 在 `include/action_profile.h` 追加声明**

在 `HYD_ActionProfile_BuildClampClose` 声明之后追加：

```c
/*
 * Build a clamp-close segment with a low-pressure mold-protect envelope.
 *
 * targetPosition          — final mold-closed position (mm).
 * protectWindowStart      — position at which the protect window begins (mm).
 *                           Below this, normal clamp velocity is used.
 * pressureCeiling         — soft upper bound during the window (MPa).
 * pressureCeilingTolerance — hysteresis above ceiling before DERATE (MPa);
 *                           pass 0 to fall back to params->pressureTolerance.
 * derateRatio             — fraction of normal output flow to use when ceiling
 *                           is exceeded (0,1). Pass 0 to use library default.
 *
 * Returns false on NULL pointers, invalid params, or invalid arguments.
 * Window ends at targetPosition (inclusive). To reuse outside clamping
 * (e.g. injection mold protect), use BuildClampClose + manual field set.
 */
HYD_BOOL HYD_ActionProfile_BuildClampCloseWithMoldProtect(HYD_MotionSegment* segment,
                                                          const HYD_MotionFBParams* params,
                                                          HYD_UINT8 segmentTag,
                                                          HYD_REAL targetPosition,
                                                          HYD_REAL protectWindowStart,
                                                          HYD_REAL pressureCeiling,
                                                          HYD_REAL pressureCeilingTolerance,
                                                          HYD_REAL derateRatio);
```

- [ ] **Step 5.2: 在 `src/action_profile.c` 追加实现**

在 `HYD_ActionProfile_BuildClampClose` 实现之后追加：

```c
HYD_BOOL HYD_ActionProfile_BuildClampCloseWithMoldProtect(HYD_MotionSegment* segment,
                                                          const HYD_MotionFBParams* params,
                                                          HYD_UINT8 segmentTag,
                                                          HYD_REAL targetPosition,
                                                          HYD_REAL protectWindowStart,
                                                          HYD_REAL pressureCeiling,
                                                          HYD_REAL pressureCeilingTolerance,
                                                          HYD_REAL derateRatio) {
    if (segment == NULL || pressureCeiling <= 0.0 ||
        protectWindowStart >= targetPosition) {
        return false;
    }
    /* derateRatio must be 0 (use default) or strictly in (0, 1). */
    if (derateRatio != 0.0 && (derateRatio <= 0.0 || derateRatio >= 1.0)) {
        return false;
    }
    if (!HYD_ActionProfile_BuildClampClose(segment, params, segmentTag, targetPosition)) {
        return false;
    }
    segment->pressureCeiling = pressureCeiling;
    segment->pressureCeilingTolerance = pressureCeilingTolerance;  /* 0 → use pressureTolerance via getter */
    segment->pressureCeilingPositionStart = protectWindowStart;
    segment->pressureCeilingPositionEnd = targetPosition;
    segment->derateRatio = derateRatio;
    return true;
}
```

- [ ] **Step 5.3: 在 `tests/test_action_profile.c` 追加用例**

先看现有测试风格：

```bash
grep -n "^static void test_\|^int main" tests/test_action_profile.c | head -20
```

在 `main()` 之前追加：

```c
static void test_build_clamp_close_with_mold_protect_populates_window(void) {
    HYD_MotionFBParams params;
    HYD_MotionSegment seg;

    memset(&params, 0, sizeof(params));
    params.maxFlow = 30.0;
    params.maxVelocity = 50.0;
    params.maxAcceleration = 200.0;
    params.maxDeceleration = 200.0;
    params.velocityToFlowGain = 0.25;
    params.positionTolerance = 0.5;
    params.pressureTolerance = 0.3;  /* fallback when ceilingTolerance=0 */

    assert(HYD_ActionProfile_BuildClampCloseWithMoldProtect(
        &seg, &params, 1, /*targetPosition*/ 100.0,
        /*protectWindowStart*/ 70.0,
        /*pressureCeiling*/ 5.0,
        /*ceilingTolerance*/ 0.0,  /* should fall back */
        /*derateRatio*/ 0.2));

    assert(seg.segmentTag == 1);
    assert(seg.segmentType == HYD_SEGMENT_TYPE_CLAMPING);
    assert(seg.mode == HYD_MODE_POSITION);
    assert(seg.endCondition == HYD_END_POSITION);
    assert(seg.direction == HYD_DIRECTION_EXTEND);
    assert(seg.pressureCeiling == 5.0);
    assert(seg.pressureCeilingPositionStart == 70.0);
    assert(seg.pressureCeilingPositionEnd == 100.0);
    assert(seg.derateRatio == 0.2);

    /* ceilingTolerance=0 in struct; getter falls back to pressureTolerance */
    assert(HYD_Segment_GetPressureCeilingTolerance(&seg) == 0.3);

    printf("test_build_clamp_close_with_mold_protect_populates_window PASSED\n");
}

static void test_build_clamp_close_with_mold_protect_rejects_bad_args(void) {
    HYD_MotionFBParams params;
    HYD_MotionSegment seg;

    memset(&params, 0, sizeof(params));
    params.maxFlow = 30.0;
    params.maxVelocity = 50.0;
    params.maxAcceleration = 200.0;
    params.maxDeceleration = 200.0;
    params.velocityToFlowGain = 0.25;

    /* window start >= target */
    assert(!HYD_ActionProfile_BuildClampCloseWithMoldProtect(
        &seg, &params, 1, 100.0, 100.0, 5.0, 0.2, 0.2));
    assert(!HYD_ActionProfile_BuildClampCloseWithMoldProtect(
        &seg, &params, 1, 100.0, 120.0, 5.0, 0.2, 0.2));

    /* ceiling <= 0 */
    assert(!HYD_ActionProfile_BuildClampCloseWithMoldProtect(
        &seg, &params, 1, 100.0, 70.0, 0.0, 0.2, 0.2));
    assert(!HYD_ActionProfile_BuildClampCloseWithMoldProtect(
        &seg, &params, 1, 100.0, 70.0, -1.0, 0.2, 0.2));

    /* derateRatio out of (0, 1) range — except 0 which is "use default" */
    assert(!HYD_ActionProfile_BuildClampCloseWithMoldProtect(
        &seg, &params, 1, 100.0, 70.0, 5.0, 0.2, 1.0));
    assert(!HYD_ActionProfile_BuildClampCloseWithMoldProtect(
        &seg, &params, 1, 100.0, 70.0, 5.0, 0.2, -0.1));

    /* derateRatio = 0 is accepted (means "use default") */
    assert(HYD_ActionProfile_BuildClampCloseWithMoldProtect(
        &seg, &params, 1, 100.0, 70.0, 5.0, 0.2, 0.0));
    assert(seg.derateRatio == 0.0);  /* preserved as 0; runtime resolves */

    printf("test_build_clamp_close_with_mold_protect_rejects_bad_args PASSED\n");
}
```

在 `main()` 中追加调用：

```c
    test_build_clamp_close_with_mold_protect_populates_window();
    test_build_clamp_close_with_mold_protect_rejects_bad_args();
```

确保 `tests/test_action_profile.c` 顶部 `#include "segment_limits.h"`（用于 `HYD_Segment_GetPressureCeilingTolerance`）。若已存在则跳过。

- [ ] **Step 5.4: 构建 + 跑测试**

```bash
cmake --build --preset unixgcc 2>&1 | tail -10
ctest --test-dir out/build/unixgcc -R '^test_action_profile$' --output-on-failure
```

Expected：PASS。

- [ ] **Step 5.5: 跑全量回归**

```bash
ctest --test-dir out/build/unixgcc --output-on-failure
```

Expected：全部 PASS。

- [ ] **Step 5.6: 提交**

```bash
git add include/action_profile.h src/action_profile.c tests/test_action_profile.c
git commit -m "$(cat <<'EOF'
feat: add HYD_ActionProfile_BuildClampCloseWithMoldProtect template builder

Wraps the bare BuildClampClose with the four mold-protect segment fields
(pressureCeiling, ceilingTolerance, ceilingPositionStart/End, derateRatio)
so process engineers can author a low-pressure clamp segment in one call:

    HYD_ActionProfile_BuildClampCloseWithMoldProtect(
        &seg, &params, tag, /*target*/ 100.0,
        /*protectWindowStart*/ 70.0, /*ceiling*/ 5.0,
        /*ceilingTol*/ 0.0 /* fall back to pressureTolerance */,
        /*derateRatio*/ 0.2);

Validates:
- protectWindowStart < targetPosition (otherwise window is degenerate or
  inverted)
- pressureCeiling > 0
- derateRatio is either 0 (use default) or strictly in (0, 1)

Window-end is fixed to targetPosition; callers needing arbitrary window
geometry must populate the four fields manually after BuildClampClose.

Refs: Sprint 1 spec §1.5 (2026-05-21-code-review-and-roadmap-design.md)

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 6: 端到端集成测试 test_mold_protect.c 扩展（spec §1.6）

**目标：** 在 Task 3 的占位测试基础上扩充，覆盖完整工艺路径：
1. 合模段进入护膜窗口前正常运行；
2. 模具内异物 → 压力提前升高 → ceiling exceeded → DERATE（commanded flow 与 PUMP_SPEED 按 derateRatio 降低）；
3. 异物未排除 → 持续超限 → fault escalation → STOP（FB_STATE 进入 FAULT，PUMP_SPEED 归零，protectionAction = STOP）；
4. 异物排除 → 压力回落 → 通过 Abort+restart 序列恢复（依赖 Sprint 0 C-3 的 FAULT→ABORT 路径）。

**Files:**
- Modify: `tests/test_mold_protect.c`（追加 3 个用例：DERATE 行为、STOP 升级、SPEED_RAMP mode 同样触发）

### Steps

- [ ] **Step 6.1: 在 `tests/test_mold_protect.c` 中追加 helper 与三个用例**

在文件顶部 `prime_clamp_close_with_ceiling` 之后追加一个驱动 helper：

```c
static void tick(HYD_MotionControlFB* fb, HYD_REAL t, HYD_REAL position, HYD_REAL pressure) {
    fb->AXIS_REF.timestamp = t;
    fb->AXIS_REF.position = position;
    fb->AXIS_REF.pressure = pressure;
    fb->AXIS_REF.flow = fabs(fb->AXIS_REF.velocity) * 0.25;  /* match velocityToFlowGain */
    HYD_MotionControlFB_Execute(fb);
}
```

确认顶部已包含 `<math.h>`。然后追加：

```c
static void test_mold_protect_derate_reduces_pump_speed(void) {
    HYD_MotionControlFB fb;
    HYD_MotionControlFB_Init(&fb);
    /* derateRatio = 0.2 to make the reduction obvious */
    HYD_MotionFBParams params;
    HYD_MotionSegment seg;
    memset(&params, 0, sizeof(params));
    params.maxFlow = 30.0;
    params.maxVelocity = 50.0;
    params.maxAcceleration = 200.0;
    params.maxDeceleration = 200.0;
    params.velocityToFlowGain = 0.25;
    params.positionTolerance = 0.5;
    params.pressureTolerance = 0.3;
    assert(HYD_ActionProfile_BuildClampCloseWithMoldProtect(
        &seg, &params, 1, /*targetPosition*/ 100.0,
        /*windowStart*/ 70.0, /*ceiling*/ 5.0, /*ceilingTol*/ 0.2,
        /*derate*/ 0.2));
    assert(HYD_MotionControlFB_LoadRecipe(&fb, &seg, 1));
    fb.USE_RECIPE = true;
    fb.FLOW_TO_PUMP_SPEED_GAIN = 100.0;  /* 1 L/min ≈ 100 rpm */
    fb.PUMP_SPEED_LIMIT = 5000.0;

    assert(HYD_MotionControlFB_StartSegment(&fb, 0, 0.0));

    /* Phase 1: well below protect window — normal pump speed */
    HYD_REAL normalPumpSpeed = 0.0;
    for (int i = 0; i < 20; i++) {
        tick(&fb, 0.01 * (i + 1), 30.0 + i * 0.5, 1.0);
        if (i > 10) {
            normalPumpSpeed = fb.PUMP_SPEED;  /* sample once warmed up */
        }
    }
    assert(normalPumpSpeed > 0.0);
    assert(!fb.DIAGNOSTIC.pressureCeilingExceeded);

    /* Phase 2: enter window with pressure above ceiling — DERATE kicks in */
    HYD_REAL deratedPumpSpeed = 0.0;
    for (int i = 0; i < 30; i++) {
        tick(&fb, 0.20 + 0.01 * (i + 1), 80.0 + i * 0.1, 6.0);
        if (fb.DIAGNOSTIC.pressureCeilingExceeded && i > 10) {
            deratedPumpSpeed = fb.PUMP_SPEED;
        }
    }
    assert(fb.DIAGNOSTIC.pressureCeilingExceeded);
    assert(fb.DIAGNOSTIC.code == HYD_DIAG_CODE_PRESSURE_CEILING_EXCEEDED);
    assert(fb.DIAGNOSTIC.protectionAction == HYD_PROTECTION_ACTION_DERATE);
    assert(deratedPumpSpeed > 0.0);
    /* Derate by 0.2 → derated speed should be ≤ 0.25 * normal (allow a small jitter margin) */
    assert(deratedPumpSpeed < normalPumpSpeed * 0.25);

    printf("test_mold_protect_derate_reduces_pump_speed PASSED\n");
}

static void test_mold_protect_escalates_to_stop(void) {
    HYD_MotionControlFB fb;
    HYD_MotionControlFB_Init(&fb);
    HYD_MotionFBParams params;
    HYD_MotionSegment seg;
    memset(&params, 0, sizeof(params));
    params.maxFlow = 30.0;
    params.maxVelocity = 50.0;
    params.maxAcceleration = 200.0;
    params.maxDeceleration = 200.0;
    params.velocityToFlowGain = 0.25;
    params.positionTolerance = 0.5;
    params.pressureTolerance = 0.3;
    assert(HYD_ActionProfile_BuildClampCloseWithMoldProtect(
        &seg, &params, 1, 100.0, 70.0, 5.0, 0.2, 0.2));
    assert(HYD_MotionControlFB_LoadRecipe(&fb, &seg, 1));
    fb.USE_RECIPE = true;
    fb.FLOW_TO_PUMP_SPEED_GAIN = 100.0;
    fb.PUMP_SPEED_LIMIT = 5000.0;

    assert(HYD_MotionControlFB_StartSegment(&fb, 0, 0.0));

    /* Hold position inside window with sustained over-ceiling pressure */
    for (int i = 0; i < 200; i++) {  /* 200 * 10 ms = 2 s → exceeds 0.3 s escalation */
        tick(&fb, 0.01 * (i + 1), 80.0, 6.5);
        if (fb.FB_STATE == HYD_FB_STATE_FAULT) {
            break;
        }
    }
    assert(fb.FB_STATE == HYD_FB_STATE_FAULT);
    assert(fb.DIAGNOSTIC.code == HYD_DIAG_CODE_PRESSURE_CEILING_VIOLATED);
    assert(fb.DIAGNOSTIC.severity == HYD_DIAG_SEVERITY_FAULT);
    assert(fb.DIAGNOSTIC.protectionAction == HYD_PROTECTION_ACTION_STOP);
    assert(fb.PUMP_SPEED == 0.0);
    assert(fb.STATE.faultActive);

    /* Recovery via Abort (Sprint 0 C-3 path) */
    assert(HYD_MotionControlFB_Abort(&fb));
    HYD_MotionControlFB_Execute(&fb);
    assert(fb.FB_STATE == HYD_FB_STATE_ABORTED);
    assert(!fb.STATE.faultActive);

    printf("test_mold_protect_escalates_to_stop PASSED\n");
}

static void test_mold_protect_applies_under_speed_ramp_mode(void) {
    /* Verify Task 3 contract: ceiling fires under HYD_MODE_SPEED_RAMP too. */
    HYD_MotionControlFB fb;
    HYD_MotionControlFB_Init(&fb);
    HYD_MotionFBParams params;
    HYD_MotionSegment seg;
    memset(&params, 0, sizeof(params));
    params.maxFlow = 30.0;
    params.maxVelocity = 50.0;
    params.maxAcceleration = 200.0;
    params.maxDeceleration = 200.0;
    params.velocityToFlowGain = 0.25;
    params.velocityTolerance = 1.0;
    params.pressureTolerance = 0.3;
    assert(HYD_ActionProfile_BuildInjectionFill(&seg, &params, 2, /*transferPos*/ 100.0));
    /* Manually attach ceiling fields (no dedicated builder for injection-protect yet) */
    seg.pressureCeiling = 8.0;
    seg.pressureCeilingTolerance = 0.2;
    seg.pressureCeilingPositionStart = 70.0;
    seg.pressureCeilingPositionEnd = 100.0;
    seg.derateRatio = 0.4;

    assert(HYD_MotionControlFB_LoadRecipe(&fb, &seg, 1));
    fb.USE_RECIPE = true;
    fb.FLOW_TO_PUMP_SPEED_GAIN = 100.0;
    fb.PUMP_SPEED_LIMIT = 5000.0;
    assert(HYD_MotionControlFB_StartSegment(&fb, 0, 0.0));

    for (int i = 0; i < 40; i++) {
        tick(&fb, 0.01 * (i + 1), 80.0, 9.5);  /* > ceiling 8 + tol 0.2 */
    }
    assert(fb.DIAGNOSTIC.pressureCeilingExceeded);
    assert(fb.DIAGNOSTIC.code == HYD_DIAG_CODE_PRESSURE_CEILING_EXCEEDED);
    assert(fb.DIAGNOSTIC.protectionAction == HYD_PROTECTION_ACTION_DERATE);

    printf("test_mold_protect_applies_under_speed_ramp_mode PASSED\n");
}
```

在 `main()` 中追加调用：

```c
    test_mold_protect_derate_reduces_pump_speed();
    test_mold_protect_escalates_to_stop();
    test_mold_protect_applies_under_speed_ramp_mode();
```

- [ ] **Step 6.2: 构建 + 跑测试**

```bash
cmake --build --preset unixgcc --target test_mold_protect
ctest --test-dir out/build/unixgcc -R '^test_mold_protect$' --output-on-failure
```

Expected：4 个用例全部 PASS（占位用例 + 3 个新用例）。

如果 FAIL：
- DERATE 比例不准：先检查 `tick()` 中 `fb->AXIS_REF.flow = fabs(velocity) * 0.25` 是否与 segment 的 velocityToFlowGain 一致；若 motion planner 的内部速度与 axis velocity 解耦，则用 `fb->_lastCommandedFlow` 对比更准；
- STOP 未发生：把 escalation 循环 i 上限拉到 500，逐 iter `printf("i=%d state=%d code=%d\n", i, fb.FB_STATE, fb.DIAGNOSTIC.code);` 观察是否在 0.30 s 后升级。

- [ ] **Step 6.3: 跑全量回归**

```bash
ctest --test-dir out/build/unixgcc --output-on-failure
```

Expected：全部 PASS。

- [ ] **Step 6.4: 提交**

```bash
git add tests/test_mold_protect.c
git commit -m "$(cat <<'EOF'
test: cover mold-protect DERATE→STOP escalation end-to-end

Extends test_mold_protect.c with three integration scenarios:

1. test_mold_protect_derate_reduces_pump_speed
   - Drives clamp axis through the protect window with pressure above
     ceiling
   - Verifies PUMP_SPEED drops to ~ derateRatio (0.2) of normal speed

2. test_mold_protect_escalates_to_stop
   - Holds pressure above ceiling beyond fault-escalation window
     (0.3 s default)
   - Verifies FB_STATE → FAULT, code → PRESSURE_CEILING_VIOLATED,
     protectionAction → STOP, PUMP_SPEED → 0
   - Confirms recovery via Abort (Sprint 0 C-3 path)

3. test_mold_protect_applies_under_speed_ramp_mode
   - Configures an injection-fill segment with manual ceiling fields
   - Verifies the SPEED_RAMP mode also evaluates ceiling (proves
     Task 3's "all modes" contract)

Refs: Sprint 1 spec §1.6 (2026-05-21-code-review-and-roadmap-design.md)

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 7: 文档同步（spec §1.7）

**目标：** 更新两份架构文档以反映 Sprint 1 引入的低压护膜原语：

- `docs/architecture/motion-profile-archetypes.md`：在"合模"档案下追加"低压护膜变体"章节；
- `docs/architecture/motion-runtime-contract.md`：在"诊断码"与"段字段契约"两节追加 ceiling/derateRatio 的契约描述。

并在 `docs/architecture/implementation-contract-gap-list.md` 顶部"Implemented Algorithm Gaps"块追加一行。

**Files:**
- Modify: `docs/architecture/motion-profile-archetypes.md`
- Modify: `docs/architecture/motion-runtime-contract.md`
- Modify: `docs/architecture/implementation-contract-gap-list.md`

### Steps

- [ ] **Step 7.1: 阅读现有 archetypes 文档结构**

```bash
grep -n "^#\|^##\|^###" docs/architecture/motion-profile-archetypes.md | head -30
```

Expected：列出各级标题。找到"合模 / Clamping"相关章节。

- [ ] **Step 7.2: 在 archetypes 中追加"低压护膜变体"章节**

定位 clamping 章节末尾（grep 下一个 `^## ` 行号定上界）。在 clamping 章节末尾插入：

```markdown
### 低压护膜变体 (Low-Pressure Mold Protect)

> Introduced in Sprint 1.

合模段进入护膜窗口后,允许"实测压力 > 段配置 ceiling + tolerance"立即触发 DERATE,
持续超限超过 fault-escalation 阈值后升级为 STOP。适用场景:模具内异物、模板平行度异常、
顶针未完全回位、嵌件未到位等。

**推荐 builder:**

```c
HYD_ActionProfile_BuildClampCloseWithMoldProtect(
    &seg, &params, tag,
    /* targetPosition */ 100.0,    /* mm, 全合位置 */
    /* protectWindowStart */ 70.0, /* mm, 低速护膜窗口起点 */
    /* pressureCeiling */ 5.0,     /* MPa, 护膜段压力上限 */
    /* pressureCeilingTolerance */ 0.0,  /* 0 = 沿用 params->pressureTolerance */
    /* derateRatio */ 0.2);        /* 触发 ceiling-exceeded 时减速到 20% */
```

**激活窗口语义:**

- `pressureCeilingPositionStart < pressureCeilingPositionEnd` → 窗口内激活
- `pressureCeilingPositionStart >= pressureCeilingPositionEnd`（含 0,0）→ 整段激活
- `pressureCeiling = 0` → 整段禁用 ceiling 检查(默认状态)

**诊断码:**

| Code | Severity | Protection Action |
|---|---|---|
| `HYD_DIAG_CODE_PRESSURE_CEILING_EXCEEDED` | WARNING | DERATE |
| `HYD_DIAG_CODE_PRESSURE_CEILING_VIOLATED` | FAULT | STOP |

EXCEEDED → VIOLATED 的升级由 `_pressureCeilingCriteria.faultEscalationDuration`
控制(默认 300 ms)。PLC 在 WARNING 阶段会观察到 PUMP_SPEED 按 derateRatio 缩放;
在 FAULT 阶段会观察到 FB_STATE = FAULT、PUMP_SPEED = 0,需要通过 Abort+重启或
Reset 恢复。

**与 OVER_PRESSURE 的区别:**

- `OVER_PRESSURE`(legacy)只在 PRESSURE_CLOSED_LOOP mode 评估,语义是"实测压力偏离
  closed-loop reference 超过 pressureTolerance",用于压力 servo 跟踪。
- `PRESSURE_CEILING_EXCEEDED`(Sprint 1)在 POSITION / SPEED_RAMP /
  PRESSURE_CLOSED_LOOP 任意 mode 都评估,语义是"实测压力超过段配置的固定软上限",
  用于安全保护。两者可以共存:一个 PRESSURE_CLOSED_LOOP 段同时配置
  `pressureTolerance`(跟踪误差告警)和 `pressureCeiling`(安全上限保护)是合法且推荐
  的工艺配置。

```

- [ ] **Step 7.3: 在 motion-runtime-contract 中追加段字段契约**

```bash
grep -n "^#\|^##\|^###" docs/architecture/motion-runtime-contract.md | head -40
```

定位"段字段"或"HYD_MotionSegment 字段约定"章节(若没有此章节,在文档末尾追加)。追加:

```markdown
### Pressure-ceiling fields (Sprint 1)

四个新字段定义"段内压力软上限 + 激活窗口":

| Field | Unit | Semantics |
|---|---|---|
| `pressureCeiling` | MPa | 0 disables; >0 enables ceiling check |
| `pressureCeilingTolerance` | MPa | hysteresis above ceiling; 0 falls back to `pressureTolerance` |
| `pressureCeilingPositionStart` | mm | window lower bound; ignored when End<=Start |
| `pressureCeilingPositionEnd` | mm | window upper bound; <=Start means always-on |

**Activation rule:**

```
activeAt(P) ⇐⇒ pressureCeiling > 0 ∧
               (End ≤ Start  ∨  (P ≥ Start ∧ P ≤ End))
```

**Trigger rule:**

```
exceeded ⇐⇒ activeAt(currentPosition) ∧
            (actualPressure > pressureCeiling + effectiveTolerance)
```

其中 `effectiveTolerance = pressureCeilingTolerance if > 0 else pressureTolerance`.

**Severity & action:**

- Initial breach (after `_pressureCeilingCriteria.debounceTime = 0.05 s`):
  `DIAGNOSTIC.code = PRESSURE_CEILING_EXCEEDED`, severity WARNING, action DERATE.
- Sustained breach (after `_pressureCeilingCriteria.faultEscalationDuration = 0.30 s`):
  `DIAGNOSTIC.code = PRESSURE_CEILING_VIOLATED`, severity FAULT, action STOP.
- Hysteresis: criteria state resets only when `actualPressure < pressureCeiling`
  (i.e. strictly below the ceiling, ignoring tolerance) to avoid chatter on jitter.

**Validation (recipe_validator):**

- `pressureCeiling > 0` 时 `pressureCeilingTolerance >= 0`(允许 0,回退到 pressureTolerance)且必须 finite。
- 位置字段必须 finite(不允许 NaN / -Inf);零是合法值(代表"always-on")。

### Derate-ratio field (Sprint 1)

| Field | Range | Semantics |
|---|---|---|
| `derateRatio` | 0 ∨ (0, 1) | 0 表示"使用库默认 0.5";其他取值即 DERATE 时的输出系数 |

`HYD_OutputLimiter_Execute` 在 `protectionAction = DERATE` 时把
`commandFlow` 与 `pumpSpeed` 都乘以 `derateRatio`。
取值 ≤0 / ≥1 / NaN 都被 `HYD_Segment_GetDerateRatio` 视为"未配置"并返回 0,
随后 `HYD_OutputLimiter_ResolveDerateRatio` 回退到 0.5。
```

- [ ] **Step 7.4: 在 contract 文档的诊断码章节追加 2 个新码**

定位"诊断码"或"DiagnosticCode"章节(grep)。追加表行:

```markdown
| `HYD_DIAG_CODE_PRESSURE_CEILING_EXCEEDED` | WARNING | EXECUTION | CHECK_COMMAND | DERATE | 段内压力超过软上限(初次) |
| `HYD_DIAG_CODE_PRESSURE_CEILING_VIOLATED` | FAULT   | EXECUTION | RESTART_SEGMENT | STOP   | 持续超限,触发 fault escalation |
```

(若文档原本不是表格风格,模仿原结构改写。)

- [ ] **Step 7.5: 更新 implementation-contract-gap-list**

在文档头部 `Implemented Algorithm Gaps`(或类似)块追加:

```markdown
- Pressure-ceiling check now runs under HYD_MODE_POSITION / SPEED_RAMP /
  PRESSURE_CLOSED_LOOP for any segment with `pressureCeiling > 0`
  (Sprint 1 §1.3). Two new diagnostic codes
  `HYD_DIAG_CODE_PRESSURE_CEILING_EXCEEDED` (WARNING/DERATE) and
  `HYD_DIAG_CODE_PRESSURE_CEILING_VIOLATED` (FAULT/STOP) emit with
  position-window-aware activation.
- `derateRatio` 改为段级可配置(`HYD_MotionSegment.derateRatio`),硬编码
  0.5 已从 motion_control.c 移除(Sprint 1 §1.4)。
- `HYD_ActionProfile_BuildClampCloseWithMoldProtect()` 新增,作为低压护
  膜工艺的一句调用模板(Sprint 1 §1.5)。
```

- [ ] **Step 7.6: 提交**

```bash
git add docs/architecture/motion-profile-archetypes.md \
        docs/architecture/motion-runtime-contract.md \
        docs/architecture/implementation-contract-gap-list.md
git commit -m "$(cat <<'EOF'
docs: log Sprint 1 low-pressure mold-protect contract

- motion-profile-archetypes.md: new "低压护膜变体" subsection under
  clamping; documents BuildClampCloseWithMoldProtect, activation window
  semantics, diagnostic codes, and the OVER_PRESSURE vs CEILING contrast
- motion-runtime-contract.md: HYD_MotionSegment pressureCeiling /
  pressureCeilingTolerance / pressureCeilingPositionStart /
  pressureCeilingPositionEnd / derateRatio fields; activation +
  trigger rules; severity/action matrix; validator constraints
- implementation-contract-gap-list.md: three new bullets covering
  Sprint 1 §1.3 / §1.4 / §1.5 implementation status

Refs: Sprint 1 spec §1.7 (2026-05-21-code-review-and-roadmap-design.md)

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## 收尾：Sprint 1 验收

- [ ] **Step 8.1: 跑全量测试**

```bash
ctest --test-dir out/build/unixgcc --output-on-failure
```

Expected：所有测试 PASS（包括 Sprint 0 留下的 + 本 Sprint 新增的 `test_mold_protect` 4 个用例 + `test_diagnostics` 增量 3 个用例 + `test_recipe_validator` 增量 2 个用例 + `test_output_limiter` 增量 1 个用例 + `test_action_profile` 增量 2 个用例）。

- [ ] **Step 8.2: 嵌入式生产构建验证**

```bash
./scripts/deploy_embedded_prod.sh
```

Expected：成功生成 `out/install/embedded_prod/`；核心库尺寸增量可接受（4 个段字段 + 2 个 diag 码不会显著增加 ROM/RAM）。

- [ ] **Step 8.3: 复盘 Sprint 1 commit 历史**

```bash
git log --oneline master..HEAD
```

Expected：7 个提交，分别对应 Task 1-7（顺序：§1.1 段字段 → §1.2 诊断码 → §1.3 所有 mode 评估 → §1.4 derate 段化 → §1.5 builder → §1.6 e2e 测试 → §1.7 文档）。

- [ ] **Step 8.4: 跑 `tests/main.c` 集成 smoke**

```bash
./out/build/unixgcc/main
```

Expected：原 5 段 recipe 顺利跑完；不应该出现新诊断码（既有段未配置 ceiling）。

---

## 自检清单（在转入 Sprint 2 之前）

- [ ] Spec §1.1 段字段（4 个 ceiling + 1 个 derateRatio）：Task 1 ✓
- [ ] Spec §1.2 诊断码（2 个新码 + flag 位 + criteria 实例）：Task 2 ✓
- [ ] Spec §1.3 所有 mode 评估 ceiling：Task 3 ✓
- [ ] Spec §1.4 derateRatio 段化：Task 4 ✓
- [ ] Spec §1.5 BuildClampCloseWithMoldProtect：Task 5 ✓
- [ ] Spec §1.6 端到端集成测试（DERATE / STOP / SPEED_RAMP）：Task 6 ✓
- [ ] Spec §1.7 archetypes + runtime-contract + gap-list 文档同步：Task 7 ✓
- [ ] 所有新测试使用 TDD（先写、再实现、再确认 PASS）：✓
- [ ] 跑全量 `ctest --output-on-failure` 全 PASS：必须验证
- [ ] 跑 `./scripts/deploy_embedded_prod.sh` 成功：必须验证
- [ ] 没有 placeholder ("TBD"/"实现合理处理"/"添加错误处理")：✓

---

## 已知风险与依赖

- **Task 3 优先级表重排**：现有测试中若有断言 `DIAGNOSTIC.code == HYD_DIAG_CODE_OVER_PRESSURE` 而真实场景下 ceiling 也触发，可能因 ceiling 优先级在 OVER_PRESSURE WARNING 之上而失败。失败时**不要**简单把断言改回去——应确认该测试是否需要显式禁用 ceiling（`pressureCeiling = 0`），保留 OVER_PRESSURE 路径覆盖。
- **Task 2 criteria 字段命名**：`HYD_DiagnosticCriteriaState` 与 `HYD_DiagnosticCriteria` 的实际字段名（`triggered` / `triggerStartTime` / `elapsedDuration` 等）需以 `include/diagnostics_criteria.h` 为准。**Step 3.3 写入前先 grep 验证**，不要依赖本 plan 的猜测命名。
- **Task 6 derate 比例断言**：测试断言 `deratedPumpSpeed < normalPumpSpeed * 0.25` 假设 derate 路径乘 0.2、且采样时刻没有外部 cap（`PUMP_SPEED_LIMIT`）截断。若 limit 截断了 normal speed，derate 比例无法直接对比——这种情况需把 `PUMP_SPEED_LIMIT` 拉到 10000 以上以排除截断。
- **Sprint 0 依赖**：本 Sprint 假设 Sprint 0 已合并；Task 6 的 "通过 Abort 从 FAULT 恢复" 直接依赖 Sprint 0 C-3 修复（FAULT mask 增加 ABORT 位）。若 Sprint 0 未合并，Task 6 的恢复断言会失败——需先合并 Sprint 0。
- **嵌入式生产构建**：新增 4 个 segment 字段会让 `HYD_MotionSegment` 增大约 40 字节，`HYD_MAX_SEGMENTS * sizeof(HYD_MotionSegment)` 内存占用同步增加。在裁剪到 `HYD_MAX_SEGMENTS=8` 的嵌入式目标上，增量约 320 字节静态数据——可接受。但若客户已极限裁剪，需在 `hyd_config.h` 加编译开关裁剪 ceiling 字段（**Sprint 1 不做**，记入 Sprint 4 性能优化备选）。

---

## 失败回退方案

如果某个 Task 中途出现无法解决的回归：

1. 用 `git stash` 暂存修改
2. 用 `git diff HEAD~ HEAD -- <相关文件>` 对照原代码理解差异
3. 把回归测试用例补到对应 `tests/test_<module>.c`
4. 重新设计修复策略，分更小的步骤
5. 不要绕过 `--no-verify` 或跳过测试 — 任何 PASS 都必须真实
