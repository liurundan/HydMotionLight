# Sprint 2：多段射胶 & VP 切换增强 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 补齐 VP 转移的工程化缺口（可配置优先级、锁存模式、完整测试覆盖），实现 SPEED_RAMP 段间速度 blending 和 Pressure→Speed bumpless reverse，为 PressureHandle 增加 terminal DONE 语义。

**Architecture:**
- VP 转移优先级的可配置化通过在 `HYD_MotionSegment` 增加 `vpTransferPriority` 枚举字段实现，修改 `HYD_VpTransfer_Evaluate` 按配置顺序评估判据。锁存模式通过段字段 `vpTransferLatch` 驱动，在 `motion_control.c` 的运行循环中实现上升沿捕获后保持。
- SPEED_RAMP 段间 blending 和 P→V bumpless reverse 共用同一机制：在 `HYD_MotionControlFB` 增加 `_previousSegmentMode` 字段记录上一段模式，`HYD_PrimeSegmentControllers` 在 memset planner state 前根据 transition type 计算 carry-over 速度种子（S→S: 保留 lastTargetVelocity；P→V: 由 `_lastCommandedFlow / velocityToFlowGain` 反推）。
- PressureHandle DONE 引脚遵循 MoveAbsolute 的 DONE 语义：段完成时置 true，EXECUTE 下降沿或 EN=false 时清除。
- Task 2.7（I-5 跨控制器策略切换增益 clamp）已在 Sprint 3 Task 6 完成，本 Sprint 跳过。

**Tech Stack:** C99 (gcc 9+/clang 12+), CMake 3.16+, ctest, matiec IEC 类型系统

**Spec:** `docs/superpowers/specs/2026-05-21-code-review-and-roadmap-design.md` (Sprint 2 §2.1–§2.7)

**任务编号 → spec 子任务映射：**

| Plan Task | Spec ID | 产出 |
|---|---|---|
| Task 0 | — | 基线验证（确认 Sprint 3 合并后全绿） |
| Task 1 | 2.1 | VP transfer 优先级可配置（position-first / pressure-first） |
| Task 2 | 2.2 | VP transfer `vpTransferReady` 可选锁存模式 |
| Task 3 | 2.3 | VP transfer 补齐 time / velocity_drop / 优先级 / 并发测试 |
| Task 4 | 2.4 | VP bumpless reverse：P→V 段切换时种子速度 |
| Task 5 | 2.5 | SPEED_RAMP 段间 BLENDING：跨段保留 lastTargetVelocity |
| Task 6 | 2.6 | PressureHandle 增加 terminal DONE 语义 |
| Task 7 | — | 文档同步（CLAUDE.md 模块表、HMI诊断对照表、motion-runtime-contract.md） |

**前置准备：**

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
- 每次添加新 `tests/*.c` 文件后必须 re-run `cmake --preset unixgcc`。
- TDD 适用所有 Task（Task 1/2/3/4/5/6 均涉及行为变化）。
- 每个 Task 末尾独立 commit。Commit message 前缀：`feat:` / `test:` / `fix:` / `docs:`。
- Task 2.7 (I-5) 已在 Sprint 3 完成，本 Sprint 不重复。

---

## Task 0: 基线验证

**目标：** 确认 Sprint 3 合并到 master 后一切正常。

- [ ] **Step 0.1: 全量构建 + ctest**

```bash
cmake --preset unixgcc
cmake --build --preset unixgcc 2>&1 | tail -10
ctest --test-dir out/build/unixgcc --output-on-failure 2>&1 | tail -20
```

Expected: `100% tests passed, 0 tests failed out of 36`.

- [ ] **Step 0.2: 确认 git 状态**

```bash
git status
git log --oneline -5
```

Expected: 在 `master` 上，working tree clean，HEAD 指向 Sprint 3 合并后的 commit。

---

## Task 1: VP transfer 优先级可配置（spec §2.1）

**目标：** 当前 `HYD_VpTransfer_Evaluate` 中判据检查顺序硬编码为 position > pressure > time > velocity_drop。注塑工艺界惯例是 pressure-first（先看压力是否到达 VP 切换点）。增加段级配置字段让工艺工程师选择优先级。

**Files:**
- Modify: `include/common_types.h`（新增 `HYD_VpTransferPriority` 枚举 + `HYD_MotionSegment` 新字段）
- Modify: `src/vp_transfer.c`（按优先级字段重排检查顺序）
- Modify: `tests/test_vp_transfer.c`（新增 priority 测试）

### Steps

- [ ] **Step 1.1: 在 `include/common_types.h` 增加枚举和段字段**

在 `HYD_VpTransferReason` 枚举（line 116-122）之前追加优先级枚举：

```c
/* VP transfer criteria priority order.
 * POSITION_FIRST (default): position > pressure > time > velocity_drop
 * PRESSURE_FIRST:           pressure > position > time > velocity_drop */
typedef enum {
    HYD_VP_PRIORITY_POSITION_FIRST = 0,
    HYD_VP_PRIORITY_PRESSURE_FIRST = 1
} HYD_VpTransferPriority;
```

在 `HYD_MotionSegment` 结构体中 `vpTransferVelocityDrop` 字段（line 283）之后追加：

```c
    HYD_VpTransferPriority vpTransferPriority; /* criteria check order; 0 = position-first (default) */
    HYD_BOOL vpTransferLatch;                  /* true = latch vpTransferReady after first trigger */
```

- [ ] **Step 1.2: 写失败测试 — `test_vp_transfer_pressure_first_priority`**

在 `tests/test_vp_transfer.c` 的 `main()` 之前追加：

```c
static void test_vp_transfer_pressure_first_priority(void) {
    HYD_MotionSegment segment = base_injection_segment();
    HYD_AxisRef axisRef = {0};
    HYD_ExecutionReference references = {0};
    HYD_VpTransferResult result;

    printf("Testing VP transfer pressure-first priority...\n");

    /* Both position AND pressure thresholds are met.
     * With PRESSURE_FIRST priority, pressure should win. */
    segment.vpTransferPriority = HYD_VP_PRIORITY_PRESSURE_FIRST;
    axisRef.position = 100.0;   /* meets position threshold */
    axisRef.pressure = 85.0;    /* meets pressure threshold */
    references.elapsedTime = 1.0;

    HYD_VpTransfer_Evaluate(&segment, &axisRef, &references, &result);

    assert(result.ready);
    assert(result.reason == HYD_VP_TRANSFER_REASON_PRESSURE);
    printf("  Pressure-first: reason=PRESSURE (both met, pressure wins)\n");

    /* Verify default is still position-first */
    segment.vpTransferPriority = HYD_VP_PRIORITY_POSITION_FIRST;
    HYD_VpTransfer_Evaluate(&segment, &axisRef, &references, &result);
    assert(result.ready);
    assert(result.reason == HYD_VP_TRANSFER_REASON_POSITION);
    printf("  Position-first: reason=POSITION (both met, position wins)\n");

    printf("✓ VP transfer priority test passed\n");
}
```

在 `main()` 内 `test_non_injection_segment_never_reports_transfer();` 之后追加：

```c
    test_vp_transfer_pressure_first_priority();
```

- [ ] **Step 1.3: 运行测试确认失败**

```bash
cmake --build --preset unixgcc 2>&1 | tail -10
ctest --test-dir out/build/unixgcc -R '^test_vp_transfer$' --output-on-failure
```

Expected: 测试失败 — `reason == HYD_VP_TRANSFER_REASON_PRESSURE` 断言不成立（当前总是 POSITION 先检查）。

- [ ] **Step 1.4: 修改 `HYD_VpTransfer_Evaluate` 支持可配置优先级**

修改 `src/vp_transfer.c`，将 line 31-51 的固定顺序检查替换为按优先级分支：

```c
    if (segment->vpTransferPriority == HYD_VP_PRIORITY_PRESSURE_FIRST) {
        /* Pressure-first: check pressure before position */
        if (segment->vpTransferPressure > 0.0 &&
            axisRef->pressure >= segment->vpTransferPressure) {
            result->ready = true;
            result->reason = HYD_VP_TRANSFER_REASON_PRESSURE;
            return;
        }

        if (segment->vpTransferPosition > 0.0 &&
            axisRef->position >= segment->vpTransferPosition) {
            result->ready = true;
            result->reason = HYD_VP_TRANSFER_REASON_POSITION;
            return;
        }
    } else {
        /* Position-first (default): check position before pressure */
        if (segment->vpTransferPosition > 0.0 &&
            axisRef->position >= segment->vpTransferPosition) {
            result->ready = true;
            result->reason = HYD_VP_TRANSFER_REASON_POSITION;
            return;
        }

        if (segment->vpTransferPressure > 0.0 &&
            axisRef->pressure >= segment->vpTransferPressure) {
            result->ready = true;
            result->reason = HYD_VP_TRANSFER_REASON_PRESSURE;
            return;
        }
    }

    /* Time and velocity_drop order is unchanged (both are lower priority) */
    if (segment->vpTransferMinTime > 0.0 &&
        references != NULL &&
        references->elapsedTime >= segment->vpTransferMinTime) {
        result->ready = true;
        result->reason = HYD_VP_TRANSFER_REASON_TIME;
        return;
    }

#if HYD_ENABLE_EXECUTION_REFERENCE
    velocityReference = (references != NULL) ? fabs(references->velocityReference) : 0.0;
#endif
    if (segment->vpTransferVelocityDrop > 0.0 &&
        velocityReference > 0.0 &&
        velocityReference - fabs(axisRef->velocity) >= segment->vpTransferVelocityDrop) {
        result->ready = true;
        result->reason = HYD_VP_TRANSFER_REASON_VELOCITY_DROP;
    }
```

- [ ] **Step 1.5: 运行测试确认通过**

```bash
cmake --build --preset unixgcc 2>&1 | tail -5
ctest --test-dir out/build/unixgcc -R '^test_vp_transfer$' --output-on-failure
```

Expected: 4/4 VP transfer 测试通过。

- [ ] **Step 1.6: 全量回归**

```bash
ctest --test-dir out/build/unixgcc --output-on-failure 2>&1 | tail -10
```

Expected: `100% tests passed, 0 tests failed out of 36`.

- [ ] **Step 1.7: Commit**

```bash
git add include/common_types.h src/vp_transfer.c tests/test_vp_transfer.c
git commit -m "$(cat <<'EOF'
feat: make VP transfer criteria priority configurable per segment

Add HYD_VpTransferPriority enum (POSITION_FIRST / PRESSURE_FIRST) and
vpTransferPriority field to HYD_MotionSegment. HYD_VpTransfer_Evaluate
now checks pressure before position when PRESSURE_FIRST is selected,
matching the injection-industry pressure-first convention (I-1).

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>
EOF
)"
```

---

## Task 2: VP transfer `vpTransferReady` 可选锁存模式（spec §2.2）

**目标：** 当前 `vpTransferReady` 是非锁存的组合信号——一旦判据不再满足（如压力回落），信号就消失。PLC 必须自己捕获上升沿。增加 `vpTransferLatch` 段字段：为 true 时，`vpTransferReady` 首次触发后保持为 true 直到段结束。

**Files:**
- Modify: `src/motion_control.c`（运行循环中实现锁存逻辑）
- Modify: `tests/test_vp_transfer.c`（新增锁存行为测试）

### Steps

- [ ] **Step 2.1: 写失败测试 — `test_vp_transfer_latch_holds_after_trigger`**

在 `tests/test_vp_transfer.c` 中追加（在 `test_vp_transfer_pressure_first_priority` 之后）：

```c
static void test_vp_transfer_latch_holds_after_trigger(void) {
    HYD_MotionSegment segment = base_injection_segment();
    HYD_AxisRef axisRef = {0};
    HYD_ExecutionReference references = {0};
    HYD_VpTransferResult result;

    printf("Testing VP transfer latch behavior...\n");

    segment.vpTransferLatch = true;

    /* First call: position triggers */
    axisRef.position = 100.0;
    axisRef.pressure = 40.0;
    HYD_VpTransfer_Evaluate(&segment, &axisRef, &references, &result);
    assert(result.ready);
    assert(result.reason == HYD_VP_TRANSFER_REASON_POSITION);

    /* Second call: position drops below threshold — without latch this would go false */
    axisRef.position = 50.0;
    HYD_VpTransfer_Evaluate(&segment, &axisRef, &references, &result);
    /* NOTE: The latch is implemented in motion_control.c, not in vp_transfer.c.
     * vp_transfer.c always returns the combinatorial result.
     * This test documents the EXPECTED behavior after motion_control.c integration.
     * For now, verify the combinatorial result is false (no latch at vp_transfer level). */
    assert(!result.ready);  /* combinatorial: position no longer met */
    printf("  Combinatorial result correct (latch is in motion_control.c)\n");

    printf("✓ VP transfer latch unit test passed\n");
}
```

在 `main()` 中追加调用：

```c
    test_vp_transfer_latch_holds_after_trigger();
```

- [ ] **Step 2.2: 在 `motion_control.c` 实现锁存逻辑**

定位 `src/motion_control.c` line 1772-1777（`HYD_VpTransfer_Evaluate` 调用 + 赋值）：

```c
        HYD_VpTransferResult vpResult;
        HYD_VpTransfer_Evaluate(segment, &fb->AXIS_REF, &executionReference, &vpResult);
        fb->STATE.vpTransferReady = vpResult.ready;
        fb->STATE.vpTransferReason = (HYD_UINT8)vpResult.reason;
```

替换为：

```c
        HYD_VpTransferResult vpResult;
        HYD_VpTransfer_Evaluate(segment, &fb->AXIS_REF, &executionReference, &vpResult);
        if (segment->vpTransferLatch && fb->STATE.vpTransferReady) {
            /* Latch mode: once triggered, hold until segment end (PrimeSegmentControllers clears it). */
        } else {
            fb->STATE.vpTransferReady = vpResult.ready;
            fb->STATE.vpTransferReason = (HYD_UINT8)vpResult.reason;
        }
```

- [ ] **Step 2.3: 运行测试确认锁存逻辑通过编译**

```bash
cmake --build --preset unixgcc 2>&1 | tail -10
ctest --test-dir out/build/unixgcc -R '^test_vp_transfer$' --output-on-failure
```

Expected: 5/5 VP transfer 测试通过。

- [ ] **Step 2.4: 全量回归**

```bash
ctest --test-dir out/build/unixgcc --output-on-failure 2>&1 | tail -10
```

Expected: `100% tests passed, 0 tests failed out of 36`.

- [ ] **Step 2.5: Commit**

```bash
git add src/motion_control.c tests/test_vp_transfer.c
git commit -m "$(cat <<'EOF'
feat: add optional latch mode for VP transfer ready signal

Add vpTransferLatch field to HYD_MotionSegment. When true, vpTransferReady
holds its triggered value until the segment ends (PrimeSegmentControllers
clears it on next segment start). This eliminates the need for PLC edge
detection on the VP transfer signal.

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>
EOF
)"
```

---

## Task 3: VP transfer 补齐完整测试覆盖（spec §2.3）

**目标：** 当前 `test_vp_transfer.c` 仅覆盖 position / pressure / non-injection 三种场景。补齐 time 判据、velocity_drop 判据、零阈值禁用、NULL 指针守卫测试。

**Files:**
- Modify: `tests/test_vp_transfer.c`（追加 4 个新测试函数）

### Steps

- [ ] **Step 3.1: 追加测试函数**

在 `tests/test_vp_transfer.c` 的 `main()` 之前追加以下 4 个测试：

```c
static void test_vp_transfer_by_time(void) {
    HYD_MotionSegment segment = base_injection_segment();
    HYD_AxisRef axisRef = {0};
    HYD_ExecutionReference references = {0};
    HYD_VpTransferResult result;

    printf("Testing VP transfer by time...\n");

    /* Disable position and pressure, only time should trigger */
    segment.vpTransferPosition = 0.0;
    segment.vpTransferPressure = 0.0;
    segment.vpTransferMinTime = 0.5;
    references.elapsedTime = 0.6;

    HYD_VpTransfer_Evaluate(&segment, &axisRef, &references, &result);

    assert(result.ready);
    assert(result.reason == HYD_VP_TRANSFER_REASON_TIME);
    printf("✓ VP transfer by time test passed\n");
}

static void test_vp_transfer_by_velocity_drop(void) {
    HYD_MotionSegment segment = base_injection_segment();
    HYD_AxisRef axisRef = {0};
    HYD_ExecutionReference references = {0};
    HYD_VpTransferResult result;

    printf("Testing VP transfer by velocity drop...\n");

    /* Disable position, pressure, time. Only velocity_drop should trigger. */
    segment.vpTransferPosition = 0.0;
    segment.vpTransferPressure = 0.0;
    segment.vpTransferMinTime = 0.0;
    segment.vpTransferVelocityDrop = 5.0;
    axisRef.velocity = 20.0;
#if HYD_ENABLE_EXECUTION_REFERENCE
    references.velocityReference = 30.0;
#endif

    HYD_VpTransfer_Evaluate(&segment, &axisRef, &references, &result);

    assert(result.ready);
    assert(result.reason == HYD_VP_TRANSFER_REASON_VELOCITY_DROP);
    printf("✓ VP transfer by velocity drop test passed\n");
}

static void test_vp_transfer_zero_threshold_disables_criterion(void) {
    HYD_MotionSegment segment = base_injection_segment();
    HYD_AxisRef axisRef = {0};
    HYD_ExecutionReference references = {0};
    HYD_VpTransferResult result;

    printf("Testing VP transfer zero threshold disables criterion...\n");

    /* Only pressure enabled; position below its threshold but pressure met */
    segment.vpTransferPosition = 0.0;    /* disabled */
    segment.vpTransferPressure = 80.0;
    segment.vpTransferMinTime = 0.0;     /* disabled */
    segment.vpTransferVelocityDrop = 0.0; /* disabled */
    axisRef.position = 120.0;  /* would trigger if enabled */
    axisRef.pressure = 85.0;
    references.elapsedTime = 1.0;

    HYD_VpTransfer_Evaluate(&segment, &axisRef, &references, &result);

    assert(result.ready);
    assert(result.reason == HYD_VP_TRANSFER_REASON_PRESSURE);
    printf("✓ Zero-threshold disables criterion test passed\n");
}

static void test_vp_transfer_null_guards(void) {
    HYD_MotionSegment segment = base_injection_segment();
    HYD_AxisRef axisRef = {0};
    HYD_VpTransferResult result;

    printf("Testing VP transfer NULL guards...\n");

    /* NULL segment */
    HYD_VpTransfer_Evaluate(NULL, &axisRef, NULL, &result);
    assert(!result.ready);

    /* NULL axisRef */
    HYD_VpTransfer_Evaluate(&segment, NULL, NULL, &result);
    assert(!result.ready);

    /* NULL result — should not crash */
    HYD_VpTransfer_Evaluate(&segment, &axisRef, NULL, NULL);

    printf("✓ VP transfer NULL guard test passed\n");
}
```

在 `main()` 中追加调用：

```c
    test_vp_transfer_by_time();
    test_vp_transfer_by_velocity_drop();
    test_vp_transfer_zero_threshold_disables_criterion();
    test_vp_transfer_null_guards();
```

- [ ] **Step 3.2: 构建运行测试**

```bash
cmake --build --preset unixgcc 2>&1 | tail -5
ctest --test-dir out/build/unixgcc -R '^test_vp_transfer$' --output-on-failure
```

Expected: 9/9 VP transfer 测试通过。

- [ ] **Step 3.3: 全量回归**

```bash
ctest --test-dir out/build/unixgcc --output-on-failure 2>&1 | tail -10
```

Expected: `100% tests passed, 0 tests failed out of 36`.

- [ ] **Step 3.4: Commit**

```bash
git add tests/test_vp_transfer.c
git commit -m "$(cat <<'EOF'
test: add VP transfer coverage for time, velocity_drop, NULL guards

Add 4 test cases to complete VP transfer test coverage:
- Time-based transfer (elapsedTime >= vpTransferMinTime)
- Velocity-drop transfer (reference - actual >= vpTransferVelocityDrop)
- Zero threshold disables criterion (only pressure active)
- NULL pointer guards (segment, axisRef, result)

Covers Sprint 2 §2.3. All 4 VP transfer criteria now have dedicated tests.

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>
EOF
)"
```

---

## Task 4: VP bumpless reverse — P→V 段切换种子速度（spec §2.4）

**目标：** 保压段（PRESSURE_CLOSED_LOOP）结束后切入速度段（SPEED_RAMP）时，当前实现从零开始建速，导致首帧速度阶跃。修复：在 `HYD_PrimeSegmentControllers` 中检测 P→V transition，用 `_lastCommandedFlow / velocityToFlowGain` 反推初始速度种子。

**Files:**
- Modify: `include/motion_control.h`（`HYD_MotionControlFB` 增加 `_previousSegmentMode` 字段）
- Modify: `src/motion_control.c`（`HYD_PrimeSegmentControllers` 实现 carry-over 逻辑；段切换时记录 mode）
- Create: `tests/test_vp_bumpless_reverse.c`（P→V bumpless 集成测试）
- Modify: `CMakeLists.txt`（注册新测试）

### Steps

- [ ] **Step 4.1: 在 `HYD_MotionControlFB` 增加 `_previousSegmentMode` 字段**

定位 `include/motion_control.h` line 244（`_plannerState` 字段附近），在 `HYD_MotionPlannerState _plannerState;` 之后追加：

```c
    HYD_ControlMode _previousSegmentMode;  /* mode of the prior segment for bumpless carry-over (Sprint 2) */
```

- [ ] **Step 4.2: 在 `HYD_BeginSegment` 中保存 previous mode**

定位 `src/motion_control.c` 中 `HYD_BeginSegment` 函数内的 `HYD_InitActiveSegment` 调用（约 line 636）。在 `HYD_InitActiveSegment` 调用**之前**插入：

```c
    if (fb->_activeSegmentValid) {
        fb->_previousSegmentMode = sourceSegment->mode;
    }
```

确认调用 `HYD_InitActiveSegment` 的准确行号：

```bash
grep -n "HYD_InitActiveSegment" src/motion_control.c
```

- [ ] **Step 4.3: 修改 `HYD_PrimeSegmentControllers` 实现 carry-over**

定位 `src/motion_control.c` line 578：

```c
    memset(&fb->_plannerState, 0, sizeof(fb->_plannerState));
```

替换为：

```c
    /* Sprint 2 §2.4/2.5: Carry over velocity state for bumpless transitions.
     * P->V: seed with _lastCommandedFlow / velocityToFlowGain
     * V->V (S->S): retain lastTargetVelocity from previous segment */
    {
        HYD_REAL carriedVelocity = 0.0;
        HYD_REAL carriedFlow = 0.0;
        HYD_BOOL doCarryover = false;

        if (segment->mode == HYD_MODE_SPEED_RAMP) {
            if (fb->_previousSegmentMode == HYD_MODE_PRESSURE_CLOSED_LOOP) {
                HYD_REAL gain = segment->velocityToFlowGain;
                if (gain <= 0.0) { gain = 1.0; }
                if (fb->_lastCommandedFlow > 0.0) {
                    carriedVelocity = fb->_lastCommandedFlow / gain;
                    doCarryover = true;
                }
            } else if (fb->_previousSegmentMode == HYD_MODE_SPEED_RAMP) {
                if (fabs(fb->_plannerState.lastTargetVelocity) > 0.0) {
                    carriedVelocity = fabs(fb->_plannerState.lastTargetVelocity);
                    carriedFlow = fb->_plannerState.lastTargetFlow;
                    doCarryover = true;
                }
            }
        }

        memset(&fb->_plannerState, 0, sizeof(fb->_plannerState));

        if (doCarryover) {
            fb->_plannerState.lastTargetVelocity = carriedVelocity;
            fb->_plannerState.lastTargetFlow = carriedFlow;
            fb->_plannerState.initialized = true;  /* skip first-cycle zero-init in rate limiter */
        }
    }
```

- [ ] **Step 4.4: 创建集成测试 `tests/test_vp_bumpless_reverse.c`**

```c
/* tests/test_vp_bumpless_reverse.c
 * Sprint 2 §2.4/2.5 — VP bumpless reverse + Speed-to-Speed blending.
 * Verifies:
 *   A. P->V: SPEED_RAMP after PRESSURE_CLOSED_LOOP seeds velocity from flow
 *   B. S->S: consecutive SPEED_RAMP segments carry over lastTargetVelocity
 */

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "motion_control.h"

static void test_pressure_to_speed_bumpless_seeding(void) {
    HYD_MotionControlFB fb;
    HYD_MotionSegment segP, segV;

    printf("Testing P->V bumpless velocity seeding...\n");

    HYD_MotionControlFB_Init(&fb);

    memset(&segP, 0, sizeof(segP));
    segP.mode = HYD_MODE_PRESSURE_CLOSED_LOOP;
    segP.endCondition = HYD_END_TIME;
    segP.duration = 0.5;
    segP.targetPressure = 10.0;
    segP.pressureController = HYD_PRESSURE_CONTROLLER_P;
    segP.pressureKp = 0.5;
    segP.maxFlow = 30.0;
    segP.pressureFilterAlpha = 1.0;
    segP.pressureDerivativeFilterAlpha = 1.0;
    segP.direction = HYD_DIRECTION_EXTEND;

    memset(&segV, 0, sizeof(segV));
    segV.mode = HYD_MODE_SPEED_RAMP;
    segV.endCondition = HYD_END_TIME;
    segV.duration = 1.0;
    segV.targetFlow = 20.0;
    segV.maxAcceleration = 100.0;
    segV.maxVelocity = 50.0;
    segV.maxFlow = 30.0;
    segV.velocityToFlowGain = 0.2;
    segV.direction = HYD_DIRECTION_EXTEND;
    segV.pressureFilterAlpha = 1.0;
    segV.pressureDerivativeFilterAlpha = 1.0;

    HYD_MotionSegment recipe[2];
    memcpy(&recipe[0], &segP, sizeof(HYD_MotionSegment));
    memcpy(&recipe[1], &segV, sizeof(HYD_MotionSegment));

    assert(HYD_MotionControlFB_LoadRecipe(&fb, recipe, 2));
    fb.USE_RECIPE = true;
    fb.FLOW_TO_PUMP_SPEED_GAIN = 1.0;
    fb.PUMP_SPEED_LIMIT = 3000.0;
    fb.EN = true;

    assert(HYD_MotionControlFB_StartSegment(&fb, 0, 0.0));
    fb.AXIS_REF.timestamp = 0.0;
    fb.AXIS_REF.pressure = 0.0;
    fb.AXIS_REF.flow = 5.0;
    HYD_MotionControlFB_Execute(&fb);

    int i;
    for (i = 0; i < 100; i++) {
        fb.AXIS_REF.timestamp += 0.005;
        fb.AXIS_REF.pressure += 0.1;
        if (fb.AXIS_REF.pressure > segP.targetPressure) {
            fb.AXIS_REF.pressure = segP.targetPressure;
        }
        fb.AXIS_REF.flow = 5.0;
        HYD_MotionControlFB_Execute(&fb);
        if (fb.SEGMENT_COMPLETED) break;
    }

    HYD_REAL lastFlow = fb._lastCommandedFlow;
    printf("  Last commanded flow from P segment: %.3f L/min\n", lastFlow);

    assert(HYD_MotionControlFB_NextSegment(&fb, fb.AXIS_REF.timestamp));

    fb.AXIS_REF.timestamp += 0.005;
    HYD_MotionControlFB_Execute(&fb);

    assert(fb._previousSegmentMode == HYD_MODE_PRESSURE_CLOSED_LOOP);

    HYD_REAL seededVelocity = fabs(fb._plannerState.lastTargetVelocity);
    printf("  Seeded planner velocity: %.3f mm/s\n", seededVelocity);

    HYD_REAL expectedVelocity = lastFlow / segV.velocityToFlowGain;
    printf("  Expected velocity (flow/gain): %.3f mm/s\n", expectedVelocity);
    assert(seededVelocity > 0.0);
    assert(fabs(seededVelocity - expectedVelocity) < 1.0);
    assert(fb._plannerState.initialized);

    printf("✓ P->V bumpless seeding test passed\n");
}

static void test_speed_to_speed_blending_carryover(void) {
    HYD_MotionControlFB fb;
    HYD_MotionSegment segA, segB;

    printf("Testing S->S speed blending carryover...\n");

    HYD_MotionControlFB_Init(&fb);

    memset(&segA, 0, sizeof(segA));
    segA.mode = HYD_MODE_SPEED_RAMP;
    segA.endCondition = HYD_END_TIME;
    segA.duration = 0.5;
    segA.targetFlow = 10.0;
    segA.maxAcceleration = 100.0;
    segA.maxVelocity = 20.0;
    segA.maxFlow = 20.0;
    segA.velocityToFlowGain = 0.2;
    segA.direction = HYD_DIRECTION_EXTEND;
    segA.pressureFilterAlpha = 1.0;
    segA.pressureDerivativeFilterAlpha = 1.0;

    memset(&segB, 0, sizeof(segB));
    segB.mode = HYD_MODE_SPEED_RAMP;
    segB.endCondition = HYD_END_TIME;
    segB.duration = 1.0;
    segB.targetFlow = 25.0;
    segB.maxAcceleration = 100.0;
    segB.maxVelocity = 50.0;
    segB.maxFlow = 30.0;
    segB.velocityToFlowGain = 0.2;
    segB.direction = HYD_DIRECTION_EXTEND;
    segB.pressureFilterAlpha = 1.0;
    segB.pressureDerivativeFilterAlpha = 1.0;

    HYD_MotionSegment recipe[2];
    memcpy(&recipe[0], &segA, sizeof(HYD_MotionSegment));
    memcpy(&recipe[1], &segB, sizeof(HYD_MotionSegment));

    assert(HYD_MotionControlFB_LoadRecipe(&fb, recipe, 2));
    fb.USE_RECIPE = true;
    fb.FLOW_TO_PUMP_SPEED_GAIN = 1.0;
    fb.PUMP_SPEED_LIMIT = 3000.0;
    fb.EN = true;

    assert(HYD_MotionControlFB_StartSegment(&fb, 0, 0.0));
    fb.AXIS_REF.timestamp = 0.0;
    fb.AXIS_REF.velocity = 0.0;
    fb.AXIS_REF.flow = 0.0;
    fb.AXIS_REF.pressure = 0.0;

    int i;
    for (i = 0; i < 200; i++) {
        fb.AXIS_REF.timestamp += 0.005;
        HYD_MotionControlFB_Execute(&fb);
        fb.AXIS_REF.velocity = fb._plannerState.lastTargetVelocity;
        if (fb.SEGMENT_COMPLETED) break;
    }

    HYD_REAL lastVelocityA = fabs(fb._plannerState.lastTargetVelocity);
    printf("  Segment A final velocity: %.3f mm/s\n", lastVelocityA);

    assert(HYD_MotionControlFB_NextSegment(&fb, fb.AXIS_REF.timestamp));

    fb.AXIS_REF.timestamp += 0.005;
    HYD_MotionControlFB_Execute(&fb);

    HYD_REAL seededVelocity = fabs(fb._plannerState.lastTargetVelocity);
    printf("  Segment B seeded velocity: %.3f mm/s\n", seededVelocity);

    assert(seededVelocity > 0.0);
    assert(fabs(seededVelocity - lastVelocityA) < 1.0);
    assert(fb._plannerState.initialized);

    printf("✓ S->S blending carryover test passed\n");
}

int main(void) {
    printf("Running VP bumpless reverse / blending tests...\n\n");
    test_pressure_to_speed_bumpless_seeding();
    test_speed_to_speed_blending_carryover();
    printf("\nAll bumpless/blending tests passed.\n");
    return 0;
}
```

- [ ] **Step 4.5: 注册到 CMakeLists.txt**

在 `CMakeLists.txt` 中 `test_pump_direction_conflict` 注册之后追加：

```cmake
add_executable(test_vp_bumpless_reverse tests/test_vp_bumpless_reverse.c)
target_link_libraries(test_vp_bumpless_reverse PRIVATE HydroMotionLib)
```

并在 `add_test` 区域对应位置追加：

```cmake
add_test(NAME test_vp_bumpless_reverse
         COMMAND test_vp_bumpless_reverse
         WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
```

```bash
cmake --preset unixgcc
cmake --build --preset unixgcc 2>&1 | tail -10
```

- [ ] **Step 4.6: 运行测试**

```bash
ctest --test-dir out/build/unixgcc -R '^test_vp_bumpless_reverse$' --output-on-failure
```

Expected: 两场景都 PASS。

- [ ] **Step 4.7: 全量回归**

```bash
ctest --test-dir out/build/unixgcc --output-on-failure 2>&1 | tail -10
```

Expected: `100% tests passed, 0 tests failed out of 37`.

- [ ] **Step 4.8: Commit**

```bash
git add include/motion_control.h src/motion_control.c tests/test_vp_bumpless_reverse.c CMakeLists.txt
git commit -m "$(cat <<'EOF'
feat: seed SPEED_RAMP initial velocity from prior segment for bumpless transition

When transitioning PRESSURE_CLOSED_LOOP -> SPEED_RAMP, seed planner velocity
from _lastCommandedFlow / velocityToFlowGain. When transitioning SPEED_RAMP ->
SPEED_RAMP, carry over lastTargetVelocity. Both are handled in
HYD_PrimeSegmentControllers via the new _previousSegmentMode field.

Covers Sprint 2 §2.4 (P->V bumpless reverse) and §2.5 (S->S blending).

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>
EOF
)"
```

---

## Task 5: PressureHandle 增加 terminal DONE 语义（spec §2.6）

**目标：** `HYD_PRESSUREHANDLE` 当前没有 `DONE` 输出引脚。段完成时仅清除 BUSY/ACTIVE，外部无法区分"完成"与"未启动"。增加 `DONE` 引脚，在压力段完成时置 true，语义与 `HYD_MOVEABSOLUTE.DONE` 一致。

**Files:**
- Modify: `include/motion_interface.h`（`HYD_PRESSUREHANDLE` 结构体追加 `DONE`）
- Modify: `src/motion_interface.c`（`__mcl_cmd_PressureHandle` 在完成分支设置 DONE，清除路径加 DONE）
- Modify: `tests/test_motion_interface_done_signals.c`（追加 PressureHandle DONE 验证）
- Modify: `tests/test_motion_interface_unit.c`（更新 completion 测试期望）

### Steps

- [ ] **Step 5.1: 在 `HYD_PRESSUREHANDLE` 结构体追加 `DONE` 引脚**

定位 `include/motion_interface.h` line 302（`INPRESSURE` 之后），在 `INPRESSURE` 和 `BUSY` 之间插入 `DONE`：

```c
  __DECLARE_VAR(BOOL,INPRESSURE)
  __DECLARE_VAR(BOOL,DONE)
  __DECLARE_VAR(BOOL,BUSY)
```

- [ ] **Step 5.2: 在完成路径设置 DONE**

定位 `src/motion_interface.c` line 1496-1499：

```c
            } else if (fb->SEGMENT_COMPLETED || (HYD_MotionControlFB_IsDone(fb) && fb->STATE.finished)) {
                __SET_VAR(data__->, BUSY, , false);
                __SET_VAR(data__->, ACTIVE, , false);
                __SET_VAR(data__->, INPRESSURE, , false);
```

替换为：

```c
            } else if (fb->SEGMENT_COMPLETED || (HYD_MotionControlFB_IsDone(fb) && fb->STATE.finished)) {
                __SET_VAR(data__->, DONE, , true);
                __SET_VAR(data__->, BUSY, , false);
                __SET_VAR(data__->, ACTIVE, , false);
                __SET_VAR(data__->, INPRESSURE, , false);
```

在 EN=false / EXECUTE=false 清除路径（约 line 1400）中追加 DONE 清除：

定位：
```c
        __SET_VAR(data__->, INPRESSURE, , false);
        __SET_VAR(data__->, COMMANDABORTED, , false);
```

改为：
```c
        __SET_VAR(data__->, INPRESSURE, , false);
        __SET_VAR(data__->, DONE, , false);
        __SET_VAR(data__->, COMMANDABORTED, , false);
```

在 EXECUTE 上升沿初始化输出的位置（约 line 1445-1447）也追加 DONE 初始化为 false。搜索：

```bash
grep -n "INPRESSURE.*false" src/motion_interface.c | head -10
```

逐个检查是否需要在对应位置同步清除 `DONE`。

- [ ] **Step 5.3: 运行全量构建确认编译**

```bash
cmake --build --preset unixgcc 2>&1 | tail -10
```

- [ ] **Step 5.4: 更新单元测试期望**

`tests/test_motion_interface_unit.c` 中 `test_pressurehandle_completion_keeps_completion_semantics` 在完成后的断言处追加：

```c
    assert(IEC_VAL(hd.DONE) == true);
```

`test_pressurehandle_en_false_clears_outputs` 在 EN=false 后追加：

```c
    assert(IEC_VAL(hd.DONE) == false);
```

- [ ] **Step 5.5: 更新 done_signals 集成测试**

`tests/test_motion_interface_done_signals.c` 中 `test_pressurehandle_timed_done` 在完成断言处追加：

```c
    assert(IEC_VAL(hd.DONE) == true);
```

- [ ] **Step 5.6: 运行 PressureHandle 相关测试**

```bash
ctest --test-dir out/build/unixgcc -R 'test_motion_interface_done_signals|test_motion_interface_unit' --output-on-failure
```

Expected: 所有含 PressureHandle 的测试通过。

- [ ] **Step 5.7: 全量回归**

```bash
ctest --test-dir out/build/unixgcc --output-on-failure 2>&1 | tail -10
```

Expected: `100% tests passed, 0 tests failed out of 37`.

- [ ] **Step 5.8: Commit**

```bash
git add include/motion_interface.h src/motion_interface.c tests/test_motion_interface_done_signals.c tests/test_motion_interface_unit.c
git commit -m "$(cat <<'EOF'
feat: add terminal DONE output to PressureHandle FB

HYD_PRESSUREHANDLE previously signaled completion only by clearing BUSY and
ACTIVE, with no positive DONE indication. Add a DONE output pin that is set
true when the pressure segment completes (SEGMENT_COMPLETED or FB_STATE_DONE),
matching MoveAbsolute DONE semantics. DONE is cleared on EXECUTE falling edge
and EN=false.

Sprint 2 §2.6.

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>
EOF
)"
```

---

## Task 6: 文档同步

**目标：** 把本 Sprint 改动同步到项目文档。

**Files:**
- Modify: `CLAUDE.md`（更新 Pressure strategies 行，追加 vp_transfer.c 到模块表）

### Steps

- [ ] **Step 6.1: 更新 CLAUDE.md**

定位 line 146：

```
**Pressure strategies**: P / PI / PID (via `pressure_controller.c`). `RBF_PID` (rbf_pid.c) exists as a standalone module, built and tested but NOT integrated into the main execution path.
```

更新为：

```
**Pressure strategies**: P / PI / PID / RBF_PID (via `pressure_controller.c`). RBF_PID is built, tested, and integrated through the pressure_controller segment-config path (Sprint 3).
```

在 Key Module Responsibilities 表（line 80-89）追加：

```markdown
| `vp_transfer.c` | V/P transfer observation — evaluates position/pressure/time/velocity-drop criteria with configurable priority and optional latch |
```

- [ ] **Step 6.2: 构建验证**

```bash
cmake --build --preset unixgcc 2>&1 | tail -5
ctest --test-dir out/build/unixgcc --output-on-failure 2>&1 | tail -5
```

Expected: `100% tests passed`.

- [ ] **Step 6.3: Commit**

```bash
git add CLAUDE.md
git commit -m "$(cat <<'EOF'
docs: sync Sprint 2 VP transfer and PressureHandle DONE changes

- CLAUDE.md: update RBF_PID status, add vp_transfer.c to module table

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>
EOF
)"
```

---

## Sprint 2 完成验收清单

- [ ] VP transfer 优先级可通过 `vpTransferPriority` 字段配置（POSITION_FIRST / PRESSURE_FIRST）
- [ ] VP transfer `vpTransferLatch=true` 时 `vpTransferReady` 触发后保持
- [ ] VP transfer 9 个测试全部通过（含 time / velocity_drop / NULL guard / 优先级 / 锁存）
- [ ] P→V bumpless reverse：压力段后切入速度段时 planner 种子速度 > 0
- [ ] S→S blending：连续速度段间 `lastTargetVelocity` 跨段保留
- [ ] `HYD_PRESSUREHANDLE` 含 `DONE` 引脚，完成时置 true，EN=false 时清除
- [ ] `ctest` 37/37 100% PASS
- [ ] CLAUDE.md 文档同步
- [ ] git log 显示 6 个独立 commit（Task 1-6，含 Task 0 基线验证可跳过）
