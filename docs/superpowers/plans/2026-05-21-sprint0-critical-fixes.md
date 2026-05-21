# Sprint 0：紧急修复 + 风险消除 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 修复 7 项 Critical/Important 缺陷与测试空白，使 HydroMotionLib 具备承载 Sprint 1-3 新功能（低压护膜 / VP 增强 / RBF 工程化）的健康基线。

**Architecture:** 不引入任何新功能，全部为 **bugfix + 单元测试补全 + CMake 清理**。所有变更必须保持现有 PLCopen 风格 API 与 PLC 工艺层契约（`docs/architecture/motion-runtime-contract.md`）不变。TDD：先写复现 bug 的失败测试，再修复，再确认通过。

**Tech Stack:** C99（gcc 9+/clang 12+）、CMake 3.16+、ctest、Beremiz/matiec IEC 类型系统、`tests/` 下纯 C 单元测试 + `hydro_sim` 物理仿真器。

**Spec:** `docs/superpowers/specs/2026-05-21-code-review-and-roadmap-design.md`

**任务编号映射到 spec 中的问题 ID：**
| Plan Task | Spec ID | 优先级 | 估时 |
|---|---|---|---|
| Task 1 | M-1, M-2 | P2 | 0.5d |
| Task 2 | C-3 | P0 | 1d + 1d test |
| Task 3 | C-7 | P1 | 1d + 1d test |
| Task 4 | C-4 | P0 | 2d + 1d test |
| Task 5 | I-7 | P1 | 3d |
| Task 6 | C-1 | P0 | 3d + 2d test |
| Task 7 | C-2 | P0 | 3d + 2d test |

**前置准备：所有 Task 共用的 working tree 与编译命令**

```bash
# 工作目录（WSL 内）
cd /home/dan/project/hdy-motion-light

# 配置（一次）
cmake --preset unixgcc

# 全量构建
cmake --build --preset unixgcc

# 单测试运行
ctest --test-dir out/build/unixgcc -R '<test_name>' --output-on-failure

# 全量测试
ctest --test-dir out/build/unixgcc --output-on-failure
```

**重要约定：每次添加新 `tests/*.c` 文件后，必须 re-run `cmake --preset unixgcc`，因为 CMakeLists 使用 `file(GLOB_RECURSE ...)`。**

---

## Task 1: 测试清理 + CMake 注册补全（M-1, M-2）

**目标：** 删重复测试 `ramp_controller_test.c`；在 CMakeLists.txt 显式注册 `test_direct_mode_simple`。`benchmark_performance` 保持仅构建不 add_test（CI 阻塞通路不跑性能测试是正确做法），但加注释说明。

**Files:**
- Delete: `tests/ramp_controller_test.c`
- Modify: `CMakeLists.txt`（注册 `test_direct_mode_simple`、补 `benchmark_performance` 注释、移除 `ramp_controller_test.c` 引用）

### Steps

- [ ] **Step 1.1: 确认重复内容已被 `test_ramp_controller.c` 全覆盖**

```bash
# 用 diff 大致看测试名/断言
diff <(grep -o 'test_[A-Za-z_]*' tests/ramp_controller_test.c | sort -u) \
     <(grep -o 'test_[A-Za-z_]*' tests/test_ramp_controller.c | sort -u)
```

Expected：`ramp_controller_test.c` 的所有测试名都出现在 `test_ramp_controller.c` 中（或为子集）。若有遗漏的断言，先迁移到 `test_ramp_controller.c` 再继续。

- [ ] **Step 1.2: 删除 `tests/ramp_controller_test.c`**

```bash
rm tests/ramp_controller_test.c
```

- [ ] **Step 1.3: 修改 `CMakeLists.txt`，移除 `ramp_controller_test` add_executable 与 add_test，注册 `test_direct_mode_simple`，给 benchmark 加注释**

打开 `CMakeLists.txt`，定位到测试注册区域（约 80-160 行）。

将原本的 `add_executable(ramp_controller_test tests/ramp_controller_test.c)` 与对应 `add_test(NAME test_ramp_controller ...)` 整段删除（保留 `test_ramp_controller_detailed` 那一对）。

确认 `test_direct_mode_simple` 已构建（CMakeLists.txt 大约 86 行附近），在其下方紧接添加 `add_test`：

```cmake
add_test(NAME test_direct_mode_simple
         COMMAND test_direct_mode_simple
         WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
```

在 `benchmark_performance` 的 `add_executable` 下方追加注释：

```cmake
# benchmark_performance intentionally NOT registered with add_test():
# performance benchmarks should be run in a separate, non-blocking CI job
# to avoid flakiness in the main test suite.
```

- [ ] **Step 1.4: 重新配置 + 构建 + 验证测试列表**

```bash
cmake --preset unixgcc
cmake --build --preset unixgcc 2>&1 | tail -20
ctest --test-dir out/build/unixgcc -N | grep -E 'ramp_controller|direct_mode_simple|benchmark'
```

Expected：
- 没有 `ramp_controller_test` 单独的 test entry（只剩 `test_ramp_controller` 和 `test_ramp_controller_detailed`）。
- 有 `test_direct_mode_simple` 一行。
- 没有 `benchmark_performance` 进入 ctest 列表。

- [ ] **Step 1.5: 跑全量测试确认无回归**

```bash
ctest --test-dir out/build/unixgcc --output-on-failure
```

Expected：所有现存测试 PASS（包括新注册的 `test_direct_mode_simple`）。

- [ ] **Step 1.6: 提交**

```bash
git add tests/ CMakeLists.txt
git commit -m "$(cat <<'EOF'
test: remove duplicate ramp_controller_test + register direct_mode_simple

- Delete tests/ramp_controller_test.c (superseded by tests/test_ramp_controller.c)
- Register test_direct_mode_simple in CMakeLists.txt
- Document why benchmark_performance is not auto-run

Refs: Sprint 0 spec M-1, M-2 (2026-05-21-code-review-and-roadmap-design.md)

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 2: FAULT 状态允许 ABORT 退出（C-3）

**目标：** 让 PLC 工艺层在 FAULT 状态下能通过 ABORT FB 把 runtime 推进到 ABORTED，而不是必须依赖直接置位 `fb->RESET` 标志位。

**根因：** `HYD_COMMAND_ALLOWED_STATE_MASKS[HYD_CMD_ABORT]`（`src/motion_control.c:74-81`）缺 `HYD_FB_STATE_FAULT` 位；`HYD_RequestAbortCommand`（约 `src/motion_control.c:1818-1820`）在 `fb->STATE.faultActive` 时提前拒绝。

**Files:**
- Modify: `src/motion_control.c:74-81`（mask 表）
- Modify: `src/motion_control.c:1818-1820`（`HYD_RequestAbortCommand` 提前拒绝逻辑）
- Modify: `include/motion_control.h:111`（contract 注释更新）
- Modify: `docs/architecture/motion-runtime-contract.md`（命令合法性表）
- Modify: `docs/integration/plc-process-layer-integration-guide.md`（追加"FAULT 退出路径"章节）
- Test: `tests/test_fault_recovery.c`（新建）

### Steps

- [ ] **Step 2.1: 创建失败测试 `tests/test_fault_recovery.c`**

```c
/* tests/test_fault_recovery.c - Verifies FAULT -> ABORTED via Abort() command */
#include "motion_control.h"
#include "action_profile.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static void prime_fb_with_simple_recipe(HYD_MotionControlFB* fb) {
    HYD_MotionFBParams params;
    HYD_MotionSegment seg;

    memset(&params, 0, sizeof(params));
    params.maxFlow = 50.0;
    params.maxVelocity = 200.0;
    params.maxAcceleration = 1000.0;
    params.maxDeceleration = 1000.0;
    params.velocityToFlowGain = 0.25;
    params.positionTolerance = 0.5;

    assert(HYD_ActionProfile_BuildClampClose(&seg, &params, 1, 100.0));
    assert(HYD_MotionControlFB_LoadRecipe(fb, &seg, 1));
    fb->FLOW_TO_PUMP_SPEED_GAIN = 1.0;
    fb->PUMP_SPEED_LIMIT = 5000.0;
    fb->USE_RECIPE = true;
}

static void test_abort_recovers_from_fault(void) {
    HYD_MotionControlFB fb;

    HYD_MotionControlFB_Init(&fb);
    prime_fb_with_simple_recipe(&fb);

    /* Step 1: enter RUNNING */
    assert(HYD_MotionControlFB_StartSegment(&fb, 0, 0.0));
    HYD_MotionControlFB_Execute(&fb);
    assert(fb.FB_STATE == HYD_FB_STATE_STARTING || fb.FB_STATE == HYD_FB_STATE_RUNNING);

    /* Step 2: force FAULT by injecting timestamp rollback */
    fb.AXIS_REF.timestamp = -1.0;
    HYD_MotionControlFB_Execute(&fb);
    assert(fb.FB_STATE == HYD_FB_STATE_FAULT);
    assert(fb.STATE.faultActive);

    /* Step 3: Abort() in FAULT state must succeed (this currently FAILS) */
    HYD_BOOL abortAccepted = HYD_MotionControlFB_Abort(&fb);
    assert(abortAccepted);  /* will fail with current implementation */

    HYD_MotionControlFB_Execute(&fb);
    assert(fb.FB_STATE == HYD_FB_STATE_ABORTED);
    assert(!fb.STATE.faultActive);

    printf("test_abort_recovers_from_fault PASSED\n");
}

int main(void) {
    test_abort_recovers_from_fault();
    return 0;
}
```

- [ ] **Step 2.2: 注册测试到 CMakeLists.txt（在测试注册块追加）**

```cmake
add_executable(test_fault_recovery tests/test_fault_recovery.c)
target_link_libraries(test_fault_recovery PRIVATE HydroMotionLib)
add_test(NAME test_fault_recovery
         COMMAND test_fault_recovery
         WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
```

- [ ] **Step 2.3: 配置 + 构建 + 跑测试，确认失败**

```bash
cmake --preset unixgcc
cmake --build --preset unixgcc --target test_fault_recovery
ctest --test-dir out/build/unixgcc -R '^test_fault_recovery$' --output-on-failure
```

Expected：测试 FAIL，断言 `abortAccepted` 失败（因为当前 `HYD_RequestAbortCommand` 在 `faultActive=true` 时返回 false）。

- [ ] **Step 2.4: 修复 mask 表（`src/motion_control.c:74-81`）**

将原 `HYD_CMD_ABORT` mask 行改为：

```c
    [HYD_CMD_ABORT] = HYD_FB_STATE_MASK_BIT(HYD_FB_STATE_IDLE) |
        HYD_FB_STATE_MASK_BIT(HYD_FB_STATE_READY) |
        HYD_FB_STATE_MASK_BIT(HYD_FB_STATE_STARTING) |
        HYD_FB_STATE_MASK_BIT(HYD_FB_STATE_RUNNING) |
        HYD_FB_STATE_MASK_BIT(HYD_FB_STATE_SEGMENT_COMPLETE) |
        HYD_FB_STATE_MASK_BIT(HYD_FB_STATE_DONE) |
        HYD_FB_STATE_MASK_BIT(HYD_FB_STATE_ABORTED) |
        HYD_FB_STATE_MASK_BIT(HYD_FB_STATE_HOLD) |
        HYD_FB_STATE_MASK_BIT(HYD_FB_STATE_FAULT),
```

（新增最后一行 `HYD_FB_STATE_FAULT`。）

- [ ] **Step 2.5: 修复 `HYD_RequestAbortCommand` — 不要在 `faultActive` 时提前拒绝**

定位 `src/motion_control.c` 中 `HYD_RequestAbortCommand`（用 grep）：

```bash
grep -n "HYD_RequestAbortCommand" src/motion_control.c | head -5
```

找到函数体内类似下面的早返回逻辑（行号附近 1818-1820）：

```c
    if (fb->STATE.faultActive) {
        return false;
    }
```

**删除**这两行（或改为仅 mask 判断）。完整函数应只依赖 `HYD_IsCommandAllowedInState` + `HYD_QueuePendingCommand`，不要额外用 `faultActive` 拒绝。

- [ ] **Step 2.6: 确认 ABORT 在 FAULT 态消费时正确进入 ABORTED**

在 `src/motion_control.c` 中找到 ABORT 消费分支（`HYD_CMD_ABORT` 的 case，约第 950-1000 行）。当前实现假设进入 ABORT 时 `faultActive=false`。修改为：在 ABORT 处理时**先清 faultActive**再走 abort 流程：

```c
        case HYD_CMD_ABORT:
            if (fb->STATE.faultActive) {
                /* FAULT->ABORTED transition: clear fault before normal abort handling */
                fb->STATE.faultActive = false;
                HYD_StateReporter_ResetDiagnosticRetention(fb);
            }
            HYD_MotionControlFB_HandleAbort(fb);
            break;
```

（具体修改前先 `grep -n "case HYD_CMD_ABORT" src/motion_control.c` 定位准确位置，并仔细对照已有 `HandleAbort` 是否需要补 reporter 调用。）

- [ ] **Step 2.7: 构建 + 跑测试**

```bash
cmake --build --preset unixgcc --target test_fault_recovery
ctest --test-dir out/build/unixgcc -R '^test_fault_recovery$' --output-on-failure
```

Expected：测试 PASS。

- [ ] **Step 2.8: 跑全量测试确认无回归**

```bash
ctest --test-dir out/build/unixgcc --output-on-failure
```

Expected：所有测试 PASS。

- [ ] **Step 2.9: 更新 `include/motion_control.h` 第 111 行附近的 ABORT 注释**

将原行 `* - ABORT: IDLE / READY / STARTING / RUNNING / SEGMENT_COMPLETE / HOLD / DONE / ABORTED` 改为：

```c
 * - ABORT: IDLE / READY / STARTING / RUNNING / SEGMENT_COMPLETE / HOLD / DONE / ABORTED / FAULT
```

- [ ] **Step 2.10: 更新 `docs/architecture/motion-runtime-contract.md`**

定位 `### `Abort`` 章节（约第 187-193 行），将 `Allowed runtime states:` 行末追加 ` / HYD_FB_STATE_FAULT`，并在 `Immediate effect:` 后追加一行：

```markdown
- From `HYD_FB_STATE_FAULT`, `Abort` clears the fault state and transitions to `HYD_FB_STATE_ABORTED`. This is the recommended PLC-driven recovery path for transient faults.
```

- [ ] **Step 2.11: 在 `docs/integration/plc-process-layer-integration-guide.md` 追加章节**

文件末尾追加：

```markdown
## Fault Recovery Path

When the runtime enters `HYD_FB_STATE_FAULT`, the PLC process layer has two recovery options:

1. **Abort then Restart** (recommended for transient sensor faults):
   - Drive `HYD_Abort.EXECUTE` rising edge. Runtime clears `STATE.faultActive` and transitions to `HYD_FB_STATE_ABORTED`.
   - PLC may then re-issue `MoveProfile` / `MoveAbsolute` / `MoveVelocity` / `PressureHandle` to start a fresh execution.

2. **Reset then Restart** (recommended for configuration-corrupting faults):
   - Drive `HYD_Reset.EXECUTE` rising edge. Runtime clears all runtime state and returns to `HYD_FB_STATE_READY` (if recipe/direct is preloaded) or `HYD_FB_STATE_IDLE`.
   - PLC reloads recipe / direct segment if needed.

3. **Acknowledge diagnostics** is **not** a fault recovery path. `HYD_AcknowledgeDiagnostics.EXECUTE` only clears retained WARNING-level diagnostics after the live event has cleared; it never transitions the FB out of `FAULT`.
```

- [ ] **Step 2.12: 提交**

```bash
git add src/motion_control.c include/motion_control.h tests/test_fault_recovery.c \
        CMakeLists.txt docs/architecture/motion-runtime-contract.md \
        docs/integration/plc-process-layer-integration-guide.md
git commit -m "$(cat <<'EOF'
fix: allow Abort() to recover from FAULT state

Previously HYD_FB_STATE_FAULT was a dead-end accessible only via direct
RESET signal poke. PLC integrations without a Reset FB instance had no
runtime command to escape fault.

- Add HYD_FB_STATE_FAULT to HYD_CMD_ABORT allowed-state mask
- Drop early-return guard against faultActive in HYD_RequestAbortCommand
- In ABORT consumer: clear faultActive before normal abort handling
- Update motion-runtime-contract.md and plc-process-layer-integration-guide.md
- Add test_fault_recovery.c verifying FAULT -> ABORTED via Abort()

Refs: Sprint 0 spec C-3 (2026-05-21-code-review-and-roadmap-design.md)

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 3: ErrorMonitor 容差感知持续时间（C-7）

**目标：** 把 `HYD_ErrorMonitor_Update` 中"`error != 0.0` 视为激活"改为"`|error| > tolerance` 视为激活"。压力收敛后的浮点抖动不再持续累加 errorDuration，criteria 层的 debounce 与 fault escalation 不再被失真数据污染。

**Files:**
- Modify: `include/diagnostics_monitor.h`（`HYD_ErrorMonitorTolerances` 新结构、`Update` 签名增加 tolerances 参数）
- Modify: `src/diagnostics_monitor.c`（4 个通道激活判定改用 fabs+tolerance）
- Modify: `src/motion_control.c`（调用 `HYD_ErrorMonitor_Update` 处把 segment tolerance 传入）
- Test: `tests/test_diagnostic_monitor.c`（新增 jitter 抑制用例）

### Steps

- [ ] **Step 3.1: 在 `include/diagnostics_monitor.h` 添加 tolerances 结构**

在 `HYD_ErrorMonitor` 结构后、`HYD_ErrorMonitor_Init` 声明前插入：

```c
/* 误差激活判据 — 仅当 |error| 超过 tolerance 时才视为"激活" */
typedef struct {
    HYD_REAL position;
    HYD_REAL velocity;
    HYD_REAL flow;
    HYD_REAL pressure;
} HYD_ErrorMonitorTolerances;
```

修改 `HYD_ErrorMonitor_Update` 签名：

```c
void HYD_ErrorMonitor_Update(HYD_ErrorMonitor* monitor,
                             const HYD_AxisRef* axisRef,
                             const HYD_ExecutionReference* references,
                             const HYD_ErrorMonitorTolerances* tolerances,
                             HYD_TIME currentTime);
```

并同步更新 header 注释：

```c
/*
 * 更新误差监视器
 *
 * 参数：
 * - tolerances: 各通道激活门限。|error| <= tolerance 视为"未激活"，duration 重置。
 *   传 NULL 等价于全 0 tolerance，回退到 != 0.0 行为（仅用于过渡兼容）。
 *
 * 说明：
 * - 计算各项误差并更新统计信息
 * - 跟踪误差持续时间（仅在 |error| > tolerance 时累加）
 * - 更新最大值/最小值/平均值
 */
```

- [ ] **Step 3.2: 写失败测试 — 在 `tests/test_diagnostic_monitor.c` 追加 jitter 抑制用例**

在 `tests/test_diagnostic_monitor.c` 末尾、`main()` 之前插入：

```c
static void test_pressure_jitter_does_not_accumulate_duration(void) {
    HYD_ErrorMonitor monitor;
    HYD_AxisRef axisRef;
    HYD_ExecutionReference refs;
    HYD_ErrorMonitorTolerances tolerances;
    HYD_TIME t;
    int i;

    HYD_ErrorMonitor_Init(&monitor);

    /* Set 0.1 MPa pressure tolerance */
    memset(&tolerances, 0, sizeof(tolerances));
    tolerances.pressure = 0.1;

    /* Reference 10.0 MPa, actual stays at 10.000 ~ 10.05 MPa (jitter under tolerance) */
    memset(&axisRef, 0, sizeof(axisRef));
    memset(&refs, 0, sizeof(refs));
    refs.pressureReference = 10.0;
    axisRef.pressure = 10.05;

    for (i = 0; i < 100; i++) {
        t = (HYD_TIME)(i * 0.01);
        axisRef.pressure = (i % 2 == 0) ? 10.05 : 9.95;  /* alternating jitter */
        HYD_ErrorMonitor_Update(&monitor, &axisRef, &refs, &tolerances, t);
        assert(!monitor.pressureErrorActive);
        assert(monitor.pressureErrorDuration == 0.0);
    }
    printf("test_pressure_jitter_does_not_accumulate_duration PASSED\n");
}

static void test_pressure_real_excursion_accumulates_duration(void) {
    HYD_ErrorMonitor monitor;
    HYD_AxisRef axisRef;
    HYD_ExecutionReference refs;
    HYD_ErrorMonitorTolerances tolerances;
    int i;

    HYD_ErrorMonitor_Init(&monitor);
    memset(&tolerances, 0, sizeof(tolerances));
    tolerances.pressure = 0.1;
    memset(&axisRef, 0, sizeof(axisRef));
    memset(&refs, 0, sizeof(refs));

    refs.pressureReference = 10.0;
    axisRef.pressure = 8.0;  /* 2.0 MPa under reference, way above tolerance */

    for (i = 0; i < 50; i++) {
        HYD_ErrorMonitor_Update(&monitor, &axisRef, &refs, &tolerances, (HYD_TIME)(i * 0.01));
    }
    assert(monitor.pressureErrorActive);
    assert(monitor.pressureErrorDuration > 0.4);  /* ~0.49 expected */
    printf("test_pressure_real_excursion_accumulates_duration PASSED\n");
}
```

并在 `main()` 中追加调用：

```c
    test_pressure_jitter_does_not_accumulate_duration();
    test_pressure_real_excursion_accumulates_duration();
```

- [ ] **Step 3.3: 构建 + 跑测试，确认编译失败（签名不匹配）**

```bash
cmake --build --preset unixgcc --target test_diagnostic_monitor 2>&1 | tail -10
```

Expected：编译错误，提示 `HYD_ErrorMonitor_Update` too few arguments 或 unknown type `HYD_ErrorMonitorTolerances`（因为 header 已声明但 src 还未更新签名）。

- [ ] **Step 3.4: 修改 `src/diagnostics_monitor.c` 实现新签名与 tolerance 逻辑**

将 `HYD_ErrorMonitor_Update` 函数体完整替换为：

```c
void HYD_ErrorMonitor_Update(HYD_ErrorMonitor* monitor,
                             const HYD_AxisRef* axisRef,
                             const HYD_ExecutionReference* references,
                             const HYD_ErrorMonitorTolerances* tolerances,
                             HYD_TIME currentTime) {
    HYD_REAL newPositionError = 0.0;
    HYD_REAL newVelocityError = 0.0;
    HYD_REAL newFlowError = 0.0;
    HYD_REAL newPressureError = 0.0;
    HYD_REAL posTol = 0.0;
    HYD_REAL velTol = 0.0;
    HYD_REAL flowTol = 0.0;
    HYD_REAL pressTol = 0.0;

    if (monitor == NULL || axisRef == NULL || references == NULL) {
        return;
    }

    if (tolerances != NULL) {
        posTol = tolerances->position;
        velTol = tolerances->velocity;
        flowTol = tolerances->flow;
        pressTol = tolerances->pressure;
    }

    /* 压力误差 = 参考压力 - 实测压力 */
    if (HYD_ErrorMonitor_IsFiniteReal(references->pressureReference) &&
        HYD_ErrorMonitor_IsFiniteReal(axisRef->pressure)) {
        newPressureError = references->pressureReference - axisRef->pressure;
        monitor->pressureError = newPressureError;
    }

    /* 流量误差 = 参考流量 - |实测流量| */
    if (HYD_ErrorMonitor_IsFiniteReal(references->flowReference) &&
        HYD_ErrorMonitor_IsFiniteReal(axisRef->flow)) {
        newFlowError = references->flowReference - fabs(axisRef->flow);
        monitor->flowError = newFlowError;
    }

    /* 速度误差 = 参考速度 - 实测速度 */
    if (HYD_ErrorMonitor_IsFiniteReal(references->velocityReference) &&
        HYD_ErrorMonitor_IsFiniteReal(axisRef->velocity)) {
        newVelocityError = references->velocityReference - axisRef->velocity;
        monitor->velocityError = newVelocityError;
    }

    /* 更新压力激活状态（仅在 |error| > tolerance 时） */
    if (fabs(monitor->pressureError) > pressTol) {
        if (!monitor->pressureErrorActive) {
            monitor->pressureErrorActive = true;
            monitor->pressureErrorStartTime = currentTime;
            monitor->pressureErrorDuration = 0.0;
        } else {
            monitor->pressureErrorDuration = currentTime - monitor->pressureErrorStartTime;
        }
    } else {
        monitor->pressureErrorActive = false;
        monitor->pressureErrorDuration = 0.0;
    }

    /* 流量 */
    if (fabs(monitor->flowError) > flowTol) {
        if (!monitor->flowErrorActive) {
            monitor->flowErrorActive = true;
            monitor->flowErrorStartTime = currentTime;
            monitor->flowErrorDuration = 0.0;
        } else {
            monitor->flowErrorDuration = currentTime - monitor->flowErrorStartTime;
        }
    } else {
        monitor->flowErrorActive = false;
        monitor->flowErrorDuration = 0.0;
    }

    /* 速度 */
    if (fabs(monitor->velocityError) > velTol) {
        if (!monitor->velocityErrorActive) {
            monitor->velocityErrorActive = true;
            monitor->velocityErrorStartTime = currentTime;
            monitor->velocityErrorDuration = 0.0;
        } else {
            monitor->velocityErrorDuration = currentTime - monitor->velocityErrorStartTime;
        }
    } else {
        monitor->velocityErrorActive = false;
        monitor->velocityErrorDuration = 0.0;
    }

    /* 位置 */
    if (fabs(monitor->positionError) > posTol) {
        if (!monitor->positionErrorActive) {
            monitor->positionErrorActive = true;
            monitor->positionErrorStartTime = currentTime;
            monitor->positionErrorDuration = 0.0;
        } else {
            monitor->positionErrorDuration = currentTime - monitor->positionErrorStartTime;
        }
    } else {
        monitor->positionErrorActive = false;
        monitor->positionErrorDuration = 0.0;
    }

    /* 统计部分保持不变 */
    if (HYD_ErrorMonitor_IsFiniteReal(newPressureError)) {
        if (monitor->sampleCount == 0) {
            monitor->maxPressureError = newPressureError;
            monitor->minPressureError = newPressureError;
            monitor->avgPressureError = newPressureError;
        } else {
            if (newPressureError > monitor->maxPressureError) {
                monitor->maxPressureError = newPressureError;
            }
            if (newPressureError < monitor->minPressureError) {
                monitor->minPressureError = newPressureError;
            }
            monitor->avgPressureError = monitor->avgPressureError +
                (newPressureError - monitor->avgPressureError) / (HYD_REAL)(monitor->sampleCount + 1);
        }
    }
    if (HYD_ErrorMonitor_IsFiniteReal(newFlowError)) {
        if (monitor->sampleCount == 0) {
            monitor->maxFlowError = newFlowError;
            monitor->minFlowError = newFlowError;
            monitor->avgFlowError = newFlowError;
        } else {
            if (newFlowError > monitor->maxFlowError) {
                monitor->maxFlowError = newFlowError;
            }
            if (newFlowError < monitor->minFlowError) {
                monitor->minFlowError = newFlowError;
            }
            monitor->avgFlowError = monitor->avgFlowError +
                (newFlowError - monitor->avgFlowError) / (HYD_REAL)(monitor->sampleCount + 1);
        }
    }
    if (HYD_ErrorMonitor_IsFiniteReal(newVelocityError)) {
        if (monitor->sampleCount == 0) {
            monitor->maxVelocityError = newVelocityError;
            monitor->minVelocityError = newVelocityError;
            monitor->avgVelocityError = newVelocityError;
        } else {
            if (newVelocityError > monitor->maxVelocityError) {
                monitor->maxVelocityError = newVelocityError;
            }
            if (newVelocityError < monitor->minVelocityError) {
                monitor->minVelocityError = newVelocityError;
            }
            monitor->avgVelocityError = monitor->avgVelocityError +
                (newVelocityError - monitor->avgVelocityError) / (HYD_REAL)(monitor->sampleCount + 1);
        }
    }
    if (HYD_ErrorMonitor_IsFiniteReal(newPositionError)) {
        if (monitor->sampleCount == 0) {
            monitor->maxPositionError = newPositionError;
            monitor->minPositionError = newPositionError;
            monitor->avgPositionError = newPositionError;
        } else {
            if (newPositionError > monitor->maxPositionError) {
                monitor->maxPositionError = newPositionError;
            }
            if (newPositionError < monitor->minPositionError) {
                monitor->minPositionError = newPositionError;
            }
            monitor->avgPositionError = monitor->avgPositionError +
                (newPositionError - monitor->avgPositionError) / (HYD_REAL)(monitor->sampleCount + 1);
        }
    }
    monitor->sampleCount++;
}
```

- [ ] **Step 3.5: 在 `src/motion_control.c` 中所有 `HYD_ErrorMonitor_Update` 调用点补 tolerances 参数**

```bash
grep -n "HYD_ErrorMonitor_Update" src/motion_control.c
```

每个调用点（应有 1-2 处，位于 `HYD_UpdateExecutionDiagnostics` 内）改为：

```c
        HYD_ErrorMonitorTolerances tolerances;
        const HYD_MotionSegment* seg = fb->_activeSegmentValid ? &fb->_activeSegment : NULL;
        tolerances.position = (seg != NULL) ? HYD_Segment_GetPositionTolerance(seg) : 0.0;
        tolerances.velocity = (seg != NULL) ? HYD_Segment_GetVelocityTolerance(seg) : 0.0;
        tolerances.flow     = (seg != NULL) ? HYD_Segment_GetFlowTolerance(seg) : 0.0;
        tolerances.pressure = (seg != NULL) ? HYD_Segment_GetPressureTolerance(seg) : 0.0;
        HYD_ErrorMonitor_Update(&fb->_errorMonitor, &fb->AXIS_REF, &references, &tolerances, currentTime);
```

确保 `#include "segment_limits.h"` 已在 `src/motion_control.c` 顶部存在（如未存在则添加）。

- [ ] **Step 3.6: 构建 + 跑新测试 + 全量测试**

```bash
cmake --build --preset unixgcc 2>&1 | tail -10
ctest --test-dir out/build/unixgcc -R '^test_diagnostic_monitor$' --output-on-failure
ctest --test-dir out/build/unixgcc --output-on-failure
```

Expected：
- 新增 2 个用例 PASS
- 全量 PASS（其他测试可能间接调用旧签名，需要修复）

如果其他测试因签名变化而构建失败：定位每个失败处，按 Step 3.5 的模式补 tolerances 参数（既存测试用 `NULL` 或零 tolerance 等价旧行为，确保现有行为不被破坏）。

- [ ] **Step 3.7: 提交**

```bash
git add include/diagnostics_monitor.h src/diagnostics_monitor.c src/motion_control.c \
        tests/test_diagnostic_monitor.c
git commit -m "$(cat <<'EOF'
fix: ErrorMonitor activates duration only when |error| > tolerance

Previously `pressureError != 0.0` (and equivalent for flow/velocity/position)
caused errorDuration to accumulate from sub-tolerance floating-point jitter.
The diagnostic-criteria layer's debounce + WARNING→FAULT escalation
mechanisms received polluted duration data and could trip on noise.

- Add HYD_ErrorMonitorTolerances struct (position/velocity/flow/pressure)
- HYD_ErrorMonitor_Update gains tolerances parameter; |error|<=tol resets
- motion_control.c passes segment tolerances at every Update call
- New tests verify jitter suppression and real-excursion accumulation

Refs: Sprint 0 spec C-7 (2026-05-21-code-review-and-roadmap-design.md)

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 4: STOP 减速分支保留最小安全检查（C-4）

**目标：** `_isStopping` 分支不再静默吞掉 sensor fault / time-rollback / timeout，避免 STOP 过程中 FB 永久卡在 stopping 状态。

**Files:**
- Modify: `src/motion_control.c:1603-1651`（`_isStopping` 分支）
- Test: `tests/test_stop_diagnostics_retained.c`（新建）

### Steps

- [ ] **Step 4.1: 写失败测试 `tests/test_stop_diagnostics_retained.c`**

```c
/* tests/test_stop_diagnostics_retained.c
 * Verifies that fault detection (sensor / time-rollback / timeout) remains
 * active during the STOP deceleration branch. */
#include "motion_control.h"
#include "action_profile.h"
#include <assert.h>
#include <stdio.h>
#include <math.h>
#include <string.h>

static void prime_velocity_run(HYD_MotionControlFB* fb) {
    HYD_MotionFBParams params;
    HYD_MotionSegment seg;
    memset(&params, 0, sizeof(params));
    params.maxFlow = 50.0;
    params.maxVelocity = 100.0;
    params.maxAcceleration = 500.0;
    params.maxDeceleration = 500.0;
    params.velocityToFlowGain = 0.25;
    params.velocityTolerance = 1.0;

    memset(&seg, 0, sizeof(seg));
    seg.segmentTag = 1;
    seg.segmentType = HYD_SEGMENT_TYPE_OTHER;
    seg.planner = HYD_PLANNER_TIME_BASED;
    seg.mode = HYD_MODE_SPEED_RAMP;
    seg.endCondition = HYD_END_MANUAL;
    seg.direction = HYD_DIRECTION_EXTEND;
    seg.maxVelocity = 80.0;
    seg.maxFlow = 50.0;
    seg.maxAcceleration = 500.0;
    seg.maxDeceleration = 500.0;
    seg.velocityToFlowGain = 0.25;
    seg.duration = 10.0;
    seg.timeoutLimit = 5.0;

    assert(HYD_MotionControlFB_LoadDirectSegment(fb, &seg));
    fb->USE_RECIPE = false;
    fb->FLOW_TO_PUMP_SPEED_GAIN = 1.0;
    fb->PUMP_SPEED_LIMIT = 5000.0;
}

static void test_stop_branch_detects_timestamp_rollback(void) {
    HYD_MotionControlFB fb;
    HYD_MotionControlFB_Init(&fb);
    prime_velocity_run(&fb);

    /* Run velocity segment for a bit */
    fb.AXIS_REF.timestamp = 0.0;
    assert(HYD_MotionControlFB_StartSegment(&fb, 0, 0.0));
    for (int i = 0; i < 20; i++) {
        fb.AXIS_REF.timestamp = (HYD_TIME)(i * 0.01);
        fb.AXIS_REF.velocity = 50.0;
        HYD_MotionControlFB_Execute(&fb);
    }
    assert(fb.STATE.active);

    /* Initiate Stop */
    assert(HYD_MotionControlFB_Stop(&fb, 0.20, 200.0));
    HYD_MotionControlFB_Execute(&fb);
    /* fb is now in stopping branch */

    /* Inject timestamp rollback */
    fb.AXIS_REF.timestamp = -1.0;
    HYD_MotionControlFB_Execute(&fb);

    /* Expect: stop branch must transition to FAULT (currently silently continues) */
    assert(fb.FB_STATE == HYD_FB_STATE_FAULT);
    assert(fb.STATE.faultActive);
    printf("test_stop_branch_detects_timestamp_rollback PASSED\n");
}

int main(void) {
    test_stop_branch_detects_timestamp_rollback();
    return 0;
}
```

注册到 CMakeLists.txt：

```cmake
add_executable(test_stop_diagnostics_retained tests/test_stop_diagnostics_retained.c)
target_link_libraries(test_stop_diagnostics_retained PRIVATE HydroMotionLib)
add_test(NAME test_stop_diagnostics_retained
         COMMAND test_stop_diagnostics_retained
         WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
```

- [ ] **Step 4.2: Re-configure + 构建 + 跑测试，确认失败**

```bash
cmake --preset unixgcc
cmake --build --preset unixgcc --target test_stop_diagnostics_retained
ctest --test-dir out/build/unixgcc -R '^test_stop_diagnostics_retained$' --output-on-failure
```

Expected：测试 FAIL（注入 rollback 后 STOP 分支静默继续，未进 FAULT）。

- [ ] **Step 4.3: 定位 STOP 分支起点**

```bash
grep -n "_isStopping" src/motion_control.c | head -20
```

找到 `HYD_MotionControlFB_RunRunningState` 内进入 stopping 的分支起点（约 1603 行）。完整阅读 1603-1651 行。

- [ ] **Step 4.4: 抽取并在 STOP 分支前先做 sensor / timestamp / timeout 检查**

在 `_isStopping` 分支开始处（即 `if (fb->_isStopping)` 之后第一行），插入：

```c
    /* Even during controlled-stop deceleration, sensor / timestamp / timeout
     * faults must still escalate. Otherwise STOP can hang forever on broken
     * feedback. */
    {
        const HYD_AxisRef* ar = &fb->AXIS_REF;
        if (ar->timestamp < 0.0 ||
            (fb->_lastFeedbackTimestamp >= 0.0 && ar->timestamp < fb->_lastFeedbackTimestamp)) {
            HYD_StateReporter_ReportFault(fb, HYD_DIAG_CODE_TIMESTAMP_ROLLBACK);
            HYD_ProtectionManager_EnterFaultStop(fb);
            return;
        }
        if (!isfinite(ar->position) || !isfinite(ar->velocity) ||
            !isfinite(ar->pressure) || !isfinite(ar->flow)) {
            HYD_StateReporter_ReportFault(fb, HYD_DIAG_CODE_SENSOR_FAULT);
            HYD_ProtectionManager_EnterFaultStop(fb);
            return;
        }
        /* Stop-specific timeout: stop must finish within e.g. 5x maxDecel time */
        if (fb->_stopStartTime > 0.0 && fb->_stopDeceleration > 0.0) {
            HYD_TIME stopElapsed = ar->timestamp - fb->_stopStartTime;
            HYD_REAL minStopTime = fabs(fb->_stopStartVel) / fb->_stopDeceleration;
            if (stopElapsed > 5.0 * minStopTime + 1.0) {
                HYD_StateReporter_ReportFault(fb, HYD_DIAG_CODE_TIMEOUT);
                HYD_ProtectionManager_EnterFaultStop(fb);
                return;
            }
        }
        fb->_lastFeedbackTimestamp = ar->timestamp;
    }
```

确保 `#include <math.h>` 已存在（用于 isfinite）。

- [ ] **Step 4.5: 构建 + 跑新测试 + 全量回归**

```bash
cmake --build --preset unixgcc --target test_stop_diagnostics_retained
ctest --test-dir out/build/unixgcc -R '^test_stop_diagnostics_retained$' --output-on-failure
ctest --test-dir out/build/unixgcc --output-on-failure
```

Expected：新测试 PASS、其他测试不回归。

- [ ] **Step 4.6: 追加正常 STOP 不误报的用例**

在 `test_stop_diagnostics_retained.c` 中追加：

```c
static void test_normal_stop_does_not_false_alarm(void) {
    HYD_MotionControlFB fb;
    HYD_MotionControlFB_Init(&fb);
    prime_velocity_run(&fb);

    fb.AXIS_REF.timestamp = 0.0;
    assert(HYD_MotionControlFB_StartSegment(&fb, 0, 0.0));
    for (int i = 0; i < 20; i++) {
        fb.AXIS_REF.timestamp = (HYD_TIME)(i * 0.01);
        fb.AXIS_REF.velocity = 50.0;
        HYD_MotionControlFB_Execute(&fb);
    }

    assert(HYD_MotionControlFB_Stop(&fb, 0.20, 200.0));
    /* Run stop to completion */
    for (int i = 20; i < 200; i++) {
        fb.AXIS_REF.timestamp = (HYD_TIME)(i * 0.01);
        if (fb.PUMP_SPEED > 0.0) {
            fb.AXIS_REF.velocity *= 0.9;
        } else {
            fb.AXIS_REF.velocity = 0.0;
        }
        HYD_MotionControlFB_Execute(&fb);
        if (fb.FB_STATE == HYD_FB_STATE_DONE) break;
    }
    assert(fb.FB_STATE == HYD_FB_STATE_DONE);
    assert(!fb.STATE.faultActive);
    printf("test_normal_stop_does_not_false_alarm PASSED\n");
}
```

并在 `main()` 追加调用。

```bash
cmake --build --preset unixgcc --target test_stop_diagnostics_retained
ctest --test-dir out/build/unixgcc -R '^test_stop_diagnostics_retained$' --output-on-failure
```

- [ ] **Step 4.7: 提交**

```bash
git add src/motion_control.c tests/test_stop_diagnostics_retained.c CMakeLists.txt
git commit -m "$(cat <<'EOF'
fix: retain critical fault checks inside STOP deceleration branch

Previously HYD_MotionControlFB_RunRunningState's _isStopping branch
returned early without sensor/timestamp/timeout checks, so STOP could
hang forever on a stuck or rolling-back feedback signal.

- Add timestamp-rollback, sensor-finiteness, and stop-timeout checks
  at the top of the stopping branch
- Stop timeout = 5x ideal-deceleration time + 1s slack
- New test_stop_diagnostics_retained.c covers fault injection during stop
  and the normal-stop-no-false-alarm case

Refs: Sprint 0 spec C-4 (2026-05-21-code-review-and-roadmap-design.md)

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 5: 补充 state_reporter / protection_manager / diagnostics 单元测试（I-7）

**目标：** 给三个核心模块（合计 1115 行实现）建立独立单元测试，把覆盖率从"仅集成路径"提升到"语义可被锁定"。这是 Sprint 1+ 大改动的安全网。

**Files:**
- Create: `tests/test_state_reporter.c`
- Create: `tests/test_protection_manager.c`
- Create: `tests/test_diagnostics.c`
- Modify: `CMakeLists.txt`（注册 3 个新测试）

### Steps

- [ ] **Step 5.1: 阅读 state_reporter.h 公共 API**

```bash
grep -E "^void HYD_StateReporter_|^HYD_BOOL HYD_StateReporter_" include/state_reporter.h
```

Expected：列出 `SetIdleState`、`SetFbState`、`SetStatus`、`SetProtectionAction`、`ApplySafeOutputs`、`ResetTransitionFlags`、`EnterFaultState`、`ReportFault`、`ClearCurrentDiagnostic`、`ResetDiagnosticRetention`、`PushDiagnosticHistory` 等公共符号。

- [ ] **Step 5.2: 创建 `tests/test_state_reporter.c`**

```c
/* tests/test_state_reporter.c - Unit tests for state_reporter module */
#include "motion_control.h"
#include "state_reporter.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static void test_apply_safe_outputs_zeros_pump_speed(void) {
    HYD_MotionControlFB fb;
    HYD_MotionControlFB_Init(&fb);
    fb.PUMP_SPEED = 1234.5;
    fb.STATE.commandedPumpSpeed = 1234.5;
    fb.STATE.plannedVelocity = 50.0;
    fb.STATE.plannedFlow = 30.0;

    HYD_StateReporter_ApplySafeOutputs(&fb);

    assert(fb.PUMP_SPEED == 0.0);
    assert(fb.STATE.commandedPumpSpeed == 0.0);
    assert(fb.STATE.plannedVelocity == 0.0);
    assert(fb.STATE.plannedFlow == 0.0);
    printf("test_apply_safe_outputs_zeros_pump_speed PASSED\n");
}

static void test_set_fb_state_propagates(void) {
    HYD_MotionControlFB fb;
    HYD_MotionControlFB_Init(&fb);
    HYD_StateReporter_SetFbState(&fb, HYD_FB_STATE_RUNNING);
    assert(fb.FB_STATE == HYD_FB_STATE_RUNNING);
    HYD_StateReporter_SetFbState(&fb, HYD_FB_STATE_FAULT);
    assert(fb.FB_STATE == HYD_FB_STATE_FAULT);
    printf("test_set_fb_state_propagates PASSED\n");
}

static void test_report_fault_sets_diagnostic_and_state(void) {
    HYD_MotionControlFB fb;
    HYD_MotionControlFB_Init(&fb);
    HYD_StateReporter_ReportFault(&fb, HYD_DIAG_CODE_OVER_PRESSURE);

    assert(fb.STATE.faultActive);
    assert(fb.DIAGNOSTIC.code == HYD_DIAG_CODE_OVER_PRESSURE);
    assert(fb.DIAGNOSTIC.severity == HYD_DIAG_SEVERITY_FAULT);
    assert(fb.ERROR_ID == HYD_DIAG_CODE_OVER_PRESSURE);
    printf("test_report_fault_sets_diagnostic_and_state PASSED\n");
}

static void test_clear_diagnostic_preserves_history(void) {
    HYD_MotionControlFB fb;
    HYD_MotionControlFB_Init(&fb);
    HYD_StateReporter_ReportFault(&fb, HYD_DIAG_CODE_TIMEOUT);
    HYD_StateReporter_PushDiagnosticHistory(&fb);

    /* clear live but keep retention */
    HYD_StateReporter_ClearCurrentDiagnostic(&fb);
    assert(fb.DIAGNOSTIC.code == HYD_DIAG_CODE_NONE);
    assert(fb.DIAGNOSTIC_HISTORY.hasRecord);
    assert(fb.DIAGNOSTIC_HISTORY.lastSnapshot.diagnostic.code == HYD_DIAG_CODE_TIMEOUT);
    printf("test_clear_diagnostic_preserves_history PASSED\n");
}

static void test_set_idle_state_with_finished_flag(void) {
    HYD_MotionControlFB fb;
    HYD_MotionControlFB_Init(&fb);
    HYD_StateReporter_SetIdleState(&fb, /*finished*/ true, /*segCompleted*/ true);
    assert(!fb.STATE.active);
    assert(fb.STATE.finished);
    assert(fb.SEGMENT_COMPLETED);
    printf("test_set_idle_state_with_finished_flag PASSED\n");
}

int main(void) {
    test_apply_safe_outputs_zeros_pump_speed();
    test_set_fb_state_propagates();
    test_report_fault_sets_diagnostic_and_state();
    test_clear_diagnostic_preserves_history();
    test_set_idle_state_with_finished_flag();
    return 0;
}
```

- [ ] **Step 5.3: 创建 `tests/test_protection_manager.c`**

```c
/* tests/test_protection_manager.c - Unit tests for runtime safety state */
#include "motion_control.h"
#include "protection_manager.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static void test_reset_runtime_actuation_clears_pressure_state(void) {
    HYD_MotionControlFB fb;
    HYD_MotionControlFB_Init(&fb);
    fb._lastFeedbackTimestamp = 1.234;
    fb._segmentStartTime = 5.0;
    fb._activeSegmentValid = true;
    fb._activeSegmentSource = HYD_SEGMENT_SOURCE_DIRECT;

    HYD_ProtectionManager_ResetRuntimeActuation(&fb);

    assert(fb._lastFeedbackTimestamp < 0.0);
    assert(fb._segmentStartTime == 0.0);
    assert(!fb._activeSegmentValid);
    assert(fb._activeSegmentSource == HYD_SEGMENT_SOURCE_NONE);
    printf("test_reset_runtime_actuation_clears_pressure_state PASSED\n");
}

static void test_apply_disabled_state_with_loaded_recipe_reports_ready(void) {
    HYD_MotionControlFB fb;
    HYD_MotionSegment seg;
    HYD_MotionControlFB_Init(&fb);
    memset(&seg, 0, sizeof(seg));
    seg.segmentTag = 1;
    seg.mode = HYD_MODE_POSITION;
    seg.endCondition = HYD_END_POSITION;
    seg.direction = HYD_DIRECTION_EXTEND;
    seg.targetPosition = 100.0;
    seg.maxVelocity = 50.0;
    seg.maxAcceleration = 200.0;
    seg.maxFlow = 30.0;
    seg.velocityToFlowGain = 0.25;
    seg.positionTolerance = 0.5;
    HYD_MotionControlFB_LoadRecipe(&fb, &seg, 1);
    fb.USE_RECIPE = true;

    HYD_ProtectionManager_ApplyDisabledState(&fb);
    assert(fb.FB_STATE == HYD_FB_STATE_DISABLED);
    assert(fb.STATE.status == HYD_STATUS_READY);
    assert(fb.PUMP_SPEED == 0.0);
    printf("test_apply_disabled_state_with_loaded_recipe_reports_ready PASSED\n");
}

static void test_apply_disabled_state_without_source_reports_idle(void) {
    HYD_MotionControlFB fb;
    HYD_MotionControlFB_Init(&fb);
    HYD_ProtectionManager_ApplyDisabledState(&fb);
    assert(fb.FB_STATE == HYD_FB_STATE_DISABLED);
    assert(fb.STATE.status == HYD_STATUS_IDLE);
    printf("test_apply_disabled_state_without_source_reports_idle PASSED\n");
}

static void test_apply_fault_hold_re_applies_safe_outputs(void) {
    HYD_MotionControlFB fb;
    HYD_MotionControlFB_Init(&fb);
    fb.PUMP_SPEED = 999.0;
    fb._lastCommandedFlow = 50.0;
    HYD_ProtectionManager_ApplyFaultHold(&fb);
    assert(fb.PUMP_SPEED == 0.0);
    assert(fb._lastCommandedFlow == 0.0);
    assert(fb.STATE.protectionAction == HYD_PROTECTION_ACTION_STOP);
    assert(fb.FB_STATE == HYD_FB_STATE_FAULT);
    printf("test_apply_fault_hold_re_applies_safe_outputs PASSED\n");
}

int main(void) {
    test_reset_runtime_actuation_clears_pressure_state();
    test_apply_disabled_state_with_loaded_recipe_reports_ready();
    test_apply_disabled_state_without_source_reports_idle();
    test_apply_fault_hold_re_applies_safe_outputs();
    return 0;
}
```

- [ ] **Step 5.4: 创建 `tests/test_diagnostics.c`**

```c
/* tests/test_diagnostics.c - Unit tests for diagnostic-code → spec table */
#include "diagnostics.h"
#include "common_types.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static void test_diag_spec_returns_fault_severity_for_over_pressure(void) {
    HYD_DiagnosticInfo info;
    memset(&info, 0, sizeof(info));
    HYD_Diagnostics_PopulateFromCode(&info, HYD_DIAG_CODE_OVER_PRESSURE);
    assert(info.code == HYD_DIAG_CODE_OVER_PRESSURE);
    assert(info.severity == HYD_DIAG_SEVERITY_FAULT);
    assert(info.protectionAction == HYD_PROTECTION_ACTION_STOP);
    printf("test_diag_spec_returns_fault_severity_for_over_pressure PASSED\n");
}

static void test_diag_spec_returns_warning_for_position_deviation(void) {
    HYD_DiagnosticInfo info;
    memset(&info, 0, sizeof(info));
    HYD_Diagnostics_PopulateFromCode(&info, HYD_DIAG_CODE_POSITION_DEVIATION);
    assert(info.code == HYD_DIAG_CODE_POSITION_DEVIATION);
    assert(info.severity == HYD_DIAG_SEVERITY_WARNING);
    printf("test_diag_spec_returns_warning_for_position_deviation PASSED\n");
}

static void test_diag_clear_zeros_all_fields(void) {
    HYD_DiagnosticInfo info;
    memset(&info, 0xAA, sizeof(info));
    HYD_Diagnostics_Clear(&info);
    assert(info.code == HYD_DIAG_CODE_NONE);
    assert(info.severity == HYD_DIAG_SEVERITY_NONE);
    assert(info.protectionAction == HYD_PROTECTION_ACTION_NONE);
    assert(!info.overPressure);
    assert(!info.timeout);
    printf("test_diag_clear_zeros_all_fields PASSED\n");
}

int main(void) {
    test_diag_spec_returns_fault_severity_for_over_pressure();
    test_diag_spec_returns_warning_for_position_deviation();
    test_diag_clear_zeros_all_fields();
    return 0;
}
```

**注意：** 实际 `diagnostics.h` 的公共函数名以代码为准。先 `grep -E "^void HYD_Diagnostics_|^HYD_BOOL HYD_Diagnostics_" include/diagnostics.h` 确认；若 `HYD_Diagnostics_PopulateFromCode`、`HYD_Diagnostics_Clear` 不存在但有等价 API，相应改名。**这个步骤要先看 header**。

- [ ] **Step 5.5: 注册 3 个测试到 CMakeLists.txt**

```cmake
add_executable(test_state_reporter tests/test_state_reporter.c)
target_link_libraries(test_state_reporter PRIVATE HydroMotionLib)
add_test(NAME test_state_reporter
         COMMAND test_state_reporter
         WORKING_DIRECTORY ${CMAKE_BINARY_DIR})

add_executable(test_protection_manager tests/test_protection_manager.c)
target_link_libraries(test_protection_manager PRIVATE HydroMotionLib)
add_test(NAME test_protection_manager
         COMMAND test_protection_manager
         WORKING_DIRECTORY ${CMAKE_BINARY_DIR})

add_executable(test_diagnostics tests/test_diagnostics.c)
target_link_libraries(test_diagnostics PRIVATE HydroMotionLib)
add_test(NAME test_diagnostics
         COMMAND test_diagnostics
         WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
```

- [ ] **Step 5.6: 配置 + 构建 + 跑测试**

```bash
cmake --preset unixgcc
cmake --build --preset unixgcc 2>&1 | tail -20
ctest --test-dir out/build/unixgcc -R 'state_reporter|protection_manager|diagnostics' --output-on-failure
```

Expected：
- 如果 `HYD_Diagnostics_PopulateFromCode` 等 API 不存在但等价物存在 → 调整测试代码到实际 API；
- 如果 `HYD_StateReporter_PushDiagnosticHistory` 是私有的 → 看 header 重命名；
- 修复编译错误直到 3 个测试全部 PASS。

- [ ] **Step 5.7: 跑全量回归**

```bash
ctest --test-dir out/build/unixgcc --output-on-failure
```

Expected：全部 PASS。

- [ ] **Step 5.8: 提交**

```bash
git add tests/test_state_reporter.c tests/test_protection_manager.c \
        tests/test_diagnostics.c CMakeLists.txt
git commit -m "$(cat <<'EOF'
test: add unit tests for state_reporter / protection_manager / diagnostics

These three modules (1115 lines combined) previously had no dedicated
unit coverage and relied on integration paths. Sprint 1+ refactors will
need them as a safety net.

- test_state_reporter.c: ApplySafeOutputs, SetFbState, ReportFault,
  ClearCurrentDiagnostic + history retention, SetIdleState transitions
- test_protection_manager.c: ResetRuntimeActuation, ApplyDisabledState
  (READY vs IDLE branching), ApplyFaultHold safety outputs
- test_diagnostics.c: spec-table lookups for fault/warning severities,
  Clear() zero-fill

Refs: Sprint 0 spec I-7 (2026-05-21-code-review-and-roadmap-design.md)

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 6: HYD_AXISMOTION 半区隔离（C-1）

**目标：** 引入"Setpoint 半区（PLC→runtime，runtime 只读）"和"Actual 半区（runtime→PLC，PLC 只读）"的契约。Runtime 不再写 Setpoint 字段；PLC 不再被指望写 Act 字段。多个 MoveProfile FB 共用同轴时不再静默互相覆盖。

**策略：** 不改 `HYD_AXISMOTION` 的物理布局（避免破坏 XML/header 同步契约），通过**审计 + 测试 + 文档**约束行为；并在 `motion_interface.c` 中**移除 runtime 反写 Setpoint 字段的代码**。

**Files:**
- Modify: `src/motion_interface.c`（移除 `writeMotionFromSegment` 对 setpoint 字段的写入；只写 act 字段）
- Modify: `include/motion_interface.h`（追加契约注释）
- Modify: `docs/integration/plc-process-layer-integration-guide.md`（HYD_AXISMOTION 半区契约）
- Test: `tests/test_axismotion_setpoint_isolation.c`（新建）

### Steps

- [ ] **Step 6.1: 列出 `HYD_AXISMOTION` 所有字段并标注 Setpoint vs Actual**

```bash
grep -A 40 "typedef struct.*HYD_AXISMOTION" include/motion_interface.h | head -50
```

阅读输出，把字段分两类：

- **Setpoint（PLC→runtime）**：targetPosition、targetVelocity、targetFlow、targetPressure、maxVelocity、maxAcceleration、maxDeceleration、maxFlow、duration、各类 tolerance、segmentTag、segmentType、planner、mode、endCondition、direction 等
- **Actual（runtime→PLC）**：ACTPOSITION、ACTVELOCITY、ACTFLOW、ACTPRESSURE、TIMESTAMP

把分类结果在本 plan 旁边记下来（用 grep 输出粘贴）。

- [ ] **Step 6.2: 阅读 `writeMotionFromSegment` 实现**

```bash
grep -n "writeMotionFromSegment" src/motion_interface.c
```

打开找到的函数，列出所有 `motion->XXX = ...` 写入语句，识别哪些写到了 Setpoint 字段。

- [ ] **Step 6.3: 写失败测试 `tests/test_axismotion_setpoint_isolation.c`**

```c
/* tests/test_axismotion_setpoint_isolation.c
 * Verifies that runtime never overwrites Setpoint fields in HYD_AXISMOTION. */
#include "motion_interface.h"
#include "motion_control.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

extern HYD_MotionControlFB *__MK_GetPublic_MotionControlFB(int axisIndex);
extern void __HydMotion_framework_Init(void);

static void test_runtime_does_not_overwrite_setpoint_fields(void) {
    HYD_MOVEPROFILE fb_inst;
    HYD_AXISMOTION sentinel;
    HYD_AXISMOTION readback;

    __HydMotion_framework_Init();
    memset(&fb_inst, 0, sizeof(fb_inst));

    /* PLC writes Setpoint fields with sentinel values */
    memset(&sentinel, 0, sizeof(sentinel));
    sentinel.TARGETPOSITION = 123.456;
    sentinel.MAXVELOCITY = 78.9;
    sentinel.MAXACCELERATION = 1234.5;
    sentinel.MAXFLOW = 67.8;
    sentinel.TARGETPRESSURE = 9.876;
    sentinel.ACTPOSITION = 5.0;       /* Actual field, runtime may overwrite */
    sentinel.TIMESTAMP = 0.0;

    __SET_VAR(fb_inst., MOTION,, sentinel);
    __SET_VAR(fb_inst., EXECUTE,, true);
    __SET_VAR(fb_inst., AXISID,, 0);
    __SET_VAR(fb_inst., BUFFERMODE,, HYD_BUFFER_MODE_ABORT);

    /* Set USE_SIMULATION so runtime ticks AXIS_REF internally */
    HYD_MotionControlFB *fb = __MK_GetPublic_MotionControlFB(0);
    assert(fb != NULL);
    fb->_useSimulation = true;

    /* One scan */
    __mcl_cmd_MoveProfile(&fb_inst);

    /* Read back MOTION; Setpoint fields must equal sentinel exactly */
    readback = __GET_VAR(fb_inst.MOTION);
    assert(readback.TARGETPOSITION == 123.456);
    assert(readback.MAXVELOCITY == 78.9);
    assert(readback.MAXACCELERATION == 1234.5);
    assert(readback.MAXFLOW == 67.8);
    assert(readback.TARGETPRESSURE == 9.876);

    /* Actual fields are allowed to differ (runtime may have written) */
    printf("test_runtime_does_not_overwrite_setpoint_fields PASSED\n");
}

int main(void) {
    test_runtime_does_not_overwrite_setpoint_fields();
    return 0;
}
```

**注意：** 字段名 `TARGETPOSITION` 等需以实际 `HYD_AXISMOTION` 结构为准；先 grep 头文件确认大小写与下划线规则。

- [ ] **Step 6.4: 注册测试**

```cmake
add_executable(test_axismotion_setpoint_isolation tests/test_axismotion_setpoint_isolation.c)
target_link_libraries(test_axismotion_setpoint_isolation PRIVATE HydroMotionLib)
add_test(NAME test_axismotion_setpoint_isolation
         COMMAND test_axismotion_setpoint_isolation
         WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
```

- [ ] **Step 6.5: 构建 + 跑测试，确认失败**

```bash
cmake --preset unixgcc
cmake --build --preset unixgcc --target test_axismotion_setpoint_isolation
ctest --test-dir out/build/unixgcc -R '^test_axismotion_setpoint_isolation$' --output-on-failure
```

Expected：FAIL — 因为当前 `writeMotionFromSegment` 会把 Setpoint 字段也写回。

- [ ] **Step 6.6: 修改 `writeMotionFromSegment` 只写 Actual**

定位 `writeMotionFromSegment` 实现。把所有写 `motion->TARGET*`、`motion->MAX*`、`motion->TIMEOUT*` 之类 Setpoint 字段的语句**删除**。保留只写 `motion->ACT*` 系列。完整修改后函数体应只包含：

```c
static void writeMotionFromSegment(HYD_AXISMOTION* motion, const HYD_MotionControlFB* fb) {
    if (motion == NULL || fb == NULL) {
        return;
    }
    /* Actual half — runtime → PLC */
    motion->ACTPOSITION = (IEC_REAL)fb->AXIS_REF.position;
    motion->ACTVELOCITY = (IEC_REAL)fb->AXIS_REF.velocity;
    motion->ACTFLOW = (IEC_REAL)fb->AXIS_REF.flow;
    motion->ACTPRESSURE = (IEC_REAL)fb->AXIS_REF.pressure;
    motion->TIMESTAMP = (IEC_REAL)fb->AXIS_REF.timestamp;
    /* Setpoint half (TARGET*, MAX*, etc.) is owned by PLC and must NOT be
     * overwritten by runtime. Multi-FB safety contract — see plc-process-
     * layer-integration-guide.md "HYD_AXISMOTION ownership". */
}
```

**注意**：如果原函数有其它必要的 act 字段写入（如 SEGMENTTAG 反馈），保留它们；目标只是删 Setpoint。完整核对原函数后再保存。

- [ ] **Step 6.7: 同时检查 `__mcl_cmd_MoveProfile` 的 simulation 分支（约 632-638 行）**

原代码：

```c
} else {
    motionData.ACTPOSITION = (IEC_REAL)fb->AXIS_REF.position;
    motionData.ACTVELOCITY = (IEC_REAL)fb->AXIS_REF.velocity;
    motionData.ACTFLOW = (IEC_REAL)fb->AXIS_REF.flow;
    motionData.ACTPRESSURE = (IEC_REAL)fb->AXIS_REF.pressure;
    motionData.TIMESTAMP = (IEC_REAL)fb->AXIS_REF.timestamp;
    __SET_VAR(data__->, MOTION,, motionData);
}
```

这段已经只写 Actual 字段，**不需要修改**。注意只读 setpoint 字段（line 625 `motionData = __GET_VAR(data__->MOTION);`）会读到 setpoint，但 `motionData` 是栈上 copy，覆写 Actual 后 SET_VAR 写回，这是把 setpoint copy-through 加 actual 更新——**这是正确行为**。

- [ ] **Step 6.8: 构建 + 跑测试**

```bash
cmake --build --preset unixgcc 2>&1 | tail -10
ctest --test-dir out/build/unixgcc -R '^test_axismotion_setpoint_isolation$' --output-on-failure
ctest --test-dir out/build/unixgcc --output-on-failure
```

Expected：新测试 PASS、全量无回归。如有回归，多半是其它消费 setpoint-back-fill 的代码——需要单独审计修复。

- [ ] **Step 6.9: 在 `include/motion_interface.h` 顶部追加契约注释**

在 `HYD_AXISMOTION` 定义前插入：

```c
/*
 * HYD_AXISMOTION ownership contract:
 *
 * The structure is a bidirectional shared buffer between the PLC process
 * layer and the HydroMotionLib runtime. Field ownership is partitioned:
 *
 * Setpoint half (PLC -> runtime, runtime MUST NOT write):
 *   TARGETPOSITION, TARGETVELOCITY, TARGETFLOW, TARGETPRESSURE,
 *   MAXVELOCITY, MAXACCELERATION, MAXDECELERATION, MAXFLOW,
 *   DURATION, all *TOLERANCE fields, SEGMENTTAG, SEGMENTTYPE,
 *   PLANNER, MODE, ENDCONDITION, DIRECTION, all tuning gains.
 *
 * Actual half (runtime -> PLC, PLC MUST NOT write):
 *   ACTPOSITION, ACTVELOCITY, ACTFLOW, ACTPRESSURE, TIMESTAMP.
 *
 * Multiple MoveProfile / MoveAbsolute / MoveVelocity / PressureHandle FB
 * instances may bind to the same physical HYD_AXISMOTION instance. The
 * runtime guarantees that no Setpoint field is silently overwritten by
 * its own pass; PLC programs are responsible for ensuring at most one
 * FB writes Setpoint values per scan.
 */
```

- [ ] **Step 6.10: 在 `docs/integration/plc-process-layer-integration-guide.md` 追加章节**

文件末尾追加：

```markdown
## HYD_AXISMOTION Half-Region Ownership

`HYD_AXISMOTION` is a bidirectional shared structure. To keep multi-FB
deployments safe, field ownership is partitioned into halves. **The
HydroMotionLib runtime treats this as a contract** and audits all
writes to ensure it never touches the Setpoint half.

### Setpoint half — owned by PLC

`TARGETPOSITION`, `TARGETVELOCITY`, `TARGETFLOW`, `TARGETPRESSURE`,
`MAXVELOCITY`, `MAXACCELERATION`, `MAXDECELERATION`, `MAXFLOW`, `DURATION`,
all `*TOLERANCE` fields, `SEGMENTTAG`, `SEGMENTTYPE`, `PLANNER`, `MODE`,
`ENDCONDITION`, `DIRECTION`, and all tuning-gain fields.

The runtime reads these fields on `EXECUTE` rising edge to build the
segment descriptor. After that, it does not look at them again until the
next rising edge.

### Actual half — owned by runtime

`ACTPOSITION`, `ACTVELOCITY`, `ACTFLOW`, `ACTPRESSURE`, `TIMESTAMP`.

In non-simulation deployments, the PLC writes these from sensor I/O each
scan before invoking any MoveProfile/MoveAbsolute/MoveVelocity/PressureHandle
FB. In simulation deployments, the runtime owns them.

### Multi-FB safety

If multiple FB instances bind to the same `HYD_AXISMOTION`, PLC programs
must ensure at most one FB writes Setpoint values per scan. The runtime
will not detect the violation, but its own pass is guaranteed not to
contribute to data races on Setpoint fields.
```

- [ ] **Step 6.11: 提交**

```bash
git add src/motion_interface.c include/motion_interface.h \
        tests/test_axismotion_setpoint_isolation.c CMakeLists.txt \
        docs/integration/plc-process-layer-integration-guide.md
git commit -m "$(cat <<'EOF'
fix: enforce HYD_AXISMOTION setpoint/actual half-region isolation

writeMotionFromSegment previously wrote both setpoint fields (TARGET*,
MAX*) and actual fields (ACT*) back into the shared HYD_AXISMOTION,
silently overwriting whatever the PLC had just queued for a subsequent
segment. Multi-FB deployments on the same axis could lose setpoint
values from later FBs.

- writeMotionFromSegment now writes only ACT* + TIMESTAMP
- Add contract comment block to HYD_AXISMOTION definition
- Add "HYD_AXISMOTION Half-Region Ownership" section to PLC integration guide
- New test_axismotion_setpoint_isolation.c uses sentinel values to lock
  the runtime out of setpoint writes

Refs: Sprint 0 spec C-1 (2026-05-21-code-review-and-roadmap-design.md)

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 7: Recipe execId 与 Direct execId 双 ID 解耦（C-2）

**目标：** 多段 recipe 推进时 `HYD_BeginSegment` 不再让 MoveProfile 的 ownership 失配。引入 `_recipeBatchId` 字段，仅在初始 START、外部 ABORT、Reset、direct 抢占等 **真正改变 recipe 批次身份** 的事件中递增；`recipeExecutionLostOwnership` 改用此字段。

**Files:**
- Modify: `include/motion_control.h`（`HYD_MotionControlFB` 增加 `_recipeBatchId` 字段）
- Modify: `src/motion_control.c`（`HYD_BeginSegment` 区分初始 start vs NextSegment；abort/reset/preempt 路径递增 batchId）
- Modify: `src/motion_interface.c`（`recipeExecutionCanAcquireOwnership` + `recipeExecutionLostOwnership` 改用 `_recipeBatchId`）
- Test: `tests/test_recipe_multi_segment_ownership.c`（新建）

### Steps

- [ ] **Step 7.1: 写失败测试 `tests/test_recipe_multi_segment_ownership.c`**

```c
/* tests/test_recipe_multi_segment_ownership.c
 * Verifies that NextSegment() does not falsely raise COMMANDABORTED on the
 * outer MoveProfile FB instance. */
#include "motion_interface.h"
#include "motion_control.h"
#include "action_profile.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

extern HYD_MotionControlFB *__MK_GetPublic_MotionControlFB(int axisIndex);
extern void __HydMotion_framework_Init(void);

static void test_moveprofile_survives_recipe_advance(void) {
    HYD_MOVEPROFILE move_inst;
    HYD_MotionSegment seg[3];
    HYD_MotionFBParams params;
    HYD_AXISMOTION motion;

    __HydMotion_framework_Init();
    memset(&move_inst, 0, sizeof(move_inst));
    memset(&params, 0, sizeof(params));
    params.maxFlow = 50.0;
    params.maxVelocity = 80.0;
    params.maxAcceleration = 500.0;
    params.maxDeceleration = 500.0;
    params.velocityToFlowGain = 0.25;
    params.positionTolerance = 0.5;

    /* 3-segment recipe: clamp close 30 -> clamp close 60 -> clamp close 100 */
    for (int i = 0; i < 3; i++) {
        HYD_ActionProfile_BuildClampClose(&seg[i], &params, (HYD_UINT8)(i + 1), (i + 1) * 30.0);
    }
    HYD_MotionControlFB *fb = __MK_GetPublic_MotionControlFB(0);
    assert(fb != NULL);
    fb->_useSimulation = true;
    fb->USE_RECIPE = true;
    fb->FLOW_TO_PUMP_SPEED_GAIN = 1.0;
    fb->PUMP_SPEED_LIMIT = 5000.0;
    HYD_MotionControlFB_LoadRecipe(fb, seg, 3);

    memset(&motion, 0, sizeof(motion));
    motion.TIMESTAMP = 0.0;
    __SET_VAR(move_inst., MOTION,, motion);
    __SET_VAR(move_inst., EXECUTE,, true);
    __SET_VAR(move_inst., AXISID,, 0);
    __SET_VAR(move_inst., BUFFERMODE,, HYD_BUFFER_MODE_ABORT);

    /* Rising edge */
    __mcl_cmd_MoveProfile(&move_inst);

    /* Run first segment to completion, then NextSegment */
    for (int step = 0; step < 1000; step++) {
        motion.TIMESTAMP = (IEC_REAL)(step * 0.01);
        __SET_VAR(move_inst., MOTION,, motion);
        __mcl_cmd_MoveProfile(&move_inst);

        if (fb->FB_STATE == HYD_FB_STATE_SEGMENT_COMPLETE) {
            assert(!__GET_VAR(move_inst.COMMANDABORTED));  /* THIS is the bug we're fixing */
            HYD_MotionControlFB_NextSegment(fb, fb->AXIS_REF.timestamp);
        }
        if (fb->FB_STATE == HYD_FB_STATE_DONE) {
            break;
        }
    }

    /* After full recipe: DONE=true, COMMANDABORTED=false on the SAME FB */
    assert(__GET_VAR(move_inst.DONE));
    assert(!__GET_VAR(move_inst.COMMANDABORTED));
    printf("test_moveprofile_survives_recipe_advance PASSED\n");
}

int main(void) {
    test_moveprofile_survives_recipe_advance();
    return 0;
}
```

注册到 CMakeLists.txt：

```cmake
add_executable(test_recipe_multi_segment_ownership tests/test_recipe_multi_segment_ownership.c)
target_link_libraries(test_recipe_multi_segment_ownership PRIVATE HydroMotionLib)
add_test(NAME test_recipe_multi_segment_ownership
         COMMAND test_recipe_multi_segment_ownership
         WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
```

- [ ] **Step 7.2: 配置 + 构建 + 跑测试，确认失败**

```bash
cmake --preset unixgcc
cmake --build --preset unixgcc --target test_recipe_multi_segment_ownership
ctest --test-dir out/build/unixgcc -R '^test_recipe_multi_segment_ownership$' --output-on-failure
```

Expected：FAIL — 在第一次 NextSegment 后 `move_inst.COMMANDABORTED` 被错误置 true。

- [ ] **Step 7.3: 在 `include/motion_control.h` 的 `HYD_MotionControlFB` 结构中增加 `_recipeBatchId` 字段**

找到 `uint16_t _executionId;` 那一行（约 262 行），在其下方追加：

```c
    uint16_t _recipeBatchId;       /* Increments per recipe batch (initial start), NOT per NextSegment. Used by IEC adapter to detect external recipe takeover separately from per-segment execution id. */
```

- [ ] **Step 7.4: 在 `src/motion_control.c` 中扩展 BeginSegment 的批次区分**

定位 `HYD_BeginSegment` 函数（grep 行号）。该函数当前会无条件 `fb->_executionId++`。修改如下：

```c
static HYD_BOOL HYD_BeginSegment(HYD_MotionControlFB* fb,
                                  const HYD_MotionSegment* sourceSegment,
                                  HYD_SegmentSource source,
                                  size_t segmentIndex,
                                  HYD_BOOL isInitialStart) {
    /* ... existing logic ... */

    fb->_executionId++;  /* per-segment id, always advances */

    if (isInitialStart || source == HYD_SEGMENT_SOURCE_DIRECT) {
        fb->_recipeBatchId++;  /* batch id only advances on initial start or direct takeover */
    }

    /* ... rest of function ... */
}
```

并在所有调用 `HYD_BeginSegment` 的地方按上下文传入 `isInitialStart`：

```bash
grep -n "HYD_BeginSegment(" src/motion_control.c
```

- **初始 Start 路径**（`HYD_HandleStartCommand` 内）→ `isInitialStart = true`
- **NextSegment 路径**（`HYD_AdvanceToNextSegment` 内）→ `isInitialStart = false`
- **Direct 接管路径**（`HYD_StartPendingDirectSlot`, `HYD_MotionControlFB_StartDirectCommand` 内）→ `isInitialStart = true`（direct 也是新 batch）

如果 `HYD_BeginSegment` 是 static helper，可以直接改签名。如果有第三方调用方，新增一个 `HYD_BeginSegmentEx` 并把旧符号实现为 `HYD_BeginSegmentEx(fb, ..., true)`。

- [ ] **Step 7.5: 在 ABORT / RESET / STOP 路径递增 `_recipeBatchId`**

`grep -n "_executionId++" src/motion_control.c` 列出所有递增点：

- `HYD_HandleAbort` / `HYD_CMD_ABORT` 消费分支 → `fb->_recipeBatchId++;` 紧随 `_executionId++`
- `HYD_CMD_STOP` 消费分支 → `fb->_recipeBatchId++;` 紧随
- SoftReset → 不需要（SoftReset 已 memset，batch id 会归零）
- Direct preempt → 已在 Step 7.4 BeginSegment 中处理

确保**只在 batch 真正失效的点递增**，不要在 NextSegment 走到 `BeginSegment` 时递增。

- [ ] **Step 7.6: 修改 `src/motion_interface.c` 的 `recipeExecutionLostOwnership` 与 `recipeExecutionCanAcquireOwnership`**

定位（约 289-325 行），把里面所有 `fb->_executionId` 引用改为 `fb->_recipeBatchId`：

```c
static HYD_BOOL recipeExecutionLostOwnership(const HYD_MotionControlFB* fb,
                                             IEC_WORD execId)
{
    if (fb == NULL || execId == 0) {
        return false;
    }

    if (fb->_activeSegmentSource == HYD_SEGMENT_SOURCE_RECIPE) {
        return false;
    }

    if (HYD_MotionControlFB_IsDone(fb) && fb->STATE.finished) {
        return false;
    }

    return execId != (IEC_WORD)fb->_recipeBatchId ||
           (!fb->STATE.active &&
            !HYD_MotionControlFB_IsBusy(fb) &&
            !fb->STATE.finished &&
            fb->_activeSegmentSource != HYD_SEGMENT_SOURCE_RECIPE);
}

static HYD_BOOL recipeExecutionCanAcquireOwnership(const HYD_MotionControlFB* fb)
{
    return fb != NULL &&
           fb->_recipeBatchId != 0U &&
           (HYD_MotionControlFB_GetDirectOwnerKind(fb) == HYD_DIRECT_CMD_NONE ||
            HYD_MotionControlFB_GetDirectOwnerExecutionId(fb) != fb->_recipeBatchId);
}

static HYD_BOOL recipeExecutionWasTakenOverBeforeLatch(const HYD_MotionControlFB* fb)
{
    return fb != NULL &&
           fb->_recipeBatchId != 0U &&
           HYD_MotionControlFB_GetDirectOwnerKind(fb) != HYD_DIRECT_CMD_NONE &&
           HYD_MotionControlFB_GetDirectOwnerExecutionId(fb) == fb->_recipeBatchId;
}
```

- [ ] **Step 7.7: 修改 `__mcl_cmd_MoveProfile` 中 _EXEC_ID 写入处（约 710 行）**

将 `(IEC_WORD)fb->_executionId` 改为 `(IEC_WORD)fb->_recipeBatchId`（两处：710 行的 _EXEC_ID 锁定，712 行的本地 myExecId 同步）：

```c
        if (recipeExecutionCanAcquireOwnership(fb)) {
            __SET_VAR(data__->, _EXEC_ID,, (IEC_WORD)fb->_recipeBatchId);
            __SET_VAR(data__->, _PENDING,, false);
            myExecId = (IEC_WORD)fb->_recipeBatchId;
        }
```

- [ ] **Step 7.8: 同步 motion_control.h 第 51 行附近的 contract 注释**

把现有 `execution_id` 描述补充一段：

```c
 * - execution_id:
 *   Runtime execution identity that ADVANCES on every successful BeginSegment.
 *   Used by direct-command ownership tracking. NOT used by IEC MoveProfile
 *   adapter — see _recipeBatchId.
 * - _recipeBatchId:
 *   Recipe-side batch identity. Advances only on initial Start, ABORT, STOP,
 *   and direct takeover. Does NOT advance on NextSegment. Used by IEC adapter
 *   to detect external recipe takeover without false-flagging
 *   multi-segment recipe progress as COMMANDABORTED.
```

- [ ] **Step 7.9: 构建 + 跑新测试 + 全量回归**

```bash
cmake --build --preset unixgcc 2>&1 | tail -20
ctest --test-dir out/build/unixgcc -R '^test_recipe_multi_segment_ownership$' --output-on-failure
ctest --test-dir out/build/unixgcc --output-on-failure
```

Expected：新测试 PASS。全量测试中可能有 1-2 个原本依赖"每段都递增 execId"行为的旧测试需要更新（如果是验证 abort/stop 抢占的，应继续 PASS；如果是验证"每段都新 execId"，需要改为验证 `_executionId` 而非 `_recipeBatchId`）。

- [ ] **Step 7.10: 追加抢占测试：MoveProfile 被 Stop 抢占时仍要触发 COMMANDABORTED**

在 `test_recipe_multi_segment_ownership.c` 中追加：

```c
static void test_moveprofile_aborted_by_stop_takeover(void) {
    HYD_MOVEPROFILE move_inst;
    HYD_STOP stop_inst;
    HYD_MotionSegment seg;
    HYD_MotionFBParams params;
    HYD_AXISMOTION motion;

    __HydMotion_framework_Init();
    memset(&move_inst, 0, sizeof(move_inst));
    memset(&stop_inst, 0, sizeof(stop_inst));
    memset(&params, 0, sizeof(params));
    params.maxFlow = 50.0;
    params.maxVelocity = 80.0;
    params.maxAcceleration = 500.0;
    params.maxDeceleration = 500.0;
    params.velocityToFlowGain = 0.25;
    params.positionTolerance = 0.5;
    HYD_ActionProfile_BuildClampClose(&seg, &params, 1, 100.0);

    HYD_MotionControlFB *fb = __MK_GetPublic_MotionControlFB(0);
    fb->_useSimulation = true;
    fb->USE_RECIPE = true;
    fb->FLOW_TO_PUMP_SPEED_GAIN = 1.0;
    fb->PUMP_SPEED_LIMIT = 5000.0;
    HYD_MotionControlFB_LoadRecipe(fb, &seg, 1);

    memset(&motion, 0, sizeof(motion));
    __SET_VAR(move_inst., MOTION,, motion);
    __SET_VAR(move_inst., EXECUTE,, true);
    __SET_VAR(move_inst., AXISID,, 0);
    __SET_VAR(move_inst., BUFFERMODE,, HYD_BUFFER_MODE_ABORT);
    __mcl_cmd_MoveProfile(&move_inst);

    /* Run a bit, then Stop */
    for (int i = 0; i < 50; i++) {
        motion.TIMESTAMP = (IEC_REAL)(i * 0.01);
        __SET_VAR(move_inst., MOTION,, motion);
        __mcl_cmd_MoveProfile(&move_inst);
    }
    __SET_VAR(stop_inst., AXISID,, 0);
    __SET_VAR(stop_inst., EXECUTE,, true);
    __SET_VAR(stop_inst., DECELERATION,, 200.0);
    __mcl_cmd_Stop(&stop_inst);

    /* Next MoveProfile scan: must see COMMANDABORTED */
    motion.TIMESTAMP = (IEC_REAL)(51 * 0.01);
    __SET_VAR(move_inst., MOTION,, motion);
    __mcl_cmd_MoveProfile(&move_inst);
    assert(__GET_VAR(move_inst.COMMANDABORTED));
    printf("test_moveprofile_aborted_by_stop_takeover PASSED\n");
}
```

并在 `main()` 中调用。重新构建跑：

```bash
cmake --build --preset unixgcc --target test_recipe_multi_segment_ownership
ctest --test-dir out/build/unixgcc -R '^test_recipe_multi_segment_ownership$' --output-on-failure
```

Expected：两个用例都 PASS。这验证了"NextSegment 不报 COMMANDABORTED" + "真抢占报 COMMANDABORTED"两条不变量。

- [ ] **Step 7.11: 提交**

```bash
git add include/motion_control.h src/motion_control.c src/motion_interface.c \
        tests/test_recipe_multi_segment_ownership.c CMakeLists.txt
git commit -m "$(cat <<'EOF'
fix: separate _recipeBatchId from _executionId to prevent false COMMANDABORTED

_executionId previously advanced on every HYD_BeginSegment, including
NextSegment within a recipe. The IEC adapter's recipeExecutionLostOwnership
check therefore saw exec-id mismatch every time a recipe advanced and
falsely raised COMMANDABORTED on the outer MoveProfile FB.

- Add _recipeBatchId field to HYD_MotionControlFB
- BeginSegment now takes isInitialStart parameter; only initial-start /
  direct-takeover paths advance _recipeBatchId; NextSegment does not
- ABORT / STOP / direct-preempt paths advance _recipeBatchId explicitly
- IEC adapter ownership predicates now compare against _recipeBatchId
- test_recipe_multi_segment_ownership.c covers both "no false abort on
  NextSegment" and "real Stop preemption still raises COMMANDABORTED"

Refs: Sprint 0 spec C-2 (2026-05-21-code-review-and-roadmap-design.md)

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## 收尾：Sprint 0 验收

- [ ] **Step 8.1: 跑全量测试**

```bash
ctest --test-dir out/build/unixgcc --output-on-failure
```

Expected：所有测试 PASS（已存在的 + 7 个新增测试文件总计 ~15-18 个新用例）。

- [ ] **Step 8.2: 嵌入式生产构建验证**

```bash
./scripts/deploy_embedded_prod.sh
```

Expected：成功生成 `out/install/embedded_prod/`，且核心库不含 sim 符号。

- [ ] **Step 8.3: 复盘 Sprint 0 commit 历史**

```bash
git log --oneline master..HEAD
```

Expected：7 个提交，分别对应 Task 1-7（顺序：M-1/M-2 → C-3 → C-7 → C-4 → I-7 → C-1 → C-2）。

- [ ] **Step 8.4: 更新 `docs/architecture/implementation-contract-gap-list.md`**

在文档头部 "Implemented Algorithm Gaps" 块追加：

```markdown
- Abort from FAULT state is now allowed and transitions to ABORTED (Sprint 0 C-3).
- ErrorMonitor duration accumulates only above per-channel tolerance (Sprint 0 C-7).
- STOP deceleration branch retains sensor / timestamp / timeout fault checks (Sprint 0 C-4).
- HYD_AXISMOTION setpoint half is no longer overwritten by runtime (Sprint 0 C-1).
- Recipe NextSegment no longer false-raises COMMANDABORTED on outer MoveProfile FB (Sprint 0 C-2).
```

- [ ] **Step 8.5: 收尾 commit**

```bash
git add docs/architecture/implementation-contract-gap-list.md
git commit -m "$(cat <<'EOF'
docs: log Sprint 0 critical fixes in implementation gap list

Refs: docs/superpowers/plans/2026-05-21-sprint0-critical-fixes.md

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## 自检清单（在转入 Sprint 1 之前）

- [ ] Spec 中 C-1 至 C-7 + I-7 + M-1 + M-2 各有对应 Task：✓
- [ ] 所有新测试使用 TDD（先写失败、再实现、再通过）：✓
- [ ] 每个修复都更新了对应的文档/契约：✓
- [ ] 跑全量 `ctest --output-on-failure` 全 PASS：必须验证
- [ ] 跑 `./scripts/deploy_embedded_prod.sh` 成功：必须验证
- [ ] 没有 placeholder ("TBD"/"实现合理处理"/"添加错误处理")：✓
- [ ] 所有命令、文件路径、行号在执行前需以仓库实际状态为准（行号会随提交漂移）

## 失败回退方案

如果某个 Task 中途出现无法解决的回归：

1. 用 `git stash` 暂存修改
2. 用 `git diff HEAD~ HEAD -- <相关文件>` 对照原代码理解差异
3. 把回归测试用例补到对应 `tests/test_<module>.c`
4. 重新设计修复策略，分更小的步骤
5. 不要绕过 `--no-verify` 或跳过测试 — 任何 PASS 都必须真实

## 已知风险与依赖

- **Task 6** 改动 `writeMotionFromSegment` 可能影响目前依赖"runtime 回写 setpoint"的隐性测试或工艺逻辑。一旦发现回归，需要逐一审计修复点而非回退。
- **Task 7** 改动 `_executionId` 语义，可能让现有 `test_motion_interface_arbitration.c` / `test_motion_interface_done_signals.c` 中以"每段都新 execId"为隐含假设的用例失败 — 失败时需更新断言到 `_recipeBatchId` 或保留 `_executionId` 但调整调用 - **不要简单删除测试**。
- 所有 IEC FB 公共 surface 不变（`__mcl_cmd_*` 函数签名、`HYD_*` FB 结构布局），故不会破坏 matiec/Beremiz 生成的 PLC 代码。
- 嵌入式平台 (`scripts/deploy_embedded_prod.sh`) 通过排除 sim 文件构建 HydroMotionLib；新增测试文件只编译为可执行测试，不进入生产库 — 风险中性。

