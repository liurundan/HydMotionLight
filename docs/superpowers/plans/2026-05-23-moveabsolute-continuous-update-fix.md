# MoveAbsolute ContinuousUpdate 完成后误报警修复

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 修复 MoveAbsolute 在 continuousUpdate=1 时，段完成后误报 `HYD_DIAG_CODE_COMMAND_NOT_ALLOWED` 的问题。

**Architecture:** 两个修改点均在 `src/motion_control.c`：(1) `HYD_RunRunningStateCompletion` 中 DIRECT 源段正常完成时记录完成状态（与已有的混合切换路径对称）；(2) `HYD_MotionControlFB_ApplyLiveUpdate` 中增加对"段已完成且同一 owner 请求更新"的容忍，避免误报错误。

**Tech Stack:** C99, CMake, ctest

---

## 文件结构

| 文件 | 职责 | 变更类型 |
|------|------|----------|
| `src/motion_control.c` | 核心修改：记录完成 + 容忍完成后状态 | 修改 |
| `tests/test_motion_interface_unit.c` | 新增单元测试：验证 continuousUpdate=1 完成流程 | 修改 |

---

### Task 1: 在 `HYD_RunRunningStateCompletion` 中记录 DIRECT 段完成

**文件:**
- 修改: `src/motion_control.c:1972-1986` (减速完成路径)
- 修改: `src/motion_control.c:2008-2019` (正常完成路径)

**背景:** 混合切换路径 `HYD_RunRunningStateBlendCutover` (L1953-1956) 在切出前调用了 `HYD_RecordDirectExecutionCompleted(fb)` 记录完成，但正常完成路径缺少这一步，导致 `_lastCompletedExecutionId` 未被设置，IEC 适配器无法检测完成状态。

- [ ] **Step 1: 修改减速完成路径 (L1972-1986)**

在 `completedSegmentSource == HYD_SEGMENT_SOURCE_DIRECT && fb->_directPendingValid` 为 false 时，调用 `ApplyIdleState` 之前记录完成。

定位到 `src/motion_control.c` 约 L1972-1986 的代码块：

```c
    if (fb->_isDecelerating &&
        fabs(plannerOutput->targetVelocity) < HYD_THRESH_DECEL_TARGET_VEL_DONE) {
        completedSegmentSource = fb->_activeSegmentSource;
        if (completedSegmentSource == HYD_SEGMENT_SOURCE_DIRECT &&
            fb->_directPendingValid) {
            (void)HYD_StartPendingDirectSlot(fb, fb->AXIS_REF.timestamp, false);
            return true;
        }
        recipeFinished = (completedSegmentSource == HYD_SEGMENT_SOURCE_DIRECT) ||
            (fb->STATE.currentSegmentIndex + 1U >= fb->RECIPE_SIZE);
        HYD_SafetyStateManager_ApplyIdleState(fb, recipeFinished, true);
        HYD_StateReporter_SetSegmentSource(fb, completedSegmentSource);
        HYD_StateReporter_RecordDiagnosticEvent(fb, fb->AXIS_REF.timestamp, segment, executionReference);
        return true;
    }
```

修改为：

```c
    if (fb->_isDecelerating &&
        fabs(plannerOutput->targetVelocity) < HYD_THRESH_DECEL_TARGET_VEL_DONE) {
        completedSegmentSource = fb->_activeSegmentSource;
        if (completedSegmentSource == HYD_SEGMENT_SOURCE_DIRECT &&
            fb->_directPendingValid) {
            (void)HYD_StartPendingDirectSlot(fb, fb->AXIS_REF.timestamp, false);
            return true;
        }
        if (completedSegmentSource == HYD_SEGMENT_SOURCE_DIRECT) {
            HYD_RecordDirectExecutionCompleted(fb);
        }
        recipeFinished = (completedSegmentSource == HYD_SEGMENT_SOURCE_DIRECT) ||
            (fb->STATE.currentSegmentIndex + 1U >= fb->RECIPE_SIZE);
        HYD_SafetyStateManager_ApplyIdleState(fb, recipeFinished, true);
        HYD_StateReporter_SetSegmentSource(fb, completedSegmentSource);
        HYD_StateReporter_RecordDiagnosticEvent(fb, fb->AXIS_REF.timestamp, segment, executionReference);
        return true;
    }
```

- [ ] **Step 2: 修改正常完成路径 (L2008-2019)**

同样的模式，在 `completedSegmentSource == HYD_SEGMENT_SOURCE_DIRECT && fb->_directPendingValid` 为 false 时，调用 `ApplyIdleState` 之前记录完成。

定位到 `src/motion_control.c` 约 L2008-2019 的代码块：

```c
    completedSegmentSource = fb->_activeSegmentSource;
    if (completedSegmentSource == HYD_SEGMENT_SOURCE_DIRECT &&
        fb->_directPendingValid) {
        (void)HYD_StartPendingDirectSlot(fb, fb->AXIS_REF.timestamp, false);
        return true;
    }
    recipeFinished = (completedSegmentSource == HYD_SEGMENT_SOURCE_DIRECT) ||
        (fb->STATE.currentSegmentIndex + 1U >= fb->RECIPE_SIZE);
    HYD_SafetyStateManager_ApplyIdleState(fb, recipeFinished, true);
    HYD_StateReporter_SetSegmentSource(fb, completedSegmentSource);
    HYD_StateReporter_RecordDiagnosticEvent(fb, fb->AXIS_REF.timestamp, segment, executionReference);
    return true;
```

修改为：

```c
    completedSegmentSource = fb->_activeSegmentSource;
    if (completedSegmentSource == HYD_SEGMENT_SOURCE_DIRECT &&
        fb->_directPendingValid) {
        (void)HYD_StartPendingDirectSlot(fb, fb->AXIS_REF.timestamp, false);
        return true;
    }
    if (completedSegmentSource == HYD_SEGMENT_SOURCE_DIRECT) {
        HYD_RecordDirectExecutionCompleted(fb);
    }
    recipeFinished = (completedSegmentSource == HYD_SEGMENT_SOURCE_DIRECT) ||
        (fb->STATE.currentSegmentIndex + 1U >= fb->RECIPE_SIZE);
    HYD_SafetyStateManager_ApplyIdleState(fb, recipeFinished, true);
    HYD_StateReporter_SetSegmentSource(fb, completedSegmentSource);
    HYD_StateReporter_RecordDiagnosticEvent(fb, fb->AXIS_REF.timestamp, segment, executionReference);
    return true;
```

**行为影响分析:**
- 仅在 `completedSegmentSource == HYD_SEGMENT_SOURCE_DIRECT` 且 `!_directPendingValid` 时触发
- RECIPE 源段不受影响（不调用 `RecordDirectExecutionCompleted`）
- 有待处理 DIRECT 段时（`_directPendingValid == true`）走 `StartPendingDirectSlot` 分支，不受影响
- `HYD_RecordDirectExecutionCompleted` 只拷贝 `_directOwnerExecutionId` 和 `_directOwnerKind` 到 `_lastCompleted*`，不修改任何其他字段

---

### Task 2: `HYD_MotionControlFB_ApplyLiveUpdate` 容忍已完成状态

**文件:**
- 修改: `src/motion_control.c:2772-2784`

**背景:** 当 DIRECT 段完成后，`_activeSegmentValid` 为 false、`STATE.active` 为 false、`_activeSegmentSource` 为 NONE。如果同一 owner 在 continuousUpdate 模式下继续发送实时更新请求，当前代码统一报 `COMMAND_NOT_ALLOWED`。应区分"段已完成"与"真正的命令冲突"两种情况。

- [ ] **Step 1: 添加已完成状态的例外处理**

定位到 `src/motion_control.c` 约 L2772-2784 的代码：

```c
    if (!fb->_activeSegmentValid ||
        !fb->STATE.active ||
        fb->_activeSegmentSource != HYD_SEGMENT_SOURCE_DIRECT ||
        fb->_directOwnerKind != request->ownerKind ||
        fb->_directOwnerExecutionId != request->ownerExecutionId) {
        HYD_StateReporter_ReportDiagnostic(fb,
                                           HYD_DIAG_CODE_COMMAND_NOT_ALLOWED,
                                           HYD_DIAG_SEVERITY_WARNING,
                                           fb->AXIS_REF.timestamp,
                                           fb->_activeSegmentValid ? &fb->_activeSegment : NULL,
                                           &fb->STATE.references);
        return false;
    }
```

修改为：

```c
    if (!fb->_activeSegmentValid ||
        !fb->STATE.active ||
        fb->_activeSegmentSource != HYD_SEGMENT_SOURCE_DIRECT ||
        fb->_directOwnerKind != request->ownerKind ||
        fb->_directOwnerExecutionId != request->ownerExecutionId) {
        if (fb->STATE.finished &&
            fb->_directOwnerKind == request->ownerKind &&
            fb->_directOwnerExecutionId == request->ownerExecutionId) {
            return true;
        }
        HYD_StateReporter_ReportDiagnostic(fb,
                                           HYD_DIAG_CODE_COMMAND_NOT_ALLOWED,
                                           HYD_DIAG_SEVERITY_WARNING,
                                           fb->AXIS_REF.timestamp,
                                           fb->_activeSegmentValid ? &fb->_activeSegment : NULL,
                                           &fb->STATE.references);
        return false;
    }
```

**条件精确性分析—仅当全部三个条件同时满足时静默返回 true：**

1. `fb->STATE.finished == true` — 段已正常完成（由 `ApplyIdleState(fb, true, ...)` 设置）
2. `fb->_directOwnerKind == request->ownerKind` — 同一命令类型（MoveAbsolute/MoveVelocity/PressureHandle）
3. `fb->_directOwnerExecutionId == request->ownerExecutionId` — 同一执行实例

**不会误容忍的场景：**
- **不同 owner 的并发命令**: ownerKind 或 ownerExecutionId 不匹配 → 仍然报 `COMMAND_NOT_ALLOWED`
- **段仍在执行中但 STATE.active 为 false** (如 HOLD 状态): `STATE.finished == false` → 仍然报 `COMMAND_NOT_ALLOWED`
- **RECIPE 模式段调用 live update**: `_activeSegmentSource != DIRECT` 且 `_directOwnerKind` 不匹配 → 仍然报 `COMMAND_NOT_ALLOWED`

---

### Task 3: 编译并运行全部测试

**文件:**
- 无新建/修改文件

- [ ] **Step 1: 编译**

```bash
cmake --build --preset unixgcc
```

预期: 编译成功，无警告。

- [ ] **Step 2: 运行全部单元测试**

```bash
ctest --test-dir out/build/unixgcc --output-on-failure
```

预期: 全部已有测试通过，无回归。

**可能受影响的测试文件:**
- `test_motion_interface_unit.c` — 包含 MoveAbsolute/MoveVelocity/PressureHandle 的 continuousUpdate 测试
- `test_motion_interface_done_signals.c` — 包含 MoveAbsolute Done 信号仿真测试
- `test_motion_interface_arbitration.c` — 包含多 FB 同轴竞争测试
- `test_moveabsolute_stop_integration.c` — 包含 MoveAbsolute + Stop 集成测试
- `test_stop_immediate_done.c` — 包含 Stop 立即完成测试

- [ ] **Step 3: 运行 main 手动端到端测试**

```bash
./out/build/unixgcc/main
```

预期: 仿真配方正常执行完成，输出 Done 信号。

---

### Task 4: 新增单元测试—验证 continuousUpdate=1 完成流程

**文件:**
- 修改: `tests/test_motion_interface_unit.c`

- [ ] **Step 1: 在文件末尾添加测试函数**

在 `tests/test_motion_interface_unit.c` 末尾（最后一个测试函数之后、`main` 函数之前）添加：

```c
/**
 * @brief MoveAbsolute continuousUpdate=1 在段完成后不应报 COMMAND_NOT_ALLOWED
 *
 * 复现步骤:
 *   1. MoveAbsolute EXECUTE 上升沿启动到 position=200
 *   2. 手动模拟段完成（调用 ApplyIdleState 设置 DONE）
 *   3. 在 continuousUpdate=1 下再次调用 MoveAbsolute
 *   4. 断言 ERROR==false（修复前会报 COMMAND_NOT_ALLOWED）
 */
static void test_moveabsolute_continuous_update_after_completion(void)
{
    HYD_MOVEABSOLUTE ma;
    HYD_MotionControlFB* fb;
    HYD_LiveUpdateRequest request;
    HYD_BOOL result;

    __HydMotion_framework_Init();
    ensure_axes_allocated(1);
    fb = __MK_GetPublic_MotionControlFB(0);
    memset(&ma, 0, sizeof(ma));

    /* --- 第一阶段: 启动 MoveAbsolute --- */
    IEC_VAL(ma.EN) = true;
    IEC_VAL(ma.EXECUTE) = true;
    ma.EXECUTE0.value = false;
    IEC_VAL(ma.AXISID) = 0;
    IEC_VAL(ma.POSITION) = 200.0f;
    IEC_VAL(ma.VELOCITY) = 50.0f;
    IEC_VAL(ma.ACCELERATION) = 100.0f;
    IEC_VAL(ma.DECELERATION) = 100.0f;
    IEC_VAL(ma.DIRECTION) = 1;
    IEC_VAL(ma.CONTINUOUSUPDATE) = true;

    __mcl_cmd_MoveAbsolute(&ma);
    ASSERT_TRUE(IEC_VAL(ma.ERROR) == false,
               "MoveAbsolute initial EXECUTE rising should not error");
    ASSERT_TRUE(IEC_VAL(ma.BUSY) == true,
               "MoveAbsolute should be BUSY after EXECUTE rising");

    /* 消耗 _PENDING → _EXEC_ID */
    __HydMotion_framework_Publish();
    ma.EXECUTE0.value = true;
    __mcl_cmd_MoveAbsolute(&ma);
    ASSERT_TRUE(IEC_VAL(ma.ERROR) == false,
               "MoveAbsolute second cycle (pending resolve) should not error");

    /* 确认段已激活 */
    ASSERT_TRUE(fb->_activeSegmentValid,
               "Active segment should be valid after MoveAbsolute start");
    ASSERT_EQ(fb->_activeSegmentSource, HYD_SEGMENT_SOURCE_DIRECT,
              "Segment source should be DIRECT");

    /* --- 第二阶段: 模拟段完成 --- */
    /* 验证 RecordDirectExecutionCompleted 记录完成状态 */
    ASSERT_TRUE(fb->_directOwnerExecutionId != 0,
               "Direct owner execution ID should be set");
    HYD_RecordDirectExecutionCompleted(fb);
    ASSERT_TRUE(fb->_lastCompletedExecutionId == fb->_directOwnerExecutionId,
               "Completion should be recorded with correct execution ID");

    /* --- 第三阶段: 完成后 continuousUpdate=1 下再次调用 --- */
    /* 先手动设置完成状态（模拟 HYD_RunRunningStateCompletion 的 ApplyIdleState） */
    fb->STATE.finished = true;
    fb->SEGMENT_COMPLETED = true;
    fb->_activeSegmentValid = false;
    fb->STATE.active = false;
    fb->_activeSegmentSource = HYD_SEGMENT_SOURCE_NONE;

    /* 模拟 continuousUpdate=1 的 live update 请求 */
    memset(&request, 0, sizeof(request));
    request.flags = HYD_LIVE_UPDATE_TARGET_POSITION | HYD_LIVE_UPDATE_MAX_VELOCITY;
    request.ownerKind = HYD_DIRECT_CMD_MOVE_ABSOLUTE;
    request.ownerExecutionId = fb->_directOwnerExecutionId;
    request.targetPosition = 200.0f;
    request.maxVelocity = 50.0f;

    result = HYD_MotionControlFB_ApplyLiveUpdate(fb, &request);
    ASSERT_TRUE(result == true,
               "ApplyLiveUpdate should succeed (not error) after segment completion");
    ASSERT_TRUE(fb->ERROR_ID != HYD_DIAG_CODE_COMMAND_NOT_ALLOWED,
               "Should NOT report COMMAND_NOT_ALLOWED after segment completion");

    /* --- 第四阶段: 验证不会误容忍错误的 owner --- */
    request.ownerExecutionId = fb->_directOwnerExecutionId + 999;
    result = HYD_MotionControlFB_ApplyLiveUpdate(fb, &request);
    ASSERT_TRUE(result == false,
               "ApplyLiveUpdate should reject request from WRONG owner even after completion");

    printf("  OK: test_moveabsolute_continuous_update_after_completion\n");
}
```

- [ ] **Step 2: 在 `main` 函数中注册测试**

找到 `tests/test_motion_interface_unit.c` 的 `main` 函数，在已有测试调用之后、`return` 之前添加：

```c
    test_moveabsolute_continuous_update_after_completion();
```

- [ ] **Step 3: 编译并运行新测试**

```bash
cmake --build --preset unixgcc
ctest --test-dir out/build/unixgcc -R '^test_motion_interface_unit$' --output-on-failure
```

预期: 新测试通过。

- [ ] **Step 4: 再次运行全部测试确认无回归**

```bash
ctest --test-dir out/build/unixgcc --output-on-failure
```

预期: 全部测试通过。

---

### Task 5: 提交

- [ ] **Step 1: 提交修改**

```bash
git add src/motion_control.c tests/test_motion_interface_unit.c
git commit -m "$(cat <<'EOF'
fix: MoveAbsolute continuousUpdate false COMMAND_NOT_ALLOWED after completion

Record direct execution completion in HYD_RunRunningStateCompletion
(symmetric with BlendCutover path) and tolerate post-completion live
update requests from the same owner in ApplyLiveUpdate.

Previously when continuousUpdate=1, after a direct segment completed,
the IEC adapter would try ApplyLiveUpdate on an inactive segment and
get COMMAND_NOT_ALLOWED. Now completion is properly recorded and
post-completion updates from the same owner silently succeed, allowing
the existing DONE check to fire.

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>
EOF
)"
```
