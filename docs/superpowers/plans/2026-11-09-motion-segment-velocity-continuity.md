# 运动段之间规划速度连续性实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 让位置段和速度段之间的规划速度从上一周期已发布速度连续过渡，并保留位置安全制动与反向先减速到零的约束。

**Architecture:** 在 `HYD_MotionPlannerState` 保存固定大小的过渡目标、过渡率和时间；仅当目标或时间改变时，才从 `lastTargetVelocity` 建立新斜坡。位置制动包络继续作为独立安全上限；运动段命令替换保留规划状态，压力模式边界及 Stop、EStop、Abort、Fault 行为保持不变。

**Tech Stack:** C99、HydroMotionLib、CMake/CTest、现有运动规划器和直接运动控制测试。

---

## 文件地图

- 修改 `include/motion_planner.h`：为 `HYD_MotionPlannerState` 增加必要的固定过渡状态字段；不添加动态分配。
- 修改 `src/motion_planner.c`：实现固定目标斜坡、位置安全包络限幅及反向时旧方向保持。
- 修改 `src/motion_control.c`：运动段到运动段替换时保留带符号规划速度及过渡状态；不从实际速度反馈构造起点。
- 修改 `tests/test_motion_planner_time_ramp.c`：覆盖 1 ms 目标过渡、目标下调、反馈速度隔离、反向和安全包络。
- 修改 `tests/test_mode_switch_bumpless.c`：增加同模式、位置/速度模式互切、反向直接替换回归；保留现有压力模式用例。
- 不修改 `iec/FB_HydAxis.st`、压力控制器、压力仿真蠕行、IEC 接口或 CMake；上述测试目标已注册。

## Task 1：增加规划器失败用例

**Files:**
- Modify: `tests/test_motion_planner_time_ramp.c`
- Modify: `tests/test_mode_switch_bumpless.c`

- [ ] **Step 1: 记录 planner state 的修改前大小。**

在 `tests/test_mode_switch_bumpless.c` 的 `main()` 开头输出：

```c
printf("planner_state=%zu bytes; motion_control_fb=%zu bytes\n",
       sizeof(HYD_MotionPlannerState), sizeof(HYD_MotionControlFB));
```

默认 float 构建还需加入硬上限：`assert(sizeof(HYD_MotionPlannerState) <= 28U);` 和 `assert(sizeof(HYD_MotionControlFB) <= 3304U);`。planner 上限来自修改前 16 bytes 加三个 `HYD_REAL`；FB 上限来自 3288 bytes 基线、12-byte planner 增量和 ABI 对齐填充。这样只允许 transition target、rate、time 三个固定字段，不允许后续无审查地继续扩张。

运行 `cmake --build --preset unixgcc --target test_mode_switch_bumpless && ctest --test-dir out/build/unixgcc -R '^test_mode_switch_bumpless$' --output-on-failure`，保存修改前的两个大小供 Task 4 比较。

- [ ] **Step 2: 收紧固定目标的 100 ms 过渡断言。**

保留现有速度段场景（`maxVelocity=20`、`maxAcceleration=100 ms`、`deltaTime=1 ms`、100 周期），将目标到达断言改为：

```c
check(fabs(output.targetVelocity - 20.0) < 0.01,
      "100 ms transition should reach the target within one-cycle tolerance");
```

- [ ] **Step 3: 添加目标下调并隔离反馈速度的测试。**

新增 `test_speed_target_reduction_rebases_from_planned_velocity()`：设置速度段目标 `40 mm/s`，运行 40 个 1 ms 周期；保存 `output.targetVelocity` 为 `previousPlanned`；将 `axis.velocity` 改为 `1.0`，将目标改为 `10 mm/s`，并令 elapsed time 继续递增。下一个输出应满足：

```c
expected = previousPlanned + (10.0 - previousPlanned) * 0.001 / 0.1;
check(fabs(output.targetVelocity - expected) < 0.01,
      "target reduction should rebase from the last planned velocity");
```

继续执行 99 个周期，断言速度到达 `10 mm/s`，误差小于 `0.01`。在 `main()` 注册此测试。

测试主体按下列顺序编写，复用当前文件的 `check()` helper：

```c
segment.maxVelocity = 40.0;
for (cycle = 0; cycle < 40; ++cycle) {
    input.elapsedTime = (cycle + 1) * 0.001;
    HYD_MotionPlanner_Execute(&input, &output);
}
previousPlanned = output.targetVelocity;
axis.velocity = 1.0;
segment.maxVelocity = 10.0;
input.elapsedTime = 0.041;
HYD_MotionPlanner_Execute(&input, &output);
expected = previousPlanned + (10.0 - previousPlanned) * 0.001 / 0.1;
check(fabs(output.targetVelocity - expected) < 0.01,
      "target reduction should rebase from the last planned velocity");
for (cycle = 1; cycle < 100; ++cycle) {
    input.elapsedTime = (41 + cycle) * 0.001;
    HYD_MotionPlanner_Execute(&input, &output);
}
check(fabs(output.targetVelocity - 10.0) < 0.01,
      "lower target should be reached within its 100 ms transition");
```

- [ ] **Step 4: 添加反向方向保持测试。**

新增 `test_reverse_keeps_old_direction_until_zero()`：构造位置段 `targetPosition=0`、`direction=HYD_DIRECTION_RETRACT`、轴位置 `100`、规划状态 `initialized=true` 且 `lastTargetVelocity=5`、加减速时间 `100 ms`。按 1 ms 周期调用规划器，断言减速过程 `targetVelocity` 保持正值且 `output.direction==HYD_DIRECTION_EXTEND`；到达零速的周期仍报告旧方向；下一周期才输出 `HYD_DIRECTION_RETRACT` 和负速度。在 `main()` 注册此测试。

以如下检查顺序实现该测试：

```c
for (cycle = 0; cycle < 99; ++cycle) {
    input.deltaTime = 0.001;
    HYD_MotionPlanner_Execute(&input, &output);
    check(output.direction == HYD_DIRECTION_EXTEND &&
              output.targetVelocity > 0.0,
          "reverse request should keep the old direction while slowing");
}
input.deltaTime = 0.001;
HYD_MotionPlanner_Execute(&input, &output);
check(output.direction == HYD_DIRECTION_EXTEND &&
          fabs(output.targetVelocity) < 0.01,
      "zero-speed cycle should still publish the old direction");
input.deltaTime = 0.001;
HYD_MotionPlanner_Execute(&input, &output);
check(output.direction == HYD_DIRECTION_RETRACT &&
          output.targetVelocity < 0.0,
      "new direction should start after the zero-speed cycle");
```

- [ ] **Step 5: 确认位置安全包络仍是硬上限。**

新增 `test_position_safety_envelope_remains_a_hard_ceiling()`：位置段最大速度 `20 mm/s`、减速时间 `100 ms`、目标位置 `100 mm`、实际位置 `99.95 mm`、规划状态速度 `20 mm/s`、反馈速度 `0`、周期 `1 ms`。安全包络期望值按现有 delay-aware 公式计算：

```c
brakingAcceleration = 20.0 / 0.1;
expectedSafety = sqrt(pow(brakingAcceleration * 0.001, 2.0) +
                      2.0 * brakingAcceleration * 0.05) -
                  brakingAcceleration * 0.001;
check(fabs(output.targetVelocity) <= expectedSafety + 0.001,
      "planned velocity must remain below the delay-aware position safety envelope");
```

在 `main()` 注册此测试。

- [ ] **Step 6: 运行新增用例并确认旧实现失败。**

运行：

```bash
cmake --build --preset unixgcc --target test_motion_planner_time_ramp test_mode_switch_bumpless
ctest --test-dir out/build/unixgcc -R '^(test_motion_planner_time_ramp|test_mode_switch_bumpless)$' --output-on-failure
```

预期：目标下调后 100 ms 内到达新目标的断言失败；反向减速期间保持旧 `output.direction` 的断言失败。向上固定目标和已有基础检查仍可运行。

- [ ] **Step 7: 提交红色回归用例。**

运行并提交：

```bash
git add tests/test_motion_planner_time_ramp.c tests/test_mode_switch_bumpless.c
git commit -m "Expose motion ramp discontinuities in tests" \
  -m "Constraint: Test fixed 1 ms cycles with the existing millisecond time contract." \
  -m "Rejected: Treating measured axis velocity as planner state | it hides command-side ramp resets." \
  -m "Confidence: high" \
  -m "Scope-risk: narrow" \
  -m "Directive: Keep pressure handover tests unchanged while adding motion-only assertions." \
  -m "Tested: New assertions fail against the current planner as expected; baseline mode-switch test passes." \
  -m "Not-tested: Implementation behavior."
```

## Task 2：实现有状态斜坡与规划器反向边界

**Files:**
- Modify: `include/motion_planner.h`
- Modify: `src/motion_planner.c`
- Test: `tests/test_motion_planner_time_ramp.c`

- [ ] **Step 1: 增加固定大小的过渡缓存。**

在 `HYD_MotionPlannerState` 末尾增加：

```c
HYD_REAL transitionTargetVelocity;
HYD_REAL transitionRate;
HYD_REAL transitionTimeMs;
```

`transitionRate` 保存带符号速度斜率，单位 `mm/s²`；`transitionTimeMs` 保存归一化毫秒数。用 `transitionTimeMs==0` 表示缓存无效，因为合法运动时间至少归一化为 100 ms；不增加额外 validity 字段。保留结构固定大小，不添加堆或轨迹缓冲。

- [ ] **Step 2: 实现一次建率、逐周期推进的 helper。**

在 `src/motion_planner.c` 添加静态 helper，主体采用以下逻辑：

```c
static HYD_REAL HYD_AdvanceVelocityTransition(HYD_MotionPlannerState* state,
                                              HYD_REAL targetVelocity,
                                              HYD_REAL transitionTimeMs,
                                              HYD_REAL deltaTime) {
    HYD_REAL currentVelocity;
    HYD_REAL nextVelocity;
    HYD_REAL transitionTime;

    if (state == NULL || deltaTime <= 0.0) {
        return (state != NULL) ? state->lastTargetVelocity : 0.0;
    }

    transitionTimeMs = HYD_NormalizeMotionTimeMs(transitionTimeMs);
    if (state->transitionTimeMs <= 0.0 ||
        fabs(state->transitionTargetVelocity - targetVelocity) > HYD_REAL_EPSILON ||
        fabs(state->transitionTimeMs - transitionTimeMs) > HYD_REAL_EPSILON) {
        transitionTime = HYD_MotionTimeMsToSeconds(transitionTimeMs);
        state->transitionTargetVelocity = targetVelocity;
        state->transitionTimeMs = transitionTimeMs;
        state->transitionRate = (targetVelocity - state->lastTargetVelocity) /
                                transitionTime;
    }

    currentVelocity = state->lastTargetVelocity;
    nextVelocity = currentVelocity + state->transitionRate * deltaTime;
    if ((state->transitionRate >= 0.0 && nextVelocity >= targetVelocity) ||
        (state->transitionRate < 0.0 && nextVelocity <= targetVelocity)) {
        nextVelocity = targetVelocity;
    }
    return nextVelocity;
}
```

该 helper 只负责普通目标斜坡；位置制动安全包络仍在其后执行，反向过程仍先建立零速目标。

- [ ] **Step 3: 让速度段和时间制位置段以段目标规划。**

替换 `HYD_ComputeSpeedRampVelocityMagnitude()` 与时间制位置段基于 `elapsedTime` 从零重建速度的部分。存在 planner state 时使用调用形式：

```c
HYD_REAL previousPlanned = input->state->initialized
    ? input->state->lastTargetVelocity : 0.0;
HYD_REAL requestedTarget = directionSign * segment->maxVelocity;
HYD_REAL transitionTimeMs =
    (fabs(requestedTarget) >= fabs(previousPlanned))
        ? segment->maxAcceleration
        : segment->maxDeceleration;
HYD_REAL plannedVelocity = HYD_AdvanceVelocityTransition(
    input->state, requestedTarget, transitionTimeMs, input->deltaTime);
```

`input->state==NULL` 时保留现有 `elapsedTime` 无状态计算分支，避免改变无状态调用者的接口行为。保留速度段 `decelElapsed/decelStartVel` 约束及位置段 delay-aware 距离制动安全上限；最终 flow 按经过安全限制后的规划速度重算。

- [ ] **Step 4: 让在线位置规划器区分巡航目标与终点目标。**

在 `HYD_ComputeOnlineTrapezoidVelocityMagnitude()` 未进入制动区时使用段最大速度作为斜坡目标；进入制动区时使用 blend terminal velocity（普通定位为零）。最终仍以已有 delay-aware braking velocity 作为硬上限，不使用该上限抬高斜坡输出。若安全上限低于斜坡输出，允许当周期安全覆盖。

- [ ] **Step 5: 反向减速期间保持旧规划方向。**

在 `HYD_MotionPlanner_Execute()` 发现请求方向与 `lastTargetVelocity` 符号相反时，将过渡目标设为零并按减速时间推进；同时把 `output->direction` 设为由旧规划速度符号解析出的方向。到零速的周期仍输出旧方向，下一周期旧速度已为零后才采用请求方向并从零启动新方向斜坡。

- [ ] **Step 6: 运行规划器用例。**

运行：

```bash
cmake --build --preset unixgcc --target test_motion_planner_time_ramp
ctest --test-dir out/build/unixgcc -R '^test_motion_planner_time_ramp$' --output-on-failure
```

预期：新目标在 100 个 1 ms 周期内到达；下调从 `lastTargetVelocity` 以固定斜率减速；反馈速度设为 `1 mm/s` 不改变该起点；反向在零速后切换方向。

- [ ] **Step 7: 提交规划器切片。**

运行并提交：

```bash
git add include/motion_planner.h src/motion_planner.c tests/test_motion_planner_time_ramp.c
git commit -m "Prevent target changes from resetting velocity ramps" \
  -m "Constraint: Keep the 1 ms cycle and millisecond transition contract." \
  -m "Rejected: Recomputing the transition rate every scan | it approaches the target asymptotically and misses the configured transition time." \
  -m "Confidence: medium" \
  -m "Scope-risk: moderate" \
  -m "Directive: Preserve the independent position braking envelope when changing planner ramps." \
  -m "Tested: test_motion_planner_time_ramp." \
  -m "Not-tested: Physical hydraulic response and machine vibration."
```

## Task 3：保留运动段替换中的完整规划状态

**Files:**
- Modify: `src/motion_control.c`
- Modify: `tests/test_mode_switch_bumpless.c`
- Test: `tests/test_motion_planner_time_ramp.c`

- [ ] **Step 1: 为运动段直接替换写回归用例。**

在 `tests/test_mode_switch_bumpless.c` 使用现有 `seed_active_segment()` 与 `make_position_segment()` / `make_speed_segment()`，增加位置→位置、速度→速度、位置→速度、速度→位置同方向替换，以及正向→反向位置段替换。每个测试把源 `_plannerState.lastTargetVelocity` 设为 `5.0`，调用 `HYD_MotionControlFB_StartDirectCommand()` 启动新段；同向场景断言状态未归零，运行一个 1 ms `HYD_MotionControlFB_Cycle()` 后速度从约 `5.0` 按新段目标过渡；反向场景断言速度归零前方向仍为 `HYD_DIRECTION_EXTEND`。保留原有压力 handover 和显式 Abort 清零测试。

同向位置→位置替换测试的核心调用和断言如下；位置→速度使用 `make_speed_segment()` 与 `HYD_DIRECT_CMD_MOVE_VELOCITY`；速度→速度使用 `make_speed_segment()` 与 `HYD_DIRECT_CMD_MOVE_VELOCITY`；速度→位置使用 `make_position_segment()` 与 `HYD_DIRECT_CMD_MOVE_ABSOLUTE`。所有后继速度目标设为 `10.0`，其余状态检查相同：

```c
HYD_MotionSegment successor = make_position_segment();
HYD_DirectStartResult result;
successor.targetPosition = 200.0;
successor.maxVelocity = 10.0;
seed_active_segment(&fb, make_position_segment(), HYD_DIRECT_CMD_MOVE_ABSOLUTE);
result = HYD_MotionControlFB_StartDirectCommand(
    &fb, HYD_DIRECT_CMD_MOVE_ABSOLUTE, &successor, NULL,
    HYD_BUFFER_MODE_ABORT, 1.001);
assert(result == HYD_DIRECT_START_STARTED);
assert(fabs(fb._plannerState.lastTargetVelocity - 5.0) < 0.001);
fb.AXIS_REF.timestamp = 1.002;
HYD_MotionControlFB_Cycle(&fb);
assert(fabs(fb.STATE.plannedVelocity - 5.05) < 0.02);
```

- [ ] **Step 2: 为运动段连续路径增加规划侧速度解析。**

调整 `HYD_ResolveContinuitySignedVelocity()` 与 `HYD_CaptureDirectContinuityState()`，增加 `allowFeedbackFallback` 参数。解析逻辑为：planner state initialized 时无条件使用 `lastTargetVelocity`（零值也有效）；未初始化时先读取 `STATE.plannedVelocity` 和 `_simFeedback.targetVelocity`；只有 pressure handover 旧路径才允许最后回退到 `AXIS_REF.velocity`。`StartPendingDirectSlot()` 和 MoveContinuousAbsolute approach/sustain 传 `false`；普通运动替换也传 `false`。`StartDirectCommand()` 仅当旧段或新段为压力模式时传 `true`，保持本轮范围外行为。

- [ ] **Step 3: 捕获和恢复所有运动到运动直接替换状态。**

在 `src/motion_control.c` 增加只匹配 `HYD_MODE_POSITION` 与 `HYD_MODE_SPEED_RAMP` 的模式判断，direct start 的 continuity 条件按以下逻辑扩展：

```c
static HYD_BOOL HYD_IsPlannerMotionMode(HYD_ControlMode mode) {
    return mode == HYD_MODE_POSITION || mode == HYD_MODE_SPEED_RAMP;
}

HYD_BOOL preserveMotionContinuity = activeDirect &&
    HYD_IsPlannerMotionMode(fb->_activeSegment.mode) &&
    HYD_IsPlannerMotionMode(segment->mode);
preserveContinuity = preserveContinuousAbsolute ||
                     preserveModeHandover ||
                     preserveMotionContinuity;
```

active direct 的旧段和新段均为运动模式时，以 `allowFeedbackFallback=false` 捕获并恢复完整 `_plannerState`，覆盖同模式、跨位置/速度模式和反向替换；压力模式不进入此新路径。`StartDirectCommand()` 捕获时用 `allowFeedbackFallback = (oldMode == HYD_MODE_PRESSURE_CLOSED_LOOP || newMode == HYD_MODE_PRESSURE_CLOSED_LOOP)`，pressure handover 沿用现有 fallback。显式 Stop、EStop、Abort、Fault 继续清零安全输出。

- [ ] **Step 4: Prime carryover 保留带符号速度和过渡状态。**

在 `HYD_PrimeSegmentControllers()` 清空 planner state 前暂存完整状态。对允许 flow carryover 的位置/速度运动段交接，在 `lastTargetVelocity != 0`、`transitionTimeMs > 0` 或已发布规划 flow 非零时保留完整状态；恢复带符号的 `lastTargetVelocity`、flow 和 transition metadata，不再仅写 `fabs(lastTargetVelocity)`。压力到运动的 flow reverse-map 和运动到压力的流量跟踪逻辑保持原样。当前 `HYD_MAX_SEGMENTS=1`，多条配方段在此构建配置不可运行；配方 successor 分支做源级审查，直接运动替换用运行时集成测试。

- [ ] **Step 5: 运行 handover 与压力边界回归。**

运行：

```bash
cmake --build --preset unixgcc --target test_motion_planner_time_ramp test_mode_switch_bumpless
ctest --test-dir out/build/unixgcc -R '^(test_motion_planner_time_ramp|test_mode_switch_bumpless)$' --output-on-failure
```

预期：新增同模式、跨模式和反向 handover 用例通过；原有四种压力 handover 和 explicit Abort 测试仍通过。

- [ ] **Step 6: 提交运动段 handover 切片。**

运行并提交：

```bash
git add src/motion_control.c tests/test_mode_switch_bumpless.c
git commit -m "Preserve planned velocity across motion handovers" \
  -m "Constraint: Pressure mode and explicit safety preemption keep their existing behavior." \
  -m "Rejected: Falling back to measured velocity | feedback lag and noise must not seed the command ramp." \
  -m "Confidence: medium" \
  -m "Scope-risk: moderate" \
  -m "Directive: Keep planner state signed and preserve the old direction through zero speed." \
  -m "Tested: test_mode_switch_bumpless and test_motion_planner_time_ramp." \
  -m "Not-tested: Physical hydraulic response and machine vibration."
```

## Task 4：完成回归和固定内存核对

**Files:**
- Verify: `include/motion_planner.h`
- Verify: `src/motion_planner.c`
- Verify: `src/motion_control.c`
- Verify: `tests/test_motion_planner_time_ramp.c`
- Verify: `tests/test_mode_switch_bumpless.c`

- [ ] **Step 1: 构建并运行运动规划与 handover 回归组。**

运行：

```bash
cmake --build --preset unixgcc --target test_motion_planner_time_ramp test_mode_switch_bumpless test_motion_interface_unit test_movevelocity_stop_reverse_restart test_moveabsolute_stop_integration test_movecontinuousabsolute_integration test_movecontinuousabsolute_external_sim_integration
ctest --test-dir out/build/unixgcc -R '^(test_motion_planner_time_ramp|test_mode_switch_bumpless|test_motion_interface_unit|test_movevelocity_stop_reverse_restart|test_moveabsolute_stop_integration|test_movecontinuousabsolute_integration|test_movecontinuousabsolute_external_sim_integration)$' --output-on-failure
```

预期：所有 CMake 注册的规划器与 handover 用例通过。`tests/test_motion_planner.c` 当前未注册到 CMake，且包含迁移前单位断言，不把它作为本计划测试门槛。

- [ ] **Step 2: 记录 planner 和 FB 静态大小。**

重新构建并运行 `test_mode_switch_bumpless`，确认 float 配置下 planner state 不超过 28 bytes、MotionControlFB 不超过 3304 bytes，并与 Task 1 记录的 16/3288 bytes 基线及输出比较。确认增长只来自三个固定 `HYD_REAL` 和 ABI 对齐，没有动态分配或轨迹缓存。

- [ ] **Step 3: 运行完整测试与空白检查。**

运行：

```bash
ctest --test-dir out/build/unixgcc --output-on-failure
git diff --check
```

预期：没有本次改动引入的新失败；已知基线失败按原类别记录，不扩大到压力模型或 IEC 接口修改。

- [ ] **Step 4: 检查最终工作区和验证记录。**

运行 `git status --short` 与 `git log -2 --oneline`；确认实现提交仅包含对应 planner、motion-control handover 和测试文件，验证结果及未覆盖的现场机械响应已记录。

## 规格覆盖检查

- 从 `lastTargetVelocity` 续接且不使用实际反馈起坡：Task 1–3。
- 目标/时间改变时从当前规划速度重建一次固定斜率：Task 1–2。
- 普通目标下调不再按 elapsed time 先加速：Task 1–2。
- 位置安全包络仍可优先钳低速度：Task 2、Task 4。
- 反向保持旧方向，归零后再切换：Task 1–2。
- 同模式和位置/速度模式直接替换保留状态：Task 3。
- Stop、EStop、Abort、Fault 与压力模式边界保持现有行为：Task 3–4。
- 固定内存及回归证据：Task 4。
- 机械振动和液压执行延迟仍需现场验证；仿真通过不等于现场验收。
