# 减速语义统一与短行程可达性 实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将运动规划器的加减速行为统一为“Δv/t 恒定过渡时间”语义（语义A），加入短行程在线可达性与刹距不足处理，并修复限压 scale 的 500ms 慢恢复，使开合模各段速度过渡时长恒等于配置的加减速时间。

**Architecture:** 规划层（锚定斜坡、制动触发、段入口过渡、短行程峰值）全部按 Δv/t；安全包络保持 maxVelocity/t_dec 不动，作为纯兜底层（正常轨迹永不介入）。短行程峰值在斜坡锚定时一次性求解，执行中只做可行性校验，不逐周期重算。刹距不足诊断沿用 motion_control 现有“每周期 WARNING”模式，不新增 FB 字段。

**Tech Stack:** 纯 C99，HYD_ 前缀，静态内存（无 malloc），CMake preset `unixgcc`（在 WSL Ubuntu-22.04 内构建），assert/check 风格单测。

**Spec:** `docs/superpowers/specs/2026-11-10-decel-semantics-and-reachability-design.md`（根因、语义决策、验收标准，执行前必读）

## Global Constraints

- 构建/测试全部在 WSL 内执行。本环境 shell 是 Windows Git Bash，统一用下面的包装（`MSYS2_ARG_CONV_EXCL` 防止路径被转换）：
  ```bash
  MSYS2_ARG_CONV_EXCL='*' wsl.exe -d Ubuntu-22.04 -- bash -lc 'cd /home/dan/project/hdy-motion-light && <命令>'
  ```
  配置：`cmake --preset unixgcc`；全量构建：`cmake --build --preset unixgcc`；单目标：`cmake --build out/build/unixgcc --target <name>`；单测：`ctest --test-dir out/build/unixgcc -R '^<name>$' --output-on-failure`；全量：`ctest --test-dir out/build/unixgcc --output-on-failure`
- git 命令在 Windows 侧执行：`cd "//wsl.localhost/Ubuntu-22.04/home/dan/project/hdy-motion-light" && git ...`
- 纯 C99；`HYD_` 前缀；无 malloc；只改任务列出的文件/区域，不“顺手”改相邻代码、注释、格式
- 工作区有**不属于本计划的未提交改动**（`.omx/metrics.json`、`scripts/coverage.sh`、`scripts/deploy_embedded_prod.sh`、`docs/data_deg/*.csv`）。提交时只 `git add <具体文件>`，**严禁 `git add -A` / `git add .` / `git commit -a`**
- 提交信息：简短中文主题行（仿照 `git log --oneline` 风格），结尾附你系统提示中规定的 Co-Authored-By 行
- 调参类浮点阈值只能放 `include/hyd_config.h` §14B（仓库约束7）
- **FB 尺寸预算**：`tests/test_mode_switch_bumpless.c` 断言 `sizeof(HYD_MotionControlFB) <= 3304U`，当前恰为 3304。**本计划不得向 `HYD_MotionControlFB` / `HYD_MotionPlannerState` 增加任何字段**
- **不变式（每个任务完成后都必须成立）**：
  - `HYD_ComputeDelayAwareBrakingVelocityMagnitude` 的 sqrt 距离界公式不改（包络兜底）；包络加速度始终是 `HYD_ResolveConfiguredBrakingAcceleration`（maxVelocity/t_dec）
  - `HYD_MIN_MOTION_TIME_MS = 100` 下限限幅不改
  - 泵速非负、方向仲裁逻辑不碰
- **已审计、明确不改**（这是设计决定，不是遗漏）：
  - `motion_control.c` `HYD_ResolveSegmentBrakingAccelerationForBlend`（只用于 >0 有效性校验）
  - `motion_control.c` `HYD_ResolveDirectBlendSwitchTolerance`（MOVE_ABSOLUTE 缓冲 blend 的切换窗口，不是速度过渡语义）
  - `motion_planner.c` 实测超速紧急路径（`if (!blendActive && measuredMagnitude > safetyVelocityMagnitude + ...)` 块，异常工况保留强减速）
  - `motion_planner.c` 无状态路径（`input->state == NULL` 的 else-if 分支，遗留路径，motion_control 总是传入 state）
  - `HYD_PlanTrapezoid` / `HYD_EvalTrapezoid`
- 既有测试失败的处理规则：只有当失败断言编码的是**旧 a_cfg=maxVelocity/t 的过渡时长/触发距离**时，才允许按语义A 更新期望值，并在提交信息中逐条说明；sqrt 包络距离界相关断言一律不许改；其他失败一律当作实现错误回头修

---

### Task 1: 段入口超速按 Δv/t_dec 过渡（修复现象 2a/2b/3）

**Files:**
- Modify: `src/motion_planner.c`（`HYD_ComputeOnlineTrapezoidVelocityMagnitude` 末尾的包络调用；`HYD_ComputePositionModeVelocityMagnitude` 删除 wrapper）
- Test: `tests/test_motion_planner_time_ramp.c`

**Interfaces:**
- Consumes: `HYD_MotionPlanner_Execute`、`HYD_MotionPlannerState`（`initialized/lastTargetVelocity/transitionTargetVelocity/transitionRate/transitionTimeMs`）
- Produces: 行为契约——新段 maxVelocity 低于入口速度 prev 时，输出以锚定速率 (maxVelocity−prev)/t_dec 线性过渡，恰好 t_dec 到位。Task 5/8 依赖。

**背景：** 现状下包络 `HYD_ComputeDelayAwareBrakingVelocityMagnitude` 末尾 `HYD_ClampReal(v, 0, maxVelocity)` 把输出一步钳到新段速度，再由 `HYD_ComputePositionModeVelocityMagnitude` 中的 wrapper 以 a_cfg=maxVelocity/t_dec 限斜率——导致 100→90% 用 11ms、30→10% 用 200ms。`HYD_AdvanceStatefulVelocity` 的锚定斜坡本身就是正确的 Δv/t 语义，只是被盖住了。修复：包络的**幅值上限**放宽为 `max(maxVelocity, prev)`（sqrt 距离界不变），并删除 wrapper。prev 随斜坡衰减回 maxVelocity，稳态上限不变。

**已知限制（本计划不处理，勿扩大范围）：** 非 blend（MoveAbsolute）时，“实测超速”紧急路径在反馈滞后（实测 > 规划）期间仍会触发并以 a_cfg 减速——该行为与现状完全相同，未变差。本次实验（MoveContinuousAbsolute）approach 阶段 blend 恒有效，不受影响。是否放宽紧急路径属于安全语义变更，需另行决策。

- [ ] **Step 1: 写失败测试**

在 `tests/test_motion_planner_time_ramp.c` 的 `main()` 之前追加（文件已有 `check(cond, msg)` 辅助与 `<math.h>/<stdio.h>/<string.h>`）：

```c
static void run_entry_overspeed_case(HYD_REAL maxVelocity,
                                     HYD_REAL entryVelocity,
                                     HYD_REAL expectedMidpoint)
{
    HYD_AxisRef axis = {0};
    HYD_MotionSegment segment = {0};
    HYD_MotionPlannerState state = {0};
    HYD_MotionPlannerInput input = {0};
    HYD_MotionPlannerOutput output = {0};
    int cycle;

    segment.mode = HYD_MODE_POSITION;
    segment.planner = HYD_PLANNER_POSITION_BASED;
    segment.endCondition = HYD_END_POSITION;
    segment.direction = HYD_DIRECTION_EXTEND;
    segment.targetPosition = 500.0;   /* 足够远，包络 sqrt 距离界不参与 */
    segment.maxVelocity = maxVelocity;
    segment.maxAcceleration = 100.0;  /* ms */
    segment.maxDeceleration = 100.0;  /* ms */
    segment.maxFlow = 1000.0;
    segment.velocityToFlowGain = 1.0;

    axis.position = 0.0;
    axis.velocity = entryVelocity;
    state.initialized = true;
    state.lastTargetVelocity = entryVelocity;  /* 上一段遗留速度 */

    input.axisRef = &axis;
    input.segment = &segment;
    input.state = &state;
    input.deltaTime = 0.001;

    for (cycle = 0; cycle < 102; ++cycle) {
        input.elapsedTime = (cycle + 1) * 0.001;
        HYD_MotionPlanner_Execute(&input, &output);
        axis.position += output.targetVelocity * 0.001; /* 运动学反馈 */
        axis.velocity = output.targetVelocity;
        if (cycle == 49) {
            check(fabs(output.targetVelocity - expectedMidpoint) < 0.5,
                  "entry overspeed must ramp linearly: midpoint at 50 ms");
        }
        check(output.targetVelocity >= maxVelocity - 0.01,
              "entry overspeed ramp must not undershoot the segment velocity");
    }
    /* HYD_REAL 为 float：允许斜坡在第 100~101 个周期落点（±1 周期） */
    check(fabs(output.targetVelocity - maxVelocity) < 0.01,
          "entry overspeed transition must complete in t_dec (+1 cycle)");
}

static void test_entry_overspeed_transitions_in_decel_time(void)
{
    /* 高速段入口（原 11ms 陡降）：100 -> 90，50ms 时应为 95 */
    run_entry_overspeed_case(90.0, 100.0, 95.0);
    /* 低速段入口（原 200ms 慢降）：30 -> 10，50ms 时应为 20 */
    run_entry_overspeed_case(10.0, 30.0, 20.0);
}
```

`main()` 中在 `test_small_target_and_time_changes_keep_cached_rate();` 之后加一行：
```c
    test_entry_overspeed_transitions_in_decel_time();
```

- [ ] **Step 2: 运行确认失败**

```bash
MSYS2_ARG_CONV_EXCL='*' wsl.exe -d Ubuntu-22.04 -- bash -lc 'cd /home/dan/project/hdy-motion-light && cmake --preset unixgcc >/dev/null && cmake --build out/build/unixgcc --target test_motion_planner_time_ramp && ctest --test-dir out/build/unixgcc -R "^test_motion_planner_time_ramp$" --output-on-failure'
```
预期：FAIL（高速用例 50ms 时已是 90 而非 95；低速用例 100ms 时约 20 而非 10）。

- [ ] **Step 3: 最小实现**

改动一：`HYD_ComputeOnlineTrapezoidVelocityMagnitude` 末尾的包络调用，只改第三个实参：

```c
    safetyVelocityMagnitude = HYD_ComputeDelayAwareBrakingVelocityMagnitude(
        remainingDistance,
        brakingAcceleration,
        HYD_MaxReal(segment->maxVelocity, previousMagnitude),
        blendActive ? 0.0 : input->deltaTime,
        terminalVelocity);
```

改动二：`HYD_ComputePositionModeVelocityMagnitude` 中删除 `if (input->deltaTime > 0.0 && previousMagnitude > input->segment->maxVelocity && ...)` 整块，POSITION_BASED 分支变为：

```c
    if (input->segment->planner == HYD_PLANNER_POSITION_BASED) {
        return HYD_ComputeOnlineTrapezoidVelocityMagnitude(input, direction);
    }
```
删除后该函数的局部变量 `previousMagnitude` 及其赋值块（`previousMagnitude = 0.0; if (input->state != NULL && input->state->initialized) { ... }`）变为无用，一并删除；`rampVelocityMagnitude`、`brakingAcceleration`、`brakeVelocityMagnitude`、`remainingDistance` 仍被下方 TIME_BASED 路径使用，保留。

- [ ] **Step 4: 运行确认通过**（同 Step 2 命令，预期 PASS）

- [ ] **Step 5: 全量测试**

```bash
MSYS2_ARG_CONV_EXCL='*' wsl.exe -d Ubuntu-22.04 -- bash -lc 'cd /home/dan/project/hdy-motion-light && cmake --build --preset unixgcc && ctest --test-dir out/build/unixgcc --output-on-failure'
```
预期全绿；失败按 Global Constraints 的处理规则处置。

- [ ] **Step 6: 提交**

```bash
cd "//wsl.localhost/Ubuntu-22.04/home/dan/project/hdy-motion-light" && git add src/motion_planner.c tests/test_motion_planner_time_ramp.c && git commit -m "段入口超速改为按 Δv/t_dec 恒时间过渡"
```

---

### Task 2: 制动触发距离改为恒时间语义（修复现象4）

**Files:**
- Modify: `src/motion_planner.c`（`HYD_ComputeOnlineTrapezoidVelocityMagnitude` 内 `brakeDistance` 计算）
- Test: `tests/test_motion_planner_time_ramp.c`

**Interfaces:**
- Consumes: Task 1 的行为
- Produces: `setup_blend_braking_case(...)` 测试夹具（Task 5 复用）；行为契约——段内制动在剩余距离 ≤ (prev+vt)/2·t_dec + (prev−vt)·dt 时触发，随后以锚定速率 (vt−prev)/t_dec 线性减速，约 t_dec 后在切换点达到 vt。

**背景：** 现状 brakeDistance = (v²−vt²)/(2·a_cfg)：高速段触发过晚（合模段4 仅 5.67mm），随后 sqrt 包络以 −a_cfg 接管（44ms 陡降）。语义A 下恒时间斜坡的精确制动距离是梯形均速×时间。

- [ ] **Step 1: 写失败测试**

```c
static void setup_blend_braking_case(HYD_AxisRef* axis,
                                     HYD_MotionSegment* segment,
                                     HYD_MotionBlendContext* blend,
                                     HYD_MotionPlannerState* state,
                                     HYD_MotionPlannerInput* input,
                                     HYD_REAL remainingDistance)
{
    memset(axis, 0, sizeof(*axis));
    memset(segment, 0, sizeof(*segment));
    memset(blend, 0, sizeof(*blend));
    memset(state, 0, sizeof(*state));
    memset(input, 0, sizeof(*input));

    segment->mode = HYD_MODE_POSITION;
    segment->planner = HYD_PLANNER_POSITION_BASED;
    segment->endCondition = HYD_END_POSITION;
    segment->direction = HYD_DIRECTION_EXTEND;
    segment->targetPosition = 100.0;
    segment->maxVelocity = 90.0;
    segment->maxAcceleration = 100.0;
    segment->maxDeceleration = 100.0;
    segment->maxFlow = 1000.0;
    segment->velocityToFlowGain = 1.0;

    blend->active = true;
    blend->bufferMode = HYD_BUFFER_MODE_BLENDING_NEXT;
    blend->blendVelocity = 50.0;            /* 终点速度 vt */
    blend->switchPosition = 100.0;
    blend->switchTolerance = 0.05;

    axis->position = 100.0 - remainingDistance;
    axis->velocity = 90.0;
    state->initialized = true;
    state->lastTargetVelocity = 90.0;       /* 巡航中 */

    input->axisRef = axis;
    input->segment = segment;
    input->state = state;
    input->blend = blend;
    input->deltaTime = 0.001;
    input->elapsedTime = 0.001;
}

static void test_braking_trigger_uses_fixed_time_distance(void)
{
    HYD_AxisRef axis;
    HYD_MotionSegment segment;
    HYD_MotionBlendContext blend;
    HYD_MotionPlannerState state;
    HYD_MotionPlannerInput input;
    HYD_MotionPlannerOutput output = {0};

    /* 新触发距离 = (90+50)/2*0.1 + 40*0.001 = 7.04mm；
     * 旧触发距离 = (90²-50²)/(2*900) + 0.04 = 3.15mm。
     * 6.5mm：新语义已制动（<90），旧语义仍巡航（=90）。 */
    setup_blend_braking_case(&axis, &segment, &blend, &state, &input, 6.5);
    HYD_MotionPlanner_Execute(&input, &output);
    check(output.targetVelocity < 90.0 - 0.2,   /* 新语义首周期 = 89.6 */
          "braking must engage at the fixed-time trigger distance");

    /* 8mm 在新触发距离之外：仍巡航，防止过早制动 */
    setup_blend_braking_case(&axis, &segment, &blend, &state, &input, 8.0);
    HYD_MotionPlanner_Execute(&input, &output);
    check(fabs(output.targetVelocity - 90.0) < 0.01,
          "cruise must continue outside the fixed-time trigger distance");
}

static void test_braking_lands_at_terminal_in_decel_time(void)
{
    HYD_AxisRef axis;
    HYD_MotionSegment segment;
    HYD_MotionBlendContext blend;
    HYD_MotionPlannerState state;
    HYD_MotionPlannerInput input;
    HYD_MotionPlannerOutput output = {0};
    int decelCycles = 0;
    int cycle;

    setup_blend_braking_case(&axis, &segment, &blend, &state, &input, 20.0);
    for (cycle = 0; cycle < 400; ++cycle) {
        input.elapsedTime = (cycle + 1) * 0.001;
        HYD_MotionPlanner_Execute(&input, &output);
        axis.position += output.targetVelocity * 0.001;
        axis.velocity = output.targetVelocity;
        if (output.targetVelocity < 90.0 - 0.05 &&
            output.targetVelocity > 50.0 + 0.05) {
            decelCycles++;
        }
        if (axis.position >= blend.switchPosition) {
            break;
        }
    }
    check(axis.position >= blend.switchPosition,
          "approach must reach the switch position");
    check(fabs(output.targetVelocity - 50.0) < 1.5,
          "velocity at the switch crossing must equal the end velocity");
    check(decelCycles >= 95 && decelCycles <= 105,
          "deceleration from 90 to 50 must take ~t_dec (100 cycles)");
}
```
`main()` 中紧跟 Task 1 的调用追加：
```c
    test_braking_trigger_uses_fixed_time_distance();
    test_braking_lands_at_terminal_in_decel_time();
```

- [ ] **Step 2: 运行确认失败**（命令同 Task 1 Step 2；预期 6.5mm 用例 FAIL）

- [ ] **Step 3: 最小实现**

把 `brakeDistance` 计算块（`if (previousMagnitude <= terminalVelocity) { brakeDistance = 0.0; } else { ... }`）替换为：

```c
            if (previousMagnitude <= terminalVelocity) {
                brakeDistance = 0.0;
            } else {
                HYD_TIME decelSeconds = HYD_MotionTimeMsToSeconds(
                    (segment->maxDeceleration > 0.0)
                        ? segment->maxDeceleration
                        : segment->maxAcceleration);
                /* 语义A：固定时间斜坡 prev->terminal 覆盖的梯形均速距离 */
                brakeDistance = 0.5 * (previousMagnitude + terminalVelocity) *
                                decelSeconds;
                if (input->deltaTime > 0.0 && isfinite(input->deltaTime)) {
                    brakeDistance += (previousMagnitude - terminalVelocity) *
                                     input->deltaTime;
                }
            }
```
（周期裕量项与原来一致；`brakingAcceleration` 仍供包络使用，不删。注意：vt=0 且 prev=maxVelocity 时新旧公式数值相同，停到零的行为不变。）

- [ ] **Step 4: 运行确认通过**

- [ ] **Step 5: 全量测试**（命令同 Task 1 Step 5）

- [ ] **Step 6: 提交**

```bash
cd "//wsl.localhost/Ubuntu-22.04/home/dan/project/hdy-motion-light" && git add src/motion_planner.c tests/test_motion_planner_time_ramp.c && git commit -m "制动触发距离改为恒时间语义"
```

---

### Task 3: SPEED_RAMP 减速改 Δv 语义

**Files:**
- Modify: `src/motion_planner.c`（`HYD_ComputeSpeedRampVelocityMagnitude` 的 decelElapsed 块；`HYD_MotionPlanner_Execute` 有状态速度分支中的 decelElapsed 块）
- Test: `tests/test_motion_planner_time_ramp.c`

**Interfaces:**
- Consumes: `input->decelStartVel`、`input->decelElapsed`
- Produces: SPEED_RAMP 减速曲线 v(t) = decelStartVel·(1 − t/t_dec)

**背景：** 两处 `decelVelocity = decelStartVel − HYD_ResolveConfiguredBrakingAcceleration(segment)·decelElapsed` 用 a_cfg：maxVelocity ≠ decelStartVel 时减速时长 ≠ t_dec。同函数内 END_POSITION 的**包络**调用保持 `HYD_ResolveConfiguredBrakingAcceleration` 不改。

- [ ] **Step 1: 写失败测试**

```c
static void test_speed_ramp_decel_uses_delta_v_rate(void)
{
    HYD_AxisRef axis = {0};
    HYD_MotionSegment segment = {0};
    HYD_MotionPlannerInput input = {0};
    HYD_MotionPlannerOutput output = {0};

    segment.mode = HYD_MODE_SPEED_RAMP;
    segment.endCondition = HYD_END_TIME;      /* 避开位置包络路径 */
    segment.direction = HYD_DIRECTION_EXTEND;
    segment.maxVelocity = 200.0;              /* 故意 != decelStartVel */
    segment.maxAcceleration = 100.0;
    segment.maxDeceleration = 100.0;
    segment.maxFlow = 1000.0;
    segment.velocityToFlowGain = 1.0;

    axis.velocity = 100.0;

    input.axisRef = &axis;
    input.segment = &segment;
    input.state = NULL;                        /* 走 HYD_ComputeSpeedRampVelocityMagnitude */
    input.deltaTime = 0.001;
    input.elapsedTime = 1.0;                   /* 加速早已完成 */
    input.decelStartVel = 100.0;
    input.decelElapsed = 0.05;                 /* 减速进行到 50ms */

    HYD_MotionPlanner_Execute(&input, &output);
    /* 语义A：a=100/0.1=1000 → v=100-1000*0.05=50；旧：a=200/0.1=2000 → v=0 */
    check(fabs(fabs(output.targetVelocity) - 50.0) < 0.5,
          "speed ramp deceleration must follow decelStartVel/t_dec");
}
```
`main()` 追加 `test_speed_ramp_decel_uses_delta_v_rate();`

- [ ] **Step 2: 运行确认失败**（预期输出 0 而非 50）

- [ ] **Step 3: 最小实现**

两处同构替换：在 `if (input->decelElapsed > 0.0 && input->decelStartVel > 0.0)` 块内，把用于计算 `decelVelocity` 的 `HYD_ResolveConfiguredBrakingAcceleration(input->segment)` 换成 `HYD_ResolveBrakingAcceleration(input->segment, input->decelStartVel, 0.0)`。
- `HYD_ComputeSpeedRampVelocityMagnitude` 中该块之前有一行 `brakingAcceleration = HYD_ResolveConfiguredBrakingAcceleration(input->segment);` 位于 if 外——把它移入 if 块并改为 Δv 形式；后面 END_POSITION 包络之前已有独立的 `brakingAcceleration = HYD_ResolveConfiguredBrakingAcceleration(...)` 赋值，保持不变。
- `HYD_MotionPlanner_Execute` 有状态分支里该块声明的局部 `HYD_REAL brakingAcceleration = HYD_ResolveConfiguredBrakingAcceleration(input->segment);` 改为 `HYD_ResolveBrakingAcceleration(input->segment, input->decelStartVel, 0.0)`；其后 END_POSITION 包络块里的 `HYD_ResolveConfiguredBrakingAcceleration` 不改。

- [ ] **Step 4: 运行确认通过**

- [ ] **Step 5: 全量测试**（`test_speed_ramp_preserves_elapsed_deceleration_cap` 中 decelStartVel=maxVelocity=20，新旧数值相同，应保持通过）

- [ ] **Step 6: 提交**

```bash
cd "//wsl.localhost/Ubuntu-22.04/home/dan/project/hdy-motion-light" && git add src/motion_planner.c tests/test_motion_planner_time_ramp.c && git commit -m "SPEED_RAMP 减速速率改为 Δv/t_dec"
```

---

### Task 4: ADAPTENDVEL 可达终速用实际斜坡加速度 + 新建 FB 级测试文件

**Files:**
- Create: `tests/test_decel_semantics_fb.c`
- Modify: `CMakeLists.txt`（在 `add_test(NAME test_mode_switch_bumpless ...)` 那一行之后注册新测试）
- Modify: `src/motion_control.c`（`HYD_ResolveContinuousAbsoluteCrossingVelocity` 中 `projectedVelocity < requested` 分支）

**Interfaces:**
- Consumes: `HYD_MotionControlFB_StartDirectCommand(fb, kind, segment, ctx, bufferMode, timestamp)` → `HYD_DirectStartResult`；`HYD_ContinuousAbsoluteContext`；`fb->_directContinuousAbsolute.crossingVelocity`
- Produces: 测试文件 `tests/test_decel_semantics_fb.c`，含 `check()`、`make_position_segment(target, maxVelocity)`、`seed_running_position(fb, position, velocity)` 夹具（Task 6 复用并追加用例）；可达终速公式 a = (maxVelocity − projected)/t_acc

**背景：** 规划器加速时锚定速率 = (巡航目标 − 当前速度)/t_acc，巡航目标是 maxVelocity；旧公式 a = maxVelocity/t_acc 只在 projected=0 时正确，axis 已在运动时高估可达终速。

- [ ] **Step 1: 创建测试文件（失败测试）**

`tests/test_decel_semantics_fb.c`：

```c
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "motion_control.h"

static int failures;

static void check(int condition, const char* message)
{
    if (!condition) {
        ++failures;
        printf("FAIL: %s\n", message);
    }
}

static HYD_MotionSegment make_position_segment(HYD_REAL targetPosition,
                                               HYD_REAL maxVelocity)
{
    HYD_MotionSegment segment;

    memset(&segment, 0, sizeof(segment));
    segment.segmentType = HYD_SEGMENT_TYPE_OTHER;
    segment.mode = HYD_MODE_POSITION;
    segment.endCondition = HYD_END_POSITION;
    segment.planner = HYD_PLANNER_POSITION_BASED;
    segment.direction = HYD_DIRECTION_EXTEND;
    segment.targetPosition = targetPosition;
    segment.maxVelocity = maxVelocity;
    segment.maxAcceleration = 100.0;
    segment.maxDeceleration = 100.0;
    segment.maxFlow = 500.0;
    segment.velocityToFlowGain = 1.0;
    segment.positionTolerance = 0.1;
    return segment;
}

/* 同 tests/test_mode_switch_bumpless.c 的 seed_active_segment 夹具：
 * 构造一个正在运行、目标=当前位置的 DIRECT 位置段（逻辑起点=position），
 * 规划器携带 velocity，供 StartDirectCommand 接管。 */
static void seed_running_position(HYD_MotionControlFB* fb,
                                  HYD_REAL position,
                                  HYD_REAL velocity)
{
    HYD_MotionControlFB_Init(fb);
    fb->USE_RECIPE = false;
    fb->FLOW_TO_PUMP_SPEED_GAIN = 1.0;
    fb->PUMP_SPEED_LIMIT = 3000.0;
    fb->AXIS_REF.position = position;
    fb->AXIS_REF.velocity = velocity;
    fb->AXIS_REF.flow = velocity;
    fb->AXIS_REF.pressure = 0.0;
    fb->AXIS_REF.timestamp = 1.0;
    fb->_activeSegment = make_position_segment(position, velocity);
    fb->_activeSegmentValid = true;
    fb->_activeSegmentSource = HYD_SEGMENT_SOURCE_DIRECT;
    fb->STATE.active = true;
    fb->FB_STATE = HYD_FB_STATE_RUNNING;
    fb->_directOwnerKind = HYD_DIRECT_CMD_MOVE_ABSOLUTE;
    fb->_directOwnerTicket = 1U;
    fb->_executionId = 1U;
    fb->PUMP_SPEED = velocity;
    fb->STATE.commandedPumpSpeed = velocity;
    fb->STATE.plannedFlow = velocity;
    fb->STATE.plannedVelocity = velocity;
    fb->_lastCommandedFlow = velocity;
    fb->_plannerState.initialized = true;
    fb->_plannerState.lastTargetVelocity = velocity;
    fb->_plannerState.lastTargetFlow = velocity;
}

static void test_adapt_end_velocity_uses_ramp_rate_from_projected_velocity(void)
{
    HYD_MotionControlFB fb;
    HYD_MotionSegment approach = make_position_segment(3.0, 100.0);
    HYD_ContinuousAbsoluteContext ctx;
    HYD_DirectStartResult result;

    seed_running_position(&fb, 0.0, 50.0);
    memset(&ctx, 0, sizeof(ctx));
    ctx.valid = true;
    ctx.phase = HYD_CONTABS_PHASE_APPROACH;
    ctx.targetPosition = approach.targetPosition;
    ctx.sustainVelocity = 80.0;
    ctx.adaptEndVelEnabled = true;
    ctx.approachDirection = HYD_DIRECTION_EXTEND;
    ctx.sustainDirection = HYD_DIRECTION_EXTEND;

    result = HYD_MotionControlFB_StartDirectCommand(
        &fb, HYD_DIRECT_CMD_MOVE_CONTINUOUS_ABSOLUTE, &approach, &ctx,
        HYD_BUFFER_MODE_ABORT, 1.001);
    check(result == HYD_DIRECT_START_STARTED,
          "continuous-absolute takeover must start");
    /* 语义A：a=(100-50)/0.1=500 → sqrt(50²+2*500*3)=74.16。
     * 旧：a=100/0.1=1000 → 92.2 → 夹到 requested=80。 */
    check(fabs(fb._directContinuousAbsolute.crossingVelocity - 74.16) < 1.0,
          "adapted crossing velocity must use (maxVelocity-projected)/t_acc");
}

int main(void)
{
    test_adapt_end_velocity_uses_ramp_rate_from_projected_velocity();
    printf("decel semantics fb: %s\n", failures == 0 ? "PASS" : "FAIL");
    return failures == 0 ? 0 : 1;
}
```

`CMakeLists.txt` 在 `add_test(NAME test_mode_switch_bumpless COMMAND test_mode_switch_bumpless)` 之后加：
```cmake
add_executable(test_decel_semantics_fb tests/test_decel_semantics_fb.c)
target_link_libraries(test_decel_semantics_fb PRIVATE HydroMotionLib)
add_test(NAME test_decel_semantics_fb COMMAND test_decel_semantics_fb)
```

- [ ] **Step 2: 运行确认失败**

```bash
MSYS2_ARG_CONV_EXCL='*' wsl.exe -d Ubuntu-22.04 -- bash -lc 'cd /home/dan/project/hdy-motion-light && cmake --preset unixgcc >/dev/null && cmake --build out/build/unixgcc --target test_decel_semantics_fb && ctest --test-dir out/build/unixgcc -R "^test_decel_semantics_fb$" --output-on-failure'
```
预期：编译通过，crossingVelocity 断言 FAIL（实际 80）。若 `result` 断言失败（StartDirectCommand 被拒），先读 `HYD_MotionControlFB_StartDirectCommand` 的拒绝条件修正**夹具**（不要改产品代码），直到只剩 crossingVelocity 断言失败。

- [ ] **Step 3: 最小实现**

`src/motion_control.c` `HYD_ResolveContinuousAbsoluteCrossingVelocity`：

```c
    if (projectedVelocity < requested) {
        /* 语义A：规划器从当前速度以 (巡航目标-当前)/t_acc 锚定加速 */
        HYD_REAL acceleration = (segment->maxVelocity - projectedVelocity) /
            HYD_MotionTimeMsToSeconds(segment->maxAcceleration);
        HYD_REAL reachable;

        if (acceleration < 0.0f) {
            acceleration = 0.0f;
        }
        reachable = sqrt(projectedVelocity * projectedVelocity +
                         2.0f * acceleration * remainingDistance);
        return HYD_ClampReal(reachable, 0.0f, requested);
    }
```

- [ ] **Step 4: 运行确认通过**

- [ ] **Step 5: 全量测试**（命令同 Task 1 Step 5）

- [ ] **Step 6: 提交**

```bash
cd "//wsl.localhost/Ubuntu-22.04/home/dan/project/hdy-motion-light" && git add CMakeLists.txt tests/test_decel_semantics_fb.c src/motion_control.c && git commit -m "终点速度自适应改用实际斜坡加速度"
```

---

### Task 5: 短行程可达峰值（D2 核心）

**Files:**
- Modify: `src/motion_planner.c`（`HYD_ComputeOnlineTrapezoidVelocityMagnitude` 的非制动巡航目标）
- Test: `tests/test_motion_planner_time_ramp.c`（复用 Task 2 的 `setup_blend_braking_case`）

**Interfaces:**
- Consumes: Task 2 的触发距离公式（两者必须用同一个 T_dec，衔接点才精确）
- Produces: 行为契约——加速前求一次可达峰值 vp，巡航目标 = min(maxVelocity, vp)；长行程（vp ≥ maxVelocity）行为与现状逐位一致。

**设计要点（给执行者，务必读懂再写代码）：**
- 语义A 下“加速 prev→vp、再在 Task 2 触发点减速 vp→vt”的总距离 = (prev+vp)/2·T_acc + (vp+vt)/2·T_dec + (vp−vt)·dt（最后一项是 Task 2 的周期裕量），对 vp 线性，**精确**闭式解：
  `vp = (2·s − T_acc·prev − T_dec·vt + 2·vt·dt) / (T_acc + T_dec + 2·dt)`
  不得用 maxVelocity 代替 vp 去估裕量——那会在 s < (maxVelocity−vt)·dt 时得到负 vp，轴停在目标前永不完成。
- **只在加速阶段（prev < maxVelocity 且未制动）施加 vp**；prev ≥ maxVelocity 或已在制动时不施加（交由 Task 1 入口过渡 / Task 2 制动 / 包络兜底）。
- **锚定保持（关键）**：`HYD_AdvanceStatefulVelocity` 在目标变化超出容差时会重新锚定。注意上面的 vp 公式假设“从 prev 新起一条耗时 T_acc 的斜坡”，而加速进行中实际是在沿已锚定的更陡斜坡前进，所以加速中途重算的 vp 会**单调下降**（可证 vp(t) = vp0 − v²/(2·vp0) − v/2），若直接使用会每周期重锚、速率越来越小。因此：
  - 已锚定目标 vA = `fabs(state->transitionTargetVelocity)`，速率 r = `fabs(state->transitionRate)`
  - 若 `vA > prev && vA < maxVelocity && freshVp < vA && r > 0`，用**精确运动学**校验 vA 是否仍可达：
    `s_need = (vA² − prev²)/(2r) + (vA+vt)/2·T_dec + (vA−vt)·dt`；`s + vA·dt ≥ s_need` 则沿用 vA（不重锚）。`vA·dt` 是离散积分松弛量：仿真/反馈按周期末速度积分位置，加速段每周期比连续模型多走 r·dt²/2，整段累计 ≤ vA·dt/2；没有这个松弛量，校验会每周期以 ~1e-4mm 的差额失败而重锚，峰值会被连续压低到约 0.44·vp（欠阻尼重锚动力学，已推导）
  - 否则用 freshVp（若低于 vA 即为“只降不升”的安全重锚）
  - freshVp ≥ vA（例如上一段遗留的陈旧锚定值）时直接用 min(maxVelocity, freshVp)，陈旧值不会压低新段
- 下限：巡航目标 ≥ max(prev, vt)。这同时覆盖两种情形：vp < prev → 保持当前速度，等待 Task 2 制动触发；vp < vt（例如 vt = 本段速度，只需加速、末端无减速段）→ 目标取 vt，vp 公式在该情形下无意义，不得生效。
- 不新增任何状态字段（FB 尺寸预算）。

- [ ] **Step 1: 写失败测试**

```c
static void test_short_stroke_caps_peak_and_keeps_ramp_time(void)
{
    HYD_AxisRef axis = {0};
    HYD_MotionSegment segment = {0};
    HYD_MotionPlannerState state = {0};
    HYD_MotionPlannerInput input = {0};
    HYD_MotionPlannerOutput output = {0};
    HYD_REAL peak = 0.0;
    HYD_REAL previous = 0.0;
    HYD_REAL maxStep = 0.0;
    int cycle;

    segment.mode = HYD_MODE_POSITION;
    segment.planner = HYD_PLANNER_POSITION_BASED;
    segment.endCondition = HYD_END_POSITION;
    segment.direction = HYD_DIRECTION_EXTEND;
    segment.targetPosition = 5.0;     /* 短行程 5mm */
    segment.maxVelocity = 90.0;       /* 高目标速度 */
    segment.maxAcceleration = 100.0;
    segment.maxDeceleration = 100.0;
    segment.maxFlow = 1000.0;
    segment.velocityToFlowGain = 1.0;
    state.initialized = true;         /* 从静止出发 */

    input.axisRef = &axis;
    input.segment = &segment;
    input.state = &state;
    input.deltaTime = 0.001;

    for (cycle = 0; cycle < 600; ++cycle) {
        input.elapsedTime = (cycle + 1) * 0.001;
        HYD_MotionPlanner_Execute(&input, &output);
        if (fabs(output.targetVelocity - previous) > maxStep) {
            maxStep = fabs(output.targetVelocity - previous);
        }
        previous = output.targetVelocity;
        axis.position += output.targetVelocity * 0.001;
        axis.velocity = output.targetVelocity;
        if (output.targetVelocity > peak) {
            peak = output.targetVelocity;
        }
    }
    /* vp = 2*5/(0.2+0.002) = 49.5：峰值必须被限制在 ~49.5。
     * 旧行为冲向 90 后被 sqrt 包络压回，峰值约 60~67。 */
    check(peak <= 50.5, "short stroke must cap the peak at the reachable vp");
    check(peak >= 40.0, "short stroke cap must not collapse the motion");
    /* 锚定一次：加速速率 = vp/T_acc ≈ 0.495/周期，绝不出现 90/0.1=0.9 的旧陡升 */
    check(maxStep <= 0.75, "short stroke ramp must use the anchored vp rate");
    check(axis.position > 4.5, "short stroke move must still converge on target");
}

static void test_long_stroke_keeps_full_cruise(void)
{
    HYD_AxisRef axis = {0};
    HYD_MotionSegment segment = {0};
    HYD_MotionPlannerState state = {0};
    HYD_MotionPlannerInput input = {0};
    HYD_MotionPlannerOutput output = {0};
    int cycle;

    segment.mode = HYD_MODE_POSITION;
    segment.planner = HYD_PLANNER_POSITION_BASED;
    segment.endCondition = HYD_END_POSITION;
    segment.direction = HYD_DIRECTION_EXTEND;
    segment.targetPosition = 500.0;
    segment.maxVelocity = 90.0;
    segment.maxAcceleration = 100.0;
    segment.maxDeceleration = 100.0;
    segment.maxFlow = 1000.0;
    segment.velocityToFlowGain = 1.0;
    state.initialized = true;

    input.axisRef = &axis;
    input.segment = &segment;
    input.state = &state;
    input.deltaTime = 0.001;

    for (cycle = 0; cycle < 102; ++cycle) {
        input.elapsedTime = (cycle + 1) * 0.001;
        HYD_MotionPlanner_Execute(&input, &output);
        axis.position += output.targetVelocity * 0.001;
        axis.velocity = output.targetVelocity;
        if (cycle == 49) {
            check(fabs(output.targetVelocity - 45.0) < 0.5,
                  "long stroke acceleration must stay linear (regression guard)");
        }
    }
    check(fabs(output.targetVelocity - 90.0) < 0.05,
          "long stroke must reach full cruise in t_acc (+1 cycle, regression guard)");
}
```
`main()` 追加：
```c
    test_short_stroke_caps_peak_and_keeps_ramp_time();
    test_long_stroke_keeps_full_cruise();
```

- [ ] **Step 2: 运行确认失败**（命令同 Task 1 Step 2；预期短行程用例 peak/maxStep 断言 FAIL，长行程用例 PASS）

- [ ] **Step 3: 最小实现**

在 `HYD_ComputeOnlineTrapezoidVelocityMagnitude` 中，`braking = ...` 赋值之后、`if (input->state != NULL) {` 之前插入；并把该 if 块里的 `segment->maxVelocity` 换成 `cruiseTargetMagnitude`：

```c
            /* D2：语义A 下的短行程可达峰值，只在加速阶段施加。
             * 已锚定峰值用精确运动学校验，仍可达则保持（不重锚，速率恒定）；
             * 不可达时以更低的新峰值重锚（只降不升，安全收敛）。 */
            {
                HYD_REAL cruiseTargetMagnitude = segment->maxVelocity;

                if (!braking && input->state != NULL &&
                    previousMagnitude < segment->maxVelocity) {
                    HYD_TIME accelSeconds =
                        HYD_MotionTimeMsToSeconds(segment->maxAcceleration);
                    HYD_TIME decelSeconds = HYD_MotionTimeMsToSeconds(
                        (segment->maxDeceleration > 0.0)
                            ? segment->maxDeceleration
                            : segment->maxAcceleration);
                    HYD_REAL cycle = (input->deltaTime > 0.0 &&
                                      isfinite(input->deltaTime))
                        ? input->deltaTime : 0.0;
                    HYD_REAL reachablePeak =
                        (2.0 * remainingDistance -
                         accelSeconds * previousMagnitude -
                         decelSeconds * terminalVelocity +
                         2.0 * terminalVelocity * cycle) /
                        (accelSeconds + decelSeconds + 2.0 * cycle);
                    HYD_REAL anchoredTarget =
                        fabs(input->state->transitionTargetVelocity);
                    HYD_REAL anchoredRate =
                        fabs(input->state->transitionRate);

                    if (anchoredTarget > previousMagnitude &&
                        anchoredTarget < segment->maxVelocity &&
                        reachablePeak < anchoredTarget &&
                        anchoredRate > 0.0) {
                        HYD_REAL neededDistance =
                            (anchoredTarget * anchoredTarget -
                             previousMagnitude * previousMagnitude) /
                                (2.0 * anchoredRate) +
                            0.5 * (anchoredTarget + terminalVelocity) *
                                decelSeconds +
                            (anchoredTarget - terminalVelocity) * cycle;
                        /* anchoredTarget*cycle：离散积分松弛量 */
                        if (remainingDistance + anchoredTarget * cycle >=
                            neededDistance) {
                            reachablePeak = anchoredTarget;
                        }
                    }
                    cruiseTargetMagnitude =
                        HYD_MinReal(cruiseTargetMagnitude, reachablePeak);
                    cruiseTargetMagnitude =
                        HYD_MaxReal(cruiseTargetMagnitude,
                                    HYD_MaxReal(previousMagnitude,
                                                terminalVelocity));
                    cruiseTargetMagnitude =
                        HYD_MinReal(cruiseTargetMagnitude, segment->maxVelocity);
                }
```

然后把原有的
```c
            if (input->state != NULL) {
                HYD_REAL targetMagnitude = braking ? terminalVelocity : segment->maxVelocity;
```
改为
```c
                if (input->state != NULL) {
                    HYD_REAL targetMagnitude = braking ? terminalVelocity : cruiseTargetMagnitude;
```
并让上面新开的 `{` 包住整个 `if (input->state != NULL) { ... } else if ... else { ... }` 链，在链末尾补一个 `}`。（非状态分支仍用 `segment->maxVelocity`，不改——见 Global Constraints“无状态路径不改”。）

**数值自检（写完代码后手算一遍，确认与测试期望一致）：**
- 短行程首周期：prev=0, s=5, vt=0, dt=0.001 → vp=(10−0)/(0.2+0.002)=49.5 → 锚定速率 495/s = 0.495/周期 ✓（≤0.75）
- 加速中途（例 v=24.75，已走≈0.62mm，s≈4.38）：freshVp≈43.2 < vA=49.5 → 校验 s_need=(49.5²−24.75²)/990+2.475+0.0495≈4.38，s+vA·dt≈4.43 ≥ 4.38 → 保持 vA，速率不变 ✓
- 到峰后（prev=vA）：校验条件 `anchoredTarget > previousMagnitude` 不成立 → freshVp<prev → 下限 clamp 为 prev，保持巡航，Task 2 触发减速 ✓；峰值 ≈49.5 ≤ 50.5 ✓
- vt = 本段速度（仅加速，例 prev=18, vmax=vt=90）：freshVp 在中短行程下 < vt → 下限 clamp 为 vt=90 → 与现状一致 ✓
- 长行程：vp ≫ 90 → cruiseTarget=90，与现状逐位一致 ✓

- [ ] **Step 4: 运行确认通过**

- [ ] **Step 5: 全量测试**（长行程回归与既有全部测试必须全绿）

- [ ] **Step 6: 提交**

```bash
cd "//wsl.localhost/Ubuntu-22.04/home/dan/project/hdy-motion-light" && git add src/motion_planner.c tests/test_motion_planner_time_ramp.c && git commit -m "短行程可达峰值限制"
```

---

### Task 6: 刹距不足 WARNING 诊断（D2 配套，不新增 FB 字段）

**Files:**
- Modify: `include/common_types.h`（`HYD_DiagnosticCode` 枚举**末尾**追加一项；`HYD_LimitFlags` 追加一项）
- Modify: `src/diagnostics.c`（`HYD_DIAGNOSTIC_SPECS[]` 追加一条；`HYD_Diagnostics_CodeToString` 追加一个 case）
- Modify: `src/motion_control.c`（`HYD_ExecuteActiveSegmentControl`：规划器执行后判定并置限制标志；`HYD_UpdateExecutionDiagnostics`：优先级链末尾加一档）
- Test: `tests/test_decel_semantics_fb.c`（复用 Task 4 的夹具）

**Interfaces:**
- Consumes: Task 4 的 `seed_running_position`、`make_position_segment`、`check`
- Produces: `HYD_DIAG_CODE_BRAKING_DISTANCE_INSUFFICIENT`（WARNING，`HYD_PROTECTION_ACTION_WARNING`，条件存续期间每周期置位）；`HYD_LIMIT_FLAG_BRAKING_DISTANCE = 1u << 5`

**设计要点：**
- 判定对象是**当前锚定的、指向终点速度的减速斜坡**：用规划器执行**前**的速度 prev、执行后锚定的斜坡速率 r = |`transitionRate`|，所需距离 `need = (prev² − vt²)/(2r)`；当 `need > remaining + prev·dt + positionTolerance` 时置位。
  - 为什么不能用规划器执行后的速度：包络一旦钳制，输出速度按构造就满足 v² ≈ vt² + 2·a_env·s，与剩余行程“自洽”，任何事后比较都检测不到（已推导：从 vmax 停到 0 的短行程接管场景，事后比较永不触发）。
  - 为什么不比较“新起一条斜坡”的恒时间刹距：正常减速进行中，剩余行程恒小于新起斜坡的刹距，全程误报。锚定斜坡的所需距离在正常减速中恒 ≤ 剩余行程（Task 2 的 (prev−vt)·dt 裕量 + 离散积分松弛），不会误报；`prev·dt` 再留一个周期的余量（默认 positionTolerance 只有 0.0001mm，不能指望它当裕量）。
  - 只在斜坡目标 == 终点速度时判定（即制动斜坡）。入口超速时斜坡目标是巡航速度，它在到位前被制动打断是正常的，不是短缺。
  - 只在剩余行程 > 0 时判定：换向时（按旧方向算出的剩余行程 ≤ 0）不得误报。
  - 两类短缺都能覆盖：A 类（所需减速度介于 Δv/T 与包络之间，例如本实验合模段2：10mm 行程内 180→162 需 17.1mm）持续置位；B 类（速度断崖，例如以 100mm/s 接管 2mm 停止段）至少在发生周期置位。
- 沿用现有诊断模式：`HYD_UpdateExecutionDiagnostics` 每周期按优先级选一个 priorityCode，无则 Clear。新码放在**优先级链最末**（低于 velocityDeviation），只用 `HYD_Diagnostics_SetEvent` 设置，不调用 `ReportDiagnostic`（不写历史快照，避免每周期刷屏）。这样不需要“只报一次”的锁存字段 → FB 尺寸不变。
- WARNING 绝不导致 IEC ERROR：`HYD_MotionControlFB_IsError()`（`include/motion_control.h:432`）只看 `faultActive` / `FB_STATE_FAULT` / `severity == FAULT`。`FB_HydAxis.st` 在 ERROR 时会进入报警步 900，所以严重度绝不能是 FAULT。
- `limitFlags` 在 `HYD_ExecuteActiveSegmentControl` 开头每周期清零（`fb->STATE.limitFlags = 0u;`），之后 `HYD_UpdateExecutionDiagnostics` 才运行，此处置位不会被覆盖。
- 判定位置：`HYD_ExecuteActiveSegmentControl` 中紧跟 `HYD_MotionPlanner_Execute(&plannerInput, plannerOutput);` 之后。那里能拿到执行前的规划速度、规划器本周期实际使用的 `selectedBlend`（终点速度与切换点，与规划器完全一致），以及规划输出的方向。`HYD_UpdateExecutionDiagnostics` 只读取限制标志，放进优先级链。

- [ ] **Step 1: 枚举、规格表与字符串**

`include/common_types.h`：
- `HYD_DiagnosticCode` 中把最后一项 `HYD_DIAG_CODE_KINEMATICS_RUNTIME_INVALID` 改为带逗号，并在其后追加 `HYD_DIAG_CODE_BRAKING_DISTANCE_INSUFFICIENT  /* WARNING: 剩余行程不足以按包络减速到终点速度 */`。**只能加在末尾**：数值是对 PLC 的 ERRORID 契约。
- `HYD_LimitFlags` 中 `HYD_LIMIT_FLAG_DERATE = 1u << 4` 改为带逗号，其后追加 `HYD_LIMIT_FLAG_BRAKING_DISTANCE = 1u << 5`（`limitFlags` 是 `HYD_UINT8`，bit5 可用）。

`src/diagnostics.c`：
- `HYD_DIAGNOSTIC_SPECS[]` 中 `HYD_DIAG_CODE_KINEMATICS_RUNTIME_INVALID` 条目之后追加（字段顺序照抄相邻条目：code, severity, source, recovery, protectionAction, message）：
  ```c
      {HYD_DIAG_CODE_BRAKING_DISTANCE_INSUFFICIENT,
       HYD_DIAG_SEVERITY_WARNING,
       HYD_DIAG_SOURCE_EXECUTION,
       HYD_DIAG_RECOVERY_CHECK_COMMAND,
       HYD_PROTECTION_ACTION_WARNING,
       "Remaining stroke too short to reach end velocity within braking envelope"},
  ```
- `HYD_Diagnostics_CodeToString` 的 switch 中 `case HYD_DIAG_CODE_KINEMATICS_RUNTIME_INVALID:` 之后追加：
  ```c
          case HYD_DIAG_CODE_BRAKING_DISTANCE_INSUFFICIENT:
              return "BRAKING_DISTANCE_INSUFFICIENT";
  ```

- [ ] **Step 2: 写失败测试**

在 `tests/test_decel_semantics_fb.c` 的 `main()` 之前追加，并在 `main()` 中调用：

```c
static void test_braking_shortfall_raises_warning_not_fault(void)
{
    HYD_MotionControlFB fb;
    HYD_MotionSegment shortSegment = make_position_segment(2.0, 100.0);
    HYD_DirectStartResult result;
    int sawFlag = 0;
    int sawCode = 0;
    int sawFault = 0;
    int cycle;

    /* 以 100mm/s 接管一个仅 2mm 的停止段（终速 0）：
     * 锚定减速斜坡所需距离 = 100²/(2*1000) = 5mm > 2 + 0.1 + 0.1 → 短缺成立 */
    seed_running_position(&fb, 0.0, 100.0);
    result = HYD_MotionControlFB_StartDirectCommand(
        &fb, HYD_DIRECT_CMD_MOVE_ABSOLUTE, &shortSegment, NULL,
        HYD_BUFFER_MODE_ABORT, 1.001);
    check(result == HYD_DIRECT_START_STARTED,
          "short stop segment takeover must start");

    for (cycle = 0; cycle < 3; ++cycle) {
        fb.AXIS_REF.timestamp = 1.002 + cycle * 0.001;
        fb.AXIS_REF.velocity = 100.0;       /* 反馈仍为高速 */
        fb.AXIS_REF.position = 0.1 * cycle;
        HYD_MotionControlFB_Cycle(&fb);
        if (fb.STATE.limitFlags & HYD_LIMIT_FLAG_BRAKING_DISTANCE) {
            sawFlag = 1;
        }
        if (fb.DIAGNOSTIC.code == HYD_DIAG_CODE_BRAKING_DISTANCE_INSUFFICIENT &&
            fb.DIAGNOSTIC.severity == HYD_DIAG_SEVERITY_WARNING) {
            sawCode = 1;
        }
        if (HYD_MotionControlFB_IsError(&fb)) {
            sawFault = 1;
        }
    }
    check(sawFlag, "braking shortfall must raise HYD_LIMIT_FLAG_BRAKING_DISTANCE");
    check(sawCode, "braking shortfall must surface a WARNING diagnostic");
    check(!sawFault, "braking shortfall must never escalate to an IEC error");
}

static void test_normal_braking_does_not_warn(void)
{
    HYD_MotionControlFB fb;
    HYD_MotionSegment longSegment = make_position_segment(50.0, 100.0);
    HYD_DirectStartResult result;
    int sawFlag = 0;
    int cycle;

    /* 50mm 足够：包络刹距 5mm 远小于剩余行程 → 全程不得报警 */
    seed_running_position(&fb, 0.0, 100.0);
    result = HYD_MotionControlFB_StartDirectCommand(
        &fb, HYD_DIRECT_CMD_MOVE_ABSOLUTE, &longSegment, NULL,
        HYD_BUFFER_MODE_ABORT, 1.001);
    check(result == HYD_DIRECT_START_STARTED,
          "long stop segment takeover must start");

    for (cycle = 0; cycle < 3; ++cycle) {
        fb.AXIS_REF.timestamp = 1.002 + cycle * 0.001;
        fb.AXIS_REF.velocity = 100.0;
        fb.AXIS_REF.position = 0.1 * cycle;
        HYD_MotionControlFB_Cycle(&fb);
        if (fb.STATE.limitFlags & HYD_LIMIT_FLAG_BRAKING_DISTANCE) {
            sawFlag = 1;
        }
    }
    check(!sawFlag, "sufficient stroke must not raise the braking shortfall flag");
}
```

- [ ] **Step 3: 运行确认失败**

```bash
MSYS2_ARG_CONV_EXCL='*' wsl.exe -d Ubuntu-22.04 -- bash -lc 'cd /home/dan/project/hdy-motion-light && cmake --build out/build/unixgcc --target test_decel_semantics_fb && ctest --test-dir out/build/unixgcc -R "^test_decel_semantics_fb$" --output-on-failure'
```
预期：编译通过（Step 1 已加枚举）；`sawFlag`/`sawCode` 断言 FAIL。若 `result` 断言失败，只修夹具，不改产品代码（参考 Task 4 Step 2 的说明）。

- [ ] **Step 4: 最小实现**

(a) `src/motion_control.c` `HYD_ExecuteActiveSegmentControl` 非压力分支：在 `HYD_MotionPlanner_Execute(&plannerInput, plannerOutput);` 这一行**之前**加一行，记录执行前的规划速度：

```c
        HYD_REAL plannedMagnitudeBefore = fabs(fb->_plannerState.lastTargetVelocity);
```
（若编译器因声明出现在语句之后而报错，把声明放到该 `else {` 块开头，赋值仍放在原位置。）

在该调用这一行**之后**紧接着插入：

```c
        /* D2：刹距可行性诊断。判定对象是规划器本周期锚定的、指向终点速度的
         * 减速斜坡；所需距离超过剩余行程即说明恒时间减速已不可行，由包络兜底
         * 并上报 WARNING。不改变规划输出；反应归流程层。 */
        if (segment->mode == HYD_MODE_POSITION &&
            segment->endCondition == HYD_END_POSITION &&
            (plannerOutput->direction == HYD_DIRECTION_EXTEND ||
             plannerOutput->direction == HYD_DIRECTION_RETRACT)) {
            HYD_REAL terminalVelocity = 0.0f;
            HYD_REAL switchPosition = segment->targetPosition;
            HYD_REAL rampRate = fabs(fb->_plannerState.transitionRate);
            HYD_REAL rampTarget = fabs(fb->_plannerState.transitionTargetVelocity);
            HYD_REAL remaining;

            if (selectedBlend != NULL && selectedBlend->active) {
                terminalVelocity = HYD_ClampReal(selectedBlend->blendVelocity,
                                                 0.0f, segment->maxVelocity);
                switchPosition = selectedBlend->switchPosition;
            }
            remaining = (plannerOutput->direction == HYD_DIRECTION_EXTEND)
                ? (switchPosition - fb->AXIS_REF.position)
                : (fb->AXIS_REF.position - switchPosition);
            if (remaining > 0.0f &&
                plannedMagnitudeBefore > terminalVelocity &&
                rampRate > 0.0f &&
                fabs(rampTarget - terminalVelocity) <=
                    0.01f + 0.001f * terminalVelocity &&
                (plannedMagnitudeBefore * plannedMagnitudeBefore -
                 terminalVelocity * terminalVelocity) / (2.0f * rampRate) >
                    remaining +
                    plannedMagnitudeBefore * plannerInput.deltaTime +
                    HYD_Segment_GetPositionTolerance(segment)) {
                fb->STATE.limitFlags |= HYD_LIMIT_FLAG_BRAKING_DISTANCE;
            }
        }
```
说明：`fb->STATE.limitFlags` 在本函数开头已清零（`fb->STATE.limitFlags = 0u;`），所以标志按周期反映当前状态。`plannerInput`、`selectedBlend` 都是本函数已有的局部变量；`HYD_Segment_GetPositionTolerance` 已被本文件引用。

(b) `HYD_UpdateExecutionDiagnostics`：在 `if (priorityCode != HYD_DIAG_CODE_NONE) {` 这一行**之前**插入（最低优先级，不覆盖任何更高优先级的码）：

```c
    if (priorityCode == HYD_DIAG_CODE_NONE &&
        (fb->STATE.limitFlags & HYD_LIMIT_FLAG_BRAKING_DISTANCE) != 0u) {
        priorityCode = HYD_DIAG_CODE_BRAKING_DISTANCE_INSUFFICIENT;
        prioritySeverity = HYD_DIAG_SEVERITY_WARNING;
    }
```
（调用顺序已核对：`HYD_ExecuteActiveSegmentControl`（运行状态中第 ~2989 行）先于 `HYD_UpdateExecutionDiagnostics`（~3002 行）。该函数开头只有空指针检查会提前 return，位置段一定执行到这里。）

**数值自检：**
- 测试 1（2mm 停止段，接管速度 100）：首周期 prev=100、vt=0，锚定斜坡 r=100/0.1=1000，need=5mm > 2+0.1+0.1=2.2 → 置位 ✓
- 测试 2（50mm）：首周期仍在巡航，斜坡目标=100≠0 → 不判定；进入制动后 need ≤ 剩余行程 → 不置位 ✓
- 本实验合模段2（10mm、180→162、vt=162）：入口斜坡目标=162=vt，r=180，need=(180²−162²)/360≈17.1mm > 10+0.18 → 置位，如实反映“行程不够 100ms 过渡”（Task 1 之后靠包络兜底收敛）✓

- [ ] **Step 5: 运行确认通过 + 全量**

单测命令同 Step 3；全量命令同 Task 1 Step 5。确认 `test_mode_switch_bumpless` 打印的 `motion_control_fb=3304 bytes` 没变（本任务没有新增字段）。`test_diagnostics` 若有“枚举数量 / 规格表覆盖所有码”之类的断言失败，按其断言把新码补进期望列表（这是该测试的维护方式），并在提交信息中说明。

- [ ] **Step 6: 提交**

```bash
cd "//wsl.localhost/Ubuntu-22.04/home/dan/project/hdy-motion-light" && git add include/common_types.h src/diagnostics.c src/motion_control.c tests/test_decel_semantics_fb.c && git commit -m "刹距不足告警与限制标志"
```
（若 Step 5 改了 `tests/test_diagnostics.c`，一并 `git add`。）

---

### Task 7: 限压 scale 回差带以下快速恢复（D3，修复现象1）

**Files:**
- Modify: `include/hyd_config.h`（§14B，`HYD_THRESH_PRESSURE_LIMIT_SCALE_RELEASE_RATE` 定义之后）
- Modify: `src/output_limiter.c`（`HYD_OutputLimiter_UpdatePressureLimitState` 的恢复分支）
- Test: `tests/test_output_limiter.c`

**Interfaces:**
- Consumes: `HYD_OutputLimiter_ExecuteWithProtection(input, state, output)`（`include/output_limiter.h`）；`HYD_OutputLimiterState.pressureScale` 可直接读
- Produces: 新 §14B 常量 `HYD_THRESH_PRESSURE_LIMIT_SCALE_FAST_RELEASE_RATE`（10.0f，单位 1/s）

**背景：** 实验中上一段是压力段（实测 3.0bar），开模段1 限压 2.0bar → 切段瞬间 scale→0；泄压 8ms 内完成，但 scale 只能以 2.0/s 爬回，泵速被压 500ms。回差带内的慢恢复是防振荡设计，要保留；压力已远低于释放阈值时没有振荡风险，应快速恢复。

- [ ] **Step 1: 写失败测试**

先读 `tests/test_output_limiter.c`，确认其断言风格（`assert` 或自定义宏）和 `main()` 的调用列表；在 `main()` 之前追加（若该文件未包含 `<string.h>`，补上）：

```c
static void test_pressure_scale_recovers_fast_below_release_threshold(void)
{
    HYD_OutputLimiterInput input;
    HYD_OutputLimiterState state;
    HYD_OutputLimiterOutput output;

    memset(&input, 0, sizeof(input));
    memset(&output, 0, sizeof(output));
    HYD_OutputLimiter_ResetState(&state);

    input.requestedFlow = 100.0;
    input.requestedPumpSpeed = 100.0;
    input.flowToPumpSpeedGain = 1.0;
    input.pumpSpeedLimit = 2000.0;
    input.effectiveMaxPressure = 2.0;    /* bar，释放阈值 2.0*(1-0.02)=1.96 */

    /* 1) 超压 50%：scale = 1 - 3.0*0.5 < 0 → 钳到 0 */
    input.actualPressure = 3.0;
    input.currentTime = 0.000;
    HYD_OutputLimiter_ExecuteWithProtection(&input, &state, &output);
    assert(state.pressureLimitActive);
    assert(state.pressureScale < 0.01);

    /* 2) 压力深落到 0（低于释放阈值）：10ms 内按 10/s 恢复到 ~0.10
     *    （旧行为 2/s 只有 0.02） */
    input.actualPressure = 0.0;
    input.currentTime = 0.010;
    HYD_OutputLimiter_ExecuteWithProtection(&input, &state, &output);
    assert(state.pressureScale > 0.09);
    assert(state.pressureScale < 0.12);

    /* 3) 回差带内（1.96 < p <= 2.0）：仍用慢速 2/s 防振荡 */
    state.pressureScale = 0.5;
    input.actualPressure = 1.98;
    input.currentTime = 0.020;
    HYD_OutputLimiter_ExecuteWithProtection(&input, &state, &output);
    assert(state.pressureScale > 0.5199 && state.pressureScale < 0.5201);
}
```
（若该文件用的不是 `assert`，把三处断言换成文件里已有的断言宏，语义不变。）

- [ ] **Step 2: 运行确认失败**

```bash
MSYS2_ARG_CONV_EXCL='*' wsl.exe -d Ubuntu-22.04 -- bash -lc 'cd /home/dan/project/hdy-motion-light && cmake --build out/build/unixgcc --target test_output_limiter && ctest --test-dir out/build/unixgcc -R "^test_output_limiter$" --output-on-failure'
```
预期：步骤 2) 的 `> 0.09` 断言失败（实际 0.02）。

- [ ] **Step 3: 最小实现**

`include/hyd_config.h`，在 `#define HYD_THRESH_PRESSURE_LIMIT_SCALE_RELEASE_RATE 2.0f` 之后追加（相邻常量没有 `#ifndef` 包裹，保持同样风格）：

```c

/* 限压解除后的快速恢复速率，单位 1/s。实际压力已低于释放阈值
 * maxPressure*(1-回差比例) 时使用，避免压力早已卸荷仍按慢速率爬坡
 * （实测：压力段切到低限压位置段后泵速爬坡 500ms）。仍处于回差带内时
 * 沿用 HYD_THRESH_PRESSURE_LIMIT_SCALE_RELEASE_RATE 防振荡。 */
#define HYD_THRESH_PRESSURE_LIMIT_SCALE_FAST_RELEASE_RATE 10.0f
```

`src/output_limiter.c`，恢复分支中把
```c
        releaseStep = HYD_THRESH_PRESSURE_LIMIT_SCALE_RELEASE_RATE * deltaTime;
```
替换为
```c
        releaseStep = ((actualPressure <= releasePressure)
                           ? HYD_THRESH_PRESSURE_LIMIT_SCALE_FAST_RELEASE_RATE
                           : HYD_THRESH_PRESSURE_LIMIT_SCALE_RELEASE_RATE) *
                      deltaTime;
```
其余逻辑（`deltaTime` 钳位、`releaseStep` 有效性检查、`>1.0` 钳位、释放条件、回差带内 0.999 保持）全部不动。

- [ ] **Step 4: 运行确认通过**

- [ ] **Step 5: 全量测试**（既有限压测试若断言了 2.0/s 的恢复斜率且场景压力低于释放阈值，按 Global Constraints 的规则处置：这是本任务有意改变的行为，更新期望并在提交信息中说明）

- [ ] **Step 6: 提交**

```bash
cd "//wsl.localhost/Ubuntu-22.04/home/dan/project/hdy-motion-light" && git add include/hyd_config.h src/output_limiter.c tests/test_output_limiter.c && git commit -m "限压scale回差带以下快速恢复"
```

---

### Task 8: 开模段序列复现 + 全量回归 + 生产构建

**Files:**
- Test: `tests/test_motion_planner_time_ramp.c`
- Modify: `docs/superpowers/specs/2026-11-10-decel-semantics-and-reachability-design.md`（头部“状态”行）

**Interfaces:**
- Consumes: Task 1/2/5 的全部行为契约（规划器级复现，单位 mm/s，t_acc=t_dec=100ms）

- [ ] **Step 1: 写序列测试**

复现实验开模段序列（真实值：段1 →50mm@18、段2 →60@90、段3 →70@54、段4 →90@18，各段终速=本段速度，即“入口钳制”写法）。段间状态 `state` 原样携带（等价于 motion_control 的 `preserveMotionPlannerState`）：

```c
static void test_mold_open_sequence_transitions_take_decel_time(void)
{
    static const HYD_REAL TARGET[4] = { 50.0, 60.0, 70.0, 90.0 };
    static const HYD_REAL VELOCITY[4] = { 18.0, 90.0, 54.0, 18.0 };
    HYD_AxisRef axis = {0};
    HYD_MotionPlannerState state = {0};
    HYD_MotionPlannerOutput output = {0};
    int seg;

    state.initialized = true;
    axis.position = 2.0;                      /* 实验中开模起点约 2mm */

    for (seg = 0; seg < 4; ++seg) {
        HYD_MotionSegment segment = {0};
        HYD_MotionBlendContext blend = {0};
        HYD_MotionPlannerInput input = {0};
        HYD_REAL entryVelocity = fabs(state.lastTargetVelocity);
        int transitionCycles = 0;
        int cycle;

        segment.mode = HYD_MODE_POSITION;
        segment.planner = HYD_PLANNER_POSITION_BASED;
        segment.endCondition = HYD_END_POSITION;
        segment.direction = HYD_DIRECTION_EXTEND;
        segment.targetPosition = TARGET[seg];
        segment.maxVelocity = VELOCITY[seg];
        segment.maxAcceleration = 100.0;
        segment.maxDeceleration = 100.0;
        segment.maxFlow = 1000.0;
        segment.velocityToFlowGain = 1.0;

        blend.active = true;
        blend.bufferMode = HYD_BUFFER_MODE_BLENDING_NEXT;
        blend.blendVelocity = VELOCITY[seg];  /* 终速 = 本段速度 */
        blend.switchPosition = TARGET[seg];
        blend.switchTolerance = 0.05;

        input.axisRef = &axis;
        input.segment = &segment;
        input.state = &state;
        input.blend = &blend;
        input.deltaTime = 0.001;

        for (cycle = 0; cycle < 20000; ++cycle) {
            HYD_REAL magnitude;

            input.elapsedTime = (cycle + 1) * 0.001;
            HYD_MotionPlanner_Execute(&input, &output);
            axis.position += output.targetVelocity * 0.001;
            axis.velocity = output.targetVelocity;
            magnitude = fabs(output.targetVelocity);
            if (fabs(magnitude - segment.maxVelocity) > 0.05 &&
                fabs(magnitude - entryVelocity) > 0.05) {
                transitionCycles++;   /* 处于段入口过渡斜坡上 */
            }
            if (axis.position >= segment.targetPosition - blend.switchTolerance) {
                break;
            }
        }
        check(axis.position >= segment.targetPosition - 0.1,
              "sequence segment must reach its switch position");
        if (seg > 0 && fabs(entryVelocity - segment.maxVelocity) > 1.0) {
            /* 实验值：90→54 用 67ms，54→18 用 200ms；修复后都应 ~100ms */
            check(transitionCycles >= 95 && transitionCycles <= 105,
                  "inter-segment velocity transition must take ~t_dec");
        }
    }
}
```
`main()` 追加 `test_mold_open_sequence_transitions_take_decel_time();`

说明：段1 从 0 起步（seg==0 不检查过渡时长，它的 18 来自加速）；段2 入口 18→90 为加速（经 Task 5 短行程判定：10mm 行程、vt=90 → 下限 clamp 为 vt，即与现状一致的 100ms 加速）；段3、段4 入口为减速——这两处正是现象 2a/2b。

- [ ] **Step 2: 运行**（命令同 Task 1 Step 2）。Task 1/2/5 已完成时应直接 PASS；若 FAIL，按失败的段号回到对应任务排查，不要放宽本测试的 95~105 窗口。

- [ ] **Step 3: 全量回归 + 嵌入式生产构建**

```bash
MSYS2_ARG_CONV_EXCL='*' wsl.exe -d Ubuntu-22.04 -- bash -lc 'cd /home/dan/project/hdy-motion-light && cmake --build --preset unixgcc && ctest --test-dir out/build/unixgcc --output-on-failure && ./scripts/deploy_embedded_prod.sh'
```
预期：ctest 全绿；生产构建成功。`scripts/deploy_embedded_prod.sh` 带有不属于本计划的未提交改动——**只运行，不提交、不修改**。若生产构建失败且原因与本计划改动无关（例如该脚本的未提交改动所致），在最终报告中如实说明，不要去修脚本。

- [ ] **Step 4: 更新 spec 状态并提交**

把 spec 头部 `- 状态：已确认（...）` 一行改为：
`- 状态：已实施（实现计划 docs/superpowers/plans/2026-11-10-decel-semantics-and-reachability.md）`

```bash
cd "//wsl.localhost/Ubuntu-22.04/home/dan/project/hdy-motion-light" && git add tests/test_motion_planner_time_ramp.c docs/superpowers/specs/2026-11-10-decel-semantics-and-reachability-design.md && git commit -m "开模段序列过渡时长集成验证"
```

---

## Self-Review 记录

- **Spec 覆盖**：D1 → Task 1（入口过渡）、2（制动触发）、3（SPEED_RAMP）、4（ADAPTENDVEL）；D2 → Task 5（可达峰值）、6（不可达告警）；D3 → Task 7；D4 → 各任务 TDD 步骤 + Task 8。spec“blend 相关逐一审计”的结论已写入 Global Constraints 的“明确不改”清单。
- **相对 spec 的有意偏差**（执行者须知）：
  1. Task 6 的诊断放在 `HYD_UpdateExecutionDiagnostics` 的优先级链里（每周期 WARNING + limit flag），没有用 spec 设想的“每段上报一次”——因为 FB 已到 3304 字节预算上限，不能加锁存字段；现有诊断本来就是每周期重算的模式。
  2. Task 4 的可达加速度用 (maxVelocity − projected)/t_acc，而不是 spec 表格里的 (requested − projected)/t_acc——规划器实际是朝 maxVelocity 锚定加速的，前者才与规划一致。
  3. Task 1 只放宽包络的**幅值上限**，sqrt 距离界不变；非 blend 的“实测超速”紧急路径行为与现状相同（见 Task 1 已知限制）。
- **占位符**：无 TBD/TODO；所有代码步骤都给出了可直接粘贴的代码与具体数值。
- **类型/命名一致性**：`HYD_ResolveBrakingAcceleration(segment, current, target)` 与现有签名一致；`setup_blend_braking_case`（Task 2 定义）、`make_position_segment`/`seed_running_position`（Task 4 定义）在后续任务中按同名同参复用；新枚举 `HYD_DIAG_CODE_BRAKING_DISTANCE_INSUFFICIENT`、`HYD_LIMIT_FLAG_BRAKING_DISTANCE`、常量 `HYD_THRESH_PRESSURE_LIMIT_SCALE_FAST_RELEASE_RATE` 前后一致。
