# 保持流量终点不做速度域刹车 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 泵流量百分比模式下，“保持流量”终点的 MoveContinuousAbsolute 段不再按减速时间做可达峰值封顶和速度域刹车，泵按流量满转，泵转速凹弧与减速时间解耦。

**Architecture:** 只改段启动时传给规划器的终点速度 `crossingVelocity`：在 `HYD_ResolveContinuousAbsoluteCrossingVelocity` 中，零终速判定之后，若本段被判为保持流量终点，则返回本段 `maxVelocity`（700）。判定放在一个新的 static 函数里，条件为：百分比模式；终点流量 ≈ 本段流量；本段不降流量。规划器、FB_HydAxis、IEC 接口、ABI、FB 尺寸都不变。

**Tech Stack:** C99（HydroMotionLib）、CMake/CTest（`unixgcc` preset，在 WSL Ubuntu-22.04 中构建），测试用 `assert`。

**Spec:** `docs/superpowers/specs/2026-11-18-clamp-flow-percent-design.md`。本计划实现其中 2026-11-20 修订的 §4.3.7、§6 T7~T9、§9 第 5 条；回放证据见 §5.1，风险见 §8。

## Global Constraints

- 纯 C99、`HYD_` 前缀、静态内存、无 malloc（CLAUDE.md 约束 5）。
- 不新增调参常数：容差复用 `HYD_FLOW_PERCENT_RAMP_EQUAL_RATIO`（`include/hyd_config.h`，0.02），即 `r·Q_base`。
- `Q_base = HYD_ResolveFlowPercentBase(fb)`（本机 80.75 L/min）。
- 判定规则（spec §4.3.7，全部满足才返回 `segment->maxVelocity`）：
  1. `ctx->flowPercent > 0`；终点方向与本段一致、`|ENDVELOCITY|` 大于零终速阈值（这两条由现有代码在前面判定）；
  2. `HYD_MotionControlFB_ResolveMaxTemplateVelocity(fb, target, target, ctx->approachDirection, …)` 成功，且 `v_lim > 0`、`q_lim > 0`；
  3. `q_end ≥ q_seg − r·Q_base`，其中 `q_end = |ENDVELOCITY| / v_lim × q_lim`，`q_seg = min(flowPercent% × Q_base, q_lim)`；
  4. `q_seg ≥ q0 − r·Q_base`，其中 `q0 = |_plannerState.lastTargetFlow|`。
- 判定与 `adaptEndVelEnabled` 无关。
- `sustainVelocity`（ENDVELOCITY）不变。
- 只改 `src/motion_control.c` 和 `tests/test_toggle_flow_percent_integration.c`。不改 `motion_planner.c`、`iec/FB_HydAxis.st`、`include/*.h`、`pousHydMotion.xml`、`CMakeLists.txt`。
- `PUMPFLOWPERCENT = 0` 的路径行为不变。现有 62 个 CTest 全部保持通过，现有 T1~T6 的断言不改。
- 换行符保持原样：`src/motion_control.c` 是 CRLF，`tests/test_toggle_flow_percent_integration.c` 是 LF。不要整文件转换；提交前 `git diff --stat` 中 `motion_control.c` 应只有约 +60 行。
- 每次 `git add` 只加本任务列出的文件。不提交 `docs/data_deg/*.csv`、`scripts/*.sh`、`.omx/metrics.json`、`docs/2026-11-18-clamp-speed-fix.md`；禁止 `git add -A`、`git add .`、`git commit -a`。
- 提交在 WSL 中执行，使用仓库已配置的身份。提交信息末尾加：`Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>`。

## 构建与运行

仓库在 WSL 里，所有命令都在 WSL 中、仓库根目录执行：

```bash
wsl.exe -d Ubuntu-22.04 bash -lc 'cd ~/project/hdy-motion-light && <命令>'
```

- 构建：`cmake --build --preset unixgcc 2>&1 | tail -20`。不新增 `.c` 文件，不用重新 configure。
- 单测：`./out/build/unixgcc/test_toggle_flow_percent_integration`
- 全量：`ctest --test-dir out/build/unixgcc --output-on-failure 2>&1 | tail -5`

基线（2026-11-20）：`100% tests passed, 0 tests failed out of 62`。

## 文件结构

| 文件 | 改动 | 职责 |
|---|---|---|
| `src/motion_control.c` | 改 | 前向声明 `HYD_ResolveFlowPercentBase`；新增 `HYD_IsContinuousAbsoluteFlowHoldEnd`；在 `HYD_ResolveContinuousAbsoluteCrossingVelocity` 中调用 |
| `tests/test_toggle_flow_percent_integration.c` | 改 | 追加 T7、T8、T9，并更新文件头注释 |

---

### Task 1: 保持流量终点判定（T7~T9）

**Files:**
- Modify: `src/motion_control.c`：前向声明块（第 66~68 行之后）；`HYD_HasCarriedAccelerationAnchor` 与 `HYD_ResolveContinuousAbsoluteCrossingVelocity` 之间（约第 1086 行）；`HYD_ResolveContinuousAbsoluteCrossingVelocity` 内零终速判定之后（约第 1109~1111 行）
- Test: `tests/test_toggle_flow_percent_integration.c`：第 1 行注释；在 `main` 之前追加 T7~T9；`main` 中追加 3 个调用

**Interfaces:**
- Consumes（现有，不改）：
  - `HYD_BOOL HYD_MotionControlFB_ResolveMaxTemplateVelocity(const HYD_MotionControlFB *fb, HYD_REAL xFrom, HYD_REAL xTo, HYD_MotionDirection direction, HYD_REAL *maxTemplateVelocity, HYD_REAL *velocityRatioMax, HYD_REAL *flowLimit, HYD_DiagnosticCode *code)`
  - `static HYD_REAL HYD_ResolveFlowPercentBase(const HYD_MotionControlFB* fb)`（定义在 `motion_control.c` 约第 1156 行，位于调用点之后，需要前向声明）
  - 测试文件现有 helper：`init_axis(fb, position, responseDelayMs, velocityTolerance)`、`make_segment(target, dir, accMs, decMs, velocityTolerance)`、`make_context(target, endVelocity, dir, percent)`、`ceiling_at(fb, x, dir, percent)`、`plant_step(fb, plant)`、`is_flag(fb, flag)`、`PumpPlant`、`PUMP_MAX_RPM`
- Produces：`static HYD_BOOL HYD_IsContinuousAbsoluteFlowHoldEnd(const HYD_MotionControlFB* fb, const HYD_ContinuousAbsoluteContext* ctx, const HYD_MotionSegment* segment, HYD_REAL requested)`，文件内私有，无外部消费者。

- [ ] **Step 1: 写失败测试**

`tests/test_toggle_flow_percent_integration.c` 第 1 行：

```c
/* 曲肘轴开合模“泵流量百分比”语义集成测试（spec 2026-11-18 §6 T1~T5）。
```

改为：

```c
/* 曲肘轴开合模“泵流量百分比”语义集成测试（spec 2026-11-18 §6 T1~T5、T7~T9）。
```

在 `int main(void)` 之前（`test_zero_percent_keeps_legacy_velocity_semantics` 函数之后）插入：

```c
/* T7（spec §4.3.7，2026-11-20）：保持流量段与减速时间无关。
 * 合模 299.36→30（100%、加速 300、减速 X，终点 = 30mm 处 100% 速度）→ 10（45%、200/150）。
 * 现库（无 §4.3.7）回放：减速 100 时 x∈[40,220] 泵转速最低 1698rpm；减速 480 时最低约 1493rpm、
 * 第 1 段报 BRAKING_DISTANCE、耗时 870ms；减速 800 时最低约 1068rpm、耗时 1127ms。 */
typedef struct {
    HYD_REAL crossing;
    HYD_REAL minPump;
    HYD_REAL v10;
    HYD_REAL v10Target;
    unsigned int seg1Cycles;
    HYD_BOOL seg1Braking;
    HYD_BOOL sawDeviation;
} FlowHoldCloseRun;

static FlowHoldCloseRun run_close_flow_hold(HYD_REAL seg1DecMs)
{
    HYD_MotionControlFB fb;
    HYD_MotionSegment seg1 = make_segment(30.0f, HYD_DIRECTION_RETRACT, 300.0f, seg1DecMs, 60.0f);
    HYD_MotionSegment seg2 = make_segment(10.0f, HYD_DIRECTION_RETRACT, 200.0f, 150.0f, 60.0f);
    HYD_ContinuousAbsoluteContext ctx1;
    HYD_ContinuousAbsoluteContext ctx2;
    PumpPlant plant;
    FlowHoldCloseRun r;
    unsigned int cycle;

    memset(&r, 0, sizeof(r));
    r.minPump = 2.0f * PUMP_MAX_RPM;
    memset(&plant, 0, sizeof(plant));
    plant.delayCycles = 20;
    plant.sign = -1.0f;
    init_axis(&fb, 299.36f, 20.0f, 60.0f);
    r.v10Target = ceiling_at(&fb, 10.0f, HYD_DIRECTION_RETRACT, 45.0f);
    ctx1 = make_context(30.0f, ceiling_at(&fb, 30.0f, HYD_DIRECTION_RETRACT, 100.0f),
                        HYD_DIRECTION_RETRACT, 100.0f);
    ctx2 = make_context(10.0f, r.v10Target, HYD_DIRECTION_RETRACT, 45.0f);

    assert(HYD_MotionControlFB_StartDirectCommand(&fb, HYD_DIRECT_CMD_MOVE_CONTINUOUS_ABSOLUTE,
                                                  &seg1, &ctx1, HYD_BUFFER_MODE_ABORT, 0.0f) ==
           HYD_DIRECT_START_STARTED);
    r.crossing = fb._directContinuousAbsolute.crossingVelocity;
    for (cycle = 0u; cycle < 3000u &&
         fb._directContinuousAbsolute.phase == HYD_CONTABS_PHASE_APPROACH; ++cycle) {
        plant_step(&fb, &plant);
        assert(fb.FB_STATE != HYD_FB_STATE_FAULT);
        if (fb.AXIS_REF.position >= 40.0f && fb.AXIS_REF.position <= 220.0f &&
            fb.PUMP_SPEED < r.minPump) {
            r.minPump = fb.PUMP_SPEED;
        }
        r.seg1Braking |= is_flag(&fb, HYD_LIMIT_FLAG_BRAKING_DISTANCE);
        r.sawDeviation |= fb.DIAGNOSTIC.velocityDeviation;
    }
    assert(fb._directContinuousAbsolute.phase == HYD_CONTABS_PHASE_SUSTAIN);
    r.seg1Cycles = cycle;

    assert(HYD_MotionControlFB_StartDirectCommand(&fb, HYD_DIRECT_CMD_MOVE_CONTINUOUS_ABSOLUTE,
                                                  &seg2, &ctx2, HYD_BUFFER_MODE_ABORT,
                                                  fb.AXIS_REF.timestamp) ==
           HYD_DIRECT_START_STARTED);
    for (cycle = 0u; cycle < 3000u && fb.AXIS_REF.position > 10.0f; ++cycle) {
        plant_step(&fb, &plant);
        assert(fb.FB_STATE != HYD_FB_STATE_FAULT);
        r.sawDeviation |= fb.DIAGNOSTIC.velocityDeviation;
    }
    assert(fb.AXIS_REF.position <= 10.0f);
    r.v10 = -fb.AXIS_REF.velocity;
    return r;
}

static void test_close_flow_hold_ignores_deceleration_time(void)
{
    static const HYD_REAL decs[3] = {100.0f, 480.0f, 800.0f};
    FlowHoldCloseRun r[3];
    int i;
    int spread;

    /* 先跑完三组并打印，失败时能看到全部数据（assert 失败会 abort，须先 fflush） */
    for (i = 0; i < 3; ++i) {
        r[i] = run_close_flow_hold(decs[i]);
        printf("T7 flow-hold close d%.0f: crossing %.1f, min pump %.2f (x 40~220), braking %d, "
               "seg1 %u ms, v@10 %.1f (target %.1f)\n",
               decs[i], r[i].crossing, r[i].minPump, (int)r[i].seg1Braking,
               r[i].seg1Cycles, r[i].v10, r[i].v10Target);
    }
    fflush(stdout);
    for (i = 0; i < 3; ++i) {
        assert(r[i].minPump >= PUMP_MAX_RPM - 0.5f);
        assert(!r[i].seg1Braking);
        assert(!r[i].sawDeviation);
        assert(fabsf(r[i].v10 - r[i].v10Target) <= 0.10f * r[i].v10Target);
        assert(fabsf(r[i].crossing - 700.0f) <= 0.01f);
        spread = (int)r[i].seg1Cycles - (int)r[0].seg1Cycles;
        assert(spread >= -1 && spread <= 1);
    }
}

/* T8（spec §4.3.7 条件 4）：降流量的保持流量段仍保留速度域刹车。
 * 299.36→260（100%、100/100，终点 100%）→ 200（40%、300/300，终点 = 200mm 处 40% 速度）。
 * 第 2 段 60mm 内流量斜坡走不完，速度域刹车是到点速度达标的唯一保证（spec §5.1）。
 * 第 1 段从静止起步、100% 保持流量，须触发规则（crossing = 700），第 2 段的“不触发”才有意义。 */
static void test_ramp_down_flow_hold_keeps_velocity_brake(void)
{
    HYD_MotionControlFB fb;
    HYD_MotionSegment seg1 = make_segment(260.0f, HYD_DIRECTION_RETRACT, 100.0f, 100.0f, 60.0f);
    HYD_MotionSegment seg2 = make_segment(200.0f, HYD_DIRECTION_RETRACT, 300.0f, 300.0f, 60.0f);
    HYD_ContinuousAbsoluteContext ctx1;
    HYD_ContinuousAbsoluteContext ctx2;
    PumpPlant plant;
    HYD_REAL v200;
    HYD_REAL seg1Crossing;
    HYD_REAL seg2Crossing;
    unsigned int cycle;

    memset(&plant, 0, sizeof(plant));
    plant.delayCycles = 20;
    plant.sign = -1.0f;
    init_axis(&fb, 299.36f, 20.0f, 60.0f);
    v200 = ceiling_at(&fb, 200.0f, HYD_DIRECTION_RETRACT, 40.0f);
    ctx1 = make_context(260.0f, ceiling_at(&fb, 260.0f, HYD_DIRECTION_RETRACT, 100.0f),
                        HYD_DIRECTION_RETRACT, 100.0f);
    ctx2 = make_context(200.0f, v200, HYD_DIRECTION_RETRACT, 40.0f);

    assert(HYD_MotionControlFB_StartDirectCommand(&fb, HYD_DIRECT_CMD_MOVE_CONTINUOUS_ABSOLUTE,
                                                  &seg1, &ctx1, HYD_BUFFER_MODE_ABORT, 0.0f) ==
           HYD_DIRECT_START_STARTED);
    seg1Crossing = fb._directContinuousAbsolute.crossingVelocity;
    for (cycle = 0u; cycle < 3000u &&
         fb._directContinuousAbsolute.phase == HYD_CONTABS_PHASE_APPROACH; ++cycle) {
        plant_step(&fb, &plant);
        assert(fb.FB_STATE != HYD_FB_STATE_FAULT);
    }
    assert(fb._directContinuousAbsolute.phase == HYD_CONTABS_PHASE_SUSTAIN);

    assert(HYD_MotionControlFB_StartDirectCommand(&fb, HYD_DIRECT_CMD_MOVE_CONTINUOUS_ABSOLUTE,
                                                  &seg2, &ctx2, HYD_BUFFER_MODE_ABORT,
                                                  fb.AXIS_REF.timestamp) ==
           HYD_DIRECT_START_STARTED);
    seg2Crossing = fb._directContinuousAbsolute.crossingVelocity;

    printf("T8 ramp-down flow-hold: seg1 crossing %.1f, seg2 crossing %.2f (end velocity %.2f)\n",
           seg1Crossing, seg2Crossing, v200);
    fflush(stdout);
    assert(fabsf(seg1Crossing - 700.0f) <= 0.01f);
    assert(fabsf(seg2Crossing - v200) <= 0.01f);
}

/* T9（spec §4.3.7）：判定边界，从静止启动后读 crossingVelocity。
 * 本机 q_lim = Q_base，条件 3 等价于终点百分比 ≥ 本段百分比 − 2%。 */
typedef struct {
    const char *name;
    HYD_REAL x0;
    HYD_REAL target;
    HYD_MotionDirection dir;
    HYD_REAL percent;
    HYD_REAL endPercent;
    HYD_BOOL flowHold;
} FlowHoldCase;

static HYD_REAL start_and_read_crossing(const FlowHoldCase *c, HYD_REAL *endVelocity)
{
    HYD_MotionControlFB fb;
    HYD_MotionSegment seg = make_segment(c->target, c->dir, 300.0f, 480.0f, 60.0f);
    HYD_ContinuousAbsoluteContext ctx;

    init_axis(&fb, c->x0, 20.0f, 60.0f);
    if (!(c->percent > 0.0f)) {
        seg.maxVelocity = 200.0f;   /* 旧语义：VELOCITY 是动模板速度 */
    }
    *endVelocity = ceiling_at(&fb, c->target, c->dir, c->endPercent);
    ctx = make_context(c->target, *endVelocity, c->dir, c->percent);
    assert(HYD_MotionControlFB_StartDirectCommand(&fb, HYD_DIRECT_CMD_MOVE_CONTINUOUS_ABSOLUTE,
                                                  &seg, &ctx, HYD_BUFFER_MODE_ABORT, 0.0f) ==
           HYD_DIRECT_START_STARTED);
    return fb._directContinuousAbsolute.crossingVelocity;
}

static void test_flow_hold_classification_boundaries(void)
{
    static const FlowHoldCase cases[] = {
        {"close 100%/end 99%", 100.0f, 30.0f, HYD_DIRECTION_RETRACT, 100.0f, 99.0f, true},
        {"close 100%/end 97%", 100.0f, 30.0f, HYD_DIRECTION_RETRACT, 100.0f, 97.0f, false},
        {"close 60%/end 60%", 100.0f, 30.0f, HYD_DIRECTION_RETRACT, 60.0f, 60.0f, true},
        {"open 95%/end 95%", 1.0f, 30.0f, HYD_DIRECTION_EXTEND, 95.0f, 95.0f, true},
        {"legacy 0%", 100.0f, 30.0f, HYD_DIRECTION_RETRACT, 0.0f, 30.0f, false},
    };
    unsigned int i;

    for (i = 0u; i < sizeof(cases) / sizeof(cases[0]); ++i) {
        HYD_REAL endVelocity = 0.0f;
        HYD_REAL crossing = start_and_read_crossing(&cases[i], &endVelocity);
        HYD_REAL expected = cases[i].flowHold ? 700.0f : endVelocity;

        printf("T9 %s: end velocity %.2f, crossing %.2f (expected %.2f)\n",
               cases[i].name, endVelocity, crossing, expected);
        fflush(stdout);
        assert(fabsf(crossing - expected) <= 0.01f);
    }
}
```

`main` 中，在 `test_zero_percent_keeps_legacy_velocity_semantics();` 之后、`printf("test_toggle_flow_percent_integration passed\n");` 之前追加：

```c
    test_close_flow_hold_ignores_deceleration_time();
    test_ramp_down_flow_hold_keeps_velocity_brake();
    test_flow_hold_classification_boundaries();
```

- [ ] **Step 2: 运行，确认失败**

这一步同时就是 spec T7 要求的变异检查：在没有 §4.3.7 的库上运行 T7，测试必须失败。

Run:
```bash
wsl.exe -d Ubuntu-22.04 bash -lc 'cd ~/project/hdy-motion-light && cmake --build --preset unixgcc 2>&1 | tail -3 && ./out/build/unixgcc/test_toggle_flow_percent_integration'
```

Expected：编译无新告警。T4/T5/I1 的打印与现在相同（T5 `switch pump 1611.5`，I1 `seg2 864 ms, cruise peak 1575.4`）。T7 三行打印为（2026-11-20 临时副本实测）：

```
T7 flow-hold close d100: crossing 208.4, min pump 1698.04 (x 40~220), braking 0, seg1 831 ms, v@10 45.3 (target 47.7)
T7 flow-hold close d480: crossing 208.4, min pump 1493.09 (x 40~220), braking 1, seg1 870 ms, v@10 45.3 (target 47.7)
T7 flow-hold close d800: crossing 208.4, min pump 1068.18 (x 40~220), braking 1, seg1 1127 ms, v@10 45.3 (target 47.7)
```

随后 `Assertion 'r[i].minPump >= PUMP_MAX_RPM - 0.5f' failed`，进程 abort，退出码非 0。

如果 T7 没有失败，或者失败在别的断言上，停下检查测试代码，不要继续。

- [ ] **Step 3: 实现判定函数并调用**

`src/motion_control.c` 前向声明块中，找到：

```c
static void HYD_CopyContinuousAbsoluteContext(HYD_ContinuousAbsoluteContext* dst,
                                              const HYD_ContinuousAbsoluteContext* src);
```

在其后加一行：

```c
static HYD_REAL HYD_ResolveFlowPercentBase(const HYD_MotionControlFB* fb);
```

在 `HYD_HasCarriedAccelerationAnchor` 函数结束（`}`）之后、`static HYD_REAL HYD_ResolveContinuousAbsoluteCrossingVelocity(` 之前插入：

```c
/* spec §4.3.7（2026-11-20）保持流量终点：终点流量 ≈ 本段流量、且本段不降流量时，段内不需要
 * 速度域刹车——泵流量不变，曲肘速比自己让动模板降到终点速度。
 * 条件 3：q_end ≥ q_seg − r·Q_base，q_end = |v_end| / v_lim × q_lim，q_seg = min(pct·Q_base, q_lim)；
 * 条件 4：q_seg ≥ q0 − r·Q_base，q0 为续接恢复后的上一拍指令流量（与流量斜坡锚定同一起点）。
 * 降流量段的流量斜坡可能在目标点前走不完，不满足条件 4，保留速度域刹车兜底。
 * r = HYD_FLOW_PERCENT_RAMP_EQUAL_RATIO，与斜坡锚定共用“流量近似相等”的定义。 */
static HYD_BOOL HYD_IsContinuousAbsoluteFlowHoldEnd(const HYD_MotionControlFB* fb,
                                                   const HYD_ContinuousAbsoluteContext* ctx,
                                                   const HYD_MotionSegment* segment,
                                                   HYD_REAL requested) {
    HYD_REAL base;
    HYD_REAL velocityLimit = 0.0f;
    HYD_REAL ratio = 0.0f;
    HYD_REAL flowLimit = 0.0f;
    HYD_REAL segmentFlow;
    HYD_REAL endFlow;
    HYD_REAL tolerance;
    HYD_DiagnosticCode code = HYD_DIAG_CODE_NONE;

    if (!(ctx->flowPercent > 0.0f)) {
        return false;
    }
    base = HYD_ResolveFlowPercentBase(fb);
    if (!(base > 0.0f) ||
        !HYD_MotionControlFB_ResolveMaxTemplateVelocity(fb,
                                                        segment->targetPosition,
                                                        segment->targetPosition,
                                                        ctx->approachDirection,
                                                        &velocityLimit,
                                                        &ratio,
                                                        &flowLimit,
                                                        &code) ||
        !(velocityLimit > 0.0f) || !(flowLimit > 0.0f)) {
        return false;
    }
    tolerance = HYD_FLOW_PERCENT_RAMP_EQUAL_RATIO * base;
    segmentFlow = ctx->flowPercent * 0.01f * base;
    if (segmentFlow > flowLimit) {
        segmentFlow = flowLimit;
    }
    endFlow = requested / velocityLimit * flowLimit;
    if (endFlow < segmentFlow - tolerance) {
        return false;
    }
    return segmentFlow >= fabs(fb->_plannerState.lastTargetFlow) - tolerance;
}

```

在 `HYD_ResolveContinuousAbsoluteCrossingVelocity` 中，找到：

```c
    if (requested <= HYD_THRESH_DECEL_TARGET_VEL_DONE) {
        return 0.0f;
    }

    remainingDistance = fabs(segment->targetPosition - fb->AXIS_REF.position);
```

改为：

```c
    if (requested <= HYD_THRESH_DECEL_TARGET_VEL_DONE) {
        return 0.0f;
    }

    /* spec §4.3.7：保持流量终点不做可达峰值封顶和速度域刹车。只改传给规划器的
     * 终点速度，sustainVelocity 与 InEndVelocity 判定不变；与 adaptEndVelEnabled 无关。 */
    if (HYD_IsContinuousAbsoluteFlowHoldEnd(fb, ctx, segment, requested)) {
        return segment->maxVelocity;
    }

    remainingDistance = fabs(segment->targetPosition - fb->AXIS_REF.position);
```

- [ ] **Step 4: 运行单测，确认通过**

Run:
```bash
wsl.exe -d Ubuntu-22.04 bash -lc 'cd ~/project/hdy-motion-light && cmake --build --preset unixgcc 2>&1 | grep -E "warning|error|Built target HydroMotionLib" ; ./out/build/unixgcc/test_toggle_flow_percent_integration'
```

Expected：没有 warning/error，打印如下（数值来自 2026-11-20 临时副本实测）：

```
T4 open: 1162 ms, final 300.217, peak 541.5, brake 665 ms
T5 close: switch pump 1615.0, max drop 5.667 rpm/cycle, v@10 45.3 (target 47.7)
I1 same-percent chain: seg2 849 ms, cruise peak 1615.0 rpm (x 60~100), pump peak 1615.0, final 300.217
T7 flow-hold close d100: crossing 700.0, min pump 1700.00 (x 40~220), braking 0, seg1 831 ms, v@10 45.3 (target 47.7)
T7 flow-hold close d480: crossing 700.0, min pump 1700.00 (x 40~220), braking 0, seg1 831 ms, v@10 45.3 (target 47.7)
T7 flow-hold close d800: crossing 700.0, min pump 1700.00 (x 40~220), braking 0, seg1 831 ms, v@10 45.3 (target 47.7)
T8 ramp-down flow-hold: seg1 crossing 700.0, seg2 crossing 191.31 (end velocity 191.31)
T9 close 100%/end 99%: end velocity 206.36, crossing 700.00 (expected 700.00)
T9 close 100%/end 97%: end velocity 202.20, crossing 202.20 (expected 202.20)
T9 close 60%/end 60%: end velocity 125.07, crossing 700.00 (expected 700.00)
T9 open 95%/end 95%: end velocity 283.74, crossing 700.00 (expected 700.00)
T9 legacy 0%: end velocity 62.53, crossing 62.53 (expected 62.53)
test_toggle_flow_percent_integration passed
```

T5 换段泵转速由 1611.5 变为 1615.0，I1 的 seg2 时间由 864 变为 849ms、巡航峰值由 1575.4 变为 1615.0rpm，这些是 spec §5.1 预期的变化，现有断言不改。末位小数可以有 ±1 的出入。

- [ ] **Step 5: 全量回归**

Run:
```bash
wsl.exe -d Ubuntu-22.04 bash -lc 'cd ~/project/hdy-motion-light && ctest --test-dir out/build/unixgcc --output-on-failure 2>&1 | tail -5'
```

Expected：`100% tests passed, 0 tests failed out of 62`

- [ ] **Step 6: 提交**

```bash
wsl.exe -d Ubuntu-22.04 bash -lc 'cd ~/project/hdy-motion-light && git add src/motion_control.c tests/test_toggle_flow_percent_integration.c && git commit -q -m "MoveContinuousAbsolute 泵流量百分比：保持流量终点不做速度域刹车（spec §4.3.7）" -m "终点流量约等于本段流量、且本段不降流量时，段启动的 crossingVelocity 取本段 VELOCITY，规划器不再按减速时间做可达峰值封顶和速度域刹车。新增 T7~T9。" -m "Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>" && git log --oneline -1 && git status --short'
```

Expected：输出新提交；`git status` 中只剩未跟踪的 CSV（以及用户原有的 `scripts/*.sh` 修改，如果有）。

- [ ] **Step 7: 条件 4 变异检查（spec T8）**

临时修改 `src/motion_control.c` 中 `HYD_IsContinuousAbsoluteFlowHoldEnd` 的最后一行：

```c
    return segmentFlow >= fabs(fb->_plannerState.lastTargetFlow) - tolerance;
```

改为：

```c
    return true;
```

Run:
```bash
wsl.exe -d Ubuntu-22.04 bash -lc 'cd ~/project/hdy-motion-light && cmake --build --preset unixgcc 2>&1 | tail -1 && ./out/build/unixgcc/test_toggle_flow_percent_integration; echo exit=$?'
```

Expected：退出码非 0，失败在 T8。2026-11-20 临时副本实测：
- T4、T5、I1 通过；
- T7 通过，但 `v@10` 由 45.3 变为 48.5（第 2 段 100%→45% 降流量也被判为保持流量，仍在 ±10% 内）；
- T8 打印 `seg2 crossing 700.00 (end velocity 191.31)`，随后 `Assertion 'fabsf(seg2Crossing - v200) <= 0.01f' failed`。

如果失败位置不同，在任务报告中记下失败位置和打印内容；只要退出码非 0，变异就算被捕获。

还原并确认：

```bash
wsl.exe -d Ubuntu-22.04 bash -lc 'cd ~/project/hdy-motion-light && git checkout -- src/motion_control.c && git diff --stat && cmake --build --preset unixgcc 2>&1 | tail -1 && ./out/build/unixgcc/test_toggle_flow_percent_integration | tail -1'
```

Expected：`git diff --stat` 无输出；最后一行为 `test_toggle_flow_percent_integration passed`。

---

## 上机（不属于代码任务，由用户执行）

全部 ctest 通过后，PLC 重新链接库，按 spec §7“§4.3.7 上机验收”执行：先保持 11-20 段参数不变，再把第 2 段减速从 151 改成 480，合模周期变化应 ≤ 5ms。ABI、IEC 接口、FB_HydAxis 不变，PLC 工程不用重新生成接口。
