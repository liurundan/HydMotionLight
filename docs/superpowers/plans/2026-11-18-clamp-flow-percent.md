# 曲肘轴开合模“泵流量百分比”速度语义 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 给 `HYD_MoveContinuousAbsolute` 增加 `PUMPFLOWPERCENT` 输入，让曲肘轴开合模巡航按“泵流量百分比”跑满泵、停车仍按动模板位置规划，开模从 2.02s 降到约 1.1s、合模从 1.50s 降到约 1.3s，且无顿挫、无急停。

**Architecture:** 只改 MoveContinuousAbsolute 的执行路径。段启动时在上下文里锁存流量百分比和斜坡速率；每个控制周期把 `min(现有流量上限, 斜坡后的百分比流量)` 交给现有的机构映射；映射报告限流时，把规划器状态回算成实际可实现的动模板速度，使刹车触发距离按真实速度计算。规划器、运动学、压力环、Stop 均不改。IEC 层追加一个输入；FB_HydAxis 增加新旧模式开关。

**Tech Stack:** C99（HydroMotionLib）、matiec IEC 类型、CMake/CTest（`unixgcc` preset，在 WSL Ubuntu-22.04 中构建）、IEC 61131-3 ST（`iec/FB_HydAxis.st`）、Python 3 契约测试。

**Spec:** `docs/superpowers/specs/2026-11-18-clamp-flow-percent-design.md`（已确认，2026-11-18 按代码与探针复核修订）

## Global Constraints

- 纯 C99、`HYD_` 前缀、静态内存、无 malloc（CLAUDE.md 约束 5）。
- 调参型浮点阈值放 `include/hyd_config.h` §14B，不在 `.c` 里内联（约束 7）；本计划不新增调参阈值。
- `PUMPFLOWPERCENT = 0` 时行为与现有完全一致；现有 61 个 CTest 全部保持通过。
- 百分比基准 `Q_base = HYD_GetPumpFlowLimit(fb)`（本机 80.75 L/min）；不大于 0 时取 `fb->_params.maxFlow`。
- 斜坡语义：固定耗时 `|q_target − q0| / t`，上升取 `maxAcceleration`、下降取 `maxDeceleration`（ms，经 `HYD_MotionTimeMsToSeconds`，最小 100ms）；`q_target == q0` 时取 `Q_base / t_acc`。
- 合法输入：有限值且 `0 ≤ PUMPFLOWPERCENT ≤ 100`；否则上升沿 `ERROR = TRUE`、`ERRORID = HYD_DIAG_CODE_COMMAND_NOT_ALLOWED`。
- FB 尺寸：`sizeof(HYD_MotionControlFB)` 由 3312 变为 3328（已在 WSL 实测）。
- FB_HydAxis：`bFlowPctMode : BOOL := TRUE`；`rToggleVelCap = 700.0` mm/s；`PUMPFLOWPERCENT := LIMIT(0.0, uiSpd×0.1, 100.0)`，不复用 `rFlowPercent`（后者下限 5%）。
- 只做 spec 列出的改动；不改 `motion_planner.c`、`actuation_mapper.c`、`output_limiter`、MoveVelocity、配方路径。
- 不提交用户未跟踪/已修改的无关文件（`.omx/metrics.json`、`scripts/*.sh`、CSV、`docs/2026-11-18-clamp-speed-fix.md`、被删的 11-13 CSV）。每次 `git add` 只加本任务列出的文件。
- 提交信息末尾加：`Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>`

## 构建与运行（所有任务通用）

仓库在 WSL 里，所有命令都在 WSL 中、仓库根目录执行：

```bash
wsl.exe -d Ubuntu-22.04 bash -lc 'cd ~/project/hdy-motion-light && <命令>'
```

- 新增 `tests/*.c` 后要重新 configure：`cmake --preset unixgcc`
- 构建：`cmake --build --preset unixgcc 2>&1 | tail -20`
- 单测：`ctest --test-dir out/build/unixgcc -R '^<name>$' --output-on-failure`
- 全量：`ctest --test-dir out/build/unixgcc --output-on-failure 2>&1 | tail -5`

基线（2026-11-18 实测）：`100% tests passed, 0 tests failed out of 61`。

## 文件结构

| 文件 | 改动 | 职责 |
|---|---|---|
| `include/motion_control.h` | 改 | `HYD_ContinuousAbsoluteContext` 末尾加 `flowPercent`、`flowRampRate` |
| `src/motion_control.c` | 改 | 斜坡锚定 helper、每周期流量上限、回算、FLOW 标志 |
| `include/motion_interface.h` | 改 | `HYD_MOVECONTINUOUSABSOLUTE` 在 `PRESSURELIMIT` 后加 `PUMPFLOWPERCENT` |
| `src/motion_interface.c` | 改 | 校验输入、`buildContinuousAbsoluteContext` 增加形参 |
| `pousHydMotion.xml` | 改 | IEC 接口声明同步 |
| `tests/test_toggle_flow_percent_integration.c` | 新建 | T1~T5（核心层，105 几何，1ms 周期） |
| `tests/test_movecontinuousabsolute_integration.c` | 改 | T6 输入校验 |
| `tests/test_interface_layout_consistency.py` | 改 | XML 与 C 头文件字段顺序核对 |
| `tests/test_mode_switch_bumpless.c` | 改 | FB 尺寸预算 3312 → 3328 |
| `tests/test_hydaxis_p0_contract.py` | 改 | FB_HydAxis 文本契约 |
| `iec/FB_HydAxis.st` | 改 | `bFlowPctMode`、目标点标尺、PUMPFLOWPERCENT、速度安全上限 |
| `CMakeLists.txt` | 改 | 注册新测试 |

`tests/test_toggle_mechanism_pool.c` 的预算是 `3280 + 48 = 3328`，正好容纳，不用改。`tests/plcdemo/POUS.h/.c` 不含此 FB，不改。

## 关键设计说明（实现者必读）

1. **回算为什么必需**：探针显示，只限流不回算时，规划器 `lastTargetVelocity` 停在 VELOCITY 上限（700mm/s），按它算出的刹车距离远大于实际，开模提前刹车，总时长约 1.7s；回算后约 1.16s。T4 的耗时断言就是用来锁住这一点的。
2. **为什么 `q_cap` 以“规划器执行前”的 `lastTargetFlow` 为起点**：每个周期末，映射块把 `lastTargetFlow` 写成映射后的请求流量，即这一拍的实际流量指令。但 `HYD_MotionPlanner_Execute` 内部会先用不含速比的增益（`motion_planner.c:990`）改写它，所以必须在规划器执行前读出（Step 6 的 `mappedFlowBefore`）。干跑已验证：如果在规划器之后读，起步时 `q_prev` 几乎为 0，斜坡永远起不来，开模 4s 只走了 6mm。段启动锚定（Step 4）在两个周期之间调用，此时 `lastTargetFlow` 就是上一拍的映射流量，链式接管时也自然连续。
3. **为什么 `q_cap ≤ 0` 要换成 `HYD_REAL_EPSILON`**：`HYD_ActuationMapper_MapVelocity` 只在 `maxFlow > 0` 时限流，传 0 等于不限流。静止起步第一拍 `q_prev = 0`；若该拍 `dt = 0`，`q_cap` 会是 0。
4. **回算对象**：`plannerOutput->targetVelocity` 与 `fb->_plannerState.lastTargetVelocity` 一起改。`STATE.plannedVelocity`、执行参考速度、仿真反馈都由 `plannerOutput->targetVelocity` 派生（`state_reporter.c:383`、`motion_control.c:2678/3425`），不需要单独改。`transitionRate`、`transitionTargetVelocity` 不改：下一拍若目标不变，规划器从回算后的速度继续按原速率前进，被上限再次截住，保持贴着上限巡航。
5. **刹车时为什么不会被回算干扰**：刹车时规划速度单调下降，低于流量上限，`flowLimitActive = false`，不触发回算。只有流量斜坡本身比规划刹车更陡时才会触发（例如合模第 3 段降流量），此时回算让规划器跟随实际速度，正是需要的行为。
6. **D2 刹距诊断**：在探针中按 `motion_control.c:2545-2593` 的判据复刻，开模 500/600/700ms、合模 2/3 段场景都没有误报。T4、T5 仍断言无 `BRAKING_DISTANCE`。
7. **限幅标志**：百分比模式下，映射层的 FLOW 只在硬件上限小于 `q_cap` 时置位；映射层的 PUMP_SPEED 判定（`unlimitedRequestedFlow > 泵上限`）在百分比模式下跳过，因为规划器本拍提议的速度本来就会被 `q_cap` 截住。泵转速限幅仍由 `pumpOutput->speedLimitActive` 判定（spec §4.3.5）。
8. **测试对象模型**：新测试的对象按“泵转速 → 纯滞后 → 当前位置速比 → 动模板速度”运动，与可行性探针一致。对象跟随泵流量而不是规划速度，所以“只限流不回算”时规划速度与实际速度会分离，测试能抓到。

---

### Task 1: 库核心——上下文字段、斜坡锚定、流量上限、回算

**Files:**
- Modify: `include/motion_control.h:239-252`
- Modify: `src/motion_control.c`（helper 放在 `HYD_ResolveContinuousAbsoluteCrossingVelocity` 之后约 1153 行；三处锚定调用在 2007、4305、4364 行附近；流量上限与回算在 2595-2635）
- Create: `tests/test_toggle_flow_percent_integration.c`
- Modify: `CMakeLists.txt:220-221`、`CMakeLists.txt:326`
- Modify: `tests/test_mode_switch_bumpless.c:446-448`

**Interfaces:**
- Consumes：现有 `HYD_MotionControlFB_StartDirectCommand(fb, kind, segment, ctx, bufferMode, timestamp)`、`HYD_MotionControlFB_Cycle(fb)`、`HYD_MotionControlFB_ResolveMaxTemplateVelocity(...)`、`HYD_ToggleMechanismPool_*`
- Produces：
  - `HYD_ContinuousAbsoluteContext.flowPercent : HYD_REAL`（%，0 = 关闭）
  - `HYD_ContinuousAbsoluteContext.flowRampRate : HYD_REAL`（L/min/s，段启动时锚定）
  - `static void HYD_AnchorContinuousAbsoluteFlowRamp(HYD_MotionControlFB* fb, HYD_ContinuousAbsoluteContext* ctx, const HYD_MotionSegment* segment)`
  - `static HYD_REAL HYD_ResolveFlowPercentBase(const HYD_MotionControlFB* fb)`

- [ ] **Step 1: 写失败测试（T1~T5）**

创建 `tests/test_toggle_flow_percent_integration.c`：

```c
/* 曲肘轴开合模“泵流量百分比”语义集成测试（spec 2026-11-18 §6 T1~T5）。
 * 105 双曲肘（LF = 323），面积 3350/4800 mm²，泵 50cc/1700rpm/0.95，1ms 周期。
 * 对象：泵转速经 delayCycles 个周期纯滞后，再按当前位置速比换算为动模板速度
 * （与可行性探针同一模型）。对象按泵流量运动而不是按规划速度运动，
 * 所以“只限流不回算”时规划速度与实际速度会分离，T4 能抓到。 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "motion_control.h"
#include "toggle_mechanism_pool.h"

#define DT 0.001f
#define MAX_DELAY 64
#define PUMP_MAX_RPM 1700.0f
#define Q_LIMIT 80.75f   /* 1700 * 50 * 0.95 / 1000 */

typedef struct {
    HYD_REAL history[MAX_DELAY];   /* 泵转速 [rpm] */
    int head;
    int delayCycles;               /* 0 = 无纯滞后 */
    HYD_REAL sign;                 /* +1 开模（EXTEND），-1 合模（RETRACT） */
} PumpPlant;

static void bind_105(HYD_MotionControlFB *fb)
{
    HYD_ToggleGeometryConfig raw = {
        260.0f, 323.0f, 190.0f, 100.0f, 125.0f,
        200.0f, 170.0f, 580.0f, 450.0f,
        0.0f, (int8_t)-1, (int8_t)-1, (int8_t)-1, (int8_t)-1
    };
    HYD_TogglePreparedConfig prepared;
    HYD_ToggleError error = HYD_TOGGLE_ERROR_NONE;
    HYD_UINT8 slot = HYD_TOGGLE_SLOT_NONE;

    assert(HYD_ToggleKinematics_ValidateBlocking(&raw, &prepared, &error));
    HYD_ToggleMechanismPool_Reset();
    assert(HYD_ToggleMechanismPool_Reserve(0u, &slot));
    assert(HYD_ToggleMechanismPool_Commit(slot, &prepared, true));
    fb->mechanismType = (HYD_UINT8)HYD_MECHANISM_FIVE_POINT_TOGGLE;
    fb->mechanismSlot = slot;
    fb->STATE.mechanismType = fb->mechanismType;
    fb->STATE.mechanismConfigVersion = HYD_ToggleMechanismPool_GetVersion(slot);
}

static void init_axis(HYD_MotionControlFB *fb, HYD_REAL position,
                      HYD_REAL responseDelayMs, HYD_REAL velocityTolerance)
{
    HYD_MotionControlFB_Init(fb);
    fb->USE_RECIPE = false;
    fb->pumpConfig.displacementMlRev = 50.0f;
    fb->pumpConfig.volumetricEfficiency = 0.95f;
    fb->pumpConfig.maxSpeedRpm = PUMP_MAX_RPM;
    fb->cylinderConfig.areaExtendMm2 = 3350.0f;
    fb->cylinderConfig.areaRetractMm2 = 4800.0f;
    fb->_params.responseDelayMs = responseDelayMs;
    fb->_params.velocityTolerance = velocityTolerance;
    fb->AXIS_REF.position = position;
    fb->AXIS_REF.velocity = 0.0f;
    fb->AXIS_REF.pressure = 20.0f;
    fb->AXIS_REF.timestamp = 0.0f;
    bind_105(fb);
}

static HYD_MotionSegment make_segment(HYD_REAL target, HYD_MotionDirection dir,
                                      HYD_REAL accMs, HYD_REAL decMs,
                                      HYD_REAL velocityTolerance)
{
    HYD_MotionSegment s;

    memset(&s, 0, sizeof(s));
    s.segmentType = HYD_SEGMENT_TYPE_OTHER;
    s.mode = HYD_MODE_POSITION;
    s.endCondition = HYD_END_POSITION;
    s.planner = HYD_PLANNER_POSITION_BASED;
    s.direction = dir;
    s.targetPosition = target;
    s.maxVelocity = 700.0f;              /* FB_HydAxis rToggleVelCap */
    s.maxAcceleration = accMs;
    s.maxDeceleration = decMs;
    s.positionTolerance = 0.05f;
    s.maxFlow = 90.0f;
    s.velocityToFlowGain = 1.0f;
    s.velocityTolerance = velocityTolerance;
    return s;
}

static HYD_ContinuousAbsoluteContext make_context(HYD_REAL target, HYD_REAL endVelocity,
                                                  HYD_MotionDirection dir, HYD_REAL percent)
{
    HYD_ContinuousAbsoluteContext c;

    memset(&c, 0, sizeof(c));
    c.valid = true;
    c.phase = HYD_CONTABS_PHASE_APPROACH;
    c.targetPosition = target;
    c.sustainVelocity = (dir == HYD_DIRECTION_RETRACT) ? -endVelocity : endVelocity;
    c.adaptEndVelEnabled = true;
    c.approachDirection = dir;
    c.sustainDirection = dir;
    c.flowPercent = percent;
    return c;
}

/* 泵满流量 percent% 时 x 处的动模板速度上限 [mm/s] */
static HYD_REAL ceiling_at(HYD_MotionControlFB *fb, HYD_REAL x, HYD_MotionDirection dir,
                           HYD_REAL percent)
{
    HYD_REAL v = 0.0f, ratio = 0.0f, flow = 0.0f;
    HYD_DiagnosticCode code = HYD_DIAG_CODE_NONE;

    assert(HYD_MotionControlFB_ResolveMaxTemplateVelocity(fb, x, x, dir, &v, &ratio, &flow, &code));
    return v * percent * 0.01f;
}

static void plant_step(HYD_MotionControlFB *fb, PumpPlant *p)
{
    HYD_MotionDirection dir = (p->sign < 0.0f) ? HYD_DIRECTION_RETRACT : HYD_DIRECTION_EXTEND;
    HYD_REAL rpm = fb->PUMP_SPEED;
    HYD_REAL velocity;

    if (p->delayCycles > 0) {
        HYD_REAL delayed = p->history[p->head];   /* delayCycles 个周期前写入 */

        p->history[p->head] = rpm;
        p->head = (p->head + 1) % p->delayCycles;
        rpm = delayed;
    }
    velocity = p->sign * ceiling_at(fb, fb->AXIS_REF.position, dir, 100.0f) *
               rpm / PUMP_MAX_RPM;
    fb->AXIS_REF.velocity = velocity;
    fb->AXIS_REF.position += velocity * DT;
    fb->AXIS_REF.timestamp += DT;
    HYD_MotionControlFB_Cycle(fb);
}

static HYD_BOOL is_flag(const HYD_MotionControlFB *fb, HYD_UINT8 flag)
{
    return (fb->STATE.limitFlags & flag) != 0u;
}

/* T1 + T2(加速) + T3：开模巡航贴着 95% 泵流量 */
static void test_open_cruise_tracks_pump_flow_percent(void)
{
    HYD_MotionControlFB fb;
    HYD_MotionSegment seg = make_segment(300.0f, HYD_DIRECTION_EXTEND, 200.0f, 700.0f, 0.0f);
    HYD_ContinuousAbsoluteContext ctx = make_context(300.0f, 0.0f, HYD_DIRECTION_EXTEND, 95.0f);
    PumpPlant plant;
    HYD_REAL prevPump = 0.0f;
    HYD_REAL maxRise = 0.0f;
    int samples = 0;
    unsigned int cycle;

    memset(&plant, 0, sizeof(plant));
    plant.sign = 1.0f;
    init_axis(&fb, 1.0f, 0.0f, 0.0f);
    assert(HYD_MotionControlFB_StartDirectCommand(&fb, HYD_DIRECT_CMD_MOVE_CONTINUOUS_ABSOLUTE,
                                                  &seg, &ctx, HYD_BUFFER_MODE_ABORT, 0.0f) ==
           HYD_DIRECT_START_STARTED);

    for (cycle = 0u; cycle < 3000u && fb.AXIS_REF.position < 100.0f; ++cycle) {
        plant_step(&fb, &plant);
        assert(fb.FB_STATE != HYD_FB_STATE_FAULT);
        if (fb.PUMP_SPEED < 0.99f * 0.95f * PUMP_MAX_RPM &&
            fb.PUMP_SPEED - prevPump > maxRise) {
            maxRise = fb.PUMP_SPEED - prevPump;
        }
        prevPump = fb.PUMP_SPEED;
        if (fb.AXIS_REF.position >= 20.0f) {
            HYD_REAL vc = ceiling_at(&fb, fb.AXIS_REF.position, HYD_DIRECTION_EXTEND, 95.0f);
            assert(fabsf(fb.PUMP_SPEED - 0.95f * PUMP_MAX_RPM) <= 0.01f * 0.95f * PUMP_MAX_RPM);
            assert(fabsf(fb.STATE.plannedVelocity - vc) <= 0.005f * vc + 0.5f);
            assert(!is_flag(&fb, HYD_LIMIT_FLAG_FLOW));
            assert(!is_flag(&fb, HYD_LIMIT_FLAG_PUMP_SPEED));
            ++samples;
        }
    }
    assert(fb.AXIS_REF.position >= 100.0f);
    assert(samples > 50);
    /* T2：每周期增量 ≤ 1615 rpm / 200 ms × 1 ms × 1.05 = 8.48 rpm */
    assert(maxRise <= 0.95f * PUMP_MAX_RPM / 200.0f * 1.05f);
}

typedef struct {
    HYD_BOOL completed;
    HYD_BOOL inEndVelocity;
    HYD_BOOL sawBrakingDistance;
    HYD_BOOL sawVelocityDeviation;
    int cycles;
    int peakCycle;
    int zeroCycle;
    HYD_REAL peak;
    HYD_REAL finalPosition;
} OpenRun;

/* T4：20ms 纯滞后对象下开模 1→300，零终速停车 */
static OpenRun run_open_zero_end(void)
{
    HYD_MotionControlFB fb;
    HYD_MotionSegment seg = make_segment(300.0f, HYD_DIRECTION_EXTEND, 200.0f, 700.0f, 60.0f);
    HYD_ContinuousAbsoluteContext ctx = make_context(300.0f, 0.0f, HYD_DIRECTION_EXTEND, 95.0f);
    PumpPlant plant;
    OpenRun r;
    int cycle;

    memset(&plant, 0, sizeof(plant));
    memset(&r, 0, sizeof(r));
    r.zeroCycle = -1;
    plant.delayCycles = 20;
    plant.sign = 1.0f;
    init_axis(&fb, 1.0f, 20.0f, 60.0f);
    seg.stableVelocityLimit = 3.0f;
    assert(HYD_MotionControlFB_StartDirectCommand(&fb, HYD_DIRECT_CMD_MOVE_CONTINUOUS_ABSOLUTE,
                                                  &seg, &ctx, HYD_BUFFER_MODE_ABORT, 0.0f) ==
           HYD_DIRECT_START_STARTED);

    for (cycle = 0; cycle < 4000; ++cycle) {
        plant_step(&fb, &plant);
        assert(fb.FB_STATE != HYD_FB_STATE_FAULT);
        if (fb.STATE.plannedVelocity > r.peak) {
            r.peak = fb.STATE.plannedVelocity;
            r.peakCycle = cycle;
        }
        if (r.zeroCycle < 0 && r.peak > 1.0f && fabsf(fb.STATE.plannedVelocity) <= 0.01f) {
            r.zeroCycle = cycle;
        }
        r.sawBrakingDistance |= is_flag(&fb, HYD_LIMIT_FLAG_BRAKING_DISTANCE);
        r.sawVelocityDeviation |= fb.DIAGNOSTIC.velocityDeviation;
        if (fb._directContinuousAbsolute.inEndVelocityLatched) {
            r.inEndVelocity = true;
        }
        if (r.inEndVelocity && !fb.STATE.active) {
            r.completed = true;
            break;
        }
    }
    r.cycles = cycle;
    {
        /* 纯滞后缓冲里还没作用到对象的泵转速，折算成剩余行程计入最终停位 */
        HYD_REAL pendingRpm = 0.0f;
        int k;

        for (k = 0; k < plant.delayCycles; ++k) {
            pendingRpm += plant.history[k];
        }
        r.finalPosition = fb.AXIS_REF.position +
            ceiling_at(&fb, fb.AXIS_REF.position, HYD_DIRECTION_EXTEND, 100.0f) *
            pendingRpm / PUMP_MAX_RPM * DT;
    }
    return r;
}

static void test_open_zero_end_stop_uses_back_calculated_velocity(void)
{
    OpenRun r = run_open_zero_end();
    int brakeMs = r.zeroCycle - r.peakCycle;

    printf("T4 open: %d ms, final %.3f, peak %.1f, brake %d ms\n",
           r.cycles, r.finalPosition, r.peak, brakeMs);
    assert(r.completed);
    assert(r.inEndVelocity);
    assert(fabsf(r.finalPosition - 300.0f) <= 1.5f);
    /* 回算后约 1160ms；不回算时提前刹车约 1700ms */
    assert(r.cycles < 1300);
    assert(brakeMs >= 600 && brakeMs <= 800);
    assert(!r.sawBrakingDistance);
    assert(!r.sawVelocityDeviation);
}

/* T5 + T2(降百分比)：合模两段链式 */
static void test_close_chained_segments_ramp_flow_down(void)
{
    HYD_MotionControlFB fb;
    HYD_MotionSegment seg1 = make_segment(40.0f, HYD_DIRECTION_RETRACT, 300.0f, 200.0f, 60.0f);
    HYD_MotionSegment seg2 = make_segment(10.0f, HYD_DIRECTION_RETRACT, 200.0f, 150.0f, 60.0f);
    HYD_ContinuousAbsoluteContext ctx1;
    HYD_ContinuousAbsoluteContext ctx2;
    PumpPlant plant;
    HYD_REAL v40;
    HYD_REAL v10;
    HYD_REAL pumpAtSwitch = 0.0f;
    HYD_REAL prevPump;
    HYD_REAL maxDrop = 0.0f;
    HYD_BOOL sawBraking = false;
    HYD_BOOL sawDeviation = false;
    unsigned int cycle;

    memset(&plant, 0, sizeof(plant));
    plant.delayCycles = 20;
    plant.sign = -1.0f;
    init_axis(&fb, 299.36f, 20.0f, 60.0f);
    v40 = ceiling_at(&fb, 40.0f, HYD_DIRECTION_RETRACT, 95.0f);
    v10 = ceiling_at(&fb, 10.0f, HYD_DIRECTION_RETRACT, 45.0f);
    ctx1 = make_context(40.0f, v40, HYD_DIRECTION_RETRACT, 95.0f);
    ctx2 = make_context(10.0f, v10, HYD_DIRECTION_RETRACT, 45.0f);

    assert(HYD_MotionControlFB_StartDirectCommand(&fb, HYD_DIRECT_CMD_MOVE_CONTINUOUS_ABSOLUTE,
                                                  &seg1, &ctx1, HYD_BUFFER_MODE_ABORT, 0.0f) ==
           HYD_DIRECT_START_STARTED);
    for (cycle = 0u; cycle < 3000u &&
         fb._directContinuousAbsolute.phase == HYD_CONTABS_PHASE_APPROACH; ++cycle) {
        plant_step(&fb, &plant);
        assert(fb.FB_STATE != HYD_FB_STATE_FAULT);
        sawBraking |= is_flag(&fb, HYD_LIMIT_FLAG_BRAKING_DISTANCE);
        sawDeviation |= fb.DIAGNOSTIC.velocityDeviation;
    }
    assert(fb._directContinuousAbsolute.phase == HYD_CONTABS_PHASE_SUSTAIN);

    pumpAtSwitch = fb.PUMP_SPEED;
    assert(HYD_MotionControlFB_StartDirectCommand(&fb, HYD_DIRECT_CMD_MOVE_CONTINUOUS_ABSOLUTE,
                                                  &seg2, &ctx2, HYD_BUFFER_MODE_ABORT,
                                                  fb.AXIS_REF.timestamp) ==
           HYD_DIRECT_START_STARTED);
    prevPump = pumpAtSwitch;
    for (cycle = 0u; cycle < 3000u && fb.AXIS_REF.position > 10.0f; ++cycle) {
        plant_step(&fb, &plant);
        assert(fb.FB_STATE != HYD_FB_STATE_FAULT);
        if (prevPump - fb.PUMP_SPEED > maxDrop) {
            maxDrop = prevPump - fb.PUMP_SPEED;
        }
        prevPump = fb.PUMP_SPEED;
        sawBraking |= is_flag(&fb, HYD_LIMIT_FLAG_BRAKING_DISTANCE);
        sawDeviation |= fb.DIAGNOSTIC.velocityDeviation;
    }

    printf("T5 close: switch pump %.1f, max drop %.3f rpm/cycle, v@10 %.1f (target %.1f)\n",
           pumpAtSwitch, maxDrop, -fb.AXIS_REF.velocity, v10);
    assert(fb.AXIS_REF.position <= 10.0f);
    /* 固定耗时斜坡：(n_switch − 45%×1700) / 150 ms × 1 ms × 1.05 */
    assert(maxDrop <= (pumpAtSwitch - 0.45f * PUMP_MAX_RPM) / 150.0f * 1.05f + 0.01f);
    assert(fabsf(-fb.AXIS_REF.velocity - v10) <= 0.10f * v10);
    assert(!sawBraking);
    assert(!sawDeviation);
}

/* T6（核心层部分）：flowPercent = 0 时与现有段标尺语义一致 */
static void test_zero_percent_keeps_legacy_velocity_semantics(void)
{
    HYD_MotionControlFB fb;
    HYD_MotionSegment seg = make_segment(300.0f, HYD_DIRECTION_EXTEND, 200.0f, 700.0f, 0.0f);
    HYD_ContinuousAbsoluteContext ctx = make_context(300.0f, 0.0f, HYD_DIRECTION_EXTEND, 0.0f);
    PumpPlant plant;
    unsigned int cycle;

    memset(&plant, 0, sizeof(plant));
    plant.sign = 1.0f;
    seg.maxVelocity = 200.0f;
    init_axis(&fb, 1.0f, 0.0f, 0.0f);
    assert(HYD_MotionControlFB_StartDirectCommand(&fb, HYD_DIRECT_CMD_MOVE_CONTINUOUS_ABSOLUTE,
                                                  &seg, &ctx, HYD_BUFFER_MODE_ABORT, 0.0f) ==
           HYD_DIRECT_START_STARTED);
    for (cycle = 0u; cycle < 3000u && fb.AXIS_REF.position < 150.0f; ++cycle) {
        plant_step(&fb, &plant);
        assert(fb.FB_STATE != HYD_FB_STATE_FAULT);
    }
    /* 150mm 处速比 < 1，200 mm/s 远低于泵上限：按段速度巡航，不受泵流量百分比影响 */
    assert(fabsf(fb.STATE.plannedVelocity - 200.0f) <= 0.01f);
    assert(fb._directContinuousAbsolute.flowRampRate == 0.0f);
}

int main(void)
{
    /* T4 放第一个：变异检查（去掉回算）时先在 T4 失败 */
    test_open_zero_end_stop_uses_back_calculated_velocity();
    test_open_cruise_tracks_pump_flow_percent();
    test_close_chained_segments_ramp_flow_down();
    test_zero_percent_keeps_legacy_velocity_semantics();
    printf("test_toggle_flow_percent_integration passed\n");
    return 0;
}
```

说明（写给实现者）：
- T4 的 `velocityTolerance = 60`：探针中回算后最大跟踪误差 38mm/s（20ms 纯滞后下加速阶段的固有滞后），60 留出余量。它能抓到“规划速度 700、实际 ≤ 600”这类不回算的误报。T5 同理（探针 30mm/s）。
- T4 判据来自探针（同一对象、同一斜坡公式）：回算时总时长 1164ms、停位 300.22mm、峰值到零 665ms；不回算时 1708ms。
- T5 判据来自探针：换段处泵转速 1606rpm，第 2 段每周期最大降幅 5.64rpm，上限 (1606−765)/150×1.05 = 5.89rpm；到 10mm 时 45.3mm/s，目标 47.7。

- [ ] **Step 2: 注册测试，确认编译失败**

`CMakeLists.txt` 第 221 行后追加：

```cmake
add_executable(test_toggle_flow_percent_integration tests/test_toggle_flow_percent_integration.c)
target_link_libraries(test_toggle_flow_percent_integration PRIVATE HydroMotionLib m)
```

第 326 行 `add_test(NAME test_toggle_motion_integration ...)` 后追加：

```cmake
add_test(NAME test_toggle_flow_percent_integration COMMAND test_toggle_flow_percent_integration)
```

Run: `cmake --preset unixgcc && cmake --build out/build/unixgcc --target test_toggle_flow_percent_integration 2>&1 | grep -E "error|Error" | head`
Expected: 编译失败，`'HYD_ContinuousAbsoluteContext' has no member named 'flowPercent'`。

- [ ] **Step 3: 上下文增加字段**

`include/motion_control.h`，在 `HYD_BOOL inEndVelocityLatched;` 后、`} HYD_ContinuousAbsoluteContext;` 前加：

```c
    /* 泵流量百分比模式（spec 2026-11-18）：0 = 关闭；(0,100] = 泵流量上限的百分比。 */
    HYD_REAL flowPercent;
    /* 流量斜坡速率 [L/min/s]，段启动时由 HYD_AnchorContinuousAbsoluteFlowRamp 锚定。 */
    HYD_REAL flowRampRate;
```

Run: `cmake --build out/build/unixgcc --target test_toggle_flow_percent_integration 2>&1 | grep -E "error" | head; ./out/build/unixgcc/test_toggle_flow_percent_integration`
Expected: 编译通过，运行时第一个测试（T4）在 `assert(r.cycles < 1300)` 失败，打印 `T4 open: ~1606 ms`。原因是字段还没被使用，走旧路径，规划速度冲到 700mm/s 后被泵硬件上限截住。（干跑实测）

- [ ] **Step 4: 斜坡锚定 helper**

`src/motion_control.c`，在 `HYD_ResolveContinuousAbsoluteCrossingVelocity` 函数结束（约 1153 行的 `}`）之后插入：

```c
/* 泵流量百分比模式的基准流量：泵转速上限对应的流量；未配置泵时取 maxFlow。 */
static HYD_REAL HYD_ResolveFlowPercentBase(const HYD_MotionControlFB* fb) {
    HYD_REAL base = HYD_GetPumpFlowLimit(fb);

    if (!(base > 0.0f)) {
        base = fb->_params.maxFlow;
    }
    return (base > 0.0f) ? base : 0.0f;
}

/* 段启动时锚定流量斜坡（固定耗时语义，2026-11-10）：从上一拍指令流量出发，
 * 上升用 t_acc、下降用 t_dec 到达目标百分比。须在续接恢复规划状态之后调用。 */
static void HYD_AnchorContinuousAbsoluteFlowRamp(HYD_MotionControlFB* fb,
                                                 HYD_ContinuousAbsoluteContext* ctx,
                                                 const HYD_MotionSegment* segment) {
    HYD_REAL base;
    HYD_REAL target;
    HYD_REAL start;
    HYD_REAL seconds;

    if (fb == NULL || ctx == NULL || segment == NULL) {
        return;
    }
    ctx->flowRampRate = 0.0f;
    if (!(ctx->flowPercent > 0.0f)) {
        return;
    }
    base = HYD_ResolveFlowPercentBase(fb);
    target = ctx->flowPercent * 0.01f * base;
    start = fabs(fb->_plannerState.lastTargetFlow);
    seconds = HYD_MotionTimeMsToSeconds((target < start)
                                            ? segment->maxDeceleration
                                            : segment->maxAcceleration);
    ctx->flowRampRate = (fabs(target - start) > 0.0f)
        ? fabs(target - start) / seconds
        : base / HYD_MotionTimeMsToSeconds(segment->maxAcceleration);
}
```

- [ ] **Step 5: 三处锚定调用**

在三处 `crossingVelocity = HYD_ResolveContinuousAbsoluteCrossingVelocity(...)` 赋值语句之后、同一个 `if` 块内各加一行。

`HYD_StartPendingDirectSlot`（约 2007 行）：

```c
        fb->_directContinuousAbsolute.crossingVelocity =
            HYD_ResolveContinuousAbsoluteCrossingVelocity(fb,
                                                          &fb->_directContinuousAbsolute,
                                                          &fb->_activeSegment);
        HYD_AnchorContinuousAbsoluteFlowRamp(fb, &fb->_directContinuousAbsolute,
                                             &fb->_activeSegment);
```

`HYD_MotionControlFB_StartDirectCommand` 的 abort 分支（约 4305 行）和直接启动分支（约 4364 行）做同样的追加，代码一致。

- [ ] **Step 6: 每周期流量上限、回算、FLOW 标志**

先在 `HYD_ExecuteActiveSegmentControl` 中 `HYD_REAL plannedMagnitudeBefore = fabs(fb->_plannerState.lastTargetVelocity);`（约 2536 行，`HYD_MotionPlanner_Execute` 调用之前）后面加一行：

```c
        /* 规划器执行时会用不含速比的油缸增益改写 lastTargetFlow，须在执行前取上一拍映射流量 */
        HYD_REAL mappedFlowBefore = fabs(fb->_plannerState.lastTargetFlow);
```

再改同一函数的流量上限块（约 2595-2622 行）。把原块：

```c
        {
            HYD_REAL velocityFlowLimit = segment->maxFlow;
            HYD_REAL pumpFlowLimit = HYD_GetPumpFlowLimit(fb);
            if (segment->targetFlow > 0.0f &&
                segment->targetFlow < velocityFlowLimit) {
                velocityFlowLimit = segment->targetFlow;
            }
            if (pumpFlowLimit > 0.0f &&
                (velocityFlowLimit <= 0.0f || pumpFlowLimit < velocityFlowLimit)) {
                velocityFlowLimit = pumpFlowLimit;
            }
            if (HYD_MotionControlFB_MapTemplateVelocity(
                    fb, segment, plannerOutput->targetVelocity,
                    velocityFlowLimit, &mapped, &mappingCode)) {
                plannerOutput->targetFlow = mapped.requestedFlow;
                fb->_plannerState.lastTargetFlow = mapped.requestedFlow;
#if HYD_ENABLE_FLOW_DIAGNOSTIC_TELEMETRY
                fb->STATE.requestedFlow = mapped.unlimitedRequestedFlow;
                fb->STATE.effectiveCylinderGain = mapped.effectiveCylinderGain;
                fb->STATE.maxTemplateVelocity = mapped.maxTemplateVelocity;
#endif
                if (mapped.flowLimitActive) {
                    fb->STATE.limitFlags |= HYD_LIMIT_FLAG_FLOW;
                }
                if ((pumpFlowLimit > 0.0f) &&
                    (mapped.unlimitedRequestedFlow > pumpFlowLimit)) {
                    fb->STATE.limitFlags |= HYD_LIMIT_FLAG_PUMP_SPEED;
                }
```

改为：

```c
        {
            HYD_REAL velocityFlowLimit = segment->maxFlow;
            HYD_REAL pumpFlowLimit = HYD_GetPumpFlowLimit(fb);
            HYD_REAL hardwareFlowLimit;
            HYD_BOOL flowPercentActive =
                fb->_directOwnerKind == HYD_DIRECT_CMD_MOVE_CONTINUOUS_ABSOLUTE &&
                fb->_directContinuousAbsolute.valid &&
                fb->_directContinuousAbsolute.flowPercent > 0.0f;
            HYD_REAL percentFlowCap = 0.0f;
            if (segment->targetFlow > 0.0f &&
                segment->targetFlow < velocityFlowLimit) {
                velocityFlowLimit = segment->targetFlow;
            }
            if (pumpFlowLimit > 0.0f &&
                (velocityFlowLimit <= 0.0f || pumpFlowLimit < velocityFlowLimit)) {
                velocityFlowLimit = pumpFlowLimit;
            }
            hardwareFlowLimit = velocityFlowLimit;
            if (flowPercentActive) {
                /* spec §4.3.3：q_cap = clamp(q_target, q_prev ∓ rate·dt)。 */
                HYD_REAL qTarget = fb->_directContinuousAbsolute.flowPercent * 0.01f *
                                   HYD_ResolveFlowPercentBase(fb);
                HYD_REAL qPrev = mappedFlowBefore;
                HYD_REAL step = fb->_directContinuousAbsolute.flowRampRate *
                                plannerInput.deltaTime;

                percentFlowCap = HYD_ClampReal(qTarget, qPrev - step, qPrev + step);
                if (!(percentFlowCap > 0.0f)) {
                    /* 映射层把 maxFlow = 0 当作不限流 */
                    percentFlowCap = HYD_REAL_EPSILON;
                }
                if (velocityFlowLimit <= 0.0f || percentFlowCap < velocityFlowLimit) {
                    velocityFlowLimit = percentFlowCap;
                }
            }
            if (HYD_MotionControlFB_MapTemplateVelocity(
                    fb, segment, plannerOutput->targetVelocity,
                    velocityFlowLimit, &mapped, &mappingCode)) {
                if (flowPercentActive && mapped.flowLimitActive) {
                    /* spec §4.3.4：规划速度回算为本周期可实现速度，刹车距离按真实速度计算 */
                    HYD_REAL sign = (plannerOutput->targetVelocity < 0.0f) ? -1.0f : 1.0f;
                    plannerOutput->targetVelocity = sign * mapped.maxTemplateVelocity;
                    fb->_plannerState.lastTargetVelocity = plannerOutput->targetVelocity;
                }
                plannerOutput->targetFlow = mapped.requestedFlow;
                fb->_plannerState.lastTargetFlow = mapped.requestedFlow;
#if HYD_ENABLE_FLOW_DIAGNOSTIC_TELEMETRY
                fb->STATE.requestedFlow = mapped.unlimitedRequestedFlow;
                fb->STATE.effectiveCylinderGain = mapped.effectiveCylinderGain;
                fb->STATE.maxTemplateVelocity = mapped.maxTemplateVelocity;
#endif
                if (mapped.flowLimitActive &&
                    (!flowPercentActive ||
                     (hardwareFlowLimit > 0.0f && hardwareFlowLimit < percentFlowCap))) {
                    /* spec §4.3.5：百分比上限生效是正常巡航，只有硬件上限更小时才报 FLOW */
                    fb->STATE.limitFlags |= HYD_LIMIT_FLAG_FLOW;
                }
                if (!flowPercentActive &&
                    (pumpFlowLimit > 0.0f) &&
                    (mapped.unlimitedRequestedFlow > pumpFlowLimit)) {
                    /* 百分比模式下请求流量已被 q_cap（≤ 泵上限）截住，
                     * 泵转速限幅只由 pump_converter 判定（下方 speedLimitActive） */
                    fb->STATE.limitFlags |= HYD_LIMIT_FLAG_PUMP_SPEED;
                }
```

后面的 `else` 故障分支保持原样。

- [ ] **Step 7: 运行新测试**

Run: `cmake --build --preset unixgcc 2>&1 | grep -E "error|warning: " | head; ./out/build/unixgcc/test_toggle_flow_percent_integration`
Expected（干跑实测）：
```
T4 open: 1162 ms, final 300.217, peak 541.5, brake 665 ms
T5 close: switch pump 1611.5, max drop 5.667 rpm/cycle, v@10 45.3 (target 47.7)
test_toggle_flow_percent_integration passed
```

若 T4 的 `brakeMs` 或 T5 的 `maxDrop` 超界，先打印实际值，与“说明”里的探针值对比。差距在 5% 内说明是离散化差异，回到 spec §6 调整判据；差距更大说明实现有误。

- [ ] **Step 8: 变异检查——去掉回算后 T4 必须失败**

临时把 Step 6 的回算块条件改成 `if (false && flowPercentActive && mapped.flowLimitActive)`，重新构建运行：

Run: `cmake --build out/build/unixgcc --target test_toggle_flow_percent_integration 2>&1 | grep error; ./out/build/unixgcc/test_toggle_flow_percent_integration; echo "exit=$?"`
Expected（干跑实测）：`exit` 非 0，T4 在 `assert(r.cycles < 1300)` 失败，打印 `T4 open: 1706 ms ... peak 663.0, brake 1502 ms`：规划器按虚高速度提前刹车，同时速度偏差诊断置位。把这组数记到提交信息里，然后**恢复**原条件，重新构建，确认通过。

- [ ] **Step 9: 尺寸预算与全量回归**

`tests/test_mode_switch_bumpless.c` 第 446-448 行改为：

```c
    /* +8: _params.responseDelayMs / zeroEndSettleVelocity (zero-end stop).
     * +16: 两份 HYD_ContinuousAbsoluteContext 各加 flowPercent / flowRampRate。 */
    assert(sizeof(HYD_MotionControlFB) <= 3328U);
```

Run: `cmake --build --preset unixgcc 2>&1 | grep -E "error" | head; ctest --test-dir out/build/unixgcc --output-on-failure 2>&1 | tail -5`
Expected: `100% tests passed, 0 tests failed out of 62`。

- [ ] **Step 10: 提交**

```bash
git add include/motion_control.h src/motion_control.c tests/test_toggle_flow_percent_integration.c CMakeLists.txt tests/test_mode_switch_bumpless.c
git commit -m "$(cat <<'EOF'
MoveContinuousAbsolute 增加泵流量百分比巡航：流量斜坡、每周期上限与规划速度回算

- 上下文增加 flowPercent / flowRampRate，段启动时按固定耗时语义锚定斜坡
- 百分比上限经机构映射生效，限流时把规划器状态回算为可实现速度
- 百分比上限属正常巡航，不置 FLOW；仅硬件上限更小时才置
- 新增 test_toggle_flow_percent_integration（T1~T5）；变异检查：去掉回算后 T4 失败（开模 1706ms，回算后 1162ms）

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>
EOF
)"
```

---

### Task 2: IEC 接口——PUMPFLOWPERCENT 输入、校验与布局一致性

**Files:**
- Modify: `include/motion_interface.h:257-287`
- Modify: `src/motion_interface.c:254-281`（`buildContinuousAbsoluteContext`）、`src/motion_interface.c:2264-2344`（`__mcl_cmd_MoveContinuousAbsolute` 上升沿）
- Modify: `pousHydMotion.xml:618`
- Modify: `tests/test_movecontinuousabsolute_integration.c`（新增 T6 测试并加入 `main`）
- Modify: `tests/test_interface_layout_consistency.py`（`required_pous` 检查之后）

**Interfaces:**
- Consumes：Task 1 的 `HYD_ContinuousAbsoluteContext.flowPercent`
- Produces：IEC 字段 `HYD_MOVECONTINUOUSABSOLUTE.PUMPFLOWPERCENT : REAL`（位于 `PRESSURELIMIT` 之后、`INENDVELOCITY` 之前）；`buildContinuousAbsoluteContext(..., HYD_REAL flowPercent)`

- [ ] **Step 1: 写失败测试（T6）**

`tests/test_movecontinuousabsolute_integration.c`，在 `test_stop_takes_over_and_sets_commandaborted` 定义之前插入：

```c
/* spec 2026-11-18 T6：PUMPFLOWPERCENT 校验，合法值写入上下文 */
static void test_pump_flow_percent_input_is_validated(void) {
    const HYD_REAL invalid[3] = { -1.0f, 101.0f, NAN };
    HYD_MOVECONTINUOUSABSOLUTE cmd;
    HYD_MotionControlFB* core;
    int axisId;

    for (int i = 0; i < 3; i++) {
        __HydMotion_framework_Init();
        axisId = create_manual_axis();
        ASSERT_TRUE(axisId >= 0, "CreateMotion should allocate a manual axis");
        if (axisId < 0) {
            return;
        }
        ASSERT_TRUE(set_axis_feedback(axisId, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f),
                    "Manual feedback should initialize the axis");
        init_movecontinuousabsolute(&cmd, axisId, 10.0f, 10.0f, 0.0f,
                                    HYD_DIRECTION_POSITIVE, HYD_DIRECTION_POSITIVE);
        IEC_VAL(cmd.PUMPFLOWPERCENT) = invalid[i];
        rising_edge_scan(&cmd);
        ASSERT_TRUE(IEC_VAL(cmd.ERROR), "Out-of-range PUMPFLOWPERCENT must be rejected");
        ASSERT_TRUE(IEC_VAL(cmd.ERRORID) == (IEC_WORD)HYD_DIAG_CODE_COMMAND_NOT_ALLOWED,
                    "Rejected PUMPFLOWPERCENT should report COMMAND_NOT_ALLOWED");
    }

    __HydMotion_framework_Init();
    axisId = create_manual_axis();
    core = __MK_GetPublic_MotionControlFB(axisId);
    ASSERT_TRUE(core != NULL, "Public FB should be available");
    if (core == NULL) {
        return;
    }
    ASSERT_TRUE(set_axis_feedback(axisId, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f),
                "Manual feedback should initialize the axis");
    init_movecontinuousabsolute(&cmd, axisId, 10.0f, 10.0f, 0.0f,
                                HYD_DIRECTION_POSITIVE, HYD_DIRECTION_POSITIVE);
    IEC_VAL(cmd.PUMPFLOWPERCENT) = 95.0f;
    rising_edge_scan(&cmd);
    ASSERT_TRUE(!IEC_VAL(cmd.ERROR), "PUMPFLOWPERCENT = 95 should be accepted");
    ASSERT_TRUE(fabsf(core->_directContinuousAbsolute.flowPercent - 95.0f) < 1e-6f,
                "Accepted PUMPFLOWPERCENT should be latched in the context");
    ASSERT_TRUE(core->_directContinuousAbsolute.flowRampRate > 0.0f,
                "Accepted PUMPFLOWPERCENT should anchor a flow ramp");
}
```

在 `main` 中 `test_stop_takes_over_and_sets_commandaborted();` 之后加：

```c
    test_pump_flow_percent_input_is_validated();
```

`tests/test_interface_layout_consistency.py`，在 `required_pous` 循环结束之后（`debug_pou = next(` 之前）加：

```python
    contabs_pou = next(
        (pou for pou in ET.parse(XML).getroot().iter()
         if local_name(pou.tag) == "pou" and
         pou.attrib.get("name") == "HYD_MoveContinuousAbsolute"),
        None,
    )
    if contabs_pou is None:
        print("expected XML POU HYD_MoveContinuousAbsolute")
        return 1
    contabs_xml = [
        variable.attrib.get("name").upper()
        for variable in contabs_pou.iter()
        if local_name(variable.tag) == "variable"
    ]
    header_start = c_header.find("// FUNCTION_BLOCK HYD_MoveContinuousAbsolute")
    header_end = c_header.find("} HYD_MOVECONTINUOUSABSOLUTE;", header_start)
    if header_start < 0 or header_end < 0:
        print("expected C layout for HYD_MOVECONTINUOUSABSOLUTE")
        return 1
    contabs_c = [
        name for name in re.findall(
            r"__DECLARE_VAR\(\s*\w+\s*,\s*(\w+)\s*\)",
            c_header[header_start:header_end])
        if name not in ("EN", "ENO")
    ]
    if contabs_xml != contabs_c:
        print(f"HYD_MoveContinuousAbsolute XML {contabs_xml} != C {contabs_c}")
        return 1
    if ("PUMPFLOWPERCENT" not in contabs_xml or
            contabs_xml.index("PUMPFLOWPERCENT") != contabs_xml.index("PRESSURELIMIT") + 1):
        print("expected PUMPFLOWPERCENT right after PRESSURELIMIT")
        return 1
```

文件头的 import 增加 `import re`。

- [ ] **Step 2: 确认失败**

Run: `cmake --build --preset unixgcc 2>&1 | grep -E "error" | head -3; python3 tests/test_interface_layout_consistency.py`
Expected: C 编译失败 `has no member named 'PUMPFLOWPERCENT'`；Python 打印 `expected PUMPFLOWPERCENT right after PRESSURELIMIT`，exit 1。

- [ ] **Step 3: C 头文件与 XML**

`include/motion_interface.h`，在 `__DECLARE_VAR(REAL,PRESSURELIMIT)`（HYD_MoveContinuousAbsolute 内，约 270 行）之后加：

```c
  /* 泵流量百分比（%）：0 = 关闭；(0,100] = 巡航按泵流量上限的百分比，VELOCITY 仅作安全上限。 */
  __DECLARE_VAR(REAL,PUMPFLOWPERCENT)
```

`pousHydMotion.xml` 第 618 行 `<variable name="PRESSURELIMIT"><type><REAL /></type></variable>` 之后（HYD_MoveContinuousAbsolute 的 inputVars 内）加：

```xml
						<variable name="PUMPFLOWPERCENT"><type><REAL /></type></variable>
```

- [ ] **Step 4: 适配层校验与上下文**

`src/motion_interface.c` 的 `buildContinuousAbsoluteContext`：形参列表末尾加 `HYD_REAL flowPercent`，函数体 `ctx.sustainDirection = normalizedSustainDirection;` 之后加 `ctx.flowPercent = flowPercent;`：

```c
static HYD_ContinuousAbsoluteContext buildContinuousAbsoluteContext(
    HYD_REAL targetPosition,
    HYD_REAL endVelocity,
    HYD_REAL pressureLimit,
    HYD_BOOL adaptEndVelEnabled,
    HYD_MotionDirection approachDirection,
    HYD_MotionDirection sustainDirection,
    HYD_REAL flowPercent)
```

`__mcl_cmd_MoveContinuousAbsolute` 上升沿分支：
- 局部变量区（`HYD_REAL pressureLimit;` 后）加 `HYD_REAL flowPercent;`；
- `pressureLimit` 读取之后加 `flowPercent = __GET_VAR(data__->PUMPFLOWPERCENT);`；
- 校验条件 `!isfinite(pressureLimit)` 之后追加：

```c
            !isfinite(pressureLimit) ||
            !isfinite(flowPercent) ||
            flowPercent < 0.0f ||
            flowPercent > 100.0f) {
```

- `buildContinuousAbsoluteContext(...)` 调用末尾加实参 `flowPercent`：

```c
        context = buildContinuousAbsoluteContext(
            __GET_VAR(data__->POSITION),
            requestedEndVelocity,
            pressureLimit,
            __GET_VAR(data__->ADAPTENDVELTOAVOIDOVERSHOOT),
            approachDirection,
            sustainDirection,
            flowPercent);
```

- [ ] **Step 5: 运行**

Run: `cmake --build --preset unixgcc 2>&1 | grep -E "error|warning: " | head; ./out/build/unixgcc/test_movecontinuousabsolute_integration | tail -3; python3 tests/test_interface_layout_consistency.py`
Expected: `=== Results: N/N passed ===`（无 FAIL 行）；`interface layout consistency tests passed`。

- [ ] **Step 6: 全量回归**

Run: `ctest --test-dir out/build/unixgcc --output-on-failure 2>&1 | tail -5`
Expected: `100% tests passed, 0 tests failed out of 62`。

- [ ] **Step 7: 提交**

```bash
git add include/motion_interface.h src/motion_interface.c pousHydMotion.xml tests/test_movecontinuousabsolute_integration.c tests/test_interface_layout_consistency.py
git commit -m "$(cat <<'EOF'
HYD_MoveContinuousAbsolute 增加 PUMPFLOWPERCENT 输入并校验

- 追加在 PRESSURELIMIT 之后；非有限、<0、>100 上升沿报 COMMAND_NOT_ALLOWED
- 布局一致性测试增加该 FB 的 XML/C 字段顺序核对
- 接口布局改变：PLC 工程需用新的 pousHydMotion.xml 重新生成并编译

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>
EOF
)"
```

---

### Task 3: FB_HydAxis——新旧模式开关、目标点标尺、PUMPFLOWPERCENT

**Files:**
- Modify: `iec/FB_HydAxis.st:75-91`（VAR_INPUT）、`:102-148`（VAR）、`:323-348`（标尺与速度换算）、`:503-516`（fbMoveContAbs 调用）
- Modify: `tests/test_hydaxis_p0_contract.py`

**Interfaces:**
- Consumes：Task 2 的 IEC 输入 `PUMPFLOWPERCENT`；现有 `HYD_ReadMaxTemplateVelocity`（`XFROM = XTO` 时返回该点速度，已核实 `range_max_abs_ratio` 支持零宽区间）
- Produces：`FB_HydAxis.bFlowPctMode : BOOL := TRUE`（VAR_INPUT）

- [ ] **Step 1: 写失败的契约测试**

`tests/test_hydaxis_p0_contract.py`，在 `test_emergency_stop_does_not_use_legacy_large_time` 之后加：

```python
    def test_toggle_flow_percent_mode_contract(self):
        # spec 2026-11-18 §4.4
        self.assertIn("bFlowPctMode : BOOL := TRUE", SOURCE)
        self.assertIn("bFlowPctSeg : BOOL", SOURCE)
        self.assertIn("rToggleVelCap : REAL := 700.0", SOURCE)
        self.assertIn("rPumpFlowPct := LIMIT(0.0, UINT_TO_REAL(uiSpd)*0.1, 100.0)", SOURCE)
        self.assertIn("PUMPFLOWPERCENT := rPumpFlowPct", SOURCE)
        # 目标点标尺：XFROM = XTO = rPos
        self.assertIn("XFROM := SEL(bFlowPctSeg, stHydAxisRef.rActPosition, rPos)", SOURCE)
        # PressureHandle 仍用带 5% 下限的 rFlowPercent
        self.assertIn("FLOWLIMITPERCENT:=rFlowPercent", SOURCE)
        self.assertIn("rFlowPercent := LIMIT(5, rFlowPercent, 100)", SOURCE)
```

Run: `python3 tests/test_hydaxis_p0_contract.py`
Expected: FAIL，`'bFlowPctMode : BOOL := TRUE' not found`。

- [ ] **Step 2: 声明**

`VAR_INPUT` 区 `uiPressureAccTime : UINT;` 之后加：

```
    bFlowPctMode : BOOL := TRUE;   // 曲肘轴：TRUE = 速度百分比按泵流量；FALSE = 旧段标尺（A/B 对比与回退）
```

`VAR` 区 `bScaleOk : BOOL := TRUE;` 之后加：

```
    bFlowPctSeg : BOOL;                       // 段启动沿锁存的本段模式
    rPumpFlowPct : REAL;                      // 传给 PUMPFLOWPERCENT 的泵流量百分比
    rToggleVelCap : REAL := 700.0;            // mm/s：百分比模式下动模板速度安全上限（> 泵满流量最小速比处开模 687mm/s）
```

- [ ] **Step 3: 标尺与速度换算**

把第 323-348 行替换为：

```
  // 曲肘轴：段启动沿锁存本段模式与速度标尺。
  //  百分比模式（MoveContinuousAbsolute）：标尺取目标点泵满流量速度，rEndVel = 目标点处 uiEndSpd% 流量的速度；
  //  旧模式：标尺取本段区间最坏速比处泵满流量速度（MoveVelocity 始终用旧模式）。
  IF rTrigStart.Q THEN
      bScaleOk := TRUE;
      bFlowPctSeg := bFlowPctMode AND iMechBaseClass = 1 AND uiMode = 1;
      IF iMechBaseClass = 1 AND (uiMode = 1 OR uiMode = 2) THEN
          fbMaxVel(AXISID := siAxisId, ENABLE := TRUE,
                   XFROM := SEL(bFlowPctSeg, stHydAxisRef.rActPosition, rPos),
                   XTO := SEL(uiMode = 1, stHydAxisRef.rActPosition, rPos),
                   DIRECTION := iDir);
          bScaleOk := fbMaxVel.VALID AND NOT fbMaxVel.ERROR;
          IF bScaleOk THEN
              rMaxSpeedSeg := fbMaxVel.MAXVELOCITY;
          END_IF;
      END_IF;
  END_IF;

  IF iMechBaseClass = 1 THEN
      rMaxSpeed := rMaxSpeedSeg;
  ELSE
      rMaxSpeed := stHydAxisRef.rMaxSpeed;   // 直压轴保持原标尺
  END_IF;

  rPres := MAX(UINT_TO_REAL(uiPres), 0.0)*0.1;   //压力设定 bar
  IF bFlowPctSeg THEN
      rPumpFlowPct := LIMIT(0.0, UINT_TO_REAL(uiSpd)*0.1, 100.0);
      rVel := SEL(rPumpFlowPct > 0.0, 0.0, rToggleVelCap);   // uiSpd = 0 保持库端拒绝零速
  ELSE
      rPumpFlowPct := 0.0;
      rVel  := UINT_TO_REAL(uiSpd)*0.1*0.01*rMaxSpeed;  //速度设定
  END_IF;
  rEndVel := UINT_TO_REAL(uiEndSpd)*0.1*0.01*rMaxSpeed;  //终点速度
  rFlowPercent := UINT_TO_REAL(uiSpd)*0.1;
  rFlowPercent := LIMIT(5, rFlowPercent, 100);
```

说明：`SEL(G, IN0, IN1)` 在 G = FALSE 时取 IN0。百分比模式的 XFROM、XTO 都是 `rPos`（XTO 的 `SEL(uiMode = 1, ...)` 在 uiMode = 1 时取 IN1 = rPos），即目标点标尺；旧模式 XFROM 为当前位置，与现有行为一致。

- [ ] **Step 4: 传入 PUMPFLOWPERCENT**

`fbMoveContAbs(...)` 调用里 `PRESSURELIMIT := rPres` 改为：

```
      PRESSURELIMIT := rPres,
      PUMPFLOWPERCENT := rPumpFlowPct
```

- [ ] **Step 5: 运行契约测试与全量回归**

Run: `python3 tests/test_hydaxis_p0_contract.py && ctest --test-dir out/build/unixgcc --output-on-failure 2>&1 | tail -3`
Expected: `OK`；`100% tests passed, 0 tests failed out of 62`。

- [ ] **Step 6: 人工评审 ST（仓库内无 ST 编译器）**

逐条确认：
1. `bFlowPctSeg` 只在 `rTrigStart.Q` 时更新，段中途改 `bFlowPctMode` 不影响当前段；
2. `uiMode = 2`（MoveVelocity）与直压轴都走 `ELSE` 分支，`rPumpFlowPct = 0`；
3. 标尺查询失败时 `bScaleOk = FALSE`，由第 412/436 行现有逻辑报警进 900；
4. `PressureHandle` 仍用 `rFlowPercent`（5% 下限）。

- [ ] **Step 7: 提交**

```bash
git add iec/FB_HydAxis.st tests/test_hydaxis_p0_contract.py
git commit -m "$(cat <<'EOF'
FB_HydAxis 曲肘轴开合模增加泵流量百分比模式开关

- bFlowPctMode（默认 TRUE）在段启动沿锁存；FALSE 回到旧段标尺
- 百分比模式：PUMPFLOWPERCENT = uiSpd%，VELOCITY = 700mm/s 安全上限，
  标尺取目标点，uiEndSpd 表示目标点处该流量百分比的速度
- PressureHandle 仍用带 5% 下限的 rFlowPercent

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>
EOF
)"
```

---

### Task 4: 上机验收（用户在现场执行，不在本仓库内完成）

**Files:** 无代码改动。结果记录到新的 vofa CSV，与 `docs/data_deg/vofa_waveform_20261117_161526_748-开合模.csv` 对比。

- [ ] **Step 1: PLC 工程更新**

用新的 `pousHydMotion.xml` 重新生成接口，链接新的 `libHydroMotionLib.a`（`./scripts/deploy_embedded_prod.sh` 产出在 `out/install/embedded_prod`），替换 `FB_HydAxis.st` 并编译。

- [ ] **Step 2: 基线**

`bFlowPctMode = FALSE`，用现有工艺参数各跑 3 次开合模，确认与 11-17 实测一致（开模约 2.0s、合模约 1.5s）。

- [ ] **Step 3: 新模式 70%**

`bFlowPctMode = TRUE`，按 spec §7 的段结构、两个动作都先用 70%：
- 巡航泵转速 ≈ 0.70 × 1700 = 1190rpm（±2%）；
- `ReadStatus` 的 FLOW 标志不置位；
- 实测速度 / 规划速度 k ≈ 1（0.95~1.05）。

- [ ] **Step 4: 升到 spec §7 初值**

合模：20%→295（终点 20%）、95%→40（终点 95%）、45%→10（加速 200 / 减速 150）。开模：95% 单段，减速 700→600→500ms。逐项核对：
- 开模终点误差 ≤ ±1.5mm，InEndVelocity 正常置位，无反向回弹；超差时参数 35 按 5ms 一档调，先确认 k ≥ 1；
- 开模落地段峰值减速度 ≤ 2.8 m/s²（现状 1.85 的 1.5 倍）；
- 合模到 10mm 速度 ≤ 48mm/s（现状 43.5 的 1.1 倍），护模动作与报警不受影响；
- 全程泵转速无台阶式突降（合模 x=50 附近原来有 75ms 内 1700→930rpm 的跌落，现在应消失）。

- [ ] **Step 5: 周期验收**

合模 ≤ 1.30s（含起步段）、开模 ≤ 1.25s。把 CSV 存到 `docs/data_deg/`，再对照 spec §5 的探针值复盘。

---

## Self-Review（写计划时已完成）

**Spec 覆盖：**
- §4.1 语义、§4.2 接口：Task 2；
- §4.3.1~4.3.6：Task 1（Step 3~6、9）；
- §4.4 FB_HydAxis：Task 3；
- §6 T1~T5：Task 1，T6：Task 2（校验）与 Task 1 `test_zero_percent_keeps_legacy_velocity_semantics`、全量回归（关闭时不变）；
- §7 上机验收：Task 4；
- §8 风险：D2 误报由 T4/T5 断言覆盖，并在探针中复刻判据验证；其余风险项不需要代码。

**与 spec 原文的差异（已同步修订 spec）：**
- plcdemo 不含此 FB，三处同步改为两处，外加布局测试；
- `test_toggle_mechanism_pool` 预算已够，不改；
- 新增 `q_cap ≤ 0` 保护与 `uiSpd = 0` 零速保护；
- 合模初值由“95%→30、减速 250”改为“95%→40、减速 150”（原值按 spec 斜坡公式复核后到 10mm 仍有 74mm/s）；
- T4 参数 35 用 20（= 对象纯滞后），只隔离“回算”这一个变量。

**类型一致性：**
- `flowPercent`、`flowRampRate` 在 Task 1 定义，Task 2 与测试使用的名称一致；
- `HYD_AnchorContinuousAbsoluteFlowRamp` 与 `HYD_ResolveFlowPercentBase` 只在 `motion_control.c` 内部使用；
- IEC 字段名 `PUMPFLOWPERCENT` 在 C 头、XML、ST、测试中一致。
