/* 曲肘轴开合模“泵流量百分比”语义集成测试（spec 2026-11-18 §6 T1~T5、T7~T10）。
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
    /* 终审 I1：降百分比过渡完成后，本段余下的恢复速率为 Q_base / t_acc（seg2 t_acc 200ms） */
    assert(fabsf(fb._directContinuousAbsolute.flowRampRate -
                 PUMP_MAX_RPM * 50.0f * 0.95f / 1000.0f / 0.2f) <= 0.01f);
}

/* 终审 I1：同百分比链式开模 1→30（95%，终点 = 30mm 处 95% 速度）→ 300（95%，零终速）。
 * 两段流量目标只差约 0.4 L/min：旧锚定 |Δq|/t_acc 只有约 2 L/min/s，第 2 段起步时
 * 规划器短暂下探，上限跟着下来后只能按该速率爬回，泵转速被压在约 1500rpm（棘轮）。
 * 2026-11-20 起第 1 段按 §4.3.7 保持流量处理，第 2 段起步 q0 = q_target，同流量回退分支的护栏改由 T10 承担。 */
static void test_open_same_percent_chain_recovers_pump_flow(void)
{
    HYD_MotionControlFB fb;
    HYD_MotionSegment seg1 = make_segment(30.0f, HYD_DIRECTION_EXTEND, 200.0f, 200.0f, 60.0f);
    HYD_MotionSegment seg2 = make_segment(300.0f, HYD_DIRECTION_EXTEND, 200.0f, 700.0f, 60.0f);
    HYD_ContinuousAbsoluteContext ctx1;
    HYD_ContinuousAbsoluteContext ctx2;
    PumpPlant plant;
    HYD_REAL cruisePeak = 0.0f;
    HYD_REAL pumpPeak = 0.0f;
    HYD_REAL pendingRpm = 0.0f;
    HYD_REAL finalPosition;
    HYD_BOOL inEndVelocity = false;
    HYD_BOOL completed = false;
    HYD_BOOL sawBraking = false;
    HYD_BOOL sawDeviation = false;
    int cycle;
    int k;

    memset(&plant, 0, sizeof(plant));
    plant.delayCycles = 20;
    plant.sign = 1.0f;
    init_axis(&fb, 1.0f, 20.0f, 60.0f);
    seg2.stableVelocityLimit = 3.0f;
    ctx1 = make_context(30.0f, ceiling_at(&fb, 30.0f, HYD_DIRECTION_EXTEND, 95.0f),
                        HYD_DIRECTION_EXTEND, 95.0f);
    ctx2 = make_context(300.0f, 0.0f, HYD_DIRECTION_EXTEND, 95.0f);

    assert(HYD_MotionControlFB_StartDirectCommand(&fb, HYD_DIRECT_CMD_MOVE_CONTINUOUS_ABSOLUTE,
                                                  &seg1, &ctx1, HYD_BUFFER_MODE_ABORT, 0.0f) ==
           HYD_DIRECT_START_STARTED);
    for (cycle = 0; cycle < 3000 &&
         fb._directContinuousAbsolute.phase == HYD_CONTABS_PHASE_APPROACH; ++cycle) {
        plant_step(&fb, &plant);
        assert(fb.FB_STATE != HYD_FB_STATE_FAULT);
        sawBraking |= is_flag(&fb, HYD_LIMIT_FLAG_BRAKING_DISTANCE);
        sawDeviation |= fb.DIAGNOSTIC.velocityDeviation;
    }
    assert(fb._directContinuousAbsolute.phase == HYD_CONTABS_PHASE_SUSTAIN);

    assert(HYD_MotionControlFB_StartDirectCommand(&fb, HYD_DIRECT_CMD_MOVE_CONTINUOUS_ABSOLUTE,
                                                  &seg2, &ctx2, HYD_BUFFER_MODE_ABORT,
                                                  fb.AXIS_REF.timestamp) ==
           HYD_DIRECT_START_STARTED);
    for (cycle = 0; cycle < 4000; ++cycle) {
        plant_step(&fb, &plant);
        assert(fb.FB_STATE != HYD_FB_STATE_FAULT);
        if (fb.PUMP_SPEED > pumpPeak) {
            pumpPeak = fb.PUMP_SPEED;
        }
        /* 起步下探（x < 60）之后、刹车之前的巡航窗口 */
        if (fb.AXIS_REF.position >= 60.0f && fb.AXIS_REF.position <= 100.0f &&
            fb.PUMP_SPEED > cruisePeak) {
            cruisePeak = fb.PUMP_SPEED;
        }
        sawBraking |= is_flag(&fb, HYD_LIMIT_FLAG_BRAKING_DISTANCE);
        sawDeviation |= fb.DIAGNOSTIC.velocityDeviation;
        if (fb._directContinuousAbsolute.inEndVelocityLatched) {
            inEndVelocity = true;
        }
        if (inEndVelocity && !fb.STATE.active) {
            completed = true;
            break;
        }
    }
    for (k = 0; k < plant.delayCycles; ++k) {
        pendingRpm += plant.history[k];
    }
    finalPosition = fb.AXIS_REF.position +
        ceiling_at(&fb, fb.AXIS_REF.position, HYD_DIRECTION_EXTEND, 100.0f) *
        pendingRpm / PUMP_MAX_RPM * DT;

    printf("I1 same-percent chain: seg2 %d ms, cruise peak %.1f rpm (x 60~100), "
           "pump peak %.1f, final %.3f\n", cycle, cruisePeak, pumpPeak, finalPosition);
    assert(completed);
    assert(fabsf(finalPosition - 300.0f) <= 1.5f);
    /* 上限仍以 q_prev 为中心限幅且不超过 q_target */
    assert(pumpPeak <= 0.95f * PUMP_MAX_RPM + 0.5f);
    /* 实测（2026-11-19）：棘轮时窗口内最高 1501 rpm；修复后 1575 rpm（此时只受规划器限制，
     * 评审探针用 Q_base/t_acc 强制速率得到 1507~1588）。取中点 1540，两侧各留约 35 rpm。 */
    assert(cruisePeak >= 1540.0f);
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

/* T7（spec §4.3.7，2026-11-20）：保持流量段与减速时间无关。
 * 合模 299.36→30（100%、加速 300、减速 X，终点 = 30mm 处 100% 速度）→ 10（45%、200/150）。
 * 现库（无 §4.3.7）回放：减速 100 时 x∈[30.5,220] 泵转速最低 1698rpm；减速 480 时最低约 1493rpm、
 * 第 1 段报 BRAKING_DISTANCE、耗时 870ms；减速 800 时最低约 1068rpm、耗时 1127ms。
 * 到 30mm 时速度在终点速度 ±2% 内且 InEndVelocity 置位（无刹车后到点速度由流量 × 速比决定）。 */
typedef struct {
    HYD_REAL crossing;
    HYD_REAL minPump;
    HYD_REAL v10;
    HYD_REAL v10Target;
    HYD_REAL v30;
    HYD_REAL v30Target;
    HYD_BOOL latched30;
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
    r.v30Target = ceiling_at(&fb, 30.0f, HYD_DIRECTION_RETRACT, 100.0f);
    ctx1 = make_context(30.0f, r.v30Target, HYD_DIRECTION_RETRACT, 100.0f);
    ctx2 = make_context(10.0f, r.v10Target, HYD_DIRECTION_RETRACT, 45.0f);

    assert(HYD_MotionControlFB_StartDirectCommand(&fb, HYD_DIRECT_CMD_MOVE_CONTINUOUS_ABSOLUTE,
                                                  &seg1, &ctx1, HYD_BUFFER_MODE_ABORT, 0.0f) ==
           HYD_DIRECT_START_STARTED);
    r.crossing = fb._directContinuousAbsolute.crossingVelocity;
    for (cycle = 0u; cycle < 3000u &&
         fb._directContinuousAbsolute.phase == HYD_CONTABS_PHASE_APPROACH; ++cycle) {
        plant_step(&fb, &plant);
        assert(fb.FB_STATE != HYD_FB_STATE_FAULT);
        if (fb.AXIS_REF.position >= 30.5f && fb.AXIS_REF.position <= 220.0f &&
            fb.PUMP_SPEED < r.minPump) {
            r.minPump = fb.PUMP_SPEED;
        }
        r.seg1Braking |= is_flag(&fb, HYD_LIMIT_FLAG_BRAKING_DISTANCE);
        r.sawDeviation |= fb.DIAGNOSTIC.velocityDeviation;
    }
    assert(fb._directContinuousAbsolute.phase == HYD_CONTABS_PHASE_SUSTAIN);
    r.v30 = -fb.AXIS_REF.velocity;
    r.latched30 = fb._directContinuousAbsolute.inEndVelocityLatched;
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
        printf("T7 flow-hold close d%.0f: crossing %.1f, min pump %.2f (x 30.5~220), braking %d, "
               "seg1 %u ms, v@10 %.1f (target %.1f), v@30 %.2f (end %.2f), latched %d\n",
               decs[i], r[i].crossing, r[i].minPump, (int)r[i].seg1Braking,
               r[i].seg1Cycles, r[i].v10, r[i].v10Target,
               r[i].v30, r[i].v30Target, (int)r[i].latched30);
    }
    fflush(stdout);
    for (i = 0; i < 3; ++i) {
        assert(r[i].minPump >= PUMP_MAX_RPM - 0.5f);
        assert(!r[i].seg1Braking);
        assert(!r[i].sawDeviation);
        assert(fabsf(r[i].v10 - r[i].v10Target) <= 0.10f * r[i].v10Target);
        assert(fabsf(r[i].crossing - 700.0f) <= 0.01f);
        assert(r[i].latched30);
        assert(r[i].v30 >= r[i].v30Target * 0.98f && r[i].v30 <= r[i].v30Target * 1.02f);
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
 * 条件 3 用相对容差：终点流量 ≥ 本段流量 × (1 − 2%)，本机即终点百分比 ≥ 本段百分比 × 0.98。 */
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
        {"close 10%/end 9%", 100.0f, 30.0f, HYD_DIRECTION_RETRACT, 10.0f, 9.0f, false},
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

/* T10（2026-11-20 终审 I2）：§4.3.2 同流量回退的回归护栏。§4.3.7 之后 I1 第 1 段按保持流量处理，第 2 段起步
 * q0 恰好等于 q_target，两条锚定分支速率相同，I1 已护不住该回退。这里中间段为降流量保持段（100%→95%，条件 4
 * 不满足，保留速度域刹车），第 3 段起步 q0 比 q_target 低约 0.2 L/min（在 2% 带内）。
 * 实测（0ea8dfd）：正常时第 3 段起步速率 403.75 L/min/s、x 60~100 巡航峰值 1615.0rpm；
 * 去掉同流量回退（速率改为 |Δq|/t_acc ≈ 1.05 L/min/s）后峰值 1523.2rpm。门限沿用 I1 的 1540。 */
static void test_ramp_down_hold_chain_recovers_pump_flow(void)
{
    HYD_MotionControlFB fb;
    HYD_MotionSegment seg1 = make_segment(10.0f, HYD_DIRECTION_EXTEND, 200.0f, 200.0f, 60.0f);
    HYD_MotionSegment seg2 = make_segment(30.0f, HYD_DIRECTION_EXTEND, 200.0f, 200.0f, 60.0f);
    HYD_MotionSegment seg3 = make_segment(300.0f, HYD_DIRECTION_EXTEND, 200.0f, 700.0f, 60.0f);
    HYD_ContinuousAbsoluteContext ctx1;
    HYD_ContinuousAbsoluteContext ctx2;
    HYD_ContinuousAbsoluteContext ctx3;
    PumpPlant plant;
    HYD_REAL seg2Crossing;
    HYD_REAL seg2End;
    HYD_REAL q0;
    HYD_REAL qTarget;
    HYD_REAL cruisePeak = 0.0f;
    unsigned int cycle;

    memset(&plant, 0, sizeof(plant));
    plant.delayCycles = 20;
    plant.sign = 1.0f;
    init_axis(&fb, 1.0f, 20.0f, 60.0f);
    seg3.stableVelocityLimit = 3.0f;
    ctx1 = make_context(10.0f, ceiling_at(&fb, 10.0f, HYD_DIRECTION_EXTEND, 100.0f),
                        HYD_DIRECTION_EXTEND, 100.0f);
    seg2End = ceiling_at(&fb, 30.0f, HYD_DIRECTION_EXTEND, 95.0f);
    ctx2 = make_context(30.0f, seg2End, HYD_DIRECTION_EXTEND, 95.0f);
    ctx3 = make_context(300.0f, 0.0f, HYD_DIRECTION_EXTEND, 95.0f);
    qTarget = 0.95f * PUMP_MAX_RPM * 50.0f * 0.95f / 1000.0f;

    assert(HYD_MotionControlFB_StartDirectCommand(&fb, HYD_DIRECT_CMD_MOVE_CONTINUOUS_ABSOLUTE,
                                                  &seg1, &ctx1, HYD_BUFFER_MODE_ABORT, 0.0f) ==
           HYD_DIRECT_START_STARTED);
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
    for (cycle = 0u; cycle < 3000u &&
         fb._directContinuousAbsolute.phase == HYD_CONTABS_PHASE_APPROACH; ++cycle) {
        plant_step(&fb, &plant);
        assert(fb.FB_STATE != HYD_FB_STATE_FAULT);
    }
    assert(fb._directContinuousAbsolute.phase == HYD_CONTABS_PHASE_SUSTAIN);

    q0 = fabsf(fb._plannerState.lastTargetFlow);
    assert(HYD_MotionControlFB_StartDirectCommand(&fb, HYD_DIRECT_CMD_MOVE_CONTINUOUS_ABSOLUTE,
                                                  &seg3, &ctx3, HYD_BUFFER_MODE_ABORT,
                                                  fb.AXIS_REF.timestamp) ==
           HYD_DIRECT_START_STARTED);
    for (cycle = 0u; cycle < 4000u && fb.AXIS_REF.position < 100.0f; ++cycle) {
        plant_step(&fb, &plant);
        assert(fb.FB_STATE != HYD_FB_STATE_FAULT);
        if (fb.AXIS_REF.position >= 60.0f && fb.PUMP_SPEED > cruisePeak) {
            cruisePeak = fb.PUMP_SPEED;
        }
    }

    printf("T10 ramp-down hold chain: seg2 crossing %.2f (end %.2f), seg3 q0 %.3f (target %.3f), "
           "cruise peak %.1f rpm (x 60~100)\n", seg2Crossing, seg2End, q0, qTarget, cruisePeak);
    fflush(stdout);
    /* 前提：中间段为降流量保持段（不触发 §4.3.7），第 3 段起步 q0 在 2% 带内但不等于 q_target */
    assert(fabsf(seg2Crossing - seg2End) <= 0.01f);
    assert(q0 < qTarget - 0.01f);
    assert(qTarget - q0 <= HYD_FLOW_PERCENT_RAMP_EQUAL_RATIO * PUMP_MAX_RPM * 50.0f * 0.95f / 1000.0f);
    assert(fb.AXIS_REF.position >= 100.0f);
    assert(cruisePeak >= 1540.0f);
}

int main(void)
{
    /* T4 放第一个：变异检查（去掉回算）时先在 T4 失败 */
    test_open_zero_end_stop_uses_back_calculated_velocity();
    test_open_cruise_tracks_pump_flow_percent();
    test_close_chained_segments_ramp_flow_down();
    test_open_same_percent_chain_recovers_pump_flow();
    test_zero_percent_keeps_legacy_velocity_semantics();
    test_close_flow_hold_ignores_deceleration_time();
    test_ramp_down_flow_hold_keeps_velocity_brake();
    test_flow_hold_classification_boundaries();
    test_ramp_down_hold_chain_recovers_pump_flow();
    printf("test_toggle_flow_percent_integration passed\n");
    return 0;
}
