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

static void seed_running_position_anchor(HYD_MotionControlFB* fb,
                                         HYD_REAL position,
                                         HYD_REAL velocity,
                                         HYD_REAL anchorTargetVelocity,
                                         HYD_REAL anchorRate,
                                         HYD_REAL anchorTimeMs)
{
    seed_running_position(fb, position, velocity);
    /* 前置段加速锚定：同号、目标同为 maxVelocity、t_acc 相同 →
     * HYD_AdvanceStatefulVelocity 保持缓存速率继续加速 */
    fb->_plannerState.transitionTargetVelocity = anchorTargetVelocity;
    fb->_plannerState.transitionRate = anchorRate;
    fb->_plannerState.transitionTimeMs = anchorTimeMs;
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

static void test_adapt_end_velocity_uses_carried_anchor_rate(void)
{
    HYD_MotionControlFB fb;
    HYD_MotionSegment approach = make_position_segment(3.0, 100.0);
    HYD_ContinuousAbsoluteContext ctx;
    HYD_DirectStartResult result;

    seed_running_position_anchor(&fb, 0.0, 51.0, 100.0, 1000.0, 100.0);
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
          "continuous-absolute anchor takeover must start");
    /* 锚定速率 1000：sqrt(51²+2*1000*3)=92.7 → 夹到 requested=80。
     * 公式法：(100-51)/0.1=490 → 74.44。 */
    check(fabs(fb._directContinuousAbsolute.crossingVelocity - 80.0) < 0.5,
          "carried acceleration anchor must drive crossing velocity");
}

static void test_adapt_end_velocity_anchor_rate_distinguishes_formulas(void)
{
    HYD_MotionControlFB fb;
    HYD_MotionSegment approach = make_position_segment(3.0, 100.0);
    HYD_ContinuousAbsoluteContext ctx;
    HYD_DirectStartResult result;

    seed_running_position_anchor(&fb, 0.0, 85.0, 100.0, 200.0, 100.0);
    memset(&ctx, 0, sizeof(ctx));
    ctx.valid = true;
    ctx.phase = HYD_CONTABS_PHASE_APPROACH;
    ctx.targetPosition = approach.targetPosition;
    ctx.sustainVelocity = 95.0;
    ctx.adaptEndVelEnabled = true;
    ctx.approachDirection = HYD_DIRECTION_EXTEND;
    ctx.sustainDirection = HYD_DIRECTION_EXTEND;

    result = HYD_MotionControlFB_StartDirectCommand(
        &fb, HYD_DIRECT_CMD_MOVE_CONTINUOUS_ABSOLUTE, &approach, &ctx,
        HYD_BUFFER_MODE_ABORT, 1.001);
    check(result == HYD_DIRECT_START_STARTED,
          "continuous-absolute anchor takeover must start");
    /* 锚定速率 200：sqrt(85²+2*200*3)=91.78，低于 requested=95 不夹。
     * 公式法：(100-85)/0.1=150 → 90.14；旧公式 1000 → 101.1 → 夹到 95。 */
    check(fabs(fb._directContinuousAbsolute.crossingVelocity - 91.78) < 0.5,
          "anchor rate result must be distinguishable from both formulas");
}

int main(void)
{
    test_adapt_end_velocity_uses_ramp_rate_from_projected_velocity();
    test_adapt_end_velocity_uses_carried_anchor_rate();
    test_adapt_end_velocity_anchor_rate_distinguishes_formulas();
    printf("decel semantics fb: %s\n", failures == 0 ? "PASS" : "FAIL");
    return failures == 0 ? 0 : 1;
}
