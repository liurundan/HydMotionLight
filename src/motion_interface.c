#include "motion_interface.h"
#include "segment_limits.h"
#include "toggle_mechanism_pool.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

/* ======================================================================
 * FB实例池管理
 * ====================================================================== */
static HYD_REAL dfCycleTime = 0.001f;  /* 默认周期时间，单位秒；可通过外部接口调整以适配不同PLC扫描周期 */

// 全局静态运动控制数组，STM32指定专用内存段，Win32走默认内存,aligned(8)
static
#if defined(__GNUC__)
__attribute__((section(".motionAlgoSection"), aligned(8)))
#endif
HYD_MotionControlFB HYD_MotionControlFB_inst[HYD_MAX_AXIS_MOTION];

typedef enum {
    HYD_AXIS_SLOT_FREE = 0,
    HYD_AXIS_SLOT_RESERVED,
    HYD_AXIS_SLOT_ACTIVE
} HYD_AxisSlotState;

static HYD_AxisSlotState HYD_AxisSlots[HYD_MAX_AXIS_MOTION];
static HYD_UINT16 HYD_FrameworkGeneration;

static const HYD_REAL HYD_CONTABS_DIRECTION_VELOCITY_THRESHOLD = 0.01f;

static int allocMotionControlFB(void)
{
    int index;

    for (index = 0; index < HYD_MAX_AXIS_MOTION; ++index) {
        if (HYD_AxisSlots[index] == HYD_AXIS_SLOT_FREE) {
            HYD_AxisSlots[index] = HYD_AXIS_SLOT_RESERVED;
            return index;
        }
    }
    return -1;
}

HYD_MotionControlFB* __MK_GetPublic_MotionControlFB(int index)
{
    if ((index >= 0) && (index < HYD_MAX_AXIS_MOTION) &&
        (HYD_AxisSlots[index] == HYD_AXIS_SLOT_ACTIVE)) {
        return &HYD_MotionControlFB_inst[index];
    }
    return NULL;
}

/* ======================================================================
 * 命令仲裁与所有权追踪
 *
 * 采用两阶段所有权模型:
 *
 * 阶段1 (_PENDING=true): execRising 后, FB 等待核心引擎确认为
 * 活跃段。确认后将 epoch (recipe 用 _recipeBatchId, direct 用
 * _executionId) 写入 _EXEC_ID, 并清除 _PENDING。
 * 如果核心引擎报告 ABORTED, 则立即输出 COMMANDABORTED。
 *
 * 阶段2 (_EXEC_ID != 0): 持有 epoch 比较值, 与 FB 对应 epoch 比对:
 *   - 匹配 → 仍是活跃命令, 正常读取核心 FB 输出
 *   - 不匹配 → 被其他命令取代, 输出 COMMANDABORTED
 *
 * Epoch 字段语义 (motion_control.h 详细描述):
 *   - _executionId: per-segment epoch, 递增于每次 BeginSegment
 *     (含 NextSegment)。用于 direct-command (MoveAbsolute / MoveVelocity /
 *     PressureHandle / Stop) 的所有权追踪。
 *   - _recipeBatchId: per-recipe-batch epoch, 仅在 initial Start /
 *     ABORT / STOP / Reset / direct takeover 时递增, NextSegment 不递增。
 *     用于 MoveProfile 的所有权追踪。
 *   两者解耦避免了多段 recipe 推进时 MoveProfile 误报 COMMANDABORTED。
 *
 * 仲裁规则:
 *   - MoveProfile (Recipe模式) 与 Direct模式命令互斥
 *   - 新命令可以接管正在执行的旧命令
 *   - 被接管的命令输出 COMMANDABORTED=true, BUSY=false
 * ====================================================================== */

/* ======================================================================
 * 内部辅助函数
 * ====================================================================== */


/* PLC方向映射 — Beckhoff TF5810 MC_Direction 兼容
 * DIRECTION SINT 值: 0=Shortest_Way, 1=Positive, 2=Negative, 3=Current
 * 超出范围的值默认视为 Shortest_Way */
static HYD_MotionDirection mapPlcOpenDirection(IEC_SINT direction) {
    switch ((int)direction) {
        case 1:  return HYD_DIRECTION_POSITIVE;
        case 2:  return HYD_DIRECTION_NEGATIVE;
        case 3:  return HYD_DIRECTION_CURRENT;
        default: return HYD_DIRECTION_SHORTEST_WAY;
    }
}

static HYD_MotionDirection mapContinuousEndVelocityDirectionRequest(IEC_SINT direction,
                                                                    HYD_BOOL* ok)
{
    if (ok != NULL) {
        *ok = true;
    }

    switch ((int)direction) {
        case 0:
            /* Unset IEC inputs arrive as zero; treat that as CURRENT so
             * defaulted callers use the existing runtime resolution chain. */
            return HYD_DIRECTION_CURRENT;
        case 1:
            return HYD_DIRECTION_POSITIVE;
        case 2:
            return HYD_DIRECTION_NEGATIVE;
        case 3:
            return HYD_DIRECTION_CURRENT;
        default:
            if (ok != NULL) {
                *ok = false;
            }
            return HYD_DIRECTION_HOLD;
    }
}

static HYD_MotionDirection resolveContinuousEndVelocityDirection(const HYD_MotionControlFB* fb,
                                                                 HYD_MotionDirection requestedDirection,
                                                                 HYD_MotionDirection approachDirection)
{
    if (requestedDirection != HYD_DIRECTION_CURRENT) {
        return requestedDirection;
    }

    if (fb != NULL) {
        if (fb->AXIS_REF.velocity > HYD_CONTABS_DIRECTION_VELOCITY_THRESHOLD) {
            return HYD_DIRECTION_POSITIVE;
        }
        if (fb->AXIS_REF.velocity < -HYD_CONTABS_DIRECTION_VELOCITY_THRESHOLD) {
            return HYD_DIRECTION_NEGATIVE;
        }
        if (fb->_lastActiveDirection == HYD_DIRECTION_POSITIVE ||
            fb->_lastActiveDirection == HYD_DIRECTION_NEGATIVE) {
            return fb->_lastActiveDirection;
        }
    }

    return approachDirection;
}

/* 构建位置控制运动段 (MoveAbsolute用) */
static HYD_MotionSegment buildPositionSegment(
    HYD_REAL targetPosition,
    HYD_REAL velocity,
    HYD_REAL acceleration,
    HYD_REAL deceleration,
    HYD_MotionDirection direction,
    const HYD_MotionControlFB* fb)
{
    HYD_MotionSegment seg;
    memset(&seg, 0, sizeof(seg));

    seg.segmentTag = HYD_SEGMENT_TYPE_OTHER;
    seg.segmentType = HYD_SEGMENT_TYPE_OTHER;
    seg.mode = HYD_MODE_POSITION;
    seg.endCondition = HYD_END_POSITION;
    seg.direction = direction;
    seg.planner = HYD_PLANNER_POSITION_BASED;

    seg.targetPosition = targetPosition;
    seg.maxVelocity = velocity;
    seg.maxAcceleration = acceleration;
    seg.maxDeceleration = (deceleration > 0.0f) ? deceleration : acceleration;
    seg.velocityToFlowGain = fb->_params.velocityToFlowGain;

    seg.positionTolerance = fb->_params.positionTolerance;
    seg.timeoutLimit = fb->_params.timeoutLimit;

    seg.maxFlow = fb->_params.maxFlow;

    return seg;
}

/* 构建速度控制运动段 (MoveVelocity用) */
static HYD_MotionSegment buildVelocitySegment(
    HYD_REAL velocity,
    HYD_REAL acceleration,
    HYD_REAL deceleration,
    HYD_MotionDirection direction,
    HYD_REAL pressureLimit,
    const HYD_MotionControlFB* fb)
{
    HYD_MotionSegment seg;
    memset(&seg, 0, sizeof(seg));

    seg.segmentTag = HYD_SEGMENT_TYPE_OTHER;
    seg.segmentType = HYD_SEGMENT_TYPE_OTHER;
    seg.mode = HYD_MODE_SPEED_RAMP;
    seg.endCondition = HYD_END_MANUAL;
    seg.direction = direction;
    seg.planner = HYD_PLANNER_TIME_BASED;

    seg.maxVelocity = velocity;
    seg.maxAcceleration = acceleration;
    seg.maxDeceleration = (deceleration > 0.0f) ? deceleration : acceleration;
    seg.velocityToFlowGain = fb->_params.velocityToFlowGain;
    seg.maxPressure = pressureLimit;

    seg.timeoutLimit = 0.0f;

    seg.maxFlow = fb->_params.maxFlow;

    return seg;
}

static HYD_MotionSegment buildContinuousAbsoluteApproachSegment(
    HYD_REAL targetPosition,
    HYD_REAL velocity,
    HYD_REAL acceleration,
    HYD_REAL deceleration,
    HYD_MotionDirection direction,
    HYD_REAL pressureLimit,
    const HYD_MotionControlFB* fb)
{
    HYD_MotionSegment seg = buildPositionSegment(targetPosition,
                                                 velocity,
                                                 acceleration,
                                                 deceleration,
                                                 direction,
                                                 fb);

    seg.maxPressure = pressureLimit;
    seg.velocityTolerance = fb->_params.velocityTolerance;

    return seg;
}

static HYD_MotionDirection normalizeContinuousMotionDirection(HYD_MotionDirection direction,
                                                              HYD_MotionDirection fallbackDirection)
{
    if (direction == HYD_DIRECTION_POSITIVE || direction == HYD_DIRECTION_NEGATIVE) {
        return direction;
    }

    return (fallbackDirection == HYD_DIRECTION_NEGATIVE)
        ? HYD_DIRECTION_NEGATIVE
        : HYD_DIRECTION_POSITIVE;
}

static HYD_ContinuousAbsoluteContext buildContinuousAbsoluteContext(
    HYD_REAL targetPosition,
    HYD_REAL endVelocity,
    HYD_REAL pressureLimit,
    HYD_BOOL adaptEndVelEnabled,
    HYD_MotionDirection approachDirection,
    HYD_MotionDirection sustainDirection)
{
    HYD_ContinuousAbsoluteContext ctx;
    HYD_MotionDirection normalizedSustainDirection =
        normalizeContinuousMotionDirection(sustainDirection, approachDirection);
    HYD_MotionDirection normalizedApproachDirection =
        normalizeContinuousMotionDirection(approachDirection, normalizedSustainDirection);

    memset(&ctx, 0, sizeof(ctx));
    ctx.valid = true;
    ctx.phase = HYD_CONTABS_PHASE_APPROACH;
    ctx.targetPosition = targetPosition;
    ctx.sustainVelocity = (normalizedSustainDirection == HYD_DIRECTION_NEGATIVE)
        ? -endVelocity
        : endVelocity;
    ctx.effectivePressureLimit = pressureLimit;
    ctx.adaptEndVelEnabled = adaptEndVelEnabled;
    ctx.approachDirection = normalizedApproachDirection;
    ctx.sustainDirection = normalizedSustainDirection;

    return ctx;
}

/* 构建压力控制运动段 (PressureHandle用) */
static HYD_MotionSegment buildPressureSegment(
    HYD_REAL targetPressure,
    HYD_REAL rampRate,
    HYD_REAL duration,
    const HYD_MotionControlFB* fb)
{
    HYD_MotionSegment seg;
    memset(&seg, 0, sizeof(seg));

    seg.segmentTag = HYD_SEGMENT_TYPE_HOLDING;
    seg.segmentType = HYD_SEGMENT_TYPE_HOLDING;
    seg.mode = HYD_MODE_PRESSURE_CLOSED_LOOP;
    seg.endCondition = (duration > 0.0f) ? HYD_END_TIME : HYD_END_MANUAL;
    seg.direction = HYD_DIRECTION_HOLD;

    seg.targetPressure = targetPressure;
    seg.targetFlow = fb->_params.defaultTargetFlow;
    /* Preserve the established PressureHandle limit; recipe/profile paths use
     * the configurable maxFlow parameter independently. */
    seg.maxFlow = 20.0;
    seg.duration = duration;
    seg.pressureRampRate = rampRate;
    seg.pressureCeiling  = MAX_PRESSURE;
    seg.systemGain = 30.f; // 80.5 bar/L/min, 2028-08-04 测试数据，后续可通过配置表调整
    seg.pressureController = (HYD_PressureControllerType)(int)fb->_params.pressureControllerType;
    seg.pressureKp = fb->_params.pressureKp;
    seg.pressureKi = fb->_params.pressureKi;
    seg.pressureKd = fb->_params.pressureKd;
    seg.pressureIntegralLimit = fb->_params.pressureIntegralLimit;
    seg.pressureDeadband = fb->_params.pressureDeadband;
    seg.pressureFilterAlpha = fb->_params.pressureFilterAlpha;
    seg.pressureDerivativeFilterAlpha = fb->_params.pressureDerivativeFilterAlpha;

    seg.pressureTolerance = fb->_params.pressureTolerance;
    seg.flowTolerance = fb->_params.flowTolerance;
    seg.timeoutLimit = fb->_params.timeoutLimit;

    return seg;
}

/* 从MOTION结构体构建运动段 (MoveProfile用) */
static HYD_MotionSegment buildSegmentFromMotion(const HYD_AXISMOTION* motion,
                                                 const HYD_MotionControlFB* fb)
{
    HYD_MotionSegment seg;
    memset(&seg, 0, sizeof(seg));

    seg.segmentTag = (HYD_UINT8)motion->SEGMENTTAG;
    seg.segmentType = (HYD_SegmentType)motion->SEGMENTTYPE;
    seg.mode = (HYD_ControlMode)motion->MODE;
    seg.endCondition = (HYD_EndConditionType)motion->ENDCONDITION;
    seg.direction = (HYD_MotionDirection)motion->DIRECTION;
    seg.planner = (HYD_PlannerType)motion->PLANNER;

    seg.targetPosition = motion->SETPOSITION;
    seg.maxVelocity = motion->SETVELOCITY;
    seg.targetFlow = motion->SETFLOW;
    seg.maxFlow = motion->SETFLOW;
    seg.targetPressure = motion->SETPRESSURE;
    seg.maxAcceleration = motion->ACCELERATION;
    seg.maxDeceleration = (motion->DECELERATION > 0.0f)
        ? motion->DECELERATION
        : motion->ACCELERATION;
    seg.duration = motion->DURATION;
    seg.pressureRampRate = motion->PRESSURERAMPRATE;

    seg.velocityToFlowGain = fb->_params.velocityToFlowGain;
    seg.positionTolerance = fb->_params.positionTolerance;
    seg.pressureTolerance = fb->_params.pressureTolerance;
    seg.flowTolerance = fb->_params.flowTolerance;
    seg.timeoutLimit = fb->_params.timeoutLimit;

    seg.pressureController = (HYD_PressureControllerType)(int)fb->_params.pressureControllerType;
    seg.pressureKp = fb->_params.pressureKp;
    seg.pressureKi = fb->_params.pressureKi;
    seg.pressureKd = fb->_params.pressureKd;
    seg.pressureIntegralLimit = fb->_params.pressureIntegralLimit;
    seg.pressureDeadband = fb->_params.pressureDeadband;
    seg.pressureFilterAlpha = fb->_params.pressureFilterAlpha;
    seg.pressureDerivativeFilterAlpha = fb->_params.pressureDerivativeFilterAlpha;

    return seg;
}

/*
 * HYD_AXISMOTION ownership contract (Sprint 0 spec C-1):
 *
 * The runtime MUST NOT write back to the Setpoint half of HYD_AXISMOTION
 * (SEGMENTTAG, SEGMENTTYPE, PLANNER, MODE, ENDCONDITION, DIRECTION,
 *  SET*, ACCELERATION, DECELERATION, DURATION, PRESSURERAMPRATE).
 *
 * Multiple MoveProfile / MoveAbsolute / MoveVelocity / PressureHandle FB
 * instances may bind to the same physical HYD_AXISMOTION. Reflecting the
 * currently active segment back into MOTION silently clobbers whatever
 * setpoint the PLC has just queued for the next scan, leading to data
 * races. The Setpoint half is therefore PLC-owned (and write-only from
 * the runtime's perspective); the Actual half (ACT*, TIMESTAMP) remains
 * runtime-owned. See:
 *   - include/motion_interface.h HYD_AXISMOTION typedef header comment
 *   - docs/integration/plc-process-layer-integration-guide.md
 *     "HYD_AXISMOTION Half-Region Ownership"
 *
 * The previous writeMotionFromSegment() helper that projected the active
 * segment back into MOTION has been removed for this reason.
 */

typedef enum {
    HYD_DIRECT_PENDING_WAITING = 0,
    HYD_DIRECT_PENDING_ACQUIRED,
    HYD_DIRECT_PENDING_ABORTED
} HYD_DirectPendingStatus;

static HYD_DirectPendingStatus resolveDirectPendingOwnership(HYD_MotionControlFB* fb,
                                                             IEC_WORD* execId)
{
    if (fb == NULL) {
        return HYD_DIRECT_PENDING_WAITING;
    }

    if (fb->STATE.active && fb->_activeSegmentSource == HYD_SEGMENT_SOURCE_DIRECT) {
        /* Still queued in the pending slot — not yet the active segment.
         * Return WAITING so the IEC FB does not latch the predecessor's
         * executionId and mistakenly claim ownership of the wrong segment. */
        if (fb->_directPendingValid) {
            return HYD_DIRECT_PENDING_WAITING;
        }
        if (execId != NULL) {
            *execId = (IEC_WORD)fb->_executionId;
        }
        return HYD_DIRECT_PENDING_ACQUIRED;
    }

    if (fb->FB_STATE == HYD_FB_STATE_ABORTED && !fb->STATE.active) {
        return HYD_DIRECT_PENDING_ABORTED;
    }

    return HYD_DIRECT_PENDING_WAITING;
}

static HYD_BOOL directExecutionWasPreempted(const HYD_MotionControlFB* fb,
                                            IEC_WORD execId,
                                            HYD_DirectCommandKind kind)
{
    return HYD_MotionControlFB_WasDirectTicketPreempted(fb, (uint16_t)execId, kind);
}

static HYD_BOOL directExecutionWasCompleted(HYD_MotionControlFB* fb,
                                            IEC_WORD execId,
                                            HYD_DirectCommandKind kind)
{
    return HYD_MotionControlFB_ConsumeDirectTicketCompleted(fb, (uint16_t)execId, kind);
}

static HYD_BOOL directExecutionIsCurrentOwner(const HYD_MotionControlFB* fb,
                                              IEC_WORD execId,
                                              HYD_DirectCommandKind kind)
{
    return execId == (IEC_WORD)HYD_MotionControlFB_GetDirectOwnerTicket(fb) &&
           HYD_MotionControlFB_GetDirectOwnerKind(fb) == kind;
}

static HYD_BOOL directExecutionLostOwnership(const HYD_MotionControlFB* fb,
                                             IEC_WORD execId,
                                             HYD_DirectCommandKind kind)
{
    return execId != (IEC_WORD)HYD_MotionControlFB_GetDirectOwnerTicket(fb) ||
           HYD_MotionControlFB_GetDirectOwnerKind(fb) != kind;
}

/* Recipe-side ownership predicates use _recipeBatchId rather than the
 * per-segment _executionId so that multi-segment recipe NextSegment
 * progress is NOT mistaken for external takeover. _recipeBatchId advances
 * only on initial Start / ABORT / STOP / Reset / direct takeover; it stays
 * constant across recipe NextSegment. See HYD_MotionControlFB definition
 * in motion_control.h for full contract. */
static HYD_BOOL recipeExecutionLostOwnership(const HYD_MotionControlFB* fb,
                                             IEC_WORD execId)
{
    if (fb == NULL || execId == 0) {
        return false;
    }

    if (HYD_MotionControlFB_IsDone(fb) && fb->STATE.finished) {
        return false;
    }

    /* Batch-id mismatch ⇒ a fresh recipe batch began (or external takeover
     * fired). With the batch-id semantics, NextSegment within the same batch
     * keeps execId equal — no false positive. The recipe-source short-circuit
     * that used to live here was specifically masking external Stop/Abort
     * preemption while the recipe source flag was still RECIPE; the batch-id
     * change now provides the correct discrimination. */
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
           HYD_MotionControlFB_GetDirectOwnerKind(fb) == HYD_DIRECT_CMD_NONE;
}

static HYD_BOOL recipeExecutionWasTakenOverBeforeLatch(const HYD_MotionControlFB* fb)
{
    return fb != NULL &&
           fb->_recipeBatchId != 0U &&
           HYD_MotionControlFB_GetDirectOwnerKind(fb) != HYD_DIRECT_CMD_NONE &&
           fb->_activeSegmentSource == HYD_SEGMENT_SOURCE_DIRECT;
}

static HYD_BOOL directStopCanCompleteImmediately(const HYD_MotionControlFB* fb)
{
    return fb != NULL &&
           fb->FB_STATE != HYD_FB_STATE_STARTING &&
           fb->FB_STATE != HYD_FB_STATE_RUNNING &&
           fb->FB_STATE != HYD_FB_STATE_HOLD;
}

static HYD_BOOL axisIndexIsAllocated(IEC_SINT axisIndex)
{
    return (axisIndex >= 0) && (axisIndex < (IEC_SINT)HYD_MAX_AXIS_MOTION) &&
           (HYD_AxisSlots[(int)axisIndex] == HYD_AXIS_SLOT_ACTIVE);
}

static IEC_WORD commandFailureErrorId(const HYD_MotionControlFB* fb)
{
    if (fb != NULL) {
        if (fb->ERROR_ID != HYD_DIAG_CODE_NONE) {
            return (IEC_WORD)fb->ERROR_ID;
        }
        if (fb->DIAGNOSTIC.code != HYD_DIAG_CODE_NONE) {
            return (IEC_WORD)fb->DIAGNOSTIC.code;
        }
    }
    return (IEC_WORD)HYD_DIAG_CODE_COMMAND_NOT_ALLOWED;
}

static HYD_BOOL validateSupportedBufferMode(IEC_INT bufferMode, IEC_WORD* errorId)
{
    if (bufferMode >= HYD_BUFFER_MODE_ABORT &&
        bufferMode <= HYD_BUFFER_MODE_BLENDING_HIGH) {
        return true;
    }

    if (errorId != NULL) {
        *errorId = (IEC_WORD)HYD_DIAG_CODE_COMMAND_NOT_ALLOWED;
    }
    return false;
}

static HYD_BOOL validateUnsupportedMotionOptions(IEC_REAL jerk,
                                                 IEC_WORD* errorId)
{
    if (fabs((double)jerk) <= 1e-6) {
        return true;
    }

    if (errorId != NULL) {
        *errorId = (IEC_WORD)HYD_DIAG_CODE_COMMAND_NOT_ALLOWED;
    }
    return false;
}

static HYD_BOOL resolvePressureLimit(HYD_REAL requestedLimit,
                                     const HYD_MotionControlFB* fb,
                                     HYD_REAL* resolvedLimit,
                                     IEC_WORD* errorId)
{
    HYD_REAL effectiveLimit;

    if (!isfinite(requestedLimit) || fb == NULL || resolvedLimit == NULL) {
        if (errorId != NULL) {
            *errorId = (IEC_WORD)HYD_DIAG_CODE_COMMAND_NOT_ALLOWED;
        }
        return false;
    }

    effectiveLimit = (requestedLimit > 0.0f) ? requestedLimit : fb->PRESSURE_LIMIT;
    if (!isfinite(effectiveLimit)) {
        if (errorId != NULL) {
            *errorId = (IEC_WORD)HYD_DIAG_CODE_COMMAND_NOT_ALLOWED;
        }
        return false;
    }

    *resolvedLimit = effectiveLimit;
    return true;
}

static HYD_BOOL applyMoveAbsoluteLiveUpdate(HYD_MotionControlFB* fb,
                                            IEC_WORD execId,
                                            HYD_MOVEABSOLUTE* data__)
{
    HYD_LiveUpdateRequest request;

    if (fb == NULL || data__ == NULL || !__GET_VAR(data__->CONTINUOUSUPDATE)) {
        return true;
    }

    memset(&request, 0, sizeof(request));
    request.flags = HYD_LIVE_UPDATE_TARGET_POSITION |
                    HYD_LIVE_UPDATE_MAX_VELOCITY |
                    HYD_LIVE_UPDATE_ACCELERATION |
                    HYD_LIVE_UPDATE_DECELERATION |
                    HYD_LIVE_UPDATE_CONTINUOUS_UPDATE |
                    HYD_LIVE_UPDATE_DIRECTION;
    request.ownerKind = HYD_DIRECT_CMD_MOVE_ABSOLUTE;
    request.ownerTicket = (uint16_t)execId;
    request.targetPosition = __GET_VAR(data__->POSITION);
    request.maxVelocity = __GET_VAR(data__->VELOCITY);
    request.maxAcceleration = __GET_VAR(data__->ACCELERATION);
    request.maxDeceleration = __GET_VAR(data__->DECELERATION);
    request.direction = mapPlcOpenDirection(__GET_VAR(data__->DIRECTION));
    return HYD_MotionControlFB_ApplyLiveUpdate(fb, &request);
}

static HYD_BOOL applyMoveVelocityLiveUpdate(HYD_MotionControlFB* fb,
                                            IEC_WORD execId,
                                            HYD_MOVEVELOCITY* data__)
{
    HYD_LiveUpdateRequest request;
    IEC_WORD errorId = 0;
    HYD_REAL pressureLimit;

    if (fb == NULL || data__ == NULL || !__GET_VAR(data__->CONTINUOUSUPDATE)) {
        return true;
    }

    if (!resolvePressureLimit(__GET_VAR(data__->PRESSURELIMIT),
                              fb,
                              &pressureLimit,
                              &errorId)) {
        return false;
    }

    HYD_REAL rawVelocity = __GET_VAR(data__->VELOCITY);
    HYD_MotionDirection dir = mapPlcOpenDirection(__GET_VAR(data__->DIRECTION));

    /* SHORTEST_WAY: derive direction from Velocity sign — match execRising path */
    if (dir == HYD_DIRECTION_SHORTEST_WAY) {
        if (rawVelocity > 0.0f) {
            dir = HYD_DIRECTION_POSITIVE;
        } else if (rawVelocity < 0.0f) {
            dir = HYD_DIRECTION_NEGATIVE;
        } else {
            dir = (fb->_lastActiveDirection == HYD_DIRECTION_NEGATIVE)
                  ? HYD_DIRECTION_NEGATIVE : HYD_DIRECTION_POSITIVE;
        }
    }

    memset(&request, 0, sizeof(request));
    request.flags = HYD_LIVE_UPDATE_MAX_VELOCITY |
                    HYD_LIVE_UPDATE_ACCELERATION |
                    HYD_LIVE_UPDATE_DECELERATION |
                    HYD_LIVE_UPDATE_CONTINUOUS_UPDATE |
                    HYD_LIVE_UPDATE_DIRECTION |
                    HYD_LIVE_UPDATE_MAX_PRESSURE;
    request.ownerKind = HYD_DIRECT_CMD_MOVE_VELOCITY;
    request.ownerTicket = (uint16_t)execId;
    request.maxVelocity = (IEC_REAL)fabs((double)rawVelocity);
    request.maxAcceleration = __GET_VAR(data__->ACCELERATION);
    request.maxDeceleration = __GET_VAR(data__->DECELERATION);
    request.maxPressure = pressureLimit;
    request.direction = dir;
    return HYD_MotionControlFB_ApplyLiveUpdate(fb, &request);
}

static HYD_BOOL applyPressureHandleLiveUpdate(HYD_MotionControlFB* fb,
                                              IEC_WORD execId,
                                              HYD_PRESSUREHANDLE* data__)
{
    HYD_LiveUpdateRequest request;

    if (fb == NULL || data__ == NULL || !__GET_VAR(data__->CONTINUOUSUPDATE)) {
        return true;
    }

    memset(&request, 0, sizeof(request));
    request.flags = HYD_LIVE_UPDATE_TARGET_PRESSURE |
                    HYD_LIVE_UPDATE_PRESSURE_RAMP_RATE |
                    HYD_LIVE_UPDATE_CONTINUOUS_UPDATE;
    request.ownerKind = HYD_DIRECT_CMD_PRESSURE_HANDLE;
    request.ownerTicket = (uint16_t)execId;
    request.targetPressure = __GET_VAR(data__->PRESSURE);
    request.pressureRampRate = __GET_VAR(data__->PRESSURERAMPRATE);
    return HYD_MotionControlFB_ApplyLiveUpdate(fb, &request);
}

static HYD_DirectCommandKind inferAdapterDirectKind(const HYD_MotionSegment* segment) {
    if (segment == NULL) {
        return HYD_DIRECT_CMD_NONE;
    }

    if (segment->mode == HYD_MODE_POSITION &&
        segment->endCondition == HYD_END_POSITION) {
        return HYD_DIRECT_CMD_MOVE_ABSOLUTE;
    }

    if (segment->mode == HYD_MODE_SPEED_RAMP) {
        return HYD_DIRECT_CMD_MOVE_VELOCITY;
    }

    if (segment->mode == HYD_MODE_PRESSURE_CLOSED_LOOP) {
        return HYD_DIRECT_CMD_PRESSURE_HANDLE;
    }

    return HYD_DIRECT_CMD_NONE;
}

static HYD_DirectStartResult startDirectSegmentExecution(HYD_MotionControlFB* fb,
                                                         IEC_INT bufferMode,
                                                         const HYD_MotionSegment* segment,
                                                         IEC_WORD* errorId)
{
    HYD_DirectStartResult result;

    if (errorId != NULL) {
        *errorId = (IEC_WORD)0;
    }

    if (fb == NULL || segment == NULL) {
        if (errorId != NULL) {
            *errorId = (IEC_WORD)HYD_DIAG_CODE_INTERNAL_ERROR;
        }
        return HYD_DIRECT_START_REJECTED;
    }

    result = HYD_MotionControlFB_StartDirectCommand(fb,
                                                    inferAdapterDirectKind(segment),
                                                    segment,
                                                    NULL,
                                                    (HYD_BufferMode)bufferMode,
                                                    fb->AXIS_REF.timestamp);
    if (result == HYD_DIRECT_START_REJECTED) {
        if (errorId != NULL) {
            *errorId = commandFailureErrorId(fb);
        }
    }

    return result;
}

static HYD_DirectStartResult startDirectSegmentExecutionWithKind(HYD_MotionControlFB* fb,
                                                                 HYD_DirectCommandKind kind,
                                                                 HYD_BufferMode bufferMode,
                                                                 const HYD_MotionSegment* segment,
                                                                 const HYD_ContinuousAbsoluteContext* continuousAbsolute,
                                                                 IEC_WORD* errorId)
{
    HYD_DirectStartResult result;

    if (errorId != NULL) {
        *errorId = (IEC_WORD)0;
    }

    if (fb == NULL || segment == NULL) {
        if (errorId != NULL) {
            *errorId = (IEC_WORD)HYD_DIAG_CODE_INTERNAL_ERROR;
        }
        return HYD_DIRECT_START_REJECTED;
    }

    result = HYD_MotionControlFB_StartDirectCommand(fb,
                                                    kind,
                                                    segment,
                                                    continuousAbsolute,
                                                    bufferMode,
                                                    fb->AXIS_REF.timestamp);
    if (result == HYD_DIRECT_START_REJECTED && errorId != NULL) {
        *errorId = commandFailureErrorId(fb);
    }

    return result;
}

/* ======================================================================
 * 框架生命周期函数
 * ====================================================================== */

int __HydMotion_framework_Init()
{
    ++HYD_FrameworkGeneration;
    if (HYD_FrameworkGeneration == 0u) {
        HYD_FrameworkGeneration = 1u;
    }

    for (int i = 0; i < HYD_MAX_AXIS_MOTION; i++) {
        memset(&HYD_MotionControlFB_inst[i], 0, sizeof(HYD_MotionControlFB));
        HYD_AxisSlots[i] = HYD_AXIS_SLOT_FREE;
    }
    HYD_ToggleMechanismPool_Reset();

    return 0;
}

char* __HydMotion_framework_GetVersion()
{
	return (char*)HYD_VERSION_BUILD_TIME;
}

void __HydMotion_framework_Cleanup()
{
    /* 当前无动态资源需要释放 */
}

void __HydMotion_framework_Retrieve()
{
    /* 反馈数据由各命令函数或工艺层直接更新到AXIS_REF */
}

void __HydMotion_framework_Publish()
{
    for (int i = 0; i < HYD_MAX_AXIS_MOTION; i++) {
        if (HYD_AxisSlots[i] != HYD_AXIS_SLOT_ACTIVE) {
            continue;
        }
        HYD_MotionControlFB* fb = &HYD_MotionControlFB_inst[i];
        fb->_simulationCycleTime = (HYD_TIME)dfCycleTime;
        fb->_useFixedCycleTime = true;
    }

    for (int i = 0; i < HYD_MAX_AXIS_MOTION; i++) {
        if (HYD_AxisSlots[i] != HYD_AXIS_SLOT_ACTIVE) {
            continue;
        }
        HYD_MotionControlFB* fb = &HYD_MotionControlFB_inst[i];
        if (fb->_useFixedCycleTime) {
            fb->_simTick++;
        }
        if (!fb->_useSimulation && fb->_useFixedCycleTime) {
            fb->AXIS_REF.timestamp = (HYD_TIME)((double)fb->_simTick *
                                                (double)fb->_simulationCycleTime);
        }
        HYD_MotionControlFB_Scan(fb);

        /* Simulation: close the feedback loop with planner outputs */
        if (fb->_useSimulation && fb->_simFeedback.valid) {
            HYD_TIME simDeltaTime = (fb->_simulationCycleTime > 0.0f)
                ? fb->_simulationCycleTime
                : (HYD_TIME)dfCycleTime;

            if (simDeltaTime > 0.0) {
                fb->AXIS_REF.position += fb->_simFeedback.targetVelocity * simDeltaTime;
            }
            fb->AXIS_REF.velocity  = fb->_simFeedback.targetVelocity;
            fb->AXIS_REF.flow      = fb->_simFeedback.targetFlow;
            fb->AXIS_REF.pressure  = fb->_simFeedback.targetPressure;
            fb->AXIS_REF.timestamp += simDeltaTime;
        }
    }

}

static void clearCreateTransaction(HYD_CREATEMOTION *data__)
{
    __SET_VAR(data__->, _RESERVED_AXIS, , (IEC_SINT)-1);
    __SET_VAR(data__->, _RESERVED_SLOT, , (IEC_USINT)HYD_TOGGLE_SLOT_NONE);
    __SET_VAR(data__->, _VALIDATION_TOKEN, ,
              (IEC_USINT)HYD_TOGGLE_VALIDATION_NONE);
    __SET_VAR(data__->, _CREATE_ACTIVE, , false);
    __SET_VAR(data__->, _FRAMEWORK_GENERATION, , (IEC_WORD)0);
}

static void rollbackCreateTransaction(HYD_CREATEMOTION *data__,
                                      HYD_DiagnosticCode diagnostic)
{
    if (__GET_VAR(data__->_CREATE_ACTIVE) &&
        (__GET_VAR(data__->_FRAMEWORK_GENERATION) ==
         (IEC_WORD)HYD_FrameworkGeneration)) {
        IEC_SINT axis = __GET_VAR(data__->_RESERVED_AXIS);
        IEC_USINT slot = __GET_VAR(data__->_RESERVED_SLOT);
        IEC_USINT token = __GET_VAR(data__->_VALIDATION_TOKEN);

        if (token != (IEC_USINT)HYD_TOGGLE_VALIDATION_NONE) {
            HYD_ToggleMechanismPool_ReleaseValidation((HYD_UINT8)token);
        }
        if (slot != (IEC_USINT)HYD_TOGGLE_SLOT_NONE) {
            HYD_ToggleMechanismPool_Release((HYD_UINT8)slot);
        }
        if ((axis >= 0) && (axis < (IEC_SINT)HYD_MAX_AXIS_MOTION) &&
            (HYD_AxisSlots[(int)axis] == HYD_AXIS_SLOT_RESERVED)) {
            memset(&HYD_MotionControlFB_inst[(int)axis], 0,
                   sizeof(HYD_MotionControlFB_inst[(int)axis]));
            HYD_AxisSlots[(int)axis] = HYD_AXIS_SLOT_FREE;
        }
    }

    clearCreateTransaction(data__);
    __SET_VAR(data__->, DONE, , false);
    __SET_VAR(data__->, BUSY, , false);
    __SET_VAR(data__->, ERROR, , true);
    __SET_VAR(data__->, ERRORID, , (IEC_WORD)diagnostic);
}

static void initializeCreatedAxis(HYD_CREATEMOTION *data__, int axisIndex,
                                  HYD_MechanismType mechanismType,
                                  HYD_UINT8 mechanismSlot)
{
    HYD_MotionControlFB *fb = &HYD_MotionControlFB_inst[axisIndex];

    HYD_MotionControlFB_Init(fb);
    fb->_index = (HYD_UINT8)axisIndex;
    fb->mechanismType = (HYD_UINT8)mechanismType;
    fb->mechanismSlot = mechanismSlot;
    fb->STATE.mechanismType = (HYD_UINT8)mechanismType;
    fb->STATE.actuatorDirection = HYD_DIRECTION_HOLD;
    fb->STATE.mechanismConfigVersion =
        (mechanismSlot == HYD_TOGGLE_SLOT_NONE)
            ? 0u
            : HYD_ToggleMechanismPool_GetVersion(mechanismSlot);
    fb->FB_STATE = HYD_FB_STATE_IDLE;
    fb->USE_RECIPE = __GET_VAR(data__->USE_RECIPE);
    fb->_configuredUseRecipe = fb->USE_RECIPE;
    fb->FLOW_TO_PUMP_SPEED_GAIN = __GET_VAR(data__->FLOW_TO_PUMPSPEED);
    fb->PUMP_SPEED_LIMIT = __GET_VAR(data__->PUMPSPEED_LIMIT);
    fb->_useSimulation = __GET_VAR(data__->USE_SIMULATION);
    fb->_useFixedCycleTime = true;
}

static void completeCreateTransaction(HYD_CREATEMOTION *data__, int axisIndex)
{
    HYD_AxisSlots[axisIndex] = HYD_AXIS_SLOT_ACTIVE;
    clearCreateTransaction(data__);
    __SET_VAR(data__->, AXISID, , (IEC_SINT)axisIndex);
    __SET_VAR(data__->, DONE, , true);
    __SET_VAR(data__->, BUSY, , false);
    __SET_VAR(data__->, ERROR, , false);
    __SET_VAR(data__->, ERRORID, , (IEC_WORD)HYD_DIAG_CODE_NONE);
}

void __mcl_cmd_CreateMotion(HYD_CREATEMOTION *data__)
{
    HYD_MechanismType mechanismType;
    IEC_SINT axisIndex;
    IEC_USINT slot;
    IEC_USINT token;
    HYD_ToggleValidation *validation;
    HYD_ToggleError toggleError = HYD_TOGGLE_ERROR_NONE;

    if (__GET_VAR(data__->DONE) || __GET_VAR(data__->ERROR)) {
        return;
    }

    if (__GET_VAR(data__->_CREATE_ACTIVE) &&
        (__GET_VAR(data__->_FRAMEWORK_GENERATION) !=
         (IEC_WORD)HYD_FrameworkGeneration)) {
        rollbackCreateTransaction(data__,
                                  HYD_DIAG_CODE_MECHANISM_CONFIG_BUSY);
        return;
    }

    mechanismType = (HYD_MechanismType)__GET_VAR(data__->MECHANISM_TYPE);
    if (!__GET_VAR(data__->_CREATE_ACTIVE)) {
        int reservedAxis;

        if ((mechanismType != HYD_MECHANISM_DIRECT) &&
            (mechanismType != HYD_MECHANISM_FIVE_POINT_TOGGLE)) {
            rollbackCreateTransaction(data__,
                                      HYD_DIAG_CODE_MECHANISM_TYPE_INVALID);
            return;
        }

        reservedAxis = allocMotionControlFB();
        if (reservedAxis < 0) {
            rollbackCreateTransaction(data__, HYD_DIAG_CODE_INTERNAL_ERROR);
            return;
        }

        __SET_VAR(data__->, _CREATE_ACTIVE, , true);
        __SET_VAR(data__->, _FRAMEWORK_GENERATION, ,
                  (IEC_WORD)HYD_FrameworkGeneration);
        __SET_VAR(data__->, _RESERVED_AXIS, , (IEC_SINT)reservedAxis);
        __SET_VAR(data__->, _RESERVED_SLOT, ,
                  (IEC_USINT)HYD_TOGGLE_SLOT_NONE);
        __SET_VAR(data__->, _VALIDATION_TOKEN, ,
                  (IEC_USINT)HYD_TOGGLE_VALIDATION_NONE);

        if (mechanismType == HYD_MECHANISM_DIRECT) {
            initializeCreatedAxis(data__, reservedAxis, mechanismType,
                                  HYD_TOGGLE_SLOT_NONE);
            completeCreateTransaction(data__, reservedAxis);
            return;
        }

        {
            HYD_UINT8 reservedSlot;
            HYD_UINT8 validationToken;
            HYD_ToggleGeometryConfig config;
            HYD_ToggleValidationLimits limits;

            if (!HYD_ToggleMechanismPool_Reserve((HYD_UINT8)reservedAxis,
                                                 &reservedSlot)) {
                rollbackCreateTransaction(
                    data__, HYD_DIAG_CODE_MECHANISM_POOL_EXHAUSTED);
                return;
            }
            __SET_VAR(data__->, _RESERVED_SLOT, , (IEC_USINT)reservedSlot);

            if (!HYD_ToggleMechanismPool_AcquireValidation(
                    &validationToken)) {
                rollbackCreateTransaction(
                    data__, HYD_DIAG_CODE_MECHANISM_VALIDATION_BUSY);
                return;
            }
            __SET_VAR(data__->, _VALIDATION_TOKEN, ,
                      (IEC_USINT)validationToken);

            validation = HYD_ToggleMechanismPool_GetValidation(
                validationToken);
            config = HYD_ToggleKinematics_DefaultConfig();
            limits = HYD_ToggleKinematics_DefaultValidationLimits();
            if ((validation == NULL) ||
                !HYD_ToggleKinematics_BeginValidation(
                    &config, &limits, validation, &toggleError)) {
                rollbackCreateTransaction(
                    data__, HYD_DIAG_CODE_MECHANISM_CONFIG_INVALID);
                return;
            }
        }

        __SET_VAR(data__->, BUSY, , true);
        __SET_VAR(data__->, ERROR, , false);
        __SET_VAR(data__->, ERRORID, , (IEC_WORD)HYD_DIAG_CODE_NONE);
    }

    axisIndex = __GET_VAR(data__->_RESERVED_AXIS);
    slot = __GET_VAR(data__->_RESERVED_SLOT);
    token = __GET_VAR(data__->_VALIDATION_TOKEN);
    validation = HYD_ToggleMechanismPool_GetValidation((HYD_UINT8)token);
    if ((axisIndex < 0) || (axisIndex >= (IEC_SINT)HYD_MAX_AXIS_MOTION) ||
        (slot == (IEC_USINT)HYD_TOGGLE_SLOT_NONE) ||
        (validation == NULL)) {
        rollbackCreateTransaction(data__, HYD_DIAG_CODE_INTERNAL_ERROR);
        return;
    }

    if (!HYD_ToggleKinematics_ValidationStep(validation, 4u,
                                             &toggleError)) {
        rollbackCreateTransaction(data__,
                                  HYD_DIAG_CODE_MECHANISM_CONFIG_INVALID);
        return;
    }

    if (!HYD_ToggleKinematics_ValidationDone(validation)) {
        __SET_VAR(data__->, BUSY, , true);
        return;
    }

    {
        HYD_TogglePreparedConfig prepared;

        if (!HYD_ToggleKinematics_FinishValidation(validation, &prepared,
                                                   &toggleError) ||
            !HYD_ToggleMechanismPool_Commit((HYD_UINT8)slot, &prepared,
                                            true)) {
            rollbackCreateTransaction(data__,
                                      HYD_DIAG_CODE_MECHANISM_CONFIG_INVALID);
            return;
        }
    }

    HYD_ToggleMechanismPool_ReleaseValidation((HYD_UINT8)token);
    __SET_VAR(data__->, _VALIDATION_TOKEN, ,
              (IEC_USINT)HYD_TOGGLE_VALIDATION_NONE);
    initializeCreatedAxis(data__, (int)axisIndex,
                          HYD_MECHANISM_FIVE_POINT_TOGGLE, (HYD_UINT8)slot);
    completeCreateTransaction(data__, (int)axisIndex);
}

static HYD_BOOL toggleConfigurationStateAllowed(
    const HYD_MotionControlFB *fb)
{
    if ((fb == NULL) || fb->STATE.active) {
        return false;
    }

    switch (fb->FB_STATE) {
        case HYD_FB_STATE_IDLE:
        case HYD_FB_STATE_READY:
        case HYD_FB_STATE_DONE:
        case HYD_FB_STATE_ABORTED:
            return true;
        default:
            return false;
    }
}

static void clearToggleConfigurationTransaction(
    HYD_CONFIGURETOGGLEMECHANISM *data__, HYD_BOOL releaseWorkspace)
{
    if (releaseWorkspace && __GET_VAR(data__->ACTIVE) &&
        (__GET_VAR(data__->_FRAMEWORK_GENERATION) ==
         (IEC_WORD)HYD_FrameworkGeneration)) {
        IEC_USINT token = __GET_VAR(data__->VALIDATION_TOKEN);

        if (token != (IEC_USINT)HYD_TOGGLE_VALIDATION_NONE) {
            HYD_ToggleMechanismPool_ReleaseValidation((HYD_UINT8)token);
        }
    }

    __SET_VAR(data__->, VALIDATION_TOKEN, ,
              (IEC_USINT)HYD_TOGGLE_VALIDATION_NONE);
    __SET_VAR(data__->, ACTIVE, , false);
    __SET_VAR(data__->, CONFIG_AXIS, , (IEC_SINT)-1);
    __SET_VAR(data__->, _FRAMEWORK_GENERATION, , (IEC_WORD)0);
}

static IEC_WORD currentToggleConfigurationVersion(
    const HYD_CONFIGURETOGGLEMECHANISM *data__)
{
    IEC_SINT axisIndex = __GET_VAR(data__->ACTIVE)
        ? __GET_VAR(data__->CONFIG_AXIS)
        : __GET_VAR(data__->AXISID);
    HYD_MotionControlFB *fb = __MK_GetPublic_MotionControlFB((int)axisIndex);

    if ((fb == NULL) ||
        (fb->mechanismType !=
         (HYD_UINT8)HYD_MECHANISM_FIVE_POINT_TOGGLE) ||
        (fb->mechanismSlot == HYD_TOGGLE_SLOT_NONE)) {
        return (IEC_WORD)0;
    }

    return (IEC_WORD)HYD_ToggleMechanismPool_GetVersion(fb->mechanismSlot);
}

static void failToggleConfiguration(HYD_CONFIGURETOGGLEMECHANISM *data__,
                                    HYD_DiagnosticCode diagnostic,
                                    HYD_BOOL releaseWorkspace)
{
    IEC_WORD version = currentToggleConfigurationVersion(data__);

    clearToggleConfigurationTransaction(data__, releaseWorkspace);
    __SET_VAR(data__->, DONE, , false);
    __SET_VAR(data__->, BUSY, , false);
    __SET_VAR(data__->, ERROR, , true);
    __SET_VAR(data__->, ERRORID, , (IEC_WORD)diagnostic);
    __SET_VAR(data__->, CONFIG_VERSION, , version);
}

static HYD_ToggleGeometryConfig toggleConfigurationInputs(
    const HYD_CONFIGURETOGGLEMECHANISM *data__)
{
    HYD_ToggleGeometryConfig raw;

    raw.lr = (HYD_REAL)__GET_VAR(data__->LR);
    raw.lf = (HYD_REAL)__GET_VAR(data__->LF);
    raw.lpf = (HYD_REAL)__GET_VAR(data__->LPF);
    raw.lpk = (HYD_REAL)__GET_VAR(data__->LPK);
    raw.ld = (HYD_REAL)__GET_VAR(data__->LD);
    raw.hf = (HYD_REAL)__GET_VAR(data__->HF);
    raw.hm = (HYD_REAL)__GET_VAR(data__->HM);
    raw.dc = (HYD_REAL)__GET_VAR(data__->DC);
    raw.sm = (HYD_REAL)__GET_VAR(data__->SM);
    raw.xHandoff = (HYD_REAL)__GET_VAR(data__->XHANDOFF);
    raw.sigmaK = (int8_t)__GET_VAR(data__->SIGMA_K);
    raw.signB = (int8_t)__GET_VAR(data__->SIGN_B);
    raw.tauS = (int8_t)__GET_VAR(data__->TAU_S);
    raw.sigmaC = (int8_t)__GET_VAR(data__->SIGMA_C);
    return raw;
}

void __mcl_cmd_ConfigureToggleMechanism(
    HYD_CONFIGURETOGGLEMECHANISM *data__)
{
    IEC_BOOL execute = __GET_VAR(data__->EXECUTE);
    IEC_BOOL rising = execute && !__GET_VAR(data__->EXECUTE0);
    HYD_ToggleValidation *validation;
    HYD_ToggleError toggleError = HYD_TOGGLE_ERROR_NONE;
    HYD_MotionControlFB *fb;
    IEC_SINT axisIndex;

    if (!execute) {
        clearToggleConfigurationTransaction(data__, true);
        __SET_VAR(data__->, DONE, , false);
        __SET_VAR(data__->, BUSY, , false);
        __SET_VAR(data__->, ERROR, , false);
        __SET_VAR(data__->, ERRORID, , (IEC_WORD)HYD_DIAG_CODE_NONE);
        __SET_VAR(data__->, EXECUTE0, , false);
        return;
    }

    if (__GET_VAR(data__->ACTIVE) &&
        (__GET_VAR(data__->_FRAMEWORK_GENERATION) !=
         (IEC_WORD)HYD_FrameworkGeneration)) {
        failToggleConfiguration(data__, HYD_DIAG_CODE_MECHANISM_CONFIG_BUSY,
                                false);
        __SET_VAR(data__->, EXECUTE0, , true);
        return;
    }

    if (rising) {
        HYD_UINT8 validationToken;
        HYD_ToggleGeometryConfig raw;
        HYD_ToggleValidationLimits limits;

        axisIndex = __GET_VAR(data__->AXISID);
        fb = __MK_GetPublic_MotionControlFB((int)axisIndex);
        if (fb == NULL) {
            failToggleConfiguration(data__,
                                    HYD_DIAG_CODE_START_CONTEXT_INVALID,
                                    false);
            __SET_VAR(data__->, EXECUTE0, , true);
            return;
        }
        if ((fb->mechanismType !=
             (HYD_UINT8)HYD_MECHANISM_FIVE_POINT_TOGGLE) ||
            (fb->mechanismSlot == HYD_TOGGLE_SLOT_NONE)) {
            failToggleConfiguration(data__,
                                    HYD_DIAG_CODE_MECHANISM_TYPE_INVALID,
                                    false);
            __SET_VAR(data__->, EXECUTE0, , true);
            return;
        }
        __SET_VAR(data__->, CONFIG_VERSION, ,
                  (IEC_WORD)HYD_ToggleMechanismPool_GetVersion(
                      fb->mechanismSlot));
        fb->STATE.mechanismConfigVersion =
            HYD_ToggleMechanismPool_GetVersion(fb->mechanismSlot);
        if (!toggleConfigurationStateAllowed(fb)) {
            failToggleConfiguration(data__,
                                    HYD_DIAG_CODE_MECHANISM_CONFIG_BUSY,
                                    false);
            __SET_VAR(data__->, EXECUTE0, , true);
            return;
        }
        if (!HYD_ToggleMechanismPool_AcquireValidation(&validationToken)) {
            failToggleConfiguration(
                data__, HYD_DIAG_CODE_MECHANISM_VALIDATION_BUSY, false);
            __SET_VAR(data__->, EXECUTE0, , true);
            return;
        }

        __SET_VAR(data__->, VALIDATION_TOKEN, ,
                  (IEC_USINT)validationToken);
        __SET_VAR(data__->, ACTIVE, , true);
        __SET_VAR(data__->, CONFIG_AXIS, , axisIndex);
        __SET_VAR(data__->, _FRAMEWORK_GENERATION, ,
                  (IEC_WORD)HYD_FrameworkGeneration);
        validation = HYD_ToggleMechanismPool_GetValidation(validationToken);
        raw = toggleConfigurationInputs(data__);
        limits = HYD_ToggleKinematics_DefaultValidationLimits();
        if ((validation == NULL) ||
            !HYD_ToggleKinematics_BeginValidation(
                &raw, &limits, validation, &toggleError)) {
            failToggleConfiguration(
                data__, HYD_DIAG_CODE_MECHANISM_CONFIG_INVALID, true);
            __SET_VAR(data__->, EXECUTE0, , true);
            return;
        }

        __SET_VAR(data__->, DONE, , false);
        __SET_VAR(data__->, BUSY, , true);
        __SET_VAR(data__->, ERROR, , false);
        __SET_VAR(data__->, ERRORID, , (IEC_WORD)HYD_DIAG_CODE_NONE);
    }

    if (!__GET_VAR(data__->ACTIVE)) {
        __SET_VAR(data__->, EXECUTE0, , true);
        return;
    }

    validation = HYD_ToggleMechanismPool_GetValidation(
        (HYD_UINT8)__GET_VAR(data__->VALIDATION_TOKEN));
    if ((validation == NULL) ||
        !HYD_ToggleKinematics_ValidationStep(validation, 4u,
                                             &toggleError)) {
        failToggleConfiguration(data__,
                                HYD_DIAG_CODE_MECHANISM_CONFIG_INVALID,
                                true);
        __SET_VAR(data__->, EXECUTE0, , true);
        return;
    }

    if (HYD_ToggleKinematics_ValidationDone(validation)) {
        HYD_TogglePreparedConfig prepared;

        axisIndex = __GET_VAR(data__->CONFIG_AXIS);
        fb = __MK_GetPublic_MotionControlFB((int)axisIndex);
        if ((fb == NULL) ||
            (fb->mechanismType !=
             (HYD_UINT8)HYD_MECHANISM_FIVE_POINT_TOGGLE) ||
            !toggleConfigurationStateAllowed(fb) ||
            !HYD_ToggleKinematics_FinishValidation(validation, &prepared,
                                                   &toggleError) ||
            !HYD_ToggleMechanismPool_Commit(fb->mechanismSlot, &prepared,
                                            false)) {
            failToggleConfiguration(
                data__, HYD_DIAG_CODE_MECHANISM_CONFIG_INVALID, true);
            __SET_VAR(data__->, EXECUTE0, , true);
            return;
        }

        clearToggleConfigurationTransaction(data__, true);
        fb->STATE.mechanismConfigVersion =
            HYD_ToggleMechanismPool_GetVersion(fb->mechanismSlot);
        __SET_VAR(data__->, CONFIG_VERSION, ,
                  (IEC_WORD)HYD_ToggleMechanismPool_GetVersion(
                      fb->mechanismSlot));
        __SET_VAR(data__->, DONE, , true);
        __SET_VAR(data__->, BUSY, , false);
        __SET_VAR(data__->, ERROR, , false);
        __SET_VAR(data__->, ERRORID, , (IEC_WORD)HYD_DIAG_CODE_NONE);
    } else {
        __SET_VAR(data__->, BUSY, , true);
    }

    __SET_VAR(data__->, EXECUTE0, , true);
}

void __mcl_cmd_ReadToggleMechanism(HYD_READTOGGLEMECHANISM *data__)
{
    IEC_SINT axisIndex = __GET_VAR(data__->AXISID);
    HYD_MotionControlFB *fb;
    const HYD_TogglePreparedConfig *prepared;
    const HYD_ToggleGeometryConfig *raw;

    if (!__GET_VAR(data__->ENABLE)) {
        __SET_VAR(data__->, VALID, , false);
        __SET_VAR(data__->, ERROR, , false);
        __SET_VAR(data__->, ERRORID, , (IEC_WORD)HYD_DIAG_CODE_NONE);
        return;
    }

    fb = __MK_GetPublic_MotionControlFB((int)axisIndex);
    if (fb == NULL) {
        __SET_VAR(data__->, VALID, , false);
        __SET_VAR(data__->, ERROR, , true);
        __SET_VAR(data__->, ERRORID, ,
                  (IEC_WORD)HYD_DIAG_CODE_START_CONTEXT_INVALID);
        return;
    }
    if ((fb->mechanismType !=
         (HYD_UINT8)HYD_MECHANISM_FIVE_POINT_TOGGLE) ||
        (fb->mechanismSlot == HYD_TOGGLE_SLOT_NONE)) {
        __SET_VAR(data__->, VALID, , false);
        __SET_VAR(data__->, ERROR, , true);
        __SET_VAR(data__->, ERRORID, ,
                  (IEC_WORD)HYD_DIAG_CODE_MECHANISM_TYPE_INVALID);
        return;
    }

    prepared = HYD_ToggleMechanismPool_GetPrepared(fb->mechanismSlot);
    raw = HYD_ToggleMechanismPool_GetRaw(fb->mechanismSlot);
    if ((prepared == NULL) || (raw == NULL)) {
        __SET_VAR(data__->, VALID, , false);
        __SET_VAR(data__->, ERROR, , true);
        __SET_VAR(data__->, ERRORID, ,
                  (IEC_WORD)HYD_DIAG_CODE_MECHANISM_CONFIG_INVALID);
        return;
    }

    __SET_VAR(data__->, LR, , (IEC_LREAL)raw->lr);
    __SET_VAR(data__->, LF, , (IEC_LREAL)raw->lf);
    __SET_VAR(data__->, LPF, , (IEC_LREAL)raw->lpf);
    __SET_VAR(data__->, LPK, , (IEC_LREAL)raw->lpk);
    __SET_VAR(data__->, LD, , (IEC_LREAL)raw->ld);
    __SET_VAR(data__->, HF, , (IEC_LREAL)raw->hf);
    __SET_VAR(data__->, HM, , (IEC_LREAL)raw->hm);
    __SET_VAR(data__->, DC, , (IEC_LREAL)raw->dc);
    __SET_VAR(data__->, SM, , (IEC_LREAL)raw->sm);
    __SET_VAR(data__->, XHANDOFF, , (IEC_LREAL)raw->xHandoff);
    __SET_VAR(data__->, SIGMA_K, , (IEC_SINT)raw->sigmaK);
    __SET_VAR(data__->, SIGN_B, , (IEC_SINT)raw->signB);
    __SET_VAR(data__->, TAU_S, , (IEC_SINT)raw->tauS);
    __SET_VAR(data__->, SIGMA_C, , (IEC_SINT)raw->sigmaC);
    __SET_VAR(data__->, CONFIG_VERSION, ,
              (IEC_WORD)HYD_ToggleMechanismPool_GetVersion(
                  fb->mechanismSlot));
    __SET_VAR(data__->, X_GEOMETRY_MIN, ,
              (IEC_LREAL)prepared->xGeometryMin);
    __SET_VAR(data__->, X_HANDOFF_EFFECTIVE, ,
              (IEC_LREAL)prepared->xHandoffEffective);
    __SET_VAR(data__->, XS_MIN, , (IEC_LREAL)prepared->xsMin);
    __SET_VAR(data__->, XS_MAX, , (IEC_LREAL)prepared->xsMax);
    __SET_VAR(data__->, K_MIN, , (IEC_LREAL)prepared->kMin);
    __SET_VAR(data__->, K_MAX, , (IEC_LREAL)prepared->kMax);
    __SET_VAR(data__->, USING_DEFAULTS, ,
              HYD_ToggleMechanismPool_UsingDefaults(fb->mechanismSlot));
    __SET_VAR(data__->, VALID, , true);
    __SET_VAR(data__->, ERROR, , false);
    __SET_VAR(data__->, ERRORID, , (IEC_WORD)HYD_DIAG_CODE_NONE);
}

/* ======================================================================
 * MoveProfile (Recipe模式) 命令实现
 * ====================================================================== */

void __mcl_cmd_LoadProfile(HYD_LOADPROFILE *data__)
{
    IEC_SINT axisIndex = __GET_VAR(data__->AXISID);
    HYD_MotionControlFB* fb = __MK_GetPublic_MotionControlFB(axisIndex);
    IEC_BOOL execute = __GET_VAR(data__->EXECUTE);
    IEC_BOOL execRising = execute && !__GET_VAR(data__->EXECUTE0);

    if (fb == NULL) {
        __SET_VAR(data__->, ERROR,, true);
        __SET_VAR(data__->, ERRORID,, (IEC_WORD)HYD_DIAG_CODE_START_CONTEXT_INVALID);
        __SET_VAR(data__->, EXECUTE0,, execute);
        return;
    }

    if (!execute) {
        __SET_VAR(data__->, BUSY,, false);
        __SET_VAR(data__->, EXECUTE0,, execute);
        return;
    }

    if (execRising) {
        HYD_AXISMOTION motionData = __GET_VAR(data__->MOTION);
        HYD_MotionSegment segment = buildSegmentFromMotion(&motionData, fb);
        HYD_BOOL ok;

        if (fb->_configuredUseRecipe) {
            ok = HYD_MotionControlFB_LoadRecipe(fb, &segment, 1U);
        } else {
            ok = HYD_MotionControlFB_LoadDirectSegment(fb, &segment);
        }

        if (!ok) {
            __SET_VAR(data__->, ERROR,, true);
            __SET_VAR(data__->, ERRORID,, (IEC_WORD)HYD_DIAG_CODE_SEGMENT_INVALID);
            __SET_VAR(data__->, EXECUTE0,, execute);
            return;
        }

        __SET_VAR(data__->, DONE,, true);
        __SET_VAR(data__->, BUSY,, false);
        __SET_VAR(data__->, ERROR,, false);
        __SET_VAR(data__->, ERRORID,, (IEC_WORD)0);
    }

    __SET_VAR(data__->, EXECUTE0,, execute);
}

void __mcl_cmd_MoveProfile(HYD_MOVEPROFILE *data__)
{
    IEC_SINT axisIndex = __GET_VAR(data__->AXISID);
    HYD_MotionControlFB *fb = __MK_GetPublic_MotionControlFB(axisIndex);
    HYD_AXISMOTION motionData;
    IEC_INT bufferMode;
    IEC_BOOL isPending;
    IEC_WORD myExecId;

    if (fb == NULL) {
        __SET_VAR(data__->, ERROR,, true);
        __SET_VAR(data__->, ERRORID,, (IEC_WORD)HYD_DIAG_CODE_INTERNAL_ERROR);
        return;
    }

    IEC_BOOL execute = __GET_VAR(data__->EXECUTE);
    IEC_BOOL execRising = execute && !__GET_VAR(data__->EXECUTE0);

    /* In simulation the FB owns feedback progression; otherwise feedback arrives via MOTION.ACT*. */
    motionData = __GET_VAR(data__->MOTION);
    if (!fb->_useSimulation) {
        fb->AXIS_REF.position = motionData.ACTPOSITION;
        fb->AXIS_REF.velocity = motionData.ACTVELOCITY;
        fb->AXIS_REF.flow = motionData.ACTFLOW;
        fb->AXIS_REF.pressure = motionData.ACTPRESSURE;
    } else {
        motionData.ACTPOSITION = (IEC_REAL)fb->AXIS_REF.position;
        motionData.ACTVELOCITY = (IEC_REAL)fb->AXIS_REF.velocity;
        motionData.ACTFLOW = (IEC_REAL)fb->AXIS_REF.flow;
        motionData.ACTPRESSURE = (IEC_REAL)fb->AXIS_REF.pressure;
        motionData.TIMESTAMP = (IEC_REAL)fb->AXIS_REF.timestamp;
        __SET_VAR(data__->, MOTION,, motionData);
    }

    bufferMode = __GET_VAR(data__->BUFFERMODE);
    isPending = __GET_VAR(data__->_PENDING);
    myExecId = __GET_VAR(data__->_EXEC_ID);

    if (!execute) {
        __SET_VAR(data__->, DONE,, false);
        __SET_VAR(data__->, COMMANDABORTED,, false);
        __SET_VAR(data__->, ERROR,, false);
        __SET_VAR(data__->, ERRORID,, (IEC_WORD)0);
        __SET_VAR(data__->, BUSY,, false);
        __SET_VAR(data__->, ACTIVE,, false);
        __SET_VAR(data__->, _PENDING,, false);
        __SET_VAR(data__->, _EXEC_ID,, (IEC_WORD)0);
        __SET_VAR(data__->, ACTIVE0,, false);
        __SET_VAR(data__->, EXECUTE0,, execute);
        return;
    }

    if (execRising) {
        IEC_WORD errorId = 0;
        if (!validateSupportedBufferMode(bufferMode, &errorId)) {
            __SET_VAR(data__->, ERROR,, true);
            __SET_VAR(data__->, ERRORID,, errorId);
            __SET_VAR(data__->, EXECUTE0,, execute);
            return;
        }

        fb->USE_RECIPE = true;

        if (bufferMode == HYD_BUFFER_MODE_ABORT) {
            HYD_MotionControlFB_Abort(fb);
            HYD_MotionControlFB_Scan(fb);
        }

        /* Build 1-segment recipe from MOTION if no preloaded recipe */
        if (fb->RECIPE_SIZE == 0 && !fb->DIRECT_SEGMENT_VALID) {
            HYD_MotionSegment segment = buildSegmentFromMotion(&motionData, fb);
            if (!HYD_MotionControlFB_LoadRecipe(fb, &segment, 1)) {
                __SET_VAR(data__->, ERROR,, true);
                __SET_VAR(data__->, ERRORID,, (IEC_WORD)HYD_DIAG_CODE_SEGMENT_INVALID);
                __SET_VAR(data__->, EXECUTE0,, execute);
                return;
            }
        }

        /* Start segment (recipe or direct) */
        if (!HYD_MotionControlFB_StartSegment(fb, 0, fb->AXIS_REF.timestamp)) {
            __SET_VAR(data__->, ERROR,, true);
            __SET_VAR(data__->, ERRORID,, (IEC_WORD)HYD_DIAG_CODE_START_CONTEXT_INVALID);
            __SET_VAR(data__->, EXECUTE0,, execute);
            return;
        }

        __SET_VAR(data__->, _PENDING,, true);
        __SET_VAR(data__->, _EXEC_ID,, (IEC_WORD)0);
        __SET_VAR(data__->, ACTIVE,, true);
        __SET_VAR(data__->, BUSY,, true);
        __SET_VAR(data__->, DONE,, false);
        __SET_VAR(data__->, COMMANDABORTED,, false);
        __SET_VAR(data__->, ERROR,, false);
        __SET_VAR(data__->, ERRORID,, (IEC_WORD)0);
        __SET_VAR(data__->, EXECUTE0,, execute);
        return;
    }

    /* Ownership tracking — MoveProfile uses _recipeBatchId (recipe-side
     * batch epoch). _executionId is the per-segment epoch used by direct
     * commands; using it here would falsely flag every NextSegment as a
     * COMMANDABORTED event. See motion_control.h field comments. */
    if (isPending) {
        if (recipeExecutionCanAcquireOwnership(fb)) {
            __SET_VAR(data__->, _EXEC_ID,, (IEC_WORD)fb->_recipeBatchId);
            __SET_VAR(data__->, _PENDING,, false);
            myExecId = (IEC_WORD)fb->_recipeBatchId;
        } else if (recipeExecutionWasTakenOverBeforeLatch(fb) ||
                   (fb->_pendingCommand != HYD_CMD_START &&
                    fb->FB_STATE == HYD_FB_STATE_ABORTED && !fb->STATE.active) ||
                   (fb->_pendingCommand != HYD_CMD_START &&
                    fb->_recipeBatchId == 0U &&
                    !fb->STATE.active &&
                    !HYD_MotionControlFB_IsBusy(fb) &&
                    !HYD_MotionControlFB_IsDone(fb) &&
                    !fb->STATE.finished)) {
            __SET_VAR(data__->, COMMANDABORTED,, true);
            __SET_VAR(data__->, ACTIVE,, false);
            __SET_VAR(data__->, BUSY,, false);
            __SET_VAR(data__->, DONE,, false);
            __SET_VAR(data__->, ERROR,, false);
            __SET_VAR(data__->, ERRORID,, (IEC_WORD)0);
            __SET_VAR(data__->, _PENDING,, false);
            __SET_VAR(data__->, _EXEC_ID,, (IEC_WORD)0);
            __SET_VAR(data__->, EXECUTE0,, execute);
            return;
        }
    }

    if (myExecId != 0) {
        if (recipeExecutionLostOwnership(fb, myExecId)) {
            __SET_VAR(data__->, COMMANDABORTED,, true);
            __SET_VAR(data__->, ACTIVE,, false);
            __SET_VAR(data__->, BUSY,, false);
            __SET_VAR(data__->, DONE,, false);
            __SET_VAR(data__->, ERROR,, false);
            __SET_VAR(data__->, ERRORID,, (IEC_WORD)0);
            __SET_VAR(data__->, _EXEC_ID,, (IEC_WORD)0);
        } else {
            __SET_VAR(data__->, COMMANDABORTED,, false);
            __SET_VAR(data__->, ACTIVE,, fb->STATE.active ? true : false);
            __SET_VAR(data__->, BUSY,, HYD_MotionControlFB_IsBusy(fb));
            __SET_VAR(data__->, DONE,, (HYD_MotionControlFB_IsDone(fb) && fb->STATE.finished) ? true : false);
            __SET_VAR(data__->, ERROR,, HYD_MotionControlFB_IsError(fb) ? true : false);
            __SET_VAR(data__->, ERRORID,, (IEC_WORD)fb->ERROR_ID);

            /* Setpoint half is PLC-owned -- runtime MUST NOT write back
             * SEGMENT*, MODE, ENDCONDITION, DIRECTION, PLANNER, SET*,
             * ACCELERATION, DECELERATION, DURATION, PRESSURERAMPRATE
             * here. Doing so would clobber whatever the PLC has staged
             * for a later FB on the same axis. See ownership contract
             * note above writeMotionFromSegment() removal. */
        }
    }

    __SET_VAR(data__->, ACTIVE0,, __GET_VAR(data__->ACTIVE));
    __SET_VAR(data__->, EXECUTE0,, execute);
}

/* ======================================================================
 * Stop (Direct模式) 命令实现
 * ====================================================================== */

void __mcl_cmd_Stop(HYD_STOP *data__)
{
    IEC_BOOL execute = __GET_VAR(data__->EXECUTE);
    IEC_BOOL execRising = execute && !__GET_VAR(data__->EXECUTE0);
    IEC_SINT axisIndex = __GET_VAR(data__->AXISID);

    if (!axisIndexIsAllocated(axisIndex))
    {
        __SET_VAR(data__->, ERROR, , true);
        __SET_VAR(data__->, ERRORID, , (IEC_WORD)HYD_DIAG_CODE_START_CONTEXT_INVALID);
        __SET_VAR(data__->, EXECUTE0, , execute);
        return;
    }

    HYD_MotionControlFB* fb = &HYD_MotionControlFB_inst[axisIndex];
    IEC_BOOL isPending = __GET_VAR(data__->_PENDING);

    if (!execute)
    {
        __SET_VAR(data__->, DONE, , false);
        __SET_VAR(data__->, COMMANDABORTED, , false);
        __SET_VAR(data__->, ERROR, , false);
        __SET_VAR(data__->, ERRORID, , (IEC_WORD)0);
        __SET_VAR(data__->, BUSY, , false);
        __SET_VAR(data__->, _PENDING, , false);
        __SET_VAR(data__->, EXECUTE0, , execute);
        return;
    }

    if (execRising)
    {
        HYD_MotionControlFB_Scan(fb);

        if (directStopCanCompleteImmediately(fb)) {
            __SET_VAR(data__->, DONE, , true);
            __SET_VAR(data__->, BUSY, , false);
            __SET_VAR(data__->, EXECUTE0, , execute);
            return;
        }

        HYD_MotionControlFB_Stop(fb,
                                 fb->AXIS_REF.timestamp,
                                 __GET_VAR(data__->DECELERATION));
        HYD_MotionControlFB_Cycle(fb);

        __SET_VAR(data__->, _PENDING, , true);
        __SET_VAR(data__->, BUSY, , true);
        __SET_VAR(data__->, DONE, , false);
        __SET_VAR(data__->, COMMANDABORTED, , false);
        __SET_VAR(data__->, EXECUTE0, , execute);
        return;
    }

    if (isPending)
    {
        if (HYD_MotionControlFB_IsError(fb))
        {
            __SET_VAR(data__->, ERROR, , true);
            __SET_VAR(data__->, ERRORID, , (IEC_WORD)fb->ERROR_ID);
            __SET_VAR(data__->, BUSY, , false);
            __SET_VAR(data__->, _PENDING, , false);
        }
        else if (HYD_MotionControlFB_GetDirectOwnerKind(fb) == HYD_DIRECT_CMD_STOP &&
                 HYD_MotionControlFB_GetDirectSessionState(fb) == HYD_DIRECT_SESSION_DONE)
        {
            __SET_VAR(data__->, DONE, , true);
            __SET_VAR(data__->, BUSY, , false);
            __SET_VAR(data__->, _PENDING, , false);
        }
        else
        {
            __SET_VAR(data__->, BUSY, , true);
        }
    }

    __SET_VAR(data__->, EXECUTE0, , execute);
}

void __mcl_cmd_Hold(HYD_HOLD *data__)
{
    IEC_BOOL execute = __GET_VAR(data__->EXECUTE);
    IEC_BOOL execRising = execute && !__GET_VAR(data__->EXECUTE0);
    IEC_SINT axisIndex = __GET_VAR(data__->AXISID);

    if (!axisIndexIsAllocated(axisIndex))
    {
        __SET_VAR(data__->, ERROR, , true);
        __SET_VAR(data__->, ERRORID, , (IEC_WORD)HYD_DIAG_CODE_START_CONTEXT_INVALID);
        __SET_VAR(data__->, BUSY, , false);
        __SET_VAR(data__->, EXECUTE0, , execute);
        return;
    }

    HYD_MotionControlFB* fb = &HYD_MotionControlFB_inst[axisIndex];
    IEC_BOOL isPending = __GET_VAR(data__->_PENDING);

    if (!execute)
    {
        __SET_VAR(data__->, DONE, , false);
        __SET_VAR(data__->, ERROR, , false);
        __SET_VAR(data__->, ERRORID, , (IEC_WORD)0);
        __SET_VAR(data__->, BUSY, , false);
        __SET_VAR(data__->, _PENDING, , false);
        __SET_VAR(data__->, EXECUTE0, , execute);
        return;
    }

    if (execRising)
    {
        if (!HYD_MotionControlFB_Hold(fb))
        {
            __SET_VAR(data__->, ERROR, , true);
            __SET_VAR(data__->, ERRORID, , commandFailureErrorId(fb));
            __SET_VAR(data__->, BUSY, , false);
            __SET_VAR(data__->, EXECUTE0, , execute);
            return;
        }

        __SET_VAR(data__->, DONE, , false);
        __SET_VAR(data__->, BUSY, , true);
        __SET_VAR(data__->, ERROR, , false);
        __SET_VAR(data__->, ERRORID, , (IEC_WORD)0);
        __SET_VAR(data__->, _PENDING, , true);
        __SET_VAR(data__->, EXECUTE0, , execute);
        return;
    }

    if (isPending)
    {
        if (HYD_MotionControlFB_IsError(fb))
        {
            __SET_VAR(data__->, ERROR, , true);
            __SET_VAR(data__->, ERRORID, , (IEC_WORD)fb->ERROR_ID);
            __SET_VAR(data__->, BUSY, , false);
            __SET_VAR(data__->, _PENDING, , false);
        }
        else if (fb->FB_STATE == HYD_FB_STATE_HOLD)
        {
            __SET_VAR(data__->, DONE, , true);
            __SET_VAR(data__->, BUSY, , false);
            __SET_VAR(data__->, ERROR, , false);
            __SET_VAR(data__->, ERRORID, , (IEC_WORD)0);
            __SET_VAR(data__->, _PENDING, , false);
        }
        else
        {
            __SET_VAR(data__->, BUSY, , true);
        }
    }

    __SET_VAR(data__->, EXECUTE0, , execute);
}

void __mcl_cmd_Resume(HYD_RESUME *data__)
{
    IEC_BOOL execute = __GET_VAR(data__->EXECUTE);
    IEC_BOOL execRising = execute && !__GET_VAR(data__->EXECUTE0);
    IEC_SINT axisIndex = __GET_VAR(data__->AXISID);

    if (!axisIndexIsAllocated(axisIndex))
    {
        __SET_VAR(data__->, ERROR, , true);
        __SET_VAR(data__->, ERRORID, , (IEC_WORD)HYD_DIAG_CODE_START_CONTEXT_INVALID);
        __SET_VAR(data__->, BUSY, , false);
        __SET_VAR(data__->, EXECUTE0, , execute);
        return;
    }

    HYD_MotionControlFB* fb = &HYD_MotionControlFB_inst[axisIndex];
    IEC_BOOL isPending = __GET_VAR(data__->_PENDING);

    if (!execute)
    {
        __SET_VAR(data__->, DONE, , false);
        __SET_VAR(data__->, ERROR, , false);
        __SET_VAR(data__->, ERRORID, , (IEC_WORD)0);
        __SET_VAR(data__->, BUSY, , false);
        __SET_VAR(data__->, _PENDING, , false);
        __SET_VAR(data__->, EXECUTE0, , execute);
        return;
    }

    if (execRising)
    {
        if (!HYD_MotionControlFB_Resume(fb))
        {
            __SET_VAR(data__->, ERROR, , true);
            __SET_VAR(data__->, ERRORID, , commandFailureErrorId(fb));
            __SET_VAR(data__->, BUSY, , false);
            __SET_VAR(data__->, EXECUTE0, , execute);
            return;
        }

        __SET_VAR(data__->, DONE, , false);
        __SET_VAR(data__->, BUSY, , true);
        __SET_VAR(data__->, ERROR, , false);
        __SET_VAR(data__->, ERRORID, , (IEC_WORD)0);
        __SET_VAR(data__->, _PENDING, , true);
        __SET_VAR(data__->, EXECUTE0, , execute);
        return;
    }

    if (isPending)
    {
        if (HYD_MotionControlFB_IsError(fb))
        {
            __SET_VAR(data__->, ERROR, , true);
            __SET_VAR(data__->, ERRORID, , (IEC_WORD)fb->ERROR_ID);
            __SET_VAR(data__->, BUSY, , false);
            __SET_VAR(data__->, _PENDING, , false);
        }
        else if (fb->FB_STATE != HYD_FB_STATE_HOLD)
        {
            __SET_VAR(data__->, DONE, , true);
            __SET_VAR(data__->, BUSY, , false);
            __SET_VAR(data__->, ERROR, , false);
            __SET_VAR(data__->, ERRORID, , (IEC_WORD)0);
            __SET_VAR(data__->, _PENDING, , false);
        }
        else
        {
            __SET_VAR(data__->, BUSY, , true);
        }
    }

    __SET_VAR(data__->, EXECUTE0, , execute);
}

/* ======================================================================
 * MoveAbsolute (Direct模式) 命令实现
 * ====================================================================== */

void __mcl_cmd_MoveAbsolute(HYD_MOVEABSOLUTE *data__)
{
    IEC_BOOL execute = __GET_VAR(data__->EXECUTE);
    IEC_BOOL execRising = execute && !__GET_VAR(data__->EXECUTE0);
    IEC_SINT axisIndex = __GET_VAR(data__->AXISID);

    if (!axisIndexIsAllocated(axisIndex))
    {
        __SET_VAR(data__->, ERROR, , true);
        __SET_VAR(data__->, ERRORID, , (IEC_WORD)HYD_DIAG_CODE_START_CONTEXT_INVALID);
        __SET_VAR(data__->, EXECUTE0, , execute);
        return;
    }

    HYD_MotionControlFB* fb = &HYD_MotionControlFB_inst[axisIndex];
    IEC_INT bufferMode = __GET_VAR(data__->BUFFERMODE);
    IEC_BOOL isPending = __GET_VAR(data__->_PENDING);
    IEC_WORD myExecId = __GET_VAR(data__->_EXEC_ID);

    if (!execute)
    {
        __SET_VAR(data__->, DONE, , false);
        __SET_VAR(data__->, COMMANDABORTED, , false);
        __SET_VAR(data__->, ERROR, , false);
        __SET_VAR(data__->, ERRORID, , (IEC_WORD)0);
        __SET_VAR(data__->, BUSY, , false);
        __SET_VAR(data__->, ACTIVE, , false);
        __SET_VAR(data__->, _PENDING, , false);
        __SET_VAR(data__->, _EXEC_ID, , (IEC_WORD)0);
        __SET_VAR(data__->, ACTIVE0, , false);
        __SET_VAR(data__->, EXECUTE0, , execute);
        return;
    }

    if (execRising)
    {
        IEC_WORD errorId = 0;
        if (!validateSupportedBufferMode(bufferMode, &errorId) ||
            !validateUnsupportedMotionOptions(__GET_VAR(data__->JERK),
                                              &errorId)) {
            __SET_VAR(data__->, ERROR, , true);
            __SET_VAR(data__->, ERRORID, , errorId);
            __SET_VAR(data__->, EXECUTE0, , execute);
            return;
        }

        fb->USE_RECIPE = false;

        HYD_MotionDirection dir = mapPlcOpenDirection(__GET_VAR(data__->DIRECTION));
        HYD_REAL velocity = __GET_VAR(data__->VELOCITY);
        HYD_REAL targetPos = __GET_VAR(data__->POSITION);
        velocity = (IEC_REAL)fabs((double)velocity);

        HYD_MotionSegment segment = buildPositionSegment(
            targetPos,
            velocity,
            __GET_VAR(data__->ACCELERATION),
            __GET_VAR(data__->DECELERATION),
            dir,
            fb);

        HYD_DirectStartResult startResult =
            startDirectSegmentExecution(fb, bufferMode, &segment, &errorId);
        if (startResult == HYD_DIRECT_START_REJECTED)
        {
            __SET_VAR(data__->, ERROR, , true);
            __SET_VAR(data__->, ERRORID, , errorId);
            __SET_VAR(data__->, EXECUTE0, , execute);
            return;
        }

        if (startResult == HYD_DIRECT_START_STARTED) {
            __SET_VAR(data__->, _PENDING, , false);
            __SET_VAR(data__->, _EXEC_ID, , (IEC_WORD)HYD_MotionControlFB_GetDirectOwnerTicket(fb));
        } else {
            __SET_VAR(data__->, _PENDING, , true);
            __SET_VAR(data__->, _EXEC_ID, , (IEC_WORD)0);
        }
        __SET_VAR(data__->, BUSY, , true);
        __SET_VAR(data__->, ACTIVE, , startResult == HYD_DIRECT_START_STARTED);
        __SET_VAR(data__->, DONE, , false);
        __SET_VAR(data__->, COMMANDABORTED, , false);
        __SET_VAR(data__->, ACTIVE0, , __GET_VAR(data__->ACTIVE));
        __SET_VAR(data__->, EXECUTE0, , execute);
        return;
    }

    if (isPending)
    {
        HYD_DirectPendingStatus pendingStatus = resolveDirectPendingOwnership(fb, &myExecId);

        if (pendingStatus == HYD_DIRECT_PENDING_ACQUIRED) {
            __SET_VAR(data__->, _EXEC_ID, , myExecId);
            __SET_VAR(data__->, _PENDING, , false);
            /* Owner transfer already happened inside the runtime during the
             * preceding Publish/cycle. Fall through so this same PLC scan maps
             * ACTIVE/DONE from the newly acquired direct owner instead of
             * delaying visibility by one scan. */
        } else if (pendingStatus == HYD_DIRECT_PENDING_ABORTED) {
            __SET_VAR(data__->, COMMANDABORTED, , true);
            __SET_VAR(data__->, BUSY, , false);
            __SET_VAR(data__->, ACTIVE, , false);
            __SET_VAR(data__->, _PENDING, , false);
            __SET_VAR(data__->, EXECUTE0, , execute);
            return;
        } else {
            __SET_VAR(data__->, EXECUTE0, , execute);
            return;
        }
    }

    if (myExecId != 0)
    {
        if (directExecutionWasCompleted(fb, myExecId, HYD_DIRECT_CMD_MOVE_ABSOLUTE)) {
            HYD_BOOL ownerStillSame = directExecutionIsCurrentOwner(fb,
                                                                    myExecId,
                                                                    HYD_DIRECT_CMD_MOVE_ABSOLUTE);

            if (__GET_VAR(data__->CONTINUOUSUPDATE) && ownerStillSame) {
                if (!applyMoveAbsoluteLiveUpdate(fb, myExecId, data__)) {
                    if (HYD_MotionControlFB_IsError(fb) ||
                        fb->ERROR_ID != HYD_DIAG_CODE_NONE) {
                        __SET_VAR(data__->, ERROR, , true);
                        __SET_VAR(data__->, ERRORID, , commandFailureErrorId(fb));
                        __SET_VAR(data__->, BUSY, , false);
                        __SET_VAR(data__->, ACTIVE, , false);
                        __SET_VAR(data__->, DONE, , false);
                        __SET_VAR(data__->, COMMANDABORTED, , false);
                    } else {
                        __SET_VAR(data__->, DONE, , true);
                        __SET_VAR(data__->, BUSY, , false);
                        __SET_VAR(data__->, ACTIVE, , false);
                        __SET_VAR(data__->, COMMANDABORTED, , false);
                    }
                } else if (HYD_MotionControlFB_IsError(fb)) {
                    __SET_VAR(data__->, ERROR, , true);
                    __SET_VAR(data__->, ERRORID, , (IEC_WORD)fb->ERROR_ID);
                    __SET_VAR(data__->, BUSY, , false);
                    __SET_VAR(data__->, ACTIVE, , false);
                    __SET_VAR(data__->, DONE, , false);
                    __SET_VAR(data__->, COMMANDABORTED, , false);
                } else if (fb->FB_STATE == HYD_FB_STATE_HOLD) {
                    __SET_VAR(data__->, DONE, , false);
                    __SET_VAR(data__->, BUSY, , true);
                    __SET_VAR(data__->, ACTIVE, , false);
                    __SET_VAR(data__->, COMMANDABORTED, , false);
                } else if (fb->SEGMENT_COMPLETED || (HYD_MotionControlFB_IsDone(fb) && fb->STATE.finished)) {
                    __SET_VAR(data__->, DONE, , true);
                    __SET_VAR(data__->, BUSY, , false);
                    __SET_VAR(data__->, ACTIVE, , false);
                    __SET_VAR(data__->, COMMANDABORTED, , false);
                } else {
                    __SET_VAR(data__->, BUSY, , true);
                    __SET_VAR(data__->, ACTIVE, , true);
                    __SET_VAR(data__->, DONE, , false);
                    __SET_VAR(data__->, COMMANDABORTED, , false);
                }
            } else {
                __SET_VAR(data__->, DONE, , true);
                __SET_VAR(data__->, BUSY, , false);
                __SET_VAR(data__->, ACTIVE, , false);
                __SET_VAR(data__->, COMMANDABORTED, , false);
                /* Blended cutover switches ownership to the follower segment
                 * in the same Publish pass. Clear the stale ticket after
                 * reporting DONE so later scans do not reinterpret the old
                 * command as COMMANDABORTED. Keep the ticket only for the
                 * same-owner CONTINUOUSUPDATE path above. */
                __SET_VAR(data__->, _EXEC_ID, , (IEC_WORD)0);
            }
        } else if (directExecutionWasPreempted(fb, myExecId, HYD_DIRECT_CMD_MOVE_ABSOLUTE)) {
            __SET_VAR(data__->, COMMANDABORTED, , true);
            __SET_VAR(data__->, BUSY, , false);
            __SET_VAR(data__->, ACTIVE, , false);
            __SET_VAR(data__->, DONE, , false);
        } else if (directExecutionIsCurrentOwner(fb, myExecId, HYD_DIRECT_CMD_MOVE_ABSOLUTE)) {
            if (!applyMoveAbsoluteLiveUpdate(fb, myExecId, data__)) {
                __SET_VAR(data__->, ERROR, , true);
                __SET_VAR(data__->, ERRORID, , commandFailureErrorId(fb));
                __SET_VAR(data__->, BUSY, , false);
                __SET_VAR(data__->, ACTIVE, , false);
            }
            else
            if (HYD_MotionControlFB_IsError(fb))
            {
                __SET_VAR(data__->, ERROR, , true);
                __SET_VAR(data__->, ERRORID, , (IEC_WORD)fb->ERROR_ID);
                __SET_VAR(data__->, BUSY, , false);
                __SET_VAR(data__->, ACTIVE, , false);
            }
            else if (fb->FB_STATE == HYD_FB_STATE_HOLD)
            {
                __SET_VAR(data__->, DONE, , false);
                __SET_VAR(data__->, BUSY, , true);
                __SET_VAR(data__->, ACTIVE, , false);
            }
            else if (fb->SEGMENT_COMPLETED || (HYD_MotionControlFB_IsDone(fb) && fb->STATE.finished))
            {
                __SET_VAR(data__->, DONE, , true);
                __SET_VAR(data__->, BUSY, , false);
                __SET_VAR(data__->, ACTIVE, , false);
            }
            else
            {
                __SET_VAR(data__->, BUSY, , true);
                __SET_VAR(data__->, ACTIVE, , true);
                __SET_VAR(data__->, DONE, , false);
            }
        } else if (directExecutionLostOwnership(fb, myExecId, HYD_DIRECT_CMD_MOVE_ABSOLUTE)) {
            __SET_VAR(data__->, COMMANDABORTED, , true);
            __SET_VAR(data__->, BUSY, , false);
            __SET_VAR(data__->, ACTIVE, , false);
            __SET_VAR(data__->, DONE, , false);
        }
    }

    __SET_VAR(data__->, ACTIVE0, , __GET_VAR(data__->ACTIVE));
    __SET_VAR(data__->, EXECUTE0, , execute);
}

/* ======================================================================
 * MoveContinuousAbsolute (Direct模式) 命令实现
 * ====================================================================== */

void __mcl_cmd_MoveContinuousAbsolute(HYD_MOVECONTINUOUSABSOLUTE *data__)
{
    IEC_BOOL execute = __GET_VAR(data__->EXECUTE);
    IEC_BOOL execRising = execute && !__GET_VAR(data__->EXECUTE0);
    IEC_SINT axisIndex = __GET_VAR(data__->AXISID);

    if (!axisIndexIsAllocated(axisIndex))
    {
        __SET_VAR(data__->, ERROR, , true);
        __SET_VAR(data__->, ERRORID, , (IEC_WORD)HYD_DIAG_CODE_START_CONTEXT_INVALID);
        __SET_VAR(data__->, EXECUTE0, , execute);
        return;
    }

    HYD_MotionControlFB* fb = &HYD_MotionControlFB_inst[axisIndex];
    IEC_BOOL isPending = __GET_VAR(data__->_PENDING);
    IEC_WORD myExecId = __GET_VAR(data__->_EXEC_ID);

    if (!execute)
    {
        __SET_VAR(data__->, INENDVELOCITY, , false);
        __SET_VAR(data__->, POSITIONREACHED, , false);
        __SET_VAR(data__->, COMMANDABORTED, , false);
        __SET_VAR(data__->, ERROR, , false);
        __SET_VAR(data__->, ERRORID, , (IEC_WORD)0);
        __SET_VAR(data__->, BUSY, , false);
        __SET_VAR(data__->, _PENDING, , false);
        __SET_VAR(data__->, _EXEC_ID, , (IEC_WORD)0);
        __SET_VAR(data__->, INENDVELOCITY0, , false);
        __SET_VAR(data__->, POSITIONREACHED0, , false);
        __SET_VAR(data__->, EXECUTE0, , execute);
        return;
    }

    if (execRising)
    {
        IEC_WORD errorId = 0;
        HYD_BOOL endDirectionOk = false;
        HYD_MotionDirection requestedDirection;
        HYD_MotionDirection requestedEndDirection;
        HYD_MotionDirection approachDirection;
        HYD_MotionDirection sustainDirection;
        HYD_REAL requestedVelocity;
        HYD_REAL rawEndVelocity;
        HYD_REAL requestedEndVelocity;
        HYD_REAL pressureLimit;
        HYD_MotionSegment approachSegment;
        HYD_ContinuousAbsoluteContext context;
        HYD_DirectStartResult startResult;

        if (!validateUnsupportedMotionOptions(__GET_VAR(data__->JERK), &errorId)) {
            __SET_VAR(data__->, ERROR, , true);
            __SET_VAR(data__->, ERRORID, , errorId);
            __SET_VAR(data__->, EXECUTE0, , execute);
            return;
        }

        fb->USE_RECIPE = false;

        requestedDirection = mapPlcOpenDirection(__GET_VAR(data__->DIRECTION));
        requestedEndDirection =
            mapContinuousEndVelocityDirectionRequest(__GET_VAR(data__->ENDVELOCITYDIRECTION),
                                                     &endDirectionOk);
        requestedVelocity = (HYD_REAL)fabs((double)__GET_VAR(data__->VELOCITY));
        rawEndVelocity = __GET_VAR(data__->ENDVELOCITY);
        requestedEndVelocity = (HYD_REAL)fabs((double)rawEndVelocity);
        pressureLimit = __GET_VAR(data__->PRESSURELIMIT);
        if (pressureLimit <= 0.0f) {
            pressureLimit = fb->PRESSURE_LIMIT;
        }

        if (!endDirectionOk ||
            !(requestedVelocity > 0.0f) ||
            rawEndVelocity < 0.0f ||
            !isfinite(requestedVelocity) ||
            !isfinite(rawEndVelocity) ||
            !isfinite(requestedEndVelocity) ||
            !isfinite(__GET_VAR(data__->POSITION)) ||
            !isfinite(__GET_VAR(data__->ACCELERATION)) ||
            !(__GET_VAR(data__->ACCELERATION) > 0.0f) ||
            !isfinite(__GET_VAR(data__->DECELERATION)) ||
            !isfinite(pressureLimit)) {
            __SET_VAR(data__->, ERROR, , true);
            __SET_VAR(data__->, ERRORID, , (IEC_WORD)HYD_DIAG_CODE_COMMAND_NOT_ALLOWED);
            __SET_VAR(data__->, EXECUTE0, , execute);
            return;
        }

        approachSegment = buildContinuousAbsoluteApproachSegment(
            __GET_VAR(data__->POSITION),
            requestedVelocity,
            __GET_VAR(data__->ACCELERATION),
            __GET_VAR(data__->DECELERATION),
            requestedDirection,
            pressureLimit,
            fb);
        approachDirection = HYD_Segment_ResolveDirection(&approachSegment,
                                                         &fb->AXIS_REF,
                                                         fb->_lastActiveDirection);
        if (requestedDirection == HYD_DIRECTION_SHORTEST_WAY &&
            (approachDirection == HYD_DIRECTION_POSITIVE ||
             approachDirection == HYD_DIRECTION_NEGATIVE)) {
            /* Freeze the initial shortest-way choice for the approach phase so
             * position crossing and sustain handoff do not re-resolve and flip
             * direction around the target on later scans. */
            approachSegment.direction = approachDirection;
        }
        sustainDirection = resolveContinuousEndVelocityDirection(fb,
                                                                 requestedEndDirection,
                                                                 approachDirection);
        context = buildContinuousAbsoluteContext(
            __GET_VAR(data__->POSITION),
            requestedEndVelocity,
            pressureLimit,
            __GET_VAR(data__->ADAPTENDVELTOAVOIDOVERSHOOT),
            approachDirection,
            sustainDirection);

        startResult = startDirectSegmentExecutionWithKind(
            fb,
            HYD_DIRECT_CMD_MOVE_CONTINUOUS_ABSOLUTE,
            HYD_BUFFER_MODE_ABORT,
            &approachSegment,
            &context,
            &errorId);
        if (startResult == HYD_DIRECT_START_REJECTED)
        {
            __SET_VAR(data__->, ERROR, , true);
            __SET_VAR(data__->, ERRORID, , errorId);
            __SET_VAR(data__->, EXECUTE0, , execute);
            return;
        }

        __SET_VAR(data__->, _PENDING, , startResult != HYD_DIRECT_START_STARTED);
        __SET_VAR(data__->, _EXEC_ID, , (IEC_WORD)HYD_MotionControlFB_GetDirectOwnerTicket(fb));
        __SET_VAR(data__->, BUSY, , true);
        __SET_VAR(data__->, INENDVELOCITY, , false);
        __SET_VAR(data__->, POSITIONREACHED, , false);
        __SET_VAR(data__->, COMMANDABORTED, , false);
        __SET_VAR(data__->, EXECUTE0, , execute);
        return;
    }

    if (isPending)
    {
        HYD_DirectPendingStatus pendingStatus = resolveDirectPendingOwnership(fb, &myExecId);

        if (pendingStatus == HYD_DIRECT_PENDING_ACQUIRED) {
            __SET_VAR(data__->, _EXEC_ID, , myExecId);
            __SET_VAR(data__->, _PENDING, , false);
        } else if (pendingStatus == HYD_DIRECT_PENDING_ABORTED) {
            __SET_VAR(data__->, COMMANDABORTED, , true);
            __SET_VAR(data__->, BUSY, , false);
            __SET_VAR(data__->, POSITIONREACHED, , false);
            __SET_VAR(data__->, INENDVELOCITY, , false);
            __SET_VAR(data__->, _PENDING, , false);
            __SET_VAR(data__->, EXECUTE0, , execute);
            return;
        } else {
            __SET_VAR(data__->, BUSY, , true);
            __SET_VAR(data__->, EXECUTE0, , execute);
            return;
        }
    }

    if (myExecId != 0)
    {
        if (directExecutionWasPreempted(fb, myExecId, HYD_DIRECT_CMD_MOVE_CONTINUOUS_ABSOLUTE) ||
            directExecutionLostOwnership(fb, myExecId, HYD_DIRECT_CMD_MOVE_CONTINUOUS_ABSOLUTE)) {
            __SET_VAR(data__->, COMMANDABORTED, , true);
            __SET_VAR(data__->, BUSY, , false);
            __SET_VAR(data__->, POSITIONREACHED, , false);
            __SET_VAR(data__->, INENDVELOCITY, , false);
        } else if (directExecutionIsCurrentOwner(fb, myExecId, HYD_DIRECT_CMD_MOVE_CONTINUOUS_ABSOLUTE)) {
            __SET_VAR(data__->, BUSY, , true);
            __SET_VAR(data__->, POSITIONREACHED, , fb->_directContinuousAbsolute.positionReachedLatched ? true : false);
            __SET_VAR(data__->, INENDVELOCITY, , fb->_directContinuousAbsolute.inEndVelocityLatched ? true : false);
            __SET_VAR(data__->, COMMANDABORTED, , false);
            if (HYD_MotionControlFB_IsError(fb)) {
                __SET_VAR(data__->, ERROR, , true);
                __SET_VAR(data__->, ERRORID, , (IEC_WORD)fb->ERROR_ID);
                __SET_VAR(data__->, BUSY, , false);
            }
        }
    }

    __SET_VAR(data__->, POSITIONREACHED0, , __GET_VAR(data__->POSITIONREACHED));
    __SET_VAR(data__->, INENDVELOCITY0, , __GET_VAR(data__->INENDVELOCITY));
    __SET_VAR(data__->, EXECUTE0, , execute);
}

/* ======================================================================
 * MoveVelocity (Direct模式) 命令实现
 * ====================================================================== */

void __mcl_cmd_MoveVelocity(HYD_MOVEVELOCITY *data__)
{
    IEC_BOOL execute = __GET_VAR(data__->EXECUTE);
    IEC_BOOL execRising = execute && !__GET_VAR(data__->EXECUTE0);
    IEC_SINT axisIndex = __GET_VAR(data__->AXISID);

    if (!axisIndexIsAllocated(axisIndex))
    {
        __SET_VAR(data__->, ERROR, , true);
        __SET_VAR(data__->, ERRORID, , (IEC_WORD)HYD_DIAG_CODE_START_CONTEXT_INVALID);
        __SET_VAR(data__->, EXECUTE0, , execute);
        return;
    }

    HYD_MotionControlFB* fb = &HYD_MotionControlFB_inst[axisIndex];
    IEC_INT bufferMode = __GET_VAR(data__->BUFFERMODE);
    IEC_BOOL isPending = __GET_VAR(data__->_PENDING);
    IEC_WORD myExecId = __GET_VAR(data__->_EXEC_ID);
    HYD_REAL targetVelocity = __GET_VAR(data__->VELOCITY);

    if (!execute)
    {
        __SET_VAR(data__->, INVELOCITY, , false);
        __SET_VAR(data__->, COMMANDABORTED, , false);
        __SET_VAR(data__->, ERROR, , false);
        __SET_VAR(data__->, ERRORID, , (IEC_WORD)0);
        __SET_VAR(data__->, BUSY, , false);
        __SET_VAR(data__->, ACTIVE, , false);
        __SET_VAR(data__->, _PENDING, , false);
        __SET_VAR(data__->, _EXEC_ID, , (IEC_WORD)0);
        __SET_VAR(data__->, ACTIVE0, , false);
        __SET_VAR(data__->, EXECUTE0, , execute);
        return;
    }

    if (execRising)
    {
        IEC_WORD errorId = 0;
        HYD_REAL pressureLimit;
        if (!validateSupportedBufferMode(bufferMode, &errorId) ||
            !validateUnsupportedMotionOptions(__GET_VAR(data__->JERK),
                                              &errorId) ||
            !resolvePressureLimit(__GET_VAR(data__->PRESSURELIMIT),
                                  fb,
                                  &pressureLimit,
                                  &errorId)) {
            __SET_VAR(data__->, ERROR, , true);
            __SET_VAR(data__->, ERRORID, , errorId);
            __SET_VAR(data__->, EXECUTE0, , execute);
            return;
        }

        fb->USE_RECIPE = false;

        HYD_MotionDirection dir = mapPlcOpenDirection(__GET_VAR(data__->DIRECTION));
        HYD_REAL velocity = targetVelocity;

        /* SHORTEST_WAY: 根据 Velocity 正负区分方向 */
        if (dir == HYD_DIRECTION_SHORTEST_WAY) {
            if (velocity > 0.0f) {
                dir = HYD_DIRECTION_POSITIVE;
            } else if (velocity < 0.0f) {
                dir = HYD_DIRECTION_NEGATIVE;
            } else {
                /* Velocity == 0: 利用 lastActiveDirection 或默认正向 */
                dir = (fb->_lastActiveDirection == HYD_DIRECTION_NEGATIVE)
                      ? HYD_DIRECTION_NEGATIVE : HYD_DIRECTION_POSITIVE;
            }
        }

        /* 所有模式下，Velocity 取绝对值（Direction 优先级高于符号） */
        velocity = (IEC_REAL)fabs((double)velocity);

        HYD_MotionSegment segment = buildVelocitySegment(
            velocity,
            __GET_VAR(data__->ACCELERATION),
            __GET_VAR(data__->DECELERATION),
            dir,
            pressureLimit,
            fb);

        HYD_DirectStartResult startResult =
            startDirectSegmentExecution(fb, bufferMode, &segment, &errorId);
        if (startResult == HYD_DIRECT_START_REJECTED)
        {
            __SET_VAR(data__->, ERROR, , true);
            __SET_VAR(data__->, ERRORID, , errorId);
            __SET_VAR(data__->, EXECUTE0, , execute);
            return;
        }

        if (startResult == HYD_DIRECT_START_STARTED) {
            __SET_VAR(data__->, _PENDING, , false);
            __SET_VAR(data__->, _EXEC_ID, , (IEC_WORD)HYD_MotionControlFB_GetDirectOwnerTicket(fb));
        } else {
            __SET_VAR(data__->, _PENDING, , true);
            __SET_VAR(data__->, _EXEC_ID, , (IEC_WORD)0);
        }
        __SET_VAR(data__->, BUSY, , true);
        __SET_VAR(data__->, ACTIVE, , startResult == HYD_DIRECT_START_STARTED);
        __SET_VAR(data__->, INVELOCITY, , false);
        __SET_VAR(data__->, COMMANDABORTED, , false);
        __SET_VAR(data__->, ACTIVE0, , __GET_VAR(data__->ACTIVE));
        __SET_VAR(data__->, EXECUTE0, , execute);
        return;
    }

    if (isPending)
    {
        HYD_DirectPendingStatus pendingStatus = resolveDirectPendingOwnership(fb, &myExecId);

        if (pendingStatus == HYD_DIRECT_PENDING_ACQUIRED) {
            __SET_VAR(data__->, _EXEC_ID, , myExecId);
            __SET_VAR(data__->, _PENDING, , false);
        } else if (pendingStatus == HYD_DIRECT_PENDING_ABORTED) {
            __SET_VAR(data__->, COMMANDABORTED, , true);
            __SET_VAR(data__->, BUSY, , false);
            __SET_VAR(data__->, ACTIVE, , false);
            __SET_VAR(data__->, INVELOCITY, , false);
            __SET_VAR(data__->, _PENDING, , false);
            __SET_VAR(data__->, EXECUTE0, , execute);
            return;
        }
        __SET_VAR(data__->, EXECUTE0, , execute);
        return;
    }

    if (myExecId != 0)
    {
        if (directExecutionWasPreempted(fb, myExecId, HYD_DIRECT_CMD_MOVE_VELOCITY)) {
            __SET_VAR(data__->, COMMANDABORTED, , true);
            __SET_VAR(data__->, BUSY, , false);
            __SET_VAR(data__->, ACTIVE, , false);
            __SET_VAR(data__->, INVELOCITY, , false);
        } else if (directExecutionIsCurrentOwner(fb, myExecId, HYD_DIRECT_CMD_MOVE_VELOCITY)) {
            if (!applyMoveVelocityLiveUpdate(fb, myExecId, data__)) {
                __SET_VAR(data__->, ERROR, , true);
                __SET_VAR(data__->, ERRORID, , commandFailureErrorId(fb));
                __SET_VAR(data__->, BUSY, , false);
                __SET_VAR(data__->, ACTIVE, , false);
                __SET_VAR(data__->, INVELOCITY, , false);
            }
            else
            if (HYD_MotionControlFB_IsError(fb))
            {
                __SET_VAR(data__->, ERROR, , true);
                __SET_VAR(data__->, ERRORID, , (IEC_WORD)fb->ERROR_ID);
                __SET_VAR(data__->, BUSY, , false);
                __SET_VAR(data__->, ACTIVE, , false);
                __SET_VAR(data__->, INVELOCITY, , false);
            }
            else if (fb->FB_STATE == HYD_FB_STATE_HOLD)
            {
                __SET_VAR(data__->, BUSY, , true);
                __SET_VAR(data__->, ACTIVE, , false);
                __SET_VAR(data__->, INVELOCITY, , false);
            }
            else
            {
                __SET_VAR(data__->, BUSY, , true);
                __SET_VAR(data__->, ACTIVE, , true);

                HYD_REAL velError = fb->AXIS_REF.velocity - targetVelocity;
                if (velError < 0.0f) velError = -velError;
                if (targetVelocity > 0.0f && velError < targetVelocity * 0.05f)
                {
                    __SET_VAR(data__->, INVELOCITY, , true);
                }
                else
                {
                    __SET_VAR(data__->, INVELOCITY, , false);
                }
            }
        } else if (directExecutionLostOwnership(fb, myExecId, HYD_DIRECT_CMD_MOVE_VELOCITY)) {
            __SET_VAR(data__->, COMMANDABORTED, , true);
            __SET_VAR(data__->, BUSY, , false);
            __SET_VAR(data__->, ACTIVE, , false);
            __SET_VAR(data__->, INVELOCITY, , false);
        }
    }

    __SET_VAR(data__->, ACTIVE0, , __GET_VAR(data__->ACTIVE));
    __SET_VAR(data__->, INVELOCITY0, , __GET_VAR(data__->INVELOCITY));
    __SET_VAR(data__->, EXECUTE0, , execute);
}

/* ======================================================================
 * Reset (Direct模式) 命令实现
 * ====================================================================== */

void __mcl_cmd_Reset(HYD_RESET *data__)
{
    IEC_BOOL execute = __GET_VAR(data__->EXECUTE);
    IEC_BOOL execRising = execute && !__GET_VAR(data__->EXECUTE0);
    IEC_SINT axisIndex = __GET_VAR(data__->AXISID);

    if (!axisIndexIsAllocated(axisIndex))
    {
        __SET_VAR(data__->, ERROR, , true);
        __SET_VAR(data__->, ERRORID, , (IEC_WORD)HYD_DIAG_CODE_START_CONTEXT_INVALID);
        __SET_VAR(data__->, EXECUTE0, , execute);
        return;
    }

    HYD_MotionControlFB* fb = &HYD_MotionControlFB_inst[axisIndex];
    if (fb->FB_STATE == HYD_FB_STATE_DISABLED)
    {
        __SET_VAR(data__->, DONE, , true);
        __SET_VAR(data__->, BUSY, , false);
        __SET_VAR(data__->, EXECUTE0, , execute);
        return;
    }

    if (execRising)
    {
        HYD_MotionControlFB_SoftReset(fb);
        __SET_VAR(data__->, DONE, , true);
        __SET_VAR(data__->, BUSY, , false);
    }

    __SET_VAR(data__->, EXECUTE0, , execute);
}

/* ======================================================================
 * PressureHandle (Direct模式) 命令实现
 * ====================================================================== */

void __mcl_cmd_PressureHandle(HYD_PRESSUREHANDLE *data__)
{
    IEC_BOOL execute = __GET_VAR(data__->EXECUTE);
    IEC_BOOL execRising = execute && !__GET_VAR(data__->EXECUTE0);
    IEC_SINT axisIndex = __GET_VAR(data__->AXISID);

    if (!axisIndexIsAllocated(axisIndex))
    {
        __SET_VAR(data__->, ERROR, , true);
        __SET_VAR(data__->, ERRORID, , (IEC_WORD)HYD_DIAG_CODE_START_CONTEXT_INVALID);
        __SET_VAR(data__->, EXECUTE0, , execute);
        return;
    }

    HYD_MotionControlFB* fb = &HYD_MotionControlFB_inst[axisIndex];
    IEC_INT bufferMode = __GET_VAR(data__->BUFFERMODE);
    IEC_BOOL isPending = __GET_VAR(data__->_PENDING);
    IEC_WORD myExecId = __GET_VAR(data__->_EXEC_ID);
    HYD_REAL targetPressure = __GET_VAR(data__->PRESSURE);

    if (!__GET_VAR(data__->EN) || !execute)
    {
        __SET_VAR(data__->, INPRESSURE, , false);
        __SET_VAR(data__->, DONE, , false);
        __SET_VAR(data__->, COMMANDABORTED, , false);
        __SET_VAR(data__->, ERROR, , false);
        __SET_VAR(data__->, ERRORID, , (IEC_WORD)0);
        __SET_VAR(data__->, BUSY, , false);
        __SET_VAR(data__->, ACTIVE, , false);
        __SET_VAR(data__->, _PENDING, , false);
        __SET_VAR(data__->, _EXEC_ID, , (IEC_WORD)0);
        __SET_VAR(data__->, ACTIVE0, , false);
        __SET_VAR(data__->, INPRESSURE0, , false);
        __SET_VAR(data__->, EXECUTE0, , execute);
        return;
    }

    if (execRising)
    {
        IEC_WORD errorId = 0;
        if (!validateSupportedBufferMode(bufferMode, &errorId)) {
            __SET_VAR(data__->, ERROR, , true);
            __SET_VAR(data__->, ERRORID, , errorId);
            __SET_VAR(data__->, EXECUTE0, , execute);
            return;
        }

        fb->USE_RECIPE = false;

        HYD_MotionSegment segment = buildPressureSegment(
            targetPressure,
            __GET_VAR(data__->PRESSURERAMPRATE),
            __GET_VAR(data__->DURATION),
            fb);

        HYD_DirectStartResult startResult =
            startDirectSegmentExecution(fb, bufferMode, &segment, &errorId);
        if (startResult == HYD_DIRECT_START_REJECTED)
        {
            __SET_VAR(data__->, ERROR, , true);
            __SET_VAR(data__->, ERRORID, , errorId);
            __SET_VAR(data__->, EXECUTE0, , execute);
            return;
        }

        if (startResult == HYD_DIRECT_START_STARTED) {
            __SET_VAR(data__->, _PENDING, , false);
            __SET_VAR(data__->, _EXEC_ID, , (IEC_WORD)HYD_MotionControlFB_GetDirectOwnerTicket(fb));
        } else {
            __SET_VAR(data__->, _PENDING, , true);
            __SET_VAR(data__->, _EXEC_ID, , (IEC_WORD)0);
        }
        __SET_VAR(data__->, BUSY, , true);
        __SET_VAR(data__->, ACTIVE, , startResult == HYD_DIRECT_START_STARTED);
        __SET_VAR(data__->, INPRESSURE, , false);
        __SET_VAR(data__->, DONE, , false);
        __SET_VAR(data__->, COMMANDABORTED, , false);
        __SET_VAR(data__->, ACTIVE0, , __GET_VAR(data__->ACTIVE));
        __SET_VAR(data__->, EXECUTE0, , execute);
        return;
    }

    if (isPending)
    {
        HYD_DirectPendingStatus pendingStatus = resolveDirectPendingOwnership(fb, &myExecId);

        if (pendingStatus == HYD_DIRECT_PENDING_ACQUIRED) {
            __SET_VAR(data__->, _EXEC_ID, , myExecId);
            __SET_VAR(data__->, _PENDING, , false);
        } else if (pendingStatus == HYD_DIRECT_PENDING_ABORTED) {
            __SET_VAR(data__->, COMMANDABORTED, , true);
            __SET_VAR(data__->, BUSY, , false);
            __SET_VAR(data__->, ACTIVE, , false);
            __SET_VAR(data__->, INPRESSURE, , false);
            __SET_VAR(data__->, DONE, , false);
            __SET_VAR(data__->, _PENDING, , false);
            __SET_VAR(data__->, EXECUTE0, , execute);
            return;
        }
        __SET_VAR(data__->, EXECUTE0, , execute);
        return;
    }

    if (myExecId != 0)
    {
        if (directExecutionWasPreempted(fb, myExecId, HYD_DIRECT_CMD_PRESSURE_HANDLE)) {
            __SET_VAR(data__->, COMMANDABORTED, , true);
            __SET_VAR(data__->, BUSY, , false);
            __SET_VAR(data__->, ACTIVE, , false);
            __SET_VAR(data__->, INPRESSURE, , false);
            __SET_VAR(data__->, DONE, , false);
        } else if (directExecutionIsCurrentOwner(fb, myExecId, HYD_DIRECT_CMD_PRESSURE_HANDLE)) {
            if (!applyPressureHandleLiveUpdate(fb, myExecId, data__)) {
                __SET_VAR(data__->, ERROR, , true);
                __SET_VAR(data__->, ERRORID, , commandFailureErrorId(fb));
                __SET_VAR(data__->, BUSY, , false);
                __SET_VAR(data__->, ACTIVE, , false);
                __SET_VAR(data__->, INPRESSURE, , false);
                __SET_VAR(data__->, DONE, , false);
            }
            else
            if (HYD_MotionControlFB_IsError(fb))
            {
                __SET_VAR(data__->, ERROR, , true);
                __SET_VAR(data__->, ERRORID, , (IEC_WORD)fb->ERROR_ID);
                __SET_VAR(data__->, BUSY, , false);
                __SET_VAR(data__->, ACTIVE, , false);
                __SET_VAR(data__->, INPRESSURE, , false);
                __SET_VAR(data__->, DONE, , false);
            } else if (fb->FB_STATE == HYD_FB_STATE_HOLD) {
                __SET_VAR(data__->, BUSY, , true);
                __SET_VAR(data__->, ACTIVE, , false);
                __SET_VAR(data__->, INPRESSURE, , false);
                __SET_VAR(data__->, DONE, , false);
            } else if (fb->SEGMENT_COMPLETED || (HYD_MotionControlFB_IsDone(fb) && fb->STATE.finished)) {
                __SET_VAR(data__->, DONE, , true);
                __SET_VAR(data__->, BUSY, , false);
                __SET_VAR(data__->, ACTIVE, , false);
                __SET_VAR(data__->, INPRESSURE, , false);
            } else {
                __SET_VAR(data__->, BUSY, , true);
                __SET_VAR(data__->, ACTIVE, , true);

                HYD_REAL pressError = fb->AXIS_REF.pressure - targetPressure;
                if (pressError < 0.0f) pressError = -pressError;
                if (targetPressure > 0.0f && pressError < 0.5f) {
                    __SET_VAR(data__->, INPRESSURE, , true);
                } else {
                    __SET_VAR(data__->, INPRESSURE, , false);
                }
                __SET_VAR(data__->, DONE, , false);
            }
        } else if (directExecutionLostOwnership(fb, myExecId, HYD_DIRECT_CMD_PRESSURE_HANDLE)) {
            __SET_VAR(data__->, COMMANDABORTED, , true);
            __SET_VAR(data__->, BUSY, , false);
            __SET_VAR(data__->, ACTIVE, , false);
            __SET_VAR(data__->, INPRESSURE, , false);
            __SET_VAR(data__->, DONE, , false);
        }
    }

    __SET_VAR(data__->, ACTIVE0, , __GET_VAR(data__->ACTIVE));
    __SET_VAR(data__->, INPRESSURE0, , __GET_VAR(data__->INPRESSURE));
    __SET_VAR(data__->, EXECUTE0, , execute);
}

void __mcl_cmd_SetAxisFeedback(HYD_SETAXISFEEDBACK *data__)
{
    IEC_BOOL enable = __GET_VAR(data__->ENABLE);
    IEC_SINT axisIndex = __GET_VAR(data__->AXISID);

    if (!axisIndexIsAllocated(axisIndex))
    {
        __SET_VAR(data__->, ERROR, , true);
        __SET_VAR(data__->, ERRORID, , (IEC_WORD)HYD_DIAG_CODE_START_CONTEXT_INVALID);
        return;
    }

    HYD_MotionControlFB* fb = &HYD_MotionControlFB_inst[axisIndex];

    if (enable && !fb->_useSimulation) {
        fb->AXIS_REF.position = __GET_VAR(data__->ACT_POSITION);
        fb->AXIS_REF.flow     = __GET_VAR(data__->ACT_FLOW);
        fb->AXIS_REF.pressure = __GET_VAR(data__->ACT_PRESSURE);
        fb->AXIS_REF.velocity = __GET_VAR(data__->ACT_VELOCITY);
    }

    __SET_VAR(data__->, DONE, , true);

}

void __mcl_cmd_GetPumpRequest(HYD_GETPUMPREQUEST *data__)
{
    IEC_BOOL enable = __GET_VAR(data__->ENABLE);

    if (!enable)
    {
        __SET_VAR(data__->, PUMPSPEED, , (IEC_REAL)0.0);
        __SET_VAR(data__->, CONFLICT, , false);
        __SET_VAR(data__->, DONE, , true);
        return;
    }

    /* 正反向分别仲裁:
     * - 正向：多轴取 MAX（供油需求最大的轴）
     * - 反向：多轴取 MIN（抽油需求最大的轴，用于快速卸压）
     * - 优先级：反向 > 正向（卸压优先于供油，安全设计）
     *
     * ALLOW_NEGATIVE：全局策略配置，PLC 层设置。
     *   默认 FALSE（向后兼容，不写此引脚时行为与原版一致）。
     */
    IEC_BOOL allowNegative = __GET_VAR(data__->ALLOW_NEGATIVE);
    HYD_REAL maxForward = 0.0f;
    HYD_REAL minReverse = 0.0f;   /* 初始化为 0 = 无反转需求 */
    IEC_BOOL hasReverse = false;
    IEC_BOOL sawExtend = false;
    IEC_BOOL sawRetract = false;

    for (int i = 0; i < HYD_MAX_AXIS_MOTION; i++) {
        if (HYD_AxisSlots[i] != HYD_AXIS_SLOT_ACTIVE) {
            continue;
        }
        HYD_MotionControlFB* fb = &HYD_MotionControlFB_inst[i];
        if (!fb->STATE.active) {
            continue;
        }

        /* 正向仲裁 */
        if (fb->PUMP_SPEED > maxForward) {
            maxForward = fb->PUMP_SPEED;
        }

        /* 反向仲裁（负转速 = 反转卸压）
         * 只有当 allowNegative=TRUE 时才考虑负转速 */
        if (allowNegative && fb->PUMP_SPEED < 0.0f) {
            hasReverse = true;
            if (fb->PUMP_SPEED < minReverse) {
                minReverse = fb->PUMP_SPEED;
            }
        }

        /* Toggle platen and cylinder directions can be opposite. */
        HYD_MotionDirection arbitrationDirection =
            (fb->mechanismType ==
             (HYD_UINT8)HYD_MECHANISM_FIVE_POINT_TOGGLE)
                ? fb->STATE.actuatorDirection
                : fb->STATE.plannedDirection;
        switch (arbitrationDirection) {
            case HYD_DIRECTION_EXTEND:
                sawExtend = true;
                break;
            case HYD_DIRECTION_RETRACT:
                sawRetract = true;
                break;
            case HYD_DIRECTION_HOLD:
            case HYD_DIRECTION_AUTO:
            default:
                /* HOLD / AUTO axes do not contribute to direction conflict. */
                break;
        }
    }

    /* 输出仲裁：反向优先（卸压优先级高） */
    HYD_REAL outputSpeed;
    if (hasReverse) {
        /* 有反转卸压需求：输出最小的负值（绝对值最大的反转请求）
         * 负转速下限已由上游 output_limiter 保证在
         *   [-pumpSpeedLimit * HYD_PUMP_NEGATIVE_SPEED_RATIO, 0] 范围内 */
        outputSpeed = minReverse;
    } else {
        outputSpeed = maxForward;
    }

    __SET_VAR(data__->, PUMPSPEED, , (IEC_REAL)outputSpeed);
    __SET_VAR(data__->, CONFLICT, , (sawExtend && sawRetract) ? true : false);
    __SET_VAR(data__->, DONE, , true);
}

void __mcl_cmd_ReadStatus(HYD_READSTATUS* data__)
{
    IEC_BOOL enable = __GET_VAR(data__->ENABLE);
    IEC_SINT axisIndex = __GET_VAR(data__->AXISID);

    if (!axisIndexIsAllocated(axisIndex))
    {
        __SET_VAR(data__->, STATE,, 0);
        __SET_VAR(data__->, BUSY,, false);
        __SET_VAR(data__->, PRESSURECONTROLLERAPPLIED,,
                  (IEC_INT)HYD_PRESSURE_CONTROLLER_NONE);
        __SET_VAR(data__->, MECHANISMTYPE,,
                  (IEC_SINT)HYD_MECHANISM_DIRECT);
        __SET_VAR(data__->, ACTUATORDIRECTION,,
                  (IEC_SINT)HYD_DIRECTION_HOLD);
        __SET_VAR(data__->, ACTUATORPOSITION,, (IEC_REAL)0.0f);
        __SET_VAR(data__->, ACTUATORVELOCITYCOMMAND,, (IEC_REAL)0.0f);
        __SET_VAR(data__->, VELOCITYRATIO,, (IEC_REAL)0.0f);
        __SET_VAR(data__->, MECHANISMCONFIGVERSION,, (IEC_WORD)0u);
        __SET_VAR(data__->, REQUESTEDFLOW,, (IEC_REAL)0.0f);
        __SET_VAR(data__->, MAXFLOW,, (IEC_REAL)0.0f);
        __SET_VAR(data__->, MAXTEMPLATEVELOCITY,, (IEC_REAL)0.0f);
        __SET_VAR(data__->, EFFECTIVECYLINDERGAIN,, (IEC_REAL)0.0f);
        __SET_VAR(data__->, FLOWLIMITACTIVE,, false);
        __SET_VAR(data__->, PUMPSPEEDLIMITACTIVE,, false);
        __SET_VAR(data__->, PRESSURELIMITACTIVE,, false);
        __SET_VAR(data__->, SOFTLIMITACTIVE,, false);
        __SET_VAR(data__->, DERATED,, false);
        return;
    }

    HYD_MotionControlFB* fb = &HYD_MotionControlFB_inst[axisIndex];

    if (enable)
    {
        __SET_VAR(data__->, STATE,, (IEC_UINT)fb->STATE.status);
        __SET_VAR(data__->, BUSY,, HYD_MotionControlFB_IsBusy(fb) ? true : false);
        __SET_VAR(data__->, PRESSURECONTROLLERAPPLIED,,
                  (IEC_INT)fb->STATE.pressureControllerApplied);
        __SET_VAR(data__->, MECHANISMTYPE,,
                  (IEC_SINT)fb->STATE.mechanismType);
        __SET_VAR(data__->, ACTUATORDIRECTION,,
                  (IEC_SINT)fb->STATE.actuatorDirection);
        __SET_VAR(data__->, MECHANISMCONFIGVERSION,,
                  (IEC_WORD)fb->STATE.mechanismConfigVersion);
#if HYD_ENABLE_MECHANISM_TELEMETRY
        __SET_VAR(data__->, ACTUATORPOSITION,,
                  (IEC_REAL)fb->STATE.actuatorPosition);
        __SET_VAR(data__->, ACTUATORVELOCITYCOMMAND,,
                  (IEC_REAL)fb->STATE.actuatorVelocityCommand);
        __SET_VAR(data__->, VELOCITYRATIO,,
                  (IEC_REAL)fb->STATE.velocityRatio);
#else
        __SET_VAR(data__->, ACTUATORPOSITION,, (IEC_REAL)0.0f);
        __SET_VAR(data__->, ACTUATORVELOCITYCOMMAND,, (IEC_REAL)0.0f);
        __SET_VAR(data__->, VELOCITYRATIO,, (IEC_REAL)0.0f);
#endif
#if HYD_ENABLE_FLOW_DIAGNOSTIC_TELEMETRY
        __SET_VAR(data__->, REQUESTEDFLOW,, (IEC_REAL)fb->STATE.requestedFlow);
        __SET_VAR(data__->, MAXFLOW,, (IEC_REAL)fb->STATE.maxFlow);
        __SET_VAR(data__->, MAXTEMPLATEVELOCITY,, (IEC_REAL)fb->STATE.maxTemplateVelocity);
        __SET_VAR(data__->, EFFECTIVECYLINDERGAIN,, (IEC_REAL)fb->STATE.effectiveCylinderGain);
#else
        __SET_VAR(data__->, REQUESTEDFLOW,, (IEC_REAL)0.0f);
        __SET_VAR(data__->, MAXFLOW,, (IEC_REAL)0.0f);
        __SET_VAR(data__->, MAXTEMPLATEVELOCITY,, (IEC_REAL)0.0f);
        __SET_VAR(data__->, EFFECTIVECYLINDERGAIN,, (IEC_REAL)0.0f);
#endif
        __SET_VAR(data__->, FLOWLIMITACTIVE,,
                  (fb->STATE.limitFlags & HYD_LIMIT_FLAG_FLOW) != 0u);
        __SET_VAR(data__->, PUMPSPEEDLIMITACTIVE,,
                  (fb->STATE.limitFlags & HYD_LIMIT_FLAG_PUMP_SPEED) != 0u);
        __SET_VAR(data__->, PRESSURELIMITACTIVE,,
                  (fb->STATE.limitFlags & HYD_LIMIT_FLAG_PRESSURE) != 0u);
        __SET_VAR(data__->, SOFTLIMITACTIVE,,
                  (fb->STATE.limitFlags & HYD_LIMIT_FLAG_SOFT) != 0u);
        __SET_VAR(data__->, DERATED,,
                  (fb->STATE.limitFlags & HYD_LIMIT_FLAG_DERATE) != 0u);
    }
    else
    {
        __SET_VAR(data__->, STATE,, 0);
        __SET_VAR(data__->, BUSY,, false);
        __SET_VAR(data__->, PRESSURECONTROLLERAPPLIED,,
                  (IEC_INT)HYD_PRESSURE_CONTROLLER_NONE);
        __SET_VAR(data__->, MECHANISMTYPE,,
                  (IEC_SINT)HYD_MECHANISM_DIRECT);
        __SET_VAR(data__->, ACTUATORDIRECTION,,
                  (IEC_SINT)HYD_DIRECTION_HOLD);
        __SET_VAR(data__->, ACTUATORPOSITION,, (IEC_REAL)0.0f);
        __SET_VAR(data__->, ACTUATORVELOCITYCOMMAND,, (IEC_REAL)0.0f);
        __SET_VAR(data__->, VELOCITYRATIO,, (IEC_REAL)0.0f);
        __SET_VAR(data__->, MECHANISMCONFIGVERSION,, (IEC_WORD)0u);
        __SET_VAR(data__->, REQUESTEDFLOW,, (IEC_REAL)0.0f);
        __SET_VAR(data__->, MAXFLOW,, (IEC_REAL)0.0f);
        __SET_VAR(data__->, MAXTEMPLATEVELOCITY,, (IEC_REAL)0.0f);
        __SET_VAR(data__->, EFFECTIVECYLINDERGAIN,, (IEC_REAL)0.0f);
        __SET_VAR(data__->, FLOWLIMITACTIVE,, false);
        __SET_VAR(data__->, PUMPSPEEDLIMITACTIVE,, false);
        __SET_VAR(data__->, PRESSURELIMITACTIVE,, false);
        __SET_VAR(data__->, SOFTLIMITACTIVE,, false);
        __SET_VAR(data__->, DERATED,, false);
    }


}

void __mcl_cmd_ReadError(HYD_READERROR* data__)
{
    IEC_BOOL enable = __GET_VAR(data__->ENABLE);
    IEC_SINT axisIndex = __GET_VAR(data__->AXISID);

    if (!axisIndexIsAllocated(axisIndex))
    {
        __SET_VAR(data__->, ERROR,, false);
        __SET_VAR(data__->, ERRORID,, (IEC_WORD)0);
        __SET_VAR(data__->, BUSY,, false);
        return;
    }

    HYD_MotionControlFB* fb = &HYD_MotionControlFB_inst[axisIndex];

    if (enable)
    {
        __SET_VAR(data__->, ERROR,, HYD_MotionControlFB_IsError(fb) ? true : false);
        __SET_VAR(data__->, ERRORID,, (IEC_WORD)fb->ERROR_ID);
        __SET_VAR(data__->, BUSY,, HYD_MotionControlFB_IsBusy(fb) ? true : false);
    }
    else
    {
        __SET_VAR(data__->, ERROR,, false);
        __SET_VAR(data__->, ERRORID,, (IEC_WORD)0);
        __SET_VAR(data__->, BUSY,, false);
    }


}

void __mcl_cmd_ReadSimFeedback(HYD_READSIMFEEDBACK* data__)
{
    IEC_BOOL enable = __GET_VAR(data__->ENABLE);
    IEC_SINT axisIndex = __GET_VAR(data__->AXISID);

    if (!axisIndexIsAllocated(axisIndex))
    {
        __SET_VAR(data__->, ERROR,, false);
        __SET_VAR(data__->, ERRORID,, (IEC_WORD)0);
        __SET_VAR(data__->, BUSY,, false);
        return;
    }

    HYD_MotionControlFB* fb = &HYD_MotionControlFB_inst[axisIndex];

    if (enable)
    {
        __SET_VAR(data__->, POSITION,, fb->AXIS_REF.position);
        __SET_VAR(data__->, VELOCITY,, fb->AXIS_REF.velocity);
        __SET_VAR(data__->, FLOW,, fb->AXIS_REF.flow);
        __SET_VAR(data__->, PRESSURE,, fb->AXIS_REF.pressure);
        __SET_VAR(data__->, BUSY,, HYD_MotionControlFB_IsBusy(fb) ? true : false);
        __SET_VAR(data__->, VALID,, true );
    }
    else
    {
        __SET_VAR(data__->, ERROR,, false);
        __SET_VAR(data__->, ERRORID,, (IEC_WORD)0);
        __SET_VAR(data__->, BUSY,, false);
    }

}

void __mcl_cmd_ReadParameter(HYD_READPARAMETER *data__)
{
    IEC_BOOL enable = __GET_VAR(data__->ENABLE);
    IEC_SINT axisIndex = __GET_VAR(data__->AXISID);

    if (!axisIndexIsAllocated(axisIndex))
    {
        __SET_VAR(data__->, ERROR,, true);
        __SET_VAR(data__->, ERRORID,, (IEC_WORD)HYD_DIAG_CODE_START_CONTEXT_INVALID);
        __SET_VAR(data__->, VALID,, false);
        __SET_VAR(data__->, BUSY,, false);
        __SET_VAR(data__->, ENABLE0,, enable);
        return;
    }

    HYD_MotionControlFB* fb = &HYD_MotionControlFB_inst[axisIndex];

    if (enable)
    {
        HYD_REAL value;
        if (HYD_MotionControlFB_ReadParameter(fb, __GET_VAR(data__->PARAMETERNUMBER), &value))
        {
            __SET_VAR(data__->, VALUE,, (IEC_LREAL)value);
            __SET_VAR(data__->, VALID,, true);
            __SET_VAR(data__->, ERROR,, false);
            __SET_VAR(data__->, ERRORID,, (IEC_WORD)HYD_DIAG_CODE_NONE);
        }
        else
        {
            __SET_VAR(data__->, ERROR,, true);
            __SET_VAR(data__->, ERRORID,, (IEC_WORD)HYD_DIAG_CODE_SEGMENT_INVALID);
            __SET_VAR(data__->, VALID,, false);
        }
        __SET_VAR(data__->, BUSY,, false);
    }
    else
    {
        __SET_VAR(data__->, VALID,, false);
        __SET_VAR(data__->, BUSY,, false);
    }

    __SET_VAR(data__->, ENABLE0,, enable);
}

void __mcl_cmd_WriteParameter(HYD_WRITEPARAMETER *data__)
{
    IEC_BOOL execute = __GET_VAR(data__->EXECUTE);
    IEC_BOOL execRising = execute && !__GET_VAR(data__->EXECUTE0);
    IEC_SINT axisIndex = __GET_VAR(data__->AXISID);

    if (!axisIndexIsAllocated(axisIndex))
    {
        __SET_VAR(data__->, ERROR,, true);
        __SET_VAR(data__->, ERRORID,, (IEC_WORD)HYD_DIAG_CODE_START_CONTEXT_INVALID);
        __SET_VAR(data__->, EXECUTE0,, execute);
        return;
    }

    HYD_MotionControlFB* fb = &HYD_MotionControlFB_inst[axisIndex];

    if (execRising)
    {
        IEC_LREAL value = __GET_VAR(data__->VALUE);
        if (HYD_MotionControlFB_WriteParameter(fb, __GET_VAR(data__->PARAMETERNUMBER), (HYD_REAL)value))
        {
            __SET_VAR(data__->, DONE,, true);
            __SET_VAR(data__->, BUSY,, false);
            __SET_VAR(data__->, ERROR,, false);
            __SET_VAR(data__->, ERRORID,, (IEC_WORD)HYD_DIAG_CODE_NONE);
        }
        else
        {
            __SET_VAR(data__->, ERROR,, true);
            __SET_VAR(data__->, ERRORID,, (IEC_WORD)HYD_DIAG_CODE_SEGMENT_INVALID);
            __SET_VAR(data__->, DONE,, false);
            __SET_VAR(data__->, BUSY,, false);
        }
    }

    __SET_VAR(data__->, EXECUTE0,, execute);
}

void __mcl_cmd_ReadBoolParameter(HYD_READBOOLPARAMETER *data__)
{
    IEC_BOOL enable = __GET_VAR(data__->ENABLE);
    IEC_SINT axisIndex = __GET_VAR(data__->AXISID);

    if (!axisIndexIsAllocated(axisIndex))
    {
        __SET_VAR(data__->, ERROR,, true);
        __SET_VAR(data__->, ERRORID,, (IEC_WORD)HYD_DIAG_CODE_START_CONTEXT_INVALID);
        __SET_VAR(data__->, VALID,, false);
        __SET_VAR(data__->, BUSY,, false);
        __SET_VAR(data__->, ENABLE0,, enable);
        return;
    }

    HYD_MotionControlFB* fb = &HYD_MotionControlFB_inst[axisIndex];

    if (enable)
    {
        HYD_BOOL value;
        if (HYD_MotionControlFB_ReadBoolParameter(fb, __GET_VAR(data__->PARAMETERNUMBER), &value))
        {
            __SET_VAR(data__->, VALUE,, value ? true : false);
            __SET_VAR(data__->, VALID,, true);
            __SET_VAR(data__->, ERROR,, false);
            __SET_VAR(data__->, ERRORID,, (IEC_WORD)HYD_DIAG_CODE_NONE);
        }
        else
        {
            __SET_VAR(data__->, ERROR,, true);
            __SET_VAR(data__->, ERRORID,, (IEC_WORD)HYD_DIAG_CODE_SEGMENT_INVALID);
            __SET_VAR(data__->, VALID,, false);
        }
        __SET_VAR(data__->, BUSY,, false);
    }
    else
    {
        __SET_VAR(data__->, VALID,, false);
        __SET_VAR(data__->, BUSY,, false);
    }

    __SET_VAR(data__->, ENABLE0,, enable);
}

void __mcl_cmd_WriteBoolParameter(HYD_WRITEBOOLPARAMETER *data__)
{
    IEC_BOOL execute = __GET_VAR(data__->EXECUTE);
    IEC_BOOL execRising = execute && !__GET_VAR(data__->EXECUTE0);
    IEC_SINT axisIndex = __GET_VAR(data__->AXISID);

    if (!axisIndexIsAllocated(axisIndex))
    {
        __SET_VAR(data__->, ERROR,, true);
        __SET_VAR(data__->, ERRORID,, (IEC_WORD)HYD_DIAG_CODE_START_CONTEXT_INVALID);
        __SET_VAR(data__->, EXECUTE0,, execute);
        return;
    }

    HYD_MotionControlFB* fb = &HYD_MotionControlFB_inst[axisIndex];

    if (execRising)
    {
        IEC_BOOL value = __GET_VAR(data__->VALUE);
        if (HYD_MotionControlFB_WriteBoolParameter(fb, __GET_VAR(data__->PARAMETERNUMBER), value ? true : false))
        {
            __SET_VAR(data__->, DONE,, true);
            __SET_VAR(data__->, BUSY,, false);
            __SET_VAR(data__->, ERROR,, false);
            __SET_VAR(data__->, ERRORID,, (IEC_WORD)HYD_DIAG_CODE_NONE);
        }
        else
        {
            __SET_VAR(data__->, ERROR,, true);
            __SET_VAR(data__->, ERRORID,, (IEC_WORD)HYD_DIAG_CODE_SEGMENT_INVALID);
            __SET_VAR(data__->, DONE,, false);
            __SET_VAR(data__->, BUSY,, false);
        }
    }

    __SET_VAR(data__->, EXECUTE0,, execute);
}
