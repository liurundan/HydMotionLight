#include "motion_interface.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

/* ======================================================================
 * FB实例池管理
 * ====================================================================== */
static HYD_REAL dfCycleTime = 0.001f;  /* 默认周期时间，单位秒；可通过外部接口调整以适配不同PLC扫描周期 */
static HYD_TIME g_lastPublishTime = 0.0;

static HYD_MotionControlFB HYD_MotionControlFB_inst[HYD_MAX_AXIS_MOTION];

static unsigned int nextAllocatedFB = 0;

static int allocMotionControlFB(void)
{
    if (nextAllocatedFB < HYD_MAX_AXIS_MOTION) {
        HYD_MotionControlFB_inst[nextAllocatedFB]._index = nextAllocatedFB;
        return (int)nextAllocatedFB++;
    }
    return -1;
}

HYD_MotionControlFB* __MK_GetPublic_MotionControlFB(int index)
{
    if (index >= 0 && index < (int)nextAllocatedFB) {
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
    seg.maxFlow = (velocity > 0.0f) ? velocity * fb->_params.velocityToFlowGain : fb->_params.maxFlow;
    seg.velocityToFlowGain = fb->_params.velocityToFlowGain;

    seg.positionTolerance = fb->_params.positionTolerance;
    seg.timeoutLimit = fb->_params.timeoutLimit;

    return seg;
}

/* 构建速度控制运动段 (MoveVelocity用) */
static HYD_MotionSegment buildVelocitySegment(
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
    seg.mode = HYD_MODE_SPEED_RAMP;
    seg.endCondition = HYD_END_MANUAL;
    seg.direction = direction;
    seg.planner = HYD_PLANNER_TIME_BASED;

    seg.maxVelocity = velocity;
    seg.maxAcceleration = acceleration;
    seg.maxDeceleration = (deceleration > 0.0f) ? deceleration : acceleration;
    seg.maxFlow = (velocity > 0.0f) ? velocity * fb->_params.velocityToFlowGain : fb->_params.maxFlow;
    seg.velocityToFlowGain = fb->_params.velocityToFlowGain;

    seg.timeoutLimit = 0.0f;

    return seg;
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
    seg.maxFlow = fb->_params.maxFlow;
    seg.duration = duration;
    seg.pressureRampRate = rampRate;
    seg.pressureCeiling  = MAX_PRESSURE;

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
    return HYD_MotionControlFB_WasExecutionPreempted(fb, (uint16_t)execId, kind);
}

static HYD_BOOL directExecutionWasCompleted(HYD_MotionControlFB* fb,
                                            IEC_WORD execId,
                                            HYD_DirectCommandKind kind)
{
    return HYD_MotionControlFB_ConsumeExecutionCompleted(fb, (uint16_t)execId, kind);
}

static HYD_BOOL directExecutionIsCurrentOwner(const HYD_MotionControlFB* fb,
                                              IEC_WORD execId,
                                              HYD_DirectCommandKind kind)
{
    return execId == (IEC_WORD)HYD_MotionControlFB_GetDirectOwnerExecutionId(fb) &&
           HYD_MotionControlFB_GetDirectOwnerKind(fb) == kind;
}

static HYD_BOOL directExecutionLostOwnership(const HYD_MotionControlFB* fb,
                                             IEC_WORD execId,
                                             HYD_DirectCommandKind kind)
{
    return execId != (IEC_WORD)HYD_MotionControlFB_GetDirectOwnerExecutionId(fb) ||
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
           (HYD_MotionControlFB_GetDirectOwnerKind(fb) == HYD_DIRECT_CMD_NONE ||
            HYD_MotionControlFB_GetDirectOwnerExecutionId(fb) != fb->_executionId);
}

static HYD_BOOL recipeExecutionWasTakenOverBeforeLatch(const HYD_MotionControlFB* fb)
{
    return fb != NULL &&
           fb->_recipeBatchId != 0U &&
           HYD_MotionControlFB_GetDirectOwnerKind(fb) != HYD_DIRECT_CMD_NONE &&
           HYD_MotionControlFB_GetDirectOwnerExecutionId(fb) == fb->_executionId;
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
    return axisIndex >= 0 && axisIndex < (IEC_SINT)nextAllocatedFB;
}

static IEC_WORD commandFailureErrorId(const HYD_MotionControlFB* fb)
{
    if (fb != NULL && fb->ERROR_ID != HYD_DIAG_CODE_NONE) {
        return (IEC_WORD)fb->ERROR_ID;
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
    request.ownerExecutionId = (uint16_t)execId;
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

    if (fb == NULL || data__ == NULL || !__GET_VAR(data__->CONTINUOUSUPDATE)) {
        return true;
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
                    HYD_LIVE_UPDATE_DIRECTION;
    request.ownerKind = HYD_DIRECT_CMD_MOVE_VELOCITY;
    request.ownerExecutionId = (uint16_t)execId;
    request.maxVelocity = (IEC_REAL)fabs((double)rawVelocity);
    request.maxAcceleration = __GET_VAR(data__->ACCELERATION);
    request.maxDeceleration = __GET_VAR(data__->DECELERATION);
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
    request.ownerExecutionId = (uint16_t)execId;
    request.targetPressure = __GET_VAR(data__->PRESSURE);
    request.pressureRampRate = __GET_VAR(data__->PRESSURERAMPRATE);
    return HYD_MotionControlFB_ApplyLiveUpdate(fb, &request);
}

static HYD_BOOL startDirectSegmentExecution(HYD_MotionControlFB* fb,
                                            IEC_INT bufferMode,
                                            const HYD_MotionSegment* segment,
                                            IEC_WORD* errorId)
{
    if (errorId != NULL) {
        *errorId = (IEC_WORD)0;
    }

    if (fb == NULL || segment == NULL) {
        if (errorId != NULL) {
            *errorId = (IEC_WORD)HYD_DIAG_CODE_INTERNAL_ERROR;
        }
        return false;
    }

    if (!HYD_MotionControlFB_StartDirectCommand(fb,
                                                segment,
                                                (HYD_BufferMode)bufferMode,
                                                fb->AXIS_REF.timestamp)) {
        if (errorId != NULL) {
            *errorId = commandFailureErrorId(fb);
        }
        return false;
    }

    return true;
}

/* ======================================================================
 * 框架生命周期函数
 * ====================================================================== */

int __HydMotion_framework_Init()
{
    for (int i = 0; i < HYD_MAX_AXIS_MOTION; i++) {
        memset(&HYD_MotionControlFB_inst[i], 0, sizeof(HYD_MotionControlFB));
    }
    nextAllocatedFB = 0;
    g_lastPublishTime = 0.0;
    return 0;
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
    HYD_TIME currentTime = HYD_MotionControlFB_inst[0].AXIS_REF.timestamp;

    HYD_TIME deltaTime;

    /* Use the first allocated FB's timestamp as the master clock.
     * If no timestamp is set yet (first cycle), default to 0 delta. */
    if (g_lastPublishTime <= 0.0 || currentTime <= g_lastPublishTime) {
        deltaTime = (currentTime > g_lastPublishTime) ? (currentTime - g_lastPublishTime) : 0.0;
    } else {
        deltaTime = currentTime - g_lastPublishTime;
    }

    for (int i = 0; i < (int)nextAllocatedFB; i++) {
        HYD_MotionControlFB* fb = &HYD_MotionControlFB_inst[i];
        HYD_MotionControlFB_Scan(fb);

        /* Simulation: close the feedback loop with planner outputs */
        if (fb->_useSimulation && fb->_simFeedback.valid) {
            if (deltaTime > 0.0) {
                fb->AXIS_REF.position += fb->_simFeedback.targetVelocity * deltaTime;
            }
            fb->AXIS_REF.velocity  = fb->_simFeedback.targetVelocity;
            fb->AXIS_REF.flow      = fb->_simFeedback.targetFlow;
            fb->AXIS_REF.pressure  = fb->_simFeedback.targetPressure;
        }
    }

    for (int i = 0; i < (int)nextAllocatedFB; i++) {
        HYD_MotionControlFB_inst[i].AXIS_REF.timestamp += dfCycleTime;
    }

    g_lastPublishTime = currentTime;
}

void __mcl_cmd_CreateMotion(HYD_CREATEMOTION *data__)
{
	/* CreateMotion命令仅负责FB实例的创建与初始化, 不直接加载或启动配方 */
	IEC_BOOL bDone = __GET_VAR(data__->DONE);
	if ( !bDone )
	{
		int index = allocMotionControlFB();
		if (index >= 0)
		{
			HYD_MotionControlFB* fb = &HYD_MotionControlFB_inst[index];
			HYD_MotionControlFB_Init(fb);
			fb->FB_STATE = HYD_FB_STATE_IDLE;
			fb->USE_RECIPE = __GET_VAR(data__->USE_RECIPE);
			fb->_configuredUseRecipe = fb->USE_RECIPE;
			fb->FLOW_TO_PUMP_SPEED_GAIN = __GET_VAR(data__->FLOW_TO_PUMPSPEED);
			fb->PUMP_SPEED_LIMIT = __GET_VAR(data__->PUMPSPEED_LIMIT);
            fb->_useSimulation = __GET_VAR(data__->USE_SIMULATION);
			
			__SET_VAR(data__->, DONE, , true);
            __SET_VAR(data__->, AXISID,, index);
		}
	}
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
        fb->AXIS_REF.timestamp = motionData.TIMESTAMP;
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
        HYD_TIME currentTime = motionData.TIMESTAMP;

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
        if (!HYD_MotionControlFB_StartSegment(fb, 0, currentTime)) {
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

    if (axisIndex < 0 || axisIndex >= (IEC_SINT)nextAllocatedFB)
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

    if (axisIndex < 0 || axisIndex >= (IEC_SINT)nextAllocatedFB)
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
        HYD_REAL currentPos = fb->AXIS_REF.position;

        /* Positive_Direction: 强制正向，校验目标位置匹配 */
        if (dir == HYD_DIRECTION_POSITIVE) {
            if (targetPos < currentPos - fb->_params.positionTolerance) {
                __SET_VAR(data__->, ERROR,, true);
                __SET_VAR(data__->, ERRORID,,
                    (IEC_WORD)HYD_DIAG_CODE_PUMP_DIRECTION_CONFLICT);
                __SET_VAR(data__->, EXECUTE0,, execute);
                return;
            }
            velocity = (IEC_REAL)fabs((double)velocity);
        }
        /* Negative_Direction: 强制负向，校验目标位置匹配 */
        else if (dir == HYD_DIRECTION_NEGATIVE) {
            if (targetPos > currentPos + fb->_params.positionTolerance) {
                __SET_VAR(data__->, ERROR,, true);
                __SET_VAR(data__->, ERRORID,,
                    (IEC_WORD)HYD_DIAG_CODE_PUMP_DIRECTION_CONFLICT);
                __SET_VAR(data__->, EXECUTE0,, execute);
                return;
            }
            velocity = (IEC_REAL)fabs((double)velocity);
        }
        /* SHORTEST_WAY 和 CURRENT: 方向由运行时解析，Velocity 恒取正值 */
        else {
            velocity = (IEC_REAL)fabs((double)velocity);
        }

        HYD_MotionSegment segment = buildPositionSegment(
            targetPos,
            velocity,
            __GET_VAR(data__->ACCELERATION),
            __GET_VAR(data__->DECELERATION),
            dir,
            fb);

        if (!startDirectSegmentExecution(fb, bufferMode, &segment, &errorId))
        {
            __SET_VAR(data__->, ERROR, , true);
            __SET_VAR(data__->, ERRORID, , errorId);
            __SET_VAR(data__->, EXECUTE0, , execute);
            return;
        }

        __SET_VAR(data__->, _PENDING, , true);
        __SET_VAR(data__->, _EXEC_ID, , (IEC_WORD)0);
        __SET_VAR(data__->, BUSY, , true);
        __SET_VAR(data__->, ACTIVE, , true);
        __SET_VAR(data__->, DONE, , false);
        __SET_VAR(data__->, COMMANDABORTED, , false);
        __SET_VAR(data__->, ACTIVE0, , true);
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
            __SET_VAR(data__->, _PENDING, , false);
            __SET_VAR(data__->, EXECUTE0, , execute);
            return;
        }
        __SET_VAR(data__->, EXECUTE0, , execute);
        return;
    }

    if (myExecId != 0)
    {
        if (directExecutionWasCompleted(fb, myExecId, HYD_DIRECT_CMD_MOVE_ABSOLUTE)) {
            __SET_VAR(data__->, DONE, , true);
            __SET_VAR(data__->, BUSY, , false);
            __SET_VAR(data__->, ACTIVE, , false);
            __SET_VAR(data__->, COMMANDABORTED, , false);
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
 * MoveVelocity (Direct模式) 命令实现
 * ====================================================================== */

void __mcl_cmd_MoveVelocity(HYD_MOVEVELOCITY *data__)
{
    IEC_BOOL execute = __GET_VAR(data__->EXECUTE);
    IEC_BOOL execRising = execute && !__GET_VAR(data__->EXECUTE0);
    IEC_SINT axisIndex = __GET_VAR(data__->AXISID);

    if (axisIndex < 0 || axisIndex >= (IEC_SINT)nextAllocatedFB)
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
            fb);

        if (!startDirectSegmentExecution(fb, bufferMode, &segment, &errorId))
        {
            __SET_VAR(data__->, ERROR, , true);
            __SET_VAR(data__->, ERRORID, , errorId);
            __SET_VAR(data__->, EXECUTE0, , execute);
            return;
        }

        __SET_VAR(data__->, _PENDING, , true);
        __SET_VAR(data__->, _EXEC_ID, , (IEC_WORD)0);
        __SET_VAR(data__->, BUSY, , true);
        __SET_VAR(data__->, ACTIVE, , true);
        __SET_VAR(data__->, INVELOCITY, , false);
        __SET_VAR(data__->, COMMANDABORTED, , false);
        __SET_VAR(data__->, ACTIVE0, , true);
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

    if (axisIndex < 0 || axisIndex >= (IEC_SINT)nextAllocatedFB)
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

    if (axisIndex < 0 || axisIndex >= (IEC_SINT)nextAllocatedFB)
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

        if (!startDirectSegmentExecution(fb, bufferMode, &segment, &errorId))
        {
            __SET_VAR(data__->, ERROR, , true);
            __SET_VAR(data__->, ERRORID, , errorId);
            __SET_VAR(data__->, EXECUTE0, , execute);
            return;
        }

        __SET_VAR(data__->, _PENDING, , true);
        __SET_VAR(data__->, _EXEC_ID, , (IEC_WORD)0);
        __SET_VAR(data__->, BUSY, , true);
        __SET_VAR(data__->, ACTIVE, , true);
        __SET_VAR(data__->, INPRESSURE, , false);
        __SET_VAR(data__->, DONE, , false);
        __SET_VAR(data__->, COMMANDABORTED, , false);
        __SET_VAR(data__->, ACTIVE0, , true);
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

    if (axisIndex < 0 || axisIndex >= (IEC_SINT)nextAllocatedFB)
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
        fb->AXIS_REF.timestamp = __GET_VAR(data__->TIMESTAMP);
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

    for (int i = 0; i < (int)nextAllocatedFB; i++) {
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

        /* 方向冲突检测（原有逻辑保留） */
        switch (fb->STATE.plannedDirection) {
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

    if (axisIndex < 0 || axisIndex >= (IEC_SINT)nextAllocatedFB)
    {
        __SET_VAR(data__->, STATE,, 0);
        __SET_VAR(data__->, BUSY,, false);
        return;
    }

    HYD_MotionControlFB* fb = &HYD_MotionControlFB_inst[axisIndex];

    if (enable)
    {
        __SET_VAR(data__->, STATE,, (IEC_UINT)fb->STATE.status);
        __SET_VAR(data__->, BUSY,, HYD_MotionControlFB_IsBusy(fb) ? true : false);
    }
    else
    {
        __SET_VAR(data__->, STATE,, 0);
        __SET_VAR(data__->, BUSY,, false);
    }


}

void __mcl_cmd_ReadError(HYD_READERROR* data__)
{
    IEC_BOOL enable = __GET_VAR(data__->ENABLE);
    IEC_SINT axisIndex = __GET_VAR(data__->AXISID);

    if (axisIndex < 0 || axisIndex >= (IEC_SINT)nextAllocatedFB)
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

    if (axisIndex < 0 || axisIndex >= (IEC_SINT)nextAllocatedFB)
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

    if (axisIndex < 0 || axisIndex >= (IEC_SINT)nextAllocatedFB)
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

    if (axisIndex < 0 || axisIndex >= (IEC_SINT)nextAllocatedFB)
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

    if (axisIndex < 0 || axisIndex >= (IEC_SINT)nextAllocatedFB)
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

    if (axisIndex < 0 || axisIndex >= (IEC_SINT)nextAllocatedFB)
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
