#include "motion_interface.h"
#include <string.h>
#include <stdio.h>

/* ======================================================================
 * FB实例池管理
 * ====================================================================== */
static HYD_REAL dfCycleTime = 0.001f;  /* 默认周期时间，单位秒；可通过外部接口调整以适配不同PLC扫描周期 */

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
 * DIRECT_SOURCE 活跃段。确认后将 _executionId 从 _EXEC_ID 写入,
 * 并清除 _PENDING。如果核心引擎报告 ABORTED, 则立即输出 COMMANDABORTED。
 *
 * 阶段2 (_EXEC_ID != 0): 持有 executionId 比较值, 与 FB._executionId 比对:
 *   - 匹配 → 仍是活跃命令, 正常读取核心 FB 输出
 *   - 不匹配 → 被其他命令取代, 输出 COMMANDABORTED
 *
 * 仲裁规则:
 *   - MoveProfile (Recipe模式) 与 Direct模式命令互斥
 *   - 新命令可以接管正在执行的旧命令
 *   - 被接管的命令输出 COMMANDABORTED=true, BUSY=false
 * ====================================================================== */

/* ======================================================================
 * 内部辅助函数
 * ====================================================================== */


/* 从PLCopen方向值映射到HDY_MotionDirection */
static HYD_MotionDirection mapPlcOpenDirection(IEC_SINT direction)
{
    if (direction > 0) {
        return HYD_DIRECTION_EXTEND;
    } else if (direction < 0) {
        return HYD_DIRECTION_RETRACT;
    }
    return HYD_DIRECTION_AUTO;
}

/* 构建位置控制运动段 (MoveAbsolute用) */
static HYD_MotionSegment buildPositionSegment(
    HYD_REAL targetPosition,
    HYD_REAL velocity,
    HYD_REAL acceleration,
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
    seg.segmentType = (HYD_SegmentType)motion->SEGMENTTAG;
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

/* 将当前活动段参数写回MOTION结构体 */
static void writeMotionFromSegment(HYD_AXISMOTION* motion, const HYD_MotionControlFB* fb)
{
    const HYD_MotionSegment* seg = &fb->_activeSegment;

    motion->SEGMENTTAG = (USINT)seg->segmentTag;
    motion->PLANNER = (USINT)seg->planner;
    motion->MODE = (USINT)seg->mode;
    motion->ENDCONDITION = (USINT)seg->endCondition;
    motion->DIRECTION = (USINT)seg->direction;
    motion->SETPOSITION = (REAL)seg->targetPosition;
    motion->SETVELOCITY = (REAL)seg->maxVelocity;
    motion->SETFLOW = (REAL)seg->targetFlow;
    motion->SETPRESSURE = (REAL)seg->targetPressure;
    motion->ACCELERATION = (REAL)seg->maxAcceleration;
    motion->DECELERATION = (REAL)seg->maxAcceleration;
    motion->DURATION = (REAL)seg->duration;
    motion->PRESSURERAMPRATE = (REAL)seg->pressureRampRate;
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
    static HYD_TIME _lastPublishTime = 0.0;

    HYD_TIME currentTime = HYD_MotionControlFB_inst[0].AXIS_REF.timestamp;

    HYD_TIME deltaTime;

    /* Use the first allocated FB's timestamp as the master clock.
     * If no timestamp is set yet (first cycle), default to 0 delta. */
    if (_lastPublishTime <= 0.0 || currentTime <= _lastPublishTime) {
        deltaTime = (currentTime > _lastPublishTime) ? (currentTime - _lastPublishTime) : 0.0;
    } else {
        deltaTime = currentTime - _lastPublishTime;
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

    if( nextAllocatedFB > 0 )
    {
    	HYD_MotionControlFB* fb = &HYD_MotionControlFB_inst[0];
		if( fb && fb->_useSimulation) {
			fb->AXIS_REF.timestamp = fb->AXIS_REF.timestamp + dfCycleTime;
		}
    }

    _lastPublishTime = currentTime;
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
    // TODO: 实现预加载配方, 目前仅支持单段MoveProfile的自动构建和执行,待后面补充完善
    IEC_SINT axisIndex = __GET_VAR(data__->AXISID);
    HYD_MotionControlFB* fb = __MK_GetPublic_MotionControlFB(axisIndex);

    if (fb != NULL) {
        // 预加载配方逻辑
    }
}

void __mcl_cmd_MoveProfile(HYD_MOVEPROFILE *data__)
{
    IEC_SINT axisIndex = __GET_VAR(data__->AXISID);
    HYD_MotionControlFB *fb = __MK_GetPublic_MotionControlFB(axisIndex);

    if (fb == NULL) {
        __SET_VAR(data__->, ERROR,, true);
        __SET_VAR(data__->, ERRORID,, (IEC_WORD)HYD_DIAG_CODE_INTERNAL_ERROR);
        return;
    }

    IEC_BOOL execute = __GET_VAR(data__->EXECUTE);
    IEC_BOOL execRising = execute && !__GET_VAR(data__->EXECUTE0);

    /* Update AXIS_REF from MOTION feedback */
    HYD_AXISMOTION motionData = __GET_VAR(data__->MOTION);
    fb->AXIS_REF.position = motionData.ACTPOSITION;
    fb->AXIS_REF.velocity = motionData.ACTVELOCITY;
    fb->AXIS_REF.flow     = motionData.ACTFLOW;
    fb->AXIS_REF.pressure = motionData.ACTPRESSURE;
    fb->AXIS_REF.timestamp = motionData.TIMESTAMP;

    IEC_INT bufferMode = __GET_VAR(data__->BUFFERMODE);
    IEC_BOOL isPending = __GET_VAR(data__->_PENDING);
    IEC_WORD myExecId = __GET_VAR(data__->_EXEC_ID);

    if (execRising) {
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
        __SET_VAR(data__->, EXECUTE0,, execute);
        return;
    }

    /* Ownership tracking */
    if (isPending) {
        if (fb->STATE.active) {
            __SET_VAR(data__->, _EXEC_ID,, (IEC_WORD)fb->_executionId);
            __SET_VAR(data__->, _PENDING,, false);
            myExecId = (IEC_WORD)fb->_executionId;
        } else if (fb->FB_STATE == HYD_FB_STATE_ABORTED && !fb->STATE.active) {
            __SET_VAR(data__->, ACTIVE,, false);
            __SET_VAR(data__->, BUSY,, false);
            __SET_VAR(data__->, _PENDING,, false);
            __SET_VAR(data__->, EXECUTE0,, execute);
            return;
        }
        __SET_VAR(data__->, EXECUTE0,, execute);
        return;
    }

    if (myExecId != 0) {
        if (myExecId != (IEC_WORD)fb->_executionId) {
            __SET_VAR(data__->, ACTIVE,, false);
            __SET_VAR(data__->, BUSY,, false);
        } else {
            __SET_VAR(data__->, ACTIVE,, fb->STATE.active ? true : false);
            __SET_VAR(data__->, BUSY,, HYD_MotionControlFB_IsBusy(fb));
            __SET_VAR(data__->, DONE,, (HYD_MotionControlFB_IsDone(fb) && fb->STATE.finished) ? true : false);
            __SET_VAR(data__->, ERROR,, HYD_MotionControlFB_IsError(fb) ? true : false);
            __SET_VAR(data__->, ERRORID,, (IEC_WORD)fb->ERROR_ID);

            if (fb->_activeSegmentValid) {
                HYD_AXISMOTION motionOut = __GET_VAR(data__->MOTION);
                writeMotionFromSegment(&motionOut, fb);
                __SET_VAR(data__->, MOTION,, motionOut);
            }
        }
    }

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

        if (fb->FB_STATE != HYD_FB_STATE_STARTING &&
            fb->FB_STATE != HYD_FB_STATE_RUNNING &&
            fb->FB_STATE != HYD_FB_STATE_HOLD) {
            __SET_VAR(data__->, DONE, , true);
            __SET_VAR(data__->, BUSY, , false);
            __SET_VAR(data__->, EXECUTE0, , execute);
            return;
        }

        HYD_MotionControlFB_Stop(fb,
                                 fb->AXIS_REF.timestamp,
                                 __GET_VAR(data__->DECELERATION));
        HYD_MotionControlFB_Scan(fb);
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
        if (bufferMode == HYD_BUFFER_MODE_ABORT) {
            HYD_MotionControlFB_Abort(fb);
            HYD_MotionControlFB_Scan(fb);
        }

        HYD_MotionDirection dir = mapPlcOpenDirection(__GET_VAR(data__->DIRECTION));
        HYD_MotionSegment segment = buildPositionSegment(
            __GET_VAR(data__->POSITION),
            __GET_VAR(data__->VELOCITY),
            __GET_VAR(data__->ACCELERATION),
            dir,
            fb);

        if (!HYD_MotionControlFB_LoadDirectSegment(fb, &segment))
        {
            __SET_VAR(data__->, ERROR, , true);
            __SET_VAR(data__->, ERRORID, , (IEC_WORD)HYD_DIAG_CODE_SEGMENT_INVALID);
            __SET_VAR(data__->, EXECUTE0, , execute);
            return;
        }

        if (!HYD_MotionControlFB_StartSegment(fb, 0, fb->AXIS_REF.timestamp))
        {
            __SET_VAR(data__->, ERROR, , true);
            __SET_VAR(data__->, ERRORID, , (IEC_WORD)fb->ERROR_ID);
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
        if (fb->STATE.active && fb->_activeSegmentSource == HYD_SEGMENT_SOURCE_DIRECT) {
            __SET_VAR(data__->, _EXEC_ID, , (IEC_WORD)fb->_executionId);
            __SET_VAR(data__->, _PENDING, , false);
            myExecId = (IEC_WORD)fb->_executionId;
        } else if (fb->FB_STATE == HYD_FB_STATE_ABORTED && !fb->STATE.active) {
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
        if (HYD_MotionControlFB_WasExecutionPreempted(fb,
                                                      (uint16_t)myExecId,
                                                      HYD_DIRECT_CMD_MOVE_ABSOLUTE)) {
            __SET_VAR(data__->, COMMANDABORTED, , true);
            __SET_VAR(data__->, BUSY, , false);
            __SET_VAR(data__->, ACTIVE, , false);
            __SET_VAR(data__->, DONE, , false);
        } else if (myExecId == (IEC_WORD)HYD_MotionControlFB_GetDirectOwnerExecutionId(fb) &&
                   HYD_MotionControlFB_GetDirectOwnerKind(fb) == HYD_DIRECT_CMD_MOVE_ABSOLUTE) {
            if (HYD_MotionControlFB_IsError(fb))
            {
                __SET_VAR(data__->, ERROR, , true);
                __SET_VAR(data__->, ERRORID, , (IEC_WORD)fb->ERROR_ID);
                __SET_VAR(data__->, BUSY, , false);
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
            }
        } else if (myExecId != (IEC_WORD)HYD_MotionControlFB_GetDirectOwnerExecutionId(fb) ||
                   HYD_MotionControlFB_GetDirectOwnerKind(fb) != HYD_DIRECT_CMD_MOVE_ABSOLUTE) {
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
        if (bufferMode == HYD_BUFFER_MODE_ABORT) {
            HYD_MotionControlFB_Abort(fb);
            HYD_MotionControlFB_Scan(fb);
        }

        HYD_MotionDirection dir = mapPlcOpenDirection(__GET_VAR(data__->DIRECTION));
        HYD_MotionSegment segment = buildVelocitySegment(
            targetVelocity,
            __GET_VAR(data__->ACCELERATION),
            dir,
            fb);

        if (!HYD_MotionControlFB_LoadDirectSegment(fb, &segment))
        {
            __SET_VAR(data__->, ERROR, , true);
            __SET_VAR(data__->, ERRORID, , (IEC_WORD)HYD_DIAG_CODE_SEGMENT_INVALID);
            __SET_VAR(data__->, EXECUTE0, , execute);
            return;
        }

        if (!HYD_MotionControlFB_StartSegment(fb, 0, fb->AXIS_REF.timestamp))
        {
            __SET_VAR(data__->, ERROR, , true);
            __SET_VAR(data__->, ERRORID, , (IEC_WORD)fb->ERROR_ID);
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
        if (fb->STATE.active && fb->_activeSegmentSource == HYD_SEGMENT_SOURCE_DIRECT) {
            __SET_VAR(data__->, _EXEC_ID, , (IEC_WORD)fb->_executionId);
            __SET_VAR(data__->, _PENDING, , false);
            myExecId = (IEC_WORD)fb->_executionId;
        } else if (fb->FB_STATE == HYD_FB_STATE_ABORTED && !fb->STATE.active) {
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
        if (HYD_MotionControlFB_WasExecutionPreempted(fb,
                                                      (uint16_t)myExecId,
                                                      HYD_DIRECT_CMD_MOVE_VELOCITY)) {
            __SET_VAR(data__->, COMMANDABORTED, , true);
            __SET_VAR(data__->, BUSY, , false);
            __SET_VAR(data__->, ACTIVE, , false);
            __SET_VAR(data__->, INVELOCITY, , false);
        } else if (myExecId == (IEC_WORD)HYD_MotionControlFB_GetDirectOwnerExecutionId(fb) &&
                   HYD_MotionControlFB_GetDirectOwnerKind(fb) == HYD_DIRECT_CMD_MOVE_VELOCITY) {
            if (HYD_MotionControlFB_IsError(fb))
            {
                __SET_VAR(data__->, ERROR, , true);
                __SET_VAR(data__->, ERRORID, , (IEC_WORD)fb->ERROR_ID);
                __SET_VAR(data__->, BUSY, , false);
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
        } else if (myExecId != (IEC_WORD)HYD_MotionControlFB_GetDirectOwnerExecutionId(fb) ||
                   HYD_MotionControlFB_GetDirectOwnerKind(fb) != HYD_DIRECT_CMD_MOVE_VELOCITY) {
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

    if (execRising)
    {
        if (bufferMode == HYD_BUFFER_MODE_ABORT) {
            HYD_MotionControlFB_Abort(fb);
            HYD_MotionControlFB_Scan(fb);
        }

        HYD_MotionSegment segment = buildPressureSegment(
            targetPressure,
            __GET_VAR(data__->PRESSURERAMPRATE),
            __GET_VAR(data__->DURATION),
            fb);

        if (!HYD_MotionControlFB_LoadDirectSegment(fb, &segment))
        {
            __SET_VAR(data__->, ERROR, , true);
            __SET_VAR(data__->, ERRORID, , (IEC_WORD)HYD_DIAG_CODE_SEGMENT_INVALID);
            __SET_VAR(data__->, EXECUTE0, , execute);
            return;
        }

        if (!HYD_MotionControlFB_StartSegment(fb, 0, fb->AXIS_REF.timestamp))
        {
            __SET_VAR(data__->, ERROR, , true);
            __SET_VAR(data__->, ERRORID, , (IEC_WORD)fb->ERROR_ID);
            __SET_VAR(data__->, EXECUTE0, , execute);
            return;
        }

        __SET_VAR(data__->, _PENDING, , true);
        __SET_VAR(data__->, _EXEC_ID, , (IEC_WORD)0);
        __SET_VAR(data__->, BUSY, , true);
        __SET_VAR(data__->, ACTIVE, , true);
        __SET_VAR(data__->, INPRESSURE, , false);
        __SET_VAR(data__->, COMMANDABORTED, , false);
        __SET_VAR(data__->, ACTIVE0, , true);
        __SET_VAR(data__->, EXECUTE0, , execute);
        return;
    }

    if (isPending)
    {
        if (fb->STATE.active && fb->_activeSegmentSource == HYD_SEGMENT_SOURCE_DIRECT) {
            __SET_VAR(data__->, _EXEC_ID, , (IEC_WORD)fb->_executionId);
            __SET_VAR(data__->, _PENDING, , false);
            myExecId = (IEC_WORD)fb->_executionId;
        } else if (fb->FB_STATE == HYD_FB_STATE_ABORTED && !fb->STATE.active) {
            __SET_VAR(data__->, COMMANDABORTED, , true);
            __SET_VAR(data__->, BUSY, , false);
            __SET_VAR(data__->, ACTIVE, , false);
            __SET_VAR(data__->, INPRESSURE, , false);
            __SET_VAR(data__->, _PENDING, , false);
            __SET_VAR(data__->, EXECUTE0, , execute);
            return;
        }
        __SET_VAR(data__->, EXECUTE0, , execute);
        return;
    }

    if (myExecId != 0)
    {
        if (myExecId != (IEC_WORD)fb->_executionId) {
            __SET_VAR(data__->, COMMANDABORTED, , true);
            __SET_VAR(data__->, BUSY, , false);
            __SET_VAR(data__->, ACTIVE, , false);
            __SET_VAR(data__->, INPRESSURE, , false);
        } else {
            if (fb->SEGMENT_COMPLETED || (HYD_MotionControlFB_IsDone(fb) && fb->STATE.finished))
            {
                __SET_VAR(data__->, BUSY, , false);
                __SET_VAR(data__->, ACTIVE, , false);
                __SET_VAR(data__->, INPRESSURE, , false);
            }
            else if (fb->STATE.active)
            {
                __SET_VAR(data__->, BUSY, , true);
                __SET_VAR(data__->, ACTIVE, , true);

                HYD_REAL pressError = fb->AXIS_REF.pressure - targetPressure;
                if (pressError < 0.0f) pressError = -pressError;
                if (targetPressure > 0.0f && pressError < 0.5f)
                {
                    __SET_VAR(data__->, INPRESSURE, , true);
                }
                else
                {
                    __SET_VAR(data__->, INPRESSURE, , false);
                }
            }
            else if (HYD_MotionControlFB_IsError(fb))
            {
                __SET_VAR(data__->, ERROR, , true);
                __SET_VAR(data__->, ERRORID, , (IEC_WORD)fb->ERROR_ID);
                __SET_VAR(data__->, BUSY, , false);
                __SET_VAR(data__->, ACTIVE, , false);
                __SET_VAR(data__->, INPRESSURE, , false);
            }
            else
            {
                __SET_VAR(data__->, BUSY, , HYD_MotionControlFB_IsBusy(fb));
                __SET_VAR(data__->, ACTIVE, , fb->STATE.active ? true : false);
            }
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
        __SET_VAR(data__->, DONE, , true);
        return;
    }

    /* STRATEGY: 0 = MAX arbitration (only supported strategy for now) */
    HYD_REAL maxSpeed = 0.0;

    for (int i = 0; i < (int)nextAllocatedFB; i++) {
        HYD_MotionControlFB* fb = &HYD_MotionControlFB_inst[i];
        if (fb->STATE.active && fb->PUMP_SPEED > maxSpeed) {
            maxSpeed = fb->PUMP_SPEED;
        }
    }

    __SET_VAR(data__->, PUMPSPEED, , (IEC_REAL)maxSpeed);
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
