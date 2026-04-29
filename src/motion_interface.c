#include "motion_interface.h"
#include <string.h>
#include <stdio.h>

/* ======================================================================
 * FB实例池管理
 * ====================================================================== */

static HDY_MotionControlFB HDY_MotionControlFB_inst[HDY_MAX_AXIS_MOTION];

static unsigned int nextAllocatedFB = 0;

static int allocMotionControlFB(void)
{
    if (nextAllocatedFB < HDY_MAX_AXIS_MOTION) {
        HDY_MotionControlFB_inst[nextAllocatedFB]._index = nextAllocatedFB;
        return (int)nextAllocatedFB++;
    }
    return -1;
}

HDY_MotionControlFB* __MK_GetPublic_MotionControlFB(int index)
{
    if (index >= 0 && index < (int)nextAllocatedFB) {
        return &HDY_MotionControlFB_inst[index];
    }
    return NULL;
}

/* ======================================================================
 * 命令仲裁
 *
 * 采用轻量化代际计数器模型: HDY_MotionControlFB._commandGeneration
 * 在每次 AbortNow() 时递增。
 *
 * 每个 IEC FB 在发起命令时记录当时的代际值(GEN)，后续扫描中比较:
 *   - 匹配 → 仍是活跃命令，正常读取核心 FB 输出
 *   - 不匹配 → 被其他命令取代，输出 COMMANDABORTED
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
static HDY_MotionDirection mapPlcOpenDirection(IEC_SINT direction)
{
    if (direction > 0) {
        return HDY_DIRECTION_EXTEND;
    } else if (direction < 0) {
        return HDY_DIRECTION_RETRACT;
    }
    return HDY_DIRECTION_AUTO;
}

/* 构建位置控制运动段 (MoveAbsolute用) */
static HDY_MotionSegment buildPositionSegment(
    HDY_REAL targetPosition,
    HDY_REAL velocity,
    HDY_REAL acceleration,
    HDY_MotionDirection direction)
{
    HDY_MotionSegment seg;
    memset(&seg, 0, sizeof(seg));

    seg.segmentTag = HDY_SEGMENT_TYPE_OTHER;
    seg.segmentType = HDY_SEGMENT_TYPE_OTHER;
    seg.mode = HDY_MODE_POSITION;
    seg.endCondition = HDY_END_POSITION;
    seg.direction = direction;
    seg.planner = HDY_PLANNER_POSITION_BASED;

    seg.targetPosition = targetPosition;
    seg.maxVelocity = velocity;
    seg.maxAcceleration = acceleration;
    seg.maxFlow = (velocity > 0.0f) ? velocity * 0.2f : 50.0f;
    seg.velocityToFlowGain = 0.2f;

    seg.positionTolerance = 0.1f;   /* mm */
    seg.timeoutLimit = 10.0f;       /* s */

    return seg;
}

/* 构建速度控制运动段 (MoveVelocity用) */
static HDY_MotionSegment buildVelocitySegment(
    HDY_REAL velocity,
    HDY_REAL acceleration,
    HDY_MotionDirection direction)
{
    HDY_MotionSegment seg;
    memset(&seg, 0, sizeof(seg));

    seg.segmentTag = HDY_SEGMENT_TYPE_OTHER;
    seg.segmentType = HDY_SEGMENT_TYPE_OTHER;
    seg.mode = HDY_MODE_SPEED_RAMP;
    seg.endCondition = HDY_END_MANUAL;  /* 持续运动直到被Stop或其他命令取代 */
    seg.direction = direction;
    seg.planner = HDY_PLANNER_TIME_BASED;

    seg.maxVelocity = velocity;
    seg.maxAcceleration = acceleration;
    seg.maxFlow = (velocity > 0.0f) ? velocity * 0.2f : 50.0f;
    seg.velocityToFlowGain = 0.2f;

    seg.timeoutLimit = 0.0f;  /* 禁用超时，持续运动 */

    return seg;
}

/* 构建压力控制运动段 (PressureHandle用) */
static HDY_MotionSegment buildPressureSegment(
    HDY_REAL targetPressure,
    HDY_REAL rampRate,
    HDY_REAL duration)
{
    HDY_MotionSegment seg;
    memset(&seg, 0, sizeof(seg));

    seg.segmentTag = HDY_SEGMENT_TYPE_HOLDING;
    seg.segmentType = HDY_SEGMENT_TYPE_HOLDING;
    seg.mode = HDY_MODE_PRESSURE_CLOSED_LOOP;
    seg.endCondition = (duration > 0.0f) ? HDY_END_TIME : HDY_END_MANUAL;
    seg.direction = HDY_DIRECTION_HOLD;

    seg.targetPressure = targetPressure;
    seg.targetFlow = 5.0f;          /* 默认保压流量 L/min */
    seg.maxFlow = 20.0f;            /* 最大流量限制 L/min */
    seg.duration = duration;
    seg.pressureRampRate = rampRate;

    seg.pressureController = HDY_PRESSURE_CONTROLLER_PI;
    seg.pressureKp = 0.5f;
    seg.pressureKi = 0.1f;
    seg.pressureKd = 0.0f;
    seg.pressureIntegralLimit = 10.0f;
    seg.pressureDeadband = 0.5f;
    seg.pressureFilterAlpha = 0.5f;
    seg.pressureDerivativeFilterAlpha = 0.5f;

    seg.pressureTolerance = 0.5f;   /* MPa */
    seg.flowTolerance = 1.0f;       /* L/min */
    seg.timeoutLimit = 30.0f;       /* s */

    return seg;
}

/* 从MOTION结构体构建运动段 (MoveProfile用) */
static HDY_MotionSegment buildSegmentFromMotion(const HDY_AXISMOTION* motion)
{
    HDY_MotionSegment seg;
    memset(&seg, 0, sizeof(seg));

    seg.segmentTag = (HDY_UINT8)motion->SEGMENTTAG;
    seg.segmentType = (HDY_SegmentType)motion->SEGMENTTAG;
    seg.mode = (HDY_ControlMode)motion->MODE;
    seg.endCondition = (HDY_EndConditionType)motion->ENDCONDITION;
    seg.direction = (HDY_MotionDirection)motion->DIRECTION;
    seg.planner = (HDY_PlannerType)motion->PLANNER;

    seg.targetPosition = motion->SETPOSITION;
    seg.maxVelocity = motion->SETVELOCITY;
    seg.targetFlow = motion->SETFLOW;
    seg.maxFlow = motion->SETFLOW;
    seg.targetPressure = motion->SETPRESSURE;
    seg.maxAcceleration = motion->ACCELERATION;
    seg.duration = motion->DURATION;
    seg.pressureRampRate = motion->PRESSURERAMPRATE;

    seg.velocityToFlowGain = 0.2f;
    seg.positionTolerance = 0.1f;
    seg.pressureTolerance = 0.5f;
    seg.flowTolerance = 1.0f;
    seg.timeoutLimit = 5.0f;

    seg.pressureController = HDY_PRESSURE_CONTROLLER_P;
    seg.pressureKp = 0.5f;
    seg.pressureKi = 0.1f;
    seg.pressureKd = 0.05f;
    seg.pressureIntegralLimit = 10.0f;
    seg.pressureDeadband = 0.2f;
    seg.pressureFilterAlpha = 0.5f;
    seg.pressureDerivativeFilterAlpha = 0.5f;

    return seg;
}

/* 将当前活动段参数写回MOTION结构体 */
static void writeMotionFromSegment(HDY_AXISMOTION* motion, const HDY_MotionControlFB* fb)
{
    const HDY_MotionSegment* seg = &fb->_activeSegment;

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

int __HdyMotion_framework_Init()
{
    for (int i = 0; i < HDY_MAX_AXIS_MOTION; i++) {
        memset(&HDY_MotionControlFB_inst[i], 0, sizeof(HDY_MotionControlFB));
    }
    nextAllocatedFB = 0;
    return 0;
}

void __HdyMotion_framework_Cleanup()
{
    /* 当前无动态资源需要释放 */
}

void __HdyMotion_framework_Retrieve()
{
    /* 反馈数据由各命令函数或工艺层直接更新到AXIS_REF */
}

void __HdyMotion_framework_Publish()
{
    /* 扫描所有已初始化的FB (包括Direct模式和MoveProfile模式) */
    for (int i = 0; i < HDY_MAX_AXIS_MOTION; i++) {
        HDY_MotionControlFB* fb = &HDY_MotionControlFB_inst[i];
        if (fb->FB_STATE != HDY_FB_STATE_DISABLED) {
            HDY_MotionControlFB_Scan(fb);
        }
    }
}

void __mcl_cmd_CreateMotion(HDY_CREATEMOTION *data__)
{
	/* CreateMotion命令仅负责FB实例的创建与初始化, 不直接加载或启动配方 */
	IEC_BOOL bDone = __GET_VAR(data__->DONE);
	if (bDone == false)
	{
		int index = allocMotionControlFB();
		if (index >= 0)
		{
			HDY_MotionControlFB* fb = &HDY_MotionControlFB_inst[index];
			HDY_MotionControlFB_Init(fb);
			fb->FB_STATE = HDY_FB_STATE_IDLE;
			fb->USE_RECIPE = __GET_VAR(data__->USE_RECIPE);
			fb->FLOW_TO_PUMP_SPEED_GAIN = __GET_VAR(data__->FLOW_TO_PUMPSPEED);
			fb->PUMP_SPEED_LIMIT = __GET_VAR(data__->PUMPSPEED_LIMIT);
			fb->EN = true;

			__SET_VAR(data__->, DONE, , true);
		}
	}
}

/* ======================================================================
 * MoveProfile (Recipe模式) 命令实现
 * ====================================================================== */

void __mcl_cmd_LoadProfile(HDY_LOADPROFILE *data__)
{
    // TODO: 实现预加载配方, 目前仅支持单段MoveProfile的自动构建和执行,待后面补充完善
    IEC_SINT axisIndex = __GET_VAR(data__->AXISID);
    HDY_MotionControlFB* fb = __MK_GetPublic_MotionControlFB(axisIndex);

    if (fb != NULL) {
        // 预加载配方逻辑
    }
}

void __mcl_cmd_MoveProfile(HDY_MOVEPROFILE *data__)
{

	/* 后续周期调用 */
	IEC_SINT axisIndex = __GET_VAR(data__->AXISID);
	HDY_MotionControlFB *fb = __MK_GetPublic_MotionControlFB(axisIndex);

	if (fb == NULL) {
		__SET_VAR(data__->, ERROR,, true);
		__SET_VAR(data__->, ERRORID,, (IEC_WORD )HDY_DIAG_CODE_INTERNAL_ERROR);
		__SET_VAR(data__->, ENO,, false);
		return;
	}

	IEC_BOOL execute = __GET_VAR(data__->EXECUTE);
	IEC_BOOL execRising = execute && !__GET_VAR(data__->EXECUTE0);
	fb->EN = execute;

	/* 从MOTION读取反馈数据更新AXIS_REF */
	HDY_AXISMOTION motionData = __GET_VAR(data__->MOTION);
	fb->AXIS_REF.position = motionData.ACTPOSITION;
	fb->AXIS_REF.velocity = motionData.ACTVELOCITY;
	fb->AXIS_REF.flow     = motionData.ACTFLOW;
	fb->AXIS_REF.pressure = motionData.ACTPRESSURE;
	fb->AXIS_REF.timestamp = motionData.TIMESTAMP;

	if (execRising) {
		HDY_TIME currentTime = motionData.TIMESTAMP;

		/* 如果没有预加载的配方, 从MOTION构建1段配方并加载 */
		if (fb->RECIPE_SIZE == 0 && !fb->DIRECT_SEGMENT_VALID) {
			HDY_MotionSegment segment = buildSegmentFromMotion(&motionData);
			if (!HDY_MotionControlFB_LoadRecipe(fb, &segment, 1)) {
				__SET_VAR(data__->, ERROR,, true);
				__SET_VAR(data__->, ERRORID,,
						(IEC_WORD )HDY_DIAG_CODE_SEGMENT_INVALID);
				__SET_VAR(data__->, EXECUTE0,, execute);
				return;
			}
		}

		/* 启动配方 */
		if (fb->RECIPE_SIZE > 0) {
			if (!HDY_MotionControlFB_StartSegment(fb, 0, currentTime)) {
				__SET_VAR(data__->, ERROR,, true);
				__SET_VAR(data__->, ERRORID,,
						(IEC_WORD )HDY_DIAG_CODE_START_CONTEXT_INVALID);
				__SET_VAR(data__->, EXECUTE0,, execute);
				return;
			}
			__SET_VAR(data__->, GEN,, (IEC_WORD )fb->_commandGeneration);
		} else if (fb->DIRECT_SEGMENT_VALID) {
			/* 使用预加载的Direct段 */
			if (!HDY_MotionControlFB_StartSegment(fb, 0, currentTime)) {
				__SET_VAR(data__->, ERROR,, true);
				__SET_VAR(data__->, ERRORID,,
						(IEC_WORD )HDY_DIAG_CODE_START_CONTEXT_INVALID);
				__SET_VAR(data__->, EXECUTE0,, execute);
				return;
			}
			__SET_VAR(data__->, GEN,, (IEC_WORD )fb->_commandGeneration);
		}
	}

	/* 更新输出 */
	uint16_t myGen = (uint16_t) __GET_VAR(data__->GEN);
	bool isOwner = (myGen == fb->_commandGeneration);

	if (isOwner) {
		__SET_VAR(data__->, ACTIVE,, fb->ACTIVE ? true : false);
		__SET_VAR(data__->, BUSY,, fb->BUSY ? true : false);
		__SET_VAR(data__->, DONE,, (fb->DONE && fb->FINISHED) ? true : false);
		__SET_VAR(data__->, ERROR,, fb->ERROR ? true : false);
		__SET_VAR(data__->, ERRORID,, (IEC_WORD )fb->ERROR_ID);
		__SET_VAR(data__->, STATE,, (IEC_WORD )fb->STATE.status);
		__SET_VAR(data__->, PUMP_SPEED,, (IEC_REAL )fb->PUMP_SPEED);
		__SET_VAR(data__->, ENO,, true);

		/* 将活动段参数写回MOTION输出字段 (保留ACT*输入字段不变) */
		if (fb->_activeSegmentValid) {
			HDY_AXISMOTION motionOut = __GET_VAR(data__->MOTION);
			writeMotionFromSegment(&motionOut, fb);
			__SET_VAR(data__->, MOTION,, motionOut);
		}
	} else {
		/* 命令已被其他命令取代 */
		__SET_VAR(data__->, ACTIVE,, false);
		__SET_VAR(data__->, BUSY,, false);
		__SET_VAR(data__->, ENO,, true);
	}

	__SET_VAR(data__->, EXECUTE0,, execute);

}

/* ======================================================================
 * Stop (Direct模式) 命令实现
 * ====================================================================== */

void __mcl_cmd_Stop(HDY_STOP *data__)
{
    IEC_BOOL execute = __GET_VAR(data__->EXECUTE);
    IEC_BOOL execRising = execute && !__GET_VAR(data__->EXECUTE0);
    IEC_SINT axisIndex = __GET_VAR(data__->AXISID);

    __SET_VAR(data__->, ENO, , __GET_VAR(data__->EN));

    if (!__GET_VAR(data__->EN))
    {
        __SET_VAR(data__->, DONE, , false);
        __SET_VAR(data__->, BUSY, , false);
        __SET_VAR(data__->, COMMANDABORTED, , false);
        __SET_VAR(data__->, ERROR, , false);
        __SET_VAR(data__->, ERRORID, , 0);
        __SET_VAR(data__->, EXECUTE0, , execute);
        return;
    }

    /* 参数校验 */
    if (axisIndex < 0 || axisIndex >= HDY_MAX_AXIS_MOTION)
    {
        __SET_VAR(data__->, ERROR, , true);
        __SET_VAR(data__->, ERRORID, , (IEC_WORD)HDY_DIAG_CODE_START_CONTEXT_INVALID);
        __SET_VAR(data__->, EXECUTE0, , execute);
        return;
    }

    HDY_MotionControlFB* fb = &HDY_MotionControlFB_inst[axisIndex];
	if (fb == NULL) {
		__SET_VAR(data__->, ERROR,, true);
		__SET_VAR(data__->, ERRORID,, (IEC_WORD )HDY_DIAG_CODE_INTERNAL_ERROR);
		__SET_VAR(data__->, ENO,, false);
		__SET_VAR(data__->, EXECUTE0, , execute);
		return;
	}

    if (execRising)
    {
        HDY_MotionControlFB_Abort(fb);
        HDY_MotionControlFB_Scan(fb);
        __SET_VAR(data__->, GEN, , (IEC_WORD)fb->_commandGeneration);
        __SET_VAR(data__->, BUSY, , true);
        __SET_VAR(data__->, DONE, , false);
        __SET_VAR(data__->, COMMANDABORTED, , false);
    }

    /* 检查所有权和状态 */
    uint16_t myGen = (uint16_t)__GET_VAR(data__->GEN);
    bool isOwner = (myGen == fb->_commandGeneration);

    if (isOwner)
    {
        /* Stop完成: 轴已停止 (ABORTED或非ACTIVE) */
        if (!fb->ACTIVE && fb->FB_STATE != HDY_FB_STATE_RUNNING)
        {
            __SET_VAR(data__->, DONE, , true);
            __SET_VAR(data__->, BUSY, , false);
        }
        else
        {
            __SET_VAR(data__->, BUSY, , true);
        }

        if (fb->ERROR)
        {
            __SET_VAR(data__->, ERROR, , true);
            __SET_VAR(data__->, ERRORID, , (IEC_WORD)fb->ERROR_ID);
            __SET_VAR(data__->, BUSY, , false);
        }
    }
    else
    {
        /* Stop被其他命令取代 */
        __SET_VAR(data__->, COMMANDABORTED, , true);
        __SET_VAR(data__->, BUSY, , false);
        __SET_VAR(data__->, DONE, , false);
    }

    __SET_VAR(data__->, EXECUTE0, , execute);
}

/* ======================================================================
 * MoveAbsolute (Direct模式) 命令实现
 * ====================================================================== */

void __mcl_cmd_MoveAbsolute(HDY_MOVEABSOLUTE *data__)
{
    IEC_BOOL execute = __GET_VAR(data__->EXECUTE);
    IEC_BOOL execRising = execute && !__GET_VAR(data__->EXECUTE0);
    IEC_SINT axisIndex = __GET_VAR(data__->AXISID);

    __SET_VAR(data__->, ENO, , __GET_VAR(data__->EN));

    if (!__GET_VAR(data__->EN))
    {
        __SET_VAR(data__->, DONE, , false);
        __SET_VAR(data__->, BUSY, , false);
        __SET_VAR(data__->, ACTIVE, , false);
        __SET_VAR(data__->, COMMANDABORTED, , false);
        __SET_VAR(data__->, ERROR, , false);
        __SET_VAR(data__->, ERRORID, , 0);
        __SET_VAR(data__->, EXECUTE0, , execute);
        return;
    }

    /* 参数校验 */
    if (axisIndex < 0 || axisIndex >= HDY_MAX_AXIS_MOTION)
    {
        __SET_VAR(data__->, ERROR, , true);
        __SET_VAR(data__->, ERRORID, , (IEC_WORD)HDY_DIAG_CODE_START_CONTEXT_INVALID);
        __SET_VAR(data__->, EXECUTE0, , execute);
        return;
    }

    HDY_MotionControlFB* fb = &HDY_MotionControlFB_inst[axisIndex];
	if (fb == NULL) {
		__SET_VAR(data__->, ERROR,, true);
		__SET_VAR(data__->, ERRORID,, (IEC_WORD )HDY_DIAG_CODE_INTERNAL_ERROR);
		__SET_VAR(data__->, ENO,, false);
		__SET_VAR(data__->, EXECUTE0, , execute);
		return;
	}

    if (execRising)
    {
        /* 构建位置控制Direct段 */
        HDY_MotionDirection dir = mapPlcOpenDirection(__GET_VAR(data__->DIRECTION));
        HDY_MotionSegment segment = buildPositionSegment(
            __GET_VAR(data__->POSITION),
            __GET_VAR(data__->VELOCITY),
            __GET_VAR(data__->ACCELERATION),
            dir);

        /* 中止当前运动(递增_commandGeneration用于抢占检测)并加载新段 */
        fb->EN = true;
        HDY_MotionControlFB_Abort(fb);
        HDY_MotionControlFB_Scan(fb);

        if (!HDY_MotionControlFB_LoadDirectSegment(fb, &segment))
        {
            __SET_VAR(data__->, ERROR, , true);
            __SET_VAR(data__->, ERRORID, , (IEC_WORD)HDY_DIAG_CODE_SEGMENT_INVALID);
            __SET_VAR(data__->, EXECUTE0, , execute);
            return;
        }

        if (!HDY_MotionControlFB_StartSegment(fb, 0, fb->AXIS_REF.timestamp))
        {
            __SET_VAR(data__->, ERROR, , true);
            __SET_VAR(data__->, ERRORID, , (IEC_WORD)HDY_DIAG_CODE_START_CONTEXT_INVALID);
            __SET_VAR(data__->, EXECUTE0, , execute);
            return;
        }

        /* 记录代际作为所有权凭证 */
        __SET_VAR(data__->, GEN, , (IEC_WORD)fb->_commandGeneration);
        __SET_VAR(data__->, BUSY, , true);
        __SET_VAR(data__->, ACTIVE, , true);
        __SET_VAR(data__->, DONE, , false);
        __SET_VAR(data__->, COMMANDABORTED, , false);
        __SET_VAR(data__->, ACTIVE0, , true);
        __SET_VAR(data__->, EXECUTE0, , execute);
        return;
    }

    /* 检查所有权和执行状态 */
    uint16_t myGen = (uint16_t)__GET_VAR(data__->GEN);
    bool isOwner = (myGen == fb->_commandGeneration);
    bool wasActive = __GET_VAR(data__->ACTIVE0);

    if (isOwner)
    {
        if (fb->SEGMENT_COMPLETED || (fb->DONE && fb->FINISHED))
        {
            /* 位置到达 → DONE */
            __SET_VAR(data__->, DONE, , true);
            __SET_VAR(data__->, BUSY, , false);
            __SET_VAR(data__->, ACTIVE, , false);
        }
        else if (fb->ACTIVE)
        {
            __SET_VAR(data__->, BUSY, , true);
            __SET_VAR(data__->, ACTIVE, , true);
        }
        else if (fb->ERROR || fb->FAULT)
        {
            __SET_VAR(data__->, ERROR, , true);
            __SET_VAR(data__->, ERRORID, , (IEC_WORD)fb->ERROR_ID);
            __SET_VAR(data__->, BUSY, , false);
            __SET_VAR(data__->, ACTIVE, , false);
        }
        else
        {
            __SET_VAR(data__->, BUSY, , fb->BUSY ? true : false);
            __SET_VAR(data__->, ACTIVE, , fb->ACTIVE ? true : false);
        }
    }
    else if (wasActive)
    {
        /* 曾经是活跃命令, 但已被取代 → COMMANDABORTED */
        __SET_VAR(data__->, COMMANDABORTED, , true);
        __SET_VAR(data__->, BUSY, , false);
        __SET_VAR(data__->, ACTIVE, , false);
        __SET_VAR(data__->, DONE, , false);
    }

    __SET_VAR(data__->, ACTIVE0, , __GET_VAR(data__->ACTIVE));
    __SET_VAR(data__->, EXECUTE0, , execute);
}

/* ======================================================================
 * MoveVelocity (Direct模式) 命令实现
 * ====================================================================== */

void __mcl_cmd_MoveVelocity(HDY_MOVEVELOCITY *data__)
{
    IEC_BOOL execute = __GET_VAR(data__->EXECUTE);
    IEC_BOOL execRising = execute && !__GET_VAR(data__->EXECUTE0);
    IEC_SINT axisIndex = __GET_VAR(data__->AXISID);

    __SET_VAR(data__->, ENO, , __GET_VAR(data__->EN));

    if (!__GET_VAR(data__->EN))
    {
        __SET_VAR(data__->, INVELOCITY, , false);
        __SET_VAR(data__->, BUSY, , false);
        __SET_VAR(data__->, ACTIVE, , false);
        __SET_VAR(data__->, COMMANDABORTED, , false);
        __SET_VAR(data__->, ERROR, , false);
        __SET_VAR(data__->, ERRORID, , 0);
        __SET_VAR(data__->, EXECUTE0, , execute);
        return;
    }

    /* 参数校验 */
    if (axisIndex < 0 || axisIndex >= HDY_MAX_AXIS_MOTION)
    {
        __SET_VAR(data__->, ERROR, , true);
        __SET_VAR(data__->, ERRORID, , (IEC_WORD)HDY_DIAG_CODE_START_CONTEXT_INVALID);
        __SET_VAR(data__->, EXECUTE0, , execute);
        return;
    }


    HDY_MotionControlFB* fb = &HDY_MotionControlFB_inst[axisIndex];
	if (fb == NULL) {
		__SET_VAR(data__->, ERROR,, true);
		__SET_VAR(data__->, ERRORID,, (IEC_WORD )HDY_DIAG_CODE_INTERNAL_ERROR);
		__SET_VAR(data__->, ENO,, false);
		__SET_VAR(data__->, EXECUTE0, , execute);
		return;
	}
    HDY_REAL targetVelocity = __GET_VAR(data__->VELOCITY);

    if (execRising)
    {
        /* 构建速度控制Direct段 */
        HDY_MotionDirection dir = mapPlcOpenDirection(__GET_VAR(data__->DIRECTION));
        HDY_MotionSegment segment = buildVelocitySegment(
            targetVelocity,
            __GET_VAR(data__->ACCELERATION),
            dir);

        /* 中止当前运动(递增_commandGeneration用于抢占检测)并加载新段 */
        fb->EN = true;
        HDY_MotionControlFB_Abort(fb);
        HDY_MotionControlFB_Scan(fb);

        if (!HDY_MotionControlFB_LoadDirectSegment(fb, &segment))
        {
            __SET_VAR(data__->, ERROR, , true);
            __SET_VAR(data__->, ERRORID, , (IEC_WORD)HDY_DIAG_CODE_SEGMENT_INVALID);
            __SET_VAR(data__->, EXECUTE0, , execute);
            return;
        }

        if (!HDY_MotionControlFB_StartSegment(fb, 0, fb->AXIS_REF.timestamp))
        {
            __SET_VAR(data__->, ERROR, , true);
            __SET_VAR(data__->, ERRORID, , (IEC_WORD)HDY_DIAG_CODE_START_CONTEXT_INVALID);
            __SET_VAR(data__->, EXECUTE0, , execute);
            return;
        }

        /* 记录代际作为所有权凭证 */
        __SET_VAR(data__->, GEN, , (IEC_WORD)fb->_commandGeneration);
        __SET_VAR(data__->, BUSY, , true);
        __SET_VAR(data__->, ACTIVE, , true);
        __SET_VAR(data__->, INVELOCITY, , false);
        __SET_VAR(data__->, COMMANDABORTED, , false);
        __SET_VAR(data__->, ACTIVE0, , true);
        __SET_VAR(data__->, EXECUTE0, , execute);
        return;
    }

    /* 检查所有权和执行状态 */
    uint16_t myGen = (uint16_t)__GET_VAR(data__->GEN);
    bool isOwner = (myGen == fb->_commandGeneration);
    bool wasActive = __GET_VAR(data__->ACTIVE0);

    if (isOwner)
    {
        if (fb->ACTIVE)
        {
            __SET_VAR(data__->, BUSY, , true);
            __SET_VAR(data__->, ACTIVE, , true);

            /* INVELOCITY检测: 实际速度接近目标速度 */
            HDY_REAL velError = fb->AXIS_REF.velocity - targetVelocity;
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
        else if (fb->ERROR || fb->FAULT)
        {
            __SET_VAR(data__->, ERROR, , true);
            __SET_VAR(data__->, ERRORID, , (IEC_WORD)fb->ERROR_ID);
            __SET_VAR(data__->, BUSY, , false);
            __SET_VAR(data__->, ACTIVE, , false);
            __SET_VAR(data__->, INVELOCITY, , false);
        }
        else
        {
            __SET_VAR(data__->, BUSY, , fb->BUSY ? true : false);
            __SET_VAR(data__->, ACTIVE, , fb->ACTIVE ? true : false);
        }
    }
    else if (wasActive)
    {
        /* 曾经是活跃命令, 但已被取代 → COMMANDABORTED */
        __SET_VAR(data__->, COMMANDABORTED, , true);
        __SET_VAR(data__->, BUSY, , false);
        __SET_VAR(data__->, ACTIVE, , false);
        __SET_VAR(data__->, INVELOCITY, , false);
    }

    __SET_VAR(data__->, ACTIVE0, , __GET_VAR(data__->ACTIVE));
    __SET_VAR(data__->, INVELOCITY0, , __GET_VAR(data__->INVELOCITY));
    __SET_VAR(data__->, EXECUTE0, , execute);
}

/* ======================================================================
 * Reset (Direct模式) 命令实现
 * ====================================================================== */

void __mcl_cmd_Reset(HDY_RESET *data__)
{
    IEC_BOOL execute = __GET_VAR(data__->EXECUTE);
    IEC_BOOL execRising = execute && !__GET_VAR(data__->EXECUTE0);
    IEC_SINT axisIndex = __GET_VAR(data__->AXISID);

    __SET_VAR(data__->, ENO, , __GET_VAR(data__->EN));

    if (!__GET_VAR(data__->EN))
    {
        __SET_VAR(data__->, DONE, , false);
        __SET_VAR(data__->, BUSY, , false);
        __SET_VAR(data__->, ERROR, , false);
        __SET_VAR(data__->, ERRORID, , 0);
        __SET_VAR(data__->, EXECUTE0, , execute);
        return;
    }

    /* 参数校验 */
    if (axisIndex < 0 || axisIndex >= HDY_MAX_AXIS_MOTION)
    {
        __SET_VAR(data__->, ERROR, , true);
        __SET_VAR(data__->, ERRORID, , (IEC_WORD)HDY_DIAG_CODE_START_CONTEXT_INVALID);
        __SET_VAR(data__->, EXECUTE0, , execute);
        return;
    }

    /* 直接访问FB实例 (Direct和MoveProfile共用同一个实例池) */
    HDY_MotionControlFB* fb = &HDY_MotionControlFB_inst[axisIndex];
	if (fb == NULL) {
		__SET_VAR(data__->, ERROR,, true);
		__SET_VAR(data__->, ERRORID,, (IEC_WORD )HDY_DIAG_CODE_INTERNAL_ERROR);
		__SET_VAR(data__->, ENO,, false);
		__SET_VAR(data__->, EXECUTE0, , execute);
		return;
	}
    if (fb->FB_STATE == HDY_FB_STATE_DISABLED)
    {
        /* FB未初始化, Reset无意义, 直接返回DONE */
        __SET_VAR(data__->, DONE, , true);
        __SET_VAR(data__->, BUSY, , false);
        __SET_VAR(data__->, EXECUTE0, , execute);
        return;
    }

    if (execRising)
    {
        /* 执行SoftReset: 保留配方/配置, 清除运行时状态和故障 */
        HDY_MotionControlFB_SoftReset(fb);
        fb->EN = true;

        /* 清除代际, SoftReset中的memset已将_commandGeneration归零 */
        __SET_VAR(data__->, GEN, , (IEC_WORD)0);

        __SET_VAR(data__->, DONE, , true);
        __SET_VAR(data__->, BUSY, , false);
    }

    __SET_VAR(data__->, EXECUTE0, , execute);
}

/* ======================================================================
 * PressureHandle (Direct模式) 命令实现
 * ====================================================================== */

void __mcl_cmd_PressureHandle(HDY_PRESSUREHANDLE *data__)
{
    IEC_BOOL execute = __GET_VAR(data__->EXECUTE);
    IEC_BOOL execRising = execute && !__GET_VAR(data__->EXECUTE0);
    IEC_SINT axisIndex = __GET_VAR(data__->AXISID);

    __SET_VAR(data__->, ENO, , __GET_VAR(data__->EN));

    if (!__GET_VAR(data__->EN))
    {
        __SET_VAR(data__->, INPRESSURE, , false);
        __SET_VAR(data__->, BUSY, , false);
        __SET_VAR(data__->, ACTIVE, , false);
        __SET_VAR(data__->, COMMANDABORTED, , false);
        __SET_VAR(data__->, ERROR, , false);
        __SET_VAR(data__->, ERRORID, , 0);
        __SET_VAR(data__->, EXECUTE0, , execute);
        return;
    }

    /* 参数校验 */
    if (axisIndex < 0 || axisIndex >= HDY_MAX_AXIS_MOTION)
    {
        __SET_VAR(data__->, ERROR, , true);
        __SET_VAR(data__->, ERRORID, , (IEC_WORD)HDY_DIAG_CODE_START_CONTEXT_INVALID);
        __SET_VAR(data__->, EXECUTE0, , execute);
        return;
    }

    HDY_MotionControlFB* fb = &HDY_MotionControlFB_inst[axisIndex];
	if (fb == NULL) {
		__SET_VAR(data__->, ERROR,, true);
		__SET_VAR(data__->, ERRORID,, (IEC_WORD )HDY_DIAG_CODE_INTERNAL_ERROR);
		__SET_VAR(data__->, ENO,, false);
		__SET_VAR(data__->, EXECUTE0, , execute);
		return;
	}
    HDY_REAL targetPressure = __GET_VAR(data__->PRESSURE);

    if (execRising)
    {
        /* 构建压力控制Direct段 */
        HDY_MotionSegment segment = buildPressureSegment(
            targetPressure,
            __GET_VAR(data__->PRESSURERAMPRATE),
            __GET_VAR(data__->DURATION));

        /* 中止当前运动(递增_commandGeneration用于抢占检测)并加载新段 */
        fb->EN = true;
        HDY_MotionControlFB_Abort(fb);
        HDY_MotionControlFB_Scan(fb);

        if (!HDY_MotionControlFB_LoadDirectSegment(fb, &segment))
        {
            __SET_VAR(data__->, ERROR, , true);
            __SET_VAR(data__->, ERRORID, , (IEC_WORD)HDY_DIAG_CODE_SEGMENT_INVALID);
            __SET_VAR(data__->, EXECUTE0, , execute);
            return;
        }

        if (!HDY_MotionControlFB_StartSegment(fb, 0, fb->AXIS_REF.timestamp))
        {
            __SET_VAR(data__->, ERROR, , true);
            __SET_VAR(data__->, ERRORID, , (IEC_WORD)HDY_DIAG_CODE_START_CONTEXT_INVALID);
            __SET_VAR(data__->, EXECUTE0, , execute);
            return;
        }

        /* 记录代际作为所有权凭证 */
        __SET_VAR(data__->, GEN, , (IEC_WORD)fb->_commandGeneration);
        __SET_VAR(data__->, BUSY, , true);
        __SET_VAR(data__->, ACTIVE, , true);
        __SET_VAR(data__->, INPRESSURE, , false);
        __SET_VAR(data__->, COMMANDABORTED, , false);
        __SET_VAR(data__->, ACTIVE0, , true);
        __SET_VAR(data__->, EXECUTE0, , execute);
        return;
    }

    /* 检查所有权和执行状态 */
    uint16_t myGen = (uint16_t)__GET_VAR(data__->GEN);
    bool isOwner = (myGen == fb->_commandGeneration);
    bool wasActive = __GET_VAR(data__->ACTIVE0);

    if (isOwner)
    {
        if (fb->SEGMENT_COMPLETED || (fb->DONE && fb->FINISHED))
        {
            /* 段完成 → DONE (对于END_TIME模式, 持续时间到达) */
            __SET_VAR(data__->, BUSY, , false);
            __SET_VAR(data__->, ACTIVE, , false);
            __SET_VAR(data__->, INPRESSURE, , false);
        }
        else if (fb->ACTIVE)
        {
            __SET_VAR(data__->, BUSY, , true);
            __SET_VAR(data__->, ACTIVE, , true);

            /* INPRESSURE检测: 实际压力接近目标压力 */
            HDY_REAL pressError = fb->AXIS_REF.pressure - targetPressure;
            if (pressError < 0.0f) pressError = -pressError;
            if (targetPressure > 0.0f && pressError < 0.5f)  /* 0.5 MPa容差 */
            {
                __SET_VAR(data__->, INPRESSURE, , true);
            }
            else
            {
                __SET_VAR(data__->, INPRESSURE, , false);
            }
        }
        else if (fb->ERROR || fb->FAULT)
        {
            __SET_VAR(data__->, ERROR, , true);
            __SET_VAR(data__->, ERRORID, , (IEC_WORD)fb->ERROR_ID);
            __SET_VAR(data__->, BUSY, , false);
            __SET_VAR(data__->, ACTIVE, , false);
            __SET_VAR(data__->, INPRESSURE, , false);
        }
        else
        {
            __SET_VAR(data__->, BUSY, , fb->BUSY ? true : false);
            __SET_VAR(data__->, ACTIVE, , fb->ACTIVE ? true : false);
        }
    }
    else if (wasActive)
    {
        /* 曾经是活跃命令, 但已被取代 → COMMANDABORTED */
        __SET_VAR(data__->, COMMANDABORTED, , true);
        __SET_VAR(data__->, BUSY, , false);
        __SET_VAR(data__->, ACTIVE, , false);
        __SET_VAR(data__->, INPRESSURE, , false);
    }

    __SET_VAR(data__->, ACTIVE0, , __GET_VAR(data__->ACTIVE));
    __SET_VAR(data__->, INPRESSURE0, , __GET_VAR(data__->INPRESSURE));
    __SET_VAR(data__->, EXECUTE0, , execute);
}
