#include "motion_interface.h"
#include <string.h>
#include <stdio.h>

/* ======================================================================
 * FB实例池管理
 * ====================================================================== */

HDY_MotionControlFB HDY_MotionControlFB_inst[HDY_MAX_AXIS_MOTION];

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
 * 命令仲裁状态
 *
 * 每个轴跟踪当前活跃的命令类型和代际计数器(generation)。
 * 当一个新命令接管轴时，代际计数器递增，前一个命令在下次扫描时
 * 检测到代际不匹配，自动设置COMMANDABORTED输出。
 *
 * 仲裁规则:
 *   - MoveProfile (Recipe模式) 与 Direct模式命令互斥
 *   - 新命令可以接管正在执行的旧命令
 *   - 被接管的命令输出COMMANDABORTED=true, BUSY=false
 * ====================================================================== */

typedef enum {
    IFACE_CMD_NONE = 0,
    IFACE_CMD_MOVE_PROFILE,      /* MoveProfile - Recipe模式 */
    IFACE_CMD_MOVE_ABSOLUTE,     /* MoveAbsolute - Direct模式 */
    IFACE_CMD_MOVE_VELOCITY,     /* MoveVelocity - Direct模式 */
    IFACE_CMD_STOP,              /* Stop - Direct模式 */
    IFACE_CMD_PRESSURE_HANDLE    /* PressureHandle - Direct模式 */
} IfaceCommandType;

#define IFACE_CMD_TYPE_COUNT 6

static struct {
    IfaceCommandType activeCommand;
    uint16_t generation;
    bool initialized;
} axisState[HDY_MAX_AXIS_MOTION];

/* 每个IEC FB实例记录自己启动时的代际，用于检测命令是否被取代 */
static uint16_t fbGenerations[HDY_MAX_AXIS_MOTION][IFACE_CMD_TYPE_COUNT];

/* ======================================================================
 * 内部辅助函数
 * ====================================================================== */

/* 确保指定轴的FB实例已正确初始化 */
static bool ensureFbInitialized(int axisIndex)
{
    HDY_MotionControlFB* fb;

    if (axisIndex < 0 || axisIndex >= HDY_MAX_AXIS_MOTION) {
        return false;
    }

    if (axisState[axisIndex].initialized) {
        return true;
    }

    fb = &HDY_MotionControlFB_inst[axisIndex];
    HDY_MotionControlFB_Init(fb);
    fb->USE_RECIPE = false;
    fb->FLOW_TO_PUMP_SPEED_GAIN = 1.2f;  /* rpm per L/min */
    fb->PUMP_SPEED_LIMIT = 3000.0f;       /* rpm */
    fb->EN = true;

    axisState[axisIndex].initialized = true;
    return true;
}

/* 从PLCopen方向值映射到HDY_MotionDirection */
static HDY_MotionDirection mapPlcOpenDirection(IEC_SINT direction)
{
    /* PLCopen方向约定 (注塑机液压场景):
     *  正数 → EXTEND (伸出/正方向, 如合模/射胶)
     *  负数 → RETRACT (缩回/负方向, 如开模/射退)
     *  0   → AUTO (由目标位置自动推断)
     */
    if (direction > 0) {
        return HDY_DIRECTION_EXTEND;
    } else if (direction < 0) {
        return HDY_DIRECTION_RETRACT;
    }
    return HDY_DIRECTION_AUTO;
}

/* 命令仲裁：接管轴的所有权，返回新的代际值 */
static uint16_t takeAxisOwnership(int axisIndex, IfaceCommandType cmdType)
{
    axisState[axisIndex].generation++;
    axisState[axisIndex].activeCommand = cmdType;
    fbGenerations[axisIndex][cmdType] = axisState[axisIndex].generation;
    return axisState[axisIndex].generation;
}

/* 检查指定命令是否仍是该轴的活跃命令 */
static bool isAxisOwner(int axisIndex, IfaceCommandType cmdType, uint16_t gen)
{
    return (axisState[axisIndex].activeCommand == cmdType &&
            fbGenerations[axisIndex][cmdType] == gen);
}

/* 释放轴的所有权 */
static void releaseAxisOwnership(int axisIndex)
{
    axisState[axisIndex].activeCommand = IFACE_CMD_NONE;
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
    memset(axisState, 0, sizeof(axisState));
    memset(fbGenerations, 0, sizeof(fbGenerations));
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
    for (int i = 0; i < (int)nextAllocatedFB; i++) {
        HDY_MotionControlFB* fb = &HDY_MotionControlFB_inst[i];
        if (fb != NULL) {
            HDY_MotionControlFB_Scan(fb);
        }
    }
}

/* ======================================================================
 * MoveProfile (Recipe模式) 命令实现
 *
 * Recipe模式工作流程:
 * 1. INIT阶段: 分配FB实例, 设置USE_RECIPE=true
 * 2. 配方加载: 若无预加载配方, 从MOTION构建1段配方自动加载
 * 3. EXECUTE上升沿: 启动配方执行
 * 4. 周期执行: 通过MOTION.ACT*更新AXIS_REF反馈, 读取输出状态
 * 5. DONE: 配方最后一段完成
 *
 * 注: 多段配方需通过外部HDY_MotionControlFB_LoadRecipe()预加载
 * ====================================================================== */

void __mcl_cmd_LoadProfile(HDY_LOADPROFILE *data__)
{
    // TODO: 实现预加载配方, 目前仅支持单段MoveProfile的自动构建和执行,待后面补充完善
    IEC_SINT axisIndex = __GET_VAR(data__->AXISINDEX);
    HDY_MotionControlFB* fb = __MK_GetPublic_MotionControlFB(axisIndex);
    
    if (fb != NULL) {
        // 预加载配方逻辑
    }
}

void __mcl_cmd_MoveProfile(HDY_MOVEPROFILE *data__)
{
    IEC_BOOL bInit = __GET_VAR(data__->INIT);

    if (!bInit)
    {
        /* 首次调用: 分配并初始化FB实例 */
        int index = allocMotionControlFB();
        if (index >= 0)
        {
            HDY_MotionControlFB* fb = &HDY_MotionControlFB_inst[index];
            HDY_MotionControlFB_Init(fb);
            fb->USE_RECIPE = true;  /* Recipe模式 */
            fb->FLOW_TO_PUMP_SPEED_GAIN = 1.2f;
            fb->PUMP_SPEED_LIMIT = 3000.0f;
            fb->EN = true;

            __SET_VAR(data__->, INIT, , true);
            __SET_VAR(data__->, AXISINDEX, , (IEC_SINT)index);

            axisState[index].initialized = true;
            takeAxisOwnership(index, IFACE_CMD_MOVE_PROFILE);
        }
    }
    else
    {
        /* 后续周期调用 */
        IEC_SINT axisIndex = __GET_VAR(data__->AXISINDEX);
        HDY_MotionControlFB* fb = __MK_GetPublic_MotionControlFB(axisIndex);

        if (fb == NULL)
        {
            __SET_VAR(data__->, ERROR, , true);
            __SET_VAR(data__->, ERRORID, , (IEC_WORD)HDY_DIAG_CODE_INTERNAL_ERROR);
            __SET_VAR(data__->, ENO, , false);
            return;
        }

        IEC_BOOL execute = __GET_VAR(data__->EXECUTE);
        IEC_BOOL execRising = execute && !__GET_VAR(data__->EXECUTE0);
        fb->EN = execute;

        /* 从MOTION读取反馈数据更新AXIS_REF */
        HDY_AXISMOTION motionData = __GET_VAR(data__->MOTION);
        fb->AXIS_REF.position = motionData.ACTPOSITION;
        fb->AXIS_REF.velocity = motionData.ACTVELOCITY;
        fb->AXIS_REF.flow = motionData.ACTFLOW;
        fb->AXIS_REF.pressure = motionData.ACTPRESSURE;
        fb->AXIS_REF.timestamp = motionData.TIMESTAMP;

        if (execRising)
        {
            HDY_TIME currentTime = motionData.TIMESTAMP;

            /* 如果没有预加载的配方, 从MOTION构建1段配方并加载 */
            if (fb->RECIPE_SIZE == 0 && !fb->DIRECT_SEGMENT_VALID)
            {
                HDY_MotionSegment segment = buildSegmentFromMotion(&motionData);
                if (!HDY_MotionControlFB_LoadRecipe(fb, &segment, 1))
                {
                    __SET_VAR(data__->, ERROR, , true);
                    __SET_VAR(data__->, ERRORID, , (IEC_WORD)HDY_DIAG_CODE_SEGMENT_INVALID);
                    __SET_VAR(data__->, EXECUTE0, , execute);
                    return;
                }
            }

            /* 启动配方 */
            if (fb->RECIPE_SIZE > 0)
            {
                if (!HDY_MotionControlFB_StartSegment(fb, 0, currentTime))
                {
                    __SET_VAR(data__->, ERROR, , true);
                    __SET_VAR(data__->, ERRORID, , (IEC_WORD)HDY_DIAG_CODE_START_CONTEXT_INVALID);
                    __SET_VAR(data__->, EXECUTE0, , execute);
                    return;
                }
                takeAxisOwnership((int)axisIndex, IFACE_CMD_MOVE_PROFILE);
            }
            else if (fb->DIRECT_SEGMENT_VALID)
            {
                /* 使用预加载的Direct段 */
                if (!HDY_MotionControlFB_StartSegment(fb, 0, currentTime))
                {
                    __SET_VAR(data__->, ERROR, , true);
                    __SET_VAR(data__->, ERRORID, , (IEC_WORD)HDY_DIAG_CODE_START_CONTEXT_INVALID);
                    __SET_VAR(data__->, EXECUTE0, , execute);
                    return;
                }
                takeAxisOwnership((int)axisIndex, IFACE_CMD_MOVE_PROFILE);
            }
        }

        /* 更新输出 */
        uint16_t myGen = fbGenerations[(int)axisIndex][IFACE_CMD_MOVE_PROFILE];
        bool isOwner = isAxisOwner((int)axisIndex, IFACE_CMD_MOVE_PROFILE, myGen);

        if (isOwner)
        {
            __SET_VAR(data__->, ACTIVE, , fb->ACTIVE ? true : false);
            __SET_VAR(data__->, BUSY, , fb->BUSY ? true : false);
            __SET_VAR(data__->, DONE, , (fb->DONE && fb->FINISHED) ? true : false);
            __SET_VAR(data__->, ERROR, , fb->ERROR ? true : false);
            __SET_VAR(data__->, ERRORID, , (IEC_WORD)fb->ERROR_ID);
            __SET_VAR(data__->, STATE, , (IEC_WORD)fb->STATE.status);
            __SET_VAR(data__->, PUMP_SPEED, , (IEC_REAL)fb->PUMP_SPEED);
            __SET_VAR(data__->, ENO, , true);

            /* 配方完成后释放所有权 */
            if (fb->DONE && fb->FINISHED)
            {
                releaseAxisOwnership((int)axisIndex);
            }

            /* 将活动段参数写回MOTION输出字段 (保留ACT*输入字段不变) */
            if (fb->_activeSegmentValid)
            {
                HDY_AXISMOTION motionOut = __GET_VAR(data__->MOTION);
                writeMotionFromSegment(&motionOut, fb);
                __SET_VAR(data__->, MOTION, , motionOut);
            }
        }
        else
        {
            /* 命令已被其他命令取代 */
            __SET_VAR(data__->, ACTIVE, , false);
            __SET_VAR(data__->, BUSY, , false);
            __SET_VAR(data__->, ENO, , true);
        }

        __SET_VAR(data__->, EXECUTE0, , execute);
    }
}

/* ======================================================================
 * Stop (Direct模式) 命令实现
 *
 * PLCopen Stop语义: 中止当前轴上所有运动命令, 输出安全零值。
 * 当前实现为立即中止(Abort), 未来可扩展为受控减速停止。
 * ====================================================================== */

void __mcl_cmd_Stop(HDY_STOP *data__)
{
    IEC_BOOL execute = __GET_VAR(data__->EXECUTE);
    IEC_BOOL execRising = execute && !__GET_VAR(data__->EXECUTE0);
    IEC_SINT axisIndex = __GET_VAR(data__->AXISINDEX);

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

    /* 确保FB实例已初始化 */
    if (!ensureFbInitialized((int)axisIndex))
    {
        __SET_VAR(data__->, ERROR, , true);
        __SET_VAR(data__->, ERRORID, , (IEC_WORD)HDY_DIAG_CODE_INTERNAL_ERROR);
        __SET_VAR(data__->, EXECUTE0, , execute);
        return;
    }

    HDY_MotionControlFB* fb = &HDY_MotionControlFB_inst[axisIndex];

    if (execRising)
    {
        /* 接管轴所有权并中止当前运动 */
        takeAxisOwnership((int)axisIndex, IFACE_CMD_STOP);
        HDY_MotionControlFB_Abort(fb);
        __SET_VAR(data__->, BUSY, , true);
        __SET_VAR(data__->, DONE, , false);
        __SET_VAR(data__->, COMMANDABORTED, , false);
    }

    /* 检查所有权和状态 */
    uint16_t myGen = fbGenerations[(int)axisIndex][IFACE_CMD_STOP];
    bool isOwner = isAxisOwner((int)axisIndex, IFACE_CMD_STOP, myGen);

    if (isOwner)
    {
        /* Stop完成: 轴已停止 (ABORTED或非ACTIVE) */
        if (!fb->ACTIVE && fb->FB_STATE != HDY_FB_STATE_RUNNING)
        {
            __SET_VAR(data__->, DONE, , true);
            __SET_VAR(data__->, BUSY, , false);
            releaseAxisOwnership((int)axisIndex);
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
            releaseAxisOwnership((int)axisIndex);
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
 *
 * Direct模式: 直接指定目标位置/速度/加速度, 构建单段Direct段执行。
 * DONE: 到达目标位置
 * COMMANDABORTED: 被其他命令取代
 *
 * 注意: CONTINUOUSUPDATE和JERK参数当前未实现, 参数在EXECUTE时锁存。
 * ====================================================================== */

void __mcl_cmd_MoveAbsolute(HDY_MOVEABSOLUTE *data__)
{
    IEC_BOOL execute = __GET_VAR(data__->EXECUTE);
    IEC_BOOL execRising = execute && !__GET_VAR(data__->EXECUTE0);
    IEC_SINT axisIndex = __GET_VAR(data__->AXISINDEX);

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

    if (!ensureFbInitialized((int)axisIndex))
    {
        __SET_VAR(data__->, ERROR, , true);
        __SET_VAR(data__->, ERRORID, , (IEC_WORD)HDY_DIAG_CODE_INTERNAL_ERROR);
        __SET_VAR(data__->, EXECUTE0, , execute);
        return;
    }

    HDY_MotionControlFB* fb = &HDY_MotionControlFB_inst[axisIndex];

    if (execRising)
    {
        /* 构建位置控制Direct段 */
        HDY_MotionDirection dir = mapPlcOpenDirection(__GET_VAR(data__->DIRECTION));
        HDY_MotionSegment segment = buildPositionSegment(
            __GET_VAR(data__->POSITION),
            __GET_VAR(data__->VELOCITY),
            __GET_VAR(data__->ACCELERATION),
            dir);

        /* 接管轴所有权 */
        takeAxisOwnership((int)axisIndex, IFACE_CMD_MOVE_ABSOLUTE);

        /* 中止当前运动并加载新段 */
        fb->EN = true;
        if (fb->ACTIVE || fb->BUSY)
        {
            HDY_MotionControlFB_Abort(fb);
            HDY_MotionControlFB_Scan(fb);
        }

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

        __SET_VAR(data__->, BUSY, , true);
        __SET_VAR(data__->, ACTIVE, , true);
        __SET_VAR(data__->, DONE, , false);
        __SET_VAR(data__->, COMMANDABORTED, , false);
    }

    /* 检查所有权和执行状态 */
    uint16_t myGen = fbGenerations[(int)axisIndex][IFACE_CMD_MOVE_ABSOLUTE];
    bool isOwner = isAxisOwner((int)axisIndex, IFACE_CMD_MOVE_ABSOLUTE, myGen);
    bool wasActive = __GET_VAR(data__->ACTIVE0);

    if (isOwner)
    {
        if (fb->SEGMENT_COMPLETED || (fb->DONE && fb->FINISHED))
        {
            /* 位置到达 → DONE */
            __SET_VAR(data__->, DONE, , true);
            __SET_VAR(data__->, BUSY, , false);
            __SET_VAR(data__->, ACTIVE, , false);
            releaseAxisOwnership((int)axisIndex);
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
            releaseAxisOwnership((int)axisIndex);
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
 *
 * Direct模式: 直接指定目标速度/加速度, 构建速度斜坡Direct段执行。
 * INVELOCITY: 实际速度到达目标速度 (检测AXIS_REF.velocity ≈ targetVelocity)
 * COMMANDABORTED: 被其他命令取代
 *
 * 注意: 速度控制为持续模式(END_MANUAL), 需通过Stop命令停止。
 * ====================================================================== */

void __mcl_cmd_MoveVelocity(HDY_MOVEVELOCITY *data__)
{
    IEC_BOOL execute = __GET_VAR(data__->EXECUTE);
    IEC_BOOL execRising = execute && !__GET_VAR(data__->EXECUTE0);
    IEC_SINT axisIndex = __GET_VAR(data__->AXISINDEX);

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

    if (!ensureFbInitialized((int)axisIndex))
    {
        __SET_VAR(data__->, ERROR, , true);
        __SET_VAR(data__->, ERRORID, , (IEC_WORD)HDY_DIAG_CODE_INTERNAL_ERROR);
        __SET_VAR(data__->, EXECUTE0, , execute);
        return;
    }

    HDY_MotionControlFB* fb = &HDY_MotionControlFB_inst[axisIndex];
    HDY_REAL targetVelocity = __GET_VAR(data__->VELOCITY);

    if (execRising)
    {
        /* 构建速度控制Direct段 */
        HDY_MotionDirection dir = mapPlcOpenDirection(__GET_VAR(data__->DIRECTION));
        HDY_MotionSegment segment = buildVelocitySegment(
            targetVelocity,
            __GET_VAR(data__->ACCELERATION),
            dir);

        /* 接管轴所有权 */
        takeAxisOwnership((int)axisIndex, IFACE_CMD_MOVE_VELOCITY);

        /* 中止当前运动并加载新段 */
        fb->EN = true;
        if (fb->ACTIVE || fb->BUSY)
        {
            HDY_MotionControlFB_Abort(fb);
            HDY_MotionControlFB_Scan(fb);
        }

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

        __SET_VAR(data__->, BUSY, , true);
        __SET_VAR(data__->, ACTIVE, , true);
        __SET_VAR(data__->, INVELOCITY, , false);
        __SET_VAR(data__->, COMMANDABORTED, , false);
    }

    /* 检查所有权和执行状态 */
    uint16_t myGen = fbGenerations[(int)axisIndex][IFACE_CMD_MOVE_VELOCITY];
    bool isOwner = isAxisOwner((int)axisIndex, IFACE_CMD_MOVE_VELOCITY, myGen);
    bool wasActive = __GET_VAR(data__->ACTIVE0);

    if (isOwner)
    {
        if (fb->ACTIVE)
        {
            __SET_VAR(data__->, BUSY, , true);
            __SET_VAR(data__->, ACTIVE, , true);

            /* INVELOCITY检测: 实际速度接近目标速度 */
            HDY_REAL velError = fb->AXIS_REF.velocity - targetVelocity;
            if (velError < 0.0f) velError = -velError;  /* fabs */
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
            releaseAxisOwnership((int)axisIndex);
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
 *
 * 复位FB实例: 清除故障状态, 恢复到READY/IDLE态。
 * 保留配方、配置增益和诊断判据设置。
 * DONE: 复位完成 (SoftReset是同步操作, 立即完成)
 * ====================================================================== */

void __mcl_cmd_Reset(HDY_RESET *data__)
{
    IEC_BOOL execute = __GET_VAR(data__->EXECUTE);
    IEC_BOOL execRising = execute && !__GET_VAR(data__->EXECUTE0);
    IEC_SINT axisIndex = __GET_VAR(data__->AXISINDEX);

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

    if (!axisState[(int)axisIndex].initialized)
    {
        /* FB未初始化, Reset无意义, 直接返回DONE */
        __SET_VAR(data__->, DONE, , true);
        __SET_VAR(data__->, BUSY, , false);
        __SET_VAR(data__->, EXECUTE0, , execute);
        return;
    }

    HDY_MotionControlFB* fb = &HDY_MotionControlFB_inst[axisIndex];

    if (execRising)
    {
        /* 执行SoftReset: 保留配方/配置, 清除运行时状态和故障 */
        HDY_MotionControlFB_SoftReset(fb);
        fb->EN = true;

        /* 释放轴所有权 */
        releaseAxisOwnership((int)axisIndex);

        __SET_VAR(data__->, DONE, , true);
        __SET_VAR(data__->, BUSY, , false);
    }

    __SET_VAR(data__->, EXECUTE0, , execute);
}

/* ======================================================================
 * PressureHandle (Direct模式) 命令实现
 *
 * Direct模式: 直接指定目标压力/斜坡速率/持续时间, 构建压力闭环Direct段执行。
 * INPRESSURE: 实际压力到达目标压力 (在容差范围内)
 * COMMANDABORTED: 被其他命令取代
 * ====================================================================== */

void __mcl_cmd_PressureHandle(HDY_PRESSUREHANDLE *data__)
{
    IEC_BOOL execute = __GET_VAR(data__->EXECUTE);
    IEC_BOOL execRising = execute && !__GET_VAR(data__->EXECUTE0);
    IEC_SINT axisIndex = __GET_VAR(data__->AXISINDEX);

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

    if (!ensureFbInitialized((int)axisIndex))
    {
        __SET_VAR(data__->, ERROR, , true);
        __SET_VAR(data__->, ERRORID, , (IEC_WORD)HDY_DIAG_CODE_INTERNAL_ERROR);
        __SET_VAR(data__->, EXECUTE0, , execute);
        return;
    }

    HDY_MotionControlFB* fb = &HDY_MotionControlFB_inst[axisIndex];
    HDY_REAL targetPressure = __GET_VAR(data__->PRESSURE);

    if (execRising)
    {
        /* 构建压力控制Direct段 */
        HDY_MotionSegment segment = buildPressureSegment(
            targetPressure,
            __GET_VAR(data__->PRESSURERAMPRATE),
            __GET_VAR(data__->DURATION));

        /* 接管轴所有权 */
        takeAxisOwnership((int)axisIndex, IFACE_CMD_PRESSURE_HANDLE);

        /* 中止当前运动并加载新段 */
        fb->EN = true;
        if (fb->ACTIVE || fb->BUSY)
        {
            HDY_MotionControlFB_Abort(fb);
            HDY_MotionControlFB_Scan(fb);
        }

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

        __SET_VAR(data__->, BUSY, , true);
        __SET_VAR(data__->, ACTIVE, , true);
        __SET_VAR(data__->, INPRESSURE, , false);
        __SET_VAR(data__->, COMMANDABORTED, , false);
    }

    /* 检查所有权和执行状态 */
    uint16_t myGen = fbGenerations[(int)axisIndex][IFACE_CMD_PRESSURE_HANDLE];
    bool isOwner = isAxisOwner((int)axisIndex, IFACE_CMD_PRESSURE_HANDLE, myGen);
    bool wasActive = __GET_VAR(data__->ACTIVE0);

    if (isOwner)
    {
        if (fb->SEGMENT_COMPLETED || (fb->DONE && fb->FINISHED))
        {
            /* 段完成 → DONE (对于END_TIME模式, 持续时间到达) */
            __SET_VAR(data__->, BUSY, , false);
            __SET_VAR(data__->, ACTIVE, , false);
            __SET_VAR(data__->, INPRESSURE, , false);
            releaseAxisOwnership((int)axisIndex);
        }
        else if (fb->ACTIVE)
        {
            __SET_VAR(data__->, BUSY, , true);
            __SET_VAR(data__->, ACTIVE, , true);

            /* INPRESSURE检测: 实际压力接近目标压力 */
            HDY_REAL pressError = fb->AXIS_REF.pressure - targetPressure;
            if (pressError < 0.0f) pressError = -pressError;  /* fabs */
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
            releaseAxisOwnership((int)axisIndex);
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
