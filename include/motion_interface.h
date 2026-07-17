#ifndef HYD_MOTION_INTERFACE_H
#define HYD_MOTION_INTERFACE_H

#include "motion_control.h"
#include "accessor.h"
#include "iec_types_all.h"
 
/********** 运动控制功能块指令 公共接口函数 **********/
/* 外部代码（例如PLC逻辑）可调用本系列函数，实现与运动控制功能块的交互。
   函数在指令处理流程中完成必要的合法性校验、状态更新与诊断信息上报。
   运动控制逻辑的实际运算执行在Cycle()/Scan()循环函数中实现，需在主控循环中周期性调用。 */
/* 指令处理约束规范：
1. 每个函数都会结合功能块当前运行状态对指令进行校验，返回布尔值，标识指令请求是否合法成功。
2. 校验通过时，指令将进入队列，等待下一次Cycle()/Scan()周期执行；同时立即完成所需的状态更新与诊断上报。
3. 校验失败时，函数返回false，并可写入诊断错误码，说明指令拒绝原因（如：状态非法、配方缺失、运动段未完成等）。
4. 本类函数不会直接操作激活(ACTIVE)、忙碌(BUSY)输出信号；
   该类输出由内部状态机在Cycle()/Scan()周期内，根据待执行指令与当前运行上下文统一管理。
5. 备注：本接口函数的具体实现位于 motion_control.c 文件；
   设计用于可访问 HYD_MotionControlFB 实例的外部代码调用（如通过API分层接口、嵌入式场景直接调用）。 */





/*
 * IEC pressure-unit contract:
 * All PLC-facing pressure values use bar and bar/s:
 *   HYD_AXISMOTION.SETPRESSURE / ACTPRESSURE / PRESSURERAMPRATE,
 *   HYD_MOVECONTINUOUSABSOLUTE.PRESSURELIMIT,
 *   HYD_PRESSUREHANDLE.PRESSURE / PRESSURERAMPRATE,
 *   HYD_SETAXISFEEDBACK.ACT_PRESSURE,
 *   HYD_READSIMFEEDBACK.PRESSURE.
 * The IEC adapter passes these values into HydroMotionLib without pressure-unit
 * conversion; the pressure closed-loop path consumes the same bar domain.
 */

/*
 * HYD_AXISMOTION ownership contract (Sprint 0 spec C-1):
 *
 * The structure is a bidirectional shared buffer between the PLC process
 * layer and the HydroMotionLib runtime. Field ownership is partitioned
 * into two halves; runtime and PLC each touch only their own half.
 *
 * Setpoint half -- PLC -> runtime, runtime MUST NOT write back:
 *   SEGMENTTAG, SEGMENTTYPE, PLANNER, MODE, ENDCONDITION, DIRECTION,
 *   SETPOSITION, SETVELOCITY, SETFLOW, SETPRESSURE, ACCELERATION,
 *   DECELERATION, DURATION, PRESSURERAMPRATE.
 *
 * The runtime reads these fields once on the EXECUTE rising edge of a
 * MoveProfile FB (via buildSegmentFromMotion) to build the active segment
 * descriptor. After that, the segment lives in fb->_activeSegment and the
 * runtime does not look at MOTION setpoint fields again until the next
 * rising edge. This guarantees that staging the next segment's setpoint
 * via MOTION between scans is safe even when several FB instances bind
 * to the same physical HYD_AXISMOTION.
 *
 * Actual half -- runtime -> PLC, PLC MUST NOT write back:
 *   ACTPOSITION, ACTVELOCITY, ACTFLOW, ACTPRESSURE, TIMESTAMP.
 *
 * In simulation deployments the runtime publishes ACT* + TIMESTAMP from
 * fb->AXIS_REF every cycle. In non-simulation deployments the PLC writes
 * ACT* + TIMESTAMP from sensor I/O before invoking any FB, and the
 * runtime treats them as input.
 *
 * Multi-FB safety: PLC programs are responsible for ensuring at most
 * one FB writes Setpoint values per scan when several FB instances are
 * pointed at the same axis. The runtime guarantees that its own pass
 * will not contribute to data races on Setpoint fields.
 *
 * See docs/integration/plc-process-layer-integration-guide.md
 * "HYD_AXISMOTION Half-Region Ownership" for the full contract.
 */
__DECLARE_STRUCT_TYPE(HYD_AXISMOTION,
    USINT SEGMENTTAG;
    USINT SEGMENTTYPE;
    USINT PLANNER;
    USINT MODE;
    USINT ENDCONDITION;
    USINT DIRECTION;
    REAL SETPOSITION;
    REAL SETVELOCITY;
    REAL SETFLOW;
    REAL SETPRESSURE;
    REAL ACCELERATION;
    REAL DECELERATION;
    REAL DURATION;
    REAL PRESSURERAMPRATE;
    REAL ACTPOSITION;
    REAL ACTVELOCITY;
    REAL ACTFLOW;
    REAL ACTPRESSURE;
    REAL TIMESTAMP;
  )

// FUNCTION_BLOCK HYD_MOVEPROFILE
// Recipe模式：配方驱动的多段运动控制
// MOTION字段为双向通道，但两半区单向语义 (见 HYD_AXISMOTION 上方契约):
//   Setpoint半区 (PLC -> runtime, runtime 不回写):
//     SET*, SEGMENT*, MODE, ENDCONDITION, DIRECTION, PLANNER,
//     ACCELERATION, DECELERATION, DURATION, PRESSURERAMPRATE
//     runtime 仅在 EXECUTE 上升沿读取一次以构建当前段。
//   Actual半区 (runtime/sensor -> PLC, PLC 不回写): ACT*, TIMESTAMP
// 配方来源:
//   1. 从MOTION字段构建1段配方(EXECUTE时自动加载)
//   2. 外部通过HDY_MotionControlFB_LoadRecipe()预加载多段配方
// Data part
typedef struct {
  // FB Interface - IN, OUT, IN_OUT variables
  __DECLARE_VAR(BOOL,EN)
  __DECLARE_VAR(BOOL,ENO)
  __DECLARE_VAR(SINT,AXISID)
  __DECLARE_VAR(BOOL,EXECUTE)
  __DECLARE_VAR(HYD_AXISMOTION,MOTION)
  __DECLARE_VAR(INT,BUFFERMODE)
  __DECLARE_VAR(BOOL,DONE)
  __DECLARE_VAR(BOOL,BUSY)
  __DECLARE_VAR(BOOL,ACTIVE)
  __DECLARE_VAR(BOOL,COMMANDABORTED)
  __DECLARE_VAR(BOOL,ERROR)
  __DECLARE_VAR(WORD,ERRORID)

  // FB private variables - TEMP, private and located variables
  __DECLARE_VAR(BOOL,EXECUTE0)
  __DECLARE_VAR(BOOL,DONE0)
  __DECLARE_VAR(BOOL,ACTIVE0)
  __DECLARE_VAR(BOOL,_PENDING)
  __DECLARE_VAR(WORD,_EXEC_ID)

} HYD_MOVEPROFILE;

// FUNCTION_BLOCK HYD_LOADPROFILE
// Data part
typedef struct {
  // FB Interface - IN, OUT, IN_OUT variables
  __DECLARE_VAR(BOOL,EN)
  __DECLARE_VAR(BOOL,ENO)
  __DECLARE_VAR(SINT,AXISID)
  __DECLARE_VAR(BOOL,EXECUTE)
  __DECLARE_VAR(HYD_AXISMOTION,MOTION)
  __DECLARE_VAR(BOOL,DONE)
  __DECLARE_VAR(BOOL,BUSY)
  __DECLARE_VAR(BOOL,ERROR)
  __DECLARE_VAR(WORD,ERRORID)

  // FB private variables - TEMP, private and located variables
  __DECLARE_VAR(BOOL,EXECUTE0)
  __DECLARE_VAR(BOOL,DONE0)

} HYD_LOADPROFILE;

// FUNCTION_BLOCK HYD_STOP
// Data part
typedef struct {
  // FB Interface - IN, OUT, IN_OUT variables
  __DECLARE_VAR(BOOL,EN)
  __DECLARE_VAR(BOOL,ENO)
  __DECLARE_VAR(SINT,AXISID)
  __DECLARE_VAR(BOOL,EXECUTE)
  __DECLARE_VAR(REAL,DECELERATION)
  __DECLARE_VAR(BOOL,DONE)
  __DECLARE_VAR(BOOL,BUSY)
  __DECLARE_VAR(BOOL,COMMANDABORTED)
  __DECLARE_VAR(BOOL,ERROR)
  __DECLARE_VAR(WORD,ERRORID)

  // FB private variables - TEMP, private and located variables
  __DECLARE_VAR(BOOL,EXECUTE0)
  __DECLARE_VAR(BOOL,DONE0)
  __DECLARE_VAR(BOOL,ACTIVE0)
  __DECLARE_VAR(BOOL,_PENDING)

} HYD_STOP;

// FUNCTION_BLOCK HYD_Hold
// Data part
typedef struct {
  // FB Interface - IN, OUT, IN_OUT variables
  __DECLARE_VAR(BOOL,EN)
  __DECLARE_VAR(BOOL,ENO)
  __DECLARE_VAR(SINT,AXISID)
  __DECLARE_VAR(BOOL,EXECUTE)
  __DECLARE_VAR(BOOL,DONE)
  __DECLARE_VAR(BOOL,BUSY)
  __DECLARE_VAR(BOOL,ERROR)
  __DECLARE_VAR(WORD,ERRORID)

  // FB private variables - TEMP, private and located variables
  __DECLARE_VAR(BOOL,EXECUTE0)
  __DECLARE_VAR(BOOL,DONE0)
  __DECLARE_VAR(BOOL,_PENDING)

} HYD_HOLD;

// FUNCTION_BLOCK HYD_Resume
// Data part
typedef struct {
  // FB Interface - IN, OUT, IN_OUT variables
  __DECLARE_VAR(BOOL,EN)
  __DECLARE_VAR(BOOL,ENO)
  __DECLARE_VAR(SINT,AXISID)
  __DECLARE_VAR(BOOL,EXECUTE)
  __DECLARE_VAR(BOOL,DONE)
  __DECLARE_VAR(BOOL,BUSY)
  __DECLARE_VAR(BOOL,ERROR)
  __DECLARE_VAR(WORD,ERRORID)

  // FB private variables - TEMP, private and located variables
  __DECLARE_VAR(BOOL,EXECUTE0)
  __DECLARE_VAR(BOOL,DONE0)
  __DECLARE_VAR(BOOL,_PENDING)

} HYD_RESUME;

// FUNCTION_BLOCK HYD_MoveAbsolute
// Note: JERK and CONTINUOUSUPDATE are currently reserved compatibility pins.
// Non-default values are rejected by the IEC adapter until runtime support exists.
// Data part
typedef struct {
  // FB Interface - IN, OUT, IN_OUT variables
  __DECLARE_VAR(BOOL,EN)
  __DECLARE_VAR(BOOL,ENO)
  __DECLARE_VAR(SINT,AXISID)
  __DECLARE_VAR(BOOL,EXECUTE)
  __DECLARE_VAR(BOOL,CONTINUOUSUPDATE)
  __DECLARE_VAR(REAL,POSITION)
  __DECLARE_VAR(REAL,VELOCITY)
  __DECLARE_VAR(REAL,ACCELERATION)
  __DECLARE_VAR(REAL,DECELERATION)
  __DECLARE_VAR(REAL,JERK)
  __DECLARE_VAR(SINT,DIRECTION)
  __DECLARE_VAR(INT,BUFFERMODE)
  __DECLARE_VAR(BOOL,DONE)
  __DECLARE_VAR(BOOL,BUSY)
  __DECLARE_VAR(BOOL,ACTIVE)
  __DECLARE_VAR(BOOL,COMMANDABORTED)
  __DECLARE_VAR(BOOL,ERROR)
  __DECLARE_VAR(WORD,ERRORID)

  // FB private variables - TEMP, private and located variables
  __DECLARE_VAR(BOOL,EXECUTE0)
  __DECLARE_VAR(BOOL,DONE0)
  __DECLARE_VAR(BOOL,ACTIVE0)
  __DECLARE_VAR(BOOL,_PENDING)
  __DECLARE_VAR(WORD,_EXEC_ID)

} HYD_MOVEABSOLUTE;

// FUNCTION_BLOCK HYD_MoveContinuousAbsolute
typedef struct {
  __DECLARE_VAR(BOOL,EN)
  __DECLARE_VAR(BOOL,ENO)
  __DECLARE_VAR(SINT,AXISID)
  __DECLARE_VAR(BOOL,EXECUTE)
  __DECLARE_VAR(REAL,POSITION)
  __DECLARE_VAR(REAL,VELOCITY)
  __DECLARE_VAR(REAL,ENDVELOCITY)
  __DECLARE_VAR(SINT,ENDVELOCITYDIRECTION)
  __DECLARE_VAR(REAL,ACCELERATION)
  __DECLARE_VAR(REAL,DECELERATION)
  __DECLARE_VAR(REAL,JERK)
  __DECLARE_VAR(SINT,DIRECTION)
  __DECLARE_VAR(BOOL,ADAPTENDVELTOAVOIDOVERSHOOT)
  __DECLARE_VAR(REAL,PRESSURELIMIT)
  __DECLARE_VAR(BOOL,INENDVELOCITY)
  __DECLARE_VAR(BOOL,POSITIONREACHED)
  __DECLARE_VAR(BOOL,BUSY)
  __DECLARE_VAR(BOOL,COMMANDABORTED)
  __DECLARE_VAR(BOOL,ERROR)
  __DECLARE_VAR(WORD,ERRORID)

  __DECLARE_VAR(BOOL,EXECUTE0)
  __DECLARE_VAR(BOOL,INENDVELOCITY0)
  __DECLARE_VAR(BOOL,POSITIONREACHED0)
  __DECLARE_VAR(BOOL,_PENDING)
  __DECLARE_VAR(WORD,_EXEC_ID)
} HYD_MOVECONTINUOUSABSOLUTE;


// FUNCTION_BLOCK HYD_MoveVelocity
// Note: JERK and CONTINUOUSUPDATE are currently reserved compatibility pins.
// Non-default values are rejected by the IEC adapter until runtime support exists.
// Data part
typedef struct {
  // FB Interface - IN, OUT, IN_OUT variables
  __DECLARE_VAR(BOOL,EN)
  __DECLARE_VAR(BOOL,ENO)
  __DECLARE_VAR(SINT,AXISID)
  __DECLARE_VAR(BOOL,EXECUTE)
  __DECLARE_VAR(BOOL,CONTINUOUSUPDATE)
  __DECLARE_VAR(REAL,VELOCITY)
  __DECLARE_VAR(REAL,ACCELERATION)
  __DECLARE_VAR(REAL,DECELERATION)
  __DECLARE_VAR(REAL,JERK)
  __DECLARE_VAR(SINT,DIRECTION)
  __DECLARE_VAR(INT,BUFFERMODE)
  __DECLARE_VAR(BOOL,INVELOCITY)
  __DECLARE_VAR(BOOL,BUSY)
  __DECLARE_VAR(BOOL,ACTIVE)
  __DECLARE_VAR(BOOL,COMMANDABORTED)
  __DECLARE_VAR(BOOL,ERROR)
  __DECLARE_VAR(WORD,ERRORID)

  // FB private variables - TEMP, private and located variables
  __DECLARE_VAR(BOOL,EXECUTE0)
  __DECLARE_VAR(BOOL,INVELOCITY0)
  __DECLARE_VAR(BOOL,ACTIVE0)
  __DECLARE_VAR(BOOL,_PENDING)
  __DECLARE_VAR(WORD,_EXEC_ID)

} HYD_MOVEVELOCITY;

// FUNCTION_BLOCK HYD_Reset
// Data part
typedef struct {
  // FB Interface - IN, OUT, IN_OUT variables
  __DECLARE_VAR(BOOL,EN)
  __DECLARE_VAR(BOOL,ENO)
  __DECLARE_VAR(SINT,AXISID)
  __DECLARE_VAR(BOOL,EXECUTE)
  __DECLARE_VAR(BOOL,DONE)
  __DECLARE_VAR(BOOL,BUSY)
  __DECLARE_VAR(BOOL,ERROR)
  __DECLARE_VAR(WORD,ERRORID)

  // FB private variables - TEMP, private and located variables
  __DECLARE_VAR(BOOL,EXECUTE0)
  __DECLARE_VAR(BOOL,DONE0)

} HYD_RESET;

// FUNCTION_BLOCK HYD_PressureHandle
// Data part
typedef struct {
  // FB Interface - IN, OUT, IN_OUT variables
  __DECLARE_VAR(BOOL,EN)
  __DECLARE_VAR(BOOL,ENO)
  __DECLARE_VAR(SINT,AXISID)
  __DECLARE_VAR(BOOL,EXECUTE)
  __DECLARE_VAR(BOOL,CONTINUOUSUPDATE)
  __DECLARE_VAR(REAL,PRESSURE)
  __DECLARE_VAR(REAL,PRESSURERAMPRATE)
  __DECLARE_VAR(REAL,DURATION)
  __DECLARE_VAR(INT,BUFFERMODE)
  __DECLARE_VAR(BOOL,INPRESSURE)
  __DECLARE_VAR(BOOL,DONE)
  __DECLARE_VAR(BOOL,BUSY)
  __DECLARE_VAR(BOOL,ACTIVE)
  __DECLARE_VAR(BOOL,COMMANDABORTED)
  __DECLARE_VAR(BOOL,ERROR)
  __DECLARE_VAR(WORD,ERRORID)

  // FB private variables - TEMP, private and located variables
  __DECLARE_VAR(BOOL,EXECUTE0)
  __DECLARE_VAR(BOOL,INPRESSURE0)
  __DECLARE_VAR(BOOL,ACTIVE0)
  __DECLARE_VAR(BOOL,_PENDING)
  __DECLARE_VAR(WORD,_EXEC_ID)

} HYD_PRESSUREHANDLE;

// FUNCTION_BLOCK HYD_CREATEMOTION
// Data part
typedef struct {
  // FB Interface - IN, OUT, IN_OUT variables
  __DECLARE_VAR(BOOL,EN)
  __DECLARE_VAR(BOOL,ENO)
  __DECLARE_VAR(BOOL,USE_RECIPE)
  __DECLARE_VAR(REAL,FLOW_TO_PUMPSPEED)
  __DECLARE_VAR(REAL,PUMPSPEED_LIMIT)
  __DECLARE_VAR(BOOL,USE_SIMULATION)
  __DECLARE_VAR(SINT,AXISID)
  __DECLARE_VAR(BOOL,DONE)
  __DECLARE_VAR(BOOL,BUSY)
  __DECLARE_VAR(BOOL,ERROR)
  __DECLARE_VAR(WORD,ERRORID)

  // FB private variables - TEMP, private and located variables
  __DECLARE_VAR(BOOL,DONE0)

} HYD_CREATEMOTION;

// FUNCTION_BLOCK HYD_SETAXISFEEDBACK
// Data part
typedef struct {
  // FB Interface - IN, OUT, IN_OUT variables
  __DECLARE_VAR(BOOL,EN)
  __DECLARE_VAR(BOOL,ENO)
  __DECLARE_VAR(SINT,AXISID)
  __DECLARE_VAR(BOOL,ENABLE)
  __DECLARE_VAR(REAL,ACT_POSITION)
  __DECLARE_VAR(REAL,ACT_VELOCITY)
  __DECLARE_VAR(REAL,ACT_FLOW)
  __DECLARE_VAR(REAL,ACT_PRESSURE)
  __DECLARE_VAR(REAL,TIMESTAMP)
  __DECLARE_VAR(BOOL,DONE)
  __DECLARE_VAR(BOOL,BUSY)
  __DECLARE_VAR(BOOL,ERROR)
  __DECLARE_VAR(WORD,ERRORID)

  // FB private variables - TEMP, private and located variables
  __DECLARE_VAR(BOOL,DONE0)

} HYD_SETAXISFEEDBACK;

// FUNCTION_BLOCK HYD_GETPUMPREQUEST
// Data part
typedef struct {
  // FB Interface - IN, OUT, IN_OUT variables
  __DECLARE_VAR(BOOL,EN)
  __DECLARE_VAR(BOOL,ENO)
  __DECLARE_VAR(BOOL,ENABLE)
  __DECLARE_VAR(SINT,STRATEGY)
  __DECLARE_VAR(BOOL,ALLOW_NEGATIVE)   /* 允许负转速输出（反转卸压），默认 FALSE */
  __DECLARE_VAR(REAL,PUMPSPEED)
  __DECLARE_VAR(BOOL,CONFLICT)
  __DECLARE_VAR(BOOL,BUSY)
  __DECLARE_VAR(BOOL,DONE)
  __DECLARE_VAR(BOOL,ERROR)
  __DECLARE_VAR(WORD,ERRORID)

  // FB private variables - TEMP, private and located variables
  __DECLARE_VAR(BOOL,DONE0)
  __DECLARE_VAR(BOOL,ALLOW_NEGATIVE0)  /* 内部缓存，用于边沿检测（如需） */

} HYD_GETPUMPREQUEST;

// FUNCTION_BLOCK HYD_READSTATUS
// Data part
typedef struct {
  // FB Interface - IN, OUT, IN_OUT variables
  __DECLARE_VAR(BOOL,EN)
  __DECLARE_VAR(BOOL,ENO)
  __DECLARE_VAR(SINT,AXISID)
  __DECLARE_VAR(BOOL,ENABLE)
  __DECLARE_VAR(BOOL,VALID)
  __DECLARE_VAR(BOOL,BUSY)
  __DECLARE_VAR(BOOL,ERROR)
  __DECLARE_VAR(WORD,ERRORID)
  __DECLARE_VAR(UINT,STATE)

  // FB private variables - TEMP, private and located variables

} HYD_READSTATUS;

// FUNCTION_BLOCK HYD_READERROR
// Data part
typedef struct {
  // FB Interface - IN, OUT, IN_OUT variables
  __DECLARE_VAR(BOOL,EN)
  __DECLARE_VAR(BOOL,ENO)
  __DECLARE_VAR(SINT,AXISID)
  __DECLARE_VAR(BOOL,ENABLE)
  __DECLARE_VAR(BOOL,VALID)
  __DECLARE_VAR(BOOL,BUSY)
  __DECLARE_VAR(BOOL,ERROR)
  __DECLARE_VAR(WORD,ERRORID)

  // FB private variables - TEMP, private and located variables

} HYD_READERROR;

// FUNCTION_BLOCK HYD_READSIMFEEDBACK
// Data part
typedef struct {
  // FB Interface - IN, OUT, IN_OUT variables
  __DECLARE_VAR(BOOL,EN)
  __DECLARE_VAR(BOOL,ENO)
  __DECLARE_VAR(SINT,AXISID)
  __DECLARE_VAR(BOOL,ENABLE)
  __DECLARE_VAR(BOOL,VALID)
  __DECLARE_VAR(BOOL,BUSY)
  __DECLARE_VAR(BOOL,ERROR)
  __DECLARE_VAR(WORD,ERRORID)
  __DECLARE_VAR(REAL,POSITION)
  __DECLARE_VAR(REAL,VELOCITY)
  __DECLARE_VAR(REAL,FLOW)
  __DECLARE_VAR(REAL,PRESSURE)

  // FB private variables - TEMP, private and located variables

} HYD_READSIMFEEDBACK;

// FUNCTION_BLOCK HYD_READPARAMETER
// Data part
typedef struct {
  // FB Interface - IN, OUT, IN_OUT variables
  __DECLARE_VAR(BOOL,EN)
  __DECLARE_VAR(BOOL,ENO)
  __DECLARE_VAR(SINT,AXISID)
  __DECLARE_VAR(BOOL,ENABLE)
  __DECLARE_VAR(INT,PARAMETERNUMBER)
  __DECLARE_VAR(BOOL,VALID)
  __DECLARE_VAR(BOOL,BUSY)
  __DECLARE_VAR(BOOL,ERROR)
  __DECLARE_VAR(WORD,ERRORID)
  __DECLARE_VAR(LREAL,VALUE)

  // FB private variables - TEMP, private and located variables
  __DECLARE_VAR(BOOL,ENABLE0)

} HYD_READPARAMETER;

// FUNCTION_BLOCK HYD_WRITEPARAMETER
// Data part
typedef struct {
  // FB Interface - IN, OUT, IN_OUT variables
  __DECLARE_VAR(BOOL,EN)
  __DECLARE_VAR(BOOL,ENO)
  __DECLARE_VAR(SINT,AXISID)
  __DECLARE_VAR(BOOL,EXECUTE)
  __DECLARE_VAR(INT,PARAMETERNUMBER)
  __DECLARE_VAR(LREAL,VALUE)
  __DECLARE_VAR(BOOL,DONE)
  __DECLARE_VAR(BOOL,BUSY)
  __DECLARE_VAR(BOOL,ERROR)
  __DECLARE_VAR(WORD,ERRORID)

  // FB private variables - TEMP, private and located variables
  __DECLARE_VAR(BOOL,EXECUTE0)
  __DECLARE_VAR(BOOL,DONE0)

} HYD_WRITEPARAMETER;

// FUNCTION_BLOCK HYD_READBOOLPARAMETER
// Data part
typedef struct {
  // FB Interface - IN, OUT, IN_OUT variables
  __DECLARE_VAR(BOOL,EN)
  __DECLARE_VAR(BOOL,ENO)
  __DECLARE_VAR(SINT,AXISID)
  __DECLARE_VAR(BOOL,ENABLE)
  __DECLARE_VAR(INT,PARAMETERNUMBER)
  __DECLARE_VAR(BOOL,VALID)
  __DECLARE_VAR(BOOL,BUSY)
  __DECLARE_VAR(BOOL,ERROR)
  __DECLARE_VAR(WORD,ERRORID)
  __DECLARE_VAR(BOOL,VALUE)

  // FB private variables - TEMP, private and located variables
  __DECLARE_VAR(BOOL,ENABLE0)

} HYD_READBOOLPARAMETER;

// FUNCTION_BLOCK HYD_WRITEBOOLPARAMETER
// Data part
typedef struct {
  // FB Interface - IN, OUT, IN_OUT variables
  __DECLARE_VAR(BOOL,EN)
  __DECLARE_VAR(BOOL,ENO)
  __DECLARE_VAR(SINT,AXISID)
  __DECLARE_VAR(BOOL,EXECUTE)
  __DECLARE_VAR(INT,PARAMETERNUMBER)
  __DECLARE_VAR(BOOL,VALUE)
  __DECLARE_VAR(BOOL,DONE)
  __DECLARE_VAR(BOOL,BUSY)
  __DECLARE_VAR(BOOL,ERROR)
  __DECLARE_VAR(WORD,ERRORID)

  // FB private variables - TEMP, private and located variables
  __DECLARE_VAR(BOOL,EXECUTE0)
  __DECLARE_VAR(BOOL,DONE0)

} HYD_WRITEBOOLPARAMETER;

extern int  __HydMotion_framework_Init();
extern void __HydMotion_framework_Cleanup();
extern void __HydMotion_framework_Retrieve();
extern void __HydMotion_framework_Publish();
extern char* __HydMotion_framework_GetVersion();

extern void __mcl_cmd_CreateMotion(HYD_CREATEMOTION *data__);
extern void __mcl_cmd_LoadProfile(HYD_LOADPROFILE *data__);
extern void __mcl_cmd_MoveProfile(HYD_MOVEPROFILE *data__);
extern void __mcl_cmd_Stop(HYD_STOP *data__);
extern void __mcl_cmd_Hold(HYD_HOLD *data__);
extern void __mcl_cmd_Resume(HYD_RESUME *data__);
extern void __mcl_cmd_MoveAbsolute(HYD_MOVEABSOLUTE *data__);
extern void __mcl_cmd_MoveContinuousAbsolute(HYD_MOVECONTINUOUSABSOLUTE *data__);
extern void __mcl_cmd_Reset(HYD_RESET *data__);
extern void __mcl_cmd_MoveVelocity(HYD_MOVEVELOCITY *data__);
extern void __mcl_cmd_PressureHandle(HYD_PRESSUREHANDLE *data__);
extern void __mcl_cmd_SetAxisFeedback(HYD_SETAXISFEEDBACK *data__);
extern void __mcl_cmd_GetPumpRequest(HYD_GETPUMPREQUEST *data__);

extern void __mcl_cmd_ReadStatus(HYD_READSTATUS* data__);
extern void __mcl_cmd_ReadError(HYD_READERROR* data__);
extern void __mcl_cmd_ReadSimFeedback(HYD_READSIMFEEDBACK* data__);

extern void __mcl_cmd_ReadParameter(HYD_READPARAMETER* data__);
extern void __mcl_cmd_WriteParameter(HYD_WRITEPARAMETER* data__);
extern void __mcl_cmd_ReadBoolParameter(HYD_READBOOLPARAMETER* data__);
extern void __mcl_cmd_WriteBoolParameter(HYD_WRITEBOOLPARAMETER* data__);

#endif
