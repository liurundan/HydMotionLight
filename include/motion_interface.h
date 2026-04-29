#ifndef HDY_MOTION_INTERFACE_H
#define HDY_MOTION_INTERFACE_H

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
   设计用于可访问 HDY_MotionControlFB 实例的外部代码调用（如通过API分层接口、嵌入式场景直接调用）。 */





__DECLARE_STRUCT_TYPE(HDY_AXISMOTION,
    USINT SEGMENTTAG;
    USINT PLANNER;
    USINT MODE;
    USINT ENDCONDITION;
    USINT DIRECTION;
    REAL SETPOSITION;
    REAL SETVELOCITY; // 仅在速度/流量控制模式下使用，表示目标速度； 
    REAL SETFLOW;     // 仅在速度/流量控制模式下使用，表示目标流量； 
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

// FUNCTION_BLOCK HDY_MOVEPROFILE
// Recipe模式：配方驱动的多段运动控制
// MOTION字段为双向通道：
//   输入侧(ACT*): 轴反馈数据 → AXIS_REF
//   输出侧(SET*, SEGMENTTAG等): 当前活动段参数读回
// 配方来源:
//   1. 从MOTION字段构建1段配方(EXECUTE时自动加载)
//   2. 外部通过HDY_MotionControlFB_LoadRecipe()预加载多段配方
// Data part
typedef struct
{
   // FB Interface - IN, OUT, IN_OUT variables
   __DECLARE_VAR(BOOL, EN)
   __DECLARE_VAR(BOOL, ENO)
   __DECLARE_VAR(BOOL, EXECUTE)
   __DECLARE_VAR(HDY_AXISMOTION, MOTION)
   __DECLARE_VAR(SINT, AXISID)
   __DECLARE_VAR(BOOL, ACTIVE)
   __DECLARE_VAR(BOOL, BUSY)
   __DECLARE_VAR(BOOL, DONE)
   __DECLARE_VAR(BOOL, ERROR)
   __DECLARE_VAR(WORD, ERRORID)
   __DECLARE_VAR(WORD, STATE)
   __DECLARE_VAR(REAL, PUMP_SPEED)

   // FB private variables - TEMP, private and located variables
   __DECLARE_VAR(BOOL, INIT)

   __DECLARE_VAR(BOOL, EXECUTE0)
   __DECLARE_VAR(BOOL, DONE0)
   __DECLARE_VAR(BOOL, ACTIVE0)
   __DECLARE_VAR(WORD, GEN)
} HDY_MOVEPROFILE;

// FUNCTION_BLOCK HDY_LOADPROFILE
// Data part
typedef struct
{
   // FB Interface - IN, OUT, IN_OUT variables
   __DECLARE_VAR(BOOL, EN)
   __DECLARE_VAR(BOOL, ENO)
   __DECLARE_VAR(BOOL, EXECUTE)
   __DECLARE_VAR(HDY_AXISMOTION, MOTION)
   __DECLARE_VAR(SINT, AXISID)
   __DECLARE_VAR(BOOL, BUSY)
   __DECLARE_VAR(BOOL, DONE)
   __DECLARE_VAR(BOOL, ERROR)
   __DECLARE_VAR(WORD, ERRORID)

    // Note: LOADPROFILE does not have ACTIVE/DONE outputs, as it only loads the recipe without starting execution.
    // The presence of the loaded recipe can be inferred from the BUSY/DONE/ERROR outputs and diagnostic info.


   // FB private variables - TEMP, private and located variables
   __DECLARE_VAR(BOOL, INIT)

   __DECLARE_VAR(BOOL, EXECUTE0)
   __DECLARE_VAR(BOOL, DONE0)
   __DECLARE_VAR(WORD, GEN)
} HDY_LOADPROFILE;

// FUNCTION_BLOCK HDY_STOP
// Data part
typedef struct {
    __DECLARE_VAR(BOOL,EN)
    __DECLARE_VAR(BOOL,ENO)
    __DECLARE_VAR(BOOL,EXECUTE)
    __DECLARE_VAR(SINT,AXISID)

    __DECLARE_VAR(BOOL,DONE)
    __DECLARE_VAR(BOOL,BUSY)
    __DECLARE_VAR(BOOL,COMMANDABORTED)
    __DECLARE_VAR(BOOL,ERROR)
    __DECLARE_VAR(WORD,ERRORID)
    __DECLARE_VAR(BOOL,EXECUTE0)
    __DECLARE_VAR(BOOL,DONE0)
    __DECLARE_VAR(BOOL,ACTIVE0)
    __DECLARE_VAR(WORD,GEN)
} HDY_STOP;

// FUNCTION_BLOCK HDY_MoveAbsolute
// Data part
typedef struct {
    __DECLARE_VAR(BOOL,EN)
    __DECLARE_VAR(BOOL,ENO)
    __DECLARE_VAR(BOOL,EXECUTE)   
    __DECLARE_VAR(SINT,AXISID)
    __DECLARE_VAR(BOOL,CONTINUOUSUPDATE)
    __DECLARE_VAR(REAL,POSITION)
    __DECLARE_VAR(REAL,VELOCITY)
    __DECLARE_VAR(REAL,ACCELERATION)
    __DECLARE_VAR(REAL,DECELERATION)
    __DECLARE_VAR(REAL,JERK)
    __DECLARE_VAR(SINT,DIRECTION)
    __DECLARE_VAR(BOOL,DONE)
    __DECLARE_VAR(BOOL,BUSY)
    __DECLARE_VAR(BOOL,ACTIVE)
    __DECLARE_VAR(BOOL,COMMANDABORTED)
    __DECLARE_VAR(BOOL,ERROR)
    __DECLARE_VAR(WORD,ERRORID)
    __DECLARE_VAR(BOOL,EXECUTE0)
    __DECLARE_VAR(BOOL,DONE0)
    __DECLARE_VAR(BOOL,ACTIVE0)
    __DECLARE_VAR(WORD,GEN)
} HDY_MOVEABSOLUTE;

// FUNCTION_BLOCK HDY_MoveVelocity
// Data part
typedef struct {
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
    __DECLARE_VAR(BOOL,INVELOCITY)
    __DECLARE_VAR(BOOL,BUSY)
    __DECLARE_VAR(BOOL,ACTIVE)
    __DECLARE_VAR(BOOL,COMMANDABORTED)
    __DECLARE_VAR(BOOL,ERROR)
    __DECLARE_VAR(WORD,ERRORID)
    __DECLARE_VAR(BOOL,EXECUTE0)
    __DECLARE_VAR(BOOL,INVELOCITY0)
    __DECLARE_VAR(BOOL,ACTIVE0)
    __DECLARE_VAR(WORD,GEN)
} HDY_MOVEVELOCITY;

// FUNCTION_BLOCK HDY_Reset
// Data part
typedef struct {
    __DECLARE_VAR(BOOL,EN)
    __DECLARE_VAR(BOOL,ENO)
    __DECLARE_VAR(SINT,AXISID)
    __DECLARE_VAR(BOOL,EXECUTE)
    __DECLARE_VAR(BOOL,DONE)
    __DECLARE_VAR(BOOL,BUSY)
    __DECLARE_VAR(BOOL,ERROR)
    __DECLARE_VAR(WORD,ERRORID)
    __DECLARE_VAR(BOOL,EXECUTE0)
    __DECLARE_VAR(BOOL,DONE0)
    __DECLARE_VAR(WORD,GEN)
} HDY_RESET;

// FUNCTION_BLOCK HDY_PressureHandle
// Data part
typedef struct {
    __DECLARE_VAR(BOOL,EN)
    __DECLARE_VAR(BOOL,ENO)
    __DECLARE_VAR(SINT,AXISID)
    __DECLARE_VAR(BOOL,EXECUTE)

    __DECLARE_VAR(REAL,PRESSURE)
    __DECLARE_VAR(REAL,PRESSURERAMPRATE)
    __DECLARE_VAR(REAL,DURATION)
    __DECLARE_VAR(BOOL,INPRESSURE)
    __DECLARE_VAR(BOOL,BUSY)
    __DECLARE_VAR(BOOL,ACTIVE)
    __DECLARE_VAR(BOOL,COMMANDABORTED)
    __DECLARE_VAR(BOOL,ERROR)
    __DECLARE_VAR(WORD,ERRORID)
    __DECLARE_VAR(BOOL,EXECUTE0)
    __DECLARE_VAR(BOOL,INPRESSURE0)
    __DECLARE_VAR(BOOL,ACTIVE0)
    __DECLARE_VAR(WORD,GEN)
} HDY_PRESSUREHANDLE;

// FUNCTION_BLOCK HDY_CREATEMOTION
// Data part
typedef struct
{
   // FB Interface - IN, OUT, IN_OUT variables
   __DECLARE_VAR(BOOL, EN)
   __DECLARE_VAR(BOOL, ENO)
   __DECLARE_VAR(BOOL, USE_RECIPE)
   __DECLARE_VAR(REAL, FLOW_TO_PUMPSPEED)
   __DECLARE_VAR(REAL, PUMPSPEED_LIMIT)
   __DECLARE_VAR(BOOL, USE_SIMULATION)

   __DECLARE_VAR(SINT, AXISID)
   __DECLARE_VAR(BOOL, BUSY)
   __DECLARE_VAR(BOOL, DONE)
   __DECLARE_VAR(BOOL, ERROR)
   __DECLARE_VAR(WORD, ERRORID)

   // FB private variables - TEMP, private and located variables
   __DECLARE_VAR(BOOL, DONE0)

} HDY_CREATEMOTION;

// FUNCTION_BLOCK HDY_SETAXISFEEDBACK
// Data part
typedef struct
{
   // FB Interface - IN, OUT, IN_OUT variables
   __DECLARE_VAR(BOOL, EN)
   __DECLARE_VAR(BOOL, ENO)

   __DECLARE_VAR(BOOL, ENABLE)
   __DECLARE_VAR(SINT, AXISID)
   __DECLARE_VAR(REAL, ACT_POSITION)
   __DECLARE_VAR(REAL, ACT_VELOCITY)
   __DECLARE_VAR(REAL, ACT_FLOW)
   __DECLARE_VAR(REAL, ACT_PRESSURE)
   __DECLARE_VAR(REAL, TIMESTAMP)


   __DECLARE_VAR(BOOL, BUSY)
   __DECLARE_VAR(BOOL, DONE)
   __DECLARE_VAR(BOOL, ERROR)
   __DECLARE_VAR(WORD, ERRORID)

   // FB private variables - TEMP, private and located variables
   __DECLARE_VAR(BOOL, DONE0)

} HDY_SETAXISFEEDBACK;

// FUNCTION_BLOCK HDY_GETPUMPREQUEST
// Data part
typedef struct
{
   // FB Interface - IN, OUT, IN_OUT variables
   __DECLARE_VAR(BOOL, EN)
   __DECLARE_VAR(BOOL, ENO)
   __DECLARE_VAR(BOOL, ENABLE)
   __DECLARE_VAR(SINT, STRATEGY)

   __DECLARE_VAR(REAL, PUMPSPEED)
   __DECLARE_VAR(BOOL, BUSY)
   __DECLARE_VAR(BOOL, DONE)
   __DECLARE_VAR(BOOL, ERROR)
   __DECLARE_VAR(WORD, ERRORID)

   // FB private variables - TEMP, private and located variables
   __DECLARE_VAR(BOOL, DONE0)

} HDY_GETPUMPREQUEST;

extern int  __HdyMotion_framework_Init();
extern void __HdyMotion_framework_Cleanup();
extern void __HdyMotion_framework_Retrieve();
extern void __HdyMotion_framework_Publish();

extern void __mcl_cmd_CreateMotion(HDY_CREATEMOTION *data__);
extern void __mcl_cmd_LoadProfile(HDY_LOADPROFILE *data__);
extern void __mcl_cmd_MoveProfile(HDY_MOVEPROFILE *data__);
extern void __mcl_cmd_Stop(HDY_STOP *data__);
extern void __mcl_cmd_MoveAbsolute(HDY_MOVEABSOLUTE *data__);
extern void __mcl_cmd_Reset(HDY_RESET *data__);
extern void __mcl_cmd_MoveVelocity(HDY_MOVEVELOCITY *data__);
extern void __mcl_cmd_PressureHandle(HDY_PRESSUREHANDLE *data__);
extern void __mcl_cmd_SetAxisFeedback(HDY_SETAXISFEEDBACK *data__);
extern void __mcl_cmd_GetPumpRequest(HDY_GETPUMPREQUEST *data__);


#endif
