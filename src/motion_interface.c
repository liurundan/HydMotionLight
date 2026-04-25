#include "motion_interface.h"
#include <string.h>
#include <stdio.h>

HDY_MotionControlFB HDY_MotionControlFB_inst[HDY_MAX_AXIS_MOTION];

static unsigned int NextAllocatedMotionControlFB = 0;

static int __MK_Alloc_MotionControlFB()
{
	if(NextAllocatedMotionControlFB<HDY_MAX_AXIS_MOTION){
		HDY_MotionControlFB_inst[NextAllocatedMotionControlFB]._index = NextAllocatedMotionControlFB;
		return NextAllocatedMotionControlFB++;
	}else{
		return -1;
	}
}

HDY_MotionControlFB* __MK_GetPublic_MotionControlFB(int index)
{
	if(index < NextAllocatedMotionControlFB){
		return &HDY_MotionControlFB_inst[index];
	}
	return NULL;
}

void __mcl_cmd_axismotioncontrol(HDY_AXISMOTIONCONTROL *data__)
{
    IEC_BOOL bInit = __GET_VAR(data__->INIT);
    int index = -1;
    if (!bInit)
    {
        int index = __MK_Alloc_MotionControlFB();
        if (index >= 0)
        {
            HDY_MotionControlFB* fb = __MK_GetPublic_MotionControlFB(index);
            if (fb != NULL)
            {
                HDY_MotionControlFB_Init(fb);
                __GET_VAR(data__->INIT) = true;
                __SET_VAR(data__->,AXISINDEX,,index);

                fb->USE_RECIPE = false;  /* Direct mode: use DIRECT_SEGMENT */
                fb->FLOW_TO_PUMP_SPEED_GAIN = 1.2;  /* rpm per L/min */
                fb->PUMP_SPEED_LIMIT = 3000.0;       /* rpm */
            }
        }
    }
    else
    {
        // 已经初始化过了，可以在这里添加其他周期性需要执行的代码
        // 例如：HDY_MotionControlFB_SampleCommands(fb);
        index = __GET_VAR(data__->AXISINDEX);
        HDY_MotionControlFB* fb = __MK_GetPublic_MotionControlFB(index);
        if (fb != NULL)        
        {
            IEC_BOOL Execute = __GET_VAR(data__->EXECUTE);
            IEC_BOOL ExecUpdate = Execute && (!__GET_VAR(data__->EXECUTE0));
            fb->EN = Execute; /* EN follows EXECUTE input */
            if (ExecUpdate)
            {
                HDY_MotionSegment segment;
                HDY_TIME currentTime = 0.0;
                memset(&segment, 0, sizeof(segment));

                segment.segmentTag = HDY_SEGMENT_TYPE_INJECTION;
                segment.segmentType = HDY_SEGMENT_TYPE_INJECTION;
                segment.mode = HDY_MODE_SPEED_RAMP;
                segment.endCondition = HDY_END_POSITION;
                segment.direction = HDY_DIRECTION_EXTEND;
                segment.planner = HDY_PLANNER_TIME_BASED;

                segment.targetPosition = __GET_VAR(data__->MOTION).SETPOSITION;   /* mm */
                segment.targetFlow = __GET_VAR(data__->MOTION).SETFLOW;           /* L/min */
                segment.targetPressure = __GET_VAR(data__->MOTION).SETPRESSURE;   /* MPa */
                segment.maxVelocity = __GET_VAR(data__->MOTION).SETVELOCITY;      /* mm/s */
                segment.maxAcceleration = __GET_VAR(data__->MOTION).ACCELERATION; /* mm/s² */
                segment.maxFlow = __GET_VAR(data__->MOTION).SETFLOW;              /* L/min */

                segment.velocityToFlowGain = 0.2; /* L/min per mm/s */

                segment.pressureRampRate = 10.0; /* MPa/s */

                segment.positionTolerance = 0.1; /* mm  */
                segment.pressureTolerance = 0.5; /* MPa */
                segment.flowTolerance = 1.0;     /* L/min */
                segment.timeoutLimit = 5.0;      /* s */
                segment.pressureController = HDY_PRESSURE_CONTROLLER_P;
                segment.pressureKp = 0.5;       /* L/min per MPa */
                segment.pressureKi = 0.1;       /* L/min per (MPa*s) */
                segment.pressureKd = 0.05;      /* L/min per (MPa/s) */
                segment.pressureIntegralLimit = 10.0; /* L/min */
                segment.pressureDeadband = 0.2; /* MPa */
                segment.pressureFilterAlpha = 0.5; /* 0<alpha<=1    */
                segment.pressureDerivativeFilterAlpha = 0.5; /* 0<alpha<=1 */           
                 
                if (!HDY_MotionControlFB_LoadDirectSegment(fb, &segment))
                {
                    printf("ERROR: Failed to load holding segment\n");
                    return;
                }

                if (!HDY_MotionControlFB_StartSegment(fb, 0, currentTime))
                {
                    printf("ERROR: Failed to start holding segment\n");
                    return;
                }
            }
            
            /* 执行控制 */
            HDY_MotionControlFB_Execute(fb);

            __SET_VAR(data__->,ACTIVE,,fb->ACTIVE);
            __SET_VAR(data__->,BUSY,,fb->BUSY);
            __SET_VAR(data__->,DONE,,fb->DONE);
            __SET_VAR(data__->,ERROR,,fb->ERROR);
            __SET_VAR(data__->,ERRORID,,fb->ERROR_ID);
            __SET_VAR(data__->,STATE,,fb->STATE.status);
            __SET_VAR(data__->,PUMP_SPEED,,fb->PUMP_SPEED);
          
        }
 
    }
    
}
