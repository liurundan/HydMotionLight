#include "motion_interface.h"

HDY_MotionControlFB HDY_MotionControlFB_inst[HDY_MAX_AXIS_MOTION];

static unsigned int NextAllocatedMotionControlFB = 0;

int __MK_Alloc_MotionControlFB(){
	if(NextAllocatedMotionControlFB<HDY_MAX_AXIS_MOTION){
		HDY_MotionControlFB_inst[NextAllocatedMotionControlFB]._index = NextAllocatedMotionControlFB;
		return NextAllocatedMotionControlFB++;
	}else{
		return -1;
	}
}

HDY_MotionControlFB* __MK_GetPublic_MotionControlFB(int index){
	if(index < NextAllocatedMotionControlFB){
		return &HDY_MotionControlFB_inst[index];
	}
	return NULL;
}

void __mcl_cmd_hdyaxis_motion(HDY_AXISMOTION *data__)
{

}