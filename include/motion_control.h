#ifndef HDY_MOTION_CONTROL_H
#define HDY_MOTION_CONTROL_H

#include "common_types.h"
#include "ramp_controller.h"


typedef struct {
    HDY_BOOL EN;
    HDY_BOOL RESET;
    HDY_BOOL START_SEGMENT;
    HDY_UINT START_SEGMENT_INDEX;
    HDY_REAL FLOW_TO_PUMP_SPEED_GAIN;
    HDY_REAL PUMP_SPEED_LIMIT;
    HDY_UINT RECIPE_SIZE;
    HDY_AxisRef AXIS_REF;
    HDY_MotionSegment RECIPE[HDY_MAX_SEGMENTS];

    HDY_BOOL ENO;
    HDY_BOOL ACTIVE;
    HDY_BOOL FINISHED;
    HDY_REAL PUMP_SPEED;
    HDY_BOOL SEGMENT_COMPLETED;
    HDY_BOOL SEGMENT_CHANGED;
    HDY_MotionState STATE;
    HDY_DiagnosticInfo DIAGNOSTIC;
    char CURRENT_SEGMENT_NAME[HDY_NAME_MAX];

    /* Internal */
    HDY_REAL _segmentStartTime;
    HDY_BOOL _segmentChangedFlag;
    HDY_RampController _rampController;
} HDY_MotionControlFB;

void HDY_MotionControlFB_Init(HDY_MotionControlFB* fb);
void HDY_MotionControlFB_LoadRecipe(HDY_MotionControlFB* fb, const HDY_MotionSegment* recipe, size_t recipeSize);
void HDY_MotionControlFB_StartSegment(HDY_MotionControlFB* fb, size_t segmentIndex, HDY_TIME timestamp);
void HDY_MotionControlFB_NextSegment(HDY_MotionControlFB* fb, HDY_TIME timestamp);
void HDY_MotionControlFB_Abort(HDY_MotionControlFB* fb);
void HDY_MotionControlFB_Execute(HDY_MotionControlFB* fb);

#endif /* HDY_MOTION_CONTROL_H */
