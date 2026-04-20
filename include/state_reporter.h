#ifndef HDY_STATE_REPORTER_H
#define HDY_STATE_REPORTER_H

#include "motion_control.h"
#include "motion_planner.h"
#include "pump_converter.h"

void HDY_StateReporter_SetActive(HDY_MotionControlFB* fb, HDY_BOOL active);
void HDY_StateReporter_SetFinished(HDY_MotionControlFB* fb, HDY_BOOL finished);
void HDY_StateReporter_SetFault(HDY_MotionControlFB* fb, HDY_BOOL fault);
void HDY_StateReporter_SetStatus(HDY_MotionControlFB* fb, HDY_ControllerStatus status);
void HDY_StateReporter_SetFbState(HDY_MotionControlFB* fb, HDY_FbState state);
void HDY_StateReporter_SetProtectionAction(HDY_MotionControlFB* fb, HDY_ProtectionAction action);
void HDY_StateReporter_SetPlannedDirection(HDY_MotionControlFB* fb, HDY_MotionDirection direction);
void HDY_StateReporter_SetSegmentSource(HDY_MotionControlFB* fb, HDY_SegmentSource source);
void HDY_StateReporter_RefreshStandardOutputs(HDY_MotionControlFB* fb);
void HDY_StateReporter_ResetTransitionFlags(HDY_MotionControlFB* fb);
void HDY_StateReporter_ApplySafeOutputs(HDY_MotionControlFB* fb);
void HDY_StateReporter_ClearSegmentName(HDY_MotionControlFB* fb);
void HDY_StateReporter_SetSegmentName(HDY_MotionControlFB* fb, const char* name);
void HDY_StateReporter_SetIdleState(HDY_MotionControlFB* fb,
                                    HDY_BOOL finished,
                                    HDY_BOOL segmentCompleted);
void HDY_StateReporter_SetHoldState(HDY_MotionControlFB* fb);
void HDY_StateReporter_EnterFaultState(HDY_MotionControlFB* fb);
void HDY_StateReporter_ReportExecution(HDY_MotionControlFB* fb,
                                       const HDY_MotionPlannerOutput* plannerOutput,
                                       const HDY_PumpConverterOutput* pumpOutput,
                                       const HDY_ExecutionReference* references,
                                       HDY_PressureControllerType pressureControllerApplied,
                                       const HDY_PressureControllerOutput* pressureOutput,
                                       const HDY_DiagnosticInfo* diagnostic);

#endif /* HDY_STATE_REPORTER_H */
