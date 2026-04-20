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

/* High-level reporting helpers that compose diagnostics, update standard
 * outputs, and capture snapshots/history. `ReportFault` additionally
 * triggers a protected fault stop via the ProtectionManager.
 */
void HDY_StateReporter_ReportDiagnostic(HDY_MotionControlFB* fb,
                                        HDY_DiagnosticCode code,
                                        HDY_DiagnosticSeverity severity,
                                        const char* message,
                                        HDY_TIME eventTimestamp,
                                        const HDY_MotionSegment* segment,
                                        const HDY_ExecutionReference* references);

void HDY_StateReporter_ReportFault(HDY_MotionControlFB* fb,
                                   HDY_DiagnosticCode code,
                                   const char* message,
                                   HDY_TIME eventTimestamp,
                                   const HDY_MotionSegment* segment,
                                   const HDY_ExecutionReference* references);

/* Diagnostic helper APIs moved from motion_control to centralize recording
 * and retention logic in the StateReporter module. These API calls operate on
 * the FB instance and manage DIAGNOSTIC, DIAGNOSTIC_LATCH and history.
 */
void HDY_StateReporter_ClearCurrentDiagnostic(HDY_MotionControlFB* fb);
void HDY_StateReporter_ClearDiagnosticRetentionOnly(HDY_MotionControlFB* fb);
void HDY_StateReporter_ResetDiagnosticRetention(HDY_MotionControlFB* fb);
void HDY_StateReporter_ClearLiveDiagnosticInNonFaultHold(HDY_MotionControlFB* fb);
void HDY_StateReporter_RecordDiagnosticEvent(HDY_MotionControlFB* fb,
                                              HDY_TIME eventTimestamp,
                                              const HDY_MotionSegment* segment,
                                              const HDY_ExecutionReference* references);

#endif /* HDY_STATE_REPORTER_H */
