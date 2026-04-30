#ifndef HYD_STATE_REPORTER_H
#define HYD_STATE_REPORTER_H

#include "motion_control.h"
#include "motion_planner.h"
#include "pump_converter.h"

void HYD_StateReporter_SetActive(HYD_MotionControlFB* fb, HYD_BOOL active);
void HYD_StateReporter_SetFinished(HYD_MotionControlFB* fb, HYD_BOOL finished);
void HYD_StateReporter_SetFault(HYD_MotionControlFB* fb, HYD_BOOL fault);
void HYD_StateReporter_SetStatus(HYD_MotionControlFB* fb, HYD_ControllerStatus status);
void HYD_StateReporter_SetFbState(HYD_MotionControlFB* fb, HYD_FbState state);
void HYD_StateReporter_SetProtectionAction(HYD_MotionControlFB* fb, HYD_ProtectionAction action);
void HYD_StateReporter_SetPlannedDirection(HYD_MotionControlFB* fb, HYD_MotionDirection direction);
void HYD_StateReporter_SetSegmentSource(HYD_MotionControlFB* fb, HYD_SegmentSource source);
void HYD_StateReporter_RefreshStandardOutputs(HYD_MotionControlFB* fb);
void HYD_StateReporter_ResetTransitionFlags(HYD_MotionControlFB* fb);
void HYD_StateReporter_ApplySafeOutputs(HYD_MotionControlFB* fb);
void HYD_StateReporter_ClearSegmentTag(HYD_MotionControlFB* fb);
void HYD_StateReporter_SetSegmentTag(HYD_MotionControlFB* fb, HYD_UINT8 tag);
void HYD_StateReporter_SetIdleState(HYD_MotionControlFB* fb,
                                    HYD_BOOL finished,
                                    HYD_BOOL segmentCompleted);
void HYD_StateReporter_SetHoldState(HYD_MotionControlFB* fb);
void HYD_StateReporter_EnterFaultState(HYD_MotionControlFB* fb);
void HYD_StateReporter_ReportExecution(HYD_MotionControlFB* fb,
                                       const HYD_MotionPlannerOutput* plannerOutput,
                                       const HYD_PumpConverterOutput* pumpOutput,
                                       const HYD_ExecutionReference* references,
                                       HYD_PressureControllerType pressureControllerApplied,
                                       const HYD_PressureControllerOutput* pressureOutput,
                                       const HYD_DiagnosticInfo* diagnostic);

/* High-level reporting helpers that compose diagnostics, synchronize live
 * protection action/state outputs, and capture snapshots/history.
 * `ReportFault` additionally triggers a protected fault stop via the
 * ProtectionManager.
 */
void HYD_StateReporter_ReportDiagnostic(HYD_MotionControlFB* fb,
                                        HYD_DiagnosticCode code,
                                        HYD_DiagnosticSeverity severity,
                                        HYD_TIME eventTimestamp,
                                        const HYD_MotionSegment* segment,
                                        const HYD_ExecutionReference* references);

void HYD_StateReporter_ReportFault(HYD_MotionControlFB* fb,
                                   HYD_DiagnosticCode code,
                                   HYD_TIME eventTimestamp,
                                   const HYD_MotionSegment* segment,
                                   const HYD_ExecutionReference* references);

/* Diagnostic helper APIs moved from motion_control to centralize recording
 * and retention logic in the StateReporter module. These API calls operate on
 * the FB instance and manage DIAGNOSTIC and history.
 */
void HYD_StateReporter_ClearCurrentDiagnostic(HYD_MotionControlFB* fb);
void HYD_StateReporter_ClearDiagnosticRetentionOnly(HYD_MotionControlFB* fb);
void HYD_StateReporter_ResetDiagnosticRetention(HYD_MotionControlFB* fb);
void HYD_StateReporter_ClearLiveDiagnosticInNonFaultHold(HYD_MotionControlFB* fb);
void HYD_StateReporter_RecordDiagnosticEvent(HYD_MotionControlFB* fb,
                                              HYD_TIME eventTimestamp,
                                              const HYD_MotionSegment* segment,
                                              const HYD_ExecutionReference* references);

#endif /* HYD_STATE_REPORTER_H */
