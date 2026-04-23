#ifndef HDY_DIAGNOSTICS_H
#define HDY_DIAGNOSTICS_H

#include "common_types.h"

typedef struct {
    const HDY_AxisRef* axisRef;
    const HDY_MotionSegment* segment;
    const HDY_ExecutionReference* references;
    HDY_BOOL enableVelocityDeviationCheck;  /* 默认false，设为true启用速度偏差诊断 */
} HDY_DiagnosticsContext;

void HDY_Diagnostics_Clear(HDY_DiagnosticInfo* diagnostic);
void HDY_Diagnostics_SetMessage(HDY_DiagnosticInfo* diagnostic, const char* message);
void HDY_Diagnostics_SetEvent(HDY_DiagnosticInfo* diagnostic,
                              HDY_DiagnosticCode code,
                              HDY_DiagnosticSeverity severity,
                              const char* message);

void HDY_Diagnostics_ClearSnapshot(HDY_DiagnosticSnapshot* snapshot);
void HDY_Diagnostics_CaptureSnapshot(HDY_DiagnosticSnapshot* snapshot,
                                     const HDY_DiagnosticInfo* diagnostic,
                                     const HDY_AxisRef* axisRef,
                                     const HDY_ExecutionReference* references,
                                     HDY_TIME eventTimestamp,
                                     HDY_UINT8 segmentIndex,
                                     const char* segmentName,
                                     HDY_ControllerStatus status,
                                     HDY_BOOL active,
                                     HDY_BOOL finished,
                                     HDY_BOOL fault);
void HDY_DiagnosticsHistory_Clear(HDY_DiagnosticHistory* history);
void HDY_DiagnosticsHistory_Push(HDY_DiagnosticHistory* history,
                                 const HDY_DiagnosticSnapshot* snapshot);
HDY_BOOL HDY_DiagnosticsHistory_GetEntry(const HDY_DiagnosticHistory* history,
                                         HDY_UINT8 chronologicalIndex,
                                         HDY_DiagnosticSnapshot* snapshot);
HDY_BOOL HDY_DiagnosticsHistory_GetLatest(const HDY_DiagnosticHistory* history,
                                          HDY_DiagnosticSnapshot* snapshot);
HDY_DiagnosticFlags HDY_Diagnostics_GetFlagMask(const HDY_DiagnosticInfo* diagnostic);
HDY_BOOL HDY_Diagnostics_HasFlag(const HDY_DiagnosticInfo* diagnostic,
                                 HDY_DiagnosticFlag flag);
const char* HDY_Diagnostics_CodeToString(HDY_DiagnosticCode code);
const char* HDY_Diagnostics_SeverityToString(HDY_DiagnosticSeverity severity);
const char* HDY_Diagnostics_SourceToString(HDY_DiagnosticSource source);
const char* HDY_Diagnostics_RecoveryToString(HDY_DiagnosticRecovery recovery);
const char* HDY_Diagnostics_ProtectionActionToString(HDY_ProtectionAction action);

#endif /* HDY_DIAGNOSTICS_H */
