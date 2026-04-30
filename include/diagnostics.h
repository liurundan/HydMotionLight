#ifndef HYD_DIAGNOSTICS_H
#define HYD_DIAGNOSTICS_H

#include "common_types.h"

typedef struct {
    const HYD_AxisRef* axisRef;
    const HYD_MotionSegment* segment;
    const HYD_ExecutionReference* references;
    HYD_BOOL enableVelocityDeviationCheck;  /* 默认false，设为true启用速度偏差诊断 */
} HYD_DiagnosticsContext;

void HYD_Diagnostics_Clear(HYD_DiagnosticInfo* diagnostic);
void HYD_Diagnostics_SetEvent(HYD_DiagnosticInfo* diagnostic,
                              HYD_DiagnosticCode code,
                              HYD_DiagnosticSeverity severity);

void HYD_Diagnostics_ClearSnapshot(HYD_DiagnosticSnapshot* snapshot);
void HYD_Diagnostics_CaptureSnapshot(HYD_DiagnosticSnapshot* snapshot,
                                     const HYD_DiagnosticInfo* diagnostic,
                                     const HYD_AxisRef* axisRef,
                                     const HYD_ExecutionReference* references,
                                     HYD_TIME eventTimestamp,
                                     HYD_UINT8 segmentIndex,
                                     HYD_UINT8 segmentTag,
                                     HYD_ControllerStatus status,
                                     HYD_BOOL active,
                                     HYD_BOOL finished,
                                     HYD_BOOL fault);
void HYD_DiagnosticsHistory_Clear(HYD_DiagnosticHistory* history);
void HYD_DiagnosticsHistory_Push(HYD_DiagnosticHistory* history,
                                 const HYD_DiagnosticSnapshot* snapshot);
HYD_BOOL HYD_DiagnosticsHistory_GetEntry(const HYD_DiagnosticHistory* history,
                                         HYD_UINT8 chronologicalIndex,
                                         HYD_DiagnosticSnapshot* snapshot);  /* Only index 0 is valid; returns the latest snapshot. */
HYD_BOOL HYD_DiagnosticsHistory_GetLatest(const HYD_DiagnosticHistory* history,
                                          HYD_DiagnosticSnapshot* snapshot);
HYD_DiagnosticFlags HYD_Diagnostics_GetFlagMask(const HYD_DiagnosticInfo* diagnostic);
HYD_BOOL HYD_Diagnostics_HasFlag(const HYD_DiagnosticInfo* diagnostic,
                                 HYD_DiagnosticFlag flag);
const char* HYD_Diagnostics_CodeToString(HYD_DiagnosticCode code);
const char* HYD_Diagnostics_SeverityToString(HYD_DiagnosticSeverity severity);
const char* HYD_Diagnostics_SourceToString(HYD_DiagnosticSource source);
const char* HYD_Diagnostics_RecoveryToString(HYD_DiagnosticRecovery recovery);
const char* HYD_Diagnostics_ProtectionActionToString(HYD_ProtectionAction action);

#endif /* HYD_DIAGNOSTICS_H */
