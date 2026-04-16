#ifndef HDY_DIAGNOSTICS_H
#define HDY_DIAGNOSTICS_H

#include "common_types.h"
#include "motion_planner.h"

typedef struct {
    const HDY_AxisRef* axisRef;
    const HDY_MotionSegment* segment;
    const HDY_MotionPlannerOutput* plannerOutput;
    HDY_REAL commandedFlow;
    HDY_REAL pressureReference;
    HDY_REAL elapsedTime;
} HDY_DiagnosticsContext;

void HDY_Diagnostics_Clear(HDY_DiagnosticInfo* diagnostic);
void HDY_Diagnostics_SetMessage(HDY_DiagnosticInfo* diagnostic, const char* message);
void HDY_Diagnostics_SetEvent(HDY_DiagnosticInfo* diagnostic,
                              HDY_DiagnosticCode code,
                              HDY_DiagnosticSeverity severity,
                              const char* message);
void HDY_Diagnostics_UpdateExecution(HDY_DiagnosticInfo* diagnostic,
                                     const HDY_DiagnosticsContext* context);

#endif /* HDY_DIAGNOSTICS_H */
