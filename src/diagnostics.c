#include "diagnostics.h"
#include "segment_limits.h"
#include <math.h>
#include <string.h>

static void HDY_Diagnostics_SetExecutionPriorityCode(HDY_DiagnosticInfo* diagnostic) {
    if (diagnostic == NULL) {
        return;
    }

    diagnostic->code = HDY_DIAG_CODE_NONE;
    diagnostic->severity = HDY_DIAG_SEVERITY_NONE;

    if (diagnostic->timeout) {
        diagnostic->code = HDY_DIAG_CODE_TIMEOUT;
        diagnostic->severity = HDY_DIAG_SEVERITY_FAULT;
        return;
    }

    if (diagnostic->overPressure) {
        diagnostic->code = HDY_DIAG_CODE_OVER_PRESSURE;
        diagnostic->severity = HDY_DIAG_SEVERITY_WARNING;
        return;
    }

    if (diagnostic->underPressure) {
        diagnostic->code = HDY_DIAG_CODE_UNDER_PRESSURE;
        diagnostic->severity = HDY_DIAG_SEVERITY_WARNING;
        return;
    }

    if (diagnostic->flowDeviation) {
        diagnostic->code = HDY_DIAG_CODE_FLOW_DEVIATION;
        diagnostic->severity = HDY_DIAG_SEVERITY_WARNING;
        return;
    }

    if (diagnostic->positionDeviation) {
        diagnostic->code = HDY_DIAG_CODE_POSITION_DEVIATION;
        diagnostic->severity = HDY_DIAG_SEVERITY_WARNING;
        return;
    }

    if (diagnostic->velocityDeviation) {
        diagnostic->code = HDY_DIAG_CODE_VELOCITY_DEVIATION;
        diagnostic->severity = HDY_DIAG_SEVERITY_WARNING;
    }
}

void HDY_Diagnostics_Clear(HDY_DiagnosticInfo* diagnostic) {
    if (diagnostic == NULL) {
        return;
    }

    memset(diagnostic, 0, sizeof(*diagnostic));
    diagnostic->code = HDY_DIAG_CODE_NONE;
    diagnostic->severity = HDY_DIAG_SEVERITY_NONE;
}

void HDY_Diagnostics_SetMessage(HDY_DiagnosticInfo* diagnostic, const char* message) {
    if (diagnostic == NULL) {
        return;
    }

    diagnostic->message[0] = '\0';
    if (message == NULL) {
        return;
    }

    strncpy(diagnostic->message, message, HDY_MESSAGE_MAX - 1);
    diagnostic->message[HDY_MESSAGE_MAX - 1] = '\0';
}

void HDY_Diagnostics_SetEvent(HDY_DiagnosticInfo* diagnostic,
                              HDY_DiagnosticCode code,
                              HDY_DiagnosticSeverity severity,
                              const char* message) {
    if (diagnostic == NULL) {
        return;
    }

    HDY_Diagnostics_Clear(diagnostic);
    diagnostic->code = code;
    diagnostic->severity = severity;
    HDY_Diagnostics_SetMessage(diagnostic, message);
}

void HDY_Diagnostics_UpdateExecution(HDY_DiagnosticInfo* diagnostic,
                                     const HDY_DiagnosticsContext* context) {
    HDY_REAL pressureTolerance;
    HDY_REAL flowTolerance;
    HDY_REAL positionTolerance;
    HDY_REAL velocityTolerance;
    HDY_REAL timeoutLimit;

    if (diagnostic == NULL || context == NULL || context->axisRef == NULL ||
        context->segment == NULL || context->plannerOutput == NULL) {
        return;
    }

    HDY_Diagnostics_Clear(diagnostic);

    pressureTolerance = HDY_Segment_GetPressureTolerance(context->segment);
    flowTolerance = HDY_Segment_GetFlowTolerance(context->segment);
    positionTolerance = HDY_Segment_GetPositionTolerance(context->segment);
    velocityTolerance = HDY_Segment_GetVelocityTolerance(context->segment);
    timeoutLimit = HDY_Segment_GetTimeoutLimit(context->segment);

    diagnostic->pressureError = context->pressureReference - context->axisRef->pressure;
    diagnostic->flowError = context->commandedFlow - fabs(context->axisRef->flow);
    diagnostic->velocityError = context->plannerOutput->targetVelocity - context->axisRef->velocity;

    diagnostic->overPressure = context->axisRef->pressure > context->pressureReference + pressureTolerance;
    diagnostic->underPressure = context->axisRef->pressure < context->pressureReference - pressureTolerance;
    diagnostic->flowDeviation = fabs(diagnostic->flowError) > flowTolerance;
    diagnostic->positionDeviation = (context->segment->endCondition == HDY_END_POSITION) &&
        (fabs(context->segment->targetPosition - context->axisRef->position) > positionTolerance);
    diagnostic->velocityDeviation = fabs(diagnostic->velocityError) > velocityTolerance;

    if (timeoutLimit > 0.0 && context->elapsedTime > timeoutLimit) {
        diagnostic->timeout = true;
        HDY_Diagnostics_SetMessage(diagnostic, "Segment timeout limit exceeded");
    }

    HDY_Diagnostics_SetExecutionPriorityCode(diagnostic);
}
