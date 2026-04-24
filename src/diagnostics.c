#include "diagnostics.h"
#include "segment_limits.h"
#include <math.h>
#include <string.h>

typedef struct {
    HDY_DiagnosticCode code;
    HDY_DiagnosticSeverity severity;
    HDY_DiagnosticSource source;
    HDY_DiagnosticRecovery recovery;
    HDY_ProtectionAction protectionAction;
    const char* defaultMessage;
} HDY_DiagnosticSpec;

static const HDY_DiagnosticSpec HDY_DIAGNOSTIC_SPECS[] = {
    {HDY_DIAG_CODE_NONE,
     HDY_DIAG_SEVERITY_NONE,
     HDY_DIAG_SOURCE_NONE,
     HDY_DIAG_RECOVERY_NONE,
     HDY_PROTECTION_ACTION_NONE,
     ""},
    {HDY_DIAG_CODE_RECIPE_EMPTY,
     HDY_DIAG_SEVERITY_WARNING,
     HDY_DIAG_SOURCE_RECIPE,
     HDY_DIAG_RECOVERY_RELOAD_RECIPE,
     HDY_PROTECTION_ACTION_WARNING,
     "Recipe is empty"},
    {HDY_DIAG_CODE_RECIPE_TOO_LARGE,
     HDY_DIAG_SEVERITY_WARNING,
     HDY_DIAG_SOURCE_RECIPE,
     HDY_DIAG_RECOVERY_RELOAD_RECIPE,
     HDY_PROTECTION_ACTION_WARNING,
     "Recipe exceeds maximum segment capacity"},
    {HDY_DIAG_CODE_SEGMENT_INVALID,
     HDY_DIAG_SEVERITY_WARNING,
     HDY_DIAG_SOURCE_RECIPE,
     HDY_DIAG_RECOVERY_RELOAD_RECIPE,
     HDY_PROTECTION_ACTION_WARNING,
     "Recipe segment is invalid"},
    {HDY_DIAG_CODE_RUNTIME_CONFIG_INVALID,
     HDY_DIAG_SEVERITY_FAULT,
     HDY_DIAG_SOURCE_RUNTIME,
     HDY_DIAG_RECOVERY_RESET_CONTROLLER,
     HDY_PROTECTION_ACTION_STOP,
     "Runtime configuration is invalid"},
    {HDY_DIAG_CODE_START_CONTEXT_INVALID,
     HDY_DIAG_SEVERITY_WARNING,
     HDY_DIAG_SOURCE_COMMAND,
     HDY_DIAG_RECOVERY_CHECK_COMMAND,
     HDY_PROTECTION_ACTION_WARNING,
     "Start context is invalid"},
    {HDY_DIAG_CODE_COMMAND_NOT_ALLOWED,
     HDY_DIAG_SEVERITY_WARNING,
     HDY_DIAG_SOURCE_COMMAND,
     HDY_DIAG_RECOVERY_CHECK_COMMAND,
     HDY_PROTECTION_ACTION_WARNING,
     "Command is not allowed in the current state"},
    {HDY_DIAG_CODE_NO_RECIPE,
     HDY_DIAG_SEVERITY_WARNING,
     HDY_DIAG_SOURCE_COMMAND,
     HDY_DIAG_RECOVERY_RELOAD_RECIPE,
     HDY_PROTECTION_ACTION_WARNING,
     "No recipe loaded"},
    {HDY_DIAG_CODE_NO_DIRECT_SEGMENT,
     HDY_DIAG_SEVERITY_WARNING,
     HDY_DIAG_SOURCE_COMMAND,
     HDY_DIAG_RECOVERY_CHECK_COMMAND,
     HDY_PROTECTION_ACTION_WARNING,
     "No direct segment configured"},
    {HDY_DIAG_CODE_SEGMENT_INDEX_OUT_OF_RANGE,
     HDY_DIAG_SEVERITY_WARNING,
     HDY_DIAG_SOURCE_COMMAND,
     HDY_DIAG_RECOVERY_CHECK_COMMAND,
     HDY_PROTECTION_ACTION_WARNING,
     "Segment index is out of range"},
    {HDY_DIAG_CODE_SEGMENT_NOT_COMPLETED,
     HDY_DIAG_SEVERITY_WARNING,
     HDY_DIAG_SOURCE_COMMAND,
     HDY_DIAG_RECOVERY_CHECK_COMMAND,
     HDY_PROTECTION_ACTION_WARNING,
     "Segment has not completed"},
    {HDY_DIAG_CODE_RECIPE_ALREADY_FINISHED,
     HDY_DIAG_SEVERITY_INFO,
     HDY_DIAG_SOURCE_COMMAND,
     HDY_DIAG_RECOVERY_CHECK_COMMAND,
     HDY_PROTECTION_ACTION_NONE,
     "Recipe is already finished"},
    {HDY_DIAG_CODE_ABORTED,
     HDY_DIAG_SEVERITY_INFO,
     HDY_DIAG_SOURCE_COMMAND,
     HDY_DIAG_RECOVERY_NONE,
     HDY_PROTECTION_ACTION_NONE,
     "Execution aborted by caller"},
    {HDY_DIAG_CODE_TIMEOUT,
     HDY_DIAG_SEVERITY_FAULT,
     HDY_DIAG_SOURCE_EXECUTION,
     HDY_DIAG_RECOVERY_RESTART_SEGMENT,
     HDY_PROTECTION_ACTION_STOP,
     "Segment timeout limit exceeded"},
    {HDY_DIAG_CODE_OVER_PRESSURE,
     HDY_DIAG_SEVERITY_WARNING,
     HDY_DIAG_SOURCE_EXECUTION,
     HDY_DIAG_RECOVERY_CHECK_COMMAND,
     HDY_PROTECTION_ACTION_DERATE,
     "Measured pressure exceeds upper tolerance"},
    {HDY_DIAG_CODE_UNDER_PRESSURE,
     HDY_DIAG_SEVERITY_WARNING,
     HDY_DIAG_SOURCE_EXECUTION,
     HDY_DIAG_RECOVERY_CHECK_COMMAND,
     HDY_PROTECTION_ACTION_WARNING,
     "Measured pressure is below lower tolerance"},
    {HDY_DIAG_CODE_FLOW_DEVIATION,
     HDY_DIAG_SEVERITY_WARNING,
     HDY_DIAG_SOURCE_EXECUTION,
     HDY_DIAG_RECOVERY_CHECK_COMMAND,
     HDY_PROTECTION_ACTION_DERATE,
     "Measured flow deviates from reference"},
    {HDY_DIAG_CODE_POSITION_DEVIATION,
     HDY_DIAG_SEVERITY_WARNING,
     HDY_DIAG_SOURCE_EXECUTION,
     HDY_DIAG_RECOVERY_CHECK_COMMAND,
     HDY_PROTECTION_ACTION_WARNING,
     "Measured position deviates from target"},
    {HDY_DIAG_CODE_VELOCITY_DEVIATION,
     HDY_DIAG_SEVERITY_WARNING,
     HDY_DIAG_SOURCE_EXECUTION,
     HDY_DIAG_RECOVERY_CHECK_COMMAND,
     HDY_PROTECTION_ACTION_WARNING,
     "Measured velocity deviates from reference"},
    {HDY_DIAG_CODE_SENSOR_FAULT,
     HDY_DIAG_SEVERITY_FAULT,
     HDY_DIAG_SOURCE_SENSOR,
     HDY_DIAG_RECOVERY_CHECK_SENSOR,
     HDY_PROTECTION_ACTION_STOP,
     "Axis feedback is invalid"},
    {HDY_DIAG_CODE_TIMESTAMP_ROLLBACK,
     HDY_DIAG_SEVERITY_FAULT,
     HDY_DIAG_SOURCE_SENSOR,
     HDY_DIAG_RECOVERY_CHECK_SENSOR,
     HDY_PROTECTION_ACTION_STOP,
     "Axis timestamp moved backwards"},
    {HDY_DIAG_CODE_INTERNAL_ERROR,
     HDY_DIAG_SEVERITY_FAULT,
     HDY_DIAG_SOURCE_INTERNAL,
     HDY_DIAG_RECOVERY_RESET_CONTROLLER,
     HDY_PROTECTION_ACTION_STOP,
     "Internal controller error"},
};

static HDY_DiagnosticFlags HDY_Diagnostics_BuildFlagMask(const HDY_DiagnosticInfo* diagnostic) {
    HDY_DiagnosticFlags flags = HDY_DIAG_FLAG_NONE;

    if (diagnostic == NULL) {
        return flags;
    }

    if (diagnostic->overPressure) {
        flags |= HDY_DIAG_FLAG_OVER_PRESSURE;
    }
    if (diagnostic->underPressure) {
        flags |= HDY_DIAG_FLAG_UNDER_PRESSURE;
    }
    if (diagnostic->flowDeviation) {
        flags |= HDY_DIAG_FLAG_FLOW_DEVIATION;
    }
    if (diagnostic->positionDeviation) {
        flags |= HDY_DIAG_FLAG_POSITION_DEVIATION;
    }
    if (diagnostic->velocityDeviation) {
        flags |= HDY_DIAG_FLAG_VELOCITY_DEVIATION;
    }
    if (diagnostic->timeout) {
        flags |= HDY_DIAG_FLAG_TIMEOUT;
    }
    if (diagnostic->sensorFault) {
        flags |= HDY_DIAG_FLAG_SENSOR_FAULT;
    }
    if (diagnostic->timestampRollback) {
        flags |= HDY_DIAG_FLAG_TIMESTAMP_ROLLBACK;
    }

    return flags;
}

static void HDY_Diagnostics_RefreshFlags(HDY_DiagnosticInfo* diagnostic) {
    if (diagnostic == NULL) {
        return;
    }

    diagnostic->flags = HDY_Diagnostics_BuildFlagMask(diagnostic);
}

static const HDY_DiagnosticSpec* HDY_Diagnostics_FindSpec(HDY_DiagnosticCode code) {
    size_t index;

    for (index = 0U; index < (sizeof(HDY_DIAGNOSTIC_SPECS) / sizeof(HDY_DIAGNOSTIC_SPECS[0])); ++index) {
        if (HDY_DIAGNOSTIC_SPECS[index].code == code) {
            return &HDY_DIAGNOSTIC_SPECS[index];
        }
    }

    return &HDY_DIAGNOSTIC_SPECS[0];
}

/* Internal: returns the default human-readable description for a diagnostic code.
 * Intended for debug-printing only; upper layers should use code/severity/source
 * for programmatic decisions, not message strings. */
HDY_UNUSED
static const char* HDY_Diagnostics_GetDefaultMessage(HDY_DiagnosticCode code) {
    return HDY_Diagnostics_FindSpec(code)->defaultMessage;
}

static void HDY_Diagnostics_ApplySpec(HDY_DiagnosticInfo* diagnostic,
                                      HDY_DiagnosticCode code,
                                      HDY_DiagnosticSeverity severityOverride) {
    const HDY_DiagnosticSpec* spec;

    if (diagnostic == NULL) {
        return;
    }

    spec = HDY_Diagnostics_FindSpec(code);
    diagnostic->code = code;
    diagnostic->severity = (severityOverride == HDY_DIAG_SEVERITY_NONE && code != HDY_DIAG_CODE_NONE)
        ? spec->severity
        : severityOverride;
    diagnostic->source = spec->source;
    diagnostic->recovery = spec->recovery;
    diagnostic->protectionAction = spec->protectionAction;
}

void HDY_Diagnostics_Clear(HDY_DiagnosticInfo* diagnostic) {
    if (diagnostic == NULL) {
        return;
    }

    memset(diagnostic, 0, sizeof(*diagnostic));
    diagnostic->code = HDY_DIAG_CODE_NONE;
    diagnostic->severity = HDY_DIAG_SEVERITY_NONE;
    diagnostic->source = HDY_DIAG_SOURCE_NONE;
    diagnostic->recovery = HDY_DIAG_RECOVERY_NONE;
    diagnostic->protectionAction = HDY_PROTECTION_ACTION_NONE;
    diagnostic->flags = HDY_DIAG_FLAG_NONE;
}

void HDY_Diagnostics_SetEvent(HDY_DiagnosticInfo* diagnostic,
                              HDY_DiagnosticCode code,
                              HDY_DiagnosticSeverity severity) {
    if (diagnostic == NULL) {
        return;
    }

    HDY_Diagnostics_Clear(diagnostic);
    HDY_Diagnostics_ApplySpec(diagnostic, code, severity);
    HDY_Diagnostics_RefreshFlags(diagnostic);
}


void HDY_Diagnostics_ClearSnapshot(HDY_DiagnosticSnapshot* snapshot) {
    if (snapshot == NULL) {
        return;
    }

    memset(snapshot, 0, sizeof(*snapshot));
}

void HDY_Diagnostics_CaptureSnapshot(HDY_DiagnosticSnapshot* snapshot,
                                     const HDY_DiagnosticInfo* diagnostic,
                                     const HDY_AxisRef* axisRef,
                                     const HDY_ExecutionReference* references,
                                     HDY_TIME eventTimestamp,
                                     HDY_UINT8 segmentIndex,
                                     HDY_UINT8 segmentTag,
                                     HDY_ControllerStatus status,
                                     HDY_BOOL active,
                                     HDY_BOOL finished,
                                     HDY_BOOL fault) {
    if (snapshot == NULL) {
        return;
    }

    HDY_Diagnostics_ClearSnapshot(snapshot);
    snapshot->valid = (diagnostic != NULL) && (diagnostic->code != HDY_DIAG_CODE_NONE);
    snapshot->eventTimestamp = eventTimestamp;
    snapshot->segmentIndex = segmentIndex;
    snapshot->segmentTag = segmentTag;
    snapshot->status = status;
    snapshot->active = active;
    snapshot->finished = finished;
    snapshot->fault = fault;

    if (diagnostic != NULL) {
        snapshot->diagnostic = *diagnostic;
    } else {
        HDY_Diagnostics_Clear(&snapshot->diagnostic);
    }

    if (axisRef != NULL) {
        snapshot->axisRef = *axisRef;
    }

    if (references != NULL) {
        snapshot->references = *references;
    }
}

void HDY_DiagnosticsHistory_Clear(HDY_DiagnosticHistory* history) {
    if (history == NULL) {
        return;
    }

    HDY_Diagnostics_ClearSnapshot(&history->lastSnapshot);
    history->totalRecorded = 0U;
    history->hasRecord = false;
}

void HDY_DiagnosticsHistory_Push(HDY_DiagnosticHistory* history,
                                 const HDY_DiagnosticSnapshot* snapshot) {
    if (history == NULL || snapshot == NULL || !snapshot->valid) {
        return;
    }

    history->lastSnapshot = *snapshot;
    history->hasRecord = true;

    if (history->totalRecorded < UINT16_MAX) {
        history->totalRecorded++;
    }
}

HDY_BOOL HDY_DiagnosticsHistory_GetEntry(const HDY_DiagnosticHistory* history,
                                         HDY_UINT8 chronologicalIndex,
                                         HDY_DiagnosticSnapshot* snapshot) {
    if (history == NULL || snapshot == NULL) {
        return false;
    }

    /* Only index 0 is valid — it maps to the single lastSnapshot. */
    if (chronologicalIndex != 0U || !history->hasRecord) {
        return false;
    }

    *snapshot = history->lastSnapshot;
    return snapshot->valid;
}

HDY_BOOL HDY_DiagnosticsHistory_GetLatest(const HDY_DiagnosticHistory* history,
                                          HDY_DiagnosticSnapshot* snapshot) {
    if (history == NULL || snapshot == NULL || !history->hasRecord) {
        return false;
    }

    *snapshot = history->lastSnapshot;
    return snapshot->valid;
}

HDY_DiagnosticFlags HDY_Diagnostics_GetFlagMask(const HDY_DiagnosticInfo* diagnostic) {
    return HDY_Diagnostics_BuildFlagMask(diagnostic);
}

HDY_BOOL HDY_Diagnostics_HasFlag(const HDY_DiagnosticInfo* diagnostic,
                                 HDY_DiagnosticFlag flag) {
    if (flag == HDY_DIAG_FLAG_NONE) {
        return false;
    }

    return (HDY_Diagnostics_GetFlagMask(diagnostic) & (HDY_DiagnosticFlags)flag) != 0U;
}

const char* HDY_Diagnostics_CodeToString(HDY_DiagnosticCode code) {
    switch (code) {
        case HDY_DIAG_CODE_NONE:
            return "NONE";
        case HDY_DIAG_CODE_RECIPE_EMPTY:
            return "RECIPE_EMPTY";
        case HDY_DIAG_CODE_RECIPE_TOO_LARGE:
            return "RECIPE_TOO_LARGE";
        case HDY_DIAG_CODE_SEGMENT_INVALID:
            return "SEGMENT_INVALID";
        case HDY_DIAG_CODE_RUNTIME_CONFIG_INVALID:
            return "RUNTIME_CONFIG_INVALID";
        case HDY_DIAG_CODE_START_CONTEXT_INVALID:
            return "START_CONTEXT_INVALID";
        case HDY_DIAG_CODE_COMMAND_NOT_ALLOWED:
            return "COMMAND_NOT_ALLOWED";
        case HDY_DIAG_CODE_NO_RECIPE:
            return "NO_RECIPE";
        case HDY_DIAG_CODE_SEGMENT_INDEX_OUT_OF_RANGE:
            return "SEGMENT_INDEX_OUT_OF_RANGE";
        case HDY_DIAG_CODE_SEGMENT_NOT_COMPLETED:
            return "SEGMENT_NOT_COMPLETED";
        case HDY_DIAG_CODE_RECIPE_ALREADY_FINISHED:
            return "RECIPE_ALREADY_FINISHED";
        case HDY_DIAG_CODE_ABORTED:
            return "ABORTED";
        case HDY_DIAG_CODE_TIMEOUT:
            return "TIMEOUT";
        case HDY_DIAG_CODE_OVER_PRESSURE:
            return "OVER_PRESSURE";
        case HDY_DIAG_CODE_UNDER_PRESSURE:
            return "UNDER_PRESSURE";
        case HDY_DIAG_CODE_FLOW_DEVIATION:
            return "FLOW_DEVIATION";
        case HDY_DIAG_CODE_POSITION_DEVIATION:
            return "POSITION_DEVIATION";
        case HDY_DIAG_CODE_VELOCITY_DEVIATION:
            return "VELOCITY_DEVIATION";
        case HDY_DIAG_CODE_SENSOR_FAULT:
            return "SENSOR_FAULT";
        case HDY_DIAG_CODE_TIMESTAMP_ROLLBACK:
            return "TIMESTAMP_ROLLBACK";
        case HDY_DIAG_CODE_INTERNAL_ERROR:
            return "INTERNAL_ERROR";
        default:
            return "UNKNOWN";
    }
}

const char* HDY_Diagnostics_SeverityToString(HDY_DiagnosticSeverity severity) {
    switch (severity) {
        case HDY_DIAG_SEVERITY_NONE:
            return "NONE";
        case HDY_DIAG_SEVERITY_INFO:
            return "INFO";
        case HDY_DIAG_SEVERITY_WARNING:
            return "WARNING";
        case HDY_DIAG_SEVERITY_FAULT:
            return "FAULT";
        default:
            return "UNKNOWN";
    }
}

const char* HDY_Diagnostics_SourceToString(HDY_DiagnosticSource source) {
    switch (source) {
        case HDY_DIAG_SOURCE_NONE:
            return "NONE";
        case HDY_DIAG_SOURCE_RECIPE:
            return "RECIPE";
        case HDY_DIAG_SOURCE_RUNTIME:
            return "RUNTIME";
        case HDY_DIAG_SOURCE_COMMAND:
            return "COMMAND";
        case HDY_DIAG_SOURCE_EXECUTION:
            return "EXECUTION";
        case HDY_DIAG_SOURCE_SENSOR:
            return "SENSOR";
        case HDY_DIAG_SOURCE_INTERNAL:
            return "INTERNAL";
        default:
            return "UNKNOWN";
    }
}

const char* HDY_Diagnostics_RecoveryToString(HDY_DiagnosticRecovery recovery) {
    switch (recovery) {
        case HDY_DIAG_RECOVERY_NONE:
            return "NONE";
        case HDY_DIAG_RECOVERY_AUTO_CLEAR:
            return "AUTO_CLEAR";
        case HDY_DIAG_RECOVERY_CHECK_COMMAND:
            return "CHECK_COMMAND";
        case HDY_DIAG_RECOVERY_CHECK_SENSOR:
            return "CHECK_SENSOR";
        case HDY_DIAG_RECOVERY_RELOAD_RECIPE:
            return "RELOAD_RECIPE";
        case HDY_DIAG_RECOVERY_RESTART_SEGMENT:
            return "RESTART_SEGMENT";
        case HDY_DIAG_RECOVERY_RESET_CONTROLLER:
            return "RESET_CONTROLLER";
        default:
            return "UNKNOWN";
    }
}

const char* HDY_Diagnostics_ProtectionActionToString(HDY_ProtectionAction action) {
    switch (action) {
        case HDY_PROTECTION_ACTION_NONE:
            return "NONE";
        case HDY_PROTECTION_ACTION_WARNING:
            return "WARNING";
        case HDY_PROTECTION_ACTION_DERATE:
            return "DERATE";
        case HDY_PROTECTION_ACTION_STOP:
            return "STOP";
        default:
            return "UNKNOWN";
    }
}
