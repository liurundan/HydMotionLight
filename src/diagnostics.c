#include "diagnostics.h"
#include "segment_limits.h"
#include <math.h>
#include <string.h>

typedef struct {
    HYD_DiagnosticCode code;
    HYD_DiagnosticSeverity severity;
    HYD_DiagnosticSource source;
    HYD_DiagnosticRecovery recovery;
    HYD_ProtectionAction protectionAction;
    const char* defaultMessage;
} HYD_DiagnosticSpec;

static const HYD_DiagnosticSpec HYD_DIAGNOSTIC_SPECS[] = {
    {HYD_DIAG_CODE_NONE,
     HYD_DIAG_SEVERITY_NONE,
     HYD_DIAG_SOURCE_NONE,
     HYD_DIAG_RECOVERY_NONE,
     HYD_PROTECTION_ACTION_NONE,
     ""},
    {HYD_DIAG_CODE_RECIPE_EMPTY,
     HYD_DIAG_SEVERITY_WARNING,
     HYD_DIAG_SOURCE_RECIPE,
     HYD_DIAG_RECOVERY_RELOAD_RECIPE,
     HYD_PROTECTION_ACTION_WARNING,
     "Recipe is empty"},
    {HYD_DIAG_CODE_RECIPE_TOO_LARGE,
     HYD_DIAG_SEVERITY_WARNING,
     HYD_DIAG_SOURCE_RECIPE,
     HYD_DIAG_RECOVERY_RELOAD_RECIPE,
     HYD_PROTECTION_ACTION_WARNING,
     "Recipe exceeds maximum segment capacity"},
    {HYD_DIAG_CODE_SEGMENT_INVALID,
     HYD_DIAG_SEVERITY_WARNING,
     HYD_DIAG_SOURCE_RECIPE,
     HYD_DIAG_RECOVERY_RELOAD_RECIPE,
     HYD_PROTECTION_ACTION_WARNING,
     "Recipe segment is invalid"},
    {HYD_DIAG_CODE_RUNTIME_CONFIG_INVALID,
     HYD_DIAG_SEVERITY_FAULT,
     HYD_DIAG_SOURCE_RUNTIME,
     HYD_DIAG_RECOVERY_RESET_CONTROLLER,
     HYD_PROTECTION_ACTION_STOP,
     "Runtime configuration is invalid"},
    {HYD_DIAG_CODE_START_CONTEXT_INVALID,
     HYD_DIAG_SEVERITY_WARNING,
     HYD_DIAG_SOURCE_COMMAND,
     HYD_DIAG_RECOVERY_CHECK_COMMAND,
     HYD_PROTECTION_ACTION_WARNING,
     "Start context is invalid"},
    {HYD_DIAG_CODE_COMMAND_NOT_ALLOWED,
     HYD_DIAG_SEVERITY_WARNING,
     HYD_DIAG_SOURCE_COMMAND,
     HYD_DIAG_RECOVERY_CHECK_COMMAND,
     HYD_PROTECTION_ACTION_WARNING,
     "Command is not allowed in the current state"},
    {HYD_DIAG_CODE_NO_RECIPE,
     HYD_DIAG_SEVERITY_WARNING,
     HYD_DIAG_SOURCE_COMMAND,
     HYD_DIAG_RECOVERY_RELOAD_RECIPE,
     HYD_PROTECTION_ACTION_WARNING,
     "No recipe loaded"},
    {HYD_DIAG_CODE_NO_DIRECT_SEGMENT,
     HYD_DIAG_SEVERITY_WARNING,
     HYD_DIAG_SOURCE_COMMAND,
     HYD_DIAG_RECOVERY_CHECK_COMMAND,
     HYD_PROTECTION_ACTION_WARNING,
     "No direct segment configured"},
    {HYD_DIAG_CODE_SEGMENT_INDEX_OUT_OF_RANGE,
     HYD_DIAG_SEVERITY_WARNING,
     HYD_DIAG_SOURCE_COMMAND,
     HYD_DIAG_RECOVERY_CHECK_COMMAND,
     HYD_PROTECTION_ACTION_WARNING,
     "Segment index is out of range"},
    {HYD_DIAG_CODE_SEGMENT_NOT_COMPLETED,
     HYD_DIAG_SEVERITY_WARNING,
     HYD_DIAG_SOURCE_COMMAND,
     HYD_DIAG_RECOVERY_CHECK_COMMAND,
     HYD_PROTECTION_ACTION_WARNING,
     "Segment has not completed"},
    {HYD_DIAG_CODE_RECIPE_ALREADY_FINISHED,
     HYD_DIAG_SEVERITY_INFO,
     HYD_DIAG_SOURCE_COMMAND,
     HYD_DIAG_RECOVERY_CHECK_COMMAND,
     HYD_PROTECTION_ACTION_NONE,
     "Recipe is already finished"},
    {HYD_DIAG_CODE_ABORTED,
     HYD_DIAG_SEVERITY_INFO,
     HYD_DIAG_SOURCE_COMMAND,
     HYD_DIAG_RECOVERY_NONE,
     HYD_PROTECTION_ACTION_NONE,
     "Execution aborted by caller"},
    {HYD_DIAG_CODE_TIMEOUT,
     HYD_DIAG_SEVERITY_FAULT,
     HYD_DIAG_SOURCE_EXECUTION,
     HYD_DIAG_RECOVERY_RESTART_SEGMENT,
     HYD_PROTECTION_ACTION_STOP,
     "Segment timeout limit exceeded"},
    {HYD_DIAG_CODE_OVER_PRESSURE,
     HYD_DIAG_SEVERITY_WARNING,
     HYD_DIAG_SOURCE_EXECUTION,
     HYD_DIAG_RECOVERY_CHECK_COMMAND,
     HYD_PROTECTION_ACTION_DERATE,
     "Measured pressure exceeds upper tolerance"},
    {HYD_DIAG_CODE_UNDER_PRESSURE,
     HYD_DIAG_SEVERITY_WARNING,
     HYD_DIAG_SOURCE_EXECUTION,
     HYD_DIAG_RECOVERY_CHECK_COMMAND,
     HYD_PROTECTION_ACTION_WARNING,
     "Measured pressure is below lower tolerance"},
    {HYD_DIAG_CODE_FLOW_DEVIATION,
     HYD_DIAG_SEVERITY_WARNING,
     HYD_DIAG_SOURCE_EXECUTION,
     HYD_DIAG_RECOVERY_CHECK_COMMAND,
     HYD_PROTECTION_ACTION_DERATE,
     "Measured flow deviates from reference"},
    {HYD_DIAG_CODE_POSITION_DEVIATION,
     HYD_DIAG_SEVERITY_WARNING,
     HYD_DIAG_SOURCE_EXECUTION,
     HYD_DIAG_RECOVERY_CHECK_COMMAND,
     HYD_PROTECTION_ACTION_WARNING,
     "Measured position deviates from target"},
    {HYD_DIAG_CODE_VELOCITY_DEVIATION,
     HYD_DIAG_SEVERITY_WARNING,
     HYD_DIAG_SOURCE_EXECUTION,
     HYD_DIAG_RECOVERY_CHECK_COMMAND,
     HYD_PROTECTION_ACTION_WARNING,
     "Measured velocity deviates from reference"},
    {HYD_DIAG_CODE_SENSOR_FAULT,
     HYD_DIAG_SEVERITY_FAULT,
     HYD_DIAG_SOURCE_SENSOR,
     HYD_DIAG_RECOVERY_CHECK_SENSOR,
     HYD_PROTECTION_ACTION_STOP,
     "Axis feedback is invalid"},
    {HYD_DIAG_CODE_TIMESTAMP_ROLLBACK,
     HYD_DIAG_SEVERITY_FAULT,
     HYD_DIAG_SOURCE_SENSOR,
     HYD_DIAG_RECOVERY_CHECK_SENSOR,
     HYD_PROTECTION_ACTION_STOP,
     "Axis timestamp moved backwards"},
    {HYD_DIAG_CODE_PRESSURE_CEILING_EXCEEDED,
     HYD_DIAG_SEVERITY_WARNING,
     HYD_DIAG_SOURCE_EXECUTION,
     HYD_DIAG_RECOVERY_CHECK_COMMAND,
     HYD_PROTECTION_ACTION_DERATE,
     "Pressure exceeded segment soft ceiling"},
    {HYD_DIAG_CODE_PRESSURE_CEILING_VIOLATED,
     HYD_DIAG_SEVERITY_FAULT,
     HYD_DIAG_SOURCE_EXECUTION,
     HYD_DIAG_RECOVERY_RESTART_SEGMENT,
     HYD_PROTECTION_ACTION_STOP,
     "Pressure remained above ceiling beyond fault-escalation window"},
    {
        HYD_DIAG_CODE_OVER_PRESSURE_LIMIT,
        HYD_DIAG_SEVERITY_WARNING,
        HYD_DIAG_SOURCE_EXECUTION,
        HYD_DIAG_RECOVERY_CHECK_COMMAND,
        HYD_PROTECTION_ACTION_DERATE,
        "Pressure limit active: proportional flow reduction"
    },
    {
        HYD_DIAG_CODE_OVER_PRESSURE_LIMIT_FAULT,
        HYD_DIAG_SEVERITY_FAULT,
        HYD_DIAG_SOURCE_EXECUTION,
        HYD_DIAG_RECOVERY_RESTART_SEGMENT,
        HYD_PROTECTION_ACTION_STOP,
        "Pressure limit violated: sustained over-pressure, emergency stop"
    },
    {
        HYD_DIAG_CODE_SOFT_LIMIT_REACHED,
        HYD_DIAG_SEVERITY_WARNING,
        HYD_DIAG_SOURCE_EXECUTION,
        HYD_DIAG_RECOVERY_CHECK_COMMAND,
        HYD_PROTECTION_ACTION_DERATE,
        "Soft position limit reached: flow reduction active"
    },
    {
        HYD_DIAG_CODE_SOFT_LIMIT_VIOLATED,
        HYD_DIAG_SEVERITY_FAULT,
        HYD_DIAG_SOURCE_EXECUTION,
        HYD_DIAG_RECOVERY_RESTART_SEGMENT,
        HYD_PROTECTION_ACTION_STOP,
        "Soft position limit violated: beyond stroke boundary"
    },
    {HYD_DIAG_CODE_PUMP_DIRECTION_CONFLICT,
     HYD_DIAG_SEVERITY_WARNING,
     HYD_DIAG_SOURCE_RUNTIME,
     HYD_DIAG_RECOVERY_NONE,
     HYD_PROTECTION_ACTION_NONE,
     "Pump direction conflict: opposing planned directions on active axes."},
    {HYD_DIAG_CODE_INTERNAL_ERROR,
     HYD_DIAG_SEVERITY_FAULT,
     HYD_DIAG_SOURCE_INTERNAL,
     HYD_DIAG_RECOVERY_RESET_CONTROLLER,
     HYD_PROTECTION_ACTION_STOP,
     "Internal controller error"},
};

static HYD_DiagnosticFlags HYD_Diagnostics_BuildFlagMask(const HYD_DiagnosticInfo* diagnostic) {
    HYD_DiagnosticFlags flags = HYD_DIAG_FLAG_NONE;

    if (diagnostic == NULL) {
        return flags;
    }

    if (diagnostic->overPressure) {
        flags |= HYD_DIAG_FLAG_OVER_PRESSURE;
    }
    if (diagnostic->underPressure) {
        flags |= HYD_DIAG_FLAG_UNDER_PRESSURE;
    }
    if (diagnostic->flowDeviation) {
        flags |= HYD_DIAG_FLAG_FLOW_DEVIATION;
    }
    if (diagnostic->positionDeviation) {
        flags |= HYD_DIAG_FLAG_POSITION_DEVIATION;
    }
    if (diagnostic->velocityDeviation) {
        flags |= HYD_DIAG_FLAG_VELOCITY_DEVIATION;
    }
    if (diagnostic->timeout) {
        flags |= HYD_DIAG_FLAG_TIMEOUT;
    }
    if (diagnostic->sensorFault) {
        flags |= HYD_DIAG_FLAG_SENSOR_FAULT;
    }
    if (diagnostic->timestampRollback) {
        flags |= HYD_DIAG_FLAG_TIMESTAMP_ROLLBACK;
    }
    if (diagnostic->pressureCeilingExceeded) {
        flags |= HYD_DIAG_FLAG_PRESSURE_CEILING_EXCEEDED;
    }
    if (diagnostic->pressureCeilingViolated) {
        flags |= HYD_DIAG_FLAG_PRESSURE_CEILING_VIOLATED;
    }

    return flags;
}

static void HYD_Diagnostics_RefreshFlags(HYD_DiagnosticInfo* diagnostic) {
    if (diagnostic == NULL) {
        return;
    }

    diagnostic->flags = HYD_Diagnostics_BuildFlagMask(diagnostic);
}

static const HYD_DiagnosticSpec* HYD_Diagnostics_FindSpec(HYD_DiagnosticCode code) {
    size_t index;

    for (index = 0U; index < (sizeof(HYD_DIAGNOSTIC_SPECS) / sizeof(HYD_DIAGNOSTIC_SPECS[0])); ++index) {
        if (HYD_DIAGNOSTIC_SPECS[index].code == code) {
            return &HYD_DIAGNOSTIC_SPECS[index];
        }
    }

    return &HYD_DIAGNOSTIC_SPECS[0];
}

/* Internal: returns the default human-readable description for a diagnostic code.
 * Intended for debug-printing only; upper layers should use code/severity/source
 * for programmatic decisions, not message strings. */
HYD_UNUSED
static const char* HYD_Diagnostics_GetDefaultMessage(HYD_DiagnosticCode code) {
    return HYD_Diagnostics_FindSpec(code)->defaultMessage;
}

static void HYD_Diagnostics_ApplySpec(HYD_DiagnosticInfo* diagnostic,
                                      HYD_DiagnosticCode code,
                                      HYD_DiagnosticSeverity severityOverride) {
    const HYD_DiagnosticSpec* spec;

    if (diagnostic == NULL) {
        return;
    }

    spec = HYD_Diagnostics_FindSpec(code);
    diagnostic->code = code;
    diagnostic->severity = (severityOverride == HYD_DIAG_SEVERITY_NONE && code != HYD_DIAG_CODE_NONE)
        ? spec->severity
        : severityOverride;
    diagnostic->source = spec->source;
    diagnostic->recovery = spec->recovery;
    diagnostic->protectionAction = spec->protectionAction;
}

void HYD_Diagnostics_Clear(HYD_DiagnosticInfo* diagnostic) {
    if (diagnostic == NULL) {
        return;
    }

    memset(diagnostic, 0, sizeof(*diagnostic));
    diagnostic->code = HYD_DIAG_CODE_NONE;
    diagnostic->severity = HYD_DIAG_SEVERITY_NONE;
    diagnostic->source = HYD_DIAG_SOURCE_NONE;
    diagnostic->recovery = HYD_DIAG_RECOVERY_NONE;
    diagnostic->protectionAction = HYD_PROTECTION_ACTION_NONE;
    diagnostic->flags = HYD_DIAG_FLAG_NONE;
}

void HYD_Diagnostics_SetEvent(HYD_DiagnosticInfo* diagnostic,
                              HYD_DiagnosticCode code,
                              HYD_DiagnosticSeverity severity) {
    if (diagnostic == NULL) {
        return;
    }

    HYD_Diagnostics_Clear(diagnostic);
    HYD_Diagnostics_ApplySpec(diagnostic, code, severity);
    HYD_Diagnostics_RefreshFlags(diagnostic);
}


void HYD_Diagnostics_ClearSnapshot(HYD_DiagnosticSnapshot* snapshot) {
    if (snapshot == NULL) {
        return;
    }

    memset(snapshot, 0, sizeof(*snapshot));
}

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
                                     HYD_BOOL fault) {
    if (snapshot == NULL) {
        return;
    }

    HYD_Diagnostics_ClearSnapshot(snapshot);
    snapshot->valid = (diagnostic != NULL) && (diagnostic->code != HYD_DIAG_CODE_NONE);
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
        HYD_Diagnostics_Clear(&snapshot->diagnostic);
    }

    if (axisRef != NULL) {
        snapshot->axisRef = *axisRef;
    }

    if (references != NULL) {
        snapshot->references = *references;
    }
}

void HYD_DiagnosticsHistory_Clear(HYD_DiagnosticHistory* history) {
    if (history == NULL) {
        return;
    }

    HYD_Diagnostics_ClearSnapshot(&history->lastSnapshot);
    history->totalRecorded = 0U;
    history->hasRecord = false;
}

void HYD_DiagnosticsHistory_Push(HYD_DiagnosticHistory* history,
                                 const HYD_DiagnosticSnapshot* snapshot) {
    if (history == NULL || snapshot == NULL || !snapshot->valid) {
        return;
    }

    history->lastSnapshot = *snapshot;
    history->hasRecord = true;

    if (history->totalRecorded < UINT16_MAX) {
        history->totalRecorded++;
    }
}

HYD_BOOL HYD_DiagnosticsHistory_GetEntry(const HYD_DiagnosticHistory* history,
                                         HYD_UINT8 chronologicalIndex,
                                         HYD_DiagnosticSnapshot* snapshot) {
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

HYD_BOOL HYD_DiagnosticsHistory_GetLatest(const HYD_DiagnosticHistory* history,
                                          HYD_DiagnosticSnapshot* snapshot) {
    if (history == NULL || snapshot == NULL || !history->hasRecord) {
        return false;
    }

    *snapshot = history->lastSnapshot;
    return snapshot->valid;
}

HYD_DiagnosticFlags HYD_Diagnostics_GetFlagMask(const HYD_DiagnosticInfo* diagnostic) {
    return HYD_Diagnostics_BuildFlagMask(diagnostic);
}

HYD_BOOL HYD_Diagnostics_HasFlag(const HYD_DiagnosticInfo* diagnostic,
                                 HYD_DiagnosticFlag flag) {
    if (flag == HYD_DIAG_FLAG_NONE) {
        return false;
    }

    return (HYD_Diagnostics_GetFlagMask(diagnostic) & (HYD_DiagnosticFlags)flag) != 0U;
}

const char* HYD_Diagnostics_CodeToString(HYD_DiagnosticCode code) {
    switch (code) {
        case HYD_DIAG_CODE_NONE:
            return "NONE";
        case HYD_DIAG_CODE_RECIPE_EMPTY:
            return "RECIPE_EMPTY";
        case HYD_DIAG_CODE_RECIPE_TOO_LARGE:
            return "RECIPE_TOO_LARGE";
        case HYD_DIAG_CODE_SEGMENT_INVALID:
            return "SEGMENT_INVALID";
        case HYD_DIAG_CODE_RUNTIME_CONFIG_INVALID:
            return "RUNTIME_CONFIG_INVALID";
        case HYD_DIAG_CODE_START_CONTEXT_INVALID:
            return "START_CONTEXT_INVALID";
        case HYD_DIAG_CODE_COMMAND_NOT_ALLOWED:
            return "COMMAND_NOT_ALLOWED";
        case HYD_DIAG_CODE_NO_RECIPE:
            return "NO_RECIPE";
        case HYD_DIAG_CODE_SEGMENT_INDEX_OUT_OF_RANGE:
            return "SEGMENT_INDEX_OUT_OF_RANGE";
        case HYD_DIAG_CODE_SEGMENT_NOT_COMPLETED:
            return "SEGMENT_NOT_COMPLETED";
        case HYD_DIAG_CODE_RECIPE_ALREADY_FINISHED:
            return "RECIPE_ALREADY_FINISHED";
        case HYD_DIAG_CODE_ABORTED:
            return "ABORTED";
        case HYD_DIAG_CODE_TIMEOUT:
            return "TIMEOUT";
        case HYD_DIAG_CODE_OVER_PRESSURE:
            return "OVER_PRESSURE";
        case HYD_DIAG_CODE_UNDER_PRESSURE:
            return "UNDER_PRESSURE";
        case HYD_DIAG_CODE_FLOW_DEVIATION:
            return "FLOW_DEVIATION";
        case HYD_DIAG_CODE_POSITION_DEVIATION:
            return "POSITION_DEVIATION";
        case HYD_DIAG_CODE_VELOCITY_DEVIATION:
            return "VELOCITY_DEVIATION";
        case HYD_DIAG_CODE_SENSOR_FAULT:
            return "SENSOR_FAULT";
        case HYD_DIAG_CODE_TIMESTAMP_ROLLBACK:
            return "TIMESTAMP_ROLLBACK";
        case HYD_DIAG_CODE_PRESSURE_CEILING_EXCEEDED:
            return "PRESSURE_CEILING_EXCEEDED";
        case HYD_DIAG_CODE_PRESSURE_CEILING_VIOLATED:
            return "PRESSURE_CEILING_VIOLATED";
        case HYD_DIAG_CODE_PUMP_DIRECTION_CONFLICT:
            return "PUMP_DIRECTION_CONFLICT";
        case HYD_DIAG_CODE_OVER_PRESSURE_LIMIT:
            return "OVER_PRESSURE_LIMIT";
        case HYD_DIAG_CODE_OVER_PRESSURE_LIMIT_FAULT:
            return "OVER_PRESSURE_LIMIT_FAULT";
        case HYD_DIAG_CODE_SOFT_LIMIT_REACHED:
            return "SOFT_LIMIT_REACHED";
        case HYD_DIAG_CODE_SOFT_LIMIT_VIOLATED:
            return "SOFT_LIMIT_VIOLATED";
        case HYD_DIAG_CODE_INTERNAL_ERROR:
            return "INTERNAL_ERROR";
        default:
            return "UNKNOWN";
    }
}

const char* HYD_Diagnostics_SeverityToString(HYD_DiagnosticSeverity severity) {
    switch (severity) {
        case HYD_DIAG_SEVERITY_NONE:
            return "NONE";
        case HYD_DIAG_SEVERITY_INFO:
            return "INFO";
        case HYD_DIAG_SEVERITY_WARNING:
            return "WARNING";
        case HYD_DIAG_SEVERITY_FAULT:
            return "FAULT";
        default:
            return "UNKNOWN";
    }
}

const char* HYD_Diagnostics_SourceToString(HYD_DiagnosticSource source) {
    switch (source) {
        case HYD_DIAG_SOURCE_NONE:
            return "NONE";
        case HYD_DIAG_SOURCE_RECIPE:
            return "RECIPE";
        case HYD_DIAG_SOURCE_RUNTIME:
            return "RUNTIME";
        case HYD_DIAG_SOURCE_COMMAND:
            return "COMMAND";
        case HYD_DIAG_SOURCE_EXECUTION:
            return "EXECUTION";
        case HYD_DIAG_SOURCE_SENSOR:
            return "SENSOR";
        case HYD_DIAG_SOURCE_INTERNAL:
            return "INTERNAL";
        default:
            return "UNKNOWN";
    }
}

const char* HYD_Diagnostics_RecoveryToString(HYD_DiagnosticRecovery recovery) {
    switch (recovery) {
        case HYD_DIAG_RECOVERY_NONE:
            return "NONE";
        case HYD_DIAG_RECOVERY_AUTO_CLEAR:
            return "AUTO_CLEAR";
        case HYD_DIAG_RECOVERY_CHECK_COMMAND:
            return "CHECK_COMMAND";
        case HYD_DIAG_RECOVERY_CHECK_SENSOR:
            return "CHECK_SENSOR";
        case HYD_DIAG_RECOVERY_RELOAD_RECIPE:
            return "RELOAD_RECIPE";
        case HYD_DIAG_RECOVERY_RESTART_SEGMENT:
            return "RESTART_SEGMENT";
        case HYD_DIAG_RECOVERY_RESET_CONTROLLER:
            return "RESET_CONTROLLER";
        default:
            return "UNKNOWN";
    }
}

const char* HYD_Diagnostics_ProtectionActionToString(HYD_ProtectionAction action) {
    switch (action) {
        case HYD_PROTECTION_ACTION_NONE:
            return "NONE";
        case HYD_PROTECTION_ACTION_WARNING:
            return "WARNING";
        case HYD_PROTECTION_ACTION_DERATE:
            return "DERATE";
        case HYD_PROTECTION_ACTION_STOP:
            return "STOP";
        default:
            return "UNKNOWN";
    }
}
