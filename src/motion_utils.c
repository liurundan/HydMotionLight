#include "motion_utils.h"
#include "common_types.h"
#include "hyd_config.h"
#include <math.h>
#include <string.h>

HYD_REAL HYD_MotionUtils_MinReal(HYD_REAL left, HYD_REAL right) {
    return (left < right) ? left : right;
}

HYD_REAL HYD_MotionUtils_AbsReal(HYD_REAL value) {
    return (value < 0.0) ? -value : value;
}

HYD_BOOL HYD_MotionUtils_IsFiniteReal(HYD_REAL value) {
    return isfinite(value) ? true : false;
}

HYD_BOOL HYD_MotionUtils_AxisRefIsValid(const HYD_AxisRef* axisRef) {
    if (axisRef == NULL) {
        return false;
    }

    return HYD_MotionUtils_IsFiniteReal(axisRef->position) &&
        HYD_MotionUtils_IsFiniteReal(axisRef->velocity) &&
        HYD_MotionUtils_IsFiniteReal(axisRef->flow) &&
        HYD_MotionUtils_IsFiniteReal(axisRef->pressure) &&
        HYD_MotionUtils_IsFiniteReal(axisRef->timestamp) &&
        (axisRef->pressure >= 0.0) &&
        (axisRef->timestamp >= 0.0);
}

const char* HYD_MotionUtils_CommandToString(HYD_FbCommand command) {
    switch (command) {
        case HYD_CMD_START:
            return "START";
        case HYD_CMD_NEXT:
            return "NEXT";
        case HYD_CMD_STOP:
            return "STOP";
        case HYD_CMD_HOLD:
            return "HOLD";
        case HYD_CMD_RESUME:
            return "RESUME";
        case HYD_CMD_ABORT:
            return "ABORT";
        case HYD_CMD_RESET:
            return "RESET";
        case HYD_CMD_ACK:
            return "ACK";
        case HYD_CMD_NONE:
        default:
            return "NONE";
    }
}

const char* HYD_MotionUtils_StateToString(HYD_FbState state) {
    switch (state) {
        case HYD_FB_STATE_DISABLED:
            return "DISABLED";
        case HYD_FB_STATE_IDLE:
            return "IDLE";
        case HYD_FB_STATE_READY:
            return "READY";
        case HYD_FB_STATE_STARTING:
            return "STARTING";
        case HYD_FB_STATE_RUNNING:
            return "RUNNING";
        case HYD_FB_STATE_SEGMENT_COMPLETE:
            return "SEGMENT_COMPLETE";
        case HYD_FB_STATE_HOLD:
            return "HOLD";
        case HYD_FB_STATE_DONE:
            return "DONE";
        case HYD_FB_STATE_ABORTED:
            return "ABORTED";
        case HYD_FB_STATE_FAULT:
            return "FAULT";
        default:
            return "UNKNOWN";
    }
}

HYD_ConfigInfo HYD_GetConfigInfo(void) {
    HYD_ConfigInfo config;
    memset(&config, 0, sizeof(config));

    config.maxSegments = HYD_MAX_SEGMENTS;
    config.maxSegmentTagValue = HYD_SEGMENT_TAG_MAX;
    config.maxNameLength = HYD_SEGMENT_TAG_MAX;
    config.maxHistoryDepth = 1;  /* Single-snapshot model; HYD_DIAG_HISTORY_DEPTH is deprecated */
    config.diagnosticHistoryEnabled = HYD_ENABLE_DIAGNOSTIC_HISTORY;
    config.pressureLoopTelemetryEnabled = HYD_ENABLE_PRESSURE_LOOP_TELEMETRY;
    config.executionReferenceEnabled = HYD_ENABLE_EXECUTION_REFERENCE;
    config.versionString = HYD_VERSION_STRING;
    config.buildTime = HYD_VERSION_BUILD_TIME;

    return config;
}
