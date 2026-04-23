#include "motion_utils.h"
#include "common_types.h"
#include "hdy_config.h"
#include <math.h>
#include <string.h>

HDY_REAL HDY_MotionUtils_MinReal(HDY_REAL left, HDY_REAL right) {
    return (left < right) ? left : right;
}

HDY_REAL HDY_MotionUtils_AbsReal(HDY_REAL value) {
    return (value < 0.0) ? -value : value;
}

HDY_BOOL HDY_MotionUtils_IsFiniteReal(HDY_REAL value) {
    return isfinite(value) ? true : false;
}

HDY_BOOL HDY_MotionUtils_AxisRefIsValid(const HDY_AxisRef* axisRef) {
    if (axisRef == NULL) {
        return false;
    }

    return HDY_MotionUtils_IsFiniteReal(axisRef->position) &&
        HDY_MotionUtils_IsFiniteReal(axisRef->velocity) &&
        HDY_MotionUtils_IsFiniteReal(axisRef->flow) &&
        HDY_MotionUtils_IsFiniteReal(axisRef->pressure) &&
        HDY_MotionUtils_IsFiniteReal(axisRef->timestamp) &&
        (axisRef->pressure >= 0.0) &&
        (axisRef->timestamp >= 0.0);
}

const char* HDY_MotionUtils_CommandToString(HDY_FbCommand command) {
    switch (command) {
        case HDY_CMD_START:
            return "START";
        case HDY_CMD_NEXT:
            return "NEXT";
        case HDY_CMD_STOP:
            return "STOP";
        case HDY_CMD_HOLD:
            return "HOLD";
        case HDY_CMD_RESUME:
            return "RESUME";
        case HDY_CMD_ABORT:
            return "ABORT";
        case HDY_CMD_RESET:
            return "RESET";
        case HDY_CMD_ACK:
            return "ACK";
        case HDY_CMD_NONE:
        default:
            return "NONE";
    }
}

const char* HDY_MotionUtils_StateToString(HDY_FbState state) {
    switch (state) {
        case HDY_FB_STATE_DISABLED:
            return "DISABLED";
        case HDY_FB_STATE_IDLE:
            return "IDLE";
        case HDY_FB_STATE_READY:
            return "READY";
        case HDY_FB_STATE_STARTING:
            return "STARTING";
        case HDY_FB_STATE_RUNNING:
            return "RUNNING";
        case HDY_FB_STATE_SEGMENT_COMPLETE:
            return "SEGMENT_COMPLETE";
        case HDY_FB_STATE_HOLD:
            return "HOLD";
        case HDY_FB_STATE_DONE:
            return "DONE";
        case HDY_FB_STATE_ABORTED:
            return "ABORTED";
        case HDY_FB_STATE_FAULT:
            return "FAULT";
        default:
            return "UNKNOWN";
    }
}

HDY_ConfigInfo HDY_GetConfigInfo(void) {
    HDY_ConfigInfo config;
    memset(&config, 0, sizeof(config));

    config.maxSegments = HDY_MAX_SEGMENTS;
    config.maxSegmentTagValue = HDY_SEGMENT_TAG_MAX;
    config.maxNameLength = HDY_SEGMENT_TAG_MAX;
    config.maxHistoryDepth = 1;  /* Single-snapshot model; HDY_DIAG_HISTORY_DEPTH is deprecated */
    config.diagnosticHistoryEnabled = HDY_ENABLE_DIAGNOSTIC_HISTORY;
    config.pressureLoopTelemetryEnabled = HDY_ENABLE_PRESSURE_LOOP_TELEMETRY;
    config.executionReferenceEnabled = HDY_ENABLE_EXECUTION_REFERENCE;
    config.versionString = HDY_VERSION_STRING;
    config.buildTime = HDY_VERSION_BUILD_TIME;

    return config;
}
