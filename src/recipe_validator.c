#include "recipe_validator.h"
#include "segment_limits.h"
#include <stdarg.h>
#include <stdio.h>

static HDY_BOOL HDY_IsValidPlannerType(HDY_PlannerType planner) {
    return (planner == HDY_PLANNER_POSITION_BASED) ||
           (planner == HDY_PLANNER_TIME_BASED);
}

static HDY_BOOL HDY_IsValidControlMode(HDY_ControlMode mode) {
    return (mode == HDY_MODE_POSITION) ||
           (mode == HDY_MODE_SPEED_RAMP) ||
           (mode == HDY_MODE_PRESSURE_CLOSED_LOOP);
}

static HDY_BOOL HDY_IsValidEndCondition(HDY_EndConditionType endCondition) {
    return (endCondition == HDY_END_POSITION) ||
           (endCondition == HDY_END_TIME) ||
           (endCondition == HDY_END_PRESSURE) ||
           (endCondition == HDY_END_FLOW) ||
           (endCondition == HDY_END_MANUAL);
}

static HDY_BOOL HDY_IsValidMotionDirection(HDY_MotionDirection direction) {
    return (direction == HDY_DIRECTION_AUTO) ||
           (direction == HDY_DIRECTION_EXTEND) ||
           (direction == HDY_DIRECTION_RETRACT) ||
           (direction == HDY_DIRECTION_HOLD);
}

static HDY_BOOL HDY_IsLinearMotionDirection(HDY_MotionDirection direction) {
    return (direction == HDY_DIRECTION_EXTEND) ||
           (direction == HDY_DIRECTION_RETRACT);
}

static HDY_BOOL HDY_RecipeValidator_Fail(HDY_DiagnosticCode* code,
                                         HDY_DiagnosticCode failCode,
                                         char* message,
                                         size_t messageSize,
                                         const char* format,
                                         ...) {
    va_list args;

    if (code != NULL) {
        *code = failCode;
    }

    if (message != NULL && messageSize > 0U) {
        va_start(args, format);
        vsnprintf(message, messageSize, format, args);
        va_end(args);
    }

    return false;
}

HDY_BOOL HDY_RecipeValidator_ValidateSegment(const HDY_MotionSegment* segment,
                                            size_t segmentIndex,
                                            HDY_DiagnosticCode* code,
                                            char* message,
                                            size_t messageSize) {
    if (segment == NULL) {
        return HDY_RecipeValidator_Fail(code,
                                        HDY_DIAG_CODE_SEGMENT_INVALID,
                                        message,
                                        messageSize,
                                        "Segment %zu is NULL",
                                        segmentIndex);
    }

    if (!HDY_IsValidPlannerType(segment->planner)) {
        return HDY_RecipeValidator_Fail(code,
                                        HDY_DIAG_CODE_SEGMENT_INVALID,
                                        message,
                                        messageSize,
                                        "Segment %zu planner is invalid",
                                        segmentIndex);
    }

    if (!HDY_IsValidControlMode(segment->mode)) {
        return HDY_RecipeValidator_Fail(code,
                                        HDY_DIAG_CODE_SEGMENT_INVALID,
                                        message,
                                        messageSize,
                                        "Segment %zu mode is invalid",
                                        segmentIndex);
    }

    if (!HDY_IsValidEndCondition(segment->endCondition)) {
        return HDY_RecipeValidator_Fail(code,
                                        HDY_DIAG_CODE_SEGMENT_INVALID,
                                        message,
                                        messageSize,
                                        "Segment %zu end condition is invalid",
                                        segmentIndex);
    }

    if (!HDY_IsValidMotionDirection(segment->direction)) {
        return HDY_RecipeValidator_Fail(code,
                                        HDY_DIAG_CODE_SEGMENT_INVALID,
                                        message,
                                        messageSize,
                                        "Segment %zu direction is invalid",
                                        segmentIndex);
    }

    if ((segment->mode == HDY_MODE_POSITION || segment->mode == HDY_MODE_SPEED_RAMP) &&
        !HDY_IsLinearMotionDirection(segment->direction)) {
        return HDY_RecipeValidator_Fail(code,
                                        HDY_DIAG_CODE_SEGMENT_INVALID,
                                        message,
                                        messageSize,
                                        "Segment %zu direction must be EXTEND or RETRACT for motion mode",
                                        segmentIndex);
    }

    if (segment->mode == HDY_MODE_SPEED_RAMP && segment->planner != HDY_PLANNER_TIME_BASED) {
        return HDY_RecipeValidator_Fail(code,
                                        HDY_DIAG_CODE_SEGMENT_INVALID,
                                        message,
                                        messageSize,
                                        "Segment %zu speed-ramp mode requires TIME_BASED planner",
                                        segmentIndex);
    }

    if (segment->tolerance < 0.0) {
        return HDY_RecipeValidator_Fail(code,
                                        HDY_DIAG_CODE_SEGMENT_INVALID,
                                        message,
                                        messageSize,
                                        "Segment %zu legacy tolerance must be >= 0",
                                        segmentIndex);
    }

    if (segment->positionTolerance < 0.0) {
        return HDY_RecipeValidator_Fail(code,
                                        HDY_DIAG_CODE_SEGMENT_INVALID,
                                        message,
                                        messageSize,
                                        "Segment %zu positionTolerance must be >= 0",
                                        segmentIndex);
    }

    if (segment->pressureTolerance < 0.0) {
        return HDY_RecipeValidator_Fail(code,
                                        HDY_DIAG_CODE_SEGMENT_INVALID,
                                        message,
                                        messageSize,
                                        "Segment %zu pressureTolerance must be >= 0",
                                        segmentIndex);
    }

    if (segment->flowTolerance < 0.0) {
        return HDY_RecipeValidator_Fail(code,
                                        HDY_DIAG_CODE_SEGMENT_INVALID,
                                        message,
                                        messageSize,
                                        "Segment %zu flowTolerance must be >= 0",
                                        segmentIndex);
    }

    if (segment->velocityTolerance < 0.0) {
        return HDY_RecipeValidator_Fail(code,
                                        HDY_DIAG_CODE_SEGMENT_INVALID,
                                        message,
                                        messageSize,
                                        "Segment %zu velocityTolerance must be >= 0",
                                        segmentIndex);
    }

    if (segment->timeoutLimit < 0.0) {
        return HDY_RecipeValidator_Fail(code,
                                        HDY_DIAG_CODE_SEGMENT_INVALID,
                                        message,
                                        messageSize,
                                        "Segment %zu timeoutLimit must be >= 0",
                                        segmentIndex);
    }

    if (segment->maxAcceleration < 0.0) {
        return HDY_RecipeValidator_Fail(code,
                                        HDY_DIAG_CODE_SEGMENT_INVALID,
                                        message,
                                        messageSize,
                                        "Segment %zu maxAcceleration must be >= 0",
                                        segmentIndex);
    }

    if (segment->maxVelocity < 0.0) {
        return HDY_RecipeValidator_Fail(code,
                                        HDY_DIAG_CODE_SEGMENT_INVALID,
                                        message,
                                        messageSize,
                                        "Segment %zu maxVelocity must be >= 0",
                                        segmentIndex);
    }

    if (segment->maxFlow <= 0.0) {
        return HDY_RecipeValidator_Fail(code,
                                        HDY_DIAG_CODE_SEGMENT_INVALID,
                                        message,
                                        messageSize,
                                        "Segment %zu maxFlow must be > 0",
                                        segmentIndex);
    }

    if (segment->pressureRampRate < 0.0) {
        return HDY_RecipeValidator_Fail(code,
                                        HDY_DIAG_CODE_SEGMENT_INVALID,
                                        message,
                                        messageSize,
                                        "Segment %zu pressureRampRate must be >= 0",
                                        segmentIndex);
    }

    if (segment->targetFlow < 0.0) {
        return HDY_RecipeValidator_Fail(code,
                                        HDY_DIAG_CODE_SEGMENT_INVALID,
                                        message,
                                        messageSize,
                                        "Segment %zu targetFlow must be >= 0",
                                        segmentIndex);
    }

    if ((segment->mode != HDY_MODE_PRESSURE_CLOSED_LOOP) &&
        (segment->velocityToFlowGain <= 0.0)) {
        return HDY_RecipeValidator_Fail(code,
                                        HDY_DIAG_CODE_SEGMENT_INVALID,
                                        message,
                                        messageSize,
                                        "Segment %zu velocityToFlowGain must be > 0",
                                        segmentIndex);
    }

    if ((segment->planner == HDY_PLANNER_TIME_BASED) &&
        (segment->mode != HDY_MODE_PRESSURE_CLOSED_LOOP) &&
        (segment->maxVelocity <= 0.0)) {
        return HDY_RecipeValidator_Fail(code,
                                        HDY_DIAG_CODE_SEGMENT_INVALID,
                                        message,
                                        messageSize,
                                        "Segment %zu maxVelocity must be > 0 for time planning",
                                        segmentIndex);
    }

    if ((segment->endCondition == HDY_END_TIME) && (segment->duration <= 0.0)) {
        return HDY_RecipeValidator_Fail(code,
                                        HDY_DIAG_CODE_SEGMENT_INVALID,
                                        message,
                                        messageSize,
                                        "Segment %zu duration must be > 0 for time end",
                                        segmentIndex);
    }

    if (code != NULL) {
        *code = HDY_DIAG_CODE_NONE;
    }
    if (message != NULL && messageSize > 0U) {
        message[0] = '\0';
    }
    return true;
}

HDY_BOOL HDY_RecipeValidator_ValidateRecipe(const HDY_MotionSegment* recipe,
                                           size_t recipeSize,
                                           HDY_DiagnosticCode* code,
                                           char* message,
                                           size_t messageSize) {
    size_t index;

    if (recipe == NULL) {
        return HDY_RecipeValidator_Fail(code,
                                        HDY_DIAG_CODE_RECIPE_EMPTY,
                                        message,
                                        messageSize,
                                        "Recipe pointer is NULL");
    }

    if (recipeSize == 0U) {
        return HDY_RecipeValidator_Fail(code,
                                        HDY_DIAG_CODE_RECIPE_EMPTY,
                                        message,
                                        messageSize,
                                        "Recipe is empty");
    }

    if (recipeSize > HDY_MAX_SEGMENTS) {
        return HDY_RecipeValidator_Fail(code,
                                        HDY_DIAG_CODE_RECIPE_TOO_LARGE,
                                        message,
                                        messageSize,
                                        "Recipe size exceeds HDY_MAX_SEGMENTS");
    }

    for (index = 0U; index < recipeSize; ++index) {
        if (!HDY_RecipeValidator_ValidateSegment(&recipe[index], index, code, message, messageSize)) {
            return false;
        }
    }

    if (code != NULL) {
        *code = HDY_DIAG_CODE_NONE;
    }
    if (message != NULL && messageSize > 0U) {
        message[0] = '\0';
    }
    return true;
}

HDY_BOOL HDY_RecipeValidator_ValidateRuntimeConfig(HDY_REAL flowToPumpSpeedGain,
                                                  HDY_REAL pumpSpeedLimit,
                                                  HDY_DiagnosticCode* code,
                                                  char* message,
                                                  size_t messageSize) {
    if (flowToPumpSpeedGain <= 0.0) {
        return HDY_RecipeValidator_Fail(code,
                                        HDY_DIAG_CODE_RUNTIME_CONFIG_INVALID,
                                        message,
                                        messageSize,
                                        "FLOW_TO_PUMP_SPEED_GAIN must be > 0");
    }

    if (pumpSpeedLimit < 0.0) {
        return HDY_RecipeValidator_Fail(code,
                                        HDY_DIAG_CODE_RUNTIME_CONFIG_INVALID,
                                        message,
                                        messageSize,
                                        "PUMP_SPEED_LIMIT must be >= 0");
    }

    if (code != NULL) {
        *code = HDY_DIAG_CODE_NONE;
    }
    if (message != NULL && messageSize > 0U) {
        message[0] = '\0';
    }
    return true;
}

HDY_BOOL HDY_RecipeValidator_ValidateStartContext(const HDY_MotionSegment* segment,
                                                 size_t segmentIndex,
                                                 const HDY_AxisRef* axisRef,
                                                 HDY_DiagnosticCode* code,
                                                 char* message,
                                                 size_t messageSize) {
    HDY_REAL positionTolerance;

    if (segment == NULL || axisRef == NULL) {
        return HDY_RecipeValidator_Fail(code,
                                        HDY_DIAG_CODE_START_CONTEXT_INVALID,
                                        message,
                                        messageSize,
                                        "Start context is invalid for segment %zu",
                                        segmentIndex);
    }

    positionTolerance = HDY_Segment_GetPositionTolerance(segment);

    if ((segment->mode == HDY_MODE_POSITION || segment->endCondition == HDY_END_POSITION) &&
        (segment->direction == HDY_DIRECTION_EXTEND) &&
        (segment->targetPosition + positionTolerance < axisRef->position)) {
        return HDY_RecipeValidator_Fail(code,
                                        HDY_DIAG_CODE_START_CONTEXT_INVALID,
                                        message,
                                        messageSize,
                                        "Segment %zu direction conflicts with current position and targetPosition",
                                        segmentIndex);
    }

    if ((segment->mode == HDY_MODE_POSITION || segment->endCondition == HDY_END_POSITION) &&
        (segment->direction == HDY_DIRECTION_RETRACT) &&
        (segment->targetPosition - positionTolerance > axisRef->position)) {
        return HDY_RecipeValidator_Fail(code,
                                        HDY_DIAG_CODE_START_CONTEXT_INVALID,
                                        message,
                                        messageSize,
                                        "Segment %zu direction conflicts with current position and targetPosition",
                                        segmentIndex);
    }

    if (code != NULL) {
        *code = HDY_DIAG_CODE_NONE;
    }
    if (message != NULL && messageSize > 0U) {
        message[0] = '\0';
    }
    return true;
}
