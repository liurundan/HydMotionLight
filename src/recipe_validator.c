#include "recipe_validator.h"
#include "rbf_pid.h"
#include "segment_limits.h"
#include "pump_converter.h"

static HYD_BOOL HYD_IsValidPlannerType(HYD_PlannerType planner) {
    return (planner == HYD_PLANNER_POSITION_BASED) ||
           (planner == HYD_PLANNER_TIME_BASED);
}

static HYD_BOOL HYD_IsValidControlMode(HYD_ControlMode mode) {
    return (mode == HYD_MODE_POSITION) ||
           (mode == HYD_MODE_SPEED_RAMP) ||
           (mode == HYD_MODE_PRESSURE_CLOSED_LOOP);
}

static HYD_BOOL HYD_IsValidEndCondition(HYD_EndConditionType endCondition) {
    return (endCondition == HYD_END_POSITION) ||
           (endCondition == HYD_END_TIME) ||
           (endCondition == HYD_END_PRESSURE) ||
           (endCondition == HYD_END_FLOW) ||
           (endCondition == HYD_END_MANUAL);
}

static HYD_BOOL HYD_IsValidMotionDirection(HYD_MotionDirection direction) {
    return (direction == HYD_DIRECTION_AUTO) ||
           (direction == HYD_DIRECTION_EXTEND) ||
           (direction == HYD_DIRECTION_RETRACT) ||
           (direction == HYD_DIRECTION_HOLD);
}

static HYD_BOOL HYD_IsLinearMotionDirection(HYD_MotionDirection direction) {
    return (direction == HYD_DIRECTION_EXTEND) ||
           (direction == HYD_DIRECTION_RETRACT);
}

static HYD_BOOL HYD_IsValidPressureControllerType(HYD_PressureControllerType strategy) {
    return (strategy == HYD_PRESSURE_CONTROLLER_NONE) ||
           (strategy == HYD_PRESSURE_CONTROLLER_P) ||
           (strategy == HYD_PRESSURE_CONTROLLER_PI) ||
           (strategy == HYD_PRESSURE_CONTROLLER_PID) ||
           (strategy == HYD_PRESSURE_CONTROLLER_RBF_PID);
}

static HYD_BOOL HYD_IsSupportedPressureControllerType(HYD_PressureControllerType strategy) {
    return (strategy == HYD_PRESSURE_CONTROLLER_NONE) ||
           (strategy == HYD_PRESSURE_CONTROLLER_P) ||
           (strategy == HYD_PRESSURE_CONTROLLER_PI) ||
           (strategy == HYD_PRESSURE_CONTROLLER_PID) ||
           (strategy == HYD_PRESSURE_CONTROLLER_RBF_PID);
}

static HYD_BOOL HYD_RecipeValidator_Fail(HYD_DiagnosticCode* code,
                                         HYD_DiagnosticCode failCode) {
    if (code != NULL) {
        *code = failCode;
    }

    return false;
}

static HYD_REAL HYD_RecipeValidator_ResolvePositiveOrDefault(HYD_REAL configuredValue,
                                                             HYD_REAL defaultValue) {
    if (configuredValue > 0.0) {
        return configuredValue;
    }
    return defaultValue;
}

static HYD_BOOL HYD_RecipeValidator_HasCustomRbfConfig(const HYD_MotionSegment* segment) {
    if (segment == NULL) {
        return false;
    }

    return (segment->pressureRbfConfig.minKp > 0.0) ||
           (segment->pressureRbfConfig.maxKp > 0.0) ||
           (segment->pressureRbfConfig.minKi > 0.0) ||
           (segment->pressureRbfConfig.maxKi > 0.0) ||
           (segment->pressureRbfConfig.minKd > 0.0) ||
           (segment->pressureRbfConfig.maxKd > 0.0) ||
           (segment->pressureRbfConfig.etaW > 0.0) ||
           (segment->pressureRbfConfig.etaC > 0.0) ||
           (segment->pressureRbfConfig.etaB > 0.0) ||
           (segment->pressureRbfConfig.etaP > 0.0) ||
           (segment->pressureRbfConfig.etaI > 0.0) ||
           (segment->pressureRbfConfig.etaD > 0.0);
}

HYD_BOOL HYD_RecipeValidator_ValidateSegment(const HYD_MotionSegment* segment,
                                            size_t segmentIndex,
                                            HYD_DiagnosticCode* code) {
    if (segment == NULL) {
        return HYD_RecipeValidator_Fail(code, HYD_DIAG_CODE_SEGMENT_INVALID);
    }

    if (!HYD_IsValidPlannerType(segment->planner)) {
        return HYD_RecipeValidator_Fail(code, HYD_DIAG_CODE_SEGMENT_INVALID);
    }

    if (!HYD_IsValidControlMode(segment->mode)) {
        return HYD_RecipeValidator_Fail(code, HYD_DIAG_CODE_SEGMENT_INVALID);
    }

    if (!HYD_IsValidEndCondition(segment->endCondition)) {
        return HYD_RecipeValidator_Fail(code, HYD_DIAG_CODE_SEGMENT_INVALID);
    }

    if (!HYD_IsValidMotionDirection(segment->direction)) {
        return HYD_RecipeValidator_Fail(code, HYD_DIAG_CODE_SEGMENT_INVALID);
    }

    if (!HYD_IsValidPressureControllerType(segment->pressureController)) {
        return HYD_RecipeValidator_Fail(code, HYD_DIAG_CODE_SEGMENT_INVALID);
    }

    if (!HYD_IsSupportedPressureControllerType(segment->pressureController)) {
        return HYD_RecipeValidator_Fail(code, HYD_DIAG_CODE_SEGMENT_INVALID);
    }

    if ((segment->mode == HYD_MODE_POSITION || segment->mode == HYD_MODE_SPEED_RAMP) &&
        !HYD_IsLinearMotionDirection(segment->direction)) {
        return HYD_RecipeValidator_Fail(code, HYD_DIAG_CODE_SEGMENT_INVALID);
    }

    if (segment->mode == HYD_MODE_SPEED_RAMP && segment->planner != HYD_PLANNER_TIME_BASED) {
        return HYD_RecipeValidator_Fail(code, HYD_DIAG_CODE_SEGMENT_INVALID);
    }

    if (segment->tolerance < 0.0) {
        return HYD_RecipeValidator_Fail(code, HYD_DIAG_CODE_SEGMENT_INVALID);
    }

    if (segment->positionTolerance < 0.0) {
        return HYD_RecipeValidator_Fail(code, HYD_DIAG_CODE_SEGMENT_INVALID);
    }

    if (segment->pressureTolerance < 0.0) {
        return HYD_RecipeValidator_Fail(code, HYD_DIAG_CODE_SEGMENT_INVALID);
    }

    if (segment->flowTolerance < 0.0) {
        return HYD_RecipeValidator_Fail(code, HYD_DIAG_CODE_SEGMENT_INVALID);
    }

    if (segment->velocityTolerance < 0.0) {
        return HYD_RecipeValidator_Fail(code, HYD_DIAG_CODE_SEGMENT_INVALID);
    }

    if (segment->timeoutLimit < 0.0) {
        return HYD_RecipeValidator_Fail(code, HYD_DIAG_CODE_SEGMENT_INVALID);
    }

    if (segment->maxAcceleration < 0.0) {
        return HYD_RecipeValidator_Fail(code, HYD_DIAG_CODE_SEGMENT_INVALID);
    }

    if (segment->maxDeceleration < 0.0) {
        return HYD_RecipeValidator_Fail(code, HYD_DIAG_CODE_SEGMENT_INVALID);
    }

    if (segment->maxVelocity < 0.0) {
        return HYD_RecipeValidator_Fail(code, HYD_DIAG_CODE_SEGMENT_INVALID);
    }

    if (segment->maxFlow <= 0.0) {
        return HYD_RecipeValidator_Fail(code, HYD_DIAG_CODE_SEGMENT_INVALID);
    }

    if (segment->pressureRampRate < 0.0) {
        return HYD_RecipeValidator_Fail(code, HYD_DIAG_CODE_SEGMENT_INVALID);
    }

    if (segment->targetFlow < 0.0) {
        return HYD_RecipeValidator_Fail(code, HYD_DIAG_CODE_SEGMENT_INVALID);
    }

    if (segment->pressureKp < 0.0) {
        return HYD_RecipeValidator_Fail(code, HYD_DIAG_CODE_SEGMENT_INVALID);
    }

    if (segment->pressureKi < 0.0) {
        return HYD_RecipeValidator_Fail(code, HYD_DIAG_CODE_SEGMENT_INVALID);
    }

    if (segment->pressureKd < 0.0) {
        return HYD_RecipeValidator_Fail(code, HYD_DIAG_CODE_SEGMENT_INVALID);
    }

    if (segment->pressureIntegralLimit < 0.0) {
        return HYD_RecipeValidator_Fail(code, HYD_DIAG_CODE_SEGMENT_INVALID);
    }

    if (segment->pressureDeadband < 0.0) {
        return HYD_RecipeValidator_Fail(code, HYD_DIAG_CODE_SEGMENT_INVALID);
    }

    if (segment->pressureFilterAlpha < 0.0 || segment->pressureFilterAlpha > 1.0) {
        return HYD_RecipeValidator_Fail(code, HYD_DIAG_CODE_SEGMENT_INVALID);
    }

    if (segment->pressureDerivativeFilterAlpha < 0.0 || segment->pressureDerivativeFilterAlpha > 1.0) {
        return HYD_RecipeValidator_Fail(code, HYD_DIAG_CODE_SEGMENT_INVALID);
    }

    if (segment->pressureRbfConfig.minKp < 0.0 ||
        segment->pressureRbfConfig.maxKp < 0.0 ||
        segment->pressureRbfConfig.minKi < 0.0 ||
        segment->pressureRbfConfig.maxKi < 0.0 ||
        segment->pressureRbfConfig.minKd < 0.0 ||
        segment->pressureRbfConfig.maxKd < 0.0 ||
        segment->pressureRbfConfig.etaW < 0.0 ||
        segment->pressureRbfConfig.etaC < 0.0 ||
        segment->pressureRbfConfig.etaB < 0.0 ||
        segment->pressureRbfConfig.etaP < 0.0 ||
        segment->pressureRbfConfig.etaI < 0.0 ||
        segment->pressureRbfConfig.etaD < 0.0) {
        return HYD_RecipeValidator_Fail(code, HYD_DIAG_CODE_SEGMENT_INVALID);
    }

    if ((segment->mode == HYD_MODE_PRESSURE_CLOSED_LOOP) &&
        (segment->pressureController == HYD_PRESSURE_CONTROLLER_RBF_PID ||
         HYD_RecipeValidator_HasCustomRbfConfig(segment))) {
        HYD_REAL resolvedMinKp;
        HYD_REAL resolvedMaxKp;
        HYD_REAL resolvedMinKi;
        HYD_REAL resolvedMaxKi;
        HYD_REAL resolvedMinKd;
        HYD_REAL resolvedMaxKd;

        resolvedMinKp = HYD_RecipeValidator_ResolvePositiveOrDefault(segment->pressureRbfConfig.minKp,
                                                                     (HYD_REAL)PID_MIN_KP);
        resolvedMaxKp = HYD_RecipeValidator_ResolvePositiveOrDefault(segment->pressureRbfConfig.maxKp,
                                                                     (HYD_REAL)PID_MAX_KP);
        resolvedMinKi = HYD_RecipeValidator_ResolvePositiveOrDefault(segment->pressureRbfConfig.minKi,
                                                                     (HYD_REAL)PID_MIN_KI);
        resolvedMaxKi = HYD_RecipeValidator_ResolvePositiveOrDefault(segment->pressureRbfConfig.maxKi,
                                                                     (HYD_REAL)PID_MAX_KI);
        resolvedMinKd = HYD_RecipeValidator_ResolvePositiveOrDefault(segment->pressureRbfConfig.minKd,
                                                                     (HYD_REAL)PID_MIN_KD);
        resolvedMaxKd = HYD_RecipeValidator_ResolvePositiveOrDefault(segment->pressureRbfConfig.maxKd,
                                                                     (HYD_REAL)PID_MAX_KD);

        if (resolvedMinKp > resolvedMaxKp ||
            resolvedMinKi > resolvedMaxKi ||
            resolvedMinKd > resolvedMaxKd) {
            return HYD_RecipeValidator_Fail(code, HYD_DIAG_CODE_SEGMENT_INVALID);
        }
    }

    if ((segment->mode == HYD_MODE_PRESSURE_CLOSED_LOOP) &&
        (segment->pressureController == HYD_PRESSURE_CONTROLLER_PI ||
         segment->pressureController == HYD_PRESSURE_CONTROLLER_PID) &&
        (segment->pressureKi <= 0.0)) {
        return HYD_RecipeValidator_Fail(code, HYD_DIAG_CODE_SEGMENT_INVALID);
    }

    if ((segment->mode == HYD_MODE_PRESSURE_CLOSED_LOOP) &&
        (segment->pressureController == HYD_PRESSURE_CONTROLLER_PID) &&
        (segment->pressureKd <= 0.0)) {
        return HYD_RecipeValidator_Fail(code, HYD_DIAG_CODE_SEGMENT_INVALID);
    }

    if ((segment->mode == HYD_MODE_PRESSURE_CLOSED_LOOP) &&
        (segment->pressureController == HYD_PRESSURE_CONTROLLER_P ||
         segment->pressureController == HYD_PRESSURE_CONTROLLER_PI ||
         segment->pressureController == HYD_PRESSURE_CONTROLLER_PID) &&
        (segment->pressureKp <= 0.0)) {
        return HYD_RecipeValidator_Fail(code, HYD_DIAG_CODE_SEGMENT_INVALID);
    }

    if ((segment->mode != HYD_MODE_PRESSURE_CLOSED_LOOP) &&
        (segment->velocityToFlowGain <= 0.0)) {
        return HYD_RecipeValidator_Fail(code, HYD_DIAG_CODE_SEGMENT_INVALID);
    }

    if ((segment->planner == HYD_PLANNER_TIME_BASED) &&
        (segment->mode != HYD_MODE_PRESSURE_CLOSED_LOOP) &&
        (segment->maxVelocity <= 0.0)) {
        return HYD_RecipeValidator_Fail(code, HYD_DIAG_CODE_SEGMENT_INVALID);
    }

    if ((segment->endCondition == HYD_END_TIME) && (segment->duration <= 0.0)) {
        return HYD_RecipeValidator_Fail(code, HYD_DIAG_CODE_SEGMENT_INVALID);
    }

    if (code != NULL) {
        *code = HYD_DIAG_CODE_NONE;
    }
    return true;
}

HYD_BOOL HYD_RecipeValidator_ValidateRecipe(const HYD_MotionSegment* recipe,
                                           size_t recipeSize,
                                           HYD_DiagnosticCode* code) {
    size_t index;

    if (recipe == NULL) {
        return HYD_RecipeValidator_Fail(code, HYD_DIAG_CODE_RECIPE_EMPTY);
    }

    if (recipeSize == 0U) {
        return HYD_RecipeValidator_Fail(code, HYD_DIAG_CODE_RECIPE_EMPTY);
    }

    if (recipeSize > HYD_MAX_SEGMENTS) {
        return HYD_RecipeValidator_Fail(code, HYD_DIAG_CODE_RECIPE_TOO_LARGE);
    }

    for (index = 0U; index < recipeSize; ++index) {
        if (!HYD_RecipeValidator_ValidateSegment(&recipe[index], index, code)) {
            return false;
        }
    }

    if (code != NULL) {
        *code = HYD_DIAG_CODE_NONE;
    }
    return true;
}

HYD_BOOL HYD_RecipeValidator_ValidateRuntimeConfig(HYD_REAL flowToPumpSpeedGain,
                                                  HYD_REAL pumpSpeedLimit,
                                                  HYD_DiagnosticCode* code) {
    /* Delegate pump-related runtime config checks to the PumpConverter to
     * centralize validation logic and avoid duplication.
     */
    return HYD_PumpConverter_ValidateConfig(flowToPumpSpeedGain, pumpSpeedLimit, code);
}

HYD_BOOL HYD_RecipeValidator_ValidateStartContext(const HYD_MotionSegment* segment,
                                                 size_t segmentIndex,
                                                 const HYD_AxisRef* axisRef,
                                                 HYD_DiagnosticCode* code) {
    HYD_REAL positionTolerance;

    if (segment == NULL || axisRef == NULL) {
        return HYD_RecipeValidator_Fail(code, HYD_DIAG_CODE_START_CONTEXT_INVALID);
    }

    positionTolerance = HYD_Segment_GetPositionTolerance(segment);

    if ((segment->mode == HYD_MODE_POSITION || segment->endCondition == HYD_END_POSITION) &&
        (segment->direction == HYD_DIRECTION_EXTEND) &&
        (segment->targetPosition + positionTolerance < axisRef->position)) {
        return HYD_RecipeValidator_Fail(code, HYD_DIAG_CODE_START_CONTEXT_INVALID);
    }

    if ((segment->mode == HYD_MODE_POSITION || segment->endCondition == HYD_END_POSITION) &&
        (segment->direction == HYD_DIRECTION_RETRACT) &&
        (segment->targetPosition - positionTolerance > axisRef->position)) {
        return HYD_RecipeValidator_Fail(code, HYD_DIAG_CODE_START_CONTEXT_INVALID);
    }

    if (code != NULL) {
        *code = HYD_DIAG_CODE_NONE;
    }
    return true;
}
