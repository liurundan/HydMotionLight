#include "actuation_mapper.h"

#include <math.h>

static void set_error(HYD_ToggleError *error, HYD_ToggleError value)
{
    if (error != NULL) {
        *error = value;
    }
}

static HYD_MotionDirection direction_from_velocity(HYD_REAL velocity)
{
    if (velocity > 0.0f) {
        return HYD_DIRECTION_EXTEND;
    }
    if (velocity < 0.0f) {
        return HYD_DIRECTION_RETRACT;
    }
    return HYD_DIRECTION_HOLD;
}

static HYD_BOOL resolve_position_and_ratio(
    const HYD_ActuationMapperInput *input,
    HYD_REAL template_velocity,
    HYD_REAL *actuator_position,
    HYD_REAL *velocity_ratio,
    HYD_REAL *actuator_velocity,
    HYD_ToggleError *error)
{
    HYD_ToggleSolution solution;

    if (input->mechanismType == HYD_MECHANISM_DIRECT) {
        *actuator_position = input->templatePosition;
        *velocity_ratio = 1.0f;
        *actuator_velocity = template_velocity;
        return 1;
    }

    if (input->mechanismType != HYD_MECHANISM_FIVE_POINT_TOGGLE) {
        set_error(error, HYD_TOGGLE_ERROR_INVALID_BRANCH);
        return 0;
    }
    if (input->toggleConfig == NULL) {
        set_error(error, HYD_TOGGLE_ERROR_NULL_ARGUMENT);
        return 0;
    }
    if (!HYD_ToggleKinematics_SolveOnline(input->toggleConfig,
                                          input->templatePosition,
                                          template_velocity,
                                          &solution,
                                          error)) {
        return 0;
    }

    *actuator_position = solution.xs;
    *velocity_ratio = solution.velocityRatio;
    *actuator_velocity = solution.vs;
    return 1;
}

static HYD_BOOL resolve_cylinder_gain(
    const HYD_ActuationMapperInput *input,
    HYD_MotionDirection direction,
    HYD_REAL *gain,
    HYD_ToggleError *error)
{
    HYD_REAL area = 0.0f;
    HYD_REAL resolved;

    if (input->cylinderConfig != NULL) {
        area = (direction == HYD_DIRECTION_RETRACT)
                   ? input->cylinderConfig->areaRetractMm2
                   : input->cylinderConfig->areaExtendMm2;
        if (!isfinite(area)) {
            set_error(error, HYD_TOGGLE_ERROR_NONFINITE_PARAMETER);
            return 0;
        }
    }

    resolved = HYD_CylinderConfig_GetVelocityToFlowGain(
        input->cylinderConfig, direction);
    if (resolved <= 0.0f) {
        resolved = input->fallbackCylinderVelocityToFlowGain;
    }
    if (!isfinite(resolved)) {
        set_error(error, HYD_TOGGLE_ERROR_NONFINITE_PARAMETER);
        return 0;
    }
    if (resolved <= 0.0f) {
        set_error(error, HYD_TOGGLE_ERROR_VELOCITY_RATIO_UNSAFE);
        return 0;
    }

    *gain = resolved;
    return 1;
}

static HYD_BOOL ratio_is_safe(HYD_REAL velocity_ratio,
                              HYD_ToggleError *error)
{
    HYD_ToggleValidationLimits limits =
        HYD_ToggleKinematics_DefaultValidationLimits();

    if (!isfinite(velocity_ratio)) {
        set_error(error, HYD_TOGGLE_ERROR_NONFINITE_RESULT);
        return 0;
    }
    if (fabsf(velocity_ratio) <= limits.minAbsVelocityRatio) {
        set_error(error, HYD_TOGGLE_ERROR_VELOCITY_RATIO_UNSAFE);
        return 0;
    }
    return 1;
}

static HYD_BOOL map_input_values_are_finite(
    const HYD_ActuationMapperInput *input,
    HYD_ToggleError *error)
{
    if (!isfinite(input->templatePosition) ||
        !isfinite(input->templateVelocity) ||
        !isfinite(input->maxFlow)) {
        set_error(error, HYD_TOGGLE_ERROR_NONFINITE_PARAMETER);
        return 0;
    }
    return 1;
}

static HYD_BOOL reverse_input_values_are_finite(
    const HYD_ActuationMapperInput *input,
    HYD_REAL actuator_flow,
    HYD_ToggleError *error)
{
    if (!isfinite(input->templatePosition) || !isfinite(actuator_flow)) {
        set_error(error, HYD_TOGGLE_ERROR_NONFINITE_PARAMETER);
        return 0;
    }
    return 1;
}

HYD_BOOL HYD_ActuationMapper_MapVelocity(
    const HYD_ActuationMapperInput *input,
    HYD_ActuationMapperOutput *output,
    HYD_ToggleError *error)
{
    HYD_ActuationMapperOutput next;

    if ((input == NULL) || (output == NULL)) {
        set_error(error, HYD_TOGGLE_ERROR_NULL_ARGUMENT);
        return 0;
    }
    if (!map_input_values_are_finite(input, error)) {
        return 0;
    }
    if (!resolve_position_and_ratio(input, input->templateVelocity,
                                    &next.actuatorPosition,
                                    &next.velocityRatio,
                                    &next.actuatorVelocity, error) ||
        !ratio_is_safe(next.velocityRatio, error)) {
        return 0;
    }

    next.actuatorDirection = direction_from_velocity(next.actuatorVelocity);
    if (next.actuatorDirection == HYD_DIRECTION_HOLD) {
        next.effectiveCylinderGain = 0.0f;
        next.unlimitedRequestedFlow = 0.0f;
        next.requestedFlow = 0.0f;
        next.maxTemplateVelocity = 0.0f;
        next.flowLimitActive = false;
        *output = next;
        set_error(error, HYD_TOGGLE_ERROR_NONE);
        return 1;
    }
    if (!resolve_cylinder_gain(input, next.actuatorDirection,
                               &next.effectiveCylinderGain, error)) {
        return 0;
    }

    next.unlimitedRequestedFlow = fabsf(next.actuatorVelocity) *
                                  next.effectiveCylinderGain;
    if (!isfinite(next.unlimitedRequestedFlow)) {
        set_error(error, HYD_TOGGLE_ERROR_NONFINITE_RESULT);
        return 0;
    }
    next.requestedFlow = next.unlimitedRequestedFlow;
    next.maxTemplateVelocity = 0.0f;
    next.flowLimitActive = false;
    if ((input->maxFlow > 0.0f) &&
        (next.requestedFlow > input->maxFlow)) {
        next.requestedFlow = input->maxFlow;
        next.maxTemplateVelocity = input->maxFlow /
            (next.effectiveCylinderGain * fabsf(next.velocityRatio));
        next.flowLimitActive = true;
    }

    *output = next;
    set_error(error, HYD_TOGGLE_ERROR_NONE);
    return 1;
}

HYD_BOOL HYD_ActuationMapper_FlowToTemplateVelocity(
    const HYD_ActuationMapperInput *input,
    HYD_REAL actuatorFlow,
    HYD_REAL *templateVelocity,
    HYD_ToggleError *error)
{
    HYD_REAL actuator_position;
    HYD_REAL velocity_ratio;
    HYD_REAL ignored_velocity;
    HYD_REAL effective_gain;
    HYD_REAL dynamic_gain;
    HYD_REAL template_sign;
    HYD_REAL actuator_sign;
    HYD_REAL next;
    HYD_MotionDirection direction;

    if ((input == NULL) || (templateVelocity == NULL)) {
        set_error(error, HYD_TOGGLE_ERROR_NULL_ARGUMENT);
        return 0;
    }
    if (!reverse_input_values_are_finite(input, actuatorFlow, error)) {
        return 0;
    }
    if (!resolve_position_and_ratio(input, 0.0f, &actuator_position,
                                    &velocity_ratio, &ignored_velocity,
                                    error) ||
        !ratio_is_safe(velocity_ratio, error)) {
        return 0;
    }

    if (actuatorFlow == 0.0f) {
        *templateVelocity = 0.0f;
        set_error(error, HYD_TOGGLE_ERROR_NONE);
        return 1;
    }

    template_sign = (actuatorFlow > 0.0f) ? 1.0f :
                    (actuatorFlow < 0.0f) ? -1.0f : 0.0f;
    actuator_sign = template_sign *
                    ((velocity_ratio < 0.0f) ? -1.0f : 1.0f);
    direction = direction_from_velocity(actuator_sign);
    if (!resolve_cylinder_gain(input, direction, &effective_gain, error)) {
        return 0;
    }

    dynamic_gain = effective_gain * fabsf(velocity_ratio);
    if (!isfinite(dynamic_gain) || (dynamic_gain <= 0.0f)) {
        set_error(error, HYD_TOGGLE_ERROR_VELOCITY_RATIO_UNSAFE);
        return 0;
    }

    next = template_sign * fabsf(actuatorFlow) / dynamic_gain;
    if (!isfinite(next)) {
        set_error(error, HYD_TOGGLE_ERROR_NONFINITE_RESULT);
        return 0;
    }

    *templateVelocity = next;
    set_error(error, HYD_TOGGLE_ERROR_NONE);
    return 1;
}

static HYD_BOOL range_max_abs_ratio(const HYD_ActuationMapperInput *input,
                                    HYD_REAL low,
                                    HYD_REAL high,
                                    HYD_REAL *ratio_max,
                                    HYD_REAL *ratio_sign,
                                    HYD_ToggleError *error)
{
    HYD_UINT16 point;
    HYD_UINT16 last = (HYD_UINT16)(HYD_TOGGLE_RANGE_RATIO_SAMPLES + 1u);

    *ratio_max = 0.0f;
    *ratio_sign = 1.0f;
    if (input->mechanismType == HYD_MECHANISM_DIRECT) {
        *ratio_max = 1.0f;
        return 1;
    }
    for (point = 0u; point <= last; ++point) {
        HYD_REAL x = (point == last)
            ? high
            : low + (high - low) * (HYD_REAL)point / (HYD_REAL)last;
        HYD_ActuationMapperInput at = *input;
        HYD_REAL position;
        HYD_REAL ratio;
        HYD_REAL ignored;

        at.templatePosition = x;
        if (!resolve_position_and_ratio(&at, 0.0f, &position, &ratio,
                                        &ignored, error) ||
            !ratio_is_safe(ratio, error)) {
            return 0;
        }
        if (fabsf(ratio) > *ratio_max) {
            *ratio_max = fabsf(ratio);
            *ratio_sign = (ratio < 0.0f) ? -1.0f : 1.0f;
        }
    }
    return 1;
}

static HYD_BOOL velocity_for_direction(const HYD_ActuationMapperInput *input,
                                       HYD_REAL template_sign,
                                       HYD_REAL ratio_max,
                                       HYD_REAL ratio_sign,
                                       HYD_REAL *velocity,
                                       HYD_ToggleError *error)
{
    HYD_REAL gain;
    HYD_MotionDirection actuator_direction =
        direction_from_velocity(template_sign * ratio_sign);

    if (!resolve_cylinder_gain(input, actuator_direction, &gain, error)) {
        return 0;
    }
    *velocity = input->maxFlow / (gain * ratio_max);
    if (!isfinite(*velocity)) {
        set_error(error, HYD_TOGGLE_ERROR_NONFINITE_RESULT);
        return 0;
    }
    return 1;
}

HYD_BOOL HYD_ActuationMapper_MaxTemplateVelocityOverRange(
    const HYD_ActuationMapperInput *input,
    HYD_REAL xFrom,
    HYD_REAL xTo,
    HYD_MotionDirection direction,
    HYD_REAL *maxTemplateVelocity,
    HYD_REAL *velocityRatioMax,
    HYD_ToggleError *error)
{
    HYD_REAL low;
    HYD_REAL high;
    HYD_REAL ratio_max;
    HYD_REAL ratio_sign;
    HYD_REAL velocity;

    if ((input == NULL) || (maxTemplateVelocity == NULL) ||
        (velocityRatioMax == NULL)) {
        set_error(error, HYD_TOGGLE_ERROR_NULL_ARGUMENT);
        return 0;
    }
    if (!isfinite(xFrom) || !isfinite(xTo) || !isfinite(input->maxFlow) ||
        (input->maxFlow <= 0.0f)) {
        set_error(error, HYD_TOGGLE_ERROR_NONFINITE_PARAMETER);
        return 0;
    }

    low = fminf(xFrom, xTo);
    high = fmaxf(xFrom, xTo);
    if (input->mechanismType == HYD_MECHANISM_FIVE_POINT_TOGGLE) {
        if (input->toggleConfig == NULL) {
            set_error(error, HYD_TOGGLE_ERROR_NULL_ARGUMENT);
            return 0;
        }
        low = fmaxf(low, input->toggleConfig->xHandoffEffective);
        high = fminf(high, input->toggleConfig->raw.sm);
        if (low > high) {
            set_error(error, HYD_TOGGLE_ERROR_POSITION_OUT_OF_RANGE);
            return 0;
        }
    }

    if (!range_max_abs_ratio(input, low, high, &ratio_max, &ratio_sign,
                             error)) {
        return 0;
    }

    if (direction == HYD_DIRECTION_EXTEND || direction == HYD_DIRECTION_RETRACT) {
        if (!velocity_for_direction(input,
                                    (direction == HYD_DIRECTION_EXTEND) ? 1.0f : -1.0f,
                                    ratio_max, ratio_sign, &velocity, error)) {
            return 0;
        }
    } else {
        HYD_REAL extend_velocity;
        HYD_REAL retract_velocity;

        if (!velocity_for_direction(input, 1.0f, ratio_max, ratio_sign,
                                    &extend_velocity, error) ||
            !velocity_for_direction(input, -1.0f, ratio_max, ratio_sign,
                                    &retract_velocity, error)) {
            return 0;
        }
        velocity = fminf(extend_velocity, retract_velocity);
    }

    *maxTemplateVelocity = velocity;
    *velocityRatioMax = ratio_max;
    set_error(error, HYD_TOGGLE_ERROR_NONE);
    return 1;
}
