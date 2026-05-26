#include "motion_validator.h"
#include "recipe_validator.h"
#include "pump_converter.h"

HYD_BOOL HYD_MotionValidator_UsesRecipeSource(const HYD_MotionControlFB* fb) {
    return (fb != NULL) ? fb->USE_RECIPE : true;
}

HYD_BOOL HYD_MotionValidator_HasSelectedStartSource(const HYD_MotionControlFB* fb) {
    if (fb == NULL) {
        return false;
    }

    if (HYD_MotionValidator_UsesRecipeSource(fb)) {
        return fb->RECIPE_SIZE > 0U;
    }

    return fb->DIRECT_SEGMENT_VALID;
}

const HYD_MotionSegment* HYD_MotionValidator_ResolveStartSourceSegment(
    const HYD_MotionControlFB* fb,
    size_t requestedSegmentIndex,
    size_t* resolvedSegmentIndex,
    HYD_SegmentSource* resolvedSource) {
    if (resolvedSegmentIndex != NULL) {
        *resolvedSegmentIndex = HYD_MAX_SEGMENTS;
    }
    if (resolvedSource != NULL) {
        *resolvedSource = HYD_SEGMENT_SOURCE_NONE;
    }

    if (fb == NULL) {
        return NULL;
    }

    if (HYD_MotionValidator_UsesRecipeSource(fb)) {
        if (requestedSegmentIndex >= fb->RECIPE_SIZE) {
            return NULL;
        }
        if (resolvedSegmentIndex != NULL) {
            *resolvedSegmentIndex = requestedSegmentIndex;
        }
        if (resolvedSource != NULL) {
            *resolvedSource = HYD_SEGMENT_SOURCE_RECIPE;
        }
        return &fb->RECIPE[requestedSegmentIndex];
    }

    if (!fb->DIRECT_SEGMENT_VALID) {
        return NULL;
    }
    if (resolvedSource != NULL) {
        *resolvedSource = HYD_SEGMENT_SOURCE_DIRECT;
    }
    return &fb->DIRECT_SEGMENT;
}

HYD_FbState HYD_MotionValidator_ResolveEffectiveFbState(const HYD_MotionControlFB* fb) {
    if (fb == NULL) {
        return HYD_FB_STATE_IDLE;
    }

    return fb->FB_STATE;
}

HYD_BOOL HYD_MotionValidator_ValidateStartRequest(const HYD_MotionControlFB* fb,
                                                   size_t segmentIndex,
                                                   HYD_DiagnosticCode* code) {
    const HYD_MotionSegment* segment;
    size_t resolvedSegmentIndex;

    if (fb == NULL) {
        return false;
    }

    if (!HYD_MotionValidator_HasSelectedStartSource(fb)) {
        if (code != NULL) {
            *code = HYD_MotionValidator_UsesRecipeSource(fb) ? HYD_DIAG_CODE_NO_RECIPE
                                                             : HYD_DIAG_CODE_NO_DIRECT_SEGMENT;
        }
        return false;
    }

    segment = HYD_MotionValidator_ResolveStartSourceSegment(fb, segmentIndex, &resolvedSegmentIndex, NULL);
    if (segment == NULL) {
        if (code != NULL) {
            *code = HYD_DIAG_CODE_SEGMENT_INDEX_OUT_OF_RANGE;
        }
        return false;
    }

    return HYD_PumpConverter_ValidateConfig(fb->FLOW_TO_PUMP_SPEED_GAIN,
                                            fb->PUMP_SPEED_LIMIT,
                                            code) &&
        HYD_RecipeValidator_ValidateSegment(segment,
                                            resolvedSegmentIndex,
                                            code,
                                            &fb->cylinderConfig) &&
        HYD_RecipeValidator_ValidateStartContext(segment,
                                                 resolvedSegmentIndex,
                                                 &fb->AXIS_REF,
                                                 code);
}

HYD_BOOL HYD_MotionValidator_ValidateNextRequest(const HYD_MotionControlFB* fb,
                                                  HYD_DiagnosticCode* code) {
    HYD_FbState effectiveState;

    if (fb == NULL) {
        return false;
    }

    effectiveState = HYD_MotionValidator_ResolveEffectiveFbState(fb);
    if (!HYD_MotionValidator_UsesRecipeSource(fb)) {
        if (code != NULL) {
            *code = HYD_DIAG_CODE_COMMAND_NOT_ALLOWED;
        }
        return false;
    }

    if (fb->RECIPE_SIZE == 0U) {
        if (code != NULL) {
            *code = HYD_DIAG_CODE_NO_RECIPE;
        }
        return false;
    }

    if (effectiveState == HYD_FB_STATE_DONE) {
        if (code != NULL) {
            *code = HYD_DIAG_CODE_RECIPE_ALREADY_FINISHED;
        }
        return false;
    }

    if (effectiveState == HYD_FB_STATE_STARTING || effectiveState == HYD_FB_STATE_RUNNING) {
        if (code != NULL) {
            *code = HYD_DIAG_CODE_SEGMENT_NOT_COMPLETED;
        }
        return false;
    }

    if (!fb->SEGMENT_COMPLETED) {
        if (code != NULL) {
            *code = HYD_DIAG_CODE_SEGMENT_NOT_COMPLETED;
        }
        return false;
    }

    return true;
}

HYD_BOOL HYD_MotionValidator_ValidatePumpConfig(HYD_REAL flowToPumpSpeedGain,
                                                  HYD_REAL pumpSpeedLimit,
                                                  HYD_DiagnosticCode* code) {
    return HYD_PumpConverter_ValidateConfig(flowToPumpSpeedGain, pumpSpeedLimit, code);
}
