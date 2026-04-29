#include "motion_validator.h"
#include "recipe_validator.h"
#include "pump_converter.h"

HDY_BOOL HDY_MotionValidator_UsesRecipeSource(const HDY_MotionControlFB* fb) {
    return (fb != NULL) ? fb->USE_RECIPE : true;
}

HDY_BOOL HDY_MotionValidator_HasSelectedStartSource(const HDY_MotionControlFB* fb) {
    if (fb == NULL) {
        return false;
    }

    if (HDY_MotionValidator_UsesRecipeSource(fb)) {
        return fb->RECIPE_SIZE > 0U;
    }

    return fb->DIRECT_SEGMENT_VALID;
}

const HDY_MotionSegment* HDY_MotionValidator_ResolveStartSourceSegment(
    const HDY_MotionControlFB* fb,
    size_t requestedSegmentIndex,
    size_t* resolvedSegmentIndex,
    HDY_SegmentSource* resolvedSource) {
    if (resolvedSegmentIndex != NULL) {
        *resolvedSegmentIndex = HDY_MAX_SEGMENTS;
    }
    if (resolvedSource != NULL) {
        *resolvedSource = HDY_SEGMENT_SOURCE_NONE;
    }

    if (fb == NULL) {
        return NULL;
    }

    if (HDY_MotionValidator_UsesRecipeSource(fb)) {
        if (requestedSegmentIndex >= fb->RECIPE_SIZE) {
            return NULL;
        }
        if (resolvedSegmentIndex != NULL) {
            *resolvedSegmentIndex = requestedSegmentIndex;
        }
        if (resolvedSource != NULL) {
            *resolvedSource = HDY_SEGMENT_SOURCE_RECIPE;
        }
        return &fb->RECIPE[requestedSegmentIndex];
    }

    if (!fb->DIRECT_SEGMENT_VALID) {
        return NULL;
    }
    if (resolvedSource != NULL) {
        *resolvedSource = HDY_SEGMENT_SOURCE_DIRECT;
    }
    return &fb->DIRECT_SEGMENT;
}

HDY_FbState HDY_MotionValidator_ResolveEffectiveFbState(const HDY_MotionControlFB* fb) {
    if (fb == NULL) {
        return HDY_FB_STATE_IDLE;
    }

    return fb->FB_STATE;
}

HDY_BOOL HDY_MotionValidator_ValidateStartRequest(const HDY_MotionControlFB* fb,
                                                   size_t segmentIndex,
                                                   HDY_DiagnosticCode* code) {
    const HDY_MotionSegment* segment;
    size_t resolvedSegmentIndex;

    if (fb == NULL) {
        return false;
    }

    if (!HDY_MotionValidator_HasSelectedStartSource(fb)) {
        if (code != NULL) {
            *code = HDY_MotionValidator_UsesRecipeSource(fb) ? HDY_DIAG_CODE_NO_RECIPE
                                                             : HDY_DIAG_CODE_NO_DIRECT_SEGMENT;
        }
        return false;
    }

    segment = HDY_MotionValidator_ResolveStartSourceSegment(fb, segmentIndex, &resolvedSegmentIndex, NULL);
    if (segment == NULL) {
        if (code != NULL) {
            *code = HDY_DIAG_CODE_SEGMENT_INDEX_OUT_OF_RANGE;
        }
        return false;
    }

    return HDY_PumpConverter_ValidateConfig(fb->FLOW_TO_PUMP_SPEED_GAIN,
                                            fb->PUMP_SPEED_LIMIT,
                                            code) &&
        HDY_RecipeValidator_ValidateSegment(segment,
                                            resolvedSegmentIndex,
                                            code) &&
        HDY_RecipeValidator_ValidateStartContext(segment,
                                                 resolvedSegmentIndex,
                                                 &fb->AXIS_REF,
                                                 code);
}

HDY_BOOL HDY_MotionValidator_ValidateNextRequest(const HDY_MotionControlFB* fb,
                                                  HDY_DiagnosticCode* code) {
    HDY_FbState effectiveState;

    if (fb == NULL) {
        return false;
    }

    effectiveState = HDY_MotionValidator_ResolveEffectiveFbState(fb);
    if (!HDY_MotionValidator_UsesRecipeSource(fb)) {
        if (code != NULL) {
            *code = HDY_DIAG_CODE_COMMAND_NOT_ALLOWED;
        }
        return false;
    }

    if (fb->RECIPE_SIZE == 0U) {
        if (code != NULL) {
            *code = HDY_DIAG_CODE_NO_RECIPE;
        }
        return false;
    }

    if (effectiveState == HDY_FB_STATE_DONE) {
        if (code != NULL) {
            *code = HDY_DIAG_CODE_RECIPE_ALREADY_FINISHED;
        }
        return false;
    }

    if (effectiveState == HDY_FB_STATE_STARTING || effectiveState == HDY_FB_STATE_RUNNING) {
        if (code != NULL) {
            *code = HDY_DIAG_CODE_SEGMENT_NOT_COMPLETED;
        }
        return false;
    }

    if (!fb->SEGMENT_COMPLETED) {
        if (code != NULL) {
            *code = HDY_DIAG_CODE_SEGMENT_NOT_COMPLETED;
        }
        return false;
    }

    return true;
}

HDY_BOOL HDY_MotionValidator_ValidatePumpConfig(HDY_REAL flowToPumpSpeedGain,
                                                  HDY_REAL pumpSpeedLimit,
                                                  HDY_DiagnosticCode* code) {
    return HDY_PumpConverter_ValidateConfig(flowToPumpSpeedGain, pumpSpeedLimit, code);
}
