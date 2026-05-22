#include "action_profile.h"
#include <string.h>

static HYD_BOOL HYD_ActionProfile_HasUsableParams(const HYD_MotionFBParams* params) {
    return params != NULL &&
           params->maxFlow > 0.0 &&
           params->maxVelocity > 0.0 &&
           params->maxAcceleration > 0.0 &&
           params->velocityToFlowGain > 0.0;
}

static void HYD_ActionProfile_ApplyCommon(HYD_MotionSegment* segment,
                                          const HYD_MotionFBParams* params,
                                          HYD_UINT8 segmentTag,
                                          HYD_SegmentType segmentType) {
    memset(segment, 0, sizeof(*segment));
    segment->segmentTag = segmentTag;
    segment->segmentType = segmentType;
    segment->targetFlow = params->defaultTargetFlow;
    segment->maxFlow = params->maxFlow;
    segment->maxVelocity = params->maxVelocity;
    segment->maxAcceleration = params->maxAcceleration;
    segment->maxDeceleration = params->maxDeceleration;
    segment->positionTolerance = params->positionTolerance;
    segment->velocityTolerance = params->velocityTolerance;
    segment->flowTolerance = params->flowTolerance;
    segment->pressureTolerance = params->pressureTolerance;
    segment->timeoutLimit = params->timeoutLimit;
    segment->velocityToFlowGain = params->velocityToFlowGain;
    segment->velocityKp = params->velocityKp;
    segment->velocityDeadband = params->velocityDeadband;
    segment->velocityCorrectionLimit = params->velocityCorrectionLimit;
    segment->pressureRampRate = params->pressureRampRate;
    segment->pressureController = (HYD_PressureControllerType)((int)params->pressureControllerType);
    segment->pressureKp = params->pressureKp;
    segment->pressureKpHigh = params->pressureKpHigh;
    segment->pressureGainBand = params->pressureGainBand;
    segment->pressureKi = params->pressureKi;
    segment->pressureKd = params->pressureKd;
    segment->pressureIntegralLimit = params->pressureIntegralLimit;
    segment->pressureDeadband = params->pressureDeadband;
    segment->pressureFilterAlpha = params->pressureFilterAlpha;
    segment->pressureDerivativeFilterAlpha = params->pressureDerivativeFilterAlpha;
}

static HYD_BOOL HYD_ActionProfile_BuildPosition(HYD_MotionSegment* segment,
                                                const HYD_MotionFBParams* params,
                                                HYD_UINT8 segmentTag,
                                                HYD_SegmentType segmentType,
                                                HYD_MotionDirection direction,
                                                HYD_REAL targetPosition) {
    if (segment == NULL || !HYD_ActionProfile_HasUsableParams(params)) {
        return false;
    }

    HYD_ActionProfile_ApplyCommon(segment, params, segmentTag, segmentType);
    segment->planner = HYD_PLANNER_TIME_BASED;
    segment->mode = HYD_MODE_POSITION;
    segment->endCondition = HYD_END_POSITION;
    segment->direction = direction;
    segment->targetPosition = targetPosition;
    return true;
}

HYD_BOOL HYD_ActionProfile_BuildClampClose(HYD_MotionSegment* segment,
                                           const HYD_MotionFBParams* params,
                                           HYD_UINT8 segmentTag,
                                           HYD_REAL targetPosition) {
    return HYD_ActionProfile_BuildPosition(segment, params, segmentTag,
                                           HYD_SEGMENT_TYPE_CLAMPING,
                                           HYD_DIRECTION_EXTEND,
                                           targetPosition);
}

HYD_BOOL HYD_ActionProfile_BuildClampCloseWithMoldProtect(HYD_MotionSegment* segment,
                                                          const HYD_MotionFBParams* params,
                                                          HYD_UINT8 segmentTag,
                                                          HYD_REAL targetPosition,
                                                          HYD_REAL protectWindowStart,
                                                          HYD_REAL pressureCeiling,
                                                          HYD_REAL pressureCeilingTolerance,
                                                          HYD_REAL derateRatio) {
    if (segment == NULL || pressureCeiling <= 0.0 ||
        protectWindowStart >= targetPosition) {
        return false;
    }
    /* derateRatio must be 0 (use default) or strictly in (0, 1). */
    if (derateRatio != 0.0 && (derateRatio <= 0.0 || derateRatio >= 1.0)) {
        return false;
    }
    if (!HYD_ActionProfile_BuildClampClose(segment, params, segmentTag, targetPosition)) {
        return false;
    }
    segment->pressureCeiling = pressureCeiling;
    segment->pressureCeilingTolerance = pressureCeilingTolerance;  /* 0 -> use pressureTolerance via getter */
    segment->pressureCeilingPositionStart = protectWindowStart;
    segment->pressureCeilingPositionEnd = targetPosition;
    segment->derateRatio = derateRatio;
    return true;
}

HYD_BOOL HYD_ActionProfile_BuildClampOpen(HYD_MotionSegment* segment,
                                          const HYD_MotionFBParams* params,
                                          HYD_UINT8 segmentTag,
                                          HYD_REAL targetPosition) {
    return HYD_ActionProfile_BuildPosition(segment, params, segmentTag,
                                           HYD_SEGMENT_TYPE_OPENING,
                                           HYD_DIRECTION_RETRACT,
                                           targetPosition);
}

HYD_BOOL HYD_ActionProfile_BuildInjectionFill(HYD_MotionSegment* segment,
                                              const HYD_MotionFBParams* params,
                                              HYD_UINT8 segmentTag,
                                              HYD_REAL transferPosition) {
    if (segment == NULL || !HYD_ActionProfile_HasUsableParams(params)) {
        return false;
    }

    HYD_ActionProfile_ApplyCommon(segment, params, segmentTag, HYD_SEGMENT_TYPE_INJECTION);
    segment->planner = HYD_PLANNER_TIME_BASED;
    segment->mode = HYD_MODE_SPEED_RAMP;
    segment->endCondition = HYD_END_POSITION;
    segment->direction = HYD_DIRECTION_EXTEND;
    segment->targetPosition = transferPosition;
    return true;
}

HYD_BOOL HYD_ActionProfile_BuildHoldingPressure(HYD_MotionSegment* segment,
                                                const HYD_MotionFBParams* params,
                                                HYD_UINT8 segmentTag,
                                                HYD_REAL targetPressure,
                                                HYD_TIME duration) {
    if (segment == NULL || params == NULL || params->maxFlow <= 0.0 ||
        targetPressure <= 0.0 || duration <= 0.0) {
        return false;
    }

    HYD_ActionProfile_ApplyCommon(segment, params, segmentTag, HYD_SEGMENT_TYPE_HOLDING);
    segment->planner = HYD_PLANNER_TIME_BASED;
    segment->mode = HYD_MODE_PRESSURE_CLOSED_LOOP;
    segment->endCondition = HYD_END_TIME;
    segment->direction = HYD_DIRECTION_HOLD;
    segment->targetPressure = targetPressure;
    segment->duration = duration;
    return true;
}

HYD_BOOL HYD_ActionProfile_BuildEjectAdvance(HYD_MotionSegment* segment,
                                             const HYD_MotionFBParams* params,
                                             HYD_UINT8 segmentTag,
                                             HYD_REAL targetPosition) {
    return HYD_ActionProfile_BuildPosition(segment, params, segmentTag,
                                           HYD_SEGMENT_TYPE_EJECTION,
                                           HYD_DIRECTION_EXTEND,
                                           targetPosition);
}

HYD_BOOL HYD_ActionProfile_BuildEjectRetract(HYD_MotionSegment* segment,
                                             const HYD_MotionFBParams* params,
                                             HYD_UINT8 segmentTag,
                                             HYD_REAL targetPosition) {
    return HYD_ActionProfile_BuildPosition(segment, params, segmentTag,
                                           HYD_SEGMENT_TYPE_EJECTION,
                                           HYD_DIRECTION_RETRACT,
                                           targetPosition);
}

HYD_BOOL HYD_ActionProfile_BuildCarriageMove(HYD_MotionSegment* segment,
                                             const HYD_MotionFBParams* params,
                                             HYD_UINT8 segmentTag,
                                             HYD_REAL targetPosition,
                                             HYD_MotionDirection direction) {
    if (direction != HYD_DIRECTION_EXTEND && direction != HYD_DIRECTION_RETRACT) {
        return false;
    }
    return HYD_ActionProfile_BuildPosition(segment, params, segmentTag,
                                           HYD_SEGMENT_TYPE_OTHER,
                                           direction,
                                           targetPosition);
}
