#ifndef HYD_ACTION_PROFILE_H
#define HYD_ACTION_PROFILE_H

#include "common_types.h"

HYD_BOOL HYD_ActionProfile_BuildClampClose(HYD_MotionSegment* segment,
                                           const HYD_MotionFBParams* params,
                                           HYD_UINT8 segmentTag,
                                           HYD_REAL targetPosition);

/*
 * Build a clamp-close segment with a low-pressure mold-protect envelope.
 *
 * targetPosition          - final mold-closed position (mm).
 * protectWindowStart      - position at which the protect window begins (mm).
 *                           Below this, normal clamp velocity is used.
 * pressureCeiling         - soft upper bound during the window (bar).
 * pressureCeilingTolerance - hysteresis above ceiling before DERATE (bar);
 *                           pass 0 to fall back to params->pressureTolerance.
 * derateRatio             - fraction of normal output flow to use when ceiling
 *                           is exceeded (0,1). Pass 0 to use library default.
 *
 * Returns false on NULL pointers, invalid params, or invalid arguments.
 * Window ends at targetPosition (inclusive). To reuse outside clamping
 * (e.g. injection mold protect), use BuildClampClose + manual field set.
 */
HYD_BOOL HYD_ActionProfile_BuildClampCloseWithMoldProtect(HYD_MotionSegment* segment,
                                                          const HYD_MotionFBParams* params,
                                                          HYD_UINT8 segmentTag,
                                                          HYD_REAL targetPosition,
                                                          HYD_REAL protectWindowStart,
                                                          HYD_REAL pressureCeiling,
                                                          HYD_REAL pressureCeilingTolerance,
                                                          HYD_REAL derateRatio);

HYD_BOOL HYD_ActionProfile_BuildClampOpen(HYD_MotionSegment* segment,
                                          const HYD_MotionFBParams* params,
                                          HYD_UINT8 segmentTag,
                                          HYD_REAL targetPosition);

HYD_BOOL HYD_ActionProfile_BuildInjectionFill(HYD_MotionSegment* segment,
                                              const HYD_MotionFBParams* params,
                                              HYD_UINT8 segmentTag,
                                              HYD_REAL transferPosition);

HYD_BOOL HYD_ActionProfile_BuildHoldingPressure(HYD_MotionSegment* segment,
                                                const HYD_MotionFBParams* params,
                                                HYD_UINT8 segmentTag,
                                                HYD_REAL targetPressure,
                                                HYD_TIME duration);

HYD_BOOL HYD_ActionProfile_BuildEjectAdvance(HYD_MotionSegment* segment,
                                             const HYD_MotionFBParams* params,
                                             HYD_UINT8 segmentTag,
                                             HYD_REAL targetPosition);

HYD_BOOL HYD_ActionProfile_BuildEjectRetract(HYD_MotionSegment* segment,
                                             const HYD_MotionFBParams* params,
                                             HYD_UINT8 segmentTag,
                                             HYD_REAL targetPosition);

HYD_BOOL HYD_ActionProfile_BuildCarriageMove(HYD_MotionSegment* segment,
                                             const HYD_MotionFBParams* params,
                                             HYD_UINT8 segmentTag,
                                             HYD_REAL targetPosition,
                                             HYD_MotionDirection direction);

#endif /* HYD_ACTION_PROFILE_H */
