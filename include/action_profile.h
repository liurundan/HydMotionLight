#ifndef HYD_ACTION_PROFILE_H
#define HYD_ACTION_PROFILE_H

#include "common_types.h"

HYD_BOOL HYD_ActionProfile_BuildClampClose(HYD_MotionSegment* segment,
                                           const HYD_MotionFBParams* params,
                                           HYD_UINT8 segmentTag,
                                           HYD_REAL targetPosition);

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
