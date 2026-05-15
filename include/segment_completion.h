#ifndef HYD_SEGMENT_COMPLETION_H
#define HYD_SEGMENT_COMPLETION_H

#include "common_types.h"

typedef struct {
    const HYD_MotionSegment* segment;
    const HYD_AxisRef* axisRef;
    const HYD_ExecutionReference* references;
    HYD_TIME timestamp;
    HYD_TIME* candidateStartTime;
    HYD_BOOL* candidateActive;
} HYD_SegmentCompletionContext;

HYD_BOOL HYD_SegmentCompletion_CheckWithContext(const HYD_SegmentCompletionContext* context);
HYD_BOOL HYD_SegmentCompletion_Check(const HYD_MotionSegment* segment,
                                     const HYD_AxisRef* axisRef,
                                     HYD_REAL elapsed);

#endif /* HYD_SEGMENT_COMPLETION_H */
