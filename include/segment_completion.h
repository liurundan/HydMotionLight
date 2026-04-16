#ifndef HDY_SEGMENT_COMPLETION_H
#define HDY_SEGMENT_COMPLETION_H

#include "common_types.h"

typedef struct {
    const HDY_MotionSegment* segment;
    const HDY_AxisRef* axisRef;
    const HDY_ExecutionReference* references;
} HDY_SegmentCompletionContext;

HDY_BOOL HDY_SegmentCompletion_CheckWithContext(const HDY_SegmentCompletionContext* context);
HDY_BOOL HDY_SegmentCompletion_Check(const HDY_MotionSegment* segment,
                                     const HDY_AxisRef* axisRef,
                                     HDY_REAL elapsed);

#endif /* HDY_SEGMENT_COMPLETION_H */
