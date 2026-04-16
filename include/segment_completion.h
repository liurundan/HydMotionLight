#ifndef HDY_SEGMENT_COMPLETION_H
#define HDY_SEGMENT_COMPLETION_H

#include "common_types.h"

HDY_BOOL HDY_SegmentCompletion_Check(const HDY_MotionSegment* segment,
                                     const HDY_AxisRef* axisRef,
                                     HDY_REAL elapsed);

#endif /* HDY_SEGMENT_COMPLETION_H */
