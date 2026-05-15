#ifndef HYD_VP_TRANSFER_H
#define HYD_VP_TRANSFER_H

#include "common_types.h"

typedef struct {
    HYD_BOOL ready;
    HYD_VpTransferReason reason;
} HYD_VpTransferResult;

void HYD_VpTransfer_Evaluate(const HYD_MotionSegment* segment,
                             const HYD_AxisRef* axisRef,
                             const HYD_ExecutionReference* references,
                             HYD_VpTransferResult* result);

#endif /* HYD_VP_TRANSFER_H */
