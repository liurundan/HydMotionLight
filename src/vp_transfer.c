#include "vp_transfer.h"

#include <math.h>

static void HYD_VpTransfer_Clear(HYD_VpTransferResult* result) {
    if (result == NULL) {
        return;
    }

    result->ready = false;
    result->reason = HYD_VP_TRANSFER_REASON_NONE;
}

void HYD_VpTransfer_Evaluate(const HYD_MotionSegment* segment,
                             const HYD_AxisRef* axisRef,
                             const HYD_ExecutionReference* references,
                             HYD_VpTransferResult* result) {
    HYD_REAL velocityReference = 0.0;

    HYD_VpTransfer_Clear(result);

    if (segment == NULL || axisRef == NULL || result == NULL) {
        return;
    }

    if (segment->segmentType != HYD_SEGMENT_TYPE_INJECTION ||
        segment->mode != HYD_MODE_SPEED_RAMP) {
        return;
    }

    if (segment->vpTransferPriority == HYD_VP_PRIORITY_PRESSURE_FIRST) {
        /* Pressure-first: check pressure before position */
        if (segment->vpTransferPressure > 0.0 &&
            axisRef->pressure >= segment->vpTransferPressure) {
            result->ready = true;
            result->reason = HYD_VP_TRANSFER_REASON_PRESSURE;
            return;
        }

        if (segment->vpTransferPosition > 0.0 &&
            axisRef->position >= segment->vpTransferPosition) {
            result->ready = true;
            result->reason = HYD_VP_TRANSFER_REASON_POSITION;
            return;
        }
    } else {
        /* Position-first (default): check position before pressure */
        if (segment->vpTransferPosition > 0.0 &&
            axisRef->position >= segment->vpTransferPosition) {
            result->ready = true;
            result->reason = HYD_VP_TRANSFER_REASON_POSITION;
            return;
        }

        if (segment->vpTransferPressure > 0.0 &&
            axisRef->pressure >= segment->vpTransferPressure) {
            result->ready = true;
            result->reason = HYD_VP_TRANSFER_REASON_PRESSURE;
            return;
        }
    }

    /* Time and velocity_drop order is unchanged (both are lower priority) */

    if (segment->vpTransferMinTime > 0.0 &&
        references != NULL &&
        references->elapsedTime >= segment->vpTransferMinTime) {
        result->ready = true;
        result->reason = HYD_VP_TRANSFER_REASON_TIME;
        return;
    }

#if HYD_ENABLE_EXECUTION_REFERENCE
    velocityReference = (references != NULL) ? fabs(references->velocityReference) : 0.0;
#endif
    if (segment->vpTransferVelocityDrop > 0.0 &&
        velocityReference > 0.0 &&
        velocityReference - fabs(axisRef->velocity) >= segment->vpTransferVelocityDrop) {
        result->ready = true;
        result->reason = HYD_VP_TRANSFER_REASON_VELOCITY_DROP;
    }
}
