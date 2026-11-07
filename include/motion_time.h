#ifndef HYD_MOTION_TIME_H
#define HYD_MOTION_TIME_H

#include <math.h>

#include "common_types.h"

#define HYD_MIN_MOTION_TIME_MS       ((HYD_REAL)100.0f)
#define HYD_DEFAULT_PRESSURE_TIME_MS ((HYD_REAL)20.0f)

static inline HYD_BOOL HYD_IsValidMotionTimeMs(HYD_REAL value)
{
    return isfinite(value) && value >= 0.0f;
}

static inline HYD_BOOL HYD_IsValidPressureTimeMs(HYD_REAL value)
{
    return isfinite(value) && value >= 0.0f;
}

static inline HYD_REAL HYD_NormalizeMotionTimeMs(HYD_REAL value)
{
    return (value < HYD_MIN_MOTION_TIME_MS)
        ? HYD_MIN_MOTION_TIME_MS
        : value;
}

static inline HYD_REAL HYD_NormalizePressureTimeMs(HYD_REAL value)
{
    return (value == 0.0f) ? HYD_DEFAULT_PRESSURE_TIME_MS : value;
}

static inline HYD_TIME HYD_MotionTimeMsToSeconds(HYD_REAL value)
{
    return (HYD_TIME)(HYD_NormalizeMotionTimeMs(value) * 0.001f);
}

static inline HYD_TIME HYD_PressureTimeMsToSeconds(HYD_REAL value)
{
    return (HYD_TIME)(HYD_NormalizePressureTimeMs(value) * 0.001f);
}

static inline void HYD_NormalizeMotionSegmentTimes(HYD_MotionSegment *segment)
{
    if (segment == NULL) {
        return;
    }
    segment->maxAcceleration = HYD_NormalizeMotionTimeMs(segment->maxAcceleration);
    segment->maxDeceleration = HYD_NormalizeMotionTimeMs(segment->maxDeceleration);
    if (segment->mode == HYD_MODE_PRESSURE_CLOSED_LOOP) {
        segment->pressureRampRate =
            HYD_NormalizePressureTimeMs(segment->pressureRampRate);
    }
}

#endif /* HYD_MOTION_TIME_H */
