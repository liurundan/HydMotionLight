#include "ripple_compensator.h"
#include "pressure_ripple_table.h"

#include <math.h>
#include <string.h>

#define HYD_RIPPLE_COMP_PI 3.14159265358979323846
#define HYD_RIPPLE_COMP_TWO_PI (2.0 * HYD_RIPPLE_COMP_PI)
#define HYD_RIPPLE_COMP_MAX_SAMPLE_INTERVAL_S 0.004
#define HYD_RIPPLE_COMP_ANGLE_TOLERANCE_DEG 2.0
#define HYD_RIPPLE_COMP_MIN_RUNNING_RPM 1.0
#define HYD_RIPPLE_COMP_MAX_FEEDBACK_SPEED_RATIO 1.05

/* One quarter-wave, linearly interpolated at runtime; no per-scan trig call. */
static const float HYD_RIPPLE_COMP_SINE_LUT[17] = {
    0.000000f, 0.098017f, 0.195090f, 0.290285f,
    0.382683f, 0.471397f, 0.555570f, 0.634393f,
    0.707107f, 0.773010f, 0.831470f, 0.881921f,
    0.923880f, 0.956940f, 0.980785f, 0.995185f,
    1.000000f
};

static HYD_REAL HYD_RippleComp_Clamp(HYD_REAL value,
                                      HYD_REAL lower,
                                      HYD_REAL upper) {
    if (value < lower) {
        return lower;
    }
    if (value > upper) {
        return upper;
    }
    return value;
}

static HYD_REAL HYD_RippleComp_NormalizeAngle(HYD_REAL angleDeg) {
    HYD_REAL normalized = angleDeg;

    while (normalized < 0.0) {
        normalized += 360.0;
    }
    while (normalized >= 360.0) {
        normalized -= 360.0;
    }
    return normalized;
}

static HYD_REAL HYD_RippleComp_ShortestAngleDelta(HYD_REAL current,
                                                   HYD_REAL previous) {
    HYD_REAL delta = current - previous;

    if (delta > 180.0) {
        delta -= 360.0;
    } else if (delta < -180.0) {
        delta += 360.0;
    }
    return delta;
}

static HYD_REAL HYD_RippleComp_LookupSine(HYD_REAL radians) {
    HYD_REAL phase = radians;
    HYD_REAL quadrantPosition;
    unsigned int quadrant;
    HYD_REAL quarterPosition;
    HYD_REAL offset;
    unsigned int lower;
    HYD_REAL fraction;
    HYD_REAL magnitude;

    while (phase < 0.0) {
        phase += HYD_RIPPLE_COMP_TWO_PI;
    }
    while (phase >= HYD_RIPPLE_COMP_TWO_PI) {
        phase -= HYD_RIPPLE_COMP_TWO_PI;
    }

    quadrantPosition = phase / (HYD_RIPPLE_COMP_PI * 0.5);
    quadrant = (unsigned int)quadrantPosition;
    if (quadrant > 3U) {
        quadrant = 3U;
    }
    quarterPosition = quadrantPosition - (HYD_REAL)quadrant;
    if (quadrant == 1U || quadrant == 3U) {
        quarterPosition = 1.0 - quarterPosition;
    }
    offset = quarterPosition * 16.0;
    lower = (unsigned int)offset;
    if (lower >= 16U) {
        lower = 15U;
        fraction = 1.0;
    } else {
        fraction = offset - (HYD_REAL)lower;
    }
    magnitude = (HYD_REAL)HYD_RIPPLE_COMP_SINE_LUT[lower] +
        fraction * ((HYD_REAL)HYD_RIPPLE_COMP_SINE_LUT[lower + 1U] -
                    (HYD_REAL)HYD_RIPPLE_COMP_SINE_LUT[lower]);

    return (quadrant < 2U) ? magnitude : -magnitude;
}

static HYD_BOOL HYD_RippleComp_TableIsValid(const HYD_RippleCompTable* table) {
    size_t index;

    if (table == NULL || !table->calibrated || table->entries == NULL ||
        table->count == 0U || table->count > HYD_RIPPLE_COMP_MAX_TABLE_ENTRIES) {
        return false;
    }

    for (index = 0U; index < table->count; ++index) {
        const HYD_PressureRippleEntry* entry = &table->entries[index];

        if (!isfinite(entry->rpm) || !isfinite(entry->amp13_rpm) ||
            !isfinite(entry->phase13_rad) || !isfinite(entry->amp26_rpm) ||
            !isfinite(entry->phase26_rad) || entry->rpm < 0.0f ||
            entry->phase13_rad < -HYD_RIPPLE_COMP_PI ||
            entry->phase13_rad > HYD_RIPPLE_COMP_PI ||
            entry->phase26_rad < -HYD_RIPPLE_COMP_PI ||
            entry->phase26_rad > HYD_RIPPLE_COMP_PI ||
            fabs((double)entry->amp13_rpm) > 0.30 * (double)entry->rpm ||
            fabs((double)entry->amp26_rpm) > 0.30 * (double)entry->rpm ||
            (index > 0U && entry->rpm <= table->entries[index - 1U].rpm)) {
            return false;
        }
    }

    return true;
}

static HYD_BOOL HYD_RippleComp_Interpolate(const HYD_RippleCompTable* table,
                                            HYD_REAL rpm,
                                            HYD_PressureRippleEntry* result) {
    size_t index;

    if (rpm < table->entries[0].rpm || rpm > table->entries[table->count - 1U].rpm) {
        return false;
    }

    for (index = 1U; index < table->count; ++index) {
        if (rpm <= table->entries[index].rpm) {
            const HYD_PressureRippleEntry* left = &table->entries[index - 1U];
            const HYD_PressureRippleEntry* right = &table->entries[index];
            HYD_REAL span = (HYD_REAL)right->rpm - (HYD_REAL)left->rpm;
            HYD_REAL fraction = (rpm - (HYD_REAL)left->rpm) / span;

            result->rpm = (float)rpm;
            result->amp13_rpm = (float)((HYD_REAL)left->amp13_rpm +
                fraction * ((HYD_REAL)right->amp13_rpm - (HYD_REAL)left->amp13_rpm));
            result->phase13_rad = (float)((HYD_REAL)left->phase13_rad +
                fraction * ((HYD_REAL)right->phase13_rad - (HYD_REAL)left->phase13_rad));
            result->amp26_rpm = (float)((HYD_REAL)left->amp26_rpm +
                fraction * ((HYD_REAL)right->amp26_rpm - (HYD_REAL)left->amp26_rpm));
            result->phase26_rad = (float)((HYD_REAL)left->phase26_rad +
                fraction * ((HYD_REAL)right->phase26_rad - (HYD_REAL)left->phase26_rad));
            return true;
        }
    }

    *result = table->entries[table->count - 1U];
    result->rpm = (float)rpm;
    return true;
}

void HYD_RippleComp_Reset(HYD_RippleCompState* state) {
    if (state != NULL) {
        memset(state, 0, sizeof(*state));
    }
}

HYD_RippleCompTable HYD_RippleComp_DefaultTable(void) {
    HYD_RippleCompTable table;

    table.entries = HYD_PRESSURE_RIPPLE_TABLE;
    table.count = HYD_PRESSURE_RIPPLE_TABLE_COUNT;
    table.calibrated = HYD_PRESSURE_RIPPLE_TABLE_CALIBRATED ? true : false;
    return table;
}

HYD_BOOL HYD_RippleComp_Scan(const HYD_PumpFeedback* feedback,
                             const HYD_RippleCompTable* table,
                             HYD_RippleCompState* state,
                             HYD_REAL baseRpm,
                             HYD_REAL pumpSpeedLimit,
                             HYD_RippleCompOutput* output) {
    HYD_REAL angle;
    HYD_REAL dt;
    HYD_REAL angleDelta;
    HYD_REAL expectedAngleDelta;
    HYD_REAL maxAngleDelta;
    HYD_REAL absoluteRpm;
    HYD_BOOL observable13;
    HYD_BOOL observable26;
    HYD_PressureRippleEntry entry;
    HYD_REAL deltaRpm;

    if (state == NULL || output == NULL) {
        return false;
    }
    memset(output, 0, sizeof(*output));

    if (feedback == NULL || !HYD_RippleComp_TableIsValid(table) ||
        !isfinite(baseRpm) || !isfinite(pumpSpeedLimit) ||
        pumpSpeedLimit <= 0.0 || fabs(baseRpm) <= HYD_RIPPLE_COMP_MIN_RUNNING_RPM ||
        !HYD_PumpFeedback_HasValid(feedback->validFlags,
                                   HYD_PUMP_FEEDBACK_VALID_RPM |
                                       HYD_PUMP_FEEDBACK_VALID_ANGLE |
                                       HYD_PUMP_FEEDBACK_VALID_TIMESTAMP) ||
        !isfinite(feedback->rpm) || !isfinite(feedback->angleDeg) ||
        !isfinite(feedback->timestamp) ||
        fabs(feedback->rpm) <= HYD_RIPPLE_COMP_MIN_RUNNING_RPM ||
        fabs(feedback->rpm) >
            HYD_RIPPLE_COMP_MAX_FEEDBACK_SPEED_RATIO * pumpSpeedLimit ||
        feedback->rpm * baseRpm < 0.0) {
        HYD_RippleComp_Reset(state);
        return false;
    }

    angle = HYD_RippleComp_NormalizeAngle(feedback->angleDeg);
    if (!state->initialized) {
        state->initialized = true;
        state->previousAngleDeg = angle;
        state->previousTimestamp = feedback->timestamp;
        state->previousRpm = feedback->rpm;
        return false;
    }

    dt = feedback->timestamp - state->previousTimestamp;
    angleDelta = HYD_RippleComp_ShortestAngleDelta(angle, state->previousAngleDeg);
    expectedAngleDelta = feedback->rpm * 6.0 * dt;
    maxAngleDelta = pumpSpeedLimit * 6.0 * dt + HYD_RIPPLE_COMP_ANGLE_TOLERANCE_DEG;
    if (dt <= 0.0 || dt > HYD_RIPPLE_COMP_MAX_SAMPLE_INTERVAL_S || !isfinite(dt) ||
        fabs(angleDelta) > 180.0 || fabs(angleDelta) > maxAngleDelta ||
        fabs(angleDelta - expectedAngleDelta) >
            HYD_RIPPLE_COMP_ANGLE_TOLERANCE_DEG ||
        (fabs(state->previousRpm) > HYD_RIPPLE_COMP_MIN_RUNNING_RPM &&
         state->previousRpm * feedback->rpm < 0.0) ||
        (fabs(angleDelta) > HYD_RIPPLE_COMP_ANGLE_TOLERANCE_DEG &&
         angleDelta * feedback->rpm < 0.0)) {
        HYD_RippleComp_Reset(state);
        return false;
    }

    absoluteRpm = fabs(feedback->rpm);
    if (!HYD_RippleComp_Interpolate(table, absoluteRpm, &entry)) {
        HYD_RippleComp_Reset(state);
        return false;
    }

    observable13 = (13.0 * absoluteRpm / 60.0) <= (0.45 / dt);
    observable26 = (26.0 * absoluteRpm / 60.0) <= (0.45 / dt);
    deltaRpm = 0.0;
    if (observable13) {
        deltaRpm += (HYD_REAL)entry.amp13_rpm * HYD_RippleComp_LookupSine(
            13.0 * angle * HYD_RIPPLE_COMP_PI / 180.0 + (HYD_REAL)entry.phase13_rad);
    }
    if (observable26) {
        deltaRpm += (HYD_REAL)entry.amp26_rpm * HYD_RippleComp_LookupSine(
            26.0 * angle * HYD_RIPPLE_COMP_PI / 180.0 + (HYD_REAL)entry.phase26_rad);
    }

    state->previousAngleDeg = angle;
    state->previousTimestamp = feedback->timestamp;
    state->previousRpm = feedback->rpm;
    if (!observable13 && !observable26) {
        return false;
    }

    output->deltaRpm = HYD_RippleComp_Clamp(deltaRpm,
                                             -0.30 * pumpSpeedLimit,
                                             0.30 * pumpSpeedLimit);
    output->active = isfinite(output->deltaRpm) ? true : false;
    if (!output->active) {
        output->deltaRpm = 0.0;
    }
    return output->active;
}
