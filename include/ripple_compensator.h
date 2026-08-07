#ifndef HYD_RIPPLE_COMPENSATOR_H
#define HYD_RIPPLE_COMPENSATOR_H

#include <stddef.h>
#include "common_types.h"

#ifndef HYD_PRESSURE_RIPPLE_ENTRY_DEFINED
#define HYD_PRESSURE_RIPPLE_ENTRY_DEFINED
typedef struct {
    float rpm;
    float amp13_rpm;
    float phase13_rad;
    float amp26_rpm;
    float phase26_rad;
} HYD_PressureRippleEntry;
#endif

typedef struct {
    const HYD_PressureRippleEntry* entries;
    size_t count;
    HYD_BOOL calibrated;
} HYD_RippleCompTable;

#define HYD_RIPPLE_COMP_MAX_TABLE_ENTRIES 32U

typedef struct {
    HYD_BOOL initialized;
    HYD_REAL previousAngleDeg;
    HYD_REAL previousTimestamp;
    HYD_REAL previousRpm;
} HYD_RippleCompState;

typedef struct {
    HYD_BOOL active;
    HYD_REAL deltaRpm;
} HYD_RippleCompOutput;

void HYD_RippleComp_Reset(HYD_RippleCompState* state);
/*
 * `state` belongs to the caller/adapter, not the motion FB. `table` and
 * `feedback` are borrowed for one scan. The generated production table is
 * deliberately disabled until calibration marks it as valid.
 */
HYD_BOOL HYD_RippleComp_Scan(const HYD_PumpFeedback* feedback,
                             const HYD_RippleCompTable* table,
                             HYD_RippleCompState* state,
                             HYD_REAL baseRpm,
                             HYD_REAL pumpSpeedLimit,
                             HYD_RippleCompOutput* output);

HYD_RippleCompTable HYD_RippleComp_DefaultTable(void);

#endif /* HYD_RIPPLE_COMPENSATOR_H */
