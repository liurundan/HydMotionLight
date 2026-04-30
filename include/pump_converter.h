#ifndef HYD_PUMP_CONVERTER_H
#define HYD_PUMP_CONVERTER_H

#include "common_types.h"

typedef struct {
    HYD_REAL requestedFlow;          /* L/min magnitude request; negative values are normalized, non-finite values are rejected to safe zero */
    HYD_REAL flowToPumpSpeedGain;    /* rpm per L/min, finite and > 0 */
    HYD_REAL pumpSpeedLimit;         /* rpm, finite and >= 0 */
    HYD_MotionDirection direction;   /* reserved for future directional pump mappings */
} HYD_PumpConverterInput;

typedef struct {
    HYD_REAL commandFlow;            /* L/min after pump-speed limit back-projection */
    HYD_REAL pumpSpeed;              /* rpm, nonnegative */
} HYD_PumpConverterOutput;

void HYD_PumpConverter_Execute(const HYD_PumpConverterInput* input,
                               HYD_PumpConverterOutput* output);

/* Validate runtime configuration for pump conversion. Returns true if the
 * provided gain and limit are finite and within the accepted range.
 * On failure, `code` is populated when provided.
 */
HYD_BOOL HYD_PumpConverter_ValidateConfig(HYD_REAL flowToPumpSpeedGain,
                                          HYD_REAL pumpSpeedLimit,
                                          HYD_DiagnosticCode* code);

#endif /* HYD_PUMP_CONVERTER_H */
