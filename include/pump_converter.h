#ifndef HDY_PUMP_CONVERTER_H
#define HDY_PUMP_CONVERTER_H

#include "common_types.h"

typedef struct {
    HDY_REAL requestedFlow;          /* L/min magnitude request; negative values are normalized, non-finite values are rejected to safe zero */
    HDY_REAL flowToPumpSpeedGain;    /* rpm per L/min, finite and > 0 */
    HDY_REAL pumpSpeedLimit;         /* rpm, finite and >= 0 */
    HDY_MotionDirection direction;   /* reserved for future directional pump mappings */
} HDY_PumpConverterInput;

typedef struct {
    HDY_REAL commandFlow;            /* L/min after pump-speed limit back-projection */
    HDY_REAL pumpSpeed;              /* rpm, nonnegative */
} HDY_PumpConverterOutput;

void HDY_PumpConverter_Execute(const HDY_PumpConverterInput* input,
                               HDY_PumpConverterOutput* output);

/* Validate runtime configuration for pump conversion. Returns true if the
 * provided gain and limit are finite and within the accepted range.
 * On failure, `code` is populated when provided.
 */
HDY_BOOL HDY_PumpConverter_ValidateConfig(HDY_REAL flowToPumpSpeedGain,
                                          HDY_REAL pumpSpeedLimit,
                                          HDY_DiagnosticCode* code);

#endif /* HDY_PUMP_CONVERTER_H */
