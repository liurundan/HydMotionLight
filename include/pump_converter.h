#ifndef HDY_PUMP_CONVERTER_H
#define HDY_PUMP_CONVERTER_H

#include "common_types.h"

typedef struct {
    HDY_REAL requestedFlow;          /* L/min, nonnegative magnitude expected */
    HDY_REAL flowToPumpSpeedGain;    /* rpm per L/min */
    HDY_REAL pumpSpeedLimit;         /* rpm, nonnegative */
    HDY_MotionDirection direction;   /* reserved for future directional pump mappings */
} HDY_PumpConverterInput;

typedef struct {
    HDY_REAL commandFlow;            /* L/min after pump-speed limit back-projection */
    HDY_REAL pumpSpeed;              /* rpm, nonnegative */
} HDY_PumpConverterOutput;

void HDY_PumpConverter_Execute(const HDY_PumpConverterInput* input,
                               HDY_PumpConverterOutput* output);

#endif /* HDY_PUMP_CONVERTER_H */
