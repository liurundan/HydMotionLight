#ifndef HYD_RAMP_CONTROLLER_H
#define HYD_RAMP_CONTROLLER_H

#include "common_types.h"

typedef struct {
    HYD_REAL rampedPressure;
    HYD_TIME lastTimestamp;
    HYD_REAL transitionTargetPressure;
    HYD_REAL transitionRate;
    HYD_TIME transitionDuration;
    HYD_REAL profileRate;      /* bar/s, profiled reference slope */
    HYD_REAL profileRateLimit; /* bar/s, latched when the target or time changes */
    HYD_REAL profileDecel;     /* bar/s^2, latched together with profileRateLimit */
    HYD_BOOL profileLatched;
} HYD_RampController;

typedef struct {
    HYD_REAL targetPressure;
    HYD_REAL rampRate; /* ms pressure transition time */
    HYD_TIME currentTime;
} HYD_RampControllerInput;

/* Profiled reference limits (spec 2026-11-23 §4.2). */
typedef struct {
    HYD_REAL riseRate;  /* bar/s, slope bound when the target is above the reference */
    HYD_REAL fallRate;  /* bar/s, slope bound when the target is below the reference */
    HYD_REAL brakeTime; /* s, deceleration = slope bound / brakeTime */
} HYD_RampProfileLimits;

typedef struct {
    HYD_REAL rampedPressure;
    HYD_REAL rampedPressureRate; /* bar/s; the legacy time-based functions always report 0 */
} HYD_RampControllerOutput;

void HYD_RampController_Init(HYD_RampController* controller, HYD_REAL initialPressure, HYD_TIME initialTime);
void HYD_RampController_Execute(HYD_RampController* controller, const HYD_RampControllerInput* input, HYD_RampControllerOutput* output);
void HYD_RampController_ExecuteWithDt(HYD_RampController* controller,
                                      const HYD_RampControllerInput* input,
                                      HYD_TIME deltaTime,
                                      HYD_RampControllerOutput* output);
/* Rate-limited reference with sqrt braking. input->rampRate becomes the
 * minimum transition time: the slower of |target - reference| / time and the
 * latched slope bound wins. Never passes the target; reports the slope. */
void HYD_RampController_ExecuteProfiledWithDt(HYD_RampController* controller,
                                              const HYD_RampControllerInput* input,
                                              const HYD_RampProfileLimits* limits,
                                              HYD_TIME deltaTime,
                                              HYD_RampControllerOutput* output);

#endif /* HYD_RAMP_CONTROLLER_H */
