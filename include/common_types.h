#ifndef HDY_COMMON_TYPES_H
#define HDY_COMMON_TYPES_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Basic type definitions
typedef bool HDY_BOOL;
typedef double HDY_REAL;
typedef double HDY_TIME;
typedef uint8_t HDY_UINT8;
typedef size_t HDY_UINT;

// Constants
#define HDY_MAX_SEGMENTS 16
#define HDY_NAME_MAX 32
#define HDY_MESSAGE_MAX 64

// Enums shared between modules
typedef enum {
    HDY_SEGMENT_TYPE_CLAMPING,
    HDY_SEGMENT_TYPE_OPENING,
    HDY_SEGMENT_TYPE_INJECTION,
    HDY_SEGMENT_TYPE_HOLDING,
    HDY_SEGMENT_TYPE_EJECTION,
    HDY_SEGMENT_TYPE_CORE_PULL,
    HDY_SEGMENT_TYPE_OTHER
} HDY_SegmentType;

typedef enum {
    HDY_PLANNER_POSITION_BASED,
    HDY_PLANNER_TIME_BASED
} HDY_PlannerType;

typedef enum {
    HDY_MODE_POSITION,
    HDY_MODE_SPEED_RAMP,
    HDY_MODE_PRESSURE_CLOSED_LOOP
} HDY_ControlMode;

typedef enum {
    HDY_END_POSITION,
    HDY_END_TIME,
    HDY_END_PRESSURE,
    HDY_END_FLOW,
    HDY_END_MANUAL
} HDY_EndConditionType;

// Data structures
typedef struct {
    HDY_REAL position;   /* mm */
    HDY_REAL velocity;   /* mm/s */
    HDY_REAL flow;       /* L/min */
    HDY_REAL pressure;   /* MPa */
    HDY_TIME timestamp;  /* s */
} HDY_AxisRef;

typedef struct {
    char name[HDY_NAME_MAX];
    HDY_SegmentType type;
    HDY_PlannerType planner;
    HDY_ControlMode mode;
    HDY_EndConditionType endCondition;

    HDY_REAL targetPosition;
    HDY_REAL targetFlow;
    HDY_REAL targetPressure;
    HDY_REAL maxAcceleration;
    HDY_REAL maxVelocity;
    HDY_TIME duration;
    HDY_REAL tolerance;
    HDY_REAL velocityToFlowGain; /* L/min per mm/s, actuator flow gain */
    HDY_REAL pressureRampRate; /* MPa/s, rate to ramp pressure to prevent sudden changes */
} HDY_MotionSegment;

typedef struct {
    HDY_BOOL overPressure;
    HDY_BOOL underPressure;
    HDY_BOOL flowDeviation;
    HDY_BOOL positionDeviation;
    HDY_BOOL timeout;
    HDY_REAL pressureError;
    HDY_REAL flowError;
    HDY_REAL velocityError;
    char message[HDY_MESSAGE_MAX];
} HDY_DiagnosticInfo;

typedef struct {
    HDY_UINT currentSegmentIndex;
    HDY_BOOL active;
    HDY_BOOL finished;
    HDY_REAL plannedVelocity;
    HDY_REAL plannedFlow;
    HDY_REAL commandedPumpSpeed;
    char currentSegmentName[HDY_NAME_MAX];
} HDY_MotionState;

#endif /* HDY_COMMON_TYPES_H */