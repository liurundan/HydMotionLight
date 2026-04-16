#ifndef HDY_COMMON_TYPES_H
#define HDY_COMMON_TYPES_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Basic type definitions */
typedef bool HDY_BOOL;
typedef double HDY_REAL;
typedef double HDY_TIME;
typedef uint8_t HDY_UINT8;
typedef size_t HDY_UINT;

/* Constants */
#define HDY_MAX_SEGMENTS 16
#define HDY_NAME_MAX 32
#define HDY_MESSAGE_MAX 64

/* Enums shared between modules */
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

typedef enum {
    HDY_DIRECTION_AUTO,
    HDY_DIRECTION_EXTEND,
    HDY_DIRECTION_RETRACT,
    HDY_DIRECTION_HOLD
} HDY_MotionDirection;

typedef enum {
    HDY_STATUS_IDLE,
    HDY_STATUS_READY,
    HDY_STATUS_RUNNING,
    HDY_STATUS_SEGMENT_COMPLETE,
    HDY_STATUS_FINISHED,
    HDY_STATUS_FAULT
} HDY_ControllerStatus;

typedef enum {
    HDY_DIAG_SEVERITY_NONE,
    HDY_DIAG_SEVERITY_INFO,
    HDY_DIAG_SEVERITY_WARNING,
    HDY_DIAG_SEVERITY_FAULT
} HDY_DiagnosticSeverity;

typedef enum {
    HDY_DIAG_CODE_NONE,
    HDY_DIAG_CODE_RECIPE_EMPTY,
    HDY_DIAG_CODE_RECIPE_TOO_LARGE,
    HDY_DIAG_CODE_SEGMENT_INVALID,
    HDY_DIAG_CODE_RUNTIME_CONFIG_INVALID,
    HDY_DIAG_CODE_START_CONTEXT_INVALID,
    HDY_DIAG_CODE_NO_RECIPE,
    HDY_DIAG_CODE_SEGMENT_INDEX_OUT_OF_RANGE,
    HDY_DIAG_CODE_SEGMENT_NOT_COMPLETED,
    HDY_DIAG_CODE_RECIPE_ALREADY_FINISHED,
    HDY_DIAG_CODE_ABORTED,
    HDY_DIAG_CODE_TIMEOUT,
    HDY_DIAG_CODE_OVER_PRESSURE,
    HDY_DIAG_CODE_UNDER_PRESSURE,
    HDY_DIAG_CODE_FLOW_DEVIATION,
    HDY_DIAG_CODE_POSITION_DEVIATION,
    HDY_DIAG_CODE_VELOCITY_DEVIATION,
    HDY_DIAG_CODE_INTERNAL_ERROR
} HDY_DiagnosticCode;

/* Data structures */
typedef struct {
    HDY_REAL position;   /* mm */
    HDY_REAL velocity;   /* mm/s, signed by mechanism direction */
    HDY_REAL flow;       /* L/min, magnitude at the pump side */
    HDY_REAL pressure;   /* MPa */
    HDY_TIME timestamp;  /* s */
} HDY_AxisRef;

/*
 * Motion-mode semantics:
 * - HDY_MODE_POSITION:
 *   Converges to targetPosition. planner selects either pure position-based
 *   braking or time-ramp buildup with position braking protection.
 * - HDY_MODE_SPEED_RAMP:
 *   Builds velocity magnitude by time ramp. The process layer still owns valve
 *   direction, so the segment must declare EXTEND/RETRACT explicitly.
 * - HDY_MODE_PRESSURE_CLOSED_LOOP:
 *   Uses targetPressure and targetFlow as the pressure reference and flow
 *   feedforward/cap inputs.
 */
typedef struct {
    char name[HDY_NAME_MAX];
    HDY_SegmentType type;
    HDY_PlannerType planner;
    HDY_ControlMode mode;
    HDY_EndConditionType endCondition;
    HDY_MotionDirection direction; /* Declared by process layer; planner never drives valves directly. */

    HDY_REAL targetPosition;
    HDY_REAL targetFlow;      /* L/min, mode-dependent setpoint/cap/feedforward */
    HDY_REAL targetPressure;
    HDY_REAL maxAcceleration;
    HDY_REAL maxVelocity;     /* mm/s, velocity magnitude limit */
    HDY_REAL maxFlow;         /* L/min, flow magnitude limit before pump conversion */
    HDY_TIME duration;        /* s, used by HDY_END_TIME */

    HDY_REAL tolerance;          /* Legacy generic tolerance fallback. Prefer the typed tolerances below. */
    HDY_REAL positionTolerance;  /* mm */
    HDY_REAL pressureTolerance;  /* MPa */
    HDY_REAL flowTolerance;      /* L/min */
    HDY_REAL velocityTolerance;  /* mm/s */
    HDY_TIME timeoutLimit;       /* s, 0 means disabled or auto-derived for time-ended segments */

    HDY_REAL velocityToFlowGain; /* L/min per mm/s, actuator flow gain */
    HDY_REAL pressureRampRate;   /* MPa/s, target pressure ramp rate */
} HDY_MotionSegment;

typedef struct {
    HDY_DiagnosticCode code;
    HDY_DiagnosticSeverity severity;
    HDY_BOOL overPressure;
    HDY_BOOL underPressure;
    HDY_BOOL flowDeviation;
    HDY_BOOL positionDeviation;
    HDY_BOOL velocityDeviation;
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
    HDY_BOOL faultActive;
    HDY_REAL plannedVelocity;      /* mm/s, signed */
    HDY_REAL plannedFlow;          /* L/min, nonnegative pump-side magnitude */
    HDY_REAL commandedPumpSpeed;   /* rpm, nonnegative */
    HDY_MotionDirection plannedDirection;
    HDY_ControllerStatus status;
    char currentSegmentName[HDY_NAME_MAX];
} HDY_MotionState;

#endif /* HDY_COMMON_TYPES_H */
