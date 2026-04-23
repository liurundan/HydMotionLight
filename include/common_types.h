#ifndef HDY_COMMON_TYPES_H
#define HDY_COMMON_TYPES_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "hdy_config.h"

/* ============================================================================
 * 基本类型定义
 * 从hdy_config.h继承HDY_REAL和HDY_TIME的精度配置
 * ============================================================================ */
typedef bool HDY_BOOL;
/* HDY_REAL已在hdy_config.h中定义 */
/* HDY_TIME已在hdy_config.h中定义 */
typedef uint8_t HDY_UINT8;
typedef uint16_t HDY_UINT16;
typedef size_t HDY_UINT;

/* ============================================================================
 * 常量定义
 * 从hdy_config.h继承各MAX值，支持平台裁剪
 * ============================================================================ */
/* HDY_MAX_SEGMENTS已在hdy_config.h中定义 */
/* HDY_NAME_MAX已在hdy_config.h中定义 */
/* HDY_DIAG_HISTORY_DEPTH已在hdy_config.h中定义 */

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
    HDY_SEGMENT_SOURCE_NONE,
    HDY_SEGMENT_SOURCE_RECIPE,
    HDY_SEGMENT_SOURCE_DIRECT
} HDY_SegmentSource;

typedef enum {
    HDY_STATUS_IDLE,
    HDY_STATUS_READY,
    HDY_STATUS_RUNNING,
    HDY_STATUS_HOLD,
    HDY_STATUS_DEGRADED,
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
    HDY_DIAG_SOURCE_NONE,
    HDY_DIAG_SOURCE_RECIPE,
    HDY_DIAG_SOURCE_RUNTIME,
    HDY_DIAG_SOURCE_COMMAND,
    HDY_DIAG_SOURCE_EXECUTION,
    HDY_DIAG_SOURCE_SENSOR,
    HDY_DIAG_SOURCE_INTERNAL
} HDY_DiagnosticSource;

typedef enum {
    HDY_DIAG_RECOVERY_NONE,
    HDY_DIAG_RECOVERY_AUTO_CLEAR,
    HDY_DIAG_RECOVERY_CHECK_COMMAND,
    HDY_DIAG_RECOVERY_CHECK_SENSOR,
    HDY_DIAG_RECOVERY_RELOAD_RECIPE,
    HDY_DIAG_RECOVERY_RESTART_SEGMENT,
    HDY_DIAG_RECOVERY_RESET_CONTROLLER
} HDY_DiagnosticRecovery;

typedef enum {
    HDY_PROTECTION_ACTION_NONE,
    HDY_PROTECTION_ACTION_WARNING,
    HDY_PROTECTION_ACTION_DERATE,
    HDY_PROTECTION_ACTION_STOP
} HDY_ProtectionAction;

typedef enum {
    HDY_DIAG_CODE_NONE,
    HDY_DIAG_CODE_RECIPE_EMPTY,
    HDY_DIAG_CODE_RECIPE_TOO_LARGE,
    HDY_DIAG_CODE_SEGMENT_INVALID,
    HDY_DIAG_CODE_RUNTIME_CONFIG_INVALID,
    HDY_DIAG_CODE_START_CONTEXT_INVALID,
    HDY_DIAG_CODE_COMMAND_NOT_ALLOWED,
    HDY_DIAG_CODE_NO_RECIPE,
    HDY_DIAG_CODE_NO_DIRECT_SEGMENT,
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
    HDY_DIAG_CODE_SENSOR_FAULT,
    HDY_DIAG_CODE_TIMESTAMP_ROLLBACK,
    HDY_DIAG_CODE_INTERNAL_ERROR
} HDY_DiagnosticCode;

typedef HDY_UINT16 HDY_DiagnosticFlags;

typedef enum {
    HDY_DIAG_FLAG_NONE = 0U,
    HDY_DIAG_FLAG_OVER_PRESSURE = 1U << 0,
    HDY_DIAG_FLAG_UNDER_PRESSURE = 1U << 1,
    HDY_DIAG_FLAG_FLOW_DEVIATION = 1U << 2,
    HDY_DIAG_FLAG_POSITION_DEVIATION = 1U << 3,
    HDY_DIAG_FLAG_VELOCITY_DEVIATION = 1U << 4,
    HDY_DIAG_FLAG_TIMEOUT = 1U << 5,
    HDY_DIAG_FLAG_SENSOR_FAULT = 1U << 6,
    HDY_DIAG_FLAG_TIMESTAMP_ROLLBACK = 1U << 7
} HDY_DiagnosticFlag;

typedef enum {
    HDY_PRESSURE_CONTROLLER_NONE,
    HDY_PRESSURE_CONTROLLER_P,
    HDY_PRESSURE_CONTROLLER_PI,
    HDY_PRESSURE_CONTROLLER_PID,
    HDY_PRESSURE_CONTROLLER_RBF_PID
} HDY_PressureControllerType;

typedef struct {
    HDY_REAL minKp;
    HDY_REAL maxKp;
    HDY_REAL minKi;
    HDY_REAL maxKi;
    HDY_REAL minKd;
    HDY_REAL maxKd;
    HDY_REAL etaW;
    HDY_REAL etaC;
    HDY_REAL etaB;
    HDY_REAL etaP;
    HDY_REAL etaI;
    HDY_REAL etaD;
} HDY_RbfPidConfig;

typedef struct {
    HDY_REAL targetPressure;
    HDY_REAL filteredPressure;
    HDY_REAL filteredPressureRate;
    HDY_REAL controlError;
    HDY_REAL feedforwardFlow;
    HDY_REAL feedbackFlow;
    HDY_REAL outputFlow;
    HDY_REAL unsaturatedOutputFlow;
    HDY_REAL samplingPeriod;
#if HDY_ENABLE_PRESSURE_LOOP_TELEMETRY
    HDY_REAL adaptiveKp;
    HDY_REAL adaptiveKi;
    HDY_REAL adaptiveKd;
    HDY_REAL adaptiveJacobian;
    HDY_BOOL trackingApplied;
    HDY_BOOL saturated;
    HDY_BOOL adaptiveActive;
#endif
} HDY_PressureLoopState;

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
 *   Uses targetPressure as the closed-loop reference and targetFlow as the
 *   feedforward bias / nominal hold flow.
 *
 * segmentTag:
 *   Replaces the former name[HDY_NAME_MAX] char array to save RAM on
 *   embedded targets. The tag is an opaque identifier assigned by the
 *   process layer; the library only stores and copies it. Tag-to-name
 *   resolution is the caller's responsibility (e.g. HMI lookup table).
 *   Value 0 means "not set" / anonymous.
 */
typedef struct {
    HDY_UINT8 segmentTag;
    HDY_SegmentType segmentType;
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

    /* Pressure-controller tuning. Zero values keep legacy-compatible defaults.
     * Adaptive RBF-PID reuses the generic filter/deadband envelope and exposes
     * additional bounded tuning / learning fields through pressureRbfConfig.
     */
    HDY_PressureControllerType pressureController;
    HDY_REAL pressureKp;                     /* L/min per MPa */
    HDY_REAL pressureKi;                     /* L/min per (MPa*s) */
    HDY_REAL pressureKd;                     /* L/min per (MPa/s) */
    HDY_REAL pressureIntegralLimit;          /* L/min absolute limit for integral contribution */
    HDY_REAL pressureDeadband;               /* MPa */
    HDY_REAL pressureFilterAlpha;            /* 0<alpha<=1, 1 means no measurement filtering */
    HDY_REAL pressureDerivativeFilterAlpha;  /* 0<alpha<=1, 1 means no derivative filtering */
    HDY_RbfPidConfig pressureRbfConfig;      /* Optional RBF-PID bounded tuning / learning profile. Zero uses library defaults. */
} HDY_MotionSegment;

typedef struct {
    HDY_DiagnosticCode code;
    HDY_DiagnosticSeverity severity;
    HDY_DiagnosticSource source;
    HDY_DiagnosticRecovery recovery;
    HDY_ProtectionAction protectionAction;
#if HDY_ENABLE_DIAGNOSTIC_FLAGS
    HDY_DiagnosticFlags flags;  /* Compact embedded-facing summary of active diagnostic conditions. */
#endif
    HDY_BOOL overPressure;
    HDY_BOOL underPressure;
    HDY_BOOL flowDeviation;
    HDY_BOOL positionDeviation;
    HDY_BOOL velocityDeviation;
    HDY_BOOL timeout;
    HDY_BOOL sensorFault;
    HDY_BOOL timestampRollback;
    HDY_REAL pressureError;
    HDY_REAL flowError;
    HDY_REAL velocityError;
} HDY_DiagnosticInfo;

typedef struct {
    HDY_REAL elapsedTime;
#if HDY_ENABLE_EXECUTION_REFERENCE
    HDY_REAL pressureReference;
    HDY_REAL flowReference;
    HDY_REAL velocityReference;
#endif
} HDY_ExecutionReference;

typedef struct {
    HDY_BOOL valid;
    HDY_TIME eventTimestamp;              /* Event capture time in seconds. */
    HDY_UINT8 segmentIndex;               /* HDY_MAX_SEGMENTS or larger means no active segment context. */
    HDY_ControllerStatus status;
    HDY_BOOL active;
    HDY_BOOL finished;
    HDY_BOOL fault;
    HDY_DiagnosticInfo diagnostic;
    HDY_AxisRef axisRef;
#if HDY_ENABLE_EXECUTION_REFERENCE
    HDY_ExecutionReference references;
#endif
    HDY_UINT8 segmentTag;
} HDY_DiagnosticSnapshot;

#if HDY_ENABLE_DIAGNOSTIC_HISTORY
typedef struct {
    HDY_DiagnosticSnapshot entries[HDY_DIAG_HISTORY_DEPTH];
    HDY_UINT8 count;
    HDY_UINT8 nextWriteIndex;
    HDY_UINT16 totalRecorded;
    HDY_BOOL wrapped;
} HDY_DiagnosticHistory;
#else
/* 禁用诊断历史时，使用最小化结构以节省RAM */
typedef struct {
    HDY_DiagnosticSnapshot entries[1];  /* 只保留一个当前快照 */
    HDY_UINT8 count;
    HDY_UINT8 nextWriteIndex;
    HDY_UINT16 totalRecorded;
    HDY_BOOL wrapped;
} HDY_DiagnosticHistory;
#endif

typedef struct {
    HDY_UINT currentSegmentIndex;
    HDY_BOOL active;
    HDY_BOOL finished;
    HDY_BOOL faultActive;
    HDY_REAL plannedVelocity;             /* mm/s, signed */
    HDY_REAL plannedFlow;                 /* L/min, nonnegative pump-side magnitude */
    HDY_REAL commandedPumpSpeed;          /* rpm, nonnegative */
#if HDY_ENABLE_EXECUTION_REFERENCE
    HDY_ExecutionReference references;    /* Current runtime reference bundle shared by diagnostics/completion/HMI */
#endif
    HDY_PressureControllerType pressureControllerApplied;
#if HDY_ENABLE_PRESSURE_LOOP_TELEMETRY
    HDY_PressureLoopState pressureLoop;    /* Embedded-facing pressure-loop telemetry / adaptive summary. */
#endif
    HDY_ProtectionAction protectionAction;
    HDY_MotionDirection plannedDirection;
    HDY_SegmentSource segmentSource;       /* Latched source of the active/last executed segment. */
    HDY_ControllerStatus status;
    HDY_UINT8 currentSegmentTag;
} HDY_MotionState;

/* ============================================================================
 * 公共工具函数
 * 这些函数在多个模块中使用，集中在此处避免代码重复
 * ============================================================================ */

/**
 * @brief 将值限制在 [minimum, maximum] 范围内
 * @param value 要限制的值
 * @param minimum 下界
 * @param maximum 上界
 * @return 限制后的值
 */
static inline HDY_REAL HDY_ClampReal(HDY_REAL value, HDY_REAL minimum, HDY_REAL maximum) {
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

#endif /* HDY_COMMON_TYPES_H */
