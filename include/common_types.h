#ifndef HYD_COMMON_TYPES_H
#define HYD_COMMON_TYPES_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "hyd_config.h"
#include "accessor.h"
#include "iec_types_all.h"
/* ============================================================================
 * 基本类型定义
 * 从hdy_config.h继承HDY_REAL和HDY_TIME的精度配置
 * ============================================================================ */
typedef bool HYD_BOOL;
/* HYD_REAL已在hdy_config.h中定义 */
/* HYD_TIME已在hdy_config.h中定义 */
typedef uint8_t HYD_UINT8;
typedef uint16_t HYD_UINT16;
typedef uint16_t HYD_UINT;

/* ============================================================================
 * 常量定义
 * 从hdy_config.h继承各MAX值，支持平台裁剪
 * ============================================================================ */
/* HYD_MAX_SEGMENTS已在hdy_config.h中定义 */
/* HYD_NAME_MAX已在hdy_config.h中定义 */

/* Enums shared between modules */
typedef enum {
    HYD_SEGMENT_TYPE_CLAMPING,
    HYD_SEGMENT_TYPE_OPENING,
    HYD_SEGMENT_TYPE_INJECTION,
    HYD_SEGMENT_TYPE_HOLDING,
    HYD_SEGMENT_TYPE_EJECTION,
    HYD_SEGMENT_TYPE_CORE_PULL,
    HYD_SEGMENT_TYPE_OTHER
} HYD_SegmentType;

typedef enum {
    HYD_PLANNER_POSITION_BASED,
    HYD_PLANNER_TIME_BASED
} HYD_PlannerType;

typedef enum {
    HYD_MODE_POSITION,
    HYD_MODE_SPEED_RAMP,
    HYD_MODE_PRESSURE_CLOSED_LOOP
} HYD_ControlMode;

typedef enum {
    HYD_END_POSITION,
    HYD_END_TIME,
    HYD_END_PRESSURE,
    HYD_END_FLOW,
    HYD_END_MANUAL
} HYD_EndConditionType;

typedef enum {
    HYD_DIRECTION_AUTO,
    HYD_DIRECTION_EXTEND,
    HYD_DIRECTION_RETRACT,
    HYD_DIRECTION_HOLD
} HYD_MotionDirection;

typedef enum {
    HYD_SEGMENT_SOURCE_NONE,
    HYD_SEGMENT_SOURCE_RECIPE,
    HYD_SEGMENT_SOURCE_DIRECT
} HYD_SegmentSource;

typedef enum {
    HYD_STATUS_IDLE,
    HYD_STATUS_READY,
    HYD_STATUS_RUNNING,
    HYD_STATUS_HOLD,
    HYD_STATUS_DEGRADED,
    HYD_STATUS_SEGMENT_COMPLETE,
    HYD_STATUS_FINISHED,
    HYD_STATUS_FAULT
} HYD_ControllerStatus;

typedef enum {
    HYD_DIAG_SEVERITY_NONE,
    HYD_DIAG_SEVERITY_INFO,
    HYD_DIAG_SEVERITY_WARNING,
    HYD_DIAG_SEVERITY_FAULT
} HYD_DiagnosticSeverity;

typedef enum {
    HYD_DIAG_SOURCE_NONE,
    HYD_DIAG_SOURCE_RECIPE,
    HYD_DIAG_SOURCE_RUNTIME,
    HYD_DIAG_SOURCE_COMMAND,
    HYD_DIAG_SOURCE_EXECUTION,
    HYD_DIAG_SOURCE_SENSOR,
    HYD_DIAG_SOURCE_INTERNAL
} HYD_DiagnosticSource;

typedef enum {
    HYD_DIAG_RECOVERY_NONE,
    HYD_DIAG_RECOVERY_AUTO_CLEAR,
    HYD_DIAG_RECOVERY_CHECK_COMMAND,
    HYD_DIAG_RECOVERY_CHECK_SENSOR,
    HYD_DIAG_RECOVERY_RELOAD_RECIPE,
    HYD_DIAG_RECOVERY_RESTART_SEGMENT,
    HYD_DIAG_RECOVERY_RESET_CONTROLLER
} HYD_DiagnosticRecovery;

typedef enum {
    HYD_PROTECTION_ACTION_NONE,
    HYD_PROTECTION_ACTION_WARNING,
    HYD_PROTECTION_ACTION_DERATE,
    HYD_PROTECTION_ACTION_STOP
} HYD_ProtectionAction;

typedef enum {
    HYD_VP_TRANSFER_REASON_NONE = 0,
    HYD_VP_TRANSFER_REASON_POSITION = 1,
    HYD_VP_TRANSFER_REASON_PRESSURE = 2,
    HYD_VP_TRANSFER_REASON_TIME = 3,
    HYD_VP_TRANSFER_REASON_VELOCITY_DROP = 4
} HYD_VpTransferReason;

typedef enum {
    HYD_DIAG_CODE_NONE,
    HYD_DIAG_CODE_RECIPE_EMPTY,
    HYD_DIAG_CODE_RECIPE_TOO_LARGE,
    HYD_DIAG_CODE_SEGMENT_INVALID,
    HYD_DIAG_CODE_RUNTIME_CONFIG_INVALID,
    HYD_DIAG_CODE_START_CONTEXT_INVALID,
    HYD_DIAG_CODE_COMMAND_NOT_ALLOWED,
    HYD_DIAG_CODE_NO_RECIPE,
    HYD_DIAG_CODE_NO_DIRECT_SEGMENT,
    HYD_DIAG_CODE_SEGMENT_INDEX_OUT_OF_RANGE,
    HYD_DIAG_CODE_SEGMENT_NOT_COMPLETED,
    HYD_DIAG_CODE_RECIPE_ALREADY_FINISHED,
    HYD_DIAG_CODE_ABORTED,
    HYD_DIAG_CODE_TIMEOUT,
    HYD_DIAG_CODE_OVER_PRESSURE,
    HYD_DIAG_CODE_UNDER_PRESSURE,
    HYD_DIAG_CODE_FLOW_DEVIATION,
    HYD_DIAG_CODE_POSITION_DEVIATION,
    HYD_DIAG_CODE_VELOCITY_DEVIATION,
    HYD_DIAG_CODE_SENSOR_FAULT,
    HYD_DIAG_CODE_TIMESTAMP_ROLLBACK,
    HYD_DIAG_CODE_INTERNAL_ERROR
} HYD_DiagnosticCode;

typedef HYD_UINT16 HYD_DiagnosticFlags;

typedef enum {
    HYD_DIAG_FLAG_NONE = 0U,
    HYD_DIAG_FLAG_OVER_PRESSURE = 1U << 0,
    HYD_DIAG_FLAG_UNDER_PRESSURE = 1U << 1,
    HYD_DIAG_FLAG_FLOW_DEVIATION = 1U << 2,
    HYD_DIAG_FLAG_POSITION_DEVIATION = 1U << 3,
    HYD_DIAG_FLAG_VELOCITY_DEVIATION = 1U << 4,
    HYD_DIAG_FLAG_TIMEOUT = 1U << 5,
    HYD_DIAG_FLAG_SENSOR_FAULT = 1U << 6,
    HYD_DIAG_FLAG_TIMESTAMP_ROLLBACK = 1U << 7
} HYD_DiagnosticFlag;

typedef enum {
    HYD_PRESSURE_CONTROLLER_NONE,
    HYD_PRESSURE_CONTROLLER_P,
    HYD_PRESSURE_CONTROLLER_PI,
    HYD_PRESSURE_CONTROLLER_PID,
    HYD_PRESSURE_CONTROLLER_RBF_PID
} HYD_PressureControllerType;

/* BufferMode values follow Beckhoff / PLCopen MC2 ordering.
 * ABORT (0): preempt current motion and execute immediately.
 * BUFFER (1): queue one following command after the active command.
 * BLENDING_* (2..5): queue one following command with a continuous transition
 * policy selected from the previous/next segment limits. */
typedef enum {
    HYD_BUFFER_MODE_ABORT = 0,
    HYD_BUFFER_MODE_BUFFER = 1,
    HYD_BUFFER_MODE_BLENDING_LOW = 2,
    HYD_BUFFER_MODE_BLENDING_PREVIOUS = 3,
    HYD_BUFFER_MODE_BLENDING_NEXT = 4,
    HYD_BUFFER_MODE_BLENDING_HIGH = 5
} HYD_BufferMode;

typedef struct {
    HYD_REAL minKp;
    HYD_REAL maxKp;
    HYD_REAL minKi;
    HYD_REAL maxKi;
    HYD_REAL minKd;
    HYD_REAL maxKd;
    HYD_REAL etaW;
    HYD_REAL etaC;
    HYD_REAL etaB;
    HYD_REAL etaP;
    HYD_REAL etaI;
    HYD_REAL etaD;
} HYD_RbfPidConfig;

typedef struct {
    HYD_REAL targetPressure;
    HYD_REAL filteredPressure;
    HYD_REAL filteredPressureRate;
    HYD_REAL controlError;
    HYD_REAL feedforwardFlow;
    HYD_REAL feedbackFlow;
    HYD_REAL outputFlow;
    HYD_REAL unsaturatedOutputFlow;
    HYD_REAL samplingPeriod;
#if HYD_ENABLE_PRESSURE_LOOP_TELEMETRY
    HYD_REAL adaptiveKp;
    HYD_REAL adaptiveKi;
    HYD_REAL adaptiveKd;
    HYD_REAL adaptiveJacobian;
    HYD_BOOL trackingApplied;
    HYD_BOOL saturated;
    HYD_BOOL adaptiveActive;
#endif
} HYD_PressureLoopState;

/* Data structures */
typedef struct {
    HYD_REAL position;   /* mm */
    HYD_REAL velocity;   /* mm/s, signed by mechanism direction */
    HYD_REAL flow;       /* L/min, magnitude at the pump side */
    HYD_REAL pressure;   /* MPa */
    HYD_TIME timestamp;  /* s */
} HYD_AxisRef;

/*
 * Motion-mode semantics:
 * - HYD_MODE_POSITION:
 *   Converges to targetPosition. planner selects either pure position-based
 *   braking or time-ramp buildup with position braking protection.
 * - HYD_MODE_SPEED_RAMP:
 *   Builds velocity magnitude by time ramp. The process layer still owns valve
 *   direction, so the segment must declare EXTEND/RETRACT explicitly.
 * - HYD_MODE_PRESSURE_CLOSED_LOOP:
 *   Uses targetPressure as the closed-loop reference and targetFlow as the
 *   feedforward bias / nominal hold flow.
 *
 * segmentTag:
 *   Replaces the former name[HYD_NAME_MAX] char array to save RAM on
 *   embedded targets. The tag is an opaque identifier assigned by the
 *   process layer; the library only stores and copies it. Tag-to-name
 *   resolution is the caller's responsibility (e.g. HMI lookup table).
 *   Value 0 means "not set" / anonymous.
 */
typedef struct {
    HYD_UINT8 segmentTag;
    HYD_SegmentType segmentType;
    HYD_PlannerType planner;
    HYD_ControlMode mode;
    HYD_EndConditionType endCondition;
    HYD_MotionDirection direction; /* Declared by process layer; planner never drives valves directly. */

    HYD_REAL targetPosition;
    HYD_REAL targetFlow;      /* L/min, mode-dependent setpoint/cap/feedforward */
    HYD_REAL targetPressure;
    HYD_REAL maxAcceleration;
    HYD_REAL maxDeceleration;
    HYD_REAL maxVelocity;     /* mm/s, velocity magnitude limit */
    HYD_REAL maxFlow;         /* L/min, flow magnitude limit before pump conversion */
    HYD_TIME duration;        /* s, used by HYD_END_TIME */

    HYD_REAL tolerance;          /* Legacy generic tolerance fallback. Prefer the typed tolerances below. */
    HYD_REAL positionTolerance;  /* mm */
    HYD_REAL pressureTolerance;  /* MPa */
    HYD_REAL flowTolerance;      /* L/min */
    HYD_REAL velocityTolerance;  /* mm/s */
    HYD_TIME timeoutLimit;       /* s, 0 means disabled or auto-derived for time-ended segments */
    HYD_TIME stableWindow;       /* s, 0 means immediate completion */
    HYD_REAL stableVelocityLimit; /* mm/s, 0 disables velocity-settled gate */
    HYD_REAL vpTransferPosition;        /* mm, 0 disables position transfer observation */
    HYD_REAL vpTransferPressure;        /* MPa, 0 disables pressure transfer observation */
    HYD_TIME vpTransferMinTime;         /* s, 0 disables elapsed-time transfer observation */
    HYD_REAL vpTransferVelocityDrop;    /* mm/s, 0 disables velocity-drop observation */

    HYD_REAL velocityToFlowGain; /* L/min per mm/s, actuator flow gain */
    HYD_REAL velocityKp;        /* L/min per mm/s, 0 disables velocity feedback correction */
    HYD_REAL velocityDeadband;  /* mm/s, 0 means no deadband */
    HYD_REAL velocityCorrectionLimit; /* L/min absolute correction limit, 0 uses maxFlow */
    HYD_REAL pressureRampRate;   /* MPa/s, target pressure ramp rate */

    /* Pressure-controller tuning. Zero values keep legacy-compatible defaults.
     * Adaptive RBF-PID reuses the generic filter/deadband envelope and exposes
     * additional bounded tuning / learning fields through pressureRbfConfig.
     */
    HYD_PressureControllerType pressureController;
    HYD_REAL pressureKp;                     /* L/min per MPa */
    HYD_REAL pressureKpHigh;                 /* L/min per MPa, high-error gain for scheduling; 0 disables */
    HYD_REAL pressureGainBand;               /* error ratio threshold for gain interpolation, 0 uses default 0.2 */
    HYD_REAL pressureKi;                     /* L/min per (MPa*s) */
    HYD_REAL pressureKd;                     /* L/min per (MPa/s) */
    HYD_REAL pressureIntegralLimit;          /* L/min absolute limit for integral contribution */
    HYD_REAL pressureDeadband;               /* MPa */
    HYD_REAL pressureFilterAlpha;            /* 0<alpha<=1, 1 means no measurement filtering */
    HYD_REAL pressureDerivativeFilterAlpha;  /* 0<alpha<=1, 1 means no derivative filtering */
    HYD_RbfPidConfig pressureRbfConfig;      /* Optional RBF-PID bounded tuning / learning profile. Zero uses library defaults. */
} HYD_MotionSegment;

typedef struct {
    HYD_DiagnosticCode code;
    HYD_DiagnosticSeverity severity;
    HYD_DiagnosticSource source;
    HYD_DiagnosticRecovery recovery;
    HYD_ProtectionAction protectionAction;
#if HYD_ENABLE_DIAGNOSTIC_FLAGS
    HYD_DiagnosticFlags flags;  /* Compact embedded-facing summary of active diagnostic conditions. */
#endif
    HYD_BOOL overPressure;
    HYD_BOOL underPressure;
    HYD_BOOL flowDeviation;
    HYD_BOOL positionDeviation;
    HYD_BOOL velocityDeviation;
    HYD_BOOL timeout;
    HYD_BOOL sensorFault;
    HYD_BOOL timestampRollback;
    HYD_REAL pressureError;
    HYD_REAL flowError;
    HYD_REAL velocityError;
} HYD_DiagnosticInfo;

typedef struct {
    HYD_REAL elapsedTime;
#if HYD_ENABLE_EXECUTION_REFERENCE
    HYD_REAL pressureReference;
    HYD_REAL flowReference;
    HYD_REAL velocityReference;
#endif
} HYD_ExecutionReference;

typedef struct {
    HYD_BOOL valid;
    HYD_TIME eventTimestamp;              /* Event capture time in seconds. */
    HYD_UINT8 segmentIndex;               /* HYD_MAX_SEGMENTS or larger means no active segment context. */
    HYD_ControllerStatus status;
    HYD_BOOL active;
    HYD_BOOL finished;
    HYD_BOOL fault;
    HYD_DiagnosticInfo diagnostic;
    HYD_AxisRef axisRef;
#if HYD_ENABLE_EXECUTION_REFERENCE
    HYD_ExecutionReference references;
#endif
    HYD_UINT8 segmentTag;
} HYD_DiagnosticSnapshot;

#if HYD_ENABLE_DIAGNOSTIC_HISTORY
/* Simplified diagnostic history: retains only the most recent snapshot
 * and a running event counter.  The former ring-buffer / multi-entry
 * design was removed to save RAM on embedded targets — production code
 * never read historical entries beyond the latest one.
 *
 * Semantics:
 *   lastSnapshot  – the most recently pushed diagnostic snapshot (or
 *                   zeroed if no event has been recorded yet).
 *   totalRecorded – cumulative count of all events since Init / Clear.
 *                   Useful as an "alarm counter" for HMI display.
 *   hasRecord     – true once at least one snapshot has been pushed.
 *                   Equivalent to the former count > 0 check.
 */
typedef struct {
    HYD_DiagnosticSnapshot lastSnapshot;
    HYD_UINT16 totalRecorded;
    HYD_BOOL hasRecord;
} HYD_DiagnosticHistory;
#else
/* 禁用诊断历史时，使用最小化结构以节省RAM */
typedef struct {
    HYD_DiagnosticSnapshot lastSnapshot;
    HYD_UINT16 totalRecorded;
    HYD_BOOL hasRecord;
} HYD_DiagnosticHistory;
#endif

typedef struct {
    HYD_UINT currentSegmentIndex;
    HYD_BOOL active;
    HYD_BOOL finished;
    HYD_BOOL faultActive;
    HYD_REAL plannedVelocity;             /* mm/s, signed */
    HYD_REAL plannedFlow;                 /* L/min, nonnegative pump-side magnitude */
    HYD_REAL commandedPumpSpeed;          /* rpm, nonnegative */
#if HYD_ENABLE_EXECUTION_REFERENCE
    HYD_ExecutionReference references;    /* Current runtime reference bundle shared by diagnostics/completion/HMI */
#endif
    HYD_PressureControllerType pressureControllerApplied;
#if HYD_ENABLE_PRESSURE_LOOP_TELEMETRY
    HYD_PressureLoopState pressureLoop;    /* Embedded-facing pressure-loop telemetry / adaptive summary. */
#endif
    HYD_ProtectionAction protectionAction;
    HYD_MotionDirection plannedDirection;
    HYD_SegmentSource segmentSource;       /* Latched source of the active/last executed segment. */
    HYD_ControllerStatus status;
    HYD_UINT8 currentSegmentTag;
    HYD_BOOL vpTransferReady;
    HYD_UINT8 vpTransferReason;
} HYD_MotionState;

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
static inline HYD_REAL HYD_ClampReal(HYD_REAL value, HYD_REAL minimum, HYD_REAL maximum) {
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

/* ============================================================================
 * FB Parameter Access — PARAMETERNUMBER → field mapping
 * Used by HYD_ReadParameter / HYD_WriteParameter / HYD_ReadBoolParameter / HYD_WriteBoolParameter
 * ============================================================================ */
typedef enum {
    HYD_PARAM_POSITION_TOLERANCE = 0,
    HYD_PARAM_VELOCITY_TOLERANCE,
    HYD_PARAM_FLOW_TOLERANCE,
    HYD_PARAM_PRESSURE_TOLERANCE,
    HYD_PARAM_TIMEOUT_LIMIT,
    HYD_PARAM_VELOCITY_TO_FLOW_GAIN,
    HYD_PARAM_MAX_VELOCITY,
    HYD_PARAM_MAX_ACCELERATION,
    HYD_PARAM_MAX_DECELERATION,
    HYD_PARAM_MAX_FLOW,
    HYD_PARAM_PRESSURE_RAMP_RATE,
    HYD_PARAM_PRESSURE_KP,
    HYD_PARAM_PRESSURE_KP_HIGH,
    HYD_PARAM_PRESSURE_GAIN_BAND,
    HYD_PARAM_PRESSURE_KI,
    HYD_PARAM_PRESSURE_KD,
    HYD_PARAM_PRESSURE_INTEGRAL_LIMIT,
    HYD_PARAM_PRESSURE_DEADBAND,
    HYD_PARAM_PRESSURE_FILTER_ALPHA,
    HYD_PARAM_PRESSURE_DERIVATIVE_FILTER_ALPHA,
    HYD_PARAM_VELOCITY_KP,
    HYD_PARAM_VELOCITY_DEADBAND,
    HYD_PARAM_VELOCITY_CORRECTION_LIMIT,
    HYD_PARAM_FLOW_TO_PUMP_SPEED_GAIN,
    HYD_PARAM_PUMP_SPEED_LIMIT,
    HYD_PARAM_PRESSURE_CONTROLLER_TYPE,
    HYD_PARAM_DEFAULT_TARGET_FLOW,
    HYD_PARAM_USE_SIMULATION,
    HYD_PARAM_COUNT
} HYD_ParameterNumber;

typedef struct {
    HYD_REAL positionTolerance;
    HYD_REAL velocityTolerance;
    HYD_REAL flowTolerance;
    HYD_REAL pressureTolerance;
    HYD_REAL timeoutLimit;
    HYD_REAL velocityToFlowGain;
    HYD_REAL maxVelocity;
    HYD_REAL maxAcceleration;
    HYD_REAL maxDeceleration;
    HYD_REAL maxFlow;
    HYD_REAL pressureRampRate;
    HYD_REAL pressureKp;
    HYD_REAL pressureKpHigh;
    HYD_REAL pressureGainBand;
    HYD_REAL pressureKi;
    HYD_REAL pressureKd;
    HYD_REAL pressureIntegralLimit;
    HYD_REAL pressureDeadband;
    HYD_REAL pressureFilterAlpha;
    HYD_REAL pressureDerivativeFilterAlpha;
    HYD_REAL velocityKp;
    HYD_REAL velocityDeadband;
    HYD_REAL velocityCorrectionLimit;
    HYD_REAL flowToPumpSpeedGain;
    HYD_REAL pumpSpeedLimit;
    HYD_REAL pressureControllerType;
    HYD_REAL defaultTargetFlow;
    HYD_BOOL useSimulation;
} HYD_MotionFBParams;

#endif /* HYD_COMMON_TYPES_H */
