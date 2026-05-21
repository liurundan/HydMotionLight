#ifndef HYD_MOTION_CONTROL_H
#define HYD_MOTION_CONTROL_H

#include "common_types.h"
#include "pressure_controller.h"
#include "ramp_controller.h"
#include "diagnostics_monitor.h"
#include "diagnostics_criteria.h"
#include "motion_planner.h"

/*
 * PLCopen-style motion control function block lifecycle:
 *   Init -> LoadRecipe -> StartSegment / START_SEGMENT -> Cycle / Scan -> Complete / Abort
 *
 * The EN/ENO gate is handled by the IEC layer (motion_interface.c). This core FB
 * assumes the caller has already performed the enable check; it does not store EN/ENO.
 *
 * Signal semantics:
 * - RESET=true:
 *   The next Cycle()/Scan()/Execute() performs a soft reset: it clears
 *   runtime state, active segment, fault/diagnostic status, and returns
 *   the FB to READY (if recipe/direct is loaded) or IDLE. Recipe contents,
 *   configuration gains, DIRECT_SEGMENT, and diagnostic criteria settings
 *   are preserved. For a full reinitialization that clears everything,
 *   call Init() explicitly.
 * - STATE.active:
 *   True only while an already started segment is executing in the current cycle.
 *   LoadRecipe() alone never sets STATE.active=true.
 * - IsBusy():
 *   Derived from FB_STATE. True while a started motion context is still owned by
 *   the FB (STARTING / RUNNING / HOLD / SEGMENT_COMPLETE). False in READY / IDLE /
 *   DISABLED and after terminal DONE / ABORTED / FAULT outcomes.
 * - IsDone():
 *   Derived from FB_STATE == DONE. Intentionally separate from SEGMENT_COMPLETED /
 *   STATE.finished so Abort() and fault paths never raise DONE.
 * - IsError() / ERROR_ID:
 *   IsError() is derived from STATE.faultActive, FB_STATE, and DIAGNOSTIC.severity.
 *   ERROR_ID mirrors the active fault diagnostic code when IsError() is true,
 *   otherwise HYD_DIAG_CODE_NONE.
 * - STATE.finished:
 *   True after the last segment reaches its end condition or after Abort().
 * - STATE.faultActive:
 *   Asserted when execution enters a protected stop state such as timeout or
 *   runtime configuration corruption.
 * - STATE.status:
 *   Aggregated controller status output for HMI / upper-layer integration.
 * - FB_STATE:
 *   Explicit framework-layer state machine output used to distinguish READY /
 *   RUNNING / SEGMENT_COMPLETE / DONE / ABORTED / FAULT / DISABLED states.
 * - SEGMENT_COMPLETED:
 *   Latched true when the active segment reaches its end condition. For middle
 *   segments it stays true until the caller advances with NextSegment() or starts
 *   another segment explicitly. For the last segment it is raised together with
 *   STATE.finished=true.
 * - SEGMENT_CHANGED:
 *   One-cycle pulse asserted on the first Cycle()/Scan()/Execute() after a
 *   successful StartSegment() / START_SEGMENT or NextSegment() transition.
 * - DIAGNOSTIC:
 *   Live diagnostic result for the current command/cycle. In non-fault idle/
 *   finished/disabled hold states it auto-clears on the next Cycle()/Scan()/
 *   Execute(), while retention remains available through DIAGNOSTIC_HISTORY.
 * - DIAGNOSTIC_HISTORY:
 *   Bounded diagnostic-retention output for commissioning and service. Keeps
 *   the last snapshot and a running event counter until Init()/RESET or
 *   an explicit AcknowledgeDiagnostics() after the live event has cleared.
 * - LAST_FAULT_SNAPSHOT:
 *   Captures the most recent fault-state transition with axis feedback,
 *   references, segment index/name, and controller status for root-cause review.
 *
 * Direction semantics for HYD_MODE_POSITION / HYD_MODE_SPEED_RAMP:
 * - Each segment must declare EXTEND or RETRACT direction explicitly.
 * - The process layer still owns valve actions.
 * - This library therefore plans signed velocity internally, while PUMP_SPEED
 *   remains a nonnegative pump-side magnitude command.
 *
 * Pressure-control semantics for HYD_MODE_PRESSURE_CLOSED_LOOP:
 * - targetPressure is first smoothed by RampController.
 * - targetFlow acts as the feedforward bias / nominal holding flow.
 * - pressureController selects P / PI / PID / RBF-PID strategy; PI/PID keep
 *   classical integral/derivative state in the FB, while RBF-PID keeps its
 *   adaptive learning state there for cycle-to-cycle continuity.
 * - STATE.pressureLoop exposes the currently applied pressure-loop telemetry
 *   (filtered pressure, controller contribution, saturation, adaptive gains)
 *   so HMI / commissioning code can inspect closed-loop behavior uniformly.
 *
 * Mode contract:
 * - HYD_MODE_POSITION accepts POSITION_BASED or TIME_BASED planner selection.
 * - HYD_MODE_SPEED_RAMP always uses TIME_BASED velocity ramping; when combined
 *   with HYD_END_POSITION it only adds braking protection near targetPosition.
 * - Typed tolerances (position/pressure/flow/velocity) should be configured in
 *   new recipes; the legacy generic tolerance field is kept only as fallback.
 *
 * Segment-parameter source contract:
 * - USE_RECIPE=true:
 *   START uses RECIPE[segmentIndex] and preserves the current multi-segment
 *   semantics with NextSegment().
 * - USE_RECIPE=false:
 *   START ignores segmentIndex and locks DIRECT_SEGMENT as a single direct-run
 *   action. Direct mode finishes after that one segment; NextSegment() is not
 *   meaningful there.
 * - RECIPE and DIRECT_SEGMENT may coexist in memory. The active segment always
 *   latches the source selected at start time, so changing USE_RECIPE or editing
 *   DIRECT_SEGMENT while running does not disturb the current execution.
 *
 * Current command legality matrix (framework-layer contract):
 * - START: IDLE / READY / SEGMENT_COMPLETE / DONE / ABORTED
 * - NEXT: SEGMENT_COMPLETE only
 * - STOP: STARTING / RUNNING
 * - HOLD: STARTING / RUNNING
 * - RESUME: HOLD only
 * - ABORT: IDLE / READY / STARTING / RUNNING / SEGMENT_COMPLETE / HOLD / DONE / ABORTED / FAULT
 * - ACK: DISABLED / IDLE / READY / SEGMENT_COMPLETE / HOLD / DONE / ABORTED
 * - RESET: handled by RESET input and consumed on the next Cycle()/Scan()/Execute()
 *
 * Execution identity contract (IEC adapter ownership tracking):
 * The FB exposes two distinct uint16_t epochs that the IEC adapter
 * (motion_interface.c) uses to detect command takeover.
 * - _executionId:
 *   Per-segment execution epoch. Advances on EVERY successful
 *   HYD_BeginSegment, including recipe NextSegment. Used by
 *   direct-command paths (MoveAbsolute / MoveVelocity / PressureHandle /
 *   Stop) where each new segment represents a brand-new direct session.
 *   NOT used by the recipe-side MoveProfile FB.
 * - _recipeBatchId:
 *   Recipe-side batch identity. Advances on:
 *     - HYD_CMD_START consumption (initial recipe Start)
 *     - HYD_CMD_STOP consumption (external Stop preemption)
 *     - HYD_AbortNow() (external Abort)
 *     - HYD_StartPendingDirectSlot() (direct takeover of an active recipe)
 *     - SoftReset memset zeroes it (a re-issued Start then advances it to 1)
 *   Does NOT advance on recipe NextSegment. The IEC MoveProfile adapter
 *   latches _recipeBatchId into its _EXEC_ID field and uses batch-id
 *   mismatch as the trigger for COMMANDABORTED, which preserves ownership
 *   stability across multi-segment recipe progression.
 *

 * Hold / Resume semantics in the current minimal skeleton:
 * - HOLD drives safe zero outputs, preserves the active segment context, and freezes
 *   elapsed segment time until RESUME is consumed.
 * - RESUME re-primes ramp / pressure-controller state from the current feedback and
 *   continues the same segment without advancing the recipe index.
 * - HOLD / RESUME are exposed through API calls and IEC adapter command wrappers.
 */
typedef enum {
    HYD_CMD_NONE,
    HYD_CMD_START,
    HYD_CMD_NEXT,
    HYD_CMD_STOP,
    HYD_CMD_HOLD,
    HYD_CMD_RESUME,
    HYD_CMD_ABORT,
    HYD_CMD_RESET,
    HYD_CMD_ACK
} HYD_FbCommand;

typedef enum {
    HYD_FB_STATE_DISABLED,
    HYD_FB_STATE_IDLE,
    HYD_FB_STATE_READY,
    HYD_FB_STATE_STARTING,
    HYD_FB_STATE_RUNNING,
    HYD_FB_STATE_SEGMENT_COMPLETE,
    HYD_FB_STATE_HOLD,
    HYD_FB_STATE_DONE,
    HYD_FB_STATE_ABORTED,
    HYD_FB_STATE_FAULT
} HYD_FbState;

typedef enum {
    HYD_DIRECT_CMD_NONE = 0,
    HYD_DIRECT_CMD_MOVE_ABSOLUTE,
    HYD_DIRECT_CMD_MOVE_VELOCITY,
    HYD_DIRECT_CMD_PRESSURE_HANDLE,
    HYD_DIRECT_CMD_STOP
} HYD_DirectCommandKind;

typedef enum {
    HYD_DIRECT_SESSION_IDLE = 0,
    HYD_DIRECT_SESSION_RUNNING,
    HYD_DIRECT_SESSION_STOPPING,
    HYD_DIRECT_SESSION_DONE,
    HYD_DIRECT_SESSION_ABORTED,
    HYD_DIRECT_SESSION_FAULT
} HYD_DirectSessionState;

typedef enum {
    HYD_LIVE_UPDATE_TARGET_POSITION = 1U << 0,
    HYD_LIVE_UPDATE_MAX_VELOCITY = 1U << 1,
    HYD_LIVE_UPDATE_ACCELERATION = 1U << 2,
    HYD_LIVE_UPDATE_DECELERATION = 1U << 3,
    HYD_LIVE_UPDATE_TARGET_PRESSURE = 1U << 4,
    HYD_LIVE_UPDATE_PRESSURE_RAMP_RATE = 1U << 5
} HYD_LiveUpdateFlags;

typedef struct {
    HYD_UINT16 flags;
    HYD_DirectCommandKind ownerKind;
    uint16_t ownerExecutionId;
    HYD_REAL targetPosition;
    HYD_REAL maxVelocity;
    HYD_REAL maxAcceleration;
    HYD_REAL maxDeceleration;
    HYD_REAL targetPressure;
    HYD_REAL pressureRampRate;
} HYD_LiveUpdateRequest;

typedef struct {
    HYD_BOOL RESET;
    HYD_BOOL START_SEGMENT;
    HYD_UINT START_SEGMENT_INDEX;
    HYD_BOOL USE_RECIPE; /* true: start from RECIPE[index], false: start from DIRECT_SEGMENT */
    HYD_REAL FLOW_TO_PUMP_SPEED_GAIN;
    HYD_REAL PUMP_SPEED_LIMIT;
    HYD_UINT RECIPE_SIZE;
    HYD_AxisRef AXIS_REF;
    HYD_MotionSegment RECIPE[HYD_MAX_SEGMENTS];
    HYD_MotionSegment DIRECT_SEGMENT;

    HYD_DiagnosticCode ERROR_ID;
    HYD_FbState FB_STATE;
    HYD_REAL PUMP_SPEED;
    HYD_BOOL SEGMENT_COMPLETED;
    HYD_BOOL SEGMENT_CHANGED;
    HYD_MotionState STATE;      /* Unified runtime state: active, finished, faultActive, status, segmentTag, pressure-loop telemetry. */
    HYD_DiagnosticInfo DIAGNOSTIC;         /* Live current-cycle / current-command diagnostic. */
    HYD_DiagnosticSnapshot LAST_FAULT_SNAPSHOT;
    HYD_DiagnosticHistory DIAGNOSTIC_HISTORY;
    HYD_BOOL DIRECT_SEGMENT_VALID;

    /* Internal */
    HYD_REAL _segmentStartTime;
    HYD_BOOL _segmentChangedFlag;
    HYD_REAL _lastCommandedFlow;
    HYD_TIME _lastFeedbackTimestamp;       /* < 0.0 means invalid / never set */
    HYD_BOOL _startSegmentSignalPrev;
    HYD_FbCommand _pendingCommand;
    HYD_UINT _pendingCommandSegmentIndex;
    HYD_TIME _pendingCommandTimestamp;
    HYD_MotionSegment _activeSegment;
    HYD_BOOL _activeSegmentValid;
    HYD_SegmentSource _activeSegmentSource;
    HYD_TIME _holdStateTime;
    HYD_RampController _rampController;
    HYD_MotionPlannerState _plannerState;
    HYD_PressureControllerState _pressureController;
    HYD_TIME _completionCandidateStartTime;
    HYD_BOOL _completionCandidateActive;
    HYD_BOOL _isDecelerating;
    HYD_TIME _decelStartTime;
    HYD_REAL _decelStartVel;
    HYD_DirectCommandKind _directOwnerKind;
    HYD_DirectSessionState _directSessionState;
    uint16_t _directOwnerExecutionId;
    uint16_t _lastPreemptedExecutionId;
    HYD_DirectCommandKind _lastPreemptedKind;
    uint16_t _lastCompletedExecutionId;
    HYD_DirectCommandKind _lastCompletedKind;
    HYD_BOOL _directPendingValid;
    HYD_MotionSegment _directPendingSegment;
    HYD_DirectCommandKind _directPendingKind;
    HYD_BufferMode _directPendingBufferMode;
    HYD_MotionBlendContext _directBlendContext;
    HYD_BOOL _isStopping;
    HYD_TIME _stopStartTime;
    HYD_REAL _stopStartVel;
    HYD_REAL _stopDeceleration;

    /* Diagnostic criteria layer - unified through diagnostics_monitor + diagnostics_criteria
     * Supports: startup suppress, switch suppress, debounce, hysteresis, fault escalation. */
    HYD_ErrorMonitor _errorMonitor;
    HYD_DiagnosticCriteria _pressureCriteria;
    HYD_DiagnosticCriteriaState _pressureCriteriaState;
    HYD_DiagnosticCriteria _flowCriteria;
    HYD_DiagnosticCriteriaState _flowCriteriaState;
    HYD_DiagnosticCriteria _velocityCriteria;
    HYD_DiagnosticCriteriaState _velocityCriteriaState;
    HYD_DiagnosticCriteria _positionCriteria;
    HYD_DiagnosticCriteriaState _positionCriteriaState;
    HYD_DiagnosticCriteria _timeoutCriteria;
    HYD_DiagnosticCriteriaState _timeoutCriteriaState;
    HYD_BOOL _isSwitchPhase;            /* True during segment transition window for switch suppress */
    HYD_TIME _switchSuppressEndTime;    /* Elapsed time at which switch suppress phase expires */
    HYD_UINT8 _index;
    uint16_t _executionId;   /* Per-segment execution epoch. Advances on every successful HYD_BeginSegment, including recipe NextSegment. Used by direct-command ownership tracking. NOT used by IEC MoveProfile adapter — see _recipeBatchId. */
    uint16_t _recipeBatchId; /* Per-recipe-batch epoch. Advances ONLY on initial Start, external ABORT/STOP, Reset, and direct takeover. Does NOT advance on recipe NextSegment. Used by IEC MoveProfile adapter to detect external recipe takeover separately from per-segment progress. */
    HYD_BOOL _useSimulation;           /* If true, the FB simulates motion without real hardware interaction for testing purposes. */
    HYD_BOOL _configuredUseRecipe;     /* Stable preload/source preference from axis setup; not the transient start selector. */
    HYD_MotionFBParams _params;        /* Tunable parameter defaults for segment builders */
    struct {
        HYD_REAL targetPosition;
        HYD_REAL targetVelocity;
        HYD_REAL targetFlow;
        HYD_REAL targetPressure;
        HYD_BOOL valid;
    } _simFeedback;
} HYD_MotionControlFB;

/* Derived PLCopen-standard outputs — computed from FB_STATE / STATE / DIAGNOSTIC.
 * These replace the former stored BOOL fields (ACTIVE, BUSY, DONE, ERROR).
 * ACTIVE is read directly from STATE.active. */
static inline HYD_BOOL HYD_MotionControlFB_IsBusy(const HYD_MotionControlFB* fb) {
    if (fb == NULL) { return false; }
    switch (fb->FB_STATE) {
        case HYD_FB_STATE_STARTING:
        case HYD_FB_STATE_RUNNING:
        case HYD_FB_STATE_SEGMENT_COMPLETE:
        case HYD_FB_STATE_HOLD:
            return true;
        default:
            return false;
    }
}

static inline HYD_BOOL HYD_MotionControlFB_IsDone(const HYD_MotionControlFB* fb) {
    return (fb != NULL) && (fb->FB_STATE == HYD_FB_STATE_DONE);
}

static inline HYD_BOOL HYD_MotionControlFB_IsError(const HYD_MotionControlFB* fb) {
    if (fb == NULL) { return false; }
    return fb->STATE.faultActive ||
           fb->FB_STATE == HYD_FB_STATE_FAULT ||
           fb->DIAGNOSTIC.severity == HYD_DIAG_SEVERITY_FAULT;
}

/* Full reset of configuration, recipe, runtime state, and internal helpers. */
void HYD_MotionControlFB_Init(HYD_MotionControlFB* fb);

/*
 * Soft reset: clears runtime execution state, active segment, fault status,
 * diagnostic retention, and internal controllers, but preserves recipe,
 * configuration gains (FLOW_TO_PUMP_SPEED_GAIN, PUMP_SPEED_LIMIT, USE_RECIPE),
 * DIRECT_SEGMENT, and diagnostic criteria settings. After a soft reset the FB
 * enters READY (if a recipe/direct segment is loaded) or IDLE.
 * This is what RESET=true triggers in the cyclic entry points.
 */
void HYD_MotionControlFB_SoftReset(HYD_MotionControlFB* fb);

/*
 * Validates and loads a recipe into the function block.
 * This call only prepares the controller; it does not start execution and keeps
 * ACTIVE=false until StartSegment() / START_SEGMENT is consumed in Cycle()/Scan().
 */
HYD_BOOL HYD_MotionControlFB_LoadRecipe(HYD_MotionControlFB* fb, const HYD_MotionSegment* recipe, size_t recipeSize);

/*
 * Validates and stores the single direct-mode segment buffer.
 * This does not start execution and can coexist with an already loaded recipe.
 */
HYD_BOOL HYD_MotionControlFB_LoadDirectSegment(HYD_MotionControlFB* fb, const HYD_MotionSegment* segment);

/* Clears the stored direct-mode segment buffer without affecting recipe memory. */
void HYD_MotionControlFB_ClearDirectSegment(HYD_MotionControlFB* fb);

/*
 * Validates and queues a segment-start command.
 * When USE_RECIPE=true, segmentIndex addresses RECIPE[segmentIndex].
 * When USE_RECIPE=false, segmentIndex is ignored and DIRECT_SEGMENT is used.
 * The actual state transition occurs on the next Cycle()/Scan()/Execute().
 */
HYD_BOOL HYD_MotionControlFB_StartSegment(HYD_MotionControlFB* fb, size_t segmentIndex, HYD_TIME timestamp);

/*
 * Validates and queues an advance-to-next-segment command.
 * The actual transition occurs on the next Cycle()/Scan()/Execute().
 */
HYD_BOOL HYD_MotionControlFB_NextSegment(HYD_MotionControlFB* fb, HYD_TIME timestamp);

/* Queues a hold command for the current active segment. Safe zero outputs are applied on the next Cycle()/Scan()/Execute(). */
HYD_BOOL HYD_MotionControlFB_Hold(HYD_MotionControlFB* fb);

/* Queues a resume command for the currently held segment. Execution continues on the next Cycle()/Scan()/Execute(). */
HYD_BOOL HYD_MotionControlFB_Resume(HYD_MotionControlFB* fb);

/* Queues a decelerating stop command. The stop profile is executed on the next Cycle()/Scan()/Execute(). */
HYD_BOOL HYD_MotionControlFB_Stop(HYD_MotionControlFB* fb,
                                  HYD_TIME timestamp,
                                  HYD_REAL deceleration);

/* Queues an abort command. Safe outputs are applied on the next Cycle()/Scan()/Execute(). */
HYD_BOOL HYD_MotionControlFB_Abort(HYD_MotionControlFB* fb);

/*
 * Clears retained diagnostic latch/snapshot/history after the live event has
 * cleared (typically on the next non-fault Cycle()/Scan()/Execute() in a hold
 * state). Fault-state retention is intentionally not clearable without RESET.
 */
HYD_BOOL HYD_MotionControlFB_AcknowledgeDiagnostics(HYD_MotionControlFB* fb);

HYD_DirectCommandKind HYD_MotionControlFB_GetDirectOwnerKind(const HYD_MotionControlFB* fb);
HYD_DirectSessionState HYD_MotionControlFB_GetDirectSessionState(const HYD_MotionControlFB* fb);
uint16_t HYD_MotionControlFB_GetDirectOwnerExecutionId(const HYD_MotionControlFB* fb);
HYD_BOOL HYD_MotionControlFB_WasExecutionPreempted(const HYD_MotionControlFB* fb,
                                                   uint16_t executionId,
                                                   HYD_DirectCommandKind kind);
HYD_BOOL HYD_MotionControlFB_WasExecutionCompleted(const HYD_MotionControlFB* fb,
                                                   uint16_t executionId,
                                                   HYD_DirectCommandKind kind);
HYD_BOOL HYD_MotionControlFB_ConsumeExecutionCompleted(HYD_MotionControlFB* fb,
                                                       uint16_t executionId,
                                                       HYD_DirectCommandKind kind);
HYD_BOOL HYD_MotionControlFB_ApplyLiveUpdate(HYD_MotionControlFB* fb,
                                             const HYD_LiveUpdateRequest* request);
HYD_BOOL HYD_MotionControlFB_StartDirectCommand(HYD_MotionControlFB* fb,
                                                const HYD_MotionSegment* segment,
                                                HYD_BufferMode bufferMode,
                                                HYD_TIME timestamp);

/* Executes the already-sampled pending command and the explicit state machine. */
void HYD_MotionControlFB_Cycle(HYD_MotionControlFB* fb);

/* Samples edge-triggered command inputs (for example START_SEGMENT) then calls Cycle(). */
void HYD_MotionControlFB_Scan(HYD_MotionControlFB* fb);

/* Compatibility cyclic entry; currently equivalent to Scan(). */
void HYD_MotionControlFB_Execute(HYD_MotionControlFB* fb);

/* Parameter accessors for IEC Read/Write Parameter FBs.
 * Return false on out-of-range paramNumber or type mismatch. */
HYD_BOOL HYD_MotionControlFB_ReadParameter(const HYD_MotionControlFB* fb, int paramNumber, HYD_REAL* value);
HYD_BOOL HYD_MotionControlFB_WriteParameter(HYD_MotionControlFB* fb, int paramNumber, HYD_REAL value);
HYD_BOOL HYD_MotionControlFB_ReadBoolParameter(const HYD_MotionControlFB* fb, int paramNumber, HYD_BOOL* value);
HYD_BOOL HYD_MotionControlFB_WriteBoolParameter(HYD_MotionControlFB* fb, int paramNumber, HYD_BOOL value);

#endif /* HYD_MOTION_CONTROL_H */
