#ifndef HDY_MOTION_CONTROL_H
#define HDY_MOTION_CONTROL_H

#include "common_types.h"
#include "pressure_controller.h"
#include "ramp_controller.h"
#include "diagnostics_monitor.h"
#include "diagnostics_criteria.h"

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
 *   otherwise HDY_DIAG_CODE_NONE.
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
 * Direction semantics for HDY_MODE_POSITION / HDY_MODE_SPEED_RAMP:
 * - Each segment must declare EXTEND or RETRACT direction explicitly.
 * - The process layer still owns valve actions.
 * - This library therefore plans signed velocity internally, while PUMP_SPEED
 *   remains a nonnegative pump-side magnitude command.
 *
 * Pressure-control semantics for HDY_MODE_PRESSURE_CLOSED_LOOP:
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
 * - HDY_MODE_POSITION accepts POSITION_BASED or TIME_BASED planner selection.
 * - HDY_MODE_SPEED_RAMP always uses TIME_BASED velocity ramping; when combined
 *   with HDY_END_POSITION it only adds braking protection near targetPosition.
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
 * - HOLD: STARTING / RUNNING
 * - RESUME: HOLD only
 * - ABORT: STARTING / RUNNING / SEGMENT_COMPLETE / HOLD
 * - ACK: DISABLED / IDLE / READY / SEGMENT_COMPLETE / HOLD / DONE / ABORTED
 * - RESET: handled by RESET input and consumed on the next Cycle()/Scan()/Execute()
 * - STOP: still reserved and intentionally rejected until its state semantics are implemented
 *
 * Hold / Resume semantics in the current minimal skeleton:
 * - HOLD drives safe zero outputs, preserves the active segment context, and freezes
 *   elapsed segment time until RESUME is consumed.
 * - RESUME re-primes ramp / pressure-controller state from the current feedback and
 *   continues the same segment without advancing the recipe index.
 * - HOLD / RESUME are currently exposed through API calls, not dedicated BOOL inputs.
 */
typedef enum {
    HDY_CMD_NONE,
    HDY_CMD_START,
    HDY_CMD_NEXT,
    HDY_CMD_STOP,
    HDY_CMD_HOLD,
    HDY_CMD_RESUME,
    HDY_CMD_ABORT,
    HDY_CMD_RESET,
    HDY_CMD_ACK
} HDY_FbCommand;

typedef enum {
    HDY_FB_STATE_DISABLED,
    HDY_FB_STATE_IDLE,
    HDY_FB_STATE_READY,
    HDY_FB_STATE_STARTING,
    HDY_FB_STATE_RUNNING,
    HDY_FB_STATE_SEGMENT_COMPLETE,
    HDY_FB_STATE_HOLD,
    HDY_FB_STATE_DONE,
    HDY_FB_STATE_ABORTED,
    HDY_FB_STATE_FAULT
} HDY_FbState;

typedef struct {
    HDY_BOOL RESET;
    HDY_BOOL START_SEGMENT;
    HDY_UINT START_SEGMENT_INDEX;
    HDY_BOOL USE_RECIPE; /* true: start from RECIPE[index], false: start from DIRECT_SEGMENT */
    HDY_REAL FLOW_TO_PUMP_SPEED_GAIN;
    HDY_REAL PUMP_SPEED_LIMIT;
    HDY_UINT RECIPE_SIZE;
    HDY_AxisRef AXIS_REF;
    HDY_MotionSegment RECIPE[HDY_MAX_SEGMENTS];
    HDY_MotionSegment DIRECT_SEGMENT;

    HDY_DiagnosticCode ERROR_ID;
    HDY_FbState FB_STATE;
    HDY_REAL PUMP_SPEED;
    HDY_BOOL SEGMENT_COMPLETED;
    HDY_BOOL SEGMENT_CHANGED;
    HDY_MotionState STATE;      /* Unified runtime state: active, finished, faultActive, status, segmentTag, pressure-loop telemetry. */
    HDY_DiagnosticInfo DIAGNOSTIC;         /* Live current-cycle / current-command diagnostic. */
    HDY_DiagnosticSnapshot LAST_FAULT_SNAPSHOT;
    HDY_DiagnosticHistory DIAGNOSTIC_HISTORY;
    HDY_BOOL DIRECT_SEGMENT_VALID;

    /* Internal */
    HDY_REAL _segmentStartTime;
    HDY_BOOL _segmentChangedFlag;
    HDY_REAL _lastCommandedFlow;
    HDY_TIME _lastFeedbackTimestamp;       /* < 0.0 means invalid / never set */
    HDY_BOOL _startSegmentSignalPrev;
    HDY_FbCommand _pendingCommand;
    HDY_UINT _pendingCommandSegmentIndex;
    HDY_TIME _pendingCommandTimestamp;
    HDY_MotionSegment _activeSegment;
    HDY_BOOL _activeSegmentValid;
    HDY_SegmentSource _activeSegmentSource;
    HDY_TIME _holdStateTime;
    HDY_RampController _rampController;
    HDY_PressureControllerState _pressureController;

    /* Diagnostic criteria layer - unified through diagnostics_monitor + diagnostics_criteria
     * Supports: startup suppress, switch suppress, debounce, hysteresis, fault escalation. */
    HDY_ErrorMonitor _errorMonitor;
    HDY_DiagnosticCriteria _pressureCriteria;
    HDY_DiagnosticCriteriaState _pressureCriteriaState;
    HDY_DiagnosticCriteria _flowCriteria;
    HDY_DiagnosticCriteriaState _flowCriteriaState;
    HDY_DiagnosticCriteria _velocityCriteria;
    HDY_DiagnosticCriteriaState _velocityCriteriaState;
    HDY_DiagnosticCriteria _positionCriteria;
    HDY_DiagnosticCriteriaState _positionCriteriaState;
    HDY_DiagnosticCriteria _timeoutCriteria;
    HDY_DiagnosticCriteriaState _timeoutCriteriaState;
    HDY_BOOL _isSwitchPhase;            /* True during segment transition window for switch suppress */
    HDY_TIME _switchSuppressEndTime;    /* Elapsed time at which switch suppress phase expires */
    HDY_UINT8 _index;
    uint16_t _executionId;   /* incremented on BeginSegment success; IEC layer uses for ownership tracking */
    HDY_BOOL _useSimulation;           /* If true, the FB simulates motion without real hardware interaction for testing purposes. */
    struct {
        HDY_REAL targetPosition;
        HDY_REAL targetVelocity;
        HDY_REAL targetFlow;
        HDY_REAL targetPressure;
        HDY_BOOL valid;
    } _simFeedback;
} HDY_MotionControlFB;

/* Derived PLCopen-standard outputs — computed from FB_STATE / STATE / DIAGNOSTIC.
 * These replace the former stored BOOL fields (ACTIVE, BUSY, DONE, ERROR).
 * ACTIVE is read directly from STATE.active. */
static inline HDY_BOOL HDY_MotionControlFB_IsBusy(const HDY_MotionControlFB* fb) {
    if (fb == NULL) { return false; }
    switch (fb->FB_STATE) {
        case HDY_FB_STATE_STARTING:
        case HDY_FB_STATE_RUNNING:
        case HDY_FB_STATE_SEGMENT_COMPLETE:
        case HDY_FB_STATE_HOLD:
            return true;
        default:
            return false;
    }
}

static inline HDY_BOOL HDY_MotionControlFB_IsDone(const HDY_MotionControlFB* fb) {
    return (fb != NULL) && (fb->FB_STATE == HDY_FB_STATE_DONE);
}

static inline HDY_BOOL HDY_MotionControlFB_IsError(const HDY_MotionControlFB* fb) {
    if (fb == NULL) { return false; }
    return fb->STATE.faultActive ||
           fb->FB_STATE == HDY_FB_STATE_FAULT ||
           fb->DIAGNOSTIC.severity == HDY_DIAG_SEVERITY_FAULT;
}

/* Full reset of configuration, recipe, runtime state, and internal helpers. */
void HDY_MotionControlFB_Init(HDY_MotionControlFB* fb);

/*
 * Soft reset: clears runtime execution state, active segment, fault status,
 * diagnostic retention, and internal controllers, but preserves recipe,
 * configuration gains (FLOW_TO_PUMP_SPEED_GAIN, PUMP_SPEED_LIMIT, USE_RECIPE),
 * DIRECT_SEGMENT, and diagnostic criteria settings. After a soft reset the FB
 * enters READY (if a recipe/direct segment is loaded) or IDLE.
 * This is what RESET=true triggers in the cyclic entry points.
 */
void HDY_MotionControlFB_SoftReset(HDY_MotionControlFB* fb);

/*
 * Validates and loads a recipe into the function block.
 * This call only prepares the controller; it does not start execution and keeps
 * ACTIVE=false until StartSegment() / START_SEGMENT is consumed in Cycle()/Scan().
 */
HDY_BOOL HDY_MotionControlFB_LoadRecipe(HDY_MotionControlFB* fb, const HDY_MotionSegment* recipe, size_t recipeSize);

/*
 * Validates and stores the single direct-mode segment buffer.
 * This does not start execution and can coexist with an already loaded recipe.
 */
HDY_BOOL HDY_MotionControlFB_LoadDirectSegment(HDY_MotionControlFB* fb, const HDY_MotionSegment* segment);

/* Clears the stored direct-mode segment buffer without affecting recipe memory. */
void HDY_MotionControlFB_ClearDirectSegment(HDY_MotionControlFB* fb);

/*
 * Validates and queues a segment-start command.
 * When USE_RECIPE=true, segmentIndex addresses RECIPE[segmentIndex].
 * When USE_RECIPE=false, segmentIndex is ignored and DIRECT_SEGMENT is used.
 * The actual state transition occurs on the next Cycle()/Scan()/Execute().
 */
HDY_BOOL HDY_MotionControlFB_StartSegment(HDY_MotionControlFB* fb, size_t segmentIndex, HDY_TIME timestamp);

/*
 * Validates and queues an advance-to-next-segment command.
 * The actual transition occurs on the next Cycle()/Scan()/Execute().
 */
HDY_BOOL HDY_MotionControlFB_NextSegment(HDY_MotionControlFB* fb, HDY_TIME timestamp);

/* Queues a hold command for the current active segment. Safe zero outputs are applied on the next Cycle()/Scan()/Execute(). */
HDY_BOOL HDY_MotionControlFB_Hold(HDY_MotionControlFB* fb);

/* Queues a resume command for the currently held segment. Execution continues on the next Cycle()/Scan()/Execute(). */
HDY_BOOL HDY_MotionControlFB_Resume(HDY_MotionControlFB* fb);

/* Queues an abort command. Safe outputs are applied on the next Cycle()/Scan()/Execute(). */
HDY_BOOL HDY_MotionControlFB_Abort(HDY_MotionControlFB* fb);

/*
 * Clears retained diagnostic latch/snapshot/history after the live event has
 * cleared (typically on the next non-fault Cycle()/Scan()/Execute() in a hold
 * state). Fault-state retention is intentionally not clearable without RESET.
 */
HDY_BOOL HDY_MotionControlFB_AcknowledgeDiagnostics(HDY_MotionControlFB* fb);

/* Executes the already-sampled pending command and the explicit state machine. */
void HDY_MotionControlFB_Cycle(HDY_MotionControlFB* fb);

/* Samples edge-triggered command inputs (for example START_SEGMENT) then calls Cycle(). */
void HDY_MotionControlFB_Scan(HDY_MotionControlFB* fb);

/* Compatibility cyclic entry; currently equivalent to Scan(). */
void HDY_MotionControlFB_Execute(HDY_MotionControlFB* fb);

#endif /* HDY_MOTION_CONTROL_H */
