#ifndef HDY_MOTION_CONTROL_H
#define HDY_MOTION_CONTROL_H

#include "common_types.h"
#include "ramp_controller.h"

/*
 * PLCopen-style motion control function block lifecycle:
 *   Init -> LoadRecipe -> StartSegment / START_SEGMENT -> Execute -> Complete / Abort
 *
 * Signal semantics:
 * - EN=false:
 *   Immediately applies safe zero outputs, forces ACTIVE=false, sets ENO=false,
 *   and clears any pending START_SEGMENT command. Re-enabling does not resume
 *   motion automatically; the caller must issue StartSegment() or START_SEGMENT again.
 * - RESET=true:
 *   The next Execute() performs a full reinitialization equivalent to Init().
 *   This clears runtime state, recipe contents, and configuration gains, so the
 *   caller must reload configuration and recipe afterwards.
 * - ACTIVE:
 *   True only while an already started segment is executing in the current cycle.
 *   LoadRecipe() alone never sets ACTIVE=true.
 * - FINISHED:
 *   True after the last segment reaches its end condition or after Abort().
 * - FAULT:
 *   Reserved embedded-facing fault output. It is asserted when execution enters
 *   a protected stop state such as timeout or runtime configuration corruption.
 * - STATUS:
 *   Aggregated controller status output for HMI / upper-layer integration.
 * - SEGMENT_COMPLETED:
 *   Latched true when the active segment reaches its end condition. For middle
 *   segments it stays true until the caller advances with NextSegment() or starts
 *   another segment explicitly. For the last segment it is raised together with
 *   FINISHED=true.
 * - SEGMENT_CHANGED:
 *   One-cycle pulse asserted on the first Execute() after StartSegment() or a
 *   successful NextSegment() transition.
 *
 * Direction semantics for HDY_MODE_POSITION / HDY_MODE_SPEED_RAMP:
 * - Each segment must declare EXTEND or RETRACT direction explicitly.
 * - The process layer still owns valve actions.
 * - This library therefore plans signed velocity internally, while PUMP_SPEED
 *   remains a nonnegative pump-side magnitude command.
 *
 * Mode contract:
 * - HDY_MODE_POSITION accepts POSITION_BASED or TIME_BASED planner selection.
 * - HDY_MODE_SPEED_RAMP always uses TIME_BASED velocity ramping; when combined
 *   with HDY_END_POSITION it only adds braking protection near targetPosition.
 * - Typed tolerances (position/pressure/flow/velocity) should be configured in
 *   new recipes; the legacy generic tolerance field is kept only as fallback.
 */
typedef struct {
    HDY_BOOL EN;
    HDY_BOOL RESET;
    HDY_BOOL START_SEGMENT;
    HDY_UINT START_SEGMENT_INDEX;
    HDY_REAL FLOW_TO_PUMP_SPEED_GAIN;
    HDY_REAL PUMP_SPEED_LIMIT;
    HDY_UINT RECIPE_SIZE;
    HDY_AxisRef AXIS_REF;
    HDY_MotionSegment RECIPE[HDY_MAX_SEGMENTS];

    HDY_BOOL ENO;
    HDY_BOOL ACTIVE;
    HDY_BOOL FINISHED;
    HDY_BOOL FAULT;
    HDY_ControllerStatus STATUS;
    HDY_REAL PUMP_SPEED;
    HDY_BOOL SEGMENT_COMPLETED;
    HDY_BOOL SEGMENT_CHANGED;
    HDY_MotionState STATE;
    HDY_DiagnosticInfo DIAGNOSTIC;
    char CURRENT_SEGMENT_NAME[HDY_NAME_MAX];

    /* Internal */
    HDY_REAL _segmentStartTime;
    HDY_BOOL _segmentChangedFlag;
    HDY_RampController _rampController;
} HDY_MotionControlFB;

/* Full reset of configuration, recipe, runtime state, and internal helpers. */
void HDY_MotionControlFB_Init(HDY_MotionControlFB* fb);

/*
 * Validates and loads a recipe into the function block.
 * This call only prepares the controller; it does not start execution and keeps
 * ACTIVE=false until StartSegment() or START_SEGMENT is issued.
 */
HDY_BOOL HDY_MotionControlFB_LoadRecipe(HDY_MotionControlFB* fb, const HDY_MotionSegment* recipe, size_t recipeSize);

/*
 * Explicitly arms the requested segment for cyclic execution.
 * Safe outputs are applied before the new segment becomes active, and the next
 * Execute() emits SEGMENT_CHANGED for one cycle.
 */
HDY_BOOL HDY_MotionControlFB_StartSegment(HDY_MotionControlFB* fb, size_t segmentIndex, HDY_TIME timestamp);

/*
 * Advances to the next segment only after SEGMENT_COMPLETED=true.
 * When the current segment is already the last one, the caller should observe
 * FINISHED instead of expecting another successful transition.
 */
HDY_BOOL HDY_MotionControlFB_NextSegment(HDY_MotionControlFB* fb, HDY_TIME timestamp);

/* Immediately enters a safe finished state and clears any pending start command. */
HDY_BOOL HDY_MotionControlFB_Abort(HDY_MotionControlFB* fb);

/* Cyclic execution entry point. Consumes START_SEGMENT as a one-shot command. */
void HDY_MotionControlFB_Execute(HDY_MotionControlFB* fb);

#endif /* HDY_MOTION_CONTROL_H */
