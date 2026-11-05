# FB_HydAxis Segment Ramp Planning Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make `FB_HydAxis` convert millisecond acceleration/deceleration inputs into hydraulic-library rates, preserve command-side segment velocity continuity, and protect hydraulic direction changes by decelerating to zero before reversal.

**Architecture:** Keep the change inside the IEC wrapper. Add a private method-based ramp planner with a nonnegative velocity-magnitude chain (`V0`, `Vset`, `Vend`) and a separate active/target direction pair. Reuse the existing `105` position-segment retrigger and `MoveVelocity.CONTINUOUSUPDATE`; add one bounded reverse-deceleration state that owns the existing `HYD_Stop` FB until zero-speed completion.

**Tech Stack:** IEC 61131-3 Structured Text in `iec/FB_HydAxis.st`, Python source-contract tests, CMake/CTest, existing HydroMotionLib motion FBs.

---

## File Map

- Modify `iec/FB_HydAxis.st`
  - Add command-side velocity and direction latches.
  - Add `CalcStartVelocity`, `ClearSegLatch`, and `CalcSegmentRamp` methods.
  - Select direction-specific maximum speed before converting `uiSpd`/`uiEndSpd`.
  - Add the reverse deceleration state and route active values into the motion FB calls.
- Modify `tests/test_hydaxis_p0_contract.py`
  - Replace assertions for the reverted implementation with source-contract checks for the confirmed design.
  - Cover 100 ms floors, positive speed magnitudes, `Vend`/`Vset` carryover, direction-specific limits, and zero-speed reversal sequencing.
- Do not modify `src/motion_planner.c` or `src/motion_interface.c`
  - The existing C planner already owns signed internal velocity and position-distance direction handling; this change must preserve that contract.
- Do not modify `CMakeLists.txt`
  - `test_hydaxis_p0_contract` is already registered with CTest.

## Task 1: Replace stale source-contract tests with failing tests for the approved design

**Files:**
- Modify: `tests/test_hydaxis_p0_contract.py`
- Test: `tests/test_hydaxis_p0_contract.py`

- [ ] **Step 1: Replace the reverted-method assertions.**

Keep `section_between()` and replace tests that require removed symbols such as `rVel0Magnitude`, `rDirSignLastSeg`, or the old pressure section anchors with these focused checks:

```python
class HydAxisRampContractTest(unittest.TestCase):
    def test_ramp_method_uses_positive_magnitudes_and_time_floor(self):
        ramp = section_between("METHOD PRIVATE CalcSegmentRamp", "END_METHOD")
        self.assertIn("rVel0Magnitude : REAL;", ramp)
        self.assertIn("rDeltaV : REAL;", ramp)
        self.assertIn("MAX(UINT_TO_REAL(uiAcc), rMinRampTime)", ramp)
        self.assertIn("MAX(UINT_TO_REAL(uiDec), rMinRampTime)", ramp)
        self.assertIn("rVel0Magnitude := CalcStartVelocity();", ramp)
        self.assertIn("ABS(rVel - rVel0Magnitude)", ramp)

    def test_direction_limit_is_selected_before_velocity_conversion(self):
        conversion = section_between("//--- 1. 边沿检测", "//--- 2. 状态机主逻辑")
        self.assertIn("rMaxSpeedExtend", conversion)
        self.assertIn("rMaxSpeedRetract", conversion)
        self.assertLess(conversion.index("rMaxSpeedExtend"), conversion.index("rVel  :="))
        self.assertLess(conversion.index("rMaxSpeedRetract"), conversion.index("rEndVel :="))

    def test_speed_chain_uses_position_end_velocity_and_velocity_target(self):
        start = section_between("METHOD PRIVATE CalcStartVelocity", "END_METHOD")
        self.assertIn("rEndVelLastSeg", start)
        self.assertIn("rVelLastSeg", start)
        self.assertIn("ELSE", start)
        self.assertIn("CalcStartVelocity := 0.0;", start)

    def test_reverse_transition_keeps_active_direction_until_zero(self):
        declarations = section_between("  VAR\n    iStep : INT;", "  END_VAR")
        ramp = section_between("METHOD PRIVATE CalcSegmentRamp", "END_METHOD")
        self.assertIn("iDirActive", declarations)
        self.assertIn("iDirTarget", declarations)
        self.assertIn("bDecelToZero", declarations)
        self.assertIn("bDirectionChange", ramp)
        self.assertIn("iDirActive := iDirLastSeg", ramp)
        self.assertIn("rDec := rVel0Magnitude / tDec", ramp)

    def test_motion_fbs_use_latched_active_values(self):
        calls = section_between("fbMoveContAbs(", "IF iStep = 150")
        self.assertIn("POSITION := rPosActive", calls)
        self.assertIn("VELOCITY := rVelActive", calls)
        self.assertIn("ENDVELOCITY := rEndVelActive", calls)
        self.assertIn("ENDVELOCITYDIRECTION := iDirActive", calls)
        self.assertIn("DIRECTION := iDirActive", calls)

    def test_reverse_state_owns_stop_until_done(self):
        reverse_state = section_between("150: // 反向减速到零", "200: // 正常停止")
        self.assertIn("fbStop.DONE", reverse_state)
        self.assertIn("iDirActive := iDirTarget", reverse_state)
        self.assertIn("iStep := 105", reverse_state)

    def test_stop_and_reset_clear_speed_chain(self):
        clear = section_between("METHOD PRIVATE ClearSegLatch", "END_METHOD")
        self.assertIn("bSegVelValid := FALSE", clear)
        self.assertIn("bDecelToZero := FALSE", clear)
        self.assertIn("iDirActive := 0", clear)
        self.assertIn("iDirTarget := 0", clear)
```

The existing status, pressure, and emergency-stop tests remain, but their section anchors must match the current file. Do not retain assertions for fields or comments that are absent from the current branch.

- [ ] **Step 2: Run the focused test before implementation.**

Run:

```bash
PYTHONDONTWRITEBYTECODE=1 python3 tests/test_hydaxis_p0_contract.py
```

Expected: FAIL because `CalcSegmentRamp`, the active-direction latches, and the reverse state do not yet exist. This confirms the tests are testing the requested change.

- [ ] **Step 3: Commit the failing-test contract.**

```bash
git add tests/test_hydaxis_p0_contract.py
git commit -m "Define FB_HydAxis segment ramp contracts" -m "Constraint: IEC source is validated through source-contract tests\nRejected: testing reverted implementation details | they do not describe the approved behavior\nConfidence: high\nScope-risk: narrow\nDirective: Keep speed magnitude and direction assertions separate\nTested: focused source-contract test expected to fail before implementation\nNot-tested: IEC runtime before implementation"
```

## Task 2: Add velocity-chain and direction-latch state plus lifecycle methods

**Files:**
- Modify: `iec/FB_HydAxis.st:31-78`
- Modify: `iec/FB_HydAxis.st` after the existing public configuration methods
- Test: `tests/test_hydaxis_p0_contract.py`

- [ ] **Step 1: Add the private state required by the method boundary.**

Add these variables to the existing `VAR` block without changing public inputs or outputs:

```iecst
    iDirLastSeg : SINT := 0;
    iDirActive : SINT := 0;
    iDirTarget : SINT := 0;
    rMaxSpeedDir : REAL := 0.0;
    rVelLastSeg : REAL := 0.0;
    rEndVelLastSeg : REAL := 0.0;
    uiLastSegMode : UINT := 0;
    bSegVelValid : BOOL := FALSE;
    bDirectionChange : BOOL := FALSE;
    bDecelToZero : BOOL := FALSE;
    bTransDecel : BOOL := FALSE;
    bNewSeg : BOOL := FALSE;
    rVel0 : REAL := 0.0;
    rVel0Magnitude : REAL := 0.0;
    rDeltaV : REAL := 0.0;
    rDecSeg : REAL := 200.0;
    rTransTime : REAL := 0.0;
    rTransTimer : REAL := 0.0;
    rDecDist : REAL := 0.0;
    rPosActive : REAL := 0.0;
    rVelActive : REAL := 0.0;
    rEndVelActive : REAL := 0.0;
    rPresActive : REAL := 0.0;
    rCycleTime : REAL := 0.001;
    rMinRampTime : REAL := 100.0;
    rDeltaVDeadZone : REAL := 0.001;
```

Use `rMinRampTime` as the fixed 100 ms floor. Keep `rVelLastSeg` and `rEndVelLastSeg` as physical nonnegative speed magnitudes.

- [ ] **Step 2: Add `CalcStartVelocity`.**

Insert this private method before the main FB body:

```iecst
  METHOD PRIVATE CalcStartVelocity : REAL
    IF NOT bSegVelValid THEN
        CalcStartVelocity := 0.0;
        RETURN;
    END_IF;

    CASE uiLastSegMode OF
        1: CalcStartVelocity := MAX(rEndVelLastSeg, 0.0);
        2: CalcStartVelocity := MAX(rVelLastSeg, 0.0);
    ELSE
        CalcStartVelocity := 0.0;
    END_CASE;
  END_METHOD
```

- [ ] **Step 3: Add `ClearSegLatch`.**

Use one method for every stop, emergency-stop, reset, and alarm-to-idle path:

```iecst
  METHOD PRIVATE ClearSegLatch : BOOL
    bSegVelValid := FALSE;
    bDirectionChange := FALSE;
    bDecelToZero := FALSE;
    bTransDecel := FALSE;
    bNewSeg := FALSE;
    iDirLastSeg := 0;
    iDirActive := 0;
    iDirTarget := 0;
    rVelLastSeg := 0.0;
    rEndVelLastSeg := 0.0;
    uiLastSegMode := 0;
    rVel0 := 0.0;
    rVel0Magnitude := 0.0;
    rTransTime := 0.0;
    rTransTimer := 0.0;
    ClearSegLatch := TRUE;
  END_METHOD
```

- [ ] **Step 4: Run the focused source test.**

Run the same Python command from Task 1. It should still fail on the ramp formulas, but the method/lifecycle assertions must pass. Do not move to implementation of the ramp method until the declarations and lifecycle methods are syntactically present.

- [ ] **Step 5: Commit the state/lifecycle slice.**

```bash
git add iec/FB_HydAxis.st
git commit -m "Add FB_HydAxis velocity chain state" -m "Constraint: Preserve the existing IEC FB surface and lifecycle\nRejected: external velocity-chain inputs | process segments already provide the required values\nConfidence: high\nScope-risk: moderate\nDirective: Clear all command-side latches when the axis leaves an active motion lifecycle\nTested: source-contract method and declaration checks\nNot-tested: runtime motion until ramp integration"
```

## Task 3: Implement direction-aware conversion and `CalcSegmentRamp`

**Files:**
- Modify: `iec/FB_HydAxis.st:296-320`
- Modify: `iec/FB_HydAxis.st` method section
- Test: `tests/test_hydaxis_p0_contract.py`

- [ ] **Step 1: Resolve direction and maximum speed before converting percentages.**

Replace the current direct `rMaxSpeed := stHydAxisRef.rMaxSpeed` block with:

```iecst
  CASE uiDir OF
      1:
          iDir := 1;
          rMaxSpeedDir := stHydAxisRef.rMaxSpeedExtend;
      2:
          iDir := 2;
          rMaxSpeedDir := stHydAxisRef.rMaxSpeedRetract;
  ELSE
      iDir := 0;
      rMaxSpeedDir := 0.0;
  END_CASE;

  IF rMaxSpeedDir <= 0.0 THEN
      rMaxSpeedDir := MAX(stHydAxisRef.rMaxSpeed, 0.0);
  END_IF;

  rMaxSpeed := rMaxSpeedDir;
  rVel := LIMIT(0.0,
                UINT_TO_REAL(uiSpd) * 0.001 * rMaxSpeedDir,
                rMaxSpeedDir);
  rEndVel := LIMIT(0.0,
                   UINT_TO_REAL(uiEndSpd) * 0.001 * rMaxSpeedDir,
                   rMaxSpeedDir);
```

Do not create negative `rVel` or `rEndVel` for `uiDir=2`.

- [ ] **Step 2: Implement the method’s time floor and same-direction formulas.**

The method must use this sequence:

```iecst
  tAcc := MAX(UINT_TO_REAL(uiAcc), rMinRampTime) * 0.001;
  tDec := MAX(UINT_TO_REAL(uiDec), rMinRampTime) * 0.001;
  rVel0Magnitude := CalcStartVelocity();
  iDirTarget := iDir;

  IF uiMode = 3 THEN
      ClearSegLatch();
      CalcSegmentRamp := TRUE;
      RETURN;
  END_IF;

  IF rMaxSpeedDir <= 0.0 THEN
      rAcc := 0.0;
      rDec := 0.0;
      CalcSegmentRamp := FALSE;
      RETURN;
  END_IF;

  rDeltaV := rVel - rVel0Magnitude;
  IF rDeltaV > rDeltaVDeadZone THEN
      rAcc := rDeltaV / tAcc;
      bTransDecel := FALSE;
  ELSIF rDeltaV < -rDeltaVDeadZone THEN
      rDec := ABS(rDeltaV) / tDec;
      bTransDecel := TRUE;
      rTransTime := ABS(rDeltaV) / MAX(rDec, 1.0);
      rTransTimer := 0.0;
  END_IF;
```

When the computed side is not used in the current transition, preserve a positive compatibility value (`200.0`) so the underlying motion FB still receives valid acceleration and deceleration limits.

- [ ] **Step 3: Add terminal deceleration and latch the segment.**

Use the current segment’s `Vend` only for its terminal ramp and next-segment carryover:

```iecst
  IF uiMode = 1 THEN
      rDecSeg := ABS(rVel - rEndVel) / tDec;
  ELSE
      rDecSeg := rVel / tDec;
  END_IF;
  rDecSeg := MAX(rDecSeg, 1.0);

  rVelLastSeg := MAX(rVel, 0.0);
  rEndVelLastSeg := MAX(rEndVel, 0.0);
  uiLastSegMode := uiMode;
  iDirLastSeg := iDirTarget;
  bSegVelValid := TRUE;
  CalcSegmentRamp := TRUE;
```

For a downward same-direction transition, keep the transition `rDec` until the transition completes, then switch to `rDecSeg` through the existing `105` position retrigger or the velocity FB’s continuous update.

- [ ] **Step 4: Run the focused test and inspect failure output.**

Run:

```bash
PYTHONDONTWRITEBYTECODE=1 python3 tests/test_hydaxis_p0_contract.py
```

Expected: conversion, magnitude, and 100 ms assertions pass; reverse-state and active-call assertions remain failing until Task 4.

- [ ] **Step 5: Commit the conversion and ramp method.**

```bash
git add iec/FB_HydAxis.st tests/test_hydaxis_p0_contract.py
git commit -m "Calculate FB_HydAxis ramps from segment velocity deltas" -m "Constraint: uiAcc and uiDec are process times in milliseconds while the library consumes positive rates per second\nRejected: max-speed divided by time for every segment | it ignores V0 and causes discontinuities\nConfidence: high\nScope-risk: moderate\nDirective: Compare only nonnegative velocity magnitudes inside the ramp method\nTested: source-contract conversion and time-floor checks\nNot-tested: reverse transition lifecycle until Task 4"
```

## Task 4: Add explicit reverse-deceleration state and active command latching

**Files:**
- Modify: `iec/FB_HydAxis.st` state machine and motion FB calls
- Test: `tests/test_hydaxis_p0_contract.py`

- [ ] **Step 1: Detect direction changes before applying `Vset-V0`.**

Inside `CalcSegmentRamp`, before the same-direction branch, add:

```iecst
  bDirectionChange := bSegVelValid
                       AND (rVel0Magnitude > rDeltaVDeadZone)
                       AND (iDirLastSeg <> 0)
                       AND (iDirTarget <> 0)
                       AND (iDirLastSeg <> iDirTarget);

  IF bDirectionChange THEN
      bDecelToZero := TRUE;
      iDirActive := iDirLastSeg;
      rDec := rVel0Magnitude / tDec;
      rAcc := MAX(rVel / tAcc, 1.0);
      rTransTime := rVel0Magnitude / MAX(rDec, 1.0);
      rTransTimer := 0.0;
  ELSIF NOT bSegVelValid OR (iDirActive = 0) THEN
      iDirActive := iDirTarget;
  END_IF;
```

The reverse path must not compute a signed `Vset-V0` transition and must not replace `iDirActive` with `iDirTarget` before the stop completes.

- [ ] **Step 2: Latch active command values only on a new segment.**

Before the state machine calls the motion FBs, use the existing start-edge gate and add:

```iecst
  bNewSeg := rTrigStart.Q
             AND (uiMode >= 1) AND (uiMode <= 3)
             AND NOT rTrigStop.Q
             AND NOT rTrigEStop.Q
             AND (iStep = 10 OR iStep = 100 OR iStep = 105);

  IF bNewSeg THEN
      rPosActive := rPos;
      rVelActive := rVel;
      rEndVelActive := rEndVel;
      rPresActive := rPres;
      CalcSegmentRamp();
      IF bDecelToZero THEN
          iStep := 150;
      END_IF;
  END_IF;
```

Pressure targets may continue to update through the existing pressure path; position and velocity setpoints used by their FB calls must come from the active latches.

- [ ] **Step 3: Add state `150` for hydraulic reverse deceleration.**

Insert this state before the existing normal-stop state:

```iecst
      150: // 反向减速到零
          bCmdExecute := FALSE;
          IF fbStop.DONE THEN
              bDecelToZero := FALSE;
              bDirectionChange := FALSE;
              iDirActive := iDirTarget;
              rVel0 := 0.0;
              iStep := 105;
          ELSIF fbStop.ERROR THEN
              bAlarm := TRUE;
              dwAlarmID := fbStop.ERRORID;
              iStep := 900;
          END_IF;

```

When `bDecelToZero` is set while running, set `iStep := 150` after the current segment is latched. Preserve the pending target values in `rPosActive`, `rVelActive`, `rEndVelActive`, and `iDirTarget` while `HYD_Stop` owns the axis.

- [ ] **Step 4: Route `HYD_Stop` through the reverse state with the planned deceleration.**

Use this call ordering:

```iecst
  IF iStep = 150 THEN
      fbStop(AXISID := siAxisId, EXECUTE := TRUE, DECELERATION := rDec);
  ELSIF iStep = 200 THEN
      fbStop(AXISID := siAxisId, EXECUTE := TRUE, DECELERATION := rStopDec);
  ELSE
      fbStop(AXISID := siAxisId, EXECUTE := FALSE, DECELERATION := rStopDec);
  END_IF;
```

This keeps ordinary stop behavior unchanged while making a planned reverse use the segment’s `uiDec`-derived rate.

- [ ] **Step 5: Use active magnitudes and active direction in motion FB calls.**

Change the calls to:

```iecst
  fbMoveContAbs(
      AXISID := siAxisId,
      EXECUTE := (uiActiveMode = 1) AND bCmdExecute,
      POSITION := rPosActive,
      VELOCITY := rVelActive,
      ENDVELOCITY := rEndVelActive,
      ENDVELOCITYDIRECTION := iDirActive,
      ACCELERATION := rAcc,
      DECELERATION := rDec,
      JERK := rJerk,
      DIRECTION := iDirActive,
      ADAPTENDVELTOAVOIDOVERSHOOT := TRUE,
      PRESSURELIMIT := rPresActive);

  fbMoveVel(
      AXISID := siAxisId,
      EXECUTE := (uiActiveMode = 2) AND bCmdExecute,
      CONTINUOUSUPDATE := TRUE,
      VELOCITY := rVelActive,
      ACCELERATION := rAcc,
      DECELERATION := rDec,
      JERK := rJerk,
      DIRECTION := iDirActive,
      BUFFERMODE := 0,
      PRESSURELIMIT := rPresActive);
```

Do not pass a negative `VELOCITY` or `ENDVELOCITY` for retract.

- [ ] **Step 6: Clear the chain on all lifecycle exits.**

Call `ClearSegLatch()` when normal stop, emergency stop, reset, or alarm recovery transitions back to idle. Do not clear the latches inside state `150` before `fbStop.DONE`; the pending reverse segment still needs them.

- [ ] **Step 7: Run the focused source test.**

Run:

```bash
PYTHONDONTWRITEBYTECODE=1 python3 tests/test_hydaxis_p0_contract.py
```

Expected: all focused source-contract tests pass.

- [ ] **Step 8: Commit reverse sequencing and active calls.**

```bash
git add iec/FB_HydAxis.st tests/test_hydaxis_p0_contract.py
git commit -m "Protect FB_HydAxis hydraulic direction changes" -m "Constraint: Hydraulic reversal must settle to zero before the direction command changes\nRejected: signed velocity reversal in one segment | it creates a pump/valve direction step\nConfidence: high\nScope-risk: broad\nDirective: Keep iDirActive on the old direction until HYD_Stop.DONE\nTested: source-contract reversal and active-call checks\nNot-tested: full CTest until terminal-ramp integration"
```

## Task 5: Integrate terminal `Vend` deceleration and online transition switching

**Files:**
- Modify: `iec/FB_HydAxis.st` before the main state machine and in state `100`
- Test: `tests/test_hydaxis_p0_contract.py`

- [ ] **Step 1: Add transition-completion tracking.**

For same-direction downward transitions, increment `rTransTimer` by `rCycleTime` while `bTransDecel` is active. Use the existing FB capabilities:

```iecst
  IF bTransDecel AND (iStep = 100) THEN
      rTransTimer := rTransTimer + rCycleTime;
      CASE uiActiveMode OF
          2:
              IF fbMoveVel.InVelocity OR
                 (rTransTimer >= rTransTime * 1.2) THEN
                  rDec := rDecSeg;
                  bTransDecel := FALSE;
              END_IF;
          1:
              rDecDist := MAX((rVelActive * rVelActive -
                               rEndVelActive * rEndVelActive) /
                              (2.0 * MAX(rDecSeg, 1.0)), 0.0);
              IF (ABS(rPosActive - stHydAxisRef.rActPosition) <= rDecDist * 1.2) OR
                 (rTransTimer >= rTransTime * 1.2) THEN
                  rDec := rDecSeg;
                  bTransDecel := FALSE;
                  iStep := 105;
              END_IF;
      END_CASE;
  END_IF;
```

The position path keeps the existing `105` one-scan execute drop so the library re-locks the new `DECELERATION`. The velocity path changes `rDec` online.

- [ ] **Step 2: Preserve `Vend` as the next segment start.**

Do not overwrite `rEndVelLastSeg` with `rVelLastSeg`. The carryover method must continue to select:

```iecst
1: CalcStartVelocity := rEndVelLastSeg;
2: CalcStartVelocity := rVelLastSeg;
```

Pressure mode must call `ClearSegLatch()` when it becomes the active mode so a later position or velocity segment starts with `V0=0`.

- [ ] **Step 3: Add source-contract checks for terminal and direction behavior.**

Add assertions for:

```python
def test_position_terminal_deceleration_uses_vend(self):
    ramp = section_between("METHOD PRIVATE CalcSegmentRamp", "END_METHOD")
    self.assertIn("ABS(rVel - rEndVel)", ramp)
    self.assertIn("rEndVelActive", SOURCE)

def test_position_retrigger_updates_terminal_deceleration(self):
    self.assertIn("rDecDist", SOURCE)
    self.assertIn("iStep := 105", SOURCE)
    self.assertIn("DECELERATION := rDec", SOURCE)

def test_retract_keeps_velocity_positive(self):
    conversion = section_between("CASE uiDir OF", "//--- 2. 状态机主逻辑")
    self.assertNotIn("rVel := -", conversion)
    self.assertNotIn("rEndVel := -", conversion)
    self.assertIn("DIRECTION := iDirActive", SOURCE)
```

- [ ] **Step 4: Run the focused source test and commit.**

```bash
PYTHONDONTWRITEBYTECODE=1 python3 tests/test_hydaxis_p0_contract.py
git add iec/FB_HydAxis.st tests/test_hydaxis_p0_contract.py
git commit -m "Complete FB_HydAxis terminal ramp handoff" -m "Constraint: Position FBs latch parameters on execute edges while velocity FBs update continuously\nRejected: replacing terminal Vend planning with a single max-speed ramp | it loses segment-end continuity\nConfidence: high\nScope-risk: moderate\nDirective: Keep transition deceleration and terminal deceleration as separate phases\nTested: focused source-contract suite\nNot-tested: full CTest until final verification"
```

## Task 6: Full verification and implementation handoff

**Files:**
- Inspect: `iec/FB_HydAxis.st`
- Inspect: `tests/test_hydaxis_p0_contract.py`
- Test: existing CMake/CTest targets

- [ ] **Step 1: Run the focused source-contract test.**

```bash
PYTHONDONTWRITEBYTECODE=1 python3 tests/test_hydaxis_p0_contract.py
```

Expected: all tests pass, including `0→300`, `300→0`, same-direction delta selection, `Vend` carryover, 100 ms floors, and reverse-state sequencing.

- [ ] **Step 2: Build the existing C targets.**

```bash
cmake --build --preset unixgcc
```

Expected: build completes without changing public C headers or C planner behavior.

- [ ] **Step 3: Run the complete regression suite.**

```bash
ctest --test-dir out/build/unixgcc --output-on-failure
```

Expected: all existing tests pass, especially `test_motion_planner`, `test_movecontinuousabsolute_integration`, `test_movevelocity_stop_reverse_restart`, `test_moveabsolute_stop_integration`, and `test_hydaxis_p0_contract`.

- [ ] **Step 4: Review the final diff for direction and unit mistakes.**

```bash
git diff -- iec/FB_HydAxis.st tests/test_hydaxis_p0_contract.py
git diff --check
```

Confirm that:

- `rVel` and `rEndVel` remain nonnegative;
- `iDirActive` is the only direction passed to motion FBs;
- `uiAcc/uiDec` are converted from milliseconds with the 100 ms floor;
- no current input overwrites latched position/velocity setpoints mid-segment;
- stop and reset paths clear the velocity chain.

- [ ] **Step 5: Commit only if the final verification changes the implementation.**

```bash
git status --short
git commit -m "Verify hydraulic segment ramp integration" -m "Constraint: Completion requires fresh source-contract, build, and CTest evidence\nRejected: declaring completion from static inspection alone | runtime regressions remain possible\nConfidence: high\nScope-risk: narrow\nDirective: Preserve positive speed magnitudes and explicit direction semantics in future edits\nTested: source contract, CMake build, full CTest\nNot-tested: physical machine acceptance without field hardware"
```

## Self-review checklist

- The plan covers every approved design requirement: method encapsulation, 100 ms floors, direction-specific speed selection, positive magnitudes, `Vend`/`Vset` carryover, reverse zero-speed sequencing, lifecycle clearing, and regression tests.
- No C-library or public IEC-interface changes are planned.
- Every implementation task names exact files and provides the required ST/Python content or command.
- The plan keeps source-contract testing as the executable IEC validation because this repository does not compile `iec/FB_HydAxis.st` into the C targets.
