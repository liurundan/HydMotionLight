# Beckhoff BufferMode Live Update Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add Beckhoff-compatible `BufferMode` values, one-slot direct command buffering, and online target updates for `MoveAbsolute`, `MoveVelocity`, and `PressureHandle`.

**Architecture:** Keep IEC-facing FB pin validation in `motion_interface.c`, direct command lifecycle and arbitration in `motion_control.c`, and shared enum/type contracts in public headers. A live update mutates selected fields on `_activeSegment` without restarting planners/controllers or changing the active execution id. Direct command buffering is deliberately limited to the running command plus one pending direct slot for embedded hydraulic runtime predictability.

**Tech Stack:** C99 runtime library, matiec-style IEC generated structs/macros, CMake, CTest, project-specific C unit tests, `pousHydMotion.xml` POU surface.

---

## File Structure

- Modify `include/common_types.h`: define Beckhoff-compatible `HYD_BufferMode` values `0..5`.
- Modify `include/motion_control.h`: expose direct pending state fields on `HYD_MotionControlFB`, live-update request types, and direct start/update APIs.
- Modify `src/motion_control.c`: implement one-slot direct buffering, endless-segment degrade-to-abort behavior, pending direct startup, and live updates against `_activeSegment`.
- Modify `include/motion_interface.h`: add `CONTINUOUSUPDATE` to `HYD_PRESSUREHANDLE`.
- Modify `pousHydMotion.xml`: add `PressureHandle.CONTINUOUSUPDATE` to the public POU metadata.
- Modify `src/motion_interface.c`: accept BufferMode `0..5`, route direct starts through core BufferMode API, and map sustained `CONTINUOUSUPDATE` scans into core live-update requests.
- Modify `tests/test_motion_interface_unit.c`: cover BufferMode validation and live update acceptance for velocity/pressure.
- Modify `tests/test_motion_interface_arbitration.c`: cover one-slot buffered direct lifecycle and endless direct fallback behavior.
- Run `tests/test_interface_layout_consistency.py`: verify C struct and XML surface remain aligned.

## Execution Status

This plan has been executed in isolated worktree:

```bash
/home/dan/project/hdy-motion-light/.worktrees/beckhoff-buffer-live-update
```

Known verification result: targeted tests, layout consistency, and full `ctest` pass after restoring recipe capacity to the documented minimum of 4 segments.

---

### Task 1: Expand Beckhoff BufferMode enum

**Files:**
- Modify: `include/common_types.h`
- Test: `tests/test_motion_interface_unit.c`

- [x] **Step 1: Write the failing enum/validation test**

Add this test to `tests/test_motion_interface_unit.c`:

```c
static void test_moveabsolute_accepts_beckhoff_buffer_modes(void) {
    HYD_MOTIONCONTROLFB fb;
    HYD_MOVEABSOLUTE ma;

    for (int mode = 0; mode <= 5; ++mode) {
        resetAll();
        HYD_MotionControlFB_Init(&fb);
        HYD_MoveAbsolute_Init(&ma);

        IEC_VAL(ma.EXECUTE) = true;
        IEC_VAL(ma.POSITION) = 10.0;
        IEC_VAL(ma.VELOCITY) = 5.0;
        IEC_VAL(ma.ACCELERATION) = 10.0;
        IEC_VAL(ma.DECELERATION) = 10.0;
        IEC_VAL(ma.BUFFERMODE) = mode;

        HYD_MoveAbsolute_Call(&ma, &fb);

        assertTrue(!IEC_VAL(ma.ERROR),
                   "MoveAbsolute should accept Beckhoff BufferMode 0..5");
    }

    resetAll();
    HYD_MotionControlFB_Init(&fb);
    HYD_MoveAbsolute_Init(&ma);

    IEC_VAL(ma.EXECUTE) = true;
    IEC_VAL(ma.POSITION) = 10.0;
    IEC_VAL(ma.VELOCITY) = 5.0;
    IEC_VAL(ma.ACCELERATION) = 10.0;
    IEC_VAL(ma.DECELERATION) = 10.0;
    IEC_VAL(ma.BUFFERMODE) = 6;

    HYD_MoveAbsolute_Call(&ma, &fb);

    assertTrue(IEC_VAL(ma.ERROR),
               "MoveAbsolute should reject unsupported BufferMode values");
}
```

- [x] **Step 2: Run test to verify it fails before implementation**

Run:

```bash
cmake --build build -j2
./build/test_motion_interface_unit
```

Expected before implementation: the new test fails because the old enum/validation does not accept all Beckhoff values `0..5`.

- [x] **Step 3: Implement enum values**

Replace the old `HYD_BufferMode` definition in `include/common_types.h` with:

```c
typedef enum {
    HYD_BUFFER_MODE_ABORT = 0,
    HYD_BUFFER_MODE_BUFFER = 1,
    HYD_BUFFER_MODE_BLENDING_LOW = 2,
    HYD_BUFFER_MODE_BLENDING_PREVIOUS = 3,
    HYD_BUFFER_MODE_BLENDING_NEXT = 4,
    HYD_BUFFER_MODE_BLENDING_HIGH = 5
} HYD_BufferMode;
```

Update `validateSupportedBufferMode` in `src/motion_interface.c` to accept the inclusive range:

```c
static HYD_BOOL validateSupportedBufferMode(HYD_LREAL bufferMode) {
    if (bufferMode >= HYD_BUFFER_MODE_ABORT &&
        bufferMode <= HYD_BUFFER_MODE_BLENDING_HIGH) {
        return true;
    }
    HYD_SetDiag(HYD_DIAG_INVALID_BUFFER_MODE);
    return false;
}
```

- [x] **Step 4: Run test to verify it passes**

Run:

```bash
cmake --build build -j2
./build/test_motion_interface_unit
```

Expected after implementation: `test_moveabsolute_accepts_beckhoff_buffer_modes` passes.

---

### Task 2: Add core one-slot pending direct command state

**Files:**
- Modify: `include/motion_control.h`
- Modify: `src/motion_control.c`
- Test: `tests/test_motion_interface_arbitration.c`

- [x] **Step 1: Write the failing buffered direct lifecycle test**

Add this test to `tests/test_motion_interface_arbitration.c`:

```c
static void test_buffered_moveabsolute_waits_without_preempting_active_owner(void) {
    HYD_MOTIONCONTROLFB fb;
    HYD_MOVEABSOLUTE first;
    HYD_MOVEABSOLUTE second;

    resetAll();
    HYD_MotionControlFB_Init(&fb);
    HYD_MoveAbsolute_Init(&first);
    HYD_MoveAbsolute_Init(&second);

    IEC_VAL(first.EXECUTE) = true;
    IEC_VAL(first.POSITION) = 50.0;
    IEC_VAL(first.VELOCITY) = 10.0;
    IEC_VAL(first.ACCELERATION) = 20.0;
    IEC_VAL(first.DECELERATION) = 20.0;
    IEC_VAL(first.BUFFERMODE) = HYD_BUFFER_MODE_ABORT;
    HYD_MoveAbsolute_Call(&first, &fb);

    IEC_VAL(second.EXECUTE) = true;
    IEC_VAL(second.POSITION) = 70.0;
    IEC_VAL(second.VELOCITY) = 10.0;
    IEC_VAL(second.ACCELERATION) = 20.0;
    IEC_VAL(second.DECELERATION) = 20.0;
    IEC_VAL(second.BUFFERMODE) = HYD_BUFFER_MODE_BUFFER;
    HYD_MoveAbsolute_Call(&second, &fb);

    assertTrue(IEC_VAL(first.BUSY), "first command should remain busy");
    assertTrue(IEC_VAL(first.ACTIVE), "first command should remain active");
    assertTrue(IEC_VAL(second.BUSY), "second command should be accepted as pending");
    assertTrue(!IEC_VAL(second.ACTIVE), "pending command should not be active yet");
    assertTrue(!IEC_VAL(first.COMMANDABORTED),
               "Buffered command should not abort the active command");
}
```

- [x] **Step 2: Run test to verify it fails before implementation**

Run:

```bash
cmake --build build -j2
./build/test_motion_interface_arbitration
```

Expected before implementation: the second command aborts or is rejected instead of waiting as pending.

- [x] **Step 3: Add pending fields and helper prototypes**

Add these fields to `HYD_MotionControlFB` in `include/motion_control.h`:

```c
    HYD_BOOL _directPendingValid;
    HYD_MotionSegment _directPendingSegment;
    HYD_DirectCommandKind _directPendingKind;
    HYD_BufferMode _directPendingBufferMode;
```

Add this public direct start API declaration in `include/motion_control.h`:

```c
HYD_BOOL HYD_MotionControlFB_StartDirectCommand(HYD_MotionControlFB* fb,
                                                HYD_BufferMode bufferMode,
                                                const HYD_MotionSegment* segment,
                                                HYD_DiagnosticCode* errorId);
```

Add this pending slot reset helper in `src/motion_control.c`:

```c
static void HYD_ClearDirectPendingSlot(HYD_MotionControlFB* fb) {
    if (fb == NULL) {
        return;
    }
    fb->_directPendingValid = false;
    memset(&fb->_directPendingSegment, 0, sizeof(fb->_directPendingSegment));
    fb->_directPendingKind = HYD_DIRECT_CMD_NONE;
    fb->_directPendingBufferMode = HYD_BUFFER_MODE_ABORT;
}
```

Call `HYD_ClearDirectPendingSlot(fb)` from initialization, soft reset, recipe load preparation, and abort paths.

- [x] **Step 4: Implement BufferMode start routing**

Implement `HYD_MotionControlFB_StartDirectCommand` in `src/motion_control.c`:

```c
HYD_BOOL HYD_MotionControlFB_StartDirectCommand(HYD_MotionControlFB* fb,
                                                HYD_BufferMode bufferMode,
                                                const HYD_MotionSegment* segment,
                                                HYD_DiagnosticCode* errorId) {
    HYD_BOOL hasActiveDirect;
    HYD_BOOL shouldAbort;

    if (errorId != NULL) {
        *errorId = HYD_DIAG_NONE;
    }
    if (fb == NULL || segment == NULL) {
        if (errorId != NULL) {
            *errorId = HYD_DIAG_INVALID_PARAMETER;
        }
        return false;
    }
    if (bufferMode < HYD_BUFFER_MODE_ABORT ||
        bufferMode > HYD_BUFFER_MODE_BLENDING_HIGH) {
        if (errorId != NULL) {
            *errorId = HYD_DIAG_INVALID_BUFFER_MODE;
        }
        return false;
    }

    hasActiveDirect = (fb->_owner == HYD_OWNER_DIRECT &&
                       fb->_state == HYD_FB_STATE_BUSY);
    shouldAbort = (bufferMode == HYD_BUFFER_MODE_ABORT && hasActiveDirect);

    if (shouldAbort) {
        HYD_AbortActiveExecution(fb);
        HYD_ClearDirectPendingSlot(fb);
        return HYD_MotionControlFB_StartSegment(fb, segment);
    }

    if (hasActiveDirect && bufferMode != HYD_BUFFER_MODE_ABORT) {
        if (HYD_IsSegmentEndlessForBuffering(&fb->_activeSegment)) {
            HYD_AbortActiveExecution(fb);
            HYD_ClearDirectPendingSlot(fb);
            return HYD_MotionControlFB_StartSegment(fb, segment);
        }
        if (fb->_directPendingValid) {
            if (errorId != NULL) {
                *errorId = HYD_DIAG_CODE_COMMAND_NOT_ALLOWED;
            }
            return false;
        }
        fb->_directPendingSegment = *segment;
        fb->_directPendingKind = HYD_InferDirectCommandKindFromSegment(segment);
        fb->_directPendingBufferMode = bufferMode;
        fb->_directPendingValid = true;
        return true;
    }

    return HYD_MotionControlFB_StartSegment(fb, segment);
}
```

The final implementation may preserve existing same-scan Stop behavior by routing idle starts through the existing pending START path; the observable contract remains the same.

- [x] **Step 5: Run test to verify it passes**

Run:

```bash
cmake --build build -j2
./build/test_motion_interface_arbitration
```

Expected after implementation: the new buffered direct lifecycle assertions pass. Existing unrelated recipe assertions may still fail if they already failed at baseline.

---

### Task 3: Add endless segment fallback behavior

**Files:**
- Modify: `src/motion_control.c`
- Test: `tests/test_motion_interface_arbitration.c`

- [x] **Step 1: Write the failing endless fallback test**

Add this test to `tests/test_motion_interface_arbitration.c`:

```c
static void test_buffered_endless_movevelocity_degrades_to_abort_takeover(void) {
    HYD_MOTIONCONTROLFB fb;
    HYD_MOVEVELOCITY first;
    HYD_MOVEVELOCITY second;

    resetAll();
    HYD_MotionControlFB_Init(&fb);
    HYD_MoveVelocity_Init(&first);
    HYD_MoveVelocity_Init(&second);

    IEC_VAL(first.EXECUTE) = true;
    IEC_VAL(first.VELOCITY) = 5.0;
    IEC_VAL(first.ACCELERATION) = 10.0;
    IEC_VAL(first.DECELERATION) = 10.0;
    IEC_VAL(first.BUFFERMODE) = HYD_BUFFER_MODE_ABORT;
    HYD_MoveVelocity_Call(&first, &fb);

    IEC_VAL(second.EXECUTE) = true;
    IEC_VAL(second.VELOCITY) = 8.0;
    IEC_VAL(second.ACCELERATION) = 10.0;
    IEC_VAL(second.DECELERATION) = 10.0;
    IEC_VAL(second.BUFFERMODE) = HYD_BUFFER_MODE_BUFFER;
    HYD_MoveVelocity_Call(&second, &fb);

    assertTrue(IEC_VAL(first.COMMANDABORTED),
               "Buffered command against endless MoveVelocity should abort first command");
    assertTrue(IEC_VAL(second.BUSY), "second command should become active owner");
    assertTrue(IEC_VAL(second.ACTIVE), "second command should be active after fallback takeover");
}
```

- [x] **Step 2: Run test to verify it fails before implementation**

Run:

```bash
cmake --build build -j2
./build/test_motion_interface_arbitration
```

Expected before implementation: the endless active command does not degrade buffered/blending to immediate takeover.

- [x] **Step 3: Implement endless detection**

Add this helper to `src/motion_control.c`:

```c
static HYD_BOOL HYD_IsSegmentEndlessForBuffering(const HYD_MotionSegment* segment) {
    if (segment == NULL) {
        return false;
    }
    if (segment->endCondition != HYD_END_MANUAL) {
        return false;
    }
    return segment->mode == HYD_MODE_SPEED_RAMP ||
           segment->mode == HYD_MODE_PRESSURE_CLOSED_LOOP;
}
```

Use it inside `HYD_MotionControlFB_StartDirectCommand` so `Buffered` and all `Blending*` modes fall back to abort takeover for endless `MoveVelocity` and no-duration `PressureHandle`.

- [x] **Step 4: Run test to verify it passes**

Run:

```bash
cmake --build build -j2
./build/test_motion_interface_arbitration
```

Expected after implementation: the new endless fallback test passes. Existing unrelated recipe assertions may still fail if they already failed at baseline.

---

### Task 4: Add live-update core API

**Files:**
- Modify: `include/motion_control.h`
- Modify: `src/motion_control.c`
- Test: `tests/test_motion_interface_unit.c`

- [x] **Step 1: Define live update request contract**

Add these definitions to `include/motion_control.h`:

```c
typedef enum {
    HYD_LIVE_UPDATE_TARGET_POSITION = 1u << 0,
    HYD_LIVE_UPDATE_MAX_VELOCITY = 1u << 1,
    HYD_LIVE_UPDATE_ACCELERATION = 1u << 2,
    HYD_LIVE_UPDATE_DECELERATION = 1u << 3,
    HYD_LIVE_UPDATE_TARGET_PRESSURE = 1u << 4,
    HYD_LIVE_UPDATE_PRESSURE_RAMP_RATE = 1u << 5
} HYD_LiveUpdateFlags;

typedef struct {
    HYD_DirectCommandKind ownerKind;
    HYD_UINT32 ownerExecutionId;
    HYD_UINT32 flags;
    HYD_LREAL targetPosition;
    HYD_LREAL maxVelocity;
    HYD_LREAL acceleration;
    HYD_LREAL deceleration;
    HYD_LREAL targetPressure;
    HYD_LREAL pressureRampRate;
} HYD_LiveUpdateRequest;

HYD_BOOL HYD_MotionControlFB_ApplyLiveUpdate(HYD_MotionControlFB* fb,
                                             const HYD_LiveUpdateRequest* request);
```

- [x] **Step 2: Implement active-owner guarded update**

Add this implementation to `src/motion_control.c`:

```c
HYD_BOOL HYD_MotionControlFB_ApplyLiveUpdate(HYD_MotionControlFB* fb,
                                             const HYD_LiveUpdateRequest* request) {
    HYD_MotionSegment updated;

    if (fb == NULL || request == NULL) {
        HYD_SetDiag(HYD_DIAG_INVALID_PARAMETER);
        return false;
    }
    if (fb->_owner != HYD_OWNER_DIRECT ||
        fb->_directKind != request->ownerKind ||
        fb->_directExecutionId != request->ownerExecutionId ||
        fb->_state != HYD_FB_STATE_BUSY) {
        HYD_SetDiag(HYD_DIAG_CODE_COMMAND_NOT_ALLOWED);
        return false;
    }

    updated = fb->_activeSegment;
    if ((request->flags & HYD_LIVE_UPDATE_TARGET_POSITION) != 0u) {
        updated.targetPosition = request->targetPosition;
    }
    if ((request->flags & HYD_LIVE_UPDATE_MAX_VELOCITY) != 0u) {
        updated.maxVelocity = request->maxVelocity;
    }
    if ((request->flags & HYD_LIVE_UPDATE_ACCELERATION) != 0u) {
        updated.acceleration = request->acceleration;
    }
    if ((request->flags & HYD_LIVE_UPDATE_DECELERATION) != 0u) {
        updated.deceleration = request->deceleration;
    }
    if ((request->flags & HYD_LIVE_UPDATE_TARGET_PRESSURE) != 0u) {
        updated.targetPressure = request->targetPressure;
    }
    if ((request->flags & HYD_LIVE_UPDATE_PRESSURE_RAMP_RATE) != 0u) {
        updated.pressureRampRate = request->pressureRampRate;
    }

    if (!HYD_ValidateSegment(&updated, NULL)) {
        HYD_SetDiag(HYD_DIAG_INVALID_PARAMETER);
        return false;
    }

    fb->_activeSegment = updated;
    return true;
}
```

Preserve planner, pressure controller, ramp controller, owner, and execution id state by only replacing validated `_activeSegment` fields.

- [x] **Step 3: Run targeted tests**

Run:

```bash
cmake --build build -j2
./build/test_motion_interface_unit
```

Expected: existing unit tests pass after adapters are wired in Task 5 and Task 6.

---

### Task 5: Wire MoveAbsolute and MoveVelocity live updates

**Files:**
- Modify: `src/motion_interface.c`
- Test: `tests/test_motion_interface_unit.c`

- [x] **Step 1: Write the failing MoveVelocity continuous update test**

Add this test to `tests/test_motion_interface_unit.c`:

```c
static void test_movevelocity_accepts_continuousupdate_and_updates_active_target(void) {
    HYD_MOTIONCONTROLFB fb;
    HYD_MOVEVELOCITY mv;

    resetAll();
    HYD_MotionControlFB_Init(&fb);
    HYD_MoveVelocity_Init(&mv);

    IEC_VAL(mv.EXECUTE) = true;
    IEC_VAL(mv.VELOCITY) = 5.0;
    IEC_VAL(mv.ACCELERATION) = 10.0;
    IEC_VAL(mv.DECELERATION) = 10.0;
    IEC_VAL(mv.CONTINUOUSUPDATE) = true;

    HYD_MoveVelocity_Call(&mv, &fb);
    IEC_VAL(mv.VELOCITY) = 8.0;
    HYD_MoveVelocity_Call(&mv, &fb);

    assertTrue(!IEC_VAL(mv.ERROR),
               "MoveVelocity should accept supported CONTINUOUSUPDATE");
    assertNear(fb._activeSegment.maxVelocity, 8.0, 0.0001,
               "MoveVelocity live update should change active target velocity");
}
```

- [x] **Step 2: Run test to verify it fails before implementation**

Run:

```bash
cmake --build build -j2
./build/test_motion_interface_unit
```

Expected before implementation: `CONTINUOUSUPDATE` is rejected or the active segment target stays latched.

- [x] **Step 3: Add MoveAbsolute and MoveVelocity update adapters**

Add helpers to `src/motion_interface.c`:

```c
static HYD_BOOL applyMoveAbsoluteLiveUpdate(HYD_MotionControlFB* fb,
                                            HYD_UINT32 executionId,
                                            HYD_MOVEABSOLUTE* data__) {
    HYD_LiveUpdateRequest request;

    if (fb == NULL || data__ == NULL || !__GET_VAR(data__->CONTINUOUSUPDATE)) {
        return true;
    }

    memset(&request, 0, sizeof(request));
    request.ownerKind = HYD_DIRECT_CMD_MOVE_ABSOLUTE;
    request.ownerExecutionId = executionId;
    request.flags = HYD_LIVE_UPDATE_TARGET_POSITION |
                    HYD_LIVE_UPDATE_MAX_VELOCITY |
                    HYD_LIVE_UPDATE_ACCELERATION |
                    HYD_LIVE_UPDATE_DECELERATION;
    request.targetPosition = __GET_VAR(data__->POSITION);
    request.maxVelocity = __GET_VAR(data__->VELOCITY);
    request.acceleration = __GET_VAR(data__->ACCELERATION);
    request.deceleration = __GET_VAR(data__->DECELERATION);
    return HYD_MotionControlFB_ApplyLiveUpdate(fb, &request);
}

static HYD_BOOL applyMoveVelocityLiveUpdate(HYD_MotionControlFB* fb,
                                            HYD_UINT32 executionId,
                                            HYD_MOVEVELOCITY* data__) {
    HYD_LiveUpdateRequest request;

    if (fb == NULL || data__ == NULL || !__GET_VAR(data__->CONTINUOUSUPDATE)) {
        return true;
    }

    memset(&request, 0, sizeof(request));
    request.ownerKind = HYD_DIRECT_CMD_MOVE_VELOCITY;
    request.ownerExecutionId = executionId;
    request.flags = HYD_LIVE_UPDATE_MAX_VELOCITY |
                    HYD_LIVE_UPDATE_ACCELERATION |
                    HYD_LIVE_UPDATE_DECELERATION;
    request.maxVelocity = __GET_VAR(data__->VELOCITY);
    request.acceleration = __GET_VAR(data__->ACCELERATION);
    request.deceleration = __GET_VAR(data__->DECELERATION);
    return HYD_MotionControlFB_ApplyLiveUpdate(fb, &request);
}
```

Call these helpers in the sustained `EXECUTE=TRUE` path when the FB is the active direct owner.

- [x] **Step 4: Route direct starts through core BufferMode API**

Replace direct start calls in `src/motion_interface.c` with:

```c
static HYD_BOOL startDirectSegmentExecution(HYD_MotionControlFB* fb,
                                            HYD_BufferMode bufferMode,
                                            const HYD_MotionSegment* segment,
                                            HYD_DiagnosticCode* errorId) {
    if (errorId != NULL) {
        *errorId = HYD_DIAG_NONE;
    }
    return HYD_MotionControlFB_StartDirectCommand(fb,
                                                 bufferMode,
                                                 segment,
                                                 errorId);
}
```

Remove the old adapter-side behavior that treated nonzero `BufferMode` as unsupported or manually aborted active direct commands before starting.

- [x] **Step 5: Run unit tests**

Run:

```bash
cmake --build build -j2
./build/test_motion_interface_unit
```

Expected after implementation: MoveVelocity continuous update test passes and `BUFFERMODE=0..5` remains accepted.

---

### Task 6: Add PressureHandle CONTINUOUSUPDATE pin and live update

**Files:**
- Modify: `include/motion_interface.h`
- Modify: `pousHydMotion.xml`
- Modify: `src/motion_interface.c`
- Test: `tests/test_motion_interface_unit.c`
- Test: `tests/test_interface_layout_consistency.py`

- [x] **Step 1: Write the failing PressureHandle continuous update test**

Add this test to `tests/test_motion_interface_unit.c`:

```c
static void test_pressurehandle_accepts_continuousupdate_and_updates_active_target(void) {
    HYD_MOTIONCONTROLFB fb;
    HYD_PRESSUREHANDLE ph;

    resetAll();
    HYD_MotionControlFB_Init(&fb);
    HYD_PressureHandle_Init(&ph);

    IEC_VAL(ph.EN) = true;
    IEC_VAL(ph.EXECUTE) = true;
    IEC_VAL(ph.PRESSURE) = 80.0;
    IEC_VAL(ph.RAMPRATE) = 20.0;
    IEC_VAL(ph.CONTINUOUSUPDATE) = true;

    HYD_PressureHandle_Call(&ph, &fb);
    IEC_VAL(ph.PRESSURE) = 100.0;
    IEC_VAL(ph.RAMPRATE) = 25.0;
    HYD_PressureHandle_Call(&ph, &fb);

    assertTrue(!IEC_VAL(ph.ERROR),
               "PressureHandle should expose and accept CONTINUOUSUPDATE");
    assertNear(fb._activeSegment.targetPressure, 100.0, 0.0001,
               "PressureHandle live update should change active pressure target");
    assertNear(fb._activeSegment.pressureRampRate, 25.0, 0.0001,
               "PressureHandle live update should change active pressure ramp rate");
}
```

- [x] **Step 2: Run tests to verify they fail before implementation**

Run:

```bash
cmake --build build -j2
./build/test_motion_interface_unit
python3 tests/test_interface_layout_consistency.py
```

Expected before implementation: C compile or layout consistency fails because `HYD_PRESSUREHANDLE` has no `CONTINUOUSUPDATE` pin.

- [x] **Step 3: Add PressureHandle pin to C header and XML**

Add this field to `HYD_PRESSUREHANDLE` in `include/motion_interface.h`:

```c
  __DECLARE_VAR(BOOL,CONTINUOUSUPDATE)
```

Add this variable to `pousHydMotion.xml` inside `PressureHandle` inputs:

```xml
<variable name="CONTINUOUSUPDATE">
  <type><BOOL/></type>
  <initialValue><simpleValue value="FALSE"/></initialValue>
</variable>
```

- [x] **Step 4: Add PressureHandle update adapter**

Add this helper to `src/motion_interface.c`:

```c
static HYD_BOOL applyPressureHandleLiveUpdate(HYD_MotionControlFB* fb,
                                              HYD_UINT32 executionId,
                                              HYD_PRESSUREHANDLE* data__) {
    HYD_LiveUpdateRequest request;

    if (fb == NULL || data__ == NULL || !__GET_VAR(data__->CONTINUOUSUPDATE)) {
        return true;
    }

    memset(&request, 0, sizeof(request));
    request.ownerKind = HYD_DIRECT_CMD_PRESSURE;
    request.ownerExecutionId = executionId;
    request.flags = HYD_LIVE_UPDATE_TARGET_PRESSURE |
                    HYD_LIVE_UPDATE_PRESSURE_RAMP_RATE;
    request.targetPressure = __GET_VAR(data__->PRESSURE);
    request.pressureRampRate = __GET_VAR(data__->RAMPRATE);
    return HYD_MotionControlFB_ApplyLiveUpdate(fb, &request);
}
```

Call it in the sustained `PressureHandle.EXECUTE=TRUE` owner path.

- [x] **Step 5: Run unit and layout tests**

Run:

```bash
cmake --build build -j2
./build/test_motion_interface_unit
python3 tests/test_interface_layout_consistency.py
```

Expected after implementation: unit tests pass and layout consistency reports success.

---

### Task 7: Verify queue capacity and lifecycle behavior

**Files:**
- Modify: `src/motion_control.c`
- Test: `tests/test_motion_interface_arbitration.c`

- [x] **Step 1: Enforce one pending direct command**

Ensure `HYD_MotionControlFB_StartDirectCommand` rejects a second pending direct command with `HYD_DIAG_CODE_COMMAND_NOT_ALLOWED`:

```c
if (fb->_directPendingValid) {
    if (errorId != NULL) {
        *errorId = HYD_DIAG_CODE_COMMAND_NOT_ALLOWED;
    }
    HYD_SetDiag(HYD_DIAG_CODE_COMMAND_NOT_ALLOWED);
    return false;
}
```

- [x] **Step 2: Start pending direct when active direct completes**

Add pending startup logic to the direct completion path in `src/motion_control.c`:

```c
static HYD_BOOL HYD_StartPendingDirectSlot(HYD_MotionControlFB* fb) {
    HYD_MotionSegment segment;
    HYD_DirectCommandKind kind;

    if (fb == NULL || !fb->_directPendingValid) {
        return false;
    }

    segment = fb->_directPendingSegment;
    kind = fb->_directPendingKind;
    HYD_ClearDirectPendingSlot(fb);
    fb->_directKind = kind;
    return HYD_MotionControlFB_StartSegment(fb, &segment);
}
```

Call it after a finite direct segment reaches its completion condition, before the FB becomes idle.

- [x] **Step 3: Keep abort semantics dominant**

Ensure `Aborting` clears pending and immediately takes ownership:

```c
if (bufferMode == HYD_BUFFER_MODE_ABORT && hasActiveDirect) {
    HYD_AbortActiveExecution(fb);
    HYD_ClearDirectPendingSlot(fb);
    return HYD_MotionControlFB_StartSegment(fb, segment);
}
```

- [x] **Step 4: Run arbitration tests**

Run:

```bash
cmake --build build -j2
./build/test_motion_interface_arbitration
```

Expected after implementation: new BufferMode lifecycle tests pass. Existing unrelated recipe assertions may still fail if they already failed at baseline.

---

### Task 8: Final verification

**Files:**
- Verify: all changed files

- [x] **Step 1: Build**

Run:

```bash
cmake --build build -j2
```

Expected: build completes successfully.

- [x] **Step 2: Run targeted unit tests**

Run:

```bash
./build/test_motion_interface_unit
./build/test_stop_immediate_done
python3 tests/test_interface_layout_consistency.py
```

Expected:

```text
test_motion_interface_unit: all tests pass
test_stop_immediate_done: all tests pass
test_interface_layout_consistency.py: pass
```

- [x] **Step 3: Run full CTest suite**

Run:

```bash
ctest --test-dir build --output-on-failure
```

Expected for this worktree: all CTest tests pass.

Observed after recipe-capacity fix:

```text
100% tests passed, 0 tests failed out of 26
```

- [x] **Step 4: Restore recipe capacity while keeping direct buffer limited**

Set `HYD_MAX_SEGMENTS` in `include/hyd_config.h` to the documented minimum of 4 recipe segments:

```c
#define HYD_MAX_SEGMENTS 4
```

Keep direct FB buffering limited independently through the fixed `_directPendingValid` slot; `HYD_MAX_SEGMENTS` is recipe capacity and must not be used to enforce the direct "current + one pending" embedded constraint.

---

## Scope Notes

- This implementation accepts and preserves all Beckhoff BufferMode values `0..5`.
- Runtime queue capacity is intentionally limited to current direct command plus one pending direct command.
- `BlendingLow`, `BlendingPrevious`, `BlendingNext`, and `BlendingHigh` currently share the finite pending lifecycle behavior. The selected BufferMode is stored in `_directPendingBufferMode`, but the current planner does not yet have a transition-window hook for physically distinct Low/Previous/Next/High blend curves.
- All blending and live updates remain subject to existing segment validation, safety states, output limits, and stop/hold/fault behavior.

## Self-Review

- Spec coverage: BufferMode enum, one-slot direct pending, Aborting/Buffered/endless fallback, continuous update pins, PressureHandle XML/C layout, and verification are covered.
- Placeholder scan: no placeholder tasks remain.
- Type consistency: public names match the implemented code: `HYD_LiveUpdateRequest`, `HYD_MotionControlFB_ApplyLiveUpdate`, `HYD_MotionControlFB_StartDirectCommand`, and `HYD_BUFFER_MODE_BLENDING_HIGH`.
