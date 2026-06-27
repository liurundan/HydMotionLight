# Direct MoveAbsolute Blending Ownership Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the current direct-command `_executionId` ownership inference with exact ticket-based lifecycle tracking so multi-FB `MoveAbsolute` blending behaves correctly under continuous PLC polling while preserving the existing planner blend curves and one-slot direct buffer model.

**Architecture:** Keep `_executionId` as the core per-segment runtime epoch and keep `_recipeBatchId` as the recipe-side epoch. Add a separate ticket lifecycle for accepted direct commands in `motion_control.c`, propagate those tickets through `motion_interface.c`, and use them for pending acquisition, completion, preemption, and live-update authorization. Do not redesign `motion_planner.c`; only tighten arbitration and `BufferMode` policy around the already-shipped blend math.

**Tech Stack:** C99, matiec IEC structs/macros, CMake preset `unixgcc`, C unit tests under `tests/`, static bounded memory, no heap allocation.

---

## File Structure

- Modify `include/motion_control.h`
  - add direct-ticket data structures and state fields
  - rename `HYD_LiveUpdateRequest.ownerExecutionId` to `ownerTicket`
  - add ticket-based public accessors and extend `HYD_MotionControlFB_StartDirectCommand(...)`
- Modify `src/motion_control.c`
  - allocate tickets for every accepted direct command
  - preserve a requested-ticket until `HYD_BeginSegment(...)` makes the command active
  - preserve a pending-ticket for buffered/blended direct commands
  - keep a bounded preempted-ticket history with capacity `2`
  - keep one completed-ticket record for normal direct completion
- Modify `src/motion_interface.c`
  - rewrite direct `_PENDING` / `_EXEC_ID` handling to use tickets instead of `_executionId`
  - keep waiting buffered/blended commands `BUSY=true, ACTIVE=false`
  - migrate direct live-update requests to `ownerTicket`
- Modify `tests/test_motion_interface_arbitration.c`
  - add multi-FB direct-ticket lifecycle regressions
  - add pending-abort and `BufferMode` fallback coverage
- Modify `tests/test_motion_interface_unit.c`
  - add direct live-update authorization coverage for `ownerTicket`

No `motion_interface.h` or XML POU layout change is required in this round.

### Task 1: Replace direct start / pending acquisition with explicit tickets

**Files:**
- Modify: `tests/test_motion_interface_arbitration.c`
- Modify: `include/motion_control.h`
- Modify: `src/motion_control.c`
- Modify: `src/motion_interface.c`

- [ ] **Step 1: Write the failing arbitration tests for direct tickets**

Add these tests to `tests/test_motion_interface_arbitration.c` above `main()`:

```c
static void test_direct_moveabsolute_start_latches_ticket_before_publish(void) {
    HYD_MOVEABSOLUTE ma;
    IEC_WORD ticket = 0;

    __HydMotion_framework_Init();
    ensure_axes_allocated(1);

    memset(&ma, 0, sizeof(ma));
    IEC_VAL(ma.EN) = true;
    IEC_VAL(ma.EXECUTE) = true;
    ma.EXECUTE0.value = false;
    IEC_VAL(ma.AXISID) = 0;
    IEC_VAL(ma.POSITION) = 100.0f;
    IEC_VAL(ma.VELOCITY) = 50.0f;
    IEC_VAL(ma.ACCELERATION) = 200.0f;
    IEC_VAL(ma.DIRECTION) = 1;
    IEC_VAL(ma.BUFFERMODE) = HYD_BUFFER_MODE_ABORT;

    __mcl_cmd_MoveAbsolute(&ma);

    ticket = IEC_VAL(ma._EXEC_ID);
    ASSERT_TRUE(ticket != 0,
               "MoveAbsolute EXECUTE rising should latch a nonzero direct ticket immediately");
    ASSERT_TRUE(IEC_VAL(ma._PENDING) == true,
               "MoveAbsolute should stay pending until the core publishes ownership");

    advance_non_sim_feedback(0, 0.01f);
    __HydMotion_framework_Publish();

    IEC_VAL(ma.EXECUTE) = true;
    ma.EXECUTE0.value = true;
    __mcl_cmd_MoveAbsolute(&ma);

    ASSERT_TRUE(IEC_VAL(ma._PENDING) == false,
               "MoveAbsolute should clear _PENDING after the ticket becomes owner");
    ASSERT_TRUE(IEC_VAL(ma._EXEC_ID) == ticket,
               "MoveAbsolute should keep the same direct ticket after ownership is acquired");
}

static void test_buffered_moveabsolute_keeps_ticket_pending_until_real_cutover(void) {
    HYD_MOVEABSOLUTE first;
    HYD_MOVEABSOLUTE second;
    IEC_WORD pendingTicket = 0;

    __HydMotion_framework_Init();
    ensure_axes_allocated(1);

    start_moveabsolute_on_axis(0, &first);
    ASSERT_TRUE(IEC_VAL(first._EXEC_ID) != 0,
               "First MoveAbsolute should own a direct ticket before buffering the next command");

    memset(&second, 0, sizeof(second));
    IEC_VAL(second.EN) = true;
    IEC_VAL(second.EXECUTE) = true;
    second.EXECUTE0.value = false;
    IEC_VAL(second.AXISID) = 0;
    IEC_VAL(second.POSITION) = 200.0f;
    IEC_VAL(second.VELOCITY) = 40.0f;
    IEC_VAL(second.ACCELERATION) = 100.0f;
    IEC_VAL(second.DECELERATION) = 100.0f;
    IEC_VAL(second.DIRECTION) = 1;
    IEC_VAL(second.BUFFERMODE) = HYD_BUFFER_MODE_BLENDING_NEXT;

    __mcl_cmd_MoveAbsolute(&second);

    pendingTicket = IEC_VAL(second._EXEC_ID);
    ASSERT_TRUE(pendingTicket != 0,
               "Buffered MoveAbsolute should receive its direct ticket on EXECUTE rising");
    ASSERT_TRUE(IEC_VAL(second._PENDING) == true,
               "Buffered MoveAbsolute should remain pending before cutover");
    ASSERT_TRUE(IEC_VAL(second.ACTIVE) == false,
               "Buffered MoveAbsolute must not report ACTIVE while another command owns the axis");

    for (int step = 0; step < 5; step++) {
        advance_non_sim_feedback(0, 0.01f);
        __HydMotion_framework_Publish();

        IEC_VAL(first.EXECUTE) = true;
        first.EXECUTE0.value = true;
        __mcl_cmd_MoveAbsolute(&first);

        IEC_VAL(second.EXECUTE) = true;
        second.EXECUTE0.value = true;
        __mcl_cmd_MoveAbsolute(&second);

        ASSERT_TRUE(IEC_VAL(second._EXEC_ID) == pendingTicket,
                   "Buffered MoveAbsolute should keep the same ticket while waiting");
        ASSERT_TRUE(IEC_VAL(second._PENDING) == true,
                   "Buffered MoveAbsolute should keep waiting until the real cutover");
        ASSERT_TRUE(IEC_VAL(second.ACTIVE) == false,
                   "Buffered MoveAbsolute should stay inactive until its own ticket becomes owner");
    }
}
```

Register them in `main()`:

```c
    test_direct_moveabsolute_start_latches_ticket_before_publish();
    test_buffered_moveabsolute_keeps_ticket_pending_until_real_cutover();
```

- [ ] **Step 2: Run the arbitration target to verify the new tests fail**

Run:

```bash
cmake --build --preset unixgcc
./out/build/unixgcc/test_motion_interface_arbitration
```

Expected before implementation:

- `test_direct_moveabsolute_start_latches_ticket_before_publish` fails because `_EXEC_ID` is still `0` on `EXECUTE` rising.
- `test_buffered_moveabsolute_keeps_ticket_pending_until_real_cutover` fails because the waiting command either keeps `_EXEC_ID == 0` or clears `_PENDING` and reports `ACTIVE` too early.

- [ ] **Step 3: Add the direct-ticket contract to `include/motion_control.h`**

Add these declarations near `HYD_DirectSessionState` and `HYD_LiveUpdateRequest`:

```c
#define HYD_DIRECT_PREEMPTED_HISTORY_CAPACITY 2U

typedef struct {
    uint16_t ticket;
    HYD_DirectCommandKind kind;
} HYD_DirectTicketRecord;

typedef struct {
    HYD_UINT16 flags;
    HYD_DirectCommandKind ownerKind;
    uint16_t ownerTicket;
    HYD_REAL targetPosition;
    HYD_REAL maxVelocity;
    HYD_REAL maxAcceleration;
    HYD_REAL maxDeceleration;
    HYD_REAL targetPressure;
    HYD_REAL pressureRampRate;
    HYD_MotionDirection direction;
} HYD_LiveUpdateRequest;
```

Replace the current direct-session private fields with:

```c
    HYD_DirectCommandKind _directOwnerKind;
    HYD_DirectSessionState _directSessionState;
    HYD_BOOL _directRequestedValid;
    uint16_t _directRequestedTicket;
    HYD_DirectCommandKind _directRequestedKind;
    uint16_t _directTicketCounter;
    uint16_t _directOwnerTicket;
    uint16_t _directPendingTicket;
    HYD_DirectTicketRecord _directCompletedTicket;
    HYD_DirectTicketRecord _directPreemptedTickets[HYD_DIRECT_PREEMPTED_HISTORY_CAPACITY];
    HYD_UINT8 _directPreemptedCount;
    HYD_BOOL _directPendingValid;
    HYD_MotionSegment _directPendingSegment;
    HYD_DirectCommandKind _directPendingKind;
    HYD_BufferMode _directPendingBufferMode;
    HYD_MotionBlendContext _directBlendContext;
```

Replace the public accessors and direct-start signature with:

```c
uint16_t HYD_MotionControlFB_GetDirectOwnerTicket(const HYD_MotionControlFB* fb);
HYD_BOOL HYD_MotionControlFB_WasDirectTicketPreempted(const HYD_MotionControlFB* fb,
                                                      uint16_t ticket,
                                                      HYD_DirectCommandKind kind);
HYD_BOOL HYD_MotionControlFB_WasDirectTicketCompleted(const HYD_MotionControlFB* fb,
                                                      uint16_t ticket,
                                                      HYD_DirectCommandKind kind);
HYD_BOOL HYD_MotionControlFB_ConsumeDirectTicketCompleted(HYD_MotionControlFB* fb,
                                                          uint16_t ticket,
                                                          HYD_DirectCommandKind kind);
HYD_BOOL HYD_MotionControlFB_StartDirectCommand(HYD_MotionControlFB* fb,
                                                const HYD_MotionSegment* segment,
                                                HYD_BufferMode bufferMode,
                                                HYD_TIME timestamp,
                                                uint16_t* acceptedTicket,
                                                HYD_BOOL* queuedBehindActive);
```

- [ ] **Step 4: Implement ticket allocation, requested-ticket carryover, and pending-ticket promotion in `src/motion_control.c`**

Add these helpers near the other direct-session utilities:

```c
static uint16_t HYD_NextDirectTicket(HYD_MotionControlFB* fb) {
    if (fb == NULL) {
        return 0U;
    }

    fb->_directTicketCounter++;
    if (fb->_directTicketCounter == 0U) {
        fb->_directTicketCounter = 1U;
    }
    return fb->_directTicketCounter;
}

static void HYD_ClearRequestedDirectTicket(HYD_MotionControlFB* fb) {
    if (fb == NULL) {
        return;
    }
    fb->_directRequestedValid = false;
    fb->_directRequestedTicket = 0U;
    fb->_directRequestedKind = HYD_DIRECT_CMD_NONE;
}

static void HYD_ArmRequestedDirectTicket(HYD_MotionControlFB* fb,
                                         uint16_t ticket,
                                         HYD_DirectCommandKind kind) {
    if (fb == NULL) {
        return;
    }
    fb->_directRequestedValid = (ticket != 0U && kind != HYD_DIRECT_CMD_NONE);
    fb->_directRequestedTicket = ticket;
    fb->_directRequestedKind = kind;
}

static void HYD_ClearDirectPreemptedTickets(HYD_MotionControlFB* fb) {
    if (fb == NULL) {
        return;
    }
    memset(fb->_directPreemptedTickets, 0, sizeof(fb->_directPreemptedTickets));
    fb->_directPreemptedCount = 0U;
}

static void HYD_RecordCompletedDirectTicket(HYD_MotionControlFB* fb) {
    if (fb == NULL) {
        return;
    }
    fb->_directCompletedTicket.ticket = fb->_directOwnerTicket;
    fb->_directCompletedTicket.kind = fb->_directOwnerKind;
}
```

Update `HYD_ClearDirectPendingSlot(...)` so it also clears `_directPendingTicket`, and update `HYD_StartPendingDirectSlot(...)` so it preserves the pending ticket across cutover:

```c
void HYD_ClearDirectPendingSlot(HYD_MotionControlFB* fb) {
    if (fb == NULL) {
        return;
    }

    fb->_directPendingValid = false;
    fb->_directPendingTicket = 0U;
    memset(&fb->_directPendingSegment, 0, sizeof(fb->_directPendingSegment));
    fb->_directPendingKind = HYD_DIRECT_CMD_NONE;
    fb->_directPendingBufferMode = HYD_BUFFER_MODE_ABORT;
    HYD_ClearDirectBlendContext(fb);
}

static HYD_BOOL HYD_StartPendingDirectSlot(HYD_MotionControlFB* fb,
                                           HYD_TIME timestamp,
                                           HYD_BOOL preservePlannerState) {
    HYD_MotionSegment segment;
    HYD_DirectCommandKind kind;
    uint16_t ticket;
    HYD_BOOL savedUseRecipe;
    HYD_MotionPlannerState preservedPlannerState;

    if (fb == NULL || !fb->_directPendingValid) {
        return false;
    }

    segment = fb->_directPendingSegment;
    kind = fb->_directPendingKind;
    ticket = fb->_directPendingTicket;
    preservedPlannerState = fb->_plannerState;

    savedUseRecipe = fb->USE_RECIPE;
    fb->DIRECT_SEGMENT = segment;
    fb->DIRECT_SEGMENT_VALID = true;
    fb->USE_RECIPE = false;
    fb->_recipeBatchId++;
    HYD_ArmRequestedDirectTicket(fb, ticket, kind);

    if (!HYD_BeginSegment(fb, 0U, timestamp)) {
        HYD_ClearRequestedDirectTicket(fb);
        fb->USE_RECIPE = savedUseRecipe;
        return false;
    }

    HYD_ClearDirectPendingSlot(fb);
    if (preservePlannerState) {
        fb->_plannerState = preservedPlannerState;
    }
    fb->USE_RECIPE = savedUseRecipe;
    return true;
}
```

Modify `HYD_BeginSegment(...)` so a direct segment takes ownership of the requested ticket instead of mirroring `_executionId`:

```c
    fb->_executionId++;
    if (resolvedSource == HYD_SEGMENT_SOURCE_DIRECT) {
        fb->_directOwnerKind = HYD_InferDirectCommandKindFromSegment(sourceSegment);
        fb->_directSessionState = HYD_DIRECT_SESSION_RUNNING;
        if (fb->_directRequestedValid &&
            fb->_directRequestedKind == fb->_directOwnerKind) {
            fb->_directOwnerTicket = fb->_directRequestedTicket;
            HYD_ClearRequestedDirectTicket(fb);
        } else {
            fb->_directOwnerTicket = 0U;
        }
    }
```

Finally, replace `HYD_MotionControlFB_StartDirectCommand(...)` with a ticket-aware version:

```c
HYD_BOOL HYD_MotionControlFB_StartDirectCommand(HYD_MotionControlFB* fb,
                                                const HYD_MotionSegment* segment,
                                                HYD_BufferMode bufferMode,
                                                HYD_TIME timestamp,
                                                uint16_t* acceptedTicket,
                                                HYD_BOOL* queuedBehindActive) {
    HYD_DiagnosticCode code = HYD_DIAG_CODE_NONE;
    HYD_DirectCommandKind kind;
    HYD_BOOL savedUseRecipe;
    HYD_BOOL activeDirect;
    HYD_BOOL shouldAbort;
    uint16_t ticket;

    if (acceptedTicket != NULL) {
        *acceptedTicket = 0U;
    }
    if (queuedBehindActive != NULL) {
        *queuedBehindActive = false;
    }
    if (fb == NULL || segment == NULL) {
        return false;
    }
    if (!HYD_RecipeValidator_ValidateSegment(segment, HYD_MAX_SEGMENTS, &code, &fb->cylinderConfig)) {
        HYD_StateReporter_ReportDiagnostic(fb, code, HYD_DIAG_SEVERITY_WARNING, timestamp, NULL, NULL);
        return false;
    }

    kind = HYD_InferDirectCommandKindFromSegment(segment);
    ticket = HYD_NextDirectTicket(fb);
    if (acceptedTicket != NULL) {
        *acceptedTicket = ticket;
    }

    activeDirect = fb->STATE.active &&
                   fb->_activeSegmentValid &&
                   fb->_activeSegmentSource == HYD_SEGMENT_SOURCE_DIRECT;
    shouldAbort = (bufferMode == HYD_BUFFER_MODE_ABORT &&
                   (fb->STATE.active || HYD_MotionControlFB_IsBusy(fb))) ||
                  (activeDirect && HYD_IsSegmentEndlessForBuffering(&fb->_activeSegment));

    if (shouldAbort) {
        HYD_ClearDirectPendingSlot(fb);
        fb->DIRECT_SEGMENT = *segment;
        fb->DIRECT_SEGMENT_VALID = true;
        savedUseRecipe = fb->USE_RECIPE;
        fb->USE_RECIPE = false;
        HYD_ArmRequestedDirectTicket(fb, ticket, kind);
        if (activeDirect || fb->STATE.active || HYD_MotionControlFB_IsBusy(fb)) {
            HYD_AbortNow(fb, timestamp);
        }
        if (!HYD_BeginSegment(fb, 0U, timestamp)) {
            HYD_ClearRequestedDirectTicket(fb);
            fb->USE_RECIPE = savedUseRecipe;
            return false;
        }
        fb->USE_RECIPE = savedUseRecipe;
        return true;
    }

    if (bufferMode != HYD_BUFFER_MODE_ABORT &&
        (activeDirect || fb->STATE.active || HYD_MotionControlFB_IsBusy(fb))) {
        if (fb->_directPendingValid) {
            HYD_StateReporter_ReportDiagnostic(fb,
                                               HYD_DIAG_CODE_COMMAND_NOT_ALLOWED,
                                               HYD_DIAG_SEVERITY_WARNING,
                                               timestamp,
                                               &fb->_activeSegment,
                                               &fb->STATE.references);
            return false;
        }
        fb->_directPendingSegment = *segment;
        fb->_directPendingKind = kind;
        fb->_directPendingTicket = ticket;
        fb->_directPendingBufferMode = bufferMode;
        fb->_directPendingValid = true;
        if (queuedBehindActive != NULL) {
            *queuedBehindActive = true;
        }
        (void)HYD_TryCreateDirectBlendContext(fb, bufferMode, segment);
        return true;
    }

    fb->DIRECT_SEGMENT = *segment;
    fb->DIRECT_SEGMENT_VALID = true;
    savedUseRecipe = fb->USE_RECIPE;
    fb->USE_RECIPE = false;
    HYD_ArmRequestedDirectTicket(fb, ticket, kind);
    if (!HYD_MotionControlFB_StartSegment(fb, 0U, timestamp)) {
        HYD_ClearRequestedDirectTicket(fb);
        fb->USE_RECIPE = savedUseRecipe;
        return false;
    }
    fb->USE_RECIPE = savedUseRecipe;
    return true;
}
```

Initialize and soft-reset the new ticket fields anywhere the old `_directOwnerExecutionId` / `_lastPreemptedExecutionId` / `_lastCompletedExecutionId` fields were zeroed.

- [ ] **Step 5: Migrate `src/motion_interface.c` direct start / pending acquisition to tickets**

Update the ownership header comment so it describes tickets, not direct `_executionId` epochs:

```c
 * 阶段1 (_PENDING=true): execRising 后, direct FB 立即拿到一个稳定 ticket，
 * 写入 _EXEC_ID，但仍等待核心引擎把该 ticket 提升为 active owner。
 * 阶段2 (_EXEC_ID != 0): direct FB 持有自己的 ticket，通过 owner ticket /
 * preempted ticket / completed ticket 查询生命周期，而不再把 _executionId
 * 误当作 IEC 所有权标识。
```

Replace the direct pending resolver and helper predicates with ticket-based versions:

```c
static HYD_DirectPendingStatus resolveDirectPendingTicket(const HYD_MotionControlFB* fb,
                                                          IEC_WORD ticket,
                                                          HYD_DirectCommandKind kind)
{
    if (fb == NULL || ticket == 0) {
        return HYD_DIRECT_PENDING_WAITING;
    }

    if ((uint16_t)ticket == HYD_MotionControlFB_GetDirectOwnerTicket(fb) &&
        HYD_MotionControlFB_GetDirectOwnerKind(fb) == kind) {
        return HYD_DIRECT_PENDING_ACQUIRED;
    }

    if (HYD_MotionControlFB_WasDirectTicketPreempted(fb, (uint16_t)ticket, kind)) {
        return HYD_DIRECT_PENDING_ABORTED;
    }

    return HYD_DIRECT_PENDING_WAITING;
}

static HYD_BOOL directTicketWasCompleted(HYD_MotionControlFB* fb,
                                         IEC_WORD ticket,
                                         HYD_DirectCommandKind kind)
{
    return HYD_MotionControlFB_ConsumeDirectTicketCompleted(fb, (uint16_t)ticket, kind);
}

static HYD_BOOL directTicketIsCurrentOwner(const HYD_MotionControlFB* fb,
                                           IEC_WORD ticket,
                                           HYD_DirectCommandKind kind)
{
    return ticket == (IEC_WORD)HYD_MotionControlFB_GetDirectOwnerTicket(fb) &&
           HYD_MotionControlFB_GetDirectOwnerKind(fb) == kind;
}
```

Extend `startDirectSegmentExecution(...)` to return the accepted ticket and whether the command is queued behind an already-active owner:

```c
static HYD_BOOL startDirectSegmentExecution(HYD_MotionControlFB* fb,
                                            IEC_INT bufferMode,
                                            const HYD_MotionSegment* segment,
                                            IEC_WORD* acceptedTicket,
                                            IEC_BOOL* queuedBehindActive,
                                            IEC_WORD* errorId)
{
    uint16_t rawTicket = 0U;
    HYD_BOOL rawQueuedBehindActive = false;

    if (errorId != NULL) {
        *errorId = (IEC_WORD)0;
    }
    if (acceptedTicket != NULL) {
        *acceptedTicket = (IEC_WORD)0;
    }
    if (queuedBehindActive != NULL) {
        *queuedBehindActive = false;
    }
    if (fb == NULL || segment == NULL) {
        if (errorId != NULL) {
            *errorId = (IEC_WORD)HYD_DIAG_CODE_INTERNAL_ERROR;
        }
        return false;
    }
    if (!HYD_MotionControlFB_StartDirectCommand(fb,
                                                segment,
                                                (HYD_BufferMode)bufferMode,
                                                fb->AXIS_REF.timestamp,
                                                &rawTicket,
                                                &rawQueuedBehindActive)) {
        if (errorId != NULL) {
            *errorId = commandFailureErrorId(fb);
        }
        return false;
    }

    if (acceptedTicket != NULL) {
        *acceptedTicket = (IEC_WORD)rawTicket;
    }
    if (queuedBehindActive != NULL) {
        *queuedBehindActive = rawQueuedBehindActive ? true : false;
    }
    return true;
}
```

Update `__mcl_cmd_MoveAbsolute(...)` to latch the ticket immediately and keep queued buffered commands inactive while waiting:

```c
        IEC_WORD acceptedTicket = 0;
        IEC_BOOL queuedBehindActive = false;
        if (!startDirectSegmentExecution(fb, bufferMode, &segment,
                                         &acceptedTicket,
                                         &queuedBehindActive,
                                         &errorId)) {
            __SET_VAR(data__->, ERROR, , true);
            __SET_VAR(data__->, ERRORID, , errorId);
            __SET_VAR(data__->, EXECUTE0, , execute);
            return;
        }

        __SET_VAR(data__->, _PENDING, , true);
        __SET_VAR(data__->, _EXEC_ID, , acceptedTicket);
        __SET_VAR(data__->, BUSY, , true);
        __SET_VAR(data__->, ACTIVE, , queuedBehindActive ? false : true);
        __SET_VAR(data__->, DONE, , false);
        __SET_VAR(data__->, COMMANDABORTED, , false);
```

And replace the waiting branch with:

```c
    if (isPending)
    {
        HYD_DirectPendingStatus pendingStatus =
            resolveDirectPendingTicket(fb, myExecId, HYD_DIRECT_CMD_MOVE_ABSOLUTE);

        if (pendingStatus == HYD_DIRECT_PENDING_ACQUIRED) {
            __SET_VAR(data__->, _PENDING, , false);
            __SET_VAR(data__->, ACTIVE, , true);
        } else if (pendingStatus == HYD_DIRECT_PENDING_ABORTED) {
            __SET_VAR(data__->, COMMANDABORTED, , true);
            __SET_VAR(data__->, BUSY, , false);
            __SET_VAR(data__->, ACTIVE, , false);
            __SET_VAR(data__->, _PENDING, , false);
            __SET_VAR(data__->, EXECUTE0, , execute);
            return;
        } else {
            __SET_VAR(data__->, BUSY, , true);
            __SET_VAR(data__->, ACTIVE, , false);
            __SET_VAR(data__->, EXECUTE0, , execute);
            return;
        }
    }
```

Use the same `acceptedTicket` / `queuedBehindActive` and ticket predicates for `MoveVelocity` and `PressureHandle` start / pending / owner branches so all direct FBs share one contract.

- [ ] **Step 6: Re-run the arbitration target and confirm the new tests pass**

Run:

```bash
cmake --build --preset unixgcc
./out/build/unixgcc/test_motion_interface_arbitration
```

Expected after implementation:

- both new tests pass
- existing arbitration tests still pass

- [ ] **Step 7: Commit the direct-ticket foundation**

Run:

```bash
git add include/motion_control.h src/motion_control.c src/motion_interface.c tests/test_motion_interface_arbitration.c
git commit -m "Make direct command ownership explicit under PLC polling" -m "Constraint: MoveAbsolute blending already has core motion math, so this change only replaces the IEC ownership identity and start/pending contract
Rejected: Reusing _executionId as the direct FB lifecycle key | It lets a waiting buffered command look active before real cutover
Confidence: high
Scope-risk: moderate
Directive: Keep direct tickets stable across accepted -> pending -> owner -> done/commandaborted, and keep _executionId core-private
Tested: cmake --build --preset unixgcc && ./out/build/unixgcc/test_motion_interface_arbitration
Not-tested: Full ctest deferred until later tasks migrate completion/preemption history and live-update authorization"
```

### Task 2: Record both active and pending ticket invalidations

**Files:**
- Modify: `tests/test_motion_interface_arbitration.c`
- Modify: `src/motion_control.c`
- Modify: `src/motion_interface.c`

- [ ] **Step 1: Add failing tests for pending-ticket invalidation**

Add these tests to `tests/test_motion_interface_arbitration.c` above `main()`:

```c
static void test_pending_moveabsolute_is_commandaborted_by_third_abort_takeover(void) {
    HYD_MOVEABSOLUTE first;
    HYD_MOVEABSOLUTE second;
    HYD_MOVEABSOLUTE third;

    __HydMotion_framework_Init();
    ensure_axes_allocated(1);

    start_moveabsolute_on_axis(0, &first);

    memset(&second, 0, sizeof(second));
    IEC_VAL(second.EN) = true;
    IEC_VAL(second.EXECUTE) = true;
    second.EXECUTE0.value = false;
    IEC_VAL(second.AXISID) = 0;
    IEC_VAL(second.POSITION) = 200.0f;
    IEC_VAL(second.VELOCITY) = 30.0f;
    IEC_VAL(second.ACCELERATION) = 100.0f;
    IEC_VAL(second.DECELERATION) = 100.0f;
    IEC_VAL(second.DIRECTION) = 1;
    IEC_VAL(second.BUFFERMODE) = HYD_BUFFER_MODE_BLENDING_NEXT;
    __mcl_cmd_MoveAbsolute(&second);
    ASSERT_TRUE(IEC_VAL(second._PENDING) == true,
               "Second MoveAbsolute should be pending before the third takeover");

    memset(&third, 0, sizeof(third));
    IEC_VAL(third.EN) = true;
    IEC_VAL(third.EXECUTE) = true;
    third.EXECUTE0.value = false;
    IEC_VAL(third.AXISID) = 0;
    IEC_VAL(third.POSITION) = 300.0f;
    IEC_VAL(third.VELOCITY) = 60.0f;
    IEC_VAL(third.ACCELERATION) = 100.0f;
    IEC_VAL(third.DECELERATION) = 100.0f;
    IEC_VAL(third.DIRECTION) = 1;
    IEC_VAL(third.BUFFERMODE) = HYD_BUFFER_MODE_ABORT;
    __mcl_cmd_MoveAbsolute(&third);

    advance_non_sim_feedback(0, 0.01f);
    __HydMotion_framework_Publish();

    IEC_VAL(second.EXECUTE) = true;
    second.EXECUTE0.value = true;
    __mcl_cmd_MoveAbsolute(&second);
    ASSERT_TRUE(IEC_VAL(second.COMMANDABORTED) == true,
               "Pending MoveAbsolute should report COMMANDABORTED after a third abort takeover discards it");
    ASSERT_TRUE(IEC_VAL(second._PENDING) == false,
               "Pending MoveAbsolute should clear _PENDING after being discarded");

    IEC_VAL(third.EXECUTE) = true;
    third.EXECUTE0.value = true;
    __mcl_cmd_MoveAbsolute(&third);
    ASSERT_TRUE(IEC_VAL(third.ACTIVE) || IEC_VAL(third.BUSY),
               "Third MoveAbsolute should own the axis after abort takeover");
}

static void test_stop_aborts_waiting_moveabsolute_ticket(void) {
    HYD_MOVEABSOLUTE first;
    HYD_MOVEABSOLUTE second;
    HYD_STOP stop;

    __HydMotion_framework_Init();
    ensure_axes_allocated(1);

    start_moveabsolute_on_axis(0, &first);

    memset(&second, 0, sizeof(second));
    IEC_VAL(second.EN) = true;
    IEC_VAL(second.EXECUTE) = true;
    second.EXECUTE0.value = false;
    IEC_VAL(second.AXISID) = 0;
    IEC_VAL(second.POSITION) = 200.0f;
    IEC_VAL(second.VELOCITY) = 30.0f;
    IEC_VAL(second.ACCELERATION) = 100.0f;
    IEC_VAL(second.DECELERATION) = 100.0f;
    IEC_VAL(second.DIRECTION) = 1;
    IEC_VAL(second.BUFFERMODE) = HYD_BUFFER_MODE_BLENDING_NEXT;
    __mcl_cmd_MoveAbsolute(&second);
    ASSERT_TRUE(IEC_VAL(second._PENDING) == true,
               "Second MoveAbsolute should be pending before Stop");

    memset(&stop, 0, sizeof(stop));
    IEC_VAL(stop.EN) = true;
    IEC_VAL(stop.EXECUTE) = true;
    stop.EXECUTE0.value = false;
    IEC_VAL(stop.AXISID) = 0;
    __mcl_cmd_Stop(&stop);

    advance_non_sim_feedback(0, 0.01f);
    __HydMotion_framework_Publish();

    IEC_VAL(second.EXECUTE) = true;
    second.EXECUTE0.value = true;
    __mcl_cmd_MoveAbsolute(&second);
    ASSERT_TRUE(IEC_VAL(second.COMMANDABORTED) == true,
               "Pending MoveAbsolute should report COMMANDABORTED when Stop clears the pending slot");
}
```

Register them in `main()`:

```c
    test_pending_moveabsolute_is_commandaborted_by_third_abort_takeover();
    test_stop_aborts_waiting_moveabsolute_ticket();
```

- [ ] **Step 2: Run the arbitration target and verify the new tests fail**

Run:

```bash
cmake --build --preset unixgcc
./out/build/unixgcc/test_motion_interface_arbitration
```

Expected before implementation:

- at least one new test fails because the core only remembers one preempted execution and does not record discarded pending commands

- [ ] **Step 3: Replace single preemption bookkeeping with a bounded ticket history**

In `src/motion_control.c`, add these helpers:

```c
static void HYD_RecordPreemptedDirectTicket(HYD_MotionControlFB* fb,
                                            uint16_t ticket,
                                            HYD_DirectCommandKind kind) {
    if (fb == NULL || ticket == 0U || kind == HYD_DIRECT_CMD_NONE) {
        return;
    }

    if (fb->_directPreemptedCount == HYD_DIRECT_PREEMPTED_HISTORY_CAPACITY) {
        memmove(&fb->_directPreemptedTickets[0],
                &fb->_directPreemptedTickets[1],
                sizeof(fb->_directPreemptedTickets[0]) *
                (HYD_DIRECT_PREEMPTED_HISTORY_CAPACITY - 1U));
        fb->_directPreemptedCount--;
    }

    fb->_directPreemptedTickets[fb->_directPreemptedCount].ticket = ticket;
    fb->_directPreemptedTickets[fb->_directPreemptedCount].kind = kind;
    fb->_directPreemptedCount++;
}

static void HYD_DiscardPendingDirectSlot(HYD_MotionControlFB* fb) {
    if (fb == NULL || !fb->_directPendingValid) {
        return;
    }

    HYD_RecordPreemptedDirectTicket(fb,
                                    fb->_directPendingTicket,
                                    fb->_directPendingKind);
    HYD_ClearDirectPendingSlot(fb);
}
```

Use `HYD_RecordPreemptedDirectTicket(...)` anywhere the active direct owner is forcibly replaced:

```c
        HYD_RecordPreemptedDirectTicket(fb,
                                        fb->_directOwnerTicket,
                                        fb->_directOwnerKind);
```

Use `HYD_DiscardPendingDirectSlot(...)` instead of `HYD_ClearDirectPendingSlot(...)` on these paths:

- abort-takeover in `HYD_MotionControlFB_StartDirectCommand(...)`
- Stop / Abort / Reset / Fault teardown branches
- any direct-session hard reset path that discards a pending command without promotion

Replace the old accessors with ticket-based implementations:

```c
uint16_t HYD_MotionControlFB_GetDirectOwnerTicket(const HYD_MotionControlFB* fb) {
    return (fb != NULL) ? fb->_directOwnerTicket : 0U;
}

HYD_BOOL HYD_MotionControlFB_WasDirectTicketPreempted(const HYD_MotionControlFB* fb,
                                                      uint16_t ticket,
                                                      HYD_DirectCommandKind kind) {
    if (fb == NULL || ticket == 0U || kind == HYD_DIRECT_CMD_NONE) {
        return false;
    }

    for (HYD_UINT8 i = 0U; i < fb->_directPreemptedCount; i++) {
        if (fb->_directPreemptedTickets[i].ticket == ticket &&
            fb->_directPreemptedTickets[i].kind == kind) {
            return true;
        }
    }
    return false;
}

HYD_BOOL HYD_MotionControlFB_WasDirectTicketCompleted(const HYD_MotionControlFB* fb,
                                                      uint16_t ticket,
                                                      HYD_DirectCommandKind kind) {
    return (fb != NULL) &&
           fb->_directCompletedTicket.ticket == ticket &&
           fb->_directCompletedTicket.kind == kind;
}

HYD_BOOL HYD_MotionControlFB_ConsumeDirectTicketCompleted(HYD_MotionControlFB* fb,
                                                          uint16_t ticket,
                                                          HYD_DirectCommandKind kind) {
    if (fb == NULL ||
        fb->_directCompletedTicket.ticket != ticket ||
        fb->_directCompletedTicket.kind != kind) {
        return false;
    }

    fb->_directCompletedTicket.ticket = 0U;
    fb->_directCompletedTicket.kind = HYD_DIRECT_CMD_NONE;
    return true;
}
```

Update `src/motion_interface.c` to call `HYD_MotionControlFB_WasDirectTicketPreempted(...)` and `HYD_MotionControlFB_ConsumeDirectTicketCompleted(...)`.

- [ ] **Step 4: Re-run the arbitration target and confirm the invalidation tests pass**

Run:

```bash
cmake --build --preset unixgcc
./out/build/unixgcc/test_motion_interface_arbitration
```

Expected after implementation:

- both new invalidation tests pass
- the earlier direct-ticket tests remain green

- [ ] **Step 5: Commit the bounded preemption history**

Run:

```bash
git add src/motion_control.c src/motion_interface.c tests/test_motion_interface_arbitration.c
git commit -m "Preserve direct pending cancellation facts across takeovers" -m "Constraint: One scan can invalidate both the current owner and one queued pending direct command, so single-slot preemption memory is insufficient
Rejected: Recording only the active owner's preemption | It loses COMMANDABORTED for a discarded pending direct FB
Confidence: high
Scope-risk: narrow
Directive: Any path that clears an accepted pending direct command without promoting it must record that ticket as preempted first
Tested: cmake --build --preset unixgcc && ./out/build/unixgcc/test_motion_interface_arbitration
Not-tested: Live-update authorization still uses ownerExecutionId until the next task"
```

### Task 3: Migrate live updates and lock BufferMode policy for MoveVelocity / PressureHandle

**Files:**
- Modify: `tests/test_motion_interface_unit.c`
- Modify: `tests/test_motion_interface_arbitration.c`
- Modify: `include/motion_control.h`
- Modify: `src/motion_control.c`
- Modify: `src/motion_interface.c`

- [ ] **Step 1: Add the failing unit and arbitration coverage**

Add this unit test to `tests/test_motion_interface_unit.c` above `main()`:

```c
static void test_apply_live_update_uses_owner_ticket(void) {
    HYD_MOVEABSOLUTE ma;
    HYD_MotionControlFB* fb;
    HYD_LiveUpdateRequest request;
    uint16_t ownerTicket;

    __HydMotion_framework_Init();
    ensure_axes_allocated(1);
    start_moveabsolute_on_axis(0, &ma);

    fb = __MK_GetPublic_MotionControlFB(0);
    ASSERT_TRUE(fb != NULL, "Axis 0 FB should exist for owner-ticket live update test");

    ownerTicket = fb->_directOwnerTicket;
    ASSERT_TRUE(ownerTicket != 0U, "Active MoveAbsolute should expose a nonzero owner ticket");

    memset(&request, 0, sizeof(request));
    request.flags = HYD_LIVE_UPDATE_TARGET_POSITION |
                    HYD_LIVE_UPDATE_MAX_VELOCITY |
                    HYD_LIVE_UPDATE_ACCELERATION |
                    HYD_LIVE_UPDATE_DECELERATION;
    request.ownerKind = HYD_DIRECT_CMD_MOVE_ABSOLUTE;
    request.ownerTicket = (uint16_t)(ownerTicket + 1U);
    request.targetPosition = 150.0f;
    request.maxVelocity = 60.0f;
    request.maxAcceleration = 100.0f;
    request.maxDeceleration = 100.0f;

    ASSERT_TRUE(!HYD_MotionControlFB_ApplyLiveUpdate(fb, &request),
               "ApplyLiveUpdate should reject the wrong direct owner ticket");

    request.ownerTicket = ownerTicket;
    ASSERT_TRUE(HYD_MotionControlFB_ApplyLiveUpdate(fb, &request),
               "ApplyLiveUpdate should accept the current direct owner ticket");
}
```

Add these arbitration tests to `tests/test_motion_interface_arbitration.c` above `main()`:

```c
static void test_blending_endless_movevelocity_degrades_to_abort_takeover(void) {
    HYD_MOVEVELOCITY first;
    HYD_MOVEABSOLUTE second;

    __HydMotion_framework_Init();
    ensure_axes_allocated(1);

    start_movevelocity_on_axis(0, &first);

    memset(&second, 0, sizeof(second));
    IEC_VAL(second.EN) = true;
    IEC_VAL(second.EXECUTE) = true;
    second.EXECUTE0.value = false;
    IEC_VAL(second.AXISID) = 0;
    IEC_VAL(second.POSITION) = 120.0f;
    IEC_VAL(second.VELOCITY) = 45.0f;
    IEC_VAL(second.ACCELERATION) = 100.0f;
    IEC_VAL(second.DECELERATION) = 100.0f;
    IEC_VAL(second.DIRECTION) = 1;
    IEC_VAL(second.BUFFERMODE) = HYD_BUFFER_MODE_BLENDING_NEXT;
    __mcl_cmd_MoveAbsolute(&second);
    __HydMotion_framework_Publish();

    IEC_VAL(first.EXECUTE) = true;
    first.EXECUTE0.value = true;
    __mcl_cmd_MoveVelocity(&first);
    ASSERT_TRUE(IEC_VAL(first.COMMANDABORTED) == true,
               "BLENDING_* after endless MoveVelocity should degrade to abort takeover");
}

static void test_blending_endless_pressurehandle_degrades_to_abort_takeover(void) {
    HYD_PRESSUREHANDLE first;
    HYD_MOVEABSOLUTE second;

    __HydMotion_framework_Init();
    ensure_axes_allocated(1);

    memset(&first, 0, sizeof(first));
    IEC_VAL(first.EN) = true;
    IEC_VAL(first.EXECUTE) = true;
    first.EXECUTE0.value = false;
    IEC_VAL(first.AXISID) = 0;
    IEC_VAL(first.PRESSURE) = 80.0f;
    IEC_VAL(first.PRESSURERAMPRATE) = 50.0f;
    IEC_VAL(first.DURATION) = 0.0f;
    IEC_VAL(first.BUFFERMODE) = HYD_BUFFER_MODE_ABORT;
    __mcl_cmd_PressureHandle(&first);
    __HydMotion_framework_Publish();

    memset(&second, 0, sizeof(second));
    IEC_VAL(second.EN) = true;
    IEC_VAL(second.EXECUTE) = true;
    second.EXECUTE0.value = false;
    IEC_VAL(second.AXISID) = 0;
    IEC_VAL(second.POSITION) = 150.0f;
    IEC_VAL(second.VELOCITY) = 40.0f;
    IEC_VAL(second.ACCELERATION) = 100.0f;
    IEC_VAL(second.DECELERATION) = 100.0f;
    IEC_VAL(second.DIRECTION) = 1;
    IEC_VAL(second.BUFFERMODE) = HYD_BUFFER_MODE_BLENDING_HIGH;
    __mcl_cmd_MoveAbsolute(&second);
    __HydMotion_framework_Publish();

    IEC_VAL(first.EXECUTE) = true;
    first.EXECUTE0.value = true;
    __mcl_cmd_PressureHandle(&first);
    ASSERT_TRUE(IEC_VAL(first.COMMANDABORTED) == true,
               "BLENDING_* after no-duration PressureHandle should degrade to abort takeover");
}

static void test_timed_pressurehandle_blending_waits_as_plain_buffer(void) {
    HYD_PRESSUREHANDLE first;
    HYD_PRESSUREHANDLE second;

    __HydMotion_framework_Init();
    ensure_axes_allocated(1);

    memset(&first, 0, sizeof(first));
    IEC_VAL(first.EN) = true;
    IEC_VAL(first.EXECUTE) = true;
    first.EXECUTE0.value = false;
    IEC_VAL(first.AXISID) = 0;
    IEC_VAL(first.PRESSURE) = 60.0f;
    IEC_VAL(first.PRESSURERAMPRATE) = 20.0f;
    IEC_VAL(first.DURATION) = 0.2f;
    IEC_VAL(first.BUFFERMODE) = HYD_BUFFER_MODE_ABORT;
    __mcl_cmd_PressureHandle(&first);
    __HydMotion_framework_Publish();

    memset(&second, 0, sizeof(second));
    IEC_VAL(second.EN) = true;
    IEC_VAL(second.EXECUTE) = true;
    second.EXECUTE0.value = false;
    IEC_VAL(second.AXISID) = 0;
    IEC_VAL(second.PRESSURE) = 80.0f;
    IEC_VAL(second.PRESSURERAMPRATE) = 20.0f;
    IEC_VAL(second.DURATION) = 0.2f;
    IEC_VAL(second.BUFFERMODE) = HYD_BUFFER_MODE_BLENDING_HIGH;
    __mcl_cmd_PressureHandle(&second);

    ASSERT_TRUE(IEC_VAL(second._PENDING) == true,
               "Timed PressureHandle should wait in the pending slot");
    ASSERT_TRUE(IEC_VAL(second.ACTIVE) == false,
               "Timed PressureHandle should remain inactive while waiting behind another command");
    ASSERT_TRUE(IEC_VAL(second._EXEC_ID) != 0,
               "Timed PressureHandle should still receive a direct ticket immediately");
}
```

Register them in the relevant `main()` functions:

```c
    test_apply_live_update_uses_owner_ticket();
```

```c
    test_blending_endless_movevelocity_degrades_to_abort_takeover();
    test_blending_endless_pressurehandle_degrades_to_abort_takeover();
    test_timed_pressurehandle_blending_waits_as_plain_buffer();
```

- [ ] **Step 2: Run the unit and arbitration targets and verify the new coverage fails**

Run:

```bash
cmake --build --preset unixgcc
./out/build/unixgcc/test_motion_interface_unit
./out/build/unixgcc/test_motion_interface_arbitration
```

Expected before implementation:

- the unit build or run fails because `HYD_LiveUpdateRequest` still uses `ownerExecutionId`
- at least one new arbitration test fails if `MoveVelocity` / `PressureHandle` still follow stale `_EXEC_ID` or `ACTIVE` waiting semantics

- [ ] **Step 3: Migrate live-update authorization and direct helper predicates to `ownerTicket`**

In `include/motion_control.h`, rename the field:

```c
    uint16_t ownerTicket;
```

Update `HYD_MotionControlFB_ApplyLiveUpdate(...)` in `src/motion_control.c`:

```c
    sameOwner = (fb->_activeSegmentSource == HYD_SEGMENT_SOURCE_DIRECT &&
                 fb->_directOwnerKind == request->ownerKind &&
                 fb->_directOwnerTicket == request->ownerTicket);
```

Update `applyMoveAbsoluteLiveUpdate(...)`, `applyMoveVelocityLiveUpdate(...)`, and `applyPressureHandleLiveUpdate(...)` in `src/motion_interface.c`:

```c
    request.ownerTicket = (uint16_t)execId;
```

Replace any remaining direct owner `_executionId` comparisons in `src/motion_interface.c` with ticket comparisons:

```c
static HYD_BOOL directTicketLostOwnership(const HYD_MotionControlFB* fb,
                                          IEC_WORD ticket,
                                          HYD_DirectCommandKind kind)
{
    return ticket != 0 &&
           !directTicketIsCurrentOwner(fb, ticket, kind) &&
           !HYD_MotionControlFB_WasDirectTicketPreempted(fb, (uint16_t)ticket, kind) &&
           !HYD_MotionControlFB_WasDirectTicketCompleted(fb, (uint16_t)ticket, kind);
}
```

Also simplify the recipe-side helper predicates so they no longer depend on `GetDirectOwnerExecutionId(...)`:

```c
static HYD_BOOL recipeExecutionCanAcquireOwnership(const HYD_MotionControlFB* fb)
{
    return fb != NULL &&
           fb->_recipeBatchId != 0U &&
           HYD_MotionControlFB_GetDirectOwnerKind(fb) == HYD_DIRECT_CMD_NONE;
}

static HYD_BOOL recipeExecutionWasTakenOverBeforeLatch(const HYD_MotionControlFB* fb)
{
    return fb != NULL &&
           fb->_recipeBatchId != 0U &&
           HYD_MotionControlFB_GetDirectOwnerKind(fb) != HYD_DIRECT_CMD_NONE &&
           fb->_activeSegmentSource == HYD_SEGMENT_SOURCE_DIRECT;
}
```

- [ ] **Step 4: Re-run the targeted tests and confirm the new coverage passes**

Run:

```bash
cmake --build --preset unixgcc
./out/build/unixgcc/test_motion_interface_unit
./out/build/unixgcc/test_motion_interface_arbitration
```

Expected after implementation:

- the new `ownerTicket` unit test passes
- the new `MoveVelocity` / `PressureHandle` `BufferMode` policy tests pass
- existing unit and arbitration coverage stays green

- [ ] **Step 5: Commit the ownerTicket migration**

Run:

```bash
git add include/motion_control.h src/motion_control.c src/motion_interface.c tests/test_motion_interface_unit.c tests/test_motion_interface_arbitration.c
git commit -m "Keep direct live updates and waiting FBs aligned to the same ticket" -m "Constraint: The IEC adapter now stores a direct ticket in _EXEC_ID, so live-update authorization and waiting-command state must use that same identity
Rejected: Leaving ownerExecutionId in the live-update path | It reintroduces a hidden split between pending/owner state and update authorization
Confidence: high
Scope-risk: moderate
Directive: When extending any direct FB, route _PENDING, COMMANDABORTED, DONE, and live-update authorization through the same direct ticket
Tested: cmake --build --preset unixgcc && ./out/build/unixgcc/test_motion_interface_unit && ./out/build/unixgcc/test_motion_interface_arbitration
Not-tested: Full repository ctest deferred to the final verification task"
```

### Task 4: Full verification

**Files:**
- No code changes expected in this task unless verification exposes a regression

- [ ] **Step 1: Run the focused motion targets**

Run:

```bash
cmake --build --preset unixgcc
./out/build/unixgcc/test_motion_planner
./out/build/unixgcc/test_motion_interface_arbitration
./out/build/unixgcc/test_motion_interface_unit
./out/build/unixgcc/test_moveabsolute_stop_integration
./out/build/unixgcc/test_motion_interface_done_signals
```

Expected:

- all five binaries exit `0`

- [ ] **Step 2: Run the full regression suite**

Run:

```bash
ctest --test-dir out/build/unixgcc --output-on-failure
```

Expected:

- `100% tests passed`

- [ ] **Step 3: If any regression appears, fix it before merging**

Use the smallest fix that preserves the ticket contract. Typical hotspots if something fails:

```c
/* src/motion_control.c */
HYD_RecordCompletedDirectTicket(fb);
HYD_DiscardPendingDirectSlot(fb);
HYD_ClearRequestedDirectTicket(fb);

/* src/motion_interface.c */
resolveDirectPendingTicket(...);
directTicketIsCurrentOwner(...);
directTicketLostOwnership(...);
```

Do not change `motion_planner.c` unless a failure proves the ticket migration accidentally disturbed a previously green blend test.

- [ ] **Step 4: Final verification commit if Task 4 required a fix**

If Step 3 changed code, run:

```bash
git add include/motion_control.h src/motion_control.c src/motion_interface.c tests/test_motion_interface_unit.c tests/test_motion_interface_arbitration.c
git commit -m "Preserve direct BufferMode behavior after ticket migration" -m "Constraint: Final verification found a regression after replacing direct ownership inference with explicit tickets
Rejected: Reverting to _executionId-based IEC ownership | It reopens the multi-FB pending-acquisition bug
Confidence: medium
Scope-risk: narrow
Directive: Keep any final fix local to ticket lifecycle or adapter mapping; planner blend math is not the root cause in this phase
Tested: cmake --build --preset unixgcc && ./out/build/unixgcc/test_motion_planner && ./out/build/unixgcc/test_motion_interface_arbitration && ./out/build/unixgcc/test_motion_interface_unit && ./out/build/unixgcc/test_moveabsolute_stop_integration && ./out/build/unixgcc/test_motion_interface_done_signals && ctest --test-dir out/build/unixgcc --output-on-failure
Not-tested: None"
```

If Step 3 required no code change, skip this commit.

## Self-Review

- Spec coverage:
  - direct ticket lifecycle: Task 1
  - active + pending invalidation: Task 2
  - `ownerTicket` live updates: Task 3
  - `MoveVelocity` / `PressureHandle` fallback and buffered policy: Task 3
  - full regression / planner continuity retention: Task 4
- Placeholder scan:
  - no `TODO`, `TBD`, or deferred design choices remain
- Type consistency:
  - plan consistently uses `ownerTicket`, `GetDirectOwnerTicket`, `WasDirectTicketPreempted`, `WasDirectTicketCompleted`, and `ConsumeDirectTicketCompleted`
