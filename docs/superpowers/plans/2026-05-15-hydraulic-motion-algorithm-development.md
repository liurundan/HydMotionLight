# Hydraulic Motion Algorithm Development Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Close the control-algorithm gaps found in `motion_control.c` and `motion_planner.c` so the hydraulic motion library better supports clamp, injection, ejector, carriage, and holding-pressure actions on servo-pump injection molding machines.

**Architecture:** Keep the existing runtime contract: the library owns motion math, pressure/flow planning, pump-speed conversion, diagnostics, and stable runtime outputs; the PLC process layer keeps valve sequencing, machine interlocks, and machine workflow ownership. Add small focused modules for output constraints, velocity closed-loop correction, action profile defaults, and V/P transfer observation instead of expanding `motion_control.c` further. Each phase is independently testable and keeps existing recipe/direct/IEC behavior intact.

**Tech Stack:** C99, CMake presets, static bounded memory, existing `HYD_` naming style, existing unit tests under `tests/`, no heap allocation, PLCopen-style function-block runtime.

---

## Scope Check

This plan covers one connected algorithm-hardening program for the current motion runtime. It is intentionally ordered so every task leaves the library in a working state:

1. Make existing protection semantics affect outputs.
2. Make existing motion planning physically smoother.
3. Align stop behavior with the deceleration contract.
4. Add optional velocity correction for speed-governed moves.
5. Add action templates and stricter recipe validation for injection-machine actions.
6. Add stable completion semantics.
7. Add V/P transfer observation outputs while leaving transfer ownership with the PLC process layer.

Do not start with V/P transfer or action templates. They depend on reliable output limiting, trajectory generation, stop behavior, and completion semantics.

## File Map

- Create: `include/output_limiter.h`
  Defines a focused output-limiting API that applies diagnostic protection actions to planner and pump outputs.
- Create: `src/output_limiter.c`
  Implements derate behavior, command-flow clamping, and reference synchronization.
- Create: `tests/test_output_limiter.c`
  Unit tests for output derating without involving the full runtime state machine.
- Modify: `CMakeLists.txt`
  Add new test executables. Core `src/*.c` files are already globbed into `HydroMotionLib` after reconfigure.
- Modify: `src/motion_control.c`
  Call output limiter after diagnostics and before `HYD_StateReporter_ReportExecution`; fix stop default deceleration; integrate velocity loop; pass stable completion context; publish V/P transfer observation.
- Modify: `include/motion_planner.h`
  Add persistent planner state and richer input/output fields for trapezoid planning.
- Modify: `src/motion_planner.c`
  Use persistent acceleration-limited velocity evolution for position and speed-ramp modes.
- Modify: `tests/test_motion_planner.c`
  Add continuity and deceleration tests for the runtime planner path.
- Modify: `tests/test_moveabsolute_stop_integration.c`
  Add stop default deceleration contract coverage.
- Create: `include/velocity_controller.h`
  Defines a small bounded velocity correction API.
- Create: `src/velocity_controller.c`
  Implements proportional flow correction with deadband and saturation.
- Create: `tests/test_velocity_controller.c`
  Unit tests for speed feedback correction.
- Modify: `include/common_types.h`
  Add action-profile, stable-completion, velocity-loop, and V/P observation fields in a RAM-conscious way.
- Create: `include/action_profile.h`
  Defines action template helpers for clamp, injection, holding, ejector, and carriage defaults.
- Create: `src/action_profile.c`
  Builds validated `HYD_MotionSegment` defaults from `HYD_MotionFBParams`.
- Create: `tests/test_action_profile.c`
  Unit tests for generated motion profiles and validation compatibility.
- Modify: `src/recipe_validator.c`
  Enforce action-sensitive constraints without taking over machine sequencing.
- Modify: `src/segment_completion.c`
  Add stable completion windows using existing `HYD_SegmentCompletionContext`.
- Modify: `include/segment_completion.h`
  Extend completion context with stable window state.
- Modify: `tests/segment_completion_test.c`
  Add stable completion tests.
- Create: `include/vp_transfer.h`
  Defines V/P transfer observation criteria and result structs.
- Create: `src/vp_transfer.c`
  Computes transfer-ready events from axis feedback, references, and segment configuration.
- Create: `tests/test_vp_transfer.c`
  Unit tests for V/P transfer observation criteria.
- Modify: `docs/architecture/motion-profile-archetypes.md`
  Document the new action profile defaults and what still belongs to the PLC process layer.
- Modify: `docs/architecture/motion-runtime-contract.md`
  Document output derate, acceleration-limited planning, stable completion, velocity loop, and V/P observation outputs.
- Modify: `docs/architecture/implementation-contract-gap-list.md`
  Move implemented items out of the gap list after their tests pass.

## Build And Test Baseline

- [ ] **Step 1: Check the working tree before implementation**

Run:

```bash
git status --short
```

Expected: Existing unrelated documentation changes may be present. Do not revert or edit unrelated user changes.

- [ ] **Step 2: Configure the build**

Run:

```bash
cmake --preset unixgcc
```

Expected: configure completes and regenerates build files.

- [ ] **Step 3: Build the current project**

Run:

```bash
cmake --build --preset unixgcc
```

Expected: all current targets build.

- [ ] **Step 4: Run the current test suite**

Run:

```bash
ctest --test-dir out/build/unixgcc --output-on-failure
```

Expected: all current tests pass before algorithm changes begin.

## Task 1: Output Derate Limiter

**Files:**
- Create: `include/output_limiter.h`
- Create: `src/output_limiter.c`
- Create: `tests/test_output_limiter.c`
- Modify: `src/motion_control.c`
- Modify: `CMakeLists.txt`

**Purpose:** Make `HYD_PROTECTION_ACTION_DERATE` reduce command flow and pump speed instead of only changing diagnostics/status.

- [ ] **Step 1: Write the failing output limiter tests**

Create `tests/test_output_limiter.c` with:

```c
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "output_limiter.h"

static void test_derate_halves_flow_and_pump_speed(void) {
    HYD_OutputLimiterInput input = {0};
    HYD_OutputLimiterOutput output = {0};

    input.requestedFlow = 40.0;
    input.requestedPumpSpeed = 1200.0;
    input.flowToPumpSpeedGain = 30.0;
    input.pumpSpeedLimit = 3000.0;
    input.protectionAction = HYD_PROTECTION_ACTION_DERATE;
    input.derateRatio = 0.5;

    HYD_OutputLimiter_Execute(&input, &output);

    assert(fabs(output.commandFlow - 20.0) < 0.001);
    assert(fabs(output.pumpSpeed - 600.0) < 0.001);
    assert(output.derated);
}

static void test_no_derate_preserves_valid_outputs(void) {
    HYD_OutputLimiterInput input = {0};
    HYD_OutputLimiterOutput output = {0};

    input.requestedFlow = 18.0;
    input.requestedPumpSpeed = 540.0;
    input.flowToPumpSpeedGain = 30.0;
    input.pumpSpeedLimit = 3000.0;
    input.protectionAction = HYD_PROTECTION_ACTION_NONE;
    input.derateRatio = 0.5;

    HYD_OutputLimiter_Execute(&input, &output);

    assert(fabs(output.commandFlow - 18.0) < 0.001);
    assert(fabs(output.pumpSpeed - 540.0) < 0.001);
    assert(!output.derated);
}

static void test_stop_action_forces_safe_zero(void) {
    HYD_OutputLimiterInput input = {0};
    HYD_OutputLimiterOutput output = {0};

    input.requestedFlow = 18.0;
    input.requestedPumpSpeed = 540.0;
    input.flowToPumpSpeedGain = 30.0;
    input.pumpSpeedLimit = 3000.0;
    input.protectionAction = HYD_PROTECTION_ACTION_STOP;
    input.derateRatio = 0.5;

    HYD_OutputLimiter_Execute(&input, &output);

    assert(output.commandFlow == 0.0);
    assert(output.pumpSpeed == 0.0);
    assert(!output.derated);
}

static void test_invalid_ratio_uses_default_derate(void) {
    HYD_OutputLimiterInput input = {0};
    HYD_OutputLimiterOutput output = {0};

    input.requestedFlow = 40.0;
    input.requestedPumpSpeed = 1200.0;
    input.flowToPumpSpeedGain = 30.0;
    input.pumpSpeedLimit = 3000.0;
    input.protectionAction = HYD_PROTECTION_ACTION_DERATE;
    input.derateRatio = 0.0;

    HYD_OutputLimiter_Execute(&input, &output);

    assert(fabs(output.commandFlow - 20.0) < 0.001);
    assert(fabs(output.pumpSpeed - 600.0) < 0.001);
    assert(output.derated);
}

int main(void) {
    test_derate_halves_flow_and_pump_speed();
    test_no_derate_preserves_valid_outputs();
    test_stop_action_forces_safe_zero();
    test_invalid_ratio_uses_default_derate();
    printf("Output limiter tests passed\n");
    return 0;
}
```

- [ ] **Step 2: Add the test target**

Modify `CMakeLists.txt` by adding the executable near the other unit tests:

```cmake
add_executable(test_output_limiter tests/test_output_limiter.c)
target_link_libraries(test_output_limiter PRIVATE HydroMotionLib)
```

Add the CTest entry near the other `add_test` calls:

```cmake
add_test(NAME test_output_limiter COMMAND test_output_limiter)
```

- [ ] **Step 3: Run the failing test**

Run:

```bash
cmake --preset unixgcc
cmake --build --preset unixgcc --target test_output_limiter
```

Expected: build fails because `output_limiter.h` does not exist.

- [ ] **Step 4: Create the output limiter header**

Create `include/output_limiter.h`:

```c
#ifndef HYD_OUTPUT_LIMITER_H
#define HYD_OUTPUT_LIMITER_H

#include "common_types.h"

typedef struct {
    HYD_REAL requestedFlow;
    HYD_REAL requestedPumpSpeed;
    HYD_REAL flowToPumpSpeedGain;
    HYD_REAL pumpSpeedLimit;
    HYD_ProtectionAction protectionAction;
    HYD_REAL derateRatio;
} HYD_OutputLimiterInput;

typedef struct {
    HYD_REAL commandFlow;
    HYD_REAL pumpSpeed;
    HYD_BOOL derated;
} HYD_OutputLimiterOutput;

void HYD_OutputLimiter_Execute(const HYD_OutputLimiterInput* input,
                               HYD_OutputLimiterOutput* output);

#endif /* HYD_OUTPUT_LIMITER_H */
```

- [ ] **Step 5: Implement output limiter**

Create `src/output_limiter.c`:

```c
#include "output_limiter.h"
#include <math.h>

#define HYD_DEFAULT_DERATE_RATIO 0.5

static HYD_BOOL HYD_OutputLimiter_IsFinite(HYD_REAL value) {
    return isfinite(value) ? true : false;
}

static HYD_REAL HYD_OutputLimiter_ResolveDerateRatio(HYD_REAL configuredRatio) {
    if (configuredRatio > 0.0 && configuredRatio < 1.0) {
        return configuredRatio;
    }
    return HYD_DEFAULT_DERATE_RATIO;
}

void HYD_OutputLimiter_Execute(const HYD_OutputLimiterInput* input,
                               HYD_OutputLimiterOutput* output) {
    HYD_REAL commandFlow;
    HYD_REAL pumpSpeed;
    HYD_REAL ratio;

    if (output == NULL) {
        return;
    }

    output->commandFlow = 0.0;
    output->pumpSpeed = 0.0;
    output->derated = false;

    if (input == NULL) {
        return;
    }

    if (input->protectionAction == HYD_PROTECTION_ACTION_STOP) {
        return;
    }

    if (!HYD_OutputLimiter_IsFinite(input->requestedFlow) ||
        !HYD_OutputLimiter_IsFinite(input->requestedPumpSpeed) ||
        !HYD_OutputLimiter_IsFinite(input->flowToPumpSpeedGain) ||
        !HYD_OutputLimiter_IsFinite(input->pumpSpeedLimit) ||
        input->flowToPumpSpeedGain <= 0.0 ||
        input->pumpSpeedLimit < 0.0) {
        return;
    }

    commandFlow = input->requestedFlow;
    if (commandFlow < 0.0) {
        commandFlow = -commandFlow;
    }

    pumpSpeed = input->requestedPumpSpeed;
    if (pumpSpeed < 0.0) {
        pumpSpeed = -pumpSpeed;
    }

    if (input->protectionAction == HYD_PROTECTION_ACTION_DERATE) {
        ratio = HYD_OutputLimiter_ResolveDerateRatio(input->derateRatio);
        commandFlow *= ratio;
        pumpSpeed *= ratio;
        output->derated = true;
    }

    if (pumpSpeed > input->pumpSpeedLimit) {
        pumpSpeed = input->pumpSpeedLimit;
        commandFlow = pumpSpeed / input->flowToPumpSpeedGain;
    }

    output->commandFlow = commandFlow;
    output->pumpSpeed = pumpSpeed;
}
```

- [ ] **Step 6: Run output limiter unit test**

Run:

```bash
cmake --preset unixgcc
cmake --build --preset unixgcc --target test_output_limiter
./out/build/unixgcc/test_output_limiter
```

Expected: executable prints `Output limiter tests passed`.

- [ ] **Step 7: Integrate limiter into runtime execution**

Modify `src/motion_control.c`:

Add include:

```c
#include "output_limiter.h"
```

Inside `HYD_MotionControlFB_RunRunningState`, after:

```c
HYD_UpdateExecutionDiagnostics(fb, segment, &executionReference, elapsed);
```

insert:

```c
{
    HYD_OutputLimiterInput limiterInput;
    HYD_OutputLimiterOutput limiterOutput;

    limiterInput.requestedFlow = pumpOutput.commandFlow;
    limiterInput.requestedPumpSpeed = pumpOutput.pumpSpeed;
    limiterInput.flowToPumpSpeedGain = fb->FLOW_TO_PUMP_SPEED_GAIN;
    limiterInput.pumpSpeedLimit = fb->PUMP_SPEED_LIMIT;
    limiterInput.protectionAction = fb->DIAGNOSTIC.protectionAction;
    limiterInput.derateRatio = 0.5;

    HYD_OutputLimiter_Execute(&limiterInput, &limiterOutput);

    pumpOutput.commandFlow = limiterOutput.commandFlow;
    pumpOutput.pumpSpeed = limiterOutput.pumpSpeed;
    plannerOutput.targetFlow = limiterOutput.commandFlow;
    executionReference.flowReference = limiterOutput.commandFlow;
}
```

Keep the existing `STOP` handling after this block. The runtime already enters fault stop for `HYD_PROTECTION_ACTION_STOP`; this limiter provides safe zero if a STOP action reaches reporting before that branch.

- [ ] **Step 8: Add runtime derate integration test**

Add to `tests/test_sprint_b_integration.c`:

```c
static void test_derate_reduces_runtime_pump_speed(void) {
    HYD_MotionControlFB fb;
    HYD_MotionSegment segment;
    HYD_REAL nominalSpeed;
    HYD_REAL deratedSpeed;

    HYD_MotionControlFB_Init(&fb);
    fb.FLOW_TO_PUMP_SPEED_GAIN = 10.0;
    fb.PUMP_SPEED_LIMIT = 3000.0;
    fb.USE_RECIPE = false;
    fb.AXIS_REF.position = 0.0;
    fb.AXIS_REF.velocity = 0.0;
    fb.AXIS_REF.flow = 0.0;
    fb.AXIS_REF.pressure = 5.0;
    fb.AXIS_REF.timestamp = 0.0;

    memset(&segment, 0, sizeof(segment));
    segment.planner = HYD_PLANNER_TIME_BASED;
    segment.mode = HYD_MODE_PRESSURE_CLOSED_LOOP;
    segment.endCondition = HYD_END_TIME;
    segment.direction = HYD_DIRECTION_HOLD;
    segment.targetPressure = 20.0;
    segment.targetFlow = 20.0;
    segment.maxFlow = 100.0;
    segment.duration = 1.0;
    segment.pressureTolerance = 0.1;
    segment.pressureRampRate = 100.0;
    segment.pressureController = HYD_PRESSURE_CONTROLLER_P;
    segment.pressureKp = 1.0;

    assert(HYD_MotionControlFB_LoadDirectSegment(&fb, &segment));
    assert(HYD_MotionControlFB_StartSegment(&fb, 0, 0.0));

    fb.AXIS_REF.timestamp = 0.1;
    HYD_MotionControlFB_Cycle(&fb);
    nominalSpeed = fb.PUMP_SPEED;

    fb.AXIS_REF.timestamp = 1.0;
    fb.AXIS_REF.pressure = 0.0;
    HYD_MotionControlFB_Cycle(&fb);
    deratedSpeed = fb.PUMP_SPEED;

    assert(nominalSpeed > 0.0);
    assert(deratedSpeed > 0.0);
    assert(deratedSpeed < nominalSpeed);
    assert(fb.STATE.status == HYD_STATUS_DEGRADED);
}
```

Call it from `main()` in that test file:

```c
test_derate_reduces_runtime_pump_speed();
```

- [ ] **Step 9: Run focused tests**

Run:

```bash
cmake --build --preset unixgcc --target test_output_limiter test_sprint_b_integration
./out/build/unixgcc/test_output_limiter
./out/build/unixgcc/test_sprint_b_integration
```

Expected: both executables pass.

- [ ] **Step 10: Commit Task 1**

Run:

```bash
git add include/output_limiter.h src/output_limiter.c tests/test_output_limiter.c tests/test_sprint_b_integration.c src/motion_control.c CMakeLists.txt
git commit -m "feat: apply diagnostic derate to pump outputs"
```

## Task 2: Acceleration-Limited Runtime Planner

**Files:**
- Modify: `include/motion_planner.h`
- Modify: `src/motion_planner.c`
- Modify: `include/motion_control.h`
- Modify: `src/motion_control.c`
- Modify: `tests/test_motion_planner.c`

**Purpose:** Replace position-mode velocity jumps with acceleration-limited runtime target evolution while preserving existing braking protection.

- [ ] **Step 1: Add failing planner continuity tests**

Append to `tests/test_motion_planner.c`:

```c
static void test_time_based_position_planner_limits_acceleration_between_cycles(void) {
    HYD_AxisRef axisRef;
    HYD_MotionSegment segment;
    HYD_MotionPlannerState state;
    HYD_MotionPlannerInput input;
    HYD_MotionPlannerOutput output;

    printf("Testing acceleration-limited position planner continuity...\n");

    memset(&state, 0, sizeof(state));
    axisRef = create_test_axis_ref(0.0);
    segment = create_test_segment();
    segment.planner = HYD_PLANNER_TIME_BASED;
    segment.mode = HYD_MODE_POSITION;
    segment.direction = HYD_DIRECTION_EXTEND;
    segment.targetPosition = 100.0;
    segment.maxVelocity = 100.0;
    segment.maxAcceleration = 10.0;
    segment.maxDeceleration = 10.0;
    segment.maxFlow = 500.0;

    memset(&input, 0, sizeof(input));
    input.axisRef = &axisRef;
    input.segment = &segment;
    input.elapsedTime = 0.0;
    input.deltaTime = 0.0;
    input.state = &state;
    HYD_MotionPlanner_Execute(&input, &output);
    assert(fabs(output.targetVelocity) < 0.001);

    input.elapsedTime = 0.1;
    input.deltaTime = 0.1;
    HYD_MotionPlanner_Execute(&input, &output);
    assert(fabs(output.targetVelocity - 1.0) < 0.001);

    input.elapsedTime = 0.2;
    input.deltaTime = 0.1;
    HYD_MotionPlanner_Execute(&input, &output);
    assert(fabs(output.targetVelocity - 2.0) < 0.001);

    printf("✓ Acceleration-limited position planner continuity test passed\n");
}

static void test_position_planner_decelerates_with_max_deceleration(void) {
    HYD_AxisRef axisRef;
    HYD_MotionSegment segment;
    HYD_MotionPlannerState state;
    HYD_MotionPlannerInput input;
    HYD_MotionPlannerOutput output;

    printf("Testing position planner deceleration continuity...\n");

    memset(&state, 0, sizeof(state));
    state.initialized = true;
    state.lastTargetVelocity = 20.0;
    state.lastTimestamp = 1.0;

    axisRef = create_test_axis_ref(99.0);
    segment = create_test_segment();
    segment.planner = HYD_PLANNER_TIME_BASED;
    segment.mode = HYD_MODE_POSITION;
    segment.direction = HYD_DIRECTION_EXTEND;
    segment.targetPosition = 100.0;
    segment.maxVelocity = 100.0;
    segment.maxAcceleration = 100.0;
    segment.maxDeceleration = 5.0;
    segment.maxFlow = 500.0;

    memset(&input, 0, sizeof(input));
    input.axisRef = &axisRef;
    input.segment = &segment;
    input.elapsedTime = 1.1;
    input.deltaTime = 0.1;
    input.state = &state;

    HYD_MotionPlanner_Execute(&input, &output);

    assert(output.targetVelocity < 20.0);
    assert(output.targetVelocity >= 0.0);
    assert(fabs(output.targetVelocity - 19.5) < 0.001);

    printf("✓ Position planner deceleration continuity test passed\n");
}
```

Add calls in `main()`:

```c
test_time_based_position_planner_limits_acceleration_between_cycles();
test_position_planner_decelerates_with_max_deceleration();
```

- [ ] **Step 2: Run the failing planner test**

Run:

```bash
cmake --build --preset unixgcc --target test_motion_planner
```

Expected: build fails because `HYD_MotionPlannerState`, `deltaTime`, and `state` are not defined.

- [ ] **Step 3: Extend planner API**

Modify `include/motion_planner.h`:

```c
typedef struct {
    HYD_BOOL initialized;
    HYD_REAL lastTargetVelocity;
    HYD_REAL lastTargetFlow;
    HYD_TIME lastTimestamp;
} HYD_MotionPlannerState;
```

Extend `HYD_MotionPlannerInput`:

```c
typedef struct {
    const HYD_AxisRef* axisRef;
    const HYD_MotionSegment* segment;
    HYD_REAL elapsedTime;
    HYD_REAL deltaTime;
    HYD_REAL rampedPressure;
    HYD_REAL decelElapsed;
    HYD_REAL decelStartVel;
    HYD_MotionPlannerState* state;
} HYD_MotionPlannerInput;
```

- [ ] **Step 4: Add planner state to the FB**

Modify `include/motion_control.h` inside `HYD_MotionControlFB` internal fields:

```c
HYD_MotionPlannerState _plannerState;
```

Place it near `_rampController` and `_pressureController`.

- [ ] **Step 5: Reset planner state on segment start and reset**

Modify `src/motion_control.c`:

Inside `HYD_PrimeSegmentControllers`, after deceleration state reset:

```c
memset(&fb->_plannerState, 0, sizeof(fb->_plannerState));
```

Inside `HYD_MotionControlFB_Init`, after ramp controller init:

```c
memset(&fb->_plannerState, 0, sizeof(fb->_plannerState));
```

Inside `HYD_MotionControlFB_SoftReset`, after ramp controller init:

```c
memset(&fb->_plannerState, 0, sizeof(fb->_plannerState));
```

- [ ] **Step 6: Pass delta time and planner state from runtime**

Modify `HYD_ExecuteActiveSegmentControl` in `src/motion_control.c`.

Before calling `HYD_MotionPlanner_Execute`, set:

```c
plannerInput.deltaTime = 0.0;
if (fb->_lastFeedbackTimestamp >= 0.0 &&
    fb->AXIS_REF.timestamp >= fb->_lastFeedbackTimestamp) {
    plannerInput.deltaTime = fb->AXIS_REF.timestamp - fb->_lastFeedbackTimestamp;
}
plannerInput.state = &fb->_plannerState;
```

Keep `elapsedTime`, `axisRef`, `segment`, and deceleration fields as they are.

- [ ] **Step 7: Implement acceleration-limited evolution**

Modify `src/motion_planner.c` by adding helper:

```c
static HYD_REAL HYD_ApplyVelocityRateLimit(HYD_REAL previousVelocity,
                                           HYD_REAL desiredVelocity,
                                           HYD_REAL acceleration,
                                           HYD_REAL deceleration,
                                           HYD_REAL deltaTime) {
    HYD_REAL delta;
    HYD_REAL limit;

    if (deltaTime <= 0.0) {
        return previousVelocity;
    }

    delta = desiredVelocity - previousVelocity;
    if (delta >= 0.0) {
        limit = acceleration * deltaTime;
    } else {
        limit = deceleration * deltaTime;
    }

    if (limit <= 0.0) {
        return desiredVelocity;
    }

    if (delta > limit) {
        return previousVelocity + limit;
    }
    if (delta < -limit) {
        return previousVelocity - limit;
    }
    return desiredVelocity;
}
```

Inside `HYD_MotionPlanner_Execute`, after computing unsigned `velocityMagnitude`, apply stateful limiting for `HYD_PLANNER_TIME_BASED` and all `HYD_MODE_SPEED_RAMP` segments:

```c
if (input->state != NULL &&
    (input->segment->planner == HYD_PLANNER_TIME_BASED ||
     input->segment->mode == HYD_MODE_SPEED_RAMP)) {
    HYD_REAL previousMagnitude = fabs(input->state->lastTargetVelocity);
    HYD_REAL brakingAcceleration = (input->segment->maxDeceleration > 0.0)
        ? input->segment->maxDeceleration
        : input->segment->maxAcceleration;

    if (!input->state->initialized) {
        previousMagnitude = 0.0;
        input->state->initialized = true;
    }

    velocityMagnitude = HYD_ApplyVelocityRateLimit(previousMagnitude,
                                                  velocityMagnitude,
                                                  input->segment->maxAcceleration,
                                                  brakingAcceleration,
                                                  input->deltaTime);
}
```

At the end, after setting output fields:

```c
if (input->state != NULL) {
    input->state->lastTargetVelocity = output->targetVelocity;
    input->state->lastTargetFlow = output->targetFlow;
    input->state->lastTimestamp = input->axisRef->timestamp;
}
```

- [ ] **Step 8: Run planner tests**

Run:

```bash
cmake --build --preset unixgcc --target test_motion_planner
./out/build/unixgcc/test_motion_planner
```

Expected: all motion planner tests pass, including continuity tests.

- [ ] **Step 9: Run runtime integration smoke tests**

Run:

```bash
cmake --build --preset unixgcc --target test_direct_mode test_moveabsolute_stop_integration
./out/build/unixgcc/test_direct_mode
./out/build/unixgcc/test_moveabsolute_stop_integration
```

Expected: direct mode and stop integration still pass.

- [ ] **Step 10: Commit Task 2**

Run:

```bash
git add include/motion_planner.h src/motion_planner.c include/motion_control.h src/motion_control.c tests/test_motion_planner.c
git commit -m "feat: add acceleration-limited motion planning"
```

## Task 3: Stop Deceleration Contract

**Files:**
- Modify: `src/motion_control.c`
- Modify: `tests/test_moveabsolute_stop_integration.c`
- Modify: `docs/architecture/motion-runtime-contract.md`

**Purpose:** Ensure user stop behavior defaults to `maxDeceleration`, not `maxAcceleration`.

- [ ] **Step 1: Add failing stop contract test**

Add to `tests/test_moveabsolute_stop_integration.c`:

```c
static void test_stop_without_deceleration_uses_segment_max_deceleration(void) {
    HYD_MotionControlFB fb;
    HYD_MotionSegment segment;
    HYD_REAL firstStopVelocity;

    HYD_MotionControlFB_Init(&fb);
    fb.USE_RECIPE = false;
    fb.FLOW_TO_PUMP_SPEED_GAIN = 10.0;
    fb.PUMP_SPEED_LIMIT = 3000.0;
    fb.AXIS_REF.position = 0.0;
    fb.AXIS_REF.velocity = 20.0;
    fb.AXIS_REF.flow = 20.0;
    fb.AXIS_REF.pressure = 5.0;
    fb.AXIS_REF.timestamp = 0.0;

    memset(&segment, 0, sizeof(segment));
    segment.planner = HYD_PLANNER_TIME_BASED;
    segment.mode = HYD_MODE_SPEED_RAMP;
    segment.endCondition = HYD_END_MANUAL;
    segment.direction = HYD_DIRECTION_EXTEND;
    segment.maxVelocity = 50.0;
    segment.maxAcceleration = 100.0;
    segment.maxDeceleration = 5.0;
    segment.maxFlow = 100.0;
    segment.velocityToFlowGain = 1.0;

    assert(HYD_MotionControlFB_LoadDirectSegment(&fb, &segment));
    assert(HYD_MotionControlFB_StartSegment(&fb, 0, 0.0));

    fb.AXIS_REF.timestamp = 0.1;
    HYD_MotionControlFB_Cycle(&fb);

    assert(HYD_MotionControlFB_Stop(&fb, 0.1, 0.0));

    fb.AXIS_REF.timestamp = 0.2;
    fb.AXIS_REF.velocity = 19.5;
    HYD_MotionControlFB_Cycle(&fb);
    firstStopVelocity = fabs(fb.STATE.plannedVelocity);

    assert(fabs(firstStopVelocity - 19.5) < 0.01);
}
```

Call it from `main()`:

```c
test_stop_without_deceleration_uses_segment_max_deceleration();
```

- [ ] **Step 2: Run the failing stop test**

Run:

```bash
cmake --build --preset unixgcc --target test_moveabsolute_stop_integration
./out/build/unixgcc/test_moveabsolute_stop_integration
```

Expected: test fails because the runtime uses `segment->maxAcceleration`, producing a much lower planned velocity.

- [ ] **Step 3: Fix default stop deceleration**

Modify `src/motion_control.c` in the `_isStopping` block:

Replace:

```c
HYD_REAL stopDeceleration = (fb->_stopDeceleration > 0.0f) ?
    fb->_stopDeceleration : segment->maxAcceleration;
```

with:

```c
HYD_REAL stopDeceleration = (fb->_stopDeceleration > 0.0f)
    ? fb->_stopDeceleration
    : ((segment->maxDeceleration > 0.0f) ? segment->maxDeceleration : segment->maxAcceleration);
```

- [ ] **Step 4: Update runtime contract documentation**

Modify `docs/architecture/motion-runtime-contract.md` under `Stop` semantics with:

```md
When the caller supplies a positive stop deceleration, the runtime uses that value. When the caller supplies zero or a negative value, the runtime falls back to the active segment's `maxDeceleration`; if that field is zero, it falls back to `maxAcceleration` for legacy recipes.
```

- [ ] **Step 5: Run focused stop tests**

Run:

```bash
cmake --build --preset unixgcc --target test_moveabsolute_stop_integration test_stop_immediate_done
./out/build/unixgcc/test_moveabsolute_stop_integration
./out/build/unixgcc/test_stop_immediate_done
```

Expected: both stop tests pass.

- [ ] **Step 6: Commit Task 3**

Run:

```bash
git add src/motion_control.c tests/test_moveabsolute_stop_integration.c docs/architecture/motion-runtime-contract.md
git commit -m "fix: use segment deceleration for stop fallback"
```

## Task 4: Optional Velocity Closed-Loop Correction

**Files:**
- Create: `include/velocity_controller.h`
- Create: `src/velocity_controller.c`
- Create: `tests/test_velocity_controller.c`
- Modify: `include/common_types.h`
- Modify: `src/motion_control.c`
- Modify: `CMakeLists.txt`

**Purpose:** Give `HYD_MODE_SPEED_RAMP` an optional feedback correction layer so injection fill and other velocity-governed moves are not only open-loop flow feedforward.

- [ ] **Step 1: Add velocity-loop segment fields**

Modify `include/common_types.h` in `HYD_MotionSegment` after `velocityToFlowGain`:

```c
HYD_REAL velocityKp;        /* L/min per mm/s, 0 disables velocity feedback correction */
HYD_REAL velocityDeadband;  /* mm/s, 0 means no deadband */
HYD_REAL velocityCorrectionLimit; /* L/min absolute correction limit, 0 uses maxFlow */
```

Modify `HYD_MotionFBParams` after `velocityToFlowGain`:

```c
HYD_REAL velocityKp;
HYD_REAL velocityDeadband;
HYD_REAL velocityCorrectionLimit;
```

- [ ] **Step 2: Write failing velocity controller tests**

Create `tests/test_velocity_controller.c`:

```c
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "velocity_controller.h"

static void test_velocity_controller_adds_flow_when_too_slow(void) {
    HYD_VelocityControllerInput input = {0};
    HYD_VelocityControllerOutput output = {0};

    input.targetVelocity = 20.0;
    input.actualVelocity = 15.0;
    input.feedforwardFlow = 30.0;
    input.kp = 2.0;
    input.deadband = 0.1;
    input.correctionLimit = 20.0;
    input.outputMin = 0.0;
    input.outputMax = 100.0;

    HYD_VelocityController_Execute(&input, &output);

    assert(fabs(output.correctedFlow - 40.0) < 0.001);
    assert(fabs(output.correctionFlow - 10.0) < 0.001);
    assert(output.active);
}

static void test_velocity_controller_reduces_flow_when_too_fast(void) {
    HYD_VelocityControllerInput input = {0};
    HYD_VelocityControllerOutput output = {0};

    input.targetVelocity = 20.0;
    input.actualVelocity = 25.0;
    input.feedforwardFlow = 30.0;
    input.kp = 2.0;
    input.deadband = 0.1;
    input.correctionLimit = 20.0;
    input.outputMin = 0.0;
    input.outputMax = 100.0;

    HYD_VelocityController_Execute(&input, &output);

    assert(fabs(output.correctedFlow - 20.0) < 0.001);
    assert(fabs(output.correctionFlow + 10.0) < 0.001);
    assert(output.active);
}

static void test_velocity_controller_deadband_disables_correction(void) {
    HYD_VelocityControllerInput input = {0};
    HYD_VelocityControllerOutput output = {0};

    input.targetVelocity = 20.0;
    input.actualVelocity = 20.05;
    input.feedforwardFlow = 30.0;
    input.kp = 2.0;
    input.deadband = 0.1;
    input.correctionLimit = 20.0;
    input.outputMin = 0.0;
    input.outputMax = 100.0;

    HYD_VelocityController_Execute(&input, &output);

    assert(fabs(output.correctedFlow - 30.0) < 0.001);
    assert(fabs(output.correctionFlow) < 0.001);
    assert(!output.active);
}

int main(void) {
    test_velocity_controller_adds_flow_when_too_slow();
    test_velocity_controller_reduces_flow_when_too_fast();
    test_velocity_controller_deadband_disables_correction();
    printf("Velocity controller tests passed\n");
    return 0;
}
```

- [ ] **Step 3: Add test target**

Modify `CMakeLists.txt`:

```cmake
add_executable(test_velocity_controller tests/test_velocity_controller.c)
target_link_libraries(test_velocity_controller PRIVATE HydroMotionLib)
add_test(NAME test_velocity_controller COMMAND test_velocity_controller)
```

- [ ] **Step 4: Run failing velocity controller test**

Run:

```bash
cmake --preset unixgcc
cmake --build --preset unixgcc --target test_velocity_controller
```

Expected: build fails because `velocity_controller.h` does not exist.

- [ ] **Step 5: Create velocity controller header**

Create `include/velocity_controller.h`:

```c
#ifndef HYD_VELOCITY_CONTROLLER_H
#define HYD_VELOCITY_CONTROLLER_H

#include "common_types.h"

typedef struct {
    HYD_REAL targetVelocity;
    HYD_REAL actualVelocity;
    HYD_REAL feedforwardFlow;
    HYD_REAL kp;
    HYD_REAL deadband;
    HYD_REAL correctionLimit;
    HYD_REAL outputMin;
    HYD_REAL outputMax;
} HYD_VelocityControllerInput;

typedef struct {
    HYD_REAL velocityError;
    HYD_REAL correctionFlow;
    HYD_REAL correctedFlow;
    HYD_BOOL active;
    HYD_BOOL saturated;
} HYD_VelocityControllerOutput;

void HYD_VelocityController_Execute(const HYD_VelocityControllerInput* input,
                                    HYD_VelocityControllerOutput* output);

#endif /* HYD_VELOCITY_CONTROLLER_H */
```

- [ ] **Step 6: Implement velocity controller**

Create `src/velocity_controller.c`:

```c
#include "velocity_controller.h"
#include <math.h>

void HYD_VelocityController_Execute(const HYD_VelocityControllerInput* input,
                                    HYD_VelocityControllerOutput* output) {
    HYD_REAL error;
    HYD_REAL correctionLimit;
    HYD_REAL correction;
    HYD_REAL correctedFlow;

    if (output == NULL) {
        return;
    }

    output->velocityError = 0.0;
    output->correctionFlow = 0.0;
    output->correctedFlow = 0.0;
    output->active = false;
    output->saturated = false;

    if (input == NULL) {
        return;
    }

    output->correctedFlow = HYD_ClampReal(input->feedforwardFlow, input->outputMin, input->outputMax);

    if (input->kp <= 0.0) {
        return;
    }

    error = fabs(input->targetVelocity) - fabs(input->actualVelocity);
    output->velocityError = error;

    if (input->deadband > 0.0 && fabs(error) <= input->deadband) {
        return;
    }

    correctionLimit = (input->correctionLimit > 0.0) ? input->correctionLimit : input->outputMax;
    correction = HYD_ClampReal(input->kp * error, -correctionLimit, correctionLimit);
    correctedFlow = HYD_ClampReal(input->feedforwardFlow + correction, input->outputMin, input->outputMax);

    output->correctionFlow = correctedFlow - input->feedforwardFlow;
    output->correctedFlow = correctedFlow;
    output->active = fabs(output->correctionFlow) > 0.0;
    output->saturated = (correction != input->kp * error) ||
                        (correctedFlow != input->feedforwardFlow + correction);
}
```

- [ ] **Step 7: Run velocity controller unit test**

Run:

```bash
cmake --build --preset unixgcc --target test_velocity_controller
./out/build/unixgcc/test_velocity_controller
```

Expected: `Velocity controller tests passed`.

- [ ] **Step 8: Integrate velocity controller into runtime**

Modify `src/motion_control.c`:

Add include:

```c
#include "velocity_controller.h"
```

Inside `HYD_ExecuteActiveSegmentControl`, after `HYD_MotionPlanner_Execute(&plannerInput, plannerOutput);`, insert:

```c
if (segment->mode == HYD_MODE_SPEED_RAMP && segment->velocityKp > 0.0) {
    HYD_VelocityControllerInput velocityInput;
    HYD_VelocityControllerOutput velocityOutput;

    velocityInput.targetVelocity = plannerOutput->targetVelocity;
    velocityInput.actualVelocity = fb->AXIS_REF.velocity;
    velocityInput.feedforwardFlow = plannerOutput->targetFlow;
    velocityInput.kp = segment->velocityKp;
    velocityInput.deadband = segment->velocityDeadband;
    velocityInput.correctionLimit = segment->velocityCorrectionLimit;
    velocityInput.outputMin = 0.0;
    velocityInput.outputMax = segment->maxFlow;

    HYD_VelocityController_Execute(&velocityInput, &velocityOutput);
    plannerOutput->targetFlow = velocityOutput.correctedFlow;
}
```

- [ ] **Step 9: Update parameter accessors**

Modify `HYD_MotionControlFB_ReadParameter` in `src/motion_control.c`:

```c
case HYD_PARAM_VELOCITY_KP: *value = fb->_params.velocityKp; break;
case HYD_PARAM_VELOCITY_DEADBAND: *value = fb->_params.velocityDeadband; break;
case HYD_PARAM_VELOCITY_CORRECTION_LIMIT: *value = fb->_params.velocityCorrectionLimit; break;
```

Modify `HYD_MotionControlFB_WriteParameter`:

```c
case HYD_PARAM_VELOCITY_KP: fb->_params.velocityKp = value; break;
case HYD_PARAM_VELOCITY_DEADBAND: fb->_params.velocityDeadband = value; break;
case HYD_PARAM_VELOCITY_CORRECTION_LIMIT: fb->_params.velocityCorrectionLimit = value; break;
```

Add enum values in `HYD_ParameterNumber` before `HYD_PARAM_COUNT`:

```c
HYD_PARAM_VELOCITY_KP,
HYD_PARAM_VELOCITY_DEADBAND,
HYD_PARAM_VELOCITY_CORRECTION_LIMIT,
```

- [ ] **Step 10: Run parameter and velocity tests**

Run:

```bash
cmake --build --preset unixgcc --target test_velocity_controller test_parameter_access test_motion_planner
./out/build/unixgcc/test_velocity_controller
./out/build/unixgcc/test_parameter_access
./out/build/unixgcc/test_motion_planner
```

Expected: all pass.

- [ ] **Step 11: Commit Task 4**

Run:

```bash
git add include/common_types.h include/velocity_controller.h src/velocity_controller.c src/motion_control.c tests/test_velocity_controller.c CMakeLists.txt
git commit -m "feat: add optional velocity feedback correction"
```

## Task 5: Injection-Machine Action Profiles

**Files:**
- Create: `include/action_profile.h`
- Create: `src/action_profile.c`
- Create: `tests/test_action_profile.c`
- Modify: `src/recipe_validator.c`
- Modify: `docs/architecture/motion-profile-archetypes.md`
- Modify: `CMakeLists.txt`

**Purpose:** Provide standard segment defaults for core injection-machine actions without embedding PLC workflow or valve sequencing.

- [ ] **Step 1: Define action profile API and tests first**

Create `tests/test_action_profile.c`:

```c
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "action_profile.h"
#include "recipe_validator.h"

static HYD_MotionFBParams default_params(void) {
    HYD_MotionFBParams params;
    memset(&params, 0, sizeof(params));
    params.positionTolerance = 0.1;
    params.velocityTolerance = 0.5;
    params.flowTolerance = 1.0;
    params.pressureTolerance = 0.5;
    params.timeoutLimit = 10.0;
    params.velocityToFlowGain = 1.0;
    params.maxVelocity = 100.0;
    params.maxAcceleration = 50.0;
    params.maxDeceleration = 40.0;
    params.maxFlow = 120.0;
    params.pressureRampRate = 20.0;
    params.pressureKp = 1.0;
    params.pressureKi = 0.2;
    params.pressureControllerType = HYD_PRESSURE_CONTROLLER_PI;
    return params;
}

static void test_clamp_close_profile_is_position_extend(void) {
    HYD_MotionSegment segment;
    HYD_DiagnosticCode code;
    HYD_MotionFBParams params = default_params();

    assert(HYD_ActionProfile_BuildClampClose(&segment, &params, 1, 250.0));

    assert(segment.segmentType == HYD_SEGMENT_TYPE_CLAMPING);
    assert(segment.mode == HYD_MODE_POSITION);
    assert(segment.endCondition == HYD_END_POSITION);
    assert(segment.direction == HYD_DIRECTION_EXTEND);
    assert(segment.targetPosition == 250.0);
    assert(HYD_RecipeValidator_ValidateSegment(&segment, 0, &code));
}

static void test_injection_fill_profile_is_speed_ramp_extend(void) {
    HYD_MotionSegment segment;
    HYD_DiagnosticCode code;
    HYD_MotionFBParams params = default_params();

    assert(HYD_ActionProfile_BuildInjectionFill(&segment, &params, 2, 100.0));

    assert(segment.segmentType == HYD_SEGMENT_TYPE_INJECTION);
    assert(segment.mode == HYD_MODE_SPEED_RAMP);
    assert(segment.planner == HYD_PLANNER_TIME_BASED);
    assert(segment.direction == HYD_DIRECTION_EXTEND);
    assert(segment.targetPosition == 100.0);
    assert(HYD_RecipeValidator_ValidateSegment(&segment, 0, &code));
}

static void test_holding_profile_is_pressure_closed_loop(void) {
    HYD_MotionSegment segment;
    HYD_DiagnosticCode code;
    HYD_MotionFBParams params = default_params();

    assert(HYD_ActionProfile_BuildHoldingPressure(&segment, &params, 3, 80.0, 2.0));

    assert(segment.segmentType == HYD_SEGMENT_TYPE_HOLDING);
    assert(segment.mode == HYD_MODE_PRESSURE_CLOSED_LOOP);
    assert(segment.endCondition == HYD_END_TIME);
    assert(segment.direction == HYD_DIRECTION_HOLD);
    assert(segment.targetPressure == 80.0);
    assert(segment.duration == 2.0);
    assert(HYD_RecipeValidator_ValidateSegment(&segment, 0, &code));
}

int main(void) {
    test_clamp_close_profile_is_position_extend();
    test_injection_fill_profile_is_speed_ramp_extend();
    test_holding_profile_is_pressure_closed_loop();
    printf("Action profile tests passed\n");
    return 0;
}
```

- [ ] **Step 2: Add test target**

Modify `CMakeLists.txt`:

```cmake
add_executable(test_action_profile tests/test_action_profile.c)
target_link_libraries(test_action_profile PRIVATE HydroMotionLib)
add_test(NAME test_action_profile COMMAND test_action_profile)
```

- [ ] **Step 3: Run failing action profile test**

Run:

```bash
cmake --preset unixgcc
cmake --build --preset unixgcc --target test_action_profile
```

Expected: build fails because `action_profile.h` does not exist.

- [ ] **Step 4: Create action profile header**

Create `include/action_profile.h`:

```c
#ifndef HYD_ACTION_PROFILE_H
#define HYD_ACTION_PROFILE_H

#include "common_types.h"

HYD_BOOL HYD_ActionProfile_BuildClampClose(HYD_MotionSegment* segment,
                                           const HYD_MotionFBParams* params,
                                           HYD_UINT8 segmentTag,
                                           HYD_REAL targetPosition);

HYD_BOOL HYD_ActionProfile_BuildClampOpen(HYD_MotionSegment* segment,
                                          const HYD_MotionFBParams* params,
                                          HYD_UINT8 segmentTag,
                                          HYD_REAL targetPosition);

HYD_BOOL HYD_ActionProfile_BuildInjectionFill(HYD_MotionSegment* segment,
                                              const HYD_MotionFBParams* params,
                                              HYD_UINT8 segmentTag,
                                              HYD_REAL transferPosition);

HYD_BOOL HYD_ActionProfile_BuildHoldingPressure(HYD_MotionSegment* segment,
                                                const HYD_MotionFBParams* params,
                                                HYD_UINT8 segmentTag,
                                                HYD_REAL targetPressure,
                                                HYD_TIME duration);

HYD_BOOL HYD_ActionProfile_BuildEjectAdvance(HYD_MotionSegment* segment,
                                             const HYD_MotionFBParams* params,
                                             HYD_UINT8 segmentTag,
                                             HYD_REAL targetPosition);

HYD_BOOL HYD_ActionProfile_BuildEjectRetract(HYD_MotionSegment* segment,
                                             const HYD_MotionFBParams* params,
                                             HYD_UINT8 segmentTag,
                                             HYD_REAL targetPosition);

HYD_BOOL HYD_ActionProfile_BuildCarriageMove(HYD_MotionSegment* segment,
                                             const HYD_MotionFBParams* params,
                                             HYD_UINT8 segmentTag,
                                             HYD_REAL targetPosition,
                                             HYD_MotionDirection direction);

#endif /* HYD_ACTION_PROFILE_H */
```

- [ ] **Step 5: Implement action profile defaults**

Create `src/action_profile.c`:

```c
#include "action_profile.h"
#include <string.h>

static HYD_BOOL HYD_ActionProfile_HasUsableParams(const HYD_MotionFBParams* params) {
    return params != NULL &&
           params->maxFlow > 0.0 &&
           params->maxVelocity > 0.0 &&
           params->maxAcceleration > 0.0 &&
           params->velocityToFlowGain > 0.0;
}

static void HYD_ActionProfile_ApplyCommon(HYD_MotionSegment* segment,
                                          const HYD_MotionFBParams* params,
                                          HYD_UINT8 segmentTag,
                                          HYD_SegmentType segmentType) {
    memset(segment, 0, sizeof(*segment));
    segment->segmentTag = segmentTag;
    segment->segmentType = segmentType;
    segment->targetFlow = params->defaultTargetFlow;
    segment->maxFlow = params->maxFlow;
    segment->maxVelocity = params->maxVelocity;
    segment->maxAcceleration = params->maxAcceleration;
    segment->maxDeceleration = params->maxDeceleration;
    segment->positionTolerance = params->positionTolerance;
    segment->velocityTolerance = params->velocityTolerance;
    segment->flowTolerance = params->flowTolerance;
    segment->pressureTolerance = params->pressureTolerance;
    segment->timeoutLimit = params->timeoutLimit;
    segment->velocityToFlowGain = params->velocityToFlowGain;
    segment->pressureRampRate = params->pressureRampRate;
    segment->pressureController = (HYD_PressureControllerType)((int)params->pressureControllerType);
    segment->pressureKp = params->pressureKp;
    segment->pressureKpHigh = params->pressureKpHigh;
    segment->pressureGainBand = params->pressureGainBand;
    segment->pressureKi = params->pressureKi;
    segment->pressureKd = params->pressureKd;
    segment->pressureIntegralLimit = params->pressureIntegralLimit;
    segment->pressureDeadband = params->pressureDeadband;
    segment->pressureFilterAlpha = params->pressureFilterAlpha;
    segment->pressureDerivativeFilterAlpha = params->pressureDerivativeFilterAlpha;
    segment->velocityKp = params->velocityKp;
    segment->velocityDeadband = params->velocityDeadband;
    segment->velocityCorrectionLimit = params->velocityCorrectionLimit;
}

static HYD_BOOL HYD_ActionProfile_BuildPosition(HYD_MotionSegment* segment,
                                                const HYD_MotionFBParams* params,
                                                HYD_UINT8 segmentTag,
                                                HYD_SegmentType segmentType,
                                                HYD_MotionDirection direction,
                                                HYD_REAL targetPosition) {
    if (segment == NULL || !HYD_ActionProfile_HasUsableParams(params)) {
        return false;
    }

    HYD_ActionProfile_ApplyCommon(segment, params, segmentTag, segmentType);
    segment->planner = HYD_PLANNER_TIME_BASED;
    segment->mode = HYD_MODE_POSITION;
    segment->endCondition = HYD_END_POSITION;
    segment->direction = direction;
    segment->targetPosition = targetPosition;
    return true;
}

HYD_BOOL HYD_ActionProfile_BuildClampClose(HYD_MotionSegment* segment,
                                           const HYD_MotionFBParams* params,
                                           HYD_UINT8 segmentTag,
                                           HYD_REAL targetPosition) {
    return HYD_ActionProfile_BuildPosition(segment, params, segmentTag,
                                           HYD_SEGMENT_TYPE_CLAMPING,
                                           HYD_DIRECTION_EXTEND,
                                           targetPosition);
}

HYD_BOOL HYD_ActionProfile_BuildClampOpen(HYD_MotionSegment* segment,
                                          const HYD_MotionFBParams* params,
                                          HYD_UINT8 segmentTag,
                                          HYD_REAL targetPosition) {
    return HYD_ActionProfile_BuildPosition(segment, params, segmentTag,
                                           HYD_SEGMENT_TYPE_OPENING,
                                           HYD_DIRECTION_RETRACT,
                                           targetPosition);
}

HYD_BOOL HYD_ActionProfile_BuildInjectionFill(HYD_MotionSegment* segment,
                                              const HYD_MotionFBParams* params,
                                              HYD_UINT8 segmentTag,
                                              HYD_REAL transferPosition) {
    if (segment == NULL || !HYD_ActionProfile_HasUsableParams(params)) {
        return false;
    }

    HYD_ActionProfile_ApplyCommon(segment, params, segmentTag, HYD_SEGMENT_TYPE_INJECTION);
    segment->planner = HYD_PLANNER_TIME_BASED;
    segment->mode = HYD_MODE_SPEED_RAMP;
    segment->endCondition = HYD_END_POSITION;
    segment->direction = HYD_DIRECTION_EXTEND;
    segment->targetPosition = transferPosition;
    return true;
}

HYD_BOOL HYD_ActionProfile_BuildHoldingPressure(HYD_MotionSegment* segment,
                                                const HYD_MotionFBParams* params,
                                                HYD_UINT8 segmentTag,
                                                HYD_REAL targetPressure,
                                                HYD_TIME duration) {
    if (segment == NULL || params == NULL || params->maxFlow <= 0.0 ||
        targetPressure <= 0.0 || duration <= 0.0) {
        return false;
    }

    HYD_ActionProfile_ApplyCommon(segment, params, segmentTag, HYD_SEGMENT_TYPE_HOLDING);
    segment->planner = HYD_PLANNER_TIME_BASED;
    segment->mode = HYD_MODE_PRESSURE_CLOSED_LOOP;
    segment->endCondition = HYD_END_TIME;
    segment->direction = HYD_DIRECTION_HOLD;
    segment->targetPressure = targetPressure;
    segment->duration = duration;
    return true;
}

HYD_BOOL HYD_ActionProfile_BuildEjectAdvance(HYD_MotionSegment* segment,
                                             const HYD_MotionFBParams* params,
                                             HYD_UINT8 segmentTag,
                                             HYD_REAL targetPosition) {
    return HYD_ActionProfile_BuildPosition(segment, params, segmentTag,
                                           HYD_SEGMENT_TYPE_EJECTION,
                                           HYD_DIRECTION_EXTEND,
                                           targetPosition);
}

HYD_BOOL HYD_ActionProfile_BuildEjectRetract(HYD_MotionSegment* segment,
                                             const HYD_MotionFBParams* params,
                                             HYD_UINT8 segmentTag,
                                             HYD_REAL targetPosition) {
    return HYD_ActionProfile_BuildPosition(segment, params, segmentTag,
                                           HYD_SEGMENT_TYPE_EJECTION,
                                           HYD_DIRECTION_RETRACT,
                                           targetPosition);
}

HYD_BOOL HYD_ActionProfile_BuildCarriageMove(HYD_MotionSegment* segment,
                                             const HYD_MotionFBParams* params,
                                             HYD_UINT8 segmentTag,
                                             HYD_REAL targetPosition,
                                             HYD_MotionDirection direction) {
    if (direction != HYD_DIRECTION_EXTEND && direction != HYD_DIRECTION_RETRACT) {
        return false;
    }
    return HYD_ActionProfile_BuildPosition(segment, params, segmentTag,
                                           HYD_SEGMENT_TYPE_OTHER,
                                           direction,
                                           targetPosition);
}
```

- [ ] **Step 6: Add action-sensitive validation**

Modify `src/recipe_validator.c` inside `HYD_RecipeValidator_ValidateSegment` after mode/direction checks:

```c
if (segment->segmentType == HYD_SEGMENT_TYPE_INJECTION &&
    segment->mode == HYD_MODE_POSITION) {
    return HYD_RecipeValidator_Fail(code, HYD_DIAG_CODE_SEGMENT_INVALID);
}

if (segment->segmentType == HYD_SEGMENT_TYPE_HOLDING &&
    segment->mode != HYD_MODE_PRESSURE_CLOSED_LOOP) {
    return HYD_RecipeValidator_Fail(code, HYD_DIAG_CODE_SEGMENT_INVALID);
}

if ((segment->segmentType == HYD_SEGMENT_TYPE_CLAMPING ||
     segment->segmentType == HYD_SEGMENT_TYPE_OPENING ||
     segment->segmentType == HYD_SEGMENT_TYPE_EJECTION) &&
    segment->mode == HYD_MODE_PRESSURE_CLOSED_LOOP) {
    return HYD_RecipeValidator_Fail(code, HYD_DIAG_CODE_SEGMENT_INVALID);
}
```

- [ ] **Step 7: Run action profile tests**

Run:

```bash
cmake --build --preset unixgcc --target test_action_profile test_recipe_validator
./out/build/unixgcc/test_action_profile
./out/build/unixgcc/test_recipe_validator
```

Expected: both pass.

- [ ] **Step 8: Update motion profile archetype documentation**

Modify `docs/architecture/motion-profile-archetypes.md` by adding a section:

```md
## Runtime Action Profile Helpers

The library provides helper builders for common injection-machine control profiles:

- `HYD_ActionProfile_BuildClampClose`
- `HYD_ActionProfile_BuildClampOpen`
- `HYD_ActionProfile_BuildInjectionFill`
- `HYD_ActionProfile_BuildHoldingPressure`
- `HYD_ActionProfile_BuildEjectAdvance`
- `HYD_ActionProfile_BuildEjectRetract`
- `HYD_ActionProfile_BuildCarriageMove`

These helpers populate `HYD_MotionSegment` defaults only. They do not start motion, switch valves, decide V/P transfer, or own machine sequencing. PLC process logic may still override generated segment values before loading the recipe.
```

- [ ] **Step 9: Commit Task 5**

Run:

```bash
git add include/action_profile.h src/action_profile.c tests/test_action_profile.c src/recipe_validator.c docs/architecture/motion-profile-archetypes.md CMakeLists.txt
git commit -m "feat: add injection-machine action profile builders"
```

## Task 6: Stable Completion Windows

**Files:**
- Modify: `include/common_types.h`
- Modify: `include/segment_completion.h`
- Modify: `src/segment_completion.c`
- Modify: `src/motion_control.c`
- Modify: `tests/segment_completion_test.c`

**Purpose:** Prevent position/pressure/flow completion from latching on a single transient sample.

- [ ] **Step 1: Add stable completion fields**

Modify `HYD_MotionSegment` in `include/common_types.h` after `timeoutLimit`:

```c
HYD_TIME stableWindow;          /* s, 0 means immediate completion */
HYD_REAL stableVelocityLimit;   /* mm/s, 0 disables velocity-settled gate */
```

Add internal state to `HYD_MotionControlFB` in `include/motion_control.h`:

```c
HYD_TIME _completionCandidateStartTime;
HYD_BOOL _completionCandidateActive;
```

- [ ] **Step 2: Extend completion context**

Modify `include/segment_completion.h`:

```c
typedef struct {
    const HYD_MotionSegment* segment;
    const HYD_AxisRef* axisRef;
    const HYD_ExecutionReference* references;
    HYD_TIME timestamp;
    HYD_TIME* candidateStartTime;
    HYD_BOOL* candidateActive;
} HYD_SegmentCompletionContext;
```

- [ ] **Step 3: Add failing stable completion tests**

Add to `tests/segment_completion_test.c`:

```c
static void test_position_completion_requires_stable_window(void) {
    HYD_MotionSegment segment;
    HYD_AxisRef axisRef;
    HYD_ExecutionReference references;
    HYD_SegmentCompletionContext context;
    HYD_TIME candidateStart = 0.0;
    HYD_BOOL candidateActive = false;

    memset(&segment, 0, sizeof(segment));
    segment.mode = HYD_MODE_POSITION;
    segment.endCondition = HYD_END_POSITION;
    segment.direction = HYD_DIRECTION_EXTEND;
    segment.targetPosition = 10.0;
    segment.positionTolerance = 0.1;
    segment.stableWindow = 0.2;
    segment.stableVelocityLimit = 0.5;

    memset(&axisRef, 0, sizeof(axisRef));
    axisRef.position = 9.95;
    axisRef.velocity = 0.1;
    axisRef.timestamp = 1.0;

    memset(&references, 0, sizeof(references));
    references.elapsedTime = 1.0;

    context.segment = &segment;
    context.axisRef = &axisRef;
    context.references = &references;
    context.timestamp = 1.0;
    context.candidateStartTime = &candidateStart;
    context.candidateActive = &candidateActive;

    assert(!HYD_SegmentCompletion_CheckWithContext(&context));
    assert(candidateActive);

    axisRef.timestamp = 1.1;
    context.timestamp = 1.1;
    assert(!HYD_SegmentCompletion_CheckWithContext(&context));

    axisRef.timestamp = 1.25;
    context.timestamp = 1.25;
    assert(HYD_SegmentCompletion_CheckWithContext(&context));
}

static void test_position_completion_resets_when_velocity_not_settled(void) {
    HYD_MotionSegment segment;
    HYD_AxisRef axisRef;
    HYD_SegmentCompletionContext context;
    HYD_TIME candidateStart = 0.0;
    HYD_BOOL candidateActive = false;

    memset(&segment, 0, sizeof(segment));
    segment.mode = HYD_MODE_POSITION;
    segment.endCondition = HYD_END_POSITION;
    segment.direction = HYD_DIRECTION_EXTEND;
    segment.targetPosition = 10.0;
    segment.positionTolerance = 0.1;
    segment.stableWindow = 0.2;
    segment.stableVelocityLimit = 0.5;

    memset(&axisRef, 0, sizeof(axisRef));
    axisRef.position = 9.95;
    axisRef.velocity = 2.0;
    axisRef.timestamp = 1.0;

    context.segment = &segment;
    context.axisRef = &axisRef;
    context.references = NULL;
    context.timestamp = 1.0;
    context.candidateStartTime = &candidateStart;
    context.candidateActive = &candidateActive;

    assert(!HYD_SegmentCompletion_CheckWithContext(&context));
    assert(!candidateActive);
}
```

Call both from `main()`.

- [ ] **Step 4: Run failing completion tests**

Run:

```bash
cmake --build --preset unixgcc --target segment_completion_test
./out/build/unixgcc/segment_completion_test
```

Expected: build fails or tests fail because stable completion is not implemented.

- [ ] **Step 5: Implement stable completion in `segment_completion.c`**

In `src/segment_completion.c`, compute the raw end condition as the current switch result does today, then apply:

```c
static HYD_BOOL HYD_SegmentCompletion_ApplyStableWindow(
    const HYD_SegmentCompletionContext* context,
    HYD_BOOL rawComplete) {
    HYD_TIME elapsedStable;

    if (context == NULL || context->segment == NULL) {
        return false;
    }

    if (!rawComplete) {
        if (context->candidateActive != NULL) {
            *context->candidateActive = false;
        }
        return false;
    }

    if (context->segment->stableVelocityLimit > 0.0 &&
        context->axisRef != NULL &&
        fabs(context->axisRef->velocity) > context->segment->stableVelocityLimit) {
        if (context->candidateActive != NULL) {
            *context->candidateActive = false;
        }
        return false;
    }

    if (context->segment->stableWindow <= 0.0 ||
        context->candidateStartTime == NULL ||
        context->candidateActive == NULL) {
        return true;
    }

    if (!*context->candidateActive) {
        *context->candidateActive = true;
        *context->candidateStartTime = context->timestamp;
        return false;
    }

    elapsedStable = context->timestamp - *context->candidateStartTime;
    return elapsedStable >= context->segment->stableWindow;
}
```

Return this helper result instead of raw completion from `HYD_SegmentCompletion_CheckWithContext`.

- [ ] **Step 6: Wire stable state into runtime**

Modify `src/motion_control.c`:

Reset completion candidate in `HYD_BeginSegment`:

```c
fb->_completionCandidateStartTime = 0.0;
fb->_completionCandidateActive = false;
```

Set fields before calling completion:

```c
completionContext.timestamp = fb->AXIS_REF.timestamp;
completionContext.candidateStartTime = &fb->_completionCandidateStartTime;
completionContext.candidateActive = &fb->_completionCandidateActive;
```

- [ ] **Step 7: Run completion and runtime tests**

Run:

```bash
cmake --build --preset unixgcc --target segment_completion_test test_direct_mode test_motion_interface_done_signals
./out/build/unixgcc/segment_completion_test
./out/build/unixgcc/test_direct_mode
./out/build/unixgcc/test_motion_interface_done_signals
```

Expected: all pass.

- [ ] **Step 8: Commit Task 6**

Run:

```bash
git add include/common_types.h include/segment_completion.h src/segment_completion.c src/motion_control.c tests/segment_completion_test.c
git commit -m "feat: add stable segment completion windows"
```

## Task 7: V/P Transfer Observation

**Files:**
- Create: `include/vp_transfer.h`
- Create: `src/vp_transfer.c`
- Create: `tests/test_vp_transfer.c`
- Modify: `include/common_types.h`
- Modify: `src/motion_control.c`
- Modify: `docs/architecture/motion-runtime-contract.md`
- Modify: `docs/integration/plc-process-layer-integration-guide.md`
- Modify: `CMakeLists.txt`

**Purpose:** Provide standard transfer-ready observation for injection fill while preserving the rule that PLC process logic decides and commands the transition to holding pressure.

- [ ] **Step 1: Add V/P fields to runtime state and segment**

Modify `HYD_MotionSegment` in `include/common_types.h` after stable completion fields:

```c
HYD_REAL vpTransferPosition;        /* mm, 0 disables position transfer observation */
HYD_REAL vpTransferPressure;        /* MPa, 0 disables pressure transfer observation */
HYD_TIME vpTransferMinTime;         /* s, 0 disables minimum-time gate */
HYD_REAL vpTransferVelocityDrop;    /* mm/s, 0 disables velocity-drop observation */
```

Modify `HYD_MotionState` in `include/common_types.h` after `currentSegmentTag`:

```c
HYD_BOOL vpTransferReady;
HYD_UINT8 vpTransferReason;
```

Define reason constants near shared enums:

```c
typedef enum {
    HYD_VP_TRANSFER_REASON_NONE = 0,
    HYD_VP_TRANSFER_REASON_POSITION = 1,
    HYD_VP_TRANSFER_REASON_PRESSURE = 2,
    HYD_VP_TRANSFER_REASON_TIME = 3,
    HYD_VP_TRANSFER_REASON_VELOCITY_DROP = 4
} HYD_VpTransferReason;
```

- [ ] **Step 2: Write failing V/P transfer tests**

Create `tests/test_vp_transfer.c`:

```c
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "vp_transfer.h"

static HYD_MotionSegment base_injection_segment(void) {
    HYD_MotionSegment segment;
    memset(&segment, 0, sizeof(segment));
    segment.segmentType = HYD_SEGMENT_TYPE_INJECTION;
    segment.mode = HYD_MODE_SPEED_RAMP;
    segment.direction = HYD_DIRECTION_EXTEND;
    segment.vpTransferPosition = 100.0;
    segment.vpTransferPressure = 80.0;
    segment.vpTransferMinTime = 0.5;
    segment.vpTransferVelocityDrop = 5.0;
    return segment;
}

static void test_vp_transfer_by_position(void) {
    HYD_MotionSegment segment = base_injection_segment();
    HYD_AxisRef axisRef = {0};
    HYD_ExecutionReference references = {0};
    HYD_VpTransferResult result;

    axisRef.position = 100.0;
    axisRef.pressure = 40.0;
    axisRef.velocity = 30.0;
    references.elapsedTime = 0.2;
    references.velocityReference = 30.0;

    HYD_VpTransfer_Evaluate(&segment, &axisRef, &references, &result);

    assert(result.ready);
    assert(result.reason == HYD_VP_TRANSFER_REASON_POSITION);
}

static void test_vp_transfer_by_pressure(void) {
    HYD_MotionSegment segment = base_injection_segment();
    HYD_AxisRef axisRef = {0};
    HYD_ExecutionReference references = {0};
    HYD_VpTransferResult result;

    axisRef.position = 20.0;
    axisRef.pressure = 85.0;
    axisRef.velocity = 30.0;
    references.elapsedTime = 0.2;
    references.velocityReference = 30.0;

    HYD_VpTransfer_Evaluate(&segment, &axisRef, &references, &result);

    assert(result.ready);
    assert(result.reason == HYD_VP_TRANSFER_REASON_PRESSURE);
}

static void test_non_injection_segment_never_reports_transfer(void) {
    HYD_MotionSegment segment = base_injection_segment();
    HYD_AxisRef axisRef = {0};
    HYD_ExecutionReference references = {0};
    HYD_VpTransferResult result;

    segment.segmentType = HYD_SEGMENT_TYPE_CLAMPING;
    axisRef.position = 100.0;
    axisRef.pressure = 85.0;
    references.elapsedTime = 1.0;

    HYD_VpTransfer_Evaluate(&segment, &axisRef, &references, &result);

    assert(!result.ready);
    assert(result.reason == HYD_VP_TRANSFER_REASON_NONE);
}

int main(void) {
    test_vp_transfer_by_position();
    test_vp_transfer_by_pressure();
    test_non_injection_segment_never_reports_transfer();
    printf("V/P transfer tests passed\n");
    return 0;
}
```

- [ ] **Step 3: Add test target**

Modify `CMakeLists.txt`:

```cmake
add_executable(test_vp_transfer tests/test_vp_transfer.c)
target_link_libraries(test_vp_transfer PRIVATE HydroMotionLib)
add_test(NAME test_vp_transfer COMMAND test_vp_transfer)
```

- [ ] **Step 4: Run failing V/P transfer test**

Run:

```bash
cmake --preset unixgcc
cmake --build --preset unixgcc --target test_vp_transfer
```

Expected: build fails because `vp_transfer.h` does not exist.

- [ ] **Step 5: Create V/P transfer header**

Create `include/vp_transfer.h`:

```c
#ifndef HYD_VP_TRANSFER_H
#define HYD_VP_TRANSFER_H

#include "common_types.h"

typedef struct {
    HYD_BOOL ready;
    HYD_VpTransferReason reason;
} HYD_VpTransferResult;

void HYD_VpTransfer_Evaluate(const HYD_MotionSegment* segment,
                             const HYD_AxisRef* axisRef,
                             const HYD_ExecutionReference* references,
                             HYD_VpTransferResult* result);

#endif /* HYD_VP_TRANSFER_H */
```

- [ ] **Step 6: Implement V/P transfer observation**

Create `src/vp_transfer.c`:

```c
#include "vp_transfer.h"
#include <math.h>

static void HYD_VpTransfer_Clear(HYD_VpTransferResult* result) {
    if (result == NULL) {
        return;
    }
    result->ready = false;
    result->reason = HYD_VP_TRANSFER_REASON_NONE;
}

void HYD_VpTransfer_Evaluate(const HYD_MotionSegment* segment,
                             const HYD_AxisRef* axisRef,
                             const HYD_ExecutionReference* references,
                             HYD_VpTransferResult* result) {
    HYD_REAL velocityReference;

    HYD_VpTransfer_Clear(result);

    if (segment == NULL || axisRef == NULL || result == NULL) {
        return;
    }

    if (segment->segmentType != HYD_SEGMENT_TYPE_INJECTION ||
        segment->mode != HYD_MODE_SPEED_RAMP) {
        return;
    }

    if (segment->vpTransferPosition > 0.0 &&
        axisRef->position >= segment->vpTransferPosition) {
        result->ready = true;
        result->reason = HYD_VP_TRANSFER_REASON_POSITION;
        return;
    }

    if (segment->vpTransferPressure > 0.0 &&
        axisRef->pressure >= segment->vpTransferPressure) {
        result->ready = true;
        result->reason = HYD_VP_TRANSFER_REASON_PRESSURE;
        return;
    }

    if (segment->vpTransferMinTime > 0.0 &&
        references != NULL &&
        references->elapsedTime >= segment->vpTransferMinTime) {
        result->ready = true;
        result->reason = HYD_VP_TRANSFER_REASON_TIME;
        return;
    }

    velocityReference = (references != NULL) ? fabs(references->velocityReference) : 0.0;
    if (segment->vpTransferVelocityDrop > 0.0 &&
        velocityReference > 0.0 &&
        velocityReference - fabs(axisRef->velocity) >= segment->vpTransferVelocityDrop) {
        result->ready = true;
        result->reason = HYD_VP_TRANSFER_REASON_VELOCITY_DROP;
    }
}
```

- [ ] **Step 7: Integrate V/P transfer observation into runtime**

Modify `src/motion_control.c`:

Add include:

```c
#include "vp_transfer.h"
```

After `HYD_UpdateExecutionDiagnostics(fb, segment, &executionReference, elapsed);`, insert:

```c
{
    HYD_VpTransferResult vpResult;
    HYD_VpTransfer_Evaluate(segment, &fb->AXIS_REF, &executionReference, &vpResult);
    fb->STATE.vpTransferReady = vpResult.ready;
    fb->STATE.vpTransferReason = (HYD_UINT8)vpResult.reason;
}
```

In safe/idle/fault reset paths, clear:

```c
fb->STATE.vpTransferReady = false;
fb->STATE.vpTransferReason = (HYD_UINT8)HYD_VP_TRANSFER_REASON_NONE;
```

Add the clear in `HYD_StateReporter_ApplySafeOutputs` so all non-executing paths clear stale transfer flags.

- [ ] **Step 8: Run V/P transfer tests**

Run:

```bash
cmake --build --preset unixgcc --target test_vp_transfer test_sprint_b_integration
./out/build/unixgcc/test_vp_transfer
./out/build/unixgcc/test_sprint_b_integration
```

Expected: both pass.

- [ ] **Step 9: Document V/P transfer boundary**

Modify `docs/architecture/motion-runtime-contract.md`:

```md
### V/P Transfer Observation

The runtime may report `STATE.vpTransferReady=true` and `STATE.vpTransferReason` for injection fill segments. This is an observation signal only. The PLC process layer remains responsible for deciding whether to stop the fill segment, command the holding-pressure segment, switch valves, and enforce machine-specific interlocks.
```

Modify `docs/integration/plc-process-layer-integration-guide.md`:

```md
When `STATE.vpTransferReady` is true during an injection fill segment, PLC logic should treat it as a transfer recommendation. The PLC must still validate machine interlocks and explicitly command the transition into the holding-pressure phase.
```

- [ ] **Step 10: Commit Task 7**

Run:

```bash
git add include/common_types.h include/vp_transfer.h src/vp_transfer.c src/motion_control.c src/state_reporter.c tests/test_vp_transfer.c docs/architecture/motion-runtime-contract.md docs/integration/plc-process-layer-integration-guide.md CMakeLists.txt
git commit -m "feat: report injection vp transfer readiness"
```

## Task 8: Documentation And Gap List Closure

**Files:**
- Modify: `docs/architecture/implementation-contract-gap-list.md`
- Modify: `docs/architecture/motion-runtime-contract.md`
- Modify: `docs/architecture/motion-profile-archetypes.md`
- Modify: `docs/integration/plc-process-layer-integration-guide.md`

**Purpose:** Keep architecture documents aligned with the new runtime behavior and keep unsupported machine-process responsibilities explicit.

- [ ] **Step 1: Update implementation gap list**

Modify `docs/architecture/implementation-contract-gap-list.md` by moving these items to an implemented section:

```md
## Implemented Algorithm Gaps

- Diagnostic derate now reduces command flow and pump speed before execution reporting.
- Position and speed-ramp planning now support acceleration-limited target evolution.
- Stop fallback deceleration now uses `maxDeceleration` before legacy `maxAcceleration`.
- Speed-ramp segments may opt into velocity feedback correction through `velocityKp`.
- Action profile helpers provide standard segment defaults for clamp, injection, holding, ejector, and carriage roles.
- Segment completion may require a stable window and velocity-settled condition.
- Injection fill segments may report V/P transfer readiness as an observation signal.
```

- [ ] **Step 2: Add unsupported semantics that remain outside the runtime**

Add:

```md
## Still Outside Runtime Scope

- Valve sequencing remains in PLC process logic.
- Machine interlocks remain in PLC process logic.
- The runtime reports V/P transfer readiness but does not automatically switch to holding pressure.
- Multi-stage injection recipe scheduling remains a PLC or recipe composition responsibility.
- Clamp force build-up and mold-protection workflow remain machine-process responsibilities unless a future approved design moves a bounded part into the control layer.
```

- [ ] **Step 3: Run full test suite**

Run:

```bash
cmake --preset unixgcc
cmake --build --preset unixgcc
ctest --test-dir out/build/unixgcc --output-on-failure
```

Expected: all tests pass.

- [ ] **Step 4: Run production build script**

Run:

```bash
./scripts/deploy_embedded_prod.sh
```

Expected: embedded production build succeeds without simulator-only dependencies leaking into the core runtime.

- [ ] **Step 5: Commit Task 8**

Run:

```bash
git add docs/architecture/implementation-contract-gap-list.md docs/architecture/motion-runtime-contract.md docs/architecture/motion-profile-archetypes.md docs/integration/plc-process-layer-integration-guide.md
git commit -m "docs: update hydraulic motion algorithm contracts"
```

## Final Verification

- [ ] **Step 1: Confirm no unrelated changes were touched**

Run:

```bash
git status --short
```

Expected: only intended files are modified, or the working tree is clean after commits. If pre-existing user documentation changes remain, leave them untouched.

- [ ] **Step 2: Run all tests one final time**

Run:

```bash
ctest --test-dir out/build/unixgcc --output-on-failure
```

Expected: all tests pass.

- [ ] **Step 3: Inspect final commits**

Run:

```bash
git log --oneline -n 10
```

Expected: task commits appear in order:

```text
docs: update hydraulic motion algorithm contracts
feat: report injection vp transfer readiness
feat: add stable segment completion windows
feat: add injection-machine action profile builders
feat: add optional velocity feedback correction
fix: use segment deceleration for stop fallback
feat: add acceleration-limited motion planning
feat: apply diagnostic derate to pump outputs
```

## Self-Review

Spec coverage:

- Output derate gap is covered by Task 1.
- Position planner discontinuity gap is covered by Task 2.
- Stop deceleration mismatch is covered by Task 3.
- Speed-ramp open-loop gap is covered by Task 4.
- Injection-machine action adaptation gap is covered by Task 5.
- Completion stability gap is covered by Task 6.
- V/P transfer observation gap is covered by Task 7.
- Documentation and remaining boundary clarity are covered by Task 8.

Placeholder scan:

- The plan contains no unresolved placeholder markers.
- Every created module has concrete header, implementation, tests, commands, and commit steps.
- Remaining machine workflow responsibilities are explicitly marked as outside runtime scope, not deferred implementation.

Type consistency:

- New modules use `HYD_` prefix and C99 structs.
- New fields use existing `HYD_REAL`, `HYD_TIME`, `HYD_BOOL`, and enum style.
- Runtime integration keeps `PUMP_SPEED` nonnegative and direction in `STATE.plannedDirection`.
- `HYD_VpTransferReason` values are cast to the compact `HYD_UINT8` runtime state field for embedded memory discipline.
