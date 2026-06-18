# HYD_MAX_SEGMENTS=1 Test Alignment Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Align the six multi-segment recipe tests with the current embedded platform contract where `HYD_MAX_SEGMENTS == 1`, so the full `ctest` suite passes without changing production multi-segment rejection behavior.

**Architecture:** Keep the production code path unchanged: oversized recipes continue to fail in `HYD_RecipeValidator_ValidateRecipe(...)` and `HYD_MotionControlFB_LoadRecipe(...)`. Add one tiny header-only test helper to centralize rejection assertions, update the six stale tests to assert rejection instead of multi-segment execution, and clarify the platform constraint in `include/hyd_config.h`.

**Tech Stack:** C99, CMake, CTest, existing HydroMotion test executables.

---

## File Structure

- Create: `tests/test_recipe_rejection_helpers.h`
  - Header-only helper for asserting oversized recipe rejection through `ValidateRecipe(...)` and `LoadRecipe(...)`.
- Modify: `include/hyd_config.h`
  - Clarify that the current embedded target disables multi-segment recipe workflows by pinning `HYD_MAX_SEGMENTS` to `1`.
- Modify: `tests/test_recipe_validator.c`
  - Keep a single-segment success path and add an explicit oversized-recipe rejection test.
- Modify: `tests/test_sprint_b_integration.c`
  - Replace the 3-segment preload/start expectation with explicit rejection coverage.
- Modify: `tests/test_motion_interface_arbitration.c`
  - Replace the two-step recipe preload/advance expectation with explicit rejection coverage.
- Modify: `tests/test_recipe_multi_segment_ownership.c`
  - Replace the 3-segment ownership/advance scenario with explicit rejection coverage while preserving the single-segment ownership path.
- Modify: `tests/test_rbf_pid_hil.c`
  - Replace the PI->RBF two-segment HIL recipe expectation with explicit rejection coverage.
- Modify: `tests/test_vp_bumpless_reverse.c`
  - Replace the two two-segment recipe blending scenarios with explicit rejection coverage.

## Task 1: Add Shared Oversized-Recipe Test Helper And Reframe Recipe Validator

**Files:**
- Create: `tests/test_recipe_rejection_helpers.h`
- Modify: `tests/test_recipe_validator.c`
- Modify: `include/hyd_config.h`

- [ ] **Step 1: Update the recipe validator test to express the new platform contract before the helper exists**

Edit `tests/test_recipe_validator.c` so it includes the new helper header, changes the success-path recipe to one segment, and adds an explicit oversized-recipe rejection test:

```c
#include "recipe_validator.h"
#include "test_recipe_rejection_helpers.h"
#include "segment_limits.h"

static void test_validate_recipe_success(void) {
    HYD_MotionSegment recipe[1];
    HYD_DiagnosticCode code = HYD_DIAG_CODE_INTERNAL_ERROR;

    printf("Testing recipe validator success path...\n");
    recipe[0] = make_valid_segment();

    assert(HYD_RecipeValidator_ValidateRecipe(recipe, 1, &code));
    assert(code == HYD_DIAG_CODE_NONE);
    printf("✓ Recipe validator success path test passed\n");
}

static void test_validate_recipe_rejects_oversized_recipe_for_current_platform(void) {
    HYD_MotionSegment recipe[2];

    printf("Testing oversized recipe rejection on current platform...\n");
    recipe[0] = make_valid_segment();
    recipe[1] = make_valid_segment();
    recipe[1].direction = HYD_DIRECTION_RETRACT;

    assert_oversized_recipe_validation_rejected(recipe, 2U);
    printf("✓ Oversized recipe rejection test passed\n");
}
```

Add the new test to `main()`:

```c
int main(void) {
    test_validate_recipe_success();
    test_validate_recipe_rejects_oversized_recipe_for_current_platform();
    test_validate_recipe_rejects_speed_ramp_non_time_planner();
    test_validate_runtime_config();
    test_validate_pressure_derivative_filter_alpha();
    test_validate_start_context_direction_conflict();
    test_invalid_ceiling_tolerance_rejected();
    return 0;
}
```

- [ ] **Step 2: Build the target and verify it fails because the helper header does not exist yet**

Run:

```bash
cmake --build --preset unixgcc --target test_recipe_validator
```

Expected: build failure complaining that `test_recipe_rejection_helpers.h` is missing or that `assert_oversized_recipe_validation_rejected(...)` is undefined.

- [ ] **Step 3: Add the helper header and document the platform limit**

Create `tests/test_recipe_rejection_helpers.h` with header-only helpers:

```c
#ifndef TEST_RECIPE_REJECTION_HELPERS_H
#define TEST_RECIPE_REJECTION_HELPERS_H

#include <assert.h>
#include <stddef.h>

#include "motion_control.h"
#include "recipe_validator.h"

static void assert_oversized_recipe_validation_rejected(const HYD_MotionSegment* recipe,
                                                        size_t recipeSize) {
    HYD_DiagnosticCode code = HYD_DIAG_CODE_NONE;

    assert(recipe != NULL);
    assert(recipeSize > HYD_MAX_SEGMENTS);
    assert(!HYD_RecipeValidator_ValidateRecipe(recipe, recipeSize, &code));
    assert(code == HYD_DIAG_CODE_RECIPE_TOO_LARGE);
}

static void assert_oversized_recipe_load_rejected(HYD_MotionControlFB* fb,
                                                  const HYD_MotionSegment* recipe,
                                                  size_t recipeSize) {
    assert(fb != NULL);
    assert(recipe != NULL);
    assert(recipeSize > HYD_MAX_SEGMENTS);
    assert(!HYD_MotionControlFB_LoadRecipe(fb, recipe, recipeSize));
    assert(fb->RECIPE_SIZE == 0U);
    assert(!fb->STATE.active);
    assert(fb->DIAGNOSTIC.code == HYD_DIAG_CODE_RECIPE_TOO_LARGE);
}

#endif
```

Clarify `include/hyd_config.h` next to `HYD_MAX_SEGMENTS`:

```c
/* 最大配方段数
 * 默认: 16段
 * 最小: 4段（至少支持基本的注塑周期）
 * 当前嵌入式目标因 RAM 受限固定为 1，这会禁用多段配方工作流；
 * 所有多段 recipe 输入都应在验证/加载阶段被拒绝。
 * 影响: RAM占用约 sizeof(HYD_MotionSegment) * HYD_MAX_SEGMENTS 字节
 */
#define HYD_MAX_SEGMENTS 1
```

- [ ] **Step 4: Run the recipe validator target and verify the new contract passes**

Run:

```bash
cmake --build --preset unixgcc --target test_recipe_validator
ctest --test-dir out/build/unixgcc -R '^test_recipe_validator$' --output-on-failure
```

Expected: `test_recipe_validator` passes, proving both the supported 1-segment path and the explicit oversized-recipe rejection path.

- [ ] **Step 5: Commit the helper/comment/validator slice**

```bash
git add include/hyd_config.h tests/test_recipe_rejection_helpers.h tests/test_recipe_validator.c
git commit -m "Document the single-segment recipe contract in tests" -m "Constraint: The current embedded target must keep HYD_MAX_SEGMENTS at 1, so multi-segment recipes are intentionally unsupported
Rejected: Silently truncating oversized recipes to one segment | The platform contract should reject unsupported inputs explicitly
Confidence: high
Scope-risk: narrow
Directive: Reuse the shared oversized-recipe helper instead of hand-writing divergent rejection assertions
Tested: ctest --test-dir out/build/unixgcc -R '^test_recipe_validator$' --output-on-failure
Not-tested: Full suite not run in this slice"
```

### Task 2: Retarget Sprint-B And Motion-Interface Arbitration Tests

**Files:**
- Modify: `tests/test_sprint_b_integration.c`
- Modify: `tests/test_motion_interface_arbitration.c`

- [ ] **Step 1: Replace the stale multi-segment expectations with explicit rejection assertions**

In `tests/test_sprint_b_integration.c`, add the helper include and replace the first recipe-loading test:

```c
#include "motion_control.h"
#include "motion_planner.h"
#include "test_recipe_rejection_helpers.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static void test_fb_rejects_multi_segment_recipe_when_platform_limit_is_one(void) {
    HYD_MotionControlFB fb;
    HYD_MotionSegment recipe[3];

    printf("Testing FB rejects multi-segment recipes on current platform...\n");

    HYD_MotionControlFB_Init(&fb);
    fb.FLOW_TO_PUMP_SPEED_GAIN = 10.0;
    fb.PUMP_SPEED_LIMIT = 3000.0;
    fb.USE_RECIPE = true;

    memset(recipe, 0, sizeof(recipe));

    recipe[0].segmentTag = 1;
    recipe[0].segmentType = HYD_SEGMENT_TYPE_CLAMPING;
    recipe[0].planner = HYD_PLANNER_TIME_BASED;
    recipe[0].mode = HYD_MODE_POSITION;
    recipe[0].endCondition = HYD_END_POSITION;
    recipe[0].direction = HYD_DIRECTION_EXTEND;
    recipe[0].targetPosition = 100.0;
    recipe[0].maxAcceleration = 20.0;
    recipe[0].maxVelocity = 40.0;
    recipe[0].maxFlow = 50.0;
    recipe[0].velocityToFlowGain = 1.0;
    recipe[0].positionTolerance = 0.5;
    recipe[0].timeoutLimit = 10.0;

    recipe[1].segmentTag = 2;
    recipe[1].segmentType = HYD_SEGMENT_TYPE_INJECTION;
    recipe[1].planner = HYD_PLANNER_TIME_BASED;
    recipe[1].mode = HYD_MODE_SPEED_RAMP;
    recipe[1].endCondition = HYD_END_TIME;
    recipe[1].direction = HYD_DIRECTION_EXTEND;
    recipe[1].duration = 0.5;
    recipe[1].maxAcceleration = 10.0;
    recipe[1].maxVelocity = 30.0;
    recipe[1].maxFlow = 60.0;
    recipe[1].velocityToFlowGain = 1.0;
    recipe[1].timeoutLimit = 5.0;

    recipe[2].segmentTag = 3;
    recipe[2].segmentType = HYD_SEGMENT_TYPE_HOLDING;
    recipe[2].mode = HYD_MODE_PRESSURE_CLOSED_LOOP;
    recipe[2].endCondition = HYD_END_TIME;
    recipe[2].direction = HYD_DIRECTION_HOLD;
    recipe[2].targetPressure = 80.0;
    recipe[2].targetFlow = 3.0;
    recipe[2].duration = 0.2;
    recipe[2].maxFlow = 15.0;
    recipe[2].pressureController = HYD_PRESSURE_CONTROLLER_P;
    recipe[2].pressureKp = 0.3;
    recipe[2].pressureKpHigh = 1.0;
    recipe[2].pressureGainBand = 0.2;
    recipe[2].pressureTolerance = 2.0;
    recipe[2].timeoutLimit = 3.0;

    assert_oversized_recipe_load_rejected(&fb, recipe, 3U);
    printf("✓ FB rejects multi-segment recipes on current platform\n");
}
```

Update the call site in `main()` to use `test_fb_rejects_multi_segment_recipe_when_platform_limit_is_one();`.

In `tests/test_motion_interface_arbitration.c`, add the helper include and replace the two-step recipe test:

```c
static void test_moveprofile_rejects_two_segment_recipe_when_platform_limit_is_one(void) {
    HYD_CREATEMOTION cm;
    HYD_MOVEPROFILE mp;
    HYD_AXISMOTION motion;
    HYD_MotionControlFB* fb;
    HYD_MotionSegment recipe[2];

    __HydMotion_framework_Init();

    memset(&cm, 0, sizeof(cm));
    IEC_VAL(cm.EN) = true;
    IEC_VAL(cm.USE_RECIPE) = true;
    IEC_VAL(cm.FLOW_TO_PUMPSPEED) = 1.0f;
    IEC_VAL(cm.PUMPSPEED_LIMIT) = 3000.0f;
    IEC_VAL(cm.USE_SIMULATION) = false;
    __mcl_cmd_CreateMotion(&cm);
    fb = __MK_GetPublic_MotionControlFB((int)IEC_VAL(cm.AXISID));
    ASSERT_TRUE(fb != NULL, "Recipe axis should expose an FB");

    memset(recipe, 0, sizeof(recipe));
    recipe[0].segmentTag = 1;
    recipe[0].segmentType = HYD_SEGMENT_TYPE_HOLDING;
    recipe[0].mode = HYD_MODE_PRESSURE_CLOSED_LOOP;
    recipe[0].endCondition = HYD_END_TIME;
    recipe[0].direction = HYD_DIRECTION_HOLD;
    recipe[0].targetPressure = 5.0f;
    recipe[0].targetFlow = 1.0f;
    recipe[0].maxFlow = 5.0f;
    recipe[0].duration = 0.002f;
    recipe[0].pressureController = HYD_PRESSURE_CONTROLLER_P;
    recipe[0].pressureKp = 0.5f;
    recipe[0].pressureTolerance = 0.5f;
    recipe[0].timeoutLimit = 1.0f;

    recipe[1] = recipe[0];
    recipe[1].segmentTag = 2;

    assert_oversized_recipe_load_rejected(fb, recipe, 2U);
    ASSERT_TRUE(IEC_VAL(mp.COMMANDABORTED) == false,
               "Rejecting an oversized recipe should not fabricate a MoveProfile abort");
}
```

Update the call site in `main()` to use `test_moveprofile_rejects_two_segment_recipe_when_platform_limit_is_one();`.

- [ ] **Step 2: Run the two affected targets**

Run:

```bash
cmake --build --preset unixgcc --target test_sprint_b_integration test_motion_interface_arbitration
ctest --test-dir out/build/unixgcc -R '^(test_sprint_b_integration|test_motion_interface_arbitration)$' --output-on-failure
```

Expected: both targets pass with explicit multi-segment rejection coverage.

- [ ] **Step 3: Commit the sprint/arbitration slice**

```bash
git add tests/test_sprint_b_integration.c tests/test_motion_interface_arbitration.c
git commit -m "Align integration recipe tests with the one-segment platform" -m "Constraint: HYD_MAX_SEGMENTS is fixed at 1 on the current target, so recipe preload tests must reject multi-segment inputs
Rejected: Keeping old NextSegment execution expectations in these tests | Those paths are unreachable on this platform
Confidence: high
Scope-risk: narrow
Directive: When a test sets up a 2+ segment recipe on this target, assert rejection early instead of simulating runtime advancement
Tested: ctest --test-dir out/build/unixgcc -R '^(test_sprint_b_integration|test_motion_interface_arbitration)$' --output-on-failure
Not-tested: Full suite not run in this slice"
```

### Task 3: Retarget Recipe Ownership Coverage To The Single-Segment Platform

**Files:**
- Modify: `tests/test_recipe_multi_segment_ownership.c`

- [ ] **Step 1: Replace the 3-segment ownership path with explicit rejection coverage**

Add the helper include and replace the first test body:

```c
#include "motion_control.h"
#include "test_recipe_rejection_helpers.h"

static void test_multi_segment_recipe_is_rejected_when_platform_limit_is_one(void) {
    HYD_CREATEMOTION cm;
    HYD_MotionSegment seg[3];
    HYD_MotionControlFB* fb;
    int axisIndex;
    int i;

    printf("Running: test_multi_segment_recipe_is_rejected_when_platform_limit_is_one\n");

    __HydMotion_framework_Init();

    memset(&cm, 0, sizeof(cm));
    IEC_VAL(cm.EN) = true;
    IEC_VAL(cm.USE_RECIPE) = true;
    IEC_VAL(cm.FLOW_TO_PUMPSPEED) = 1.0f;
    IEC_VAL(cm.PUMPSPEED_LIMIT) = 5000.0f;
    IEC_VAL(cm.USE_SIMULATION) = true;
    __mcl_cmd_CreateMotion(&cm);
    axisIndex = (int)IEC_VAL(cm.AXISID);
    ASSERT_TRUE(axisIndex >= 0, "CreateMotion should succeed");

    fb = __MK_GetPublic_MotionControlFB(axisIndex);
    ASSERT_TRUE(fb != NULL, "Should fetch axis FB");

    for (i = 0; i < 3; i++) {
        build_clamp_close_segment(&seg[i], (HYD_UINT8)(i + 1), (HYD_REAL)((i + 1) * 30));
    }

    assert_oversized_recipe_load_rejected(fb, seg, 3U);
    ASSERT_TRUE(fb->DIAGNOSTIC.code == HYD_DIAG_CODE_RECIPE_TOO_LARGE,
                "Oversized recipes should report RECIPE_TOO_LARGE");
}
```

Update `main()` to call `test_multi_segment_recipe_is_rejected_when_platform_limit_is_one();` while keeping the existing single-segment stop/takeover regression test intact.

- [ ] **Step 2: Run the ownership target**

Run:

```bash
cmake --build --preset unixgcc --target test_recipe_multi_segment_ownership
ctest --test-dir out/build/unixgcc -R '^test_recipe_multi_segment_ownership$' --output-on-failure
```

Expected: the target passes, with the first test asserting platform rejection and the single-segment regression still intact.

- [ ] **Step 3: Commit the ownership slice**

```bash
git add tests/test_recipe_multi_segment_ownership.c
git commit -m "Retarget recipe ownership tests to the single-segment platform" -m "Constraint: Multi-segment recipe ownership cannot be exercised when the platform only permits one loaded segment
Rejected: Emulating NextSegment behavior in the test harness | That would test a capability the target intentionally disables
Confidence: high
Scope-risk: narrow
Directive: Keep ownership regression coverage on reachable single-segment paths only while HYD_MAX_SEGMENTS remains 1
Tested: ctest --test-dir out/build/unixgcc -R '^test_recipe_multi_segment_ownership$' --output-on-failure
Not-tested: Full suite not run in this slice"
```

### Task 4: Retarget HIL And Bumpless-Transition Recipe Tests

**Files:**
- Modify: `tests/test_rbf_pid_hil.c`
- Modify: `tests/test_vp_bumpless_reverse.c`

- [ ] **Step 1: Replace the multi-segment recipe expectations with explicit rejection tests**

In `tests/test_rbf_pid_hil.c`, add the helper include and replace the cross-segment HIL test:

```c
#include "motion_control.h"
#include "pressure_model.h"
#include "test_recipe_rejection_helpers.h"

static void test_pi_to_rbf_recipe_is_rejected_when_platform_limit_is_one(void) {
    HYD_MotionControlFB fb;
    HYD_MotionSegment recipe[2];

    printf("HIL scenario B — multi-segment PI->RBF recipe rejected on current platform...\n");

    HYD_MotionControlFB_Init(&fb);
    memset(recipe, 0, sizeof(recipe));

    recipe[0].segmentType = HYD_SEGMENT_TYPE_HOLDING;
    recipe[0].mode = HYD_MODE_PRESSURE_CLOSED_LOOP;
    recipe[0].endCondition = HYD_END_TIME;
    recipe[0].direction = HYD_DIRECTION_HOLD;
    recipe[0].duration = HIL_SCENARIO_B_PI_DUR_S;
    recipe[0].targetPressure = HIL_TARGET_B1_MPA;
    recipe[0].pressureCeiling = 20.0;
    recipe[0].pressureRampRate = 10.0;
    recipe[0].pressureFilterAlpha = 1.0;
    recipe[0].pressureDerivativeFilterAlpha = 1.0;
    recipe[0].maxFlow = 30.0;
    recipe[0].pressureController = HYD_PRESSURE_CONTROLLER_PI;
    recipe[0].pressureKp = 0.5;
    recipe[0].pressureKi = 0.1;
    recipe[0].pressureIntegralLimit = 5.0;

    recipe[1].segmentType = HYD_SEGMENT_TYPE_HOLDING;
    recipe[1].mode = HYD_MODE_PRESSURE_CLOSED_LOOP;
    recipe[1].endCondition = HYD_END_TIME;
    recipe[1].direction = HYD_DIRECTION_HOLD;
    recipe[1].duration = HIL_SCENARIO_B_RBF_DUR_S;
    recipe[1].targetPressure = HIL_TARGET_B2_MPA;
    recipe[1].pressureCeiling = 20.0;
    recipe[1].pressureRampRate = 10.0;
    recipe[1].pressureFilterAlpha = 1.0;
    recipe[1].pressureDerivativeFilterAlpha = 1.0;
    recipe[1].maxFlow = 30.0;
    recipe[1].pressureController = HYD_PRESSURE_CONTROLLER_RBF_PID;
    recipe[1].pressureRbfConfig.minKp = 0.5;
    recipe[1].pressureRbfConfig.maxKp = 1.2;
    recipe[1].pressureRbfConfig.minKi = 0.005;
    recipe[1].pressureRbfConfig.maxKi = 0.050;
    recipe[1].pressureRbfConfig.minKd = 0.5;
    recipe[1].pressureRbfConfig.maxKd = 2.0;

    assert_oversized_recipe_load_rejected(&fb, recipe, 2U);
}
```

Update `main()` to call `test_pi_to_rbf_recipe_is_rejected_when_platform_limit_is_one();`.

In `tests/test_vp_bumpless_reverse.c`, add the helper include and replace both tests:

```c
#include "motion_control.h"
#include "test_recipe_rejection_helpers.h"

static void test_pressure_to_speed_recipe_is_rejected_when_platform_limit_is_one(void) {
    HYD_MotionControlFB fb;
    HYD_MotionSegment segP, segV;
    HYD_MotionSegment recipe[2];

    printf("Testing P->V recipe rejection on current platform...\n");

    HYD_MotionControlFB_Init(&fb);
    memset(&segP, 0, sizeof(segP));
    memset(&segV, 0, sizeof(segV));

    segP.segmentType = HYD_SEGMENT_TYPE_HOLDING;
    segP.mode = HYD_MODE_PRESSURE_CLOSED_LOOP;
    segP.endCondition = HYD_END_TIME;
    segP.duration = 0.5;
    segP.targetPressure = 10.0;
    segP.pressureController = HYD_PRESSURE_CONTROLLER_PI;
    segP.pressureKp = 0.5;
    segP.pressureKi = 0.2;
    segP.pressureIntegralLimit = 10.0;
    segP.maxFlow = 30.0;
    segP.pressureFilterAlpha = 1.0;
    segP.pressureDerivativeFilterAlpha = 1.0;
    segP.direction = HYD_DIRECTION_EXTEND;

    segV.segmentType = HYD_SEGMENT_TYPE_INJECTION;
    segV.mode = HYD_MODE_SPEED_RAMP;
    segV.planner = HYD_PLANNER_TIME_BASED;
    segV.endCondition = HYD_END_TIME;
    segV.duration = 1.0;
    segV.targetFlow = 20.0;
    segV.maxAcceleration = 100.0;
    segV.maxVelocity = 50.0;
    segV.maxFlow = 30.0;
    segV.velocityToFlowGain = 0.2;
    segV.direction = HYD_DIRECTION_EXTEND;
    segV.pressureFilterAlpha = 1.0;
    segV.pressureDerivativeFilterAlpha = 1.0;

    memcpy(&recipe[0], &segP, sizeof(HYD_MotionSegment));
    memcpy(&recipe[1], &segV, sizeof(HYD_MotionSegment));

    assert_oversized_recipe_load_rejected(&fb, recipe, 2U);
}

static void test_speed_to_speed_recipe_is_rejected_when_platform_limit_is_one(void) {
    HYD_MotionControlFB fb;
    HYD_MotionSegment segA, segB;
    HYD_MotionSegment recipe[2];

    printf("Testing S->S recipe rejection on current platform...\n");

    HYD_MotionControlFB_Init(&fb);
    memset(&segA, 0, sizeof(segA));
    memset(&segB, 0, sizeof(segB));

    segA.segmentType = HYD_SEGMENT_TYPE_INJECTION;
    segA.mode = HYD_MODE_SPEED_RAMP;
    segA.planner = HYD_PLANNER_TIME_BASED;
    segA.endCondition = HYD_END_TIME;
    segA.duration = 0.5;
    segA.targetFlow = 10.0;
    segA.maxAcceleration = 100.0;
    segA.maxVelocity = 20.0;
    segA.maxFlow = 20.0;
    segA.velocityToFlowGain = 0.2;
    segA.direction = HYD_DIRECTION_EXTEND;
    segA.pressureFilterAlpha = 1.0;
    segA.pressureDerivativeFilterAlpha = 1.0;

    segB.segmentType = HYD_SEGMENT_TYPE_INJECTION;
    segB.mode = HYD_MODE_SPEED_RAMP;
    segB.planner = HYD_PLANNER_TIME_BASED;
    segB.endCondition = HYD_END_TIME;
    segB.duration = 1.0;
    segB.targetFlow = 25.0;
    segB.maxAcceleration = 100.0;
    segB.maxVelocity = 50.0;
    segB.maxFlow = 30.0;
    segB.velocityToFlowGain = 0.2;
    segB.direction = HYD_DIRECTION_EXTEND;
    segB.pressureFilterAlpha = 1.0;
    segB.pressureDerivativeFilterAlpha = 1.0;

    memcpy(&recipe[0], &segA, sizeof(HYD_MotionSegment));
    memcpy(&recipe[1], &segB, sizeof(HYD_MotionSegment));

    assert_oversized_recipe_load_rejected(&fb, recipe, 2U);
}
```

Update `main()` to call:

```c
int main(void) {
    printf("Running VP bumpless reverse / blending tests...\n\n");
    test_pressure_to_speed_recipe_is_rejected_when_platform_limit_is_one();
    test_speed_to_speed_recipe_is_rejected_when_platform_limit_is_one();
    printf("\nAll bumpless/blending tests passed.\n");
    return 0;
}
```

- [ ] **Step 2: Run the HIL and bumpless targets**

Run:

```bash
cmake --build --preset unixgcc --target test_rbf_pid_hil test_vp_bumpless_reverse
ctest --test-dir out/build/unixgcc -R '^(test_rbf_pid_hil|test_vp_bumpless_reverse)$' --output-on-failure
```

Expected: both targets pass, now asserting recipe rejection instead of unreachable multi-segment transitions.

- [ ] **Step 3: Commit the HIL/bumpless slice**

```bash
git add tests/test_rbf_pid_hil.c tests/test_vp_bumpless_reverse.c
git commit -m "Retarget cross-segment pressure tests to the current platform limit" -m "Constraint: The current target cannot load multi-segment recipes, so cross-segment PI/RBF and bumpless-transition recipe tests must assert rejection
Rejected: Preserving cross-segment runtime assertions in a dead path | Those transitions cannot execute while HYD_MAX_SEGMENTS is 1
Confidence: high
Scope-risk: narrow
Directive: Keep any future cross-segment pressure or blending coverage behind a real multi-segment-capable configuration, not this target profile
Tested: ctest --test-dir out/build/unixgcc -R '^(test_rbf_pid_hil|test_vp_bumpless_reverse)$' --output-on-failure
Not-tested: Full suite not run in this slice"
```

### Task 5: Run Fresh Full Verification

**Files:**
- No code changes

- [ ] **Step 1: Rebuild the full test suite**

Run:

```bash
cmake --build --preset unixgcc
```

Expected: all test executables rebuild successfully with the aligned expectations.

- [ ] **Step 2: Run the full CTest suite**

Run:

```bash
ctest --test-dir out/build/unixgcc --output-on-failure
```

Expected: the full suite passes, including the six tests that previously assumed multi-segment recipes were still available.

- [ ] **Step 3: Verify no unintended edits remain**

Run:

```bash
git status --short
```

Expected: only the intended tracked changes from this plan remain, or the tree is clean if every task commit has already landed.

## Self-Review Checklist

### Spec coverage

- keep `HYD_MAX_SEGMENTS == 1`: preserved in Task 1
- do not change production rejection path: preserved by limiting edits to tests and config comment
- add a tiny shared test helper: covered in Task 1
- update all six failing tests: covered in Tasks 1 through 4
- achieve full `ctest` green: covered in Task 5

### Placeholder scan

- No `TODO`, `TBD`, or “similar to Task N” placeholders
- Every code-changing step includes concrete code blocks
- Every verification step includes exact commands and expected results

### Type consistency

- shared helper names are defined once in `tests/test_recipe_rejection_helpers.h` and reused consistently:
  - `assert_oversized_recipe_validation_rejected(...)`
  - `assert_oversized_recipe_load_rejected(...)`
- diagnostic assertion uses the existing public field `fb->DIAGNOSTIC.code`
- all recipe-size assertions use `size_t` literals with `U` suffix where appropriate
