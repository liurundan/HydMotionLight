# HYD_MAX_SEGMENTS=1 Test Alignment Design

Date: 2026-06-18

## Goal

On the current embedded target, RAM is constrained enough that `HYD_MAX_SEGMENTS` must remain `1`. Multi-segment recipe workflows are therefore intentionally unavailable on this platform. The immediate goal is to make the test suite reflect that product constraint so `ctest` passes without pretending multi-segment recipe support still exists.

This design is intentionally narrow:

- keep `HYD_MAX_SEGMENTS == 1`
- do not restore or emulate multi-segment recipe execution
- do not change the production rejection path for oversize recipes
- update the six failing tests so they assert the current platform behavior

## Current Context

The current implementation already rejects `recipeSize > HYD_MAX_SEGMENTS` inside `HYD_RecipeValidator_ValidateRecipe(...)`, which causes `HYD_MotionControlFB_LoadRecipe(...)` to fail with `HYD_DIAG_CODE_RECIPE_TOO_LARGE`.

The six failing tests still assume multi-segment recipe loading succeeds:

- `tests/test_recipe_validator.c`
- `tests/test_sprint_b_integration.c`
- `tests/test_motion_interface_arbitration.c`
- `tests/test_recipe_multi_segment_ownership.c`
- `tests/test_rbf_pid_hil.c`
- `tests/test_vp_bumpless_reverse.c`

After a fresh rebuild, these tests fail because they still encode the older expectation that 2- or 3-segment recipes are valid.

## Chosen Approach

Use the smallest possible correction:

1. leave production behavior as-is
2. add a tiny shared test helper for asserting that an oversized recipe is rejected on this platform
3. rewrite the six failing tests so they validate rejection of multi-segment recipes instead of multi-segment runtime behavior
4. clarify the platform constraint in `include/hyd_config.h`

This is preferred over conditional dual-mode tests because the current hardware contract is explicit and the user asked for the simplest fix.

## Alternatives Considered

### 1. Inline six independent test rewrites

This would work, but it would duplicate the same rejection checks in multiple files and make future edits noisy.

### 2. Keep old multi-segment test bodies behind `#if HYD_MAX_SEGMENTS > 1`

This preserves old test logic for a future platform, but it adds complexity now and keeps unsupported behavior in the main path of the current suite.

### 3. Change production code to silently truncate multi-segment recipes to one segment

Rejected. That would hide unsupported input and change behavior beyond the requested test alignment.

## Design

### Production Behavior

Production behavior stays unchanged:

- `HYD_RecipeValidator_ValidateRecipe(...)` continues to reject `recipeSize > HYD_MAX_SEGMENTS`
- `HYD_MotionControlFB_LoadRecipe(...)` continues to clear the internal recipe buffer, set `RECIPE_SIZE` to zero, and report the diagnostic when validation fails

The only production-facing edit in this slice is a clarifying comment in `include/hyd_config.h` documenting that the current embedded target disables multi-segment recipe workflows by setting `HYD_MAX_SEGMENTS` to `1`.

### Shared Test Helper

Add a tiny helper under `tests/` to centralize the common assertion shape for oversized recipes.

Expected helper responsibilities:

- receive an initialized `HYD_MotionControlFB`
- attempt `HYD_MotionControlFB_LoadRecipe(...)` with `recipeSize > HYD_MAX_SEGMENTS`
- assert load failure
- assert `fb.RECIPE_SIZE == 0`
- assert runtime state remains non-active
- assert the reported diagnostic resolves to `HYD_DIAG_CODE_RECIPE_TOO_LARGE` if that field is exposed stably through the public FB state

The helper should stay local to tests and avoid any production linkage changes beyond adding its compilation unit to the relevant test targets if needed.

### Test File Changes

#### `tests/test_recipe_validator.c`

- keep a single-segment success-path test so recipe validation still proves the supported path
- change the old 2-segment success expectation into an explicit rejection test for oversized recipes

#### `tests/test_sprint_b_integration.c`

- remove the expectation that a 3-segment recipe can preload and start
- replace it with a targeted assertion that the 3-segment recipe is rejected on this platform
- keep any unrelated non-recipe planner/controller tests intact

#### `tests/test_motion_interface_arbitration.c`

- replace the 2-step recipe preload/advance scenario with a rejection assertion for the same 2-segment recipe definition
- do not keep `NextSegment` expectations in this platform configuration

#### `tests/test_recipe_multi_segment_ownership.c`

- reframe the test from “ownership across recipe advances” to “multi-segment ownership path is unavailable because recipe preload is rejected”
- keep the single-segment ownership test intact

#### `tests/test_rbf_pid_hil.c`

- replace the PI-to-RBF cross-segment recipe path with an assertion that the 2-segment recipe is rejected
- do not try to preserve cross-segment bumpless transfer behavior in a platform that forbids multi-segment recipes

#### `tests/test_vp_bumpless_reverse.c`

- replace both dual-segment recipe scenarios with rejection assertions for their 2-segment recipes
- keep any direct per-segment behavior tests intact if they do not rely on recipe advancement

## Verification Plan

Minimum verification for this slice:

```bash
cmake --build --preset unixgcc
ctest --test-dir out/build/unixgcc --output-on-failure
```

Success means:

- the six currently failing tests are updated and passing
- no previously passing tests regress
- full `ctest` is green on the current platform configuration

## Risks

The main tradeoff is reduced behavioral coverage for multi-segment recipe workflows. That is acceptable because those workflows are intentionally unsupported when `HYD_MAX_SEGMENTS == 1`.

The design avoids adding runtime conditionals or fallback behavior that could blur the platform contract.

## Out of Scope

- restoring multi-segment recipe support
- preserving old multi-segment runtime assertions behind compile-time branches
- changing `HYD_MAX_SEGMENTS`
- adding new production feature flags
- redesigning recipe ownership, `NextSegment`, or cross-segment blending

## Future Optimization

If a future target restores `HYD_MAX_SEGMENTS > 1`, the clean follow-up is to add capability-aware branching in the tests or split multi-segment coverage into a separate configuration profile. That work is intentionally deferred from this slice.
