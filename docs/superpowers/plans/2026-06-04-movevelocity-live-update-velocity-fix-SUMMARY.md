# MoveVelocity CONTINUOUSUPDATE Velocity Fix — Execution Summary

**One-liner:** Fixed MoveVelocity CONTINUOUSUPDATE rejecting VELOCITY=0/negative by adding SHORTEST_WAY direction derivation and fabs normalization in the live update path, relaxing validator for maxVelocity=0, and fixing test infrastructure to use global FB instances.

**Status:** DONE

**Plan:** 2026-06-04-movevelocity-live-update-velocity-fix
**Completed date:** 2025-06-04
**Duration:** ~30 min

---

## Task Summary

| Task | Name | Commit | Status |
|------|------|--------|--------|
| 1 | Add 3 failing tests for VELOCITY normalization | `db66f9a` | DONE |
| 2 | Fix `applyMoveVelocityLiveUpdate` — normalize VELOCITY and resolve direction | `ec5be29` | DONE |
| 3 | Fix `recipe_validator.c` — accept maxVelocity=0 for TIME_BASED SPEED_RAMP | `7bd461a` | DONE |
| 4 | Full regression verification | `df6da96` | DONE |

---

## Final Regression Results

- **Build:** Clean, no warnings
- **Tests:** 38/38 passed (100%), 0 failed
- **Key test:** `test_motion_interface_unit` — 256/256 assertions passed
  - `test_movevelocity_live_update_negative_velocity_flips_direction` — PASS
  - `test_movevelocity_live_update_zero_velocity_decel_to_stop` — PASS
  - `test_validate_segment_accepts_zero_maxvelocity_speed_ramp` — PASS

---

## Deviations from Plan

### Auto-fixed Issues

**1. [Rule 1 - Bug] Tests used local stack FB instead of global instance**
- **Found during:** Task 4 (regression)
- **Issue:** The three new tests (Tasks 1-2) created a local `HYD_MotionControlFB fb` on the stack and called `HYD_MotionControlFB_Scan(&fb)`, but `__mcl_cmd_MoveVelocity` operates on the global `HYD_MotionControlFB_inst[]` array. The tests were checking a zeroed stack variable instead of the actual active segment.
- **Fix:** Rewrote `test_movevelocity_live_update_negative_velocity_flips_direction` and `test_movevelocity_live_update_zero_velocity_decel_to_stop` to use the standard pattern: `__HydMotion_framework_Init()` + `ensure_axes_allocated(1)` + `__MK_GetPublic_MotionControlFB(0)`. The third test (`test_validate_segment_accepts_zero_maxvelocity_speed_ramp`) was unaffected as it calls `HYD_RecipeValidator_ValidateSegment` directly.
- **Files modified:** `tests/test_motion_interface_unit.c`
- **Commit:** `df6da96`

---

## Key Files

| File | Change |
|------|--------|
| `src/motion_interface.c` | Added SHORTEST_WAY direction derivation + `fabs()` normalization in `applyMoveVelocityLiveUpdate` |
| `src/recipe_validator.c` | Changed `maxVelocity <= 0.0` to `maxVelocity < 0.0` for TIME_BASED non-PRESSURE modes |
| `tests/test_motion_interface_unit.c` | Added 3 tests + fixed test infrastructure to use global FB instances |

---

## Decisions Made

- Used `fabs()` for velocity normalization (matching execRising path) rather than conditional sign flip
- SHORTEST_WAY direction derivation: positive velocity → POSITIVE, negative → NEGATIVE, zero → lastActiveDirection (default POSITIVE)
- Validator relaxed from `<= 0.0` to `< 0.0` for TIME_BASED non-PRESSURE modes only — the planner already safely handles maxVelocity=0

---

## Self-Check: PASSED

- [x] All 4 commits exist in git log
- [x] Build passes cleanly with no warnings
- [x] All 38 tests pass (100%)
- [x] 256/256 assertions pass in test_motion_interface_unit
