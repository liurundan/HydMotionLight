# Five-Point Toggle Motion Control Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add online five-point toggle kinematics to clamp axes while preserving platen-coordinate PLCopen semantics and legacy direct-axis behavior.

**Architecture:** Keep the motion planner in platen coordinates and insert an enum-dispatched actuation mapper before hydraulic flow conversion. Store prepared toggle geometry in a bounded static pool, validate candidate configurations incrementally in a separately bounded workspace, and expose atomic IEC configuration FBs.

**Tech Stack:** C99, single-precision `HYD_REAL`, matiec IEC structs/macros, static bounded memory, CMake preset `unixgcc`, standalone C regression tests, Python interface-layout check, Linux monotonic-clock benchmark.

**Design:** `docs/superpowers/specs/2026-07-29-five-point-toggle-motion-control-architecture-design.md`

---

## File Structure

### New focused modules

- `include/toggle_kinematics.h`: geometry types, default configuration, online solve, inverse solve, and incremental validation API.
- `src/toggle_kinematics.c`: formula implementation with no motion-control or IEC dependencies.
- `include/toggle_mechanism_pool.h`: bounded slot and validation-workspace ownership API.
- `src/toggle_mechanism_pool.c`: active configuration pool, atomic commits, and reusable validation workspaces.
- `include/actuation_mapper.h`: platen-to-actuator velocity/flow mapping contract.
- `src/actuation_mapper.c`: direct identity mapping and five-point mapping.
- `tests/test_toggle_kinematics.c`: formula, validation, inverse, and numerical protection tests.
- `tests/test_toggle_mechanism_pool.c`: allocation, atomic commit, version, and workspace tests.
- `tests/test_actuation_mapper.c`: direction, area, fallback gain, and direct compatibility tests.
- `tests/test_toggle_motion_interface.c`: CreateMotion, IEC configuration/readback, and failure-transaction tests.
- `tests/test_toggle_motion_integration.c`: position/velocity/pressure/Stop/blending/runtime-fault integration tests.

### Existing files to modify

- `include/hyd_config.h`: compile-time pool/workspace limits and optional mechanism telemetry switch.
- `include/common_types.h`: mechanism enum, diagnostic codes, and actuator-facing motion state.
- `include/motion_control.h`: axis mechanism binding and shared actuation-mapping helpers.
- `src/motion_control.c`: main control, carryover, Stop, blending, pressure bypass, and runtime-fault integration.
- `include/motion_interface.h`: CreateMotion input, configure/read FBs, ReadStatus outputs, and prototypes.
- `src/motion_interface.c`: slot-aware axis allocation, IEC state machines, status publication, and actuator-direction arbitration.
- `pousHydMotion.xml`: matching IEC public POU definitions.
- `tests/plcdemo/POUS.h`: generated-equivalent IEC layouts.
- `tests/plcdemo/POUS.c`: generated-equivalent initializers and wrapper bodies.
- `tests/test_interface_layout_consistency.py`: assertions for the new public fields and initializers.
- `tests/benchmark_performance.c`: high-resolution kinematics and full-axis benchmarks with anti-optimization checksum.
- `CMakeLists.txt`: register the five new test executables.

## Fixed Contracts

- Default geometry: `150, 230, 135, 75, 60, 130, 100, 378, 202 mm`; branches `-1, -1, -1`; `xHandoff=0` means automatic.
- Full physical `[0, Sm]` must be geometrically reachable. `xGeometryMin` may exclude only numerically unsafe ordinary-speed control near the locking region.
- `HYD_AxisRef.position/velocity`, planner targets, completion, diagnostics, and soft limits remain `Xm/Vm`.
- Pressure-loop output is actuator-side flow and bypasses kinematic flow mapping.
- No heap allocation and no runtime function-pointer registry.
- Existing `HYD_ParameterNumber` numeric values remain unchanged.
- STM32 timing is not an acceptance claim in this environment; PC timing is regression evidence only.

### Task 1: Online Kinematics Core

**Files:**
- Create: `include/toggle_kinematics.h`
- Create: `src/toggle_kinematics.c`
- Create: `tests/test_toggle_kinematics.c`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Write the failing default-geometry and golden-point tests**

Create `tests/test_toggle_kinematics.c` with explicit default values and double-derived golden points:

```c
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "toggle_kinematics.h"

static void assert_near(HYD_REAL actual, HYD_REAL expected, HYD_REAL tolerance) {
    assert(fabsf(actual - expected) <= tolerance);
}

static void test_default_prepare(void) {
    HYD_ToggleGeometryConfig raw = HYD_ToggleKinematics_DefaultConfig();
    HYD_TogglePreparedConfig prepared;
    HYD_ToggleError error = HYD_TOGGLE_ERROR_NONE;

    assert(raw.dc == 378.0f);
    assert(HYD_ToggleKinematics_Prepare(&raw, &prepared, &error));
    assert(error == HYD_TOGGLE_ERROR_NONE);
    assert_near(prepared.aP, 117.0f, 1.0e-5f);
    assert_near(prepared.bP, -67.349833f, 1.0e-4f);
}

static void test_online_golden_points(void) {
    static const struct {
        HYD_REAL xm;
        HYD_REAL xs;
        HYD_REAL k;
    } cases[] = {
        {0.0f,   64.910771f,  -10.150074f},
        {50.0f, -20.397682f,   -0.935184f},
        {101.0f,-63.808094f,   -0.808380f},
        {202.0f,-138.295657f,  -0.520517f}
    };
    HYD_ToggleGeometryConfig raw = HYD_ToggleKinematics_DefaultConfig();
    HYD_TogglePreparedConfig prepared;
    HYD_ToggleSolution solution;
    HYD_ToggleError error;
    size_t i;

    assert(HYD_ToggleKinematics_Prepare(&raw, &prepared, &error));
    for (i = 0U; i < sizeof(cases) / sizeof(cases[0]); ++i) {
        assert(HYD_ToggleKinematics_SolveOnline(
            &prepared, cases[i].xm, 10.0f, &solution, &error));
        assert_near(solution.xs, cases[i].xs, 2.0e-3f);
        assert_near(solution.velocityRatio, cases[i].k, 2.0e-4f);
        assert_near(solution.vs, cases[i].k * 10.0f, 2.0e-3f);
    }
}

int main(void) {
    test_default_prepare();
    test_online_golden_points();
    puts("toggle kinematics core tests passed");
    return 0;
}
```

- [ ] **Step 2: Register the test and verify the missing API fails the build**

Add to `CMakeLists.txt` near the other core unit tests:

```cmake
add_executable(test_toggle_kinematics tests/test_toggle_kinematics.c)
target_link_libraries(test_toggle_kinematics PRIVATE HydroMotionLib m)
```

Add after `enable_testing()`:

```cmake
add_test(NAME test_toggle_kinematics COMMAND test_toggle_kinematics)
```

Run:

```bash
cmake --preset unixgcc
cmake --build --preset unixgcc --target test_toggle_kinematics
```

Expected: compilation fails because `toggle_kinematics.h` and its symbols do not exist.

- [ ] **Step 3: Define compact public types and stable error values**

Create `include/toggle_kinematics.h` with include guards and these exact contracts:

```c
#ifndef HYD_TOGGLE_KINEMATICS_H
#define HYD_TOGGLE_KINEMATICS_H

#include <stdint.h>
#include "common_types.h"

typedef enum {
    HYD_TOGGLE_ERROR_NONE = 0,
    HYD_TOGGLE_ERROR_NULL_ARGUMENT,
    HYD_TOGGLE_ERROR_NONFINITE_PARAMETER,
    HYD_TOGGLE_ERROR_NONPOSITIVE_LENGTH,
    HYD_TOGGLE_ERROR_INVALID_BRANCH,
    HYD_TOGGLE_ERROR_FIXED_TRIANGLE_INVALID,
    HYD_TOGGLE_ERROR_MAIN_LINKAGE_UNREACHABLE,
    HYD_TOGGLE_ERROR_DRIVE_LINK_UNREACHABLE,
    HYD_TOGGLE_ERROR_MAIN_JACOBIAN_UNSAFE,
    HYD_TOGGLE_ERROR_DRIVE_PROJECTION_UNSAFE,
    HYD_TOGGLE_ERROR_VELOCITY_RATIO_UNSAFE,
    HYD_TOGGLE_ERROR_POSITION_OUT_OF_RANGE,
    HYD_TOGGLE_ERROR_NONFINITE_RESULT,
    HYD_TOGGLE_ERROR_NONMONOTONIC
} HYD_ToggleError;

typedef struct {
    HYD_REAL lr, lf, lpf, lpk, ld;
    HYD_REAL hf, hm, dc, sm;
    HYD_REAL xHandoff;
    int8_t sigmaK, signB, tauS;
} HYD_ToggleGeometryConfig;

typedef struct {
    HYD_ToggleGeometryConfig raw;
    HYD_REAL deltaH, aP, bP;
    HYD_REAL lr2, lf2, ld2, invLr;
    HYD_REAL xGeometryMin, xHandoffEffective;
    HYD_REAL xsMin, xsMax, kMin, kMax;
} HYD_TogglePreparedConfig;

typedef struct {
    HYD_REAL xs, velocityRatio, vs;
    HYD_REAL radicandK, radicandS;
    HYD_REAL normalizedMainJacobian;
    HYD_REAL driveProjection;
} HYD_ToggleSolution;

HYD_ToggleGeometryConfig HYD_ToggleKinematics_DefaultConfig(void);
HYD_BOOL HYD_ToggleKinematics_Prepare(
    const HYD_ToggleGeometryConfig *config,
    HYD_TogglePreparedConfig *prepared,
    HYD_ToggleError *error);
HYD_BOOL HYD_ToggleKinematics_SolveOnline(
    const HYD_TogglePreparedConfig *prepared,
    HYD_REAL xm,
    HYD_REAL vm,
    HYD_ToggleSolution *solution,
    HYD_ToggleError *error);

#endif
```

- [ ] **Step 4: Implement preparation and the single-pass online formula**

Create `src/toggle_kinematics.c`. Use `sqrtf`, `fabsf`, and `isfinite`; precompute `aP/bP` once. The hot function must compute `D`, `hK`, and `g` once each, reuse `invD`, `invDeltaJ`, and `invG`, and assign `solution` only after all checks pass. Use a local zeroed result so a failure never exposes partial output.

The final Jacobian calculation must be exactly:

```c
invDeltaJ = 1.0f / deltaJ;
uPrime = (v - raw->hf) * (u - d) * invDeltaJ;
vPrime = -u * (u - d) * invDeltaJ;
pxPrime = (prepared->aP * uPrime - prepared->bP * vPrime) * prepared->invLr;
pyPrime = (prepared->aP * vPrime + prepared->bP * uPrime) * prepared->invLr;
k = pxPrime - (HYD_REAL)raw->tauS * py * pyPrime / g;
```

Use these default values in `HYD_ToggleKinematics_DefaultConfig()`:

```c
const HYD_ToggleGeometryConfig value = {
    150.0f, 230.0f, 135.0f, 75.0f, 60.0f,
    130.0f, 100.0f, 378.0f, 202.0f,
    0.0f, -1, -1, -1
};
```

- [ ] **Step 5: Run the focused test**

Run:

```bash
cmake --build --preset unixgcc --target test_toggle_kinematics
./out/build/unixgcc/test_toggle_kinematics
```

Expected: `toggle kinematics core tests passed`.

- [ ] **Step 6: Commit the mathematical core**

```bash
git add CMakeLists.txt include/toggle_kinematics.h src/toggle_kinematics.c tests/test_toggle_kinematics.c
git commit -m "establish a testable online toggle kinematics core" -m "Constraint: Keep formulas independent from motion and IEC state" -m "Confidence: high" -m "Scope-risk: narrow" -m "Tested: ./out/build/unixgcc/test_toggle_kinematics"
```

### Task 2: Incremental Envelope Validation and Position Inverse

**Files:**
- Modify: `include/toggle_kinematics.h`
- Modify: `src/toggle_kinematics.c`
- Modify: `tests/test_toggle_kinematics.c`

- [ ] **Step 1: Add failing reachability, automatic-boundary, derivative, and round-trip tests**

Add tests that assert:

```c
static void test_validation_and_inverse(void) {
    HYD_ToggleValidation validation;
    HYD_ToggleValidationLimits limits = HYD_ToggleKinematics_DefaultValidationLimits();
    HYD_ToggleGeometryConfig raw = HYD_ToggleKinematics_DefaultConfig();
    HYD_TogglePreparedConfig prepared;
    HYD_ToggleError error;
    HYD_REAL recoveredXm;

    assert(HYD_ToggleKinematics_BeginValidation(&raw, &limits, &validation, &error));
    while (!HYD_ToggleKinematics_ValidationDone(&validation)) {
        assert(HYD_ToggleKinematics_ValidationStep(&validation, 4U, &error));
    }
    assert(HYD_ToggleKinematics_FinishValidation(&validation, &prepared, &error));
    assert(prepared.xGeometryMin >= 0.0f);
    assert(prepared.xGeometryMin <= raw.sm);
    assert(prepared.xHandoffEffective == prepared.xGeometryMin);
    assert(HYD_ToggleKinematics_InversePosition(
        &prepared, -63.808094f, &recoveredXm, &error));
    assert_near(recoveredXm, 101.0f, 1.0e-3f);
}

static void test_unreachable_full_stroke_is_rejected(void) {
    HYD_ToggleGeometryConfig raw = HYD_ToggleKinematics_DefaultConfig();
    HYD_TogglePreparedConfig prepared;
    HYD_ToggleError error;

    raw.dc = 400.0f;
    assert(!HYD_ToggleKinematics_ValidateBlocking(&raw, &prepared, &error));
    assert(error == HYD_TOGGLE_ERROR_MAIN_LINKAGE_UNREACHABLE);
}

static void test_analytic_ratio_matches_center_difference(void) {
    HYD_TogglePreparedConfig prepared;
    HYD_ToggleSolution left, center, right;
    HYD_ToggleError error;
    HYD_REAL numeric;
    const HYD_REAL h = 0.01f;

    assert(HYD_ToggleKinematics_ValidateBlocking(
        &(HYD_ToggleGeometryConfig){150,230,135,75,60,130,100,378,202,0,-1,-1,-1},
        &prepared, &error));
    assert(HYD_ToggleKinematics_SolveOnline(&prepared, 100.0f-h, 1.0f, &left, &error));
    assert(HYD_ToggleKinematics_SolveOnline(&prepared, 100.0f, 1.0f, &center, &error));
    assert(HYD_ToggleKinematics_SolveOnline(&prepared, 100.0f+h, 1.0f, &right, &error));
    numeric = (right.xs - left.xs) / (2.0f * h);
    assert_near(center.velocityRatio, numeric, 2.0e-3f);
}
```

- [ ] **Step 2: Run the test and verify the validation APIs are missing**

Run `cmake --build --preset unixgcc --target test_toggle_kinematics`.

Expected: compilation fails on `HYD_ToggleValidation` and the validation/inverse functions.

- [ ] **Step 3: Define deterministic validation state and limits**

Add to the header:

```c
#define HYD_TOGGLE_VALIDATION_POINTS 257U
#define HYD_TOGGLE_BOUNDARY_ITERATIONS 24U

typedef struct {
    HYD_REAL radicandTolerance;
    HYD_REAL minNormalizedMainJacobian;
    HYD_REAL minDriveProjectionRatio;
    HYD_REAL minAbsVelocityRatio;
    HYD_REAL maxAbsVelocityRatio;
    HYD_REAL geometryMarginMm;
} HYD_ToggleValidationLimits;

typedef enum {
    HYD_TOGGLE_VALIDATION_SCAN,
    HYD_TOGGLE_VALIDATION_REFINE,
    HYD_TOGGLE_VALIDATION_COMPLETE,
    HYD_TOGGLE_VALIDATION_FAILED
} HYD_ToggleValidationPhase;

typedef struct {
    HYD_TogglePreparedConfig candidate;
    HYD_ToggleValidationLimits limits;
    HYD_ToggleValidationPhase phase;
    HYD_UINT16 nextPoint;
    HYD_UINT8 refineIteration;
    HYD_BOOL foundSafeSuffix;
    HYD_REAL unsafeX, safeX;
    HYD_REAL previousXs;
} HYD_ToggleValidation;
```

The default limits must be finite constants defined in one function, not macros duplicated across call sites:

```c
HYD_ToggleValidationLimits HYD_ToggleKinematics_DefaultValidationLimits(void);
HYD_BOOL HYD_ToggleKinematics_BeginValidation(
    const HYD_ToggleGeometryConfig *raw,
    const HYD_ToggleValidationLimits *limits,
    HYD_ToggleValidation *validation,
    HYD_ToggleError *error);
HYD_BOOL HYD_ToggleKinematics_ValidationStep(
    HYD_ToggleValidation *validation,
    HYD_UINT16 maxEvaluations,
    HYD_ToggleError *error);
HYD_BOOL HYD_ToggleKinematics_ValidationDone(const HYD_ToggleValidation *validation);
HYD_BOOL HYD_ToggleKinematics_FinishValidation(
    const HYD_ToggleValidation *validation,
    HYD_TogglePreparedConfig *prepared,
    HYD_ToggleError *error);
HYD_BOOL HYD_ToggleKinematics_ValidateBlocking(
    const HYD_ToggleGeometryConfig *raw,
    HYD_TogglePreparedConfig *prepared,
    HYD_ToggleError *error);
HYD_BOOL HYD_ToggleKinematics_InversePosition(
    const HYD_TogglePreparedConfig *prepared,
    HYD_REAL xs,
    HYD_REAL *xm,
    HYD_ToggleError *error);
```

Use `0.02f` for normalized main Jacobian, `0.02f` for `|px-Xs|/Ld`, `0.01f` for minimum `|k|`, `20.0f` for maximum `|k|`, and `0.5f` for the post-boundary geometry margin. The radicand tolerance must scale from squared link lengths rather than use an unscaled machine-size-independent constant.

- [ ] **Step 4: Implement incremental scan, blocking wrapper, and fixed-iteration bisection inverse**

`ValidationStep(maxPoints)` must process no more than `maxPoints` scan or refinement evaluations. Across the full physical stroke it must reject any negative root below scaled tolerance. It may use the safety thresholds only to derive the contiguous safe suffix and must reject an unsafe point that appears after the suffix has begun.

Resolve `xHandoffEffective` with:

```c
if (candidate.raw.xHandoff == 0.0f) {
    candidate.xHandoffEffective = candidate.xGeometryMin;
} else if (candidate.raw.xHandoff < candidate.xGeometryMin ||
           candidate.raw.xHandoff > candidate.raw.sm) {
    if (error != NULL) {
        *error = HYD_TOGGLE_ERROR_POSITION_OUT_OF_RANGE;
    }
    return false;
} else {
    candidate.xHandoffEffective = candidate.raw.xHandoff;
}
```

`InversePosition` must first check the requested `xs` against the prepared endpoint envelope, then perform 32 bisection iterations over `[xGeometryMin, sm]`. Select the half interval using the validated monotonic direction, not a hardcoded decreasing assumption.

- [ ] **Step 5: Run the mathematical test and full CTest smoke**

Run:

```bash
cmake --build --preset unixgcc --target test_toggle_kinematics
./out/build/unixgcc/test_toggle_kinematics
ctest --test-dir out/build/unixgcc -R "test_toggle_kinematics|test_motion_planner" --output-on-failure
```

Expected: both selected CTest cases pass.

- [ ] **Step 6: Commit validation and inverse support**

```bash
git add include/toggle_kinematics.h src/toggle_kinematics.c tests/test_toggle_kinematics.c
git commit -m "bound toggle motion to a validated geometric envelope" -m "Constraint: Full physical travel must remain reachable while ordinary-speed control may start above the locking region" -m "Rejected: Newton-only inverse | initial-value and singularity sensitivity" -m "Confidence: high" -m "Scope-risk: narrow" -m "Tested: test_toggle_kinematics and test_motion_planner"
```

### Task 3: Static Configuration Pool and Shared Validation Workspace

**Files:**
- Modify: `include/hyd_config.h`
- Create: `include/toggle_mechanism_pool.h`
- Create: `src/toggle_mechanism_pool.c`
- Create: `tests/test_toggle_mechanism_pool.c`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Write failing pool transaction tests**

Cover reset, 20 active slots, one validation workspace, version increments, failed candidate isolation, and release/reuse:

```c
static void test_atomic_commit(void) {
    HYD_UINT8 slot;
    HYD_TogglePreparedConfig before;
    HYD_TogglePreparedConfig after;
    HYD_ToggleGeometryConfig raw = HYD_ToggleKinematics_DefaultConfig();
    HYD_ToggleError error;

    HYD_ToggleMechanismPool_Reset();
    assert(HYD_ToggleMechanismPool_Reserve(3U, &slot));
    assert(HYD_ToggleKinematics_ValidateBlocking(&raw, &after, &error));
    assert(HYD_ToggleMechanismPool_Commit(slot, &after, true));
    before = *HYD_ToggleMechanismPool_GetPrepared(slot);

    raw.dc = 400.0f;
    assert(!HYD_ToggleKinematics_ValidateBlocking(&raw, &after, &error));
    assert(HYD_ToggleMechanismPool_GetPrepared(slot)->raw.dc == before.raw.dc);
    assert(HYD_ToggleMechanismPool_GetVersion(slot) == 1U);
}
```

- [ ] **Step 2: Register and run the new test to see missing-symbol failures**

Add `test_toggle_mechanism_pool` executable/test to `CMakeLists.txt`, build it, and expect missing header/API errors.

- [ ] **Step 3: Add compile-time resource limits**

Append to `include/hyd_config.h` near `HYD_MAX_AXIS_MOTION`:

```c
#ifndef HYD_MAX_TOGGLE_MECHANISMS
#define HYD_MAX_TOGGLE_MECHANISMS HYD_MAX_AXIS_MOTION
#endif

#ifndef HYD_MAX_TOGGLE_VALIDATIONS
#define HYD_MAX_TOGGLE_VALIDATIONS 1
#endif

#ifndef HYD_ENABLE_MECHANISM_TELEMETRY
#define HYD_ENABLE_MECHANISM_TELEMETRY 1
#endif
```

- [ ] **Step 4: Implement slots and validation tokens without heap allocation**

Expose these operations in `toggle_mechanism_pool.h`:

```c
#define HYD_TOGGLE_SLOT_NONE UINT8_MAX
#define HYD_TOGGLE_VALIDATION_NONE UINT8_MAX

void HYD_ToggleMechanismPool_Reset(void);
HYD_BOOL HYD_ToggleMechanismPool_Reserve(HYD_UINT8 ownerAxis, HYD_UINT8 *slot);
void HYD_ToggleMechanismPool_Release(HYD_UINT8 slot);
HYD_BOOL HYD_ToggleMechanismPool_Commit(
    HYD_UINT8 slot,
    const HYD_TogglePreparedConfig *prepared,
    HYD_BOOL usingDefaults);
const HYD_TogglePreparedConfig *HYD_ToggleMechanismPool_GetPrepared(HYD_UINT8 slot);
const HYD_ToggleGeometryConfig *HYD_ToggleMechanismPool_GetRaw(HYD_UINT8 slot);
HYD_UINT16 HYD_ToggleMechanismPool_GetVersion(HYD_UINT8 slot);
HYD_BOOL HYD_ToggleMechanismPool_UsingDefaults(HYD_UINT8 slot);
size_t HYD_ToggleMechanismPool_SlotSize(void);
HYD_BOOL HYD_ToggleMechanismPool_AcquireValidation(HYD_UINT8 *token);
HYD_ToggleValidation *HYD_ToggleMechanismPool_GetValidation(HYD_UINT8 token);
void HYD_ToggleMechanismPool_ReleaseValidation(HYD_UINT8 token);
```

`HYD_TogglePreparedConfig` already contains the committed raw configuration. The active slot must store only one prepared object, not a duplicate raw object. `Commit` must copy the prepared value into a temporary, update the active slot, and increment version last. `GetRaw` returns `&slot.prepared.raw`. Validation workspaces are independent from active slots so a rejected candidate cannot overwrite the running configuration.

- [ ] **Step 5: Run pool and kinematics tests**

Run:

```bash
cmake --build --preset unixgcc --target test_toggle_mechanism_pool test_toggle_kinematics
ctest --test-dir out/build/unixgcc -R "test_toggle_(mechanism_pool|kinematics)" --output-on-failure
```

Expected: both tests pass.

- [ ] **Step 6: Commit the bounded resource layer**

```bash
git add CMakeLists.txt include/hyd_config.h include/toggle_mechanism_pool.h src/toggle_mechanism_pool.c tests/test_toggle_mechanism_pool.c
git commit -m "isolate toggle geometry in bounded static resources" -m "Constraint: Support twenty configured axes without heap allocation while serializing rare validation work" -m "Confidence: high" -m "Scope-risk: narrow" -m "Tested: test_toggle_mechanism_pool and test_toggle_kinematics"
```

### Task 4: Actuation Mapper

**Files:**
- Modify: `include/common_types.h`
- Create: `include/actuation_mapper.h`
- Create: `src/actuation_mapper.c`
- Create: `tests/test_actuation_mapper.c`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Write failing direct/toggle mapping tests**

Test direct identity, default toggle at `Xm=101`, actuator-direction inversion, direction-specific area, fallback gain, and max-flow limiting. With `areaRetractMm2=1000`, `Vm=10`, the expected toggle flow is approximately `0.485028 L/min`.

```c
assert(HYD_ActuationMapper_MapVelocity(&input, &output, &error));
assert(fabsf(output.actuatorVelocity + 8.083796f) < 2.0e-3f);
assert(output.actuatorDirection == HYD_DIRECTION_RETRACT);
assert(fabsf(output.requestedFlow - 0.485028f) < 2.0e-4f);
```

- [ ] **Step 2: Add mechanism enum without renumbering existing public enums**

Add to `common_types.h` before motion state declarations:

```c
typedef enum {
    HYD_MECHANISM_DIRECT = 0,
    HYD_MECHANISM_FIVE_POINT_TOGGLE = 1
} HYD_MechanismType;
```

- [ ] **Step 3: Define one mapping boundary for all velocity-to-flow paths**

Create `actuation_mapper.h`:

```c
typedef struct {
    HYD_MechanismType mechanismType;
    const HYD_TogglePreparedConfig *toggleConfig;
    HYD_REAL templatePosition;
    HYD_REAL templateVelocity;
    const HYD_CylinderConfig *cylinderConfig;
    HYD_REAL fallbackCylinderVelocityToFlowGain;
    HYD_REAL maxFlow;
} HYD_ActuationMapperInput;

typedef struct {
    HYD_REAL actuatorPosition;
    HYD_REAL velocityRatio;
    HYD_REAL actuatorVelocity;
    HYD_REAL effectiveCylinderGain;
    HYD_REAL requestedFlow;
    HYD_MotionDirection actuatorDirection;
} HYD_ActuationMapperOutput;

HYD_BOOL HYD_ActuationMapper_MapVelocity(
    const HYD_ActuationMapperInput *input,
    HYD_ActuationMapperOutput *output,
    HYD_ToggleError *error);
HYD_BOOL HYD_ActuationMapper_FlowToTemplateVelocity(
    const HYD_ActuationMapperInput *input,
    HYD_REAL actuatorFlow,
    HYD_REAL *templateVelocity,
    HYD_ToggleError *error);
```

- [ ] **Step 4: Implement direct identity and toggle conversion**

For direct axes use `actuatorPosition=templatePosition`, `velocityRatio=1`, and `actuatorVelocity=templateVelocity`. For toggle axes call `SolveOnline`. Resolve the cylinder gain from the actuator direction; only fall back to the supplied legacy gain when the corresponding configured area is unavailable. Clamp the final magnitude to `maxFlow` only when `maxFlow>0`.

`FlowToTemplateVelocity` must divide by `effectiveCylinderGain*abs(velocityRatio)` and reject gains at or below the validation limit; it must preserve the caller-provided sign separately from magnitude.

- [ ] **Step 5: Run focused tests**

Run:

```bash
cmake --build --preset unixgcc --target test_actuation_mapper
./out/build/unixgcc/test_actuation_mapper
```

Expected: all direct and toggle mapping assertions pass.

- [ ] **Step 6: Commit the mapping boundary**

```bash
git add CMakeLists.txt include/common_types.h include/actuation_mapper.h src/actuation_mapper.c tests/test_actuation_mapper.c
git commit -m "separate platen planning from actuator conversion" -m "Constraint: Pressure flow bypasses this velocity mapping and direct axes retain identity behavior" -m "Confidence: high" -m "Scope-risk: moderate" -m "Tested: test_actuation_mapper"
```

### Task 5: Transactional CreateMotion Binding

**Files:**
- Modify: `include/common_types.h`
- Modify: `include/motion_control.h`
- Modify: `include/motion_interface.h`
- Modify: `src/motion_interface.c`
- Create: `tests/test_toggle_motion_interface.c`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Add failing legacy/direct/toggle/pool-failure creation tests**

The test must prove:

- zero-initialized `MECHANISM_TYPE` creates direct axis 0 immediately;
- direct axis has `mechanismSlot=HYD_TOGGLE_SLOT_NONE`;
- toggle create holds `BUSY` across scans and completes with default `dc=378`;
- invalid mechanism type returns an error;
- pool exhaustion does not consume an axis ID;
- framework re-init resets both pools and allocation state.

- [ ] **Step 2: Append mechanism diagnostics before their first use**

Append these codes at the end of `HYD_DiagnosticCode` so existing numeric values remain stable:

```c
HYD_DIAG_CODE_MECHANISM_TYPE_INVALID,
HYD_DIAG_CODE_MECHANISM_POOL_EXHAUSTED,
HYD_DIAG_CODE_MECHANISM_VALIDATION_BUSY,
HYD_DIAG_CODE_MECHANISM_CONFIG_BUSY,
HYD_DIAG_CODE_MECHANISM_CONFIG_INVALID,
HYD_DIAG_CODE_KINEMATICS_POSITION_OUT_OF_RANGE,
HYD_DIAG_CODE_KINEMATICS_RUNTIME_INVALID
```

- [ ] **Step 3: Add compact binding fields to the core FB**

Place adjacent to existing identity fields in `HYD_MotionControlFB`:

```c
HYD_UINT8 mechanismType;
HYD_UINT8 mechanismSlot;
```

Initialize direct/none in `HYD_MotionControlFB_Init()`.

- [ ] **Step 4: Extend CreateMotion without breaking zero initialization**

Append `MECHANISM_TYPE` to the public input group and add private transaction fields:

```c
__DECLARE_VAR(SINT,MECHANISM_TYPE)
__DECLARE_VAR(SINT,_RESERVED_AXIS)
__DECLARE_VAR(USINT,_RESERVED_SLOT)
__DECLARE_VAR(USINT,_VALIDATION_TOKEN)
__DECLARE_VAR(BOOL,_CREATE_ACTIVE)
```

Initialize private IDs to `-1` or `255` in the corresponding generated-equivalent initializer later in Task 7.

- [ ] **Step 5: Replace monotonic allocation assumptions with slot states**

In `motion_interface.c`, introduce:

```c
typedef enum {
    HYD_AXIS_SLOT_FREE = 0,
    HYD_AXIS_SLOT_RESERVED,
    HYD_AXIS_SLOT_ACTIVE
} HYD_AxisSlotState;

static HYD_AxisSlotState HYD_AxisSlots[HYD_MAX_AXIS_MOTION];
```

`__MK_GetPublic_MotionControlFB(index)` returns only ACTIVE slots. Framework Publish and pump arbitration iterate `0..HYD_MAX_AXIS_MOTION-1` and skip non-ACTIVE slots. Keep axis IDs deterministic by reserving the lowest free index.

- [ ] **Step 6: Implement CreateMotion reserve/validate/commit/rollback**

Direct create reserves and commits in one call. Toggle create reserves an axis, mechanism slot, and validation workspace; starts default validation; advances at most four validation evaluations per `__mcl_cmd_CreateMotion` call; and commits only after validation succeeds.

Every failure path must call one rollback helper that releases the validation token, mechanism slot, and axis reservation, then clears FB private transaction fields. `ERRORID` must distinguish invalid mechanism type, pool exhaustion, validation-workspace exhaustion, and invalid default geometry.

- [ ] **Step 7: Run creation and legacy interface tests**

Run:

```bash
cmake --build --preset unixgcc --target test_toggle_motion_interface test_motion_interface_unit
ctest --test-dir out/build/unixgcc -R "test_toggle_motion_interface|test_motion_interface_unit" --output-on-failure
```

Expected: both tests pass; legacy direct creation still returns axis 0.

- [ ] **Step 8: Commit transactional creation**

```bash
git add CMakeLists.txt include/common_types.h include/motion_control.h include/motion_interface.h src/motion_interface.c tests/test_toggle_motion_interface.c
git commit -m "make mechanism-aware axis creation transactional" -m "Constraint: A rejected toggle configuration must not consume either an axis or geometry slot" -m "Confidence: high" -m "Scope-risk: moderate" -m "Tested: test_toggle_motion_interface and test_motion_interface_unit"
```

### Task 6: Atomic IEC Configuration and Readback

**Files:**
- Modify: `include/motion_interface.h`
- Modify: `src/motion_interface.c`
- Modify: `tests/test_toggle_motion_interface.c`

- [ ] **Step 1: Add failing configure/readback state-machine tests**

Test a valid update from `dc=378` to `377.5`, input mutation while BUSY, failed `dc=400` rollback, version stability after failure, explicit `xHandoff<xGeometryMin` rejection, `xHandoff=0` auto mode, and active-axis rejection.

- [ ] **Step 2: Define dedicated IEC FB layouts**

Add `HYD_CONFIGURETOGGLEMECHANISM` with all raw inputs, standard `DONE/BUSY/ERROR/ERRORID`, `CONFIG_VERSION`, and private `EXECUTE0/VALIDATION_TOKEN/ACTIVE`. Add `HYD_READTOGGLEMECHANISM` with `ENABLE`, raw fields, `X_GEOMETRY_MIN`, `X_HANDOFF_EFFECTIVE`, `XS_MIN/XS_MAX/K_MIN/K_MAX`, `VALID`, `USING_DEFAULTS`, version, and errors.

Use `LREAL` for configuration/readback fields so PLC values are not truncated before explicit conversion to `HYD_REAL`; store only `HYD_REAL` in the embedded pool.

- [ ] **Step 3: Implement configure as an incremental atomic transaction**

On rising edge:

1. resolve ACTIVE axis and require toggle mechanism;
2. require `IDLE/READY/DONE/ABORTED` and `STATE.active=false`;
3. acquire validation workspace;
4. copy every IEC input into the workspace candidate immediately;
5. begin validation.

On subsequent calls with `EXECUTE=true`, advance at most four evaluations. On success commit to the existing slot and return the new version. On error release only the workspace and keep the old slot untouched. On `EXECUTE=false`, clear terminal outputs and the edge latch.

- [ ] **Step 4: Implement readback from committed state only**

When ENABLE and axis/slot are valid, populate every raw and envelope output from `GetRaw/GetPrepared`, set `VALID=true`, and return the committed version. During a concurrent candidate validation, the reader must continue returning the previous committed version.

- [ ] **Step 5: Run IEC transaction tests**

Run `./out/build/unixgcc/test_toggle_motion_interface` after rebuilding.

Expected: valid updates increment versions once, invalid updates preserve prior values, and input mutations during BUSY do not alter the latched candidate.

- [ ] **Step 6: Commit the IEC transaction API**

```bash
git add include/motion_interface.h src/motion_interface.c tests/test_toggle_motion_interface.c
git commit -m "expose toggle geometry as an atomic IEC transaction" -m "Constraint: Geometry fields cannot become visible one at a time" -m "Rejected: Extend generic WriteParameter | intermediate combinations are unsafe" -m "Confidence: high" -m "Scope-risk: moderate" -m "Tested: test_toggle_motion_interface"
```

### Task 7: XML, Generated-Equivalent Layouts, and Status Surface

**Files:**
- Modify: `pousHydMotion.xml`
- Modify: `include/common_types.h`
- Modify: `include/motion_interface.h`
- Modify: `src/motion_interface.c`
- Modify: `tests/plcdemo/POUS.h`
- Modify: `tests/plcdemo/POUS.c`
- Modify: `tests/test_interface_layout_consistency.py`
- Modify: `tests/test_parameter_iec.c`

- [ ] **Step 1: Add failing layout and status assertions**

Extend the Python test to require `MECHANISM_TYPE` in CreateMotion and both new POU names in XML/header. Require generated-equivalent initializer blocks to initialize `MECHANISM_TYPE`, validation tokens, and every configure/read FB terminal output.

Extend `test_parameter_iec.c` to assert ReadStatus exposes:

```text
MECHANISMTYPE
ACTUATORDIRECTION
ACTUATORPOSITION
ACTUATORVELOCITYCOMMAND
VELOCITYRATIO
MECHANISMCONFIGVERSION
```

- [ ] **Step 2: Update XML and header in lockstep**

Add the CreateMotion input and complete `HYD_ConfigureToggleMechanism`/`HYD_ReadToggleMechanism` POU definitions to `pousHydMotion.xml`. Add matching structs/prototypes to `motion_interface.h`. Append the ReadStatus outputs without reordering existing fields.

- [ ] **Step 3: Update generated-equivalent PLC demo artifacts**

Mirror the layouts in `tests/plcdemo/POUS.h`. In `POUS.c`, initialize every new field with `__INIT_VAR`, add wrapper bodies that call `__mcl_cmd_ConfigureToggleMechanism` and `__mcl_cmd_ReadToggleMechanism`, and initialize CreateMotion private IDs to their none sentinels.

- [ ] **Step 4: Publish mechanism status**

Add to `HYD_MotionState`:

```c
HYD_UINT8 mechanismType;
HYD_MotionDirection actuatorDirection;
HYD_UINT16 mechanismConfigVersion;
#if HYD_ENABLE_MECHANISM_TELEMETRY
HYD_REAL actuatorPosition;
HYD_REAL actuatorVelocityCommand;
HYD_REAL velocityRatio;
#endif
```

`__mcl_cmd_ReadStatus` must clear every new output on invalid axis or disabled read, and publish the committed state when enabled.

- [ ] **Step 5: Run layout, IEC, and PLC demo build checks**

Run:

```bash
cmake --build --preset unixgcc --target test_parameter_iec
ctest --test-dir out/build/unixgcc -R "test_interface_layout_consistency|test_parameter_iec" --output-on-failure
cmake --build --preset unixgcc
```

Expected: layout consistency and IEC parameter tests pass; the full build compiles generated-equivalent demo sources.

- [ ] **Step 6: Commit the public IEC surface**

```bash
git add pousHydMotion.xml include/common_types.h include/motion_interface.h src/motion_interface.c tests/plcdemo/POUS.h tests/plcdemo/POUS.c tests/test_interface_layout_consistency.py tests/test_parameter_iec.c
git commit -m "keep the IEC mechanism contract synchronized end to end" -m "Constraint: XML, C layouts, generated-equivalent demo code, and status outputs must agree" -m "Confidence: high" -m "Scope-risk: moderate" -m "Tested: layout consistency, test_parameter_iec, and full build"
```

### Task 8: Main Position/Velocity Control Integration

**Files:**
- Modify: `include/motion_control.h`
- Modify: `src/motion_control.c`
- Create: `tests/test_toggle_motion_integration.c`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Lock direct behavior and add failing toggle feedforward tests**

Capture direct-axis planner/flow/pump outputs at representative MoveAbsolute and MoveVelocity cycles before integration. Add toggle cases that expect the same `plannedVelocity` in platen coordinates but a position-dependent actuator velocity and flow. Add a pressure-mode case asserting the pressure controller's `outputFlow` is unchanged by `k`.

- [ ] **Step 2: Add one shared mapper helper inside motion control**

Declare in `motion_control.h` for focused tests:

```c
HYD_BOOL HYD_MotionControlFB_MapTemplateVelocity(
    HYD_MotionControlFB *fb,
    const HYD_MotionSegment *segment,
    HYD_REAL templateVelocity,
    HYD_REAL maxFlow,
    HYD_ActuationMapperOutput *mapped,
    HYD_DiagnosticCode *code);
```

The helper resolves the prepared slot, populates `HYD_ActuationMapperInput`, maps errors to diagnostics, and never mutates the active configuration.

- [ ] **Step 3: Insert mapping after velocity planning and before velocity correction**

In the non-pressure branch of `HYD_ExecuteActiveSegmentControl`:

1. run planner to produce `targetVelocity`;
2. map `targetVelocity` to dynamic feedforward flow;
3. set `plannerOutput->targetFlow` to mapped flow;
4. update `_plannerState.lastTargetFlow` to that mapped value;
5. use mapped flow as velocity-controller feedforward;
6. apply existing pump and output limits.

For direct axes, the helper must reproduce the current fixed-gain formula and cap order. For pressure mode, do not call the helper.

- [ ] **Step 4: Persist actuator status without changing platen references**

After a successful map, set actuator direction, position, command, ratio, mechanism type, and slot version in `STATE`. Keep `executionReference.velocityReference=plannerOutput.targetVelocity`, `AXIS_REF`, completion checks, and velocity diagnostics in platen coordinates.

- [ ] **Step 5: Run focused integration and direct regression tests**

Run:

```bash
cmake --build --preset unixgcc --target test_toggle_motion_integration test_direct_mode test_motion_planner
ctest --test-dir out/build/unixgcc -R "test_toggle_motion_integration|test_direct_mode$|test_motion_planner" --output-on-failure
```

Expected: toggle flow changes with position, pressure flow bypasses mapping, and existing direct tests pass unchanged.

- [ ] **Step 6: Commit the main runtime integration**

```bash
git add CMakeLists.txt include/motion_control.h src/motion_control.c tests/test_toggle_motion_integration.c
git commit -m "apply toggle kinematics at the hydraulic actuation boundary" -m "Constraint: Planner and diagnostics remain in platen coordinates while pressure output remains actuator-side flow" -m "Confidence: high" -m "Scope-risk: broad" -m "Tested: toggle integration, direct mode, and motion planner tests"
```

### Task 9: Special Paths, Runtime Faults, and Pump Direction Arbitration

**Files:**
- Modify: `src/motion_control.c`
- Modify: `src/motion_interface.c`
- Modify: `src/diagnostics.c`
- Modify: `tests/test_toggle_motion_integration.c`
- Modify: `tests/test_stop_immediate_done.c`
- Modify: `tests/test_moveabsolute_blending_done.c`
- Modify: `tests/test_pump_direction_conflict.c`

- [ ] **Step 1: Add failing tests for every fixed-gain escape path**

Add toggle cases for:

- Stop deceleration at two positions;
- P-to-V carryover using current dynamic gain;
- MoveContinuousAbsolute approach-to-sustain transition;
- buffered/blended transition;
- Hold/Resume preserving platen state;
- out-of-range and runtime-root failure producing same-cycle zero flow/pump speed;
- pump direction conflict using actuator direction rather than platen direction;
- simulation integrating `plannerOutput.targetVelocity`, not actuator velocity.

- [ ] **Step 2: Register mechanism diagnostics and recovery actions**

Add diagnostic specs/messages in `diagnostics.c` for the codes appended in Task 5. Configuration errors use `HYD_DIAG_RECOVERY_CHECK_COMMAND`; runtime kinematics errors use `HYD_DIAG_RECOVERY_RESET_CONTROLLER` and `HYD_PROTECTION_ACTION_STOP`.

- [ ] **Step 3: Replace remaining fixed-gain conversions**

Use `HYD_MotionControlFB_MapTemplateVelocity` or `HYD_ActuationMapper_FlowToTemplateVelocity` in:

- `HYD_PrimeSegmentControllers` P-to-V seeding;
- `HYD_RunRunningStateStopping` flow generation;
- continuous absolute phase transitions;
- blend carryover state;
- simulation target-flow publication.

Do not replace pressure-controller flow math.

- [ ] **Step 4: Make kinematic failures produce immediate safe state**

Change the internal active-segment control helper to return `HYD_BOOL`. On mapping failure, zero local planner/pump outputs, call `HYD_StateReporter_ApplySafeOutputs`, report the mapped diagnostic with `HYD_StateReporter_ReportFault`, and return before OutputLimiter or StateReporter can republish stale values.

Record mechanism config version and current `Xm` in the existing fault snapshot fields; add a compact mechanism error field only if the diagnostic code cannot preserve the cause.

- [ ] **Step 5: Use actuator direction for shared pump conflict detection**

In `__mcl_cmd_GetPumpRequest`, use `STATE.actuatorDirection` for active toggle axes and preserve `STATE.plannedDirection` for direct axes. Pump-speed magnitude arbitration remains unchanged.

- [ ] **Step 6: Run all affected integration tests**

Run:

```bash
cmake --build --preset unixgcc
ctest --test-dir out/build/unixgcc -R "test_toggle_motion_integration|test_stop_immediate_done|test_moveabsolute_blending_done|test_movecontinuousabsolute_integration|test_pump_direction_conflict|test_fault_recovery" --output-on-failure
```

Expected: all selected tests pass with no stale output after injected kinematic faults.

- [ ] **Step 7: Commit special-path and fault coverage**

```bash
git add src/motion_control.c src/motion_interface.c src/diagnostics.c tests/test_toggle_motion_integration.c tests/test_stop_immediate_done.c tests/test_moveabsolute_blending_done.c tests/test_pump_direction_conflict.c
git commit -m "close toggle kinematics across transitions and faults" -m "Constraint: No fixed-gain path may bypass position-dependent mapping and no failed solve may reuse stale actuation" -m "Confidence: high" -m "Scope-risk: broad" -m "Tested: Stop, blending, continuous motion, pump conflict, and fault recovery integration tests"
```

### Task 10: Resource Evidence, PC Performance, and Full Verification

**Files:**
- Modify: `tests/benchmark_performance.c`
- Modify: `tests/test_toggle_mechanism_pool.c`
- Modify: `README.md`

- [ ] **Step 1: Add failing resource-budget assertions**

In `test_toggle_mechanism_pool.c`, print and bound the actual structures:

```c
printf("toggle slot bytes=%zu validation bytes=%zu motion fb bytes=%zu\n",
       HYD_ToggleMechanismPool_SlotSize(),
       sizeof(HYD_ToggleValidation),
       sizeof(HYD_MotionControlFB));
assert(HYD_ToggleMechanismPool_SlotSize() <= 112U);
assert(sizeof(HYD_ToggleValidation) <= 160U);
assert(sizeof(HYD_MotionControlFB) <= 3176U + 32U);
```

The recorded pre-feature host-build baseline is `3176 B`. Keep that value in the test as `HYD_BASELINE_MOTION_FB_BYTES` with a comment naming commit `f8bc1b9`; assert that growth is at most 32 bytes and consists only of the approved binding/status fields rather than an embedded geometry object. This is a host-ABI regression check, not a target ABI invariant.

- [ ] **Step 2: Replace benchmark timing with monotonic nanoseconds and checksum consumption**

Use `clock_gettime(clockId, &timestamp)` with `clockId=CLOCK_MONOTONIC_RAW` on Linux and `CLOCK_MONOTONIC` fallback. Add:

```c
static volatile HYD_REAL benchmark_checksum;

static uint64_t elapsed_ns(struct timespec start, struct timespec end) {
    return (uint64_t)(end.tv_sec - start.tv_sec) * 1000000000ULL +
           (uint64_t)(end.tv_nsec - start.tv_nsec);
}
```

Every benchmark loop must add one output field to `benchmark_checksum` so the compiler cannot delete the call.

- [ ] **Step 3: Add kinematics and complete-toggle-cycle benchmarks**

Run at least `100,000` warmed iterations. Sweep `Xm` over the validated range rather than benchmark one easy point. Print `mean ns/call`, observed maximum batch time, direct full-cycle time, toggle full-cycle time, and the incremental percentage.

Label the output exactly:

```text
PC regression evidence only; not STM32 WCET.
Target estimate basis: Cortex-M7F 480 MHz, -Os, 480000 cycles per 1 ms.
```

- [ ] **Step 4: Run benchmark and capture the evidence in README**

Run:

```bash
cmake --build --preset unixgcc --target benchmark_performance
./out/build/unixgcc/benchmark_performance
```

Add a short README subsection with the command, current PC compiler/build mode, slot sizes, and the explicit target-board validation gap. Do not convert PC nanoseconds to a claimed STM32 WCET.

- [ ] **Step 5: Run complete verification**

Run:

```bash
cmake --preset unixgcc
cmake --build --preset unixgcc
ctest --test-dir out/build/unixgcc --output-on-failure
python3 tests/test_interface_layout_consistency.py
git diff --check
```

Expected: full build succeeds, all CTest tests pass, layout check prints `interface layout consistency tests passed`, and `git diff --check` is silent.

- [ ] **Step 6: Review the final diff against the design acceptance list**

Verify with focused searches:

```bash
rg -n "velocityToFlowGain|targetFlow|lastTargetFlow" src/motion_control.c src/motion_planner.c
rg -n "HYD_MECHANISM_|KINEMATICS_|TOGGLE_" include src tests
git diff --stat HEAD~10..HEAD
```

Confirm every remaining fixed-gain use is either direct-axis compatibility, cylinder-side conversion, or pressure mode. Confirm no geometry config was added to `HYD_MotionSegment`, `HYD_AxisRef`, or `HYD_MotionFBParams`.

- [ ] **Step 7: Commit performance and documentation evidence**

```bash
git add tests/benchmark_performance.c tests/test_toggle_mechanism_pool.c README.md
git commit -m "make toggle resource and timing costs observable" -m "Constraint: PC timing is regression evidence and cannot substitute for target-board DWT measurement" -m "Confidence: high" -m "Scope-risk: narrow" -m "Directive: Measure production -Os firmware on STM32H743/H750 before claiming 1 ms compliance" -m "Tested: full build, full CTest, layout check, resource assertions, and PC benchmark" -m "Not-tested: STM32 cross-compile and DWT CYCCNT timing"
```

## Final Stop Condition

Stop implementation only when:

- all ten task commits exist and follow Lore trailers;
- the full default geometry uses `dc=378 mm` and validates across physical travel;
- automatic `xGeometryMin` and `xHandoff` rules are exercised by tests;
- direct-axis regression tests are unchanged and passing;
- all position/velocity/Stop/blending/carryover paths use the shared mapper;
- pressure flow bypass is covered;
- runtime solve failure publishes safe zero in the same scan;
- XML, header, PLC demo, and layout checker agree;
- resource sizes and PC regression timing are reported;
- STM32 1 ms compliance remains explicitly unverified until target DWT evidence exists.
