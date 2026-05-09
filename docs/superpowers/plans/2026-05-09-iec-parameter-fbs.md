# IEC Parameter Access FBs Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add 4 IEC function blocks for reading/writing MotionControlFB parameters by integer ID, and fix hardcoded literals in segment builders to use FB-stored defaults.

**Architecture:** A `HYD_MotionFBParams` struct groups 25 tunable parameters and is stored as a single `_params` field on `HYD_MotionControlFB`. A `HYD_ParameterNumber` enum maps integer IDs to struct members. Four accessor functions in `motion_control.c` switch on the enum. The IEC layer exposes these through 4 new FBs. Segment builders read from `fb->_params` instead of hardcoded literals.

**Tech Stack:** C99, matiec IEC type system, CMake

---

### Task 1: Add HYD_ParameterNumber enum and HYD_MotionFBParams struct

**Files:**
- Modify: `include/common_types.h` (append before `#endif`)

- [ ] **Step 1: Add enum and struct to common_types.h**

Add before `#endif /* HYD_COMMON_TYPES_H */`:

```c
/* ============================================================================
 * FB Parameter Access — PARAMETERNUMBER → field mapping
 * Used by HYD_ReadParameter / HYD_WriteParameter / HYD_ReadBoolParameter / HYD_WriteBoolParameter
 * ============================================================================ */
typedef enum {
    HYD_PARAM_POSITION_TOLERANCE = 0,
    HYD_PARAM_VELOCITY_TOLERANCE,
    HYD_PARAM_FLOW_TOLERANCE,
    HYD_PARAM_PRESSURE_TOLERANCE,
    HYD_PARAM_TIMEOUT_LIMIT,
    HYD_PARAM_VELOCITY_TO_FLOW_GAIN,
    HYD_PARAM_MAX_VELOCITY,
    HYD_PARAM_MAX_ACCELERATION,
    HYD_PARAM_MAX_DECELERATION,
    HYD_PARAM_MAX_FLOW,
    HYD_PARAM_PRESSURE_RAMP_RATE,
    HYD_PARAM_PRESSURE_KP,
    HYD_PARAM_PRESSURE_KP_HIGH,
    HYD_PARAM_PRESSURE_GAIN_BAND,
    HYD_PARAM_PRESSURE_KI,
    HYD_PARAM_PRESSURE_KD,
    HYD_PARAM_PRESSURE_INTEGRAL_LIMIT,
    HYD_PARAM_PRESSURE_DEADBAND,
    HYD_PARAM_PRESSURE_FILTER_ALPHA,
    HYD_PARAM_PRESSURE_DERIVATIVE_FILTER_ALPHA,
    HYD_PARAM_FLOW_TO_PUMP_SPEED_GAIN,
    HYD_PARAM_PUMP_SPEED_LIMIT,
    HYD_PARAM_PRESSURE_CONTROLLER_TYPE,
    HYD_PARAM_DEFAULT_TARGET_FLOW,
    HYD_PARAM_USE_SIMULATION,
    HYD_PARAM_COUNT
} HYD_ParameterNumber;

typedef struct {
    HYD_REAL positionTolerance;
    HYD_REAL velocityTolerance;
    HYD_REAL flowTolerance;
    HYD_REAL pressureTolerance;
    HYD_REAL timeoutLimit;
    HYD_REAL velocityToFlowGain;
    HYD_REAL maxVelocity;
    HYD_REAL maxAcceleration;
    HYD_REAL maxDeceleration;
    HYD_REAL maxFlow;
    HYD_REAL pressureRampRate;
    HYD_REAL pressureKp;
    HYD_REAL pressureKpHigh;
    HYD_REAL pressureGainBand;
    HYD_REAL pressureKi;
    HYD_REAL pressureKd;
    HYD_REAL pressureIntegralLimit;
    HYD_REAL pressureDeadband;
    HYD_REAL pressureFilterAlpha;
    HYD_REAL pressureDerivativeFilterAlpha;
    HYD_REAL flowToPumpSpeedGain;
    HYD_REAL pumpSpeedLimit;
    HYD_REAL pressureControllerType;
    HYD_REAL defaultTargetFlow;
    HYD_BOOL useSimulation;
} HYD_MotionFBParams;
```

- [ ] **Step 2: Build to verify compilation**

```bash
cmake --build --preset unixgcc
```

Expected: builds successfully (no consumers yet, just type definitions).

---

### Task 2: Add _params field to HYD_MotionControlFB and declare accessor functions

**Files:**
- Modify: `include/motion_control.h`

- [ ] **Step 1: Add _params field to the struct**

Add after the `_useSimulation` field (around line 205 in `HYD_MotionControlFB`):

```c
    HYD_MotionFBParams _params;          /* Tunable parameter defaults for segment builders */
```

- [ ] **Step 2: Declare 4 accessor functions**

Add before `#endif /* HYD_MOTION_CONTROL_H */`:

```c
/* Parameter accessors for IEC Read/Write Parameter FBs.
 * Return false on out-of-range paramNumber or type mismatch. */
HYD_BOOL HYD_MotionControlFB_ReadParameter(const HYD_MotionControlFB* fb, int paramNumber, HYD_REAL* value);
HYD_BOOL HYD_MotionControlFB_WriteParameter(HYD_MotionControlFB* fb, int paramNumber, HYD_REAL value);
HYD_BOOL HYD_MotionControlFB_ReadBoolParameter(const HYD_MotionControlFB* fb, int paramNumber, HYD_BOOL* value);
HYD_BOOL HYD_MotionControlFB_WriteBoolParameter(HYD_MotionControlFB* fb, int paramNumber, HYD_BOOL value);
```

- [ ] **Step 3: Build to verify compilation**

```bash
cmake --build --preset unixgcc
```

Expected: builds successfully.

---

### Task 3: Implement Init defaults and 4 accessor functions

**Files:**
- Modify: `src/motion_control.c`

- [ ] **Step 1: Add Init defaults after memset in HYD_MotionControlFB_Init**

Find `HYD_MotionControlFB_Init` and add after the memset:

```c
    /* Set parameter defaults (matching previous hardcoded values in motion_interface.c) */
    fb->_params.positionTolerance = 0.1f;
    fb->_params.velocityTolerance = 5.0f;
    fb->_params.flowTolerance = 1.0f;
    fb->_params.pressureTolerance = 0.5f;
    fb->_params.timeoutLimit = 30.0f;
    fb->_params.velocityToFlowGain = 0.2f;
    fb->_params.maxVelocity = 100.0f;
    fb->_params.maxAcceleration = 500.0f;
    fb->_params.maxDeceleration = 500.0f;
    fb->_params.maxFlow = 50.0f;
    fb->_params.pressureRampRate = 10.0f;
    fb->_params.pressureKp = 0.5f;
    fb->_params.pressureKpHigh = 0.0f;
    fb->_params.pressureGainBand = 0.2f;
    fb->_params.pressureKi = 0.1f;
    fb->_params.pressureKd = 0.0f;
    fb->_params.pressureIntegralLimit = 10.0f;
    fb->_params.pressureDeadband = 0.5f;
    fb->_params.pressureFilterAlpha = 0.5f;
    fb->_params.pressureDerivativeFilterAlpha = 0.5f;
    fb->_params.flowToPumpSpeedGain = 20.0f;
    fb->_params.pumpSpeedLimit = 1800.0f;
    fb->_params.pressureControllerType = (HYD_REAL)HYD_PRESSURE_CONTROLLER_PI;
    fb->_params.defaultTargetFlow = 5.0f;
    fb->_params.useSimulation = false;
```

- [ ] **Step 2: Implement HYD_MotionControlFB_ReadParameter**

```c
HYD_BOOL HYD_MotionControlFB_ReadParameter(const HYD_MotionControlFB* fb, int paramNumber, HYD_REAL* value)
{
    if (fb == NULL || value == NULL) return false;
    if (paramNumber < 0 || paramNumber >= HYD_PARAM_COUNT) return false;
    if (paramNumber == HYD_PARAM_USE_SIMULATION) return false; /* BOOL param, use ReadBoolParameter */

    switch ((HYD_ParameterNumber)paramNumber) {
        case HYD_PARAM_POSITION_TOLERANCE:             *value = fb->_params.positionTolerance; break;
        case HYD_PARAM_VELOCITY_TOLERANCE:             *value = fb->_params.velocityTolerance; break;
        case HYD_PARAM_FLOW_TOLERANCE:                 *value = fb->_params.flowTolerance; break;
        case HYD_PARAM_PRESSURE_TOLERANCE:             *value = fb->_params.pressureTolerance; break;
        case HYD_PARAM_TIMEOUT_LIMIT:                  *value = fb->_params.timeoutLimit; break;
        case HYD_PARAM_VELOCITY_TO_FLOW_GAIN:          *value = fb->_params.velocityToFlowGain; break;
        case HYD_PARAM_MAX_VELOCITY:                   *value = fb->_params.maxVelocity; break;
        case HYD_PARAM_MAX_ACCELERATION:               *value = fb->_params.maxAcceleration; break;
        case HYD_PARAM_MAX_DECELERATION:               *value = fb->_params.maxDeceleration; break;
        case HYD_PARAM_MAX_FLOW:                       *value = fb->_params.maxFlow; break;
        case HYD_PARAM_PRESSURE_RAMP_RATE:             *value = fb->_params.pressureRampRate; break;
        case HYD_PARAM_PRESSURE_KP:                    *value = fb->_params.pressureKp; break;
        case HYD_PARAM_PRESSURE_KP_HIGH:               *value = fb->_params.pressureKpHigh; break;
        case HYD_PARAM_PRESSURE_GAIN_BAND:             *value = fb->_params.pressureGainBand; break;
        case HYD_PARAM_PRESSURE_KI:                    *value = fb->_params.pressureKi; break;
        case HYD_PARAM_PRESSURE_KD:                    *value = fb->_params.pressureKd; break;
        case HYD_PARAM_PRESSURE_INTEGRAL_LIMIT:        *value = fb->_params.pressureIntegralLimit; break;
        case HYD_PARAM_PRESSURE_DEADBAND:              *value = fb->_params.pressureDeadband; break;
        case HYD_PARAM_PRESSURE_FILTER_ALPHA:          *value = fb->_params.pressureFilterAlpha; break;
        case HYD_PARAM_PRESSURE_DERIVATIVE_FILTER_ALPHA: *value = fb->_params.pressureDerivativeFilterAlpha; break;
        case HYD_PARAM_FLOW_TO_PUMP_SPEED_GAIN:        *value = fb->_params.flowToPumpSpeedGain; break;
        case HYD_PARAM_PUMP_SPEED_LIMIT:               *value = fb->_params.pumpSpeedLimit; break;
        case HYD_PARAM_PRESSURE_CONTROLLER_TYPE:       *value = fb->_params.pressureControllerType; break;
        case HYD_PARAM_DEFAULT_TARGET_FLOW:            *value = fb->_params.defaultTargetFlow; break;
        default: return false;
    }
    return true;
}
```

- [ ] **Step 3: Implement HYD_MotionControlFB_WriteParameter**

```c
HYD_BOOL HYD_MotionControlFB_WriteParameter(HYD_MotionControlFB* fb, int paramNumber, HYD_REAL value)
{
    if (fb == NULL) return false;
    if (paramNumber < 0 || paramNumber >= HYD_PARAM_COUNT) return false;
    if (paramNumber == HYD_PARAM_USE_SIMULATION) return false; /* BOOL param, use WriteBoolParameter */

    switch ((HYD_ParameterNumber)paramNumber) {
        case HYD_PARAM_POSITION_TOLERANCE:             fb->_params.positionTolerance = value; break;
        case HYD_PARAM_VELOCITY_TOLERANCE:             fb->_params.velocityTolerance = value; break;
        case HYD_PARAM_FLOW_TOLERANCE:                 fb->_params.flowTolerance = value; break;
        case HYD_PARAM_PRESSURE_TOLERANCE:             fb->_params.pressureTolerance = value; break;
        case HYD_PARAM_TIMEOUT_LIMIT:                  fb->_params.timeoutLimit = value; break;
        case HYD_PARAM_VELOCITY_TO_FLOW_GAIN:          fb->_params.velocityToFlowGain = value; break;
        case HYD_PARAM_MAX_VELOCITY:                   fb->_params.maxVelocity = value; break;
        case HYD_PARAM_MAX_ACCELERATION:               fb->_params.maxAcceleration = value; break;
        case HYD_PARAM_MAX_DECELERATION:               fb->_params.maxDeceleration = value; break;
        case HYD_PARAM_MAX_FLOW:                       fb->_params.maxFlow = value; break;
        case HYD_PARAM_PRESSURE_RAMP_RATE:             fb->_params.pressureRampRate = value; break;
        case HYD_PARAM_PRESSURE_KP:                    fb->_params.pressureKp = value; break;
        case HYD_PARAM_PRESSURE_KP_HIGH:               fb->_params.pressureKpHigh = value; break;
        case HYD_PARAM_PRESSURE_GAIN_BAND:             fb->_params.pressureGainBand = value; break;
        case HYD_PARAM_PRESSURE_KI:                    fb->_params.pressureKi = value; break;
        case HYD_PARAM_PRESSURE_KD:                    fb->_params.pressureKd = value; break;
        case HYD_PARAM_PRESSURE_INTEGRAL_LIMIT:        fb->_params.pressureIntegralLimit = value; break;
        case HYD_PARAM_PRESSURE_DEADBAND:              fb->_params.pressureDeadband = value; break;
        case HYD_PARAM_PRESSURE_FILTER_ALPHA:          fb->_params.pressureFilterAlpha = value; break;
        case HYD_PARAM_PRESSURE_DERIVATIVE_FILTER_ALPHA: fb->_params.pressureDerivativeFilterAlpha = value; break;
        case HYD_PARAM_FLOW_TO_PUMP_SPEED_GAIN:
            fb->_params.flowToPumpSpeedGain = value;
            fb->FLOW_TO_PUMP_SPEED_GAIN = value;  /* sync legacy field */
            break;
        case HYD_PARAM_PUMP_SPEED_LIMIT:
            fb->_params.pumpSpeedLimit = value;
            fb->PUMP_SPEED_LIMIT = value;  /* sync legacy field */
            break;
        case HYD_PARAM_PRESSURE_CONTROLLER_TYPE:       fb->_params.pressureControllerType = value; break;
        case HYD_PARAM_DEFAULT_TARGET_FLOW:            fb->_params.defaultTargetFlow = value; break;
        default: return false;
    }
    return true;
}
```

- [ ] **Step 4: Implement HYD_MotionControlFB_ReadBoolParameter**

```c
HYD_BOOL HYD_MotionControlFB_ReadBoolParameter(const HYD_MotionControlFB* fb, int paramNumber, HYD_BOOL* value)
{
    if (fb == NULL || value == NULL) return false;
    if (paramNumber != HYD_PARAM_USE_SIMULATION) return false; /* only BOOL param currently */

    *value = fb->_params.useSimulation;
    return true;
}
```

- [ ] **Step 5: Implement HYD_MotionControlFB_WriteBoolParameter**

```c
HYD_BOOL HYD_MotionControlFB_WriteBoolParameter(HYD_MotionControlFB* fb, int paramNumber, HYD_BOOL value)
{
    if (fb == NULL) return false;
    if (paramNumber != HYD_PARAM_USE_SIMULATION) return false; /* only BOOL param currently */

    fb->_params.useSimulation = value;
    fb->_useSimulation = value;  /* sync legacy field */
    return true;
}
```

- [ ] **Step 6: Build to verify compilation**

```bash
cmake --build --preset unixgcc
```

Expected: builds successfully.

---

### Task 4: Write unit tests for parameter accessors

**Files:**
- Create: `tests/test_parameter_access.c`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Write the failing test file**

```c
#include <stdio.h>
#include <string.h>
#include "motion_control.h"

static int tests_run = 0;
static int tests_passed = 0;

#define ASSERT_TRUE(cond, msg) do { \
    tests_run++; \
    if (cond) { tests_passed++; } \
    else { printf("  FAIL: %s\n", msg); } \
} while (0)

#define ASSERT_EQ(a, b, msg) do { \
    tests_run++; \
    if ((a) == (b)) { tests_passed++; } \
    else { printf("  FAIL: %s (got %d, expected %d)\n", msg, (int)(a), (int)(b)); } \
} while (0)

#define ASSERT_FLOAT_EQ(a, b, eps, msg) do { \
    tests_run++; \
    HYD_REAL diff = (a) - (b); \
    if (diff < 0.0f) diff = -diff; \
    if (diff <= (eps)) { tests_passed++; } \
    else { printf("  FAIL: %s (got %.6f, expected %.6f)\n", msg, (double)(a), (double)(b)); } \
} while (0)

/* Test: Init sets expected defaults */
static void test_init_sets_defaults(void) {
    HYD_MotionControlFB fb;
    HYD_MotionControlFB_Init(&fb);

    HYD_REAL val;
    HYD_BOOL ok;

    ok = HYD_MotionControlFB_ReadParameter(&fb, HYD_PARAM_POSITION_TOLERANCE, &val);
    ASSERT_TRUE(ok, "Read positionTolerance should succeed");
    ASSERT_FLOAT_EQ(val, 0.1f, 0.001f, "Default positionTolerance should be 0.1");

    ok = HYD_MotionControlFB_ReadParameter(&fb, HYD_PARAM_TIMEOUT_LIMIT, &val);
    ASSERT_TRUE(ok, "Read timeoutLimit should succeed");
    ASSERT_FLOAT_EQ(val, 30.0f, 0.001f, "Default timeoutLimit should be 30.0");

    ok = HYD_MotionControlFB_ReadParameter(&fb, HYD_PARAM_VELOCITY_TO_FLOW_GAIN, &val);
    ASSERT_TRUE(ok, "Read velocityToFlowGain should succeed");
    ASSERT_FLOAT_EQ(val, 0.2f, 0.001f, "Default velocityToFlowGain should be 0.2");

    ok = HYD_MotionControlFB_ReadParameter(&fb, HYD_PARAM_FLOW_TO_PUMP_SPEED_GAIN, &val);
    ASSERT_TRUE(ok, "Read flowToPumpSpeedGain should succeed");
    ASSERT_FLOAT_EQ(val, 20.0f, 0.001f, "Default flowToPumpSpeedGain should be 20.0");

    ok = HYD_MotionControlFB_ReadParameter(&fb, HYD_PARAM_PUMP_SPEED_LIMIT, &val);
    ASSERT_TRUE(ok, "Read pumpSpeedLimit should succeed");
    ASSERT_FLOAT_EQ(val, 1800.0f, 0.001f, "Default pumpSpeedLimit should be 1800.0");

    HYD_BOOL bval;
    ok = HYD_MotionControlFB_ReadBoolParameter(&fb, HYD_PARAM_USE_SIMULATION, &bval);
    ASSERT_TRUE(ok, "ReadBool useSimulation should succeed");
    ASSERT_TRUE(bval == false, "Default useSimulation should be false");
}

/* Test: Write then Read round-trip */
static void test_write_read_roundtrip(void) {
    HYD_MotionControlFB fb;
    HYD_MotionControlFB_Init(&fb);

    HYD_REAL val;
    HYD_BOOL ok;

    ok = HYD_MotionControlFB_WriteParameter(&fb, HYD_PARAM_POSITION_TOLERANCE, 0.05f);
    ASSERT_TRUE(ok, "Write positionTolerance should succeed");
    ok = HYD_MotionControlFB_ReadParameter(&fb, HYD_PARAM_POSITION_TOLERANCE, &val);
    ASSERT_TRUE(ok, "Read after write should succeed");
    ASSERT_FLOAT_EQ(val, 0.05f, 0.001f, "positionTolerance should be 0.05 after write");

    ok = HYD_MotionControlFB_WriteParameter(&fb, HYD_PARAM_PRESSURE_KP, 1.5f);
    ASSERT_TRUE(ok, "Write pressureKp should succeed");
    ok = HYD_MotionControlFB_ReadParameter(&fb, HYD_PARAM_PRESSURE_KP, &val);
    ASSERT_TRUE(ok, "Read after write should succeed");
    ASSERT_FLOAT_EQ(val, 1.5f, 0.001f, "pressureKp should be 1.5 after write");

    ok = HYD_MotionControlFB_WriteParameter(&fb, HYD_PARAM_PRESSURE_CONTROLLER_TYPE, (HYD_REAL)HYD_PRESSURE_CONTROLLER_PID);
    ASSERT_TRUE(ok, "Write pressureControllerType should succeed");
    ok = HYD_MotionControlFB_ReadParameter(&fb, HYD_PARAM_PRESSURE_CONTROLLER_TYPE, &val);
    ASSERT_TRUE(ok, "Read pressureControllerType should succeed");
    ASSERT_EQ((int)val, (int)HYD_PRESSURE_CONTROLLER_PID, "pressureControllerType should be PID(3)");
}

/* Test: Write then Read Bool round-trip */
static void test_write_read_bool_roundtrip(void) {
    HYD_MotionControlFB fb;
    HYD_MotionControlFB_Init(&fb);

    HYD_BOOL bval;
    HYD_BOOL ok;

    ok = HYD_MotionControlFB_WriteBoolParameter(&fb, HYD_PARAM_USE_SIMULATION, true);
    ASSERT_TRUE(ok, "WriteBool useSimulation should succeed");
    ok = HYD_MotionControlFB_ReadBoolParameter(&fb, HYD_PARAM_USE_SIMULATION, &bval);
    ASSERT_TRUE(ok, "ReadBool after write should succeed");
    ASSERT_TRUE(bval == true, "useSimulation should be true after write");
    ASSERT_TRUE(fb._useSimulation == true, "Legacy _useSimulation field should be synced");
}

/* Test: Invalid paramNumber returns false */
static void test_invalid_param_number(void) {
    HYD_MotionControlFB fb;
    HYD_MotionControlFB_Init(&fb);

    HYD_REAL val;
    HYD_BOOL bval;
    HYD_BOOL ok;

    ok = HYD_MotionControlFB_ReadParameter(&fb, -1, &val);
    ASSERT_TRUE(!ok, "ReadParameter with -1 should fail");

    ok = HYD_MotionControlFB_ReadParameter(&fb, HYD_PARAM_COUNT, &val);
    ASSERT_TRUE(!ok, "ReadParameter with PARAM_COUNT should fail");

    ok = HYD_MotionControlFB_ReadParameter(&fb, 999, &val);
    ASSERT_TRUE(!ok, "ReadParameter with 999 should fail");

    ok = HYD_MotionControlFB_WriteParameter(&fb, -1, 0.0f);
    ASSERT_TRUE(!ok, "WriteParameter with -1 should fail");

    ok = HYD_MotionControlFB_ReadBoolParameter(&fb, HYD_PARAM_POSITION_TOLERANCE, &bval);
    ASSERT_TRUE(!ok, "ReadBoolParameter with REAL param number should fail");

    ok = HYD_MotionControlFB_WriteBoolParameter(&fb, HYD_PARAM_TIMEOUT_LIMIT, true);
    ASSERT_TRUE(!ok, "WriteBoolParameter with REAL param number should fail");
}

/* Test: Type mismatch — WriteParameter with USE_SIMULATION ID fails */
static void test_type_mismatch_rejected(void) {
    HYD_MotionControlFB fb;
    HYD_MotionControlFB_Init(&fb);

    HYD_REAL val;
    HYD_BOOL ok;

    ok = HYD_MotionControlFB_ReadParameter(&fb, HYD_PARAM_USE_SIMULATION, &val);
    ASSERT_TRUE(!ok, "ReadParameter with USE_SIMULATION should fail (type mismatch)");

    ok = HYD_MotionControlFB_WriteParameter(&fb, HYD_PARAM_USE_SIMULATION, 1.0f);
    ASSERT_TRUE(!ok, "WriteParameter with USE_SIMULATION should fail (type mismatch)");
}

/* Test: Legacy field sync on WriteParameter for pump fields */
static void test_legacy_field_sync_on_write(void) {
    HYD_MotionControlFB fb;
    HYD_MotionControlFB_Init(&fb);

    HYD_BOOL ok;

    ok = HYD_MotionControlFB_WriteParameter(&fb, HYD_PARAM_FLOW_TO_PUMP_SPEED_GAIN, 50.0f);
    ASSERT_TRUE(ok, "Write flowToPumpSpeedGain should succeed");
    ASSERT_FLOAT_EQ(fb.FLOW_TO_PUMP_SPEED_GAIN, 50.0f, 0.001f, "Legacy FLOW_TO_PUMP_SPEED_GAIN should be 50.0");

    ok = HYD_MotionControlFB_WriteParameter(&fb, HYD_PARAM_PUMP_SPEED_LIMIT, 3000.0f);
    ASSERT_TRUE(ok, "Write pumpSpeedLimit should succeed");
    ASSERT_FLOAT_EQ(fb.PUMP_SPEED_LIMIT, 3000.0f, 0.001f, "Legacy PUMP_SPEED_LIMIT should be 3000.0");

    ok = HYD_MotionControlFB_WriteBoolParameter(&fb, HYD_PARAM_USE_SIMULATION, true);
    ASSERT_TRUE(ok, "WriteBool useSimulation should succeed");
    ASSERT_TRUE(fb._useSimulation == true, "Legacy _useSimulation should be synced to true");
}

int main(void) {
    test_init_sets_defaults();
    test_write_read_roundtrip();
    test_write_read_bool_roundtrip();
    test_invalid_param_number();
    test_type_mismatch_rejected();
    test_legacy_field_sync_on_write();

    printf("Parameter access tests: %d/%d passed\n", tests_passed, tests_run);
    return (tests_passed == tests_run) ? 0 : 1;
}
```

- [ ] **Step 2: Add test target to CMakeLists.txt**

Add before the `enable_testing()` line:

```cmake
add_executable(test_parameter_access tests/test_parameter_access.c)
target_link_libraries(test_parameter_access PRIVATE HydroMotionLib)
```

Add after `add_test(NAME test_motion_planner ...)`:

```cmake
add_test(NAME test_parameter_access COMMAND test_parameter_access)
```

- [ ] **Step 3: Configure and build**

```bash
cmake --preset unixgcc
cmake --build --preset unixgcc --target test_parameter_access
```

- [ ] **Step 4: Run the test — verify it passes**

```bash
./out/build/unixgcc/test_parameter_access
```

Expected: `Parameter access tests: N/N passed`

---

### Task 5: Fix segment builders to read from fb->_params

**Files:**
- Modify: `src/motion_interface.c`

- [ ] **Step 1: Update buildPositionSegment signature and body**

Change the function to accept `const HYD_MotionControlFB* fb`:

```c
static HYD_MotionSegment buildPositionSegment(
    HYD_REAL targetPosition,
    HYD_REAL velocity,
    HYD_REAL acceleration,
    HYD_MotionDirection direction,
    const HYD_MotionControlFB* fb)
{
    HYD_MotionSegment seg;
    memset(&seg, 0, sizeof(seg));

    seg.segmentTag = HYD_SEGMENT_TYPE_OTHER;
    seg.segmentType = HYD_SEGMENT_TYPE_OTHER;
    seg.mode = HYD_MODE_POSITION;
    seg.endCondition = HYD_END_POSITION;
    seg.direction = direction;
    seg.planner = HYD_PLANNER_POSITION_BASED;

    seg.targetPosition = targetPosition;
    seg.maxVelocity = velocity;
    seg.maxAcceleration = acceleration;
    seg.maxFlow = (velocity > 0.0f) ? velocity * fb->_params.velocityToFlowGain : fb->_params.maxFlow;
    seg.velocityToFlowGain = fb->_params.velocityToFlowGain;

    seg.positionTolerance = fb->_params.positionTolerance;
    seg.timeoutLimit = fb->_params.timeoutLimit;

    return seg;
}
```

- [ ] **Step 2: Update call site in __mcl_cmd_MoveAbsolute**

Find `buildPositionSegment(...)` call and add `fb`:

```c
        HYD_MotionSegment segment = buildPositionSegment(
            __GET_VAR(data__->POSITION),
            __GET_VAR(data__->VELOCITY),
            __GET_VAR(data__->ACCELERATION),
            dir,
            fb);
```

- [ ] **Step 3: Update buildVelocitySegment signature and body**

```c
static HYD_MotionSegment buildVelocitySegment(
    HYD_REAL velocity,
    HYD_REAL acceleration,
    HYD_MotionDirection direction,
    const HYD_MotionControlFB* fb)
{
    HYD_MotionSegment seg;
    memset(&seg, 0, sizeof(seg));

    seg.segmentTag = HYD_SEGMENT_TYPE_OTHER;
    seg.segmentType = HYD_SEGMENT_TYPE_OTHER;
    seg.mode = HYD_MODE_SPEED_RAMP;
    seg.endCondition = HYD_END_MANUAL;
    seg.direction = direction;
    seg.planner = HYD_PLANNER_TIME_BASED;

    seg.maxVelocity = velocity;
    seg.maxAcceleration = acceleration;
    seg.maxFlow = (velocity > 0.0f) ? velocity * fb->_params.velocityToFlowGain : fb->_params.maxFlow;
    seg.velocityToFlowGain = fb->_params.velocityToFlowGain;

    seg.timeoutLimit = 0.0f;

    return seg;
}
```

- [ ] **Step 4: Update call site in __mcl_cmd_MoveVelocity**

Find `buildVelocitySegment(...)` call and add `fb`:

```c
        HYD_MotionSegment segment = buildVelocitySegment(
            targetVelocity,
            __GET_VAR(data__->ACCELERATION),
            dir,
            fb);
```

- [ ] **Step 5: Update buildPressureSegment signature and body**

```c
static HYD_MotionSegment buildPressureSegment(
    HYD_REAL targetPressure,
    HYD_REAL rampRate,
    HYD_REAL duration,
    const HYD_MotionControlFB* fb)
{
    HYD_MotionSegment seg;
    memset(&seg, 0, sizeof(seg));

    seg.segmentTag = HYD_SEGMENT_TYPE_HOLDING;
    seg.segmentType = HYD_SEGMENT_TYPE_HOLDING;
    seg.mode = HYD_MODE_PRESSURE_CLOSED_LOOP;
    seg.endCondition = (duration > 0.0f) ? HYD_END_TIME : HYD_END_MANUAL;
    seg.direction = HYD_DIRECTION_HOLD;

    seg.targetPressure = targetPressure;
    seg.targetFlow = fb->_params.defaultTargetFlow;
    seg.maxFlow = fb->_params.maxFlow;
    seg.duration = duration;
    seg.pressureRampRate = rampRate;

    seg.pressureController = (HYD_PressureControllerType)(int)fb->_params.pressureControllerType;
    seg.pressureKp = fb->_params.pressureKp;
    seg.pressureKi = fb->_params.pressureKi;
    seg.pressureKd = fb->_params.pressureKd;
    seg.pressureIntegralLimit = fb->_params.pressureIntegralLimit;
    seg.pressureDeadband = fb->_params.pressureDeadband;
    seg.pressureFilterAlpha = fb->_params.pressureFilterAlpha;
    seg.pressureDerivativeFilterAlpha = fb->_params.pressureDerivativeFilterAlpha;

    seg.pressureTolerance = fb->_params.pressureTolerance;
    seg.flowTolerance = fb->_params.flowTolerance;
    seg.timeoutLimit = fb->_params.timeoutLimit;

    return seg;
}
```

- [ ] **Step 6: Update call site in __mcl_cmd_PressureHandle**

Find `buildPressureSegment(...)` call and add `fb`:

```c
        HYD_MotionSegment segment = buildPressureSegment(
            targetPressure,
            __GET_VAR(data__->PRESSURERAMPRATE),
            __GET_VAR(data__->DURATION),
            fb);
```

- [ ] **Step 7: Update buildSegmentFromMotion signature and body**

Add `fb` parameter, replace hardcoded values:

```c
static HYD_MotionSegment buildSegmentFromMotion(const HYD_AXISMOTION* motion,
                                                 const HYD_MotionControlFB* fb)
{
    HYD_MotionSegment seg;
    memset(&seg, 0, sizeof(seg));

    seg.segmentTag = (HYD_UINT8)motion->SEGMENTTAG;
    seg.segmentType = (HYD_SegmentType)motion->SEGMENTTAG;
    seg.mode = (HYD_ControlMode)motion->MODE;
    seg.endCondition = (HYD_EndConditionType)motion->ENDCONDITION;
    seg.direction = (HYD_MotionDirection)motion->DIRECTION;
    seg.planner = (HYD_PlannerType)motion->PLANNER;

    seg.targetPosition = motion->SETPOSITION;
    seg.maxVelocity = motion->SETVELOCITY;
    seg.targetFlow = motion->SETFLOW;
    seg.maxFlow = motion->SETFLOW;
    seg.targetPressure = motion->SETPRESSURE;
    seg.maxAcceleration = motion->ACCELERATION;
    seg.duration = motion->DURATION;
    seg.pressureRampRate = motion->PRESSURERAMPRATE;

    seg.velocityToFlowGain = fb->_params.velocityToFlowGain;
    seg.positionTolerance = fb->_params.positionTolerance;
    seg.pressureTolerance = fb->_params.pressureTolerance;
    seg.flowTolerance = fb->_params.flowTolerance;
    seg.timeoutLimit = fb->_params.timeoutLimit;

    seg.pressureController = (HYD_PressureControllerType)(int)fb->_params.pressureControllerType;
    seg.pressureKp = fb->_params.pressureKp;
    seg.pressureKi = fb->_params.pressureKi;
    seg.pressureKd = fb->_params.pressureKd;
    seg.pressureIntegralLimit = fb->_params.pressureIntegralLimit;
    seg.pressureDeadband = fb->_params.pressureDeadband;
    seg.pressureFilterAlpha = fb->_params.pressureFilterAlpha;
    seg.pressureDerivativeFilterAlpha = fb->_params.pressureDerivativeFilterAlpha;

    return seg;
}
```

- [ ] **Step 8: Update call site in __mcl_cmd_MoveProfile**

Find `buildSegmentFromMotion(&motionData)` and add `fb`:

```c
            HYD_MotionSegment segment = buildSegmentFromMotion(&motionData, fb);
```

- [ ] **Step 9: Build and verify compilation**

```bash
cmake --build --preset unixgcc
```

Expected: builds successfully.

---

### Task 6: Add 4 new IEC FB typedefs to motion_interface.h

**Files:**
- Modify: `include/motion_interface.h`

- [ ] **Step 1: Add FB typedefs before the extern declarations**

Add after the `HYD_READSIMFEEDBACK` typedef and before the `extern int __HydMotion_framework_Init();` line:

```c
// FUNCTION_BLOCK HYD_READPARAMETER
// Data part
typedef struct {
    __DECLARE_VAR(BOOL, EN)
    __DECLARE_VAR(BOOL, ENO)
    __DECLARE_VAR(SINT, AXISID)
    __DECLARE_VAR(BOOL, ENABLE)
    __DECLARE_VAR(INT, PARAMETERNUMBER)
    __DECLARE_VAR(BOOL, VALID)
    __DECLARE_VAR(BOOL, BUSY)
    __DECLARE_VAR(BOOL, ERROR)
    __DECLARE_VAR(WORD, ERRORID)
    __DECLARE_VAR(LREAL, VALUE)
    __DECLARE_VAR(BOOL, ENABLE0)
} HYD_READPARAMETER;

// FUNCTION_BLOCK HYD_WRITEPARAMETER
// Data part
typedef struct {
    __DECLARE_VAR(BOOL, EN)
    __DECLARE_VAR(BOOL, ENO)
    __DECLARE_VAR(SINT, AXISID)
    __DECLARE_VAR(BOOL, EXECUTE)
    __DECLARE_VAR(INT, PARAMETERNUMBER)
    __DECLARE_VAR(LREAL, VALUE)
    __DECLARE_VAR(BOOL, DONE)
    __DECLARE_VAR(BOOL, BUSY)
    __DECLARE_VAR(BOOL, ERROR)
    __DECLARE_VAR(WORD, ERRORID)
    __DECLARE_VAR(BOOL, EXECUTE0)
    __DECLARE_VAR(BOOL, DONE0)
} HYD_WRITEPARAMETER;

// FUNCTION_BLOCK HYD_READBOOLPARAMETER
// Data part
typedef struct {
    __DECLARE_VAR(BOOL, EN)
    __DECLARE_VAR(BOOL, ENO)
    __DECLARE_VAR(SINT, AXISID)
    __DECLARE_VAR(BOOL, ENABLE)
    __DECLARE_VAR(INT, PARAMETERNUMBER)
    __DECLARE_VAR(BOOL, VALID)
    __DECLARE_VAR(BOOL, BUSY)
    __DECLARE_VAR(BOOL, ERROR)
    __DECLARE_VAR(WORD, ERRORID)
    __DECLARE_VAR(BOOL, VALUE)
    __DECLARE_VAR(BOOL, ENABLE0)
} HYD_READBOOLPARAMETER;

// FUNCTION_BLOCK HYD_WRITEBOOLPARAMETER
// Data part
typedef struct {
    __DECLARE_VAR(BOOL, EN)
    __DECLARE_VAR(BOOL, ENO)
    __DECLARE_VAR(SINT, AXISID)
    __DECLARE_VAR(BOOL, EXECUTE)
    __DECLARE_VAR(INT, PARAMETERNUMBER)
    __DECLARE_VAR(BOOL, VALUE)
    __DECLARE_VAR(BOOL, DONE)
    __DECLARE_VAR(BOOL, BUSY)
    __DECLARE_VAR(BOOL, ERROR)
    __DECLARE_VAR(WORD, ERRORID)
    __DECLARE_VAR(BOOL, EXECUTE0)
    __DECLARE_VAR(BOOL, DONE0)
} HYD_WRITEBOOLPARAMETER;
```

- [ ] **Step 2: Add extern function declarations**

Add before `#endif` at end of file:

```c
extern void __mcl_cmd_ReadParameter(HYD_READPARAMETER* data__);
extern void __mcl_cmd_WriteParameter(HYD_WRITEPARAMETER* data__);
extern void __mcl_cmd_ReadBoolParameter(HYD_READBOOLPARAMETER* data__);
extern void __mcl_cmd_WriteBoolParameter(HYD_WRITEBOOLPARAMETER* data__);
```

- [ ] **Step 3: Build to verify**

```bash
cmake --build --preset unixgcc
```

Expected: linker errors for the 4 undefined `__mcl_cmd_*` functions — expected, will implement next.

---

### Task 7: Implement 4 IEC command functions in motion_interface.c

**Files:**
- Modify: `src/motion_interface.c`

- [ ] **Step 1: Add __mcl_cmd_ReadParameter implementation**

Add before the `__mcl_cmd_SetAxisFeedback` function:

```c
void __mcl_cmd_ReadParameter(HYD_READPARAMETER *data__)
{
    IEC_BOOL enable = __GET_VAR(data__->ENABLE);
    IEC_BOOL enRising = enable && !__GET_VAR(data__->ENABLE0);
    IEC_SINT axisIndex = __GET_VAR(data__->AXISID);

    if (axisIndex < 0 || axisIndex >= (IEC_SINT)nextAllocatedFB)
    {
        __SET_VAR(data__->, ERROR,, true);
        __SET_VAR(data__->, ERRORID,, (IEC_WORD)HYD_DIAG_CODE_START_CONTEXT_INVALID);
        __SET_VAR(data__->, VALID,, false);
        __SET_VAR(data__->, BUSY,, false);
        __SET_VAR(data__->, ENABLE0,, enable);
        return;
    }

    HYD_MotionControlFB* fb = &HYD_MotionControlFB_inst[axisIndex];

    if (enable)
    {
        HYD_REAL value;
        if (HYD_MotionControlFB_ReadParameter(fb, __GET_VAR(data__->PARAMETERNUMBER), &value))
        {
            __SET_VAR(data__->, VALUE,, (IEC_LREAL)value);
            __SET_VAR(data__->, VALID,, true);
            __SET_VAR(data__->, ERROR,, false);
            __SET_VAR(data__->, ERRORID,, (IEC_WORD)HYD_DIAG_CODE_NONE);
        }
        else
        {
            __SET_VAR(data__->, ERROR,, true);
            __SET_VAR(data__->, ERRORID,, (IEC_WORD)HYD_DIAG_CODE_SEGMENT_INVALID);
            __SET_VAR(data__->, VALID,, false);
        }
        __SET_VAR(data__->, BUSY,, false);
    }
    else
    {
        __SET_VAR(data__->, VALID,, false);
        __SET_VAR(data__->, BUSY,, false);
    }

    __SET_VAR(data__->, ENABLE0,, enable);
}
```

- [ ] **Step 2: Add __mcl_cmd_WriteParameter implementation**

```c
void __mcl_cmd_WriteParameter(HYD_WRITEPARAMETER *data__)
{
    IEC_BOOL execute = __GET_VAR(data__->EXECUTE);
    IEC_BOOL execRising = execute && !__GET_VAR(data__->EXECUTE0);
    IEC_SINT axisIndex = __GET_VAR(data__->AXISID);

    if (axisIndex < 0 || axisIndex >= (IEC_SINT)nextAllocatedFB)
    {
        __SET_VAR(data__->, ERROR,, true);
        __SET_VAR(data__->, ERRORID,, (IEC_WORD)HYD_DIAG_CODE_START_CONTEXT_INVALID);
        __SET_VAR(data__->, EXECUTE0,, execute);
        return;
    }

    HYD_MotionControlFB* fb = &HYD_MotionControlFB_inst[axisIndex];

    if (execRising)
    {
        IEC_LREAL value = __GET_VAR(data__->VALUE);
        if (HYD_MotionControlFB_WriteParameter(fb, __GET_VAR(data__->PARAMETERNUMBER), (HYD_REAL)value))
        {
            __SET_VAR(data__->, DONE,, true);
            __SET_VAR(data__->, BUSY,, false);
            __SET_VAR(data__->, ERROR,, false);
            __SET_VAR(data__->, ERRORID,, (IEC_WORD)HYD_DIAG_CODE_NONE);
        }
        else
        {
            __SET_VAR(data__->, ERROR,, true);
            __SET_VAR(data__->, ERRORID,, (IEC_WORD)HYD_DIAG_CODE_SEGMENT_INVALID);
            __SET_VAR(data__->, DONE,, false);
            __SET_VAR(data__->, BUSY,, false);
        }
    }

    __SET_VAR(data__->, EXECUTE0,, execute);
}
```

- [ ] **Step 3: Add __mcl_cmd_ReadBoolParameter implementation**

```c
void __mcl_cmd_ReadBoolParameter(HYD_READBOOLPARAMETER *data__)
{
    IEC_BOOL enable = __GET_VAR(data__->ENABLE);
    IEC_BOOL enRising = enable && !__GET_VAR(data__->ENABLE0);
    IEC_SINT axisIndex = __GET_VAR(data__->AXISID);

    if (axisIndex < 0 || axisIndex >= (IEC_SINT)nextAllocatedFB)
    {
        __SET_VAR(data__->, ERROR,, true);
        __SET_VAR(data__->, ERRORID,, (IEC_WORD)HYD_DIAG_CODE_START_CONTEXT_INVALID);
        __SET_VAR(data__->, VALID,, false);
        __SET_VAR(data__->, BUSY,, false);
        __SET_VAR(data__->, ENABLE0,, enable);
        return;
    }

    HYD_MotionControlFB* fb = &HYD_MotionControlFB_inst[axisIndex];

    if (enable)
    {
        HYD_BOOL value;
        if (HYD_MotionControlFB_ReadBoolParameter(fb, __GET_VAR(data__->PARAMETERNUMBER), &value))
        {
            __SET_VAR(data__->, VALUE,, value ? true : false);
            __SET_VAR(data__->, VALID,, true);
            __SET_VAR(data__->, ERROR,, false);
            __SET_VAR(data__->, ERRORID,, (IEC_WORD)HYD_DIAG_CODE_NONE);
        }
        else
        {
            __SET_VAR(data__->, ERROR,, true);
            __SET_VAR(data__->, ERRORID,, (IEC_WORD)HYD_DIAG_CODE_SEGMENT_INVALID);
            __SET_VAR(data__->, VALID,, false);
        }
        __SET_VAR(data__->, BUSY,, false);
    }
    else
    {
        __SET_VAR(data__->, VALID,, false);
        __SET_VAR(data__->, BUSY,, false);
    }

    __SET_VAR(data__->, ENABLE0,, enable);
}
```

- [ ] **Step 4: Add __mcl_cmd_WriteBoolParameter implementation**

```c
void __mcl_cmd_WriteBoolParameter(HYD_WRITEBOOLPARAMETER *data__)
{
    IEC_BOOL execute = __GET_VAR(data__->EXECUTE);
    IEC_BOOL execRising = execute && !__GET_VAR(data__->EXECUTE0);
    IEC_SINT axisIndex = __GET_VAR(data__->AXISID);

    if (axisIndex < 0 || axisIndex >= (IEC_SINT)nextAllocatedFB)
    {
        __SET_VAR(data__->, ERROR,, true);
        __SET_VAR(data__->, ERRORID,, (IEC_WORD)HYD_DIAG_CODE_START_CONTEXT_INVALID);
        __SET_VAR(data__->, EXECUTE0,, execute);
        return;
    }

    HYD_MotionControlFB* fb = &HYD_MotionControlFB_inst[axisIndex];

    if (execRising)
    {
        IEC_BOOL value = __GET_VAR(data__->VALUE);
        if (HYD_MotionControlFB_WriteBoolParameter(fb, __GET_VAR(data__->PARAMETERNUMBER), value ? true : false))
        {
            __SET_VAR(data__->, DONE,, true);
            __SET_VAR(data__->, BUSY,, false);
            __SET_VAR(data__->, ERROR,, false);
            __SET_VAR(data__->, ERRORID,, (IEC_WORD)HYD_DIAG_CODE_NONE);
        }
        else
        {
            __SET_VAR(data__->, ERROR,, true);
            __SET_VAR(data__->, ERRORID,, (IEC_WORD)HYD_DIAG_CODE_SEGMENT_INVALID);
            __SET_VAR(data__->, DONE,, false);
            __SET_VAR(data__->, BUSY,, false);
        }
    }

    __SET_VAR(data__->, EXECUTE0,, execute);
}
```

- [ ] **Step 5: Build and verify**

```bash
cmake --build --preset unixgcc
```

Expected: builds successfully, all symbols resolved.

---

### Task 8: Add IEC interface layer integration test

**Files:**
- Create: `tests/test_parameter_iec.c`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Write the IEC-level test**

```c
#include <stdio.h>
#include <string.h>
#include "motion_interface.h"
#include "motion_control.h"

extern HYD_MotionControlFB* __MK_GetPublic_MotionControlFB(int index);

#define IEC_VAL(var) ((var).value)

static int tests_run = 0;
static int tests_passed = 0;

#define ASSERT_TRUE(cond, msg) do { \
    tests_run++; \
    if (cond) { tests_passed++; } \
    else { printf("  FAIL: %s\n", msg); } \
} while (0)

#define ASSERT_FLOAT_EQ(a, b, eps, msg) do { \
    tests_run++; \
    HYD_REAL diff = (a) - (b); \
    if (diff < 0.0f) diff = -diff; \
    if (diff <= (eps)) { tests_passed++; } \
    else { printf("  FAIL: %s (got %.6f, expected %.6f)\n", msg, (double)(a), (double)(b)); } \
} while (0)

static void ensure_axis_allocated(void) {
    HYD_CREATEMOTION cm;
    memset(&cm, 0, sizeof(cm));
    IEC_VAL(cm.EN) = true;
    IEC_VAL(cm.USE_RECIPE) = false;
    IEC_VAL(cm.FLOW_TO_PUMPSPEED) = 20.0f;
    IEC_VAL(cm.PUMPSPEED_LIMIT) = 1800.0f;
    IEC_VAL(cm.USE_SIMULATION) = false;
    __mcl_cmd_CreateMotion(&cm);
}

/* Test: ReadParameter works through IEC FB */
static void test_read_parameter_iec(void) {
    __HydMotion_framework_Init();
    ensure_axis_allocated();

    HYD_READPARAMETER rp;
    memset(&rp, 0, sizeof(rp));
    IEC_VAL(rp.EN) = true;
    IEC_VAL(rp.AXISID) = 0;
    IEC_VAL(rp.ENABLE) = true;
    IEC_VAL(rp.PARAMETERNUMBER) = HYD_PARAM_POSITION_TOLERANCE;

    __mcl_cmd_ReadParameter(&rp);

    ASSERT_TRUE(IEC_VAL(rp.VALID) == true, "ReadParameter should set VALID");
    ASSERT_TRUE(IEC_VAL(rp.ERROR) == false, "ReadParameter should not set ERROR");
    ASSERT_FLOAT_EQ((HYD_REAL)IEC_VAL(rp.VALUE), 0.1f, 0.001f, "Default positionTolerance should be 0.1");
}

/* Test: WriteParameter then ReadParameter through IEC FBs */
static void test_write_then_read_iec(void) {
    __HydMotion_framework_Init();
    ensure_axis_allocated();

    HYD_WRITEPARAMETER wp;
    memset(&wp, 0, sizeof(wp));
    IEC_VAL(wp.EN) = true;
    IEC_VAL(wp.AXISID) = 0;
    IEC_VAL(wp.PARAMETERNUMBER) = HYD_PARAM_PRESSURE_KP;
    IEC_VAL(wp.VALUE) = 2.5;
    IEC_VAL(wp.EXECUTE) = true;

    __mcl_cmd_WriteParameter(&wp);

    ASSERT_TRUE(IEC_VAL(wp.DONE) == true, "WriteParameter should set DONE");
    ASSERT_TRUE(IEC_VAL(wp.ERROR) == false, "WriteParameter should not set ERROR");

    /* Reset EXECUTE for next call */
    IEC_VAL(wp.EXECUTE) = false;
    __mcl_cmd_WriteParameter(&wp);

    HYD_READPARAMETER rp;
    memset(&rp, 0, sizeof(rp));
    IEC_VAL(rp.EN) = true;
    IEC_VAL(rp.AXISID) = 0;
    IEC_VAL(rp.ENABLE) = true;
    IEC_VAL(rp.PARAMETERNUMBER) = HYD_PARAM_PRESSURE_KP;

    __mcl_cmd_ReadParameter(&rp);

    ASSERT_TRUE(IEC_VAL(rp.VALID) == true, "ReadParameter after write should set VALID");
    ASSERT_FLOAT_EQ((HYD_REAL)IEC_VAL(rp.VALUE), 2.5f, 0.001f, "pressureKp should be 2.5 after write");
}

/* Test: WriteBoolParameter then ReadBoolParameter through IEC FBs */
static void test_write_read_bool_iec(void) {
    __HydMotion_framework_Init();
    ensure_axis_allocated();

    HYD_WRITEBOOLPARAMETER wbp;
    memset(&wbp, 0, sizeof(wbp));
    IEC_VAL(wbp.EN) = true;
    IEC_VAL(wbp.AXISID) = 0;
    IEC_VAL(wbp.PARAMETERNUMBER) = HYD_PARAM_USE_SIMULATION;
    IEC_VAL(wbp.VALUE) = true;
    IEC_VAL(wbp.EXECUTE) = true;

    __mcl_cmd_WriteBoolParameter(&wbp);

    ASSERT_TRUE(IEC_VAL(wbp.DONE) == true, "WriteBoolParameter should set DONE");

    HYD_READBOOLPARAMETER rbp;
    memset(&rbp, 0, sizeof(rbp));
    IEC_VAL(rbp.EN) = true;
    IEC_VAL(rbp.AXISID) = 0;
    IEC_VAL(rbp.ENABLE) = true;
    IEC_VAL(rbp.PARAMETERNUMBER) = HYD_PARAM_USE_SIMULATION;

    __mcl_cmd_ReadBoolParameter(&rbp);

    ASSERT_TRUE(IEC_VAL(rbp.VALID) == true, "ReadBoolParameter should set VALID");
    ASSERT_TRUE(IEC_VAL(rbp.VALUE) == true, "useSimulation should be true after write");
}

/* Test: Invalid AXISID returns ERROR */
static void test_invalid_axisid_iec(void) {
    __HydMotion_framework_Init();

    HYD_READPARAMETER rp;
    memset(&rp, 0, sizeof(rp));
    IEC_VAL(rp.EN) = true;
    IEC_VAL(rp.AXISID) = -1;
    IEC_VAL(rp.ENABLE) = true;

    __mcl_cmd_ReadParameter(&rp);

    ASSERT_TRUE(IEC_VAL(rp.ERROR) == true, "ReadParameter with invalid AXISID should set ERROR");
    ASSERT_TRUE(IEC_VAL(rp.VALID) == false, "ReadParameter with invalid AXISID should not set VALID");
}

/* Test: Segment builder uses FB params after WriteParameter */
static void test_segment_builder_uses_fb_params(void) {
    __HydMotion_framework_Init();
    ensure_axis_allocated();

    /* Write custom tolerance */
    HYD_WRITEPARAMETER wp;
    memset(&wp, 0, sizeof(wp));
    IEC_VAL(wp.EN) = true;
    IEC_VAL(wp.AXISID) = 0;
    IEC_VAL(wp.PARAMETERNUMBER) = HYD_PARAM_POSITION_TOLERANCE;
    IEC_VAL(wp.VALUE) = 0.025;
    IEC_VAL(wp.EXECUTE) = true;
    __mcl_cmd_WriteParameter(&wp);

    /* Verify the FB's _params was updated */
    HYD_MotionControlFB* fb = __MK_GetPublic_MotionControlFB(0);
    ASSERT_TRUE(fb != NULL, "FB should exist");
    ASSERT_FLOAT_EQ(fb->_params.positionTolerance, 0.025f, 0.001f,
                    "FB _params.positionTolerance should be 0.025 after WriteParameter");
}

int main(void) {
    test_read_parameter_iec();
    test_write_then_read_iec();
    test_write_read_bool_iec();
    test_invalid_axisid_iec();
    test_segment_builder_uses_fb_params();

    printf("IEC parameter FB tests: %d/%d passed\n", tests_passed, tests_run);
    return (tests_passed == tests_run) ? 0 : 1;
}
```

- [ ] **Step 2: Add test target to CMakeLists.txt**

```cmake
add_executable(test_parameter_iec tests/test_parameter_iec.c)
target_link_libraries(test_parameter_iec PRIVATE HydroMotionLib)
```

And add:

```cmake
add_test(NAME test_parameter_iec COMMAND test_parameter_iec)
```

- [ ] **Step 3: Configure, build, run**

```bash
cmake --preset unixgcc
cmake --build --preset unixgcc --target test_parameter_iec
./out/build/unixgcc/test_parameter_iec
```

Expected: all tests pass.

---

### Task 9: Update pousHydMotion.xml with 4 new POU definitions

**Files:**
- Modify: `pousHydMotion.xml`

- [ ] **Step 1: Add 4 new POU definitions to the XML**

After the `HYD_ReadSimFeedback` POU closing `</pou>` tag and before `</pous>`, add:

```xml
			<pou name="HYD_ReadParameter" pouType="functionBlock">
				<interface>
					<inputVars>
						<variable name="EN">
							<type><BOOL /></type>
						</variable>
						<variable name="ENO">
							<type><BOOL /></type>
						</variable>
						<variable name="AXISID">
							<type><SINT /></type>
						</variable>
						<variable name="ENABLE">
							<type><BOOL /></type>
						</variable>
						<variable name="PARAMETERNUMBER">
							<type><INT /></type>
						</variable>
					</inputVars>
					<outputVars>
						<variable name="VALID">
							<type><BOOL /></type>
						</variable>
						<variable name="BUSY">
							<type><BOOL /></type>
						</variable>
						<variable name="ERROR">
							<type><BOOL /></type>
						</variable>
						<variable name="ERRORID">
							<type><WORD /></type>
						</variable>
						<variable name="VALUE">
							<type><LREAL /></type>
						</variable>
					</outputVars>
					<localVars>
						<variable name="ENABLE0">
							<type><BOOL /></type>
						</variable>
					</localVars>
				</interface>
				<body>
					<ST><![CDATA[{{ extern void __mcl_cmd_ReadParameter(HYD_READPARAMETER*); __mcl_cmd_ReadParameter(data__); }}]]></ST>
				</body>
			</pou>
			<pou name="HYD_WriteParameter" pouType="functionBlock">
				<interface>
					<inputVars>
						<variable name="EN">
							<type><BOOL /></type>
						</variable>
						<variable name="ENO">
							<type><BOOL /></type>
						</variable>
						<variable name="AXISID">
							<type><SINT /></type>
						</variable>
						<variable name="EXECUTE">
							<type><BOOL /></type>
						</variable>
						<variable name="PARAMETERNUMBER">
							<type><INT /></type>
						</variable>
						<variable name="VALUE">
							<type><LREAL /></type>
						</variable>
					</inputVars>
					<outputVars>
						<variable name="DONE">
							<type><BOOL /></type>
						</variable>
						<variable name="BUSY">
							<type><BOOL /></type>
						</variable>
						<variable name="ERROR">
							<type><BOOL /></type>
						</variable>
						<variable name="ERRORID">
							<type><WORD /></type>
						</variable>
					</outputVars>
					<localVars>
						<variable name="EXECUTE0">
							<type><BOOL /></type>
						</variable>
						<variable name="DONE0">
							<type><BOOL /></type>
						</variable>
					</localVars>
				</interface>
				<body>
					<ST><![CDATA[{{ extern void __mcl_cmd_WriteParameter(HYD_WRITEPARAMETER*); __mcl_cmd_WriteParameter(data__); }}]]></ST>
				</body>
			</pou>
			<pou name="HYD_ReadBoolParameter" pouType="functionBlock">
				<interface>
					<inputVars>
						<variable name="EN">
							<type><BOOL /></type>
						</variable>
						<variable name="ENO">
							<type><BOOL /></type>
						</variable>
						<variable name="AXISID">
							<type><SINT /></type>
						</variable>
						<variable name="ENABLE">
							<type><BOOL /></type>
						</variable>
						<variable name="PARAMETERNUMBER">
							<type><INT /></type>
						</variable>
					</inputVars>
					<outputVars>
						<variable name="VALID">
							<type><BOOL /></type>
						</variable>
						<variable name="BUSY">
							<type><BOOL /></type>
						</variable>
						<variable name="ERROR">
							<type><BOOL /></type>
						</variable>
						<variable name="ERRORID">
							<type><WORD /></type>
						</variable>
						<variable name="VALUE">
							<type><BOOL /></type>
						</variable>
					</outputVars>
					<localVars>
						<variable name="ENABLE0">
							<type><BOOL /></type>
						</variable>
					</localVars>
				</interface>
				<body>
					<ST><![CDATA[{{ extern void __mcl_cmd_ReadBoolParameter(HYD_READBOOLPARAMETER*); __mcl_cmd_ReadBoolParameter(data__); }}]]></ST>
				</body>
			</pou>
			<pou name="HYD_WriteBoolParameter" pouType="functionBlock">
				<interface>
					<inputVars>
						<variable name="EN">
							<type><BOOL /></type>
						</variable>
						<variable name="ENO">
							<type><BOOL /></type>
						</variable>
						<variable name="AXISID">
							<type><SINT /></type>
						</variable>
						<variable name="EXECUTE">
							<type><BOOL /></type>
						</variable>
						<variable name="PARAMETERNUMBER">
							<type><INT /></type>
						</variable>
						<variable name="VALUE">
							<type><BOOL /></type>
						</variable>
					</inputVars>
					<outputVars>
						<variable name="DONE">
							<type><BOOL /></type>
						</variable>
						<variable name="BUSY">
							<type><BOOL /></type>
						</variable>
						<variable name="ERROR">
							<type><BOOL /></type>
						</variable>
						<variable name="ERRORID">
							<type><WORD /></type>
						</variable>
					</outputVars>
					<localVars>
						<variable name="EXECUTE0">
							<type><BOOL /></type>
						</variable>
						<variable name="DONE0">
							<type><BOOL /></type>
						</variable>
					</localVars>
				</interface>
				<body>
					<ST><![CDATA[{{ extern void __mcl_cmd_WriteBoolParameter(HYD_WRITEBOOLPARAMETER*); __mcl_cmd_WriteBoolParameter(data__); }}]]></ST>
				</body>
			</pou>
```

- [ ] **Step 2: Build — XML changes don't affect C compilation, but verify nothing broke**

```bash
cmake --build --preset unixgcc
```

Expected: builds successfully (XML not compiled, just verified for consistency).

---

### Task 10: Run all tests and verify no regressions

**Files:** None (verification only)

- [ ] **Step 1: Run full test suite**

```bash
ctest --test-dir out/build/unixgcc --output-on-failure
```

Expected: all existing tests pass, plus new `test_parameter_access` and `test_parameter_iec` pass.

- [ ] **Step 2: Run manual end-to-end check**

```bash
./out/build/unixgcc/main
```

Expected: runs without errors.

- [ ] **Step 3: Commit all changes**

```bash
git add include/common_types.h include/motion_control.h include/motion_interface.h
git add src/motion_control.c src/motion_interface.c
git add tests/test_parameter_access.c tests/test_parameter_iec.c
git add CMakeLists.txt pousHydMotion.xml
git commit -m "feat: add HYD_ReadParameter/WriteParameter/ReadBoolParameter/WriteBoolParameter FBs

Add 4 IEC function blocks for reading/writing MotionControlFB instance
parameters by integer ID. Group 25 tunable parameters into HYD_MotionFBParams
struct. Fix hardcoded literals in segment builders to read from FB defaults.

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>"
```
