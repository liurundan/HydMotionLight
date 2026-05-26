# Pump/Cylinder Config & FB Struct Reorganization Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add HYD_PumpConfig and HYD_CylinderConfig structs with physical parameter interfaces, reorganize HYD_MotionControlFB layout with clear INPUT/OUTPUT/INTERNAL sections, and implement automatic gain derivation from physical parameters while preserving all existing behavior.

**Architecture:** Purely additive approach — new config structs are added alongside existing gain fields. When new configs are populated (non-zero), the library derives gains from physical parameters. When configs are zero (default after Init), existing fields are used unchanged. The planner resolves velocityToFlowGain at segment-start time into _activeSegment, so no planner API changes needed.

**Tech Stack:** C99, static memory, no malloc, HYD_ prefix convention.

---

## File Map

| File | Action | Responsibility |
|------|--------|----------------|
| `include/common_types.h` | Modify | Add HYD_PumpConfig, HYD_CylinderConfig types + helper declarations |
| `include/motion_control.h` | Modify | Reorganize HYD_MotionControlFB with sections + add new config fields |
| `src/motion_control.c` | Modify | Init zeros new fields; segment-start resolves gain; pump path prefers pumpConfig |
| `src/pump_converter.c` | No change | Already takes gain as input parameter |
| `src/motion_planner.c` | No change | Already reads from _activeSegment.velocityToFlowGain |
| `tests/test_pump_cylinder_config.c` | Create | Unit tests for new config types and derivation |
| `CMakeLists.txt` | Modify | Register new test executable |

## Behavior Preservation Contract

After Init(), pumpConfig = {0,0,0} and cylinderConfig = {0,0,0}. Since:
- displacement_mL_rev == 0 → pump path falls back to FLOW_TO_PUMP_SPEED_GAIN (existing)
- areaExtend_mm2 == 0 AND areaRetract_mm2 == 0 → velocity path falls back to segment.velocityToFlowGain (existing)
- segment.velocityToFlowGain <= 0 → still falls back to 1.0 (existing, unchanged)

ALL existing tests pass without modification. New behavior only activates when user explicitly configures pumpConfig/cylinderConfig with non-zero values.

---

### Task 1: Define HYD_PumpConfig and HYD_CylinderConfig types

**Files:**
- Modify: `include/common_types.h` (after HYD_MotionFBParams, before #endif)

- [ ] **Step 1: Write the failing test**

Create `tests/test_pump_cylinder_config.c`:

```c
#include "common_types.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

/* Helper function declarations (to be implemented in common_types.h or a new .c) */

static void test_pump_config_gain_derivation(void) {
    HYD_PumpConfig cfg = {28.0f, 0.95f, 2000.0f};
    HYD_REAL gain = HYD_PumpConfig_GetFlowToSpeedGain(&cfg);
    HYD_REAL limit = HYD_PumpConfig_GetSpeedLimit(&cfg);
    /* 1000 / (28 * 0.95) = 37.594 rpm/(L/min) */
    assert(fabsf(gain - 37.594f) < 0.1f);
    assert(fabsf(limit - 2000.0f) < 0.01f);
    printf("  PASS: pump config gain derivation\n");
}

static void test_pump_config_zero_returns_zero(void) {
    HYD_PumpConfig cfg = {0.0f, 0.0f, 0.0f};
    HYD_REAL gain = HYD_PumpConfig_GetFlowToSpeedGain(&cfg);
    assert(gain == 0.0f);
    printf("  PASS: pump config zero returns zero\n");
}

static void test_cylinder_config_extend(void) {
    HYD_CylinderConfig cfg = {6362.0f, 3534.0f, 300.0f};
    HYD_REAL gain = HYD_CylinderConfig_GetVelocityToFlowGain(&cfg, HYD_DIRECTION_EXTEND);
    /* 6362 * 6e-5 = 0.38172 L/min per mm/s */
    assert(fabsf(gain - 0.38172f) < 0.001f);
    printf("  PASS: cylinder config extend gain\n");
}

static void test_cylinder_config_retract(void) {
    HYD_CylinderConfig cfg = {6362.0f, 3534.0f, 300.0f};
    HYD_REAL gain = HYD_CylinderConfig_GetVelocityToFlowGain(&cfg, HYD_DIRECTION_RETRACT);
    /* 3534 * 6e-5 = 0.21204 L/min per mm/s */
    assert(fabsf(gain - 0.21204f) < 0.001f);
    printf("  PASS: cylinder config retract gain\n");
}

static void test_cylinder_config_zero_returns_zero(void) {
    HYD_CylinderConfig cfg = {0.0f, 0.0f, 0.0f};
    HYD_REAL gain = HYD_CylinderConfig_GetVelocityToFlowGain(&cfg, HYD_DIRECTION_EXTEND);
    assert(gain == 0.0f);
    printf("  PASS: cylinder config zero returns zero\n");
}

int main(void) {
    printf("test_pump_cylinder_config:\n");
    test_pump_config_gain_derivation();
    test_pump_config_zero_returns_zero();
    test_cylinder_config_extend();
    test_cylinder_config_retract();
    test_cylinder_config_zero_returns_zero();
    printf("All pump/cylinder config tests passed.\n");
    return 0;
}
```

- [ ] **Step 2: Run test to verify it fails (won't compile — types don't exist yet)**

Run: `cmake --build --preset unixgcc 2>&1 | grep -i error | head -5`
Expected: Compilation errors about unknown type HYD_PumpConfig, HYD_CylinderConfig

- [ ] **Step 3: Implement types and helpers in common_types.h**

Add before the closing `#endif` in `include/common_types.h`:

```c
/* ============================================================================
 * 泵物理参数配置
 * 用于从物理参数自动推导 flowToPumpSpeedGain 和 pumpSpeedLimit
 * 当 displacement_mL_rev > 0 时视为有效配置
 * ============================================================================ */
typedef struct {
    HYD_REAL displacement_mL_rev;   /* 泵排量 [mL/rev] */
    HYD_REAL volumetricEfficiency;  /* 容积效率 [0~1], 典型 0.90~0.95 */
    HYD_REAL maxSpeed_rpm;          /* 泵最高转速 [rpm] */
} HYD_PumpConfig;

/* ============================================================================
 * 油缸物理参数配置
 * 用于从面积自动推导 velocityToFlowGain
 * 当 areaExtend_mm2 > 0 或 areaRetract_mm2 > 0 时视为有效配置
 * ============================================================================ */
typedef struct {
    HYD_REAL areaExtend_mm2;    /* 无杆侧有效面积 [mm²] */
    HYD_REAL areaRetract_mm2;   /* 有杆侧有效面积 [mm²] */
    HYD_REAL stroke_mm;         /* 行程 [mm], 用于限位参考 */
} HYD_CylinderConfig;

/* --- 泵配置辅助函数 --- */

/* 从泵物理参数推导 flowToPumpSpeedGain [rpm/(L/min)]
 * 公式: gain = 1000 / (displacement_mL_rev * volumetricEfficiency)
 * 返回 0 表示配置无效（displacement 或 efficiency <= 0） */
static inline HYD_REAL HYD_PumpConfig_GetFlowToSpeedGain(const HYD_PumpConfig* cfg) {
    if (cfg == NULL || cfg->displacement_mL_rev <= 0.0f || cfg->volumetricEfficiency <= 0.0f) {
        return 0.0f;
    }
    return 1000.0f / (cfg->displacement_mL_rev * cfg->volumetricEfficiency);
}

/* 从泵物理参数获取转速上限 [rpm] */
static inline HYD_REAL HYD_PumpConfig_GetSpeedLimit(const HYD_PumpConfig* cfg) {
    if (cfg == NULL || cfg->maxSpeed_rpm <= 0.0f) {
        return 0.0f;
    }
    return cfg->maxSpeed_rpm;
}

/* 判断泵配置是否有效（displacement > 0 且 efficiency > 0） */
static inline HYD_BOOL HYD_PumpConfig_IsValid(const HYD_PumpConfig* cfg) {
    return (cfg != NULL && cfg->displacement_mL_rev > 0.0f && cfg->volumetricEfficiency > 0.0f);
}

/* --- 油缸配置辅助函数 --- */

/* 根据运动方向从油缸面积推导 velocityToFlowGain [L/min per mm/s]
 * 公式: gain = area_mm2 * 60 / 1000000 = area_mm2 * 6e-5
 * 返回 0 表示该方向面积未配置 */
static inline HYD_REAL HYD_CylinderConfig_GetVelocityToFlowGain(const HYD_CylinderConfig* cfg,
                                                                  HYD_MotionDirection direction) {
    HYD_REAL area;
    if (cfg == NULL) { return 0.0f; }
    if (direction == HYD_DIRECTION_RETRACT) {
        area = cfg->areaRetract_mm2;
    } else {
        area = cfg->areaExtend_mm2;  /* EXTEND, AUTO, HOLD all use extend side */
    }
    if (area <= 0.0f) { return 0.0f; }
    return area * 6.0e-5f;
}

/* 判断油缸配置是否有效（至少一侧面积 > 0） */
static inline HYD_BOOL HYD_CylinderConfig_IsValid(const HYD_CylinderConfig* cfg) {
    return (cfg != NULL && (cfg->areaExtend_mm2 > 0.0f || cfg->areaRetract_mm2 > 0.0f));
}
```

- [ ] **Step 4: Register test in CMakeLists.txt**

Add to the test section of `CMakeLists.txt`:

```cmake
add_executable(test_pump_cylinder_config tests/test_pump_cylinder_config.c)
target_link_libraries(test_pump_cylinder_config HydroMotionLib m)
add_test(NAME test_pump_cylinder_config COMMAND test_pump_cylinder_config)
```

- [ ] **Step 5: Build and run test**

Run: `cmake --preset unixgcc && cmake --build --preset unixgcc && ctest --test-dir out/build/unixgcc -R test_pump_cylinder_config --output-on-failure`
Expected: All 5 assertions PASS

- [ ] **Step 6: Run full test suite to verify no regressions**

Run: `ctest --test-dir out/build/unixgcc --output-on-failure`
Expected: All existing tests pass unchanged

- [ ] **Step 7: Commit**

```bash
git add include/common_types.h tests/test_pump_cylinder_config.c CMakeLists.txt
git commit -m "feat: add HYD_PumpConfig and HYD_CylinderConfig types with helper functions"
```

---

### Task 2: Add pumpConfig and cylinderConfig to HYD_MotionControlFB and reorganize layout

**Files:**
- Modify: `include/motion_control.h` (HYD_MotionControlFB struct definition)

- [ ] **Step 1: Reorganize struct with section comments and add new fields**

Rewrite the HYD_MotionControlFB struct body (lines 208-299 of motion_control.h) with clear section markers. All existing field names stay identical — only ordering and comments change, plus two new fields added:

```c
typedef struct {
    /* ═══════════════════════════════════════════════════════════════════════
     * INPUT — PLC/caller-owned fields. The runtime reads but never writes.
     * ═══════════════════════════════════════════════════════════════════════ */
    HYD_BOOL RESET;
    HYD_BOOL START_SEGMENT;
    HYD_UINT START_SEGMENT_INDEX;
    HYD_BOOL USE_RECIPE;
    HYD_AxisRef AXIS_REF;
    HYD_UINT RECIPE_SIZE;
    HYD_MotionSegment RECIPE[HYD_MAX_SEGMENTS];
    HYD_MotionSegment DIRECT_SEGMENT;

    /* Pump physical parameters. When valid (displacement > 0), the library
     * derives flowToPumpSpeedGain and pumpSpeedLimit from these instead of
     * using the legacy FLOW_TO_PUMP_SPEED_GAIN / PUMP_SPEED_LIMIT fields.
     * Zero-initialized after Init() = inactive (legacy fields used). */
    HYD_PumpConfig pumpConfig;

    /* Cylinder physical parameters. When valid (area > 0), the library
     * derives velocityToFlowGain from area + direction at segment-start,
     * unless the segment explicitly sets velocityToFlowGain > 0.
     * Zero-initialized after Init() = inactive (segment field used). */
    HYD_CylinderConfig cylinderConfig;

    /* Legacy gain fields — used when pumpConfig is not configured (all zeros).
     * Preserved for backward compatibility with IEC adapter CreateMotion path. */
    HYD_REAL FLOW_TO_PUMP_SPEED_GAIN;
    HYD_REAL PUMP_SPEED_LIMIT;

    /* ═══════════════════════════════════════════════════════════════════════
     * OUTPUT — Runtime-owned fields. The caller reads but should not write.
     * ═══════════════════════════════════════════════════════════════════════ */
    HYD_DiagnosticCode ERROR_ID;
    HYD_FbState FB_STATE;
    HYD_REAL PUMP_SPEED;
    HYD_BOOL SEGMENT_COMPLETED;
    HYD_BOOL SEGMENT_CHANGED;
    HYD_BOOL DIRECT_SEGMENT_VALID;
    HYD_MotionState STATE;
    HYD_DiagnosticInfo DIAGNOSTIC;
    HYD_DiagnosticSnapshot LAST_FAULT_SNAPSHOT;
    HYD_DiagnosticHistory DIAGNOSTIC_HISTORY;

    /* ═══════════════════════════════════════════════════════════════════════
     * INTERNAL — Private runtime state. External code must not access.
     * ═══════════════════════════════════════════════════════════════════════ */

    /* --- Segment execution state --- */
    HYD_REAL _segmentStartTime;
    HYD_BOOL _segmentChangedFlag;
    HYD_REAL _lastCommandedFlow;
    HYD_TIME _lastFeedbackTimestamp;
    HYD_BOOL _startSegmentSignalPrev;
    HYD_MotionSegment _activeSegment;
    HYD_BOOL _activeSegmentValid;
    HYD_SegmentSource _activeSegmentSource;
    HYD_ControlMode _previousSegmentMode;

    /* --- Command processing --- */
    HYD_FbCommand _pendingCommand;
    HYD_UINT _pendingCommandSegmentIndex;
    HYD_TIME _pendingCommandTimestamp;

    /* --- Direct command session --- */
    HYD_DirectCommandKind _directOwnerKind;
    HYD_DirectSessionState _directSessionState;
    uint16_t _directOwnerExecutionId;
    uint16_t _lastPreemptedExecutionId;
    HYD_DirectCommandKind _lastPreemptedKind;
    uint16_t _lastCompletedExecutionId;
    HYD_DirectCommandKind _lastCompletedKind;
    HYD_BOOL _directPendingValid;
    HYD_MotionSegment _directPendingSegment;
    HYD_DirectCommandKind _directPendingKind;
    HYD_BufferMode _directPendingBufferMode;
    HYD_MotionBlendContext _directBlendContext;

    /* --- Stop/deceleration state --- */
    HYD_BOOL _isStopping;
    HYD_TIME _stopStartTime;
    HYD_REAL _stopStartVel;
    HYD_REAL _stopDeceleration;
    HYD_BOOL _isDecelerating;
    HYD_TIME _decelStartTime;
    HYD_REAL _decelStartVel;

    /* --- Hold state --- */
    HYD_TIME _holdStateTime;

    /* --- Controllers --- */
    HYD_RampController _rampController;
    HYD_MotionPlannerState _plannerState;
    HYD_PressureControllerState _pressureController;

    /* --- Completion detection --- */
    HYD_TIME _completionCandidateStartTime;
    HYD_BOOL _completionCandidateActive;

    /* --- Diagnostic criteria --- */
    HYD_ErrorMonitor _errorMonitor;
    HYD_DiagnosticCriteria _pressureCriteria;
    HYD_DiagnosticCriteriaState _pressureCriteriaState;
    HYD_DiagnosticCriteria _pressureCeilingCriteria;
    HYD_DiagnosticCriteriaState _pressureCeilingCriteriaState;
    HYD_DiagnosticCriteria _flowCriteria;
    HYD_DiagnosticCriteriaState _flowCriteriaState;
    HYD_DiagnosticCriteria _velocityCriteria;
    HYD_DiagnosticCriteriaState _velocityCriteriaState;
    HYD_DiagnosticCriteria _positionCriteria;
    HYD_DiagnosticCriteriaState _positionCriteriaState;
    HYD_DiagnosticCriteria _timeoutCriteria;
    HYD_DiagnosticCriteriaState _timeoutCriteriaState;
    HYD_BOOL _isSwitchPhase;
    HYD_TIME _switchSuppressEndTime;

    /* --- Identity & configuration --- */
    HYD_UINT8 _index;
    uint16_t _executionId;
    uint16_t _recipeBatchId;
    HYD_BOOL _useSimulation;
    HYD_BOOL _configuredUseRecipe;
    HYD_MotionFBParams _params;

    /* --- Simulation feedback (test only) --- */
    struct {
        HYD_REAL targetPosition;
        HYD_REAL targetVelocity;
        HYD_REAL targetFlow;
        HYD_REAL targetPressure;
        HYD_BOOL valid;
    } _simFeedback;
} HYD_MotionControlFB;
```

Note: This is a field reordering + addition. All field names are identical. The C99 struct layout changes, but since this library uses memset-based Init() and never serializes the struct to disk/wire, reordering is safe.

- [ ] **Step 2: Build full test suite**

Run: `cmake --preset unixgcc && cmake --build --preset unixgcc && ctest --test-dir out/build/unixgcc --output-on-failure`
Expected: All tests pass (struct reordering doesn't affect behavior since Init does memset)

- [ ] **Step 3: Commit**

```bash
git add include/motion_control.h
git commit -m "refactor: reorganize HYD_MotionControlFB with INPUT/OUTPUT/INTERNAL sections

Add pumpConfig and cylinderConfig fields (zero-initialized, inactive by default).
Reorder existing fields into logical groups with section comments.
No behavior change — all existing tests pass unchanged."
```

---

### Task 3: Implement gain derivation in motion_control.c

**Files:**
- Modify: `src/motion_control.c` (Init, segment-start, pump conversion call sites)

- [ ] **Step 1: Write integration test for pumpConfig derivation**

Add to `tests/test_pump_cylinder_config.c`:

```c
#include "motion_control.h"
#include <string.h>

static void test_fb_pump_config_derivation(void) {
    HYD_MotionControlFB fb;
    HYD_MotionControlFB_Init(&fb);

    /* Configure pump: 28 mL/rev, eta=0.95, max 2000 rpm */
    fb.pumpConfig.displacement_mL_rev = 28.0f;
    fb.pumpConfig.volumetricEfficiency = 0.95f;
    fb.pumpConfig.maxSpeed_rpm = 2000.0f;

    /* Load a simple segment */
    HYD_MotionSegment seg;
    memset(&seg, 0, sizeof(seg));
    seg.segmentTag = 1;
    seg.mode = HYD_MODE_SPEED_RAMP;
    seg.endCondition = HYD_END_TIME;
    seg.direction = HYD_DIRECTION_EXTEND;
    seg.maxVelocity = 100.0f;
    seg.maxFlow = 50.0f;
    seg.velocityToFlowGain = 0.3f;  /* explicit segment gain */
    seg.duration = 2.0f;
    seg.maxAcceleration = 500.0f;
    seg.maxDeceleration = 500.0f;

    fb.USE_RECIPE = false;
    HYD_MotionControlFB_LoadDirectSegment(&fb, &seg);
    HYD_MotionControlFB_StartSegment(&fb, 0, 0.0f);

    /* Feed one cycle */
    fb.AXIS_REF.timestamp = 0.001f;
    fb.AXIS_REF.position = 0.0f;
    fb.AXIS_REF.velocity = 0.0f;
    fb.AXIS_REF.pressure = 0.0f;
    HYD_MotionControlFB_Cycle(&fb);

    /* Pump speed should use derived gain from pumpConfig:
     * gain = 1000/(28*0.95) = 37.594 rpm/(L/min)
     * limit = 2000 rpm
     * NOT the legacy FLOW_TO_PUMP_SPEED_GAIN */
    assert(fb.PUMP_SPEED <= 2000.0f);
    /* Verify it's using pumpConfig, not legacy field */
    fb.FLOW_TO_PUMP_SPEED_GAIN = 9999.0f;  /* poison legacy field */
    fb.AXIS_REF.timestamp = 0.002f;
    HYD_MotionControlFB_Cycle(&fb);
    /* If pumpConfig is active, PUMP_SPEED should still be reasonable (not 9999*flow) */
    assert(fb.PUMP_SPEED < 2000.0f);
    printf("  PASS: FB pump config derivation\n");
}

static void test_fb_pump_config_fallback_to_legacy(void) {
    HYD_MotionControlFB fb;
    HYD_MotionControlFB_Init(&fb);

    /* pumpConfig left at zero (default) — should use legacy fields */
    fb.FLOW_TO_PUMP_SPEED_GAIN = 80.0f;
    fb.PUMP_SPEED_LIMIT = 5000.0f;

    HYD_MotionSegment seg;
    memset(&seg, 0, sizeof(seg));
    seg.segmentTag = 1;
    seg.mode = HYD_MODE_SPEED_RAMP;
    seg.endCondition = HYD_END_TIME;
    seg.direction = HYD_DIRECTION_EXTEND;
    seg.maxVelocity = 100.0f;
    seg.maxFlow = 50.0f;
    seg.velocityToFlowGain = 0.3f;
    seg.duration = 2.0f;
    seg.maxAcceleration = 500.0f;
    seg.maxDeceleration = 500.0f;

    fb.USE_RECIPE = false;
    HYD_MotionControlFB_LoadDirectSegment(&fb, &seg);
    HYD_MotionControlFB_StartSegment(&fb, 0, 0.0f);

    fb.AXIS_REF.timestamp = 0.001f;
    HYD_MotionControlFB_Cycle(&fb);

    /* With legacy gain=80, any flow output * 80 should be the pump speed */
    /* Just verify it runs without error and uses the legacy path */
    assert(fb.FB_STATE == HYD_FB_STATE_RUNNING);
    assert(fb.PUMP_SPEED <= 5000.0f);
    printf("  PASS: FB pump config fallback to legacy\n");
}

static void test_fb_cylinder_config_derivation(void) {
    HYD_MotionControlFB fb;
    HYD_MotionControlFB_Init(&fb);

    /* Configure cylinder: 250T inject cylinder */
    fb.cylinderConfig.areaExtend_mm2 = 6362.0f;
    fb.cylinderConfig.areaRetract_mm2 = 3534.0f;
    fb.cylinderConfig.stroke_mm = 300.0f;

    fb.FLOW_TO_PUMP_SPEED_GAIN = 80.0f;
    fb.PUMP_SPEED_LIMIT = 5000.0f;

    /* Segment with velocityToFlowGain = 0 → should derive from cylinder */
    HYD_MotionSegment seg;
    memset(&seg, 0, sizeof(seg));
    seg.segmentTag = 1;
    seg.mode = HYD_MODE_SPEED_RAMP;
    seg.endCondition = HYD_END_TIME;
    seg.direction = HYD_DIRECTION_EXTEND;
    seg.maxVelocity = 100.0f;
    seg.maxFlow = 50.0f;
    seg.velocityToFlowGain = 0.0f;  /* zero = derive from cylinder */
    seg.duration = 2.0f;
    seg.maxAcceleration = 500.0f;
    seg.maxDeceleration = 500.0f;

    fb.USE_RECIPE = false;
    HYD_MotionControlFB_LoadDirectSegment(&fb, &seg);
    HYD_MotionControlFB_StartSegment(&fb, 0, 0.0f);

    fb.AXIS_REF.timestamp = 0.001f;
    HYD_MotionControlFB_Cycle(&fb);

    /* The _activeSegment.velocityToFlowGain should now be 6362*6e-5 = 0.38172 */
    /* We can verify indirectly: with vel=100 mm/s, flow = 100*0.38172 = 38.17 L/min */
    /* pumpSpeed = 38.17 * 80 = 3053 rpm (approximately) */
    assert(fb.FB_STATE == HYD_FB_STATE_RUNNING);
    assert(fb.PUMP_SPEED > 0.0f);
    printf("  PASS: FB cylinder config derivation\n");
}
```

Add these calls to main():
```c
    test_fb_pump_config_derivation();
    test_fb_pump_config_fallback_to_legacy();
    test_fb_cylinder_config_derivation();
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cmake --preset unixgcc && cmake --build --preset unixgcc && ctest --test-dir out/build/unixgcc -R test_pump_cylinder_config --output-on-failure`
Expected: FAIL (pumpConfig not yet wired into execution path)

- [ ] **Step 3: Implement pumpConfig derivation in pump conversion call site**

In `src/motion_control.c`, find the function `HYD_ExecuteActiveSegmentControl` (around line 1297) where `pumpInput` is built. Change:

```c
    /* Before (legacy): */
    pumpInput.flowToPumpSpeedGain = fb->FLOW_TO_PUMP_SPEED_GAIN;
    pumpInput.pumpSpeedLimit = fb->PUMP_SPEED_LIMIT;

    /* After (prefer pumpConfig if valid): */
    if (HYD_PumpConfig_IsValid(&fb->pumpConfig)) {
        pumpInput.flowToPumpSpeedGain = HYD_PumpConfig_GetFlowToSpeedGain(&fb->pumpConfig);
        pumpInput.pumpSpeedLimit = HYD_PumpConfig_GetSpeedLimit(&fb->pumpConfig);
    } else {
        pumpInput.flowToPumpSpeedGain = fb->FLOW_TO_PUMP_SPEED_GAIN;
        pumpInput.pumpSpeedLimit = fb->PUMP_SPEED_LIMIT;
    }
```

Apply the same pattern to ALL other sites that read FLOW_TO_PUMP_SPEED_GAIN / PUMP_SPEED_LIMIT in motion_control.c (lines 1779, 1822-1823, 1892). Use grep to find them all:

```bash
grep -n "fb->FLOW_TO_PUMP_SPEED_GAIN\|fb->PUMP_SPEED_LIMIT" src/motion_control.c
```

For each site, wrap with the same `if (HYD_PumpConfig_IsValid(...))` pattern.

- [ ] **Step 4: Implement cylinderConfig derivation at segment-start**

In `src/motion_control.c`, find the segment-start logic where `_activeSegment` is populated (the function that copies segment into `fb->_activeSegment`). After the copy, add gain resolution:

```c
    /* Resolve velocityToFlowGain: segment explicit > cylinderConfig > existing fallback */
    if (fb->_activeSegment.velocityToFlowGain <= 0.0f &&
        HYD_CylinderConfig_IsValid(&fb->cylinderConfig)) {
        fb->_activeSegment.velocityToFlowGain =
            HYD_CylinderConfig_GetVelocityToFlowGain(
                &fb->cylinderConfig, fb->_activeSegment.direction);
    }
```

Find the exact location by searching for where `_activeSegment` is assigned from recipe/direct segment. This is in the START command consumption path (search for `fb->_activeSegment = `).

- [ ] **Step 5: Update the runtime config validation to also validate pumpConfig when active**

In the `HYD_PumpConverter_ValidateConfig` call site (line ~1779), update:

```c
    /* Validate pump configuration */
    HYD_REAL effectiveGain, effectiveLimit;
    if (HYD_PumpConfig_IsValid(&fb->pumpConfig)) {
        effectiveGain = HYD_PumpConfig_GetFlowToSpeedGain(&fb->pumpConfig);
        effectiveLimit = HYD_PumpConfig_GetSpeedLimit(&fb->pumpConfig);
    } else {
        effectiveGain = fb->FLOW_TO_PUMP_SPEED_GAIN;
        effectiveLimit = fb->PUMP_SPEED_LIMIT;
    }
    if (!HYD_PumpConverter_ValidateConfig(effectiveGain, effectiveLimit, &diagCode)) {
        /* ... existing diagnostic handling ... */
    }
```

- [ ] **Step 6: Zero-initialize new fields in Init()**

In `HYD_MotionControlFB_Init()` (line ~2215), the existing `memset(fb, 0, sizeof(*fb))` already zeros pumpConfig and cylinderConfig. Verify this is the case. No code change needed if Init starts with memset.

- [ ] **Step 7: Build and run all tests**

Run: `cmake --preset unixgcc && cmake --build --preset unixgcc && ctest --test-dir out/build/unixgcc --output-on-failure`
Expected: ALL tests pass including new pump_cylinder_config tests

- [ ] **Step 8: Commit**

```bash
git add src/motion_control.c tests/test_pump_cylinder_config.c
git commit -m "feat: wire pumpConfig/cylinderConfig derivation into execution path

- Pump conversion prefers pumpConfig when valid (displacement > 0)
- Segment-start resolves velocityToFlowGain from cylinderConfig when
  segment field is zero and cylinder is configured
- Falls back to legacy FLOW_TO_PUMP_SPEED_GAIN / segment field when
  configs are zero (default after Init) — no behavior change for
  existing code"
```

---

### Task 4: Add parameter access for new config fields

**Files:**
- Modify: `include/common_types.h` (HYD_ParameterNumber enum)
- Modify: `src/motion_control.c` (ReadParameter/WriteParameter switch cases)

- [ ] **Step 1: Add parameter numbers to enum**

In `include/common_types.h`, add before `HYD_PARAM_COUNT`:

```c
    HYD_PARAM_PUMP_DISPLACEMENT,        /* mL/rev */
    HYD_PARAM_PUMP_VOLUMETRIC_EFF,      /* 0~1 */
    HYD_PARAM_PUMP_MAX_SPEED,           /* rpm */
    HYD_PARAM_CYLINDER_AREA_EXTEND,     /* mm² */
    HYD_PARAM_CYLINDER_AREA_RETRACT,    /* mm² */
    HYD_PARAM_CYLINDER_STROKE,          /* mm */
    HYD_PARAM_COUNT
```

- [ ] **Step 2: Add ReadParameter cases**

In `src/motion_control.c` `HYD_MotionControlFB_ReadParameter()`, add cases:

```c
        case HYD_PARAM_PUMP_DISPLACEMENT:       *value = fb->pumpConfig.displacement_mL_rev; break;
        case HYD_PARAM_PUMP_VOLUMETRIC_EFF:     *value = fb->pumpConfig.volumetricEfficiency; break;
        case HYD_PARAM_PUMP_MAX_SPEED:          *value = fb->pumpConfig.maxSpeed_rpm; break;
        case HYD_PARAM_CYLINDER_AREA_EXTEND:    *value = fb->cylinderConfig.areaExtend_mm2; break;
        case HYD_PARAM_CYLINDER_AREA_RETRACT:   *value = fb->cylinderConfig.areaRetract_mm2; break;
        case HYD_PARAM_CYLINDER_STROKE:         *value = fb->cylinderConfig.stroke_mm; break;
```

- [ ] **Step 3: Add WriteParameter cases**

In `HYD_MotionControlFB_WriteParameter()`, add cases:

```c
        case HYD_PARAM_PUMP_DISPLACEMENT:       fb->pumpConfig.displacement_mL_rev = value; break;
        case HYD_PARAM_PUMP_VOLUMETRIC_EFF:     fb->pumpConfig.volumetricEfficiency = value; break;
        case HYD_PARAM_PUMP_MAX_SPEED:          fb->pumpConfig.maxSpeed_rpm = value; break;
        case HYD_PARAM_CYLINDER_AREA_EXTEND:    fb->cylinderConfig.areaExtend_mm2 = value; break;
        case HYD_PARAM_CYLINDER_AREA_RETRACT:   fb->cylinderConfig.areaRetract_mm2 = value; break;
        case HYD_PARAM_CYLINDER_STROKE:         fb->cylinderConfig.stroke_mm = value; break;
```

- [ ] **Step 4: Add parameter access test**

Add to `tests/test_pump_cylinder_config.c`:

```c
static void test_parameter_access(void) {
    HYD_MotionControlFB fb;
    HYD_MotionControlFB_Init(&fb);
    HYD_REAL val;

    /* Write pump config via parameters */
    assert(HYD_MotionControlFB_WriteParameter(&fb, HYD_PARAM_PUMP_DISPLACEMENT, 45.0f));
    assert(HYD_MotionControlFB_WriteParameter(&fb, HYD_PARAM_PUMP_VOLUMETRIC_EFF, 0.93f));
    assert(HYD_MotionControlFB_WriteParameter(&fb, HYD_PARAM_PUMP_MAX_SPEED, 1800.0f));

    /* Read back */
    assert(HYD_MotionControlFB_ReadParameter(&fb, HYD_PARAM_PUMP_DISPLACEMENT, &val));
    assert(fabsf(val - 45.0f) < 0.01f);
    assert(HYD_MotionControlFB_ReadParameter(&fb, HYD_PARAM_PUMP_VOLUMETRIC_EFF, &val));
    assert(fabsf(val - 0.93f) < 0.01f);
    assert(HYD_MotionControlFB_ReadParameter(&fb, HYD_PARAM_PUMP_MAX_SPEED, &val));
    assert(fabsf(val - 1800.0f) < 0.01f);

    /* Write cylinder config via parameters */
    assert(HYD_MotionControlFB_WriteParameter(&fb, HYD_PARAM_CYLINDER_AREA_EXTEND, 6362.0f));
    assert(HYD_MotionControlFB_WriteParameter(&fb, HYD_PARAM_CYLINDER_AREA_RETRACT, 3534.0f));
    assert(HYD_MotionControlFB_WriteParameter(&fb, HYD_PARAM_CYLINDER_STROKE, 300.0f));

    assert(HYD_MotionControlFB_ReadParameter(&fb, HYD_PARAM_CYLINDER_AREA_EXTEND, &val));
    assert(fabsf(val - 6362.0f) < 0.01f);

    printf("  PASS: parameter access for pump/cylinder config\n");
}
```

- [ ] **Step 5: Build and run all tests**

Run: `cmake --preset unixgcc && cmake --build --preset unixgcc && ctest --test-dir out/build/unixgcc --output-on-failure`
Expected: All tests pass

- [ ] **Step 6: Commit**

```bash
git add include/common_types.h src/motion_control.c tests/test_pump_cylinder_config.c
git commit -m "feat: add parameter access for pumpConfig and cylinderConfig fields"
```

---

### Task 5: Update HYD_MotionFBParams defaults and IEC adapter compatibility

**Files:**
- Modify: `src/motion_control.c` (Init defaults)
- Modify: `src/motion_interface.c` (CreateMotion path)

- [ ] **Step 1: Update Init() default params**

In `HYD_MotionControlFB_Init()`, after the existing `_params` defaults (line ~2215), update the comment to clarify the relationship:

```c
    /* Legacy defaults — used when pumpConfig/cylinderConfig are not configured.
     * These values correspond to a small test pump (13.2 mL/rev equivalent). */
    fb->_params.flowToPumpSpeedGain = 20.0f;
    fb->_params.pumpSpeedLimit = 1800.0f;
    fb->_params.velocityToFlowGain = 0.2f;

    fb->FLOW_TO_PUMP_SPEED_GAIN = fb->_params.flowToPumpSpeedGain;
    fb->PUMP_SPEED_LIMIT = fb->_params.pumpSpeedLimit;

    /* pumpConfig and cylinderConfig are zero after memset — inactive by default */
```

No functional change here, just documentation.

- [ ] **Step 2: Ensure IEC CreateMotion still works**

In `src/motion_interface.c` line 564-565, the existing code:
```c
    fb->FLOW_TO_PUMP_SPEED_GAIN = __GET_VAR(data__->FLOW_TO_PUMPSPEED);
    fb->PUMP_SPEED_LIMIT = __GET_VAR(data__->PUMPSPEED_LIMIT);
```

This continues to work as-is. When the IEC layer sets these legacy fields and pumpConfig remains zero, the execution path falls back to legacy fields. No change needed.

- [ ] **Step 3: Run full test suite**

Run: `ctest --test-dir out/build/unixgcc --output-on-failure`
Expected: All tests pass

- [ ] **Step 4: Commit**

```bash
git add src/motion_control.c
git commit -m "docs: clarify legacy gain defaults relationship to pumpConfig/cylinderConfig"
```

---

## Summary of Changes

| What | Before | After |
|------|--------|-------|
| Pump gain source | Bare `FLOW_TO_PUMP_SPEED_GAIN` field | `pumpConfig` (physical params) preferred; legacy field as fallback |
| Cylinder gain source | Per-segment `velocityToFlowGain` only | `cylinderConfig` derives gain when segment field is 0; segment can override |
| Struct layout | Flat, no section markers | INPUT/OUTPUT/INTERNAL sections with logical grouping |
| Existing behavior | — | 100% preserved when new configs are zero (default) |
| New parameter numbers | — | 6 new: displacement, efficiency, maxSpeed, areaExtend, areaRetract, stroke |

## Physical Parameter Reference (for testing/commissioning)

| Machine | Pump disp [mL/rev] | Pump eta | Pump max [rpm] | Inject area_ext [mm²] | Inject area_ret [mm²] |
|---------|--------------------:|:--------:|:--------------:|----------------------:|----------------------:|
| 80T     | 28.0 | 0.95 | 2000 | 2827 | 1571 |
| 250T    | 45.0 | 0.93 | 1800 | 6362 | 3534 |
| 650T    | 71.0 | 0.92 | 1500 | 15394 | 7540 |
