# HYD_ReadParameter / HYD_WriteParameter — IEC Parameter Access FBs

## Summary

Add 4 IEC function blocks (`HYD_ReadParameter`, `HYD_WriteParameter`, `HYD_ReadBoolParameter`, `HYD_WriteBoolParameter`) that allow the PLC layer to read and write individual MotionControlFB instance parameters by integer ID. Fix the hardcoded literals in `motion_interface.c` segment builders to read from the FB's stored parameter defaults instead.

## Motivation

Currently many motion parameters (tolerances, gains, limits) are hardcoded in `motion_interface.c` segment builders. After `HYD_CreateMotion` creates an axis, there is no IEC-level mechanism to tune these values — they can only be set via recipe segments. The new FBs provide runtime parameter access for commissioning, adaptive tuning, and HMI integration.

## Design

### Approach 1: Grouped parameter struct with enum-driven access

All tunable parameters are grouped into a single `HYD_MotionFBParams` struct, stored as a single `_params` field on `HYD_MotionControlFB`. A `HYD_ParameterNumber` enum maps integer IDs to struct members. The read/write accessor functions switch on the enum to access the right field. Init zero-fills the struct then sets defaults, keeping the FB clean with one field instead of 22.

### HYD_MotionFBParams struct

```c
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
    HYD_PARAM_USE_SIMULATION,       // BOOL parameter
    HYD_PARAM_COUNT
} HYD_ParameterNumber;

typedef struct {
    HYD_REAL positionTolerance;              // mm, default 0.1
    HYD_REAL velocityTolerance;              // mm/s, default 5.0
    HYD_REAL flowTolerance;                  // L/min, default 1.0
    HYD_REAL pressureTolerance;              // MPa, default 0.5
    HYD_REAL timeoutLimit;                   // s, default 30.0
    HYD_REAL velocityToFlowGain;             // L/min per mm/s, default 0.2
    HYD_REAL maxVelocity;                    // mm/s, default 100.0
    HYD_REAL maxAcceleration;                // mm/s², default 500.0
    HYD_REAL maxDeceleration;                // mm/s², default 500.0
    HYD_REAL maxFlow;                        // L/min, default 50.0
    HYD_REAL pressureRampRate;               // MPa/s, default 10.0
    HYD_REAL pressureKp;                     // L/min per MPa, default 0.5
    HYD_REAL pressureKpHigh;                 // L/min per MPa, default 0.0 (disabled)
    HYD_REAL pressureGainBand;               // ratio, default 0.2
    HYD_REAL pressureKi;                     // L/min per (MPa*s), default 0.1
    HYD_REAL pressureKd;                     // L/min per (MPa/s), default 0.0
    HYD_REAL pressureIntegralLimit;          // L/min, default 10.0
    HYD_REAL pressureDeadband;               // MPa, default 0.5
    HYD_REAL pressureFilterAlpha;            // 0<alpha<=1, default 0.5
    HYD_REAL pressureDerivativeFilterAlpha;  // 0<alpha<=1, default 0.5
    HYD_REAL flowToPumpSpeedGain;            // rpm per L/min, default 20.0
    HYD_REAL pumpSpeedLimit;                 // rpm, default 1800.0
    HYD_REAL pressureControllerType;         // cast from HYD_PressureControllerType, default PI
    HYD_REAL defaultTargetFlow;              // L/min, default 5.0
    HYD_BOOL useSimulation;                  // default false
} HYD_MotionFBParams;
```

### HYD_MotionControlFB change

The FB gets a single new field: `HYD_MotionFBParams _params;`

The existing `FLOW_TO_PUMP_SPEED_GAIN`, `PUMP_SPEED_LIMIT`, and `_useSimulation` fields remain on the FB for backward compatibility; they are synced from `_params` on Init and whenever `_params` is written via the parameter FBs.

### PARAMETERNUMBER mapping table

| ID | Field | Type |
|----|-------|------|
| 0 | `_params.positionTolerance` | REAL |
| 1 | `_params.velocityTolerance` | REAL |
| 2 | `_params.flowTolerance` | REAL |
| 3 | `_params.pressureTolerance` | REAL |
| 4 | `_params.timeoutLimit` | REAL |
| 5 | `_params.velocityToFlowGain` | REAL |
| 6 | `_params.maxVelocity` | REAL |
| 7 | `_params.maxAcceleration` | REAL |
| 8 | `_params.maxDeceleration` | REAL |
| 9 | `_params.maxFlow` | REAL |
| 10 | `_params.pressureRampRate` | REAL |
| 11 | `_params.pressureKp` | REAL |
| 12 | `_params.pressureKpHigh` | REAL |
| 13 | `_params.pressureGainBand` | REAL |
| 14 | `_params.pressureKi` | REAL |
| 15 | `_params.pressureKd` | REAL |
| 16 | `_params.pressureIntegralLimit` | REAL |
| 17 | `_params.pressureDeadband` | REAL |
| 18 | `_params.pressureFilterAlpha` | REAL |
| 19 | `_params.pressureDerivativeFilterAlpha` | REAL |
| 20 | `_params.flowToPumpSpeedGain` | REAL |
| 21 | `_params.pumpSpeedLimit` | REAL |
| 22 | `_params.pressureControllerType` | REAL |
| 23 | `_params.defaultTargetFlow` | REAL |
| 24 | `_params.useSimulation` | BOOL |

### New IEC Function Blocks

All 4 FBs follow the same pattern as existing FBs: AXISID selects the FB instance, ENABLE/EXECUTE trigger the operation.

**HYD_ReadParameter** — read a REAL parameter by ID
- IN: EN(BOOL), ENO(BOOL), AXISID(SINT), ENABLE(BOOL), PARAMETERNUMBER(INT)
- OUT: VALID(BOOL), BUSY(BOOL), ERROR(BOOL), ERRORID(WORD), VALUE(LREAL)

**HYD_WriteParameter** — write a REAL parameter by ID
- IN: EN(BOOL), ENO(BOOL), AXISID(SINT), EXECUTE(BOOL), PARAMETERNUMBER(INT), VALUE(LREAL)
- OUT: DONE(BOOL), BUSY(BOOL), ERROR(BOOL), ERRORID(WORD)

**HYD_ReadBoolParameter** — read a BOOL parameter by ID
- IN: EN(BOOL), ENO(BOOL), AXISID(SINT), ENABLE(BOOL), PARAMETERNUMBER(INT)
- OUT: VALID(BOOL), BUSY(BOOL), ERROR(BOOL), ERRORID(WORD), VALUE(BOOL)

**HYD_WriteBoolParameter** — write a BOOL parameter by ID
- IN: EN(BOOL), ENO(BOOL), AXISID(SINT), EXECUTE(BOOL), PARAMETERNUMBER(INT), VALUE(BOOL)
- OUT: DONE(BOOL), BUSY(BOOL), ERROR(BOOL), ERRORID(WORD)

### Segment Builder Changes

All 4 segment builders (`buildPositionSegment`, `buildVelocitySegment`, `buildPressureSegment`, `buildSegmentFromMotion`) take an additional `const HYD_MotionControlFB* fb` parameter. Where they currently use hardcoded literals, they instead read from `fb->_params.<field>`. The `buildSegmentFromMotion` builder (used by MoveProfile) reads from FB defaults for fields not provided by the MOTION struct.

### Accessor Functions (motion_control.c)

```c
HYD_BOOL HYD_MotionControlFB_ReadParameter(const HYD_MotionControlFB* fb, int paramNumber, HYD_REAL* value);
HYD_BOOL HYD_MotionControlFB_WriteParameter(HYD_MotionControlFB* fb, int paramNumber, HYD_REAL value);
HYD_BOOL HYD_MotionControlFB_ReadBoolParameter(const HYD_MotionControlFB* fb, int paramNumber, HYD_BOOL* value);
HYD_BOOL HYD_MotionControlFB_WriteBoolParameter(HYD_MotionControlFB* fb, int paramNumber, HYD_BOOL value);
```

Return `false` on invalid PARAMETERNUMBER (out of range) or type mismatch (calling WriteParameter for ID 24, or WriteBoolParameter for IDs 0-23). After a successful `WriteParameter` or `WriteBoolParameter`, sync the corresponding legacy FB fields (`FLOW_TO_PUMP_SPEED_GAIN`, `PUMP_SPEED_LIMIT`, `_useSimulation`) for backward compatibility.

### Init Defaults

`HYD_MotionControlFB_Init()` zero-fills the entire `_params` struct via memset (since it zero-fills the whole FB), then sets each field to the default values matching the current hardcoded literals.

### Error Handling

- Invalid AXISID → ERROR=true, ERRORID=HYD_DIAG_CODE_START_CONTEXT_INVALID
- Invalid PARAMETERNUMBER (out of range) → ERROR=true, ERRORID=HYD_DIAG_CODE_SEGMENT_INVALID
- Writing a BOOL parameter through HYD_WriteParameter → ERROR=true, ERRORID=HYD_DIAG_CODE_SEGMENT_INVALID
- Writing a REAL parameter through HYD_WriteBoolParameter → ERROR=true, ERRORID=HYD_DIAG_CODE_SEGMENT_INVALID

## Files Changed

| File | Change |
|------|--------|
| `include/common_types.h` | Add `HYD_ParameterNumber` enum and `HYD_MotionFBParams` struct |
| `include/motion_control.h` | Add `_params` field to HYD_MotionControlFB; add 4 accessor function declarations |
| `src/motion_control.c` | Init defaults in Init(); implement 4 accessor functions; sync legacy fields on write |
| `include/motion_interface.h` | Add 4 new FB typedefs (HYD_READPARAMETER, HYD_WRITEPARAMETER, HYD_READBOOLPARAMETER, HYD_WRITEBOOLPARAMETER) |
| `src/motion_interface.c` | Implement 4 new `__mcl_cmd_*` functions; update segment builders to read from `fb->_params` |
| `pousHydMotion.xml` | Add 4 new POU definitions |

## Testing

- Unit tests for parameter read/write accessors (valid/invalid param numbers, type mismatches)
- Unit tests for segment builders reading FB defaults instead of hardcoded values
- Unit tests verifying legacy field sync after WriteParameter for IDs 20, 21, 24
- Integration test: CreateMotion → WriteParameter → MoveAbsolute verifies parameter takes effect
