# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Commands

All commands run from the repository root.

**Configure & Build:**
```bash
cmake --preset unixgcc                      # configure (re-run after adding new src/*.c files)
cmake --build --preset unixgcc              # build all targets
cmake --build out/build/unixgcc --target <name>  # build single target
```

**Test:**
```bash
ctest --test-dir out/build/unixgcc --output-on-failure           # run all tests
ctest --test-dir out/build/unixgcc -R '^test_motion_planner$' --output-on-failure  # run one test
./out/build/unixgcc/main                  # quick manual end-to-end check
./out/build/unixgcc/test_hydro_sim_fb     # simulator PLC adapter test
```

**Embedded production build (excludes simulator):**
```bash
./scripts/deploy_embedded_prod.sh
```

## Architecture

This is a C99 motion-control library for hydraulic injection-molding machines. The fundamental design boundary: **the external process layer owns machine sequencing, valve logic, and segment-switch decisions; this library owns only motion math, pressure/flow planning, pump-speed conversion, and diagnostics**.

### Layer Stack

```
IEC61131-3 PLC Program (matiec/Beremiz compiled)
  │  calls PLCopen function blocks via __mcl_cmd_* functions
  ▼
┌─────────────────────────────────────────────┐
│  IEC Interface Adapter (motion_interface.c) │
│  - FB instance pool (up to 20 axes)         │
│  - Command arbitration with generation      │
│    counters for multi-FB-per-axis safety    │
│  - Bidirectional HDY_AXISMOTION channel     │
│  - Framework: Init/Cleanup/Retrieve/Publish │
├─────────────────────────────────────────────┤
│  Core Library (HydroMotionLib)              │
│  ┌─────────────────────────────────────┐    │
│  │  HDY_MotionControlFB (orchestrator) │    │
│  │  - State machine, command dispatch  │    │
│  │  - Glue between feedback, planner,  │    │
│  │    pressure loop, diagnostics       │    │
│  ├─────────────────────────────────────┤    │
│  │  motion_planner  - velocity/flow    │    │
│  │  pressure_controller - P/PI/PID     │    │
│  │  pump_converter  - flow→rpm         │    │
│  │  ramp_controller - pressure target  │    │
│  │    smoothing                        │    │
│  │  segment_completion - end detection │    │
│  │  diagnostics* - alarm/fault system  │    │
│  └─────────────────────────────────────┘    │
└─────────────────────────────────────────────┘
         │
         ▼ PUMP_SPEED (rpm, nonnegative)
┌─────────────────────────────────────────────┐
│  Simulator (HydroSimLib, dev/test only)      │
│  ┌───────────────────────────────────────┐  │
│  │  L1: hydro_sim_fb.c - PLC adapter     │  │
│  │  createSimAxis / moveSimAxis /        │  │
│  │  readSimAxis IEC function blocks      │  │
│  ├───────────────────────────────────────┤  │
│  │  L2: hydro_sim.c - Physics kernel     │  │
│  │  Single-pump scheduling, cylinder     │  │
│  │  models (CLAMP/INJECT), fault inject  │  │
│  │  ISensorBackend DI abstraction        │  │
│  └───────────────────────────────────────┘  │
└─────────────────────────────────────────────┘
```

### Key Module Responsibilities

| Module | Role |
|---|---|
| `motion_control.c` | State machine orchestrator — `Execute()` runs the cycle: check EN/RESET, handle START_SEGMENT trigger, run pressure ramp, invoke planner, copy outputs, evaluate completion, populate diagnostics |
| `motion_interface.c` | IEC61131-3 bridge — FB pool, axis ownership arbitration, 6 PLCopen FBs (MoveProfile, MoveAbsolute, MoveVelocity, PressureHandle, Stop, Reset) |
| `motion_planner.c` | Motion math — position-mode braking via `sqrt(2*a*s)`, speed-ramp buildup with braking protection, velocity→flow conversion |
| `pressure_controller.c` | P/PI/PID with anti-windup, bumpless tracking, measurement filtering, derivative-rate filtering |
| `segment_completion.c` | Single source of truth for end-condition evaluation (position/time/pressure/flow/manual) |
| `ramp_controller.c` | Smooths pressure target changes at `rampRate * deltaTime` |

### Execution Flow (each scan cycle)

```
AXIS_REF (feedback) → Execute()
  → ramp_controller (smooth pressure target)
  → motion_planner (compute velocity/flow reference)
  → pressure_controller (closed-loop correction)
  → pump_converter (flow → pump rpm)
  → segment_completion (end-condition check)
  → diagnostics (deviation/alarm checks)
  → outputs: PUMP_SPEED, STATE, DIAGNOSTIC, SEGMENT_COMPLETED
```

### PLCopen Function Block Lifecycle

```
Init → LoadRecipe/LoadDirectSegment → StartSegment → Cycle/Scan → Complete → NextSegment (repeat) → Done
                                                                       → Abort (any time)
```

Critical semantics:
- `LoadRecipe()` only loads, never auto-starts; `ACTIVE` stays false
- `START_SEGMENT` / `StartSegment()` is the only entry to running state
- `EN=false` outputs safe zero immediately; re-enabling does NOT auto-resume
- `RESET=true` performs full `memset` — reload config/recipe after reset
- `SEGMENT_CHANGED` is a one-cycle pulse; `SEGMENT_COMPLETED` is latched
- Last segment completion sets `FINISHED=true`

### Recipe vs Direct Mode

- **Recipe mode** (`USE_RECIPE=true`): `StartSegment(index)` uses `RECIPE[index]`, `NextSegment()` advances through multi-segment recipe
- **Direct mode** (`USE_RECIPE=false`): `StartSegment()` ignores index, uses `DIRECT_SEGMENT` as single-shot. `NextSegment()` is rejected. Finishes after one segment.

### Simulator Key Constraints

- Shared `HydraulicSimEnv` is stepped exactly **once per `__HdySimulator_framework_Publish()`** call — multiple FB publishes in the same scan only advance physics by one tick
- Single-pump model: only the axis matching `pump_owner_axis_id` receives flow each step
- CLAMP axis type: includes tie-bar stiffness and mold-obstacle force models
- INJECT axis type: includes melt resistance proportional to position

### matiec IEC Type System

The library uses matiec's IEC61131-3 type infrastructure from `include/matiec/lib/C/`:
- `__DECLARE_VAR(type, name)` declares IEC-typed variables with force-flag write protection
- `__GET_VAR` / `__SET_VAR` read/write through the force-flag guard
- `__DECLARE_STRUCT_TYPE` for custom struct types (e.g., `HDY_AXISMOTION`)
- All base IEC types: `IEC_BOOL`, `IEC_SINT`, `IEC_REAL`, `IEC_WORD`, etc.
- This type system is what makes the library callable from IEC61131-3 PLC runtimes (matiec/Beremiz)

### Control Modes & End Conditions

**Modes**: `HDY_MODE_POSITION` (position convergence), `HDY_MODE_SPEED_RAMP` (velocity ramp), `HDY_MODE_PRESSURE_CLOSED_LOOP` (pressure servo)

**End conditions**: `HDY_END_POSITION`, `HDY_END_TIME`, `HDY_END_PRESSURE`, `HDY_END_FLOW`, `HDY_END_MANUAL`

**Pressure strategies**: P / PI / PID (via `pressure_controller.c`). `RBF_PID` (rbf_pid.c) exists as a standalone module, built and tested but NOT integrated into the main execution path.

### Repository Constraints

1. Pump speed is always nonnegative pump-side magnitude; direction is signaled via `STATE.plannedDirection` — the process layer owns valve actuation
2. `HDY_MotionControlFB_Init()` does full `memset` — resets gains, recipe, runtime state, diagnostics. Reload everything after Init
3. Pressure gain fields (`pressureKp/Ki/Kd`) use zero as "not configured" — zero/negative values fall back to legacy defaults. To disable proportional control, set `pressureController = HDY_PRESSURE_CONTROLLER_NONE`
4. `HDY_DiagnosticCriteria_CheckFaultEscalation()` requires `result.severity == WARNING` and `result.triggered == true` from a prior check call — never pass uninitialized result struct
5. Stay in pure C99, `HDY_` prefix, static bounded memory, no malloc, PLCopen function-block style
6. New `src/*.c` files require re-running `cmake --preset unixgcc` because the build uses `file(GLOB_RECURSE ...)`

### Key Files for Understanding the System

- [tests/main.c](tests/main.c) — simulated multi-segment recipe harness; best starting point for understanding call flow
- [include/motion_control.h](include/motion_control.h) — PLCopen FB struct and full API contract
- [include/common_types.h](include/common_types.h) — all shared types, enums, and data structures
- [include/motion_interface.h](include/motion_interface.h) — IEC PLCopen FB definitions for PLC programs
- [src/motion_interface.c](src/motion_interface.c) — IEC adapter implementation with command arbitration
- [src/motion_control.c](src/motion_control.c) — state machine orchestrator
- [src/sim/hydro_sim.c](src/sim/hydro_sim.c) — physics simulation kernel
- [src/sim/hydro_sim_fb.c](src/sim/hydro_sim_fb.c) — simulator PLC adapter layer
- [include/hdy_config.h](include/hdy_config.h) — compile-time feature flags and platform configuration
