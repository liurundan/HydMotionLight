# CLAUDE.md

Guidance for Claude Code when working with this repository.

---

## Coding Behavior

**Bias toward caution over speed. Use judgment for trivial tasks.**

### Think Before Coding
- State assumptions explicitly; if uncertain, ask.
- Present multiple interpretations — don't pick silently.
- If a simpler approach exists, say so and push back when warranted.
- If something is unclear, stop, name what's confusing, and ask.

### Simplicity First
- Minimum code that solves the problem. Nothing speculative.
- No features, abstractions, or "flexibility" beyond what was asked.
- No error handling for impossible scenarios.
- If you write 200 lines and it could be 50, rewrite it.

### Surgical Changes
- Touch only what you must. Don't "improve" adjacent code, comments, or formatting.
- Match existing style, even if you'd do it differently.
- If you notice unrelated dead code, mention it — don't delete it.
- Remove only imports/variables/functions that **your** changes made unused.

### Goal-Driven Execution
Transform tasks into verifiable goals before starting:
- "Fix the bug" → "Write a test that reproduces it, then make it pass"
- "Refactor X" → "Ensure tests pass before and after"

For multi-step tasks, state a brief plan:
```
1. [Step] → verify: [check]
2. [Step] → verify: [check]
```

---

## Commands

All commands run from the repository root.

**Configure & Build:**
```bash
cmake --preset unixgcc                               # configure (re-run after adding new src/*.c files)
cmake --build --preset unixgcc                       # build all targets
cmake --build out/build/unixgcc --target <name>      # build single target
```

**Test:**
```bash
ctest --test-dir out/build/unixgcc --output-on-failure                           # run all tests
ctest --test-dir out/build/unixgcc -R '^test_motion_planner$' --output-on-failure  # run one test
./out/build/unixgcc/main                   # quick manual end-to-end check
./out/build/unixgcc/test_hydro_sim_fb      # simulator PLC adapter test
```

**Embedded production build (excludes simulator):**
```bash
./scripts/deploy_embedded_prod.sh
```

**Coverage:**
```bash
./scripts/coverage.sh           # text report (out/coverage/coverage.txt)
./scripts/coverage.sh --html    # text + HTML report (out/coverage/index.html)
```

---

## Architecture

C99 motion-control library for hydraulic injection-molding machines.

**Design boundary:** the external process layer owns machine sequencing, valve logic, and segment-switch decisions; **this library owns only motion math, pressure/flow planning, pump-speed conversion, and diagnostics**.

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
│  - Bidirectional HYD_AXISMOTION channel     │
│  - Framework: Init/Cleanup/Retrieve/Publish │
├─────────────────────────────────────────────┤
│  Core Library (HydroMotionLib)              │
│  ┌─────────────────────────────────────┐    │
│  │  HYD_MotionControlFB (orchestrator) │    │
│  │  - State machine, command dispatch  │    │
│  │  - Glue: feedback/planner/pressure/ │    │
│  │    diagnostics                      │    │
│  ├─────────────────────────────────────┤    │
│  │  motion_planner  - velocity/flow    │    │
│  │  pressure_controller - P/PI/PID     │    │
│  │  pump_converter  - flow→rpm         │    │
│  │  ramp_controller - pressure target  │    │
│  │  segment_completion - end detection │    │
│  │  diagnostics* - alarm/fault system  │    │
│  └─────────────────────────────────────┘    │
└─────────────────────────────────────────────┘
         │
         ▼ PUMP_SPEED (rpm, nonnegative)
┌─────────────────────────────────────────────┐
│  Simulator (HydroSimLib, dev/test only)     │
│  - L1: hydro_sim_fb.c — PLC adapter         │
│    (createSimAxis / moveSimAxis / readSimAxis)│
│  - L2: hydro_sim.c — Physics kernel         │
│    (single-pump scheduling, CLAMP/INJECT     │
│     cylinder models, fault injection,        │
│     ISensorBackend DI abstraction)           │
└─────────────────────────────────────────────┘
```

### Module Responsibilities

| Module | Role |
|---|---|
| `motion_control.c` | State machine orchestrator — `Execute()`: check EN/RESET, handle START_SEGMENT, run pressure ramp, invoke planner, copy outputs, evaluate completion, populate diagnostics |
| `motion_interface.c` | IEC61131-3 bridge — FB pool, axis ownership arbitration, 6 PLCopen FBs (MoveProfile, MoveAbsolute, MoveVelocity, PressureHandle, Stop, Reset) |
| `motion_planner.c` | Motion math — position-mode braking via `sqrt(2*a*s)`, speed-ramp buildup with braking protection, velocity→flow conversion |
| `pressure_controller.c` | P/PI/PID with anti-windup, bumpless tracking, measurement filtering, derivative-rate filtering |
| `segment_completion.c` | Single source of truth for end-condition evaluation (position/time/pressure/flow/manual) |
| `ramp_controller.c` | Smooths pressure target changes at `rampRate * deltaTime` |
| `vp_transfer.c` | V/P transfer — evaluates position/pressure/time/velocity-drop criteria with configurable priority and optional latch |
| `safety_state_manager.c` | Preserves/restores runtime safety state (ResetRuntimeActuation, ApplyIdleState, ApplyDisabledState, ApplyFaultHold, EnterFaultStop) |
| `diagnostics.c` | Diagnostic code table, code-to-string, alarm/fault severity classification |
| `state_reporter.c` | Execution reporting, FB state transitions, diagnostic event recording |

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

- **Recipe mode** (`USE_RECIPE=true`): `StartSegment(index)` uses `RECIPE[index]`; `NextSegment()` advances through multi-segment recipe
- **Direct mode** (`USE_RECIPE=false`): `StartSegment()` ignores index, uses `DIRECT_SEGMENT` as single-shot; `NextSegment()` is rejected; finishes after one segment

### Control Modes & End Conditions

**Modes:** `HYD_MODE_POSITION` | `HYD_MODE_SPEED_RAMP` | `HYD_MODE_PRESSURE_CLOSED_LOOP`

**End conditions:** `HYD_END_POSITION` | `HYD_END_TIME` | `HYD_END_PRESSURE` | `HYD_END_FLOW` | `HYD_END_MANUAL`

**Pressure strategies:** P / PI / PID / RBF_PID (via `pressure_controller.c`). RBF_PID is built, tested, and integrated through the pressure_controller segment-config path (Sprint 3).

### Simulator Key Constraints

- Shared `HydraulicSimEnv` is stepped exactly **once per `__HydSimulator_framework_Publish()`** call — multiple FB publishes in the same scan only advance physics by one tick
- Single-pump model: only the axis matching `pump_owner_axis_id` receives flow each step
- CLAMP axis: includes tie-bar stiffness and mold-obstacle force models
- INJECT axis: includes melt resistance proportional to position

### matiec IEC Type System

From `include/matiec/lib/C/`:
- `__DECLARE_VAR(type, name)` — declares IEC-typed variables with force-flag write protection
- `__GET_VAR` / `__SET_VAR` — read/write through the force-flag guard
- `__DECLARE_STRUCT_TYPE` — for custom struct types (e.g., `HYD_AXISMOTION`)
- Base types: `IEC_BOOL`, `IEC_SINT`, `IEC_REAL`, `IEC_WORD`, etc.

---

## Repository Constraints

1. Pump speed is always nonnegative; direction is signaled via `STATE.plannedDirection` — the process layer owns valve actuation
2. `HYD_MotionControlFB_Init()` does full `memset` — resets gains, recipe, runtime state, diagnostics; reload everything after Init
3. `pressureKp/Ki/Kd` use zero as "not configured" — zero/negative values fall back to legacy defaults; to disable proportional control set `pressureController = HYD_PRESSURE_CONTROLLER_NONE`
4. `HYD_DiagnosticCriteria_CheckFaultEscalation()` requires `result.severity == WARNING` and `result.triggered == true` from a prior check call — never pass uninitialized result struct
5. Pure C99, `HYD_` prefix, static bounded memory, no malloc, PLCopen function-block style
6. New `src/*.c` files require re-running `cmake --preset unixgcc` (build uses `file(GLOB_RECURSE ...)`)
7. Tuning-type float thresholds → `include/hyd_config.h` §14B; do not inline in `.c` files

---

## Key Files

| File | Purpose |
|---|---|
| `tests/main.c` | Simulated multi-segment recipe harness — best entry point for call flow |
| `include/motion_control.h` | PLCopen FB struct and full API contract |
| `include/common_types.h` | All shared types, enums, and data structures |
| `include/motion_interface.h` | IEC PLCopen FB definitions for PLC programs |
| `src/motion_interface.c` | IEC adapter implementation with command arbitration |
| `src/motion_control.c` | State machine orchestrator |
| `src/sim/hydro_sim.c` | Physics simulation kernel |
| `src/sim/hydro_sim_fb.c` | Simulator PLC adapter layer |
| `include/hyd_config.h` | Compile-time feature flags and platform configuration |
