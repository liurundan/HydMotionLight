# Motion Control Continuous Command Roadmap Design

Date: 2026-05-16

## Goal

Extend the hydraulic motion control library so runtime commands can change smoothly while a move is already running.

The target is not just "more FB pins". The target is a control contract that matches injection-molding work:

- velocity commands can be updated continuously during a speed move
- pressure commands can be updated continuously during pressure hold / pressure ramp
- manual jog / electric-style motion can be expressed explicitly in IEC and executed safely in core

## Current State

The runtime already has a solid base:

- `motion_planner.c` owns velocity shaping for position and speed segments
- `motion_control.c` owns lifecycle, ownership, diagnostics, and direct-session arbitration
- `motion_interface.c` already maps IEC FB inputs into direct segments and owns the public PLC-facing surface
- `ActionProfile` helpers already build reusable defaults for clamp, injection, eject, carriage, and pressure segments

The missing part is runtime mutability.

Today, the active segment is copied into `_activeSegment` at start time. After that:

- `MoveVelocity.VELOCITY` is latched only at start
- `PressureHandle.PRESSURE` is latched only at start
- position/direct segments do not expose a safe live-update path
- `CONTINUOUSUPDATE` is rejected rather than used

That is why the library can execute a move, but cannot yet behave like a real machine operator expects during fill, pressure trim, or jog tuning.

## Problem

Hydraulic injection molding does not only need one-shot target values.

It needs three behaviors that are currently missing or incomplete:

1. continuous target velocity updates for fill, trim, and manual speed motion
2. continuous target pressure updates for pressure hold and pressure trim
3. a dedicated jog / electric-style manual motion path for setup and service

These are related, but they are not the same subsystem. They should not be forced into one oversized implementation.

## Recommendation

Use a phased roadmap with shared core update semantics:

### Phase 1: Live target update infrastructure

Add a core-level "active segment update" mechanism that can safely adjust selected setpoints on the running segment without resetting ownership or controller state.

### Phase 2: Continuous velocity update

Expose the update path through `MoveVelocity` and any speed-ramp motion surface that is explicitly marked updateable.

### Phase 3: Continuous pressure update

Expose the same update path through `PressureHandle` and pressure-hold / pressure-ramp segments.

### Phase 4: Jog / electric manual motion

Add a dedicated manual motion FB and core behavior for press-and-hold motion, decel-on-release, and direction-safe jogging.

This order is recommended because it reuses the same core update contract and keeps the risk surface small.

## Design Options

### Option A: Update existing FBs and add one shared core update API

Pros:

- smallest public surface change
- easiest to keep compatible with current IEC naming
- reuse current direct-session and ownership model

Cons:

- `MoveVelocity` and `PressureHandle` remain special cases in the adapter layer

### Option B: Introduce new FBs for continuous update and jog

Pros:

- clean IEC API separation
- less compatibility pressure on existing FB behavior

Cons:

- more surface area
- more work for PLC users
- duplicates lifecycle semantics that already exist

### Option C: Restart segments on every update

Pros:

- easy to sketch

Cons:

- wrong for hydraulics
- resets planner/controller state
- creates discontinuities in velocity and pressure
- not recommended

**Recommendation:** Option A for velocity/pressure updates, plus a dedicated jog FB only if manual motion cannot be expressed cleanly with the existing command family.

## Architecture

Keep responsibilities separated:

- `motion_planner.c`: generate smooth target motion from the current running target
- `motion_control.c`: own runtime state, live update application, ownership, and safety gating
- `motion_interface.c`: translate IEC inputs into update requests
- `pressure_controller.c`: keep controller state stable across target changes
- `velocity_controller.c`: continue to provide local correction for speed-ramp modes

The key architectural rule is this:

**A live update must not reinitialize the active motion unless the user explicitly requests a stop or restart.**

That means:

- do not clear planner state on a normal update
- do not reset pressure controller integral/adaptive state on a normal update
- do not alter ownership or execution IDs on a normal update
- do not convert an update into a new segment start

## Proposed Core Contract

Introduce a shared live-update request concept in the core.

It should support at least these fields:

- new target velocity
- new target pressure
- optional new target flow bias
- optional new ramp rate / deceleration limit
- update flags indicating which fields are present

The core should apply updates only when:

- the FB still owns the active execution
- the segment is active and not finished
- the command is compatible with the running mode
- the new value is within the segment's legal limits

When an update is accepted:

- planner state continues from the current target
- controller state continues from the current filtered / integrated state
- the next scan uses the new target

When an update is rejected:

- keep the current motion unchanged
- report a diagnostic that identifies the bad update reason

## Velocity Update Semantics

Continuous velocity update should be supported for speed-governed motion and manual jog-like motion.

Expected behavior:

- `MoveVelocity` can change its target while still executing
- the change should be rate-limited by existing acceleration and deceleration limits
- direction changes should still respect the current braking logic
- updates must not create a step change in pump flow

The adapter layer should treat `CONTINUOUSUPDATE` as an explicit opt-in for live target edits.

Recommended rule:

- when `CONTINUOUSUPDATE = false`, current one-shot behavior stays unchanged
- when `CONTINUOUSUPDATE = true`, the `VELOCITY` input becomes a live target, not just a start value

## Pressure Update Semantics

Continuous pressure update should be supported for hold-pressure and pressure-ramp motion.

Expected behavior:

- `PressureHandle.PRESSURE` can be updated while running
- the pressure ramp controller should move toward the new target smoothly
- PI / PID / RBF-PID state should be preserved across the update
- feedforward flow bias should remain stable unless explicitly changed

Recommended rule:

- live pressure updates are accepted only when the active segment is pressure-controlled
- pressure updates should change the current target, not restart the segment
- pressure updates should respect the configured pressure ramp rate

## Jog / Electric Manual Motion

Add a dedicated manual motion path for setup and service use.

This is the closest thing to an "electric" manual function in the current hydraulic library: an operator holds a direction command, the axis moves, and release triggers controlled deceleration.

Expected behavior:

- explicit direction input
- explicit speed limit
- press-and-hold execution
- release-to-stop with deceleration
- no position completion
- no automatic phase switch
- safe preemption by Stop / Abort / Reset

This should be modeled as a manual speed-ramp command, not as a fake position move.

## IEC Surface

The IEC layer should expose the new behavior clearly instead of burying it behind unrelated fields.

Recommended surface changes:

- preserve `MoveVelocity`
- preserve `PressureHandle`
- add or repurpose `CONTINUOUSUPDATE` only where it has clear semantics
- add a dedicated jog FB if manual motion cannot be expressed cleanly through the existing surfaces

The IEC layer should not invent a second ownership model.
It should reuse the current direct-session ownership, abort, and busy/done mapping.

## Data Flow

1. PLC writes target values into the FB
2. IEC adapter validates the command and converts it into a live-update request
3. `motion_control.c` accepts or rejects the update
4. `motion_planner.c` sees the new target on the next scan
5. controller and pump outputs continue from the current runtime state
6. diagnostics and state reporter reflect the new command without a lifecycle reset

## Error Handling

Live updates should fail fast when they are unsafe.

Reject updates when:

- the FB does not own the active execution
- the axis is in Hold / Abort / Fault
- the command mode does not match the active mode
- the new target violates configured limits
- the requested behavior conflicts with the current segment contract

On rejection:

- do not mutate the current running target
- set a clear diagnostic code
- keep BUSY/ACTIVE semantics consistent with the existing lifecycle model

## Testing

Each phase needs its own tests.

### Velocity update tests

- update target velocity while running and verify the planner continues smoothly
- update to a lower target and verify deceleration is continuous
- update to a higher target and verify acceleration is continuous
- reject updates while not owning the active motion

### Pressure update tests

- update target pressure while running and verify ramp continuity
- preserve controller state across the update
- reject pressure updates in the wrong mode

### Jog tests

- press starts motion
- hold keeps motion active
- release decelerates to stop
- stop/abort/reset preempt jog immediately

### Integration tests

- direct `MoveVelocity` update in a live simulation loop
- direct `PressureHandle` update in a live simulation loop
- existing move and stop tests remain stable
- V/P observation and done signaling remain unchanged unless explicitly part of the new contract

## Scope Boundaries

This roadmap does not change:

- jerk-limited S-curve planning
- automatic V/P transfer execution
- recipe validation rules
- segment count limits
- hardware abstraction for true electric servo axes

If the team later wants a real electric-axis abstraction, that should be a separate project. The current library should first solve the hydraulic motion contract it already owns.

## Relationship To Existing Documents

This roadmap is consistent with:

- [motion-runtime-contract.md](/home/dan/project/hdy-motion-light/docs/architecture/motion-runtime-contract.md)
- [control-layer-boundary.md](/home/dan/project/hdy-motion-light/docs/architecture/control-layer-boundary.md)
- [plc-process-layer-integration-guide.md](/home/dan/project/hdy-motion-light/docs/integration/plc-process-layer-integration-guide.md)
- [2026-05-16-strict-position-completion-with-online-planner-design.md](/home/dan/project/hdy-motion-light/docs/superpowers/specs/2026-05-16-strict-position-completion-with-online-planner-design.md)

This design treats live target updates and jog as control-layer capabilities, while preserving PLC ownership of phase sequencing and machine workflow.

## Recommended Next Step

Start with the velocity live-update subproject.

Reason:

- it is the most immediately useful for injection fill and tuning
- it exercises the shared update path that pressure updates will reuse
- it is easier to verify than jog
- it exposes any ownership or controller-state problems early

