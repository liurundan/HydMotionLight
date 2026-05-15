# Implementation vs Contract Gap List

Date: 2026-05-14

## Purpose

This document records the current gaps between:

- the intended motion-control-layer contract described in the architecture documents
- the code that is currently implemented in the runtime and IEC adapter layers

It is not a redesign document. It is a prioritized discrepancy list meant to drive the next round of implementation planning.

## Scope

This list compares:

- [motion-runtime-contract.md](/home/dan/project/hdy-motion-light/docs/architecture/motion-runtime-contract.md)
- [control-layer-boundary.md](/home/dan/project/hdy-motion-light/docs/architecture/control-layer-boundary.md)
- [plc-process-layer-integration-guide.md](/home/dan/project/hdy-motion-light/docs/integration/plc-process-layer-integration-guide.md)
- [motion-profile-archetypes.md](/home/dan/project/hdy-motion-light/docs/architecture/motion-profile-archetypes.md)

against the current implementation in:

- [motion_control.h](/home/dan/project/hdy-motion-light/include/motion_control.h)
- [motion_interface.h](/home/dan/project/hdy-motion-light/include/motion_interface.h)
- [motion_control.c](/home/dan/project/hdy-motion-light/src/motion_control.c)
- [motion_interface.c](/home/dan/project/hdy-motion-light/src/motion_interface.c)

## Gap Summary

### High

1. IEC FB surfaces still expose a small set of compatibility pins whose runtime semantics are not yet implemented end-to-end.
2. Recipe-side takeover semantics are weaker and less explicit than direct-side takeover semantics.

### Medium

3. `HYD_LOADPROFILE` is implemented as preload-only; PLC integration examples should make that boundary explicit.
4. `Hold` and `Resume` now have dedicated IEC FB wrappers; PLC integration examples still need to be documented.
5. `BufferMode` remains a very small subset of the apparent PLCopen surface.
6. Some in-code comments and header contracts are narrower than the actual runtime behavior.

### Low

7. A few adapter/runtime naming and field-mirroring patterns still make the contract harder to maintain than necessary.

## Detailed Gaps

### 1. Exposed IEC Pins Without Full Runtime Semantics

Severity: High

Affected surface:

- [motion_interface.h](/home/dan/project/hdy-motion-light/include/motion_interface.h:121)
- [motion_interface.h](/home/dan/project/hdy-motion-light/include/motion_interface.h:154)

Observed mismatch:

- `HYD_MoveAbsolute` still exposes `JERK` and `CONTINUOUSUPDATE`.
- `HYD_MoveVelocity` still exposes `JERK` and `CONTINUOUSUPDATE`.
- The direct segment builders for those FBs currently consume only a subset of the exposed fields.

Evidence:

- `buildPositionSegment()` now maps `DECELERATION` independently, but still does not consume `JERK` or `CONTINUOUSUPDATE`: [motion_interface.c](/home/dan/project/hdy-motion-light/src/motion_interface.c:66)
- `buildVelocitySegment()` now maps `DECELERATION` independently, but still does not consume `JERK` or `CONTINUOUSUPDATE`: [motion_interface.c](/home/dan/project/hdy-motion-light/src/motion_interface.c:96)

Impact:

- The IEC surface suggests a broader PLCopen-style contract than the runtime currently delivers.
- PLC engineers can wire standard pins that are silently ignored.

Recommended direction:

- Either implement those semantics properly in the runtime and adapter path, or explicitly remove/deprecate/mark them unsupported at the interface level.

### 2. Recipe Takeover Is Less Explicit Than Direct Takeover

Severity: High

Affected surface:

- [motion-runtime-contract.md](/home/dan/project/hdy-motion-light/docs/architecture/motion-runtime-contract.md:237)
- [motion_interface.c](/home/dan/project/hdy-motion-light/src/motion_interface.c:425)

Observed mismatch:

- Direct-side preemption is surfaced explicitly through `COMMANDABORTED`.
- Recipe-side `MoveProfile` ownership loss is currently observed mostly through `ACTIVE/BUSY` changes rather than through a dedicated takeover signal.

Evidence:

- `MoveAbsolute`, `MoveVelocity`, and `PressureHandle` explicitly raise `COMMANDABORTED` on ownership loss: [motion_interface.c](/home/dan/project/hdy-motion-light/src/motion_interface.c:694), [motion_interface.c](/home/dan/project/hdy-motion-light/src/motion_interface.c:822), [motion_interface.c](/home/dan/project/hdy-motion-light/src/motion_interface.c:994)
- `MoveProfile` does not expose a corresponding `COMMANDABORTED` output and instead falls back to execution-id/activity comparison: [motion_interface.c](/home/dan/project/hdy-motion-light/src/motion_interface.c:501)

Impact:

- The runtime contract now documents this asymmetry, but it remains a real implementation inconsistency.
- Mixed recipe/direct projects must treat recipe takeover differently from direct takeover.

Recommended direction:

- Decide whether recipe-side takeover should remain implicitly observed or be surfaced as a first-class lifecycle signal.

### 3. `HYD_LOADPROFILE` Is Preload-Only

Severity: Medium

Affected surface:

- [motion_interface.h](/home/dan/project/hdy-motion-light/include/motion_interface.h:75)
- [motion_interface.c](/home/dan/project/hdy-motion-light/src/motion_interface.c:500)
- [tests/test_motion_interface_unit.c](/home/dan/project/hdy-motion-light/tests/test_motion_interface_unit.c:336)

Observed mismatch:

- `HYD_LOADPROFILE` performs meaningful preload behavior, but it is not an execution lifecycle owner.
- It loads one segment into `RECIPE[0]` for recipe-configured axes or into `DIRECT_SEGMENT` for direct-configured axes.

Evidence:

- `__mcl_cmd_LoadProfile()` builds a segment from `MOTION` and calls `HYD_MotionControlFB_LoadRecipe()` or `HYD_MotionControlFB_LoadDirectSegment()`.
- Unit tests assert recipe preload, direct preload, and independent `SEGMENTTAG` / `SEGMENTTYPE` mapping.

Impact:

- PLC users must not treat `LoadProfile.DONE` as motion completion or ownership acquisition.
- PLC integration docs should describe it as configuration/preload completion only.

Recommended direction:

- Keep this preload-only boundary explicit in the integration guide.

### 4. `Hold` and `Resume` IEC Surface Added

Severity: Resolved implementation gap; documentation follow-up remains

Affected surface:

- [motion_control.h](/home/dan/project/hdy-motion-light/include/motion_control.h:114)
- [motion_control.c](/home/dan/project/hdy-motion-light/src/motion_control.c:558)
- [motion_interface.h](/home/dan/project/hdy-motion-light/include/motion_interface.h)
- [motion_interface.c](/home/dan/project/hdy-motion-light/src/motion_interface.c)
- [pousHydMotion.xml](/home/dan/project/hdy-motion-light/pousHydMotion.xml)

Current status:

- `Hold` and `Resume` are fully modeled in the runtime core.
- The IEC surface now exposes dedicated `HYD_Hold` and `HYD_Resume` command wrappers.
- The wrappers expose the same command-lifecycle fields as other simple command FBs: `AXISID`, `EXECUTE`, `DONE`, `BUSY`, `ERROR`, and `ERRORID`.

Evidence:

- Runtime command enums and implementation contain `HYD_CMD_HOLD` and `HYD_CMD_RESUME`: [motion_control.h](/home/dan/project/hdy-motion-light/include/motion_control.h:121), [motion_control.c](/home/dan/project/hdy-motion-light/src/motion_control.c:782)
- IEC adapter wrappers call `HYD_MotionControlFB_Hold()` and `HYD_MotionControlFB_Resume()` and report `DONE` after the requested state transition is observed.
- Interface layout consistency tests cover the XML/C field-order contract for both new POUs.

Impact:

- PLC process-layer users can consume hold/resume through the same documented FB workflow as the rest of the library.
- PLC integration documentation should still describe when process logic should prefer `Hold`/`Resume` over `Stop`, `Abort`, and process-level phase transitions.

Recommended direction:

- Add PLC integration examples for `HYD_Hold` and `HYD_Resume`.
- Keep `Hold`/`Resume` documented as runtime pause/resume semantics, not as machine phase sequencing.

### 5. `BufferMode` Is Only a Small Subset of the Apparent Surface

Severity: Medium

Affected surface:

- [common_types.h](/home/dan/project/hdy-motion-light/include/common_types.h:163)
- [motion_interface.h](/home/dan/project/hdy-motion-light/include/motion_interface.h:101)

Observed mismatch:

- Multiple FBs expose `BUFFERMODE`.
- The current implementation meaningfully supports only `ABORT` and `BUFFER`.
- Blending-style semantics suggested by broader PLCopen expectations do not exist.

Evidence:

- Buffer-mode enum defines only `HYD_BUFFER_MODE_ABORT` and `HYD_BUFFER_MODE_BUFFER`: [common_types.h](/home/dan/project/hdy-motion-light/include/common_types.h:167)
- Adapter logic only checks for `ABORT` behavior in direct start paths: [motion_interface.c](/home/dan/project/hdy-motion-light/src/motion_interface.c:229), [motion_interface.c](/home/dan/project/hdy-motion-light/src/motion_interface.c:645), [motion_interface.c](/home/dan/project/hdy-motion-light/src/motion_interface.c:774), [motion_interface.c](/home/dan/project/hdy-motion-light/src/motion_interface.c:947)

Impact:

- The interface surface is broader than the actual execution semantics.
- PLC users may expect more buffering/blending behavior than exists.

Recommended direction:

- Keep the limited subset explicit, or narrow the visible surface until more modes are genuinely supported.

### 6. In-Code Header Contract Does Not Fully Match Runtime Implementation

Severity: Medium

Affected surface:

- [motion_control.h](/home/dan/project/hdy-motion-light/include/motion_control.h:104)
- [motion_control.c](/home/dan/project/hdy-motion-light/src/motion_control.c:57)

Observed mismatch:

- The header comment for command legality is narrower than the actual implementation for at least `ABORT`.

Evidence:

- Header comment says `ABORT: STARTING / RUNNING / SEGMENT_COMPLETE / HOLD`
- Implementation state mask also allows `IDLE`, `READY`, `DONE`, and `ABORTED`: [motion_control.c](/home/dan/project/hdy-motion-light/src/motion_control.c:70)

Impact:

- Developers reading the header can form a different understanding than the code actually enforces.

Recommended direction:

- Make the header contract and the code match exactly; do not leave legality comments as historical approximations.

### 7. Configuration Source-of-Truth Is Still Duplicated

Severity: Low

Affected surface:

- [motion_control.h](/home/dan/project/hdy-motion-light/include/motion_control.h:168)
- [motion_control.h](/home/dan/project/hdy-motion-light/include/motion_control.h:231)
- [motion_control.c](/home/dan/project/hdy-motion-light/src/motion_control.c:1629)

Observed mismatch:

- Some tunable fields still exist both as top-level compatibility fields and in `_params`.

Evidence:

- `FLOW_TO_PUMP_SPEED_GAIN`, `PUMP_SPEED_LIMIT`, `_useSimulation`
- mirrored with `_params.flowToPumpSpeedGain`, `_params.pumpSpeedLimit`, `_params.useSimulation`

Impact:

- This does not immediately violate the published runtime contract, but it increases maintenance risk and makes the implementation harder to align with documentation over time.

Recommended direction:

- Move toward a single source of truth for tunable configuration or make the mirror contract explicitly transitional.

## Recommended Fix Order

1. Harden the IEC/public surface:
   - unsupported pins
   - `BufferMode` subset
2. Normalize lifecycle exposure:
   - recipe takeover visibility
   - `Hold/Resume` surface decision
3. Clean structural ambiguities:
   - header-comment vs implementation legality
4. Clean maintainability debt:
   - duplicated config source-of-truth

## Notes

This gap list intentionally mixes:

- hard public-surface mismatches
- runtime-vs-header mismatches
- structural implementation debt that weakens the durability of the published contract

That is deliberate. All of them affect how safely the new documentation set can be treated as the long-term product contract.
