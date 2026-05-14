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

1. IEC FB surfaces expose parameters whose runtime semantics are not actually implemented end-to-end.
2. `HYD_LOADPROFILE` exists in the IEC layer but remains effectively unimplemented.
3. Recipe-side takeover semantics are weaker and less explicit than direct-side takeover semantics.

### Medium

4. `Hold` and `Resume` exist in the runtime core but have no equivalent IEC FB surface.
5. `BufferMode` remains a very small subset of the apparent PLCopen surface.
6. `segmentTag` and `segmentType` are still partially conflated in the adapter/build path.
7. Some in-code comments and header contracts are narrower than the actual runtime behavior.

### Low

8. A few adapter/runtime naming and field-mirroring patterns still make the contract harder to maintain than necessary.

## Detailed Gaps

### 1. Exposed IEC Pins Without Full Runtime Semantics

Severity: High

Affected surface:

- [motion_interface.h](/home/dan/project/hdy-motion-light/include/motion_interface.h:121)
- [motion_interface.h](/home/dan/project/hdy-motion-light/include/motion_interface.h:154)

Observed mismatch:

- `HYD_MoveAbsolute` exposes `DECELERATION`, `JERK`, and `CONTINUOUSUPDATE`.
- `HYD_MoveVelocity` exposes `DECELERATION`, `JERK`, and `CONTINUOUSUPDATE`.
- The direct segment builders for those FBs currently consume only a subset of the exposed fields.

Evidence:

- `buildPositionSegment()` uses `POSITION`, `VELOCITY`, `ACCELERATION`, and `DIRECTION`, but not `DECELERATION`, `JERK`, or `CONTINUOUSUPDATE`: [motion_interface.c](/home/dan/project/hdy-motion-light/src/motion_interface.c:66)
- `buildVelocitySegment()` uses `VELOCITY`, `ACCELERATION`, and `DIRECTION`, but not `DECELERATION`, `JERK`, or `CONTINUOUSUPDATE`: [motion_interface.c](/home/dan/project/hdy-motion-light/src/motion_interface.c:96)

Impact:

- The IEC surface suggests a broader PLCopen-style contract than the runtime currently delivers.
- PLC engineers can wire standard pins that are silently ignored.

Recommended direction:

- Either implement those semantics properly in the runtime and adapter path, or explicitly remove/deprecate/mark them unsupported at the interface level.

### 2. `HYD_LOADPROFILE` Is Still a Stub

Severity: High

Affected surface:

- [motion_interface.h](/home/dan/project/hdy-motion-light/include/motion_interface.h:75)
- [motion_interface.c](/home/dan/project/hdy-motion-light/src/motion_interface.c:414)

Observed mismatch:

- The library exposes a dedicated `HYD_LOADPROFILE` FB but does not currently implement meaningful preload behavior in the IEC adapter path.

Evidence:

- `__mcl_cmd_LoadProfile()` is still marked TODO and contains no real logic: [motion_interface.c](/home/dan/project/hdy-motion-light/src/motion_interface.c:414)

Impact:

- The IEC surface implies a recipe-loading workflow that is not actually present.
- This weakens recipe-mode completeness and confuses the intended contract.

Recommended direction:

- Either implement the preload path properly or remove/hide the FB from the supported contract until it exists.

### 3. Recipe Takeover Is Less Explicit Than Direct Takeover

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

### 4. `Hold` and `Resume` Exist Only in the Core

Severity: Medium

Affected surface:

- [motion_control.h](/home/dan/project/hdy-motion-light/include/motion_control.h:114)
- [motion_control.c](/home/dan/project/hdy-motion-light/src/motion_control.c:558)

Observed mismatch:

- `Hold` and `Resume` are fully modeled in the runtime core.
- The IEC surface does not currently expose dedicated FBs or equivalent BOOL-driven command wrappers for them.

Evidence:

- Runtime command enums and implementation contain `HYD_CMD_HOLD` and `HYD_CMD_RESUME`: [motion_control.h](/home/dan/project/hdy-motion-light/include/motion_control.h:121), [motion_control.c](/home/dan/project/hdy-motion-light/src/motion_control.c:782)
- Header comments explicitly say these are currently exposed only through API calls: [motion_control.h](/home/dan/project/hdy-motion-light/include/motion_control.h:114)

Impact:

- The core is more capable than the public PLC-facing surface.
- PLC process-layer users cannot consume these capabilities through the same documented FB workflow as the rest of the library.

Recommended direction:

- Either add IEC-facing wrappers or keep them explicitly internal/API-only and document that as a product boundary.

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

### 6. `segmentTag` and `segmentType` Are Still Partially Conflated

Severity: Medium

Affected surface:

- [common_types.h](/home/dan/project/hdy-motion-light/include/common_types.h:229)
- [motion_interface.c](/home/dan/project/hdy-motion-light/src/motion_interface.c:161)

Observed mismatch:

- The shared type contract treats `segmentTag` as an opaque identifier assigned by the process layer.
- In adapter code, `SEGMENTTAG` from `HYD_AXISMOTION` is reused as both `segmentTag` and `segmentType`.

Evidence:

- `seg.segmentTag = (HYD_UINT8)motion->SEGMENTTAG;`
- `seg.segmentType = (HYD_SegmentType)motion->SEGMENTTAG;`
- in [motion_interface.c](/home/dan/project/hdy-motion-light/src/motion_interface.c:168)

Impact:

- This couples HMI/process-layer identity with domain segment typing.
- It makes long-term template, traceability, and diagnostic naming harder to cleanly separate.

Recommended direction:

- Split “opaque step tag” from “domain segment type” more explicitly in the IEC mapping path.

### 7. In-Code Header Contract Does Not Fully Match Runtime Implementation

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

### 8. Configuration Source-of-Truth Is Still Duplicated

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
   - `LoadProfile`
   - `BufferMode` subset
2. Normalize lifecycle exposure:
   - recipe takeover visibility
   - `Hold/Resume` surface decision
3. Clean structural ambiguities:
   - `segmentTag` vs `segmentType`
   - header-comment vs implementation legality
4. Clean maintainability debt:
   - duplicated config source-of-truth

## Notes

This gap list intentionally mixes:

- hard public-surface mismatches
- runtime-vs-header mismatches
- structural implementation debt that weakens the durability of the published contract

That is deliberate. All of them affect how safely the new documentation set can be treated as the long-term product contract.
