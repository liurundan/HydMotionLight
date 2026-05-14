# Hold Resume IEC Surface Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add PLC-facing `HYD_Hold` and `HYD_Resume` function blocks that expose the runtime core's existing hold/resume commands through the same IEC adapter contract as the rest of the motion surface.

**Architecture:** Keep the runtime core unchanged because `HYD_MotionControlFB_Hold()` and `HYD_MotionControlFB_Resume()` already exist and have state-machine semantics. Add small IEC wrapper structs, command functions, XML POU entries, layout-check mappings, and focused unit coverage. The wrappers are command FBs, not execution owners: `DONE` acknowledges the requested state transition, `BUSY` is true only while the transition is pending, and invalid state/axis requests report `ERROR`.

**Tech Stack:** C99, existing matiec-style `__DECLARE_VAR` structs, `src/motion_interface.c`, `pousHydMotion.xml`, CMake/CTest, Python interface layout checker.

---

## File Map

- Modify: `include/motion_interface.h`
  Add `HYD_HOLD` and `HYD_RESUME` structs plus `__mcl_cmd_Hold` / `__mcl_cmd_Resume` declarations.
- Modify: `src/motion_interface.c`
  Add IEC adapter implementations that call `HYD_MotionControlFB_Hold()` / `HYD_MotionControlFB_Resume()` on rising edges and project `DONE/BUSY/ERROR/ERRORID`.
- Modify: `pousHydMotion.xml`
  Add `HYD_Hold` and `HYD_Resume` function block POUs with the same field order as the C structs.
- Modify: `scripts/check_interface_layout_consistency.py`
  Add XML-to-C type mappings for the new POUs.
- Modify: `tests/test_motion_interface_unit.c`
  Add failing tests for successful Hold/Resume, invalid axis handling, and invalid state handling.
- Reference: `include/motion_control.h`
  Runtime state and public Hold/Resume API.
- Reference: `src/motion_control.c`
  Runtime state transitions for `HYD_CMD_HOLD` and `HYD_CMD_RESUME`.

## Task 1: Add Failing Unit Tests

**Files:**
- Modify: `tests/test_motion_interface_unit.c`

- [ ] **Step 1: Add Hold/Resume tests before `test_multiple_axes_operate_independently`**

Add tests that assert:

- `HYD_Hold` reports `BUSY` on the trigger scan, then `DONE` once runtime reaches `HYD_FB_STATE_HOLD`.
- `HYD_Resume` reports `BUSY` on the trigger scan, then `DONE` once runtime leaves `HYD_FB_STATE_HOLD`.
- invalid axis requests report `ERROR` with `HYD_DIAG_CODE_START_CONTEXT_INVALID`.
- invalid runtime states report `ERROR` and keep `BUSY=false`.

- [ ] **Step 2: Run the focused test and verify it fails**

Run:

```bash
cmake --build build --target test_motion_interface_unit -j2
```

Expected: compile failure for unknown `HYD_HOLD`, `HYD_RESUME`, `__mcl_cmd_Hold`, and `__mcl_cmd_Resume`.

## Task 2: Add C Header and Adapter Implementations

**Files:**
- Modify: `include/motion_interface.h`
- Modify: `src/motion_interface.c`

- [ ] **Step 1: Add `HYD_HOLD` and `HYD_RESUME` structs after `HYD_STOP`**

Both structs use:

```c
EN, ENO, AXISID, EXECUTE, DONE, BUSY, ERROR, ERRORID, EXECUTE0, DONE0, _PENDING
```

- [ ] **Step 2: Add function declarations**

```c
extern void __mcl_cmd_Hold(HYD_HOLD *data__);
extern void __mcl_cmd_Resume(HYD_RESUME *data__);
```

- [ ] **Step 3: Implement adapter functions**

`__mcl_cmd_Hold` calls `HYD_MotionControlFB_Hold()` on rising edge and completes when `fb->FB_STATE == HYD_FB_STATE_HOLD`.

`__mcl_cmd_Resume` calls `HYD_MotionControlFB_Resume()` on rising edge and completes when `fb->FB_STATE != HYD_FB_STATE_HOLD`.

Both wrappers clear `DONE/ERROR/BUSY/_PENDING` when `EXECUTE=false`.

- [ ] **Step 4: Run focused build and test**

Run:

```bash
cmake --build build --target test_motion_interface_unit -j2
./build/test_motion_interface_unit
```

Expected: unit test passes.

## Task 3: Add XML Surface and Layout Mapping

**Files:**
- Modify: `pousHydMotion.xml`
- Modify: `scripts/check_interface_layout_consistency.py`

- [ ] **Step 1: Add script mappings**

```python
    "HYD_Hold": "HYD_HOLD",
    "HYD_Resume": "HYD_RESUME",
```

- [ ] **Step 2: Add `HYD_Hold` and `HYD_Resume` POU blocks after `HYD_Stop`**

Use matching field order:

```text
AXISID, EXECUTE, DONE, BUSY, ERROR, ERRORID, EXECUTE0, DONE0, _PENDING
```

- [ ] **Step 3: Run layout check**

Run:

```bash
python3 tests/test_interface_layout_consistency.py
```

Expected: `interface layout consistency tests passed`

## Task 4: Documentation Cleanup and Verification

**Files:**
- Modify: `docs/architecture/implementation-contract-gap-list.md`
- Modify: `docs/architecture/motion-runtime-contract.md`

- [ ] **Step 1: Update docs**

Update the gap list and runtime contract so they no longer say Hold/Resume wrappers are absent.

- [ ] **Step 2: Run targeted verification**

Run:

```bash
cmake --build build -j2
./build/test_motion_interface_unit
python3 tests/test_interface_layout_consistency.py
ctest --test-dir build -R "test_motion_interface_unit|test_interface_layout_consistency|test_motion_interface_done_signals|test_motion_interface_arbitration" --output-on-failure
```

Expected: all commands pass.

- [ ] **Step 3: Commit**

```bash
git add include/motion_interface.h src/motion_interface.c pousHydMotion.xml scripts/check_interface_layout_consistency.py tests/test_motion_interface_unit.c docs/architecture/implementation-contract-gap-list.md docs/architecture/motion-runtime-contract.md docs/superpowers/plans/2026-05-14-hold-resume-iec-surface.md
git commit -m "feat: add hold resume IEC surface"
```

## Self-Review

Spec coverage:

- Covers the highest-value follow-up from the gap list: runtime already has Hold/Resume and PLC surface now gains direct wrappers.
- Does not mix in `segmentTag` / `segmentType` or config source-of-truth cleanup.

Placeholder scan:

- No `TODO`, `TBD`, or unspecified implementation steps are present.

Type consistency:

- XML names use `HYD_Hold` and `HYD_Resume`; C types use `HYD_HOLD` and `HYD_RESUME`; adapter functions use `__mcl_cmd_Hold` and `__mcl_cmd_Resume`.
