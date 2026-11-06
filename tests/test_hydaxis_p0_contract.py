#!/usr/bin/env python3
"""Source-contract checks for the FB_HydAxis segment-ramp design."""

from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
SOURCE_PATH = ROOT / "iec" / "FB_HydAxis.st"
SOURCE = SOURCE_PATH.read_text(encoding="utf-8")


def section_between(start: str, end: str) -> str:
    start_index = SOURCE.index(start)
    end_index = SOURCE.index(end, start_index)
    return SOURCE[start_index:end_index]


class HydAxisP0ContractTest(unittest.TestCase):
    def test_segment_ramp_uses_positive_magnitudes_and_start_velocity(self):
        declarations = section_between("rPos : REAL;", "END_VAR")
        ramp = section_between("METHOD PRIVATE CalcSegmentRamp", "END_METHOD")

        self.assertIn("rMinRampTime : REAL := 100.0", declarations)
        self.assertIn("rVelMagnitude := ABS(rVel)", ramp)
        self.assertIn("rEndVelMagnitude := ABS(rEndVel)", ramp)
        self.assertIn("MAX(UINT_TO_REAL(uiAcc), rMinRampTime)", ramp)
        self.assertIn("MAX(UINT_TO_REAL(uiDec), rMinRampTime)", ramp)
        self.assertIn("tAcc := MAX(UINT_TO_REAL(uiAcc), rMinRampTime) * 0.001", ramp)
        self.assertIn("tDec := MAX(UINT_TO_REAL(uiDec), rMinRampTime) * 0.001", ramp)
        self.assertIn("rVel0Magnitude := CalcStartVelocity();", ramp)
        self.assertIn("ABS(rVel - rVel0Magnitude)", ramp)

    def test_direction_speed_limit_is_selected_before_velocity_conversion(self):
        conversion = section_between("rPres :=", "//--- 2. 状态机主逻辑")

        self.assertLess(conversion.index("rMaxSpeedExtend"), conversion.index("rVel :="))
        self.assertLess(conversion.index("rMaxSpeedRetract"), conversion.index("rEndVel :="))
        self.assertIn("rMaxSpeedDir := stHydAxisRef.rMaxSpeedExtend", conversion)
        self.assertIn("rMaxSpeedDir := stHydAxisRef.rMaxSpeedRetract", conversion)
        self.assertIn("rVel := UINT_TO_REAL(uiSpd)", conversion)
        self.assertIn("rEndVel := UINT_TO_REAL(uiEndSpd)", conversion)
        self.assertIn("IF iDir = 1", conversion)
        self.assertIn("rMaxSpeedExtend", conversion)
        self.assertIn("rMaxSpeedRetract", conversion)

    def test_segment_ramp_rejection_blocks_motion_before_state_machine(self):
        segment_gate = section_between("bNewSeg := rTrigStart.Q", "//--- 2. 状态机主逻辑")
        failure_start = segment_gate.index("ELSIF NOT CalcSegmentRamp()")
        failure_branch = segment_gate[failure_start:segment_gate.index("END_IF;", failure_start)]
        active_route_end = failure_branch.index("ELSE")
        active_route = failure_branch[:active_route_end]
        idle_route = failure_branch[active_route_end:]

        self.assertIn("IF NOT CalcSegmentRamp() THEN", segment_gate)
        self.assertIn("bAlarm := TRUE", failure_branch)
        self.assertIn("dwAlarmID := 16#00000002", failure_branch)
        self.assertIn("bCmdExecute := FALSE", failure_branch)
        self.assertIn("IF iStep = 100 OR iStep = 105 THEN", failure_branch)
        self.assertIn("iStep := 200", active_route)
        self.assertIn("bCmdExecute := FALSE", active_route)
        self.assertIn("iStep := 900", idle_route)
        self.assertLess(SOURCE.index("IF NOT CalcSegmentRamp()"), SOURCE.index("CASE iStep OF"))
        idle_state = section_between("10: // 空闲", "100: // 运行中")
        self.assertIn("rTrigStart.Q AND NOT bAlarm", idle_state)

    def test_motion_segment_rejects_unknown_direction_but_pressure_does_not(self):
        segment_gate = section_between("bNewSeg := rTrigStart.Q", "//--- 2. 状态机主逻辑")
        direction_check = segment_gate[:segment_gate.index("ELSIF NOT CalcSegmentRamp()")]
        direction_start = direction_check.index("IF (uiMode = 1 OR uiMode = 2)")
        direction_branch = direction_check[direction_start:]
        step_start = direction_branch.index("IF iStep = 100 OR iStep = 105 THEN")
        step_routes = direction_branch[step_start:direction_branch.index("END_IF;", step_start)]
        idle_route_start = step_routes.index("ELSE")

        self.assertIn("(uiMode = 1 OR uiMode = 2)", direction_check)
        self.assertIn("(iDir <> 1) AND (iDir <> 2)", direction_check)
        self.assertNotIn("uiMode = 3", direction_check)
        self.assertIn("bCmdExecute := FALSE", direction_branch)
        self.assertIn("iStep := 200", step_routes[:idle_route_start])
        self.assertIn("iStep := 900", step_routes[idle_route_start:])

    def test_segment_ramp_rejects_nonpositive_speed_limit_before_division(self):
        ramp = section_between("METHOD PRIVATE CalcSegmentRamp", "END_METHOD")
        guard_start = ramp.index("IF rMaxSpeedDir <= 0.0 THEN")
        guard_end = ramp.index("END_IF;", guard_start)
        invalid_speed_path = ramp[guard_start:guard_end]

        for assignment in (
            "rVel := 0.0;",
            "rEndVel := 0.0;",
            "rAcc := 0.0;",
            "rDec := 0.0;",
            "CalcSegmentRamp := FALSE;",
        ):
            self.assertIn(assignment, invalid_speed_path)
        self.assertNotIn("/", ramp[:guard_start])
        self.assertLess(ramp.index("CalcSegmentRamp := FALSE;"), ramp.index(" / tDec"))

    def test_pressure_mode_clears_segment_latch_before_speed_limit_guard(self):
        ramp = section_between("METHOD PRIVATE CalcSegmentRamp", "END_METHOD")
        pressure_start = ramp.index("IF uiMode = 3 THEN")
        clear_latch = ramp.index("ClearSegLatch();", pressure_start)
        pressure_success = ramp.index("CalcSegmentRamp := TRUE;", clear_latch)
        pressure_return = ramp.index("RETURN;", pressure_success)
        speed_guard = ramp.index("IF rMaxSpeedDir <= 0.0 THEN")

        self.assertLess(pressure_start, clear_latch)
        self.assertLess(clear_latch, pressure_success)
        self.assertLess(pressure_success, pressure_return)
        self.assertLess(pressure_return, speed_guard)

    def test_segment_rates_remain_latched_between_new_segment_edges(self):
        declarations = section_between("rPos : REAL;", "END_VAR")
        conversion = section_between("rPres :=", "//--- Terminal same-direction deceleration handoff ---")

        self.assertIn("rAcc : REAL := 200.0", declarations)
        self.assertIn("rDec : REAL := 200.0", declarations)
        self.assertNotRegex(conversion, r"(?m)^\s*rAcc\s*:=")
        self.assertNotRegex(conversion, r"(?m)^\s*rDec\s*:=")

    def test_start_velocity_uses_previous_segment_position_velocity_or_zero(self):
        start_velocity = section_between(
            "METHOD PRIVATE CalcStartVelocity", "END_METHOD"
        )

        self.assertIn("rEndVelLastSeg", start_velocity)
        self.assertIn("rVelLastSeg", start_velocity)
        self.assertIn("ELSE", start_velocity)
        self.assertIn("CalcStartVelocity := 0.0;", start_velocity)
        self.assertLess(start_velocity.index("rEndVelLastSeg"), start_velocity.index("rVelLastSeg"))
        self.assertLess(start_velocity.index("rVelLastSeg"), start_velocity.index("CalcStartVelocity := 0.0;"))

    def test_segment_latches_capture_direction_and_decel_to_zero(self):
        declarations = section_between("rPos : REAL;", "END_VAR")
        segment_trigger = section_between(
            "bNewSeg := rTrigStart.Q", "//--- 2. 状态机主逻辑"
        )
        ramp = section_between("METHOD PRIVATE CalcSegmentRamp", "END_METHOD")

        self.assertIn("iDirActive", declarations)
        self.assertIn("iDirTarget", declarations)
        self.assertIn("bDecelToZero", declarations)
        stop_decel_default = ramp.index("rStopDec := MAX(rVelMagnitude / tDec, 1.0);")
        reversal_start = ramp.index("IF (rVel0Magnitude > rDeltaVDeadZone)")
        delta_velocity = ramp.index("rDeltaV := rVelMagnitude - rVel0Magnitude;")
        self.assertLess(stop_decel_default, reversal_start)
        reversal_branch = ramp[reversal_start:delta_velocity]
        self.assertLess(reversal_start, delta_velocity)
        self.assertIn("iDirLastSeg <> iDirTarget", reversal_branch)
        self.assertIn("bDirectionChange := TRUE", reversal_branch)
        self.assertIn("bDecelToZero := TRUE", reversal_branch)
        self.assertIn("rDec := rVel0Magnitude / tDec", reversal_branch)
        self.assertIn("rAcc := MAX(rVel / tAcc, 1.0)", reversal_branch)
        self.assertIn("rStopDec := rDec", reversal_branch)
        self.assertIn("iDirActive := iDirLastSeg", reversal_branch)
        self.assertIn("rTransTimer := 0.0", reversal_branch)
        self.assertIn("iDirActive := iDirTarget", ramp)
        self.assertIn("rPosActive := rPos", segment_trigger)
        self.assertIn("rVelActive := MAX(rVel, 0.0)", segment_trigger)
        self.assertIn("rEndVelActive := MAX(rEndVel, 0.0)", segment_trigger)
        self.assertIn("rPresActive := rPres", segment_trigger)

    def test_motion_calls_use_active_segment_values_and_direction(self):
        move_calls = section_between("fbMoveContAbs(", "IF iStep = 200 THEN")

        self.assertIn("POSITION := rPosActive", move_calls)
        self.assertIn("VELOCITY := rVelActive", move_calls)
        self.assertIn("ENDVELOCITY := rEndVelActive", move_calls)
        self.assertIn("ENDVELOCITYDIRECTION := iDirActive", move_calls)
        self.assertIn("DIRECTION := iDirActive", move_calls)

    def test_reverse_state_prioritizes_estop_and_stop_edges_before_completion(self):
        reverse_state = section_between("150: // 反向减速到零", "200: // 正常停止")

        self.assertIn("bCmdExecute := FALSE", reverse_state)
        self.assertIn("IF rTrigEStop.Q THEN", reverse_state)
        self.assertIn("iStep := 300", reverse_state)
        self.assertIn("ELSIF rTrigStop.Q THEN", reverse_state)
        self.assertIn("iStep := 200", reverse_state)
        self.assertLess(reverse_state.index("rTrigEStop.Q"), reverse_state.index("rTrigStop.Q"))
        self.assertLess(reverse_state.index("rTrigStop.Q"), reverse_state.index("fbStop.DONE"))
        self.assertLess(reverse_state.index("rTrigStop.Q"), reverse_state.index("fbStop.ERROR"))

    def test_reverse_state_owns_stop_completion_before_switching_direction(self):
        reverse_state = section_between("150: // 反向减速到零", "200: // 正常停止")
        run_state = section_between("100: // 运行中", "105: // 运行过渡步")
        ramp = section_between("METHOD PRIVATE CalcSegmentRamp", "END_METHOD")
        stop_route = section_between(
            "IF iStep = 150 THEN", "IF iStep = 300 THEN"
        )

        reverse_start = run_state.index("IF bDecelToZero THEN")
        same_mode_retrigger = run_state.index("ELSIF uiMode = uiActiveMode THEN")
        self.assertLess(reverse_start, same_mode_retrigger)
        self.assertIn("uiActiveMode := uiMode", run_state[reverse_start:same_mode_retrigger])
        self.assertIn("bCmdExecute := FALSE", run_state[reverse_start:same_mode_retrigger])
        self.assertIn("iStep := 150", run_state[reverse_start:same_mode_retrigger])
        self.assertIn("bCmdExecute := FALSE", reverse_state)
        self.assertIn("IF fbStop.DONE THEN", reverse_state)
        self.assertIn("ELSIF fbStop.ERROR THEN", reverse_state)
        self.assertIn("dwAlarmID := fbStop.ERRORID", reverse_state)
        self.assertIn("iDirActive := iDirTarget", reverse_state)
        self.assertIn("rDec := rDecSeg", reverse_state)
        self.assertIn("rDecSeg := MAX(ABS(rVelMagnitude - rEndVelMagnitude) / tDec, 1.0)", ramp)
        self.assertIn("rDecSeg := MAX(rVelMagnitude / tDec, 1.0)", ramp)
        self.assertIn("rVel0 := 0.0", reverse_state)
        self.assertIn("rVel0Magnitude := 0.0", reverse_state)
        self.assertIn("iStep := 105", reverse_state)
        self.assertNotIn("ClearSegLatch()", reverse_state)
        self.assertLess(reverse_state.index("fbStop.DONE"), reverse_state.index("iDirActive := iDirTarget"))
        self.assertIn("EXECUTE := TRUE, DECELERATION := rDec", stop_route)
        self.assertIn("EXECUTE := TRUE, DECELERATION := rStopDec", stop_route)

    def test_terminal_position_handoff_uses_conservative_braking_speed_and_vend(self):
        transition = section_between(
            "//--- Terminal same-direction deceleration handoff ---",
            "//--- 2. 状态机主逻辑",
        )
        ramp = section_between("METHOD PRIVATE CalcSegmentRamp", "END_METHOD")
        position = transition[transition.index("ELSIF uiActiveMode = 1 THEN"):]

        self.assertIn("rDecSeg := MAX(ABS(rVelMagnitude - rEndVelMagnitude) / tDec, 1.0)", ramp)
        self.assertIn(
            "rBrakeSpeed := MAX(MAX(rVel0Magnitude, rVelActive), ABS(stHydAxisRef.rActVelocity));",
            position,
        )
        self.assertIn(
            "rDecDist := MAX((rBrakeSpeed * rBrakeSpeed - rEndVelActive * rEndVelActive)",
            position,
        )
        self.assertIn("MAX(rDecSeg, 1.0)", position)
        self.assertIn("ABS(rPosActive - stHydAxisRef.rActPosition)", position)
        self.assertIn(
            "IF ABS(rPosActive - stHydAxisRef.rActPosition) <= (rDecDist * 1.2) THEN",
            position,
        )
        self.assertNotIn("rTransTimer", position)
        self.assertIn("rDec := rDecSeg;", position)
        self.assertIn("bTransDecel := FALSE;", position)
        self.assertIn("iStep := 105;", position)

    def test_terminal_velocity_handoff_uses_in_velocity_or_timer(self):
        transition = section_between(
            "//--- Terminal same-direction deceleration handoff ---",
            "//--- 2. 状态机主逻辑",
        )
        velocity = transition[
            transition.index("IF uiActiveMode = 2 THEN"):
            transition.index("ELSIF uiActiveMode = 1 THEN")
        ]

        self.assertIn("rTransTimer := rTransTimer + rCycleTime;", transition)
        self.assertIn("IF fbMoveVel.InVelocity OR (rTransTimer >= (rTransTime * 1.2))", velocity)
        self.assertIn("rDec := rDecSeg;", velocity)
        self.assertIn("bTransDecel := FALSE;", velocity)
        self.assertNotIn("iStep := 105", velocity)

    def test_terminal_handoff_excludes_reverse_pressure_stop_and_alarm(self):
        transition = section_between(
            "//--- Terminal same-direction deceleration handoff ---",
            "//--- 2. 状态机主逻辑",
        )
        guard = transition[transition.index("IF bTransDecel"):transition.index(" THEN")]

        self.assertIn("(iStep = 100)", guard)
        self.assertIn("NOT bDecelToZero", guard)
        self.assertIn("NOT bAlarm", guard)
        self.assertIn("NOT rTrigStop.Q", guard)
        self.assertIn("NOT rTrigEStop.Q", guard)
        self.assertIn("(uiActiveMode = 1) OR (uiActiveMode = 2)", guard)
        handoff_anchor = "//--- Terminal same-direction deceleration handoff ---"
        self.assertLess(SOURCE.index("rPresActive := rPres;"), SOURCE.index(handoff_anchor))
        self.assertLess(SOURCE.index(handoff_anchor), SOURCE.index("CASE iStep OF"))

    def test_clear_segment_latch_clears_chain_and_direction_state(self):
        clear_latch = section_between("METHOD PRIVATE ClearSegLatch", "END_METHOD")
        normal_stop = section_between("200: // 正常停止", "300: // 急停")
        estop = section_between("300: // 急停", "400: // 报警复位")
        reset = section_between("400: // 报警复位", "900: // 报警等待")

        self.assertIn("rEndVelLastSeg := 0.0", clear_latch)
        self.assertIn("rVelLastSeg := 0.0", clear_latch)
        self.assertIn("iDirActive := 0", clear_latch)
        self.assertIn("iDirTarget := 0", clear_latch)
        self.assertIn("bDecelToZero := FALSE", clear_latch)
        self.assertIn("bSegVelValid := FALSE", clear_latch)
        self.assertIn("ClearSegLatch()", normal_stop)
        self.assertIn("ClearSegLatch()", estop)
        self.assertIn("ClearSegLatch()", reset)

    def test_active_motion_calls_use_latched_pressure_limit(self):
        move_calls = section_between("fbMoveContAbs(", "fbPresHdl(")

        self.assertIn("PRESSURELIMIT := rPresActive", move_calls)
        self.assertIn("PRESSURELIMIT := rPresActive", section_between("fbMoveVel(", "IF iStep = 150 THEN"))
        self.assertIn("PRESSURE := rPres", section_between("fbPresHdl(", "IF iStep = 150 THEN"))

    def test_pressure_ramp_uses_zero_policy_and_rate_cap(self):
        pressure_ramp = section_between(
            "METHOD PUBLIC CalcPresRampRate", "END_METHOD"
        )
        self.assertIn("uiTimeMs = 0", pressure_ramp)
        self.assertIn("800.0", pressure_ramp)

    def test_estop_done_and_error_edges_remain_distinct(self):
        estop_step = section_between("300: // 急停", "400: // 报警复位")
        self.assertIn("fbEStop.DONE", estop_step)
        self.assertIn("fbEStop.ERROR", estop_step)


if __name__ == "__main__":
    unittest.main()
