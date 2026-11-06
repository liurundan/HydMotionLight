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
        segment_trigger = section_between("// 3.1 新段触发", "// 3.2 在线速度更新")
        ramp = section_between("METHOD PRIVATE CalcSegmentRamp", "END_METHOD")

        self.assertIn("iDirActive", declarations)
        self.assertIn("iDirTarget", declarations)
        self.assertIn("bDecelToZero", declarations)
        self.assertIn("IF bDirectionChange THEN", ramp)
        self.assertIn("rDec := rVel0Magnitude / tDec", ramp)
        self.assertIn("iDirActive := iDirLastSeg", ramp)
        self.assertIn("iStep := 150", ramp)
        self.assertIn("iDirActive := iDirTarget", segment_trigger)
        self.assertIn("bDecelToZero", segment_trigger)

    def test_motion_calls_use_active_segment_values_and_direction(self):
        move_calls = section_between("fbMoveContAbs(", "IF iStep = 200 THEN")

        self.assertIn("POSITION := rPosActive", move_calls)
        self.assertIn("VELOCITY := rVelActive", move_calls)
        self.assertIn("ENDVELOCITY := rEndVelActive", move_calls)
        self.assertIn("ENDVELOCITYDIRECTION := iDirActive", move_calls)
        self.assertIn("DIRECTION := iDirActive", move_calls)

    def test_reverse_state_owns_stop_completion_before_switching_direction(self):
        reverse_state = section_between("150: // 反向减速到零", "200: // 正常停止")

        self.assertIn("IF fbStop.DONE THEN", reverse_state)
        self.assertIn("iDirActive := iDirTarget", reverse_state)
        self.assertIn("iStep := 105", reverse_state)
        self.assertLess(reverse_state.index("fbStop.DONE"), reverse_state.index("iDirActive := iDirTarget"))

    def test_clear_segment_latch_clears_chain_and_direction_state(self):
        clear_latch = section_between("METHOD PRIVATE ClearSegLatch", "END_METHOD")

        self.assertIn("rEndVelLastSeg := 0.0", clear_latch)
        self.assertIn("rVelLastSeg := 0.0", clear_latch)
        self.assertIn("iDirActive := 0", clear_latch)
        self.assertIn("iDirTarget := 0", clear_latch)
        self.assertIn("bDecelToZero := FALSE", clear_latch)
        self.assertIn("bSegVelValid := FALSE", clear_latch)

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
