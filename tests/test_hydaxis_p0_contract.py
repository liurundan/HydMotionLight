#!/usr/bin/env python3
"""Source-contract checks for critical FB_HydAxis process-layer behavior."""

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
    def test_negative_direction_from_rest_is_an_acceleration_transition(self):
        ramp = section_between(
            "METHOD PRIVATE CalcSegmentRamp", "END_METHOD"
        )

        self.assertIn("rVel0Magnitude : REAL;", ramp)
        self.assertIn("bDirectionChange : BOOL;", ramp)
        self.assertIn("rVel0Magnitude := CalcStartVelocity();", ramp)
        self.assertIn("rVel0Magnitude > rDeltaVDeadZone", ramp)
        self.assertIn("IF bDirectionChange THEN", ramp)
        self.assertIn("rDeltaV := -rVel0Magnitude;", ramp)
        self.assertIn("rDeltaV := rVel - rVel0Magnitude;", ramp)

    def test_position_segment_parameters_are_latched_and_used(self):
        declarations = section_between("rPos : REAL;", "END_VAR")
        position_logic = section_between("// 3.1 新段触发", "// 3.2 在线速度更新")
        move_call = section_between("fbMoveContAbs(", "fbMoveVel(")

        self.assertIn("rPosActive", declarations)
        self.assertIn("rVelActive", declarations)
        self.assertIn("rEndVelActive", declarations)
        self.assertIn("iDirActive", declarations)
        self.assertIn("rPosActive := rPos", position_logic)
        self.assertIn("rVelActive := rVel", position_logic)
        self.assertIn("rEndVelActive := rEndVel", position_logic)
        self.assertIn("POSITION   := rPosActive", move_call)
        self.assertIn("VELOCITY   := rVelActive", move_call)
        self.assertIn("ENDVELOCITY:= rEndVelActive", move_call)
        self.assertIn("ENDVELOCITYDIRECTION := iDirActive", move_call)

    def test_status_output_exposes_library_state_and_clears_band_flags(self):
        outputs = section_between("VAR_OUTPUT", "END_VAR")
        status_read = section_between(
            "fbHYD_ReadStatus(", "//=====================================================================\n  // 9. 状态输出汇总"
        )
        output_logic = section_between("// 9. 状态输出汇总", "END_FUNCTION_BLOCK")

        self.assertIn("uiAxisState", outputs)
        self.assertIn("uiAxisState := iAxisState", status_read)
        self.assertIn("bInVelocity := FALSE", output_logic)
        self.assertIn("bInPressure := FALSE", output_logic)
        self.assertIn("2:  bDone := fbMoveVel.InVelocity", output_logic)
        self.assertIn("3:  bDone := fbPresHdl.INPRESSURE", output_logic)

    def test_pump_limit_is_shared_and_max_speed_validated(self):
        startup = section_between("fbCalcMaxSpeed(", "IF iMechBaseClass = 1 THEN")

        self.assertIn("rPumpMaxRPM", startup)
        self.assertIn("SetPumpConfig( rPumpDisplacement, rPumpMaxRPM )", startup)
        self.assertIn("IF NOT fbCalcMaxSpeed.bValid THEN", startup)

    def test_pressure_ramp_restarts_from_actual_pressure(self):
        pressure_logic = section_between(
            "// 6. 压力斜率动态计算", "// 7. 底层 FB 统一调用区"
        )

        self.assertIn("rPrevPres   := stHydAxisRef.rActPressure", pressure_logic)
        self.assertIn("rPresLastCycle := stHydAxisRef.rActPressure", pressure_logic)

    def test_end_velocity_uses_selected_direction_speed_limit(self):
        conversion = section_between(
            "rPres    :=", "rVelSigned :="
        )
        speed_selection_end = conversion.index("END_IF", conversion.index("// 2. 方向相关最大速度"))
        end_velocity_index = conversion.index("rEndVel  :=")
        velocity_index = conversion.index("rVel :=")

        self.assertGreater(end_velocity_index, speed_selection_end)
        self.assertLess(end_velocity_index, velocity_index)

    def test_zero_end_velocity_waits_for_runtime_stop_before_done(self):
        declarations = section_between("rEndVelLastSeg : REAL", "rVel0 : REAL")
        clear_latch = section_between("METHOD PRIVATE ClearSegLatch", "END_METHOD")
        new_segment = section_between("IF bNewSeg THEN", "// 3.2 在线速度更新")
        running_state = section_between("100: // 运行中", "105: // 运行过渡步")
        status_output = section_between("// 9. 状态输出汇总", "END_FUNCTION_BLOCK")

        self.assertIn("bZeroEndStopDone : BOOL := FALSE;", declarations)
        self.assertIn("bZeroEndStopDone := FALSE", clear_latch)
        self.assertIn("bZeroEndStopDone := FALSE", new_segment)
        self.assertIn("rEndVelLastSeg <= 0.001", running_state)
        self.assertIn("fbMoveContAbs.InEndVelocity", running_state)
        self.assertIn("iStep := 10", running_state)
        self.assertLess(
            running_state.index("ELSIF rTrigStart.Q"),
            running_state.index("ELSIF (uiActiveMode = 1)"),
        )
        self.assertIn("bDone := bZeroEndStopDone", status_output)

    def test_transition_step_preserves_start_stop_and_estop_edges(self):
        segment_trigger = section_between("bNewSeg :=", "IF bNewSeg THEN")
        transition_step = section_between("105: // 运行过渡步", "200: // 正常停止")

        self.assertIn("iStep = 105", segment_trigger)
        self.assertIn("rTrigEStop.Q", transition_step)
        self.assertIn("rTrigStop.Q", transition_step)
        self.assertIn("rTrigStart.Q", transition_step)

    def test_pressure_ramp_uses_zero_policy_and_rate_cap(self):
        pressure_ramp = section_between(
            "METHOD PUBLIC CalcPresRampRate", "END_METHOD"
        )
        self.assertIn("uiTimeMs = 0", pressure_ramp)
        self.assertIn("MIN(", pressure_ramp)
        self.assertIn("800.0", pressure_ramp)
        self.assertNotIn("MAX(uiPressureAccTime, 1)", SOURCE)

    def test_estop_error_is_separate_and_keeps_underlying_error_id(self):
        estop_step = section_between("300: // 急停", "400: // 报警复位")
        self.assertIn("IF fbEStop.DONE THEN", estop_step)
        self.assertIn("ELSIF fbEStop.ERROR THEN", estop_step)
        self.assertIn("fbEStop.ERRORID", estop_step)
        self.assertNotIn("fbEStop.DONE OR fbEStop.ERROR", estop_step)


if __name__ == "__main__":
    unittest.main()
