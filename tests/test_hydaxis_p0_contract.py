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
