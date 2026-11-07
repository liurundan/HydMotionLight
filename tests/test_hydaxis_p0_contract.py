#!/usr/bin/env python3
"""Contract checks for the time-based FB_HydAxis boundary."""
from pathlib import Path
import unittest

SOURCE = (Path(__file__).resolve().parents[1] / "iec" / "FB_HydAxis.st").read_text(encoding="utf-8")


class HydAxisTimeContractTest(unittest.TestCase):
    def test_iec_passes_times_without_physical_slope_calculation(self):
        self.assertNotIn("CalcPresRampRate", SOURCE)
        self.assertNotIn("rPresLastCycle", SOURCE)
        self.assertNotIn("rPressureRampRate", SOURCE)
        self.assertNotIn("rMaxSpeed/UINT_TO_REAL(uiAcc)", SOURCE)
        self.assertNotIn("rMaxSpeed/UINT_TO_REAL(uiDec)", SOURCE)
        self.assertIn("ACCELERATION := UINT_TO_REAL(uiAcc)", SOURCE)
        self.assertIn("DECELERATION := UINT_TO_REAL(uiDec)", SOURCE)
        self.assertIn("PRESSURERAMPRATE := UINT_TO_REAL(uiPressureAccTime)", SOURCE)

    def test_fixed_cycle_and_outputs_are_explicit(self):
        self.assertIn("rCycleTimeMs : REAL := 1.0", SOURCE)
        self.assertIn("rCycle := rCycle + rCycleTimeMs * 0.001", SOURCE)
        self.assertIn("bPositionReached : BOOL", SOURCE)
        self.assertIn("bPositionReached := fbMoveContAbs.PositionReached", SOURCE)

    def test_emergency_stop_does_not_use_legacy_large_time(self):
        self.assertNotIn("DECELERATION := 99999.0", SOURCE)
        self.assertIn("rEmergencyStopTimeMs : REAL := 100.0", SOURCE)


if __name__ == "__main__":
    unittest.main()
