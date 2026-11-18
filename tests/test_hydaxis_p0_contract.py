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

    def test_toggle_flow_percent_mode_contract(self):
        # spec 2026-11-18 §4.4
        self.assertIn("bFlowPctMode : BOOL := TRUE", SOURCE)
        self.assertIn("bFlowPctSeg : BOOL", SOURCE)
        self.assertIn("rToggleVelCap : REAL := 700.0", SOURCE)
        self.assertIn("rPumpFlowPct := LIMIT(0.0, UINT_TO_REAL(uiSpd)*0.1, 100.0)", SOURCE)
        self.assertIn("PUMPFLOWPERCENT := rPumpFlowPct", SOURCE)
        # 目标点标尺：XFROM = XTO = rPos
        self.assertIn("XFROM := SEL(bFlowPctSeg, stHydAxisRef.rActPosition, rPos)", SOURCE)
        # PressureHandle 仍用带 5% 下限的 rFlowPercent
        self.assertIn("FLOWLIMITPERCENT:=rFlowPercent", SOURCE)
        self.assertIn("rFlowPercent := LIMIT(5, rFlowPercent, 100)", SOURCE)
        # MoveVelocity 不受百分比模式影响：fbMoveVel 仍用旧标尺 rVel
        self.assertIn("rVelContAbs := SEL(rPumpFlowPct > 0.0, 0.0, rToggleVelCap)", SOURCE)
        self.assertIn("rVelContAbs := rVel;", SOURCE)
        self.assertIn("VELOCITY := rVelContAbs", SOURCE)
        self.assertIn("CONTINUOUSUPDATE := TRUE, VELOCITY := rVel,", SOURCE)
        self.assertNotIn("rVel := SEL(", SOURCE)


if __name__ == "__main__":
    unittest.main()
