#!/usr/bin/env python3
"""Hermetic failure-path coverage for Task 3 calibration tools."""
import json
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
REPLAY = Path(sys.argv[1])


def run(*args):
    return subprocess.run(args, cwd=ROOT, text=True, capture_output=True)


with tempfile.TemporaryDirectory() as tmp_name:
    tmp = Path(tmp_name)
    bad_csv = tmp / "nan.csv"
    bad_csv.write_text("h,h,h,h,h,h,h,h\n0,1,1,1,0,10,1,1\n1,nan,1,1,0,10,1,1\n")
    result = run(sys.executable, "scripts/ripple/analyze_open_loop.py", bad_csv, tmp / "out.json")
    assert result.returncode != 0 and "NaN or infinity" in result.stderr

    sentinel = tmp / "pressure_ripple_table.h"
    sentinel.write_text("sentinel\n")
    result = run(sys.executable, "scripts/ripple/export_ripple_table.py",
                 "docs/ripple-analysis/open_loop_summary.json",
                 "docs/ripple-analysis/identified_params.json", sentinel)
    assert result.returncode != 0 and sentinel.read_text() == "sentinel\n"

    long_kv = tmp / "long.kv"
    long_kv.write_text("x=" + "1" * 300 + "\n")
    result = run(REPLAY, "physical", "20", "1", long_kv)
    assert result.returncode != 0 and "model not calibrated" in result.stderr

    uncalibrated = ROOT / "docs/ripple-analysis/identified_params.kv"
    result = run(REPLAY, "physical", "20", "1", uncalibrated)
    assert result.returncode != 0 and "model not calibrated" in result.stderr

    report = json.loads((ROOT / "docs/ripple-analysis/identified_params.json").read_text())
    assert len(report["coordinate_descent_trace"]) == 20
    assert report["training_windows"] != report["validation_windows"]
    assert isinstance(report["training_objective"]["value"], float)
