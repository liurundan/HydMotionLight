#!/usr/bin/env python3
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "check_interface_layout_consistency.py"
XML = ROOT / "pousHydMotion.xml"
HEADER = ROOT / "include" / "motion_interface.h"
BAD_XML = ROOT / "tests" / "fixtures" / "pous_layout_bad.xml"
PLC_SOURCE = ROOT / "tests" / "plcdemo" / "POUS.c"


def run_check(xml_path: Path, header_path: Path) -> subprocess.CompletedProcess:
    return subprocess.run(
        [sys.executable, str(SCRIPT), "--xml", str(xml_path), "--header", str(header_path)],
        capture_output=True,
        text=True,
    )


def main() -> int:
    if not SCRIPT.exists():
        print(f"missing script: {SCRIPT}")
        return 1

    good = run_check(XML, HEADER)
    if good.returncode != 0:
        print("expected repository interface layout check to pass")
        print(good.stdout)
        print(good.stderr)
        return 1

    bad = run_check(BAD_XML, HEADER)
    if bad.returncode == 0:
        print("expected bad fixture layout check to fail")
        return 1

    if "HYD_Stop" not in bad.stdout and "HYD_Stop" not in bad.stderr:
        print("expected failing output to mention HYD_Stop mismatch")
        print(bad.stdout)
        print(bad.stderr)
        return 1

    plc_source = PLC_SOURCE.read_text(encoding="utf-8")
    init_start = plc_source.find("void HYD_READSTATUS_init__")
    init_end = plc_source.find("// Code part", init_start)
    if init_start < 0 or init_end < 0:
        print("expected generated HYD_READSTATUS initializer")
        return 1
    if "PRESSURECONTROLLERAPPLIED" not in plc_source[init_start:init_end]:
        print("expected generated HYD_READSTATUS initializer to clear PRESSURECONTROLLERAPPLIED")
        return 1

    print("interface layout consistency tests passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
