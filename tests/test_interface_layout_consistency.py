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
PLC_HEADER = ROOT / "tests" / "plcdemo" / "POUS.h"


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

    generated = run_check(XML, PLC_HEADER)
    if generated.returncode != 0:
        print("expected generated-equivalent interface layout check to pass")
        print(generated.stdout)
        print(generated.stderr)
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
    plc_header = PLC_HEADER.read_text(encoding="utf-8")
    xml_source = XML.read_text(encoding="utf-8")
    c_header = HEADER.read_text(encoding="utf-8")

    required_pous = {
        "HYD_ConfigureToggleMechanism": "HYD_CONFIGURETOGGLEMECHANISM",
        "HYD_ReadToggleMechanism": "HYD_READTOGGLEMECHANISM",
        "HYD_SetPumpFeedback": "HYD_SETPUMPFEEDBACK",
    }
    for pou_name, c_name in required_pous.items():
        if f'<pou name="{pou_name}"' not in xml_source:
            print(f"expected XML POU {pou_name}")
            return 1
        if c_name not in c_header or c_name not in plc_header:
            print(f"expected C and generated-equivalent layouts for {c_name}")
            return 1

    pump_feedback_body = "void HYD_SETPUMPFEEDBACK_body__"
    if pump_feedback_body not in plc_source:
        print("expected generated HYD_SETPUMPFEEDBACK body")
        return 1
    if "__mcl_cmd_SetPumpFeedback" not in plc_source:
        print("expected generated HYD_SETPUMPFEEDBACK command call")
        return 1

    create_start = plc_source.find("void HYD_CREATEMOTION_init__")
    create_end = plc_source.find("// Code part", create_start)
    if create_start < 0 or create_end < 0:
        print("expected generated HYD_CREATEMOTION initializer")
        return 1
    create_init = plc_source[create_start:create_end]
    for field in (
        "MECHANISM_TYPE",
        "_RESERVED_AXIS",
        "_RESERVED_SLOT",
        "_VALIDATION_TOKEN",
        "_CREATE_ACTIVE",
        "_FRAMEWORK_GENERATION",
    ):
        if field not in create_init:
            print(f"expected generated HYD_CREATEMOTION initializer for {field}")
            return 1
    for sentinel in (
        "_RESERVED_AXIS,-1",
        "_RESERVED_SLOT,255",
        "_VALIDATION_TOKEN,255",
    ):
        if sentinel not in create_init:
            print(f"expected generated CreateMotion sentinel {sentinel}")
            return 1

    configure_start = plc_source.find("void HYD_CONFIGURETOGGLEMECHANISM_init__")
    configure_end = plc_source.find("// Code part", configure_start)
    if configure_start < 0 or configure_end < 0:
        print("expected generated HYD_CONFIGURETOGGLEMECHANISM initializer")
        return 1
    configure_init = plc_source[configure_start:configure_end]
    for field in (
        "DONE", "BUSY", "ERROR", "ERRORID", "CONFIG_VERSION",
        "VALIDATION_TOKEN", "ACTIVE", "CONFIG_AXIS", "_FRAMEWORK_GENERATION",
    ):
        if field not in configure_init:
            print(f"expected generated configure initializer for {field}")
            return 1
    for sentinel in ("VALIDATION_TOKEN,255", "CONFIG_AXIS,-1"):
        if sentinel not in configure_init:
            print(f"expected generated configure sentinel {sentinel}")
            return 1

    read_start = plc_source.find("void HYD_READTOGGLEMECHANISM_init__")
    read_end = plc_source.find("// Code part", read_start)
    if read_start < 0 or read_end < 0:
        print("expected generated HYD_READTOGGLEMECHANISM initializer")
        return 1
    read_init = plc_source[read_start:read_end]
    for field in ("VALID", "USING_DEFAULTS", "CONFIG_VERSION", "ERROR", "ERRORID"):
        if field not in read_init:
            print(f"expected generated read initializer for {field}")
            return 1

    init_start = plc_source.find("void HYD_READSTATUS_init__")
    init_end = plc_source.find("// Code part", init_start)
    if init_start < 0 or init_end < 0:
        print("expected generated HYD_READSTATUS initializer")
        return 1
    if "PRESSURECONTROLLERAPPLIED" not in plc_source[init_start:init_end]:
        print("expected generated HYD_READSTATUS initializer to clear PRESSURECONTROLLERAPPLIED")
        return 1
    for field in (
        "MECHANISMTYPE",
        "ACTUATORDIRECTION",
        "ACTUATORPOSITION",
        "ACTUATORVELOCITYCOMMAND",
        "VELOCITYRATIO",
        "MECHANISMCONFIGVERSION",
    ):
        if field not in plc_source[init_start:init_end]:
            print(f"expected generated HYD_READSTATUS initializer to clear {field}")
            return 1

    print("interface layout consistency tests passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
