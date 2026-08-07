#!/usr/bin/env python3
"""Export a production table only after explicit calibration gates pass."""
import json
import math
import sys
from pathlib import Path


def main(argv):
    if len(argv) != 4:
        print("usage: export_ripple_table.py SUMMARY.json PARAMS.json OUTPUT.h", file=sys.stderr)
        return 2
    output = Path(argv[3])
    try:
        with Path(argv[1]).open("r", encoding="utf-8") as handle:
            summary = json.load(handle)
        with Path(argv[2]).open("r", encoding="utf-8") as handle:
            params = json.load(handle)
        gates = params.get("gates", {})
        required = ("mean_pressure_error", "order13_amplitude_error",
                    "order13_phase_error", "order26_amplitude_error",
                    "provenance_complete")
        if params.get("schema_version") != 1 or summary.get("schema_version") != 1 or \
                any(gates.get(key) is not True for key in required) or \
                params.get("calibration_status") != "calibrated":
            print("model not calibrated: production ripple table was not generated", file=sys.stderr)
            if output.exists():
                output.unlink()
            return 1
        rows = []
        for segment in summary["segments"]:
            orders = segment["orders"]
            rows.append((segment["set_rpm"], orders["13"]["amplitude_bar"],
                         orders["13"]["phase_rad"], orders["26"]["amplitude_bar"],
                         orders["26"]["phase_rad"]))
        lines = [
            "/* generated only from a passing calibration artifact */",
            "#ifndef PRESSURE_RIPPLE_TABLE_H",
            "#define PRESSURE_RIPPLE_TABLE_H",
            "#define PRESSURE_RIPPLE_TABLE_COUNT %d" % len(rows),
            "typedef struct { float rpm; float amp13_rpm; float phase13_rad; float amp26_rpm; float phase26_rad; } HYD_PressureRippleEntry;",
            "static const HYD_PressureRippleEntry HYD_PRESSURE_RIPPLE_TABLE[] = {",
        ]
        lines.extend("    {%.9gF, %.9gF, %.9gF, %.9gF, %.9gF}," % row for row in rows)
        lines.extend(["};", "#endif /* PRESSURE_RIPPLE_TABLE_H */", ""])
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text("\n".join(lines), encoding="ascii")
        return 0
    except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError) as exc:
        print("model not calibrated: %s" % exc, file=sys.stderr)
        if output.exists():
            output.unlink()
        return 1


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
