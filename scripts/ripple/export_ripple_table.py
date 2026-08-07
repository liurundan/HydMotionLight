#!/usr/bin/env python3
"""Atomically export a table only from a separately passing validation report."""
import json
import math
import os
import sys
from pathlib import Path

def fail(message):
    print("model not calibrated: " + message, file=sys.stderr)
    return 1

def main(argv):
    if len(argv) != 4:
        print("usage: export_ripple_table.py SUMMARY.json PARAMS.json OUTPUT.h", file=sys.stderr); return 2
    try:
        summary = json.loads(Path(argv[1]).read_text(encoding="utf-8"))
        params_path = Path(argv[2]); params = json.loads(params_path.read_text(encoding="utf-8"))
        manifest = json.loads((params_path.parent / "physical_parameter_manifest.json").read_text(encoding="utf-8"))
        validation = json.loads((params_path.parent / "model_validation.json").read_text(encoding="utf-8"))
        ids = {summary.get("calibration_id"), params.get("calibration_id"), manifest.get("calibration_id"), validation.get("calibration_id")}
        if len(ids) != 1 or None in ids or any(item.get("schema_version") != 1
            for item in (summary, params, manifest, validation)):
            return fail("schema or calibration ID mismatch")
        if len({summary.get("calibration_status"), params.get("calibration_status"),
                manifest.get("calibration_status"), validation.get("calibration_status")}) != 1 or \
           len({summary.get("source_sha256"), params.get("source_sha256"),
                manifest.get("source_sha256"), validation.get("source_sha256")}) != 1 or \
           validation.get("source_sha256") != params.get("source_sha256") or \
           validation.get("manifest_provenance_sha256") != manifest.get("manifest_provenance_sha256") or \
           validation.get("calibration_status") != "calibrated" or \
           not all(validation.get("gates", {}).get(key) is True for key in
                   ("mean_pressure_error", "order13_amplitude_error", "order13_phase_error",
                    "order26_amplitude_error", "provenance_complete")):
            return fail("validation gates are incomplete")
        entries = validation.get("rpm_table_entries")
        if not isinstance(entries, list) or not entries:
            return fail("validated RPM-domain compensation entries are missing")
        rows = []
        for entry in entries:
            values = (entry["rpm"], entry["amp13_rpm"], entry["phase13_rad"],
                      entry["amp26_rpm"], entry["phase26_rad"])
            if not all(isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)
                       for value in values):
                return fail("RPM-domain table contains non-numeric values")
            if not (-2000.0 <= values[0] <= 2000.0) or (rows and values[0] <= rows[-1][0]):
                return fail("RPM breakpoints must be strictly increasing within the admitted domain")
            rows.append(values)
        lines = ["/* generated from model_validation.json */", "#ifndef PRESSURE_RIPPLE_TABLE_H",
                 "#define PRESSURE_RIPPLE_TABLE_H", "#define PRESSURE_RIPPLE_TABLE_COUNT %d" % len(rows),
                 "typedef struct { float rpm; float amp13_rpm; float phase13_rad; float amp26_rpm; float phase26_rad; } HYD_PressureRippleEntry;",
                 "static const HYD_PressureRippleEntry HYD_PRESSURE_RIPPLE_TABLE[] = {"]
        lines.extend("    {%.9gF, %.9gF, %.9gF, %.9gF, %.9gF}," % row for row in rows)
        lines.extend(["};", "#endif /* PRESSURE_RIPPLE_TABLE_H */", ""])
        output = Path(argv[3]); output.parent.mkdir(parents=True, exist_ok=True)
        temporary = output.with_name(output.name + ".tmp")
        temporary.write_text("\n".join(lines), encoding="ascii")
        os.replace(temporary, output)
        return 0
    except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
        return fail(str(exc))
if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
