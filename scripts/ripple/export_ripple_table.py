#!/usr/bin/env python3
"""Atomically export a table only from a separately passing validation report."""
import json
import math
import os
import sys
from pathlib import Path
import hashlib

def fail(message):
    print("model not calibrated: " + message, file=sys.stderr)
    return 1

def canonical_lines(parameters, status, source_hash, manifest_hash):
    return ("schema_version=1\ncalibration_status=%s\nsource_sha256=%s\n"
            "manifest_provenance_sha256=%s\n" % (status, source_hash, manifest_hash)) + \
        "".join("%s=%.17g\n" % (key, parameters[key]) for key in sorted(parameters))
def canonical_json(value, excluded=()):
    if isinstance(value, dict):
        value = {key: item for key, item in value.items() if key not in excluded}
    return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False)

def read_kv(path):
    values = {}
    for raw in Path(path).read_text(encoding="ascii").splitlines():
        if raw.count("=") != 1:
            raise ValueError("malformed KV")
        key, value = raw.split("=", 1)
        if key in values:
            raise ValueError("duplicate KV field")
        values[key] = value
    required = {"schema_version", "calibration_id", "calibration_status",
                "source_sha256", "manifest_provenance_sha256"}
    if not required.issubset(values) or values["schema_version"] != "1":
        raise ValueError("incomplete KV identity")
    params = {key: float(value) for key, value in values.items() if key not in required}
    if not all(math.isfinite(value) for value in params.values()):
        raise ValueError("nonfinite KV parameter")
    canonical = canonical_lines(params, values["calibration_status"], values["source_sha256"],
                                values["manifest_provenance_sha256"])
    if hashlib.sha256(canonical.encode("ascii")).hexdigest() != values["calibration_id"]:
        raise ValueError("KV calibration SHA-256 mismatch")
    return values, params

def main(argv):
    if len(argv) != 4:
        print("usage: export_ripple_table.py SUMMARY.json PARAMS.json OUTPUT.h", file=sys.stderr); return 2
    try:
        summary = json.loads(Path(argv[1]).read_text(encoding="utf-8"))
        params_path = Path(argv[2]); params = json.loads(params_path.read_text(encoding="utf-8"))
        manifest = json.loads((params_path.parent / "physical_parameter_manifest.json").read_text(encoding="utf-8"))
        validation = json.loads((params_path.parent / "model_validation.json").read_text(encoding="utf-8"))
        kv, kv_params = read_kv(params_path.with_suffix(".kv"))
        manifest_digest = hashlib.sha256(
            canonical_json(manifest, ("manifest_provenance_sha256", "calibration_id")).encode("utf-8")).hexdigest()
        summary_digest = hashlib.sha256(canonical_json(summary, ("summary_sha256",)).encode("utf-8")).hexdigest()
        ids = {summary.get("calibration_id"), params.get("calibration_id"), manifest.get("calibration_id"), validation.get("calibration_id")}
        if len(ids) != 1 or None in ids or any(item.get("schema_version") != 1
            for item in (summary, params, manifest, validation)):
            return fail("schema or calibration ID mismatch")
        if kv["calibration_id"] != params["calibration_id"] or \
           kv["calibration_status"] != params["calibration_status"] or \
           kv["source_sha256"] != params["source_sha256"] or \
           kv["manifest_provenance_sha256"] != manifest_digest or \
           params.get("manifest_provenance_sha256") != manifest_digest or \
           params.get("summary_sha256") != summary_digest or summary.get("summary_sha256") != summary_digest or \
           set(kv_params) != set(params.get("parameters", {})) or \
           any(kv_params[key] != params["parameters"][key] for key in kv_params):
            return fail("KV integrity anchor does not match JSON parameters")
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
        metrics = validation.get("heldout_metrics", {})
        threshold = metrics.get("mean_pressure_error_bar")
        reference = metrics.get("mean_pressure_reference_bar")
        amp13 = metrics.get("order13_amplitude_relative_error")
        phase13 = metrics.get("order13_phase_error_deg")
        amp26 = metrics.get("order26_amplitude_relative_error")
        numeric = (threshold, reference, amp13, phase13, amp26)
        if not all(isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)
                   for value in numeric) or reference <= 0.0 or any(value < 0.0 for value in
                   (threshold, amp13, phase13, amp26)) or threshold > max(.05 * reference, 5.0) or \
           amp13 > .20 or phase13 > 15.0 or amp26 > .30:
            return fail("held-out numeric calibration thresholds failed")
        digests = validation.get("input_digests", {})
        if digests.get("calibration_id") != params["calibration_id"] or \
           digests.get("manifest_provenance_sha256") != manifest_digest or \
           digests.get("summary_sha256") != summary_digest or \
           digests.get("kv_sha256") != hashlib.sha256(params_path.with_suffix(".kv").read_bytes()).hexdigest():
            return fail("validation input digest graph mismatch")
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
            if not (0.0 <= values[0] <= 2000.0) or (rows and values[0] <= rows[-1][0]):
                return fail("RPM breakpoints must be strictly increasing within the admitted domain")
            if not (-math.pi <= values[2] <= math.pi and -math.pi <= values[4] <= math.pi) or \
               abs(values[1]) > .30 * values[0] or abs(values[3]) > .30 * values[0]:
                return fail("table phase or RPM-domain amplitude is outside the admitted contract")
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
