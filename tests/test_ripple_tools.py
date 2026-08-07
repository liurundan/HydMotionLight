#!/usr/bin/env python3
"""Hermetic calibrated-artifact admission tests for the ripple toolchain."""
import hashlib
import json
import math
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.dont_write_bytecode = True
sys.path.insert(0, str(ROOT / "scripts" / "ripple"))
from fit_physical_model import DEFAULTS, PARAM_BOUNDS, canonical_json, canonical_lines

REPLAY = Path(sys.argv[1])


def run(*args):
    return subprocess.run(args, cwd=ROOT, text=True, capture_output=True)


def write_json(path, value):
    path.write_text(json.dumps(value, sort_keys=True, separators=(",", ":"),
                               allow_nan=True), encoding="utf-8")


def build_bundle(directory):
    status = "calibrated"
    source_hash = "a" * 64
    parameters = dict(DEFAULTS)
    manifest_parameters = {
        key: {"value": value, "unit": "SI", "source": "fixture", "acquisition_window": "bench"}
        for key, value in parameters.items() if key not in PARAM_BOUNDS
    }
    manifest_payload = {"schema_version": 1, "calibration_status": status,
                        "source_sha256": source_hash, "parameters": manifest_parameters,
                        "missing_provenance": []}
    manifest_hash = hashlib.sha256(canonical_json(manifest_payload).encode()).hexdigest()
    calibration_id = hashlib.sha256(
        canonical_lines(parameters, status, source_hash, manifest_hash).encode("ascii")).hexdigest()
    summary = {"schema_version": 1, "calibration_id": calibration_id,
               "calibration_status": status, "source_sha256": source_hash, "segments": []}
    summary["summary_sha256"] = hashlib.sha256(
        canonical_json(summary, ("summary_sha256",)).encode()).hexdigest()
    manifest = dict(manifest_payload)
    manifest.update({"calibration_id": calibration_id, "manifest_provenance_sha256": manifest_hash})
    params = {"schema_version": 1, "calibration_id": calibration_id,
              "calibration_status": status, "source_sha256": source_hash,
              "manifest_provenance_sha256": manifest_hash,
              "summary_sha256": summary["summary_sha256"], "parameters": parameters}
    kv = directory / "params.kv"
    kv.write_text("schema_version=1\ncalibration_id=%s\n%s" %
                  (calibration_id, canonical_lines(parameters, status, source_hash, manifest_hash)
                   .split("\n", 1)[1]), encoding="ascii")
    validation = {
        "schema_version": 1, "calibration_id": calibration_id, "calibration_status": status,
        "source_sha256": source_hash, "manifest_provenance_sha256": manifest_hash,
        "gates": {"mean_pressure_error": True, "order13_amplitude_error": True,
                  "order13_phase_error": True, "order26_amplitude_error": True,
                  "provenance_complete": True},
        "heldout_metrics": {"mean_pressure_error_bar": 1.0, "mean_pressure_reference_bar": 100.0,
                            "order13_amplitude_relative_error": .1,
                            "order13_phase_error_deg": 5.0, "order26_amplitude_relative_error": .1},
        "rpm_table_entries": [{"rpm": 10.0, "amp13_rpm": .5, "phase13_rad": .1,
                               "amp26_rpm": -.25, "phase26_rad": -.2}],
        "input_digests": {"calibration_id": calibration_id,
                          "manifest_provenance_sha256": manifest_hash,
                          "summary_sha256": summary["summary_sha256"],
                          "kv_sha256": hashlib.sha256(kv.read_bytes()).hexdigest()},
    }
    write_json(directory / "summary.json", summary)
    write_json(directory / "params.json", params)
    write_json(directory / "physical_parameter_manifest.json", manifest)
    write_json(directory / "model_validation.json", validation)
    return kv


def export(directory, output):
    return run(sys.executable, "scripts/ripple/export_ripple_table.py",
               directory / "summary.json", directory / "params.json", output)


def assert_rejected_after_mutation(mutator):
    with tempfile.TemporaryDirectory() as name:
        directory = Path(name)
        build_bundle(directory)
        table = directory / "table.h"
        table.write_text("sentinel\n")
        mutator(directory)
        result = export(directory, table)
        assert result.returncode != 0, result.stderr
        assert table.read_text() == "sentinel\n"


with tempfile.TemporaryDirectory() as name:
    directory = Path(name)
    kv = build_bundle(directory)
    replay = run(REPLAY, "physical", "20", "1", kv)
    assert replay.returncode == 0 and "# calibration_status=calibrated" in replay.stdout, replay.stderr
    table = directory / "table.h"
    result = export(directory, table)
    assert result.returncode == 0 and "PRESSURE_RIPPLE_TABLE_COUNT 1" in table.read_text(), result.stderr
    tu = directory / "table.c"
    tu.write_text('#include "table.h"\nint main(void) { return 0; }\n')
    compile_result = subprocess.run(["gcc", "-std=c99", "-Wall", "-Wextra", "-Werror",
                                     "-I", str(directory), "-c", str(tu), "-o", str(directory / "table.o")],
                                    text=True, capture_output=True)
    assert compile_result.returncode == 0, compile_result.stderr

assert_rejected_after_mutation(lambda d: (d / "params.kv").write_text(
    (d / "params.kv").read_text().replace("calibration_status=calibrated",
                                           "calibration_status=model not calibrated")))
assert_rejected_after_mutation(lambda d: (d / "params.kv").write_text(
    (d / "params.kv").read_text().replace("source_sha256=" + "a" * 64, "source_sha256=" + "b" * 64)))
assert_rejected_after_mutation(lambda d: (d / "physical_parameter_manifest.json").write_text(
    (d / "physical_parameter_manifest.json").read_text().replace("\"source\":\"fixture\"",
                                                                   "\"source\":\"mutated\"")))
assert_rejected_after_mutation(lambda d: (d / "physical_parameter_manifest.json").write_text(
    (d / "physical_parameter_manifest.json").read_text().replace("\"manifest_provenance_sha256\":\"",
                                                                   "\"manifest_provenance_sha256\":\"bad")))
assert_rejected_after_mutation(lambda d: (d / "params.json").write_text(
    (d / "params.json").read_text().replace("\"gas_fraction\":0.002", "\"gas_fraction\":0.003")))
assert_rejected_after_mutation(lambda d: (d / "model_validation.json").write_text(
    (d / "model_validation.json").read_text().replace("\"calibration_id\":\"", "\"calibration_id\":\"bad")))
assert_rejected_after_mutation(lambda d: (d / "model_validation.json").write_text(
    (d / "model_validation.json").read_text().replace("\"mean_pressure_error_bar\":1.0",
                                                        "\"mean_pressure_error_bar\":-1.0")))
assert_rejected_after_mutation(lambda d: (d / "summary.json").write_text(
    (d / "summary.json").read_text().replace("\"segments\":[]", "\"segments\":[1]")))
assert_rejected_after_mutation(lambda d: (d / "model_validation.json").write_text(
    (d / "model_validation.json").read_text().replace("\"summary_sha256\":\"",
                                                        "\"summary_sha256\":\"bad")))
assert_rejected_after_mutation(lambda d: (d / "model_validation.json").write_text(
    (d / "model_validation.json").read_text().replace("\"rpm\":10.0", "\"rpm\":-1.0")))
assert_rejected_after_mutation(lambda d: (d / "model_validation.json").write_text(
    (d / "model_validation.json").read_text().replace("\"order13_phase_error_deg\":5.0",
                                                        "\"order13_phase_error_deg\":16.0")))
assert_rejected_after_mutation(lambda d: (d / "model_validation.json").write_text(
    (d / "model_validation.json").read_text().replace("\"order13_amplitude_relative_error\":0.1",
                                                        "\"order13_amplitude_relative_error\":0.21")))
assert_rejected_after_mutation(lambda d: (d / "model_validation.json").write_text(
    (d / "model_validation.json").read_text().replace("\"order26_amplitude_relative_error\":0.1",
                                                        "\"order26_amplitude_relative_error\":0.31")))
for replacement in ("NaN", "Infinity", "7.0", "4.0"):
    def mutate(directory, replacement=replacement):
        path = directory / "model_validation.json"
        text = path.read_text()
        if replacement == "NaN":
            text = text.replace("\"amp13_rpm\":0.5", "\"amp13_rpm\":NaN")
        elif replacement == "Infinity":
            text = text.replace("\"phase13_rad\":0.1", "\"phase13_rad\":Infinity")
        elif replacement == "7.0":
            text = text.replace("\"phase13_rad\":0.1", "\"phase13_rad\":7.0")
        else:
            text = text.replace("\"amp13_rpm\":0.5", "\"amp13_rpm\":4.0")
        path.write_text(text)
    assert_rejected_after_mutation(mutate)
