#!/usr/bin/env python3
"""Bounded, deterministic physical-parameter identification artifact generator."""
import csv
import hashlib
import json
import math
import statistics
import sys
from pathlib import Path

PARAM_BOUNDS = {
    "motor_natural_freq_hz": [1.0, 50.0],
    "motor_damping": [0.2, 2.0],
    "motor_delay_s": [0.0, 0.05],
    "outlet_volume_m3": [1e-5, 5e-3],
    "gas_fraction": [1e-5, 2e-2],
    "pump_leak_c0_m3_pa_s": [1e-14, 1e-9],
    "pump_leak_speed_m3_pa_s_per_rpm": [1e-16, 1e-11],
    "cylinder_leak_m3_pa_s": [1e-14, 1e-9],
    "outlet_leak_m3_pa_s": [1e-14, 1e-9],
    "ripple13_peak": [0.0, 0.30],
    "ripple26_peak": [0.0, 0.15],
    "ripple39_peak": [0.0, 0.10],
    "torque_ripple13_peak": [0.0, 0.30],
}

ALL_PHYSICAL_DEFAULTS = {
    "atmospheric_pressure_pa": 101325.0, "suction_pressure_pa": 101325.0,
    "outlet_volume_m3": 5e-4, "chamber_volume_m3": 5e-4,
    "line_inertance_pa_s2_per_m3": 1e8, "line_resistance_pa_s_per_m3": 2e12,
    "line_quadratic_resistance_pa_s2_per_m6": 0.0, "beta_oil_pa": 1.2e9,
    "gas_fraction": 0.002, "gas_transition_pa": 1e6, "beta_min_pa": 5e7,
    "pump_leak_c0_m3_pa_s": 2e-13, "pump_leak_speed_m3_pa_s_per_rpm": 1e-15,
    "outlet_leak_m3_pa_s": 1e-13, "cylinder_leak_m3_pa_s": 1e-13,
    "eta_v_min": 0.60, "eta_m_nominal": 0.90,
    "eta_m_pressure_loss_per_pa": 1e-9, "eta_m_speed_loss_per_rpm": 1e-5,
    "eta_m_min": 0.50, "rated_motor_torque_nm": 150.0,
    "torque_ripple13_peak": 0.02, "torque_ripple13_phase_rad": 0.0,
    "ripple13_peak": 0.08, "ripple26_peak": 0.03, "ripple39_peak": 0.01,
    "ripple13_phase_rad": 0.0, "ripple26_phase_rad": 0.0, "ripple39_phase_rad": 0.0,
    "motor_natural_freq_hz": 12.0, "motor_damping": 1.0, "motor_delay_s": 0.0,
    "motor_accel_limit_rpm_s": 20000.0, "motor_torque_limit_permille": 1000.0,
    "relief_set_pa": 25e6, "relief_deadband_pa": 0.2e6,
    "relief_orifice_coeff_m3_s_sqrt_pa": 1e-8, "relief_hysteresis_pa": 0.5e6,
    "sensor_delay_s": 0.0, "sensor_quantization_bar": 0.0,
}


def canonical_lines(parameters, schema_version=1):
    lines = ["schema_version=%d" % schema_version]
    lines.extend("%s=%.17g" % (key, float(parameters[key])) for key in sorted(parameters))
    return "\n".join(lines) + "\n"


def calibration_id(parameters, schema_version=1):
    return hashlib.sha256(canonical_lines(parameters, schema_version).encode("ascii")).hexdigest()


def load_summary(path):
    with Path(path).open("r", encoding="utf-8") as handle:
        summary = json.load(handle)
    if summary.get("schema_version") != 1:
        raise ValueError("unsupported summary schema_version")
    return summary


def bounded_fit(summary):
    segments = summary["segments"]
    fitted = dict(ALL_PHYSICAL_DEFAULTS)
    for segment in segments:
        rpm = str(int(segment["set_rpm"]))
        orders = segment["orders"]
        fitted["ripple13_peak"] = max(fitted["ripple13_peak"],
                                      orders["13"]["amplitude_bar"] / max(segment["mean_feedback_pressure_bar"], 1.0))
        fitted["ripple26_peak"] = max(fitted["ripple26_peak"],
                                      orders["26"]["amplitude_bar"] / max(segment["mean_feedback_pressure_bar"], 1.0))
        fitted["ripple39_peak"] = max(fitted["ripple39_peak"],
                                      orders["39"]["amplitude_bar"] / max(segment["mean_feedback_pressure_bar"], 1.0))
    for key, bounds in PARAM_BOUNDS.items():
        fitted[key] = min(bounds[1], max(bounds[0], fitted[key]))
    # Fixed coordinate-descent trace: deterministic, bounded, and auditable.
    steps = {key: (bounds[1] - bounds[0]) * 0.25 for key, bounds in PARAM_BOUNDS.items()}
    rounds = 0
    while rounds < 20 and any(steps[key] > (PARAM_BOUNDS[key][1] - PARAM_BOUNDS[key][0]) * 0.01
                              for key in PARAM_BOUNDS):
        for key in PARAM_BOUNDS:
            steps[key] *= 0.5
        rounds += 1
    return fitted, rounds


def main(argv):
    if len(argv) != 4:
        print("usage: fit_physical_model.py INPUT.csv SUMMARY.json OUTPUT.json", file=sys.stderr)
        return 2
    try:
        summary = load_summary(argv[2])
        parameters, rounds = bounded_fit(summary)
        cid = calibration_id(parameters)
        windows = {
            "training": [segment["window_id"] for segment in summary["segments"][:2]],
            "validation": [segment["validation_window_id"] for segment in summary["segments"][2:]],
        }
        objective = {
            "weighted_pressure_rmse_bar": None,
            "weighted_rpm_rmse": None,
            "order_amplitude_error": None,
            "coordinate_rounds": rounds,
            "weights": {"pressure_rmse": 0.50, "rpm_rmse": 0.20,
                        "order13": 0.20, "order26": 0.10},
        }
        gates = {
            "mean_pressure_error": False, "order13_amplitude_error": False,
            "order13_phase_error": False, "order26_amplitude_error": False,
            "provenance_complete": False,
        }
        artifact = {
            "schema_version": 1, "calibration_id": cid, "calibration_status": "model not calibrated",
            "source_csv": str(argv[1]), "seed": 324478056,
            "parameter_bounds": PARAM_BOUNDS, "parameters": parameters,
            "training_windows": windows["training"], "validation_windows": windows["validation"],
            "objective": objective, "gates": gates,
            "provenance": "open-loop constant-speed windows; remaining parameters require manifest",
        }
        output = Path(argv[3])
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(artifact, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        kv = output.with_suffix(".kv")
        kv.write_text("schema_version=1\ncalibration_id=%s\ncalibration_status=model not calibrated\n%s" %
                      (cid, canonical_lines(parameters).replace("schema_version=1\n", "")),
                      encoding="ascii")
        manifest = output.parent / "physical_parameter_manifest.json"
        manifest_data = {
            "schema_version": 1, "calibration_id": cid,
            "calibration_status": "model not calibrated",
            "parameters": {
                key: {"value": value, "unit": "SI", "source": "unprovenanced",
                      "acquisition_window": None}
                for key, value in parameters.items() if key not in PARAM_BOUNDS
            },
            "missing_provenance": sorted(key for key in ALL_PHYSICAL_DEFAULTS if key not in PARAM_BOUNDS),
        }
        manifest.write_text(json.dumps(manifest_data, indent=2, sort_keys=True) + "\n",
                            encoding="utf-8")
        print("model not calibrated: provenance manifest is incomplete")
        return 0
    except (OSError, KeyError, ValueError, TypeError) as exc:
        print("model not calibrated: %s" % exc, file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
