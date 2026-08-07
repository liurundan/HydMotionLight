#!/usr/bin/env python3
"""Deterministic, bounded host-only physical identification."""
import hashlib
import json
import math
import sys
from pathlib import Path

from analyze_open_loop import read_rows, solve_order

def canonical_json(value, excluded=()):
    if isinstance(value, dict):
        value = {key: item for key, item in value.items() if key not in excluded}
    return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False)

PARAM_BOUNDS = {
    "motor_natural_freq_hz": [1.0, 50.0], "motor_damping": [0.2, 2.0],
    "motor_delay_s": [0.0, 0.05], "outlet_volume_m3": [1e-5, 5e-3],
    "gas_fraction": [1e-5, 2e-2], "pump_leak_c0_m3_pa_s": [1e-14, 1e-9],
    "pump_leak_speed_m3_pa_s_per_rpm": [1e-16, 1e-11],
    "cylinder_leak_m3_pa_s": [1e-14, 1e-9], "outlet_leak_m3_pa_s": [1e-14, 1e-9],
    "ripple13_peak": [0.0, 0.30], "ripple26_peak": [0.0, 0.15],
    "ripple39_peak": [0.0, 0.10], "torque_ripple13_peak": [0.0, 0.30],
    "ripple13_phase_rad": [-math.pi, math.pi], "ripple26_phase_rad": [-math.pi, math.pi],
    "ripple39_phase_rad": [-math.pi, math.pi], "torque_ripple13_phase_rad": [-math.pi, math.pi],
}
DEFAULTS = {
    "atmospheric_pressure_pa": 101325.0, "suction_pressure_pa": 101325.0,
    "outlet_volume_m3": 5e-4, "chamber_volume_m3": 5e-4,
    "line_inertance_pa_s2_per_m3": 1e8, "line_resistance_pa_s_per_m3": 2e12,
    "line_quadratic_resistance_pa_s2_per_m6": 0.0, "beta_oil_pa": 1.2e9,
    "gas_fraction": .002, "gas_transition_pa": 1e6, "beta_min_pa": 5e7,
    "pump_leak_c0_m3_pa_s": 2e-13, "pump_leak_speed_m3_pa_s_per_rpm": 1e-15,
    "outlet_leak_m3_pa_s": 1e-13, "cylinder_leak_m3_pa_s": 1e-13,
    "eta_v_min": .6, "eta_m_nominal": .9, "eta_m_pressure_loss_per_pa": 1e-9,
    "eta_m_speed_loss_per_rpm": 1e-5, "eta_m_min": .5, "rated_motor_torque_nm": 150.0,
    "torque_ripple13_peak": .02, "torque_ripple13_phase_rad": 0.0,
    "ripple13_peak": .08, "ripple26_peak": .03, "ripple39_peak": .01,
    "ripple13_phase_rad": 0.0, "ripple26_phase_rad": 0.0, "ripple39_phase_rad": 0.0,
    "motor_natural_freq_hz": 12.0, "motor_damping": 1.0, "motor_delay_s": 0.0,
    "motor_accel_limit_rpm_s": 20000.0, "motor_torque_limit_permille": 1000.0,
    "relief_set_pa": 25e6, "relief_deadband_pa": .2e6,
    "relief_orifice_coeff_m3_s_sqrt_pa": 1e-8, "relief_hysteresis_pa": .5e6,
    "sensor_delay_s": 0.0, "sensor_quantization_bar": 0.0,
}

def canonical_lines(parameters, status, source_hash, manifest_hash):
    return ("schema_version=1\ncalibration_status=%s\nsource_sha256=%s\n"
            "manifest_provenance_sha256=%s\n" % (status, source_hash, manifest_hash)) + \
        "".join("%s=%.17g\n" % (key, parameters[key])
                                           for key in sorted(parameters))

def clamp(value, bounds): return max(bounds[0], min(bounds[1], value))

def phase_error(a, b):
    return abs((a - b + math.pi) % (2.0 * math.pi) - math.pi)

def predict(window, params):
    """A fixed-1ms second-order motor and fluid-chain surrogate for fit scoring."""
    rpm = accel = phase = p_out = p_ch = q_line = 0.0
    rpms, pressures, torque = [], [], []
    delay = [0.0] * 51
    delay_steps = int(round(params["motor_delay_s"] * 1000.0))
    for sample_index, row in enumerate(window):
        target = row[5]
        delay[sample_index % len(delay)] = target
        delayed = delay[(sample_index - delay_steps) % len(delay)]
        wn = 2.0 * math.pi * params["motor_natural_freq_hz"]
        accel += .001 * (wn * wn * (delayed - rpm) - 2.0 * params["motor_damping"] * wn * accel)
        accel = max(-20000.0, min(20000.0, accel))
        rpm += .001 * accel
        phase = (phase + rpm * .001 / 60.0) % 1.0
        wave13 = math.sin(2.0 * math.pi * 13.0 * phase + params["ripple13_phase_rad"])
        wave26 = math.sin(2.0 * math.pi * 26.0 * phase + params["ripple26_phase_rad"])
        wave39 = math.sin(2.0 * math.pi * 39.0 * phase + params["ripple39_phase_rad"])
        q_pump = 20e-6 * max(rpm, 0.0) / 60.0 * (1.0 + params["ripple13_peak"] * wave13 +
                                                  params["ripple26_peak"] * wave26 +
                                                  params["ripple39_peak"] * wave39)
        leak = (params["pump_leak_c0_m3_pa_s"] +
                params["pump_leak_speed_m3_pa_s_per_rpm"] * abs(rpm)) * max(p_out, 0.0)
        q_line = max(-.002, min(.002, q_line + .001 * (p_out - p_ch - 2e12 * q_line) / 1e8))
        p_out = max(0.0, min(3.0e7, p_out + .001 * 1.2e9 / params["outlet_volume_m3"] *
                              (q_pump - leak - params["outlet_leak_m3_pa_s"] * p_out - q_line)))
        p_ch = max(0.0, min(3.0e7, p_ch + .001 * 1.2e9 / params["chamber_volume_m3"] *
                             (q_line - params["cylinder_leak_m3_pa_s"] * p_ch)))
        rpms.append(rpm); pressures.append(p_ch / 1e5)
        torque.append(1000.0 * params["torque_ripple13_peak"] *
                      math.sin(2.0 * math.pi * 13.0 * phase + params["torque_ripple13_phase_rad"]))
    synth = [[0.0, pressures[i], rpms[i], 0.0, 0.0, 0.0, torque[i],
              (phase * 360.0)] for i in range(len(window))]
    # Use measured angle to preserve angle-synchronous comparison contract.
    for i, row in enumerate(window): synth[i][7] = row[7]
    return {"rpm": sum(rpms) / len(rpms), "pressure": sum(pressures) / len(pressures),
            "orders": {str(m): solve_order(synth, m) for m in (13, 26, 39)},
            "torque13": solve_order(synth, 13, 6)}

def objective(windows, params):
    pressure_sq = rpm_sq = order13_sq = order26_sq = phase13_sq = 0.0
    count = 0
    for window in windows:
        actual = window["actual"]; predicted = predict(window["rows"], params)
        pressure_sq += (predicted["pressure"] - actual["mean_feedback_pressure_bar"]) ** 2
        rpm_sq += (predicted["rpm"] - actual["mean_feedback_rpm"]) ** 2
        order13_sq += (predicted["orders"]["13"]["amplitude_bar"] -
                       actual["orders"]["13"]["amplitude_bar"]) ** 2
        order26_sq += (predicted["orders"]["26"]["amplitude_bar"] -
                       actual["orders"]["26"]["amplitude_bar"]) ** 2
        phase13_sq += math.degrees(phase_error(predicted["orders"]["13"]["phase_rad"],
                                                actual["orders"]["13"]["phase_rad"])) ** 2
        count += 1
    pressure = math.sqrt(pressure_sq / count); rpm = math.sqrt(rpm_sq / count)
    order13 = math.sqrt(order13_sq / count); order26 = math.sqrt(order26_sq / count)
    phase13 = math.sqrt(phase13_sq / count)
    return {"pressure_rmse_bar": pressure, "rpm_rmse": rpm, "order13_rmse_bar": order13,
            "order26_rmse_bar": order26,
            "order13_phase_rmse_deg": phase13,
            "value": .50 * pressure + .20 * rpm + .20 * order13 + .10 * order26 + .01 * phase13}

def window_sets(rows, summary):
    result = []
    for segment in summary["segments"]:
        matching = [row for row in rows if segment["timestamp_start_ms"] <= row[0] <= segment["timestamp_end_ms"]]
        tail = matching[2000:][-5000:]
        result.append(({"rows": tail[:2500], "actual": segment["training"],
                        "id": segment["training_window_id"]},
                       {"rows": tail[2500:], "actual": segment["heldout"],
                        "id": segment["validation_window_id"]}))
    return result

def main(argv):
    if len(argv) != 4:
        print("usage: fit_physical_model.py INPUT.csv SUMMARY.json OUTPUT.json", file=sys.stderr); return 2
    try:
        source = Path(argv[1]); summary = json.loads(Path(argv[2]).read_text(encoding="utf-8"))
        if summary.get("schema_version") != 1: raise ValueError("unsupported summary schema")
        rows = read_rows(source); pairs = window_sets(rows, summary)
        training = [pair[0] for pair in pairs]; heldout = [pair[1] for pair in pairs]
        params = dict(DEFAULTS)
        for pair in pairs:
            actual = pair[0]["actual"]
            for order, key in ((13, "ripple13_peak"), (26, "ripple26_peak"), (39, "ripple39_peak")):
                params[key] = clamp(actual["orders"][str(order)]["amplitude_bar"] /
                                    max(actual["mean_feedback_pressure_bar"], 1.0), PARAM_BOUNDS[key])
                params["ripple%d_phase_rad" % order] = actual["orders"][str(order)]["phase_rad"]
            params["torque_ripple13_peak"] = clamp(actual["torque_order13"]["amplitude_bar"] / 1000.0,
                                                    PARAM_BOUNDS["torque_ripple13_peak"])
            params["torque_ripple13_phase_rad"] = actual["torque_order13"]["phase_rad"]
        steps = {key: (bounds[1] - bounds[0]) * .25 for key, bounds in PARAM_BOUNDS.items()}
        trace = []
        for round_index in range(20):
            baseline = objective(training, params)
            for key, bounds in PARAM_BOUNDS.items():
                candidates = []
                for sign in (-1.0, 1.0):
                    candidate = dict(params); candidate[key] = clamp(params[key] + sign * steps[key], bounds)
                    candidates.append((objective(training, candidate), candidate))
                best_score, best = min(candidates, key=lambda item: item[0]["value"])
                if best_score["value"] < baseline["value"]:
                    params, baseline = best, best_score
                else:
                    steps[key] *= .5
            trace.append({"round": round_index + 1, "objective": baseline,
                          "max_step_fraction": max(steps[k] / (PARAM_BOUNDS[k][1] - PARAM_BOUNDS[k][0])
                                                   for k in steps)})
        train_metrics = objective(training, params); heldout_metrics = objective(heldout, params)
        source_hash = hashlib.sha256(source.read_bytes()).hexdigest()
        manifest_parameters = {key: {"value": value, "unit": "SI", "source": "unprovenanced",
                               "acquisition_window": None} for key, value in params.items()
                               if key not in PARAM_BOUNDS}
        status = "model not calibrated"
        manifest_payload = {"schema_version": 1, "calibration_status": status,
                            "source_sha256": source_hash, "parameters": manifest_parameters,
                            "missing_provenance": sorted(k for k in DEFAULTS if k not in PARAM_BOUNDS)}
        manifest_hash = hashlib.sha256(canonical_json(manifest_payload).encode("utf-8")).hexdigest()
        cid = hashlib.sha256(canonical_lines(params, status, source_hash, manifest_hash).encode("ascii")).hexdigest()
        gates = {"mean_pressure_error": False, "order13_amplitude_error": False,
                 "order13_phase_error": False, "order26_amplitude_error": False,
                 "provenance_complete": False, "heldout_report_present": False}
        artifact = {"schema_version": 1, "calibration_id": cid, "calibration_status": status,
                    "source_csv": str(source), "source_sha256": source_hash, "seed": 324478056,
                    "parameter_bounds": PARAM_BOUNDS, "parameters": params,
                    "training_windows": [item["id"] for item in training],
                    "validation_windows": [item["id"] for item in heldout],
                    "training_objective": train_metrics, "heldout_objective": heldout_metrics,
                    "coordinate_descent_trace": trace, "gates": gates}
        output = Path(argv[3]); output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(artifact, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        output.with_suffix(".kv").write_text("schema_version=1\ncalibration_id=%s\n%s" %
                                             (cid, canonical_lines(params, status, source_hash, manifest_hash)
                                              .split("\n", 1)[1]), encoding="ascii")
        manifest = dict(manifest_payload)
        manifest.update({"calibration_id": cid, "manifest_provenance_sha256": manifest_hash})
        summary["calibration_id"] = cid
        summary["calibration_status"] = status
        summary["source_sha256"] = source_hash
        summary["summary_sha256"] = hashlib.sha256(
            canonical_json(summary, ("summary_sha256",)).encode("utf-8")).hexdigest()
        artifact["summary_sha256"] = summary["summary_sha256"]
        artifact["manifest_provenance_sha256"] = manifest_hash
        output.write_text(json.dumps(artifact, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        Path(argv[2]).write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        (output.parent / "physical_parameter_manifest.json").write_text(
            json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        print("model not calibrated: manifest provenance and held-out validation report are required")
        return 0
    except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError) as exc:
        print("model not calibrated: %s" % exc, file=sys.stderr); return 1

if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
