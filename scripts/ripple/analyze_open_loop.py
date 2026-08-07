#!/usr/bin/env python3
"""Deterministic angle-synchronous analysis for the open-loop recording."""
import csv
import json
import math
import statistics
import sys
from pathlib import Path


def read_rows(path):
    last_error = None
    for encoding in ("utf-8-sig", "gbk"):
        try:
            with Path(path).open("r", encoding=encoding, newline="") as handle:
                rows = list(csv.reader(handle))
            break
        except UnicodeDecodeError as exc:
            last_error = exc
    else:
        raise ValueError("unable to decode CSV as utf-8-sig or gbk") from last_error
    if not rows:
        raise ValueError("CSV is empty")
    data = []
    for line_no, row in enumerate(rows[1:], 2):
        if len(row) < 8:
            raise ValueError("row %d has fewer than 8 columns" % line_no)
        try:
            data.append([float(value.strip()) for value in row[:8]])
        except ValueError as exc:
            raise ValueError("row %d contains a non-numeric value" % line_no) from exc
    if len(data) < 2:
        raise ValueError("CSV needs at least two samples")
    for index in range(1, len(data)):
        delta = data[index][0] - data[index - 1][0]
        if abs(delta - 1.0) > 1.0e-9:
            raise ValueError("timestamp delta at row %d is %.9g ms, expected exactly 1 ms" %
                             (index + 2, delta))
    if any(abs(row[4]) > 1.0e-9 for row in data):
        raise ValueError("open-loop target pressure must be zero")
    return data


def solve_order(samples, order):
    matrix = [[0.0, 0.0, 0.0] for _ in range(3)]
    vector = [0.0, 0.0, 0.0]
    for row in samples:
        angle = math.radians(order * row[7])
        basis = (1.0, math.cos(angle), math.sin(angle))
        for i in range(3):
            vector[i] += basis[i] * row[1]
            for j in range(3):
                matrix[i][j] += basis[i] * basis[j]
    for pivot in range(3):
        selected = max(range(pivot, 3), key=lambda index: abs(matrix[index][pivot]))
        if abs(matrix[selected][pivot]) < 1.0e-12:
            raise ValueError("singular angle-synchronous normal equations")
        matrix[pivot], matrix[selected] = matrix[selected], matrix[pivot]
        vector[pivot], vector[selected] = vector[selected], vector[pivot]
        scale = matrix[pivot][pivot]
        for column in range(pivot, 3):
            matrix[pivot][column] /= scale
        vector[pivot] /= scale
        for row in range(3):
            if row == pivot:
                continue
            factor = matrix[row][pivot]
            for column in range(pivot, 3):
                matrix[row][column] -= factor * matrix[pivot][column]
            vector[row] -= factor * vector[pivot]
    dc, cosine, sine = vector
    return {
        "dc_bar": dc,
        "cosine_bar": cosine,
        "sine_bar": sine,
        "amplitude_bar": math.hypot(cosine, sine),
        "phase_rad": math.atan2(cosine, sine),
        "phase_deg": math.degrees(math.atan2(cosine, sine)),
    }


def analyze(path):
    rows = read_rows(path)
    segments = []
    start = 0
    while start < len(rows):
        rpm = rows[start][5]
        end = start + 1
        while end < len(rows) and rows[end][5] == rpm:
            end += 1
        if rpm > 0.0:
            segment = rows[start:end]
            stable = segment[2000:] if len(segment) > 2000 else []
            tail = stable[-5000:]
            if len(tail) < 5000:
                raise ValueError("set-rpm %.9g segment has fewer than 5000 tail samples" % rpm)
            metrics = {
                "window_id": "set_rpm_%g_full" % rpm,
                "validation_window_id": "set_rpm_%g_tail5000" % rpm,
                "set_rpm": rpm,
                "timestamp_start_ms": int(segment[0][0]),
                "timestamp_end_ms": int(segment[-1][0]),
                "sample_count": len(segment),
                "discarded_prefix_samples": min(2000, len(segment)),
                "tail_sample_count": len(tail),
                "mean_feedback_rpm": statistics.fmean(row[2] for row in segment),
                "mean_feedback_pressure_bar": statistics.fmean(row[1] for row in segment),
                "tail_pressure_peak_to_peak_bar": max(row[1] for row in tail) -
                min(row[1] for row in tail),
                "feedback_rpm_stddev": statistics.pstdev(row[2] for row in tail),
                "torque_mean_permille": statistics.fmean(row[6] for row in tail),
                "orders": {str(order): solve_order(tail, order) for order in (13, 26, 39)},
            }
            segments.append(metrics)
        start = end
    if not segments:
        raise ValueError("no positive set-rpm segments found")
    return {
        "schema_version": 1,
        "analysis": "angle_synchronous_open_loop",
        "source": str(path),
        "timestamp_period_ms": 1,
        "open_loop_target_pressure_bar": 0,
        "discard_prefix_s": 2,
        "tail_samples": 5000,
        "orders": [13, 26, 39],
        "segments": segments,
    }


def main(argv):
    if len(argv) != 3:
        print("usage: analyze_open_loop.py INPUT.csv OUTPUT.json", file=sys.stderr)
        return 2
    try:
        result = analyze(argv[1])
        output = Path(argv[2])
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        return 0
    except (OSError, ValueError) as exc:
        print("analysis failed: %s" % exc, file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
