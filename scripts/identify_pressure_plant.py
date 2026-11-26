#!/usr/bin/env python3
"""Identify the pressure plant from a VOFA CSV (spec 2026-11-23 §8.1).

Model, rpm domain (parameters 34 and 37):
    dP/dt = Kq * (n(t - lag) - P / ksys)
    Kq    bar/s per rpm   (parameter 37 = 0.9 x the lowest Kq over the working range)
    ksys  bar per rpm     (parameter 34)

Method: pump speed n comes from the unwrapped pump angle (rActPumpPosition),
pressure from ActPressure (after the PLC MA3). Both are smoothed over 11 ms,
then dP/dt = a*n(t - lag) - b*P is fitted by least squares over every sample
above 5 bar, for lag 0..8 ms; the best fit gives Kq = a and ksys = a / b.
Kq is also fitted per 20 bar pressure band (ksys fixed) to expose its rise
with pressure. The ripple orders are pressure amplitudes at orders 1..30 of
the pump angle over the longest flat-pressure stretch.

Pressure-limit PI (spec 2026-11-26 §10, parameters 40/41) uses the HIGHEST
band Kq: Kp = 2*pi*fc / (Kq_max * G), Ki = Kp * 2*pi*fc / 4, fc = 10 Hz.

Records without a pump angle (e.g. injection-axis records) pass the column
names; --speed-col reads the servo speed directly (times --speed-scale, the
VOFA records log 10 rpm per count), --pos-col with --max-pos-rate keeps only
samples where the screw is stationary. Ripple orders need the pump angle.

Standard library only. Usage:
    python3 scripts/identify_pressure_plant.py FILE.csv [FILE.csv ...]
    python3 scripts/identify_pressure_plant.py --pressure-col fbPressureSensor.rActualValue \
        --speed-col fbServoControl.ActSpeedRPM --speed-scale 10 \
        --pos-col G_Inject.udiInjElecRulerVal --max-pos-rate 0.5 FILE.csv
"""
import argparse
import csv
import math
import sys

DT = 0.001
SMOOTH = 11            # samples
MIN_PRESSURE = 5.0     # bar
BAND = 20.0            # bar, Kq pressure bands
MIN_BAND_SAMPLES = 200
MIN_DRIVE = 2.0        # rpm, n - P/ksys
HOLD_SLOPE = 2.0       # bar/s over 1 s (whole ripple periods at >= 5 rpm)
MIN_R2 = 0.8
MIN_HOLD = 2000        # samples
MAX_ORDER = 30
LIMIT_FC = 10.0        # Hz, pressure-limit PI crossover (spec 2026-11-26 §5.1)


def load(path, args):
    with open(path, encoding="utf-8-sig", newline="") as handle:
        rows = list(csv.DictReader(handle))
    pressure = [float(r[args.pressure_col]) for r in rows]
    angle = None
    speed = None
    if args.speed_col:
        speed = [float(r[args.speed_col]) * args.speed_scale for r in rows]
    else:
        angle = [float(r[args.angle_col]) for r in rows]
    position = [float(r[args.pos_col]) for r in rows] if args.pos_col else None
    return pressure, angle, speed, position


def valid_samples(pressure, position, max_rate):
    if position is None:
        return [p >= MIN_PRESSURE for p in pressure]
    rate = gradient(smooth(position))
    return [p >= MIN_PRESSURE and abs(r) <= max_rate for p, r in zip(pressure, rate)]


def unwrap_deg(angle):
    out = [angle[0]]
    offset = 0.0
    for previous, current in zip(angle, angle[1:]):
        step = current - previous
        if step > 180.0:
            offset -= 360.0
        elif step < -180.0:
            offset += 360.0
        out.append(current + offset)
    return out


def smooth(values, width=SMOOTH):
    half = width // 2
    prefix = [0.0]
    for v in values:
        prefix.append(prefix[-1] + v)
    out = []
    for i in range(len(values)):
        lo = max(0, i - half)
        hi = min(len(values), i + half + 1)
        out.append((prefix[hi] - prefix[lo]) / (hi - lo))
    return out


def gradient(values, span=1):
    out = [0.0] * len(values)
    for i in range(span, len(values) - span):
        out[i] = (values[i + span] - values[i - span]) / (2 * span * DT)
    return out


def fit_plant(pressure, speed, rate, valid):
    best = None
    for lag in range(0, 9):
        sxx = sxy = syy = sxd = syd = sdd = 0.0
        count = 0
        for i in range(lag + SMOOTH, len(pressure) - SMOOTH):
            if not valid[i]:
                continue
            x = speed[i - lag]
            y = -pressure[i]
            d = rate[i]
            sxx += x * x; sxy += x * y; syy += y * y
            sxd += x * d; syd += y * d; sdd += d * d
            count += 1
        det = sxx * syy - sxy * sxy
        if count < 500 or det <= 0.0:
            continue
        a = (sxd * syy - syd * sxy) / det
        b = (syd * sxx - sxd * sxy) / det
        residual = sdd - 2 * (a * sxd + b * syd) + a * a * sxx + 2 * a * b * sxy + b * b * syy
        r2 = 1.0 - residual / max(sdd, 1e-12)
        if best is None or r2 > best["r2"]:
            best = {"lag": lag, "kq": a, "b": b, "r2": r2, "count": count}
    return best


def kq_bands(pressure, speed, rate, lag, ksys, valid):
    bands = {}
    for i in range(lag + SMOOTH, len(pressure) - SMOOTH):
        if not valid[i]:
            continue
        drive = speed[i - lag] - pressure[i] / ksys
        if drive < MIN_DRIVE:
            continue
        key = int(pressure[i] // BAND)
        num, den, cnt = bands.get(key, (0.0, 0.0, 0))
        bands[key] = (num + rate[i] * drive, den + drive * drive, cnt + 1)
    return [(key * BAND, num / den, cnt) for key, (num, den, cnt) in sorted(bands.items())
            if cnt >= MIN_BAND_SAMPLES and den > 0.0]


def longest_hold(pressure):
    slope = gradient(pressure, span=500)
    best = (0, 0)
    start = None
    for i, s in enumerate(slope + [1e9]):
        flat = i < len(pressure) and abs(s) < HOLD_SLOPE and pressure[i] > 10.0
        if flat and start is None:
            start = i
        elif not flat and start is not None:
            if i - start > best[1] - best[0]:
                best = (start, i)
            start = None
    return best


def ripple_orders(pressure, angle, window):
    start, end = window
    if end - start < MIN_HOLD:
        return []
    mean = sum(pressure[start:end]) / (end - start)
    result = []
    for order in range(1, MAX_ORDER + 1):
        c = s = 0.0
        for i in range(start, end):
            theta = math.radians(angle[i]) * order
            c += (pressure[i] - mean) * math.cos(theta)
            s += (pressure[i] - mean) * math.sin(theta)
        result.append((2.0 * math.hypot(c, s) / (end - start), order))
    return sorted(result, reverse=True)


def main(args):
    for path in args.files:
        raw_pressure, raw_angle, raw_speed, position = load(path, args)
        pressure = smooth(raw_pressure)
        if raw_speed is not None:
            speed = smooth(raw_speed)
        else:
            speed = [v * 60.0 / 360.0 for v in gradient(smooth(unwrap_deg(raw_angle)))]
        rate = gradient(pressure)
        valid = valid_samples(pressure, position, args.max_pos_rate)
        print(f"== {path}")
        fit = fit_plant(pressure, speed, rate, valid)
        if fit is None or fit["kq"] <= 0.0 or fit["b"] <= 0.0:
            print("   not enough data above 5 bar")
            continue
        ksys = fit["kq"] / fit["b"]
        print(f"   Kq   = {fit['kq']:.3f} bar/s/rpm   ksys = {ksys:.3f} bar/rpm"
              f"   (lag {fit['lag']} ms, R2 {fit['r2']:.3f}, {fit['count']} samples)")
        if fit["r2"] < MIN_R2:
            print(f"   WARNING: R2 < {MIN_R2}: pressure rises too slowly for the ripple;"
                  " identify from a closed-loop step record instead")
        bands = kq_bands(pressure, speed, rate, fit["lag"], ksys, valid)
        if bands:
            print("   Kq by pressure band: " +
                  ", ".join(f"{p:.0f}-{p + BAND:.0f} bar {k:.2f}" for p, k, _ in bands))
            lowest = min(k for _, k, _ in bands)
            highest = max(k for _, k, _ in bands)
        else:
            lowest = highest = fit["kq"]
        print(f"   parameter 34 = {ksys:.2f}, parameter 37 = {0.9 * min(lowest, fit['kq']):.2f}"
              f" (0.9 x lowest Kq)")
        highest = max(highest, fit["kq"])
        if fit["r2"] < MIN_R2 or not bands or min(k for _, k, _ in bands) <= 0.0:
            print("   parameter 40/41: not identified (poor fit), keep the built-in defaults")
        else:
            omega = 2.0 * math.pi * LIMIT_FC
            kp = omega / (highest * args.gain)
            print(f"   parameter 40 = {kp:.2f} L/min/bar, parameter 41 = {kp * omega / 4.0:.1f} L/min/(bar*s)"
                  f" (pressure-limit PI, fc {LIMIT_FC:.0f} Hz, highest Kq {highest:.2f}, G {args.gain:g})")
        if raw_angle is None:
            continue
        window = ripple_orders(raw_pressure, raw_angle, longest_hold(raw_pressure))
        if window:
            print("   hold ripple: " + ", ".join(f"order {k} {a:.2f} bar" for a, k in window[:4]))


def parse_args(argv):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("files", nargs="+")
    parser.add_argument("--pressure-col", default="ActPressure")
    parser.add_argument("--angle-col", default="rActPumpPosition")
    parser.add_argument("--speed-col", default=None,
                        help="servo speed column; replaces the pump-angle derivative")
    parser.add_argument("--speed-scale", type=float, default=1.0,
                        help="rpm per logged count of --speed-col (VOFA records: 10)")
    parser.add_argument("--pos-col", default=None,
                        help="actuator position column [mm] for the stationary filter")
    parser.add_argument("--max-pos-rate", type=float, default=0.5,
                        help="mm/s; samples moving faster are skipped (needs --pos-col)")
    parser.add_argument("--gain", type=float, default=20.0,
                        help="flowToPumpSpeedGain G [rpm per L/min] for parameters 40/41")
    return parser.parse_args(argv)


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(2)
    main(parse_args(sys.argv[1:]))
