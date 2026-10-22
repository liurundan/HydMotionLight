"""Scan hydraulic test CSVs: segment detection + steady-state statistics.

Pure standard library (no numpy). Assumes fixed 1 ms sampling.
"""
import csv
import math
import os
import sys

DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'docs', 'data_deg')
DT = 0.001

FILES = [
    'open-8-10-12_rpm.csv',
    'open-15-20_rpmp.csv',
    'close-50-80-100-120-140_bar_deg.csv',
]


def load(path):
    with open(path, 'r', newline='', encoding='utf-8-sig', errors='replace') as fh:
        rd = csv.reader(fh)
        header = next(rd)
        cols = {name: [] for name in header}
        for row in rd:
            if not row or len(row) < len(header):
                continue
            for name, val in zip(header, row):
                try:
                    cols[name].append(float(val))
                except ValueError:
                    cols[name].append(float('nan'))
    return header, cols


def stats(vals):
    v = [x for x in vals if x == x]
    n = len(v)
    if n == 0:
        return None
    mean = sum(v) / n
    var = sum((x - mean) ** 2 for x in v) / max(1, n - 1)
    sd = math.sqrt(var)
    s = sorted(v)
    def q(p):
        idx = p * (n - 1)
        lo = int(math.floor(idx))
        hi = min(lo + 1, n - 1)
        return s[lo] + (s[hi] - s[lo]) * (idx - lo)
    return {
        'n': n, 'mean': mean, 'sd': sd, 'min': s[0], 'max': s[-1],
        'p2p': s[-1] - s[0], 'p1': q(0.01), 'p99': q(0.99),
        'p2p_98': q(0.99) - q(0.01), 'med': q(0.5),
    }


def quantization(vals):
    s = sorted(set(vals))
    if len(s) < 3:
        return len(s), None
    diffs = [round(s[i + 1] - s[i], 6) for i in range(len(s) - 1)]
    pos = [d for d in diffs if d > 1e-9]
    return len(s), (min(pos) if pos else None)


def segments(cmd, tol=1e-6, min_len=200):
    """Split by command-value changes; merge tiny blips."""
    segs = []
    start = 0
    cur = cmd[0]
    for i in range(1, len(cmd)):
        if abs(cmd[i] - cur) > tol:
            if i - start >= min_len:
                segs.append((start, i, cur))
            start = i
            cur = cmd[i]
    if len(cmd) - start >= min_len:
        segs.append((start, len(cmd), cur))
    return segs


def fmt(st, unit=''):
    if st is None:
        return 'n/a'
    return (f"mean={st['mean']:.4f} sd={st['sd']:.4f} p2p={st['p2p']:.4f} "
            f"p2p98={st['p2p_98']:.4f} min={st['min']:.4f} max={st['max']:.4f}{unit}")


def main():
    for fname in FILES:
        path = os.path.join(DATA_DIR, fname)
        header, cols = load(path)
        n = len(cols[header[0]])
        print('=' * 100)
        print(f'FILE {fname}  rows={n}  duration={n * DT:.2f}s')
        print('header:', ','.join(header))
        # choose command column
        cmd_col = 'rTargetPressure' if 'close' in fname else 'rTargetSpeedRPM'
        cmd = cols[cmd_col]
        segs = segments(cmd)
        print(f'command column = {cmd_col}; segments({len(segs)}):')
        for (a, b, c) in segs:
            print(f'  idx[{a}:{b}] t={a * DT:.2f}-{b * DT:.2f}s len={b - a} cmd={c:.4f}')
        # steady-state detail per segment (skip first 30% for settling)
        for (a, b, c) in segs:
            ln = b - a
            ss = a + int(ln * 0.4)
            if b - ss < 500:
                ss = max(a + 200, b - 500)
            sl = slice(ss, b)
            print('-' * 100)
            print(f'SEG cmd={c:.4f} idx[{a}:{b}] steady idx[{ss}:{b}] ({(b - ss) * DT:.2f}s)')
            for col in header:
                st = stats(cols[col][sl])
                if st is None:
                    continue
                print(f'   {col:28s} {fmt(st)}')
            raw = cols['rActPressureRaw'][sl]
            nq, step = quantization(raw)
            print(f'   raw pressure: unique={nq} min_step={step}')
            filt = cols['rActPressureFiltered'][sl]
            nq2, step2 = quantization(filt)
            print(f'   filtered pressure: unique={nq2} min_step={step2}')
            # pump shaft speed from position
            pos = cols['rActPumpPosition'][sl]
            if pos:
                d = [pos[i + 1] - pos[i] for i in range(len(pos) - 1)]
                # unwrap +-180 deg jumps
                du = []
                for x in d:
                    while x > 180:
                        x -= 360
                    while x < -180:
                        x += 360
                    du.append(x)
                st = stats(du)
                if st:
                    rpm = st['mean'] * 60.0 / 360.0
                    print(f'   dPos/dt: {fmt(st)} deg/step -> mean {rpm:.3f} rpm')
        sys.stdout.flush()


if __name__ == '__main__':
    main()
