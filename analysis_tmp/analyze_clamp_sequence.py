"""Extract mold-close (合模) segment sequence from the VOFA waveform.

Sampling: 1 ms / row (row 1 = header, data row k => t = (k-2) ms).
Field units (from design spec):
  uiSpdCmd    : speed setpoint in 0.1% of 1800 mm/s  => x1.8 mm/s
  uiEndSpdCmd : END_SPEED in the same 0.1% unit      => x1.8 mm/s
  udiPosCmd   : target position in mm
  PUMPSPEED   : pump speed in 0.1 rpm = |v_mm_s| * 0.96
"""
import csv
import os

HERE = os.path.dirname(os.path.abspath(__file__))
CSV = os.path.join(HERE, '..', 'docs', 'data_deg',
                   'vofa_waveform_20261110_082428_443.csv')

KEY = ('udiPosCmd', 'uiSpdCmd', 'uiEndSpdCmd',
       'uiAxisSpdAcc', 'uiAxisSpdDec')


def main():
    with open(CSV, 'r', newline='', encoding='utf-8-sig') as fh:
        rows = list(csv.DictReader(fh))
    n = len(rows)
    print('rows =', n, ' (t = 0 .. %d ms)' % (n - 1))

    def f(row, name):
        try:
            return float(row[name])
        except (ValueError, TypeError):
            return float('nan')

    # Group consecutive rows with identical command tuple.
    blocks = []
    start = 0
    cur = tuple(rows[0][k] for k in KEY)
    for i in range(1, n):
        tup = tuple(rows[i][k] for k in KEY)
        if tup != cur:
            blocks.append((start, i - 1, cur))
            start, cur = i, tup
    blocks.append((start, n - 1, cur))

    print('\n%-9s %-9s %-7s %-8s %-8s %-8s %-8s %-8s %-9s %-9s %-9s' % (
        't0(ms)', 't1(ms)', 'dur', 'posCmd', 'spdCmd', 'endSpd',
        'acc', 'dec', 'pos0', 'pos1', 'pump_ss'))
    print('-' * 110)
    for a, b, tup in blocks:
        if b - a < 2:      # skip 1-2 scan command glitches
            continue
        pos0 = f(rows[a], 'G_Clamp.ClampHydAxisRef.rActPosition')
        pos1 = f(rows[b], 'G_Clamp.ClampHydAxisRef.rActPosition')
        # steady-state pump speed = median of the last third of the block
        tail = [f(rows[k], 'fbGetPumpRequest.PUMPSPEED')
                for k in range(a + 2 * (b - a) // 3, b + 1)]
        tail = sorted(x for x in tail if x == x)
        pump = tail[len(tail) // 2] if tail else float('nan')
        print('%-9d %-9d %-7d %-8s %-8s %-8s %-8s %-8s %-9.3f %-9.3f %-9.3f' % (
            a, b, b - a + 1, *tup, pos0, pos1, pump))

    # Locate the mold-close phase: position descending from ~130 to ~0.
    pos = [f(r, 'G_Clamp.ClampHydAxisRef.rActPosition') for r in rows]
    peaks = max(range(n), key=lambda i: pos[i] if pos[i] == pos[i] else -1e9)
    print('\nmax position = %.3f mm at t=%d ms' % (pos[peaks], peaks))
    # first index after the peak where position <= 1 mm
    end = next((i for i in range(peaks, n) if pos[i] <= 1.0), n - 1)
    print('mold-close phase: t = %d .. %d ms, pos %.3f -> %.3f mm'
          % (peaks, end, pos[peaks], pos[end]))

    # Velocity from position difference (mm/s), smoothed with 5-point median.
    print('\nVelocity-slope detail around command changes inside mold-close:')
    for a, b, tup in blocks:
        if a < peaks or a > end or b - a < 2:
            continue
        print('\n  block t=%d..%d  posCmd=%s spdCmd=%s endSpd=%s acc=%s dec=%s'
              % (a, b, *tup))
        # measured axis speed = -(dpos/dt) for closing (position decreasing)
        for k in range(a, min(a + 130, b + 1), 10):
            v = -(pos[min(k + 1, n - 1)] - pos[k]) * 1000.0
            pump = f(rows[k], 'fbGetPumpRequest.PUMPSPEED')
            print('    t=%5d pos=%8.3f  v=%9.3f mm/s  pump=%9.3f'
                  % (k, pos[k], v, pump))


if __name__ == '__main__':
    main()
