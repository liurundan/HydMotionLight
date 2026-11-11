"""Measure transition slopes of the mold-close sequence in the VOFA waveform.

Pump speed column = |v_mm_s| * 0.96, so v = pump / 0.96.
Command tuple changes define segment boundaries.
"""
import csv
import os

HERE = os.path.dirname(os.path.abspath(__file__))
CSV = os.path.join(HERE, '..', 'docs', 'data_deg',
                   'vofa_waveform_20261110_082428_443.csv')
KEY = ('udiPosCmd', 'uiSpdCmd', 'uiEndSpdCmd')
PUMP = 'fbGetPumpRequest.PUMPSPEED'
POS = 'G_Clamp.ClampHydAxisRef.rActPosition'


def main():
    with open(CSV, 'r', newline='', encoding='utf-8-sig') as fh:
        rows = list(csv.DictReader(fh))

    def num(r, k):
        try:
            return float(r[k])
        except (ValueError, TypeError):
            return float('nan')

    vel = [num(r, PUMP) / 0.96 for r in rows]
    pos = [num(r, POS) for r in rows]
    cmd = [tuple(r[k] for k in KEY) for r in rows]

    bounds = [i for i in range(1, len(rows)) if cmd[i] != cmd[i - 1]]
    print('command changes at t(ms):', bounds[:20])

    # Locate the mold-close phase automatically: start at the position peak
    # (130 mm) and end when the axis first reaches <= 1 mm.  The last command
    # change in that window (5 -> 0 mm) starts the low-pressure hold phase,
    # which is pressure-controlled and therefore excluded from the ramp scan.
    peak = max(range(len(rows)), key=lambda i: pos[i] if pos[i] == pos[i] else -1e9)
    hold = next((i for i in range(peak, len(rows)) if pos[i] <= 1.0), len(rows) - 1)
    close_bounds = [i for i in bounds if peak <= i < hold]
    print('mold-close phase: t=%d..%d ms, command changes %s'
          % (peak, hold, close_bounds))

    for idx, b in enumerate(close_bounds[:-1]):
        end = close_bounds[idx + 1]
        print('\n=== segment %d: t=%d..%d  cmd=%s (pos %.3f -> %.3f) ===' %
              (idx + 1, b, end - 1, cmd[b], pos[b], pos[end - 1]))
        # list contiguous runs where |dv/dt| > 0.02 mm/s per ms
        runs = []
        k = b
        while k < end:
            dv = vel[k + 1] - vel[k] if k + 1 < len(vel) else 0.0
            if abs(dv) > 0.02:
                s = k
                sign = 1.0 if dv > 0 else -1.0
                steps = []
                while k < end - 1 and (vel[k + 1] - vel[k]) * sign > 0.02:
                    steps.append((vel[k + 1] - vel[k]) * sign)
                    k += 1
                runs.append((s, k, sign, steps))
            k += 1
        for s, e, sign, steps in runs:
            if len(steps) < 3:
                continue
            v0, v1 = vel[s], vel[e]
            mean_step = sum(steps) / len(steps)
            print('  ramp t=%5d..%5d  v %8.3f -> %8.3f mm/s  dur=%4d ms  '
                  'slope=%+8.1f mm/s^2 (mean step %.4f mm/s/ms, max %.4f)'
                  % (s, e, v0, v1, e - s, sign * mean_step * 1000.0,
                     mean_step, max(steps)))
        # steady-state speed
        tail = vel[min(b + 150, end - 30):end - 20]
        if tail:
            print('  steady v ~ %.3f mm/s (pos %.3f)' %
                  (sum(tail) / len(tail), pos[end - 20]))

    print('\n=== low-pressure / hold phase after t=7281 ===')
    for t in (7281, 7282, 7285, 7290, 7300, 7350, 7400, 8000):
        print('  t=%5d pos=%8.4f  v=%8.3f mm/s' % (t, pos[t], vel[t]))


if __name__ == '__main__':
    main()
