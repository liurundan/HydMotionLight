"""Discriminate the origin of the 12/24/48/72 order family and verify the
speed-feedback resolution problem.

If the 24-family appears strongly in TORQUE and n_cmd but the 13-family is
pump-side, the two families have different physical sources.
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from analyze5 import (mean, rms, unwrap, load, segs_of, resample, full_spectrum)

ORDS = [1, 2, 3, 6, 7, 12, 13, 14, 24, 25, 26, 38, 39, 40, 48, 52, 59, 65, 72,
        78, 91, 98, 104, 117, 130, 156]


def main():
    cc = load('close-50-80-100-120-140_bar_deg.csv')
    cmd = cc['rTargetPressure']
    for (a, b, c) in segs_of(cmd):
        if c not in (100, 120, 140):
            continue
        ss = a + int((b - a) * 0.45)
        pos = cc['rActPumpPosition'][ss:b]
        ang = unwrap(pos)
        n = len(pos)
        d = [ang[i + 1] - ang[i] for i in range(n - 1)]
        shaft = mean(d) * 60000.0 / 360.0
        print('=' * 108)
        print(f'P_set={c:g} bar  shaft={shaft:.3f} rpm  f_rev={shaft / 60:.4f} Hz')
        # shaft speed fluctuation itself (mechanical torsional / cogging signature)
        print(f'  shaft instantaneous speed: mean={mean(d) * 60000 / 360:.4f} rpm '
              f'sd={rms(d) * 60000 / 360:.4f} rpm  '
              f'p2p={(max(d) - min(d)) * 60000 / 360:.3f} rpm  '
              f'({100 * rms(d) / mean(d):.1f}% of mean)')
        w, nr, _ = resample(ang, d, 4)
        if w:
            spec, m = full_spectrum(w, nr)
            dd = {}
            for o, amp, ph in spec:
                r = round(o)
                if abs(o - r) < 1e-6 and r in ORDS:
                    dd[r] = max(dd.get(r, 0.0), amp * 60000.0 / 360.0)
            print(f'  shaft-speed order content (rpm): '
                  f'{ {k: round(v, 3) for k, v in sorted(dd.items()) if v > 0.005} }')
        res = {}
        for name in ('rActPressureRaw', 'rActTorquePercent', 'rTargetSpeedRPM',
                     'rActRawSpeedRPM'):
            w2, nr2, _ = resample(ang, cc[name][ss:b], 4)
            if w2 is None:
                continue
            spec, m = full_spectrum(w2, nr2)
            dd = {}
            for o, amp, ph in spec:
                r = round(o)
                if abs(o - r) < 1e-6 and r in ORDS:
                    dd[r] = max(dd.get(r, 0.0), amp)
            res[name] = dd
        print(f'  {"ord":>5s} {"f(Hz)":>8s} {"P(bar)":>9s} {"T(%)":>9s} '
              f'{"n_cmd":>9s} {"T/P":>8s} {"n_cmd/P":>9s}')
        for o in ORDS:
            p = res.get('rActPressureRaw', {}).get(o, 0.0)
            t = res.get('rActTorquePercent', {}).get(o, 0.0)
            nn = res.get('rTargetSpeedRPM', {}).get(o, 0.0)
            if p < 0.02 and t < 0.05 and nn < 0.05:
                continue
            print(f'  {o:5d} {o * shaft / 60:8.3f} {p:9.4f} {t:9.4f} {nn:9.4f} '
                  f'{(t / p if p > 1e-6 else 0):8.2f} {(nn / p if p > 1e-6 else 0):9.2f}')

    # ---- speed feedback resolution audit ----
    print()
    print('=' * 108)
    print('SPEED FEEDBACK RESOLUTION AUDIT')
    for fn in ('close-50-80-100-120-140_bar_deg.csv', 'open-15-20_rpmp.csv'):
        cols = load(fn)
        nf = cols['rActRawSpeedRPM']
        uq = sorted(set(nf))
        gaps = [uq[i + 1] - uq[i] for i in range(len(uq) - 1) if uq[i + 1] - uq[i] > 1e-9]
        lsb = min(gaps) if gaps else float('nan')
        print(f'  {fn}: n_fb unique={len(uq)} values={[round(u, 4) for u in uq]}')
        print(f'     LSB={lsb:.4f} rpm  (=1 encoder count per 1 ms)')
        up = sorted(set(cols['rActPumpPosition']))
        gaps2 = [up[i + 1] - up[i] for i in range(len(up) - 1) if up[i + 1] - up[i] > 1e-9]
        plsb = min(gaps2) if gaps2 else float('nan')
        print(f'     position LSB={plsb:.6f} deg -> {360 / plsb:.0f} counts/rev '
              f'({math.log(360 / plsb, 2):.1f} bit)')
        print(f'     at 22.7 rpm the true motion is {22.7 / 60 * 360 / plsb:.3f} counts/ms '
              f'-> speed feedback quantisation error = +-{lsb / 2:.2f} rpm '
              f'({100 * lsb / 2 / 22.7:.0f}% of the actual speed)')

    # ---- torque vs pressure: is torque the disturbance source? ----
    print()
    print('=' * 108)
    print('TORQUE-TO-PRESSURE RATIO ACROSS OPERATING POINTS (load stiffness check)')
    print('  If |T|/P is constant, torque tracks pressure (load-driven). '
          'Harmonic |T|/P >> mean |T|/P means pump displacement ripple, not load.')
    for (a, b, c) in segs_of(cc['rTargetPressure']):
        if c == 0 or b - a < 5000:
            continue
        ss = a + int((b - a) * 0.45)
        P = cc['rActPressureRaw'][ss:b]
        T = cc['rActTorquePercent'][ss:b]
        print(f'  P_set={c:6.1f}: mean|T|/mean P = {abs(mean(T)) / mean(P):.4f} %/bar, '
              f'sd(T)/sd(P) = {rms(T) / rms(P):.4f} %/bar  '
              f'-> ripple amplification {(rms(T) / rms(P)) / (abs(mean(T)) / mean(P)):.2f}x')


if __name__ == '__main__':
    main()
