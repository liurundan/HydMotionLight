"""Exact time-domain reconstruction of order components -> true p2p under
each candidate mitigation, plus non-tooth-order (electrical/mechanical) scan.
"""
import csv
import math
import os
import sys

DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'docs', 'data_deg')
FS = 1000.0
PTS = 512
OUT = []


def log(m=''):
    print(m)
    OUT.append(str(m))
    sys.stdout.flush()


def fft(re, im):
    n = len(re)
    j = 0
    for i in range(1, n):
        bit = n >> 1
        while j & bit:
            j ^= bit
            bit >>= 1
        j |= bit
        if i < j:
            re[i], re[j] = re[j], re[i]
            im[i], im[j] = im[j], im[i]
    ln = 2
    while ln <= n:
        ang = -2.0 * math.pi / ln
        wr, wi = math.cos(ang), math.sin(ang)
        half = ln >> 1
        for s in range(0, n, ln):
            cwr, cwi = 1.0, 0.0
            for k in range(half):
                a, b = s + k, s + k + half
                tr = re[b] * cwr - im[b] * cwi
                ti = re[b] * cwi + im[b] * cwr
                re[b] = re[a] - tr
                im[b] = im[a] - ti
                re[a] += tr
                im[a] += ti
                cwr, cwi = cwr * wr - cwi * wi, cwr * wi + cwi * wr
        ln <<= 1
    return re, im


def mean(v):
    return sum(v) / len(v) if v else 0.0


def rms(v):
    m = mean(v)
    return math.sqrt(sum((x - m) ** 2 for x in v) / len(v))


def unwrap(pos):
    out = [pos[0]]
    acc = 0.0
    for i in range(1, len(pos)):
        d = pos[i] - pos[i - 1]
        while d > 180:
            d -= 360
        while d < -180:
            d += 360
        acc += d
        out.append(pos[0] + acc)
    return out


def load(fn):
    with open(os.path.join(DATA_DIR, fn), 'r', newline='', encoding='utf-8-sig',
              errors='replace') as fh:
        rd = csv.reader(fh)
        h = next(rd)
        c = {k: [] for k in h}
        for row in rd:
            if len(row) < len(h):
                continue
            for k, v in zip(h, row):
                try:
                    c[k].append(float(v))
                except ValueError:
                    c[k].append(float('nan'))
    return c


def segs_of(cmd, tol=1e-6, minlen=200):
    out, s, cur = [], 0, cmd[0]
    for i in range(1, len(cmd)):
        if abs(cmd[i] - cur) > tol:
            if i - s >= minlen:
                out.append((s, i, cur))
            s, cur = i, cmd[i]
    if len(cmd) - s >= minlen:
        out.append((s, len(cmd), cur))
    return out


def resample(ang, sig, want, pts=PTS):
    span = ang[-1] - ang[0]
    avail = int(span // 360)
    if avail < 1:
        return None, 0, 0.0
    cap = min(want, avail)
    p = 1
    while p * 2 <= cap:
        p *= 2
    nrev = p
    a0 = ang[-1] - nrev * 360.0
    N = nrev * pts
    out, i, n = [], 0, len(ang)
    while i < n - 1 and ang[i + 1] < a0:
        i += 1
    idx_of = []
    for j in range(N):
        t = a0 + (j + 0.5) * 360.0 / pts
        while i < n - 1 and ang[i + 1] < t:
            i += 1
        if i >= n - 1:
            i = n - 2
        a1, a2 = ang[i], ang[i + 1]
        w = (t - a1) / (a2 - a1) if a2 > a1 else 0.0
        w = max(0.0, min(1.0, w))
        out.append(sig[i] * (1 - w) + sig[i + 1] * w)
        idx_of.append(i)
    return out, nrev, a0


def full_spectrum(wave, nrev):
    """All bins -> (order, amp, phase) with order possibly fractional."""
    N = len(wave)
    m = mean(wave)
    re = [v - m for v in wave]
    im = [0.0] * N
    fft(re, im)
    res = []
    for k in range(1, N // 2):
        amp = 2.0 * math.hypot(re[k], im[k]) / N
        ph = math.atan2(-im[k], re[k])
        res.append((k / float(nrev), amp, ph))
    return res, m


def reconstruct(spec, m, nrev, keep_orders=None, drop_orders=None, N=None):
    """Rebuild angle-domain waveform from selected orders."""
    if N is None:
        N = int(round(nrev)) * PTS
    out = [m] * N
    for o, amp, ph in spec:
        if amp < 1e-9:
            continue
        if keep_orders is not None and int(round(o)) not in keep_orders:
            continue
        if drop_orders is not None and int(round(o)) in drop_orders:
            continue
        if abs(o - round(o)) > 1e-6:
            continue
        for j in range(N):
            th = 2 * math.pi * o * (j + 0.5) / PTS
            out[j] += amp * math.cos(th + ph)
    return out


def main():
    cc = load('close-50-80-100-120-140_bar_deg.csv')
    cmd = cc['rTargetPressure']
    segs = segs_of(cmd)

    for (a, b, c) in segs:
        if c not in (80, 100, 120, 140):
            continue
        ss = a + int((b - a) * 0.45)
        Praw = cc['rActPressureRaw'][ss:b]
        pos = cc['rActPumpPosition'][ss:b]
        ang = unwrap(pos)
        n = len(Praw)
        shaft = mean([ang[i + 1] - ang[i] for i in range(n - 1)]) * 60000.0 / 360.0
        wave, nrev, _ = resample(ang, Praw, 4)
        if wave is None:
            continue
        spec, m = full_spectrum(wave, nrev)
        # true measured p2p on the resampled window (angle-domain)
        meas_p2p = max(wave) - min(wave)
        meas_rms = rms(wave)
        log()
        log('=' * 100)
        log(f'P_set={c:g} bar  shaft={shaft:.3f} rpm  revs={nrev}  '
            f'resampled p2p={meas_p2p:.4f} bar  rms={meas_rms:.4f} bar  '
            f'(raw window p2p={max(Praw) - min(Praw):.4f}, rms={rms(Praw):.4f})')

        # integer-order reconstruction fidelity
        intspec = [(round(o), amp, ph) for o, amp, ph in spec if abs(o - round(o)) < 1e-6]
        rec = reconstruct(intspec, m, nrev)
        log(f'  integer-order reconstruction: p2p={max(rec) - min(rec):.4f} '
            f'rms={rms(rec):.4f} -> captures {100 * rms(rec) / meas_rms:.1f}% of rms, '
            f'{100 * (max(rec) - min(rec)) / meas_p2p:.1f}% of p2p')
        nonint = [(o, amp) for o, amp, ph in spec if abs(o - round(o)) >= 1e-6]
        nonint.sort(key=lambda t: -t[1])
        log(f'  fractional-order energy (shaft-speed fluctuation signature): '
            f'rms={math.sqrt(sum(a * a for _, a in nonint) / 2):.4f} bar  '
            f'top={[(round(o, 2), round(a, 4)) for o, a in nonint[:5]]}')

        log(f'  --- top 16 integer orders ---')
        srt = sorted(intspec, key=lambda t: -t[1])[:16]
        log(f'  {"ord":>4s} {"amp":>8s} {"phase":>8s} {"f(Hz)":>8s} {"cumP2P_est":>11s}')
        cum = 0.0
        for o, amp, ph in srt:
            cum += amp * amp / 2
            log(f'  {o:4d} {amp:8.4f} {math.degrees(ph):8.1f} {o * shaft / 60:8.3f} '
                f'{4 * math.sqrt(cum):11.4f}')

        # mitigation scenarios with EXACT reconstructed p2p
        tooth = {13, 26, 39, 52, 65, 78, 91, 104}
        scen = [
            ('baseline (measured)', None, None),
            ('cancel 13th', None, {13}),
            ('cancel 13+26', None, {13, 26}),
            ('cancel 13+26+39', None, {13, 26, 39}),
            ('cancel 13+26+39+52', None, {13, 26, 39, 52}),
            ('cancel all 13*k (k=1..8)', None, tooth),
            ('cancel ALL integer orders', None, {o for o, _, _ in intspec}),
        ]
        log(f'  --- mitigation scenarios: exact reconstructed p2p ---')
        log(f'  {"scenario":32s} {"p2p":>8s} {"rms":>8s} {"vs base":>9s}')
        base_p2p = None
        for name, keep, drop in scen:
            if drop is not None and all(o == 0 for o in drop):
                w = wave
            elif keep is not None:
                w = reconstruct(intspec, m, nrev, keep_orders=keep)
            elif drop is not None:
                w = reconstruct(intspec, m, nrev, drop_orders=drop)
            else:
                w = rec
            p2p = max(w) - min(w)
            r = rms(w)
            if base_p2p is None:
                base_p2p = max(rec) - min(rec)
            log(f'  {name:32s} {p2p:8.4f} {r:8.4f} {100 * (1 - p2p / base_p2p):8.1f}%')

        # sensitivity-aware: what if S(f) could be pushed to 0.2 across the ripple band?
        log(f'  --- what-if: uniform sensitivity S in ripple band (orders 13..104) ---')
        for S in (1.0, 0.5, 0.3, 0.2, 0.1, 0.05):
            w = []
            N = len(rec)
            # scale each order's amplitude to open-loop amplitude * S
            for o, amp, ph in intspec:
                if 13 <= o <= 104:
                    pass
            rec2 = [m] * N
            for o, amp, ph in intspec:
                a2 = amp * S if 13 <= o <= 104 else amp
                for j in range(N):
                    th = 2 * math.pi * o * (j + 0.5) / PTS
                    rec2[j] += a2 * math.cos(th + ph)
            log(f'  S={S:5.2f} -> p2p={max(rec2) - min(rec2):8.4f} rms={rms(rec2):8.4f}')

        # non-tooth order scan: look for electrical / mechanical families
        log(f'  --- order family scan (amp>0.05 bar) ---')
        fam = {}
        for o, amp, ph in intspec:
            if amp < 0.05:
                continue
            fam[o] = amp
        for base_o in (1, 2, 3, 4, 6, 8, 12, 13, 24):
            members = [(o, a) for o, a in fam.items() if o % base_o == 0]
            if len(members) >= 3:
                log(f'   multiples of {base_o:3d}: {[(o, round(a, 3)) for o, a in sorted(members)]}'
                    f'  n={len(members)}')


if __name__ == '__main__':
    main()
    with open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'analyze5_out.txt'), 'w',
              encoding='utf-8') as fh:
        fh.write('\n'.join(str(o) for o in OUT))
