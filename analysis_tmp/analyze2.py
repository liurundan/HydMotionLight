"""Corrected spectral / order / step-response analysis for servo-pump pressure tests.

Fixes vs analyze.py:
  - PSD computed on mean-removed + high-pass-detrended data (no DC leakage).
  - df = fs/nfft (was 2x wrong).
  - shaft rpm from position uses 1 ms steps correctly.
Adds:
  - order tracking vs shaft angle with variance decomposition
  - dominant-frequency table across pump speeds (order-related vs fixed)
  - closed-loop step response metrics (rise, overshoot, decay, oscillation freq)
  - empirical transfer gain pressure/speed-command at ripple frequency
Pure standard library.
"""
import csv
import math
import os
import sys

DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'docs', 'data_deg')
FS = 1000.0
OUT = []


def log(msg=''):
    print(msg)
    OUT.append(str(msg))
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
    length = 2
    while length <= n:
        ang = -2.0 * math.pi / length
        wr, wi = math.cos(ang), math.sin(ang)
        half = length >> 1
        for start in range(0, n, length):
            cwr, cwi = 1.0, 0.0
            for k in range(half):
                a, b = start + k, start + k + half
                tr = re[b] * cwr - im[b] * cwi
                ti = re[b] * cwi + im[b] * cwr
                re[b] = re[a] - tr
                im[b] = im[a] - ti
                re[a] += tr
                im[a] += ti
                cwr, cwi = cwr * wr - cwi * wi, cwr * wi + cwi * wr
        length <<= 1
    return re, im


def next_pow2(n):
    p = 1
    while p < n:
        p <<= 1
    return p


def mean(v):
    return sum(v) / len(v) if v else 0.0


def rms(v):
    if not v:
        return 0.0
    m = mean(v)
    return math.sqrt(sum((x - m) ** 2 for x in v) / len(v))


def movavg_lp(x, win):
    """Low-pass: centred moving average (isolates slow drift)."""
    n = len(x)
    if win < 3:
        return list(x)
    half = win // 2
    csum = [0.0] * (n + 1)
    for i, v in enumerate(x):
        csum[i + 1] = csum[i] + v
    out = []
    for i in range(n):
        a = max(0, i - half)
        b = min(n, i + half + 1)
        out.append((csum[b] - csum[a]) / (b - a))
    return out


def welch(x, nfft, hp_win=0):
    n = len(x)
    if hp_win:
        trend = movavg_lp(x, hp_win)
        d = [x[i] - trend[i] for i in range(n)]
    else:
        d = [v - mean(x) for v in x]
    step = nfft // 2
    win = [0.5 - 0.5 * math.cos(2 * math.pi * i / (nfft - 1)) for i in range(nfft)]
    wsum = sum(w * w for w in win)
    acc = [0.0] * (nfft // 2 + 1)
    cnt = 0
    for s in range(0, n - nfft + 1, step):
        re = [d[s + i] * win[i] for i in range(nfft)]
        im = [0.0] * nfft
        fft(re, im)
        for k in range(nfft // 2 + 1):
            scale = 1.0 if k in (0, nfft // 2) else 2.0
            acc[k] += scale * (re[k] * re[k] + im[k] * im[k]) / (FS * wsum)
        cnt += 1
    if cnt == 0:
        return None, None, None, 0
    df = FS / nfft
    freqs = [k * df for k in range(nfft // 2 + 1)]
    psd = [a / cnt for a in acc]
    var_psd = sum(psd) * df
    return freqs, psd, df, cnt


def peaks(freqs, psd, df, npeaks=10, fmin=0.03):
    cand = []
    for i in range(1, len(psd) - 1):
        if freqs[i] < fmin:
            continue
        if psd[i] > psd[i - 1] and psd[i] >= psd[i + 1]:
            cand.append(i)
    cand.sort(key=lambda i: -psd[i])
    res = []
    for i in cand:
        if any(abs(freqs[i] - f) < max(0.4, 2 * df) for f, _, _ in res):
            continue
        # local power around peak -> sinusoid amplitude
        local = 0.0
        for k in range(max(0, i - 3), min(len(psd), i + 4)):
            local += psd[k] * df
        res.append((freqs[i], local, math.sqrt(2 * local)))
        if len(res) >= npeaks:
            break
    return res


def band(psd, df, lo, hi):
    return sum(psd[k] * df for k in range(len(psd)) if lo <= k * df < hi)


def unwrap_deg(pos):
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


def order_profile(ang, sig, m=360):
    """Resample signal onto shaft angle over complete revolutions."""
    n = len(sig)
    total = ang[-1] - ang[0]
    nrev = int(abs(total) // 360)
    if nrev < 1:
        return None
    sgn = 1.0 if total > 0 else -1.0
    sums = [0.0] * m
    sqs = [0.0] * m
    cnts = [0] * m
    idx = 0
    for r in range(nrev):
        lo = ang[0] + sgn * r * 360.0
        for j in range(m):
            target = lo + sgn * (j + 0.5) * 360.0 / m
            while idx < n - 1 and (ang[idx + 1] - target) * sgn < 0:
                idx += 1
            if idx >= n - 1:
                break
            a1, a2 = ang[idx], ang[idx + 1]
            if abs(a2 - a1) < 1e-12:
                continue
            t = (target - a1) / (a2 - a1)
            if t < 0.0 or t > 1.0:
                continue
            val = sig[idx] * (1 - t) + sig[idx + 1] * t
            sums[j] += val
            sqs[j] += val * val
            cnts[j] += 1
    sync = [sums[j] / cnts[j] if cnts[j] else None for j in range(m)]
    if all(s is None for s in sync):
        return None
    mv = mean([s for s in sync if s is not None])
    sync = [s if s is not None else mv for s in sync]
    # residual rms
    resid_sq = 0.0
    resid_n = 0
    for j in range(m):
        if cnts[j]:
            resid_sq += cnts[j] * (sqs[j] / cnts[j] - sync[j] ** 2)
            resid_n += cnts[j]
    resid = math.sqrt(max(resid_sq / resid_n, 0.0)) if resid_n else 0.0
    tr = rms(sig)
    sr = rms(sync)
    return {'nrev': nrev, 'sync': sync, 'sync_rms': sr, 'sync_p2p': max(sync) - min(sync),
            'tot_rms': tr, 'resid_rms': resid,
            'var_expl': (sr / tr) ** 2 if tr > 1e-12 else 0.0, 'm': m}


def order_spectrum(sync):
    m = len(sync)
    base = [v - mean(sync) for v in sync]
    nfft = next_pow2(m)
    re = base + [0.0] * (nfft - m)
    im = [0.0] * nfft
    fft(re, im)
    out = []
    for k in range(1, nfft // 2):
        order = k * m / float(nfft)
        if order > m / 2:
            break
        # zero-padding interpolates; amplitude scale uses original length m
        amp = 2.0 * math.hypot(re[k], im[k]) / m
        out.append((round(order, 2), amp))
    return out


def step_metrics(t, y, target, base):
    """Rise time, overshoot, settling for a step from base to target."""
    span = target - base
    if abs(span) < 1e-9:
        return None
    t10 = t50 = t90 = None
    pk = base
    tpk = None
    for i, v in enumerate(y):
        if t10 is None and (v - base) / span >= 0.1:
            t10 = t[i]
        if t50 is None and (v - base) / span >= 0.5:
            t50 = t[i]
        if t90 is None and (v - base) / span >= 0.9:
            t90 = t[i]
        if (v - base) / span > (pk - base) / span:
            pk = v
            tpk = t[i]
        if t90 is not None and t[i] - (t90 or 0) > 3.0:
            break
    over = (pk - target) / span * 100.0 if span else 0.0
    return {'t10': t10, 't50': t50, 't90': t90, 'rise': (t90 - t10) if (t10 and t90) else None,
            'overshoot_pct': over, 'peak': pk, 't_peak': tpk}


def load(fname):
    path = os.path.join(DATA_DIR, fname)
    with open(path, 'r', newline='', encoding='utf-8-sig', errors='replace') as fh:
        rd = csv.reader(fh)
        header = next(rd)
        cols = {h: [] for h in header}
        for row in rd:
            if len(row) < len(header):
                continue
            for h, v in zip(header, row):
                try:
                    cols[h].append(float(v))
                except ValueError:
                    cols[h].append(float('nan'))
    return cols


def segments(cmd, tol=1e-6, min_len=200):
    segs, start, cur = [], 0, cmd[0]
    for i in range(1, len(cmd)):
        if abs(cmd[i] - cur) > tol:
            if i - start >= min_len:
                segs.append((start, i, cur))
            start, cur = i, cmd[i]
    if len(cmd) - start >= min_len:
        segs.append((start, len(cmd), cur))
    return segs


def uniq(v):
    return sorted(set(v))


SUMMARY = []


def analyse(tag, cols, a, b, hp_win=1000):
    sl = slice(a, b)
    P = cols['rActPressureRaw'][sl]
    Pf = cols['rActPressureFiltered'][sl]
    pos = cols['rActPumpPosition'][sl]
    nc = cols['rTargetSpeedRPM'][sl]
    fl = cols['rPIDControllerOutputFlow'][sl]
    tq = cols['rActTorquePercent'][sl]
    nf = cols['rActRawSpeedRPM'][sl]
    n = len(P)
    ang = unwrap_deg(pos)
    dpos = [(ang[i + 1] - ang[i]) for i in range(n - 1)]
    shaft_rpm = mean(dpos) * 60000.0 / 360.0
    rev_period = 60.0 / shaft_rpm if shaft_rpm > 1e-9 else float('inf')

    log()
    log('#' * 98)
    log(f'# {tag}   samples={n} ({n / FS:.2f}s)  window idx[{a}:{b}]')
    log('#' * 98)
    log(f'P_raw  mean={mean(P):8.3f} sd={rms(P):7.4f} p2p={max(P) - min(P):7.3f} bar')
    log(f'P_filt mean={mean(Pf):8.3f} sd={rms(Pf):7.4f} p2p={max(Pf) - min(Pf):7.3f} bar  '
        f'(filter variance reduction {100 * (1 - (rms(Pf) / rms(P)) ** 2):.2f}%)')
    log(f'shaft  true speed={shaft_rpm:8.4f} rpm  rev period={rev_period:7.3f} s  '
        f'revs in window={(n / FS) / rev_period if rev_period != float("inf") else 0:.1f}')
    log(f'n_cmd  mean={mean(nc):8.4f} sd={rms(nc):7.4f} p2p={max(nc) - min(nc):7.3f} rpm '
        f'(sd/mean={100 * rms(nc) / mean(nc) if mean(nc) else 0:.1f}%)')
    log(f'flow   mean={mean(fl):8.4f} sd={rms(fl):7.4f} p2p={max(fl) - min(fl):7.3f} L/min')
    log(f'torque mean={mean(tq):8.3f} sd={rms(tq):7.4f} p2p={max(tq) - min(tq):7.3f} %')
    uf = uniq(nf)
    log(f'n_fb   unique values={[round(u, 4) for u in uf[:6]]} count={len(uf)}')
    up = uniq(pos)
    if len(up) > 2:
        st = min(up[i + 1] - up[i] for i in range(len(up) - 1) if up[i + 1] - up[i] > 1e-9)
        log(f'encoder: position LSB={st:.6f} deg -> {360 / st:.0f} counts/rev '
            f'({math.log(360 / st, 2):.1f} bit); speed LSB=1 count/ms={st * 60000 / 360:.4f} rpm')

    # ---------- slow drift vs fast ripple decomposition ----------
    for W in (1000, 200):
        trend = movavg_lp(P, W)
        resid = [P[i] - trend[i] for i in range(n)]
        log(f'split @{W}ms MA: trend_p2p={max(trend) - min(trend):7.3f} trend_sd={rms(trend):6.3f} | '
            f'fast_p2p={max(resid) - min(resid):7.3f} fast_sd={rms(resid):6.3f} bar')

    # ---------- PSD (detrended) ----------
    nfft = 4096
    freqs, psd, df, cnt = welch(P, nfft, hp_win=hp_win)
    if not freqs:
        log('PSD: not enough data')
        return None
    tot = sum(psd) * df
    log(f'PSD (HP-detrend {hp_win} ms): windows={cnt} df={df:.4f} Hz  residual var={tot:.4f} '
        f'-> rms={math.sqrt(tot):.4f} bar (raw sd={rms(P):.4f})')
    bands = [(0.03, 0.3), (0.3, 1.0), (1.0, 3.0), (3.0, 10.0), (10.0, 30.0),
             (30.0, 80.0), (80.0, 200.0), (200.0, 500.0)]
    for lo, hi in bands:
        p = band(psd, df, lo, hi)
        log(f'   {lo:6.2f}-{hi:6.1f} Hz var={p:9.5f} rms={math.sqrt(max(p, 0)):7.4f} '
            f'share={100 * p / tot if tot else 0:6.2f}%')
    pk = peaks(freqs, psd, df, npeaks=8)
    log('   peaks: ' + ' | '.join(f'{f:.3f}Hz rms={math.sqrt(max(p, 0)):.3f} amp={a:.3f}'
                                  for f, p, a in pk))
    f1 = pk[0][0] if pk else 0.0
    if f1 > 1e-6 and shaft_rpm > 1e-9:
        log(f'   dominant {f1:.3f} Hz -> shaft order = {f1 / (shaft_rpm / 60.0):.2f} '
            f'(rev freq {shaft_rpm / 60.0:.4f} Hz)')

    # ---------- order tracking ----------
    for name, sig in (('P_raw', P), ('torque', tq), ('n_cmd', nc)):
        oa = order_profile(ang, sig, m=360)
        if oa is None:
            log(f'order[{name}]: <1 revolution')
            continue
        sp = sorted(order_spectrum(oa['sync']), key=lambda t: -t[1])[:5]
        log(f'order[{name}]: revs={oa["nrev"]} sync_p2p={oa["sync_p2p"]:.4f} sync_rms={oa["sync_rms"]:.4f} '
            f'tot_rms={oa["tot_rms"]:.4f} resid_rms={oa["resid_rms"]:.4f} '
            f'var_explained={100 * oa["var_expl"]:.1f}%  top(order,amp)={[(k, round(a, 3)) for k, a in sp]}')

    # ---------- coherence-ish: pressure vs n_cmd ----------
    if rms(nc) > 1e-6:
        freqs2, psd2, df2, _ = welch(nc, nfft, hp_win=hp_win)
        if freqs2:
            pk2 = peaks(freqs2, psd2, df2, npeaks=4)
            log(f'n_cmd peaks: ' + ' | '.join(f'{f:.3f}Hz amp={a:.4f}' for f, p, a in pk2))
            # amplitude ratio at shared dominant freq
            if pk and pk2:
                ratio = pk[0][2] / pk2[0][2] if pk2[0][2] > 1e-12 else float('inf')
                log(f'amplitude ratio P/n_cmd at dominant freq = {ratio:.3f} bar/rpm '
                    f'(P peak {pk[0][2]:.4f} bar @ {pk[0][0]:.3f} Hz, '
                    f'n peak {pk2[0][2]:.4f} rpm @ {pk2[0][0]:.3f} Hz)')
    # ---------- cross-correlation P vs n_cmd (loop self-oscillation check) ----------
    if rms(nc) > 1e-6:
        dp = [P[i] - mean(P) for i in range(n)]
        dn = [nc[i] - mean(nc) for i in range(n)]
        nn = min(len(dp), 8000)
        dp, dn = dp[:nn], dn[:nn]
        nfft = next_pow2(2 * nn)
        ar = dp + [0.0] * (nfft - nn)
        ai = [0.0] * nfft
        br = dn + [0.0] * (nfft - nn)
        bi = [0.0] * nfft
        fft(ar, ai)
        fft(br, bi)
        cr = [ar[k] * br[k] + ai[k] * bi[k] for k in range(nfft)]
        ci = [ai[k] * br[k] - ar[k] * bi[k] for k in range(nfft)]
        fft(cr, ci)
        best = None
        for lag in range(-400, 401):
            v = cr[lag % nfft]
            if best is None or v > best[1]:
                best = (lag, v)
        d0 = cr[0] if cr[0] else 1.0
        lag, v = best
        # direction: P leads or lags n_cmd
        lead = 'n_cmd leads P' if lag > 0 else ('P leads n_cmd' if lag < 0 else 'in phase')
        log(f'xcorr(P, n_cmd): best lag={lag} ms  norm={v / d0:.3f}  ({lead})')
        # also P vs torque
        dt_ = [tq[i] - mean(tq) for i in range(n)][:nn]
        br2 = dt_ + [0.0] * (nfft - nn)
        bi2 = [0.0] * nfft
        fft(br2, bi2)
        cr2 = [ar[k] * br2[k] + ai[k] * bi2[k] for k in range(nfft)]
        ci2 = [ai[k] * br2[k] - ar[k] * bi2[k] for k in range(nfft)]
        fft(cr2, ci2)
        best2 = None
        for lag2 in range(-400, 401):
            v2 = cr2[lag2 % nfft]
            if best2 is None or v2 > best2[1]:
                best2 = (lag2, v2)
        d02 = cr2[0] if cr2[0] else 1.0
        log(f'xcorr(P, torque): best lag={best2[0]} ms  norm={best2[1] / d02:.3f}')

    SUMMARY.append({'tag': tag, 'P_mean': mean(P), 'P_sd': rms(P), 'P_p2p': max(P) - min(P),
                    'shaft_rpm': shaft_rpm, 'n_cmd_mean': mean(nc), 'n_cmd_sd': rms(nc),
                    'flow_mean': mean(fl), 'flow_sd': rms(fl), 'tq_mean': mean(tq),
                    'tq_sd': rms(tq), 'f_dom': f1, 'amp_dom': pk[0][2] if pk else 0.0})
    return SUMMARY[-1]


def main():
    # steady-state windows: use last 60% of each active segment
    for fname, cmdcol in (('open-8-10-12_rpm.csv', 'rTargetSpeedRPM'),
                          ('open-15-20_rpmp.csv', 'rTargetSpeedRPM'),
                          ('close-50-80-100-120-140_bar_deg.csv', 'rTargetPressure')):
        cols = load(fname)
        cmd = cols[cmdcol]
        segs = segments(cmd)
        log('=' * 98)
        log(f'FILE {fname}')
        is_open = 'open' in fname
        for (a, b, c) in segs:
            if c == 0 or b - a < 4500:
                continue
            ss = a + int((b - a) * (0.15 if is_open else 0.40))
            analyse(f'{fname} cmd={c:g}', cols, ss, b)

    # ---------- closed-loop step response ----------
    cols = load('close-50-80-100-120-140_bar_deg.csv')
    cmd = cols['rTargetPressure']
    P = cols['rActPressureRaw']
    nc = cols['rTargetSpeedRPM']
    segs = segments(cmd)
    log()
    log('=' * 98)
    log('CLOSED-LOOP STEP RESPONSE (from pressure command transitions)')
    for i in range(len(segs) - 1):
        a0, b0, c0 = segs[i]
        a1, b1, c1 = segs[i + 1]
        if c1 == 0:
            continue
        # base = mean pressure just before transition
        base_win = P[max(0, a1 - 300):a1]
        base = mean(base_win) if base_win else 0.0
        t = [(k - a1) / FS for k in range(a1, min(b1, a1 + 12000))]
        y = P[a1:min(b1, a1 + 12000)]
        sm = step_metrics(t, y, c1, base)
        if not sm:
            continue
        # n_cmd peak during transient
        nseg = nc[a1:min(b1, a1 + 12000)]
        log(f'step {base:6.2f} -> {c1:6.2f} bar : '
            f't10={sm["t10"] if sm["t10"] is None else round(sm["t10"], 3)}s '
            f't50={sm["t50"] if sm["t50"] is None else round(sm["t50"], 3)}s '
            f't90={sm["t90"] if sm["t90"] is None else round(sm["t90"], 3)}s '
            f'rise={sm["rise"] if sm["rise"] is None else round(sm["rise"], 3)}s '
            f'overshoot={sm["overshoot_pct"]:6.2f}% peak={sm["peak"]:7.2f}bar '
            f'@{sm["t_peak"]:.3f}s  n_cmd max={max(nseg):7.2f} mean={mean(nseg):7.3f} rpm')

    # ---------- summary ----------
    log()
    log('=' * 98)
    log('SUMMARY (steady-state windows)')
    log(f'{"case":38s} {"P_mean":>8s} {"P_sd":>7s} {"P_p2p":>7s} {"shaft":>8s} '
        f'{"n_sd":>7s} {"flow":>8s} {"f_dom":>7s} {"amp":>7s} {"bar/rpm":>8s}')
    for s in SUMMARY:
        gain = s['P_sd'] / s['n_cmd_sd'] if s['n_cmd_sd'] > 1e-6 else float('nan')
        log(f'{s["tag"][:38]:38s} {s["P_mean"]:8.2f} {s["P_sd"]:7.3f} {s["P_p2p"]:7.3f} '
            f'{s["shaft_rpm"]:8.3f} {s["n_cmd_sd"]:7.3f} {s["flow_mean"]:8.4f} '
            f'{s["f_dom"]:7.3f} {s["amp_dom"]:7.3f} {gain:8.3f}')
    log()
    log('OPEN-LOOP static characteristic (dead-headed pump):')
    op = sorted([(s['shaft_rpm'], s['P_mean'], s['P_sd'], s['P_p2p'], abs(s['tq_mean']))
                 for s in SUMMARY if 'open' in s['tag'] and s['shaft_rpm'] > 0])
    for r, p, sd, pp, tq in op:
        log(f'   n={r:7.3f} rpm  P={p:7.3f} bar  sd={sd:6.3f}  p2p={pp:6.3f}  '
            f'|T|={tq:6.2f}%  bar/rpm={p / r:6.3f}  %/bar={tq / p:6.4f}  leak L/min={r / 20.0 * 0.05:6.4f}')
    if len(op) >= 2:
        xs = [o[0] for o in op]
        ys = [o[1] for o in op]
        mx, my = mean(xs), mean(ys)
        slope = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sum((x - mx) ** 2 for x in xs)
        log(f'   linear fit P = {slope:.4f}*n + {my - slope * mx:.3f}  [bar]')
        # leakage coefficient from closed loop
    cl = sorted([(s['n_cmd_mean'], s['P_mean'], s['flow_mean'], abs(s['tq_mean']))
                 for s in SUMMARY if 'close' in s['tag'] and s['n_cmd_mean'] > 0])
    log('CLOSED-LOOP steady operating points (flow = leakage compensation):')
    for n_, p_, f_, tq in cl:
        log(f'   P={p_:7.2f} bar  n_cmd={n_:7.3f} rpm  flow={f_:7.4f} L/min  |T|={tq:6.2f}%  '
            f'K_flow2rpm={n_ / f_:6.3f} rpm/(L/min)  leak_coef={f_ / p_ * 1000:7.4f} mL/min/bar  '
            f'|T|/P={tq / p_:6.4f} %/bar')


if __name__ == '__main__':
    main()
    with open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'analyze2_out.txt'), 'w',
              encoding='utf-8') as fh:
        fh.write('\n'.join(str(o) for o in OUT))
