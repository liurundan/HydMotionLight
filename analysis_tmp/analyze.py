"""Spectral / order-tracking / transient analysis of servo-pump pressure test data.

Pure standard library. Sampling is a fixed 1 ms (fs = 1000 Hz).
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
    OUT.append(msg)
    sys.stdout.flush()


# ---------------------------------------------------------------- FFT (radix-2)
def fft(re, im):
    n = len(re)
    if n & (n - 1):
        raise ValueError('length must be power of two')
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
        wr = math.cos(ang)
        wi = math.sin(ang)
        half = length >> 1
        for start in range(0, n, length):
            cwr, cwi = 1.0, 0.0
            for k in range(half):
                a = start + k
                b = a + half
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


def detrend(v):
    m = mean(v)
    return [x - m for x in v]


# ---------------------------------------------------------------- Welch PSD
def welch(x, nfft=4096, overlap=0.5):
    n = len(x)
    step = max(1, int(nfft * (1 - overlap)))
    win = [0.5 - 0.5 * math.cos(2 * math.pi * i / (nfft - 1)) for i in range(nfft)]
    wsum = sum(w * w for w in win)
    acc = [0.0] * (nfft // 2 + 1)
    cnt = 0
    for s in range(0, n - nfft + 1, step):
        re = [x[s + i] * win[i] for i in range(nfft)]
        im = [0.0] * nfft
        fft(re, im)
        for k in range(nfft // 2 + 1):
            p = (re[k] * re[k] + im[k] * im[k]) / (FS * wsum)
            acc[k] += p * (1.0 if k in (0, nfft // 2) else 2.0)
        cnt += 1
    if cnt == 0:
        return None, None, 0
    freqs = [k * FS / nfft for k in range(nfft // 2 + 1)]
    psd = [a / cnt for a in acc]
    return freqs, psd, cnt


def band_power(freqs, psd, lo, hi):
    df = FS / (len(freqs) - 1) / 2.0 * 2.0  # = FS/nfft
    tot = 0.0
    for f, p in zip(freqs, psd):
        if lo <= f < hi:
            tot += p * df
    return tot


def top_peaks(freqs, psd, npeaks=12, fmin=0.05):
    idx = []
    for i in range(1, len(psd) - 1):
        if freqs[i] < fmin:
            continue
        if psd[i] > psd[i - 1] and psd[i] >= psd[i + 1]:
            idx.append(i)
    idx.sort(key=lambda i: -psd[i])
    picks = []
    for i in idx:
        if any(abs(freqs[i] - freqs[j]) < 0.35 for j in [p for p, _ in picks]):
            continue
        amp = math.sqrt(psd[i] * (FS / (len(freqs) - 1) / 2.0) * 2.0)
        picks.append((i, (freqs[i], psd[i], amp)))
        if len(picks) >= npeaks:
            break
    return [p for _, p in picks]


# ---------------------------------------------------------------- autocorrelation
def autocorr_period(x, max_lag=6000):
    n = min(len(x), 30000)
    x = x[:n]
    d = detrend(x)
    nfft = next_pow2(2 * n)
    re = d + [0.0] * (nfft - n)
    im = [0.0] * nfft
    fft(re, im)
    for k in range(nfft):
        a, b = re[k], im[k]
        re[k] = a * a + b * b
        im[k] = 0.0
    fft(re, im)
    ac = re[:max_lag + 1]
    ac0 = ac[0] if ac[0] else 1.0
    norm = [a / ac0 for a in ac]
    best = (0, -2.0)
    for lag in range(20, max_lag + 1):
        if norm[lag] > best[1] and norm[lag] > norm[lag - 1] and norm[lag] >= norm[lag + 1 if lag < max_lag else lag]:
            best = (lag, norm[lag])
    return best, norm


# ---------------------------------------------------------------- order tracking
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


def order_analysis(ang, sig, m=360):
    """Bin signal by shaft angle over complete revolutions; return synchronous waveform."""
    n = len(sig)
    a0 = ang[0]
    base = int(math.floor((a0 - a0) / 360))
    start_rev = int(math.ceil((ang[0] - ang[0]) / 360.0))
    # revolutions covered fully
    total_deg = ang[-1] - ang[0]
    nrev = int(total_deg // 360)
    if nrev < 1:
        return None
    sums = [0.0] * m
    sqs = [0.0] * m
    cnts = [0] * m
    idx = 0
    for r in range(nrev):
        lo = ang[0] + r * 360.0
        hi = lo + 360.0
        for j in range(m):
            target = lo + (j + 0.5) * 360.0 / m
            while idx < n - 1 and ang[idx + 1] < target:
                idx += 1
            if idx >= n - 1:
                break
            a1, a2 = ang[idx], ang[idx + 1]
            if a2 <= a1:
                continue
            t = (target - a1) / (a2 - a1)
            if t < -0.2 or t > 1.2:
                continue
            t = max(0.0, min(1.0, t))
            val = sig[idx] * (1 - t) + sig[idx + 1] * t
            sums[j] += val
            sqs[j] += val * val
            cnts[j] += 1
    sync = []
    resid_rms_sq = 0.0
    resid_n = 0
    for j in range(m):
        if cnts[j] > 0:
            sync.append(sums[j] / cnts[j])
        else:
            sync.append(None)
    # variance decomposition
    valid = [s for s in sync if s is not None]
    if not valid:
        return None
    sync_rms = rms(valid)
    tot_rms = rms(sig)
    return {
        'nrev': nrev,
        'sync_rms': sync_rms,
        'tot_rms': tot_rms,
        'var_explained': (sync_rms / tot_rms) ** 2 if tot_rms else 0.0,
        'sync': [s if s is not None else mean(valid) for s in sync],
        'm': m,
    }


def order_fft(wave):
    m = len(wave)
    re = list(wave)
    im = [0.0] * m
    fft(re, im)
    amps = []
    for k in range(1, m // 2):
        amps.append((k, 2.0 * math.hypot(re[k], im[k]) / m))
    return amps


# ---------------------------------------------------------------- io
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


def uniq_sorted(v):
    return sorted(set(v))


def report_segment(tag, cols, a, b):
    sl = slice(a, b)
    praw = cols['rActPressureRaw'][sl]
    pfilt = cols['rActPressureFiltered'][sl]
    pos = cols['rActPumpPosition'][sl]
    nspd = cols['rTargetSpeedRPM'][sl]
    flow = cols['rPIDControllerOutputFlow'][sl]
    torq = cols['rActTorquePercent'][sl]
    rawspd = cols['rActRawSpeedRPM'][sl]
    n = len(praw)
    log()
    log('#' * 96)
    log(f'# {tag}  samples={n} ({n / FS:.2f}s)')
    log('#' * 96)
    log(f'P_raw   mean={mean(praw):9.4f} sd={rms(praw):8.4f} p2p={max(praw) - min(praw):8.4f}')
    log(f'P_filt  mean={mean(pfilt):9.4f} sd={rms(pfilt):8.4f} p2p={max(pfilt) - min(pfilt):8.4f}')
    log(f'filter variance reduction = {100 * (1 - (rms(pfilt) / rms(praw)) ** 2):.2f}%')
    log(f'P_raw LSB(unique gaps)={uniq_sorted([round(praw[i+1]-praw[i],4) for i in range(min(2000,n-1))])[:5]}')
    log(f'torque  mean={mean(torq):9.4f} sd={rms(torq):8.4f} p2p={max(torq)-min(torq):8.4f}')
    log(f'flow_cmd mean={mean(flow):9.4f} sd={rms(flow):8.4f} p2p={max(flow)-min(flow):8.4f}')
    log(f'n_cmd   mean={mean(nspd):9.4f} sd={rms(nspd):8.4f} p2p={max(nspd)-min(nspd):8.4f}')
    uq = uniq_sorted(rawspd)
    log(f'n_fb    mean={mean(rawspd):9.4f} sd={rms(rawspd):8.4f} unique={len(uq)} values={[round(u,4) for u in uq[:8]]}')
    if len(uq) > 2:
        log(f'n_fb LSB={min(uq[i+1]-uq[i] for i in range(len(uq)-1) if uq[i+1]-uq[i]>1e-9):.4f} rpm')
    # position quantization
    up = uniq_sorted(pos)
    if len(up) > 2:
        st = min(up[i + 1] - up[i] for i in range(len(up) - 1) if up[i + 1] - up[i] > 1e-9)
        log(f'shaft position LSB={st:.6f} deg  ({360.0 / st:.1f} counts/rev)')
    dpos = []
    for i in range(len(pos) - 1):
        d = pos[i + 1] - pos[i]
        while d > 180:
            d -= 360
        while d < -180:
            d += 360
        dpos.append(d)
    md = mean(dpos)
    log(f'shaft speed from position = {md * 60 / 360:.4f} rpm  ({md:.5f} deg/ms), '
        f'rev period = {360.0 / md / 1000.0 if md > 0 else float("inf"):.3f} s')

    # ---- Welch PSD of pressure
    nfft = 4096 if n >= 4096 else next_pow2(max(256, n // 2))
    freqs, psd, cnt = welch(praw, nfft=nfft)
    if freqs:
        total = band_power(freqs, psd, 0.0, FS / 2)
        log(f'PSD: nfft={nfft} windows={cnt} total_var={total:.5f} (signal var={rms(praw) ** 2:.5f})')
        bands = [(0.05, 0.5), (0.5, 2), (2, 5), (5, 20), (20, 50), (50, 100), (100, 250), (250, 500)]
        for lo, hi in bands:
            p = band_power(freqs, psd, lo, hi)
            log(f'   band {lo:6.2f}-{hi:6.1f} Hz : var={p:9.5f}  rms={math.sqrt(max(p, 0)):8.4f}  '
                f'{100 * p / total if total else 0:6.2f}%')
        pk = top_peaks(freqs, psd, npeaks=8)
        log('   top spectral peaks (Hz, PSD, approx sin amplitude):')
        for f, p, amp in pk:
            log(f'      {f:8.3f} Hz  psd={p:10.5f}  amp={amp:8.4f}')

    # ---- autocorrelation dominant period
    (lag, val), norm = autocorr_period(praw, max_lag=min(6000, n // 2))
    log(f'autocorr: dominant lag={lag} ms ({lag / 1000.0:.3f} s) rho={val:.4f}')
    short = [(i, round(norm[i], 3)) for i in range(20, min(400, len(norm))) if
             norm[i] > norm[i - 1] and norm[i] >= norm[i + 1]]
    short.sort(key=lambda t: -t[1])
    log(f'autocorr local peaks (lag_ms, rho) top6: {short[:6]}')

    # ---- order tracking
    ang = unwrap_deg(pos)
    for name, sig in (('P_raw', praw), ('n_cmd', nspd), ('flow_cmd', flow), ('torque', torq)):
        oa = order_analysis(ang, sig, m=360)
        if oa is None:
            log(f'order[{name}]: insufficient revolutions')
            continue
        amps = order_fft(oa['sync'])
        amps_sorted = sorted(amps, key=lambda t: -t[1])[:6]
        log(f'order[{name}]: revs={oa["nrev"]} sync_rms={oa["sync_rms"]:.4f} '
            f'tot_rms={oa["tot_rms"]:.4f} var_explained={100 * oa["var_explained"]:.2f}%  '
            f'top orders(order,amp)={[(k, round(a, 4)) for k, a in amps_sorted]}')
        if name == 'P_raw' and oa['nrev'] >= 2:
            sy = oa['sync']
            log(f'   sync waveform stats: mean={mean(sy):.4f} p2p={max(sy) - min(sy):.4f} rms={rms(sy):.4f}')
    return {'mean_p': mean(praw), 'sd_p': rms(praw), 'p2p_p': max(praw) - min(praw),
            'mean_n': mean(nspd), 'mean_flow': mean(flow), 'mean_torque': mean(torq),
            'shaftrpm': md * 60 / 360 if md else 0.0}


def main():
    results = {}
    for fname, cmdcol in (('open-8-10-12_rpm.csv', 'rTargetSpeedRPM'),
                          ('open-15-20_rpmp.csv', 'rTargetSpeedRPM'),
                          ('close-50-80-100-120-140_bar_deg.csv', 'rTargetPressure')):
        cols = load(fname)
        cmd = cols[cmdcol]
        segs = segments(cmd)
        log('=' * 96)
        log(f'FILE {fname}')
        for (a, b, c) in segs:
            if c == 0:
                continue
            ln = b - a
            if ln < 3000:
                continue
            ss = a + int(ln * 0.25)
            key = f'{fname}|{c:g}'
            results[key] = report_segment(f'{fname} cmd={c:g}', cols, ss, b)

    log()
    log('=' * 96)
    log('SUMMARY TABLE (steady state)')
    log(f'{"case":44s} {"P_mean":>9s} {"P_sd":>8s} {"P_p2p":>8s} {"n_rpm":>8s} {"flow":>8s} {"torque":>9s}')
    for k, v in results.items():
        log(f'{k:44s} {v["mean_p"]:9.3f} {v["sd_p"]:8.3f} {v["p2p_p"]:8.3f} '
            f'{v["mean_n"]:8.3f} {v["mean_flow"]:8.4f} {v["mean_torque"]:9.3f}')
    # static gain fits
    open_cases = [(v['shaftrpm'], v['mean_p']) for k, v in results.items() if 'open' in k and v['shaftrpm'] > 0]
    closed_cases = [(v['mean_n'], v['mean_p'], v['mean_flow']) for k, v in results.items()
                    if 'close' in k and v['mean_n'] > 0]
    log()
    log('OPEN-LOOP  rpm -> bar : ' + ', '.join(f'({r:.2f},{p:.2f})' for r, p in sorted(open_cases)))
    if len(open_cases) >= 2:
        pts = sorted(open_cases)
        slope = (pts[-1][1] - pts[0][1]) / (pts[-1][0] - pts[0][0])
        inter = pts[0][1] - slope * pts[0][0]
        log(f'   least-squares-ish gain K = {slope:.4f} bar/rpm, offset = {inter:.3f} bar')
        log('   per-point bar/rpm: ' + ', '.join(f'{p / r:.3f}' for r, p in pts))
    log('CLOSED-LOOP n_cmd/P/flow : ' + ', '.join(f'({n:.2f},{p:.2f},{f:.4f})'
                                                  for n, p, f in sorted(closed_cases)))
    for n, p, f in sorted(closed_cases):
        if f > 1e-6:
            log(f'   P={p:7.2f} bar  flow={f:7.4f} L/min  Kp_flow={p / f:8.3f} bar/(L/min)  '
                f'bar/rpm={p / n:6.3f}  gain={n / f:6.3f} rpm/(L/min)')


if __name__ == '__main__':
    main()
    with open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'analyze_out.txt'), 'w',
              encoding='utf-8') as fh:
        fh.write('\n'.join(OUT))
