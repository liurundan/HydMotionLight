"""Focused identification: sensor noise floor, plant (R_h, C_h, tau, delay),
loop gain / crossover, measured sensitivity S(f), and quantified improvement
scenarios for pressure ripple reduction.
"""
import csv
import math
import os
import sys

DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'docs', 'data_deg')
FS = 1000.0
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


def welch_psd(x, nfft=2048):
    n = len(x)
    m = mean(x)
    d = [v - m for v in x]
    step = nfft // 2
    win = [0.5 - 0.5 * math.cos(2 * math.pi * i / (nfft - 1)) for i in range(nfft)]
    ws = sum(w * w for w in win)
    acc = [0.0] * (nfft // 2 + 1)
    cnt = 0
    for s in range(0, n - nfft + 1, step):
        re = [d[s + i] * win[i] for i in range(nfft)]
        im = [0.0] * nfft
        fft(re, im)
        for k in range(nfft // 2 + 1):
            sc = 1.0 if k in (0, nfft // 2) else 2.0
            acc[k] += sc * (re[k] ** 2 + im[k] ** 2) / (FS * ws)
        cnt += 1
    df = FS / nfft
    return [k * df for k in range(nfft // 2 + 1)], [a / cnt for a in acc], df, cnt


def band(psd, df, lo, hi):
    return sum(psd[k] * df for k in range(len(psd)) if lo <= k * df < hi)


def resample_revs(ang, sig, want, pts=512):
    span = ang[-1] - ang[0]
    avail = int(span // 360)
    if avail < 1:
        return None, 0
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
    return out, nrev


def order_amp(wave, nrev, orders):
    N = len(wave)
    m = mean(wave)
    re = [v - m for v in wave]
    im = [0.0] * N
    fft(re, im)
    res = {}
    for o in orders:
        k = o * nrev
        if k >= N // 2:
            continue
        res[o] = (2.0 * math.hypot(re[k], im[k]) / N,
                  math.degrees(math.atan2(-im[k], re[k])))
    return res


GAIN_RPM_PER_FLOW = 20.0   # verified from data: n_cmd / flow_cmd == 20.000 exactly
ORDS = [1, 2, 3, 13, 26, 39, 52, 65, 78, 91, 104]


def main():
    close = 'close-50-80-100-120-140_bar_deg.csv'
    cc = load(close)
    cmd = cc['rTargetPressure']
    segs = segs_of(cmd)

    # ---------------- 1. sensor noise floor ----------------
    log('=' * 96)
    log('1. SENSOR NOISE FLOOR AND FILTER CHARACTERISATION')
    for (a, b, c) in segs:
        if c == 0 or b - a < 5000:
            continue
        ss = a + int((b - a) * 0.5)
        raw = cc['rActPressureRaw'][ss:b]
        fil = cc['rActPressureFiltered'][ss:b]
        dif = [raw[i] - fil[i] for i in range(len(raw))]
        fr, psr, dfr, _ = welch_psd(raw, 2048)
        ff_, psf, dff, _ = welch_psd(fil, 2048)
        fd, psd_, dfd, _ = welch_psd(dif, 2048)
        log(f'--- P_set={c:g} bar, n={len(raw)} ---')
        log(f'  raw rms={rms(raw):.4f}  filt rms={rms(fil):.4f}  (raw-filt) rms={rms(dif):.4f} bar')
        for lo, hi in ((0.5, 5), (5, 20), (20, 50), (50, 150), (150, 300), (300, 500)):
            pr = math.sqrt(max(band(psr, dfr, lo, hi), 0))
            pf = math.sqrt(max(band(psf, dff, lo, hi), 0))
            pd = math.sqrt(max(band(psd_, dfd, lo, hi), 0))
            att = pf / pr if pr > 1e-9 else float('nan')
            log(f'    {lo:6.1f}-{hi:6.1f} Hz : raw={pr:7.4f} filt={pf:7.4f} filt/raw={att:6.3f} '
                f'diff={pd:7.4f}')
        # estimate white-noise floor from 300-500 Hz band (no pump harmonics that high
        # at these shaft speeds: 104th order @22.7rpm = 39 Hz, so >200 Hz is pure noise)
        floor = math.sqrt(max(band(psf, dff, 200, 500), 0))
        floor_r = math.sqrt(max(band(psr, dfr, 200, 500), 0))
        log(f'  white-noise floor estimate (200-500 Hz): raw={floor_r:.4f} filt={floor:.4f} bar rms')
        # LSB / quantisation
        uq = sorted(set(raw))
        gaps = [uq[i + 1] - uq[i] for i in range(len(uq) - 1) if uq[i + 1] - uq[i] > 1e-9]
        lsb = min(gaps) if gaps else float('nan')
        log(f'  raw LSB={lsb:.5f} bar -> quantisation noise={lsb / math.sqrt(12):.5f} bar rms '
            f'(negligible: {100 * (lsb / math.sqrt(12)) / rms(raw):.3f}% of signal)')

    # ---------------- 2. plant identification ----------------
    log()
    log('=' * 96)
    log('2. PLANT IDENTIFICATION (dead-headed pressure-hold, flow-domain)')
    log('   model: C_h dP/dt = Q_pump - P/R_h ; G(s)=P/Q = R_h/(1+s*R_h*C_h)')
    rows = []
    for (a, b, c) in segs:
        if c == 0 or b - a < 5000:
            continue
        ss = a + int((b - a) * 0.45)
        P = cc['rActPressureRaw'][ss:b]
        fl = cc['rPIDControllerOutputFlow'][ss:b]
        nc = cc['rTargetSpeedRPM'][ss:b]
        Pm, Fm, Nm = mean(P), mean(fl), mean(nc)
        Rh = Pm / Fm if Fm > 1e-9 else float('nan')
        rows.append((c, Pm, Fm, Nm, Rh))
        log(f'  P_set={c:6.1f} bar  Q_leak={Fm:7.4f} L/min  n={Nm:7.3f} rpm  '
            f'R_h=P/Q={Rh:8.3f} bar/(L/min)  leak={Fm / Pm * 1000:7.4f} mL/min/bar')
    # C_h from step initial slope
    P = cc['rActPressureRaw']
    nc = cc['rTargetSpeedRPM']
    fl = cc['rPIDControllerOutputFlow']
    slopes = []
    for i in range(len(segs) - 1):
        a1, b1, c1 = segs[i + 1]
        if c1 == 0:
            continue
        w = 60
        y = P[a1:a1 + w]
        q = max(fl[a1:a1 + w])
        dp = mean(y[w // 2:]) - mean(P[max(0, a1 - 200):a1])
        rate = dp / (w / FS / 2)
        if rate > 50 and q > 0.5:
            ch = q / rate
            slopes.append(ch)
            log(f'  step->{c1:6.1f} bar : Q_peak={q:6.3f} L/min  dP/dt={rate:8.1f} bar/s  '
                f'=> C_h = Q/(dP/dt) = {ch * 1000:7.4f} mL/bar')
    Ch = mean(slopes) if slopes else float('nan')
    Rh = mean([r[4] for r in rows if r[4] == r[4]])
    tau = Rh * Ch
    log(f'  ==> R_h={Rh:.3f} bar/(L/min)   C_h={Ch * 1000:.4f} mL/bar   '
        f'tau=R_h*C_h={tau:.4f} s  ({tau * 1000:.1f} ms)')
    log(f'      V/beta_e = {Ch * 1000:.3f} mL/bar  (e.g. V=0.5 L & beta_e=900 bar -> '
        f'{0.5 / 900 * 1000:.3f} mL/bar)')

    # ---------------- 3. loop gain, crossover, controller reconstruction -------
    log()
    log('=' * 96)
    log('3. CONTROLLER RECONSTRUCTION FROM n_cmd / P AT RIPPLE FREQUENCIES')
    log('   incremental PID: C(w) = Kp + Ki/(j*w*Ts) + Kd*(j*w*Ts)   [ (L/min)/bar ]')
    log('   expected: |P/n_cmd| = 1/|C| * (1/20) [bar/rpm] if n_cmd = -C*P')
    KP, KI, KD = 0.25, 0.0065, 0.025
    meas = {}
    for (a, b, c) in segs:
        if c == 0 or b - a < 5000:
            continue
        ss = a + int((b - a) * 0.45)
        Praw = cc['rActPressureRaw'][ss:b]
        ncmd = cc['rTargetSpeedRPM'][ss:b]
        pos = cc['rActPumpPosition'][ss:b]
        ang = unwrap(pos)
        n = len(Praw)
        shaft = mean([ang[i + 1] - ang[i] for i in range(n - 1)]) * 60000.0 / 360.0
        wP, nr = resample_revs(ang, Praw, 4)
        wN, _ = resample_revs(ang, ncmd, 4)
        if wP is None or wN is None:
            continue
        oP = order_amp(wP, nr, ORDS)
        oN = order_amp(wN, nr, ORDS)
        log(f'--- P_set={c:g} bar  shaft={shaft:.3f} rpm  revs={nr} ---')
        log(f'  {"ord":>4s} {"f(Hz)":>8s} {"|P|":>8s} {"|n|":>8s} {"|n|/|P|":>9s} '
            f'{"rpm/bar":>8s} {"L/min/bar":>10s} {"pred":>8s} {"err%":>7s} {"ph(n-P)":>9s}')
        for o in ORDS:
            if o not in oP or o not in oN:
                continue
            ap, pp = oP[o]
            an, pn = oN[o]
            if ap < 0.02:
                continue
            f = o * shaft / 60.0
            w = 2 * math.pi * f
            wc = w * 0.001
            Cc = complex(KP, 0) + complex(0, -KI / wc) + complex(0, KD * wc)
            pred = abs(Cc)
            got = an / ap / GAIN_RPM_PER_FLOW
            meas[(c, o)] = (f, ap, an, got)
            dph = (pn - pp + 180) % 360 - 180
            log(f'  {o:4d} {f:8.3f} {ap:8.4f} {an:8.4f} {an / ap:9.4f} {an / ap:8.3f} '
                f'{got:10.4f} {pred:8.4f} {100 * (got / pred - 1):7.1f} {dph:9.1f}')
    fits = [v[3] for v in meas.values() if v[0] > 3]
    log(f'  mean measured |C| (f>3Hz) = {mean(fits):.4f} (L/min)/bar, sd={rms(fits):.4f}')
    log(f'  -> equivalent Kp ~ {mean(fits):.3f} (L/min)/bar = '
        f'{mean(fits) * GAIN_RPM_PER_FLOW:.3f} rpm/bar')

    Kp_fit = mean(fits)
    L0 = Kp_fit * Rh
    wc = L0 / tau
    log()
    log(f'  DC loop gain L0 = Kp*R_h = {L0:.2f}   (S(0)=1/(1+L0)={1 / (1 + L0):.4f})')
    log(f'  crossover w_c = L0/tau = {wc:.2f} rad/s -> f_c = {wc / (2 * math.pi):.2f} Hz')
    log(f'  13th order at 22.73 rpm = {13 * 22.7313 / 60:.2f} Hz ; '
        f'26th = {26 * 22.7313 / 60:.2f} Hz ; 52nd = {52 * 22.7313 / 60:.2f} Hz')
    log('  => the ripple band (5-35 Hz) sits AT or ABOVE crossover: |L|<=1, so S>=0.5')

    # ---------------- 4. sensitivity from open/closed order amplitudes ---------
    log()
    log('=' * 96)
    log('4. MEASURED SENSITIVITY S(f) AND IMPLIED LOOP DELAY')
    op = {}
    for fn, ccol in (('open-8-10-12_rpm.csv', 'rTargetSpeedRPM'),
                     ('open-15-20_rpmp.csv', 'rTargetSpeedRPM')):
        oc = load(fn)
        for (a, b, c) in segs_of(oc[ccol]):
            if c == 0 or b - a < 5000:
                continue
            ss = a + int((b - a) * 0.25)
            Praw = oc['rActPressureRaw'][ss:b]
            pos = oc['rActPumpPosition'][ss:b]
            ang = unwrap(pos)
            n = len(Praw)
            shaft = mean([ang[i + 1] - ang[i] for i in range(n - 1)]) * 60000.0 / 360.0
            wP, nr = resample_revs(ang, Praw, 8)
            if wP is None:
                continue
            oP = order_amp(wP, nr, ORDS)
            for o, (amp, ph) in oP.items():
                op.setdefault(o, []).append((shaft, amp, ph))
    cl = {}
    for (a, b, c) in segs:
        if c == 0 or b - a < 5000:
            continue
        ss = a + int((b - a) * 0.45)
        Praw = cc['rActPressureRaw'][ss:b]
        pos = cc['rActPumpPosition'][ss:b]
        ang = unwrap(pos)
        n = len(Praw)
        shaft = mean([ang[i + 1] - ang[i] for i in range(n - 1)]) * 60000.0 / 360.0
        wP, nr = resample_revs(ang, Praw, 4)
        if wP is None:
            continue
        oP = order_amp(wP, nr, ORDS)
        for o, (amp, ph) in oP.items():
            cl.setdefault(o, []).append((shaft, amp, ph, c))
    log(f'  {"P_set":>6s} {"ord":>4s} {"f(Hz)":>8s} {"A_open":>8s} {"A_close":>8s} '
        f'{"S":>7s} {"S_model":>8s} {"S_model+delay":>14s}')
    delays = []
    for o in sorted(cl):
        for (shaft, amp, ph, pset) in sorted(cl[o], key=lambda t: t[0]):
            pts = sorted(op.get(o, []))
            if len(pts) < 2:
                continue
            xs = [p[0] for p in pts]
            ys = [p[1] for p in pts]
            if shaft < xs[0] or shaft > xs[-1]:
                aop = None
            else:
                aop = None
                for j in range(len(xs) - 1):
                    if xs[j] <= shaft <= xs[j + 1]:
                        t = (shaft - xs[j]) / (xs[j + 1] - xs[j])
                        aop = ys[j] * (1 - t) + ys[j + 1] * t
                        break
            if not aop or aop < 0.05:
                continue
            f = o * shaft / 60.0
            w = 2 * math.pi * f
            S = amp / aop
            Gm = Rh / math.sqrt(1 + (w * tau) ** 2)
            Lm = Kp_fit * Gm
            Sm = 1.0 / math.sqrt(1 + 2 * Lm * math.cos(math.atan(w * tau)) + Lm ** 2)
            # find delay that reproduces measured S
            best = None
            for td_ms in range(0, 60):
                td = td_ms / 1000.0
                phL = -math.atan(w * tau) - w * td
                Sd = 1.0 / math.sqrt(1 + 2 * Lm * math.cos(phL) + Lm ** 2)
                err = abs(Sd - S)
                if best is None or err < best[1]:
                    best = (td_ms, err, Sd)
            delays.append((f, best[0], S, Sm))
            log(f'  {pset:6.1f} {o:4d} {f:8.3f} {aop:8.4f} {amp:8.4f} {S:7.3f} '
                f'{Sm:8.3f} {best[2]:8.3f} (td={best[0]} ms)')
    hi = [d for d in delays if d[0] > 4]
    if hi:
        log(f'  implied equivalent loop delay (f>4Hz) : mean={mean([d[1] for d in hi]):.1f} ms  '
            f'median={sorted([d[1] for d in hi])[len(hi) // 2]:.0f} ms')

    # ---------------- 5. improvement scenarios ----------------
    log()
    log('=' * 96)
    log('5. QUANTIFIED IMPROVEMENT SCENARIOS (140 bar case, shaft 22.73 rpm)')
    shaft140 = 22.7313
    a140 = {}
    for (a, b, c) in segs:
        if c == 140:
            ss = a + int((b - a) * 0.45)
            Praw = cc['rActPressureRaw'][ss:b]
            pos = cc['rActPumpPosition'][ss:b]
            ang = unwrap(pos)
            wP, nr = resample_revs(ang, Praw, 4)
            if wP:
                a140 = order_amp(wP, nr, list(range(1, 120)))
            tot_rms = rms(Praw)
            tot_p2p = max(Praw) - min(Praw)
    if a140:
        def tot(keep):
            s = 0.0
            for o, (amp, ph) in a140.items():
                if o in keep:
                    continue
                s += amp * amp / 2.0
            return math.sqrt(s)
        all_ord = set(a140.keys())
        base_det = tot(set())
        log(f'  measured total: rms={tot_rms:.4f} bar  p2p={tot_p2p:.4f} bar')
        log(f'  deterministic(1..119) rms={base_det:.4f} bar  '
            f'broadband residual rms={math.sqrt(max(tot_rms ** 2 - base_det ** 2, 0)):.4f} bar')
        for label, kill in (('cancel 13th only', {13}),
                            ('cancel 13+26', {13, 26}),
                            ('cancel 13+26+39+52', {13, 26, 39, 52}),
                            ('cancel 13,26,39,52,65,78', {13, 26, 39, 52, 65, 78}),
                            ('cancel all o>=13', {o for o in all_ord if o >= 13})):
            r = tot(kill)
            res = math.sqrt(max(r ** 2 + (tot_rms ** 2 - base_det ** 2), 0))
            log(f'  angle-sync FF {label:26s}: residual rms={res:.4f} bar  '
                f'-> est p2p={4.0 * res:.3f} bar  ({100 * (1 - res / tot_rms):.1f}% reduction)')
        # scenario: higher pump speed / smaller displacement
        log()
        log('  scenario: reduce displacement xN and raise speed xN (same mean flow):')
        log(f'  {"N":>4s} {"shaft rpm":>10s} {"f13":>8s} {"f26":>8s} {"|Z|@f13":>9s} '
            f'{"|Z|@f26":>9s} {"A13 rel":>8s} {"A26 rel":>8s} {"est p2p":>9s}')
        Z0_13 = Rh / math.sqrt(1 + (2 * math.pi * 13 * shaft140 / 60 * tau) ** 2)
        Z0_26 = Rh / math.sqrt(1 + (2 * math.pi * 26 * shaft140 / 60 * tau) ** 2)
        for N in (1, 2, 4, 8, 16, 32):
            sh = shaft140 * N
            f13, f26 = 13 * sh / 60, 26 * sh / 60
            w13, w26 = 2 * math.pi * f13, 2 * math.pi * f26
            Z13 = Rh / math.sqrt(1 + (w13 * tau) ** 2)
            Z26 = Rh / math.sqrt(1 + (w26 * tau) ** 2)
            # flow ripple amplitude unchanged (same mean flow, same relative ripple)
            r13, r26 = Z13 / Z0_13, Z26 / Z0_26
            a13n = a140.get(13, (0, 0))[0] * r13
            a26n = a140.get(26, (0, 0))[0] * r26
            a39n = a140.get(39, (0, 0))[0] * (Rh / math.sqrt(1 + (2 * math.pi * 39 * sh / 60 * tau) ** 2)) / \
                (Rh / math.sqrt(1 + (2 * math.pi * 39 * shaft140 / 60 * tau) ** 2))
            a52n = a140.get(52, (0, 0))[0] * (Rh / math.sqrt(1 + (2 * math.pi * 52 * sh / 60 * tau) ** 2)) / \
                (Rh / math.sqrt(1 + (2 * math.pi * 52 * shaft140 / 60 * tau) ** 2))
            est = math.sqrt(max(a13n ** 2 + a26n ** 2 + a39n ** 2 + a52n ** 2, 0) / 2 +
                            (tot_rms ** 2 - base_det ** 2) * 0.5)
            log(f'  {N:4d} {sh:10.1f} {f13:8.2f} {f26:8.2f} {Z13:9.3f} {Z26:9.3f} '
                f'{r13:8.3f} {r26:8.3f} {4.0 * est:9.3f}')
        # scenario: measurement LPF + Kp change
        log()
        log('  scenario: measurement low-pass alpha (1-pole IIR @1ms) + Kp scaling')
        log(f'  {"fc(Hz)":>7s} {"alpha":>7s} {"att@f13":>8s} {"att@f26":>8s} '
            f'{"att@f52":>8s} {"L0":>7s} {"fc_loop":>8s}')
        f13 = 13 * shaft140 / 60
        f26 = 26 * shaft140 / 60
        f52 = 52 * shaft140 / 60
        for fc in (5, 8, 12, 20, 30, 50, 100):
            al = 1.0 - math.exp(-2 * math.pi * fc / FS)
            def att(f):
                w = 2 * math.pi * f * 0.001
                return al / math.sqrt(1 - 2 * (1 - al) * math.cos(w) + (1 - al) ** 2)
            log(f'  {fc:7.1f} {al:7.4f} {att(f13):8.3f} {att(f26):8.3f} {att(f52):8.3f} '
                f'{Kp_fit * Rh:7.2f} {Kp_fit * Rh / tau / (2 * math.pi):8.2f}')

    # ---------------- 6. speed-command authority check ----------------
    log()
    log('=' * 96)
    log('6. ACTUATOR AUTHORITY / SATURATION CHECK (closed loop)')
    for (a, b, c) in segs:
        if c == 0 or b - a < 5000:
            continue
        ss = a + int((b - a) * 0.45)
        nc = cc['rTargetSpeedRPM'][ss:b]
        fl = cc['rPIDControllerOutputFlow'][ss:b]
        log(f'  P_set={c:6.1f} : n_cmd min={min(nc):8.3f} max={max(nc):8.3f} '
            f'mean={mean(nc):8.3f} p2p={max(nc) - min(nc):8.3f} rpm  '
            f'(p2p/mean={100 * (max(nc) - min(nc)) / mean(nc):6.1f}%)  '
            f'flow min={min(fl):7.4f} max={max(fl):7.4f} L/min  '
            f'negative-flow frac={100 * sum(1 for v in nc if v < 0) / len(nc):5.2f}%')


if __name__ == '__main__':
    main()
    with open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'analyze4_out.txt'), 'w',
              encoding='utf-8') as fh:
        fh.write('\n'.join(str(o) for o in OUT))
