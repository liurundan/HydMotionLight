"""Angle-synchronous order demodulation + plant identification.

Strategy: resample each signal onto an exact integer number of shaft revolutions
(512 points/rev), then FFT. Integer orders land exactly on bins -> no leakage.
Also: open-loop trajectory (drift/equilibrium), step initial slope (beta_e/V),
50 Hz interference, residual broadband floor.
"""
import csv
import math
import os
import sys

DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'docs', 'data_deg')
FS = 1000.0
PTS_PER_REV = 512
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


def resample_revs(ang, sig, nrev, pts=PTS_PER_REV):
    """Resample over the LAST nrev complete revolutions.

    nrev is snapped down to a power of two so that N = nrev*pts is a power of
    two and every integer order lands exactly on an FFT bin (no leakage).
    """
    span = ang[-1] - ang[0]
    avail = int(span // 360)
    if avail < 1:
        return None, 0
    # largest power of two <= min(nrev, avail), at least 1
    cap = min(nrev, avail)
    p = 1
    while p * 2 <= cap:
        p *= 2
    nrev = p
    a_start = ang[-1] - nrev * 360.0
    N = nrev * pts
    out = []
    i = 0
    n = len(ang)
    while i < n - 1 and ang[i + 1] < a_start:
        i += 1
    for j in range(N):
        t = a_start + (j + 0.5) * 360.0 / pts
        while i < n - 1 and ang[i + 1] < t:
            i += 1
        if i >= n - 1:
            i = n - 2
        a1, a2 = ang[i], ang[i + 1]
        w = (t - a1) / (a2 - a1) if a2 > a1 else 0.0
        w = max(0.0, min(1.0, w))
        out.append(sig[i] * (1 - w) + sig[i + 1] * w)
    return out, nrev


def order_fft(wave, nrev):
    """Return {order: (amp, phase_deg)} for integer orders."""
    N = len(wave)
    m = mean(wave)
    re = [v - m for v in wave]
    im = [0.0] * N
    fft(re, im)
    res = {}
    for k in range(1, N // 2):
        order = k / float(nrev)
        if abs(order - round(order)) > 1e-9:
            continue
        o = int(round(order))
        amp = 2.0 * math.hypot(re[k], im[k]) / N
        ph = math.degrees(math.atan2(-im[k], re[k]))
        res[o] = (amp, ph)
    return res, m


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


ORDERS = [1, 2, 3, 13, 26, 39, 52, 65, 78]
RESULTS = {}


def analyse(tag, cols, a, b, want_rev=8):
    sl = slice(a, b)
    P = cols['rActPressureRaw'][sl]
    pos = cols['rActPumpPosition'][sl]
    nc = cols['rTargetSpeedRPM'][sl]
    tq = cols['rActTorquePercent'][sl]
    fl = cols['rPIDControllerOutputFlow'][sl]
    n = len(P)
    ang = unwrap(pos)
    d = [(ang[i + 1] - ang[i]) for i in range(n - 1)]
    shaft = mean(d) * 60000.0 / 360.0
    log()
    log('=' * 96)
    log(f'{tag}   idx[{a}:{b}] n={n} ({n / FS:.2f}s)  shaft={shaft:.4f} rpm  '
        f'f_rev={shaft / 60:.4f} Hz  revs_avail={(n / FS) * shaft / 60:.2f}')
    log(f'P mean={mean(P):.4f} sd={rms(P):.4f} p2p={max(P) - min(P):.4f} bar')
    log(f'n_cmd mean={mean(nc):.4f} sd={rms(nc):.4f} p2p={max(nc) - min(nc):.4f} rpm')
    log(f'torque mean={mean(tq):.4f} sd={rms(tq):.4f} p2p={max(tq) - min(tq):.4f} %')

    sigs = {'P': P, 'n_cmd': nc, 'torque': tq, 'flow': fl}
    od = {}
    nrev_used = None
    for name, s in sigs.items():
        w, nrev = resample_revs(ang, s, want_rev)
        if w is None:
            log(f'  {name}: <1 rev, skip order analysis')
            continue
        nrev_used = nrev
        spec, m = order_fft(w, nrev)
        od[name] = spec
        tot = rms(w)
        orms = math.sqrt(sum(a * a for o, (a, p) in spec.items() if o <= 80) / 2.0)
        log(f'  {name:6s} revs={nrev} mean={m:9.4f} rms={tot:8.4f} '
            f'order_rms(1..80)={orms:8.4f} -> deterministic share='
            f'{100 * (orms / tot) ** 2 if tot else 0:6.2f}%')

    if nrev_used and 'P' in od:
        log(f'  --- order table (amp, phase_deg) over {nrev_used} revs ---')
        hdr = f'  {"order":>6s} {"f(Hz)":>8s} ' + ' '.join(f'{k:>22s}' for k in sigs)
        log(hdr)
        allord = sorted(set(od['P'].keys()) | set([13, 26, 39, 52]))
        show = [o for o in ORDERS if o in allord]
        top = sorted(od['P'].items(), key=lambda t: -t[1][0])[:8]
        for o, _ in top:
            if o not in show:
                show.append(o)
        show = sorted(set(show))
        for o in show:
            f = o * shaft / 60.0
            row = f'  {o:6d} {f:8.3f} '
            for k in sigs:
                if k in od and o in od[k]:
                    a_, p_ = od[k][o]
                    row += f'{a_:10.4f}@{p_:8.1f}  '
                else:
                    row += f'{"-":>22s}'
            log(row)
        # phase relation n_cmd vs P at key orders
        log('  --- loop phase relation (n_cmd relative to P) ---')
        for o in (13, 26, 39, 52):
            if 'n_cmd' in od and o in od['n_cmd'] and o in od['P']:
                ap, pp = od['P'][o]
                an, pn = od['n_cmd'][o]
                dp = (pn - pp + 180) % 360 - 180
                f = o * shaft / 60.0
                log(f'   order {o:3d} @ {f:6.3f} Hz : P amp={ap:.4f} bar  n_cmd amp={an:.4f} rpm  '
                    f'|P/n|={ap / an if an else float("inf"):.4f} bar/rpm  phase(n-P)={dp:8.2f} deg  '
                    f'-> delay equiv {dp / 360 / f * 1000 if f else 0:7.2f} ms')
            if 'torque' in od and o in od['torque'] and o in od['P']:
                at, pt = od['torque'][o]
                ap, pp = od['P'][o]
                dp = (pp - pt + 180) % 360 - 180
                log(f'                torque amp={at:.4f} %  phase(P-T)={dp:8.2f} deg  '
                    f'|T/P|={at / ap if ap else float("inf"):.4f} %/bar')
    RESULTS[tag] = {'shaft': shaft, 'P_mean': mean(P), 'P_sd': rms(P),
                    'P_p2p': max(P) - min(P), 'n_mean': mean(nc), 'n_sd': rms(nc),
                    'n_p2p': max(nc) - min(nc), 'flow_mean': mean(fl), 'orders': od,
                    'nrev': nrev_used}
    return RESULTS[tag]


def main():
    files = [('open-8-10-12_rpm.csv', 'rTargetSpeedRPM', 'OPEN'),
             ('open-15-20_rpmp.csv', 'rTargetSpeedRPM', 'OPEN'),
             ('close-50-80-100-120-140_bar_deg.csv', 'rTargetPressure', 'CLOSE')]
    for fn, cc, kind in files:
        cols = load(fn)
        cmd = cols[cc]
        for (a, b, c) in segs_of(cmd):
            if c == 0 or b - a < 4500:
                continue
            ss = a + int((b - a) * (0.20 if kind == 'OPEN' else 0.40))
            analyse(f'{kind} {fn} cmd={c:g}', cols, ss, b)

    # ---- open-loop trajectory: 1 s averages ----
    log()
    log('=' * 96)
    log('OPEN-LOOP 1-s AVERAGED TRAJECTORY (equilibrium / drift check)')
    for fn, cc in (('open-8-10-12_rpm.csv', 'rTargetSpeedRPM'),
                   ('open-15-20_rpmp.csv', 'rTargetSpeedRPM')):
        cols = load(fn)
        cmd = cols[cc]
        P = cols['rActPressureRaw']
        for (a, b, c) in segs_of(cmd):
            if c == 0:
                continue
            log(f'  {fn} cmd={c:g} rpm:')
            for s in range(a, b, 1000):
                e = min(s + 1000, b)
                if e - s < 200:
                    break
                log(f'    t={(s - a) / FS:5.1f}-{(e - a) / FS:5.1f}s  P={mean(P[s:e]):8.3f} '
                    f'+-{rms(P[s:e]):6.3f}')

    # ---- closed-loop step initial slope -> beta_e/V ----
    cols = load('close-50-80-100-120-140_bar_deg.csv')
    cmd = cols['rTargetPressure']
    P = cols['rActPressureRaw']
    nc = cols['rTargetSpeedRPM']
    fl = cols['rPIDControllerOutputFlow']
    log()
    log('=' * 96)
    log('STEP INITIAL SLOPE -> beta_e/V (plant integrator gain)')
    sg = segs_of(cmd)
    for i in range(len(sg) - 1):
        a1, b1, c1 = sg[i + 1]
        if c1 == 0:
            continue
        base = mean(P[max(0, a1 - 300):a1])
        for w in (10, 20, 50, 100):
            y = P[a1:a1 + w]
            if len(y) < w:
                continue
            slope = (mean(y[w // 2:]) - base) / (w / FS / 2)  # bar/s approx
            log(f'  -> {c1:6.1f} bar: first {w:3d} ms  dP/dt~{slope:9.1f} bar/s  '
                f'n_cmd max={max(nc[a1:a1 + w]):7.2f} flow max={max(fl[a1:a1 + w]):7.3f} L/min  '
                f'P_end={y[-1]:7.2f}')
        log('')

    # ---- comparison open vs closed at matched shaft speed ----
    log('=' * 96)
    log('OPEN vs CLOSED LOOP ORDER AMPLITUDES AT MATCHED SHAFT SPEED')
    op = {k: v for k, v in RESULTS.items() if k.startswith('OPEN') and v.get('orders', {}).get('P')}
    cl = {k: v for k, v in RESULTS.items() if k.startswith('CLOSE') and v.get('orders', {}).get('P')}
    okeys = sorted(op.keys(), key=lambda k: op[k]['shaft'])
    for ck, cv in sorted(cl.items(), key=lambda t: t[1]['shaft']):
        cs = cv['shaft']
        # linear interpolation of open-loop amplitudes vs shaft speed
        interp = {}
        for o in (13, 26, 39, 52):
            pts = [(op[k]['shaft'], op[k]['orders']['P'].get(o, (0, 0))[0]) for k in okeys]
            val = None
            for j in range(len(pts) - 1):
                x0, y0 = pts[j]
                x1, y1 = pts[j + 1]
                if x0 <= cs <= x1:
                    val = y0 + (y1 - y0) * (cs - x0) / (x1 - x0) if x1 > x0 else y0
                    break
            interp[o] = val
        cop = cv['orders']['P']
        log(f'  {ck}  shaft={cs:.3f} rpm  (13th={13 * cs / 60:.3f} Hz, 26th={26 * cs / 60:.3f} Hz)')
        for o in (13, 26, 39, 52):
            a_cl = cop.get(o, (0, 0))[0]
            a_op = interp[o]
            if a_op and a_op > 1e-9:
                log(f'     order {o:3d}: open(interp)={a_op:7.4f} bar  closed={a_cl:7.4f} bar  '
                    f'sensitivity S={a_cl / a_op:6.3f}  reduction={a_op / a_cl:5.2f}x')
            else:
                log(f'     order {o:3d}: open(interp)= n/a (outside range)  closed={a_cl:7.4f} bar')

    # ---- scaling laws ----
    log()
    log('=' * 96)
    log('SCALING LAWS')
    log('OPEN LOOP order amplitude vs shaft speed / pressure:')
    log(f'  {"n(rpm)":>8s} {"P(bar)":>8s} {"A13":>8s} {"A26":>8s} {"A39":>8s} {"A52":>8s} '
        f'{"A13/n":>8s} {"A26/n":>8s}')
    for k in okeys:
        v = op[k]
        g = lambda o: v['orders']['P'].get(o, (0, 0))[0]
        log(f'  {v["shaft"]:8.3f} {v["P_mean"]:8.2f} {g(13):8.4f} {g(26):8.4f} {g(39):8.4f} '
            f'{g(52):8.4f} {g(13) / v["shaft"]:8.4f} {g(26) / v["shaft"]:8.4f}')
    log('CLOSED LOOP:')
    log(f'  {"P_set":>8s} {"n(rpm)":>8s} {"P_sd":>8s} {"P_p2p":>8s} {"n_sd":>8s} {"n_p2p":>8s} '
        f'{"A13":>8s} {"A26":>8s} {"A52":>8s} {"flow":>8s}')
    for k, v in sorted(cl.items(), key=lambda t: t[1]['P_mean']):
        g = lambda o: v['orders']['P'].get(o, (0, 0))[0]
        log(f'  {v["P_mean"]:8.2f} {v["shaft"]:8.3f} {v["P_sd"]:8.4f} {v["P_p2p"]:8.3f} '
            f'{v["n_sd"]:8.4f} {v["n_p2p"]:8.3f} {g(13):8.4f} {g(26):8.4f} {g(52):8.4f} '
            f'{v["flow_mean"]:8.4f}')
    log()
    log('n_cmd jitter vs mean (closed loop):')
    for k, v in sorted(cl.items(), key=lambda t: t[1]['P_mean']):
        log(f'  P={v["P_mean"]:7.2f} bar  n_mean={v["n_mean"]:7.3f}  n_sd={v["n_sd"]:6.3f}  '
            f'n_p2p={v["n_p2p"]:7.3f}  p2p/mean={100 * v["n_p2p"] / v["n_mean"] if v["n_mean"] else 0:6.1f}%  '
            f'flow_sd={v.get("n_sd", 0) / 20.0:.4f} L/min')


if __name__ == '__main__':
    main()
    with open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'analyze3_out.txt'), 'w',
              encoding='utf-8') as fh:
        fh.write('\n'.join(str(o) for o in OUT))
