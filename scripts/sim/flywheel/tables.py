"""Compact text tables from results.json -> results_tables.txt"""
import os, sys, json, math
from collections import defaultdict
HERE = os.path.dirname(os.path.abspath(__file__))
R = json.load(open(os.path.join(HERE, 'results.json')))
out = []
P = lambda s='': out.append(s)
CT = ['oracle', 'ffpi', 'ffpid', 'ffpiz', 'ffp', 'bb', 'bbff', 'hybrid', 'boost', 'pred', 'kf', 'hub']
REAL = [c for c in CT if c != 'oracle']
JS = [5e-5, 1e-4, 2e-4, 5e-4]


def f(x, d=0, w=6):
    if x is None or (isinstance(x, float) and not math.isfinite(x)):
        return 'nan'.rjust(w)
    return f'{x:{w}.{d}f}'


def fs(d, k, dd=0, w=6, nanfrac=True):
    x = d.get(k)
    s = f(x, dd, w)
    if nanfrac and d.get(k + '_nanfrac', 0) > 0:
        s = (s.strip() + f'*{int(round(d[k+"_nanfrac"]*8))}').rjust(w + 2)
    else:
        s = s.rjust(w + 2)
    return s


def find(sec, **kw):
    for d in R[sec]:
        ok = True
        for k, v in kw.items():
            if k == 'variant':
                if any(abs(d['variant'].get(a, -9) - b) > 1e-12 for a, b in v.items()):
                    ok = False
            elif d.get(k) != v:
                ok = False
        if ok:
            return d
    return None


P('TUNED GAINS (baseline: 12.5 V, Rint 0.10, friction mid, 30 g ball, 20 ms contact, 15+-2 ms loop, 50 ms window, ts +-0.5 ms)')
P('units: kP V/rpm, kI V/(rpm s), kD V/(rpm/s), thr rpm, B V, lead/dur s; hub kP,kI internal(1/32767)/tps (kI per s)')
P('cost = shot_err/50rpm + t_spin50/0.5s + rec50/0.25s + overshoot/200rpm; averaged over FF error {-5,0,+5}% x 4 seeds')
for J in JS:
    P(f'  J={J:.0e}:')
    for d in sorted([d for d in R['tune_base'] if d['J'] == J], key=lambda d: d['cost']):
        P(f"    {d['ctrl']:7s} cost {d['cost']:6.2f}  {d['gains']}" + (f"  (grid edge: {','.join(d['edge'])})" if d['edge'] else ''))
P()

P('=== CORE (perfect FF, 12.5 V/0.10 ohm, 15 ms loop, 50 ms window, ts +-0.5 ms; mean of 8 seeds; "*n" = n of 8 seeds never settled/recovered) ===')
for J in JS:
    P(f'--- J = {J:.0e} kg m^2 ---')
    P('ctrl    | spin-up (fric mid)                         | single shot 30 g, 15 ms contact      | burst 30 g/15 ms: |e| at touch b2,b3 (rpm)          | exit spread % (250) | meas-true at touch b2,3 |')
    P('        | t50set  tfirst  ovsht  ssStd  measStd Irms | dip    rec25   rec50   rec100        | 150ms      250ms      400ms                        |                     | 250 ms                  |')
    for c in CT:
        sp = find('core', ctrl=c, J=J, kind='spin', variant=dict(fric=0.008))
        si = find('core', ctrl=c, J=J, kind='single', variant=dict(m=0.03, Tc=0.015))
        b = {s: find('core', ctrl=c, J=J, kind='burst', spacing=s, variant=dict(m=0.03, Tc=0.015)) for s in (0.15, 0.25, 0.4)}
        if not sp:
            continue
        line = (f"{c:7s} |{fs(sp,'t_settle50',3)}{f(sp['t_first50'],3,7)}{f(sp['overshoot'],0,6)}{f(sp['ss_std'],1,6)}{f(sp['ssm_std'],1,7)}{f(sp['I_rms'],2,5)} |"
                f"{f(si['dip'],0,6)}{fs(si,'rec25',3)}{fs(si,'rec50',3)}{fs(si,'rec100',3)} |")
        for s in (0.15, 0.25, 0.4):
            line += f"{f(b[s]['abs_e2'],0,5)},{f(b[s]['abs_e3'],0,4)}  "
        line += f"| {f(b[0.25]['exit_spread_pct'],2,5)}  Irms {f(b[0.25]['I_rms'],2,4)} | {f(b[0.25]['abs_meas_err_touch'],0,5)}"
        P(line)
    P()

P('=== BALL MASS x CONTACT x SPACING: mean(|e2|,|e3|) rpm at touch, per J (rows ctrl, cols m/Tc/spacing) ===')
mvars = [(m, tc) for m in (0.01, 0.03, 0.06) for tc in (0.015, 0.03)]
for J in JS:
    P(f'--- J = {J:.0e} ---   cols: ' + '  '.join(f'{int(m*1000)}g/{int(tc*1000)}ms' for m, tc in mvars) + '   (each col: 150/250/400 ms)')
    for c in CT:
        line = f'{c:7s}'
        for m, tc in mvars:
            vals = []
            for s in (0.15, 0.25, 0.4):
                d = find('core', ctrl=c, J=J, kind='burst', spacing=s, variant=dict(m=m, Tc=tc))
                vals.append(0.5 * (d['abs_e2'] + d['abs_e3']))
            line += ' |' + '/'.join(f'{v:.0f}' for v in vals)
        P(line)
P()

P('=== WINNERS: lowest mean(|e2|,|e3|) among real controllers, per scenario (J x m x Tc x spacing = 72 scenarios) ===')
wins = defaultdict(int)
rank_sum = defaultdict(float)
nsc = 0
for J in JS:
    for m, tc in mvars:
        for s in (0.15, 0.25, 0.4):
            vals = {}
            for c in REAL:
                d = find('core', ctrl=c, J=J, kind='burst', spacing=s, variant=dict(m=m, Tc=tc))
                vals[c] = 0.5 * (d['abs_e2'] + d['abs_e3'])
            order = sorted(vals, key=vals.get)
            wins[order[0]] += 1
            for i, c in enumerate(order):
                rank_sum[c] += i + 1
            nsc += 1
P('wins: ' + ', '.join(f'{c} {wins[c]}' for c in sorted(wins, key=lambda c: -wins[c])))
P('mean rank (1=best of 11): ' + ', '.join(f'{c} {rank_sum[c]/nsc:.1f}' for c in sorted(REAL, key=lambda c: rank_sum[c])))
for J in JS:
    ws = defaultdict(int)
    for m, tc in mvars:
        for s in (0.15, 0.25, 0.4):
            vals = {c: 0.5 * (find('core', ctrl=c, J=J, kind='burst', spacing=s, variant=dict(m=m, Tc=tc))['abs_e2'] +
                              find('core', ctrl=c, J=J, kind='burst', spacing=s, variant=dict(m=m, Tc=tc))['abs_e3']) for c in REAL}
            ws[min(vals, key=vals.get)] += 1
    P(f'  J={J:.0e} wins: ' + ', '.join(f'{c} {ws[c]}' for c in sorted(ws, key=lambda c: -ws[c])))
P()

P('=== SINGLE-SHOT RECOVERY to 50 rpm (s), by ball mass/contact; oracle = physics floor ===')
for J in JS:
    P(f'--- J = {J:.0e} ---  cols: ' + '  '.join(f'{int(m*1000)}g/{int(tc*1000)}ms' for m, tc in mvars))
    for c in CT:
        line = f'{c:7s}'
        for m, tc in mvars:
            d = find('core', ctrl=c, J=J, kind='single', variant=dict(m=m, Tc=tc))
            line += fs(d, 'rec50', 3, 7)
        P(line)
P()

P('=== FRICTION LEVEL (spin-up t_settle50 s / ss bias rpm / ss std rpm) ===')
for J in (1e-4, 5e-4):
    P(f'--- J = {J:.0e} --- cols: fric low(0.004) mid(0.008) high(0.016) N.m at 3000 rpm')
    for c in CT:
        line = f'{c:7s}'
        for fr in (0.004, 0.008, 0.016):
            d = find('core', ctrl=c, J=J, kind='spin', variant=dict(fric=fr))
            line += f" | {fs(d,'t_settle50',3).strip():>7s} {f(d['ss_bias'],0,4)} {f(d['ss_std'],1,5)}"
        P(line)
P()

if 'robust' in R:
    P('=== ROBUSTNESS (gains from baseline tuning). spin: t_settle50 s / ss_bias rpm; burst 250 ms 30 g: mean(|e2|,|e3|) rpm ===')
    names = ['base', 'ff-5', 'ff+5', 'ff-10', 'ff+10', 'bat12.0_R0.20', 'bat12.0_R0.05', 'bat13.5_R0.05', 'bat13.5_R0.20',
             'pwm_coast', 'kA-20', 'kA+20', 'sag25A']
    for J in JS:
        P(f'--- J = {J:.0e} ---  cols: ' + ' | '.join(names))
        for c in CT:
            line = f'{c:7s}'
            for nm in names:
                if nm == 'base':
                    sp = find('core', ctrl=c, J=J, kind='spin', variant=dict(fric=0.008))
                    bu = find('core', ctrl=c, J=J, kind='burst', spacing=0.25, variant=dict(m=0.03, Tc=0.015))
                    # core burst uses 15 ms contact; robust uses BASE 20 ms contact
                else:
                    sp = find('robust', ctrl=c, J=J, kind='spin', tag=nm)
                    bu = find('robust', ctrl=c, J=J, kind='burst', tag=nm)
                if nm == 'sag25A':
                    line += f" | e1 {f(bu['e1'],0,4)} e23 {f(0.5*(bu['abs_e2']+bu['abs_e3']),0,4)}"
                else:
                    ts = sp['t_settle50']
                    tss = 'ns' if (sp.get('t_settle50_nanfrac', 0) > 0) else f'{ts:.2f}'
                    line += f" | {tss:>4s} {f(sp['ss_bias'],0,4)} {f(0.5*(bu['abs_e2']+bu['abs_e3']),0,4)}"
            P(line)
    P('(ns = not settled within 50 rpm by end of spin-up run in >=1 of 8 seeds; base burst column uses 15 ms contact, others 20 ms)')
    P()
    P('=== CURRENT (A rms): spin-up steady-state window / burst window (-50 ms..+1 s around ball 1), J=1e-4 and 5e-4; I_max in burst ===')
    for c in CT:
        line = f'{c:7s}'
        for J in (1e-4, 5e-4):
            sp = find('core', ctrl=c, J=J, kind='spin', variant=dict(fric=0.008))
            bu = find('core', ctrl=c, J=J, kind='burst', spacing=0.25, variant=dict(m=0.03, Tc=0.015))
            line += f" | J={J:.0e}: ss {f(sp['I_rms'],2,5)}  burst {f(bu['I_rms'],2,5)}  max {f(bu['I_max'],1,4)}"
        P(line)
    P()

if 'hubdefault' in R:
    P('=== HUB PIDF with REV-guide gains (F=32767/maxV, P=0.1F, I=0.1P per loop -> x100/s) ===')
    for J in JS:
        sp = find('hubdefault', J=J, kind='spin')
        bu = find('hubdefault', J=J, kind='burst')
        P(f"  J={J:.0e}: spin t50set {fs(sp,'t_settle50',3).strip()} ovsht {f(sp['overshoot'],0)} ss_bias {f(sp['ss_bias'],0)} ss_std {f(sp['ss_std'],1)} | "
          f"burst250 |e2|,|e3| {f(bu['abs_e2'],0)},{f(bu['abs_e3'],0)} rec50 {fs(bu,'rec50',3).strip()}")
    P()

if 'timing' in R:
    P('=== TIMING SWEEP (J=1e-4 and 5e-4). Gains RETUNED per (loop T, window W) at ts +-0.5 ms; then evaluated at ts 0 / 0.5 / 1.5 ms ===')
    P('cell = t_settle50(s) | ss_std(rpm) | mean(|e2|,|e3|) burst 250 ms 30 g 20 ms (rpm) | rec50 (s) | |meas-true| at touch b2,b3 (rpm)')
    for J in (1e-4, 5e-4):
        for c in [c for c in CT if c not in ('oracle', 'hub')]:
            P(f'--- {c} J={J:.0e} ---')
            for tsA in (0.0, 0.5e-3, 1.5e-3):
                P(f'   ts +-{tsA*1e3:.1f} ms:')
                for T in (0.008, 0.015, 0.025):
                    line = f'     T={int(T*1000):2d}ms'
                    for W in (0.015, 0.03, 0.05):
                        tag = f'T{T}_W{W}'
                        sp = find('timing', ctrl=c, J=J, kind='spin', tag=tag, variant=dict(tsA=tsA))
                        bu = find('timing', ctrl=c, J=J, kind='burst', tag=tag, variant=dict(tsA=tsA))
                        if not sp:
                            continue
                        tss = 'ns' if sp.get('t_settle50_nanfrac', 0) > 0 else f"{sp['t_settle50']:.2f}"
                        rr = 'nr' if bu.get('rec50_nanfrac', 0) > 0 else f"{bu['rec50']:.3f}"
                        line += f" | W{int(W*1000)}: {tss:>4s} {sp['ss_std']:5.1f} {0.5*(bu['abs_e2']+bu['abs_e3']):5.0f} {rr:>5s} {bu['abs_meas_err_touch']:4.0f}"
                    P(line)
        P()
    P('retuned gains per (T,W):')
    for d in sorted(R['tune_timing'], key=lambda d: (d['J'], d['ctrl'], d['tag'])):
        P(f"  J={d['J']:.0e} {d['ctrl']:6s} {d['tag']:14s} cost {d['cost']:5.2f} {d['gains']}" + (f" edge:{','.join(d['edge'])}" if d['edge'] else ''))

open(os.path.join(HERE, 'results_tables.txt'), 'w').write('\n'.join(out) + '\n')
print('\n'.join(out))
