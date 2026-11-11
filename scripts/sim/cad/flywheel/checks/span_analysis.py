"""Re-scores the stored burst results (../results/summary.json, read only) under four pass rules.

Every rule needs every ball of every seed to pass, and every longer interval to pass too.
  anc        exit deviations from ball 1 (the nominal) inside the reference window (2.8 in ball, 14 in exit)
  anc_own    the same inside each shooter's own window (CAD exit point, own ball size, hood stop)
  bias       one constant setpoint bias for all balls and seeds is allowed: pass if max - min <= window width (own)
  bias_mass  bias, with the ball-mass spread (light - heavy, BALL-MASS SPREAD at the nearest distance) added to the span
Usage: python3 -I span_analysis.py [summary.json path] [output name]
"""
import json, os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import numpy as np
import ballistics as B

SRC = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, '..', 'results', 'summary.json')
S = json.load(open(SRC))
A = dict(S['burst']); A.update(S.get('changes', {}))
M = S['mass']
DS = [36, 60, 84, 108, 132]
WV = B.REF_WIN
WO = {sh: {d: (B.shot(sh, d)['win_lo_pct'], B.shot(sh, d)['win_hi_pct']) for d in DS} for sh in ('pollen', 'nectar')}
REAL = [('team', 'hub', 0.02), ('team', 'oq', 0.02), ('rec', 'hub', 0.02), ('rec', 'oq', 0.02), ('rect', 'hub', 0.02), ('rect', 'oq', 0.02)]
CH_REAL = [('team', 'hub', 0.02), ('rec', 'oq', 0.02), ('rect', 'oq', 0.02), ('rect', 'hub', 0.02)]
MASS_PK = {'2mot_steel82': 'steel82', 'g1.25_steel82': 'steel82', 'g1.5_steel82': 'steel82', 'g0.8': 'asbuilt'}


def mass(sh, pk, d, f):
    dd = min((36, 84, 132), key=lambda x: abs(x - d))
    m = M.get(f'{sh}|{MASS_PK.get(pk, pk)}|{dd}|{f}')
    return (m['lo'], m['hi']) if m else (0.0, 0.0)   # lo = light-ball rise (+), hi = heavy-ball drop (-)


def passes(rec, sp, rule, sh, pk, d, f, nb):
    mn = np.array(rec['per_ball_min'][sp][:nb]); mx = np.array(rec['per_ball_max'][sp][:nb])
    if not (np.all(np.isfinite(mn)) and np.all(np.isfinite(mx))):
        return False
    if rule == 'anc':
        lo, hi = WV[d]; return mn.min() >= lo and mx.max() <= hi
    lo, hi = WO[sh][d]
    if rule == 'anc_own':
        return mn.min() >= lo and mx.max() <= hi
    span = mx.max() - mn.min()
    if rule == 'bias_mass':
        ml, mh = mass(sh, pk, d, f)
        span += ml - mh
    return span <= hi - lo


def fastest(sh, pk, d, f, drag, cm, rule, nb=10):
    rec = A.get('|'.join(str(x) for x in (sh, pk, d, f, drag, *cm)))
    if rec is None:
        return 'n/a'
    if rec.get('infeasible'):
        return 'inf'
    best = None
    for sp in sorted(rec['per_ball_min'], key=float, reverse=True):
        if passes(rec, sp, rule, sh, pk, d, f, nb):
            best = sp
        else:
            break
    return '>300' if best is None else str(int(round(float(best) * 1000)))


def num(x):
    return {'>300': 999, 'inf': 9999, 'n/a': 99999}.get(x, None) or int(x)


def best_real(sh, pk, d, f, drag, rule, nb=10):
    cms = REAL if ('|'.join((sh, pk, '36', '0.45', '1.0', 'team', 'oq', '0.02')) in A) else CH_REAL
    vals = [(num(fastest(sh, pk, d, f, drag, cm, rule, nb)), fastest(sh, pk, d, f, drag, cm, rule, nb), cm) for cm in cms]
    v = min(vals, key=lambda t: t[0])
    return v[1]


L = []
P = L.append
out = {}
P('Re-scored fastest 10-ball interval (ms). Cells per distance 36/60/84/108/132: ideal | team/hub | best real loop')
for rule in ('anc', 'anc_own', 'bias', 'bias_mass'):
    P(f'\n== pass rule {rule}')
    for f in (0.35, 0.45, 0.5):
        for drag in (1.0, 1.14):
            for sh in ('pollen', 'nectar'):
                for pk in ('asbuilt', 'steel82', 'J2.4', 'J4.0', '2mot', '2mot_steel82', 'g1.25', 'g1.5', 'w96'):
                    i = [fastest(sh, pk, d, f, drag, ('oracle', 'none', 0.02), rule) for d in DS]
                    t = [fastest(sh, pk, d, f, drag, ('team', 'hub', 0.02), rule) for d in DS]
                    b = [best_real(sh, pk, d, f, drag, rule) for d in DS]
                    out[f'{rule}|{sh}|{pk}|{f}|{drag}'] = dict(ideal=i, team_hub=t, best_real=b)
                    if f == 0.45 or pk == 'asbuilt':
                        P(f'f={f:.2f} x{drag:.2f} {sh:6s} {pk:13s} | ideal {"/".join(i):24s} | team/hub {"/".join(t):24s} | best real {"/".join(b)}')

P('\n== 3/5/10-ball fastest interval, as-built, f .45, drag x1.00, best real loop, rules anc and bias')
for rule in ('anc', 'bias'):
    for sh in ('pollen', 'nectar'):
        cells = ['/'.join(best_real(sh, 'asbuilt', d, 0.45, 1.0, rule, nb) for nb in (3, 5, 10)) for d in DS]
        P(f'{rule} {sh:6s} ' + ' | '.join(f'{d}: {c}' for d, c in zip(DS, cells)))

INT = [92, 105, 120, 135, 150, 175, 200, 250, 300]
step = lambda v: {'>300': 9, 'inf': 10}.get(v) if v in ('>300', 'inf') else INT.index(int(v))
diff = tot = 0
dist = {}
for k, rec in S['burst'].items():
    if rec.get('infeasible'):
        continue
    sh, pk, d, f, drag, c, m, q = k.split('|')
    a3 = fastest(sh, pk, int(d), float(f), float(drag), (c, m, float(q)), 'anc', 3)
    a10 = fastest(sh, pk, int(d), float(f), float(drag), (c, m, float(q)), 'anc', 10)
    tot += 1; diff += (a3 != a10)
    s = step(a10) - step(a3)
    dist[s] = dist.get(s, 0) + 1
P(f'\nrule anc, base plants, all controllers/f/drag: 3-ball fastest interval differs from 10-ball in {diff}/{tot} cells')
P(f'steps(10-ball) - steps(3-ball): {dict(sorted(dist.items()))}')

P('\n== rule bias by loop and measurement, f .45, drag x1.00; cells 36/60/84/108/132')
for sh in ('pollen', 'nectar'):
    for pk in ('asbuilt', 'J1.6', 'J2.4', 'J4.0'):
        for cm in REAL:
            P(f'{sh:6s} {pk:8s} {cm[0]}/{cm[1]:4s} ' + '/'.join(fastest(sh, pk, d, 0.45, 1.0, cm, 'bias') for d in DS))

P('\n== required shift (setpoint bias, %) that rule bias uses at 92 ms, as-built f .45 x1.00 team/hub and best case')
for sh in ('pollen', 'nectar'):
    for d in DS:
        rec = A.get(f'{sh}|asbuilt|{d}|0.45|1.0|team|hub|0.02')
        if rec is None or rec.get('infeasible'):
            continue
        mn = min(rec['per_ball_min']['0.092']); mx = max(rec['per_ball_max']['0.092'])
        lo, hi = WO[sh][d]
        P(f'{sh} {d}: team/hub 92 ms span {mn:+.2f}..{mx:+.2f} = {mx-mn:.2f} % vs own window width {hi-lo:.2f} % '
          f'-> {"fits" if mx-mn <= hi-lo else "does not fit"}; centring bias {(lo+hi)/2-(mn+mx)/2:+.2f} %')

P('\n== mass spread vs own window with the best constant bias (FF, f .45, 84/132 in)')
for sh in ('pollen', 'nectar'):
    for pk in ('asbuilt', 'steel82', 'J4.0'):
        for d in (84, 132):
            ml, mh = mass(sh, pk, d, 0.45)
            lo, hi = WO[sh][d]; lv, hv = WV[d]
            P(f'{sh} {pk} {d}: mass {mh:+.2f}/{ml:+.2f} (width {ml-mh:.2f}) | own window {lo:+.2f}/{hi:+.2f} (width {hi-lo:.2f}) '
              f'| unbiased fits own: {mh >= lo and ml <= hi}, reference: {mh >= lv and ml <= hv} | with bias fits own: {ml-mh <= hi-lo}')

txt = '\n'.join(L)
os.makedirs(os.path.join(HERE, 'results'), exist_ok=True)
tag = os.path.basename(os.path.dirname(os.path.dirname(os.path.abspath(SRC))))
open(os.path.join(HERE, 'results', 'span_analysis.txt' if len(sys.argv) < 3 else sys.argv[2] + '.txt'), 'w').write(txt + '\n')
json.dump(out, open(os.path.join(HERE, 'results', 'span_analysis.json' if len(sys.argv) < 3 else sys.argv[2] + '.json'), 'w'), indent=1)
print(txt)
