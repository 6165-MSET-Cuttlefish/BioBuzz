"""Pass test with the ball-mass spread taken out of the window first (window low side + heavy-ball drop,
high side - light-ball rise). Uses per-ball min/max over seeds from summary.json. Writes budget.txt, results/budget.json."""
import json, os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import numpy as np
import cadcfg as C
import run_cad as R

S = json.load(open(os.path.join(R.OUT, 'summary.json')))
A = dict(S['burst']); A.update(S.get('changes', {}))
M = S['mass']
DS = [36, 60, 84, 108, 132]
REAL = [('team', 'hub', 0.02), ('team', 'oq', 0.02), ('rec', 'hub', 0.02), ('rec', 'oq', 0.02), ('rect', 'hub', 0.02), ('rect', 'oq', 0.02)]
CH_REAL = [('team', 'hub', 0.02), ('rec', 'oq', 0.02), ('rect', 'oq', 0.02), ('rect', 'hub', 0.02)]
LADDER = ['asbuilt', 'g1.25', 'g1.5', 'steel82', 'g1.25_steel82', 'g1.5_steel82', 'J4.0', '2mot', '2mot_steel82', 'w96']
MASS_PK = {'2mot_steel82': 'steel82', 'g1.25_steel82': 'steel82', 'g1.5_steel82': 'steel82', 'g0.8': 'asbuilt'}


def mass_spread(sh, pk, f):
    m = M.get(f'{sh}|{MASS_PK.get(pk, pk)}|84|{f}')
    return (m['lo'], m['hi']) if m else (0.0, 0.0)


def fastest(rec, lo, hi):
    ok = {}
    for sp, mn in rec['per_ball_min'].items():
        mx = rec['per_ball_max'][sp]
        ok[float(sp)] = all(np.isfinite(mn)) and min(mn) >= lo and max(mx) <= hi
    best = None
    for sp in sorted(ok, reverse=True):
        if ok[sp]:
            best = sp
        else:
            break
    return best


def fast(sh, pk, d, f, drag, cm, budget):
    rec = A.get('|'.join(str(x) for x in (sh, pk, d, f, drag, *cm)))
    if rec is None:
        return None, 'n/a'
    if rec.get('infeasible'):
        return None, 'inf'
    lo, hi = C.WIN_REF[d]
    if budget:
        mlo, mhi = mass_spread(sh, pk, f)
        lo, hi = lo - mhi, hi - mlo
        if lo >= 0 or hi <= 0:
            return None, 'none'
    v = fastest(rec, lo, hi)
    return v, ('>300' if v is None else str(int(round(v * 1000))))


def best(sh, pk, d, f, drag, cms, budget):
    vals = [(fast(sh, pk, d, f, drag, cm, budget), cm) for cm in cms]
    good = [(v[0], s, cm) for (v, cm) in vals for s in [v[1]] if v[0] is not None]
    if good:
        g = min(good, key=lambda x: x[0])
        return g[1], g[2]
    return vals[0][0][1], None


L = []
out = {}
L.append('== MASS BUDGET: fastest 10-ball interval (ms) with the ball-mass spread taken out of the reference window; best real loop / ideal')
L.append('   mass spread (heavy/light ball exit change, %) from BALL-MASS SPREAD at 84 in for that plant and f')
for f in C.F_EXIT:
    for drag in C.DRAG:
        for sh in ('pollen', 'nectar'):
            for pk in ['asbuilt', 'steel82', 'J4.0', 'g1.5', '2mot', '2mot_steel82', 'w96']:
                cms = REAL if pk in R.BASE_PLANTS else CH_REAL
                mlo, mhi = mass_spread(sh, pk, f)
                cells = []
                for d in DS:
                    b, cm = best(sh, pk, d, f, drag, cms, True)
                    i = fast(sh, pk, d, f, drag, ('oracle', 'none', 0.02), True)[1]
                    nb, _ = best(sh, pk, d, f, drag, cms, False)
                    cells.append(f'{d}: {b}/{i} (no budget {nb})')
                    out[f'{sh}|{pk}|{d}|{f}|{drag}'] = dict(real_budget=b, ideal_budget=i, real_nobudget=nb, mass=(mlo, mhi))
                L.append(f'f={f:.2f} x{drag:.2f} {sh:6s} {pk:13s} mass {mlo:+.2f}/{mhi:+.2f} | ' + ' | '.join(cells))
txt = '\n'.join(L)
open(os.path.join(HERE, 'budget.txt'), 'w').write(txt + '\n')
json.dump(out, open(os.path.join(R.OUT, 'budget.json'), 'w'), indent=1)
print(txt)
