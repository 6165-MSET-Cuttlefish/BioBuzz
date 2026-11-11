"""Reruns with the study's simulator (../flysim_cad.py) and its tuned gains (../results/tune.json, from run_cad.py tune).

python3 -I rerun.py <experiment> [workers]
  seeds   : new seeds 901-908 for as-built/steel82/J4.0/2mot_steel82, f .45, drag 1.00/1.14
  f040    : exit fraction 0.40 (between the study's 0.35 and 0.45), as-built and steel82, original seeds
  dt      : plant step 1e-4 s instead of 2e-4 (run with env FLY_DT=1e-4), as-built
  heavy   : heavy-tailed loop period (run with env FLY_HEAVY=1), as-built
  ktke    : motor torque constant kt = ke (stall 1.75 kg cm, near the spec sheet's 1.74), as-built, same targets and gains
  orig    : the original seeds 301-308 for the same cells as 'seeds' (must reproduce ../results/summary.json)
Writes results/rerun_<experiment>.json with the fastest intervals under the pass rules anc, anc_own and bias
(see span_analysis.py).
"""
import json, os, sys, time
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import numpy as np
import flysim_cad as F
import cadcfg as C
import ballistics as B
import run_cad as R

DS = [36, 60, 84, 108, 132]
WV = B.REF_WIN
WO = {sh: {d: (B.shot(sh, d)['win_lo_pct'], B.shot(sh, d)['win_hi_pct']) for d in DS} for sh in ('pollen', 'nectar')}
CMS = [('oracle', 'none', 0.02), ('team', 'hub', 0.02), ('rec', 'hub', 0.02), ('rect', 'hub', 0.02), ('rect', 'oq', 0.02), ('rec', 'oq', 0.02)]


def fastest(by, rule, sh, d, nb=10):
    best = None
    for sp in sorted(by, reverse=True):
        x = by[sp][:, :nb]
        ok = np.all(np.isfinite(x))
        if ok:
            if rule == 'anc':
                lo, hi = WV[d]; ok = x.min() >= lo and x.max() <= hi
            elif rule == 'anc_own':
                lo, hi = WO[sh][d]; ok = x.min() >= lo and x.max() <= hi
            else:
                lo, hi = WO[sh][d]; ok = (x.max() - x.min()) <= hi - lo
        if ok:
            best = sp
        else:
            break
    return None if best is None else int(round(best * 1000))


def analyse(rows, skipped):
    G = {}
    for r in rows:
        G.setdefault((r['sh'], r['pk'], r['d'], r['f'], r['drag'], r['ctrl'], r['meas'], r['Nq']), []).append(r)
    out = {}
    for key, rs in G.items():
        sh, d = key[0], key[2]
        ref = np.mean([r['w_exit'][0] for r in rs])
        by = {}
        for r in rs:
            by.setdefault(r['spacing'], []).append(np.array(r['w_exit']) / ref * 100 - 100)
        by = {k: np.array(v) for k, v in by.items()}
        out['|'.join(map(str, key))] = {f'{rule}{nb}': fastest(by, rule, sh, d, nb) for rule in ('anc', 'anc_own', 'bias') for nb in (3, 10)}
        out['|'.join(map(str, key))]['tgt_rpm'] = rs[0]['tgt_rpm']
        out['|'.join(map(str, key))]['worst92'] = [float(by[0.092].min()), float(by[0.092].max())] if 0.092 in by else None
    for s in skipped:
        k = '|'.join(map(str, (s['sh'], s['pk'], s['d'], s['f'], s['drag'], s['ctrl'], s['meas'], s['Nq'])))
        out.setdefault(k, dict(infeasible=True, frac=s['frac']))
    return out


def build(exp):
    best = json.load(open(os.path.join(R.OUT, 'tune.json')))
    rows = []
    if exp in ('seeds', 'orig'):
        seeds = list(range(901, 909)) if exp == 'seeds' else R.SEEDS
        cells = [(sh, pk, f, dr) for sh in ('pollen', 'nectar') for pk in ('asbuilt', 'steel82', 'J4.0', '2mot_steel82')
                 for f in (0.45,) for dr in (1.0, 1.14)]
        cms = CMS[:5]
    elif exp == 'f040':
        seeds = R.SEEDS
        cells = [(sh, pk, 0.40, dr) for sh in ('pollen', 'nectar') for pk in ('asbuilt', 'steel82') for dr in (1.0, 1.14)]
        cms = CMS
    elif exp == 'ktke':
        seeds = R.SEEDS
        cells = [(sh, 'asbuilt', 0.45, dr) for sh in ('pollen', 'nectar') for dr in (1.0, 1.14)]
        cms = CMS
    elif exp in ('dt', 'heavy'):
        seeds = R.SEEDS
        cells = [(sh, 'asbuilt', 0.45, 1.0) for sh in ('pollen', 'nectar')]
        cms = CMS[:5]
    else:
        raise SystemExit(exp)
    for (sh, pk, f, dr) in cells:
        for cm in cms:
            g = R.gains_for(best, sh, pk, *cm)
            for d in DS:
                for sp in R.INTERVALS:
                    for sd in seeds:
                        r = R.row(sh, d, f, dr, pk, cm[0], cm[1], cm[2], sp, sd, g)
                        if exp == 'ktke':
                            r['ktm'] = F.KE
                        rows.append(r)
    return rows


def main():
    exp = sys.argv[1]
    workers = int(sys.argv[2]) if len(sys.argv) > 2 else 8
    t0 = time.time()
    rows = build(exp)
    out, sk = R.execute(rows, R.NB, 0.4, workers)
    A = analyse(out, sk)
    A['_meta'] = dict(exp=exp, n=len(out), skipped=len(sk), dt=F.DT, heavy=os.environ.get('FLY_HEAVY'), secs=time.time() - t0)
    os.makedirs(os.path.join(HERE, 'results'), exist_ok=True)
    json.dump(A, open(os.path.join(HERE, 'results', f'rerun_{exp}.json'), 'w'), indent=1)
    print(exp, len(out), 'sims', len(sk), 'skipped', f'{time.time()-t0:.0f}s')


if __name__ == '__main__':
    main()
