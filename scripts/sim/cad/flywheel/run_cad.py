"""Stages: tune, single, burst, oqsweep, changes, mass. Results go to results/<stage>.json.

python3 -I run_cad.py <stage> [workers]
"""
import itertools, json, math, os, sys, time
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import numpy as np
import flysim_cad as F
import cadcfg as C
from concurrent.futures import ProcessPoolExecutor

RPM = F.RPM
OUT = os.path.join(HERE, 'results')
os.makedirs(OUT, exist_ok=True)
INTERVALS = [0.092, 0.105, 0.120, 0.135, 0.150, 0.175, 0.200, 0.250, 0.300]
NB = 10
WARM = 0.5
SEEDS = [301, 302, 303, 304, 305, 306, 307, 308]
TUNE_SEEDS = [101, 102, 103, 104, 105, 106]
CTRLM = [('oracle', 'none', 0.02), ('team', 'hub', 0.02), ('team', 'oq', 0.02), ('rec', 'hub', 0.02), ('rec', 'oq', 0.02),
         ('rect', 'hub', 0.02), ('rect', 'oq', 0.02)]
GRID = {
    'team': [dict(kP=a, kI=b) for a in (0.0025, 0.005, 0.01, 0.02, 0.04, 0.08) for b in (0.0, 0.01, 0.03, 0.1)],
    'rec': [dict(kP=a, kI=b) for a in (0.0025, 0.005, 0.01, 0.02, 0.04, 0.08) for b in (0.0, 0.03, 0.1, 0.3)],
    'rect': [dict(kP=a, kI=b, thr=t) for a in (0.005, 0.01, 0.02, 0.04, 0.08) for b in (0.0, 0.03, 0.1)
             for t in (0.02, 0.03, 0.04, 0.06, 0.10)],
}
SIMCTRL = {'rect': 'rec'}


def plants(sh):
    ja = C.J_ASBUILT[sh]
    roller = C.J_ROLLER if sh == 'pollen' else 0.0
    base = dict(nmot=1, g=1.0, r=C.R_WHEEL, fric=0.008, ktm=F.KT_STALL)
    P = {'asbuilt': dict(base, J=ja)}
    for j in C.J_SWEEP:
        P[f'J{j*1e4:.1f}'] = dict(base, J=j)
    P['steel82'] = dict(base, J=ja + C.J_STEEL82)
    P['2mot'] = dict(base, J=ja + C.J_ROTOR, nmot=2, fric=0.010)
    P['2mot_steel82'] = dict(base, J=ja + C.J_ROTOR + C.J_STEEL82, nmot=2, fric=0.010)
    P['w96'] = dict(base, J=C.J_WHEEL96 + (C.J_DESIGN - C.J_WHEEL72) + C.J_ROTOR + roller, r=0.048)
    for g in (0.8, 1.25, 1.5):
        P[f'g{g}'] = dict(base, J=ja - C.J_ROTOR + C.J_ROTOR / (g * g), g=g)
        P[f'g{g}_steel82'] = dict(base, J=ja - C.J_ROTOR + C.J_ROTOR / (g * g) + C.J_STEEL82, g=g)
    return P


BASE_PLANTS = ['asbuilt', 'J1.2', 'J1.6', 'J2.4', 'J4.0']
CHANGE_PLANTS = ['steel82', '2mot', '2mot_steel82', 'w96', 'g0.8', 'g1.25', 'g1.5', 'g1.25_steel82', 'g1.5_steel82']

_tcache = {}


def tgt_info(sh, d, f, drag, pk, m=None):
    key = (sh, d, f, drag, pk, m)
    if key not in _tcache:
        pl = plants(sh)[pk]
        w, info = C.target(sh, d, f, drag, pl['J'], r=pl['r'], nmot=pl['nmot'], g=pl['g'], ktm=pl['ktm'], m=m)
        wf = F.free_speed(12.5, 0.1, pl['fric'], pl['nmot'], pl['g'], pl['ktm'])
        info.update(tgt=w, free=wf, frac=w / wf)
        _tcache[key] = info
    return _tcache[key]


def row(sh, d, f, drag, pk, ctrl, meas, Nq, spacing, seed, gains, m=None, Voc=12.5):
    pl = plants(sh)[pk]
    ti = tgt_info(sh, d, f, drag, pk, m)
    g = gains or {}
    lat = C.LAT['rec' if ctrl == 'rect' else ctrl]
    return dict(sh=sh, d=d, f=f, drag=drag, pk=pk, ctrl=ctrl, meas=meas, Nq=Nq, spacing=spacing, seed=seed,
                J=pl['J'], m=C.BALL[sh]['m'] if m is None else m, Tc=ti['Tc'], Voc=Voc, Rint=0.1, fric=pl['fric'],
                T=0.015, W=0.05, tsA=0.5e-3, tgt=ti['tgt'], r=pl['r'], f_=f, nmot=pl['nmot'], g=pl['g'], ktm=pl['ktm'],
                lat_lo=lat[0], lat_hi=lat[1], kP=g.get('kP', 0.0) * RPM, kI=g.get('kI', 0.0) * RPM,
                iz=75.0 / RPM, thr=g.get('thr', 0.02), frac=ti['frac'])


SIMKEYS = ['J', 'm', 'Tc', 'Voc', 'Rint', 'fric', 'T', 'W', 'tsA', 'seed', 'tgt', 'r', 'nmot', 'g', 'ktm', 'Nq',
           'lat_lo', 'lat_hi', 'kP', 'kI', 'iz', 'thr', 'spacing']


def run_batch(args):
    rows, ctrl, meas, nb, T_end = args
    P = {k: np.array([r[k] for r in rows], float) for k in SIMKEYS}
    P['f'] = np.array([r['f_'] for r in rows], float)
    P['seed'] = np.array([r['seed'] for r in rows], int)
    res = F.simulate(P, SIMCTRL.get(ctrl, ctrl), 'hub' if meas == 'none' else meas, nb=nb, warm=WARM, T_end=T_end)
    out = []
    for i, r in enumerate(rows):
        o = {k: r[k] for k in ('sh', 'd', 'f', 'drag', 'pk', 'ctrl', 'meas', 'Nq', 'spacing', 'seed', 'm', 'frac')}
        if 'gi' in r:
            o['gi'] = r['gi']
        o['tgt_rpm'] = float(res['tgt_rpm'][i])
        o['w_exit'] = [float(x) for x in res['w_exit'][i]]
        o['w_touch'] = [float(x) for x in res['w_touch'][i]]
        o['wm_err_touch'] = [float(a - b) for a, b in zip(res['wm_touch'][i], res['w_touch'][i])]
        o['e_ball1'] = float(res['e_ball'][i, 0])
        o['dip1_rpm'] = float(res['dip1_rpm'][i])
        for b in (1, 2, 3):
            o[f'rec_last_{b}'] = float(res[f'rec_last_{b}'][i])
            o[f'rec_first_{b}'] = float(res[f'rec_first_{b}'][i])
        o['ss_std_rpm'] = float(res['ss_std_rpm'][i])
        o['ss_bias_rpm'] = float(res['ss_bias_rpm'][i])
        o['meas_std_rpm'] = float(res['meas_std_rpm'][i])
        out.append(o)
    return out


def execute(rows, nb, tail, workers, chunk=900):
    groups = {}
    for r in rows:
        if r['frac'] >= 0.995:
            continue
        T_end = WARM + (nb - 1) * r['spacing'] + tail
        groups.setdefault((r['ctrl'], r['meas'], round(T_end, 4)), []).append(r)
    tasks = []
    for (ctrl, meas, T_end), rs in groups.items():
        for i in range(0, len(rs), chunk):
            tasks.append((rs[i:i + chunk], ctrl, meas, nb, T_end))
    tasks.sort(key=lambda t: -len(t[0]) * t[4])
    out = []
    with ProcessPoolExecutor(max_workers=workers) as ex:
        for r in ex.map(run_batch, tasks):
            out.extend(r)
    skipped = [{k: r[k] for k in ('sh', 'd', 'f', 'drag', 'pk', 'ctrl', 'meas', 'Nq', 'spacing', 'seed', 'frac')}
               for r in rows if r['frac'] >= 0.995]
    return out, skipped


def tune_cost(res_rows):
    by = {}
    for o in res_rows:
        by.setdefault(o['gi'], []).append(o)
    costs = {}
    for gi, rs in by.items():
        refs = {}
        for o in rs:
            refs.setdefault((o['sh'], o['spacing']), []).append(o['w_exit'][0])
        c = []
        for o in rs:
            ref = np.mean(refs[(o['sh'], o['spacing'])])
            dev = np.abs(np.array(o['w_exit'][:5]) / ref - 1) * 100
            ci = np.max(dev) if np.all(np.isfinite(dev)) else 50.0
            ci += 2.0 * o['ss_std_rpm'] / o['tgt_rpm'] * 100
            ci += 5.0 if not np.isfinite(o['rec_last_2']) else 0.0
            c.append(ci)
        costs[gi] = float(np.mean(c))
    return costs


def stage_tune(workers):
    tasks = []
    meta = []
    for sh in ('pollen', 'nectar'):
        for pk in BASE_PLANTS + CHANGE_PLANTS:
            for ctrl in ('team', 'rec', 'rect'):
                for meas, Nq in (('hub', 0.02), ('oq', 0.02), ('oq', 0.01), ('oq', 0.05)):
                    if Nq != 0.02 and pk not in BASE_PLANTS:
                        continue
                    rows = []
                    for gi, gset in enumerate(GRID[ctrl]):
                        for sp in (0.15, 0.25):
                            for sd in TUNE_SEEDS:
                                r = row(sh, 84, 0.45, 1.0, pk, ctrl, meas, Nq, sp, sd, gset)
                                r['gi'] = gi
                                rows.append(r)
                    meta.append((sh, pk, ctrl, meas, Nq))
                    tasks.append(rows)
    allrows = [r for t in tasks for r in t]
    t0 = time.time()
    out, _ = execute(allrows, 5, 0.6, workers)
    print(f'tune sims {len(out)} in {time.time()-t0:.0f}s', flush=True)
    best = {}
    for (sh, pk, ctrl, meas, Nq) in meta:
        rs = [o for o in out if o['sh'] == sh and o['pk'] == pk and o['ctrl'] == ctrl and o['meas'] == meas and o['Nq'] == Nq]
        costs = tune_cost(rs)
        gi = min(costs, key=costs.get)
        order = sorted(costs, key=costs.get)[:3]
        key = f'{sh}|{pk}|{ctrl}|{meas}|{Nq}'
        best[key] = dict(gains=GRID[ctrl][gi], cost=costs[gi], top3=[(GRID[ctrl][i], round(costs[i], 3)) for i in order])
    json.dump(best, open(os.path.join(OUT, 'tune.json'), 'w'), indent=1)
    return best


def gains_for(best, sh, pk, ctrl, meas, Nq):
    if ctrl in ('oracle', 'ff'):
        return {}
    return best[f'{sh}|{pk}|{ctrl}|{meas}|{Nq}']['gains']


def main():
    stage = sys.argv[1]
    workers = int(sys.argv[2]) if len(sys.argv) > 2 else 6
    t0 = time.time()
    if stage == 'tune':
        stage_tune(workers)
        print(f'done {time.time()-t0:.0f}s')
        return
    best = json.load(open(os.path.join(OUT, 'tune.json')))
    rows = []
    if stage in ('burst', 'single', 'oqsweep', 'changes'):
        if stage == 'oqsweep':
            cms = [('team', 'oq', 0.01), ('team', 'oq', 0.05), ('rec', 'oq', 0.01), ('rec', 'oq', 0.05), ('rect', 'oq', 0.01), ('rect', 'oq', 0.05)]
            pks, fs = BASE_PLANTS, [0.45]
        elif stage == 'changes':
            cms = [('oracle', 'none', 0.02), ('team', 'hub', 0.02), ('rec', 'oq', 0.02), ('rect', 'oq', 0.02), ('rect', 'hub', 0.02)]
            pks, fs = CHANGE_PLANTS, C.F_EXIT
        else:
            cms, pks, fs = CTRLM, BASE_PLANTS, C.F_EXIT
        ints = [1.0] if stage == 'single' else INTERVALS
        for sh in ('pollen', 'nectar'):
            for pk in pks:
                for (ctrl, meas, Nq) in cms:
                    gset = gains_for(best, sh, pk, ctrl, meas, Nq)
                    for d in (36, 60, 84, 108, 132):
                        for f in fs:
                            for dr in C.DRAG:
                                for sp in ints:
                                    for sd in SEEDS:
                                        rows.append(row(sh, d, f, dr, pk, ctrl, meas, Nq, sp, sd, gset))
        nb, tail = (1, 1.2) if stage == 'single' else (NB, 0.4)
        out, skipped = execute(rows, nb, tail, workers)
        json.dump(dict(rows=out, skipped=skipped), open(os.path.join(OUT, f'{stage}.json'), 'w'))
        print(f'{stage}: {len(out)} sims, {len(skipped)} infeasible skipped, {time.time()-t0:.0f}s')
    elif stage == 'mass':
        for sh in ('pollen', 'nectar'):
            for pk in BASE_PLANTS + ['steel82', '2mot', 'w96', 'g1.25', 'g1.5']:
                for d in (36, 84, 132):
                    for f in C.F_EXIT:
                        ti = tgt_info(sh, d, f, 1.0, pk)
                        for mm in (C.BALL[sh]['m_lo'], C.BALL[sh]['m'], C.BALL[sh]['m_hi']):
                            for ctrl, meas in (('ff', 'none'), ('oracle', 'none')):
                                r = row(sh, d, f, 1.0, pk, ctrl, meas, 0.02, 1.0, 1, {})
                                r['m'] = mm
                                rows.append(r)
        out, skipped = execute(rows, 1, 0.3, workers)
        json.dump(dict(rows=out, skipped=skipped), open(os.path.join(OUT, 'mass.json'), 'w'))
        print(f'mass: {len(out)} sims {time.time()-t0:.0f}s')


if __name__ == '__main__':
    main()
