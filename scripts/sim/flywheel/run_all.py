"""Tune every controller by grid search, then sweep scenarios. Writes results.json + results_tables.txt."""
import os, sys, json, math, time, itertools
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import numpy as np
import flysim as F
from concurrent.futures import ProcessPoolExecutor

RPM = F.RPM
JS = [5e-5, 1e-4, 2e-4, 5e-4]
SPIN_T = {5e-5: 1.5, 1e-4: 2.0, 2e-4: 3.0, 5e-4: 5.0}
REC_T = {5e-5: 1.2, 1e-4: 1.5, 2e-4: 2.0, 5e-4: 3.0}
BASE = dict(m=0.03, Tc=0.02, Voc=12.5, Rint=0.1, fric=0.008, T=0.015, W=0.05, tsA=0.5e-3, ffk=1.0, kAk=1.0)
TUNE_SEEDS = [101, 102, 103, 104]
EVAL_SEEDS = list(range(201, 209))
WARM = 1.5


def P_(**k):
    return dict(k)


GRIDS = {
    'oracle': [{}],
    'ffpi': [dict(kP=a, kI=b) for a in (0.0025, 0.005, 0.01, 0.02, 0.04, 0.08, 0.16) for b in (0, 0.01, 0.03, 0.1, 0.3, 1.0)],
    'ffpid': [dict(kP=a, kI=b, kD=d) for a in (0.0025, 0.005, 0.01, 0.02, 0.04, 0.08, 0.16) for b in (0, 0.03, 0.1, 0.3)
              for d in (1e-5, 3e-5, 1e-4)],
    'ffpiz': [dict(kP=a, kI=b, iz=z) for a in (0.0025, 0.005, 0.01, 0.02, 0.04, 0.08) for b in (0.003, 0.01, 0.03, 0.1, 0.3, 1.0)
              for z in (25, 50, 100, 200)],
    'ffp': [dict(kP=a) for a in (0.00125, 0.0025, 0.005, 0.01, 0.02, 0.04, 0.08, 0.16, 0.32)],
    'bb': [{}],
    'bbff': [dict(alpha=a) for a in (0.5, 0.6, 0.7, 0.8, 0.9, 0.95, 1.0)],
    'hybrid': [dict(thr=t, kP=a, kI=b) for t in (50, 100, 200, 400, 800) for a in (0.0025, 0.005, 0.01, 0.02, 0.04, 0.08)
               for b in (0, 0.03, 0.1, 0.3, 1.0)],
    'boost': [dict(kP=a, B=b, lead=l, dur=d) for a in (0.005, 0.01, 0.02, 0.04, 0.08) for b in (3.0, 6.0, 13.0)
              for l in (-0.03, -0.015, 0.0, 0.015) for d in (0.03, 0.06, 0.1, 0.15)],
    'pred': [dict(kP=a) for a in (0.0025, 0.005, 0.01, 0.02, 0.04, 0.08, 0.16, 0.32)],
    'kf': [dict(kP=a, qd=q, dr=r) for a in (0.005, 0.01, 0.02, 0.04, 0.08, 0.16) for q in (1e2, 1e4, 1e6, 1e8) for r in (0, 1)],
    'hub': [dict(kP=a, kI=b) for a in (3, 10, 30, 100, 300, 1000) for b in (0, 1, 3, 10, 30)],
}
NAMES = {
    'oracle': '0 oracle (true w, 5 kHz, BB+FF) = physics floor',
    'ffpi': '1a table FF + PI (ours: I holds while pinned, D=0)', 'ffpid': '1b table FF + PID (ours, D on meas)', 'ffpiz': '1c table FF + PI with I-zone (integrate only |e|<iz)',
    'ffp': '2 kS+kV FF + P', 'bb': '3 bang-bang (float)', 'bbff': '4 bang-bang + a*FF floor',
    'hybrid': '5 hybrid: full power if e>thr else FF+PI (I frozen when saturated)',
    'boost': '6 FF + P + feeder-triggered boost', 'pred': '7a FF + P + model delay-comp predictor',
    'kf': '7b FF + P + Kalman obs (theta,w,dist) + dist rejection', 'hub': '8 hub RUN_USING_ENCODER PIDF approx (28 CPR)'}
CTRLS = list(GRIDS)


def to_internal(ctrl, g):
    c = dict(g)
    if ctrl in ('ffpi', 'ffpid', 'ffpiz', 'ffp', 'hybrid', 'boost', 'pred', 'kf'):
        for k in ('kP', 'kI', 'kD'):
            if k in c:
                c[k] = c[k] * RPM
    for k in ('thr', 'iz'):
        if k in c:
            c[k] = c[k] / RPM
    return c


def scen_spin(J, **kw):
    T = SPIN_T[J]
    s = dict(T_end=T, w0_rpm=0.0, target_rpm=3000.0, shots=[], ss_win=(T - 0.5, T), cur_win=(T - 0.5, T))
    s.update(kw)
    return s


def scen_burst(J, spacing, nb=3, **kw):
    shots = [WARM + i * spacing for i in range(nb)]
    T = shots[-1] + REC_T[J]
    s = dict(T_end=T, w0_rpm=3000.0, target_rpm=3000.0, shots=shots, ss_win=(WARM - 0.5, WARM),
             cur_win=(WARM - 0.05, WARM + 1.0))
    s.update(kw)
    return s


def run(ctrl, J, gains_list, seeds, scen, over=None, per_sim=None):
    """Batch: every gain set x every seed (x every per_sim variant). Returns list of (gain, variant, res-dict-of-arrays)."""
    over = over or {}
    per_sim = per_sim or [{}]
    rows = list(itertools.product(range(len(gains_list)), range(len(per_sim)), seeds))
    N = len(rows)
    P = {k: np.full(N, v, float) for k, v in BASE.items()}
    for k, v in over.items():
        P[k] = np.full(N, v, float)
    P['J'] = np.full(N, J)
    P['seed'] = np.array([r[2] for r in rows])
    for vi, ps in enumerate(per_sim):
        sel = np.array([r[1] == vi for r in rows])
        for k, v in ps.items():
            P[k][sel] = v
    keys = set(k for g in gains_list for k in g)
    C = {}
    for k in keys:
        C[k] = np.array([to_internal(ctrl, gains_list[r[0]]).get(k, 0.0) for r in rows], float)
    res = F.simulate(P, ctrl, C, scen)
    return rows, res


def agg(rows, res, ng, nv):
    """mean over seeds -> dict[(gi, vi)] -> metric -> value"""
    out = {}
    for gi in range(ng):
        for vi in range(nv):
            sel = np.array([r[0] == gi and r[1] == vi for r in rows])
            d = {}
            for k, v in res.items():
                v = v[sel]
                if v.ndim == 1:
                    d[k] = float(np.nanmean(v)) if np.isfinite(v).any() else float('nan')
                    d[k + '_nanfrac'] = float(np.mean(~np.isfinite(v)))
                    if k in ('rec25', 'rec50', 'rec100', 't_settle50'):
                        d[k + '_max'] = float(np.nanmax(v)) if np.isfinite(v).any() else float('nan')
                else:
                    if k == 'w_exit':
                        we = v
                        spread = (np.nanmax(we, 1) - np.nanmin(we, 1)) / np.nanmean(we, 1) * 100
                        d['exit_spread_pct'] = float(np.nanmean(spread))
                        d['exit_dev23_pct'] = float(np.nanmean(np.abs(we[:, 1:] - we[:, :1]) / we[:, :1] * 100)) if we.shape[1] > 1 else float('nan')
                    elif k == 'e_touch':
                        for b in range(v.shape[1]):
                            d[f'e{b+1}'] = float(np.nanmean(v[:, b]))
                            d[f'abs_e{b+1}'] = float(np.nanmean(np.abs(v[:, b])))
                    elif k == 'meas_err_touch':
                        d['abs_meas_err_touch'] = float(np.nanmean(np.abs(v[:, 1:]))) if v.shape[1] > 1 else float(np.nanmean(np.abs(v)))
                        d['abs_meas_err_b1'] = float(np.nanmean(np.abs(v[:, 0])))
            out[(gi, vi)] = d
    return out


def cost_of(sp, bu, J):
    ts = sp['t_settle50'] if math.isfinite(sp['t_settle50']) and sp['t_settle50_nanfrac'] == 0 else SPIN_T[J]
    rec = bu['rec50'] if math.isfinite(bu['rec50']) and bu['rec50_nanfrac'] == 0 else REC_T[J]
    shot = (0.8 * sp['ss_std'] + abs(sp['ss_bias']) + bu['abs_e2'] + bu['abs_e3']) / 3
    return shot / 50 + ts / 0.5 + rec / 0.25 + max(sp['overshoot'], 0) / 200, shot, ts, rec


TUNE_FF = [dict(ffk=0.95), dict(ffk=1.0), dict(ffk=1.05)]


def tune(args):
    """Grid search; cost averaged over 4 seeds x feed-forward error {-5%, 0, +5%}."""
    ctrl, J, over, tag = args
    grid = GRIDS[ctrl]
    r1, s1 = run(ctrl, J, grid, TUNE_SEEDS, scen_spin(J), over, TUNE_FF)
    a1 = agg(r1, s1, len(grid), len(TUNE_FF))
    r2, s2 = run(ctrl, J, grid, TUNE_SEEDS, scen_burst(J, 0.25), over, TUNE_FF)
    a2 = agg(r2, s2, len(grid), len(TUNE_FF))
    costs = []
    for i in range(len(grid)):
        cs = [cost_of(a1[(i, v)], a2[(i, v)], J) for v in range(len(TUNE_FF))]
        costs.append([float(np.mean([c[j] for c in cs])) for j in range(4)])
    best = int(np.argmin([c[0] for c in costs]))
    edge = []
    for k in grid[best]:
        vals = sorted(set(g[k] for g in grid))
        if len(vals) > 1 and grid[best][k] in (vals[0], vals[-1]):
            edge.append(k)
    order = np.argsort([c[0] for c in costs])[:3]
    return dict(ctrl=ctrl, J=J, tag=tag, gains=grid[best], cost=costs[best][0], shot=costs[best][1],
                spin=costs[best][2], rec=costs[best][3], edge=edge, ngrid=len(grid),
                top3=[(grid[i], round(costs[i][0], 3)) for i in order])


def evaluate(args):
    ctrl, J, gains, over, kind, tag, extra = args
    if kind == 'burst':
        sp = extra['spacing']
        variants = extra.get('variants', [{}])
        rows, res = run(ctrl, J, [gains], EVAL_SEEDS, scen_burst(J, sp, **extra.get('scen', {})), over, variants)
        a = agg(rows, res, 1, len(variants))
        return [dict(ctrl=ctrl, J=J, tag=tag, kind=kind, spacing=sp, variant=variants[vi], **a[(0, vi)]) for vi in range(len(variants))]
    if kind == 'single':
        variants = extra.get('variants', [{}])
        rows, res = run(ctrl, J, [gains], EVAL_SEEDS, scen_burst(J, 1.0, nb=1, **extra.get('scen', {})), over, variants)
        a = agg(rows, res, 1, len(variants))
        return [dict(ctrl=ctrl, J=J, tag=tag, kind=kind, variant=variants[vi], **a[(0, vi)]) for vi in range(len(variants))]
    if kind == 'spin':
        variants = extra.get('variants', [{}])
        rows, res = run(ctrl, J, [gains], EVAL_SEEDS, scen_spin(J, **extra.get('scen', {})), over, variants)
        a = agg(rows, res, 1, len(variants))
        return [dict(ctrl=ctrl, J=J, tag=tag, kind=kind, variant=variants[vi], **a[(0, vi)]) for vi in range(len(variants))]
    raise ValueError(kind)


def main():
    t0 = time.time()
    stage = sys.argv[1] if len(sys.argv) > 1 else 'all'
    resf = os.path.join(HERE, 'results.json')
    R = json.load(open(resf)) if os.path.exists(resf) else {}
    ex = ProcessPoolExecutor(max_workers=10)

    if stage in ('all', 'tune'):
        tasks = [(c, J, {}, 'base') for c in CTRLS for J in JS]
        tasks.sort(key=lambda t: -len(GRIDS[t[0]]) * SPIN_T[t[1]])
        R['tune_base'] = list(ex.map(tune, tasks))
        json.dump(R, open(resf, 'w'), indent=1)
        print(f'tune done {time.time()-t0:.0f}s', flush=True)

    best = {(d['ctrl'], d['J']): d['gains'] for d in R['tune_base']}

    if stage in ('all', 'core'):
        tasks = []
        mvars = [dict(m=m, Tc=tc) for m in (0.01, 0.03, 0.06) for tc in (0.015, 0.03)]
        for c in CTRLS:
            for J in JS:
                g = best[(c, J)]
                for sp in (0.15, 0.25, 0.4):
                    tasks.append((c, J, g, {}, 'burst', 'core', dict(spacing=sp, variants=mvars)))
                tasks.append((c, J, g, {}, 'single', 'core', dict(variants=mvars)))
                tasks.append((c, J, g, {}, 'spin', 'core', dict(variants=[dict(fric=f) for f in (0.004, 0.008, 0.016)])))
        R['core'] = [x for r in ex.map(evaluate, tasks) for x in r]
        json.dump(R, open(resf, 'w'), indent=1)
        print(f'core done {time.time()-t0:.0f}s', flush=True)

    if stage in ('all', 'robust'):
        tasks = []
        for c in CTRLS:
            for J in JS:
                g = best[(c, J)]
                for name, over, scen in (
                        ('ff-10', dict(ffk=0.9), {}), ('ff+10', dict(ffk=1.1), {}),
                        ('ff-5', dict(ffk=0.95), {}), ('ff+5', dict(ffk=1.05), {}),
                        ('bat12.0_R0.20', dict(Voc=12.0, Rint=0.2), {}), ('bat13.5_R0.05', dict(Voc=13.5, Rint=0.05), {}),
                        ('bat12.0_R0.05', dict(Voc=12.0, Rint=0.05), {}), ('bat13.5_R0.20', dict(Voc=13.5, Rint=0.2), {}),
                        ('pwm_coast', {}, dict(pwm='coast')), ('kA-20', dict(kAk=0.8), {}), ('kA+20', dict(kAk=1.2), {})):
                    tasks.append((c, J, g, over, 'burst', name, dict(spacing=0.25, scen=scen)))
                    tasks.append((c, J, g, over, 'spin', name, dict(scen=scen)))
                tasks.append((c, J, g, dict(Rint=0.15), 'burst', 'sag25A',
                              dict(spacing=0.25, scen=dict(sag=(25.0, WARM - 0.1, WARM + 1.0)))))
        R['robust'] = [x for r in ex.map(evaluate, tasks) for x in r]
        json.dump(R, open(resf, 'w'), indent=1)
        print(f'robust done {time.time()-t0:.0f}s', flush=True)

    if stage in ('all', 'timing'):
        combos = [(T, W) for T in (0.008, 0.015, 0.025) for W in (0.015, 0.03, 0.05)]
        Jt = [1e-4, 5e-4]
        tasks = [(c, J, dict(T=T, W=W), f'T{T}_W{W}') for c in CTRLS if c not in ('oracle', 'hub')
                 for J in Jt for (T, W) in combos]
        tasks.sort(key=lambda t: -len(GRIDS[t[0]]) * SPIN_T[t[1]])
        R['tune_timing'] = list(ex.map(tune, tasks))
        json.dump(R, open(resf, 'w'), indent=1)
        print(f'timing tune done {time.time()-t0:.0f}s', flush=True)
        tb = {(d['ctrl'], d['J'], d['tag']): d['gains'] for d in R['tune_timing']}
        tasks = []
        for (c, J, tag), g in tb.items():
            T, W = [float(x[1:]) for x in tag.split('_')]
            tv = [dict(tsA=a) for a in (0.0, 0.5e-3, 1.5e-3)]
            tasks.append((c, J, g, dict(T=T, W=W), 'burst', tag, dict(spacing=0.25, variants=tv)))
            tasks.append((c, J, g, dict(T=T, W=W), 'spin', tag, dict(variants=tv)))
            # fixed (baseline-tuned) gains in the same timing condition
            tasks.append((c, J, best[(c, J)], dict(T=T, W=W), 'burst', tag + '_basegains', dict(spacing=0.25, variants=tv)))
            tasks.append((c, J, best[(c, J)], dict(T=T, W=W), 'spin', tag + '_basegains', dict(variants=tv)))
        R['timing'] = [x for r in ex.map(evaluate, tasks) for x in r]
        json.dump(R, open(resf, 'w'), indent=1)
        print(f'timing eval done {time.time()-t0:.0f}s', flush=True)

    if stage in ('all', 'hubdefault'):
        # REV FGC guide: F = 32767/maxV, P = 0.1F, I = 0.1P (I units assumed per-loop sum -> per-second x100)
        tasks = []
        for J in JS:
            maxv = F.free_speed_rpm(12.5, 0.1, *F.friction_coeffs(0.008)) / 60 * F.CPR_MOTOR
            Fh = 32767 / maxv
            g = dict(kP=0.1 * Fh, kI=0.1 * 0.1 * Fh / 0.010)
            tasks.append(('hub', J, g, {}, 'burst', 'hub_revguide', dict(spacing=0.25)))
            tasks.append(('hub', J, g, {}, 'spin', 'hub_revguide', {}))
        R['hubdefault'] = [x for r in ex.map(evaluate, tasks) for x in r]
        json.dump(R, open(resf, 'w'), indent=1)
    ex.shutdown()
    print(f'done {time.time()-t0:.0f}s')


if __name__ == '__main__':
    main()
