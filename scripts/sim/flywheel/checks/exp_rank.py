"""Ranking experiments. Usage: python3 -I exp_rank.py TAG GAINS(base|retune|retune_shot) SEEDSTART NSEEDS [ctrls]
Plant overrides come from env (FLY_KT=ke, FLY_QW, FLY_DT). Writes out_TAG.json next to itself and prints a summary."""
import os, sys, json, math, itertools, time
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import numpy as np
import flysim as F
import run_all as R
from concurrent.futures import ProcessPoolExecutor

JS = R.JS
MV = [(m, tc) for m in (0.01, 0.03, 0.06) for tc in (0.015, 0.03)]
SPS = (0.15, 0.25, 0.4)
REAL = ['ffpi', 'ffpid', 'ffpiz', 'ffp', 'bb', 'bbff', 'hybrid', 'boost', 'pred', 'kf', 'hub']


def cost_shot(sp, bu, J):
    ts = sp['t_settle50'] if math.isfinite(sp['t_settle50']) and sp['t_settle50_nanfrac'] == 0 else R.SPIN_T[J]
    rec = bu['rec50'] if math.isfinite(bu['rec50']) and bu['rec50_nanfrac'] == 0 else R.REC_T[J]
    shot = (0.8 * sp['ss_std'] + abs(sp['ss_bias']) + bu['abs_e2'] + bu['abs_e3']) / 3
    pen = (ts >= R.SPIN_T[J]) * 5 + (rec >= R.REC_T[J]) * 5
    return shot / 10 + pen + max(sp['overshoot'], 0) / 200, shot, ts, rec


def tune_w(args):
    ctrl, J, kind = args
    if kind == 'retune':
        return R.tune((ctrl, J, {}, 'x'))
    grid = R.GRIDS[ctrl]
    r1, s1 = R.run(ctrl, J, grid, R.TUNE_SEEDS, R.scen_spin(J), {}, R.TUNE_FF)
    a1 = R.agg(r1, s1, len(grid), len(R.TUNE_FF))
    r2, s2 = R.run(ctrl, J, grid, R.TUNE_SEEDS, R.scen_burst(J, 0.25), {}, R.TUNE_FF)
    a2 = R.agg(r2, s2, len(grid), len(R.TUNE_FF))
    costs = []
    for i in range(len(grid)):
        cs = [cost_shot(a1[(i, v)], a2[(i, v)], J) for v in range(len(R.TUNE_FF))]
        costs.append(float(np.mean([c[0] for c in cs])))
    b = int(np.argmin(costs))
    return dict(ctrl=ctrl, J=J, gains=grid[b], cost=costs[b])


def eval_w(args):
    ctrl, J, gains, sp, seeds = args
    per = [dict(m=m, Tc=tc) for m, tc in MV]
    rows, res = R.run(ctrl, J, [gains], seeds, R.scen_burst(J, sp), {}, per)
    vi = np.array([r[1] for r in rows])
    out = {}
    for v in range(len(per)):
        s = vi == v
        et = res['e_touch'][s]
        we = res['w_exit'][s]
        out[v] = dict(e23=(0.5 * (np.abs(et[:, 1]) + np.abs(et[:, 2]))).tolist(),
                      exdev=(np.mean(np.abs(we[:, 1:] - we[:, :1]), 1) / we[:, 0] * 100).tolist(),
                      spread=((we.max(1) - we.min(1)) / we.mean(1) * 100).tolist(),
                      esterr=np.mean(np.abs(res['est_err_touch'][s][:, 1:]), 1).tolist(),
                      measerr=np.mean(np.abs(res['meas_err_touch'][s][:, 1:]), 1).tolist())
    return ctrl, J, sp, out


def main():
    tag, gsrc, s0, ns = sys.argv[1], sys.argv[2], int(sys.argv[3]), int(sys.argv[4])
    ctrls = sys.argv[5].split(',') if len(sys.argv) > 5 else REAL
    seeds = list(range(s0, s0 + ns))
    base = {(d['ctrl'], d['J']): d['gains'] for d in json.load(open(os.path.join(HERE, '..', 'results.json')))['tune_base']}
    t0 = time.time()
    with ProcessPoolExecutor(max_workers=int(os.environ.get('NW', '8'))) as ex:
        if gsrc == 'base':
            gains = {(c, J): base[(c, J)] for c in ctrls for J in JS}
        else:
            kind = 'retune' if gsrc == 'retune' else 'shot'
            tasks = [(c, J, kind) for c in ctrls for J in JS]
            tasks.sort(key=lambda t: -len(R.GRIDS[t[0]]) * R.SPIN_T[t[1]])
            tr = list(ex.map(tune_w, tasks))
            gains = {(d['ctrl'], d['J']): d['gains'] for d in tr}
            for d in sorted(tr, key=lambda d: (d['J'], d['ctrl'])):
                print(f"tuned {tag} J={d['J']:.0e} {d['ctrl']:7s} {d['gains']}  (base {base[(d['ctrl'], d['J'])]})", flush=True)
        print(f'tuning {time.time()-t0:.0f}s', flush=True)
        tasks = [(c, J, gains[(c, J)], sp, seeds) for c in ctrls for J in JS for sp in SPS]
        ev = list(ex.map(eval_w, tasks))
    D = {}
    for c, J, sp, out in ev:
        for v, d in out.items():
            D[(c, J, sp, v)] = d
    json.dump(dict(gains={f'{c}|{J}': g for (c, J), g in gains.items()},
                   data={f'{c}|{J}|{sp}|{v}': d for (c, J, sp, v), d in D.items()}),
              open(os.path.join(HERE, f'out_{tag}.json'), 'w'))
    summarize(tag, ctrls, D)
    print(f'total {time.time()-t0:.0f}s')


def summarize(tag, ctrls, D):
    for metric in ('e23', 'exdev', 'esterr'):
        wins = {c: 0 for c in ctrls}; ranks = {c: 0.0 for c in ctrls}; n = 0
        perJ = {J: {c: 0 for c in ctrls} for J in JS}
        for J in JS:
            for v in range(len(MV)):
                for sp in SPS:
                    vals = {c: float(np.mean(D[(c, J, sp, v)][metric])) for c in ctrls}
                    order = sorted(vals, key=vals.get)
                    wins[order[0]] += 1; perJ[J][order[0]] += 1
                    for i, c in enumerate(order):
                        ranks[c] += i + 1
                    n += 1
        print(f'[{tag}] metric {metric}: wins ' + ', '.join(f'{c} {wins[c]}' for c in sorted(ctrls, key=lambda c: -wins[c]) if wins[c]))
        print(f'[{tag}] metric {metric}: mean rank ' + ', '.join(f'{c} {ranks[c]/n:.1f}' for c in sorted(ctrls, key=lambda c: ranks[c])))
        for J in JS:
            print(f'    J={J:.0e} wins: ' + ', '.join(f'{c} {perJ[J][c]}' for c in sorted(ctrls, key=lambda c: -perJ[J][c]) if perJ[J][c]))


if __name__ == '__main__':
    main()
