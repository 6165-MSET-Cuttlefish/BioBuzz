"""Sensitivity of the 10-ball burst result to unmeasured plant values: kt (=ke, +23 % torque), load friction
(0.004 / 0.016 N.m at 3000 rpm vs 0.008), battery 12.0 / 13.5 V (vs 12.5), contact over hood+guides wrap.
Same targets and tuned gains as the baseline. Writes results/sens.json and appends to results_tables.txt."""
import json, os, sys, time
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import numpy as np
import flysim_cad as F
import cadcfg as C
import run_cad as R
import tables_cad as T

VARIANTS = {
    'base': {},
    'kt=ke': dict(ktm=F.KE),
    'fric.004': dict(fric=0.004),
    'fric.016': dict(fric=0.016),
    'V12.0': dict(Voc=12.0),
    'V13.5': dict(Voc=13.5),
    'wrapmax': dict(wrap=True),
}


def main():
    workers = int(sys.argv[1]) if len(sys.argv) > 1 else 8
    best = json.load(open(os.path.join(R.OUT, 'tune.json')))
    cms = [('oracle', 'none', 0.02), ('team', 'hub', 0.02), ('rect', 'oq', 0.02)]
    rows = []
    for vname, mod in VARIANTS.items():
        for sh in ('pollen', 'nectar'):
            for pk in ('asbuilt', 'J2.4'):
                pl = R.plants(sh)[pk]
                for cm in cms:
                    g = R.gains_for(best, sh, pk, *cm)
                    for d in T.DS:
                        for drag in C.DRAG:
                            for sp in R.INTERVALS:
                                for sd in R.SEEDS:
                                    r = R.row(sh, d, 0.45, drag, pk, cm[0], cm[1], cm[2], sp, sd, g)
                                    for k, v in mod.items():
                                        if k != 'wrap':
                                            r[k] = v
                                    if 'wrap' in mod:
                                        v = C.SHOTS[sh][d]['v_in_s'] * C.IN * drag
                                        r['Tc'] = C.contact_time(sh, v, pl['r'], C.HOOD[sh]['wrap_max'])
                                    wf = F.free_speed(r['Voc'], 0.1, r['fric'], r['nmot'], r['g'], r['ktm'])
                                    r['frac'] = r['tgt'] / wf
                                    r['pk'] = f'{pk}~{vname}'
                                    rows.append(r)
    t0 = time.time()
    out, sk = R.execute(rows, R.NB, 0.4, workers)
    json.dump(dict(rows=out, skipped=sk), open(os.path.join(R.OUT, 'sens.json'), 'w'))
    A = T.analyse_bursts(out, sk)
    L = ['', '== SENSITIVITY to unmeasured plant values: fastest 10-ball interval (ms), reference window, f=0.45; cells 36/60/84/108/132 in',
         '   (same targets and gains as baseline; kt=ke is +23 % torque; inf = target beyond free speed for that variant)']
    for drag in C.DRAG:
        L.append(f'-- drag x{drag:.2f}')
        for sh in ('pollen', 'nectar'):
            for pk in ('asbuilt', 'J2.4'):
                for cm in cms:
                    cells = []
                    for vname in VARIANTS:
                        cells.append(f'{vname}: ' + ' '.join(T.cell(A, T.K(sh, f'{pk}~{vname}', d, 0.45, drag, cm), 'fast_v_10').strip() for d in T.DS))
                    L.append(f'{sh:6s} {pk:7s} {T.CM_LABEL[cm]:17s} | ' + ' | '.join(cells))
    open(os.path.join(HERE, 'results_tables.txt'), 'a').write('\n'.join(L) + '\n')
    json.dump({k: {kk: vv for kk, vv in v.items() if kk.startswith('fast') or kk in ('tgt_rpm', 'frac', 'infeasible')} for k, v in A.items()},
              open(os.path.join(R.OUT, 'sens_summary.json'), 'w'), indent=1, default=lambda o: None)
    print('\n'.join(L))
    print(f'sens {len(out)} sims {len(sk)} skipped {time.time()-t0:.0f}s')


if __name__ == '__main__':
    main()
