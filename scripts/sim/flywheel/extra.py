"""Supplementary sweeps: voltage-sensor lag under battery sag; physics floor vs target rpm. Appends to results.json['extra']."""
import os, sys, json
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import numpy as np
import run_all as R
from concurrent.futures import ProcessPoolExecutor


def job(a):
    kind, ctrl, J, gains, over, scen_kw, tag = a
    if kind == 'burst':
        rows, res = R.run(ctrl, J, [gains], R.EVAL_SEEDS, R.scen_burst(J, 0.25, **scen_kw), over)
    elif kind == 'single':
        rows, res = R.run(ctrl, J, [gains], R.EVAL_SEEDS, R.scen_burst(J, 1.0, nb=1, **scen_kw), over)
    else:
        rows, res = R.run(ctrl, J, [gains], R.EVAL_SEEDS, R.scen_spin(J, **scen_kw), over)
    d = R.agg(rows, res, 1, 1)[(0, 0)]
    return dict(kind=kind, ctrl=ctrl, J=J, tag=tag, **d)


def main():
    resf = os.path.join(HERE, 'results.json')
    RR = json.load(open(resf))
    best = {(d['ctrl'], d['J']): d['gains'] for d in RR['tune_base']}
    tasks = []
    ctrls = ['ffpi', 'ffpiz', 'ffp', 'hybrid', 'boost', 'pred', 'hub', 'oracle']
    for J in (1e-4, 5e-4):
        for c in ctrls:
            for vtag, vkw in (('vs_instant', dict(vs_tau=0.0)), ('vs_tau100ms', dict(vs_tau=0.1)),
                              ('vs_hold750ms', dict(vs_hold=0.75)), ('vs_hold250ms', dict(vs_hold=0.25))):
                kw = dict(vkw, sag=(25.0, R.WARM - 0.1, R.WARM + 1.0))
                tasks.append(('burst', c, J, best[(c, J)], dict(Rint=0.15), kw, 'sag25A_' + vtag))
                tasks.append(('burst', c, J, best[(c, J)], {}, vkw, 'nosag_' + vtag))
    for J in (1e-4, 5e-4):
        for c in ('oracle', 'ffpiz', 'pred', 'boost'):
            for tr in (2500.0, 3000.0, 3500.0, 4000.0, 4500.0):
                tasks.append(('single', c, J, best[(c, J)], {}, dict(target_rpm=tr, w0_rpm=tr), f'target{int(tr)}'))
                tasks.append(('burst', c, J, best[(c, J)], {}, dict(target_rpm=tr, w0_rpm=tr), f'target{int(tr)}'))
    with ProcessPoolExecutor(max_workers=10) as ex:
        out = list(ex.map(job, tasks))
    RR = json.load(open(resf))
    RR['extra'] = out
    json.dump(RR, open(resf, 'w'), indent=1)
    lines = []
    lines.append('=== VOLTAGE SENSING under 25 A drivetrain sag (Rint 0.15 ohm, starts 100 ms before ball 1): e1 (signed) / mean(|e2|,|e3|) rpm; no-sag mean(|e2|,|e3|) ===')
    for J in (1e-4, 5e-4):
        lines.append(f'--- J={J:.0e} --- cols: instant | 100 ms filter | 250 ms sample-hold | 750 ms sample-hold (= 50 loops x 15 ms)')
        for c in ctrls:
            row = f'{c:7s}'
            for v in ('vs_instant', 'vs_tau100ms', 'vs_hold250ms', 'vs_hold750ms'):
                a = next(d for d in out if d['ctrl'] == c and d['J'] == J and d['tag'] == 'sag25A_' + v)
                b = next(d for d in out if d['ctrl'] == c and d['J'] == J and d['tag'] == 'nosag_' + v)
                row += f" | sag e1 {a['e1']:5.0f} e23 {0.5*(a['abs_e2']+a['abs_e3']):4.0f}  nosag e23 {0.5*(b['abs_e2']+b['abs_e3']):3.0f}"
            lines.append(row)
    lines.append('')
    lines.append('=== TARGET RPM vs recovery (30 g, 20 ms contact; gains tuned at 3000): single-shot dip rpm / rec50 s ; burst 250 ms mean(|e2|,|e3|) rpm ===')
    for J in (1e-4, 5e-4):
        lines.append(f'--- J={J:.0e} --- cols: 2500 | 3000 | 3500 | 4000 | 4500 rpm')
        for c in ('oracle', 'ffpiz', 'pred', 'boost'):
            row = f'{c:7s}'
            for tr in (2500, 3000, 3500, 4000, 4500):
                a = next(d for d in out if d['ctrl'] == c and d['J'] == J and d['tag'] == f'target{tr}' and d['kind'] == 'single')
                b = next(d for d in out if d['ctrl'] == c and d['J'] == J and d['tag'] == f'target{tr}' and d['kind'] == 'burst')
                rec = 'nr' if a.get('rec50_nanfrac', 0) > 0 else f"{a['rec50']:.3f}"
                row += f" | {a['dip']:5.0f} {rec:>5s} {0.5*(b['abs_e2']+b['abs_e3']):4.0f}"
            lines.append(row)
    txt = '\n'.join(lines) + '\n'
    open(os.path.join(HERE, 'results_extra.txt'), 'w').write(txt)
    print(txt)


if __name__ == '__main__':
    main()
