"""Misc checks: (a) I-zone vs same-gain pinned-hold PI; (b) oracle infeasibility boundary vs kt; (c) DT convergence."""
import os, sys, json
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import numpy as np
import flysim as F
import run_all as R

SEEDS = list(range(401, 409))
what = sys.argv[1]


def burst(ctrl, J, g, sp, over=None, m=0.03, tc=0.015):
    rows, res = R.run(ctrl, J, [g], SEEDS, R.scen_burst(J, sp), dict(m=m, Tc=tc, **(over or {})))
    et = res['e_touch']; we = res['w_exit']
    return dict(e2=np.mean(et[:, 1]), e3=np.mean(et[:, 2]), ae23=np.mean(0.5 * (np.abs(et[:, 1]) + np.abs(et[:, 2]))),
                exdev=np.mean(np.abs(we[:, 1:] - we[:, :1]) / we[:, :1]) * 100, rec50=np.nanmean(res['rec50']),
                dip=np.mean(res['dip']), bias=np.mean(res['ss_bias']))


def spin(ctrl, J, g, over=None):
    rows, res = R.run(ctrl, J, [g], SEEDS, R.scen_spin(J), over or {})
    return dict(ts=np.nanmean(res['t_settle50']), tsnan=np.mean(~np.isfinite(res['t_settle50'])), os=np.mean(res['overshoot']),
                bias=np.mean(res['ss_bias']), std=np.mean(res['ss_std']))


f = lambda d: ' '.join(f'{k}={v:.3g}' for k, v in d.items())
if what == 'izone':
    J = 5e-5
    for c, g in (('ffpiz', dict(kP=0.005, kI=0.03, iz=100)), ('ffpi', dict(kP=0.005, kI=0.03)), ('ffpi', dict(kP=0.005, kI=0.0)),
                 ('ffpi', dict(kP=0.01, kI=0.01)), ('ffpi', dict(kP=0.01, kI=0.0))):
        for ffk in (1.0, 0.9, 1.1):
            print(f'J=5e-5 {c} {g} ffk={ffk}: b150 ' + f(burst(c, J, g, 0.15, dict(ffk=ffk))) + ' | spin ' + f(spin(c, J, g, dict(ffk=ffk))), flush=True)
if what == 'oracle':
    print('KT', F.KT)
    for J in (5e-5, 1e-4, 2e-4, 5e-4):
        for m, tc in ((0.06, 0.015), (0.06, 0.03), (0.03, 0.015)):
            b = burst('oracle', J, {}, 0.15, m=m, tc=tc)
            print(f'oracle J={J:.0e} m={m*1e3:.0f}g Tc={tc*1e3:.0f}ms 150ms: ae23={b["ae23"]:.0f} dip={b["dip"]:.0f}', flush=True)
if what == 'dt':
    print('DT', F.DT)
    base = {(d['ctrl'], d['J']): d['gains'] for d in json.load(open(os.path.join(HERE, '..', 'results.json')))['tune_base']}
    for J in (5e-5, 1e-4):
        for c in ('oracle', 'ffpiz', 'boost', 'kf'):
            b = burst(c, J, base[(c, J)], 0.15)
            s = spin(c, J, base[(c, J)])
            print(f'DT={F.DT} J={J:.0e} {c}: ' + f(b) + ' | ' + f(s), flush=True)
