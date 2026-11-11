"""Controller checks: windup attribution, KF process-noise sensitivity, team-code fidelity
(750 ms voltage hold + WriteCache 0.005)."""
import os, sys, json
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import numpy as np
import flysim as F
import run_all as R

SEEDS = list(range(401, 409))
base = {(d['ctrl'], d['J']): d['gains'] for d in json.load(open(os.path.join(HERE, '..', 'results.json')))['tune_base']}
what = sys.argv[1]


def burst(ctrl, J, g, sp, scen_kw=None, over=None, m=0.03, tc=0.015):
    rows, res = R.run(ctrl, J, [g], SEEDS, R.scen_burst(J, sp, **(scen_kw or {})), dict(m=m, Tc=tc, **(over or {})))
    et = res['e_touch']
    we = res['w_exit']
    return dict(e2=np.mean(et[:, 1]), e3=np.mean(et[:, 2]), ae23=np.mean(0.5 * (np.abs(et[:, 1]) + np.abs(et[:, 2]))),
                e1=np.mean(et[:, 0]), exdev=np.mean(np.abs(we[:, 1:] - we[:, :1]) / we[:, :1]) * 100,
                rec50=np.nanmean(res['rec50']), rec50_nan=np.mean(~np.isfinite(res['rec50'])),
                ssstd=np.mean(res['ss_std']), est=np.mean(np.abs(res['est_err_touch'][:, 1:])),
                meas=np.mean(np.abs(res['meas_err_touch'][:, 1:])))


def spin(ctrl, J, g, scen_kw=None, over=None):
    rows, res = R.run(ctrl, J, [g], SEEDS, R.scen_spin(J, **(scen_kw or {})), over or {})
    return dict(ts=np.nanmean(res['t_settle50']), tsnan=np.mean(~np.isfinite(res['t_settle50'])), os=np.mean(res['overshoot']),
                bias=np.mean(res['ss_bias']), std=np.mean(res['ss_std']), irms=np.mean(res['I_rms']))


f = lambda d: ' '.join(f'{k}={v:.3g}' for k, v in d.items())

if what == 'windup':
    for J in (5e-5, 1e-4):
        g = base[('ffpi', J)]
        for lab, gg in (('tuned', g), ('kI=0', dict(g, kI=0)), ('ffpiz', base[('ffpiz', J)])):
            c = 'ffpiz' if lab == 'ffpiz' else 'ffpi'
            print(f'J={J:.0e} {c} {lab} {gg}:  burst150 ' + f(burst(c, J, gg, 0.15)) + ' | spin ' + f(spin(c, J, gg)))

if what == 'kfqw':
    print('QW env', F.QW)
    for J in (5e-5, 1e-4, 5e-4):
        g = base[('kf', J)]
        for kp in (g['kP'] / 2, g['kP'], g['kP'] * 2):
            for qd in (1e4, 1e6):
                gg = dict(g, kP=kp, qd=qd)
                b1 = burst('kf', J, gg, 0.15)
                b2 = burst('kf', J, gg, 0.25)
                s = spin('kf', J, gg)
                print(f'QW={F.QW:g} J={J:.0e} kP={kp:g} qd={qd:g}: b150 ae23={b1["ae23"]:.0f} b250 ae23={b2["ae23"]:.0f} exdev250={b2["exdev"]:.2f} '
                      f'rec50={b2["rec50"]:.3f}(nan {b2["rec50_nan"]:.2f}) est_err={b2["est"]:.1f} win_meas_err={b2["meas"]:.1f} | spin ts={s["ts"]:.3f} nan={s["tsnan"]:.2f} std={s["std"]:.1f}', flush=True)

if what == 'team':
    # team code: voltage read every 50 loops (sample-hold ~750 ms), WriteCache tolerance 0.005 power
    for J in (5e-5, 1e-4, 5e-4):
        for c in ('ffpi', 'ffpiz', 'ffp'):
            g = base[(c, J)]
            for lab, kw in (('default(100ms filt)', {}), ('hold750', dict(vs_hold=0.75)), ('hold750+wc.005', dict(vs_hold=0.75, wc_tol=0.005)),
                            ('filt+wc.005', dict(wc_tol=0.005))):
                b = burst(c, J, g, 0.25, kw)
                s = spin(c, J, g, kw)
                print(f'J={J:.0e} {c:5s} {lab:16s}: burst250 ae23={b["ae23"]:.1f} exdev={b["exdev"]:.2f}% rec50={b["rec50"]:.3f} ssstd(true)={b["ssstd"]:.1f} | spin ts={s["ts"]:.3f} nan={s["tsnan"]:.2f} bias={s["bias"]:.1f} std={s["std"]:.1f} Irms={s["irms"]:.2f}', flush=True)

if what == 'est':
    # observer estimate vs windowed measurement at touch (what a hood-follows-measured-rpm scheme would see)
    for J in (5e-5, 1e-4, 5e-4):
        for c in ('ffpiz', 'pred', 'kf', 'hub'):
            g = base[(c, J)]
            for ts in (0.0, 0.5e-3, 1.5e-3):
                b = burst(c, J, g, 0.25, over=dict(tsA=ts))
                print(f'J={J:.0e} {c:5s} ts+-{ts*1e3:.1f}ms: |window meas - true| at touch b2,3 = {b["meas"]:.1f} rpm; |controller estimate - true| = {b["est"]:.1f} rpm', flush=True)
