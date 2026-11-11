"""Joules per ball drawn from the wheel, net motor power above friction at the target speed,
and the sustained fire rate that power supports with no margin. Writes results/energy.json."""
import json, os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import numpy as np
import flysim_cad as F
import cadcfg as C
import run_cad as R

out = []
for sh in ('pollen', 'nectar'):
    for pk in ['asbuilt', 'J2.4', '2mot', 'w96', 'g1.25', 'g1.5']:
        pl = R.plants(sh)[pk]
        for d in (36, 60, 84, 108, 132):
            for f in C.F_EXIT:
                for drag in C.DRAG:
                    ti = R.tgt_info(sh, d, f, drag, pk)
                    w = ti['tgt']
                    if ti['frac'] >= 0.995:
                        out.append(dict(sh=sh, pk=pk, d=d, f=f, drag=drag, tgt_rpm=w * F.RPM, frac=ti['frac'], feasible=False))
                        continue
                    row = R.row(sh, d, f, drag, pk, 'ff', 'none', 0.02, 1.0, 1, {})
                    res = R.run_batch(([row], 'ff', 'none', 1, R.WARM + ti['Tc'] + 0.05))[0]
                    E = res['e_ball1']
                    m = C.BALL[sh]['m']
                    v = ti['v_exit_ms']
                    ke_ball = F.SHELL * m * v * v
                    rec = dict(sh=sh, pk=pk, d=d, f=f, drag=drag, tgt_rpm=w * F.RPM, frac=ti['frac'], feasible=True,
                               E_ball_J=E, ball_KE_J=ke_ball, ratio=E / ke_ball)
                    for V in (12.0, 12.5, 13.5):
                        Pa = F.avail_power(w, V, 0.1, pl['fric'], pl['nmot'], pl['g'], pl['ktm'])
                        Pt = F.avail_power(w, V, 0.1, pl['fric'], pl['nmot'], pl['g'], pl['ktm'], vcap=13.0)
                        rec[f'P_{V}'] = Pa
                        rec[f'rate_{V}'] = Pa / E if Pa > 0 else 0.0
                        rec[f'P13cap_{V}'] = Pt
                        rec[f'rate13cap_{V}'] = Pt / E if Pt > 0 else 0.0
                    out.append(rec)
json.dump(out, open(os.path.join(HERE, 'results', 'energy.json'), 'w'), indent=1)
for r in out:
    if r['pk'] == 'asbuilt' and r['drag'] == 1.0:
        if not r['feasible']:
            print(f"{r['sh']} {r['d']} f={r['f']}: target {r['tgt_rpm']:.0f} rpm = {100*r['frac']:.0f}% of free: infeasible")
            continue
        print(f"{r['sh']} {r['d']} f={r['f']}: tgt {r['tgt_rpm']:.0f} ({100*r['frac']:.0f}%) E {r['E_ball_J']:.2f} J (KE {r['ball_KE_J']:.2f}, x{r['ratio']:.2f}) "
              f"P 12.0/12.5/13.5 = {r['P_12.0']:.1f}/{r['P_12.5']:.1f}/{r['P_13.5']:.1f} W -> {r['rate_12.0']:.1f}/{r['rate_12.5']:.1f}/{r['rate_13.5']:.1f} balls/s "
              f"(13 V cap at 13.5: {r['rate13cap_13.5']:.1f})")
