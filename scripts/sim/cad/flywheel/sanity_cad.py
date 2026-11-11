"""Plant checks: motor constants, free speed, momentum transfer vs J/(J+k), energy per ball vs k w^2, required rpm."""
import json, math, os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
import flysim_cad as F
import cadcfg as C

out = {}
print(f"motor: R={F.RM:.4f} ohm kt(stall)={F.KT_STALL:.5f} N.m/A ke={F.KE:.5f} V.s/rad (free {F.W_FREE_RPM:.0f} rpm) kt/ke={F.KT_STALL/F.KE:.3f}")
out['motor'] = dict(R=F.RM, kt=F.KT_STALL, ke=F.KE, free_rpm_spec=F.W_FREE_RPM)
print(f"J: design {C.J_DESIGN:.3e} rotor {C.J_ROTOR:.1e} roller {C.J_ROLLER:.3e} -> as-built pollen {C.J_ASBUILT['pollen']:.3e} nectar {C.J_ASBUILT['nectar']:.3e}")
print(f"   steel 82 mm {C.J_STEEL82:.3e} (old listing {C.J_STEEL82_OLD:.1e}); 96 mm Rhino wheel est {C.J_WHEEL96:.3e} (72 mm I/(m r^2)={C.K72:.3f})")
out['J'] = dict(design=C.J_DESIGN, rotor=C.J_ROTOR, roller=C.J_ROLLER, asbuilt=C.J_ASBUILT, steel82=C.J_STEEL82, wheel96=C.J_WHEEL96)
fs = {}
for V in (12.0, 12.5, 13.5):
    for fr in (0.004, 0.008, 0.016):
        w = F.free_speed(V, 0.1, fr)
        fs[f'{V}_{fr}'] = w * F.RPM
print("loaded free speed rpm (Rint 0.1) V x fric 0.004/0.008/0.016:")
for V in (12.0, 12.5, 13.5):
    print(f"  {V}: " + " / ".join(f"{fs[f'{V}_{fr}']:.0f}" for fr in (0.004, 0.008, 0.016)))
out['free_rpm'] = fs

# momentum transfer check: no friction, Rint 0, FF-off (p=0 float) -> exit ratio should be J/(J+k)
rows = []
for J in (0.83e-4, 1.02e-4, 2.4e-4):
    for sh in ('pollen', 'nectar'):
        kb = C.kball(C.BALL[sh]['m'], 0.45, C.R_WHEEL)
        P = dict(J=J, m=C.BALL[sh]['m'], Tc=0.001, Voc=12.5, Rint=0.0, fric=1e-12, T=0.015, W=0.05, tsA=0, seed=1,
                 tgt=4000 / F.RPM, r=C.R_WHEEL, f=0.45, nmot=1, g=1.0, ktm=F.KT_STALL, Nq=0.02, lat_lo=0, lat_hi=0,
                 kP=0, kI=0, iz=0, thr=0, spacing=1.0, Voc_=0)
        P.pop('Voc_')
        r = F.simulate(P, 'ff', 'hub', nb=1, warm=0.05, T_end=0.08, shot_jit=0.0)
        ratio = r['w_exit'][0, 0] / 4000
        E = r['e_ball'][0, 0]
        w = 4000 / F.RPM
        rows.append(dict(J=J, sh=sh, ratio=float(ratio), JJk=J / (J + kb), E=float(E), k_w2=kb * w * w * ratio))
        print(f" J={J:.2e} {sh}: exit/target {ratio:.4f} (Tc 1 ms, no friction) vs J/(J+k) {J/(J+kb):.4f}; E_ball {E:.3f} J vs k w w_exit {kb*w*w*ratio:.3f} J")
out['momentum_check'] = rows

print("\nrequired targets (f=0.45, drag 1.0, as-built J):")
req = []
for sh in ('pollen', 'nectar'):
    for d in (36, 60, 84, 108, 132):
        for f in C.F_EXIT:
            for dr in (1.0, 1.07, 1.14):
                w, info = C.target(sh, d, f, dr, C.J_ASBUILT[sh])
                req.append(dict(sh=sh, d=d, f=f, drag=dr, rpm=w * F.RPM, frac=w / F.free_speed(12.5, 0.1, 0.008), **info))
for q in req:
    if q['drag'] == 1.0 and q['f'] == 0.45:
        print(f" {q['sh']} {q['d']}: v {q['v_exit_ms']:.2f} m/s, w_exit {q['w_exit_rpm']:.0f}, rho {q['rho']:.4f}, Tc {1000*q['Tc']:.1f} ms -> target {q['rpm']:.0f} rpm ({100*q['frac']:.0f}% of loaded free)")
out['req'] = req
json.dump(out, open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'sanity_cad.json'), 'w'), indent=1)
