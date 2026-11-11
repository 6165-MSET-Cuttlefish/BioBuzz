"""Sensitivity experiments with the independent solver (indep.py):
(a) stronger topspin lift than the study's Cl -0.30 bound; (b) Nectar exit geometry / base angle perturbed (hood reshaped,
numbers not re-derived); (c) min-speed angle and floor penalty under (a) and (b)."""
import math, sys, os
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import indep as I


def vmin_scan(sh, D, k, kl, base, dz=0.0, dx=0.0, lo=44, hi=80):
    ths = np.arange(lo, hi, 0.25)
    vs = []
    for th in ths:
        x0, z0 = I.exitp(sh, th - base)
        try:
            vs.append(I.vaim(th, x0 + dx, z0 + dz, D, k, kl))
        except ValueError:
            vs.append(np.nan)
    vs = np.array(vs)
    j = int(np.nanargmin(vs))
    return float(ths[j]), float(vs[j])


def speed_window(sh, m, D, k, kl, base, dz=0.0, dx=0.0):
    dia = I.BALL[sh][0]
    th = base + m
    x0, z0 = I.exitp(sh, m)
    x0, z0 = x0 + dx, z0 + dz
    v = I.vaim(th, x0, z0, D, k, kl)
    o = I.fly(v, th, x0, z0, D, k, kl)
    if I.margin_fn(o, dia) < 0:
        return v, None, None
    fs = lambda s: I.margin_fn(I.fly(v * (1 + s), th, x0, z0, D, k, kl), dia)
    return v, 100 * I.edge(fs, -0.002, 0.3), 100 * I.edge(fs, 0.002, 0.3)


def lowest_scoring_m(sh, D, k, kl, base, dz=0.0, dx=0.0):
    dia = I.BALL[sh][0]
    for m in np.arange(0, 25, 0.25):
        th = base + m
        x0, z0 = I.exitp(sh, m)
        v = I.vaim(th, x0 + dx, z0 + dz, D, k, kl)
        if I.margin_fn(I.fly(v, th, x0 + dx, z0 + dz, D, k, kl), dia) >= 0:
            return float(m)
    return None


print("(a) Pollen, geometric base, nominal drag; Cl < 0 = topspin (study bound -0.30)")
dia, mass = I.BALL["pollen"]
k = I.kq(dia, mass, 0.5)
for cl in (0.0, -0.15, -0.30, -0.45, -0.60):
    kl = k * cl / 0.5
    row = []
    for D in (36, 84, 132):
        thv, vv = vmin_scan("pollen", D, k, kl, 57.7)
        v0, lo, hi = speed_window("pollen", 0.0 if D > 36 else 15.0, D, k, kl, 57.7)
        row.append(f"D{D}: th_vmin {thv:5.2f} (m {thv - 57.7:+5.2f}) vmin {vv:6.1f} | sched m {0 if D > 36 else 15}: v {v0:6.1f} w {hi - lo if lo is not None else float('nan'):4.2f}")
    print(f"  Cl {cl:+.2f}: " + "  ||  ".join(row), flush=True)

print("(b) Nectar: base angle and exit point perturbed (hood reshaped after the CAD numbers)")
dia, mass = I.BALL["nectar"]
k = I.kq(dia, mass, 0.5)
for dbase, dz, dx in ((0, 0, 0), (-2, 0, 0), (2, 0, 0), (-4, 0, 0), (0, -0.5, 0), (0, 0.5, 0), (0, 0, 0.5)):
    base = 60.8 + dbase
    mlo36 = lowest_scoring_m("nectar", 36, k, 0.0, base, dz, dx)
    v132, lo, hi = speed_window("nectar", 0.0, 132, k, 0.0, base, dz, dx)
    thv, vv = vmin_scan("nectar", 132, k, 0.0, base, dz, dx)
    print(f"  base {base:5.1f} dz {dz:+.1f} dx {dx:+.1f}: 36 in lowest scoring m {mlo36} | 132 in m=0: v {v132:6.1f} "
          f"window {lo:+.2f}/{hi:+.2f} (w {hi - lo:.2f}) | 132 in min-speed {thv:.2f} deg v {vv:.1f} (floor penalty {100 * (v132 / vv - 1):+.2f}%)", flush=True)
