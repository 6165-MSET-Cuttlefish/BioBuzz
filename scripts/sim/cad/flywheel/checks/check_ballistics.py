"""Checks: drag-free shot check with the crossing and entry code of ../../../ballistics (geom.py, cross.py),
re-parametrised for the CAD exit points, hood ranges and ball sizes; compares with ../ballistics.py SHOTS."""
import math, os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', '..', '..', 'ballistics'))
import numpy as np
import geom as Gm
import cross as X
sys.path.insert(0, os.path.dirname(HERE))
import ballistics as B

G = Gm.G
AIMZ = Gm.AIM[2]
print(f"aim z {AIMZ:.4f}")

# 1) the 14 in exit numbers of ../../../ballistics/nodrag.py
X.EXIT_Z = 14.0
for d in B.DS:
    h = AIMZ - 14.0
    r = math.hypot(d, h)
    v, th = math.sqrt(G * (h + r)), math.atan((h + r) / d)
    print(f" 14in d={d}: vmin {v:.2f} th {math.degrees(th):.2f}")

CAD = {'pollen': dict(a0=57.7, mmax=30, x0=1.56, z0=12.46, x1=1.99, z1=13.83, dia=2.8),
       'nectar': dict(a0=60.8, mmax=20, x0=1.23, z0=12.53, x1=1.60, z1=13.57, dia=3.6)}


def ok(dexit, z, v, th, dia):
    X.EXIT_Z = z
    p0 = X.launch_point(dexit, 0.0)
    v0 = X.launch_vel(v, th, 0.0)
    res, before = X.crossing_nodrag(p0, v0)
    if before <= 0 or res is None:
        return False
    t, p, vv = res
    u, w, inc, margin, slack, desc = X.entry_eval(p, vv, dia=dia)
    return slack >= 0 and desc


def window(dexit, z, v, th, dia):
    out = []
    for sgn in (-1, 1):
        lo, hi = 0.0, None
        for x in np.arange(0.0005, 0.2, 0.0005):
            if not ok(dexit, z, v * (1 + sgn * x), th, dia):
                hi = x; break
            lo = x
        for _ in range(30):
            mid = 0.5 * (lo + hi)
            if ok(dexit, z, v * (1 + sgn * mid), th, dia):
                lo = mid
            else:
                hi = mid
        out.append(100 * sgn * lo)
    return out

maxdiff = 0
for sh, s in CAD.items():
    for d in B.DS:
        # fixed point: hood m so that launch = min-speed angle from the exit point at that m
        m = 0.0
        for _ in range(200):
            a = m / s['mmax']
            x = s['x0'] + a * (s['x1'] - s['x0']); z = s['z0'] + a * (s['z1'] - s['z0'])
            dx, h = d - x, AIMZ - z
            thm = math.degrees(math.atan((h + math.hypot(dx, h)) / dx))
            m2 = min(max(thm - s['a0'], 0.0), s['mmax'])
            if abs(m2 - m) < 1e-12:
                break
            m = m2
        a = m / s['mmax']
        x = s['x0'] + a * (s['x1'] - s['x0']); z = s['z0'] + a * (s['z1'] - s['z0'])
        dx, h = d - x, AIMZ - z
        th = math.radians(s['a0'] + m)
        # speed through the aim point at this angle (equals vmin when not pinned)
        v = math.sqrt(G * dx * dx / (2 * math.cos(th) ** 2 * (dx * math.tan(th) - h)))
        vmin = math.sqrt(G * (h + math.hypot(dx, h)))
        lo, hi = window(dx, z, v, th, s['dia'])
        lo14, hi14 = window(dx, z, v, th, 2.8)
        a_ = B.shot(sh, d)
        maxdiff = max(maxdiff, abs(v - a_['v_in_s']))
        print(f"{sh} {d}: m {m:5.2f} launch {math.degrees(th):.2f} pinned {m==0 and thm < s['a0']} v {v:.2f} (+{100*(v/vmin-1):.2f}% vs vmin) "
              f"study v {a_['v_in_s']:.2f} | own-ball window {lo:+.2f}/{hi:+.2f} (study {a_['win_lo_pct']:+.2f}/{a_['win_hi_pct']:+.2f}); "
              f"2.8in-ball window at this exit {lo14:+.2f}/{hi14:+.2f}")
print(f"max |v check - v study| = {maxdiff:.4f} in/s")
