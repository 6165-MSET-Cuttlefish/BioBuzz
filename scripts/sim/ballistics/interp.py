import math
import os
import sys
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from geom import *  # noqa

h = AIM[2] - EXIT_Z


def vmin(d):
    r = np.hypot(d, h)
    return np.sqrt(G * (h + r)), np.arctan((h + r) / d)


def for_descent(d, beta_deg):
    tb = math.tan(math.radians(beta_deg))
    tth = 2 * h / d + tb
    vx2 = G * d / (2 * h / d + 2 * tb)
    return np.sqrt(vx2 * (1 + tth * tth)), np.arctan(tth)


def dz(d, v, th):
    return d * np.tan(th) - G * d * d / (2 * v * v * np.cos(th) ** 2) - h


dd = np.linspace(36, 132, 9601)
rules = {"vmin": vmin}
for b in (30, 45, 55):
    rules[f"entry{b}"] = (lambda bb: (lambda d: for_descent(d, bb)))(b)
nodesets = {"5 pts (24 in)": [36, 60, 84, 108, 132], "4 pts (32 in)": [36, 68, 100, 132], "3 pts (48 in)": [36, 84, 132]}
for name, rule in rules.items():
    v, th = rule(dd)
    # linear-fit residual
    A = np.vstack([dd, np.ones_like(dd)]).T
    coef, *_ = np.linalg.lstsq(A, v, rcond=None)
    res = v - A @ coef
    print(f"\n{name}: v 36->132 = {v[0]:.1f}->{v[-1]:.1f} in/s, theta {math.degrees(th[0]):.2f}->{math.degrees(th[-1]):.2f} deg; best straight line max dev {100*np.max(np.abs(res)/v):.2f}% of v")
    for nn, nodes in nodesets.items():
        nv, nth = rule(np.array(nodes, dtype=float))
        vi = np.interp(dd, nodes, nv)
        thi = np.interp(dd, nodes, nth)
        ev = (vi - v) / v
        eth = np.degrees(thi - th)
        miss_v_only = dz(dd, vi, th)
        miss_both = dz(dd, vi, thi)
        i = np.argmax(np.abs(ev))
        print(f"  {nn}: speed interp err max {100*np.max(np.abs(ev)):.3f}% (at {dd[i]:.0f} in, sign {np.sign(ev[i]):+.0f}); angle interp err max {np.max(np.abs(eth)):.3f} deg; "
              f"vertical miss from speed err {np.min(miss_v_only):+.3f}/{np.max(miss_v_only):+.3f} in; with both interpolated {np.min(miss_both):+.3f}/{np.max(miss_both):+.3f} in")
