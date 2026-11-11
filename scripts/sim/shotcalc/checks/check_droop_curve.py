"""Inspect Study 3's hood-vs-deficit curves where the 'lin dev' metric is largest: centre hood, half-width, branch."""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import shotcore as sc
import tuner as tu
import study3 as s3
from evalcore import Truth

H = sc.H_GRID
L = []
for name, Ds in (("T1_vac", (72, 84, 96)), ("T2_10g_cd.6", (60, 72, 96))):
    T = Truth(name)
    for D in Ds:
        site = tu.CalSite(T, D)
        i, j = site.ref["maxP"]
        r0, h0 = float(sc.R_GRID[i]), float(H[j])
        h_prev, connected = h0, True
        pts = []
        for dlt in range(0, 301, 20):
            c, hw, hb = s3.best_hood(T, D, r0 - dlt, h_prev)
            tag = ""
            if connected and np.isfinite(c):
                h_prev = c
            elif np.isfinite(c):
                tag = "*"
            else:
                connected = False
            if not np.isfinite(c):
                connected = False
            pts.append(f"{dlt}:{float(sc.hood_to_theta(c)) if np.isfinite(c) else float('nan'):.1f}{tag}/{50*hw:.1f}")
        L.append(f"{name} D={D} maxP theta0={float(sc.hood_to_theta(h0)):.1f}  deficit:centreTheta/halfwidth(deg~)  * = after connection lost")
        L.append("   " + " ".join(pts))
txt = "\n".join(L)
print(txt)
open(os.path.join(HERE, "check_droop_curve.txt"), "w").write(txt)
