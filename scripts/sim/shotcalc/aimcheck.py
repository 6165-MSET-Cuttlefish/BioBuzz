"""Where a trajectory through ShotSolver's aim point (pentagon centroid, aimDepthIn = 3 in inside) crosses the
opening plane, for the launch angles the good policies use. Vacuum and linear drag, exact constants."""
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import shotcore as sc
import physmodel as pm

res = json.load(open(os.path.join(sc.HERE, "results", "study1.json")))
lines = [f"Allowed band at u=0: w in [{sc.W_LO:.2f}, {sc.W_HI:.2f}] in, centre {sc.W_MID:.2f}; centroid w = {sc.WC:.2f}",
         "w at the plane for a trajectory through the solver aim point (centroid, 3 in deep):"]
out = {}
for policy in ("maxP", "minDistSens", "centre"):
    pts = res["T2_25g_cd.5"]["policies"][policy]
    D = np.array([p["D"] for p in pts], float)
    th = np.array([p["theta"] for p in pts])
    row = {}
    for kd in (0.0, 0.5):
        p = dict(pm.NOMINAL)
        p.update(th0=0.0, th1=1.0, kd=kd, k=1.0)
        pm.AIM["mode"] = "solver"
        v = pm.solve_rpm(p, D, th, lo=50, hi=2000)
        pm.AIM["mode"] = "band"
        t, w, vz = pm.plane_crossing(D, v, th, np.full(D.shape, kd))
        row[f"kd{kd}"] = dict(w=w.round(2).tolist(), vz=vz.round(0).tolist())
        lines.append(f"  {policy:12s} kd={kd}: theta " + " ".join(f"{x:.0f}" for x in th) + " -> w " + " ".join(f"{x:.2f}" for x in w)
                     + f"  (mean offset from band centre {np.mean(w - sc.W_MID):+.2f} in)")
    out[policy] = row
txt = "\n".join(lines)
print(txt)
open(os.path.join(sc.HERE, "results", "aimcheck.txt"), "w").write(txt)
