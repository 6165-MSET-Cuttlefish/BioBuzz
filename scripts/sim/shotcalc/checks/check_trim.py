"""Additive vs multiplicative single trim at 84 in after an exit-speed gain change of 5% and 10%, on the oracle maxP
shots (no tuner noise): P15 at each D with the worn gain."""
import json, os, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import shotcore as sc
from evalcore import Truth
r1 = json.load(open(os.path.join(os.path.dirname(HERE), "results", "study1.json")))
L = []
for name in ("T1_vac", "T2_25g_cd.5", "T2_10g_cd.6"):
    T = Truth(name)
    pts = r1[name]["policies"]["maxP"]
    D = np.array([p["D"] for p in pts], float); r = np.array([p["rpm"] for p in pts]); h = np.array([p["hood"] for p in pts])
    r84 = float(np.interp(84, D, r))
    for wear in (0.95, 0.90):
        g = wear * sc.K_GAIN
        add = r + (r84 / wear - r84)
        mul = r / wear
        Pa = T.P(D, add, h, gain=g); Pm = T.P(D, mul, h, gain=g)
        L.append(f"{name:12s} gain x{wear}: additive rpm error at 36/132 = {add[0]-mul[0]:+.0f}/{add[-1]-mul[-1]:+.0f}; "
                 f"P15 mul mean {Pm.mean():.3f} min {Pm.min():.3f} | add mean {Pa.mean():.3f} min {Pa.min():.3f} (per D: {' '.join(f'{x:.2f}' for x in Pa)})")
txt = "\n".join(L); print(txt); open(os.path.join(HERE, "check_trim.txt"), "w").write(txt)
