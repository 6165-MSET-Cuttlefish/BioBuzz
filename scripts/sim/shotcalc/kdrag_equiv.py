"""Linear-drag rate kDragPerS that makes ShotSolver.dragTime(tof, k) equal the true lead time, on the maxP / minDist shots."""
import json, os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import shotcore as sc
from evalcore import Truth

r = json.load(open(os.path.join(sc.HERE, "results", "study1.json")))
lines = ["kDragPerS equivalent (1/s) per D (maxP policy shots | minDistSens shots):"]
for n in sc.MAIN_TRUTHS:
    T = Truth(n)
    out = []
    for pol in ("maxP", "minDistSens"):
        pts = r[n]["policies"][pol]
        D = np.array([q["D"] for q in pts], float); rpm = np.array([q["rpm"] for q in pts]); h = np.array([q["hood"] for q in pts])
        a = T.at(D, rpm, h, keys=("tof", "lat"))
        ks = []
        for t, l in zip(a["tof"], a["lat"]):
            lo, hi = 0.0, 20.0
            for _ in range(60):
                m = 0.5 * (lo + hi)
                f = (1 - np.exp(-m * t)) / m if m > 0 else t
                lo, hi = (m, hi) if f > l else (lo, m)
            ks.append(0.5 * (lo + hi))
        out.append(" ".join(f"{k:.2f}" for k in ks) + f"  (tof-lat {1000*np.mean(a['tof']-a['lat']):.0f} ms)")
    lines.append(f"  {n:20s} " + " | ".join(out))
txt = "\n".join(lines)
print(txt)
open(os.path.join(sc.HERE, "results", "kdrag_equiv.txt"), "w").write(txt)
