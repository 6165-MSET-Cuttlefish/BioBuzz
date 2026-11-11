"""Drag-free closed form vs the study's drag-free npz on the CAD geometry: v_aim, TOF to x = D, and descent at x = D,
over every in-travel angle and distance (read only)."""
import math, os
import numpy as np
G, ZAIM = 386.09, 56.81508
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "out")
EX = {"pollen": ((1.56, 12.46), (1.99, 13.83), 30.0), "nectar": ((1.23, 12.53), (1.60, 13.57), 20.0)}
for sh in ("pollen", "nectar"):
    for base in ("geo", "cal"):
        d = np.load(os.path.join(OUT, f"raw_{sh}_{base}_none.npz"))
        (x0a, z0a), (x0b, z0b), mm = EX[sh]
        wv = wt = wd = 0.0
        for i, D in enumerate(d["DS"]):
            for j, th in enumerate(d["TH"]):
                m = th - d["base"]
                if not (0 <= m <= mm) or not np.isfinite(d["v"][i, j]):
                    continue
                x0 = x0a + m / mm * (x0b - x0a); z0 = z0a + m / mm * (z0b - z0a)
                t = math.radians(th); dd = D - x0; h = ZAIM - z0
                v = math.sqrt(G * dd * dd / (2 * math.cos(t) ** 2 * (dd * math.tan(t) - h)))
                tof = dd / (v * math.cos(t))
                vz = v * math.sin(t) - G * tof
                desc = math.degrees(math.atan2(-vz, v * math.cos(t)))
                wv = max(wv, abs(v - d["v"][i, j])); wt = max(wt, abs(1000 * (tof - d["ta"][i, j])))
                wd = max(wd, abs(desc - d["descent_aim"][i, j]))
        print(f"{sh} {base} none: max |dv| {wv:.4f} in/s, |dTOF| {wt:.3f} ms, |d descent@aim| {wd:.4f} deg")
