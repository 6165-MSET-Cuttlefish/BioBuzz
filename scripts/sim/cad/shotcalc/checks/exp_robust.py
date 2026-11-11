"""Stability of schedule i and of the hood-floor benefit to the robust-pick thresholds [asm: hood >= +-2 deg,
distance >= +-3 in]. Reads the study's raw npz (read only). Also lists which side of the distance window binds."""
import os
import numpy as np
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "out")
MMAX = {"pollen": 30.0, "nectar": 20.0}

for sh in ("pollen", "nectar"):
    d = np.load(os.path.join(OUT, f"raw_{sh}_geo_nom.npz"))
    m = d["TH"] - d["base"]
    trav = (m > -1e-6) & (m < MMAX[sh] + 1e-6)
    print(f"== {sh} geo nom")
    for hmin, dmin in ((0, 0), (1, 2), (1.5, 3), (2, 3), (2.5, 3), (2, 4), (3, 4)):
        cells, fl = [], []
        for i, D in enumerate(d["DS"]):
            ok = d["ok"][i]
            rob = ok & (d["h_lo"][i] <= -hmin) & (d["h_hi"][i] >= hmin) & (d["d_lo"][i] <= -dmin) & (d["d_hi"][i] >= dmin)
            sel = rob & trav
            if sel.any():
                j = np.where(sel)[0][int(np.nanargmax(d["s_w"][i][sel]))]
                cells.append(f"{D}:{m[j]:.1f}({100 * d['s_w'][i, j]:.2f})")
            else:
                cells.append(f"{D}:none")
            if D == 132:
                for flo in (46.0, 50.0, float(d["base"])):
                    s2 = rob & (d["TH"] >= flo - 1e-6) & (m <= MMAX[sh] + 1e-6)
                    if s2.any():
                        j = np.where(s2)[0][int(np.nanargmax(d["s_w"][i][s2]))]
                        fl.append(f"floor {flo:.1f}: {100 * d['s_w'][i, j]:.2f}% @{d['TH'][j]:.1f} tof {1000 * d['ta'][i, j]:.0f}")
                    else:
                        fl.append(f"floor {flo:.1f}: none")
        print(f"  hood>=+-{hmin} dist>=+-{dmin}: " + " ".join(cells) + "  | 132 in " + "; ".join(fl))
    print("  binding side of the distance window on schedule (i) [near = target closer than believed]:")
    sched = {"pollen": [15, 8, 2, 0, 0, 0, 0, 0, 0], "nectar": [11.5, 4.5, 0, 0, 0, 0, 0, 0, 0]}[sh]
    cells = []
    for i, D in enumerate(d["DS"]):
        j = int(np.argmin(np.abs(m - sched[i])))
        lo, hi = d["d_lo"][i, j], d["d_hi"][i, j]
        cells.append(f"{D}:{lo:+.1f}/{hi:+.1f} {'near' if -lo < hi else 'far'}")
    print("   " + "  ".join(cells))
