"""Study 3: best-P hood (what the oracle and droop points use) vs deficit, compared with the centre-of-window curve
on which the reported slope / 'lin dev' metrics are computed. Uses study3.json oracle P and recomputes curves."""
import os, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import shotcore as sc, tuner as tu, study3 as s3
from evalcore import Truth
H = sc.H_GRID
L = []
for name in ("T1_vac", "T2_10g_cd.6", "T2_25g_cd.5"):
    T = Truth(name)
    for D in (36, 48, 60, 72, 84, 96, 108, 120, 132):
        site = tu.CalSite(T, D)
        i, j = site.ref["maxP"]
        r0, h0 = float(sc.R_GRID[i]), float(H[j])
        h_prev, conn = h0, True
        dl, hp, cen = [], [], []
        for dlt in range(0, 301, 10):
            c, hw, hb = s3.best_hood(T, D, r0 - dlt, h_prev)
            if not (conn and np.isfinite(c)):
                conn = False
                break
            h_prev = c
            dl.append(dlt); hp.append(float(sc.hood_to_theta(hb))); cen.append(float(sc.hood_to_theta(c)))
        dl, hp, cen = map(np.array, (dl, hp, cen))
        def lin(y):
            if len(dl) < 3:
                return float('nan'), float('nan')
            A = np.vstack([np.ones(len(dl)), dl]).T
            co, *_ = np.linalg.lstsq(A, y, rcond=None)
            return co[1] * 100, float(np.max(np.abs(y - A @ co)))
        sh, dh = lin(hp); scn, dcn = lin(cen)
        jump = float(np.max(np.abs(np.diff(hp)))) if len(hp) > 1 else float('nan')
        L.append(f"{name:12s} D={D:3d} th0={hp[0]:5.1f} limit={dl[-1]:3d}  bestP-hood slope {sh:+6.2f} deg/100 maxdev {dh:5.2f} deg maxstep(10rpm) {jump:5.2f} | centre slope {scn:+6.2f} maxdev {dcn:5.2f}; "
                 f"bestP at -50/-100: {np.interp(50, dl, hp)-hp[0]:+.2f}/{(np.interp(100, dl, hp)-hp[0]) if dl[-1]>=100 else float('nan'):+.2f} deg")
txt = "\n".join(L); print(txt); open(os.path.join(HERE, "check_droop_hp.txt"), "w").write(txt)
