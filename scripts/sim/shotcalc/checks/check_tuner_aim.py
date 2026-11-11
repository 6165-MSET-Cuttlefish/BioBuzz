"""(a) How spread are the simulated tuner's calibration points (hood in deg, rpm)?  (b) At a fixed hood, is the
band-centre aim (w = W_MID, what the physics reps solve for) the P-optimal rpm?  (c) P at the tuner's points."""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import shotcore as sc
import tuner as tu
from evalcore import Truth

R, H = sc.R_GRID, sc.H_GRID
out = []
for name in sys.argv[1:] or ["T2_25g_cd.5", "T2_10g_cd.6"]:
    T = Truth(name)
    rng = np.random.default_rng(99)
    out.append(f"== {name}")
    for D in (36.0, 60.0, 84.0, 108.0, 132.0):
        s = tu.CalSite(T, D)
        i, j = s.ref["maxP"]
        r0, h0 = R[i], H[j]
        th0 = float(sc.hood_to_theta(h0))
        draws = np.array([s.draw("maxP", rng) for _ in range(1500)])
        thd = sc.hood_to_theta(draws[:, 1])
        Pd = T.P(np.full(len(draws), D), draws[:, 0], draws[:, 1])
        # fixed hood h0: rpm sweep
        rr = np.arange(r0 - 400, r0 + 400, 1.0)
        m = T.at(np.full(rr.shape, D), rr, np.full(rr.shape, h0), keys=("w", "vz"))
        P = T.P(np.full(rr.shape, D), rr, np.full(rr.shape, h0))
        k_mid = int(np.nanargmin(np.abs(m["w"] - sc.W_MID)))
        k_P = int(np.argmax(P))
        sc_mask = T.scored(np.full(rr.shape, D), rr, np.full(rr.shape, h0))
        lo, hi = rr[sc_mask].min(), rr[sc_mask].max()
        # which constraint bounds the window on the high side?
        top = T.at(np.array([D]), np.array([hi + 2.0]), np.array([h0]), keys=("w", "vz"))
        out.append(f" D={D:5.0f} maxP rpm {r0:.0f} theta {th0:.1f}; window@h0 [{lo:.0f},{hi:.0f}] "
                   f"(upper bound: w={float(top['w'][0]):.2f} vz={float(top['vz'][0]):+.0f}); "
                   f"rpm@w=Wmid {rr[k_mid]:.0f} (P={P[k_mid]:.3f}, vz={m['vz'][k_mid]:+.0f}); rpm@maxP|h0 {rr[k_P]:.0f} (P={P[k_P]:.3f}, w={m['w'][k_P]:.2f})")
        out.append(f"        tuner draws: theta mean {thd.mean():.1f} sd {thd.std():.2f} deg (range {thd.min():.1f}-{thd.max():.1f}); "
                   f"rpm sd {draws[:,0].std():.0f}; P at drawn points mean {Pd.mean():.3f} p10 {np.percentile(Pd,10):.3f}")
txt = "\n".join(out)
print(txt)
open(os.path.join(HERE, "check_tuner_aim.txt"), "w").write(txt)
