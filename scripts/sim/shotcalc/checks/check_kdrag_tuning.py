"""Study 4's 'table TOF + best constant kDragPerS' fits k against the TRUE lead time at the 200 evaluation distances
(an oracle). Realistic alternative: k chosen from strafing tests at 2 distances (60, 120 in) where the observed lead
error is measured with noise (sigma_ms per test, e.g. 5 shots at 40 in/s with ~1 in lateral reading error each ->
~10 ms). Compare rms lead error over 200 D."""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import shotcore as sc
import tuner as tu
from evalcore import Truth

FRAME = 1 / 240
DQ = np.sort(np.random.default_rng(55).uniform(36, 132, 200))
ks = np.linspace(0, 3, 601)


def dragtime(t, k):
    return np.where(k > 0, (1 - np.exp(-k * t)) / np.where(k > 0, k, 1), t)


L = []
for name in ("T2_45g_cd.4", "T2_25g_cd.5", "T2_10g_cd.6", "T3_10g_cd.6_cl.30"):
    T = Truth(name)
    rng = np.random.default_rng(7)
    Dc = np.array(tu.SPACINGS["even5"], float)
    sites = {d: tu.CalSite(T, d) for d in Dc}
    res = {"k0": [], "oracle": [], "2pt_s0": [], "2pt_s10": [], "2pt_s20": []}
    for rep in range(40):
        cal = [sites[d].draw("maxP", rng) for d in Dc]
        rc = np.array([c[0] for c in cal]); hc = np.array([c[1] for c in cal])
        tofc = T.at(Dc, rc, hc, keys=("tof",))["tof"] + rng.uniform(-FRAME, FRAME, len(Dc))
        r = np.interp(DQ, Dc, rc); h = np.interp(DQ, Dc, hc)
        tr = T.at(DQ, r, h, keys=("lat",))
        ok = np.isfinite(tr["lat"])
        lat = tr["lat"][ok]
        tab = np.interp(DQ, Dc, tofc)[ok]
        rms = lambda k: float(np.sqrt(np.mean((dragtime(tab, k) - lat) ** 2)))
        res["k0"].append(rms(0.0))
        res["oracle"].append(min(rms(k) for k in ks))
        Dt = np.array([60.0, 120.0])
        rt, ht = np.interp(Dt, Dc, rc), np.interp(Dt, Dc, hc)
        lat_t = T.at(Dt, rt, ht, keys=("lat",))["lat"]
        tab_t = np.interp(Dt, Dc, tofc)
        for sig, key in ((0.0, "2pt_s0"), (0.010, "2pt_s10"), (0.020, "2pt_s20")):
            obs = lat_t + sig * rng.standard_normal(2)
            kk = ks[int(np.argmin([np.sum((dragtime(tab_t, k) - obs) ** 2) for k in ks]))]
            res[key].append(rms(kk))
    L.append(f"{name:18s} rms lead error ms: " + "  ".join(f"{k} {1000*np.mean(v):.1f}" for k, v in res.items()))
txt = "\n".join(L)
print(txt)
open(os.path.join(HERE, "check_kdrag_tuning.txt"), "w").write(txt)
