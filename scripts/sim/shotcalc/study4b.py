"""STUDY 4b: pure time-of-flight accuracy (vs the true TOF to the opening plane), table vs physics, on the shots fired."""
import json
import os
import sys
from multiprocessing import Pool

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import shotcore as sc
import physmodel as pm
import tuner as tu
from evalcore import Truth

NREP = 30
FRAME = 1 / 240
DQ = np.sort(np.random.default_rng(4242).uniform(36, 132, 200))


def worker(name):
    T = Truth(name)
    rng = np.random.default_rng(sum(name.encode()) * 13)
    out = {}
    for sched in ("maxP", "minDist"):
        for spn in ("even4", "even5", "even8", "uneven4"):
            Dc = np.array(tu.SPACINGS[spn], float)
            sites = {}
            for d in Dc:
                s = tu.CalSite(T, float(d))
                if sched == "minDist":
                    s.ref["minDist"] = tu.min_dist_ref(T, float(d))
                sites[float(d)] = s
            acc = {}
            for rep in range(NREP):
                cal = [sites[float(d)].draw(sched, rng) for d in Dc]
                rc = np.array([c[0] for c in cal])
                hc = np.array([c[1] for c in cal])
                tofc = T.at(Dc, rc, hc, keys=("tof",))["tof"] + rng.uniform(-FRAME, FRAME, len(Dc))
                reps = tu.Reps(Dc, rc, hc, name, rng, tofc=tofc)
                r, h = reps.settings("A_lin", DQ)
                true = T.at(DQ, r, h, keys=("tof",))["tof"]
                cands = {"table": np.interp(DQ, Dc, tofc),
                         "table_pchip": tu.pchip(Dc, tofc, DQ)}
                for pn in ("D3lin", "D3lin+tof", "E_indep"):
                    cands["phys_" + pn] = pm.model_tof(reps.p[pn], DQ, r, h)
                for cn, v in cands.items():
                    e = (v - true)
                    e = e[np.isfinite(e)]
                    acc.setdefault(cn, []).append((np.sqrt(np.mean(e ** 2)), np.max(np.abs(e)), np.mean(e)))
            for cn, v in acc.items():
                a = np.array(v) * 1000
                out[f"{sched}|{spn}|{cn}"] = dict(rms_ms=float(a[:, 0].mean()), max_ms=float(a[:, 1].mean()), bias_ms=float(a[:, 2].mean()))
    return name, out


if __name__ == "__main__":
    res = {}
    with Pool(6) as p:
        for name, out in p.imap_unordered(worker, sc.MAIN_TRUTHS):
            res[name] = out
            print("done", name, flush=True)
    os.makedirs(os.path.join(sc.HERE, "results"), exist_ok=True)
    json.dump(res, open(os.path.join(sc.HERE, "results", "study4b.json"), "w"), indent=1)
