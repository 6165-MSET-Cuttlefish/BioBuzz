"""Study 3 Part 2 (droop, realistic) with a new seed and a less noisy tuner: primaries with the hood held at the
maxP reference hood (rpm central 60%), droop-point hoods drawn from the central 30% of the hood run instead of 60%.
Copies study3.part2 logic; only the draws change. Mode 'orig' reproduces study3's draws with a new seed."""
import json
import os
import sys
import time
from multiprocessing import Pool

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import shotcore as sc
import tuner as tu
import study3 as s3
from evalcore import Truth

H = sc.H_GRID
NREP = int(os.environ.get("NREP", "40"))
SEED = int(os.environ.get("SEED", "777001"))


def part2(T, sched, rng, mode):
    CAL = s3.CAL
    sites = {d: tu.CalSite(T, d) for d in CAL}
    if sched == "minDist":
        for d in CAL:
            sites[d].ref["minDist"] = tu.min_dist_ref(T, d)
    Dc = np.array(CAL, float)
    DQ = s3.DQ
    acc = {}
    dfrac = 0.6 if mode == "orig" else 0.3
    for rep in range(NREP):
        prim = []
        for d in CAL:
            if mode == "orig":
                prim.append(sites[d].draw(sched, rng))
            else:
                i, j = sites[d].ref[sched]
                lo, hi = tu.run_of(sites[d].S[:, j], i)
                f = 0.2 + 0.6 * rng.random()
                prim.append((float(sc.R_GRID[lo] + f * (sc.R_GRID[hi] - sc.R_GRID[lo])), float(H[j])))
        rc = np.array([q[0] for q in prim])
        hc = np.array([q[1] for q in prim])
        pts = []
        for i, d in enumerate(CAL):
            lst = [(rc[i], hc[i])]
            for dd in (100, 200):
                row = s3.hood_runs(T, d, rc[i] - dd)
                j = int(np.argmin(np.abs(H - hc[i])))
                run = tu.run_of(row, j)
                if run is None:
                    lst.append((rc[i] - dd, hc[i]))
                    continue
                f = 0.5 - dfrac / 2 + dfrac * rng.random()
                lst.append((rc[i] - dd, H[run[0]] + f * (H[run[1]] - H[run[0]])))
            pts.append(lst)
        g = np.array([(pts[i][1][1] - hc[i]) / 100 for i in range(len(CAL))])
        rT = np.interp(DQ, Dc, rc)
        hA = np.interp(DQ, Dc, hc)
        for dlt in s3.DELTAS:
            r = rT - dlt
            hood = {"a_none": hA,
                    "b_droop1": s3.solver_droop_hood(Dc, rc, [l[:2] for l in pts], DQ, dlt),
                    "b_droop2": s3.solver_droop_hood(Dc, rc, pts, DQ, dlt),
                    "c_gainSched": hA + np.interp(DQ, Dc, g) * dlt}
            for m, hh in hood.items():
                acc.setdefault((dlt, m), []).append(float(T.P(DQ, r, hh).mean()))
    return {f"{d}|{m}": float(np.mean(v)) for (d, m), v in acc.items()}


def worker(args):
    name, sched, mode = args
    T = Truth(name)
    rng = np.random.default_rng(SEED + sum(name.encode()) + len(sched) + len(mode))
    t0 = time.time()
    res = part2(T, sched, rng, mode)
    print(name, sched, mode, f"{time.time()-t0:.0f}s", flush=True)
    return name, sched, mode, res


if __name__ == "__main__":
    jobs = [(n, s, m) for n in sc.MAIN_TRUTHS for s in ("maxP",) for m in ("orig", "calm")]
    out = {}
    with Pool(8) as p:
        for name, sched, mode, res in p.imap_unordered(worker, jobs):
            out[f"{name}|{sched}|{mode}"] = res
    json.dump(out, open(os.path.join(HERE, "study3_variant.json"), "w"), indent=1)
    L = []
    for mode in ("orig", "calm"):
        L.append(f"-- Part 2 maxP, mode={mode}: mean P over 6 truths vs deficit")
        L.append("method       " + " ".join(f"{d:>6d}" for d in s3.DELTAS))
        for m in ("a_none", "b_droop1", "b_droop2", "c_gainSched"):
            L.append(f"{m:12s} " + " ".join(f"{np.mean([out[k][f'{d}|{m}'] for k in out if k.endswith('|' + mode)]):6.3f}" for d in s3.DELTAS))
    txt = "\n".join(L)
    print(txt)
    open(os.path.join(HERE, "study3_variant.txt"), "w").write(txt)
