"""Re-run of Study 2 (clean + one bad point) with a new seed and alternative tuner models.

Tuner modes:
  orig      : study2's draw (rpm uniform in central 60% of the rpm window at the reference hood, then hood uniform
              in the central 60% of the hood window at that rpm)
  frac30    : study2's draw with the central 30% instead of 60%
  hoodfixed : hood held at the schedule's reference hood; rpm uniform in the central 60% of the rpm window there
  hoodjit2  : hood = reference hood + uniform(-2, +2) deg; rpm uniform in the central 60% of the rpm window there
Metrics on 200 unseen D: mean P with shot noise 1.5%/0.5deg (P15) and 2.0%/0.5deg (P20 ~ adds ~40 rpm controller error).
"""
import json
import os
import sys
import time
from multiprocessing import Pool

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import shotcore as sc
import physmodel as pm
import tuner as tu
from evalcore import Truth

NREP = int(os.environ.get("NREP", "40"))
SEED = int(os.environ.get("SEED", "20261110"))
MODES = os.environ.get("MODES", "orig,frac30,hoodfixed,hoodjit2").split(",")
SCHEDS = os.environ.get("SCHEDS", "maxP,minDist").split(",")
SPACINGS = os.environ.get("SPACINGS", "even4,even5,even8,uneven4,uneven8").split(",")
REPS = ["A_lin", "B_pchip", "C2_quad", "D2vac", "D3lin", "D4lin", "F_hybrid"]
if os.environ.get("AIMMODE"):
    pm.AIM["mode"] = os.environ["AIMMODE"]
DQ = np.sort(np.random.default_rng(SEED + 1).uniform(36, 132, 200))
BAD = 150.0


def hood_index_for_deg(j, ddeg):
    th = float(sc.hood_to_theta(sc.H_GRID[j])) + ddeg
    return int(np.clip(round(float(sc.theta_to_hood(th)) / (sc.H_GRID[1] - sc.H_GRID[0])), 0, len(sc.H_GRID) - 1))


def draw(site, sched, rng, mode):
    if mode == "orig":
        return site.draw(sched, rng)
    if mode == "frac30":
        return site.draw(sched, rng, frac=0.3)
    i, j = site.ref[sched]
    jj = j if mode == "hoodfixed" else hood_index_for_deg(j, rng.uniform(-2, 2))
    col = site.S[:, jj]
    while not col.any():
        jj += 1 if jj < j else -1
        col = site.S[:, jj]
    lo, hi = tu.run_of(col, i)
    f = 0.2 + 0.6 * rng.random()
    return float(sc.R_GRID[lo] + f * (sc.R_GRID[hi] - sc.R_GRID[lo])), float(sc.H_GRID[jj])


def metrics(T, r, h):
    p15 = T.P(DQ, r, h)
    p20 = T.P(DQ, r, h, sig_v=0.02)
    return float(p15.mean()), float(p15.min()), float(p20.mean())


def worker(name):
    t0 = time.time()
    T = Truth(name)
    T.P_field(0.02, 0.5)
    rng = np.random.default_rng(SEED + sum(name.encode()))
    dists = sorted({float(d) for sp in SPACINGS for d in tu.SPACINGS[sp]})
    sites = {}
    for d in dists:
        s = tu.CalSite(T, d)
        if "minDist" in SCHEDS:
            s.ref["minDist"] = tu.min_dist_ref(T, d)
        sites[d] = s
    acc = {}
    for mode in MODES:
        for sched in SCHEDS:
            for spn in SPACINGS:
                Dc = np.array(tu.SPACINGS[spn], float)
                for rep in range(NREP):
                    cal = [draw(sites[float(d)], sched, rng, mode) for d in Dc]
                    rc = np.array([c[0] for c in cal])
                    hc = np.array([c[1] for c in cal])
                    for scen in ("clean", "bad"):
                        rcx = rc.copy()
                        if scen == "bad":
                            rcx[int(rng.integers(1, len(Dc) - 1))] += BAD
                        reps = tu.Reps(Dc, rcx, hc, name, rng, eparams=dict(pm.NOMINAL))
                        for n in REPS:
                            r, h = reps.settings(n, DQ)
                            acc.setdefault(f"{mode}|{sched}|{spn}|{scen}|{n}", []).append(metrics(T, r, h))
                print(f"{name} {mode} {sched} {spn} {time.time() - t0:.0f}s", flush=True)
    return name, {k: dict(meanP=float(np.mean([x[0] for x in v])), minP=float(np.mean([x[1] for x in v])),
                          meanP20=float(np.mean([x[2] for x in v])), sdMeanP=float(np.std([x[0] for x in v])), n=len(v))
                  for k, v in acc.items()}


if __name__ == "__main__":
    names = sys.argv[1:] or sc.MAIN_TRUTHS
    out = {}
    with Pool(min(len(names), 8)) as p:
        for name, res in p.imap_unordered(worker, names):
            out[name] = res
            print("finished", name, flush=True)
    tag = os.environ.get("TAG", "")
    json.dump(out, open(os.path.join(HERE, f"study2_variants{tag}.json"), "w"), indent=1)
