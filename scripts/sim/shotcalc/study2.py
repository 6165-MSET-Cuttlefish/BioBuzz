"""STUDY 2 (representations) and STUDY 4 (time of flight / lead) on the same simulated calibrations."""
import json
import os
import sys
import time
from multiprocessing import Pool

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import shotcore as sc
import physmodel as pm
import tuner as tu
from evalcore import Truth, fmt_table

NREP = int(os.environ.get("NREP", "60"))
SCHEDULES = ["maxP", "minDist", "wander"]
BAD_RPM = 150.0
WEAR = 0.95
TRIM_D = 84.0
FRAME = 1 / 240
DQ = np.sort(np.random.default_rng(12345).uniform(36, 132, 200))
NAMES = tu.Reps.NAMES + ["E_trim1", "E_solverAim"]


def rep_metrics(T, r, h, gain=sc.K_GAIN):
    P = T.P(DQ, r, h, gain=gain)
    m = T.at(DQ, r, h, gain=gain, keys=("w", "vz"))
    miss = m["w"] - sc.W_MID
    with np.errstate(invalid="ignore"):
        ok = (m["vz"] < 0) & (np.abs(miss) <= (sc.W_HI - sc.W_LO) / 2)
    miss = np.where(np.isfinite(miss), miss, 20.0)
    return dict(meanP=float(P.mean()), minP=float(P.min()), fracLow=float((P < 0.5).mean()),
                rmsMiss=float(np.sqrt(np.mean(miss ** 2))), nomScored=float(ok.mean()))


def e_trim1(reps, idx):
    p = dict(reps.p["E_indep"])
    q, _ = pm.fit(p, ["k"], reps.Dc[idx:idx + 1], reps.rc[idx:idx + 1], reps.hc[idx:idx + 1])
    return q


def all_settings(reps, Dq, extra=True):
    out = {}
    for n in tu.Reps.NAMES:
        out[n] = reps.settings(n, Dq)
    if extra:
        idx = int(np.argmin(np.abs(reps.Dc - TRIM_D)))
        q = e_trim1(reps, idx)
        h = reps.hood(Dq)
        out["E_trim1"] = (pm.solve_rpm(q, Dq, h), h)
        pm.AIM["mode"] = "solver"
        out["E_solverAim"] = (pm.solve_rpm(reps.p["E_indep"], Dq, h), h)
        pm.AIM["mode"] = "band"
    return out


def worker(name):
    t0 = time.time()
    T = Truth(name)
    rng = np.random.default_rng(sum(name.encode()) * 7919)
    dists = sorted({float(d) for sp in tu.SPACINGS.values() for d in sp})
    sites = {}
    for d in dists:
        s = tu.CalSite(T, d)
        s.ref["minDist"] = tu.min_dist_ref(T, d)
        sites[d] = s
    S_worn = tu.slice_S(T, TRIM_D, gain=WEAR * sc.K_GAIN)
    rec = {}
    tofrec = {}
    det = {}

    def add(key, met):
        rec.setdefault(key, []).append(met)

    for sched in SCHEDULES:
        for spn, sp in tu.SPACINGS.items():
            Dc = np.array(sp, float)
            for rep in range(NREP):
                cal = [sites[float(d)].draw(sched, rng) for d in Dc]
                rc = np.array([c[0] for c in cal])
                hc = np.array([c[1] for c in cal])
                tof_true_c = T.at(Dc, rc, hc, keys=("tof",))["tof"]
                tofc = tof_true_c + rng.uniform(-FRAME, FRAME, len(Dc))
                eparams = tu.independent_params(name, rng)
                reps = tu.Reps(Dc, rc, hc, name, rng, tofc=tofc, eparams=eparams)
                st = all_settings(reps, DQ)
                for n, (r, h) in st.items():
                    add((sched, spn, "clean", n), rep_metrics(T, r, h))
                # wear: uncorrected, additive trim, multiplicative trim (trim found at TRIM_D with the worn ball)
                for n, (r, h) in st.items():
                    add((sched, spn, "wear", n), rep_metrics(T, r, h, gain=WEAR * sc.K_GAIN))
                    r84, h84 = reps.settings(n, np.array([TRIM_D])) if n in tu.Reps.NAMES else (
                        np.interp([TRIM_D], DQ, r), np.interp([TRIM_D], DQ, h))
                    j = int(np.clip(round(float(h84[0]) / (sc.H_GRID[1] - sc.H_GRID[0])), 0, len(sc.H_GRID) - 1))
                    i_guess = int(np.clip(round((float(r84[0]) / WEAR - sc.R_GRID[0]) / 5), 0, len(sc.R_GRID) - 1))
                    run = tu.run_of(S_worn[:, j], i_guess)
                    if run is None:
                        add((sched, spn, "wear+add", n), dict(meanP=0, minP=0, fracLow=1, rmsMiss=20, nomScored=0))
                        add((sched, spn, "wear+mul", n), dict(meanP=0, minP=0, fracLow=1, rmsMiss=20, nomScored=0))
                        continue
                    f = 0.2 + 0.6 * rng.random()
                    r_new = sc.R_GRID[run[0]] + f * (sc.R_GRID[run[1]] - sc.R_GRID[run[0]])
                    add((sched, spn, "wear+add", n), rep_metrics(T, r + (r_new - r84[0]), h, gain=WEAR * sc.K_GAIN))
                    add((sched, spn, "wear+mul", n), rep_metrics(T, r * (r_new / r84[0]), h, gain=WEAR * sc.K_GAIN))
                # one bad calibration point (rpm recorded 150 high), interior
                b = int(rng.integers(1, len(Dc) - 1))
                rcb = rc.copy()
                rcb[b] += BAD_RPM
                repb = tu.Reps(Dc, rcb, hc, name, rng, eparams=eparams)
                stb = all_settings(repb, DQ)
                for n, (r, h) in stb.items():
                    add((sched, spn, "bad", n), rep_metrics(T, r, h))
                for k in ("D3lin", "D4lin"):
                    det.setdefault((sched, spn, k), []).append(int(np.argmax(np.abs(repb.res[k])) == b))
                if len(Dc) >= 5:
                    c = np.polyfit((Dc - 84) / 48, rcb, 2)
                    res = rcb - np.polyval(c, (Dc - 84) / 48)
                    det.setdefault((sched, spn, "C2_quad"), []).append(int(np.argmax(np.abs(res)) == b))
                # STUDY 4: lead time error on the shots actually fired (A's settings and F's settings)
                for fired in ("A_lin", "F_hybrid"):
                    r, h = st[fired]
                    tr = T.at(DQ, r, h, keys=("tof", "lat"))
                    ok = np.isfinite(tr["lat"])
                    lat = tr["lat"]
                    tab = np.interp(DQ, Dc, tofc)
                    cands = {"table_tof": tab}
                    ks = np.linspace(0, 3, 301)
                    errs = [np.nanmean((((1 - np.exp(-k * tab)) / k if k > 0 else tab) - lat) ** 2) for k in ks]
                    kbest = ks[int(np.nanargmin(errs))]
                    cands["table_tof+bestKdrag"] = (1 - np.exp(-kbest * tab)) / kbest if kbest > 0 else tab
                    for pn in ("D3lin", "D3lin+tof", "E_indep"):
                        cands["phys_" + pn] = pm.lead_time(reps.p[pn], DQ, r, h)
                    cands["truth_tof_as_lead"] = tr["tof"]
                    for cn, lead in cands.items():
                        e = (lead - lat)[ok]
                        e = e[np.isfinite(e)]
                        tofrec.setdefault((sched, spn, fired, cn), []).append(
                            (float(np.sqrt(np.mean(e ** 2))) if e.size else np.nan, float(np.max(np.abs(e))) if e.size else np.nan,
                             float(np.mean(e)) if e.size else np.nan))
            print(f"{name} {sched} {spn} done {time.time() - t0:.0f}s", flush=True)
    agg = {}
    for key, lst in rec.items():
        a = {k: np.array([m[k] for m in lst]) for k in lst[0]}
        agg["|".join(key)] = dict(meanP=float(a["meanP"].mean()), p10_meanP=float(np.percentile(a["meanP"], 10)),
                                  minP=float(a["minP"].mean()), fracLow=float(a["fracLow"].mean()),
                                  rmsMiss=float(a["rmsMiss"].mean()), nomScored=float(a["nomScored"].mean()))
    tagg = {"|".join(k): dict(rms_ms=1000 * float(np.nanmean([x[0] for x in v])), max_ms=1000 * float(np.nanmean([x[1] for x in v])),
                              bias_ms=1000 * float(np.nanmean([x[2] for x in v]))) for k, v in tofrec.items()}
    dagg = {"|".join(k): float(np.mean(v)) for k, v in det.items()}
    return name, agg, tagg, dagg


def main():
    names = sys.argv[1:] or sc.MAIN_TRUTHS
    out = {}
    with Pool(min(len(names), 6)) as p:
        for name, agg, tagg, dagg in p.imap_unordered(worker, names):
            out[name] = dict(rep=agg, tof=tagg, detect=dagg)
            print(f"finished {name}", flush=True)
    tag = os.environ.get("TAG", "")
    os.makedirs(os.path.join(sc.HERE, "results"), exist_ok=True)
    path = os.path.join(sc.HERE, "results", f"study2_4{tag}.json")
    with open(path, "w") as f:
        json.dump(out, f, indent=1)


if __name__ == "__main__":
    main()
