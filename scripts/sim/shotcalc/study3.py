"""STUDY 3: flywheel droop during a burst, and how each hood-compensation scheme copes (plus its TOF effect)."""
import json
import math
import os
import sys
import time
from multiprocessing import Pool

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import shotcore as sc
import physmodel as pm
import tuner as tu
from evalcore import Truth

DS = [36, 48, 60, 72, 84, 96, 108, 120, 132]
DELTAS = [0, 50, 100, 150, 200, 250, 300]
H = sc.H_GRID
NREP = int(os.environ.get("NREP", "40"))
CAL = tu.SPACINGS["even5"]
DQ = np.sort(np.random.default_rng(777).uniform(36, 132, 200))
METHODS = ["a_none", "b_droop1", "b_droop2", "c_gainConst", "c_gainSched", "d_physDelta", "d_physDelta+droopFit", "oracle"]


def hood_runs(T, D, rpm):
    row = T.scored(np.full(H.shape, float(D)), np.full(H.shape, float(rpm)), H)
    return row


def best_hood(T, D, rpm, h_near):
    """Truth: centre of the scoring hood run nearest h_near, its half-width, and the max-P hood in that run."""
    row = hood_runs(T, D, rpm)
    if not row.any():
        return np.nan, 0.0, np.nan
    j = int(np.argmin(np.abs(H - h_near)))
    lo, hi = tu.run_of(row, j)
    P = T.P(np.full(hi - lo + 1, float(D)), np.full(hi - lo + 1, float(rpm)), H[lo:hi + 1])
    return 0.5 * (H[lo] + H[hi]), 0.5 * (H[hi] - H[lo]), float(H[lo + int(np.argmax(P))])


def solver_droop_hood(Dc, prim_r, pts, D, droop):
    """ShotSolver.lookupHood: per calibrated distance, hood interpolated by rpm among that distance's points
    (primary + droop points), clamped at the lowest; then linear in distance. pts[i] = [(rpm, hood), ...]."""
    vals = []
    for i in range(len(Dc)):
        p = sorted(pts[i])                      # ascending rpm
        rpm = prim_r[i] - droop
        rs = np.array([q[0] for q in p])
        hs = np.array([q[1] for q in p])
        vals.append(np.interp(rpm, rs, hs))      # np.interp clamps at both ends, as the solver does
    vals = np.array(vals)
    return np.interp(D, Dc, vals)


def part1(T, sched_ref):
    """Oracle base shots (no calibration error)."""
    res = {"perD": [], "methods": {}}
    base = {}
    for D in DS:
        site = tu.CalSite(T, D)
        if sched_ref == "minDist":
            i, j = tu.min_dist_ref(T, D)
        else:
            i, j = site.ref["maxP"]
        r0, h0 = float(sc.R_GRID[i]), float(H[j])
        base[D] = (r0, h0)
    # physics model fitted (D3lin) on the 5 calibration distances of the oracle schedule
    Dc = np.array(CAL, float)
    rc = np.array([np.interp(d, DS, [base[x][0] for x in DS]) for d in Dc])
    hc = np.array([np.interp(d, DS, [base[x][1] for x in DS]) for d in Dc])
    p, _ = tu.fit_physics("D3lin", Dc, rc, hc)
    gains = {}
    curves = {}
    for D in DS:
        r0, h0 = base[D]
        # centre-of-window hood vs deficit, up to the absorbable limit
        cen, half, hp = [], [], []
        max_any, max_conn, max_p80 = 0, 0, 0
        h_prev = h0
        connected = True
        for dlt in range(0, 1001, 10):
            row = hood_runs(T, D, r0 - dlt)
            if row.any() and np.any(row):
                max_any = dlt
            c, hw, hb = best_hood(T, D, r0 - dlt, h_prev)
            if connected and np.isfinite(c):
                max_conn = dlt
                h_prev = c
                Pb = float(T.P(D, r0 - dlt, hb))
                if Pb >= 0.8:
                    max_p80 = dlt
            else:
                connected = False
            if dlt <= 300:
                cen.append(c)
                half.append(hw)
                hp.append(hb)
        cen = np.array(cen)
        dl = np.arange(0, 301, 10)
        good = np.isfinite(cen)
        A = np.vstack([np.ones(good.sum()), dl[good]]).T
        coef, *_ = np.linalg.lstsq(A, cen[good], rcond=None)
        dev = np.abs(cen[good] - A @ coef)
        dth = np.gradient(sc.hood_to_theta(H), H)[int(np.argmin(np.abs(H - h0)))]
        res["perD"].append(dict(D=D, r0=r0, h0=h0, theta0=float(sc.hood_to_theta(h0)),
                                max_deficit_any=max_any, max_deficit_connected=max_conn, max_deficit_P80=max_p80,
                                slope_hood_per100=float(coef[1] * 100), slope_deg_per100=float(coef[1] * 100 * dth),
                                lin_maxdev_hood=float(dev.max()), lin_maxdev_frac_halfwidth=float(dev.max() / max(np.nanmin(np.array(half)[good]), 1e-9)),
                                halfwidth_hood_min=float(np.nanmin(np.array(half)[good]))))
        curves[D] = (np.array(hp), cen)
        gains[D] = (hp[10] - h0) / 100.0 if np.isfinite(hp[10]) else np.nan   # one droop point at -100 rpm (best-P hood)
    gconst = float(np.nanmean(list(gains.values())))
    dD, dr, dh = [], [], []
    for D in CAL:
        r0, h0 = base[D]
        hp = curves[D][0]
        for dlt, hh in ((0, h0), (100, hp[10]), (200, hp[20])):
            if np.isfinite(hh):
                dD.append(D)
                dr.append(r0 - dlt)
                dh.append(hh)
    p2, _ = tu.fit_physics("D3lin", np.array(dD, float), np.array(dr), np.array(dh))
    for D in DS:
        r0, h0 = base[D]
        hp, cen = curves[D]
        for dlt in DELTAS:
            k = dlt // 10
            r = r0 - dlt
            hood = {}
            hood["a_none"] = h0
            hood["b_droop1"] = float(np.interp(r, [r0 - 100, r0], [hp[10], h0]))
            hood["b_droop2"] = float(np.interp(r, [r0 - 200, r0 - 100, r0], [hp[20], hp[10], h0]))
            hood["c_gainConst"] = h0 + gconst * dlt
            hood["c_gainSched"] = h0 + gains[D] * dlt
            hd = pm.solve_hood(p, np.array([D]), np.array([r, r0]), np.array([h0, h0]))
            hood["d_physDelta"] = float(h0 + hd[0] - hd[1]) if np.all(np.isfinite(hd)) else h0
            hd2 = pm.solve_hood(p2, np.array([D]), np.array([r, r0]), np.array([h0, h0]))
            hood["d_physDelta+droopFit"] = float(h0 + hd2[0] - hd2[1]) if np.all(np.isfinite(hd2)) else h0
            hood["oracle"] = float(hp[k]) if np.isfinite(hp[k]) else h0
            hood = {m: (hh if np.isfinite(hh) else h0) for m, hh in hood.items()}
            tof0 = float(T.at(D, r0, h0, keys=("tof",))["tof"])
            for m, hh in hood.items():
                Pm = float(T.P(D, r, hh))
                tof = float(T.at(D, r, hh, keys=("tof",))["tof"])
                res["methods"].setdefault(m, {}).setdefault(str(dlt), []).append(dict(D=D, P=Pm, dtof_ms=1000 * (tof - tof0)))
    res["gconst_hood_per100"] = gconst * 100
    res["gains_hood_per100"] = {str(D): gains[D] * 100 for D in DS}
    return res


def part2(T, name, sched, rng):
    """Realistic: tuner calibrates primaries + droop points (central 60% of the windows) at 5 distances;
    evaluated at 200 unseen distances with deficit Delta; optional measurement bias of the deficit."""
    sites = {d: tu.CalSite(T, d) for d in CAL}
    if sched == "minDist":
        for d in CAL:
            sites[d].ref["minDist"] = tu.min_dist_ref(T, d)
    Dc = np.array(CAL, float)
    acc = {}
    for rep in range(NREP):
        prim = [sites[d].draw(sched, rng) for d in CAL]
        rc = np.array([q[0] for q in prim])
        hc = np.array([q[1] for q in prim])
        pts = []
        for i, d in enumerate(CAL):
            lst = [(rc[i], hc[i])]
            for dd in (100, 200):
                row = hood_runs(T, d, rc[i] - dd)
                j = int(np.argmin(np.abs(H - hc[i])))
                run = tu.run_of(row, j)
                if run is None:
                    lst.append((rc[i] - dd, hc[i]))
                    continue
                f = 0.2 + 0.6 * rng.random()
                lst.append((rc[i] - dd, H[run[0]] + f * (H[run[1]] - H[run[0]])))
            pts.append(lst)
        g = np.array([(pts[i][1][1] - hc[i]) / 100 for i in range(len(CAL))])
        p, _ = tu.fit_physics("D3lin", Dc, rc, hc)
        allD = np.concatenate([Dc, Dc, Dc])
        allr = np.concatenate([rc, [q[1][0] for q in pts], [q[2][0] for q in pts]])
        allh = np.concatenate([hc, [q[1][1] for q in pts], [q[2][1] for q in pts]])
        p2, _ = tu.fit_physics("D3lin", allD, allr, allh)
        rT = np.interp(DQ, Dc, rc)
        hA = np.interp(DQ, Dc, hc)
        base_h = {}
        for pp, key in ((p, "d_physDelta"), (p2, "d_physDelta+droopFit")):
            base_h[key] = pm.solve_hood(pp, DQ, rT, hA)
        for bias in (0.0, 30.0):
            for dlt in DELTAS:
                meas = dlt + bias if dlt > 0 or bias == 0 else 0.0   # no shot-induced deficit -> nothing to mis-measure
                r = rT - dlt
                hood = {}
                hood["a_none"] = hA
                pts1 = [lst[:2] for lst in pts]
                hood["b_droop1"] = solver_droop_hood(Dc, rc, pts1, DQ, meas)
                hood["b_droop2"] = solver_droop_hood(Dc, rc, pts, DQ, meas)
                hood["c_gainConst"] = hA + g.mean() * meas
                hood["c_gainSched"] = hA + np.interp(DQ, Dc, g) * meas
                for pp, key in ((p, "d_physDelta"), (p2, "d_physDelta+droopFit")):
                    hd = pm.solve_hood(pp, DQ, rT - meas, hA)
                    delta = np.where(np.isfinite(hd) & np.isfinite(base_h[key]), hd - base_h[key], 0.0)
                    hood[key] = hA + delta
                for m, hh in hood.items():
                    P = T.P(DQ, r, hh)
                    acc.setdefault((bias, dlt, m), []).append(float(P.mean()))
    return {f"bias{int(b)}|{d}|{m}": float(np.mean(v)) for (b, d, m), v in acc.items()}


def worker(args):
    name, sched = args
    t0 = time.time()
    T = Truth(name)
    rng = np.random.default_rng(sum(name.encode()) * 31 + len(sched))
    r1 = part1(T, sched)
    r2 = part2(T, name, sched, rng)
    print(f"{name} {sched} {time.time() - t0:.0f}s", flush=True)
    return name, sched, r1, r2


def main():
    names = sys.argv[1:] or sc.MAIN_TRUTHS
    jobs = [(n, s) for n in names for s in ("maxP", "minDist")]
    out = {}
    with Pool(min(len(jobs), 8)) as p:
        for name, sched, r1, r2 in p.imap_unordered(worker, jobs):
            out[f"{name}|{sched}"] = dict(part1=r1, part2=r2)
    os.makedirs(os.path.join(sc.HERE, "results"), exist_ok=True)
    with open(os.path.join(sc.HERE, "results", "study3.json"), "w") as f:
        json.dump(out, f, indent=1)


if __name__ == "__main__":
    main()
