"""Turns out/raw_*.npz into the answers: out/results.json and out/tables.txt."""
import json, math, os, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from cadcore import SHOOTERS, RPM_TO_INPS

OUT = os.path.join(HERE, "out")
HOOD_RATE = 78.0                 # deg of hood per s, no load (vendor 0.09 s/60 deg at 6 V through 8.571:1, as given)
ETAS = (0.35, 0.45, 0.50)
FREES = (5400, 5800)
KS = (3, 5, 8)
HOOD_MIN_SIDE, DIST_MIN_SIDE = 2.0, 3.0     # robustness floor for the best-tolerance pick [assumed]
V_REACH = 0.9 * 5400 * 0.45 * RPM_TO_INPS
LINES = []


def P(s=""):
    LINES.append(s)


def load(shooter, base, drag, cl=0.0):
    tag = f"{shooter}_{base}_{drag}" + (f"_cl{cl:+.2f}" if cl else "")
    p = os.path.join(OUT, f"raw_{tag}.npz")
    if not os.path.exists(p):
        return None
    d = np.load(p)
    R = {k: d[k] for k in d.files}
    R["shooter"], R["mmax"], R["tag"] = shooter, SHOOTERS[shooter]["mmax"], tag
    R["m"] = R["TH"] - R["base"]
    R["mF"] = R["TH_F"] - R["base"]
    R["trav"] = (R["m"] > -1e-6) & (R["m"] < R["mmax"] + 1e-6)
    R["travF"] = (R["mF"] > -1e-6) & (R["mF"] < R["mmax"] + 1e-6)
    return R


def iD(R, D):
    return int(np.where(R["DS"] == D)[0][0])


def vrow(R, D):
    j = int(np.argmin(np.abs(R["D_F"] - D)))
    return R["vfield"][j]


def parab_min(x, y):
    i = int(np.nanargmin(y))
    if 0 < i < len(y) - 1 and np.all(np.isfinite(y[i - 1:i + 2])):
        a, b, c = y[i - 1], y[i], y[i + 1]
        den = a - 2 * b + c
        dx = 0.5 * (a - c) / den if den > 0 else 0.0
        h = x[1] - x[0]
        return float(x[i] + dx * h), float(b - 0.25 * (a - c) * dx)
    return float(x[i]), float(y[i])


def v_at(R, D, th):
    row = vrow(R, D)
    ok = np.isfinite(row)
    return float(np.interp(th, R["TH_F"][ok], row[ok], left=np.nan, right=np.nan))


def g(R, key, i, th):
    """Window quantity at angle th (linear between 0.5 deg grid points; NaN outside the scoring set)."""
    y = R[key][i].astype(float)
    x = R["TH"]
    ok = R["ok"][i] & np.isfinite(y)
    if not ok.any():
        return float("nan")
    j = np.searchsorted(x, th)
    if th < x[ok].min() - 1e-9 or th > x[ok].max() + 1e-9:
        return float("nan")
    return float(np.interp(th, x[ok], y[ok]))


def per_distance(R):
    """Everything per distance at the 9 study distances."""
    rows = []
    for D in R["DS"]:
        i = iD(R, D)
        row = vrow(R, D)
        thv, vmin = parab_min(R["TH_F"], row)
        okT = R["ok"][i] & R["trav"]
        r = dict(D=int(D), vmin=vmin, th_vmin=thv, m_vmin=thv - R["base"])
        r["vmin_reach"] = bool(0 <= r["m_vmin"] <= R["mmax"])
        if not okT.any():
            r["scores"] = False
            rows.append(r)
            continue
        r["scores"] = True
        ms = R["m"][okT]
        r["m_ok_lo"], r["m_ok_hi"] = float(ms.min()), float(ms.max())
        # lowest in-travel angle that does not score because the ball is still rising at the opening
        rising = R["trav"] & ~R["ok"][i] & (R["vzp"][i] >= 0)
        r["rising_below_m"] = float(R["m"][rising].max() + 0.5) if rising.any() else None
        th_lo, th_hi = R["base"] + r["m_ok_lo"], R["base"] + r["m_ok_hi"]
        thf = min(max(thv, th_lo), th_hi)
        r["th_floor"], r["m_floor"], r["v_floor"] = thf, thf - R["base"], v_at(R, D, thf)
        for end, th in (("lo", th_lo), ("hi", th_hi)):
            j = int(np.argmin(np.abs(R["TH"] - th)))
            r[f"end_{end}"] = dict(m=float(R["m"][j]), v=float(R["v"][i, j]), tof_ms=float(1000 * R["ta"][i, j]),
                                   descent=float(R["descent"][i, j]), vz_plane=float(R["vzp"][i, j]))
        # windows on the in-travel scoring grid
        idx = np.where(okT)[0]
        j_d = idx[int(np.nanargmax(R["d_w"][i, idx]))]
        cand = okT & (R["v"][i] <= V_REACH) & (R["h_lo"][i] < 0) & (R["h_hi"][i] > 0) & (R["d_lo"][i] < 0) & (R["d_hi"][i] > 0)
        robust = cand & (R["h_lo"][i] <= -HOOD_MIN_SIDE) & (R["h_hi"][i] >= HOOD_MIN_SIDE) & \
            (R["d_lo"][i] <= -DIST_MIN_SIDE) & (R["d_hi"][i] >= DIST_MIN_SIDE)
        r["best_speed"] = best_of(R, i, cand)
        r["best_dist"] = shot(R, i, j_d)
        r["best_robust"] = best_of(R, i, robust)
        rows.append(r)
    return rows


def shot(R, i, j):
    return dict(m=float(R["m"][j]), th=float(R["TH"][j]), v=float(R["v"][i, j]), tof_ms=float(1000 * R["ta"][i, j]),
                descent=float(R["descent"][i, j]), s_lo=100 * float(R["s_lo"][i, j]), s_hi=100 * float(R["s_hi"][i, j]),
                s_w=100 * float(R["s_w"][i, j]), h_lo=float(R["h_lo"][i, j]), h_hi=float(R["h_hi"][i, j]),
                d_lo=float(R["d_lo"][i, j]), d_hi=float(R["d_hi"][i, j]), d_w=float(R["d_w"][i, j]))


def best_of(R, i, mask):
    if not mask.any():
        return None
    return shot(R, i, np.where(mask)[0][int(np.nanargmax(R["s_w"][i, mask]))])


def shot_at(R, i, th):
    D = int(R["DS"][i])
    out = dict(m=th - float(R["base"]), th=th, v=v_at(R, D, th), tof_ms=1000 * g(R, "ta", i, th),
               descent=g(R, "descent", i, th))
    for k in ("s_lo", "s_hi", "s_w"):
        out[k] = 100 * g(R, k, i, th)
    for k in ("h_lo", "h_hi", "d_lo", "d_hi", "d_w"):
        out[k] = g(R, k, i, th)
    return out


def sag_cover(R, pd, i, th0):
    """Sag the hood can absorb from nominal th0 by re-aiming through the aim point, staying in travel and scoring."""
    D = int(R["DS"][i])
    r = pd[i]
    v0 = v_at(R, D, th0)
    thf, vf = r["th_floor"], r["v_floor"]
    cov = 1 - vf / v0
    # to the edge of the window: lowest in-travel speed that still scores
    okT = R["ok"][i] & R["trav"]
    vedge = np.nanmin(R["v"][i, okT] * (1 + R["s_lo"][i, okT]))
    edge = 1 - vedge / v0
    # angle and TOF change for 1 % sag and for the full coverage, along v(th) between th0 and the floor
    ths = np.linspace(th0, thf, 400)
    vs = np.array([v_at(R, D, t) for t in ths])
    out = dict(th0=th0, m0=th0 - float(R["base"]), v0=v0, cover_pct=100 * cov, edge_pct=100 * edge,
               direction=("down" if thf < th0 - 1e-6 else ("up" if thf > th0 + 1e-6 else "none")),
               dth_full=thf - th0, hood_ms_full=1000 * abs(thf - th0) / HOOD_RATE,
               tof0_ms=1000 * g(R, "ta", i, th0))
    for s in (0.01, 0.02, 0.03, 0.05):
        tgt = v0 * (1 - s)
        if s > cov + 1e-9:
            out[f"dth_{int(s*100)}"] = None
            out[f"dtof_{int(s*100)}"] = None
            continue
        k = int(np.argmax(vs <= tgt)) if (vs <= tgt).any() else len(vs) - 1
        if k > 0:
            f = (vs[k - 1] - tgt) / (vs[k - 1] - vs[k])
            th = ths[k - 1] + f * (ths[k] - ths[k - 1])
        else:
            th = ths[0]
        out[f"dth_{int(s*100)}"] = th - th0
        out[f"dtof_{int(s*100)}"] = 1000 * (g(R, "ta", i, th) - g(R, "ta", i, th0))
    out["dtof_full"] = 1000 * (g(R, "ta", i, thf) - g(R, "ta", i, th0))
    return out


def theta_floor_F(R, D):
    """Lowest-speed in-travel angle on the fine field (ignores the short-range rising limit; used beyond 36 in)."""
    row = vrow(R, D)
    thv, _ = parab_min(R["TH_F"], row)
    lo, hi = R["base"], R["base"] + R["mmax"]
    return min(max(thv, lo), hi)


def sched_v(R, D, rule, k=0.0, table=None):
    if rule == "floor":
        th = theta_floor_F(R, D)
    elif rule == "room":
        th = theta_floor_F(R, D) + k
        if th > R["base"] + R["mmax"]:
            return float("nan"), th
    elif rule == "table":
        th = R["base"] + float(np.interp(D, table[0], table[1]))
    return v_at(R, D, th), th


def reach(R, rule, k=0.0, table=None):
    Ds = R["D_F"][R["D_F"] >= 36]
    vs = np.array([sched_v(R, D, rule, k, table)[0] for D in Ds])
    out = {}
    for eta in ETAS:
        for free in FREES:
            for f in (0.7, 0.8, 0.9):
                vmax = f * free * eta * RPM_TO_INPS
                okd = np.isfinite(vs) & (vs <= vmax)
                if not okd[0]:
                    out[f"{eta}_{free}_{f}"] = 0.0
                    continue
                k1 = int(np.argmax(~okd)) if (~okd).any() else len(Ds)
                if k1 == len(Ds):
                    out[f"{eta}_{free}_{f}"] = float(Ds[-1])
                else:
                    a, b = vs[k1 - 1], vs[k1]
                    out[f"{eta}_{free}_{f}"] = float(Ds[k1 - 1] + (vmax - a) / (b - a) * (Ds[k1] - Ds[k1 - 1])) if np.isfinite(b) else float(Ds[k1 - 1])
    return out


def interp_check(R, pd, rule, k=0.0, table=None):
    """Linear-in-distance interpolation of rpm and hood between N evenly spaced nodes (as ShotSolver.lookup)."""
    Ds = R["D_F"][(R["D_F"] >= 36) & (R["D_F"] <= 132)]
    truth = np.array([sched_v(R, D, rule, k, table) for D in Ds])
    sw_D = np.array([p["sched_sw"] for p in pd])
    res = {}
    for N in range(2, 10):
        nodes = np.linspace(36, 132, N)
        nv = np.array([sched_v(R, D, rule, k, table) for D in nodes])
        if not np.all(np.isfinite(nv)):
            res[N] = None
            continue
        v_int = np.interp(Ds, nodes, nv[:, 0])
        th_int = np.interp(Ds, nodes, nv[:, 1])
        v_true = np.array([v_at(R, D, t) for D, t in zip(Ds, th_int)])
        e = v_int / v_true - 1
        thr = 0.25 * np.interp(Ds, R["DS"], sw_D) / 100
        res[N] = dict(max_err_pct=float(100 * np.nanmax(np.abs(e))), max_ratio=float(np.nanmax(np.abs(e) / thr)),
                      worst_D=float(Ds[int(np.nanargmax(np.abs(e) / thr))]))
    ok = [N for N in res if res[N] and res[N]["max_ratio"] < 1]
    return res, (min(ok) if ok else None)


def linearity(R, rule, k=0.0, table=None):
    Ds = R["D_F"][(R["D_F"] >= 36) & (R["D_F"] <= 132)]
    v = np.array([sched_v(R, D, rule, k, table)[0] for D in Ds])
    if not np.all(np.isfinite(v)):
        return None
    chord = v[0] + (v[-1] - v[0]) * (Ds - Ds[0]) / (Ds[-1] - Ds[0])
    A = np.vstack([Ds, np.ones_like(Ds)]).T
    coef, *_ = np.linalg.lstsq(A, v, rcond=None)
    return dict(chord_dev_pct=float(100 * np.max(np.abs(v / chord - 1))), lsq_dev_pct=float(100 * np.max(np.abs(v / (A @ coef) - 1))))


def clean(o):
    if isinstance(o, dict):
        return {str(k): clean(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [clean(v) for v in o]
    if isinstance(o, (np.bool_, bool)):
        return bool(o)
    if isinstance(o, (np.integer,)):
        return int(o)
    if isinstance(o, (float, np.floating)):
        return None if not np.isfinite(o) else round(float(o), 4)
    if isinstance(o, np.ndarray):
        return clean(o.tolist())
    return o


def rpm(v, eta=0.45):
    return v / (eta * RPM_TO_INPS)


def fmt(x, f="{:.1f}", none="-"):
    return none if x is None or (isinstance(x, float) and not np.isfinite(x)) else f.format(x)


def analyse_combo(R):
    pd = per_distance(R)
    res = dict(tag=R["tag"], per_distance=pd)
    # ---- sag coverage for (a) best speed shot and (b) k above the lowest-speed in-travel angle and literal m = k
    res["sag_a"], res["sag_b"], res["sag_lit"] = [], {k: [] for k in KS}, {k: [] for k in KS}
    for i, r in enumerate(pd):
        if not r.get("scores"):
            res["sag_a"].append(None)
            for k in KS:
                res["sag_b"][k].append(None); res["sag_lit"][k].append(None)
            continue
        res["sag_a"].append(dict(sag_cover(R, pd, i, r["best_speed"]["th"]), shot=r["best_speed"]) if r["best_speed"] else None)
        for k in KS:
            th0 = r["th_floor"] + k
            if th0 <= R["base"] + R["mmax"] and np.isfinite(g(R, "s_w", i, th0)):
                res["sag_b"][k].append(dict(sag_cover(R, pd, i, th0), shot=shot_at(R, i, th0)))
            else:
                res["sag_b"][k].append(None)
            th0 = R["base"] + k
            j = int(np.argmin(np.abs(R["TH"] - th0)))
            res["sag_lit"][k].append(dict(sag_cover(R, pd, i, th0), shot=shot(R, i, j)) if R["ok"][i, j] else None)
    # ---- schedules: (i) robust best tolerance (raw per distance), (ii) floor + k
    raw_i = [(r["D"], r["best_robust"]["m"] if r["best_robust"] else None) for r in pd if r.get("scores")]
    res["sched_i_raw"] = raw_i
    # ---- steepest in-travel scoring m the flywheel can drive at 90 % of 5400 rpm free
    for i, r in enumerate(pd):
        if not r.get("scores"):
            continue
        okT = R["ok"][i] & R["trav"]
        for eta in ETAS:
            vmax = 0.9 * 5400 * eta * RPM_TO_INPS
            sel = okT & (R["v"][i] <= vmax)
            r[f"m_cap_{eta}"] = float(R["m"][sel].max()) if sel.any() else None
    # ---- hood offset needed to absorb a given sag by re-aiming down to the floor (high branch)
    res["need"] = []
    for i, r in enumerate(pd):
        if not r.get("scores"):
            res["need"].append(None); continue
        D = r["D"]
        ths = np.arange(r["th_floor"], 89.5, 0.05)
        vs = np.array([v_at(R, D, t) for t in ths])
        row = {}
        for sg in (0.02, 0.05, 0.10, 0.15):
            tgt = r["v_floor"] / (1 - sg)
            ok = np.isfinite(vs) & (vs >= tgt)
            if not ok.any():
                row[str(int(sg * 100))] = None; continue
            k1 = int(np.argmax(ok))
            th = ths[k1]
            row[str(int(sg * 100))] = dict(m=th - float(R["base"]), in_travel=bool(th <= R["base"] + R["mmax"] + 1e-9),
                                           rpm_frac=rpm(tgt) / 5400, tof_ms=1000 * g(R, "ta", i, th) if np.isfinite(g(R, "ta", i, th)) else None,
                                           s_w=100 * g(R, "s_w", i, th), dtof_ms=1000 * (g(R, "ta", i, r["th_floor"]) - g(R, "ta", i, th)))
        res["need"].append(row)
    # ---- unconstrained (any launch angle on the grid) best shots versus the CAD travel
    res["design"] = []
    for i, r in enumerate(pd):
        D = r["D"]
        okA = R["ok"][i]
        robust = okA & (R["h_lo"][i] <= -HOOD_MIN_SIDE) & (R["h_hi"][i] >= HOOD_MIN_SIDE) & \
            (R["d_lo"][i] <= -DIST_MIN_SIDE) & (R["d_hi"][i] >= DIST_MIN_SIDE)
        jr = np.where(robust)[0][int(np.nanargmax(R["s_w"][i, robust]))]
        d = dict(D=D, th_vmin=r["th_vmin"], m_vmin=r["m_vmin"], vmin=r["vmin"], tof_vmin=1000 * g(R, "ta", i, r["th_vmin"]),
                 sw_vmin=100 * g(R, "s_w", i, r["th_vmin"]), robust_free=shot(R, i, jr))
        if r.get("scores"):
            d.update(v_floor=r["v_floor"], th_floor=r["th_floor"], dv_pct=100 * (r["v_floor"] / r["vmin"] - 1),
                     dtof_ms=1000 * (g(R, "ta", i, r["th_floor"]) - g(R, "ta", i, r["th_vmin"])) if np.isfinite(g(R, "ta", i, r["th_vmin"])) else None,
                     sw_cad=r["best_robust"]["s_w"] if r["best_robust"] else None, sw_free=float(100 * R["s_w"][i, jr]))
        res["design"].append(d)
    return res


def smooth_table(raw, mmax):
    """Monotone non-increasing piecewise-linear fit of the raw best-tolerance m(D), rounded to 0.5 deg."""
    D = np.array([a for a, b in raw if b is not None], float)
    m = np.array([b for _, b in raw if b is not None], float)
    m = np.minimum.accumulate(m)          # tolerance optimum flattens with distance; enforce monotone
    return D, np.clip(np.round(m * 2) / 2, 0, mmax)


def main():
    combos = {}
    for s in ("pollen", "nectar"):
        for b in ("geo", "cal"):
            for d in ("none", "low", "nom", "high"):
                R = load(s, b, d)
                if R is not None:
                    combos[(s, b, d)] = R
        for cl in (-0.30, -0.15, 0.15):
            R = load(s, "geo", "nom", cl)
            if R is not None:
                combos[(s, "geo", f"nom_cl{cl:+.2f}")] = R
    results = {}
    for key, R in combos.items():
        res = analyse_combo(R)
        Dt, mt = smooth_table(res["sched_i_raw"], R["mmax"])
        table = (Dt, mt)
        res["sched_i_table"] = [[float(a), float(b)] for a, b in zip(Dt, mt)]
        pd = res["per_distance"]
        # schedule windows / rpm along (i) and (ii)
        sch = {}
        for name, rule, k, tb in [("i", "table", 0, table), ("floor", "floor", 0, None)] + [(f"k{k}", "room", k, None) for k in KS]:
            rows = []
            for i, r in enumerate(pd):
                D = r["D"]
                v, th = sched_v(R, D, rule, k, tb)
                if not r.get("scores"):
                    rows.append(None); continue
                if rule == "floor" or rule == "room":
                    th = r["th_floor"] + k
                    v = v_at(R, D, th) if th <= R["base"] + R["mmax"] + 1e-9 else float("nan")
                sh = shot_at(R, i, th) if np.isfinite(v) else None
                if sh is not None and np.isfinite(sh["s_w"]):
                    sh["sag"] = sag_cover(R, pd, i, th)
                    sh["rpm45"] = rpm(sh["v"])
                    sh["rpm_frac"] = {f"{eta}_{fr}": rpm(sh["v"], eta) / fr for eta in ETAS for fr in FREES}
                rows.append(sh)
            for p_, sh in zip(pd, rows):
                p_["sched_sw"] = sh["s_w"] if sh else np.nan
            ic, nmin = interp_check(R, pd, rule, k, tb) if all(rows) and all(np.isfinite(x["s_w"]) for x in rows) else (None, None)
            sch[name] = dict(rows=rows, reach=reach(R, rule, k, tb), linearity=linearity(R, rule, k, tb), interp=ic, n_nodes=nmin)
        res["schedules"] = sch
        results["|".join(key)] = res
    json.dump(clean(results), open(os.path.join(OUT, "results.json"), "w"), indent=1)
    write_tables(results, combos)
    open(os.path.join(OUT, "tables.txt"), "w").write("\n".join(LINES) + "\n")
    print("\n".join(LINES))


def write_tables(results, combos):
    for s in ("pollen", "nectar"):
        mm = SHOOTERS[s]["mmax"]
        for b in ("geo", "cal"):
            key = f"{s}|{b}|nom"
            if key not in results:
                continue
            res = results[key]
            R = combos[(s, b, "nom")]
            alt = {d: results.get(f"{s}|{b}|{d}") for d in ("none", "low", "high")}
            P(f"==================== {s.upper()}  base {b} ({R['base']:.1f} + m, m 0..{mm:.0f})  drag nom  [none | high drag in brackets]")
            P("SCORING RANGE  D | min-speed shot: th (m) v reach? | in-travel scoring m range | rising below m | low end: v tof desc | high end: v tof desc")
            for i, r in enumerate(res["per_distance"]):
                n0 = alt["none"]["per_distance"][i] if alt["none"] else {}
                hh = alt["high"]["per_distance"][i] if alt["high"] else {}
                if not r.get("scores"):
                    P(f"  {r['D']:3d} | {r['th_vmin']:5.1f} ({r['m_vmin']:+5.1f}) {r['vmin']:5.1f} {'Y' if r['vmin_reach'] else 'N'} | NO SCORING ANGLE IN TRAVEL")
                    continue
                lo, hi = r["end_lo"], r["end_hi"]
                P(f"  {r['D']:3d} | {r['th_vmin']:5.1f} ({r['m_vmin']:+5.1f}) [{n0.get('m_vmin', np.nan):+5.1f}|{hh.get('m_vmin', np.nan):+5.1f}] v {r['vmin']:5.1f} [{n0.get('vmin', np.nan):5.1f}|{hh.get('vmin', np.nan):5.1f}] {'Y' if r['vmin_reach'] else 'N'} | "
                  f"{r['m_ok_lo']:4.1f}..{r['m_ok_hi']:4.1f} | {fmt(r['rising_below_m'])} | {lo['v']:5.1f} {lo['tof_ms']:4.0f} {lo['descent']:4.1f} | {hi['v']:5.1f} {hi['tof_ms']:5.0f} {hi['descent']:4.1f}")
            P("WINDOWS  D | m: speed lo/hi (width) %, hood lo/hi deg, dist lo/hi in, v, rpm@.45, tof ms   [rows: m=0, floor, best speed, best distance, robust best]")
            for i, r in enumerate(res["per_distance"]):
                if not r.get("scores"):
                    continue
                Ri = combos[(s, b, "nom")]
                cand = []
                j0 = int(np.argmin(np.abs(Ri["m"])))
                if Ri["ok"][i, j0]:
                    cand.append(("m=0", shot(Ri, i, j0)))
                cand.append(("floor", shot_at(Ri, i, r["th_floor"])))
                cand += [("bestSpd", r["best_speed"]), ("bestDst", r["best_dist"]), ("robust", r["best_robust"])]
                for nm, sh in cand:
                    if sh is None:
                        P(f"  {r['D']:3d} {nm:7s} none")
                        continue
                    P(f"  {r['D']:3d} {nm:7s} m {sh['m']:5.1f}: spd {sh['s_lo']:+5.2f}/{sh['s_hi']:+5.2f} ({sh['s_w']:4.2f})  hood {sh['h_lo']:+6.2f}/{sh['h_hi']:+6.2f}  "
                      f"dist {sh['d_lo']:+6.1f}/{sh['d_hi']:+5.1f}  v {sh['v']:5.1f} rpm {rpm(sh['v']):5.0f} tof {sh['tof_ms']:5.0f} desc {sh['descent']:4.1f}")
            P("SAG COVER: sag the hood can absorb by re-aiming (cover%, to window edge%), hood direction, full move deg (ms at 78 deg/s), dTOF per 1% / full (ms)")
            for i, r in enumerate(res["per_distance"]):
                if not r.get("scores"):
                    continue
                a = res["sag_a"][i]
                txt = [f"(a)m{a['m0']:5.1f}: {a['cover_pct']:4.1f}/{a['edge_pct']:4.1f}% {a['direction']:4s} {a['dth_full']:+5.1f}" if a else "(a) none"]
                for k in KS:
                    bb = res["sag_b"][k][i]
                    if bb is None:
                        txt.append(f"(b)+{k}: n/a")
                        continue
                    sh = bb["shot"]
                    rel = f"rpm+{100*(bb['v0']/a['v0']-1):4.1f}% tof+{bb['tof0_ms']-a['tof0_ms']:4.0f}" if a else "rpm+ - tof+ -"
                    txt.append(f"(b)+{k} m{bb['m0']:4.1f}: {bb['cover_pct']:4.1f}/{bb['edge_pct']:4.1f}% {bb['direction']} {bb['dth_full']:+4.1f}({bb['hood_ms_full']:3.0f}ms) "
                               f"w{sh['s_w']:4.2f} {rel} dT1%{fmt(bb['dtof_1'],'{:+.0f}')}")
                P(f"  {r['D']:3d} " + " | ".join(txt))
            P("SAG COVER AT LITERAL m = 3/5/8 (k deg above the hood's lower limit): cover%/edge%, direction, speed width")
            for i, r in enumerate(res["per_distance"]):
                if not r.get("scores"):
                    continue
                txt = []
                for k in KS:
                    bb = res["sag_lit"][k][i]
                    txt.append(f"m={k}: " + ("does not score" if bb is None else f"{bb['cover_pct']:4.1f}/{bb['edge_pct']:4.1f}% {bb['direction']:4s} w{bb['shot']['s_w']:4.2f}"))
                P(f"  {r['D']:3d} " + " | ".join(txt))
            sch = res["schedules"]
            P("RPM AND REACH: required rpm (eta .45) and fraction of 5400 free [eta .35 | .50] per schedule; reach (in) at 70/80/90% free")
            for nm in ("floor", "i", "k3", "k5", "k8"):
                rows = sch[nm]["rows"]
                cells = []
                for r_, sh in zip(res["per_distance"], rows):
                    if sh is None or not np.isfinite(sh.get("v", np.nan)):
                        cells.append(f"{r_['D']}:-")
                        continue
                    cells.append(f"{r_['D']}:{rpm(sh['v']):4.0f}({sh['rpm_frac']['0.45_5400']:.2f}|{sh['rpm_frac']['0.35_5400']:.2f}|{sh['rpm_frac']['0.5_5400']:.2f})")
                P(f"  {nm:5s} " + " ".join(cells))
                rc = sch[nm]["reach"]
                P("        reach: " + "  ".join(f"eta{eta} free{fr}: " + "/".join(f"{rc[f'{eta}_{fr}_{f}']:.0f}" for f in (0.7, 0.8, 0.9)) for eta in ETAS for fr in FREES))
            P("SCHEDULES: i = best tolerance (robust pick, smoothed, 0.5 deg): D -> m")
            P("    " + "  ".join(f"{int(a)}:{bm:.1f}" for a, bm in res["sched_i_table"]) + "   raw: " + " ".join(f"{a}:{bm:.1f}" if bm is not None else f"{a}:none" for a, bm in res["sched_i_raw"]))
            for nm in ("i", "floor", "k3", "k5", "k8"):
                sc = sch[nm]
                rows = sc["rows"]
                P(f"    {nm:5s} m: " + " ".join(f"{r_['D']}:{sh['m']:.1f}" if sh else f"{r_['D']}:-" for r_, sh in zip(res["per_distance"], rows)))
                P(f"          rpm@.45: " + " ".join(f"{rpm(sh['v']):.0f}" if sh else "-" for sh in rows) + "   width%: " + " ".join(f"{sh['s_w']:.2f}" if sh else "-" for sh in rows))
                P(f"          tof ms: " + " ".join(f"{sh['tof_ms']:.0f}" if sh else "-" for sh in rows) + "   cover%: " + " ".join(f"{sh['sag']['cover_pct']:.1f}" if sh and 'sag' in sh else "-" for sh in rows))
                lin = sc["linearity"]
                ic = sc["interp"]
                P(f"          linearity: " + (f"chord dev {lin['chord_dev_pct']:.2f}%  lsq dev {lin['lsq_dev_pct']:.2f}%" if lin else "-") +
                  "   interp max ratio (err / quarter width) N=2..9: " + (" ".join(f"{N}:{ic[N]['max_ratio']:.2f}" if ic and ic.get(N) else f"{N}:-" for N in range(2, 10))) +
                  f"   min N: {sc['n_nodes']}")
            P("TIME OF FLIGHT on schedule and change when the hood covers sag: dTOF ms for 1%/2%/3%/5% sag (lead error in at 30 | 60 in/s)")
            for nm in ("i", "k5", "k8"):
                for r_, sh in zip(res["per_distance"], sch[nm]["rows"]):
                    if not sh or "sag" not in sh:
                        continue
                    sg = sh["sag"]
                    parts = []
                    for s_ in (1, 2, 3, 5):
                        dt = sg.get(f"dtof_{s_}")
                        parts.append(f"{s_}%:" + ("n/a" if dt is None else f"{dt:+4.0f}({abs(dt)*0.03:3.1f}|{abs(dt)*0.06:3.1f})"))
                    P(f"  {nm:3s} {r_['D']:3d} m {sh['m']:4.1f} tof {sh['tof_ms']:5.0f}  " + "  ".join(parts) + f"  full {sg['cover_pct']:.1f}%: {sg['dtof_full']:+.0f} ms")
            P("STEEPEST REACHABLE m: steepest in-travel scoring m within 90% of 5400 free, eta .35/.45/.50")
            P("    " + "  ".join(f"{r_['D']}:{fmt(r_.get('m_cap_0.35'))}/{fmt(r_.get('m_cap_0.45'))}/{fmt(r_.get('m_cap_0.5'))}" for r_ in res["per_distance"] if r_.get("scores")))
            P("HOOD OFFSET FOR A GIVEN SAG: hood offset m needed so that re-aiming down to the floor absorbs 2/5/10/15% sag: m (rpm frac eta.45/5400, tof ms, width%)  * = outside travel")
            for r_, nd in zip(res["per_distance"], res["need"]):
                if nd is None:
                    continue
                P(f"  {r_['D']:3d} floor m {r_['m_floor']:4.1f}: " + " | ".join(
                    f"{k}%: " + ("none" if nd[k] is None else f"m {nd[k]['m']:4.1f}{'' if nd[k]['in_travel'] else '*'} ({nd[k]['rpm_frac']:.2f}, {fmt(nd[k]['tof_ms'], '{:.0f}')}, {fmt(nd[k]['s_w'], '{:.2f}')})")
                    for k in ("2", "5", "10", "15")))
            P("HOOD FLOOR: unconstrained min-speed angle and robust-best angle (any hood angle) vs CAD floor: th_vmin (m), robust-best m (width), CAD best width, floor penalty dv% dTOF")
            for d in res["design"]:
                rb = d["robust_free"]
                P(f"  {d['D']:3d}: th_vmin {d['th_vmin']:5.1f} (m {d['m_vmin']:+5.1f}) v {d['vmin']:5.1f} tof {fmt(d['tof_vmin'], '{:.0f}')} width {fmt(d['sw_vmin'], '{:.2f}')} | robust-free m {rb['m']:+5.1f} width {rb['s_w']:.2f} tof {rb['tof_ms']:.0f} | "
                  f"CAD best width {fmt(d.get('sw_cad'), '{:.2f}')} | floor penalty dv {fmt(d.get('dv_pct'), '{:+.2f}')}% dTOF {fmt(d.get('dtof_ms'), '{:+.0f}')} ms")
            P("HOOD FLOOR SWEEP: robust speed window / TOF / rpm frac (eta .45, 5400) at the best angle reachable if the hood's lowest launch angle were th_lo")
            Rn = combos[(s, b, "nom")]
            for th_lo in (46.0, 48.0, 50.0, 52.0, 54.0, 56.0, float(Rn["base"])):
                cells = []
                for i, r_ in enumerate(res["per_distance"]):
                    if r_["D"] < 72:
                        continue
                    okA = Rn["ok"][i] & (Rn["TH"] >= th_lo - 1e-6) & (Rn["TH"] <= Rn["base"] + Rn["mmax"] + 1e-6)
                    rob = okA & (Rn["h_lo"][i] <= -HOOD_MIN_SIDE) & (Rn["h_hi"][i] >= HOOD_MIN_SIDE) & (Rn["d_lo"][i] <= -DIST_MIN_SIDE) & (Rn["d_hi"][i] >= DIST_MIN_SIDE)
                    if not rob.any():
                        cells.append(f"{r_['D']}:-"); continue
                    j = np.where(rob)[0][int(np.nanargmax(Rn["s_w"][i, rob]))]
                    cells.append(f"{r_['D']}:{100*Rn['s_w'][i, j]:.2f}%/{1000*Rn['ta'][i, j]:.0f}ms/{rpm(Rn['v'][i, j])/5400:.2f}@{Rn['TH'][j]:.1f}")
                P(f"   th_lo {th_lo:5.1f}: " + "  ".join(cells))
            P()
    summary(results)


def summary(results):
    P("==================== CROSS-CASE SUMMARY (all bases x drag cases)")
    P("case | min-speed shot reachable to D (in) | v_floor@36/84/132 in/s | rpm frac eta.45/5400 floor@132 | width% schedule(i) @36/84/132 | cover% k5 @36/84/132 | cover% k8 @84/132 | sched(i) m@36,48,60,72")
    for key, res in results.items():
        pd = res["per_distance"]
        reach_D = max([r["D"] for r in pd if r.get("vmin_reach")], default=None)
        def at(D, f):
            for r in pd:
                if r["D"] == D:
                    return f(r)
        sch = res["schedules"]
        def sw(nm, D):
            for r_, sh in zip(pd, sch[nm]["rows"]):
                if r_["D"] == D and sh:
                    return sh["s_w"]
        def cov(nm, D):
            for r_, sh in zip(pd, sch[nm]["rows"]):
                if r_["D"] == D and sh and "sag" in sh:
                    return sh["sag"]["cover_pct"]
        vf = [at(D, lambda r: r.get("v_floor")) for D in (36, 84, 132)]
        P(f"  {key:22s} | {fmt(reach_D, '{:.0f}')} | " + "/".join(fmt(x, "{:.0f}") for x in vf) +
          f" | {fmt(rpm(vf[2]) / 5400 if vf[2] else None, '{:.2f}')} | " + "/".join(fmt(sw('i', D), '{:.2f}') for D in (36, 84, 132)) +
          " | " + "/".join(fmt(cov('k5', D), '{:.1f}') for D in (36, 84, 132)) + " | " + "/".join(fmt(cov('k8', D), '{:.1f}') for D in (84, 132)) +
          " | " + ",".join(f"{m:.1f}" for _, m in res["sched_i_table"][:4]))


if __name__ == "__main__":
    main()
