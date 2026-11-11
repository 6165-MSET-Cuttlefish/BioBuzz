"""STUDY 1: the shot family at each distance, window sizes, and policies for choosing one shot."""
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import shotcore as sc
from evalcore import Truth, run_extent, fmt_table, SIG_V, SIG_TH

DS = [36, 48, 60, 72, 84, 96, 108, 120, 132]
D_CONST = list(range(36, 133, 4))
R = sc.R_GRID
H = sc.H_GRID
DR = R[1] - R[0]
DH = H[1] - H[0]
RNG = np.random.default_rng(1)
NMC = 4000
Z = RNG.standard_normal((4, NMC))
SIG_RPM_ERR = 40.0     # controller error at the moment of the shot (rpm, 1 sigma)
SIG_D_ERR = 3.0        # localization / distance error (in, 1 sigma)


def slice_scored(T, D, Rs=R, Hs=H, gain=sc.K_GAIN):
    rr, hh = np.meshgrid(Rs, Hs, indexing="ij")
    return T.scored(np.full(rr.shape, float(D)), rr, hh, gain)


def dist_tolerance(T, D, S0, r_idx):
    """Symmetric distance tolerance (in, 1-in steps) for every scoring cell of S0 (rows r_idx of R)."""
    Rs = R[r_idx]
    res = {}
    for direction in (+1, -1):
        alive = S0.copy()
        count = np.zeros(S0.shape, int)
        for step in range(1, 41):
            Dp = D + direction * step
            if Dp < sc.D_GRID[0] or Dp > sc.D_GRID[-1]:
                break
            alive &= slice_scored(T, Dp, Rs)
            count += alive
            if not alive.any():
                break
        res[direction] = count
    return np.minimum(res[1], res[-1]).astype(float)


def pick_ridge(S):
    """For each hood column, the middle of the widest rpm run: (row index or -1, width in cells)."""
    down, up = run_extent(S, 0)
    width = np.where(S, down + up + 1, 0)
    best_w = width.max(0)
    rows = np.full(S.shape[1], -1)
    for j in np.where(best_w > 0)[0]:
        i = int(np.argmax(width[:, j]))   # some cell of the widest run
        lo = i - down[i, j]
        hi = i + up[i, j]
        rows[j] = (lo + hi) // 2
    return rows, best_w


def mc_combined(T, D, rpm, hood, sig_v=SIG_V, sig_th=SIG_TH, sig_rpm=SIG_RPM_ERR, sig_d=SIG_D_ERR):
    """P(score) with shot noise plus rpm error and distance error (settings fixed for the intended D)."""
    Dt = D + sig_d * Z[3]
    r = rpm + sig_rpm * Z[2]
    v = sc.rpm_to_v(r, hood) * (1 + sig_v * Z[0])
    th = sc.hood_to_theta(hood) + sig_th * Z[1]
    w = sc.trilinear(T.F["w"], Dt, v, th)
    vz = sc.trilinear(T.F["vz"], Dt, v, th)
    vx = sc.trilinear(T.F["vx"], Dt, v, th)
    with np.errstate(invalid="ignore"):
        ok = sc.descending(vz, vx) & (w >= sc.W_LO) & (w <= sc.W_HI)
    return ok.mean()


def analyse(name):
    T = Truth(name)
    P15 = None
    out = {"truth": name, "windows": [], "policies": {}, "const": {}}
    per_D = {}
    for D in DS:
        S = slice_scored(T, D)
        if not S.any():
            out["windows"].append({"D": D, "empty": True})
            continue
        rows_any = np.where(S.any(1))[0]
        cols_any = np.where(S.any(0))[0]
        r_lo, r_hi = rows_any.min(), rows_any.max()
        sub = slice(max(r_lo - 2, 0), min(r_hi + 3, len(R)))
        Ssub = S[sub]
        Rsub = R[sub]
        rd, ru = run_extent(Ssub, 0)
        hd, hu = run_extent(Ssub, 1)
        rpm_tol = np.where(Ssub, np.minimum(rd, ru) * DR, np.nan)
        rpm_def = np.where(Ssub, rd * DR, np.nan)
        hood_tol = np.where(Ssub, np.minimum(hd, hu) * DH, np.nan)
        rpm_width = np.where(Ssub, (rd + ru + 1) * DR, 0)
        hood_width = np.where(Ssub, (hd + hu + 1) * DH, 0)
        dtol = np.where(Ssub, dist_tolerance(T, D, Ssub, np.arange(len(R))[sub]), np.nan)
        rr, hh = np.meshgrid(Rsub, H, indexing="ij")
        tof = T.at(np.full(rr.shape, float(D)), rr, hh, keys=("tof",))["tof"]
        P = np.where(Ssub, T.P(np.full(rr.shape, float(D)), rr, hh), np.nan)
        dthdh = np.gradient(sc.hood_to_theta(H), DH)

        # windows
        jw = int(np.argmax(rpm_width.max(0)))
        iw = int(np.argmax(hood_width.max(1)))
        win = {
            "D": D,
            "hood_range": [float(H[cols_any.min()]), float(H[cols_any.max()])],
            "theta_range_deg": [float(sc.hood_to_theta(H[cols_any.min()])), float(sc.hood_to_theta(H[cols_any.max()]))],
            "rpm_range": [float(R[r_lo]), float(R[r_hi])],
            "max_rpm_window": float(rpm_width.max()),
            "max_rpm_window_pct": float(100 * rpm_width.max() / Rsub[np.argmax(rpm_width[:, jw])]),
            "max_rpm_window_at_hood": float(H[jw]),
            "max_hood_window": float(hood_width.max()),
            "max_hood_window_deg": float(hood_width.max() * dthdh[np.argmax(hood_width[iw])]),
            "max_hood_window_at_rpm": float(Rsub[iw]),
        }
        out["windows"].append(win)

        # policies
        ridge_rows, ridge_w = pick_ridge(Ssub)
        pol = {}
        hc = 0.5 * (H[cols_any.min()] + H[cols_any.max()])
        jc = int(np.argmin(np.where(ridge_rows >= 0, np.abs(H - hc), 9)))
        pol["centre"] = (ridge_rows[jc], jc)
        score = np.nan_to_num(rpm_tol, nan=-1) + 1e-3 * np.nan_to_num(hood_tol, nan=0)
        pol["minRpmSens"] = np.unravel_index(int(np.argmax(score)), score.shape)
        score = np.nan_to_num(dtol, nan=-1) + 1e-3 * np.nan_to_num(P, nan=0)
        pol["minDistSens"] = np.unravel_index(int(np.argmax(score)), score.shape)
        ok_cols = np.where(ridge_w >= 0.5 * ridge_w.max())[0]
        tof_r = np.array([tof[ridge_rows[j], j] if j in set(ok_cols) else np.inf for j in range(len(H))])
        jt = int(np.nanargmin(tof_r))
        pol["minTOF"] = (ridge_rows[jt], jt)
        pol["maxP"] = np.unravel_index(int(np.nanargmax(np.nan_to_num(P, nan=-1))), P.shape)
        per_D[D] = dict(S=Ssub, Rsub=Rsub, rpm_tol=rpm_tol, rpm_def=rpm_def, hood_tol=hood_tol, dtol=dtol, tof=tof, P=P,
                        dthdh=dthdh)
        for pname, (i, j) in pol.items():
            out["policies"].setdefault(pname, []).append(point_metrics(T, D, per_D[D], int(i), int(j)))

    # constant rpm / constant hood over the whole range (4-in steps)
    Sall = {D: slice_scored(T, D) for D in D_CONST}
    # constant rpm: hood runs along axis 1
    best = None
    for i in range(len(R)):
        mins = []
        cols = []
        for D in D_CONST:
            row = Sall[D][i]
            if not row.any():
                mins = None
                break
            d, u = run_extent(row[None, :], 1)
            width = (d + u + 1)[0] * row
            j = int(np.argmax(width))
            lo, hi = j - d[0, j], j + u[0, j]
            jc = (lo + hi) // 2
            mins.append(min(jc - lo, hi - jc) * DH)
            cols.append(jc)
        if mins is None:
            continue
        m = min(mins)
        if best is None or m > best[0]:
            best = (m, i, cols)
    out["const"]["constRpm"] = None if best is None else {"rpm": float(R[best[1]]), "min_hood_halfwidth": best[0]}
    if best is not None:
        i = best[1]
        for D in DS:
            jc = best[2][D_CONST.index(D)]
            out["policies"].setdefault("constRpm", []).append(point_metrics_at(T, D, per_D.get(D), R[i], H[jc]))
    best = None
    for j in range(len(H)):
        mins = []
        rows = []
        for D in D_CONST:
            col = Sall[D][:, j]
            if not col.any():
                mins = None
                break
            d, u = run_extent(col[:, None], 0)
            width = (d + u + 1)[:, 0] * col
            i = int(np.argmax(width))
            lo, hi = i - d[i, 0], i + u[i, 0]
            ic = (lo + hi) // 2
            mins.append(min(ic - lo, hi - ic) * DR)
            rows.append(ic)
        if mins is None:
            continue
        m = min(mins)
        if best is None or m > best[0]:
            best = (m, j, rows)
    out["const"]["constHood"] = None if best is None else {"hood": float(H[best[1]]), "theta": float(sc.hood_to_theta(H[best[1]])), "min_rpm_halfwidth": best[0]}
    if best is not None:
        j = best[1]
        for D in DS:
            ic = best[2][D_CONST.index(D)]
            out["policies"].setdefault("constHood", []).append(point_metrics_at(T, D, per_D.get(D), R[ic], H[j]))
    return out


def point_metrics(T, D, pd, i, j):
    rpm = float(pd["Rsub"][i])
    hood = float(H[j])
    m = point_metrics_at(T, D, pd, rpm, hood)
    return m


def point_metrics_at(T, D, pd, rpm, hood):
    i = int(np.argmin(np.abs(pd["Rsub"] - rpm))) if pd is not None else None
    j = int(np.argmin(np.abs(H - hood)))
    g = lambda A: float(A[i, j]) if (pd is not None and abs(pd["Rsub"][i] - rpm) < 1e-6) else float("nan")
    v, th = T.vt(rpm, hood)
    w = T.at(D, rpm, hood, keys=("w", "vz", "tof"))
    # local sensitivities of entry w to rpm (per 100 rpm), to distance (per in), to hood (per 0.01)
    dw_drpm = (T.at(D, rpm + 25, hood, keys=("w",))["w"] - T.at(D, rpm - 25, hood, keys=("w",))["w"]) / 50 * 100
    dw_dD = (T.at(D + 1, rpm, hood, keys=("w",))["w"] - T.at(D - 1, rpm, hood, keys=("w",))["w"]) / 2
    dw_dh = (T.at(D, rpm, hood + 0.005, keys=("w",))["w"] - T.at(D, rpm, hood - 0.005, keys=("w",))["w"]) / 0.01 * 0.01
    return {
        "D": D, "rpm": rpm, "hood": hood, "theta": float(th), "v": float(v),
        "tof": float(w["tof"]), "w": float(w["w"]), "vz_entry": float(w["vz"]),
        "rpm_tol": g(pd["rpm_tol"]) if pd else float("nan"),
        "rpm_def": g(pd["rpm_def"]) if pd else float("nan"),
        "hood_tol": g(pd["hood_tol"]) if pd else float("nan"),
        "hood_tol_deg": g(pd["hood_tol"]) * float(pd["dthdh"][j]) if pd else float("nan"),
        "dist_tol": g(pd["dtol"]) if pd else float("nan"),
        "P15": float(T.P(D, rpm, hood)),
        "P10": float(T.P(D, rpm, hood, sig_v=0.01)),
        "P20": float(T.P(D, rpm, hood, sig_v=0.02)),
        "Pcomb": float(mc_combined(T, D, rpm, hood)),
        "dw_d100rpm": float(dw_drpm), "dw_dD": float(dw_dD), "dw_d0.01hood": float(dw_dh),
    }


def summarise(res):
    lines = []
    name = res["truth"]
    lines.append(f"\n=== STUDY 1  truth={name} ===")
    cols = ["D", "theta_lo", "theta_hi", "rpm_lo", "rpm_hi", "maxRpmWin", "maxRpmWin%", "@hood", "maxHoodWin", "maxHoodWin_deg", "@rpm"]
    rows = []
    for w in res["windows"]:
        if w.get("empty"):
            rows.append({"D": w["D"], "theta_lo": "EMPTY"})
            continue
        rows.append({"D": w["D"], "theta_lo": w["theta_range_deg"][0], "theta_hi": w["theta_range_deg"][1],
                     "rpm_lo": w["rpm_range"][0], "rpm_hi": w["rpm_range"][1], "maxRpmWin": w["max_rpm_window"],
                     "maxRpmWin%": w["max_rpm_window_pct"], "@hood": w["max_rpm_window_at_hood"],
                     "maxHoodWin": w["max_hood_window"], "maxHoodWin_deg": w["max_hood_window_deg"], "@rpm": w["max_hood_window_at_rpm"]})
    lines.append("Scoring family (hood limited to theta 30-80 deg):")
    lines.append(fmt_table(rows, cols, "{:.4g}"))
    lines.append(f"constRpm: {res['const']['constRpm']}   constHood: {res['const']['constHood']}")
    cols = ["policy", "mean_rpmTol", "min_rpmTol", "min_rpmDef", "mean_hoodTol_deg", "mean_distTol", "min_distTol", "mean_tof", "max_tof", "mean_P15", "min_P15", "mean_P20", "mean_Pcomb", "min_Pcomb"]
    rows = []
    for p, pts in res["policies"].items():
        a = lambda k: np.array([x[k] for x in pts], float)
        rows.append({"policy": p, "mean_rpmTol": np.nanmean(a("rpm_tol")), "min_rpmTol": np.nanmin(a("rpm_tol")),
                     "min_rpmDef": np.nanmin(a("rpm_def")), "mean_hoodTol_deg": np.nanmean(a("hood_tol_deg")),
                     "mean_distTol": np.nanmean(a("dist_tol")), "min_distTol": np.nanmin(a("dist_tol")),
                     "mean_tof": np.nanmean(a("tof")), "max_tof": np.nanmax(a("tof")),
                     "mean_P15": np.mean(a("P15")), "min_P15": np.min(a("P15")), "mean_P20": np.mean(a("P20")),
                     "mean_Pcomb": np.mean(a("Pcomb")), "min_Pcomb": np.min(a("Pcomb"))})
    lines.append("Policies (over D = 36..132 step 12; tolerances are symmetric half-widths at fixed other settings):")
    lines.append(fmt_table(rows, cols, "{:.3g}"))
    for p in ("minRpmSens", "minDistSens", "maxP", "centre", "constRpm"):
        pts = res["policies"].get(p)
        if not pts:
            continue
        lines.append(f"  {p} per D: " + "; ".join(
            f"{x['D']}:{x['rpm']:.0f}rpm/{x['theta']:.1f}deg tof{x['tof']:.2f} rT{x['rpm_tol']:.0f} dT{x['dist_tol']:.0f} Pc{x['Pcomb']:.2f}" for x in pts))
    return "\n".join(lines)


def main(names):
    os.makedirs(os.path.join(sc.HERE, "results"), exist_ok=True)
    allres = {}
    text = []
    for n in names:
        res = analyse(n)
        allres[n] = res
        s = summarise(res)
        print(s, flush=True)
        text.append(s)
    suffix = "" if sc.DESCENT_MIN_DEG <= 0 else f"_desc{sc.DESCENT_MIN_DEG:g}"
    with open(os.path.join(sc.HERE, "results", f"study1{suffix}.json"), "w") as f:
        json.dump(allres, f, indent=1, default=float)
    with open(os.path.join(sc.HERE, "results", f"study1{suffix}.txt"), "w") as f:
        f.write("\n".join(text))


if __name__ == "__main__":
    main(sys.argv[1:] or list(sc.TRUTHS))
