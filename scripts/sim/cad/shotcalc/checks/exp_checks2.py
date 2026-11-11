"""(1) How much the CAD exit (1.2-2.0 in forward, 12.5-13.8 in high) moves the needed speed versus the solver's
14 in exit at the axis, drag-free, at fixed launch angle and at the minimum-speed angle.
(2) Interpolation check with a stricter criterion: error against half the LOW side of the speed window (a too-low rpm
lands on the low side), using the study's own report functions (../report.py) on the study's npz (read only).
(3) Window cost of the interpolated hood leaving the best-tolerance schedule with 3 nodes."""
import math, os, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import report as RP
G, ZAIM = 386.09, 56.81508


def v_nd(th, x0, z0, D):
    t = math.radians(th); d = D - x0; h = ZAIM - z0
    den = 2 * math.cos(t) ** 2 * (d * math.tan(t) - h)
    return math.sqrt(G * d * d / den) if den > 0 else float("nan")


print("(1) drag-free needed speed, CAD exit vs (0, 14):  D: max |dv| over in-travel angles; dv at min-speed angle")
for sh, base, mmax, ex in (("pollen", 57.7, 30, ((1.56, 12.46), (1.99, 13.83))), ("nectar", 60.8, 20, ((1.23, 12.53), (1.60, 13.57)))):
    cells = []
    for D in (36, 84, 132):
        dvs, best_cad, best_14 = [], 1e9, 1e9
        for m in np.arange(0, mmax + 1e-9, 0.25):
            f = m / mmax
            x0 = ex[0][0] + f * (ex[1][0] - ex[0][0]); z0 = ex[0][1] + f * (ex[1][1] - ex[0][1])
            a, b = v_nd(base + m, x0, z0, D), v_nd(base + m, 0, 14, D)
            if np.isfinite(a) and np.isfinite(b):
                dvs.append(100 * (a / b - 1))
        for th in np.arange(40, 85, 0.01):
            best_14 = min(best_14, v_nd(th, 0, 14, D))
            m = th - base; f = m / mmax
            x0 = ex[0][0] + f * (ex[1][0] - ex[0][0]); z0 = ex[0][1] + f * (ex[1][1] - ex[0][1])
            best_cad = min(best_cad, v_nd(th, x0, z0, D))
        cells.append(f"{D}: max|dv| {max(abs(x) for x in dvs):.2f}% (range {min(dvs):+.2f}..{max(dvs):+.2f}), v_min {100 * (best_cad / best_14 - 1):+.2f}%")
    print(f"  {sh}: " + " | ".join(cells))

print("(2) interpolation error with N evenly spaced nodes: ratio to a quarter of the width (study) and to half the low side (stricter)")
for sh in ("pollen", "nectar"):
    for b in ("geo",):
        R = RP.load(sh, b, "nom")
        res = RP.analyse_combo(R)
        Dt, mt = RP.smooth_table(res["sched_i_raw"], R["mmax"])
        pd = res["per_distance"]
        for name, rule, k, tb in (("i", "table", 0, (Dt, mt)), ("k5", "room", 5, None), ("k8", "room", 8, None)):
            lows, widths = [], []
            for i, r in enumerate(pd):
                th = RP.sched_v(R, r["D"], rule, k, tb)[1] if rule == "table" else r["th_floor"] + k
                sh_ = RP.shot_at(R, i, th)
                lows.append(-sh_["s_lo"]); widths.append(sh_["s_w"])
            Ds = R["D_F"][(R["D_F"] >= 36) & (R["D_F"] <= 132)]
            out = []
            for N in (2, 3, 4, 5, 9):
                nodes = np.linspace(36, 132, N)
                nv = np.array([RP.sched_v(R, D, rule, k, tb) for D in nodes])
                v_int = np.interp(Ds, nodes, nv[:, 0]); th_int = np.interp(Ds, nodes, nv[:, 1])
                v_true = np.array([RP.v_at(R, D, t) for D, t in zip(Ds, th_int)])
                e = 100 * (v_int / v_true - 1)
                q = 0.25 * np.interp(Ds, R["DS"], widths)
                lo = 0.5 * np.interp(Ds, R["DS"], lows)
                neg = np.where(e < 0, -e, 0)
                out.append(f"N{N}: quarter-width {np.nanmax(np.abs(e) / q):.2f}, half-low-side {np.nanmax(neg / lo):.2f} (max err {np.nanmax(np.abs(e)):.2f}%)")
            print(f"  {sh} {b} {name}: " + "  ".join(out))
        if sh == "pollen":
            nodes = np.array([36, 84, 132.0])
            m_int = np.interp(60, nodes, np.interp(nodes, Dt, mt))
            i60 = RP.iD(R, 60)
            sh_ = RP.shot_at(R, i60, R["base"] + m_int)
            print(f"  (3) pollen 3 nodes: hood at 60 in interpolates to m {m_int:.1f} (schedule 2.0): width {sh_['s_w']:.2f}% vs 8.08%, "
                  f"low side {sh_['s_lo']:+.2f}%, tof {sh_['tof_ms']:.0f} ms vs 545")
