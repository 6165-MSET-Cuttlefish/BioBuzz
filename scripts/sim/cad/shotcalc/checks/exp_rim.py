"""Sensitivity to the rim-clearance placeholder (ShotTuning.rimClearanceIn = 1, TODO) and to the robust-pick floors.
Reloads ../cadcore.py and ../windows.py with RIM_CLEARANCE_IN set to 0, 1 and 2 in. Pollen and Nectar, geometric base,
nominal drag."""
import importlib, json, os, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
HOOD_MIN, DIST_MIN = 2.0, 3.0


sys.path.insert(0, os.path.dirname(HERE))


def load(rim):
    os.environ["RIM_CLEARANCE_IN"] = str(float(rim))
    for m in ("cadcore", "windows"):
        sys.modules.pop(m, None)
    cc = importlib.import_module("cadcore")
    wd = importlib.import_module("windows")
    return cc, wd


def run(rim, shooter, Ds=(36, 48, 84, 132)):
    cc, wd = load(rim)
    b = cc.SHOOTERS[shooter]["base"]["geo"]
    mmax = cc.SHOOTERS[shooter]["mmax"]
    TH = b + np.arange(-14.0, 89.5 - b, 0.5)
    m = TH - b
    kq, kl = cc.drag_consts(shooter, "nom")
    ex = lambda th: cc.exit_point(shooter, np.asarray(th, float) - b)
    out = {}
    for D in Ds:
        a = wd.analyze(cc.SHOOTERS[shooter]["dia"], ex, TH, float(D), kq, kl)
        ok = a["ok"]
        trav = (m > -1e-6) & (m < mmax + 1e-6)
        rob = ok & (a["h_lo"] <= -HOOD_MIN) & (a["h_hi"] >= HOOD_MIN) & (a["d_lo"] <= -DIST_MIN) & (a["d_hi"] >= DIST_MIN)
        def best(mask):
            if not mask.any():
                return None
            j = np.where(mask)[0][int(np.nanargmax(a["s_w"][mask]))]
            return dict(m=float(m[j]), th=float(TH[j]), sw=100 * float(a["s_w"][j]), tof=1000 * float(a["ta"][j]),
                        v=float(a["v"][j]), hood=(float(a["h_lo"][j]), float(a["h_hi"][j])), dist=(float(a["d_lo"][j]), float(a["d_hi"][j])))
        r = dict(m_ok_lo=float(m[ok & trav].min()) if (ok & trav).any() else None,
                 cad=best(rob & trav) or best(ok & trav),
                 cad_is_robust=bool((rob & trav).any()),
                 floor46=best(rob & (TH >= 46 - 1e-6) & (m <= mmax + 1e-6)),
                 floor48=best(rob & (TH >= 48 - 1e-6) & (m <= mmax + 1e-6)),
                 floor50=best(rob & (TH >= 50 - 1e-6) & (m <= mmax + 1e-6)))
        # rising rule off: lowest in-travel m whose centre passes the outline test (descending or not)
        noris = trav & np.isfinite(a["slack"]) & (a["slack"] >= 0)
        r["m_lo_if_rising_allowed"] = float(m[noris].min()) if noris.any() else None
        out[D] = r
        c = r["cad"]
        print(f"rim {rim} {shooter:6s} D{D:4d}: m_ok_lo {r['m_ok_lo']} (rising allowed: {r['m_lo_if_rising_allowed']}) | CAD pick m {c['m']:.1f} "
              f"w {c['sw']:.2f}% tof {c['tof']:.0f} robust={r['cad_is_robust']} | floor46 " +
              (f"th {r['floor46']['th']:.1f} w {r['floor46']['sw']:.2f}% tof {r['floor46']['tof']:.0f}" if r['floor46'] else "-") + " | floor50 " +
              (f"th {r['floor50']['th']:.1f} w {r['floor50']['sw']:.2f}% tof {r['floor50']['tof']:.0f}" if r['floor50'] else "-"), flush=True)
    return out


if __name__ == "__main__":
    res = {}
    for rim in (0, 1, 2):
        for s in ("pollen", "nectar"):
            res[f"{rim}_{s}"] = run(rim, s)
    json.dump(res, open(os.path.join(HERE, "exp_rim.json"), "w"), indent=1, default=str)
