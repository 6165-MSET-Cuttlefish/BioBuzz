"""Main sweep: per shooter x launch-angle base x drag case, windows at 36..132 in on a launch-angle grid
(44..89.5 deg, inside and outside the hood travel), plus the aim-point speed field v(D, theta) for 24..240 in.
Writes out/raw_<shooter>_<base>_<drag>[_cl<cl>].npz. With arguments <shooter> <base> <drag> [cl] it runs that case only."""
import os, sys, time
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from cadcore import SHOOTERS, drag_consts, exit_point, v_aim
from windows import analyze

DS = np.arange(36, 133, 12)
D_F = np.arange(24.0, 240.01, 2.0)
OUT = os.path.join(HERE, "out")


def run(shooter, base, drag, cl=0.0):
    tag = f"{shooter}_{base}_{drag}" + (f"_cl{cl:+.2f}" if cl else "")
    path = os.path.join(OUT, f"raw_{tag}.npz")
    if os.path.exists(path):
        return path
    os.makedirs(OUT, exist_ok=True)
    t0 = time.time()
    b = SHOOTERS[shooter]["base"][base]
    TH = b + np.arange(-14.0, 89.5 - b, 0.5)        # aligned so m = 0, 3, 5, 8 and the top of travel are grid points
    TH_F = b + np.arange(-14.0, 89.5 - b, 0.25)
    dia = SHOOTERS[shooter]["dia"]
    ex = lambda th: exit_point(shooter, np.asarray(th, float) - b)
    kq, kl = drag_consts(shooter, drag, cl)
    res = {}
    for D in DS:
        a = analyze(dia, ex, TH, float(D), kq, kl)
        for k, v in a.items():
            res.setdefault(k, []).append(v)
    out = {k: np.array(v) for k, v in res.items()}
    tt, dd = np.meshgrid(TH_F, D_F)
    x0, z0 = ex(tt)
    out["vfield"] = v_aim(tt, x0, z0, dd, kq, kl)
    out.update(DS=DS, TH=TH, TH_F=TH_F, D_F=D_F, base=b, kq=kq, kl=kl)
    np.savez_compressed(path, **out)
    print(f"{tag}: {time.time() - t0:.0f} s", flush=True)
    return path


if __name__ == "__main__":
    if len(sys.argv) > 1:
        run(sys.argv[1], sys.argv[2], sys.argv[3], float(sys.argv[4]) if len(sys.argv) > 4 else 0.0)
        sys.exit()
    jobs = [(s, b, d, 0.0) for s in ("pollen", "nectar") for b in ("geo", "cal") for d in ("none", "nom", "low", "high")]
    jobs += [(s, "geo", "nom", cl) for s in ("pollen", "nectar") for cl in (-0.30, -0.15, 0.15)]
    for j in jobs:
        run(*j)
