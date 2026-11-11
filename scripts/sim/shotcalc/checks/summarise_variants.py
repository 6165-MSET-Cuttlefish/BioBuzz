import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "study2_variants.json")
r = json.load(open(path))
truths = list(r)
keys0 = list(r[truths[0]])
modes = sorted({k.split("|")[0] for k in keys0}, key=["orig", "frac30", "hoodjit2", "hoodfixed"].index)
scheds = sorted({k.split("|")[1] for k in keys0})
spacings = sorted({k.split("|")[2] for k in keys0})
reps = ["A_lin", "B_pchip", "C2_quad", "D2vac", "D3lin", "D4lin", "F_hybrid"]
L = []


def g(t, m, s, sp, scen, rep, k="meanP"):
    return r[t][f"{m}|{s}|{sp}|{scen}|{rep}"][k]


def avg(m, scen, rep, k="meanP", ts=None, ss=None, sps=None):
    return float(np.mean([g(t, m, s, sp, scen, rep, k) for t in (ts or truths) for s in (ss or scheds) for sp in (sps or spacings)]))


for scen in ("clean", "bad"):
    for k in ("meanP", "minP", "meanP20"):
        L.append(f"-- {scen} {k}: mean over {len(truths)} truths x {scheds} x {len(spacings)} spacings")
        L.append("mode       " + " ".join(f"{x:>9s}" for x in reps) + "   D3-A    F-A    B-A")
        for m in modes:
            v = {x: avg(m, scen, x, k) for x in reps}
            L.append(f"{m:10s} " + " ".join(f"{v[x]:9.3f}" for x in reps) +
                     f" {v['D3lin']-v['A_lin']:+.3f} {v['F_hybrid']-v['A_lin']:+.3f} {v['B_pchip']-v['A_lin']:+.3f}")
L.append("-- clean meanP, D3lin-A and F-A and B-A by N (even+uneven), per mode; [min over truths]")
for m in modes:
    row = []
    for N in ("4", "5", "6", "8"):
        sps = [s for s in spacings if s.endswith(N)]
        if not sps:
            continue
        d3 = [avg(m, "clean", "D3lin", ts=[t], sps=sps) - avg(m, "clean", "A_lin", ts=[t], sps=sps) for t in truths]
        f = [avg(m, "clean", "F_hybrid", ts=[t], sps=sps) - avg(m, "clean", "A_lin", ts=[t], sps=sps) for t in truths]
        b = [avg(m, "clean", "B_pchip", ts=[t], sps=sps) - avg(m, "clean", "A_lin", ts=[t], sps=sps) for t in truths]
        row.append(f"N{N}: D3-A {np.mean(d3):+.3f}[{min(d3):+.3f}] F-A {np.mean(f):+.3f}[{min(f):+.3f}] B-A {np.mean(b):+.3f}[{min(b):+.3f}]")
    L.append(f"  {m:10s} " + " | ".join(row))
L.append("-- clean meanP per truth x schedule: rank of A among 7 and best rep (mean over spacings)")
for m in modes:
    cells = []
    for t in truths:
        for s in scheds:
            v = {x: avg(m, "clean", x, ts=[t], ss=[s]) for x in reps}
            order = sorted(reps, key=lambda x: -v[x])
            cells.append(f"{t[:11]}/{s}:A#{order.index('A_lin')+1} best {order[0]} {v[order[0]]:.3f} A {v['A_lin']:.3f} F {v['F_hybrid']:.3f}")
    L.append(f"  {m}:")
    L += ["     " + c for c in cells]
L.append("-- count of (truth,sched,spacing) cells where A_lin is best / F >= A / D3 >= A (clean meanP)")
for m in modes:
    nA = nF = nD = tot = 0
    for t in truths:
        for s in scheds:
            for sp in spacings:
                v = {x: g(t, m, s, sp, "clean", x) for x in reps}
                tot += 1
                nA += max(v, key=v.get) == "A_lin"
                nF += v["F_hybrid"] >= v["A_lin"]
                nD += v["D3lin"] >= v["A_lin"]
    L.append(f"  {m}: A best {nA}/{tot}; F>=A {nF}/{tot}; D3>=A {nD}/{tot}")
L.append("-- per-cell MC noise: sd of per-rep meanP / sqrt(n), typical")
sds = [v["sdMeanP"] / np.sqrt(v["n"]) for t in truths for v in r[t].values()]
L.append(f"  median SE {np.median(sds):.4f}, p90 {np.percentile(sds, 90):.4f}")
txt = "\n".join(L)
print(txt)
open(os.path.splitext(path)[0] + "_summary.txt", "w").write(txt)
