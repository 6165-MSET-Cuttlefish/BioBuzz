"""Collects study1.json, study2_4.json, study3.json, aimcheck.txt into results/RESULTS.txt (compact tables)."""
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import shotcore as sc
from evalcore import fmt_table

RD = os.path.join(sc.HERE, "results")
L = []


def P(*a):
    L.append(" ".join(str(x) for x in a))


def short(n):
    return {"T1_vac": "T1vac", "T2_10g_cd.6": "T2-10g.6", "T2_10g_cd.4": "T2-10g.4", "T2_25g_cd.5": "T2-25g.5",
            "T2_45g_cd.6": "T2-45g.6", "T2_45g_cd.4": "T2-45g.4", "T3_25g_cd.5_cl.15": "T3-25g-cl.15",
            "T3_25g_cd.5_cl.30": "T3-25g-cl.3", "T3_10g_cd.6_cl.30": "T3-10g-cl.3"}[n]


def study1():
    r = json.load(open(os.path.join(RD, "study1.json")))
    P("=" * 100)
    P("STUDY 1  shot family (straight-on, hood theta 30-80 deg, exit speed = 0.09 in/s/rpm * (1-0.04h) * rpm)")
    P("Window = scoring set at fixed other setting; 'rpmWin' = widest rpm interval at any one hood; 'hoodWin' = widest hood interval at any one rpm")
    rows = []
    for n, x in r.items():
        w = {d["D"]: d for d in x["windows"]}
        row = {"truth": short(n)}
        for D in (36, 84, 132):
            row[f"rpmWin@{D}"] = f"{w[D]['max_rpm_window']:.0f} ({w[D]['max_rpm_window_pct']:.1f}%)"
            row[f"hoodWin@{D}"] = f"{w[D]['max_hood_window_deg']:.1f}deg"
        row["theta@36"] = f"{w[36]['theta_range_deg'][0]:.0f}-{w[36]['theta_range_deg'][1]:.0f}"
        row["theta@132"] = f"{w[132]['theta_range_deg'][0]:.0f}-{w[132]['theta_range_deg'][1]:.0f}"
        row["rpm@36"] = f"{w[36]['rpm_range'][0]:.0f}-{w[36]['rpm_range'][1]:.0f}"
        row["rpm@132"] = f"{w[132]['rpm_range'][0]:.0f}-{w[132]['rpm_range'][1]:.0f}"
        c = x["const"]
        row["constRpm"] = "infeasible" if c["constRpm"] is None else f"{c['constRpm']['rpm']:.0f}"
        row["constHood"] = "infeasible" if c["constHood"] is None else f"{c['constHood']['theta']:.1f}deg +-{c['constHood']['min_rpm_halfwidth']:.0f}rpm"
        rows.append(row)
    P(fmt_table(rows, list(rows[0].keys())))
    P("")
    P("Policies, averaged over D=36..132 (step 12). rpmTol/hoodTol/distTol = symmetric half-width that still scores (noise-free);")
    P("rpmDef = deficit tolerated; P15 = P(score) with exit-speed sigma 1.5% + angle sigma 0.5 deg; Pc = P15 noise + rpm err sigma 40 + distance err sigma 3 in")
    pols = ["maxP", "minRpmSens", "minDistSens", "centre", "minTOF", "constHood"]
    metrics = [("P15", "mean"), ("Pcomb", "mean"), ("Pcomb", "min"), ("rpm_tol", "mean"), ("rpm_def", "min"), ("dist_tol", "mean"),
               ("dist_tol", "min"), ("hood_tol_deg", "mean"), ("tof", "mean"), ("tof", "max")]
    for metric, how in metrics:
        rows = []
        for n, x in r.items():
            row = {"truth": short(n)}
            for p in pols:
                pts = x["policies"].get(p)
                if not pts:
                    row[p] = "n/a"
                    continue
                a = np.array([q[metric] for q in pts], float)
                row[p] = float(np.nanmean(a) if how == "mean" else np.nanmin(a) if how == "min" else np.nanmax(a))
            rows.append(row)
        P(f"-- {how} {metric}")
        P(fmt_table(rows, ["truth"] + pols, "{:.3g}"))
    P("")
    P("maxP / minDistSens operating points (T2-25g.5 and T3-10g-cl.3): D: rpm / theta / tof / rpmTol / distTol / dw per 100rpm / dw per in of D")
    for n in ("T2_25g_cd.5", "T3_10g_cd.6_cl.30", "T1_vac"):
        for p in ("maxP", "minDistSens"):
            pts = r[n]["policies"][p]
            P(f"  {short(n)} {p}: " + "; ".join(f"{q['D']}:{q['rpm']:.0f}/{q['theta']:.0f}/{q['tof']:.2f}/{q['rpm_tol']:.0f}/{q['dist_tol']:.0f}/{q['dw_d100rpm']:.1f}/{q['dw_dD']:.2f}" for q in pts))


def study1_desc():
    pols = ["maxP", "minRpmSens", "minDistSens", "centre", "minTOF", "constHood"]
    base = json.load(open(os.path.join(RD, "study1.json")))
    P("-- descent-angle sensitivity: entry must descend at >= 0 / 15 / 30 deg below horizontal (default 0, i.e. vz < 0)")
    P("   cells: mean Pcomb / mean tof (s) / mean distTol (in) / mean rpmTol")
    for g in ("0", "15", "30"):
        path = os.path.join(RD, "study1.json" if g == "0" else f"study1_desc{g}.json")
        if not os.path.exists(path):
            continue
        r = json.load(open(path))
        rows = []
        for n in ("T1_vac", "T2_10g_cd.6", "T2_25g_cd.5", "T3_10g_cd.6_cl.30"):
            if n not in r:
                continue
            row = {"truth": short(n), "minDescent": g}
            for p in pols:
                pts = r[n]["policies"].get(p)
                if not pts:
                    row[p] = "n/a"
                    continue
                a = lambda k: np.nanmean([q[k] for q in pts])
                row[p] = f"{a('Pcomb'):.2f}/{a('tof'):.2f}/{a('dist_tol'):.0f}/{a('rpm_tol'):.0f}"
            c = r[n]["const"]
            row["constRpmFeasible"] = "no" if c["constRpm"] is None else f"{c['constRpm']['rpm']:.0f}"
            row["constHoodFeasible"] = "no" if c["constHood"] is None else f"{c['constHood']['theta']:.0f}deg"
            rows.append(row)
        P(fmt_table(rows, ["truth", "minDescent"] + pols + ["constRpmFeasible", "constHoodFeasible"]))


def study2():
    path = os.path.join(RD, "study2_4.json")
    if not os.path.exists(path):
        P("STUDY 2: not run")
        return
    r = json.load(open(path))
    truths = list(r.keys())
    reps = ["A_lin", "B_pchip", "C2_quad", "C3_cubic", "D2vac", "D3lin", "D4lin", "F_hybrid", "E_indep", "E_trim1", "E_solverAim"]
    scheds = ["maxP", "minDist", "wander"]
    spacings = ["even4", "even5", "even6", "even8", "uneven4", "uneven5", "uneven6", "uneven8"]

    def get(t, sched, sp, scen, rep, k="meanP"):
        return r[t]["rep"][f"{sched}|{sp}|{scen}|{rep}"][k]

    def avg(t, scen, rep, k="meanP", scheds_=scheds, sps=spacings):
        return float(np.mean([get(t, s, sp, scen, rep, k) for s in scheds_ for sp in sps]))

    P("=" * 100)
    P("STUDY 2  representations. 80 simulated calibrations per (truth, tuner schedule, spacing); evaluated at 200 unseen D in [36,132].")
    P("meanP = mean P(score) with shot noise (1.5%, 0.5 deg); tuner point uniform in the central 60% of the rpm window then of the hood window.")
    P("Schedules: maxP = tuner aims for the most tolerant shot; minDist = the distance-tolerant (steeper) shot; wander = hood anywhere in the central 60% of the family.")
    for scen in ("clean",):
        for s in scheds + ["ALL"]:
            rows = []
            for rep in reps:
                row = {"rep": rep}
                for t in truths:
                    row[short(t)] = avg(t, scen, rep, scheds_=scheds if s == "ALL" else [s])
                row["mean"] = float(np.mean([row[short(t)] for t in truths]))
                row["worst"] = float(np.min([row[short(t)] for t in truths]))
                rows.append(row)
            P(f"-- {scen}, schedule={s}: meanP averaged over 8 spacings")
            P(fmt_table(rows, ["rep"] + [short(t) for t in truths] + ["mean", "worst"], "{:.3f}"))
    P("-- clean: mean over truths and schedules, by spacing (meanP)")
    rows = []
    for rep in reps:
        row = {"rep": rep}
        for sp in spacings:
            row[sp] = float(np.mean([get(t, s, sp, "clean", rep) for t in truths for s in scheds]))
        rows.append(row)
    P(fmt_table(rows, ["rep"] + spacings, "{:.3f}"))
    P("-- clean: mean over truths and schedules, by spacing (mean of per-calibration worst-distance P = minP)")
    rows = []
    for rep in reps:
        row = {"rep": rep}
        for sp in spacings:
            row[sp] = float(np.mean([get(t, s, sp, "clean", rep, "minP") for t in truths for s in scheds]))
        rows.append(row)
    P(fmt_table(rows, ["rep"] + spacings, "{:.3f}"))
    P("-- scenarios: mean over truths, schedules, spacings. bad = one interior point recorded +150 rpm; wear = exit-speed gain -5%;")
    P("   +add = one additive rpm trim found at 84 in (as ShotTuning.rpmTrim); +mul = one multiplicative trim found at 84 in")
    rows = []
    for rep in reps:
        row = {"rep": rep}
        for scen in ("clean", "bad", "wear", "wear+add", "wear+mul"):
            row[scen] = float(np.mean([avg(t, scen, rep) for t in truths]))
            row[scen + "_min"] = float(np.mean([avg(t, scen, rep, "minP") for t in truths]))
        row["rmsMiss_clean_in"] = float(np.mean([avg(t, "clean", rep, "rmsMiss") for t in truths]))
        rows.append(row)
    P(fmt_table(rows, ["rep", "clean", "bad", "wear", "wear+add", "wear+mul", "clean_min", "bad_min", "wear+add_min", "wear+mul_min", "rmsMiss_clean_in"], "{:.3f}"))
    P("-- scenario 'bad' by spacing (meanP, mean over truths/schedules)")
    rows = []
    for rep in reps:
        row = {"rep": rep}
        for sp in spacings:
            row[sp] = float(np.mean([get(t, s, sp, "bad", rep) for t in truths for s in scheds]))
        rows.append(row)
    P(fmt_table(rows, ["rep"] + spacings, "{:.3f}"))
    P("-- wear+mul per truth (meanP)")
    rows = []
    for rep in reps:
        row = {"rep": rep}
        for t in truths:
            row[short(t)] = avg(t, "wear+mul", rep)
        rows.append(row)
    P(fmt_table(rows, ["rep"] + [short(t) for t in truths], "{:.3f}"))
    P("-- rank of A_lin among {A,B,C2,C3,D2,D3,D4,F} by clean meanP, per truth x schedule (1 = best), averaged over spacings")
    core = ["A_lin", "B_pchip", "C2_quad", "C3_cubic", "D2vac", "D3lin", "D4lin", "F_hybrid"]
    rows = []
    for t in truths:
        row = {"truth": short(t)}
        for s in scheds:
            vals = {rep: avg(t, "clean", rep, scheds_=[s]) for rep in core}
            order = sorted(core, key=lambda k: -vals[k])
            row[s] = f"A#{order.index('A_lin') + 1} best={order[0]}({vals[order[0]]:.3f}) A={vals['A_lin']:.3f}"
        rows.append(row)
    P(fmt_table(rows, ["truth"] + scheds))
    P("-- best representation per (truth, schedule, spacing), clean meanP: counts")
    from collections import Counter
    cnt = Counter()
    for t in truths:
        for s in scheds:
            for sp in spacings:
                vals = {rep: get(t, s, sp, "clean", rep) for rep in core}
                cnt[max(vals, key=vals.get)] += 1
    P("   " + ", ".join(f"{k}:{v}" for k, v in cnt.most_common()))
    P("-- A_lin minus F_hybrid / D3lin / B_pchip, clean meanP, by N (mean over truths, schedules, even+uneven)")
    rows = []
    for N in ("4", "5", "6", "8"):
        sps = [f"even{N}", f"uneven{N}"]
        row = {"N": N}
        for rep in ("B_pchip", "C2_quad", "D3lin", "F_hybrid"):
            row[f"{rep}-A"] = float(np.mean([avg(t, "clean", rep, sps=sps) - avg(t, "clean", "A_lin", sps=sps) for t in truths]))
            row[f"{rep}-A_worst_truth"] = float(np.min([avg(t, "clean", rep, sps=sps) - avg(t, "clean", "A_lin", sps=sps) for t in truths]))
        rows.append(row)
    P(fmt_table(rows, list(rows[0].keys()), "{:+.3f}"))
    P("-- outlier detection ('bad' point = largest |fit residual|), rate")
    rows = []
    for t in truths:
        row = {"truth": short(t)}
        for k in ("D3lin", "D4lin", "C2_quad"):
            vals = [v for kk, v in r[t]["detect"].items() if kk.endswith("|" + k)]
            row[k] = float(np.mean(vals)) if vals else float("nan")
        for N in ("5", "8"):
            vals = [v for kk, v in r[t]["detect"].items() if kk.endswith("|D3lin") and kk.split("|")[1].endswith(N)]
            row[f"D3lin_N{N}"] = float(np.mean(vals))
        rows.append(row)
    P(fmt_table(rows, list(rows[0].keys()), "{:.2f}"))


def study2_nasty():
    path = os.path.join(RD, "study2_4_nasty.json")
    if not os.path.exists(path):
        return
    r = json.load(open(path))
    base = json.load(open(os.path.join(RD, "study2_4.json")))
    P("-- ROBUSTNESS: nonlinear shooter (hood bow beta 0.35, eta drop 8%, exit speed per rpm -10% per 3000 rpm above 3000);")
    P("   physics models keep their linear assumptions. 40 reps. cells: nasty meanP (baseline meanP). Mean over schedules and spacings.")
    reps = ["A_lin", "B_pchip", "C2_quad", "D2vac", "D3lin", "D4lin", "F_hybrid", "E_indep", "E_trim1"]
    for scen in ("clean", "bad", "wear+mul"):
        rows = []
        for rep in reps:
            row = {"rep": rep, "scenario": scen}
            for t in r:
                v = np.mean([x["meanP"] for k, x in r[t]["rep"].items() if k.endswith(f"|{scen}|{rep}")])
                b = np.mean([x["meanP"] for k, x in base[t]["rep"].items() if k.endswith(f"|{scen}|{rep}")])
                mn = np.mean([x["minP"] for k, x in r[t]["rep"].items() if k.endswith(f"|{scen}|{rep}")])
                row[short(t)] = f"{v:.3f} ({b:.3f}) min{mn:.2f}"
            rows.append(row)
        P(fmt_table(rows, ["rep", "scenario"] + [short(t) for t in r]))
    rows = []
    for t in r:
        row = {"truth": short(t)}
        for s in ("maxP", "minDist", "wander"):
            for rep in ("A_lin", "B_pchip", "D3lin", "F_hybrid"):
                row[f"{s}:{rep}"] = float(np.mean([x["meanP"] for k, x in r[t]["rep"].items() if k.startswith(s + "|") and k.endswith(f"|clean|{rep}")]))
        rows.append(row)
    P("   nasty, clean, by schedule:")
    P(fmt_table(rows, list(rows[0].keys()), "{:.3f}"))


def study4():
    path = os.path.join(RD, "study2_4.json")
    if not os.path.exists(path):
        return
    r = json.load(open(path))
    truths = list(r.keys())
    P("=" * 100)
    P("STUDY 4  lead time for shooting on the move. Reference = true effective lead time (integral of the decay of an inherited")
    P("lateral velocity; = TOF in vacuum). Table TOF measured at the N calibration distances by 240 fps video (+-1 frame), linear in D.")
    P("Errors over 200 unseen D on the shots actually fired (A_lin settings), averaged over schedules/spacings/reps.")
    cands = ["table_tof", "table_tof+bestKdrag", "phys_D3lin", "phys_D3lin+tof", "phys_E_indep", "truth_tof_as_lead"]
    for fired in ("A_lin", "F_hybrid"):
        rows = []
        for t in truths:
            row = {"truth": short(t)}
            for c in cands:
                v = [x for k, x in r[t]["tof"].items() if k.endswith(f"|{fired}|{c}")]
                row[c] = f"{np.mean([x['rms_ms'] for x in v]):.1f}/{np.mean([x['max_ms'] for x in v]):.1f}/{np.mean([x['bias_ms'] for x in v]):+.1f}"
            rows.append(row)
        P(f"-- fired={fired}: lead-time error rms/max/bias (ms)")
        P(fmt_table(rows, ["truth"] + cands))
    P("-- lateral miss (in) = robot speed x lead error; rms (max) over distances, fired=A_lin, worst truth for each method")
    rows = []
    for c in cands:
        rms = max(np.mean([x["rms_ms"] for k, x in r[t]["tof"].items() if k.endswith(f"|A_lin|{c}")]) for t in truths)
        mx = max(np.mean([x["max_ms"] for k, x in r[t]["tof"].items() if k.endswith(f"|A_lin|{c}")]) for t in truths)
        row = {"method": c}
        for v in (20, 40, 60):
            row[f"{v}in/s"] = f"{v * rms / 1000:.2f} ({v * mx / 1000:.2f})"
        rows.append(row)
    P(fmt_table(rows, ["method", "20in/s", "40in/s", "60in/s"]))
    P("-- table_tof (kDrag=0) rms lead error by N (ms), fired=A_lin, mean over truths")
    rows = []
    for c in ("table_tof", "table_tof+bestKdrag", "phys_D3lin+tof"):
        row = {"method": c}
        for N in ("4", "5", "6", "8"):
            v = [x["rms_ms"] for t in truths for k, x in r[t]["tof"].items() if k.endswith(f"|A_lin|{c}") and k.split("|")[1].endswith(N)]
            row["N" + N] = float(np.mean(v))
        rows.append(row)
    P(fmt_table(rows, ["method", "N4", "N5", "N6", "N8"], "{:.1f}"))


def study4b():
    path = os.path.join(RD, "study4b.json")
    if not os.path.exists(path):
        return
    r = json.load(open(path))
    P("-- STUDY 4b: TOF error vs the true TOF to the opening plane (not lead), on A_lin's fired shots; rms/max/bias ms; 30 reps")
    cands = ["table", "table_pchip", "phys_D3lin", "phys_D3lin+tof", "phys_E_indep"]
    for spn in ("even4", "even5", "even8", "uneven4"):
        rows = []
        for t in r:
            row = {"truth": short(t), "spacing": spn}
            for c in cands:
                v = [r[t][f"{s}|{spn}|{c}"] for s in ("maxP", "minDist")]
                row[c] = f"{np.mean([x['rms_ms'] for x in v]):.1f}/{np.mean([x['max_ms'] for x in v]):.1f}/{np.mean([x['bias_ms'] for x in v]):+.1f}"
            rows.append(row)
        P(fmt_table(rows, ["truth", "spacing"] + cands))


def study3():
    path = os.path.join(RD, "study3.json")
    if not os.path.exists(path):
        P("STUDY 3: not run")
        return
    r = json.load(open(path))
    P("=" * 100)
    P("STUDY 3  droop. Base shot = oracle schedule point at D (maxP or minDist); flywheel Delta rpm low at the shot.")
    keys = sorted(r.keys())
    P("-- hood capacity: max deficit with any scoring hood (connected branch) / with P>=0.8; hood slope deg per 100 rpm (lin fit 0..300 or to limit);")
    P("   lin dev = max deviation of centre-of-window hood from its straight line, as a fraction of the hood half-window")
    for k in keys:
        pd = r[k]["part1"]["perD"]
        P(f"  {k}:")
        P("    D        : " + " ".join(f"{q['D']:>6}" for q in pd))
        P("    theta0   : " + " ".join(f"{q['theta0']:6.1f}" for q in pd))
        P("    maxDef   : " + " ".join(f"{q['max_deficit_connected']:6.0f}" for q in pd))
        P("    maxDefP80: " + " ".join(f"{q['max_deficit_P80']:6.0f}" for q in pd))
        P("    deg/100  : " + " ".join(f"{q['slope_deg_per100']:6.2f}" for q in pd))
        P("    lin dev  : " + " ".join(f"{q['lin_maxdev_frac_halfwidth']:6.2f}" for q in pd))
    deltas = ["0", "50", "100", "150", "200", "250", "300"]
    methods = ["a_none", "b_droop1", "b_droop2", "c_gainConst", "c_gainSched", "d_physDelta", "d_physDelta+droopFit", "oracle"]
    for sched in ("maxP", "minDist"):
        rows = []
        for m in methods:
            row = {"method": m}
            for d in deltas:
                row[d] = float(np.mean([np.mean([q["P"] for q in r[k]["part1"]["methods"][m][d]]) for k in keys if k.endswith(sched)]))
            row["worstTruth@150"] = float(np.min([np.mean([q["P"] for q in r[k]["part1"]["methods"][m]["150"]]) for k in keys if k.endswith(sched)]))
            rows.append(row)
        P(f"-- Part 1 (oracle droop points, schedule {sched}): mean P over D and truths vs deficit (rpm)")
        P(fmt_table(rows, ["method"] + deltas + ["worstTruth@150"], "{:.3f}"))
    for sched in ("maxP", "minDist"):
        for bias in ("bias0", "bias30"):
            rows = []
            for m in methods[:-1]:
                row = {"method": m}
                for d in deltas:
                    row[d] = float(np.mean([r[k]["part2"][f"{bias}|{d}|{m}"] for k in keys if k.endswith(sched)]))
                row["worstTruth@150"] = float(np.min([r[k]["part2"][f"{bias}|150|{m}"] for k in keys if k.endswith(sched)]))
                rows.append(row)
            P(f"-- Part 2 (tuner-measured, 5 distances, 200 unseen D, schedule {sched}, deficit measurement {bias}): mean P vs deficit")
            P(fmt_table(rows, ["method"] + deltas + ["worstTruth@150"], "{:.3f}"))
    P("-- TOF change caused by the droop-compensating hood (ms), Part 1 oracle hood, mean |dTOF| over D and truths (max)")
    rows = []
    for sched in ("maxP", "minDist"):
        for m in ("a_none", "b_droop2", "oracle"):
            row = {"sched": sched, "method": m}
            for d in deltas[1:]:
                vals = [abs(q["dtof_ms"]) for k in keys if k.endswith(sched) for q in r[k]["part1"]["methods"][m][d] if math.isfinite(q["dtof_ms"])]
                row[d] = f"{np.mean(vals):.0f} ({np.max(vals):.0f})" if vals else "nan"
            rows.append(row)
    P(fmt_table(rows, ["sched", "method"] + deltas[1:]))
    P("-- burst model: 3 balls, flywheel drops 300 rpm per shot and recovers exponentially; t_rec = time to get back within 50 rpm.")
    P("   wait: feed only when within 50 rpm (P at Delta<=50, no comp); feed@120ms: next ball after 120 ms with that method. Part 2 numbers, maxP + minDist avg, bias0")
    rows = []
    for t_rec in (150, 250, 400):
        tau = t_rec / math.log(300 / 50)
        for t_feed in (120,):
            dlist = [0.0]
            dd = 0.0
            for _ in range(2):
                dd = (dd + 300) * math.exp(-t_feed / tau)
                dlist.append(dd)
            row = {"t_rec_ms": t_rec, "deficits": "/".join(f"{x:.0f}" for x in dlist)}
            for m in ("a_none", "b_droop2", "c_gainSched", "d_physDelta+droopFit"):
                tot = 0
                for x in dlist:
                    vals = []
                    for k in keys:
                        pts = [float(r[k]["part2"][f"bias0|{d}|{m}"]) for d in deltas]
                        vals.append(np.interp(min(x, 300), [float(d) for d in deltas], pts))
                    tot += np.mean(vals)
                row[m] = f"{tot:.2f} in {2 * t_feed}ms"
            wait = np.mean([np.mean([r[k]["part2"][f"bias0|{d}|a_none"] for d in ("0", "50")]) for k in keys]) * 3
            row["wait"] = f"{wait:.2f} in {2 * t_rec}ms"
            rows.append(row)
    P(fmt_table(rows, list(rows[0].keys())))


def aim():
    p = os.path.join(RD, "aimcheck.txt")
    if os.path.exists(p):
        P("=" * 100)
        P("AIM POINT CHECK")
        L.append(open(p).read())
    p = os.path.join(RD, "kdrag_equiv.txt")
    if os.path.exists(p):
        P("")
        L.append(open(p).read())


if __name__ == "__main__":
    study1()
    study1_desc()
    study2()
    study2_nasty()
    study3()
    study4()
    study4b()
    aim()
    txt = "\n".join(L)
    open(os.path.join(RD, "RESULTS.txt"), "w").write(txt)
    print(txt)
