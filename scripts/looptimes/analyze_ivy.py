#!/usr/bin/env python3
"""Summarizes Ivy Path Test's loop-time log: per run, loop periods by phase and leg, each profiler section, the worst
loops and sections that spike on a schedule.

    adb pull /sdcard/FIRST/looptimes/IvyPathTest.csv
    python3 scripts/looptimes/analyze_ivy.py IvyPathTest.csv

Python 3 standard library only.
"""

import collections
import statistics
import sys

SPIKE_MS = 4.0


def parse(path):
    runs = []
    for line in open(path, encoding="utf-8"):
        line = line.rstrip("\n")
        if line.startswith("# run"):
            runs.append({"header": line[2:], "rows": [], "summary": None})
            continue
        if line.startswith("# summary"):
            runs[-1]["summary"] = line[2:]
            continue
        if line.startswith("#") or line.startswith("phase,") or not line:
            continue
        phase, t_us, period_us, leg, completion, busy, sections = line.split(",", 6)
        runs[-1]["rows"].append({
            "phase": phase,
            "t": int(t_us) / 1e6,
            "period": int(period_us) / 1000,
            "leg": leg,
            "completion": float(completion),
            "busy": busy == "true",
            "sections": {k: int(v) / 1000 for k, v in (x.split("=") for x in sections.strip(";").split(";") if x)},
        })
    return runs


def pct(values, q):
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, int(q * len(ordered)))]


def report(run_index, run):
    print(f"== run {run_index}: {run['header']}")
    for phase in ("init", "run"):
        rows = [r for r in run["rows"] if r["phase"] == phase]
        if not rows:
            continue
        periods = [r["period"] for r in rows]
        print(f"  {phase}: {len(rows)} loops over {rows[-1]['t']:.1f} s; period mean {statistics.mean(periods):.1f} / "
              f"p50 {pct(periods, .5):.1f} / p90 {pct(periods, .9):.1f} / p99 {pct(periods, .99):.1f} / "
              f"max {max(periods):.1f} ms; {sum(p > 25 for p in periods)} over 25 ms, {sum(p > 40 for p in periods)} "
              "over 40 ms")
        if phase != "run":
            continue
        by_leg = collections.defaultdict(list)
        for r in rows:
            by_leg[(r["leg"], r["busy"])].append(r)
        print("  by leg:")
        for (leg, busy), legs in by_leg.items():
            p = [r["period"] for r in legs]
            f = [r["sections"].get("follower.update", 0) for r in legs]
            print(f"    {leg:<18} busy={busy!s:<5} {len(legs):5} loops; period mean {statistics.mean(p):.1f} / "
                  f"p90 {pct(p, .9):.1f} ms; follower.update mean {statistics.mean(f):.1f} / p90 {pct(f, .9):.1f} ms")
        print("  sections (ms):")
        for key in sorted({k for r in rows for k in r["sections"]}):
            v = [r["sections"].get(key, 0) for r in rows]
            print(f"    {key:<24} mean {statistics.mean(v):6.2f}  p50 {pct(v, .5):6.2f}  p90 {pct(v, .9):6.2f}  "
                  f"max {max(v):6.1f}")
        print("  worst loops:")
        for r in sorted(rows, key=lambda r: -r["period"])[:12]:
            top = sorted(r["sections"].items(), key=lambda kv: -kv[1])[:3]
            print(f"    t {r['t']:6.2f} s  {r['period']:5.1f} ms  {r['leg']:<10} completion {r['completion']:.2f}  "
                  + "  ".join(f"{k} {v:.1f}" for k, v in top))
        print(f"  sections over {SPIKE_MS:.0f} ms, median gap between spikes:")
        for key in sorted({k for r in rows for k in r["sections"]}):
            times = [r["t"] for r in rows if r["sections"].get(key, 0) > SPIKE_MS]
            if len(times) < 3:
                continue
            gaps = [b - a for a, b in zip(times, times[1:])]
            print(f"    {key:<24} {len(times):4} spikes, median gap {statistics.median(gaps):.2f} s")
    if run["summary"]:
        print(f"  {run['summary']}")
    print()


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    for i, run in enumerate(parse(sys.argv[1]), 1):
        report(i, run)


if __name__ == "__main__":
    main()
