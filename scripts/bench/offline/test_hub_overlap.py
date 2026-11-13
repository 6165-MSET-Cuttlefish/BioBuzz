"""Offline checks for Bench: Photon's two-hubs-at-once phase in run_bench.py: the overlap reading, its record-only line
and summary table, fed synthetic result files shaped like the hub's. Run with:
python3 -I scripts/bench/offline/test_hub_overlap.py"""
import copy
import importlib.util
import json
import math
import pathlib
import sys
import tempfile

sys.dont_write_bytecode = True
HERE = pathlib.Path(__file__).resolve().parent
REPO = HERE.parents[2]
spec = importlib.util.spec_from_file_location("run_bench", REPO / "scripts/bench/run_bench.py")
rb = importlib.util.module_from_spec(spec)
spec.loader.exec_module(rb)
FIXTURES = REPO / "scripts/bench/photon-offline/runner/test_photon_checks.py"
F = {"__file__": str(FIXTURES)}
exec(compile(FIXTURES.read_text().split("# --- one workload phase")[0], str(FIXTURES), "exec"), F)
overlap_phase, ADDS_UP, OVERLAPS = F["overlap_phase"], F["OVERLAP_ADDS_UP"], F["OVERLAP_OVERLAPS"]

passed = 0
failed = []


def expect(label, cond):
    global passed
    if cond:
        passed += 1
    else:
        failed.append(label)
        print("FAIL", label)


def main_result(on8=None, on2=None, **sets_over):
    """A Bench: Photon result with the hub overlap phases: Photon off adds up (each call waits), on8/on4 as given."""
    modes = {"off": overlap_phase(0), "on8": overlap_phase(8, on8 or OVERLAPS), "on4": overlap_phase(4, on8 or OVERLAPS),
             "on2": overlap_phase(2, on2 or on8 or OVERLAPS)}
    for path, value in sets_over.items():
        mode, *rest = path.split(".")
        t = modes[mode]
        for k in rest[:-1]:
            t = t[k]
        if value is None:
            del t[rest[-1]]
        else:
            t[rest[-1]] = value
    return {"bench": "Photon", "complete": True, "hubOverlap": modes}


def through_file(result, folder, name):
    path = pathlib.Path(folder) / "raw" / f"{name}.json"
    path.parent.mkdir(exist_ok=True)
    path.write_text(json.dumps(result, indent=1))
    return json.loads(path.read_text())


def reading(main, topology="usb"):
    return rb.photon_overlap_verdict(main, topology)["reading"]


def line(main, topology="usb"):
    return rb.photon_overlap_verdict(main, topology)["line"]


# --- the ratio and its reading
r = rb.overlap_reading(10.86, 4.66, 6.20)
expect("reading: 12 together costing both hubs' sum adds up", r["reading"] == "adds up" and abs(r["ratio"] - 1) < 1e-9)
r = rb.overlap_reading(6.30, 4.66, 6.20)
expect("reading: 12 together near the slower hub alone overlaps",
       r["reading"] == "overlaps" and abs(r["full"] - 6.20 / 10.86) < 1e-9 and abs(r["ratio"] - 6.30 / 10.86) < 1e-9)
expect("reading: halfway is in between", rb.overlap_reading(8.5, 4.66, 6.20)["reading"] == "in between")
expect("reading: just under a third hidden adds up", rb.overlap_reading(8.01, 3, 6)["reading"] == "adds up")
expect("reading: just over a third hidden is in between", rb.overlap_reading(7.99, 3, 6)["reading"] == "in between")
expect("reading: just under two thirds hidden is in between", rb.overlap_reading(7.01, 3, 6)["reading"] == "in between")
expect("reading: just over two thirds hidden overlaps", rb.overlap_reading(6.99, 3, 6)["reading"] == "overlaps")
too_high = rb.overlap_reading(13.0, 4.66, 6.20)
expect("reading: 12 together costing more than both hubs' sum adds up, never overlaps",
       too_high["reading"] == "adds up" and too_high["ratio"] > 1 and too_high["hidden"] < 0)
expect("reading: 12 together below the slower hub alone still overlaps",
       rb.overlap_reading(5.0, 4.66, 6.20)["reading"] == "overlaps")
expect("reading: the hubs' order doesn't matter",
       rb.overlap_reading(8.5, 6.20, 4.66)["ratio"] == rb.overlap_reading(8.5, 4.66, 6.20)["ratio"])
for bad, args in (("a negative 12-together time", (-6.3, 4.66, 6.2)), ("a negative hub time", (6.3, -4.66, 6.2)),
                  ("a zero hub time", (6.3, 0, 6.2)), ("a NaN", (math.nan, 4.66, 6.2)),
                  ("a missing time", (6.3, None, 6.2)), ("a boolean", (True, 4.66, 6.2))):
    expect(f"reading: {bad} gives no reading", rb.overlap_reading(*args) is None)

with tempfile.TemporaryDirectory() as folder:
    adds_up = through_file(main_result(on8=ADDS_UP), folder, "photon_1_adds_up")
    overlaps = through_file(main_result(on8=OVERLAPS), folder, "photon_1_overlaps")
    between = through_file(main_result(on8=dict(ADDS_UP, **{"6 + 6": 8.6, "6 + 6 alternating": 8.5})), folder,
                           "photon_1_between")

    # --- the record-only line
    expect("verdict: a synthetic adds-up run reads adds up", reading(adds_up) == "adds up")
    expect("verdict: a synthetic overlapping run reads overlaps", reading(overlaps) == "overlaps")
    expect("verdict: a run halfway reads in between", reading(between) == "in between")
    v = rb.photon_overlap_verdict(overlaps, "usb")
    expect("verdict: the line names the reading, both ratios of both metrics and the full-overlap mark",
           "**overlaps** at 8 parallel" in v["line"] and "0.64 hub by hub" in v["line"]
           and "0.59 alternating" in v["line"] and "0.57 that they overlap fully" in v["line"]
           and "until the calls return, 0.64 hub by hub, 0.59 alternating" in v["line"])
    expect("verdict: the line says it is record only", "record only: not a check, not in the Photon verdict" in v["line"])
    mixed = main_result(on8=dict(ADDS_UP, **{"6 + 6": 10.4, "6 + 6 alternating": 6.5}))
    v = rb.photon_overlap_verdict(mixed, "usb")
    expect("verdict: it reads the order that hid more", v["reading"] == "overlaps" and v["order"] == "alternating")
    expect("verdict: it is read at 8 parallel, not at 2", reading(main_result(on8=ADDS_UP, on2=OVERLAPS)) == "adds up")
    ret_only = copy.deepcopy(adds_up)
    for key, ms in OVERLAPS.items():
        ret_only["hubOverlap"]["on8"]["sets"][key]["returnMs"]["p50"] = ms
    expect("verdict: it is read on the drained time, not the return time", reading(ret_only) == "adds up")
    expect("verdict: Photon off, the control, adds up", rb.overlap_reading(
        *[adds_up["hubOverlap"]["off"]["sets"][k]["drainedMs"]["p50"] for k in ("6 + 6", "6 + 0", "0 + 6")])["reading"]
        == "adds up")

    # --- what it must not read
    expect("verdict: no hub overlap result on USB is not measured", reading({"bench": "Photon"}) is None
           and "not measured" in line({"bench": "Photon"}))
    expect("verdict: no Bench: Photon result at all is not measured", reading(None) is None)
    no_on8 = copy.deepcopy(overlaps)
    del no_on8["hubOverlap"]["on8"]
    expect("verdict: a run without the 8-parallel phase is not measured", reading(no_on8) is None)
    no_eh = main_result(**{"on8.sets.0 + 6": None})
    expect("verdict: a missing set is not measured and named", reading(no_eh) is None and "no 0 + 6" in line(no_eh))
    short = main_result(**{"on8.sets.6 + 0.drainedMs.count": 299})
    expect("verdict: under 300 loops is not measured", reading(short) is None and "299 loops" in line(short))
    negative = main_result(**{"on8.sets.6 + 6.drainedMs.p50": -6.9, "on8.sets.6 + 6 alternating.drainedMs.p50": -6.4})
    expect("verdict: negative times are not measured", reading(negative) is None)
    one_order = main_result(**{"on8.sets.6 + 6 alternating": None})
    expect("verdict: one order missing still reads the other and says what it couldn't read",
           reading(one_order) == "overlaps" and "Not read: no 6 + 6 alternating" in line(one_order))
    expect("verdict: RS485 isn't run and says why, whatever the result holds",
           reading(overlaps, "rs485") is None and "not run: the Expansion Hub is on RS485" in line(overlaps, "rs485"))
    expect("verdict: a Control Hub alone isn't run and says why",
           reading(overlaps, "none") is None and "not run: there is no Expansion Hub" in line(overlaps, "none"))

    # --- the summary table
    text = "\n".join(rb.photon_overlap_summary(overlaps, "usb"))
    expect("summary: one column per mode", "| set | Photon off | 8 parallel | 4 parallel | 2 parallel |" in text)
    expect("summary: one row per set", all(f"| {label} |" in text for _, label in rb.PHOTON_OVERLAP_SETS))
    expect("summary: a ratio row per metric and order",
           all(f"| 12 / (6 + 6), {m}, {o} |" in text for m in ("returned", "drained") for o in ("hub by hub", "alternating")))
    expect("summary: p50 / p95 of return and drained per cell", "| 6 on the Control Hub | 4.66 / 6.06; 4.66 / 6.06 |" in text
           and "3.73 / 4.85; 4.66 / 6.06 |" in text)
    expect("summary: the record line once", text.count("**Two hubs at once**") == 1)
    expect("summary: loops per set and drain timeouts", "loops per set: Photon off 300, 8 parallel 300" in text
           and "drains past 500 ms: Photon off 0" in text)
    no_on2 = copy.deepcopy(overlaps)
    del no_on2["hubOverlap"]["on2"]
    expect("summary: a missing mode drops its column",
           "| set | Photon off | 8 parallel | 4 parallel |\n" in "\n".join(rb.photon_overlap_summary(no_on2, "usb")))
    rs_text = rb.photon_overlap_summary(overlaps, "rs485")
    expect("summary: RS485 has the heading and why, no table", len(rs_text) == 3 and "| set |" not in "\n".join(rs_text))

    # --- record only: the reading never changes a check or the Photon verdict
    def checks(main):
        return rb.photon_workload_checks("phases", "", {f"hubOverlap {m}": ph for m, ph in main["hubOverlap"].items()})

    a, b = checks(adds_up), checks(overlaps)
    expect("record only: adds up and overlaps give the same checks, all passing",
           [(c.name, c.ok) for c in a] == [(c.name, c.ok) for c in b] and all(c.ok for c in a) and len(a) == 4)
    expect("record only: and the same Photon verdict",
           rb.photon_verdict(a, True, True, True, True, 200) == rb.photon_verdict(b, True, True, True, True, 200))
    broken = copy.deepcopy(overlaps)
    broken["hubOverlap"]["on2"]["maxInFlight"]["controlHub"] = 3
    bad = checks(broken)
    expect("record only: a broken phase invariant still fails its check and the verdict",
           not all(c.ok for c in bad) and rb.photon_verdict(bad, True, True, True, True, 200)[0] == "NOT SAFE"
           and reading(broken) == "overlaps")

print(f"{passed} passed, {len(failed)} failed")
sys.exit(1 if failed else 0)
