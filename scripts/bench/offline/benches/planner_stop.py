"""Bench: Planner Stop offline: its verdicts on synthetic trial records (each one fails when weakened), and the bench end
to end on fake_hub.py with a fake Bench: Planner Stop that plans, reports and logs STOP on the hub's clock. No hub, no
adb, no network. Run with: python3 -I scripts/bench/offline/benches/planner_stop.py
"""
import contextlib
import copy
import datetime
import importlib.util
import io
import json
import pathlib
import signal
import sys
import tempfile
import threading
import time
import traceback

sys.dont_write_bytecode = True
HERE = pathlib.Path(__file__).resolve().parent
OFFLINE = HERE.parent
WORKTREE = OFFLINE.parents[2]
OUT = OFFLINE / "out" / "planner_stop"
RUNNER = WORKTREE / "scripts/bench/run_bench.py"
OPMODE = "Bench: Planner Stop"

passed = 0
failed = []


def expect(label, cond):
    global passed
    if cond:
        passed += 1
    else:
        failed.append(label)
        print("FAIL", label)


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


rb = load("run_bench", RUNNER)
fake_hub = load("fake_hub", OFFLINE / "fake_hub.py")


def summary(count=10, p50=12.0, mx=60.0):
    return {"count": count, "mean": p50, "min": 1.0, "p50": p50, "p90": mx * 0.8, "p95": mx * 0.9, "p99": mx, "max": mx}


def trial(balls, delay, **kw):
    """A sliced trial as _planner_stop records it: STOP 40 ms into a slice, idle 300 ms later by the hub's clock."""
    t = {"balls": balls, "delay": delay, "warmSeconds": 4.0, "nextPlanSeconds": 1.9, "plans": [1900.0, 1850.0, 1870.0],
         "planComputeMs": [1450.0, 1420.0, 1440.0], "planLoops": [70, 69, 70], "stepMs": summary(mx=120.0),
         "overrunMs": summary(p50=1.0, mx=100.0), "loopPeriodMs": summary(p50=26.0, mx=130.0),
         "layout": f"{balls} balls on both sides of the rail", "sliceMs": 20.0, "firstPlanInProcess": False,
         "stopSeen": True, "planAtStop": 4, "msIntoPlan": 80.0, "inSlice": True, "msIntoSlice": 10.0,
         "sliceLeftMs": 12.0, "stopToStopPassMs": 30.0, "stopSeconds": 0.45, "restarted": False, "markers": [],
         "hubStopToIdleMs": 300.0, "pidBefore": 4000, "pidAfter": 4000}
    t.update(kw)
    return t


def good_trials():
    return [trial(b, d) for b in rb.PLANNER_STOP_BALLS for d in rb.PLANNER_STOP_DELAYS_S]


def named_trials():
    """--planner-stop's sliced trials: the default run's round, then rounds 2 and 3."""
    return good_trials() + [trial(b, d, repeat=r) for r in range(2, rb.PLANNER_STOP_REPEATS + 1)
                            for b in rb.PLANNER_STOP_BALLS for d in rb.PLANNER_STOP_DELAYS_S]


NAMED_SLICED = len(rb.PLANNER_STOP_BALLS) * len(rb.PLANNER_STOP_DELAYS_S) * rb.PLANNER_STOP_REPEATS
NAMED_BEFORE_CAMERA = NAMED_SLICED + len(rb.PLANNER_UNSLICED_DELAYS_S)


def changed(trials, balls, delay, **kw):
    for t in trials:
        if t["balls"] == balls and t["delay"] == delay:
            t.update(kw)
    return trials


def failed_trial(trials, balls, delay, why="Bench: Planner Stop to INIT timed out"):
    return [{"balls": balls, "delay": delay, "failed": why} if t["balls"] == balls and t["delay"] == delay else t
            for t in trials]


def key(name):
    if name.startswith("planner stop ") and " balls: " in name:
        balls = int(name.split()[2])
        kind = "restart" if "never restarts" in name else "idle" if "reaching idle" in name else "slice"
        return kind, balls
    if name.startswith("planner stop unsliced control"):
        return "control", None
    if name.startswith("planner stop with the webcam: "):
        kind = ("restart" if "never restarts" in name else "idle" if "reaching onEnd()" in name
                else "stop" if "CellTipCamera.stop() under" in name else "back" if "event loop came back" in name
                else "worst" if "worst case" in name else name)
        return kind, "webcam"
    return name, None


def only_fails(checks, *keys):
    return {key(name) for name, ok, _, _ in checks if not ok} == set(keys)


def detail(checks, wanted):
    return next((d for name, ok, d, _ in checks if key(name) == wanted), "")


def unsliced_trial(delay, idle=1200.0, **kw):
    """An unsliced trial: the whole plan in the step running at STOP."""
    return trial(4, delay, unsliced=True, plans=[1500.0, 1480.0, 1510.0], planComputeMs=[1500.0, 1480.0, 1510.0],
                 planLoops=[1, 1, 1], stepMs=summary(count=4, p50=1490.0, mx=1510.0), inSlice=True,
                 msIntoSlice=60.0, sliceLeftMs=1440.0, stopToStopPassMs=1450.0, hubStopToIdleMs=idle, **kw)


def unsliced(*idles, **kw):
    return [unsliced_trial(d, i, **kw) for d, i in zip(rb.PLANNER_UNSLICED_DELAYS_S, idles)]


SLICED_CASES = [
    ("every sliced STOP idle in 300 ms with slices under 300 ms: all pass", good_trials, lambda v: only_fails(v)),
    ("three rounds of them (--planner-stop): all pass", named_trials, lambda v: only_fails(v)),
    ("three rounds with one round-3 trial lost: that ball count's two checks fail",
     lambda: [t for t in named_trials() if (t["balls"], t["delay"], t.get("repeat")) != (5, 0.3, 3)],
     lambda v: only_fails(v, ("restart", 5), ("idle", 5))),
    ("a round-2 STOP at 950 ms fails the idle check, naming it",
     lambda: [dict(t, hubStopToIdleMs=950.0) if (t["balls"], t["delay"], t.get("repeat")) == (4, 0.6, 2) else t
              for t in named_trials()],
     lambda v: only_fails(v, ("idle", 4))),
    ("an app restart fails that ball count's restart check alone",
     lambda: changed(good_trials(), 4, 0.3, restarted=True), lambda v: only_fails(v, ("restart", 4))),
    ("a stuck-in-stop marker without a restart fails it too",
     lambda: changed(good_trials(), 5, 0.05, markers=["was stuck in stop()"]), lambda v: only_fails(v, ("restart", 5))),
    ("a STOP at 900 ms by the hub's clock fails the idle check",
     lambda: changed(good_trials(), 4, 0.6, hubStopToIdleMs=900.0), lambda v: only_fails(v, ("idle", 4))),
    ("one at 899 ms passes", lambda: changed(good_trials(), 4, 0.6, hubStopToIdleMs=899.0), lambda v: only_fails(v)),
    ("no '***STOP - OPMODE***' line (None) fails the idle check",
     lambda: changed(good_trials(), 5, 0.3, hubStopToIdleMs=None), lambda v: only_fails(v, ("idle", 5))),
    ("a trial that didn't finish fails both STOP checks for its ball count",
     lambda: failed_trial(good_trials(), 5, 0.6), lambda v: only_fails(v, ("restart", 5), ("idle", 5))),
    ("a 4-ball slice of 300 ms fails the slice check",
     lambda: changed(good_trials(), 4, 0.05, stepMs=summary(mx=300.0)), lambda v: only_fails(v, ("slice", 4))),
    ("a 5-ball slice of 400 ms isn't judged", lambda: changed(good_trials(), 5, 0.05, stepMs=summary(mx=400.0)),
     lambda v: only_fails(v)),
    ("a 4-ball trial with no slice recorded fails the slice check",
     lambda: changed(good_trials(), 4, 0.3, stepMs={"count": 0}), lambda v: only_fails(v, ("slice", 4))),
    ("no 4-ball trial finishing fails all three 4-ball checks",
     lambda: [{"balls": 4, "delay": d, "failed": "x"} for d in rb.PLANNER_STOP_DELAYS_S]
     + [trial(5, d) for d in rb.PLANNER_STOP_DELAYS_S],
     lambda v: only_fails(v, ("restart", 4), ("idle", 4), ("slice", 4))),
]


def camera_trial(kind, delay, repeat, **kw):
    """A webcam trial: a 1 ms CellTipCamera.stop(), onEnd() 40 ms after STOP (80 for the worst case, which STOPs before
    the first frame) and '***STOP - OPMODE***' at 700 ms, after EasyOpenCV's close, by the hub's clock."""
    worst = kind == "worst"
    t = trial(4, delay, kind=kind, repeat=repeat, hubStopToIdleMs=700.0, hubStopToOnEndMs=80.0 if worst else 40.0,
              framesAtStop=0 if worst else 30, framesAtStart=0 if worst else None,
              cellTipStopMs=2.0 if worst else 1.0, sliceAfterStopMs=45.0 if worst else 12.0, tailMs=5.0,
              stopPassToOnEndMs=25.0 if worst else 20.0, startToFirstSliceMs=4.0 if worst else None)
    if worst:
        t.update(initSeenS=0.15, startSentS=0.2, stopSentS=0.25)
    t.update(kw)
    return t


def camera_trials():
    out = [camera_trial("replan", d, r) for r in range(1, rb.PLANNER_CAMERA_REPEATS + 1) for d in rb.PLANNER_STOP_DELAYS_S]
    out += [camera_trial("init", d, r) for r in range(1, rb.PLANNER_CAMERA_REPEATS + 1)
            for d in rb.PLANNER_CAMERA_INIT_DELAYS_S]
    return out + [camera_trial("worst", rb.PLANNER_CAMERA_WORST_STOP_S, r)
                  for r in range(1, rb.PLANNER_CAMERA_WORST_TRIALS + 1)]


def camera_changed(trials, kind, repeat, **kw):
    for t in trials:
        if t["kind"] == kind and t["repeat"] == repeat:
            t.update(kw)
    return trials


def every(trials, kind, **kw):
    for t in trials:
        if t["kind"] == kind:
            t.update(kw)
    return trials


ALL_COMPLETE = (("restart", "webcam"), ("idle", "webcam"), ("stop", "webcam"), ("back", "webcam"))

CAMERA_CASES = [
    ("20 webcam trials with onEnd() 40-80 ms after STOP and the worst case reproduced: all pass",
     lambda: (good_trials(), [], camera_trials()), lambda c: only_fails(c)),
    ("one trial reaching onEnd() at 905 ms fails the window check alone",
     lambda: (good_trials(), [], camera_changed(camera_trials(), "init", 2, hubStopToOnEndMs=905.0)),
     lambda c: only_fails(c, ("idle", "webcam"))),
    ("one at 899 ms passes",
     lambda: (good_trials(), [], camera_changed(camera_trials(), "worst", 3, hubStopToOnEndMs=899.0)),
     lambda c: only_fails(c)),
    ("a STOP with no onEnd line fails the window check",
     lambda: (good_trials(), [], camera_changed(camera_trials(), "replan", 1, hubStopToOnEndMs=None)),
     lambda c: only_fails(c, ("idle", "webcam"))),
    ("EasyOpenCV's close after the window taking STOP to idle to 1500 ms isn't judged",
     lambda: (good_trials(), [], every(camera_trials(), "init", hubStopToIdleMs=1500.0)), lambda c: only_fails(c)),
    ("a STOP with no '***STOP - OPMODE***' line fails the event-loop check alone",
     lambda: (good_trials(), [], camera_changed(camera_trials(), "replan", 3, hubStopToIdleMs=None)),
     lambda c: only_fails(c, ("back", "webcam"))),
    ("a 100 ms CellTipCamera.stop() fails the stop() check alone",
     lambda: (good_trials(), [], camera_changed(camera_trials(), "init", 2, cellTipStopMs=100.0)),
     lambda c: only_fails(c, ("stop", "webcam"))),
    ("one of 99 ms passes",
     lambda: (good_trials(), [], camera_changed(camera_trials(), "replan", 2, cellTipStopMs=99.0)),
     lambda c: only_fails(c)),
    ("a trial with no stop() time fails the stop() check",
     lambda: (good_trials(), [], camera_changed(camera_trials(), "worst", 4, cellTipStopMs=None)),
     lambda c: only_fails(c, ("stop", "webcam"))),
    ("the old close inside stop() (783 ms, onEnd() at 1084 ms) fails the stop() and window checks",
     lambda: (good_trials(), [], every(camera_trials(), "worst", cellTipStopMs=783.0, hubStopToOnEndMs=1084.0,
                                       hubStopToIdleMs=1100.0)),
     lambda c: only_fails(c, ("idle", "webcam"), ("stop", "webcam"))),
    ("an app restart fails the restart check",
     lambda: (good_trials(), [], camera_changed(camera_trials(), "replan", 2, restarted=True, pidAfter=4001)),
     lambda c: only_fails(c, ("restart", "webcam"))),
    ("a stuck-in-stop marker fails it too",
     lambda: (good_trials(), [], camera_changed(camera_trials(), "init", 1, markers=["was stuck in stop()"])),
     lambda c: only_fails(c, ("restart", "webcam"))),
    ("a trial that didn't finish fails the four checks that count trials",
     lambda: (good_trials(), [], [t if (t["kind"], t["repeat"], t["delay"]) != ("init", 1, 0.3)
                                  else {"kind": "init", "repeat": 1, "delay": 0.3, "balls": 4, "failed": "x"}
                                  for t in camera_trials()]),
     lambda c: only_fails(c, *ALL_COMPLETE)),
    ("fewer trials than planned fail them too",
     lambda: (good_trials(), [], camera_trials()[:-6] + camera_trials()[-5:]),
     lambda c: only_fails(c, *ALL_COMPLETE)),
    ("worst-case STOPs after the first frame: not reproduced, which fails",
     lambda: (good_trials(), [], every(camera_trials(), "worst", framesAtStop=3)),
     lambda c: only_fails(c, ("worst", "webcam"))),
    ("one worst-case STOP before the first frame is enough",
     lambda: (good_trials(), [], camera_changed(every(camera_trials(), "worst", framesAtStop=3), "worst", 5,
                                                framesAtStop=0)), lambda c: only_fails(c)),
    ("a STOP before the first frame with no planner slice to wait for: not reproduced",
     lambda: (good_trials(), [], every(camera_trials(), "worst", sliceAfterStopMs=0.0)),
     lambda c: only_fails(c, ("worst", "webcam"))),
    ("a slice begun after STOP in the same loop counts as waited for",
     lambda: (good_trials(), [], every(camera_trials(), "worst", inSlice=False, sliceLeftMs=None,
                                       sliceAfterStopMs=21.0)), lambda c: only_fails(c)),
    ("an INIT trial with no frame and a slice doesn't stand in for the worst case",
     lambda: (good_trials(), [], camera_changed(every(camera_trials(), "worst", framesAtStop=3), "init", 1,
                                                framesAtStop=0, sliceAfterStopMs=40.0)),
     lambda c: only_fails(c, ("worst", "webcam"))),
    ("worst-case trials that didn't finish don't reproduce it",
     lambda: (good_trials(), [], [t if t["kind"] != "worst" else dict(t, failed="x") for t in camera_trials()]),
     lambda c: only_fails(c, *ALL_COMPLETE, ("worst", "webcam"))),
    ("without webcam trials there is no webcam check", lambda: (good_trials(), [], []),
     lambda c: not [n for n, *_ in c if "webcam" in n]),
]


def run_cases(checks, cases):
    results = []
    for label, make, predicate in cases:
        try:
            args = make()
            ok = predicate(checks(*(args if isinstance(args, tuple) else (args,))))
        except Exception as e:
            ok = False
            label = f"{label} ({type(e).__name__}: {e})"
        results.append((label, ok))
    return results


CONTROL_CASES = [
    ("an unsliced STOP at 1200 ms: the control fired, and every check passes",
     lambda: (good_trials(), unsliced(1200.0, 1210.0, 1190.0)),
     lambda c: only_fails(c) and rb.planner_control_verdict(unsliced(1200.0, 1210.0, 1190.0))[0] == "fired"
     and "unproven" not in detail(c, ("idle", 4))),
    ("unsliced STOPs at 400 ms: the control didn't fire, its check fails, and the 4-ball STOP checks say unproven",
     lambda: (good_trials(), unsliced(400.0, 400.0, 400.0)),
     lambda c: only_fails(c, ("control", None)) and rb.planner_control_verdict(unsliced(400.0, 400.0, 400.0))[0]
     == "not fired" and "unproven on this layout" in detail(c, ("idle", 4))
     and "unproven on this layout" in detail(c, ("restart", 4)) and "unproven" not in detail(c, ("idle", 5))),
    ("one of three at 900 ms is enough", lambda: (good_trials(), unsliced(400.0, 900.0, 400.0)),
     lambda c: only_fails(c)),
    ("one at 899 ms isn't", lambda: (good_trials(), unsliced(400.0, 899.0, 400.0)),
     lambda c: only_fails(c, ("control", None))),
    ("an app restart (no idle line) fires it",
     lambda: (good_trials(), unsliced(400.0, None, 400.0)[:1] + [unsliced_trial(0.15, None, restarted=True,
                                                                               pidAfter=4001)]
              + unsliced(400.0, 400.0, 400.0)[2:]), lambda c: only_fails(c)),
    ("a stuck-in-stop marker fires it", lambda: (good_trials(), unsliced(400.0, 400.0, 400.0)[:2]
                                                 + [unsliced_trial(0.3, 700.0, markers=["was stuck in stop()"])]),
     lambda c: only_fails(c)),
    ("a missing idle line with no restart isn't an overrun",
     lambda: (good_trials(), unsliced(400.0, None, 400.0)), lambda c: only_fails(c, ("control", None))),
    ("a trial that didn't finish and none overran: not finished, and the check fails",
     lambda: (good_trials(), unsliced(400.0, 400.0) + [{"balls": 4, "delay": 0.3, "unsliced": True, "failed": "x"}]),
     lambda c: only_fails(c, ("control", None)) and "only 2 of 3" in detail(c, ("control", None))),
    ("a trial that didn't finish beside one that overran: fired",
     lambda: (good_trials(), unsliced(1200.0, 400.0) + [{"balls": 4, "delay": 0.3, "unsliced": True, "failed": "x"}]),
     lambda c: only_fails(c)),
    ("a failing sliced check isn't marked unproven, it just fails",
     lambda: (changed(good_trials(), 4, 0.3, hubStopToIdleMs=950.0), unsliced(400.0, 400.0, 400.0)),
     lambda c: only_fails(c, ("idle", 4), ("control", None)) and "unproven" not in detail(c, ("idle", 4))),
    ("without the control there is no control check", lambda: (good_trials(), []), lambda c: only_fails(c)
     and ("control", None) not in {key(n) for n, *_ in c}),
]


def verdict_cases():
    for label, ok in run_cases(rb.planner_stop_checks, SLICED_CASES + CONTROL_CASES + CAMERA_CASES):
        expect(f"verdict: {label}", ok)
    expect("webcam trials: the plan's count, 9 replanning, 6 INIT and 5 worst-case",
           rb.planner_camera_expected() == 20 == len(camera_trials()))
    checks = rb.planner_stop_checks(good_trials(), [], camera_changed(camera_trials(), "worst", 2,
                                                                       hubStopToOnEndMs=845.0))
    means = {key(n): m for n, ok, d, m in checks}
    expect("webcam window check: the margin to the 900 ms window is printed",
           "slowest 845 ms, 55 ms inside the 900 ms window" in detail(checks, ("idle", "webcam")))
    expect("webcam event-loop check: EasyOpenCV's close after the OpMode is printed per trial, not judged",
           "(onEnd() to '***STOP - OPMODE***', not judged) [660.0, " in detail(checks, ("back", "webcam")))
    expect("webcam FAIL means: a slow stop() puts the close back inside the window; a hung close leaves no INIT",
           "back inside the SDK's 900 ms stop window" in means[("stop", "webcam")]
           and "If CellTipCamera.stop() is slow too" in means[("idle", "webcam")]
           and "Restart Robot" in means[("back", "webcam")])
    text = "\n".join(rb.planner_stop_summary(good_trials(), [], camera_trials()))
    expect("summary: a webcam table row per trial with stop(), STOP to onEnd, the margin and EasyOpenCV's close",
           text.count("| INIT + ") == 6 and text.count("| START at INIT + 0.2 s, STOP 0.05 s later, #") == 5
           and "| 30 | in plan 4, 80 ms into it, 10 ms into a slice | 12 | 5 | 1 | 20 | 40 | 860 | 660 | 700 | False "
               "(4000 to 4000) | 0 |" in text)
    expect("summary: each worst-case trial's timing from INIT, and the worst case reproduced",
           "INIT seen 0.15 s after it was sent, START sent at 0.2 s, STOP at 0.25 s; webcam frames at START 0" in text
           and "reproduced in 5 of 5 worst-case trials" in text)
    text = "\n".join(rb.planner_stop_summary(good_trials(), [], every(camera_trials(), "worst", framesAtStop=3)))
    expect("summary: not reproduced is said so", "not reproduced, so the STOP checks with the webcam don't cover it"
           in text)
    names = [c[0] for c in rb.planner_stop_checks(good_trials())]
    expect("checks: per ball count a restart and an idle check, and a slice check for 4 balls",
           len(names) == 2 * len(rb.PLANNER_STOP_BALLS) + 1 and len(set(names)) == len(names))
    text = "\n".join(rb.planner_stop_summary(good_trials()))
    expect("summary: one table row per trial, saying where STOP landed",
           text.count("| 4 | ") == 3 and text.count("| 5 | ") == 3 and "in plan 4, 80 ms into it, 10 ms into a slice"
           in text)
    text = "\n".join(rb.planner_stop_summary(failed_trial(good_trials(), 4, 0.3)))
    expect("summary: a trial that didn't finish is a FAILED row", "| 4 | 0.3 s | FAILED: Bench: Planner Stop to INIT "
                                                                  "timed out |" in text)
    restarted = unsliced(400.0, None, 400.0)
    restarted[1].update(restarted=True, pidAfter=4001, hubStopToRunningS=44.9)
    text = "\n".join(rb.planner_stop_summary(good_trials(), restarted))
    expect("summary: the control's table gives each trial's longest step, hub STOP to idle and pid before and after",
           "| 0.05 s | 3 | 1510 | in plan 4" in text and "| True (4000 to 4001) | 44.9 |" in text
           and "| 400 | False (4000 to 4000) |" in text)
    expect("summary: the control's verdict, fired", "- The control fired: 1 of 3 STOPs overran" in text)
    text = "\n".join(rb.planner_stop_summary(good_trials(), unsliced(400.0, 400.0, 400.0)))
    expect("summary: the control's verdict, not fired, says the sliced STOP checks are unproven",
           "- The control didn't fire: none of 3 STOPs overran" in text and "unproven on this layout" in text)
    text = "\n".join(rb.planner_stop_summary(good_trials(), (), (), ["Run it with --planner-stop."]))
    expect("summary: without the control, the note given, and no control or webcam table",
           "_Run it with --planner-stop._" in text and "Unsliced control" not in text and "webcam open" not in text)


SLICED_MUTATIONS = [
    ("restarts aren't looked at", 'restarted = [planner_trial(t) for t in ran if t["restarted"] or t["markers"]]',
     'restarted = []'),
    ("the stop window is raised out of reach", "\n" + " " * 53 + "and t[\"hubStopToIdleMs\"] < SDK_STOP_WINDOW_MS)]",
     "\n" + " " * 53 + "and t[\"hubStopToIdleMs\"] < 1e9)]"),
    ("a missing idle line passes", "slow = [planner_trial(t) for t in ran if not (t.get(\"hubStopToIdleMs\") is not None",
     "slow = [planner_trial(t) for t in ran if not (t.get(\"hubStopToIdleMs\") is None or True"),
    ("the slice limit is raised out of reach", "max(slices) < PLANNER_SLICE_MAX_MS", "max(slices) < 1e9"),
    ("a trial with no slice isn't missed", "len(slices) == len(ran) and bool(ran) and max(slices)",
     "bool(ran) and max(slices)"),
    ("unfinished trials aren't counted", "ok = len(ran) == expected and not restarted\n", "ok = not restarted\n"),
    ("a run's rounds aren't counted", "* max([t.get(\"repeat\") or 1 for t in trials] or [1])", "* 1"),
]


def mutated(old, new):
    """run_bench.py with old replaced by new, run in memory as if from its own path; None unless old is in it once."""
    text = RUNNER.read_text()
    if text.count(old) != 1:
        return None
    module = type(sys)("run_bench_mutant")
    module.__file__ = str(RUNNER)
    exec(compile(text.replace(old, new), str(RUNNER), "exec"), module.__dict__)
    return module


CONTROL_MUTATIONS = [
    ("an overrun needs a restart", "return bool(t.get(\"restarted\") or t.get(\"markers\") or (idle is not None and "
     "idle >= SDK_STOP_WINDOW_MS))", "return bool(t.get(\"restarted\") or t.get(\"markers\"))"),
    ("a restart isn't an overrun", "return bool(t.get(\"restarted\") or t.get(\"markers\") or (idle",
     "return bool(t.get(\"markers\") or (idle"),
    ("the control check always passes", "f\"{SDK_STOP_WINDOW_MS} ms stop window at least once\", control[0] == \"fired\"",
     "f\"{SDK_STOP_WINDOW_MS} ms stop window at least once\", True"),
    ("the sliced checks never say unproven", "f\"({control[1]})\" if control is not None and control[0] != \"fired\" else \"\")",
     "f\"({control[1]})\" if False else \"\")"),
    ("an unfinished control counts as not fired", "    if len(ran) < len(PLANNER_UNSLICED_DELAYS_S):\n",
     "    if False:\n"),
]


CAMERA_MUTATIONS = [
    ("the webcam stop window is raised out of reach",
     "for t in ran\n            if not (t.get(\"hubStopToOnEndMs\") is not None and t[\"hubStopToOnEndMs\"] < "
     "SDK_STOP_WINDOW_MS)]",
     "for t in ran\n            if not (t.get(\"hubStopToOnEndMs\") is not None and t[\"hubStopToOnEndMs\"] < 1e9)]"),
    ("the webcam window is judged on STOP to idle again",
     "for t in ran\n            if not (t.get(\"hubStopToOnEndMs\") is not None and t[\"hubStopToOnEndMs\"] <",
     "for t in ran\n            if not (t.get(\"hubStopToIdleMs\") is not None and t[\"hubStopToIdleMs\"] <"),
    ("webcam restarts aren't looked at",
     "restarted = [planner_camera_label(t) for t in ran if t.get(\"restarted\") or t.get(\"markers\")]",
     "restarted = []"),
    ("unfinished webcam trials aren't counted", "complete = len(ran) == expected", "complete = True"),
    ("the stop() limit is raised out of reach", "t[\"cellTipStopMs\"] < PLANNER_CAMERA_STOP_MS)]",
     "t[\"cellTipStopMs\"] < 1e9)]"),
    ("a missing stop() time passes", "if not (t.get(\"cellTipStopMs\") is not None and t[\"cellTipStopMs\"] < "
     "PLANNER_CAMERA_STOP_MS)]", "if not (t.get(\"cellTipStopMs\") is None or t[\"cellTipStopMs\"] < "
     "PLANNER_CAMERA_STOP_MS)]"),
    ("a missing '***STOP - OPMODE***' passes",
     "no_idle = [planner_camera_label(t) for t in ran if t.get(\"hubStopToIdleMs\") is None]", "no_idle = []"),
    ("the worst case needs no planner slice", "and waited is not None\n            and waited > 0)", ")"),
    ("the worst case needs no STOP before the first frame", "t.get(\"framesAtStop\") == 0 and waited", "waited"),
    ("any trial stands in for the worst case",
     "    worst = [t for t in camera if t.get(\"kind\") == \"worst\"]\n    reproduced", "    worst = list(camera)\n    reproduced"),
    ("an unfinished worst-case trial counts", "return (\"failed\" not in t and t.get(\"framesAtStop\")",
     "return (t.get(\"framesAtStop\")"),
    ("the worst-case check always passes", "\"the webcam was still opening)\", bool(reproduced),",
     "\"the webcam was still opening)\", True,"),
]


def mutation_cases(mutations, cases):
    for label, old, new in mutations:
        mutant = mutated(old, new)
        if mutant is None:
            expect(f"weakened: {label}: the text to weaken is in run_bench.py once", False)
            continue
        caught = [case for case, ok in run_cases(mutant.planner_stop_checks, cases) if not ok]
        expect(f"weakened: {label}: a verdict case fails ({caught[:1]})", bool(caught))


class PlannerHub(fake_hub.Hub):
    """fake_hub's Control Hub running a fake Bench: Planner Stop: it reports plans while INITed (from START with
    planFrom=start), and on STOP it writes its result file and logs 'auto-stopping', the bench's onEnd line
    on_end_ms(run_id) later and '***STOP - OPMODE***' idle_ms(run_id) after STOP, on the hub's clock; restarts(run_id)
    makes that STOP restart the app instead. webcam=True: Bench: Probe finds a webcam, a camera=1 trial's
    CellTipCamera.stop() takes close_ms(run_id), and a worst-case trial (from START, no frames to wait for) STOPs
    before the first frame."""

    def __init__(self, idle_ms=lambda run_id: 300.0, restarts=lambda run_id: False, close_ms=lambda run_id: 1.0,
                 webcam=False, no_frames=False, on_end_ms=lambda run_id: 40.0, **kw):
        super().__init__(**kw)
        self.idle_ms, self.restarts, self.close_ms, self.no_frames = idle_ms, restarts, close_ms, no_frames
        self.on_end_ms = on_end_ms
        if webcam:
            self.probe["webcams"] = [{"attached": True, "serial": "UC762"}]
        self.runs, self.messages, self.init_configs = [], [], []
        self.planning = None

    def params(self):
        return json.loads(self.bench.get("files/bench/params.json") or "{}")

    def log_ms(self, text, ms, tag):
        t = datetime.datetime.fromtimestamp(self.clock) + datetime.timedelta(milliseconds=ms)
        self.logcat.append((self.clock, f"{t:%Y-%m-%d %H:%M:%S}.{t.microsecond // 1000:03d}  {self.pid}  {self.pid} I "
                                        f"{tag}: {text}"))

    def _report(self, stop_event, plans, params):
        n = frames = 0
        while not stop_event.wait(0.02):
            frames += 1 if params.get("camera") and not self.no_frames else 0
            camera = {rb.KEY_FRAMES: str(frames)} if params.get("camera") else {}
            if params.get("planFrom") == "start" and self.status["activeOpModeStatus"] != "RUNNING":
                if camera:
                    self.dash.broadcast({"type": "RECEIVE_TELEMETRY", "telemetry": [
                        {"data": camera, "timestamp": self.clock, "fieldOverlay": {"ops": []}}]})
                continue
            if frames < params.get("framesBeforePlanning", 0):
                continue
            n += 1
            plans[0] = n
            data = {rb.KEY_PLANS: str(n), rb.KEY_PHASE: "planning", rb.KEY_STATUS: f"{n} plans", **camera}
            if n >= 3:
                data[rb.KEY_DONE] = "warm; the runner presses STOP mid-plan"
            self.dash.broadcast({"type": "RECEIVE_TELEMETRY", "telemetry": [
                {"data": data, "timestamp": self.clock, "fieldOverlay": {"ops": []}}]})

    def result(self, params, plans):
        run_id = params["runId"]
        info = {"layout": f"{params.get('balls')} balls on both sides of the rail", "balls": params.get("balls"),
                "sliceMs": None if params.get("unsliced") else 20.0, "unsliced": bool(params.get("unsliced")),
                "firstPlanInProcess": len(self.runs) == 1, "warmPlans": 3}
        one_step = bool(params.get("unsliced"))
        out = {"bench": "Planner Stop", "info": info, "planMs": [1500.0 if one_step else 1900.0] * plans,
               "planComputeMs": [1500.0] * plans, "planLoops": [1 if one_step else 70] * plans,
               "stepMs": summary(mx=1500.0 if one_step else 120.0), "overrunMs": summary(p50=1.0),
               "loopPeriodMs": summary(p50=26.0, mx=1510.0 if one_step else 130.0),
               "stop": {"plans": plans, "stopSeen": True, "planRunningAtStop": plans + 1, "stopToStopPassMs": 40.0,
                        "msIntoPlanAtStop": 60.0, "inSliceAtStop": True, "msIntoSliceAtStop": 8.0,
                        "sliceLeftAtStopMs": 12.0, "sliceAfterStopMs": 12.0, "tailMs": 3.0},
               "timeline": {"initializeMs": 40.0, "firstPlanMs": 60.0}, "runId": run_id}
        if params.get("camera"):
            close = self.close_ms(run_id)
            start = params.get("planFrom") == "start"
            frames = params.get("framesBeforePlanning", 0)
            out["camera"] = {"framesAtStart": frames if start else None,
                             "framesAtStop": (frames + 31 if frames else 0) if start else 31,
                             "framesAtFirstPlan": frames, "cellTipStopMs": close}
            out["stop"].update(stopPassToOnEndMs=close + 6.0)
            if start:
                out["timeline"].update(startMs=230.0, firstPlanMs=234.0)
        return out

    def on_message(self, session, msg):
        kind = msg.get("type")
        with self.lock:
            if kind in ("INIT_OP_MODE", "START_OP_MODE", "STOP_OP_MODE"):
                self.messages.append((kind, msg.get("opModeName") or self.status["activeOpMode"], self.params()))
            if kind == "INIT_OP_MODE" and msg["opModeName"] == OPMODE:
                self.dash.broadcast({"type": "RECEIVE_TELEMETRY", "telemetry": []})
                self.set_active(OPMODE, "INIT")
                params = self.params()
                self.runs.append(params)
                stop_event, plans = threading.Event(), [0]
                self.planning = (stop_event, plans, params)
                self.init_configs.append(self.active["name"])
                threading.Thread(target=self._report, args=(stop_event, plans, params), daemon=True).start()
                return
            if kind == "STOP_OP_MODE" and self.status["activeOpMode"] == OPMODE and self.planning is not None:
                stop_event, plans, params = self.planning
                self.planning = None
                stop_event.set()
                run_id = params["runId"]
                self.bench[f"files/bench/planner_stop_{run_id}.json"] = json.dumps(self.result(params, plans[0]))
                self.log_ms(f"auto-stopping OpMode '{OPMODE}'", 0, "FtcEventLoop")
                self.log_ms(f"planner_stop {run_id} STOP seen 60.0 ms into plan {plans[0] + 1}, 8.0 ms into a slice",
                            1, "BENCH")
                if self.restarts(run_id):
                    self.log_ms("OpMode was stuck in stop(); restarting", 1006, "OpModeManager")
                    self.restart_app()
                    return
                self.log_ms(f"planner_stop {run_id} onEnd", self.on_end_ms(run_id), "BENCH")
                self.log_ms("******************** STOP - OPMODE /sdcard/FIRST/matchlogs/x.txt ********************",
                            self.idle_ms(run_id), "RobotCore")
        super().on_message(session, msg)


class Interrupt(Exception):
    pass


def dry_run(argv, hub, runner_class=None):
    """Runner.run() on the hub; (exit code or the exception it raised, results folder)."""
    OUT.mkdir(parents=True, exist_ok=True)
    out = pathlib.Path(tempfile.mkdtemp(prefix="run-", dir=OUT)) / "run"
    handlers = signal.getsignal(signal.SIGINT), signal.getsignal(signal.SIGHUP)
    stdin, sys.stdin = sys.stdin, io.StringIO("")
    try:
        with hub.installed(rb), contextlib.redirect_stdout(io.StringIO()):
            try:
                code = (runner_class or rb.Runner)(rb.parse_args(["--out", str(out), "--ports-empty"] + list(argv))).run()
            except (KeyboardInterrupt, Interrupt) as e:
                code = e
    finally:
        sys.stdin = stdin
        signal.signal(signal.SIGINT, handlers[0])
        signal.signal(signal.SIGHUP, handlers[1])
    return code, out


def check_line(text, part):
    return next((l for l in text.splitlines() if l.startswith("- ") and part in l), "")


def put_back(hub):
    return hub.active["name"] == "competition" and not [n for n in hub.configs if n.startswith("bench_")]


def sliced_on_the_fake_hub():
    hub = PlannerHub()
    code, out = dry_run(["--planner-stop"], hub)
    text = (out / "summary.md").read_text()
    results = json.loads((out / "runner_results.json").read_text())
    inits = [p for k, o, p in hub.messages if k == "INIT_OP_MODE" and o == OPMODE]
    sliced = [{"balls": b, "runId": f"b{b}_d{int(d * 1000)}"} for b in rb.PLANNER_STOP_BALLS
              for d in rb.PLANNER_STOP_DELAYS_S]
    sliced += [{"balls": b, "runId": f"b{b}_d{int(d * 1000)}_{r}"} for r in range(2, rb.PLANNER_STOP_REPEATS + 1)
               for b in rb.PLANNER_STOP_BALLS for d in rb.PLANNER_STOP_DELAYS_S]
    expect("--planner-stop: the 18 sliced trials (each ball count and delay, three rounds) ran first, in order, with "
           "only balls and runId as params", inits[:NAMED_SLICED] == sliced and len(sliced) == 18)
    expect("--planner-stop: Bench: Planner Stop was never STARTed",
           not [m for m in hub.messages if m[0] == "START_OP_MODE" and m[1] == OPMODE])
    expect("--planner-stop: every sliced check passes on 300 ms STOPs",
           all(check_line(text, f"planner stop {b} balls").startswith("- PASS") for b in rb.PLANNER_STOP_BALLS)
           and "9 of 9 trials ran" in text and "hub STOP to idle [" + ", ".join(["300.0"] * 9) + "] ms" in text)
    rec = (results.get("planner_stop_b4_d50") or {}).get("runner") or {}
    expect("record: the hub's STOP to idle, where STOP landed and the pid before and after, per trial",
           rec.get("hubStopToIdleMs") == 300.0 and rec.get("planAtStop") and rec.get("sliceLeftMs") == 12.0
           and rec.get("pidBefore") == rec.get("pidAfter") is not None)
    expect("--planner-stop: the run ends with the config put back", put_back(hub))
    return hub, code, out, text


def slow_stop_on_the_fake_hub():
    hub = PlannerHub(idle_ms=lambda run_id: 950.0 if run_id == "b4_d300" else 300.0)
    code, out = dry_run(["--planner-stop"], hub)
    text = (out / "summary.md").read_text()
    expect("a 950 ms STOP: the 4-ball idle check fails, naming the trial, and the run exits 1",
           check_line(text, "planner stop 4 balls: the hub logs every STOP").startswith("- FAIL")
           and "missing at ['0.3 s']" in text and code == 1 and put_back(hub))


def interrupted_on_the_fake_hub():
    class CtrlC(rb.Runner):
        def _planner_stop(self, balls, delay, *args, **kw):
            if (balls, delay) == (4, rb.PLANNER_STOP_DELAYS_S[1]):
                raise KeyboardInterrupt
            return super()._planner_stop(balls, delay, *args, **kw)

    hub = PlannerHub()
    code, out = dry_run(["--planner-stop"], hub, CtrlC)
    expect("Ctrl-C between trials: the runner re-raises it after putting the config back",
           isinstance(code, KeyboardInterrupt) and put_back(hub))


def control_on_the_fake_hub():
    restart = "unsliced_b4_d150"
    hub = PlannerHub(idle_ms=lambda run_id: 1200.0 if run_id.startswith("unsliced") else 300.0,
                     restarts=lambda run_id: run_id == restart)
    code, out = dry_run(["--planner-stop"], hub)
    text = (out / "summary.md").read_text()
    results = json.loads((out / "runner_results.json").read_text())
    inits = [p for k, o, p in hub.messages if k == "INIT_OP_MODE" and o == OPMODE]
    expect("--planner-stop: after the 18 sliced trials, three unsliced ones on 4 balls with unsliced=1",
           inits[NAMED_SLICED:] == [{"balls": 4, "runId": f"unsliced_b4_d{int(d * 1000)}", "unsliced": 1}
                         for d in rb.PLANNER_UNSLICED_DELAYS_S])
    expect("--planner-stop: the control fired on 1200 ms STOPs and a restart, and its check passes",
           check_line(text, "planner stop unsliced control").startswith("- PASS")
           and "3 of 3 STOPs overran" in text and "- The control fired" in text)
    rec = (results.get(f"planner_stop_{restart}") or {}).get("runner") or {}
    expect("record: the restarted unsliced trial has the pid change and no idle line",
           rec.get("restarted") is True and int(rec.get("pidAfter")) == int(rec.get("pidBefore")) + 1
           and rec.get("hubStopToIdleMs") is None and rec.get("unsliced") is True)
    expect("--planner-stop with the control: the run ends with the config put back", put_back(hub))

    hub = PlannerHub()
    code, out = dry_run(["--planner-stop"], hub)
    text = (out / "summary.md").read_text()
    expect("--planner-stop with every STOP at 300 ms: the control check fails and the 4-ball STOP checks say unproven",
           check_line(text, "planner stop unsliced control").startswith("- FAIL")
           and "unproven on this layout" in check_line(text, "planner stop 4 balls: the hub logs every STOP")
           and check_line(text, "planner stop 4 balls: the hub logs every STOP").startswith("- PASS") and code == 1)

    class CtrlC(rb.Runner):
        def _planner_stop(self, balls, delay, *args, **kw):
            if kw.get("unsliced"):
                raise KeyboardInterrupt
            return super()._planner_stop(balls, delay, *args, **kw)

    hub = PlannerHub()
    code, out = dry_run(["--planner-stop"], hub, CtrlC)
    expect("Ctrl-C at the first unsliced trial: the config is put back", isinstance(code, KeyboardInterrupt)
           and put_back(hub))


def camera_params():
    replan = [{"balls": 4, "runId": f"camera_plan_d{int(d * 1000)}_{r}", "camera": 1, "framesBeforePlanning": 10,
               "planFrom": "start"} for r in range(1, 4) for d in rb.PLANNER_STOP_DELAYS_S]
    init = [{"balls": 4, "runId": f"camera_init_d{int(d * 1000)}_{r}", "camera": 1} for r in range(1, 4)
            for d in rb.PLANNER_CAMERA_INIT_DELAYS_S]
    return replan + init + [{"balls": 4, "runId": f"camera_worst_{r}", "camera": 1, "planFrom": "start"}
                            for r in range(1, 6)]


def camera_on_the_fake_hub():
    hub = PlannerHub(webcam=True, idle_ms=lambda run_id: 1200.0 if run_id.startswith("unsliced") else 700.0)
    code, out = dry_run(["--planner-stop"], hub)
    text = (out / "summary.md").read_text()
    results = json.loads((out / "runner_results.json").read_text())
    own = [(k, p.get("runId")) for k, o, p in hub.messages if o == OPMODE]
    inits = [p for k, o, p in hub.messages if k == "INIT_OP_MODE" and o == OPMODE]
    expect("with a webcam: after the sliced and unsliced trials, 9 replanning, 6 INIT and 5 worst-case webcam trials, "
           "in that order, with the plan's params", inits[NAMED_BEFORE_CAMERA:] == camera_params())
    starts = [run_id for k, run_id in own if k == "START_OP_MODE"]
    order_ok = all(own.index(("INIT_OP_MODE", r)) < own.index(("START_OP_MODE", r)) < own.index(("STOP_OP_MODE", r))
                   for r in starts)
    replans = [f"camera_plan_d{int(d * 1000)}_{r}" for r in range(1, 4) for d in rb.PLANNER_STOP_DELAYS_S]
    expect("with a webcam: the replanning trials (they plan in RUN, as Ball Collection Auto does) and the worst cases "
           "are STARTed, each between its INIT and its STOP; the INIT trials never are",
           starts == replans + [f"camera_worst_{r}" for r in range(1, 6)] and order_ok)
    replan = (results.get("planner_stop_camera_plan_d300_1") or {}).get("runner") or {}
    expect("record, replanning: 10 frames in INIT before START, then 3 warm plans in RUN, STOP 0.3 s after the next",
           replan.get("framesSeconds") is not None and replan.get("warmSeconds") is not None
           and replan.get("nextPlanSeconds") is not None and replan.get("framesAtStart") == 10
           and "failed" not in replan)
    expect("with a webcam: every webcam trial ran on bench_main, which names the probe's webcam aprilTagDetector",
           set(hub.init_configs[NAMED_BEFORE_CAMERA:]) == {rb.CONFIG_MAIN}
           and 'Webcam name="aprilTagDetector" serialNumber="UC762"' in (out / "logs" / "bench_main.xml").read_text())
    expect("with a webcam: its five checks pass, the worst case reproduced by STOPs before the first frame",
           all(check_line(text, f"planner stop with the webcam: {w}").startswith("- PASS")
               for w in ("STOP never restarts", "the hub logs every STOP", "CellTipCamera.stop() under",
                         "every STOP logs '***STOP - OPMODE***'", "the worst case was reproduced"))
           and "reproduced in 5 of 5 worst-case trials" in text)
    worst = (results.get("planner_stop_camera_worst_1") or {}).get("runner") or {}
    expect("record, worst case: START at INIT + 0.2 s or later, STOP 0.05 s after it, the close, frames at START, "
           "the slice waited for and the pid before and after",
           worst.get("startSentS", 0) >= 0.2 and worst.get("stopSentS", 0) - worst.get("startSentS", 9) >= 0.05
           and worst.get("cellTipStopMs") == 1.0 and worst.get("framesAtStart") == 0 and worst.get("framesAtStop") == 0
           and worst.get("hubStopToOnEndMs") == 40.0
           and worst.get("sliceAfterStopMs") == 12.0 and worst.get("startToFirstSliceMs") == 4.0
           and worst.get("pidBefore") == worst.get("pidAfter") is not None and worst.get("hubStopToIdleMs") == 700.0)
    init = (results.get("planner_stop_camera_init_d300_1") or {}).get("runner") or {}
    expect("record, INIT trial: STOP 0.3 s after INIT was seen, the hub's STOP to onEnd and to idle, the loop's tail",
           init.get("stopSentS", 0) - init.get("initSeenS", 9) >= 0.3 and init.get("tailMs") == 3.0
           and init.get("hubStopToIdleMs") == 700.0 and init.get("hubStopToOnEndMs") == 40.0
           and "startSentS" not in init)
    expect("with a webcam: the run ends with the config put back", put_back(hub))

    hub = PlannerHub(webcam=True, no_frames=True, idle_ms=lambda run_id: 1200.0 if run_id.startswith("unsliced") else
                     700.0)
    code, out = dry_run(["--planner-stop"], hub)
    results = json.loads((out / "runner_results.json").read_text())
    starts = [p.get("runId") for k, o, p in hub.messages if k == "START_OP_MODE" and o == OPMODE]
    expect("a webcam that gives no frames: no replanning trial is STARTed, each fails saying so, the run goes on",
           not [r for r in starts if r.startswith("camera_plan")] and len(starts) == 5
           and "didn't give 10 fresh frames in INIT" in results["planner_stop_camera_plan_d50_1"].get("failed", "")
           and check_line((out / "summary.md").read_text(), "STOP never restarts").startswith("- FAIL")
           and put_back(hub))

    hub = PlannerHub(webcam=True, idle_ms=lambda run_id: 1200.0 if run_id.startswith("unsliced") else 300.0)
    code, out = dry_run(["--absent", "--planner-stop"], hub)
    expect("after Bench: Webcam Stop (absent) left bench_nowebcam active, the webcam trials switch back to bench_main",
           hub.init_configs[:NAMED_BEFORE_CAMERA] == [rb.CONFIG_NO_WEBCAM] * NAMED_BEFORE_CAMERA
           and set(hub.init_configs[NAMED_BEFORE_CAMERA:]) == {rb.CONFIG_MAIN}
           and put_back(hub))

    hub = PlannerHub(webcam=True, close_ms=lambda run_id: 300.0,
                     on_end_ms=lambda run_id: 905.0 if run_id == "camera_init_d300_2" else 40.0,
                     idle_ms=lambda run_id: 1600.0 if run_id == "camera_init_d300_2" else
                     1200.0 if run_id.startswith("unsliced") else 700.0)
    code, out = dry_run(["--planner-stop"], hub)
    text = (out / "summary.md").read_text()
    window = check_line(text, "planner stop with the webcam: the hub logs every STOP")
    expect("onEnd() 905 ms after a webcam STOP: the window check fails naming the trial, and the run exits 1",
           window.startswith("- FAIL") and "INIT + 0.3 s, #2" in window and "If CellTipCamera.stop() is slow too"
           in window and code == 1)
    expect("300 ms stop() calls: the stop() check fails, saying the close is back inside the window",
           check_line(text, "CellTipCamera.stop() under 100 ms").startswith("- FAIL")
           and "back inside the SDK's 900 ms stop window" in check_line(text, "CellTipCamera.stop() under 100 ms")
           and check_line(text, "event loop came back").startswith("- PASS") and put_back(hub))

    hub = PlannerHub(idle_ms=lambda run_id: 1200.0 if run_id.startswith("unsliced") else 300.0)
    code, out = dry_run(["--planner-stop"], hub)
    text = (out / "summary.md").read_text()
    expect("no webcam plugged in: the webcam trials are skipped with a line saying what to plug in, and not judged",
           "_The webcam trials were skipped: no webcam plugged in. Plug a UVC webcam into a Control Hub USB port to run "
           "them._" in text and not [p for k, o, p in hub.messages if "camera" in p]
           and "planner stop with the webcam" not in text and code == 0 and put_back(hub))

    class CtrlC(rb.Runner):
        def _planner_stop_timed(self, run_id, *args, **kw):
            if run_id == "camera_worst_2":
                raise KeyboardInterrupt
            return super()._planner_stop_timed(run_id, *args, **kw)

    hub = PlannerHub(webcam=True)
    code, out = dry_run(["--planner-stop"], hub, CtrlC)
    expect("Ctrl-C during the webcam trials: the config is put back", isinstance(code, KeyboardInterrupt)
           and put_back(hub) and ("INIT_OP_MODE", "camera_worst_1") in [(k, p.get("runId")) for k, o, p in hub.messages])


def default_on_the_fake_hub():
    """Bench: Planner Stop as a run without bench flags runs it (dryrun_default.py runs the whole default suite)."""
    hub = PlannerHub(webcam=True)
    OUT.mkdir(parents=True, exist_ok=True)
    out = pathlib.Path(tempfile.mkdtemp(prefix="default-", dir=OUT)) / "run"
    with hub.installed(rb), contextlib.redirect_stdout(io.StringIO()):
        r = rb.Runner(rb.parse_args(["--out", str(out), "--ports-empty"]))
        r.connect()
        r.run_probe()
        r.activate_config(rb.CONFIG_MAIN, r.main_config())
        r.run_planner_stop()
        r.dash.close()
        r.link.close()
    inits = [p for k, o, p in hub.messages if k == "INIT_OP_MODE" and o == OPMODE]
    text = "\n".join(r.summary)
    expect("no bench flags, a webcam plugged in: only the six sliced trials, once each, and a line saying how to run "
           "the rest", [p["runId"] for p in inits] == [f"b{b}_d{int(d * 1000)}" for b in rb.PLANNER_STOP_BALLS
                                                       for d in rb.PLANNER_STOP_DELAYS_S]
           and "_Each sliced trial ran once. The other 2 rounds, the unsliced control and, with a webcam plugged in, "
               "the webcam trials run only when --planner-stop is given._" in text
           and all("3 of 3 trials ran" in c[2] for c in r.checks if "never restarts" in c[0]) and not [c for c in r.checks if "unsliced control" in c[0] or "webcam" in c[0]])


def main():
    for section in (verdict_cases, lambda: mutation_cases(SLICED_MUTATIONS + CONTROL_MUTATIONS + CAMERA_MUTATIONS,
                                                          SLICED_CASES + CONTROL_CASES + CAMERA_CASES),
                    sliced_on_the_fake_hub, slow_stop_on_the_fake_hub, interrupted_on_the_fake_hub,
                    control_on_the_fake_hub, camera_on_the_fake_hub, default_on_the_fake_hub):
        try:
            section()
        except Exception as e:
            expect(f"{getattr(section, '__name__', 'section')} ran to its end ({type(e).__name__}: {e})", False)
            traceback.print_exc()
    print(f"{passed} passed, {len(failed)} failed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
