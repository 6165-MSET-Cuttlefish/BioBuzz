"""Bench: Pinpoint's soak, recalibration run and raw-driver unplug offline: their verdicts on synthetic results (each
check fails when weakened), and run_pinpoint end to end on fake_hub.py: the prompts in order, the soak only with its
flag (300 s with --quick), the recalibration as its own OpMode run, the skip with no Pinpoint, and Ctrl-C at a prompt
putting the config back. No hub, no adb, no network. Run with: python3 -I scripts/bench/offline/benches/pinpoint.py
"""
import ast
import contextlib
import copy
import importlib.util
import io
import json
import pathlib
import signal
import sys
import tempfile
import traceback

sys.dont_write_bytecode = True
HERE = pathlib.Path(__file__).resolve().parent
OFFLINE = HERE.parent
WORKTREE = OFFLINE.parents[2]
OUT = OFFLINE / "out" / "pinpoint"
RUNNER = WORKTREE / "scripts/bench/run_bench.py"

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
SOURCE = RUNNER.read_text()
TREE = ast.parse(SOURCE)


def tail(p50, p95, p99, top, count=26000, over20=0, over40=0):
    return {"count": count, "mean": p50 + 0.2, "p50": p50, "p95": p95, "p99": p99, "max": top, "over20": over20,
            "over40": over40, "binMs": 0.05}


def drift(step, window, slope):
    return {"samples": 2600, "seconds": 300.0, "change0to5Deg": step, "change5to120Deg": window,
            "slopeDegPerMin": slope, "maxAbsChangeDeg": abs(step) + abs(window), "finalChangeDeg": step + window}


def good_soak(asked=300):
    reads = 26170
    return {"present": True, "octoQuadPresent": True,
            "info": {"yawScalar": 1.0, "gcStatsReadable": True, "seconds": asked},
            "soak": {"ran": True, "ended": True, "seconds": asked + 0.02, "askedSeconds": asked, "runLoops": 26000,
                     "readMs": tail(5.8, 6.7, 9.3, 18.0), "followerUpdateMs": tail(6.4, 7.4, 10.1, 19.0),
                     "loopPeriodMs": tail(11.5, 13.0, 16.0, 24.0, over20=4),
                     "slowReads": {"reads": 26000, "over20": 0, "over20WithBlockingGc": 0, "over20Rate": 0.0,
                                   "over40": 0, "over40WithBlockingGc": 0, "over40WithoutGc": 0, "over40Rate": 0.0,
                                   "rows": []},
                     "status": {"statusCounts": {"FAULT_BAD_READ": 1, "READY": reads}, "readsAfterFirstReady": reads,
                                "notReadyAfterFirstReady": 0, "streakCount": 0, "longestStreakReads": 0,
                                "longestStreakMs": 0, "guardWouldStopMs": None, "guardWouldStopPhase": None,
                                "streaks": []},
                     "drift": {"pinpoint": drift(0.05, 0.2, 0.1), "octoQuad": drift(0.0, 0.01, 0.005)}}}


def good_recal():
    return {"recal": {"settledMs": 40.0, "pinpointRecalMs": 260.0, "pinpointLeftReady": True,
                      "octoQuadRecalMs": 1620.0, "octoQuadLeftRunning": True, "backWithinLimit": True,
                      "step": "done", "ran": True, "pinpointDrift": drift(0.02, 0.1, 0.05),
                      "octoQuadDrift": drift(0.0, 0.0, 0.0)}}


OK_OUTCOME = {"crashed": False, "errorMessage": "", "timedOut": False}


def soak_args(res=None, outcome=None, recal=None, recal_outcome=None, asked=300):
    return (res if res is not None else good_soak(asked), outcome or dict(OK_OUTCOME),
            recal if recal is not None else good_recal(), recal_outcome or dict(OK_OUTCOME), asked)


def edit(fn):
    res = good_soak()
    fn(res["soak"])
    return res


def streak(soak):
    soak["status"].update(notReadyAfterFirstReady=1, streakCount=1, longestStreakReads=1, longestStreakMs=11.0,
                          guardWouldStopMs=187000.0, guardWouldStopPhase="running",
                          streaks=[{"startMs": 187000.0, "phase": "running", "status": "FAULT_BAD_READ",
                                    "pose": "Pose(72, 134, 90)", "reads": 1, "readyAgainAfterMs": 11.0}])


def stall(soak, with_gc):
    soak["readMs"].update(max=45.0, over20=1, over40=1)
    soak["slowReads"].update(over20=1, over40=1, over20WithBlockingGc=1 if with_gc else 0,
                             over40WithBlockingGc=1 if with_gc else 0, over40WithoutGc=0 if with_gc else 1,
                             rows=[[9000, 104000.0, 45.0, 1 if with_gc else 0, 1]])


KEYS = (("ran", "pinpoint soak: ran"), ("ready", "pinpoint soak: no status"), ("stall", "pinpoint soak: no Pinpoint read"),
        ("control", "pinpoint soak: setup"), ("drift", "pinpoint soak: the Pinpoint's heading"),
        ("recal", "pinpoint recalibration"))


def verdicts(fn, args):
    out = {}
    for name, ok, detail, means in fn(*args)[0]:
        key = next(k for k, prefix in KEYS if name.startswith(prefix))
        out[key] = (ok, detail)
    return out


def only_fails(v, *keys, judged=("ran", "ready", "stall", "control", "drift", "recal")):
    return set(v) == set(judged) and {k for k, (ok, _) in v.items() if not ok} == set(keys)


CASES = [
    ("a quiet hour passes every check", lambda: soak_args(asked=3600), lambda v: only_fails(v)),
    ("a 45 ms read with no blocking GC across it fails the stall check",
     lambda: soak_args(edit(lambda s: stall(s, False))), lambda v: only_fails(v, "stall")),
    ("the same 45 ms read under a blocking GC passes",
     lambda: soak_args(edit(lambda s: stall(s, True))), lambda v: only_fails(v)),
    ("one FAULT_BAD_READ after the first READY fails, naming its streak and where the guard would stop",
     lambda: soak_args(edit(streak)),
     lambda v: only_fails(v, "ready") and "FAULT_BAD_READ" in v["ready"][1] and "187.0 s" in v["ready"][1]),
    ("0.6 deg of Pinpoint drift over 5-120 s fails",
     lambda: soak_args(edit(lambda s: s["drift"].update(pinpoint=drift(0.05, 0.6, 0.3)))),
     lambda v: only_fails(v, "drift")),
    ("a 0.3 deg step in the first 5 s fails",
     lambda: soak_args(edit(lambda s: s["drift"].update(pinpoint=drift(0.3, 0.1, 0.05)))),
     lambda v: only_fails(v, "drift")),
    ("drift backwards counts the same: -0.6 deg fails",
     lambda: soak_args(edit(lambda s: s["drift"].update(pinpoint=drift(0.0, -0.6, -0.3)))),
     lambda v: only_fails(v, "drift")),
    ("0.45 deg over 5-120 s and 0.15 deg in the first 5 s pass",
     lambda: soak_args(edit(lambda s: s["drift"].update(pinpoint=drift(0.15, 0.45, 0.2)))), lambda v: only_fails(v)),
    ("an OctoQuad control that moved 0.15 deg fails setup, and the Pinpoint's drift isn't judged",
     lambda: soak_args(edit(lambda s: s["drift"].update(octoQuad=drift(0.0, 0.15, 0.1), pinpoint=drift(1, 1, 1)))),
     lambda v: only_fails(v, "control", judged=("ran", "ready", "stall", "control", "recal"))),
    ("no OctoQuad: setup fails saying so, and the drift isn't judged",
     lambda: soak_args(edit(lambda s: s["drift"].update(octoQuad=None))),
     lambda v: only_fails(v, "control", judged=("ran", "ready", "stall", "control", "recal"))
     and "no OctoQuad answered" in v["control"][1]),
    ("a soak that stopped at 200 of 300 s fails the ran check",
     lambda: soak_args(edit(lambda s: s.update(seconds=200.0))), lambda v: only_fails(v, "ran")),
    ("a crashed soak fails the ran check",
     lambda: soak_args(outcome=dict(OK_OUTCOME, crashed=True, errorMessage="boom")), lambda v: only_fails(v, "ran")),
    ("a soak saved mid-run (not ended) fails the ran check",
     lambda: soak_args(edit(lambda s: s.update(ended=False))), lambda v: only_fails(v, "ran")),
    ("no reads after a READY fails the status check rather than passing on nothing",
     lambda: soak_args(edit(lambda s: s["status"].update(readsAfterFirstReady=0))), lambda v: only_fails(v, "ready")),
    ("a recalibration that never came back fails",
     lambda: soak_args(recal={"recal": dict(good_recal()["recal"], backWithinLimit=False)}),
     lambda v: only_fails(v, "recal")),
    ("a recalibration run that stopped before its 60 s fails",
     lambda: soak_args(recal={"recal": dict(good_recal()["recal"], step="hold")}), lambda v: only_fails(v, "recal")),
    ("a crashed recalibration run fails", lambda: soak_args(recal_outcome=dict(OK_OUTCOME, crashed=True)),
     lambda v: only_fails(v, "recal")),
    ("missing data: nothing passes", lambda: soak_args({}, recal={}),
     lambda v: not any(ok for ok, _ in v.values()) and {"ran", "ready", "stall", "control", "recal"} <= set(v)),
]


def run_cases(fn):
    results = []
    for label, make, predicate in CASES:
        try:
            ok = predicate(verdicts(fn, make()))
        except Exception as e:
            ok = False
            label = f"{label} ({type(e).__name__}: {e})"
        results.append((label, ok))
    return results


RAW_CASES = [
    ("a zeroed pose passes: the answer is recorded, not judged",
     lambda: ({}, {"dropout": {"badLoops": 40, "poseDuringDropout": "zeroed", "firstNotReadyStatus": "NOT_READY"}}),
     True),
    ("a frozen pose passes too",
     lambda: ({}, {"dropout": {"badLoops": 3, "poseDuringDropout": "frozen", "firstNotReadyStatus": "FAULT_BAD_READ"}}),
     True),
    ("an unplug that never showed fails", lambda: ({}, {"dropout": {"badLoops": 0}}), False),
    ("no result fails", lambda: ({}, {}), False),
    ("a skipped unplug fails", lambda: ({"skipped": "not READY"}, {"dropout": {"badLoops": 9}}), False),
]


def run_raw_cases(fn):
    results = []
    for label, make, want in RAW_CASES:
        rec, res = make()
        checks, _ = fn(rec, res, dict(OK_OUTCOME))
        results.append((label, len(checks) == 1 and checks[0][1] is want))
    return results


def function_source(name):
    node = next(n for n in TREE.body if isinstance(n, ast.FunctionDef) and n.name == name)
    return ast.get_source_segment(SOURCE, node)


def mutant(name, old, new, **constants):
    src = function_source(name)
    if old and src.count(old) != 1:
        return None
    namespace = dict(vars(rb))
    namespace.update(constants)
    exec(compile(src.replace(old, new) if old else src, f"<{name} weakened>", "exec"), namespace)
    return namespace[name]


MUTATIONS = [
    ("pinpoint_soak_checks", "a non-READY read no longer fails", "after > 0 and bad == 0", "after > 0", {}),
    ("pinpoint_soak_checks", "a stall with no GC no longer fails", "counted(reads) and without == 0", "counted(reads)", {}),
    ("pinpoint_soak_checks", "the 5-120 s drift limit is dropped", "and abs(span) <= PINPOINT_WINDOW_DEG", "", {}),
    ("pinpoint_soak_checks", "the first-5-s limit is dropped", "abs(step) <= PINPOINT_STEP_DEG and ", "", {}),
    ("pinpoint_soak_checks", "the control's limit is out of reach", "", "", {"PINPOINT_CONTROL_DEG": 1e9}),
    ("pinpoint_soak_checks", "the window limit is out of reach", "", "", {"PINPOINT_WINDOW_DEG": 1e9}),
    ("pinpoint_soak_checks", "a short soak passes", "and ran_s >= asked_s", "", {}),
    ("pinpoint_soak_checks", "a crashed soak passes", "not crashed and soak.get(\"ended\") is True",
     "soak.get(\"ended\") is True", {}),
    ("pinpoint_soak_checks", "the recalibration needn't come back", "and rc.get(\"backWithinLimit\") is True", "", {}),
    ("pinpoint_raw_checks", "an unplug that never showed passes", 'READY", bad > 0,', 'READY", True,', {}),
]


def verdict_cases():
    for label, ok in run_cases(rb.pinpoint_soak_checks):
        expect(f"soak verdict: {label}", ok)
    for label, ok in run_raw_cases(rb.pinpoint_raw_checks):
        expect(f"raw dropout verdict: {label}", ok)
    text = "\n".join(rb.pinpoint_soak_checks(*soak_args())[1])
    expect("summary: 26,170 quiet reads (five minutes) bound the rate at only 1.15e-04 per read, over a 4950-read match",
           "1.15e-04 per read" in text and "4950 reads" in text)
    reads = 313000
    res = good_soak(3600)
    res["soak"]["status"]["readsAfterFirstReady"] = reads
    text = "\n".join(rb.pinpoint_soak_checks(*soak_args(res, asked=3600))[1])
    expect("summary: 313,000 quiet reads bound the rate at 9.58e-06 per read, at most 4.6% per match",
           "9.58e-06 per read" in text and "4.6% chance" in text)
    res = edit(streak)
    text = "\n".join(rb.pinpoint_soak_checks(*soak_args(res))[1])
    expect("summary: a streak's start, status, reads and time to READY, and the held guard's stop",
           "[187000.0, 'FAULT_BAD_READ', 1, 11.0]" in text and "have stopped 187.0 s" in text)
    res = edit(lambda s: s["status"].update(notReadyAfterFirstReady=500, streakCount=5))
    text = "\n".join(rb.pinpoint_soak_checks(*soak_args(res))[1])
    expect("summary: non-READY reads give the streak rate seen, never the rule of three's none-in-N bound",
           "rule of three: with none" not in text and "500 non-READY reads in 26170" in text
           and "a streak starts 1.91e-04 per read" in text and "about 61.2% chance" in text)
    text = "\n".join(rb.pinpoint_soak_checks(*soak_args(edit(lambda s: s["drift"].update(octoQuad=None))))[1])
    expect("summary: a missing control says the Pinpoint's drift isn't judged", "isn't judged" in text)


def mutation_cases():
    for name, label, old, new, constants in MUTATIONS:
        fn = mutant(name, old, new, **constants)
        if fn is None:
            expect(f"weakened: {label}: the text to weaken is in {name} once", False)
            continue
        cases = run_raw_cases(fn) if name == "pinpoint_raw_checks" else run_cases(fn)
        caught = [case for case, ok in cases if not ok]
        expect(f"weakened: {label}: a verdict case fails ({caught[:1]})", bool(caught))


def flags():
    expect("--pinpoint-soak alone: 3600 s, and it implies --pinpoint", rb.parse_args(["--pinpoint-soak"]).pinpoint_soak
           == 3600 and rb.parse_args(["--pinpoint-soak"]).pinpoint)
    expect("--pinpoint-soak with --quick: 300 s", rb.parse_args(["--pinpoint-soak", "--quick"]).pinpoint_soak == 300)
    expect("--pinpoint-soak 900: 900 s", rb.parse_args(["--pinpoint-soak", "900"]).pinpoint_soak == 900)
    expect("no flag: no soak", rb.parse_args([]).pinpoint_soak is None and rb.parse_args(["--pinpoint"]).pinpoint_soak
           is None)
    for bad in (["--pinpoint-soak", "100"], ["--pinpoint-soak", "0"], ["--probe-only", "--pinpoint-soak"]):
        try:
            with contextlib.redirect_stderr(io.StringIO()):
                rb.parse_args(bad)
            refused = False
        except SystemExit:
            refused = True
        expect(f"{' '.join(bad)} is refused", refused)


class Terminal:
    def __init__(self, hub, actions):
        self.hub, self.actions, self.reads = hub, list(actions), []

    def isatty(self):
        return True

    def readline(self):
        self.reads.append(len(self.hub.calls))
        if not self.actions:
            return ""
        action = self.actions.pop(0)
        if action is KeyboardInterrupt:
            raise KeyboardInterrupt
        if action is not None:
            action()
        return "\n"


class Station(fake_hub.Hub):
    def __init__(self, present=True, **kw):
        super().__init__(**kw)
        self.present_pinpoint = present
        self.inits = []
        self.pinpoint_status = "READY"

    def telemetry(self, data):
        self.dash.broadcast({"type": "RECEIVE_TELEMETRY", "telemetry": [
            {"data": data, "timestamp": self.clock, "fieldOverlay": {"ops": []}}]})

    def pull(self):
        self.pinpoint_status = "NOT_READY"
        self.telemetry({rb.KEY_PINPOINT_STATUS: "NOT_READY", rb.KEY_LOOP: "80"})

    def plug(self):
        self.pinpoint_status = "READY"
        self.telemetry({rb.KEY_PINPOINT_STATUS: "READY", rb.KEY_LOOP: "120"})

    def on_message(self, session, msg):
        kind = msg.get("type")
        with self.lock:
            params = json.loads(self.bench.get("files/bench/params.json") or "{}")
            name = msg.get("opModeName") or self.status["activeOpMode"]
            if kind == "INIT_OP_MODE" and name == "Bench: Pinpoint":
                self.inits.append(params)
                mode = params.get("mode")
                result = {"present": self.present_pinpoint, "deviceVersion": 4 if self.present_pinpoint else 0}
                if self.present_pinpoint and mode == "seed":
                    result.update(info={"seed": "Pose(56, 8, 90)"}, seed={"ran": True, "loops": 400,
                                                                          "maxOffsetIn": 0.01, "maxTurnDeg": 0.02})
                if self.present_pinpoint and mode in ("dropout", "rawDropout"):
                    result["dropout"] = {"badLoops": 30, "silentBadLoops": 30, "poseDuringDropout": "zeroed"}
                if self.present_pinpoint and mode == "soak":
                    result = good_soak(params.get("seconds"))
                if self.present_pinpoint and mode == "recal":
                    result.update(good_recal())
                file = "pinpoint_" + {"rawDropout": "raw_dropout"}.get(mode, mode)
                self.bench[f"files/bench/{file}.json"] = json.dumps(result)
            if kind == "INIT_OP_MODE" and name == "Bench: Pinpoint Absent":
                self.inits.append({"mode": "absent"})
        super().on_message(session, msg)
        with self.lock:
            if kind == "START_OP_MODE" and name == "Bench: Pinpoint":
                self.telemetry({rb.KEY_PINPOINT_STATUS: self.pinpoint_status, rb.KEY_LOOP: "50"})


def station_run(argv, hub, actions=()):
    OUT.mkdir(parents=True, exist_ok=True)
    out = pathlib.Path(tempfile.mkdtemp(prefix="run-", dir=OUT)) / "run"
    handlers = signal.getsignal(signal.SIGINT), signal.getsignal(signal.SIGHUP)
    terminal = Terminal(hub, actions)
    saved, sys.stdin = sys.stdin, terminal
    interrupted = False
    try:
        with hub.installed(rb), contextlib.redirect_stdout(io.StringIO()):
            try:
                code = rb.Runner(rb.parse_args(["--out", str(out), "--ports-empty"] + list(argv))).run()
            except KeyboardInterrupt:
                code, interrupted = None, True
    finally:
        sys.stdin = saved
        signal.signal(signal.SIGINT, handlers[0])
        signal.signal(signal.SIGHUP, handlers[1])
    return code, out, terminal, interrupted


def prompts(out):
    return [line.split(">>> ", 1)[1].split(" ")[0] + " " + line.split(">>> ", 1)[1].split(" ")[2]
            for line in (out / "run.log").read_text().splitlines() if "] >>> " in line]


def check_line(summary, part):
    return next((l for l in summary.splitlines() if l.startswith("- ") and part in l), "")


def configs_written(out):
    return sorted(p.stem for p in (out / "logs").glob("bench_*.xml"))


def on_the_fake_hub():
    hub = Station()
    code, out, terminal, _ = station_run(["--pinpoint", "--interactive"], hub,
                                         [hub.pull, hub.plug, hub.pull, hub.plug])
    summary = (out / "summary.md").read_text()
    expect("--pinpoint --interactive: seed, then the raw driver's unplug, then the follower's",
           [p.get("mode") for p in hub.inits] == ["seed", "rawDropout", "dropout"])
    expect("--pinpoint --interactive: four prompts in order, each naming Control Hub I2C port 1 for the raw run",
           prompts(out) == ["PULL Pinpoint's", "PLUG Pinpoint's", "PULL Pinpoint's", "PLUG Pinpoint's"]
           and "Control Hub I2C port 1" in (out / "run.log").read_text().split(">>> PULL", 1)[1].split("\n", 1)[0])
    expect("--pinpoint --interactive: the raw unplug passes on a zeroed pose and its section says so",
           check_line(summary, "pinpoint raw dropout: the unplug showed").startswith("- PASS")
           and "the pose then zeroed" in summary)
    expect("--pinpoint without --pinpoint-soak: no soak, no recalibration run, no bench_octoquad",
           "bench_octoquad" not in configs_written(out) and "soak" not in [p.get("mode") for p in hub.inits])
    expect("the run ended with the original config back and no bench config left",
           hub.active["name"] == "betta_decode" and not [n for n in hub.configs if n.startswith("bench_")])

    hub = Station()
    code, out, terminal, _ = station_run(["--pinpoint-soak", "--quick"], hub)
    summary = (out / "summary.md").read_text()
    modes = [p.get("mode") for p in hub.inits]
    soak = next((p for p in hub.inits if p.get("mode") == "soak"), {})
    expect("--pinpoint-soak --quick: the seed run, then the 300 s soak, then the recalibration as its own OpMode run",
           modes == ["seed", "soak", "recal"] and soak.get("seconds") == 300)
    expect("--pinpoint-soak: no prompt without --interactive", not terminal.reads)
    expect("--pinpoint-soak: the soak runs on bench_octoquad, the Pinpoint on bus 1 and the OctoQuad on bus 2",
           "bench_octoquad" in configs_written(out)
           and 'bus="2"' in (out / "logs" / "bench_octoquad.xml").read_text()
           and 'bus="1"' in (out / "logs" / "bench_octoquad.xml").read_text())
    expect("--pinpoint-soak: its checks are in summary.md and pass on a quiet soak",
           all(check_line(summary, n).startswith("- PASS") for n in (
               "pinpoint soak: ran its 300 s", "pinpoint soak: no status other than READY",
               "pinpoint soak: no Pinpoint read over 40 ms", "pinpoint soak: setup:",
               "pinpoint soak: the Pinpoint's heading", "pinpoint recalibration:")))
    results = json.loads((out / "runner_results.json").read_text())
    expect("--pinpoint-soak: the soak and recalibration outcomes are in runner_results.json",
           "pinpoint_soak" in results and "pinpoint_recal" in results)
    expect("--pinpoint-soak: the original config is back afterwards",
           hub.active["name"] == "betta_decode" and not [n for n in hub.configs if n.startswith("bench_")])

    hub = Station(present=False)
    code, out, terminal, _ = station_run(["--pinpoint-soak", "--quick", "--interactive"], hub)
    summary = (out / "summary.md").read_text()
    expect("no Pinpoint: Bench: Pinpoint Absent instead, no soak, no unplug prompt, and the summary says the soak "
           "was skipped", [p.get("mode") for p in hub.inits] == ["seed", "absent"] and not terminal.reads
           and "soak and recalibration runs skipped" in summary)

    hub = Station()
    code, out, terminal, interrupted = station_run(["--pinpoint", "--interactive"], hub, [KeyboardInterrupt])
    expect("Ctrl-C at the raw unplug prompt: the runner stops, puts the original config back and leaves no bench "
           "config", interrupted and len(terminal.reads) == 1 and hub.active["name"] == "betta_decode"
           and not [n for n in hub.configs if n.startswith("bench_")])


def main():
    for section in (verdict_cases, mutation_cases, flags, on_the_fake_hub):
        try:
            section()
        except Exception as e:
            expect(f"{section.__name__} ran to its end ({type(e).__name__}: {e})", False)
            traceback.print_exc()
    print(f"{passed} passed, {len(failed)} failed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
