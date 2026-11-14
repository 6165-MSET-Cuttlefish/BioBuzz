"""Bench: OctoQuad's chain (--octoquad-chain) offline: its verdicts on synthetic records (each check fails when weakened),
and run_octoquad end to end on fake_hub.py: params.json for every run (portX, portY, guard), the prompts in order (roll,
unguarded pull, replug, guarded pull, late plug-in), Session A's prompts as before, the skip with no OctoQuad, and
Ctrl-C at a prompt putting the config back. No hub, no adb, no network.
Run with: python3 -I scripts/bench/offline/benches/octoquad.py
"""
import ast
import contextlib
import copy
import importlib.util
import io
import json
import pathlib
import re
import signal
import sys
import tempfile
import traceback

sys.dont_write_bytecode = True
HERE = pathlib.Path(__file__).resolve().parent
OFFLINE = HERE.parent
WORKTREE = OFFLINE.parents[2]
OUT = OFFLINE / "out" / "octoquad"
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
GUARD_ERROR = ('java.lang.IllegalStateException: OctoQuad "octoquad" (Control Hub I2C bus 2) sent no valid localizer '
               'data for 262 ms: no valid reply')
QUAD, MIXED = rb.OCTOQUAD_QUADRATURE, rb.OCTOQUAD_MIXED_BANKS
FOUND = {"chipId": 81, "uptimeMs": 900000, "bank": QUAD, "intervalsMs": [50] * 8, "directionsReversed": [1, 1] + [0] * 6}
MARKED = dict(FOUND, uptimeMs=901000, bank=MIXED, intervalsMs=[50] * 7 + [17])
REBOOTED = dict(FOUND, uptimeMs=2100, directionsReversed=[0] * 8)
RUNNING = {"crashed": False, "errorMessage": "", "timedOut": False, "endedItself": False}
DONE = dict(RUNNING, endedItself=True)


def good_chain():
    return {
        "ports": dict(rb.OCTOQUAD_CHAIN_PORTS),
        "earlystart": {"outcome": dict(DONE), "result": {"early": {"startWaitMs": 1310.0, "validAtLastInitLoop": False,
                                                                   "msSinceCreateAtStart": 340.0, "ranSeconds": 5.01}}},
        "markers": {"outcome": dict(DONE), "runner": {}, "result": {"markers": {
            "podPorts": [0, 1], "found": FOUND, "markersSet": MARKED, "markersTook": True,
            "positionsBefore": [0] * 8, "positionsAfterRoll": [2048, 3] + [0] * 6, "rollCounts": 2048,
            "afterLocalizerReset": {"leftRunningMs": 3.0, "runningMs": 1650.0, "status": "RUNNING", "bank": MIXED},
            "positionsAfterLocalizerReset": [2048, 3] + [0] * 6, "localizerResetZeroedPods": False,
            "resetEverythingAnsweredMs": 120.0, "afterResetEverything": REBOOTED, "rebootedByResetEverything": True,
            "markerIntervalKeptThroughResetEverything": False, "bankKeptThroughResetEverything": False,
            "intervalChanges": [[7, 17, 50]], "directionChanges": [[0, 1, 0], [1, 1, 0]],
            "positionsAfterResetEverything": [0] * 8, "resetEverythingZeroedPods": True,
            "statusAfterResetEverything": "NOT_INITIALIZED", "restored": {"bank": QUAD, "interval7Ms": 50},
            "step": "done", "readFailures": []}}},
        "unguarded": {"outcome": dict(RUNNING), "runner": {"invalidSeenS": 0.4, "runningAfterOutS": True,
                                                            "replugReadS": 1.2, "validAgainS": None, "warnings": []},
                      "result": {"dropout": {"badLoops": 700, "silentBadLoops": 700, "poseDuringDropout": "frozen",
                                             "firstInvalidStatus": "INVALID", "firstInvalidCrcOk": False},
                                 "unguarded": {"markersSet": MARKED, "afterReplug": REBOOTED, "answeredAgainMs": 14000.0,
                                               "noReplyForMs": 11000.0, "rebooted": True, "bankKept": False,
                                               "markerIntervalKept": False, "intervalChanges": [[7, 17, 50]],
                                               "directionChanges": [[0, 1, 0], [1, 1, 0]],
                                               "restored": {"bank": QUAD, "interval7Ms": 50}, "faultsAtEnd": "None"}}},
        "guarded": {"outcome": dict(DONE, crashed=True, errorMessage=GUARD_ERROR), "runner": {"stoppedS": 0.6},
                    "result": {"dropout": {"firstInvalidMs": 3000.0, "finishedMs": 3262.0, "badLoops": 12,
                                           "silentBadLoops": 12},
                               "loopPeriodInvalidMs": {"count": 24, "p50": 10.5, "p95": 12.0, "max": 14.0}}},
        "latejoin": {"outcome": dict(DONE), "runner": {"faultSeenS": 3.1, "clearSeenS": 0.4, "validSeenS": 1.8},
                     "result": {"latejoin": {"answeredAtInit": False, "faultMs": 3010.0, "answerMs": 9000.0,
                                             "answerStatus": "NOT_INITIALIZED", "clearMs": 9000.0,
                                             "clearAfterAnswerMs": 0.0, "runningMs": 10650.0,
                                             "runningAfterAnswerMs": 1650.0, "runningPhase": "init",
                                             "podPorts": [0, 1], "directionsReversed": [1, 1],
                                             "directionsExpected": [1, 1], "directionsMatch": True},
                                "seed": {"ran": True, "loops": 300, "maxOffsetIn": 0.02, "maxTurnDeg": 0.03}}},
    }


def change(path, value):
    def make():
        chain = good_chain()
        target = chain
        for key in path[:-1]:
            target = target[key]
        if value is DELETE:
            del target[path[-1]]
        else:
            target[path[-1]] = value
        return chain
    return make


DELETE = object()
KEYS = (("early", "octoquad earlystart"), ("markers", "octoquad markers"), ("control", "octoquad unguarded control"),
        ("replug", "octoquad power cycle"), ("guarded", "octoquad guarded"), ("fault", "octoquad latejoin: with the"),
        ("joined", "octoquad latejoin: INIT with"), ("ran", "octoquad latejoin: ran"),
        ("directions", "octoquad latejoin: the pod directions"), ("seed", "octoquad latejoin: a start pose"))
ALL = ("early", "markers", "control", "replug", "guarded", "fault", "directions", "seed")


def verdicts(fn, chain):
    out = {}
    checks, lines = fn(chain)
    for name, ok, detail, means in checks:
        key = next((k for k, prefix in KEYS if name.startswith(prefix)), name)
        out[key] = ok
    return out, "\n".join(lines)


def only_fails(v, *keys, judged=ALL):
    return set(v) == set(judged) and {k for k, ok in v.items() if not ok} == set(keys)


CASES = [
    ("a good chain passes every check", good_chain, lambda v, t: only_fails(v)),
    ("a lost marker and bank through the power cycle are reported, not failed", good_chain,
     lambda v, t: only_fails(v) and "bank kept False" in t and "channel 7's 17 ms kept False" in t),
    ("an unguarded run that stops on its own fails the control and voids the guarded verdict",
     change(("unguarded", "outcome"), dict(DONE, crashed=True, errorMessage="boom")),
     lambda v, t: only_fails(v, "control", judged=tuple(k for k in ALL if k != "guarded")) and "is void" in t),
    ("a pose that snapped to zero under Pedro's localizer fails the control (and voids nothing: it kept running)",
     change(("unguarded", "result", "dropout", "poseDuringDropout"), "zeroed"), lambda v, t: only_fails(v, "control")),
    ("an OctoQuad fault under Pedro's localizer fails the control",
     change(("unguarded", "result", "dropout", "silentBadLoops"), 0), lambda v, t: only_fails(v, "control")),
    ("a control whose pull never showed fails it",
     change(("unguarded", "result", "dropout", "badLoops"), 0), lambda v, t: only_fails(v, "control")),
    ("a control skipped (not RUNNING before the pull) fails and voids the guarded verdict",
     change(("unguarded", "runner"), {"skipped": "not valid"}),
     lambda v, t: only_fails(v, "control", judged=tuple(k for k in ALL if k not in ("guarded", "replug")))
     and "is void" in t),
    ("RUNNING 6 s after the plug-in fails", change(("latejoin", "result", "latejoin", "runningAfterAnswerMs"), 6000.0),
     lambda v, t: only_fails(v, "fault")),
    ("the fault clearing 6 s after the plug-in fails",
     change(("latejoin", "result", "latejoin", "clearAfterAnswerMs"), 6000.0), lambda v, t: only_fails(v, "fault")),
    ("no fault in INIT within 5 s fails", change(("latejoin", "result", "latejoin", "faultMs"), 5200.0),
     lambda v, t: only_fails(v, "fault")),
    ("RUNNING never coming fails", change(("latejoin", "result", "latejoin", "runningAfterAnswerMs"), DELETE),
     lambda v, t: only_fails(v, "fault")),
    ("a late join with the cable in at INIT fails as not run",
     change(("latejoin", "runner"), {"skipped": "no fault"}),
     lambda v, t: only_fails(v, "joined", judged=tuple(k for k in ALL if k not in ("fault", "directions", "seed"))
                             + ("joined",))),
    ("directions read back FORWARD fail", change(("latejoin", "result", "latejoin", "directionsMatch"), False),
     lambda v, t: only_fails(v, "directions")),
    ("a start pose 0.3 in off after the late join fails", change(("latejoin", "result", "seed", "maxOffsetIn"), 0.3),
     lambda v, t: only_fails(v, "seed")),
    ("a start pose 0.5 deg off fails", change(("latejoin", "result", "seed", "maxTurnDeg"), 0.5),
     lambda v, t: only_fails(v, "seed")),
    ("a late join that crashed fails the start-pose check, though its INIT loops held the pose",
     change(("latejoin", "outcome"), dict(DONE, crashed=True, errorMessage="java.lang.IllegalStateException: boom")),
     lambda v, t: only_fails(v, "seed")),
    ("a late join whose seed phase never ran (never STARTed) fails the start-pose check",
     change(("latejoin", "result", "seed", "ran"), False), lambda v, t: only_fails(v, "seed")),
    ("the guard stopping after one read (50 ms) fails", change(("guarded", "result", "dropout", "finishedMs"), 3050.0),
     lambda v, t: only_fails(v, "guarded")),
    ("the guard stopping at 240 ms, under its own 250 ms less 5, fails",
     change(("guarded", "result", "dropout", "finishedMs"), 3240.0), lambda v, t: only_fails(v, "guarded")),
    ("the guard stopping at 245 ms passes", change(("guarded", "result", "dropout", "finishedMs"), 3245.0),
     lambda v, t: only_fails(v)),
    ("the guard stopping at 274 ms, 2 x the 12 ms loop p95 past 250, passes",
     change(("guarded", "result", "dropout", "finishedMs"), 3274.0), lambda v, t: only_fails(v)),
    ("the guard stopping at 275 ms fails", change(("guarded", "result", "dropout", "finishedMs"), 3275.0),
     lambda v, t: only_fails(v, "guarded")),
    ("a guard four times slower than its 250 ms (1000 ms) fails",
     change(("guarded", "result", "dropout", "finishedMs"), 4000.0), lambda v, t: only_fails(v, "guarded")),
    ("no loop period while invalid: no window, so it fails",
     change(("guarded", "result", "loopPeriodInvalidMs"), {"count": 0}), lambda v, t: only_fails(v, "guarded")),
    ("the guard taking 1.5 s to stop fails", change(("guarded", "result", "dropout", "finishedMs"), 4500.0),
     lambda v, t: only_fails(v, "guarded")),
    ("the guarded OpMode not stopping by itself fails", change(("guarded", "outcome"), dict(RUNNING)),
     lambda v, t: only_fails(v, "guarded")),
    ("an error naming the OctoQuad with the OpMode still running at the runner's STOP fails",
     change(("guarded", "outcome"), dict(RUNNING, errorMessage=GUARD_ERROR)), lambda v, t: only_fails(v, "guarded")),
    ("an error that doesn't name the OctoQuad fails",
     change(("guarded", "outcome"), dict(DONE, crashed=True, errorMessage="java.lang.NullPointerException")),
     lambda v, t: only_fails(v, "guarded")),
    ("a skipped guarded pull fails", change(("guarded", "runner"), {"skipped": "not valid"}),
     lambda v, t: only_fails(v, "guarded")),
    ("START waiting 3.2 s fails", change(("earlystart", "result", "early", "startWaitMs"), 3200.0),
     lambda v, t: only_fails(v, "early")),
    ("an early START that threw fails",
     change(("earlystart", "outcome"), dict(DONE, crashed=True, errorMessage="not RUNNING")),
     lambda v, t: only_fails(v, "early")),
    ("an early START that didn't run 5 s fails", change(("earlystart", "result", "early", "ranSeconds"), 2.0),
     lambda v, t: only_fails(v, "early")),
    ("a markers read that didn't answer within 5 s fails",
     change(("markers", "result", "markers", "readFailures"), ["chip id after resetEverything"]),
     lambda v, t: only_fails(v, "markers")),
    ("a markers run that stopped early fails", change(("markers", "result", "markers", "step"), "roll"),
     lambda v, t: only_fails(v, "markers")),
    ("no answer after the replug fails the power-cycle check",
     change(("unguarded", "result", "unguarded", "afterReplug"), DELETE), lambda v, t: only_fails(v, "replug")),
    ("an empty chain: every part fails as not run, nothing passes", lambda: {},
     lambda v, t: set(v) == {"early", "markers", "control", "guarded", "ran"} and not any(v.values())),
]


def run_cases(fn):
    out = []
    for label, make, predicate in CASES:
        try:
            v, text = verdicts(fn, make())
            ok = bool(predicate(v, text))
        except Exception as e:
            ok = False
            label = f"{label} ({type(e).__name__}: {e})"
        out.append((label, ok))
    return out


def function_source(name):
    node = next(n for n in TREE.body if isinstance(n, ast.FunctionDef) and n.name == name)
    return ast.get_source_segment(SOURCE, node)


def mutant(old, new, **constants):
    src = function_source("octoquad_chain_checks")
    if old and src.count(old) != 1:
        return None
    namespace = dict(vars(rb))
    namespace.update(constants)
    exec(compile(src.replace(old, new) if old else src, "<octoquad_chain_checks weakened>", "exec"), namespace)
    return namespace["octoquad_chain_checks"]


MUTATIONS = [
    ("a stopped control no longer voids the guarded verdict",
     'kept_running = not o.get("crashed") and not o.get("endedItself") and "skipped" not in runner',
     "kept_running = True", {}),
    ("the control's frozen-pose test is dropped", 'and d.get("poseDuringDropout") == "frozen"', "", {}),
    ("the control's no-fault test is dropped", "bad > 0 and silent == bad and", "bad > 0 and", {}),
    ("the late join's 5 s limit is out of reach", "", "", {"OCTOQUAD_LATE_MS": 10 ** 9}),
    ("the guard's streak floor is gone", "", "", {"OCTOQUAD_GUARD_LIMIT_MS": 50}),
    ("the guard's ceiling is gone", "hi = OCTOQUAD_GUARD_LIMIT_MS + 2 * p95", "hi = 10 ** 9 + p95", {}),
    ("the guard's error needn't name the OctoQuad", "OCTOQUAD_GUARD_PREFIX in error.lower() and ", "", {}),
    ("the guard needn't stop by itself", 'bool(o.get("endedItself")) and OCTOQUAD_GUARD', "OCTOQUAD_GUARD", {}),
    ("the early START's 3 s limit is out of reach", "", "", {"OCTOQUAD_EARLY_WAIT_MS": 10 ** 9}),
    ("the early START needn't run 5 s", "and ran >= OCTOQUAD_EARLY_RUN_S", "", {}),
    ("markers read failures are ignored", "and failures == []", "", {}),
    ("the directions needn't match", 'lj.get("directionsMatch") is True', "True", {}),
    ("the late join's seed offset is ignored", "and offset <= PINPOINT_SEED_OFFSET_IN", "", {}),
    ("a crashed late join's seed passes", 'not o.get("crashed") and seed.get("ran") is True and finite(offset)',
     'seed.get("ran") is True and finite(offset)', {}),
    ("a late join's seed needn't have run", 'seed.get("ran") is True and finite(offset)', "finite(offset)", {}),
    ("the replug answer is ignored", "bool(after), f\"answered again", "True, f\"answered again", {}),
]


def verdict_cases():
    for label, ok in run_cases(rb.octoquad_chain_checks):
        expect(f"verdict: {label}", ok)
    checks, lines = rb.octoquad_chain_checks(good_chain())
    text = "\n".join(lines)
    expect("summary: the guard's stop time and its own streak count", "stopped 262 ms after it" in text
           and "the guard's own count 262 ms" in text)
    expect("summary: what resetLocalizerAndCalibrateIMU and resetEverything did to the counts and the markers",
           "pods zeroed False" in text and "pods zeroed True" in text and "rebooted True" in text)
    names = [c[0] for c in checks]
    expect("checks: eight on a good chain, all different", len(names) == 8 and len(set(names)) == 8)
    guard = (WORKTREE / "TeamCode/src/main/java/org/firstinspires/ftc/teamcode/architecture/hardware/"
             "GuardedOctoQuadLocalizer.java").read_text()
    limit = re.search(r"INVALID_LIMIT_NANOS = ([\d_]+)L;", guard)
    expect("the runner's guard limit is GuardedOctoQuadLocalizer's INVALID_LIMIT_NANOS",
           limit is not None and int(limit.group(1).replace("_", "")) == rb.OCTOQUAD_GUARD_LIMIT_MS * 1_000_000)
    expect("the guarded check names its window and where it comes from",
           "window 245-274 ms: the guard's 250 ms" in next(c[2] for c in checks if c[0].startswith("octoquad guarded")))


def mutation_cases():
    for label, old, new, constants in MUTATIONS:
        fn = mutant(old, new, **constants)
        if fn is None:
            expect(f"weakened: {label}: the text to weaken is in octoquad_chain_checks once", False)
            continue
        caught = [case for case, ok in run_cases(fn) if not ok]
        expect(f"weakened: {label}: a verdict case fails ({caught[:1]})", bool(caught))


def flags():
    expect("--octoquad-chain --interactive implies --octoquad", rb.parse_args(["--octoquad-chain", "--interactive"])
           .octoquad)
    for bad in (["--octoquad-chain"], ["--octoquad", "--octoquad-chain"]):
        try:
            with contextlib.redirect_stderr(io.StringIO()):
                rb.parse_args(bad)
            refused = False
        except SystemExit:
            refused = True
        expect(f"{' '.join(bad)} without --interactive is refused", refused)


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
    def __init__(self, present=True, late_fault=True, **kw):
        super().__init__(**kw)
        self.present_octoquad = present
        self.late_fault = late_fault
        self.inits = []
        self.mode = None

    def telemetry(self, data):
        self.dash.broadcast({"type": "RECEIVE_TELEMETRY", "telemetry": [
            {"data": data, "timestamp": self.clock, "fieldOverlay": {"ops": []}}]})

    def pull(self):
        if self.mode == "dropout" and self.inits[-1].get("guard", 1) == 1 and "portX" in self.inits[-1]:
            with self.lock:
                self.status["errorMessage"] = GUARD_ERROR
                self.set_active(rb.IDLE_OPMODE, "STOPPED")
            return
        self.telemetry({rb.KEY_OCTOQUAD_VALID: "false", rb.KEY_LOOP: "300"})

    def plug(self):
        if self.mode == "latejoin":
            self.telemetry({rb.KEY_OCTOQUAD_FAULT: "cleared", rb.KEY_OCTOQUAD_VALID: "true"})
        else:
            self.telemetry({rb.KEY_OCTOQUAD_REPLUG: "read", rb.KEY_OCTOQUAD_VALID: "true"})

    def result(self, mode):
        chain = good_chain()
        if not self.present_octoquad:
            return {"present": False, "chipId": 0}
        base = {"present": True, "chipId": 81, "info": {"seed": "Pose(56, 8, 90)"}, "phaseLog": ["create", mode]}
        if mode == "seed":
            base.update(seed={"ran": True, "loops": 300, "maxOffsetIn": 0.01, "maxTurnDeg": 0.02}, crcFailures=0,
                        invalidLoops=0)
        elif mode in ("earlystart", "markers", "unguarded", "latejoin"):
            base.update(chain[mode]["result"])
        elif mode == "dropout":
            base.update(chain["guarded"]["result"])
        return base

    def on_message(self, session, msg):
        kind = msg.get("type")
        with self.lock:
            params = json.loads(self.bench.get("files/bench/params.json") or "{}")
            name = msg.get("opModeName") or self.status["activeOpMode"]
            if kind == "INIT_OP_MODE" and name == "Bench: OctoQuad":
                self.status["errorMessage"] = ""
                self.inits.append(params)
                self.mode = params.get("mode")
                self.bench[f"files/bench/octoquad_{self.mode}.json"] = json.dumps(self.result(self.mode))
        super().on_message(session, msg)
        with self.lock:
            if kind == "INIT_OP_MODE" and name == "Bench: OctoQuad" and self.mode == "latejoin" and self.late_fault:
                self.telemetry({rb.KEY_OCTOQUAD_FAULT: "up", rb.KEY_OCTOQUAD_VALID: "false"})
            if kind == "START_OP_MODE" and name == "Bench: OctoQuad":
                self.telemetry({rb.KEY_OCTOQUAD_VALID: "true", rb.KEY_LOOP: "50"})
                if self.mode == "markers":
                    self.telemetry({rb.KEY_PHASE: "roll"})


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
    return [line.split("] >>> ", 1)[1] for line in (out / "run.log").read_text().splitlines() if "] >>> " in line]


def check_line(summary, part):
    return next((l for l in summary.splitlines() if l.startswith("- ") and part in l), "")


def on_the_fake_hub():
    hub = Station()
    code, out, terminal, _ = station_run(["--octoquad", "--octoquad-chain", "--interactive"], hub,
                                         [None, hub.pull, hub.plug, hub.pull, hub.plug])
    summary = (out / "summary.md").read_text()
    ports = rb.OCTOQUAD_CHAIN_PORTS
    expect("chain: params.json for each run: seed, earlystart, markers, unguarded with guard 0, the guarded pull, "
           "latejoin, all with the pods on channels 0 and 1",
           hub.inits == [dict(mode="seed", **ports), dict(mode="earlystart", **ports), dict(mode="markers", **ports),
                         dict(mode="unguarded", guard=0, **ports), dict(mode="dropout", **ports),
                         dict(mode="latejoin", **ports)])
    expect("chain: the prompts in order: roll, unguarded pull, replug, guarded pull (left out), late plug-in",
           prompts(out) == [rb.OCTOQUAD_ROLL[4:], rb.OCTOQUAD_PULL[4:], rb.OCTOQUAD_PLUG[4:],
                            rb.OCTOQUAD_PULL_LEAVE[4:], rb.OCTOQUAD_PLUG[4:]])
    expect("chain: the runner told the markers run the pod was rolled, through the bench folder",
           json.loads(hub.bench.get(f"files/bench/{rb.OCTOQUAD_ROLLED}") or "{}") == {"rolled": True})
    expect("chain: every chain check passes on good runs, the guarded one included",
           all(check_line(summary, n).startswith("- PASS") for n in (
               "octoquad earlystart:", "octoquad markers:", "octoquad unguarded control:", "octoquad power cycle:",
               "octoquad guarded:", "octoquad latejoin: with the", "octoquad latejoin: the pod directions",
               "octoquad latejoin: a start pose")))
    expect("chain: no Session A replug prompt after the guarded pull, and no Session A dropout check",
           "PLUG the OctoQuad's I2C cable back in, then" not in "\n".join(prompts(out))
           and not check_line(summary, "octoquad dropout: an invalid localizer read stops"))
    results = json.loads((out / "runner_results.json").read_text())
    expect("chain: its record is in runner_results.json under octoquad_chain, one entry per step",
           {"earlystart", "markers", "unguarded", "guarded", "latejoin"} <= set(results.get("octoquad_chain") or {}))
    expect("chain: one config switch to bench_octoquad for the whole chain, and the original config back at the end",
           hub.active["name"] == "betta_decode" and not [n for n in hub.configs if n.startswith("bench_")])

    hub = Station(late_fault=False)
    code, out, terminal, _ = station_run(["--octoquad", "--octoquad-chain", "--interactive"], hub,
                                         [None, hub.pull, hub.plug, hub.pull, None])
    log = (out / "run.log").read_text()
    expect("no OctoQuad fault at the late join: the plug-in is asked for after the run, and nothing says the cable was in",
           prompts(out)[-2:] == [rb.OCTOQUAD_PULL_LEAVE[4:], rb.OCTOQUAD_PLUG[4:]] and len(terminal.reads) == 5
           and "late plug-in skipped" in log and "was in Control Hub" not in log
           and rb.OCTOQUAD_LEFT_OUT not in log)

    hub = Station()
    code, out, terminal, interrupted = station_run(["--octoquad", "--octoquad-chain", "--interactive"], hub,
                                                   [None, hub.pull, hub.plug, hub.pull, KeyboardInterrupt])
    expect("Ctrl-C at the late plug-in: the log says to plug the OctoQuad back in, and the config is put back",
           interrupted and rb.OCTOQUAD_LEFT_OUT in (out / "run.log").read_text()
           and hub.active["name"] == "betta_decode" and not [n for n in hub.configs if n.startswith("bench_")])

    hub = Station()
    code, out, terminal, interrupted = station_run(["--octoquad", "--octoquad-chain", "--interactive"], hub,
                                                   [None, hub.pull, KeyboardInterrupt])
    expect("Ctrl-C at the control's replug: the log says to plug the OctoQuad back in",
           interrupted and rb.OCTOQUAD_LEFT_OUT in (out / "run.log").read_text())

    hub = Station()
    code, out, terminal, interrupted = station_run(["--octoquad", "--octoquad-chain", "--interactive"], hub,
                                                   [KeyboardInterrupt])
    expect("Ctrl-C at the roll prompt, before any pull: no OctoQuad left-out line",
           interrupted and rb.OCTOQUAD_LEFT_OUT not in (out / "run.log").read_text())

    hub = Station()
    code, out, terminal, _ = station_run(["--octoquad", "--interactive"], hub, [hub.pull, hub.plug])
    expect("Session A (--octoquad --interactive): the seed run as before, then the pull and the replug",
           [p for p in hub.inits] == [{"mode": "seed"}, {"mode": "dropout"}]
           and [p.split(" ")[0] for p in prompts(out)] == ["PULL", "PLUG"])
    expect("Session A: its dropout check says no unguarded control ran",
           "no unguarded control ran" in check_line((out / "summary.md").read_text(), "octoquad dropout:"))

    hub = Station(present=False)
    code, out, terminal, _ = station_run(["--octoquad", "--octoquad-chain", "--interactive"], hub)
    expect("no OctoQuad: the seed run reports it missing, and no chain run or prompt follows",
           [p.get("mode") for p in hub.inits] == ["seed"] and not terminal.reads
           and "no OctoQuad answered" in (out / "summary.md").read_text())

    hub = Station()
    code, out, terminal, interrupted = station_run(["--octoquad", "--octoquad-chain", "--interactive"], hub,
                                                   [KeyboardInterrupt])
    expect("Ctrl-C at the roll prompt: the runner stops, puts the original config back and leaves no bench config",
           interrupted and len(terminal.reads) == 1 and hub.active["name"] == "betta_decode"
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
