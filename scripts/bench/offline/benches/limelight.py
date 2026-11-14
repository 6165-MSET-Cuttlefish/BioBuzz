"""Bench: Limelight's ball phases offline: the sync, staleness, still-ball and pipeline-change verdicts on synthetic
results (each fails when weakened), then run_limelight end to end on fake_hub.py with a Limelight: its checks and
summary from a served result, the same params as before, no prompt, the skip without a Limelight and Ctrl-C putting
the config back. No hub, no adb, no network. Run with: python3 -I scripts/bench/offline/benches/limelight.py
"""
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
OUT = OFFLINE / "out" / "limelight"
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

KEYS = (("both upload syncs", "upload"), ("switch-only sync", "switch"), ("with syncOnInit off", "off"),
        ("INIT to the first", "first"), ("staleness:", "stale"), ("balls seen in at least", "count"),
        ("max deviation", "deviation"), ("speed p95", "speed"), ("track id", "ids"),
        ("LimelightSync thread", "senders"), ("new frames within", "fresh"), ("'on pipeline N", "wrong"),
        ("loop period", "loop"))
ALL = {k for _, k in KEYS}


def summary(n=10, mx=12.0):
    return {"count": n, "mean": mx / 2, "min": 1.0, "p50": mx / 2, "p90": mx * 0.8, "p95": mx * 0.9, "p99": mx,
            "max": mx, "largest": [mx]}


def ball(x, **over):
    b = {"meanXIn": x, "meanYIn": 0.2, "samples": 138, "framesSeen": 138, "framesMissing": 2, "sdIn": 0.3,
         "maxDeviationIn": 0.9, "maxSpreadIn": 1.6, "speedP50Ips": 0.5, "speedP95Ips": 1.4, "speedMaxIps": 2.0,
         "ids": [3], "idChanges": 0}
    b.update(over)
    return b


def change(to, upload, at, **over):
    c = {"to": to, "upload": upload, "atMs": at, "syncDoneMs": 1750.0 if upload else 230.0, "problemAtSyncDone": None,
         "firstFreshMs": 1980.0 if upload else None, "syncingAtEnd": False, "problems": [], "problemAtEnd": None,
         "loopMs": summary(), "sendersMax": 1}
    c.update(over)
    return c


def good_record():
    counts = [0] * 100
    counts[22:27] = [30, 120, 100, 25, 4]
    return {"phases": {
        "proxyQuery": {"from": 4, "to": 1, "indexChanges": [[0.0, 4], [300.0, 1]], "reachedToMs": 300.0,
                       "restoreAccepted": True},
        "sync": {"stamp": 8912345,
                 "first": {"upload": True, "done": True, "ms": 1720.0, "problem": None},
                 "again": {"upload": True, "done": True, "ms": 1690.0, "problem": None},
                 "switchOnly": {"upload": False, "done": True, "ms": 240.0, "problem": None},
                 "syncOnInitOff": {"gotFrame": True, "ms": 610.0, "problem": None, "switchToOtherAccepted": True}},
        "behindItsBack": {"firstFrame": {"gotFrame": True, "ms": 1800.0, "problem": None}},
        "staleness": {"firstFrame": {"gotFrame": True, "ms": 1850.0, "problem": None, "syncOnInit": True},
                      "liveThresholdS": 0.5, "seconds": 60.0, "loops": 20000, "frames": 279, "firstBallLoopMs": 700.0,
                      "loopsFromFirstBall": 19800, "loopsWithBalls": 19800, "liveEpisodes": 0,
                      "replayEpisodes": {"0.4": 0, "0.5": 0, "0.6": 0, "0.7": 0}, "smallestCleanThresholdS": 0.4,
                      "longestGapMs": 262.0, "gapMs": summary(278, 262.0),
                      "gapHistogram": {"bucketMs": 10, "counts": counts}, "arrivalsMs": [0.0, 233.0],
                      "updateMs": summary(), "problems": []},
        "stillBalls": {"seconds": 30.0, "frames": 140, "radiusIn": 6,
                       "balls": [ball(x) for x in (-5.6, 18.4, 42.4, 66.4)], "problems": []},
        "pipelineChange": {"start": change(4, True, 0, syncDoneMs=None),
                           "toOther": change(0, False, 300, problems=[[305.0, "pipeline 0 is a pipe_fiducial "
                                                                              "pipeline, not Python"]]),
                           "back": change(4, True, 1600, syncDoneMs=None, firstFreshMs=None, syncingAtEnd=True),
                           "otherInFlight": change(0, False, 1700, syncDoneMs=None, syncingAtEnd=True),
                           "last": change(4, True, 1800, problems=[[1810.0, None]]),
                           "finalPipelineIndex": 4,
                           "backRequestEnded": {"pipeline": 4, "upload": True, "done": True, "problem": "cancelled"},
                           "otherInFlightRequestEnded": {"pipeline": 0, "upload": False, "done": True,
                                                         "problem": "cancelled"},
                           "loopMs": summary(3000, 14.0), "sendersMax": 1,
                           "senders": [[0.0, 0], [305.0, 1], [700.0, 0], [1600.0, 1], [3700.0, 0]]}}}


def at(record, path, value):
    *head, last = path
    node = record
    for p in head:
        node = node[p]
    if value is DROP:
        node.pop(last, None)
    else:
        node[last] = value
    return record


DROP = object()


def edit(*changes):
    def make():
        r = good_record()
        for path, value in changes:
            at(r, path, value)
        return r
    return make


def fails(checks_fn, record):
    out = set()
    means = {}
    for name, ok, detail, fail_means in checks_fn(record):
        key = next((k for frag, k in KEYS if frag in name), name)
        if not ok:
            out.add(key)
            means[key] = fail_means
    return out, means


P = ("phases",)
CASES = [
    ("a run with every phase as it should be passes everything", good_record, set()),
    ("an upload sync over 2.5 s fails the upload check alone", edit((P + ("sync", "first", "ms"), 2600.0)), {"upload"}),
    ("an upload sync with a problem fails it", edit((P + ("sync", "again", "problem"),
                                                     "pipeline 4 never reported the upload's stamp")), {"upload"}),
    ("an upload sync not done fails it", edit((P + ("sync", "again", "done"), False)), {"upload"}),
    ("a switch-only sync over 1 s fails its check alone", edit((P + ("sync", "switchOnly", "ms"), 1100.0)), {"switch"}),
    ("a switch-only sync with a problem fails it",
     edit((P + ("sync", "switchOnly", "problem"), "pipeline 4 is a pipe_fiducial pipeline, not Python")), {"switch"}),
    ("a switch-only sync not done, though quick and with no problem, fails it",
     edit((P + ("sync", "switchOnly", "done"), False), (P + ("sync", "switchOnly", "ms"), 300.0)), {"switch"}),
    ("syncOnInit off without a frame fails its check alone",
     edit((P + ("sync", "syncOnInitOff", "gotFrame"), False),
          (P + ("sync", "syncOnInitOff", "problem"), "unknown SCRIPT_ID 7 on pipeline 4, want 1")), {"off"}),
    ("no sync phase fails all three sync checks", edit((P + ("sync",), DROP)), {"upload", "switch", "off"}),
    ("INIT to the first frame over 2.5 s fails that check alone",
     edit((P + ("staleness", "firstFrame", "ms"), 2600.0)), {"first"}),
    ("no first frame, though timed at 100 ms with no problem, fails that check alone",
     edit((P + ("staleness", "firstFrame", "gotFrame"), False), (P + ("staleness", "firstFrame", "ms"), 100.0)),
     {"first"}),
    ("98.5% of loops publishing balls fails the staleness check alone",
     edit((P + ("staleness", "loopsWithBalls"), 19503)), {"stale"}),
    ("exactly 99% passes", edit((P + ("staleness", "loopsWithBalls"), 19602)), set()),
    ("no ball ever published fails the staleness check",
     edit((P + ("staleness", "loopsFromFirstBall"), 0), (P + ("staleness", "loopsWithBalls"), 0),
          (P + ("staleness", "firstBallLoopMs"), None)), {"stale"}),
    ("no staleness phase fails it and the first-frame check", edit((P + ("staleness",), DROP)), {"stale", "first"}),
    ("a ball wandering 2.0 in fails the deviation check alone",
     edit((P + ("stillBalls", "balls", 3, "maxDeviationIn"), 2.0)), {"deviation"}),
    ("a ball at 1.99 in passes", edit((P + ("stillBalls", "balls", 3, "maxDeviationIn"), 1.99)), set()),
    ("a reported speed p95 of 3.0 in/s fails the speed check alone",
     edit((P + ("stillBalls", "balls", 0, "speedP95Ips"), 3.0)), {"speed"}),
    ("one id change fails the id check alone", edit((P + ("stillBalls", "balls", 1, "idChanges"), 1),
                                                    (P + ("stillBalls", "balls", 1, "ids"), [3, 9])), {"ids"}),
    ("a ball seen in under half the frames is a flicker, leaving 3 balls: the count fails alone",
     edit((P + ("stillBalls", "balls", 2, "framesSeen"), 69),
          (P + ("stillBalls", "balls", 2, "maxDeviationIn"), 5.0)), {"count"}),
    ("a ball read 7 in off on 20% of its frames, same id: that place is it jumping, so its deviation fails",
     lambda: at(good_record(), P + ("stillBalls", "balls"), [ball(x) for x in (-5.6, 18.4, 42.4, 66.4)]
                + [ball(73.4, framesSeen=28, samples=28, maxDeviationIn=0.2)]), {"deviation"}),
    ("the same jump under a new track id fails the id check too",
     lambda: at(good_record(), P + ("stillBalls", "balls"), [ball(x) for x in (-5.6, 18.4, 42.4, 66.4)]
                + [ball(66.4, meanYIn=11.9, framesSeen=28, samples=28, ids=[11])]), {"deviation", "ids"}),
    ("a flicker over 12 in from every ball is no ball's jump: nothing fails",
     lambda: at(good_record(), P + ("stillBalls", "balls"), [ball(x) for x in (-5.6, 18.4, 42.4, 66.4)]
                + [ball(30.4, meanYIn=6.0, framesSeen=10, samples=10, ids=[12])]), set()),
    ("five balls fail the count alone", lambda: at(good_record(), P + ("stillBalls", "balls"),
                                                   [ball(x) for x in (-5.6, 18.4, 30.0, 42.4, 66.4)]), {"count"}),
    ("no ball at all fails the count and every per-ball check",
     edit((P + ("stillBalls", "balls"), [])), {"count", "deviation", "speed", "ids"}),
    ("no stillBalls phase fails the same four", edit((P + ("stillBalls",), DROP)),
     {"count", "deviation", "speed", "ids"}),
    ("two LimelightSync threads at once fail the sender check alone",
     edit((P + ("pipelineChange", "sendersMax"), 2)), {"senders"}),
    ("the first new frame 3.1 s after the last change fails the fresh check alone",
     edit((P + ("pipelineChange", "last", "firstFreshMs"), 3100.0)), {"fresh"}),
    ("no new frame after the last change fails it", edit((P + ("pipelineChange", "last", "firstFreshMs"), None)),
     {"fresh"}),
    ("ending on another pipeline fails it", edit((P + ("pipelineChange", "finalPipelineIndex"), 0)), {"fresh"}),
    ("'on pipeline 0, want 4' after the last change fails the wrong-pipeline check alone",
     edit((P + ("pipelineChange", "last", "problems"), [[1810.0, None], [3900.0, "on pipeline 0, want 4"]])),
     {"wrong"}),
    ("'on pipeline 4, want 0' (another target) isn't that problem",
     edit((P + ("pipelineChange", "last", "problems"), [[1810.0, "on pipeline 4, want 0"]])), set()),
    ("a 50 ms loop fails the loop check alone", edit((P + ("pipelineChange", "loopMs", "max"), 50.0)), {"loop"}),
    ("no pipelineChange phase fails its four checks", edit((P + ("pipelineChange",), DROP)),
     {"senders", "fresh", "wrong", "loop"}),
    ("no result at all fails every check", lambda: None, ALL),
]


def run_cases(checks_fn):
    results = []
    for label, make, want in CASES:
        try:
            got, _ = fails(checks_fn, make())
            ok = got == want
            if not ok:
                label = f"{label} (failed {sorted(got)}, wanted {sorted(want)})"
        except Exception as e:
            ok = False
            label = f"{label} ({type(e).__name__}: {e})"
        results.append((label, ok))
    return results


def verdict_cases():
    for label, ok in run_cases(rb.limelight_ball_checks):
        expect(f"verdict: {label}", ok)
    names = [c[0] for c in rb.limelight_ball_checks(good_record())]
    expect("checks: thirteen, each named once, each found by one key",
           len(names) == 13 and len(set(names)) == 13
           and sorted(next(k for f, k in KEYS if f in n) for n in names) == sorted(ALL))
    _, means = fails(rb.limelight_ball_checks, edit((P + ("stillBalls", "balls"), []))())
    expect("no ball in view: the FAIL says where the S2 setup puts the balls", "24, 48, 72 and 96 in" in means["count"])
    text = "\n".join(rb.limelight_ball_summary(good_record()))
    expect("summary: the smallest clean threshold, the replay and the gap bins",
           "smallest staleFrameSeconds with no stale episode: 0.4 s" in text and "0.6 s: 0" in text
           and "230-240 ms: 120" in text)
    expect("summary: a row per ball, and every change with the cancelled requests' ends",
           text.count("| -5.6, 0.2 |") == 1 and text.count("| 66.4, 0.2 |") == 1
           and all(f"pipeline change {k}:" in text for k in ("toOther", "back", "otherInFlight", "last"))
           and "'problem': 'cancelled'" in text)
    jumping = at(good_record(), P + ("stillBalls", "balls"), [ball(x) for x in (-5.6, 18.4, 42.4, 66.4)]
                 + [ball(73.4, framesSeen=28, samples=28)])
    text = "\n".join(rb.limelight_ball_summary(jumping))
    expect("summary: a jump's row names the ball it belongs to, and the grouping is said",
           "| 73.4, 0.2 (ball at x 66.4 jumping, 7.0 in) |" in text and "within 6 in of its mean" in text)
    dirty = edit((P + ("staleness", "replayEpisodes"), {"0.4": 3, "0.5": 1, "0.6": 1, "0.7": 1}),
                 (P + ("staleness", "smallestCleanThresholdS"), None))()
    expect("summary: no clean threshold is said so",
           "no stale episode: none of 0.4, 0.5, 0.6, 0.7 s" in "\n".join(rb.limelight_ball_summary(dirty)))
    expect("summary: an empty record gives lines, not an exception", rb.limelight_ball_summary(None))


MUTATIONS = [
    ("the upload sync time limit is lifted", "LIMELIGHT_UPLOAD_SYNC_MS = 2500", "LIMELIGHT_UPLOAD_SYNC_MS = 1e9"),
    ("an upload sync's problem is ignored",
     'all(r.get("done") is True and r.get("problem") is None and', 'all(r.get("done") is True and'),
    ("the switch-only time limit is lifted", "LIMELIGHT_SWITCH_SYNC_MS = 1000", "LIMELIGHT_SWITCH_SYNC_MS = 1e9"),
    ("a switch-only sync needn't be done", 'switch.get("done") is True and switch.get("problem")',
     'switch.get("problem")'),
    ("the first frame needn't have come", 'first.get("gotFrame") is True and first.get("problem")',
     'first.get("problem")'),
    ("syncOnInit off needn't get a frame", 'off.get("gotFrame") is True and off.get("problem") is None,', "True,"),
    ("the first-frame limit is lifted", "LIMELIGHT_FIRST_FRAME_MS = 2500", "LIMELIGHT_FIRST_FRAME_MS = 1e9"),
    ("90% publishing is enough", "LIMELIGHT_PUBLISHED_SHARE = 0.99", "LIMELIGHT_PUBLISHED_SHARE = 0.9"),
    ("no ball published passes", "share is not None and share >= LIMELIGHT_PUBLISHED_SHARE",
     "share is None or share >= LIMELIGHT_PUBLISHED_SHARE"),
    ("any number of balls passes the count", "len(real) == LIMELIGHT_STILL_BALLS", "len(real) >= 1"),
    ("a flicker counts as a ball", "LIMELIGHT_STILL_SEEN_SHARE = 0.5", "LIMELIGHT_STILL_SEEN_SHARE = 0.0"),
    ("2.5 in of wander passes", "LIMELIGHT_STILL_DEVIATION_IN = 2.0", "LIMELIGHT_STILL_DEVIATION_IN = 2.5"),
    ("exactly 2 in passes", 'b["maxDeviationIn"] < LIMELIGHT_STILL_DEVIATION_IN',
     'b["maxDeviationIn"] <= LIMELIGHT_STILL_DEVIATION_IN'),
    ("4 in/s passes", "LIMELIGHT_STILL_SPEED_IPS = 3.0", "LIMELIGHT_STILL_SPEED_IPS = 4.0"),
    ("one id change passes", 'lambda b, j: b.get("idChanges") == 0', 'lambda b, j: b.get("idChanges") <= 1'),
    ("no ball passes the per-ball checks", "bool(real) and all(ok(b, j) for b, j in zip(real, jumps))",
     "all(ok(b, j) for b, j in zip(real, jumps))"),
    ("a jump passes the deviation check", "lambda b, j: (not j and finite", "lambda b, j: (finite"),
    ("only a place within the place radius is a jump", "LIMELIGHT_STILL_CAPTURE_IN = 12.0",
     "LIMELIGHT_STILL_CAPTURE_IN = 6.0"),
    ("a jump's new id isn't an id change", 'b.get("idChanges") == 0 and not jump_ids(b, j)', 'b.get("idChanges") == 0'),
    ("two senders pass", 'pc["sendersMax"] <= 1', 'pc["sendersMax"] <= 2'),
    ("the comeback limit is lifted", "LIMELIGHT_CHANGE_FRESH_MS = 3000", "LIMELIGHT_CHANGE_FRESH_MS = 1e9"),
    ("the final pipeline isn't looked at", 'pc.get("finalPipelineIndex") == ball_pipeline,', "True,"),
    ("the wrong-pipeline problem is never matched", "int(m.group(2)) == ball_pipeline", "False"),
    ("the wrong-pipeline check always passes", "bool(last) and not wrong,", "True,"),
    ("the loop limit is lifted", "LIMELIGHT_CHANGE_LOOP_MS = 50", "LIMELIGHT_CHANGE_LOOP_MS = 1e9"),
]


def block():
    text = RUNNER.read_text()
    start = text.index("LIMELIGHT_UPLOAD_SYNC_MS = ")
    return text[start:text.index("\ndef counted(summary):", start)]


def mutated(old, new):
    source = block()
    if source.count(old) != 1:
        return None
    ns = dict(vars(rb))
    exec(compile(source.replace(old, new), "mutant", "exec"), ns)
    return ns["limelight_ball_checks"]


def mutation_cases():
    for label, old, new in MUTATIONS:
        checks_fn = mutated(old, new)
        if checks_fn is None:
            expect(f"weakened: {label}: the text to weaken is in run_bench.py's Limelight block once", False)
            continue
        caught = [case for case, ok in run_cases(checks_fn) if not ok]
        expect(f"weakened: {label}: a verdict case fails ({caught[:1]})", bool(caught))


class FakeStdin:
    def __init__(self):
        self.reads = 0

    def isatty(self):
        return True

    def readline(self):
        self.reads += 1
        return "\n"


class LimelightHub(fake_hub.Hub):
    """fake_hub's Control Hub with a Limelight that serves `record` as Bench: Limelight's result at START."""

    def __init__(self, record, limelight=True, **kw):
        super().__init__(**kw)
        if limelight:
            self.probe["limelights"] = [{"status": "ok", "ipAddress": "172.29.0.1",
                                         "serial": "EthernetOverUsb:eth0:172.29.0.22"}]
        self.record = record
        self.inits = []

    def on_message(self, session, msg):
        with self.lock:
            if msg.get("type") == "INIT_OP_MODE":
                self.inits.append((msg["opModeName"], json.loads(self.bench.get("files/bench/params.json") or "{}")))
            if msg.get("type") == "START_OP_MODE" and self.status["activeOpMode"] == "Bench: Limelight":
                self.bench["files/bench/limelight.json"] = json.dumps(self.record)
        super().on_message(session, msg)


def hub_run(argv, hub, runner_class=None):
    OUT.mkdir(parents=True, exist_ok=True)
    out = pathlib.Path(tempfile.mkdtemp(prefix="run-", dir=OUT)) / "run"
    handlers = signal.getsignal(signal.SIGINT), signal.getsignal(signal.SIGHUP)
    stdin = FakeStdin()
    saved, sys.stdin = sys.stdin, stdin
    raised = None
    try:
        with hub.installed(rb), contextlib.redirect_stdout(io.StringIO()):
            try:
                code = (runner_class or rb.Runner)(rb.parse_args(["--out", str(out)] + list(argv))).run()
            except KeyboardInterrupt as e:
                code, raised = None, e
    finally:
        sys.stdin = saved
        signal.signal(signal.SIGINT, handlers[0])
        signal.signal(signal.SIGHUP, handlers[1])
    return code, out, stdin, raised


def summary_line(text, fragment):
    return next((l for l in text.splitlines() if l.startswith("- ") and fragment in l), "")


def restored(hub):
    return hub.active["name"] == "betta_decode" and not [n for n in hub.configs if n.startswith("bench_")]


def on_the_fake_hub():
    hub = LimelightHub(good_record())
    code, out, stdin, _ = hub_run(["--limelight", "--ports-empty"], hub)
    text = (out / "summary.md").read_text()
    log = (out / "run.log").read_text()
    params = [p for o, p in hub.inits if o == "Bench: Limelight"]
    expect("--limelight with a Limelight: Bench: Limelight ran once, with the params it had before, and every check "
           "passed", code == 0 and "] === limelight ===" in log
           and params == [{"ballPipeline": 4, "otherPipeline": 0, "proxyPipeline": 1}])
    names = [n for n, _, _, _ in rb.limelight_ball_checks(good_record())]
    expect("its thirteen ball-phase checks are in summary.md, each PASS",
           all(summary_line(text, n).startswith("- PASS") for n in names))
    expect("summary.md has the staleness, still-ball and pipeline-change lines",
           "smallest staleFrameSeconds with no stale episode: 0.4 s" in text and "| 42.4, 0.2 |" in text
           and "- pipeline change last:" in text)
    expect("the old untimed sync check is gone", "LimelightSync put the ball pipeline on the Limelight" not in text)
    expect("no prompt: nothing was read from the terminal", stdin.reads == 0)
    expect("the original config is back and no bench config is left", restored(hub))
    raw = out / "raw" / "limelight.json"
    expect("raw/limelight.json keeps the new phases", raw.exists() and "pipelineChange" in json.loads(raw.read_text())
           ["phases"])

    hub = LimelightHub(edit((P + ("stillBalls", "balls", 3, "maxDeviationIn"), 2.4))())
    code, out, _, _ = hub_run(["--limelight", "--ports-empty"], hub)
    text = (out / "summary.md").read_text()
    expect("a wandering ball on the hub: its check FAILs in summary.md and the run exits 1",
           code == 1 and summary_line(text, "max deviation under 2 in").startswith("- FAIL"))

    hub = LimelightHub(good_record(), limelight=False)
    code, out, stdin, _ = hub_run(["--limelight", "--ports-empty"], hub)
    text = (out / "summary.md").read_text()
    expect("--limelight with no Limelight plugged in: the run stops saying what to plug in, nothing INITed but the "
           "probe, the config back",
           code == 1 and "or drop --limelight" in text and restored(hub)
           and not [o for o, _ in hub.inits if o != "Bench: Probe"])

    class CtrlC(rb.Runner):
        def pull(self, name, required=True, local_name=None):
            if name == "limelight":
                raise KeyboardInterrupt
            return super().pull(name, required, local_name)

    hub = LimelightHub(good_record())
    code, out, _, raised = hub_run(["--limelight", "--ports-empty"], hub, CtrlC)
    expect("Ctrl-C while Bench: Limelight's result comes back: the runner re-raises it with the original config back "
           "and no bench config left", raised is not None and restored(hub)
           and hub.status["activeOpMode"] == rb.IDLE_OPMODE)


def selection():
    r = object.__new__(rb.Runner)
    r.args = rb.parse_args([])
    r.summary, r.log = [], (lambda text: None)
    r.expansion = r.webcam = r.limelight = None
    expect("a default run on a bare hub leaves Bench: Limelight out", "limelight" not in r.selection())


def main():
    for section in (verdict_cases, mutation_cases, on_the_fake_hub, selection):
        try:
            section()
        except Exception as e:
            expect(f"{section.__name__} ran to its end ({type(e).__name__}: {e})", False)
            traceback.print_exc()
    print(f"{passed} passed, {len(failed)} failed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
