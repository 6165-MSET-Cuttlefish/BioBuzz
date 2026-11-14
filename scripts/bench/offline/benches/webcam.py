"""Bench: Webcam's clamped and rejected settings offline: the clamp and reject verdicts on synthetic update() traces
(one write passes, a retry fails, no refused value says the fault path is unexercised; each check fails when
weakened), then run_webcam end to end on fake_hub.py with a webcam: its checks and summary from a served result, the
same params as before, no prompt, the skip without a webcam and Ctrl-C putting the config back. No hub, no adb, no
network. Run with: python3 -I scripts/bench/offline/benches/webcam.py
"""
import contextlib
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
OUT = OFFLINE / "out" / "webcam"
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
FAULT = "aprilTagDetector rejected "


def hold(control, shown, writes=((0, 83.0),), rejected=None, fault=None, faults=(), loops=480, **over):
    h = {"control": control, "shownBefore": None, "rejectedBefore": None, "faultBefore": None, "loops": loops,
         "readMs": {"count": loops, "p50": 0.2, "p95": 0.4, "max": 84.0}, "blockingWrites": [list(w) for w in writes],
         "shown": [], "shownAtEnd": shown, "rejected": [], "rejectedAtEnd": rejected, "fault": [list(f) for f in faults],
         "faultAtEnd": fault}
    h.update(over)
    return h


def good_record():
    """The OV9782 as the SDK source suggests: exposure's whole-ms min is 0, which the SDK refuses before sending."""
    exposure0 = "exposure 0 ms"
    return {"phases": {
        "clamp": {"opened": hold(None, None, writes=((0, 81.0),)), "holds": [
            hold("gain", "100 (asked 150)", asked=150, min=0, max=100, clampExpected=True,
                 restore=hold("gain", "0")),
            hold("exposure", "0 ms REJECTED", writes=(), rejected=exposure0, fault=FAULT + exposure0,
                 faults=((0, 0.3, FAULT + exposure0),), asked=0, min=0, max=1000, clampExpected=False,
                 restore=hold("exposure", "8 ms", faults=((0, 84.0, None),))),
            hold("white balance", "6500 K (asked 20000 K)", asked=20000, min=2800, max=6500, clampExpected=True,
                 restore=hold("white balance", "3250 K"))]},
        "rejectFind": {
            "exposure": {"min": 0, "max": 1000, "tried": 150, "inRange": 1001, "refused": [[0, 0.1]],
                         "firstAccepted": 1, "setMs": {"count": 150, "p50": 80.0, "p95": 88.0, "max": 90.0}},
            "gain": {"min": 0, "max": 100, "tried": 101, "inRange": 101, "refused": [], "firstAccepted": 0,
                     "setMs": {"count": 101, "p50": 80.0, "p95": 88.0, "max": 90.0}},
            "white balance": {"min": 2800, "max": 6500, "tried": 150, "inRange": 3701, "refused": [],
                              "firstAccepted": 2800, "setMs": {"count": 150, "p50": 80.0, "p95": 88.0, "max": 90.0}}},
        "reject": {"manualOff": hold(None, None, writes=((0, 160.0),)),
                   "manualOn": hold(None, None, writes=((0, 400.0),)),
                   "holds": [{"control": "exposure", "value": 0, "valid": 8, "min": 0, "max": 1000,
                              "before": hold("exposure", "8 ms", writes=()),
                              "held": hold("exposure", "0 ms REJECTED", writes=(), rejected="exposure 0 ms",
                                           fault=FAULT + "exposure 0 ms",
                                           faults=((0, 0.3, FAULT + "exposure 0 ms"),)),
                              "restored": hold("exposure", "8 ms", faults=((0, 84.0, None),))}]}}}


def camera_refusal():
    r = good_record()
    r["phases"]["rejectFind"]["gain"]["refused"] = [[33, 6.0]]
    r["phases"]["reject"]["holds"].append({
        "control": "gain", "value": 33, "valid": 0, "min": 0, "max": 100,
        "before": hold("gain", "0", writes=()),
        "held": hold("gain", "33 REJECTED", writes=((0, 79.0),), rejected="gain 33", fault=FAULT + "gain 33",
                     faults=((0, 79.2, FAULT + "gain 33"),)),
        "restored": hold("gain", "0", faults=((0, 80.0, None),))})
    return r


def nothing_refused():
    r = good_record()
    r["phases"]["rejectFind"]["exposure"].update(min=1, refused=[])
    r["phases"]["clamp"]["holds"][1] = hold("exposure", "1 ms (asked 0 ms)", asked=0, min=1, max=1000,
                                            clampExpected=True, restore=hold("exposure", "8 ms"))
    r["phases"]["reject"]["holds"] = []
    return r


def edit(*changes, base=good_record):
    def make():
        r = base()
        for path, value in changes:
            node = r
            for p in path[:-1]:
                node = node[p]
            if value is DROP:
                node.pop(path[-1], None)
            else:
                node[path[-1]] = value
        return r
    return make


DROP = object()
P = ("phases",)
CL = P + ("clamp", "holds")
RJ = P + ("reject", "holds", 0)
KEYS = (("shows '(asked X)'", "asked"), ("one blocking write per held value", "write"),
        ("Webcam fault came", "fault"), ("shows REJECTED", "shown"), ("at most one blocking write", "retry"),
        ("fault cleared", "clear"))


def fails(record):
    out = set()
    for name, ok, _, _ in rb.webcam_rejected_checks(record):
        key = next(k for frag, k in KEYS if frag in name)
        if not ok:
            out.add(key)
    return out


CASES = [
    ("the SDK refusing exposure 0: every check passes", good_record, set()),
    ("a camera refusal held once, with its one write: every check passes", camera_refusal, set()),
    ("nothing refused: the two clamp checks pass and no reject check runs", nothing_refused, set()),
    ("a clamped value shown without '(asked X)' fails that check alone", edit((CL + (0, "shownAtEnd"), "100")),
     {"asked"}),
    ("no value outside the camera's range fails it: the clamp went unexercised",
     edit(*[(CL + (i, "clampExpected"), False) for i in range(3)]), {"asked"}),
    ("a held value written again (a retry) fails the write check alone",
     edit((CL + (0, "blockingWrites"), [[0, 84.0], [37, 83.0]])), {"write"}),
    ("an accepted value never written fails it", edit((CL + (2, "blockingWrites"), [])), {"write"}),
    ("its one write a loop late fails it", edit((CL + (0, "blockingWrites"), [[1, 84.0]])), {"write"}),
    ("no write and only a refused mode, not the value, fails it",
     edit((CL + (1, "rejectedAtEnd"), "exposure mode MANUAL")), {"write"}),
    ("a hold that never looped fails it", edit((CL + (0, "loops"), 0)), {"write"}),
    ("a missing hold fails it", lambda: (lambda r: (r["phases"]["clamp"]["holds"].pop(), r)[1])(good_record()),
     {"write"}),
    ("no clamp phase fails both clamp checks", edit((P + ("clamp",), DROP)), {"asked", "write"}),
    ("the fault two loops late fails the fault check alone",
     edit((RJ + ("held", "fault"), [[2, 11.0, FAULT + "exposure 0 ms"]])), {"fault"}),
    ("a fault that doesn't name the setting fails it",
     edit((RJ + ("held", "fault"), [[0, 0.3, FAULT + "gain 5"]])), {"fault"}),
    ("a fault already up before the hold fails it", edit((RJ + ("before", "faultAtEnd"), FAULT + "gain 5")),
     {"fault"}),
    ("no fault at all fails it and the clear check", edit((RJ + ("held", "fault"), []), (RJ + ("held", "faultAtEnd"),
                                                                                         None)), {"fault", "clear"}),
    ("no REJECTED shown fails that check alone", edit((RJ + ("held", "shownAtEnd"), "0 ms")), {"shown"}),
    ("a refused value resent every loop (~80 ms each) fails the retry check alone",
     edit((RJ + ("held", "blockingWrites"), [[0, 80.0], [1, 81.0], [2, 80.0]])), {"retry"}),
    ("one write but late fails it", edit((RJ + ("held", "blockingWrites"), [[3, 80.0]])), {"retry"}),
    ("a held refused value that never looped fails it", edit((RJ + ("held", "loops"), 1)), {"retry"}),
    ("a fault still up after the valid value fails the clear check alone",
     edit((RJ + ("restored", "fault"), []), (RJ + ("restored", "faultAtEnd"), FAULT + "exposure 0 ms")), {"clear"}),
    ("a fault cleared and raised again fails it",
     edit((RJ + ("restored", "fault"), [[0, 84.0, None], [9, 120.0, FAULT + "exposure 8 ms"]]),
          (RJ + ("restored", "faultAtEnd"), FAULT + "exposure 8 ms")), {"clear"}),
    ("no result at all fails the clamp checks", lambda: None, {"asked", "write"}),
]


def run_cases(checks_fn):
    saved = rb.webcam_rejected_checks
    rb.webcam_rejected_checks = checks_fn
    results = []
    try:
        for label, make, want in CASES:
            try:
                got = fails(make())
                ok = got == want
                if not ok:
                    label = f"{label} (failed {sorted(got)}, wanted {sorted(want)})"
            except Exception as e:
                ok = False
                label = f"{label} ({type(e).__name__}: {e})"
            results.append((label, ok))
    finally:
        rb.webcam_rejected_checks = saved
    return results


def verdict_cases():
    for label, ok in run_cases(rb.webcam_rejected_checks):
        expect(f"verdict: {label}", ok)
    expect("checks: two clamp checks, and four per held refused value",
           len(rb.webcam_rejected_checks(good_record())) == 6 and len(rb.webcam_rejected_checks(camera_refusal())) == 10
           and len(rb.webcam_rejected_checks(nothing_refused())) == 2)
    expect("the SDK's own refusals: exposure 0 and below, gain below 0, white balance outside the camera's range",
           rb.sdk_refuses("exposure", 0, 0, 1000) and not rb.sdk_refuses("exposure", 1, 0, 1000)
           and rb.sdk_refuses("gain", -1, 0, 100) and not rb.sdk_refuses("gain", 0, 0, 100)
           and not rb.sdk_refuses("gain", 33, 0, 100) and rb.sdk_refuses("white balance", 2799, 2800, 6500)
           and not rb.sdk_refuses("white balance", 6500, 2800, 6500))
    text = "\n".join(rb.webcam_rejected_summary(good_record()))
    expect("summary, the SDK's refusal only: the camera accepted everything, the question stays partly answered",
           "only the SDK refused exposure 0" in text and "partly answered" in text and "camera itself refused" not in text)
    text = "\n".join(rb.webcam_rejected_summary(camera_refusal()))
    expect("summary, a camera refusal: named as the camera's, not partly",
           "the camera itself refused gain 33" in text and "partly answered" not in text)
    text = "\n".join(rb.webcam_rejected_summary(nothing_refused()))
    expect("summary, nothing refused: the fault path is unexercised on hardware and the question stays partly answered",
           "fault path is unexercised on hardware" in text and "partly answered" in text)
    text = "\n".join(rb.webcam_rejected_summary(good_record()))
    expect("summary: each clamp hold, each control's sweep, the manual toggle and each reject hold",
           text.count("- clamp: ") == 3 and text.count("- rejectFind ") == 3 and "reject, manualOn" in text
           and "- reject: exposure 0:" in text and "1000 values" not in text and "150 of 1001 values" in text)
    expect("summary: an empty record gives no lines and no exception", rb.webcam_rejected_summary(None) == [])


MUTATIONS = [
    ("nothing expected to clamp passes", "bool(expected) and not unshown,", "not unshown,"),
    ("'(asked X)' isn't looked for", "f\"(asked {h.get('asked')}\" not in", "\"\" not in"),
    ("a retried write passes", "[w[0] for w in writes] == [0]", "bool(writes)"),
    ("a late write passes", "[w[0] for w in writes] == [0]", "len(writes) == 1"),
    ("a missing write passes", "or (not writes and named))", "or not writes)"),
    ("any rejected text excuses a missing write",
     "named = re.search(rf\"{re.escape(str(h.get('control')))} -?\\d\", str(h.get(\"rejectedAtEnd\") or \"\")) is not None",
     "named = bool(h.get(\"rejectedAtEnd\"))"),
    ("a hold that never looped passes the write check", "return (h.get(\"loops\") or 0) > 1 and (", "return ("),
    ("a missing hold passes", "len(holds) == len(WEBCAM_CONTROLS) and all(one_write(h) for h in holds)",
     "all(one_write(h) for h in holds)"),
    ("a fault 5 loops late passes", "raised[0] <= 1", "raised[0] <= 5"),
    ("the fault needn't name the setting", "\n                    and f\"{control} {value}\" in str(raised[2]),", ","),
    ("a fault up before the hold passes", "before.get(\"faultAtEnd\") is None and raised is not None",
     "raised is not None"),
    ("REJECTED isn't looked for", "\"REJECTED\" in str(held.get(\"shownAtEnd\") or \"\")", "True"),
    ("retries pass", "and [w[0] for w in writes] in ([], [0]),", "and len(writes) <= 5,"),
    ("a late write of the refused value passes", "[w[0] for w in writes] in ([], [0]),", "len(writes) <= 1,"),
    ("a hold that never looped passes the retry check",
     "(held.get(\"loops\") or 0) > 1 and [w[0] for w in writes] in ([], [0])", "[w[0] for w in writes] in ([], [0])"),
    ("a fault that never clears passes",
     "raised is not None and cleared is not None and restored.get(\"faultAtEnd\") is None,", "raised is not None,"),
]


def block():
    text = RUNNER.read_text()
    start = text.index("WEBCAM_CONTROLS = ")
    return text[start:text.index("\ndef counted(summary):", start)]


def mutation_cases():
    source = block()
    for label, old, new in MUTATIONS:
        if source.count(old) != 1:
            expect(f"weakened: {label}: the text to weaken is in run_bench.py's webcam block once", False)
            continue
        ns = dict(vars(rb))
        exec(compile(source.replace(old, new), "mutant", "exec"), ns)
        caught = [case for case, ok in run_cases(ns["webcam_rejected_checks"]) if not ok]
        expect(f"weakened: {label}: a verdict case fails ({caught[:1]})", bool(caught))


class FakeStdin:
    def __init__(self):
        self.reads = 0

    def isatty(self):
        return True

    def readline(self):
        self.reads += 1
        return "\n"


class WebcamHub(fake_hub.Hub):
    """fake_hub's Control Hub with a webcam that serves `record` as Bench: Webcam's result at START."""

    def __init__(self, record, webcam=True, **kw):
        super().__init__(**kw)
        if webcam:
            self.probe["webcams"] = [{"attached": True, "serial": "UC762"}]
        self.record = record
        self.inits = []

    def on_message(self, session, msg):
        with self.lock:
            if msg.get("type") == "INIT_OP_MODE":
                self.inits.append((msg["opModeName"], json.loads(self.bench.get("files/bench/params.json") or "{}")))
            if msg.get("type") == "START_OP_MODE" and self.status["activeOpMode"] == "Bench: Webcam":
                self.bench["files/bench/webcam.json"] = json.dumps(self.record)
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
    hub = WebcamHub(camera_refusal())
    code, out, stdin, _ = hub_run(["--webcam", "--ports-empty"], hub)
    text = (out / "summary.md").read_text()
    log = (out / "run.log").read_text()
    params = [p for o, p in hub.inits if o == "Bench: Webcam"]
    expect("--webcam with a webcam: Bench: Webcam ran once, with the params it had before",
           "] === webcam ===" in log and params == [{"subPhaseSeconds": 5}])
    names = [n for n, _, _, _ in rb.webcam_rejected_checks(camera_refusal())]
    expect("its clamp and reject checks are in summary.md, each PASS",
           len(names) == 10 and all(summary_line(text, n).startswith("- PASS") for n in names))
    expect("summary.md names the camera's refusal and each hold", "the camera itself refused gain 33" in text
           and "- clamp: gain 150:" in text and "- reject: gain 33:" in text)
    expect("no prompt: nothing was read from the terminal", stdin.reads == 0)
    expect("the original config is back and no bench config is left", restored(hub))

    hub = WebcamHub(edit((P + ("reject", "holds", 0, "held", "blockingWrites"), [[0, 80.0], [1, 80.0]]))())
    code, out, _, _ = hub_run(["--webcam", "--ports-empty"], hub)
    text = (out / "summary.md").read_text()
    expect("a retried refused value on the hub: its check FAILs in summary.md and the run exits 1",
           code == 1 and summary_line(text, "exposure 0: at most one blocking write").startswith("- FAIL"))

    hub = WebcamHub(good_record(), webcam=False)
    code, out, _, _ = hub_run(["--webcam", "--ports-empty"], hub)
    text = (out / "summary.md").read_text()
    expect("--webcam with no webcam plugged in: the run stops saying what to plug in, nothing INITed but the probe, "
           "the config back", code == 1 and "or drop --webcam" in text and restored(hub)
           and not [o for o, _ in hub.inits if o != "Bench: Probe"])

    class CtrlC(rb.Runner):
        def pull(self, name, required=True, local_name=None):
            if name == "webcam":
                raise KeyboardInterrupt
            return super().pull(name, required, local_name)

    hub = WebcamHub(good_record())
    code, out, _, raised = hub_run(["--webcam", "--ports-empty"], hub, CtrlC)
    expect("Ctrl-C while Bench: Webcam's result comes back: the runner re-raises it with the original config back and "
           "no bench config left", raised is not None and restored(hub) and hub.status["activeOpMode"] == rb.IDLE_OPMODE)


def selection():
    r = object.__new__(rb.Runner)
    r.args = rb.parse_args([])
    r.summary, r.log = [], (lambda text: None)
    r.expansion = r.webcam = r.limelight = None
    expect("a default run on a bare hub leaves Bench: Webcam out", "webcam" not in r.selection())


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
