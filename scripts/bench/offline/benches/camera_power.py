"""Bench: Camera Power Loss offline: its verdicts on synthetic records (each check fails when weakened), the
bench_biobuzz_cam config it writes, and the bench end to end on fake_hub.py with an Expansion Hub at address 2, a
webcam and a Limelight: the typed ports statement, the control INIT before any prompt, the prompts in order, the probe
with the bank out, the wait for both cameras, Bench: Full Load Soak with the pull and the replug, a control that fails,
a crash at the pull, cameras that never come back and Ctrl-C at a prompt putting the config back. No hub, no adb, no
network. Run with: python3 -I scripts/bench/offline/benches/camera_power.py
"""
import contextlib
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
from xml.etree import ElementTree

sys.dont_write_bytecode = True
HERE = pathlib.Path(__file__).resolve().parent
OFFLINE = HERE.parent
WORKTREE = OFFLINE.parents[2]
OUT = OFFLINE / "out" / "camera_power"
SOURCE = WORKTREE / "scripts/bench/benches/camera_power.py"
WEBCAM = "UC762"
LL_IP = "172.29.0.1"
PINPOINT = 'java.lang.IllegalArgumentException: Unable to find a hardware device with name "pinpoint"'
OPEN_FAILED = "java.lang.IllegalStateException: aprilTagDetector failed to open (EasyOpenCV error -1)"
WEBCAM_GONE = "java.lang.IllegalStateException: aprilTagDetector camera thread failed: device detached"

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


rb = load("run_bench", WORKTREE / "scripts/bench/run_bench.py")
fake_hub = load("fake_hub", OFFLINE / "fake_hub.py")
BENCH = rb.module_bench("camera_power")
cp = BENCH.module
KEYS = ("control", "unplugged", "fault", "crash")


def good_record():
    held = {"crashed": False, "error": "", "initSeenS": 0.8, "stopS": 0.4, "pidBefore": 4000, "pidAfter": 4000,
            "appRestarted": False, "stackTop": None, "constructor": None, "namesCamera": False}
    stopped = dict(held, crashed=True, error=OPEN_FAILED, namesCamera=True,
                   stackTop="org.firstinspires.ftc.teamcode.biobuzz.modules.vision.WebcamSession.update "
                            "(WebcamSession.java:95)")
    return {"atStart": {"webcam": WEBCAM, "limelight": {"ipAddress": LL_IP}}, "control": held,
            "unplugged": {"probe": {"webcams": [], "limelights": []}, "init": stopped},
            "back": {"both": True, "seconds": 40.0, "probes": []},
            "midrun": {"crashed": False, "error": "", "pullEnterS": 25.0, "faultSeenAfterPullEnterS": 0.4,
                       "replugEnterS": 40.0,
                       "result": {"pull": {"raiseMs": 21300.0, "msSinceLastPoll": 290, "lastPollMs": 21010.0,
                                           "lastWebcamFrameMs": 21000.0, "scorable": False, "tipped": True,
                                           "stalenessMs": 300.0},
                                  "cameras": {"ranSeconds": 95.0, "webcamBackMs": None, "limelightBackMs": 52000.0},
                                  "loop": {"beforePullMs": {"count": 900, "p95": 21.0},
                                           "afterPullMs": {"count": 2500, "p95": 24.0}}}}}


def edit(path, value, base=None):
    r = base or good_record()
    node = r
    for key in path[:-1]:
        node = node[key]
    node[path[-1]] = value
    return r


def verdicts(checks_fn, record):
    names = [c[0] for c in checks_fn(record)]
    return dict(zip(KEYS, (c[1] for c in checks_fn(record)))) if len(names) == len(KEYS) else {}


def only_fails(v, *keys):
    return bool(v) and {k for k, ok in v.items() if not ok} == set(keys)


CASES = [
    ("the expected run: all pass", good_record, lambda v: only_fails(v)),
    ("a control INIT that stops at a missing device FAILs setup",
     lambda: edit(("control", "crashed"), True, edit(("control", "error"), PINPOINT)), lambda v: only_fails(v, "control")),
    ("a control INIT the runner lost FAILs setup", lambda: edit(("control",), {"failed": "INIT timed out"}),
     lambda v: only_fails(v, "control")),
    ("a control INIT that restarted the app FAILs setup", lambda: edit(("control", "appRestarted"), True),
     lambda v: only_fails(v, "control")),
    ("phase 1 not run fails it", lambda: edit(("unplugged",), {"notRun": "the control INIT didn't hold"}),
     lambda v: only_fails(v, "unplugged")),
    ("INIT holding with the bank out, as in the control, fails phase 1",
     lambda: edit(("unplugged", "init"), good_record()["control"]), lambda v: only_fails(v, "unplugged")),
    ("the probe still finding the webcam with the bank out fails phase 1",
     lambda: edit(("unplugged", "probe"), {"webcams": [WEBCAM], "limelights": []}), lambda v: only_fails(v, "unplugged")),
    ("the probe still finding the Limelight fails phase 1",
     lambda: edit(("unplugged", "probe"), {"webcams": [], "limelights": [LL_IP]}), lambda v: only_fails(v, "unplugged")),
    ("a fault 1.4 s after the last poll FAILs",
     lambda: edit(("midrun", "result", "pull", "raiseMs"), 22410.0), lambda v: only_fails(v, "fault")),
    ("a fault exactly 1 s after the last poll passes",
     lambda: edit(("midrun", "result", "pull", "raiseMs"), 22010.0), lambda v: only_fails(v)),
    ("no Limelight fault after the pull FAILs", lambda: edit(("midrun", "result", "pull"), None),
     lambda v: only_fails(v, "fault")),
    ("a fault with no poll before it FAILs", lambda: edit(("midrun", "result", "pull", "lastPollMs"), None),
     lambda v: only_fails(v, "fault")),
    ("a crash at the pull FAILs, and with no fault before it the fault check too",
     lambda: edit(("midrun", "result", "pull"), None, edit(("midrun", "crashed"), True,
                                                          edit(("midrun", "error"), WEBCAM_GONE))),
     lambda v: only_fails(v, "fault", "crash")),
    ("a crash after the fault FAILs the crash check alone",
     lambda: edit(("midrun", "crashed"), True, edit(("midrun", "error"), WEBCAM_GONE)), lambda v: only_fails(v, "crash")),
    ("a mid-run step skipped fails both its checks",
     lambda: {**good_record(), "midrun": {"skipped": "the OpMode crashed before the pull"}},
     lambda v: only_fails(v, "fault", "crash")),
    ("no mid-run record fails both", lambda: {k: v for k, v in good_record().items() if k != "midrun"},
     lambda v: only_fails(v, "fault", "crash")),
]


def run_cases(checks_fn):
    out = []
    for label, make, predicate in CASES:
        try:
            ok = predicate(verdicts(checks_fn, make()))
        except Exception as e:
            ok = False
            label = f"{label} ({type(e).__name__}: {e})"
        out.append((label, ok))
    return out


def verdict_cases():
    for label, ok in run_cases(BENCH.checks):
        expect(f"verdict: {label}", ok)
    text = "\n".join(BENCH.summary(good_record()))
    expect("summary: the fault's 290 ms after the last poll, the loop p95 on both sides, each camera back or not",
           "290 ms after its last good poll" in text and "p95 before the pull 21 ms" in text and "after it 24 ms" in text
           and "webcam no; Limelight yes, at 52000 ms" in text and "last webcam frame -10 ms from that poll" in text)
    frames = cp.stack_frames("E AndroidRuntime: \tat org.firstinspires.ftc.teamcode.biobuzz.modules.vision."
                             "WebcamSession.<init>(WebcamSession.java:47)\nat android.os.Handler.dispatch(Handler.java:1)\n"
                             "\tat org.firstinspires.ftc.teamcode.biobuzz.modules.CellTipCamera.init(CellTipCamera.java:63)")
    expect("stack: team-code frames only, innermost first, constructors named <init>",
           frames == [("org.firstinspires.ftc.teamcode.biobuzz.modules.vision.WebcamSession", "<init>",
                       "WebcamSession.java:47"),
                      ("org.firstinspires.ftc.teamcode.biobuzz.modules.CellTipCamera", "init", "CellTipCamera.java:63")])


MUTATIONS = [
    ("the 1 s limit is raised out of reach", "FAULT_LIMIT_MS = 1000.0", "FAULT_LIMIT_MS = 1e9"),
    ("a crash passes", '    if m.get("crashed"):\n        return False, (f"crashed', '    if False:\n        return False, (f"crashed'),
    ("a control that stopped passes", '    if c.get("crashed"):\n        return False, (f"INIT stopped',
     '    if False:\n        return False, (f"INIT stopped'),
    ("INIT holding with the bank out counts", '        return False, f"INIT held {HOLD_S:g} s with both cameras',
     '        return True, f"INIT held {HOLD_S:g} s with both cameras'),
    ("a camera still found with the bank out is ignored", "    if left:\n", "    if False:\n"),
    ("no fault passes", '    if not pull:\n        return False,', '    if not pull:\n        return True,'),
]


def mutated(old, new):
    text = SOURCE.read_text()
    if text.count(old) != 1:
        return None
    OUT.mkdir(parents=True, exist_ok=True)
    folder = pathlib.Path(tempfile.mkdtemp(prefix="mutant-", dir=OUT))
    (folder / "camera_power.py").write_text(text.replace(old, new))
    (folder / "order.txt").write_text("camera_power\n")
    return rb.load_benches(folder)[0]


def mutation_cases():
    for label, old, new in MUTATIONS:
        bench = mutated(old, new)
        if bench is None:
            expect(f"weakened: {label}: the text to weaken is in camera_power.py once", False)
            continue
        caught = [case for case, ok in run_cases(bench.checks) if not ok]
        expect(f"weakened: {label}: a verdict case fails ({caught[:1]})", bool(caught))


def config_cases():
    xml = cp.camera_config(cp.shipped_xml(), WEBCAM, {"ipAddress": LL_IP, "serial": rb.LIMELIGHT_CONFIG_SERIAL})
    root = ElementTree.fromstring(xml)
    shipped = ElementTree.fromstring(cp.shipped_xml())
    names = lambda r: sorted((e.tag, e.get("name"), e.get("port")) for e in r.iter() if e.get("name"))
    added = set(names(root)) - set(names(shipped))
    expect("bench_biobuzz_cam: biobuzz.xml's devices kept, the webcam and the Limelight added",
           set(names(shipped)) <= set(names(root))
           and added == {("Webcam", rb.CELL_TIP_WEBCAM, None), ("EthernetDevice", rb.LIMELIGHT_DEVICE, "0")})
    webcam = next(root.iter("Webcam"))
    ll = next(root.iter("EthernetDevice"))
    expect("bench_biobuzz_cam: this station's webcam serial and the Limelight's address and serial",
           webcam.get("serialNumber") == WEBCAM and ll.get("ipAddress") == LL_IP
           and ll.get("serialNumber") == rb.LIMELIGHT_CONFIG_SERIAL and xml.startswith("<?xml"))
    again = cp.camera_config(xml, "OTHER", {"ipAddress": LL_IP, "serial": "x"})
    root2 = ElementTree.fromstring(again)
    expect("a config that already names them gets theirs replaced, not a second one",
           len(list(root2.iter("Webcam"))) == 1 and len(list(root2.iter("EthernetDevice"))) == 1
           and next(root2.iter("Webcam")).get("serialNumber") == "OTHER")


class FakeStdin:
    """Answers in order; each may carry the person's action, done when the prompt is answered."""

    def __init__(self, answers, hub, tty=True, interrupt_at=None):
        self.answers, self.hub, self.tty, self.reads, self.interrupt_at = list(answers), hub, tty, [], interrupt_at

    def isatty(self):
        return self.tty

    def readline(self):
        self.reads.append(len(self.hub.calls))
        if self.interrupt_at is not None and len(self.reads) == self.interrupt_at:
            raise KeyboardInterrupt
        if not self.answers:
            return ""
        answer = self.answers.pop(0)
        if isinstance(answer, tuple):
            answer, action = answer
            action()
        return answer + "\n"


class Station(fake_hub.Hub):
    """fake_hub's Control Hub with an Expansion Hub at address 2 and the webcam and Limelight on a powered USB hub
    whose bank the answers pull and replug. BioBuzz Tele's INIT and Bench: Full Load Soak follow the bank."""

    def __init__(self, control_error=None, crash_at_pull=False, never_back=False, boot_probes=1, expansion=True, **kw):
        super().__init__(everything=True, **kw)
        if not expansion:
            self.probe["hubs"] = [h for h in self.probe["hubs"] if h["parent"]]
        self.control_error, self.crash_at_pull, self.never_back = control_error, crash_at_pull, never_back
        self.boot_probes, self.bank, self.pulls = boot_probes, True, []
        self.messages, self.switched, self.ticks = [], [], 0
        self.ticker = None

    def _restart_from_prefs(self):
        super()._restart_from_prefs()
        self.switched.append(dict(self.active))

    def unplug(self):
        with self.lock:
            self.bank = False
            self.pulls.append(("out", self.status["activeOpMode"]))
            self.probe["webcams"], self.probe["limelights"] = [], []
            if self.status["activeOpMode"] == cp.SOAK:
                pull = None if self.crash_at_pull else {
                    "raiseMs": 21300.0, "msSinceLastPoll": 290, "lastPollMs": 21010.0, "lastWebcamFrameMs": 21000.0,
                    "scorable": False, "tipped": True, "stalenessMs": 300.0}
                self.soak_result(pull)
                if self.crash_at_pull:
                    self.log_line("User code threw an uncaught exception", tag="OpModeManager", level="E")
                    self.log_line("\tat org.firstinspires.ftc.teamcode.biobuzz.modules.vision.WebcamSession.update"
                                  "(WebcamSession.java:95)", tag="OpModeManager", level="E")
                    self.status["errorMessage"] = WEBCAM_GONE
                    self.set_active(rb.IDLE_OPMODE, "STOPPED")

    def replug(self):
        with self.lock:
            self.bank = True
            self.pulls.append(("in", self.status["activeOpMode"]))
            self.booting = 0 if self.never_back else self.boot_probes

    def soak_result(self, pull):
        self.bench[f"files/bench/{cp.SOAK_RESULT}.json"] = json.dumps({
            "complete": True, "pull": pull,
            "cameras": {"ranSeconds": 95.0, "webcamBackMs": None, "limelightBackMs": 52000.0},
            "loop": {"beforePullMs": {"count": 900, "p95": 21.0}, "afterPullMs": {"count": 2500, "p95": 24.0}}})

    def _bench_read(self, path):
        if path.endswith("/probe.json") and self.bank and not self.never_back and getattr(self, "booting", 0) == 0:
            self.probe["webcams"] = [{"attached": True, "serial": WEBCAM}]
            self.probe["limelights"] = [{"status": "ok", "ipAddress": LL_IP, "serial": rb.LIMELIGHT_CONFIG_SERIAL}]
        elif path.endswith("/probe.json") and self.bank and getattr(self, "booting", 0) > 0:
            self.booting -= 1
            self.probe["webcams"] = [{"attached": True, "serial": WEBCAM}]
        return super()._bench_read(path)

    def tick(self):
        while True:
            time.sleep(0.02)
            with self.lock:
                if self.status["activeOpMode"] != cp.SOAK or self.status["activeOpModeStatus"] != "RUNNING":
                    return
                self.ticks += 1
                items = [] if self.bank else [{"caption": "FAULT Limelight", "value": "no data for 0.3 s"}]
                self.dash.broadcast({"type": "RECEIVE_TELEMETRY", "telemetry": [
                    {"data": {"Bench loop": self.ticks, "BENCH DONE": "powerloss: recording until STOP"},
                     "items": items, "timestamp": self.clock, "fieldOverlay": {"ops": []}}]})

    def on_message(self, session, msg):
        kind = msg.get("type")
        with self.lock:
            if kind in ("INIT_OP_MODE", "START_OP_MODE"):
                self.messages.append((kind, msg.get("opModeName") or self.status["activeOpMode"],
                                      json.loads(self.bench.get("files/bench/params.json") or "{}"),
                                      self.active["name"], self.bank))
            if kind == "INIT_OP_MODE" and msg["opModeName"] == cp.TELE:
                self.status["errorMessage"] = ""
                self.set_active(cp.TELE, "INIT")
                error = self.control_error if self.bank else OPEN_FAILED
                if error:
                    self.log_line("User code threw an uncaught exception", tag="OpModeManager", level="E")
                    self.log_line("\tat org.firstinspires.ftc.teamcode.biobuzz.modules.vision.WebcamSession.update"
                                  "(WebcamSession.java:95)", tag="OpModeManager", level="E")
                    self.status["errorMessage"] = error
                    self.set_active(rb.IDLE_OPMODE, "STOPPED")
                return
            if kind == "INIT_OP_MODE" and msg["opModeName"] == cp.SOAK:
                self.soak_result(None)
        super().on_message(session, msg)
        if kind == "START_OP_MODE" and self.status["activeOpMode"] == cp.SOAK:
            self.ticker = threading.Thread(target=self.tick, daemon=True)
            self.ticker.start()


def station_run(argv, hub, answers=(), tty=True, interrupt_at=None):
    OUT.mkdir(parents=True, exist_ok=True)
    out = pathlib.Path(tempfile.mkdtemp(prefix="run-", dir=OUT)) / "run"
    handlers = signal.getsignal(signal.SIGINT), signal.getsignal(signal.SIGHUP)
    stdin = FakeStdin(answers, hub, tty, interrupt_at)
    saved, sys.stdin = sys.stdin, stdin
    code = None
    try:
        with hub.installed(rb), contextlib.redirect_stdout(io.StringIO()):
            hub.opmode_names.append(cp.TELE)
            try:
                code = rb.Runner(rb.parse_args(["--out", str(out)] + list(argv))).run()
            except KeyboardInterrupt:
                code = "interrupted"
    finally:
        sys.stdin = saved
        signal.signal(signal.SIGINT, handlers[0])
        signal.signal(signal.SIGHUP, handlers[1])
    return code, out, stdin


def prompts(out):
    found = []
    for line in (out / "run.log").read_text().splitlines():
        for key, text in (("unplug", cp.UNPLUG), ("replug", cp.REPLUG), ("empty", "Then type empty.")):
            if text in line:
                found.append(key)
    return found


def check_line(summary, part):
    return next((l for l in summary.splitlines() if l.startswith("- ") and part in l), "")


def restored(hub):
    return hub.active["name"] == "betta_decode" and not [n for n in hub.configs if n.startswith("bench_")]


def person(hub):
    return ["empty", ("", hub.unplug), ("", hub.replug), ("", hub.unplug), ("", hub.replug)]


ARGV = ["--camera-power", "--interactive", "--s3-ports"]


def on_the_fake_hub():
    hub = Station()
    code, out, stdin = station_run(["--camera-power", "--interactive"], hub)
    expect("without --s3-ports: refused before any adb command, naming the flag",
           code == 1 and hub.calls == [] and "rerun with --s3-ports" in (out / "summary.md").read_text())

    hub = Station()
    code, out, stdin = station_run(["--camera-power", "--s3-ports"], hub, answers=["empty"])
    summary = (out / "summary.md").read_text()
    expect("without --interactive: stops after the probe saying so, nothing pulled, nothing INITed",
           code == 1 and "so it needs --interactive" in summary and "or drop --camera-power" in summary
           and stdin.reads == [0] and not [m for m in hub.messages if m[1] != "Bench: Probe"] and restored(hub))

    hub = Station(expansion=False)
    code, out, stdin = station_run(ARGV, hub, answers=["empty"])
    expect("no Expansion Hub at address 2: stops after the probe, saying what to connect",
           "No Expansion Hub answered on RS485 at address 2" in (out / "summary.md").read_text()
           and not [m for m in hub.messages if m[1] != "Bench: Probe"] and restored(hub))

    hub = Station(boot_probes=2)
    code, out, stdin = station_run(ARGV, hub, answers=person(hub))
    summary = (out / "summary.md").read_text()
    msgs = [(k, o, p, cfg, bank) for k, o, p, cfg, bank in hub.messages]
    tele = [m for m in msgs if m[1] == cp.TELE]
    expect("the prompts in order: the typed statement, then unplug, replug, unplug, replug",
           prompts(out) == ["empty", "unplug", "replug", "unplug", "replug"] and not stdin.answers)
    expect("the control INIT comes first, on bench_biobuzz_cam with the bank in; the second with it out; neither STARTed",
           [(m[0], m[3], m[4]) for m in tele] == [("INIT_OP_MODE", cp.CONFIG_CAM, True),
                                                  ("INIT_OP_MODE", cp.CONFIG_CAM, False)]
           and msgs.index(tele[0]) < len(msgs) and hub.pulls[0][1] == rb.IDLE_OPMODE)
    probes_out = [m for m in msgs if m[1] == "Bench: Probe" and m[3] == cp.CONFIG_CAM and m[0] == "INIT_OP_MODE"]
    expect("Bench: Probe with the bank out between the two INITs, then until both cameras are back",
           len(probes_out) >= 3 and not probes_out[0][4] and all(p[4] for p in probes_out[1:]))
    soak = [m for m in msgs if m[1] == cp.SOAK]
    expect("then Bench: Full Load Soak, mode powerloss, STARTed on bench_main with the bank in",
           [(m[0], m[3], m[4], m[2].get("mode")) for m in soak] == [("INIT_OP_MODE", rb.CONFIG_MAIN, True, "powerloss"),
                                                                    ("START_OP_MODE", rb.CONFIG_MAIN, True, "powerloss")])
    expect("the mid-run pull and replug happened while Bench: Full Load Soak ran",
           hub.pulls[2:] == [("out", cp.SOAK), ("in", cp.SOAK)])
    xml = (out / "logs" / f"{cp.CONFIG_CAM}.xml").read_text()
    expect("bench_biobuzz_cam on the hub: biobuzz.xml's Expansion Hub 2, this webcam and this Limelight",
           'name="Expansion Hub 2"' in xml and f'serialNumber="{WEBCAM}"' in xml and f'ipAddress="{LL_IP}"' in xml)
    expect("every check passes, and the run ends on the original config with no bench config left",
           code == 0 and "- FAIL" not in summary and restored(hub))
    results = json.loads((out / "runner_results.json").read_text())["camera_power"]
    expect("record: the probe with the bank out found nothing, the bank-out INIT's error and stack top, both back",
           results["unplugged"]["probe"] == {"webcams": [], "limelights": []}
           and "failed to open" in results["unplugged"]["init"]["error"]
           and "WebcamSession.update" in results["unplugged"]["init"]["stackTop"]
           and results["back"]["both"] and len(results["back"]["probes"]) == 3)

    hub = Station(control_error=PINPOINT)
    code, out, stdin = station_run(ARGV, hub, answers=["empty", ("", hub.unplug), ("", hub.replug)])
    summary = (out / "summary.md").read_text()
    expect("a control INIT that fails: setup FAIL naming the device, no INIT with the bank out, straight to mid-run",
           check_line(summary, "control:").startswith("- FAIL") and "pinpoint" in check_line(summary, "control:")
           and check_line(summary, "power bank out").startswith("- FAIL")
           and len([m for m in hub.messages if m[1] == cp.TELE]) == 1 and prompts(out) == ["empty", "unplug", "replug"]
           and check_line(summary, "within 1 s").startswith("- PASS") and restored(hub))

    hub = Station(crash_at_pull=True)
    code, out, stdin = station_run(ARGV, hub, answers=person(hub))
    summary = (out / "summary.md").read_text()
    expect("a crash at the mid-run pull: the replug is still asked, the crash and fault checks FAIL with the stack top",
           prompts(out) == ["empty", "unplug", "replug", "unplug", "replug"]
           and check_line(summary, "no crash").startswith("- FAIL") and "WebcamSession.update" in check_line(summary, "no crash")
           and check_line(summary, "within 1 s").startswith("- FAIL") and restored(hub))

    hub = Station(never_back=True)
    code, out, stdin = station_run(ARGV, hub, answers=["empty", ("", hub.unplug), ("", hub.replug)])
    summary = (out / "summary.md").read_text()
    expect("cameras that never come back: the bench stops before Bench: Full Load Soak, saying what to check",
           "weren't both back" in summary and not [m for m in hub.messages if m[1] == cp.SOAK] and restored(hub))

    hub = Station()
    code, out, stdin = station_run(ARGV, hub, answers=person(hub), interrupt_at=2)
    expect("Ctrl-C at the first unplug prompt: the run stops, nothing more is INITed, the config is put back",
           code == "interrupted" and len(stdin.reads) == 2 and restored(hub)
           and len([m for m in hub.messages if m[1] == cp.TELE]) == 1)

    hub = Station()
    code, out, stdin = station_run(ARGV, hub, answers=["empty", ("", hub.unplug)])
    expect("the terminal closed at the replug prompt: the bench fails, never taken as plugged back, config put back, "
           "and the log says to plug the bank back in",
           "the terminal closed before you confirmed" in (out / "summary.md").read_text() and restored(hub)
           and not [m for m in hub.messages if m[1] == cp.SOAK] and f"!! {cp.REPLUG}" in (out / "run.log").read_text())


def declaration():
    expect("declared: flag-only, the typed S3 statement, a person with --interactive, Bench: Full Load Soak",
           not BENCH.default and BENCH.statement == "--s3-ports" and BENCH.person is not None
           and BENCH.person_flags == ("--camera-power", "--interactive", "--s3-ports")
           and BENCH.opmodes == (cp.SOAK,) and BENCH.configs == (cp.CONFIG_CAM,))
    args = rb.parse_args(next(e for e in rb.needs_a_person() if e[0] == "camera_power")[2])
    expect("its Needs a person command parses with --s3-ports and --interactive",
           args.camera_power and args.s3_ports and args.interactive)


def main():
    for section in (verdict_cases, mutation_cases, config_cases, declaration, on_the_fake_hub):
        try:
            section()
        except Exception as e:
            expect(f"{section.__name__} ran to its end ({type(e).__name__}: {e})", False)
            traceback.print_exc()
    print(f"{passed} passed, {len(failed)} failed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
