"""Bench: Config Check offline: its verdicts on synthetic results (each one fails when weakened), its source scan, and
the bench end to end on fake_hub.py with an Expansion Hub at address 2: the typed ports statement before any adb
command, the port probe gate, four shipped configs by name, seven INITs and no START. No hub, no adb, no network.
Run with: python3 -I scripts/bench/offline/benches/config_check.py
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
OUT = OFFLINE / "out" / "config_check"
SOURCE = WORKTREE / "scripts/bench/benches/config_check.py"

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
BENCH = rb.module_bench("config_check")
cc = BENCH.module
ROBOT_OPMODES = [o for o, _ in cc.PAIRS]
GOOD_PROBE = {"power": 0.1, "ms": 200, "limitAmps": 0.3, "empty": True, "servoPortsProbed": False,
              "ports": [{"channel": c, "hub": h, "idleAmps": 0.0, "drivenAmps": 0.05, "empty": True}
                        for h in (173, 2) for c in (("fl", "fr", "bl", "br") if h == 173 else ("efl", "efr", "ebl",
                                                                                              "ebr"))]}
NECTAR = 'java.lang.IllegalArgumentException: Unable to find a hardware device with name "NectarTurret" and type Servo'
NPE = "java.lang.NullPointerException: Attempt to invoke virtual method 'double Pose.getX()' on a null object reference"
WEBCAM = "java.lang.IllegalStateException: aprilTagDetector failed to open (EasyOpenCV error CAMERA_OPEN_ERROR)"
PINPOINT = "java.lang.IllegalStateException: goBILDA Pinpoint 'pinpoint' isn't answering on its I2C port"


def good_record():
    record = {"portProbe": copy.deepcopy(GOOD_PROBE), "webcams": ["UC762"], "pairs": []}
    for opmode, config in cc.PAIRS:
        xml = cc.config_xml(config)
        record["pairs"].append({"opmode": opmode, "config": config, "cameras": cc.config_webcams(xml),
                                "elsewhere": cc.elsewhere_tokens(xml) if cc.ELSEWHERE_HERE.get(opmode) == config else [],
                                "initSeenS": 0.6, "crashed": False, "error": "", "stopS": 0.4, "pidBefore": 4000,
                                "pidAfter": 4000, "appRestarted": False})
    return record


def pair(record, opmode):
    return next(p for p in record["pairs"] if p["opmode"] == opmode)


def crash(record, opmode, error, **kw):
    p = pair(record, opmode)
    p.update(crashed=True, error=error, stopS=None, initToErrorMs=450, **kw)
    return record


def verdicts(checks_fn, record):
    """{(OpMode or 'probe', 'lookups' or 'clean'): PASS/FAIL} from a checks function."""
    out = {}
    for name, ok, detail, means in checks_fn(record):
        if "port probe" in name:
            out[("probe", "")] = ok
            continue
        opmode = next(o for o in ROBOT_OPMODES if f": {o} on " in name)
        out[(opmode, "lookups" if "every hardware lookup" in name else "clean")] = ok
    return out


def on_one_hub(record):
    for row in record["portProbe"]["ports"]:
        row["hub"] = 173
    return record


def only_fails(v, *keys):
    return {k for k, ok in v.items() if not ok} == set(keys)


CASES = [
    ("every OpMode held INIT on its own config: all pass", good_record, lambda v: only_fails(v)),
    ("a missing device on its own config fails the lookup check alone",
     lambda: crash(good_record(), "Turret Aim Test", NECTAR), lambda v: only_fails(v, ("Turret Aim Test", "lookups"))),
    ("the SDK's other two missing-device messages fail it too",
     lambda: crash(crash(good_record(), "SOTM Test", 'Unable to find a hardware device with name "PollenHood"'),
                   "BioBuzz Tele", 'Unable to find a hardware device with the name "NectarShooter"'),
     lambda v: only_fails(v, ("SOTM Test", "lookups"), ("BioBuzz Tele", "lookups"))),
    ("a NullPointerException fails both checks",
     lambda: crash(good_record(), "BioBuzz Tele", NPE),
     lambda v: only_fails(v, ("BioBuzz Tele", "lookups"), ("BioBuzz Tele", "clean"))),
    ("INIT taking over 10 s to show fails the clean check",
     lambda: (pair(r := good_record(), "SOTM Test").update(initSeenS=12.0), r)[1],
     lambda v: only_fails(v, ("SOTM Test", "clean"))),
    ("STOP taking over 10 s fails the clean check",
     lambda: (pair(r := good_record(), "SOTM Test").update(stopS=10.5), r)[1],
     lambda v: only_fails(v, ("SOTM Test", "clean"))),
    ("a run the runner lost (INIT never showed) fails both",
     lambda: (pair(r := good_record(), "Cell Tip Test").update(failed="Cell Tip Test to INIT timed out"), r)[1],
     lambda v: only_fails(v, ("Cell Tip Test", "lookups"), ("Cell Tip Test", "clean"))),
    ("an app restart (pid changed) fails the clean check",
     lambda: (pair(r := good_record(), "Decode Tele").update(pidAfter=4001, appRestarted=True), r)[1],
     lambda v: only_fails(v, ("Decode Tele", "clean"))),
    ("a pid change the runner didn't flag as a restart still fails",
     lambda: (pair(r := good_record(), "Decode Tele").update(pidAfter=4001), r)[1],
     lambda v: only_fails(v, ("Decode Tele", "clean"))),
    ("an OpMode not run fails both",
     lambda: (pair(r := good_record(), "Decode Tele").update(notRun="this install doesn't ship cuttle_decode"), r)[1],
     lambda v: only_fails(v, ("Decode Tele", "lookups"), ("Decode Tele", "clean"))),
    ("the webcam failing to open with another serial attached is allowed",
     lambda: crash(good_record(), "Cell Tip Test", WEBCAM), lambda v: only_fails(v)),
    ("the webcam failing to open with the config's own serial attached fails",
     lambda: (lambda r: (r.update(webcams=["UC852"]), r)[1])(crash(good_record(), "Cell Tip Test", WEBCAM)),
     lambda v: only_fails(v, ("Cell Tip Test", "lookups"))),
    ("Decode Tele's Pinpoint fault (its config has it on the Expansion Hub) is allowed",
     lambda: crash(good_record(), "Decode Tele", PINPOINT), lambda v: only_fails(v)),
    ("the same Pinpoint error from a BioBuzz OpMode fails: the allowance is Decode Tele's",
     lambda: crash(good_record(), "BioBuzz Tele", PINPOINT), lambda v: only_fails(v, ("BioBuzz Tele", "lookups"))),
    ("a missing device is never allowed, even one Decode Tele's allowance names",
     lambda: crash(good_record(), "Decode Tele", 'Unable to find a hardware device with name "pinpoint" and type '
                                                 'SparkFunOTOS'),
     lambda v: only_fails(v, ("Decode Tele", "lookups"))),
    ("a NullPointerException naming an allowed device still fails",
     lambda: crash(good_record(), "Decode Tele", NPE + " in prism"),
     lambda v: only_fails(v, ("Decode Tele", "lookups"), ("Decode Tele", "clean"))),
    ("an error the station doesn't explain fails the lookup check",
     lambda: crash(good_record(), "Intake Transfer Shooter Test", "java.lang.IllegalStateException: boom"),
     lambda v: only_fails(v, ("Intake Transfer Shooter Test", "lookups"))),
    ("a stale error line on an OpMode that held INIT isn't its error",
     lambda: (pair(r := good_record(), "SOTM Test").update(error=NECTAR), r)[1], lambda v: only_fails(v)),
    ("a busy motor port fails the probe check",
     lambda: (lambda r: (r["portProbe"]["ports"][5].update(empty=False, drivenAmps=0.9),
                         r["portProbe"].update(empty=False), r)[2])(good_record()),
     lambda v: only_fails(v, ("probe", ""))),
    ("a probe that saw only the Control Hub fails",
     lambda: (lambda r: (r["portProbe"].update(ports=r["portProbe"]["ports"][:4]), r)[1])(good_record()),
     lambda v: only_fails(v, ("probe", ""))),
    ("eight motor ports all on one hub fail",
     lambda: on_one_hub(good_record()),
     lambda v: only_fails(v, ("probe", ""))),
    ("no probe result fails",
     lambda: (lambda r: (r.pop("portProbe"), r)[1])(good_record()), lambda v: only_fails(v, ("probe", ""))),
]


def run_cases(checks_fn):
    results = []
    for label, make, predicate in CASES:
        try:
            ok = predicate(verdicts(checks_fn, make()))
        except Exception as e:
            ok = False
            label = f"{label} ({type(e).__name__}: {e})"
        results.append((label, ok))
    return results


def verdict_cases():
    for label, ok in run_cases(BENCH.checks):
        expect(f"verdict: {label}", ok)
    record = crash(good_record(), "Turret Aim Test", NECTAR)
    text = "\n".join(BENCH.summary(record))
    expect("summary: the table names the missing device and the OpMode's config", "| Turret Aim Test | biobuzz | threw |"
           in text and "| NectarTurret |" in text)
    expect("summary: one row per OpMode, and a line per OpMode for the source scan",
           sum(f"| {o} |" in text for o in ROBOT_OPMODES) == len(ROBOT_OPMODES) and text.count("names; not in") == 7)
    names = [c[0] for c in BENCH.checks(good_record())]
    expect("checks: the probe, then two per OpMode", len(names) == 1 + 2 * len(cc.PAIRS) and len(set(names)) == len(names))


MUTATIONS = [
    ("the missing-device pattern never matches", "MISSING_DEVICE = re.compile(r'Unable to find",
     "MISSING_DEVICE = re.compile(r'NEVER Unable to find"),
    ("NullPointerExceptions aren't looked for", 'NULL_POINTER = "NullPointerException"', 'NULL_POINTER = "NoSuchText"'),
    ("the hang limit is raised out of reach", "HANG_S = 10.0", "HANG_S = 1e9"),
    ("a pid change isn't looked at", 'if pair.get("appRestarted") or pair.get("pidBefore") != pair.get("pidAfter"):',
     "if False:"),
    ("any webcam open failure is allowed", "and config_cameras[m.group(1)] not in webcams_attached:", ":"),
    ("the probe needn't cover both hubs", "and len(hubs) == PROBE_HUBS", ""),
    ("any crash passes the lookup check", "    elif pair.get(\"crashed\"):\n        lookups = (False,",
     "    elif False:\n        lookups = (False,"),
    ("a lost run passes", "    if pair.get(\"failed\"):", "    if False:"),
]


def mutated(old, new):
    text = SOURCE.read_text()
    if text.count(old) != 1:
        return None
    OUT.mkdir(parents=True, exist_ok=True)
    folder = pathlib.Path(tempfile.mkdtemp(prefix="mutant-", dir=OUT))
    (folder / "config_check.py").write_text(text.replace(old, new))
    (folder / "order.txt").write_text("config_check\n")
    return rb.load_benches(folder)[0]


def mutation_cases():
    for label, old, new in MUTATIONS:
        bench = mutated(old, new)
        if bench is None:
            expect(f"weakened: {label}: the text to weaken is still in config_check.py once", False)
            continue
        caught = [case for case, ok in run_cases(bench.checks) if not ok]
        expect(f"weakened: {label}: a verdict case fails ({caught[:1]})", bool(caught))


def source_scan():
    found = cc.desktop_lookups(WORKTREE)
    its = found.get("Intake Transfer Shooter Test") or {}
    wanted = {"nectarFlywheel", "pollenFlywheel", "nectarHood", "pollenHood", "intake", "transfer", "melonbotics",
              "blocker"}
    expect("source scan: every OpMode's file is found", all(found.get(o) for o in ROBOT_OPMODES))
    expect("source scan: Intake Transfer Shooter Test's literal lookups are found, all in its config",
           wanted <= set(its.get("literals") or []) and not wanted & set(its.get("notInConfig") or []))
    shooter = found.get("BioBuzz Shooter Test") or {}
    expect("source scan: it follows imports into the modules (Shooter's PollenHood, which biobuzz.xml lacks today)",
           "PollenHood" in (shooter.get("literals") or []) and shooter.get("files", 0) > 1)
    expect("source scan: the bench OpModes aren't followed", not any("/opmodes/test/bench/" in str(v)
                                                                   for v in found.values()))
    expect("record: Decode Tele alone gets the station's allowance, from cuttle_decode's I2C devices",
           {"pinpoint", "prism", "backleftcolor", "colorsensor"} <= set(cc.elsewhere_tokens(cc.config_xml("cuttle_decode")))
           and "imu" not in cc.elsewhere_tokens(cc.config_xml("cuttle_decode")))


class FakeStdin:
    def __init__(self, answers, hub, tty=True):
        self.answers, self.hub, self.tty, self.reads = list(answers), hub, tty, []

    def isatty(self):
        return self.tty

    def readline(self):
        self.reads.append(len(self.hub.calls))
        return self.answers.pop(0) + "\n" if self.answers else ""


class Station(fake_hub.Hub):
    """fake_hub's Control Hub with an RS485 Expansion Hub at address 2, the robot OpModes and scripted INIT errors."""

    def __init__(self, crashes=None, probe=None, expansion=True, **kw):
        super().__init__(**kw)
        if expansion:
            self.probe["hubs"].append({"parent": False, "address": 2, "firmware": "1.8.2", "responding": True})
        self.probe["webcams"] = [{"attached": True, "serial": "UC762"}]
        self.probe["shippedConfigs"] = {n: 2131820545 + i for i, n in enumerate(
            ("betta_decode", "biobuzz", "camera", "cuttle_decode", "intake_transfer_shooter"))}
        self.crashes = crashes or {}
        self.port_probe = GOOD_PROBE if probe is None else probe
        self.messages, self.switched = [], []

    def _restart_from_prefs(self):
        super()._restart_from_prefs()
        self.switched.append(dict(self.active))

    def on_message(self, session, msg):
        kind = msg.get("type")
        with self.lock:
            if kind in ("INIT_OP_MODE", "START_OP_MODE"):
                self.messages.append((kind, msg.get("opModeName") or self.status["activeOpMode"],
                                      json.loads(self.bench.get("files/bench/params.json") or "{}")))
            if kind == "INIT_OP_MODE" and msg["opModeName"] in ROBOT_OPMODES:
                name = msg["opModeName"]
                self.status["errorMessage"] = ""
                self.set_active(name, "INIT")
                if name in self.crashes:
                    self.status["errorMessage"] = self.crashes[name]
                    self.set_active(rb.IDLE_OPMODE, "STOPPED")
                return
            if kind == "START_OP_MODE" and self.status["activeOpMode"] == "Bench: Photon Stock":
                self.bench[f"files/bench/{cc.PROBE_RESULT}.json"] = json.dumps({"portProbe": self.port_probe})
        super().on_message(session, msg)


def station_run(argv, hub, answers=("empty",), tty=True):
    OUT.mkdir(parents=True, exist_ok=True)
    out = pathlib.Path(tempfile.mkdtemp(prefix="run-", dir=OUT)) / "run"
    handlers = signal.getsignal(signal.SIGINT), signal.getsignal(signal.SIGHUP)
    stdin = FakeStdin(answers, hub, tty)
    saved, sys.stdin = sys.stdin, stdin
    try:
        with hub.installed(rb), contextlib.redirect_stdout(io.StringIO()):
            hub.opmode_names += ROBOT_OPMODES
            code = rb.Runner(rb.parse_args(["--out", str(out)] + list(argv))).run()
    finally:
        sys.stdin = saved
        signal.signal(signal.SIGINT, handlers[0])
        signal.signal(signal.SIGHUP, handlers[1])
    return code, out, stdin


def check_line(summary, name_part):
    return next((l for l in summary.splitlines() if l.startswith("- ") and name_part in l), "")


def on_the_fake_hub():
    hub = Station()
    code, out, stdin = station_run(["--config-check"], hub)
    summary = (out / "summary.md").read_text()
    expect("without --s3-ports: the run stops before any adb command, saying which ports and flag",
           code == 1 and hub.calls == [] and "rerun with --s3-ports" in summary and "servo ports 0-5" in summary
           and not stdin.reads)
    hub = Station()
    code, out, stdin = station_run(["--config-check", "--s3-ports"], hub, tty=False)
    expect("--s3-ports without a terminal: refused before any adb command",
           code == 1 and hub.calls == [] and "needs a terminal" in (out / "summary.md").read_text())
    hub = Station()
    code, out, stdin = station_run(["--config-check", "--s3-ports"], hub, answers=("full",))
    expect("--s3-ports with anything but empty typed: refused before any adb command",
           code == 1 and hub.calls == [] and stdin.reads == [0] and "nothing was changed" in
           (out / "summary.md").read_text())
    hub = Station()
    code, out, stdin = station_run(["--config-check", "--s3-ports"], hub, answers=())
    expect("--s3-ports with the terminal closed before an answer: refused before any adb command",
           code == 1 and hub.calls == [])

    hub = Station(crashes={"Turret Aim Test": NECTAR, "Cell Tip Test": WEBCAM, "Decode Tele": PINPOINT})
    code, out, stdin = station_run(["--config-check", "--s3-ports"], hub)
    summary = (out / "summary.md").read_text()
    log = (out / "run.log").read_text()
    inits = [(o, p) for k, o, p in hub.messages if k == "INIT_OP_MODE"]
    robot_inits = [o for o, _ in inits if o in ROBOT_OPMODES]
    starts = [o for k, o, _ in hub.messages if k == "START_OP_MODE"]
    names = [s["name"] for s in hub.switched]
    expect("the statement was typed once, before the first adb command", stdin.reads == [0])
    expect("the bench ran, and the run ended putting the original config back with no bench config left",
           "] === config_check ===" in log and hub.active["name"] == "betta_decode"
           and not [n for n in hub.configs if n.startswith("bench_")])
    probe = next((p for o, p in inits if o == "Bench: Photon Stock"), {})
    expect("the port probe ran first: Bench: Photon Stock on bench_photon, probe only, with the Expansion Hub on RS485",
           names[names.index("bench_photon") + 1:names.index("bench_photon") + 5] == list(cc.CONFIGS)
           and probe.get("probeOnly") == 1 and probe.get("expansion") == 2 and "Bench: Photon Stock" in starts
           and 'name="efl" port="0"' in (out / "logs" / "bench_photon.xml").read_text())
    shipped = [s for s in hub.switched if s["name"] in cc.CONFIGS]
    expect("four shipped configs, each activated by name with this install's resource id",
           [s["name"] for s in shipped] == list(cc.CONFIGS)
           and all(s["location"] == "RESOURCE" and s["resourceId"] == hub.probe["shippedConfigs"][s["name"]]
                   for s in shipped))
    expect("seven INITs, in the table's order, each on its own config", robot_inits == ROBOT_OPMODES)
    expect("no robot OpMode was ever STARTed", not set(starts) & set(ROBOT_OPMODES))
    expect("then back to bench_main before the restore", names[names.index("cuttle_decode") + 1] == rb.CONFIG_MAIN)
    expect("summary: Turret Aim Test's missing NectarTurret fails its lookup check",
           check_line(summary, "Turret Aim Test on biobuzz: every hardware lookup").startswith("- FAIL")
           and "'NectarTurret' as Servo" in check_line(summary, "Turret Aim Test on biobuzz: every hardware lookup"))
    expect("summary: the webcam with another serial and Decode Tele's Pinpoint pass as allowed faults",
           check_line(summary, "Cell Tip Test on camera: every hardware lookup").startswith("- PASS")
           and check_line(summary, "Decode Tele on cuttle_decode: every hardware lookup").startswith("- PASS"))
    expect("summary: the OpModes that held INIT pass, and the probe passes",
           check_line(summary, "BioBuzz Tele on biobuzz: every hardware lookup").startswith("- PASS")
           and check_line(summary, "port probe").startswith("- PASS"))
    results = json.loads((out / "runner_results.json").read_text()).get("config_check") or {}
    turret = next((p for p in results.get("pairs", []) if p["opmode"] == "Turret Aim Test"), {})
    expect("record: per OpMode the INIT outcome, the error, the time to it and the pid before and after",
           turret.get("crashed") is True and "NectarTurret" in turret.get("error", "")
           and isinstance(turret.get("initToErrorMs"), int) and turret.get("pidBefore") == turret.get("pidAfter"))
    expect("the run exits 1 for the failed check", code == 1)

    hub = Station(probe=dict(GOOD_PROBE, empty=False, ports=[dict(GOOD_PROBE["ports"][0], empty=False,
                                                                  drivenAmps=0.8)] + GOOD_PROBE["ports"][1:]))
    code, out, stdin = station_run(["--config-check", "--s3-ports"], hub)
    summary = (out / "summary.md").read_text()
    expect("a busy port: no shipped config activated and no robot OpMode INITed, the probe check fails and says why",
           not [s for s in hub.switched if s["name"] in cc.CONFIGS]
           and not [o for k, o, _ in hub.messages if o in ROBOT_OPMODES]
           and check_line(summary, "port probe").startswith("- FAIL") and "No robot OpMode was INITed" in summary
           and hub.active["name"] == "betta_decode")

    hub = Station(expansion=False)
    code, out, stdin = station_run(["--config-check", "--s3-ports"], hub)
    summary = (out / "summary.md").read_text()
    expect("no Expansion Hub at address 2: the run stops after the probe, nothing driven, saying what to connect",
           code == 1 and "No Expansion Hub answered on RS485 at address 2" in summary and "or drop --config-check"
           in summary and "bench_photon" not in [s["name"] for s in hub.switched]
           and not [m for m in hub.messages if m[1] != "Bench: Probe"])


def declaration():
    expect("declared: flag-only, needs a person, the S3 statement, Bench: Photon Stock, bench_photon and four shipped "
           "configs", not BENCH.default and BENCH.person is not None and BENCH.statement == "--s3-ports"
           and BENCH.opmodes == ("Bench: Photon Stock",) and BENCH.configs == (rb.CONFIG_PHOTON,) + cc.CONFIGS)
    expect("its Needs a person command parses and carries --s3-ports",
           rb.parse_args(next(e for e in rb.needs_a_person() if e[0] == "config_check")[2]).s3_ports)
    expect("--s3-ports stands for --ports-empty too", rb.parse_args(["--s3-ports"]).ports_empty)


def main():
    for section in (verdict_cases, mutation_cases, source_scan, declaration, on_the_fake_hub):
        try:
            section()
        except Exception as e:
            expect(f"{section.__name__} ran to its end ({type(e).__name__}: {e})", False)
            traceback.print_exc()
    print(f"{passed} passed, {len(failed)} failed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
