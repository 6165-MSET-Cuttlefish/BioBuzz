"""Drives the adb-port preflight, --list-configs and Bench: Dashboard Off end to end with every hub call stubbed: no
adb, no socket, no network. Run with: python3 -I scripts/bench/offline/dryrun_stations.py"""
import contextlib
import importlib.util
import io
import pathlib
import signal
import subprocess
import sys
import tempfile
import time

sys.dont_write_bytecode = True
HERE = pathlib.Path(__file__).resolve().parent
WORKTREE = HERE.parents[2]
OUT = HERE / "out" / "dryrun"
spec = importlib.util.spec_from_file_location("run_bench", WORKTREE / "scripts/bench/run_bench.py")
rb = importlib.util.module_from_spec(spec)
spec.loader.exec_module(rb)

passed = 0
failed = []


def expect(label, cond):
    global passed
    if cond:
        passed += 1
    else:
        failed.append(label)
        print("FAIL", label)


def run_dir():
    OUT.mkdir(parents=True, exist_ok=True)
    return pathlib.Path(tempfile.mkdtemp(prefix="stations-", dir=OUT))


class Subprocess:
    """Stands in for subprocess.run: lsof answers from a fixture; any other command is recorded and fails."""

    def __init__(self, lsof_out):
        self.lsof_out = lsof_out
        self.calls = []

    def __call__(self, argv, **kw):
        self.calls.append(list(argv))
        if pathlib.Path(argv[0]).name == "lsof":
            return subprocess.CompletedProcess(argv, 0 if self.lsof_out else 1, self.lsof_out, "")
        return subprocess.CompletedProcess(argv, 1, "", "stubbed: no adb in the dry run")

    def adb_calls(self):
        return [c for c in self.calls if pathlib.Path(c[0]).name != "lsof"]


@contextlib.contextmanager
def stubbed(fake):
    saved = subprocess.run, rb.find_adb, rb.port_answers
    subprocess.run, rb.find_adb, rb.port_answers = fake, lambda: "/nonexistent/adb", lambda port: False
    try:
        yield
    finally:
        subprocess.run, rb.find_adb, rb.port_answers = saved


@contextlib.contextmanager
def quiet_run():
    console = io.StringIO()
    handlers = signal.getsignal(signal.SIGINT), signal.getsignal(signal.SIGHUP)
    try:
        with contextlib.redirect_stdout(console):
            yield console
    finally:
        signal.signal(signal.SIGINT, handlers[0])
        signal.signal(signal.SIGHUP, handlers[1])


# --- the preflight in Runner.connect, with subprocess stubbed
fake = Subprocess("p4242\ncssh\nf5\nf6\n")
with stubbed(fake):
    out = run_dir()
    with quiet_run():
        code = rb.Runner(rb.parse_args(["--out", str(out)])).run()
summary = (out / "summary.md").read_text()
expect("connect: an ssh listener on adb's port stops the run", code == 1 and "Run stopped early" in summary)
expect("connect: it stops before any adb command", fake.calls and not fake.adb_calls())
expect("connect: the summary names ssh and its pid", "ssh (pid 4242)" in summary)

fake = Subprocess("p45913\ncadb\nf7\n")
with stubbed(fake):
    out = run_dir()
    with quiet_run():
        code = rb.Runner(rb.parse_args(["--out", str(out), "--serial", "fake-serial"])).run()
expect("connect: with adb on the port the runner goes on to its first adb command",
       code == 1 and fake.adb_calls() and pathlib.Path(fake.calls[0][0]).name == "lsof"
       and fake.adb_calls()[0][1:] == ["devices"])
expect("connect: and stops on the stub's answer, not the preflight",
       "adb doesn't see fake-serial" in (out / "summary.md").read_text())

# --- --list-configs: only reads
OLD = ('<?xml version="1.0"?><Robot type="FirstInspires-FTC"><Webcam name="ballDetector" serialNumber="UC852" />'
       '</Robot>')


class ListAdb:
    """Has only the reads --list-configs may use: any other call is an AttributeError."""
    serial = "fake"

    def __init__(self):
        self.calls = []

    def ensure_connected(self):
        self.calls.append("ensure_connected")

    def _raw(self, args, timeout=60):
        assert args == ["version"], args
        return subprocess.CompletedProcess(args, 0, "Android Debug Bridge version 1.0.41\n", "")

    def active_config_entry(self):
        return {"name": "cuttledecode", "location": "LOCAL_STORAGE", "resourceId": 0}

    def run(self, *args, check=True, timeout=60):
        assert args == ("shell", f"ls -l {rb.CONFIG_DIR}/"), args
        return ("-rw-rw---- 1 root sdcard_rw 99 2026-09-23 08:00 cuttledecode.xml\n"
                "-rw-rw---- 1 root sdcard_rw 1234 2026-10-01 12:34 betta_decode.xml\n")

    def config_names(self):
        return {"cuttledecode", "betta_decode"}

    def read_config(self, name):
        self.calls.append(f"read {name}")
        return OLD if name == "cuttledecode" else (WORKTREE / "TeamCode/src/main/res/xml/betta_decode.xml").read_text()


fake = Subprocess("p45913\ncadb\nf7\n")
with stubbed(fake):
    out = run_dir()
    runner = rb.Runner(rb.parse_args(["--list-configs", "--out", str(out)]))
    runner.adb = ListAdb()
    with quiet_run() as console:
        code = runner.run()
summary = (out / "summary.md").read_text()
expect("list-configs: reads every config and nothing else",
       runner.adb.calls == ["ensure_connected", "read betta_decode", "read cuttledecode"])
expect("list-configs: never opens the dashboard", runner.dash is None)
expect("list-configs: lists the stale cuttledecode", "WARN cuttledecode: saved on the hub but not a config" in summary)
expect("list-configs: an active config naming ballDetector fails its check",
       code == 1 and "FAIL saved configs: the active config cuttledecode" in summary)
expect("list-configs: prints the table to the terminal", "| cuttledecode (active) | 99 | 2026-09-23 08:00 |"
       in console.getvalue())

# --- Bench: Dashboard Off
rb.DASH_OFF_ON_S, rb.DASH_OFF_GRACE_S, rb.DASH_OFF_WINDOW_S = 0.3, 0.1, 0.3
rb.DASH_OFF_DONE_WAIT_S, rb.DASH_OFF_SWITCH_WAIT_S, rb.DASH_OFF_BOOT_WAIT_S = 3.0, 2.0, 3.0
rb.DASH_OFF_POLL_S, rb.DASH_OFF_RECOVERY_S, rb.DASH_OFF_PROBE_S, rb.CONFIG_SWITCH_TIMEOUT_S = 0.02, 2.0, 0.05, 3
rb.http_request = lambda method, url, timeout=3.0: {"code": 200, "ms": 1.0}
PREFS = ("<?xml version='1.0' encoding='utf-8' standalone='yes' ?>\n<map>\n"
         '    <boolean name="autoEnable" value="{}" />\n</map>\n')


class FakeHub:
    """slothboard on a Control Hub, as the source reads: the switch saves autoEnable, a restart reads it."""

    def __init__(self, mode=None):
        self.mode = mode
        self.enabled = True
        self.auto = True
        self.active = rb.IDLE_OPMODE
        self.pid = 100
        self.up = self.app_up = True
        self.booted = time.monotonic() - 1000
        self.down_polls = 0
        self.events = []
        self.started = self.switched = self.result = self.done = None
        self.opmode_seq = 0
        self.phase = None
        self.history = []
        self.starts = []

    def listening(self):
        if not (self.up and self.app_up):
            return set()
        return {8000, 8080} | (set(rb.DASH_OFF_PROXY_PORTS) if self.enabled else set())

    def start_app(self):
        self.starts.append(self.phase)
        self.pid += 1
        self.app_up = True
        self.active = rb.IDLE_OPMODE
        self.enabled = self.auto is not False
        if self.enabled:
            self.auto = True
        self.opmode_seq += 1

    def go_down(self, polls):
        self.up = self.app_up = False
        self.down_polls = polls

    def boot(self):
        self.up = True
        self.booted = time.monotonic()
        self.start_app()

    def recover(self):
        self.history.append("recovery")
        self.phase = "recovery"
        if self.mode != "recovery fails":
            self.auto = None
        self.start_app()

    def tick(self):
        now = time.monotonic()
        if self.started is None:
            return
        if self.switched is None and now >= self.started + rb.DASH_OFF_ON_S:
            t = self.started
            while t < now:
                self.events.append((t, "RECEIVE_TELEMETRY", 2))
                self.events.append((t + 0.01, "RECEIVE_IMAGE", 0))
                t += 0.1
            self.enabled, self.auto, self.switched = False, False, now
        if self.switched is not None and self.done is None and \
                now >= self.switched + rb.DASH_OFF_GRACE_S + rb.DASH_OFF_WINDOW_S:
            if self.mode == "leaky":
                self.events.append((now - 0.01, "RECEIVE_TELEMETRY", 1))
            self.result = {"enabledAtStart": True, "enabledAfterDisable": False, "onPackets": 6, "onFramesServed": 3,
                           "offPackets": 8, "offFramesServed": 0, "disableMs": 4.0}
            self.done = "isEnabled true -> false, 0 frames served after the switch"
            self.active, self.started = rb.IDLE_OPMODE, None


class FakeDash:
    def __init__(self, hub):
        self.hub = hub
        self.status = None
        self.opmode_log = []
        names = rb.BENCH_OPMODES + ([] if hub.mode == "old install" else [rb.DASH_OFF_OPMODE])
        self.opmodes = [(n, "Test") for n in names]

    @property
    def opmode_seq(self):
        return self.hub.opmode_seq

    def _up(self):
        if not (self.hub.up and self.hub.app_up):
            raise rb.BenchError("Can't open the dashboard websocket (the fake hub is down)")

    def get_status(self, timeout=10):
        self.hub.tick()
        self._up()
        self.status = {"enabled": self.hub.enabled, "available": True, "activeOpMode": self.hub.active,
                       "activeOpModeStatus": "RUNNING" if self.hub.started else "INIT", "errorMessage": "",
                       "batteryVoltage": 12.6}
        return self.status

    def send(self, obj):
        self._up()
        kind = obj["type"]
        if not self.hub.enabled and kind != "GET_ROBOT_STATUS" and self.hub.mode != "init taken":
            return
        if kind == "INIT_OP_MODE":
            self.hub.active = obj["opModeName"]
        elif kind == "START_OP_MODE" and self.hub.active == rb.DASH_OFF_OPMODE:
            self.hub.started = time.monotonic()
        elif kind == "STOP_OP_MODE":
            self.hub.active = rb.IDLE_OPMODE

    def wait_until(self, predicate, timeout, what, poll_status_every=None, tick=None):
        deadline = time.monotonic() + timeout
        while True:
            if poll_status_every is not None:
                self.get_status()
            if predicate():
                return
            if time.monotonic() > deadline:
                raise rb.BenchError(f"timed out after {timeout:.0f} s waiting for {what}")
            time.sleep(0.02)

    def ensure_connected(self, timeout=60):
        self._up()

    def close(self):
        pass


class FakeCounter:
    def __init__(self, hub):
        self.hub = hub
        self.opened = time.monotonic()
        hub.events += [(self.opened, "RECEIVE_CONFIG", 0), (self.opened, "RECEIVE_OP_MODE_LIST", 0)]

    def events(self):
        self.hub.tick()
        return [e for e in self.hub.events if e[0] >= self.opened]

    def open_at(self, t):
        return not (self.hub.mode == "socket drops" and self.hub.phase == "reboot")

    def close(self):
        pass


class FakeAdb:
    serial = "192.168.43.1:5555"
    private = True

    def __init__(self, hub):
        self.hub = hub
        self.calls = []

    def ensure_connected(self):
        h = self.hub
        if h.up:
            return
        h.down_polls -= 1
        if h.mode == "reboot never back" or h.down_polls > 0:
            raise rb.BenchError(f"adb can't connect to {self.serial}")
        h.boot()

    def clock(self):
        return {"epochS": 0, "uptimeS": round(time.monotonic() - self.hub.booted, 3)}

    def pid(self):
        return str(self.hub.pid) if self.hub.app_up else None

    def hub_time(self):
        return "11-11 00:00:00.000"

    def mark(self, text):
        pass

    def bench_dir(self):
        return rb.PRIVATE_BENCH_DIR

    def bench_shell(self, command, check=True):
        return ""

    def push_json(self, obj, name, scratch):
        self.hub.params = obj

    def listeners(self, ports):
        return [{"port": p, "address": "any", "uid": "10000"} for p in sorted(self.hub.listening()) if p in ports]

    def logcat_matches(self, hub_time, *needles):
        return rb.ROBOT_SETUP_DONE if self.hub.up and self.hub.app_up else ""

    def run(self, *args, check=True, timeout=60):
        self.calls.append(args)
        h = self.hub
        if not h.up:
            raise rb.BenchError(f"adb {' '.join(args)} failed: device offline")
        if args == ("reboot",):
            h.phase = "reboot"
            h.go_down(60 if h.mode == "late reboot" else 3)
            return ""
        if args[0] == "pull":
            pathlib.Path(args[2]).write_bytes(b"apk")
            return ""
        if args == ("shell", f"pm path {rb.RC_PACKAGE}"):
            return f"package:/data/app/{rb.RC_PACKAGE}-1/base.apk\n"
        if args == ("shell", f"am start -n {rb.RC_LAUNCHER}"):
            if not h.app_up:
                h.start_app()
            return ""
        if args == ("shell", rb.DASHBOARD_RECOVERY_SHELL):
            h.recover()
            return ""
        if args == ("shell", f"logcat -G {rb.LOG_BUFFER}"):
            return ""
        if args == ("shell", f"ls -l {rb.CONFIG_DIR}/"):
            return "-rw-rw---- 1 root sdcard_rw 1234 2026-10-01 12:34 betta_decode.xml\n"
        raise AssertionError(f"unexpected adb {args}")

    def config_names(self):
        return {"betta_decode"}

    def read_config(self, name):
        assert name == "betta_decode", name
        return (WORKTREE / "TeamCode/src/main/res/xml/betta_decode.xml").read_text()

    def _raw(self, args, timeout=60):
        self.calls.append(tuple(args))
        h = self.hub
        if args[2:] == ["shell", f"run-as {rb.RC_PACKAGE} cat {rb.DASHBOARD_PREFS}"]:
            if h.auto is None:
                return subprocess.CompletedProcess(args, 1, "", f"cat: {rb.DASHBOARD_PREFS}: No such file or directory\n")
            return subprocess.CompletedProcess(args, 0, PREFS.format("true" if h.auto else "false"), "")
        if args[2] == "install":
            h.phase = "reinstall"
            h.app_up = False
            if h.mode == "on after reinstall":
                h.auto = None
            return subprocess.CompletedProcess(args, 0, "Performing Streamed Install\nSuccess\n", "")
        raise AssertionError(f"unexpected adb {args}")


class Dry(rb.Runner):
    def __init__(self, hub, argv=()):
        self.args = rb.parse_args(["--dashboard-off", *argv])
        self.out = run_dir()
        (self.out / "raw").mkdir()
        (self.out / "logs").mkdir()
        self.logfile = open(self.out / "run.log", "w")
        self.summary, self.checks, self.results = [], [], {}
        self.hub = hub
        self.adb = FakeAdb(hub)
        self.dash = FakeDash(hub)
        self.switch_with_adb = True
        self.limelight = None
        self.known_pid = None
        self.restarts, self.switches, self.restore_notes = [], [], []
        self.watch = None
        self.dashboard_left_off = None
        self.original_entry = {"name": "betta_decode", "location": "LOCAL_STORAGE", "resourceId": 0}
        self.original_xml = None
        self.asked = []

    def log(self, text):
        self.logfile.write(text + "\n")

    def ask(self, text):
        self.asked.append(text)
        if "PULL" in text:
            self.hub.phase = "battery"
            self.hub.go_down(3)

    def message_counter(self):
        return FakeCounter(self.hub)

    def wait_idle(self, timeout=30):
        self.dash.get_status()

    def wait_ready(self, battery=True):
        self.dash.get_status()

    def begin_watch(self, opmode, hub_t0=None):
        self.watch = {"crashSignal": None, "stopSent": False}

    def crashed(self):
        return False

    def adb_crash_tick(self):
        pass

    def user_crash(self):
        return None

    def _stop(self, opmode, outcome):
        self.hub.active = rb.IDLE_OPMODE

    def _save_log(self, relative, hub_t0):
        return ""

    def logcat_done(self, hub_t0, result_files):
        self.hub.tick()
        return self.hub.done

    def pull(self, name, required=True, local_name=None):
        return self.hub.result if name == "dashboard_off" else {}

    def active_config(self):
        return rb.CONFIG_MAIN

    def activate_config(self, name, xml_text=None, config=None, accept=None, battery=True):
        self.switches.append({"config": name, "appKilled": False, "overAdb": True})
        self.hub.phase = "restart"
        self.hub.start_app()

    def run_opmode(self, opmode, step, params=None, **kw):
        if not self.hub.enabled:
            raise rb.BenchError(f"timed out after 30 s waiting for {opmode} to INIT")
        now = time.monotonic()
        self.hub.events += [(now + i * 0.001, "RECEIVE_TELEMETRY", 1) for i in range(5)]
        if kw.get("during"):
            kw["during"]()
        return {"step": step, "opmode": opmode, "crashed": False}

    def connect(self):
        pass

    def run_probe(self):
        self.summary.append("## Probe\n")

    def selection(self):
        return {"dashboard_off"}

    def main_config(self, *args, **kw):
        return ""

    def restore_config(self):
        self.hub.history.append("restore")

    def copy_bench_folder(self):
        pass


def bench(mode=None, argv=()):
    """Runs run_dashboard_off alone; returns the runner, the error it raised, and {check name: ok}."""
    d = Dry(FakeHub(mode), argv)
    error = None
    try:
        d.run_dashboard_off()
    except rb.BenchError as e:
        error = str(e)
    return d, error, {name: state == "PASS" for name, state, _, _ in d.checks}


def only_failing(results, fragment):
    bad = [name for name, ok in results.items() if not ok]
    return bool(bad) and all(fragment in name for name in bad)


d, error, results = bench()
expect("dashboard off: a clean hub passes every check", error is None and results and all(results.values()))
expect("dashboard off: every planned check ran", len(results) == 4 + 4 * 3 + 1)
expect("dashboard off: the recovery ran its own command", ("shell", rb.DASHBOARD_RECOVERY_SHELL) in d.adb.calls)
expect("dashboard off: it reinstalled the hub's own APK",
       any(c[2:4] == ("install", "-r") for c in d.adb.calls if len(c) > 3))
expect("dashboard off: no APK copy is left in the results", not list((d.out / "logs").rglob("*.apk")))
expect("dashboard off: a clean recovery leaves nothing to print", d.dashboard_left_off is None)
expect("dashboard off: the app started again in each phase, the restart's included",
       [p for p in d.hub.starts if p in ("restart", "reboot", "reinstall")] == ["restart", "reboot", "reinstall"])
expect("dashboard off: the summary has a row per window",
       sum(line.startswith("| after ") for line in d.summary) == 3)

d, error, results = bench("old install")
expect("an install without the OpMode: stops before any adb command or check",
       error and "doesn't have Bench: Dashboard Off" in error and d.adb.calls == [] and results == {}
       and d.dashboard_left_off is None)
d, error, results = bench("leaky")
expect("leaky: telemetry after the switch fails only that check", only_failing(results, "after the switch, with"))
d, error, results = bench("on after reinstall")
expect("reinstall drops the setting: fails only the reinstall checks", only_failing(results, "reinstall"))
d, error, results = bench("init taken")
expect("INIT taken while off: fails only the INIT checks", only_failing(results, "is ignored"))
d, error, results = bench("socket drops")
expect("a socket that drops in the reboot window fails that window only", only_failing(results, "streams after an adb"))
d, error, results = bench("late reboot")
expect("a slow reboot inside the wait still passes", error is None and all(results.values()))
d, error, results = bench(argv=["--interactive"])
expect("--interactive: the battery pull is asked for and judged",
       any("PULL" in a for a in d.asked) and len(results) == 4 + 4 * 4 + 1 and all(results.values()))

d, error, results = bench("recovery fails")
expect("recovery fails: the bench raises with the exact command", error and rb.DASHBOARD_RECOVERY_SHELL in error)
expect("recovery fails: the runner keeps the command to print", d.dashboard_left_off == d.dashboard_recovery())
expect("recovery fails: only the recovery check fails", only_failing(results, "recovery"))

for mode, label in (("recovery fails", "a failed recovery"), ("reboot never back", "a hub lost in the reboot")):
    d = Dry(FakeHub(mode))
    with quiet_run() as console:
        code = d.run()
    printed = console.getvalue()
    summary = (d.out / "summary.md").read_text()
    expect(f"run, {label}: the bench's recovery came before the run's config restore",
           d.hub.history == (["recovery", "restore"] if mode == "recovery fails" else ["restore"])
           and "recovery" in d.results["dashboard_off"])
    expect(f"run, {label}: the exact recovery command is printed after the results",
           code == 1 and f"Turn it back on with:\n{d.dashboard_recovery()}\n" in printed.split("Nothing to send back")[-1])
    expect(f"run, {label}: the summary says the dashboard may still be off", "may still be off" in summary)
    expect(f"run, {label}: the default run's saved-configs step ran after the probe and judged the active config",
           0 <= summary.find("## Probe") < summary.find("## Saved hardware configs")
           and "PASS saved configs: the active config betta_decode" in summary)
    caution = "First check that `lsof -nP -iTCP:5037 -sTCP:LISTEN` shows nothing or adb"
    tail = printed.split("Nothing to send back")[-1]
    expect(f"run, {label}: the port check is asked for before the printed command, and in the summary",
           0 <= tail.find(caution) < tail.find(d.dashboard_recovery()) and caution in summary)
d = Dry(FakeHub("reboot never back"))
with quiet_run():
    d.run()
rec = d.results["dashboard_off"]
expect("lost hub: the phases after it are reported as not run",
       any(state == "FAIL" and "not run" in detail and "reinstall" in name for name, state, detail, _ in d.checks))
expect("lost hub: the recovery's failure is recorded", "adb can't connect" in rec["recovery"].get("error", ""))

print(f"{passed} passed, {len(failed)} failed")
sys.exit(1 if failed else 0)
