"""session.py against fake_hub.py's Control Hub with every stage's runner stubbed (no hub, no adb, no network; about
1 min): the plan for a bare hub, an Expansion Hub on USB, one on RS485 and every optional device; every question before
the first stage and none after; a failing stage, a stage that raises and a hub reboot don't stop the chain, another
hub, a hijacked adb port and a statement that doesn't cover a stage do; Ctrl-C during each stage still puts the hub
back; every prompted bench in "Needs a person" with a command run_bench.py's parser takes; bare_hub.sh --install-only;
run_bench.py --probe-only and --photon-soak-hours. Run with: python3 -I scripts/bench/offline/test_session.py
"""
import contextlib
import importlib.util
import io
import json
import os
import pathlib
import re
import shlex
import signal
import subprocess
import sys
import tempfile
import threading
import time
import traceback

sys.dont_write_bytecode = True
HERE = pathlib.Path(__file__).resolve().parent
WORKTREE = HERE.parents[2]
OUT = HERE / "out" / "session"
USB_EH = "DQ2AB3CD"

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


fake_hub = load("fake_hub", HERE / "fake_hub.py")
S = load("session", WORKTREE / "scripts/bench/session.py")
rb = S.rb

BARE = {"imuType": "BHI260", "webcam": None, "limelight": None, "expansion": None, "usbRevHubs": []}


class FakeStdin:
    """The terminal: answers given in order; every read records whether a stage had started by then."""

    def __init__(self, answers, tty=True):
        self.answers, self.tty = list(answers), tty
        self.reads = []
        self.started = False

    def isatty(self):
        return self.tty

    def readline(self):
        self.reads.append(self.started)
        return self.answers.pop(0) + "\n" if self.answers else ""


class FakeProc:
    def __init__(self, lines=(), code=0, on_wait=None):
        self.stdout = io.StringIO("".join(f"{line}\n" for line in lines))
        self.code, self.on_wait = code, on_wait
        self.returncode, self.pid, self.waits = None, 4242, 0

    def wait(self, timeout=None):
        self.waits += 1
        if self.on_wait is not None:
            self.on_wait(self)
        self.returncode = self.code
        return self.code

    def send_signal(self, sig):
        self.signals = getattr(self, "signals", []) + [sig]

    def kill(self):
        pass


def stage_key(argv):
    if "--probe-only" in argv:
        return "restore" if str(argv[argv.index("--out") + 1]).endswith("99-restore") else "probe"
    for flag, key in (("--photon-soak", "photon_soak"), ("--photon", "photon"), ("--dashboard-off", "dashboard_off")):
        if flag in argv:
            return key
    return "default"


def write_stage(folder, key, hardware, fails=0, verdict="PASSED", soak_rate=900.0, stopped=None):
    folder.mkdir(parents=True, exist_ok=True)
    lines = [f"# Bench results {folder.name}\n", "## Checks\n"]
    lines += [f"- PASS check {i}: fine" for i in range(5)]
    lines += [f"- FAIL broken {i}: detail {i} (FAIL means: something)" for i in range(fails)]
    if stopped:
        lines.insert(1, f"**Run stopped early: {stopped}**\n")
    (folder / "summary.md").write_text("\n".join(lines) + "\n")
    results = {"link": {}}
    if key == "probe":
        results["hardware"] = hardware
    if key.startswith("photon"):
        results["photon"] = {"verdict": {"level": verdict},
                             "main": {"soak": {"seconds": 60.0, "issuedWrites": {"controlHub": soak_rate * 60,
                                                                                 "expansionHub": soak_rate * 60}}}}
    (folder / "runner_results.json").write_text(json.dumps(results))


class Stub:
    """Stands in for every program session.py starts. A stage's behaviour comes from `on[key]`, a function of
    (stub, argv, folder) returning a FakeProc, else a clean run."""

    def __init__(self, hub, stdin, hardware, on=None, gradle=(0, 0)):
        self.hub, self.stdin, self.hardware, self.on = hub, stdin, hardware, on or {}
        self.gradle = list(gradle)
        self.spawned, self.events = [], []

    def popen(self, argv, **kw):
        argv = [str(a) for a in argv]
        self.spawned.append((argv, dict(kw, sigint=signal.getsignal(signal.SIGINT))))
        if argv[0] == "caffeinate":
            return FakeProc()
        if argv[0] == "./gradlew":
            return FakeProc(["gradle"], self.gradle.pop(0) if self.gradle else 0)
        if argv[:2] == ["/bin/bash", str(S.BARE_HUB)]:
            folder = S.REPO_ROOT / argv[3]
            folder.mkdir(parents=True, exist_ok=True)
            reinstall = folder / "REINSTALL.sh"
            (folder / "reinstall_path.txt").write_text(f"{reinstall}\n")
            (folder / "installed.json").write_text(json.dumps({
                "serial": self.hub.serial, "backup": str(folder), "reinstall": str(reinstall),
                "originalEntry": json.dumps(dict(self.hub.active)), "runnerFlags": ["--gradle-offline"]}))
            return FakeProc(["installed"])
        if argv[0] == "/bin/bash":
            self.events.append(("reinstall", len(self.hub.calls)))
            return FakeProc(["reinstalled"])
        key = stage_key(argv)
        self.stdin.started = True
        self.events.append(("spawn " + key, len(self.hub.calls)))
        folder = pathlib.Path(argv[argv.index("--out") + 1])
        if key in self.on:
            return self.on[key](self, argv, folder)
        if key == "restore":
            self.hub.active = {"name": "betta_decode", "location": "LOCAL_STORAGE", "resourceId": 0, "isDirty": False}
            for name in [n for n in self.hub.configs if n.startswith("bench_")]:
                del self.hub.configs[name]
        write_stage(folder, "probe" if key == "restore" else key, self.hardware)
        return FakeProc([f"runner {key}"])

    def keys(self):
        return [stage_key(argv) for argv, _ in self.spawned if argv[0] == sys.executable]

    def argv(self, key):
        return next(argv for argv, _ in self.spawned if argv[0] == sys.executable and stage_key(argv) == key)


def run_session(answers, hardware, argv=("--usb",), on=None, hub=None, setup=None, gradle=(0, 0)):
    hub = hub or fake_hub.Hub(usb="--usb" in argv)
    if setup:
        setup(hub)
    OUT.mkdir(parents=True, exist_ok=True)
    folder = pathlib.Path(tempfile.mkdtemp(prefix="s-", dir=OUT)) / "session"
    stdin = FakeStdin(answers)
    console = io.StringIO()
    s = S.Session(S.parse_args([*argv, "--out", str(folder)]), stdin=stdin, out=console)
    stub = Stub(hub, stdin, hardware, on, gradle)
    s.popen = stub.popen
    handlers = signal.getsignal(signal.SIGINT), signal.getsignal(signal.SIGHUP)
    with hub.installed(rb):
        code = s.run()
    expect(f"{folder.parent.name}: the session put back the terminal's own Ctrl-C and hangup handling",
           (signal.getsignal(signal.SIGINT), signal.getsignal(signal.SIGHUP)) == handlers)
    if "--usb" in argv:
        expect(f"{folder.parent.name}: no adb connect or disconnect over USB",
               not [c for c in hub.calls if c.split()[1:2] in (["connect"], ["disconnect"])])
    return s, stub, hub, code, console.getvalue(), stdin


def index(s):
    return (s.folder / "summary.md").read_text()


def no_reads_after_start(stdin):
    return stdin.started and True not in stdin.reads and not stdin.answers


def every_child_without_terminal(stub):
    return all(kw.get("stdin") == subprocess.DEVNULL for _, kw in stub.spawned)


def plans():
    s, stub, hub, code, console, stdin = run_session(["n", "empty"], BARE)
    plan = console.split("=== Plan", 1)[-1]
    expect("bare hub: it asks only whether an Expansion Hub is on RS485, then the ports statement",
           no_reads_after_start(stdin) and len(stdin.reads) == 2)
    expect("bare hub: the statement names the Control Hub's bench ports alone",
           'Type "empty" once nothing is plugged into Control Hub motor ports 0-3 and servo ports 0-1:' in console
           and "Expansion Hub's motor ports" not in console.split("=== Plan")[0])
    expect("bare hub: the plan is the default suite, then Bench: Dashboard Off, with durations",
           "1. Default suite: about 45-90 min" in plan and "2. Bench: Dashboard Off: about 10 min" in plan
           and "3." not in plan.split("In all")[0] and "In all about" in plan)
    expect("bare hub: the plan says why the Photon block doesn't run",
           "not run: Photon block: no Expansion Hub" in plan)
    expect("bare hub: the stages ran in order (probe, default, Dashboard Off) and the session ended clean",
           stub.keys() == ["probe", "default", "dashboard_off"] and code == 0)
    default = stub.argv("default")
    expect("bare hub: the default suite's runner gets no bench flag, --ports-empty, the cable's serial and its own "
           "folder", "--ports-empty" in default and default[default.index("--serial") + 1] == fake_hub.USB_SERIAL
           and not [f for f in default if f in ("--photon", "--dashboard-off", "--interactive", "--all", "--turret-zero")]
           and default[default.index("--out") + 1].endswith("01-default"))
    expect("bare hub: every stage's runner gets the hub's ro.serialno (--hub-serialno), which run_bench.py parses",
           all(argv[argv.index("--hub-serialno") + 1] == "FAKEHUB0001" for argv, _ in stub.spawned
               if argv[0] == sys.executable)
           and rb.parse_args(default[2:]).hub_serialno == "FAKEHUB0001")
    expect("bare hub: the plan came before the first stage after the probe",
           console.index("=== Plan") < console.index("=== Default suite ("))
    expect("every child ran with no terminal input (stdin /dev/null)", every_child_without_terminal(stub))
    expect("the Mac stays awake: caffeinate -dims -w the session's pid, in its own session",
           any(a[:4] == ["caffeinate", "-dims", "-w", str(os.getpid())] and kw.get("start_new_session")
               for a, kw in stub.spawned))
    expect("the console is in the session folder", "=== Plan" in (s.folder / "console.log").read_text()
           and ">>> empty" in (s.folder / "console.log").read_text())
    text = index(s)
    expect("the index lists every stage with its verdict line and its summary's path",
           all(f"| {n} |" in text for n in ("00-probe", "01-default", "02-dashboard_off"))
           and "5 checks passed; runner exit 0" in text and "01-default/summary.md" in text)

    hw = dict(BARE, usbRevHubs=[USB_EH])
    s, stub, hub, code, console, stdin = run_session(["c", "n", "empty"], hw,
                                                     setup=lambda h: setattr(h, "usb_rev_hubs", [USB_EH]))
    plan = console.split("=== Plan", 1)[-1]
    expect("USB Expansion Hub: no RS485 or go-run question; Pinpoint, OctoQuad, then the statement",
           len(stdin.reads) == 3 and no_reads_after_start(stdin) and "RS485 port" not in console.split("=== Plan")[0])
    expect("USB Expansion Hub: the statement names both hubs' bench ports",
           "Control Hub motor ports 0-3 and servo ports 0-1 and the Expansion Hub's motor ports 0-3 and servo ports 0-1:"
           in console)
    expect("USB Expansion Hub: the plan is default, the Photon block on USB without I2C, then Dashboard Off",
           "1. Default suite" in plan and "2. Photon block (usb wiring): about 53 min" in plan
           and "3. Bench: Dashboard Off" in plan)
    expect("USB Expansion Hub: the plan says why the I2C reads and the go run don't run",
           "not run: the Photon block's I2C reads (--photon-i2c): the OctoQuad isn't in Control Hub I2C port 2" in plan
           and "not run: the Photon go run's long soak: on USB wiring the go run needs its cable pulls" in plan)
    photon = stub.argv("photon")
    expect("USB Expansion Hub: the Photon runner gets --photon --usb-expansion, no --photon-i2c, no prompt flag",
           "--usb-expansion" in photon and "--photon-i2c" not in photon and "--interactive" not in photon
           and stub.keys() == ["probe", "default", "photon", "dashboard_off"])
    expect("USB Expansion Hub: Needs a person names the I2C run that was left out",
           "(photon_i2c)" in index(s) and "--photon --usb-expansion --photon-i2c --ports-empty" in index(s))

    hw = {"imuType": "BHI260", "webcam": "UC762", "limelight": {"ipAddress": "172.29.0.1"}, "expansion": None,
          "usbRevHubs": [USB_EH]}
    s, stub, hub, code, console, stdin = run_session(["e", "y", "empty"], hw,
                                                     setup=lambda h: setattr(h, "usb_rev_hubs", [USB_EH]))
    plan = console.split("=== Plan", 1)[-1]
    expect("every optional device on USB wiring: no go-run question, since that go run needs its cable pulls",
           len(stdin.reads) == 3 and no_reads_after_start(stdin) and "go run too" not in console)
    expect("every optional device: the plan is default, Photon with I2C, Dashboard Off, and says why no go run",
           "1. Default suite: about 55-75 min" in plan and "2. Photon block (usb wiring): about 55 min" in plan
           and "3. Bench: Dashboard Off" in plan and "needs its cable pulls (--interactive)" in plan
           and stub.keys() == ["probe", "default", "photon", "dashboard_off"])
    expect("every optional device: it says the default suite runs Pinpoint Absent with the Pinpoint on the Expansion Hub",
           "runs Bench: Pinpoint Absent in its place" in plan)
    expect("every optional device: Needs a person names Bench: Pinpoint with the Pinpoint back on the Control Hub",
           "(pinpoint_ch)" in index(s))
    expect("every optional device: by hand afterwards, move the Pinpoint back", "move the Pinpoint back" in index(s))
    s, stub, hub, code, console, stdin = run_session(["empty"], hw, argv=("--usb", "--pinpoint", "expansion-hub",
                                                                          "--octoquad", "yes", "--soak-hours", "2"),
                                                     setup=lambda h: setattr(h, "usb_rev_hubs", [USB_EH]))
    expect("USB wiring with --soak-hours: no soak stage, and the index says why",
           "photon_soak" not in stub.keys() and "needs its cable pulls (--interactive)" in index(s))

    s, stub, hub, code, console, stdin = run_session(["y", "e", "y", "y", "2.5", "empty"], dict(BARE, expansion=2))
    plan = console.split("=== Plan", 1)[-1]
    expect("RS485 Expansion Hub: the Photon block runs --photon-rs485 --photon-i2c, the soak capped at 2.5 h",
           "--photon-rs485" in stub.argv("photon") and "--photon-i2c" in stub.argv("photon")
           and stub.argv("photon_soak")[stub.argv("photon_soak").index("--photon-soak-hours") + 1] == "2.5"
           and "about 50 min" in plan)
    soak = stub.argv("photon_soak")
    expect("RS485 with the I2C reads: the go run ran after the short block passed, asked with the docs' order",
           stub.keys() == ["probe", "default", "photon", "photon_soak", "dashboard_off"]
           and "--photon-soak" in soak and "--photon-i2c" in soak and "3. Photon go run with the long soak (rs485 "
           "wiring)" in plan and "only once the USB short block has passed too" in console)
    s, stub, hub, code, console, stdin = run_session(["y", "c", "y", "empty"], dict(BARE, expansion=2))
    expect("RS485 without the I2C reads: no go-run question, no soak, and the plan says why",
           len(stdin.reads) == 4 and "photon_soak" not in stub.keys() and "go run too" not in console
           and "the go run needs the I2C reads (--photon-i2c)" in console.split("=== Plan", 1)[-1])

    slow = {"photon": lambda st, argv, folder: (write_stage(folder, "photon", None, soak_rate=508.0), FakeProc())[1]}
    s, stub, hub, code, console, stdin = run_session(["y", "e", "y", "y", "2.5", "empty"], dict(BARE, expansion=2),
                                                     on=slow)
    expect("soak gate: a short-block rate that can't reach the loss bound within the cap skips the soak and says why",
           "photon_soak" not in stub.keys() and "dashboard_off" in stub.keys()
           and "past the 2.5 h cap" in index(s))
    failing = {"photon": lambda st, argv, folder: (write_stage(folder, "photon", None, fails=2, verdict="NOT SAFE"),
                                                    FakeProc(code=1))[1]}
    s, stub, hub, code, console, stdin = run_session(["y", "e", "y", "y", "", "empty"], dict(BARE, expansion=2),
                                                     on=failing)
    expect("soak gate: a short block that didn't pass skips the soak, and Dashboard Off still runs",
           "photon_soak" not in stub.keys() and stub.keys()[-1] == "dashboard_off"
           and "verdict is NOT SAFE" in index(s) and code == 1)


def chain():
    def fail_default(st, argv, folder):
        write_stage(folder, "default", None, fails=3, stopped="a bench raised")
        return FakeProc(["runner default"], code=1)

    hw = dict(BARE, usbRevHubs=[USB_EH])
    usb_eh = (lambda h: setattr(h, "usb_rev_hubs", [USB_EH]))
    s, stub, hub, code, console, stdin = run_session(["c", "y", "empty"], hw, on={"default": fail_default},
                                                     setup=usb_eh)
    text = index(s)
    expect("a failing stage (exit 1, 3 checks failed, stopped early) doesn't stop the chain",
           stub.keys() == ["probe", "default", "photon", "dashboard_off"] and code == 1)
    expect("the index gives its verdict line and lists its failures",
           "3 of 8 checks failed; stopped early: a bench raised; runner exit 1" in text
           and "- broken 2: detail 2 (FAIL means: something)" in text)

    def raises(st, argv, folder):
        raise OSError("no python3 here")

    s, stub, hub, code, console, stdin = run_session(["c", "y", "empty"], hw, on={"photon": raises}, setup=usb_eh)
    expect("a stage that raises doesn't stop the chain: Dashboard Off still runs",
           stub.keys() == ["probe", "default", "photon", "dashboard_off"] and code == 1
           and "the session couldn't run it: OSError: no python3 here" in index(s))

    procs = {}

    def hangs(st, argv, folder):
        def on_wait(proc):
            if proc.waits == 1:
                raise subprocess.TimeoutExpired(argv, 1)
            write_stage(folder, "default", None)
            proc.code = -2
        procs["default"] = FakeProc(on_wait=on_wait)
        return procs["default"]

    s, stub, hub, code, console, stdin = run_session(["c", "y", "empty"], hw, on={"default": hangs}, setup=usb_eh)
    expect("a stage past its time limit gets Ctrl-C (so its runner puts the hub back), and the chain goes on",
           getattr(procs["default"], "signals", []) == [signal.SIGINT]
           and stub.keys() == ["probe", "default", "photon", "dashboard_off"] and "ran past its 4.0 h limit" in index(s))

    def reboot(st, argv, folder):
        write_stage(folder, "default", None)
        st.hub.disappear(5)
        return FakeProc(["runner default"], code=1)

    s, stub, hub, code, console, stdin = run_session(["c", "y", "empty"], hw, on={"default": reboot}, setup=usb_eh)
    expect("a hub reboot mid-session doesn't stop the chain: the session waits for it on the cable and goes on",
           hub.reboots == 1 and stub.keys() == ["probe", "default", "photon", "dashboard_off"]
           and "waiting for the hub before Photon block" in console and not s.stopped)

    def left_on_bench(st, argv, folder):
        write_stage(folder, "default", None, stopped="restoring the config failed")
        st.hub.configs["bench_main"] = "<Robot type=\"FirstInspires-FTC\"/>"
        st.hub.active = {"name": "bench_main", "location": "LOCAL_STORAGE", "resourceId": 0, "isDirty": False}
        return FakeProc(code=1)

    s, stub, hub, code, console, stdin = run_session(["n", "empty"], BARE, on={"default": left_on_bench})
    dash = stub.argv("dashboard_off")
    expect("after a stage that couldn't restore, the next runner is told the session's original config",
           json.loads(dash[dash.index("--original-entry") + 1])["name"] == "betta_decode"
           and "--original-entry" not in stub.argv("default"))
    expect("at the end a probe-only runner puts the config back, in its own session so Ctrl-C can't cut it short",
           "restore" in stub.keys() and hub.active["name"] == "betta_decode"
           and not [n for n in hub.configs if n.startswith("bench_")]
           and any(stage_key(a) == "restore" and kw.get("start_new_session") and callable(kw.get("sigint"))
                   for a, kw in stub.spawned)
           and stub.argv("restore")[stub.argv("restore").index("--hub-serialno") + 1] == "FAKEHUB0001"
           and "betta_decode active again after the restore run" in index(s))


def unsafe():
    hw = dict(BARE, usbRevHubs=[USB_EH])
    usb_eh = (lambda h: setattr(h, "usb_rev_hubs", [USB_EH]))
    mark = {}

    def other_hub(st, argv, folder):
        write_stage(folder, "default", None)
        st.hub.hub_serial = "OTHERHUB0002"
        return FakeProc()

    s, stub, hub, code, console, stdin = run_session(["c", "y", "empty"], hw, on={"default": other_hub},
                                                     setup=usb_eh)
    after = hub.calls[[i for i, c in enumerate(hub.calls) if c.endswith("getprop ro.serialno")][-1] + 1:]
    expect("another hub in its place stops the session before the next stage",
           stub.keys() == ["probe", "default"] and "isn't the one the session started on" in s.stopped and code == 1)
    expect("... and the session doesn't touch that hub at the end: no settings, config, forward or app command",
           not [c for c in after if any(w in c for w in ("run-as", " push ", "--remove", " am ", "install"))]
           and "not touched at the end" in index(s))

    def hijack(st, argv, folder):
        write_stage(folder, "default", None)
        st.hub.lsof = "p999\ncssh\n"
        mark["calls"] = len(st.hub.calls)
        return FakeProc()

    s, stub, hub, code, console, stdin = run_session(["c", "y", "empty"], hw, on={"default": hijack}, setup=usb_eh)
    expect("a failing adb-port check stops the session before the next stage, naming the process",
           stub.keys() == ["probe", "default"] and "ssh (pid 999)" in s.stopped)
    expect("... and not one adb command is sent after it, the end's put-back included",
           len(hub.calls) == mark["calls"] and "not touched at the end" in index(s))

    s, stub, hub, code, console, stdin = run_session(["n", "empty"], dict(BARE, expansion=2))
    expect("an Expansion Hub on RS485 the answers said wasn't there stops the session after the probe, nothing driven",
           stub.keys() == ["probe"] and "the ports statement didn't name the Expansion Hub's motor ports" in s.stopped)

    def swap(st, argv, folder):
        write_stage(folder, "default", None)
        st.hub.usb_rev_hubs = ["DQ9ZZZZZ"]
        return FakeProc()

    s, stub, hub, code, console, stdin = run_session(["c", "y", "empty"], hw, on={"default": swap}, setup=usb_eh)
    expect("another Expansion Hub on USB before the Photon block stops the session",
           stub.keys() == ["probe", "default"] and "not DQ2AB3CD" in s.stopped)
    expect("a statement that doesn't cover a stage's ports stops it, even if the plan were wrong",
           guard_check())


def guard_check():
    s = S.Session(S.parse_args(["--usb"]))
    s.covered = {S.CH}
    s.check_adb_port = lambda when: None
    s.wait_for_hub = lambda what: None
    s.read_identity = lambda: "X"
    s.identity = "X"
    s.adb = type("A", (), {"serial": "1"})()
    try:
        s.check_safe(S.Stage("photon", "Photon block", ["--photon"], "", drives={S.CH, S.EH}))
    except S.Stop as e:
        return "didn't name" in str(e)
    return False


def ctrl_c():
    hw = dict(BARE, expansion=2)
    for key in ("probe", "default", "photon", "photon_soak", "dashboard_off"):
        state = {}
        put_back = []

        def interrupted(st, argv, folder, key=key):
            def on_wait(proc):
                if proc.waits == 1:
                    os.kill(os.getpid(), signal.SIGINT)
                    t0 = time.monotonic()
                    while time.monotonic() - t0 < 2:
                        pass
                    state["notRaised"] = True
                else:
                    write_stage(folder, key, st.hardware)
                    if key == "dashboard_off":
                        st.hub.dashboard_off = True
                    st.hub.forwards.append(fake_hub.Forward(st.hub, st.hub.serial, 40404, 8000, virtual=True))
                    state["restoredAt"] = len(st.hub.calls)
                    proc.code = -2
                    spy(st.hub)
            return FakeProc([f"runner {key}"], on_wait=on_wait)

        def spy(hub):
            adb = hub.adb

            def recorded(argv, env=None):
                put_back.append((" ".join(argv[1:]), signal.getsignal(signal.SIGINT)))
                return adb(argv, env)
            hub.adb = recorded

        mine, theirs = (fake_hub.USB_SERIAL, 7777, 8000), ("R58M90ABCDE", 7778, 8000)
        s, stub, hub, code, console, stdin = run_session(
            ["y", "e", "y", "y", "", "y", "empty"], hw, argv=("--usb", "--install"), on={key: interrupted},
            hub=fake_hub.Hub(usb=True, foreign_forwards=[mine, theirs]))
        later = hub.calls[state.get("restoredAt", 0):]
        expect(f"Ctrl-C during {key}: every adb command of the put-back ran with Ctrl-C ignored, so its children "
               "inherit that", put_back and all(handler == signal.SIG_IGN for _, handler in put_back))
        expect(f"Ctrl-C during {key}: REINSTALL.sh and the restore runner start in their own session, with Ctrl-C "
               "caught (so exec gives them its default back for the time limit)",
               all(kw.get("start_new_session") and callable(kw.get("sigint")) for a, kw in stub.spawned
                   if a[:2] == ["/bin/bash", s.reinstall] or stage_key(a) == "restore")
               and any(a[:2] == ["/bin/bash", s.reinstall] for a, _ in stub.spawned))
        expect(f"Ctrl-C during {key}: a forward the person had before and another device's forward stay",
               [(f.serial, f.local, f.remote) for f in hub.forwards] == [mine, theirs]
               and not [c for c in hub.calls if "--remove tcp:7777" in c or "--remove tcp:7778" in c])
        expect(f"Ctrl-C during {key}: the session waited for its runner to put the hub back, then stopped the chain",
               "notRaised" not in state and "restoredAt" in state and stub.keys()[-1] == key
               and s.stopped == "stopped by Ctrl-C" and code == 1)
        expect(f"Ctrl-C during {key}: afterwards it checked the config and removed the runner's leftover forward",
               any("run-as com.qualcomm.ftcrobotcontroller cat shared_prefs/com.qualcomm" in c for c in later)
               and any(c.endswith("forward --remove tcp:40404") for c in later)
               and not [f for f in hub.forwards if f.local == 40404])
        expect(f"Ctrl-C during {key}: the hub's own app went back last, as asked (REINSTALL.sh)",
               stub.events[-1][0] == "reinstall" and "own app put back" in index(s))
        if key == "dashboard_off":
            expect("Ctrl-C during Dashboard Off with the dashboard left off: the session turns it back on",
                   not hub.dashboard_off and any("rm -f shared_prefs/FtcDashboard.xml" in c for c in later)
                   and "dashboard on" in index(s))
    real_ctrl_c()

    class Stopping(FakeStdin):
        def readline(self):
            if len(self.reads) == 1:
                raise KeyboardInterrupt
            return super().readline()

    hub = fake_hub.Hub(usb=True)
    OUT.mkdir(parents=True, exist_ok=True)
    folder = pathlib.Path(tempfile.mkdtemp(prefix="s-", dir=OUT)) / "session"
    stdin = Stopping(["n", "empty"])
    s = S.Session(S.parse_args(["--usb", "--out", str(folder)]), stdin=stdin, out=io.StringIO())
    stub = Stub(hub, stdin, BARE)
    s.popen = stub.popen
    with hub.installed(rb):
        code = s.run()
    expect("Ctrl-C during the questions: it stops with nothing on the hub changed and no child started",
           code == 1 and "Ctrl-C during the questions" in s.stopped and not stub.spawned
           and "nothing on the hub was changed" in index(s)
           and not [c for c in hub.calls if any(w in c for w in (" push ", " am ", "install", "--remove"))])

    def install_interrupted(st, argv, **kw):
        folder = pathlib.Path(argv[3])
        folder.mkdir(parents=True, exist_ok=True)
        (folder / "reinstall_path.txt").write_text(f"{folder / 'REINSTALL.sh'}\n")

        def on_wait(proc):
            if proc.waits == 1:
                os.kill(os.getpid(), signal.SIGINT)
                t0 = time.monotonic()
                while time.monotonic() - t0 < 2:
                    pass
            proc.code = -2
        return FakeProc(["backing up"], on_wait=on_wait)

    class InstallStub(Stub):
        def popen(self, argv, **kw):
            argv = [str(a) for a in argv]
            if argv[:2] == ["/bin/bash", str(S.BARE_HUB)]:
                self.spawned.append((argv, kw))
                return install_interrupted(self, argv, **kw)
            return super().popen(argv, **kw)

    hub = fake_hub.Hub(usb=True)
    folder = pathlib.Path(tempfile.mkdtemp(prefix="s-", dir=OUT)) / "session"
    stdin = FakeStdin(["n", "y", "empty"])
    s = S.Session(S.parse_args(["--usb", "--install", "--out", str(folder)]), stdin=stdin, out=io.StringIO())
    stub = InstallStub(hub, stdin, BARE)
    s.popen = stub.popen
    with hub.installed(rb):
        code = s.run()
    expect("Ctrl-C during the install: no stage runs, and the hub's own app goes back as asked (REINSTALL.sh)",
           s.stopped == "stopped by Ctrl-C" and not stub.keys() and stub.events and stub.events[-1][0] == "reinstall"
           and "own app put back" in index(s))


RUNNER_STUB = r'''import pathlib, signal, sys, time
out = pathlib.Path(sys.argv[sys.argv.index("--out") + 1])
out.mkdir(parents=True, exist_ok=True)
(out / "started").write_text("")
try:
    time.sleep(60)
except KeyboardInterrupt:
    time.sleep(0.5)
    (out / "restored").write_text("")
    raise
'''


def real_ctrl_c():
    """A real child that takes half a second to put the hub back after Ctrl-C: the session must wait for it."""
    script = OUT / "runner_stub.py"
    script.write_text(RUNNER_STUB)
    holder = {}

    def real(st, argv, folder):
        proc = subprocess.Popen([sys.executable, "-I", str(script), "--out", str(folder)], stdin=subprocess.DEVNULL,
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        holder["proc"] = proc

        def ctrl_c():
            while not (folder / "started").exists():
                time.sleep(0.05)
            os.kill(proc.pid, signal.SIGINT)
            os.kill(os.getpid(), signal.SIGINT)
        threading.Thread(target=ctrl_c, daemon=True).start()
        return proc

    s, stub, hub, code, console, stdin = run_session(["n", "empty"], BARE, on={"default": real})
    folder = s.folder / "01-default"
    expect("a real runner: Ctrl-C reached it, the session waited until it had put the hub back, and stopped there",
           (folder / "restored").exists() and holder["proc"].returncode is not None
           and stub.keys() == ["probe", "default"] and s.stopped == "stopped by Ctrl-C")


CHILD_STUB = r'''import pathlib, sys, time
out = pathlib.Path(sys.argv[sys.argv.index("--out") + 1])
out.mkdir(parents=True, exist_ok=True)
(out / "started").write_text("")
t0 = time.monotonic()
while not (out / "go").exists() and time.monotonic() - t0 < 30:
    time.sleep(0.05)
time.sleep(0.3)
(out / "done").write_text("")
'''


def spawn_cases():
    """A Ctrl-C while a stage's runner starts, a closed terminal mid-stage, and what Ctrl-C a child starts with."""
    script = OUT / "runner_stub.py"
    script.write_text(RUNNER_STUB)
    holder, seen = {}, []

    def early(st, argv, folder):
        proc = subprocess.Popen([sys.executable, "-I", str(script), "--out", str(folder)], stdin=subprocess.DEVNULL,
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        holder["proc"], holder["folder"] = proc, folder
        while not (folder / "started").exists():
            time.sleep(0.05)
        os.kill(proc.pid, signal.SIGINT)
        os.kill(os.getpid(), signal.SIGINT)
        adb = st.hub.adb

        def recorded(argv, env=None):
            seen.append((folder / "restored").exists())
            return adb(argv, env)
        st.hub.adb = recorded
        return proc

    s, stub, hub, code, console, stdin = run_session(["n", "empty"], BARE, on={"default": early})
    expect("a Ctrl-C while a stage's runner is still starting: the session waits for that runner to put the hub back "
           "before its own first adb command, then stops",
           "Ctrl-C: waiting for it to put the hub back" in console and holder["proc"].returncode is not None
           and seen and all(seen) and stub.keys() == ["probe", "default"] and s.stopped == "stopped by Ctrl-C")

    child = OUT / "child_stub.py"
    child.write_text(CHILD_STUB)
    hung = {}

    def hangup(st, argv, folder):
        proc = subprocess.Popen([sys.executable, "-I", str(child), "--out", str(folder)], stdin=subprocess.DEVNULL,
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        hung["proc"] = proc
        write_stage(folder, "default", st.hardware)

        def close_terminal():
            while not (folder / "started").exists():
                time.sleep(0.05)
            os.kill(proc.pid, signal.SIGHUP)
            os.kill(os.getpid(), signal.SIGHUP)
            (folder / "go").write_text("")
        threading.Thread(target=close_terminal, daemon=True).start()
        return proc

    s, stub, hub, code, console, stdin = run_session(["n", "empty"], BARE, on={"default": hangup})
    expect("a closed terminal mid-stage (SIGHUP to the session and its runner) ends neither: the runner finishes and "
           "the next stage starts", hung["proc"].returncode == 0 and stub.keys() == ["probe", "default", "dashboard_off"]
           and s.stopped is None)

    probe = [sys.executable, "-I", "-c", "import signal; print(signal.getsignal(signal.SIGINT) is "
                                         "signal.default_int_handler, flush=True)"]
    s = S.Session(S.parse_args(["--usb"]), out=io.StringIO())
    outputs = {}
    previous = signal.signal(signal.SIGINT, signal.SIG_IGN)
    try:
        for new_session in (True, False):
            proc = s.spawn(probe, new_session)
            outputs[new_session] = proc.stdout.read().strip()
            proc.wait()
        after = signal.getsignal(signal.SIGINT)
    finally:
        signal.signal(signal.SIGINT, previous)
    expect("while the session ignores Ctrl-C, a child in its own session starts with Python's Ctrl-C handler, any "
           "other inherits ignoring it, and the session still ignores it afterwards",
           outputs == {True: "True", False: "False"} and after == signal.SIG_IGN)


def install_checks():
    s, stub, hub, code, console, stdin = run_session(["n", "y", "empty"], BARE, argv=("--install",))
    bare = next(a for a, _ in stub.spawned if a[:2] == ["/bin/bash", str(S.BARE_HUB)])
    expect("--install over Wi-Fi: bare_hub.sh gets the hub's ro.serialno to check before it backs up and installs",
           bare[bare.index("--expect-serialno") + 1] == "FAKEHUB0001" and bare[3].endswith("install"))

    class Swap(Stub):
        def popen(self, argv, **kw):
            if str(argv[0]) == "caffeinate":
                self.hub.hub_serial = "OTHERHUB0002"
            return super().popen(argv, **kw)

    hub = fake_hub.Hub()
    OUT.mkdir(parents=True, exist_ok=True)
    folder = pathlib.Path(tempfile.mkdtemp(prefix="s-", dir=OUT)) / "session"
    stdin = FakeStdin(["n", "y", "empty"])
    s = S.Session(S.parse_args(["--install", "--out", str(folder)]), stdin=stdin, out=io.StringIO())
    stub = Swap(hub, stdin, BARE)
    s.popen = stub.popen
    with hub.installed(rb):
        code = s.run()
    expect("--install: another hub answering after the questions stops it before bare_hub.sh, with nothing installed",
           code == 1 and "isn't the one the session started on" in s.stopped
           and not [a for a, _ in stub.spawned if a[0] == "/bin/bash"] and "nothing on the hub was changed" in index(s))


def put_back_cases():
    usb_eh = (lambda h: setattr(h, "usb_rev_hubs", [USB_EH]))

    def app_dies(st, argv, folder):
        write_stage(folder, "default", None)
        st.hub.app_broken, st.hub.app_up = True, False
        return FakeProc(code=1)

    saved = S.HUB_BACK_S
    S.HUB_BACK_S = 20
    try:
        s, stub, hub, code, console, stdin = run_session(["n", "y", "empty"], BARE, argv=("--usb", "--install"),
                                                         on={"default": app_dies})
    finally:
        S.HUB_BACK_S = saved
    text = index(s)
    expect("the app down at the end (the hub right, adb up): the session still checks the config, removes forwards "
           "and puts the hub's own app back, as asked",
           "wasn't back within" in s.stopped and stub.events[-1][0] == "reinstall" and "own app put back" in text
           and "betta_decode active" in text and "not touched" not in text and "once adb reaches" not in text)

    def unreadable(st, argv, folder):
        write_stage(folder, "dashboard_off", None)
        st.hub.dashboard_unreadable = True
        return FakeProc()

    s, stub, hub, code, console, stdin = run_session(["n", "empty"], BARE, on={"dashboard_off": unreadable})
    text = index(s)
    expect("a saved dashboard switch run-as can't read at the end: FAILED, with the command, never 'dashboard on'",
           "dashboard NOT CHECKED: FAILED to read its saved switch (run-as: package not debuggable" in text
           and rb.DASHBOARD_RECOVERY_SHELL in text and "dashboard on" not in text and code == 1
           and "what the Put back section above says FAILED" in text)

    def forwards_fail(st, argv, folder):
        write_stage(folder, "dashboard_off", None)
        st.hub.forward_list_fails = True
        return FakeProc()

    s, stub, hub, code, console, stdin = run_session(["n", "empty"], BARE, on={"dashboard_off": forwards_fail})
    expect("adb forward --list failing at the end: FAILED remove_forwards, never 'none left'",
           "FAILED remove_forwards: BenchError: adb forward --list failed" in index(s)
           and "none left by the runners" not in index(s) and code == 1)
    s, stub, hub, code, console, stdin = run_session(["n", "empty"], BARE,
                                                     setup=lambda h: setattr(h, "forward_list_fails", True))
    expect("adb forward --list failing at the start: it stops before any question, with nothing changed",
           not stdin.reads and "adb forward --list failed" in s.stopped and not stub.keys()
           and "nothing on the hub was changed" in index(s))

    def table_flaky(st, argv, folder):
        write_stage(folder, "default", None)
        st.hub.socket_table_failures = 2
        return FakeProc()

    s, stub, hub, code, console, stdin = run_session(["c", "y", "empty"], dict(BARE, usbRevHubs=[USB_EH]),
                                                     on={"default": table_flaky}, setup=usb_eh)
    expect("an unreadable socket table after a stage: the session waits instead of taking port 8000 as up",
           "the hub's socket table couldn't be read" in console and hub.socket_table_failures == 0
           and stub.keys() == ["probe", "default", "photon", "dashboard_off"] and not s.stopped)


PROMPT_SITES = {"gp_interactive": "gamepad", "run_clock": "clock", "run_limelight_replug": "limelight",
                "_pinpoint_dropout": "pinpoint", "_pinpoint_raw_dropout": "pinpoint", "_octoquad_dropout": "octoquad",
                "_octoquad_markers": "octoquad_chain", "_octoquad_unguarded": "octoquad_chain",
                "_octoquad_latejoin": "octoquad_chain", "run_expansion": "expansion",
                "photon_replug": "photon_replug", "_dash_off_phase": "dashboard_battery",
                "run_turret_zero": "turret_zero", "tz_branch": "turret_zero", "_turret_zero": "turret_zero",
                "tz_v2": "turret_zero", "tz_track": "turret_zero", "tz_brownout": "turret_zero"}
HELPERS = {"ask", "tell", "answer"}


def prompt_sites():
    """{method: kind} for every place run_bench.py waits for a person, by the method it sits in."""
    found, method = {}, None
    for line in (WORKTREE / "scripts/bench/run_bench.py").read_text().splitlines():
        m = re.match(r"(?:    )?def (\w+)\(", line)
        if m:
            method = m.group(1)
        for kind, needle in (("ask", "self.ask("), ("answer", "self.answer("), ("input", "input("), ("tell", "self.tell(")):
            if needle in line and method not in HELPERS:
                found.setdefault(method, set()).add(kind)
    return found


def module_prompts():
    """{bench name: whether its file in scripts/bench/benches/ waits for a person anywhere}."""
    out = {}
    for b in rb.MODULE_BENCHES:
        text = pathlib.Path(b.module.__file__).read_text()
        out[b.name] = any(n in text for n in (".ask(", ".answer(", "input(", ".tell("))
    return out


def needs_a_person():
    sites = prompt_sites()
    asks = {m for m, kinds in sites.items() if kinds - {"tell"}} - {"photon_guard_ports", "port_statements"}
    expect("the typed port statements are asked only for a bench that declares a person, and the session gives no "
           "stage their flags", set(rb.TYPED_STATEMENTS) <= set(S.PROMPTED_FLAGS)
           and all(b.person for b in rb.MODULE_BENCHES if b.statement in rb.TYPED_STATEMENTS))
    prompts = module_prompts()
    expect(f"every bench in scripts/bench/benches/ that waits for a person declares one (not: "
           f"{sorted(n for n, asks in prompts.items() if asks and not rb.module_bench(n).person)})",
           all(rb.module_bench(n).person for n, asks in prompts.items() if asks))
    expect(f"every place run_bench.py waits for a person is a known prompted bench (unknown: "
           f"{sorted(asks - set(PROMPT_SITES))})", asks == set(PROMPT_SITES))
    expect("the only other prompts are the Photon block's: the ports confirmation and the I2C plug-in notes, which the "
           "session's start replaces and which never wait without a terminal",
           {m for m, kinds in sites.items() if kinds == {"tell"}} == {"run_photon"})
    s, stub, hub, code, console, stdin = run_session(["c", "y", "empty"], dict(BARE, usbRevHubs=[USB_EH]),
                                                     setup=lambda h: setattr(h, "usb_rev_hubs", [USB_EH]))
    text = index(s).split("## Needs a person", 1)[-1].split("## By hand", 1)[0]
    keys = set(re.findall(r"\*\* \((\w+)\)\. Have: ", text))
    expect("Needs a person names every prompted bench, each with what to have on the table",
           set(PROMPT_SITES.values()) <= keys and text.count("Have: ") == len(keys))
    expect("Needs a person names every bench in scripts/bench/benches/ that declares a person",
           {b.name for b in rb.MODULE_BENCHES if b.person} <= keys)
    commands = re.findall(r"`python3 scripts/bench/run_bench.py ([^`]*)`", text)
    bad = []
    for c in commands:
        try:
            with contextlib.redirect_stderr(io.StringIO()):
                args = rb.parse_args(shlex.split(c))
        except SystemExit:
            bad.append(c)
            continue
        if not (args.ports_empty and args.usb and args.serial == fake_hub.USB_SERIAL):
            bad.append(c)
    expect(f"each command parses with run_bench.py's own parser and keeps the session's link (bad: {bad})",
           len(commands) == len(keys) and not bad)
    expect("no stage of the session carries a prompt flag", not [a for a, _ in stub.spawned
                                                                 if set(a) & set(S.PROMPTED_FLAGS)])
    st = S.Stage("default", "Default suite", ["--interactive"], "")
    try:
        S.Session(S.parse_args([])).run_stage(st)
        refused = False
    except RuntimeError:
        refused = True
    expect("a stage with --interactive is refused before it starts", refused)


def questions():
    stdin = FakeStdin([], tty=False)
    OUT.mkdir(parents=True, exist_ok=True)
    folder = pathlib.Path(tempfile.mkdtemp(prefix="s-", dir=OUT)) / "session"
    with contextlib.redirect_stderr(io.StringIO()) as err:
        code = S.Session(S.parse_args(["--out", str(folder)]), stdin=stdin, out=io.StringIO()).run()
    expect("without a terminal it refuses to start, touching nothing", code == 2 and not folder.exists()
           and "start it from a terminal" in err.getvalue() and not stdin.reads)
    s, stub, hub, code, console, stdin = run_session(["maybe", "n", "full", "empty"], BARE)
    expect("a wrong answer is asked again, at the start", "That isn't one of the answers" in console
           and no_reads_after_start(stdin) and len(stdin.reads) == 4)
    s, stub, hub, code, console, stdin = run_session(["empty"], dict(BARE, usbRevHubs=[USB_EH]),
                                                     argv=("--usb", "--pinpoint", "expansion-hub", "--octoquad", "yes",
                                                           "--soak-hours", "0"),
                                                     setup=lambda h: setattr(h, "usb_rev_hubs", [USB_EH]))
    expect("answers given as flags aren't asked; only the typed statement is", len(stdin.reads) == 1
           and "--photon-i2c" in stub.argv("photon") and "photon_soak" not in stub.keys())
    s, stub, hub, code, console, stdin = run_session(["n"], BARE)
    expect("a terminal that closes before the statement stops it with nothing changed",
           "terminal closed" in s.stopped and not stub.keys() and "nothing on the hub was changed" in index(s))
    s, stub, hub, code, console, stdin = run_session(["n", "empty"], BARE, argv=())
    expect("over Wi-Fi: no --usb or --serial to the runners, --gradle-offline when Gradle builds only offline, and the "
           "end says to reconnect to the internet", "--usb" not in stub.argv("default")
           and "--gradle-offline" in stub.argv("default") and "Reconnect to the internet" in console)
    s, stub, hub, code, console, stdin = run_session(["n", "empty"], BARE, gradle=(1, 1))
    expect("Gradle building neither online nor offline: the runners get --no-sloth and the plan says what that costs",
           "--no-sloth" in stub.argv("default") and "can't pass" in console)
    s, stub, hub, code, console, stdin = run_session(["n", "empty"], BARE,
                                                     setup=lambda h: setattr(h, "dashboard_off", True))
    expect("a hub with its dashboard off is refused before any question, with the command to turn it back on",
           not stdin.reads and "switched off" in s.stopped and "rm -f shared_prefs/FtcDashboard.xml" in s.stopped)


def install_only():
    tb = load("test_bare_hub_usb", HERE / "test_bare_hub_usb.py")
    OUT.mkdir(parents=True, exist_ok=True)
    absolute = pathlib.Path(tempfile.mkdtemp(prefix="install-", dir=OUT)) / "install"
    for devices, argv, label, where in (([(tb.SERIAL, "device")], ["--usb", "--serial", tb.SERIAL], "USB", absolute),
                                        ([(tb.WIFI_SERIAL, "device")], ["--expect-serialno", "FAKEHUB0001"], "Wi-Fi",
                                         "bench-results/sess/install")):
        code, out, calls, py, root, gradle = tb.bare_hub(devices, ["--install-only", str(where)] + argv,
                                                         intercept=True)
        folder = root / where
        info = json.loads((folder / "installed.json").read_text()) if (folder / "installed.json").exists() else {}
        expect(f"bare_hub.sh --install-only ({label}): backs up, installs, starts the app and writes installed.json",
               code == 0 and any(" install -r -t -g " in f" {c} " for c in calls) and info.get("reinstall", "")
               .endswith("REINSTALL.sh") and pathlib.Path(info["reinstall"]).exists()
               and (folder / "reinstall_path.txt").read_text().strip() == info["reinstall"])
        expect(f"bare_hub.sh --install-only ({label}): never starts the runner, and hands over its flags",
               not [p for p in py if p.startswith("runner")] and "running the benches" not in out
               and info.get("runnerFlags") == (["--gradle-offline"] if label == "Wi-Fi" else [])
               and info.get("serial") == (tb.SERIAL if label == "USB" else tb.WIFI_SERIAL)
               and (folder / "console.log").exists() and "Reconnect to the internet" not in out)


def runner_flags():
    hub = fake_hub.Hub(usb=True, everything=True)
    hub.usb_rev_hubs = [USB_EH]
    OUT.mkdir(parents=True, exist_ok=True)
    out = pathlib.Path(tempfile.mkdtemp(prefix="probe-", dir=OUT)) / "run"
    handlers = signal.getsignal(signal.SIGINT), signal.getsignal(signal.SIGHUP)
    with hub.installed(rb), contextlib.redirect_stdout(io.StringIO()):
        code = rb.Runner(rb.parse_args(["--probe-only", "--usb", "--ports-empty", "--out", str(out)])).run()
    signal.signal(signal.SIGINT, handlers[0])
    signal.signal(signal.SIGHUP, handlers[1])
    hw = json.loads((out / "runner_results.json").read_text()).get("hardware") or {}
    entered = [l for l in (out / "run.log").read_text().splitlines() if "] === " in l]
    expect("run_bench.py --probe-only records what the probe found and the USB REV hubs, runs no bench",
           code == 0 and hw.get("expansion") == 2 and hw.get("webcam") == "UC762" and hw.get("usbRevHubs") == [USB_EH]
           and not entered)
    expect("run_bench.py --probe-only puts the config back and leaves no bench config or forward",
           hub.active["name"] == "betta_decode" and not [n for n in hub.configs if n.startswith("bench_")]
           and not hub.forwards)
    for argv in (["--probe-only", "--hub"], ["--probe-only", "--all"], ["--photon-soak-hours", "2"],
                 ["--photon-soak", "--photon-soak-hours", "5"], ["--photon-soak", "--photon-soak-hours", "0"]):
        try:
            with contextlib.redirect_stderr(io.StringIO()):
                rb.parse_args(argv)
            expect(f"run_bench.py refuses {' '.join(argv)}", False)
        except SystemExit:
            expect(f"run_bench.py refuses {' '.join(argv)}", True)
    expect("--photon-soak-hours sets the soak's seconds; without it the soak and the short block keep theirs",
           rb.photon_soak_seconds(rb.parse_args(["--photon-soak", "--photon-soak-hours", "2.5"])) == 9000
           and rb.photon_soak_seconds(rb.parse_args(["--photon-soak"])) == rb.PHOTON_SOAK_MAX_S
           and rb.photon_soak_seconds(rb.parse_args(["--photon"])) == 60
           and rb.photon_soak_seconds(rb.parse_args(["--photon", "--quick"])) == 20)


def main():
    for section in (plans, questions, chain, unsafe, ctrl_c, spawn_cases, install_checks, put_back_cases, needs_a_person,
                    install_only, runner_flags):
        try:
            section()
        except Exception as e:
            expect(f"{section.__name__} ran to its end ({type(e).__name__}: {e})", False)
            traceback.print_exc()
    print(f"{passed} passed, {len(failed)} failed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
