#!/usr/bin/env python3
"""Start it once: asks everything first, probes the hub, prints the plan, then runs every bench stage the plugged-in
hardware allows back to back, each in its own results folder, and puts the hub back however it ends.

    python3 scripts/bench/session.py [--usb [--serial S]] [--install]

See "Start it once" in scripts/bench/README.md. Python 3 standard library only.
"""

import argparse
import datetime
import importlib.util
import json
import math
import os
import shlex
import signal
import subprocess
import sys
import threading
import traceback
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parents[1]
RUNNER = HERE / "run_bench.py"
BARE_HUB = HERE / "bare_hub.sh"


def load_runner(path):
    spec = importlib.util.spec_from_file_location("run_bench", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


rb = load_runner(RUNNER)

CH, EH = "controlHub", "expansionHub"
PORTS = {CH: "Control Hub motor ports 0-3 and servo ports 0-1",
         EH: "the Expansion Hub's motor ports 0-3 and servo ports 0-1"}
PINPOINT_PLACES = {"control-hub": "Control Hub I2C port 1", "expansion-hub": "Expansion Hub I2C port 1",
                   "none": "not plugged in"}
HUB_BACK_S = rb.DASH_OFF_BOOT_WAIT_S
POLL_S = rb.DASH_OFF_POLL_S
SOAK_CAP_H = rb.PHOTON_SOAK_MAX_S / 3600
SOAK_WRITES_NEEDED = math.ceil(3.0 / rb.PHOTON_SOAK_LOSS_TARGET)
INSTALL_LIMIT_S = 3600
GRADLE_LIMIT_S = 600
REINSTALL_LIMIT_S = 1800
# What the runner's own finally may take to put the config back after a Ctrl-C, plus its summary.
RUNNER_RESTORE_S = rb.RESTORE_WINDOW_S + 300
PROMPTED_FLAGS = ("--interactive", "--turret-zero", "--turret-zero-v2")
UNREADABLE = "unreadable"

# Every bench that stops for a person: (key, what it is, run_bench.py flags, what to have on the table).
PROMPTED = (
    ("gamepad", "Bench: Gamepad and Gamepad Linear driven by hand from the dashboard's Gamepad view",
     ["--gamepad", "--interactive"],
     "a USB gamepad on this Mac and a browser, about 5 min of keyboard, on-screen stick and pad presses; over --usb, "
     "nothing else on this Mac's port 8000"),
    ("clock", "Bench: Clock with the RC's web page opened while the hub's clock reads about 1970",
     ["--clock", "--interactive"],
     "a browser; power-cycle the hub first and connect no Driver Station or browser before the run, or the clock is "
     "already set and the step is skipped; over --usb, two adjacent free ports on this Mac"),
    ("limelight", "the Limelight pulled and replugged under Bench: Limelight Fault", ["--limelight", "--interactive"],
     "the Limelight 3A booted on a Control Hub USB port"),
    ("pinpoint", "the Pinpoint's I2C cable pulled and replugged in Bench: Pinpoint", ["--pinpoint", "--interactive"],
     "the goBILDA Pinpoint in Control Hub I2C port 1, both pods plugged in and still"),
    ("octoquad", "the OctoQuad's I2C cable pulled and replugged in Bench: OctoQuad", ["--octoquad", "--interactive"],
     "the OctoQuad MK2 in Control Hub I2C port 2, pods on its encoder ports 3 and 4, still"),
    ("expansion", "Bench: Expansion Hub again with the Pinpoint and OctoQuad moved to the Expansion Hub",
     ["--expansion", "--interactive"],
     "an Expansion Hub on RS485 on its own 12 V, the Pinpoint in Control Hub I2C port 1 and the OctoQuad in port 2"),
    ("photon_replug", "the Photon block's four Expansion Hub cable pulls (USB wiring), inside the go run that can say SAFE",
     ["--photon", "--usb-expansion", "--photon-soak", "--photon-i2c", "--interactive"],
     "the Expansion Hub on USB only (no RS485 cable), nothing on either hub's motor ports 0-3 or servo ports 0-1, the "
     "Pinpoint in Expansion Hub I2C port 1, the OctoQuad in Control Hub I2C port 2; 6-7 h, someone at the hub about "
     "6 h in for five prompts"),
    ("dashboard_battery", "Bench: Dashboard Off with the Control Hub's battery lead pulled once",
     ["--dashboard-off", "--interactive"],
     "the Control Hub on a battery lead you can pull, no Driver Station, every dashboard tab closed; about 12 min"),
    ("turret_zero", "Bench: Turret Zero (add --turret-zero-v2 with a REV-11-3174 on hand)", ["--turret-zero"],
     "the OctoQuad in Control Hub I2C port 2 with nothing on its channel 4, a REV Through Bore V1 on a stand with a "
     "12-mark dial, the REV-31-1815 cable, the split cable, a multimeter; about 25 min at the hub"),
)


class Stop(Exception):
    """Going on would be unsafe or can't work: the session stops and says why."""


class Interrupted(BaseException):
    """The first Ctrl-C."""


class Stage:
    def __init__(self, key, title, flags, about, drives=(), limit_s=None, minutes=(0, 0)):
        self.key, self.title, self.flags, self.about = key, title, list(flags), about
        self.drives = set(drives)
        self.minutes = minutes
        self.limit_s = limit_s
        self.folder = None
        self.exit = None
        self.raised = None
        self.interrupted = False
        self.timed_out = False
        self.seconds = None
        self.result = None


def dig(obj, *path):
    for key in path:
        if not isinstance(obj, dict):
            return None
        obj = obj.get(key)
    return obj


def stage_results(folder):
    """What a stage's runner left in its folder: its checks, why it stopped, and runner_results.json."""
    out = {"summary": None, "checks": 0, "failed": [], "stopped": None, "restoreFailed": None, "dashboardOff": False,
           "results": {}}
    summary = folder / "summary.md"
    if summary.exists():
        out["summary"] = str(summary)
        in_checks = False
        for line in summary.read_text().splitlines():
            if line.startswith("## "):
                in_checks = line.strip() == "## Checks"
            elif in_checks and line.startswith("- PASS "):
                out["checks"] += 1
            elif in_checks and line.startswith("- FAIL "):
                out["checks"] += 1
                out["failed"].append(line[len("- FAIL "):])
            if line.startswith("**Run stopped early: "):
                out["stopped"] = line[len("**Run stopped early: "):].rstrip("*")
            elif line.startswith("**Restore failed: "):
                out["restoreFailed"] = line[len("**Restore failed: "):].rstrip("*")
            elif line.startswith("**The dashboard may still be off"):
                out["dashboardOff"] = True
    results = folder / "runner_results.json"
    if results.exists():
        out["results"] = json.loads(results.read_text())
    return out


def verdict_line(stage):
    if stage.exit is None and stage.raised is None and not stage.interrupted:
        return "not started"
    r = stage.result or {}
    parts = []
    if stage.raised:
        parts.append(f"the session couldn't run it: {stage.raised}")
    if stage.interrupted:
        parts.append("stopped by Ctrl-C (its runner put the config back)")
    if stage.timed_out:
        parts.append(f"ran past its {stage.limit_s / 3600:.1f} h limit, so it was sent Ctrl-C")
    level = dig(r.get("results"), "photon", "verdict", "level")
    if level:
        parts.append({"SAFE": "Photon verdict SAFE TO TRY ON THE ROBOT", "PASSED": "Photon verdict PASSED THIS RUN, NOT "
                      "YET SAFE"}.get(level, "Photon verdict NOT SAFE YET"))
    if r.get("summary"):
        parts.append(f"{len(r['failed'])} of {r['checks']} checks failed" if r["failed"] else f"{r['checks']} checks passed")
    elif stage.exit is not None:
        parts.append("no summary.md")
    if r.get("stopped"):
        parts.append(f"stopped early: {r['stopped']}")
    if r.get("restoreFailed"):
        parts.append(f"RESTORE FAILED: {r['restoreFailed']}")
    if r.get("dashboardOff"):
        parts.append("it says the dashboard may still be off")
    if stage.exit is not None:
        parts.append(f"runner exit {stage.exit}")
    return "; ".join(parts)


def soak_gate(short, cap_h):
    """Why the go run's soak shouldn't start after this short Photon block, or None (docs/photon-bench.md)."""
    if short is None or short.exit is None:
        return "the short Photon block didn't run"
    results = (short.result or {}).get("results") or {}
    level = dig(results, "photon", "verdict", "level")
    if level != "PASSED":
        return (f"the short Photon block's verdict is {level or 'missing'}, not PASSED THIS RUN: the go run only follows "
                "a short run that passed (docs/photon-bench.md)")
    soak = dig(results, "photon", "main", "soak") or {}
    issued, seconds = soak.get("issuedWrites") or {}, soak.get("seconds")
    rates = {hub: n / seconds for hub, n in issued.items()
             if isinstance(n, (int, float)) and isinstance(seconds, (int, float)) and seconds > 0}
    if not rates:
        return "the short Photon block recorded no soak rate"
    slowest = min(rates.values())
    hours = SOAK_WRITES_NEEDED / slowest / 3600 if slowest > 0 else math.inf
    if hours > cap_h:
        return (f"the short block's soak did {slowest:.0f} writes a second on its slowest hub, so the loss bound's "
                f"{SOAK_WRITES_NEEDED:,} writes per hub would take about {hours:.1f} h, past the {cap_h:g} h cap: the "
                "go run would end NOT SAFE however well Photon behaves (docs/photon-bench.md)")
    return None


def command(flags):
    return "python3 scripts/bench/run_bench.py " + " ".join(shlex.quote(f) for f in flags)


class Session:
    def __init__(self, args, stdin=None, out=None):
        self.args = args
        self.stdin = stdin if stdin is not None else sys.stdin
        self.out = out if out is not None else sys.stdout
        self.popen = subprocess.Popen
        self.lock = threading.Lock()
        self.log_file = None
        self.folder = None
        self.adb = None
        self.adb_port = rb.ADB_SERVER_PORT
        self.identity = None
        self.serialno = None
        self.switch_why = None
        self.usb_rev_hubs = []
        self.original_entry = None
        self.forwards_at_start = set()
        self.expected_eh = None
        self.pinpoint = None
        self.octoquad = None
        self.soak_hours = 0.0
        self.restore_app = False
        self.covered = {CH}
        self.asking = True
        self.interrupted = False
        self.spawning = False
        self.deferred = False
        self.restoring = False
        self.touched = False
        self.dashboard_touched = False
        self.unsafe = None
        self.installed = False
        self.reinstall = None
        self.runner_flags = []
        self.hardware = None
        self.topology = None
        self.eh_serial = None
        self.stages = []
        self.skipped = []
        self.notes = []
        self.needs = []
        self.put_back_notes = []
        self.stopped = None
        self.started = datetime.datetime.now()
        self.ended = None

    def emit(self, text):
        with self.lock:
            if self.log_file is not None:
                self.log_file.write(text + "\n")
                self.log_file.flush()
            try:
                self.out.write(text + "\n")
                self.out.flush()
            except OSError:
                pass

    def say(self, text):
        self.emit(f"[{datetime.datetime.now().strftime('%H:%M:%S')}] {text}")

    def ask(self, question, parse):
        if not self.asking:
            raise RuntimeError(f"a question after the start: {question}")
        while True:
            self.emit(question)
            try:
                self.out.write(">>> ")
                self.out.flush()
            except OSError:
                pass
            line = self.stdin.readline()
            if not line:
                raise Stop("the terminal closed before every question was answered; nothing on the hub was changed")
            answer = line.strip()
            with self.lock:
                self.log_file.write(f">>> {answer}\n")
            value = parse(answer.lower())
            if value is not None:
                return value
            self.emit("    That isn't one of the answers it takes.")

    def ask_yes_no(self, question):
        return self.ask(question + " [y/n]", lambda a: True if a in ("y", "yes") else False if a in ("n", "no") else None)

    def run(self):
        if not self.stdin.isatty():
            print("session.py asks its questions at the start, then runs unattended: start it from a terminal",
                  file=sys.stderr)
            return 2
        stamp = self.started.strftime("%Y-%m-%d_%H%M%S")
        self.folder = Path(self.args.out).resolve() if self.args.out else REPO_ROOT / "bench-results" / f"session-{stamp}"
        if self.folder.exists() and any(self.folder.iterdir()):
            print(f"{self.folder} isn't empty: pass a new --out folder", file=sys.stderr)
            return 2
        self.folder.mkdir(parents=True, exist_ok=True)
        self.log_file = open(self.folder / "console.log", "w")
        handlers = signal.getsignal(signal.SIGINT), signal.getsignal(signal.SIGHUP)
        try:
            self.preflight()
            self.ask_everything()
            self.asking = False
            signal.signal(signal.SIGINT, self.on_sigint)
            signal.signal(signal.SIGHUP, signal.SIG_IGN)
            self.say("no more questions: the rest runs unattended; Ctrl-C stops it and still puts the hub back")
            self.keep_awake()
            self.prepare()
            self.probe()
            self.plan()
            self.run_stages()
        except KeyboardInterrupt:
            self.stopped = "Ctrl-C during the questions, before anything on the hub was changed"
            self.emit("")
        except Interrupted:
            self.stopped = "stopped by Ctrl-C"
            self.say("STOPPED by Ctrl-C: putting the hub back (more Ctrl-C is ignored until that's done)")
        except (Stop, rb.BenchError) as e:
            self.stopped = str(e)
            self.say(f"STOPPED: {e}")
        except Exception as e:
            self.stopped = f"the session raised {type(e).__name__}: {e}"
            self.say(f"STOPPED: {self.stopped}\n{traceback.format_exc()}")
        finally:
            self.restoring = True
            # Ignored, not caught: exec keeps it ignored, so the put-back's own lsof and adb survive a Ctrl-C too.
            signal.signal(signal.SIGINT, signal.SIG_IGN)
            try:
                self.put_back()
            finally:
                self.ended = datetime.datetime.now()
                self.write_index()
                self.finish_message()
                self.log_file.close()
                signal.signal(signal.SIGINT, handlers[0])
                signal.signal(signal.SIGHUP, handlers[1])
        clean = all(s.exit == 0 and not (s.raised or s.timed_out or s.interrupted) for s in self.stages)
        return 0 if clean and not self.stopped and not self.put_back_failed() else 1

    def put_back_failed(self):
        return any("FAILED" in n or "STILL" in n or n.startswith("not touched") for n in self.put_back_notes)

    def on_sigint(self, signum, frame):
        if self.interrupted or self.restoring:
            return
        self.interrupted = True
        if self.spawning:
            self.deferred = True
            return
        raise Interrupted()

    def check_adb_port(self, when):
        try:
            found = rb.adb_port_preflight()
        except rb.BenchError as e:
            self.unsafe = f"adb's server port check failed {when}: {e}"
            raise Stop(self.unsafe)
        return found

    def preflight(self):
        a = self.args
        self.say(f"session folder: {self.folder}")
        found = self.check_adb_port("before the session's first adb command")
        self.adb_port = found["port"]
        self.say(f"adb's server port {found['port']}: "
                 + (", ".join(f"{o['command']} pid {o['pid']}" for o in found["owners"]) or "nothing listening yet"))
        if a.usb:
            self.adb = rb.Adb(None, self.say, usb=True)
            self.adb.serial = rb.pick_usb_device(self.adb.devices(), a.serial)
        else:
            self.adb = rb.Adb(f"{rb.HUB_IP}:{rb.ADB_PORT}", self.say)
        self.adb.ensure_connected()
        prop = self.adb.run("shell", f"getprop {rb.CONTROL_HUB_PROP}").strip()
        if prop not in rb.PROP_TRUE:
            raise Stop(f"{self.adb.serial} isn't a REV Control Hub ({rb.CONTROL_HUB_PROP} is {prop!r}); nothing was "
                       "changed on it")
        if "package:" not in self.adb.run("shell", f"pm path {rb.RC_PACKAGE}", check=False):
            raise Stop(f"the Control Hub at {self.adb.serial} has no Robot Controller app; nothing was changed on it")
        self.serialno = self.read_identity()
        self.identity = self.serialno
        if not self.identity:
            if not a.usb:
                raise Stop("this hub's ro.serialno reads empty, so over Wi-Fi the session couldn't notice another hub "
                           f"answering at {rb.HUB_IP} later: run it with --usb, or run the stages by hand")
            self.identity = f"adb {self.adb.serial}"
            self.say("this hub's ro.serialno reads empty: the same-hub check uses adb's USB serial alone")
        self.say(f"Control Hub {self.adb.serial}, ro.serialno {self.identity}")
        self.usb_rev_hubs = rb.usb_rev_hub_serials(self.adb)
        switch = self.dashboard_switch()
        if switch is False:
            raise Stop("the hub's dashboard is switched off (slothboard's saved autoEnable is false), so every runner "
                       f"would refuse it. {self.adb_port_caution()} Then turn it back on with: adb -s "
                       f"{self.adb.serial} shell '{rb.DASHBOARD_RECOVERY_SHELL}'")
        try:
            self.original_entry = self.adb.active_config_entry()
            self.say(f"the app's active hardware config: {json.dumps(self.original_entry)}")
        except rb.BenchError as e:
            self.say(f"!! {e}: the session can't check the hardware config itself at the end")
        if a.usb:
            self.forwards_at_start = set(self.forwards())

    def adb_port_caution(self):
        return (f"First check that `lsof -nP -iTCP:{self.adb_port} -sTCP:LISTEN` shows nothing or adb: anything else on "
                "adb's port takes the command to another hub.")

    def read_identity(self):
        value = self.adb.run("shell", "getprop ro.serialno").strip()
        return "" if value in ("", "unknown") else value

    def dashboard_switch(self):
        """slothboard's saved switch: True, False, None when unset, or UNREADABLE when run-as can't read it (why in
        switch_why)."""
        r = self.adb._raw(["-s", self.adb.serial, "shell", f"run-as {rb.RC_PACKAGE} cat {rb.DASHBOARD_PREFS}"])
        text = r.stdout if r.returncode == 0 else r.stdout + r.stderr
        if r.returncode != 0 and "No such file" not in text:
            self.switch_why = text.strip()[:160] or f"exit {r.returncode}"
            self.say(f"run-as can't read the dashboard's saved switch: {self.switch_why}")
            return UNREADABLE
        return rb.dashboard_auto_enable(text)

    def forwards(self):
        r = self.adb._raw(["forward", "--list"], timeout=20)
        if r.returncode != 0:
            raise rb.BenchError(f"adb forward --list failed (exit {r.returncode}): "
                                f"{(r.stderr or r.stdout).strip()[:200]}")
        return [tuple(line.split()) for line in r.stdout.splitlines() if len(line.split()) == 3]

    def ask_everything(self):
        a = self.args
        hubs = self.usb_rev_hubs
        if len(hubs) == 1:
            self.expected_eh = "usb"
            self.say(f"an Expansion Hub is on the Control Hub's USB (serial {hubs[0]})")
        elif hubs:
            self.say(f"{len(hubs)} REV hubs are on the Control Hub's USB ({', '.join(hubs)}): the Photon block takes "
                     "exactly one, so it won't run")
        else:
            rs485 = (a.rs485 == "yes") if a.rs485 else self.ask_yes_no(
                "Is a REV Expansion Hub on the Control Hub's RS485 port (address "
                f"{a.expansion_address}), on its own 12 V?")
            self.expected_eh = "rs485" if rs485 else None
        if self.expected_eh:
            self.pinpoint = a.pinpoint or self.ask(
                "Where is the goBILDA Pinpoint plugged in? The Photon block's I2C reads need it in Expansion Hub I2C "
                "port 1; the default suite's Bench: Pinpoint needs it in Control Hub I2C port 1.\n"
                "  [c] Control Hub I2C port 1   [e] Expansion Hub I2C port 1   [n] not plugged in",
                lambda x: {"c": "control-hub", "e": "expansion-hub", "n": "none"}.get(x))
            self.octoquad = (a.octoquad == "yes") if a.octoquad else self.ask_yes_no(
                "Is the OctoQuad in Control Hub I2C port 2?")
            if a.soak_hours is not None:
                self.soak_hours = a.soak_hours
            elif self.go_run_why_not(self.expected_eh) is None and self.ask_yes_no(
                    f"Run the RS485 go run too, its long soak up to {SOAK_CAP_H:g} h (about {SOAK_CAP_H + 2:g}-"
                    f"{SOAK_CAP_H + 3:g} h in all), if the short block passes and its soak rate can reach the loss "
                    "bound? docs/photon-bench.md runs it only once the USB short block has passed too."):
                self.soak_hours = self.ask(f"Cap the soak at how many hours? Enter for {SOAK_CAP_H:g}, the runner's "
                                           "own cap; less can end it before its loss bound.",
                                           lambda x: SOAK_CAP_H if x == "" else self.hours(x))
        if a.install:
            self.restore_app = (a.restore_app == "yes") if a.restore_app else self.ask_yes_no(
                "At the end, put the hub's own app back from the backup (its REINSTALL.sh)?")
        self.covered = {CH} | ({EH} if self.expected_eh else set())
        ports = " and ".join(PORTS[h] for h in (CH, EH) if h in self.covered)
        self.emit(f"\nThe benches drive {ports} with real power: up to 0.35 on motor ports and servo positions "
                  "0.35-0.6. Keep Control Hub I2C bus 3 empty and analog port 0 unwired too.")
        self.ask(f'Type "empty" once nothing is plugged into {ports}:', lambda x: True if x == "empty" else None)
        self.say(f"ports statement: nothing is plugged into {ports}")

    @staticmethod
    def hours(text):
        try:
            value = float(text)
        except ValueError:
            return None
        return value if 0 < value <= SOAK_CAP_H else None

    def keep_awake(self):
        # Its own session, so the Ctrl-C that stops the stages can't end it before the hub is put back.
        self.popen(["caffeinate", "-dims", "-w", str(os.getpid())], stdin=subprocess.DEVNULL,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
        self.say("the Mac stays awake until the session ends (caffeinate)")

    def prepare(self):
        if self.args.install:
            self.install()
            return
        if self.args.usb:
            if self.gradle(offline=False):
                return
            if self.gradle(offline=True):
                self.runner_flags = ["--gradle-offline"]
                self.say("Gradle builds only offline here: every runner gets --gradle-offline")
                return
        elif self.gradle(offline=True):
            self.runner_flags = ["--gradle-offline"]
            return
        self.runner_flags = ["--no-sloth"]
        self.say("Gradle can't build here: the runners skip their Sloth steps (--no-sloth), so the Photon verdict "
                 "can't pass")

    def gradle(self, offline):
        self.say(f"checking that Gradle builds{' offline' if offline else ''}, for the runners' deploySloth steps")
        code, _ = self.run_child(["./gradlew"] + (["--offline"] if offline else []) + ["-q", ":TeamCode:assembleSloth"],
                                 GRADLE_LIMIT_S)
        return code == 0

    def install(self):
        folder = self.folder / "install"
        argv = ["/bin/bash", str(BARE_HUB), "--install-only", str(folder)]
        if self.args.usb:
            argv += ["--usb", "--serial", self.adb.serial]
        else:
            argv += ["--expect-serialno", self.serialno]
        self.check_adb_port("before the install")
        self.adb.ensure_connected()
        self.same_hub()
        self.touched = True
        self.say("backing up the hub's app and installing the stamped bench build (bare_hub.sh)")
        try:
            code, _ = self.run_child(argv, INSTALL_LIMIT_S)
        finally:
            path = folder / "reinstall_path.txt"
            if path.exists():
                self.reinstall = path.read_text().strip()
        info = folder / "installed.json"
        if code != 0 or not info.exists():
            raise Stop(f"bare_hub.sh's install stopped (exit {code}); what it printed is above and in "
                       f"{folder / 'console.log'}")
        installed = json.loads(info.read_text())
        self.installed = True
        self.reinstall = installed["reinstall"]
        self.runner_flags = list(installed["runnerFlags"])
        if installed["originalEntry"]:
            self.original_entry = json.loads(installed["originalEntry"])

    def run_child(self, argv, limit_s, new_session=False):
        """Runs argv with no terminal input and its output in the console; (exit status, timed out). The first Ctrl-C
        reaches it too, and the session waits for it to finish putting the hub back."""
        self.say("$ " + " ".join(shlex.quote(str(x)) for x in argv))
        proc = pump = None
        timed_out = False
        try:
            # A Ctrl-C while the child starts is raised once it has started, so the session can wait for it.
            self.spawning = True
            try:
                proc = self.spawn(argv, new_session)
                pump = threading.Thread(target=self.pump, args=(proc.stdout,), daemon=True)
                pump.start()
            finally:
                self.spawning = False
                if self.deferred:
                    self.deferred = False
                    raise Interrupted()
            try:
                proc.wait(timeout=limit_s)
            except subprocess.TimeoutExpired:
                timed_out = True
                self.say(f"!! still running after {limit_s / 60:.0f} min: sending it Ctrl-C so it puts the hub back")
                proc.send_signal(signal.SIGINT)
                try:
                    proc.wait(timeout=RUNNER_RESTORE_S)
                except subprocess.TimeoutExpired:
                    self.say("!! it didn't end after Ctrl-C either: killing it; the session checks the hub at the end")
                    proc.kill()
                    proc.wait()
        except Interrupted:
            if proc is not None:
                self.say("Ctrl-C: waiting for it to put the hub back and end (more Ctrl-C is ignored)")
                try:
                    proc.wait(timeout=RUNNER_RESTORE_S)
                except subprocess.TimeoutExpired:
                    self.say("!! it didn't end after Ctrl-C: killing it; the session checks the hub itself next")
                    proc.kill()
                    proc.wait()
            raise
        finally:
            if pump is not None and pump.is_alive():
                pump.join(timeout=10)
        return proc.returncode, timed_out

    def spawn(self, argv, new_session):
        kw = dict(cwd=REPO_ROOT, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                  env=dict(os.environ, PYTHONUNBUFFERED="1"), start_new_session=new_session)
        if not new_session:
            return self.popen([str(x) for x in argv], **kw)
        # exec resets a caught signal to default: this child takes the time limit's Ctrl-C while the session ignores it.
        previous = signal.signal(signal.SIGINT, lambda signum, frame: None)
        try:
            return self.popen([str(x) for x in argv], **kw)
        finally:
            signal.signal(signal.SIGINT, previous)

    def pump(self, stream):
        for line in stream:
            self.emit(line.rstrip("\n"))

    def common_flags(self, stage):
        a = self.args
        flags = ["--ports-empty"] + self.link_flags() + self.runner_flags + self.address_flags()
        if a.usb_expansion_serial and stage.key.startswith("photon"):
            flags += ["--usb-expansion-serial", a.usb_expansion_serial]
        entry = self.entry_flag()
        return flags + (["--original-entry", entry] if entry else [])

    def entry_flag(self):
        """The config to restore: from before the install, or after a stage that couldn't restore it, the session's."""
        if self.original_entry is None:
            return None
        if self.installed:
            return json.dumps(self.original_entry)
        try:
            now = self.adb.active_config_entry()
        except rb.BenchError:
            return None
        if now.get("name") == self.original_entry.get("name"):
            return None
        self.say(f"!! the hub's active config is {now.get('name')}, not {self.original_entry.get('name')}: an earlier "
                 "stage couldn't put it back, so this runner is told the original")
        return json.dumps(self.original_entry)

    def wait_for_hub(self, what, app=True):
        deadline = rb.time.monotonic() + HUB_BACK_S
        last = None
        while True:
            try:
                self.adb.ensure_connected()
                if not app:
                    return
                if self.adb.pid() is None:
                    problem = "the Robot Controller app isn't running"
                else:
                    listeners = self.adb.listeners_or_none((rb.DASHBOARD_PORT,))
                    if listeners is None:
                        problem = "the hub's socket table couldn't be read"
                    elif rb.forward_reach(rb.DASHBOARD_PORT, listeners)[0]:
                        return
                    else:
                        problem = "the dashboard isn't listening on the hub's port 8000 (any address or loopback) yet"
            except (rb.BenchError, subprocess.TimeoutExpired) as e:
                problem = str(e)
            if rb.time.monotonic() > deadline:
                raise Stop(f"the hub wasn't back within {HUB_BACK_S:.0f} s {what} ({problem})")
            if problem != last:
                self.say(f"waiting for the hub {what}: {problem}")
                last = problem
            rb.time.sleep(POLL_S)

    def check_safe(self, stage):
        what = f"before {stage.title}"
        self.check_adb_port(what)
        self.wait_for_hub(what)
        self.same_hub()
        uncovered = stage.drives - self.covered
        if uncovered:
            raise Stop(f"{stage.title} drives {' and '.join(PORTS[h] for h in sorted(uncovered))}, which the ports "
                       "statement at the start didn't name; nothing was driven there")
        if stage.key.startswith("photon") and self.topology == "usb":
            serials = rb.usb_rev_hub_serials(self.adb)
            if serials and self.eh_serial not in serials:
                raise Stop(f"the Expansion Hub on the Control Hub's USB is {', '.join(serials)} now, not "
                           f"{self.eh_serial}, whose ports the statement at the start covered")

    def same_hub(self):
        now = self.read_identity() or f"adb {self.adb.serial}"
        if now != self.identity:
            self.unsafe = (f"the hub isn't the one the session started on (ro.serialno {self.identity} then, {now} now), "
                           "so none of the answers given at the start cover it")
            raise Stop(self.unsafe)

    def hub_name(self):
        return f"ro.serialno {self.serialno}" if self.serialno else f"adb serial {self.adb.serial}"

    def run_stage(self, stage):
        if set(stage.flags) & set(PROMPTED_FLAGS):
            raise RuntimeError(f"{stage.title} would stop for a person ({stage.flags}): prompted benches stay out")
        self.check_safe(stage)
        stage.folder = self.folder / f"{len([s for s in self.stages if s.folder]):02d}-{stage.key}"
        argv = [sys.executable, RUNNER] + stage.flags + self.common_flags(stage) + ["--out", stage.folder]
        self.say(f"=== {stage.title} ({stage.about})")
        self.touched = True
        if stage.key == "dashboard_off":
            self.dashboard_touched = True
        t0 = rb.time.monotonic()
        try:
            stage.exit, stage.timed_out = self.run_child(argv, stage.limit_s)
        except Interrupted:
            stage.interrupted = True
            raise
        except Exception as e:
            stage.raised = f"{type(e).__name__}: {e}"
            self.say(f"!! {stage.title}: {stage.raised}\n{traceback.format_exc()}")
        finally:
            stage.seconds = rb.time.monotonic() - t0
            stage.result = stage_results(stage.folder)
            self.say(f"=== {stage.title}: {verdict_line(stage)}")

    def probe(self):
        stage = Stage("probe", "Probe", ["--probe-only"], "about 3 min: what is plugged in", limit_s=1800)
        self.stages.append(stage)
        self.run_stage(stage)
        self.hardware = dig(stage.result, "results", "hardware")
        if self.hardware is None:
            raise Stop(f"the probe didn't record what is plugged in ({verdict_line(stage)}); see "
                       f"{stage.folder / 'summary.md'}")

    def plan(self):
        hw = self.hardware
        a = self.args
        rs485 = hw.get("expansion") is not None
        usb_hubs = hw.get("usbRevHubs") or []
        devices = [d for d, present in (("a webcam", hw.get("webcam")), ("the Limelight", hw.get("limelight")),
                                        (f"an Expansion Hub on RS485 at address {hw.get('expansion')}", rs485))
                   if present]
        minutes = (55, 75) if devices else (45, 90)
        self.stages.append(Stage("default", "Default suite", [], f"about {minutes[0]}-{minutes[1]} min", drives={CH},
                                 limit_s=4 * 3600, minutes=minutes))
        if self.pinpoint == "expansion-hub":
            self.notes.append("The Pinpoint is in Expansion Hub I2C port 1, so the default suite's Bench: Pinpoint finds "
                              "none in Control Hub I2C port 1 and runs Bench: Pinpoint Absent in its place"
                              + (", and Bench: Expansion Hub's Pinpoint reads find nothing there." if rs485 else "."))
            self.needs.append(("pinpoint_ch", "Bench: Pinpoint with the Pinpoint where the robot's code reads it",
                               ["--pinpoint"], "the goBILDA Pinpoint in Control Hub I2C port 1, both pods in and still"))
        if usb_hubs and rs485:
            self.skipped.append(("Photon block", "an Expansion Hub answers on RS485 and a REV hub is on the Control "
                                 "Hub's USB: the Photon block takes one wiring at a time; unplug one"))
        elif len(usb_hubs) > 1:
            self.skipped.append(("Photon block", f"{len(usb_hubs)} REV hubs are on the Control Hub's USB; it takes one"))
        elif usb_hubs or rs485:
            self.topology = "usb" if usb_hubs else "rs485"
            self.eh_serial = usb_hubs[0] if usb_hubs else None
            if self.topology == "usb" and self.eh_serial not in self.usb_rev_hubs:
                raise Stop(f"the probe found a REV hub on the Control Hub's USB ({self.eh_serial}) that wasn't there at "
                           f"the start ({', '.join(self.usb_rev_hubs) or 'none'}), so the ports statement wasn't about "
                           "its ports, which the Photon block drives. Nothing was driven. Check them and start the "
                           "session again")
            if EH not in self.covered:
                raise Stop(f"the probe found an Expansion Hub answering on RS485 at address {hw.get('expansion')}, "
                           "which the answers at the start said wasn't there, so the ports statement didn't name "
                           f"{PORTS[EH]}, which the Photon block drives. Nothing was driven. Check them and start the "
                           "session again")
            self.plan_photon()
        elif self.expected_eh == "rs485":
            self.skipped.append(("Photon block", f"no Expansion Hub answered on RS485 at address {a.expansion_address}"))
        else:
            self.skipped.append(("Photon block", "no Expansion Hub (it needs one, on USB or RS485)"))
        rejoin = ("it waits for the hub on the cable" if a.usb
                  else "the Mac must rejoin the hub's Wi-Fi by itself, so leave auto-join on for it")
        self.stages.append(Stage("dashboard_off", "Bench: Dashboard Off", ["--dashboard-off"],
                                 f"about 10 min; the hub reboots once and {rejoin}", limit_s=2 * 3600,
                                 minutes=(10, 15)))
        if "--no-sloth" in self.runner_flags:
            self.notes.append("Gradle can't build here, so every runner skips its Sloth steps (--no-sloth) and the "
                              "Photon verdict can't pass.")
        self.needs = list(PROMPTED) + self.needs
        self.print_plan(devices)

    def plan_photon(self):
        wiring = {"usb": "--usb-expansion", "rs485": "--photon-rs485"}[self.topology]
        i2c = self.pinpoint == "expansion-hub" and self.octoquad
        flags = ["--photon", wiring] + (["--photon-i2c"] if i2c else [])
        about = {"usb": 55, "rs485": 50}[self.topology] - (0 if i2c else 2)
        self.stages.append(Stage("photon", f"Photon block ({self.topology} wiring)", flags, f"about {about} min",
                                 drives={CH, EH}, limit_s=4 * 3600, minutes=(about, about + 10)))
        if not i2c:
            why = ("the OctoQuad isn't in Control Hub I2C port 2" if not self.octoquad
                   else f"the Pinpoint is {PINPOINT_PLACES[self.pinpoint]}, not in Expansion Hub I2C port 1")
            self.skipped.append(("the Photon block's I2C reads (--photon-i2c)", why))
            self.needs.append(("photon_i2c", "the Photon block's I2C reads, without which its verdict can't say SAFE",
                               ["--photon", wiring, "--photon-i2c"],
                               "the Pinpoint in Expansion Hub I2C port 1 and the OctoQuad in Control Hub I2C port 2, "
                               "nothing else on those ports; about 55 min"))
        why_not = self.go_run_why_not(self.topology)
        if self.soak_hours and why_not is None:
            hours = self.soak_hours
            self.stages.append(Stage("photon_soak", f"Photon go run with the long soak ({self.topology} wiring)",
                                     flags + ["--photon-soak"]
                                     + (["--photon-soak-hours", f"{hours:g}"] if hours < SOAK_CAP_H else []),
                                     f"about {hours + 2:g}-{hours + 3:g} h; only if the short block passes and its soak "
                                     "rate can reach the loss bound within the cap",
                                     drives={CH, EH}, limit_s=(hours + 6) * 3600,
                                     minutes=((hours + 2) * 60, (hours + 3) * 60)))
        else:
            self.skipped.append(("the Photon go run's long soak", why_not or "not asked for at the start"))

    def go_run_why_not(self, topology):
        """Why a soak on this wiring couldn't be the go run, the only Photon run that can say SAFE, or None."""
        if topology == "usb":
            return ("on USB wiring the go run needs its cable pulls (--interactive), so it is in Needs a person "
                    "(photon_replug)")
        if not (self.pinpoint == "expansion-hub" and self.octoquad):
            return ("the go run needs the I2C reads (--photon-i2c): the Pinpoint in Expansion Hub I2C port 1 and the "
                    "OctoQuad in Control Hub I2C port 2")
        return None

    def print_plan(self, devices):
        hw = self.hardware
        self.emit("\n=== Plan")
        self.emit(f"Found: IMU {hw.get('imuType')}; "
                  + (", ".join(devices) if devices else "no webcam, no Limelight, no RS485 Expansion Hub")
                  + (f"; a REV hub on the Control Hub's USB ({', '.join(hw.get('usbRevHubs'))})"
                     if hw.get("usbRevHubs") else "")
                  + (f"; Pinpoint {PINPOINT_PLACES[self.pinpoint]}, OctoQuad "
                     f"{'in Control Hub I2C port 2' if self.octoquad else 'not plugged in'} (your answers)"
                     if self.pinpoint else ""))
        for i, stage in enumerate(s for s in self.stages if s.key != "probe"):
            self.emit(f"{i + 1}. {stage.title}: {stage.about}"
                      + (f"; drives {' and '.join(PORTS[h] for h in (CH, EH) if h in stage.drives)}" if stage.drives
                         else "; drives no port"))
        planned = [s for s in self.stages if s.key != "probe"]
        low, high = (sum(s.minutes[i] for s in planned) / 60 for i in (0, 1))
        self.emit(f"In all about {low:.1f}-{high:.1f} h from now, then putting the hub back.")
        for what, why in self.skipped:
            self.emit(f"- not run: {what}: {why}")
        for note in self.notes:
            self.emit(f"- {note}")
        self.emit(f"- needs a person, so not in this session: {', '.join(n[0] for n in self.needs)} (the commands are in "
                  "the session's summary.md at the end)")
        self.emit("")

    def run_stages(self):
        short = None
        for stage in [s for s in self.stages if s.folder is None]:
            if stage.key == "photon_soak":
                why = soak_gate(short, self.soak_hours)
                if why:
                    self.skipped.append((stage.title, why))
                    self.say(f"not running {stage.title}: {why}")
                    continue
            self.run_stage(stage)
            if stage.key == "photon":
                short = stage

    def put_back(self):
        notes = self.put_back_notes
        if not self.touched:
            notes.append("nothing on the hub was changed")
            return
        if self.unsafe:
            notes.append(f"not touched at the end: {self.unsafe}. Put the hub back by hand once that's sorted: each "
                         "stage's summary.md says what it changed")
            if self.reinstall:
                notes.append(f"the hub's own app: bash {self.reinstall}, on the hub with {self.hub_name()}")
            return
        self.say("putting the hub back")
        try:
            self.check_adb_port("before putting the hub back")
            self.wait_for_hub("before putting the hub back", app=False)
            self.same_hub()
        except (Stop, rb.BenchError) as e:
            notes.append(f"not touched at the end: {e}")
            if self.reinstall:
                notes.append(f"the hub's own app: bash {self.reinstall}, on the hub with {self.hub_name()}")
            return
        for step in (self.put_dashboard_back, self.put_config_back, self.remove_forwards, self.put_app_back):
            try:
                step()
            except Exception as e:
                notes.append(f"FAILED {step.__name__}: {type(e).__name__}: {e}")
                self.say(f"!! {step.__name__} raised {type(e).__name__}: {e}\n{traceback.format_exc()}")

    def put_dashboard_back(self):
        if not self.dashboard_touched:
            return
        switch = self.dashboard_switch()
        if switch is False:
            self.say("the dashboard is still switched off: turning it back on over adb")
            self.adb.run("shell", rb.DASHBOARD_RECOVERY_SHELL, check=False)
            self.wait_for_hub("after turning the dashboard back on")
            switch = self.dashboard_switch()
        if switch is False:
            self.put_back_notes.append(f"dashboard STILL OFF: {self.adb_port_caution()} Then run adb -s "
                                       f"{self.adb.serial} shell '{rb.DASHBOARD_RECOVERY_SHELL}'")
        elif switch == UNREADABLE:
            self.put_back_notes.append(f"dashboard NOT CHECKED: FAILED to read its saved switch ({self.switch_why}). "
                                       f"{self.adb_port_caution()} Then run adb -s {self.adb.serial} shell "
                                       f"'{rb.DASHBOARD_RECOVERY_SHELL}'")
        else:
            self.put_back_notes.append("dashboard on (its saved switch is "
                                       + ("unset" if switch is None else "on") + ")")

    def put_config_back(self):
        if self.original_entry is None:
            self.put_back_notes.append("hardware config not checked: run-as couldn't read the app's settings at the "
                                       "start; each stage's summary.md has its Restore line")
            return
        name = self.original_entry.get("name")
        if self.config_is_back(name):
            self.put_back_notes.append(f"hardware config {name} active, no bench_* config left")
            return
        try:
            self.wait_for_hub("before putting the config back")
        except Stop as e:
            self.put_back_notes.append(f"hardware config FAILED to come back to {name}: it isn't active or a bench_* "
                                       f"config is left, and the probe-only runner that puts it back needs the app: {e}")
            return
        self.say(f"the hub isn't back on {name} with no bench_* config: a probe-only runner puts it back")
        stage = Stage("restore", "Restore", ["--probe-only"], "about 3 min", limit_s=1800)
        stage.folder = self.folder / "99-restore"
        self.stages.append(stage)
        argv = ([sys.executable, RUNNER, "--probe-only", "--ports-empty"] + self.link_flags() + self.runner_flags
                + self.address_flags() + ["--original-entry", json.dumps(self.original_entry), "--out", stage.folder])
        stage.exit, _ = self.run_child(argv, stage.limit_s, new_session=True)
        stage.result = stage_results(stage.folder)
        if self.config_is_back(name):
            self.put_back_notes.append(f"hardware config {name} active again after the restore run ({stage.folder.name})")
        else:
            self.put_back_notes.append(f"hardware config FAILED to come back to {name}: {verdict_line(stage)}; its "
                                       f"summary.md says how to switch it back by hand ({stage.folder.name})")

    def link_flags(self):
        return ((["--usb", "--serial", self.adb.serial] if self.args.usb else [])
                + (["--hub-serialno", self.serialno] if self.serialno else []))

    def address_flags(self):
        a = self.args
        return ["--expansion-address", str(a.expansion_address)] if a.expansion_address != 2 else []

    def config_is_back(self, name):
        active = self.adb.active_config_entry().get("name")
        left = [n for n in rb.BENCH_CONFIGS if n != name and n in self.adb.config_names()]
        return active == name and not left

    def remove_forwards(self):
        if not self.args.usb:
            return
        hub_ports = {f"tcp:{p}" for p in rb.USB_HUB_PORTS}
        left = [row for row in self.forwards()
                if row[0] == self.adb.serial and row[2] in hub_ports and row not in self.forwards_at_start]
        kept = []
        for serial, local, remote in left:
            r = self.adb._raw(["-s", serial, "forward", "--remove", local], timeout=20)
            if r.returncode != 0:
                kept.append(f"{local} ({(r.stderr or r.stdout).strip()[:160]})")
            self.say(f"removing a runner's leftover forward {local} -> {remote}: exit {r.returncode}")
        self.put_back_notes.append(f"adb forwards: {len(left)} left by a runner, "
                                   + (f"FAILED to remove {', '.join(kept)}" if kept else "removed") if left
                                   else "adb forwards: none left by the runners")

    def put_app_back(self):
        if not self.reinstall:
            return
        if not self.restore_app:
            self.put_back_notes.append(f"the hub keeps the bench build, as asked; its own app goes back with: bash "
                                       f"{self.reinstall}")
            return
        self.say("putting the hub's own app back (REINSTALL.sh)")
        code, _ = self.run_child(["/bin/bash", self.reinstall], REINSTALL_LIMIT_S, new_session=True)
        self.put_back_notes.append(f"the hub's own app put back (REINSTALL.sh exit {code})" if code == 0
                                   else f"REINSTALL.sh FAILED (exit {code}): run bash {self.reinstall} again")

    def write_index(self):
        a = self.args
        serial = self.adb.serial if self.adb is not None else None
        link = (f"USB cable (adb {serial or 'no device picked'})" if a.usb
                else f"Wi-Fi (adb {rb.HUB_IP}:{rb.ADB_PORT})")
        lines = [f"# Bench session {self.folder.name}\n",
                 f"Command: `{' '.join(shlex.quote(x) for x in sys.argv)}`\n",
                 f"Link: {link}; hub ro.serialno {self.identity}\n",
                 f"Started {self.started:%Y-%m-%d %H:%M:%S}, ended {self.ended:%Y-%m-%d %H:%M:%S}\n",
                 f"**Stopped: {self.stopped}**\n" if self.stopped else "Every planned stage ran.\n"]
        if self.hardware:
            octoquad = {True: "in Control Hub I2C port 2", False: "not plugged in"}.get(self.octoquad, "not asked")
            lines.append(f"Hardware the probe found: {json.dumps(self.hardware)}. Answers: Pinpoint "
                         f"{PINPOINT_PLACES.get(self.pinpoint, 'not asked')}, OctoQuad {octoquad}, soak "
                         + (f"capped at {self.soak_hours:g} h" if self.soak_hours else "not asked for") + "\n")
        lines += ["## Stages\n", "| # | stage | verdict | summary |", "|---|---|---|---|"]
        for stage in self.stages:
            where = (str(Path(stage.result["summary"]).relative_to(self.folder))
                     if stage.result and stage.result.get("summary") else "-")
            lines.append(f"| {stage.folder.name if stage.folder else '-'} | {stage.title} | {verdict_line(stage)} | "
                         f"{where} |")
        lines.append("")
        for stage in self.stages:
            failed = (stage.result or {}).get("failed") or []
            if failed:
                lines.append(f"### {stage.title}: failed checks\n")
                lines += [f"- {f[:400]}" for f in failed]
                lines.append("")
        if self.skipped:
            lines.append("## Not run\n")
            lines += [f"- {what}: {why}" for what, why in self.skipped]
            lines.append("")
        lines.append("## Put back\n")
        lines += [f"- {n}" for n in self.put_back_notes]
        lines.append("")
        if self.needs:
            lines.append("## Needs a person\n")
            lines.append("Each runs on its own, from the repo root, once its hardware is on the table and the bench "
                         "ports are empty:\n")
            for key, what, flags, have in self.needs:
                lines.append(f"- **{what}** ({key}). Have: {have}.\n  `{command(self.later_flags(flags))}`")
            lines.append("")
        lines.append("## By hand afterwards\n")
        lines += [f"- {t}" for t in self.by_hand()]
        lines.append("")
        lines.append(f"The whole console is in console.log; each stage's folder has its own summary.md, run.log, raw/ "
                     "and logs/.")
        (self.folder / "summary.md").write_text("\n".join(lines) + "\n")

    def later_flags(self, flags):
        link = []
        if self.args.usb:
            link = ["--usb"] + (["--serial", self.adb.serial] if self.adb is not None and self.adb.serial else [])
        return list(flags) + ["--ports-empty"] + link + self.address_flags()

    def by_hand(self):
        out = []
        if self.pinpoint == "expansion-hub":
            out.append("move the Pinpoint back to Control Hub I2C port 1")
        if self.reinstall and not (self.restore_app and any("own app put back" in n for n in self.put_back_notes)):
            out.append(f"to put the hub's own app back: bash {self.reinstall}")
        if any("FAILED" in n or "STILL" in n or n.startswith("not touched") for n in self.put_back_notes):
            out.append("what the Put back section above says FAILED or wasn't touched")
        out.append("plug back what you took off the bench ports before the hub goes on a robot")
        if self.needs:
            out.append("the Needs a person runs, when there is someone at the hub")
        out.append("tell Claude the session finished" if self.args.usb
                   else "reconnect the Mac to the internet and tell Claude the session finished")
        return out

    def finish_message(self):
        self.emit("")
        self.emit(f"Session summary: {self.folder / 'summary.md'}")
        for note in self.put_back_notes:
            self.emit(f"- {note}")
        if self.stopped:
            self.emit(f"The session stopped: {self.stopped}")
        self.emit(("Tell Claude" if self.args.usb else "Reconnect to the internet and tell Claude")
                  + " the session " + ("stopped." if self.stopped else "finished."))


def parse_args(argv):
    p = argparse.ArgumentParser(description="Start it once: every bench stage the plugged-in hardware allows, "
                                            "unattended after the questions (scripts/bench/README.md)")
    p.add_argument("--usb", action="store_true", help="reach the Control Hub over a USB cable (run_bench.py --usb)")
    p.add_argument("--serial", default=None, help="with --usb: the hub's serial when adb lists more than one USB device")
    p.add_argument("--install", action="store_true",
                   help="first back up the hub's app and install the stamped bench build, as bare_hub.sh does (run "
                        "scripts/bench/bare_hub.sh --build first)")
    p.add_argument("--expansion-address", type=int, default=2, help="the RS485 Expansion Hub's address (default 2)")
    p.add_argument("--usb-expansion-serial", default=None,
                   help="the USB Expansion Hub's serial, if the runner can't read it from the Control Hub")
    p.add_argument("--rs485", choices=("yes", "no"), default=None,
                   help="answer: an Expansion Hub is on RS485 (asked only when none is on the Control Hub's USB)")
    p.add_argument("--pinpoint", choices=tuple(PINPOINT_PLACES), default=None,
                   help="answer: where the Pinpoint is plugged in (asked only with an Expansion Hub)")
    p.add_argument("--octoquad", choices=("yes", "no"), default=None,
                   help="answer: the OctoQuad is in Control Hub I2C port 2 (asked only with an Expansion Hub)")
    p.add_argument("--soak-hours", type=float, default=None,
                   help=f"answer: the long Photon soak's cap in hours (at most {SOAK_CAP_H:g}), 0 for none")
    p.add_argument("--restore-app", choices=("yes", "no"), default=None,
                   help="answer, with --install: put the hub's own app back at the end")
    p.add_argument("--out", default=None, help="the session folder (default bench-results/session-<time>)")
    args = p.parse_args(argv)
    if args.serial and not args.usb:
        p.error("--serial names the hub's USB serial: use it with --usb")
    if args.soak_hours is not None and not 0 <= args.soak_hours <= SOAK_CAP_H:
        p.error(f"--soak-hours must be 0 to {SOAK_CAP_H:g}")
    if args.restore_app and not args.install:
        p.error("--restore-app answers a question only --install asks")
    if not 1 <= args.expansion_address <= rb.EXPANSION_MAX_ADDRESS:
        p.error(f"--expansion-address must be 1-{rb.EXPANSION_MAX_ADDRESS}")
    return args


def main(argv=None):
    return Session(parse_args(sys.argv[1:] if argv is None else argv)).run()


if __name__ == "__main__":
    sys.exit(main())
