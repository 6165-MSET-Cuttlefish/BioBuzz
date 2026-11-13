"""The runner's default step list, end to end, against fake_hub.py's Control Hub: its adb calls, its sockets and its
config restore, with no hub, no adb and no network (about 4 min). Run with:
python3 -I scripts/bench/offline/dryrun_default.py

    --record DIR    write the Wi-Fi runs' adb calls (a bare hub, and one with a webcam, the Limelight and an Expansion
                    Hub, whose sockets and HTTP requests are recorded too) and the step selections to DIR (how fixtures/
                    was made, from the runner before the USB link: `git show ea69f20:scripts/bench/run_bench.py`)
    --runner PATH   drive that copy of run_bench.py instead of this checkout's
"""
import argparse
import contextlib
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
WORKTREE = HERE.parents[2]
OUT = HERE / "out" / "default"
FIXTURES = HERE / "fixtures"
CALLS = "wifi_default_adb_calls.txt"
SELECTIONS = "step_selections.json"
EVERYTHING = "wifi_everything.json"
NONCE = re.compile(r'"nonce": "[0-9-]+"')
POLL = re.compile(r"^adb devices$| shell pidof | shell logcat -d ")
HUB_TIME = re.compile(r"'(?:\d{4}-)?\d\d-\d\d \d\d:\d\d:\d\d\.\d{3}'")
SELECTION_ARGV = ([], ["--all"], ["--hub", "--loop"], ["--photon"], ["--turret-zero"], ["--dashboard-off"], ["--no-sloth"],
                  ["--all", "--no-sloth"], ["--webcam"], ["--photon", "--hub"])

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


def runner_module(path):
    rb = load("run_bench", path)
    rb.REPO_ROOT = WORKTREE
    return rb


@contextlib.contextmanager
def quiet():
    console = io.StringIO()
    handlers = signal.getsignal(signal.SIGINT), signal.getsignal(signal.SIGHUP)
    stdin = sys.stdin
    sys.stdin = io.StringIO("")
    try:
        with contextlib.redirect_stdout(console):
            yield console
    finally:
        sys.stdin = stdin
        signal.signal(signal.SIGINT, handlers[0])
        signal.signal(signal.SIGHUP, handlers[1])


def default_run(rb, argv=(), hub=None):
    """Runner.run() with no bench flags; returns (hub, runner, exit status, console)."""
    hub = hub or fake_hub.Hub()
    OUT.mkdir(parents=True, exist_ok=True)
    out = pathlib.Path(tempfile.mkdtemp(prefix="default-", dir=OUT))
    with hub.installed(rb), quiet() as console:
        runner = rb.Runner(rb.parse_args(["--out", str(out / "run"), "--ports-empty", *argv]))
        code = runner.run()
    return hub, runner, code, console.getvalue()


def normalized(hub, runner):
    out = str(runner.out)
    return [call.replace(out, "<out>") for call in hub.calls]


def same_calls(label, now, recorded):
    """In order and byte for byte, polls aside: how many `adb devices`, pidof and logcat reads a wait makes follows how
    fast the fake hub answers on a loaded Mac. A kind of poll the recorded run never made still fails."""
    a, b = [c for c in now if not POLL.search(c)], [c for c in recorded if not POLL.search(c)]
    expect(label, a == b)
    if a != b:
        at = next((i for i, (x, y) in enumerate(zip(a, b)) if x != y), min(len(a), len(b)))
        print(f"  first difference at call {at + 1} of those: now {a[at:at + 1]}, recorded {b[at:at + 1]}")
    shapes = (lambda calls: {HUB_TIME.sub("'<time>'", c) for c in calls if POLL.search(c)})
    new = shapes(now) - shapes(recorded)
    expect(f"{label}: no kind of poll the recorded run didn't make", not new)
    if new:
        print(f"  new polls: {sorted(new)[:3]}")


def benches_entered(runner):
    text = (runner.out / "run.log").read_text()
    return [line.split("=== ", 1)[1].split(" ===")[0] for line in text.splitlines() if "] === " in line]


def selections(rb, extra=()):
    """{argv and hardware: the benches selection() picks, or the error it raises}, without a hub."""
    out = {}
    for argv in SELECTION_ARGV:
        for hardware in ("bare", "everything"):
            r = object.__new__(rb.Runner)
            r.args = rb.parse_args(list(argv) + list(extra))
            r.summary, r.log = [], (lambda text: None)
            present = hardware == "everything"
            r.expansion, r.webcam, r.limelight = (2, "UC1", {"ipAddress": "172.29.0.1"}) if present else (None,) * 3
            try:
                out[f"{' '.join(argv) or '(none)'} | {hardware}"] = sorted(r.selection())
            except rb.BenchError as e:
                out[f"{' '.join(argv) or '(none)'} | {hardware}"] = f"error: {e}"
    return out


def everything_record(rb):
    """The Wi-Fi run on a hub with a webcam, the Limelight and an Expansion Hub: its adb calls (each run's nonce taken
    out), sockets, HTTP requests and benches."""
    hub, runner, code, _ = default_run(rb, hub=fake_hub.Hub(everything=True))
    return json.loads(json.dumps({"adb": [NONCE.sub('"nonce": "<nonce>"', c) for c in normalized(hub, runner)],
                                  "tcp": hub.tcp, "http": hub.http, "benches": benches_entered(runner)}))


def wifi_checks(rb):
    hub, runner, code, console = default_run(rb)
    wanted = [n for n in rb.BENCHES if n in selections(rb)["(none) | bare"]]
    expect("wifi: every default bench ran, in the runner's order", benches_entered(runner) == wanted)
    expect("wifi: the run ended and wrote its summary", (runner.out / "summary.md").exists() and "Nothing to send back"
           in console)
    expect("wifi: the original config is active again and no bench config is left",
           hub.active["name"] == "betta_decode" and not [n for n in hub.configs if n.startswith("bench_")])
    expect("wifi: every socket went to the hub's 192.168.43.1:8000",
           hub.tcp and {(h, p) for h, p, _ in hub.tcp} == {(fake_hub.HUB_IP, 8000)})
    expect("wifi: deploySloth ran with ANDROID_SERIAL at the hub's Wi-Fi serial",
           hub.deploys and all(d["ANDROID_SERIAL"] == fake_hub.WIFI_SERIAL for d in hub.deploys))
    expect("wifi: the fake hub knew every adb command", not hub.unknown)
    return hub, runner


def other_hub(rb):
    """--hub-serialno (session.py's): another hub answering at 192.168.43.1 after a reboot gets nothing more, not even
    the config put back; one answering at connect gets nothing at all; this hub's run is the run without the flag."""
    class RebootOnly(rb.Runner):
        def run_dashboard_off(self):
            rec = self.results["dashboard_off"] = {"planned": ["reboot"], "phases": [], "limelight": False}
            try:
                self._dash_off_phase(rec, "reboot")
            finally:
                self._dash_off_recover(rec)

    hub, seen = fake_hub.Hub(), {}

    def another():
        hub.hub_serial = "OTHERHUB0002"
        seen.update(calls=len(hub.calls), tcp=len(hub.tcp))

    hub.on_present = another
    OUT.mkdir(parents=True, exist_ok=True)
    with hub.installed(rb), quiet():
        r = RebootOnly(rb.parse_args(["--out", str(pathlib.Path(tempfile.mkdtemp(prefix="other-", dir=OUT)) / "run"),
                                      "--ports-empty", "--dashboard-off", "--hub-serialno", "FAKEHUB0001"]))
        code = r.run()
    summary = (r.out / "summary.md").read_text()
    said = "a different hub answered at 192.168.43.1:5555 (ro.serialno OTHERHUB0002, not FAKEHUB0001); nothing more " \
           "was sent to it"
    expect("another hub after Bench: Dashboard Off's reboot: after its ro.serialno, not one adb command or socket",
           code == 1 and hub.calls[seen["calls"]:] == [f"adb -s {fake_hub.WIFI_SERIAL} shell getprop ro.serialno"]
           and len(hub.tcp) == seen["tcp"])
    expect("another hub after the reboot: no recovery and no config put back there, and summary.md says why",
           hub.active["name"] == rb.CONFIG_MAIN and said in summary and f"Restore: not put back: {said}" in summary
           and said in r.results["dashboard_off"]["recovery"].get("error", ""))

    hub = fake_hub.Hub()
    hub, runner, code, _ = default_run(rb, ["--hub-serialno", "SOMEOTHER9"], hub=hub)
    expect("--hub-serialno naming another hub: stops at connect after reading its ro.serialno, changing nothing",
           code == 1 and hub.calls[-1].endswith("shell getprop ro.serialno")
           and not [c for c in hub.calls if " push " in c or "run-as" in c or " am " in c]
           and "ro.serialno FAKEHUB0001, not SOMEOTHER9" in (runner.out / "summary.md").read_text())

    class Restore(rb.Runner):
        def restore_config(self):
            seen["restore"] = len(hub.calls)
            return super().restore_config()

    hub = fake_hub.Hub()
    with hub.installed(rb), quiet():
        r = Restore(rb.parse_args(["--out", str(pathlib.Path(tempfile.mkdtemp(prefix="other-", dir=OUT)) / "run"),
                                   "--ports-empty", "--hub", "--hub-serialno", "FAKEHUB0001"]))
        r.run()
    checks = [i for i, c in enumerate(hub.calls) if c.endswith("shell getprop ro.serialno")]
    expect("--hub-serialno naming this hub: the run ends with the config put back, reading ro.serialno after every "
           "adb connection check, the restore's first included",
           hub.active["name"] == "betta_decode" and not [n for n in hub.configs if n.startswith("bench_")]
           and checks and all(hub.calls[i - 1] == "adb devices" for i in checks)
           and seen["restore"] - 1 in checks and "different hub" not in (r.out / "summary.md").read_text())


def wifi_link(rb):
    link = rb.WifiLink(rb.HUB_IP)
    expect("Wi-Fi link: the dashboard page, the RC page and the restore by hand are all at 192.168.43.1:8080",
           link.dash_page() == ("http://192.168.43.1:8080/dash", None)
           and link.rc_page() == ("http://192.168.43.1:8080", None)
           and "open http://192.168.43.1:8080/dash, add the Hardware Config view, and activate betta_decode"
           in link.restore_by_hand(fake_hub.WIFI_SERIAL, "", "betta_decode"))


def connects(hub):
    return [c for c in hub.calls if " connect " in f" {c} " or " disconnect" in c]


def through_forwards(hub):
    """Every socket the runner opened went to a port fake adb forwarded to this hub (and none elsewhere)."""
    return hub.tcp and all(w.startswith("forward to ") for _, _, w in hub.tcp)


def usb_default(rb):
    foreign = [("R58M90ABCDE", 9999, 9999), (fake_hub.USB_SERIAL, 9998, 8000)]
    hub = fake_hub.Hub(usb=True, everything=True, foreign_forwards=foreign)
    hub, runner, code, console = default_run(rb, ["--usb"], hub=hub)
    wanted = [n for n in rb.BENCHES if n in selections(rb)["(none) | everything"]]
    summary = (runner.out / "summary.md").read_text()
    expect("usb: every default bench ran, webcam, Limelight and Expansion Hub ones included, in the runner's order",
           benches_entered(runner) == wanted and {"webcam", "limelight", "expansion"} <= set(wanted))
    expect("usb: the run ended, wrote its summary and put the original config back",
           "Nothing to send back" in console and hub.active["name"] == "betta_decode"
           and not [n for n in hub.configs if n.startswith("bench_")])
    expect("usb: not one adb connect or disconnect", not connects(hub))
    expect("usb: every socket went to a port adb forwarded to this hub's port, never 192.168.43.1",
           through_forwards(hub) and not [t for t in hub.tcp if t[0] == fake_hub.HUB_IP])
    expect("usb: the Limelight proxy's HTTP went through a forward to 5807",
           any(remote == 5807 and line.startswith("POST /pipeline-switch") for remote, line in hub.http)
           and any(w == "forward to 5807" for _, _, w in hub.tcp))
    first_push = next(i for i, c in enumerate(hub.calls) if " push " in c)
    expect("usb: it checked the device is a Control Hub with the RC app before changing anything",
           any("getprop persist.ftcandroid.serialasusb" in c for c in hub.calls[:first_push])
           and any("pm path com.qualcomm.ftcrobotcontroller" in c for c in hub.calls[:first_push]))
    expect("usb: the forwards it made are gone at the end, the two it didn't make are still there",
           [(f.serial, f.local, f.remote) for f in hub.forwards] == foreign)
    expect("usb: it removed only forwards it made, one by one, never --remove-all",
           not [c for c in hub.calls if "--remove-all" in c]
           and all(c.split("tcp:")[-1] not in ("9999", "9998") for c in hub.calls if "--remove" in c))
    expect("usb: deploySloth ran with ANDROID_SERIAL at the cable's serial, the device listed, so the Load plugin "
           "doesn't adb connect", hub.deploys and all(d["ANDROID_SERIAL"] == fake_hub.USB_SERIAL
                                                      and d["listedDevices"] for d in hub.deploys))
    expect("usb: every deploySloth that ended by itself was left alone", hub.gradle_procs
           and not [p for p in hub.gradle_procs if p.killed])
    expect("usb: summary.md says the run used the cable and which forwards it made",
           "Link: USB cable (adb " + fake_hub.USB_SERIAL in summary and "removed at the end:" in summary)
    expect("usb: summary.md judges each hub port from the hub's own socket table", "8000 yes (listens on any)" in summary)


def usb_runner(rb, hub, argv=()):
    OUT.mkdir(parents=True, exist_ok=True)
    out = pathlib.Path(tempfile.mkdtemp(prefix="usb-", dir=OUT))
    return rb.Runner(rb.parse_args(["--out", str(out / "run"), "--ports-empty", "--usb", *argv]))


def usb_reboot(rb):
    """Bench: Dashboard Off's adb reboot: the device leaves adb with its forwards, the runner waits for it on the cable,
    then makes new forwards before its next socket."""
    hub = fake_hub.Hub(usb=True)
    with hub.installed(rb), quiet():
        r = usb_runner(rb, hub)
        r.connect()
        before = len(hub.tcp)
        rec = {"phases": [], "limelight": True}
        r._dash_off_phase(rec, "reboot")
        events = list(r.link.events)
        r._dash_off_recover(rec)
        r.dash.close()
        r.link.close()
    made8000 = [e["local"] for e in events if e.get("made") == 8000]
    after = hub.tcp[before:]
    expect("reboot: the hub left adb and came back on the cable", hub.reboots == 1 and rec["phases"][0].get("backSeconds"))
    expect("reboot: the runner found the dashboard forward gone and made a new one",
           any(e.get("gone") == 8000 for e in events) and len(made8000) == 2)
    expect("reboot: every socket after it went through the new forwards (the dashboard, the second socket, the "
           "RC page on 8080 and the Limelight UI on 5801)", after and all(w.startswith("forward to ") for _, _, w in after)
           and {w for _, _, w in after} >= {"forward to 8000", "forward to 8080", "forward to 5801"}
           and all(p == made8000[1] for _, p, w in after if w == "forward to 8000"))
    expect("reboot: not one adb connect", not connects(hub))
    expect("reboot: the recovery's checks saw the dashboard back over the cable",
           rec["recovery"].get("statusEnabled") is True and rec["recovery"].get("limelightHttp", {}).get("code") == 200)
    expect("reboot: the forwards it made are all removed at the end, and it didn't try to remove the gone one",
           not hub.forwards and not [c for c in hub.calls if c.endswith(f"--remove tcp:{made8000[0]}")]
           or made8000[0] == made8000[1])


def usb_cable_pull(rb):
    """A cable pulled and plugged back between steps: the runner waits for the device as for a hub lost in Bench:
    Turret Zero's dip, then makes the dashboard's forward again before its next request."""
    hub = fake_hub.Hub(usb=True)
    with hub.installed(rb), quiet():
        r = usb_runner(rb, hub)
        r.connect()
        before = len(hub.tcp)
        first = r.link.ours[8000]
        hub.disappear(4)
        r.wait_hub_back("a pulled cable")
        status = r.dash.get_status()
        events = list(r.link.events)
        r.dash.close()
        r.link.close()
    after = hub.tcp[before:]
    second = [e["local"] for e in events if e.get("made") == 8000][-1]
    expect("cable pull: the device came back and the runner read a status through a new forward",
           hub.reboots == 1 and status.get("enabled") is True and any(e.get("gone") == 8000 for e in events))
    expect("cable pull: every socket after it went to the new forward, none to the old port",
           after and all(w == "forward to 8000" and p == second for _, p, w in after)
           and (second == first or not [t for t in after if t[1] == first]))
    expect("cable pull: not one adb connect", not connects(hub))
    expect("cable pull: the forwards it made are removed at the end", not hub.forwards)


def usb_stops(rb):
    for hub, argv, words, label in (
            (fake_hub.Hub(usb=True, extra_usb=["R58M90ABCDE"]), [], "adb lists 2 USB devices",
             "two USB devices and no --serial"),
            (fake_hub.Hub(usb=True, extra_usb=["R58M90ABCDE"]), ["--serial", "NOPE123"], "doesn't list NOPE123",
             "a --serial adb doesn't list"),
            (fake_hub.Hub(usb=True), ["--serial", fake_hub.WIFI_SERIAL], "not the network address",
             "a network --serial"),
            (fake_hub.Hub(usb=False), [], "adb lists no USB device", "no USB device")):
        with hub.installed(rb), quiet():
            code = usb_runner(rb, hub, argv).run()
        summary = (pathlib.Path(sorted(OUT.glob("usb-*"), key=lambda q: q.stat().st_mtime)[-1]) / "run/summary.md")
        expect(f"usb, {label}: stops before any device command",
               code == 1 and hub.calls == ["adb devices"] and words in summary.read_text())
        expect(f"usb, {label}: summary.md's link line says no device was picked",
               "Link: USB cable (stopped before picking a device" in summary.read_text())
    hub = fake_hub.Hub(usb=True, extra_usb=["R58M90ABCDE"])
    with hub.installed(rb), quiet():
        code = usb_runner(rb, hub, ["--list-configs", "--serial", fake_hub.USB_SERIAL]).run()
    expect("usb, two USB devices and --serial naming one: uses that one, without adb connect",
           code in (0, 1) and any(c.startswith(f"adb -s {fake_hub.USB_SERIAL} shell ls -l") for c in hub.calls)
           and not [c for c in hub.calls if "R58M90ABCDE" in c] and not connects(hub))
    hub = fake_hub.Hub(usb=True, control_hub=False)
    with hub.installed(rb), quiet():
        code = usb_runner(rb, hub).run()
    expect("usb, an Android device that isn't a Control Hub: stops after reading its properties, changing nothing",
           code == 1 and hub.calls[-1].endswith("getprop persist.ftcandroid.serialasusb")
           and not [c for c in hub.calls if " push " in c or " am " in c])


def usb_unjudged(rb):
    wifi_only = "012BA8C0"
    reach = rb.forward_reach
    expect("forward reach: an unreadable socket table is not judged",
           reach(8000, None)[0] is None and "not judged over USB" in reach(8000, None)[1])
    expect("forward reach: any address and loopback are reached, a Wi-Fi-only one and an absent one aren't",
           reach(8000, [{"port": 8000, "address": "any"}])[0] is True
           and reach(8000, [{"port": 8000, "address": "0100007F"}])[0] is True
           and reach(8000, [{"port": 8000, "address": wifi_only}])[0] is False
           and reach(8000, [{"port": 5807, "address": "any"}])[0] is False)
    hub = fake_hub.Hub(usb=True, socket_table=False)
    with hub.installed(rb), quiet():
        r = usb_runner(rb, hub)
        r.connect()
        r.write_summary()
        r.dash.close()
        r.link.close()
    summary = (r.out / "summary.md").read_text()
    expect("unreadable socket table: summary.md says each hub port is not judged, never yes",
           "8000 not judged (not judged over USB" in summary and "8000 yes" not in summary)

    class Limelight(rb.Runner):
        def run_opmode(self, opmode, step, params=None, during=None, **kw):
            self.dash.data = {rb.KEY_PHASE: "proxyQuery", rb.KEY_STATUS: "ready"}
            self.watch = {"stopSent": True}
            during()
            self.results["limelight"] = {}
            return {"crashed": False, "errorMessage": ""}

        def pull(self, name, required=True, local_name=None):
            return {"phases": {"proxyQuery": {}}}

    hub = fake_hub.Hub(usb=True, socket_table=False, http_answers=False)
    with hub.installed(rb), quiet():
        OUT.mkdir(parents=True, exist_ok=True)
        r = Limelight(rb.parse_args(["--out", str(pathlib.Path(tempfile.mkdtemp(prefix="usb-", dir=OUT)) / "run"),
                                     "--usb", "--limelight", "--ports-empty"]))
        r.connect()
        r.run_limelight()
        r.dash.close()
        r.link.close()
    check = next(c for c in r.checks if c[0].startswith("limelight proxy: a POST"))
    expect("Limelight proxy, no HTTP answer through the forward and an unreadable socket table: FAIL saying not judged "
           "over USB, not that the proxy isn't listening", check[1] == "FAIL" and "not judged over USB" in check[3]
           and "isn't listening there" not in check[3])


def usb_interrupted(rb):
    """Ctrl-C in the first bench: the runner's finally puts the config back and removes its own forwards."""
    class FirstBenchInterrupted(rb.Runner):
        def guarded(self, name, step):
            raise KeyboardInterrupt

    foreign = [("R58M90ABCDE", 9999, 9999), (fake_hub.USB_SERIAL, 9998, 8000)]
    hub = fake_hub.Hub(usb=True, foreign_forwards=foreign)
    raised = False
    with hub.installed(rb), quiet():
        r = FirstBenchInterrupted(rb.parse_args(["--out", str(pathlib.Path(tempfile.mkdtemp(prefix="usb-", dir=OUT))
                                                             / "run"), "--ports-empty", "--usb"]))
        try:
            r.run()
        except KeyboardInterrupt:
            raised = True
    link = next(line for line in (r.out / "summary.md").read_text().splitlines() if line.startswith("Link: "))
    expect("usb, Ctrl-C in the first bench: the runner put the original config back, left no bench config and "
           "re-raised", raised and hub.active["name"] == "betta_decode"
           and not [n for n in hub.configs if n.startswith("bench_")])
    expect("usb, Ctrl-C in the first bench: only the forwards it made are gone, and summary.md's Link line says which",
           [(f.serial, f.local, f.remote) for f in hub.forwards] == foreign
           and re.search(r"removed at the end: \d+", link) and "9998" not in link.split("removed at the end:")[1])


def usb_rc_page(rb):
    """The RC page's second forward: a hung adb skips only that step, and page ports given up are discarded, not
    reported as removed at the end."""
    hub = fake_hub.Hub(usb=True)
    with hub.installed(rb), quiet():
        r = usb_runner(rb, hub, ["--clock", "--interactive"])
        r.connect()
        hub.clock, hub.hang_no_rebind = 3600, True
        r.run_clock()
        events = list(r.link.events)
        r.dash.close()
        r.link.close()
        describe = r.link.describe(hub.serial)
    summary = "\n".join(r.summary)
    discarded = [e["local"] for e in events if "discarded" in e]
    expect("rc page, adb hanging on the websocket's forward: Bench: Clock still runs, saying the page step was skipped",
           "the RC web page step was skipped: adb -s" in summary and "didn't answer within 20 s" in summary
           and any(c[0].startswith("clock: PathCommands.timeout") for c in r.checks)
           and not [c for c in r.checks if "raised" in c[2]])
    expect("rc page, adb hanging: the page's own forward is discarded at once, and the Link line says so, apart from "
           "the forwards removed at the end",
           len(discarded) == 1 and not [f for f in hub.forwards if f.local == discarded[0]]
           and f"discarded unused page ports: {discarded[0]}" in describe
           and str(discarded[0]) not in describe.split("removed at the end:")[1].split(";")[0])


def deploy_interrupted(rb):
    hub = fake_hub.Hub(usb=True)
    raised = False
    with hub.installed(rb), quiet():
        r = usb_runner(rb, hub)
        r.connect()

        def ctrl_c():
            raise KeyboardInterrupt

        try:
            r._deploy_sloth("t1", ctrl_c)
        except KeyboardInterrupt:
            raised = True
        r.dash.close()
        r.link.close()
    expect("deploySloth when the runner alone gets Ctrl-C after the push: the runner kills it rather than leave it "
           "waiting on the hub's lock", raised and hub.gradle_procs and hub.gradle_procs[-1].killed)


def link_checks(rb):
    for section in (usb_default, usb_reboot, usb_cable_pull, usb_stops, usb_unjudged, usb_interrupted, usb_rc_page,
                    deploy_interrupted, other_hub, wifi_link):
        try:
            section(rb)
        except Exception as e:
            expect(f"{section.__name__} ran to its end ({type(e).__name__}: {e})", False)
            traceback.print_exc()


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--record", default=None)
    p.add_argument("--runner", default=str(WORKTREE / "scripts/bench/run_bench.py"))
    a = p.parse_args()
    rb = runner_module(a.runner)
    if a.record:
        hub, runner, code, _ = default_run(rb)
        folder = pathlib.Path(a.record)
        folder.mkdir(parents=True, exist_ok=True)
        (folder / CALLS).write_text("".join(c + "\n" for c in normalized(hub, runner)))
        (folder / SELECTIONS).write_text(json.dumps(selections(rb), indent=1, sort_keys=True) + "\n")
        (folder / EVERYTHING).write_text(json.dumps(everything_record(rb), indent=1) + "\n")
        print(f"recorded {len(hub.calls)} adb calls and {len(selections(rb))} selections in {folder}")
        return 0

    hub, runner = wifi_checks(rb)
    link_checks(rb)
    same_calls("wifi: the adb calls are the recorded ones, in order and byte for byte, polls aside (fixtures/" + CALLS
               + ")", normalized(hub, runner), (FIXTURES / CALLS).read_text().splitlines())
    expect("the step selection matches the recorded one for every flag set (fixtures/" + SELECTIONS + ")",
           selections(rb) == json.loads((FIXTURES / SELECTIONS).read_text()))
    expect("--usb leaves the step selection as it is for every flag set",
           selections(rb, ["--usb"]) == json.loads((FIXTURES / SELECTIONS).read_text()))
    now, recorded = everything_record(rb), json.loads((FIXTURES / EVERYTHING).read_text())
    expect("wifi with a webcam, the Limelight and an Expansion Hub: their benches ran, the Limelight's HTTP included",
           {"webcam", "limelight", "expansion"} <= set(recorded["benches"])
           and any(remote == 5807 for remote, _ in recorded["http"]))
    for key in recorded:
        if key == "adb":
            same_calls(f"wifi with a webcam, the Limelight and an Expansion Hub: its adb calls are the recorded ones, "
                       f"polls aside (fixtures/{EVERYTHING})", now[key], recorded[key])
            continue
        expect(f"wifi with a webcam, the Limelight and an Expansion Hub: its {key} are the recorded ones, in order "
               f"(fixtures/{EVERYTHING})", now[key] == recorded[key])
        if now[key] != recorded[key]:
            at = next((i for i, (x, y) in enumerate(zip(now[key], recorded[key])) if x != y),
                      min(len(now[key]), len(recorded[key])))
            print(f"  first difference at {at + 1}: now {now[key][at:at + 1]}, recorded {recorded[key][at:at + 1]}")

    print(f"{passed} passed, {len(failed)} failed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
