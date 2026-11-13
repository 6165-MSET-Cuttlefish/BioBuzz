"""Offline checks for run_bench.py's adb-port preflight, saved-configs listing and Bench: Dashboard Off verdicts: each
must pass on a good synthetic input and fail on each broken one.
Run with: python3 -I scripts/bench/offline/test_station_checks.py"""
import copy
import importlib.util
import pathlib
import re
import subprocess
import sys

sys.dont_write_bytecode = True
HERE = pathlib.Path(__file__).resolve().parent
WORKTREE = HERE.parents[2]
spec = importlib.util.spec_from_file_location("run_bench", WORKTREE / "scripts/bench/run_bench.py")
rb = importlib.util.module_from_spec(spec)
spec.loader.exec_module(rb)
JAVA = WORKTREE / "TeamCode/src/main/java/org/firstinspires/ftc/teamcode"

passed = 0
failed = []


def expect(label, cond):
    global passed
    if cond:
        passed += 1
    else:
        failed.append(label)
        print("FAIL", label)


def raises(fn, *args, **kw):
    try:
        fn(*args, **kw)
    except rb.BenchError as e:
        return str(e)
    return None


def java_constant(path, name):
    m = re.search(rf'\b{name}\s*=\s*"([^"]*)"', path.read_text())
    return m.group(1) if m else None


# --- adb's port: lsof parsing and the verdict
SSH = "p4242\ncssh\nf5\nf6\n"
ADB = "p45913\ncadb\nf7\n"
expect("lsof: this Mac's -Fcp format (a Python listener, two fds) parses",
       rb.adb_port_owners("p79396\ncPython\nf3\nf4\n") == [("Python", "79396")])
expect("lsof: two processes", rb.adb_port_owners(ADB + SSH) == [("adb", "45913"), ("ssh", "4242")])
expect("lsof: a process without a command line still counts", rb.adb_port_owners("p77\nf3\n")[0][1] == "77")
expect("lsof: no output is no listener", rb.adb_port_owners("") == [])
problem = rb.preflight_problem(rb.adb_port_owners(SSH), 5037)
expect("verdict: an ssh listener stops the run and names it and its pid", problem and "ssh (pid 4242)" in problem)
expect("verdict: the message names no private tool or launchctl line",
       problem and not any(w in problem.lower() for w in ("launchctl", "bootout")))
expect("verdict: adb beside ssh still stops", rb.preflight_problem(rb.adb_port_owners(ADB + SSH), 5037))
expect("verdict: an unnamed process stops", rb.preflight_problem(rb.adb_port_owners("p77\nf3\n"), 5037))
expect("verdict: adb alone passes", rb.preflight_problem(rb.adb_port_owners(ADB), 5037) is None)
expect("verdict: nothing listening passes", rb.preflight_problem([], 5037) is None)
expect("verdict: a listener lsof can't see stops", rb.preflight_problem([], 5037, unseen_listener=True))
expect("port: 5037 by default", rb.adb_server_port({}) == 5037)
expect("port: ANDROID_ADB_SERVER_PORT moves it", rb.adb_server_port({"ANDROID_ADB_SERVER_PORT": "15037"}) == 15037)
for bad in ("abc", "0", "70000", "-1"):
    expect(f"port: ANDROID_ADB_SERVER_PORT={bad} stops", raises(rb.adb_server_port, {"ANDROID_ADB_SERVER_PORT": bad}))
for name in rb.ADB_SERVER_ELSEWHERE:
    expect(f"port: {name} pointing adb elsewhere stops", raises(rb.adb_server_port, {name: "tcp:192.168.43.1:5037"}))


class FakeLsof:
    def __init__(self, *outputs, code=None):
        self.outputs = list(outputs)
        self.code = code
        self.calls = []

    def __call__(self, argv, **kw):
        self.calls.append(argv)
        out = self.outputs.pop(0) if self.outputs else ""
        return subprocess.CompletedProcess(argv, self.code if self.code is not None else (0 if out else 1), out, "")


def no_listener(port):
    return False


def hidden_listener(port):
    return True


lsof = FakeLsof(SSH)
expect("preflight: an ssh listener raises", raises(rb.adb_port_preflight, {}, lsof, no_listener))
expect("preflight: it asks lsof about TCP listeners on the port, in -F form",
       lsof.calls and lsof.calls[0][1:] == ["-nP", "-iTCP:5037", "-sTCP:LISTEN", "-Fcp"])
lsof = FakeLsof(SSH)
expect("preflight: the env port is the one checked",
       raises(rb.adb_port_preflight, {"ANDROID_ADB_SERVER_PORT": "15037"}, lsof, no_listener)
       and lsof.calls[0][2] == "-iTCP:15037")
expect("preflight: adb passes and is recorded",
       rb.adb_port_preflight({}, FakeLsof(ADB), no_listener)["owners"] == [{"command": "adb", "pid": "45913"}])
expect("preflight: nothing listening passes",
       rb.adb_port_preflight({}, FakeLsof(""), no_listener) == {"port": 5037, "owners": []})
expect("preflight: something answering that lsof can't see raises",
       raises(rb.adb_port_preflight, {}, FakeLsof("", ""), hidden_listener))
expect("preflight: an adb server that started between the two looks passes",
       rb.adb_port_preflight({}, FakeLsof("", ADB), hidden_listener)["owners"][0]["command"] == "adb")
expect("preflight: lsof failing raises", raises(rb.adb_port_preflight, {}, FakeLsof("", code=2), no_listener))


def missing_lsof(argv, **kw):
    raise FileNotFoundError(argv[0])


expect("preflight: no lsof raises rather than passing", raises(rb.adb_port_preflight, {}, missing_lsof, no_listener))
start = {"port": 5037, "owners": [{"command": "adb", "pid": "45913"}]}
expect("end of run: the same adb server says nothing", rb.adb_server_change(start, copy.deepcopy(start)) is None)
expect("end of run: a new adb pid warns", "45913 -> 50000" in (rb.adb_server_change(
    start, {"port": 5037, "owners": [{"command": "adb", "pid": "50000"}]}) or ""))
expect("end of run: ssh on the port warns loudly", "**WARNING" in (rb.adb_server_change(
    start, {"port": 5037, "owners": [{"command": "ssh", "pid": "4242"}]}) or ""))
expect("end of run: an lsof failure is reported", "couldn't check" in (rb.adb_server_change(
    start, {"port": 5037, "error": "lsof exit 2"}) or ""))
expect("end of run: no server at the start and adb at the end says nothing",
       rb.adb_server_change({"port": 5037, "owners": []}, start) is None)


def hung_lsof(argv, **kw):
    raise subprocess.TimeoutExpired(argv, kw.get("timeout"))


def denied_lsof(argv, **kw):
    raise PermissionError(13, "Permission denied", argv[0])


for fake_lsof, label in ((hung_lsof, "a hung lsof"), (denied_lsof, "an lsof it can't run"),
                         (FakeLsof("", code=2), "lsof failing"), (missing_lsof, "no lsof")):
    end = rb.adb_port_record(5037, fake_lsof)
    expect(f"end of run: {label} is recorded, not raised, and the summary warns",
           end.get("port") == 5037 and end.get("error") and "couldn't check" in (rb.adb_server_change(start, end) or ""))

# --- saved configs
GOOD = (WORKTREE / "TeamCode/src/main/res/xml/betta_decode.xml").read_text()
OLD = ('<?xml version="1.0"?><Robot type="FirstInspires-FTC"><Webcam name="ballDetector" serialNumber="UC852" />'
       '<EthernetDevice name="ll" serialNumber="EthernetOverUsb:eth0:172.29.0.22" port="0" ipAddress="172.29.0.1" />'
       '</Robot>')
BARE = '<?xml version="1.0"?><Robot type="FirstInspires-FTC"></Robot>'
shipped = rb.shipped_config_names()
expect("shipped: the hardware configs in res/xml", {"betta_decode", "cuttle_decode", "biobuzz"} <= shipped)
expect("shipped: app_settings and device_filter aren't configs", not {"app_settings", "device_filter"} & shipped)
expect("devices: betta_decode names the webcam and the Limelight as the code does",
       rb.config_device_names(GOOD) == {"webcams": ["aprilTagDetector"], "ethernet": ["limelight"]})
expect("devices: broken XML is None", rb.config_device_names("<Robot") is None)
configs = {"betta_decode": GOOD, "cuttledecode": OLD, "bench_main": OLD, "broken": "<Robot", "spare": BARE}
found = rb.saved_config_findings(configs, shipped, "betta_decode", GOOD)
warned = "\n".join(found["warnings"])
expect("saved: cuttledecode is listed stale", "cuttledecode: saved on the hub but not a config" in warned)
expect("saved: a webcam named ballDetector warns", "Webcam 'ballDetector'" in warned)
expect("saved: an EthernetDevice not named limelight warns", "EthernetDevice 'll'" in warned)
expect("saved: unparseable XML warns", "broken: its XML doesn't parse" in warned)
expect("saved: bench_* configs never warn", "bench_main" not in warned)
expect("saved: the shipped, correct config doesn't warn", "betta_decode" not in warned)
expect("saved: every config gets a row", [r["name"] for r in found["rows"]] == sorted(configs))
expect("saved: an active config naming aprilTagDetector passes", found["check"][0] is True)
expect("saved: an active config naming ballDetector fails",
       rb.saved_config_findings(configs, shipped, "cuttledecode", OLD)["check"][0] is False)
expect("saved: an active config with no webcam passes",
       rb.saved_config_findings(configs, shipped, "spare", BARE)["check"] == (True, "it names no webcam"))
expect("saved: an unparseable active config fails",
       rb.saved_config_findings(configs, shipped, "broken", "<Robot")["check"][0] is False)
expect("saved: an unknown active config isn't judged",
       rb.saved_config_findings(configs, shipped, "unknown", None)["check"] is None)
two = OLD.replace('<EthernetDevice', '<Webcam name="aprilTagDetector" serialNumber="X" /><EthernetDevice')
expect("saved: one wrong webcam beside the right one still fails",
       rb.saved_config_findings({"two": two}, shipped, "two", two)["check"][0] is False)
expect("active: unread settings", rb.active_config_source(None, configs)[0] == "unknown")
expect("active: no config", rb.active_config_source({"name": rb.NO_CONFIG}, configs)[1] is None)
expect("active: a res/xml config reads this checkout's copy",
       rb.active_config_source({"name": "betta_decode", "location": "RESOURCE"}, {})[1] == GOOD)
expect("active: a saved config reads the hub's copy",
       rb.active_config_source({"name": "cuttledecode", "location": "LOCAL_STORAGE"}, configs)[1] == OLD)
expect("active: the copy from before the run wins over the run's own",
       rb.active_config_source({"name": "bench_main", "location": "LOCAL_STORAGE"}, configs, original_xml=BARE)[1] == BARE)
expect("active: a name nowhere isn't judged", rb.active_config_source({"name": "gone"}, configs)[1] is None)
listing = rb.parse_config_listing("total 24\n-rw-rw---- 1 root sdcard_rw 1234 2026-10-01 12:34 betta_decode.xml\n"
                                  "drwxrwx--x 2 root sdcard_rw 4096 2026-10-01 12:34 bench\n"
                                  "-rw-rw---- 1 root sdcard_rw 99 2026-09-23 08:00 cuttledecode.xml\n")
expect("listing: sizes and dates of the .xml files only",
       listing == {"betta_decode": {"size": 1234, "modified": "2026-10-01 12:34"},
                   "cuttledecode": {"size": 99, "modified": "2026-09-23 08:00"}})
expect("names: the runner's webcam is CellTipCamera's",
       rb.CELL_TIP_WEBCAM == java_constant(JAVA / "biobuzz/modules/CellTipCamera.java", "WEBCAM_NAME"))
expect("names: the runner's Limelight is LimelightBallSource's",
       rb.LIMELIGHT_DEVICE == java_constant(JAVA / "biobuzz/modules/vision/LimelightBallSource.java", "LIMELIGHT_NAME"))

# --- Bench: Dashboard Off
BENCH = JAVA / "opmodes/test/bench/DashboardOffBench.java"
expect("names: the runner sends the confirmation the OpMode requires",
       rb.DASH_OFF_CONFIRM == java_constant(BENCH, "CONFIRM"))
expect("names: the OpMode's name", f'@TeleOp(name = "{rb.DASH_OFF_OPMODE}"' in BENCH.read_text())
src = BENCH.read_text()
guard, switch, call = src.find("if (!opModeIsActive())"), src.find('report.phase("off")'), src.find("invoke(disable,")
expect("the OpMode checks for a STOP after its on stretch and before it calls disable()",
       0 <= src.find("stream(report, dashboard, onSeconds") < guard < switch < call)
expect("default run: connect doesn't require Bench: Dashboard Off on the hub", rb.DASH_OFF_OPMODE not in rb.BENCH_OPMODES)
OFF_PREFS = ("<?xml version='1.0' encoding='utf-8' standalone='yes' ?>\n<map>\n"
             '    <boolean name="autoEnable" value="false" />\n</map>\n')
expect("prefs: off", rb.dashboard_auto_enable(OFF_PREFS) is False)
expect("prefs: on", rb.dashboard_auto_enable(OFF_PREFS.replace("false", "true")) is True)
expect("prefs: no file is unset (on)",
       rb.dashboard_auto_enable("cat: shared_prefs/FtcDashboard.xml: No such file or directory\n") is None)
expect("prefs: no key is unset (on)", rb.dashboard_auto_enable("<map></map>") is None)
expect("prefs: garbage raises", raises(rb.dashboard_auto_enable, "<map"))
events = [(0.5, "RECEIVE_CONFIG", 0), (1.0, "RECEIVE_TELEMETRY", 3), (1.5, "RECEIVE_TELEMETRY", 0),
          (2.0, "RECEIVE_IMAGE", 0), (2.5, "RECEIVE_LOGCAT_ERRORS", 0), (3.0, "RECEIVE_TELEMETRY", 1)]
c = rb.window_counts(events, 1.0, 3.0)
expect("window: packets, clears, images, logcat, end exclusive",
       (c["telemetry packets"], c["telemetry clears"], c["images"], c["logcat"]) == (3, 1, 1, 1))
expect("window: every type counted", c["types"] == {"RECEIVE_TELEMETRY": 2, "RECEIVE_IMAGE": 1,
                                                    "RECEIVE_LOGCAT_ERRORS": 1})
expect("window: an empty INIT clear isn't streaming", rb.streamed(rb.window_counts(events, 1.4, 1.6)) == {})
expect("first packet: seconds after t0", rb.first_packet_after(events, 0.0) == 1.0)

PROXY = {p: True for p in rb.DASH_OFF_PROXY_PORTS}
CLOSED = {p: False for p in rb.DASH_OFF_PROXY_PORTS}
QUIET = {"telemetry packets": 0, "telemetry clears": 0, "images": 0, "logcat": 0, "seconds": 10.0,
         "types": {"RECEIVE_CONFIG": 1, "RECEIVE_OP_MODE_LIST": 1}}


def phase(name, **over):
    p = {"name": name, "statusEnabled": False, "autoEnable": False, "window": dict(QUIET), "socketOpen": True,
         "initIgnored": True, "listeners": dict(CLOSED), "backSeconds": 40.0}
    p.update(over)
    return p


GOOD_REC = {
    "planned": ["restart", "reboot", "reinstall"], "limelight": False,
    "baseline": {"statusEnabled": True, "autoEnable": None, "listeners": dict(PROXY)},
    "switch": {"offSeen": True, "socketOpen": True, "listeners": dict(CLOSED),
               "onCounts": dict(QUIET, **{"telemetry packets": 60, "images": 25}), "offCounts": dict(QUIET),
               "result": {"enabledAtStart": True, "enabledAfterDisable": False, "onFramesServed": 25,
                          "offFramesServed": 0}},
    "phases": [phase("restart"), phase("reboot"), phase("reinstall")],
    "recovery": {"statusEnabled": True, "enabledSeconds": 41.0, "opModeListSeconds": 41.5, "probePackets": 50,
                 "listeners": dict(PROXY), "autoEnable": True},
}


def checks(rec):
    return {name: ok for name, ok, _, _ in rb.dashboard_off_checks(rec)}


def broken(path, value, rec=None):
    o = copy.deepcopy(rec or GOOD_REC)
    t = o
    for p in path[:-1]:
        t = t[p]
    t[path[-1]] = value
    return checks(o)


def fails(results, fragment):
    hit = [ok for name, ok in results.items() if fragment in name]
    return bool(hit) and not all(hit)


good = checks(GOOD_REC)
expect("dashboard off: a clean run passes every check", good and all(good.values()))
expect("dashboard off: 4 checks for the switch, 4 per phase, 1 for the recovery", len(good) == 4 + 4 * 3 + 1)
expect("dashboard off: a Limelight adds the web UI check",
       len(broken(["recovery", "limelightHttp"], {"code": 200}, dict(GOOD_REC, limelight=True))) == len(good) + 1)
expect("dashboard off: the web UI not answering fails",
       fails(broken(["recovery", "limelightHttp"], {"error": "refused"}, dict(GOOD_REC, limelight=True)), "web UI"))
expect("setup: no telemetry while on fails", fails(broken(["switch", "onCounts", "telemetry packets"], 0), "(setup)"))
expect("setup: no images while on fails", fails(broken(["switch", "onCounts", "images"], 0), "(setup)"))
expect("switch: isEnabled still true fails", fails(broken(["switch", "result", "enabledAfterDisable"], True), "disable()"))
expect("switch: the status never said off fails", fails(broken(["switch", "offSeen"], False), "disable()"))
expect("switch: isEnabled false before the switch fails",
       fails(broken(["switch", "result", "enabledAtStart"], False), "disable()"))
expect("switch: one telemetry packet after the switch fails",
       fails(broken(["switch", "offCounts", "telemetry packets"], 1), "after the switch"))
expect("switch: a frame served after the switch fails", fails(broken(["switch", "result", "offFramesServed"], 3),
                                                             "after the switch"))
expect("switch: no frame count from the OpMode fails",
       fails(broken(["switch", "result"], {"enabledAtStart": True, "enabledAfterDisable": False}), "after the switch"))
expect("switch: a socket that closed mid-window fails", fails(broken(["switch", "socketOpen"], False), "after the switch"))
expect("switch: 5807 still listening fails", fails(broken(["switch", "listeners", 5807], True), "closed after the switch"))
for i, name in enumerate(GOOD_REC["planned"]):
    what = rb.DASH_OFF_PHASES[name]
    expect(f"{name}: an image fails", fails(broken(["phases", i, "window", "images"], 1), f"streams after {what}"))
    expect(f"{name}: a logcat line fails", fails(broken(["phases", i, "window", "logcat"], 1), f"streams after {what}"))
    expect(f"{name}: autoEnable back on fails", fails(broken(["phases", i, "autoEnable"], True), f"off after {what}"))
    expect(f"{name}: the prefs file gone fails", fails(broken(["phases", i, "autoEnable"], None), f"off after {what}"))
    expect(f"{name}: the status enabled fails", fails(broken(["phases", i, "statusEnabled"], True), f"off after {what}"))
    expect(f"{name}: an INIT taken fails", fails(broken(["phases", i, "initIgnored"], False), f"after {what} is ignored"))
    expect(f"{name}: 5800 listening fails", fails(broken(["phases", i, "listeners", 5800], True), f"closed after {what}"))
missing = copy.deepcopy(GOOD_REC)
missing["phases"] = missing["phases"][:1]
missing["failed"] = "the hub wasn't back from an adb reboot within 300 s"
r = rb.dashboard_off_checks(missing)
expect("phases: a phase that didn't run fails its checks, naming why",
       any(not ok and "not run: the hub wasn't back" in detail and "adb reboot" in name for name, ok, detail, _ in r))
expect("phases: the battery pull adds its checks when planned",
       len(checks(dict(GOOD_REC, planned=["battery"] + GOOD_REC["planned"],
                       phases=[phase("battery")] + GOOD_REC["phases"]))) == len(good) + 4)
not_before = copy.deepcopy(GOOD_REC)
not_before["baseline"]["listeners"][5805] = False
not_before["switch"]["listeners"][5805] = True
expect("ports: one that didn't listen before isn't judged", all(checks(not_before).values()))
none_before = copy.deepcopy(GOOD_REC)
none_before["baseline"]["listeners"] = dict(CLOSED)
expect("ports: none listening before the switch fails (proves nothing)", fails(checks(none_before), "proxy ports"))
expect("recovery: an error fails", fails(broken(["recovery", "error"], "adb lost the hub"), "recovery"))
expect("recovery: still off fails", fails(broken(["recovery", "statusEnabled"], False), "recovery"))
expect("recovery: on only after 75 s fails", fails(broken(["recovery", "enabledSeconds"], 75.0), "recovery"))
expect("recovery: never on fails", fails(broken(["recovery", "enabledSeconds"], None), "recovery"))
expect("recovery: no OpMode list fails", fails(broken(["recovery", "opModeListSeconds"], None), "recovery"))
expect("recovery: the OpMode list only after 75 s fails",
       fails(broken(["recovery", "opModeListSeconds"], 75.0), "recovery"))
expect("recovery: no Probe telemetry fails", fails(broken(["recovery", "probePackets"], 0), "recovery"))
expect("recovery: 5801 not listening fails", fails(broken(["recovery", "listeners", 5801], False), "recovery"))
expect("recovery: 5807 not listening fails", fails(broken(["recovery", "listeners", 5807], False), "recovery"))
expect("recovery: not run fails", fails(checks(dict(GOOD_REC, recovery={})), "recovery"))

# --- the default run doesn't change
BEFORE = ("baseline", "hub", "loop", "failure", "threads", "gamepad", "edge", "write_recovery", "handover", "photon",
          "hardware_view", "turret", "webcam", "webcam_failure", "limelight", "llfault", "sloth", "clock", "pinpoint",
          "octoquad", "octoquad_absent", "expansion", "absent", "planner_stop", "overlay_nan", "config_by_name",
          "freeze_dump")


def chosen(argv):
    r = object.__new__(rb.Runner)
    r.args = rb.parse_args(argv)
    r.expansion = r.webcam = r.limelight = None
    r.summary = []
    r.log = lambda text: None
    return r.selection()


default = chosen([])
expect("default run: the same benches as before", default == set(BEFORE) - {"photon", "expansion", "webcam",
                                                                           "webcam_failure", "limelight"})
expect("default run: in the same order", [n for n in rb.BENCHES if n in BEFORE] == list(BEFORE))
expect("default run: Dashboard Off is last", rb.BENCHES[-1] == "dashboard_off")
r_all = object.__new__(rb.Runner)
r_all.args = rb.parse_args(["--all"])
r_all.expansion, r_all.webcam, r_all.limelight, r_all.summary, r_all.log = 2, "w", {"ipAddress": "x"}, [], lambda t: None
expect("--all: no Dashboard Off and no Photon", not {"dashboard_off", "photon"} & r_all.selection())
expect("--dashboard-off: only that bench", chosen(["--dashboard-off"]) == {"dashboard_off"})
expect("--dashboard-off --hub: both", chosen(["--dashboard-off", "--hub"]) == {"dashboard_off", "hub"})
r = object.__new__(rb.Runner)
r.adb = type("A", (), {"serial": "192.168.43.1:5555"})()
expect("recovery command: unchanged from before",
       r.dashboard_recovery() == "adb -s 192.168.43.1:5555 shell 'am force-stop com.qualcomm.ftcrobotcontroller && "
       "run-as com.qualcomm.ftcrobotcontroller rm -f shared_prefs/FtcDashboard.xml; am start -n "
       "com.qualcomm.ftcrobotcontroller/org.firstinspires.ftc.robotcontroller.internal.PermissionValidatorWrapper'")
r.results = {"adbServer": {"atStart": {"port": 15037, "owners": []}}}
expect("recovery command: the caution before it names the run's adb port",
       "`lsof -nP -iTCP:15037 -sTCP:LISTEN`" in r.adb_port_caution())
r.results = {}
expect("recovery command: without a port check it names 5037", "-iTCP:5037 " in r.adb_port_caution())
expect("recovery command: the one docs/dashboard-findings.md prints",
       f"adb shell '{rb.DASHBOARD_RECOVERY_SHELL}'" in (WORKTREE / "docs/dashboard-findings.md").read_text())

print(f"{passed} passed, {len(failed)} failed")
sys.exit(1 if failed else 0)
