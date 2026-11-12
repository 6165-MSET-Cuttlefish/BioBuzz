"""Drives Runner.run_photon end to end with every hub call stubbed, to catch orchestration bugs offline."""
import copy
import importlib.util
import json
import pathlib
import sys
import tempfile

sys.dont_write_bytecode = True
HERE = pathlib.Path(__file__).resolve().parent
WORKTREE = str(HERE.parents[3])
OUT = HERE.parent / "out" / "dryrun"
spec = importlib.util.spec_from_file_location("run_bench", WORKTREE + "/scripts/bench/run_bench.py")
rb = importlib.util.module_from_spec(spec)
spec.loader.exec_module(rb)
tspec = importlib.util.spec_from_file_location("fixtures", HERE / "test_photon_checks.py")


def load_fixtures():
    src = (HERE / "test_photon_checks.py").read_text().split("# --- one workload phase")[0]
    ns = {"__file__": str(HERE / "test_photon_checks.py")}
    exec(compile(src, "fixtures", "exec"), ns)
    return ns


F = load_fixtures()
phase, counters, clean_leftovers = F["phase"], F["counters"], F["clean_leftovers"]
depth_trial, superseded_trial = F["depth_trial"], F["superseded_trial"]
i2c_phase, i2c_info, i2c_control = F["i2c_phase"], F["i2c_info"], F["i2c_control"]
INSTALLED = rb.photon_source_sha256(rb.REPO_ROOT / rb.PHOTON_SOURCE_DIR)


def sampler(control=None, fraction=0.3, late_read=False, linear=False, interrupted=True):
    rows = []
    for i in range(40):
        gate = 1 if i >= 10 else 0
        vals = [0] * 12 if gate and i >= 12 else [1, 6553, 1, 6553, 1, 6553, 1, 6553, 1, 0, 1, 0]
        if control == "held" and i == 15:
            vals = [1, 6553] + [0] * 10
        rows.append([i * 10.0, i * 10.0 + 9, gate] + vals)
    hub = {"key": "controlHub", "address": 173, "kinds": ["motor"] * 4 + ["servo"] * 2,
           "channels": ["fl", "fr", "bl", "br", "benchServo", "benchServo2"], "gateMs": 100.0, "sweeps": rows}
    eh = dict(hub, key="expansionHub", address=2)
    outputs = [{"channel": n, "hub": "controlHub", "kind": "motor", "enabled": False, "raw": 0} for n in ("fl", "fr")]
    if control == "once" or late_read:
        outputs[0] = dict(outputs[0], enabled=True, raw=6553)
    extra = {"stopSawBeforeZeroing": {"gateUp": True, "photonEnabled": False, "outputs": outputs},
             "countersAtStop": counters()}
    if linear:
        extra["interruptedAtStop"] = interrupted
    if control == "held":
        extra["negativeControl"] = {"variant": "held", "samplerSawFailSafeFirst": True}
    if control == "once":
        extra["negativeControl"] = {"variant": "once"}
    return {"complete": True, "gateSeen": True, "failure": None, "loopAtGate": {"fractionOfLoop": fraction},
            "info": {"variant": "linear" if linear else "iterative"}, "extra": extra, "hubs": [hub, eh]}


def routing(order, pid=100, bench="PathClassLoader@1", ended=0):
    r = {"atInit": {"photonEnabled": False, "swapFailures": [], "routingProblems": [], "swapOrder": list(order),
                    "inFlight": {"173": 0, "2": 0}, "pendingEntries": {"173": 0, "2": 0},
                    "standIns": {"173": 0, "2": 0}, "routes": {"173": 0, "2": 0},
                    "counters": counters(supersededWrites=ended),
                    "loaders": {"pid": pid, "photonCoreLoader": "PathClassLoader@1", "benchLoader": bench,
                                "photonCorePinned": True, "photonModulePinned": True,
                                "hubs": [{"identity": 11}, {"identity": 22}]}},
         "notRespondingRecovery": {m: {h: {"visible": True, "clearedMs": 2.0, "voltsAfter": 12.3, "notRespondingAtEnd": False}
                                       for h in ("controlHub", "expansionHub")} for m in ("off", "on8")},
         "parallelCheck": {"drainMs": 1.0, "issuedOnParallelHubs": 240, "counters": counters(parallelWrites=240),
                           "readback": {"checks": 12, "mismatches": 0, "events": []}, "routingProblemsAfter": [],
                           "leftoversAfterDrain": clean_leftovers()}}
    return r


class FakeStream:
    def __init__(self, path, extra=()):
        self.path = path
        path.write_text("\n".join(["x I BENCH   : phase timing on8", *extra]) + "\n")

    def stop(self):
        pass


class FakeAdb:
    serial = "fake"
    private = True

    def hub_time(self):
        return "10-29 00:00:00.000"

    def pid(self):
        return 100

    def config_names(self):
        return ["betta_decode"]

    def read_config(self, name):
        return "<Robot type=\"FirstInspires-FTC\"></Robot>"

    def mark(self, text):
        pass

    def ensure_connected(self):
        pass


class Dry(rb.Runner):
    def __init__(self, argv, break_what=None):
        self.args = rb.parse_args(argv)
        OUT.mkdir(parents=True, exist_ok=True)
        self.out = pathlib.Path(tempfile.mkdtemp(prefix="photon-dry-", dir=OUT))
        (self.out / "raw").mkdir()
        (self.out / "logs").mkdir()
        self.logfile = open(self.out / "run.log", "w")
        self.summary, self.checks, self.results = [], [], {}
        self.adb = FakeAdb()
        self.probe = {"hubs": [{"firmware": "1.8.2"}]}
        self.imu_type = "BNO055"
        self.expansion = 2 if self.args.photon_rs485 else None
        self.known_pid = 100
        self.photon = None
        self.switches, self.restarts = [], []
        self.last = {}
        self.restart_count = 0
        self.break_what = break_what
        self.steps = []
        self.configs = {}
        self.asked = []
        self.summary_writes = []

    def log(self, text):
        self.logfile.write(text + "\n")

    def ask(self, text):
        self.asked.append(text)
        self.log(text)

    def tell(self, text):
        self.ask(text)
        if self.break_what == "ctrl-c at the last prompt" and "put the Pinpoint back" in text:
            raise KeyboardInterrupt

    def write_summary(self, error=None):
        self.summary_writes.append((len(self.asked), "### Photon verdict" in "\n".join(self.summary)))

    def main_log_extra(self):
        lines = []
        if self.break_what == "i2c warning":
            lines += ["x I BENCH   : phase i2c on8", "x E LynxI2cDeviceSynch: placeholder: readStatusQuery"]
        if self.args.photon_i2c:
            lines += ["x I BENCH   : phase i2c control", "x E RobotCore: exception thrown during lynx communication"]
        return lines

    def activate_config(self, name, xml_text=None, config=None):
        self.restart_count += 1
        self.steps.append(("config", name))
        if xml_text is not None:
            self.configs[name] = xml_text

    def wait_ready(self):
        pass

    def wait_restart(self, during):
        pass

    def usb_expansion_serial(self):
        return "DQTEST"

    def run_opmode(self, opmode, step, params=None, **kw):
        self.steps.append((opmode, step))
        self.last[step] = (opmode, params or {})
        lines = [f"x I BENCH   : phase {ph}" for ph in ("writing", "timing on8", "rs485 shared link off",
                                                          "rs485 shared link on8")]
        if self.break_what == "abandon in a STOP trial" and step.startswith("photon_stop_iter_on_000"):
            lines.append("x E LynxModule: timeout: abandoning waiting 250ms for ack: cmd=X mod=2 msg#=4")
        if step.startswith("photon_stop_"):
            lines.append(f"10-29 00:00:01.000  1  2 I FtcEventLoop: auto-stopping OpMode '{opmode}'")
            leaks = 5 if self.break_what == "linear leaks" else 1
            if step.startswith("photon_stop_linear_on_") and int(step.rsplit("_", 1)[1]) < leaks:
                lines.append("10-29 00:00:01.508  1  9 E RobotCore: lynx xmit lock: #### abandoning lock: "
                             "old=LynxSetServoEnableCommand(0) new=LynxGetMotorChannelEnableCommand(0)")
            if step == "photon_stop_linear_once_002" and self.break_what in ("linear once leak", "linear once stray"):
                lines.append(f"10-29 00:00:{'01.508' if self.break_what == 'linear once leak' else '01.900'}  1  9 E RobotCore: "
                             "lynx xmit lock: #### abandoning lock: old=LynxSetServoEnableCommand(0) "
                             "new=LynxGetMotorChannelEnableCommand(0)")
        if step == "photon_main":
            lines += self.main_log_extra()
        (self.out / "logs" / f"{step}.log").write_text("\n".join(lines) + "\n")
        crashed = bool(kw.get("expect_crash")) or (self.break_what == "crashed STOP trial"
                                                   and step == "photon_stop_iter_on_001")
        if self.break_what == "stock init crash" and step == "photon_stock":
            return {"step": step, "opmode": opmode, "crashed": True, "statusErrorMessage": "", "appRestarted": False,
                    "timedOut": False, "errorMessage": "IllegalStateException: the Pinpoint on expansionHub I2C bus 1 "
                                                      "read device ID 0, version 0: it isn't answering"}
        return {"step": step, "opmode": opmode, "crashed": crashed, "errorMessage": "boom" if crashed else "",
                "statusErrorMessage": "", "appRestarted": False, "timedOut": bool(kw.get("timeout_ok"))}

    def logcat_stream(self, path):
        return FakeStream(path, self.main_log_extra())

    def _deploy_sloth(self, run_id, on_pushed, timeout_s=None):
        self.sloth_loaded = True
        return {"seconds": 30.0, "endedAt": 0, "returncode": 0, "hung": False, "secondsFromPushToEnd": 1.0,
                "listMarkAtPush": 0, "connectionAtPush": 1, "pushedAt": 0.0}

    def sloth_load(self, hub_t0, run_id, deploy):
        return {"result": "processed"}

    def pull(self, name, required=True):
        topo = "rs485" if self.args.photon_rs485 else "usb"
        i2c_topo = "rs485" if self.args.photon_rs485 else "usb" if self.args.usb_expansion else "none"
        if name == "photon_stock_1" and self.break_what == "stock init crash":
            return {}
        if name.startswith("photon_stock_"):
            ph = phase(parallel=-1, writeBlockPerHubMs={"controlHub": {"count": 9, "p50": 9.0, "p95": 19, "max": 29},
                                                       "expansionHub": {"count": 9, "p50": 16.0, "p95": 26, "max": 41}})
            return {"info": {"hubs": [{"key": "controlHub", "address": 173, "swapped": name != "photon_stock_1", "firmware": "1.8.2"},
                                      {"key": "expansionHub", "address": 2, "swapped": name != "photon_stock_1", "firmware": "1.8.2"}],
                             "inFlightAtInit": 0, "photonEnabledAtInit": False, "sdkVersion": "26.09.07",
                             "leftoversAtInit": clean_leftovers(),
                             "installedPhotonSourceSha256": "0" * 64 if self.break_what == "stale install" else INSTALLED},
                    "enableRefused": {"refused": True, "message": "needs @Photon"},
                    "portProbe": {"empty": self.break_what != "busy port", "power": 0.1, "limitAmps": 0.1, "ports": []},
                    "timing": {"stock": ph, "stock+poller": ph}, "burst": ph,
                    **(self.i2c_stock(i2c_topo) if self.args.photon_i2c and name == "photon_stock_1" else {})}
        if name.startswith("photon_routing_"):
            rid = name.rsplit("_", 1)[1]
            order = (173, 2) if self.restart_count % 2 else (2, 173)
            if self.break_what == "one order":
                order = (173, 2)
            lost = 2 * (2 if topo == "usb" else 1)
            ended = lost - (1 if self.break_what == "crash leak" else 0) if rid != "h1" and rid != "h3" and rid != "r01" else 0
            return routing(order, bench="SlothClassLoader@9" if getattr(self, "sloth_loaded", False) else "PathClassLoader@1",
                           ended=ended)
        if name == "photon_1":
            on8 = phase()
            off = phase(parallel=0, writeBlockPerHubMs={"controlHub": {"count": 9, "p50": 9.1, "p95": 19, "max": 29},
                                                       "expansionHub": {"count": 9, "p50": 16.1, "p95": 26, "max": 41}})
            if self.break_what == "lost write in on8":
                on8 = copy.deepcopy(on8)
                on8["counters"]["abandonedWrites"] = 1
            main = {"timing": {"off": off, "on8": on8, "on4": phase(parallel=4), "on2": phase(parallel=2, burst=6,
                     counters=counters(parallelWrites=600), maxInFlight={"controlHub": 2, "expansionHub": 2})},
                    "burst": {"off": off}, "interleave": {"on8": on8}, "reads": {"on8": on8},
                    "twoWriters": phase(countExact=False, toggles=50, servoWrites=10, motorWrites=10),
                    "storm": phase(parallel=1, burst=3000, maxInFlight={"controlHub": 1, "expansionHub": 1},
                                   counters=counters(parallelWrites=600), maxStallMs=5.0, writes=3000),
                    "soak": phase(issued=2 * 7_000_000, issuedWrites={"controlHub": 7_000_000, "expansionHub": 7_000_000})
                    if self.args.photon_soak else phase(),
                    "info": {"routingProblems": [], "photonSwapFailures": [], "inFlightAtInit": 0,
                             "photonEnabledAtInit": False, "leftoversAtInit": clean_leftovers(),
                             "hubs": [{"address": 173, "swapped": True}, {"address": 2, "swapped": True}]}}
            main["timing"]["on8 profiled"] = phase(writeProfile={"controlHub": {"sweep": {"count": 3.0, "meanUs": 2.0,
                                                                                          "p50Us": 2.5, "p95Us": 7.5}}})
            if self.args.photon_i2c:
                main.update(self.i2c_main(i2c_topo))
            if self.break_what == "number clash":
                main["timing"]["on8"]["counters"]["numberClashes"] = 1
            if topo == "rs485":
                ch_only = {"controlHub": 6, "expansionHub": 0}
                main["rs485SharedLink"] = {"off": phase(parallel=0),
                                           "on8": phase(parallel=8, issued=300, issuedWrites={"controlHub": 300, "expansionHub": 300},
                                                        counters=counters(parallelWrites=300), maxInFlight=ch_only,
                                                        maxPendingEntries=ch_only, maxStandIns=ch_only)}
            return main
        if name == "photon_faults_1":
            hubs = ["controlHub"] + (["expansionHub"] if topo == "usb" else [])
            src = (HERE / "test_photon_checks.py").read_text()
            ns = {}
            exec(compile(src.split("# --- lost writes")[1].split("expect(\"lost: good passes\"")[0], "lost", "exec"),
                 {"counters": counters}, ns)
            lost = [dict(ns["lost"], hub=h) for h in hubs for _ in range(6)]
            over = [dict(ns["over"], hub=h) for h in hubs]
            rec = {"settledBeforeReset": True, "resetSeen": True, "sdkCacheValidAfterRawFailSafe": True,
                   "recoveredMs": {"fl (WriteCache)": 210.0}}
            all_hubs = ["controlHub", "expansionHub"]
            if self.break_what == "faults stopped early":
                lost = lost[:3]
            throws = [{"hub": h, "thrown": 1, "writeCallMs": 2.0, "landed": True, "inFlightAfter": 0, "notResponding": False,
                       "shutdownReason": "NORMAL", "counters": counters(writeFailures=1)} for h in hubs]
            _, params = self.last["photon_faults"]
            among = [depth_trial(hub=h, depth=d) for h in hubs for d in range(8) for _ in range(params["depthReps"])]
            sup = [superseded_trial(hub=h) for h in hubs for _ in range(params["supersededReps"])]
            if self.break_what == "lost at depth":
                among[-1] = depth_trial(hub=hubs[-1], depth=7, **{"leftoversAfterAbandon.standIns": 1})
            return {"lostWrite": lost, "lostOverLimit": over, "writeThrows": throws, "lostAmongWrites": among,
                    "lostThenSuperseded": sup, "resetRecovery": {h: {"off": rec, "on8": rec} for h in all_hubs}}
        if name.startswith("photon_stop_race_"):
            _, params = self.last[f"photon_race_{name[len('photon_stop_race_'):]}"]
            return {"result": {"photon": params["photon"], "gateRaisedInsideWrite": True,
                               "violation": self.break_what == "race" and params["photon"] > 0, "hub": params["hub"], "reads": []}}
        if name.startswith("photon_stop_main_"):
            return {}
        if name.startswith("photon_stop_"):
            tid = name[len("photon_stop_"):]
            n = int(tid.rsplit("_", 1)[1])
            control = "held" if "_held_" in tid else "once" if "_once_" in tid else None
            if self.break_what == "silent control" and control == "once":
                control = None
            late = self.break_what == "late write" and tid == "iter_on_002"
            return sampler(control=control, fraction=(n % 4) / 4 + 0.05, late_read=late, linear=tid.startswith("linear_"),
                           interrupted=not (self.break_what == "linear interrupt missed" and tid == "linear_on_003"))
        if name.startswith("photon_crash_sampler_"):
            return sampler()
        if name.startswith("photon_crash_"):
            lost = {"controlHub": 2} if topo == "rs485" else {"controlHub": 2, "expansionHub": 2}
            return {"inFlightAtThrow": 5, "lostAtThrow": lost}
        if name == "framework_loop":
            return {"phases": {}, "photonParallel": 8, "complete": True}
        if name == "write_recovery":
            return self.write_recovery()
        if name.startswith("photon_replug_"):
            on = name.endswith("_on8")
            return {"result": {"faultSeen": True, "maxWriteStallMs": 110.0, "maxSingleWriteMs": 102.0,
                               "recoveryMsAfterBack": 200.0, "inFlightAtEnd": 0, "drainMs": 1.0,
                               "leftoversAtEnd": clean_leftovers(), "controlHubReadback": {"mismatches": 0},
                               "routingProblemsAfter": [], "parallelWriteMsAfterBack": 9.0 if on else None,
                               "countersDuringFault": counters(writeFailures=1) if on else counters()}}
        raise AssertionError(f"unexpected pull {name}")

    def main_config(self, **kw):
        return "<Robot/>"

    def write_recovery(self):
        wrapped = {"fl": "motor", "benchServo2": "servo"}
        channels = {n: {"wrapped": n in wrapped, "kind": wrapped.get(n, "motor" if n == "fr" else "servo"),
                        "settledBeforeFailSafe": True} for n in ("fl", "fr", "benchServo", "benchServo2")}
        trials = []
        for i, phase_ms in enumerate((0, 40, 80, 120, 160, 200, 235)):
            back = {n: ({"recoveredMs": 250.0 - phase_ms + 4.0, "recoveredLoops": 26} if n in wrapped
                        else {"recoveredMs": 2.0, "recoveredLoops": 1}) for n in channels}
            if self.break_what == "write recovery late" and i == 0:
                back["fl"] = {"recoveredMs": 400.0, "recoveredLoops": 40}
            trials.append({"trial": i, "phaseMs": phase_ms, "channels": back})
        return {"channels": channels, "trials": trials, "info": {"resendMs": 250.0, "batteryVolts": 12.6},
                "loopPeriodMs": {"count": 900, "p50": 9.5, "p95": 10.0, "max": 14.0}, "photonParallel": 8}

    def i2c_stock(self, topo):
        info = i2c_info(topo, swapped=False)
        if self.break_what == "i2c device missing":
            info["devices"] = info["devices"][:1]
        return {"i2cInfo": info, "i2c": {"stock": i2c_phase(parallel=-1, topo=topo, swapped=False)}}

    def i2c_main(self, topo):
        on8 = i2c_phase(8, topo)
        if self.break_what == "i2c wrong byte":
            on8["i2cReaders"][2]["wrong"] = 1
        m = {"i2cInfo": i2c_info(topo), "i2c": {"off": i2c_phase(0, topo, replaced=self.break_what != "no replaced reader"),
                                                "on2": i2c_phase(2, topo), "on8": on8},
             "i2cControl": i2c_control(topo), "i2cAfterControl": i2c_phase(0, topo, reads=120),
             "soak": i2c_phase(8, topo, reads=500000 if self.args.photon_soak else 6000)}
        if self.args.photon_soak:
            m["soak"]["issuedWrites"] = {"controlHub": 7_000_000, "expansionHub": 7_000_000}
            m["soak"]["issuedOnParallelHubs"] = 14_000_000
            m["soak"]["counters"] = counters(parallelWrites=14_000_000)
        return m


def run(argv, break_what=None):
    d = Dry(argv, break_what)
    d.run_photon()
    v = d.results["photon"]["verdict"]
    text = "\n".join(d.summary)
    json.dumps(d.results, default=list)
    return d, v, text


fails = []
base_args = ["--photon", "--usb-expansion", "--ports-empty", "--photon-seed", "7"]
d, v, text = run(base_args)
print("usb verdict:", v["level"], v["blockers"][:5], v["stillNeeded"])
print("restarts:", d.restart_count, "STOP runs:", sum(1 for s in d.steps if s[0] == "Bench: Photon Stop"),
      "linear STOP runs:", sum(1 for s in d.steps if s[0] == "Bench: Photon Stop Linear"))
if v["level"] != "PASSED" or v["safe"]:
    fails.append("an all-good short USB run should be PASSED, not SAFE")
if not any("soak" in n for n in v["stillNeeded"]) or not any("cable pulls" in n for n in v["stillNeeded"]) \
        or not any("200 STOP trials" in n for n in v["stillNeeded"]) or not any("--photon-i2c" in n for n in v["stillNeeded"]):
    fails.append("a short run's verdict must name the soak, the cable pulls, the STOP trial count and the I2C reads")
linear_steps = [s[1] for s in d.steps if s[0] == "Bench: Photon Stop Linear"]
if sum(1 for s in linear_steps if "_off_" in s) != 40 or sum(1 for s in linear_steps if "_once_" in s) != 5 \
        or sum(1 for s in linear_steps if "_on_" in s) != 40:
    fails.append("the linear STOP condition needs its Photon-off control (40) and its once negative control (5)")
if d.last["photon_stop_linear_off_000"][1].get("photon") != 0 or d.last["photon_stop_linear_once_000"][1].get("control") != 2:
    fails.append("the linear controls must run with Photon off, and the once control with control=2")
if "linear STOP trials: Photon leaks the SDK's lock at STOP no more often than the SDK alone" not in text:
    fails.append("the summary must judge the linear lock leaks against the Photon-off control")
if "- I2C: not run (--photon-i2c)" not in text:
    fails.append("the summary header must say I2C wasn't run")
if "PASSED THIS RUN, NOT YET SAFE" not in text or "SAFE TO TRY ON THE ROBOT" in text:
    fails.append("short-run verdict text wrong")
if ("Bench: Framework Loop Photon", "framework_loop_photon8") not in d.steps or ("config", "bench_main") != d.steps[-1]:
    fails.append("the framework step must use the @Photon subclass and end with a fresh bench_main map")
d, v, text = run(base_args + ["--interactive", "--photon-soak"])
print("full usb verdict without --photon-i2c:", v["level"], v["blockers"][:5], v["stillNeeded"])
if v["level"] != "PASSED" or v["stillNeeded"] != ["I2C reads through Photon's hubs on every hub (--photon-i2c)"]:
    fails.append("an all-good soak + cable-pull USB run without --photon-i2c must stop at PASSED, naming the I2C reads")
full_args = base_args + ["--interactive", "--photon-soak", "--photon-i2c"]
d, v, text = run(full_args)
print("full usb verdict:", v["level"], v["blockers"][:5], v["stillNeeded"])
if v["level"] != "SAFE" or "SAFE TO TRY ON THE ROBOT" not in text:
    fails.append("an all-good soak + cable-pull + I2C USB run should be SAFE")
xml = d.configs.get("bench_photon", "")
eh_block = xml.split('<LynxUsbDevice name="Expansion Hub Portal"')[-1]
if '<OctoQuadFTC name="octoquad" port="0" bus="2" />' not in xml.split("</LynxModule>")[0] \
        or '<goBILDAPinpoint name="pinpoint" port="0" bus="1" />' not in eh_block or xml.count("goBILDAPinpoint") != 1:
    fails.append("--photon-i2c must put the OctoQuad on Control Hub bus 2 and the Pinpoint on the USB Expansion Hub's bus 1")
if not any("Expansion Hub I2C port 1" in a for a in d.asked) or not any("back in Control Hub I2C port 1" in a for a in d.asked) \
        or "back in Control Hub I2C port 1" not in d.asked[-1]:
    fails.append("--photon-i2c must ask for the Pinpoint on the Expansion Hub, and back afterwards")
if "### I2C reads" not in text or "rule of three" not in text or "- I2C: the goBILDA Pinpoint in Expansion Hub I2C port 1" not in text:
    fails.append("the summary must carry the I2C table, its rule-of-three bound and the I2C header line")
if "### Parallel write path at 8 parallel" not in text:
    fails.append("the summary must carry the write-path profile")
if sum(1 for s in d.steps if s[0] == "Bench: Photon Replug") != 4:
    fails.append("the cable pulls must run each pull with Photon off and on")
if d.summary_writes[:1] != [(len(d.asked) - 1, True)]:
    fails.append("summary.md must be written with the Photon verdict before the OctoQuad move-back prompt")
d = Dry(full_args, "ctrl-c at the last prompt")
try:
    d.run_photon()
    fails.append("a Ctrl-C at the move-back prompt must propagate")
except KeyboardInterrupt:
    pass
if d.results.get("photon", {}).get("verdict", {}).get("level") != "SAFE" or "### Photon verdict" not in "\n".join(d.summary):
    fails.append("a Ctrl-C at the move-back prompt must leave the Photon verdict in the summary")
_, v, _ = run(full_args, "linear once leak")
print("linear once leak:", v["level"], v["blockers"][:3])
if v["level"] != "SAFE":
    fails.append("a STOP-interrupt leak in the linear negative control is the SDK's own and must not block")
_, v, _ = run(full_args, "linear once stray")
print("linear once stray:", v["level"], v["blockers"][:3])
if v["level"] != "NOT SAFE" or not any("every abandoned network lock is one STOP's interrupt leaked" in b for b in v["blockers"]):
    fails.append("an abandon outside the STOP window in the linear negative control must block")
for broken, needle in (("lost write in on8", "phases"), ("race", "race"), ("silent control", "negative control"),
                       ("one order", "routing"), ("faults stopped early", "lost"), ("stale install", "install"),
                       ("crash leak", "crash"), ("lost at depth", "lost"), ("late write", "stop"),
                       ("crashed STOP trial", "stop"), ("abandon in a STOP trial", "logcat"),
                       ("number clash", "phases"), ("linear interrupt missed", "stop: linear"),
                       ("linear leaks", "logcat: linear STOP trials"), ("i2c wrong byte", "i2c: I2C on8"),
                       ("i2c device missing", "i2c: the stock baseline"), ("no replaced reader", "i2c: I2C off"),
                       ("i2c warning", "logcat: Bench: Photon:"),
                       ("write recovery late", "reset: Write Recovery with Photon on: EnhancedMotor fl")):
    _, v, _ = run(full_args, broken)
    print(f"{broken}: {v['level']}, blockers {v['blockers'][:3]}")
    if v["level"] != "NOT SAFE" or not any(needle in b for b in v["blockers"]):
        fails.append(f"{broken} should block the verdict")
_, v, text = run(["--photon", "--photon-rs485", "--ports-empty", "--no-sloth", "--photon-stop-trials", "4"])
print("rs485 --no-sloth:", v["level"], v["blockers"])
if v["level"] != "NOT SAFE" or not any("deploySloth" in b for b in v["blockers"]):
    fails.append("skipping deploySloth should block the verdict")
if "RS485 Expansion Hub" not in text:
    fails.append("rs485 section missing")
_, v, text = run(["--photon", "--photon-rs485", "--ports-empty", "--photon-stop-trials", "4"])
print("rs485:", v["level"], v["blockers"], v["stillNeeded"])
if v["blockers"]:
    fails.append("an all-good RS485 run should have no blockers")
d, v, text = run(["--photon", "--photon-rs485", "--ports-empty", "--photon-stop-trials", "4", "--photon-i2c"])
print("rs485 --photon-i2c:", v["level"], v["blockers"], v["stillNeeded"])
xml = d.configs.get("bench_photon", "")
child = xml.split('<LynxModule name="Expansion Hub 2" port="2">')[-1]
if v["blockers"] or any("--photon-i2c" in n for n in v["stillNeeded"]) or '<goBILDAPinpoint name="pinpoint" port="0" bus="1" />' not in child \
        or '<OctoQuadFTC name="octoquad" port="0" bus="2" />' not in xml.split("</LynxModule>")[0]:
    fails.append("an all-good RS485 --photon-i2c run should have no blockers, put the Pinpoint on the RS485 child's bus 1 "
                 "and keep the OctoQuad on Control Hub bus 2")
d, v, text = run(["--photon", "--ports-empty", "--photon-stop-trials", "4", "--photon-i2c"])
print("Control Hub only --photon-i2c:", v["level"], v["blockers"][:3])
none_xml = d.configs.get("bench_photon", "")
if any(b.startswith("i2c") for b in v["blockers"]) or '<OctoQuadFTC name="octoquad" port="0" bus="2" />' not in none_xml \
        or '<goBILDAPinpoint name="pinpoint" port="0" bus="1" />' not in none_xml \
        or "Expansion Hub" in none_xml or any("back in Control Hub" in a for a in d.asked):
    fails.append("with no Expansion Hub, --photon-i2c keeps both devices on the Control Hub and asks for no move back")
_, v, text = run(["--photon", "--usb-expansion", "--ports-empty", "--interactive", "--photon-stop-trials", "4"])
if "Cable pulls" not in text:
    fails.append("replug section missing")
try:
    run(["--photon", "--usb-expansion", "--ports-empty"], "busy port")
    fails.append("a busy port should stop the block")
except rb.BenchError as e:
    print("busy port stops the block:", e)
try:
    run(full_args, "stock init crash")
    fails.append("a stock baseline that crashed at INIT should stop the block")
except rb.BenchError as e:
    print("stock init crash stops the block:", e)
    if "crashed before its port probe ran" not in str(e) or "Pinpoint on expansionHub I2C bus 1" not in str(e) \
            or "drew current" in str(e):
        fails.append("a stock baseline that crashed at INIT must be reported with its own error, not as a busy port")
d = Dry(["--photon", "--usb-expansion", "--ports-empty"], "busy port")
try:
    d.run_photon()
except rb.BenchError:
    pass
v = d.results["photon"]["verdict"]
print("early stop verdict:", v["level"], v["blockers"][:3])
if v["safe"] or v["level"] != "NOT SAFE" or "### Photon verdict" not in "\n".join(d.summary):
    fails.append("an early stop must still write a NOT SAFE verdict")
print("FAILURES:", fails or "none")
sys.exit(1 if fails else 0)
