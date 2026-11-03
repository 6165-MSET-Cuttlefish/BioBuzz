"""Offline negative controls for run_bench.py's Photon checks: every check must pass on a good synthetic result and
fail on each broken one. Run with: python3 -I runner/test_photon_checks.py [bench results dir]; the real-results checks
default to runner/fixtures."""
import copy
import importlib.util
import json
import pathlib
import sys

HERE = pathlib.Path(__file__).resolve().parent
WORKTREE = str(HERE.parents[3])
RESULTS = pathlib.Path(sys.argv[1]) if len(sys.argv) > 1 else HERE / "fixtures"
RUNNER = WORKTREE + "/scripts/bench/run_bench.py"
spec = importlib.util.spec_from_file_location("run_bench", RUNNER)
rb = importlib.util.module_from_spec(spec)
sys.dont_write_bytecode = True
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


def all_ok(checks):
    return bool(checks) and all(c.ok for c in checks)


def any_bad(checks):
    return any(not c.ok for c in checks)


def mutate(obj, path, value):
    o = copy.deepcopy(obj)
    t = o
    for p in path[:-1]:
        t = t[p]
    if value is DELETE:
        del t[path[-1]]
    else:
        t[path[-1]] = value
    return o


DELETE = object()


def counters(**over):
    c = {k: 0 for k in ("parallelWrites", "retransmits", "abandonedWrites", "supersededWrites", "gateFallbacks",
                        "postStopFailSafes", "slotTimeouts", "notRespondingFallbacks", "deviceNotReadyFallbacks",
                        "writeFailures", "clearedAtInit", "orphanDatagrams", "numberClashes", "stockFallbacks")}
    c["nacks"] = {}
    c.update(over)
    return c


def clean_leftovers():
    return {h: {"inFlight": 0, "pendingEntries": 0, "standIns": 0, "routes": 0} for h in ("controlHub", "expansionHub")}


def phase(parallel=8, issued=600, burst=6, **over):
    depth = min(parallel, 6) if parallel > 0 else 0
    p = {"parallel": parallel, "countExact": True, "burstPerHub": burst, "drainBeforeMs": 0.01, "drainAfterMs": 1.2,
         "photonEnabled": parallel > 0, "inFlightAfterDrain": 0, "allAtEnd": True,
         "endReadback": {"checks": 12, "mismatches": 0, "events": []},
         "readback": {"checks": 600, "mismatches": 0, "events": []},
         "counters": counters(parallelWrites=issued if parallel > 0 else 0),
         "issuedWrites": {"controlHub": issued // 2, "expansionHub": issued // 2}, "issuedOnParallelHubs": issued,
         "maxInFlight": {"controlHub": depth, "expansionHub": depth},
         "maxPendingEntries": {"controlHub": depth, "expansionHub": depth},
         "maxStandIns": {"controlHub": depth, "expansionHub": depth},
         "leftoversAfterDrain": clean_leftovers(),
         "maxPendingMs": {"controlHub": 3.0, "expansionHub": 4.0}, "notResponding": [],
         "writeBlockMs": {"count": 10, "p50": 6.0, "p95": 12.0, "max": 20.0},
         "writeBlockPerHubMs": {"controlHub": {"count": 10, "p50": 2.0, "p95": 4.0, "max": 9.0},
                                "expansionHub": {"count": 10, "p50": 4.0, "p95": 8.0, "max": 12.0}},
         "loopPeriodMs": {"count": 10, "p50": 11.0, "p95": 18.0, "max": 25.0}}
    p.update(over)
    return p


def depth_trial(hub="controlHub", depth=0, **over):
    r = {"hub": hub, "channel": "fl", "kind": "motor", "depth": depth, "registerKeptOldValue": True, "writeCallMs": 0.5,
         "inFlightBeforeLostWrite": depth, "abandonedAtMs": 255.0, "busyWriteMs": {"count": 300, "max": 4.0},
         "tapLog": [{"msAfterArming": 0.1, "retransmit": False}, {"msAfterArming": 103.0, "retransmit": True},
                    {"msAfterArming": 206.0, "retransmit": True}],
         "drainAfterAbandonMs": 1.0, "leftoversAfterAbandon": {"inFlight": 0, "pendingEntries": 0, "standIns": 0, "routes": 0},
         "rewriteLanded": True, "othersLanded": True, "drainAfterMs": 1.0, "notRespondingAtEnd": False,
         "counters": counters(abandonedWrites=1, retransmits=2, notRespondingFallbacks=2, slotTimeouts=1)}
    for k, val in over.items():
        r = mutate(r, k.split("."), val)
    return r


def superseded_trial(**over):
    r = {"hub": "controlHub", "channel": "fr", "kind": "motor", "registerKeptOldValue": True, "newerWriteAtMs": 51.0,
         "tapLog": [{"msAfterArming": 0.1, "retransmit": False}],
         "leftovers": {"inFlight": 0, "pendingEntries": 0, "standIns": 0, "routes": 0}, "newerLanded": True,
         "notResponding": False, "counters": counters(supersededWrites=1)}
    for k, val in over.items():
        r = mutate(r, k.split("."), val)
    return r


I2C_HUBS = {"usb": {"pinpoint": ("expansionHub", 1), "octoquad": ("controlHub", 2)},
            "rs485": {"pinpoint": ("expansionHub", 1), "octoquad": ("controlHub", 2)},
            "none": {"pinpoint": ("controlHub", 1), "octoquad": ("controlHub", 2)}}
I2C_EXPECT = {"pinpoint": ("02000000", "03000000", 0x31), "octoquad": ("51030102", "030102", 0x30)}


def i2c_reader(device, client, topo="usb", swapped=True, reads=800, **over):
    hub, bus = I2C_HUBS[topo][device]
    a, b, address = I2C_EXPECT[device]
    r = {"device": device, "client": client, "hub": hub, "hubAddress": 173 if hub == "controlHub" else 2, "bus": bus,
         "i2cAddress": address, "register": 1,
         "expected": a if client == "driver" else b, "moduleSwapped": swapped and client != "replaced", "periodMs": 10.0,
         "reads": reads, "wrong": 0, "placeholders": 0, "firstWrong": None, "failure": None,
         "readMs": {"count": reads, "mean": 2.2, "p50": 2.0, "p95": 4.0, "p99": 6.0, "max": 9.0}}
    r.update(over)
    return r


def i2c_phase(parallel=8, topo="usb", swapped=True, replaced=False, reads=800):
    second = "replaced" if replaced else "second"
    readers = [i2c_reader(d, c, topo, swapped, reads) for d in ("pinpoint", "octoquad") for c in ("driver", second)]
    ph = phase(parallel=parallel, counters=counters(parallelWrites=600 if parallel > 0 else 0,
                                                    orphanDatagrams=3 * 2 * reads if replaced else 0))
    ph.update({"i2cReplacedReader": replaced, "i2cReaders": readers,
               "replacedPendingAfter": {"pinpoint": 0, "octoquad": 0} if replaced else {}})
    return ph


def i2c_info(topo="usb", swapped=True):
    devices = []
    for d in ("pinpoint", "octoquad"):
        hub, bus = I2C_HUBS[topo][d]
        a, b, address = I2C_EXPECT[d]
        devices.append({"device": d, "hub": hub, "hubAddress": 173 if hub == "controlHub" else 2, "bus": bus,
                        "i2cAddress": address, "clientClass": "LynxI2cDeviceSynchV2", "clientHub": hub,
                        "clientModuleSwapped": swapped, "secondHub": hub, "secondBus": bus, "secondI2cAddress": address,
                        "secondModuleSwapped": swapped, "registerA": 1, "expectedA": a, "registerB": 2, "expectedB": b,
                        "readersDistinguishable": True})
    return {"devices": devices, "periodMs": 10.0, "photonSwapFailures": []}


def i2c_control(topo="usb", attempts=200, wrong=0, refused=None, locked_wrong=0):
    out = {}
    for d in ("pinpoint", "octoquad"):
        out[d] = {"hub": I2C_HUBS[topo][d][0], "attempted": attempts,
                  "unlockedRight": attempts - wrong - sum((refused or {}).values()), "unlockedWrong": wrong,
                  "unlockedFirstWrong": None, "unlockedRefused": dict(refused or {}),
                  "unlockedReadMs": {"count": attempts, "p50": 2.0},
                  "lockedReader": i2c_reader(d, "driver", topo, True, 900, periodMs=0.0, wrong=locked_wrong)}
    return out


# --- one workload phase
good = phase()
expect("phase: good on8 passes", rb.photon_phase_problems(good) == [])
expect("phase: good off passes", rb.photon_phase_problems(phase(parallel=0)) == [])
expect("phase: good stock passes", rb.photon_phase_problems(phase(parallel=-1)) == [])
broken = {
    "read-back mismatch": mutate(good, ["readback", "mismatches"], 1),
    "end read-back": mutate(good, ["allAtEnd"], False),
    "drain NaN (null)": mutate(good, ["drainAfterMs"], None),
    "drain over limit": mutate(good, ["drainAfterMs"], 51.0),
    "drain before NaN": mutate(good, ["drainBeforeMs"], None),
    "left in flight": mutate(good, ["inFlightAfterDrain"], 1),
    "over limit": mutate(good, ["maxInFlight", "controlHub"], 9),
    "stale": mutate(good, ["maxPendingMs", "expansionHub"], 400.0),
    "stale past one retransmit interval": mutate(good, ["maxPendingMs", "expansionHub"], 150.0),
    "retransmitted in a workload phase": mutate(good, ["counters", "retransmits"], 1),
    "SDK stand-in left after the drain": mutate(good, ["leftoversAfterDrain", "expansionHub", "standIns"], 1),
    "route left after the drain": mutate(good, ["leftoversAfterDrain", "controlHub", "routes"], 1),
    "pending entry left after the drain": mutate(good, ["leftoversAfterDrain", "controlHub", "pendingEntries"], 2),
    "no leftover report": mutate(good, ["leftoversAfterDrain"], DELETE),
    "incomplete leftover report": mutate(good, ["leftoversAfterDrain", "controlHub"], {"inFlight": 0}),
    "SDK stand-ins over the limit": mutate(good, ["maxStandIns", "controlHub"], 9),
    "pending entries over the limit": mutate(good, ["maxPendingEntries", "expansionHub"], 9),
    "no SDK stand-in sampling": mutate(good, ["maxStandIns"], DELETE),
    "a stand-in with Photon off": mutate(phase(parallel=0), ["maxStandIns", "controlHub"], 1),
    "abandoned": mutate(good, ["counters", "abandonedWrites"], 1),
    "superseded": mutate(good, ["counters", "supersededWrites"], 1),
    "write failure": mutate(good, ["counters", "writeFailures"], 1),
    "device not ready": mutate(good, ["counters", "deviceNotReadyFallbacks"], 1),
    "not responding fallback": mutate(good, ["counters", "notRespondingFallbacks"], 1),
    "gate fallback": mutate(good, ["counters", "gateFallbacks"], 1),
    "orphan datagrams": mutate(good, ["counters", "orphanDatagrams"], 2),
    "a reply dropped as a message-number clash": mutate(good, ["counters", "numberClashes"], 1),
    "cleared at init": mutate(good, ["counters", "clearedAtInit"], 1),
    "nack": mutate(good, ["counters", "nacks"], {"BATTERY_TOO_LOW_TO_RUN_MOTOR": 1}),
    "unexpected slot timeout": mutate(mutate(good, ["counters", "slotTimeouts"], 1), ["counters", "parallelWrites"], 599),
    "write skipped Photon": mutate(good, ["counters", "parallelWrites"], 599),
    "parallel writes while off": mutate(phase(parallel=0), ["counters", "parallelWrites"], 3),
    "not enabled": mutate(good, ["photonEnabled"], False),
    "not responding": mutate(good, ["notResponding"], [2]),
    "low voltage reads": mutate(good, ["voltageReadsUnder9V"], 3),
    "no counters": mutate(good, ["counters"], DELETE),
}
for label, ph in broken.items():
    expect(f"phase: {label} fails", rb.photon_phase_problems(ph) != [])
expect("phase: empty result fails", rb.photon_phase_problems({}) != [])
expect("phase: slot timeouts allowed past the limit",
       rb.photon_phase_problems(phase(parallel=2, counters=counters(parallelWrites=590, slotTimeouts=10),
                                      maxInFlight={"controlHub": 2, "expansionHub": 2},
                                      maxPendingEntries={"controlHub": 2, "expansionHub": 2},
                                      maxStandIns={"controlHub": 2, "expansionHub": 2})) == [])
expect("phase: workload checks wrap it", all_ok(rb.photon_workload_checks("phases", "", {"a": good}))
       and any_bad(rb.photon_workload_checks("phases", "", {"a": broken["abandoned"]})))

# --- stock baseline and port probe
stock = {"info": {"hubs": [{"key": "controlHub", "address": 173, "swapped": False, "class": "LynxModule"},
                           {"key": "expansionHub", "address": 2, "swapped": False, "class": "LynxModule"}],
                  "inFlightAtInit": 0, "photonEnabledAtInit": False},
         "enableRefused": {"refused": True, "message": "needs @Photon"},
         "portProbe": {"empty": True, "power": 0.1, "limitAmps": 0.1,
                       "ports": [{"channel": "fl", "empty": True, "drivenAmps": 0.0}]},
         "timing": {"stock": phase(parallel=-1, writeBlockPerHubMs={"controlHub": {"count": 9, "p50": 9.0, "p95": 19, "max": 29},
                                                                   "expansionHub": {"count": 9, "p50": 16.0, "p95": 26, "max": 41}})}}
expect("stock: good passes", all_ok(rb.photon_stock_checks(stock)))
expect("stock: swapped hub fails", any_bad(rb.photon_stock_checks(mutate(stock, ["info", "hubs", 0, "swapped"], True))))
expect("stock: enable allowed fails", any_bad(rb.photon_stock_checks(mutate(stock, ["enableRefused", "refused"], False))))
expect("stock: wrong read-back fails", any_bad(rb.photon_stock_checks(mutate(stock, ["timing", "stock", "readback", "mismatches"], 2))))
expect("ports: empty passes", all_ok(rb.photon_port_checks(stock)))
expect("ports: busy fails", any_bad(rb.photon_port_checks(mutate(mutate(stock, ["portProbe", "empty"], False),
                                                                  ["portProbe", "ports", 0, "empty"], False))))

# --- gain
main = {"timing": {"off": phase(parallel=0, writeBlockPerHubMs={"controlHub": {"count": 9, "p50": 9.2, "p95": 19, "max": 29},
                                                               "expansionHub": {"count": 9, "p50": 16.1, "p95": 26, "max": 41}}),
                   "on8": phase()}}
expect("gain: faster and no disabled cost passes", all_ok(rb.photon_gain_checks(stock, main, ["controlHub", "expansionHub"])))
expect("gain: no speed-up fails", any_bad(rb.photon_gain_checks(stock, mutate(main, ["timing", "on8", "writeBlockPerHubMs", "controlHub", "p50"], 9.5),
                                                               ["controlHub"])))
expect("gain: disabled costs more fails", any_bad(rb.photon_gain_checks(stock, mutate(main, ["timing", "off", "writeBlockPerHubMs", "expansionHub", "p50"], 25.0),
                                                                       ["expansionHub"])))
expect("gain: rows built", len(rb.photon_gain_rows(stock, main)) == 3)

# --- installed Photon
manifest = {"photonSourceSha256": "ab" * 32}
installed = mutate(stock, ["info", "installedPhotonSourceSha256"], "ab" * 32)
expect("install: matching hash passes", all_ok(rb.photon_install_checks(installed, manifest)))
expect("install: another install fails", any_bad(rb.photon_install_checks(
    mutate(installed, ["info", "installedPhotonSourceSha256"], "cd" * 32), manifest)))
expect("install: no hash from the hub fails", any_bad(rb.photon_install_checks(stock, manifest)))
expect("install: the runner hashes like build.gradle", rb.photon_source_sha256(rb.REPO_ROOT / rb.PHOTON_SOURCE_DIR)
       == __import__("re").search(r'PHOTON_SOURCE_SHA256 = "([0-9a-f]{64})"', open(
           WORKTREE + "/TeamCode/build/generated/source/buildConfig/debug/com/qualcomm/"
           "ftcrobotcontroller/BuildConfig.java").read()).group(1))


# --- STOP sampler
def outputs(violate=None):
    rows = [{"channel": c, "hub": "controlHub", "kind": k, "enabled": False, "raw": 0}
            for c, k in (("fl", "motor"), ("fr", "motor"), ("benchServo", "servo"))]
    if violate == "motor":
        rows[0] = dict(rows[0], enabled=True, raw=6553)
    if violate == "servo":
        rows[2] = dict(rows[2], enabled=True, raw=1500)
    if violate == "zeroed":
        rows[0] = dict(rows[0], enabled=True, raw=0)
    return {"gateUp": True, "photonEnabled": False, "outputs": rows}


def sampler(violate=None, servo_on=False, never_safe=False, gate=True, failure=None, error=None, read=None):
    kinds = ["motor", "motor", "servo"]
    rows = []
    t = 0.0
    for i in range(40):
        gate_up = 1 if gate and i >= 10 else 0
        if gate_up and i >= 12 and not never_safe:
            vals = [0, 0, 0, 0, 0, 0]
        else:
            vals = [1, 6553, 1, 8191, 1, 0]
        if violate is not None and i == violate:
            vals = [1, 6553, 0, 0, 0, 0]
        if servo_on and i == 20:
            vals = [0, 0, 0, 0, 1, 0]
        if i >= 25 and not never_safe:
            vals = [1, 0, 1, 0, 0, 0]
        rows.append([t, t + 9, gate_up] + vals)
        t += 10
    return {"complete": failure is None and gate, "gateSeen": gate, "failure": failure,
            "loopAtGate": {"fractionOfLoop": 0.3},
            "extra": {"stopSawBeforeZeroing": read if read is not None else outputs(),
                      "countersAtStop": counters()},
            "hubs": [{"key": "controlHub", "address": 173, "kinds": kinds, "channels": ["fl", "fr", "benchServo"],
                      "gateMs": 100.0, "sweeps": rows, "error": error}]}


def iterative_sampler(late=False):
    """STOP's fail-safe lands mid-sweep and stop()'s zeroMotors re-enables the motors at 0 within one sweep, so no
    sweep ever reads every channel disabled; with late, a single late write re-energized a motor in between, which
    only stop()'s read saw."""
    kinds = ["motor", "motor", "servo"]
    rows = []
    for i in range(30):
        gate_up = 1 if i >= 10 else 0
        vals = [1, 6553, 1, 8191, 1, 0] if i <= 10 else [1, 0, 1, 0, 0, 0]
        rows.append([i * 16.0, i * 16.0 + 15, gate_up] + vals)
    return {"complete": True, "gateSeen": True, "failure": None, "loopAtGate": {"fractionOfLoop": 0.6},
            "extra": {"stopSawBeforeZeroing": outputs("motor" if late else None), "countersAtStop": counters()},
            "hubs": [{"key": "expansionHub", "address": 2, "kinds": kinds, "channels": ["efl", "efr", "eServo"],
                      "gateMs": 160.0, "sweeps": rows, "error": None}]}


def trial(tid, smp, **over):
    t = {"trial": tid, "step": f"photon_stop_{tid}", "crashed": False, "appRestarted": False, "error": "", "sampler": smp}
    t.update(over)
    return t


v = rb.stop_trial_verdict(sampler(), require_zeroing_read=True)
expect("stop verdict: clean trial has no violations or problems", v["violations"] == 0 and not v["problems"])
expect("stop verdict: re-energized motor counted", rb.stop_trial_verdict(sampler(violate=15))["violations"] == 1)
expect("stop verdict: re-enabled servo counted", rb.stop_trial_verdict(sampler(servo_on=True))["violations"] == 1)
expect("stop verdict: enabled at power 0 is not a violation", v["violations"] == 0)
expect("stop verdict: fail-safe never seen is a problem", rb.stop_trial_verdict(sampler(never_safe=True))["problems"])
expect("stop verdict: gate never seen is a problem", rb.stop_trial_verdict(sampler(gate=False))["problems"])
expect("stop verdict: sampler failure is a problem", rb.stop_trial_verdict(sampler(failure="boom"))["problems"])
expect("stop verdict: thread error is a problem", rb.stop_trial_verdict(sampler(error="NACK"))["problems"])
expect("stop verdict: missing file is a problem", rb.stop_trial_verdict(None)["problems"])
expect("stop verdict: malformed sweep is a problem",
       rb.stop_trial_verdict(mutate(sampler(), ["hubs", 0, "sweeps", 3], [0, 1, 0]))["problems"])
v = rb.stop_trial_verdict(iterative_sampler(), require_zeroing_read=True)
expect("stop verdict: iterative, fail-safe window shorter than a sweep and motors zeroed at 0, is clean",
       not v["problems"] and v["violations"] == 0 and v["failSafeMsAfterGate"])
v = rb.stop_trial_verdict(iterative_sampler(late=True), require_zeroing_read=True)
expect("stop verdict: iterative, one late re-energize seen only by stop()'s read is a violation",
       v["zeroingReadViolations"] == 1 and v["samplerViolations"] == 0)
expect("stop verdict: a servo enabled in stop()'s read is a violation",
       rb.stop_trial_verdict(sampler(read=outputs("servo")))["zeroingReadViolations"] == 1)
expect("stop verdict: a motor enabled at 0 in stop()'s read is fine",
       rb.stop_trial_verdict(sampler(read=outputs("zeroed")))["zeroingReadViolations"] == 0)
expect("stop verdict: stop()'s read missing is a problem when required",
       rb.stop_trial_verdict(mutate(sampler(), ["extra"], {}), require_zeroing_read=True)["problems"])
expect("stop verdict: stop()'s read with the gate down is a problem",
       rb.stop_trial_verdict(sampler(read=dict(outputs(), gateUp=False)))["problems"])
expect("stop verdict: sweep period reported", rb.stop_trial_verdict(iterative_sampler())["sweepMs"] == {"expansionHub": 16.0})
trials = [trial(f"t{i}", mutate(sampler(), ["loopAtGate", "fractionOfLoop"], (i % 4) / 4 + 0.1)) for i in range(8)]
expect("stop checks: clean trials pass", all_ok(rb.photon_stop_checks("on8", trials)))
expect("stop checks: clean iterative short-window trials pass", all_ok(rb.photon_stop_checks(
    "on8", [trial(f"i{i}", iterative_sampler()) for i in range(4)])))
expect("stop checks: one late re-energize seen only in stop()'s read fails", any_bad(rb.photon_stop_checks(
    "on8", trials[:-1] + [trial("late", iterative_sampler(late=True))])))
bad_trials = trials[:-1] + [trial("bad", sampler(violate=15))]
expect("stop checks: one violating trial fails", any_bad(rb.photon_stop_checks("on8", bad_trials)))
expect("stop checks: no trials fails", any_bad(rb.photon_stop_checks("on8", [])))
expect("stop checks: a crashed trial fails", any_bad(rb.photon_stop_checks(
    "on8", trials[:-1] + [trial("c", sampler(), crashed=True, error="didn't take its fail-safe after STOP")])))
expect("stop checks: an app restart fails", any_bad(rb.photon_stop_checks("on8", trials[:-1] + [trial("r", sampler(), appRestarted=True)])))
expect("stop checks: an error line fails", any_bad(rb.photon_stop_checks("on8", trials[:-1] + [trial("e", sampler(), error="boom")])))
expect("stop checks: the detail states the violation-rate bound", "95% bound" in rb.photon_stop_checks("on8", trials)[0].detail)
expect("stop bound: 40 trials bound 7.5%", abs(rb.photon_stop_bound(40) - 0.075) < 1e-9)
expect("stop coverage: all quarters passes", rb.photon_stop_coverage("on8", trials).ok)
expect("stop coverage: one quarter fails", not rb.photon_stop_coverage("on8", trials[:1] * 8).ok)
held = [trial("c", mutate(sampler(violate=15), ["extra", "negativeControl"], {"variant": "held", "samplerSawFailSafeFirst": True}))]
expect("negative control held: violations seen passes", all_ok(rb.photon_stop_checks("ctl", held, expect="held")))
expect("negative control held: no violation fails", any_bad(rb.photon_stop_checks(
    "ctl", [trial("c", mutate(sampler(), ["extra", "negativeControl"], {"samplerSawFailSafeFirst": True}))], expect="held")))
expect("negative control held: re-energized before the sampler saw the fail-safe fails", any_bad(rb.photon_stop_checks(
    "ctl", [mutate(held[0], ["sampler", "extra", "negativeControl", "samplerSawFailSafeFirst"], False)], expect="held")))
expect("negative control held: a crashed control trial fails", any_bad(rb.photon_stop_checks(
    "ctl", [dict(held[0], crashed=True, error="x")], expect="held")))
once = [trial("o", mutate(iterative_sampler(late=True), ["extra", "negativeControl"], {"variant": "once"}))]
expect("negative control once: stop()'s read catches it passes", all_ok(rb.photon_stop_checks("ctl", once, expect="once")))
expect("negative control once: missed fails", any_bad(rb.photon_stop_checks(
    "ctl", [trial("o", iterative_sampler())], expect="once")))
expect("negative control once: the sweep period is reported",
       "expansionHub 16-16 ms" in rb.photon_stop_checks("ctl", once, expect="once")[0].detail)

# --- STOP inside a write
race = [{"result": {"photon": p, "gateRaisedInsideWrite": True, "violation": False, "hub": "controlHub",
                    "reads": []}} for p in (8, 8, 0)]
expect("race: no violation passes", all_ok(rb.photon_race_checks(race)))
expect("race: violation with Photon on fails", any_bad(rb.photon_race_checks(mutate(race, [0, "result", "violation"], True))))
expect("race: tap never fired fails", any_bad(rb.photon_race_checks(mutate(race, [1, "result", "gateRaisedInsideWrite"], False))))
expect("race: violation on the SDK path fails too", any_bad(rb.photon_race_checks(mutate(race, [2, "result", "violation"], True))))

# --- lost writes
lost = {"hub": "controlHub", "kind": "motor", "channel": "fl", "primedLanded": True, "registerKeptOldValue": True,
        "writeCallMs": 0.6, "abandonedAtMs": 256.0, "inFlightAfterAbandon": 0, "pendingEntriesAfterAbandon": 0,
        "standInsAfterAbandon": 0, "sdkCacheValidAfterAbandon": False, "rewriteMsAfterLostWrite": 262.0, "rewriteLanded": True,
        "otherHubAllAt": True, "otherWriteMs": {"count": 50, "max": 3.0}, "drainAfterMs": 1.0, "notRespondingAtEnd": False,
        "tapLog": [{"msAfterArming": 0.1, "retransmit": False}, {"msAfterArming": 104.0, "retransmit": True},
                   {"msAfterArming": 207.0, "retransmit": True}],
        "counters": counters(abandonedWrites=1, retransmits=2, notRespondingFallbacks=1)}
over = {"hub": "controlHub", "limit": 4, "inFlightWithAllLost": 4, "overflowWriteMs": {"count": 2, "max": 7.0},
        "slotTimeoutsSoFar": 2, "registersKeptOldValues": True, "allAbandonedAtMs": 258.0, "inFlightAfterAbandon": 0,
        "standInsAfterAbandon": 0,
        "rewritesLanded": True, "counters": counters(abandonedWrites=4, slotTimeouts=2)}
faults = {"lostWrite": [lost], "lostOverLimit": [over]}
expect("lost: good passes", all_ok(rb.photon_lost_checks(faults)))
lost_broken = {
    "tap didn't stop it": ["lostWrite", 0, "registerKeptOldValue", False],
    "call blocked": ["lostWrite", 0, "writeCallMs", 25.0],
    "never given up": ["lostWrite", 0, "abandonedAtMs", None],
    "given up late": ["lostWrite", 0, "abandonedAtMs", 400.0],
    "missing retransmit": ["lostWrite", 0, "tapLog", [{"msAfterArming": 0.1, "retransmit": False}]],
    "retransmit late": ["lostWrite", 0, "tapLog", [{"msAfterArming": 104.0, "retransmit": True}, {"msAfterArming": 300.0, "retransmit": True}]],
    "still in flight": ["lostWrite", 0, "inFlightAfterAbandon", 1],
    "SDK stand-in left": ["lostWrite", 0, "standInsAfterAbandon", 1],
    "over: SDK stand-in left": ["lostOverLimit", 0, "standInsAfterAbandon", 1],
    "cache still valid": ["lostWrite", 0, "sdkCacheValidAfterAbandon", True],
    "re-write too late to judge": ["lostWrite", 0, "rewriteMsAfterLostWrite", 520.0],
    "re-write skipped": ["lostWrite", 0, "rewriteLanded", False],
    "other hub broken": ["lostWrite", 0, "otherHubAllAt", False],
    "two abandoned": ["lostWrite", 0, "counters", counters(abandonedWrites=2)],
    "nack": ["lostWrite", 0, "counters", counters(abandonedWrites=1, nacks={"X": 1})],
    "over: not full": ["lostOverLimit", 0, "inFlightWithAllLost", 3],
    "over: blocked": ["lostOverLimit", 0, "overflowWriteMs", {"count": 2, "max": 60.0}],
    "over: no slot timeout": ["lostOverLimit", 0, "slotTimeoutsSoFar", 0],
    "over: re-writes lost": ["lostOverLimit", 0, "rewritesLanded", False],
    "over: left in flight": ["lostOverLimit", 0, "inFlightAfterAbandon", 2],
}
for label, path in lost_broken.items():
    expect(f"lost: {label} fails", any_bad(rb.photon_lost_checks(mutate(faults, path[:-1], path[-1]))))

expect("lost: all trials present passes", all_ok(rb.photon_lost_checks({"lostWrite": [lost] * 2, "lostOverLimit": [over]},
                                                                  ["controlHub"], 1)))
expect("lost: a missing trial fails", any_bad(rb.photon_lost_checks({"lostWrite": [lost], "lostOverLimit": [over]},
                                                                  ["controlHub"], 1)))
expect("lost: no results at all fails when trials were expected", any_bad(rb.photon_lost_checks({}, ["controlHub"], 3)))

depth_faults = {"lostAmongWrites": [depth_trial(depth=d) for d in range(8) for _ in range(2)]}
expect("lost depth: every depth covered passes", all_ok(rb.photon_lost_depth_checks(depth_faults, ["controlHub"], 2)))
expect("lost depth: a depth short of trials fails", any_bad(rb.photon_lost_depth_checks(
    {"lostAmongWrites": depth_faults["lostAmongWrites"][:-1]}, ["controlHub"], 2)))
expect("lost depth: no trials on a hub fails", any_bad(rb.photon_lost_depth_checks(depth_faults, ["controlHub", "expansionHub"], 2)))
for label, over in (("stand-in left at depth 5", {"leftoversAfterAbandon.standIns": 1}),
                    ("a newer write's ACK ended it as superseded", {"counters": counters(supersededWrites=1)}),
                    ("never given up", {"abandonedAtMs": None}),
                    ("a write around it didn't land", {"othersLanded": False}),
                    ("the writes around it blocked", {"busyWriteMs.max": 30.0}),
                    ("tap missed", {"registerKeptOldValue": False}),
                    ("a retransmit missing", {"tapLog": [{"msAfterArming": 0.1, "retransmit": False}]}),
                    ("re-write lost", {"rewriteLanded": False})):
    broken_trials = depth_faults["lostAmongWrites"][:-1] + [depth_trial(depth=7, **over)]
    expect(f"lost depth: {label} fails", any_bad(rb.photon_lost_depth_checks({"lostAmongWrites": broken_trials}, ["controlHub"], 2)))


sup = {"lostThenSuperseded": [superseded_trial() for _ in range(3)]}
expect("superseded: ends quietly passes", all_ok(rb.photon_superseded_checks(sup, ["controlHub"], 3)))
expect("superseded: too few trials fails", any_bad(rb.photon_superseded_checks(sup, ["controlHub"], 4)))
for label, over in (("retransmitted over the newer value", {"tapLog": [{"msAfterArming": 0.1, "retransmit": False},
                                                                        {"msAfterArming": 103.0, "retransmit": True}]}),
                    ("given up instead", {"counters": counters(abandonedWrites=1)}),
                    ("hub marked unresponsive", {"notResponding": True}),
                    ("newer value lost", {"newerLanded": False}),
                    ("stand-in left", {"leftovers.standIns": 1}),
                    ("newer write too late to test it", {"newerWriteAtMs": 120.0})):
    expect(f"superseded: {label} fails", any_bad(rb.photon_superseded_checks(
        {"lostThenSuperseded": sup["lostThenSuperseded"][:2] + [superseded_trial(**over)]}, ["controlHub"], 3)))

throws = {"writeThrows": [{"hub": "controlHub", "thrown": 1, "writeCallMs": 2.1, "landed": True, "inFlightAfter": 0,
                           "notResponding": False, "shutdownReason": "NORMAL", "counters": counters(writeFailures=1)}]}
expect("write failure: SDK fallback passes", all_ok(rb.photon_write_failure_checks(throws, ["controlHub"])))
for key, value in (("thrown", 0), ("landed", False), ("shutdownReason", "ABNORMAL"), ("inFlightAfter", 1),
                   ("writeCallMs", 40.0), ("counters", counters(writeFailures=0)), ("counters", counters(writeFailures=1, parallelWrites=1))):
    expect(f"write failure: {key}={value} fails", any_bad(rb.photon_write_failure_checks(mutate(throws, ["writeThrows", 0, key], value))))
expect("write failure: missing hub fails", any_bad(rb.photon_write_failure_checks(throws, ["controlHub", "expansionHub"])))

# --- reset recovery
reset = {"resetRecovery": {"controlHub": {
    "off": {"settledBeforeReset": True, "resetSeen": True, "sdkCacheValidAfterRawFailSafe": True,
            "recoveredMs": {"fl (WriteCache)": 210.0, "fr": 215.0}},
    "on8": {"settledBeforeReset": True, "resetSeen": True, "sdkCacheValidAfterRawFailSafe": True,
            "recoveredMs": {"fl (WriteCache)": 212.0, "fr": 220.0}}}}}
expect("reset: same recovery passes", all_ok(rb.photon_reset_checks(reset)))
expect("reset: slower with Photon fails", any_bad(rb.photon_reset_checks(
    mutate(reset, ["resetRecovery", "controlHub", "on8", "recoveredMs", "fr"], 400.0))))
expect("reset: never back fails", any_bad(rb.photon_reset_checks(
    mutate(reset, ["resetRecovery", "controlHub", "on8", "recoveredMs", "fr"], None))))
expect("reset: a hub missing fails", any_bad(rb.photon_reset_checks(reset, ["controlHub", "expansionHub"])))
expect("reset: every hub present passes", all_ok(rb.photon_reset_checks(reset, ["controlHub"])))
expect("reset: reset not seen fails", any_bad(rb.photon_reset_checks(
    mutate(reset, ["resetRecovery", "controlHub", "off", "resetSeen"], False))))
expect("reset: cache-expiry recovery is labelled so", "cache expiry" in rb.photon_reset_checks(reset)[0].name)
expect("reset: a reset the SDK learned from the hub is labelled attention-driven", "attention-driven" in rb.photon_reset_checks(
    mutate(reset, ["resetRecovery", "controlHub", "on8", "sdkCacheValidAfterRawFailSafe"], False))[0].name)

# --- storm, threads, soak
storm = phase(parallel=1, burst=3000, counters=counters(parallelWrites=2990, slotTimeouts=10),
              maxInFlight={"controlHub": 1, "expansionHub": 1}, issuedOnParallelHubs=3000, maxStallMs=8.0,
              writes=3000, writeMs={"max": 7.0})
expect("storm: bounded passes", all_ok(rb.photon_storm_checks(storm)))
expect("storm: stall fails", any_bad(rb.photon_storm_checks(mutate(storm, ["maxStallMs"], 80.0))))
expect("storm: two in flight at a limit of 1 fails", any_bad(rb.photon_storm_checks(mutate(storm, ["maxInFlight", "controlHub"], 2))))
two = phase(countExact=False, issuedOnParallelHubs=0, counters=counters(parallelWrites=5000), toggles=50,
            servoWrites=2000, motorWrites=4000)
expect("threads: good passes", all_ok(rb.photon_thread_checks(two)))
expect("threads: wrong read-back fails", any_bad(rb.photon_thread_checks(mutate(two, ["readback", "mismatches"], 1))))
expect("threads: toggler never ran fails", any_bad(rb.photon_thread_checks(mutate(two, ["toggles"], 0))))
expect("threads: no motor read-backs during the phase fails", any_bad(rb.photon_thread_checks(mutate(two, ["readback", "checks"], 0))))
expect("threads: two threads over the limit fails", any_bad(rb.photon_thread_checks(mutate(two, ["maxInFlight", "controlHub"], 9))))
expect("threads: a route left behind fails", any_bad(rb.photon_thread_checks(mutate(two, ["leftoversAfterDrain", "controlHub", "routes"], 1))))
soak = phase(issued=2 * 7_000_000, issuedWrites={"controlHub": 7_000_000, "expansionHub": 7_000_000}, seconds=7000.0)
checks, met = rb.photon_soak_checks(soak)
expect("soak: 7e6 clean writes per hub meets the bound", all_ok(checks) and met)
checks, met = rb.photon_soak_checks(phase(issued=120000, issuedWrites={"controlHub": 60000, "expansionHub": 60000}))
expect("soak: a short soak doesn't meet the bound", checks[0].ok and not checks[1].ok and not met)
checks, met = rb.photon_soak_checks(mutate(soak, ["counters", "abandonedWrites"], 1))
expect("soak: one lost write fails", not checks[0].ok and not met)
checks, met = rb.photon_soak_checks(mutate(soak, ["readback", "mismatches"], 1))
expect("soak: one wrong read-back fails", not checks[0].ok and not met)
checks, met = rb.photon_soak_checks(mutate(soak, ["counters", "retransmits"], 3))
expect("soak: retransmits fail", not checks[0].ok and not met)


# --- routing
def routing(order=(173, 2), **over):
    r = {"atInit": {"photonEnabled": False, "swapFailures": [], "routingProblems": [], "swapOrder": list(order),
                    "inFlight": {"173": 0, "2": 0}, "pendingEntries": {"173": 0, "2": 0},
                    "standIns": {"173": 0, "2": 0}, "routes": {"173": 0, "2": 0},
                    "counters": counters(), "loaders": {"pid": 100, "photonCoreLoader": "PathClassLoader@1",
                                                        "benchLoader": "PathClassLoader@1", "photonCorePinned": True,
                                                        "photonModulePinned": True,
                                                        "hubs": [{"identity": 11}, {"identity": 22}]}},
         "notRespondingRecovery": {m: {h: {"visible": True, "clearedMs": 2.5, "voltsAfter": 12.4, "notRespondingAtEnd": False}
                                       for h in ("controlHub", "expansionHub")} for m in ("off", "on8")},
         "parallelCheck": {"drainMs": 1.0, "issuedOnParallelHubs": 240, "counters": counters(parallelWrites=240),
                           "readback": {"checks": 12, "mismatches": 0, "events": []}, "routingProblemsAfter": [],
                           "leftoversAfterDrain": clean_leftovers()}}
    for path, value in over.items():
        r = mutate(r, path.split("."), value)
    return r


expect("routing: good passes", rb.photon_routing_problems(routing()) == [])
routing_broken = {"atInit.routingProblems": ["misrouted"], "atInit.swapFailures": ["x"], "atInit.photonEnabled": True,
                  "atInit.inFlight": {"173": 1}, "atInit.pendingEntries": {"2": 2},
                  "notRespondingRecovery.on8.expansionHub.visible": False,
                  "notRespondingRecovery.on8.expansionHub.clearedMs": None,
                  "notRespondingRecovery.off.controlHub.clearedMs": 80.0,
                  "notRespondingRecovery.off.controlHub.voltsAfter": 0.0,
                  "parallelCheck.issuedOnParallelHubs": 241, "parallelCheck.drainMs": None,
                  "parallelCheck.routingProblemsAfter": ["x"],
                  "atInit.standIns": {"2": 1}, "atInit.routes": {"173": 1},
                  "parallelCheck.leftoversAfterDrain.expansionHub.standIns": 1}
for path, value in routing_broken.items():
    expect(f"routing: {path} fails", rb.photon_routing_problems(routing(**{path: value})) != [])
expect("routing: no SDK stand-in count at INIT fails", rb.photon_routing_problems(
    mutate(routing(), ["atInit", "standIns"], DELETE)) != [])
runs = [(f"restart {i}", routing(order=(173, 2) if i % 2 else (2, 173))) for i in range(1, 9)]
checks, both = rb.photon_routing_checks(runs, 8, 2)
expect("routing orders: 8 restarts, both orders passes", all_ok(checks) and both)
checks, both = rb.photon_routing_checks([(f"restart {i}", routing()) for i in range(1, 17)], 8, 2)
expect("routing orders: one order only fails", any_bad(checks) and not both)
checks, both = rb.photon_routing_checks(runs[:5], 8, 2)
expect("routing orders: too few restarts fails", any_bad(checks))
checks, both = rb.photon_routing_checks([(f"restart {i}", routing(order=(173,))) for i in range(1, 9)], 8, 1)
expect("routing orders: one hub can't see both orders", any_bad(checks) and not both)

main_info = {"info": {"routingProblems": [], "photonSwapFailures": [], "inFlightAtInit": 0, "photonEnabledAtInit": False,
                      "leftoversAtInit": clean_leftovers(),
                      "hubs": [{"address": 173, "swapped": True}, {"address": 2, "swapped": True}]}}
expect("main INIT: swapped and routed passes", all_ok(rb.photon_main_init_checks(main_info)))
for path, value in ((["info", "routingProblems"], ["x"]), (["info", "photonSwapFailures"], ["final field"]),
                    (["info", "inFlightAtInit"], 2), (["info", "hubs", 1, "swapped"], False), (["info"], {}),
                    (["info", "leftoversAtInit", "controlHub", "standIns"], 1), (["info", "leftoversAtInit"], DELETE)):
    expect(f"main INIT: {path[-1]} fails", any_bad(rb.photon_main_init_checks(mutate(main_info, path, value))))
expect("main INIT: no result fails", any_bad(rb.photon_main_init_checks({})))

# --- consecutive, crash, sloth
seq = [("a", "routing", routing()), ("b", "stock", {"info": {"inFlightAtInit": 0, "photonEnabledAtInit": False,
                                                             "leftoversAtInit": clean_leftovers()},
                                                    "enableRefused": {"refused": True}}),
       ("c", "routing", routing()), ("e", "routing", routing())]
expect("consecutive: clean passes", all_ok(rb.photon_consecutive_checks(seq)))
expect("consecutive: in flight at a stock INIT fails", any_bad(rb.photon_consecutive_checks(
    mutate(seq, [1, 2, "info", "inFlightAtInit"], 2))))
expect("consecutive: an SDK stand-in at a stock INIT fails", any_bad(rb.photon_consecutive_checks(
    mutate(seq, [1, 2, "info", "leftoversAtInit", "expansionHub", "standIns"], 1))))
expect("consecutive: leftover at a Photon INIT fails", any_bad(rb.photon_consecutive_checks(
    [seq[0], seq[1], seq[2], ("e", "routing", routing(**{"atInit.inFlight": {"2": 3}}))])))
expect("consecutive: write failures rising fails", any_bad(rb.photon_consecutive_checks(
    [seq[0], seq[1], ("e", "routing", routing(**{"atInit.counters": counters(writeFailures=2)}))])))
expect("consecutive: writes given up before the crash fails", any_bad(rb.photon_consecutive_checks(
    [seq[0], seq[1], ("c", "routing", routing(**{"atInit.counters": counters(abandonedWrites=2)})), seq[3]])))
expect("consecutive: the crash's own lost writes don't count against it", all_ok(rb.photon_consecutive_checks(
    [seq[0], seq[1], seq[2], ("e", "routing", routing(**{"atInit.counters": counters(supersededWrites=4)}))])))
expect("consecutive: hub objects replaced fails", any_bad(rb.photon_consecutive_checks(
    [seq[0], ("e", "routing", routing(**{"atInit.loaders.hubs": [{"identity": 99}, {"identity": 22}]}))])))
crash = {"outcome": {"crashed": True, "appRestarted": False},
         "main": {"inFlightAtThrow": 5, "lostAtThrow": {"controlHub": 2, "expansionHub": 2}}, "sampler": sampler()}
expect("crash: clean passes", all_ok(rb.photon_crash_checks(crash)))
before_crash, after_crash = routing(), routing(**{"atInit.counters": counters(supersededWrites=3, abandonedWrites=1)})
expect("crash: every never-acknowledged write finished and the next INIT clean passes",
       all_ok(rb.photon_crash_checks(crash, before_crash, after_crash)))
expect("crash: a never-acknowledged write unaccounted for fails", any_bad(rb.photon_crash_checks(
    crash, before_crash, routing(**{"atInit.counters": counters(supersededWrites=3)}))))
expect("crash: cleared at the next INIT counts", all_ok(rb.photon_crash_checks(
    crash, before_crash, routing(**{"atInit.counters": counters(clearedAtInit=4)}))))
expect("crash: an SDK stand-in left at the next INIT fails", any_bad(rb.photon_crash_checks(
    crash, before_crash, routing(**{"atInit.counters": counters(clearedAtInit=4), "atInit.standIns": {"2": 1}}))))
expect("crash: nothing left unacknowledged at the throw fails", any_bad(rb.photon_crash_checks(
    mutate(crash, ["main", "lostAtThrow"], {"controlHub": 0, "expansionHub": 2}), before_crash, after_crash)))
expect("crash: an app restart fails", any_bad(rb.photon_crash_checks(mutate(crash, ["outcome", "appRestarted"], True))))
expect("crash: violation fails", any_bad(rb.photon_crash_checks(mutate(crash, ["sampler"], sampler(violate=15)))))
expect("crash: nothing in flight fails", any_bad(rb.photon_crash_checks(mutate(crash, ["main", "inFlightAtThrow"], 0))))
expect("crash: didn't crash fails", any_bad(rb.photon_crash_checks(mutate(crash, ["outcome", "crashed"], False))))
before = routing()
after = routing(**{"atInit.loaders.benchLoader": "SlothClassLoader@2"})
expect("sloth: pinned core after a hot load passes", all_ok(rb.photon_sloth_checks(before, after, {"result": "processed"})))
expect("sloth: app restarted fails", any_bad(rb.photon_sloth_checks(before, routing(**{"atInit.loaders.pid": 7,
                                                                                     "atInit.loaders.benchLoader": "S@2"}),
                                                                    {"result": "processed"})))
expect("sloth: second PhotonCore fails", any_bad(rb.photon_sloth_checks(before, routing(**{"atInit.loaders.photonCoreLoader": "S@2",
                                                                                         "atInit.loaders.benchLoader": "S@2"}),
                                                                        {"result": "processed"})))
expect("sloth: no reload happened fails", any_bad(rb.photon_sloth_checks(before, routing(), {"result": "processed"})))
expect("sloth: load cancelled fails", any_bad(rb.photon_sloth_checks(before, after, {"result": "cancelled"})))
expect("sloth: not pinned fails", any_bad(rb.photon_sloth_checks(before, routing(**{"atInit.loaders.benchLoader": "S@2",
                                                                                   "atInit.loaders.photonCorePinned": False}),
                                                                 {"result": "processed"})))

# --- RS485
zero_in_eh = {"controlHub": 6, "expansionHub": 0}
rs = {"rs485SharedLink": {
    "off": phase(parallel=0, writeBlockPerHubMs={"controlHub": {"count": 9, "p50": 9, "p95": 19, "max": 29},
                                                 "expansionHub": {"count": 9, "p50": 12, "p95": 20, "max": 30}}),
    "on8": phase(parallel=8, issued=300, issuedWrites={"controlHub": 300, "expansionHub": 300},
                 counters=counters(parallelWrites=300), maxInFlight=zero_in_eh, maxPendingEntries=zero_in_eh,
                 maxStandIns=zero_in_eh,
                 writeBlockPerHubMs={"controlHub": {"count": 9, "p50": 2, "p95": 4, "max": 9},
                                     "expansionHub": {"count": 9, "p50": 12, "p95": 20, "max": 30}})}}
logs = {"lines": 40, "phases": {"init": {n: 0 for n in rb.PHOTON_LOG_NEEDLES},
                                "rs485 shared link off": {n: 0 for n in rb.PHOTON_LOG_NEEDLES},
                                "rs485 shared link on8": {n: 0 for n in rb.PHOTON_LOG_NEEDLES}}}
expect("rs485: child on the SDK path beside parallel Control Hub writes passes", all_ok(rb.photon_rs485_checks(rs, logs)))
expect("rs485: a parallel write to the child fails", any_bad(rb.photon_rs485_checks(
    mutate(rs, ["rs485SharedLink", "on8", "maxInFlight", "expansionHub"], 1), logs)))
expect("rs485: a stand-in on the child fails", any_bad(rb.photon_rs485_checks(
    mutate(rs, ["rs485SharedLink", "on8", "maxStandIns", "expansionHub"], 1), logs)))
expect("rs485: no Control Hub parallel writes beside it fails", any_bad(rb.photon_rs485_checks(
    mutate(rs, ["rs485SharedLink", "on8", "counters", "parallelWrites"], 0), logs)))
expect("rs485: a wrong read-back fails", any_bad(rb.photon_rs485_checks(
    mutate(rs, ["rs485SharedLink", "off", "readback", "mismatches"], 1), logs)))
expect("rs485: lock warning fails", any_bad(rb.photon_rs485_checks(
    rs, mutate(logs, ["phases", "rs485 shared link on8", "incorrect owner releasing"], 1))))
expect("rs485: a missing phase marker fails", any_bad(rb.photon_rs485_checks(
    rs, mutate(logs, ["phases", "rs485 shared link off"], DELETE))))

# --- cable pulls
def replug_res(**over):
    r = {"faultSeen": True, "maxWriteStallMs": 112.0, "maxSingleWriteMs": 104.0, "recoveryMsAfterBack": 320.0,
         "inFlightAtEnd": 0, "drainMs": 1.0, "leftoversAtEnd": clean_leftovers(),
         "controlHubReadback": {"checks": 100, "mismatches": 0, "events": []}, "routingProblemsAfter": [],
         "countersDuringFault": counters(writeFailures=1, deviceNotReadyFallbacks=40), "parallelWriteMsAfterBack": 12.0}
    r.update(over)
    return {"result": r}


off_pull, on_pull = replug_res(countersDuringFault=counters()), replug_res(maxWriteStallMs=118.0)
expect("replug: on no worse than off passes", all_ok(rb.photon_replug_checks(off_pull, on_pull, "usb")))
expect("replug: the SDK's own ~100 ms retransmit stall isn't blamed on Photon",
       all_ok(rb.photon_replug_checks(replug_res(maxWriteStallMs=140.0, countersDuringFault=counters()),
                                      replug_res(maxWriteStallMs=150.0), "usb")))
for label, off_r, on_r in (("Photon stalls longer than the SDK", off_pull, replug_res(maxWriteStallMs=200.0)),
                           ("one write call over 300 ms", off_pull, replug_res(maxSingleWriteMs=400.0)),
                           ("nothing fell back while the hub was gone", off_pull, replug_res(countersDuringFault=counters())),
                           ("slow recovery with Photon", off_pull, replug_res(recoveryMsAfterBack=1500.0)),
                           ("slow recovery without Photon", replug_res(recoveryMsAfterBack=1500.0), on_pull),
                           ("fault never seen", off_pull, replug_res(faultSeen=False)),
                           ("left in flight", off_pull, replug_res(inFlightAtEnd=2)),
                           ("SDK stand-in left", off_pull, replug_res(leftoversAtEnd=mutate(clean_leftovers(), ["expansionHub", "standIns"], 1))),
                           ("routing broken after", off_pull, replug_res(routingProblemsAfter=["x"])),
                           ("no off run", {}, on_pull)):
    expect(f"replug: {label} fails", any_bad(rb.photon_replug_checks(off_r, on_r, "usb")))

# --- logcat
log = "\n".join([
    "10-28 23:47:46.123  1  2 I BENCH   : phase timing on8",
    "10-28 23:47:46.200  1  2 V LynxModule: retransmitting: mod=2 cmd=0x100f msg#=3 ref#=0",
    "10-28 23:47:47.123  1  2 I BENCH   : phase lost write fl 0",
    "10-28 23:47:47.300  1  2 E LynxModule: timeout: abandoning waiting 250ms for ack: cmd=X mod=2 msg#=4",
    "10-28 23:47:48.000  1  2 I BENCH_RUNNER: phase not a bench phase",
    "10-28 23:47:48.123  1  2 I BENCH   : phase reads on8",
])
parsed = rb.photon_log_counts(log)
counts = parsed["phases"]
expect("logcat: counted per phase", counts.get("timing on8", {}).get("retransmitting") == 1
       and counts.get("lost write fl 0", {}).get("timeout: abandoning waiting") == 1)
expect("logcat: a phase with no warnings is still listed", "reads on8" in counts)
expect("logcat: runner marks aren't phases", "not a bench phase" not in counts)
clean_log = rb.photon_log_counts("\n".join(log.splitlines()[:2] + log.splitlines()[5:]))
expect("logcat: retransmits alone pass", all_ok(rb.photon_log_checks("x", [("s", clean_log, ["timing on8", "reads on8"])])))
expect("logcat: abandon in a workload phase fails", any_bad(rb.photon_log_checks("x", [("s", parsed, [])])))
expect("logcat: abandon in an exempt phase passes", all_ok(rb.photon_log_checks("x", [("s", parsed, [])], exempt=("lost write",))))
expect("logcat: an empty log fails", any_bad(rb.photon_log_checks("x", [("s", rb.photon_log_counts(""), [])])))
expect("logcat: a step with no log fails", any_bad(rb.photon_log_checks("x", [("s", None, [])])))
expect("logcat: a phase the result names but the log lacks fails", any_bad(rb.photon_log_checks(
    "x", [("s", clean_log, ["timing on8", "soak"])])))
expect("logcat: one bad step among many STOP trials fails", any_bad(rb.photon_log_checks(
    "x", [("a", clean_log, []), ("b", parsed, []), ("c", clean_log, [])])))
expect("logcat: no steps fails", any_bad(rb.photon_log_checks("x", [])))
slow_version = rb.photon_log_counts("\n".join([
    "12-31 16:08:57.310  1830  2069 E LynxModule: timeout: abandoning waiting 250ms for response: cmd=LynxReadVersionStringCommand mod=173 msg#=185",
    "12-31 16:08:57.318  1830  2061 E LynxModule: unable to find originating command for packetid=0x9030 msg#=212 ref#=185",
    "12-31 16:08:58.000  1  2 I BENCH   : phase writing"]))
expect("logcat: the hub's slow version-string reply after a restart (seen with no Photon) is reported, not judged",
       all_ok(rb.photon_log_checks("x", [("s", slow_version, ["writing"])]))
       and "slowVersionStringReplies" in rb.photon_log_checks("x", [("s", slow_version, ["writing"])])[0].detail)
expect("logcat: any other command's timeout still fails", any_bad(rb.photon_log_checks("x", [("s", rb.photon_log_counts(
    "x E LynxModule: timeout: abandoning waiting 250ms for response: cmd=LynxGetADCCommand mod=2 msg#=194\n"
    "x E LynxModule: unable to find originating command for packetid=0x9007 msg#=194 ref#=194"), [])])))


# --- I2C reads through the replaced hub object: their replies go there on purpose
off_replaced = i2c_phase(parallel=0, replaced=True)
expect("replaced reader: an off phase whose second readers go through the replaced hub object passes with its replies",
       rb.photon_phase_problems(off_replaced) == [])
expect("replaced reader: too few replies for its reads fails",
       rb.photon_phase_problems(mutate(off_replaced, ["counters", "orphanDatagrams"], 2 * 1600 - 1)))
expect("replaced reader: no replies at all fails", rb.photon_phase_problems(mutate(off_replaced, ["counters", "orphanDatagrams"], 0)))
expect("replaced reader: replies to the replaced hub object in an ordinary I2C phase still fail",
       rb.photon_phase_problems(mutate(i2c_phase(), ["counters", "orphanDatagrams"], 3)))
expect("replaced reader: a phase claiming the replaced reader with no replaced reads fails",
       rb.photon_phase_problems(mutate(off_replaced, ["i2cReaders"], [r for r in off_replaced["i2cReaders"]
                                                                      if r["client"] != "replaced"])))
expect("replaced reader: a number clash still fails there", rb.photon_phase_problems(
    mutate(off_replaced, ["counters", "numberClashes"], 1)))

# --- I2C checks
def i2c_stock(topo="usb"):
    return {"i2cInfo": i2c_info(topo, swapped=False), "i2c": {"stock": i2c_phase(parallel=-1, topo=topo, swapped=False)}}


def i2c_main(topo="usb", soak_reads=6000):
    return {"i2cInfo": i2c_info(topo), "i2c": {"off": i2c_phase(0, topo, replaced=True), "on2": i2c_phase(2, topo),
                                               "on8": i2c_phase(8, topo)},
            "i2cControl": i2c_control(topo), "i2cAfterControl": i2c_phase(0, topo, reads=120),
            "soak": i2c_phase(8, topo, reads=soak_reads)}


expect("i2c: the hub plan keeps the OctoQuad on Control Hub bus 2 and puts the Pinpoint on the Expansion Hub when there is one",
       rb.i2c_hub_plan("usb")["pinpoint"] == ("expansionHub", 1) and rb.i2c_hub_plan("rs485")["pinpoint"] == ("expansionHub", 1)
       and rb.i2c_hub_plan("none")["pinpoint"] == ("controlHub", 1)
       and all(rb.i2c_hub_plan(t)["octoquad"] == ("controlHub", 2) for t in ("usb", "rs485", "none")))
for topo in ("usb", "rs485", "none"):
    expect(f"i2c: an all-good {topo} block passes every I2C check",
           all_ok(rb.photon_i2c_checks(i2c_stock(topo), i2c_main(topo), topo)))
good_i2c = rb.photon_i2c_checks(i2c_stock(), i2c_main(), "usb")
expect("i2c: one check per INIT, per phase (stock, off, on2, on8, after control, soak) and the negative control",
       len(good_i2c) == 9)
expect("i2c: a soak without its readers fails", any_bad(rb.photon_i2c_checks(i2c_stock(), mutate(i2c_main(), ["soak"], phase()), "usb")))
expect("i2c: a soak with a wrong byte fails",
       any_bad(rb.photon_i2c_checks(i2c_stock(), mutate(i2c_main(), ["soak", "i2cReaders", 3, "wrong"], 1), "usb")))
expect("i2c: a negative control that caught nothing says, per hub, the lock check couldn't be shown to fail",
       good_i2c[-1].ok and "Pinpoint on expansionHub: 0 of 200 caught, so the lock check couldn't be shown to fail on this hub"
       in good_i2c[-1].detail and "OctoQuad on controlHub: 0 of 200 caught" in good_i2c[-1].detail)
caught_i2c = rb.photon_i2c_checks(i2c_stock(), mutate(i2c_main(), ["i2cControl"], i2c_control(wrong=3, refused={"status X": 2})), "usb")
expect("i2c: a negative control that caught overlaps says how many, per hub",
       caught_i2c[-1].ok and caught_i2c[-1].detail.count("the byte compares saw 5 anomalies of 200") == 2)
one_hub = mutate(i2c_main(), ["i2cControl", "pinpoint", "unlockedWrong"], 6)
one_hub = mutate(one_hub, ["i2cControl", "pinpoint", "unlockedRight"], 194)
one_hub_check = rb.photon_i2c_checks(i2c_stock(), one_hub, "usb")[-1]
expect("i2c: one hub's caught anomalies don't speak for the other hub",
       one_hub_check.ok and "Pinpoint on expansionHub: the byte compares saw 6 anomalies of 200" in one_hub_check.detail
       and "OctoQuad on controlHub: 0 of 200 caught, so the lock check couldn't be shown to fail on this hub" in one_hub_check.detail)
expect("i2c: the hub whose control caught nothing is named for the verdict",
       rb.i2c_control_blind(one_hub) == ["OctoQuad on controlHub: 0 of 200 caught, so the lock check couldn't be shown to fail on this hub"]
       and rb.i2c_control_blind(mutate(one_hub, ["i2cControl", "octoquad", "lockedReader", "wrong"], 1)) == [])
expect("i2c summary: the negative control's verdict per hub",
       "- negative control, OctoQuad on controlHub: 0 of 200 caught" in "\n".join(rb.photon_i2c_summary(i2c_stock(), one_hub)))
for label, stock_m, main_m in (
        ("no stock I2C info", mutate(i2c_stock(), ["i2cInfo"], DELETE), i2c_main()),
        ("no Pinpoint at INIT", mutate(i2c_stock(), ["i2cInfo", "devices"], i2c_info()["devices"][1:]), i2c_main()),
        ("an all-zero expected register", mutate(i2c_stock(), ["i2cInfo", "devices", 0, "expectedA"], "00000000"), i2c_main()),
        ("stock clients already Photon's", mutate(i2c_stock(), ["i2cInfo", "devices", 0, "clientModuleSwapped"], True), i2c_main()),
        ("the Pinpoint planned on the wrong hub", i2c_stock(), mutate(i2c_main(), ["i2cInfo", "devices", 0, "hub"], "controlHub")),
        ("the OctoQuad planned on the wrong hub", i2c_stock(), mutate(i2c_main(), ["i2cInfo", "devices", 1, "hub"], "expansionHub")),
        ("the OctoQuad's driver client on the wrong hub", i2c_stock(), mutate(i2c_main(), ["i2cInfo", "devices", 1, "clientHub"], "expansionHub")),
        ("the Pinpoint's second client on another bus", i2c_stock(), mutate(i2c_main(), ["i2cInfo", "devices", 0, "secondBus"], 2)),
        ("the Pinpoint's second client on another hub", i2c_stock(), mutate(i2c_main(), ["i2cInfo", "devices", 0, "secondHub"], "REV Hub #5")),
        ("the OctoQuad's second client at another address", mutate(i2c_stock(), ["i2cInfo", "devices", 1, "secondI2cAddress"], 0x31), i2c_main()),
        ("a Photon client left on the SDK's hub", i2c_stock(), mutate(i2c_main(), ["i2cInfo", "devices", 1, "secondModuleSwapped"], False)),
        ("a swap failure", i2c_stock(), mutate(i2c_main(), ["i2cInfo", "photonSwapFailures"], ["final field"])),
        ("a constant register reading differently after Photon's INIT", i2c_stock(),
         mutate(i2c_main(), ["i2cInfo", "devices", 0, "expectedB"], "04000000")),
        ("no stock I2C phase", mutate(i2c_stock(), ["i2c"], {}), i2c_main()),
        ("a wrong byte in on8", i2c_stock(), mutate(i2c_main(), ["i2c", "on8", "i2cReaders", 1, "wrong"], 1)),
        ("an all-zero read in on2", i2c_stock(), mutate(i2c_main(), ["i2c", "on2", "i2cReaders", 2, "placeholders"], 1)),
        ("a reader that failed", i2c_stock(), mutate(i2c_main(), ["i2c", "on8", "i2cReaders", 0, "failure"], "ISE")),
        ("a reader still alive at halt", i2c_stock(), mutate(i2c_main(), ["i2c", "on8", "i2cReaders", 0, "haltProblems"], ["alive"])),
        ("a reader short of reads", i2c_stock(), mutate(i2c_main(), ["i2c", "on8", "i2cReaders", 3, "reads"], 10)),
        ("a reader on the SDK's hub object in on8", i2c_stock(), mutate(i2c_main(), ["i2c", "on8", "i2cReaders", 0, "moduleSwapped"], False)),
        ("a stock reader on Photon's hub object", mutate(i2c_stock(), ["i2c", "stock", "i2cReaders", 0, "moduleSwapped"], True), i2c_main()),
        ("one hub's readers missing", i2c_stock(), mutate(i2c_main(), ["i2c", "on8", "i2cReaders"], i2c_main()["i2c"]["on8"]["i2cReaders"][:2])),
        ("only one reader on a device", i2c_stock(), mutate(i2c_main(), ["i2c", "on2", "i2cReaders"], i2c_main()["i2c"]["on2"]["i2cReaders"][:3])),
        ("a reader on the wrong bus", i2c_stock(), mutate(i2c_main(), ["i2c", "on2", "i2cReaders", 2, "bus"], 1)),
        ("a replaced reader on the other hub", i2c_stock(), mutate(i2c_main(), ["i2c", "off", "i2cReaders", 3, "hub"], "expansionHub")),
        ("no on2 phase", i2c_stock(), mutate(i2c_main(), ["i2c", "on2"], DELETE)),
        ("off without the replaced reader", i2c_stock(), mutate(i2c_main(), ["i2c", "off"], i2c_phase(0))),
        ("a command left pending on the replaced hub object", i2c_stock(),
         mutate(i2c_main(), ["i2c", "off", "replacedPendingAfter", "octoquad"], 1)),
        ("no phase after the negative control", i2c_stock(), mutate(i2c_main(), ["i2cAfterControl"], DELETE)),
        ("a wrong byte after the negative control", i2c_stock(), mutate(i2c_main(), ["i2cAfterControl", "i2cReaders", 0, "wrong"], 2)),
        ("no negative control", i2c_stock(), mutate(i2c_main(), ["i2cControl"], DELETE)),
        ("a negative control on one device only", i2c_stock(), mutate(i2c_main(), ["i2cControl", "octoquad"], DELETE)),
        ("a negative control cut short", i2c_stock(), mutate(i2c_main(), ["i2cControl"], i2c_control(attempts=150)))):
    expect(f"i2c: {label} fails", any_bad(rb.photon_i2c_checks(stock_m, main_m, "usb")))
expect("i2c: a missing main result fails rather than passing", any_bad(rb.photon_i2c_checks(i2c_stock(), {}, "usb")))
expect("i2c: no results at all fail every check", not any(c.ok for c in rb.photon_i2c_checks({}, {}, "usb")))
summary = "\n".join(rb.photon_i2c_summary(i2c_stock(), i2c_main()))
expect("i2c summary: a row per reader per phase and the rule-of-three bound",
       summary.count("| pinpoint |") == 12 and "rule of three" in summary and "upper bound" in summary)
expect("i2c summary: the bound counts every clean read (stock, off, on2, on8, after control and soak)",
       "37280 clean reads" in summary and "8e-05" in summary)
dirty = "\n".join(rb.photon_i2c_summary(i2c_stock(), mutate(i2c_main(), ["i2c", "on8", "i2cReaders", 0, "wrong"], 1)))
expect("i2c summary: a reader with a wrong byte doesn't count toward the bound", "36480 clean reads" in dirty)
expect("i2c summary: read time p95 against Photon off", "pinpoint driver on8 1.00x" in summary)
profiled = {"timing": {"on8 profiled": {"writeProfile": {"controlHub": {"sweep": {"count": 3.0, "meanUs": 2.5, "p50Us": 2.5, "p95Us": 7.5}}},
                                        "prepareMs": {"count": 1, "p50": 0.1, "p95": 0.2, "max": 0.3},
                                        "noteWritesMs": {"count": 1, "p50": 0.1, "p95": 0.2, "max": 0.3}}}}
expect("profile summary: a row per step", "| sweep | 2.5 / 7.5 / 2.5 (3) |" in "\n".join(rb.photon_profile_summary(profiled)))
expect("profile summary: nothing without a profile", rb.photon_profile_summary({}) == [])

# --- logcat: I2C warnings
i2c_log = rb.photon_log_counts("\n".join([
    "10-29 17:00:00.000  1  2 I BENCH   : phase i2c on8",
    "10-29 17:00:00.100  1  2 E LynxI2cDeviceSynch: placeholder: readStatusQuery",
    "10-29 17:00:01.000  1  2 I BENCH   : phase i2c control",
    "10-29 17:00:01.100  1  2 E RobotCore: exception thrown during lynx communication",
    "10-29 17:00:01.200  1  2 E LynxI2cDeviceSynch: readStatusQuery: cbExpected=4 cbRead=3"]))
expect("logcat: an I2C placeholder in a judged phase fails", any_bad(rb.photon_log_checks("x", [("s", i2c_log, [])],
                                                                                         exempt=("i2c control",))))
expect("logcat: I2C warnings in the negative control's own phase pass",
       all_ok(rb.photon_log_checks("x", [("s", rb.photon_log_counts("\n".join(
           ["10-29 17:00:01.000  1  2 I BENCH   : phase i2c control",
            "10-29 17:00:01.100  1  2 E RobotCore: exception thrown during lynx communication",
            "10-29 17:00:01.200  1  2 E LynxI2cDeviceSynch: readStatusQuery: cbExpected=4 cbRead=3"])), [])],
           exempt=("i2c control",))))
expect("logcat: the negative control's exemption doesn't cover the phase after it",
       any_bad(rb.photon_log_checks("x", [("s", rb.photon_log_counts("\n".join(
           ["10-29 17:00:01.000  1  2 I BENCH   : phase i2c after control",
            "10-29 17:00:01.100  1  2 E LynxI2cDeviceSynch: placeholder: x"])), [])], exempt=("i2c control",))))

# --- linear STOP: LinearOpMode's interrupt and the SDK's own network-lock leak
STOP = "12-31 17:06:09.328  3050  3286 I FtcEventLoop: auto-stopping OpMode 'Bench: Photon Stop Linear'"


def abandon_at(ms, old="LynxSetServoEnableCommand", msg=0):
    sec, rest = divmod(9328 + ms, 1000)
    return (f"12-31 17:06:{sec:02d}.{rest:03d}  3050  9023 E RobotCore: lynx xmit lock: #### abandoning lock: "
            f"old={old}({msg})")


def lock_events(*lines):
    return rb.stop_lock_events("\n".join(lines))


clean_stop = lock_events(STOP)
leak_stop = lock_events(STOP, abandon_at(508))


def lock_trials(n, leaks=0, kind=None):
    return [(f"t{i}", kind if kind is not None and i < leaks else leak_stop if i < leaks else clean_stop) for i in range(n)]


logs = {q.stem: q.read_text(errors="replace") for q in sorted((RESULTS / "logs").glob("photon_stop_*.log"))}
linear_on = {k: v for k, v in logs.items() if k.startswith("photon_stop_linear_on_")}
expect("real: 40 linear on8 trial logs", len(linear_on) == 40)
expect("real: every STOP trial log has its STOP mark", logs and all(rb.stop_lock_events(t)["stopMark"] for t in logs.values()))
real = {k: rb.stop_lock_events(v) for k, v in linear_on.items()}
with_events = {k: e for k, e in real.items() if e["events"]}
e018 = (real.get("photon_stop_linear_on_018") or {}).get("events") or [{}]
expect("real: the one abandoned lock is trial 018's, 508 ms after STOP, a STOP-interrupt leak of the gated servo enable",
       list(with_events) == ["photon_stop_linear_on_018"] and e018[0].get("msAfterStop") == 508 and e018[0].get("stopLeak")
       and e018[0].get("old") == "LynxSetServoEnableCommand" and e018[0].get("oldMsg") == 0)
expect("real: no abandoned lock in any iterative or negative-control trial",
       not any(rb.stop_lock_events(v)["events"] for k, v in logs.items() if not k.startswith("photon_stop_linear_")))
on_real = sorted(real.items())
expect("real: with a clean 40-trial Photon-off control, Photon on passes both lock checks (1 vs 0, p 0.5)",
       all_ok(rb.photon_linear_lock_checks(on_real, lock_trials(40)))
       and "p 0.500" in rb.photon_linear_lock_checks(on_real, lock_trials(40))[1].detail)
expect("real: without the Photon-off control the comparison fails", not rb.photon_linear_lock_checks(on_real, [])[1].ok)
parsed018 = rb.photon_log_counts(linear_on["photon_stop_linear_on_018"])
expect("real: the plain log check flags trial 018", rb.photon_log_problems(parsed018, ["writing"])[0])
expect("real: with abandons judged by the comparison instead, trial 018's log has no other warning",
       not rb.photon_log_problems(parsed018, ["writing"], ignore=("abandoning lock",))[0])
samplers = {q.stem: json.loads(q.read_text()) for q in (RESULTS / "raw").glob("photon_stop_linear_on_*.json")}
expect("real: 40 linear sampler files", len(samplers) == 40)
expect("real: every linear sampler file from the old bench is caught (no interrupt record, no counters at STOP)",
       samplers and all("the OpMode thread never saw LinearOpMode's STOP interrupt" in rb.linear_trial_problems(x)
                        and any("counters at STOP" in q for q in rb.linear_trial_problems(x)) for x in samplers.values()))
expect("real: the old bench's linear trials fail the STOP verdict on the new problems too",
       all("the OpMode thread never saw LinearOpMode's STOP interrupt"
           in rb.stop_trial_verdict(x, require_zeroing_read=True)["problems"] for x in samplers.values()))
expect("real: trial 018's 521 ms Control Hub sampler gap fits a STOP-interrupt leak, so isn't a blind-sampler problem",
       not any("no sweep for" in q for q in rb.linear_trial_problems(samplers["photon_stop_linear_on_018"])))
gap018 = rb.max_gap_after_gate(samplers["photon_stop_linear_on_018"]["hubs"][0])
expect("real: trial 018's Control Hub gap is the leak's ~520 ms", gap018 is not None and 500 < gap018 < 540)
expect("real: no other linear trial has a post-STOP sampler gap over 100 ms",
       all((rb.max_gap_after_gate(h) or 0) < 100 for k, x in samplers.items() if not k.endswith("_018") for h in x["hubs"]))
iter_samplers = [json.loads(q.read_text()) for q in (RESULTS / "raw").glob("photon_stop_iter_*.json")]
expect("real: iterative trials gain no new problem (they record counters and need no interrupt)",
       iter_samplers and all(rb.linear_trial_problems(x) == [] for x in iter_samplers))

expect("stop events: a STOP mark is found", clean_stop["stopMark"] and not clean_stop["events"])
expect("stop events: 508 ms after STOP is a STOP-interrupt leak", leak_stop["events"][0]["stopLeak"])
expect("stop events: 495 ms after STOP is too early to be one (the lock times out at 500 ms)",
       not lock_events(STOP, abandon_at(495))["events"][0]["stopLeak"])
expect("stop events: the latest a STOP leak can be abandoned is one",
       lock_events(STOP, abandon_at(rb.PHOTON_STOP_LEAK_MS[1]))["events"][0]["stopLeak"])
expect("stop events: 1 ms later isn't", not lock_events(STOP, abandon_at(rb.PHOTON_STOP_LEAK_MS[1] + 1))["events"][0]["stopLeak"])
expect("stop events: an abandon before STOP isn't", not lock_events(abandon_at(-300), STOP)["events"][0]["stopLeak"])
expect("stop events: an abandon with no STOP mark can't be timed", lock_events(abandon_at(508))["events"][0]["msAfterStop"] is None)
expect("lock checks: no abandons at all pass", all_ok(rb.photon_linear_lock_checks(lock_trials(40), lock_trials(40))))
expect("lock checks: one STOP leak each side passes", all_ok(rb.photon_linear_lock_checks(lock_trials(40, 1), lock_trials(40, 1))))
for label, bad in (("an abandon 2 s after STOP", lock_events(STOP, abandon_at(2000))),
                   ("an abandon mid-run", lock_events(abandon_at(-1500), STOP)),
                   ("an abandon in a log with no STOP mark", lock_events(abandon_at(508))),
                   ("a log with no STOP mark", lock_events())):
    expect(f"lock checks: {label} in a Photon-on trial fails",
           not rb.photon_linear_lock_checks(lock_trials(39) + [("x", bad)], lock_trials(40))[0].ok)
    expect(f"lock checks: {label} in a Photon-off trial fails too",
           not rb.photon_linear_lock_checks(lock_trials(40), lock_trials(39) + [("x", bad)])[0].ok)
expect("lock checks: a step with no parsed log fails", not rb.photon_linear_lock_checks(lock_trials(39) + [("x", None)], lock_trials(40))[0].ok)
expect("fisher: 1 vs 0 of 40 is p 0.5", abs(rb.fisher_greater(1, 40, 0, 40) - 0.5) < 1e-12)
expect("fisher: equal counts are p >= 0.5", rb.fisher_greater(3, 40, 3, 40) >= 0.5)
expect("fisher: no events at all is p 1", rb.fisher_greater(0, 40, 0, 40) == 1.0)
for a, b, n, ok in ((1, 0, 40, True), (3, 0, 40, True), (4, 0, 40, True), (5, 0, 40, False), (7, 0, 40, False),
                    (7, 5, 40, True), (0, 5, 40, True), (5, 15, 200, True), (15, 5, 200, False)):
    pv = rb.fisher_greater(a, n, b, n)
    expect(f"lock checks: Photon on {a} vs off {b} of {n} {'passes' if ok else 'fails'} (p {pv:.3f})",
           rb.photon_linear_lock_checks(lock_trials(n, a), lock_trials(n, b))[1].ok == ok)
expect("lock checks: a leak in the control alone passes (Photon is no worse)",
       all_ok(rb.photon_linear_lock_checks(lock_trials(40), lock_trials(40, 2))))
expect("lock checks: no Photon-on trials fails", not all_ok(rb.photon_linear_lock_checks([], lock_trials(40))))
expect("lock checks: no control trials fails", not all_ok(rb.photon_linear_lock_checks(lock_trials(40), [])))
expect("lock checks: the detail names both counts and p",
       "1 of 40 Photon on8" in rb.photon_linear_lock_checks(lock_trials(40, 1), lock_trials(40))[1].detail)
fs_leak = lock_events(STOP, abandon_at(530, "LynxFailSafeCommand", 41))
fs_checks = rb.photon_linear_lock_checks(lock_trials(39) + [("f", fs_leak)], lock_trials(40))
expect("lock checks: a leak by a fail-safe on the OpMode thread (only Photon's post-STOP one) fails outright, p aside",
       not fs_checks[1].ok and "LynxFailSafeCommand" in fs_checks[1].detail and "only Photon's post-STOP fail-safe" in fs_checks[1].detail
       and fs_checks[0].ok)
expect("lock checks: so does one in the linear negative control",
       not rb.photon_linear_lock_checks(lock_trials(40), lock_trials(40), [("c", fs_leak)])[1].ok)
expect("lock checks: a linear-once control abandon 508 ms after STOP passes and stays out of the comparison",
       all_ok(rb.photon_linear_lock_checks(lock_trials(40), lock_trials(40), lock_trials(5, 1)))
       and "0 of 40 Photon on8 trials and 0 of 40 Photon off" in rb.photon_linear_lock_checks(lock_trials(40), lock_trials(40),
                                                                                         lock_trials(5, 5))[1].detail)
expect("lock checks: a linear-once control abandon 900 ms after STOP fails",
       not rb.photon_linear_lock_checks(lock_trials(40), lock_trials(40),
                                        lock_trials(4) + [("c", lock_events(STOP, abandon_at(900)))])[0].ok)
expect("lock checks: a linear-once control log with no STOP mark fails",
       not rb.photon_linear_lock_checks(lock_trials(40), lock_trials(40), lock_trials(4) + [("c", lock_events())])[0].ok)
expect("fisher power: matches an independent count at 40 vs 40 and a 2.5% base (4x 0.18, 6x 0.46, 8x 0.73)",
       abs(rb.fisher_power(40, 40, 1 / 40, 4) - 0.180) < 0.005 and abs(rb.fisher_power(40, 40, 1 / 40, 6) - 0.464) < 0.005
       and abs(rb.fisher_power(40, 40, 1 / 40, 8) - 0.725) < 0.005)
expect("fisher power: no rise is caught at most alpha of the time", rb.fisher_power(200, 200, 1 / 40, 1) <= rb.PHOTON_LEAK_ALPHA)
expect("fisher power: the smallest rise caught half the time is 8x at 40 vs 40 and 3x at 200 vs 200",
       rb.smallest_detectable_rise(40, 40, 1 / 40) == 8 and rb.smallest_detectable_rise(200, 200, 1 / 40) == 3)
expect("lock checks: a short block's pass says how small a rise it could have caught",
       "catches a 8x rise in at least half of all blocks" in rb.photon_linear_lock_checks(lock_trials(40), lock_trials(40))[1].detail
       and "catches a 3x rise" in rb.photon_linear_lock_checks(lock_trials(200), lock_trials(200))[1].detail)
expect("lock checks: the power uses the control's own leak rate when it has one",
       "a 5.0% base rate" in rb.photon_linear_lock_checks(lock_trials(40), lock_trials(40, 2))[1].detail)
expect("lock checks: with too few trials it says no rise is caught",
       "can't catch even a 40x rise" in rb.photon_linear_lock_checks(lock_trials(1), lock_trials(1))[1].detail)
other = rb.photon_log_counts("\n".join([STOP, "12-31 17:06:09.900  3050  9023 E RobotCore: lynx xmit lock: #### "
                                        "incorrect owner releasing message keyed lock: ignored: old=X(2:3)"]))
expect("lock log: an ownership warning in a linear trial still fails with abandons judged elsewhere",
       rb.photon_log_problems(other, [], ignore=("abandoning lock",))[0])
expect("lock log: an abandon is still an absolute failure where nothing ignores it",
       rb.photon_log_problems(rb.photon_log_counts("\n".join([STOP, abandon_at(508)])), [])[0])
expect("lock log: ignore never hides it from photon_log_checks without the argument",
       any_bad(rb.photon_log_checks("x", [("s", rb.photon_log_counts("\n".join([STOP, abandon_at(508)])), [])])))
lin_good = {"info": {"variant": "linear"}, "extra": {"interruptedAtStop": True, "countersAtStop": {}},
            "hubs": [{"key": "controlHub", "gateMs": 100.0, "sweeps": [[90.0], [101.0], [111.0], [620.0], [630.0]]}]}
expect("linear verdict: a clean linear trial has no new problems", rb.linear_trial_problems(lin_good) == [])
for label, path, value in (("the interrupt never arrived", ["extra", "interruptedAtStop"], False),
                           ("no interrupt record", ["extra", "interruptedAtStop"], DELETE),
                           ("no counters at STOP", ["extra", "countersAtStop"], DELETE)):
    expect(f"linear verdict: {label} is a problem", rb.linear_trial_problems(mutate(lin_good, path, value)))
expect("linear verdict: a sampler blind for 799 ms after STOP is a problem",
       any("no sweep for 799 ms" in q for q in rb.linear_trial_problems(
           mutate(lin_good, ["hubs", 0, "sweeps"], [[90.0], [101.0], [900.0], [910.0]]))))
expect("linear verdict: an iterative trial needs no interrupt record",
       rb.linear_trial_problems({"info": {"variant": "iterative"}, "extra": {"countersAtStop": {}}, "hubs": []}) == [])
expect("linear verdict: the STOP checks fail a linear trial that never saw its interrupt",
       any_bad(rb.photon_stop_checks("linear", [trial("l", mutate(dict(sampler(), info={"variant": "linear"}),
                                                                   ["extra", "interruptedAtStop"], False))])))
expect("linear verdict: the STOP checks pass a linear trial that saw it",
       all_ok(rb.photon_stop_checks("linear", [trial("l", mutate(dict(sampler(), info={"variant": "linear"}),
                                                                  ["extra", "interruptedAtStop"], True))])))

# --- verdict
ok_checks = [rb.PhotonCheck("stop", "x", True, "", "")]
level, blockers, needed = rb.photon_verdict(ok_checks, True, True, True, True, 200)
expect("verdict: all green with the soak and 200 STOP trials is SAFE", level == "SAFE" and not blockers and not needed)
level, blockers, needed = rb.photon_verdict(ok_checks, True, True, False, False, 40)
expect("verdict: a short run is only PASSED and says what's still needed",
       level == "PASSED" and any("soak" in n for n in needed) and any("7.5%" in n for n in needed))
level, _, needed = rb.photon_verdict(ok_checks, True, True, True, True, 200, still_needed=["the cable pulls (--interactive)"])
expect("verdict: without the cable pulls it is only PASSED", level == "PASSED" and needed == ["the cable pulls (--interactive)"])
level, _, needed = rb.photon_verdict(ok_checks, True, True, True, True, 40)
expect("verdict: a soak with 40 STOP trials is only PASSED", level == "PASSED" and len(needed) == 1)
expect("verdict: a failed check blocks", rb.photon_verdict(ok_checks + [rb.PhotonCheck("lost", "y", False, "", "")],
                                                           True, True, True, True, 200)[0] == "NOT SAFE")
expect("verdict: silent negative control blocks", rb.photon_verdict(ok_checks, False, True, True, True, 200)[0] == "NOT SAFE")
expect("verdict: one module order blocks", rb.photon_verdict(ok_checks, True, False, True, True, 200)[0] == "NOT SAFE")
expect("verdict: a soak short of its bound blocks", rb.photon_verdict(ok_checks, True, True, True, False, 200)[0] == "NOT SAFE")
expect("verdict: a skipped test blocks", rb.photon_verdict(ok_checks, True, True, True, True, 200, ["deploySloth"])[0] == "NOT SAFE")

# --- port scan of configs
xml = """<?xml version='1.0' encoding='UTF-8' standalone='yes' ?>
<Robot type="FirstInspires-FTC">
    <LynxUsbDevice name="Control Hub Portal" serialNumber="(embedded)" parentModuleAddress="173">
        <LynxModule name="Control Hub" port="173"></LynxModule>
        <LynxModule name="Expansion Hub 2" port="2"><goBILDA5202SeriesMotor name="lift" port="1" /></LynxModule>
    </LynxUsbDevice>
    <LynxUsbDevice name="EH Portal" serialNumber="DQTEST" parentModuleAddress="2">
        <LynxModule name="Expansion Hub 2" port="2"><Servo name="claw" port="0" /></LynxModule>
    </LynxUsbDevice>
</Robot>"""
expect("ports scan: RS485 child at the address found", any("lift" in d for d in rb.expansion_port_devices(xml, "NOPE", 2)))
expect("ports scan: USB hub by serial found", any("claw" in d for d in rb.expansion_port_devices(xml, "DQTEST", 9)))
expect("ports scan: other address and serial ignored", rb.expansion_port_devices(xml, "NOPE", 9) == [])

print(f"{passed} passed, {len(failed)} failed")
sys.exit(1 if failed else 0)
