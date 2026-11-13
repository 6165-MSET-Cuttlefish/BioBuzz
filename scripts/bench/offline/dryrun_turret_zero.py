"""Drives Runner.run_turret_zero end to end with every hub call stubbed and the person's answers scripted: prompt order
(wiring before power, the OctoQuad branch unplugged around anything that can unpower the OctoQuad, brownout last), what
the config names, the params each run gets, and that the default run is unchanged. Run with: python3 -I <this file>"""
import copy
import importlib.util
import pathlib
import sys
import tempfile
import xml.etree.ElementTree as ElementTree

sys.dont_write_bytecode = True
HERE = pathlib.Path(__file__).resolve().parent
OUT = HERE / "out" / "dryrun"


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


rb = load("run_bench", HERE.parent / "run_bench.py")
fx = load("turret_zero_fixtures", HERE / "turret_zero_fixtures.py")

passed = 0
failed = []


def expect(label, cond):
    global passed
    if cond:
        passed += 1
    else:
        failed.append(label)
        print("FAIL", label)


class FakeAdb:
    def __init__(self, dry):
        self.dry = dry

    def push_json(self, obj, name, scratch):
        self.dry.events.append(("push", name))


class FakeDash:
    fatal = None

    def __init__(self):
        self.data, self.status = {}, {}

    def get_status(self):
        return {}


class Dry(rb.Runner):
    def __init__(self, argv, replies, fixture, crash=None, lose_hub=None, lose_as=None, interrupt_at=None):
        self.args = rb.parse_args(argv)
        OUT.mkdir(parents=True, exist_ok=True)
        self.out = pathlib.Path(tempfile.mkdtemp(prefix="turret-zero-dry-", dir=OUT))
        (self.out / "raw").mkdir()
        (self.out / "logs").mkdir()
        self.logfile = open(self.out / "run.log", "w")
        self.summary, self.checks, self.results = [], [], {}
        self.imu_type = "BHI260"
        self.expansion = self.webcam = self.limelight = None
        self.adb, self.dash = FakeAdb(self), FakeDash()
        self.replies, self.fixture, self.crash, self.lose_hub = list(replies), fixture, crash, lose_hub
        self.lose_as = lose_as or rb.BenchError("adb lost the hub")
        self.interrupt_at = interrupt_at
        self.config, self.configs, self.events = rb.CONFIG_MAIN, {}, []

    def log(self, text):
        self.logfile.write(text + "\n")

    def ask(self, text):
        self.events.append(("ask", text))
        if self.interrupt_at and self.interrupt_at in text:
            self.interrupt_at = None
            raise KeyboardInterrupt

    def answer(self, text):
        reply = self.replies.pop(0)
        self.events.append(("answer", text, reply))
        return reply

    def active_config(self):
        return self.config

    def activate_config(self, name, xml_text=None, config=None, accept=None, battery=True):
        self.config = name
        if xml_text is not None:
            self.configs[name] = xml_text
        self.events.append(("config", name))

    def settle(self):
        self.events.append(("settle",))

    def wait_hub_back(self, what, booted_after=None):
        self.events.append(("wait hub", what, booted_after))

    def run_opmode(self, opmode, step, params=None, start=True, during=None, **kw):
        if step == self.lose_hub:
            raise self.lose_as
        self.events.append(("run", step, dict(params or {}), start))
        self.dash.data = {rb.KEY_PHASE: (params or {}).get("phase"), rb.KEY_STATUS: "status"}
        if during is not None:
            during()
        crashed = step == self.crash
        return {"step": step, "opmode": opmode, "crashed": crashed, "errorMessage": "boom" if crashed else "",
                "timedOut": False, "appRestarted": False, "pidBefore": 100, "pidAfter": 100}

    def wait_or_crash(self, predicate, timeout):
        return 0.1 if predicate() else None

    def pull(self, name, required=True, local_name=None):
        run = name[len("turret_zero_"):]
        if name.startswith("turret_zero_") and run in self.fixture["runs"]:
            return copy.deepcopy(self.fixture["runs"][run]["result"]) if run != self.crash else {}
        raise AssertionError(f"unexpected pull {name}")


def drive(argv, replies, fixture=None, **kw):
    d = Dry(argv, replies, fixture or fx.good(brownout="ran", v2=True), **kw)
    d.run_turret_zero()
    checks = {name: state for name, state, _, _ in d.checks}
    return d, checks


def prompts(d):
    return [e[1] for e in d.events if e[0] in ("ask", "answer")]


def first(d, needle, kind=None, after=-1):
    for i, e in enumerate(d.events):
        if i > after and (kind is None or e[0] == kind) and any(needle in str(x) for x in e[1:]):
            return i
    return None


def ordered(*indices):
    return all(i is not None for i in indices) and list(indices) == sorted(indices) and len(set(indices)) == len(indices)


def run_at(d, step):
    return next((i for i, e in enumerate(d.events) if e[0] == "run" and e[1] == step), None)


def runs(d):
    return [e[1] for e in d.events if e[0] == "run"]


OK_NO = ["ok", "ok", "3.29", "3.27", "no"]
d, checks = drive(["--turret-zero"], OK_NO, fx.good())
expect("good: every run in order, brownout and V2 left out",
       runs(d) == ["turret_zero_" + r for r in ["quad", "split_plugged", "split_unplugged", "span"]
                   + [f"init_{i:02d}" for i in range(1, 11)] + ["init_moved", "agreement", "sign", "reset", "reset_after"]])
expect("good: 16 checks, all pass", len([n for n in checks if n.startswith("turret zero:")]) == 16
       and all(s == "PASS" for s in checks.values()))
expect("good: the config switch to bench_turret_zero and back", [e[1] for e in d.events if e[0] == "config"]
       == [rb.CONFIG_TURRET_ZERO, rb.CONFIG_MAIN])
root = ElementTree.fromstring(d.configs[rb.CONFIG_TURRET_ZERO])
outputs = [(e.tag, e.get("name"), e.get("port")) for e in root.iter() if "Motor" in e.tag or "Servo" in e.tag]
expect("config: br on motor port 2 is the only motor or servo it names",
       outputs == [("goBILDA5202SeriesMotor", "br", "2")])
expect("config: the OctoQuad on Control Hub I2C bus 2", any(e.tag == "OctoQuadFTC" and e.get("bus") == "2"
                                                             for e in root.iter()))
expect("wiring is metered before anything is plugged in or run",
       ordered(first(d, "METER the split cable"), first(d, ">>> PLUG", "ask"))
       and ordered(first(d, "METER the split cable"), first(d, "run")))
expect("the levels are metered with the branch unplugged, before it is plugged",
       ordered(first(d, "leave its OctoQuad branch unplugged"), first(d, "Type the highest reading"),
               first(d, "PLUG the split cable's OctoQuad branch")))
expect("the unplugged split run sits between unplugging the branch and plugging it back",
       ordered(first(d, "UNPLUG the split cable's OctoQuad branch and set"), first(d, "turret_zero_split_unplugged"),
               first(d, "PLUG the OctoQuad branch back")))
tracks = ["turret_zero_" + r for r in ("quad", "split_plugged", "split_unplugged", "span", "agreement")]
expect("every tracking run is told to finish after its prompt",
       all(d.events[first(d, step) + 2][0] == "push" and d.events[first(d, step) + 1][0] == "ask" for step in tracks))
expect("the INIT runs never START; the tracking runs do",
       all(e[3] is False for e in d.events if e[0] == "run" and "init_" in e[1])
       and all(e[3] is True for e in d.events if e[0] == "run" and e[1] in tracks))
span = rb.tz_span(fx.good()["runs"]["span"]["result"]["track"]["rows"])
after_span = [e[2] for e in d.events if e[0] == "run" and e[1] not in tracks[:4]]
expect("every run after the span gets its sign and pulse range", all(
    p.get("sign") == span["sign"] and p.get("minUs") == span["minUs"] and p.get("maxUs") == span["maxUs"]
    for p in after_span))
expect("only the first INIT times 50 reads", [e[2].get("reads") for e in d.events if e[0] == "run" and "init_" in e[1]]
       == [50] + [1] * 9 + [None])
expect("no prompt pulls the OctoQuad's I2C cable", not any("I2C cable" in p for p in prompts(d)))
TEARDOWN = "UNPLUG the split cable's OctoQuad branch from OctoQuad channel 4 now"
config_back = max(i for i, e in enumerate(d.events) if e[0] == "config")
expect("good: the bench ends asking for the OctoQuad branch out, before the config goes back",
       prompts(d)[-1].startswith(">>> " + TEARDOWN) and ordered(run_at(d, "turret_zero_reset_after"),
                                                                 first(d, TEARDOWN), config_back))
expect("the summary says brownout and V2 weren't run",
       "brownout: not run (no lab supply)" in "\n".join(d.summary) and "V2: not run" in "\n".join(d.summary))

V2_LEVELS = ["3.29", "3.28", "3.27"]
d, checks = drive(["--turret-zero-v2"], ["ok", "ok", "3.3", "3.25"] + V2_LEVELS + ["yes"])
order = runs(d)
BRANCH_OUT = "UNPLUG the split cable's OctoQuad branch from OctoQuad channel 4."
BRANCH_IN = "PLUG the split cable's OctoQuad branch into OctoQuad channel 4 (signal on its A pin) and set the V2's"
swap = first(d, "PLUG a REV-11-3174")
out_a = first(d, BRANCH_OUT, "ask")
meter_a = first(d, "METER the V2's 3.3 V pin")
in_a = first(d, BRANCH_IN, "ask")
out_s = first(d, BRANCH_OUT, "ask", after=run_at(d, "turret_zero_v2_a") or 0)
switch = first(d, "SWITCH the V2 to S")
meter_s = first(d, "Type the highest reading", after=switch or 0)
in_s = first(d, BRANCH_IN, "ask", after=meter_s or 0)
out_v1 = first(d, BRANCH_OUT, "ask", after=run_at(d, "turret_zero_v2_s") or 0)
v1_back = first(d, "PUT the V1 back")
in_v1 = first(d, "PLUG the OctoQuad branch back", after=v1_back or 0)
expect("v2: the branch is out for the swap and the switch to S, each metered before it goes back in",
       ordered(run_at(d, "turret_zero_reset_after"), out_a, swap, meter_a, in_a, run_at(d, "turret_zero_v2_a"), out_s,
               switch, meter_s, in_s, run_at(d, "turret_zero_v2_s"), out_v1, v1_back, in_v1, first(d, "lab supply")))
expect("v2: the levels are recorded", all(d.results["turret_zero"]["answers"].get(k) == v
                                          for k, v in zip(("v2SupplyV", "v2HighV", "v2sHighV"), V2_LEVELS)))
expect("v2 + supply: V2 before the brownout, the brownout last",
       order[-5:] == ["turret_zero_v2_a", "turret_zero_v2_s", "turret_zero_brownout_before", "turret_zero_brownout",
                      "turret_zero_brownout_after"])
expect("v2 + supply: the V2 runs get the V2's range", [e[2].get("version") for e in d.events if e[0] == "run"
                                                       and "v2_" in e[1]] == ["V2", "V2"])
expect("v2 + supply: the lab-supply question comes after V2", ordered(first(d, "turret_zero_v2_s"), first(d, "lab supply")))
expect("v2 + supply: the branch is unplugged before the dip and plugged back after it, before the INIT read",
       ordered(run_at(d, "turret_zero_brownout_before"), first(d, "UNPLUG the split cable's OctoQuad branch: the dip"),
               run_at(d, "turret_zero_brownout"), first(d, "DIP the Control Hub"),
               first(d, "PLUG the OctoQuad branch back", after=first(d, "DIP the Control Hub") or 0),
               run_at(d, "turret_zero_brownout_after")))
expect("v2 + supply: 22 checks, all pass", len([n for n in checks if n.startswith("turret zero:")]) == 22
       and all(s == "PASS" for s in checks.values()))

for levels, label in ((["3.3", "3.6", "3.27"], "on A"), (["3.3", "-3.3", "3.27"], "on A, reversed probes"),
                      (["5.0", "3.3", "3.27"], "on A, a 5 V supply")):
    d, checks = drive(["--turret-zero-v2"], ["ok", "ok", "3.3", "3.25"] + levels[:2] + ["no"])
    expect(f"V2 level wrong {label}: no V2 run, the branch stays out until the V1 is back, the level check fails",
           not any("v2_" in r for r in runs(d)) and first(d, BRANCH_IN) is None
           and ordered(first(d, "PUT the V1 back"), first(d, "PLUG the OctoQuad branch back",
                                                          after=first(d, "PUT the V1 back") or 0), first(d, "lab supply"))
           and checks.get(next((n for n in checks if n.startswith("turret zero: V2 on A: its supply")), "")) == "FAIL"
           and "V2: not run (stopped: the V2's levels on A" in "\n".join(d.summary))
d, checks = drive(["--turret-zero-v2"], ["ok", "ok", "3.3", "3.25", "3.29", "3.28", "3.9", "no"])
expect("V2 level wrong on S: the A run only, the branch kept out for S, the S level check fails",
       [r for r in runs(d) if "v2_" in r] == ["turret_zero_v2_a"]
       and first(d, BRANCH_IN, after=first(d, "SWITCH the V2 to S") or 0) is None
       and checks.get(next((n for n in checks if n.startswith("turret zero: V2 on S: its absolute")), "")) == "FAIL"
       and ordered(first(d, "SWITCH the V2 to S"), first(d, "PUT the V1 back"),
                   first(d, "PLUG the OctoQuad branch back", after=first(d, "PUT the V1 back") or 0)))

d, checks = drive(["--turret-zero"], ["ok", "pin 3 open"])
expect("wiring wrong: nothing plugged or run, the wiring check fails",
       runs(d) == [] and checks.get("turret zero: the REV-31-1815 and the split cable metered right (typed ok)") == "FAIL"
       and first(d, ">>> PLUG", "ask") is None)
expect("wiring wrong: no branch to unplug, so no teardown prompt", first(d, TEARDOWN) is None)
d, checks = drive(["--turret-zero"], ["pin 2 open", "ok"])
expect("REV-31-1815 wrong: nothing plugged or run, the wiring check fails",
       runs(d) == [] and first(d, ">>> PLUG", "ask") is None
       and checks.get("turret zero: the REV-31-1815 and the split cable metered right (typed ok)") == "FAIL")

for supply, high, label in (("3.3", "3.6", "level too high"), ("3.3", "-3.6", "a negative level (reversed probes)"),
                            ("3.3", "0", "a zero level"), ("5.0", "3.3", "a 5 V supply"),
                            ("2.5", "3.3", "a 2.5 V supply")):
    d, checks = drive(["--turret-zero"], ["ok", "ok", supply, high])
    expect(f"{label}: only the cable-1 run, the branch never plugged",
           runs(d) == ["turret_zero_quad"] and first(d, "PLUG the split cable's OctoQuad branch") is None
           and any(n.startswith("turret zero: the encoder's supply") and s == "FAIL" for n, s in checks.items())
           and first(d, TEARDOWN) is None)

short = fx.good()
short["runs"]["span"]["result"]["track"]["rows"] = fx.span_rows(turns=1.5)
d, checks = drive(["--turret-zero"], OK_NO, short)
expect("short span: stops before the INIT runs", runs(d)[-1] == "turret_zero_span" and first(d, "lab supply") is None)
expect("short span: the branch is still plugged, so the bench asks for it out", prompts(d)[-1].startswith(">>> " + TEARDOWN))
d = Dry(["--turret-zero"], OK_NO, fx.good(), interrupt_at="PLUG the split cable's OctoQuad branch into OctoQuad channel 4")
try:
    d.run_turret_zero()
    interrupted = False
except KeyboardInterrupt:
    interrupted = True
expect("Ctrl-C at the prompt to plug the branch in: the branch may be in, so the bench still asks for it out",
       interrupted and prompts(d)[-1].startswith(">>> " + TEARDOWN) and runs(d) == ["turret_zero_quad"])
d, checks = drive(["--turret-zero"], OK_NO, fx.good(), lose_hub="turret_zero_agreement")
expect("a runner error mid-bench: recorded, and the teardown prompt still comes",
       checks.get("turret_zero bench completed") == "FAIL" and prompts(d)[-1].startswith(">>> " + TEARDOWN)
       and "turret_zero_sign" not in runs(d))

flipped = fx.good()
flipped["runs"]["span"]["result"]["track"]["rows"] = fx.span_rows(sign=-1)
d, _ = drive(["--turret-zero"], OK_NO, flipped)
expect("a falling count gives every later run sign -1", {e[2].get("sign") for e in d.events if e[0] == "run"
                                                         and e[1] not in tracks[:4]} == {-1})

d, checks = drive(["--turret-zero"], OK_NO, fx.good(), crash="turret_zero_agreement")
expect("a crashed agreement run fails, and the later phases still run",
       checks.get("turret zero: agreement: the run finished") == "FAIL" and "turret_zero_reset_after" in runs(d))

d, checks = drive(["--turret-zero"], ["ok", "ok", "3.3", "3.25", "yes"], fx.good(brownout="ran"),
                  lose_hub="turret_zero_brownout")
expect("hub lost in the dip: recorded, settled, the branch plugged back and the INIT read after still run",
       d.results["turret_zero"].get("brownoutError") == "adb lost the hub" and ("settle",) in d.events
       and runs(d)[-1] == "turret_zero_brownout_after" and "whole-hub reset" in "\n".join(d.summary))
expect("hub lost in the dip: the runner waits for the hub as long as a reboot takes, then settles, before the read after",
       ordered(first(d, "the dip", "wait hub"), d.events.index(("settle",)), run_at(d, "turret_zero_brownout_after"))
       and d.events[first(d, "the dip", "wait hub")] == ("wait hub", "the dip", None))
d, checks = drive(["--turret-zero"], ["ok", "ok", "3.3", "3.25", "yes"], fx.good(brownout="ran"),
                  lose_hub="turret_zero_brownout", lose_as=rb.subprocess.TimeoutExpired(["adb"], 60))
expect("an adb timeout in the dip is handled as a lost hub too",
       "timed out" in d.results["turret_zero"].get("brownoutError", "") and first(d, "the dip", "wait hub") is not None
       and runs(d)[-1] == "turret_zero_brownout_after")
d, checks = drive(["--turret-zero"], ["ok", "ok", "3.3", "3.25", "yes"], fx.good(brownout="ran"))
expect("a dip the hub rides through: no wait for the hub", first(d, "the dip", "wait hub") is None)


class BackAdb:
    """Answers adb only from its third look, and has the app's process only from its fourth."""

    def __init__(self, never=False):
        self.looks, self.never, self.calls = 0, never, []

    def ensure_connected(self):
        self.looks += 1
        if self.never or self.looks < 3:
            raise rb.BenchError("adb can't connect to 192.168.43.1:5555")

    def pid(self):
        return "4321" if self.looks >= 4 else None

    def run(self, *args, check=True, timeout=60):
        self.calls.append(args)
        return ""


saved = rb.DASH_OFF_POLL_S, rb.DASH_OFF_BOOT_WAIT_S
rb.DASH_OFF_POLL_S, rb.DASH_OFF_BOOT_WAIT_S = 0.001, 0.5
try:
    w = object.__new__(rb.Runner)
    w.adb = BackAdb()
    w.wait_hub_back("the dip")
    expect("wait_hub_back: past a silent adb and a missing app to the app's process, then the log buffer again",
           w.adb.looks == 4 and w.adb.calls == [("shell", f"logcat -G {rb.LOG_BUFFER}")])
    w.adb = BackAdb(never=True)
    try:
        w.wait_hub_back("the dip")
        error = None
    except rb.BenchError as e:
        error = str(e)
    expect("wait_hub_back: a hub that never answers raises, naming the dip", error and "back from the dip" in error)
finally:
    rb.DASH_OFF_POLL_S, rb.DASH_OFF_BOOT_WAIT_S = saved
expect("brownout: the bench ends asking for the branch out after the INIT read after the dip",
       ordered(run_at(d, "turret_zero_brownout_after"), first(d, TEARDOWN)))

old_default = {"baseline", "hub", "loop", "failure", "threads", "gamepad", "edge", "write_recovery", "handover",
               "hardware_view", "turret", "llfault", "sloth", "clock", "pinpoint", "octoquad", "octoquad_absent",
               "absent", "planner_stop", "overlay_nan", "config_by_name", "freeze_dump"}
for argv, want, label in (([], old_default, "no bench flags"), (["--all"], None, "--all"),
                          (["--turret-zero"], {"turret_zero"}, "--turret-zero"),
                          (["--turret-zero-v2"], {"turret_zero"}, "--turret-zero-v2")):
    d = Dry(argv, [], fx.good())
    try:
        chosen = d.selection()
    except rb.BenchError:
        chosen = "refused"
    if want is None:
        expect(f"selection: {label} still refuses without a webcam, Limelight or Expansion Hub", chosen == "refused")
    else:
        expect(f"selection: {label} picks {sorted(want)}", chosen == want)
expect("BENCHES: the old order with turret_zero just before freeze_dump",
       [n for n in rb.BENCHES if n not in ("turret_zero", "dashboard_off")] == ["baseline", "hub", "loop", "failure",
       "threads", "gamepad", "edge", "write_recovery", "handover", "photon", "hardware_view", "turret", "webcam",
       "webcam_failure", "limelight", "llfault", "sloth", "clock", "pinpoint", "octoquad", "octoquad_absent",
       "expansion", "absent", "planner_stop", "overlay_nan", "config_by_name", "freeze_dump"]
       and rb.BENCHES.index("turret_zero") == rb.BENCHES.index("freeze_dump") - 1)

print(f"{passed} passed, {len(failed)} failed")
sys.exit(1 if failed else 0)
