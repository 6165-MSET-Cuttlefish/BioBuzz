"""Bench: Rangefinders offline: its verdicts on synthetic records (each check fails when weakened), the part a fault
follows through the swaps, the statics it saves for Configure Laser Rangefinder, and the bench end to end on
fake_hub.py with a person who moves the cables at each prompt: the prompts in order, sensor A woken on I2C while Bench:
Rangefinder Read polls, the 20 ms control and the as-found settings written back (A, silent on I2C once its pins are
written, woken again to read them back), sensor A back at its as-found settings even when the control's timing phase
crashes or meets Ctrl-C, a second Ctrl-C leaving the log line that says how to put A back, a sensor that never answers
on I2C (nothing written), a dead sensor, a slow swing, a bad cable named, and Ctrl-C at a prompt putting the config
back. No hub, no adb, no network.
Run with: python3 -I scripts/bench/offline/benches/rangefinders.py
"""
import _thread
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
OUT = OFFLINE / "out" / "rangefinders"
SOURCE = WORKTREE / "scripts/bench/benches/rangefinders.py"

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
BENCH = rb.module_bench("rangefinders")
rf = BENCH.module
A, B = rf.A, rf.B
KEYS = ("stillA", "stillB", "rateA", "rateB", "control", "readback", "swaps")
AS_FOUND = {"pin0": [1, 0, 1300], "pin1": [1, 0, 1300], "distanceModeRaw": 1, "timing": [10, 0], "roi": [6, 9, 9, 6],
            "opticalCenter": [8, 8], "distanceMm": 151.0, "distanceStatus": 0}
CLEAN = {"count": 6000, "meanMv": 400.0, "sdMv": 5.0, "p1Mv": 390, "p99Mv": 410, "minMv": 385, "maxMv": 415,
         "belowLowMv": 0, "stepsOverStepMv": 0, "largestStepMv": 30}
BAD = dict(CLEAN, minMv=4, maxMv=2926, belowLowMv=4, stepsOverStepMv=3, largestStepMv=2900)


def rate_rec(budget, p50, steps=400, fast=0.95):
    return {"channel": A, "budgetMs": budget, "thresholdMv": 15.0, "timedSteps": steps, "fastFraction": fast,
            "dwellP10Ms": p50 - 1, "dwellP50Ms": p50, "dwellP90Ms": p50 + 1, "stepsPerSecond": 60.0}


def arrangements(faults=()):
    """faults: (arrangement, channel) pairs whose phases read the swing."""
    out = {}
    for key, what, _ in rf.ARRANGEMENTS:
        phases = ("still", "flex A", "flex B") if key == "base" else ("still", "flex")
        out[key] = {"what": what}
        for phase in phases:
            out[key][phase] = {ch: dict(BAD if (key, ch) in faults else CLEAN) for ch in (A, B)}
    return out


def good_record(faults=()):
    rates = {"stored_A": rate_rec(10, 10.2), "stored_B": rate_rec(10, 10.4), "control_A": rate_rec(20, 20.1)}
    return {"arrangements": arrangements(faults), "rate": rates, "putBack": {},
            "control": {"aOn": "analog", "asFound": copy.deepcopy(AS_FOUND),
                        "readBack": {"answering": True, "settings": copy.deepcopy(AS_FOUND)}, "restored": True}}


CABLE_A = (("base", A), ("sensors swapped", A), ("cables swapped", B))
SENSOR_B = (("base", B), ("sensors swapped", A), ("cables swapped", A))
PORT_1 = (("base", A), ("sensors swapped", A), ("cables swapped", A))
SENSOR_A_TWICE = (("base", A), ("sensors swapped", B))


def verdicts(checks_fn, record):
    oks = [c[1] for c in checks_fn(record)]
    return dict(zip(KEYS, oks)) if len(oks) == len(KEYS) else {}


def only_fails(v, *keys):
    return bool(v) and {k for k, ok in v.items() if not ok} == set(keys)


def with_rate(key, rec):
    r = good_record(CABLE_A)
    r["rate"][key] = rec
    return r


def with_still(ch, **stats):
    r = good_record(CABLE_A)
    for phase in ("still",):
        r["arrangements"]["base"][phase][ch].update(stats)
    return r


CASES = [
    ("a bad cable A: still A fails, the swaps name cable A", lambda: good_record(CABLE_A),
     lambda v: only_fails(v, "stillA")),
    ("a bad sensor B: still B fails, the swaps name sensor B", lambda: good_record(SENSOR_B),
     lambda v: only_fails(v, "stillB")),
    ("a bad hub port 1: the swaps name it", lambda: good_record(PORT_1), lambda v: only_fails(v, "stillA")),
    ("no fault anywhere: the swaps say not reproduced, not PASS", lambda: good_record(),
     lambda v: only_fails(v, "swaps")),
    ("a fault that follows no single part: the swaps fail", lambda: good_record(SENSOR_A_TWICE),
     lambda v: only_fails(v, "stillA", "swaps")),
    ("a fault seen in one arrangement only: the swaps fail", lambda: good_record((("cables swapped", B),)),
     lambda v: only_fails(v, "swaps")),
    ("an arrangement that didn't run: the swaps fail",
     lambda: (lambda r: (r["arrangements"].pop("cables swapped"), r)[1])(good_record(CABLE_A)),
     lambda v: only_fails(v, "stillA", "swaps")),
    ("a still spread of 60 mV fails", lambda: with_still(B, p1Mv=380, p99Mv=440), lambda v: only_fails(v, "stillA", "stillB")),
    ("a spread of 49 mV passes", lambda: with_still(B, p1Mv=380, p99Mv=429), lambda v: only_fails(v, "stillA")),
    ("one still reading under 50 mV fails (and, a fault in no single part, the swaps)",
     lambda: with_still(B, belowLowMv=1), lambda v: only_fails(v, "stillA", "stillB", "swaps")),
    ("no still samples fail", lambda: with_still(B, count=0), lambda v: only_fails(v, "stillA", "stillB")),
    ("a 15 ms dwell at the stored budget fails", lambda: with_rate("stored_B", rate_rec(10, 15.0)),
     lambda v: only_fails(v, "stillA", "rateB")),
    ("a 20 ms dwell at the stored budget fails", lambda: with_rate("stored_A", rate_rec(10, 20.1)),
     lambda v: only_fails(v, "stillA", "rateA")),
    ("199 level steps: not enough motion", lambda: with_rate("stored_A", rate_rec(10, 10.2, steps=199)),
     lambda v: only_fails(v, "stillA", "rateA")),
    ("79% fast enough: not enough motion", lambda: with_rate("stored_B", rate_rec(10, 10.2, fast=0.79)),
     lambda v: only_fails(v, "stillA", "rateB")),
    ("no rate result fails", lambda: with_rate("stored_B", None), lambda v: only_fails(v, "stillA", "rateB")),
    ("a 10 ms dwell at the 20 ms control fails setup", lambda: with_rate("control_A", rate_rec(20, 10.2)),
     lambda v: only_fails(v, "stillA", "control")),
    ("the control with not enough motion fails", lambda: with_rate("control_A", rate_rec(20, 20.0, fast=0.5)),
     lambda v: only_fails(v, "stillA", "control")),
    ("the control not run fails it and the read-back",
     lambda: (lambda r: (r["rate"].pop("control_A"), r.update(control={"notRun": "no answer"}), r)[2])(
         good_record(CABLE_A)),
     lambda v: only_fails(v, "stillA", "control", "readback")),
    ("a read-back that differs from the as-found settings fails",
     lambda: (lambda r: (r["control"]["readBack"]["settings"].update(timing=[20, 0]), r)[1])(good_record(CABLE_A)),
     lambda v: only_fails(v, "stillA", "readback")),
    ("no read-back fails", lambda: (lambda r: (r["control"].pop("readBack"), r)[1])(good_record(CABLE_A)),
     lambda v: only_fails(v, "stillA", "readback")),
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
    expect("the swaps name cable A, sensor B and port 1 for their patterns",
           rf.bad_part(good_record(CABLE_A))[0] == "cable A" and rf.bad_part(good_record(SENSOR_B))[0] == "sensor B"
           and rf.bad_part(good_record(PORT_1))[0] == "Control Hub analog port 1")
    text = "\n".join(BENCH.summary(good_record(CABLE_A)))
    expect("summary: a row per arrangement, phase and port, the rate runs and what the swaps found",
           text.count("| base |") == 6 and "| sensors swapped | still | 1 | sensor B, cable A |" in text
           and "| stored_A (analog0) | 10 ms |" in text and "| control_A (analog0) | 20 ms |" in text
           and "the fault follows cable A" in text)
    s = rf.settings_for({"settings": AS_FOUND}, budget=20)
    expect("the statics for the control: 20 ms, the as-found period, SHORT, a 4x4 ROI and the as-found range",
           s["values"] == {"analogLowMm": 0, "analogHighMm": 1300, "distanceMode": "SHORT", "timingBudgetMs": 20,
                           "measurementPeriodMs": 0, "roiSize": 4} and not s["notes"])
    s = rf.settings_for({"settings": dict(AS_FOUND, pin0=[1, 40, 900], pin1=[1, 40, 900], timing=[33, 40],
                                          roi=[4, 11, 11, 4], distanceModeRaw=3)})
    expect("as-found settings Configure Laser Rangefinder can write are written back exactly",
           s["values"] == {"analogLowMm": 40, "analogHighMm": 900, "distanceMode": "LONG", "timingBudgetMs": 33,
                           "measurementPeriodMs": 40, "roiSize": 8} and not s["notes"])
    for label, odd in (("an off-centre ROI", dict(roi=[0, 15, 3, 12])),
                       ("a ROI under 4x4", dict(roi=[7, 8, 9, 6])),
                       ("a ROI that isn't square", dict(roi=[6, 9, 9, 4])),
                       ("a centred ROI off the 0-15 grid", dict(roi=[-2, 17, 17, -2], opticalCenter=[8, 8])),
                       ("pin 0 not in analog mode", dict(pin0=[0, 0, 0], pin1=[0, 0, 0])),
                       ("pin 1 unlike pin 0", dict(pin1=[1, 0, 1000])),
                       ("a period too short for the as-found budget", dict(timing=[10, 12])),
                       ("a budget over 1000 ms", dict(timing=[1200, 0])),
                       ("a distance mode Configure Laser Rangefinder can't write", dict(distanceModeRaw=0))):
        try:
            rf.settings_for({"settings": dict(AS_FOUND, **odd)}, budget=20)
            refused = False
        except rb.BenchError as e:
            refused = "nothing was written" in str(e)
        expect(f"{label}: refused, so nothing is written", refused)
    s = rf.settings_for({"settings": dict(AS_FOUND, timing=[10, 13])}, budget=20)
    expect("a measurement period too short for the 20 ms control is written as 0, and noted; as found it is kept",
           s["values"]["measurementPeriodMs"] == 0 and len(s["notes"]) == 1
           and rf.settings_for({"settings": dict(AS_FOUND, timing=[10, 13])})["values"]["measurementPeriodMs"] == 13)
    diff = rf.config_diff({"timingBudgetMs": 20, "distanceMode": "SHORT"})["configDiff"]["__value"][rf.CONFIGURE]
    expect("SAVE_CONFIG: ints as int, the distance mode as an enum by name",
           diff["__value"] == {"timingBudgetMs": {"__type": "int", "__value": 20},
                               "distanceMode": {"__type": "enum", "__value": "SHORT"}})


MUTATIONS = [
    ("the spread limit is raised", "SPREAD_MV = 50", "SPREAD_MV = 1e9"),
    ("readings under 50 mV are allowed", ' and stats.get("belowLowMv") == 0,', ","),
    ("200 level steps aren't needed", "MIN_STEPS = 200", "MIN_STEPS = 0"),
    ("80% fast enough isn't needed", "MIN_FAST = 0.8", "MIN_FAST = 0.0"),
    ("any dwell passes at 10 ms", "BUDGET_MS: (9.0, 12.0)", "BUDGET_MS: (0.0, 1e9)"),
    ("any dwell passes the control", "CONTROL_MS: (18.0, 22.0)", "CONTROL_MS: (0.0, 1e9)"),
    ("any read-back passes", "    return not differ, (", "    return True, ("),
    ("no fault anywhere passes the swaps", "    if not bad:\n        return None,", "    if not bad:\n        return 'x',"),
    ("the clean channels are ignored", "set.intersection(*bad) - set().union(*good)", "set.intersection(*bad)"),
]


def mutated(old, new):
    text = SOURCE.read_text()
    if text.count(old) != 1:
        return None
    OUT.mkdir(parents=True, exist_ok=True)
    folder = pathlib.Path(tempfile.mkdtemp(prefix="mutant-", dir=OUT))
    (folder / "rangefinders.py").write_text(text.replace(old, new))
    (folder / "order.txt").write_text("rangefinders\n")
    return rb.load_benches(folder)[0]


def mutation_cases():
    for label, old, new in MUTATIONS:
        bench = mutated(old, new)
        if bench is None:
            expect(f"weakened: {label}: the text to weaken is in rangefinders.py once", False)
            continue
        caught = [case for case, ok in run_cases(bench.checks) if not ok]
        expect(f"weakened: {label}: a verdict case fails ({caught[:1]})", bool(caught))


class Person:
    """Does what each prompt says to the station, and keeps the prompts in order."""
    ACTIONS = (
        ("Unplug the Prism from", lambda s: setattr(s, "prism", False)),
        ("Plug the Prism back", lambda s: setattr(s, "prism", True)),
        ("Unplug rangefinder A's cable from Control Hub analog port 1", lambda s: setattr(s, "a_on", "hand")),
        ("Unplug rangefinder A's cable from Control Hub I2C port 3", lambda s: setattr(s, "a_on", "hand")),
        ("Plug rangefinder A's cable into Control Hub I2C port 3 now", lambda s: s.plug_i2c()),
        ("Rangefinder A didn't answer on I2C. Unplug", lambda s: setattr(s, "a_on", "hand")),
        ("Move rangefinder A's cable from Control Hub I2C port 3 to analog port 1", lambda s: s.to_analog()),
        ("Swap the two sensors at the sensor end", lambda s: setattr(s, "sensors_swapped", True)),
        ("Put each sensor back on its own cable", lambda s: setattr(s, "sensors_swapped", False)),
        ("Swap the two cables at the hub", lambda s: setattr(s, "cables_swapped", True)),
        ("Swap the cables back at the hub", lambda s: setattr(s, "cables_swapped", False)),
    )

    def __init__(self, station, log, interrupt_on=None, close_on=None):
        """interrupt_on: a prompt to press Ctrl-C at, or {prompt: which times it is seen (1 = the first)}."""
        self.station, self.log, self.prompts = station, log, []
        self.interrupt_on = {interrupt_on: (1,)} if isinstance(interrupt_on, str) else dict(interrupt_on or {})
        self.close_on = close_on

    def isatty(self):
        return True

    def readline(self):
        prompt = next((l.split(">>> ", 1)[1] for l in reversed(self.log.read_text().splitlines()) if ">>> " in l
                       and "GO:" not in l), "")
        self.prompts.append(prompt)
        for needle, times in self.interrupt_on.items():
            if needle in prompt and sum(needle in p for p in self.prompts) in times:
                raise KeyboardInterrupt
        if self.close_on and self.close_on in prompt:
            return ""
        with self.station.lock:
            for needle, action in self.ACTIONS:
                if needle in prompt:
                    action(self.station)
        return "\n"


class Station(fake_hub.Hub):
    """fake_hub's Control Hub with rangefinder A and B on its analog ports, a person moving them, and the OpModes the
    bench runs answering from what is plugged where."""

    def __init__(self, faulty=(), dead_b=False, slow=False, crash_label=None, never_wakes=False, interrupt_label=None,
                 **kw):
        super().__init__(**kw)
        self.faulty, self.dead_b, self.slow = set(faulty), dead_b, slow
        self.crash_label, self.never_wakes, self.interrupt_label = crash_label, never_wakes, interrupt_label
        self.a_on, self.a_mode, self.prism = "analog", "analog", True
        self.sensors_swapped = self.cables_swapped = False
        self.a = copy.deepcopy(AS_FOUND)
        self.statics, self.messages, self.switched, self.saves = {}, [], [], []
        self.reader = None
        self.opmode_extra = [rf.CONFIGURE]

    def _restart_from_prefs(self):
        super()._restart_from_prefs()
        self.switched.append(self.active["name"])

    def wiring(self):
        sensor = {A: "sensor B" if self.sensors_swapped != self.cables_swapped else "sensor A",
                  B: "sensor A" if self.sensors_swapped != self.cables_swapped else "sensor B"}
        cable = {A: "cable B" if self.cables_swapped else "cable A", B: "cable A" if self.cables_swapped else "cable B"}
        port = {A: "Control Hub analog port 1", B: "Control Hub analog port 3"}
        return {ch: {sensor[ch], cable[ch], port[ch]} for ch in (A, B)}

    def channel(self, ch):
        parts = self.wiring()[ch]
        if self.dead_b and "sensor B" in parts:
            return dict(CLEAN, meanMv=3.0, minMv=0, maxMv=10, p1Mv=1, p99Mv=9, belowLowMv=6000)
        return dict(BAD if parts & self.faulty else CLEAN)

    def plug_i2c(self):
        self.a_on = "i2c"
        if self.reader is not None and not self.never_wakes:
            self.a_mode = "i2c"
            self.read_result(self.reader)

    def to_analog(self):
        self.a_on = "analog"
        if self.a["pin0"][0] == 1 and self.a["pin1"][0] == 1:
            self.a_mode = "analog"

    def result(self, name, obj):
        self.bench[f"files/bench/{name}.json"] = json.dumps(obj)

    def read_result(self, label):
        answering = self.a_on == "i2c" and self.a_mode == "i2c"
        self.result(f"rangefinder_read_{label}", {"answering": answering, "firstAnswerMs": 900.0,
                                                  **({"settings": copy.deepcopy(self.a)} if answering else {})})
        self.status["warningMessage"] = "" if answering else "Error communicating with I2C device(s) 'Laser'"

    def poll_result(self, params):
        label = params["label"]
        if params["mode"] == "steady":
            phase = {ch: self.channel(ch) for ch in (A, B)}
            self.result(f"analog_poll_{label}", {"phases": {f"steady {label}": phase}})
            return
        sensor = next(p for p in self.wiring()[params["channel"]] if p.startswith("sensor"))
        budget = self.a["timing"][0] if sensor == "sensor A" else 10
        self.result(f"analog_poll_{label}", {"phases": {f"rate {label}": rate_rec(
            budget, budget + 0.3, fast=0.4 if self.slow else 0.95)}})

    def configure(self):
        if not (self.a_on == "i2c" and self.a_mode == "i2c"):
            self.status["errorMessage"] = "Laser rangefinder not answering on I2C"
            self.set_active(rb.IDLE_OPMODE, "STOPPED")
            return
        s = self.statics
        size, (cx, cy) = s["roiSize"], self.a["opticalCenter"]
        tlx, tly = cx - size // 2, cy + size // 2 - 1
        self.a.update(timing=[s["timingBudgetMs"], s["measurementPeriodMs"]],
                      distanceModeRaw={"SHORT": 1, "MEDIUM": 2, "LONG": 3}[s["distanceMode"]],
                      roi=[tlx, tly, tlx + size - 1, tly - size + 1],
                      pin0=[1, s["analogLowMm"], s["analogHighMm"]], pin1=[1, s["analogLowMm"], s["analogHighMm"]])
        self.a_mode = "analog"
        self.dash.broadcast({"type": "RECEIVE_TELEMETRY", "telemetry": [{"data": {
            "Answering on I2C": "true", "Timing [Budget, Period]": str(self.a["timing"])}, "timestamp": self.clock,
            "fieldOverlay": {"ops": []}}]})

    def on_message(self, session, msg):
        kind = msg.get("type")
        with self.lock:
            params = json.loads(self.bench.get("files/bench/params.json") or "{}")
            if kind in ("INIT_OP_MODE", "START_OP_MODE"):
                self.messages.append((kind, msg.get("opModeName") or self.status["activeOpMode"], params,
                                      self.active["name"], dict(self.statics)))
            if kind == "SAVE_CONFIG":
                leaves = msg["configDiff"]["__value"][rf.CONFIGURE]["__value"]
                self.statics.update({k: v["__value"] for k, v in leaves.items()})
                self.saves.append(dict(self.statics))
                return
            if kind == "GET_CONFIG":
                session.send({"type": "RECEIVE_CONFIG", "configRoot": {"__type": "custom", "__value": {
                    rf.CONFIGURE: {"__type": "custom", "__value": {
                        k: {"__type": "enum" if k == "distanceMode" else "int", "__value": v}
                        for k, v in self.statics.items()}}}}})
                return
            if kind == "INIT_OP_MODE" and msg["opModeName"] == rf.READ:
                self.reader = params["label"]
                self.read_result(self.reader)
            if kind == "INIT_OP_MODE" and msg["opModeName"] == rf.POLL:
                self.status["errorMessage"] = ""
                if params.get("label") == self.crash_label:
                    self.status["errorMessage"] = f"java.lang.IllegalStateException: boom in {self.crash_label}"
                    self.set_active(rb.IDLE_OPMODE, "STOPPED")
                    return
                self.poll_result(params)
                if params.get("label") == self.interrupt_label:
                    self.interrupt_label = None
                    _thread.interrupt_main()
            if kind == "STOP_OP_MODE":
                self.reader = None
        super().on_message(session, msg)
        with self.lock:
            if kind == "START_OP_MODE" and self.status["activeOpMode"] == rf.CONFIGURE:
                self.configure()


def station_run(argv, hub, interrupt_on=None, close_on=None):
    OUT.mkdir(parents=True, exist_ok=True)
    out = pathlib.Path(tempfile.mkdtemp(prefix="run-", dir=OUT)) / "run"
    handlers = signal.getsignal(signal.SIGINT), signal.getsignal(signal.SIGHUP)
    person = Person(hub, out / "run.log", interrupt_on, close_on)
    saved, sys.stdin = sys.stdin, person
    code = None
    try:
        with hub.installed(rb), contextlib.redirect_stdout(io.StringIO()):
            hub.opmode_names += hub.opmode_extra
            try:
                code = rb.Runner(rb.parse_args(["--out", str(out)] + list(argv))).run()
            except KeyboardInterrupt:
                code = "interrupted"
    finally:
        sys.stdin = saved
        signal.signal(signal.SIGINT, handlers[0])
        signal.signal(signal.SIGHUP, handlers[1])
    return code, out, person


def check_line(summary, part):
    return next((l for l in summary.splitlines() if l.startswith("- ") and part in l), "")


def restored(hub):
    return hub.active["name"] == "betta_decode" and not [n for n in hub.configs if n.startswith("bench_")]


def back_home(hub):
    return (hub.a == AS_FOUND and hub.a_on == "analog" and hub.a_mode == "analog" and hub.prism
            and not hub.sensors_swapped and not hub.cables_swapped)


def starts(hub, opmode):
    return [m for m in hub.messages if m[0] == "START_OP_MODE" and m[1] == opmode]


ARGV = ["--rangefinders", "--interactive", "--ports-empty"]
ORDER = ["Aim rangefinder A", "swing the fast target through rangefinder A's beam",
         "swing the fast target through rangefinder B's beam", "Unplug the Prism",
         "Unplug rangefinder A's cable from Control Hub analog port 1", "Plug rangefinder A's cable into Control Hub I2C",
         "Move rangefinder A's cable from Control Hub I2C port 3 to analog port 1",
         "swing the fast target through rangefinder A's beam",
         "Unplug rangefinder A's cable from Control Hub analog port 1", "Plug rangefinder A's cable into Control Hub I2C",
         "Unplug rangefinder A's cable from Control Hub I2C port 3", "Plug rangefinder A's cable into Control Hub I2C",
         "Move rangefinder A's cable from Control Hub I2C port 3 to analog port 1", "Plug the Prism back",
         "flex rangefinder A's cable", "flex rangefinder B's cable", "Swap the two sensors", "Aim both rangefinders",
         "flex both cables", "Put each sensor back", "Swap the two cables at the hub", "Aim both rangefinders",
         "flex both cables", "Swap the cables back"]


def in_order(prompts, needles):
    return len(prompts) == len(needles) and all(n in p for p, n in zip(prompts, needles))


def on_the_fake_hub():
    hub = Station()
    code, out, person = station_run(["--rangefinders", "--ports-empty"], hub)
    expect("without --interactive: stops after the probe saying so, nothing asked",
           code == 1 and "so it needs --interactive" in (out / "summary.md").read_text() and not person.prompts
           and not [m for m in hub.messages if m[1] != "Bench: Probe"] and restored(hub))

    hub = Station(faulty={"cable A"})
    code, out, person = station_run(ARGV, hub)
    summary = (out / "summary.md").read_text()
    expect("the prompts in order, one action each", in_order(person.prompts, ORDER))
    expect("config switches: analog, I2C for the read and the write, analog, I2C for the write back, analog, main",
           hub.switched[2:8] == [rf.CONFIG_ANALOG, rf.CONFIG_I2C, rf.CONFIG_ANALOG, rf.CONFIG_I2C, rf.CONFIG_ANALOG,
                                 rb.CONFIG_MAIN])
    configure_starts = starts(hub, rf.CONFIGURE)
    expect("Configure Laser Rangefinder STARTed twice, each on statics saved and read back first: 20 ms, then as found",
           [m[4]["timingBudgetMs"] for m in configure_starts] == [20, 10]
           and all(m[4]["distanceMode"] == "SHORT" and m[4]["roiSize"] == 4 for m in configure_starts)
           and len(hub.saves) == 2)
    expect("Bench: Rangefinder Read only ever INITed, never STARTed", not starts(hub, rf.READ)
           and len([m for m in hub.messages if m[1] == rf.READ]) == 3)
    expect("sensor A ends at its as-found settings, back on analog port 1, everything put back", back_home(hub))
    expect("raw/rangefinder_asfound.json holds A's settings from before the bench",
           json.loads((out / "raw" / "rangefinder_asfound.json").read_text()) == {"A": AS_FOUND})
    expect("the checks: still A FAILs, the rates and the control pass, the read-back passes, the swaps name cable A",
           check_line(summary, "still: rangefinder A").startswith("- FAIL")
           and check_line(summary, "still: rangefinder B").startswith("- PASS")
           and check_line(summary, "rate: rangefinder A").startswith("- PASS")
           and check_line(summary, "control: at a 20 ms").startswith("- PASS")
           and check_line(summary, "reads back").startswith("- PASS")
           and "the fault follows cable A" in check_line(summary, "swaps:") and restored(hub))
    rates = json.loads((out / "runner_results.json").read_text())["rangefinders"]["rate"]
    expect("the rate runs: the stored budget on A and B, then 20 ms on A, each threshold 3x its still SD",
           [(k, v["channel"], v["budgetMs"], v["thresholdMv"]) for k, v in rates.items()]
           == [("stored_A", A, 10, 15.0), ("stored_B", B, 10, 15.0), ("control_A", A, 20, 15.0)])
    expect("the rate runs' params: back-to-back GetADC on that channel, 3x the still SD, the 10 ms motion gate",
           [(m[2]["channel"], m[2]["thresholdMv"], m[2]["expectedMs"]) for m in starts(hub, rf.POLL)
            if m[2]["mode"] == "rate"] == [(A, 15.0, 10.0), (B, 15.0, 10.0), (A, 15.0, 10.0)])

    hub = Station(crash_label="rate_control_A")
    code, out, person = station_run(ARGV, hub)
    summary = (out / "summary.md").read_text()
    expect("the control's timing phase crashes: A is written back to its as-found settings before the bench fails",
           back_home(hub) is False and hub.a == AS_FOUND and hub.a_on == "analog"
           and "rangefinders bench completed" in summary and "boom in rate_control_A" in summary
           and [m[4]["timingBudgetMs"] for m in starts(hub, rf.CONFIGURE)] == [20, 10] and restored(hub))
    expect("and the run's log says what to put back by hand: the Prism",
           "!! Put back by hand: the Prism into Control Hub I2C port 3" in (out / "run.log").read_text())

    hub = Station(never_wakes=True)
    code, out, person = station_run(ARGV, hub)
    summary = (out / "summary.md").read_text()
    i2c = [i for i, p in enumerate(person.prompts) if "into Control Hub I2C port 3 now" in p]
    expect("A never answers on I2C: asked to replug once, nothing written, A moved from I2C port 3 to analog port 1, "
           "the bench goes on",
           not starts(hub, rf.CONFIGURE) and hub.a == AS_FOUND and hub.a_on == "analog"
           and sum("didn't answer on I2C" in p for p in person.prompts) == 1 and len(i2c) == 2
           and person.prompts[i2c[-1] + 1].startswith("Move rangefinder A's cable from Control Hub I2C port 3 to analog "
                                                     "port 1.")
           and "not run: rangefinder A didn't answer on I2C twice" in check_line(summary, "control: at a 20 ms")
           and back_home(hub) and restored(hub))

    hub = Station()
    hub.a["roi"] = [0, 15, 3, 12]
    code, out, person = station_run(ARGV, hub)
    summary = (out / "summary.md").read_text()
    expect("an as-found ROI the bench can't write back: nothing saved or written, A moved back, the bench goes on",
           not starts(hub, rf.CONFIGURE) and not hub.saves and hub.a == dict(AS_FOUND, roi=[0, 15, 3, 12])
           and hub.a_on == "analog" and hub.prism
           and "can't be written back as found" in check_line(summary, "control: at a 20 ms")
           and check_line(summary, "reads back").startswith("- FAIL") and restored(hub))

    hub = Station(dead_b=True)
    code, out, person = station_run(ARGV, hub)
    summary = (out / "summary.md").read_text()
    expect("rangefinder B unplugged: the bench stops after the still phase, saying where to plug it",
           "rangefinder B on Control Hub analog port 3 read under 50 mV the whole time" in summary
           and len(person.prompts) == 1 and restored(hub))

    hub = Station(slow=True, faulty={"sensor B"})
    code, out, person = station_run(ARGV, hub)
    summary = (out / "summary.md").read_text()
    expect("a slow swing: the rate and control checks FAIL as not enough motion, the swaps name sensor B",
           "not enough motion" in check_line(summary, "rate: rangefinder A")
           and "not enough motion" in check_line(summary, "control: at a 20 ms")
           and "the fault follows sensor B" in check_line(summary, "swaps:") and back_home(hub))

    move = "Move rangefinder A's cable from Control Hub I2C port 3"
    hub = Station()
    code, out, person = station_run(ARGV, hub, interrupt_on=move)
    log = (out / "run.log").read_text()
    after = person.prompts[person.prompts.index(next(p for p in person.prompts if move in p)) + 1:]
    expect("Ctrl-C at the move back to analog after the 20 ms write: A woken, written back, woken and read back at its "
           "as-found settings, moved to analog port 1, then the run stops with the config put back",
           code == "interrupted" and in_order(after, ["Unplug rangefinder A's cable from Control Hub I2C port 3",
                                                      "Plug rangefinder A's cable into Control Hub I2C",
                                                      "Unplug rangefinder A's cable from Control Hub I2C port 3",
                                                      "Plug rangefinder A's cable into Control Hub I2C", move])
           and hub.a == AS_FOUND and hub.a_on == "analog" and hub.a_mode == "analog"
           and [m[4]["timingBudgetMs"] for m in starts(hub, rf.CONFIGURE)] == [20, 10]
           and rf.LEFT_WRITTEN not in log and restored(hub))
    expect("and the run's log says what is left to put back by hand: only the Prism",
           "!! Put back by hand: the Prism into Control Hub I2C port 3" in log)

    hub = Station(interrupt_label="rate_control_A")
    code, out, person = station_run(ARGV, hub)
    log = (out / "run.log").read_text()
    expect("Ctrl-C during the control's timing run: A ends read back at its as-found settings, on analog port 1",
           code == "interrupted" and "KeyboardInterrupt" in log and hub.a == AS_FOUND and hub.a_on == "analog"
           and hub.a_mode == "analog" and [m[4]["timingBudgetMs"] for m in starts(hub, rf.CONFIGURE)] == [20, 10]
           and person.prompts[-1].startswith(move) and rf.LEFT_WRITTEN not in log and restored(hub))

    hub = Station()
    code, out, person = station_run(ARGV, hub, interrupt_on={
        move: (1,), "Unplug rangefinder A's cable from Control Hub I2C port 3": (1,)})
    log = (out / "run.log").read_text()
    expect("Ctrl-C again at the write back's first prompt: the run stops, the log says how to put A back by hand",
           code == "interrupted" and rf.LEFT_WRITTEN in log and hub.a["timing"] == [20, 0]
           and len(starts(hub, rf.CONFIGURE)) == 1 and restored(hub))

    hub = Station()
    code, out, person = station_run(ARGV, hub, close_on="Swap the two sensors")
    summary = (out / "summary.md").read_text()
    expect("the terminal closed at a swap: the bench fails, never taken as done, nothing left to put back",
           "the terminal closed before you confirmed" in summary and not hub.sensors_swapped and restored(hub)
           and "Put back by hand" not in (out / "run.log").read_text() and hub.prism)


def declaration():
    expect("declared: flag-only, a person with --interactive, no ports driven, its two bench configs",
           not BENCH.default and BENCH.person is not None and BENCH.statement is None and not BENCH.drives
           and BENCH.person_flags == ("--rangefinders", "--interactive")
           and BENCH.configs == (rf.CONFIG_ANALOG, rf.CONFIG_I2C) and BENCH.opmodes == (rf.POLL, rf.READ))
    args = rb.parse_args(next(e for e in rb.needs_a_person() if e[0] == "rangefinders")[2])
    expect("its Needs a person command parses with --interactive", args.rangefinders and args.interactive)
    expect("bench_rangefinders copies laser_rangefinder.xml's two analog inputs and nothing else",
           rf.analog_lines() == ['<AnalogInput name="analog0" port="1" />', '<AnalogInput name="analog1" port="3" />'])


def main():
    for section in (verdict_cases, mutation_cases, declaration, on_the_fake_hub):
        try:
            section()
        except Exception as e:
            expect(f"{section.__name__} ran to its end ({type(e).__name__}: {e})", False)
            traceback.print_exc()
    print(f"{passed} passed, {len(failed)} failed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
