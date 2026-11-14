"""Bench: Fault Render offline: its answer parsing and verdicts on synthetic records (each check fails when weakened),
and the bench end to end on fake_hub.py: Bench: Limelight Fault on bench_llfault, then Bench: Fault Render's five
phases moved by the dashboard cue and its first-frame INIT, the eight questions in order, a Driver Hub screenshot, a
crash mid-phase, a closed terminal and Ctrl-C at a question putting the config back. No hub, no adb, no network.
Run with: python3 -I scripts/bench/offline/benches/fault_render.py
"""
import contextlib
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
OUT = OFFLINE / "out" / "fault_render"
SOURCE = WORKTREE / "scripts/bench/benches/fault_render.py"
DS_SERIAL = "DRIVERHUB01"

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
BENCH = rb.module_bench("fault_render")
fr = BENCH.module
KEYS = ("llfault",) + fr.PHASES + ("modules", "firstFrame")
ASKED = ("llfault", "short", "modules", "long", "two", "slow", "off", "firstFrame")
GOOD_TYPED = {"llfault": "y", "short": "y", "modules": "y", "long": "y", "two": "y", "slow": "y", "off": "n",
              "firstFrame": "y"}


def good_record():
    phases = {name: {"seconds": 15.0, "dsFramesOpened": 60, "dsFramesPerSecond": 4.0, "faults": ["Bench"],
                     "faultLineHtmlChars": 90} for name in ("waiting",) + fr.PHASES}
    return {"answers": {k: {"answer": a, "typed": a} for k, a in GOOD_TYPED.items()},
            "llfault": {"crashed": False, "error": "", "timedOut": False, "firstFaultMs": 420.0},
            "phasesRun": {"crashed": False, "error": "", "timedOut": False}, "phases": phases,
            "screencaps": {"llfault": "raw/ds_fault.png"}}


def verdicts(checks_fn, record):
    out = {}
    for name, ok, detail, means in checks_fn(record):
        key = "setup" if "five phases" in name else next(k for k in KEYS if name.endswith(fr.CHECK_NAMES[k]))
        out[key] = ok
    return out


def answered(key, typed):
    r = good_record()
    r["answers"][key] = {"answer": typed, "typed": typed}
    return r


def without(key):
    r = good_record()
    del r["answers"][key]
    return r


def only_fails(v, *keys):
    return {k for k, ok in v.items() if not ok} == set(keys)


CASES = [("every answer as expected: all pass", good_record, lambda v: only_fails(v))]
for _key in ("llfault", "short", "long", "two", "slow", "modules", "firstFrame"):
    CASES.append((f"{_key} answered n fails its check alone", (lambda k: lambda: answered(k, "n"))(_key),
                  (lambda k: lambda v: only_fails(v, k))(_key)))
    CASES.append((f"{_key} not asked fails its check alone", (lambda k: lambda: without(k))(_key),
                  (lambda k: lambda v: only_fails(v, k))(_key)))
CASES += [
    ("a FAULT Hidden line seen with Driver Station telemetry off fails", lambda: answered("off", "y"),
     lambda v: only_fails(v, "off")),
    ("off not asked fails", lambda: without("off"), lambda v: only_fails(v, "off")),
    ("Bench: Limelight Fault crashing fails its check although the answer was y",
     lambda: (lambda r: (r["llfault"].update(crashed=True, error="boom"), r)[1])(good_record()),
     lambda v: only_fails(v, "llfault")),
    ("a phase with no result fails the setup check",
     lambda: (lambda r: (r["phases"].update(slow=None), r)[1])(good_record()), lambda v: only_fails(v, "setup")),
    ("Bench: Fault Render crashing fails the setup check",
     lambda: (lambda r: (r["phasesRun"].update(crashed=True, error="boom"), r)[1])(good_record()),
     lambda v: only_fails(v, "setup")),
    ("no phases at all fails the setup check", lambda: (lambda r: (r.pop("phases"), r)[1])(good_record()),
     lambda v: only_fails(v, "setup")),
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
    names = [c[0] for c in BENCH.checks(good_record())]
    expect("checks: the setup check, then one per question", len(names) == 1 + len(KEYS) == len(set(names)))
    expect("parse: y, yes, n, no in any case, a photo path after the answer",
           fr.parse_answer("y") == ("y", None) and fr.parse_answer(" YES ") == ("y", None)
           and fr.parse_answer("No") == ("n", None) and fr.parse_answer("n ~/Desktop/ds 1.jpg") == ("n", "~/Desktop/ds 1.jpg"))
    expect("parse: anything else, or nothing, is not an answer",
           all(fr.parse_answer(t) is None for t in ("", "   ", "maybe", "yn", "1", "ok")))
    text = "\n".join(BENCH.summary(good_record()))
    expect("summary: one row per question, frames per second from the phase results",
           all(f"| {fr.CHECK_NAMES[k]} |" in text for k in KEYS) and "| 4.0 | 90 |" in text)


MUTATIONS = [
    ("off is expected to show the fault", '"off": NO,', '"off": YES,'),
    ("any answer passes", 'return entry["answer"] == expected, detail', "return True, detail"),
    ("a question not asked passes", '        return False, "not asked:', '        return True, "not asked:'),
    ("the setup check ignores missing phases", 'ok = reached == list(PHASES) and not run.get("crashed")',
     "ok = True"),
    ("a Bench: Limelight Fault crash is ignored", '    if llfault.get("crashed"):', "    if False:"),
]


def mutated(old, new):
    text = SOURCE.read_text()
    if text.count(old) != 1:
        return None
    OUT.mkdir(parents=True, exist_ok=True)
    folder = pathlib.Path(tempfile.mkdtemp(prefix="mutant-", dir=OUT))
    (folder / "fault_render.py").write_text(text.replace(old, new))
    (folder / "order.txt").write_text("fault_render\n")
    return rb.load_benches(folder)[0]


def mutation_cases():
    for label, old, new in MUTATIONS:
        bench = mutated(old, new)
        if bench is None:
            expect(f"weakened: {label}: the text to weaken is in fault_render.py once", False)
            continue
        caught = [case for case, ok in run_cases(bench.checks) if not ok]
        expect(f"weakened: {label}: a verdict case fails ({caught[:1]})", bool(caught))
    folder = pathlib.Path(tempfile.mkdtemp(prefix="mutant-", dir=OUT))
    (folder / "fault_render.py").write_text(SOURCE.read_text().replace(
        'answer = YES if first in ("y", "yes") else NO if first in ("n", "no") else None',
        'answer = YES if first not in ("n", "no") else NO'))
    (folder / "order.txt").write_text("fault_render\n")
    weak = rb.load_benches(folder)[0].module
    expect("weakened: a parser that takes anything for yes is caught", weak.parse_answer("maybe") is not None)


class FakeStdin:
    def __init__(self, answers, hub, tty=True, interrupt_at=None):
        self.answers, self.hub, self.tty, self.reads, self.interrupt_at = list(answers), hub, tty, [], interrupt_at

    def isatty(self):
        return self.tty

    def readline(self):
        self.reads.append(len(self.hub.calls))
        if self.interrupt_at is not None and len(self.reads) == self.interrupt_at:
            raise KeyboardInterrupt
        return self.answers.pop(0) + "\n" if self.answers else ""


class Station(fake_hub.Hub):
    """fake_hub's Control Hub with a dashboard config tree, Bench: Fault Render's results following the cue, and a
    Driver Hub on this Mac's USB."""

    def __init__(self, crash_phase=None, ignore_cue=False, ds=False, **kw):
        super().__init__(**kw)
        self.crash_phase, self.ignore_cue = crash_phase, ignore_cue
        self.cue, self.cues, self.messages, self.switched, self.ds_calls = 0, [], [], [], []
        if ds:
            self.extra_usb.append(DS_SERIAL)

    def _restart_from_prefs(self):
        super()._restart_from_prefs()
        self.switched.append(dict(self.active))

    def _command(self, serial, args):
        if serial == DS_SERIAL:
            self.ds_calls.append(list(args))
            if args[:1] == ["pull"]:
                pathlib.Path(args[2]).write_bytes(b"\x89PNG fake")
                return "1 file pulled\n", "", 0
            return ("\n" if args[:1] == ["shell"] and "getprop" in args[1] else ""), "", 0
        return super()._command(serial, args)

    def config_root(self):
        return {"__type": "custom", "__value": {fr.OPMODE: {"__type": "custom", "__value": {
            "cue": {"__type": "int", "__value": self.cue}}}}}

    def result(self, name, obj):
        self.bench[f"files/bench/{name}.json"] = json.dumps(obj)

    def phases_result(self):
        names = ["waiting"] + [fr.PHASES[c - 1] for c in self.cues]
        self.result("fault_render_phases", {"info": {"longMessage": "x" * 146 + " END"}, "phases": {
            n: {"seconds": 15.0, "dsFramesOpened": 60, "dsFramesPerSecond": 4.0, "faults": ["Bench"],
                "faultLineHtmlChars": 90} for n in names}})

    def on_message(self, session, msg):
        kind = msg.get("type")
        with self.lock:
            if kind in ("INIT_OP_MODE", "START_OP_MODE", "SAVE_CONFIG", "STOP_OP_MODE"):
                params = json.loads(self.bench.get("files/bench/params.json") or "{}")
                self.messages.append((kind, msg.get("opModeName") or self.status["activeOpMode"], params))
            if kind == "SAVE_CONFIG":
                value = msg["configDiff"]["__value"][fr.OPMODE]["__value"]["cue"]["__value"]
                if not self.ignore_cue:
                    self.cue = value
                    self.cues.append(value)
                    if self.status["activeOpMode"] == fr.OPMODE:
                        self.phases_result()
                    if self.crash_phase is not None and fr.PHASES[value - 1] == self.crash_phase:
                        self.status["errorMessage"] = "java.lang.IllegalStateException: boom in " + self.crash_phase
                        self.set_active(rb.IDLE_OPMODE, "STOPPED")
                return
            if kind == "GET_CONFIG":
                session.send({"type": "RECEIVE_CONFIG", "configRoot": self.config_root()})
                return
            if kind == "INIT_OP_MODE" and msg["opModeName"] == "Bench: Limelight Fault":
                self.result("limelight_fault_absent", {"firstFaultMs": 420.0})
            if kind == "INIT_OP_MODE" and msg["opModeName"] == fr.OPMODE:
                self.status["errorMessage"] = ""
                params = json.loads(self.bench.get("files/bench/params.json") or "{}")
                if params.get("mode") == "firstFrame":
                    self.result("fault_render_firstFrame", {"dsFramesOpened": 1})
                else:
                    self.cue, self.cues = 0, []
                    self.phases_result()
        super().on_message(session, msg)


def station_run(argv, hub, answers=(), tty=True, interrupt_at=None):
    OUT.mkdir(parents=True, exist_ok=True)
    out = pathlib.Path(tempfile.mkdtemp(prefix="run-", dir=OUT)) / "run"
    handlers = signal.getsignal(signal.SIGINT), signal.getsignal(signal.SIGHUP)
    stdin = FakeStdin(answers, hub, tty, interrupt_at)
    saved, sys.stdin = sys.stdin, stdin
    code = None
    try:
        with hub.installed(rb), contextlib.redirect_stdout(io.StringIO()):
            try:
                code = rb.Runner(rb.parse_args(["--out", str(out)] + list(argv))).run()
            except KeyboardInterrupt:
                code = "interrupted"
    finally:
        sys.stdin = saved
        signal.signal(signal.SIGINT, handlers[0])
        signal.signal(signal.SIGHUP, handlers[1])
    return code, out, stdin


def prompts(out):
    """The question keys the run asked, in order, from run.log."""
    keys = []
    for line in (out / "run.log").read_text().splitlines():
        for key, question in fr.QUESTIONS.items():
            if f">>> {question}" in line:
                keys.append(key)
    return keys


def check_line(summary, key):
    return next((l for l in summary.splitlines() if l.startswith("- ") and fr.CHECK_NAMES[key] in l), "")


def restored(hub):
    return hub.active["name"] == "betta_decode" and not [n for n in hub.configs if n.startswith("bench_")]


def on_the_fake_hub():
    hub = Station()
    code, out, stdin = station_run(["--fault-render", "--interactive", "--ports-empty"], hub, tty=False)
    expect("--interactive without a terminal: refused before any adb command",
           code == 1 and hub.calls == [] and "needs a terminal" in (out / "summary.md").read_text())

    hub = Station()
    code, out, stdin = station_run(["--fault-render", "--ports-empty"], hub)
    summary = (out / "summary.md").read_text()
    expect("without --interactive: the run stops after the probe, saying so, nothing driven and no question asked",
           code == 1 and "so it needs --interactive" in summary and "or drop --fault-render" in summary
           and not [m for m in hub.messages if m[1] != "Bench: Probe"] and not stdin.reads and restored(hub))

    answers = ["maybe"] + [GOOD_TYPED[k] for k in ASKED]
    hub = Station(ds=True)
    code, out, stdin = station_run(["--fault-render", "--interactive", "--ports-empty", "--fault-render-ds-serial",
                                    DS_SERIAL], hub, answers=answers)
    summary = (out / "summary.md").read_text()
    inits = [(o, p) for k, o, p in hub.messages if k == "INIT_OP_MODE"]
    names = [s["name"] for s in hub.switched]
    expect("the eight questions in order, the one not answered y or n asked again",
           prompts(out) == ["llfault"] + list(ASKED) and not stdin.answers)
    expect("Bench: Limelight Fault first, on bench_llfault naming the Limelight at the bench's address, absent mode, "
           "without its drive creep",
           inits[1][0] == "Bench: Limelight Fault" and inits[1][1].get("mode") == "absent"
           and inits[1][1].get("creep") == 0
           and names[names.index(rb.CONFIG_LLFAULT) + 1] == rb.CONFIG_MAIN
           and rb.LIMELIGHT_CONFIG_IP in (out / "logs" / "bench_llfault.xml").read_text())
    expect("then Bench: Fault Render's phases on bench_main, then its first-frame INIT, never STARTed",
           [(o, p.get("mode")) for o, p in inits[2:]] == [(fr.OPMODE, "phases"), (fr.OPMODE, "firstFrame")]
           and [o for k, o, _ in hub.messages if k == "START_OP_MODE"][-2:] == ["Bench: Limelight Fault", fr.OPMODE])
    expect("the cue went 1 to 5, one phase per question", hub.cues == [1, 2, 3, 4, 5])
    expect("every check passes and the run exits 0", code == 0 and "- FAIL" not in summary)
    results = json.loads((out / "runner_results.json").read_text())["fault_render"]
    expect("a Driver Hub screenshot per question, run 1's as raw/ds_fault.png, the Control Hub never asked for one",
           results["screencaps"]["llfault"] == "raw/ds_fault.png" and (out / "raw" / "ds_fault.png").exists()
           and len(results["screencaps"]) == 7 and not [c for c in hub.calls if "screencap" in c and DS_SERIAL not in c]
           and sum(1 for c in hub.ds_calls if c[:1] == ["pull"]) == 7)
    expect("the run ends on the original config with no bench config left", restored(hub))

    hub = Station(crash_phase="long")
    code, out, stdin = station_run(["--fault-render", "--interactive", "--ports-empty"], hub,
                                   answers=["y", "y", "y", "y"])
    summary = (out / "summary.md").read_text()
    expect("a crash in the long phase: no question after it but the first-frame one, setup and the rest FAIL",
           prompts(out) == ["llfault", "short", "modules", "firstFrame"]
           and check_line(summary, "long").startswith("- FAIL") and "not asked" in check_line(summary, "long")
           and "- FAIL fault render: Bench: Fault Render went through its five phases" in summary
           and check_line(summary, "short").startswith("- PASS") and restored(hub))

    hub = Station(ignore_cue=True)
    code, out, stdin = station_run(["--fault-render", "--interactive", "--ports-empty"], hub, answers=["y"])
    summary = (out / "summary.md").read_text()
    expect("a cue the dashboard doesn't take: no phase question asked, the bench fails saying why",
           prompts(out) == ["llfault"] and "didn't take Bench: Fault Render's cue 1" in summary and restored(hub))

    hub = Station()
    code, out, stdin = station_run(["--fault-render", "--interactive", "--ports-empty"], hub, answers=["y", "y"])
    summary = (out / "summary.md").read_text()
    expect("the terminal closed at a question: the bench fails, never taking it as an answer, config put back",
           "the terminal closed before the answer" in summary and "modules" not in (
               json.loads((out / "runner_results.json").read_text())["fault_render"].get("answers") or {})
           and restored(hub))

    hub = Station()
    code, out, stdin = station_run(["--fault-render", "--interactive", "--ports-empty"], hub, answers=["y", "y"],
                                   interrupt_at=3)
    expect("Ctrl-C at the third question: the run stops and the config is put back",
           code == "interrupted" and len(stdin.reads) == 3 and restored(hub)
           and "STOPPED by KeyboardInterrupt" in (out / "run.log").read_text())

    hub = Station()
    code, out, stdin = station_run(["--fault-render", "--interactive", "--ports-empty", "--fault-render-ds-serial",
                                    DS_SERIAL], hub)
    expect("a Driver Hub serial adb doesn't list: refused before any OpMode, nothing asked",
           "adb doesn't list the Driver Hub" in (out / "summary.md").read_text() and not stdin.reads
           and not [m for m in hub.messages if m[1] != "Bench: Probe"] and restored(hub))


def declaration():
    expect("declared: flag-only, needs a person with --interactive, drives no port",
           not BENCH.default and BENCH.person is not None and not BENCH.drives and BENCH.statement is None
           and BENCH.person_flags == ("--fault-render", "--interactive")
           and BENCH.opmodes == ("Bench: Limelight Fault", fr.OPMODE) and BENCH.configs == (rb.CONFIG_LLFAULT,))
    command = next(e for e in rb.needs_a_person() if e[0] == "fault_render")[2]
    args = rb.parse_args(command)
    expect("its Needs a person command parses and carries --interactive", args.fault_render and args.interactive)


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
