"""The benches in scripts/bench/benches/ (its README.md): every declaration loads and keeps the safety rules, order.txt
is the run order, a bench's flag, hardware needs, checks and summary reach run_bench.py and session.py, the in-place
benches' flags and order are as before; then each bench's own offline file in offline/benches/. No hub, no adb, no
network. Run with: python3 -I scripts/bench/offline/test_benches.py
"""
import contextlib
import importlib.util
import io
import json
import os
import pathlib
import re
import shutil
import signal
import subprocess
import sys
import tempfile
import textwrap
import traceback

sys.dont_write_bytecode = True
HERE = pathlib.Path(__file__).resolve().parent
WORKTREE = HERE.parents[2]
OUT = HERE / "out" / "benches"
OWN_TESTS = HERE / "benches"
IN_PLACE_ORDER = ("baseline", "hub", "loop", "failure", "threads", "gamepad", "edge", "write_recovery", "handover",
                  "photon", "hardware_view", "turret", "webcam", "webcam_failure", "limelight", "llfault", "sloth",
                  "clock", "pinpoint", "octoquad", "octoquad_absent", "expansion", "absent", "planner_stop",
                  "overlay_nan", "config_by_name", "turret_zero", "freeze_dump", "dashboard_off")

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
fake_hub = load("fake_hub", HERE / "fake_hub.py")


def stub(name, default=False, needs=None, person=None, drives=None, powers=None, configs=(), opmodes=(), raises=None,
         flag=None, options="()"):
    """A bench module's source: one Bench: Probe run, a check and a summary line, all from its record."""
    fields = [f"name={name!r}", f"title='Bench: Stub {name}'", f"flag={flag or '--' + name.replace('_', '-')!r}",
              f"help='stub bench {name}'", "run=run", "checks=checks", "summary=summary", f"default={default}",
              f"configs={tuple(configs)!r}", f"opmodes={tuple(opmodes)!r}", f"options={options}"]
    if needs:
        fields.append(f"needs=lambda r: None if r.webcam else {needs!r}")
    if person:
        fields.append(f"person=rb.Person({person!r}, 'a stub table')")
    if drives:
        fields.append(f"drives=rb.Ports({drives})")
    if powers:
        fields.append(f"powers={powers!r}")
    body = f"""
        def run(r, record):
            record["ran"] = True
            {"raise rb.BenchError(" + repr(raises) + ")" if raises else "pass"}
            o = r.run_opmode("Bench: Probe", "{name}_probe", timeout=60, result_files=["probe"])
            record["crashed"] = o["crashed"]


        def checks(record):
            return [("{name}: Bench: Probe ran", record.get("crashed") is False, f"crashed {{record.get('crashed')}}",
                     "the stub's OpMode crashed")]


        def summary(record):
            return ["## Stub {name}\\n", f"- record {{sorted(record)}}\\n"]


        BENCH = rb.Bench({", ".join(fields)})
        """
    return textwrap.dedent(body)


def folder(*benches, order=None, extra=()):
    """A benches folder: (name, source) pairs, listed in order.txt in the order given unless order says otherwise."""
    OUT.mkdir(parents=True, exist_ok=True)
    f = pathlib.Path(tempfile.mkdtemp(prefix="layout-", dir=OUT))
    for name, source in list(benches) + list(extra):
        (f / f"{name}.py").write_text(source)
    (f / "order.txt").write_text("".join(n + "\n" for n in (order if order is not None else [b[0] for b in benches])))
    return f


def layout_error(f):
    try:
        rb.load_benches(f)
    except rb.BenchLayoutError as e:
        return str(e)
    return None


@contextlib.contextmanager
def using(f):
    rb.use_benches(rb.load_benches(f))
    try:
        yield
    finally:
        rb.use_benches(rb.load_benches())


def help_text(argv=("--help",)):
    out = io.StringIO()
    env = os.environ.get("COLUMNS")
    os.environ["COLUMNS"] = "120"
    try:
        with contextlib.redirect_stdout(out):
            rb.parse_args(list(argv))
    except SystemExit:
        pass
    finally:
        if env is None:
            os.environ.pop("COLUMNS", None)
        else:
            os.environ["COLUMNS"] = env
    return out.getvalue()


def options_block(text):
    return text.split("\noptions:\n", 1)[1].split("\n\n", 1)[0].rstrip("\n")


ALL_ENTRY = re.compile(r"\n  --all .*?(?=\n  -)", re.S)


def without_all(block):
    """The options block without --all's entry, whose help names the folder's flag-only benches."""
    return ALL_ENTRY.sub("", block)


def all_help(text):
    return " ".join(ALL_ENTRY.search(options_block(text)).group(0).split())


def selection(argv, everything=False):
    r = object.__new__(rb.Runner)
    r.args = rb.parse_args(list(argv))
    r.summary, r.log = [], (lambda text: None)
    r.expansion, r.webcam, r.limelight = (2, "UC1", {"ipAddress": "172.29.0.1"}) if everything else (None,) * 3
    try:
        return r.selection(), r.summary
    except rb.BenchError as e:
        return f"error: {e}", r.summary


def the_real_folder():
    names = [l.strip() for l in (rb.BENCH_FOLDER / rb.BENCH_ORDER_FILE).read_text().splitlines() if l.strip()]
    expect("the real benches folder loads, in order.txt's order", [b.name for b in rb.MODULE_BENCHES] == names)
    expect("the in-place benches keep their names and order (BENCHES)", rb.BENCHES == IN_PLACE_ORDER)
    expect("Bench: Turret Zero, Freeze Dump and Dashboard Off are still the last three in the run order",
           rb.bench_order()[-3:] == ("turret_zero", "freeze_dump", "dashboard_off"))
    expect("every bench in the folder runs just before Bench: Turret Zero, in order.txt's order",
           rb.bench_order() == IN_PLACE_ORDER[:-3] + tuple(names) + IN_PLACE_ORDER[-3:])
    expect("every bench's flag is in --help", all(b.flag in help_text() for b in rb.MODULE_BENCHES))
    expect("--all's help names every bench in the folder that isn't in the default suite",
           all(b.title in all_help(help_text()) for b in rb.MODULE_BENCHES if not b.default))
    expect("the session reads each bench that waits for a person from its declaration, after the hand-kept ones",
           [p[0] for p in load("session", WORKTREE / "scripts/bench/session.py").PROMPTED][-len(rb.needs_a_person()):]
           == [p[0] for p in rb.needs_a_person()] or not rb.needs_a_person())
    for b in rb.MODULE_BENCHES:
        test = OWN_TESTS / f"{b.name}.py"
        expect(f"{b.name}: has its offline file {test.relative_to(WORKTREE)}", test.exists())
        if b.default:
            expect(f"{b.name}: in the default suite, it drives at most what --ports-empty covers",
                   b.statement in (None, "--ports-empty"))


def missing_folder():
    OUT.mkdir(parents=True, exist_ok=True)
    gone = pathlib.Path(tempfile.mkdtemp(prefix="gone-", dir=OUT)) / "benches"
    expect("a missing benches folder stops the load, naming order.txt",
           "order.txt is missing" in (layout_error(gone) or ""))
    f = folder(("a_bench", stub("a_bench")))
    (f / "order.txt").unlink()
    expect("a folder without order.txt stops the load", "order.txt is missing" in (layout_error(f) or ""))
    root = pathlib.Path(tempfile.mkdtemp(prefix="nobenches-", dir=OUT))
    shutil.copy(WORKTREE / "scripts/bench/run_bench.py", root / "run_bench.py")
    r = subprocess.run([sys.executable, "-I", str(root / "run_bench.py"), "--help"], capture_output=True, text=True,
                       timeout=60)
    expect("run_bench.py without its benches folder stops at import with BenchLayoutError, before --help",
           r.returncode != 0 and "BenchLayoutError" in r.stderr and "order.txt is missing" in r.stderr
           and "--config-check" not in r.stdout)


def rules():
    f = folder(("a_bench", stub("a_bench")), extra=[("not_listed", stub("not_listed"))])
    expect("a .py file order.txt doesn't list stops the load", "not_listed.py isn't listed" in (layout_error(f) or ""))
    f = folder(("a_bench", stub("a_bench")), order=["a_bench", "a_bench"])
    expect("a name listed twice stops the load", "listed twice" in (layout_error(f) or ""))
    f = folder(("a_bench", stub("a_bench")), order=["a_bench", "ghost"])
    expect("a listed name without a file stops the load", "ghost.py doesn't exist" in (layout_error(f) or ""))
    f = folder(("a_bench", stub("b_bench")))
    expect("a declaration named unlike its file stops the load", "not its file's 'a_bench'" in (layout_error(f) or ""))
    f = folder(("a_bench", stub("a_bench", flag="--abench")))
    expect("a flag that isn't --<name> stops the load", "not --a-bench" in (layout_error(f) or ""))
    f = folder(("hub", stub("hub")))
    expect("a name an in-place bench has stops the load", "already a bench" in (layout_error(f) or ""))
    f = folder(("a_bench", "x = 1\n"))
    expect("a file without BENCH stops the load", "declares no BENCH" in (layout_error(f) or ""))
    f = folder(("a_bench", stub("a_bench", drives="ch_motors=(0,)")))
    expect("ports driven with no word on what it commands stop the load", "powers" in (layout_error(f) or ""))
    f = folder(("a_bench", stub("a_bench", drives="ch_servos=(5,), eh_servos=(9,)", powers="x")))
    expect("ports no statement covers stop the load", "no port statement covers" in (layout_error(f) or ""))
    f = folder(("a_bench", stub("a_bench", default=True, person="a stub that asks")))
    expect("a default-suite bench that waits for a person stops the load", "make it flag-only" in (layout_error(f) or ""))
    f = folder(("a_bench", stub("a_bench", default=True, drives="ch_servos=(2,)", powers="x")))
    expect("a default-suite bench that drives more than --ports-empty covers stops the load",
           "default suite but needs --s3-ports" in (layout_error(f) or ""))
    f = folder(("a_bench", stub("a_bench", drives="eh_motors=(0,)", powers="x")))
    expect("a bench whose statement is typed at the terminal but that declares no person stops the load",
           "--s3-ports is typed at the terminal, so it needs a person" in (layout_error(f) or ""))
    f = folder(("a_bench", stub("a_bench", configs=("",))))
    expect("an empty config name stops the load", "isn't a config's name" in (layout_error(f) or ""))
    f = folder(("a_bench", stub("a_bench", configs=("no_such_config",))),
               ("b_bench", stub("b_bench", default=True, configs=("no_such_config",))))
    expect("a robot config this checkout doesn't ship doesn't stop the load (a renamed config stops no other run)",
           layout_error(f) is None)
    with using(f):
        expect("--help still works with that config gone", "--a-bench" in help_text())
        chosen, _ = selection(["--a-bench"])
        expect("a run that names the bench stops, naming the config and the bench's file",
               chosen == "error: Bench: Stub a_bench activates no_such_config, which this checkout's res/xml doesn't "
                         "ship: put its name right in a_bench.py, or drop --a-bench")
        chosen, summary = selection([])
        expect("a default run skips a default bench whose config isn't shipped, saying so, and runs the rest",
               "b_bench" not in chosen and "hub" in chosen
               and "_Bench: Stub b_bench skipped: res/xml doesn't ship no_such_config._\n" in summary)
    f = folder(("a_bench", stub("a_bench", configs=("bench_a_bench", "betta_decode"),
                                drives="ch_motors=(0, 1, 2, 3), ch_servos=(0, 1)", powers="0.1 on motor ports 0-3")))
    expect("a bench within --ports-empty's ports, writing a bench_* config and using a shipped one, loads",
           layout_error(f) is None and rb.load_benches(f)[0].statement == "--ports-empty")
    expect("the ports read as the session's statement does",
           rb.Ports(ch_motors=(0, 1, 2, 3), ch_servos=(0, 1), eh_motors=(0, 1, 2, 3), eh_servos=(0, 1)).describe()
           == "Control Hub motor ports 0-3 and servo ports 0-1 and the Expansion Hub's motor ports 0-3 and servo "
              "ports 0-1")


def wiring():
    f = folder(("zeta", stub("zeta", opmodes=("Bench: Zeta",), configs=("bench_zeta",))), ("alpha", stub("alpha")),
               ("dflt", stub("dflt", default=True)), ("cam", stub("cam", default=True, needs="no webcam plugged in")),
               ("asks", stub("asks", person="a stub that asks", options="(('--asks-times', {'type': int}),)")))
    before = help_text()
    with using(f):
        order = rb.bench_order()
        expect("order.txt's order, not the folder's: zeta before alpha, both just before Bench: Turret Zero",
               order[order.index("config_by_name") + 1:order.index("turret_zero")] == ("zeta", "alpha", "dflt", "cam",
                                                                                         "asks"))
        expect("a bench's OpModes are required at connect, and its bench_* configs are deleted at the end",
               "Bench: Zeta" in rb.BENCH_OPMODES and "bench_zeta" in rb.BENCH_CONFIGS
               and rb.BENCH_CONFIGS[:len(rb.IN_PLACE_BENCH_CONFIGS)] == rb.IN_PLACE_BENCH_CONFIGS)
        after = help_text()
        expect("--help: the existing options are listed exactly as before; the benches' flags and options come after "
               "them in their own group", without_all(options_block(after)) == without_all(options_block(before))
               and "benches in scripts/bench/benches/" in after and "--asks-times" in after
               and after.index("--zeta") < after.index("--alpha"))
        expect("a bench's own option parses", rb.parse_args(["--asks", "--asks-times", "3"]).asks_times == 3)
        chosen, _ = selection([])
        expect("a run with no bench flags runs the default-suite benches, not the flag-only ones",
               "dflt" in chosen and not {"zeta", "alpha", "asks"} & chosen)
        chosen, _ = selection(["--all"], everything=True)
        expect("--all leaves the flag-only benches out too", isinstance(chosen, set) and {"dflt", "cam"} <= chosen
               and not {"zeta", "alpha", "asks"} & chosen)
        expect("--all's help names the flag-only benches it leaves out, and not the default-suite ones",
               all(f"Bench: Stub {n}" in all_help(after) for n in ("zeta", "alpha", "asks"))
               and not any(f"Bench: Stub {n}" in all_help(after) for n in ("dflt", "cam")))
        chosen, summary = selection([])
        expect("missing hardware in a default run: skipped, with the bench's own words in summary.md",
               "cam" not in chosen and "_Bench: Stub cam skipped: no webcam plugged in._\n" in summary)
        chosen, _ = selection(["--cam"])
        expect("missing hardware in a run that names the bench: the run stops, saying what to plug in or which flag "
               "to drop", chosen == "error: No webcam plugged in, or drop --cam")
        chosen, _ = selection(["--alpha", "--hub"])
        expect("a bench flag runs only what it names", chosen == {"alpha", "hub"})
        expect("--probe-only refuses a bench's flag", refused(["--probe-only", "--alpha"]))
        expect("the session's Needs a person entry comes from the declaration",
               rb.needs_a_person() == (("asks", "a stub that asks", ["--asks"], "a stub table"),)
               and rb.person_flags()[-1:] == ("--asks",))
    session = session_copy(f)
    expect("session.py lists a bench that waits for a person under Needs a person, and refuses a stage with its flag",
           session.PROMPTED[-1][0] == "asks" and "--asks" in session.PROMPTED_FLAGS
           and session_refuses(session, ["--asks"]))


def refused(argv):
    try:
        with contextlib.redirect_stderr(io.StringIO()):
            rb.parse_args(argv)
    except SystemExit:
        return True
    return False


def session_copy(benches):
    """session.py and run_bench.py loaded from a copy beside the given benches folder."""
    OUT.mkdir(parents=True, exist_ok=True)
    root = pathlib.Path(tempfile.mkdtemp(prefix="session-", dir=OUT))
    bench = root / "scripts" / "bench"
    bench.mkdir(parents=True)
    for name in ("run_bench.py", "session.py", "bare_hub.sh"):
        shutil.copy(WORKTREE / "scripts/bench" / name, bench / name)
    shutil.copytree(benches, bench / "benches")
    return load("session_copy", bench / "session.py")


def session_refuses(session, flags):
    stage = session.Stage("default", "Default suite", flags, "")
    try:
        session.Session(session.parse_args([])).run_stage(stage)
    except RuntimeError:
        return True
    return False


def on_the_fake_hub():
    """A flag-only bench end to end on fake_hub.py: it runs where order.txt puts it, its checks and summary reach
    summary.md, its record reaches runner_results.json; a bench that raises fails as a bench and the hub is put back."""
    f = folder(("alpha", stub("alpha")), ("broken", stub("broken", raises="the stub stopped")))
    with using(f):
        hub = fake_hub.Hub()
        OUT.mkdir(parents=True, exist_ok=True)
        out = pathlib.Path(tempfile.mkdtemp(prefix="hub-", dir=OUT)) / "run"
        handlers = signal.getsignal(signal.SIGINT), signal.getsignal(signal.SIGHUP)
        stdin, sys.stdin = sys.stdin, io.StringIO("")
        try:
            with hub.installed(rb), contextlib.redirect_stdout(io.StringIO()):
                code = rb.Runner(rb.parse_args(["--out", str(out), "--ports-empty", "--baseline", "--alpha",
                                                "--broken"])).run()
        finally:
            sys.stdin = stdin
            signal.signal(signal.SIGINT, handlers[0])
            signal.signal(signal.SIGHUP, handlers[1])
    log = (out / "run.log").read_text()
    entered = [l.split("=== ", 1)[1].split(" ===")[0] for l in log.splitlines() if "] === " in l]
    summary = (out / "summary.md").read_text()
    results = json.loads((out / "runner_results.json").read_text())
    expect("fake hub: the benches ran in run order (baseline, then order.txt's)", entered == ["baseline", "alpha",
                                                                                              "broken"])
    expect("fake hub: the bench's check is in summary.md's checks, from its record",
           "- PASS alpha: Bench: Probe ran: crashed False" in summary)
    expect("fake hub: the bench's summary lines are in summary.md", "## Stub alpha\n" in summary)
    expect("fake hub: the bench's record is in runner_results.json under its name",
           results.get("alpha") == {"ran": True, "crashed": False})
    expect("fake hub: a bench that raises fails as a bench, and its checks don't run on a half record",
           "- FAIL broken bench completed: the stub stopped" in summary and "broken: Bench: Probe ran" not in summary)
    expect("fake hub: the run exits 1 for the failed bench, with the original config back and no bench config left",
           code == 1 and hub.active["name"] == "betta_decode"
           and not [n for n in hub.configs if n.startswith("bench_")])


class Terminal:
    def __init__(self, answers, hub, tty=True):
        self.answers, self.hub, self.tty, self.reads = list(answers), hub, tty, []

    def isatty(self):
        return self.tty

    def readline(self):
        self.reads.append(len(self.hub.calls))
        return self.answers.pop(0) + "\n" if self.answers else ""


def gated_run(argv, answers=(), tty=True):
    f = folder(("gated", stub("gated", drives="eh_motors=(0, 1, 2, 3), ch_servos=(2,)", powers="0.1 on them",
                              person="a stub on both hubs")))
    hub = fake_hub.Hub()
    OUT.mkdir(parents=True, exist_ok=True)
    out = pathlib.Path(tempfile.mkdtemp(prefix="gate-", dir=OUT)) / "run"
    handlers = signal.getsignal(signal.SIGINT), signal.getsignal(signal.SIGHUP)
    terminal = Terminal(answers, hub, tty)
    stdin, sys.stdin = sys.stdin, terminal
    try:
        with using(f), hub.installed(rb), contextlib.redirect_stdout(io.StringIO()):
            code = rb.Runner(rb.parse_args(["--out", str(out)] + list(argv))).run()
    finally:
        sys.stdin = stdin
        signal.signal(signal.SIGINT, handlers[0])
        signal.signal(signal.SIGHUP, handlers[1])
    return code, hub, terminal, (out / "summary.md").read_text()


def typed_statement():
    code, hub, terminal, summary = gated_run(["--gated", "--ports-empty"])
    expect("a bench needing --s3-ports, run without it: stops before any adb command, naming the ports and the flag",
           code == 1 and hub.calls == [] and "rerun with --s3-ports" in summary
           and "the Expansion Hub's motor ports 0-3" in summary and not terminal.reads)
    code, hub, terminal, summary = gated_run(["--gated", "--s3-ports"], tty=False)
    expect("--s3-ports without a terminal: stops before any adb command", code == 1 and hub.calls == []
           and "needs a terminal" in summary)
    for answers, label in ((["yes"],  "anything but empty typed"), ([], "the terminal closed")):
        code, hub, terminal, summary = gated_run(["--gated", "--s3-ports"], answers)
        expect(f"--s3-ports, {label}: stops before any adb command", code == 1 and hub.calls == []
               and "nothing was changed on the hub" in summary)
    code, hub, terminal, summary = gated_run(["--gated", "--s3-ports"], ["empty"])
    expect("--s3-ports with empty typed: asked once, before the first adb command, then the bench runs",
           code == 0 and terminal.reads == [0] and "- PASS gated: Bench: Probe ran" in summary)
    code, hub, terminal, summary = gated_run(["--baseline", "--s3-ports"], ["empty"])
    expect("--s3-ports is typed even when no bench in the run needs it, and stands for --ports-empty",
           code == 0 and terminal.reads == [0] and rb.parse_args(["--s3-ports"]).ports_empty)

    class Untyped(rb.Runner):
        def port_statements(self):
            pass

    f = folder(("gated", stub("gated", drives="eh_motors=(0,)", powers="x", person="a stub")))
    with using(f):
        r = object.__new__(Untyped)
        r.typed_statements = set()
        try:
            r.run_module_bench(rb.module_bench("gated"))
            refused = False
        except rb.BenchError as e:
            refused = "runs only once --s3-ports" in str(e)
    expect("a bench needing a typed statement refuses to run when the statement wasn't typed, whatever skipped it",
           refused)


def own_tests():
    """Every file in offline/benches/: a new bench's, and one an extension of a bench in run_bench.py adds."""
    for test in sorted(OWN_TESTS.glob("*.py")):
        r = subprocess.run([sys.executable, "-I", str(test)], cwd=WORKTREE, capture_output=True, text=True)
        last = (r.stdout.strip().splitlines() or [""])[-1]
        ok = r.returncode == 0 and re.fullmatch(r"\d+ passed, 0 failed", last) is not None
        expect(f"{test.stem}: its offline file passes ({last or r.stderr.strip()[-300:]})", ok)
        if not ok:
            print(r.stdout[-3000:], r.stderr[-3000:])


def main():
    for section in (the_real_folder, missing_folder, rules, wiring, on_the_fake_hub, typed_statement, own_tests):
        try:
            section()
        except Exception as e:
            expect(f"{section.__name__} ran to its end ({type(e).__name__}: {e})", False)
            traceback.print_exc()
    print(f"{passed} passed, {len(failed)} failed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
