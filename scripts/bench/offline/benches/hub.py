"""Bench: Hub offline, for --hub-planner-baseline: reading an earlier run's planner phase, the comparison lines on
synthetic phases (each check of them fails when weakened), and Bench: Hub end to end on fake_hub.py with and without the
flag. No hub, no adb, no network. Run with: python3 -I scripts/bench/offline/benches/hub.py
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
OFFLINE = HERE.parent
WORKTREE = OFFLINE.parents[2]
OUT = OFFLINE / "out" / "hub"
RUNNER = WORKTREE / "scripts/bench/run_bench.py"
POLL = re.compile(r"^adb devices$| shell pidof | shell logcat -d ")

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


rb = load("run_bench", RUNNER)
fake_hub = load("fake_hub", OFFLINE / "fake_hub.py")


def plans(p50, p95, mx, n, dropped=0):
    return {"plans": n, "planMs": {"count": n, "mean": p50, "min": 1.0, "p50": p50, "p90": p95, "p95": p95, "p99": mx,
                                   "max": mx}, "plansWithDroppedBalls": dropped}


def planner(scale=1.0, five=True):
    by_count = {"n=3": plans(141 * scale, 333 * scale, 465 * scale, 60), "n=4": plans(558 * scale, 1888 * scale,
                                                                                    2434 * scale, 48, 2)}
    if five:
        by_count["n=5"] = plans(1934 * scale, 13067 * scale, 14769 * scale, 40, 5)
    return {"sliceMs": 20.0, "byBallCount": by_count,
            "byCase": {"4 balls on both sides of the rail": plans(1452 * scale, 1728 * scale, 1728 * scale, 5)},
            "coldFirstPlan": {"balls": 4, "planMs": 2046.0 * scale}}


def folder(hub_json):
    OUT.mkdir(parents=True, exist_ok=True)
    f = pathlib.Path(tempfile.mkdtemp(prefix="earlier-", dir=OUT))
    (f / "raw").mkdir()
    (f / "raw" / "hub.json").write_text(hub_json if isinstance(hub_json, str) else json.dumps(hub_json))
    return f


def refused(value):
    try:
        rb.planner_baseline_arg(str(value))
    except argparse.ArgumentTypeError as e:
        return str(e)
    return None


def reading():
    good = folder({"phases": {"planner": planner()}})
    base = rb.planner_baseline_arg(str(good))
    expect("a results folder: its raw/hub.json's planner phase, labelled as given",
           base["label"] == str(good) and base["planner"] == planner())
    expect("the hub.json itself works too", rb.planner_baseline_arg(str(good / "raw" / "hub.json"))["planner"]
           == planner())
    expect("parse_args reads it before anything runs",
           rb.parse_args(["--hub", "--hub-planner-baseline", str(good)]).hub_planner_baseline["planner"] == planner())
    expect("without the flag there is no baseline", rb.parse_args(["--hub"]).hub_planner_baseline is None)
    expect("a folder with no raw/hub.json is refused", "has no Bench: Hub planner phase" in (refused(OUT / "nope")
                                                                                             or ""))
    expect("a hub.json that isn't JSON is refused", "has no Bench: Hub planner phase" in (refused(folder("{not json"))
                                                                                          or ""))
    expect("a hub.json from a run that stopped before the planner phase is refused",
           "has no Bench: Hub planner phase" in (refused(folder({"phases": {}})) or ""))
    expect("a planner phase with no plan times is refused", "has no plan times" in
           (refused(folder({"phases": {"planner": {"sliceMs": 20.0}}})) or ""))
    with contextlib.redirect_stderr(io.StringIO()) as err:
        try:
            rb.parse_args(["--hub", "--hub-planner-baseline", str(OUT / "nope")])
            stopped = False
        except SystemExit:
            stopped = True
    expect("parse_args stops on an unreadable one, naming it", stopped and str(OUT / "nope") in err.getvalue())


def lines_cases(lines_fn):
    """[(label, ok)] for planner_baseline_lines on synthetic phases: now 1.25x then."""
    out = []
    text = "\n".join(lines_fn(planner(1.25), planner(), "bench-results/earlier"))
    out.append(("the header names the earlier run", "against bench-results/earlier" in text))
    out.append(("per ball count: then and now p50 / p95 / max, and now as a share of then",
                "  - random, n=4: then 558.00 / 1888.00 / 2434.00, now 697.50 / 2360.00 / 3042.50 (1.25x)" in text))
    out.append(("per ball count: plans that dropped a ball, then and now",
                "(1.25x); then 2 of 48 dropped a ball; now 2 of 48 dropped a ball" in text))
    out.append(("per hand-built layout too", "  - 4 balls on both sides of the rail: then 1452.00 / 1728.00 / 1728.00, "
                                             "now 1815.00 / 2160.00 / 2160.00 (1.25x)" in text))
    out.append(("the first plan in the app process", "first plan in the app process: then 2046 ms (4 balls), now 2558 "
                                                     "ms (4 balls)" in text))
    text = "\n".join(lines_fn(planner(five=False), planner(), "x"))
    out.append(("a ball count only one run has: n/a for the other, no share",
                "  - random, n=5: then 1934.00 / 13067.00 / 14769.00, now n/a; then 5 of 40 dropped a ball" in text))
    out.append(("ball counts in order", text.index("n=3") < text.index("n=4") < text.index("n=5")))
    return out


MUTATIONS = [
    ("the share is upside down", "share = f\" ({y['p50'] / x['p50']:.2f}x)\"", "share = f\" ({x['p50'] / y['p50']:.2f}x)\""),
    ("the earlier run's row is lost", "row(title.format(name), then.get(name), now_group.get(name))",
     "row(title.format(name), None, now_group.get(name))"),
    ("a ball count only the earlier run has is left out", "names = sorted(set(then) | set(now_group))",
     "names = sorted(set(now_group))"),
    ("dropped balls aren't given", "drops = \"\".join(", "drops = \"\" if True else \"\".join("),
]


def mutated(old, new):
    text = RUNNER.read_text()
    if text.count(old) != 1:
        return None
    module = type(sys)("run_bench_mutant")
    module.__file__ = str(RUNNER)
    exec(compile(text.replace(old, new), str(RUNNER), "exec"), module.__dict__)
    return module


def comparison():
    for label, ok in lines_cases(rb.planner_baseline_lines):
        expect(f"lines: {label}", ok)
    for label, old, new in MUTATIONS:
        mutant = mutated(old, new)
        if mutant is None:
            expect(f"weakened: {label}: the text to weaken is in run_bench.py once", False)
            continue
        try:
            caught = [case for case, ok in lines_cases(mutant.planner_baseline_lines) if not ok]
        except Exception as e:
            caught = [f"{type(e).__name__}: {e}"]
        expect(f"weakened: {label}: a lines case fails ({caught[:1]})", bool(caught))


class HubBenchHub(fake_hub.Hub):
    """fake_hub's Control Hub whose Bench: Hub writes a hub.json with a planner phase when INITed."""

    def __init__(self, phase, **kw):
        super().__init__(**kw)
        self.phase = phase

    def on_message(self, session, msg):
        with self.lock:
            if msg.get("type") == "INIT_OP_MODE" and msg.get("opModeName") == "Bench: Hub":
                self.bench["files/bench/hub.json"] = json.dumps({"info": {}, "phases": {"planner": self.phase}})
        super().on_message(session, msg)


def hub_run(argv):
    hub = HubBenchHub(planner(1.25))
    OUT.mkdir(parents=True, exist_ok=True)
    out = pathlib.Path(tempfile.mkdtemp(prefix="run-", dir=OUT)) / "run"
    handlers = signal.getsignal(signal.SIGINT), signal.getsignal(signal.SIGHUP)
    stdin, sys.stdin = sys.stdin, io.StringIO("")
    try:
        with hub.installed(rb), contextlib.redirect_stdout(io.StringIO()):
            code = rb.Runner(rb.parse_args(["--out", str(out), "--ports-empty", "--hub"] + list(argv))).run()
    finally:
        sys.stdin = stdin
        signal.signal(signal.SIGINT, handlers[0])
        signal.signal(signal.SIGHUP, handlers[1])
    calls = [c.replace(str(out), "<out>") for c in hub.calls if not POLL.search(c)]
    return hub, calls, (out / "summary.md").read_text()


def on_the_fake_hub():
    earlier = folder({"phases": {"planner": planner()}})
    hub, calls, text = hub_run(["--hub-planner-baseline", str(earlier)])
    expect("--hub --hub-planner-baseline: Bench: Hub's section gives this run's plan times against the earlier run's",
           f"against {earlier}" in text and "  - random, n=4: then 558.00 / 1888.00 / 2434.00, now 697.50 / 2360.00 / "
                                            "3042.50 (1.25x)" in text)
    expect("--hub --hub-planner-baseline: the config is put back", hub.active["name"] == "betta_decode"
           and not [n for n in hub.configs if n.startswith("bench_")])
    hub, alone, text = hub_run([])
    expect("--hub alone: no comparison lines", "Plan times from request to done against" not in text)
    expect("the flag sends the hub nothing more: the same adb commands as --hub alone, polls aside", calls == alone)


def main():
    for section in (reading, comparison, on_the_fake_hub):
        try:
            section()
        except Exception as e:
            expect(f"{section.__name__} ran to its end ({type(e).__name__}: {e})", False)
            traceback.print_exc()
    print(f"{passed} passed, {len(failed)} failed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
