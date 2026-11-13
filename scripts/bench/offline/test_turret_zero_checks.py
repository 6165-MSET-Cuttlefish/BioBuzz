"""Offline checks for Bench: Turret Zero's verdicts in run_bench.py: a good synthetic run passes every check, and each
broken one (a wrong sign, a stationary shaft, a missing phase, a level too high, a silent lost count...) fails the
check it should. Run with: python3 -I <this file>"""
import copy
import importlib.util
import pathlib
import sys

sys.dont_write_bytecode = True
HERE = pathlib.Path(__file__).resolve().parent


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


def judge(tz):
    checks, lines = rb.turret_zero_checks(tz)
    return {name[len("turret zero: "):]: ok for name, ok, _, _ in checks}, "\n".join(lines)


def state(got, prefix):
    hits = [ok for name, ok in got.items() if name.startswith(prefix)]
    return hits[0] if len(hits) == 1 else ("missing" if not hits else "several")


def fails(tz, prefix):
    return state(judge(tz)[0], prefix) is False


def with_run(tz, name, result=None, **outcome):
    t = copy.deepcopy(tz)
    if result is None and not outcome:
        del t["runs"][name]
    else:
        if result is not None:
            t["runs"][name]["result"] = result
        t["runs"][name].update(outcome)
    return t


GOOD = fx.good()
got, text = judge(GOOD)
expect("good: 16 checks, every one passes", len(got) == 16 and all(got.values()))
expect("good: brownout and V2 are reported not run", "brownout: not run (no lab supply)" in text and "V2: not run" in text)
got, text = judge(fx.good(brownout="ran", v2=True))
expect("good with brownout and V2: 22 checks, every one passes", len(got) == 22 and all(got.values()))

t = copy.deepcopy(GOOD)
t["answers"]["split"] = "pin 3 open"
expect("wiring: a cable not typed ok fails", fails(t, "the REV-31-1815 and the split cable"))
t = copy.deepcopy(GOOD)
t["answers"]["cable1"] = "pin 2 open"
expect("wiring: the REV-31-1815 not typed ok fails", fails(t, "the REV-31-1815 and the split cable"))
for high, supply in (("3.6", "3.3"), ("3.3", "5.0"), ("3.3", "2.5"), ("high", "3.3"), ("-3.6", "3.3"), ("0", "3.3"),
                     ("3.47", "3.3"), ("3.3", "3.47")):
    t = copy.deepcopy(GOOD)
    t["answers"].update(highV=high, supplyV=supply)
    expect(f"level: high {high} V, supply {supply} V fails", fails(t, "the encoder's supply"))
t = copy.deepcopy(GOOD)
t["answers"].update(highV="3.46", supplyV="2.97")
expect("level: high 3.46 V, supply 2.97 V passes (the limits the check names)",
       state(judge(t)[0], "the encoder's supply 2.97-3.46 V and its absolute output's high level above 0 and at most "
                          "3.46 V") is True)

t = with_run(GOOD, "quad", fx.track(fx.to_mark("median", 1000, 5, per_turn=8204)))
expect("quad: 12 counts per turn too many fails", fails(t, "OctoQuad channel 4 counting"))
t = with_run(GOOD, "quad", fx.track(fx.to_mark("median", 1000, 5, end_off=60)))
expect("quad: 60 counts off the mark after 5 turns each way fails", fails(t, "OctoQuad channel 4 counting"))
t = with_run(GOOD, "quad", fx.track(fx.to_mark("median", 1000, 5)[:3]))
expect("quad: no rest back at the mark fails", fails(t, "OctoQuad channel 4 counting"))
t = with_run(GOOD, "split_plugged", fx.track(fx.to_mark("count", 500, 10, per_turn=8185)))
got, _ = judge(t)
expect("split: 7 counts per turn short passes alone", state(got, "hub encoder port 2, OctoQuad branch plugged") is True)
t = with_run(t, "split_unplugged", fx.track(fx.to_mark("count", 500, 10, per_turn=8199)))
expect("split: plugged and unplugged 14 counts per turn apart fails the load check", fails(t, "the hub counts within"))

t = with_run(GOOD, "span", fx.track(rows=fx.span_rows(per_turn=8100)))
expect("span: 8100 counts per turn fails", fails(t, "span"))
t = with_run(GOOD, "span", fx.track(rows=fx.span_rows(turns=1.5)))
expect("span: 1.5 turns fails", fails(t, "span"))
t = with_run(GOOD, "span", fx.track(rows=fx.span_rows(lo=20.0, hi=1000.0)))
expect("span: a pulse range of 20-1000 us fails", fails(t, "span"))
t = with_run(GOOD, "span", fx.track(rows=fx.span_rows(lo=20.0)))
expect("span: a pulse range starting at 20 us fails on its own", fails(t, "span"))
t = with_run(GOOD, "span", fx.track(rows=fx.span_rows(hi=1040.0)))
expect("span: a pulse range ending at 1040 us fails on its own", fails(t, "span"))
t = with_run(GOOD, "span", fx.track(rows=fx.span_rows(sign=-1)))
expect("span: the count falling as the angle rises still passes (the sign is measured)", state(judge(t)[0], "span") is True)
span = rb.tz_span(fx.span_rows(sign=-1))
expect("tz_span: a falling count gives sign -1 and -8192 per turn", span["sign"] == -1
       and abs(span["countsPerTurn"] + 8192) < 1 and abs(span["turns"] - 3) < 0.01)
span = rb.tz_span(fx.span_rows())
expect("tz_span: 3 turns, about 8192 per turn, residual under 0.5 deg", span["sign"] == 1
       and abs(span["countsPerTurn"] - 8192) < 1 and span["maxResidualDeg"] < 0.5 and span["wraps"] == 3)
expect("tz_span: a still shaft gives None", rb.tz_span([[i, 50, 300, 1, 0] for i in range(100)]) is None)
expect("tz_span: bad CRC and failed bulk reads are left out",
       rb.tz_span([[i, 50, 300, 0, 0] for i in range(100)] + [[i, 50, 300 + i, 1, 1] for i in range(100)]) is None)

t = copy.deepcopy(GOOD)
t["runs"]["init_01"]["result"]["init"]["crcFailures"] = 1
expect("INIT read: a CRC failure fails", fails(t, "INIT read"))
t = copy.deepcopy(GOOD)
t["runs"]["init_01"]["result"]["init"]["readMs"] = fx.summary(8.0, 50, p95=21.0)
expect("INIT read: p95 21 ms fails", fails(t, "INIT read"))
t = copy.deepcopy(GOOD)
t["runs"]["init_01"]["result"]["init"]["spreadDeg"] = 0.7
expect("INIT read: a 0.7 deg spread at rest fails", fails(t, "INIT read"))
t = copy.deepcopy(GOOD)
t["runs"]["init_01"]["result"]["init"]["readMs"] = fx.summary(4.5, 49)
expect("INIT read: 49 reads fails", fails(t, "INIT read"))
t = copy.deepcopy(GOOD)
t["runs"]["init_01"]["result"]["init"]["valid"] = False
expect("INIT read: no valid absolute read, with plausible pulses, fails", fails(t, "INIT read"))

t = copy.deepcopy(GOOD)
t["runs"]["init_05"]["result"]["init"]["offsetDeg"] = 77.8
expect("re-zero: one zero 0.7 deg off the others fails", fails(t, "re-zero: 10 INITs"))
t = with_run(GOOD, "init_07", crashed=True, error="IllegalStateException: no valid absolute read")
got, _ = judge(t)
expect("re-zero: a crashed re-INIT fails its run and the spread check", got.get("re-INIT 7: the run finished") is False
       and state(got, "re-zero: 10 INITs") is False)
t = copy.deepcopy(GOOD)
t["runs"]["init_moved"]["result"]["init"]["offsetDeg"] = 78.6
expect("re-zero: a moved shaft's zero 1.6 deg off fails", fails(t, "re-zero: after the shaft moved"))
t = copy.deepcopy(GOOD)
moved = t["runs"]["init_moved"]["result"]["init"]
moved["absoluteDeg"] = (t["runs"]["init_01"]["result"]["init"]["absoluteDeg"] + 1.0) % 360
expect("re-zero: a shaft that didn't move fails (nothing proven)", fails(t, "re-zero: after the shaft moved"))

agree = lambda **kw: with_run(GOOD, "agreement", fx.track(fx.agreement_rests(**kw)))
got, _ = judge(agree(still=True))
expect("agreement: a stationary shaft fails coverage, the controls and the agreement",
       state(got, "agreement: coverage") is False and state(got, "agreement: setup") is False
       and state(got, "agreement: every rest") is False)
got, _ = judge(agree(steps=13, step_turns=0.05))
expect("agreement: 0.6 turns of motion fails coverage and so the agreement",
       state(got, "agreement: coverage") is False and state(got, "agreement: every rest") is False)
got, _ = judge(agree(steps=9))
expect("agreement: 9 rests fails coverage", state(got, "agreement: coverage") is False)
for kw, label in (({"steps": 9, "step_turns": 10 / 9}, "9 rests, 40 deg apart over 8.9 turns"),
                  ({"step_turns": 1.0}, "13 rests on one angle over 12 turns"),
                  ({"steps": 13, "step_turns": 0.12}, "1.44 turns, over 13 rests and 317 deg")):
    got, _ = judge(agree(**kw))
    expect(f"agreement: {label} fails coverage on that alone", state(got, "agreement: coverage") is False)
t = copy.deepcopy(GOOD)
for r in t["runs"]["agreement"]["result"]["track"]["rests"]:
    r["error180Deg"] = 5.0
got, _ = judge(t)
expect("agreement: a 180 deg offset control that errs only 5 deg fails setup",
       state(got, "agreement: setup") is False and state(got, "agreement: coverage") is True)
got, _ = judge(agree(bad_at=6, bad_deg=3.0))
expect("agreement: a 3 deg error at one rest fails only the agreement",
       state(got, "agreement: every rest") is False and state(got, "agreement: coverage") is True
       and state(got, "agreement: setup") is True)
got, _ = judge(agree(hub_sign=-1))
expect("agreement: the hub count's sign wrong fails the agreement", state(got, "agreement: every rest") is False)
got, _ = judge(agree(bad_at=6, bad_deg=0.75))
expect("agreement: 0.95 deg worst passes", state(got, "agreement: every rest") is True)
t = copy.deepcopy(GOOD)
t["runs"]["agreement"]["result"]["track"]["rests"][7]["absoluteDeg"] = None
t["runs"]["agreement"]["result"]["track"]["rests"][7]["errorDeg"] = None
got, _ = judge(t)
expect("agreement: a rest without a valid absolute read fails the agreement",
       state(got, "agreement: every rest") is False and state(got, "agreement: coverage") is True)
t = with_run(GOOD, "agreement")
got, text = judge(t)
expect("agreement: a missing phase adds no agreement check and says not run",
       state(got, "agreement") == "missing" and "- agreement: not run" in text)
t = with_run(GOOD, "agreement", {})
expect("agreement: a run with no result file fails", judge(t)[0].get("agreement: the run finished") is False)

t = copy.deepcopy(GOOD)
t["runs"]["sign"]["result"]["sign"]["REVERSE"]["bulk"] = 2050
expect("sign: the raw count off the bulk read fails", fails(t, "sign:"))
for direction, key, value, label in (("FORWARD", "bulkFake", True, "a failed bulk read on FORWARD"),
                                     ("FORWARD", "getCurrentPosition", 2048, "getCurrentPosition() not negated on FORWARD"),
                                     ("REVERSE", "getCurrentPosition", -2048, "getCurrentPosition() negated on REVERSE"),
                                     ("REVERSE", "raw", 0, "a raw read of 0 (a failed encoder read)")):
    t = copy.deepcopy(GOOD)
    t["runs"]["sign"]["result"]["sign"][direction][key] = value
    expect(f"sign: {label} fails", fails(t, "sign:"))
t = copy.deepcopy(GOOD)
s = t["runs"]["sign"]["result"]["sign"]
s["FORWARD"].update(raw=40, bulk=40, getCurrentPosition=-40)
s["REVERSE"].update(raw=40, bulk=40, getCurrentPosition=40)
expect("sign: a count under 100 fails (the sign can't be told)", fails(t, "sign:"))
t = copy.deepcopy(GOOD)
s = t["runs"]["sign"]["result"]["sign"]
s["orientation"], s["FORWARD"]["getCurrentPosition"], s["REVERSE"]["getCurrentPosition"] = "CW", 2048, -2048
expect("sign: a CW motor type negating on REVERSE passes", state(judge(t)[0], "sign:") is True)
s["orientation"] = None
expect("sign: no motor type orientation fails", fails(t, "sign:"))
got, text = judge(GOOD)
expect("sign: getCurrentPosition's negation on FORWARD is reported", "negates the raw count on FORWARD" in text)

for key, value, label in (("afterRunUsingEncoder", [3000, 0], "RUN_USING_ENCODER zeroing the count"),
                          ("countAtInit", 40, "a count under 100 to start from"),
                          ("afterRunUsingEncoder", [3000], "a missing read after RUN_USING_ENCODER")):
    t = copy.deepcopy(GOOD)
    t["runs"]["reset"]["result"]["reset"][key] = value
    expect(f"mode changes: {label} fails", fails(t, "mode changes"))
t = copy.deepcopy(GOOD)
t["runs"]["reset_after"]["result"]["resetAfter"]["countAtInit"] = 0
expect("mode changes: the next OpMode starting at 0 fails", fails(t, "mode changes"))
t = copy.deepcopy(GOOD)
t["runs"]["reset_after"]["result"]["resetAfter"]["absoluteAfter"]["absoluteDeg"] = 103.0
expect("encoder reset: the absolute angle 3 deg off afterwards fails", fails(t, "encoder reset"))
expect("encoder reset: STOP_AND_RESET zeroing the count is reported as the hazard",
       "the hazard the turret code must never trigger" in judge(GOOD)[1])

BROWN = fx.good(brownout="ran")
t = copy.deepcopy(BROWN)
t["runs"]["brownout"]["result"]["track"]["lastCount"] = 0
expect("brownout: a count lost with nothing reporting it fails", fails(t, "brownout: a count lost"))
t["brownoutRunner"]["warnings"] = ["Control Hub has encountered an issue"]
expect("brownout: the same loss with a warning passes", state(judge(t)[0], "brownout: a count lost") is True)
t = copy.deepcopy(BROWN)
t["runs"]["brownout"]["result"]["track"].update(lastCount=0, fakeReads=3)
expect("brownout: a failed bulk read counts as reported", state(judge(t)[0], "brownout: a count lost") is True)
t = copy.deepcopy(BROWN)
t["runs"]["brownout"]["result"]["track"]["firstCount"] = 0
expect("brownout: starting from a zero count fails", fails(t, "brownout: a count lost"))
t = copy.deepcopy(BROWN)
t["runs"]["brownout"].update(crashed=True, appRestarted=True, pidAfter=222, result={})
got, text = judge(t)
expect("brownout: a whole-hub reset isn't judged for silence, only the INIT read after it",
       state(got, "brownout: a count lost") == "missing" and "whole-hub reset" in text
       and state(got, "brownout: the INIT read after") is True and not any(n.startswith("brownout: the run") for n in got))
t = copy.deepcopy(BROWN)
del t["runs"]["brownout"]
t["brownoutError"] = "adb lost the hub"
got, text = judge(t)
expect("brownout: a hub the runner lost counts as a whole-hub reset", "whole-hub reset (the runner lost the hub" in text
       and state(got, "brownout: a count lost") == "missing")
t = copy.deepcopy(BROWN)
t["runs"]["brownout_after"]["result"]["init"]["absoluteDeg"] = 98.0
expect("brownout: the INIT read after it 2 deg off fails", fails(t, "brownout: the INIT read after"))

V2 = fx.good(v2=True)
t = copy.deepcopy(V2)
t["runs"]["v2_s"]["result"]["track"]["rows"][100][1] = 40
expect("V2: counts on S fail", fails(t, "V2 on S: no counts"))
t = with_run(V2, "v2_a", fx.track(fx.to_mark("count", 100, 1), rows=fx.span_rows(turns=1.2)))
expect("V2: a V1's pulse range on A fails", fails(t, "V2 on A: one turn"))
V2_ROWS = dict(turns=1.2, lo=3.884, hi=998.06, positions=4096)
t = with_run(V2, "v2_a", fx.track(fx.to_mark("count", 100, 1, per_turn=8100), rows=fx.span_rows(**V2_ROWS)))
expect("V2: 8100 counts per turn on A fails on its own", fails(t, "V2 on A: one turn"))
t = with_run(V2, "v2_a", fx.track(fx.to_mark("count", 100, 1), rows=fx.span_rows(**dict(V2_ROWS, lo=20.0))))
expect("V2: a pulse range starting at 20 us on A fails on its own", fails(t, "V2 on A: one turn"))
t = with_run(V2, "v2_s", fx.track(rows=[[i * 3.5, 0, 500, 1, 0] for i in range(2001)]))
expect("V2: no counts on S but a flat absolute output (the shaft never turned) fails", fails(t, "V2 on S: no counts"))
t = with_run(V2, "v2_a")
expect("V2: asked for but never run says not run", "V2 on A: not run" in judge(t)[1])
for key, value, prefix in (("v2HighV", "3.6", "V2 on A: its supply"), ("v2HighV", "-3.3", "V2 on A: its supply"),
                           ("v2SupplyV", "5.0", "V2 on A: its supply"), ("v2sHighV", "3.6", "V2 on S: its absolute"),
                           ("v2sHighV", "0", "V2 on S: its absolute")):
    t = copy.deepcopy(V2)
    t["answers"][key] = value
    expect(f"V2 levels: {key} {value} fails {prefix}", fails(t, prefix))
t = copy.deepcopy(V2)
t["v2"] = "stopped: the V2's levels on A don't suit the OctoQuad's 3.3 V input"
t["answers"]["v2HighV"] = "3.6"
del t["answers"]["v2sHighV"]
for name in ("v2_a", "v2_s"):
    del t["runs"][name]
got, text = judge(t)
expect("V2 stopped at its level gate: the level check fails, no V2 run check, says why",
       state(got, "V2 on A: its supply") is False and state(got, "V2 on A: one turn") == "missing"
       and state(got, "V2 on S") == "missing" and "V2: not run (stopped: the V2's levels" in text)

t = {"answers": {"cable1": "ok", "split": "ok", "supplyV": "3.3", "highV": "5.0"}, "runs": {"quad": GOOD["runs"]["quad"]},
     "stopped": "the encoder's levels don't suit the OctoQuad's 3.3 V input", "v2": "not asked for"}
got, text = judge(t)
expect("stopped: only the checks that ran, a FAIL on the level, the rest not run",
       len(got) == 3 and state(got, "the encoder's supply") is False
       and "span: not run (the bench stopped: the encoder's levels" in text)

expect("circular span: across 0", abs(rb.circular_span_deg([350, 10, 0]) - 20) < 1e-9)
expect("circular span: three thirds", abs(rb.circular_span_deg([0, 120, 240]) - 240) < 1e-9)
expect("circular span: one angle", rb.circular_span_deg([42]) == 0)
expect("circular mean: across 0", abs(rb.wrap_error_deg(rb.circular_mean_deg([359, 1]), 0)) < 1e-9)
expect("turns to mark: no rest after the far one", rb.tz_turns_to_mark([{"c": 0}, {"c": 5}, {"c": 9}], "c", 1) is None)
expect("volts: parses '3.27 V'", rb.tz_volts("3.27 V") == 3.27 and rb.tz_volts("") is None)

print(f"{passed} passed, {len(failed)} failed")
sys.exit(1 if failed else 0)
