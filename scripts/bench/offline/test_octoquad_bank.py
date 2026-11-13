"""Offline checks for Bench: OctoQuad's bank phase in run_bench.py: a good synthetic result passes, each broken one
fails, and a crash in the bank phase leaves the seed checks as they were. Run with: python3 -I <this file>"""
import copy
import importlib.util
import pathlib
import sys
import tempfile

sys.dont_write_bytecode = True
HERE = pathlib.Path(__file__).resolve().parent
OUT = HERE / "out" / "checks"
spec = importlib.util.spec_from_file_location("run_bench", HERE.parent / "run_bench.py")
rb = importlib.util.module_from_spec(spec)
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


def ms(p50, count=rb.OCTOQUAD_BANK_READS, **extra):
    s = {"count": count, "mean": p50 + 0.1, "min": p50 - 0.4, "p50": p50, "p90": p50 + 0.4, "p95": p50 + 0.6,
         "p99": p50 + 1.5, "max": p50 + 4.0, "largest": [p50 + 4.0]}
    s.update(extra)
    return s


MIXED, QUAD = rb.OCTOQUAD_MIXED_BANKS, rb.OCTOQUAD_QUADRATURE
GOOD = {"podPorts": [3, 4], "asFound": QUAD,
        "readLocalizerDataAsFound": ms(3.20, invalid=0),
        "readLocalizerDataAndAllEncoderDataAsFound": ms(6.58, invalid=0),
        "afterSet": MIXED,
        "readLocalizerDataMixed": ms(3.24, invalid=0),
        "initRead": ms(4.6, count=rb.OCTOQUAD_INIT_READS, crcFailures=0, channel4=[0] * rb.OCTOQUAD_INIT_READS),
        "afterLocalizerSetup": {"leftRunningMs": 4.1, "runningMs": 1650.0, "status": "RUNNING", "bank": MIXED},
        "afterLocalizerReset": {"leftRunningMs": 3.9, "runningMs": 1630.0, "status": "RUNNING", "bank": MIXED},
        "afterRestore": QUAD}


def mutate(**changes):
    b = copy.deepcopy(GOOD)
    for k, v in changes.items():
        if v is DELETE:
            del b[k]
        else:
            b[k] = v
    return b


DELETE = object()


def results(bank, error=""):
    checks, lines = rb.octoquad_bank_checks(bank, error)
    return {name: ok for name, ok, _, _ in checks}, "\n".join(lines)


def failing(bank, needle):
    got, _ = results(bank)
    return any(needle in name and not ok for name, ok in got.items())


got, text = results(GOOD)
expect("good: five checks", len(got) == 5)
expect("good: every check passes", got and all(got.values()))
expect("good: the summary says the bank survives the localizer's setup and reset", "the bank survives" in text)
expect("good: the summary records the pod ports and the robot's channels",
       "[3, 4]" in text and "on the robot the pods sit on channels 0 and 1" in text)

expect("cost: +0.5 ms with bank 2 in pulse-width mode fails",
       failing(mutate(readLocalizerDataMixed=ms(3.70, invalid=0)), "within 0.2 ms"))
expect("cost: -0.3 ms fails too", failing(mutate(readLocalizerDataMixed=ms(2.90, invalid=0)), "within 0.2 ms"))
expect("cost: +0.15 ms passes", not failing(mutate(readLocalizerDataMixed=ms(3.35, invalid=0)), "within 0.2 ms"))

blind = mutate(**{"readLocalizerDataAndAllEncoderDataAsFound": ms(3.30, invalid=0)})
got, text = results(blind)
expect("control: a combined read no slower fails setup", failing(blind, "setup: the timing sees a cost"))
expect("control: with a blind setup the cost isn't judged", not any("within 0.2 ms" in n for n in got)
       and "isn't judged" in text)
expect("control: 2.4 ms slower still fails setup",
       failing(mutate(readLocalizerDataAndAllEncoderDataAsFound=ms(5.60, invalid=0)), "setup"))

stuck = mutate(afterSet=QUAD)
got, _ = results(stuck)
expect("bank: reading back ALL_QUADRATURE after the set fails", failing(stuck, "read back after the set"))
expect("bank: the cost isn't judged when the set didn't take", not any("within 0.2 ms" in n for n in got))
expect("bank: found in pulse-width mode fails", failing(mutate(asFound=MIXED, afterRestore=MIXED), "as found"))

expect("restore: a bank left in pulse-width mode fails", failing(mutate(afterRestore=MIXED), "reads back after the phase"))
expect("restore: no read-back fails", failing(mutate(afterRestore=DELETE), "reads back after the phase"))

crashed = mutate(afterLocalizerSetup=DELETE, afterLocalizerReset=DELETE)
got, _ = results(crashed, "IllegalStateException: boom")
expect("crash: the phase-finished check fails", failing(crashed, "the phase finished"))
expect("crash: the restore that ran still passes", got.get(
    "octoquad bank: the bank found at the start reads back after the phase") is True)
expect("crash: the timings measured before it are still judged", any("within 0.2 ms" in n for n in got))
stopped = {name: (ok, detail) for name, ok, detail, _ in rb.octoquad_bank_checks(
    mutate(afterLocalizerSetup=DELETE, afterLocalizerReset=DELETE, stopRequested=True), "")[0]}
expect("STOP mid-phase: the phase-finished check fails naming the STOP, and the bank found reads back",
       stopped["octoquad bank: the phase finished"][0] is False
       and "STOP during it True" in stopped["octoquad bank: the phase finished"][1]
       and stopped["octoquad bank: the bank found at the start reads back after the phase"][0] is True)

expect("init read: a CRC failure fails", failing(mutate(initRead=ms(4.6, count=50, crcFailures=1, channel4=[])),
                                                 "INIT read"))
expect("init read: p95 at 20 ms fails", failing(mutate(initRead=ms(19.4, count=50, crcFailures=0, channel4=[])),
                                                "INIT read"))
expect("init read: 49 reads fails", failing(mutate(initRead=ms(4.6, count=49, crcFailures=0, channel4=[])),
                                            "INIT read"))

lost = mutate(afterLocalizerSetup=dict(GOOD["afterLocalizerSetup"], bank=QUAD))
got, text = results(lost)
expect("survival: a bank lost to the localizer setup is recorded, not failed",
       all(got.values()) and "does NOT survive" in text)

got, text = results(None)
expect("not run: no checks and a not-run line", got == {} and "not run" in text)

expect("bank_phase_crashed: a crash after the bank phase began",
       rb.bank_phase_crashed({"crashed": True}, {"phaseLog": ["create", "seed", "bank"]}))
expect("bank_phase_crashed: a crash in the seed phase isn't one",
       not rb.bank_phase_crashed({"crashed": True}, {"phaseLog": ["create", "seed"]}))
expect("bank_phase_crashed: no crash isn't one", not rb.bank_phase_crashed({"crashed": False}, {"phaseLog": ["bank"]}))


class Seed(rb.Runner):
    def __init__(self):
        self.summary, self.checks = [], []
        OUT.mkdir(parents=True, exist_ok=True)
        self.logfile = open(tempfile.mkstemp(prefix="octoquad-bank-", dir=OUT)[1], "w")
        self.args = rb.parse_args([])

    def log(self, text):
        self.logfile.write(text + "\n")


def seed_checks(crashed, phases, bank):
    r = Seed()
    res = {"chipId": 0x51, "present": True, "info": {"seed": "Pose(56, 8, 90)"}, "phaseLog": phases,
           "seed": {"ran": True, "loops": 300, "maxOffsetIn": 0.01, "maxTurnDeg": 0.02},
           "crcFailures": 0, "invalidLoops": 0, "bank": bank}
    o = {"crashed": crashed, "errorMessage": "IllegalStateException: boom" if crashed else ""}
    r._octoquad_seed(o, res)
    return {name: state for name, state, _, _ in r.checks}, "\n".join(r.summary)


SEED_NAMES = ("octoquad: a start pose", "octoquad: every loop's localizer read")
got, text = seed_checks(False, ["create", "seed", "bank"], GOOD)
expect("seed: a good run passes the seed and bank checks", all(s == "PASS" for s in got.values()) and len(got) == 7)
got, text = seed_checks(True, ["create", "seed", "bank"], mutate(afterLocalizerSetup=DELETE, afterLocalizerReset=DELETE))
expect("seed: a crash in the bank phase leaves both seed checks passing",
       all(s == "PASS" for n, s in got.items() if n.startswith(SEED_NAMES)) and
       sum(n.startswith(SEED_NAMES) for n in got) == 2)
expect("seed: a crash in the bank phase fails only the bank phase", got.get("octoquad bank: the phase finished") == "FAIL"
       and "in the bank phase, after the seed phase finished" in text)
got, text = seed_checks(True, ["create", "seed"], None)
expect("seed: a crash before the bank phase still fails the seed checks (control)",
       all(s == "FAIL" for n, s in got.items() if n.startswith(SEED_NAMES)) and not any("bank" in n for n in got)
       and "not run" in text)

print(f"{passed} passed, {len(failed)} failed")
sys.exit(1 if failed else 0)
