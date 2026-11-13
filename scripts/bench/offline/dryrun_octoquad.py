"""Drives Runner.run_octoquad end to end with every hub call stubbed: the bank phase rides on the seed run, and a crash
in it neither fails the seed checks nor skips the cable pull. Run with: python3 -I <this file>"""
import importlib.util
import pathlib
import sys
import tempfile

sys.dont_write_bytecode = True
HERE = pathlib.Path(__file__).resolve().parent
OUT = HERE / "out" / "dryrun"
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


GOOD_BANK = {"podPorts": [3, 4], "asFound": "ALL_QUADRATURE",
             "readLocalizerDataAsFound": {"count": 300, "p50": 3.2, "p95": 3.8, "p99": 4.0, "max": 8.0, "invalid": 0},
             "readLocalizerDataAndAllEncoderDataAsFound": {"count": 300, "p50": 6.6, "p95": 8.5, "p99": 9.0,
                                                           "max": 17.0, "invalid": 0},
             "afterSet": "BANK1_QUADRATURE_BANK2_PULSE_WIDTH",
             "readLocalizerDataMixed": {"count": 300, "p50": 3.25, "p95": 3.8, "p99": 4.0, "max": 8.0, "invalid": 0},
             "initRead": {"count": 50, "p50": 4.6, "p95": 5.2, "p99": 6.0, "max": 7.0, "crcFailures": 0,
                          "channel4": [0] * 50},
             "afterLocalizerSetup": {"runningMs": 1650.0, "bank": "BANK1_QUADRATURE_BANK2_PULSE_WIDTH"},
             "afterLocalizerReset": {"runningMs": 1630.0, "bank": "BANK1_QUADRATURE_BANK2_PULSE_WIDTH"},
             "afterRestore": "ALL_QUADRATURE"}


class Dry(rb.Runner):
    def __init__(self, argv, scenario):
        self.args = rb.parse_args(argv)
        self.scenario = scenario
        OUT.mkdir(parents=True, exist_ok=True)
        self.out = pathlib.Path(tempfile.mkdtemp(prefix="octoquad-dry-", dir=OUT))
        (self.out / "raw").mkdir()
        (self.out / "logs").mkdir()
        self.logfile = open(self.out / "run.log", "w")
        self.summary, self.checks, self.results = [], [], {}
        self.expansion, self.imu_type, self.webcam, self.limelight = None, "BHI260", None, None
        self.steps, self.configs, self.asked = [], {}, []

    def log(self, text):
        self.logfile.write(text + "\n")

    def ask(self, text):
        self.asked.append(text)

    def activate_config(self, name, xml_text=None, config=None, accept=None, battery=True):
        self.steps.append(("config", name))
        self.configs[name] = xml_text

    def run_opmode(self, opmode, step, params=None, **kw):
        self.steps.append((opmode, step, params))
        crashed = self.scenario in ("bank crash", "seed crash") and step == "octoquad_seed"
        self.results[step] = {}
        if kw.get("during") is not None and not crashed:
            kw["during"]()
        return {"step": step, "opmode": opmode, "crashed": crashed, "timedOut": False,
                "errorMessage": "IllegalStateException: boom" if crashed else "", "statusErrorMessage": ""}

    def wait_or_crash(self, predicate, timeout):
        return 0.5

    def pull(self, name, required=True, local_name=None):
        if name == "octoquad_seed":
            phases = ["create", "seed"] + ([] if self.scenario == "seed crash" else ["bank"])
            bank = None if self.scenario == "seed crash" else dict(GOOD_BANK)
            if self.scenario == "bank crash":
                del bank["afterLocalizerSetup"], bank["afterLocalizerReset"]
            seed = ({"ran": True, "loops": 300, "maxOffsetIn": 0.01, "maxTurnDeg": 0.02} if self.scenario != "seed crash"
                    else {})
            return {"chipId": 0x51, "present": True, "info": {"seed": "Pose(56, 8, 90)"}, "phaseLog": phases,
                    "seed": seed, "crcFailures": 0, "invalidLoops": 0, "bank": bank}
        if name == "octoquad_dropout":
            return {"dropout": {"badLoops": 5, "silentBadLoops": 0}}
        raise AssertionError(f"unexpected pull {name}")


class FakeDash:
    data = {rb.KEY_OCTOQUAD_VALID: "true", rb.KEY_LOOP: 50}
    status = {}


def run(argv, scenario):
    d = Dry(argv, scenario)
    d.dash = FakeDash()
    d.run_octoquad()
    return d, {name: state for name, state, _, _ in d.checks}


d, got = run([], "good")
opmodes = [s for s in d.steps if s[0] != "config"]
expect("default: one Bench: OctoQuad run in seed mode, the bank phase inside it",
       opmodes == [("Bench: OctoQuad", "octoquad_seed", {"mode": "seed"})])
expect("default: the OctoQuad on Control Hub I2C bus 2", '<OctoQuadFTC name="octoquad" port="0" bus="2" />'
       in (d.configs.get(rb.CONFIG_OCTOQUAD) or ""))
expect("default: every seed and bank check passes", got and all(s == "PASS" for s in got.values()))
expect("default: the bank checks are in the run", sum(n.startswith("octoquad bank:") for n in got) == 5)
expect("default: the summary has the bank section", any("bank phase" in line for line in d.summary))

d, got = run(["--interactive"], "bank crash")
expect("bank crash: the seed checks still pass",
       [s for n, s in got.items() if n.startswith("octoquad:")] == ["PASS", "PASS"])
expect("bank crash: the bank phase's own check fails", got.get("octoquad bank: the phase finished") == "FAIL")
expect("bank crash: the cable pull still runs", any(s[1] == "octoquad_dropout" for s in d.steps if s[0] != "config")
       and any("PULL the OctoQuad" in a for a in d.asked))

d, got = run(["--interactive"], "seed crash")
expect("seed crash (control): the seed checks fail", [s for n, s in got.items() if n.startswith("octoquad:")]
       == ["FAIL", "FAIL"])
expect("seed crash (control): the cable pull is skipped", not any(s[1] == "octoquad_dropout" for s in d.steps
                                                                  if s[0] != "config"))
expect("seed crash (control): no bank checks", not any(n.startswith("octoquad bank:") for n in got))

print(f"{passed} passed, {len(failed)} failed")
sys.exit(1 if failed else 0)
