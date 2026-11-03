"""Negative controls for the harness: each mutant undoes one Photon change (or swaps in an older Photon) and must make
every named harness check fail, and nothing else but its listed knock-on checks (the ones that follow from the same
change). Run after harness/run.sh has built the harness:
    python3 -I harness/mutants.py   -> ends "mutants: N of N caught" """
import importlib.util
import pathlib
import re
import subprocess
import sys

sys.dont_write_bytecode = True
spec = importlib.util.spec_from_file_location("classpath", pathlib.Path(__file__).resolve().parents[1] / "classpath.py")
classpath = importlib.util.module_from_spec(spec)
spec.loader.exec_module(classpath)
CP = classpath.resolve()
WT = CP["REPO"]
B = CP["OUT"] / "harness"
PHOTON = "TeamCode/src/main/java/org/firstinspires/ftc/teamcode/architecture/hardware/photon"
TC = CP["TC"]
JARS = [CP[k] for k in ("HW", "RC", "FC", "SIN", "ANN", "ANDROID")]
KOT = CP["KOT"]


def source(name):
    return (WT / PHOTON / name).read_text()


def at(rev, name):
    r = subprocess.run(["git", "-C", str(WT), "show", f"{rev}:{PHOTON}/{name}"], capture_output=True, text=True)
    if r.returncode != 0:
        raise SystemExit(f"git can't show {rev}:{PHOTON}/{name}, which needs this branch's history: {r.stderr.strip()}")
    return r.stdout


def swap(text, old, new):
    if text.count(old) != 1:
        raise SystemExit(f"mutant source no longer matches: {old[:70]!r}")
    return text.replace(old, new)


def method(text, signature):
    """The whole method starting at signature (through its closing brace at the same indent)."""
    i = text.index(signature)
    j = text.index("\n    }\n", i) + len("\n    }\n")
    return text[i:j]


UNBOUNDED = """    protected byte getNewMessageNumber() {
        byte number;
        do {
            number = super.getNewMessageNumber();
        } while (orphanUnfinished.containsKey(number & 0xFF));
        return number;
    }
"""
OLD_ROUTING = """    public void onIncomingDatagramReceived(LynxDatagram datagram) {
        int reference = datagram.getReferenceNumber();
        if (!unfinishedCommands.containsKey(reference) && orphanUnfinished.containsKey(reference)) {
            core.orphanDatagrams.incrementAndGet();
            noteDatagramReceived();
            orphan.onIncomingDatagramReceived(datagram);
            return;
        }
        super.onIncomingDatagramReceived(datagram);
    }
"""
GUESSED_ROUTING = """    public void onIncomingDatagramReceived(LynxDatagram datagram) {
        int reference = datagram.getReferenceNumber();
        LynxRespondable<?> theirs = orphanUnfinished.get(reference);
        if (theirs != null) {
            LynxRespondable<?> mine = unfinishedCommands.get(reference);
            if (mine == null || answers(theirs, datagram) && !answers(mine, datagram)) {
                core.orphanDatagrams.incrementAndGet();
                noteDatagramReceived();
                orphan.onIncomingDatagramReceived(datagram);
                return;
            }
        }
        super.onIncomingDatagramReceived(datagram);
    }

    private static boolean answers(LynxRespondable<?> command, LynxDatagram reply) {
        if (reply.isResponse()) {
            return command.isResponseExpected() && command.getCommandNumber() == reply.getCommandNumber();
        }
        return reply.getCommandNumber() != LynxAck.getStandardCommandNumber() || !command.isResponseExpected();
    }
"""
I2C_OVERRIDE = """    @Override
    public <T> T acquireI2cLockWhile(Supplier<T> supplier) throws InterruptedException, RobotCoreException, LynxNackException {
        warnIfClosed();
        return orphan.acquireI2cLockWhile(supplier);
    }
"""


def draft_module():
    """c825493 plus an unfinished set of fixes: the I2C override, the unbounded number loop and the reply-type routing
    guess."""
    m = at("c825493", "PhotonLynxModule.java")
    m = swap(m, "import com.qualcomm.hardware.lynx.commands.LynxDatagram;\n",
             "import com.qualcomm.hardware.lynx.Supplier;\nimport com.qualcomm.hardware.lynx.commands.LynxDatagram;\n")
    m = swap(m, "    @Override\n" + OLD_ROUTING, I2C_OVERRIDE + "\n    @Override\n" + UNBOUNDED + "\n    @Override\n" + GUESSED_ROUTING)
    return m


def mutate_module(fn):
    return lambda: {"PhotonLynxModule.java": fn(source("PhotonLynxModule.java")), "PhotonCore.java": source("PhotonCore.java")}


def mutate_core(fn):
    return lambda: {"PhotonLynxModule.java": source("PhotonLynxModule.java"), "PhotonCore.java": fn(source("PhotonCore.java"))}


def unbounded(m):
    return swap(m, method(m, "    protected byte getNewMessageNumber() {"), UNBOUNDED)


def no_number_override(m):
    return swap(m, "    @Override\n" + method(m, "    protected byte getNewMessageNumber() {"), "")


def routing(replacement):
    def fn(m):
        old = method(m, "    public void onIncomingDatagramReceived(LynxDatagram datagram) {")
        return swap(m, old, replacement)
    return fn


def traced_failure_log(m):
    m = swap(m, """            } catch (RobotUsbException | RuntimeException e) {
                failure = e;
            }""", """            } catch (RobotUsbException | RuntimeException e) {
                RobotLog.ww(PhotonCore.TAG, e, "parallel write to REV Hub #%d failed; sending it the SDK's way",
                        getModuleAddress());
                core.writeFailures.incrementAndGet();
                return false;
            }""")
    return m


def ping_every_write(m):
    return swap(m, "        if (now - lastPingResetNanos < PING_RESET_NS) return;\n", "")


def no_ping_reset(m):
    return swap(m, "        resetPingTimerIfDue(command, sentNanos);\n", "")


def no_i2c_delegation(m):
    return swap(m, "        return orphan.acquireI2cLockWhile(supplier);", "        return super.acquireI2cLockWhile(supplier);")


LIVE_DEVICE = "if (device == null || !armed || impl.getShutdownReason() != SyncdDevice.ShutdownReason.NORMAL || !implEngaged()) {"


def live_device(condition):
    return lambda m: swap(m, LIVE_DEVICE, condition)


def clash_not_heard(m):
    return swap(m, """            noteDatagramReceived();
            // An ACK or NACK""", """            if (!unfinishedCommands.containsKey(reference)) noteDatagramReceived();
            // An ACK or NACK""")


def no_upper_bound(c):
    return swap(c, "if (writes < 1 || writes > MAX_PARALLEL_WRITES) {", "if (writes < 1) {")


def profiling_on(c):
    return swap(c, "    private static volatile boolean writeProfiling;", "    private static volatile boolean writeProfiling = true;")


I2C_LOCK = ["I2C lock: a transaction through Photon's module holds", "I2C lock: a transaction through Photon's module shuts out",
            "I2C lock: a transaction through the replaced module shuts out", "I2C clients: no transaction on the hub began",
            "I2C clients: every read returned its own"]
NUMBERS = ["message numbers: 300 parallel and 300 SDK-path", "message numbers: Photon's parallel writes meanwhile wrap",
           "message numbers: with all 255 pending"]
CLASH = ["shared number: a NACK, an ACK and a response"]
CLASH_AFTER = ["shared number: Photon's write is then abandoned", "shared number: the replaced module's query takes",
               "message numbers: nothing leaked, and the one abandoned"]
CLASH_HEARD = ["shared number: a dropped reply still counts"]
ACK_PROFILE = ["write profile: 50 parallel writes counted"]
PROFILE_OFF = ["write profile: switched off again"]
I2C_AFTER = ["I2C clients: every reply to the replaced module's", "I2C lock: Photon's module, once closed"]
FAILURE_LOG = ["write failure: logged as one line", "write failure: logged after the USB device's write lock"]
PING = ["keep-alive: 200 parallel writes over"]
PING_KEPT = ["keep-alive: none sent while writes come"]
PING_SILENCE = ["keep-alive: the hub hears one"]
PROFILE = ["write profile: off by default", "write profile: 50 parallel writes counted"]
BOUND = ["setMaxParallelWrites(65) throws"]

# (name, sources, checks that must fail, knock-on checks that may also fail)
MUTANTS = [
    ("unbounded message-number loop", mutate_module(unbounded), ["message numbers: with all 255 pending"], []),
    ("no message-number override", mutate_module(no_number_override), NUMBERS, []),
    ("replies routed by guessing from the reply type", mutate_module(routing(GUESSED_ROUTING)), CLASH,
     CLASH_AFTER + CLASH_HEARD + ACK_PROFILE),
    ("replies routed to Photon's map first", mutate_module(routing(OLD_ROUTING)), CLASH, CLASH_AFTER + CLASH_HEARD + ACK_PROFILE),
    ("a dropped clashed reply not counted as hearing from the hub", mutate_module(clash_not_heard), CLASH_HEARD, []),
    ("write failure logged with its stack trace under the write lock", mutate_module(traced_failure_log), FAILURE_LOG, []),
    ("ping timer reset on every write", mutate_module(ping_every_write), PING, []),
    ("no ping-timer reset on parallel writes", mutate_module(no_ping_reset), PING_KEPT, PING_SILENCE),
    ("I2C lock not delegated to the replaced module", mutate_module(no_i2c_delegation), I2C_LOCK, I2C_AFTER),
    ("no shutdown-reason check before a parallel write",
     mutate_module(live_device("if (device == null || !armed || !implEngaged()) {")), ["device not ready: shut down abnormally"], []),
    ("no engaged check before a parallel write",
     mutate_module(live_device("if (device == null || !armed || impl.getShutdownReason() != SyncdDevice.ShutdownReason.NORMAL) {")),
     ["device not ready: disengaged"], []),
    ("no null-device or shutdown-reason check before a parallel write",
     mutate_module(live_device("if (!armed || !implEngaged()) {")),
     ["device not ready: shut down abnormally", "device not ready: no USB device"], []),
    ("no upper bound on parallel writes", mutate_core(no_upper_bound), BOUND, []),
    ("write profiling on by default", mutate_core(profiling_on), ["write profile: off by default"], ACK_PROFILE + PROFILE_OFF),
    ("c825493's Photon", lambda: {"PhotonLynxModule.java": at("c825493", "PhotonLynxModule.java"),
                                  "PhotonCore.java": at("c825493", "PhotonCore.java")},
     I2C_LOCK + NUMBERS + CLASH + FAILURE_LOG + PING + PROFILE + BOUND, CLASH_AFTER + CLASH_HEARD + PROFILE_OFF + I2C_AFTER),
    ("the unfinished draft", lambda: {"PhotonLynxModule.java": draft_module(), "PhotonCore.java": at("c825493", "PhotonCore.java")},
     ["message numbers: with all 255 pending"] + CLASH + FAILURE_LOG + PING + PROFILE + BOUND,
     CLASH_AFTER + CLASH_HEARD + PROFILE_OFF + I2C_AFTER),
]


def run(name, files, expect, knock_on):
    d = B / "mutants" / re.sub(r"\W+", "-", name)
    src, cls = d / "src", d / "classes"
    src.mkdir(parents=True, exist_ok=True)
    cls.mkdir(parents=True, exist_ok=True)
    for f, text in files.items():
        (src / f).write_text(text)
    cp = ":".join(str(x) for x in [TC, *JARS])
    subprocess.run(["javac", "-nowarn", "-source", "8", "-target", "8", "-d", str(cls), "-cp", cp,
                    *[str(src / f) for f in files]], check=True, capture_output=True)
    run_cp = ":".join(str(x) for x in [B / "fakes", B / "classes", cls, TC, *JARS, KOT])
    try:
        out = subprocess.run(["java", "-cp", run_cp, "com.qualcomm.hardware.lynx.PhotonHarness"], capture_output=True,
                             text=True, timeout=240).stdout
    except subprocess.TimeoutExpired as e:
        out = (e.stdout or b"").decode() if isinstance(e.stdout, bytes) else (e.stdout or "")
        out += "\nTIMEOUT"
    fails = [l[5:] for l in out.splitlines() if l.startswith("FAIL ")]
    tally = re.search(r"^(\d+) passed, (\d+) failed$", out, re.M)
    if tally is None or int(tally.group(2)) != len(fails):
        fails.append("the harness aborted or never finished: " + (out.strip().splitlines() or ["no output"])[-1][:120])
    missed = [e for e in expect if not any(f.startswith(e) for f in fails)]
    stray = [f for f in fails if not any(f.startswith(e) for e in expect + knock_on)]
    caught = not missed and not stray
    print(f"{'CAUGHT' if caught else 'MISSED'} {name}: {len(fails)} failing check(s)"
          + (f"; expected to fail but passed: {missed}" if missed else "")
          + (f"; failing outside its named and knock-on checks: {[s[:90] for s in stray]}" if stray else ""))
    return caught


if not (B / "classes").is_dir():
    raise SystemExit("run harness/run.sh first")
results = [run(n, f(), e, k) for n, f, e, k in MUTANTS]
print(f"mutants: {sum(results)} of {len(results)} caught")
sys.exit(0 if all(results) else 1)
