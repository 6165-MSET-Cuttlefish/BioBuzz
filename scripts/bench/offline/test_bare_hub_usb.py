"""bare_hub.sh --usb end to end from a copy of the scripts, with stub lsof, git, gradlew and adb first on PATH (adb a
fake Control Hub that logs every call), so nothing reaches a hub, this Mac's adb server or the network: no adb connect,
REINSTALL.sh for the cable (and run), the dashboard port judged from the hub's socket table, the runner given --usb and
the serial, the --usb build with and without internet; and the Wi-Fi run's adb calls, runner flags and REINSTALL.sh
byte for byte the ones recorded from the script before --usb (fixtures/bare_hub_wifi.json, from
`git show ea69f20:scripts/bench/bare_hub.sh`). Run with: python3 -I scripts/bench/offline/test_bare_hub_usb.py
(about 1.5 min)

    --record FILE --script PATH   write the Wi-Fi run's record from that copy of bare_hub.sh to FILE
"""
import argparse
import hashlib
import importlib.util
import json
import pathlib
import re
import shutil
import socket
import subprocess
import sys
import tempfile

HERE = pathlib.Path(__file__).resolve().parent
WORKTREE = HERE.parents[2]
OUT = HERE / "out" / "bare_hub_usb"
FIXTURE = HERE / "fixtures" / "bare_hub_wifi.json"
HEAD = "0123456789abcdef0123456789abcdef01234567"
SERIAL = "1a2b3c4d5e6f"
WIFI_SERIAL = "192.168.43.1:5555"
OTHER = "R58M90ABCDE"

passed = 0
failed = []


def expect(label, cond):
    global passed
    if cond:
        passed += 1
    else:
        failed.append(label)
        print("FAIL", label)


def closed_port():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    assert port != 5037 and not 5554 <= port <= 5585, port
    return port


ADB = r'''#!PYTHON
import os, pathlib, sys
args = sys.argv[1:]
with open(os.environ["ADB_LOG"], "a") as log:
    log.write(" ".join(args) + "\n")
devices = [d.split("=") for d in os.environ["ADB_DEVICES"].split(",") if d]
if args[:1] == ["devices"]:
    print("List of devices attached")
    for serial, state in devices:
        print(f"{serial}\t{state}")
    print()
    sys.exit(0)
if args[:1] == ["version"]:
    print("Android Debug Bridge version 1.0.41")
    sys.exit(0)
if args[:1] == ["connect"]:
    print(f"connected to {args[1]}")
    sys.exit(0)
if args[:2] == ["forward", "--list"]:
    sys.exit(0)
if args[:1] != ["-s"] or args[1] not in [s for s, _ in devices]:
    print(f"adb: device not found", file=sys.stderr)
    sys.exit(1)
cmd, rest = args[2], args[3:]
shell = " ".join(rest)
if cmd == "get-state":
    print("device")
elif cmd == "forward" and rest[:1] == ["tcp:0"]:
    print(os.environ["FORWARD_PORT"])
elif cmd == "pull":
    path = pathlib.Path(rest[-1])
    if rest[0].endswith("sloth"):
        path.mkdir(parents=True, exist_ok=True)
    else:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("pulled")
elif cmd in ("install", "install-multiple"):
    print("Performing Streamed Install\nSuccess")
elif cmd == "shell":
    if shell == "getprop persist.ftcandroid.serialasusb":
        print("1")
    elif shell.startswith("pm path"):
        print("package:/data/app/com.qualcomm.ftcrobotcontroller-1/base.apk")
    elif shell.startswith("ls -1 /sdcard/FIRST"):
        print("betta_decode.xml")
    elif "_preferences.xml" in shell and "cat" in shell:
        print("<?xml version='1.0' encoding='utf-8' standalone='yes' ?><map></map>")
    elif shell.startswith("test -d"):
        sys.exit(1)
    elif shell.startswith("date"):
        print("11-12 10:00:00")
    elif "grep -F -e '======= INIT FINISH ======='" in shell:
        print("11-12 10:00:05.000  1  1 I FtcEventLoop: ======= INIT FINISH =======")
    elif shell == "cat /proc/net/tcp /proc/net/tcp6":
        print("  sl  local_address rem_address   st tx_queue rx_queue tr tm->when retrnsmt   uid  timeout inode")
        address = os.environ.get("DASHBOARD_ADDRESS")
        if address:
            print(f"   0: {address}:1F40 00000000:0000 0A 00000000:00000000 00:00000000 00000000 10070 0 1")
    elif shell.startswith("pidof"):
        print("4000")
    elif shell == "getprop ro.serialno":
        values = os.environ.get("HUB_SERIALNO", "FAKEHUB0001").split(",")
        with open(os.environ["ADB_LOG"]) as log:
            n = sum(1 for line in log if line.endswith("shell getprop ro.serialno\n"))
        print(values[min(n, len(values)) - 1])
'''

# Stands in for python3: logs the runner's flags and the Wi-Fi TCP check instead of running them, runs the rest.
PYTHON = r'''#!PYTHON -I
import os, sys
args = sys.argv[1:]
if args[:1] == ["scripts/bench/run_bench.py"] or any("socket.create_connection" in a for a in args):
    with open(os.environ["PY_LOG"], "a") as log:
        log.write(("runner " + " ".join(args[1:]) if args[:1] == ["scripts/bench/run_bench.py"]
                   else "tcp " + " ".join(args[-2:])) + "\n")
    sys.exit(0)
os.execv(sys.executable, [sys.executable] + args)
'''


def bare_hub(devices, args=("--usb",), dashboard_address="00000000", gradle_online=True, start_wait=None, build=None,
             script=None, intercept=False, serialno="FAKEHUB0001", git_works=True):
    """Runs a copy of bare_hub.sh (this checkout's unless script names another); returns (exit status, output, adb
    calls, intercepted python calls, the folder it ran in, the gradlew calls)."""
    OUT.mkdir(parents=True, exist_ok=True)
    root = pathlib.Path(tempfile.mkdtemp(prefix="run-", dir=OUT))
    (root / "scripts/bench").mkdir(parents=True)
    text = pathlib.Path(script or WORKTREE / "scripts/bench/bare_hub.sh").read_text()
    if start_wait is not None:
        text = text.replace("START_WAIT_S=180", f"START_WAIT_S={start_wait}")
    (root / "scripts/bench/bare_hub.sh").write_text(text)
    shutil.copy(WORKTREE / "scripts/bench/run_bench.py", root / "scripts/bench/run_bench.py")
    apk = root / "TeamCode/build/outputs/apk/debug/TeamCode-debug.apk"
    apk.parent.mkdir(parents=True)
    apk.write_bytes(b"apk")
    spec = importlib.util.spec_from_file_location("copied_run_bench", root / "scripts/bench/run_bench.py")
    rb = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(rb)
    photon = rb.photon_source_sha256(root / rb.PHOTON_SOURCE_DIR)
    if build is None:
        (apk.parent / ".hubtest-build").write_text(f"head={HEAD}\napk={apk.name}\nsha256="
                                                   f"{hashlib.sha256(b'apk').hexdigest()}\nphoton={photon}\nbuilt=now\n")
    config = root / "TeamCode/build/generated/source/buildConfig/debug/com/qualcomm/ftcrobotcontroller/BuildConfig.java"
    config.parent.mkdir(parents=True)
    config.write_text(f'  public static final String PHOTON_SOURCE_SHA256 = "{photon}";\n')
    stubs = root / "stubs"
    stubs.mkdir()
    (stubs / "lsof").write_text('#!/bin/sh\nprintf "p45913\\ncadb\\nf7\\n"\n')
    (stubs / "git").write_text(f'#!/bin/sh\ncase "$*" in\n  "rev-parse HEAD") echo {HEAD} ;;\n'
                               f'  "rev-parse --short HEAD") echo 0123456 ;;\n'
                               f'  "rev-parse --path-format=absolute --git-common-dir") echo {root}/common ;;\n'
                               '  status*) ;;\nesac\n')
    if not git_works:
        (stubs / "git").write_text('#!/bin/sh\necho "You have not agreed to the licence" >&2\nexit 69\n')
    (stubs / "adb").write_text(ADB.replace("PYTHON", sys.executable))
    (root / "gradlew").write_text('#!/bin/sh\necho "$@" >> "$GRADLE_LOG"\ncase "$*" in\n  *--offline*) exit 0 ;;\n'
                                  f'  *) exit {0 if gradle_online else 1} ;;\nesac\n')
    if intercept:
        (stubs / "python3").write_text(PYTHON.replace("PYTHON", sys.executable))
        (stubs / "python3").chmod(0o755)
    else:
        (stubs / "python3").symlink_to(sys.executable)
    for f in (stubs / "lsof", stubs / "git", stubs / "adb", root / "gradlew"):
        f.chmod(0o755)
    adb_log, gradle_log, py_log = root / "adb.log", root / "gradle.log", root / "py.log"
    env = {"PATH": f"{stubs}:/usr/bin:/bin", "HOME": str(root), "ADB_LOG": str(adb_log),
           "GRADLE_LOG": str(gradle_log), "PY_LOG": str(py_log),
           "ADB_DEVICES": ",".join(f"{s}={st}" for s, st in devices),
           "FORWARD_PORT": str(closed_port()), "DASHBOARD_ADDRESS": dashboard_address or "", "HUB_SERIALNO": serialno}
    argv = ["/bin/bash", str(root / "scripts/bench/bare_hub.sh")] + list(build if build is not None else args)
    r = subprocess.run(argv, env=env, capture_output=True, text=True, timeout=300, cwd=root)
    lines = (lambda p: p.read_text().splitlines() if p.exists() else [])
    return r.returncode, r.stdout + r.stderr, lines(adb_log), lines(py_log), root, lines(gradle_log)


def reinstall_script(root):
    return next((root / "common/bench-backups").glob("*/REINSTALL.sh"))


def run_reinstall(root, devices, serialno="FAKEHUB0001"):
    """Runs the REINSTALL.sh a run wrote, against the same stub adb; returns (exit status, output, adb calls)."""
    log = root / "reinstall_adb.log"
    log.unlink(missing_ok=True)
    env = {"PATH": f"{root / 'stubs'}:/usr/bin:/bin", "HOME": str(root), "ADB_LOG": str(log),
           "ADB_DEVICES": ",".join(f"{s}={st}" for s, st in devices), "HUB_SERIALNO": serialno}
    r = subprocess.run(["/bin/bash", str(reinstall_script(root))], env=env, capture_output=True, text=True, timeout=60)
    return r.returncode, r.stdout + r.stderr, log.read_text().splitlines() if log.exists() else []


def wifi_record(script=None):
    """The Wi-Fi run's adb calls, TCP checks, runner flags and REINSTALL.sh, with this run's folder and time taken out."""
    code, out, calls, py, root, gradle = bare_hub([(WIFI_SERIAL, "device")], (), script=script, intercept=True)

    def norm(text):
        return re.sub(r"\d{4}-\d\d-\d\d_\d{6}", "<time>", text.replace(str(root), "<root>"))

    return {"exit": code, "adb": [norm(c) for c in calls], "python": [norm(c) for c in py],
            "gradle": gradle, "reinstall": norm(reinstall_script(root).read_text())}


def usb_runs():
    code, out, calls, _, root, gradle = bare_hub([(SERIAL, "device")])
    reinstall = reinstall_script(root).read_text()
    results = next((root / "bench-results").glob("*"))
    summary = (results / "summary.md").read_text() if (results / "summary.md").exists() else ""
    wrapper_calls = calls[:next((i for i, c in enumerate(calls) if c == "version"), len(calls))]
    expect("one USB device: it backs up, installs and hands over to the runner", "installing" in out
           and "running the benches" in out and any(c.startswith(f"-s {SERIAL} install -r -t -g") for c in calls))
    expect("one USB device: not one adb connect from the wrapper or the runner",
           not [c for c in calls if c.split()[:1] == ["connect"] or " connect " in f" {c} "])
    expect("one USB device: the wrapper's device commands all name the cable's serial",
           all(c.startswith(f"-s {SERIAL} ") for c in wrapper_calls if c != "devices"))
    expect("one USB device: it judged the dashboard port from the hub's socket table",
           f"-s {SERIAL} shell cat /proc/net/tcp /proc/net/tcp6" in wrapper_calls)
    expect("one USB device: the runner got --usb and the serial", "--usb" in summary
           and f"--serial {SERIAL}" in summary)
    expect("one USB device: no --gradle-offline when Gradle builds online", "--gradle-offline" not in summary
           and any("--offline" not in g and "assembleSloth" in g for g in gradle))
    expect("one USB device: summary.md says the run used the cable", "Link: USB cable (adb " + SERIAL in summary)
    expect("REINSTALL.sh over USB: waits for the cable's serial on USB and never runs adb connect",
           f"-s {SERIAL} wait-for-usb-device" in reinstall and " connect " not in reinstall
           and f"-s {SERIAL} install -r -t -d" in reinstall)
    expect("REINSTALL.sh over USB: the adb-port check comes before its first adb command",
           0 <= reinstall.find("lsof -nP") < reinstall.find("wait-for-usb-device"))
    expect("one USB device: it ends telling you to tell Claude, without reconnecting to the internet",
           "Tell Claude the run finished." in out and "Reconnect to the internet" not in out)
    rcode, rout, rcalls = run_reinstall(root, [(SERIAL, "device"), (OTHER, "device")])
    expect("REINSTALL.sh over USB runs to its end on the cable, every adb command naming the hub's serial",
           rcode == 0 and rcalls and rcalls[0] == f"-s {SERIAL} wait-for-usb-device"
           and all(c.startswith(f"-s {SERIAL} ") for c in rcalls)
           and any(c.startswith(f"-s {SERIAL} install -r -t -d") for c in rcalls)
           and rcalls[-1].startswith(f"-s {SERIAL} shell am start -n "))

    code, out, calls, _, root, _ = bare_hub([(SERIAL, "device"), (OTHER, "device")])
    expect("two USB devices and no --serial: stops before any device command",
           code != 0 and "adb lists 2 USB devices" in out and calls == ["devices"])
    code, out, calls, _, root, _ = bare_hub([(SERIAL, "device"), (OTHER, "device")], ["--usb", "--serial", SERIAL],
                                            gradle_online=False)
    summary = (next((root / "bench-results").glob("*")) / "summary.md").read_text()
    expect("two USB devices and --serial: uses the named one, never the other",
           "running the benches" in out and not [c for c in calls if OTHER in c])
    expect("two USB devices and --serial: the runner gets that --serial once", summary.count(f"--serial {SERIAL}") == 1)
    expect("Gradle only offline: the runner gets --gradle-offline", "--gradle-offline" in summary)
    code, out, calls, _, root, _ = bare_hub([(SERIAL, "device"), (OTHER, "device")], ["--usb", f"--serial={OTHER}"])
    expect("--serial=X: the wrapper uses that device", any(c.startswith(f"-s {OTHER} install") for c in calls)
           and not [c for c in calls if c.startswith(f"-s {SERIAL} ")])
    code, out, calls, _, root, _ = bare_hub([])
    expect("no USB device: stops before any device command", code != 0 and "adb lists no USB device" in out
           and calls == ["devices"])
    code, out, calls, _, root, _ = bare_hub([(WIFI_SERIAL, "device")])
    expect("only the Wi-Fi hub in adb devices: --usb stops before any device command, never using it",
           code != 0 and "adb lists no USB device" in out and calls == ["devices"])
    code, out, calls, _, root, _ = bare_hub([(SERIAL, "device")], dashboard_address=None, start_wait=4)
    expect("the hub's dashboard port not listening: it keeps waiting and stops, though a forward would accept",
           code != 0 and "no dashboard on port 8000" in out and "running the benches" not in out)
    code, out, calls, _, root, _ = bare_hub([(SERIAL, "device")], dashboard_address="012BA8C0", start_wait=4)
    expect("the dashboard listening only on the hub's Wi-Fi address, which a forward can't reach: stops the same way",
           code != 0 and "no dashboard on port 8000" in out and "running the benches" not in out)
    code, out, calls, _, root, _ = bare_hub([(SERIAL, "unauthorized")])
    expect("an unauthorized USB device: stops before any device command, saying to accept the prompt",
           code != 0 and "accept its debugging prompt" in out and calls == ["devices"])


def serialno_runs():
    """--install-only --expect-serialno, as session.py runs it over Wi-Fi."""
    args = ["--install-only", "bench-results/sess/install", "--expect-serialno", "FAKEHUB0001"]
    wifi = [(WIFI_SERIAL, "device")]
    code, out, calls, _, root, _ = bare_hub(wifi, args, intercept=True)
    reads = [i for i, c in enumerate(calls) if c.endswith("shell getprop ro.serialno")]
    install = next((i for i, c in enumerate(calls) if " install -r -t -g " in f" {c} "), None)
    serialasusb = next(i for i, c in enumerate(calls) if c.endswith("getprop persist.ftcandroid.serialasusb"))
    first_pull = next(i for i, c in enumerate(calls) if " pull " in f" {c} ")
    expect("--expect-serialno, the right hub: installs, reading ro.serialno right after the Control Hub check and "
           "again right before the install", code == 0 and install is not None and len(reads) == 2
           and reads[0] == serialasusb + 1 and reads[0] < first_pull and reads[1] == install - 1)
    reinstall = reinstall_script(root).read_text()
    expect("--expect-serialno: REINSTALL.sh reads ro.serialno after waiting for the hub and before installing",
           0 <= reinstall.find("wait-for-device") < reinstall.find("getprop ro.serialno")
           < reinstall.find("install -r -t -d") and "FAKEHUB0001" in reinstall)
    rcode, rout, rcalls = run_reinstall(root, wifi, serialno="OTHERHUB0002")
    expect("REINSTALL.sh on another hub at the same address: stops before installing, saying which hub answered",
           rcode != 0 and "ro.serialno OTHERHUB0002, not FAKEHUB0001" in rout
           and not [c for c in rcalls if " install" in f" {c}"])
    rcode, rout, rcalls = run_reinstall(root, wifi)
    expect("REINSTALL.sh on the right hub runs to its end", rcode == 0 and any(" install -r -t -d" in f" {c}"
                                                                              for c in rcalls))
    code, out, calls, _, root, _ = bare_hub(wifi, args, intercept=True, serialno="OTHERHUB0002")
    expect("--expect-serialno, another hub at the address: stops before the backup, nothing installed",
           code != 0 and "a different hub answers at 192.168.43.1:5555 (ro.serialno OTHERHUB0002, not FAKEHUB0001): "
           "nothing was installed" in out and not [c for c in calls if " pull " in f" {c} " or " install" in f" {c}"])
    code, out, calls, _, root, _ = bare_hub(wifi, args, intercept=True, serialno="FAKEHUB0001,OTHERHUB0002")
    expect("--expect-serialno, the hub changing after the backup: stops before the install",
           code != 0 and "ro.serialno OTHERHUB0002" in out and any(" pull " in f" {c} " for c in calls)
           and not [c for c in calls if " install -r -t -g " in f" {c} "])
    code, out, calls, _, root, _ = bare_hub(wifi, ["--expect-serialno", "FAKEHUB0001"], intercept=True)
    expect("--expect-serialno without --install-only: refused before any adb command",
           code != 0 and "goes with --install-only" in out and calls == [])


def build_runs():
    for argv, online, label in ((["--build", "--usb"], True, "online"), (["--usb", "--build"], False, "offline only")):
        code, out, calls, _, root, gradle = bare_hub([], build=argv, gradle_online=online)
        stamp = root / "TeamCode/build/outputs/apk/debug/.hubtest-build"
        expect(f"{' '.join(argv)}, Gradle {label}: stamps the APK and says to run --usb",
               code == 0 and stamp.exists() and "bare_hub.sh --usb" in out and calls == [])
        expect(f"{' '.join(argv)}, Gradle {label}: builds the APK"
               + (" online" if online else ", falling back to the offline cache"),
               gradle == ([":TeamCode:assembleDebug"] if online
                          else [":TeamCode:assembleDebug", "--offline :TeamCode:assembleDebug"]))
    code, out, calls, _, root, gradle = bare_hub([], build=["--build"])
    expect("--build without --usb: still builds online and checks the offline Sloth build",
           code == 0 and gradle == [":TeamCode:assembleDebug", "--offline :TeamCode:assembleSloth"]
           and "join its Wi-Fi" in out)
    code, out, calls, _, root, gradle = bare_hub([], build=["--build", "--usb"], git_works=False)
    expect("--build with a git that won't run: stops before Gradle and writes no stamp",
           code != 0 and "git doesn't run here (exit 69: You have not agreed" in out and gradle == []
           and not (root / "TeamCode/build/outputs/apk/debug/.hubtest-build").exists() and calls == [])
    for argv in (["--usb"], []):
        code, out, calls, py, _, _ = bare_hub([(SERIAL if argv else WIFI_SERIAL, "device")], argv, intercept=True,
                                              git_works=False)
        expect(f"a run{' --usb' if argv else ''} with a git that won't run: stops before any adb command",
               code != 0 and "git doesn't run here (exit 69" in out and calls == [] and py == [])


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--record", default=None)
    p.add_argument("--script", default=None)
    a = p.parse_args()
    if a.record:
        pathlib.Path(a.record).write_text(json.dumps(wifi_record(a.script), indent=1) + "\n")
        print(f"recorded the Wi-Fi run of {a.script or 'this checkout'} in {a.record}")
        return 0
    usb_runs()
    serialno_runs()
    build_runs()
    now, recorded = wifi_record(), json.loads(FIXTURE.read_text())
    for key in recorded:
        expect(f"Wi-Fi run without --usb: its {key} are the recorded ones (fixtures/{FIXTURE.name})",
               now.get(key) == recorded[key])
        if now.get(key) != recorded[key] and isinstance(recorded[key], list):
            diff = next((i for i, (x, y) in enumerate(zip(now[key], recorded[key])) if x != y),
                        min(len(now[key]), len(recorded[key])))
            print(f"  first difference at {diff}: now {now[key][diff:diff + 1]}, recorded {recorded[key][diff:diff + 1]}")
    expect("Wi-Fi run: it connects adb to the hub's Wi-Fi address and checks its port first",
           recorded["python"][:1] == ["tcp 192.168.43.1 5555"] and f"connect {WIFI_SERIAL}" in recorded["adb"])
    print(f"{passed} passed, {len(failed)} failed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
