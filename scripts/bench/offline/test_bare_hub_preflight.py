"""bare_hub.sh's check of adb's server port, run from a copy of the scripts with stub lsof and adb first on PATH, so
nothing reaches a hub or this Mac's adb server: anything but adb on the port must stop it before its first adb call.
Run with: python3 -I scripts/bench/offline/test_bare_hub_preflight.py"""
import os
import pathlib
import shutil
import socket
import subprocess
import sys
import tempfile

HERE = pathlib.Path(__file__).resolve().parent
WORKTREE = HERE.parents[2]
OUT = HERE / "out" / "bare_hub"

passed = 0
failed = []


def expect(label, cond):
    global passed
    if cond:
        passed += 1
    else:
        failed.append(label)
        print("FAIL", label)


def free_port():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    assert port != 5037 and not 5554 <= port <= 5585, port
    return port


def bare_hub(lsof_out, lsof_code=None, **env):
    """Runs the copied bare_hub.sh; returns (exit status, its output, the adb calls, the lsof calls)."""
    OUT.mkdir(parents=True, exist_ok=True)
    root = pathlib.Path(tempfile.mkdtemp(prefix="bare-hub-", dir=OUT))
    (root / "scripts/bench").mkdir(parents=True)
    for name in ("bare_hub.sh", "run_bench.py"):
        shutil.copy(WORKTREE / "scripts/bench" / name, root / "scripts/bench" / name)
    shutil.copytree(WORKTREE / "scripts/bench/benches", root / "scripts/bench/benches",
                    ignore=shutil.ignore_patterns("__pycache__"))
    stubs = root / "stubs"
    stubs.mkdir()
    (stubs / "lsof").write_text('#!/bin/sh\necho "$@" >> "$LSOF_LOG"\nprintf "$LSOF_OUT"\nexit "$LSOF_CODE"\n')
    (stubs / "adb").write_text('#!/bin/sh\necho "$@" >> "$ADB_LOG"\nexit 0\n')
    for stub in ("lsof", "adb"):
        (stubs / stub).chmod(0o755)
    (stubs / "python3").symlink_to(sys.executable)
    adb_log, lsof_log = root / "adb.log", root / "lsof.log"
    run_env = {"PATH": f"{stubs}:/usr/bin:/bin", "HOME": str(root), "LSOF_OUT": lsof_out,
               "LSOF_CODE": str(lsof_code if lsof_code is not None else (0 if lsof_out else 1)),
               "ADB_LOG": str(adb_log), "LSOF_LOG": str(lsof_log), **env}
    r = subprocess.run(["/bin/bash", str(root / "scripts/bench/bare_hub.sh")], env=run_env, capture_output=True,
                       text=True, timeout=60)
    calls = adb_log.read_text().splitlines() if adb_log.exists() else []
    asked = lsof_log.read_text().splitlines() if lsof_log.exists() else []
    return r.returncode, r.stdout + r.stderr, calls, asked


NO_APK = "no single prebuilt APK"

code, out, adb, asked = bare_hub("p4242\\ncssh\\nf5\\n")
expect("ssh on 5037: stops", code != 0 and "STOPPED: ssh (pid 4242) listens on port 5037" in out)
expect("ssh on 5037: before any adb call", adb == [])
expect("ssh on 5037: it asked lsof about 5037's TCP listeners", asked == ["-nP -iTCP:5037 -sTCP:LISTEN -Fcp"])
expect("ssh on 5037: before the build checks", NO_APK not in out)

code, out, adb, asked = bare_hub("p45913\\ncadb\\nf7\\n")
expect("adb on 5037: passes the check and goes on to the build checks",
       "adb's server port: adb pid 45913" in out and NO_APK in out)
expect("adb on 5037: still no adb call before the build checks", adb == [])

port = free_port()
code, out, adb, asked = bare_hub("", ANDROID_ADB_SERVER_PORT=str(port))
expect("nothing on the port: passes", "nothing listening, so adb starts its own server" in out and NO_APK in out)
expect("nothing on the port: the env port is the one checked", asked == [f"-nP -iTCP:{port} -sTCP:LISTEN -Fcp"])

hidden = socket.socket()
hidden.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
hidden.bind(("127.0.0.1", 0))
hidden.listen(1)
try:
    code, out, adb, asked = bare_hub("", ANDROID_ADB_SERVER_PORT=str(hidden.getsockname()[1]))
finally:
    hidden.close()
expect("a listener lsof can't see: stops", code != 0 and "that lsof can't name" in out and adb == [])
expect("a listener lsof can't see: lsof asked twice", len(asked) == 2)

code, out, adb, asked = bare_hub("p45913\\ncadb\\nf7\\n", ADB_SERVER_SOCKET="tcp:192.168.43.1:5037")
expect("ADB_SERVER_SOCKET set: stops before any adb call", code != 0 and "ADB_SERVER_SOCKET is set" in out
       and adb == [])
code, out, adb, asked = bare_hub("", lsof_code=2)
expect("lsof failing: stops", code != 0 and "lsof couldn't list" in out and adb == [])


def reinstall(lsof_out, lsof_code=None, **env):
    """Runs the port check bare_hub.sh writes at the top of REINSTALL.sh, then an adb command, with stub lsof and adb
    first on PATH; returns (exit status, its output, the adb calls, the lsof calls)."""
    OUT.mkdir(parents=True, exist_ok=True)
    root = pathlib.Path(tempfile.mkdtemp(prefix="reinstall-", dir=OUT))
    text = (WORKTREE / "scripts/bench/bare_hub.sh").read_text()
    start = text.index("\nreinstall_preflight() {\n")
    function = text[start:text.index("\n}\n", start) + 3]
    preflight = subprocess.run(["/bin/bash", "-c", function + "reinstall_preflight\n"], capture_output=True, text=True,
                               check=True, timeout=30).stdout
    script = root / "REINSTALL.sh"
    script.write_text("#!/bin/bash\nset -e\n" + preflight + 'adb connect 192.168.43.1:5555\n')
    stubs = root / "stubs"
    stubs.mkdir()
    (stubs / "lsof").write_text('#!/bin/sh\necho "$@" >> "$LSOF_LOG"\nprintf "$LSOF_OUT"\nexit "$LSOF_CODE"\n')
    (stubs / "adb").write_text('#!/bin/sh\necho "$@" >> "$ADB_LOG"\nexit 0\n')
    for stub in ("lsof", "adb"):
        (stubs / stub).chmod(0o755)
    adb_log, lsof_log = root / "adb.log", root / "lsof.log"
    run_env = {"PATH": f"{stubs}:/usr/bin:/bin", "HOME": str(root), "LSOF_OUT": lsof_out,
               "LSOF_CODE": str(lsof_code if lsof_code is not None else (0 if lsof_out else 1)),
               "ADB_LOG": str(adb_log), "LSOF_LOG": str(lsof_log), **env}
    r = subprocess.run(["/bin/bash", str(script)], env=run_env, capture_output=True, text=True, timeout=30)
    calls = adb_log.read_text().splitlines() if adb_log.exists() else []
    asked = lsof_log.read_text().splitlines() if lsof_log.exists() else []
    return r.returncode, r.stdout + r.stderr, calls, asked


text = (WORKTREE / "scripts/bench/bare_hub.sh").read_text()
written = text[text.index("echo '#!/bin/bash'"):text.index('} > "$reinstall"')]
expect("REINSTALL.sh: bare_hub.sh writes the port check before its first adb command",
       0 <= written.find("reinstall_preflight") < written.find('"$ADB"'))
code, out, adb, asked = reinstall("p4242\\ncssh\\nf5\\n")
expect("REINSTALL.sh, ssh on 5037: stops before any adb call, naming it",
       code != 0 and "ssh (pid 4242) listening on port 5037" in out and adb == []
       and asked == ["-nP -iTCP:5037 -sTCP:LISTEN -Fcp"])
code, out, adb, asked = reinstall("p45913\\ncadb\\nf7\\np4242\\ncssh\\nf5\\n")
expect("REINSTALL.sh, adb beside ssh: stops", code != 0 and "ssh (pid 4242)" in out and "adb (pid" not in out
       and adb == [])
code, out, adb, asked = reinstall("p77\\nf3\\n")
expect("REINSTALL.sh, a process lsof doesn't name: stops", code != 0 and "(pid 77)" in out and adb == [])
code, out, adb, asked = reinstall("p45913\\ncadb\\nf7\\n")
expect("REINSTALL.sh, adb on 5037: goes on to adb", code == 0 and adb == ["connect 192.168.43.1:5555"])
code, out, adb, asked = reinstall("", ANDROID_ADB_SERVER_PORT="15037")
expect("REINSTALL.sh, nothing on the env port: goes on, having checked that port",
       code == 0 and adb and asked == ["-nP -iTCP:15037 -sTCP:LISTEN -Fcp"])
code, out, adb, asked = reinstall("p45913\\ncadb\\nf7\\n", ANDROID_ADB_SERVER_ADDRESS="192.168.43.1")
expect("REINSTALL.sh, ANDROID_ADB_SERVER_ADDRESS set: stops", code != 0 and "is set" in out and adb == [])
code, out, adb, asked = reinstall("", lsof_code=2)
expect("REINSTALL.sh, lsof failing: stops", code != 0 and "lsof couldn't list" in out and adb == [])

print(f"{passed} passed, {len(failed)} failed")
sys.exit(1 if failed else 0)
