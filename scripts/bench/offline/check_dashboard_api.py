"""Fails when a dashboard bump renames or changes a member that Bench: Dashboard Off or run_bench.py relies on: javap on
the slothboard build TeamCode pins (TeamCode/libs/m2), plus negative controls on doctored javap text. Needs a JDK's
javap on PATH or in JAVA_HOME. Run with: python3 -I scripts/bench/offline/check_dashboard_api.py"""
import importlib.util
import os
import pathlib
import re
import shutil
import subprocess
import sys
import zipfile

sys.dont_write_bytecode = True
HERE = pathlib.Path(__file__).resolve().parent
WORKTREE = HERE.parents[2]
OUT = HERE / "out" / "dashboard-api"
M2 = WORKTREE / "TeamCode/libs/m2/com/acmerobotics/slothboard"
spec = importlib.util.spec_from_file_location("run_bench", WORKTREE / "scripts/bench/run_bench.py")
rb = importlib.util.module_from_spec(spec)
spec.loader.exec_module(rb)

FTC_DASHBOARD = "com.acmerobotics.dashboard.FtcDashboard"
PROXY = "com.acmerobotics.dashboard.limelight.LimelightProxyManager"
REQUIRED = {
    FTC_DASHBOARD: [
        "public static com.acmerobotics.dashboard.FtcDashboard getInstance();",
        "public boolean isEnabled();",
        "private void disable();",
        "private void setAutoEnable(boolean);",
        "public void sendTelemetryPacket(com.acmerobotics.dashboard.telemetry.TelemetryPacket);",
        "public void startCameraStream(org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource, double);",
        "public void stopCameraStream();",
        f'private static final java.lang.String PREFS_NAME = "{pathlib.PurePosixPath(rb.DASHBOARD_PREFS).stem}";',
        f'private static final java.lang.String PREFS_AUTO_ENABLE_KEY = "{rb.DASHBOARD_AUTO_ENABLE_KEY}";',
        f'private static final java.lang.String TOGGLE_OP_MODE_NAME = "{rb.DASH_TOGGLE_OPMODE}";',
    ],
    "com.acmerobotics.dashboard.RobotStatus": ["private boolean enabled;"],
    "com.acmerobotics.dashboard.telemetry.TelemetryPacket": [
        "public com.acmerobotics.dashboard.telemetry.TelemetryPacket(boolean);",
        "public void put(java.lang.String, java.lang.Object);",
    ],
}

passed = 0
failed = []


def expect(label, cond):
    global passed
    if cond:
        passed += 1
    else:
        failed.append(label)
        print("FAIL", label)


def missing_members(javap_text, required):
    lines = {line.strip() for line in javap_text.splitlines()}
    return [r for r in required if r not in lines]


def method_body(javap_c_text, signature):
    """The Code: lines of one method in `javap -p -c` output, or None if the class has no such method."""
    out, inside = [], False
    for line in javap_c_text.splitlines():
        if line.strip() == signature:
            inside = True
            continue
        if inside:
            if not line.strip():
                break
            out.append(line.strip())
    return out if inside else None


def calls_with_false(body, method):
    """Whether the body calls method:(Z)V right after pushing false (iconst_0)."""
    ops = [line.split(":", 1)[1].strip() if re.match(r"^\d+:", line) else line for line in body or []]
    return any(a.startswith("iconst_0") and b.startswith("invoke") and f"{method}:(Z)V" in b
               for a, b in zip(ops, ops[1:]))


def pushed_shorts(body):
    return {int(m.group(1)) for line in body or [] for m in [re.search(r"\bsipush\s+(\d+)", line)] if m}


def javap():
    found = shutil.which("javap") or (os.environ.get("JAVA_HOME") and str(pathlib.Path(os.environ["JAVA_HOME"]) / "bin/javap"))
    if not found or not pathlib.Path(found).exists():
        print("no javap: install JDK 17 or set JAVA_HOME")
        sys.exit(1)
    return found


def pinned_version():
    m = re.search(r"com\.acmerobotics\.slothboard:dashboard:([^'\"]+)['\"]", (WORKTREE / "TeamCode/build.gradle").read_text())
    return m.group(1) if m else None


# --- negative controls: the checks must catch a doctored dashboard
GOOD = "\n".join("  " + r for r in REQUIRED[FTC_DASHBOARD])
expect("control: the required members pass against themselves", missing_members(GOOD, REQUIRED[FTC_DASHBOARD]) == [])
expect("control: a renamed disable() is caught",
       "private void disable();" in missing_members(GOOD.replace("void disable()", "void disableDashboard()"),
                                                    REQUIRED[FTC_DASHBOARD]))
expect("control: a public disable() is caught",
       missing_members(GOOD.replace("private void disable()", "public void disable()"), REQUIRED[FTC_DASHBOARD]))
expect("control: a renamed prefs file is caught",
       missing_members(GOOD.replace('"FtcDashboard"', '"Dashboard"'), REQUIRED[FTC_DASHBOARD]))
expect("control: a renamed prefs key is caught",
       missing_members(GOOD.replace('"autoEnable"', '"enabled"'), REQUIRED[FTC_DASHBOARD]))
BODY = ["0: aload_0", "1: iconst_0", "2: invokespecial #167                // Method setAutoEnable:(Z)V", "5: return"]
expect("control: disable() saving false passes", calls_with_false(BODY, "setAutoEnable"))
expect("control: disable() saving true is caught",
       not calls_with_false([b.replace("iconst_0", "iconst_1") for b in BODY], "setAutoEnable"))
expect("control: disable() not saving at all is caught", not calls_with_false(BODY[:2] + BODY[3:], "setAutoEnable"))
expect("control: a missing method has no body", method_body("  private void enable();\n    Code:\n", "private void disable();")
       is None)
expect("control: a port moved off 5801 is caught",
       pushed_shorts(["13: sipush        5800", "33: sipush        5802"]) != set(rb.DASH_OFF_PROXY_PORTS))

# --- the pinned build
version = pinned_version()
expect("pinned: TeamCode/build.gradle names a slothboard dashboard build", version is not None)
aar = M2 / "dashboard" / f"{version}" / f"dashboard-{version}.aar"
core = M2 / "core" / f"{version}" / f"core-{version}.jar"
expect(f"pinned: the AAR is in TeamCode/libs/m2 ({aar.name})", aar.exists())
expect(f"pinned: its core jar is beside it ({core.name})", core.exists())
if aar.exists() and core.exists():
    OUT.mkdir(parents=True, exist_ok=True)
    classes = OUT / "classes.jar"
    with zipfile.ZipFile(aar) as z:
        classes.write_bytes(z.read("classes.jar"))
    cp = os.pathsep.join([str(classes), str(core)])
    tool = javap()
    for cls, required in REQUIRED.items():
        text = subprocess.run([tool, "-p", "-constants", "-classpath", cp, cls], capture_output=True, text=True).stdout
        gone = missing_members(text, required)
        expect(f"pinned: {cls} has {len(required)} members the bench uses" + (f"; missing {gone}" if gone else ""),
               bool(text) and not gone)
    code = subprocess.run([tool, "-p", "-c", "-classpath", cp, FTC_DASHBOARD], capture_output=True, text=True).stdout
    expect("pinned: disable() saves autoEnable false (setAutoEnable(false)), so off survives a restart",
           calls_with_false(method_body(code, "private void disable();"), "setAutoEnable"))
    proxy = subprocess.run([tool, "-p", "-c", "-classpath", cp, PROXY], capture_output=True, text=True).stdout
    ports = pushed_shorts(method_body(proxy, "public synchronized void start();"))
    expect(f"pinned: the Limelight proxy binds the ports the runner judges {rb.DASH_OFF_PROXY_PORTS}",
           set(rb.DASH_OFF_PROXY_PORTS) <= ports)

print(f"{passed} passed, {len(failed)} failed")
sys.exit(1 if failed else 0)
