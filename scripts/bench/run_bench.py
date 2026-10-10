#!/usr/bin/env python3
"""Runs the Control Hub bench OpModes (opmodes/test/bench/) over adb and the dashboard websocket, no Driver Station.

    python3 scripts/bench/run_bench.py            # every bench the plugged-in hardware allows
    python3 scripts/bench/run_bench.py --all      # every bench; fails if no webcam, Limelight or Expansion Hub is plugged in
    python3 scripts/bench/run_bench.py --hub --loop --webcam --limelight --llfault --sloth ...

Results land in bench-results/<timestamp>/ at the repo root. See scripts/bench/README.md.
Python 3 standard library only.
"""

import argparse
import base64
import datetime
import hashlib
import json
import math
import os
import re
import shlex
import shutil
import socket
import struct
import subprocess
import sys
import threading
import time
import traceback
import urllib.error
import urllib.request
import xml.etree.ElementTree as ElementTree
import xml.sax.saxutils
from pathlib import Path

HUB_IP = "192.168.43.1"
ADB_PORT = 5555
DASHBOARD_PORT = 8000
RC_PACKAGE = "com.qualcomm.ftcrobotcontroller"
RC_LAUNCHER = f"{RC_PACKAGE}/org.firstinspires.ftc.robotcontroller.internal.PermissionValidatorWrapper"
IDLE_OPMODE = "$Stop$Robot$"
HUB_BENCH_DIR = "/sdcard/FIRST/bench"
PRIVATE_BENCH_DIR = "files/bench"
REPO_ROOT = Path(__file__).resolve().parents[2]

KEY_PHASE = "Bench phase"
KEY_STATUS = "Bench status"
KEY_DONE = "BENCH DONE"

BENCH_OPMODES = [
    "Bench: Probe", "Bench: Hub", "Bench: Framework Loop", "Bench: Raw Loop", "Bench: Failure Path",
    "Bench: Webcam", "Bench: Webcam Stop", "Bench: Limelight", "Bench: Limelight Fault", "Bench: Pinpoint Absent",
    "Bench: Sloth Probe", "Bench: Gamepad", "Bench: Gamepad Linear", "Bench: Write Recovery", "Bench: Drive Handover",
    "Bench: Power Readback", "Bench: Overlay NaN", "Bench: Planner Stop", "Bench: Webcam Thread Failure",
    "Bench: Pinpoint", "Bench: Clock", "Bench: OctoQuad", "Bench: OctoQuad Absent", "Bench: Expansion Hub",
]

CONFIG_PROBE = "bench_probe"
CONFIG_MAIN = "bench_main"
CONFIG_PINPOINT = "bench_pinpoint"
CONFIG_NO_WEBCAM = "bench_nowebcam"
CONFIG_LLFAULT = "bench_llfault"
CONFIG_OCTOQUAD = "bench_octoquad"
CONFIG_OCTOQUAD_ABSENT = "bench_octoquad_absent"
CONFIG_EXPANSION = "bench_expansion"
CONFIG_EXPANSION_ODO = "bench_expansion_odo"
BENCH_CONFIGS = (CONFIG_MAIN, CONFIG_PINPOINT, CONFIG_NO_WEBCAM, CONFIG_LLFAULT, CONFIG_OCTOQUAD, CONFIG_OCTOQUAD_ABSENT,
                 CONFIG_EXPANSION, CONFIG_EXPANSION_ODO, CONFIG_PROBE)
# AppUtil.CONFIG_FILES_DIR; the app keeps the active config's name in its default SharedPreferences.
CONFIG_DIR = "/sdcard/FIRST"
RC_PREFS = f"shared_prefs/{RC_PACKAGE}_preferences.xml"
PREF_ACTIVE_CONFIG = "pref_hardware_config_filename"
DASHBOARD_PREFS = "shared_prefs/FtcDashboard.xml"
NO_CONFIG = "<No Config Set>"
# FtcEventLoop.init logs this once the robot is set up on its config, after every restart.
ROBOT_SETUP_DONE = "======= INIT FINISH ======="
APP_KILLED = "took too long to exit; emergency killing app"
# The Control Hub OS's FtcAccessPointService force-stops and relaunches an RC that stops reporting in for 10 s.
WATCHDOG_KILLED = "has not reported itself as alive"
HUB_PREFS_TMP = "/data/local/tmp/bench_rc_prefs.xml"
PREFS_COPIED = "BENCH_PREFS_COPIED"
LOG_BUFFER = "8M"
RESTART_FAILED = "Something went wrong when reflecting to restart the robot"
ROBOT_FAILED = "Robot failed to start"
CONFIG_SWITCH_TIMEOUT_S = 150
FAKE_WEBCAM_SERIAL = "BenchNoSuchWebcam"

BENCH_MOTOR_PORTS = (0, 1, 2, 3)
BENCH_SERVO_PORTS = (0, 1)
SERVO_PORT_TAGS = {"RevBlinkinLedDriver", "RevSPARKMini"}
SHIPPED_CONFIG_DIR = Path("TeamCode/src/main/res/xml")
BENCH_POWERS = ("the Drivetrain module's creep, up to 0.35 on motor ports 0-3, for about 95 s (Framework Loop and "
                "Limelight Fault; longer with --interactive); the Pedro follower, capped at 0.2, for 8 s each in "
                "Framework Loop and Raw Loop; 0.1 and -0.1 on motor ports 0-3 and 0.4/0.6 on servo port 0 in Bench: Hub; "
                "0.2 on motor ports 0-1 and 0.4 on servo ports 0-1 for 3 s in Write Recovery; a 0.3 stick and the "
                "Pedro follower, capped at 0.2, on motor ports 0-3 for 12 s in Drive Handover; 0.2 on motor port 0 "
                "from the dashboard's Hardware view for 4 s in the hardware-view step")

NO_DS = "no Driver Station connected: no TelemetryMessage serialization or send"

LIMELIGHT_PORT = 5807
FAKE_LIMELIGHT_PORT = 15807
TCP_LISTEN = "0A"
FAKE_LIMELIGHT_PHASES = ("limelightPostLoopback", "limelightPollOff", "limelightPoll100Hz", "limelightPoll200Hz",
                         "limelightPoll250Hz", "limelightFailureModes")

BALL_PIPELINE = 4
LIMELIGHT_PROXY_PENDING = ("expected until slothboard 6165.6: its Limelight proxy forwards session.getUri(), which drops "
                           "the query string, so /pipeline-switch?index=N reaches the Limelight as a bare /pipeline-switch "
                           "while the proxy passes back the Limelight's answer. The dashboard's own Limelight view only "
                           "GETs /status, so this bites the Limelight web UI and REST calls made through the hub's ports")

LIMELIGHT_FAULT_KEY = "Limelight"
COLOR_FAULT = "#ff1744"
# cuttle_decode.xml's Limelight line. With a Limelight plugged in, the fault bench points at TEST-NET-1 (RFC 5737),
# which nothing answers, so the plugged-in one isn't reached.
LIMELIGHT_CONFIG_SERIAL = "EthernetOverUsb:eth0:172.29.0.22"
LIMELIGHT_CONFIG_IP = "172.29.0.1"
UNREACHABLE_IP = "192.0.2.1"
LLFAULT_INIT_HOLD_S = 6.0
LLFAULT_RUN_S = 15.0
LLFAULT_EXPECT_S = 5.0

STUCK_MARKERS = ["was stuck in", "OpMode Force-Stopped", "Begin thread dump", "Robot Controller app restart",
                 "FATAL EXCEPTION", "has died", "Force stopping", APP_KILLED, WATCHDOG_KILLED]

OPMODE_MANAGER_TAG = "OpModeManager"
USER_CODE_THREW = "User code threw an uncaught exception"
EXCEPTION_LINE = re.compile(r"^[\w$.]+(Exception|Error|Throwable)\b")
ERROR_LINE_WAIT_S = 2.0
SILENT_AFTER_S = 5.0
LOGCAT_DONE_EVERY_S = 5.0
REPORT_STARTED = "starting, results in"
CRASH_DETECT_S = 5.0
FAILURE_INJECTED = "BENCH injected failure in "
FAILURE_STUB_B_STOP = "BENCH stub B stop() failure"
ERROR_LINE_PENDING = ("expected until the hub runs the EnhancedOpMode fix that copies a crash to the dashboard's Error "
                      "line, or slothboard 6165.6, which fills the Error line from the crash: SDK 12 only logs "
                      "user-code exceptions")

WRITE_RECOVERY_LIMIT_MS = 500
WRITE_CACHE_PENDING = ("expected until the WriteCache fix that re-sends an unchanged value at least every 250 ms: "
                       "EnhancedMotor and EnhancedServo forward a value only when it changes, so after a hub reset, a "
                       "keep-alive fail-safe or a write the hub dropped, the motor stays disabled and the servo limp "
                       "until the commanded value moves")

HANDOVER_MIN_SAMPLES = 10
HANDOVER_CASES = (("C", "path cancelled from gameLoop"), ("S", "a command sequence turns writes back on"),
                  ("E", "collection-style command cancelled from gameLoop"))
HANDOVER_PENDING = ("expected until the Drivetrain fix that resyncs Pedro's motor cache and its own when it takes the "
                    "motors back: Pedro's IDLE stop() writes 0 to a leg Drivetrain's write cache believes already holds "
                    "the stick, so the motors sit at 0 while the stick is held")

HARDWARE_OPMODE = "Hardware"
HARDWARE_POWER_PATH = ("__hardware__", "Motors", "fl", "Power")
HARDWARE_VIEW_POWER = 0.2
HARDWARE_VIEW_MOVED = 0.01
HARDWARE_VIEW_PENDING = ("expected until slothboard 6165.6: after the built-in Hardware OpMode stops, its Motors tree "
                         "stays in the dashboard's config and a Power edit still calls fl.setPower() from the websocket "
                         "thread under whatever OpMode runs next, so the motor moves behind that OpMode's back (and "
                         "behind EnhancedMotor's write cache, which won't re-send an unchanged 0)")


def config_value(root, path):
    node = root
    for key in path:
        node = dig(node, "__value", key)
        if node is None:
            return None
    return node


def config_diff(path, value):
    node = {"__type": "double", "__value": value}
    for key in reversed(path):
        node = {"__type": "custom", "__value": {key: node}}
    return {"type": "SAVE_CONFIG", "configDiff": node}


BASELINE_CATEGORY = "OptimizationToggles"
BASELINE_PENDING = ("expected until slothboard 6165.6: slothboard records a baseline only from addConfigVariable(), "
                    "which nothing on the robot calls, so the Config view's 'show only modified' filter is always "
                    "empty and no field shows its deployed value")

SLOTH_PROBE_OPMODE = "Bench: Sloth Probe"
DASH_TOGGLE_OPMODE = "Enable/Disable Dashboard"
SLOTH_LOADED = "Processed Sloth Load"
SLOTH_CANCELLED = "Cancelled Sloth Load"
SLOTH_STAGED = "Staged Sloth Load"
SLOTH_FAILED = ("failed to switch loader", "Failed to sloth load")
SLOTH_DEPLOY_MARK = "BENCH sloth deploy"
SLOTH_PUSH_MARK = "BENCH sloth pushed"
SLOTH_DIR = "/sdcard/FIRST/dairy/sloth"
SLOTH_LOCAL_JAR = Path("TeamCode/build/libs/to_load.jar")
SLOTH_JAR = re.compile(r"/(\d+)\.jar")
SLOTH_UPDATE_TIME = re.compile(r"application last update time is (\d+)")
SLOTH_NEWER = re.compile(r"sloth upload is newer (\d+), delta: (-?\d+)")
SLOTH_DROPPED = ("removing outdated sloth load", "removing old sloth uploads due to application hash change")
SLOTH_LOAD_PENDING = ("a hot load ended Cancelled or failed, or logged nothing within the wait, although deploySloth "
                      "reported success: the hub kept running the code from before the push, and without a Driver "
                      "Station only a failed load says so (on the dashboard's Error line); Sloth reports a landed or "
                      "cancelled load only as an RC-screen toast and in logcat. No fix is in flight")
SLOTH_PERSIST_PENDING = ("the app restart dropped the hot-loaded jar (Sloth keeps it only if its mtime is newer than the "
                         "package's lastUpdateTime, and deletes it as outdated otherwise), so after any config switch, "
                         "watchdog relaunch or crash the hub silently runs the installed APK's code until the next "
                         "deploySloth; the jar-clock line says which clock stamped it. No fix is in flight")
SLOTH_LOAD_WAIT_S = 30
SLOTH_LIST_SETTLE_S = 3
SLOTH_ON_OPEN_S = 1.0
SLOTH_LIST_PENDING = ("expected until slothboard 6165.6: while a hot load swaps TeamCode, slothboard broadcasts an "
                      "OpMode list holding only its own OpModes ('Enable/Disable Dashboard' and 'Hardware'), the "
                      "dashboard's picker falls to 'Enable/Disable Dashboard', and running that disables the dashboard "
                      "on a hub with no Driver Station to turn it back on; every list also offers that OpMode. Runs "
                      "where the app restarted or no load reached logcat aren't judged, and with none judged this "
                      "FAILs without saying anything about slothboard")

FLOODGATE_THRESHOLD_A = 25.0
FLOODGATE_LIMIT_V = FLOODGATE_THRESHOLD_A / 80.0 * 3.3
KEY_FLOODGATE_AMPS = "Drivetrain Floodgate Current (A)"
KEY_LIMITER = "Drivetrain Current Limiter Multiplier"
FLOODGATE_PENDING = ("the floodgate pin with nothing wired to it (Control Hub analog 0, as on the Betta bot, whose "
                     f"Floodgate isn't wired) reads {FLOODGATE_LIMIT_V:.2f} V or more, which Drivetrain's current limiter, "
                     f"on by default, takes for {FLOODGATE_THRESHOLD_A:.0f} A or more, so it scales every teleop drive power "
                     "down; until a Floodgate is wired, turn Drivetrain.currentLimiterConfig.enabled off for the Betta "
                     "bot. No fix is in flight: this run decides whether one is needed")

CAPTURE_CLOCK_PENDING = ("EasyOpenCV's captureTimeNanos isn't on System.nanoTime()'s clock: a negative age means the capture "
                         "clock runs ahead, so CellTipCamera's one-sided freshness gate (staleness <= maxStalenessMs) stays "
                         "open and a frozen webcam keeps reporting its last verdict (make the gate two-sided, 0 <= "
                         "staleness <= maxStalenessMs); an age past maxStalenessMs (a drifting or offset clock, another "
                         "epoch, or a stalled camera thread) makes every verdict read stale, since a verdict is only older "
                         "by the time the OpMode reads it. No fix is in flight")

WEBCAM_FAILURE_FRAME = 30
WEBCAM_FAILURE_PENDING = ("expected until the WebcamSession fix that catches a pipeline exception on the camera thread and "
                          "rethrows it from update() on the OpMode thread, or slothboard 6165.6, which puts the crash text "
                          "in errorMessage: EasyOpenCV catches the exception itself, stops the OpMode as if STOP were "
                          "pressed (emulateEStop) and logs it only under the OpenCvCamera tag, which slothboard's "
                          "OpModeManager monitor doesn't forward, so the dashboard shows the OpMode stopping with no cause")

KEY_PINPOINT_STATUS = "Bench pinpoint status"
PINPOINT_INIT_HOLD_S = 2.0
PINPOINT_SEED_OFFSET_IN = 0.1
PINPOINT_SEED_TURN_DEG = 0.2
PINPOINT_SEED_PENDING = ("a start pose set in initialize() didn't survive the Pinpoint's IMU recalibration at INIT: Close "
                         "Flower Auto, Mock Auto and the Linear autos set the pose only in INIT, so they start their first "
                         "path from a pose the robot isn't at; they need the START re-seed Ball Collection Auto does, or a "
                         "framework re-seed at START. A PASS means only that the INIT sequence the autos use keeps the "
                         "pose; a pose set at other delays after the recalibration isn't tried. No fix is in flight")
PINPOINT_CRASHED = ("Bench: Pinpoint crashed (see the error): with a Pinpoint status check in BettaConstants or the "
                    "framework, a status other than READY at INIT (CALIBRATING right after create, FAULT_NO_PODS_DETECTED "
                    "without both pods) stops every OpMode, which the check must allow for")
PINPOINT_DROPOUT_PENDING = ("expected until the Pinpoint status fix: on a failed I2C read the SDK driver keeps the last pose "
                            "and velocity (firmware v1/v2, FAULT_BAD_READ) or zeroes them (v3+, NOT_READY), and only its "
                            "status says so, which nothing in Pedro or the framework reads, so the follower drives on a "
                            "frozen or (0, 0, 0) pose with no fault and no error")

KEY_OCTOQUAD_STATUS = "Bench octoquad status"
KEY_OCTOQUAD_VALID = "Bench octoquad valid"
OCTOQUAD_BUS = 2
OCTOQUAD_ABSENT_BUS = 3
OCTOQUAD_ABSENT_WAIT_S = 10
OCTOQUAD_ABSENT_EXPECT_S = 5
OCTOQUAD_SEED_PENDING = ("a start pose set in initialize() moved by START + 3 s on Pedro's OctoQuadLocalizer with the "
                         "OctoQuad at rest: Pedro's setPose sends whole millimetres (under 0.04 in lost), so more than that "
                         "means the OctoQuad didn't keep the pose setLocalizerPose sent or its IMU heading drifted at rest, "
                         "and the autos that set the pose only in INIT would start from a pose the robot isn't at. A PASS "
                         "means only that this INIT sequence keeps the pose. No fix is in flight")
OCTOQUAD_VALID_PENDING = ("a localizer read failed its CRC or reported a status other than RUNNING with the OctoQuad plugged "
                          "in and at rest: Pedro's OctoQuadLocalizer skips such a read without a word and keeps the last "
                          "pose, so on the robot those loops drive on a stale pose; statusChanges says when and which "
                          "status. No fix is in flight")
OCTOQUAD_CRASHED = ("Bench: OctoQuad crashed (see the error); at INIT that means Pedro's OctoQuadLocalizer constructor, "
                    "or the bench's own setup, threw while the follower was built")
OCTOQUAD_HUNG = ("Bench: OctoQuad never finished INIT: Pedro's OctoQuadLocalizer constructor waits with no timeout for the "
                 "OctoQuad to report RUNNING, so one whose IMU never finishes calibrating, or an MK1 with no IMU "
                 "(FAULT_NO_IMU), hangs INIT with no message; the runner's STOP got it out (see the step's log)")
OCTOQUAD_DROPOUT_PENDING = ("Pedro's OctoQuadLocalizer drops a read that fails its CRC or isn't RUNNING without a word and "
                            "keeps the last pose, and nothing in the framework reads the OctoQuad's status, so with its "
                            "cable out the follower drives on a frozen pose with no fault and no error. No fix is in flight")
OCTOQUAD_ABSENT_PENDING = ("Pedro's OctoQuadLocalizer constructor waits for the OctoQuad to report RUNNING in a loop with "
                           "no timeout and no STOP check, so with nothing answering on its I2C port (an unplugged or loose "
                           "cable), or an MK1 that reports FAULT_NO_IMU, INIT hangs with no message until STOP; the STOP "
                           "line says how the SDK got out of it. No fix is in flight")
OCTOQUAD_ABSENT_RETURNED = ("Pedro's OctoQuadLocalizer was built with nothing on the bus and INIT carried on with no "
                            "error: its update() keeps the last pose whenever a read isn't valid, so a robot with its "
                            "OctoQuad unplugged would sit on its start pose with no fault")

EXPANSION_MAX_ADDRESS = 10
EXPANSION_COMMANDS = ("inputVoltage", "bulkData", "current")
EXPANSION_LOOP_PASSES = ("controlHubBulk", "bothHubsBulk", "bothHubsBulkPlusOdometry")
EXPANSION_MOVE = (">>> MOVE the Pinpoint's I2C cable to the Expansion Hub's I2C port 1 and the OctoQuad's to the "
                  "Expansion Hub's I2C port 2 (pods stay in), then press Enter.")
EXPANSION_HOME = "the Pinpoint back to Control Hub I2C port 1 and the OctoQuad back to Control Hub I2C port 2"

CLOCK_TIMEOUT_MS = 15000
CLOCK_INTERACTIVE_TIMEOUT_MS = 40000
CLOCK_TOLERANCE_MS = 250
CLOCK_UNSET_YEAR = 1975
CLOCK_PENDING = ("Ivy's Commands.waitMs times on System.currentTimeMillis(), and the hub's wall clock jumped while it ran "
                 "(a Driver Station connecting, or the RC's web page setting a clock left near 1970 since boot, moves it "
                 "about 56 years), so PathCommands.timeout ended early or late; every PathCommands.timeout, Mock Auto "
                 "pause and checkTip timeout in flight does the same. With --interactive on a hub whose clock was unset "
                 "this FAIL is the expected demonstration; it stays until TeamCode's waits time on System.nanoTime(). "
                 "No fix is in flight")

KEY_PLANS = "Bench plans"
PLANNER_STOP_BALLS = (4, 5)
PLANNER_STOP_DELAYS_S = (0.05, 0.3, 0.6)
PLANNER_STOP_WARM = 3
PLANNER_STOP_WARM_WAIT_S = 180
PLANNER_STOP_SEEN = re.compile(r"planner_stop (\S+) STOP seen (?:between plans|([\d.]+) ms into plan (\d+))")
SDK_STOP_WINDOW_MS = 900
PLANNER_STOP_PENDING = ("STOP waited for a route plan running on the OpMode thread, the plan and stop() together outlasted "
                        f"the SDK's {SDK_STOP_WINDOW_MS} ms stop window, and the SDK restarted the Robot Controller app, "
                        "losing dashboard @Config edits and the alliance: Vision Ball Collection and Ball Collection plan "
                        "on the OpMode thread. No fix is in flight; the plan times decide between a time budget in "
                        "RouteOptimizer, fewer balls, or planning on a worker thread")
PLANNER_JUDGED_BALLS = 4
PLANNER_PLAN_P99_MS = 500
PLANNER_HUB_MAX_MS = 800
PLANNER_BUDGET_PENDING = ("a route plan on the OpMode thread takes long enough that a STOP landing during it leaves stop() "
                          f"little or none of the SDK's {SDK_STOP_WINDOW_MS} ms stop window: Vision Ball Collection and "
                          "Ball Collection plan there. No fix is in flight; these numbers decide between a time budget in "
                          "RouteOptimizer, fewer balls, or planning on a worker thread")

OVERLAY_NAN_PENDING = ("expected until slothboard 6165.6: one NaN or infinite field-overlay coordinate makes "
                       "slothboard's telemetry thread throw while it serializes a batch, which ends dashboard telemetry "
                       "for every OpMode until the app restarts (the runner restarts it after this step)")

GAMEPAD_FIELDS = {
    "left_stick_x": 0.0, "left_stick_y": 0.0, "right_stick_x": 0.0, "right_stick_y": 0.0,
    "dpad_up": False, "dpad_down": False, "dpad_left": False, "dpad_right": False,
    "a": False, "b": False, "x": False, "y": False, "guide": False, "start": False, "back": False,
    "left_bumper": False, "right_bumper": False, "left_stick_button": False, "right_stick_button": False,
    "left_trigger": 0.0, "right_trigger": 0.0, "touchpad": False,
}
GP_INIT_DELIVERY, GP_INIT_IDLE, GP_RUN_DELIVERY, GP_RUN_IDLE, GP_HOLD, GP_HOLD_IDLE = 1, 2, 3, 4, 5, 6
GP_WATCHDOG, GP_WATCHDOG_IDLE, GP_SECOND, GP_SECOND_IDLE = 7, 8, 9, 10
GP_KEYBOARD, GP_SCREEN, GP_USB, GP_INTERACTIVE_IDLE = 11, 12, 13, 14
GP_DONE = 99
GP_DELIVERY_LY = -0.5
GP_DELIVERY_S = 1.5
GP_SEND_EVERY_S = 0.05
GP_RESEND_S = 0.15
GP_HOLD_S = 2.0
GP_WATCHDOG_HOLD_S = 1.0
GP_WATCHDOG_SILENCE_S = 1.5
GP_WATCHDOG_LIMIT_MS = 1000
GP_SECOND_S = 3.0
GP_SECOND_REST_EVERY_S = 0.2
GP_SECOND_OFFSET_S = 0.07
GP_VARIANTS = (("Bench: Gamepad", "gamepad", "iterative"), ("Bench: Gamepad Linear", "gamepad_linear", "linear"))
GP_NOT_DELIVERED = {
    "iterative": ("dashboard gamepads don't reach an iterative OpMode: expected only on slothboard before 6165.5, "
                  "which writes gamepad1 directly and the SDK copies its own state over it before every loop"),
    "linear": "the runner's RECEIVE_GAMEPAD_STATE never reached the OpMode (websocket or slothboard trouble)",
}


GP_SIGN_PENDING = ("expected until slothboard 6165.6, whose client sends up as -1 like a USB pad and the Driver "
                   "Station; before it, keyboard W and the on-screen stick send +1, so every OpMode's -left_stick_y "
                   "drives backward. No loop past 0.9 either way means the input never arrived (keyboard input off, "
                   "the page without focus, or the stick not dragged to the edge)")
GP_INTERACTIVE = (
    (GP_KEYBOARD, "keyboard W",
     "In the Gamepad view turn keyboard input on and click the page, with no USB gamepad plugged in. Hold W for 3 s "
     "and let go", GP_SIGN_PENDING),
    (GP_SCREEN, "on-screen stick up",
     "Drag the Gamepad view's on-screen left stick to its top edge, hold it there for 3 s and let go", GP_SIGN_PENDING),
    (GP_USB, "USB pad forward with the Gamepad view open",
     "Plug a USB gamepad into the Mac and press Start+A to bind it as gamepad 1, with the Gamepad view still open. "
     "Hold its left stick fully forward for 3 s and let go",
     "expected until slothboard 6165.6: with the Gamepad view open, its 100 ms keepalive sends a rest state between "
     "the bound pad's frames, so a held stick reads 0 and a held button presses again. No loop past -0.9 means the "
     "pad wasn't bound with Start+A or the stick didn't reach full forward"),
)


def gamepad_message(phase=0, seq=0, **pad1):
    marker = {"right_trigger": phase / 100, "left_trigger": (seq % 997 + 1) / 1000} if phase else {}
    return {"type": "RECEIVE_GAMEPAD_STATE", "gamepad1": dict(GAMEPAD_FIELDS, **pad1),
            "gamepad2": dict(GAMEPAD_FIELDS, **marker)}


def telemetry_float(value):
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


class BenchError(Exception):
    """A failure the user can act on; printed without a traceback."""


class ProtocolError(BenchError):
    pass


class ConnectionClosed(Exception):
    pass


# ---------------------------------------------------------------------------------------------------------------------
# RFC 6455 client
# ---------------------------------------------------------------------------------------------------------------------

class WebSocket:
    GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
    OP_CONT, OP_TEXT, OP_BINARY, OP_CLOSE, OP_PING, OP_PONG = 0x0, 0x1, 0x2, 0x8, 0x9, 0xA

    def __init__(self, host, port, path="/", timeout=5.0):
        self.sock = socket.create_connection((host, port), timeout=timeout)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.send_lock = threading.Lock()
        self.buf = b""
        self.last_pong = time.monotonic()
        key = base64.b64encode(os.urandom(16)).decode("ascii")
        request = (f"GET {path} HTTP/1.1\r\nHost: {host}:{port}\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n"
                   f"Sec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n\r\n")
        self.sock.sendall(request.encode("ascii"))
        response = b""
        while b"\r\n\r\n" not in response:
            chunk = self.sock.recv(4096)
            if not chunk:
                raise ProtocolError("websocket server closed the connection during the handshake")
            response += chunk
            if len(response) > 65536:
                raise ProtocolError("websocket handshake response is too long")
        head, self.buf = response.split(b"\r\n\r\n", 1)
        lines = head.decode("latin-1").split("\r\n")
        if len(lines[0].split()) < 2 or lines[0].split()[1] != "101":
            raise ProtocolError(f"websocket handshake refused: {lines[0]!r}")
        headers = {}
        for line in lines[1:]:
            name, _, value = line.partition(":")
            headers[name.strip().lower()] = value.strip()
        expected = base64.b64encode(hashlib.sha1((key + self.GUID).encode("ascii")).digest()).decode("ascii")
        if headers.get("upgrade", "").lower() != "websocket":
            raise ProtocolError(f"handshake reply has Upgrade {headers.get('upgrade')!r}, not websocket")
        if headers.get("sec-websocket-accept") != expected:
            raise ProtocolError("handshake reply has the wrong Sec-WebSocket-Accept")
        self.sock.settimeout(None)

    def _read_exact(self, n):
        while len(self.buf) < n:
            chunk = self.sock.recv(max(65536, n - len(self.buf)))
            if not chunk:
                raise ConnectionClosed("server closed the TCP connection")
            self.buf += chunk
        out, self.buf = self.buf[:n], self.buf[n:]
        return out

    def _read_frame(self):
        b0, b1 = self._read_exact(2)
        fin = bool(b0 & 0x80)
        if b0 & 0x70:
            raise ProtocolError(f"server frame has reserved bits set (0x{b0:02x})")
        opcode = b0 & 0x0F
        if opcode not in (self.OP_CONT, self.OP_TEXT, self.OP_BINARY, self.OP_CLOSE, self.OP_PING, self.OP_PONG):
            raise ProtocolError(f"server frame has unknown opcode {opcode}")
        if b1 & 0x80:
            raise ProtocolError("server frame is masked, which RFC 6455 forbids")
        length = b1 & 0x7F
        if length == 126:
            (length,) = struct.unpack("!H", self._read_exact(2))
        elif length == 127:
            (length,) = struct.unpack("!Q", self._read_exact(8))
            if length >> 63:
                raise ProtocolError("server frame length has its top bit set")
        if opcode >= 0x8 and (not fin or length > 125):
            raise ProtocolError("server sent a fragmented or oversized control frame")
        return fin, opcode, self._read_exact(length)

    def recv_message(self):
        """Returns (opcode, payload) for the next text or binary message; answers pings and closes itself."""
        message_opcode = None
        parts = []
        while True:
            fin, opcode, payload = self._read_frame()
            if opcode == self.OP_PING:
                self._send_frame(self.OP_PONG, payload)
                continue
            if opcode == self.OP_PONG:
                self.last_pong = time.monotonic()
                continue
            if opcode == self.OP_CLOSE:
                code = struct.unpack("!H", payload[:2])[0] if len(payload) >= 2 else 1005
                reason = payload[2:].decode("utf-8", "replace")
                try:
                    self._send_frame(self.OP_CLOSE, payload[:2])
                except OSError:
                    pass  # The peer may already be gone; the close is reported either way.
                raise ConnectionClosed(f"server closed the websocket ({code} {reason})")
            if opcode == self.OP_CONT:
                if message_opcode is None:
                    raise ProtocolError("continuation frame with no message started")
            else:
                if message_opcode is not None:
                    raise ProtocolError("new data frame inside a fragmented message")
                message_opcode = opcode
            parts.append(payload)
            if fin:
                return message_opcode, b"".join(parts)

    @staticmethod
    def mask(payload, key):
        n = len(payload)
        if n == 0:
            return b""
        repeated = (key * (n // 4 + 1))[:n]
        return (int.from_bytes(payload, "big") ^ int.from_bytes(repeated, "big")).to_bytes(n, "big")

    @staticmethod
    def encode_frame(opcode, payload, fin=True, key=None):
        key = os.urandom(4) if key is None else key
        header = bytes([(0x80 if fin else 0) | opcode])
        n = len(payload)
        if n < 126:
            header += bytes([0x80 | n])
        elif n < 65536:
            header += bytes([0x80 | 126]) + struct.pack("!H", n)
        else:
            header += bytes([0x80 | 127]) + struct.pack("!Q", n)
        return header + key + WebSocket.mask(payload, key)

    def _send_frame(self, opcode, payload, fin=True):
        frame = self.encode_frame(opcode, payload, fin)
        with self.send_lock:
            self.sock.sendall(frame)

    def send_text(self, text):
        self._send_frame(self.OP_TEXT, text.encode("utf-8"))

    def send_ping(self, payload=b"bench"):
        self._send_frame(self.OP_PING, payload)

    def close(self):
        try:
            self._send_frame(self.OP_CLOSE, struct.pack("!H", 1000))
        except OSError:
            pass  # Closing a socket that already dropped.
        try:
            self.sock.close()
        except OSError:
            pass


# ---------------------------------------------------------------------------------------------------------------------
# slothboard client
# ---------------------------------------------------------------------------------------------------------------------

KNOWN_SERVER_TYPES = {
    "RECEIVE_ROBOT_STATUS", "RECEIVE_OP_MODE_LIST", "RECEIVE_CONFIG", "RECEIVE_CONFIG_BASELINE", "RECEIVE_TELEMETRY",
    "RECEIVE_IMAGE", "RECEIVE_HARDWARE_CONFIG_LIST", "RECEIVE_LOGCAT_ERRORS", "RECEIVE_LOGCAT_LINES",
    "RECEIVE_GAMEPAD_STATE",
}


class Dashboard:
    """slothboard's websocket (port 8000): OpMode control, hardware configs, and the telemetry the benches report on."""

    def __init__(self, host, port, log):
        self.host, self.port, self.log = host, port, log
        self.cond = threading.Condition()
        self.closing = False
        self.ws = None
        self.connected = False
        self.fatal = None
        # Off only while restoring the config: an unknown message type is then logged instead of stopping the restore.
        self.strict = True
        self.disconnects = 0
        self.status = None
        self.status_seq = 0
        self.seen_active = set()
        self.opmodes = None
        self.opmode_seq = 0
        self.config_root = None
        self.config_seq = 0
        self.config_baseline = None
        self.baseline_seq = 0
        self.connection = 0
        self.connected_at = {}
        self.opmode_lists = []
        self.data = {}
        self.done_line = None
        self.last_telemetry = 0.0
        self.opmode_log = []
        self.opmode_log_at = []
        self.recording = None
        # Called on the reader thread with (monotonic time, packet) for every telemetry packet; must be quick.
        self.packet_hook = None
        self.connect(timeout=10)
        threading.Thread(target=self._keepalive, name="bench-ws-keepalive", daemon=True).start()

    def connect(self, timeout):
        deadline = time.monotonic() + timeout
        last_error = None
        while time.monotonic() < deadline:
            try:
                ws = WebSocket(self.host, self.port)
            except OSError as e:
                last_error = e
                time.sleep(1)
                continue
            with self.cond:
                self.ws = ws
                self.connected = True
                self.connection += 1
                connection = self.connection
                self.connected_at[connection] = time.monotonic()
                self.cond.notify_all()
            threading.Thread(target=self._reader, args=(ws, connection), name="bench-ws-reader", daemon=True).start()
            return
        raise BenchError(
            f"Can't open the dashboard websocket at ws://{self.host}:{self.port}/ ({last_error}). Join the Control Hub's "
            "Wi-Fi network, make sure the Robot Controller app is running, and that the dashboard is enabled "
            "(RC app menu → Enable Dashboard).")

    def _reader(self, ws, connection):
        try:
            while True:
                opcode, payload = ws.recv_message()
                if opcode != WebSocket.OP_TEXT:
                    raise ProtocolError("slothboard sent a binary websocket message")
                try:
                    msg = json.loads(payload.decode("utf-8"))
                except ValueError as e:
                    raise ProtocolError(f"slothboard sent a message that isn't JSON: {e}")
                self._handle(msg, len(payload), connection)
        except (ConnectionClosed, OSError) as e:
            with self.cond:
                if self.ws is ws:
                    self.connected = False
                    self.disconnects += 1
                    if not self.closing:
                        self.log(f"dashboard websocket dropped: {e}")
                self.cond.notify_all()
        except BenchError as e:
            with self.cond:
                self.fatal = e
                self.cond.notify_all()

    def _keepalive(self):
        # NanoHTTPD closes a socket that stays silent for 5 s; pings keep it open without GET_ROBOT_STATUS's hub reads.
        while True:
            time.sleep(1.5)
            ws = self.ws
            if ws is not None and self.connected:
                try:
                    ws.send_ping()
                except OSError:
                    pass  # The reader thread reports the drop.

    def _handle(self, msg, size, connection):
        kind = msg.get("type") if isinstance(msg, dict) else None
        if kind is None:
            raise ProtocolError(f"slothboard message without a type: {str(msg)[:200]}")
        if kind not in KNOWN_SERVER_TYPES:
            if self.strict:
                raise ProtocolError(f"slothboard sent an unexpected message type {kind}; the runner needs updating")
            self.log(f"ignoring unexpected slothboard message type {kind} while restoring")
            return
        now = time.monotonic()
        with self.cond:
            if kind == "RECEIVE_ROBOT_STATUS":
                self.status = msg["status"]
                self.status_seq += 1
                self.seen_active.add(self.status["activeOpMode"])
            elif kind == "RECEIVE_OP_MODE_LIST":
                self.opmodes = [(o["name"], o.get("group")) for o in msg["opModeInfoList"]]
                self.opmode_seq += 1
                self.opmode_lists.append((now, connection, now - self.connected_at[connection],
                                          [name for name, _ in self.opmodes]))
            elif kind == "RECEIVE_CONFIG":
                self.config_root = msg.get("configRoot")
                self.config_seq += 1
            elif kind == "RECEIVE_CONFIG_BASELINE":
                self.config_baseline = msg.get("configBaseline")
                self.baseline_seq += 1
            elif kind == "RECEIVE_TELEMETRY":
                packets = msg["telemetry"]
                if not packets:
                    self.data = {}
                for p in packets:
                    if self.packet_hook is not None:
                        self.packet_hook(now, p)
                    data = p.get("data") or {}
                    self.data.update(data)
                    if KEY_DONE in data:
                        self.done_line = data[KEY_DONE]
                    if self.recording is not None:
                        self.recording["packets"].append((now, data.get(KEY_PHASE, self.data.get(KEY_PHASE)),
                                                          data.get("Bench loop"), size / max(1, len(packets))))
                if self.recording is not None:
                    self.recording["messages"] += 1
                    self.recording["bytes"] += size
                self.last_telemetry = now
            elif kind == "RECEIVE_IMAGE":
                if self.recording is not None:
                    self.recording["images"].append((now, self.data.get(KEY_PHASE), size))
            elif kind == "RECEIVE_LOGCAT_ERRORS":
                entries = [e for e in (msg.get("errors") or []) if e.get("tag") == OPMODE_MANAGER_TAG]
                self.opmode_log += entries
                self.opmode_log_at += [now] * len(entries)
            self.cond.notify_all()

    def check(self):
        if self.fatal is not None:
            raise self.fatal

    def recover_for_restore(self):
        """A protocol error kills the reader thread; the restore still needs a working socket, so open a fresh one."""
        with self.cond:
            self.strict = False
            if self.fatal is None:
                return
            self.log(f"reconnecting after '{self.fatal}' to restore the config")
            self.fatal = None
            old, self.ws, self.connected = self.ws, None, False
        if old is not None:
            old.close()
        self.connect(timeout=60)

    def ensure_connected(self, timeout=60):
        self.check()
        if not self.connected:
            self.log("reconnecting to the dashboard")
            self.connect(timeout)

    def send(self, obj):
        self.ensure_connected()
        try:
            self.ws.send_text(json.dumps(obj))
        except OSError as e:
            # The app can die under the socket (a config switch can get it killed) before the reader notices.
            self.log(f"dashboard send failed ({e}); reconnecting")
            with self.cond:
                old, self.ws, self.connected = self.ws, None, False
            old.close()
            self.connect(timeout=60)
            self.ws.send_text(json.dumps(obj))

    def fresh_active_config(self, timeout=10):
        """The active config from the list slothboard sends a socket as it opens, which reads it from the app at that
        moment; None if no list comes (slothboard skips it until it has read the configs after a restart)."""
        deadline = time.monotonic() + timeout
        ws = WebSocket(self.host, self.port)
        try:
            while True:
                left = deadline - time.monotonic()
                if left <= 0:
                    return None
                ws.sock.settimeout(left)
                try:
                    opcode, payload = ws.recv_message()
                except (socket.timeout, ConnectionClosed):
                    return None
                msg = json.loads(payload.decode("utf-8")) if opcode == WebSocket.OP_TEXT else {}
                if msg.get("type") == "RECEIVE_HARDWARE_CONFIG_LIST":
                    return msg["currentHardwareConfig"]
        finally:
            ws.close()

    def wait_until(self, predicate, timeout, what, poll_status_every=None, tick=None):
        deadline = time.monotonic() + timeout
        next_poll = 0.0
        while True:
            self.check()
            with self.cond:
                if predicate():
                    return
            if tick is not None:
                tick()
            now = time.monotonic()
            if now > deadline:
                raise BenchError(f"timed out after {timeout:.0f} s waiting for {what}")
            if not self.connected:
                self.ensure_connected(timeout=max(1.0, deadline - now))
                continue
            if poll_status_every is not None and now >= next_poll:
                self.send({"type": "GET_ROBOT_STATUS"})
                next_poll = now + poll_status_every
            with self.cond:
                self.cond.wait(0.2)

    def get_status(self, timeout=10):
        seq = self.status_seq
        self.send({"type": "GET_ROBOT_STATUS"})
        self.wait_until(lambda: self.status_seq > seq, timeout, "a robot status reply", poll_status_every=1.0)
        return self.status

    def close(self):
        self.closing = True
        if self.ws is not None:
            self.ws.close()

    def reset_telemetry(self):
        with self.cond:
            self.data = {}
            self.done_line = None

    def start_recording(self):
        with self.cond:
            self.recording = {"start": time.monotonic(), "packets": [], "images": [], "messages": 0, "bytes": 0}

    def stop_recording(self):
        with self.cond:
            rec, self.recording = self.recording, None
        rec["end"] = time.monotonic()
        return rec


class SideSocket:
    def __init__(self, host, port):
        self.ws = WebSocket(host, port)
        threading.Thread(target=self._drain, name="bench-side-socket", daemon=True).start()

    def _drain(self):
        try:
            while True:
                self.ws.recv_message()
        except (ConnectionClosed, OSError):
            return

    def send(self, obj):
        self.ws.send_text(json.dumps(obj))

    def close(self):
        self.ws.close()


class FaultWatch:
    """What the dashboard websocket shows of one fault key: whether its FAULT row leads each packet the bench OpMode
    sent (the ones carrying its loop counter), whether the field overlay draws it red, and when it came and went."""

    def __init__(self, key, loop_key="Bench loop"):
        self.caption = f"FAULT {key}"
        self.loop_key = loop_key
        self.start = time.monotonic()
        self.active = False
        self.message = None
        self.transitions = []
        self.bench_packets = 0
        self.with_fault = 0
        self.fault_first = 0
        self.fault_not_first = []
        self.without_fault_after_first = 0
        self.overlays = 0
        self.overlays_after_first = 0
        self.overlays_with_text = 0
        self.overlays_red = 0
        self.samples = []

    def on_packet(self, now, packet):
        data = packet.get("data") or {}
        ops = (packet.get("fieldOverlay") or {}).get("ops") or []
        if ops:
            self.overlays += 1
            if self.transitions:
                self.overlays_after_first += 1
            fill = alpha = None
            for op in ops:
                kind = op.get("type")
                if kind == "fill":
                    fill = op.get("color")
                elif kind == "alpha":
                    alpha = op.get("alpha")
                elif kind == "text" and str(op.get("text", "")).startswith(self.caption + ":"):
                    self.overlays_with_text += 1
                    if fill == COLOR_FAULT and alpha == 1:
                        self.overlays_red += 1
                    break
        if self.loop_key not in data:
            return
        self.bench_packets += 1
        items = packet.get("items") or []
        captions = [i.get("caption") for i in items]
        present = self.caption in captions
        if present:
            self.with_fault += 1
            index = captions.index(self.caption)
            leading = all(str(c).startswith("FAULT ") for c in captions[:index + 1])
            if leading:
                self.fault_first += 1
            elif len(self.fault_not_first) < 5:
                self.fault_not_first.append(captions[:index + 1])
            message = items[index].get("value")
            if len(self.samples) < 3:
                self.samples.append({"items": items[:6],
                                     "overlayFaultOps": [o for o in ops if o.get("type") in ("fill", "alpha", "text")][-6:]})
        else:
            message = None
            if self.transitions:
                self.without_fault_after_first += 1
        if present != self.active or (present and message != self.message):
            what = "update" if present and self.active else ("raise" if present else "clear")
            self.transitions.append({"s": round(now - self.start, 3), "what": what, "message": message})
        self.active, self.message = present, message

    def seconds_now(self):
        return time.monotonic() - self.start

    def summary(self):
        return {k: v for k, v in self.__dict__.items() if k not in ("start",)}


# ---------------------------------------------------------------------------------------------------------------------
# adb
# ---------------------------------------------------------------------------------------------------------------------

def find_adb():
    found = shutil.which("adb")
    if found:
        return found
    for candidate in [Path.home() / "Library/Android/sdk/platform-tools/adb",
                      Path(os.environ.get("ANDROID_HOME", "/nonexistent")) / "platform-tools/adb",
                      Path(os.environ.get("ANDROID_SDK_ROOT", "/nonexistent")) / "platform-tools/adb"]:
        if candidate.exists():
            return str(candidate)
    raise BenchError("adb isn't on PATH or in ~/Library/Android/sdk/platform-tools. Install Android Studio's "
                     "platform-tools or add them to PATH.")


class Adb:
    def __init__(self, serial, log):
        self.path = find_adb()
        self.serial = serial
        self.log = log
        self.private = False

    def _raw(self, args, timeout=60):
        return subprocess.run([self.path] + args, capture_output=True, text=True, timeout=timeout)

    def run(self, *args, check=True, timeout=60):
        r = self._raw(["-s", self.serial] + list(args), timeout)
        if check and r.returncode != 0:
            raise BenchError(f"adb {' '.join(args)} failed: {(r.stderr or r.stdout).strip()}")
        return r.stdout

    def devices(self):
        return [line.split()[:2] for line in self._raw(["devices"]).stdout.splitlines()[1:] if len(line.split()) >= 2]

    def ensure_connected(self):
        for attempt in range(3):
            devices = self._raw(["devices"]).stdout
            for line in devices.splitlines()[1:]:
                parts = line.split()
                if len(parts) >= 2 and parts[0] == self.serial:
                    if parts[1] == "device":
                        return
                    if parts[1] == "unauthorized":
                        raise BenchError(f"adb sees {self.serial} but it is unauthorized; accept the debugging prompt")
            if ":" not in self.serial:
                raise BenchError(f"adb doesn't see {self.serial}; plug in its USB cable or pass --serial")
            out = self._raw(["connect", self.serial], timeout=20)
            self.log(f"adb connect {self.serial}: {(out.stdout or out.stderr).strip()}")
            time.sleep(1 + attempt)
        raise BenchError(f"adb can't connect to {self.serial}. Join the Control Hub's Wi-Fi, then run "
                         f"`adb connect {self.serial}` yourself to see why.")

    def pid(self):
        # pidof where toybox has it, else ps (which needs -A from Android 8 on) and its PID column.
        out = self.run("shell", f"pidof {RC_PACKAGE} 2>/dev/null", check=False).strip()
        if out:
            pid = out.split()[0]
        else:
            out = self.run("shell", f"(ps -A 2>/dev/null || ps) | grep {RC_PACKAGE}$", check=False).strip()
            if not out:
                return None
            fields = out.splitlines()[0].split()
            pid = fields[1] if len(fields) > 1 else ""
        if not pid.isdigit():
            raise BenchError(f"can't read the Robot Controller's pid from {out!r}")
        return pid

    def hub_time(self):
        return self.run("shell", "date '+%m-%d %H:%M:%S'").strip() + ".000"

    def hub_epoch_ms(self):
        return int(self.run("shell", "date +%s").strip()) * 1000

    def logcat_since(self, hub_time):
        return self.run("shell", f"logcat -d -v threadtime -T '{hub_time}'", timeout=120)

    def bench_dir(self):
        return PRIVATE_BENCH_DIR if self.private else HUB_BENCH_DIR

    def bench_shell(self, command, check=True):
        if self.private:
            command = f"run-as {RC_PACKAGE} sh -c {shlex.quote(command)}"
        return self.run("shell", command, check=check)

    def push_json(self, obj, name, scratch):
        text = json.dumps(obj)
        scratch.write_text(text)
        path = f"{self.bench_dir()}/{name}"
        self.bench_shell(f"printf '%s' {shlex.quote(text)} > {path}.tmp && mv {path}.tmp {path}")
        back = self.bench_shell(f"cat {path}", check=False).strip()
        if back != text:
            raise BenchError(f"wrote {text!r} to the hub's {path} but read back {back!r}")

    def read_bench_file(self, name):
        path = f"{self.bench_dir()}/{name}"
        r = self._raw(["-s", self.serial, "shell", f"run-as {RC_PACKAGE} cat {path}"])
        if r.returncode != 0 or not r.stdout.strip() or "No such file" in r.stdout:
            raise BenchError(f"the hub has no {path}: {(r.stderr or r.stdout).strip()}")
        return r.stdout

    def mark(self, text):
        self.run("shell", f"log -t BENCH_RUNNER {shlex.quote(text)}", check=False)

    def logcat_matches(self, hub_time, *needles):
        args = " ".join(f"-e {shlex.quote(n)}" for n in needles)
        return self.run("shell", f"logcat -d -v threadtime -T '{hub_time}' | grep -F {args}", check=False)

    def active_config(self):
        """The config the app loads on its next robot restart, from its own SharedPreferences. run-as needs a
        debuggable app, which the IDE's Run button installs."""
        return self.active_config_entry()["name"]

    def active_config_entry(self):
        r = self._raw(["-s", self.serial, "shell", f"run-as {RC_PACKAGE} cat {RC_PREFS}"])
        try:
            root = ElementTree.fromstring(r.stdout)
        except ElementTree.ParseError:
            raise BenchError(f"run-as can't read the Robot Controller's settings: {(r.stderr or r.stdout).strip()[:200]}")
        for entry in root.iter("string"):
            if entry.get("name") == PREF_ACTIVE_CONFIG:
                # RobotConfigFile.toString(), a JSON object.
                return json.loads(entry.text)
        return {"name": NO_CONFIG, "location": "NONE", "resourceId": 0}

    def stage_active_config(self, name, scratch, config=None):
        """Pushes a copy of the app's settings naming the active config, for switch_app().
        Read while the app is idle, so the file matches what it holds in memory."""
        r = self._raw(["-s", self.serial, "shell", f"run-as {RC_PACKAGE} cat {RC_PREFS}"])
        try:
            root = ElementTree.fromstring(r.stdout)
        except ElementTree.ParseError:
            raise BenchError(f"run-as can't read the Robot Controller's settings: {(r.stderr or r.stdout).strip()[:200]}")
        entry = next((e for e in root.iter("string") if e.get("name") == PREF_ACTIVE_CONFIG), None)
        if entry is None:
            entry = ElementTree.SubElement(root, "string", {"name": PREF_ACTIVE_CONFIG})
        # RobotConfigFile's Gson form, which RobotConfigFileManager.getActiveConfig() parses at robot setup.
        entry.text = json.dumps(config or {"name": name, "resourceId": 0, "location": "LOCAL_STORAGE", "isDirty": False})
        scratch.write_bytes(ElementTree.tostring(root, encoding="utf-8", xml_declaration=True))
        # adb push keeps the local file's mode, and run-as reads it as the app's uid.
        os.chmod(scratch, 0o644)
        self.run("push", str(scratch), HUB_PREFS_TMP)

    def switch_app(self):
        """Stops the app, puts the staged settings in place and starts it again, all in one adb call: the Control
        Hub's watchdog relaunches a silent app about 8.5 s after it stops hearing from it, on whatever settings are
        there by then. The app is started even if the copy fails."""
        out = self.run("shell", f"am force-stop {RC_PACKAGE} && run-as {RC_PACKAGE} cp {HUB_PREFS_TMP} {RC_PREFS} "
                                f"&& echo {PREFS_COPIED}; am start -n {RC_LAUNCHER}; rm -f {HUB_PREFS_TMP}", check=False)
        if PREFS_COPIED not in out:
            raise BenchError(f"couldn't put the Robot Controller's new settings in place: {out.strip()[:300]}")

    def clock(self):
        out = self.run("shell", "date +%s; cat /proc/uptime", check=False).split()
        try:
            return {"epochS": int(out[0]), "uptimeS": float(out[1])}
        except (IndexError, ValueError):
            return {"raw": " ".join(out)[:200]}

    def listeners(self, ports):
        out = self.run("shell", "cat /proc/net/tcp /proc/net/tcp6", check=False)
        found = []
        for line in out.splitlines():
            fields = line.split()
            if len(fields) > 7 and fields[3] == TCP_LISTEN and ":" in fields[1]:
                address, _, port = fields[1].rpartition(":")
                if int(port, 16) in ports:
                    found.append({"port": int(port, 16), "address": "any" if not address.strip("0") else address,
                                  "uid": fields[7]})
        return found

    def config_names(self):
        out = self.run("shell", f"ls -1 {CONFIG_DIR}")
        return {line.strip()[:-4] for line in out.splitlines() if line.strip().endswith(".xml")}

    def read_config(self, name):
        return self.run("shell", f"cat {shlex.quote(f'{CONFIG_DIR}/{name}.xml')}")

    def remove_config(self, name):
        self.run("shell", f"rm {shlex.quote(f'{CONFIG_DIR}/{name}.xml')}")


# ---------------------------------------------------------------------------------------------------------------------
# Hardware configs
# ---------------------------------------------------------------------------------------------------------------------

def attr(value):
    return xml.sax.saxutils.quoteattr(str(value))


def hub_config(devices, extra=(), expansion=None):
    body = "".join(f"            {d}\n" for d in devices)
    tail = "".join(f"    {e}\n" for e in extra)
    expansion_module = ""
    if expansion is not None:
        address, expansion_devices = expansion
        expansion_module = (f'        <LynxModule name="Expansion Hub {address}" port="{address}">\n'
                            + "".join(f"            {d}\n" for d in expansion_devices)
                            + "        </LynxModule>\n")
    return ("<?xml version='1.0' encoding='UTF-8' standalone='yes' ?>\n"
            '<Robot type="FirstInspires-FTC">\n'
            '    <LynxUsbDevice name="Control Hub Portal" serialNumber="(embedded)" parentModuleAddress="173">\n'
            '        <LynxModule name="Control Hub" port="173">\n'
            f"{body}"
            "        </LynxModule>\n"
            f"{expansion_module}"
            "    </LynxUsbDevice>\n"
            f"{tail}"
            "</Robot>\n")


def pinpoint_tag(bus):
    return f'<goBILDAPinpoint name="pinpoint" port="0" bus="{bus}" />'


def octoquad_tag(bus):
    return f'<OctoQuadFTC name="octoquad" port="0" bus="{bus}" />'


# LynxModuleImuType.name() as the probe reports it -> the SDK config tag (LynxConstants.EMBEDDED_*_IMU_XML_TAG).
IMU_TAGS = {"BNO055": "LynxEmbeddedIMU", "BHI260": "ControlHubImuBHI260AP"}


def main_devices(imu_type, pinpoint=False, octoquad_bus=None):
    if imu_type not in IMU_TAGS:
        raise BenchError(f"The probe read the Control Hub's embedded IMU as {imu_type!r}, which the bench config can't "
                         "name; power-cycle the hub and rerun, and tell Claude if it persists.")
    devices = [
        '<goBILDA5202SeriesMotor name="fl" port="0" />',
        '<goBILDA5202SeriesMotor name="fr" port="1" />',
        '<goBILDA5202SeriesMotor name="bl" port="2" />',
        '<goBILDA5202SeriesMotor name="br" port="3" />',
        '<Servo name="benchServo" port="0" />',
        '<Servo name="benchServo2" port="1" />',
        '<AnalogInput name="floodgate" port="0" />',
    ]
    devices.append(f'<{IMU_TAGS[imu_type]} name="imu" port="0" bus="0" />')
    if pinpoint:
        devices.append(pinpoint_tag(1))
    if octoquad_bus is not None:
        devices.append(octoquad_tag(octoquad_bus))
    return devices


def extras(webcam_serial, limelight):
    out = []
    if webcam_serial is not None:
        out.append(f"<Webcam name=\"aprilTagDetector\" serialNumber={attr(webcam_serial)} />")
    if limelight is not None:
        out.append(f"<EthernetDevice name=\"limelight\" serialNumber={attr(limelight['serial'])} port=\"0\" "
                   f"ipAddress={attr(limelight['ipAddress'])} />")
    return out


def bench_port_devices(xml_text):
    try:
        root = ElementTree.fromstring(xml_text)
    except ElementTree.ParseError as e:
        raise BenchError(f"can't parse a hardware config to check the bench ports: {e}")
    found = []
    for usb in root.iter("LynxUsbDevice"):
        if usb.get("serialNumber") != "(embedded)":
            continue
        for module in usb.iter("LynxModule"):
            if module.get("port") != usb.get("parentModuleAddress"):
                continue
            for device in module:
                port = device.get("port") or ""
                if device.get("bus") is not None or not port.isdigit():
                    continue
                tag = device.tag.lower()
                if "motor" in tag and int(port) in BENCH_MOTOR_PORTS:
                    found.append(f"motor {device.get('name')} on port {port}")
                elif ("servo" in tag or device.tag in SERVO_PORT_TAGS) and int(port) in BENCH_SERVO_PORTS:
                    found.append(f"servo {device.get('name')} on port {port}")
    return found


def limelight_tag(ip_address):
    return (f"<EthernetDevice name=\"limelight\" serialNumber={attr(LIMELIGHT_CONFIG_SERIAL)} port=\"0\" "
            f"ipAddress={attr(ip_address)} />")


# ---------------------------------------------------------------------------------------------------------------------
# Runner
# ---------------------------------------------------------------------------------------------------------------------

def fmt(summary, unit="ms"):
    if not isinstance(summary, dict) or not summary.get("count"):
        return "n/a"
    return (f"p50 {summary['p50']:.2f} / p95 {summary['p95']:.2f} / p99 {summary.get('p99', float('nan')):.2f} / "
            f"max {summary['max']:.2f} {unit} (n={summary['count']})")


def fmt_volts(summary):
    if not isinstance(summary, dict) or not summary.get("count"):
        return "n/a"
    return (f"min {summary['min']:.3f} / mean {summary['mean']:.3f} / p99 {summary['p99']:.3f} / max {summary['max']:.3f} V "
            f"(n={summary['count']})")


def percentile(values, p):
    ordered = sorted(values)
    return ordered[max(0, min(len(ordered) - 1, math.ceil(p / 100 * len(ordered)) - 1))]


def fmt_plans(values):
    if not values:
        return "no plans"
    return (f"p50 {percentile(values, 50):.0f} / p95 {percentile(values, 95):.0f} / max {max(values):.0f} ms "
            f"(n={len(values)})")


def fmt_cell(summary):
    if not isinstance(summary, dict) or not summary.get("count"):
        return "n/a"
    return f"{summary['p50']:.2f} / {summary['p95']:.2f} / {summary['max']:.2f}"


def p50_diff(before, after):
    if not all(isinstance(s, dict) and s.get("count") for s in (before, after)):
        return "n/a"
    return f"{after['p50'] - before['p50']:+.2f}"


def fmt_chip(value):
    return f"0x{value:02x}" if isinstance(value, int) else value



def dig(obj, *path, default=None):
    for p in path:
        if isinstance(obj, dict) and p in obj:
            obj = obj[p]
        elif isinstance(obj, list) and isinstance(p, int) and -len(obj) <= p < len(obj):
            obj = obj[p]
        else:
            return default
    return obj


def http_request(method, url, timeout=3.0):
    request = urllib.request.Request(url, data=b"" if method == "POST" else None, method=method,
                                     headers={"Content-Type": "application/json"})
    t0 = time.monotonic()
    try:
        with urllib.request.urlopen(request, timeout=timeout) as reply:
            out = {"code": reply.status, "body": reply.read(2000).decode("utf-8", "replace")}
    except urllib.error.HTTPError as e:
        out = {"code": e.code, "body": e.read(2000).decode("utf-8", "replace")}
    except (urllib.error.URLError, OSError) as e:
        out = {"error": f"{type(e).__name__}: {e}"}
    out["ms"] = round((time.monotonic() - t0) * 1000, 1)
    return out


LOGCAT_STAMP = re.compile(r"^(\d\d)-(\d\d) (\d\d):(\d\d):(\d\d)\.(\d{3})")


def logcat_time(line):
    m = LOGCAT_STAMP.match(line or "")
    if m is None:
        return None
    month, day, hour, minute, second, ms = (int(g) for g in m.groups())
    return datetime.datetime(2000, month, day, hour, minute, second, ms * 1000)


def logcat_delta(start, end):
    a, b = logcat_time(start), logcat_time(end)
    if a is None or b is None:
        return None
    seconds = (b - a).total_seconds()
    if seconds < -180 * 86400:
        seconds += 366 * 86400
    return round(seconds, 2)


def sloth_jar(loader):
    m = SLOTH_JAR.search(str(loader or ""))
    return f"{m.group(1)}.jar" if m else None


def jar_clock(jar_s, host_s, hub_s):
    if jar_s is None or hub_s is None:
        return "unknown: no jar or no hub time"
    if host_s is not None and abs(hub_s - host_s) < 600:
        return "can't tell: the hub's clock is within 10 minutes of the Mac's"
    if host_s is not None and abs(jar_s - host_s) <= 2:
        return "the Mac's: adb push stamped the jar with to_load.jar's time"
    if abs(jar_s - hub_s) <= 3600:
        return "the hub's: Sloth's setLastModified at CREATE, or adb push not keeping the Mac's time"
    return "neither the Mac's nor the hub's"


def crash_text(messages):
    for i, text in enumerate(messages):
        if USER_CODE_THREW in text:
            return next((t for t in messages[i + 1:i + 6] if EXCEPTION_LINE.match(t)), text)
    return None


def logcat_messages(log_text, tag):
    out = []
    for line in log_text.splitlines():
        parts = line.split(None, 5)
        if len(parts) == 6:
            line_tag, sep, message = parts[5].partition(":")
            if sep and line_tag.strip() == tag:
                out.append(message.strip())
    return out


def mentions(text, phrase):
    return re.search(re.escape(phrase) + r"(?!\w)", text or "") is not None


class Runner:
    def __init__(self, args):
        self.args = args
        stamp = datetime.datetime.now().strftime("%Y-%m-%d_%H%M%S")
        self.out = REPO_ROOT / "bench-results" / stamp
        n = 1
        while self.out.exists():
            n += 1
            self.out = REPO_ROOT / "bench-results" / f"{stamp}-{n}"
        (self.out / "raw").mkdir(parents=True)
        (self.out / "logs").mkdir()
        self.logfile = open(self.out / "run.log", "w")
        self.summary = []
        self.checks = []
        self.results = {}
        self.adb = Adb(args.serial or f"{args.host}:{ADB_PORT}", self.log)
        self.dash = None
        self.probe = None
        self.original = None
        self.original_entry = None
        self.original_shipped = False
        self.configs_touched = False
        self.read_active_config = None
        self.switch_with_adb = False
        self.switches = []
        self.watch = None
        self.known_pid = None
        self.restarts = []

    def log(self, text):
        line = f"[{datetime.datetime.now().strftime('%H:%M:%S')}] {text}"
        print(line, flush=True)
        self.logfile.write(line + "\n")
        self.logfile.flush()

    def check(self, name, ok, detail, fail_means=None):
        self.checks.append((name, "PASS" if ok else "FAIL", detail, fail_means))
        self.log(f"{'PASS' if ok else 'FAIL'} {name}: {detail}")

    # --- setup -------------------------------------------------------------------------------------------------------

    def connect(self):
        self.log(f"results folder: {self.out}")
        self.adb.ensure_connected()
        pid = self.note_pid()
        if pid is None:
            raise BenchError("The Robot Controller app isn't running on the hub. Do the full install with the IDE's "
                             "Run button (TeamCode configuration) and let the app start.")
        self.log(f"adb ok, Robot Controller pid {pid}")
        others = [serial for serial, state in self.adb.devices() if serial != self.adb.serial]
        if others:
            self.log(f"WARNING: adb also lists {', '.join(others)}; the runner and its deploySloth use only "
                     f"{self.adb.serial} (deploySloth through ANDROID_SERIAL)")
        self.adb.run("shell", f"mkdir -p {HUB_BENCH_DIR}")
        # The hub logs ~50 Lights Hal lines a second, which pushes the app's own lines out of the default buffer.
        self.adb.run("shell", f"logcat -G {LOG_BUFFER}", check=False)
        self.dash = Dashboard(self.args.host, DASHBOARD_PORT, self.log)
        status = self.dash.get_status()
        if status.get("enabled") is False:
            raise BenchError("slothboard reports the dashboard disabled (its saved setting survives app restarts and the "
                             "Run button's reinstall), so it ignores INIT and sends no telemetry. Turn it back on "
                             f"with: {self.dashboard_recovery()}")
        self.dash.wait_until(lambda: self.dash.opmodes is not None, 10, "the OpMode list")
        self.log(f"dashboard ok: {status}")
        self.results["tcpListeners"] = {"atConnect": self.adb.listeners((LIMELIGHT_PORT, FAKE_LIMELIGHT_PORT))}
        self.log(f"listening on {LIMELIGHT_PORT}/{FAKE_LIMELIGHT_PORT}: {self.results['tcpListeners']['atConnect']}")
        self.results["hubClock"] = {"atConnect": self.adb.clock()}
        try:
            self.original_entry = self.adb.active_config_entry()
            active = self.original_entry["name"]
            self.read_active_config = self.adb.active_config
            self.switch_with_adb = True
            self.log("reading the active hardware config from the app's settings (run-as); switching configs over adb")
        except BenchError as e:
            self.log(f"{e}; asking the dashboard on a fresh connection instead, and switching configs through it")
            self.read_active_config = self.dash.fresh_active_config
            self.switch_with_adb = False
            active = self.active_config()
            self.original_entry = {"name": active}
        self.adb.private = self.switch_with_adb
        self.adb.bench_shell(f"mkdir -p {self.adb.bench_dir()}")
        self.log(f"bench parameters and results go through {self.adb.bench_dir()}"
                 + (" in the app's own storage (run-as)" if self.adb.private else ""))
        if status["activeOpMode"] != IDLE_OPMODE:
            raise BenchError(f"'{status['activeOpMode']}' is running; stop it before benching")
        names = {n for n, _ in self.dash.opmodes}
        missing = [n for n in BENCH_OPMODES if n not in names]
        if missing:
            raise BenchError(f"The hub doesn't have {', '.join(missing)}. Do a full install with the IDE's Run button "
                             "(TeamCode configuration) so the bench OpModes are on it.")
        self.check_bench_ports(active)
        if active in BENCH_CONFIGS:
            self.log(f"WARNING: {active} is active, left by an earlier bench run; it stays active at the end")
        if active != NO_CONFIG and active in self.adb.config_names():
            self.original = active
            self.log(f"active hardware config: {active} (restored at the end)")
            (self.out / "original_config.xml").write_text(self.adb.read_config(active))
        elif (self.original_entry.get("location") == "RESOURCE"
              or ("location" not in self.original_entry and (REPO_ROOT / SHIPPED_CONFIG_DIR / f"{active}.xml").exists())):
            self.original, self.original_shipped = active, True
            self.log(f"active hardware config: {active}, shipped in res/xml ({self.original_entry}); re-activated by "
                     "name at the end")
            (self.out / "original_config.txt").write_text(f"active config: {json.dumps(self.original_entry)}\n")
        else:
            self.log(f"active hardware config: {active}, which isn't a file in {CONFIG_DIR}; the hub will be left on "
                     f"the {CONFIG_PROBE} config")
            (self.out / "original_config.txt").write_text(f"active config: {active}\n")

    def check_bench_ports(self, active):
        files = self.adb.config_names()
        shipped = REPO_ROOT / SHIPPED_CONFIG_DIR / f"{active}.xml"
        if active in BENCH_CONFIGS or active == NO_CONFIG:
            sources = [(f"{name} (a config on the hub)", self.adb.read_config(name))
                       for name in sorted(files) if name not in BENCH_CONFIGS]
        elif active in files:
            sources = [(f"the active config {active}", self.adb.read_config(active))]
        elif shipped.exists():
            sources = [(f"the active config {active} (shipped in res/xml)", shipped.read_text())]
        else:
            sources = None
        self.log(f"this run commands {BENCH_POWERS}")
        if self.args.ports_empty:
            self.log("--ports-empty: not checking the hardware configs for devices on those ports")
            return
        if sources is None:
            raise BenchError(f"Can't read the active hardware config {active} to check what is on the ports the bench "
                             "drives. Make sure Control Hub motor ports 0-3 and servo ports 0-1 are empty, then rerun with "
                             "--ports-empty.")
        found = [f"{what}: {', '.join(devices)}" for what, devices in
                 ((what, bench_port_devices(text)) for what, text in sources) if devices]
        if found:
            raise BenchError(f"{'; '.join(found)}. The bench drives Control Hub motor ports 0-3 and servo ports 0-1 "
                             f"({BENCH_POWERS}). Unplug everything from those ports, then rerun with --ports-empty.")

    def active_config(self):
        """The hub's own answer, never the last config list a socket happened to receive: a restart can drop the
        socket and lose the list that named the new config."""
        name = self.read_active_config()
        if name is None:
            raise BenchError("Can't tell which hardware config is active: run-as can't read the app's settings and "
                             "the dashboard sent no config list on a fresh connection")
        return name

    def wait_idle(self, timeout=30):
        with self.dash.cond:
            self.dash.status = None
        try:
            self.dash.wait_until(lambda: self.dash.status and self.dash.status.get("available")
                                 and self.dash.status["activeOpMode"] == IDLE_OPMODE, timeout,
                                 "the robot to be idle", poll_status_every=1.0)
        except ProtocolError:
            raise
        except BenchError as e:
            if not self.app_restarting():
                raise
            self.log(f"!! the Robot Controller app is restarting ({e}); waiting for it before the next step")
            self.wait_restart("between steps")

    def note_pid(self):
        self.known_pid = self.adb.pid()
        return self.known_pid

    def step_pid(self):
        before = self.known_pid
        pid = self.note_pid()
        if before is not None and pid != before:
            self.log(f"!! the Robot Controller app restarted since the last step (pid {before} -> {pid})")
            self.restarts.append({"during": "between steps", "pid": pid})
        return pid

    def app_restarting(self):
        self.adb.ensure_connected()
        pid = self.adb.pid()
        s = self.dash.status
        return (pid is None or pid != self.known_pid or not self.dash.connected
                or (s is not None and (not s.get("available") or s["activeOpMode"] == "")))

    def wait_restart(self, during):
        t0 = time.monotonic()
        self.dash.ensure_connected(timeout=120)
        self.wait_ready()
        pid = self.note_pid()
        self.restarts.append({"during": during, "pid": pid, "waitedSeconds": round(time.monotonic() - t0, 1)})
        self.log(f"the Robot Controller app is back up (pid {pid}) after {time.monotonic() - t0:.0f} s")

    def activate_config(self, name, xml_text=None, config=None):
        """Over adb: write the file, stop the app, point its settings at the file and start it. Through slothboard's
        WRITE/SET_HARDWARE_CONFIG only when run-as can't edit the settings: that path freezes the app until the
        Control Hub's watchdog relaunches it. Done when the hub logs a robot setup and its settings name the config."""
        self.wait_idle()
        if xml_text is None and config is None and name not in self.adb.config_names():
            # slothboard would activate "no config" for a name it doesn't list.
            raise BenchError(f"hardware config {name} isn't in {CONFIG_DIR} on the hub")
        pid = self.adb.pid()
        hub_t0 = self.adb.hub_time()
        self.adb.mark(f"activate config {name}")
        t0 = time.monotonic()
        self.configs_touched = True
        if xml_text is not None:
            (self.out / "logs" / f"{name}.xml").write_text(xml_text)
        if self.switch_with_adb:
            self.log(f"{'writing and ' if xml_text is not None else ''}activating hardware config {name} over adb")
            if xml_text is not None:
                self.adb.run("push", str(self.out / "logs" / f"{name}.xml"), f"{CONFIG_DIR}/{name}.xml")
            self.adb.stage_active_config(name, self.out / "logs" / "rc_prefs.xml", config)
            self.adb.switch_app()
        elif xml_text is not None:
            self.log(f"writing and activating hardware config {name} through the dashboard")
            self.dash.send({"type": "WRITE_HARDWARE_CONFIG", "hardwareConfigName": name,
                            "hardwareConfigContents": xml_text})
        else:
            self.log(f"activating hardware config {name} through the dashboard")
            self.dash.send({"type": "SET_HARDWARE_CONFIG", "hardwareConfigName": name})
        log_name = f"config_{len(self.switches) + 1:02d}_{name}.logcat.txt"
        record = {"config": name, "pidBefore": pid, "overAdb": self.switch_with_adb, "failed": None, "log": log_name}
        log_text = ""
        try:
            while True:
                hits = self.adb.logcat_matches(hub_t0, ROBOT_SETUP_DONE, RESTART_FAILED, ROBOT_FAILED)
                if RESTART_FAILED in hits:
                    raise BenchError(f"slothboard couldn't restart the robot onto {name} (logs/{log_name})")
                if ROBOT_FAILED in hits:
                    raise BenchError(f"the robot failed to start on {name} (logs/{log_name})")
                if ROBOT_SETUP_DONE in hits:
                    break
                if time.monotonic() - t0 > CONFIG_SWITCH_TIMEOUT_S:
                    raise BenchError(f"the robot didn't finish setting up within {CONFIG_SWITCH_TIMEOUT_S} s of "
                                     f"switching to {name}; Robot Controller pid {pid} before, {self.adb.pid()} now "
                                     f"(logs/{log_name})")
                time.sleep(2)
            active = self.active_config()
            if active != name:
                raise BenchError(f"the robot restarted on hardware config {active}, not {name} (logs/{log_name})")
            self.dash.ensure_connected(timeout=60)
            self.wait_ready()
            # The settings file is what the runner wrote; the dashboard's list says what the running app loaded.
            live = None
            for _ in range(5):
                live = self.dash.fresh_active_config()
                if live is not None:
                    break
                time.sleep(2)
            if live is not None and live != name:
                raise BenchError(f"the running app has hardware config {live}, not {name} (logs/{log_name})")
        except BenchError as e:
            record["failed"] = str(e)
            raise
        finally:
            log_text = self._save_log(f"logs/{log_name}", hub_t0) or log_text
            watchdog = log_text.count(WATCHDOG_KILLED)
            # Over adb a relaunch can also follow the deliberate stop if the switch overran; the live check covers that.
            record.update(seconds=round(time.monotonic() - t0, 1), pidAfter=self.note_pid(),
                          appKilled=APP_KILLED in log_text or (watchdog > 0 and not self.switch_with_adb),
                          watchdogRelaunches=watchdog)
            self.switches.append(record)
        if record["appKilled"]:
            self.log(f"!! the Robot Controller app froze during the switch and was killed "
                     f"(pid {pid} -> {record['pidAfter']})")
        elif watchdog:
            self.log(f"the Control Hub's watchdog relaunched the app during the switch ({watchdog}x)")

    def wait_ready(self):
        # A reconnected socket still holds the status from before the switch.
        with self.dash.cond:
            self.dash.status = None
        self.dash.wait_until(lambda: self.dash.status and self.dash.status.get("available")
                             and self.dash.status["activeOpMode"] == IDLE_OPMODE
                             and self.dash.status.get("batteryVoltage", -1) >= 0, 90,
                             "the robot to finish restarting", poll_status_every=1.0)
        time.sleep(2)

    def remove_bench_configs(self, keep):
        """Deletes the bench_* files with adb rather than DELETE_HARDWARE_CONFIG, which restarts the robot each time
        and activates "no config" if the one it deletes is active."""
        active = self.active_config()
        doomed = [n for n in BENCH_CONFIGS if n in self.adb.config_names() and n != keep]
        if active in doomed:
            raise BenchError(f"{active} is still the active hardware config, so the bench configs weren't deleted")
        for name in doomed:
            self.log(f"deleting hardware config {name}")
            self.adb.remove_config(name)
        return doomed

    # --- running an OpMode --------------------------------------------------------------------------------------------

    def run_opmode(self, opmode, step, params=None, start=True, timeout=600, expect_crash=False,
                   stop_when_done=True, on_telemetry=None, result_files=(), timeout_ok=False, status_every=5.0,
                   quiet_after=3.0, init_hold=0.0, during=None, during_init=None):
        """INIT (and START), wait for BENCH DONE or a crash, STOP; returns what happened. Never leaves it running.
        With timeout_ok, running out of time is an outcome ("timedOut") instead of an error. Status is polled every
        status_every s, or every 1 s once telemetry has been silent for quiet_after s (to catch a crash).
        init_hold keeps it in INIT that many seconds before START; during() runs right after START."""
        self.wait_idle()
        for f in result_files:
            self.adb.bench_shell(f"rm -f {self.adb.bench_dir()}/{f}.json {self.adb.bench_dir()}/{f}.json.tmp")
        self.adb.push_json(params or {}, "params.json", self.out / "logs" / "params.json")
        hub_t0 = self.adb.hub_time()
        self.adb.mark(f"begin {step}")
        outcome = {"step": step, "opmode": opmode, "params": params or {}, "pidBefore": self.step_pid(),
                   "timedOut": False}
        failed = None
        self.begin_watch(opmode, hub_t0)
        self.dash.reset_telemetry()
        self.dash.start_recording()
        t_init = time.monotonic()
        t_start = None
        try:
            self.dash.send({"type": "INIT_OP_MODE", "opModeName": opmode})
            self.dash.wait_until(lambda: self._status_is(opmode) or self.crashed(), 30,
                                 f"{opmode} to INIT", poll_status_every=0.5, tick=self.adb_crash_tick)
            outcome["initSeenSeconds"] = time.monotonic() - t_init
            t_seen = time.monotonic()
            if self._status_is(opmode):
                outcome["initStatusError"] = self.dash.status.get("errorMessage") or ""
            if init_hold > 0:
                self.wait_or_crash(lambda: False, init_hold)
            if during_init is not None and not self.crashed():
                during_init()
            if start and not self.crashed():
                t_start = time.monotonic()
                self.dash.send({"type": "START_OP_MODE"})
                self.dash.wait_until(lambda: self._status_is(opmode, "RUNNING") or self.crashed(), 30,
                                     f"{opmode} to START", poll_status_every=0.5)
                if during is not None and not self.crashed():
                    during()
            deadline = time.monotonic() + timeout
            next_status = 0.0
            next_logcat = 0.0
            done_from_logcat = None
            while not (self.dash.done_line is not None or self.crashed()):
                self.dash.check()
                if on_telemetry is not None:
                    on_telemetry(self.dash.data)
                now = time.monotonic()
                if now > deadline:
                    if timeout_ok:
                        outcome["timedOut"] = True
                        break
                    raise BenchError(f"{opmode} didn't report BENCH DONE within {timeout} s "
                                     f"(last phase {self.dash.data.get(KEY_PHASE)!r}, "
                                     f"{self.dash.recording['messages']} telemetry messages since INIT; logs/{step}.log)")
                quiet = now - self.dash.last_telemetry > quiet_after
                if now >= next_status:
                    self.dash.get_status()
                    next_status = now + (1.0 if quiet else status_every)
                if ("dashboardSilent" not in outcome and now - t_seen >= SILENT_AFTER_S
                        and not self.dash.recording["packets"] and self._status_is(opmode)):
                    outcome["dashboardSilent"] = self.silent_cause(hub_t0)
                    self.log(f"!! no dashboard telemetry from {opmode} in {SILENT_AFTER_S:.0f} s: "
                             f"{outcome['dashboardSilent']}; watching logcat for its BENCH DONE")
                if (quiet or "dashboardSilent" in outcome) and result_files and now >= next_logcat:
                    next_logcat = now + LOGCAT_DONE_EVERY_S
                    done_from_logcat = self.logcat_done(hub_t0, result_files)
                    if done_from_logcat is not None:
                        break
                time.sleep(0.2)
            crashed = self.crashed()
            if done_from_logcat is not None and not crashed:
                outcome["doneLine"], outcome["telemetryLost"] = done_from_logcat, True
                self.log(f"{opmode}: BENCH DONE {done_from_logcat} (from logcat: dashboard telemetry lost)")
            if crashed:
                outcome["crashSignal"] = self.watch["crashSignal"]
                outcome["crashSeenSeconds"] = round(self.watch["crashAt"] - t_init, 2)
                if t_start is not None and self.watch["crashAt"] >= t_start:
                    outcome["crashSeenAfterStartSeconds"] = round(self.watch["crashAt"] - t_start, 2)
            if not crashed and self.dash.done_line is not None:
                outcome["doneLine"] = self.dash.done_line
                self.log(f"{opmode}: BENCH DONE {self.dash.done_line}")
            result = self._finish(outcome, step, hub_t0, crashed=crashed, expect_crash=expect_crash,
                                  stop=stop_when_done)
        except BenchError as e:
            failed = e
            raise
        finally:
            try:
                if self.dash.recording is not None:
                    outcome["dashboard"] = self._dashboard_stats(self.dash.stop_recording())
                if self.dash.status and self.dash.status["activeOpMode"] == opmode:
                    self._stop(opmode, outcome)
            finally:
                # After the stop, so a hang's stop-time thread dump or kill is in the log.
                if failed is not None:
                    log_text = self._save_log(f"logs/{step}.log", hub_t0)
                    self.results[step] = dict(outcome, failed=str(failed), logMarkers=self._markers(log_text),
                                              appKilled=APP_KILLED in log_text or WATCHDOG_KILLED in log_text)
        if "dashboardSilent" in outcome:
            self.log("restarting the Robot Controller app so the next step gets dashboard telemetry again")
            self.activate_config(self.active_config())
        return result

    def dashboard_recovery(self):
        return (f"adb -s {self.adb.serial} shell 'am force-stop {RC_PACKAGE} && run-as {RC_PACKAGE} rm -f "
                f"{DASHBOARD_PREFS}; am start -n {RC_LAUNCHER}'")

    def silent_cause(self, hub_t0):
        status = self.dash.status or {}
        if status.get("enabled") is False:
            return f"slothboard reports the dashboard disabled; turn it back on with: {self.dashboard_recovery()}"
        if not self.adb.logcat_matches(hub_t0, REPORT_STARTED).strip():
            return "the OpMode never logged its BenchReport start, so it is stuck before its first packet"
        if not self.dash.recording["messages"]:
            return "not even slothboard's INIT clear reached this socket, so it gets no telemetry at all"
        return ("only slothboard's INIT clear arrived: its telemetry thread has probably died, which a NaN or infinite "
                "field-overlay value in an earlier packet does until the app restarts (expected only before slothboard "
                "6165.6)")

    def logcat_done(self, hub_t0, result_files):
        for line in self.adb.logcat_matches(hub_t0, "BENCH DONE ").splitlines():
            for f in result_files:
                _, found, rest = line.partition(f"BENCH DONE {f}: ")
                if found:
                    return rest.strip()
        return None

    def wait_or_crash(self, predicate, timeout):
        """Seconds until predicate() held, or None on timeout or once the watched OpMode crashed."""
        t0 = time.monotonic()
        next_status = 0.0
        while True:
            self.dash.check()
            if predicate():
                return time.monotonic() - t0
            now = time.monotonic()
            if now - t0 > timeout:
                return None
            if now >= next_status:
                self.dash.get_status()
                next_status = now + 1.0
                if self.crashed():
                    return None
            time.sleep(0.1)

    def begin_watch(self, opmode, hub_t0=None):
        status = self.dash.get_status()
        since_ms = self.adb.hub_epoch_ms()
        with self.dash.cond:
            self.dash.seen_active = set()
            log_index = len(self.dash.opmode_log)
        self.watch = {"opmode": opmode, "errorBefore": status.get("errorMessage") or "", "sinceMs": since_ms,
                      "logIndex": log_index, "hubT0": hub_t0, "stopSent": False, "adbCrash": None,
                      "nextAdbCheck": time.monotonic() + 1.0, "crashAt": None, "crashSignal": None}

    def _status_is(self, opmode, state=None):
        s = self.dash.status
        return s is not None and s["activeOpMode"] == opmode and (state is None or s["activeOpModeStatus"] == state)

    def user_crash(self):
        log, w = self.dash.opmode_log, self.watch
        for i in range(w["logIndex"], len(log)):
            if (USER_CODE_THREW in str(log[i].get("message", ""))
                    and (log[i].get("timestamp") or 0) >= w["sinceMs"]):
                return crash_text([str(e.get("message", "")) for e in log[i:i + 6]])
        return None

    def crashed(self):
        w = self.watch
        if w["stopSent"]:
            return False
        signal = self._crash_signal()
        if signal is not None and w["crashAt"] is None:
            w["crashAt"], w["crashSignal"] = time.monotonic(), signal
        return signal is not None

    def _crash_signal(self):
        w = self.watch
        if self.user_crash() is not None:
            return "dashboard logcat"
        s = self.dash.status
        if s is None or s["activeOpMode"] != IDLE_OPMODE:
            return None
        if w["adbCrash"]:
            return "adb logcat"
        if w["opmode"] in self.dash.seen_active and self.dash.done_line is None:
            return "back to idle without BENCH DONE"
        error = s.get("errorMessage") or ""
        if error and (w["opmode"] in self.dash.seen_active or error != w["errorBefore"]):
            return "status errorMessage"
        return None

    def adb_crash_tick(self):
        w = self.watch
        if w["stopSent"] or w["adbCrash"] or w["hubT0"] is None or time.monotonic() < w["nextAdbCheck"]:
            return
        s = self.dash.status
        if s is None or s["activeOpMode"] != IDLE_OPMODE or w["opmode"] in self.dash.seen_active:
            return
        w["nextAdbCheck"] = time.monotonic() + 2.0
        if self.user_crash() is None:
            hit = self.adb.logcat_matches(w["hubT0"], USER_CODE_THREW).strip()
            if hit:
                w["adbCrash"] = hit.splitlines()[0]

    def _stop(self, opmode, outcome):
        if self.watch is not None:
            self.watch["stopSent"] = True
        t0 = time.monotonic()
        try:
            self.dash.send({"type": "STOP_OP_MODE"})
            self.dash.wait_until(lambda: self.dash.status and self.dash.status["activeOpMode"] != opmode, 20,
                                 f"{opmode} to stop", poll_status_every=0.25)
        except ProtocolError:
            raise
        except BenchError as e:
            if not self.app_restarting():
                raise
            self.log(f"!! the Robot Controller app restarted while {opmode} was stopping ({e}); waiting for it")
            outcome["restartedDuringStop"] = True
            self.wait_restart(f"stopping {opmode}")
        outcome["stopSecondsRunnerSide"] = time.monotonic() - t0

    def _finish(self, outcome, step, hub_t0, crashed, expect_crash, stop=True):
        if not crashed and stop and self._status_is(outcome["opmode"]):
            self._stop(outcome["opmode"], outcome)
        self.adb.ensure_connected()
        outcome["pidAfter"] = self.adb.pid()
        outcome["appRestarted"] = outcome["pidAfter"] != outcome["pidBefore"]
        if outcome["appRestarted"] and not outcome.get("restartedDuringStop"):
            self.log(f"!! the Robot Controller app restarted during {step}; waiting for it")
            self.wait_restart(step)
            outcome["pidAfter"] = self.known_pid
        status = self.dash.get_status()
        if crashed or self.user_crash() is not None:
            status = self._wait_error_line(status, outcome)
        outcome["crashed"] = crashed
        outcome["statusErrorMessage"] = status.get("errorMessage") or ""
        outcome["warningMessage"] = status.get("warningMessage")
        outcome["dashboardCrashText"] = self.user_crash()
        log_text = self.adb.logcat_since(hub_t0)
        (self.out / "logs" / f"{step}.log").write_text(log_text)
        outcome["appKilled"] = APP_KILLED in log_text or WATCHDOG_KILLED in log_text
        outcome["logMarkers"] = self._markers(log_text)
        for source, text in (("status", outcome["statusErrorMessage"]), ("dashboard logcat", outcome["dashboardCrashText"]),
                             ("adb logcat", crash_text(logcat_messages(log_text, OPMODE_MANAGER_TAG)))):
            if text:
                outcome["errorMessage"], outcome["errorSource"] = text, source
                break
        else:
            outcome["errorMessage"], outcome["errorSource"] = "", None
        if crashed and not expect_crash:
            self.log(f"{outcome['opmode']} CRASHED: {outcome['errorMessage']}")
        elif crashed:
            self.log(f"{outcome['opmode']} crashed as expected: {outcome['errorMessage']}")
        if outcome["appRestarted"]:
            self.log(f"!! the Robot Controller app restarted during {step} (pid {outcome['pidBefore']} -> {outcome['pidAfter']})")
        self.results[step] = outcome
        return outcome

    def _wait_error_line(self, status, outcome):
        t0 = self.watch["crashAt"] or time.monotonic()
        while not status.get("errorMessage") and time.monotonic() - t0 < ERROR_LINE_WAIT_S:
            time.sleep(0.25)
            status = self.dash.get_status()
        if status.get("errorMessage"):
            outcome["errorLineSeconds"] = round(time.monotonic() - t0, 2)
        return status

    @staticmethod
    def _markers(log_text):
        return [line for line in log_text.splitlines() if any(m in line for m in STUCK_MARKERS)][:40]

    def _save_log(self, relative, hub_t0):
        """Best effort, for a failure path: never replaces the error being reported. Returns "" if it couldn't."""
        try:
            self.adb.ensure_connected()
            log_text = self.adb.logcat_since(hub_t0)
        except (BenchError, subprocess.TimeoutExpired) as e:
            self.log(f"couldn't save {relative}: {e}")
            return ""
        (self.out / relative).write_text(log_text)
        return log_text

    def _dashboard_stats(self, rec):
        seconds = max(1e-6, rec["end"] - rec["start"])
        per_phase = {}
        for t, phase, loop, size in rec["packets"]:
            p = per_phase.setdefault(str(phase), {"packets": 0, "bytes": 0.0, "first": t, "last": t, "loops": set(),
                                                  "images": 0, "imageBytes": 0})
            p["packets"] += 1
            p["bytes"] += size
            p["last"] = t
            if loop is not None:
                p["loops"].add(loop)
        for t, phase, size in rec["images"]:
            p = per_phase.setdefault(str(phase), {"packets": 0, "bytes": 0.0, "first": t, "last": t, "loops": set(),
                                                  "images": 0, "imageBytes": 0})
            p["images"] += 1
            p["imageBytes"] += size
            p["last"] = max(p["last"], t)
        out = {"seconds": seconds, "messages": rec["messages"], "telemetryBytes": rec["bytes"],
               "packets": len(rec["packets"]), "images": len(rec["images"]), "perPhase": {}}
        for name, p in per_phase.items():
            span = max(1e-6, p["last"] - p["first"])
            out["perPhase"][name] = {
                "packets": p["packets"], "packetsPerSecond": p["packets"] / span,
                "telemetryKBps": p["bytes"] / span / 1000, "distinctBenchLoops": len(p["loops"]),
                "images": p["images"], "imagesPerSecond": p["images"] / span, "imageKBps": p["imageBytes"] / span / 1000,
                "seconds": span}
        return out

    def pull(self, name, required=True):
        local = self.out / "raw" / f"{name}.json"
        if self.adb.private:
            try:
                local.write_text(self.adb.read_bench_file(f"{name}.json"))
            except BenchError:
                if required:
                    raise
                return None
            return json.loads(local.read_text())
        r = self.adb._raw(["-s", self.adb.serial, "pull", f"{HUB_BENCH_DIR}/{name}.json", str(local)])
        if r.returncode != 0:
            if required:
                raise BenchError(f"the hub has no {HUB_BENCH_DIR}/{name}.json: {r.stderr.strip()}")
            return None
        return json.loads(local.read_text())

    # --- benches ------------------------------------------------------------------------------------------------------

    def run_probe(self):
        address = self.args.expansion_address
        self.activate_config(CONFIG_PROBE, hub_config([], expansion=(address, [])))
        stored = self.original_entry.get("resourceId") if self.original_shipped else None
        self.run_opmode("Bench: Probe", "probe", {"originalResourceId": stored} if stored else None, timeout=60,
                        result_files=["probe"])
        self.probe = self.pull("probe")
        hub = next(h for h in self.probe["hubs"] if h["parent"])
        expansion = next((h for h in self.probe["hubs"] if h.get("address") == address and not h["parent"]), None)
        webcams = [w for w in self.probe["webcams"] if w["attached"]]
        limelights = [l for l in self.probe["limelights"] if not str(l["status"]).startswith("unreachable")]
        self.imu_type = hub["imuType"]
        self.webcam = webcams[0]["serial"] if webcams else None
        self.limelight = limelights[0] if limelights else None
        self.expansion = address if expansion is not None and expansion.get("responding") is True else None
        self.summary.append("## Probe\n")
        self.summary.append(f"- Control Hub IMU: {self.imu_type}; firmware {hub.get('firmware')}")
        self.summary.append(f"- Expansion Hub at address {address}: "
                            + (f"answered; firmware {expansion.get('firmware')}" if self.expansion is not None
                               else "configured but not answering" if expansion is not None
                               else "not in the hardware map, so not answering"))
        self.summary.append(f"- Webcams: {[w['serial'] for w in webcams] or 'none'}")
        self.summary.append(f"- Limelight: {self.limelight['ipAddress'] + ' ' + self.limelight['serial'] if self.limelight else 'none'}")
        self.summary.append(f"- Interfaces: {self.probe['interfaces']}")
        self.summary.append(f"- Shipped configs in this install: {self.probe.get('shippedConfigs')}; active at the "
                            f"start: {self.original_entry}\n")
        self.log(f"probe: IMU {self.imu_type}, webcam {self.webcam}, Limelight {self.limelight}, Expansion Hub "
                 f"{'at address ' + str(address) if self.expansion is not None else 'none answering at address ' + str(address)}")
        if stored:
            name, current = self.original, self.probe.get("shippedConfigs", {}).get(self.original)
            target = self.probe.get("originalResolvesTo")
            advice = f"re-activate {name}" if current else f"{name} isn't shipped any more: activate another config"
            self.check(f"the hub's active config {name} loads res/xml/{name}", current == stored,
                       f"stored as resource {stored:#010x}, which this install resolves to {target}; res/xml/{name} is "
                       + (f"{current:#010x}" if current else "no longer shipped") + ("" if current == stored else f"; {advice}"),
                       "an install that added, renamed or removed a res/xml file renumbered the shipped configs, so the "
                       "hub loaded a different XML than its config name says; re-activate the config on every hub that "
                       "uses it (this run's restore does that here while the name is still shipped); expected only until "
                       "the robot controller re-resolves shipped configs by name at startup")
            if current != stored:
                self.log(f"!! the hub's active config {name} was loading {target}: {advice} on every hub that uses it")

    def main_config(self, webcam="detected", pinpoint=False, octoquad_bus=None, expansion=None):
        serial = self.webcam if webcam == "detected" else webcam
        return hub_config(main_devices(self.imu_type, pinpoint, octoquad_bus), extras(serial, self.limelight), expansion)

    def run_hub(self):
        params = {"limelightPresent": 1 if self.limelight else 0,
                  "plannerScale": 0.3 if self.args.quick else 1.0}
        # Each status reply costs a hub voltage read on slothboard's thread, so ask rarely while the bus is timed.
        o = self.run_opmode("Bench: Hub", "hub", params, timeout=1200, result_files=["hub"], status_every=15.0)
        listeners = self.results["tcpListeners"]
        listeners["afterHub"] = self.adb.listeners((LIMELIGHT_PORT, FAKE_LIMELIGHT_PORT))
        leaked = [entry for entry in listeners["afterHub"] if entry["port"] == FAKE_LIMELIGHT_PORT]
        self.check(f"hub: nothing left listening on the fake Limelight's port {FAKE_LIMELIGHT_PORT}", not leaked,
                   f"after Bench: Hub: {leaked or 'nothing'}",
                   "a fake Limelight server (and maybe its poller) outlived Bench: Hub and loads every later bench in "
                   "this app session")
        hub = self.pull("hub", required=not o["crashed"])
        ph = (hub or {}).get("phases", {})
        missing = [k for k in FAKE_LIMELIGHT_PHASES if k not in ph]
        self.check("hub: every fake-Limelight phase ran", not missing,
                   f"missing {missing}; error {o['errorMessage']!r}" if missing else "all six are in hub.json",
                   "Bench: Hub stopped before or during its fake-Limelight phases (before the fake moved to "
                   f"{FAKE_LIMELIGHT_PORT}, slothboard's Limelight proxy on *:{LIMELIGHT_PORT} kept it from binding)")
        if hub is None:
            self.summary.append(f"## Bench: Hub\n\nCRASHED: {o['errorMessage']}\n")
            return
        pl = ph.get("limelightPostLoopback", {})
        refused = dig(ph, "limelightFailureModes", "refusedPipelineSwitch", default={})
        if not missing:
            self.check("hub: loopback POSTs reached the fake Limelight, not slothboard's proxy",
                       pl.get("requestsServed") == 300 and str(pl.get("limelightBaseUrl")).endswith(f":{FAKE_LIMELIGHT_PORT}"),
                       f"{pl.get('requestsServed')} of 300 served at {pl.get('limelightBaseUrl')}")
            self.check("hub: the refused pipelineSwitch was refused",
                       refused.get("returnedTrue") == 0 and refused.get("refusedTargetProbe") == "ConnectException",
                       f"returned true {refused.get('returnedTrue')} times; a raw connect got {refused.get('refusedTargetProbe')}")
        s = ["## Bench: Hub (bare Control Hub)\n", f"- Battery {dig(hub, 'info', 'batteryVolts')} V, IMU {dig(hub, 'info', 'imuType')}, "
             f"SDK default DS interval {dig(hub, 'info', 'sdkDefaultDsMsTransmissionInterval')} ms, "
             f"OptimizationToggles.dsTransmissionIntervalMs {dig(hub, 'info', 'optimizationTogglesDsIntervalMs')} ms, "
             f"slothboard interval at init {dig(hub, 'info', 'slothboardIntervalAtInit')} ms"]
        s.append(f"- Empty OpMode loop period: {fmt(dig(ph, 'sdkLoopEmpty', 'loopPeriodMs'))}")
        no_ds = f" ({NO_DS})" if dig(hub, "info", "driverStationConnected") is False else ""
        for k in [k for k in ph if k.startswith("dsTransmission")]:
            s.append(f"- {k}: {dig(ph, k, 'transmissions')} sends in {dig(ph, k, 'loops')} loops; interval "
                     f"{fmt(dig(ph, k, 'intervalBetweenTransmissionsMs'))}; update() that sent {fmt(dig(ph, k, 'updateThatTransmittedMs'))}"
                     f"{no_ds}")
        s.append(f"- Bulk read (clear + first read): {fmt(dig(ph, 'bulkReadManual', 'callMs'))}; cached read "
                 f"{fmt(dig(ph, 'bulkReadManual', 'cachedPositionMs'))}")
        s.append(f"- Bulk caching OFF: getCurrentPosition {fmt(dig(ph, 'readsBulkCachingOff', 'getCurrentPositionMs'))}, "
                 f"analog {fmt(dig(ph, 'readsBulkCachingOff', 'analogGetVoltageMs'))}")
        volts = {k: dig(ph, k, "floodgateVolts") for k in ("bulkReadManual", "readsBulkCachingOff")}
        s.append(f"- floodgate (Control Hub analog 0, nothing wired): bulk-cached reads {fmt_volts(volts['bulkReadManual'])}; "
                 f"uncached reads {fmt_volts(volts['readsBulkCachingOff'])}")
        peaks = [v["max"] for v in volts.values() if isinstance(v, dict) and v.get("count")]
        self.check(f"hub: the unwired floodgate pin reads under {FLOODGATE_LIMIT_V:.2f} V ({FLOODGATE_THRESHOLD_A:.0f} A to "
                   "Drivetrain's current limiter)", bool(peaks) and max(peaks) < FLOODGATE_LIMIT_V,
                   f"highest reading {max(peaks):.3f} V" if peaks else "Bench: Hub recorded no floodgate readings",
                   FLOODGATE_PENDING if peaks else "Bench: Hub didn't reach its analog phases")
        for k in ("voltageSensorGetVoltage", "hubGetInputVoltage", "hubGetCurrent", "motorGetCurrent", "motorSetPowerChanging",
                  "motorSetPowerSame", "enhancedMotorSetPowerSame", "fourMotorWrite", "servoSetPositionChanging",
                  "servoSetPositionSame", "imuYawPitchRoll", "imuAngularVelocity"):
            if k in ph:
                s.append(f"- {k}: {fmt(dig(ph, k, 'callMs'))}")
        tb = ph.get("telemetryBuild", {})
        s.append(f"- Telemetry frame build, dashboard-only: {fmt(tb.get('dashboardOnlyBuildMs'))}; DS frame build "
                 f"{fmt(tb.get('dsFrameBuildMs'))}; DS update() {fmt(tb.get('dsFrameUpdateMs'))}; field map "
                 f"{fmt(tb.get('fieldMapRenderMs'))}; overlay {fmt(tb.get('overlayMs'))}; GSON per packet "
                 f"{fmt(tb.get('gsonSerializeOnePacketMs'))} ({dig(tb, 'serializedPacketChars', 'p50')} chars); "
                 f"bytes allocated per dashboard frame {dig(tb, 'allocDashboardOnlyFrames', 'bytesAllocatedPerLoop')}"
                 f"{' (DS frame costs: ' + NO_DS + ')' if no_ds else ''}")
        al = ph.get("allocatingLoop", {})
        s.append(f"- Allocating loop: {fmt(al.get('loopPeriodMs'))}; GC {dig(al, 'gc', 'gcCount')} runs, "
                 f"{dig(al, 'gc', 'gcTimeMs')} ms, blocking {dig(al, 'gc', 'blockingGcCount')}; "
                 f"{dig(al, 'gc', 'bytesAllocatedPerLoop')} B/loop")
        cold = dig(ph, "planner", "coldFirstPlan", default={})
        s.append(f"- Planner cold first plan ({cold.get('balls')} balls"
                 f"{'' if cold.get('firstPlanInProcess') else ', NOT the first in this app process'}): "
                 f"{cold.get('totalMs', float('nan')):.1f} ms")
        for n, v in sorted(dig(ph, "planner", "byBallCount", default={}).items()):
            s.append(f"- Planner {n}: total {fmt(v.get('totalMs'))}; {v.get('plansWithDroppedBalls')} of {v.get('plans')} dropped a ball")
        judged = dig(ph, "planner", "byBallCount", f"n={PLANNER_JUDGED_BALLS}", "totalMs", default={})
        worst = [judged["max"]] if judged.get("count") else []
        if cold.get("balls") == PLANNER_JUDGED_BALLS and cold.get("totalMs") is not None:
            worst.append(cold["totalMs"])
        self.check(f"hub: every {PLANNER_JUDGED_BALLS}-ball route plan, the cold first one included, takes under "
                   f"{PLANNER_HUB_MAX_MS} ms", bool(judged.get("count")) and max(worst) < PLANNER_HUB_MAX_MS,
                   f"{PLANNER_JUDGED_BALLS}-ball plans {fmt(judged)}; cold first plan {cold.get('totalMs')} ms "
                   f"({cold.get('balls')} balls)",
                   PLANNER_BUDGET_PENDING if judged.get("count") else "Bench: Hub didn't reach its planner phase")
        for n, v in dig(ph, "planner", "byCase", default={}).items():
            if not n.startswith("n="):
                s.append(f"  - {n}: {fmt(v.get('totalMs'))}")
        for c in ("small", "typical", "ballScriptMax", "large", "xlarge"):
            v = dig(ph, "limelightParse", c, default={})
            s.append(f"- LLResult parse {c} ({v.get('contents')}, {v.get('jsonChars')} chars): JSONObject "
                     f"{fmt(v.get('jsonObjectMs'))}, LLResult {fmt(v.get('llResultMs'))}, "
                     f"{dig(v, 'gc', 'bytesAllocatedPerLoop', default=0):.0f} B allocated per parse")
        pl = ph.get("limelightPostLoopback", {})
        s.append(f"- Loopback POSTs (fake Limelight, no USB): pipelineSwitch {fmt(pl.get('pipelineSwitchMs'))}; "
                 f"updatePythonInputs {fmt(pl.get('updatePythonInputsMs'))}; updateRobotOrientation "
                 f"{fmt(pl.get('updateRobotOrientationMs'))}")
        for k in ("limelightPollOff", "limelightPoll100Hz", "limelightPoll200Hz", "limelightPoll250Hz"):
            v = ph.get(k, {})
            s.append(f"- {k}: interval {v.get('pollIntervalMs')} ms, {v.get('requestsServedPerSecond', 0):.0f} req/s, "
                     f"fake server {v.get('fakeServerCpuMsPerSecond', 0):.1f} ms CPU/s, process CPU "
                     f"{dig(v, 'cpu', 'processCpuPercentOfOneCore', default=0):.0f}% of a core; loop {fmt(v.get('loopPeriodMs'))}; "
                     f"workload {fmt(v.get('workloadMs'))}")
        fm = ph.get("limelightFailureModes", {})
        for k in ("refusedPipelineSwitch", "unpluggedPipelineSwitch", "http500PipelineSwitch"):
            v = fm.get(k)
            s.append(f"- {k}: {fmt(v.get('pipelineSwitchMs')) if isinstance(v, dict) else v}")
        s.append(f"- hanging Limelight: pipelineSwitch {fmt(dig(fm, 'hangingLimelight', 'pipelineSwitch', 'pipelineSwitchMs'))}, "
                 f"getStatus {fmt(dig(fm, 'hangingLimelight', 'getStatusMs'))}")
        s.append(f"- TCP listeners on {LIMELIGHT_PORT} (slothboard's Limelight proxy) and {FAKE_LIMELIGHT_PORT} (the fake): "
                 f"at connect {listeners['atConnect']}, after Bench: Hub {listeners['afterHub']}\n")
        self.summary += s

    def run_loop(self):
        secs = 4 if self.args.quick else 8
        limiter = {KEY_LIMITER: [], KEY_FLOODGATE_AMPS: []}

        def hook(now, packet):
            data = packet.get("data") or {}
            for key, values in limiter.items():
                value = telemetry_float(data.get(key))
                if value is not None and not math.isnan(value):
                    values.append(value)

        self.dash.packet_hook = hook
        try:
            o = self.run_opmode("Bench: Framework Loop", "framework_loop", {"phaseSeconds": secs}, timeout=30 * secs + 120,
                                result_files=["framework_loop"], status_every=15.0, quiet_after=secs + 4)
        finally:
            self.dash.packet_hook = None
        multipliers, amps = limiter[KEY_LIMITER], limiter[KEY_FLOODGATE_AMPS]
        limiter_detail = (f"{len(multipliers)} packets carried the multiplier, lowest {min(multipliers, default=None)}; "
                          f"{len(amps)} carried a floodgate current, highest {max(amps, default=None)} A")
        self.check("framework loop: Drivetrain's current limiter stays at 1.00 on the unwired floodgate",
                   bool(multipliers) and min(multipliers) >= 1.0 and max(amps, default=0.0) < FLOODGATE_THRESHOLD_A,
                   limiter_detail, FLOODGATE_PENDING if multipliers else
                   "no Framework Loop packet carried the Drivetrain keys, so the limiter wasn't observed")
        fw = self.pull("framework_loop", required=not o["crashed"])
        raw_o = self.run_opmode("Bench: Raw Loop", "raw_loop", {"phaseSeconds": secs}, timeout=4 * secs + 60,
                                result_files=["raw_loop"], status_every=15.0)
        raw = self.pull("raw_loop", required=not raw_o["crashed"])
        s = ["## Bench: Framework Loop vs Raw Loop\n"]
        if fw is None:
            s.append(f"Framework Loop CRASHED: {o['errorMessage']}")
        else:
            if dig(fw, "info", "driverStationConnected") is False:
                s.append(f"DS frame costs (the DS frames/s column, dsEveryLoop): {NO_DS}.\n")
            s.append("| phase | loop period | DS frames/s | top sections (mean ms) | alloc B/loop | dashboard pkts/s | KB/s |")
            s.append("|---|---|---|---|---|---|---|")
            dash = dig(o, "dashboard", "perPhase", default={})
            for name, ph in fw["phases"].items():
                top = ", ".join(f"{x['section']} {x['meanMs']:.2f}" for x in ph.get("sections", [])
                                if not x["nested"])
                d = dash.get(name, {})
                s.append(f"| {name} | {fmt(ph.get('loopPeriodMs'))} | {ph.get('dsFramesPerSecond', 0):.1f} | {top} | "
                         f"{dig(ph, 'gc', 'bytesAllocatedPerLoop', default=0):.0f} | {d.get('packetsPerSecond', 0):.1f} | "
                         f"{d.get('telemetryKBps', 0):.1f} |")
        if raw is not None:
            for name, ph in raw["phases"].items():
                s.append(f"- Raw Loop {name}: {fmt(ph.get('loopPeriodMs'))}")
        s.append(f"- Drivetrain current limiter in Framework Loop's dashboard packets ({KEY_LIMITER!r}, "
                 f"{KEY_FLOODGATE_AMPS!r}): {limiter_detail}")
        s.append("")
        self.summary += s

    def run_failure(self):
        s = ["## Bench: Failure Path\n"]
        variants = [("initialize", False), ("initializeLoop", False), ("moduleInit", False), ("onStart", True),
                    ("gameLoop", True), ("none", True)]
        shown = None
        for throw_in, start in variants:
            step = f"failure_{throw_in}"
            try:
                o = self.run_opmode("Bench: Failure Path", step, {"throwIn": throw_in}, start=start, timeout=20,
                                    expect_crash=throw_in != "none", result_files=[step], timeout_ok=True,
                                    status_every=0.5)
            except ProtocolError:
                raise
            except BenchError as e:
                self.check(f"failure {throw_in}: variant finished", False, str(e),
                           "the runner lost this variant (timeout or hub trouble); the other variants still ran")
                s.append(f"- throwIn={throw_in}: FAILED: {e}")
                self.settle()
                shown = None
                continue
            res = self.pull(step, required=False) or {}
            events = res.get("events", [])
            stops = [e.split()[1] for e in events if e.startswith("stop ")]
            log_text = (self.out / "logs" / f"{step}.log").read_text()
            line = o.get("statusErrorMessage") or ""
            expected_stops = ["A", "B"] if throw_in == "moduleInit" else ["A", "B", "C"]
            self.check(f"failure {throw_in}: module stops", stops == expected_stops, f"stops {stops}, expected {expected_stops}")
            if shown is not None:
                self.check(f"failure {throw_in}: the previous crash's Error line was cleared at INIT",
                           not mentions(o.get("initStatusError"), shown) and not mentions(line, shown),
                           f"Error line {o.get('initStatusError')!r} when {throw_in}'s INIT showed, {line!r} at its end",
                           "a crash's Error line outlived the next INIT, so the dashboard showed a stale crash")
            if throw_in == "none":
                text = FAILURE_STUB_B_STOP
                self.check("failure none: onEnd ran after the stops", "onEnd" in events, f"events {events}")
                self.check("failure none: not taken for a crash", not o["crashed"],
                           f"crashed {o['crashed']} via {o.get('crashSignal')}",
                           "the runner took the stop() failure after its own STOP for a crash")
                self.check("failure none: stub B's stop() failure surfaced",
                           "stub B stop()" in o["errorMessage"] or "stub B stop()" in log_text,
                           f"error {o['errorMessage']!r} from {o.get('errorSource')}")
                self.check("failure none: dashboard Error line shows stub B's stop() failure", mentions(line, text),
                           f"Error line {line!r}", ERROR_LINE_PENDING)
            else:
                text = FAILURE_INJECTED + ("module B init()" if throw_in == "moduleInit" else throw_in)
                seen = o.get("crashSeenAfterStartSeconds" if start else "crashSeenSeconds")
                self.check(f"failure {throw_in}: crash detected within {CRASH_DETECT_S:.0f} s of {'START' if start else 'INIT'}",
                           o["crashed"] and not o["timedOut"] and seen is not None and seen < CRASH_DETECT_S,
                           f"crashed {o['crashed']}, timed out {o['timedOut']}, seen after {seen} s via {o.get('crashSignal')}",
                           "the runner missed the crash: no 'User code threw' line from slothboard or adb, and the "
                           "status never showed the OpMode leave")
                self.check(f"failure {throw_in}: slothboard pushed the exception to the runner",
                           mentions(o.get("dashboardCrashText"), text), f"pushed {o.get('dashboardCrashText')!r}",
                           "slothboard's OpModeManager logcat monitor didn't reach the runner's socket; the crash was "
                           "found some other way")
                self.check(f"failure {throw_in}: original exception reported",
                           mentions(o["errorMessage"], text) or text in log_text,
                           f"error {o['errorMessage']!r} from {o.get('errorSource')}")
                self.check(f"failure {throw_in}: dashboard Error line shows the exception", mentions(line, text),
                           f"Error line {line!r}" + (f" {o['errorLineSeconds']} s after the crash" if "errorLineSeconds" in o
                                                     else " (the runner saw no crash)" if not o["crashed"] else
                                                     f", still empty {ERROR_LINE_WAIT_S:.0f} s after the crash"),
                           ERROR_LINE_PENDING)
                self.check(f"failure {throw_in}: stop failure attached as Suppressed",
                           "Suppressed:" in log_text and "stub B stop()" in log_text, "looked for 'Suppressed:' in the log")
                self.check(f"failure {throw_in}: onEnd not called", "onEnd" not in events, f"events {events}")
            shown = text if mentions(line, text) else None
            s.append(f"- throwIn={throw_in}: crashed={o['crashed']} (via {o.get('crashSignal')}, "
                     f"{o.get('crashSeenAfterStartSeconds', o.get('crashSeenSeconds'))} s), error={o['errorMessage']!r} "
                     f"from {o.get('errorSource')}, Error line {line!r}, events={events}")
        s.append("- The Error-line checks read GET_ROBOT_STATUS's errorMessage, which the dashboard shows as its "
                 "Error line, with no log fallback.\n")
        self.summary += s

    def run_write_recovery(self):
        o = self.run_opmode("Bench: Write Recovery", "write_recovery", timeout=60, result_files=["write_recovery"])
        res = self.pull("write_recovery", required=not o["crashed"]) or {}
        channels = res.get("channels") or {}
        s = ["## Bench: Write Recovery (a failSafe() mid-run, hub registers read with Lynx commands)\n",
             f"- battery {dig(res, 'info', 'batteryVolts')} V; loop {fmt(res.get('loopPeriodMs'))}"]
        if o["crashed"]:
            s.append(f"- CRASHED: {o['errorMessage']}")
        for name, c in channels.items():
            s.append(f"- {name} ({'EnhancedMotor/EnhancedServo' if c.get('wrapped') else 'raw SDK device'}, {c.get('kind')} "
                     f"port {c.get('port')}): at {c.get('target')} before failSafe {c.get('settledBeforeFailSafe')}; back "
                     f"{c.get('recoveredMs')} ms after it; {c.get('loopsAtTargetAfterFailSafe')} of "
                     f"{c.get('loopsAfterFailSafe')} loops at target afterwards")
        s.append("")
        self.summary += s
        if not channels:
            self.check("write recovery: bench reported its channels", False, f"error {o.get('errorMessage')!r}")
            return
        settled = [n for n, c in channels.items() if not c.get("settledBeforeFailSafe")]
        self.check("write recovery: every channel reached its commanded state before failSafe", not settled,
                   f"not at target: {settled or 'none'}",
                   "the hub didn't take the writes at all (a low battery, or a config problem), so the recovery "
                   "checks can't be judged")
        if settled:
            return
        for wrapped in (False, True):
            for name, c in channels.items():
                if bool(c.get("wrapped")) != wrapped:
                    continue
                ms = c.get("recoveredMs")
                ok = ms is not None and ms <= WRITE_RECOVERY_LIMIT_MS
                label = "EnhancedMotor" if c.get("kind") == "motor" else "EnhancedServo"
                if wrapped:
                    self.check(f"write recovery: {label} {name} reaches the hub again within "
                               f"{WRITE_RECOVERY_LIMIT_MS} ms of a failSafe", ok,
                               f"back after {ms} ms" if ms is not None else "never back in 2 s", WRITE_CACHE_PENDING)
                else:
                    self.check(f"write recovery: raw {c.get('kind')} {name} reaches the hub again within "
                               f"{WRITE_RECOVERY_LIMIT_MS} ms of a failSafe (control)", ok,
                               f"back after {ms} ms" if ms is not None else "never back in 2 s",
                               "the SDK's own re-send didn't heal a raw device either, so failSafe() did something "
                               "else on this hub and the wrapped results don't say anything about WriteCache")

    def run_handover(self):
        o = self.run_opmode("Bench: Drive Handover", "drive_handover", timeout=60, result_files=["drive_handover"])
        res = self.pull("drive_handover", required=not o["crashed"]) or {}
        cases = res.get("phases") or {}
        s = ["## Bench: Drive Handover (Drivetrain takes the drive motors back from Pedro)\n",
             f"- stick {dig(res, 'info', 'stick')}, follower capped at {dig(res, 'info', 'followerDriveCap')}; each "
             "sample is fl's SDK power and Drivetrain's commanded fl power at the end of a loop"]
        if o["crashed"]:
            s.append(f"- CRASHED: {o['errorMessage']}")

        def handed_back(case):
            return (case.get("reenableSample") is not None
                    and (case.get("samplesFromSecondAfterReenable") or 0) >= HANDOVER_MIN_SAMPLES
                    and case.get("mismatchedFromSecondAfterReenable") == 0)

        def detail(case):
            return (f"writes back on at sample {case.get('reenableSample')}; from the 2nd loop after it "
                    f"{case.get('mismatchedFromSecondAfterReenable')} of {case.get('samplesFromSecondAfterReenable')} "
                    f"loops had fl off the stick, first [loops after, fl, commanded, follower idle] "
                    f"{case.get('firstMismatch')}")

        for key, case in cases.items():
            s.append(f"- {key}: {case.get('what')}. {detail(case)}; the first loops after it {case.get('afterReenable', [])[:4]}")
        s.append("")
        self.summary += s
        control = cases.get("D") or {}
        self.check("drive handover D (control): a command that ends itself in the scheduler hands the stick back",
                   handed_back(control), detail(control),
                   "the readback can't tell the held stick from 0 on these ports (or the current limiter moved the "
                   "commanded power), so the other cases can't be judged")
        if not handed_back(control):
            return
        for key, label in HANDOVER_CASES:
            case = cases.get(key) or {}
            self.check(f"drive handover {key}: the held stick reaches fl within 2 loops of writes coming back on "
                       f"({label})", handed_back(case), detail(case), HANDOVER_PENDING)

    def run_baseline(self):
        seq = self.dash.baseline_seq
        self.dash.send({"type": "GET_CONFIG_BASELINE"})
        self.dash.wait_until(lambda: self.dash.baseline_seq > seq, 10, "the config baseline reply")
        baseline = sorted(dig(self.dash.config_baseline, "__value", default=None) or {})
        config = sorted(dig(self.dash.config_root, "__value", default=None) or {})
        missing = [c for c in config if c not in baseline and c != "__hardware__"]
        self.results["baseline"] = {"baselineCategories": baseline, "configCategories": config}
        self.check(f"config baseline: GET_CONFIG_BASELINE lists {BASELINE_CATEGORY}", BASELINE_CATEGORY in baseline,
                   f"{len(baseline)} categories in the baseline, {len(config)} in the config; missing from the "
                   f"baseline: {missing[:8]}{' ...' if len(missing) > 8 else ''}", BASELINE_PENDING)
        self.summary += ["## Config baseline\n", f"- baseline categories: {baseline}",
                         f"- config categories missing from it: {missing}\n"]

    def run_overlay_nan(self):
        arrivals = []

        def hook(now, packet):
            loop = telemetry_float((packet.get("data") or {}).get("Bench loop"))
            if loop is not None:
                arrivals.append(int(loop))

        self.dash.packet_hook = hook
        try:
            o = self.run_opmode("Bench: Overlay NaN", "overlay_nan", timeout=60, result_files=["overlay_nan"])
        finally:
            self.dash.packet_hook = None
            if self.dash.fatal is None:
                self.log("restarting the Robot Controller app so a dead dashboard telemetry thread can't outlive "
                         "this step")
                self.activate_config(self.active_config())
        res = self.pull("overlay_nan", required=not o["crashed"]) or {}
        start, end, last = res.get("nanStartLoop"), res.get("nanEndLoop"), res.get("lastLoop")
        seen = max(arrivals, default=None)
        after = sum(1 for loop in arrivals if end is not None and loop > end)
        self.check("overlay NaN: dashboard telemetry still flows 5 s after NaN overlay ops",
                   last is not None and seen is not None and seen >= last,
                   f"NaN circles in loops {start} to {end}; the bench reached loop {last} {res.get('afterSeconds')} s "
                   f"later; the newest loop that reached the runner was {seen} ({after} packets after the NaN ones)"
                   + ("; BENCH DONE came from logcat" if o.get("telemetryLost") else ""), OVERLAY_NAN_PENDING)
        self.summary += ["## Bench: Overlay NaN (run last)\n",
                         f"- a NaN circle in every overlay from loop {start} to {end} (only the last packet of each "
                         f"slothboard batch keeps its overlay, so one loop alone could be dropped unsent); bench done "
                         f"at loop {last}; newest loop received {seen}; packets after the NaN ones {after}",
                         f"- status warning after it: {o.get('warningMessage')!r}; error {o.get('errorMessage')!r}",
                         "- the runner restarted the app afterwards\n"]

    def run_planner_stop(self):
        s = ["## Bench: Planner Stop (STOP while Vision Ball Collection-style replanning runs in INIT)\n",
             "| balls | STOP sent after the next plan began | plans | STOP landed | plan left at STOP (ms) | STOP to stop() "
             "(ms) | runner STOP to idle (s) | app restarted | stuck markers |",
             "|---|---|---|---|---|---|---|---|---|"]
        trials = []
        for balls in PLANNER_STOP_BALLS:
            for delay in PLANNER_STOP_DELAYS_S:
                trial = self._planner_stop(balls, delay)
                trials.append(trial)
                if "failed" in trial:
                    s.append(f"| {balls} | {delay} s | FAILED: {trial['failed']} | | | | | | |")
                    continue
                landed = "not seen"
                if trial["planAtStop"]:
                    landed = f"in plan {trial['planAtStop']}" + (f", {trial['msIntoPlan']:.0f} ms into it"
                                                                 if trial["msIntoPlan"] is not None else "")
                elif trial["stopSeen"]:
                    landed = "between plans"
                if trial.get("warmSeconds") is None:
                    landed += f" (never reached {PLANNER_STOP_WARM} plans; STOP came after the 10 s timeout)"
                s.append(f"| {balls} | {delay} s | {len(trial['plans'])} | {landed} | {trial.get('planLeftMs')} | "
                         f"{trial.get('stopToStopPassMs')} | {trial['stopSeconds']} | {trial['restarted']} | "
                         f"{len(trial['markers'])} |")
        s.append("")
        for balls in PLANNER_STOP_BALLS:
            ran = [t for t in trials if t["balls"] == balls and "failed" not in t]
            plans = [ms for t in ran for ms in t["plans"]]
            firsts = [round(t["plans"][0]) for t in ran if t["plans"]]
            cold = [round(t["plans"][0]) for t in ran if t["firstPlanInProcess"] and t["plans"]]
            s.append(f"- {balls} balls ({ran[0]['layout'] if ran else 'no trial ran'}): every plan {fmt_plans(plans)}; "
                     f"first plan of each trial {firsts} ms; first plans in a fresh app process {cold} ms")
            restarted = [f"{t['delay']} s" for t in ran if t["restarted"] or t["markers"]]
            mid = [t for t in ran if t["planAtStop"]]
            left = max((t["planLeftMs"] for t in mid if t.get("planLeftMs") is not None), default=None)
            self.check(f"planner stop {balls} balls: STOP during replanning never restarts the app",
                       len(ran) == len(PLANNER_STOP_DELAYS_S) and not restarted,
                       f"{len(ran)} of {len(PLANNER_STOP_DELAYS_S)} trials ran, {len(mid)} with STOP mid-plan"
                       + (f" (at most {left:.0f} ms of plan left at STOP)" if left is not None else "")
                       + f"; plans {fmt_plans(plans)}; restarts or stuck markers after STOP at {restarted or 'none'}",
                       PLANNER_STOP_PENDING if len(ran) == len(PLANNER_STOP_DELAYS_S) else
                       "a trial didn't finish (see the table), so not every STOP was tried")
            slow = [f"{t['delay']} s" for t in ran if not t["stopSeconds"] < SDK_STOP_WINDOW_MS / 1000]
            self.check(f"planner stop {balls} balls: the runner sees every STOP reach idle within {SDK_STOP_WINDOW_MS} ms",
                       len(ran) == len(PLANNER_STOP_DELAYS_S) and not slow,
                       f"STOP to idle {[t['stopSeconds'] for t in ran]} s; at or over {SDK_STOP_WINDOW_MS} ms after STOP "
                       f"at {slow or 'none'}",
                       PLANNER_BUDGET_PENDING if len(ran) == len(PLANNER_STOP_DELAYS_S) else
                       "a trial didn't finish (see the table), so not every STOP was timed")
            if balls == PLANNER_JUDGED_BALLS:
                p99 = percentile(plans, 99) if plans else None
                self.check(f"planner stop {balls} balls: per-plan p99 under {PLANNER_PLAN_P99_MS} ms",
                           p99 is not None and p99 < PLANNER_PLAN_P99_MS,
                           f"p99 {p99:.0f} ms; plans {fmt_plans(plans)}" if plans else "no trial recorded a plan",
                           PLANNER_BUDGET_PENDING if plans else "no trial ran far enough to plan")
        s.append("- The bench replans the crafted layout every 250 ms as Vision Ball Collection does when the balls "
                 "keep moving (back to back once a plan takes longer), and records when the SDK's stopRequested flips; "
                 "STOP is sent the set delay after the runner sees a plan finish, and the SDK gives STOP "
                 f"{SDK_STOP_WINDOW_MS} ms (+100) to finish the running init_loop() and stop() before it restarts the "
                 "app.\n")
        self.summary += s

    def _planner_stop(self, balls, delay):
        run_id = f"b{balls}_d{int(delay * 1000)}"
        step = f"planner_stop_{run_id}"
        rec = {"balls": balls, "delay": delay}

        def plans():
            return telemetry_float(self.dash.data.get(KEY_PLANS)) or 0

        def in_init():
            rec["warmSeconds"] = self.wait_or_crash(lambda: plans() >= PLANNER_STOP_WARM, PLANNER_STOP_WARM_WAIT_S)
            if rec["warmSeconds"] is None:
                return
            seen = plans()
            rec["nextPlanSeconds"] = self.wait_or_crash(lambda: plans() > seen, 60)
            if rec["nextPlanSeconds"] is not None:
                time.sleep(delay)

        try:
            o = self.run_opmode("Bench: Planner Stop", step, {"balls": balls, "runId": run_id}, start=False,
                                during_init=in_init, timeout=10, timeout_ok=True, result_files=[step])
        except ProtocolError:
            raise
        except BenchError as e:
            self.settle()
            return dict(rec, failed=str(e))
        self.results[step]["runner"] = rec
        if o["crashed"]:
            return dict(rec, failed=f"crashed: {o['errorMessage']}")
        res = self.pull(step, required=False) or {}
        stop = res.get("stop") or {}
        rec.update(plans=res.get("planMs") or [], layout=dig(res, "info", "layout"),
                   firstPlanInProcess=dig(res, "info", "firstPlanInProcess"), stopSeen=bool(stop.get("stopSeen")),
                   planAtStop=stop.get("planRunningAtStop"), msIntoPlan=stop.get("msIntoPlanAtStop"),
                   planLeftMs=stop.get("planLeftAtStopMs"), stopToStopPassMs=stop.get("stopToStopPassMs"),
                   stopSeconds=round(o.get("stopSecondsRunnerSide", float("nan")), 2),
                   restarted=bool(o.get("appRestarted") or o.get("restartedDuringStop")),
                   markers=o.get("logMarkers") or [])
        if not stop.get("stopSeen"):
            log_file = self.out / "logs" / f"{step}.log"
            for m in PLANNER_STOP_SEEN.finditer(log_file.read_text() if log_file.exists() else ""):
                if m.group(1) == run_id:
                    rec.update(stopSeen=True, planAtStop=int(m.group(3)) if m.group(3) else None,
                               msIntoPlan=float(m.group(2)) if m.group(2) else None)
        return rec

    def send_on_side_socket(self, obj):
        side = SideSocket(self.args.host, DASHBOARD_PORT)
        try:
            side.send(obj)
            time.sleep(0.3)
        finally:
            side.close()

    def run_hardware_view(self):
        s = ["## Hardware view stale motor power (the Hardware OpMode, then Bench: Power Readback)\n"]
        rec = {}
        self.results["hardware_view_setup"] = rec
        self.wait_idle()
        self.begin_watch(HARDWARE_OPMODE, self.adb.hub_time())
        self.dash.send({"type": "INIT_OP_MODE", "opModeName": HARDWARE_OPMODE})
        self.dash.wait_until(lambda: self._status_is(HARDWARE_OPMODE) or self.crashed(), 30,
                             f"{HARDWARE_OPMODE} to INIT", poll_status_every=0.5, tick=self.adb_crash_tick)
        rec["flPowerListedSeconds"] = self.wait_or_crash(
            lambda: config_value(self.dash.config_root, HARDWARE_POWER_PATH) is not None, 10)
        if not self.crashed():
            self.dash.send({"type": "START_OP_MODE"})
            self.dash.wait_until(lambda: self._status_is(HARDWARE_OPMODE, "RUNNING") or self.crashed(), 30,
                                 f"{HARDWARE_OPMODE} to START", poll_status_every=0.5)
            time.sleep(1)
        if self._status_is(HARDWARE_OPMODE):
            self._stop(HARDWARE_OPMODE, rec)
        seq = self.dash.config_seq
        self.dash.send({"type": "GET_CONFIG"})
        self.dash.wait_until(lambda: self.dash.config_seq > seq, 10, "the dashboard's config")
        rec["flPowerListedAfterStop"] = config_value(self.dash.config_root, HARDWARE_POWER_PATH) is not None
        listed = rec["flPowerListedSeconds"] is not None
        self.check("hardware view: the Hardware OpMode listed fl's Power", listed,
                   f"Motors/fl/Power {'appeared' if listed else 'never appeared'} in the config tree; still there after "
                   f"the Hardware OpMode stopped: {rec['flPowerListedAfterStop']}",
                   "the runner couldn't set the test up, so the stale-power check didn't run")
        if not listed:
            s.append("- not run: the Hardware OpMode never listed fl's Power\n")
            self.summary += s
            return

        def in_init():
            rec["readbackReadySeconds"] = self.wait_or_crash(
                lambda: (telemetry_float(self.dash.data.get("Bench loop")) or 0) >= 20, 15)
            rec["selfCheck"] = self.dash.data.get("Bench self-check")
            self.send_on_side_socket(config_diff(HARDWARE_POWER_PATH, HARDWARE_VIEW_POWER))
            time.sleep(2)

        def running():
            time.sleep(2)
            self.send_on_side_socket(config_diff(HARDWARE_POWER_PATH, 0.0))

        o = self.run_opmode("Bench: Power Readback", "hardware_view", timeout=20, during_init=in_init,
                            during=running, result_files=["power_readback"])
        res = self.pull("power_readback", required=not o["crashed"]) or {}
        self_check = res.get("selfCheck") or {}
        windows = {"INIT": res.get("init") or {}, "running": res.get("run") or {}}
        s.append(f"- Motors/fl/Power still in the config after the Hardware OpMode stopped: {rec['flPowerListedAfterStop']}")
        s.append(f"- readback self-check: {self_check}")
        for when, w in windows.items():
            s.append(f"- {when}: fl's hub power peaked at {w.get('maxHubPower')} (SDK power {w.get('maxSdkPower')}), "
                     f"first off 0 at {w.get('firstMovedMs')} ms, over {w.get('loops')} loops")
        s.append(f"- The runner saved Motors/fl/Power = {HARDWARE_VIEW_POWER} through a second websocket in INIT, then "
                 "0.0 while running; the OpMode never commands fl after its self-check.\n")
        self.summary += s
        self.check("hardware view: Bench: Power Readback's hub readback works", self_check.get("ok") is True,
                   f"self-check {self_check}; error {o.get('errorMessage')!r}",
                   "setting fl to 0.2 and back didn't show on the hub's power register, so the stale-power check "
                   "can't be judged")
        if self_check.get("ok") is not True:
            return
        moved = {when: w.get("maxHubPower") for when, w in windows.items()
                 if w.get("maxHubPower") is None or w.get("maxHubPower") >= HARDWARE_VIEW_MOVED}
        self.check(f"hardware view: a stale Power edit of {HARDWARE_VIEW_POWER} doesn't move fl under the next OpMode",
                   not moved, f"fl's hub power peaked at {moved or 'under ' + str(HARDWARE_VIEW_MOVED)}",
                   HARDWARE_VIEW_PENDING)

    GP_PHASE_NAMES = {GP_INIT_DELIVERY: "INIT delivery", GP_RUN_DELIVERY: "delivery", GP_HOLD: "hold, one sender",
                      GP_WATCHDOG: "hold, then silence", GP_SECOND: "hold + second client"}

    def run_gamepad(self):
        s = ["## Bench: Gamepad and Bench: Gamepad Linear (dashboard gamepads)\n",
             "| OpMode | phase | loops | deliveries | lb rises | lb falls | loops at ly <= -0.9 | zero-stick loops | "
             "rest after last message (ms) | ly range |",
             "|---|---|---|---|---|---|---|---|---|---|"]
        for opmode, result, variant in GP_VARIANTS:
            self._gamepad(opmode, result, variant, s)
        s.append("")
        s.append("- The runner names each phase in gamepad2's right trigger, so a rest state from another socket or "
                 "slothboard's watchdog leaves the phase as it was. Zero-stick loops are loops reading left_stick_y "
                 "exactly 0 between two loops at full stick; rest after last message is the time from the last held "
                 "message reaching the OpMode to the stick reading 0 (slothboard's watchdog rests after 500 ms). The "
                 "Linear twin polls without sleeping and sees every state the server delivers; the iterative bench "
                 "sees the state at the start of each loop. The hold + second client rows are measured, not judged: "
                 "a second websocket sends a rest state every 200 ms while the runner drives, as a second tab with the "
                 "Gamepad view did before slothboard 6165.6, and every slothboard server so far applies every "
                 "socket's state, so held loops reading the stick as 0 and extra lb rises are expected there. 6165.6 "
                 "stops it in the client, which only --interactive sees.\n")
        self.summary += s

    def gp_send(self, phase, **pad):
        self.gp_seq += 1
        self.dash.send(gamepad_message(phase, self.gp_seq, **pad))

    def gp_hold(self, phase, seconds, every, **pad):
        end = time.monotonic() + seconds
        while True:
            self.gp_send(phase, **pad)
            if time.monotonic() + every >= end:
                return
            time.sleep(every)

    def gp_delivery(self, phase):
        self.gp_send(phase)
        time.sleep(0.1)
        t0 = time.monotonic()
        seen = None
        while time.monotonic() - t0 < GP_DELIVERY_S:
            self.gp_send(phase, left_stick_y=GP_DELIVERY_LY)
            if (seen is None and telemetry_float(self.dash.data.get("gp phase")) == phase
                    and telemetry_float(self.dash.data.get("gp ly")) == GP_DELIVERY_LY):
                seen = round(time.monotonic() - t0, 3)
            time.sleep(GP_SEND_EVERY_S)
        return seen

    def gp_mark(self, phase):
        t0 = time.monotonic()
        while time.monotonic() - t0 < 2.0:
            self.gp_send(phase)
            time.sleep(GP_SEND_EVERY_S)
            if telemetry_float(self.dash.data.get("gp phase")) == phase:
                return
        self.log(f"!! the gamepad bench didn't show phase {phase} within 2 s; its counts may land in the previous phase")

    def gp_interactive(self):
        self.log(f">>> Open http://{self.args.host}:8080/dash in a browser, pick the Custom layout and add the Gamepad "
                 "view.")
        for phase, _, text, _ in GP_INTERACTIVE:
            self.gp_mark(phase)
            self.log(f">>> {text}, then come back here.")
            input(">>> press Enter: ")
        self.gp_mark(GP_INTERACTIVE_IDLE)
        self.log(">>> Unplug the gamepad and close the dashboard tab: the next benches time the bus and count gamepad "
                 "states.")
        input(">>> press Enter: ")

    def gp_second_client(self, side):
        self.gp_send(GP_SECOND)
        time.sleep(0.1)
        t0 = time.monotonic()
        next_main, next_side = t0, t0 + GP_SECOND_OFFSET_S
        while time.monotonic() - t0 < GP_SECOND_S:
            if time.monotonic() >= next_main:
                self.gp_send(GP_SECOND, left_stick_y=-1.0, left_bumper=True)
                next_main += GP_RESEND_S
            if time.monotonic() >= next_side:
                side.send(gamepad_message())
                next_side += GP_SECOND_REST_EVERY_S
            time.sleep(max(0.0, min(next_main, next_side) - time.monotonic()))

    def _gamepad(self, opmode, result, variant, s):
        rec = {}
        self.gp_seq = 0
        interactive = self.args.interactive and variant == "linear"
        names = dict(self.GP_PHASE_NAMES)
        if interactive:
            names.update({code: label for code, label, _, _ in GP_INTERACTIVE})

        def in_init():
            rec["probeReadySeconds"] = self.wait_or_crash(lambda: self.dash.data.get("gp phase") is not None, 15)
            rec["initSeenSeconds"] = self.gp_delivery(GP_INIT_DELIVERY)
            self.gp_send(GP_INIT_IDLE)
            time.sleep(0.3)

        def running():
            rec["runSeenSeconds"] = self.gp_delivery(GP_RUN_DELIVERY)
            self.gp_send(GP_RUN_IDLE)
            time.sleep(0.5)
            self.gp_send(GP_HOLD)
            time.sleep(0.1)
            self.gp_hold(GP_HOLD, GP_HOLD_S, GP_RESEND_S, left_stick_y=-1.0, left_bumper=True)
            self.gp_send(GP_HOLD_IDLE)
            time.sleep(0.5)
            self.gp_send(GP_WATCHDOG)
            time.sleep(0.1)
            self.gp_hold(GP_WATCHDOG, GP_WATCHDOG_HOLD_S, GP_SEND_EVERY_S, left_stick_y=-1.0, left_bumper=True)
            time.sleep(GP_WATCHDOG_SILENCE_S)
            self.gp_send(GP_WATCHDOG_IDLE)
            side = SideSocket(self.args.host, DASHBOARD_PORT)
            try:
                time.sleep(0.5)
                self.gp_second_client(side)
            finally:
                side.close()
            self.gp_send(GP_SECOND_IDLE)
            time.sleep(0.5)
            if interactive:
                self.gp_interactive()
            self.gp_send(GP_DONE)

        o = self.run_opmode(opmode, result, timeout=10, timeout_ok=True, during_init=in_init, during=running,
                            result_files=[result])
        self.results[result]["runner"] = rec
        res = self.pull(result, required=False) or {}
        phases = res.get("phases", {})
        for code, label in names.items():
            ph = phases.get(f"p{code}")
            if ph is None:
                s.append(f"| {opmode} | {label} | never seen | | | | | | | |")
                continue
            s.append(f"| {opmode} | {label} | {ph.get('loops')} | {ph.get('deliveries')} | {ph.get('lbRises')} | "
                     f"{ph.get('lbFalls')} | {ph.get('strongNegLoops')} | {ph.get('zeroStickLoops')} | "
                     f"{ph.get('restAfterLastDeliveryMs')} | {ph.get('minLy')} to {ph.get('maxLy')} |")
        if o["crashed"]:
            s.append(f"| {opmode} | CRASHED: {o['errorMessage']} | | | | | | | | |")
        name = f"gamepad {variant}"
        delivered = []
        for code, when, key in ((GP_INIT_DELIVERY, "INIT", "initSeenSeconds"),
                                (GP_RUN_DELIVERY, "RUNNING", "runSeenSeconds")):
            ph = phases.get(f"p{code}") or {}
            ok = (ph.get("deliveryValueLoops") or 0) > 0 and ph.get("running") is (when == "RUNNING")
            delivered.append(ok)
            self.check(f"{name}: dashboard gamepad delivered in {when}", ok,
                       f"{ph.get('deliveryValueLoops', 0)} of {ph.get('loops', 0)} loops read left_stick_y "
                       f"{GP_DELIVERY_LY}; telemetry showed it {rec.get(key)} s after the first send",
                       GP_NOT_DELIVERED[variant])
        if not any(delivered):
            s.append(f"| {opmode} | not delivered: the hold and watchdog checks were skipped | | | | | | | | |")
            return
        for code, label, _, fail_means in GP_INTERACTIVE if interactive else ():
            ph = phases.get(f"p{code}") or {}
            neg, pos, zero = ph.get("strongNegLoops") or 0, ph.get("strongPosLoops") or 0, ph.get("zeroStickLoops") or 0
            self.check(f"gamepad interactive: {label} reads forward (left_stick_y <= -0.9) with no zero-stick loops",
                       neg > 0 and pos == 0 and zero == 0,
                       f"{neg} loops at left_stick_y <= -0.9, {pos} at >= 0.9, {zero} reading 0 between full-stick "
                       f"loops; left_stick_y {ph.get('minLy')} to {ph.get('maxLy')}", fail_means)
        hold = phases.get(f"p{GP_HOLD}") or {}
        self.check(f"{name}: one press is one edge and a held stick never reads 0",
                   hold.get("lbRises") == 1 and hold.get("lbFalls") == 0 and hold.get("zeroStickLoops") == 0
                   and (hold.get("strongNegLoops") or 0) > 0,
                   f"{hold.get('lbRises')} rises, {hold.get('lbFalls')} falls, {hold.get('zeroStickLoops')} zero-stick "
                   f"of {hold.get('loops')} loops",
                   "the runner was the only sender, so slothboard or the SDK's gamepad copy dropped or reordered input")
        watchdog = phases.get(f"p{GP_WATCHDOG}") or {}
        rest = watchdog.get("restAfterLastDeliveryMs")
        self.check(f"{name}: sticks rest within {GP_WATCHDOG_LIMIT_MS} ms of the runner's last message",
                   rest is not None and rest <= GP_WATCHDOG_LIMIT_MS,
                   f"left_stick_y read 0 {rest} ms after the last held message reached the OpMode" if rest is not None
                   else f"left_stick_y never read 0 in the {GP_WATCHDOG_SILENCE_S} s of silence",
                   "slothboard's 500 ms gamepad watchdog didn't rest the pads, so a dashboard client that drops "
                   "leaves its last stick held")

    def run_webcam(self):
        o = self.run_opmode("Bench: Webcam", "webcam", {"subPhaseSeconds": 3 if self.args.quick else 5}, timeout=600,
                            result_files=["webcam"])
        wc = self.pull("webcam", required=not o["crashed"])
        s = ["## Bench: Webcam\n"]
        if wc is None:
            s.append(f"CRASHED: {o['errorMessage']}")
        else:
            ph = wc["phases"]
            for r in dig(ph, "rawOpenClose", "runs", default=[]):
                s.append(f"- raw: open {r['openMs']:.0f} ms, startStreaming {r['startStreamingMs']:.0f} ms, first frame "
                         f"{r['firstFrameAfterStartMs']:.0f} ms, {r['fps']:.1f} fps, close {r['closeMs']:.0f} ms")
            for r in dig(ph, "asyncClose", "runs", default=[]):
                s.append(f"- async close: call {r['closeAsyncCallMs']:.1f} ms, done {r['closeAsyncDoneMs']:.0f} ms")
            ct = ph.get("cellTipProcessing", {})
            s.append(f"- session first frame {ct.get('sessionFirstFrameMs')} ms; close {ct.get('closeMs')} ms, release {ct.get('releaseMs')} ms")
            dash = dig(o, "dashboard", "perPhase", default={})
            ages = []
            for k, label in (("noCamera", "noCamera"), ("detectionOnStreamOn", "detectionOnStreamOn"),
                             ("detectionOffStreamOn", "detectionOffStreamOn"), ("detectionOnStreamOff", "detectionOnStreamOff"),
                             ("detectionOnStreamOnDsPreviewRequests", "dsPreviewRequests")):
                v = ct.get(k, {})
                d = dash.get(f"cellTip:{label}", {})
                age = v.get("captureAgeMs") or {}
                if age.get("count"):
                    ages.append(age)
                s.append(f"- {k}: processFrame {fmt(v.get('processFrameMs'))}; fps {dig(v, 'cameraFps', 'p50')}; loop "
                         f"{fmt(v.get('loopPeriodMs'))}; workload {fmt(v.get('workloadMs'))}; dashboard images "
                         f"{d.get('imagesPerSecond', 0):.1f}/s ({d.get('imageKBps', 0):.0f} KB/s)"
                         + (f"; capture age (nanoTime at processFrame minus captureTimeNanos) min {age['min']:.1f} / "
                            f"{fmt(age)}" if age.get("count") else ""))
            low = min((a["min"] for a in ages), default=None)
            high = max((a["p95"] for a in ages), default=None)
            gate = ct.get("maxStalenessMs")
            self.check("webcam: captureTimeNanos is on System.nanoTime()'s clock (capture age from 0 to under "
                       "CellTipCamera.maxStalenessMs, p95)", bool(ages) and gate is not None and low >= 0 and high < gate,
                       f"capture age from {low:.1f} ms, p95 up to {high:.1f} ms, max {max(a['max'] for a in ages):.1f} ms "
                       f"over {sum(a['count'] for a in ages)} frames; maxStalenessMs {gate}"
                       if ages else "no frame recorded a capture age",
                       CAPTURE_CLOCK_PENDING if ages else "the cell-tip phases processed no frames")
            wr = ct.get("webcamControlWrites", {})
            s.append(f"- WebcamControls update() with writes {fmt(wr.get('updateWithChangesMs'))}; without {fmt(wr.get('updateWithoutChangesMs'))}")
            dp = dash.get("cellTip:dsPreviewRequests", {})
            if dp:
                self.check("dashboard stream keeps updating while the DS preview requests frames",
                           dp.get("imagesPerSecond", 0) > 2, f"{dp.get('imagesPerSecond', 0):.1f} images/s")
            for r in dig(ph, "closeAtDelay", "runs", default=[]):
                s.append(f"- close at delay {r['delayMs']} ms: frames {r.get('framesAtClose')}, close {r.get('closeMs', 0):.0f} ms, "
                         f"release {r.get('releaseMs', 0):.1f} ms")
        s.append("")
        self.summary += s
        self.run_webcam_stops()

    def run_webcam_stops(self):
        s = ["## Bench: Webcam Stop (the real framework stop path)\n",
             "| run | STOP after | cellTip stop ms | stop pass → onEnd | runner-side stop s | stuck/restart markers | app restarted |",
             "|---|---|---|---|---|---|---|"]
        for delay in (0.0, 0.3, 1.0, 2.5):
            run_id = f"init_{int(delay * 1000)}ms"
            self._webcam_stop(run_id, lambda d=delay: time.sleep(d), f"INIT + {delay} s", s)
        self._webcam_stop("started_3s", self._start_after_fresh, "START after frames, + 3 s", s)
        s.append("")
        self.summary += s

    def _start_after_fresh(self):
        self.dash.wait_until(lambda: str(self.dash.data.get("Bench fresh")) == "true", 15, "webcam frames during INIT")
        self.dash.send({"type": "START_OP_MODE"})
        time.sleep(3)

    def _webcam_stop(self, run_id, wait, label, s):
        self.wait_idle()
        name = f"webcam_stop_{run_id}"
        self.adb.bench_shell(f"rm -f {self.adb.bench_dir()}/{name}.json")
        self.adb.push_json({"runId": run_id}, "params.json", self.out / "logs" / "params.json")
        hub_t0 = self.adb.hub_time()
        outcome = {"step": name, "opmode": "Bench: Webcam Stop", "pidBefore": self.step_pid()}
        self.begin_watch("Bench: Webcam Stop", hub_t0)
        self.dash.reset_telemetry()
        self.dash.send({"type": "INIT_OP_MODE", "opModeName": "Bench: Webcam Stop"})
        self.dash.wait_until(lambda: self._status_is("Bench: Webcam Stop") or self.crashed(), 20,
                             "Bench: Webcam Stop to INIT", poll_status_every=0.25)
        crashed = self.crashed()
        if not crashed:
            try:
                wait()
            finally:
                self._stop("Bench: Webcam Stop", outcome)
            time.sleep(2)
        self._finish(outcome, name, hub_t0, crashed=crashed, expect_crash=False, stop=False)
        res = self.pull(name, required=False) or {}
        ok = not (outcome["appRestarted"] or outcome["logMarkers"] or crashed or outcome.get("errorMessage"))
        self.check(f"8b stop {run_id}: no force-stop, restart or error", ok,
                   f"markers {outcome['logMarkers'][:3]}, restarted {outcome['appRestarted']}, "
                   f"error {outcome.get('errorMessage')!r}")
        pass_ms = None
        if "onEndMs" in res and "moduleStopsStartMs" in res:
            pass_ms = round(res["onEndMs"] - res["moduleStopsStartMs"], 1)
        s.append(f"| {run_id} | {label} | {res.get('cellTipStopMs')} | {pass_ms} | "
                 f"{outcome.get('stopSecondsRunnerSide', 0):.2f} | {len(outcome['logMarkers'])} | {outcome['appRestarted']} |")
        time.sleep(1.5)

    def run_absent_webcam(self):
        self.activate_config(CONFIG_NO_WEBCAM, self.main_config(webcam=FAKE_WEBCAM_SERIAL))
        o = self.run_opmode("Bench: Webcam Stop", "webcam_absent", {"runId": "absent"}, start=False, timeout=25,
                            expect_crash=True, result_files=["webcam_stop_absent"], timeout_ok=True)
        res = self.pull("webcam_stop_absent", required=False) or {}
        log_text = (self.out / "logs" / "webcam_absent.log").read_text()
        err = o.get("errorMessage") or ""
        self.check("webcam absent: INIT fails with aprilTagDetector's open error", o["crashed"] and "aprilTagDetector failed to open" in err,
                   f"crashed {o['crashed']} via {o.get('crashSignal')}, error {err!r} from {o.get('errorSource')}",
                   "INIT didn't fail with CellTipCamera's open error, or the runner missed the crash")
        self.check("webcam absent: no second 'stopStreaming() called, but camera is not opened'",
                   "stopStreaming() called, but camera is not opened" not in log_text, "searched the INIT-to-stop log")
        self.check("webcam absent: CellTipCamera.stop() ran (pipeline released)", "cellTipStopMs" in res, f"result {res}")
        self.summary += ["## Webcam absent\n", f"- error: {err!r}", f"- stop timings: {res}\n"]

    def run_webcam_failure(self):
        nonce = f"{os.getpid()}-{time.time_ns() % 10 ** 10}"
        o = self.run_opmode("Bench: Webcam Thread Failure", "webcam_failure", {"nonce": nonce}, timeout=40,
                            timeout_ok=True, expect_crash=True, result_files=["webcam_failure"], status_every=0.5)
        res = self.pull("webcam_failure", required=False) or {}
        crash_at, index = self.watch["crashAt"], self.watch["logIndex"]
        with self.dash.cond:
            pushed = [at for e, at in zip(self.dash.opmode_log[index:], self.dash.opmode_log_at[index:])
                      if nonce in str(e.get("message", ""))]
        pushed_after = round(pushed[0] - crash_at, 2) if pushed and crash_at is not None else None
        error_line = o.get("statusErrorMessage") or ""
        line_after = o.get("errorLineSeconds") if nonce in error_line else None
        log_text = (self.out / "logs" / "webcam_failure.log").read_text()
        camera_lines = [line for line in log_text.splitlines() if " OpenCvCamera" in line and nonce in line]
        manager_lines = [line for line in logcat_messages(log_text, OPMODE_MANAGER_TAG) if nonce in line]
        threw = res.get("msFromStartToThrow") is not None
        self.check(f"webcam thread failure: the pipeline threw on frame {WEBCAM_FAILURE_FRAME}", threw,
                   f"{res.get('framesSeen')} frames; {o.get('doneLine') or o.get('errorMessage') or 'no report'}",
                   "the webcam never delivered enough frames, so the dashboard check didn't run")
        if threw:
            reached = [t for t in (line_after, pushed_after) if t is not None and t <= ERROR_LINE_WAIT_S]
            self.check(f"webcam thread failure: the camera-thread exception reaches the dashboard within "
                       f"{ERROR_LINE_WAIT_S:.0f} s", o["crashed"] and bool(reached),
                       f"OpMode ended via {o.get('crashSignal')} {o.get('crashSeenAfterStartSeconds')} s after START; "
                       f"Error line {error_line!r}" + (f" ({line_after} s)" if line_after is not None else "")
                       + (f"; slothboard pushed the text {pushed_after} s after the stop was seen"
                          if pushed_after is not None else "; slothboard pushed no OpModeManager line with the text"),
                       WEBCAM_FAILURE_PENDING)
        self.summary += ["## Bench: Webcam Thread Failure (a pipeline that throws on the camera thread)\n",
                         (f"- the pipeline threw {res.get('msFromStartToThrow'):.0f} ms after START, on frame "
                          f"{WEBCAM_FAILURE_FRAME}; STOP requested on the OpMode thread afterwards: "
                          f"{res.get('stopRequestedAfterThrow')} (true means EasyOpenCV stopped the OpMode itself)"
                          if threw else f"- the pipeline never reached frame {WEBCAM_FAILURE_FRAME}: "
                                        f"{res.get('framesSeen')} frames"),
                         f"- the OpMode ended via {o.get('crashSignal')}, {o.get('crashSeenAfterStartSeconds')} s after "
                         f"START; dashboard Error line {error_line!r}; slothboard's OpModeManager push "
                         f"{'had it' if pushed else 'never had it'}",
                         f"- adb logcat: {len(camera_lines)} OpenCvCamera and {len(manager_lines)} OpModeManager lines "
                         f"with the exception, e.g. {(camera_lines or manager_lines or ['none'])[0][-160:]!r}\n"]

    def run_limelight(self):
        proxy_pipeline = self.args.other_pipeline if self.args.other_pipeline != 0 else 1
        params = {"ballPipeline": BALL_PIPELINE, "otherPipeline": self.args.other_pipeline, "proxyPipeline": proxy_pipeline}
        rec = {}

        def proxy_switch():
            ready = self.wait_or_crash(lambda: self.dash.data.get(KEY_PHASE) == "proxyQuery"
                                       and str(self.dash.data.get(KEY_STATUS, "")).startswith("ready"), 60)
            if ready is None:
                rec["skipped"] = "Bench: Limelight never reached its proxyQuery phase"
                return
            base = f"http://{self.args.host}:{LIMELIGHT_PORT}"
            rec["post"] = http_request("POST", f"{base}/pipeline-switch?index={proxy_pipeline}")
            t0 = time.monotonic()
            while time.monotonic() - t0 < 3:
                reply = http_request("GET", f"{base}/status")
                try:
                    reply["pipelineIndex"] = json.loads(reply.get("body") or "").get("pipelineIndex")
                except ValueError:
                    reply["pipelineIndex"] = None
                reply.pop("body", None)
                rec["statusViaProxy"] = reply
                if reply["pipelineIndex"] == proxy_pipeline:
                    break
                time.sleep(0.25)

        o = self.run_opmode("Bench: Limelight", "limelight", params, timeout=900, result_files=["limelight"],
                            during=proxy_switch)
        self.results["limelight"]["proxyRunner"] = rec
        ll = self.pull("limelight", required=False)
        proxy = dig(ll, "phases", "proxyQuery", default={})
        post = rec.get("post") or {}
        if "skipped" in rec:
            self.check("limelight proxy: the runner sent its POST through slothboard's proxy", False, rec["skipped"],
                       "Bench: Limelight crashed or stalled before its proxyQuery phase, so the proxy wasn't tried")
        else:
            answered = "code" in post
            self.check(f"limelight proxy: a POST with ?index={proxy_pipeline} through slothboard's proxy switches the "
                       "pipeline", proxy.get("reachedToMs") is not None,
                       f"POST {post.get('code', post.get('error'))} in {post.get('ms')} ms; /status through the proxy "
                       f"{rec.get('statusViaProxy')}; the Limelight's own status on the hub {proxy.get('indexChanges')} "
                       f"(from {proxy.get('from')}, reached {proxy_pipeline} at {proxy.get('reachedToMs')} ms)",
                       LIMELIGHT_PROXY_PENDING if answered else
                       f"the runner's POST to the hub's port {LIMELIGHT_PORT} got no HTTP answer, so slothboard's "
                       "Limelight proxy isn't listening there")
        s = ["## Bench: Limelight\n",
             f"- proxy query string: POST /pipeline-switch?index={proxy_pipeline} through the hub's port {LIMELIGHT_PORT} "
             f"{post}; /status through the proxy {rec.get('statusViaProxy')}; the Limelight's index on the hub "
             f"{proxy.get('indexChanges')}; switched back to {proxy.get('from')} directly: {proxy.get('restoreAccepted')}"]
        if o["crashed"]:
            s.append(f"CRASHED (the data up to the crash is in raw/limelight.json): {o['errorMessage']}")
        if ll is not None:
            ph = ll["phases"]
            s.append(f"- initial status: {ll.get('initialStatus')}; poller at INIT: {ll.get('pollerAtInit')}")
            s.append(f"- getStatus: {fmt(dig(ph, 'statusLatency', 'getStatusMs'))}")
            sy = ph.get("sync", {})
            for k in ("first", "again"):
                v = sy.get(k, {})
                s.append(f"- LimelightSync {k}: {v.get('ms')} ms, problem {v.get('problem')!r} (stamp {sy.get('stamp')})")
            if sy:
                self.check("LimelightSync put the ball pipeline on the Limelight",
                           all(dig(sy, k, "done") and dig(sy, k, "problem") is None for k in ("first", "again")),
                           f"sync phase {sy}")
            for k in ("toBallPipeline", "toOtherPipeline", "backToBallPipeline"):
                v = dig(ph, "pipelineSwitch", k, default={})
                s.append(f"- switch {k} ({v.get('target')}): {v.get('pipelineSwitchMs')} ms, accepted {v.get('accepted')}; first "
                         f"result on it {v.get('firstResultOnTargetMs')} ms; last old-pipeline result {v.get('lastResultOffTargetMs')} ms; "
                         f"{v.get('allZeroLlpythonOnTarget')} of {v.get('resultsOnTarget')} results all-zero llpython")
            pl = ph.get("postLatency", {})
            for call in ("updatePythonInputs", "updateRobotOrientation", "pipelineSwitchSameIndex"):
                ok = pl.get(f"{call}ReturnedTrue")
                sent = dig(pl, f"{call}Ms", "count")
                s.append(f"- POST {call}: {fmt(pl.get(call + 'Ms'))}; returned true {ok} of {sent}")
                if pl:
                    self.check(f"Limelight accepted every {call} POST", ok == sent,
                               f"{ok} of {sent} returned true; a false return's time is a failure's time")
            s.append(f"- POST pipelineSwitch in the switch phase: {fmt(pl.get('pipelineSwitchPhaseMs'))}")
            tc = ph.get("tsClock", {})
            s.append(f"- ts: {tc.get('tsSecondsPerHubSecond')} ts-s per hub-s (drift {tc.get('driftPpm')} ppm); poll wait "
                     f"{fmt(tc.get('pollWaitMs'))}; cl {fmt(tc.get('captureLatencyClMs'))}; tl {fmt(tc.get('targetingLatencyTlMs'))}; "
                     f"capture→arrival {fmt(tc.get('captureToArrivalMs'))}")
            rj = ph.get("realJsonParse", {})
            s.append(f"- real result JSON {rj.get('jsonChars')} chars: JSONObject {fmt(rj.get('jsonObjectMs'))}, LLResult {fmt(rj.get('llResultMs'))}")
            for k, v in dig(ph, "pollRates", default={}).items():
                if k == "what":
                    continue
                s.append(f"- poll {k}: interval {v.get('pollIntervalMs')} ms, {v.get('newFramesPerSecond', 0):.1f} new frames/s, "
                         f"staleness when seen {fmt(v.get('stalenessWhenFirstSeenMs'))}, loop {fmt(v.get('loopPeriodMs'))}, "
                         f"process CPU {dig(v, 'cpu', 'processCpuPercentOfOneCore', default=0):.0f}%")
            bb = ph.get("behindItsBack", {})
            ff = bb.get("firstFrame", {})
            s.append(f"- LimelightBallSource INIT to first frame: {ff.get('ms')} ms, frame {ff.get('gotFrame')}, problem {ff.get('problem')!r}")
            for k in ("own", "switchedBehindItsBack", "switchedBack"):
                v = bb.get(k, {})
                s.append(f"- LimelightBallSource {k}: {v.get('newFrames')} new frames, {v.get('staleLoops')} stale loops, "
                         f"update {fmt(v.get('updateMs'))}")
            if "threw" in bb:
                s.append(f"- LimelightBallSource threw: {bb['threw']}")
        s.append("")
        self.summary += s
        if self.args.interactive:
            self.run_limelight_replug(ll.get("restoredPipeline", -1) if ll is not None else -1)

    def run_limelight_replug(self, restore_pipeline):
        """The user pulls and replugs the Limelight while Bench: Limelight Fault runs the real Camera."""
        watch = FaultWatch(LIMELIGHT_FAULT_KEY)
        rec = {}

        def prompt(text):
            self.log(text)
            input(">>> press Enter: ")
            return watch.seconds_now()

        def script():
            settled = self.wait_or_crash(lambda: watch.bench_packets > 20 and not watch.active, 30)
            if settled is None:
                rec["skipped"] = (f"the Limelight was faulted before the unplug: {watch.message!r}" if watch.active
                                  else "no healthy Limelight frames within 30 s, or the OpMode crashed")
                self.log(f"replug step skipped: {rec['skipped']}")
                return
            rec["unplugEnterS"] = prompt(">>> PULL the Limelight's USB cable out, then press Enter.")
            seen = self.wait_or_crash(lambda: watch.active, 15)
            rec["faultSeenAfterUnplugEnterS"] = seen
            self.log(f"fault on the dashboard {seen:.2f} s after Enter: {watch.message!r}" if seen is not None
                     else "no Limelight fault on the dashboard within 15 s of the unplug")
            rec["replugEnterS"] = prompt(">>> PLUG the Limelight back in, then press Enter. It takes a while to boot.")
            cleared = self.wait_or_crash(lambda: not watch.active, 120)
            rec["clearSeenAfterReplugEnterS"] = cleared
            self.log(f"fault cleared on the dashboard {cleared:.1f} s after Enter" if cleared is not None
                     else "the Limelight fault didn't clear within 120 s of the replug")
            self.wait_or_crash(lambda: False, 3)

        self.dash.packet_hook = watch.on_packet
        try:
            o = self.run_opmode("Bench: Limelight Fault", "limelight_replug",
                                {"mode": "replug", "restorePipeline": restore_pipeline}, timeout=60, during=script,
                                result_files=["limelight_fault_replug"], quiet_after=10.0)
        finally:
            self.dash.packet_hook = None
        res = self.pull("limelight_fault_replug", required=False) or {}
        rec["dashboard"] = watch.summary()
        self.results["limelight_replug"]["runner"] = rec
        events = [e for e in res.get("events", []) if e.get("key") == LIMELIGHT_FAULT_KEY]
        running = [e for e in events if e.get("phase") == "running"]
        raised = next((e for e in running if e["what"] == "raise"), None)
        cleared = next((e for e in running if e["what"] == "clear"), None)
        replug = dig(res, "phases", "replug", default={})
        s = ["## Limelight unplug and replug (Bench: Limelight Fault, replug mode)\n"]
        if o["crashed"]:
            s.append(f"CRASHED: {o['errorMessage']}")
        if "skipped" in rec:
            self.check("Limelight replug step ran", False, rec["skipped"])
            s.append(f"- skipped: {rec['skipped']}")
        else:
            self.check("Limelight replug: fault raised after the unplug", raised is not None,
                       f"hub raise {raised}" if raised else f"hub events {events}")
            self.check("Limelight replug: fault cleared after the replug", cleared is not None,
                       f"hub clear {cleared}" if cleared else f"hub events {events}")
        for e in events:
            if e["what"] == "update":
                continue
            extra = f", {e['msSinceReconnect']:.0f} ms after the first poll back" if "msSinceReconnect" in e else ""
            s.append(f"- hub {e['what']} at {e.get('msSinceStart', e['ms']):.0f} ms ({e['phase']}), "
                     f"{e['msSinceLastPoll']} ms since the last successful poll{extra}: {e['message']!r}")
        s.append(f"- connection changes [ms since INIT, connected, ms since last poll]: {res.get('connectionChanges')}")
        s.append(f"- runner: fault on the dashboard {rec.get('faultSeenAfterUnplugEnterS')} s after the unplug Enter, "
                 f"cleared {rec.get('clearSeenAfterReplugEnterS')} s after the replug Enter; transitions {watch.transitions}")
        s.append(f"- loop with the fault {fmt(replug.get('loopPeriodFaultedMs'))}; without {fmt(replug.get('loopPeriodCleanMs'))}")
        s.append(f"- vision {res.get('vision')}; pipeline {res.get('restorePipeline')} restored: {res.get('restoreAccepted')}\n")
        self.summary += s

    def run_llfault(self):
        ip = LIMELIGHT_CONFIG_IP if self.limelight is None else UNREACHABLE_IP
        tag = limelight_tag(ip)
        previous = self.active_config()
        self.activate_config(CONFIG_LLFAULT, hub_config(main_devices(self.imu_type), [tag]))
        try:
            self._llfault(tag)
        finally:
            # A dead websocket is left to restore(), which reconnects first.
            if self.dash.fatal is None and self.active_config() != previous:
                self.activate_config(previous)

    def _llfault(self, tag):
        watch = FaultWatch(LIMELIGHT_FAULT_KEY)
        self.dash.packet_hook = watch.on_packet
        try:
            o = self.run_opmode("Bench: Limelight Fault", "llfault", {"mode": "absent", "runSeconds": LLFAULT_RUN_S},
                                init_hold=LLFAULT_INIT_HOLD_S, timeout=LLFAULT_RUN_S + 60,
                                result_files=["limelight_fault_absent"], quiet_after=5.0)
        finally:
            self.dash.packet_hook = None
        res = self.pull("limelight_fault_absent", required=False) or {}
        self.results["llfault"]["dashboard"] = watch.summary()
        s = ["## Bench: Limelight Fault (limelight configured, nothing answering)\n", f"- config line: `{tag}`"]
        self.check("llfault: OpMode ran without crashing", not o["crashed"], f"error {o.get('errorMessage')!r}")
        first_ms, phase = res.get("firstFaultMs"), res.get("firstFaultPhase")
        self.check(f"llfault: {LIMELIGHT_FAULT_KEY} fault raised in INIT within {LLFAULT_EXPECT_S:.0f} s",
                   first_ms is not None and first_ms <= LLFAULT_EXPECT_S * 1000 and phase in ("initialize", "init"),
                   f"first at {first_ms} ms ({phase}): {res.get('firstFaultMessage')!r}")
        events = res.get("events", [])
        clears = [e for e in events if e.get("key") == LIMELIGHT_FAULT_KEY and e["what"] == "clear"]
        running = dig(res, "phases", "running", default={})
        self.check("llfault: fault stays raised with nothing plugged in",
                   not clears and running.get("loops") is not None and running.get("loopsFaulted") == running.get("loops")
                   and watch.without_fault_after_first == 0,
                   f"{len(clears)} clears; {running.get('loopsFaulted')} of {running.get('loops')} running loops faulted; "
                   f"{watch.without_fault_after_first} bench packets without it after the first")
        faulted = running.get("loopPeriodFaultedMs") or {}
        # A per-loop POST to the missing Limelight would cost up to its 100 ms connect timeout.
        self.check("llfault: the loop keeps running with the fault active",
                   (running.get("loopsPerSecond") or 0) >= 10 and faulted.get("count", 0) > 0
                   and faulted.get("p95", float("inf")) <= 50 and faulted.get("max", float("inf")) <= 500,
                   f"{running.get('loopsPerSecond', 0):.0f} loops/s, {fmt(faulted)}")
        if o.get("dashboardSilent"):
            s.append(f"- dashboard row and overlay checks not measured: {o['dashboardSilent']}")
        else:
            self.check("llfault: the FAULT row leads every dashboard packet that carries it",
                       watch.with_fault > 0 and watch.fault_first == watch.with_fault,
                       f"first in {watch.fault_first} of {watch.with_fault} ({watch.bench_packets} bench packets); "
                       f"not first: {watch.fault_not_first}")
            self.check("llfault: the fault is drawn red on the field overlay",
                       watch.overlays_with_text > 0 and watch.overlays_red == watch.overlays_with_text
                       and watch.overlays_with_text >= watch.overlays_after_first,
                       f"{watch.overlays_with_text} overlays with the text ({watch.overlays_red} red) of "
                       f"{watch.overlays_after_first} after the first fault")
        vision = res.get("vision") or {}
        self.check("llfault: ball vision reports no balls",
                   vision.get("maxBalls") == 0 and vision.get("maxVisibleFieldBalls") == 0
                   and vision.get("liveFrameLoops") == 0, f"{vision}")
        collection = res.get("collection") or {}
        self.check("llfault: ball collection scheduled at START is SKIPPED on the fault and gives back the Drivetrain",
                   collection.get("status") == "SKIPPED" and str(collection.get("detail", "")).startswith("Limelight: ")
                   and collection.get("drivetrainWritesEnabled") is True, f"{collection}")
        s.append(f"- ball collection: {collection}")
        # Messages carrying a running time update every 100 ms; list only raises and clears.
        for e in events:
            if e["what"] == "update":
                continue
            s.append(f"- {e['what']} {e['key']} at {e['ms']:.0f} ms ({e['phase']}), isConnected {e['isConnected']}, "
                     f"ms since last poll {e['msSinceLastPoll']}: {e['message']!r}")
        s.append(f"- {sum(1 for e in events if e['what'] == 'update')} message updates")
        init = dig(res, "phases", "init", default={})
        s.append(f"- INIT loop with the fault {fmt(init.get('loopPeriodFaultedMs'))}; without {fmt(init.get('loopPeriodCleanMs'))}")
        s.append(f"- running loop with the fault {fmt(faulted)}, {running.get('loopsPerSecond', 0):.0f} loops/s; "
                 f"histogram (2 ms buckets) {dig(running, 'loopPeriodFaultedHistogram', 'counts')}")
        s.append(f"- dashboard: {watch.bench_packets} bench packets, {watch.with_fault} with the fault, first in "
                 f"{watch.fault_first}; runner-side transitions {watch.transitions}; sample {watch.samples[:1]}")
        s.append("- Driver Station rendering isn't observable without a Driver Station; only the dashboard side is checked.\n")
        self.summary += s

    def run_clock(self):
        before = self.adb.clock()
        connect = self.results.get("hubClock", {}).get("atConnect") or {}
        year = time.gmtime(before["epochS"]).tm_year if "epochS" in before else None
        unset = year is not None and year <= CLOCK_UNSET_YEAR
        interactive = self.args.interactive and unset
        if self.args.interactive and not unset:
            self.log(f"the hub's clock is already set ({year}): skipping the RC web page step; to see the jump, "
                     "power-cycle the hub and open neither a Driver Station nor the RC's web pages before the run")
        timeout_ms = CLOCK_INTERACTIVE_TIMEOUT_MS if interactive else CLOCK_TIMEOUT_MS

        def open_rc_page():
            self.wait_or_crash(lambda: False, 3)
            self.log(f">>> Open http://{self.args.host}:8080 in the Mac's browser now: the Robot Controller's own page, "
                     "which sets the hub's clock while it is unset. Wait for it to load, then press Enter.")
            input(">>> press Enter: ")

        o = self.run_opmode("Bench: Clock", "clock", {"timeoutMs": timeout_ms}, timeout=timeout_ms / 1000 + 60,
                            during=open_rc_page if interactive else None, result_files=["clock"])
        after = self.adb.clock()
        self.results["hubClock"].update(beforeClock=before, afterClock=after)
        res = self.pull("clock", required=not o["crashed"]) or {}
        ended = res.get("timeoutEndedAfterMs")
        self.check(f"clock: PathCommands.timeout({timeout_ms} ms) ends {timeout_ms} ms after START by System.nanoTime() "
                   f"(within {CLOCK_TOLERANCE_MS} ms)", ended is not None and abs(ended - timeout_ms) <= CLOCK_TOLERANCE_MS,
                   f"ended {ended} ms after START; the wall clock moved {res.get('wallMinusNanoTimeChangeMs')} ms against "
                   f"System.nanoTime() during the OpMode, jumps [ms after INIT, ms] {res.get('jumps')}",
                   CLOCK_PENDING)
        drift = None
        if "epochS" in connect and "epochS" in after:
            drift = round((after["epochS"] - after["uptimeS"]) - (connect["epochS"] - connect["uptimeS"]), 1)
        self.summary += ["## Bench: Clock (Ivy's waitMs against the hub's wall clock)\n",
                         f"- the hub's clock read {year} before the step: "
                         + ("unset since boot, so the first Driver Station heartbeat or RC web page sets it about 56 "
                            "years forward" if unset else "already set (a Driver Station or the RC's web page set it "
                                                          "since boot, or the hub kept time)"),
                         f"- PathCommands.timeout({timeout_ms} ms) ended {ended} ms after START by System.nanoTime(); "
                         f"biggest one-loop wall-clock step {res.get('biggestLoopStepMs')} ms; jumps {res.get('jumps')}"
                         + ("; you opened the RC's web page during it" if interactive else ""),
                         f"- across the run so far the wall clock moved {drift} s against the hub's uptime (connect "
                         f"{connect}, after this step {after})\n"]

    def run_pinpoint(self):
        self.activate_config(CONFIG_PINPOINT, self.main_config(pinpoint=True))
        o = self.run_opmode("Bench: Pinpoint", "pinpoint_seed", {"mode": "seed"}, init_hold=PINPOINT_INIT_HOLD_S,
                            timeout=60, timeout_ok=True, result_files=["pinpoint_seed"])
        res = self.pull("pinpoint_seed", required=False) or {}
        if res.get("present") is False:
            self.log("no Pinpoint answered on I2C bus 1: running Bench: Pinpoint Absent")
            self.summary.append("## Bench: Pinpoint\n\n- no Pinpoint answered on I2C bus 1, so the firmware, start-pose "
                                "and dropout checks were skipped and Bench: Pinpoint Absent ran instead\n")
            self.run_pinpoint_absent()
            return
        self._pinpoint_seed(o, res)
        if self.args.interactive:
            self._pinpoint_dropout()

    def _pinpoint_seed(self, o, res):
        info, seed = res.get("info") or {}, res.get("seed") or {}
        s = ["## Bench: Pinpoint (a Pinpoint on I2C bus 1 through BettaConstants.create, the follower left idle)\n",
             f"- firmware (device version) {res.get('deviceVersion')}, device id {info.get('deviceId')}; status right "
             f"after BettaConstants.create {info.get('statusAfterCreate')} ({info.get('createMs')} ms to create); "
             f"Pinpoint loop {fmt(res.get('frequencyHz'), 'Hz')}",
             f"- pod offsets x {info.get('xOffsetIn')} in, y {info.get('yOffsetIn')} in; yaw scalar {info.get('yawScalar')}",
             f"- status changes [ms since create, status, phase]: {res.get('statusChanges')}; loops per status "
             f"{res.get('statusLoops')}",
             f"- start pose {info.get('seed')} set in initialize() after Robot's placeholder: first update "
             f"{seed.get('firstOffsetIn')} in / {seed.get('firstTurnDeg')} deg off, at START {seed.get('offsetAtStartIn')} "
             f"in / {seed.get('turnAtStartDeg')} deg, worst through START + 3 s {seed.get('maxOffsetIn')} in / "
             f"{seed.get('maxTurnDeg')} deg; final pose {res.get('finalPose')}, status {res.get('finalStatus')}"]
        if o["crashed"]:
            s.append(f"- CRASHED: {o['errorMessage']}")
        s.append("")
        self.summary += s
        offset, turn = seed.get("maxOffsetIn"), seed.get("maxTurnDeg")
        held = (not o["crashed"] and seed.get("ran") is True and offset is not None and offset <= PINPOINT_SEED_OFFSET_IN
                and turn <= PINPOINT_SEED_TURN_DEG)
        self.check(f"pinpoint: a start pose set in INIT survives the Pinpoint's recalibration through START + 3 s "
                   f"(within {PINPOINT_SEED_OFFSET_IN} in and {PINPOINT_SEED_TURN_DEG} deg)", held,
                   f"worst {offset} in / {turn} deg over {seed.get('loops')} loops; final status {res.get('finalStatus')}"
                   + (f"; error {o['errorMessage']!r}" if o["crashed"] else ""),
                   PINPOINT_CRASHED if o["crashed"] else PINPOINT_SEED_PENDING)

    def _pinpoint_dropout(self):
        rec = {"warnings": []}

        def status():
            warning = (self.dash.status or {}).get("warningMessage") or ""
            if warning and warning not in rec["warnings"]:
                rec["warnings"].append(warning)
            return self.dash.data.get(KEY_PINPOINT_STATUS)

        def prompt(text):
            self.log(text)
            input(">>> press Enter: ")

        def script():
            ready = self.wait_or_crash(lambda: status() == "READY"
                                       and (telemetry_float(self.dash.data.get("Bench loop")) or 0) > 20, 15)
            if ready is None:
                rec["skipped"] = f"the Pinpoint wasn't READY before the unplug (status {status()!r}), or the OpMode crashed"
                self.log(f"Pinpoint dropout skipped: {rec['skipped']}")
                return
            prompt(">>> PULL the Pinpoint's I2C cable out of the Control Hub (leave both pods in), then press Enter.")
            rec["notReadySeenS"] = self.wait_or_crash(lambda: status() not in (None, "READY"), 10)
            self.log(f"Pinpoint status {status()!r} on the dashboard {rec['notReadySeenS']} s after Enter")
            prompt(">>> PLUG the Pinpoint's I2C cable back in, then press Enter.")
            rec["readyAgainS"] = self.wait_or_crash(lambda: status() == "READY", 30)
            self.wait_or_crash(keep_watching, 2)

        def keep_watching():
            status()
            return False

        o = self.run_opmode("Bench: Pinpoint", "pinpoint_dropout", {"mode": "dropout"}, timeout=60, during=script,
                            result_files=["pinpoint_dropout"], quiet_after=10.0)
        self.results["pinpoint_dropout"]["runner"] = rec
        res = self.pull("pinpoint_dropout", required=False) or {}
        d = res.get("dropout") or {}
        bad, silent = d.get("badLoops") or 0, d.get("silentBadLoops") or 0
        error = o.get("errorMessage") or ""
        ended_itself = "stopSecondsRunnerSide" not in o
        stopped = ended_itself and "pinpoint" in error.lower()
        s = ["## Pinpoint unplug and replug (Bench: Pinpoint, dropout mode)\n",
             f"- firmware {res.get('deviceVersion')}; first status off READY {d.get('firstNotReadyStatus')} at "
             f"{d.get('firstNotReadyMs')} ms; pose during the dropout {d.get('poseDuringDropout')} "
             f"({d.get('firstNotReadyPose')}, last READY pose {d.get('lastReadyPose')})",
             f"- {bad} loops reached the bench with the status off READY, {silent} with no Pinpoint fault; READY again "
             f"at {d.get('readyAgainMs')} ms with pose {d.get('poseWhenReadyAgain')}; after the replug {d.get('afterReplug')}",
             f"- loop while READY {fmt(res.get('loopPeriodReadyMs'))}; while not {fmt(res.get('loopPeriodNotReadyMs'))}",
             f"- dashboard: status off READY {rec.get('notReadySeenS')} s after the unplug Enter, READY "
             f"{rec.get('readyAgainS')} s after the replug Enter; warnings {rec['warnings']}",
             f"- OpMode crashed {o['crashed']}: {error!r}\n"]
        self.summary += s
        if "skipped" in rec:
            self.check("pinpoint dropout: the unplug step ran", False, rec["skipped"],
                       "the Pinpoint never read READY before the unplug prompt (pods missing, a loose cable) or the "
                       "OpMode crashed, so the dropout wasn't tried")
            return
        self.check("pinpoint dropout: a status other than READY stops the OpMode or raises a fault",
                   stopped or (bad > 0 and silent == 0),
                   f"{bad} loops off READY reached the bench, {silent} of them with no Pinpoint fault; ended before "
                   f"the runner's STOP {ended_itself} (crash seen {o['crashed']}): {error!r}",
                   PINPOINT_DROPOUT_PENDING if bad or ended_itself else
                   "the status never left READY: the cable wasn't pulled, or the SDK driver didn't notice")

    def run_pinpoint_absent(self):
        o = self.run_opmode("Bench: Pinpoint Absent", "pinpoint_absent", timeout=120,
                            result_files=["pinpoint_absent"], timeout_ok=True)
        res = self.pull("pinpoint_absent", required=False) or {}
        ph = res.get("phases", {})
        self.summary += ["## Pinpoint absent (can the real autos run on a bare hub?)\n",
                         f"- crashed: {o['crashed']} {o.get('errorMessage') or ''}",
                         f"- create: {ph.get('create')}",
                         f"- update: {fmt(dig(ph, 'update', 'updateMs'))}; threw {dig(ph, 'update', 'threw')}; poses "
                         f"{dig(ph, 'update', 'posesEvery50')}\n"]

    def run_octoquad(self):
        expansion = (self.expansion, []) if self.expansion is not None else None
        self.activate_config(CONFIG_OCTOQUAD, self.main_config(pinpoint=True, octoquad_bus=OCTOQUAD_BUS,
                                                               expansion=expansion))
        o = self.run_opmode("Bench: OctoQuad", "octoquad_seed", {"mode": "seed"}, init_hold=PINPOINT_INIT_HOLD_S,
                            timeout=60, timeout_ok=True, result_files=["octoquad_seed"])
        res = self.pull("octoquad_seed", required=False) or {}
        if res.get("present") is False:
            self.log(f"no OctoQuad answered on I2C bus {OCTOQUAD_BUS} (chip id {fmt_chip(res.get('chipId'))})")
            self.summary.append(f"## Bench: OctoQuad\n\n- no OctoQuad answered on Control Hub I2C bus {OCTOQUAD_BUS} "
                                f"(chip id {fmt_chip(res.get('chipId'))}), so the read-cost, start-pose and dropout "
                                "checks were skipped\n")
            return
        self._octoquad_seed(o, res)
        if not self.args.interactive:
            return
        if o["crashed"] or not res.get("info"):
            self.log("Bench: OctoQuad didn't get through INIT: skipping the OctoQuad unplug step")
            self.summary.append("_OctoQuad unplug and replug skipped: Bench: OctoQuad didn't get through INIT._\n")
            return
        self._octoquad_dropout()

    def _octoquad_seed(self, o, res):
        info, seed, cost = res.get("info") or {}, res.get("seed") or {}, res.get("readCostMs") or {}
        s = [f"## Bench: OctoQuad (Pedro's OctoQuadLocalizer on Control Hub I2C bus {OCTOQUAD_BUS}, follower idle)\n",
             f"- chip id {fmt_chip(res.get('chipId'))}, firmware {res.get('firmware')}; status right after the localizer "
             f"was built {info.get('statusAfterCreate')} ({info.get('createMs')} ms to build, Pedro's wait for RUNNING "
             f"included); heading axis {info.get('headingAxis')}",
             f"- parameters sent through Pedro: {info.get('params')}",
             f"- read cost before the localizer was built: readLocalizerData {fmt(cost.get('readLocalizerData'))}; "
             f"readLocalizerDataAndAllEncoderData {fmt(cost.get('readLocalizerDataAndAllEncoderData'))}; "
             f"getLocalizerStatus {fmt(cost.get('getLocalizerStatus'))}",
             f"- status changes [ms since create, status, phase]: {res.get('statusChanges')}; loops per status "
             f"{res.get('statusLoops')}; CRC failures {res.get('crcFailures')}; invalid loops {res.get('invalidLoops')}",
             f"- loop while the read was valid {fmt(res.get('loopPeriodValidMs'))}; while invalid "
             f"{fmt(res.get('loopPeriodInvalidMs'))}",
             f"- start pose {info.get('seed')} set in initialize(): first update {seed.get('firstOffsetIn')} in / "
             f"{seed.get('firstTurnDeg')} deg off, at START {seed.get('offsetAtStartIn')} in / "
             f"{seed.get('turnAtStartDeg')} deg, worst through START + 3 s {seed.get('maxOffsetIn')} in / "
             f"{seed.get('maxTurnDeg')} deg; final pose {res.get('finalPose')}, status {res.get('finalStatus')}"]
        if o["crashed"]:
            s.append(f"- CRASHED: {o['errorMessage']}")
        elif not info:
            s.append(f"- INIT never finished (timed out {o.get('timedOut')}); error {o.get('errorMessage')!r}")
        s.append("")
        self.summary += s
        stuck = OCTOQUAD_CRASHED if o["crashed"] else OCTOQUAD_HUNG if not info else None
        offset, turn = seed.get("maxOffsetIn"), seed.get("maxTurnDeg")
        held = (stuck is None and seed.get("ran") is True and offset is not None and turn is not None
                and offset <= PINPOINT_SEED_OFFSET_IN and turn <= PINPOINT_SEED_TURN_DEG)
        self.check(f"octoquad: a start pose set in INIT survives through START + 3 s (within {PINPOINT_SEED_OFFSET_IN} in "
                   f"and {PINPOINT_SEED_TURN_DEG} deg)", held,
                   f"worst {offset} in / {turn} deg over {seed.get('loops')} loops; final status {res.get('finalStatus')}"
                   + (f"; error {o['errorMessage']!r}" if o["crashed"] else ""), stuck or OCTOQUAD_SEED_PENDING)
        crc, invalid = res.get("crcFailures"), res.get("invalidLoops")
        self.check("octoquad: every loop's localizer read was valid (CRC ok and RUNNING) after the localizer was built",
                   stuck is None and bool(seed.get("loops")) and crc == 0 and invalid == 0,
                   f"{crc} CRC failures and {invalid} invalid loops over {seed.get('loops')} loops; loops per status "
                   f"{res.get('statusLoops')}", stuck or OCTOQUAD_VALID_PENDING)

    def _octoquad_dropout(self):
        rec = {"warnings": []}

        def valid():
            warning = (self.dash.status or {}).get("warningMessage") or ""
            if warning and warning not in rec["warnings"]:
                rec["warnings"].append(warning)
            value = self.dash.data.get(KEY_OCTOQUAD_VALID)
            return None if value is None else str(value)

        def prompt(text):
            self.log(text)
            input(">>> press Enter: ")

        def script():
            ready = self.wait_or_crash(lambda: valid() == "true"
                                       and (telemetry_float(self.dash.data.get("Bench loop")) or 0) > 20, 15)
            if ready is None:
                rec["skipped"] = (f"the OctoQuad's localizer reads weren't valid before the unplug (status "
                                  f"{self.dash.data.get(KEY_OCTOQUAD_STATUS)!r}), or the OpMode crashed")
                self.log(f"OctoQuad dropout skipped: {rec['skipped']}")
                return
            prompt(">>> PULL the OctoQuad's I2C cable out of the Control Hub (leave both pods in), then press Enter.")
            rec["invalidSeenS"] = self.wait_or_crash(lambda: valid() == "false", 10)
            self.log(f"OctoQuad status {self.dash.data.get(KEY_OCTOQUAD_STATUS)!r}, valid {valid()!r} on the dashboard "
                     f"{rec['invalidSeenS']} s after Enter")
            prompt(">>> PLUG the OctoQuad's I2C cable back in, then press Enter.")
            rec["validAgainS"] = self.wait_or_crash(lambda: valid() == "true", 30)
            self.wait_or_crash(keep_watching, 2)

        def keep_watching():
            valid()
            return False

        o = self.run_opmode("Bench: OctoQuad", "octoquad_dropout", {"mode": "dropout"}, timeout=60, during=script,
                            result_files=["octoquad_dropout"], quiet_after=10.0)
        self.results["octoquad_dropout"]["runner"] = rec
        res = self.pull("octoquad_dropout", required=False) or {}
        d = res.get("dropout") or {}
        bad, silent = d.get("badLoops") or 0, d.get("silentBadLoops") or 0
        error = o.get("errorMessage") or ""
        ended_itself = "stopSecondsRunnerSide" not in o
        stopped = ended_itself and "octoquad" in error.lower()
        self.summary += [
            "## OctoQuad unplug and replug (Bench: OctoQuad, dropout mode)\n",
            f"- first invalid read at {d.get('firstInvalidMs')} ms: status {d.get('firstInvalidStatus')}, CRC ok "
            f"{d.get('firstInvalidCrcOk')}; pose during the dropout {d.get('poseDuringDropout')} "
            f"({d.get('firstInvalidPose')}, last valid pose {d.get('lastValidPose')})",
            f"- {bad} loops reached the bench with an invalid read, {silent} with no OctoQuad fault; valid again at "
            f"{d.get('validAgainMs')} ms with pose {d.get('poseWhenValidAgain')}",
            f"- CRC failures {res.get('crcFailures')}; loops per status {res.get('statusLoops')}; status changes [ms since "
            f"create, status, phase] {res.get('statusChanges')}; final status {res.get('finalStatus')}",
            f"- loop while the read was valid {fmt(res.get('loopPeriodValidMs'))}; while invalid "
            f"{fmt(res.get('loopPeriodInvalidMs'))}",
            f"- dashboard: invalid {rec.get('invalidSeenS')} s after the unplug Enter, valid {rec.get('validAgainS')} s "
            f"after the replug Enter; warnings {rec['warnings']}",
            f"- OpMode crashed {o['crashed']}: {error!r}\n"]
        if "skipped" in rec:
            self.check("octoquad dropout: the unplug step ran", False, rec["skipped"],
                       "the OctoQuad's localizer reads weren't valid before the unplug prompt, or the OpMode crashed, so "
                       "the dropout wasn't tried")
            return
        self.check("octoquad dropout: an invalid localizer read stops the OpMode or raises a fault",
                   stopped or (bad > 0 and silent == 0),
                   f"{bad} loops with an invalid read reached the bench, {silent} of them with no OctoQuad fault; ended "
                   f"before the runner's STOP {ended_itself} (crash seen {o['crashed']}): {error!r}",
                   OCTOQUAD_DROPOUT_PENDING if bad or ended_itself else
                   "the localizer reads never went invalid: the cable wasn't pulled, or the reads didn't notice")

    def run_octoquad_absent(self):
        opmode, step = "Bench: OctoQuad Absent", "octoquad_absent"
        self.activate_config(CONFIG_OCTOQUAD_ABSENT, self.main_config(octoquad_bus=OCTOQUAD_ABSENT_BUS))
        self.wait_idle()
        self.adb.bench_shell(f"rm -f {self.adb.bench_dir()}/{step}.json {self.adb.bench_dir()}/{step}.json.tmp")
        self.adb.push_json({}, "params.json", self.out / "logs" / "params.json")
        hub_t0 = self.adb.hub_time()
        self.adb.mark(f"begin {step}")
        outcome = {"step": step, "opmode": opmode, "params": {}, "pidBefore": self.step_pid()}
        self.begin_watch(opmode, hub_t0)
        self.dash.reset_telemetry()
        t_init = time.monotonic()
        self.dash.send({"type": "INIT_OP_MODE", "opModeName": opmode})
        self.dash.wait_until(lambda: self._status_is(opmode) or self.crashed(), 30, f"{opmode} to INIT",
                             poll_status_every=0.5, tick=self.adb_crash_tick)
        outcome["doneSeconds"] = self.wait_or_crash(lambda: self.dash.done_line is not None, OCTOQUAD_ABSENT_WAIT_S)
        outcome["doneLine"] = self.dash.done_line
        crashed = self.crashed()
        if crashed:
            outcome["crashSignal"] = self.watch["crashSignal"]
            outcome["crashSeenSeconds"] = round(self.watch["crashAt"] - t_init, 2)
        else:
            try:
                self._stop(opmode, outcome)
            except ProtocolError:
                raise
            except BenchError as e:
                outcome["stillStuck"] = str(e)
                self.log(f"!! {opmode} is still stuck after STOP ({e}); force-stopping the Robot Controller app and "
                         "starting it again")
                self.adb.run("shell", f"am force-stop {RC_PACKAGE}; am start -n {RC_LAUNCHER}", check=False)
                outcome["restartedDuringStop"] = True
                self.wait_restart(f"force-stopping {opmode}")
        self._finish(outcome, step, hub_t0, crashed=crashed, expect_crash=True, stop=False)
        res = self.pull(step, required=False) or {}
        markers = outcome.get("logMarkers") or []
        if crashed:
            stopped = "not needed: INIT had already ended"
        elif "stillStuck" in outcome:
            stopped = "still stuck after STOP, so the runner force-stopped the app"
        elif outcome.get("restartedDuringStop") or outcome.get("appRestarted"):
            stopped = "the app restarted"
        elif outcome["doneSeconds"] is None:
            stopped = (f"the SDK force-stopped it, still in INIT {SDK_STOP_WINDOW_MS} ms after STOP, without restarting "
                       "the app")
        else:
            stopped = "the OpMode stopped"
        err = outcome.get("errorMessage") or ""
        seconds = outcome.get("crashSeenSeconds")
        stop_s = outcome.get("stopSecondsRunnerSide")
        init = (f"failed after {seconds} s: {err!r}" if crashed
                else f"BENCH DONE after {outcome['doneSeconds']:.1f} s: {outcome['doneLine']!r}"
                if outcome["doneSeconds"] is not None
                else f"no BENCH DONE and no error within {OCTOQUAD_ABSENT_WAIT_S} s")
        self.check(f"octoquad absent: INIT fails within {OCTOQUAD_ABSENT_EXPECT_S} s with an error naming the OctoQuad",
                   crashed and seconds is not None and seconds <= OCTOQUAD_ABSENT_EXPECT_S and "octoquad" in err.lower(),
                   f"INIT {init}; STOP: {stopped}",
                   OCTOQUAD_ABSENT_RETURNED if res.get("returned") is True else OCTOQUAD_ABSENT_PENDING)
        self.summary += [
            "## Bench: OctoQuad Absent (Pedro's OctoQuadLocalizer with nothing on the bus)\n",
            f"- config line on the Control Hub: `{octoquad_tag(OCTOQUAD_ABSENT_BUS)}`; chip id read there "
            f"{fmt_chip(res.get('chipId'))}",
            f"- result file: constructing {res.get('constructing')}, returned {res.get('returned')}, build "
            f"{res.get('createMs')} ms, status after it {res.get('statusAfterCreate')}",
            f"- INIT: {init}",
            f"- STOP: {stopped}; runner STOP to idle {round(stop_s, 2) if stop_s is not None else None} s; app restarted "
            f"{outcome.get('appRestarted')}" + ("" if crashed else f"; error after STOP {err!r}")
            + f"; stuck markers {len(markers)}" + (f", e.g. {markers[0][-160:]!r}" if markers else "") + "\n"]

    def run_expansion(self):
        address = self.expansion
        self.activate_config(CONFIG_EXPANSION, self.main_config(pinpoint=True, octoquad_bus=OCTOQUAD_BUS,
                                                                expansion=(address, [])))
        o, ch = self._expansion_hub("ch")
        hubs = (ch or {}).get("hubs") or []
        answered = [h for h in hubs if h.get("responding") is True]
        self.check(f"expansion: the Control Hub and the Expansion Hub at address {address} both answered",
                   any(h.get("parent") for h in answered)
                   and any(h.get("address") == address and not h.get("parent") for h in answered),
                   f"hubs {hubs}" if ch is not None else f"no result: {o.get('errorMessage')!r}")
        self.summary += self._expansion_summary(o, ch, address)
        if not self.args.interactive:
            return
        self.log(EXPANSION_MOVE)
        input(">>> press Enter: ")
        try:
            self.activate_config(CONFIG_EXPANSION_ODO, self.main_config(
                expansion=(address, [pinpoint_tag(1), octoquad_tag(OCTOQUAD_BUS)])))
            o_eh, eh = self._expansion_hub("eh")
        except BenchError:
            self.log(f">>> MOVE {EXPANSION_HOME}, then press Enter.")
            input(">>> press Enter: ")
            raise
        except BaseException:
            self.log(f"!! the run is stopping: move {EXPANSION_HOME} before the next run")
            raise
        self.log(f">>> MOVE {EXPANSION_HOME}, then press Enter.")
        input(">>> press Enter: ")
        self.summary += self._expansion_moved_summary(ch, o_eh, eh)

    def _expansion_hub(self, run_id):
        name = f"expansion_hub_{run_id}"
        o = self.run_opmode("Bench: Expansion Hub", name, {"runId": run_id, "iterations": 150 if self.args.quick else 300},
                            timeout=300, result_files=[name], status_every=30.0, quiet_after=15.0)
        return o, self.pull(name, required=not o["crashed"])

    @staticmethod
    def _expansion_summary(o, res, address):
        s = ["## Bench: Expansion Hub (the same Lynx commands on each hub; odometry on the Control Hub)\n"]
        if res is None:
            return s + [f"CRASHED: {o['errorMessage']}\n"]
        hubs = res.get("hubs") or []
        parent = next((h for h in hubs if h.get("parent")), {})
        commands = res.get("commands") or {}
        ch, eh = commands.get(str(parent.get("address"))) or {}, commands.get(str(address)) or {}
        s.append("- hubs: " + "; ".join(f"{'Control Hub' if h.get('parent') else 'Expansion Hub'} at address "
                                        f"{h.get('address')}, firmware {h.get('firmware')}, responding "
                                        f"{h.get('responding')}" for h in hubs))
        s += ["", "| command | Control Hub p50 / p95 / max (ms) | Expansion Hub p50 / p95 / max (ms) | p50 difference "
                  "(ms) |", "|---|---|---|---|"]
        s += [f"| {k} | {fmt_cell(ch.get(k))} | {fmt_cell(eh.get(k))} | {p50_diff(ch.get(k), eh.get(k))} |"
              for k in EXPANSION_COMMANDS]
        s += ["", "| odometry read | connection | p50 / p95 / max (ms) | chip id or version |", "|---|---|---|---|"]
        s += [f"| {n} | {d.get('connectionInfo')} | {fmt_cell(d.get('readMs'))} | {d.get('chipIdOrVersion')} |"
              for n, d in (res.get("devices") or {}).items()]
        passes = res.get("loopPass") or {}
        s += ["", "| one loop's reads | p50 / p95 / max (ms) |", "|---|---|"]
        s += [f"| {k} | {fmt_cell(passes.get(k))} |" for k in EXPANSION_LOOP_PASSES if k in passes]
        s.append(f"\n- {res.get('summary')}; bulk caching off on every hub while measuring\n")
        return s

    @staticmethod
    def _expansion_moved_summary(ch, o, eh):
        s = ["## Odometry on the Expansion Hub (Bench: Expansion Hub again, the Pinpoint and OctoQuad moved over)\n"]
        if eh is None:
            return s + [f"CRASHED: {o['errorMessage']}\n"]
        before, after = (ch or {}).get("devices") or {}, eh.get("devices") or {}
        s += ["| read | odometry on the Control Hub: p50 / p95 / max (ms) | on the Expansion Hub: p50 / p95 / max (ms) | "
              "p50 difference (ms) |", "|---|---|---|---|"]
        for n in ("octoquad", "pinpoint"):
            a, b = dig(before, n, "readMs"), dig(after, n, "readMs")
            s.append(f"| {n} read | {fmt_cell(a)} | {fmt_cell(b)} | {p50_diff(a, b)} |")
        for k in EXPANSION_LOOP_PASSES:
            a, b = dig(ch, "loopPass", k), dig(eh, "loopPass", k)
            s.append(f"| loop: {k} | {fmt_cell(a)} | {fmt_cell(b)} | {p50_diff(a, b)} |")
        s.append("")
        s += [f"- {n}: on the Control Hub {dig(before, n, 'connectionInfo')} (chip id or version "
              f"{dig(before, n, 'chipIdOrVersion')}); on the Expansion Hub {dig(after, n, 'connectionInfo')} "
              f"({dig(after, n, 'chipIdOrVersion')})" for n in ("octoquad", "pinpoint")]
        s.append("")
        return s

    SLOTH_DEPLOY_TIMEOUT_S = 600
    SLOTH_PUSHED_LINE = "pushed jar"

    def run_sloth(self):
        s = ["## Sloth hot-load vs INIT\n",
             "| INIT sent | app restarted | INIT outcome | code loaded | markers | deploy | push to Staged / to load end "
             "(s) | OpMode lists push to load |",
             "|---|---|---|---|---|---|---|---|"]
        list_records = []
        load_records = []
        last_jar = None
        gradlew = REPO_ROOT / "gradlew"
        if not gradlew.exists():
            raise BenchError(f"{gradlew} is missing; run from a clone of the repo")
        # "push": INIT as soon as deploySloth has pushed the jar, while it still waits for the hub's load lock.
        for when in ("push", 0.0, 0.5, 1.0, 2.0, 5.0):
            run_id = "atpush" if when == "push" else f"d{int(when * 1000)}ms"
            self.wait_idle()
            self.adb.bench_shell(f"rm -f {self.adb.bench_dir()}/sloth_probe_{run_id}.json")
            self.adb.push_json({"runId": run_id}, "params.json", self.out / "logs" / "params.json")
            hub_t0 = self.adb.hub_time()
            pid_before = self.step_pid()
            self.dash.reset_telemetry()
            self.log(f"deploySloth, INIT {'when the jar is pushed' if when == 'push' else f'{when} s after it finishes'}")
            sent = {}

            def send_init():
                sent["at"] = time.monotonic()
                self.dash.send({"type": "INIT_OP_MODE", "opModeName": "Bench: Sloth Probe"})

            deploy = self._deploy_sloth(run_id, gradlew, send_init if when == "push" else None)
            if when != "push" and not deploy["hung"]:
                time.sleep(when)
                send_init()
            if when == "push":
                label = f"at push, {deploy['secondsFromPushToEnd']:.2f} s before deploy ended"
            elif "at" in sent:
                label = f"{when} s after (actual {sent['at'] - deploy['endedAt']:.2f})"
            else:
                label = f"{when} s after (not sent)"
            outcome = "?"
            try:
                if deploy["hung"]:
                    outcome = "deploySloth hung waiting for the hub's load lock and was killed"
                elif "at" not in sent:
                    outcome = f"INIT never sent: deploySloth didn't print '{self.SLOTH_PUSHED_LINE}'"
                else:
                    self.dash.wait_until(lambda: self.dash.done_line is not None, 30, "the Sloth probe's BENCH DONE",
                                         poll_status_every=1.0)
                    outcome = f"initialized ({self.dash.done_line})"
            except BenchError as e:
                outcome = f"no BENCH DONE: {e}"
            try:
                status = self.dash.get_status(timeout=30)
                if status["activeOpMode"] == "Bench: Sloth Probe":
                    self._stop("Bench: Sloth Probe", {})
                elif status.get("errorMessage"):
                    outcome += f"; error {status['errorMessage']!r}"
            except BenchError as e:
                outcome += f"; status unavailable ({e})"
            load = self.sloth_load(hub_t0, run_id, deploy)
            op_lists = self.sloth_lists(deploy, pid_before, load)
            list_records.append(op_lists)
            self.adb.ensure_connected()
            pid_after = self.adb.pid()
            log_text = self.adb.logcat_since(hub_t0)
            (self.out / "logs" / f"sloth_{run_id}.log").write_text(log_text)
            markers = [l for l in log_text.splitlines() if any(m in l for m in STUCK_MARKERS)]
            probe = self.pull(f"sloth_probe_{run_id}", required=False) or {}
            restarted = pid_after != pid_before
            load_records.append({"runId": run_id, "restarted": restarted, "load": load})
            last_jar = sloth_jar(dig(probe, "loaded", "classLoader")) or last_jar
            self.results[f"sloth_{run_id}"] = {"when": when, "sent": label, "deploy": deploy, "pidBefore": pid_before,
                                               "pidAfter": pid_after, "restarted": restarted, "outcome": outcome,
                                               "markers": markers[:40], "probe": probe, "opModeLists": op_lists,
                                               "load": load}
            self.log(f"sloth {run_id}: restarted={restarted} {outcome}")
            listed = (f"{len(op_lists['lists'])} ({len(op_lists['bad'])} bad)" if op_lists["judged"]
                      else f"not judged: {op_lists['why']}")
            s.append(f"| {label} | {restarted} | {outcome} | {dig(probe, 'loaded', 'classLoader', default='-')} | "
                     f"{len(markers)} | {deploy['seconds']:.1f} s, exit {deploy['returncode']} | {load.get('stagedS')} / "
                     f"{load.get('endS')} ({load['result']}) | {listed} |")
            if deploy["hung"]:
                s.append("")
                self.summary += s
                self.check_sloth_lists(list_records)
                self.check_sloth_loads(load_records)
                raise BenchError(f"deploySloth hung in run {run_id}; the hub's Sloth lock needs a robot restart "
                                 f"(see logs/sloth_deploy_{run_id}.log)")
            if restarted:
                self.wait_restart(f"sloth {run_id}")
        s.append(f"- push to Staged / to load end: seconds from the runner's logcat mark when deploySloth printed "
                 f"'{self.SLOTH_PUSHED_LINE}' to Sloth's 'Staged Sloth Load' and to its '{SLOTH_LOADED}', "
                 f"'{SLOTH_CANCELLED}' or failure line (hub timestamps; Staged can come first, since Sloth starts on the "
                 "jar while adb is still writing it)")
        s.append("")
        self.summary += s
        self.check_sloth_lists(list_records)
        self.check_sloth_loads(load_records)
        self.sloth_persistence(last_jar)

    def _deploy_sloth(self, run_id, gradlew, on_pushed):
        """Runs deploySloth, calling on_pushed the moment it prints that the jar is on the hub. deploySloth then
        waits for the hub's lock file with no timeout of its own, so it is killed after SLOTH_DEPLOY_TIMEOUT_S."""
        self.adb.mark(f"{SLOTH_DEPLOY_MARK} {run_id}")
        t0 = time.monotonic()
        proc = subprocess.Popen([str(gradlew), ":TeamCode:deploySloth", "--console=plain"], cwd=REPO_ROOT,
                                env=dict(os.environ, ANDROID_SERIAL=self.adb.serial),
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
        lines = []
        pushed_at = None
        list_mark = None
        connection = None
        hung = threading.Event()
        watchdog = threading.Timer(self.SLOTH_DEPLOY_TIMEOUT_S, lambda: (hung.set(), proc.kill()))
        watchdog.start()
        try:
            for line in proc.stdout:
                lines.append(line)
                if pushed_at is None and self.SLOTH_PUSHED_LINE in line:
                    pushed_at = time.monotonic()
                    with self.dash.cond:
                        list_mark = len(self.dash.opmode_lists)
                        connection = self.dash.connection
                    if on_pushed is not None:
                        on_pushed()
                    self.adb.mark(f"{SLOTH_PUSH_MARK} {run_id}")
            proc.wait()
        finally:
            watchdog.cancel()
        ended = time.monotonic()
        (self.out / "logs" / f"sloth_deploy_{run_id}.log").write_text("".join(lines))
        if proc.returncode != 0 and not hung.is_set():
            raise BenchError(f"deploySloth failed (see logs/sloth_deploy_{run_id}.log): {''.join(lines).strip()[-600:]}")
        return {"seconds": ended - t0, "endedAt": ended, "returncode": proc.returncode, "hung": hung.is_set(),
                "secondsFromPushToEnd": ended - pushed_at if pushed_at is not None else float("nan"),
                "listMarkAtPush": list_mark, "connectionAtPush": connection, "pushedAt": pushed_at}

    def sloth_load(self, hub_t0, run_id, deploy):
        if deploy["hung"] or deploy.get("pushedAt") is None:
            return {"result": "no push"}
        begin, mark = f"{SLOTH_DEPLOY_MARK} {run_id}", f"{SLOTH_PUSH_MARK} {run_id}"
        ends = (SLOTH_LOADED, SLOTH_CANCELLED) + SLOTH_FAILED
        deadline = deploy["pushedAt"] + SLOTH_LOAD_WAIT_S
        while True:
            lines = self.adb.logcat_matches(hub_t0, begin, mark, SLOTH_STAGED, *ends).splitlines()
            lines = lines[next((i for i, line in enumerate(lines) if begin in line), len(lines)):]
            end = next((line for line in lines if any(e in line for e in ends)), None)
            if end is not None or time.monotonic() >= deadline:
                break
            time.sleep(1)
        pushed = next((line for line in lines if mark in line), None)
        staged = next((line for line in lines if SLOTH_STAGED in line), None)
        result = ("nothing" if end is None else "processed" if SLOTH_LOADED in end
                  else "cancelled" if SLOTH_CANCELLED in end else "failed")
        return {"result": result, "stagedS": logcat_delta(pushed, staged), "endS": logcat_delta(pushed, end),
                "lines": [pushed, staged, end]}

    def check_sloth_loads(self, records):
        judged = [r for r in records if not r["restarted"] and r["load"]["result"] != "no push"]
        bad = [f"{r['runId']} {r['load']['result']}" for r in judged if r["load"]["result"] != "processed"]
        ends = [r["load"]["endS"] for r in judged if r["load"].get("endS") is not None]
        self.check(f"deploySloth: every hot load logs '{SLOTH_LOADED}' within {SLOTH_LOAD_WAIT_S} s of its push",
                   bool(judged) and not bad,
                   f"{len(judged)} of {len(records)} runs judged (runs that restarted the app aren't); push to load end "
                   f"{ends} s" + (f"; not processed: {bad}" if bad else ""),
                   SLOTH_LOAD_PENDING if judged else "no run was judged: every one restarted the app or never pushed")

    def sloth_persistence(self, jar):
        out = self.adb.run("shell", f'for f in {SLOTH_DIR}/*.jar; do echo "$f $(stat -c %Y "$f")"; done; '
                                    'echo "now $(date +%s)"', check=False)
        mtimes, hub_now = {}, None
        for line in out.splitlines():
            path, _, value = line.strip().rpartition(" ")
            if not value.isdigit():
                continue
            if path == "now":
                hub_now = int(value)
            elif path.endswith(".jar"):
                mtimes[path.rsplit("/", 1)[-1]] = int(value)
        local = REPO_ROOT / SLOTH_LOCAL_JAR
        host_mtime = int(local.stat().st_mtime) if local.exists() else None
        clock = jar_clock(mtimes.get(jar), host_mtime, hub_now)
        rec = {"jar": jar, "jarMtimes": mtimes, "hubNow": hub_now, "localJarMtime": host_mtime, "jarClock": clock,
               "statOutput": out.strip()[-400:]}
        self.results["sloth_persistence"] = rec
        s = ["## Sloth hot-load across an app restart\n",
             f"- jars in {SLOTH_DIR} (mtime, s): {mtimes}; hub clock {hub_now}; to_load.jar on the Mac {host_mtime}; the "
             f"hot-loaded {jar} carries {clock}"]
        if jar is None:
            s.append("- no run's Bench: Sloth Probe ran from a hot-loaded jar, so the restart check didn't run\n")
            self.summary += s
            self.check("deploySloth: the hot-loaded jar survives an app restart", False,
                       "no run's probe named a hot-loaded jar", "no Sloth run left a hot load to check (see the table)")
            return
        self.log(f"restarting the Robot Controller app to see whether the hot-loaded {jar} survives it")
        self.activate_config(self.active_config())
        log_file = self.out / "logs" / self.switches[-1]["log"]
        boot = log_file.read_text() if log_file.exists() else ""
        update_time, newer = SLOTH_UPDATE_TIME.search(boot), SLOTH_NEWER.search(boot)
        dropped = [d for d in SLOTH_DROPPED if d in boot]
        self.run_opmode("Bench: Sloth Probe", "sloth_after_restart", {"runId": "afterrestart"}, start=False, timeout=30,
                        result_files=["sloth_probe_afterrestart"])
        loader = dig(self.pull("sloth_probe_afterrestart", required=False), "loaded", "classLoader")
        rec.update(lastUpdateTimeMs=int(update_time.group(1)) if update_time else None,
                   keptAsNewerMs=int(newer.group(1)) if newer else None, dropped=dropped, loaderAfter=loader)
        s.append(f"- after the restart: package lastUpdateTime {rec['lastUpdateTimeMs']} ms; Sloth "
                 + (f"kept the jar as newer (mtime {newer.group(1)} ms, delta {newer.group(2)} ms)" if newer
                    else f"logged {dropped or 'neither keeping nor dropping it'}")
                 + f"; Bench: Sloth Probe then ran from {loader}\n")
        self.summary += s
        self.check("deploySloth: the hot-loaded jar survives an app restart", sloth_jar(loader) == jar,
                   f"{jar} before the restart, {sloth_jar(loader) or loader} after; Sloth at boot: "
                   + ("kept it as newer" if newer else f"{dropped or 'no keep or drop line'}") + f"; jar clock: {clock}",
                   SLOTH_PERSIST_PENDING)

    def sloth_lists(self, deploy, pid_before, load):
        mark = deploy.get("listMarkAtPush")
        if deploy["hung"] or mark is None:
            return {"judged": False, "why": "deploySloth hung" if deploy["hung"] else "no push"}
        self.adb.ensure_connected()
        if self.adb.pid() != pid_before:
            return {"judged": False, "why": "the app restarted"}
        if load["result"] != "processed":
            return {"judged": False, "why": f"no '{SLOTH_LOADED}' in logcat within {SLOTH_LOAD_WAIT_S} s"
                    if load["result"] == "nothing" else f"the load ended {load['result']}"}
        time.sleep(SLOTH_LIST_SETTLE_S)
        if self.adb.pid() != pid_before:
            return {"judged": False, "why": "the app restarted"}
        with self.dash.cond:
            recorded = list(self.dash.opmode_lists[mark:])
        before = deploy["connectionAtPush"]
        kept = [(connection, names) for _, connection, since_connect, names in recorded
                if connection != before or since_connect >= SLOTH_ON_OPEN_S]
        lists = [names for _, names in kept]
        bad = [names for names in lists if SLOTH_PROBE_OPMODE not in names or DASH_TOGGLE_OPMODE in names]
        return {"judged": True, "lists": lists, "bad": bad, "listsRightAfterConnect": len(recorded) - len(kept),
                "listsOnReconnect": sum(1 for connection, _ in kept if connection != before)}

    def check_sloth_lists(self, records):
        judged = [r for r in records if r["judged"]]
        lists = [names for r in judged for names in r["lists"]]
        bad = [names for r in judged for names in r["bad"]]
        why = sorted({r["why"] for r in records if not r["judged"]})
        reconnect = sum(r["listsOnReconnect"] for r in judged)
        self.check(f"deploySloth: every OpMode list broadcast during a hot load has the TeamCode OpModes and no "
                   f"'{DASH_TOGGLE_OPMODE}'", bool(lists) and not bad,
                   f"{len(lists)} lists between push and load in {len(judged)} of {len(records)} runs"
                   + (f" ({reconnect} sent on a socket reconnected after the push)" if reconnect else "")
                   + f", {len(bad)} without '{SLOTH_PROBE_OPMODE}' or with '{DASH_TOGGLE_OPMODE}'"
                   + (f", e.g. {bad[0][:6]}" if bad else "") + (f"; runs not judged: {why}" if why else ""),
                   SLOTH_LIST_PENDING)

    # --- driver -------------------------------------------------------------------------------------------------------

    def restore(self):
        if self.dash is None:
            return
        try:
            if not self.configs_touched:
                self.log("no hardware config was changed; nothing to restore")
                return
            self.log("restoring the original hardware config")
            self.dash.recover_for_restore()
            self.settle()
            shipped = self.original_shipped and self.shipped_config()
            target = self.original if self.original and (shipped or not self.original_shipped) else CONFIG_PROBE
            if self.active_config() != target or shipped:
                self.activate_config(target, None if self.original else hub_config([]), shipped or None)
            if not self.args.keep_configs and self.remove_bench_configs(keep=target):
                # slothboard lists configs from memory and re-reads the folder on a restart.
                self.activate_config(target, config=shipped or None)
            self.log(f"hardware config {self.active_config()} is active")
        finally:
            folder = self.out / "raw" / "hub_bench_folder"
            if self.adb.private:
                folder.mkdir(parents=True, exist_ok=True)
                names = self.adb.bench_shell(f"ls {PRIVATE_BENCH_DIR}", check=False).split()
                for n in names:
                    try:
                        (folder / n).write_text(self.adb.read_bench_file(n))
                    except BenchError as e:
                        self.log(f"couldn't copy {n} from the hub: {e}")
                self.log(f"copied the hub's bench folder ({len(names)} files)")
            else:
                r = self.adb._raw(["-s", self.adb.serial, "pull", HUB_BENCH_DIR, str(folder)])
                self.log(f"pulled the hub's bench folder: {(r.stdout or r.stderr).strip()[-200:]}")
            # A bench OpMode run by hand later must get its defaults, not this run's last parameters.
            self.adb.bench_shell(f"rm -f {self.adb.bench_dir()}/params.json", check=False)

    def shipped_config(self):
        ids = (self.probe or {}).get("shippedConfigs")
        if ids is None:
            self.log(f"the probe didn't list the shipped configs; re-activating {self.original} with its stored id")
            return dict(self.original_entry, isDirty=False)
        if self.original not in ids:
            self.log(f"!! {self.original} isn't shipped in this install any more; leaving the hub on {CONFIG_PROBE}: "
                     "activate a config from the dashboard's Hardware Config view")
            return None
        return {"name": self.original, "resourceId": ids[self.original], "location": "RESOURCE", "isDirty": False}

    def write_summary(self, error=None):
        lines = [f"# Bench results {self.out.name}\n", f"Command: `{' '.join(sys.argv)}`\n"]
        if error is not None:
            lines.append(f"**Run stopped early: {error}**\n")
        lines.append(f"Original hardware config: {self.original or 'none'}"
                     f"{' (shipped in res/xml)' if self.original_shipped else ''}\n")
        if self.switches:
            killed = sum(w["appKilled"] for w in self.switches)
            over_adb = sum(w["overAdb"] for w in self.switches)
            lines.append(f"Config switches: {len(self.switches)} ({over_adb} over adb, the rest through the dashboard); "
                         f"{killed} froze the app until the SDK or the Control Hub's watchdog killed it; per-switch "
                         f"logcat in logs/config_*.\n")
        killed_steps = [r["step"] for r in self.results.values() if isinstance(r, dict) and r.get("appKilled")]
        if killed_steps:
            lines.append(f"**The app froze and was killed during: {', '.join(killed_steps)}**\n")
        if self.restarts:
            lines.append(f"App restarts the runner waited out: {len(self.restarts)} "
                         f"({', '.join(r['during'] for r in self.restarts)})\n")
        lost = [f"{r['step']} ({r.get('dashboardSilent') or 'telemetry went quiet'})" for r in self.results.values()
                if isinstance(r, dict) and (r.get("telemetryLost") or r.get("dashboardSilent"))]
        if lost:
            lines.append(f"**Dashboard telemetry lost; those steps' results come from logcat and the result file: "
                         f"{'; '.join(lost)}**\n")
        if self.dash is not None:
            lines.append(f"slothboard pushed {len(self.dash.opmode_log)} OpModeManager log lines to the runner "
                         f"(its crash signal).\n")
        if self.checks:
            lines.append("## Checks\n")
            lines += [f"- {state} {name}: {detail}" + (f" (FAIL means: {means})" if state == "FAIL" and means else "")
                      for name, state, detail, means in self.checks]
            lines.append("")
        lines += self.summary
        lines.append("Raw JSON is in raw/, per-step logcat in logs/, the runner's log in run.log.")
        (self.out / "summary.md").write_text("\n".join(lines) + "\n")
        (self.out / "runner_results.json").write_text(
            json.dumps(dict(self.results, configSwitches=self.switches), indent=1, default=list))

    def run(self):
        error = None
        unexpected = None
        try:
            if self.args.interactive and not sys.stdin.isatty():
                raise BenchError("--interactive asks you to drive the dashboard's Gamepad view, to unplug and replug "
                                 "the Limelight, the Pinpoint and the OctoQuad, to move the odometry cables to the "
                                 "Expansion Hub and back, and to open the RC's web page, so it needs a terminal")
            self.connect()
            self.run_probe()
            wanted = self.selection()
            self.activate_config(CONFIG_MAIN, self.main_config())
            for name in ("baseline", "hub", "loop", "failure", "gamepad", "write_recovery", "handover", "hardware_view",
                         "webcam", "webcam_failure", "limelight", "llfault", "sloth", "clock", "pinpoint", "octoquad",
                         "octoquad_absent", "expansion", "absent", "planner_stop", "overlay_nan"):
                if name in wanted:
                    self.log(f"=== {name} ===")
                    self.guarded(name, getattr(self, f"run_{'absent_webcam' if name == 'absent' else name}"))
        except BenchError as e:
            error = str(e)
            self.log(f"STOPPED: {e}")
        except BaseException as e:
            # A runner bug or Ctrl-C: still put the config back and write what finished, then re-raise.
            unexpected = e
            error = f"{type(e).__name__}: {e}"
            self.log(f"STOPPED by {error}\n{traceback.format_exc()}")
        finally:
            try:
                self.restore()
            except Exception as e:
                error = (error + "; " if error else "") + f"restoring the config failed: {type(e).__name__}: {e}"
                self.log(f"RESTORE FAILED: {e}\n{traceback.format_exc()}")
                self.log(f"Switch the config back by hand: open http://{self.args.host}:8080/dash, add the Hardware Config "
                         f"view, and activate {self.original or 'your usual config'}.")
            self.write_summary(error)
            if self.dash is not None:
                self.dash.close()
        failed = [c for c in self.checks if c[1] == "FAIL"]
        print()
        print(f"Results: {self.out}")
        print(f"Summary: {self.out / 'summary.md'}")
        print("Nothing to send back: Claude reads that folder directly. Just say the run finished.")
        if unexpected is not None:
            raise unexpected
        if error or failed:
            print(f"{len(failed)} check(s) failed" + (f"; the run stopped early: {error}" if error else ""))
            return 1
        return 0

    def guarded(self, name, step):
        """A bench that times out or loses its result is recorded as failed and the run goes on; a protocol
        surprise, or the hub becoming unreachable, stops everything."""
        try:
            step()
        except ProtocolError:
            raise
        except BenchError as e:
            self.check(f"{name} bench completed", False, str(e))
            self.summary.append(f"## {name}\n\n**FAILED: {e}**\n")
            self.settle()
            if (name not in ("pinpoint", "octoquad", "octoquad_absent", "expansion", "absent", "overlay_nan")
                    and self.active_config() != CONFIG_MAIN):
                self.activate_config(CONFIG_MAIN, self.main_config())

    def settle(self):
        before, waited = self.known_pid, len(self.restarts)
        try:
            status = self.dash.get_status(timeout=30)
        except ProtocolError:
            raise
        except BenchError as e:
            self.log(f"no robot status ({e}); waiting for the app to come back")
            status = None
        if status and status.get("available") and status["activeOpMode"] not in ("", IDLE_OPMODE):
            self._stop(status["activeOpMode"], {})
        self.wait_ready()
        if self.note_pid() != before and len(self.restarts) == waited:
            self.restarts.append({"during": "recovery", "pid": self.known_pid})

    def selection(self):
        a = self.args
        every = ("baseline", "hub", "loop", "failure", "gamepad", "write_recovery", "handover", "hardware_view",
                 "webcam", "webcam_failure", "limelight", "llfault", "pinpoint", "octoquad", "expansion", "absent",
                 "sloth", "clock", "planner_stop", "overlay_nan")
        chosen = {n for n in every if getattr(a, n)}
        explicit = bool(chosen) or a.all
        if not chosen:
            chosen = set(every)
        if "octoquad" in chosen:
            chosen.add("octoquad_absent")
        if a.no_sloth:
            chosen.discard("sloth")
        if "expansion" in chosen and self.expansion is None:
            address = a.expansion_address
            if explicit:
                raise BenchError(f"No Expansion Hub answered at address {address}: connect one to the Control Hub's RS485 "
                                 "port and a 12 V supply, pass its address with --expansion-address N, or drop --expansion")
            self.log(f"no Expansion Hub answered at address {address}: skipping the Expansion Hub bench")
            self.summary.append(f"_Expansion Hub bench skipped: no Expansion Hub answered at address {address}._\n")
            chosen.discard("expansion")
        if "webcam" in chosen and self.webcam is None:
            if explicit:
                raise BenchError("No webcam is plugged in: plug a UVC webcam into a Control Hub USB port, or drop --webcam")
            self.log("no webcam plugged in: skipping the webcam benches")
            self.summary.append("_Webcam benches skipped: no webcam plugged in._\n")
            chosen.discard("webcam")
        if "webcam_failure" in chosen and self.webcam is None:
            if explicit:
                raise BenchError("No webcam is plugged in: plug a UVC webcam into a Control Hub USB port, or drop "
                                 "--webcam-failure")
            self.log("no webcam plugged in: skipping the webcam thread failure bench")
            self.summary.append("_Webcam thread failure bench skipped: no webcam plugged in._\n")
            chosen.discard("webcam_failure")
        if "limelight" in chosen and self.limelight is None:
            if explicit:
                raise BenchError("No Limelight answers on an Ethernet-over-USB interface: plug the Limelight 3A into a "
                                 "Control Hub USB port and wait for it to boot, or drop --limelight")
            self.log("no Limelight plugged in: skipping the Limelight bench")
            self.summary.append("_Limelight bench skipped: no Limelight plugged in._\n")
            chosen.discard("limelight")
        self.log(f"benches: {sorted(chosen)}")
        return chosen


def parse_args(argv):
    p = argparse.ArgumentParser(description="Control Hub bench runner (see scripts/bench/README.md)")
    p.add_argument("--baseline", action="store_true", help="whether slothboard reports a config baseline")
    p.add_argument("--hub", action="store_true", help="Bench: Hub (bare hub costs, planner, Limelight poller sim)")
    p.add_argument("--loop", action="store_true", help="Bench: Framework Loop and Bench: Raw Loop")
    p.add_argument("--failure", action="store_true", help="Bench: Failure Path (EnhancedOpMode's safe-state pass)")
    p.add_argument("--gamepad", action="store_true",
                   help="Bench: Gamepad and Bench: Gamepad Linear (dashboard gamepads sent over the websocket)")
    p.add_argument("--write-recovery", action="store_true",
                   help="Bench: Write Recovery (whether EnhancedMotor/EnhancedServo writes reach the hub after a failSafe)")
    p.add_argument("--handover", action="store_true",
                   help="Bench: Drive Handover (Drivetrain taking the drive motors back from Pedro)")
    p.add_argument("--hardware-view", action="store_true",
                   help="the Hardware OpMode, then a Hardware view Power edit under Bench: Power Readback")
    p.add_argument("--planner-stop", action="store_true",
                   help="Bench: Planner Stop (STOP pressed while the route planner replans on the OpMode thread)")
    p.add_argument("--overlay-nan", action="store_true",
                   help="Bench: Overlay NaN (run last; restarts the app afterwards)")
    p.add_argument("--webcam", action="store_true", help="Bench: Webcam and the Webcam Stop runs (needs a UVC webcam)")
    p.add_argument("--webcam-failure", action="store_true",
                   help="Bench: Webcam Thread Failure (a pipeline that throws on the camera thread; needs a UVC webcam)")
    p.add_argument("--limelight", action="store_true", help="Bench: Limelight (needs a Limelight 3A)")
    p.add_argument("--llfault", action="store_true",
                   help="Bench: Limelight Fault: the fault fallback with a limelight configured and nothing answering")
    p.add_argument("--pinpoint", action="store_true",
                   help="Bench: Pinpoint (firmware, status, INIT start pose; dropout with --interactive), or Bench: "
                        "Pinpoint Absent when none answers")
    p.add_argument("--octoquad", action="store_true",
                   help="Bench: OctoQuad (Pedro's OctoQuadLocalizer on Control Hub I2C bus 2: read cost, valid reads, INIT "
                        "start pose; dropout with --interactive) and Bench: OctoQuad Absent (nothing on I2C bus 3)")
    p.add_argument("--expansion", action="store_true",
                   help="Bench: Expansion Hub (the same Lynx commands and odometry reads on each hub; needs an Expansion "
                        "Hub; with --interactive, again with the odometry moved to it)")
    p.add_argument("--expansion-address", type=int, default=2, help="the Expansion Hub's module address (default 2)")
    p.add_argument("--absent", action="store_true", help="the webcam-unplugged INIT check")
    p.add_argument("--sloth", action="store_true", help="the deploySloth/INIT race (runs gradle six times)")
    p.add_argument("--clock", action="store_true",
                   help="Bench: Clock (Ivy's waitMs timeout against wall-clock jumps on the hub)")
    p.add_argument("--all", action="store_true",
                   help="every bench, and fail if the webcam, Limelight or Expansion Hub is missing")
    p.add_argument("--no-sloth", action="store_true", help="skip the Sloth race")
    p.add_argument("--interactive", action="store_true",
                   help="in the gamepad bench, has you drive the dashboard's Gamepad view (keyboard, on-screen stick, "
                        "USB pad); after the Limelight bench, prompts you to unplug and replug it and times the fault; "
                        "after Bench: Pinpoint and Bench: OctoQuad, to unplug and replug each; after Bench: Expansion "
                        "Hub, to move the Pinpoint and OctoQuad to the Expansion Hub and back; in Bench: Clock, to "
                        "open the RC's web page")
    p.add_argument("--quick", action="store_true", help="shorter phases (numbers are noisier)")
    p.add_argument("--keep-configs", action="store_true", help="leave the bench_* hardware configs on the hub")
    p.add_argument("--ports-empty", action="store_true",
                   help="Control Hub motor ports 0-3 and servo ports 0-1 are empty: run even if a config names devices "
                        "there")
    p.add_argument("--other-pipeline", type=int, default=0, help="the non-ball Limelight pipeline to switch to")
    p.add_argument("--host", default=HUB_IP)
    p.add_argument("--serial", default=None, help=f"adb serial (default {HUB_IP}:{ADB_PORT})")
    args = p.parse_args(argv)
    if not 0 <= args.other_pipeline <= 9 or args.other_pipeline == BALL_PIPELINE:
        p.error(f"--other-pipeline must be a Limelight pipeline 0-9 other than the ball pipeline {BALL_PIPELINE}")
    if not 1 <= args.expansion_address <= EXPANSION_MAX_ADDRESS:
        p.error(f"--expansion-address must be 1-{EXPANSION_MAX_ADDRESS}: the SDK reserves higher module addresses")
    return args


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    try:
        runner = Runner(args)
    except BenchError as e:
        print(f"error: {e}", file=sys.stderr)
        return 2
    return runner.run()


if __name__ == "__main__":
    sys.exit(main())
