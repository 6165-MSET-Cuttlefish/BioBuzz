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
import collections
import datetime
import hashlib
import json
import math
import os
import random
import re
import shlex
import shutil
import signal
import socket
import struct
import subprocess
import sys
import threading
import time
import traceback
import types
import urllib.error
import urllib.request
import xml.etree.ElementTree as ElementTree
import xml.sax.saxutils
from pathlib import Path

HUB_IP = "192.168.43.1"
ADB_PORT = 5555
DASHBOARD_PORT = 8000
RC_PAGE_PORT = 8080
# The RC page's script opens its websocket on its own port + 1 (RobotServer 12.0.0 assets/js/websocket-core.js).
RC_SOCKET_PORT = RC_PAGE_PORT + 1
LIMELIGHT_WEB_PORT = 5801
LOOPBACK = "127.0.0.1"
NETWORK_SERIAL = re.compile(r"^.+:\d+$")
CONTROL_HUB_PROP = "persist.ftcandroid.serialasusb"
PROP_TRUE = ("1", "true", "y", "yes", "on")
# /proc/net/tcp and tcp6 spell the hub's loopback this way; adbd connects a forward's tcp:<port> to its loopback.
HUB_LOOPBACK = ("0100007F", "00000000000000000000000001000000", "0000000000000000FFFF00000100007F")
RC_PAGE_TRIES = 5
RC_PACKAGE = "com.qualcomm.ftcrobotcontroller"
RC_LAUNCHER = f"{RC_PACKAGE}/org.firstinspires.ftc.robotcontroller.internal.PermissionValidatorWrapper"
IDLE_OPMODE = "$Stop$Robot$"
HUB_BENCH_DIR = "/sdcard/FIRST/bench"
PRIVATE_BENCH_DIR = "files/bench"
REPO_ROOT = Path(__file__).resolve().parents[2]

KEY_PHASE = "Bench phase"
KEY_STATUS = "Bench status"
KEY_DONE = "BENCH DONE"
KEY_LOOP = "Bench loop"

BENCH_OPMODES = [
    "Bench: Probe", "Bench: Hub", "Bench: Framework Loop", "Bench: Raw Loop", "Bench: Failure Path",
    "Bench: Webcam", "Bench: Webcam Stop", "Bench: Limelight", "Bench: Limelight Fault", "Bench: Pinpoint Absent",
    "Bench: Sloth Probe", "Bench: Gamepad", "Bench: Gamepad Linear", "Bench: Write Recovery", "Bench: Drive Handover",
    "Bench: Power Readback", "Bench: Overlay NaN", "Bench: Planner Stop", "Bench: Webcam Thread Failure",
    "Bench: Pinpoint", "Bench: Clock", "Bench: OctoQuad", "Bench: OctoQuad Absent", "Bench: Expansion Hub",
    "Bench: Photon", "Bench: Photon Stock", "Bench: Photon Faults", "Bench: Photon Routing", "Bench: Photon Stop",
    "Bench: Photon Stop Linear", "Bench: Photon Stop Race", "Bench: Photon Crash", "Bench: Photon Replug",
    "Bench: Framework Loop Photon", "Bench: Write Recovery Photon",
    "Bench: Limelight Hung", "Bench: Thread Lifecycle", "Bench: Turret PWM", "Bench: Edge Through Start",
    "Bench: Freeze Dump", "Bench: Turret Zero",
]

BENCHES = ("baseline", "hub", "loop", "failure", "threads", "gamepad", "edge", "write_recovery", "handover", "photon",
           "hardware_view", "turret", "webcam", "webcam_failure", "limelight", "llfault", "sloth", "clock", "pinpoint",
           "octoquad", "octoquad_absent", "expansion", "absent", "planner_stop", "overlay_nan", "config_by_name",
           "turret_zero", "freeze_dump", "dashboard_off")
FLAG_ONLY_BENCHES = ("photon", "turret_zero", "dashboard_off")

ADB_SERVER_PORT = 5037
ADB_SERVER_ELSEWHERE = ("ADB_SERVER_SOCKET", "ANDROID_ADB_SERVER_ADDRESS")
CELL_TIP_WEBCAM = "aprilTagDetector"
LIMELIGHT_DEVICE = "limelight"
CONFIG_LISTING = re.compile(r"^\S+\s+\d+\s+\S+\s+\S+\s+(\d+)\s+(\d{4}-\d\d-\d\d \d\d:\d\d)\s+(\S+)\.xml$")

CONFIG_PROBE = "bench_probe"
CONFIG_MAIN = "bench_main"
CONFIG_PINPOINT = "bench_pinpoint"
CONFIG_NO_WEBCAM = "bench_nowebcam"
CONFIG_LLFAULT = "bench_llfault"
CONFIG_OCTOQUAD = "bench_octoquad"
CONFIG_OCTOQUAD_ABSENT = "bench_octoquad_absent"
CONFIG_EXPANSION = "bench_expansion"
CONFIG_EXPANSION_ODO = "bench_expansion_odo"
CONFIG_PHOTON = "bench_photon"
CONFIG_TURRET = "bench_turret"
CONFIG_TURRET_ZERO = "bench_turret_zero"
CONFIG_FREEZE_SCRATCH = "bench_freeze_scratch"
CONFIG_UNSHIPPED = "bench_unshipped"
BENCH_CONFIGS = (CONFIG_MAIN, CONFIG_PINPOINT, CONFIG_NO_WEBCAM, CONFIG_LLFAULT, CONFIG_OCTOQUAD, CONFIG_OCTOQUAD_ABSENT,
                 CONFIG_EXPANSION, CONFIG_EXPANSION_ODO, CONFIG_TURRET, CONFIG_FREEZE_SCRATCH, CONFIG_UNSHIPPED,
                 CONFIG_PROBE, CONFIG_PHOTON, CONFIG_TURRET_ZERO)
# A REV hub's FTDI USB bridge; the Control Hub's own module is on a serial port, not USB.
REV_HUB_USB_ID = ("0403", "6015")
PHOTON_EXPANSION_DEVICES = [
    '<goBILDA5202SeriesMotor name="efl" port="0" />',
    '<goBILDA5202SeriesMotor name="efr" port="1" />',
    '<goBILDA5202SeriesMotor name="ebl" port="2" />',
    '<goBILDA5202SeriesMotor name="ebr" port="3" />',
    '<Servo name="eServo" port="0" />',
    '<Servo name="eServo2" port="1" />',
]
PHOTON_POWERS = ("0.2 and 0.25 alternating on motor ports 0-3 and 0.4/0.6 on servo ports 0-1 of the Control Hub and, "
                 "wired, the Expansion Hub, in every Photon OpMode; 0.15-0.3 in the three-writes and reset phases; 0.1 "
                 "for 0.2 s per motor port in the port probe; and in the STOP negative controls 0.2 on motor port 0 of "
                 "each hub after STOP (once, or for 150 ms), past the SDK's STOP gate on purpose")
PHOTON_STOP_TRIALS = 40
PHOTON_SOAK_STOP_TRIALS = 200
PHOTON_NEGATIVE_CONTROL_TRIALS = 5
PHOTON_SAFE_STOP_TRIALS = 200
PHOTON_RACE_TRIALS = 3
PHOTON_CRASH_LOST_WRITES = 2
PHOTON_DEPTH_REPS = 13
PHOTON_SOAK_DEPTH_REPS = 25
PHOTON_SUPERSEDED_REPS = 10
PHOTON_STOP_DELAY_S = (1.0, 3.0)
PHOTON_MIN_RESTARTS = 8
PHOTON_MAX_RESTARTS = 16
PHOTON_SOAK_MAX_S = 4 * 3600
PHOTON_SOAK_WRITES_PER_HUB = 10_000_000
PHOTON_SLOTH_SETTLE_S = 5.0
PHOTON_SOURCE_DIR = "TeamCode/src/main/java/org/firstinspires/ftc/teamcode/architecture/hardware/photon"
PHOTON_WORKLOAD_MODE = {0: "off", 8: "on8"}
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
CONFIG_RENUMBERED = "which this build numbers"
CONFIG_FELL_BACK = "Falling back on noConfig"
CONFIG_BY_NAME_HELD = ("the robot controller loads a shipped (res/xml) config by the resource id saved with it, so after an "
                       "install that adds, renames or removes a res/xml file the hub loads another config's XML under "
                       "the saved name, and for a name the build no longer ships it loads whatever XML has that id "
                       "instead of no config. The fix (re-resolving shipped configs by name at robot setup) is held on "
                       "another branch and not in this build, so this FAILs until it lands")
RESTORE_WINDOW_S = 600
RESTORE_RETRY_S = 30
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
                "from the dashboard's Hardware view for 4 s in the hardware-view step; BettaTurret's center and "
                "forward positions on servo ports 0-1 for about 4 s in Turret PWM; with --photon, " + PHOTON_POWERS)

NO_DS = "no Driver Station connected: no TelemetryMessage serialization or send"

LIMELIGHT_PORT = 5807
FAKE_LIMELIGHT_PORT = 15807
USB_HUB_PORTS = (DASHBOARD_PORT, RC_PAGE_PORT, RC_SOCKET_PORT, LIMELIGHT_WEB_PORT, LIMELIGHT_PORT)
TCP_LISTEN = "0A"
FAKE_LIMELIGHT_PHASES = ("limelightPostLoopback", "limelightPollOff", "limelightPoll100Hz", "limelightPoll200Hz",
                         "limelightPoll250Hz", "limelightFailureModes")

BALL_PIPELINE = 4
LIMELIGHT_PROXY_PENDING = ("regression: slothboard's Limelight proxy drops the query string again (fixed in 6165.6), "
                           "so /pipeline-switch?index=N reaches the Limelight as a bare /pipeline-switch")

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
FAILURE_EVENT = "failure path event: "
FAILURE_STUB_B_STOP = "BENCH stub B stop() failure"
ERROR_LINE_PENDING = ("regression: a crash no longer reaches the dashboard's Error line (SDK 12 only logs user-code "
                      "exceptions; EnhancedOpMode and slothboard 6165.6 put them there)")

WRITE_RECOVERY_RAW_LOOPS = 2
WRITE_RECOVERY_WRAPPED_LOOPS = 2
WRITE_CACHE_PENDING = ("regression: EnhancedMotor or EnhancedServo no longer re-sends an unchanged value every WriteCache "
                       "re-send period, so after a hub reset or a dropped write the motor stays off or the servo limp "
                       "longer than that period (or for good); nothing clears WriteCache on a failSafe, so the re-send is "
                       "the only way back")

HANDOVER_MIN_SAMPLES = 10
HANDOVER_CASES = (("C", "path cancelled from gameLoop"), ("S", "a command sequence turns writes back on"),
                  ("E", "collection-style command cancelled from gameLoop"))
HANDOVER_PENDING = ("regression: Drivetrain no longer resyncs Pedro's motor cache and its own when it takes the "
                    "motors back, so the motors sit at 0 while the stick is held")

HARDWARE_OPMODE = "Hardware"
HARDWARE_POWER_PATH = ("__hardware__", "Motors", "fl", "Power")
HARDWARE_VIEW_POWER = 0.2
HARDWARE_VIEW_MOVED = 0.01
HARDWARE_VIEW_PENDING = ("regression: a Power edit left in the dashboard's Hardware tree drives fl under the next "
                         "OpMode again (fixed in slothboard 6165.6)")


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


BASELINE_CATEGORY = "Optimization Toggles"
BASELINE_PENDING = ("regression: slothboard's config baseline is empty again (fixed in 6165.6), so the Config view's "
                    "'show only modified' filter and baseline hints show nothing")

SLOTH_PROBE_OPMODE = "Bench: Sloth Probe"
DASH_TOGGLE_OPMODE = "Enable/Disable Dashboard"
DASHBOARD_AUTO_ENABLE_KEY = "autoEnable"
DASHBOARD_RECOVERY_SHELL = (f"am force-stop {RC_PACKAGE} && run-as {RC_PACKAGE} rm -f {DASHBOARD_PREFS}; "
                            f"am start -n {RC_LAUNCHER}")

DASH_OFF_OPMODE = "Bench: Dashboard Off"
DASH_OFF_CONFIRM = "turn the dashboard off"
DASH_OFF_PROXY_PORTS = (5800, 5801, 5805, 5807)
DASH_OFF_RECORDED_PORTS = (DASHBOARD_PORT, 8080, 10158, 12649)
DASH_OFF_RECOVERY_PORTS = (5801, 5807)
DASH_OFF_STREAMED = ("telemetry packets", "images", "logcat")
DASH_OFF_PHASES = {"battery": "a battery pull", "restart": "an app restart", "reboot": "an adb reboot",
                   "reinstall": "a reinstall of the installed APK (adb install -r -t)"}
DASH_OFF_ON_S = 3.0
DASH_OFF_GRACE_S = 1.0
DASH_OFF_WINDOW_S = 10.0
DASH_OFF_PERIOD_MS = 50
DASH_OFF_SWITCH_WAIT_S = 30.0
DASH_OFF_DONE_WAIT_S = 60.0
DASH_OFF_BOOT_WAIT_S = 300.0
DASH_OFF_POLL_S = 2.0
DASH_OFF_RECOVERY_S = 60.0
DASH_OFF_PROBE_S = 5.0
DASH_OFF_INSTALL_TIMEOUT_S = 600
DASH_OFF_SETUP_MEANS = ("nothing reached the runner's socket while the dashboard was still on, so the zero counts after "
                        "the switch prove nothing")
DASH_OFF_DISABLE_MEANS = ("slothboard's private disable() didn't leave the dashboard off, so the later checks test nothing; "
                          "check the pinned dashboard with scripts/bench/offline/check_dashboard_api.py")
DASH_OFF_STREAM_MEANS = ("with the dashboard switched off, slothboard still sent telemetry, camera frames or logcat lines "
                         "to a connected client: the switch doesn't stop the streaming R704(D) bans")
DASH_OFF_PORTS_MEANS = ("with the dashboard switched off, the Limelight proxy still listens on the hub, so the Limelight's "
                        "video, web UI and API stay reachable over Wi-Fi")
DASH_OFF_INIT_MEANS = "the dashboard INITed an OpMode while switched off: the server still takes commands"
DASH_OFF_PERSIST_MEANS = ("the off setting didn't survive this, so a hub switched off before an event can come back on "
                          "by itself")
DASH_OFF_RECOVERY_MEANS = ("the adb recovery didn't bring the dashboard back: a hub switched off has no way back without "
                           "a Driver Station")
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
SLOTH_LIST_PENDING = ("regression: a hot load's OpMode list holds only slothboard's own OpModes, or a list offers "
                      "'Enable/Disable Dashboard', again (fixed in slothboard 6165.6). With no run judged (app "
                      "restarts, no load in logcat) this FAILs without saying anything about slothboard")

FLOODGATE_THRESHOLD_A = 25.0
FLOODGATE_LIMIT_V = FLOODGATE_THRESHOLD_A / 80.0 * 3.3
KEY_FLOODGATE_AMPS = "Drivetrain Floodgate Current (A)"
KEY_LIMITER = "Drivetrain Current Limiter Multiplier"
FLOODGATE_PENDING = ("the floodgate pin with nothing wired to it (Control Hub analog 0, as on the Betta bot, whose "
                     f"Floodgate isn't wired) reads {FLOODGATE_LIMIT_V:.2f} V or more, which Drivetrain's current limiter, "
                     f"on by default, takes for {FLOODGATE_THRESHOLD_A:.0f} A or more, so it scales every teleop drive power "
                     "down; until a Floodgate is wired, turn Drivetrain.currentLimiterConfig.enabled off for the Betta "
                     "bot. No fix is in flight: this run decides whether one is needed")

CAPTURE_CLOCK_PENDING = ("EasyOpenCV's captureTimeNanos isn't on System.nanoTime()'s clock, and CellTipCamera's freshness "
                         "gate (0 <= staleness <= maxStalenessMs) reads a stale verdict as NO VERDICT, never TIPPED or "
                         "SCORABLE: a negative age means the capture clock runs ahead, which makes every verdict stale once "
                         "its lead outgrows the delay to the OpMode's read; an age past maxStalenessMs (a drifting or offset "
                         "clock, another epoch, or a stalled camera thread) makes every verdict stale, since a verdict is "
                         "only older by the time the OpMode reads it. No fix is in flight")

WEBCAM_FAILURE_FRAME = 30
WEBCAM_FAILURE_PENDING = ("regression: a camera-thread pipeline exception no longer reaches the dashboard "
                          "(WebcamSession rethrows it from update() on the OpMode thread; slothboard 6165.6 puts it "
                          "in errorMessage)")

KEY_PINPOINT_STATUS = "Bench pinpoint status"
PINPOINT_INIT_HOLD_S = 2.0
PINPOINT_SEED_OFFSET_IN = 0.1
PINPOINT_SEED_TURN_DEG = 0.2
PINPOINT_SEED_PENDING = ("a start pose set in initialize() didn't survive the Pinpoint's IMU recalibration at INIT: every "
                         "auto sets the pose only in INIT, so they'd all start their first path from a pose the robot isn't "
                         "at. A PASS means only that the INIT sequence the autos use keeps the pose; a pose set at other "
                         "delays after the recalibration isn't tried. No fix is in flight")
PINPOINT_CRASHED = ("Bench: Pinpoint crashed (see the error): with a Pinpoint status check in BettaConstants or the "
                    "framework, a status other than READY at INIT (CALIBRATING right after create, FAULT_NO_PODS_DETECTED "
                    "without both pods) stops every OpMode, which the check must allow for")
PINPOINT_DROPOUT_PENDING = ("expected until the Pinpoint status fix: on a failed I2C read the SDK driver keeps the last pose "
                            "and velocity (firmware v1/v2, FAULT_BAD_READ) or zeroes them (v3+, NOT_READY), and only its "
                            "status says so, which nothing in Pedro or the framework reads, so the follower drives on a "
                            "frozen or (0, 0, 0) pose with no fault and no error")
PINPOINT_ABSENT_POSE_IN = 0.5
PINPOINT_ABSENT_TURN_DEG = 1.0
PINPOINT_ABSENT_MEANS = ("with nothing on the Pinpoint's I2C port, the follower built from BettaConstants neither raised a "
                         "fault nor kept the pose setPose set (the detail has the poses it read; on 2026-10-24 every one "
                         "was (0, 0, 0)): an auto on a robot whose Pinpoint is unplugged or dead follows every path from "
                         "the field origin with no warning. Nothing on this branch checks for a missing Pinpoint")

KEY_OCTOQUAD_STATUS = "Bench octoquad status"
KEY_OCTOQUAD_VALID = "Bench octoquad valid"
OCTOQUAD_BUS = 2
OCTOQUAD_ABSENT_BUS = 3
OCTOQUAD_ABSENT_WAIT_S = 10
OCTOQUAD_ABSENT_EXPECT_S = 5
OCTOQUAD_ABSENT_HOLD_S = 6
OCTOQUAD_ABSENT_CREATE_MS = 1000
OCTOQUAD_FAULT_KEY = "OctoQuad"
OCTOQUAD_GUARD_PREFIX = 'octoquad "'
OCTOQUAD_SEED_PENDING = ("a start pose set in initialize() moved by START + 3 s on GuardedOctoQuadLocalizer with the "
                         "OctoQuad at rest: it sends whole millimetres (under 0.04 in lost), and a pose set before RUNNING "
                         "is sent when RUNNING first arrives, so more than that means the OctoQuad didn't keep the pose "
                         "setLocalizerPose sent or its IMU heading drifted at rest, "
                         "and every auto sets the pose only in INIT, so they'd all start their first path from a pose "
                         "the robot isn't at. A PASS means only that this INIT sequence keeps the pose")
OCTOQUAD_VALID_PENDING = ("a localizer read failed its CRC or reported a status other than RUNNING after the first RUNNING "
                          "read, with the OctoQuad plugged in and at rest: GuardedOctoQuadLocalizer keeps the last pose "
                          "through up to 250 ms of such reads and then stops the OpMode, so on the robot those loops drive "
                          "on a stale pose; statusChanges says when and which status")
OCTOQUAD_CRASHED = ("Bench: OctoQuad crashed (see the error): at INIT, GuardedOctoQuadLocalizer's setup or the bench's "
                    "own setup threw while the follower was built; at START, the OctoQuad wasn't RUNNING within 3 s of "
                    "its reset")
OCTOQUAD_HUNG = ("Bench: OctoQuad never finished INIT: GuardedOctoQuadLocalizer doesn't wait for RUNNING in INIT, so "
                 "something else blocked it; the runner's STOP got it out (see the step's log)")
OCTOQUAD_DROPOUT_PENDING = ("with its cable out, the OpMode neither stopped nor raised a fault: GuardedOctoQuadLocalizer "
                            "stops the OpMode after 250 ms without a valid read once it has been RUNNING, so without that "
                            "the follower drives on a frozen pose with no error")
OCTOQUAD_ABSENT_HUNG = ("building the follower with nothing on the OctoQuad's bus took longer than it should: "
                        "GuardedOctoQuadLocalizer sends its setup and returns without waiting for RUNNING, so a slow build "
                        "means the I2C writes to the missing device block, or the guard is not the localizer in use")
OCTOQUAD_ABSENT_NO_FAULT = ("with nothing answering on the OctoQuad's I2C port, INIT didn't show the OctoQuad fault in time: "
                            "GuardedOctoQuadLocalizer raises it 3 s after its reset (or at once on FAULT_NO_IMU), and "
                            "EnhancedOpMode shows it first on every DS and dashboard frame, so a driver would not see why "
                            "the robot won't localize")
OCTOQUAD_ABSENT_SLOW = ("with the OctoQuad fault up, INIT loops were slow or missing: before RUNNING, "
                        "GuardedOctoQuadLocalizer polls only the status byte (one I2C read, no CRC-error logging), so a "
                        "slow loop here means the guard does more per loop than that while the OctoQuad is missing")
OCTOQUAD_ABSENT_STARTED = ("START with the OctoQuad not running didn't stop the OpMode with an error naming it: "
                           "EnhancedOpMode's start waits for GuardedOctoQuadLocalizer to report RUNNING and throws its "
                           "message when it can't, so without that an auto would follow paths on a pose that never moves")
OCTOQUAD_QUADRATURE = "ALL_QUADRATURE"
OCTOQUAD_MIXED_BANKS = "BANK1_QUADRATURE_BANK2_PULSE_WIDTH"
OCTOQUAD_BANK_READS = 300
OCTOQUAD_INIT_READS = 50
OCTOQUAD_BANK_SAME_MS = 0.2
OCTOQUAD_BANK_CONTROL_MS = 2.5
OCTOQUAD_INIT_READ_P95_MS = 20.0
OCTOQUAD_BANK_SETUP_MEANS = ("setup: timing cannot see a cost: readLocalizerDataAndAllEncoderData (35 bytes more; 6.58 "
                             "against 3.20 ms at p50 on 2026-10-09) wasn't clearly slower than readLocalizerData here, "
                             "so an equal cost for the bank setting would prove nothing. Not a robot problem by itself")
OCTOQUAD_BANK_SET_MEANS = ("the OctoQuad wasn't in ALL_QUADRATURE at the start, or didn't read back "
                           "BANK1_QUADRATURE_BANK2_PULSE_WIDTH after the set, so the timings compare nothing. Another bank "
                           "left by a crashed run goes back to ALL_QUADRATURE when the OctoQuad is power-cycled, unless "
                           "someone saved it to flash")
OCTOQUAD_BANK_COST_MEANS = ("with channels 4-7 in pulse-width mode the loop's localizer read costs more than with every "
                            "channel counting, so the turret zero's bank setting is not free per loop (the plan in "
                            "docs/wiring-findings.md, Zeroing the nectar turret, assumes it is)")
OCTOQUAD_INIT_READ_MEANS = ("the one-off INIT read the turret zero would use (readAllEncoderData) failed its CRC or took "
                            f"{OCTOQUAD_INIT_READ_P95_MS:.0f} ms or more at p95: the zero needs retries or costs INIT time")
OCTOQUAD_BANK_RESTORE_MEANS = ("the OctoQuad doesn't read back the bank it had before the phase, so channels 4-7 may "
                               "still be pulse-width inputs and the pod on channel 4 isn't counted until the OctoQuad is "
                               "power-cycled: check that before the next odometry run")
OCTOQUAD_BANK_CRASH_MEANS = ("the bank phase threw (see the error), or a STOP ended it early, after the seed phase had "
                             "finished, so the seed checks above stand; the bank phase restores the bank it found in a "
                             "finally")

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
CLOCK_PENDING = ("regression: PathCommands.timeout ended early or late while the hub's wall clock jumped (a Driver "
                 "Station connecting, or the RC's web page setting a clock left near 1970), so a wait in it times on "
                 "System.currentTimeMillis() again, as Ivy's Commands.waitMs does")

KEY_PLANS = "Bench plans"
PLANNER_STOP_BALLS = (4, 5)
PLANNER_STOP_DELAYS_S = (0.05, 0.3, 0.6)
PLANNER_STOP_WARM = 3
PLANNER_STOP_WARM_WAIT_S = 180
PLANNER_STOP_SEEN = re.compile(r"planner_stop (\S+) STOP seen (?:between plans|([\d.]+) ms into plan (\d+))")
SDK_STOP_WINDOW_MS = 900
PLANNER_JUDGED_BALLS = 4
PLANNER_SLICE_MAX_MS = 300
PLANNER_AUTO_STOPPING = "auto-stopping OpMode 'Bench: Planner Stop'"
PLANNER_STOPPED_LINE = "STOP - OPMODE"
PLANNER_RUNNING_LINE = re.compile(r"Robot Status: running\s*$")
PLANNER_STOP_PENDING = ("regression: a STOP during route planning restarted the Robot Controller app, losing dashboard "
                        "@Config edits and the alliance: the planner slice running at STOP plus stop() outlasted the "
                        f"SDK's {SDK_STOP_WINDOW_MS} ms stop window, so RouteOptimizer.Search.step() ran far past its "
                        "slice or a whole plan ran inside one loop again")
PLANNER_STOP_IDLE_PENDING = (f"the hub took {SDK_STOP_WINDOW_MS} ms or more from STOP ('auto-stopping') to the OpMode "
                             "stopped ('***STOP - OPMODE***'): the planner slice running at STOP plus the loop's tail "
                             "and stop() no longer fit the SDK's stop window, and past it the SDK restarts the app")
PLANNER_SLICE_PENDING = (f"a RouteOptimizer.Search.step() ran {PLANNER_SLICE_MAX_MS} ms or more: one shape() past the "
                         "slice got slower, or VisibilityGraphPlanner.routeSearchSliceMs was raised. A STOP waits for "
                         f"that step, which then leaves stop() less of the SDK's {SDK_STOP_WINDOW_MS} ms than Ball "
                         "Collection Auto's webcam close can take (up to 581 ms)")

LLHUNG_OPMODE = "Bench: Limelight Hung"
KEY_LIMELIGHT_STATE = "Bench limelight"
LLHUNG_FAULT_WAIT_S = 25.0
LLHUNG_FAULT_SLACK_MS = 2000
LLHUNG_STOP_PASS_MS = 100
LLHUNG_CAMERA_STOP_MS = 50
# Without a Driver Station the SDK's event loop sleeps 500 ms per pass, so a clean STOP alone can take most of a second.
LLHUNG_STOP_IDLE_S = 2.0
LLHUNG_OVERLAP_MS = 100
LLHUNG_SYNC_AFTER_RELEASE_MS = 5000
LLHUNG_SETUP_MEANS = ("no POST was hung at that moment (the runner took longer than the POST timeout between the runs, "
                      "or the POST never reached the fake), so the checks that rely on it prove nothing; not a robot "
                      "problem by itself")
LLHUNG_FAULT_MEANS = ("with a Limelight that serves results but never answers a command POST (e.g. busy loading a "
                      "pipeline), the Limelight fault comes later than one POST timeout after INIT, never, or with "
                      "another message, so drivers can't tell why ball vision is off")
LLHUNG_LOOP_MEANS = "a hung Limelight POST stalls the OpMode thread: INIT or the match loop waits on the Limelight"
LLHUNG_RETRY_MEANS = ("a POST to a Limelight that never answers was sent again (a silent HttpURLConnection retry or a "
                      "second sender) or never timed out, so a hung Limelight holds the sync and its fault back longer "
                      "than one POST timeout")
LLHUNG_LEFTOVER_MEANS = "a LimelightSync thread outlives a finished sync"
LLHUNG_STOP_MEANS = (f"STOP waits on a hung Limelight POST: past the SDK's {SDK_STOP_WINDOW_MS} ms stop window the SDK "
                     "force-stops the OpMode or restarts the Robot Controller app, losing dashboard edits and the "
                     "alliance")
LLHUNG_REINIT_WAIT_MEANS = ("INIT blocks behind a Limelight POST the previous OpMode left hung: a re-INIT after STOPping "
                            "with a Limelight that isn't answering commands freezes for up to the POST timeout")
LLHUNG_SINGLE_MEANS = ("two senders had POSTs open on the Limelight at once, so an old OpMode's request can land after "
                       "the new one's and leave the Limelight on the wrong pipeline or script")
LLHUNG_FRESH_MEANS = ("a second LimelightSync worker sent a POST while the first one's POST was still hung, or "
                      "LimelightSync's sender isn't the blackboard semaphore: after a deploySloth, the reloaded class "
                      "could POST over a hung one and leave the Limelight on the wrong pipeline or script")
LLHUNG_RECOVER_MEANS = ("after a STOP during a hung POST, the next INIT's sync fails or never finishes although the "
                        "Limelight answers again: re-INIT doesn't recover ball vision")

THREAD_OPMODE = "Bench: Thread Lifecycle"
THREAD_NAME = "TransferCounter"
THREAD_CRASH_IN_LOOP = "BENCH thread lifecycle: deliberate crash in loop()"
THREAD_CRASH_IN_STOP = "BENCH thread lifecycle: deliberate crash in stop() before TransferCounter.stop()"
THREAD_DIED = "TransferCounter thread died"
THREAD_INVALID_PORT = 99
THREAD_BAD_PORT = f"invalid analog input: {THREAD_INVALID_PORT}"
THREAD_TIMEOUTS = {"normal": 30, "stop_before_start": 15, "crash_in_loop": 20, "crash_in_stop": 20, "thread_failure": 20,
                   "interrupt_probe": 60}
THREAD_AFTER_CRASH_WAIT_S = 7.0
THREAD_STOP_MS = 50
THREAD_END_GAP_MS = 50
THREAD_DEATH_MS = 50
THREAD_POLL_HZ = 90
THREAD_INTERRUPTS = 100
THREAD_REPEATS = 10
THREAD_REPEAT_SECONDS = 1
THREAD_SLOTH_SECONDS = 600
THREAD_SLOTH_DEPLOY_TIMEOUT_S = 240
LOCK_ABANDONED = "#### abandoning lock"
THREAD_LISTINGS = (
    ("run-as cat /proc/<pid>/task/*/comm", "comm", "run-as {package} sh -c 'cat /proc/{pid}/task/*/comm'"),
    ("cat /proc/<pid>/task/*/comm", "comm", "cat /proc/{pid}/task/*/comm"),
    ("ps -T -p <pid>", "ps", "ps -T -p {pid}"),
    ("ps -t <pid>", "ps", "ps -t {pid}"),
)
THREAD_LISTING_MIN = 10
THREAD_LEFTOVER_MEANS = {
    "normal": "TransferCounter's poller outlives a normal STOP: every run of Transfer Count Test adds a thread reading "
              "the hub every 10 ms under every later OpMode, until the app restarts",
    "stop_before_start": "a STOP from INIT leaves a TransferCounter poller running although start() never ran",
    "crash_in_loop": "after a crash in loop() the poller's isAlive tie didn't end it: it keeps reading the hub under "
                     "every later OpMode, until the app restarts",
    "crash_in_stop": "a stop() that throws before TransferCounter.stop() leaves the poller reading the hub under every "
                     "later OpMode, until the app restarts",
    "thread_failure": "a poller thread that failed is still alive at the next INIT",
}
THREAD_ADB_MEANS = ("the adb thread listing still shows a TransferCounter thread in the app after the OpMode ended: the "
                    "poller keeps reading the hub under the next OpMode")
THREAD_SETUP_MEANS = "the deliberate failure didn't happen as planned, so this mode's thread checks prove nothing"
THREAD_RESTART_MEANS = ("the Robot Controller app restarted under this failure with TransferCounter's poller running, "
                        "losing dashboard edits and the alliance")
THREAD_SLOTH_RUNNING_MEANS = ("deploySloth while an OpMode runs stopped it or restarted the app, so a deploy during a "
                              "test run cuts it off without a clean stop()")
THREAD_SLOTH_HELD_MEANS = ("Sloth swapped the code under a running OpMode, or never landed the load after its STOP: a "
                           "deploy during a run leaves the hub on old code or mixes old and new classes")

TURRET_OPMODE = "Bench: Turret PWM"
TURRET_MEANS = {
    "CENTER: PWM on at the center pulse on both ports":
        "TurretState.CENTER doesn't power both turret servos at the center pulse, so the turret doesn't hold center",
    "stop() then writes off: PWM off right after stop() and for the whole step":
        "a mid-OpMode Module.stop() on BettaTurret with writes off leaves a turret servo powered, or something "
        "re-powers it while writes are off",
    "writes back on after a step off: PWM on again at the same position by the next loop":
        "turning writes back on after a stop() doesn't re-power the turret at its position within a loop, so it "
        "stays limp after the module is re-enabled",
    "stop() with writes on: PWM off right after stop(), on again the next loop":
        "a mid-OpMode Module.stop() on BettaTurret with writes on doesn't re-power it on the next loop: "
        "EnhancedServo's write cache kept the unchanged position after setPwmDisable(), so the turret stays limp "
        "until its target changes",
    "IDLE then stop(): PWM stays off": "IDLE re-powers a turret servo after stop(), so the turret holds when it should "
                                       "be limp",
    "RightState.FORWARD: TurretR on at the forward pulse, TurretL still off":
        "RightState.FORWARD doesn't drive TurretR alone to its forward pulse",
}
TURRET_OTHER_MEANS = "a Turret PWM step didn't behave as the bench expects; its detail lists the hub readings"

TZ_OPMODE = "Bench: Turret Zero"
TZ_FINISH = "turret_zero_finish.json"
TZ_POWERS = ("Bench: Turret Zero sets no motor power and drives no servo. Its mode-change phase changes br's (motor port "
             "2's) run mode, and the SDK re-sends the last power and enables the port with every mode change (SDK 12.0.0 "
             "LynxDcMotorController.setMotorMode), so keep every motor power port empty")
TZ_COUNTS_PER_TURN = 8192
TZ_PER_TURN_COUNTS = 8
TZ_QUAD_TURNS = 5
TZ_SPLIT_TURNS = 10
TZ_V2_TURNS = 1
TZ_VERSIONS = {"V1": (1.0, 1024.0, 1024), "V2": (3.884, 998.06, 4096)}
TZ_PULSE_SLACK_US = 10.0
TZ_LEVEL_MAX_V = 3.46
TZ_REZERO_RUNS = 10
TZ_INIT_READS = 50
TZ_INIT_P95_MS = 20.0
TZ_SPREAD_DEG = 0.5
TZ_MOVED_DEG = 1.0
TZ_AGREE_DEG = 1.0
TZ_CONTROL_DEG = 20.0
TZ_MIN_RESTS = 10
TZ_MIN_SPAN_DEG = 300.0
TZ_MIN_TURNS = 2.0
TZ_STILL_COUNTS = 2
TZ_NONZERO_COUNTS = 100
TZ_RECOVER_DEG = 1.0
TZ_SUPPLY_MIN_V = 2.97
TZ_MOVED_MIN_DEG = 5.0
TZ_WIRING_MEANS = ("the cables don't match what the bench needs, so it stopped before anything drove an OctoQuad input: "
                   "the OctoQuad's inputs take 3.3 V only, and two supplies on one encoder fight")
TZ_LEVEL_MEANS = (f"the encoder's absolute output goes above {TZ_LEVEL_MAX_V:.2f} V (3.3 V + 5%, rounded down), the "
                  "reading wasn't a positive voltage, or its supply isn't a 3.3 V supply, so it could drive the "
                  "OctoQuad's 3.3 V-only input too high; the bench stopped before the branch was plugged in")
TZ_HIGH_PROMPT = (">>> METER the absolute output (the OctoQuad branch's signal pin) to ground while turning the shaft "
                  "slowly through a whole turn. Type the highest reading in volts.")
TZ_V2_LEVEL_MEANS = (f"the V2's absolute output goes above {TZ_LEVEL_MAX_V:.2f} V, the reading wasn't a positive "
                     "voltage, or its supply isn't a 3.3 V supply, so it could drive the OctoQuad's 3.3 V-only input too "
                     "high; the runner kept the branch out and skipped the V2 runs that needed it")
TZ_TURNS_MEANS = (f"the counts over whole turns to a dial mark are off by more than {TZ_PER_TURN_COUNTS} per turn or "
                  "didn't come back to the mark: lost or extra counts on that input (or the shaft wasn't stopped on the "
                  "mark; the span and agreement checks don't depend on the hand)")
TZ_LOAD_MEANS = ("the hub's counts per turn change when the OctoQuad branch is plugged in, so the split loads the "
                 "encoder's outputs")
TZ_SPAN_MEANS = ("the absolute output's pulse range isn't the version's (DigitalChickenLabs' note: Through Bores vary by "
                 "up to 10 us), or the hub doesn't count 8192 per turn of the absolute output, or the shaft turned less "
                 "than 2 turns: the later angles use this span's range and sign")
TZ_INIT_MEANS = (f"the one-off INIT read of channel 4 failed its CRC, took {TZ_INIT_P95_MS:.0f} ms or more at p95, or "
                 f"moved more than {TZ_SPREAD_DEG} deg at rest: the zero read at INIT is slow or noisy")
TZ_REZERO_MEANS = (f"re-INITs with the shaft untouched gave zeros more than {TZ_SPREAD_DEG} deg apart, or a moved "
                   f"shaft's absolute change didn't match its count change within {TZ_MOVED_DEG} deg: the INIT zero "
                   "doesn't hold across OpModes")
TZ_AGREE_MEANS = (f"at a rest the absolute angle and the angle counted on the hub from the INIT zero differed by more "
                  f"than {TZ_AGREE_DEG} deg of encoder ({TZ_AGREE_DEG * 21 / 50:.2f} deg of turret): the absolute output "
                  "read once at INIT can't set the turret's zero to that accuracy")
TZ_COVERAGE_MEANS = ("not enough motion: too few rests, too little of the absolute range or under 2 turns, so a "
                     "stationary or barely turned shaft could have passed")
TZ_CONTROL_MEANS = (f"setup: a flipped sign or a 180 deg offset on the same rests didn't give an error over "
                    f"{TZ_CONTROL_DEG:.0f} deg, so the agreement check couldn't have failed")
TZ_SIGN_MEANS = ("the hub's own encoder read (getController().getMotorCurrentPosition with bulk caching off) and the "
                 "bulk read disagree, so the turret code can't take the count from the bulk read it already pays for; "
                 "or getCurrentPosition() isn't minus that count exactly where the motor type's rotation and the "
                 "direction say (SDK 12.0.0 DcMotorImpl), so the wiring notes' rule to read the count raw rests on a "
                 "wrong reading of the source; or the count was under 100, so the sign couldn't be told")
TZ_SIGN_FLIPS = {"CCW": {"FORWARD": -1, "REVERSE": 1}, "CW": {"FORWARD": 1, "REVERSE": -1}}
TZ_MODE_MEANS = ("a run-mode change or a new OpMode moved the hub's count with the shaft still: the turret zero would "
                 "move whenever br's mode changes or an OpMode starts")
TZ_RESET_MEANS = (f"after STOP_AND_RESET_ENCODER the absolute read didn't give the angle back within {TZ_RECOVER_DEG} "
                  "deg: the zero can't be recovered after a count reset")
TZ_BROWNOUT_MEANS = ("the Control Hub's count changed in the dip while the app kept running, and nothing reported it (no "
                     "warning, no error, no failed bulk read, no hub not responding): the turret's zero moves silently")
TZ_V2_MEANS = ("the V2 doesn't count 8192 per turn on A, counts on S, or its pulse range isn't 3.884-998.06 us: it can't "
               "replace the V1 without changing the turret zero")
TZ_NOT_RUN_MEANS = "the bench stopped before this phase or its OpMode run failed (see the error)"

EDGE_OPMODE = "Bench: Edge Through Start"
EDGE_INIT_IDLE, EDGE_INIT_HOLD, EDGE_RUN_HOLD, EDGE_RUN_RELEASE, EDGE_RUN_PRESS, EDGE_RUN_RELEASE_AGAIN = 10, 11, 12, 13, 14, 15
EDGE_INIT_HOLD_S = 1.0
EDGE_RUN_HOLD_S = 2.0
EDGE_SETUP_MEANS = ("the runner's LB hold didn't reach the OpMode unbroken from INIT through START (or LB was already "
                    "down when the suppliers were built), so the edge checks prove nothing; not a robot problem by itself")
EDGE_START_MEANS = ("BioBuzz Tele's invalidate() in its INIT loop doesn't spend a press held through START: a driver "
                    "holding gamepad1 LB (heading lock) or LT (slow mode) at START gets that toggle on the first loop")
EDGE_PRESS_MEANS = ("after the INIT invalidate, a normal press in the match gives zero or two edges, so BioBuzz Tele's "
                    "toggles miss or double a press")
EDGE_CONTROL_MEANS = ("an EdgeBooleanSupplier read only after START gives zero or two edges for one press: edge "
                      "detection itself is broken, not just the START case")

FREEZE_OPMODE = "Bench: Freeze Dump"
FREEZE_CONTROL_S = 5.0
FREEZE_RELAUNCH_WAIT_S = 120
FREEZE_TRIGGERS = ("set", "set", "set", "write", "delete")
FREEZE_MESSAGES = {"set": "SET_HARDWARE_CONFIG", "write": "WRITE_HARDWARE_CONFIG", "delete": "DELETE_HARDWARE_CONFIG"}
FREEZE_TRACES = "/data/anr/traces.txt"
FREEZE_TRACES_WROTE = "Wrote stack traces"
FREEZE_TRACES_HEADER = re.compile(r"^----- pid (\d+) at (\d{4})-(\d\d)-(\d\d) (\d\d:\d\d:\d\d) -----$", re.M)
FREEZE_TRACES_THREAD = re.compile(r'^"(.*)" (?:daemon )?prio=\d+ tid=(\d+) (\w+)')
FREEZE_TRACES_WAITS = re.compile(r"^\s+- waiting to lock <(0x[0-9a-f]+)> \(a ([^)]+)\)(?: held by thread (\d+))?")
FREEZE_DEADLINE = "deadline expired waiting for service termination: EventLoop"
FREEZE_AP_TAG = "FtcAccessPointService"
FREEZE_AP_RELAUNCH = "Relaunching the RC"
FREEZE_STOP_FILE = "freeze_watch.stop"
FREEZE_ALIVE_STALL = "no RC alive broadcast"
FREEZE_LOG_LINES = (WATCHDOG_KILLED, APP_KILLED, FREEZE_TRACES_WROTE, "Unable to open stack trace file", "freeze watch",
                    "Force stopping", "reacting to signal", ROBOT_SETUP_DONE, RESTART_FAILED, FREEZE_DEADLINE)
FREEZE_HYPOTHESIS = ("attemptRestart", "onOpModePostStop")
FREEZE_GUARD_MEANS = ("a dashboard Hardware Config switch while an OpMode is in INIT restarted the robot under it: "
                      "slothboard's stopped-only guard doesn't hold")
FREEZE_RESTART_MEANS = ("a dashboard hardware-config change at idle froze the Robot Controller app until the SDK's 10 s "
                        "deadline or the Control Hub's watchdog ended it, and the app was killed and relaunched (about "
                        "50 s with no robot, losing dashboard edits and the alliance), or the restart failed or wasn't "
                        "tried (the detail says which): slothboard before 6165.7 runs the robot restart while holding "
                        "its OpMode lock, which the event loop's teardown needs, so this FAILs on any build pinning "
                        "6165.6 or older")

OVERLAY_NAN_PENDING = ("regression: a NaN or infinite field-overlay coordinate kills slothboard's telemetry thread "
                       "again (fixed in 6165.6), ending dashboard telemetry for every OpMode until the app restarts")

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
    "iterative": ("regression: dashboard gamepads no longer reach an iterative OpMode (fixed in slothboard 6165.5): "
                  "the SDK copies its own state over gamepad1 before every loop, so a write to gamepad1 is lost"),
    "linear": "the runner's RECEIVE_GAMEPAD_STATE never reached the OpMode (websocket or slothboard trouble)",
}


GP_SIGN_PENDING = ("regression (fixed in slothboard 6165.6, not yet run on the hub): keyboard W and the on-screen "
                   "stick send up as +1, so -left_stick_y drives backward. No loop past 0.9 either way means the "
                   "input never arrived (keyboard input off, the page without focus, or the stick short of the edge)")
GP_INTERACTIVE = (
    (GP_KEYBOARD, "keyboard W",
     "In the Gamepad view turn keyboard input on and click the page, with no USB gamepad plugged in. Hold W for 3 s "
     "and let go", GP_SIGN_PENDING),
    (GP_SCREEN, "on-screen stick up",
     "Drag the Gamepad view's on-screen left stick to its top edge, hold it there for 3 s and let go", GP_SIGN_PENDING),
    (GP_USB, "USB pad forward with the Gamepad view open",
     "Plug a USB gamepad into the Mac and press Start+A to bind it as gamepad 1, with the Gamepad view still open. "
     "Hold its left stick fully forward for 3 s and let go",
     "regression (fixed in slothboard 6165.6, not yet run on the hub): the Gamepad view's 100 ms keepalive sends a "
     "rest state between a bound pad's frames, so a held stick reads 0. No loop past -0.9 means the pad wasn't "
     "bound with Start+A or the stick didn't reach full forward"),
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


class OtherHub(BenchError):
    """--hub-serialno: adb reached a hub whose ro.serialno isn't that one. Final: nothing more is sent to it."""


class ConnectionClosed(Exception):
    pass


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
                raise ConnectionResetError("websocket server closed the connection during the handshake")
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


KNOWN_SERVER_TYPES = {
    "RECEIVE_ROBOT_STATUS", "RECEIVE_OP_MODE_LIST", "RECEIVE_CONFIG", "RECEIVE_CONFIG_BASELINE", "RECEIVE_TELEMETRY",
    "RECEIVE_IMAGE", "RECEIVE_HARDWARE_CONFIG_LIST", "RECEIVE_LOGCAT_ERRORS", "RECEIVE_LOGCAT_LINES",
    "RECEIVE_GAMEPAD_STATE",
}
DASH_SHORT_CONNECTION_S = 5.0
DASH_RECONNECT_FIRST_S = 0.5
DASH_RECONNECT_MAX_S = 4.0


class Dashboard:
    """slothboard's websocket (port 8000): OpMode control, hardware configs, and the telemetry the benches report on."""

    def __init__(self, link, port, log):
        self.link, self.port, self.log = link, port, log
        self.cond = threading.Condition()
        self.closing = False
        self.ws = None
        self.connected = False
        self.fatal = None
        # Off only while restoring the config: an unknown message type is then logged instead of stopping the restore.
        self.strict = True
        self.disconnects = 0
        self.short_drops = 0
        self.last_lifetime = None
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
        wait = DASH_RECONNECT_FIRST_S
        while time.monotonic() < deadline:
            try:
                ws = WebSocket(*self.link.address(self.port))
            except OSError as e:
                last_error = e
                time.sleep(max(0.0, min(wait, deadline - time.monotonic())))
                wait = min(DASH_RECONNECT_MAX_S, wait * 2)
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
        raise BenchError(self.link.unreachable(self.port, last_error))

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
                    self.last_lifetime = time.monotonic() - self.connected_at[connection]
                    self.short_drops = self.short_drops + 1 if self.last_lifetime < DASH_SHORT_CONNECTION_S else 0
                    if not self.closing:
                        self.log(f"dashboard websocket dropped after {self.last_lifetime:.1f} s: {e}")
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
                if self.recording is not None:
                    self.recording["statuses"].append((now, self.status["activeOpMode"],
                                                       self.status.get("errorMessage") or ""))
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
                                                          data.get(KEY_LOOP), size / max(1, len(packets))))
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
        self._reconnect()

    def _reconnect(self):
        with self.cond:
            old, self.ws, self.connected = self.ws, None, False
        if old is not None:
            old.close()
        self.connect(timeout=60)

    def ensure_connected(self, timeout=60):
        self.check()
        if not self.connected:
            delay = (min(DASH_RECONNECT_MAX_S, DASH_RECONNECT_FIRST_S * 2 ** (self.short_drops - 1))
                     if self.short_drops else 0.0)
            if delay:
                self.log(f"reconnecting to the dashboard in {delay:.1f} s ({self.short_drops} connection(s) in a row "
                         f"dropped within {DASH_SHORT_CONNECTION_S:.0f} s)")
                time.sleep(min(delay, timeout))
            else:
                self.log("reconnecting to the dashboard")
            self.connect(timeout)

    def send(self, obj):
        self.ensure_connected()
        try:
            self.ws.send_text(json.dumps(obj))
        except OSError as e:
            # The app can die under the socket (a config switch can get it killed) before the reader notices.
            self.log(f"dashboard send failed ({e}); reconnecting")
            self._reconnect()
            self.ws.send_text(json.dumps(obj))

    def fresh_active_config(self, timeout=10):
        """The active config from the list slothboard sends a socket as it opens, which reads it from the app at that
        moment; None if no list comes (slothboard skips it until it has read the configs after a restart)."""
        deadline = time.monotonic() + timeout
        try:
            ws = WebSocket(*self.link.address(self.port))
        except OSError:
            return None
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
            self.recording = {"start": time.monotonic(), "packets": [], "images": [], "statuses": [], "messages": 0,
                              "bytes": 0}

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


class MessageCounter:
    """A second dashboard websocket that only listens: when slothboard sent it each message, and of what type."""

    def __init__(self, host, port):
        self.ws = WebSocket(host, port)
        self.lock = threading.Lock()
        self.received = []
        self.closed_at = None
        self.done = threading.Event()
        threading.Thread(target=self._read, name="bench-counter", daemon=True).start()
        threading.Thread(target=self._keepalive, name="bench-counter-ping", daemon=True).start()

    def _read(self):
        try:
            while True:
                opcode, payload = self.ws.recv_message()
                now = time.monotonic()
                try:
                    msg = json.loads(payload.decode("utf-8")) if opcode == WebSocket.OP_TEXT else {"type": "binary"}
                except ValueError:
                    msg = {"type": "not JSON"}
                kind = msg.get("type") if isinstance(msg, dict) else "not an object"
                packets = len(msg.get("telemetry") or []) if kind == "RECEIVE_TELEMETRY" else 0
                with self.lock:
                    self.received.append((now, kind, packets))
        except (ConnectionClosed, OSError, ProtocolError):
            with self.lock:
                self.closed_at = time.monotonic()

    def _keepalive(self):
        # NanoHTTPD drops a socket that stays silent for 5 s, and a dashboard switched off sends nothing.
        while not self.done.wait(1.5):
            try:
                self.ws.send_ping()
            except OSError:
                return

    def events(self):
        with self.lock:
            return list(self.received)

    def open_at(self, t):
        with self.lock:
            return self.closed_at is None or self.closed_at >= t

    def close(self):
        self.done.set()
        self.ws.close()


class GamepadHold:
    """Keeps sending one gamepad state from its own thread, so a hold survives the runner's START and status waits:
    slothboard rests both pads after 500 ms without a message. A failed send is raised by stop()."""

    def __init__(self, runner, phase, every, **pad):
        self.runner, self.phase, self.every, self.pad = runner, phase, every, pad
        self.done = threading.Event()
        self.thread = threading.Thread(target=self._run, name="bench-gp-hold", daemon=True)
        self.sends = 0
        self.max_gap = 0.0
        self.failure = None

    def start(self):
        self.thread.start()

    def started(self):
        return self.thread.ident is not None

    def _run(self):
        last = None
        while not self.done.is_set():
            try:
                self.runner.gp_send(self.phase, **self.pad)
            except Exception as e:
                self.failure = e
                return
            now = time.monotonic()
            if last is not None:
                self.max_gap = max(self.max_gap, now - last)
            last = now
            self.sends += 1
            self.done.wait(self.every)

    def stop(self):
        self.done.set()
        if self.started():
            self.thread.join(5)
        if self.failure is not None:
            raise BenchError(f"the runner's gamepad hold stopped sending: {self.failure}")

    def summary(self):
        return {"sends": self.sends, "maxGapSeconds": round(self.max_gap, 3), "lastPhase": self.phase}


class FaultWatch:
    """What the dashboard websocket shows of one fault key: whether its FAULT row leads each packet the bench OpMode
    sent (the ones carrying its loop counter), whether the field overlay draws it red, and when it came and went."""

    def __init__(self, key, loop_key=KEY_LOOP):
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


def adb_server_port(env):
    """The port on this Mac where adb's client looks for its server: ANDROID_ADB_SERVER_PORT, else 5037."""
    for name in ADB_SERVER_ELSEWHERE:
        if env.get(name):
            raise BenchError(f"{name} is set to {env[name]!r}, which can point adb at an adb server other than this "
                             "Mac's own, so every adb command could reach a hub that isn't on the table and the bench "
                             f"would drive its motor ports. Unset {name} and rerun. Nothing was sent to any hub.")
    raw = env.get("ANDROID_ADB_SERVER_PORT")
    if raw is None:
        return ADB_SERVER_PORT
    if not raw.isdigit() or not 0 < int(raw) < 65535:
        raise BenchError(f"ANDROID_ADB_SERVER_PORT is {raw!r}, not a port number adb accepts")
    return int(raw)


def adb_port_owners(lsof_text):
    """[(command, pid)] from `lsof -F cp` output: a p<pid> line starts each process, its c<command> line follows."""
    owners, pid, command = [], None, None
    for line in lsof_text.splitlines() + ["p"]:
        if line.startswith("p"):
            if pid is not None:
                owners.append((command or "a process lsof didn't name", pid))
            pid, command = line[1:] or None, None
        elif line.startswith("c"):
            command = line[1:]
    return owners


def preflight_problem(owners, port, unseen_listener=False):
    """Why no adb command may run, or None: whatever listens on adb's server port gets every adb command."""
    others = [f"{command} (pid {pid})" for command, pid in owners if command != "adb"]
    if others:
        return (f"{', '.join(others)} {'listens' if len(others) == 1 else 'listen'} on port {port}, where adb looks for "
                "its server, so every adb command would go to whatever is behind it, possibly a hub that isn't this "
                "one, and the bench would install onto and drive that hub's motor ports. Stop "
                f"{'that process' if len(others) == 1 else 'those processes'} (an ssh port forward, for one), check "
                f"that `lsof -nP -iTCP:{port} -sTCP:LISTEN` shows nothing or adb, and rerun. Nothing was sent to any hub.")
    if unseen_listener:
        return (f"something listens on port {port}, where adb looks for its server, that lsof can't name (another "
                "user's or a system process), and every adb command would go to it. Find it with `sudo lsof -nP "
                f"-iTCP:{port} -sTCP:LISTEN`, stop it, and rerun. Nothing was sent to any hub.")
    return None


def port_answers(port):
    """Whether something accepts a TCP connection on this Mac's loopback port: lsof can't see other users' sockets."""
    for host in ("127.0.0.1", "::1"):
        try:
            socket.create_connection((host, port), timeout=1).close()
            return True
        except OSError:
            continue
    return False


def adb_port_listeners(port, run=None):
    lsof = shutil.which("lsof") or "/usr/sbin/lsof"
    try:
        r = (run or subprocess.run)([lsof, "-nP", f"-iTCP:{port}", "-sTCP:LISTEN", "-Fcp"], capture_output=True,
                                    text=True, timeout=30)
    except FileNotFoundError:
        raise BenchError(f"can't check what listens on adb's port {port}: there is no lsof on this Mac")
    if r.returncode != 0 and (r.returncode != 1 or r.stdout.strip()):
        raise BenchError(f"lsof couldn't list what listens on adb's port {port} (exit {r.returncode}): "
                         f"{(r.stderr or r.stdout).strip()[:200]}")
    return adb_port_owners(r.stdout)


def adb_port_preflight(env=None, run=None, answers=None):
    """Raises BenchError, before any adb command, when anything but adb listens on adb's server port."""
    port = adb_server_port(os.environ if env is None else env)
    owners = adb_port_listeners(port, run)
    unseen = not owners and (answers or port_answers)(port)
    if unseen:
        # An adb server can start between the two looks.
        owners = adb_port_listeners(port, run)
        unseen = not owners
    problem = preflight_problem(owners, port, unseen)
    if problem:
        raise BenchError(problem)
    return {"port": port, "owners": [{"command": c, "pid": p} for c, p in owners]}


def adb_port_record(port, run=None):
    """What listens on adb's server port now, for the summary: a failed look is recorded, not raised."""
    try:
        return {"port": port, "owners": [{"command": c, "pid": p} for c, p in adb_port_listeners(port, run)]}
    except (BenchError, subprocess.TimeoutExpired, OSError) as e:
        return {"port": port, "error": str(e)}


def adb_server_change(start, end):
    """A warning when the process on adb's port at the end of the run isn't the one from its start, else None."""
    if end.get("error"):
        return f"WARNING: couldn't check adb's port {end.get('port')} at the end of the run: {end['error']}"
    before = {o["pid"] for o in start.get("owners", [])}
    after = {o["pid"] for o in end.get("owners", [])}
    strangers = [f"{o['command']} (pid {o['pid']})" for o in end.get("owners", []) if o["command"] != "adb"]
    if strangers:
        return (f"**WARNING: at the end of the run {', '.join(strangers)} held adb's port {end.get('port')}, where adb "
                "looks for its server: adb commands after that point may have reached another hub.**")
    if before and before != after:
        return (f"WARNING: the adb server on port {end.get('port')} changed during the run (pid "
                f"{', '.join(sorted(before))} -> {', '.join(sorted(after)) or 'none'}): something on this Mac stopped "
                "or restarted it, and steps running then may have lost the hub.")
    return None


class ForwardFailed(ConnectionError):
    """adb gave this Mac no port that reaches the hub's: raised where a refused connection would be."""


def usb_devices(rows):
    """The `adb devices` rows that are USB devices: serials that aren't host:port, an emulator's or an mDNS name."""
    return [(serial, state) for serial, state in rows
            if not NETWORK_SERIAL.match(serial) and not serial.startswith("emulator-") and "._tcp" not in serial]


def pick_usb_device(rows, serial=None):
    """--usb: the one USB device adb lists, or the one --serial names. Stops before any device command otherwise."""
    usb = usb_devices(rows)
    listed = ", ".join(f"{s} ({state})" for s, state in usb)
    if serial is not None and NETWORK_SERIAL.match(serial):
        raise BenchError(f"--usb takes the hub's USB serial from `adb devices`, not the network address {serial}. "
                         "Nothing was sent to any hub.")
    if serial is not None:
        row = next((r for r in usb if r[0] == serial), None)
        if row is None:
            raise BenchError(f"adb doesn't list {serial} on USB ({'it lists ' + listed if usb else 'no USB device'}). "
                             "Check the cable between the Mac and the Control Hub's USB-C port. Nothing was sent to "
                             "any hub.")
    elif not usb:
        raise BenchError("adb lists no USB device. Plug the Mac into the Control Hub's USB-C port with a data cable (a "
                         "charge-only cable lists nothing), wait for the hub to boot, and rerun. Nothing was sent to "
                         "any hub.")
    elif len(usb) > 1:
        raise BenchError(f"adb lists {len(usb)} USB devices: {listed}. Unplug all but the Control Hub, or pass its "
                         "serial with --serial. Nothing was sent to any hub.")
    else:
        row = usb[0]
    if row[1] != "device":
        raise BenchError(f"adb lists {row[0]} as {row[1]}: "
                         + ("accept its debugging prompt" if row[1] == "unauthorized"
                            else "wait for it to finish booting, or replug the cable")
                         + ", then rerun. Nothing was sent to any hub.")
    return row[0]


def parse_listeners(text, ports):
    found = []
    for line in text.splitlines():
        fields = line.split()
        if len(fields) > 7 and fields[3] == TCP_LISTEN and ":" in fields[1]:
            address, _, port = fields[1].rpartition(":")
            if int(port, 16) in ports:
                found.append({"port": int(port, 16), "address": "any" if not address.strip("0") else address,
                              "uid": fields[7]})
    return found


def forward_reach(port, listeners):
    """(True, False or None, why): whether an adb forward reaches the hub's port, judged from the hub's own socket
    table, None when that couldn't be read. A connect from the Mac can't tell: adb accepts it on the Mac before it
    asks the hub."""
    if listeners is None:
        return None, "not judged over USB: the hub's socket table couldn't be read"
    found = sorted({entry["address"] for entry in listeners if entry["port"] == port})
    if not found:
        return False, "nothing listens on it on the hub"
    if any(a == "any" or a in HUB_LOOPBACK for a in found):
        return True, f"listens on {', '.join(found)}"
    return False, f"listens only on {', '.join(found)}, which a forward (to the hub's loopback) can't reach"


class WifiLink:
    """The hub's ports at its Wi-Fi address."""
    usb = False
    reboot_note = "the Mac rejoins its Wi-Fi by itself"
    lost_hint = "check that the Mac rejoined the hub's Wi-Fi"

    def __init__(self, host):
        self.host = host

    def address(self, port):
        return self.host, port

    def url(self, port, path=""):
        return f"http://{self.host}:{port}{path}"

    def http(self, method, port, path):
        return http_request(method, self.url(port, path))

    def unreachable(self, port, error):
        return (f"Can't open the dashboard websocket at ws://{self.host}:{port}/ ({error}). Join the Control Hub's "
                "Wi-Fi network, make sure the Robot Controller app is running, and that the dashboard is enabled "
                "(RC app menu → Enable Dashboard).")

    def dash_page(self):
        return self.url(RC_PAGE_PORT, "/dash"), None

    def rc_page(self):
        return self.url(RC_PAGE_PORT), None

    def restore_by_hand(self, serial, caution, name):
        return (f"Switch the config back by hand: open {self.url(RC_PAGE_PORT, '/dash')}, add the Hardware Config view, "
                f"and activate {name}.")

    def describe(self, serial):
        return f"Wi-Fi (adb {serial}; the hub's ports at {self.host})"

    def record(self):
        return {"link": "wifi", "host": self.host}

    def close(self):
        pass


class UsbLink:
    """--usb: the hub's ports through adb forwards from this Mac's loopback, each made on a port adb picks when first
    needed and made again once adb stops listing it (a reboot or a pulled cable drops the device with its forwards).
    close() removes only the forwards this run made."""
    usb = True
    reboot_note = "it comes back on the USB cable by itself: leave the cable in"
    lost_hint = "check the USB cable between the Mac and the Control Hub's USB-C port"

    def __init__(self, adb, log):
        self.adb, self.log = adb, log
        self.lock = threading.RLock()
        self.ours = {}
        self.pages = []
        self.events = []

    def _adb(self, args):
        try:
            return self.adb._raw(args, timeout=20)
        except subprocess.TimeoutExpired as e:
            raise ForwardFailed(f"adb {' '.join(args)} didn't answer within {e.timeout:.0f} s") from e

    def _listed(self):
        r = self._adb(["forward", "--list"])
        if r.returncode != 0:
            raise ForwardFailed(f"adb forward --list failed: {(r.stderr or r.stdout).strip()[:200]}")
        rows = {}
        for line in r.stdout.splitlines():
            parts = line.split()
            if len(parts) == 3:
                rows[parts[1]] = (parts[0], parts[2])
        return rows

    def _forward(self, local, remote, rebind=True):
        return self._adb(["-s", self.adb.serial, "forward"] + ([] if rebind else ["--no-rebind"])
                         + [local, f"tcp:{remote}"])

    def _forward_free(self, remote):
        r = self._forward("tcp:0", remote)
        # adb prints the port it picked for tcp:0 on a line of its own.
        port = (r.stdout.strip().splitlines() or [""])[-1].strip()
        if r.returncode != 0 or not port.isdigit():
            raise ForwardFailed(f"adb couldn't forward a port on this Mac to the hub's port {remote}: "
                                f"{(r.stderr or r.stdout).strip()[:200] or 'exit ' + str(r.returncode)}")
        return int(port)

    def _remove(self, local, remote, label="removed"):
        try:
            r = self._adb(["-s", self.adb.serial, "forward", "--remove", f"tcp:{local}"])
        except ForwardFailed as e:
            self.events.append({"notRemoved": remote, "local": local, "why": str(e)})
            return
        self.events.append({label if r.returncode == 0 else "notRemoved": remote, "local": local,
                            **({} if r.returncode == 0 else {"why": (r.stderr or r.stdout).strip()[:200]})})

    def local_port(self, remote):
        with self.lock:
            local = self.ours.get(remote)
            if local is not None:
                if self._listed().get(f"tcp:{local}") == (self.adb.serial, f"tcp:{remote}"):
                    return local
                del self.ours[remote]
                self.events.append({"gone": remote, "local": local})
                self.log(f"the forward from {LOOPBACK}:{local} to the hub's port {remote} is gone (the hub left adb: a "
                         "reboot or the cable); making a new one")
            local = self.ours[remote] = self._forward_free(remote)
            self.events.append({"made": remote, "local": local})
            self.log(f"forwarded {LOOPBACK}:{local} to the hub's port {remote} over USB")
            return local

    def address(self, port):
        return LOOPBACK, self.local_port(port)

    def url(self, port, path=""):
        return f"http://{LOOPBACK}:{self.local_port(port)}{path}"

    def http(self, method, port, path):
        try:
            url = self.url(port, path)
        except ForwardFailed as e:
            return {"error": f"{type(e).__name__}: {e}", "ms": 0.0}
        return http_request(method, url)

    def unreachable(self, port, error):
        return (f"Can't open the dashboard websocket on the hub's port {port} through an adb forward ({error}). Check "
                "the USB cable between the Mac and the Control Hub's USB-C port, make sure the Robot Controller app is "
                "running, and that the dashboard is enabled (RC app menu → Enable Dashboard).")

    def dash_page(self):
        """(slothboard's page on this Mac, None), or (None, why): the page's script always opens port 8000 of the
        address it came from, so this Mac's port 8000 must forward to the hub's too."""
        with self.lock:
            try:
                held = self._listed().get(f"tcp:{DASHBOARD_PORT}")
                if held != (self.adb.serial, f"tcp:{DASHBOARD_PORT}"):
                    r = self._forward(f"tcp:{DASHBOARD_PORT}", DASHBOARD_PORT, rebind=False)
                    if r.returncode != 0:
                        return None, ("the dashboard page can't work over the cable on this Mac: its script connects "
                                      f"to port {DASHBOARD_PORT} of the address it was loaded from, and "
                                      + (f"adb already forwards {LOOPBACK}:{DASHBOARD_PORT} to {held[1]} of {held[0]}"
                                         if held else f"{LOOPBACK}:{DASHBOARD_PORT} is taken "
                                                      f"({(r.stderr or r.stdout).strip()[:160]})"))
                    self.pages.append((DASHBOARD_PORT, DASHBOARD_PORT))
                    self.events.append({"made": DASHBOARD_PORT, "local": DASHBOARD_PORT, "forBrowser": True})
                return f"http://{LOOPBACK}:{self.local_port(RC_PAGE_PORT)}/dash", None
            except ForwardFailed as e:
                return None, f"no forward for the dashboard page: {e}"

    def rc_page(self):
        """(the Robot Controller's page on this Mac, None), or (None, why): its script opens a websocket on its own
        port + 1, so two adjacent ports here forward to the hub's 8080 and 8081."""
        with self.lock:
            tried = []
            for _ in range(RC_PAGE_TRIES):
                try:
                    page = self._forward_free(RC_PAGE_PORT)
                except ForwardFailed as e:
                    return None, str(e)
                self.pages.append((page, RC_PAGE_PORT))
                try:
                    r = self._forward(f"tcp:{page + 1}", RC_SOCKET_PORT, rebind=False)
                except ForwardFailed as e:
                    self.pages.remove((page, RC_PAGE_PORT))
                    self._remove(page, RC_PAGE_PORT, label="discarded")
                    return None, str(e)
                if r.returncode == 0:
                    self.pages.append((page + 1, RC_SOCKET_PORT))
                    self.events.append({"made": RC_PAGE_PORT, "local": page, "forBrowser": True})
                    self.events.append({"made": RC_SOCKET_PORT, "local": page + 1, "forBrowser": True})
                    return f"http://{LOOPBACK}:{page}", None
                tried.append(f"{page + 1}: {(r.stderr or r.stdout).strip()[:80]}")
                self.pages.remove((page, RC_PAGE_PORT))
                self._remove(page, RC_PAGE_PORT, label="discarded")
            return None, (f"no two adjacent free ports on this Mac for the RC page and its websocket (tried "
                          f"{'; '.join(tried)})")

    def restore_by_hand(self, serial, caution, name):
        return (f"Switch the config back by hand over the cable: {caution} Then run `adb -s {serial} forward "
                f"tcp:{DASHBOARD_PORT} tcp:{DASHBOARD_PORT}` and `adb -s {serial} forward tcp:0 tcp:{RC_PAGE_PORT}`, "
                f"which prints a port P; open http://{LOOPBACK}:P/dash, add the Hardware Config view, and activate "
                f"{name}. Afterwards `adb -s {serial} forward --remove tcp:{DASHBOARD_PORT}` and `adb -s {serial} "
                "forward --remove tcp:P` take the two forwards away.")

    def describe(self, serial):
        if serial is None:
            return "USB cable (stopped before picking a device: nothing was sent to any hub, no forward made)"
        made = [f"{e['made']} at {LOOPBACK}:{e['local']}" for e in self.events if "made" in e]
        gone = sum(1 for e in self.events if "gone" in e)
        removed = [str(e["local"]) for e in self.events if "removed" in e]
        discarded = [str(e["local"]) for e in self.events if "discarded" in e]
        kept = [f"{e['local']} ({e.get('why')})" for e in self.events if "notRemoved" in e]
        return (f"USB cable (adb {serial}; the hub's ports through adb forwards from this Mac: "
                f"{', '.join(made) or 'none made'}; {gone} found gone after the hub left adb and made again; removed "
                f"at the end: {', '.join(removed) or 'none'}"
                + (f"; discarded unused page ports: {', '.join(discarded)}" if discarded else "")
                + (f"; NOT removed: {', '.join(kept)}" if kept else "") + ")")

    def record(self):
        return {"link": "usb", "serial": self.adb.serial, "events": list(self.events)}

    def close(self):
        with self.lock:
            mine = [(local, remote) for remote, local in self.ours.items()] + list(self.pages)
            self.ours, self.pages = {}, []
            if not mine:
                return
            try:
                listed = self._listed()
            except (ForwardFailed, OtherHub) as e:
                self.events += [{"notRemoved": remote, "local": local, "why": str(e)} for local, remote in mine]
                self.log(f"!! couldn't list adb's forwards to remove this run's: {e}")
                return
            for local, remote in mine:
                if listed.get(f"tcp:{local}") == (self.adb.serial, f"tcp:{remote}"):
                    self._remove(local, remote)


def shipped_config_names(folder=None):
    """Hardware configs this checkout ships in res/xml: the files whose root element is <Robot>."""
    names = set()
    for path in sorted((folder or REPO_ROOT / SHIPPED_CONFIG_DIR).glob("*.xml")):
        try:
            if ElementTree.parse(path).getroot().tag == "Robot":
                names.add(path.stem)
        except ElementTree.ParseError:
            continue
    return names


def parse_config_listing(text):
    """{name: {size, modified}} from `ls -l /sdcard/FIRST/` lines for .xml files (other lines are skipped)."""
    out = {}
    for line in text.splitlines():
        m = CONFIG_LISTING.match(line.strip())
        if m:
            out[m.group(3)] = {"size": int(m.group(1)), "modified": m.group(2)}
    return out


def config_device_names(xml_text):
    """{'webcams': [...], 'ethernet': [...]} named in a hardware config, or None if its XML doesn't parse."""
    try:
        root = ElementTree.fromstring(xml_text)
    except ElementTree.ParseError:
        return None
    return {"webcams": [e.get("name") for e in root.iter("Webcam")],
            "ethernet": [e.get("name") for e in root.iter("EthernetDevice")]}


def active_config_source(entry, configs, shipped_folder=None, original_xml=None):
    """(name, XML text or None, where it came from) for the app's active-config setting (None: unread)."""
    if entry is None:
        return "unknown", None, "run-as couldn't read the app's settings, so the active config is unknown"
    name = entry.get("name") or NO_CONFIG
    if name == NO_CONFIG:
        return name, None, "no config is active"
    shipped = (shipped_folder or REPO_ROOT / SHIPPED_CONFIG_DIR) / f"{name}.xml"
    if (entry or {}).get("location") == "RESOURCE" or (name not in configs and shipped.exists()):
        if shipped.exists():
            return name, shipped.read_text(), "shipped in res/xml (this checkout's copy)"
        return name, None, "shipped in the installed app's res/xml, which this checkout doesn't have"
    if original_xml is not None:
        return name, original_xml, f"saved in {CONFIG_DIR} (as it was before the run)"
    if name in configs:
        return name, configs[name], f"saved in {CONFIG_DIR}"
    return name, None, f"not in {CONFIG_DIR} or this checkout's res/xml"


def saved_config_findings(configs, shipped, active_name=None, active_xml=None):
    """Rows, warnings and the active-config check for the hardware configs saved on a hub ({name: XML text})."""
    rows, warnings = [], []
    for name in sorted(configs):
        bench = name.startswith("bench_")
        devices = config_device_names(configs[name])
        stale = not bench and name not in shipped
        rows.append({"name": name, "bench": bench, "stale": stale, "parses": devices is not None,
                     "webcams": (devices or {}).get("webcams"), "ethernet": (devices or {}).get("ethernet")})
        if bench:
            continue
        if devices is None:
            warnings.append(f"{name}: its XML doesn't parse, so the app can't load it")
            continue
        for webcam in devices["webcams"]:
            if webcam != CELL_TIP_WEBCAM:
                warnings.append(f"{name}: Webcam {webcam!r}; CellTipCamera opens {CELL_TIP_WEBCAM!r}, so its OpModes "
                                "fail INIT on this config")
        for device in devices["ethernet"]:
            if device != LIMELIGHT_DEVICE:
                warnings.append(f"{name}: EthernetDevice {device!r}; the Limelight code looks for {LIMELIGHT_DEVICE!r}")
        if stale:
            warnings.append(f"{name}: saved on the hub but not a config this checkout ships in res/xml (a leftover, "
                            "for one from a renamed or deleted config)")
    check = None
    if active_xml is not None:
        devices = config_device_names(active_xml)
        webcams = None if devices is None else devices["webcams"]
        wrong = [w for w in webcams or [] if w != CELL_TIP_WEBCAM]
        check = (webcams is not None and not wrong,
                 "its XML doesn't parse" if webcams is None else
                 f"webcams {webcams}" if webcams else "it names no webcam")
    return {"rows": rows, "warnings": warnings, "check": check, "active": active_name}


def window_counts(events, t0, t1):
    """What a MessageCounter got in [t0, t1): messages by type, telemetry packets, telemetry clears, images, logcat."""
    by_type = collections.Counter()
    out = {"telemetry packets": 0, "telemetry clears": 0, "images": 0, "logcat": 0}
    for t, kind, packets in events:
        if not t0 <= t < t1:
            continue
        by_type[kind] += 1
        if kind == "RECEIVE_TELEMETRY":
            if packets:
                out["telemetry packets"] += packets
            else:
                out["telemetry clears"] += 1
        elif kind == "RECEIVE_IMAGE":
            out["images"] += 1
        elif kind in ("RECEIVE_LOGCAT_ERRORS", "RECEIVE_LOGCAT_LINES"):
            out["logcat"] += 1
    out["types"] = dict(by_type)
    out["seconds"] = round(t1 - t0, 2)
    return out


def first_packet_after(events, t0):
    return next((round(t - t0, 2) for t, kind, packets in events if t >= t0 and kind == "RECEIVE_TELEMETRY" and packets),
                None)


def dashboard_auto_enable(prefs_text):
    """slothboard's saved switch (key autoEnable in shared_prefs/FtcDashboard.xml): True or False, or None when the
    file or the key is missing, which slothboard reads as on."""
    if not prefs_text.strip() or "No such file" in prefs_text:
        return None
    try:
        root = ElementTree.fromstring(prefs_text)
    except ElementTree.ParseError as e:
        raise BenchError(f"can't parse the app's {DASHBOARD_PREFS} ({e}): {prefs_text.strip()[:200]!r}")
    for entry in root.iter("boolean"):
        if entry.get("name") == DASHBOARD_AUTO_ENABLE_KEY:
            return entry.get("value") == "true"
    return None


def streamed(counts):
    return {k: counts.get(k, 0) for k in DASH_OFF_STREAMED if counts.get(k, 0)}


def dashboard_off_checks(rec):
    """(name, ok, detail, fail_means) for Bench: Dashboard Off; a phase that didn't run fails its checks."""
    out = []
    base = rec.get("baseline") or {}
    proxy_before = [p for p in DASH_OFF_PROXY_PORTS if (base.get("listeners") or {}).get(p)]
    switch = rec.get("switch") or {}
    on = switch.get("onCounts") or {}
    result = switch.get("result") or {}
    why_not = rec.get("failed") or "an earlier phase failed"

    def ports_closed(label, listeners):
        name = f"dashboard off: the Limelight proxy ports are closed after {label}"
        if listeners is None:
            out.append((name, False, f"not run: {why_not}", DASH_OFF_PORTS_MEANS))
            return
        still = [p for p in proxy_before if listeners.get(p)]
        unjudged = [p for p in DASH_OFF_PROXY_PORTS if p not in proxy_before]
        out.append((name, bool(proxy_before) and not still,
                    f"listening {still or 'none'} of {proxy_before}, the ones that listened before the switch"
                    + (f"; not judged, since they didn't listen before it either: {unjudged}" if unjudged else ""),
                    DASH_OFF_PORTS_MEANS if proxy_before else
                    "none of the proxy ports listened while the dashboard was on, so this check proves nothing"))

    def silent(label, counts, socket_open, served=None):
        name = f"dashboard off: nothing streams after {label}"
        if counts is None:
            out.append((name, False, f"not run: {why_not}", DASH_OFF_STREAM_MEANS))
            return
        got = streamed(counts)
        out.append((name, bool(socket_open) and not got and served in (None, 0),
                    f"{got or 'no telemetry, image or logcat message'} in {counts.get('seconds')} s on a socket "
                    f"{'open throughout' if socket_open else 'that closed during the window'}; every message "
                    f"{counts.get('types')}"
                    + (f"; frames the bench's camera source served after the switch {served}" if served is not None
                       else ""), DASH_OFF_STREAM_MEANS))

    out.append(("dashboard off: telemetry and the camera stream reached the runner while the dashboard was on (setup)",
                on.get("telemetry packets", 0) > 0 and on.get("images", 0) > 0,
                f"{on.get('telemetry packets', 0)} telemetry packets and {on.get('images', 0)} images in "
                f"{on.get('seconds')} s; frames the bench's source served {result.get('onFramesServed')}",
                DASH_OFF_SETUP_MEANS))
    disabled = (result.get("enabledAtStart") is True and result.get("enabledAfterDisable") is False
                and switch.get("offSeen") is True)
    out.append(("dashboard off: slothboard's disable() switched it off (isEnabled, then the status)", disabled,
                f"isEnabled {result.get('enabledAtStart')} before, {result.get('enabledAfterDisable')} after; the status "
                f"said off {switch.get('offSeen')}" + (f"; {switch['error']}" if switch.get("error") else ""),
                DASH_OFF_DISABLE_MEANS))
    silent("the switch, with the OpMode still sending telemetry and starting a camera stream", switch.get("offCounts"),
           switch.get("socketOpen"), served=result.get("offFramesServed", "not reported"))
    ports_closed("the switch", switch.get("listeners"))
    done = {p.get("name"): p for p in rec.get("phases") or []}
    for name in rec.get("planned") or []:
        p = done.get(name) or {}
        what = DASH_OFF_PHASES[name]
        ran = "statusEnabled" in p
        out.append((f"dashboard off: still off after {what}",
                    ran and p.get("statusEnabled") is False and p.get("autoEnable") is False,
                    (f"status enabled {p.get('statusEnabled')}, saved autoEnable {p.get('autoEnable')}"
                     + (f"; back {p.get('backSeconds')} s after it started" if p.get("backSeconds") is not None else ""))
                    if ran else f"not run: {p.get('error') or why_not}", DASH_OFF_PERSIST_MEANS))
        silent(what, p.get("window") if ran else None, p.get("socketOpen"))
        if ran:
            out.append((f"dashboard off: an INIT sent after {what} is ignored", p.get("initIgnored") is True,
                        f"the robot stayed idle {p.get('initIgnored')}", DASH_OFF_INIT_MEANS))
        ports_closed(what, p.get("listeners") if ran else None)
    r = rec.get("recovery") or {}
    listening = r.get("listeners") or {}
    back = (not r.get("error") and r.get("statusEnabled") is True
            and r.get("enabledSeconds") is not None and r["enabledSeconds"] <= DASH_OFF_RECOVERY_S
            and r.get("opModeListSeconds") is not None and r["opModeListSeconds"] <= DASH_OFF_RECOVERY_S
            and (r.get("probePackets") or 0) > 0 and all(listening.get(p) for p in DASH_OFF_RECOVERY_PORTS))
    out.append((f"dashboard off: the adb recovery turns it back on within {DASH_OFF_RECOVERY_S:.0f} s (enabled, the "
                f"OpMode list, Bench: Probe's telemetry, ports {' and '.join(map(str, DASH_OFF_RECOVERY_PORTS))} "
                "listening)", back,
                (f"failed: {r['error']}; " if r.get("error") else "")
                + f"enabled after {r.get('enabledSeconds')} s, OpMode list after {r.get('opModeListSeconds')} s, "
                  f"{r.get('probePackets')} Probe packets (first {r.get('firstPacketSeconds')} s after the command), "
                  f"listening {[p for p in DASH_OFF_RECOVERY_PORTS if listening.get(p)]}, saved autoEnable "
                  f"{r.get('autoEnable')}", DASH_OFF_RECOVERY_MEANS))
    if rec.get("limelight"):
        http = r.get("limelightHttp") or {}
        out.append(("dashboard off: the Limelight web UI answers on 5801 after the recovery", http.get("code") == 200,
                    f"{http or 'not asked'}", DASH_OFF_RECOVERY_MEANS))
    return out


class LogcatStream:
    """Streams the hub's logcat into a file for a step too long for the hub's ring buffer."""

    def __init__(self, adb, path):
        self.path = path
        self.file = open(path, "w")
        self.proc = subprocess.Popen([adb.path, "-s", adb.serial, "shell", f"logcat -v threadtime -T '{adb.hub_time()}'"],
                                     stdout=self.file, stderr=subprocess.STDOUT, text=True)

    def stop(self):
        self.proc.terminate()
        try:
            self.proc.wait(timeout=10)
        finally:
            self.file.close()


class Adb:
    def __init__(self, serial, log, usb=False):
        self.path = find_adb()
        self.serial = serial
        self.log = log
        self.usb = usb
        self.private = False
        self.dated_logcat = None
        self.expect_serialno = None
        self.other_hub = None

    def _raw(self, args, timeout=60):
        if self.other_hub is not None:
            raise OtherHub(self.other_hub)
        return subprocess.run([self.path] + args, capture_output=True, text=True, timeout=timeout)

    def run(self, *args, check=True, timeout=60):
        r = self._raw(["-s", self.serial] + list(args), timeout)
        if check and r.returncode != 0:
            raise BenchError(f"adb {' '.join(args)} failed: {(r.stderr or r.stdout).strip()}")
        return r.stdout

    def devices(self):
        return [line.split()[:2] for line in self._raw(["devices"]).stdout.splitlines()[1:] if len(line.split()) >= 2]

    def ensure_connected(self):
        self._listed_as_device()
        self.check_serialno()

    def check_serialno(self):
        """With expect_serialno, every check reads the hub's ro.serialno: adb's help doesn't say whether its server
        reconnects a dropped `adb connect` device by itself, maybe to another hub at the same address."""
        if self.expect_serialno is None:
            return
        now = self.run("shell", "getprop ro.serialno").strip()
        if now != self.expect_serialno:
            self.other_hub = (f"a different hub answered at {self.serial} (ro.serialno {now or 'empty'}, not "
                              f"{self.expect_serialno}); nothing more was sent to it")
            self.log(f"!! {self.other_hub}")
            raise OtherHub(self.other_hub)

    def _listed_as_device(self):
        if self.usb:
            state = None
            for attempt in range(3):
                state = {row[0]: row[1] for row in self.devices()}.get(self.serial)
                if state == "device":
                    return
                if state == "unauthorized":
                    break
                time.sleep(1 + attempt)
            raise BenchError(f"adb doesn't list {self.serial} on USB" + (f" (it lists it as {state})" if state else "")
                             + ": check the cable between the Mac and the Control Hub's USB-C port; a rebooting hub is "
                               "off adb for about a minute")
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
        if any(line.split()[:2] == [self.serial, "device"] for line in self._raw(["devices"]).stdout.splitlines()[1:]):
            return
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

    def thread_names(self, pid, first=None):
        """The app's thread names (comm, at most 15 characters) from the first listing that works on this hub,
        trying `first` before the others; (None, [], tried) when none does."""
        tried = []
        for label, kind, command in sorted(THREAD_LISTINGS, key=lambda listing: listing[0] != first):
            out = self.run("shell", command.format(package=RC_PACKAGE, pid=pid) + " 2>/dev/null", check=False)
            lines = [line.strip() for line in out.splitlines() if line.strip()]
            names = lines if kind == "comm" else [line.split()[-1] for line in lines[1:]]
            if len(names) >= THREAD_LISTING_MIN:
                return label, names, tried
            tried.append(f"{label}: {len(names)} names" + (f", first line {lines[0][:80]!r}" if lines else ""))
        return None, [], tried

    def hub_time(self):
        # logcat -T takes a missing year as the current one: a start before a clock set or local midnight reads nothing.
        if self.dated_logcat is None:
            probe = self.run("shell", "date '+%Y-%m-%d %H:%M:%S'").strip() + ".000"
            self.mark("logcat year probe")
            self.dated_logcat = "logcat year probe" in self.logcat_matches(probe, "logcat year probe")
        fmt = "%Y-%m-%d %H:%M:%S" if self.dated_logcat else "%m-%d %H:%M:%S"
        return self.run("shell", f"date '+{fmt}'").strip() + ".000"

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

    def pull_bench_file(self, name, local):
        if self.private:
            local.write_text(self.read_bench_file(name))
            return
        r = self._raw(["-s", self.serial, "pull", f"{HUB_BENCH_DIR}/{name}", str(local)])
        if r.returncode != 0:
            raise BenchError(f"the hub has no {HUB_BENCH_DIR}/{name}: {r.stderr.strip()}")

    def mark(self, text):
        self.run("shell", f"log -t BENCH_RUNNER {shlex.quote(text)}", check=False)

    def logcat_matches(self, hub_time, *needles):
        args = " ".join(f"-e {shlex.quote(n)}" for n in needles)
        return self.run("shell", f"logcat -d -v threadtime -T '{hub_time}' | grep -F {args}", check=False)

    def active_config(self):
        """The config the app loads on its next robot restart, from its own SharedPreferences. run-as needs a
        debuggable app, which the IDE's Run button installs."""
        return self.active_config_entry()["name"]

    def _read_prefs(self):
        r = self._raw(["-s", self.serial, "shell", f"run-as {RC_PACKAGE} cat {RC_PREFS}"])
        try:
            return ElementTree.fromstring(r.stdout)
        except ElementTree.ParseError:
            raise BenchError(f"run-as can't read the Robot Controller's settings: {(r.stderr or r.stdout).strip()[:200]}")

    def active_config_entry(self):
        for entry in self._read_prefs().iter("string"):
            if entry.get("name") == PREF_ACTIVE_CONFIG:
                # RobotConfigFile.toString(), a JSON object.
                return json.loads(entry.text)
        return {"name": NO_CONFIG, "location": "NONE", "resourceId": 0}

    def stage_active_config(self, name, scratch, config=None):
        """Pushes a copy of the app's settings naming the active config, for switch_app().
        Read while the app is idle, so the file matches what it holds in memory."""
        root = self._read_prefs()
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
        return parse_listeners(self.run("shell", "cat /proc/net/tcp /proc/net/tcp6", check=False), ports)

    def listeners_or_none(self, ports):
        """listeners(), or None when the hub's socket table couldn't be read (an empty list means nothing listens)."""
        out = self.run("shell", "cat /proc/net/tcp /proc/net/tcp6", check=False)
        return parse_listeners(out, ports) if "local_address" in out else None

    def config_names(self):
        out = self.run("shell", f"ls -1 {CONFIG_DIR}")
        return {line.strip()[:-4] for line in out.splitlines() if line.strip().endswith(".xml")}

    def read_config(self, name):
        return self.run("shell", f"cat {shlex.quote(f'{CONFIG_DIR}/{name}.xml')}")

    def remove_config(self, name):
        self.run("shell", f"rm {shlex.quote(f'{CONFIG_DIR}/{name}.xml')}")


def usb_rev_hub_serials(adb):
    """The serials of the REV hubs on the Control Hub's USB ports (an Expansion Hub on USB), read from sysfs."""
    vendor, product = REV_HUB_USB_ID
    out = adb.run("shell", "for d in /sys/bus/usb/devices/*; do "
                  f"[ \"$(cat $d/idVendor 2>/dev/null)\" = {vendor} ] && "
                  f"[ \"$(cat $d/idProduct 2>/dev/null)\" = {product} ] && cat $d/serial; done; true")
    return [line.strip() for line in out.splitlines() if line.strip()]


def attr(value):
    return xml.sax.saxutils.quoteattr(str(value))


def hub_config(devices, extra=(), expansion=None, usb_expansion=None):
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
            f"{usb_expansion_block(usb_expansion)}"
            f"{tail}"
            "</Robot>\n")


def usb_expansion_block(usb_expansion):
    if usb_expansion is None:
        return ""
    serial, address, devices = usb_expansion
    return (f'    <LynxUsbDevice name="Expansion Hub Portal" serialNumber={attr(serial)} parentModuleAddress="{address}">\n'
            f'        <LynxModule name="Expansion Hub {address}" port="{address}">\n'
            + "".join(f"            {d}\n" for d in devices)
            + "        </LynxModule>\n"
            "    </LynxUsbDevice>\n")


def pinpoint_tag(bus):
    return f'<goBILDAPinpoint name="pinpoint" port="0" bus="{bus}" />'


def octoquad_tag(bus):
    return f'<OctoQuadFTC name="octoquad" port="0" bus="{bus}" />'


# LynxModuleImuType.name() as the probe reports it -> the SDK config tag (LynxConstants.EMBEDDED_*_IMU_XML_TAG).
IMU_TAGS = {"BNO055": "LynxEmbeddedIMU", "BHI260": "ControlHubImuBHI260AP"}


DRIVE_MOTORS = (
    '<goBILDA5202SeriesMotor name="fl" port="0" />',
    '<goBILDA5202SeriesMotor name="fr" port="1" />',
    '<goBILDA5202SeriesMotor name="bl" port="2" />',
    '<goBILDA5202SeriesMotor name="br" port="3" />',
)


def imu_tag(imu_type):
    if imu_type not in IMU_TAGS:
        raise BenchError(f"The probe read the Control Hub's embedded IMU as {imu_type!r}, which the bench config can't "
                         "name; power-cycle the hub and rerun, and tell Claude if it persists.")
    return f'<{IMU_TAGS[imu_type]} name="imu" port="0" bus="0" />'


def main_devices(imu_type, pinpoint=False, octoquad_bus=None):
    devices = list(DRIVE_MOTORS) + [
        '<Servo name="benchServo" port="0" />',
        '<Servo name="benchServo2" port="1" />',
        '<AnalogInput name="floodgate" port="0" />',
        '<AnalogInput name="analog0" port="2" />',
        '<AnalogInput name="analog1" port="3" />',
        imu_tag(imu_type),
    ]
    if pinpoint:
        devices.append(pinpoint_tag(1))
    if octoquad_bus is not None:
        devices.append(octoquad_tag(octoquad_bus))
    return devices


def turret_devices(imu_type):
    return list(DRIVE_MOTORS) + ['<Servo name="TurretR" port="0" />', '<Servo name="TurretL" port="1" />',
                                 imu_tag(imu_type)]


def turret_zero_devices(imu_type):
    """br on motor port 2 only to read encoder port 2, as on the robot; no other motor or servo is named."""
    return ['<goBILDA5202SeriesMotor name="br" port="2" />', octoquad_tag(OCTOQUAD_BUS), imu_tag(imu_type)]


def extras(webcam_serial, limelight):
    out = []
    if webcam_serial is not None:
        out.append(f"<Webcam name=\"aprilTagDetector\" serialNumber={attr(webcam_serial)} />")
    if limelight is not None:
        out.append(limelight_tag(limelight["ipAddress"], limelight["serial"]))
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
            found += module_port_hits(module)
    return found


def module_port_hits(module):
    found = []
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


def limelight_tag(ip_address, serial=LIMELIGHT_CONFIG_SERIAL):
    return (f"<EthernetDevice name=\"limelight\" serialNumber={attr(serial)} port=\"0\" "
            f"ipAddress={attr(ip_address)} />")


def counted(summary):
    return isinstance(summary, dict) and bool(summary.get("count"))


def fmt(summary, unit="ms"):
    if not counted(summary):
        return "n/a"
    return (f"p50 {summary['p50']:.2f} / p95 {summary['p95']:.2f} / p99 {summary.get('p99', float('nan')):.2f} / "
            f"max {summary['max']:.2f} {unit} (n={summary['count']})")


def fmt_volts(summary):
    if not counted(summary):
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
    if not counted(summary):
        return "n/a"
    return f"{summary['p50']:.2f} / {summary['p95']:.2f} / {summary['max']:.2f}"


def p50_diff(before, after):
    if not (counted(before) and counted(after)):
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


LOGCAT_STAMP = re.compile(r"^(?:\d{4}-)?(\d\d)-(\d\d) (\d\d):(\d\d):(\d\d)\.(\d{3})")


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


PhotonCheck = collections.namedtuple("PhotonCheck", "section name ok detail fail_means")

PHOTON_DRAIN_LIMIT_MS = 50
PHOTON_STALE_MS = 100
PHOTON_CLEAR_LIMIT_MS = 50
PHOTON_WRITE_CALL_LIMIT_MS = 20
PHOTON_RETRANSMIT_WINDOWS_MS = ((100, 140), (200, 250))
PHOTON_ABANDON_WINDOW_MS = (250, 310)
PHOTON_SDK_CACHE_MS = 480
PHOTON_STORM_STALL_MS = 50
PHOTON_REPLUG_MARGIN_MS = 50
PHOTON_REPLUG_SINGLE_WRITE_MS = 250 + PHOTON_REPLUG_MARGIN_MS
PHOTON_REPLUG_RECOVER_MS = 1000
PHOTON_RESET_MARGIN_MS = 50
PHOTON_RESET_LIMIT_MS = 2000
PHOTON_OFF_SLACK = (1.15, 0.3)
PHOTON_SOAK_LOSS_TARGET = 4.4e-7
PHOTON_POWER_VIOLATION_RAW = int(0.01 * 32767)
PHOTON_LOG_NEEDLES = ("retransmitting", "timeout: abandoning waiting", "abandoning lock", "incorrect owner releasing",
                      "releasing ownerless", "unable to find originating", "invalid checksum received", "placeholder: ",
                      "exception thrown during lynx communication", "readStatusQuery: cbExpected")
PHOTON_LOG_FORBIDDEN = PHOTON_LOG_NEEDLES[1:]
PHOTON_SLOW_VERSION = "slow version-string reply"
PHOTON_SLOW_VERSION_MARKS = ("cmd=LynxReadVersionStringCommand", "packetid=0x9030")
PHOTON_COUNTERS_ZERO = ("retransmits", "abandonedWrites", "supersededWrites", "gateFallbacks", "postStopFailSafes",
                        "notRespondingFallbacks", "deviceNotReadyFallbacks", "writeFailures", "clearedAtInit",
                        "orphanDatagrams", "numberClashes")
PHOTON_LEFTOVERS = ("inFlight", "pendingEntries", "standIns", "routes")
PHOTON_EARLY_LOST = ("writeFailures", "deviceNotReadyFallbacks", "notRespondingFallbacks", "abandonedWrites")
PHOTON_PHASE_LINE = re.compile(r"\bBENCH\s*:\s*phase (.+?)\s*$")
# The SDK's xmit lock gives up on its owner after 500 ms and rechecks every timeout/4.
PHOTON_LOCK_TIMEOUT_MS = 500
PHOTON_LOCK_RECHECK_MS = 125
# STOP mark to LinearOpMode's interrupt: OpModeManagerImpl.callActiveOpModeStop fail-safes every hub first (seen <= 37 ms).
PHOTON_STOP_INTERRUPT_MS = 60
PHOTON_STOP_LEAK_MS = (PHOTON_LOCK_TIMEOUT_MS - 5,
                       PHOTON_STOP_INTERRUPT_MS + PHOTON_LOCK_TIMEOUT_MS + PHOTON_LOCK_RECHECK_MS + 15)
PHOTON_STOP_BLIND_MS = PHOTON_STOP_LEAK_MS[1]
PHOTON_LEAK_ALPHA = 0.05
PHOTON_LEAK_BASE_RATE = 1 / 40
PHOTON_LEAK_RISES = (1.5, 2, 3, 4, 5, 6, 8, 10, 15, 20, 30, 40)
PHOTON_STOP_MARK = "auto-stopping OpMode"
PHOTON_ABANDON = re.compile(r"abandoning lock: old=(\w+)\((-?\d+)\)")
PHOTON_EH_PINPOINT_BUS = 1
PHOTON_I2C_MIN_READS = 50
PHOTON_I2C_CONTROL_ATTEMPTS = 200
PHOTON_I2C_PHASES = ("off", "on2", "on8")
PHOTON_I2C_DEVICES = (("pinpoint", "goBILDA Pinpoint", 0x31), ("octoquad", "OctoQuad", 0x30))
PHOTON_OVERLAP_MODES = ("off", "on8", "on4", "on2")
PHOTON_OVERLAP_SETS = (("6 + 0", "6 on the Control Hub"), ("0 + 6", "6 on the Expansion Hub"),
                       ("6 + 6", "12, the Control Hub's then the Expansion Hub's"),
                       ("6 + 6 alternating", "12, hubs alternating"), ("3 + 3", "3 + 3"), ("5 + 1", "5 + 1"),
                       ("1 + 5", "1 + 5"))
PHOTON_OVERLAP_ORDERS = (("6 + 6", "hub by hub"), ("6 + 6 alternating", "alternating"))
PHOTON_OVERLAP_MIN_LOOPS = 300
PHOTON_OVERLAP_VERDICT_MODE = "on8"

PHOTON_TESTS = (
    ("install", "Installed Photon", "the hub runs the Photon built from these sources: Photon is pinned, so only a "
                                    "full install changes it and a hot load never does"),
    ("ports", "Port guard", "nothing is plugged into the motor ports the Photon benches drive, on either hub (servo "
                            "ports aren't probed: check them by eye)"),
    ("stock", "Stock baseline", "an OpMode Photon never touched, right after an app restart, writes every value exactly, "
                                "and can't turn Photon on"),
    ("gain", "Gain over stock", "how much faster Photon makes the writes, and that installed-but-disabled costs nothing"),
    ("phases", "Every write lands", "with Photon off and at 8/4/2/1 parallel, every write leaves the exact register value "
                                   "the SDK would, nothing is lost, retransmitted, left in flight (by Photon's count "
                                   "and in the SDK's own command map), stale past one retransmit interval or over the "
                                   "limit, and every write Photon should parallelize was parallelized"),
    ("routing", "Routing", "every hub's replies reach the hub object the OpMode writes through, after every app restart "
                           "and in both swap orders, so a hub marked unresponsive recovers on its next reply"),
    ("lost", "Lost writes", "a parallel write whose ACK never comes is retransmitted and given up at the SDK's own "
                            "intervals, with 0-7 other writes in flight around it, never blocks the caller, frees its "
                            "slot and its SDK entry, and the next identical write really reaches the hub; one "
                            "superseded by a newer write to the same channel ends quietly"),
    ("storm", "Write storm", "writes past the parallel limit wait a bounded time and never stall the loop"),
    ("threads", "Two writer threads", "parallel writes from two threads, with Photon toggled from a third, stay correct, "
                                      "within the limit and leave nothing behind"),
    ("stop", "STOP", "after STOP's fail-safe no motor is re-energized and no servo re-enabled, with STOP landing "
                     "anywhere in the loop, iterative and linear (Photon on and off), judged by a read of every channel "
                     "after the fail-safe and by a sampler thread, both proven able to fail; no trial crashed or "
                     "restarted; a LinearOpMode's STOP interrupt leaks the SDK's network lock no more often with Photon "
                     "than without"),
    ("race", "STOP inside a write", "STOP landing between a motor write going out and the call returning leaves the "
                                    "motor off"),
    ("crash", "Crash with writes in flight", "an OpMode that throws with parallel writes that will never be acknowledged "
                                             "leaves no motor powered, every such write accounted for, and nothing in "
                                             "flight or in the SDK's command map at the next INIT"),
    ("consecutive", "Consecutive OpModes", "Photon, non-Photon, Photon, crash, Photon: every INIT starts clean and "
                                           "routed, and a non-Photon OpMode can't enable it"),
    ("reset", "Hub reset recovery", "after a hub reset every output is back at target no later with Photon than "
                                    "without (the summary says whether the SDK learned of the reset from the hub)"),
    ("soak", "Write soak", "zero lost, stale or retransmitted writes over a production-rate run, with the 95% upper "
                           "bound on the per-write loss rate"),
    ("sloth", "deploySloth", "after a hot load a Photon OpMode still enables the one pinned PhotonCore and parallelizes"),
    ("rs485", "RS485 Expansion Hub", "an Expansion Hub on RS485 is never parallelized, and the Control Hub's parallel "
                                     "writes beside its locked ones and a poller's reads leave every value right and "
                                     "no lock warning"),
    ("replug", "Cable pulls", "pulling and replugging the Expansion Hub's USB or power stalls the loop no longer with "
                              "Photon than without, its writes fall back while it's gone and are right again within 1 s"),
    ("i2c", "I2C", "two threads per device read constant registers of the Pinpoint and the OctoQuad every 10 ms while the "
                   "OpMode thread writes every channel, with Photon off, at 2 and at 8 parallel, and every byte is right; "
                   "after Photon's INIT every I2C client is Photon's; a client still on the replaced hub object shares "
                   "its I2C lock; and unlocked transactions beside a locked reader show whether a wrong byte would be seen"),
    ("logcat", "SDK warnings", "no lock, ownership, checksum, abandon, I2C or placeholder warnings from the SDK in any "
                               "Photon step, except what a step causes on purpose, and every step's log is complete"),
)


PHOTON_TITLES = {section: title for section, title, _ in PHOTON_TESTS}


def gain_text(gain):
    return f"{gain:.2f}x" if finite(gain) else "-"


def expansion_port_devices(xml_text, usb_serial, address):
    """Devices a config puts on the bench ports of an Expansion Hub: a USB one (any, or the one with usb_serial) or an
    RS485 child at address."""
    try:
        root = ElementTree.fromstring(xml_text)
    except ElementTree.ParseError as e:
        raise BenchError(f"can't parse a hardware config to check the Expansion Hub's bench ports: {e}")
    found = []
    for usb in root.iter("LynxUsbDevice"):
        embedded = usb.get("serialNumber") == "(embedded)"
        for module in usb.iter("LynxModule"):
            child = embedded and module.get("port") != usb.get("parentModuleAddress") and module.get("port") == str(address)
            own_usb = not embedded and (usb_serial is None or usb.get("serialNumber") == usb_serial)
            if child or own_usb:
                found += [f"Expansion Hub {hit}" for hit in module_port_hits(module)]
    return found


def photon_events(readback, n=3):
    events = (readback or {}).get("events") or []
    return [{k: e.get(k) for k in ("phase", "channel", "hub", "expected", "got", "photonEnabled", "msSinceWrite",
                                   "inFlightOnHub")} for e in events[:n]]


def finite(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


def photon_phase_problems(phase):
    """What is wrong with one PhotonWorkload phase result; empty when it passed."""
    if not isinstance(phase, dict) or not phase:
        return ["no result"]
    problems = []
    par = phase.get("parallel")
    on = isinstance(par, int) and par > 0
    rb = phase.get("readback") or {}
    if rb.get("mismatches") != 0:
        problems.append(f"{rb.get('mismatches')} of {rb.get('checks')} read-backs wrong, e.g. {photon_events(rb)}")
    end = phase.get("endReadback") or {}
    if phase.get("allAtEnd") is not True or end.get("mismatches") != 0:
        problems.append(f"after the drain {end.get('mismatches')} channel(s) not at their last value "
                        f"{photon_events(end)}")
    for key in ("drainBeforeMs", "drainAfterMs"):
        v = phase.get(key)
        if not finite(v) or v > PHOTON_DRAIN_LIMIT_MS:
            problems.append(f"{key} {v} (must be finite and at most {PHOTON_DRAIN_LIMIT_MS} ms)")
    if phase.get("inFlightAfterDrain") != 0:
        problems.append(f"{phase.get('inFlightAfterDrain')} write(s) still in flight after the drain")
    problems += leftover_problems(phase.get("leftoversAfterDrain"), "after the drain")
    limit = par if on else 0
    for key, what in (("maxInFlight", "writes in flight (Photon's slots)"),
                      ("maxPendingEntries", "pending entries (Photon's map)"),
                      ("maxStandIns", "stand-ins in the SDK's command map")):
        counts = phase.get(key)
        if not isinstance(counts, dict) or not counts:
            problems.append(f"no {key}")
            continue
        for hub, n in counts.items():
            if not isinstance(n, int) or n > limit:
                problems.append(f"{hub} had {n} {what} at a limit of {limit}")
    if on and phase.get("photonEnabled") is not True:
        problems.append("Photon wasn't enabled")
    for hub, ms in (phase.get("maxPendingMs") or {}).items():
        if not finite(ms) or ms > PHOTON_STALE_MS:
            problems.append(f"{hub} held an unacknowledged write for {ms} ms (stale past {PHOTON_STALE_MS} ms)")
    c = phase.get("counters")
    if not isinstance(c, dict):
        return problems + ["no counters"]
    replaced = replaced_reads(phase)
    for key in PHOTON_COUNTERS_ZERO:
        if key == "orphanDatagrams" and replaced is not None:
            continue
        if c.get(key, 0) != 0:
            problems.append(f"{key} {c.get(key)}")
    if replaced is not None and (not replaced or c.get("orphanDatagrams", 0) < 2 * replaced):
        problems.append(f"orphanDatagrams {c.get('orphanDatagrams')} for {replaced} reads through the replaced hub object "
                        "(each needs at least its ACK and its data back)")
    if c.get("nacks"):
        problems.append(f"NACKs {c.get('nacks')}")
    slot_waits_expected = on and (phase.get("burstPerHub") or 0) > par
    if c.get("slotTimeouts", 0) and not slot_waits_expected:
        problems.append(f"slotTimeouts {c.get('slotTimeouts')} with at most {phase.get('burstPerHub')} writes per hub "
                        f"at a limit of {par}")
    issued = phase.get("issuedOnParallelHubs")
    parallel_writes = c.get("parallelWrites", 0)
    if on and phase.get("countExact", True) and parallel_writes + c.get("slotTimeouts", 0) != issued:
        problems.append(f"{parallel_writes} parallel writes + {c.get('slotTimeouts', 0)} slot timeouts for {issued} "
                        "writes issued on parallel-capable hubs")
    if not on and parallel_writes != 0:
        problems.append(f"{parallel_writes} parallel writes with Photon {'off' if par == 0 else 'out of the OpMode'}")
    if phase.get("notResponding"):
        problems.append(f"not responding: {phase.get('notResponding')}")
    if phase.get("voltageReadsUnder9V"):
        problems.append(f"{phase.get('voltageReadsUnder9V')} hub voltage reads under 9 V")
    return problems


def replaced_reads(phase):
    """Reads through the replaced hub object in an I2C phase that ran them (its replies are routed there on purpose)."""
    if (phase or {}).get("i2cReplacedReader") is not True:
        return None
    return sum(r.get("reads") or 0 for r in phase.get("i2cReaders") or [] if r.get("client") == "replaced")


def leftover_problems(left, when):
    """Problems in a {hub: PhotonProbe.leftovers} map: every count must be 0."""
    if not isinstance(left, dict) or not left:
        return [f"no leftover report {when}"]
    bad = {hub: {k: v for k, v in (m or {}).items() if k in PHOTON_LEFTOVERS and v != 0} for hub, m in left.items()}
    missing = [hub for hub, m in left.items() if not all(k in (m or {}) for k in PHOTON_LEFTOVERS)]
    out = [f"{hub} left {b} {when}" for hub, b in bad.items() if b]
    return out + [f"{hub}: incomplete leftover report {when}" for hub in missing]


def photon_workload_checks(section, label, phases):
    out = []
    for name, phase in phases.items():
        problems = photon_phase_problems(phase)
        rb = (phase or {}).get("readback") or {}
        c = (phase or {}).get("counters") or {}
        detail = (f"{rb.get('checks')} read-backs, {rb.get('mismatches')} wrong; drain {fmt_ms(dig(phase, 'drainAfterMs'))}; "
                  f"max in flight {dig(phase, 'maxInFlight')}, SDK stand-ins {dig(phase, 'maxStandIns')}; oldest "
                  f"pending {dig(phase, 'maxPendingMs')} ms; parallel writes {c.get('parallelWrites')} of "
                  f"{dig(phase, 'issuedOnParallelHubs')} issued; retransmits {c.get('retransmits')}")
        out.append(PhotonCheck(section, f"{label} {name}".strip(), not problems,
                               detail + ("" if not problems else "; " + "; ".join(problems)),
                               "a write was lost, landed wrong, stayed in flight, went stale, overran the limit or "
                               "skipped Photon in this mode"))
    return out


def fmt_ms(v):
    return f"{v:.2f} ms" if finite(v) else str(v)


def photon_source_sha256(photon_dir):
    """Matches TeamCode/build.gradle's BuildConfig.PHOTON_SOURCE_SHA256."""
    digest = hashlib.sha256()
    for f in sorted(photon_dir.glob("*.java")):
        digest.update(f.name.encode())
        digest.update(f.read_bytes().replace(b"\r\n", b"\n"))
    return digest.hexdigest()


def photon_install_checks(stock, manifest):
    installed = dig(stock, "info", "installedPhotonSourceSha256")
    local = (manifest or {}).get("photonSourceSha256")
    return [PhotonCheck("install", "the hub's installed Photon was built from the photon sources in this checkout",
                        bool(local) and installed == local,
                        f"installed {str(installed)[:16]}, local {str(local)[:16]}",
                        "the hub runs Photon from another full install, so these results don't describe these sources: "
                        "do a full install (Run) and rerun; deploySloth never changes pinned code")]


def photon_stock_checks(stock):
    out = []
    hubs = dig(stock, "info", "hubs", default=[]) or []
    out.append(PhotonCheck("stock", "the stock baseline ran on the SDK's own hub objects",
                           bool(hubs) and not any(h.get("swapped") for h in hubs),
                           f"hubs {[(h.get('address'), h.get('class')) for h in hubs]}",
                           "Photon had already swapped these hubs, so this isn't a stock baseline: the app wasn't "
                           "restarted before it, or a Photon OpMode ran first"))
    refused = dig(stock, "enableRefused", "refused")
    out.append(PhotonCheck("stock", "PhotonCore.enable() throws in an OpMode without @Photon", refused is True,
                           f"refused {refused}: {dig(stock, 'enableRefused', 'message')}",
                           "any OpMode can switch Photon on, so the opt-in isn't enforced"))
    phases = {"timing": dig(stock, "timing", "stock")}
    for key in ("burst", "interleave", "reads"):
        if dig(stock, key) is not None:
            phases[key] = dig(stock, key)
    if dig(stock, "timing", "stock+poller") is not None:
        phases["timing+poller"] = dig(stock, "timing", "stock+poller")
    out += photon_workload_checks("stock", "stock", phases)
    return out


def photon_port_checks(stock):
    probe = dig(stock, "portProbe") or {}
    busy = [p for p in probe.get("ports") or [] if not p.get("empty")]
    return [PhotonCheck("ports", f"no motor port draws over {probe.get('limitAmps')} A at {probe.get('power')} power",
                        probe.get("empty") is True,
                        f"{len(probe.get('ports') or [])} ports probed; drawing current: "
                        f"{[(p.get('channel'), round(p.get('drivenAmps') or 0, 3)) for p in busy] or 'none'}",
                        "something is plugged into a bench port: the Photon benches would drive it; unplug it")]


def photon_gain_rows(stock, main):
    rows = []
    sources = [("stock", dig(stock, "timing", "stock")), ("stock+poller", dig(stock, "timing", "stock+poller"))]
    sources += [(k, v) for k, v in (dig(main, "timing", default={}) or {}).items()]
    base = dig(stock, "timing", "stock", "writeBlockMs", "p50")
    for name, t in sources:
        if not isinstance(t, dict):
            continue
        p50 = dig(t, "writeBlockMs", "p50")
        rows.append({"mode": name, "writeBlock": dig(t, "writeBlockMs"), "perHub": dig(t, "writeBlockPerHubMs") or {},
                     "loopPeriod": dig(t, "loopPeriodMs"),
                     "gain": base / p50 if finite(base) and finite(p50) and p50 > 0 else None})
    return rows


def photon_gain_checks(stock, main, parallel_hubs):
    out = []
    stock_hubs = dig(stock, "timing", "stock", "writeBlockPerHubMs") or {}
    on8 = dig(main, "timing", "on8", "writeBlockPerHubMs") or {}
    off = dig(main, "timing", "off", "writeBlockPerHubMs") or {}
    for hub in parallel_hubs:
        s, o, d = dig(stock_hubs, hub, "p50"), dig(on8, hub, "p50"), dig(off, hub, "p50")
        out.append(PhotonCheck("gain", f"{hub}: writes are faster at 8 parallel than stock", finite(s) and finite(o) and o < s,
                               f"write block p50 stock {s} ms, on8 {o} ms", "Photon gives this hub no speed-up"))
        slack_ratio, slack_ms = PHOTON_OFF_SLACK
        out.append(PhotonCheck("gain", f"{hub}: Photon installed but disabled costs no more than stock",
                               finite(s) and finite(d) and d <= s * slack_ratio + slack_ms,
                               f"write block p50 stock {s} ms, off {d} ms (limit {slack_ratio:.2f}x + {slack_ms} ms)",
                               "swapped-but-disabled hubs are slower than the SDK's own"))
    return out


def output_violates(kind, enabled, raw):
    return bool(enabled) and (kind == "servo" or abs(raw or 0) > PHOTON_POWER_VIOLATION_RAW)


def stop_trial_verdict(sampler, require_zeroing_read=False):
    """Violations in one STOP sampler file: a motor enabled above 1% power, or a servo enabled, in the read of every
    channel that stop() makes after the SDK's fail-safe and before it zeroes the motors, or in any sampler sweep after
    the first sweep, begun with STOP's gate up, in which every channel on that hub read safe (disabled, or a motor at
    1% power or less: zeroMotors re-enables them at 0)."""
    v = {"problems": [], "violations": 0, "examples": [], "failSafeMsAfterGate": {}, "zeroingReadViolations": 0,
         "samplerViolations": 0, "sweepMs": {}, "countersAtStop": None}
    if not isinstance(sampler, dict) or not sampler:
        v["problems"].append("no sampler file")
        return v
    if sampler.get("failure"):
        v["problems"].append(f"the sampler failed: {sampler.get('failure')}")
    if not sampler.get("gateSeen"):
        v["problems"].append("STOP's gate was never seen")
    extra = sampler.get("extra") or {}
    v["countersAtStop"] = extra.get("countersAtStop")
    seen = extra.get("stopSawBeforeZeroing")
    if isinstance(seen, dict) and isinstance(seen.get("outputs"), list) and seen["outputs"]:
        if seen.get("gateUp") is not True:
            v["problems"].append("stop()'s read ran with STOP's gate down")
        for o in seen["outputs"]:
            if output_violates(o.get("kind"), o.get("enabled"), o.get("raw")):
                v["zeroingReadViolations"] += 1
                if len(v["examples"]) < 4:
                    v["examples"].append({"at": "stop() before zeroing", "hub": o.get("hub"), "channel": o.get("channel"),
                                          "raw": o.get("raw")})
    elif require_zeroing_read:
        v["problems"].append("no read of every channel in stop() before zeroing")
    if require_zeroing_read:
        v["problems"] += linear_trial_problems(sampler)
    for hub in sampler.get("hubs") or []:
        key = hub.get("key")
        if hub.get("error"):
            v["problems"].append(f"{key}: sampler thread failed: {hub.get('error')}")
        failed = hub.get("failedSweepsMs") or []
        if failed and dig(sampler, "info", "variant") != "crash":
            v["problems"].append(f"{key}: {len(failed)} sampler sweep(s) had a read fail with no lost write to mark the "
                                 f"hub not responding, first at {failed[0]:.0f} ms")
        kinds, names = hub.get("kinds") or [], hub.get("channels") or []
        sweeps = hub.get("sweeps") or []
        starts = [row[0] for row in sweeps if row]
        gaps = sorted(b - a for a, b in zip(starts, starts[1:]))
        if gaps:
            v["sweepMs"][key] = round(gaps[len(gaps) // 2], 1)
        observed = None
        for row in sweeps:
            if len(row) != 3 + 2 * len(kinds):
                v["problems"].append(f"{key}: malformed sweep {row}")
                break
            enabled, raw = row[3::2], row[4::2]
            if observed is None:
                if row[2] and not any(output_violates(k, e, r) for k, e, r in zip(kinds, enabled, raw)):
                    observed = row[0]
                continue
            for i, kind in enumerate(kinds):
                if output_violates(kind, enabled[i], raw[i]):
                    v["samplerViolations"] += 1
                    if len(v["examples"]) < 4:
                        v["examples"].append({"at": "sampler", "hub": key, "channel": names[i],
                                              "msAfterGate": row[0] - (hub.get("gateMs") or 0), "raw": raw[i]})
        if observed is None:
            v["problems"].append(f"{key}: no sweep after STOP read every channel safe, so the fail-safe was never seen")
        else:
            v["failSafeMsAfterGate"][key] = observed - (hub.get("gateMs") or 0)
    v["violations"] = v["zeroingReadViolations"] + v["samplerViolations"]
    return v


def stop_trial_problems(t, v):
    """A STOP trial's own failures: the sampler's problems, plus a crash, an app restart or an error line."""
    problems = list(v["problems"])
    if t.get("crashed"):
        problems.append(f"the OpMode crashed: {t.get('error')!r}")
    if t.get("appRestarted"):
        problems.append("the app restarted")
    if t.get("error") and not t.get("crashed"):
        problems.append(f"error line {t.get('error')!r}")
    return problems


def photon_stop_bound(n):
    """The 95% upper bound (rule of three) on the per-STOP violation rate after n clean trials."""
    return 3.0 / n if n else 1.0


def photon_stop_checks(condition, trials, expect=None):
    """expect: None for real trials, "held" for the 150 ms negative control (the sampler must see it), "once" for the
    single late write (stop()'s read must see it)."""
    if not trials:
        return [PhotonCheck("stop", f"{condition}: trials ran", False, "no trials", "the STOP trials didn't run")]
    verdicts = [(t, stop_trial_verdict(t.get("sampler"), require_zeroing_read=True)) for t in trials]
    broken = [(t.get("trial"), stop_trial_problems(t, v)) for t, v in verdicts]
    broken = [(tid, p) for tid, p in broken if p]
    sweeps = {}
    for _, v in verdicts:
        for hub, ms in v["sweepMs"].items():
            sweeps.setdefault(hub, []).append(ms)
    sweep_text = ", ".join(f"{hub} {min(ms):.0f}-{max(ms):.0f} ms" for hub, ms in sorted(sweeps.items())) or "none"
    if expect == "held":
        fired = [v for _, v in verdicts if v["samplerViolations"] > 0]
        first = [dig(t, "sampler", "extra", "negativeControl", "samplerSawFailSafeFirst") for t, _ in verdicts]
        return [PhotonCheck("stop", f"{condition}: the sampler sees a motor re-energized for 150 ms after the fail-safe",
                            len(fired) == len(trials) and all(first) and not broken,
                            f"{len(fired)} of {len(trials)} trials registered violations; sampler saw the fail-safe "
                            f"first in {sum(1 for f in first if f)}; sweep period {sweep_text}"
                            + (f"; broken trials {broken[:2]}" if broken else ""),
                            "the STOP sampler can't see a motor re-energized after the fail-safe, so a zero-violation "
                            "result elsewhere proves nothing")]
    if expect == "once":
        fired = [v for _, v in verdicts if v["zeroingReadViolations"] > 0]
        sampled = [v for _, v in verdicts if v["samplerViolations"] > 0]
        return [PhotonCheck("stop", f"{condition}: one late write and enable after the fail-safe is caught every time",
                            len(fired) == len(trials) and not broken,
                            f"stop()'s read caught {len(fired)} of {len(trials)}, the sampler too in {len(sampled)}; "
                            f"sweep period {sweep_text}" + (f"; broken trials {broken[:2]}" if broken else ""),
                            "stop()'s read after the fail-safe can't see a single late re-energize, which is how a "
                            "real Photon STOP failure would look")]
    bad = [(t.get("trial"), v, stop_trial_problems(t, v)) for t, v in verdicts]
    bad = [(tid, v, p) for tid, v, p in bad if v["violations"] or p]
    total = sum(v["violations"] for _, v in verdicts)
    fs = [x for _, v in verdicts for x in v["failSafeMsAfterGate"].values()]
    windows = sum(1 for _, v in verdicts if (v["countersAtStop"] or {}).get("postStopFailSafes")
                  or (v["countersAtStop"] or {}).get("gateFallbacks"))
    detail = (f"{len(trials)} trials, {total} violations, {len(bad)} bad trials, 95% bound on the violation rate "
              f"{photon_stop_bound(len(trials)):.1%}"
              + (f", e.g. trial {bad[0][0]}: {bad[0][2][:2]} {bad[0][1]['examples'][:2]}" if bad else "")
              + (f"; fail-safe seen {min(fs):.0f}-{max(fs):.0f} ms after the gate" if fs else "")
              + f"; {windows} trials had a write in STOP's window (gate fallback or post-STOP fail-safe); sweep "
              f"period {sweep_text}")
    return [PhotonCheck("stop", f"{condition}: nothing re-energized after STOP's fail-safe", not bad, detail,
                        "a write reached a hub after STOP's fail-safe and powered a motor or enabled a servo again, "
                        "or a trial crashed or restarted the app (or couldn't be judged: see the problems)")]


def photon_stop_coverage(condition, trials):
    fractions = [dig(t, "sampler", "loopAtGate", "fractionOfLoop") for t in trials]
    fractions = [f for f in fractions if finite(f)]
    quarters = sorted({min(3, int(f * 4)) for f in fractions})
    return PhotonCheck("stop", f"{condition}: STOP landed in every quarter of the loop", len(quarters) == 4,
                       f"quarters hit {quarters} from {len(fractions)} trials with a known loop phase",
                       "too few trials, or STOP always landed at the same point of the loop, so a race in another "
                       "part of the loop could hide")


def photon_race_checks(trials):
    out = []
    for parallel in sorted({dig(t, "result", "photon") for t in trials if dig(t, "result", "photon") is not None}):
        mine = [t for t in trials if dig(t, "result", "photon") == parallel]
        fired = [t for t in mine if dig(t, "result", "gateRaisedInsideWrite") is True]
        bad = [t for t in fired if dig(t, "result", "violation") is not False]
        label = "Photon on" if parallel > 0 else "Photon off (SDK path)"
        out.append(PhotonCheck("race", f"{label}: the motor stays off when STOP lands inside its write",
                               bool(mine) and len(fired) == len(mine) and not bad,
                               f"{len(mine)} trials, the tap fired in {len(fired)}, violations in {len(bad)}"
                               + (f": {dig(bad[0], 'result', 'hub')} reads {dig(bad[0], 'result', 'reads')}" if bad else ""),
                               "STOP landing just after a write went out let the motor be re-enabled with that power"
                               if fired else "the tap never saw the write, so the trial didn't test the window"))
    return out


def photon_lost_checks(faults, hubs=None, trials=None):
    out = []
    lost, over = dig(faults, "lostWrite", default=[]) or [], dig(faults, "lostOverLimit", default=[]) or []
    if hubs is not None:
        want = len(hubs) * 2 * trials
        out.append(PhotonCheck("lost", "every lost-write trial ran", len(lost) == want and len(over) == len(hubs),
                               f"{len(lost)} of {want} single lost writes, {len(over)} of {len(hubs)} full-limit trials",
                               "Bench: Photon Faults stopped early, so some lost-write paths weren't tested"))
    for r in lost:
        problems = []
        if r.get("primedLanded") is not True:
            problems.append("the priming write didn't land")
        if r.get("registerKeptOldValue") is not True:
            problems.append("the tap didn't stop the write (the register changed), so the trial is invalid")
        if not finite(r.get("writeCallMs")) or r["writeCallMs"] > PHOTON_WRITE_CALL_LIMIT_MS:
            problems.append(f"the lost write's call took {r.get('writeCallMs')} ms")
        lo, hi = PHOTON_ABANDON_WINDOW_MS
        if not finite(r.get("abandonedAtMs")) or not lo <= r["abandonedAtMs"] <= hi:
            problems.append(f"given up at {r.get('abandonedAtMs')} ms, not {lo}-{hi}")
        retx = [e.get("msAfterArming") for e in r.get("tapLog") or [] if e.get("retransmit")]
        if len(retx) != len(PHOTON_RETRANSMIT_WINDOWS_MS) or not all(
                finite(t) and a <= t <= b for t, (a, b) in zip(retx, PHOTON_RETRANSMIT_WINDOWS_MS)):
            problems.append(f"retransmits at {retx} ms, not one in each of {PHOTON_RETRANSMIT_WINDOWS_MS}")
        if (r.get("inFlightAfterAbandon") != 0 or r.get("pendingEntriesAfterAbandon") != 0
                or r.get("standInsAfterAbandon") != 0):
            problems.append(f"in flight {r.get('inFlightAfterAbandon')}, pending {r.get('pendingEntriesAfterAbandon')}, "
                            f"SDK stand-ins {r.get('standInsAfterAbandon')} after giving up")
        if r.get("sdkCacheValidAfterAbandon") is not False:
            problems.append("the SDK's cache for the channel was still valid")
        if not finite(r.get("rewriteMsAfterLostWrite")) or r["rewriteMsAfterLostWrite"] > PHOTON_SDK_CACHE_MS:
            problems.append(f"the identical re-write came {r.get('rewriteMsAfterLostWrite')} ms after the lost write, "
                            "past the SDK's 500 ms cache, so it can't show the cache was dropped")
        if r.get("rewriteLanded") is not True:
            problems.append("the identical re-write never reached the hub")
        if r.get("otherHubAllAt") is False:
            problems.append("the other hub's writes went wrong meanwhile")
        if not finite(dig(r, "otherWriteMs", "max", default=0)) or dig(r, "otherWriteMs", "max", default=0) > PHOTON_WRITE_CALL_LIMIT_MS:
            problems.append(f"other writes meanwhile took up to {dig(r, 'otherWriteMs', 'max')} ms")
        c = r.get("counters") or {}
        if c.get("abandonedWrites") != 1:
            problems.append(f"abandonedWrites {c.get('abandonedWrites')}, not 1")
        for key in ("writeFailures", "deviceNotReadyFallbacks", "slotTimeouts", "orphanDatagrams", "numberClashes",
                    "gateFallbacks"):
            if c.get(key, 0):
                problems.append(f"{key} {c.get(key)}")
        if c.get("nacks"):
            problems.append(f"NACKs {c.get('nacks')}")
        if not finite(r.get("drainAfterMs")) or r.get("notRespondingAtEnd") is not False:
            problems.append(f"drain {r.get('drainAfterMs')}, not responding at the end {r.get('notRespondingAtEnd')}")
        out.append(PhotonCheck("lost", f"{r.get('hub')} {r.get('kind')} {r.get('channel')}: a lost write is handled like "
                               "the SDK's timeout", not problems,
                               f"call {fmt_ms(r.get('writeCallMs'))}, retransmits {retx} ms, given up "
                               f"{fmt_ms(r.get('abandonedAtMs'))}, re-write landed {r.get('rewriteLanded')}"
                               + ("" if not problems else "; " + "; ".join(problems)),
                               "a write whose ACK never came could block the caller, stay in flight, or be followed by an "
                               "identical write the SDK silently skipped"))
    for r in over:
        problems = []
        limit = r.get("limit")
        if r.get("inFlightWithAllLost") != limit:
            problems.append(f"{r.get('inFlightWithAllLost')} in flight with {limit} lost")
        over = r.get("overflowWriteMs") or {}
        if over.get("count") in (None, 0) or not finite(over.get("max")) or over["max"] > PHOTON_WRITE_CALL_LIMIT_MS:
            problems.append(f"the writes past the limit took up to {over.get('max')} ms")
        if r.get("slotTimeoutsSoFar") != over.get("count"):
            problems.append(f"{r.get('slotTimeoutsSoFar')} slot timeouts for {over.get('count')} writes past the limit")
        if r.get("registersKeptOldValues") is not True:
            problems.append("the tap didn't stop the lost writes, so the trial is invalid")
        lo, hi = PHOTON_ABANDON_WINDOW_MS
        if not finite(r.get("allAbandonedAtMs")) or not lo <= r["allAbandonedAtMs"] <= hi + 20:
            problems.append(f"all given up at {r.get('allAbandonedAtMs')} ms")
        if r.get("inFlightAfterAbandon") != 0 or r.get("standInsAfterAbandon") != 0:
            problems.append(f"{r.get('inFlightAfterAbandon')} still in flight, {r.get('standInsAfterAbandon')} SDK stand-ins")
        if r.get("rewritesLanded") is not True:
            problems.append("the identical re-writes didn't all land")
        if dig(r, "counters", "abandonedWrites") != limit:
            problems.append(f"abandonedWrites {dig(r, 'counters', 'abandonedWrites')}, not {limit}")
        out.append(PhotonCheck("lost", f"{r.get('hub')}: a full limit of lost writes never blocks the next write",
                               not problems, f"limit {limit}, writes past it up to {over.get('max')} ms, all given up at "
                               f"{fmt_ms(r.get('allAbandonedAtMs'))}" + ("" if not problems else "; " + "; ".join(problems)),
                               "lost writes can fill the slots and hang or stall the writer, which is the failure that "
                               "froze stock Photon"))
    return out


def retransmit_problems(tap_log):
    retx = [e.get("msAfterArming") for e in tap_log or [] if e.get("retransmit")]
    if len(retx) != len(PHOTON_RETRANSMIT_WINDOWS_MS) or not all(
            finite(t) and a <= t <= b for t, (a, b) in zip(retx, PHOTON_RETRANSMIT_WINDOWS_MS)):
        return [f"retransmits at {retx} ms, not one in each of {PHOTON_RETRANSMIT_WINDOWS_MS}"]
    return []


def lost_depth_problems(r):
    problems = []
    if r.get("registerKeptOldValue") is not True:
        problems.append("the tap didn't stop the write, so the trial is invalid")
    if not finite(r.get("writeCallMs")) or r["writeCallMs"] > PHOTON_WRITE_CALL_LIMIT_MS:
        problems.append(f"the lost write's call took {r.get('writeCallMs')} ms")
    lo, hi = PHOTON_ABANDON_WINDOW_MS
    if not finite(r.get("abandonedAtMs")) or not lo <= r["abandonedAtMs"] <= hi:
        problems.append(f"given up at {r.get('abandonedAtMs')} ms, not {lo}-{hi}")
    problems += retransmit_problems(r.get("tapLog"))
    busy = dig(r, "busyWriteMs", "max", default=None)
    if busy is not None and (not finite(busy) or busy > PHOTON_WRITE_CALL_LIMIT_MS):
        problems.append(f"writes around it took up to {busy} ms")
    problems += leftover_problems({r.get("hub"): r.get("leftoversAfterAbandon")}, "after giving up")
    if r.get("rewriteLanded") is not True:
        problems.append("the identical re-write never reached the hub")
    if r.get("othersLanded") is not True:
        problems.append("a write to another channel around it didn't land")
    c = r.get("counters") or {}
    if c.get("abandonedWrites") != 1 or c.get("supersededWrites", 0) != 0:
        problems.append(f"abandoned {c.get('abandonedWrites')}, superseded {c.get('supersededWrites')}; want 1 and 0")
    for key in ("writeFailures", "deviceNotReadyFallbacks", "orphanDatagrams", "numberClashes", "gateFallbacks"):
        if c.get(key, 0):
            problems.append(f"{key} {c.get(key)}")
    if c.get("nacks"):
        problems.append(f"NACKs {c.get('nacks')}")
    if not finite(r.get("drainAfterMs")) or r.get("notRespondingAtEnd") is not False:
        problems.append(f"drain {r.get('drainAfterMs')}, not responding at the end {r.get('notRespondingAtEnd')}")
    return problems


def photon_lost_depth_checks(faults, hubs, reps, depths=8):
    out = []
    trials = dig(faults, "lostAmongWrites", default=[]) or []
    for hub in hubs:
        mine = [r for r in trials if r.get("hub") == hub]
        by_depth = collections.Counter(r.get("depth") for r in mine)
        short = {d: by_depth.get(d, 0) for d in range(depths) if by_depth.get(d, 0) < reps}
        bad = [(r.get("depth"), r.get("channel"), p) for r in mine for p in [lost_depth_problems(r)] if p]
        measured = collections.Counter(r.get("inFlightBeforeLostWrite") for r in mine)
        kinds = collections.Counter(r.get("kind") for r in mine)
        out.append(PhotonCheck("lost", f"{hub}: a lost write with 0-{depths - 1} other writes in flight is handled like "
                               "the SDK's timeout", bool(mine) and not short and not bad,
                               f"{len(mine)} lost writes ({dict(kinds)}), {reps} or more at each depth 0-{depths - 1}: "
                               f"{'yes' if not short else f'short {short}'}; measured in flight before each "
                               f"{dict(sorted(measured.items(), key=lambda kv: (kv[0] is None, kv[0])))}"
                               + (f"; {len(bad)} failed, e.g. {bad[:2]}" if bad else ""),
                               "a lost write mishandled only when other writes are in flight around it: a slot, an "
                               "SDK entry or a newer write's ACK confused the give-up"))
    return out


def superseded_problems(r):
    problems = []
    if r.get("registerKeptOldValue") is not True:
        problems.append("the tap didn't stop the write, so the trial is invalid")
    at = r.get("newerWriteAtMs")
    if not finite(at) or at >= PHOTON_RETRANSMIT_WINDOWS_MS[0][0]:
        problems.append(f"the newer write went out at {at} ms, after the first retransmit was due")
    log = r.get("tapLog") or []
    if len(log) != 1 or log[0].get("retransmit"):
        problems.append(f"the tap saw {len(log)} transmissions of the lost write, so it was retransmitted over the newer one")
    problems += leftover_problems({r.get("hub"): r.get("leftovers")}, "after it ended")
    if r.get("newerLanded") is not True:
        problems.append("the newer write's value isn't in the register")
    if r.get("notResponding") is not False:
        problems.append("the hub was marked not responding")
    c = r.get("counters") or {}
    if c.get("supersededWrites") != 1 or c.get("abandonedWrites", 0) != 0 or c.get("retransmits", 0) != 0:
        problems.append(f"superseded {c.get('supersededWrites')}, abandoned {c.get('abandonedWrites')}, retransmits "
                        f"{c.get('retransmits')}; want 1, 0, 0")
    for key in ("writeFailures", "deviceNotReadyFallbacks", "notRespondingFallbacks", "orphanDatagrams", "numberClashes",
                "gateFallbacks", "slotTimeouts"):
        if c.get(key, 0):
            problems.append(f"{key} {c.get(key)}")
    if c.get("nacks"):
        problems.append(f"NACKs {c.get('nacks')}")
    return problems


def photon_superseded_checks(faults, hubs, reps):
    out = []
    trials = dig(faults, "lostThenSuperseded", default=[]) or []
    for hub in hubs:
        mine = [r for r in trials if r.get("hub") == hub]
        bad = [(r.get("channel"), p) for r in mine for p in [superseded_problems(r)] if p]
        out.append(PhotonCheck("lost", f"{hub}: a lost write followed by a newer write to the same channel ends "
                               "superseded, never given up or re-sent", len(mine) >= reps and not bad,
                               f"{len(mine)} of {reps} trials" + (f"; {len(bad)} failed, e.g. {bad[:2]}" if bad else ""),
                               "a lost write is retransmitted over a newer value, or marks the hub unresponsive while it "
                               "keeps answering"))
    return out


def photon_write_failure_checks(faults, hubs=None):
    out = []
    results = dig(faults, "writeThrows", default=[]) or []
    if hubs is not None and len(results) != len(hubs):
        out.append(PhotonCheck("lost", "the USB write failure trial ran on every parallel hub", False,
                               f"{len(results)} of {len(hubs)}", "Bench: Photon Faults stopped early"))
    for r in results:
        c = r.get("counters") or {}
        problems = []
        if r.get("thrown") != 1:
            problems.append(f"the tap threw {r.get('thrown')} times, so the trial is invalid")
        if c.get("writeFailures") != 1 or c.get("parallelWrites") != 0:
            problems.append(f"writeFailures {c.get('writeFailures')}, parallel writes {c.get('parallelWrites')}")
        if r.get("landed") is not True:
            problems.append("the value never reached the hub by the SDK's path")
        if r.get("inFlightAfter") != 0 or r.get("notResponding") is not False or r.get("shutdownReason") != "NORMAL":
            problems.append(f"in flight {r.get('inFlightAfter')}, not responding {r.get('notResponding')}, USB device "
                            f"{r.get('shutdownReason')}")
        if not finite(r.get("writeCallMs")) or r["writeCallMs"] > PHOTON_WRITE_CALL_LIMIT_MS:
            problems.append(f"the call took {r.get('writeCallMs')} ms")
        for key in ("abandonedWrites", "slotTimeouts", "deviceNotReadyFallbacks", "orphanDatagrams", "numberClashes"):
            if c.get(key, 0):
                problems.append(f"{key} {c.get(key)}")
        out.append(PhotonCheck("lost", f"{r.get('hub')}: a USB write that throws falls back to the SDK's path and lands",
                               not problems, f"call {fmt_ms(r.get('writeCallMs'))}, landed {r.get('landed')}"
                               + ("" if not problems else "; " + "; ".join(problems)),
                               "a failed USB write is swallowed, lost, or takes the link down"))
    return out


def photon_reset_checks(faults, hubs=None):
    out = []
    results = dig(faults, "resetRecovery", default={}) or {}
    if hubs is not None:
        out.append(PhotonCheck("reset", "the reset trial ran on every hub", sorted(results) == sorted(hubs),
                               f"ran on {sorted(results)} of {sorted(hubs)}", "Bench: Photon Faults stopped early"))
    for hub, modes in results.items():
        off, on = modes.get("off") or {}, modes.get("on8") or {}
        problems = []
        for name, m in (("off", off), ("on8", on)):
            if m.get("settledBeforeReset") is not True or m.get("resetSeen") is not True:
                problems.append(f"{name}: settled {m.get('settledBeforeReset')}, reset seen {m.get('resetSeen')}")
            rec = list((m.get("recoveredMs") or {}).values())
            if not rec or not all(finite(x) and x <= PHOTON_RESET_LIMIT_MS for x in rec):
                problems.append(f"{name}: not all back within {PHOTON_RESET_LIMIT_MS} ms: {m.get('recoveredMs')}")
        off_max = max([x for x in (off.get("recoveredMs") or {}).values() if finite(x)], default=None)
        on_max = max([x for x in (on.get("recoveredMs") or {}).values() if finite(x)], default=None)
        if off_max is not None and on_max is not None and on_max > off_max + PHOTON_RESET_MARGIN_MS:
            problems.append(f"slowest channel back after {on_max:.0f} ms with Photon, {off_max:.0f} ms without")
        cache_path = off.get("sdkCacheValidAfterRawFailSafe") is True and on.get("sdkCacheValidAfterRawFailSafe") is True
        how = ("the SDK's caches stayed valid, so this tested recovery by cache expiry and re-send" if cache_path else
               "attention-driven recovery: the hub flagged the reset and the SDK dropped its caches at once, so the "
               "cache-expiry path wasn't exercised")
        out.append(PhotonCheck("reset", f"{hub}: outputs back after a hub reset no later than without Photon "
                               f"(+{PHOTON_RESET_MARGIN_MS} ms, {'cache expiry' if cache_path else 'attention-driven'})",
                               not problems,
                               f"slowest channel off {off_max} ms, on8 {on_max} ms; {how} (caches valid after the raw "
                               f"fail-safe: off {off.get('sdkCacheValidAfterRawFailSafe')}, on8 "
                               f"{on.get('sdkCacheValidAfterRawFailSafe')}; attention in its ACK: off "
                               f"{off.get('attentionInFailSafeAck')}, on8 {on.get('attentionInFailSafeAck')}; first seen "
                               f"off {off.get('attentionSeenMs')} ms, on8 {on.get('attentionSeenMs')} ms)"
                               + ("" if not problems else "; " + "; ".join(problems)),
                               "a hub reset leaves outputs off longer under Photon: the reset's attention bit or the "
                               "re-sends don't get through"))
    return out


def photon_storm_checks(storm):
    problems = photon_phase_problems(storm)
    stall = dig(storm, "maxStallMs")
    if not finite(stall) or stall > PHOTON_STORM_STALL_MS:
        problems.append(f"the writer stalled {stall} ms")
    return [PhotonCheck("storm", f"{dig(storm, 'writes')} writes at a limit of 1 never stall the writer over "
                        f"{PHOTON_STORM_STALL_MS} ms", not problems,
                        f"longest stall {stall} ms, longest write {dig(storm, 'writeMs', 'max')} ms, slot timeouts "
                        f"{dig(storm, 'counters', 'slotTimeouts')}" + ("" if not problems else "; " + "; ".join(problems)),
                        "a full limit makes the writer wait without bound (the stock Photon hang)")]


def photon_thread_checks(two):
    problems = photon_phase_problems(two)
    if (dig(two, "toggles") or 0) < 1 or (dig(two, "servoWrites") or 0) < 1:
        problems.append(f"toggles {dig(two, 'toggles')}, servo writes {dig(two, 'servoWrites')}")
    if not (dig(two, "readback", "checks") or 0) > 0:
        problems.append("no motor read-backs during the phase")
    return [PhotonCheck("threads", "servo writes from a second thread and Photon toggled from a third stay correct",
                        not problems, f"{dig(two, 'motorWrites')} motor and {dig(two, 'servoWrites')} servo writes, "
                        f"{dig(two, 'toggles')} toggles, {dig(two, 'readback', 'checks')} motor read-backs during it, "
                        f"max in flight {dig(two, 'maxInFlight')}" + ("" if not problems else "; " + "; ".join(problems)),
                        "parallel writes aren't safe from more than one thread: keep them on the OpMode thread")]


def photon_soak_checks(soak, target_loss=PHOTON_SOAK_LOSS_TARGET):
    problems = photon_phase_problems(soak)
    issued = dig(soak, "issuedWrites", default={}) or {}
    c = dig(soak, "counters", default={}) or {}
    lost = (c.get("abandonedWrites") or 0) + (c.get("supersededWrites") or 0) + (dig(soak, "readback", "mismatches") or 0)
    bounds = {hub: 3.0 / n for hub, n in issued.items() if isinstance(n, (int, float)) and n > 0}
    need = math.ceil(3.0 / target_loss)
    met = not problems and lost == 0 and bool(bounds) and all(b <= target_loss for b in bounds.values())
    detail = (f"{dig(soak, 'seconds')} s, writes per hub {issued}, lost {lost}, retransmitted {c.get('retransmits')}, "
              f"95% upper bound per write {({h: f'{b:.2e}' for h, b in bounds.items()})} (target {target_loss:.1e}, "
              f"needs {need} writes per hub); oldest-pending samples by age {dig(soak, 'pendingAgeSamples')}")
    return [PhotonCheck("soak", "zero lost, stale or retransmitted writes", not problems and lost == 0,
                        detail + ("" if not problems else "; " + "; ".join(problems)),
                        "writes were lost or went stale at production rate"),
            PhotonCheck("soak", "the loss bound reaches the target", met, detail,
                        "too few writes for the bound (run --photon-soak), or a write was lost")], met


def photon_routing_problems(run, parallel_hubs=None):
    problems = []
    init = dig(run, "atInit") or {}
    if not init:
        return ["no result"]
    if init.get("photonEnabled") is not False:
        problems.append("Photon was enabled at INIT")
    if init.get("swapFailures"):
        problems.append(f"swap failures {init.get('swapFailures')}")
    if init.get("routingProblems"):
        problems.append(f"routing {init.get('routingProblems')}")
    for key in ("inFlight", "pendingEntries", "standIns", "routes"):
        if key not in init:
            problems.append(f"no {key} at INIT")
            continue
        left = {h: n for h, n in (init.get(key) or {}).items() if n}
        if left:
            problems.append(f"{key} at INIT {left}")
    for mode, hubs in (dig(run, "notRespondingRecovery") or {}).items():
        for hub, m in hubs.items():
            if m.get("visible") is not True:
                problems.append(f"{mode} {hub}: noteNotResponding() didn't show")
            if not finite(m.get("clearedMs")) or m["clearedMs"] > PHOTON_CLEAR_LIMIT_MS:
                problems.append(f"{mode} {hub}: still not responding {m.get('clearedMs')} ms after a command")
            if not finite(m.get("voltsAfter")) or m["voltsAfter"] < 9:
                problems.append(f"{mode} {hub}: voltage read {m.get('voltsAfter')} V after")
    if not dig(run, "notRespondingRecovery"):
        problems.append("no not-responding recovery result")
    check = dig(run, "parallelCheck") or {}
    c = check.get("counters") or {}
    if not check:
        problems.append("no parallel check")
    else:
        if c.get("parallelWrites") != check.get("issuedOnParallelHubs"):
            problems.append(f"parallel check: {c.get('parallelWrites')} parallel of {check.get('issuedOnParallelHubs')}")
        if dig(check, "readback", "mismatches") != 0:
            problems.append(f"parallel check read-backs {photon_events(check.get('readback'))}")
        if not finite(check.get("drainMs")) or check["drainMs"] > PHOTON_DRAIN_LIMIT_MS:
            problems.append(f"parallel check drain {check.get('drainMs')}")
        if check.get("routingProblemsAfter"):
            problems.append(f"routing after {check.get('routingProblemsAfter')}")
        problems += [f"parallel check: {p}" for p in leftover_problems(check.get("leftoversAfterDrain"), "after the drain")]
        for key in PHOTON_COUNTERS_ZERO:
            if c.get(key, 0):
                problems.append(f"parallel check {key} {c.get(key)}")
    return problems


def photon_main_init_checks(main):
    info = dig(main, "info") or {}
    problems = []
    if not info:
        problems.append("no result")
    if info.get("routingProblems") != []:
        problems.append(f"routing {info.get('routingProblems')}")
    if info.get("photonSwapFailures") != []:
        problems.append(f"swap failures {info.get('photonSwapFailures')}")
    if info.get("inFlightAtInit") != 0 or info.get("photonEnabledAtInit") is not False:
        problems.append(f"in flight {info.get('inFlightAtInit')}, Photon enabled {info.get('photonEnabledAtInit')} at INIT")
    problems += leftover_problems(info.get("leftoversAtInit"), "at INIT")
    hubs = info.get("hubs") or []
    if not hubs or not all(h.get("swapped") for h in hubs):
        problems.append(f"not every hub swapped: {[(h.get('address'), h.get('class')) for h in hubs]}")
    return [PhotonCheck("routing", "Bench: Photon's INIT: every hub swapped and routed, nothing in flight", not problems,
                        "; ".join(problems) or f"hubs {[h.get('address') for h in hubs]}",
                        "Photon couldn't take over the hubs, or an earlier OpMode left state behind")]


def photon_routing_checks(runs, min_restarts, hub_count):
    out = []
    for label, run in runs:
        problems = photon_routing_problems(run)
        out.append(PhotonCheck("routing", f"{label}: replies routed to Photon's hubs, unresponsive hubs recover", not problems,
                               f"swap order {dig(run, 'atInit', 'swapOrder')}"
                               + ("" if not problems else "; " + "; ".join(problems)),
                               "a hub's replies go to a replaced object, so it can be marked unresponsive for good and "
                               "its reads go blind"))
    orders = [tuple(dig(r, "atInit", "swapOrder", default=[]) or []) for label, r in runs if label.startswith("restart")]
    seen = sorted(set(orders))
    both = hub_count < 2 or len({o[0] for o in seen if o}) >= 2
    out.append(PhotonCheck("routing", f"at least {min_restarts} app restarts and both swap orders seen",
                           len(orders) >= min_restarts and both and hub_count >= 2,
                           f"{len(orders)} restarts, orders {[list(o) for o in orders]}",
                           "only one hub" if hub_count < 2 else
                           "a misroute that depends on which hub Photon swaps first could hide in the order not seen"))
    return out, both and hub_count >= 2


def photon_consecutive_checks(seq):
    out = []
    problems = []
    for label, kind, res in seq:
        if kind == "routing":
            problems += [f"{label}: {p}" for p in photon_routing_problems(res)]
        elif kind == "stock":
            if dig(res, "info", "inFlightAtInit") != 0:
                problems.append(f"{label}: {dig(res, 'info', 'inFlightAtInit')} in flight at INIT")
            problems += [f"{label}: {p}" for p in leftover_problems(dig(res, "info", "leftoversAtInit"), "at INIT")]
            if dig(res, "enableRefused", "refused") is not True:
                problems.append(f"{label}: a non-Photon OpMode could enable Photon")
            if dig(res, "info", "photonEnabledAtInit") is not False:
                problems.append(f"{label}: Photon enabled at INIT")
    routings = [res for _, kind, res in seq if kind == "routing"]
    if len(routings) >= 2:
        first, last = dig(routings[0], "atInit", "counters") or {}, dig(routings[-1], "atInit", "counters") or {}
        for key in ("writeFailures", "deviceNotReadyFallbacks", "orphanDatagrams", "numberClashes", "postStopFailSafes"):
            if isinstance(first.get(key), int) and isinstance(last.get(key), int) and last[key] != first[key]:
                problems.append(f"{key} rose by {last[key] - first[key]} across the sequence")
        before_crash = dig(routings[-2], "atInit", "counters") or {}
        ended = counter_rise(first, before_crash, PHOTON_WRITE_ENDINGS)
        if ended is None:
            problems.append("a routing run's INIT counters are missing")
        elif ended != 0:
            problems.append(f"{ended} writes abandoned, superseded or cleared at INIT before the crash")
        ids = [tuple(h.get("identity") for h in dig(r, "atInit", "loaders", "hubs", default=[]) or []) for r in routings]
        if len(set(ids)) != 1:
            problems.append(f"hub objects changed between INITs without an app restart: {ids}")
    else:
        problems.append("fewer than two Photon INITs in the sequence")
    out.append(PhotonCheck("consecutive", "every INIT in Photon / non-Photon / Photon / crash / Photon starts clean",
                           not problems, f"{len(seq)} OpModes" + ("" if not problems else "; " + "; ".join(problems[:6])),
                           "state from one OpMode (in-flight writes, Photon left on, a broken route) leaks into the next"))
    return out


PHOTON_WRITE_ENDINGS = ("abandonedWrites", "supersededWrites", "clearedAtInit")


def counter_rise(before, after, keys):
    if not all(isinstance(before.get(k), int) and isinstance(after.get(k), int) for k in keys):
        return None
    return sum(after[k] - before[k] for k in keys)


def photon_crash_checks(crash, before=None, after=None):
    """before/after: the routing runs whose INITs bracket the crash."""
    v = stop_trial_verdict(dig(crash, "sampler"))
    problems = list(v["problems"])
    if v["violations"]:
        problems.append(f"{v['violations']} violating samples {v['examples'][:2]}")
    outcome = dig(crash, "outcome") or {}
    if outcome.get("crashed") is not True:
        problems.append("the OpMode didn't crash")
    if outcome.get("appRestarted"):
        problems.append("the app restarted")
    if not (dig(crash, "main", "inFlightAtThrow") or 0) > 0:
        problems.append(f"in flight at the throw {dig(crash, 'main', 'inFlightAtThrow')}, so it didn't test writes in flight")
    lost = dig(crash, "main", "lostAtThrow") or {}
    injected = sum(n for n in lost.values() if isinstance(n, int))
    if not lost or not all(isinstance(n, int) and n > 0 for n in lost.values()):
        problems.append(f"never-acknowledged writes at the throw {lost or 'none'}, so it didn't test a leak")
    leak = []
    if before is not None and after is not None:
        ended = counter_rise(dig(before, "atInit", "counters") or {}, dig(after, "atInit", "counters") or {},
                             PHOTON_WRITE_ENDINGS)
        if ended != injected:
            leak.append(f"{ended} writes abandoned, superseded or cleared at INIT for {injected} never acknowledged")
        leak += [f"next INIT: {p}" for p in photon_routing_problems(after)]
    out = [PhotonCheck("crash", "a crash with parallel writes in flight leaves every output off", not problems,
                       f"in flight at the throw {dig(crash, 'main', 'inFlightAtThrow')}, never to be acknowledged "
                       f"{lost}; fail-safe seen {v['failSafeMsAfterGate']} ms after the gate"
                       + ("" if not problems else "; " + "; ".join(problems)),
                       "a write in flight at a crash landed after the fail-safe")]
    if before is not None and after is not None:
        out.append(PhotonCheck("crash", "every write left unacknowledged by the crash is finished, and the next INIT "
                               "starts with nothing in flight or in the SDK's command map", not leak and injected > 0,
                               f"{injected} never acknowledged; next INIT leftovers: in flight "
                               f"{dig(after, 'atInit', 'inFlight')}, SDK stand-ins {dig(after, 'atInit', 'standIns')}"
                               + ("" if not leak else "; " + "; ".join(leak)),
                               "writes in flight at a crash leak into the next OpMode, holding message numbers or slots"))
    return out


def photon_sloth_checks(before, after, load):
    problems = []
    lb, la = dig(before, "atInit", "loaders") or {}, dig(after, "atInit", "loaders") or {}
    if not lb or not la:
        problems.append("missing a routing run")
    else:
        if lb.get("pid") != la.get("pid"):
            problems.append(f"the app restarted (pid {lb.get('pid')} -> {la.get('pid')}), so this wasn't a hot load")
        if lb.get("photonCoreLoader") != la.get("photonCoreLoader"):
            problems.append(f"PhotonCore came from {la.get('photonCoreLoader')} after the load, {lb.get('photonCoreLoader')} before")
        if lb.get("benchLoader") == la.get("benchLoader"):
            problems.append("the bench class loader didn't change, so the hot load didn't take effect and nothing was tested")
        if la.get("photonCorePinned") is not True or la.get("photonModulePinned") is not True:
            problems.append("PhotonCore or PhotonLynxModule isn't @Pinned at runtime")
    if (load or {}).get("result") != "processed":
        problems.append(f"the load ended {(load or {}).get('result')}")
    problems += [f"after: {p}" for p in photon_routing_problems(after)]
    return [PhotonCheck("sloth", "after deploySloth the same pinned PhotonCore takes the hubs and parallelizes",
                        not problems, f"PhotonCore loader {la.get('photonCoreLoader')}, bench loader "
                        f"{lb.get('benchLoader')} -> {la.get('benchLoader')}" + ("" if not problems else "; " + "; ".join(problems)),
                        "a hot load gives Photon a second copy whose statics the hubs never see")]


def photon_rs485_checks(main, logs):
    """logs: photon_log_counts of Bench: Photon's step."""
    out = []
    phases = dig(main, "rs485SharedLink") or {}
    on = phases.get("on8") or {}
    problems = []
    if not on:
        problems.append("no rs485SharedLink on8 phase")
    for key in ("maxInFlight", "maxPendingEntries", "maxStandIns"):
        if dig(on, key, "expansionHub") != 0:
            problems.append(f"{key} {dig(on, key, 'expansionHub')} on the RS485 hub")
    ch = dig(on, "issuedWrites", "controlHub") or 0
    if not ch or dig(on, "counters", "parallelWrites") != ch:
        problems.append(f"{dig(on, 'counters', 'parallelWrites')} parallel writes for {ch} Control Hub writes, so the "
                        "Control Hub's parallel writes didn't run beside the RS485 hub's")
    out.append(PhotonCheck("rs485", "the RS485 Expansion Hub stays on the SDK's locked path while the Control Hub "
                           "writes in parallel beside it", not problems,
                           f"write block per hub on8 {fmt_cell(dig(on, 'writeBlockPerHubMs', 'expansionHub'))} (RS485) / "
                           f"{fmt_cell(dig(on, 'writeBlockPerHubMs', 'controlHub'))} (Control Hub) vs off "
                           f"{fmt_cell(dig(phases, 'off', 'writeBlockPerHubMs', 'expansionHub'))} / "
                           f"{fmt_cell(dig(phases, 'off', 'writeBlockPerHubMs', 'controlHub'))} ms"
                           + ("" if not problems else "; " + "; ".join(problems)),
                           "Photon parallelized an RS485 child, which shares the Control Hub's link without its lock"))
    out += photon_workload_checks("rs485", "shared link", phases)
    for mode in ("off", "on8"):
        name = f"rs485 shared link {mode}"
        counts = (logs.get("phases") or {}).get(name)
        bad = {k: n for k, n in (counts or {}).items() if k in PHOTON_LOG_FORBIDDEN and n}
        out.append(PhotonCheck("rs485", f"{name}: no lock or abandon warnings with the poller running",
                               counts is not None and not bad,
                               f"{counts if counts is not None else 'no phase marker in the log'}",
                               "commands on the shared RS485 link lost their lock or timed out, or the log was cut"))
    return out


def photon_replug_checks(off, on, mode):
    out = []
    ro, rn = dig(off, "result") or {}, dig(on, "result") or {}
    for label, r in (("off", ro), ("on8", rn)):
        problems = []
        if r.get("faultSeen") is not True:
            problems.append("the bench never saw the Expansion Hub go away")
        if not finite(r.get("recoveryMsAfterBack")) or r["recoveryMsAfterBack"] > PHOTON_REPLUG_RECOVER_MS:
            problems.append(f"writes right again {r.get('recoveryMsAfterBack')} ms after the hub was back")
        if r.get("inFlightAtEnd") != 0 or not finite(r.get("drainMs")):
            problems.append(f"in flight at the end {r.get('inFlightAtEnd')}, drain {r.get('drainMs')}")
        problems += leftover_problems(r.get("leftoversAtEnd"), "at the end")
        if dig(r, "controlHubReadback", "mismatches") != 0:
            problems.append(f"Control Hub read-backs {photon_events(dig(r, 'controlHubReadback'))}")
        if r.get("routingProblemsAfter"):
            problems.append(f"routing after {r.get('routingProblemsAfter')}")
        out.append(PhotonCheck("replug", f"{mode} pull, Photon {label}: writes right again within "
                               f"{PHOTON_REPLUG_RECOVER_MS} ms, Control Hub unaffected", not problems,
                               f"longest write block {r.get('maxWriteStallMs')} ms, longest single write "
                               f"{r.get('maxSingleWriteMs')} ms, back at target {r.get('recoveryMsAfterBack')} ms after "
                               f"the hub returned" + ("" if not problems else "; " + "; ".join(problems)),
                               "a cable bump leaves the Expansion Hub's outputs wrong or upsets the Control Hub"))
    problems = []
    so, sn = ro.get("maxWriteStallMs"), rn.get("maxWriteStallMs")
    if not finite(so) or not finite(sn) or sn > so + PHOTON_REPLUG_MARGIN_MS:
        problems.append(f"longest write block {sn} ms with Photon, {so} ms without")
    single = rn.get("maxSingleWriteMs")
    if not finite(single) or single > PHOTON_REPLUG_SINGLE_WRITE_MS:
        problems.append(f"one write call took {single} ms (limit {PHOTON_REPLUG_SINGLE_WRITE_MS})")
    during = rn.get("countersDuringFault") or {}
    fell_back = {k: during.get(k) for k in PHOTON_EARLY_LOST if during.get(k)}
    if not fell_back:
        problems.append(f"no Expansion Hub write fell back or was given up while it was gone: {during or 'no counters'}")
    out.append(PhotonCheck("replug", f"{mode} pull: Photon stalls the loop no longer than the SDK, and falls back while "
                           "the hub is gone", not problems,
                           f"longest write block off {so} ms, on8 {sn} ms (margin {PHOTON_REPLUG_MARGIN_MS} ms); longest "
                           f"single write on8 {single} ms; while gone {fell_back or during}; first parallel write "
                           f"{rn.get('parallelWriteMsAfterBack')} ms after it was back"
                           + ("" if not problems else "; " + "; ".join(problems)),
                           "Photon makes a cable bump stall the loop longer than the SDK does, or keeps sending "
                           "parallel writes into a hub that isn't there"))
    return out


def photon_log_counts(log_text):
    """The SDK's warning lines per bench phase, from the phase markers BenchReport logs: {"lines": n, "phases":
    {phase: {needle: count}}}, with every phase whose marker is in the log listed (lines before the first: "init")."""
    keys = PHOTON_LOG_NEEDLES + (PHOTON_SLOW_VERSION,)
    phases = {"init": {n: 0 for n in keys}}
    phase = "init"
    lines = 0
    for line in (log_text or "").splitlines():
        lines += 1
        m = PHOTON_PHASE_LINE.search(line)
        if m:
            phase = m.group(1)
            phases.setdefault(phase, {n: 0 for n in keys})
            continue
        for needle in PHOTON_LOG_NEEDLES:
            if needle in line:
                # The SDK's own Bench: Probe, with no Photon, times out on the hub's version string after a restart.
                slow = needle in PHOTON_LOG_FORBIDDEN and any(mark in line for mark in PHOTON_SLOW_VERSION_MARKS)
                phases[phase][PHOTON_SLOW_VERSION if slow else needle] += 1
    return {"lines": lines, "phases": phases}


def photon_log_problems(parsed, expected=(), exempt=(), ignore=()):
    """exempt: phase-name prefixes whose warnings the step causes on purpose; ignore: needles another check judges."""
    if not parsed or not parsed.get("lines"):
        return ["the log is empty"], {}
    phases = parsed.get("phases") or {}
    problems = []
    missing = [p for p in expected if p not in phases]
    if missing:
        problems.append(f"no BENCH phase marker for {missing[:3]}{' ...' if len(missing) > 3 else ''} (the log was cut)")
    bad = {phase: {k: n for k, n in c.items() if k in PHOTON_LOG_FORBIDDEN and k not in ignore and n}
           for phase, c in phases.items() if not any(phase.startswith(e) for e in exempt)}
    bad = {p: c for p, c in bad.items() if c}
    if bad:
        problems.append(f"warnings {bad}")
    retx = {p: c.get("retransmitting") for p, c in phases.items() if c.get("retransmitting")}
    slow = sum(c.get(PHOTON_SLOW_VERSION, 0) for c in phases.values())
    if slow:
        retx["slowVersionStringReplies"] = slow
    return problems, retx


def photon_log_checks(label, entries, exempt=(), ignore=()):
    """One check over the logs of one or more steps: entries are (step, parsed log, phase names the step entered)."""
    bad, retx = [], {}
    for step, parsed, expected in entries:
        problems, r = photon_log_problems(parsed, expected, exempt, ignore)
        if problems:
            bad.append((step, problems))
        if r:
            retx[step] = r
    return [PhotonCheck("logcat", f"{label}: no lock, ownership, checksum, abandon, I2C or placeholder warnings, every log complete",
                        bool(entries) and not bad,
                        f"{len(entries)} step log(s); {len(bad)} with problems"
                        + (f", e.g. {bad[:2]}" if bad else "")
                        + f"; retransmits and slow version-string replies (not judged) {retx or 'none'}"
                        + (f" (not judged: phases starting {list(exempt)})" if exempt else "")
                        + (f" ({list(ignore)} judged by its own check)" if ignore else ""),
                        "the SDK saw lock misuse, a write it had to give up on, or a corrupted reply, or the log that "
                        "would show it is missing")]


def log_ms(line):
    m = LOGCAT_STAMP.match(line or "")
    if not m:
        return None
    mo, d, h, mi, s, ms = (int(x) for x in m.groups())
    return ((((mo * 31 + d) * 24 + h) * 60 + mi) * 60 + s) * 1000 + ms


def stop_lock_events(log_text):
    """Each 'abandoning lock' in a STOP trial's log, in ms after its STOP mark; stopLeak: where a STOP leak would land."""
    stop = None
    events = []
    for line in (log_text or "").splitlines():
        if stop is None and PHOTON_STOP_MARK in line:
            stop = log_ms(line)
            continue
        m = PHOTON_ABANDON.search(line)
        if not m:
            continue
        t = log_ms(line)
        after = None if stop is None or t is None else t - stop
        events.append({"msAfterStop": after, "old": m.group(1), "oldMsg": int(m.group(2)),
                       "stopLeak": after is not None and PHOTON_STOP_LEAK_MS[0] < after <= PHOTON_STOP_LEAK_MS[1]})
    return {"stopMark": stop is not None, "events": events}


def fisher_greater(a, n1, b, n2):
    """One-sided Fisher exact p that a/n1 is above b/n2 by chance alone."""
    k, n = a + b, n1 + n2
    if k == 0:
        return 1.0
    total = math.comb(n, k)
    return sum(math.comb(n1, i) * math.comb(n2, k - i) for i in range(a, min(n1, k) + 1)) / total


def binomial_pmf(n, p, x):
    return math.comb(n, x) * p ** x * (1 - p) ** (n - x)


def fisher_power(n1, n2, base, rise, alpha=PHOTON_LEAK_ALPHA):
    """Chance fisher_greater(a, n1, b, n2) < alpha when a ~ Binomial(n1, rise * base) and b ~ Binomial(n2, base)."""
    p1 = min(1.0, rise * base)
    power = 0.0
    for b in range(n2 + 1):
        pb = binomial_pmf(n2, base, b)
        if pb < 1e-12 and b > n2 * base:
            break
        a = next((a for a in range(n1 + 1) if fisher_greater(a, n1, b, n2) < alpha), None)
        if a is not None:
            power += pb * sum(binomial_pmf(n1, p1, x) for x in range(a, n1 + 1))
    return power


def smallest_detectable_rise(n1, n2, base, alpha=PHOTON_LEAK_ALPHA):
    """The smallest listed rise in the leak rate that fisher_greater catches in at least half of all blocks, or None."""
    for rise in PHOTON_LEAK_RISES:
        if rise * base > 1:
            return None
        if fisher_power(n1, n2, base, rise, alpha) >= 0.5:
            return rise
    return None


def photon_linear_lock_checks(on, off, controls=(), alpha=PHOTON_LEAK_ALPHA):
    """controls: linear negative-control trials, judged for stray abandons but left out of the Fisher comparison."""
    on, off, controls = list(on), list(off), list(controls)
    every = on + off + controls
    stray = [(tid, e) for tid, s in every for e in (s or {}).get("events", []) if not e["stopLeak"]]
    unmarked = [tid for tid, s in every if not (s or {}).get("stopMark")]
    a = sum(1 for _, s in on if any(e["stopLeak"] for e in (s or {}).get("events", [])))
    b = sum(1 for _, s in off if any(e["stopLeak"] for e in (s or {}).get("events", [])))
    p = fisher_greater(a, len(on), b, len(off)) if on and off else 0.0
    owners = sorted({e["old"] for _, s in every for e in (s or {}).get("events", []) if e["stopLeak"]})
    fail_safe = [tid for tid, s in every for e in (s or {}).get("events", []) if e["stopLeak"] and e["old"] == "LynxFailSafeCommand"]
    count = sum(len((s or {}).get("events", [])) for _, s in every)
    base = b / len(off) if off and b else PHOTON_LEAK_BASE_RATE
    rise = smallest_detectable_rise(len(on), len(off), base, alpha) if on and off else None
    power = (f"at these counts and a {base:.1%} base rate it catches a {rise:g}x rise in at least half of all blocks"
             if rise else f"at these counts and a {base:.1%} base rate it can't catch even a "
                          f"{PHOTON_LEAK_RISES[-1]:g}x rise in half of all blocks")
    where = (f"abandoned outside the STOP-interrupt window: {stray[:2]}" if stray else
             f"{count} abandoned lock(s), all {PHOTON_STOP_LEAK_MS[0]}-{PHOTON_STOP_LEAK_MS[1]} ms after STOP")
    if unmarked:
        where += f"; no STOP mark in {unmarked[:3]}"
    return [
        PhotonCheck("logcat", "linear STOP trials: every abandoned network lock is one STOP's interrupt leaked",
                    bool(on) and bool(off) and not stray and not unmarked, where,
                    "a network lock was held 500 ms for some other reason than LinearOpMode's STOP interrupt landing "
                    "between a command's last wait and its release, or a trial's log can't be timed"),
        PhotonCheck("logcat", "linear STOP trials: Photon leaks the SDK's lock at STOP no more often than the SDK alone",
                    bool(on) and bool(off) and p >= alpha and not fail_safe,
                    (f"STOP-interrupt leaks in {a} of {len(on)} Photon on8 trials and {b} of {len(off)} Photon off trials; "
                     f"one-sided Fisher p {p:.3f} (fails below {alpha}); {power}; owners {owners or 'none'}"
                     + (f"; leaked by a fail-safe on the OpMode thread, which only Photon's post-STOP fail-safe sends: "
                        f"{fail_safe[:3]}" if fail_safe else "")
                     if on and off else "the Photon-off linear control didn't run"),
                    "Photon makes the SDK's network lock leak at STOP more often than the SDK's own path does, its own "
                    "post-STOP fail-safe leaked it, or there was no control to compare with"),
    ]


def max_gap_after_gate(hub):
    gate = hub.get("gateMs")
    starts = [row[0] for row in hub.get("sweeps") or [] if row]
    if gate is None or len(starts) < 2:
        return None
    gaps = [b - a for a, b in zip(starts, starts[1:]) if b > gate]
    return max(gaps) if gaps else None


def linear_trial_problems(sampler):
    problems = []
    extra = (sampler or {}).get("extra") or {}
    if ((sampler or {}).get("info") or {}).get("variant") == "linear" and extra.get("interruptedAtStop") is not True:
        problems.append("the OpMode thread never saw LinearOpMode's STOP interrupt")
    if not isinstance(extra.get("countersAtStop"), dict):
        problems.append("no Photon counters at STOP, so a write in STOP's window can't be counted")
    for hub in (sampler or {}).get("hubs") or []:
        gap = max_gap_after_gate(hub)
        if gap is not None and gap > PHOTON_STOP_BLIND_MS:
            problems.append(f"{hub.get('key')}: no sweep for {gap:.0f} ms after STOP")
    return problems


def i2c_hub_plan(topology):
    eh = topology != "none"
    return {"pinpoint": ("expansionHub", PHOTON_EH_PINPOINT_BUS) if eh else ("controlHub", 1),
            "octoquad": ("controlHub", OCTOQUAD_BUS)}


def i2c_reader_problems(readers, swapped, plan, min_reads=PHOTON_I2C_MIN_READS):
    """swapped: every reader but a replaced one should be on Photon's hub objects (True) or the SDK's (False)."""
    if not isinstance(readers, list) or not readers:
        return ["no I2C readers"]
    problems = []
    for r in readers:
        who = f"{r.get('device')} {r.get('client')}"
        if r.get("failure") or r.get("haltProblems"):
            problems.append(f"{who}: {r.get('failure') or r.get('haltProblems')}")
        if not isinstance(r.get("reads"), int) or r["reads"] < min_reads:
            problems.append(f"{who}: {r.get('reads')} reads (at least {min_reads})")
        if r.get("wrong") != 0 or r.get("placeholders") != 0:
            problems.append(f"{who}: {r.get('wrong')} wrong (first {r.get('firstWrong')}, want {r.get('expected')}) and "
                            f"{r.get('placeholders')} all-zero reads of {r.get('reads')}")
        want = False if r.get("client") == "replaced" else swapped
        if r.get("moduleSwapped") is not want:
            problems.append(f"{who}: read through {'Photon' if r.get('moduleSwapped') else 'the SDK'}'s hub object")
        hub, bus = plan.get(r.get("device"), (None, None))
        if r.get("hub") != hub or r.get("bus") != bus:
            problems.append(f"{who}: on {r.get('hub')} bus {r.get('bus')}, not {hub} bus {bus}")
    for device in plan:
        clients = {r.get("client") for r in readers if r.get("device") == device}
        if "driver" not in clients or len(clients) < 2:
            problems.append(f"{device}: readers {sorted(c for c in clients if c)}, not the driver's client and a second one")
    return problems


def i2c_reads_text(readers):
    n = sum(r.get("reads") or 0 for r in readers or [])
    per = ", ".join(f"{r.get('device')} {r.get('client')} {r.get('reads')} reads p95 "
                    f"{fmt_ms(dig(r, 'readMs', 'p95'))}" for r in readers or [])
    return f"{n} reads ({per})"


def i2c_clean_reads(phases):
    return sum(r.get("reads") or 0 for ph in phases for r in (ph or {}).get("i2cReaders") or []
               if r.get("wrong") == 0 and r.get("placeholders") == 0)


def i2c_info_problems(info, swapped, plan):
    problems = []
    devices = {d.get("device"): d for d in (info or {}).get("devices") or []}
    for name, title, address in PHOTON_I2C_DEVICES:
        d = devices.get(name)
        if d is None:
            problems.append(f"no {title}")
            continue
        hub, bus = plan[name]
        if d.get("hub") != hub:
            problems.append(f"{title} planned on {d.get('hub')}, not {hub}")
        for client, where in (("driver", (d.get("clientHub"), d.get("bus"), d.get("i2cAddress"))),
                              ("second", (d.get("secondHub"), d.get("secondBus"), d.get("secondI2cAddress")))):
            if where != (hub, bus, address):
                problems.append(f"{title}'s {client} client on {where[0]} bus {where[1]} at {where[2]}, not {hub} bus {bus} "
                                f"at {address}")
        for key in ("expectedA", "expectedB"):
            v = d.get(key)
            if not isinstance(v, str) or not v or set(v) == {"0"}:
                problems.append(f"{title} {key} {v!r}: a failed read couldn't be told apart")
        if d.get("clientModuleSwapped") is not swapped or d.get("secondModuleSwapped") is not swapped:
            problems.append(f"{title}'s I2C clients on {'the SDK' if swapped else 'Photon'}'s hub objects "
                            f"(driver {d.get('clientModuleSwapped')}, second {d.get('secondModuleSwapped')})")
    if swapped and (info or {}).get("photonSwapFailures") != []:
        problems.append(f"swap failures {(info or {}).get('photonSwapFailures')}")
    return problems


def photon_i2c_checks(stock, main, topology):
    """Everything --photon-i2c adds; a missing result fails, never passes."""
    plan = i2c_hub_plan(topology)
    out = []
    stock_info, main_info = dig(stock, "i2cInfo"), dig(main, "i2cInfo")
    p = i2c_info_problems(stock_info, False, plan)
    out.append(PhotonCheck("i2c", "the stock baseline's Pinpoint and OctoQuad answered at INIT on the SDK's own hub objects",
                           not p, "; ".join(p) or str([(d.get("device"), d.get("hub"), d.get("bus"), d.get("expectedA"),
                                                         d.get("expectedB")) for d in stock_info.get("devices") or []]),
                           "a device isn't plugged in where the block wants it, or isn't answering, or Photon had already "
                           "swapped the stock baseline's hubs"))
    p = i2c_info_problems(main_info, True, plan)
    same = [d.get("device") for d in (dig(main_info, "devices", default=[]) or [])
            for e in (dig(stock_info, "devices", default=[]) or []) if d.get("device") == e.get("device")
            and (d.get("expectedA"), d.get("expectedB")) != (e.get("expectedA"), e.get("expectedB"))]
    if same:
        p.append(f"{same}: constant registers read differently in the stock baseline and after Photon's INIT")
    out.append(PhotonCheck("i2c", "after Photon's INIT every I2C client is Photon's, no swap failed, and the constant "
                                  "registers read as in the stock baseline", not p, "; ".join(p) or "both devices swapped",
                           "Photon left an I2C device on the SDK's hub object, couldn't take over, or a constant register "
                           "read back differently through Photon's hubs"))
    phases = [("stock", dig(stock, "i2c", "stock"), False)]
    phases += [(mode, dig(main, "i2c", mode), True) for mode in PHOTON_I2C_PHASES]
    phases.append(("after the negative control", dig(main, "i2cAfterControl"), True))
    phases.append(("soak", dig(main, "soak"), True))
    for label, ph, swapped in phases:
        problems = i2c_reader_problems((ph or {}).get("i2cReaders"), swapped, plan)
        if label == "off" and (ph or {}).get("i2cReplacedReader") is not True:
            problems.append("the second readers didn't go through the replaced hub objects")
        left = {k: v for k, v in ((ph or {}).get("replacedPendingAfter") or {}).items() if v}
        if left:
            problems.append(f"commands left pending on the replaced hub objects {left}")
        out.append(PhotonCheck("i2c", f"I2C {label}: every read of a constant register is byte-exact, from both threads on "
                                      "every hub", not problems,
                               i2c_reads_text((ph or {}).get("i2cReaders")) + ("; " + "; ".join(problems) if problems else ""),
                               "an I2C read came back wrong or as the SDK's all-zero placeholder, a reader failed or fell "
                               "short, or a device sat on the wrong hub object"))
    control = dig(main, "i2cControl") or {}
    rows, problems = [], []
    for name, title, _ in PHOTON_I2C_DEVICES:
        c = control.get(name) or {}
        locked = c.get("lockedReader") or {}
        if c.get("attempted") != PHOTON_I2C_CONTROL_ATTEMPTS or (c.get("unlockedRight") or 0) + (c.get("unlockedWrong") or 0) \
                + sum((c.get("unlockedRefused") or {}).values()) != c.get("attempted"):
            problems.append(f"{title}: {c.get('attempted')} of {PHOTON_I2C_CONTROL_ATTEMPTS} unlocked transactions accounted for")
        rows.append(f"{title} on {c.get('hub')}: {c.get('attempted')} unlocked, {c.get('unlockedWrong')} wrong, refused "
                    f"{c.get('unlockedRefused') or {}}; the locked reader beside it {locked.get('wrong')} wrong and "
                    f"{locked.get('placeholders')} all-zero of {locked.get('reads')}; {i2c_control_verdict(title, c)}")
    out.append(PhotonCheck("i2c", "negative control: unlocked I2C transactions beside a locked reader ran on every hub",
                           not problems and bool(control), "; ".join(rows) + ("; " + "; ".join(problems) if problems else ""),
                           "the negative control didn't run on every device, so how much the byte compares can see is "
                           "unknown"))
    return out


def i2c_control_caught(c):
    locked = c.get("lockedReader") or {}
    return (c.get("unlockedWrong") or 0) + sum((c.get("unlockedRefused") or {}).values()) \
        + (locked.get("wrong") or 0) + (locked.get("placeholders") or 0)


def i2c_control_verdict(title, c):
    caught = i2c_control_caught(c)
    if caught:
        return f"{title} on {c.get('hub')}: the byte compares saw {caught} anomalies of {c.get('attempted')}"
    return (f"{title} on {c.get('hub')}: 0 of {c.get('attempted')} caught, so the lock check couldn't be shown to fail "
            "on this hub")


def photon_i2c_summary(stock, main):
    phases = [("stock", dig(stock, "i2c", "stock"))] + [(m, dig(main, "i2c", m)) for m in PHOTON_I2C_PHASES]
    phases.append(("after control", dig(main, "i2cAfterControl")))
    phases.append(("soak", dig(main, "soak")))
    lines = ["### I2C reads (constant registers, every 10 ms, while every channel is written), read ms p50 / p95 / max\n",
             "| phase | device | client | hub | reads | wrong | all-zero | read ms |", "|---|---|---|---|---|---|---|---|"]
    for label, ph in phases:
        for r in (ph or {}).get("i2cReaders") or []:
            lines.append(f"| {label} | {r.get('device')} | {r.get('client')} | {r.get('hub')} bus {r.get('bus')} | "
                         f"{r.get('reads')} | {r.get('wrong')} | {r.get('placeholders')} | {fmt_cell(r.get('readMs'))} |")
    clean = i2c_clean_reads([ph for _, ph in phases])
    lines += ["", f"- {clean} clean reads in all: with none wrong, the 95% upper bound on the per-read corruption rate is "
                  f"{photon_stop_bound(clean):.2g} (rule of three)"]
    off = {(r.get("device"), r.get("client") if r.get("client") != "replaced" else "second"): dig(r, "readMs", "p95")
           for r in dig(main, "i2c", "off", "i2cReaders", default=[]) or []}
    ratios = []
    for mode in ("on2", "on8"):
        for r in dig(main, "i2c", mode, "i2cReaders", default=[]) or []:
            base, now = off.get((r.get("device"), r.get("client"))), dig(r, "readMs", "p95")
            if finite(base) and finite(now) and base > 0:
                ratios.append(f"{r.get('device')} {r.get('client')} {mode} {now / base:.2f}x")
    lines.append(f"- read time p95 against Photon off: {', '.join(ratios) or 'not measured'}")
    control = dig(main, "i2cControl") or {}
    for name, title, _ in PHOTON_I2C_DEVICES:
        lines.append(f"- negative control, {i2c_control_verdict(title, control.get(name) or {})}")
    lines.append("")
    return lines


def i2c_control_blind(main):
    control = dig(main, "i2cControl") or {}
    return [i2c_control_verdict(title, control.get(name) or {}) for name, title, _ in PHOTON_I2C_DEVICES
            if not i2c_control_caught(control.get(name) or {})]


def photon_profile_summary(main):
    prof = dig(main, "timing", "on8 profiled", "writeProfile") or {}
    if not prof:
        return []
    hubs = sorted(prof)
    steps = list(next(iter(prof.values())).keys())
    lines = ["### Parallel write path at 8 parallel, per step, p50 / p95 / mean us (timing on8 profiled)\n",
             "| step | " + " | ".join(hubs) + " |", "|---|" + "---|" * len(hubs)]
    for step in steps:
        cells = []
        for h in hubs:
            st = (prof.get(h) or {}).get(step) or {}
            cells.append(f"{st.get('p50Us', '-')} / {st.get('p95Us', '-')} / "
                         f"{round(st['meanUs'], 1) if finite(st.get('meanUs')) else '-'} ({int(st.get('count') or 0)})")
        lines.append(f"| {step} | " + " | ".join(cells) + " |")
    t = dig(main, "timing", "on8 profiled") or {}
    lines += ["", f"- bench side per loop: prepare {fmt_cell(t.get('prepareMs'))} ms, write bookkeeping "
                  f"{fmt_cell(t.get('noteWritesMs'))} ms (both outside the write block)", ""]
    return lines


def overlap_reading(together, ch, eh):
    """12 writes in one loop against each hub's 6 alone: ratio = together / (ch + eh) is 1.0 when the two hubs' times
    add up and max(ch, eh) / (ch + eh) ("full") when they overlap fully; hidden is the share of the shorter hub's time
    the overlap hid, and the reading is the third of that span the ratio falls in. None without three positive times."""
    if not all(finite(v) and v > 0 for v in (together, ch, eh)):
        return None
    full = max(ch, eh) / (ch + eh)
    ratio = together / (ch + eh)
    hidden = (1 - ratio) / (1 - full)
    reading = "adds up" if hidden < 1 / 3 else "overlaps" if hidden > 2 / 3 else "in between"
    return {"together": together, "ch": ch, "eh": eh, "ratio": ratio, "full": full, "hidden": hidden,
            "reading": reading}


def photon_overlap_skip(topology):
    """Why Bench: Photon's two-hubs-at-once phase doesn't run on this wiring; None on USB, where it does."""
    if topology == "rs485":
        return ("the Expansion Hub is on RS485, sharing the Control Hub's link, and Photon never parallelizes its "
                "writes; the phase needs each hub on its own USB link (--usb-expansion)")
    if topology != "usb":
        return "there is no Expansion Hub; the phase needs one on its own USB link (--usb-expansion)"
    return None


def overlap_p50(sets, key, field):
    stats = dig(sets, key, field)
    if not isinstance(stats, dict):
        return None, f"no {key} {field}"
    n = stats.get("count") or 0
    if n < PHOTON_OVERLAP_MIN_LOOPS:
        return None, f"{key} {field}: {n} loops, under {PHOTON_OVERLAP_MIN_LOOPS}"
    if not finite(stats.get("p50")) or stats["p50"] <= 0:
        return None, f"{key} {field} p50 {stats.get('p50')}"
    return stats["p50"], None


def photon_overlap_readings(phase):
    """({"return" | "drained": {order: overlap_reading or None}}, why any reading is missing) for one mode's phase."""
    sets = (phase or {}).get("sets") or {}
    readings, problems = {}, []
    for metric, field in (("return", "returnMs"), ("drained", "drainedMs")):
        ch, ch_problem = overlap_p50(sets, "6 + 0", field)
        eh, eh_problem = overlap_p50(sets, "0 + 6", field)
        readings[metric] = {}
        for key, order in PHOTON_OVERLAP_ORDERS:
            together, problem = overlap_p50(sets, key, field)
            problems += [p for p in (ch_problem, eh_problem, problem) if p and p not in problems]
            readings[metric][order] = overlap_reading(together, ch, eh)
    return readings, problems


def overlap_mode_words(mode):
    return "Photon off" if mode == "off" else f"{mode[2:]} parallel"


def photon_overlap_verdict(main, topology, mode=PHOTON_OVERLAP_VERDICT_MODE):
    """The record-only line for summary.md, decided on the drained time of whichever 12-write order hid more:
    {"reading": "adds up" | "overlaps" | "in between" | None, "line": ...}. Never a check, never in the Photon
    verdict."""
    head = "- **Two hubs at once** (record only: not a check, not in the Photon verdict): "
    out = {"mode": mode, "reading": None}
    skip = photon_overlap_skip(topology)
    if skip:
        return dict(out, line=head + f"not run: {skip}.")
    phase = dig(main, "hubOverlap", mode)
    if not isinstance(phase, dict):
        return dict(out, line=head + f"not measured: Bench: Photon has no hub overlap {mode} result (it stopped before "
                                     "that phase, or the hub runs a bench build without it).")
    readings, problems = photon_overlap_readings(phase)
    drained = {order: r for order, r in readings["drained"].items() if r}
    if not drained:
        return dict(out, line=head + f"not measured at {overlap_mode_words(mode)}: {'; '.join(problems)}.")
    order, best = min(drained.items(), key=lambda kv: kv[1]["ratio"])

    def ratios(metric):
        return ", ".join(f"{r['ratio']:.2f} {o}" if r else f"n/a {o}" for o, r in readings[metric].items())

    line = (head + f"**{best['reading']}** at {overlap_mode_words(mode)}. Drained, 12 together / (6 + 6 alone) = "
            f"{ratios('drained')}; until the calls return, {ratios('return')}. 1.00 means the hubs' times add up, "
            f"{best['full']:.2f} that they overlap fully (drained p50: 6 on the Control Hub {best['ch']:.2f} ms, 6 on "
            f"the Expansion Hub {best['eh']:.2f} ms, 12 {order} {best['together']:.2f} ms; {best['hidden']:.0%} of the "
            "shorter hub's time hidden).")
    if problems:
        line += f" Not read: {'; '.join(problems)}."
    return dict(out, reading=best["reading"], order=order, ratio=best["ratio"], full=best["full"],
                readings=readings, line=line)


def p50_p95(stats):
    if not counted(stats):
        return "n/a"
    return f"{stats['p50']:.2f} / {stats['p95']:.2f}"


def photon_overlap_summary(main, topology):
    """summary.md's section for Bench: Photon's two-hubs-at-once phase (USB only)."""
    lines = ["### Two hubs at once (record only)\n"]
    verdict = photon_overlap_verdict(main, topology)
    if photon_overlap_skip(topology):
        return lines + [verdict["line"], ""]
    phases = {m: dig(main, "hubOverlap", m) for m in PHOTON_OVERLAP_MODES}
    modes = [m for m, ph in phases.items() if isinstance(ph, dict)]
    if modes:
        lines += ["Changed writes in one loop, p50 / p95 ms until the calls return; p50 / p95 ms until every write is "
                  "acknowledged (drained). Each loop is drained before the next, and every set runs once per round in a "
                  "shuffled order. With Photon off each call waits for its ACK, so off should read \"adds up\" whatever "
                  "the hubs do: it is the control.\n",
                  "| set | " + " | ".join(overlap_mode_words(m) for m in modes) + " |", "|---|" + "---|" * len(modes)]
        for key, label in PHOTON_OVERLAP_SETS:
            cells = [f"{p50_p95(dig(phases[m], 'sets', key, 'returnMs'))}; {p50_p95(dig(phases[m], 'sets', key, 'drainedMs'))}"
                     for m in modes]
            lines.append(f"| {label} | " + " | ".join(cells) + " |")
        readings = {m: photon_overlap_readings(phases[m])[0] for m in modes}
        for metric, what in (("return", "returned"), ("drained", "drained")):
            for _, order in PHOTON_OVERLAP_ORDERS:
                cells = []
                for m in modes:
                    r = readings[m][metric][order]
                    cells.append(f"{r['ratio']:.2f} of {r['full']:.2f}-1.00: {r['reading']}" if r else "n/a")
                lines.append(f"| 12 / (6 + 6), {what}, {order} | " + " | ".join(cells) + " |")
        lines += ["", "- loops per set: " + ", ".join(
            f"{overlap_mode_words(m)} {dig(phases[m], 'rounds')}" for m in modes) + "; drains past 500 ms: " + ", ".join(
            f"{overlap_mode_words(m)} {dig(phases[m], 'drainTimeouts')}" for m in modes)]
    return lines + [verdict["line"], ""]


def photon_soak_seconds(args):
    if args.photon_soak:
        return round(args.photon_soak_hours * 3600) if args.photon_soak_hours else PHOTON_SOAK_MAX_S
    return 20 if args.quick else 60


def photon_verdict(checks, negative_control_fired, orders_both, soak_ran, soak_met, stop_trials, not_run=(),
                   still_needed=()):
    """("SAFE" | "PASSED" | "NOT SAFE", blockers, still needed). PASSED: nothing in this run failed, but the go also
    needs what this run didn't include (the soak, the cable pulls, the full STOP trial count)."""
    blockers = [f"{c.section}: {c.name}" for c in checks if not c.ok]
    if not negative_control_fired:
        blockers.append("a STOP negative control didn't register every time, so the STOP checks prove nothing")
    if not orders_both:
        blockers.append("the routing check didn't see both module orders")
    if soak_ran and not soak_met:
        blockers.append("the soak didn't meet its loss bound")
    blockers += [f"not run: {n}" for n in not_run]
    needed = list(still_needed)
    if not soak_ran:
        needed.append("the soak (--photon-soak), which bounds the per-write loss rate")
    if stop_trials < PHOTON_SAFE_STOP_TRIALS:
        needed.append(f"{PHOTON_SAFE_STOP_TRIALS} STOP trials per condition (this run {stop_trials}: the violation rate is "
                      f"bounded only below {photon_stop_bound(stop_trials):.1%}, 95%)")
    if blockers:
        return "NOT SAFE", blockers, needed
    return ("PASSED" if needed else "SAFE"), [], needed


def logcat_pid(line):
    parts = (line or "").split(None, 3)
    return parts[2] if len(parts) > 3 else None


def lines_after(log_text, mark):
    lines = log_text.splitlines()
    start = next((i for i, line in enumerate(lines) if mark in line), None)
    return [] if start is None else lines[start:]


def step_log(log_text, step):
    """The log from the step's begin mark on: logcat -T has whole seconds, so the file can open on the previous step's
    STOP-time crash report. The whole log when the step has no mark."""
    own = lines_after(log_text, f"begin {step}")
    return "\n".join(own) if own else log_text


def error_lines_while(statuses, opmode, t0):
    out = []
    for t, active, error in statuses:
        if active == opmode and (not out or out[-1][1] != error):
            out.append([round(t - t0, 2), error])
    return out


def traces_sections(text):
    heads = list(FREEZE_TRACES_HEADER.finditer(text))
    return [(m.group(1), f"{m.group(3)}-{m.group(4)} {m.group(5)}.000",
             text[m.start():heads[i + 1].start() if i + 1 < len(heads) else len(text)]) for i, m in enumerate(heads)]


def traces_lock_waits(section):
    names, waits, current = {}, [], None
    for line in section.splitlines():
        m = FREEZE_TRACES_THREAD.match(line)
        if m:
            current = (m.group(1), m.group(2))
            names[m.group(2)] = m.group(1)
            continue
        w = FREEZE_TRACES_WAITS.match(line)
        if w and current is not None:
            waits.append((current, w.group(1), w.group(2), w.group(3)))
    return [f"{name} (tid {tid}) waits to lock <{lock}> ({cls}) held by "
            + (f"{names.get(holder, '?')} (tid {holder})" if holder else "a thread ART didn't name")
            for (name, tid), lock, cls, holder in waits]


def shipped_hub_devices(name):
    path = REPO_ROOT / SHIPPED_CONFIG_DIR / f"{name}.xml"
    try:
        root = ElementTree.parse(path).getroot()
    except (OSError, ElementTree.ParseError):
        return None
    children = list(root)
    if len(children) != 1 or children[0].tag != "LynxUsbDevice" or children[0].get("serialNumber") != "(embedded)":
        return None
    modules = list(children[0])
    if len(modules) != 1 or modules[0].get("port") != children[0].get("parentModuleAddress"):
        return None
    devices = list(modules[0])
    if not devices or any(d.get("bus") is not None or not d.get("name") for d in devices):
        return None
    return {d.get("name") for d in devices}


def finite(x):
    return isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)


def wrap_error_deg(a, b):
    return (a - b + 180) % 360 - 180


def circular_span_deg(angles):
    """The shortest arc holding every angle: 360 minus the largest gap between neighbours."""
    a = sorted(x % 360 for x in angles)
    if len(a) < 2:
        return 0.0
    gaps = [a[i + 1] - a[i] for i in range(len(a) - 1)] + [a[0] + 360 - a[-1]]
    return 360 - max(gaps)


def circular_mean_deg(angles):
    s = sum(math.sin(math.radians(x)) for x in angles)
    c = sum(math.cos(math.radians(x)) for x in angles)
    return math.degrees(math.atan2(s, c)) % 360


def tz_answer_ok(answer):
    return isinstance(answer, str) and answer.strip().lower() == "ok"


def tz_volts(answer):
    try:
        return float(str(answer).strip().lower().rstrip("v").strip())
    except ValueError:
        return None


def tz_levels_ok(supply_answer, high_answer):
    """Whether typed meter readings let the absolute output into the OctoQuad's 3.3 V-only input: a 3.3 V supply and a
    positive high level at most TZ_LEVEL_MAX_V (reversed probes read a negative one)."""
    supply, high = tz_volts(supply_answer), tz_volts(high_answer)
    return (supply is not None and high is not None and TZ_SUPPLY_MIN_V <= supply <= TZ_LEVEL_MAX_V
            and 0 < high <= TZ_LEVEL_MAX_V)


def tz_turns_to_mark(rests, key, turns):
    """(counts per turn, counts off the start at the end) for turns to a mark and back: the first rest, the one
    farthest from it and the last; None without a rest after the farthest."""
    values = [r.get(key) for r in rests if finite(r.get(key))]
    if len(values) < 3:
        return None
    start = values[0]
    far = max(range(len(values)), key=lambda i: abs(values[i] - start))
    if far == len(values) - 1:
        return None
    return abs(values[far] - start) / turns, values[-1] - start


def tz_span(rows, version="V1"):
    """Fits the hub count to the unwrapped absolute output over rows [ms, count, channel 4, crcOk, bulkFake]: the pulse
    range, counts per turn (its sign is the count's direction against the absolute angle), turns covered and the worst
    residual in encoder degrees; None with too few good rows or no motion."""
    positions = TZ_VERSIONS[version][2]
    pts = [(r[1], r[2]) for r in rows if len(r) >= 5 and r[3] == 1 and r[4] == 0 and r[2] > 0]
    if len(pts) < 20:
        return None
    pulses = [p for _, p in pts]
    lo, hi = min(pulses), max(pulses)
    if hi <= lo:
        return None
    turn_us = (hi - lo) * positions / (positions - 1)
    turns, k, prev, wraps = [], 0, pulses[0], 0
    for p in pulses:
        if p - prev > turn_us / 2:
            k, wraps = k - 1, wraps + 1
        elif prev - p > turn_us / 2:
            k, wraps = k + 1, wraps + 1
        turns.append((p - lo) / turn_us + k)
        prev = p
    counts = [c for c, _ in pts]
    n = len(pts)
    mt, mc = sum(turns) / n, sum(counts) / n
    var = sum((t - mt) ** 2 for t in turns)
    if var == 0:
        return None
    slope = sum((t - mt) * (c - mc) for t, c in zip(turns, counts)) / var
    residual = max(abs(c - mc - slope * (t - mt)) for t, c in zip(turns, counts))
    return {"minUs": lo, "maxUs": hi, "countsPerTurn": slope, "sign": 1 if slope > 0 else -1,
            "turns": max(turns) - min(turns), "wraps": wraps, "samples": n,
            "maxResidualDeg": residual / abs(slope) * 360 if slope else None}


def tz_failed(run):
    """Why a Bench: Turret Zero run can't be judged, or None."""
    if run.get("crashed"):
        return f"crashed: {run.get('error')!r}"
    if not run.get("result"):
        return f"no result file (error {run.get('error')!r})"
    return None


def turret_zero_checks(tz):
    """(checks, summary lines) for Bench: Turret Zero from the runner's record: the typed answers, and per run its
    outcome and result file. A phase never reached adds a not-run line, a run that failed a FAIL."""
    checks, lines = [], []
    answers, runs = tz.get("answers") or {}, tz.get("runs") or {}

    def add(name, ok, detail, means):
        checks.append((f"turret zero: {name}", bool(ok), detail, means))

    def result(name, label):
        run = runs.get(name)
        if run is None:
            lines.append(f"- {label}: not run" + (f" (the bench stopped: {tz['stopped']})" if tz.get("stopped") else ""))
            return None
        failure = tz_failed(run)
        if failure is not None:
            add(f"{label}: the run finished", False, failure, TZ_NOT_RUN_MEANS)
            return None
        return run["result"]

    if "cable1" in answers or "split" in answers:
        add("the REV-31-1815 and the split cable metered right (typed ok)",
            tz_answer_ok(answers.get("cable1")) and tz_answer_ok(answers.get("split")),
            f"REV-31-1815 {answers.get('cable1')!r}; split cable {answers.get('split')!r}", TZ_WIRING_MEANS)
    if "supplyV" in answers or "highV" in answers:
        add(f"the encoder's supply {TZ_SUPPLY_MIN_V:.2f}-{TZ_LEVEL_MAX_V:.2f} V and its absolute output's high level "
            f"above 0 and at most {TZ_LEVEL_MAX_V:.2f} V, OctoQuad branch unplugged",
            tz_levels_ok(answers.get("supplyV"), answers.get("highV")),
            f"supply {answers.get('supplyV')!r} V, high level {answers.get('highV')!r} V", TZ_LEVEL_MEANS)

    def turns(name, label, key, n):
        res = result(name, label)
        if res is None:
            return None
        rests = dig(res, "track", "rests") or []
        got = tz_turns_to_mark(rests, key, n)
        add(f"{label}: {n} turns to a dial mark and back count {TZ_COUNTS_PER_TURN} +/- {TZ_PER_TURN_COUNTS} per turn and "
            f"end within {TZ_PER_TURN_COUNTS * n} counts of the start",
            got is not None and abs(got[0] - TZ_COUNTS_PER_TURN) <= TZ_PER_TURN_COUNTS
            and abs(got[1]) <= TZ_PER_TURN_COUNTS * n,
            (f"{got[0]:.1f} counts per turn, {got[1]:+.0f} counts off the start at the end" if got else
             "fewer than three rests, or none after the farthest one: the shaft wasn't held on the marks")
            + f"; rests {[r.get(key) for r in rests]}", TZ_TURNS_MEANS)
        return got

    turns("quad", "OctoQuad channel 4 counting (REV-31-1815)", "median", TZ_QUAD_TURNS)
    plugged = turns("split_plugged", "hub encoder port 2, OctoQuad branch plugged", "count", TZ_SPLIT_TURNS)
    unplugged = turns("split_unplugged", "hub encoder port 2, OctoQuad branch unplugged", "count", TZ_SPLIT_TURNS)
    if plugged and unplugged:
        add(f"the hub counts within {TZ_PER_TURN_COUNTS} per turn of each other with the OctoQuad branch plugged and "
            "unplugged", abs(plugged[0] - unplugged[0]) <= TZ_PER_TURN_COUNTS,
            f"plugged {plugged[0]:.1f}, unplugged {unplugged[0]:.1f} counts per turn", TZ_LOAD_MEANS)

    res = result("span", "span")
    if res is not None:
        span = tz_span(dig(res, "track", "rows") or [])
        lo, hi, _ = TZ_VERSIONS["V1"]
        add(f"span: the V1's pulse range {lo:g}-{hi:g} us within {TZ_PULSE_SLACK_US:g} us, {TZ_COUNTS_PER_TURN} +/- "
            f"{TZ_PER_TURN_COUNTS} hub counts per turn of the absolute output, at least {TZ_MIN_TURNS:g} turns",
            span is not None and abs(span["minUs"] - lo) <= TZ_PULSE_SLACK_US
            and abs(span["maxUs"] - hi) <= TZ_PULSE_SLACK_US
            and abs(abs(span["countsPerTurn"]) - TZ_COUNTS_PER_TURN) <= TZ_PER_TURN_COUNTS
            and span["turns"] >= TZ_MIN_TURNS,
            f"{span}" if span else "no motion or too few good rows", TZ_SPAN_MEANS)

    res = result("init_01", "INIT read")
    if res is not None:
        init, bank = res.get("init") or {}, res.get("bank") or {}
        read = init.get("readMs") or {}
        lines.append(f"- INIT read: the first plausible channel 4 pulse {bank.get('firstPlausibleMs')} ms after the bank "
                     f"was set; first reads [ms, pulse, crc] {bank.get('firstReads')}")
        add(f"INIT read: {TZ_INIT_READS} readAllEncoderData at rest, every CRC ok, p95 under {TZ_INIT_P95_MS:g} ms, "
            f"spread at most {TZ_SPREAD_DEG} deg",
            init.get("valid") is True and read.get("count") == TZ_INIT_READS and init.get("crcFailures") == 0
            and read.get("p95", float("inf")) < TZ_INIT_P95_MS and finite(init.get("spreadDeg"))
            and init["spreadDeg"] <= TZ_SPREAD_DEG,
            f"{fmt(read)}, {init.get('crcFailures')} CRC failures, spread {init.get('spreadDeg')} deg over the first 20; "
            f"pulse {init.get('pulseUs')} us", TZ_INIT_MEANS)

    offsets = []
    if "init_01" in runs:
        for i in range(1, TZ_REZERO_RUNS + 1):
            res = result(f"init_{i:02d}", f"re-INIT {i}") if i > 1 else dig(runs, "init_01", "result")
            if res is not None and finite(dig(res, "init", "offsetDeg")):
                offsets.append(res["init"]["offsetDeg"])
        spread = circular_span_deg(offsets)
        add(f"re-zero: {TZ_REZERO_RUNS} INITs with the shaft untouched give zeros within {TZ_SPREAD_DEG} deg",
            len(offsets) == TZ_REZERO_RUNS and spread <= TZ_SPREAD_DEG,
            f"{len(offsets)} valid zeros spanning {spread:.2f} deg: {[round(o, 2) for o in offsets]}", TZ_REZERO_MEANS)
    res = result("init_moved", "INIT after the shaft moved") if "init_01" in runs else None
    if res is not None and offsets:
        moved = res.get("init") or {}
        first = dig(runs, "init_01", "result", "init", "absoluteDeg")
        turned = abs(wrap_error_deg(moved.get("absoluteDeg"), first)) if finite(moved.get("absoluteDeg")) and finite(first) else None
        off = (abs(wrap_error_deg(moved["offsetDeg"], circular_mean_deg(offsets)))
               if finite(moved.get("offsetDeg")) else None)
        add(f"re-zero: after the shaft moved, the zero is within {TZ_MOVED_DEG} deg (the absolute change matches the count "
            "change)", turned is not None and turned >= TZ_MOVED_MIN_DEG and off is not None and off <= TZ_MOVED_DEG,
            f"absolute moved {turned} deg, zero off by {off} deg"
            + (f"; the shaft moved under {TZ_MOVED_MIN_DEG:g} deg, so this proves nothing" if turned is not None
               and turned < TZ_MOVED_MIN_DEG else ""), TZ_REZERO_MEANS)

    res = result("agreement", "agreement")
    if res is not None:
        rests = dig(res, "track", "rests") or []
        good = [r for r in rests if finite(r.get("errorDeg")) and finite(r.get("absoluteDeg"))]
        counts = [r["count"] for r in good]
        travel = (max(counts) - min(counts)) / TZ_COUNTS_PER_TURN if counts else 0.0
        covered = circular_span_deg([r["absoluteDeg"] for r in good])
        coverage = len(good) >= TZ_MIN_RESTS and covered >= TZ_MIN_SPAN_DEG and travel >= TZ_MIN_TURNS
        flipped = max((abs(r["errorFlippedSignDeg"]) for r in good if finite(r.get("errorFlippedSignDeg"))), default=0)
        offset = max((abs(r["error180Deg"]) for r in good if finite(r.get("error180Deg"))), default=0)
        controls = flipped > TZ_CONTROL_DEG and offset > TZ_CONTROL_DEG
        worst = max((abs(r["errorDeg"]) for r in good), default=None)
        add(f"agreement: coverage: at least {TZ_MIN_RESTS} rests with a valid absolute read, {TZ_MIN_SPAN_DEG:g} deg of "
            f"the absolute range, {TZ_MIN_TURNS:g} turns", coverage,
            f"{len(good)} of {len(rests)} rests valid, {covered:.0f} deg covered, {travel:.2f} turns", TZ_COVERAGE_MEANS)
        add(f"agreement: setup: a flipped sign and a 180 deg offset on the same rests err over {TZ_CONTROL_DEG:g} deg",
            controls, f"flipped sign worst {flipped:.1f} deg, 180 deg offset worst {offset:.1f} deg", TZ_CONTROL_MEANS)
        add(f"agreement: every rest's absolute angle valid and within {TZ_AGREE_DEG} deg of the hub count from the INIT "
            "zero", coverage and controls and len(good) == len(rests) and worst is not None and worst <= TZ_AGREE_DEG,
            (f"worst {worst:.2f} deg" if worst is not None else "no valid rest")
            + (f"; {len(rests) - len(good)} rests without a valid absolute read" if len(good) != len(rests) else "")
            + ("" if coverage and controls else "; not judged: the coverage or the negative controls failed")
            + f"; errors {[round(r['errorDeg'], 2) for r in good]}", TZ_AGREE_MEANS)

    res = result("sign", "sign")
    if res is not None:
        sign = res.get("sign") or {}
        fwd, rev = sign.get("FORWARD") or {}, sign.get("REVERSE") or {}
        flips = TZ_SIGN_FLIPS.get(sign.get("orientation"))
        add("sign: the hub's own encoder read equals the bulk read in both directions, from a count of at least 100, "
            "and getCurrentPosition() is minus it exactly where the motor type and direction say",
            bool(fwd) and bool(rev) and flips is not None and all(
                finite(d.get("raw")) and abs(d["raw"]) >= TZ_NONZERO_COUNTS and d.get("raw") == d.get("bulk")
                and d.get("bulkFake") is False and d.get("getCurrentPosition") == flips[k] * d["raw"]
                for k, d in (("FORWARD", fwd), ("REVERSE", rev))),
            f"{sign.get('orientation')} motor type; FORWARD {fwd}; REVERSE {rev}", TZ_SIGN_MEANS)
        if fwd and abs(fwd.get("raw") or 0) >= TZ_NONZERO_COUNTS:
            lines.append(f"- sign: getCurrentPosition() {'negates' if fwd.get('getCurrentPosition') == -fwd.get('raw') else 'does not negate'} "
                         f"the raw count on FORWARD ({sign.get('orientation')} motor type), "
                         f"{'negates' if rev.get('getCurrentPosition') == -rev.get('raw') else 'does not negate'} it on REVERSE")
        else:
            lines.append("- sign: the count was under 100, so whether getCurrentPosition() negates it isn't shown")

    reset, after = result("reset", "mode changes"), result("reset_after", "encoder reset")
    if reset is not None and after is not None:
        r, a = reset.get("reset") or {}, after.get("resetAfter") or {}
        start = r.get("countAtInit")
        seen = [r.get("afterRunWithoutEncoder"), r.get("afterRunUsingEncoder"), r.get("afterBackToRunWithoutEncoder")]
        flat = [c for pair in seen if isinstance(pair, list) for c in pair]
        last = flat[-1] if flat else None
        add(f"mode changes: the count stays within {TZ_STILL_COUNTS} through RUN_WITHOUT_ENCODER, RUN_USING_ENCODER and "
            "into the next OpMode, starting from a count of at least 100",
            finite(start) and abs(start) >= TZ_NONZERO_COUNTS and len(flat) == 6
            and all(abs(c - start) <= TZ_STILL_COUNTS for c in flat) and finite(a.get("countAtInit"))
            and abs(a["countAtInit"] - last) <= TZ_STILL_COUNTS,
            f"at INIT {start}, after each mode change [now, 100 ms later] {seen}, next OpMode's INIT "
            f"{a.get('countAtInit')}", TZ_MODE_MEANS)
        zeroed = a.get("afterStopAndReset")
        lines.append(f"- STOP_AND_RESET_ENCODER on br: count {a.get('countAtInit')} -> {zeroed} [now, 100 ms later], then "
                     f"{a.get('afterRunWithoutEncoder')} after RUN_WITHOUT_ENCODER"
                     + (": it zeroes the turret count, the hazard the turret code must never trigger"
                        if isinstance(zeroed, list) and zeroed and abs(zeroed[-1]) <= TZ_STILL_COUNTS else ""))
        before, back = a.get("absoluteBefore") or {}, a.get("absoluteAfter") or {}
        diff = (abs(wrap_error_deg(back["absoluteDeg"], before["absoluteDeg"]))
                if finite(back.get("absoluteDeg")) and finite(before.get("absoluteDeg")) else None)
        add(f"encoder reset: the absolute read gives the pre-reset angle back within {TZ_RECOVER_DEG} deg",
            diff is not None and diff <= TZ_RECOVER_DEG, f"before {before.get('absoluteDeg')}, after "
            f"{back.get('absoluteDeg')} deg ({diff} deg apart)", TZ_RESET_MEANS)

    if "v2SupplyV" in answers or "v2HighV" in answers:
        add(f"V2 on A: its supply {TZ_SUPPLY_MIN_V:.2f}-{TZ_LEVEL_MAX_V:.2f} V and its absolute output's high level "
            f"above 0 and at most {TZ_LEVEL_MAX_V:.2f} V, OctoQuad branch unplugged",
            tz_levels_ok(answers.get("v2SupplyV"), answers.get("v2HighV")),
            f"supply {answers.get('v2SupplyV')!r} V, high level {answers.get('v2HighV')!r} V", TZ_V2_LEVEL_MEANS)
    if "v2sHighV" in answers:
        add(f"V2 on S: its absolute output's high level above 0 and at most {TZ_LEVEL_MAX_V:.2f} V, OctoQuad branch "
            "unplugged", tz_levels_ok(answers.get("v2SupplyV"), answers.get("v2sHighV")),
            f"high level {answers.get('v2sHighV')!r} V (supply {answers.get('v2SupplyV')!r} V)", TZ_V2_LEVEL_MEANS)
    if tz.get("v2") == "asked":
        res_a, res_s = result("v2_a", "V2 on A"), result("v2_s", "V2 on S")
        lo, hi, _ = TZ_VERSIONS["V2"]
        if res_a is not None:
            got = tz_turns_to_mark(dig(res_a, "track", "rests") or [], "count", TZ_V2_TURNS)
            span = tz_span(dig(res_a, "track", "rows") or [], "V2")
            add(f"V2 on A: one turn to the mark counts {TZ_COUNTS_PER_TURN} +/- {TZ_PER_TURN_COUNTS}, pulse range "
                f"{lo}-{hi} us within {TZ_PULSE_SLACK_US:g} us",
                got is not None and abs(got[0] - TZ_COUNTS_PER_TURN) <= TZ_PER_TURN_COUNTS and span is not None
                and abs(span["minUs"] - lo) <= TZ_PULSE_SLACK_US and abs(span["maxUs"] - hi) <= TZ_PULSE_SLACK_US,
                f"counts per turn {got[0] if got else None}; span {span}", TZ_V2_MEANS)
        if res_s is not None:
            rows = [row for row in dig(res_s, "track", "rows") or [] if len(row) >= 5 and row[4] == 0]
            moved = (max(row[1] for row in rows) - min(row[1] for row in rows)) if rows else None
            pulses = [row[2] for row in rows if row[3] == 1 and row[2] > 0]
            add(f"V2 on S: no counts over a turn (within {TZ_STILL_COUNTS})", moved is not None and moved <= TZ_STILL_COUNTS
                and bool(pulses) and max(pulses) - min(pulses) > (hi - lo) / 2,
                f"count range {moved}; absolute pulses {min(pulses) if pulses else None}-"
                f"{max(pulses) if pulses else None} us (the turn must show on the absolute output)", TZ_V2_MEANS)
    else:
        lines.append(f"- V2: not run ({tz.get('v2') or 'not asked for'})")

    b = tz.get("brownout")
    if b != "ran":
        lines.append(f"- brownout: not run ({b or 'the bench stopped before it'})")
    else:
        before = result("brownout_before", "brownout: INIT read before")
        run = runs.get("brownout")
        lost = tz.get("brownoutError")
        whole = bool(lost) or (run is not None and (run.get("appRestarted") or run.get("pidBefore") != run.get("pidAfter")))
        dip = None if whole else result("brownout", "brownout")
        if whole:
            lines.append("- brownout: not run: whole-hub reset ("
                         + (f"the runner lost the hub: {lost}" if lost else
                            f"app pid {run.get('pidBefore')} -> {run.get('pidAfter')}")
                         + "), so whether a running OpMode reports a lost count can't be judged; only the INIT read "
                         "after it is")
        back = result("brownout_after", "brownout: INIT read after")
        run = run or {}
        if dip is not None:
            track = dip.get("track") or {}
            first, last = track.get("firstCount"), track.get("lastCount")
            changed = finite(first) and finite(last) and abs(last - first) > TZ_STILL_COUNTS
            warnings = (tz.get("brownoutRunner") or {}).get("warnings") or []
            reported = bool(warnings) or bool(track.get("fakeReads")) or bool(track.get("hubEvents")) or bool(run.get("error"))
            add("brownout: a count lost in the dip is reported (warning, error, failed bulk read or hub not responding), "
                "starting from a count of at least 100",
                finite(first) and abs(first) >= TZ_NONZERO_COUNTS and (not changed or reported),
                f"count {first} -> {last}; warnings {warnings}; failed bulk reads {track.get('fakeReads')}; hub events "
                f"{track.get('hubEvents')}; error {run.get('error')!r}", TZ_BROWNOUT_MEANS)
        if before is not None and back is not None:
            x, y = dig(before, "init", "absoluteDeg"), dig(back, "init", "absoluteDeg")
            diff = abs(wrap_error_deg(y, x)) if finite(x) and finite(y) else None
            add(f"brownout: the INIT read after it gives the angle back within {TZ_RECOVER_DEG} deg",
                diff is not None and diff <= TZ_RECOVER_DEG, f"before {x}, after {y} deg ({diff} deg apart)", TZ_RESET_MEANS)
    return checks, lines


def bank_phase_crashed(outcome, res):
    """Bench: OctoQuad crashed in its bank phase, which starts only after the seed phase saved its results."""
    return bool(outcome.get("crashed")) and "bank" in (res.get("phaseLog") or [])


def octoquad_bank_checks(bank, error=""):
    """(checks, summary lines) for Bench: OctoQuad's bank phase; no checks when it never started. Channel 4 carries a
    pod on this bench, so only the timings and the bank read-backs are judged."""
    if not bank:
        return [], ["- not run: the OpMode never got past the seed phase"]
    found, after_set, restored = bank.get("asFound"), bank.get("afterSet"), bank.get("afterRestore")
    local = bank.get("readLocalizerDataAsFound") or {}
    combined = bank.get("readLocalizerDataAndAllEncoderDataAsFound") or {}
    mixed = bank.get("readLocalizerDataMixed") or {}
    init = bank.get("initRead") or {}
    setup, reset = bank.get("afterLocalizerSetup") or {}, bank.get("afterLocalizerReset") or {}
    finished = restored is not None and bool(reset)
    lines = [f"- pods on OctoQuad channels {bank.get('podPorts')}: channel 4 is in the second bank, so while it is in "
             "pulse-width mode its pod reads a pulse width, not a count; only timings and read-backs are judged (on the "
             "robot the pods sit on channels 0 and 1)",
             f"- bank as found {found}; after setChannelBankConfig({OCTOQUAD_MIXED_BANKS}) {after_set}; after "
             f"GuardedOctoQuadLocalizer's setup and reset {setup.get('bank')} (RUNNING after {setup.get('runningMs')} ms); "
             f"after its reset() alone {reset.get('bank')} (RUNNING after {reset.get('runningMs')} ms); restored to "
             f"{restored}",
             f"- readLocalizerData as found {fmt(local)}, {local.get('invalid')} invalid; with bank 2 in pulse-width "
             f"mode {fmt(mixed)}, {mixed.get('invalid')} invalid (p50 {p50_diff(local, mixed)} ms)",
             f"- positive control, readLocalizerDataAndAllEncoderData as found: {fmt(combined)} (p50 "
             f"{p50_diff(local, combined)} ms)",
             f"- the turret's one-off INIT read, readAllEncoderData with bank 2 in pulse-width mode: {fmt(init)}, "
             f"{init.get('crcFailures')} CRC failures; channel 4 (a pod here) read {init.get('channel4')}"]
    if setup or reset:
        lines.append("- the bank " + ("survives" if setup.get("bank") == reset.get("bank") == OCTOQUAD_MIXED_BANKS
                                      else "does NOT survive") + " the localizer's own setup and reset calls (recorded, not judged)")
    checks = []
    if not finished:
        checks.append(("octoquad bank: the phase finished", False, f"stopped with {sorted(bank)}; STOP during it "
                       f"{bank.get('stopRequested')}; error {error!r}", OCTOQUAD_BANK_CRASH_MEANS))
    checks.append(("octoquad bank: the bank found at the start reads back after the phase", restored == found,
                   f"found {found}, read back {restored}", OCTOQUAD_BANK_RESTORE_MEANS))
    banks_ok = found == OCTOQUAD_QUADRATURE and after_set == OCTOQUAD_MIXED_BANKS
    if after_set is not None:
        checks.append((f"octoquad bank: {OCTOQUAD_QUADRATURE} as found, {OCTOQUAD_MIXED_BANKS} read back after the set",
                       banks_ok, f"as found {found}, after the set {after_set}", OCTOQUAD_BANK_SET_MEANS))
    control_ok = None
    if counted(local) and counted(combined):
        gap = combined["p50"] - local["p50"]
        control_ok = gap >= OCTOQUAD_BANK_CONTROL_MS
        checks.append((f"octoquad bank: setup: the timing sees a cost (readLocalizerDataAndAllEncoderData at least "
                       f"{OCTOQUAD_BANK_CONTROL_MS} ms slower at p50 than readLocalizerData)", control_ok,
                       f"{gap:+.2f} ms ({fmt(combined)} against {fmt(local)})", OCTOQUAD_BANK_SETUP_MEANS))
    if counted(mixed):
        if control_ok and banks_ok and counted(local):
            gap = mixed["p50"] - local["p50"]
            checks.append((f"octoquad bank: readLocalizerData p50 with bank 2 in pulse-width mode within "
                           f"{OCTOQUAD_BANK_SAME_MS} ms of every channel counting", abs(gap) <= OCTOQUAD_BANK_SAME_MS,
                           f"{gap:+.2f} ms ({fmt(mixed)} against {fmt(local)})", OCTOQUAD_BANK_COST_MEANS))
        else:
            lines.append("- the cost with bank 2 in pulse-width mode isn't judged: the positive control or the bank "
                         "read-backs failed")
    if init:
        checks.append((f"octoquad bank: the one-off INIT read: {OCTOQUAD_INIT_READS} readAllEncoderData, every CRC ok, "
                       f"p95 under {OCTOQUAD_INIT_READ_P95_MS:.0f} ms",
                       init.get("count") == OCTOQUAD_INIT_READS and init.get("crcFailures") == 0
                       and init.get("p95", float("inf")) < OCTOQUAD_INIT_READ_P95_MS,
                       f"{fmt(init)}, {init.get('crcFailures')} CRC failures", OCTOQUAD_INIT_READ_MEANS))
    return checks, lines


def write_recovery_checks(res, outcome):
    """(name, ok, detail, fail_means) for a Bench: Write Recovery result, with or without Photon."""
    channels, trials, info = res.get("channels") or {}, res.get("trials") or [], res.get("info") or {}
    resend, loop_p95 = info.get("resendMs"), dig(res, "loopPeriodMs", "p95")
    if not channels or not trials:
        return [("bench reported its failSafes", False, f"error {outcome.get('errorMessage')!r}", None)]
    settled = [n for n, c in channels.items() if not c.get("settledBeforeFailSafe")]
    out = [("every channel reached its commanded state before the first failSafe", not settled,
            f"not at target: {settled or 'none'}",
            "the hub didn't take the writes at all (a low battery, or a config problem), so the recovery checks can't "
            "be judged")]
    if settled:
        return out
    phases = [t["phaseMs"] for t in trials if t.get("phaseMs") is not None]
    for wrapped in (False, True):
        for name, c in channels.items():
            if bool(c.get("wrapped")) != wrapped:
                continue
            per = [(t.get("phaseMs"), dig(t, "channels", name, "recoveredMs"), dig(t, "channels", name, "recoveredLoops"))
                   for t in trials]
            back = [ms for _, ms, _ in per if ms is not None]
            never = len(per) - len(back)
            if wrapped:
                label = "EnhancedMotor" if c.get("kind") == "motor" else "EnhancedServo"
                limit = (resend + WRITE_RECOVERY_WRAPPED_LOOPS * loop_p95
                         if resend is not None and loop_p95 is not None else None)
                covered = bool(phases) and loop_p95 is not None and min(phases) <= loop_p95
                worst = max(back) if back else None
                out.append((f"{label} {name} reaches the hub again within WriteCache's re-send period + "
                            f"{WRITE_RECOVERY_WRAPPED_LOOPS} loops of a failSafe, wherever it lands",
                            covered and not never and limit is not None and worst <= limit,
                            f"worst {worst} ms (limit {limit} ms: {resend} + {WRITE_RECOVERY_WRAPPED_LOOPS} x loop p95 "
                            f"{loop_p95}); never back after {never} of {len(per)}; [ms after the write, back after ms] "
                            f"{[(p, ms) for p, ms, _ in per]}"
                            + ("" if covered else "; no failSafe landed within a loop of a write, so the longest wait "
                               "wasn't exercised"), WRITE_CACHE_PENDING))
            else:
                loops = [n for _, _, n in per]
                out.append((f"raw {c.get('kind')} {name} reaches the hub again within {WRITE_RECOVERY_RAW_LOOPS} loops "
                            "of every failSafe (control)",
                            not never and all(n is not None and n <= WRITE_RECOVERY_RAW_LOOPS for n in loops),
                            f"loops to come back per failSafe {loops}, worst {max(back) if back else None} ms; never back "
                            f"after {never} of {len(per)}",
                            "the SDK's own re-send didn't heal a raw device either, so failSafe() did something else on "
                            "this hub and the wrapped results don't say anything about WriteCache"))
    return out


class Runner:
    _link = None

    @property
    def link(self):
        """How the runner reaches the hub's TCP ports: its Wi-Fi address, or with --usb adb forwards over the cable."""
        if self._link is None:
            args = getattr(self, "args", None)
            self._link = (UsbLink(self.adb, self.log) if args is not None and args.usb
                          else WifiLink(HUB_IP if args is None else args.host))
        return self._link

    def __init__(self, args):
        self.args = args
        if args.out:
            self.out = Path(args.out).resolve()
            if (self.out / "raw").exists():
                raise BenchError(f"{self.out} already holds a bench run; pass a new --out folder")
        else:
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
        self.adb = (Adb(None, self.log, usb=True) if args.usb
                    else Adb(args.serial or f"{args.host}:{ADB_PORT}", self.log))
        self.adb.expect_serialno = args.hub_serialno
        self.dash = None
        self.probe = None
        self.original = None
        self.original_xml = None
        self.original_entry = None
        self.original_shipped = False
        self.restore_notes = []
        self.configs_touched = False
        self.read_active_config = None
        self.switch_with_adb = False
        self.switches = []
        self.watch = None
        self.known_pid = None
        self.restarts = []
        self.photon = None
        self.gp_seq = 0
        self.gp_lock = threading.Lock()
        self.thread_method = None
        self.thread_listings = []
        self.dashboard_left_off = None
        self.typed_statements = set()

    def log(self, text):
        line = f"[{datetime.datetime.now().strftime('%H:%M:%S')}] {text}"
        try:
            print(line, flush=True)
        except OSError:
            pass
        self.logfile.write(line + "\n")
        self.logfile.flush()

    def ask(self, text):
        self.log(text)
        input(">>> press Enter: ")

    def tell(self, text):
        """ask() at a terminal; otherwise just the log line, so an unattended run doesn't stop for it."""
        if sys.stdin.isatty():
            self.ask(text)
        else:
            self.log(text)

    def check(self, name, ok, detail, fail_means=None):
        self.checks.append((name, "PASS" if ok else "FAIL", detail, fail_means))
        self.log(f"{'PASS' if ok else 'FAIL'} {name}: {detail}")

    def connect_adb(self):
        """The runner's first adb command comes after the check of adb's server port."""
        self.log(f"results folder: {self.out}")
        server = self.results["adbServer"] = {"atStart": adb_port_preflight()}
        port = server["atStart"]["port"]
        self.log(f"adb's server port {port}: "
                 + (", ".join(f"{o['command']} pid {o['pid']}" for o in server["atStart"]["owners"])
                    or "nothing listening yet, so adb starts its own server"))
        if self.args.usb:
            self.adb.serial = pick_usb_device(self.adb.devices(), self.args.serial)
            self.log(f"--usb: the Control Hub on the cable is adb's {self.adb.serial}; its ports go through adb forwards "
                     "from this Mac, made as they are needed")
        self.adb.ensure_connected()
        server["version"] = (self.adb._raw(["version"]).stdout.strip().splitlines() or [""])[0]
        server["atStart"] = adb_port_preflight()
        self.log(f"{server['version']}; adb server on port {port}: "
                 f"{', '.join(o['command'] + ' pid ' + o['pid'] for o in server['atStart']['owners']) or 'not seen'}")
        if self.args.usb:
            self.check_control_hub()

    def check_control_hub(self):
        """--usb: the cable reaches whatever Android device is on it, so it must be a Control Hub with the RC app."""
        prop = self.adb.run("shell", f"getprop {CONTROL_HUB_PROP}").strip()
        if prop not in PROP_TRUE:
            raise BenchError(f"the USB device {self.adb.serial} isn't a REV Control Hub ({CONTROL_HUB_PROP} is {prop!r}, "
                             "which the FTC SDK reads as false). Nothing was changed on it.")
        if "package:" not in self.adb.run("shell", f"pm path {RC_PACKAGE}", check=False):
            raise BenchError(f"the Control Hub on USB ({self.adb.serial}) has no Robot Controller app ({RC_PACKAGE}). "
                             "Nothing was changed on it.")
        self.log(f"{self.adb.serial} is a Control Hub ({CONTROL_HUB_PROP} {prop}) with the Robot Controller app")

    def connect(self):
        self.connect_adb()
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
        if self.link.usb:
            listeners = self.adb.listeners_or_none(USB_HUB_PORTS)
            reach = self.results["usbHubPorts"] = {p: forward_reach(p, listeners) for p in USB_HUB_PORTS}
            self.log("hub ports a forward reaches, from the hub's socket table: "
                     + "; ".join(f"{p} {'yes' if ok else 'no' if ok is False else '?'} ({why})"
                                 for p, (ok, why) in reach.items()))
        self.dash = Dashboard(self.link, DASHBOARD_PORT, self.log)
        status = self.dash.get_status()
        if status.get("enabled") is False:
            raise BenchError("slothboard reports the dashboard disabled (its saved setting survives app restarts and the "
                             "Run button's reinstall), so it ignores INIT and sends no telemetry. "
                             f"{self.adb_port_caution()} Then turn it back on with: {self.dashboard_recovery()}")
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
        if self.args.original_entry:
            before = json.loads(self.args.original_entry)
            if before != self.original_entry:
                self.log(f"before the install the app's settings named {before}; after it, {self.original_entry}: "
                         "restoring the one from before")
            self.original_entry = before
            active = before.get("name", NO_CONFIG)
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
        if (active != NO_CONFIG and active in self.adb.config_names()
                and self.original_entry.get("location") != "RESOURCE"):
            self.original = active
            self.original_xml = self.adb.read_config(active)
            (self.out / "original_config.xml").write_text(self.original_xml)
            self.log(f"active hardware config: {active} (restored at the end"
                     + (", contents included: the run writes its own bench config of that name)" if active in BENCH_CONFIGS
                        else ")"))
        elif (self.original_entry.get("location") == "RESOURCE"
              or ("location" not in self.original_entry and (REPO_ROOT / SHIPPED_CONFIG_DIR / f"{active}.xml").exists())):
            self.original, self.original_shipped = active, True
            self.log(f"active hardware config: {active}, a res/xml config ({self.original_entry}); re-activated by "
                     "name at the end if this build ships it, else its setting is written back as it was")
            (self.out / "original_config.txt").write_text(f"active config: {json.dumps(self.original_entry)}\n")
        else:
            self.log(f"active hardware config: {active}, which isn't a file in {CONFIG_DIR}; "
                     + (f"its setting {self.original_entry} is written back as it was at the end" if self.switch_with_adb
                        else f"the hub will be left on the {CONFIG_PROBE} config"))
            (self.out / "original_config.txt").write_text(f"active config: {json.dumps(self.original_entry)}\n")

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

    def config_copy(self, name, xml_text):
        n = 1
        while True:
            local = self.out / "logs" / (f"{name}.xml" if n == 1 else f"{name}.{n}.xml")
            if not local.exists() or local.read_text() == xml_text:
                local.write_text(xml_text)
                return local
            n += 1

    def activate_config(self, name, xml_text=None, config=None, accept=None, battery=True):
        """Over adb: write the file, stop the app, point its settings at the file and start it. Through slothboard's
        WRITE/SET_HARDWARE_CONFIG only when run-as can't edit the settings: on slothboard 6165.6 or older that path
        freezes the app until the Control Hub's watchdog relaunches it. Done when the hub logs a robot setup and its settings name the config.
        battery=False for a config that may load no REV hub, where slothboard reports no battery voltage."""
        self.wait_idle()
        if xml_text is None and config is None and name not in self.adb.config_names():
            # slothboard would activate "no config" for a name it doesn't list.
            raise BenchError(f"hardware config {name} isn't in {CONFIG_DIR} on the hub")
        pid = self.adb.pid()
        hub_t0 = self.adb.hub_time()
        self.adb.mark(f"activate config {name}")
        t0 = time.monotonic()
        self.configs_touched = True
        local = self.config_copy(name, xml_text) if xml_text is not None else None
        if self.switch_with_adb:
            self.log(f"{'writing and ' if xml_text is not None else ''}activating hardware config {name} over adb")
            if local is not None:
                self.adb.run("push", str(local), f"{CONFIG_DIR}/{name}.xml")
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
            if active not in (accept or {name}):
                raise BenchError(f"the robot restarted on hardware config {active}, not {name} (logs/{log_name})")
            self.dash.ensure_connected(timeout=60)
            self.wait_ready(battery)
            # The settings file is what the runner wrote; the dashboard's list says what the running app loaded.
            live = None
            for _ in range(5):
                live = self.dash.fresh_active_config()
                if live is not None:
                    break
                time.sleep(2)
            if live is not None and live not in (accept or {name}):
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

    def wait_ready(self, battery=True):
        # A reconnected socket still holds the status from before the switch.
        with self.dash.cond:
            self.dash.status = None
        self.dash.wait_until(lambda: self.dash.status and self.dash.status.get("available")
                             and self.dash.status["activeOpMode"] == IDLE_OPMODE
                             and (not battery or self.dash.status.get("batteryVoltage", -1) >= 0), 90,
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
                    rec = self.dash.stop_recording()
                    outcome["dashboard"] = self._dashboard_stats(rec)
                    outcome["errorLinesWhileActive"] = error_lines_while(rec["statuses"], opmode, t_init)
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
        return f"adb -s {self.adb.serial} shell '{DASHBOARD_RECOVERY_SHELL}'"

    def adb_port_caution(self):
        """Said before every adb command printed for a person to run later, when this run's port check is stale."""
        port = dig(self.results, "adbServer", "atStart", "port") or ADB_SERVER_PORT
        return (f"First check that `lsof -nP -iTCP:{port} -sTCP:LISTEN` shows nothing or adb: anything else on adb's "
                "port takes the command to another hub.")

    def silent_cause(self, hub_t0):
        status = self.dash.status or {}
        if status.get("enabled") is False:
            return (f"slothboard reports the dashboard disabled. {self.adb_port_caution()} Then turn it back on with: "
                    f"{self.dashboard_recovery()}")
        if not self.adb.logcat_matches(hub_t0, REPORT_STARTED).strip():
            return "the OpMode never logged its BenchReport start, so it is stuck before its first packet"
        if not self.dash.recording["messages"]:
            return "not even slothboard's INIT clear reached this socket, so it gets no telemetry at all"
        return ("only slothboard's INIT clear arrived: its telemetry thread has probably died until the app restarts, "
                "which a NaN or infinite field-overlay value in an earlier packet causes only if slothboard 6165.6's "
                "fix has regressed")

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
        log_text = step_log(log_text, step)
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

        def phase_at(phase, t):
            return per_phase.setdefault(str(phase), {"packets": 0, "bytes": 0.0, "first": t, "last": t, "loops": set(),
                                                     "images": 0, "imageBytes": 0})

        for t, phase, loop, size in rec["packets"]:
            p = phase_at(phase, t)
            p["packets"] += 1
            p["bytes"] += size
            p["last"] = t
            if loop is not None:
                p["loops"].add(loop)
        for t, phase, size in rec["images"]:
            p = phase_at(phase, t)
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

    def pull(self, name, required=True, local_name=None):
        local = self.out / "raw" / f"{local_name or name}.json"
        try:
            self.adb.pull_bench_file(f"{name}.json", local)
        except BenchError:
            if required:
                raise
            return None
        return json.loads(local.read_text())

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
                            f"start: {self.original_entry}"
                            + (" (read before the install, so its resourceId is the old build's numbering)\n"
                               if self.args.original_entry else "\n"))
        self.log(f"probe: IMU {self.imu_type}, webcam {self.webcam}, Limelight {self.limelight}, Expansion Hub "
                 f"{'at address ' + str(address) if self.expansion is not None else 'none answering at address ' + str(address)}")
        if stored:
            name, current = self.original, self.probe.get("shippedConfigs", {}).get(self.original)
            target = self.probe.get("originalResolvesTo")
            advice = f"re-activate {name}" if current else f"{name} isn't shipped any more: activate another config"
            when = "before the install" if self.args.original_entry else "on the hub"
            self.check(f"the hub's active config {name} loads res/xml/{name} on this install", current == stored,
                       f"saved {when} as resource {stored:#010x}, which this install resolves to {target}; "
                       f"res/xml/{name} is " + (f"{current:#010x}" if current else "no longer shipped")
                       + ("" if current == stored else f"; {advice}"),
                       f"this install renumbered the shipped configs, so the hub loads {target} under the name {name}; "
                       + CONFIG_BY_NAME_HELD + "; this run's restore re-activates the config by name on this hub only")
            if current != stored:
                self.log(f"!! the hub's active config {name} loads {target} on this install: {advice} on every hub that "
                         "uses it")

    def run_saved_configs(self):
        """Read-only: the hardware configs saved on the hub, the webcam and Ethernet device names in each, and the ones
        this checkout doesn't ship."""
        listing = parse_config_listing(self.adb.run("shell", f"ls -l {CONFIG_DIR}/", check=False))
        configs = {name: self.adb.read_config(name) for name in sorted(self.adb.config_names())}
        name, xml_text, source = active_config_source(self.original_entry, configs, original_xml=self.original_xml)
        found = saved_config_findings(configs, shipped_config_names(), name, xml_text)
        for row in found["rows"]:
            row.update(listing.get(row["name"]) or {})
        self.results["savedConfigs"] = dict(found, activeSource=source)
        s = [f"## Saved hardware configs ({CONFIG_DIR}, read only)\n",
             "| config | size | modified | webcams | Ethernet devices | note |", "|---|---|---|---|---|---|"]
        for row in found["rows"]:
            note = ("written by the bench runner" if row["bench"] else "doesn't parse" if not row["parses"]
                    else "not shipped in this checkout's res/xml" if row["stale"] else "")
            s.append(f"| {row['name']}{' (active)' if row['name'] == name else ''} | {row.get('size', '?')} | "
                     f"{row.get('modified', '?')} | {row['webcams'] or ''} | {row['ethernet'] or ''} | {note} |")
        s.append("")
        s += [f"- WARN {w}" for w in found["warnings"]] or ["- no warnings"]
        s.append(f"- active config: {name}, {source}\n")
        self.summary += s
        for w in found["warnings"]:
            self.log(f"WARN saved config {w}")
        if found["check"] is None:
            self.log(f"the active config {name} wasn't judged: {source}")
        else:
            ok, detail = found["check"]
            self.check(f"saved configs: the active config {name}, if it names a webcam, names it {CELL_TIP_WEBCAM}", ok,
                       f"{detail} ({source})",
                       f"every CellTipCamera OpMode fails INIT on this hub: CellTipCamera opens the webcam named "
                       f"{CELL_TIP_WEBCAM}; rename the Webcam in {name} on the Driver Station")
        return s

    def list_configs(self):
        """--list-configs: reads the hub's configs and settings over adb and changes nothing, so any hub will do."""
        self.log("--list-configs: listing the hub's hardware configs; nothing on the hub is changed")
        try:
            self.original_entry = self.adb.active_config_entry()
        except BenchError as e:
            self.log(f"{e}; the active config is unknown, so it isn't judged")
        for line in self.run_saved_configs():
            print(line)

    def record_hardware(self):
        """--probe-only: what the probe found and the REV hubs on the Control Hub's USB ports, for session.py's plan."""
        self.results["hardware"] = {"imuType": self.imu_type, "webcam": self.webcam, "limelight": self.limelight,
                                    "expansion": self.expansion, "usbRevHubs": usb_rev_hub_serials(self.adb)}
        self.log(f"hardware: {self.results['hardware']}")

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
        planner = ph.get("planner", {})
        cold = planner.get("coldFirstPlan", {})
        s.append(f"- Planner, one {planner.get('sliceMs')} ms slice per loop: cold first plan ({cold.get('balls')} balls"
                 f"{'' if cold.get('firstPlanInProcess') else ', NOT the first in this app process'}): "
                 f"{cold.get('planMs', float('nan')):.1f} ms over {cold.get('loops')} loops, "
                 f"{cold.get('computeMs', float('nan')):.1f} ms of it in slices, longest slice "
                 f"{cold.get('longestStepMs', float('nan')):.1f} ms")
        by_count = planner.get("byBallCount", {})
        for n, v in sorted(by_count.items()):
            s.append(f"- Planner {n}: plan {fmt(v.get('planMs'))}; in slices {fmt(v.get('computeMs'))}; "
                     f"{fmt(v.get('loopsPerPlan'), 'loops')}; longest slice per plan {fmt(v.get('longestStepMs'))}; "
                     f"{v.get('plansWithDroppedBalls')} of {v.get('plans')} dropped a ball")
        cases = planner.get("byCase", {})
        for n, v in cases.items():
            s.append(f"  - {n}: plan {fmt(v.get('planMs'))}; longest slice {fmt(v.get('longestStepMs'))}")
        s.append(f"- Planner slices {fmt(planner.get('stepMs'))}; past the slice {fmt(planner.get('overrunMs'))}; "
                 f"loop period while planning {fmt(planner.get('loopPeriodMs'))}")
        judged = {n: dig(v, "longestStepMs", "max") for n, v in by_count.items()
                  if int(n.split("=")[1]) <= PLANNER_JUDGED_BALLS}
        judged.update({n: dig(v, "longestStepMs", "max") for n, v in cases.items()
                       if int(re.match(r"\d+", n).group()) <= PLANNER_JUDGED_BALLS})
        if cold.get("balls") is not None and cold["balls"] <= PLANNER_JUDGED_BALLS:
            judged["cold first plan"] = cold.get("longestStepMs")
        worst = max(judged.items(), key=lambda kv: kv[1] if kv[1] is not None else float("inf"), default=None)
        self.check(f"hub: every planner slice for up to {PLANNER_JUDGED_BALLS} balls returns within "
                   f"{PLANNER_SLICE_MAX_MS} ms", worst is not None and worst[1] is not None
                   and all(v is not None for v in judged.values()) and worst[1] < PLANNER_SLICE_MAX_MS,
                   f"longest slice {worst[1]} ms ({worst[0]}) against a {planner.get('sliceMs')} ms slice"
                   if worst is not None else "no plans of up to that many balls",
                   PLANNER_SLICE_PENDING if judged else "Bench: Hub didn't reach its planner phase")
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
                     f"fake server {v.get('fakeServerCpuMsPerSecond', 0):.1f} ms CPU/s, process CPU with the in-process "
                     f"fake Limelight included {dig(v, 'cpu', 'processCpuPercentOfOneCore', default=0):.0f}% of a core "
                     f"(Bench: Limelight measures a real one); loop {fmt(v.get('loopPeriodMs'))}; workload "
                     f"{fmt(v.get('workloadMs'))}")
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
                top = ("profiler off" if dig(ph, "settings", "loopProfile") is False else
                       ", ".join(f"{x['section']} {x['meanMs']:.2f}" for x in ph.get("sections", []) if not x["nested"]))
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
        hub_report = {}
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
            log_text = step_log((self.out / "logs" / f"{step}.log").read_text(), step)
            line = o.get("statusErrorMessage") or ""
            expected_stops = ["A", "B"] if throw_in == "moduleInit" else ["A", "B", "C"]
            self.check(f"failure {throw_in}: module stops", stops == expected_stops, f"stops {stops}, expected {expected_stops}")
            while_active = o.get("errorLinesWhileActive") or []
            if shown is not None:
                stale = [error for _, error in while_active if mentions(error, shown)] or (
                    [line] if mentions(line, shown) else [])
                if stale or any(error == "" for _, error in while_active):
                    self.check(f"failure {throw_in}: the previous crash's Error line was cleared at INIT", not stale,
                               f"Error line while {throw_in} was active [s after INIT sent, line]: {while_active}; "
                               f"{line!r} at its end",
                               "a crash's Error line outlived the next INIT, so the dashboard showed a stale crash")
                else:
                    s.append(f"- {throw_in}: whether the previous crash's Error line was cleared at INIT wasn't seen: "
                             f"every status while it was active already showed its own crash {while_active}")
            hub_s = self.failure_hub_seconds(step, throw_in, log_text)
            hub_report[throw_in] = hub_s
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
                           f"Error line {line!r}" + (f" at the runner's status read {o['errorLineSeconds']} s after it saw "
                                                     "the crash" if "errorLineSeconds" in o
                                                     else " (the runner saw no crash)" if not o["crashed"] else
                                                     f", still empty {ERROR_LINE_WAIT_S:.0f} s after the runner saw the "
                                                     "crash"), ERROR_LINE_PENDING)
                self.check(f"failure {throw_in}: stop failure attached as Suppressed",
                           "Suppressed:" in log_text and "stub B stop()" in log_text, "looked for 'Suppressed:' in the log")
                self.check(f"failure {throw_in}: onEnd not called", "onEnd" not in events, f"events {events}")
            shown = text if mentions(line, text) else None
            s.append(f"- throwIn={throw_in}: hub: the SDK logged '{USER_CODE_THREW}' {hub_s} s after the throw; runner: "
                     f"crashed={o['crashed']} (via {o.get('crashSignal')}, seen "
                     f"{o.get('crashSeenAfterStartSeconds', o.get('crashSeenSeconds'))} s after "
                     f"{'START' if start else 'INIT'} was sent), Error line {o.get('errorLineSeconds')} s after that; "
                     f"error={o['errorMessage']!r} from {o.get('errorSource')}, Error line {line!r}, events={events}")
        thrown = [v for k, v in hub_report.items() if k != "none" and v is not None]
        s.append(f"- On the hub's clock (logcat), the SDK logged '{USER_CODE_THREW}' "
                 + (f"{min(thrown):.2f}-{max(thrown):.2f} s after a hook threw" if thrown else "at times this run couldn't "
                    "read from the logs")
                 + (f" and {hub_report['none']:.2f} s after stub B's stop() threw" if hub_report.get("none") is not None
                    else "")
                 + ". The runner's times are its own: it learns of a crash from slothboard's push of that line, and "
                   "reads the Error line with an adb round trip and a status request, then every 0.25 s.")
        s.append("- The Error-line checks read GET_ROBOT_STATUS's errorMessage, which the dashboard shows as its "
                 "Error line, with no log fallback.\n")
        self.summary += s

    def failure_hub_seconds(self, step, throw_in, log_text):
        event = FAILURE_EVENT + ("init B" if throw_in == "moduleInit" else "stop B" if throw_in == "none"
                                 else f"throwing in {throw_in}")
        lines = lines_after(log_text, f"begin {step}")
        start = next((i for i, line in enumerate(lines) if event in line), None)
        if start is None:
            return None
        reported = next((line for line in lines[start:] if USER_CODE_THREW in line
                         and OPMODE_MANAGER_TAG in line), None)
        return logcat_delta(lines[start], reported) if reported else None

    def run_write_recovery(self):
        o = self.run_opmode("Bench: Write Recovery", "write_recovery", timeout=60, result_files=["write_recovery"])
        res = self.pull("write_recovery", required=not o["crashed"]) or {}
        channels, trials, info = res.get("channels") or {}, res.get("trials") or [], res.get("info") or {}
        resend, loop_p95 = info.get("resendMs"), dig(res, "loopPeriodMs", "p95")
        s = ["## Bench: Write Recovery (failSafe() at several points of WriteCache's re-send period, hub registers "
             "read with Lynx commands)\n",
             f"- battery {info.get('batteryVolts')} V; loop {fmt(res.get('loopPeriodMs'))}; WriteCache re-sends an "
             f"unchanged value after {resend} ms. Each failSafe follows a write of every device (the commanded value "
             f"moved by {info.get('nudge')}) by the set ms {info.get('phasesMs')}, so a wrapped device's re-send is due "
             "that much less than the period after the failSafe"]
        names = list(channels)
        if trials:
            s += ["", "| failSafe | ms after the write | " + " | ".join(f"{n} back after ms (loops)" for n in names) + " |",
                  "|---|---|" + "---|" * len(names)]
            for t in trials:
                cells = [f"{dig(t, 'channels', n, 'recoveredMs')} ({dig(t, 'channels', n, 'recoveredLoops')})"
                         for n in names]
                s.append(f"| {t.get('trial')} | {t.get('phaseMs')} | " + " | ".join(cells) + " |")
            s.append("")
        if o["crashed"]:
            s.append(f"- CRASHED: {o['errorMessage']}")
        for name, c in channels.items():
            s.append(f"- {name} ({'EnhancedMotor/EnhancedServo' if c.get('wrapped') else 'raw SDK device'}, {c.get('kind')} "
                     f"port {c.get('port')}): at its target before the first failSafe {c.get('settledBeforeFailSafe')}; "
                     f"worst {c.get('worstRecoveredMs')} ms ({c.get('worstRecoveredLoops')} loops) after a failSafe "
                     f"{c.get('worstPhaseMs')} ms after the write; never back {c.get('neverRecovered')} times")
        s.append("")
        self.summary += s
        for name, ok, detail, means in write_recovery_checks(res, o):
            self.check(f"write recovery: {name}", ok, detail, means)

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

    def usb_expansion_serial(self):
        if self.args.usb_expansion_serial:
            return self.args.usb_expansion_serial
        serials = usb_rev_hub_serials(self.adb)
        if len(serials) != 1:
            raise BenchError(f"--usb-expansion found {len(serials)} REV hubs on the Control Hub's USB ports ({serials}): "
                             "connect exactly one Expansion Hub by USB (no RS485 cable) on its own 12 V, or pass its "
                             "serial with --usb-expansion-serial (the REV Hardware Client shows it)")
        return serials[0]

    def photon_topology(self):
        a = self.args
        if a.usb_expansion and a.photon_rs485:
            raise BenchError("--usb-expansion and --photon-rs485 are two wirings of the same Expansion Hub: run them "
                             "as two separate runs, recabling in between")
        if a.photon_rs485:
            if self.expansion is None:
                raise BenchError(f"--photon-rs485: no Expansion Hub answered on RS485 at address {a.expansion_address}: "
                                 "connect it to the Control Hub's RS485 port only (no USB cable), on 12 V, or pass "
                                 "--expansion-address N")
            return "rs485"
        if a.usb_expansion:
            if self.expansion is not None:
                raise BenchError("--usb-expansion: an Expansion Hub also answers on RS485; a hub on both USB and RS485 "
                                 "at once confuses the SDK. Unplug the RS485 cable")
            return "usb"
        return "none"

    def photon_config(self, topology=None):
        topology = topology or ("usb" if self.args.usb_expansion else "none")
        i2c = self.args.photon_i2c
        plan = i2c_hub_plan(topology)
        on_ch = plan["pinpoint"][0] == "controlHub"
        devices = main_devices(self.imu_type, pinpoint=i2c and on_ch, octoquad_bus=plan["octoquad"][1] if i2c else None)
        eh_devices = PHOTON_EXPANSION_DEVICES + ([pinpoint_tag(plan["pinpoint"][1])] if i2c and not on_ch else [])
        if topology == "usb":
            serial = self.usb_expansion_serial()
            self.photon_eh_serial = serial
            self.log(f"photon: USB Expansion Hub serial {serial}, address {self.args.expansion_address}")
            return hub_config(devices, usb_expansion=(serial, self.args.expansion_address, eh_devices))
        if topology == "rs485":
            return hub_config(devices, expansion=(self.args.expansion_address, eh_devices))
        return hub_config(devices)

    def photon_i2c_where(self, topology):
        plan = i2c_hub_plan(topology)
        names = {"controlHub": "Control Hub", "expansionHub": "Expansion Hub"}
        return (f"the goBILDA Pinpoint in {names[plan['pinpoint'][0]]} I2C port {plan['pinpoint'][1]} and the OctoQuad in "
                f"{names[plan['octoquad'][0]]} I2C port {plan['octoquad'][1]}")

    def photon_guard_ports(self, topology):
        self.log(f"photon: this block drives {PHOTON_POWERS}")
        self.log("photon: the stock bench's probe checks only motor ports for current; servo ports 0-1 on each hub "
                 "aren't probed, so check them by eye")
        if self.args.ports_empty and sys.stdin.isatty():
            self.ask(">>> --ports-empty: confirm nothing is plugged into motor ports 0-3 or servo ports 0-1 of either hub")
        if topology == "none":
            return
        found = []
        for name in sorted(self.adb.config_names()):
            if name in BENCH_CONFIGS:
                continue
            found += [f"{name}: {d}" for d in expansion_port_devices(
                self.adb.read_config(name), getattr(self, "photon_eh_serial", None) if topology == "usb" else None,
                self.args.expansion_address)]
        if found and not self.args.ports_empty:
            raise BenchError(f"{'; '.join(found)}. The Photon benches drive the Expansion Hub's motor ports 0-3 and "
                             "servo ports 0-1 too. Unplug everything from them, then rerun with --ports-empty.")
        if found:
            self.log(f"--ports-empty: not stopping for Expansion Hub devices named in configs: {found}")

    def photon_step(self, opmode, step, params, result, timeout=600, extra_files=(), **kw):
        stream = self.logcat_stream(self.out / "logs" / f"{step}.stream.log") if kw.pop("stream_log", False) else None
        try:
            o = self.run_opmode(opmode, step, params, timeout=timeout, result_files=[result, *extra_files],
                                status_every=kw.pop("status_every", 15.0), quiet_after=kw.pop("quiet_after", 20.0), **kw)
        finally:
            if stream is not None:
                stream.stop()
        res = self.pull(result, required=False) or {}
        if res:
            (self.out / "raw" / f"{step}.json").write_text(json.dumps(res, indent=1))
        self.photon_note_log(step, list(res.get("phaseLog") or []), stream)
        if o["crashed"] and not kw.get("expect_crash"):
            self.photon_add(PhotonCheck("phases", f"{opmode} ({step}) ran to the end", False,
                                        f"crashed: {o.get('errorMessage')!r}", "something under Photon threw"))
        return o, res

    def logcat_stream(self, path):
        return LogcatStream(self.adb, path)

    def photon_note_log(self, step, expected=(), stream=None):
        log_file = stream.path if stream is not None else self.out / "logs" / f"{step}.log"
        log_text = step_log(log_file.read_text(), step) if log_file.exists() else ""
        self.photon["logs"][step] = {"parsed": photon_log_counts(log_text), "expected": list(expected),
                                     "stopLocks": stop_lock_events(log_text)}

    def photon_log_entries(self, *steps):
        return [(s, self.photon["logs"].get(s, {}).get("parsed"), self.photon["logs"].get(s, {}).get("expected", []))
                for s in steps]

    def photon_wait_file(self, name, timeout=12.0):
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            res = self.pull(name, required=False)
            if res:
                (self.out / "raw" / f"{name}.json").write_text(json.dumps(res, indent=1))
                return res
            time.sleep(0.5)
        return None

    def photon_add(self, *checks):
        for c in checks:
            self.photon["checks"].append(c)
            self.check(f"photon {PHOTON_TITLES[c.section].lower()}: {c.name}", c.ok, c.detail, c.fail_means)

    def photon_routing(self, label, base, step=None):
        rid = step or label
        _, res = self.photon_step("Bench: Photon Routing", f"photon_routing_{rid}", dict(base, runId=rid),
                                  f"photon_routing_{rid}", timeout=180)
        self.photon["routing"].append((label, res, f"photon_routing_{rid}"))
        self.log(f"photon routing {label}: swap order {dig(res, 'atInit', 'swapOrder')}, problems "
                 f"{photon_routing_problems(res) or 'none'}")
        return res

    def run_photon(self):
        """The Photon block (see scripts/bench/README.md): stock baseline first after a restart, then every Photon
        check this topology allows, then the verdict."""
        a = self.args
        topology = self.photon_topology()
        seed = a.photon_seed if a.photon_seed is not None else int(time.time())
        rng = random.Random(seed)
        hubs = ["controlHub"] + (["expansionHub"] if topology != "none" else [])
        parallel_hubs = ["controlHub"] + (["expansionHub"] if topology == "usb" else [])
        trials = a.photon_stop_trials or (PHOTON_SOAK_STOP_TRIALS if a.photon_soak else PHOTON_STOP_TRIALS)
        self.photon = {"topology": topology, "seed": seed, "checks": [], "logs": {}, "routing": [], "notRun": [],
                       "stopTrials": trials, "manifest": self.photon_manifest()}
        self.log(f"=== photon: topology {topology}, seed {seed}")
        try:
            base = {"expansion": {"none": 0, "usb": 1, "rs485": 2}[topology], "i2c": 1 if a.photon_i2c else 0}
            loops = 150 if a.quick else 500
            config = self.photon_config(topology)
            self.photon_guard_ports(topology)
            if a.photon_i2c:
                where = self.photon_i2c_where(topology)
                self.log(f"photon: --photon-i2c reads {where}; nothing is written to them but the OctoQuad driver's own "
                         "read-parameter commands")
                self.tell(f">>> --photon-i2c: plug {where} (nothing else on those ports)")
            self.activate_config(CONFIG_PHOTON, config)

            stock_run, stock = self.photon_step("Bench: Photon Stock", "photon_stock",
                                                dict(base, runId="1", loops=loops, portProbe=1), "photon_stock_1", timeout=900)
            self.photon["stock"] = stock
            self.photon_add(*photon_install_checks(stock, self.photon["manifest"]))
            self.photon_add(*photon_port_checks(stock))
            if not isinstance(stock.get("portProbe"), dict):
                raise BenchError(f"Bench: Photon Stock {'crashed' if stock_run['crashed'] else 'ended'} before its port "
                                 f"probe ran" + (f": {stock_run['errorMessage']}" if stock_run.get("errorMessage") else ""))
            if dig(stock, "portProbe", "empty") is not True:
                raise BenchError("a bench motor port drew current in Bench: Photon Stock's port probe (see the summary): "
                                 "unplug it before running the Photon benches")
            self.photon_add(*photon_stock_checks(stock))
            self.photon_add(*photon_log_checks("Bench: Photon Stock", self.photon_log_entries("photon_stock")))
            self.photon_routing("first Photon INIT after the restart", base, "r01")

            soak_seconds = photon_soak_seconds(a)
            soak_writes = PHOTON_SOAK_WRITES_PER_HUB if a.photon_soak else 0
            _, main = self.photon_step("Bench: Photon", "photon_main",
                                       dict(base, runId="1", loops=loops, soakSeconds=soak_seconds,
                                            soakWritesPerHub=soak_writes, twoWriterSeconds=20 if a.photon_soak else 8,
                                            i2cControlAttempts=PHOTON_I2C_CONTROL_ATTEMPTS),
                                       "photon_1", timeout=soak_seconds + 1800, status_every=30.0, quiet_after=60.0,
                                       stream_log=a.photon_soak)
            self.photon["main"] = main
            phases = {}
            for group in ("timing", "burst", "lastWriterWins", "interleave", "reads", "i2c", "hubOverlap"):
                for name, ph in (dig(main, group, default={}) or {}).items():
                    phases[f"{group} {name}"] = ph
            if dig(main, "i2cAfterControl") is not None:
                phases["i2c after control"] = dig(main, "i2cAfterControl")
            self.photon_add(*photon_main_init_checks(main))
            self.photon_add(*photon_workload_checks("phases", "", phases))
            self.photon_add(*photon_gain_checks(stock, main, parallel_hubs))
            if dig(main, "twoWriters") is not None:
                self.photon_add(*photon_thread_checks(dig(main, "twoWriters")))
            if dig(main, "storm") is not None:
                self.photon_add(*photon_storm_checks(dig(main, "storm")))
            soak_checks, soak_met = photon_soak_checks(dig(main, "soak"))
            self.photon_add(*(soak_checks if a.photon_soak else soak_checks[:1]))
            self.photon["soakMet"] = soak_met
            main_log = self.photon["logs"].get("photon_main", {}).get("parsed") or {}
            if topology == "rs485":
                self.photon_add(*photon_rs485_checks(main, main_log))
            if a.photon_i2c:
                self.photon_add(*photon_i2c_checks(stock, main, topology))
            self.photon_add(*photon_log_checks("Bench: Photon", self.photon_log_entries("photon_main"),
                                               exempt=("i2c control",) if a.photon_i2c else ()))
            self.photon["overlap"] = photon_overlap_verdict(main, topology)
            self.log(f"photon: {self.photon['overlap']['line']}")

            depth_reps = PHOTON_SOAK_DEPTH_REPS if a.photon_soak else PHOTON_DEPTH_REPS
            _, faults = self.photon_step("Bench: Photon Faults", "photon_faults",
                                         dict(base, runId="1", trials=3, depthReps=depth_reps,
                                              supersededReps=PHOTON_SUPERSEDED_REPS),
                                         "photon_faults_1", timeout=1800)
            self.photon["faults"] = faults
            self.photon_add(*photon_lost_checks(faults, parallel_hubs, 3))
            self.photon_add(*photon_lost_depth_checks(faults, parallel_hubs, depth_reps))
            self.photon_add(*photon_superseded_checks(faults, parallel_hubs, PHOTON_SUPERSEDED_REPS))
            self.photon_add(*photon_write_failure_checks(faults, parallel_hubs))
            self.photon_add(*photon_reset_checks(faults, hubs))
            self.photon_add(*photon_log_checks("Bench: Photon Faults", self.photon_log_entries("photon_faults"),
                                               exempt=("lost write", "lost over limit", "reset recovery")))

            self.photon["race"] = race = []
            race_steps = []
            for parallel in (8, 0):
                for hub in hubs:
                    for t in range(PHOTON_RACE_TRIALS):
                        tid = f"{'on' if parallel else 'off'}_{hub}_{t}"
                        _, res = self.photon_step("Bench: Photon Stop Race", f"photon_race_{tid}",
                                                  dict(base, trial=tid, photon=parallel, hub=hub), f"photon_stop_race_{tid}",
                                                  timeout=60)
                        race.append(res)
                        race_steps.append(f"photon_race_{tid}")
            self.photon_add(*photon_race_checks(race))
            self.photon_add(*photon_log_checks("Bench: Photon Stop Race", self.photon_log_entries(*race_steps)))

            self.photon["stop"] = stop = {}
            conditions = (("iterative, Photon on8", "Bench: Photon Stop", 8, 0, trials, None),
                          ("iterative, Photon off", "Bench: Photon Stop", 0, 0, trials, None),
                          ("linear, Photon on8", "Bench: Photon Stop Linear", 8, 0, trials, None),
                          ("linear, Photon off", "Bench: Photon Stop Linear", 0, 0, trials, None),
                          ("negative control, held", "Bench: Photon Stop", 8, 1, PHOTON_NEGATIVE_CONTROL_TRIALS, "held"),
                          ("negative control, once", "Bench: Photon Stop", 8, 2, PHOTON_NEGATIVE_CONTROL_TRIALS, "once"),
                          ("negative control, linear once", "Bench: Photon Stop Linear", 8, 2,
                           PHOTON_NEGATIVE_CONTROL_TRIALS, "once"))
            for label, opmode, parallel, control, n, _ in conditions:
                stop[label] = [self.photon_stop_trial(opmode, label, base, parallel, control, i, rng) for i in range(n)]
            fired = True
            for label, opmode, _, control, _, expect in conditions:
                checks = photon_stop_checks(label, stop[label], expect=expect)
                self.photon_add(*checks)
                if expect:
                    fired &= all(c.ok for c in checks)
                elif "off" not in label:
                    self.photon_add(photon_stop_coverage(label, stop[label]))
                linear = opmode == "Bench: Photon Stop Linear"
                self.photon_add(*photon_log_checks(f"STOP trials, {label}",
                                                   self.photon_log_entries(*[t["step"] for t in stop[label]]),
                                                   ignore=("abandoning lock",) if linear else ()))
            self.photon["negativeControlFired"] = fired
            self.photon_add(*photon_linear_lock_checks(
                *[[(t["trial"], self.photon["logs"].get(t["step"], {}).get("stopLocks")) for t in stop[label]]
                  for label in ("linear, Photon on8", "linear, Photon off", "negative control, linear once")]))

            self.photon_consecutive(base, rng)
            if a.no_sloth:
                self.photon["notRun"].append("deploySloth (--no-sloth)")
            else:
                self.photon_sloth(base)
            self.photon_restarts(base, len(hubs))
            routing_steps = [step for _, _, step in self.photon["routing"]]
            self.photon_add(*photon_log_checks("Bench: Photon Routing (every INIT)", self.photon_log_entries(*routing_steps),
                                               exempt=("not responding",)))
            if a.interactive and topology == "usb":
                self.photon_replug(base)
            self.photon_framework()
        finally:
            try:
                self.photon_summary()
            finally:
                if a.photon_i2c and topology != "none":
                    self.write_summary()
                    self.tell(f">>> --photon-i2c done: put the Pinpoint back in Control Hub I2C port 1")

    def photon_stop_trial(self, opmode, label, base, parallel, control, i, rng):
        kind = {0: "on" if parallel else "off", 1: "held", 2: "once"}[control]
        tid = f"{'linear' if 'Linear' in opmode else 'iter'}_{kind}_{i:03d}"
        step = f"photon_stop_{tid}"
        delay = round(rng.uniform(PHOTON_STOP_DELAY_S[0], PHOTON_STOP_DELAY_S[1]), 3)
        self.log(f"photon STOP {label} trial {i}: STOP {delay} s after START")
        o = self.run_opmode(opmode, step, dict(base, trial=tid, photon=parallel, control=control),
                            timeout=delay, timeout_ok=True, result_files=[f"photon_stop_{tid}", f"photon_stop_main_{tid}"],
                            status_every=1.0, quiet_after=5.0)
        sampler = self.photon_wait_file(f"photon_stop_{tid}")
        self.photon_note_log(step, ["writing"])
        t = {"trial": tid, "step": step, "delayS": delay, "crashed": o["crashed"], "appRestarted": o.get("appRestarted"),
             "error": o.get("statusErrorMessage") or o.get("errorMessage") or "", "sampler": sampler}
        v = stop_trial_verdict(sampler, require_zeroing_read=True)
        problems = stop_trial_problems(t, v)
        self.log(f"photon STOP {label} trial {i}: {v['violations']} violations ({v['zeroingReadViolations']} in stop()'s "
                 f"read), problems {problems or 'none'}")
        return t

    def photon_consecutive(self, base, rng):
        seq = []
        seq.append(("Photon OpMode", "routing", self.photon_routing("consecutive 1: Photon", base, "h1")))
        _, stock = self.photon_step("Bench: Photon Stock", "photon_stock_h2", dict(base, runId="h2", loops=50, short=1),
                                    "photon_stock_h2", timeout=300)
        seq.append(("non-Photon OpMode", "stock", stock))
        before = self.photon_routing("consecutive 3: Photon", base, "h3")
        seq.append(("Photon OpMode again", "routing", before))
        tid = "h4"
        o, main = self.photon_step("Bench: Photon Crash", "photon_crash_h4",
                                   dict(base, trial=tid, crashAfterMs=round(rng.uniform(800, 2000)),
                                        lostWrites=PHOTON_CRASH_LOST_WRITES),
                                   f"photon_crash_{tid}", timeout=60, expect_crash=True,
                                   extra_files=[f"photon_crash_sampler_{tid}"])
        crash = {"outcome": o, "main": main, "sampler": self.photon_wait_file(f"photon_crash_sampler_{tid}")}
        self.photon["crash"] = crash
        after = self.photon_routing("consecutive 5: after the crash", base, "h5")
        seq.append(("INIT after the crash", "routing", after))
        self.photon_add(*photon_crash_checks(crash, before, after))
        self.photon_add(*photon_log_checks("Bench: Photon Crash", self.photon_log_entries("photon_crash_h4"),
                                           exempt=("losing writes",)))
        self.photon["consecutive"] = seq
        self.photon_add(*photon_consecutive_checks(seq))

    def photon_sloth(self, base):
        before = self.photon_routing("before deploySloth", base, "s1")
        hub_t0 = self.adb.hub_time()
        deploy = self._deploy_sloth("photon", None)
        load = self.sloth_load(hub_t0, "photon", deploy)
        self.log(f"photon deploySloth: {load}")
        time.sleep(PHOTON_SLOTH_SETTLE_S)
        if self.adb.pid() != self.known_pid:
            self.wait_restart("photon deploySloth")
        after = self.photon_routing("after deploySloth", base, "s2")
        self.photon["sloth"] = {"deploy": {k: v for k, v in deploy.items() if k != "connectionAtPush"}, "load": load}
        self.photon_add(*photon_sloth_checks(before, after, load))

    def photon_restarts(self, base, hub_count):
        first = [r for l, r, _ in self.photon["routing"] if l == "first Photon INIT after the restart"]
        runs = [("restart 1", first[0])] if first else []
        orders = {tuple(dig(r, "atInit", "swapOrder", default=[]) or []) for _, r in runs}
        n = len(runs)
        while n < PHOTON_MAX_RESTARTS and (n < PHOTON_MIN_RESTARTS or (hub_count > 1 and len({o[0] for o in orders if o}) < 2)):
            n += 1
            self.activate_config(CONFIG_PHOTON)
            res = self.photon_routing(f"restart {n}", base, f"x{n:02d}")
            runs.append((f"restart {n}", res))
            orders.add(tuple(dig(res, "atInit", "swapOrder", default=[]) or []))
        others = [(l, r) for l, r, _ in self.photon["routing"]
                  if not l.startswith("restart") and l != "first Photon INIT after the restart"]
        checks, both = photon_routing_checks(runs + others, PHOTON_MIN_RESTARTS, hub_count)
        self.photon["ordersBoth"] = both
        self.photon_add(*checks)

    def photon_replug(self, base):
        what = {"usb": "unplug the Expansion Hub's USB cable, count to two, plug it back in",
                "power": "pull the Expansion Hub's XT30 power lead (the one from the Control Hub's second XT30 port), "
                         "count to one, plug it back in"}
        for mode in ("usb", "power"):
            res = {}
            for parallel in (0, 8):
                label = PHOTON_WORKLOAD_MODE[parallel]
                self.ask(f">>> Next ({mode} pull, Photon {label}): {what[mode]}. Press Enter, then wait for the PULL NOW "
                         "line (about 5 s). Its motor ports are empty, so nothing moves; the step ends by itself about "
                         "3 s after the hub is back.")
                _, res[parallel] = self.photon_step(
                    "Bench: Photon Replug", f"photon_replug_{mode}_{label}", dict(base, mode=mode, photon=parallel,
                                                                                 maxSeconds=90),
                    f"photon_replug_{mode}_{label}", timeout=150, during=lambda m=mode: self.log(f">>> PULL NOW: {what[m]}"))
                self.wait_ready()
            self.photon[f"replug_{mode}"] = res
            self.photon_add(*photon_replug_checks(res[0], res[8], mode))

    def photon_framework(self):
        self.activate_config(CONFIG_MAIN, self.main_config())
        secs = 4 if self.args.quick else 8
        runs = {}
        steps = []
        for parallel in (0, 8):
            step = f"framework_loop_photon{parallel}"
            o = self.run_opmode("Bench: Framework Loop Photon", step, {"phaseSeconds": secs, "photon": parallel},
                                timeout=30 * secs + 120, result_files=["framework_loop"], status_every=15.0,
                                quiet_after=secs + 4)
            res = self.pull("framework_loop", required=not o["crashed"])
            if res is not None:
                (self.out / "raw" / f"{step}.json").write_text(json.dumps(res, indent=1))
            self.photon_note_log(step)
            steps.append(step)
            runs[parallel] = (o, res)
        self.photon["framework"] = {k: dig(v[1], "phases", default={}) for k, v in runs.items()}
        o8, r8 = runs[8]
        self.photon_add(PhotonCheck("phases", "Framework Loop (EnhancedOpMode and Pedro) runs to the end with Photon on",
                                    r8 is not None and not o8["crashed"] and dig(r8, "photonParallel") == 8,
                                    f"crashed {o8['crashed']}, error {o8.get('errorMessage')!r}, photonParallel "
                                    f"{dig(r8, 'photonParallel')}", "the framework's loop broke under Photon"))
        o = self.run_opmode("Bench: Write Recovery Photon", "write_recovery_photon", {"photon": 8}, timeout=60,
                            result_files=["write_recovery"])
        res = self.pull("write_recovery", required=not o["crashed"]) or {}
        if res:
            (self.out / "raw" / "write_recovery_photon.json").write_text(json.dumps(res, indent=1))
        self.photon_note_log("write_recovery_photon")
        steps.append("write_recovery_photon")
        self.photon["writeRecovery"] = res
        for name, ok, detail, means in write_recovery_checks(res, o):
            self.photon_add(PhotonCheck("reset", f"Write Recovery with Photon on: {name}", ok, detail,
                                        means or "Bench: Write Recovery crashed under Photon"))
        self.photon_add(*photon_log_checks("Framework Loop and Write Recovery with Photon", self.photon_log_entries(*steps)))
        self.log("photon: restarting the app so the benches after this block get the SDK's own hub objects")
        self.activate_config(CONFIG_MAIN)

    def photon_manifest(self):
        def git(*args):
            r = subprocess.run(["git", *args], cwd=REPO_ROOT, capture_output=True, text=True)
            return r.stdout.strip() if r.returncode == 0 else f"git failed: {r.stderr.strip()[:200]}"

        return {"gitHead": git("rev-parse", "HEAD"), "gitDirty": bool(git("status", "--porcelain")),
                "photonSourceSha256": photon_source_sha256(REPO_ROOT / PHOTON_SOURCE_DIR),
                "probeFirmware": [h.get("firmware") for h in (self.probe or {}).get("hubs", [])]}

    def photon_summary(self):
        p = self.photon
        checks = p["checks"]
        s = [f"## Photon ({p['topology']}, seed {p['seed']})\n",
             f"- source: git {p['manifest']['gitHead'][:12]}{' (dirty tree)' if p['manifest']['gitDirty'] else ''}, "
             f"photon sources sha256 {p['manifest']['photonSourceSha256'][:16]} (installed on the hub: "
             f"{str(dig(p, 'stock', 'info', 'installedPhotonSourceSha256'))[:16]}); SDK {dig(p, 'stock', 'info', 'sdkVersion')}; "
             f"hubs {[(h.get('key'), h.get('address'), h.get('firmware')) for h in dig(p, 'stock', 'info', 'hubs', default=[]) or []]}",
             f"- drives {PHOTON_POWERS}",
             f"- STOP trials: {p['stopTrials']} per condition; with none violating, the 95% upper bound on the per-STOP "
             f"violation rate is {photon_stop_bound(p['stopTrials']):.1%} per condition",
             f"- I2C: {self.photon_i2c_where(p['topology'])}, two readers each every 10 ms, read only"
             if self.args.photon_i2c else "- I2C: not run (--photon-i2c)", ""]
        rows = photon_gain_rows(p.get("stock"), p.get("main"))
        if rows:
            hubs = sorted({h for r in rows for h in r["perHub"]})
            s += ["### Write block per loop (4 motors + 2 servos changing on each hub), p50 / p95 / max ms\n",
                  "| mode | both hubs | " + " | ".join(hubs) + " | loop period | p50 gain over stock |",
                  "|---|---|" + "---|" * len(hubs) + "---|---|"]
            for r in rows:
                s.append(f"| {r['mode']} | {fmt_cell(r['writeBlock'])} | "
                         + " | ".join(fmt_cell(r["perHub"].get(h)) for h in hubs)
                         + f" | {fmt_cell(r['loopPeriod'])} | {gain_text(r['gain'])} |")
            s.append("")
        if self.args.photon_i2c:
            s += photon_i2c_summary(p.get("stock"), p.get("main"))
        s += photon_profile_summary(p.get("main"))
        s += photon_overlap_summary(p.get("main"), p["topology"])
        for section, title, proves in PHOTON_TESTS:
            mine = [c for c in checks if c.section == section]
            if not mine:
                continue
            ok = all(c.ok for c in mine)
            s.append(f"### {title}: {'PASS' if ok else 'FAIL'}\n")
            s.append(f"_Proves: {proves}._\n")
            for c in mine:
                s.append(f"- {'PASS' if c.ok else 'FAIL'} {c.name}: {c.detail}"
                         + ("" if c.ok else f" **FAIL means:** {c.fail_means}"))
            s.append("")
        required = [sec for sec, _, _ in PHOTON_TESTS
                    if sec not in ("sloth", "rs485", "replug", "i2c") or (sec == "sloth" and not self.args.no_sloth)
                    or (sec == "rs485" and p["topology"] == "rs485")
                    or (sec == "replug" and self.args.interactive and p["topology"] == "usb")
                    or (sec == "i2c" and self.args.photon_i2c)]
        missing = [f"{PHOTON_TITLES[sec]} (no result)" for sec in required if not any(c.section == sec for c in checks)]
        still = ["the cable pulls (--interactive)"] if p["topology"] == "usb" and not self.args.interactive else []
        if not self.args.photon_i2c:
            still.append("I2C reads through Photon's hubs on every hub (--photon-i2c)")
        level, blockers, needed = photon_verdict(checks, p.get("negativeControlFired", False), p.get("ordersBoth", False),
                                                 self.args.photon_soak, p.get("soakMet", False), p["stopTrials"],
                                                 p["notRun"] + missing, still)
        s.append("### Photon verdict\n")
        if level == "SAFE":
            s.append(f"**SAFE TO TRY ON THE ROBOT** ({p['topology']} wiring): every Photon check passed, every STOP negative "
                     "control fired, the routing check saw both module orders, the soak met its loss bound, I2C read "
                     "right on every hub" + (" and the cable pulls passed." if p["topology"] == "usb" else "."))
        elif level == "PASSED":
            s.append(f"**PASSED THIS RUN, NOT YET SAFE** ({p['topology']} wiring): every check here passed, but "
                     "the go needs a run that also includes:")
            s += [f"- {n}" for n in needed]
        else:
            s.append(f"**NOT SAFE YET** ({p['topology']} wiring). Blockers:")
            s += [f"- {b}" for b in blockers]
            if needed:
                s.append("\nAlso still needed for the go:")
                s += [f"- {n}" for n in needed]
        blind = i2c_control_blind(p.get("main")) if self.args.photon_i2c else []
        if blind:
            s.append("\nNot shown by the I2C negative control: " + "; ".join(blind) + ".")
        s.append("")
        self.summary += s
        self.results["photon"] = {k: v for k, v in p.items() if k not in ("checks",)}
        self.results["photon"]["checks"] = [c._asdict() for c in checks]
        self.results["photon"]["verdict"] = {"level": level, "safe": level == "SAFE", "blockers": blockers,
                                             "stillNeeded": needed}

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
            loop = telemetry_float((packet.get("data") or {}).get(KEY_LOOP))
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
        self.summary += ["## Bench: Overlay NaN\n",
                         f"- a NaN circle in every overlay from loop {start} to {end} (only the last packet of each "
                         f"slothboard batch keeps its overlay, so one loop alone could be dropped unsent); bench done "
                         f"at loop {last}; newest loop received {seen}; packets after the NaN ones {after}",
                         f"- status warning after it: {o.get('warningMessage')!r}; error {o.get('errorMessage')!r}",
                         "- the runner restarted the app afterwards\n"]

    def run_planner_stop(self):
        s = ["## Bench: Planner Stop (STOP while Vision Ball Collection-style replanning runs in INIT)\n",
             "| balls | STOP sent after the next plan began | plans | STOP landed | slice left at STOP (ms) | STOP to "
             "stop() (ms) | hub STOP to idle (ms) | runner STOP to idle (s) | app restarted | hub STOP to running again "
             "(s) | stuck markers |",
             "|---|---|---|---|---|---|---|---|---|---|---|"]

        def whole_ms(v):
            return "" if v is None else f"{v:.0f}"

        trials = []
        for balls in PLANNER_STOP_BALLS:
            for delay in PLANNER_STOP_DELAYS_S:
                trial = self._planner_stop(balls, delay)
                trials.append(trial)
                if "failed" in trial:
                    s.append(f"| {balls} | {delay} s | FAILED: {trial['failed']} | | | | | | | | |")
                    continue
                landed = "not seen"
                if trial["planAtStop"]:
                    landed = f"in plan {trial['planAtStop']}" + (f", {trial['msIntoPlan']:.0f} ms into it"
                                                                 if trial["msIntoPlan"] is not None else "")
                    if trial.get("inSlice") is not None:
                        landed += (f", {trial['msIntoSlice']:.0f} ms into a slice" if trial["inSlice"]
                                   else ", between slices")
                elif trial["stopSeen"]:
                    landed = "between plans"
                if trial.get("warmSeconds") is None:
                    landed += f" (never reached {PLANNER_STOP_WARM} plans; STOP came after the 10 s timeout)"
                s.append(f"| {balls} | {delay} s | {len(trial['plans'])} | {landed} | {whole_ms(trial.get('sliceLeftMs'))} | "
                         f"{whole_ms(trial.get('stopToStopPassMs'))} | {whole_ms(trial.get('hubStopToIdleMs'))} | "
                         f"{trial['stopSeconds']} | {trial['restarted']} | {trial.get('hubStopToRunningS') or ''} | "
                         f"{len(trial['markers'])} |")
        s.append("")
        for balls in PLANNER_STOP_BALLS:
            ran = [t for t in trials if t["balls"] == balls and "failed" not in t]
            plans = [ms for t in ran for ms in t["plans"]]
            compute = [ms for t in ran for ms in t["planComputeMs"]]
            loops = [n for t in ran for n in t["planLoops"]]
            firsts = [round(t["plans"][0]) for t in ran if t["plans"]]
            cold = [round(t["plans"][0]) for t in ran if t["firstPlanInProcess"] and t["plans"]]
            slices = [dig(t, "stepMs", "max") for t in ran if dig(t, "stepMs", "count")]
            periods = [dig(t, "loopPeriodMs", "max") for t in ran if dig(t, "loopPeriodMs", "count")]
            s.append(f"- {balls} balls ({ran[0]['layout'] if ran else 'no trial ran'}), one "
                     f"{ran[0]['sliceMs'] if ran else '?'} ms slice per loop: every plan {fmt_plans(plans)} from "
                     f"request to done; in slices {fmt_plans(compute)}; loops per plan {sorted(set(loops))}; first plan "
                     f"of each trial {firsts} ms; first plans in a fresh app process {cold} ms")
            for t in ran:
                s.append(f"  - {t['delay']} s: slices {fmt(t.get('stepMs'))}; past the slice {fmt(t.get('overrunMs'))}; "
                         f"loop period while planning {fmt(t.get('loopPeriodMs'))}")
            restarted = [f"{t['delay']} s" for t in ran if t["restarted"] or t["markers"]]
            mid = [t for t in ran if t["planAtStop"]]
            self.check(f"planner stop {balls} balls: STOP during replanning never restarts the app",
                       len(ran) == len(PLANNER_STOP_DELAYS_S) and not restarted,
                       f"{len(ran)} of {len(PLANNER_STOP_DELAYS_S)} trials ran, {len(mid)} with STOP mid-plan; plans "
                       f"{fmt_plans(plans)}; restarts or stuck markers after STOP at {restarted or 'none'}"
                       + "".join(f"; {t['delay']} s back to Robot Status: running {t['hubStopToRunningS']} s after STOP "
                                 "(hub clock)" for t in ran if t.get("hubStopToRunningS") is not None),
                       PLANNER_STOP_PENDING if len(ran) == len(PLANNER_STOP_DELAYS_S) else
                       "a trial didn't finish (see the table), so not every STOP was tried")
            idle = [t.get("hubStopToIdleMs") for t in ran]
            slow = [f"{t['delay']} s" for t in ran if not (t.get("hubStopToIdleMs") is not None
                                                          and t["hubStopToIdleMs"] < SDK_STOP_WINDOW_MS)]
            self.check(f"planner stop {balls} balls: the hub logs every STOP reaching idle within {SDK_STOP_WINDOW_MS} ms "
                       "('auto-stopping' to '***STOP - OPMODE***')",
                       len(ran) == len(PLANNER_STOP_DELAYS_S) and not slow,
                       f"hub STOP to idle {idle} ms (None: no such line, as after a restart); at or over "
                       f"{SDK_STOP_WINDOW_MS} ms or missing at {slow or 'none'}; the runner saw "
                       f"{[t['stopSeconds'] for t in ran]} s through its status polling",
                       PLANNER_STOP_IDLE_PENDING if len(ran) == len(PLANNER_STOP_DELAYS_S) else
                       "a trial didn't finish (see the table), so not every STOP was timed")
            if balls == PLANNER_JUDGED_BALLS:
                self.check(f"planner stop {balls} balls: every planner slice returns within {PLANNER_SLICE_MAX_MS} ms",
                           len(slices) == len(ran) and bool(ran) and max(slices) < PLANNER_SLICE_MAX_MS,
                           f"longest slice per trial {[round(v, 1) for v in slices]} ms; loop period while planning "
                           f"at most {[round(v, 1) for v in periods]} ms" if slices else "no trial recorded a slice",
                           PLANNER_SLICE_PENDING if slices else "no trial ran far enough to plan")
        s.append("- The bench requests the crafted layout again at each 250 ms replan check once the last plan is done, "
                 "as Vision Ball Collection does while its balls keep moving, and steps it once per init_loop() as the "
                 "collection OpModes do; it records when the SDK's stopRequested flips. STOP is sent the set delay after "
                 "the runner sees a plan finish. Hub STOP to idle is 'auto-stopping' to '***STOP - OPMODE***' and STOP "
                 "to running again is 'auto-stopping' to 'Robot Status: running', both on the hub's logcat clock; the "
                 f"SDK gives STOP {SDK_STOP_WINDOW_MS} ms (+100) to finish the running init_loop() and stop() before it "
                 "restarts the app.\n")
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
        rec.update(plans=res.get("planMs") or [], planComputeMs=res.get("planComputeMs") or [],
                   planLoops=res.get("planLoops") or [], stepMs=res.get("stepMs"), overrunMs=res.get("overrunMs"),
                   loopPeriodMs=res.get("loopPeriodMs"), layout=dig(res, "info", "layout"),
                   sliceMs=dig(res, "info", "sliceMs"), firstPlanInProcess=dig(res, "info", "firstPlanInProcess"),
                   stopSeen=bool(stop.get("stopSeen")), planAtStop=stop.get("planRunningAtStop"),
                   msIntoPlan=stop.get("msIntoPlanAtStop"), inSlice=stop.get("inSliceAtStop"),
                   msIntoSlice=stop.get("msIntoSliceAtStop"), sliceLeftMs=stop.get("sliceLeftAtStopMs"),
                   stopToStopPassMs=stop.get("stopToStopPassMs"),
                   stopSeconds=round(o.get("stopSecondsRunnerSide", float("nan")), 2),
                   restarted=bool(o.get("appRestarted") or o.get("restartedDuringStop")),
                   markers=o.get("logMarkers") or [])
        log_file = self.out / "logs" / f"{step}.log"
        log_text = log_file.read_text() if log_file.exists() else ""
        rec.update(self._planner_stop_hub_times(log_text, step, rec["restarted"]))
        if not stop.get("stopSeen"):
            for m in PLANNER_STOP_SEEN.finditer(log_text):
                if m.group(1) == run_id:
                    rec.update(stopSeen=True, planAtStop=int(m.group(3)) if m.group(3) else None,
                               msIntoPlan=float(m.group(2)) if m.group(2) else None)
        return rec

    @staticmethod
    def _planner_stop_hub_times(log_text, step, restarted):
        """STOP to idle, and after a restart STOP to running again, from the step's logcat on the hub's clock."""
        lines = log_text.splitlines()
        begin = next((i for i, line in enumerate(lines) if "BENCH_RUNNER" in line and f"begin {step}" in line), 0)
        stop = next((i for i in range(begin, len(lines)) if PLANNER_AUTO_STOPPING in lines[i]), None)
        if stop is None:
            return {"hubStopToIdleMs": None}

        def hub_ms(end):
            a, b = logcat_time(lines[stop]), logcat_time(lines[end])
            if a is None or b is None:
                return None
            ms = (b - a).total_seconds() * 1000
            return round(ms + 366 * 86400 * 1000 if ms < -180 * 86400 * 1000 else ms, 1)

        idle = next((i for i in range(stop, len(lines)) if PLANNER_STOPPED_LINE in lines[i]), None)
        out = {"hubStopToIdleMs": hub_ms(idle) if idle is not None else None}
        if restarted:
            running = next((i for i in range(stop, len(lines)) if PLANNER_RUNNING_LINE.search(lines[i])), None)
            back = hub_ms(running) if running is not None else None
            out["hubStopToRunningS"] = round(back / 1000, 1) if back is not None else None
        return out

    def send_on_side_socket(self, obj):
        side = SideSocket(*self.link.address(DASHBOARD_PORT))
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
                lambda: (telemetry_float(self.dash.data.get(KEY_LOOP)) or 0) >= 20, 15)
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
                 "Gamepad view would without slothboard 6165.6's client fix, and the server applies every socket's "
                 "state, so held loops reading the stick as 0 and extra lb rises are expected there. Only "
                 "--interactive exercises the client fix.\n")
        self.summary += s

    def gp_send(self, phase, **pad):
        with self.gp_lock:
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
        page, _ = self.link.dash_page()
        self.log(f">>> Open {page} in a browser, pick the Custom layout and add the Gamepad view.")
        for phase, _, text, _ in GP_INTERACTIVE:
            self.gp_mark(phase)
            self.ask(f">>> {text}, then come back here.")
        self.gp_mark(GP_INTERACTIVE_IDLE)
        self.ask(">>> Unplug the gamepad and close the dashboard tab: the next benches time the bus and count gamepad "
                 "states.")

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
        if interactive and self.link.usb:
            _, why = self.link.dash_page()
            if why:
                self.log(f"!! the keyboard, on-screen stick and USB pad phases are skipped: {why}")
                s.append(f"- not run over USB, the keyboard, on-screen stick and USB pad phases: {why}")
                rec["interactiveSkipped"] = why
                interactive = False
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
            side = SideSocket(*self.link.address(DASHBOARD_PORT))
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
        self.check(f"webcam stop {run_id}: no force-stop, restart or error", ok,
                   f"markers {outcome['logMarkers'][:3]}, restarted {outcome['appRestarted']}, "
                   f"error {outcome.get('errorMessage')!r}")
        pass_ms = None
        if "onEndMs" in res and "moduleStopsStartMs" in res:
            pass_ms = round(res["onEndMs"] - res["moduleStopsStartMs"], 1)
        s.append(f"| {run_id} | {label} | {res.get('cellTipStopMs')} | {pass_ms} | "
                 f"{outcome.get('stopSecondsRunnerSide', 0):.2f} | {len(outcome['logMarkers'])} | {outcome['appRestarted']} |")
        time.sleep(1.5)

    def run_absent(self):
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
            rec["post"] = self.link.http("POST", LIMELIGHT_PORT, f"/pipeline-switch?index={proxy_pipeline}")
            t0 = time.monotonic()
            while time.monotonic() - t0 < 3:
                reply = self.link.http("GET", LIMELIGHT_PORT, "/status")
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
            unanswered = (f"the runner's POST to the hub's port {LIMELIGHT_PORT} got no HTTP answer, so slothboard's "
                          "Limelight proxy isn't listening there")
            if not answered and self.link.usb:
                _, why = forward_reach(LIMELIGHT_PORT, self.adb.listeners_or_none((LIMELIGHT_PORT,)))
                rec["hubSide"] = why
                unanswered = (f"the runner's POST through the forward to the hub's port {LIMELIGHT_PORT} got no HTTP "
                              f"answer, which over USB doesn't say whether the proxy listens; the hub's socket table: "
                              f"{why}")
            self.check(f"limelight proxy: a POST with ?index={proxy_pipeline} through slothboard's proxy switches the "
                       "pipeline", proxy.get("reachedToMs") is not None,
                       f"POST {post.get('code', post.get('error'))} in {post.get('ms')} ms; /status through the proxy "
                       f"{rec.get('statusViaProxy')}; the Limelight's own status on the hub {proxy.get('indexChanges')} "
                       f"(from {proxy.get('from')}, reached {proxy_pipeline} at {proxy.get('reachedToMs')} ms)",
                       LIMELIGHT_PROXY_PENDING if answered else unanswered)
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
            self.ask(text)
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
            self.contained("llfault", lambda: self._llfault(tag))
            self.contained("llhung", self._llhung)
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

    def _llhung(self):
        """Bench: Limelight Hung, five runs back to back on one app process: INIT held through a hung POST, then twice
        a STOP during one followed at once by the next INIT, the second time with LimelightSync's own worker flag
        cleared as a Sloth reload would leave it."""
        rec = {}

        def until_fault():
            rec["faultSeenS"] = self.wait_or_crash(
                lambda: str(self.dash.data.get(KEY_LIMELIGHT_STATE)) == "fault", LLHUNG_FAULT_WAIT_S)
            if rec["faultSeenS"] is not None:
                self.wait_or_crash(lambda: False, 1.0)

        runs = []
        o = self.run_opmode(LLHUNG_OPMODE, "llhung_hung_init", {"mode": "hung_init", "runId": "hung_init"},
                            during_init=until_fault, timeout=30, result_files=["limelight_hung_hung_init"])
        runs.append(("hung_init", o, self.pull("limelight_hung_hung_init", required=False) or {}))
        pairs = []
        for stop_id, next_id in (("stop_hung_1", "reinit"), ("stop_hung_2", "reinit_fresh")):
            # No pull between the two: the next INIT has to land while the stopped run's POST is still hung.
            o_stop = self.run_opmode(LLHUNG_OPMODE, f"llhung_{stop_id}", {"mode": "stop_hung", "runId": stop_id},
                                     start=False, timeout=30, result_files=[f"limelight_hung_{stop_id}"])
            o_next = self.run_opmode(LLHUNG_OPMODE, f"llhung_{next_id}", {"mode": next_id, "runId": next_id},
                                     start=False, timeout=40, result_files=[f"limelight_hung_{next_id}"])
            pairs.append((stop_id, o_stop, next_id, o_next))
        for stop_id, o_stop, next_id, o_next in pairs:
            runs.append((stop_id, o_stop, self.pull(f"limelight_hung_{stop_id}", required=False) or {}))
            runs.append((next_id, o_next, self.pull(f"limelight_hung_{next_id}", required=False) or {}))
        self.results["llhung_hung_init"]["runner"] = rec
        by_id = {run_id: (o, r) for run_id, o, r in runs}

        s = ["## Bench: Limelight Hung (a fake Limelight that serves results and never answers a POST)\n",
             "The config's `limelight` is pointed at a fake on 127.0.0.1:15807, so Camera, LimelightBallSource and "
             "LimelightSync run unchanged. The fake answers every GET (results, status) and holds each POST open until "
             "the hub gives up, as a Limelight busy loading a pipeline can. A hung GET too would only "
             "test the never-connected fault again: LimelightBallSource asks for the sync after its first good poll.\n",
             "| run | first INIT loop (ms) | LimelightSync threads at INIT | POSTs hung at INIT | first Limelight fault | "
             "stop pass / Camera.stop (ms) | POST still hung after stop() | runner STOP to idle (s) |",
             "|---|---|---|---|---|---|---|---|"]
        for run_id, o, r in runs:
            stop = r.get("stop") or {}
            fault = (f"{r['firstFaultMs']:.0f} ms ({r.get('firstFaultPhase')}): {r.get('firstFaultMessage')!r}"
                     if r.get("firstFaultMs") is not None else "none")
            s.append(f"| {run_id} | {r.get('firstInitLoopMs')} | {dig(r, 'atInit', 'senderThreadCount')} | "
                     f"{dig(r, 'atInit', 'openPosts', default=0)} | {fault} | {stop.get('stopPassMs')} / "
                     f"{stop.get('cameraStopMs')} | {(stop.get('openPostsAfter') or 0) > 0} | "
                     f"{o.get('stopSecondsRunnerSide', float('nan')):.2f} |")
        s.append("")
        for run_id, o, r in runs:
            self.check(f"llhung {run_id}: ran without crashing or restarting the app",
                       not o["crashed"] and not o["appRestarted"] and not r.get("stoppedBeforeDone", True),
                       f"error {o.get('errorMessage')!r}, restarted {o['appRestarted']}, done line {o.get('doneLine')!r}")

        broken = {run_id for run_id, o, r in runs if not r or o["crashed"]}
        if "hung_init" in broken:
            s.append("- hung_init: its checks weren't run, because the run crashed or wrote no result (its "
                     "'ran without crashing' check says why)")
        else:
            o, r = by_id["hung_init"]
            timeout_ms = dig(r, "info", "postTimeoutMs") or 15000
            deadline_ms = timeout_ms + LLHUNG_FAULT_SLACK_MS
            posts = r.get("posts") or []
            held = [p for p in posts if p.get("held")]
            first, message = r.get("firstFaultMs"), str(r.get("firstFaultMessage"))
            self.check(f"llhung: with every POST hung, the {LIMELIGHT_FAULT_KEY} fault is raised in INIT within "
                       f"{deadline_ms / 1000:.0f} s (Limelight3A's {timeout_ms / 1000:.0f} s POST timeout + "
                       f"{LLHUNG_FAULT_SLACK_MS / 1000:.0f} s)",
                       first is not None and first <= deadline_ms and r.get("firstFaultPhase") == "init"
                       and "rejected" in message,
                       f"first at {first} ms ({r.get('firstFaultPhase')}): {message!r}; dashboard saw it "
                       f"{rec.get('faultSeenS')} s after INIT was seen", LLHUNG_FAULT_MEANS)
            hung = dig(r, "phases", "init", "loopPeriodPostOpenMs", default={})
            self.check("llhung: INIT keeps looping while the POST hangs",
                       hung.get("count", 0) >= 100 and hung.get("p95", float("inf")) <= 50
                       and hung.get("max", float("inf")) <= 500, f"{fmt(hung)}", LLHUNG_LOOP_MEANS)
            running = dig(r, "phases", "running", default={})
            period = running.get("loopPeriodMs") or {}
            self.check("llhung: the loop keeps running after START with the fault up",
                       (running.get("loopsPerSecond") or 0) >= 10 and period.get("p95", float("inf")) <= 50
                       and period.get("max", float("inf")) <= 500,
                       f"{running.get('loopsPerSecond') or 0:.0f} loops/s, {fmt(period)}", LLHUNG_LOOP_MEANS)
            self.check("llhung: one POST went to the hung Limelight, not retried, and the hub gave up on it",
                       len(posts) == 1 and len(held) == 1 and str(held[0].get("endedBy")).startswith("client"),
                       f"{len(posts)} POSTs: {[(p.get('line'), p.get('openMs'), p.get('endedBy')) for p in posts]}",
                       LLHUNG_RETRY_MEANS)
            if held:
                columns, rows = r.get("rowColumns") or [], r.get("rows") or []
                before = ([row for row in rows if first is None or row[columns.index("ms")] < first]
                          if "ms" in columns else [])
                stale = (sum(1 for row in before if row[columns.index("frameStale")] == 1)
                         if "frameStale" in columns else None)
                s.append(f"- hung_init: the POST hung {round(held[0].get('openMs') or 0)} ms ({held[0].get('endedBy')}); "
                         f"before the fault at {first} ms, Camera's frame read stale in {stale} of {len(before)} sampled "
                         "rows")

        for stop_id, _, next_id, _ in pairs:
            if stop_id in broken:
                s.append(f"- {stop_id} then {next_id}: not tested, because {stop_id} crashed or wrote no result")
                continue
            o_stop, r_stop = by_id[stop_id]
            o_next, r_next = by_id[next_id]
            fresh = next_id == "reinit_fresh"
            stop = r_stop.get("stop") or {}
            prev_id = "hung_init" if stop_id == "stop_hung_1" else "reinit"
            if prev_id in broken:
                s.append(f"- {stop_id}: leftover-thread check not tested: {prev_id} crashed or wrote no result, so it "
                         "never closed its hung POST")
            else:
                self.check(f"llhung {stop_id}: no LimelightSync thread left from the run before",
                           dig(r_stop, "atInit", "senderThreadCount") == 0,
                           f"{dig(r_stop, 'atInit', 'senderThreads')}", LLHUNG_LEFTOVER_MEANS)
            hung_at_stop = (stop.get("openPosts") or 0) >= 1 and len(stop.get("senderThreads") or []) == 1
            self.check(f"llhung {stop_id}: STOP landed while a POST was hung", hung_at_stop,
                       f"{stop.get('openPosts')} POSTs open, sender threads {stop.get('senderThreads')}",
                       LLHUNG_SETUP_MEANS)
            stop_s = o_stop.get("stopSecondsRunnerSide", float("inf"))
            if hung_at_stop:
                self.check(f"llhung {stop_id}: STOP isn't held up by the hung POST",
                           (stop.get("stopPassMs") or float("inf")) <= LLHUNG_STOP_PASS_MS
                           and (stop.get("cameraStopMs") or float("inf")) <= LLHUNG_CAMERA_STOP_MS
                           and (stop.get("openPostsAfter") or 0) >= 1 and stop_s < LLHUNG_STOP_IDLE_S
                           and not o_stop["logMarkers"] and not o_stop["appRestarted"],
                           f"stop pass {stop.get('stopPassMs')} ms (Camera.stop {stop.get('cameraStopMs')} ms), POST "
                           f"still hung after stop() {stop.get('openPostsAfter')}, runner STOP to idle {stop_s:.2f} s "
                           f"(hung_init, nothing hung: "
                           f"{by_id['hung_init'][0].get('stopSecondsRunnerSide', float('nan')):.2f} s), "
                           f"markers {o_stop['logMarkers'][:3]}", LLHUNG_STOP_MEANS)
            else:
                s.append(f"- {stop_id}: STOP during a hung POST not tested (no POST was hung at STOP)")

            if next_id in broken:
                s.append(f"- {next_id}: not tested, because it crashed or wrote no result (if {stop_id}'s STOP was held "
                         "up, the app restart that followed is why)")
                continue
            at = r_next.get("atInit") or {}
            posts = r_next.get("posts") or []
            previous = [p for p in posts if (p.get("receivedMs") or 0) < 0]
            new = [p for p in posts if (p.get("receivedMs") or 0) >= 0]
            held = [p.get("endedMs") for p in previous if p.get("held") and p.get("endedMs") is not None]
            released = max(held) if held else None
            ready = (at.get("openPosts") or 0) >= 1 and at.get("senderThreadCount") == 1
            if fresh:
                ready = ready and at.get("senderWasRunning") is True
            self.check(f"llhung {next_id}: the next INIT came while the stopped run's POST was still hung", ready,
                       f"{at.get('openPosts')} POSTs hung, LimelightSync threads {at.get('senderThreads')}"
                       + (f", worker flag was set {at.get('senderWasRunning')}" if fresh else ""), LLHUNG_SETUP_MEANS)
            first_loop = r_next.get("firstInitLoopMs")
            overlap = r_next.get("maxPostOverlapMs")
            serialized = (overlap is not None and overlap <= LLHUNG_OVERLAP_MS and bool(new) and released is not None
                          and new[0].get("receivedMs", -1) >= released - LLHUNG_OVERLAP_MS)
            timeline = [(p.get("line"), p.get("receivedMs"), p.get("endedMs"), p.get("endedBy")) for p in posts]
            if not ready:
                s.append(f"- {next_id}: INIT behind a hung POST and its single-sender check not tested (no POST was "
                         "hung at its INIT)")
            elif fresh:
                self.check("llhung reinit_fresh: a second worker on the shared blackboard semaphore waits for the "
                           "first one's hung POST",
                           serialized and r_next.get("maxSenderThreads") == 2
                           and dig(r_next, "atInit", "sender", "syncUsesIt") is True,
                           f"{r_next.get('maxSenderThreads')} LimelightSync threads at once (2 expected: the stopped "
                           f"run's and the new worker), LimelightSync's sender is the blackboard semaphore "
                           f"{dig(r_next, 'atInit', 'sender', 'syncUsesIt')}, longest POST overlap {overlap} ms; POSTs "
                           f"[line, received, ended, by] {timeline}", LLHUNG_FRESH_MEANS)
            else:
                self.check("llhung reinit: the next run's POSTs wait for the previous run's hung POST (the stopped run's "
                           "worker takes the new request, so this doesn't exercise the shared blackboard sender; "
                           "reinit_fresh does)",
                           serialized and r_next.get("maxSenderThreads") == 1,
                           f"{r_next.get('maxSenderThreads')} LimelightSync threads at once, longest POST overlap "
                           f"{overlap} ms; POSTs [line, received, ended, by] {timeline}", LLHUNG_SINGLE_MEANS)
            if ready:
                self.check(f"llhung {next_id}: INIT doesn't wait for the previous run's hung POST",
                           first_loop is not None and (released is None or first_loop < released),
                           f"first INIT loop at {first_loop} ms ({stop_id}'s at {r_stop.get('firstInitLoopMs')} ms); "
                           f"the previous POST was released at {released} ms", LLHUNG_REINIT_WAIT_MEANS)
            live = r_next.get("firstLiveFrameMs")
            answering_from = max(released or 0, 0)
            raises = [e for e in r_next.get("events") or []
                      if e.get("key") == LIMELIGHT_FAULT_KEY and e.get("what") == "raise"]
            self.check(f"llhung {next_id}: the sync completes once the Limelight answers again",
                       dig(r_next, "sync", "doneMs") is not None and dig(r_next, "sync", "problem") is None
                       and live is not None and live - answering_from <= LLHUNG_SYNC_AFTER_RELEASE_MS and not raises,
                       f"sync done at {dig(r_next, 'sync', 'doneMs')} ms ({dig(r_next, 'sync', 'problem')!r}), first "
                       f"live frame at {live} ms, previous POST released at {released} ms, faults {raises}",
                       LLHUNG_RECOVER_MEANS)
            self.check(f"llhung {next_id}: no LimelightSync thread left once the sync is done",
                       dig(r_next, "stop", "senderThreadCountAfter") == 0,
                       f"{dig(r_next, 'stop', 'senderThreadsAfter')}", LLHUNG_LEFTOVER_MEANS)
            old = [p for p in previous if p.get("held") and p.get("endedMs") is not None]
            waited = (f"; the stopped run's POST was sent {-old[0]['receivedMs']:.0f} ms before this INIT and released "
                      f"{old[0]['endedMs'] - old[0]['receivedMs']:.0f} ms after it was sent, and the sync finished at "
                      f"{dig(r_next, 'sync', 'doneMs')} ms" if old else "")
            s.append(f"- {stop_id} then {next_id}: POSTs on the fake [line, received ms, ended ms, by], times from "
                     f"{next_id}'s INIT: {timeline}{waited}")
        senders = sorted({str(dig(r, "atInit", "sender", "identity")) for _, _, r in runs
                          if dig(r, "atInit", "sender", "present")})
        s.append(f"- the blackboard's LimelightSync.sender semaphore across the runs: {senders} (every run here shares "
                 "one class loader, so one identity is expected whatever the sender is; only reinit_fresh's check "
                 f"touches the blackboard sharing); permits at reinit's INIT "
                 f"{dig(by_id['reinit'][1], 'atInit', 'sender', 'permits')} (0 while the hung POST holds it)")
        s.append("- reinit_fresh clears LimelightSync's `running` flag by reflection, so the next request starts a "
                 "second worker while the first is still blocked, and checks LimelightSync's own sender is the "
                 "blackboard semaphore a reloaded class would share. It doesn't load the class afresh: a real "
                 "deploySloth can't land inside the 15 s window.\n")
        self.summary += s

    def _step_text(self, step):
        path = self.out / "logs" / f"{step}.log"
        return path.read_text() if path.exists() else ""

    def list_threads(self, label):
        pid = self.adb.pid()
        if pid is None:
            method, names, tried = None, [], ["no Robot Controller pid"]
        else:
            method, names, tried = self.adb.thread_names(pid, self.thread_method)
        if method is not None:
            self.thread_method = method
        rec = {"label": label, "pid": pid, "method": method, "threads": len(names),
               "counters": sum(1 for n in names if n == THREAD_NAME), "tried": tried}
        self.thread_listings.append(rec)
        self.log(f"threads {label}: {rec['counters']} {THREAD_NAME} of {rec['threads']} app threads via {method}"
                 + (f" (tried {tried})" if tried else ""))
        return rec

    def _after_crash_done(self, result):
        """Seconds until the bench's crash watch finished its file (it rewrites afterCrash up to 5 s after the
        throw), or None."""
        local = self.out / "logs" / f"{result}.poll.json"
        t0 = time.monotonic()
        while time.monotonic() - t0 < THREAD_AFTER_CRASH_WAIT_S:
            try:
                self.adb.pull_bench_file(f"{result}.json", local)
                done = dig(json.loads(local.read_text()), "afterCrash", "done") is True
            except (BenchError, ValueError):
                done = False
            if done:
                return round(time.monotonic() - t0, 1)
            time.sleep(0.5)
        return None

    def _thread_run(self, run_id, mode, params=None, during=None, timeout=None, timeout_ok=False):
        step, result = f"threads_{run_id}", f"thread_lifecycle_{mode}"
        rec = {"runId": run_id, "mode": mode, "o": {}, "res": {}, "adb": {}}

        def run():
            rec["o"] = self.run_opmode(THREAD_OPMODE, step, dict({"mode": mode}, **(params or {})),
                                       start=mode != "stop_before_start",
                                       expect_crash=mode in ("crash_in_loop", "thread_failure"),
                                       timeout=timeout or THREAD_TIMEOUTS[mode],
                                       timeout_ok=timeout_ok or mode == "thread_failure", during=during,
                                       result_files=[result])
            if mode in ("crash_in_loop", "crash_in_stop"):
                rec["afterCrashWaitS"] = self._after_crash_done(result)
            rec["res"] = self.pull(result, required=False, local_name=f"thread_lifecycle_{run_id}") or {}
            rec["adb"] = self.list_threads(f"after {run_id}")
            self.results[step]["adbThreads"] = rec["adb"]

        rec["finished"] = self.contained(f"threads {run_id}", run)
        return rec

    def run_threads(self):
        control = {}

        def positive_control():
            self.wait_or_crash(lambda: False, 2.0)
            control.update(self.list_threads("during normal, TransferCounter running"))

        runs = [self._thread_run("normal", "normal", during=positive_control)]
        for run_id, mode in (("stop_before_start", "stop_before_start"), ("crash_in_loop", "crash_in_loop"),
                             ("after_crash_in_loop", "normal"), ("crash_in_stop", "crash_in_stop"),
                             ("after_crash_in_stop", "normal"), ("thread_failure", "thread_failure"),
                             ("interrupt_probe", "interrupt_probe")):
            runs.append(self._thread_run(run_id, mode))
        repeats = [self._thread_run(f"repeat_{n:02d}", "normal", {"seconds": THREAD_REPEAT_SECONDS})
                   for n in range(1, THREAD_REPEATS + 1)]
        sloth = None
        if not self.args.no_sloth:
            sloth = {}
            self.contained("threads sloth", lambda: self._thread_sloth(sloth))
        self.results["threads"] = {"listings": self.thread_listings, "listingMethod": self.thread_method,
                                   "sloth": {k: v for k, v in (sloth or {}).items() if k not in ("run", "next")}}
        self._thread_report(runs, repeats, control, sloth)

    def _thread_sloth(self, rec):
        """deploySloth while a long normal run polls: Sloth stages the load and releases it from the idle OpMode."""
        run_id = "threads_sloth"
        hub_t0 = self.adb.hub_time()

        def polls():
            m = re.match(r"(\d+) TransferCounter polls", str(self.dash.data.get(KEY_STATUS, "")))
            return int(m.group(1)) if m else None

        def reload():
            self.wait_or_crash(lambda: polls() is not None, 5)
            rec["pollsBefore"] = polls()
            rec["deploy"] = self._deploy_sloth(run_id, None, THREAD_SLOTH_DEPLOY_TIMEOUT_S)
            self.wait_or_crash(lambda: False, 2.0)
            rec["pollsAfter"] = polls()
            status = self.dash.get_status()
            rec["afterDeploy"] = {"activeOpMode": status.get("activeOpMode"), "status": status.get("activeOpModeStatus"),
                                  "pid": self.adb.pid()}
            seen = self.adb.logcat_matches(hub_t0, SLOTH_STAGED, SLOTH_LOADED, SLOTH_CANCELLED, *SLOTH_FAILED)
            rec["stagedWhileRunning"] = SLOTH_STAGED in seen
            rec["endedWhileRunning"] = [e for e in (SLOTH_LOADED, SLOTH_CANCELLED) + SLOTH_FAILED if e in seen]

        rec["run"] = self._thread_run("sloth_reload", "normal", {"seconds": THREAD_SLOTH_SECONDS}, during=reload,
                                      timeout=3, timeout_ok=True)
        deploy = rec.get("deploy")
        if not rec["run"]["finished"] or deploy is None:
            return
        if deploy["hung"]:
            rec["load"] = {"result": f"deploySloth hung on the hub's Sloth lock and was killed after "
                                     f"{THREAD_SLOTH_DEPLOY_TIMEOUT_S} s"}
            return
        rec["load"] = self.sloth_load(hub_t0, run_id, dict(deploy, pushedAt=time.monotonic()))
        rec["afterLoad"] = self.list_threads("after the reload landed")
        rec["next"] = self._thread_run("after_sloth", "normal", {"seconds": THREAD_REPEAT_SECONDS})

    @staticmethod
    def _restart_between(prev, nxt):
        a, b = prev["o"], nxt["o"]
        if a.get("appRestarted") or a.get("restartedDuringStop"):
            return f"the app restarted during {prev['runId']} (pid {a.get('pidBefore')} -> {a.get('pidAfter')})"
        if b.get("pidBefore") != a.get("pidAfter"):
            return f"the app restarted between the runs (pid {a.get('pidAfter')} -> {b.get('pidBefore')})"
        return None

    def _thread_end_checks(self, r, adb_ok, means):
        run_id, o = r["runId"], r["o"]
        after = r["res"].get("afterCrash") or {}
        op, ctr = after.get("opModeThreadEndedMs"), after.get("counterThreadEndedMs")
        self.check(f"threads {run_id}: {THREAD_NAME}'s thread ended within {THREAD_END_GAP_MS} ms of the OpMode thread",
                   after.get("done") is True and op is not None and ctr is not None and ctr - op <= THREAD_END_GAP_MS
                   and not after.get("threads"),
                   f"after the throw the OpMode thread ended at {op} ms and {THREAD_NAME} at {ctr} ms (null: still alive "
                   f"after {after.get('watchedMs')} ms); {THREAD_NAME} threads left {after.get('threads')}; watch done "
                   f"{after.get('done')} ({r.get('afterCrashWaitS')} s wait)", means)
        if adb_ok:
            self.check(f"threads {run_id}: adb shows no {THREAD_NAME} thread afterwards", r["adb"].get("counters") == 0,
                       f"{r['adb'].get('counters')} of {r['adb'].get('threads')} threads via {r['adb'].get('method')}",
                       THREAD_ADB_MEANS)
        self.check(f"threads {run_id}: the app kept running (same process)",
                   not o.get("appRestarted") and not o.get("restartedDuringStop"),
                   f"pid {o.get('pidBefore')} -> {o.get('pidAfter')}", THREAD_RESTART_MEANS)

    def _thread_report(self, runs, repeats, control, sloth):
        by_id = {r["runId"]: r for r in runs}
        adb_ok = control.get("method") is not None and control.get("counters") == 1
        s = ["## Bench: Thread Lifecycle (biobuzz/modules/TransferCounter's poller through every way an OpMode ends)\n",
             "Each run's `leftover at INIT` counts the threads named TransferCounter alive when it INITs: no OpMode "
             "hook runs after a crash, so it is the evidence for the run before it. The adb column lists the app's "
             f"threads from outside after the run ({self.thread_method or 'no listing worked'}).\n",
             "| run | mode | leftover at INIT | stop() ms | threads after stop() | polls/s | adb TransferCounter after | "
             "pid before -> after | outcome |", "|---|---|---|---|---|---|---|---|---|"]
        for r in runs + repeats + [x for x in ((sloth or {}).get("run"), (sloth or {}).get("next")) if x]:
            o, res = r["o"], r["res"]
            stop = res.get("stop") or {}
            hz = dig(stop, "finalStats", "pollHz")
            outcome = ("FAILED" if not r["finished"] else f"crashed: {o.get('errorMessage')}" if o.get("crashed")
                       else "timed out (runner STOP)" if o.get("timedOut") else f"done: {o.get('doneLine')}")
            s.append(f"| {r['runId']} | {r['mode']} | {res.get('leftoverCountAtInit')} | {stop.get('stopMs')} | "
                     f"{stop.get('threadCountAfter')} | {round(hz) if isinstance(hz, (int, float)) else hz} | {r['adb'].get('counters')} | "
                     f"{o.get('pidBefore')} -> {o.get('pidAfter')} | {str(outcome)[:120]} |")
        s.append("")
        self.check("threads: the adb thread listing sees the running TransferCounter thread (positive control)", adb_ok,
                   f"{control.get('counters')} {THREAD_NAME} of {control.get('threads')} app threads via "
                   f"{control.get('method')}; tried first {control.get('tried')}",
                   "the adb listing can't see the poller on this hub, so every adb check below is skipped and only "
                   "the in-process thread counts in the result files judge leftovers")

        main = runs[:8]
        for prev, nxt in zip(main, main[1:]):
            means = THREAD_LEFTOVER_MEANS.get(prev["mode"])
            if means is None:
                continue
            name = f"threads {prev['runId']}: no {THREAD_NAME} thread alive at the next INIT ({nxt['runId']})"
            why = (f"{prev['runId'] if not prev['finished'] else nxt['runId']} didn't finish"
                   if not (prev["finished"] and nxt["finished"]) else self._restart_between(prev, nxt))
            if not why and nxt["res"].get("leftoverCountAtInit") is None:
                why = f"{nxt['runId']} wrote no leftover count ({'no result' if not nxt['res'] else 'crashed'})"
            if why:
                s.append(f"- {name}: not tested: {why}")
                continue
            self.check(name, nxt["res"].get("leftoverCountAtInit") == 0,
                       f"{nxt['res'].get('leftoverCountAtInit')} at INIT: {nxt['res'].get('leftoverAtInit')}", means)

        r = by_id["stop_before_start"]
        if r["finished"] and r["res"]:
            stop, o = r["res"].get("stop") or {}, r["o"]
            self.check("threads stop_before_start: stop() from INIT returns without starting or leaving a poller",
                       stop.get("started") is False and stop.get("returned") is True
                       and stop.get("threadCountAfter") == 0 and USER_CODE_THREW not in (o.get("errorMessage") or ""),
                       f"started {stop.get('started')}, returned {stop.get('returned')}, {stop.get('threadCountAfter')} "
                       f"left; error {o.get('errorMessage')!r}", THREAD_LEFTOVER_MEANS["stop_before_start"])

        r = by_id["normal"]
        if r["finished"] and not (r["res"] and not r["o"].get("crashed")):
            self.check("threads normal: the run finished without crashing", False,
                       f"crashed {r['o'].get('crashed')}: {r['o'].get('errorMessage')!r}; result file "
                       f"{'present' if r['res'] else 'missing'}",
                       "the plain TransferCounter run didn't complete, so its stop() and poll-rate checks weren't run")
        elif r["finished"]:
            res, o = r["res"], r["o"]
            samples = res.get("samples") or []
            counts = [len(x.get("threads") or []) for x in samples]
            self.check(f"threads normal: exactly one {THREAD_NAME} thread, a daemon, while it runs",
                       bool(samples) and all(c == 1 for c in counts)
                       and all(t.get("ours") and t.get("daemon") for x in samples for t in x.get("threads") or []),
                       f"{len(samples)} one-second samples with {counts} {THREAD_NAME} threads; daemon "
                       f"{res.get('counterThreadDaemon')}",
                       "start() made more than one poller, a non-daemon one, or none (no ball counting)")
            stop = res.get("stop") or {}
            ms = stop.get("stopMs")
            self.check(f"threads normal: stop() returns within {THREAD_STOP_MS} ms and leaves no {THREAD_NAME} thread",
                       stop.get("returned") is True and ms is not None and ms <= THREAD_STOP_MS
                       and stop.get("counterThreadAliveAfter") is False and stop.get("threadCountAfter") == 0,
                       f"returned {stop.get('returned')} after {ms} ms; thread alive after {stop.get('counterThreadAliveAfter')}, "
                       f"{stop.get('threadCountAfter')} left; error {o.get('errorMessage')!r}",
                       "TransferCounter.stop() is slow or leaves its thread: STOP waits on it (past the SDK's 900 ms stop "
                       "window the app restarts) or the poller outlives the OpMode")
            final = stop.get("finalStats") or {}
            hz = final.get("pollHz")
            self.check(f"threads normal: the poller keeps {THREAD_POLL_HZ}+ polls/s on a bare hub",
                       hz is not None and hz >= THREAD_POLL_HZ,
                       f"{hz} polls/s; {final.get('latePolls')} late of {final.get('polls')}; longest gap "
                       f"{final.get('maxGapMs')} ms; mean command {final.get('meanCommandMs')} ms",
                       "TransferCounter samples the sensors less often than its 10 ms design even with nothing else on "
                       "the bus, so a quick ball pass can fall between samples")

        for run_id, text, means in (
                ("crash_in_loop", THREAD_CRASH_IN_LOOP,
                 "after a crash in loop() the poller's isAlive tie didn't stop it promptly: it reads the hub after the "
                 "OpMode is gone"),
                ("crash_in_stop", THREAD_CRASH_IN_STOP,
                 "a stop() that throws before TransferCounter.stop() leaves the poller running: only the isAlive tie can "
                 "end it, and it didn't")):
            r = by_id[run_id]
            if not r["finished"]:
                continue
            o = r["o"]
            seen = text in (o.get("errorMessage") or "") or text in self._step_text(f"threads_{run_id}")
            if run_id == "crash_in_loop":
                seen = seen and bool(o.get("crashed"))
            self.check(f"threads {run_id}: the deliberate failure happened and reached the runner", seen,
                       f"crashed {o.get('crashed')} via {o.get('crashSignal')}: {o.get('errorMessage')!r}",
                       THREAD_SETUP_MEANS)
            if seen and not r["res"]:
                s.append(f"- {run_id}: thread-end checks not tested: the run wrote no result")
            elif seen:
                self._thread_end_checks(r, adb_ok, means)

        r = by_id["thread_failure"]
        if r["finished"]:
            o, res = r["o"], r["res"]
            inj, death = res.get("injection") or {}, res.get("death") or {}
            if not r["res"]:
                s.append("- thread_failure: not tested: the run wrote no result")
            elif inj.get("refused"):
                s.append(f"- thread_failure: UNAVAILABLE: ART refused the write to TransferCounter.portA "
                         f"({inj['refused']}), so a failing poller thread wasn't tried")
            elif not death and inj.get("portAfter") == THREAD_INVALID_PORT:
                self.check("threads thread_failure: the injected bad port made the poller throw", False,
                           f"portA {inj.get('portBefore')} -> {inj.get('portAfter')}; done line {o.get('doneLine')!r}",
                           "the bad port never made TransferCounter's thread throw, so this run didn't test how a "
                           "poller failure reaches the OpMode")
            else:
                seen = death.get("seenMsAfterInjection")
                text = (o.get("errorMessage") or "") + self._step_text("threads_thread_failure")
                self.check(f"threads thread_failure: a failing poller thread stops the OpMode with its cause within "
                           f"{THREAD_DEATH_MS} ms",
                           bool(o.get("crashed")) and THREAD_DIED in text and inj.get("portAfter") == THREAD_INVALID_PORT
                           and seen is not None and seen <= THREAD_DEATH_MS and THREAD_BAD_PORT in str(death.get("cause")),
                           f"crashed {o.get('crashed')}: {o.get('errorMessage')!r}; portA {inj.get('portBefore')} -> "
                           f"{inj.get('portAfter')}; seen {seen} ms ({death.get('loopsAfterInjection')} loops) after the "
                           f"injection; cause {death.get('cause')!r}; done line {o.get('doneLine')!r}",
                           "an exception on TransferCounter's thread didn't reach the OpMode thread promptly with its "
                           "cause, so a dead poller would silently stop counting balls")

        r = by_id["interrupt_probe"]
        if r["finished"]:
            o, probe = r["o"], r["res"].get("probe") or {}
            self.check(f"threads interrupt_probe: the probe ran ({THREAD_INTERRUPTS}+ interrupts, its helper stopped "
                       "with a flag and join)",
                       bool(o.get("doneLine")) and not o.get("crashed") and probe.get("helperJoined") is True
                       and (probe.get("interrupts") or 0) >= THREAD_INTERRUPTS,
                       f"{o.get('doneLine') or o.get('errorMessage')!r}",
                       "the probe didn't finish, so the interrupt finding below wasn't measured")
            abandons = self._step_text("threads_interrupt_probe").count(LOCK_ABANDONED)
            over = probe.get("ownReadsOver400Ms") or 0
            verdict = ("lock leak CONFIRMED" if over >= 1 and abandons >= 1 else "NOT REPRODUCED" if over == 0
                       and abandons == 0 else "inconclusive (a slow read or an abandoned lock, not both)")
            landed = probe.get("landed") or {}
            s.append(f"- interrupt() during hub commands, for the 'never interrupt()' rule: {verdict}. "
                     f"{probe.get('interrupts')} interrupts landed {landed}; slowest own read {probe.get('maxOwnReadMs')} ms, "
                     f"{over} over 400 ms, {probe.get('ownReadsOver50Ms')} over 50 ms; '{LOCK_ABANDONED}' x{abandons} in "
                     f"the log; own reads baseline {fmt(probe.get('baselineOwnReadMs'))}, interrupted "
                     f"{fmt(probe.get('interruptedOwnReadMs'))}"
                     + ("; no interrupt landed in MessageKeyedLock.release, so the leak window wasn't hit and the rule "
                        "stands on the source reading" if verdict == "NOT REPRODUCED" and not landed.get("inLockRelease")
                        else f"; {landed.get('inLockRelease')} interrupt(s) landed in MessageKeyedLock.release without a "
                        "leak, which contradicts the source reading" if verdict == "NOT REPRODUCED" else ""))

        bad, stop_ms = [], []
        untested = []
        for r in repeats:
            o, res = r["o"], r["res"]
            stop = res.get("stop") or {}
            if not r["finished"]:
                bad.append(f"{r['runId']} didn't finish")
                continue
            if not res:
                untested.append(r["runId"])
                continue
            stop_ms.append(stop.get("stopMs"))
            problems = [p for p, hit in (
                (f"crashed: {o.get('errorMessage')!r}", o.get("crashed")),
                ("no BENCH DONE", not o.get("doneLine")),
                (f"{res.get('leftoverCountAtInit')} {THREAD_NAME} at INIT", res.get("leftoverCountAtInit") != 0),
                (f"stop() returned {stop.get('returned')} with {stop.get('threadCountAfter')} left",
                 stop.get("returned") is not True or stop.get("threadCountAfter") != 0),
                (f"adb {r['adb'].get('counters')} {THREAD_NAME} after", adb_ok and r["adb"].get("counters") != 0),
                (f"app restarted (pid {o.get('pidBefore')} -> {o.get('pidAfter')})", o.get("appRestarted"))) if hit]
            if problems:
                bad.append(f"{r['runId']}: {'; '.join(problems)}")
        self.check(f"threads: {THREAD_REPEATS} back-to-back short runs each start with no {THREAD_NAME} thread and stop "
                   "clean", not bad, f"stop() ms {stop_ms}" + (f"; {bad}" if bad else "")
                   + (f"; not judged (no result): {untested}" if untested else ""), THREAD_LEFTOVER_MEANS["normal"])

        if sloth is None:
            s.append("- the Sloth hot reload during a running TransferCounter was skipped (--no-sloth)")
        else:
            self._thread_sloth_report(sloth, adb_ok, s)
        s.append("")
        self.summary += s

    def _thread_sloth_report(self, sloth, adb_ok, s):
        run, deploy = sloth.get("run"), sloth.get("deploy")
        if run is None or not run["finished"] or deploy is None:
            self.check("threads sloth: a hot reload during a running TransferCounter was tried", False,
                       f"run finished {bool(run and run['finished'])}, deploy {deploy}",
                       "deploySloth or the long run failed (its error is above), so the reload wasn't tried")
            return
        o, after, load = run["o"], sloth.get("afterDeploy") or {}, sloth.get("load") or {}
        before_p, after_p = sloth.get("pollsBefore"), sloth.get("pollsAfter")
        self.check("threads sloth: deploySloth while the OpMode runs leaves it running",
                   after.get("activeOpMode") == THREAD_OPMODE and after.get("status") == "RUNNING" and before_p is not None
                   and after_p is not None and after_p > before_p and not o.get("crashed") and not o.get("appRestarted"),
                   f"after the deploy ({deploy['seconds']:.0f} s, exit {deploy['returncode']}, hung {deploy['hung']}): "
                   f"{after}; TransferCounter polls {before_p} -> {after_p}; pid {o.get('pidBefore')} -> {o.get('pidAfter')}",
                   THREAD_SLOTH_RUNNING_MEANS)
        self.check("threads sloth: the hot load waits for STOP, then lands",
                   sloth.get("stagedWhileRunning") is True and not sloth.get("endedWhileRunning")
                   and load.get("result") == "processed",
                   f"staged while running {sloth.get('stagedWhileRunning')}, ended while running "
                   f"{sloth.get('endedWhileRunning')}; after STOP: {load.get('result')} (push to Staged {load.get('stagedS')} s, "
                   f"to its end {load.get('endS')} s)", THREAD_SLOTH_HELD_MEANS)
        stop, landed = run["res"].get("stop") or {}, sloth.get("afterLoad") or {}
        self.check(f"threads sloth: no {THREAD_NAME} thread survives the STOP and the reload",
                   stop.get("returned") is True and stop.get("threadCountAfter") == 0
                   and (not adb_ok or landed.get("counters") == 0),
                   f"stop() returned {stop.get('returned')} after {stop.get('stopMs')} ms with {stop.get('threadCountAfter')} "
                   f"left; adb once the load landed: {landed.get('counters')} of {landed.get('threads')} via "
                   f"{landed.get('method')}" + ("" if adb_ok else " (not judged: the adb positive control failed)"),
                   "the poller from the code before a deploySloth keeps reading the hub under the reloaded code")
        nxt = sloth.get("next")
        if nxt is None:
            s.append("- the first INIT after the reload wasn't run: the load didn't land")
            return
        no, nr = nxt["o"], nxt["res"]
        self.check(f"threads sloth: the first INIT on the reloaded code finds no {THREAD_NAME} thread and runs",
                   nxt["finished"] and nr.get("leftoverCountAtInit") == 0 and not no.get("crashed")
                   and bool(no.get("doneLine")) and no.get("pidBefore") == o.get("pidAfter"),
                   f"{nr.get('leftoverCountAtInit')} at INIT {nr.get('leftoverAtInit')}; {no.get('doneLine') or no.get('errorMessage')!r}; "
                   f"pid {o.get('pidAfter')} -> {no.get('pidBefore')}",
                   "after a deploySloth the next OpMode inherits the old code's poller, or doesn't run")
        s.append(f"- Sloth during the run: deploySloth took {deploy['seconds']:.0f} s; the OpMode was "
                 f"{after.get('status')} afterwards with TransferCounter polls {before_p} -> {after_p}; the load was "
                 f"staged while it ran ({sloth.get('stagedWhileRunning')}) and ended {load.get('result')} after STOP "
                 f"(push to end {load.get('endS')} s; lines {load.get('lines')})")

    def run_turret(self):
        previous = self.active_config()
        self.activate_config(CONFIG_TURRET, hub_config(turret_devices(self.imu_type)))
        try:
            self.contained("turret", self._turret)
        finally:
            if self.dash.fatal is None and self.active_config() != previous:
                self.activate_config(previous)

    def _turret(self):
        o = self.run_opmode(TURRET_OPMODE, "turret_pwm", init_hold=1.0, timeout=20, result_files=["turret_pwm"])
        res = self.pull("turret_pwm", required=False) or {}
        info, checks = res.get("info") or {}, res.get("checks") or {}
        s = ["## Bench: Turret PWM (BettaTurret's servo PWM through stop() and the write toggle, read from the hub)\n",
             f"- TurretL port {info.get('leftPort')} ({info.get('leftDirection')}, PWM {info.get('leftPwmRange')} us), "
             f"TurretR port {info.get('rightPort')} ({info.get('rightDirection')}, PWM {info.get('rightPwmRange')} us); "
             f"expected pulses [center L, center R, forward R] {res.get('expectedPulseUs')} us; "
             f"{info.get('stepSeconds')} s per step"]
        if o["crashed"]:
            s.append(f"- CRASHED: {o['errorMessage']}")
        for name, ph in [("INIT", res.get("init") or {})] + list((res.get("phases") or {}).items()):
            s.append(f"- {name}: {ph.get('loops')} loops, L on {ph.get('loopsLeftEnabled')}, R on "
                     f"{ph.get('loopsRightEnabled')}, SDK cache disagreeing with the hub {ph.get('loopsSdkCacheDisagrees')}; "
                     f"after the entry action {ph.get('afterEntryAction')}; last loop {ph.get('lastLoop')}")
        if not checks:
            self.check("turret pwm: the bench reported its checks", False,
                       f"crashed {o['crashed']}: {o.get('errorMessage')!r}; done line {o.get('doneLine')!r}",
                       "Bench: Turret PWM didn't finish its steps (see the error), so BettaTurret's PWM wasn't judged")
        for name, c in checks.items():
            if c.get("informational"):
                s.append(f"- informational, {'as expected' if c.get('pass') else 'NOT as expected'}: {name}: "
                         f"{c.get('detail')}")
                continue
            self.check(f"turret pwm: {name}", c.get("pass") is True, str(c.get("detail")),
                       TURRET_MEANS.get(name, TURRET_OTHER_MEANS))
        s.append("- SDK cache disagreeing: loops where isPwmEnabled() differs from the hub's own register, expected 0 "
                 "(the SDK returns its cached flag once set, so the bench reads the hub)\n")
        self.summary += s

    def answer(self, text):
        """ask() for a typed reply: what the person read off a meter or decided."""
        self.log(text)
        reply = input(">>> type it, then press Enter: ").strip()
        self.log(f"typed: {reply!r}")
        return reply

    def run_turret_zero(self):
        previous = self.active_config()
        tz = {"answers": {}, "runs": {},
              "v2": "asked" if self.args.turret_zero_v2 else "not asked for: --turret-zero-v2 with a REV-11-3174 on hand"}
        self.activate_config(CONFIG_TURRET_ZERO, hub_config(turret_zero_devices(self.imu_type)))
        try:
            self.contained("turret_zero", lambda: self._turret_zero(tz))
        finally:
            try:
                if tz.get("branchPlugged"):
                    self.ask(">>> UNPLUG the split cable's OctoQuad branch from OctoQuad channel 4 now, before the "
                             "OctoQuad or the Control Hub loses power: an unpowered OctoQuad's 3.3 V-only input must "
                             "not be driven.")
            finally:
                self._turret_zero_report(tz)
                if self.dash.fatal is None and self.active_config() != previous:
                    self.activate_config(previous)

    def tz_branch(self, tz, plugged, prompt):
        """Asks for the split cable's OctoQuad branch in or out, counting it as in from the moment it is asked for."""
        if plugged:
            tz["branchPlugged"] = True
        self.ask(prompt)
        if not plugged:
            tz["branchPlugged"] = False

    def _turret_zero(self, tz):
        a = tz["answers"]
        self.log(TZ_POWERS)
        self.ask(">>> UNPLUG everything from the Control Hub's motor power and servo ports, any pod from OctoQuad channel "
                 "4, and the turret encoder's cables from both hubs and the OctoQuad. The OctoQuad stays in Control Hub "
                 "I2C port 2.")
        a["cable1"] = self.answer(">>> METER the REV-31-1815 cable, unplugged, pin to pin against the OctoQuad channel 4 "
                                  "header's G, 3.3 V, A and B. Type ok if each pin reaches its own, else what's wrong.")
        a["split"] = self.answer(">>> METER the split cable, unplugged, pin to pin: the encoder plug's 3.3 V, A and B each "
                                 "reach the hub plug's own pin and nothing else; its absolute (pulse) pin reaches the "
                                 "OctoQuad branch's A pin and nothing else; the branch's 3.3 V and B pins reach nothing; "
                                 "ground joins all three plugs. Type ok if so, else what's wrong.")
        if not (tz_answer_ok(a["cable1"]) and tz_answer_ok(a["split"])):
            tz["stopped"] = "the cables didn't meter right"
            return
        self.ask(">>> PLUG the encoder (switch on A) into OctoQuad channel 4 with the REV-31-1815 cable, nothing in "
                 "Control Hub encoder port 2, and set its shaft on a dial mark.")
        self.tz_track(tz, "quad", "quad", None,
                      f">>> TURN the shaft {TZ_QUAD_TURNS} turns forward to the same mark and hold it a second, then "
                      f"{TZ_QUAD_TURNS} turns back to the mark and hold it a second.")
        self.ask(">>> UNPLUG the REV-31-1815 from the encoder and the OctoQuad. PLUG the encoder into the split cable and "
                 "its hub plug into Control Hub encoder port 2; leave its OctoQuad branch unplugged.")
        a["supplyV"] = self.answer(">>> METER the encoder's 3.3 V pin to ground. Type the volts.")
        a["highV"] = self.answer(TZ_HIGH_PROMPT)
        if not tz_levels_ok(a["supplyV"], a["highV"]):
            tz["stopped"] = "the encoder's levels don't suit the OctoQuad's 3.3 V input"
            return
        self.tz_branch(tz, True, ">>> PLUG the split cable's OctoQuad branch into OctoQuad channel 4 (signal on its A "
                                 "pin) and set the shaft on a dial mark.")
        turns = (f">>> TURN the shaft {TZ_SPLIT_TURNS} turns forward to the same mark and hold it a second, then "
                 f"{TZ_SPLIT_TURNS} turns back to the mark and hold it a second.")
        self.tz_track(tz, "split", "split_plugged", None, turns)
        self.tz_branch(tz, False, ">>> UNPLUG the split cable's OctoQuad branch and set the shaft on a dial mark.")
        self.tz_track(tz, "split", "split_unplugged", None, turns)
        self.tz_branch(tz, True, ">>> PLUG the OctoQuad branch back into OctoQuad channel 4.")
        self.tz_track(tz, "span", "span", None,
                      ">>> TURN the shaft slowly one way through 3 whole turns, taking about 20 s, and let go.")
        span = tz_span(dig(tz, "runs", "span", "result", "track", "rows") or [])
        if span is None or span["turns"] < TZ_MIN_TURNS:
            tz["stopped"] = "the span run didn't give the absolute output's range and the count's direction"
            return
        common = {"sign": span["sign"], "minUs": span["minUs"], "maxUs": span["maxUs"], "version": "V1"}
        self.ask(f">>> LET GO of the shaft and don't touch it until the next prompt ({TZ_REZERO_RUNS} INITs, about a "
                 "minute).")
        for i in range(1, TZ_REZERO_RUNS + 1):
            self.tz_init(tz, "init", f"init_{i:02d}", dict(common, reads=TZ_INIT_READS if i == 1 else 1))
        self.ask(">>> TURN the shaft about a quarter turn (not a whole turn) and let go.")
        self.tz_init(tz, "init", "init_moved", common)
        self.tz_track(tz, "agreement", "agreement", common,
                      f">>> TURN the shaft through at least {TZ_MIN_TURNS:g} turns, stopping on 12 or more dial marks "
                      "spread round the dial, about a second on each.")
        self.ask(">>> TURN the shaft about a quarter turn and let go.")
        self.tz_init(tz, "sign", "sign", common)
        self.ask(">>> Don't touch the shaft until the next prompt.")
        self.tz_init(tz, "reset", "reset", common)
        self.tz_init(tz, "reset_after", "reset_after", common)
        if self.args.turret_zero_v2:
            self.tz_v2(tz)
        a["labSupply"] = self.answer(">>> Is the Control Hub on a lab supply you can dip to about 6 V for half a second? "
                                     "Type yes or no.")
        if a["labSupply"].strip().lower() != "yes":
            tz["brownout"] = "no lab supply"
            return
        tz["brownout"] = "ran"
        self.ask(">>> TURN the shaft about a quarter turn and let go; don't touch it again until the bench ends.")
        self.tz_init(tz, "init", "brownout_before", common)
        self.tz_branch(tz, False, ">>> UNPLUG the split cable's OctoQuad branch: the dip can leave the OctoQuad "
                                  "unpowered while the encoder still drives its input.")
        try:
            self.tz_brownout(tz, common)
        except ProtocolError:
            raise
        except (BenchError, subprocess.TimeoutExpired) as e:
            tz["brownoutError"] = str(e)
            self.log(f"!! {TZ_OPMODE} brownout: {e}")
            self.wait_hub_back("the dip")
            self.settle()
        self.tz_branch(tz, True, ">>> PLUG the OctoQuad branch back into OctoQuad channel 4.")
        self.tz_init(tz, "init", "brownout_after", common)

    def tz_v2(self, tz):
        """The V2 on A, then on S, each metered with the branch out before it goes in, then the V1 back."""
        a, v2 = tz["answers"], {"version": "V2"}
        unplug = ">>> UNPLUG the split cable's OctoQuad branch from OctoQuad channel 4."
        plug = (">>> PLUG the split cable's OctoQuad branch into OctoQuad channel 4 (signal on its A pin) and set the "
                "V2's shaft on a dial mark.")
        self.tz_branch(tz, False, unplug)
        self.ask(">>> UNPLUG the V1 from the split cable and PLUG a REV-11-3174 (V2) in its place, switch on A; leave "
                 "the OctoQuad branch unplugged.")
        a["v2SupplyV"] = self.answer(">>> METER the V2's 3.3 V pin to ground. Type the volts.")
        a["v2HighV"] = self.answer(TZ_HIGH_PROMPT)
        if not tz_levels_ok(a["v2SupplyV"], a["v2HighV"]):
            tz["v2"] = "stopped: the V2's levels on A don't suit the OctoQuad's 3.3 V input"
        else:
            self.tz_branch(tz, True, plug)
            self.tz_track(tz, "v2", "v2_a", v2, ">>> TURN the shaft one turn forward to the same mark and hold it a "
                                                "second, then one turn back to the mark and hold it a second.")
            self.tz_branch(tz, False, unplug)
            self.ask(">>> SWITCH the V2 to S; leave the OctoQuad branch unplugged.")
            a["v2sHighV"] = self.answer(TZ_HIGH_PROMPT)
            if tz_levels_ok(a["v2SupplyV"], a["v2sHighV"]):
                self.tz_branch(tz, True, plug)
                self.tz_track(tz, "v2", "v2_s", v2,
                              ">>> TURN the shaft one turn forward to the mark and hold it a second.")
                self.tz_branch(tz, False, unplug)
        self.ask(">>> PUT the V1 back on the split cable, switch on A.")
        self.tz_branch(tz, True, ">>> PLUG the OctoQuad branch back into OctoQuad channel 4.")

    def tz_finish(self):
        self.adb.push_json({"finish": True}, TZ_FINISH, self.out / "logs" / TZ_FINISH)

    def tz_track(self, tz, phase, run, params, prompt, script=None):
        step = f"turret_zero_{run}"

        def ask_then_finish():
            if self.wait_or_crash(lambda: self.dash.data.get(KEY_PHASE) == phase, 15) is not None:
                self.ask(prompt)
                self.log(f"{TZ_OPMODE} {run}: {self.dash.data.get(KEY_STATUS)}")
            self.tz_finish()

        o = self.run_opmode(TZ_OPMODE, step, dict(params or {}, phase=phase, run=run), timeout=30,
                            during=script or ask_then_finish, result_files=[step])
        self.tz_record(tz, run, o, step)

    def tz_init(self, tz, phase, run, params):
        step = f"turret_zero_{run}"
        o = self.run_opmode(TZ_OPMODE, step, dict(params, phase=phase, run=run), start=False, timeout=30,
                            result_files=[step])
        self.tz_record(tz, run, o, step)

    def tz_record(self, tz, run, o, step):
        tz["runs"][run] = {"crashed": o["crashed"], "error": o.get("errorMessage") or "", "timedOut": o.get("timedOut"),
                           "appRestarted": o.get("appRestarted"), "pidBefore": o.get("pidBefore"),
                           "pidAfter": o.get("pidAfter"), "result": self.pull(step, required=False) or {}}

    def tz_brownout(self, tz, common):
        rec = {"warnings": [], "before": []}
        tz["brownoutRunner"] = rec

        def note():
            s = self.dash.status or {}
            for text in (s.get("warningMessage") or "", s.get("errorMessage") or ""):
                if text and text not in rec["before"] and text not in rec["warnings"]:
                    rec["warnings"].append(text)
            return False

        def dip():
            ready = self.wait_or_crash(lambda: self.dash.data.get(KEY_PHASE) == "brownout", 15)
            s = self.dash.get_status() or {}
            rec["before"] = [s.get("warningMessage") or "", s.get("errorMessage") or ""]
            if ready is not None:
                self.ask(">>> DIP the Control Hub's supply to about 6 V for half a second now, without touching the "
                         "shaft.")
                self.wait_or_crash(note, 5)
            self.tz_finish()

        self.tz_track(tz, "brownout", "brownout", common, None, script=dip)

    def _turret_zero_report(self, tz):
        checks, lines = turret_zero_checks(tz)
        runs = tz["runs"]
        info = dig(runs, "quad", "result", "info") or dig(runs, "split_plugged", "result", "info") or {}
        span = tz_span(dig(runs, "span", "result", "track", "rows") or [])
        rests = dig(runs, "agreement", "result", "track", "rests") or []
        first_pulse = {r: dig(v, "result", "bank", "firstPlausibleMs") for r, v in runs.items() if r != "quad"}
        self.summary += ["## Bench: Turret Zero (a Through Bore's count on Control Hub encoder port 2 against its absolute "
                         "output on OctoQuad channel 4)\n",
                         f"- {TZ_POWERS}",
                         f"- OctoQuad: chip id {fmt_chip(info.get('chipId'))}, firmware {info.get('firmware')}, bank as "
                         f"found {info.get('bankAsFound')}, channel 4 pulse-width params {info.get('pulseWidthParams')}, "
                         f"wrap tracking {info.get('tracksWrapAsFound')}; every run puts the bank back to "
                         f"{OCTOQUAD_QUADRATURE}",
                         f"- typed: {tz['answers']}" + (f"; stopped: {tz['stopped']}" if tz.get("stopped") else ""),
                         f"- span (the count against the unwrapped absolute output): {span}",
                         f"- ms from setting the bank to the first plausible channel 4 pulse, per run: {first_pulse}",
                         f"- agreement rests [count, absolute deg, error deg]: "
                         f"{[[r.get('count'), r.get('absoluteDeg'), r.get('errorDeg')] for r in rests]}"] + lines + [""]
        for check in checks:
            self.check(*check)
        self.results["turret_zero"] = {k: v for k, v in tz.items() if k != "runs"}

    def run_edge(self):
        rec = {}
        self.gp_seq = 0
        hold = GamepadHold(self, EDGE_INIT_HOLD, GP_SEND_EVERY_S, left_bumper=True)

        def in_init():
            rec["readySeconds"] = self.wait_or_crash(lambda: self.dash.data.get("gp phase") is not None, 15)
            if rec["readySeconds"] is None:
                return
            self.gp_mark(EDGE_INIT_IDLE)
            # The SDK swaps in its own empty pads once, about 500 ms into INIT: start the hold after that.
            self.gp_hold(EDGE_INIT_IDLE, 1.0, GP_SEND_EVERY_S)
            hold.start()
            rec["holdSeenSeconds"] = self.wait_or_crash(
                lambda: telemetry_float(self.dash.data.get("gp phase")) == EDGE_INIT_HOLD
                and str(self.dash.data.get("gp lb")).lower() == "true", 3)
            self.wait_or_crash(lambda: False, EDGE_INIT_HOLD_S)

        def running():
            if hold.started():
                hold.phase = EDGE_RUN_HOLD
                self.wait_or_crash(lambda: False, EDGE_RUN_HOLD_S)
                hold.stop()
                self.gp_hold(EDGE_RUN_RELEASE, 1.0, GP_SEND_EVERY_S)
                self.gp_hold(EDGE_RUN_PRESS, 0.5, GP_SEND_EVERY_S, left_bumper=True)
                self.gp_hold(EDGE_RUN_RELEASE_AGAIN, 0.5, GP_SEND_EVERY_S)
            self.gp_hold(GP_DONE, 0.3, GP_SEND_EVERY_S)

        try:
            o = self.run_opmode(EDGE_OPMODE, "edge_start", timeout=15, timeout_ok=True, during_init=in_init,
                                during=running, result_files=["edge_start"])
        finally:
            hold.done.set()
        hold.stop()
        rec["hold"] = hold.summary()
        self.results["edge_start"]["runner"] = rec
        res = self.pull("edge_start", required=False) or {}
        checks = res.get("checks") or {}
        s = ["## Bench: Edge Through Start (gamepad1 LB held from INIT through START, BioBuzz Tele's pattern)\n",
             f"- runner: the suppliers were built {rec.get('readySeconds')} s after INIT was seen; the hold showed "
             f"{rec.get('holdSeenSeconds')} s after it began; the background sender sent {rec['hold']['sends']} states, "
             f"longest gap {rec['hold']['maxGapSeconds']} s (slothboard rests the pads after 0.5 s)",
             f"- first running loop: {res.get('firstRunLoop')}"]
        for key, ph in (res.get("phases") or {}).items():
            s.append(f"- {key}: {ph.get('loops')} loops, {ph.get('lbLoops')} with LB, LB rose {ph.get('lbRises')} fell "
                     f"{ph.get('lbFalls')}; invalidated supplier {ph.get('spentRises')} edges, control "
                     f"{ph.get('controlRises')}")
        if o["crashed"]:
            s.append(f"- CRASHED: {o['errorMessage']}")
        if not checks:
            self.check("edge through start: the bench reported its checks", False,
                       f"crashed {o['crashed']}: {o.get('errorMessage')!r}; done line {o.get('doneLine')!r}",
                       "Bench: Edge Through Start didn't finish (see the error), so the START edge wasn't judged")
        setup = {n: c for n, c in checks.items() if n.startswith("setup:")}
        for n, c in setup.items():
            self.check(f"edge through start: {n}", c.get("pass") is True, str(c.get("detail")), EDGE_SETUP_MEANS)
        if checks and all(c.get("pass") is True for c in setup.values()):
            for n, c in checks.items():
                if n in setup:
                    continue
                means = (EDGE_CONTROL_MEANS if n.startswith("control:") else EDGE_START_MEANS if "START" in n
                         else EDGE_PRESS_MEANS)
                self.check(f"edge through start: {n}", c.get("pass") is True, str(c.get("detail")), means)
        elif checks:
            s.append("- the edge checks were skipped: the hold broke, so they would say nothing about the fix")
        s.append(f"- the unfixed pattern (a supplier first read after START) "
                 + ("fired at START: the hazard reproduced" if res.get("controlEdgeAtStart") else
                    "did not fire at START: the hazard didn't reproduce, so the fix's PASS proves less") + "\n")
        self.summary += s

    def run_freeze_dump(self):
        name = self.active_config()
        rec = {"config": name, "runs": []}
        self.results["freeze_dump_switch"] = rec

        def control():
            pid = self.adb.pid()
            self.dash.send({"type": "SET_HARDWARE_CONFIG", "hardwareConfigName": name})
            t0 = time.monotonic()
            while time.monotonic() - t0 < FREEZE_CONTROL_S and self.adb.pid() == pid:
                time.sleep(1)
            after = self.adb.pid()
            try:
                status = self.dash.get_status() if after == pid else {}
            except BenchError as e:
                self.log(f"no robot status after the INIT switch ({e})")
                status = {}
            rec["control"] = {"pidBefore": pid, "pidAfter": after, "activeOpMode": status.get("activeOpMode"),
                              "status": status.get("activeOpModeStatus")}

        o = self.run_opmode(FREEZE_OPMODE, "freeze_dump", start=False, timeout=15, during_init=control,
                            result_files=["freeze_dump", "freeze_watch"])
        c = rec.get("control")
        if o["crashed"] or not c:
            rec["notTried"] = f"Bench: Freeze Dump crashed in INIT ({o.get('errorMessage')!r}), so no change was sent"
        else:
            ignored = (c.get("pidBefore") is not None and c.get("pidAfter") == c.get("pidBefore")
                       and c.get("activeOpMode") == FREEZE_OPMODE and not o["appRestarted"])
            self.check("freeze dump: slothboard ignores a hardware-config switch while an OpMode is in INIT", ignored,
                       f"{c}; the step's app restart {o['appRestarted']}", FREEZE_GUARD_MEANS)
            if not ignored:
                rec["notTried"] = ("the switch sent during INIT already restarted the robot, taking the watch with it, "
                                   "so no change from idle was sent")
            else:
                self._freeze_runs(name, rec)
        self._freeze_report(rec, o)

    def _freeze_runs(self, name, rec):
        counts = {}
        for i, kind in enumerate(FREEZE_TRIGGERS):
            counts[kind] = counts.get(kind, 0) + 1
            run = {"id": f"{kind}_{counts[kind]}", "kind": kind, "message": FREEZE_MESSAGES[kind]}
            rec["runs"].append(run)
            try:
                if i > 0:
                    armed = self.run_opmode(FREEZE_OPMODE, f"freeze_arm_{run['id']}", start=False, timeout=15,
                                            result_files=["freeze_dump", "freeze_watch"])
                    if armed["crashed"]:
                        run["notSent"] = f"re-arming the watch crashed: {armed.get('errorMessage')!r}"
                        continue
                self._freeze_trigger(name, run)
                self._freeze_collect(run)
            except ProtocolError:
                raise
            except BenchError as e:
                run["failed"] = str(e)
                self.log(f"!! freeze dump {run['id']}: {e}; sending no more changes")
                self.settle()
                return

    def _freeze_trigger(self, name, run):
        self.wait_idle()
        run["listedAsActive"] = self.dash.fresh_active_config()
        if run["kind"] == "write":
            contents = self.adb.read_config(name)
            run["target"] = f"{name}, written with its own contents ({len(contents)} chars)"
            message = {"type": run["message"], "hardwareConfigName": name, "hardwareConfigContents": contents}
        elif run["kind"] == "delete":
            scratch = self.config_copy(CONFIG_FREEZE_SCRATCH, hub_config([]))
            self.adb.run("push", str(scratch), f"{CONFIG_DIR}/{CONFIG_FREEZE_SCRATCH}.xml")
            run["target"] = f"{CONFIG_FREEZE_SCRATCH}, pushed for it and never active"
            message = {"type": run["message"], "hardwareConfigName": CONFIG_FREEZE_SCRATCH}
        else:
            run["target"] = name
            message = {"type": run["message"], "hardwareConfigName": name}
        pid = self.adb.pid()
        hub_t0 = self.adb.hub_time()
        anr = self.adb._raw(["-s", self.adb.serial, "shell", "ls -l /data/anr/"], timeout=30)
        folder = self.out / "raw" / "freeze" / run["id"]
        folder.mkdir(parents=True, exist_ok=True)
        run.update(pidBefore=pid, hubT0=hub_t0, mark=f"BENCH freeze dump {run['id']}: {run['message']} from idle",
                   anrDirBefore=(anr.stdout or anr.stderr).strip()[:400], pidGoneSeconds=None, newPidSeconds=None,
                   setupInProcessSeconds=None, forcedRelaunch=False, restartFailed=False, traces=None)
        self.adb.mark(run["mark"])
        t0 = time.monotonic()
        self.dash.send(message)
        run["sent"] = True
        while time.monotonic() - t0 < FREEZE_RELAUNCH_WAIT_S:
            self.adb.ensure_connected()
            elapsed = round(time.monotonic() - t0, 1)
            lines = self._freeze_lines(run)
            self._freeze_copy_traces(run, lines, folder, elapsed)
            if any(RESTART_FAILED in line for line in lines):
                run["restartFailed"] = True
                break
            if any(ROBOT_SETUP_DONE in line and logcat_pid(line) == pid for line in lines):
                run["setupInProcessSeconds"] = elapsed
                break
            now_pid = self.adb.pid()
            if now_pid is None:
                if run["pidGoneSeconds"] is None:
                    run["pidGoneSeconds"] = elapsed
            elif now_pid != pid:
                run["newPidSeconds"] = elapsed
                break
            time.sleep(1)
        # The relaunched app's own startup ANR rewrites the traces file about 25 s after it starts.
        self._freeze_copy_traces(run, self._freeze_lines(run), folder, round(time.monotonic() - t0, 1))
        if run["newPidSeconds"] is None and run["setupInProcessSeconds"] is None and not run["restartFailed"]:
            self.log(f"!! the app is still frozen {FREEZE_RELAUNCH_WAIT_S} s after the {run['message']} and the Control "
                     "Hub's watchdog hasn't relaunched it: force-stopping and starting it")
            self.adb.run("shell", f"am force-stop {RC_PACKAGE}; am start -n {RC_LAUNCHER}", check=False)
            run["forcedRelaunch"] = True
        if run["setupInProcessSeconds"] is None and not run["restartFailed"]:
            self.wait_restart(f"the freeze dump's {run['id']}")
        else:
            self.wait_ready()
        run["pidAfter"] = self.note_pid()
        run["readySeconds"] = round(time.monotonic() - t0, 1)
        if run["traces"] is None:
            self._freeze_copy_traces(run, None, folder, run["readySeconds"])
        run["activeAfter"] = self.active_config()
        if run["kind"] == "delete":
            run["scratchLeft"] = CONFIG_FREEZE_SCRATCH in self.adb.config_names()
        self.log(f"freeze dump {run['id']}: pid {pid} -> {run['pidAfter']}, ready {run['readySeconds']} s after the "
                 f"{run['message']}")
        if run["activeAfter"] != name:
            self.log(f"!! {run['activeAfter']} is active after the {run['message']}, not {name}: re-activating {name}")
            self.activate_config(name)

    def _freeze_lines(self, run):
        hits = self.adb.logcat_matches(run["hubT0"], run["mark"], ROBOT_SETUP_DONE, RESTART_FAILED, FREEZE_TRACES_WROTE)
        return lines_after(hits, run["mark"])

    def _freeze_copy_traces(self, run, lines, folder, elapsed):
        if run.get("traces") is not None:
            return
        if lines is not None and not any(FREEZE_TRACES_WROTE in line and logcat_pid(line) == run["pidBefore"]
                                         for line in lines):
            return
        r = self.adb._raw(["-s", self.adb.serial, "shell", f"cat {FREEZE_TRACES}"], timeout=120)
        text = r.stdout if r.returncode == 0 else ""
        (folder / "anr_traces.txt").write_text(text)
        sections = traces_sections(text)

        def after_change(when):
            seconds = logcat_delta(run["hubT0"], when)
            return seconds is not None and seconds >= -1

        since = [sec for sec in sections if sec[0] == run["pidBefore"] and after_change(sec[1])]
        info = {"copiedSecondsAfterChange": elapsed,
                "when": "late, once the app was back" if lines is None else "as soon as that pid logged its dump",
                "chars": len(text), "error": None if text.strip() else (r.stderr or r.stdout).strip()[:200],
                "processes": [f"pid {pid} at {when[:-4]}" for pid, when, _ in sections]}
        if since:
            pid, when, section = since[-1]
            (folder / "anr_traces_frozen_pid.txt").write_text(section)
            info.update(frozenPidDump=f"pid {pid} at {when[:-4]}", lockWaits=traces_lock_waits(section))
        run["traces"] = info

    def _freeze_collect(self, run):
        folder = self.out / "raw" / "freeze" / run["id"]
        log_text = self._save_log(f"logs/freeze_dump_{run['id']}.log", run["hubT0"])
        after = lines_after(log_text, run["mark"])
        run["logLines"] = [line for line in after if any(m in line for m in FREEZE_LOG_LINES)][:80]
        run["deadlineExpired"] = any(FREEZE_DEADLINE in line for line in after)
        run["apRelaunch"] = any(FREEZE_AP_TAG in line and FREEZE_AP_RELAUNCH in line for line in after)
        run["sdkKilled"] = any(APP_KILLED in line for line in after)
        names = self.adb.bench_shell(f"ls {self.adb.bench_dir()}", check=False).split()
        run["dumpFiles"] = sorted(n for n in names if n.startswith("freeze_dump_") and n.endswith(".txt"))
        run["pulled"] = []
        for n in run["dumpFiles"] + [n for n in ("freeze_watch.json",) if n in names]:
            try:
                self.adb.pull_bench_file(n, folder / n)
                run["pulled"].append(n)
            except BenchError as e:
                self.log(f"couldn't copy {n} from the hub: {e}")
        stop_file = f"{self.adb.bench_dir()}/{FREEZE_STOP_FILE}"
        self.adb.bench_shell(f"touch {stop_file}", check=False)
        time.sleep(1.5)
        self.adb.bench_shell(f"rm -f {stop_file}", check=False)

    @staticmethod
    def _freeze_outcome(run):
        if run.get("notSent") or not run.get("sent"):
            return "not tried"
        if run.get("failed"):
            return "the runner lost it"
        if run.get("restartFailed"):
            return "restart failed"
        if run.get("forcedRelaunch"):
            return "FROZE, and no watchdog relaunch came, so the runner force-stopped it"
        if run.get("pidAfter") != run.get("pidBefore") or run.get("deadlineExpired") or run.get("apRelaunch"):
            return "FROZE"
        if run.get("setupInProcessSeconds") is not None:
            return "restarted in process"
        return "unclear: same pid, but no robot setup seen"

    @staticmethod
    def _freeze_detail(run, with_id=True):
        why = run.get("notSent") or run.get("failed")
        return ((f"{run['id']}: " if with_id else "") + run["outcome"] + (f" ({why})" if why else "")
                + (f"; pid {run.get('pidBefore')} -> {run.get('pidAfter')}; set up in the same pid after "
                   f"{run.get('setupInProcessSeconds')} s; new pid after {run.get('newPidSeconds')} s; ready "
                   f"{run.get('readySeconds')} s; '{FREEZE_DEADLINE}' {run.get('deadlineExpired')}; the Control Hub's "
                   f"'{FREEZE_AP_RELAUNCH}' {run.get('apRelaunch')}" if run.get("sent") else ""))

    def _freeze_report(self, rec, o):
        for run in rec["runs"]:
            run["outcome"] = self._freeze_outcome(run)
        for kind in dict.fromkeys(FREEZE_TRIGGERS):
            planned = FREEZE_TRIGGERS.count(kind)
            runs = [r for r in rec["runs"] if r["kind"] == kind]
            detail = "; ".join(self._freeze_detail(r) for r in runs) or f"not sent: {rec.get('notTried', 'no run')}"
            if 0 < len(runs) < planned:
                detail += f"; {planned - len(runs)} of {planned} not sent (an earlier run failed)"
            self.check(f"freeze dump: {FREEZE_MESSAGES[kind]} from idle restarts the robot in the same process "
                       f"({planned} run{'s' if planned > 1 else ''})",
                       len(runs) == planned and all(r["outcome"] == "restarted in process" for r in runs), detail,
                       FREEZE_RESTART_MEANS)
        s = ["## Bench: Freeze Dump (slothboard's hardware-config changes from idle, with a thread-dump watch armed)\n",
             f"- {', '.join(FREEZE_TRIGGERS)} on {rec['config']}, each from idle with the watch armed again first. The "
             "outcome comes from the pid, the robot setup in it, the SDK's "
             f"'{FREEZE_DEADLINE}' and the Control Hub's '{FREEZE_AP_RELAUNCH}', never from the watch's dumps: every "
             "robot setup waits 5 s for USB between two RC alive broadcasts, so a clean restart dumps too"]
        if rec.get("notTried"):
            s.append(f"- not tested: {rec['notTried']}")
        first_freeze = None
        for run in rec["runs"]:
            index_file = self.out / "raw" / "freeze" / run["id"] / "freeze_watch.json"
            index = json.loads(index_file.read_text()) if index_file.exists() else {}
            events = index.get("events") or []
            dumps = [e for e in events if e.get("type") == "dump"]
            stalls = [e for e in dumps if str(e.get("detail", "")).startswith(FREEZE_ALIVE_STALL)]
            s.append(f"- {run['id']}: {run['message']} {run.get('target')} (slothboard listed {run.get('listedAsActive')} "
                     f"as active): {self._freeze_detail(run, with_id=False)}; {run.get('activeAfter')} active afterwards"
                     + (f"; {CONFIG_FREEZE_SCRATCH} still on the hub {run.get('scratchLeft')}" if run["kind"] == "delete"
                        else ""))
            if not run.get("sent"):
                continue
            s.append(f"  - watch: {len(dumps)} dumps ({len(stalls)} on a stopped RC alive broadcast) in "
                     f"raw/freeze/{run['id']}/; SIGQUIT sent {index.get('sigquitWallMs') is not None}"
                     + (f"; the watch failed: {index.get('failure')}" if index.get("failure") else ""))
            traces = run.get("traces") or {}
            if traces.get("frozenPidDump"):
                s.append(f"  - ART's SIGQUIT dump of {traces['frozenPidDump']}, from during the change, copied "
                         f"{traces['when']} ({traces['copiedSecondsAfterChange']} s after the change): lock waits "
                         f"{traces.get('lockWaits') or 'none'}")
            elif traces:
                s.append(f"  - {FREEZE_TRACES} copied {traces['when']} holds no dump of pid {run['pidBefore']} from "
                         f"after the change (it holds {traces.get('processes')}), so no lock owners: the watch's dumps "
                         "show frames only")
            else:
                s.append(f"  - pid {run['pidBefore']} never logged '{FREEZE_TRACES_WROTE}', so {FREEZE_TRACES} wasn't "
                         "copied")
            s += [f"  - `{line[:220]}`" for line in run.get("logLines", [])[:12]]
            if first_freeze is None and run["outcome"].startswith("FROZE") and dumps:
                first_freeze = (run, dumps)
        if first_freeze is not None:
            run, dumps = first_freeze
            folder = self.out / "raw" / "freeze" / run["id"]
            texts = [(n, (folder / n).read_text()) for n in run.get("pulled") or [] if n.endswith(".txt")]
            s.append(f"- {run['id']}, suspects in the last dump: {dumps[-1].get('suspects')}")
            if texts:
                last_name, last = texts[-1]
                start = last.find("=== suspects")
                end = last.find("\n=== snapshot", start)
                section = last[start:end] if start >= 0 else ""
                s.append(f"- {run['id']}/{last_name}, suspects section:\n\n```\n{section[:6000].rstrip()}\n```")
                found = {h: any(h in t for _, t in texts) for h in FREEZE_HYPOTHESIS}
                s.append(f"- the source's prediction (a NanoHTTPD thread in FtcDashboard.attemptRestart holding the "
                         f"activeOpMode lock while EventLoop blocks in FtcDashboard.onOpModePostStop): {found}")
        s.append("")
        self.summary += s

    def message_counter(self):
        return MessageCounter(*self.link.address(DASHBOARD_PORT))

    def dash_off_listeners(self):
        ports = DASH_OFF_PROXY_PORTS + DASH_OFF_RECORDED_PORTS
        found = {entry["port"] for entry in self.adb.listeners(ports)}
        return {p: p in found for p in ports}

    def read_dashboard_switch(self):
        r = self.adb._raw(["-s", self.adb.serial, "shell", f"run-as {RC_PACKAGE} cat {DASHBOARD_PREFS}"])
        text = r.stdout if r.returncode == 0 else r.stdout + r.stderr
        if r.returncode != 0 and "No such file" not in text:
            raise BenchError(f"run-as can't read the app's {DASHBOARD_PREFS}: {text.strip()[:200]}")
        return dashboard_auto_enable(text)

    def run_dashboard_off(self):
        rec = {"planned": (["battery"] if self.args.interactive else []) + ["restart", "reboot", "reinstall"],
               "phases": [], "limelight": self.limelight is not None}
        self.results["dashboard_off"] = rec
        if not self.switch_with_adb:
            raise BenchError(f"{DASH_OFF_OPMODE} needs run-as (the Run button's debuggable APK): while the dashboard is "
                             "off it ignores every command but status, so its app restarts and its recovery go over "
                             "adb. Nothing was switched off.")
        if DASH_OFF_OPMODE not in {n for n, _ in self.dash.opmodes or []}:
            raise BenchError(f"The hub doesn't have {DASH_OFF_OPMODE}. Do a full install with the IDE's Run button "
                             "(TeamCode configuration), or bare_hub.sh, so it is on the hub. Nothing was switched off.")
        try:
            self._dash_off_baseline(rec)
            self._dash_off_switch(rec)
            for name in rec["planned"]:
                self._dash_off_phase(rec, name)
        except BenchError as e:
            rec["failed"] = str(e)
            raise
        finally:
            try:
                self._dash_off_recover(rec)
            finally:
                self._dash_off_report(rec)

    def _dash_off_baseline(self, rec):
        b = rec["baseline"] = {"statusEnabled": self.dash.get_status().get("enabled"),
                               "autoEnable": self.read_dashboard_switch(), "listeners": self.dash_off_listeners()}
        self.log(f"dashboard off, before the switch: {b}")
        if b["statusEnabled"] is not True:
            raise BenchError("the dashboard wasn't on before the switch, so the bench can't tell what switching it off "
                             "changes")

    def _dash_off_until(self, predicate, t0, limit):
        """Seconds from t0 until predicate() held, or None once limit s have passed."""
        while True:
            if predicate():
                return round(time.monotonic() - t0, 2)
            if time.monotonic() - t0 > limit:
                return None
            time.sleep(0.25)

    def _dash_off_switch(self, rec):
        s = rec["switch"] = {}
        step = "dashboard_off"
        self.wait_idle()
        self.adb.bench_shell(f"rm -f {self.adb.bench_dir()}/{step}.json {self.adb.bench_dir()}/{step}.json.tmp")
        self.adb.push_json({"confirm": DASH_OFF_CONFIRM, "onSeconds": DASH_OFF_ON_S, "graceSeconds": DASH_OFF_GRACE_S,
                            "offSeconds": DASH_OFF_WINDOW_S, "periodMs": DASH_OFF_PERIOD_MS},
                           "params.json", self.out / "logs" / "params.json")
        hub_t0 = self.adb.hub_time()
        self.adb.mark(f"begin {step}")
        self.step_pid()
        done = None
        finished = False
        counter = self.message_counter()
        try:
            self.begin_watch(DASH_OFF_OPMODE, hub_t0)
            self.dash.send({"type": "INIT_OP_MODE", "opModeName": DASH_OFF_OPMODE})
            self.dash.wait_until(lambda: self._status_is(DASH_OFF_OPMODE) or self.crashed(), 30,
                                 f"{DASH_OFF_OPMODE} to INIT", poll_status_every=0.5, tick=self.adb_crash_tick)
            if self.crashed():
                raise BenchError(f"{DASH_OFF_OPMODE} crashed in INIT, before switching anything off: "
                                 f"{self.user_crash() or self.dash.status.get('errorMessage') or self.watch['crashSignal']}")
            t_start = time.monotonic()
            self.dash.send({"type": "START_OP_MODE"})
            off_s = self._dash_off_until(lambda: self.dash.get_status().get("enabled") is False
                                         or self.dash.status["activeOpMode"] == IDLE_OPMODE, t_start,
                                         DASH_OFF_ON_S + DASH_OFF_SWITCH_WAIT_S)
            s["offSeen"] = off_s is not None and self.dash.status.get("enabled") is False
            if not s["offSeen"]:
                raise BenchError(f"the dashboard didn't report itself off within {DASH_OFF_ON_S + DASH_OFF_SWITCH_WAIT_S:.0f} "
                                 f"s of START (last status {self.dash.status})")
            off_at = t_start + off_s
            s["offSecondsAfterStart"] = off_s
            s["onCounts"] = window_counts(counter.events(), t_start, off_at)
            deadline = off_at + DASH_OFF_GRACE_S + DASH_OFF_WINDOW_S + DASH_OFF_DONE_WAIT_S
            while done is None and time.monotonic() < deadline:
                time.sleep(1.0)
                done = self.logcat_done(hub_t0, [step])
                if done is None and self.dash.get_status()["activeOpMode"] == IDLE_OPMODE:
                    done = self.logcat_done(hub_t0, [step])
                    break
            t_end = time.monotonic()
            s["offCounts"] = window_counts(counter.events(), off_at + DASH_OFF_GRACE_S, t_end)
            s["socketOpen"] = counter.open_at(t_end)
            s["doneLine"] = done
            finished = True
        finally:
            counter.close()
            if not finished:
                self._save_log(f"logs/{step}.log", hub_t0)
        # The dashboard ignores STOP while off, so the OpMode ends itself after its off stretch.
        self.dash.wait_until(lambda: self.dash.status and self.dash.status["activeOpMode"] != DASH_OFF_OPMODE,
                             DASH_OFF_DONE_WAIT_S, f"{DASH_OFF_OPMODE} to end by itself", poll_status_every=0.5)
        log_text = self._save_log(f"logs/{step}.log", hub_t0)
        s["result"] = self.pull(step, required=False)
        s["listeners"] = self.dash_off_listeners()
        self.log(f"dashboard off, the switch: {s}")
        if done is None:
            raise BenchError(f"{DASH_OFF_OPMODE} ended without BENCH DONE: "
                             f"{crash_text(logcat_messages(step_log(log_text, step), OPMODE_MANAGER_TAG)) or 'no error logged'}"
                             f" (logs/{step}.log)")

    def _dash_off_phase(self, rec, name):
        p = {"name": name}
        rec["phases"].append(p)
        t0 = time.monotonic()
        try:
            if name == "battery":
                self.ask(">>> PULL the Control Hub's battery lead now, count to three, plug it back in, then press Enter. "
                         f"The runner then waits for the hub to boot ({self.link.reboot_note}).")
                self._dash_off_wait_boot(t0, DASH_OFF_PHASES[name])
            elif name == "restart":
                self.activate_config(self.active_config())
            elif name == "reboot":
                self.adb.run("reboot", check=False, timeout=60)
                self._dash_off_wait_boot(t0, DASH_OFF_PHASES[name])
            else:
                self._dash_off_reinstall()
        except BenchError as e:
            p["error"] = str(e)
            raise
        p["backSeconds"] = round(time.monotonic() - t0, 1)
        self.restarts.append({"during": f"{DASH_OFF_OPMODE}: {DASH_OFF_PHASES[name]}", "pid": self.note_pid(),
                              "waitedSeconds": p["backSeconds"]})
        p.update(self._dash_off_observe())
        self.log(f"dashboard off, after {DASH_OFF_PHASES[name]}: {p}")

    def _dash_off_observe(self):
        o = {"statusEnabled": self.dash.get_status().get("enabled"), "autoEnable": self.read_dashboard_switch()}
        counter = self.message_counter()
        try:
            t0 = time.monotonic()
            self.dash.send({"type": "INIT_OP_MODE", "opModeName": "Bench: Probe"})
            time.sleep(DASH_OFF_WINDOW_S)
            t1 = time.monotonic()
            o["window"] = window_counts(counter.events(), t0, t1)
            o["socketOpen"] = counter.open_at(t1)
        finally:
            counter.close()
        status = self.dash.get_status()
        o["initIgnored"] = status["activeOpMode"] == IDLE_OPMODE
        if status["activeOpMode"] not in ("", IDLE_OPMODE):
            self._stop(status["activeOpMode"], o)
        o["listeners"] = self.dash_off_listeners()
        o["dashPage"] = self.link.http("GET", RC_PAGE_PORT, "/dash").get("code")
        return o

    def _dash_off_wait_setup(self, hub_t0, what):
        t0 = time.monotonic()
        while ROBOT_SETUP_DONE not in self.adb.logcat_matches(hub_t0, ROBOT_SETUP_DONE):
            if time.monotonic() - t0 > CONFIG_SWITCH_TIMEOUT_S:
                raise BenchError(f"no robot setup ('{ROBOT_SETUP_DONE}') within {CONFIG_SWITCH_TIMEOUT_S} s of {what}")
            time.sleep(DASH_OFF_POLL_S)

    def _dash_off_wait_app(self, deadline, what):
        while True:
            try:
                self.dash.ensure_connected(timeout=max(5.0, deadline - time.monotonic()))
                self.wait_ready()
                return
            except ProtocolError:
                raise
            except BenchError as e:
                if time.monotonic() > deadline:
                    raise BenchError(f"the Robot Controller app wasn't ready after {what}: {e}")

    def _dash_off_wait_boot(self, t0, what):
        """Waits until adb answers from a hub whose uptime says it booted after t0, then for the app."""
        self._dash_off_wait_app(self.wait_hub_back(what, booted_after=t0), what)

    def wait_hub_back(self, what, booted_after=None):
        """Waits until adb answers, from a hub whose uptime says it booted after booted_after when that's given, else
        with the app's process up, and sets the log buffer again (a reboot resets it); returns the deadline it used."""
        deadline = time.monotonic() + DASH_OFF_BOOT_WAIT_S
        last = "no answer yet"
        while True:
            time.sleep(DASH_OFF_POLL_S)
            if time.monotonic() > deadline:
                raise BenchError(f"the hub wasn't back from {what} within {DASH_OFF_BOOT_WAIT_S:.0f} s ({last}); "
                                 f"{self.link.lost_hint}")
            try:
                self.adb.ensure_connected()
                pid = self.adb.pid() if booted_after is None else None
            except OtherHub:
                raise
            except (BenchError, subprocess.TimeoutExpired) as e:
                last = str(e)
                continue
            if booted_after is None:
                if pid is not None:
                    break
                last = "the Robot Controller app isn't running"
                continue
            uptime = self.adb.clock().get("uptimeS")
            if uptime is not None and uptime < time.monotonic() - booted_after + 1:
                break
            last = f"its uptime is {uptime} s, so it hasn't restarted yet"
        self.adb.run("shell", f"logcat -G {LOG_BUFFER}", check=False)
        return deadline

    def _dash_off_reinstall(self):
        out = self.adb.run("shell", f"pm path {RC_PACKAGE}")
        paths = [line.strip()[len("package:"):] for line in out.splitlines() if line.strip().startswith("package:")]
        if not paths:
            raise BenchError(f"pm path found no installed Robot Controller APK to reinstall: {out.strip()[:200]}")
        folder = self.out / "logs" / "installed_apk"
        folder.mkdir(parents=True, exist_ok=True)
        local = [str(folder / Path(p).name) for p in paths]
        try:
            for remote, dest in zip(paths, local):
                self.adb.run("pull", remote, dest, timeout=900)
            hub_t0 = self.adb.hub_time()
            t0 = time.monotonic()
            command = ["install", "-r", "-t", local[0]] if len(local) == 1 else ["install-multiple", "-r", "-t"] + local
            r = self.adb._raw(["-s", self.adb.serial] + command, timeout=DASH_OFF_INSTALL_TIMEOUT_S)
        finally:
            for dest in local:
                Path(dest).unlink(missing_ok=True)
        if "Success" not in r.stdout:
            raise BenchError(f"adb install -r of the hub's own APK didn't succeed: {(r.stdout + r.stderr).strip()[-300:]}")
        self.adb.run("shell", f"am start -n {RC_LAUNCHER}", check=False)
        self._dash_off_wait_setup(hub_t0, "the reinstall")
        self._dash_off_wait_app(t0 + DASH_OFF_BOOT_WAIT_S, "the reinstall")

    def _dash_off_recover(self, rec):
        r = rec["recovery"] = {"command": self.dashboard_recovery()}
        self.dashboard_left_off = r["command"]
        self.log(f"turning the dashboard back on over adb: {r['command']}")
        try:
            self.adb.ensure_connected()
            hub_t0 = self.adb.hub_time()
            seq = self.dash.opmode_seq
            t0 = time.monotonic()
            r["output"] = self.adb.run("shell", DASHBOARD_RECOVERY_SHELL, check=False).strip()[-300:]
            self._dash_off_wait_setup(hub_t0, "the recovery's app restart")
            self.dash.ensure_connected(timeout=DASH_OFF_RECOVERY_S)
            r["enabledSeconds"] = self._dash_off_until(lambda: self.dash.get_status().get("enabled") is True, t0,
                                                       DASH_OFF_RECOVERY_S)
            r["statusEnabled"] = self.dash.status.get("enabled")
            r["opModeListSeconds"] = self._dash_off_until(
                lambda: self.dash.opmode_seq > seq and DASH_OFF_OPMODE in {n for n, _ in self.dash.opmodes or []}, t0,
                DASH_OFF_RECOVERY_S)
            self.wait_ready()
            self.restarts.append({"during": f"{DASH_OFF_OPMODE}: the recovery", "pid": self.note_pid(),
                                  "waitedSeconds": round(time.monotonic() - t0, 1)})
            counter = self.message_counter()
            try:
                t_probe = time.monotonic()
                self.run_opmode("Bench: Probe", "dashboard_off_recovery_probe", timeout=60, result_files=["probe"],
                                during=lambda: time.sleep(DASH_OFF_PROBE_S))
                r["probePackets"] = window_counts(counter.events(), t_probe, time.monotonic())["telemetry packets"]
                r["firstPacketSeconds"] = first_packet_after(counter.events(), t0)
            finally:
                counter.close()
            r["listeners"] = self.dash_off_listeners()
            r["autoEnable"] = self.read_dashboard_switch()
            if rec["limelight"]:
                r["limelightHttp"] = self.link.http("GET", LIMELIGHT_WEB_PORT, "/")
        except OtherHub as e:
            r["error"] = str(e)
            raise
        except BenchError as e:
            r["error"] = str(e)
            raise BenchError(f"the adb recovery failed ({e}); the dashboard may still be off. "
                             f"{self.adb_port_caution()} Then turn it back on with {r['command']}") from e
        finally:
            if r.get("statusEnabled") is True:
                self.dashboard_left_off = None
            else:
                self.log(f"!! THE DASHBOARD MAY STILL BE OFF. {self.adb_port_caution()} Then turn it back on with: "
                         f"{r['command']}")

    def _dash_off_report(self, rec):
        for name, ok, detail, means in dashboard_off_checks(rec):
            self.check(name, ok, detail, means)

        def listening(d):
            return [p for p, up in (d.get("listeners") or {}).items() if up] if d.get("listeners") is not None else "?"

        def row(label, enabled, auto, counts, init, d):
            counts = counts or {}
            others = {k: v for k, v in (counts.get("types") or {}).items()
                      if k not in ("RECEIVE_TELEMETRY", "RECEIVE_IMAGE", "RECEIVE_LOGCAT_ERRORS", "RECEIVE_LOGCAT_LINES")}
            return (f"| {label} | {enabled} | {auto} | {counts.get('telemetry packets', '')} | {counts.get('images', '')} | "
                    f"{counts.get('logcat', '')} | {others or ''} | {init} | {listening(d)} |")

        base, switch, r = rec.get("baseline") or {}, rec.get("switch") or {}, rec.get("recovery") or {}
        result = switch.get("result") or {}
        s = [f"## {DASH_OFF_OPMODE} (switched off, then {', '.join(DASH_OFF_PHASES[n] for n in rec['planned'])}, then "
             "the adb recovery)\n",
             f"- Every window is {DASH_OFF_WINDOW_S:.0f} s on a second websocket; while off, the runner also sends an INIT "
             "for Bench: Probe at the start of each window, which the dashboard must ignore. The proxy ports are judged "
             f"only where they listened before the switch.",
             f"- The OpMode: isEnabled {result.get('enabledAtStart')} -> {result.get('enabledAfterDisable')} "
             f"(disable() took {result.get('disableMs')} ms); it sent {result.get('onPackets')} packets and its camera "
             f"source served {result.get('onFramesServed')} frames while on, {result.get('offPackets')} packets and "
             f"{result.get('offFramesServed')} frames after the switch",
             "", "| when | status enabled | saved autoEnable | telemetry packets | images | logcat | other messages | "
                 "INIT ignored | listening |", "|---|---|---|---|---|---|---|---|---|",
             row("before (the OpMode, on)", base.get("statusEnabled"), base.get("autoEnable"), switch.get("onCounts"), "",
                 base),
             row("the switch (the OpMode, off)", False if switch.get("offSeen") else "?", "", switch.get("offCounts"), "",
                 switch)]
        for p in rec.get("phases") or []:
            s.append(row(f"after {DASH_OFF_PHASES[p['name']]}" + (f" (FAILED: {p['error']})" if p.get("error") else ""),
                         p.get("statusEnabled"), p.get("autoEnable"), p.get("window"), p.get("initIgnored"), p))
        s.append("")
        s.append(f"- Recovery (`{r.get('command')}`): " + (f"FAILED: {r['error']}; " if r.get("error") else "")
                 + f"enabled after {r.get('enabledSeconds')} s, OpMode list after {r.get('opModeListSeconds')} s, "
                   f"first Probe packet {r.get('firstPacketSeconds')} s after the command, listening {listening(r)}"
                 + (f", Limelight web UI {r.get('limelightHttp')}" if rec.get("limelight") else ""))
        if self.dashboard_left_off:
            s.append(f"- **The dashboard may still be off. {self.adb_port_caution()} Then turn it back on with "
                     f"`{self.dashboard_left_off}`**")
        s.append("")
        self.summary += s

    def run_config_by_name(self):
        s = ["## Config by name (shipped configs saved with a resource id this install gives another XML)\n"]
        shipped = (self.probe or {}).get("shippedConfigs") or {}
        pair = self.config_by_name_pair(shipped)
        if not self.switch_with_adb or pair is None:
            why = ("run-as can't write the app's settings, so the runner can't save a resource id" if not self.switch_with_adb
                   else "this install ships no two res/xml configs with Control Hub devices alone and different device "
                        f"names (shipped: {sorted(shipped)})")
            self.log(f"config by name not tested: {why}")
            self.summary += s + [f"- not tested: {why}\n"]
            return
        (a, a_devices), (b, b_devices) = pair
        results = {}
        try:
            for case, name, accept, battery in (("wrong_id", a, {a}, True),
                                                ("unshipped", CONFIG_UNSHIPPED, {CONFIG_UNSHIPPED, NO_CONFIG}, False)):
                entry = {"name": name, "resourceId": shipped[b], "location": "RESOURCE", "isDirty": False}
                self.activate_config(name, config=entry, accept=accept, battery=battery)
                boot = self.out / "logs" / self.switches[-1]["log"]
                boot_text = boot.read_text() if boot.exists() else ""
                self.run_opmode("Bench: Probe", f"config_by_name_{case}", timeout=60, result_files=["probe"])
                res = self.pull("probe", local_name=f"probe_config_by_name_{case}")
                devices = set(res.get("devices") or [])
                results[case] = {"saved": entry, "devices": sorted(devices),
                                 "loaded": self.loaded_xml(devices, a, a_devices, b, b_devices),
                                 "settingAtProbe": res.get("activeConfig"),
                                 "setupLog": [line.strip()[:220] for line in boot_text.splitlines()
                                              if CONFIG_RENUMBERED in line or CONFIG_FELL_BACK in line][:4]}
        finally:
            if self.dash.fatal is None and self.active_config() != CONFIG_MAIN:
                self.activate_config(CONFIG_MAIN, self.main_config())
        self.results["config_by_name"] = dict(results, configs={a: sorted(a_devices), b: sorted(b_devices)})
        wrong, unshipped = results["wrong_id"], results["unshipped"]
        self.check(f"config by name: {a} saved with {b}'s resource id loads res/xml/{a}", wrong["loaded"] == a,
                   f"saved {wrong['saved']}; loaded {wrong['loaded']} (devices {wrong['devices']}); the app's setting "
                   f"at the probe {wrong['settingAtProbe']}; setup logged {wrong['setupLog'] or 'nothing about it'}",
                   CONFIG_BY_NAME_HELD)
        self.check(f"config by name: a name this build doesn't ship ({CONFIG_UNSHIPPED}) loads no config",
                   unshipped["loaded"] == "no config",
                   f"saved {unshipped['saved']}; loaded {unshipped['loaded']} (devices {unshipped['devices']}); the "
                   f"app's setting at the probe {unshipped['settingAtProbe']}; setup logged "
                   f"{unshipped['setupLog'] or 'nothing about it'}", CONFIG_BY_NAME_HELD)
        s += [f"- res/xml/{a} has {sorted(a_devices)}, res/xml/{b} has {sorted(b_devices)}; this install numbers them "
              f"{shipped[a]:#010x} and {shipped[b]:#010x}",
              f"- {a} saved as {shipped[b]:#010x}: loaded {wrong['loaded']}",
              f"- {CONFIG_UNSHIPPED} saved as {shipped[b]:#010x}: loaded {unshipped['loaded']}",
              "- Each case writes the app's setting over adb and restarts it, then Bench: Probe lists the hardware map; "
              f"the runner switches back to {CONFIG_MAIN} afterwards\n"]
        self.summary += s

    @staticmethod
    def config_by_name_pair(shipped):
        candidates = sorted((len(d), n, d) for n, d in ((n, shipped_hub_devices(n)) for n in shipped) if d)
        for i, (_, a, a_devices) in enumerate(candidates):
            for _, b, b_devices in candidates[i + 1:]:
                if a_devices - b_devices and b_devices - a_devices and shipped[a] != shipped[b]:
                    return (a, a_devices), (b, b_devices)
        return None

    @staticmethod
    def loaded_xml(devices, a, a_devices, b, b_devices):
        only_a, only_b = a_devices - b_devices, b_devices - a_devices
        if only_a <= devices and not only_b & devices:
            return a
        if only_b <= devices and not only_a & devices:
            return b
        if not (a_devices | b_devices) & devices:
            return "no config"
        return f"neither {a} nor {b} cleanly"

    def run_clock(self):
        before = self.adb.clock()
        connect = self.results.get("hubClock", {}).get("atConnect") or {}
        year = time.gmtime(before["epochS"]).tm_year if "epochS" in before else None
        unset = year is not None and year <= CLOCK_UNSET_YEAR
        interactive = self.args.interactive and unset
        if self.args.interactive and not unset:
            self.log(f"the hub's clock is already set ({year}): skipping the RC web page step; to see the jump, "
                     "power-cycle the hub and open neither a Driver Station nor the RC's web pages before the run")
        page, page_skipped = self.link.rc_page() if interactive else (None, None)
        if page_skipped:
            self.log(f"!! skipping the RC web page step: {page_skipped}")
            interactive = False
        timeout_ms = CLOCK_INTERACTIVE_TIMEOUT_MS if interactive else CLOCK_TIMEOUT_MS

        def open_rc_page():
            self.wait_or_crash(lambda: False, 3)
            self.ask(f">>> Open {page} in the Mac's browser now: the Robot Controller's own page, "
                     "which sets the hub's clock while it is unset. Wait for it to load, then press Enter.")

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
        self.summary += ["## Bench: Clock (PathCommands.timeout against the hub's wall clock)\n",
                         f"- the hub's clock read {year} before the step: "
                         + ("unset since boot, so the first Driver Station heartbeat or RC web page sets it about 56 "
                            "years forward" if unset else "already set (a Driver Station or the RC's web page set it "
                                                          "since boot, or the hub kept time)"),
                         f"- PathCommands.timeout({timeout_ms} ms) ended {ended} ms after START by System.nanoTime(); "
                         f"biggest one-loop wall-clock step {res.get('biggestLoopStepMs')} ms; jumps {res.get('jumps')}"
                         + ("; you opened the RC's web page during it" if interactive else "")
                         + (f"; the RC web page step was skipped: {page_skipped}" if page_skipped else ""),
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

        def script():
            ready = self.wait_or_crash(lambda: status() == "READY"
                                       and (telemetry_float(self.dash.data.get(KEY_LOOP)) or 0) > 20, 15)
            if ready is None:
                rec["skipped"] = f"the Pinpoint wasn't READY before the unplug (status {status()!r}), or the OpMode crashed"
                self.log(f"Pinpoint dropout skipped: {rec['skipped']}")
                return
            self.ask(">>> PULL the Pinpoint's I2C cable out of the Control Hub (leave both pods in), then press Enter.")
            rec["notReadySeenS"] = self.wait_or_crash(lambda: status() not in (None, "READY"), 10)
            self.log(f"Pinpoint status {status()!r} on the dashboard {rec['notReadySeenS']} s after Enter")
            self.ask(">>> PLUG the Pinpoint's I2C cable back in, then press Enter.")
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
        seed, samples = dig(ph, "update", "setPose"), dig(ph, "update", "poseSamples") or []
        faults = [f for f in dig(ph, "update", "faults") or [] if "pinpoint" in f.lower()]
        error = o.get("errorMessage") or ""

        def kept(pose):
            turn = abs((pose[2] - seed[2] + math.pi) % (2 * math.pi) - math.pi)
            return (math.hypot(pose[0] - seed[0], pose[1] - seed[1]) <= PINPOINT_ABSENT_POSE_IN
                    and math.degrees(turn) <= PINPOINT_ABSENT_TURN_DEG)

        held = bool(seed) and bool(samples) and all(kept(p) for p in samples)
        named = "pinpoint" in error.lower()
        self.check("pinpoint absent: a missing Pinpoint is reported as a fault or keeps the pose setPose set",
                   held or bool(faults) or named,
                   f"setPose {seed}; poses every 50 updates {dig(ph, 'update', 'posesEvery50')}; Pinpoint faults "
                   f"{faults or 'none'} (all faults {dig(ph, 'update', 'faults')}); crashed {o['crashed']}: {error!r}",
                   PINPOINT_ABSENT_MEANS if samples else "Bench: Pinpoint Absent recorded no pose (see the error), so "
                   "this wasn't judged")
        self.summary += ["## Pinpoint absent (can the real autos run on a bare hub?)\n",
                         f"- crashed: {o['crashed']} {error}",
                         f"- create: {ph.get('create')}",
                         f"- update: {fmt(dig(ph, 'update', 'updateMs'))}; threw {dig(ph, 'update', 'threw')}; after "
                         f"setPose {seed}, poses {dig(ph, 'update', 'posesEvery50')}; faults after the updates "
                         f"{dig(ph, 'update', 'faults')}\n"]

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
        if (o["crashed"] and not bank_phase_crashed(o, res)) or not res.get("info"):
            self.log("Bench: OctoQuad didn't get through INIT: skipping the OctoQuad unplug step")
            self.summary.append("_OctoQuad unplug and replug skipped: Bench: OctoQuad didn't get through INIT._\n")
            return
        self._octoquad_dropout()

    def _octoquad_seed(self, o, res):
        info, seed, cost = res.get("info") or {}, res.get("seed") or {}, res.get("readCostMs") or {}
        s = [f"## Bench: OctoQuad (GuardedOctoQuadLocalizer on Control Hub I2C bus {OCTOQUAD_BUS}, follower idle)\n",
             f"- chip id {fmt_chip(res.get('chipId'))}, firmware {res.get('firmware')}; status right after the localizer "
             f"was built {info.get('statusAfterCreate')} ({info.get('createMs')} ms to build; the guard doesn't wait for "
             f"RUNNING, so statusChanges says when it came); heading axis {info.get('headingAxis')}",
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
        bank_crash = bank_phase_crashed(o, res)
        if o["crashed"]:
            s.append(f"- CRASHED{' in the bank phase, after the seed phase finished' if bank_crash else ''}: "
                     f"{o['errorMessage']}")
        elif not info:
            s.append(f"- INIT never finished (timed out {o.get('timedOut')}); error {o.get('errorMessage')!r}")
        s.append("")
        self.summary += s
        stuck = None if bank_crash else OCTOQUAD_CRASHED if o["crashed"] else OCTOQUAD_HUNG if not info else None
        offset, turn = seed.get("maxOffsetIn"), seed.get("maxTurnDeg")
        held = (stuck is None and seed.get("ran") is True and offset is not None and turn is not None
                and offset <= PINPOINT_SEED_OFFSET_IN and turn <= PINPOINT_SEED_TURN_DEG)
        self.check(f"octoquad: a start pose set in INIT survives through START + 3 s (within {PINPOINT_SEED_OFFSET_IN} in "
                   f"and {PINPOINT_SEED_TURN_DEG} deg)", held,
                   f"worst {offset} in / {turn} deg over {seed.get('loops')} loops; final status {res.get('finalStatus')}"
                   + (f"; error {o['errorMessage']!r}" if o["crashed"] and not bank_crash else ""),
                   stuck or OCTOQUAD_SEED_PENDING)
        crc, invalid = res.get("crcFailures"), res.get("invalidLoops")
        self.check("octoquad: every loop's localizer read was valid (CRC ok and RUNNING) after the first RUNNING read",
                   stuck is None and bool(seed.get("loops")) and crc == 0 and invalid == 0,
                   f"{crc} CRC failures and {invalid} invalid loops over {seed.get('loops')} loops; loops per status "
                   f"{res.get('statusLoops')}", stuck or OCTOQUAD_VALID_PENDING)
        checks, lines = octoquad_bank_checks(res.get("bank"), o.get("errorMessage") or "" if o["crashed"] else "")
        self.summary += ["### Bench: OctoQuad's bank phase (OctoQuad channels 4-7 in pulse-width mode, last)\n"] + lines + [""]
        for check in checks:
            self.check(*check)

    def _octoquad_dropout(self):
        rec = {"warnings": []}

        def valid():
            warning = (self.dash.status or {}).get("warningMessage") or ""
            if warning and warning not in rec["warnings"]:
                rec["warnings"].append(warning)
            value = self.dash.data.get(KEY_OCTOQUAD_VALID)
            return None if value is None else str(value)

        def script():
            ready = self.wait_or_crash(lambda: valid() == "true"
                                       and (telemetry_float(self.dash.data.get(KEY_LOOP)) or 0) > 20, 15)
            if ready is None:
                rec["skipped"] = (f"the OctoQuad's localizer reads weren't valid before the unplug (status "
                                  f"{self.dash.data.get(KEY_OCTOQUAD_STATUS)!r}), or the OpMode crashed")
                self.log(f"OctoQuad dropout skipped: {rec['skipped']}")
                return
            self.ask(">>> PULL the OctoQuad's I2C cable out of the Control Hub (leave both pods in), then press Enter.")
            rec["invalidSeenS"] = self.wait_or_crash(lambda: valid() == "false", 10)
            self.log(f"OctoQuad status {self.dash.data.get(KEY_OCTOQUAD_STATUS)!r}, valid {valid()!r} on the dashboard "
                     f"{rec['invalidSeenS']} s after Enter")
            self.ask(">>> PLUG the OctoQuad's I2C cable back in, then press Enter.")
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
        stopped = ended_itself and OCTOQUAD_GUARD_PREFIX in error.lower()
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
        watch = FaultWatch(OCTOQUAD_FAULT_KEY)
        self.dash.packet_hook = watch.on_packet
        try:
            o = self.run_opmode(opmode, step, init_hold=OCTOQUAD_ABSENT_HOLD_S, expect_crash=True,
                                timeout=OCTOQUAD_ABSENT_WAIT_S, timeout_ok=True, result_files=[step])
        finally:
            self.dash.packet_hook = None
        res = self.pull(step, required=False) or {}
        self.results[step]["faultWatch"] = watch.summary()
        err = o.get("errorMessage") or ""
        create_ms, fault_ms, fault_message = res.get("createMs"), res.get("faultMs"), res.get("faultMessage") or ""
        faulted = res.get("initLoopFaultedMs") or {}
        after_start = o.get("crashSeenAfterStartSeconds")
        self.check(f"octoquad absent: building the localizer returns within {OCTOQUAD_ABSENT_CREATE_MS} ms",
                   create_ms is not None and create_ms <= OCTOQUAD_ABSENT_CREATE_MS,
                   f"{create_ms} ms; INIT seen after {o.get('initSeenSeconds')} s", OCTOQUAD_ABSENT_HUNG)
        self.check(f"octoquad absent: the {OCTOQUAD_FAULT_KEY} fault is raised in INIT within "
                   f"{OCTOQUAD_ABSENT_EXPECT_S} s and names the OctoQuad",
                   fault_ms is not None and fault_ms <= OCTOQUAD_ABSENT_EXPECT_S * 1000
                   and OCTOQUAD_GUARD_PREFIX in fault_message.lower(),
                   f"after {fault_ms} ms: {fault_message!r}", OCTOQUAD_ABSENT_NO_FAULT)
        self.check("octoquad absent: INIT keeps looping with the fault up",
                   faulted.get("count", 0) > 0 and faulted.get("p95", float("inf")) <= 50
                   and faulted.get("max", float("inf")) <= 500,
                   f"INIT loop with the fault up {fmt(faulted)}", OCTOQUAD_ABSENT_SLOW)
        if o.get("dashboardSilent") or watch.bench_packets == 0:
            dashboard = ("dashboard row check not measured: "
                         f"{o.get('dashboardSilent') or 'no bench packets reached the runner'}")
        else:
            dashboard = (f"FAULT row first in {watch.fault_first} of {watch.with_fault} packets that carry it "
                         f"({watch.bench_packets} bench packets)")
            self.check("octoquad absent: the FAULT row leads every dashboard packet that carries it",
                       watch.with_fault > 0 and watch.fault_first == watch.with_fault,
                       f"{dashboard}; not first: {watch.fault_not_first}", OCTOQUAD_ABSENT_NO_FAULT)
        self.check("octoquad absent: START stops the OpMode with an error naming the OctoQuad",
                   o["crashed"] and after_start is not None and OCTOQUAD_GUARD_PREFIX in err.lower(),
                   f"crashed {o['crashed']}, seen by the runner {after_start} s after it sent START (its crash "
                   f"detection lag included): {err!r}", OCTOQUAD_ABSENT_STARTED)
        self.summary += [
            "## Bench: OctoQuad Absent (GuardedOctoQuadLocalizer with nothing on the bus)\n",
            f"- config line on the Control Hub: `{octoquad_tag(OCTOQUAD_ABSENT_BUS)}`; chip id read there "
            f"{fmt_chip(res.get('chipId'))}",
            f"- localizer built in {create_ms} ms; fault after {fault_ms} ms ({res.get('initLoopsBeforeFault')} INIT "
            f"loops): {fault_message!r}; INIT loop with the fault up {fmt(faulted)}",
            f"- {dashboard}",
            f"- START after {OCTOQUAD_ABSENT_HOLD_S} s in INIT: crashed {o['crashed']}, seen by the runner {after_start} "
            f"s after it sent START: "
            f"{err!r}\n"]

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
        self.ask(EXPANSION_MOVE)
        try:
            self.activate_config(CONFIG_EXPANSION_ODO, self.main_config(
                expansion=(address, [pinpoint_tag(1), octoquad_tag(OCTOQUAD_BUS)])))
            o_eh, eh = self._expansion_hub("eh")
        except BenchError:
            self.ask(f">>> MOVE {EXPANSION_HOME}, then press Enter.")
            raise
        except BaseException:
            self.log(f"!! the run is stopping: move {EXPANSION_HOME} before the next run")
            raise
        self.ask(f">>> MOVE {EXPANSION_HOME}, then press Enter.")
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

            deploy = self._deploy_sloth(run_id, send_init if when == "push" else None)
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

    def gradle_command(self, *tasks):
        return [str(REPO_ROOT / "gradlew"), *tasks, "--console=plain"] + (["--offline"] if self.args.gradle_offline else [])

    def _deploy_sloth(self, run_id, on_pushed, timeout_s=None):
        """Runs deploySloth, calling on_pushed the moment it prints that the jar is on the hub. deploySloth then
        waits for the hub's lock file with no timeout of its own, so it is killed after timeout_s
        (SLOTH_DEPLOY_TIMEOUT_S by default)."""
        self.adb.mark(f"{SLOTH_DEPLOY_MARK} {run_id}")
        if self.link.usb:
            # The Load plugin's adb connect 192.168.43.1 runs only when `adb devices` lists nothing (AutoConnect.MAINTAIN).
            self.adb.ensure_connected()
        t0 = time.monotonic()
        proc = subprocess.Popen(self.gradle_command(":TeamCode:deploySloth"), cwd=REPO_ROOT,
                                env=dict(os.environ, ANDROID_SERIAL=self.adb.serial),
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
        lines = []
        pushed_at = None
        list_mark = None
        connection = None
        hung = threading.Event()
        watchdog = threading.Timer(timeout_s or self.SLOTH_DEPLOY_TIMEOUT_S, lambda: (hung.set(), proc.kill()))
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
            # A session's time limit sends Ctrl-C to the runner alone, and deploySloth would wait on the lock unbounded.
            if proc.poll() is None:
                proc.kill()
                proc.wait()
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

    def restore_target(self):
        """(name, settings entry or None, config XML to write or None, whether the entry is the original verbatim)."""
        if self.original and not self.original_shipped:
            return self.original, None, self.original_xml if self.original in BENCH_CONFIGS else None, False
        shipped = self.original_shipped and self.shipped_config()
        if shipped:
            if shipped["resourceId"] != self.original_entry.get("resourceId"):
                self.note_restore(
                    f"{self.original} was saved as resource {self.original_entry.get('resourceId')}, which this install "
                    f"numbers {shipped['resourceId']}: re-activated by name, which fixes this hub only (see the "
                    "config-by-name checks)")
            return self.original, shipped, None, False
        if self.switch_with_adb:
            return self.original_entry.get("name", NO_CONFIG), dict(self.original_entry, isDirty=False), None, True
        return CONFIG_PROBE, None, hub_config([]), False

    def note_restore(self, text):
        if text not in self.restore_notes:
            self.restore_notes.append(text)

    def restore_config(self):
        if not self.configs_touched:
            self.log("no hardware config was changed; nothing to restore")
            return
        self.log("restoring the original hardware config")
        target, entry, xml, verbatim = self.restore_target()
        # A setting naming a config this build doesn't have makes the app fall back to no config at its next setup.
        accept = {target, NO_CONFIG} if verbatim else {target}
        if verbatim and target != NO_CONFIG:
            self.log(f"writing back the original setting {entry}; if this build can't load {target}, the hub shows no "
                     "config until REINSTALL.sh puts the old app back")
        if xml is not None:
            self.log(f"writing back {target}.xml as it was before the run (original_config.xml): the run wrote its own "
                     "bench config under that name")
            self.note_restore(f"{target}.xml written back with its contents from before the run")
        try:
            self.dash.recover_for_restore()
            self.settle()
            if self.active_config() not in accept or entry is not None or xml is not None:
                self.activate_config(target, xml, entry, accept=accept, battery=NO_CONFIG not in accept)
            if not self.args.keep_configs and self.remove_bench_configs(keep=target):
                # slothboard lists configs from memory and re-reads the folder on a restart.
                self.activate_config(target, config=entry, accept=accept, battery=NO_CONFIG not in accept)
        except Exception as e:
            if not self.switch_with_adb:
                raise
            self.log(f"!! restoring through the dashboard failed ({type(e).__name__}: {e}); restoring over adb alone")
            self.restore_notes.append(f"restored over adb alone after {type(e).__name__}: {e}")
            self.restore_over_adb(target, entry, xml)
            return
        self.log(f"hardware config {self.active_config()} is active")

    def restore_over_adb(self, target, entry, xml):
        hub_t0 = self.adb.hub_time()
        if xml is not None:
            self.adb.run("push", str(self.config_copy(target, xml)), f"{CONFIG_DIR}/{target}.xml")
        self.adb.stage_active_config(target, self.out / "logs" / "rc_prefs.xml", entry)
        self.adb.switch_app()
        t0 = time.monotonic()
        while ROBOT_SETUP_DONE not in self.adb.logcat_matches(hub_t0, ROBOT_SETUP_DONE):
            if time.monotonic() - t0 > CONFIG_SWITCH_TIMEOUT_S:
                self.log(f"!! no robot setup within {CONFIG_SWITCH_TIMEOUT_S} s of restoring over adb")
                break
            time.sleep(2)
        if not self.args.keep_configs:
            for n in BENCH_CONFIGS:
                if n != target and n in self.adb.config_names():
                    self.adb.remove_config(n)
        self.log(f"hardware config {self.adb.active_config()} is active (restored over adb alone)")

    def copy_bench_folder(self):
        folder = self.out / "raw" / "hub_bench_folder"
        if self.adb.private:
            folder.mkdir(parents=True, exist_ok=True)
            names = self.adb.bench_shell(f"ls {PRIVATE_BENCH_DIR}").split()
            for n in names:
                try:
                    (folder / n).write_text(self.adb.read_bench_file(n))
                except BenchError as e:
                    self.log(f"couldn't copy {n} from the hub: {e}")
            self.log(f"copied the hub's bench folder ({len(names)} files)")
            self.adb.run("shell", f"rmdir {HUB_BENCH_DIR}", check=False)
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
            self.log(f"!! {self.original} isn't shipped in this install; its setting is written back as it was, which "
                     "the app that shipped it finds again after REINSTALL.sh")
            return None
        return {"name": self.original, "resourceId": ids[self.original], "location": "RESOURCE", "isDirty": False}

    def write_summary(self, error=None):
        lines = [f"# Bench results {self.out.name}\n", f"Command: `{' '.join(sys.argv)}`\n",
                 f"Link: {self.link.describe(self.adb.serial)}\n"]
        reach = self.results.get("usbHubPorts")
        if reach:
            lines.append("Hub ports a forward reaches, judged from the hub's own socket table at connect (a connect from "
                         "this Mac can't tell: adb accepts it before asking the hub): "
                         + "; ".join(f"{p} {'yes' if ok else 'no' if ok is False else 'not judged'} ({why})"
                                     for p, (ok, why) in reach.items()) + "\n")
        if error is not None:
            lines.append(f"**Restore failed: {error}**\n" if error.startswith("restoring the config failed")
                         else f"**Run stopped early: {error}**\n")
        if self.dashboard_left_off:
            lines.append(f"**The dashboard may still be off on the hub. {self.adb_port_caution()} Then turn it back on "
                         f"with `{self.dashboard_left_off}`**\n")
        server = self.results.get("adbServer")
        if server is not None:
            server["atEnd"] = adb_port_record(server["atStart"]["port"])
            change = adb_server_change(server["atStart"], server["atEnd"])
            if change:
                lines.append(change + "\n")
        lines.append(f"Original hardware config setting: {json.dumps(self.original_entry)}"
                     + (" (from before the install)" if self.args.original_entry else "") + "\n")
        if self.restore_notes:
            lines.append(f"Restore: {'; '.join(self.restore_notes)}\n")
        if self.switches:
            killed = sum(w["appKilled"] for w in self.switches)
            over_adb = sum(w["overAdb"] for w in self.switches)
            lines.append(f"Config switches the runner made between steps: {len(self.switches)} ("
                         + ("all over adb" if over_adb == len(self.switches) else f"{over_adb} over adb, the rest through the dashboard")
                         + "); "
                         f"{killed} of them froze the app until the SDK or the Control Hub's watchdog killed it; "
                         f"per-switch logcat in logs/config_*.\n")
        freeze_runs = [r for r in (self.results.get("freeze_dump_switch") or {}).get("runs", []) if r.get("sent")]
        if freeze_runs:
            froze = sum(1 for r in freeze_runs if str(r.get("outcome", "")).startswith("FROZE"))
            lines.append(f"Bench: Freeze Dump's own dashboard config changes, sent on purpose and not counted above: "
                         f"{len(freeze_runs)}; {froze} froze the app until it was killed and relaunched (its section has "
                         f"each one).\n")
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
            distinct = len({(e.get("timestamp"), str(e.get("message", ""))) for e in self.dash.opmode_log})
            lines.append(f"slothboard pushed {distinct} distinct OpModeManager log lines to the runner (its crash "
                         f"signal), {len(self.dash.opmode_log)} counting the old lines its logcat monitor sends "
                         f"again each time it restarts.\n")
        if self.checks:
            lines.append("## Checks\n")
            lines += [f"- {state} {name}: {detail}" + (f" (FAIL means: {means})" if state == "FAIL" and means else "")
                      for name, state, detail, means in self.checks]
            lines.append("")
        lines += self.summary
        lines.append("Raw JSON is in raw/, per-step logcat in logs/, the runner's log in run.log.")
        (self.out / "summary.md").write_text("\n".join(lines) + "\n")
        (self.out / "runner_results.json").write_text(
            json.dumps(dict(self.results, configSwitches=self.switches, link=self.link.record()), indent=1,
                       default=list))

    def run(self):
        error = None
        unexpected = None
        try:
            if self.args.interactive and not sys.stdin.isatty():
                raise BenchError("--interactive asks you to drive the dashboard's Gamepad view, to unplug and replug "
                                 "the Limelight, the Pinpoint and the OctoQuad, to move the odometry cables to the "
                                 "Expansion Hub and back, and to open the RC's web page, so it needs a terminal")
            if self.args.turret_zero and not sys.stdin.isatty():
                raise BenchError("--turret-zero asks you to meter its cables, recable the encoder and turn its shaft, "
                                 "so it needs a terminal")
            if self.args.list_configs:
                self.connect_adb()
                self.list_configs()
            elif self.args.probe_only:
                self.connect()
                self.run_probe()
                self.record_hardware()
            else:
                self.port_statements()
                self.connect()
                self.run_probe()
                self.contained("saved_configs", self.run_saved_configs)
                wanted = self.selection()
                self.activate_config(CONFIG_MAIN, self.main_config())
                for name in bench_order():
                    if name in wanted:
                        self.log(f"=== {name} ===")
                        self.guarded(name, self.bench_step(name))
        except BenchError as e:
            error = str(e)
            self.log(f"STOPPED: {e}")
        except BaseException as e:
            # A runner bug or Ctrl-C: still put the config back and write what finished, then re-raise.
            unexpected = e
            error = f"{type(e).__name__}: {e}"
            self.log(f"STOPPED by {error}\n{traceback.format_exc()}")
        finally:
            # A second Ctrl-C or a closed terminal mid-restore would leave the hub on a bench config with no summary.
            signal.signal(signal.SIGINT, signal.SIG_IGN)
            signal.signal(signal.SIGHUP, signal.SIG_IGN)
            try:
                if self.dash is not None and self.adb.other_hub is not None:
                    self.log(f"not putting the hardware config back: {self.adb.other_hub}")
                    self.note_restore(f"not put back: {self.adb.other_hub}")
                elif self.dash is not None:
                    self.log("putting the hardware config back; Ctrl-C is ignored until that's done")
                    failure = None
                    deadline = time.monotonic() + RESTORE_WINDOW_S
                    attempt = 0
                    while True:
                        attempt += 1
                        try:
                            # restore_config opens the dashboard at the hub's address before its first adb command.
                            if attempt > 1 or self.adb.expect_serialno is not None:
                                self.adb.ensure_connected()
                            self.restore_config()
                            failure = None
                            if attempt > 1:
                                self.restore_notes.append(f"restored on attempt {attempt}")
                            break
                        except OtherHub as e:
                            failure = e
                            self.log(f"restore attempt {attempt}: {e}")
                            break
                        except Exception as e:
                            failure = e
                            self.log(f"restore attempt {attempt} failed: {type(e).__name__}: {e}\n{traceback.format_exc()}")
                            if time.monotonic() + RESTORE_RETRY_S > deadline:
                                break
                            time.sleep(RESTORE_RETRY_S)
                    if failure is not None:
                        error = (error + "; " if error else "") + f"restoring the config failed: {type(failure).__name__}: {failure}"
                        self.log(f"RESTORE FAILED: {failure}")
                        if isinstance(failure, OtherHub):
                            self.log(f"The hub the bench ran on still has its bench config. Reach that hub again, open "
                                     f"its dashboard, add the Hardware Config view, and activate "
                                     f"{self.original or 'your usual config'}.")
                        else:
                            self.log(self.link.restore_by_hand(self.adb.serial, self.adb_port_caution(),
                                                               self.original or "your usual config"))
                    try:
                        self.copy_bench_folder()
                    except Exception as e:
                        self.log(f"couldn't copy the hub's bench folder: {type(e).__name__}: {e}")
            finally:
                try:
                    if self.link.usb:
                        if self.dash is not None:
                            self.dash.close()
                        self.link.close()
                finally:
                    self.write_summary(error)
            if self.dash is not None:
                self.dash.close()
        failed = [c for c in self.checks if c[1] == "FAIL"]
        print()
        print(f"Results: {self.out}")
        print(f"Summary: {self.out / 'summary.md'}")
        print("Nothing to send back: Claude reads that folder directly. Just say the run finished.")
        if self.dashboard_left_off:
            print(f"\nTHE DASHBOARD MAY STILL BE OFF ON THE HUB. {self.adb_port_caution()}\n"
                  f"Turn it back on with:\n{self.dashboard_left_off}")
        if unexpected is not None:
            raise unexpected
        if error or failed:
            print(f"{len(failed)} check(s) failed" + (f"; the run stopped early: {error}" if error else ""))
            return 1
        return 0

    def guarded(self, name, step):
        """A bench that times out or loses its result is recorded as failed and the run goes on; a protocol
        surprise, or the hub becoming unreachable, stops everything."""
        if self.contained(name, step):
            return
        # A Photon OpMode leaves its hubs swapped until the map is rebuilt, even under CONFIG_MAIN.
        if name == "photon" or (name not in ("pinpoint", "octoquad", "octoquad_absent", "expansion", "absent",
                                             "overlay_nan", "freeze_dump") and self.active_config() != CONFIG_MAIN):
            self.activate_config(CONFIG_MAIN, self.main_config())

    def contained(self, name, step):
        """Records a BenchError from step as a failed bench and settles; False if it failed. Used alone for one part
        of a step whose next part runs on the same config."""
        try:
            step()
        except (ProtocolError, OtherHub):
            raise
        except BenchError as e:
            self.check(f"{name} bench completed", False, str(e))
            self.summary.append(f"## {name}\n\n**FAILED: {e}**\n")
            self.settle()
            return False
        except Exception as e:
            self.log(f"!! {name} raised {type(e).__name__}: {e}\n{traceback.format_exc()}")
            self.check(f"{name} bench completed", False, f"the runner raised {type(e).__name__}: {e} (run.log has the "
                       "traceback)", "a runner bug or an adb timeout in this step; the step's own checks are missing")
            self.summary.append(f"## {name}\n\n**FAILED: the runner raised {type(e).__name__}: {e}**\n")
            self.settle()
            return False
        return True

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

    def port_statements(self):
        """Each typed port statement this run needs, before its first adb command: a bench that drives more than
        --ports-empty covers runs only once the person has typed that those ports are empty."""
        for flag in TYPED_STATEMENTS:
            ports = next(p for f, p, _ in PORT_STATEMENTS if f == flag)
            named = [b for b in MODULE_BENCHES if b.statement == flag and getattr(self.args, b.name)]
            given = getattr(self.args, flag[2:].replace("-", "_"))
            if named and not given:
                raise BenchError(f"{', '.join(b.title for b in named)} drives {ports.describe()}. Unplug everything "
                                 f"from them, then rerun with {flag}.")
            if not given:
                continue
            if not sys.stdin.isatty():
                raise BenchError(f"{flag} has you type that nothing is plugged into {ports.describe()}, so it needs "
                                 "a terminal")
            for b in named:
                self.log(f"{b.title} commands {b.powers}")
            try:
                reply = self.answer(f">>> Unplug everything from {ports.describe()}. Then type empty.")
            except EOFError:
                raise BenchError(f"the terminal closed before the {flag} statement; nothing was changed on the "
                                 "hub") from None
            if reply != "empty":
                raise BenchError(f"{flag}: you typed {reply!r}, not empty, so the run stopped; nothing was changed on "
                                 "the hub")
            self.typed_statements.add(flag)

    def bench_step(self, name):
        bench = module_bench(name)
        return getattr(self, f"run_{name}") if bench is None else (lambda: self.run_module_bench(bench))

    def run_module_bench(self, bench):
        """A bench from scripts/bench/benches/: its run fills the record, then its checks and summary read only the
        record. The hub is left on the main bench config, as every in-place bench leaves it."""
        if bench.statement in TYPED_STATEMENTS and bench.statement not in self.typed_statements:
            raise BenchError(f"{bench.title} runs only once {bench.statement}'s statement has been typed")
        if bench.drives:
            self.log(f"{bench.title} commands {bench.powers}")
        record = self.results[bench.name] = {}
        try:
            bench.run(self, record)
        finally:
            if self.dash.fatal is None and self.active_config() != CONFIG_MAIN:
                self.activate_config(CONFIG_MAIN, self.main_config())
        for name, ok, detail, means in bench.checks(record):
            self.check(name, ok, detail, means)
        self.summary += bench.summary(record)

    def selection(self):
        a = self.args
        every = [n for n in bench_order() if n != "octoquad_absent"]
        chosen = {n for n in every if getattr(a, n)}
        explicit = bool(chosen) or a.all
        if not chosen:
            chosen = set(every) - set(FLAG_ONLY_BENCHES) - {b.name for b in MODULE_BENCHES if not b.default}
        if "octoquad" in chosen:
            chosen.add("octoquad_absent")
        if a.no_sloth:
            chosen.discard("sloth")
        address = a.expansion_address
        self._skip_unless(chosen, explicit, "expansion", self.expansion is not None,
                          f"no Expansion Hub answered at address {address}", "Expansion Hub bench",
                          f"No Expansion Hub answered at address {address}: connect one to the Control Hub's RS485 "
                          "port and a 12 V supply, pass its address with --expansion-address N, or drop --expansion")
        self._skip_unless(chosen, explicit, "webcam", self.webcam is not None, "no webcam plugged in", "webcam benches",
                          "No webcam is plugged in: plug a UVC webcam into a Control Hub USB port, or drop --webcam")
        self._skip_unless(chosen, explicit, "webcam_failure", self.webcam is not None, "no webcam plugged in",
                          "webcam thread failure bench",
                          "No webcam is plugged in: plug a UVC webcam into a Control Hub USB port, or drop "
                          "--webcam-failure")
        self._skip_unless(chosen, explicit, "limelight", self.limelight is not None, "no Limelight plugged in",
                          "Limelight bench",
                          "No Limelight answers on an Ethernet-over-USB interface: plug the Limelight 3A into a "
                          "Control Hub USB port and wait for it to boot, or drop --limelight")
        for bench in MODULE_BENCHES:
            if bench.name in chosen and bench.needs is not None:
                missing = bench.needs(self)
                self._skip_unless(chosen, explicit, bench.name, missing is None, missing, bench.title,
                                  f"{(missing or '')[:1].upper()}{(missing or '')[1:]}, or drop {bench.flag}")
        self.log(f"benches: {sorted(chosen)}")
        return chosen

    def _skip_unless(self, chosen, explicit, name, present, why, what, error):
        if name not in chosen or present:
            return
        if explicit:
            raise BenchError(error)
        self.log(f"{why}: skipping the {what}")
        self.summary.append(f"_{what[0].upper()}{what[1:]} skipped: {why}._\n")
        chosen.discard(name)


class BenchLayoutError(Exception):
    """A file in scripts/bench/benches/ that doesn't follow that folder's README."""


class RunnerNamespace:
    """What a bench module sees as `rb`: this file's globals, read at each use."""

    def __init__(self, namespace):
        object.__setattr__(self, "_namespace", namespace)

    def __getattr__(self, name):
        try:
            return self._namespace[name]
        except KeyError:
            raise AttributeError(f"run_bench has no {name}") from None

    def __setattr__(self, name, value):
        raise AttributeError("a bench module reads run_bench's globals; it never sets them")


def port_list(ports):
    runs = []
    for p in sorted(ports):
        if runs and p == runs[-1][1] + 1:
            runs[-1][1] = p
        else:
            runs.append([p, p])
    return ", ".join(f"{a}" if a == b else f"{a}-{b}" for a, b in runs)


class Ports:
    """The motor and servo ports a bench drives, by hub."""
    HUBS = (("controlHub", "Control Hub"), ("expansionHub", "the Expansion Hub's"))

    def __init__(self, ch_motors=(), ch_servos=(), eh_motors=(), eh_servos=()):
        self.ports = {("controlHub", "motor"): frozenset(ch_motors), ("controlHub", "servo"): frozenset(ch_servos),
                      ("expansionHub", "motor"): frozenset(eh_motors), ("expansionHub", "servo"): frozenset(eh_servos)}

    def __bool__(self):
        return any(self.ports.values())

    def within(self, other):
        return all(ports <= other.ports[key] for key, ports in self.ports.items())

    def describe(self):
        parts = []
        for hub, words in self.HUBS:
            kinds = [f"{kind} ports {port_list(self.ports[(hub, kind)])}" for kind in ("motor", "servo")
                     if self.ports[(hub, kind)]]
            if kinds:
                parts.append(f"{words} {' and '.join(kinds)}")
        return " and ".join(parts) or "no motor or servo port"


S3_PORTS = Ports(ch_motors=range(4), ch_servos=range(6), eh_motors=range(4), eh_servos=range(6))
PORT_STATEMENTS = (("--ports-empty", Ports(ch_motors=BENCH_MOTOR_PORTS, ch_servos=BENCH_SERVO_PORTS), None),
                   ("--s3-ports", S3_PORTS,
                    "you type at the start that nothing is plugged into either hub's motor ports 0-3 or servo ports "
                    "0-5 (needs a terminal); a bench that drives them runs only with it; it stands for --ports-empty "
                    "too"))
TYPED_STATEMENTS = ("--s3-ports",)


class Person:
    """What session.py's Needs a person says about a bench that waits for someone: what it is, what to have on the
    table, and the run_bench.py flags that run it (the bench's flag, and its typed statement's, by default)."""

    def __init__(self, what, have, flags=None):
        self.what, self.have, self.flags = what, have, flags


class Bench:
    """One bench in its own file, scripts/bench/benches/<name>.py, declared there as BENCH = Bench(...).
    scripts/bench/benches/README.md says what each field means."""

    def __init__(self, name, title, flag, help, run, checks, summary, default=False, options=(), needs=None,
                 person=None, drives=None, powers=None, configs=(), opmodes=()):
        self.name, self.title, self.flag, self.help = name, title, flag, help
        self.run, self.checks, self.summary = run, checks, summary
        self.default, self.options, self.needs, self.person = default, tuple(options), needs, person
        self.drives = Ports() if drives is None else drives
        self.powers, self.configs, self.opmodes = powers, tuple(configs), tuple(opmodes)
        self.statement = None
        self.person_flags = ()


BENCH_FOLDER = Path(__file__).resolve().parent / "benches"
BENCH_ORDER_FILE = "order.txt"
IN_PLACE_BENCH_OPMODES = tuple(BENCH_OPMODES)
IN_PLACE_BENCH_CONFIGS = tuple(BENCH_CONFIGS)
MODULE_BENCHES = ()


def bench_layout_problems(bench, name, taken):
    """Why this declaration can't be run safely, or an empty list."""
    problems = []
    if not isinstance(bench, Bench):
        return ["it declares no BENCH = rb.Bench(...)"]
    if bench.name != name:
        problems.append(f"its name is {bench.name!r}, not its file's {name!r}")
    if bench.flag != "--" + name.replace("_", "-"):
        problems.append(f"its flag is {bench.flag!r}, not --{name.replace('_', '-')}")
    if bench.name in taken:
        problems.append(f"{bench.name} is already a bench")
    if not bench.title.startswith("Bench: "):
        problems.append(f"its title {bench.title!r} doesn't start with 'Bench: '")
    if not bench.help:
        problems.append("it has no help text")
    for field in ("run", "checks", "summary"):
        if not callable(getattr(bench, field)):
            problems.append(f"its {field} isn't a function")
    if bench.needs is not None and not callable(bench.needs):
        problems.append("its needs isn't a function")
    for option in bench.options:
        if not (isinstance(option, tuple) and len(option) == 2 and str(option[0]).startswith("--")
                and isinstance(option[1], dict)):
            problems.append(f"option {option!r} isn't (\"--flag\", {{argparse keywords}})")
    if not isinstance(bench.drives, Ports):
        problems.append("its drives isn't rb.Ports(...)")
    elif bench.drives:
        if not bench.powers:
            problems.append("it drives ports but its powers doesn't say what it commands on them")
        bench.statement = next((flag for flag, ports, _ in PORT_STATEMENTS if bench.drives.within(ports)), None)
        if bench.statement is None:
            problems.append(f"no port statement covers {bench.drives.describe()}: add one to PORT_STATEMENTS")
    if bench.default and bench.statement not in (None, "--ports-empty"):
        problems.append(f"it is in the default suite but needs {bench.statement}, which a default run never gives")
    if bench.default and bench.person is not None:
        problems.append("it is in the default suite but waits for a person: make it flag-only")
    if bench.statement in TYPED_STATEMENTS and bench.person is None:
        problems.append(f"{bench.statement} is typed at the terminal, so it needs a person = rb.Person(...)")
    if bench.person is not None and not isinstance(bench.person, Person):
        problems.append("its person isn't rb.Person(...)")
    for config in bench.configs:
        if not config.startswith("bench_") and not (REPO_ROOT / SHIPPED_CONFIG_DIR / f"{config}.xml").exists():
            problems.append(f"config {config} is neither a bench_* config it writes nor shipped in res/xml")
    if isinstance(bench.person, Person):
        bench.person_flags = tuple(bench.person.flags or [bench.flag] + (
            [bench.statement] if bench.statement in TYPED_STATEMENTS else []))
    return problems


def load_benches(folder=None):
    """The benches in scripts/bench/benches/, in order.txt's order. A file it doesn't list, a name it lists twice
    or without a file, or a declaration that breaks a rule stops run_bench.py before it does anything."""
    folder = Path(folder or BENCH_FOLDER)
    order = folder / BENCH_ORDER_FILE
    names = [line.strip() for line in order.read_text().splitlines() if line.strip()] if order.exists() else []
    files = {p.stem for p in folder.glob("*.py")} if folder.is_dir() else set()
    problems = [f"{n} is listed twice in {order}" for n in sorted({n for n in names if names.count(n) > 1})]
    problems += [f"{order} lists {n}, but {folder / (n + '.py')} doesn't exist" for n in names if n not in files]
    problems += [f"{folder / (n + '.py')} isn't listed in {order}" for n in sorted(files - set(names))]
    if problems:
        raise BenchLayoutError("; ".join(problems))
    benches, taken = [], set(BENCHES)
    for name in names:
        path = folder / f"{name}.py"
        module = types.ModuleType(f"bench_{name}")
        module.__file__ = str(path)
        # Looked up at each use, not copied: the offline tests swap this file's subprocess and clock after it loads.
        module.rb = RunnerNamespace(globals())
        exec(compile(path.read_text(), str(path), "exec"), module.__dict__)
        bench = module.__dict__.get("BENCH")
        problems = bench_layout_problems(bench, name, taken)
        if problems:
            raise BenchLayoutError(f"{path}: " + "; ".join(problems))
        bench.module = module
        taken.add(name)
        benches.append(bench)
    return tuple(benches)


def use_benches(benches):
    """Makes these the benches from scripts/bench/benches/: their flags, run order, OpModes and bench configs."""
    global MODULE_BENCHES, BENCH_OPMODES, BENCH_CONFIGS
    MODULE_BENCHES = tuple(benches)
    BENCH_OPMODES = list(dict.fromkeys(IN_PLACE_BENCH_OPMODES + tuple(o for b in MODULE_BENCHES for o in b.opmodes)))
    BENCH_CONFIGS = tuple(dict.fromkeys(IN_PLACE_BENCH_CONFIGS + tuple(
        c for b in MODULE_BENCHES for c in b.configs if c.startswith("bench_"))))


def bench_order():
    """Every bench in run order: BENCHES, with scripts/bench/benches/'s inserted in order.txt's order just before
    Bench: Turret Zero, so Turret Zero, Freeze Dump and Dashboard Off stay last."""
    i = BENCHES.index("turret_zero")
    return BENCHES[:i] + tuple(b.name for b in MODULE_BENCHES) + BENCHES[i:]


def module_bench(name):
    return next((b for b in MODULE_BENCHES if b.name == name), None)


def needs_a_person():
    """session.py's Needs a person entries for the benches in scripts/bench/benches/: (key, what, flags, have)."""
    return tuple((b.name, b.person.what, list(b.person_flags), b.person.have) for b in MODULE_BENCHES
                 if b.person is not None)


def person_flags():
    """Flags that make a run wait for someone at its terminal, which session.py never gives one of its stages."""
    return tuple(TYPED_STATEMENTS) + tuple(b.flag for b in MODULE_BENCHES if b.person is not None)


def add_module_bench_flags(p):
    statements = [(flag, help) for flag, _, help in PORT_STATEMENTS if help is not None]
    if not (MODULE_BENCHES or statements):
        return
    g = p.add_argument_group("benches in scripts/bench/benches/ (its README.md)")
    for b in MODULE_BENCHES:
        g.add_argument(b.flag, action="store_true", help=b.help)
        for flag, keywords in b.options:
            g.add_argument(flag, **keywords)
    for flag, help in statements:
        g.add_argument(flag, action="store_true", help=help)


def parse_args(argv):
    p = argparse.ArgumentParser(description="Control Hub bench runner (see scripts/bench/README.md)")
    p.add_argument("--baseline", action="store_true", help="whether slothboard reports a config baseline")
    p.add_argument("--hub", action="store_true", help="Bench: Hub (bare hub costs, planner, Limelight poller sim)")
    p.add_argument("--loop", action="store_true", help="Bench: Framework Loop and Bench: Raw Loop")
    p.add_argument("--failure", action="store_true", help="Bench: Failure Path (EnhancedOpMode's safe-state pass)")
    p.add_argument("--threads", action="store_true",
                   help="Bench: Thread Lifecycle (TransferCounter's poller thread through normal stops, crashes, a "
                        "thread failure, interrupts, 10 back-to-back runs and, unless --no-sloth, a Sloth hot reload)")
    p.add_argument("--edge", action="store_true",
                   help="Bench: Edge Through Start (a gamepad button held from INIT through START)")
    p.add_argument("--turret", action="store_true",
                   help="Bench: Turret PWM (BettaTurret's servo PWM through stop() and the write toggle; its own config)")
    p.add_argument("--freeze-dump", action="store_true",
                   help="Bench: Freeze Dump (slothboard's SET (3x), WRITE and DELETE_HARDWARE_CONFIG from idle, a "
                        "thread-dump watch armed; on slothboard 6165.6 each gets the app killed, so it runs last)")
    p.add_argument("--dashboard-off", action="store_true",
                   help="Bench: Dashboard Off (switches the dashboard off, checks nothing streams and that off survives "
                        "an app restart, a reboot and a reinstall, then turns it back on over adb; with --interactive, "
                        "a battery pull too; runs last, only with this flag)")
    p.add_argument("--list-configs", action="store_true",
                   help="only list the hub's saved hardware configs, their webcam and Ethernet device names and stale "
                        "ones; changes nothing on the hub, so it works on a robot's hub")
    p.add_argument("--probe-only", action="store_true",
                   help="only Bench: Probe and the REV hubs on the Control Hub's USB ports, recorded as hardware in "
                        "runner_results.json for session.py's plan, then the config put back; drives nothing")
    p.add_argument("--gamepad", action="store_true",
                   help="Bench: Gamepad and Bench: Gamepad Linear (dashboard gamepads sent over the websocket)")
    p.add_argument("--write-recovery", action="store_true",
                   help="Bench: Write Recovery (whether EnhancedMotor/EnhancedServo writes reach the hub after a failSafe)")
    p.add_argument("--handover", action="store_true",
                   help="Bench: Drive Handover (Drivetrain taking the drive motors back from Pedro)")
    p.add_argument("--photon", action="store_true",
                   help="the Photon block: stock baseline, Bench: Photon, Faults, STOP trials, routing over app "
                        "restarts, consecutive OpModes, deploySloth, then Framework Loop off and on and Write Recovery "
                        "on (about 50 min with a USB Expansion Hub and --photon-i2c; not part of a run without bench flags, nor of "
                        "--all)")
    p.add_argument("--usb-expansion", action="store_true",
                   help="the Photon block also drives an Expansion Hub connected by USB alone (motor ports 0-3, servo "
                        "ports 0-1)")
    p.add_argument("--photon-rs485", action="store_true",
                   help="the Photon block with the Expansion Hub on RS485 at --expansion-address: it must stay on the "
                        "SDK's path")
    p.add_argument("--photon-soak", action="store_true",
                   help="the Photon block with its long write soak (up to 1e7 writes per hub, about 3 h), 200 STOP "
                        "trials per condition and more lost-write trials; with --photon-i2c (and on USB wiring "
                        "--usb-expansion --interactive too) it is the run whose verdict can say SAFE")
    p.add_argument("--photon-soak-hours", type=float, default=None,
                   help=f"end --photon-soak's soak after this many hours instead of {PHOTON_SOAK_MAX_S // 3600} (at most "
                        "that), if it hasn't reached its write count by then")
    p.add_argument("--photon-stop-trials", type=int, default=None,
                   help=f"STOP trials per condition (default {PHOTON_STOP_TRIALS}, {PHOTON_SOAK_STOP_TRIALS} with "
                        "--photon-soak)")
    p.add_argument("--photon-seed", type=int, default=None, help="seed for the STOP delays (default: the time)")
    p.add_argument("--usb-expansion-serial", default=None,
                   help="the USB Expansion Hub's serial, if the runner can't read it from the Control Hub")
    p.add_argument("--photon-i2c", action="store_true",
                   help="the Photon block also reads an OctoQuad (Control Hub I2C port 2) and a goBILDA Pinpoint "
                        "(Expansion Hub I2C port 1; Control Hub port 1 with no Expansion Hub), two threads per device every "
                        "10 ms, Photon off, on2 and on8 and through the soak, plus an unlocked-I2C negative control "
                        "(about 2 min of the block); a wiring's verdict can only say SAFE with it")
    p.add_argument("--hardware-view", action="store_true",
                   help="the Hardware OpMode, then a Hardware view Power edit under Bench: Power Readback")
    p.add_argument("--planner-stop", action="store_true",
                   help="Bench: Planner Stop (STOP pressed while the route planner replans on the OpMode thread)")
    p.add_argument("--overlay-nan", action="store_true",
                   help="Bench: Overlay NaN (restarts the app afterwards)")
    p.add_argument("--config-by-name", action="store_true",
                   help="which XML the app loads for a shipped config saved with another config's resource id, and for "
                        "a name the build doesn't ship (three config switches)")
    p.add_argument("--webcam", action="store_true", help="Bench: Webcam and the Webcam Stop runs (needs a UVC webcam)")
    p.add_argument("--webcam-failure", action="store_true",
                   help="Bench: Webcam Thread Failure (a pipeline that throws on the camera thread; needs a UVC webcam)")
    p.add_argument("--limelight", action="store_true", help="Bench: Limelight (needs a Limelight 3A)")
    p.add_argument("--llfault", action="store_true",
                   help="Bench: Limelight Fault: the fault fallback with a limelight configured and nothing answering; "
                        "then Bench: Limelight Hung: INIT, STOP and the next INIT while a Limelight POST hangs")
    p.add_argument("--pinpoint", action="store_true",
                   help="Bench: Pinpoint (firmware, status, INIT start pose; dropout with --interactive), or Bench: "
                        "Pinpoint Absent when none answers")
    p.add_argument("--octoquad", action="store_true",
                   help="Bench: OctoQuad (GuardedOctoQuadLocalizer on Control Hub I2C bus 2: read cost, valid reads, INIT "
                        "start pose, then channels 4-7 in pulse-width mode for the turret zero; dropout with "
                        "--interactive) and Bench: OctoQuad Absent (nothing on I2C bus 3)")
    p.add_argument("--expansion", action="store_true",
                   help="Bench: Expansion Hub (the same Lynx commands and odometry reads on each hub; needs an Expansion "
                        "Hub; with --interactive, again with the odometry moved to it)")
    p.add_argument("--expansion-address", type=int, default=2, help="the Expansion Hub's module address (default 2)")
    p.add_argument("--turret-zero", action="store_true",
                   help="Bench: Turret Zero, prompted (about 25 min, needs a terminal): a Through Bore on a split cable, "
                        "A/B on Control Hub encoder port 2 and its absolute output on OctoQuad channel 4; not part of a "
                        "run without bench flags, nor of --all")
    p.add_argument("--turret-zero-v2", action="store_true",
                   help="Bench: Turret Zero's V2 phase too, with a REV-11-3174 on hand (implies --turret-zero)")
    p.add_argument("--absent", action="store_true", help="the webcam-unplugged INIT check")
    p.add_argument("--sloth", action="store_true", help="the deploySloth/INIT race (runs gradle six times)")
    p.add_argument("--clock", action="store_true",
                   help="Bench: Clock (PathCommands.timeout against wall-clock jumps on the hub)")
    p.add_argument("--all", action="store_true",
                   help="every bench but the Photon block, Bench: Turret Zero and Dashboard Off, and fail if the "
                        "webcam, Limelight or Expansion Hub is missing")
    p.add_argument("--no-sloth", action="store_true",
                   help="skip the Sloth race, Thread Lifecycle's hot reload and the Photon block's deploySloth step (no "
                        "Gradle builds)")
    p.add_argument("--gradle-offline", action="store_true",
                   help="run every Gradle build the runner makes with --offline (no internet during the run)")
    p.add_argument("--original-entry", default=None,
                   help="the app's active-config setting from before a reinstall, as RobotConfigFile JSON "
                        "(bare_hub.sh passes it): restored at the end instead of the one read after the install")
    p.add_argument("--out", default=None, help="results folder (default bench-results/<timestamp> at the repo root)")
    p.add_argument("--interactive", action="store_true",
                   help="in the gamepad bench, has you drive the dashboard's Gamepad view (keyboard, on-screen stick, "
                        "USB pad); after the Limelight bench, prompts you to unplug and replug it and times the fault; "
                        "after Bench: Pinpoint and Bench: OctoQuad, to unplug and replug each; after Bench: Expansion "
                        "Hub, to move the Pinpoint and OctoQuad to the Expansion Hub and back; in Bench: Clock, to "
                        "open the RC's web page; in the Photon block (USB wiring), to pull the Expansion Hub's USB and "
                        "power cables, each once with Photon off and once on; in Bench: Dashboard Off, to pull the "
                        "Control Hub's battery lead once")
    p.add_argument("--quick", action="store_true", help="shorter phases (numbers are noisier)")
    p.add_argument("--keep-configs", action="store_true", help="leave the bench_* hardware configs on the hub")
    p.add_argument("--ports-empty", action="store_true",
                   help="motor ports 0-3 and servo ports 0-1 are empty on the Control Hub and, for the Photon block, "
                        "on the Expansion Hub: run even if a config names devices there")
    p.add_argument("--other-pipeline", type=int, default=0, help="the non-ball Limelight pipeline to switch to")
    p.add_argument("--host", default=HUB_IP)
    p.add_argument("--serial", default=None,
                   help=f"adb serial (default {HUB_IP}:{ADB_PORT}; with --usb, the one USB device adb lists)")
    p.add_argument("--usb", action="store_true",
                   help="reach the Control Hub over a USB cable to its USB-C port instead of its Wi-Fi: adb over the "
                        "cable (never adb connect) and the hub's ports through adb forwards on free ports of this Mac, "
                        "removed at the end; the Mac stays on its own network")
    p.add_argument("--hub-serialno", default=None,
                   help="the hub's ro.serialno (session.py passes it): every adb connection check reads it, and a hub "
                        "that reads otherwise stops the run with nothing more sent to it, its config not put back")
    add_module_bench_flags(p)
    args = p.parse_args(argv)
    if getattr(args, "s3_ports", False):
        args.ports_empty = True
    if args.usb and args.host != HUB_IP:
        p.error("--host is the hub's Wi-Fi address; with --usb the runner reaches the hub's ports through the cable")
    if args.hub_serialno is not None and not args.hub_serialno.strip():
        p.error("--hub-serialno takes the hub's ro.serialno, which can't be empty")
    if not 0 <= args.other_pipeline <= 9 or args.other_pipeline == BALL_PIPELINE:
        p.error(f"--other-pipeline must be a Limelight pipeline 0-9 other than the ball pipeline {BALL_PIPELINE}")
    if not 1 <= args.expansion_address <= EXPANSION_MAX_ADDRESS:
        p.error(f"--expansion-address must be 1-{EXPANSION_MAX_ADDRESS}: the SDK reserves higher module addresses")
    if args.usb_expansion or args.photon_rs485 or args.photon_soak:
        args.photon = True
    if args.turret_zero_v2:
        args.turret_zero = True
    if args.photon_stop_trials is not None and args.photon_stop_trials < 1:
        p.error("--photon-stop-trials must be at least 1")
    cap = PHOTON_SOAK_MAX_S / 3600
    if args.photon_soak_hours is not None and not (args.photon_soak and 0 < args.photon_soak_hours <= cap):
        p.error(f"--photon-soak-hours takes --photon-soak and a number of hours above 0, at most {cap:g}")
    if args.probe_only and (args.list_configs or args.all or any(getattr(args, n, False) for n in bench_order())):
        p.error("--probe-only runs no bench: leave out the bench flags, --all and --list-configs")
    return args


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    try:
        runner = Runner(args)
    except BenchError as e:
        print(f"error: {e}", file=sys.stderr)
        return 2
    return runner.run()


use_benches(load_benches())

if __name__ == "__main__":
    sys.exit(main())
