# Control Hub bench

Measures the framework on a bare Control Hub (no Driver Station, no robot): loop and bus costs, telemetry, the route
planner, the Limelight, webcam, Pinpoint, OctoQuad and Expansion Hub paths, stop and failure paths, dashboard gamepads
and the Sloth hot load. One command runs every bench the plugged-in hardware allows and writes
`bench-results/<timestamp>/` at the repo root (gitignored); its `summary.md` says what each FAIL means. What the results mean
for the code: `docs/hub-bench-findings.md`.

## Run it

1. Power the Control Hub from a 12 V battery and **unplug everything from motor ports 0-3 and servo ports 0-1**: the
   benches drive them with real power (up to 0.35). Never run it on a robot's hub. The runner refuses to start while a
   config on the hub names a motor or servo there; once they are empty, add `--ports-empty`. Keep I2C bus 3 empty.
2. Join the hub's Wi-Fi, then `adb connect 192.168.43.1:5555`.
3. Full install once (Android Studio, **TeamCode** run configuration, **Run**) and wait for the RC app.
4. Close every dashboard tab: its status requests read hub voltage on the bus being timed, and its Gamepad view sends
   gamepad states.
5. From the repo root: `python3 scripts/bench/run_bench.py` (the Mac's own python3 3.9+, nothing to install). About
   30 to 50 minutes; don't touch the hub. Ctrl-C is safe: it still restores your hardware config.
6. When it prints `Nothing to send back`, tell Claude.

The Sloth step runs `./gradlew :TeamCode:deploySloth` from the terminal: if `./gradlew --version` fails there, add
`--no-sloth`.

### Optional hardware

Missing hardware skips its benches, and the summary says so.

| plug in | where | runs |
|---|---|---|
| a UVC webcam streaming 640x480 MJPEG (the robot's Arducam OV9782 is best) | Control Hub USB | Bench: Webcam, Webcam Stop, Webcam Thread Failure |
| the Limelight 3A, booted | Control Hub USB | Bench: Limelight (the runner restores the pipeline it found) |
| a goBILDA Pinpoint, both pods plugged in and still | Control Hub I2C bus 1 | Bench: Pinpoint (else Pinpoint Absent) |
| an OctoQuad MK2, pods on encoder ports 3 and 4, still | Control Hub I2C bus 2 | Bench: OctoQuad |
| a REV Expansion Hub on its own 12 V | RS485, address 2 (`--expansion-address N`) | Bench: Expansion Hub (reads only) |

`--interactive` adds terminal prompts: pull and replug the Limelight, Pinpoint and OctoQuad cables (pods stay in); move
the Pinpoint and OctoQuad to the Expansion Hub's I2C ports 1 and 2 and back; open http://192.168.43.1:8080 during
Bench: Clock on a hub whose clock still reads about 1970; drive the dashboard's Gamepad view with the keyboard, the
on-screen stick and a USB pad (close the tab afterwards).

### Flags

| flag | does |
|---|---|
| `--ports-empty` | runs although a config names devices on motor ports 0-3 or servo ports 0-1 (only once they're unplugged) |
| `--interactive` | the prompts above |
| `--no-sloth` | skips the Sloth race (no Gradle builds) |
| `--quick` | shorter phases, noisier numbers |
| `--all` | every bench; stops if no webcam, Limelight or Expansion Hub is plugged in |
| `--keep-configs` | leaves the `bench_*` hardware configs on the hub |

`--help` lists the rest: one flag per bench (`--hub`, `--loop`, `--limelight`, ...), `--serial`, `--host`,
`--other-pipeline`.

## What it changes on the hub

It writes `bench_*` hardware configs to `/sdcard/FIRST` and switches between them by restarting the RC app; at the end
it re-activates the config that was active, deletes the `bench_*` files and restarts the app once more. The configs put
`fl`/`fr`/`bl`/`br` on motor ports 0-3, `benchServo`/`benchServo2` on servo ports 0-1 and `floodgate` on analog 0, and
command nothing on the Expansion Hub. It also raises the hub's log buffer to 8 MB until the next reboot. It never
deletes the active config, and a run that stops before writing the first `bench_*` config leaves the hub untouched.
With `--interactive`, opening the RC web page during Bench: Clock sets the hub's clock until it powers off.

If it prints `RESTORE FAILED`: open http://192.168.43.1:8080/dash, add the Hardware Config view and activate the config
the message names (a Control Hub has no Configure Robot screen without a Driver Station).

## The benches

| OpMode | needs | measures |
|---|---|---|
| Bench: Probe | nothing | the IMU chip, attached webcams and Limelights, whether an Expansion Hub answers |
| (config baseline, no OpMode) | nothing | slothboard's config baseline lists `Optimization Toggles` |
| Bench: Hub | nothing | empty-loop overhead; bus call costs; telemetry, field map and packet costs; the floodgate pin's volts; planner time for 0 to 5 balls; LLResult parse; Limelight POST and poller costs against a fake Limelight |
| Bench: Framework Loop | nothing | the EnhancedOpMode loop with Drivetrain and Pedro (capped at 0.2), one telemetry toggle per phase; the current limiter stays at 1.0 on an unwired floodgate |
| Bench: Raw Loop | nothing | the loop of a bare LinearOpMode running Pedro (capped at 0.2) without the framework, idle and then following Close Flower's first leg |
| Bench: Failure Path | nothing | a hook throwing in each lifecycle stage: which `stop()`s run, Suppressed failures, the dashboard Error line |
| Bench: Clock | nothing | `PathCommands.timeout` (15 s) against wall-clock jumps |
| Bench: Pinpoint, Pinpoint Absent | a Pinpoint / none | firmware, status, and whether a start pose set in INIT survives the IMU recalibration / the follower building without one |
| Bench: OctoQuad | an OctoQuad | read costs, status changes, CRC failures, start-pose survival |
| Bench: OctoQuad Absent | an empty I2C bus 3 | INIT with an `octoquad` configured and nothing answering |
| Bench: Expansion Hub | an Expansion Hub | the same Lynx commands and odometry reads on each hub |
| Bench: Write Recovery | nothing | EnhancedMotor and EnhancedServo reach the hub again within 500 ms of a `failSafe()` |
| Bench: Drive Handover | nothing | `fl`'s SDK power matches Drivetrain's command from the 2nd loop after Pedro hands back |
| Hardware OpMode, then Bench: Power Readback | nothing | a stale Hardware-view Power edit doesn't drive `fl` under the next OpMode |
| Bench: Gamepad, Gamepad Linear | nothing | dashboard gamepad delivery, one edge per press, sticks resting within 1 s of silence |
| Bench: Limelight Fault | nothing | the `Limelight` fault with nothing answering: raised in INIT within 5 s, loop keeps running, collection ends SKIPPED |
| Bench: Webcam Stop (absent) | nothing | INIT fails with CellTipCamera's "failed to open" |
| Bench: Webcam | a webcam | open/close times, per-frame and stream costs, frame age against `CellTipCamera.maxStalenessMs` |
| Bench: Webcam Stop | a webcam | STOP at several points with the webcam open |
| Bench: Webcam Thread Failure | a webcam | a camera-thread exception reaches the Error line within 2 s |
| Bench: Limelight | the Limelight | slothboard's proxy, `LimelightSync` time, INIT to first frame, POST costs, frame rate and latency |
| Bench: Planner Stop | nothing | STOP during 4- and 5-ball plans: no app restart, STOP to idle within 900 ms |
| Bench: Overlay NaN | nothing | a NaN in a field overlay doesn't kill dashboard telemetry (runs last) |
| Bench: Sloth Probe | Gradle | INIT racing deploySloth, the OpMode list during a load, the load signal, the jar surviving an app restart |

The OpModes also run from the dashboard, but they expect the runner's configs and write results only to the hub.

**Bench: Analog Poll** is run by hand only: `laser_rangefinder.xml` active, both rangefinders plugged in (it commands no
motor). About 80 s after START the DS and dashboard show its summary; the rest is in `analog_poll.json` in the hub's
bench folder. It measures how often each analog value refreshes and what a poller thread (`pollMs` in that folder's
`params.json`, default 10) costs a stand-in robot loop.

## Notes for changing the bench

- The active config is read from the hub, never from slothboard's broadcast lists, which a restart can drop. To switch,
  the runner stops the app, writes `RobotConfigFile`'s Gson form under `pref_hardware_config_filename` in its default
  SharedPreferences with `run-as com.qualcomm.ftcrobotcontroller` (needs the Run button's debuggable APK; a running app
  saves its own copy over it), and restarts it; a switch is done at logcat's `======= INIT FINISH =======`. Only without
  run-as does it switch through slothboard, which freezes the app until the hub's watchdog relaunches it about 10 s later.
- A shipped `res/xml` config is stored by resource id, which an install that adds, renames or removes a `res/xml` file
  can renumber: the probe checks the stored id still names that XML, and the restore re-activates shipped configs by name.
- Params and results go through the app's own `files/bench/` via run-as (`/sdcard/FIRST/bench/` only without it), and
  each write is read back: an OpMode once read a just-pushed `/sdcard` `params.json` as empty.
- Each config switch's logcat is in `logs/config_*.logcat.txt`, each step's in `logs/<step>.log`; the 8 MB log buffer
  keeps the hub's `Lights Hal` spam from pushing them out.
- The websocket is `ws://192.168.43.1:8000/`. NanoHTTPD drops a socket silent for 5 s, so the runner pings
  (`GET_ROBOT_STATUS` would put hub voltage reads in the timings). An OpMode list within 1 s of a socket opening is
  slothboard's on-connect copy, not a broadcast.
- SDK 12 only logs an OpMode's exception (`User code threw an uncaught exception`, tag `OpModeManager`). The runner takes
  it from slothboard's `RECEIVE_LOGCAT_ERRORS` (after the step began, until STOP), from a status back at `$Stop$Robot$`
  without BENCH DONE, or from adb logcat while waiting for INIT. `errorMessage` is the status's, else that line;
  `statusErrorMessage` is exactly the dashboard's Error line.
- An app restart takes about 40 s, during which slothboard's status names no OpMode and a request can close the socket:
  the runner re-sends and waits for idle wherever it stops, starts or recovers a step, and `summary.md` lists them.
- `BenchReport` writes the result file and logs `BENCH DONE <file>` under `BENCH` before the `BENCH DONE` packet: when
  the dashboard goes quiet the runner polls logcat every 5 s and finishes from the file. No packet within 5 s of INIT
  means slothboard's telemetry thread died: the runner logs why and restarts the app after the step.
- slothboard's Limelight proxy holds `*:5807` from app start and Linux refuses `127.0.0.1:5807` beside it, so Bench:
  Hub's fake Limelight is on `127.0.0.1:15807`, each loopback `Limelight3A`'s private `baseUrl` repointed by reflection.
- With no Driver Station the SDK drops telemetry before serializing it, so DS-frame rows leave out serialization and the
  UDP send; `summary.md` labels them.
- The probe names the IMU by `LynxModuleImuType.name()` (`BHI260`, not `toString()`'s `BHI260AP`) and stops if it can't.
- The Sloth race sets `ANDROID_SERIAL` (the Load plugin runs bare `adb`), kills a deploySloth still waiting on the Sloth
  lock after 600 s, and times each load from the hub's logcat after its `BENCH sloth pushed <run>` mark.
- The gamepad benches take their phase from gamepad2's right trigger (phase / 100) and a message counter on its left
  trigger, so an all-zero rest state leaves the phase alone; phase 99 ends the bench.
- Bench: OctoQuad Absent hangs in `createRobot()` before its first packet, so the runner drives it outside
  `run_opmode()`, which would take that silence for dead telemetry and restart the app.
