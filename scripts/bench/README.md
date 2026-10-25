# Control Hub bench

Measures the framework on a bare Control Hub (no Driver Station, no robot): loop and bus costs, telemetry, the route
planner, the Limelight, webcam, Pinpoint, OctoQuad and Expansion Hub paths, stop and failure paths, dashboard gamepads
and the Sloth hot load. One command runs every bench the plugged-in hardware allows and writes
`bench-results/<timestamp>/` at the repo root (gitignored); its `summary.md` says what each FAIL means. What the results mean
for the code: `docs/hub-bench-findings.md`.

## Bare hub, unattended

For a Control Hub with nothing plugged into any port, run where there is no internet and nobody to ask:

1. While still online, from the worktree root: `scripts/bench/bare_hub.sh --build`. It refuses a dirty tree, builds the
   debug APK, checks Gradle builds offline, and stamps the APK with HEAD (`TeamCode/build/outputs/apk/debug/.hubtest-build`).
   Rerun it after every commit.
2. Power the hub from a 12 V battery with nothing plugged in, join its Wi-Fi on the Mac (no internet then), plug the Mac
   in with its lid open, close every dashboard tab, and turn off auto-join for other networks so macOS doesn't hop away.
3. `scripts/bench/bare_hub.sh`. About 45 minutes, at most 90; no keypresses. Extra flags go to `run_bench.py`.
4. When it prints `Reconnect to the internet and tell Claude the run finished.` (or `...the run stopped.`), do that.

In order, it: stops unless the APK and its stamp match HEAD and no tracked file changed; finds adb as `run_bench.py`
does and connects to `192.168.43.1:5555` (stops within about 20 s, touching nothing, if the Mac isn't on the hub's
Wi-Fi), and checks it is a Control Hub with the RC app (the SDK's `persist.ftcandroid.serialasusb`); keeps the Mac awake
(`caffeinate`); stops, saying so and how to reconnect, if adb loses the hub at any step before the runner starts (told
apart from a real answer by adb's exit status and `get-state`, never by empty output); checks Gradle builds offline,
adding `--no-sloth` if not; copies the installed RC APKs (with sha256), its
version, every `/sdcard/FIRST/*.xml`, the app's settings and the Sloth folder to `.git/bench-backups/<time>/` in the
main checkout (outside every worktree, so removing one can't delete it), and
writes `REINSTALL.sh` there, which puts that app, its settings and its Sloth jar back; passes the active-config setting
from before the install to the runner (`--original-entry`), so the runner restores that one; installs the stamped APK with `adb install -r -t -g` (`-g` so no permission prompt waits for a tap;
a signature mismatch or downgrade stops it with the hub unchanged; it never uninstalls); deletes
`/storage/emulated/0/FIRST/dairy/sloth/*` as the Run button's `removeSlothRemote` does; starts the RC's launcher activity
and waits for `======= INIT FINISH =======` and the dashboard port; then runs
`run_bench.py --gradle-offline --out bench-results/<time>`, which refuses a hub whose configs name devices on the
bench ports unless you add `--ports-empty` once they are unplugged,, the whole console in `console.log` there. The
runner restores the hardware config in a `finally`, whatever fails.

## Run it

1. Power the Control Hub from a 12 V battery and **unplug everything from motor ports 0-3 and servo ports 0-1**: the
   benches drive them with real power (up to 0.35). Never run it on a robot's hub. The runner refuses to start while a
   config on the hub names a motor or servo there; once they are empty, add `--ports-empty`. Keep I2C bus 3 empty.
2. Join the hub's Wi-Fi, then `adb connect 192.168.43.1:5555`.
3. Full install once (Android Studio, **TeamCode** run configuration, **Run**) and wait for the RC app.
4. Close every dashboard tab: its status requests read hub voltage on the bus being timed, and its Gamepad view sends
   gamepad states.
5. From the repo root: `python3 scripts/bench/run_bench.py` (the Mac's own python3 3.9+, nothing to install). About
   55 to 75 minutes with every device plugged in, about 45 on a bare hub. The longest newer ones: Thread Lifecycle 6
   (2 of them its Sloth reload), Freeze Dump 6 (five dashboard config changes; on slothboard 6165.6 or older each one
   freezes the app until it is killed and relaunched), Config by name 3 (three config switches), Turret PWM 2 (two
   config switches), Limelight Hung 1.5. Don't touch the hub. Ctrl-C once is safe: it restores your hardware config and ignores more Ctrl-C until that's done.
6. When it prints `Nothing to send back`, tell Claude.

The Sloth race and Thread Lifecycle's reload run `./gradlew :TeamCode:deploySloth` from the terminal: if
`./gradlew --version` fails there, add `--no-sloth`; with no internet, add `--gradle-offline`.

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
| `--no-sloth` | skips the Sloth race and Thread Lifecycle's hot reload (no Gradle builds) |
| `--gradle-offline` | adds `--offline` to every Gradle build the runner makes |
| `--out DIR` | writes the results there instead of `bench-results/<timestamp>/` |
| `--quick` | shorter phases, noisier numbers |
| `--all` | every bench; stops if no webcam, Limelight or Expansion Hub is plugged in |
| `--keep-configs` | leaves the `bench_*` hardware configs on the hub |

`--help` lists the rest: one flag per bench (`--hub`, `--loop`, `--threads`, `--edge`, `--turret`, `--config-by-name`,
`--freeze-dump`, `--limelight`, ...), `--serial`, `--host`, `--other-pipeline`.

## What it changes on the hub

It writes `bench_*` hardware configs to `/sdcard/FIRST` and switches between them by restarting the RC app; at the end
it re-activates the config that was active (writing its XML back from `original_config.xml` first when its name is a
`bench_*` one, which the run overwrote), deletes the `bench_*` files and restarts the app once more. Config by name
saves settings that name a shipped config by another one's resource id, and a name the build doesn't ship, then
switches back. The configs put
`fl`/`fr`/`bl`/`br` on motor ports 0-3, `benchServo`/`benchServo2` on servo ports 0-1 (`TurretL`/`TurretR` in
`bench_turret`), `floodgate` on analog 0 and `analog0`/`analog1` on analog 2 and 3, and command nothing on the Expansion
Hub. Bench: Freeze Dump, last, sends slothboard's SET (three times), WRITE (the active config's own contents) and DELETE
(a `bench_freeze_scratch` it pushes first) from idle; on slothboard 6165.6 or older each one freezes the app, which the
Control Hub kills and relaunches. It also raises the hub's log buffer to 8 MB until the next reboot. It never
deletes the active config, and a run that stops before writing the first `bench_*` config leaves the configs untouched.
With `bare_hub.sh` the hub runs the bench build from its install on; only `REINSTALL.sh` puts the old app back.
With `--interactive`, opening the RC web page during Bench: Clock sets the hub's clock until it powers off.

If it prints `RESTORE FAILED`: open http://192.168.43.1:8080/dash, add the Hardware Config view and activate the config
the message names (a Control Hub has no Configure Robot screen without a Driver Station).

## The benches

| OpMode | needs | measures |
|---|---|---|
| Bench: Probe | nothing | the IMU chip, attached webcams and Limelights, whether an Expansion Hub answers, the hardware map's device names; whether the original shipped config's saved resource id still names its XML on this install |
| (config baseline, no OpMode) | nothing | slothboard's config baseline lists `Optimization Toggles` |
| Bench: Hub | nothing | empty-loop overhead; bus call costs; telemetry, field map and packet costs; the floodgate pin's volts; sliced planner latency, slice lengths and loop period for 0 to 5 balls; LLResult parse; Limelight POST and poller costs against a fake Limelight |
| Bench: Framework Loop | nothing | the EnhancedOpMode loop with Drivetrain and Pedro (capped at 0.2), one telemetry toggle per phase; the current limiter stays at 1.0 on an unwired floodgate |
| Bench: Raw Loop | nothing | the loop of a bare LinearOpMode running Pedro (capped at 0.2) without the framework, idle and then following Close Flower's first leg |
| Bench: Failure Path | nothing | a hook throwing in each lifecycle stage: which `stop()`s run, Suppressed failures, the dashboard Error line, and on the hub's clock how long the SDK takes to log the throw; the previous crash's Error line gone at the next INIT, judged only when a status showed the line empty (or stale) before the run's own crash |
| Bench: Thread Lifecycle | nothing | `modules/TransferCounter`'s poller: one daemon thread, `stop()` within 50 ms, 90+ polls/s; no poller left at the next INIT after a normal STOP, a STOP from INIT, a crash in `loop()` and a throwing `stop()` (each also seen from adb); a failing poller stops the OpMode within 50 ms; whether `interrupt()` mid-command leaks the hub lock; 10 back-to-back runs; a Sloth reload while it runs |
| Bench: Edge Through Start | nothing | BioBuzz Tele's `EdgeBooleanSupplier` pattern with gamepad1 LB held from INIT through START: no edge at START, one per later press |
| Bench: Turret PWM | nothing (`bench_turret`) | BettaTurret's PWM, read from the hub's registers, through CENTER, `stop()` with writes off and on, writes back on, IDLE and RightState.FORWARD |
| Bench: Clock | nothing | `PathCommands.timeout` (15 s) against wall-clock jumps |
| Bench: Pinpoint, Pinpoint Absent | a Pinpoint / none | firmware, status, and whether a start pose set in INIT survives the IMU recalibration / that a missing Pinpoint raises a fault or keeps the pose `setPose` set |
| Bench: OctoQuad | an OctoQuad | GuardedOctoQuadLocalizer's read costs, status changes, CRC failures after the first RUNNING read, start-pose survival |
| Bench: OctoQuad Absent | an empty I2C bus 3 | GuardedOctoQuadLocalizer with an `octoquad` configured and nothing answering: builds within 1 s, the `OctoQuad` fault raised in INIT within 5 s, INIT keeps looping, START throws naming the OctoQuad |
| Bench: Expansion Hub | an Expansion Hub | the same Lynx commands and odometry reads on each hub |
| Bench: Write Recovery | nothing | seven `failSafe()`s, 0 to 235 ms after a write: EnhancedMotor and EnhancedServo (back only on WriteCache's re-send) reach the hub again within the re-send period + 2 loops, raw devices within 2 loops; the worst seen |
| Bench: Drive Handover | nothing | `fl`'s SDK power matches Drivetrain's command from the 2nd loop after Pedro hands back |
| Hardware OpMode, then Bench: Power Readback | nothing | a stale Hardware-view Power edit doesn't drive `fl` under the next OpMode |
| Bench: Gamepad, Gamepad Linear | nothing | dashboard gamepad delivery, one edge per press, sticks resting within 1 s of silence |
| Bench: Limelight Fault | nothing | the `Limelight` fault with nothing answering: raised in INIT within 5 s, loop keeps running, collection ends SKIPPED |
| Bench: Limelight Hung | nothing | the real Camera against a fake Limelight that serves results and never answers a POST: the `Limelight` fault raised in INIT within the 15 s POST timeout + 2 s, INIT and the loop keep running, one POST and no retry; STOP during a hung POST isn't held up; the next INIT doesn't wait for it, its POSTs never overlap the old one (`reinit` reuses the stopped run's worker; only `reinit_fresh`, with a second worker on the shared blackboard semaphore as a reloaded LimelightSync would use, tests the shared sender), and its sync completes once the fake answers |
| Bench: Webcam Stop (absent) | nothing | INIT fails with CellTipCamera's "failed to open" |
| Bench: Webcam | a webcam | open/close times, per-frame and stream costs, frame age against `CellTipCamera.maxStalenessMs` |
| Bench: Webcam Stop | a webcam | STOP at several points with the webcam open |
| Bench: Webcam Thread Failure | a webcam | a camera-thread exception reaches the Error line within 2 s |
| Bench: Limelight | the Limelight | slothboard's proxy, `LimelightSync` time, INIT to first frame, POST costs, frame rate and latency |
| Bench: Planner Stop | nothing | STOP during 4- and 5-ball sliced plans: no app restart, hub-side STOP to idle within 900 ms, every 4-ball slice under 300 ms |
| Bench: Overlay NaN | nothing | a NaN in a field overlay doesn't kill dashboard telemetry |
| (config by name, Bench: Probe) | run-as | a shipped config saved with another shipped config's resource id loads its own XML; a name the build doesn't ship loads no config (both FAIL on a build without re-resolving by name at robot setup) |
| Bench: Freeze Dump | nothing | slothboard ignores a config switch during INIT; SET (3 runs), WRITE and DELETE from idle each restart the robot in the same process: no SDK `deadline expired ... EventLoop`, no Control Hub `Relaunching the RC`; thread dumps every 2 s from 3.5 s without an RC alive broadcast, a SIGQUIT and ART's dump of the frozen pid (runs last) |
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
  run-as does it switch through slothboard, which on slothboard 6165.6 or older freezes the app until the hub's
  watchdog relaunches it about 10 s later.
- A shipped `res/xml` config is stored by resource id, which an install that adds, renames or removes a `res/xml` file
  can renumber: the probe checks the stored id (from before the install with `--original-entry`) still names that XML,
  and the restore re-activates shipped configs by name, which fixes that hub only, so `summary.md`'s Restore line says
  when it did. Config by name picks two shipped configs with Control Hub devices alone, each with a device the other
  lacks, saves the first under the second's id (and `bench_unshipped` under that id), restarts the app and reads the
  hardware map's device names to tell which XML loaded; a load of no config reports no battery, which its switch allows.
- When the original config's name is a `bench_*` one (a hub left on `bench_probe` by an earlier run), the restore
  writes its XML back from `original_config.xml`: the run overwrote that file with its own. Every distinct XML the run
  writes under one name is kept in `logs/` (`<name>.xml`, `<name>.2.xml`, ...).
- Params and results go through the app's own `files/bench/` via run-as (`/sdcard/FIRST/bench/` only without it), and
  each write is read back: an OpMode once read a just-pushed `/sdcard` `params.json` as empty.
- Each config switch's logcat is in `logs/config_*.logcat.txt`, each step's in `logs/<step>.log`; the 8 MB log buffer
  keeps the hub's `Lights Hal` spam from pushing them out.
- The websocket is `ws://192.168.43.1:8000/`. NanoHTTPD drops a socket silent for 5 s, so the runner pings
  (`GET_ROBOT_STATUS` would put hub voltage reads in the timings). An OpMode list within 1 s of a socket opening is
  slothboard's on-connect copy, not a broadcast. A restarting app accepts a socket and drops it again within a second or
  two: the runner waits 0.5 s, then 1, 2 and up to 4 s before each reconnect while connections keep dying within 5 s,
  and between failed connects.
- SDK 12 only logs an OpMode's exception (`User code threw an uncaught exception`, tag `OpModeManager`). The runner takes
  it from slothboard's `RECEIVE_LOGCAT_ERRORS` (after the step began, until STOP), from a status back at `$Stop$Robot$`
  without BENCH DONE, or from adb logcat while waiting for INIT. `errorMessage` is the status's, else that line;
  `statusErrorMessage` is exactly the dashboard's Error line. The SDK logs that line about 0.45 s after the throw, so
  the runner's crash and Error-line times count from its own detection; Failure Path reports the hub's throw-to-line
  time from logcat separately. slothboard's logcat monitor sends its whole buffer again whenever it restarts,
  so `summary.md` counts its lines by (timestamp, message); the crash watch ignores lines from before the step.
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
- Bench: OctoQuad Absent holds INIT 6 s, then STARTs, expecting the guard's OctoQuad error.
- Bench: Pinpoint Absent passes on a fault whose key or message names the Pinpoint, an error naming it, or every pose
  read after `setPose` within 0.5 in and 1° of it.
- Bench: Write Recovery moves every command by 0.002 before each `failSafe()`, so all four devices write at a known
  moment, and reads WriteCache's re-send period from the class by reflection.
- Bench: Limelight Hung runs right after Bench: Limelight Fault on its config, repointing the config's `limelight` at
  the fake, which stays so until the app restarts (the switch back to the main config does that). Its fake outlives a
  `stop_hung` run in a static, and the runner INITs `reinit` right after with nothing between, so the INIT lands inside
  the stopped run's 15 s POST timeout; a slower turnaround FAILs the setup check, not the robot. `reinit_fresh` clears
  LimelightSync's private `running` flag by reflection, as a Sloth reload would leave it: a real deploySloth can't land
  in that window. A Bench: Hub after a crashed hung run, before an app restart, can't bind `127.0.0.1:15807`.
- Bench: Thread Lifecycle runs on the main config (`analog0`/`analog1`). Each run's result file is
  `thread_lifecycle_<mode>`, pulled to `raw/thread_lifecycle_<run>.json`; the next run's `leftoverAtInit` judges the one
  before (no hook runs after a crash), and a pid change between them skips that check. After each run the runner lists
  the app's threads over adb, trying `run-as ... cat /proc/<pid>/task/*/comm`, plain `cat`, `ps -T -p` and `ps -t`
  (`TransferCounter` is exactly comm's 15 characters); a listing during the first run is the positive control, and
  `summary.md` names the method that worked. Its Sloth reload deploys during a 600 s run: Sloth stages the load and
  releases it from the idle OpMode, so it lands after the runner's STOP.
- Bench: Edge Through Start needs the hold unbroken across START: a background sender (`GamepadHold`) repeats LB every
  50 ms from INIT until the RUNNING phase is going, since slothboard rests both pads after 500 ms without a message.
- Bench: Freeze Dump's watch thread outlives the OpMode (slothboard ignores a config switch unless idle): for each of
  SET, SET, SET, WRITE and DELETE the runner INITs and STOPs it (arming a fresh watch), sends the message from idle,
  waits up to 120 s for the robot to set up again in the same pid or for a new pid (else force-stops and starts the
  app itself), and pulls `freeze_watch.json` and every `freeze_dump_*.txt` into `raw/freeze/<run>/`, with the logcat
  in `logs/freeze_dump_<run>.log`. It copies `/data/anr/traces.txt` as soon as the frozen pid logs `Wrote stack
  traces`: the relaunched app's own startup ANR rewrites the file about 25 s after it starts. Lock owners come only
  from a `----- pid <frozen pid>` section dated after the change (the file also holds that pid's own startup ANR).
  A run passes only with the setup in the same pid and neither `deadline expired waiting for service termination:
  EventLoop` nor the Control Hub's `Relaunching the RC`: the watch's dumps can't decide it, since every robot setup
  waits 5 s for USB between two alive broadcasts, and the SDK's `awaiting shutdown` comes with any teardown over 2.5 s.
