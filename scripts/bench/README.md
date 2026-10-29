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
3. Full install (Android Studio, **TeamCode** run configuration, **Run**) and wait for the RC app. Do it again after
   any change to `architecture/hardware/photon/`: that code is pinned, so deploySloth never changes it on the hub, and
   the Photon block fails its first check when the installed copy doesn't match the sources.
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
| a REV Expansion Hub on 12 V, **nothing on its motor ports 0-3 or servo ports 0-1** (the Photon block drives them) | Control Hub USB to its mini-USB, no RS485 cable | the Photon block with `--usb-expansion` |
| the same Expansion Hub, same empty ports | RS485 only, address 2 (`--expansion-address N`) | the Photon block with `--photon-rs485` |

`--interactive` adds terminal prompts: pull and replug the Limelight, Pinpoint and OctoQuad cables (pods stay in); move
the Pinpoint and OctoQuad to the Expansion Hub's I2C ports 1 and 2 and back; open http://192.168.43.1:8080 during
Bench: Clock on a hub whose clock still reads about 1970; drive the dashboard's Gamepad view with the keyboard, the
on-screen stick and a USB pad (close the tab afterwards).

### Flags

| flag | does |
|---|---|
| `--ports-empty` | runs although a config names devices on motor ports 0-3 or servo ports 0-1 (only once they're unplugged) |
| `--interactive` | the prompts above |
| `--no-sloth` | skips the Sloth race and the Photon block's deploySloth step (no Gradle builds) |
| `--photon` | the Photon block (below); a run without bench flags, or with `--all`, leaves it out |
| `--usb-expansion` / `--photon-rs485` | the Photon block with the Expansion Hub on USB / on RS485 (implies `--photon`) |
| `--photon-soak` | the Photon block's long soak, 200 STOP trials per condition and more lost-write trials (implies `--photon`) |
| `--photon-stop-trials N`, `--photon-seed N` | STOP trials per condition (default 40); the seed for the STOP delays (default: the time, logged) |
| `--quick` | shorter phases, noisier numbers |
| `--all` | every bench but the Photon block; stops if no webcam, Limelight or Expansion Hub is plugged in |
| `--keep-configs` | leaves the `bench_*` hardware configs on the hub |

`--help` lists the rest: one flag per bench (`--hub`, `--loop`, `--limelight`, ...), `--serial`, `--host`,
`--other-pipeline`.

## The Photon block

`--photon` checks `architecture/hardware/photon/` against everything the robot code could do with a hub, and ends its
section of `summary.md` with a **Photon verdict**, for the wiring it ran on:

- **SAFE TO TRY ON THE ROBOT**: every Photon check passed, both STOP negative controls fired, the routing check saw both
  hub swap orders, the soak met its loss bound, there were 200 STOP trials per condition and, on USB wiring, the cable
  pulls passed. Only the go runs in the table below can say it.
- **PASSED THIS RUN, NOT YET SAFE**: nothing failed, but the run left out something the go needs; it lists what.
- **NOT SAFE YET**: it lists the blockers.

Each test's section says what it proves, the numbers, and what a FAIL means.

**Setup.** Control Hub on 12 V; the Expansion Hub powered from the Control Hub's second XT30 port; nothing on either
hub's motor ports 0-3 or servo ports 0-1 (the block drives them at 0.15-0.3 and moves servos between 0.35 and 0.6; it
prints exactly what before it starts). For `--usb-expansion` the Expansion Hub is on a USB cable only (no RS485 cable);
for `--photon-rs485` on the RS485 cable only. Pass `--ports-empty` once the ports are empty: a config on the hub that
names devices on either hub's bench ports stops the run without it, and from a terminal it asks you to confirm. The
stock bench then drives each motor port at 0.1 for 0.2 s and refuses to go on if one draws over 0.1 A. It can't probe
servo ports, and a free-spinning motor may draw less than that: check every bench port by eye. `--photon-i2c` adds a
Pinpoint (I2C bus 1) and an OctoQuad (bus 2) to the read phase.

| run | command | about |
|---|---|---|
| USB Expansion Hub, short (PASSED at best) | `python3 scripts/bench/run_bench.py --photon --usb-expansion --ports-empty` | 55 min |
| RS485 Expansion Hub, short (recable first) | `python3 scripts/bench/run_bench.py --photon --photon-rs485 --ports-empty` | 55 min |
| **go, USB wiring** | `python3 scripts/bench/run_bench.py --photon --usb-expansion --photon-soak --interactive --ports-empty` | 6-7 h, 4 prompts at the end |
| **go, RS485 wiring** | `python3 scripts/bench/run_bench.py --photon --photon-rs485 --photon-soak --ports-empty` | 6-7 h |

The steps, in order (each OpMode is `Bench: Photon ...`):

1. A config switch restarts the app; **Stock** runs first, so its hubs are the SDK's own: the installed Photon's
   source hash (from `BuildConfig`) against the checkout's, the port probe, then the same write workload as Photon's
   with exact read-backs, and a check that `PhotonCore.enable()` throws outside `@Photon`.
2. **Routing**, the first Photon INIT: every `LynxUsbDeviceImpl.knownModules` entry is the hardware map's
   `PhotonLynxModule`, every motor/servo controller points at it, nothing is in flight; then each hub is marked not
   responding and must clear within 50 ms of its next command, Photon off and on; then a parallel burst.
3. **Photon**: write block per hub and loop period off and at 8/4/2 parallel (a 10 ms poller thread off and at 8),
   undrained bursts at 8/2/1, three writes per channel back to back (the last must win), SDK-path enable toggles between
   parallel writes, bulk/voltage reads, servo writes from a second thread while the OpMode thread writes and reads back
   the motors and a third toggles Photon, 3000 writes at a limit of 1 under a 50 ms stall watchdog, and a
   production-rate soak (60 s; with `--photon-soak` up to 1e7 writes per hub, about 3 h). Every phase must have zero
   wrong read-backs, finite drains under 50 ms, nothing left after (Photon's slots and map, and its stand-ins in the
   SDK's own command map), in-flight never over the limit by either count, no write pending 100 ms or more, no
   retransmit, abandoned write, NACK or unexpected fallback, and exactly one parallel write per write it should have
   parallelized.
4. **Faults**: a tap on the hub's USB device swallows chosen parallel writes and their retransmits, so their ACK never
   comes: each must be retransmitted at ~100 and ~200 ms, given up at ~250 ms, never block the caller, free its slot and
   its SDK entry, and the next identical write must reach the hub. Single lost writes, then 13 per in-flight depth 0-7
   per hub (25 with `--photon-soak`) with that many other writes in flight and the hub kept busy, then 10 per hub
   followed by a newer write to the same channel, which must end superseded (never re-sent or given up); a full limit
   of lost writes plus one more write; a USB write that throws; and a hub reset (a raw fail-safe) timed Photon off and
   on, labelled by whether the SDK learned of the reset from the hub.
5. **Stop Race** (6 runs per hub): STOP's gate raised the instant a motor write goes out; the motor must stay off.
6. **Stop** and **Stop Linear**: 40 STOPs each (200 with `--photon-soak`) at a random 1-3 s, seeded (`--photon-seed`),
   iterative at 8 parallel and off, linear at 8 (which keeps writing for 200 ms after STOP). `stop()` reads every
   channel on both hubs after the SDK's fail-safe and before zeroing the motors, and a sampler thread reads them until
   1.5 s after the OpMode ends: any motor enabled above 1% power, or servo enabled, in that read or in a sweep after the
   first one that read every channel safe, fails, and so does a trial that crashed or restarted the app. Two negative
   controls, five runs each, re-energize motor port 0 of each hub after the fail-safe on purpose: once, the way a real
   Photon failure would (`stop()`'s read must catch every one), and for 150 ms (the sampler must catch every one).
7. Photon, non-Photon, Photon, **Crash** (throws with writes in flight, two per hub swallowed so they're never
   acknowledged), Photon: every INIT must start clean, with each swallowed write finished (superseded, given up or
   cleared at INIT) and nothing left in the SDK's command map.
8. deploySloth (skipped with `--no-sloth`), then a Photon OpMode must still enable the same pinned `PhotonCore`.
9. App restarts with **Routing** after each, at least 8 and until both hub swap orders are seen (at most 16).
10. With `--interactive` (USB wiring only), **Replug** four times: the Expansion Hub's USB cable, then its power, each
    once with Photon off and once on. Each time the runner names the cable and waits for Enter, starts the bench, then
    prints `>>> PULL NOW`: for USB, unplug the cable, count to two and plug it back in; for power, pull its XT30 lead,
    count to one and plug it back in. Its motor ports are empty, so nothing moves; the step ends about 3 s after the hub
    is back (or after 90 s). With Photon the loop must stall no longer than without (+50 ms) and no single write call
    over 300 ms, its Expansion Hub writes must fall back while the hub is gone, the Control Hub's must stay right
    throughout, and the Expansion Hub's must be right again within 1 s.
11. Framework Loop off and on, and Write Recovery on (their `Photon` variants), then an app restart so the benches after
    the block get the SDK's own hubs.

The SDK warnings in logcat are judged for every step but the cable pulls, except the ones a step causes on purpose (the
lost-write and reset phases, the routing bench's own not-responding test) and a hub answering the firmware-version read
late after a restart, which Bench: Probe shows without Photon too (counted, not judged); a step whose log is empty or
lacks one of its phase markers fails. The soak's step streams logcat to a file instead of relying on the hub's ring
buffer.

With `--photon-rs485`, Bench: Photon adds a shared-link phase, off and on, with the poller running: the Control Hub's
writes (parallel when on) and the RS485 Expansion Hub's (which must never go parallel) every loop, with exact
read-backs on both and no SDK lock warning.

## What it changes on the hub

It writes `bench_*` hardware configs to `/sdcard/FIRST` and switches between them by restarting the RC app; at the end
it re-activates the config that was active, deletes the `bench_*` files and restarts the app once more. The configs put
`fl`/`fr`/`bl`/`br` on motor ports 0-3, `benchServo`/`benchServo2` on servo ports 0-1 and `floodgate` on analog 0;
only the Photon block's config also names the Expansion Hub's motor ports 0-3 (`efl`/`efr`/`ebl`/`ebr`) and servo ports
0-1 (`eServo`/`eServo2`). It also raises the hub's log buffer to 8 MB until the next reboot. It never
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
| Bench: Photon Stock, Photon, Photon Faults, Photon Routing, Photon Stop, Photon Stop Linear, Photon Stop Race, Photon Crash, Photon Replug, Framework Loop Photon, Write Recovery Photon (`--photon`) | see the Photon block | see the Photon block |
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
