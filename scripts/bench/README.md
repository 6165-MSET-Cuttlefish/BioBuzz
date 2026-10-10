# Control Hub bench

Measures the architecture on a bare Control Hub, with no Driver Station and no robot: loop times, bus costs,
telemetry and dashboard cost, the route planner and STOP while it plans, the Limelight poller, the Limelight fault
fallback, slothboard's Limelight proxy, the webcam's open/close, its frame clock and camera-thread failures, the
framework's stop and failure paths, dashboard gamepads, the floodgate pin and current limiter, the Pinpoint, the
OctoQuad through Pedro's OctoQuadLocalizer, the same Lynx commands and odometry reads on an Expansion Hub against the
Control Hub, Ivy's wall-clock waits, and the Sloth hot-load race and load signal. One command
runs everything the plugged-in hardware allows and leaves the results in `bench-results/<timestamp>/` at the repo
root (gitignored).

## Run it

1. Power the Control Hub from a 12 V battery. Unplug everything from Control Hub motor ports 0-3 and servo ports
   0-1: the benches drive them with real power. Motor ports 0-3 get the Drivetrain module's creep (up to 0.35) for
   about 95 s in Framework Loop and Limelight Fault (longer with `--interactive`), the Pedro follower (capped at 0.2)
   for 8 s each in Framework Loop and Raw Loop, and 0.1/-0.1 toggles in Bench: Hub; servo port 0 swings between 0.4
   and 0.6 in Bench: Hub; motor ports 0-1 get 0.2 and servo ports 0-1 go to 0.4 for 3 s in Write Recovery; motor
   ports 0-3 get a 0.3 stick and the Pedro follower (capped at 0.2) for 12 s in Drive Handover; motor port 0 gets 0.2
   for about 4 s from the dashboard's Hardware view in the hardware-view step. Never run it on a robot's hub with the
   drive plugged in. The runner refuses to start when the active hardware config names a motor or servo on those
   ports (with `<No Config Set>` or a leftover `bench_*` config active, when any config on the hub does); once the
   ports are empty, add `--ports-empty`. Keep Control Hub I2C bus 3 empty: Bench: OctoQuad Absent configures an
   OctoQuad there. If you have them, the Pinpoint goes on I2C bus 1 and the OctoQuad MK2 on I2C bus 2, and a REV
   Expansion Hub on the Control Hub's RS485 port with its own 12 V; the benches only read from the Expansion Hub and
   command nothing on it (see Optional hardware).
2. On the Mac, join the Control Hub's Wi-Fi network.
3. `adb connect 192.168.43.1:5555`
4. Full install once: in Android Studio pick the **TeamCode** run configuration and press **Run**. This puts the
   `Bench: ...` OpModes on the hub and gives the Sloth step a clean install to hot-load over. Wait for the Robot
   Controller app to come up.
5. Close any browser tab showing the dashboard: it asks for robot status every second, and each ask reads the hub's
   voltage over the same bus the benches time. Its Gamepad view also sends gamepad states the gamepad benches count.
6. From the repo root:

   ```
   python3 scripts/bench/run_bench.py
   ```

   It takes about 30 to 45 minutes (the Sloth step builds and hot-loads six times, then restarts the app; each
   Bench: Planner Stop trial whose STOP restarts the app adds about a minute; the OctoQuad and Expansion Hub steps
   add about 3 to 5 minutes, mostly their three config switches). Don't touch the hub while it
   runs; it prints what it is doing. Ctrl-C is safe: it still puts your hardware config back before exiting.
7. When it prints `Nothing to send back`, tell Claude it finished. Claude reads the folder itself.

The Mac's own `python3` (3.9 or newer) is enough; there is nothing to install. The Sloth step runs
`./gradlew :TeamCode:deploySloth` from the terminal, so first check that `./gradlew --version` works there; if it
doesn't, add `--no-sloth` and tell Claude.

### Optional hardware

- **Webcam benches** (Bench: Webcam, Bench: Webcam Stop and Bench: Webcam Thread Failure): plug any UVC webcam into
  a Control Hub USB port before starting. The Arducam OV9782 the robot uses is best, since that is what the numbers
  are for. It must stream 640x480 MJPEG, as WebcamSession requires.
- **Limelight bench**: plug the Limelight 3A into a Control Hub USB port (it is powered over USB) and wait for it to
  finish booting before starting. The bench puts `limelight/`'s ball pipeline on pipeline 4 itself, as INIT does. It
  also switches to pipeline 0 (change with `--other-pipeline N`, any of 0-9 but 4), has the runner switch it once
  through slothboard's Limelight proxy, and puts the Limelight back on the pipeline it found, with its poller stopped
  at the SDK's default 100 Hz.
- **Limelight unplug and replug** (`--interactive`, with the Limelight plugged in): after the Limelight bench, the
  runner runs **Bench: Limelight Fault** on the real Limelight and asks you in the terminal to pull its USB cable and
  press Enter, then plug it back in and press Enter. It records when the `Limelight` fault is raised and cleared (on
  the hub and on the dashboard) and how the loop runs meanwhile. The Limelight takes a while to boot after the
  replug; the runner waits up to 2 minutes for the fault to clear.
- **Pinpoint bench**: plug a goBILDA Pinpoint into the Control Hub's I2C bus 1 port with both odometry pods plugged
  into it (they can lie loose; keep them and the board still). Without pods it reports FAULT_NO_PODS_DETECTED, never
  READY. The bench reads its firmware and status and checks that a start pose set in INIT survives its recalibration.
  With no Pinpoint answering, Bench: Pinpoint Absent runs instead.
- **Pinpoint unplug and replug** (`--interactive`, with the Pinpoint plugged in): after the start-pose run, the runner
  runs **Bench: Pinpoint** in dropout mode and asks you in the terminal to pull the Pinpoint's I2C cable out of the
  Control Hub (leave the pods in) and press Enter, then plug it back in and press Enter. It records the status the SDK
  driver reports, what Pedro's pose does meanwhile (frozen on firmware v1/v2, zeroed on v3+), whether the board lost
  Pedro's pod settings (a reboot), the hub's I2C warning on the dashboard, and the loop period. It passes when the
  first loop with a status other than READY stops the OpMode with a Pinpoint error or raises a Pinpoint fault, and
  FAILs until the Pinpoint status fix, since nothing reads that status today.
- **OctoQuad bench**: plug an OctoQuad MK2 into the Control Hub's I2C bus 2 port with two odometry pods on its
  encoder ports 3 and 4 (they can lie loose; keep them and the board still). The bench reads its chip id and
  firmware, times its localizer reads, builds Pedro's OctoQuadLocalizer from OctoQuad Localizer Test's tuned values,
  counts reads that fail their CRC or aren't RUNNING, and checks that a start pose set in INIT survives. With no
  OctoQuad answering it says so and skips those checks. Bench: OctoQuad Absent runs either way: it needs only an
  empty I2C bus 3.
- **OctoQuad unplug and replug** (`--interactive`, with the OctoQuad plugged in): after the start-pose run, the runner
  runs **Bench: OctoQuad** in dropout mode and asks you in the terminal to pull the OctoQuad's I2C cable out of the
  Control Hub (leave the pods in) and press Enter, then plug it back in and press Enter. It records each read's
  localizer status and CRC, what Pedro's pose does meanwhile, when reads are valid again, the hub's I2C warning on the
  dashboard, and the loop period. It passes when the first invalid read stops the OpMode with an OctoQuad error or
  raises an OctoQuad fault, and FAILs today: Pedro's OctoQuadLocalizer drops an invalid read without a word and keeps
  the last pose (no fix is in flight).
- **Expansion Hub bench**: connect a REV Expansion Hub to the Control Hub's RS485 port and give it 12 V. The runner
  looks for it at module address 2 (`--expansion-address N` for another). With the Pinpoint and OctoQuad on Control Hub
  I2C buses 1 and 2, **Bench: Expansion Hub** times the same Lynx commands on each hub, the odometry reads, and one
  loop's reads with and without the Expansion Hub. It only reads: nothing is commanded on the Expansion Hub. With no
  Expansion Hub answering, a default run skips it and `--all` or `--expansion` stops.
- **Odometry on the Expansion Hub** (`--interactive`, with an Expansion Hub): after the Expansion Hub bench, the runner
  asks you to move the Pinpoint's I2C cable to the Expansion Hub's I2C port 1 and the OctoQuad's to its I2C port 2
  (pods stay in) and press Enter, runs the bench again on a config that names them there, and compares each read and
  the loop's reads with the Control Hub run. Then it asks you to move them back to Control Hub I2C ports 1 and 2. If
  the second run fails it still asks; on Ctrl-C it prints a reminder instead.
- **RC web page and the hub's clock** (`--interactive`, on a hub whose clock still reads about 1970: power-cycled,
  with no Driver Station connected and no RC web page opened since): during Bench: Clock the runner asks you to open
  http://192.168.43.1:8080 in the Mac's browser and press Enter once it loads. That page sets the hub's clock to the
  Mac's, as a Driver Station does when it connects, and the clock stays set until the hub powers off. On a hub whose
  clock is already set the runner skips the prompt.
- **Dashboard gamepad inputs** (`--interactive`, with the Mac's browser and a USB gamepad): after its own phases on
  **Bench: Gamepad Linear**, the runner keeps it running and asks you in the terminal to open the dashboard with the
  Custom layout and the Gamepad view, then in turn to hold W with keyboard input on, to drag the on-screen left stick
  to the top, and to bind a USB pad with Start+A and hold its left stick forward, each for 3 s, pressing Enter after
  each. Each one passes when the bench reads left_stick_y at -0.9 or below and never 0 between two full-stick loops.
  All three FAIL until slothboard 6165.6: before it the keyboard and on-screen stick send up as +1, and the Gamepad
  view's keepalive puts a rest state between a bound pad's frames. Close the tab afterwards, as it asks.

With nothing plugged in, those benches are skipped and the summary says so. The Limelight fault bench (`--llfault`)
needs nothing plugged in: it configures a `limelight` that nothing answers. Bench: OctoQuad Absent likewise configures
an `octoquad` on the empty I2C bus 3.

### Flags

| flag | runs |
|---|---|
| (none) | every bench the plugged-in hardware allows |
| `--all` | every bench; stops if no webcam, Limelight or Expansion Hub is plugged in |
| `--baseline` `--hub` `--loop` `--failure` `--gamepad` `--write-recovery` `--handover` `--hardware-view` `--webcam` `--webcam-failure` `--limelight` `--llfault` `--pinpoint` `--octoquad` `--expansion` `--absent` `--sloth` `--clock` `--planner-stop` `--overlay-nan` | just those (`--octoquad` runs Bench: OctoQuad and Bench: OctoQuad Absent) |
| `--expansion-address N` | the Expansion Hub's module address, 1-10 (default 2) |
| `--no-sloth` | skips the Sloth race (no Gradle builds) |
| `--interactive` | in the gamepad bench, has you drive the dashboard's Gamepad view (keyboard, on-screen stick, USB pad); after the Limelight bench, prompts you to unplug and replug the Limelight and times the fault; after Bench: Pinpoint and Bench: OctoQuad, prompts you to unplug and replug each; after Bench: Expansion Hub, has you move the Pinpoint and OctoQuad to the Expansion Hub and back; in Bench: Clock, on a hub whose clock is still unset, has you open the RC's web page (needs a terminal) |
| `--quick` | shorter phases, noisier numbers |
| `--keep-configs` | leaves the `bench_*` hardware configs on the hub |
| `--ports-empty` | runs even though a config on the hub names devices on motor ports 0-3 or servo ports 0-1 (only once they are unplugged) |
| `--serial HOST:PORT` / `--host IP` | a different adb device or hub address |

## What it changes on the hub

It writes `bench_*` hardware configs to `/sdcard/FIRST` with adb and switches between them by stopping the Robot
Controller app, pointing its active-config setting at the file with `run-as`, and starting it again. Switching
through the dashboard instead freezes the app until the Control Hub's watchdog relaunches it about 10 s later, so the
runner only does that when `run-as` can't edit the settings (an app not installed by the IDE's Run button). At the
end it puts back the config that was active, deletes the `bench_*` files with adb, and restarts the app once more so
the dashboard's config list drops them. It also enlarges the hub's log buffer to 8 MB (until the next reboot) so a
step's log survives the hub's constant `Lights Hal` messages. It never deletes the active config. The hardware-view
step runs slothboard's built-in Hardware OpMode and saves `fl`'s Power in the dashboard's config tree (0.2, then
0.0). After the Sloth step's last hot load and after Bench: Overlay NaN, the runner restarts the app once more, the
way it switches configs. A config shipped in `res/xml` is stored on the hub by its resource id, which an
install that adds, renames or removes a `res/xml` file can renumber, so the probe checks that the stored id still
names that XML (a FAIL means the hub was loading another config under that name) and the restore re-activates the
shipped config by name with the id this install gives it. If the active config at the start is neither a file in
`/sdcard/FIRST` nor still shipped (for example `<No Config Set>`), the hub is left on the empty `bench_probe` config. The configs put motors `fl`/`fr`/`bl`/`br`
on Control Hub motor ports 0-3, `benchServo` and `benchServo2` on servo ports 0 and 1, `floodgate` on analog port 0 and the embedded IMU as
`imu`; `bench_llfault` adds `cuttle_decode.xml`'s `limelight` EthernetDevice line, with `ipAddress` 192.0.2.1 when a
Limelight is plugged in so that nothing answers. `bench_probe` names an `Expansion Hub N` module at
`--expansion-address` (default 2) with no devices, so the probe can see whether one answers. `bench_octoquad` adds
`pinpoint` on Control Hub I2C bus 1 and an `OctoQuadFTC` named `octoquad` on bus 2 (plus the empty Expansion Hub
module when one answered); `bench_octoquad_absent` puts `octoquad` on bus 3; `bench_expansion` is `bench_octoquad` with
the Expansion Hub module; `bench_expansion_odo` (`--interactive` only) moves `pinpoint` and `octoquad` to the
Expansion Hub's I2C buses 1 and 2. No config puts a motor, servo or other output on the Expansion Hub. Bench: OctoQuad
Absent's STOP can make the SDK force-stop the OpMode or restart the app; if it is still stuck after STOP, the runner
force-stops the app and starts it again. The Limelight goes back to its original
pipeline. With `--interactive`, opening the RC's web page during Bench: Clock sets a hub clock that was still unset,
until the hub powers off. Parameters and result files go through the app's own `files/bench/` folder, which the runner reads and writes with `run-as` (`/sdcard/FIRST/bench/` only when run-as isn't available); they stay there and the runner copies them into the results folder.
If the run stops early it still restores the config. If even that fails (it prints RESTORE FAILED), open
http://192.168.43.1:8080/dash, add the Hardware Config view, and activate the config the message
names (the one that was active before the run); a Control Hub has no Configure Robot screen without a Driver Station. A run that stops before it writes the first `bench_*`
config leaves the hub untouched.

## The benches

| OpMode | needs | measures |
|---|---|---|
| Bench: Probe | nothing | the embedded IMU chip, attached webcams and Limelights, and whether an Expansion Hub answers at `--expansion-address`, so the config matches the hub |
| (no OpMode) config baseline | nothing | whether slothboard's `GET_CONFIG_BASELINE` reply lists `OptimizationToggles`, which the Config view's "show only modified" filter and baseline hints need; FAILs until slothboard 6165.6, before which the baseline is always empty on the robot |
| Bench: Hub | nothing | empty-OpMode loop overhead; SDK telemetry sends at the framework's DS interval (`OptimizationToggles.dsTransmissionIntervalMs`), 100 ms and every loop; bulk read, voltage, current, analog, motor, servo and I2C (IMU) call costs; the volts the floodgate pin (analog 0, nothing wired, as on the Betta bot) reads, min/mean/p99/max, which FAIL at 1.03 V or more because Drivetrain's current limiter takes that for 25 A; the framework's telemetry frame, field map, overlay and packet costs; allocation and GC; route-planner time for 0 to 5 balls, which FAILs if a 4-ball plan, the cold first one included, takes 800 ms or more; LLResult parse cost from a 0-ball script result up to 24 AprilTags plus 64 detections; `pipelineSwitch`/`updatePythonInputs`/`updateRobotOrientation` POSTs against a fake Limelight on loopback; the SDK Limelight poller off and at 100/200/250 Hz (the SDK clamps to 250) against that fake; how long a failing `pipelineSwitch` blocks |
| Bench: Framework Loop | nothing | the real EnhancedOpMode pipeline with the Drivetrain module and Pedro (its drive capped at 0.2) on empty ports, one telemetry/dashboard setting changed per phase (DS every loop through `OptimizationToggles.dsTransmissionIntervalMs = 0`), with the loop-profile breakdown; `ToggleGuard` restores the toggles afterwards. From its dashboard packets, that `Drivetrain Current Limiter Multiplier` stays 1.00 and `Drivetrain Floodgate Current (A)` under 25 on the unwired floodgate pin; a FAIL means the limiter would throttle the Betta bot's teleop until its Floodgate is wired or the limiter is turned off |
| Bench: Raw Loop | nothing | Close Flower Linear Auto's loop without the framework, for comparison (the same capped follower) |
| Bench: Failure Path | nothing | a hook throwing in each lifecycle stage: which module `stop()`s run, that a second failure is attached as Suppressed, that the runner sees the crash within 5 s, and whether the dashboard's Error line shows it within 2 s and clears at the next INIT. SDK 12 only logs a user-code exception, so the Error-line checks FAIL until the hub runs the EnhancedOpMode fix that copies crashes there, or slothboard 6165.6 |
| Bench: Clock | nothing | Ivy's `Commands.waitMs`, which times on `System.currentTimeMillis()`, behind `PathCommands.timeout` (the wrap-every-auto rule): it schedules `PathCommands.timeout` of a command that never ends at START and records when it ends by `System.nanoTime()`, every loop's wall clock against `System.nanoTime()` (jumps of 1 s or more), the hub's clock year before the step, and its drift against uptime since the runner connected. PASS if the timeout ends within 250 ms of 15 s. A hub that boots with its clock near 1970 and no Driver Station keeps a steady clock, so this passes on its own; with `--interactive` on such a hub you open the RC's web page mid-timeout, which moves the clock about 56 years forward, and the FAIL that follows is the expected demonstration until TeamCode's waits time on `System.nanoTime()` (no fix is in flight) |
| Bench: Pinpoint | a Pinpoint (else Pinpoint Absent runs) | through BettaConstants.create with the follower left idle, so nothing drives: the Pinpoint's firmware (device version), device id, status, loop frequency and pod settings, and the start-pose question. It runs the INIT sequence the autos use (create with its IMU recalibration, Robot's placeholder pose, then a start pose set in `initialize()`), holds INIT 2 s, STARTs and runs 3 s more; PASS if the pose stays within 0.1 in and 0.2 deg of the one set. A FAIL means the INIT pose doesn't survive the recalibration, so the autos that set it only in INIT (Close Flower, Mock Auto, the Linear autos) need the START re-seed Ball Collection Auto does; a PASS means only that this INIT sequence keeps the pose (a pose set at other delays after the recalibration isn't tried). A crash at INIT means a Pinpoint status check treats a non-READY INIT status as a failure |
| Bench: Pinpoint Absent | nothing (runs only when no Pinpoint answers) | whether BettaConstants' follower builds and updates with no Pinpoint plugged in |
| Bench: OctoQuad | an OctoQuad MK2 on Control Hub I2C bus 2 (else it reports none and its checks are skipped) | a follower built like BettaConstants.create but on Pedro's OctoQuadLocalizer, configured from OctoQuad Localizer Test's tuned values (Pedro takes one ticks-per-mm for both pods, so the bench sends the X value and records the Y value when it differs), with the follower left idle: the chip id and firmware; `readLocalizerData`, `readLocalizerDataAndAllEncoderData` and `getLocalizerStatus` cost; the status right after the build and how long the build took (Pedro waits for RUNNING); the heading axis and the parameters sent; every status change, CRC failures and invalid reads per loop, read from Pedro's own last read so the bench adds no I2C traffic; the loop period with valid and invalid reads. Then the start-pose question as in Bench: Pinpoint (INIT held 2 s, START, 3 s more; PASS within 0.1 in and 0.2 deg), and PASS if every read after the build was valid. A crash at INIT means Pedro's constructor threw; a step that never gets past INIT means the OctoQuad never reported RUNNING |
| Bench: OctoQuad Absent | an empty Control Hub I2C bus 3 | Pedro's OctoQuadLocalizer with `octoquad` configured where nothing answers: the runner INITs it, gives it 10 s to report BENCH DONE, then sends STOP. PASS if INIT fails within 5 s with an error naming the OctoQuad; FAILs today, since Pedro's constructor waits for RUNNING in a loop with no timeout and no STOP check, so a loose OctoQuad cable hangs INIT with no message (no fix is in flight). Records the chip id read on the empty bus, what STOP did (stopped, the SDK force-stopped the OpMode, the app restarted, or still stuck, after which the runner force-stops the app) and the runner's STOP to idle |
| Bench: Expansion Hub | an Expansion Hub (skipped without one) | reads only, with every hub's bulk caching off: on each hub `getInputVoltage`, `getBulkData` and `getCurrent`, each call timed and the hubs taken in turn so drift hits both alike; each odometry device's read (`readLocalizerData` for the OctoQuad, `update()` for the Pinpoint) and its connection info; one loop's reads with the Control Hub's bulk read alone, both hubs' bulk reads, and both plus the odometry. `summary.md` tabulates the Control Hub against the Expansion Hub per command, and with `--interactive` each odometry read and the loop's reads with the odometry on the Control Hub against on the Expansion Hub. The only check is that both hubs answered |
| Bench: Write Recovery | nothing | whether a wrapped device's commanded state reaches the hub again after the hub drops it: `fl` as an EnhancedMotor and `benchServo2` as an EnhancedServo next to `fr` and `benchServo` as raw SDK devices, all commanded the same value every loop, with one `LynxModule.failSafe()` after 1 s and the hub's power, enable and pulse-width registers read with Lynx commands that bypass the SDK's caches. PASS within 500 ms; the EnhancedMotor and EnhancedServo checks FAIL until the WriteCache fix that re-sends an unchanged value at least every 250 ms. The raw devices are the control: they come back on the next loop through the SDK's own re-send |
| Bench: Drive Handover | nothing | the Pedro-then-teleop hand back on the bench robot (software localizer, real Mecanum and Drivetrain on empty ports, the stick at 0.3, Pedro capped at 0.2), in four 3 s cases: a path cancelled from gameLoop, a command sequence that turns Drivetrain writes back on, and a collection-style command (BallCollection's resync at zero) cancelled from gameLoop, plus a control where that command ends itself inside the scheduler. Each loop compares fl's SDK power with Drivetrain's commanded power; PASS when they match from the 2nd loop after writes come back on. The three cases FAIL until the Drivetrain fix that resyncs both motor caches at the hand back; if the control FAILs, the readback is broken and the others aren't judged |
| Hardware OpMode, then Bench: Power Readback | nothing | the Hardware view's stale motor controls: the runner runs slothboard's built-in **Hardware** OpMode (INIT, START, 1 s, STOP), then INITs Bench: Power Readback, which sets `fl` to 0.2 and back once as a readback self-check and then only reads `fl`'s hub power register (and its SDK power) every loop. In INIT the runner saves `__hardware__` / Motors / fl / Power = 0.2 through a second websocket, as the Hardware view's Save does, and 0.0 two seconds after START. PASS if `fl` never leaves 0; FAILs until slothboard 6165.6, before which the stopped Hardware OpMode's Power control still drives the motor |
| Bench: Gamepad, Bench: Gamepad Linear | nothing | dashboard gamepads, sent by the runner as `RECEIVE_GAMEPAD_STATE`, through BioBuzz Tele's inputs (EdgeBooleanSupplier on left_bumper and left_trigger) in an EnhancedOpMode on the bench robot (Drivetrain writes off) and in a LinearOpMode that polls without sleeping: delivery in INIT and while running (the iterative bench FAILs on slothboard before 6165.5), one held press giving one edge and no loops reading the stick as 0, the sticks resting within 1 s once the runner goes quiet (slothboard's watchdog), and, measured but not judged, a second websocket sending a rest state every 200 ms while the runner drives, the way a second tab with the Gamepad view did before slothboard 6165.6. That last one isn't a check: every slothboard server so far applies every socket's state, and 6165.6 fixes it in the client, which a runner socket can't show |
| Bench: Limelight Fault | nothing | with a `limelight` EthernetDevice configured (cuttle_decode.xml's line; with a Limelight plugged in, an address nothing answers) and nothing answering, through the real Camera and Drivetrain, 6 s in INIT then 15 s running: that the `Limelight` fault is raised in INIT within 5 s and stays raised, its message, that the loop keeps running with the fault active (at least 10 loops/s, p95 at most 50 ms, max at most 500 ms), that the fault's row leads every dashboard packet and is drawn red on the field overlay (from what the runner receives over the websocket), that ball vision reports no balls, and that a ball collection scheduled at START ends SKIPPED (`Limelight: ...`) with Drivetrain writes back on (the result JSON's `collection` key). Fault raises and clears are saved as they happen; message updates are only counted. The Driver Station side isn't checked, since there is no DS |
| Bench: Webcam Stop (webcam absent) | nothing | that INIT fails with CellTipCamera's "failed to open" error when the webcam is missing |
| Bench: Webcam | webcam | open, first-frame and close times; CellTipPipeline per-frame cost with detection on and off; each frame's capture age (`System.nanoTime()` at processFrame minus EasyOpenCV's `captureTimeNanos`), which must not go negative, or CellTipCamera's one-sided freshness gate is comparing two clocks, and whose p95 must stay under `CellTipCamera.maxStalenessMs` as the hub has it, or every cell-tip verdict reads stale; dashboard-stream and DS-preview cost; WebcamControls write stalls; `WebcamSession.close()` at several delays after creation |
| Bench: Webcam Stop | webcam | the framework's real stop path with the webcam, STOP pressed at several points in INIT and after START, watched for the SDK force-stopping the OpMode or restarting the app |
| Bench: Webcam Thread Failure | webcam | a pipeline inside the real WebcamSession that throws on its 30th frame, on the camera thread, after START: whether the exception's text (with a per-run nonce, so slothboard's replay of old log lines can't match) reaches the dashboard within 2 s of the OpMode stopping, as the status's `errorMessage` (the Error line) or as an OpModeManager line slothboard pushes (`RECEIVE_LOGCAT_ERRORS`), and which tags adb's logcat has it under. FAILs until the WebcamSession fix that rethrows a camera-thread exception from `update()` on the OpMode thread, or slothboard 6165.6's crash text in `errorMessage`: today EasyOpenCV stops the OpMode as if STOP were pressed and logs the exception only under the OpenCvCamera tag |
| Bench: Limelight | Limelight | first, slothboard's Limelight proxy: with the Limelight put on the ball pipeline directly, the runner POSTs `/pipeline-switch?index=N` (N is `--other-pipeline`, or 1 when that is 0, since a switch that lost its index might read as 0) to the hub's port 5807 and GETs `/status` through it, while the bench watches the Limelight's own `/status`; PASS if the pipeline changed, which FAILs until slothboard 6165.6, whose proxy keeps the query string; the bench then switches back directly. Then `getStatus` latency; how long `LimelightSync` takes to put the ball pipeline on it (park, upload, confirm; twice) and INIT to the first frame; what results report right after a switch; what the blocking POSTs (`pipelineSwitch`, same index included, `updatePythonInputs`, `updateRobotOrientation`) cost the thread that sends them with the poller running; the `ts` clock against the hub's; real-result parse cost; the poller off and at 100/200/250 Hz; LimelightBallSource when the pipeline is switched behind its back |
| Bench: Planner Stop | nothing | STOP pressed while the route planner replans on the OpMode thread in INIT, the way Vision Ball Collection does while the balls keep moving (every 250 ms, back to back once a plan takes longer), on the crafted layout with balls on both sides of the HIVE rail, 4 and then 5 balls: three trials each, STOP sent 0.05, 0.3 and 0.6 s after the runner sees a plan finish. Records every plan's ms on the hub (p50/p95/max, and each trial's first plan), where STOP landed and how much of the plan was left (from the SDK's `stopRequested`, polled every 1 ms), STOP to `stop()`, the runner's STOP to idle, and whether the app restarted. Per ball count, PASS if no trial restarts the app (a FAIL means a plan plus `stop()` outlasted the SDK's 900 ms stop window) and if the runner sees every STOP reach idle within 900 ms; at 4 balls, also if the per-plan p99 is under 500 ms. The plan times decide the fix (none is in flight). Runs just before Overlay NaN |
| Bench: Overlay NaN | nothing | runs last: an EnhancedOpMode on the bench robot that draws one NaN circle in every field overlay for 1 s (one loop's alone can be dropped unsent, since slothboard keeps only the last overlay of each batch), then runs 5 s more. PASS if its telemetry still reaches the runner 5 s later; FAILs until slothboard 6165.6, before which the NaN kills slothboard's telemetry thread until the app restarts, so the runner restarts the app after this step either way |
| Bench: Sloth Probe | nothing (runs Gradle) | INIT sent the moment `deploySloth` has pushed its jar (while the hub is still loading it), and 0 to 5 s after `deploySloth` finishes, watched for an app restart; and every OpMode list slothboard broadcasts from the push until logcat shows `Processed Sloth Load` (plus 3 s), including the list a socket the runner reconnects after the push gets on opening, which a dashboard tab acts on the same way. That check FAILs if any list lacks `Bench: Sloth Probe` or offers `Enable/Disable Dashboard`, which is expected until slothboard 6165.6: mid-load it broadcasts only its own two OpModes, and the dashboard's picker falls to the toggle that disables the dashboard. Runs where the app restarted or no load reached logcat within 30 s aren't judged. Each push's load signal, from logcat: seconds from the push to Sloth's `Staged Sloth Load` and to its `Processed Sloth Load` (or `Cancelled Sloth Load` or a failure); FAIL if a load in a run that didn't restart the app ends any other way within 30 s, since deploySloth reports success either way and nothing shows a landed load without a Driver Station. Then which clock the hot-loaded jar's mtime carries (the Mac's through adb push, or the hub's through Sloth's `setLastModified`), the package `lastUpdateTime` Sloth compares it with at boot, and whether the jar survives an app restart: the runner restarts the app as a config switch does and INITs Bench: Sloth Probe again; PASS if it still runs the hot-loaded jar, FAIL if the restart dropped the jar as outdated and the hub went back to the installed code. No fix is in flight for either |

The OpModes also run by hand from the dashboard, but they expect the runner's hardware config and write their
results only to the hub.

## Notes for changing the bench

- Which config is active comes from the hub, not from the config lists slothboard broadcasts: a restart can drop
  the socket and lose the list naming the new config. The runner reads and writes the app's own setting with
  `run-as com.qualcomm.ftcrobotcontroller` (it needs the debuggable APK the IDE's Run button installs) and, if
  run-as fails, reads the list slothboard sends a fresh socket and switches through slothboard. A switch is done
  when logcat shows `======= INIT FINISH =======`; each switch's logcat is saved in `logs/config_*.logcat.txt`, and
  every step's logcat in `logs/<step>.log`, a failed step's included.
- An OpMode once read the `params.json` the runner had just pushed to `/sdcard/FIRST/bench` as empty, so the runner writes parameters and reads results in the app's own storage through `run-as`, and checks each write by reading it back.
- The active-config setting is `RobotConfigFile`'s Gson form under `pref_hardware_config_filename` in the app's
  default SharedPreferences; the app must be stopped while it is written, or it saves its own copy over it.
- The runner drives slothboard's websocket at `ws://192.168.43.1:8000/`. NanoHTTPD drops a socket that is silent for
  5 s, so the runner pings rather than sending `GET_ROBOT_STATUS`, whose hub voltage reads would land in the timings.
- SDK 12 never puts an OpMode's exception in the status's `errorMessage`; it only logs `User code threw an uncaught
  exception` and the stack trace under the `OpModeManager` tag. The runner reads those lines from slothboard's
  `RECEIVE_LOGCAT_ERRORS` push (timestamped after the step began, and only until the runner sends STOP), and also
  counts a crash when the status goes back to `$Stop$Robot$` without BENCH DONE, or, while it waits for INIT, when adb's
  logcat has the line. A result's `errorMessage` is the status's, else that exception line; `statusErrorMessage` is
  exactly what the dashboard's Error line showed.
- The Robot Controller app can restart mid-run (a `stop()` that overruns the SDK's stop watchdog, the Control Hub's
  watchdog). A relaunch takes about 40 s, and while slothboard boots its status names no OpMode and a status request or
  STOP can close the socket. The runner re-sends status requests, and wherever it stops an OpMode, finishes a step,
  starts the next one or recovers from a failed bench, it waits for the app to come back up idle instead of failing;
  `summary.md` lists the restarts it waited out.
- A step's completion signal is its `BENCH DONE` packet, but `BenchReport` writes the result file and logs `BENCH DONE
  <file>: ...` under the `BENCH` tag first. When the dashboard has been quiet for a while, the runner looks for that
  line in logcat every 5 s, finishes the step from it and the file, and `summary.md` says the step's dashboard
  telemetry was lost. If no telemetry packet at all arrives within 5 s of INIT, it logs why (the OpMode never started
  its report, slothboard sent nothing, or only slothboard's INIT clear arrived, which means its telemetry thread died)
  and restarts the app after the step so the next one has telemetry again.
- slothboard's Limelight proxy listens on `*:5807` from app start, and Linux refuses a `127.0.0.1:5807` bind beside it,
  so Bench: Hub's fake Limelight listens on `127.0.0.1:15807` and the bench points each loopback `Limelight3A`'s private
  `baseUrl` at it by reflection (the SDK hard-codes `:5807`); the refused case uses a loopback port nothing listens on.
  INIT fails if the fake can't bind. The runner records which of the two ports are listening (`/proc/net/tcp6`) at
  connect and after Bench: Hub.
- With no Driver Station connected, the SDK drops a telemetry send before it serializes anything, so the DS-frame
  costs (Bench: Hub's `dsTransmission*` and DS frame rows, Framework Loop's DS frames) leave out the serialization and
  UDP send a match pays; Bench: Hub and Framework Loop record whether a Driver Station was connected and `summary.md`
  labels those rows. Bench: Hub's `dsTransmission*` phases hold the SDK's own post-`loop()` `telemetry.update()` off
  with a long interval, so only the timed sends happen.
- The probe identifies the embedded IMU by `LynxModuleImuType.name()` (`BHI260`; `toString()` is `BHI260AP`), and the
  run stops if it can't, since the bench configs need the right IMU tag.
- The Sloth race runs `./gradlew :TeamCode:deploySloth` six times and sends the first INIT the moment it prints
  `pushed jar`. Sloth's Load plugin runs bare `adb` commands, so the runner sets `ANDROID_SERIAL` to its own device;
  with more than one adb device listed (the hub over Wi-Fi and USB, a phone) it logs a warning. deploySloth waits on the Sloth lock with no timeout, so the runner kills it after 600 s. The runner writes `BENCH sloth deploy <run>` to the hub's logcat before each deploySloth and `BENCH sloth pushed <run>` when it prints `pushed jar`; a load's timings are the hub's logcat timestamps from the second mark, taken only from lines after the first.
- The gamepad benches take their phase from the runner's messages: gamepad2's right trigger carries the phase number
  / 100 and its left trigger a message counter, so a rest state from another socket or from slothboard's watchdog
  (all zero) leaves the phase as it was. Phase 99 ends the bench. Per phase they count loops, deliveries, left-bumper
  edges, loops at full stick, loops reading the stick as exactly 0 between two at full stick, and the time from the
  last full-stick message to the stick reading 0, into `gamepad.json` and `gamepad_linear.json`.
- The Sloth step's OpMode-list check ignores a list that arrives within 1 s of the runner's socket opening: that is
  slothboard's on-connect copy of its current list, not a broadcast.
- Bench: OctoQuad Absent hangs in `createRobot()` before its first telemetry packet, so the runner drives it outside
  `run_opmode()`, which would take that silence for dead dashboard telemetry and restart the app: INIT, up to 10 s
  for BENCH DONE, STOP, then the same restart handling as any step.
- Results land in `bench-results/<timestamp>/`; `summary.md` is the part Claude reads.
