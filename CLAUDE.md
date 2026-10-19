# CLAUDE.md

## Project context

FTC 6165 MSET Cuttlefish, 2026–27 BIOBUZZ season.

- Game-specific code goes in `biobuzz/`, `modules/`, `opmodes/`; `architecture/` stays game-agnostic. Field constants come from the game manual, never guesses. New Java files go in an existing package.
- The top level holds only Gradle and repo files, `TeamCode/`, `limelight/` (runs on the Limelight), `scripts/` and `docs/`.
- No BioBuzz robot yet: everything runs on the Betta bot (a sister team's DECODE bot, `pedro/BettaConstants`).
- `decode/` is last season's code for 6165's Cuttle bot only (`cuttle_decode.xml`). It shares `modules/Drivetrain`.
- In competition OpModes, automated ball detection and pathing are Autonomous-only and stay on our alliance's half.
- When an auto's poses change, re-export its `docs/paths/<OpMode>.pp` (visualizer.pedropathing.com).

## Keep this file fresh

Claude updates this file unasked, in the same commit as any change it describes. Only current rules, locations and footguns the code doesn't show; no history, no value lists.

## Build & deploy

IDE run configs are per laptop (`.idea/` isn't committed); set them up per Sloth's README.

- **TeamCode** (Run button, full install): first deploy, after a hub wipe or Sloth bump, and after any change outside `org.firstinspires.ftc.teamcode` (dependencies, manifest, `res/`, `assets/`, `robotcontroller/internal/`).
- **deploySloth** (hot reload): everything else. It returns when the push ends, not the load: wait for logcat's `Processed Sloth Load` (or the RC screen or DS; about 5 s) before redeploying or pressing INIT, or the app restarts or the INIT never runs. An `EnhancedOpMode`'s `Code` status line names the Sloth jar it runs, or `installed APK`.
- The hub's clock boots near 1970 until a DS connects or a browser opens `http://192.168.43.1:8080`. While it reads earlier than the APK's install time, Sloth deletes each load at the next app restart (logcat `removing outdated sloth load`) and the installed APK's code runs.
- A deploySloth replaces every `HardwareDevice` object, so state kept in one (e.g. the Limelight poll rate) doesn't carry across it.
- An old hot-load overriding new code: run the `removeSlothRemote` task alone, after `adb connect 192.168.43.1` (it never connects and silently does nothing without a device).
- With no adb device connected, deploySloth connects to `192.168.43.1` and ends with a bare `adb disconnect`, dropping every Wi-Fi adb connection: `adb connect` first to stay connected. With more than one adb device, set `ANDROID_SERIAL`.
- No unit tests. Verify with `./gradlew :TeamCode:compileDebugJavaWithJavac` (`:TeamCode:assembleDebug` for the APK).

## Dependencies

- Sloth, the `dev.frozenmilk.sinister.sloth.load` plugin and slothboard's `{sloth}` version prefix must match; slothboard pins Sloth strictly.
- slothboard is the committed `TeamCode/libs/m2/`, built from the same-named tag on `6165-MSET-Cuttlefish/slothboard`. To bump: tag a new build there, replace those files and the version string. Keep `exclude group: 'com.acmerobotics.dashboard'` (same Java packages).
- Close the dashboard tab when timing loops: it reads every hub's voltage each second.
- A loop spike every ~500 ms while motors are written is the SDK re-reading each motor's mode and re-sending its enable, not our code.

Pedro Pathing 3:

- `Pose.heading()` is in [0, 2π).
- Every `Paths` path needs a heading interpolator; a missing one compiles and throws only at run time.
- `BezierCurve.length()` (so `parameter()`, `remainingDistance()`) can under-report S-curves and loops down to the chord: never use it for timeouts or scoring. `PathCommands.followUntilRemaining` hands over early on such curves.
- `follow`/`followThrough` brake at the end; legs keep speed through a join only as segments of one `Paths.path(...)` or via a `followUntilRemaining` handover before braking.
- A heading jump over 11.25° at a join gives turning priority over driving; `architecture/auto/PathHeadings` gives headings that avoid it.
- A lambda passed to `path.heading(...)` gets path completion, not the curve's `t` (per piece inside `piecewise`); wrap it in `Interpolator.geometric(...)` to get `t`. `reverse()` drops `geometric`, so `reverseTangent()` and `facingPoint(...).reverse()` are wrong on Bezier curves: turn onto a tangent with `PathHeadings.linearThenTangent`/`holdThenTangent`/`tangentThenLinear`.

## Framework layout

`architecture/` is the portable framework; `auto/` holds field geometry and alliance mapping, `planner/` the ball planner, and `command/` the Ivy factories (`StateCommands`, `PathCommands`). A `Robot` subclass's `createFollower(HardwareMap)` and `initializeGameModules()` run inside `Robot`'s constructor, before the subclass's field initializers.

**Poses and alliance**

- Author poses for RED with `FieldPose.forAlliance(x, y, heading)`; it maps them to BLUE through `FieldConfig.symmetry()`. A bare radian passed to a path isn't mapped.
- `FieldPose` throws until the symmetry is set, and `EnhancedOpMode` clears it and the dashboard's field image at every INIT: a BIOBUZZ `Robot` constructor calls `BioBuzzField.configure()`, a bare SDK OpMode calls `FieldConfig.setSymmetry(...)` itself. The field image is a render in Pedro's frame drawn at `FieldConfig.fieldWidthInches`; a new one must match.
- `Robot` resets the follower to a placeholder pose at every INIT: set the start with `robot.follower.setPose(...)` in `initialize()`, then build paths.
- `Context.allianceColor` is set only on the dashboard or in `Context.java`, never by an OpMode, and resets on an app restart or Sloth reload.

**Ball routing**

- Planner (`architecture/planner/`): `RouteOptimizer.findOptimalRoute` → `RoutePathBuilder.build` → `RouteRun`. Every leg keeps the whole `RobotShape` footprint (frame plus intake) inside the area; `biobuzz/RobotGeometry` is the Betta bot's.
- No alliance mapping: pass mapped field coordinates, `BioBuzzField.ownHalf(margin)` as the area and, on the field, `BioBuzzField.hiveRails()` as the only obstacles. Bad input throws, so check a robot-sourced start with `RouteOptimizer.poseProblem` first.
- Pedro switches to the next piece of a compound path as soon as it would start braking (`ForesightConfig.pathSkip`, on by default), cutting sharp turns and reversals early, so don't merge `Plan.steps` into one compound path.
- A curve that stops dead mid-path (a straight cubic with handles the full chord long) makes Pedro's follower throw there.

**Faults** (`core/Faults`) are named, recoverable errors, for the Limelight only (see Fail fast). Keys are global and cleared at every `EnhancedOpMode` INIT, which also shows them; a bare `OpMode`/`LinearOpMode` must show them itself.

**Game code.** Game commands go in `RobotActions` (`robot.actions`). `Camera` starts ENABLED, polling the Limelight and able to raise its fault, so an OpMode that doesn't use ball vision activates `Camera.VisionState.DISABLED` in `initialize()`.

**Drivetrain** needs `withFollower(follower)`: without it, field-centric drive and heading lock throw and `write()` silently stops guarding against Pedro. An OpMode that follows paths calls `robot.drivetrain.setWriteEnabled(false)`; `BallCollection` does this itself.

**Ball collection** (`RobotActions.collectBalls`) waits only `visionWaitMs` for a fresh frame, so a START during the INIT Limelight sync can end it SKIPPED. `BallCollection.keepIn`/`obstacles`/`configuredPose` give other OpModes its `onField` switch.

**Vision** (`modules/vision/`). `TrackedBall` is camera-relative, `FieldBall` field-relative. A stale field ball keeps its last position with `visible()` false: check `visible()` before chasing one.

**Ball detection** is `limelight/ball_contour_snapscript.py` plus `ball_pipeline.vpr`, compiled into the generated `LimelightFiles` (so deploySloth carries edits) and uploaded at every INIT by `LimelightSync` while `syncOnInit` is on. SnapScripts get the raw image: paste `eocvsim/homography`'s whole Python block into the script, and after a lens recalibration update `LENS_*` in both the script and `HomographyCalculationPipeline`.

**Limelight faults.** A Limelight problem raises the `Limelight` fault and stales the frame until it recovers (`Camera.limelightProblem()` says why); a web-editor copy of the script faults unless `syncOnInit` is off. The sync isn't retried: a failed sync, or the Limelight landing on another pipeline (reboot, web-UI switch), faults until the next INIT.

**Cell tip** (`CellTipCamera`). Mount the webcam image-upright, facing the launch direction (an upright tag shows roll ≈ 0 on the stream overlay); a camera pointed away reads tipped. Set the alliance before INIT (tag ids are read then). Never call `CellTipCamera.stop()` yourself: it's irreversible.

## OpMode lifecycle

Modules are found through fields reachable from the OpMode and `robot` (a second pass catches ones created in `initialize()`), never locals.

In INIT, `initializeLoop()` replaces `gameLoop()` and module writes stay off unless `shouldWriteDuringInit()`, but the follower and Ivy scheduler still run: a follow, hold or path command scheduled before `onStart()` drives the robot in INIT.

- Telemetry lines go in `telemetry()`, field drawing in `dashboardOverlay(Canvas)`. Never call `telemetry.update()` in an `EnhancedOpMode`: it closes the DS frame and later DS lines are dropped. Dashboard values stay plain text (no `HtmlFormatter`) so the graph view can plot them.
- On `stop()` or a throwing hook, every Module whose `initStates()` began gets `stop()`, half-initialized ones included; `onEnd()` runs only on a normal stop.
- A crash shows on the dashboard's Error line until the next INIT. The SDK only logs it, so a bare OpMode's crash shows only in logcat and the dashboard's Error view, and an `Error` (e.g. `StackOverflowError`) thrown from a bare OpMode kills the RC app.
- Keep dashboard telemetry fast (`OptimizationToggles.dashboardTransmissionIntervalMs`) and the loop profile on: the team tunes on the dashboard.

## Module pattern

- Don't touch states in a Module's constructor: they bind in `initStates()`, and `activate()` on an unbound state just returns false.
- Name nested `@Config` classes (`@Config("Shooter")`): slothboard keys config classes by simple name, so bare nested `Tuning` classes collide and vanish.
- Telemetry goes in `onTelemetry()`, not `read()`. DS lines outside a DS frame are dropped; gate other DS-only work on `getTelemetry().isDSFrame()`.
- Safe state lives in `stop()`, never only in a command's `end()` (those don't run at OpMode stop). Use `EnhancedMotor.stop()`, since the write cache can drop `setPower(0)`, and keep it reversible: it can run mid-OpMode with `write()` resuming.
- `EnhancedMotor` and `EnhancedServo` periodically re-send an unchanged value because the hub can drop a write silently (a hub reset or NACK); keep that in any new write cache.
- `stop()` can only de-energize: on a normal STOP the SDK has already fail-safed the hubs and silently drops any move, so stow a mechanism at the end of its routine.
- `EnhancedMotor.setVelocity(double)` is ticks/s, not RPM; `withVoltageCompensation` scales only `setPower`.
- A State setpoint is dashboard-live only through `bindTunable(state, () -> field)` after `setStates`; a value passed to the enum constructor is read once.

## Commands

- Never call `Scheduler.execute()` or `Scheduler.reset()` in an `EnhancedOpMode`; the framework does (a bare `LinearOpMode` does both itself).
- Build commands in `initialize()` (`StateCommands.set` throws on a state not yet bound) and schedule them in `onStart()` (`start()` resets the scheduler).
- Requirements are `Module`s, or the `Follower` for every `PathCommands` command but `waitMs`; a group requires all its children's. A new command interrupts a running one of equal priority it collides with, so a `StateCommands.set` from `gameLoop()` on any module an auto touches kills the whole auto. Build reactions into the sequence, or give the auto `.setPriority(1)` and the reaction `.setBlockedBehavior(BlockedBehavior.QUEUE)`.
- `PathCommands.remainingBelow` measures the current leg only.
- Ivy's `Commands.waitMs` runs on the wall clock, which jumps decades the first time a DS or the RC web page connects after boot: use `PathCommands.waitMs`.
- Race every auto against `PathCommands.waitMs` (`PathCommands.timeout` does it; build the `race` by hand to know whether it fired). `opmodes/test/MockAuto` is the reference auto; `opmodes/test/auto/CloseFlowerAuto` is the same on real paths.

## Tuning

Dashboard edits are lost on deploySloth or an app restart: copy kept values into the source.

- **Pedro** (`BettaConstants`, tuned with AutoTune): Foresight's P and FF gains apply live, the rest at the next INIT. Keep the robot still for the first second of INIT while the Pinpoint recalibrates its IMU.
- **Ball colours and gates** are the constants atop the SnapScript (`docs/hsv-tuning-prompt.md` derives starting HSV ranges from photos). Tune them in the Limelight web editor with `LimelightBalls → syncOnInit` off, or the next INIT overwrites the edits, then copy them into the file. Pipeline settings (exposure, gain, resolution) live in `limelight/ball_pipeline.vpr`: save the web UI's pipeline download over it, or the next synced INIT reverts them.
- **Homography** (`eocvsim/homography`): lay the board flat and square to the robot and check the locked 6 in grid with a tape measure. A new one moves the (0,0) crosshair, so redo `Mount` in the same change.
- **`BallFieldTransform.Mount`** (committed for the Betta bot's Limelight): `xIn`/`yIn` run from the Pinpoint offsets' origin to the (0,0) crosshair. Check it with Camera Module Test, robot still, a ball on the centreline at two distances: a constant X or Y error is `xIn`/`yIn`, a Y error growing with distance is `headingDeg`, a left ball reading negative Y means flip `mirrorY`, a growing X error means redo the homography. Turning in place, a still ball tracing a circle means `xIn`/`yIn` is off.

**AutoTune**: `http://192.168.43.1:10158` on the robot wifi (websocket 12649; over USB, forward both to `localhost`). Run Mecanum, Pinpoint, Foresight, then Tests, copying each result into the matching `BettaConstants.*Settings` field (the printed code block is Quickstart-shaped: don't paste it); re-run after any drivetrain, wheel or pod change. deploySloth doesn't reach a running procedure: close it first. Tests drives within a Distance × Distance area: clear it. A `@Tuner` method in `pedro/Tuning.java` must be static, take no arguments and return exactly `Procedure`, or the RC app breaks at boot.

## Testing

Hardware configs ship in `res/xml/`; `limelight` is an EthernetDevice.

- **Mock Architecture Test** is the first run after a full install. It drives only with `Mock Auto → enableDrive` on (check motor and odometry directions first), and then finishes only on the floor.
- **Close Flower** autos: the `toAudienceWall` and `toHive` legs put part of the robot over the centre line (G402).
- **Decode Tele** starts at the placeholder pose: reset it with a gamepad1 pose button before trusting the odometry-only turret aim.
- **OctoQuad** (I2C name `octoquad`, type `OctoQuadFTC`): the SDK's built-in driver handles MK2 firmware 3.1; don't add DigitalChickenLabs' `OctoQuadFWv3` driver. Calibrate with the Deadwheel and Heading Scalar calibrators (`octoquad/`), then enter the results on the dashboard in OctoQuad Localizer Test (`opmodes/test/`; gamepad1 A applies them). Localizer parameters apply only through `resetLocalizerAndCalibrateIMU()`; after each reset, don't drive or use the pose until `getLocalizerStatus()` reads `RUNNING` (throw on `FAULT_NO_IMU`), and drop data whose `crcOk` is false. Pedro's `OctoQuadConfig` takes Pinpoint-style offsets (`xPodOffset`: the forward pod's sideways offset; `yPodOffset`: the strafe pod's forward offset) and sends `-yPodOffset` as the SDK's `tcpOffsetMM_X` and `-xPodOffset` as `tcpOffsetMM_Y`.
- **Pinpoint Localizer Test** (`opmodes/test/`) keeps its own Pinpoint and drive settings, independent of `BettaConstants`: copy tuned values into `BettaConstants.pinpoint`, which is in inches.
- **Laser rangefinders** (Brushland, Analog Inputs `analog0`/`analog1`): switch a sensor to analog with Configure Laser Rangefinder on `laser_rangefinder_i2c.xml`, and set every reader's `maxRangeMm` to its `analogHighMm` or distances scale wrong. Back to I2C: INIT and stop that OpMode with the sensor unplugged, then plug it in (LED turns magenta); trust `Answering on I2C`, not `Pin0`/`Pin1` (a silent sensor reads 0).
- **Transfer Count Test** / **Transfer Thread Count Test**: `analog0` (A) sits upstream of `analog1` (B), close enough that one ball covers both; balls must only travel A to B and start with the chute clear. Tune `diffThreshold` on the dashboard graph of `A - B (mm)`.
- **Hub-reading threads.** `modules/TransferCounter` and Bench: Analog Poll's poller are the only code reading the hub from a thread. The hub's ADC refreshes about every 10 ms and a 10 ms poller already sees almost every refresh, so poll at 10 ms: 5 ms sees no more and more than doubles what the poller costs a driving loop (`docs/hub-bench-findings.md`). Any thread that reads the hub:
  - Call `start()` on the OpMode thread and `stop()` from the OpMode's stop, not `Module.stop()`, which also runs mid-OpMode.
  - Loop only while the OpMode thread `isAlive()`: after a crash no OpMode hook runs (EnhancedOpMode skips `onEnd()`), so that is the only stop signal.
  - Stop with a volatile flag and `join`, never `interrupt()`: the SDK's `MessageKeyedLock.release` calls `lockInterruptibly()`, so an interrupt mid-command can leave the hub's USB lock held until it's abandoned after 500 ms, stalling every thread's hub commands.
  - Catch `Throwable` into a volatile field and rethrow it on the OpMode thread.
  - Read with its own `LynxGetBulkInputDataCommand` (analog in mV), never through `AnalogInput` or `clearBulkCache()`: the MANUAL bulk cache is shared across threads.
  - No writes or telemetry from the thread.
- **Shooter Test**: Through Bore velocity comes from position over `rpmWindowMs`, because the hub's 16-bit velocity wraps above about 240 RPM at 8192 counts/rev; any flywheel code reading one must do the same. Check RPM reads positive spinning forward (`reverseEncoder`) before setting `targetRpm`: reversed, the closed loop goes to full power.
- Single devices: the dashboard's Hardware View. An isolated module test extends `OpMode`; one that needs the follower uses `EnhancedOpMode` with a minimal `Robot` (Camera Module Test).

**Control Hub bench** (`opmodes/test/bench/`, `Bench: *`): usage in `scripts/bench/README.md`; results and what to design around in `docs/hub-bench-findings.md`. Read it before changing the planner, hub or I2C traffic, odometry, Limelight or stop paths, and update it after a rerun.

## Conventions

- Don't change `JavaVersion.VERSION_1_8`, `targetSdkVersion 28` (both SDK-pinned), `compileSdk`, `minSdkVersion` or `ndkVersion` without a reason. Every DS app must match the SDK version (a mismatch fails inspection); an SDK bump also bumps `versionCode`/`versionName`.
- Comments: none by default, in any language; only a one-line WHY a competent reader would otherwise get wrong. Usage notes go in this file.
- Never hand-edit vendored `robotcontroller/internal/*`, `architecture/prism/*` or `pedro/procedures/*`. Refresh the last two with their `scripts/update-*.sh`, one commit each, plus any Pedro bump the Quickstart made in `TeamCode/build.gradle`.
- Fail fast: no catch to keep a loop alive, no silent fallback, no optional devices (`hardwareMap.get`, never `tryGet`); numeric guards and try/finally are fine. The only exceptions: `WebcamControls` ignores unsupported UVC controls, Limelight problems raise the `Limelight` fault instead of throwing while Limelight-dependent actions fall back, and the vendored `pedro/procedures/*` tuners use `tryGet` upstream.
- `Limelight3A`:
  - Its POSTs (`pipelineSwitch`, `upload*`, `update*`, `reloadPipeline`) block up to 15 s and fail as a bare `false`. Call them only from one background thread (newest-wins one-slot mailbox, on change, rate-capped) and switch the pipeline once per OpMode, apart from `LimelightSync`'s park-and-return around an upload (the Limelight loads a script only on switching to its pipeline). `LimelightSync` is that sender.
  - `LLResult.getPipelineType()` is always empty (use `getStatus()`); llpython keeps the last script's output on every pipeline.
  - Poll errors are swallowed: health is `isConnected()`, `getTimeSinceLastUpdate()` and the result's contents.
  - `setPollRateHz` (clamped to 1–250 Hz) is ignored while polling, so `stop()` first; the rate persists across OpModes.
- OpMode `group` is exactly `"Test"` for test OpModes and `"A"` for the competition teleop.
- Commits: one imperative subject line, no body, no trailer. Ask before pushing.
