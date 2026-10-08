# CLAUDE.md

## Project context

FTC 6165 MSET Cuttlefish, 2026–27 BIOBUZZ season.

- Game-specific code goes in `biobuzz/`, `modules/`, `opmodes/`; `architecture/` stays game-agnostic. Field constants come from the game manual, never guesses. New Java files go in an existing package.
- The top level holds only Gradle and repo files, `TeamCode/`, `limelight/` (runs on the Limelight), `scripts/` and `docs/`.
- No BioBuzz robot yet: everything runs on the Betta bot (a sister team's DECODE bot, `pedro/BettaConstants`).
- `decode/` is last season's code for 6165's Cuttle bot only (`cuttle_decode.xml`). It shares `modules/Drivetrain`.
- Automated ball detection and pathing are Autonomous-only and stay on our alliance's half.
- When an auto's poses change, re-export its `docs/paths/<OpMode>.pp` (visualizer.pedropathing.com).

## Keep this file fresh

Claude updates this file unasked, in the same commit as any change it describes. Only current rules, locations and footguns the code doesn't show; no history, no value lists.

## Build & deploy

IDE run configs are per laptop (`.idea/` isn't committed); set them up per Sloth's README.

- **TeamCode** (Run button, full install): first deploy, after a hub wipe or Sloth bump, and after any change outside `org.firstinspires.ftc.teamcode` (dependencies, manifest, resources, `robotcontroller/internal/`).
- **deploySloth** (hot reload): everything else. It returns when the push ends, not the load; wait for the load (logcat's `Processed Sloth Load`, the RC screen or the DS) before redeploying or pressing INIT. An `EnhancedOpMode`'s `Code` status line names the Sloth jar it runs, or `installed APK`.
- An old hot-load overriding new code: run the `removeSlothRemote` task alone, after `adb connect 192.168.43.1` (it never connects and silently does nothing without a device).
- With no adb device connected, deploySloth connects to `192.168.43.1` and runs a bare `adb disconnect` when done; connect first to stay connected. Other address: `load { address = "..." }` in `TeamCode/build.gradle`.
- The DS app must match the SDK version or it fails inspection.
- No unit tests. Verify with `./gradlew :TeamCode:compileDebugJavaWithJavac` (`:TeamCode:assembleDebug` for the APK).

## Dependencies

- Sloth, the `dev.frozenmilk.sinister.sloth.load` plugin and slothboard's `{sloth}` version prefix must match; slothboard pins Sloth strictly.
- slothboard is the committed `TeamCode/libs/m2/`, built from the same-named tag on `6165-MSET-Cuttlefish/slothboard`. To bump: tag a new build there, replace the six files and the version string. Keep `exclude group: 'com.acmerobotics.dashboard'` (same Java packages).
- Close the dashboard tab when timing loops: it reads every hub's voltage each second.

Pedro Pathing 3:

- `Pose.heading()` is in [0, 2π).
- Every `Paths` path needs a heading interpolator; a missing one compiles and throws when the leg starts.
- `BezierCurve.length()` (so `parameter()`, `remainingDistance()`) can under-report S-curves and loops down to the chord: never use it for timeouts or scoring. `PathCommands.followUntilRemaining` hands over early on such curves.
- `follow`/`followThrough` brake at the end; legs keep speed through a join only as segments of one `Paths.path(...)` or via a `followUntilRemaining` handover before braking.
- A heading jump over 11.25° at a join gives turning priority over driving; `architecture/auto/PathHeadings` gives headings that avoid it.
- `Interpolator.piecewise()` is broken in 3.0.1 (a `tangent` or `facingPoint` piece replays the whole curve). Use `PathHeadings.linearThenTangent`/`holdThenTangent`/`tangentThenLinear`, or only constant/linear pieces on one curve.

## Framework layout

`architecture/` is the portable framework; `auto/` holds field geometry, alliance mapping and the ball planner, and `command/` the Ivy factories (`StateCommands`, `PathCommands`). A `Robot` subclass's `createFollower(HardwareMap)` and `initializeGameModules()` run inside `Robot`'s constructor, before the subclass's field initializers.

**Poses and alliance**

- Author poses for RED with `FieldPose.forAlliance(x, y, heading)`; it maps them to BLUE through `FieldConfig.symmetry()` (BIOBUZZ: a 180° rotation, so a BLUE heading is RED's + π). A bare radian passed to a path isn't mapped.
- `FieldPose` throws until the symmetry is set, and `EnhancedOpMode` clears it and the dashboard's field image at every INIT: a BIOBUZZ `Robot` constructor calls `BioBuzzField.configure()` (symmetry and image), a bare SDK OpMode calls `FieldConfig.setSymmetry(...)` itself. The image is a render in Pedro's frame, drawn at its real width (141.5 in), so a new one must be too.
- `Robot` resets the follower to a placeholder pose at every INIT: set the start with `robot.follower.setPose(...)` in `initialize()`, then build paths.
- `Context.allianceColor` is set only on the dashboard or in `Context.java`, never by an OpMode, and resets on an app restart or Sloth reload.

**Ball routing**

- `RouteOptimizer.findOptimalRoute` → `RoutePathBuilder.build` → `RouteRun`, the spline method of summer-2026's `Spline/Field`: the intake leg is Pedro's spline (`BezierCurve.through`: one Bezier through each point at evenly spaced t) with the robot's centre through each ball the intake doesn't sweep up in passing (`IntakeCurvePlanner`), the return a straight line, and a leg that doesn't fit falls back to that spline, then straight hops, through `VisibilityGraphPlanner` detours. Every leg keeps the whole `RobotShape` footprint (frame plus intake) inside the area; `biobuzz/RobotGeometry` is the Betta bot's.
- No alliance mapping: pass mapped field coordinates, `BioBuzzField.ownHalf(margin)` as the area and, on the field, `BioBuzzField.hiveRails()` as the only obstacles. Bad input throws, so check a robot-sourced start with `RouteOptimizer.poseProblem` first.
- Pedro switches to the next piece of a compound path as soon as it would start braking (`ForesightConfig.pathSkip`, on by default), so it starts a sharp turn or reversal early. Each drive in `Plan.steps` is one curve or hop, with a turn in place before it wherever the heading jumps, and `RouteRun` stops after each.
- A curve that stops dead mid-path (a straight cubic with handles the full chord long) makes Pedro's follower throw there.
- A `RouteRun` runs once, via `start()` or its `command()` in a group, not both; call `abort(reason)` from OpMode code, never inside a command.

**Faults** (`core/Faults`) are named, recoverable errors, for the Limelight only (see Fail fast). Keys are global and cleared at every INIT. `EnhancedOpMode` shows them; a bare `OpMode`/`LinearOpMode` must show them itself.

**Game code.** Game commands go in `RobotActions` (`robot.actions`). `Camera` starts ENABLED, polling the Limelight and able to raise its fault, so an OpMode that doesn't use ball vision activates `Camera.VisionState.DISABLED` in `initialize()`.

**Drivetrain** needs `withFollower(follower)`, or field-centric drive and heading lock throw and `write()` stops guarding against Pedro. With it, `write()` throws while the follower isn't idle (both drive the same motors): an OpMode that follows paths calls `robot.drivetrain.setWriteEnabled(false)`; `BallCollection` does this itself.

**Ball collection** (`RobotActions.collectBalls`) plans from the balls in view when it starts, waiting only `visionWaitMs` for a fresh frame, so a START during the INIT Limelight sync can end it SKIPPED. `onField` off tests away from a field (start pose (0, 0, 0), no walls or HIVE rails); `BallCollection.keepIn`/`obstacles`/`configuredPose` give other OpModes the same switch.

**Vision** (`modules/vision/`). `TrackedBall` is camera-relative, so a still ball moves when the robot does; `FieldBall` is field-relative. While `Camera.isFrameStale()`, field balls stay at their last position with `visible()` false until `forgetAfterSeconds`: check `visible()` before chasing one.

**Ball detection** is `limelight/ball_contour_snapscript.py` plus `ball_pipeline.vpr`, baked into the generated `LimelightFiles` (so deploySloth carries edits) with the uploaded `SCRIPT_ID` replaced by a stamp. While `syncOnInit` is on, `LimelightSync` uploads both at every INIT and waits for the stamp; the Limelight loads a script only on switching to its pipeline. SnapScripts get the raw image: paste `eocvsim/homography`'s whole Python block into the script, and after a lens recalibration update `LENS_*` in both the script and `HomographyCalculationPipeline`.

**Limelight faults.** A Limelight problem raises the `Limelight` fault and stales the frame until it recovers (`Camera.limelightProblem()` says why); a web-editor copy of the script faults unless `syncOnInit` is off. The sync isn't retried: a failed sync, or the Limelight landing on another pipeline (reboot, web-UI switch), faults until the next INIT.

**Cell tip** (`CellTipCamera`). An empty frame reads tipped, so a camera pointed away looks like a tipped cell. Mount the webcam image-upright, facing the launch direction (an upright tag shows roll ≈ 0 on the stream overlay). Tag ids come from `Context.allianceColor` at module init, so set the alliance before INIT. Never call `CellTipCamera.stop()` yourself: it's irreversible.

## OpMode lifecycle

`init()` runs `createRobot()`, each module's `initStates()` then `init()`, `initialize()`, then a second module pass for Modules created in `initialize()`. Modules are found through fields reachable from the OpMode and `robot`, never locals.

Each loop reads modules and updates the follower before `gameLoop()`, then runs Ivy commands, then writes. In INIT, `initializeLoop()` replaces `gameLoop()` and writes stay off unless `shouldWriteDuringInit()`, but the follower and scheduler still run: a follow, hold or path command before `onStart()` drives the robot in INIT.

- Telemetry lines go in `telemetry()`, field drawing in `dashboardOverlay(Canvas)`. Never call `telemetry.update()` in an `EnhancedOpMode`: it closes the DS frame and later DS lines are dropped. Dashboard values stay plain text (no `HtmlFormatter`) so the graph view can plot them.
- On `stop()` or a throwing hook, the framework resets the scheduler, stops the follower and its motors, and calls `stop()` on every Module whose `initStates()` began, half-initialized ones included. `onEnd()` runs only on a normal stop.
- A crash shows on the dashboard's Error line until the next INIT. The SDK only logs it, so a bare OpMode's crash shows only in logcat and the dashboard's Error view, and an `Error` (e.g. `StackOverflowError`) thrown from a bare OpMode kills the RC app.
- Keep dashboard telemetry fast (`OptimizationToggles.dashboardTransmissionIntervalMs`) and the loop profile on: the team tunes on the dashboard.

## Module pattern

- Don't touch states in a Module's constructor: they bind in `initStates()`, and `activate()` on an unbound state silently returns false.
- Name nested `@Config` classes (`@Config("Shooter")`): slothboard keys config classes by simple name, so bare nested `Tuning` classes collide and vanish.
- Telemetry goes in `onTelemetry()`, not `read()`. DS lines outside a DS frame are dropped; gate other DS-only work on `getTelemetry().isDSFrame()`.
- Safe state lives in `stop()`, never only in a command's `end()` (those don't run at OpMode stop). Use `EnhancedMotor.stop()`, since the write cache can drop `setPower(0)`, and keep it reversible: it can run mid-OpMode with `write()` resuming.
- `EnhancedMotor.setVelocity(double)` is ticks/s, not RPM; `withVoltageCompensation` scales only `setPower`.
- A State setpoint is dashboard-live only through `bindTunable(state, () -> field)` after `setStates`; a value passed to the enum constructor is read once.

## Commands

- Never call `Scheduler.execute()` or `Scheduler.reset()` in an `EnhancedOpMode`; the framework does (a bare `LinearOpMode` does both itself).
- Build commands in `initialize()` (`StateCommands.set` throws on a state not yet bound) and schedule them in `onStart()` (`start()` resets the scheduler).
- Requirements are `Module`s, or the `Follower` for every `PathCommands` command; a group requires all its children's. A new command interrupts a running one of equal priority it collides with, so a `StateCommands.set` from `gameLoop()` on any module an auto touches kills the whole auto. Build reactions into the sequence, or give the auto `.setPriority(1)` and the reaction `.setBlockedBehavior(BlockedBehavior.QUEUE)`.
- `PathCommands.remainingBelow` measures the current leg only.
- Ivy's `Commands.waitMs` runs on the wall clock, which jumps decades the first time a DS or the RC web page connects after boot: use `PathCommands.waitMs`.
- Wrap every auto in `PathCommands.timeout`. `opmodes/test/MockAuto` is the reference auto; `opmodes/test/auto/CloseFlowerAuto` is the same on real paths.

## Tuning

Dashboard edits are lost on deploySloth or an app restart: copy kept values into the source.

- **Pedro** (`BettaConstants`, tuned with AutoTune): Foresight's P and FF gains apply live, the rest at the next INIT. Keep the robot still for the first second of INIT while the Pinpoint recalibrates its IMU.
- **Ball colours and gates** are the constants atop the SnapScript. Tune them in the Limelight web editor with `LimelightBalls → syncOnInit` off, or the next INIT overwrites the edits, then copy them into the file. Pipeline settings (exposure, gain, resolution) live in `limelight/ball_pipeline.vpr`: save the web UI's pipeline download over it, or the next synced INIT reverts them.
- **Homography** (`eocvsim/homography`): print `docs/calibration-chessboard-3in-letter.pdf`, set `SQUARE_SIZE_INCHES` to the measured square, lay it flat and square to the robot with the long side away, and check the locked 6 in grid with a tape measure. A new homography moves the (0,0) crosshair, so redo `Mount` in the same change.
- **`BallFieldTransform.Mount`** (committed for the Betta bot's Limelight): `xIn`/`yIn` is the offset from the Pinpoint offsets' origin to the overlay's (0,0) crosshair. To check it, run Camera Module Test with the robot still and a ball on the centreline at two distances: a constant Y or X error is `yIn` or `xIn`, a Y error growing with distance is `headingDeg`, a left ball reading negative Y means flip `mirrorY`, and a growing X error means redo the homography. Turning in place, a still ball that traces a circle means `xIn`/`yIn` is off.

**AutoTune**: join the robot wifi and open `http://192.168.43.1:10158` (websocket 12649; over USB, forward both to `localhost`). Run Mecanum, Pinpoint, Foresight, then Tests, pasting each block into `BettaConstants`' matching `*Settings`; re-run after any drivetrain, wheel or pod change. Tests drives 48 in whatever its Distance field says, so clear a 48 × 48 in area. A `@Tuner` method in `pedro/Tuning.java` must be static, take no arguments and declare exactly `Procedure` as its return type, or the RC app breaks at boot.

## Testing

Device names: `fl` `bl` `fr` `br` `pinpoint` (every follower), `floodgate` (Analog Input, `Drivetrain`), webcam `aprilTagDetector` (cell tip), `limelight` (EthernetDevice). `res/xml/` ships `cuttle_decode.xml`, `betta_decode.xml` and webcam-only `camera.xml`. A hub set up from older code names the webcam `ballDetector` or `nerdDetector`: re-activate a shipped config or rename it.

- **Mock Architecture Test** is the first run after a full install. It drives only with `Mock Auto → enableDrive` on (check motor and odometry directions first), and then finishes only on the floor.
- **Close Flower** autos: the `toAudienceWall` and `toHive` legs put part of the robot over the centre line (G402).
- **Ball Collection Auto** reads its start pose and `collection` settings at INIT; wait for its `Vision` line to read live before START, or it skips while the INIT sync is still uploading.
- **Decode Tele** starts at a placeholder pose: reset it with gamepad1 before trusting the odometry-only turret aim.
- Single devices: the dashboard's Hardware View. An isolated module test extends `OpMode`, not `EnhancedOpMode`.

**Known issues**, unmeasured on a Control Hub: `WebcamSession.close()` runs synchronously in `stop()` and may overrun the SDK's stop watchdog; the route planner's cost on the hub and its re-plan hysteresis.

## Conventions

- Don't change `JavaVersion.VERSION_1_8`, `targetSdkVersion 28` (both SDK-pinned), `compileSdk`, `minSdkVersion` or `ndkVersion` without a reason; `versionCode`/`versionName` follow the SDK version.
- Comments: none by default, in any language; only a one-line WHY a competent reader would otherwise get wrong. Usage notes go in this file.
- Never hand-edit vendored `robotcontroller/internal/*`, `architecture/prism/*` or `pedro/procedures/*`. Refresh the last two with their `scripts/update-*.sh`, one commit each, plus any Pedro bump the Quickstart made in `TeamCode/build.gradle`.
- Fail fast: no catch to keep a loop alive, no silent fallback, no optional devices (`hardwareMap.get`, never `tryGet`); numeric guards and try/finally are fine. The only exceptions: `WebcamControls` ignores unsupported UVC controls, and Limelight problems raise the `Limelight` fault instead of throwing while Limelight-dependent actions fall back.
- `Limelight3A`:
  - Its POSTs (`pipelineSwitch`, `upload*`, `update*`, `reloadPipeline`) block up to 15 s and fail as a bare `false`. Call them only from one background thread (newest-wins one-slot mailbox, on change, rate-capped) and switch the pipeline once per OpMode, apart from `LimelightSync`'s park-and-return around an upload (the Limelight loads a script only on switching to its pipeline). `LimelightSync` is that sender.
  - `LLResult.getPipelineType()` is always empty (use `getStatus()`); llpython keeps the last script's output on every pipeline.
  - Poll errors are swallowed: health is `isConnected()`, `getTimeSinceLastUpdate()` and the result's contents.
  - `setPollRateHz` (clamped to 1–250 Hz) is ignored while polling, so `stop()` first; the rate persists across OpModes.
- OpMode `group` is exactly `"Test"` for test OpModes and `"A"` for the competition teleop.
- Commits: one imperative subject line, no body, no trailer. Ask before pushing.
