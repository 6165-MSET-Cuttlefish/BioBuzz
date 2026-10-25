# CLAUDE.md

## Project context

- Game code goes in `biobuzz/`, `modules/`, `opmodes/`; `architecture/` stays game-agnostic; new Java files go in an existing package. Field constants come from the BIOBUZZ game manual, never guesses.
- Nothing new at the top level beyond Gradle and repo files: tools go in `scripts/`, documents in `docs/`.
- No BioBuzz robot yet: BioBuzz code runs on the Betta bot (`betta_decode.xml`, `pedro/BettaConstants`, `biobuzz/RobotGeometry`).
- `decode/` runs only on the Cuttle bot (`cuttle_decode.xml`) and shares `modules/Drivetrain`. Run nothing built on `BettaConstants` there: Cuttle's config has every device a BioBuzz OpMode uses, so one starts with Betta's constants and no error.
- In competition OpModes, automated ball detection and pathing are Autonomous-only and stay on our alliance's half.
- When an auto's poses change, re-export its `docs/paths/<OpMode>.pp` (visualizer.pedropathing.com).

## Keep this file fresh

Claude updates this file unasked, in the same commit as any change it describes. Only current rules, locations and footguns the code doesn't show; no history, no value lists.

## Build & deploy

- **TeamCode** (Run button, full install): on a fresh hub and after any change outside `org.firstinspires.ftc.teamcode` (dependencies, manifest, `res/`, `assets/`, `robotcontroller/`).
- **deploySloth** (hot reload): everything else. It returns before the load lands: wait for the `Processed Sloth Load` toast (DS, RC screen or logcat; about 5 s) before redeploying or pressing INIT, or the app restarts or the INIT never runs.
- The hub's clock reads about 1970 after boot until a DS connects or a browser opens `http://192.168.43.1:8080`. A load pushed while it reads earlier than the APK's install time is deleted at the next app restart (logcat `removing outdated sloth load`) and the installed APK's code runs: connect the DS before deploySloth.
- Verify with `./gradlew :TeamCode:compileDebugJavaWithJavac` (`:TeamCode:assembleDebug` for the APK); after an AGP or Gradle bump, also `:TeamCode:assembleSloth`, which hooks AGP's dex task by name.

## Dependencies

- Bump Sloth and the `dev.frozenmilk.sinister.sloth.load` plugin (root `build.gradle`) together.
- slothboard (`TeamCode/libs/m2/`) is built from the same-named tag on `6165-MSET-Cuttlefish/slothboard`: bump by tagging a new build there.
- Close the dashboard tab when timing loops: it reads every hub's voltage each second.
- A loop spike every ~500 ms while motors are written is the SDK re-reading each motor's mode and re-sending its enable, not our code.

Pedro Pathing 3:

- `Pose.heading()` is in [0, 2π).
- `BezierCurve.length()` can under-report an S-curve down to its chord, and `parameter()` and `remainingDistance()` inherit it: never use them for timeouts or route costs. `PathCommands.followUntilRemaining` and `remainingBelow` fire early on such curves.
- `follow`/`followThrough` brake at the end; legs keep speed through a join only as segments of one `Paths.path(...)` or via a `followUntilRemaining` handover before braking.
- A heading jump over 11.25° at a join gives turning priority over driving; `architecture/auto/PathHeadings` gives headings that avoid it.
- A lambda passed to `path.heading(...)` gets path completion (per piece inside `piecewise`), not the curve's `t`: wrap it in `Interpolator.geometric(...)` for `t`. `reverse()` drops `geometric`, so `reverseTangent()` and `facingPoint(...).reverse()` are wrong on curves: use `PathHeadings` (`backwardTangent` to drive one backwards).

## Framework layout

`initializeGameModules()` runs inside `Robot`'s constructor, before the subclass's field initializers and constructor body: it sees subclass fields unset, and an initializer on a field it assigns (even `= null`) overwrites it.

**Poses and alliance**

- Author poses for RED with `FieldPose.forAlliance(x, y, heading)` and take path headings from those poses: a raw radian passed to a path (e.g. `linear(...)`) isn't mapped to BLUE.
- `Robot` puts the follower at a placeholder pose every INIT: call `robot.follower.setPose(start)` in `initialize()` before building paths. Odometry runs from INIT, so put the robot on its start before INIT and don't move it after.
- Set `Context.allianceColor` on the dashboard before INIT (modules and paths read it then), never from an OpMode.

**Ball routing**

- The planner doesn't map alliances: pass mapped coordinates, `BioBuzzField.ownHalf(margin)` as the area and, on the field, `BioBuzzField.hiveRails()` as the only obstacles.
- Drive a plan only with `RouteRun`: `Plan.intake`/`Plan.back` are for drawing. They join the drives without the plan's turns in place, and Pedro moves to a compound path's next piece as soon as it would start braking (`ForesightConfig.pathSkip`, on by default), cutting reversals.

**Faults** (`core/Faults`) show only in an `EnhancedOpMode`; a bare `OpMode` shows the problem itself (`BallFieldDriveTest`).

**Game code.** Game commands go in `RobotActions` (`robot.actions`). `Camera` starts ENABLED, polling the Limelight and able to raise its fault, so an OpMode that doesn't use ball vision calls `disableBallVision()` in `initialize()`.

**Ball collection** (`RobotActions.collectBalls`) plans once and drives back to where it planned from. It ends SKIPPED on a Limelight fault before it plans, or on a START before the INIT Limelight sync is done (about 2 s; it waits only `visionWaitMs`).

**Vision** (`modules/vision/`): `Camera`'s field-ball getters keep a ball out of view for `forgetAfterSeconds`, frozen where last seen with `visible()` false: check `visible()` before chasing one.

**Ball detection** (`limelight/`) ships as the generated `LimelightFiles`, so deploySloth carries edits. After a lens recalibration, update `LENS_*` in both the SnapScript and `HomographyCalculationPipeline`.

**Limelight faults.** A failed sync isn't retried and nothing switches the Limelight back mid-OpMode: after a failed INIT sync, or with the Limelight left on another pipeline (e.g. a web-UI switch), re-INIT.

**Cell tip** (`CellTipCamera`): mount the webcam image-upright, facing the launch direction (an upright tag shows roll ≈ 0 on the stream overlay). Turn it off with `VisionState.DISABLED`, never `stop()`: that closes the webcam for good.

## OpMode lifecycle

A Module is read, written and stopped only if a field reachable from the OpMode or `robot` holds it by the end of `initialize()`; a Module's own fields aren't searched.

In INIT the follower and Ivy scheduler run but module writes don't (unless `shouldWriteDuringInit()`). Schedule commands from `onStart()` on: START drops earlier ones without `end()`, and a path command scheduled earlier drives the robot in INIT and keeps driving after START.

- Never call `telemetry.update()` in an `EnhancedOpMode`: it closes the DS frame and later DS lines are dropped.
- On `stop()` or a throwing hook, every Module whose `initStates()` began gets `stop()`, half-initialized ones included; `onEnd()` runs only on a normal stop.
- Keep `OptimizationToggles.dashboardTransmissionIntervalMs` low and `loopProfile` on: the team tunes on the dashboard.

## Module pattern

- Name every `@Config` in spaced words, a test OpMode's exactly as the OpMode: slothboard keys groups by that name (a bare `@Config` by its class's simple name), and a duplicate hides one group.
- Safe state lives in `stop()`, never only in a command's `end()`, which doesn't run at OpMode stop; stop motors with `EnhancedMotor.stop()`, since the write cache can drop `setPower(0)`.
- Keep `WriteCache`'s periodic re-send of an unchanged value, and add one to any new write cache: a hub reset or NACK drops a write silently.
- `stop()` can only de-energize: on a normal STOP the SDK has already fail-safed the hubs and silently drops any move, so stow a mechanism at the end of its routine.
- A State setpoint follows the dashboard only through `bindTunable(state, () -> Tuning.x)`; `Tuning.x` passed to the enum constructor is read once, and `setStates()` resets every variant, undoing a `setValue` made in a Module's constructor.

## Commands

- Never call `Scheduler.execute()` or `Scheduler.reset()` in an `EnhancedOpMode`, which does both; a bare OpMode must do both itself, `reset()` at INIT.
- Build commands in `initialize()` or later: `StateCommands.set` takes each state's module when built, which in an OpMode field initializer may still be the previous OpMode's, so the command conflicts with nothing.
- A group holds all its children's requirements for its whole run, and a new command interrupts any colliding one of equal or lower priority: a `StateCommands.set` from `gameLoop()` on any module an auto touches kills the whole auto. Build reactions into the sequence, or give the auto `.setPriority(1)` and the reaction `.setBlockedBehavior(BlockedBehavior.QUEUE)`, which runs it after the auto.
- `PathCommands.remainingBelow` measures the current leg only, and reads the previous path's distance until the follower has updated on the new one, so with a follow scheduled from `gameLoop()` it can fire at once.
- Ivy's `Commands.waitMs` runs on the wall clock, which jumps decades the first time a DS or the RC web page connects after boot: use `PathCommands.waitMs`.
- Race every auto against `PathCommands.waitMs`: `PathCommands.timeout`, or a hand-built `race` to know whether it fired. Reference autos: `opmodes/test/MockAuto`, and `opmodes/test/auto/CloseFlowerAuto` on real paths.

## Tuning

Dashboard edits are lost on deploySloth or an app restart: copy kept values into the source.

- **Pedro** (`BettaConstants`): building the follower recalibrates the Pinpoint's IMU, so keep the robot still for the first second of INIT, even if you'll reset the pose later. The pod offsets are the OctoQuad's: don't overwrite them with AutoTune's Pinpoint results. Pod directions are the Pinpoint's own, not the OctoQuad's: before any Pedro-driven run, push the robot forward, then left, in Pinpoint Localizer Test and check x, then y, rises.
- **Ball detection**: `docs/hsv-tuning-prompt.md` derives starting HSV ranges from photos. Exposure, gain and resolution live in `limelight/ball_pipeline.vpr`: save the web UI's pipeline download over it, or the next synced INIT reverts them.
- **Homography** (`eocvsim/homography`): a new one moves the (0,0) crosshair, so redo `Mount` in the same change.
- **Camera Mount** (`BallFieldTransform.Mount`, measured from the Pinpoint offsets' origin): check it in Camera Module Test, robot still, a ball on the centreline at two distances: constant X or Y error is `xIn`/`yIn`, Y error growing with distance `headingDeg`, a left ball reading negative Y `mirrorY`, X error growing with distance the homography. Then, turning in place, a still ball whose field position moves means `xIn`/`yIn` is off.

**AutoTune**: `http://192.168.43.1:10158` on the robot wifi (websocket 12649; over USB, forward both to `localhost`). Run Mecanum, Pinpoint, Foresight, then Tests, copying each result into the matching `BettaConstants.*Settings` field (the printed code block is Quickstart-shaped: don't paste it). deploySloth doesn't reach a running procedure: close it first. Tests drives up to Distance ahead and to the left: clear that square.

## Testing

- **Ball Collection**, **Vision Ball Collection** and **Ball Collection Auto** default to `onField` off, which plans with no walls or HIVE rails: run them off the field. With it on (`collection.onField` in the auto), enter RED coordinates; routes keep to this alliance's half.
- **Close Flower Auto**: the `toHive` leg puts part of the robot over the centre line (G402).
- **Decode Tele** starts at the placeholder pose: reset it with a gamepad1 pose button before trusting the odometry-only turret aim.
- **Turret Aim Test**: calibrate each turret (`Turret.Tuning`'s `nectar` and `pollen`) first: `leftPosition`/`backPosition`/`rightPosition` (dpad left/down/right) facing exactly robot-left, back and right, since AUTOAIM interpolates between them, and `offset` so its two servos don't fight, keeping each end at least `offset` inside 0 to 1; `x`/`y` is its pivot, forward and left of the robot's centre. Set the start pose before INIT.
- **Ivy Path Test** loops a curve to 4 ft ahead and 4 ft left of wherever the robot stands, and back: run it off the field with about 5.5 ft clear ahead and to the left.
- **OctoQuad**: use the SDK's own driver (type `OctoQuadFTC`); don't add DigitalChickenLabs' `OctoQuadFWv3`.
- **Laser rangefinders** (Brushland): switch one to analog with Configure Laser Rangefinder on `laser_rangefinder_i2c.xml`, then set every reader's `maxRangeMm` to its `analogHighMm` (readers assume `analogLowMm` 0) or distances scale wrong. Back to I2C: INIT and stop that OpMode with the sensor unplugged, then plug it in (LED turns magenta).
- **Transfer Count Test** / **Transfer Thread Count Test**: `analog0` (A) sits upstream of `analog1` (B), close enough that one ball covers both; balls must only travel A to B and start with the chute clear.
- **Hub-reading threads** (reference: `modules/TransferCounter`): poll at 10 ms, the hub's ADC refresh; faster only costs loop time (`docs/hub-bench-findings.md`). Any such thread:
  - Call `start()` on the OpMode thread and `stop()` from the OpMode's stop, not `Module.stop()`, which also runs mid-OpMode.
  - Loop only while the OpMode thread `isAlive()`: after a crash the SDK calls no OpMode hook, so that is the only stop signal.
  - Stop with a volatile flag and `join`, never `interrupt()`: the SDK's `MessageKeyedLock.release` calls `lockInterruptibly()`, so an interrupt mid-command can leave the hub's USB lock held until it's abandoned after 500 ms, stalling every thread's hub commands.
  - Catch `Throwable` into a volatile field and rethrow it on the OpMode thread.
  - Read with its own `LynxGetBulkInputDataCommand`, never through `AnalogInput` or `clearBulkCache()`: the MANUAL bulk cache is shared across threads.
  - No writes or telemetry from the thread.
- **Flywheel RPM** from a Through Bore: the hub's 16-bit velocity overflows above about 240 RPM at 8192 counts/rev, so compute it from position over a window (Shooter Test's `rpmWindowMs`).
- Single devices: the dashboard's Hardware View.

**Control Hub bench** (`scripts/bench/`): read `docs/hub-bench-findings.md` before changing the planner, hub or I2C traffic, odometry, Limelight or stop paths.

## Conventions

- An SDK bump sets `versionCode`/`versionName` to that SDK's FtcRobotController manifest values.
- Comments: none by default, in any language; only a one-line WHY a competent reader would otherwise get wrong. Usage notes go in this file.
- Vendored, never hand-edited, never holding tuned values (those go in `BettaConstants`/`OctoQuadLocalizerTest`): `vendor/<source>/` (its `*_REV` names upstream and the only edits), `pedro/procedures/`, `robotcontroller/` and every `res/` file but the hardware configs. Refresh Prism and the procedures with `scripts/update-*.sh`, one commit each; the procedures' commit also carries any Pedro version bump the Quickstart made, in `TeamCode/build.gradle` (the script doesn't touch it); `robotcontroller/` and `res/` from the SDK's FtcRobotController at its tag; the rest by recopying upstream and reapplying its `*_REV` edits.
- Fail fast: no catch to keep a loop alive, no silent fallback, no optional devices (`hardwareMap.get`, never `tryGet`); numeric guards and try/finally are fine. Only exceptions: a webcam setting the camera rejects raises the `Webcam` fault and a Limelight problem the `Limelight` fault instead of throwing, with Limelight-dependent actions falling back.
- `Limelight3A`:
  - Every `boolean` call but `isRunning`/`isConnected` is an HTTP POST or DELETE that blocks up to 15 s and fails as a bare `false`: make them only from `LimelightSync`'s one background thread (newest request wins), and only when what's wanted changes.
  - `LLResult.getPipelineType()` is always empty (use `getStatus()`); llpython keeps the last script's output on every pipeline.
  - Poll errors are swallowed: health is `isConnected()`, `getTimeSinceLastUpdate()` and the result's contents.
  - `setPollRateHz` is ignored while polling: `stop()` first (`pause()` isn't enough). The rate persists across OpModes until the next deploySloth or app restart.
- OpModes always set `name`; `group` is exactly `"Test"`, or `"A"` for the competition TeleOp.
- Commits: one imperative subject line, no body, no trailer. Ask before pushing.
