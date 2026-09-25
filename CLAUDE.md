# CLAUDE.md

Guidance for Claude Code when working in this repository.

## Project context

FTC team 6165 MSET Cuttlefish's 2026–27 **BIOBUZZ™** season repository. The game elements are **Pollen** (~2.8" yellow balls) and **Nectar** (~3.6" red and blue balls).

Game-specific code (field coordinates, scoring logic, mechanism modules) goes under `biobuzz/`, `modules/`, and `opmodes/`. `decode/` is last season's DECODE robot code, trimmed to what exercises the architecture (shooter, turret, basic intake and transfer); it reuses BioBuzz's `modules/Drivetrain`, so a Drivetrain change reaches Decode Tele too, and nothing outside `decode/` depends on it. Nothing game-specific goes under `architecture/`, which stays portable to the next season. Field constants come from the game manual, not guesses.

Besides the Gradle and repo files, the top level holds only `TeamCode/` (the app), `limelight/` (code that runs on the Limelight, not in the APK), `scripts/` (vendored-code refresh scripts) and `docs/`. New files go in one of these, and new Java files in an existing package.

## Keep this file fresh

Claude maintains this file without being asked: a change to how the project is built, deployed, structured, tuned or tested, or to a convention, updates the affected section in the same commit, and discrepancies found while working get fixed. It says what is true now, for people using the architecture; keep entries short: what a thing is, where it lives, how to use it, and its footguns.

## Build & deploy

Deploy from Android Studio's top-bar run configurations (IntelliJ IDEA users add the same ones), set up once per laptop as in Sloth's README (`.idea/` is not committed):

- **TeamCode** (the app configuration) installs the whole app. Give it `removeSlothRemote` (`:TeamCode`) as its first before-launch Gradle task so no old hot-load survives. Use it for the first deploy, after wiping the hub or bumping Sloth, and after changing `robotcontroller/internal/`, dependencies, the manifest or resources.
- **deploySloth** (a Gradle configuration on `:TeamCode`) hot-reloads `org.firstinspires.ftc.teamcode` in seconds; use it for everything else.

- A finished `deploySloth` means the push finished, not the load (the Load plugin waits on the wrong lock file): check the RC screen or DS log before deploying again or pressing INIT.
- The Sloth tasks auto-connect adb to `192.168.43.1` when no device is attached, then run a bare `adb disconnect`, which drops every network adb device and `adb forward`. For USB or another address, add `load { address = "..."; autoconnect = dev.frozenmilk.sinister.sloth.AutoConnect.NEVER }` to `TeamCode/build.gradle`.
- If an old hot-load ever keeps replacing new code, run `removeSlothRemote` on its own (Gradle tool window, `TeamCode` → Tasks → install).
- **The Driver Station must match the SDK version**: an older DS runs OpModes but fails the inspection screen. Install `FtcDriverStation-release.apk` from the matching `FtcRobotController` release.
- **JDK 17** must be installed; the Gradle daemon is pinned to it in `gradle/gradle-daemon-jvm.properties`. The Gradle and AGP versions need Android Studio Narwhal 3 Feature Drop or newer.
- Pure Java, no Kotlin plugin. There is no test suite; this is a robot controller APK.
- Claude has no IDE, so it checks its work from the terminal: `./gradlew :TeamCode:compileDebugJavaWithJavac`, or `:TeamCode:assembleDebug` for the full APK.

## Dependencies

Versions live in `TeamCode/build.gradle` and the root `build.gradle`.

**Single-module app.** The SDK comes from Maven Central as `org.firstinspires.ftc:*`; there is no `FtcRobotController` module, and the template's activity classes live in `robotcontroller/internal/`. `abiFilters` is `arm64-v8a` only (Control Hub).

**Sloth and slothboard move together.** Sloth, the root `dev.frozenmilk.sinister.sloth.load` plugin, and the `{sloth}` prefix of the slothboard version must match, because slothboard pins Sloth strictly. The dashboard is the team's slothboard build, served from the committed `TeamCode/libs/m2/` and built from the same-named tag on the team fork `github.com/6165-MSET-Cuttlefish/slothboard`; to bump it, tag a new build there and replace the six files and the version string. Keep `exclude group: 'com.acmerobotics.dashboard'` so the upstream dashboard never reaches the classpath. Dashboard: port 8080.

**Pedro Pathing 3**: `com.pedropathing:revhub` (pulls `core`), `com.pedropathing:tuning` for AutoTune, and Ivy, Pedro's command framework (`com.pedropathing.ivy:pedro`). 3.x rules: `com.pedropathing.math.Pose` is immutable (`x()/y()/heading()`, heading normalized to [0, 2π)); paths come from `com.pedropathing.api.Paths` and must have a heading interpolator or they throw; per-path overrides are `Modifier`s (`ConfigVar.at(...)` via `Path.with(...)`).

## Framework layout

Everything framework-level is under `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/architecture/`.

- `core/` — `EnhancedOpMode` (the base OpMode), `Robot` (game-agnostic base; subclasses build mechanisms in `initializeGameModules()` and the follower in `createFollower(HardwareMap)`, which runs first), `Module`, `State`, `AllianceColor`, `Context`.
- `command/` — the Ivy command factories: `StateCommands` for Module state machines, `PathCommands` for Pedro paths.
- `auto/` — `FieldConfig.fieldWidthInches` (`@Config`); `FieldPose.forAlliance(x, y, heading)`, which mirrors RED geometry to BLUE as `(width - x, y, π - heading)` when `Context.allianceColor` is BLUE (author every pose for RED; a bare radian passed to a path is not mirrored, so read headings from a `FieldPose`); `FieldVisualization` and `PoseRing` (dashboard overlay and pose trail).
- `control/` (PID), `hardware/` (cached motor/servo wrappers, voltage, encoders, the Brushland rangefinder), `input/` (gamepad layering and edge detection), `telemetry/` (DS and dashboard output, the DS field map, `LoopProfiler`), `prism/` (vendored goBILDA Prism LED driver), and `OptimizationToggles` (framework-wide `@Config` perf toggles).

`Robot`'s constructor sets the follower pose to a placeholder on every init so nothing carries across a Sloth reload. An auto sets its real start with `robot.follower.setPose(FieldPose.forAlliance(...))` in `initialize()` before building paths. The alliance is `Context.allianceColor`, set on the dashboard (`Context → allianceColor`) or in `Context.java`. It is a static, so it carries from one OpMode to the next until the app restarts or Sloth reloads; no OpMode sets it except Decode Tele's in-match swap.

Game code: `biobuzz/` (`BioBuzzRobot` builds the modules and the follower, `BioBuzzOpMode` is the base OpMode with a typed `robot`, `RobotActions` holds the game's Ivy commands as `robot.actions`), `modules/` (mechanisms; `Drivetrain` needs `withFollower(...)` for field-centric drive, heading lock and `isBonk()`, and holds the Cuttle bot's lift PTOs disengaged; field-centric BLUE rotates the stick by π, which assumes the alliances face each other: check at kickoff), `opmodes/tele/` and `opmodes/test/`. `pedro/` follows the Pedro Quickstart layout: one constants file per robot (see Tuning), `Tuning.java` (`@Tuner` registrations) and `procedures/` (vendored).

Vision (`modules/vision/`, on the Cuttle bot) detects Pollen and red/blue Nectar. `TrackedBall` is camera-relative, so a stationary ball reads as moving when the robot moves; `FieldBall` is field-relative (from the robot pose at the frame's capture time) and exists only after `Camera.withFollower(...)`. `FieldBallTracker` keeps a ball's ID when the robot turns away and back; an out-of-view ball stays in `getFieldBalls()` and the nearest/fastest/moving getters at its last position, with `visible()` false, for `forgetAfterSeconds`, so check `visible()` before chasing one. `BallVisionConstants` holds every detection number (thresholds are live `@Config`; `H_ARRAY` and the ROI/Hough constants are `static final`). `eocvsim/balldetection/` is an older standalone copy of the detector for EOCV-Sim, which can't import TeamCode; `eocvsim/homography/` produces `H_ARRAY` from the calibration chessboard, printed with a white border about one square wide. `H_ARRAY` is in 640x480 pixels, the size `WebcamSession` streams, so the EOCV-Sim source must be 640x480 too.

`modules/CellTipCamera` reports whether this alliance's HIVE cell has tipped (`isTipped()`) or is scorable (`isScorable()`), shown as a `Cell` telemetry line. `RobotActions.checkTip()` is the Ivy form; it also finishes after `checkTipTimeoutMs`, so read `isTipped()` afterwards. `modules/vision/CellTipPipeline` runs AprilTag detection on the `nerdDetector` webcam: this alliance's cluster right-side up is scorable, upside-down or out of frame is tipped, each after `holdSeconds`. Mount the webcam image-upright, looking the way the launcher launches; an upright tag must read roll ≈ 0 on the stream overlay. The alliance's tag ids are chosen at module init from `Context.allianceColor`, so set the alliance before INIT, or in `createRobot()` (`initialize()` is too late). `CellTipCamera` and `Camera` both open `nerdDetector`, so no OpMode can build both. DECODE's `decode/modules/Turret` still owns the Limelight (pipeline 0) for its own AprilTag aiming.

Ball detection is moving to the Limelight: `limelight/ball_detection_snapscript.py` is `BallDetectionPipeline`'s detector as a SnapScript (same algorithm and HSV numbers, no tracking), uploaded as-is to pipeline 3. No hub module reads it yet; one will feed its detections into `BallTracker`. Its `llpython` layout is in the script's docstring. Its `H_ARRAY` is still the webcam's and means nothing until recalibrated for the Limelight's lens with `eocvsim/homography` at `CALIBRATION_SIZE`.

## OpMode lifecycle

`init()`: `State.clearModuleBindings()` and `Scheduler.reset()` (statics that outlive an OpMode; Ivy's also survives a Sloth reload); Lynx hubs to manual bulk caching; `createRobot()`; discover and init modules (`initStates()` binds State→Module and applies initial values, then `init()`); `initialize()`; a second discover-and-init pass for modules created inside `initialize()`; snapshot the field map.

User hooks: `createRobot()` (required), `initialize()`, `initializeLoop()`, `onStart()`, `gameLoop()`, `onLoopStart()`, `onEnd()`, `shouldReadDuringInit()` (default true), `shouldWriteDuringInit()` (default false), `telemetry()` for DS/dashboard lines, and `dashboardOverlay(Canvas)` for field drawings. Draw in `dashboardOverlay`; packets are rebuilt every loop and sent only on some, so drawings elsewhere flicker.

`init_loop()` and `loop()` share one pipeline; `loop()` calls `gameLoop()` where `init_loop()` calls `initializeLoop()`, and `init_loop()` gates reads on `shouldReadDuringInit()` and writes on `shouldWriteDuringInit()` plus a 500 ms grace. Only module I/O is gated: `robot.follower.update()` and `Scheduler.execute()` run in init too, so a follow/hold/manual call or path command before `onStart()` drives the robot in INIT.

```
telemetry.setEnabled        // applies Robot.telemetryToggles
clearBulkCaches             // every loop, manual mode
InputClock.advance          // edge suppliers refresh off this
updateVoltageThrottled
onLoopStart
readModules                 // refreshTunables() then m.read()
robot.follower.update       // Pedro odometry + path following
poseHistory.record
gameLoop                    // initializeLoop during init
Scheduler.execute           // Ivy; between user code and writes so command state lands in this write pass
writeModules                // m.write() if isWriteEnabled
updateTelemetry             // every telemetryEveryNLoops; status, modules, field map, telemetry() hook
updateDashboard             // every dashboardEveryNTelemetryFrames-th rendered frame; overlay + dashboardOverlay hook + one packet
recordLoopTime              // loop() only
```

`start()`: resets the game timer, bulk caches, throttle counters, and loop stats; `Scheduler.reset()`, which drops anything scheduled during init; schedules each Module's startup command; `onStart()`.

`stop()`: `Scheduler.reset()` (no `end()` hooks run), stop the follower and zero its drive motors, `module.stop()` on every module, then `onEnd()`; each runs even if an earlier one throws, and the first exception is rethrown afterwards. The SDK fail-safes the REV hubs first; a module's `stop()` covers what that misses, like the webcam and Limelight.

## Module pattern

```java
@Config
public class Shooter extends Module {
    public enum FlywheelState implements State {
        OFF(0), SHOOT(2500);
        FlywheelState(double ticksPerSec) { setValue(ticksPerSec); }
    }

    public static double shootTicksPerSec = 2500;

    private final EnhancedMotor flywheel;

    public Shooter(HardwareMap hw) {
        flywheel = new EnhancedMotor(hw, "flywheel");
    }

    @Override protected void initStates() {
        setStates(FlywheelState.OFF);
        bindTunable(FlywheelState.SHOOT, () -> shootTicksPerSec);
    }

    @Override protected void read() {}

    @Override protected void write() {
        flywheel.setVelocity(getState(FlywheelState.class).getValue());
    }

    @Override protected void onTelemetry() {
        logDashboard("ticks/s", "%.0f", flywheel.getVelocity());
    }

    @Override public void stop() {
        flywheel.stop();
    }
}
```

- `setStates(...)` once per state class; `bindTunable` after it. Don't touch states in the constructor; bindings exist only from `initStates()` on, and `init()` runs after that.
- Tunables are `public static` fields on the `@Config` module class, optionally grouped in plain nested holder classes. A nested class that is itself `@Config` must be named (`@Config("Shooter")`): slothboard keys config classes by simple name, so bare nested `Tuning` classes collide and vanish.
- Telemetry goes in `onTelemetry()` via `log`/`logDS`/`logDashboard` (both screens, DS only, dashboard only), never in `read()`. Dashboard data rows stay plain text so its graph view can read them.
- `stop()` is abstract and must reach the hardware: use `EnhancedMotor.stop()`, since the write cache can drop `setPower(0)`. It can run mid-OpMode with `write()` resuming afterwards, so keep it reversible.
- `EnhancedMotor`'s one-argument `setVelocity`/`getVelocity` are in encoder ticks per second, not RPM, and `withVoltageCompensation` scales only `setPower`.
- `setStartupCommand(command)` arms a command that `start()` schedules once.

## Commands

Ivy is the only scheduler, and the framework owns it: `Scheduler.execute()` runs once per loop between `gameLoop()` and `writeModules()`, and `Scheduler.reset()` runs at init, `start()`, and `stop()`. Never call either yourself.

**Build commands in `initialize()`, schedule them in `onStart()`.** `start()` resets the scheduler, so anything scheduled during init is dropped. `StateCommands.set(...)` resolves each State's Module at build time and throws if it isn't bound yet, which is also why building must wait for `initialize()`.

**Requirements are the objects a command owns**: `Module` instances (`StateCommands.set` requires the named states' modules) or the `Follower` (every `PathCommands` command). A new command whose requirements collide with a running one of equal priority interrupts it (`OVERRIDE`), and a group requires the union of its children's, so a whole-auto group holds every module it touches for its entire run: a `StateCommands.set(...)` scheduled from `gameLoop()` on one of those modules kills the **whole** auto, which does not resume. For a mid-auto reaction, build it into the sequence, or give the auto `.setPriority(1)` and the reaction `.setBlockedBehavior(BlockedBehavior.QUEUE)`.

- `StateCommands.set(State...)` — instant; activates each state in order. `setLazy(module, supplier)` resolves the state at start.
- `PathCommands.follow(f, path)` — finishes once the robot is at the end and Pedro's hold has settled. `followThrough(f, path)` — no end hold; finishes at the end with whatever error remains. `followUntilRemaining(f, path, inches)` — finishes `inches` short of the end of the whole path and leaves Pedro driving the tail, so the next command overlaps it. All three stop the follower when interrupted, so an aborted auto stops in place. `hold(f)` / `hold(f, pose)`, `stop(f)`, `timeout(command, ms)`, and `remainingBelow(f, inches)` for `waitUntil` triggers (current leg only).
- Compose with Ivy: `sequential`, `parallel` (all finish), `race` (first finishes, rest interrupted), `waitMs`, `waitUntil`, `instant`, `command.until(condition)`.

`end()` hooks do not run on OpMode stop, so a command is never the only thing putting hardware in a safe state. Wrap every auto in a timeout.

```java
// static imports: com.pedropathing.ivy.commands.Commands.*, com.pedropathing.ivy.groups.Groups.*
private Command sequence;

@Override protected void initialize() {
    Pose startPose = FieldPose.forAlliance(24, 12, Math.PI / 2);
    Pose scorePose = FieldPose.forAlliance(60, 36, Math.PI / 2);
    robot.follower.setPose(startPose);

    sequence = PathCommands.timeout(sequential(
            StateCommands.set(Intake.Mode.FORWARD),
            PathCommands.follow(robot.follower, Paths.line(startPose, scorePose).constant(scorePose.heading())),
            StateCommands.set(Gate.Position.OPEN),
            waitMs(300),
            StateCommands.set(Gate.Position.CLOSED, Intake.Mode.IDLE)),
        25000);
}

@Override protected void onStart() {
    Scheduler.schedule(sequence);
}
```

`opmodes/test/MockAuto` is the compiling reference.

## Tuning

1. **Module `@Config` fields** with `bindTunable` for setpoints.
2. **Pedro constants** in `pedro/`, one file per robot: motor names and directions, Pinpoint offsets and directions, Foresight controllers. `BettaConstants` is used by `Tuning.java` and every BioBuzz `Robot`. `CuttleConstants` is the Cuttle bot AutoTuned with front and back swapped (no Robot builds it yet); `CuttleDecodeConstants` is the same robot in the DECODE frame, for `DecodeRobot`, and shares only the Foresight block, so retune its Mecanum and Pinpoint blocks separately. Tune with AutoTune, not the dashboard. Keep the robot still for the first second of init while the Pinpoint recalibrates its IMU.
3. **`OptimizationToggles`** for telemetry/dashboard cadence and the profiler, and **`Robot.telemetryToggles`** (`loopProfile` switches on the per-section loop-time breakdown). `loopProfileTelemetryByDefault` is read once at class load; flip `telemetryToggles.loopProfile` at runtime instead.
4. **`BallVisionConstants`'s per-type HSV classes** for ball colours. `docs/hsv-tuning-prompt.md` derives starting values from photos with a vision-capable AI; confirm them live in **Ball Vision**'s `MASK` view.
5. **`BallFieldTransform.Mount`** (`@Config("CameraMount")`) is the camera's position on the robot. Set `xIn`/`yIn` by measuring from where **Ball Vision**'s (0,0) crosshair lands to the robot's pose reference (the Pinpoint offsets' origin). Then, holding still under **Ball Field Drive** or **Camera Module Test**, flip `mirrorY` if a ball on the robot's left reads negative Y and nudge `headingDeg` until a centerline ball reads Y ≈ 0.
6. **Cell-tip detection**: `CellTipDetection` (hold time, minimum tag area and count, roll threshold, detector decimation, overlay) and `CellTipCamera` (staleness, `checkTip` timeout); exposure is `WebcamControls`, shared with ball vision.
7. **Ball detection on the Limelight**: the constants at the top of `limelight/ball_detection_snapscript.py`, mirroring `BallVisionConstants`' names. Tune in the Limelight web editor with `DISPLAY_MODE = "MASK"`, then copy the values back into the file.

**AutoTune:** deploy, join the robot wifi, open `http://192.168.43.1:10158` (websocket on 12649; over USB forward both and use `localhost`). Run **Mecanum**, **Pinpoint** (it measures directions and offsets itself), **Foresight**, then **Tests**, and paste each emitted block over the matching block in the constants file `Tuning.java` builds from. The Tests procedure ignores its Distance field and always drives 48 in (upstream bug), so leave it at 48 and clear a 48 × 48 in area. Re-run them after any drivetrain, wheel, or odometry-pod change.

A `@Tuner` method in `Tuning.java` must be static, take no arguments, and be declared to return exactly `Procedure`; anything else breaks the RC app at boot.

## Testing

**Hub configs.** Every framework OpMode needs `fl`, `bl`, `fr`, `br` and `pinpoint`; BioBuzz and Decode OpModes also need `floodgate` (an Analog Input) and `leftPto`/`rightPto` (Servos), BioBuzz and camera OpModes need `nerdDetector` (named only in `Camera.WEBCAM_NAME`), and Decode OpModes `limelight`. `res/xml/` ships `cuttle_decode.xml` (the Cuttle bot, for every framework OpMode) and `camera.xml` (webcam only, for **Camera Tune** and **Ball Vision**).

- **Mock Architecture Test** (`opmodes/test/MockAuto`): the end-to-end smoke test; run it first after a full install. It drives only with `Mock Auto → enableDrive` on (confirm motor and odometry directions first), and only finishes on the floor with a clear `driveInches` lane: on a stand the pods don't move, so it runs until `safetyTimeoutMs`.
- **Camera Module Test** runs `modules/Camera` through the framework, so it also needs the drivetrain and Pinpoint wired up.
- **Ball Vision** (`opmodes/test/BallVisionTest`) runs the detector straight off the webcam with only `nerdDetector`: the webcam bench check, and where to tune HSV live.
- **Ball Field Drive** (`opmodes/test/BallFieldDriveTest`) drives raw mecanum and reports ball positions relative to its start pose, reading the Pinpoint directly with offsets copied from `CuttleConstants` (keep them in sync); it checks that a ball keeps its ID when the robot turns away and back.
- **Cell Tip Test** (`opmodes/test/CellTipTest`, on `cuttle_decode.xml`) runs `RobotActions.checkTip` against the live verdict, with the overlay on the dashboard camera stream: your alliance's cluster right-side up should read SCORABLE, upside-down or out of frame TIPPED.
- **BioBuzz Tele** is the robot-centric drive smoke test: brake mode on start, left trigger toggles 75% slow mode, left bumper toggles heading lock at the current heading. Cell-tip detection is off in it.
- **Decode Tele** and **Decode System Check** (`decode/opmodes/`, on `cuttle_decode.xml`) are last season's TeleOp and pit check on the new framework. Tele starts at the placeholder pose, not an auto's end pose, so reset the pose or relocalize before trusting turret aim.
- **Distance sensors** (`opmodes/test/distance/`): bench tests, address setup and the predictive-braking experiment for the laser, ultrasonic and REV sensors.
- For single-device bench tests of motors, servos and colour sensors use FTC Dashboard's Hardware View; it has no distance-sensor view. For an isolated module test, extend `OpMode` directly rather than `EnhancedOpMode`.
- SDK sample OpModes are not in this repo; read them on GitHub at the tag matching `TeamCode/build.gradle`.

## Conventions

- `JavaVersion.VERSION_1_8` and `targetSdkVersion 28` are pinned by the SDK; `compileSdk`, `minSdk` and `ndkVersion` are deliberate. `versionCode`/`versionName` track the SDK version. Don't change any without a reason.
- `TeamCode/libs/ftc.debug.keystore` is the FTC standard keystore. `TeamCode/libs/OpModeAnnotationProcessor.jar` is not wired into the build.
- **Comments are brief and only where necessary**, in every language. Default to none; add one only for a WHY a competent reader would otherwise get wrong (an ordering invariant, a hardware or SDK quirk, a deliberate deviation), one line where possible. Never restate code, narrate design, or add banners, history, or commented-out code; usage guidance belongs in this file.
- **Vendored code** (`architecture/prism/*`, `architecture/hardware/LaserRangefinder.java`, `pedro/procedures/*`, `robotcontroller/internal/*`) is never hand-edited. Refresh Prism with `scripts/update-prism.sh` (pin in `prism/PRISM_REV`) and the Quickstart procedures with `scripts/update-pedro-quickstart.sh` (pin in `procedures/QUICKSTART_REV`), each as its own commit; if the Quickstart bumped Pedro versions, bump `TeamCode/build.gradle` in the same commit.
- **Fail fast.** Make a problem fail loudly and early instead of working around it: let exceptions propagate, no try/catch to keep a loop alive, and no silent fallback when something is missing or wrong. Numeric guards and try/finally are fine. Exception: `modules/vision/WebcamControls.java` deliberately ignores unsupported UVC control calls.
- OpMode `group` is `"Test"` for test OpModes and `"A"` for the competition teleop; the DS treats group names as case-sensitive.
- Commit messages are a single imperative subject line, no body, no trailer. Ask before pushing.
