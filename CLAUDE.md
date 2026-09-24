# CLAUDE.md

Guidance for Claude Code when working in this repository.

## Project context

The 2026–2027 competition-season repository for **FIRST Tech Challenge team 6165 MSET Cuttlefish**, for the **BIOBUZZ™** game (kickoff September 12, 2026). The game elements are **Pollen**, ~2.8" yellow plastic balls, and **Nectar**, ~3.6" red and blue plastic balls; `modules/vision/` detects and tracks them.

Game-specific code (field coordinates, scoring logic, mechanism modules) goes under `biobuzz/`, `modules/`, and `opmodes/`. `decode/` is last season's DECODE robot code, trimmed to what exercises the architecture: its own `modules/` (`Shooter` with hood, `ShooterInterpolation`, `Turret`, and a basic `Magazine`: intake, vertical transfer and two pushers, no sorting or sensors) and `opmodes/`, plus BioBuzz's `modules/Drivetrain`, so a Drivetrain change reaches Decode Tele too; nothing outside `decode/` depends on it. Nothing game-specific goes under `architecture/`, which stays portable to the next season. Field constants come from the game manual, not guesses.

Besides the Gradle and repo files, the top level holds only `TeamCode/` (the whole app; `TeamCode/libs/` has the keystore and the committed slothboard maven repo), `limelight/` (code that runs on the Limelight, not in the APK), `scripts/` (vendored-code refresh scripts) and `docs/`. New files go in one of these, and new Java files in an existing package.

## Keep this file fresh

Claude maintains this file without being asked: a change to how the project is built, deployed, structured, tuned or tested, or to a convention, updates the affected section in the same commit, and discrepancies found while working get fixed. It says what is true now and what to do about it, for people using the architecture. Keep entries short: what a thing is, where it lives, how to use it, and its footguns.

## Build & deploy

Deploy from the buttons on Android Studio's top bar: pick a run configuration in the dropdown and press Run. IntelliJ IDEA users add the same buttons to their toolbar. There are two configurations, set up once per laptop as in Sloth's README (`.idea/` is not committed):

- **TeamCode**, the app configuration, builds and installs the whole app. Give it a before-launch Gradle task `removeSlothRemote` (Gradle project `:TeamCode`), placed first, so no old hot-load survives the install. Use it for the first deploy to a hub, after wiping the hub or bumping Sloth, and after any change to `robotcontroller/internal/`, dependencies, the manifest, or resources.
- **deploySloth**, a Gradle configuration running `deploySloth` on `:TeamCode`, hot-reloads everything under `org.firstinspires.ftc.teamcode` in seconds. Use it for every other change; it needs the full app already on the hub.

- A finished `deploySloth` means the push finished, not that the hub loaded it (the Load plugin waits on the wrong lock filename). Check the RC screen or DS log, don't deploy again while a load is in progress, and INIT nothing until the load has applied.
- The Sloth tasks auto-connect adb to `192.168.43.1` when no device is attached, then run a bare `adb disconnect`, which drops every network adb device including any `adb forward`. Deploying over USB or to another address needs a `load { address = "..."; autoconnect = dev.frozenmilk.sinister.sloth.AutoConnect.NEVER }` block in `TeamCode/build.gradle`.
- If an old hot-load ever keeps replacing new code, run `removeSlothRemote` on its own (Gradle tool window, `TeamCode` → Tasks → install).
- **The Driver Station must be on 12.0.** An 11.x DS still runs OpModes but fails the Robot Controller Inspection screen that field inspectors check. Install `FtcDriverStation-release.apk` from the `FtcRobotController` `v12.0` release over it.
- **Build JDK.** The Gradle daemon is pinned to JDK 25 in `gradle/gradle-daemon-jvm.properties`; any JDK 17 or newer works, but whatever is pinned must be installed. Gradle 9.1.0 and AGP 8.13.2 need Android Studio Narwhal 3 Feature Drop or newer.
- Pure Java, no Kotlin plugin. There is no test suite; this is a robot controller APK.
- Claude has no IDE, so it checks its work from the terminal: `./gradlew :TeamCode:compileDebugJavaWithJavac`, or `:TeamCode:assembleDebug` for the full APK.

## Dependencies

**Single-module app.** The SDK comes from Maven Central as `org.firstinspires.ftc:*:12.0.0`; there is no `FtcRobotController` module, and the template's activity classes live in `robotcontroller/internal/`. `abiFilters` is `arm64-v8a` only (Control Hub).

**Sloth and slothboard move together.** Sloth `0.3.2` (`TeamCode/build.gradle`), the root `dev.frozenmilk.sinister.sloth.load` plugin `0.3.2`, and the dashboard's `{sloth}` prefix must match, because slothboard pins Sloth strictly. The dashboard is the team's slothboard build, `com.acmerobotics.slothboard:dashboard:0.3.2+0.6.0-6165.3`, served from the committed `TeamCode/libs/m2/`; its source is the team fork `github.com/6165-MSET-Cuttlefish/slothboard`, whose tag `0.3.2+0.6.0-6165.3` is the committed build; bumping it means building a new tag there, then replacing the six files and the version string. Keep `exclude group: 'com.acmerobotics.dashboard'` so the upstream dashboard never reaches the classpath. Dashboard: port 8080, websocket 8000.

**Pedro Pathing 3.0.1**: `com.pedropathing:revhub:3.0.1` (pulls `core`), `com.pedropathing:tuning:1.0.1` for AutoTune, and Ivy, Pedro's command framework, `com.pedropathing.ivy:pedro:1.1.1`. 3.0 rules: `com.pedropathing.math.Pose` is immutable (`x()/y()/heading()`, heading normalized to [0, 2π)); paths come from `com.pedropathing.api.Paths` and must have a heading interpolator or they throw; per-path overrides are `Modifier`s (`ConfigVar.at(...)` via `Path.with(...)`).

## Framework layout

Everything framework-level is under `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/architecture/`.

- `core/` — `EnhancedOpMode` (the base OpMode), `Robot` (game-agnostic base; subclasses build mechanisms in `initializeGameModules()` and the follower in `createFollower(HardwareMap)`, which runs first), `Module`, `State`, `AllianceColor`, `Context`.
- `command/` — the Ivy command factories: `StateCommands` for Module state machines, `PathCommands` for Pedro paths.
- `auto/` — `FieldConfig.fieldWidthInches` (`@Config`, 141.5); `FieldPose.forAlliance(x, y, heading)`, which mirrors RED geometry to BLUE as `(width - x, y, π - heading)` when `Context.allianceColor` is BLUE (author every pose for RED; a bare radian passed to a path is not mirrored, so read headings from a `FieldPose`); `FieldVisualization` (dashboard overlay); `PoseRing` (pose trail).
- `control/` (PID), `hardware/` (cached motor/servo wrappers, voltage, encoders, the Brushland rangefinder), `input/` (gamepad layering and edge detection), `telemetry/` (`DualTelemetry` DS+dashboard fan-out, `HtmlFormatter` DS markup helpers, `FieldMapRenderer` braille DS field map, `LoopProfiler`), `prism/` (vendored goBILDA Prism LED driver), and `OptimizationToggles` (framework-wide `@Config` perf toggles).

`Robot`'s constructor sets the follower pose to a placeholder, `(72, fieldWidth - 10, 90°)`, on every init so nothing carries across a Sloth reload. An auto sets its real start with `robot.follower.setPose(FieldPose.forAlliance(...))` in `initialize()` before building paths. The alliance is `Context.allianceColor`, set on the dashboard (`Context → allianceColor`) or in `Context.java`. It is a static, so it carries from one OpMode to the next until the app restarts or Sloth reloads; no OpMode sets it except Decode Tele's in-match swap.

Game code: `biobuzz/` (`BioBuzzRobot` builds the modules and the follower, `BioBuzzOpMode` is the base OpMode with a typed `robot`, `RobotActions` holds the game's Ivy commands as `robot.actions`), `modules/` (mechanisms; `Drivetrain` needs `withFollower(...)` for field-centric drive, heading lock and `isBonk()`), `opmodes/tele/` and `opmodes/test/`. `pedro/` follows the Pedro Quickstart layout: one constants file per robot (see Tuning), `Tuning.java` (`@Tuner` registrations) and `procedures/` (vendored).

Vision (`modules/vision/`, on the Cuttle bot) detects Pollen and red/blue Nectar. `TrackedBall` is camera-relative, so a stationary ball reads as moving when the robot moves; `FieldBall` is field-relative, computed from the robot pose at each frame's capture time, and exists only after `Camera.withFollower(...)`. `FieldBallTracker` keeps a ball's ID when the robot turns away and back; a ball out of view stays in `getFieldBalls()` and the nearest/fastest/moving getters at its last position, with `visible()` false, until `FieldBallTracking → forgetAfterSeconds` (10 s), so check `visible()` before chasing one. Every detection number and the `H_ARRAY` homography live in `BallVisionConstants`: the thresholds are live `@Config` categories (`BallVision_*`), while `H_ARRAY` and the ROI/Hough constants are `static final` and edited in source; `BallVisionDisplay` picks the camera view (`MASK`, `OVERLAY`, `BOX`). `eocvsim/balldetection/` is a standalone EOCV-Sim copy of the detector, without the mask-blob fallback and with its own older constants, since EOCV-Sim can't import TeamCode; `eocvsim/homography/` produces `H_ARRAY` from the calibration chessboard, printed with a white border about one square wide. `H_ARRAY` is in 640x480 pixels, the size `WebcamSession` streams, so the EOCV-Sim source must be 640x480 too (the pipeline prints its input size).

`modules/LimelightCamera` owns the Limelight 3A (`limelight`) and reports whether this alliance's HIVE cell has tipped (`isTipped()`) or is scorable (`isScorable()`), shown as a `Cell` telemetry line in every OpMode that builds it. `RobotActions.checkTip()` is the Ivy form; it also finishes after `checkTipTimeoutMs`, so read `isTipped()` afterwards. The verdict comes from a SnapScript on the Limelight: this alliance's cluster reading upside-down is scorable, and right-side up or out of frame is tipped, each once it has held for 0.25 s. That inverts the roll heuristic in FIRST's "AprilTag Clusters" Tech Tip, so the Limelight must be mounted image-upright, looking the way the launcher launches. Pipelines 1 (red) and 2 (blue) hold the scripts and are chosen when modules init, from `Context.allianceColor`, so set the alliance on the dashboard before pressing INIT, or in `createRobot()` (`initialize()` is too late); pipeline 0 is DECODE's AprilTag pipeline, owned by `decode/modules/Turret`, so `DecodeRobot` never builds `LimelightCamera`. The `llpython` slots are listed in `LimelightCamera`'s `OUT_*` constants and the script's docstring; edit them together.

To change the script or the tag ids (the generator reads the ids and pipeline indices from `LimelightCamera`), edit it, run `limelight/generate_pipelines.py`, and upload each file in `limelight/pipelines/` from the Limelight web UI (`http://limelight.local:5801`: pipeline index from the file name, Input tab, pipeline type Python, exposure 1000, which the hub cannot set). A wrong script or pipeline never gives a wrong verdict: it shows NO VERDICT and the warning "incorrect pipeline! please switch manually".

## OpMode lifecycle

`init()`: `State.clearModuleBindings()` and `Scheduler.reset()` (both are statics that outlive an OpMode; Ivy's Scheduler, being library code, also survives a Sloth reload); Lynx hubs to manual bulk caching; `createRobot()`; discover and init modules (`initStates()` binds State→Module and applies initial values, then `init()`); `initialize()`; a second discover-and-init pass for modules created inside `initialize()`; sort telemetry modules; snapshot the field map.

User hooks: `createRobot()` (required), `initialize()`, `initializeLoop()`, `onStart()`, `gameLoop()`, `onLoopStart()`, `onEnd()`, `shouldReadDuringInit()` (default true), `shouldWriteDuringInit()` (default false), `telemetry()` for DS/dashboard lines, and `dashboardOverlay(Canvas)` for field drawings. Draw in `dashboardOverlay`; packets are rebuilt every loop and sent only on some, so drawings elsewhere flicker.

`init_loop()` and `loop()` share one pipeline; `loop()` calls `gameLoop()` where `init_loop()` calls `initializeLoop()`, and `init_loop()` gates reads on `shouldReadDuringInit()` and writes on `shouldWriteDuringInit()` plus a 500 ms grace. Only module I/O is gated: `robot.follower.update()` and `Scheduler.execute()` run in init too, so a follow/hold/manual call or path command before `onStart()` drives the robot in INIT.

```
telemetry.setEnabled        // applies Robot.telemetryToggles
clearBulkCaches             // every loop, manual mode
InputClock.advance          // edge suppliers refresh off this
updateVoltageThrottled      // every 50 loops
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

`stop()`: `Scheduler.reset()`, which does not run command `end()` hooks; then `module.stop()` on every module and `onEnd()`, each guaranteed to run even if an earlier one throws, with the first exception rethrown afterwards. The SDK fail-safes every REV hub before the OpMode's `stop()`; a module's `stop()` covers what that misses, like the webcam and Limelight.

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
        flywheel.setPower(0);
    }
}
```

- `setStates(...)` once per state class; `bindTunable` after it. Don't touch states in the constructor; bindings exist only from `initStates()` on, and `init()` runs after that.
- Tunables are `public static` fields on the `@Config` module class, optionally grouped in plain nested holder classes. A nested class that is itself `@Config` must be named (`@Config("Shooter")`): slothboard keys config classes by simple name, so bare nested `Tuning` classes collide and vanish.
- Telemetry goes in `onTelemetry()` via `log`/`logDS`/`logDashboard` (both screens, DS only, dashboard only), never in `read()`. Dashboard data rows stay plain text so its graph view can read them.
- `stop()` commands hardware directly (`motor.setPower(0)`); `write()` never runs after it.
- `EnhancedMotor`'s one-argument `setVelocity`/`getVelocity` are in encoder ticks per second, not RPM, and `withVoltageCompensation` scales only `setPower`.
- `setStartupCommand(command)` arms a command that `start()` schedules once. `state.activate()` returns false only if the state class is unregistered or a guard rejected it.

## Commands

Ivy is the only scheduler, and the framework owns it: `Scheduler.execute()` runs once per loop between `gameLoop()` and `writeModules()`, and `Scheduler.reset()` runs at init, `start()`, and `stop()`. Never call either yourself.

**Build commands in `initialize()`, schedule them in `onStart()`.** `start()` resets the scheduler, so anything scheduled during init is dropped. `StateCommands.set(...)` resolves each State's Module at build time and throws if it isn't bound yet, which is also why building must wait for `initialize()`.

**Requirements are the objects a command owns**: `Module` instances (`StateCommands.set` requires the named states' modules) or the `Follower` (every `PathCommands` command). A new command whose requirements collide with a running one of equal priority interrupts it (`OVERRIDE`), and a group requires the union of its children's, so a whole-auto group holds every module it touches for its entire run: a `StateCommands.set(...)` scheduled from `gameLoop()` on one of those modules kills the **whole** auto, which does not resume. For a mid-auto reaction, build it into the sequence, or give the auto `.setPriority(1)` and the reaction `.setBlockedBehavior(BlockedBehavior.QUEUE)`.

- `StateCommands.set(State...)` — instant; activates each state in order. `setLazy(module, supplier)` resolves the state at start.
- `PathCommands.follow(f, path)` — finishes once the robot is at the end and Pedro's hold has settled. `followThrough(f, path)` — no end hold; finishes at the end with whatever error remains. `followUntilRemaining(f, path, inches)` — finishes `inches` short of the end of the whole path and leaves Pedro driving the tail, so the next command overlaps it. All three stop the follower if they are interrupted (a timeout, a race, or a newer path command), so an aborted auto stops in place. `hold(f)` / `hold(f, pose)`, `stop(f)`, `timeout(command, ms)`, and `remainingBelow(f, inches)` for `waitUntil` triggers (current leg only).
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
2. **Pedro constants** in `pedro/`, one file per robot: motor names and directions, Pinpoint offsets and directions, Foresight controllers. `BettaConstants` is what `Tuning.java` and every BioBuzz `Robot` use. `CuttleConstants` is the Cuttle bot AutoTuned with front and back swapped; `CuttleDecodeConstants` is the same robot in the DECODE frame, used by `DecodeRobot`; it shares `CuttleConstants`' Foresight block, but its Mecanum and Pinpoint blocks are separate, so retuning those in one file does not update the other. No Robot builds `CuttleConstants` yet. Tune with AutoTune, not the dashboard. Keep the robot still for the first second of init while the Pinpoint recalibrates its IMU.
3. **`OptimizationToggles`** for telemetry/dashboard cadence and the profiler, and **`Robot.telemetryToggles`** (`dsTelemetry`, `dashboardTelemetry`, `voltage`, `current`, `loopProfile`, the switch for the per-section loop-time breakdown), applied at the top of every loop. `loopProfileTelemetryByDefault` is read once at class load; flip `telemetryToggles.loopProfile` at runtime instead.
4. **`BallVisionConstants`'s per-type HSV classes** for ball colours. `docs/hsv-tuning-prompt.md` derives starting values from photos with a vision-capable AI; confirm them live in **Ball Vision**'s `MASK` view.
5. **`BallFieldTransform.Mount`** (`@Config("CameraMount")`) is the camera's position on the robot. For `xIn`/`yIn`, run **Ball Vision**, mark where the drawn (0,0) crosshair lands, and measure from it to the robot's pose reference (the point the Pinpoint offsets are measured from). For `mirrorY` and `headingDeg`, hold the robot still under **Ball Field Drive** or **Camera Module Test**: flip `mirrorY` if a ball on the robot's left reads negative Y, nudge `headingDeg` until a ball on the centerline reads Y ≈ 0, then check a few more points.
6. **Cell-tip detection**: `LimelightCamera`'s `@Config` fields (pipeline indices, `maxStalenessMs`, `checkTipTimeoutMs`) on the hub; everything else is in `limelight/cell_tip_snapscript.py` (regenerate and re-upload).

**AutoTune:** deploy, join the robot wifi, open `http://192.168.43.1:10158` (websocket on 12649; over USB forward both and use `localhost`). Run **Mecanum**, **Pinpoint** (it measures directions and offsets itself), **Foresight**, then **Tests**, and paste each emitted block over the matching block in the constants file `Tuning.java` builds from. The Tests procedure ignores its Distance field and always drives 48 in (upstream bug), so leave it at 48 and clear a 48 × 48 in area. Re-run them after any drivetrain, wheel, or odometry-pod change.

A `@Tuner` method in `Tuning.java` must be static, take no arguments, and be declared to return exactly `Procedure`; anything else breaks the RC app at boot.

## Testing

**Hub configs.** Every framework OpMode needs `fl`, `bl`, `fr`, `br` and `pinpoint`, because `Robot` always builds the follower, and every OpMode that builds `Drivetrain` also needs `leftPto`/`rightPto` (Servos, held at `ptoConfig` UP on every write so the drive stays decoupled from the lift); `floodgate` (configured as an Analog Input; any other type silently disables current telemetry and the current limiter), `nerdDetector` (the webcam; the name lives only in `Camera.WEBCAM_NAME`) and `limelight` are optional (without `limelight`, building `RobotActions.checkTip()` throws). `res/xml/` ships `cuttle_decode.xml` (the Cuttle bot, for every framework OpMode on it including the DECODE ones) and `camera.xml` (webcam only, for **Camera Tune** and **Ball Vision**; breaks every framework OpMode).

- **Mock Architecture Test** (`opmodes/test/MockAuto`): the end-to-end smoke test. Run it first after a full install. It doesn't move unless `Mock Auto → enableDrive` is on. Check motor and pod directions with the wheels up, then run it on the floor with a clear `driveInches` lane; on a stand the pods don't move, so the drive never finishes and runs until `safetyTimeoutMs`.
- **Camera Module Test** runs `modules/Camera` through the framework, so it also needs the drivetrain and Pinpoint wired up.
- **Ball Vision** (`opmodes/test/BallVisionTest`) runs the detector straight off the webcam with only `nerdDetector`: the webcam bench check, and where to tune HSV live.
- **Ball Field Drive** (`opmodes/test/BallFieldDriveTest`) drives on raw mecanum and reports ball field positions from the Pinpoint read directly, with offsets copied from `CuttleConstants` (keep them in sync). Its pose is relative to where it started, and it is the check that a ball keeps its ID when the robot turns away and back.
- **Cell Tip Test** (`opmodes/test/CellTipTest`, on `cuttle_decode.xml`) runs `RobotActions.checkTip` against the live verdict; run it after uploading scripts. Your alliance's cluster upside-down should read SCORABLE, and right-side up or out of frame TIPPED, each a quarter second later. It uses `Context → allianceColor`, which applies on the next init.
- **BioBuzz Tele** is the robot-centric drive smoke test: brake mode on start, left trigger toggles 75% slow mode, left bumper toggles heading lock at the current heading.
- **Decode Tele** and **Decode System Check** (`decode/opmodes/`, on `cuttle_decode.xml`) are last season's TeleOp and pit check on the new framework: drive, shooter and hood, turret, the basic intake, vertical transfer and pushers, alliance swap, pose resets and Limelight relocalization (no sorting, sensors, lights or endgame; the PTO hold lives in `Drivetrain`). The check covers odometry, the Limelight, a forward drive, a turret sweep, the flywheel/hood, intake and vertical motor current, and the pushers' OPEN_SHOOT and OPEN positions. Tele starts at the placeholder pose, not an auto's end pose, so reset the pose or relocalize before trusting turret aim.
- **Distance sensors** (`opmodes/test/distance/`): bench tests and address setup for the laser, ultrasonic and REV sensors, plus the predictive-braking experiment built on them; they read the hub config names `Laser`, `sonar` and `distance`.
- For single-device bench tests of motors, servos and colour sensors use FTC Dashboard's Hardware View; it has no distance-sensor view. For an isolated module test, extend `OpMode` directly rather than `EnhancedOpMode`.
- SDK sample OpModes are not in this repo; read them on GitHub at the tag matching `TeamCode/build.gradle`.

## Conventions

- `JavaVersion.VERSION_1_8` and `targetSdkVersion 28` are pinned by the SDK; `compileSdk 35`, `minSdk 24`, and `ndkVersion 21.3.6528147` are deliberate. `versionCode`/`versionName` track the SDK version. Don't change any without a reason.
- `TeamCode/libs/ftc.debug.keystore` is the FTC standard keystore. `TeamCode/libs/OpModeAnnotationProcessor.jar` is not wired into the build; add `annotationProcessor files('libs/OpModeAnnotationProcessor.jar')` if the compile-time OpMode checks are wanted.
- **Comments are brief and only where necessary**, in every language. Default to none; add one only for a WHY a competent reader would otherwise get wrong (an ordering invariant, a hardware or SDK quirk, a deliberate deviation), one line where possible. Never restate code, narrate design, or add banners, history, or commented-out code; usage guidance belongs in this file.
- **Generated code** (`limelight/pipelines/*`) is never hand-edited; regenerate it with `limelight/generate_pipelines.py` after editing `limelight/cell_tip_snapscript.py` or `LimelightCamera`'s tag ids.
- **Vendored code** (`architecture/prism/*`, `architecture/hardware/LaserRangefinder.java`, `pedro/procedures/*`, `robotcontroller/internal/*`) is never hand-edited. Refresh Prism with `scripts/update-prism.sh` (pin in `prism/PRISM_REV`) and the Quickstart procedures with `scripts/update-pedro-quickstart.sh` (pin in `procedures/QUICKSTART_REV`), each as its own commit; if the Quickstart bumped Pedro versions, bump `TeamCode/build.gradle` in the same commit.
- **Fail fast.** Let exceptions propagate; no try/catch to keep a loop alive. Numeric guards and try/finally are fine. Exception: `modules/vision/WebcamControls.java` deliberately ignores unsupported UVC control calls.
- OpMode `group` is `"Test"` for test OpModes and `"A"` for the competition teleop; the DS treats group names as case-sensitive.
- Commit messages are a single imperative subject line, no body, no trailer. Ask before pushing.
