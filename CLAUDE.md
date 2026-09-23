# CLAUDE.md

Guidance for Claude Code when working in this repository.

## Project context

The 2026–2027 competition-season repository for **FIRST Tech Challenge team 6165 MSET Cuttlefish**, for the **BIOBUZZ™** game (kickoff September 12, 2026). The game elements are **Pollen**, ~2.8" yellow plastic balls, and **Nectar**, ~3.6" red and blue plastic balls; `modules/vision/` detects and tracks them.

Game-specific code (field coordinates, scoring logic, mechanism modules) goes under `biobuzz/`, `modules/`, and `opmodes/`. Nothing game-specific goes under `architecture/`, which stays portable to the next season. Field constants come from the game manual, not guesses.

Besides the Gradle and repo files, the top level holds only `TeamCode/` (the whole app; `TeamCode/libs/` has the keystore and the committed slothboard maven repo), `limelight/` (code that runs on the Limelight, not in the APK), `scripts/` (vendored-code refresh scripts) and `docs/`. New files go in one of these, and new Java files in an existing package.

## Keep this file fresh

Claude maintains this file without being asked. Any change to how the project is built, deployed, structured, tuned, or tested, or to a convention, updates the affected section in the same commit. Before substantive work, check the sections for the area being touched against the tree and fix discrepancies even if unasked. This file states what is true now and what to do about it, for people using the architecture; porter rationale stays out of the repo. Keep entries short: what a thing is, where it lives, how to use it, and its footguns.

## Build & deploy

```bash
./gradlew :TeamCode:assembleDebug      # debug APK at TeamCode/build/outputs/apk/debug/
./gradlew :TeamCode:installDebug       # full ADB install; required for the first deploy onto a Control Hub
./gradlew :TeamCode:deploySloth        # hot-reload everything under org.firstinspires.ftc.teamcode
./gradlew clean :TeamCode:assembleDebug
```

- `deploySloth` needs the Sloth runtime already on the hub, so `installDebug` once after wiping the hub or bumping Sloth. Everything under `org.firstinspires.ftc.teamcode` hot-reloads; changes to `robotcontroller/internal/`, dependencies, the manifest, or resources need `installDebug`.
- `BUILD SUCCESSFUL` on `deploySloth` means the push finished, not that the hub loaded it (the Load plugin waits on the wrong lock filename). Check the RC screen or DS log, and don't deploy again while a load is in progress.
- The Sloth tasks auto-connect adb to `192.168.43.1` when no device is attached, then run a bare `adb disconnect`, which drops every network adb device including any `adb forward`. Deploying over USB or to another address needs a `load { address = "..."; autoconnect = NEVER }` block in `TeamCode/build.gradle`.
- If an old hot-load ever keeps replacing new code: `adb shell rm -rf /storage/emulated/0/FIRST/dairy/sloth/*`.
- **The Driver Station must be on 12.0.** An 11.x DS still runs OpModes but fails the Robot Controller Inspection screen that field inspectors check. Install `FtcDriverStation-release.apk` from the `FtcRobotController` `v12.0` release over it.
- **Build JDK.** The Gradle daemon is pinned to JDK 25 in `gradle/gradle-daemon-jvm.properties`; any JDK 17 or newer works, but whatever is pinned must be installed (`/usr/libexec/java_home -V`). Gradle 9.1.0 and AGP 8.13.2 need Android Studio Narwhal 3 Feature Drop or newer.
- Pure Java, no Kotlin plugin. There is no test suite; this is a robot controller APK.

## Dependencies

**Single-module app.** The stock `FtcRobotController` module is gone; the SDK is consumed as `org.firstinspires.ftc:*:12.0.0` from Maven Central, and the template's `FtcRobotControllerActivity`, `PermissionValidatorWrapper`, and `FtcOpModeRegister` live under `TeamCode/src/main/java/org/firstinspires/ftc/robotcontroller/internal/`. The DS Utility menu is therefore empty; FTC Dashboard's Hardware View covers what `TestHardware` did. `abiFilters` is `arm64-v8a` only (Control Hub); don't add `armeabi-v7a`.

**Sloth + slothboard move together.** `dev.frozenmilk.sinister:Sloth:0.3.2` (`TeamCode/build.gradle`), the root plugin `dev.frozenmilk.sinister.sloth.load` `0.3.2`, and the dashboard's `{sloth}` prefix must all match, because slothboard pins its Sloth strictly. The dashboard is the team's own slothboard build, `com.acmerobotics.slothboard:dashboard:0.3.2+0.6.0-6165.2` (`{sloth}+{dashboard}-6165.{n}`), served from the committed `TeamCode/libs/m2/`: Dairy's `0.3.2+0.6.0` plus the team's dashboard features. Bumping it means replacing the six files under `TeamCode/libs/m2/` and the one version string; the rebuild procedure is not in the repo, so ask the software lead. Keep `configurations.configureEach { exclude group: 'com.acmerobotics.dashboard' }` so the upstream dashboard never reaches the classpath. Dashboard is on port 8080, websocket 8000.

**Pedro Pathing 3.0.1**: `com.pedropathing:revhub:3.0.1` (pulls `core`), `com.pedropathing:tuning:1.0.1` for AutoTune, and Ivy, Pedro's command framework, `com.pedropathing.ivy:pedro:1.1.1`. 3.0 rules: `com.pedropathing.math.Pose` is immutable (`x()/y()/heading()`, heading normalized to [0, 2π)); paths come from `com.pedropathing.api.Paths` and must have a heading interpolator or they throw; per-path overrides are `Modifier`s (`ConfigVar.at(...)` via `Path.with(...)`).

## Framework layout

Everything framework-level is under `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/architecture/`.

- `core/` — `EnhancedOpMode` (the base OpMode), `Robot` (game-agnostic base; subclasses build mechanisms in `initializeGameModules()` and the follower in `createFollower(HardwareMap)`, which runs first), `Module`, `State`, `AllianceColor`, `Context`.
- `command/` — `StateCommands`, Ivy commands that drive Module state machines.
- `auto/` — `FieldConfig.fieldWidthInches` (`@Config`, 141.5); `FieldPose.forAlliance(x, y, heading)`, which mirrors RED geometry to BLUE as `(width - x, y, π - heading)` when `Context.allianceColor` is BLUE (author every pose for RED); `FieldVisualization` (dashboard overlay); `PoseRing` (pose trail); `HeadingInterpolatorBuilder`; `PathCommands`, Ivy commands over a Pedro `Path`.
- `control/` (PID), `hardware/` (cached motor/servo wrappers, voltage, encoders, the Brushland rangefinder), `input/` (gamepad layering and edge detection), `telemetry/` (the DS+dashboard fan-out and the loop profiler), `prism/` (vendored goBILDA Prism LED driver), and `OptimizationToggles` (framework-wide `@Config` perf toggles).

`Robot`'s constructor sets the follower pose to a placeholder, `(72, fieldWidth - 10, 90°)`, on every init so nothing carries across a Sloth reload. An auto sets its real start with `robot.follower.setPose(FieldPose.forAlliance(...))` in `initialize()` before building paths.

Game code: `biobuzz/` (`BioBuzzRobot` builds the mechanism modules and the follower, `BioBuzzOpMode` is the base OpMode with a typed `robot`, `RobotActions` holds the game's Ivy commands and hangs off `robot.actions`), `modules/` (mechanisms; `Drivetrain` needs `withFollower(...)` for field-centric drive, heading lock, and `isBonk()`, and throws if those are used without one), `opmodes/tele/` and `opmodes/test/` (`opmodes/auto/` appears with the first auto). `pedro/` is laid out like the Pedro Quickstart: `BettaConstants.java` and `CuttleConstants.java` (ours; one robot's Pedro config and `create(HardwareMap)` each; every `Robot` and `Tuning.java` build from `BettaConstants`), `Tuning.java` (ours; `@Tuner` registrations), `procedures/` (vendored verbatim; never hand-edit).

Vision (`modules/vision/`, run on the Cuttle bot) detects Pollen and red/blue Nectar as `BallVisionConstants.BallType`. `TrackedBall` is camera-relative, so a stationary ball reads as moving when the robot moves. `FieldBall` is field-relative, computed from the robot state at each frame's capture time, and only exists once `Camera.withFollower(...)` has been called. `FieldBallTracker` keeps a ball's ID when the robot turns away and back; an unseen ball keeps its last position with `visible()` false until `FieldBallTracking.forgetAfterSeconds`. `BallVisionConstants` holds every detection number and the `H_ARRAY` homography, as `@Config` categories read every frame (`BallVision_Detection`, `BallVision_Pollen`, `BallVision_RedNectar`, `BallVision_BlueNectar`); `BallVisionDisplay` picks the view (`MASK` colour masks, `OVERLAY` raw detections, `BOX` tracked balls). `eocvsim/` holds the EOCV-Sim workspaces: `balldetection/` is a standalone copy of the detector without the mask-blob fallback, whose constants are separate from (and older than) `BallVisionConstants` because EOCV-Sim can't import from TeamCode; `homography/` produces `H_ARRAY` from the calibration chessboard, printed with a plain white border about one square wide.

`modules/LimelightCamera` owns the Limelight 3A (config name `limelight`) and reports whether this alliance's HIVE cell is scorable (`isScorable()`) or has tipped (`isTipped()`). `RobotActions.checkTip()` is the Ivy form; it also finishes after `checkTipTimeoutMs`, so read `isTipped()` afterwards. A camera that sees no cluster reads as tipped after `HOLD_SECONDS`. The test inverts the roll heuristic from FIRST's "AprilTag Clusters" Tech Tip (a cluster reading upside-down, `|roll| >= 90`, is scorable; right-side up is tipped), which assumes the camera looks the way the launcher launches, and the Limelight must be mounted with its image upright. It runs on the Limelight as a Python SnapScript, one pipeline per alliance, chosen once when modules init from `Context.allianceColor`, so set the alliance in `createRobot()`; `initialize()` is too late. `LimelightCamera`'s `OUT_*` constants and the script's docstring both list the `llpython` slots; edit them together.

`limelight/cell_tip_snapscript.py` is the template; `limelight/generate_pipelines.py` writes `limelight/pipelines/` from it, with the tag ids from `LimelightCamera`. Edit the template, regenerate, and upload each file from the Limelight web UI (`http://limelight.local:5801`: pipeline index from the file name, Input tab, pipeline type Python, exposure 1000). The script sends a grayscale image, so the stream is grayscale with a coloured overlay. Exposure is a pipeline setting the hub API can't set; set it by hand on both pipelines. A wrong script on an index shows as a permanent NO VERDICT, never a wrong verdict.

## OpMode lifecycle

`init()`: `State.clearModuleBindings()` and `Scheduler.reset()` (both are statics that survive hot-reloads); Lynx hubs to manual bulk caching; `createRobot()`; discover and init modules (`initStates()` binds State→Module and applies initial values, then `init()`); `initialize()`; a second discover-and-init pass for modules created inside `initialize()`; sort telemetry modules; snapshot the field map.

User hooks: `createRobot()` (required), `initialize()`, `initializeLoop()`, `onStart()`, `gameLoop()`, `onLoopStart()`, `onEnd()`, `shouldReadDuringInit()` (default true), `shouldWriteDuringInit()` (default false), `telemetry()` for DS/dashboard lines, and `dashboardOverlay(Canvas)` for field drawings. Draw in `dashboardOverlay`, never into `robot.packet`: the packet is rebuilt every loop and only sent on some, so anything drawn elsewhere flickers.

`init_loop()` and `loop()` share one pipeline; `loop()` calls `gameLoop()` where `init_loop()` calls `initializeLoop()`, and `init_loop()` gates writes on `shouldWriteDuringInit()` plus a 500 ms grace.

```
clearBulkCaches             // every loop, manual mode
InputClock.advance          // edge suppliers refresh off this
updateVoltageThrottled      // every voltageReadLoopInterval loops
onLoopStart
readModules                 // refreshTunables() then m.read()
robot.follower.update       // Pedro odometry + path following
poseHistory.record
gameLoop                    // initializeLoop during init
Scheduler.execute           // Ivy; between user code and writes so command state lands in this write pass
writeModules                // m.write() if isWriteEnabled
updateTelemetry             // every telemetryEveryNLoops; status, modules, field map, telemetry() hook
updateDashboard             // only on loops that rendered telemetry; overlay + dashboardOverlay hook + one packet
sleep / recordLoopTime      // loop() only; minLoopMs floor, loop stats
```

`start()`: resets the game timer, bulk caches, throttle counters, and loop stats; `Scheduler.reset()`, which drops anything scheduled during init; schedules each Module's startup command; `onStart()`.

`stop()`: `Scheduler.reset()`, which does not run command `end()` hooks; then `module.stop()` on every module and `onEnd()`, each guaranteed to run even if an earlier one throws, with the first exception rethrown afterwards. The SDK does not zero motors on stop, so every module's `stop()` must.

## Module pattern

```java
@Config
public class Shooter extends Module {
    public enum FlywheelState implements State {
        OFF(0), SHOOT(5400);
        FlywheelState(double rpm) { setValue(rpm); }
    }

    public static double shootRpm = 5400;

    private final EnhancedMotor flywheel;

    public Shooter(HardwareMap hw) {
        flywheel = new EnhancedMotor(hw, "flywheel").withVoltageCompensation(12.0);
    }

    @Override protected void initStates() {
        setStates(FlywheelState.OFF);
        bindTunable(FlywheelState.SHOOT, () -> shootRpm);
    }

    @Override protected void read() {}

    @Override protected void write() {
        flywheel.setVelocity(getState(FlywheelState.class).getValue());
    }

    @Override protected void onTelemetry() {
        logDashboard("rpm", "%.0f", flywheel.getVelocity());
    }

    @Override public void stop() {
        flywheel.setPower(0);
    }
}
```

- `setStates(...)` once per state class; `bindTunable` after it. Don't touch states in the constructor; bindings exist only from `initStates()` on, and `init()` runs after that.
- Tunables are `public static` fields on the `@Config` module class, optionally grouped in plain nested holder classes. A nested class that is itself `@Config` must be named (`@Config("Shooter")`): slothboard keys config classes by simple name, so bare nested `Tuning` classes collide and vanish.
- Telemetry goes in `onTelemetry()` via `log`/`logDS`/`logDashboard` (both screens, DS only, dashboard only), never in `read()`. Dashboard data rows stay plain text so its graph view can read them.
- `stop()` must command hardware directly (`motor.setPower(0)`); `write()` never runs after it.
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
            PathCommands.follow(robot.follower, Paths.line(startPose, scorePose).constant(Math.PI / 2)),
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
2. **`pedro/BettaConstants.java`** for every Pedro value (`CuttleConstants.java` holds the Cuttle bot's, but nothing builds from it yet): motor names and directions, Pinpoint offsets and directions, Foresight controllers. These are not dashboard sliders; tune them with AutoTune. Keep the robot still for the first second of init: the Pinpoint localizer recalibrates its IMU and blocks for 500 ms while it does.
3. **`OptimizationToggles`** for telemetry/dashboard cadence and the profiler, and **`Robot.telemetryToggles`** (`dsTelemetry`, `dashboardTelemetry`, `voltage`, `current`, `loopProfile`, the switch for the per-section loop-time breakdown). `loopProfileTelemetryByDefault` is read once at class load; flip `telemetryToggles.loopProfile` at runtime instead.
4. **`BallVisionConstants`'s per-type HSV classes** for ball colours. `docs/hsv-tuning-prompt.md` is a prompt for deriving starting values from photos of the balls with a vision-capable AI; confirm them live in **Ball Vision**'s `MASK` view.
5. **`BallFieldTransform.Mount`** (`@Config("CameraMount")`) is the camera's mount on the robot, which turns camera-relative detections into field positions. For `xIn`/`yIn`, run `Ball Vision`, mark where the drawn "(0,0)" crosshair lands, and tape-measure from it to the robot's pose reference (the point the Pinpoint offsets are measured from). For `mirrorY` and `headingDeg`, hold the robot still at a known heading under `Ball Field Drive` or `Camera Module Test`: flip `mirrorY` if a ball on the robot's left reads negative Y, then nudge `headingDeg` until a ball on its centerline reads Y ≈ 0. Check a few more measured points across the field of view.

6. **Cell-tip detection**: `LimelightCamera`'s `@Config` fields (pipeline indices, `maxStalenessMs`, `checkTipTimeoutMs`) on the hub; everything the detection does lives in `limelight/cell_tip_snapscript.py`, then regenerate and re-upload.

**AutoTune:** deploy, join the robot wifi, open `http://192.168.43.1:10158` (websocket on 12649; over USB forward both and use `localhost`). Run **Mecanum**, **Pinpoint** (it measures directions and offsets itself), **Foresight**, then **Tests**, and paste each emitted block over the matching block in the constants file `Tuning.java` builds from. The Tests procedure ignores its Distance field and always drives 48 in (upstream bug), so leave it at 48 and clear a 48 × 48 in area. Re-run them after any drivetrain, wheel, or odometry-pod change.

A `@Tuner` method in `Tuning.java` must be static, take no arguments, and be declared to return exactly `Procedure`; anything else breaks the RC app at boot.

## Testing

**Hub configuration names**: every framework OpMode needs `fl`, `bl`, `fr`, `br` (motors) and `pinpoint` (goBILDA Pinpoint), because `Robot` builds the follower unconditionally. Optional: `floodgate` (analog current sensor), `nerdDetector` (webcam) and `limelight` (Limelight 3A; without it `LimelightCamera` gives no verdict and building `RobotActions.checkTip()` throws). `res/xml/` ships three configs: `cuttledecode.xml`, the Cuttle bot's framework config (drive motors, Pinpoint, `limelight`, `nerdDetector`), whose ports are copied from `cuttle_decode.xml` and must be checked against the real wiring; `camera.xml`, webcam only, for **Camera Tune** and **Ball Vision**, which breaks every framework OpMode; and `cuttle_decode.xml`, last season's full Cuttle robot.

- **Mock Architecture Test** (`opmodes/test/MockAuto`): the end-to-end smoke test. Run it first after `installDebug`. It doesn't move unless `Mock Auto → enableDrive` is on, which needs wheels off the ground and verified directions.
- **Camera Module Test** runs `modules/Camera` through the framework, so it also needs the drivetrain and Pinpoint wired up.
- **Ball Vision** (`opmodes/test/BallVisionTest`) runs `BallDetectionPipeline` straight off the webcam with only `nerdDetector`: the webcam bench check, and where to tune HSV live against the dashboard's camera stream.
- **Ball Field Drive** (`opmodes/test/BallFieldDriveTest`) drives on raw gamepad mecanum and reports ball field positions from the Pinpoint read directly. Its Pinpoint offsets are the Cuttle bot's, copied from `CuttleConstants`; keep the two in sync. Its pose is relative to where the robot was at init. It runs its own `FieldBallTracker`, so it is also the check that a ball keeps its ID when the robot turns away and back.
- **Cell Tip Test** (`opmodes/test/CellTipTest`, on `cuttledecode.xml`) shows the live cell verdict and runs `RobotActions.checkTip`; run it after uploading scripts. A cell of your alliance whose tags read upside-down should read SCORABLE, and right-side up or out of frame should read TIPPED a quarter second later. `Cell Tip Test → alliance` applies on the next init.
- **BioBuzz Tele** is the robot-centric drive smoke test.
- For single-device bench tests use FTC Dashboard's Hardware View. For an isolated module test, extend `OpMode` directly rather than `EnhancedOpMode`.
- SDK sample OpModes are not in this repo; read them on GitHub at the tag matching `TeamCode/build.gradle`.

## Conventions

- `JavaVersion.VERSION_1_8` and `targetSdkVersion 28` are pinned by the SDK; `compileSdk 35`, `minSdk 24`, and `ndkVersion 21.3.6528147` are deliberate. `versionCode`/`versionName` track the SDK version. Don't change any without a reason.
- `TeamCode/libs/ftc.debug.keystore` is the FTC standard keystore. `TeamCode/libs/OpModeAnnotationProcessor.jar` is not wired into the build; add `annotationProcessor files('libs/OpModeAnnotationProcessor.jar')` if the compile-time OpMode checks are wanted.
- **Comments are brief and only where necessary**, in every language (Java, Python, XML). Default to none. Add one only for a WHY a competent reader would otherwise get wrong: an ordering invariant, a hardware or SDK quirk, a deliberate deviation. Keep it to one line where possible. Never restate code, narrate design, or add banners, history, changelog notes, or commented-out code; usage and tuning guidance belongs in this file, not in comments.
- **Generated code** (`limelight/pipelines/*`) is never hand-edited; regenerate it with `limelight/generate_pipelines.py` after editing `limelight/cell_tip_snapscript.py`.
- **Vendored code** (`architecture/prism/*`, `architecture/hardware/LaserRangefinder.java`, `pedro/procedures/*`, `robotcontroller/internal/*`) is never hand-edited. Refresh Prism with `scripts/update-prism.sh` (pin in `prism/PRISM_REV`) and the Quickstart procedures with `scripts/update-pedro-quickstart.sh` (pin in `procedures/QUICKSTART_REV`), each as its own commit; if the Quickstart bumped Pedro versions, bump `TeamCode/build.gradle` in the same commit.
- **Fail fast.** Let exceptions propagate; no try/catch to keep a loop alive. Numeric guards and try/finally are fine. Exception: `modules/vision/WebcamControls.java` deliberately ignores unsupported UVC control calls.
- OpMode `group` is `"Test"` for test OpModes and `"A"` for the competition teleop; the DS treats group names as case-sensitive.
- Commit messages are a single imperative subject line, no body, no trailer. Ask before pushing.
