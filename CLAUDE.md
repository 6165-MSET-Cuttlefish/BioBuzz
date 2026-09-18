# CLAUDE.md

Guidance for Claude Code when working in this repository.

## Project context

The 2026–2027 competition-season repository for **FIRST Tech Challenge team 6165 MSET Cuttlefish**, for the **BIOBUZZ™** game (kickoff September 12, 2026). The game elements are **Pollen**, ~2.8" yellow plastic balls, and **Nectar**, ~3.6" red and blue plastic balls; `modules/vision/` detects and tracks them.

Game-specific code (field coordinates, scoring logic, mechanism modules) goes under `biobuzz/`, `modules/`, and `opmodes/`. Nothing game-specific goes under `architecture/`, which stays portable to the next season. Field constants come from the game manual, not guesses.

**On-robot status.** AutoTune's Mecanum, Pinpoint, and Foresight procedures were run on the robot on 2026-09-17 and their output is in `pedro/Constants.java`, so the 12.0 RC app, Sloth 0.3.2, and Pedro 3.0.0 have booted and driven on a hub. Not yet exercised on a hub: the team's dashboard build, hot reload, `Mock Architecture Test` with drive enabled, any Ivy command, and vision. Claude keeps the ordered verification checklist in its own memory; ask for it.

Upstream: Sloth comes from `github.com/Dairy-Foundation/Sloth` as an artifact on `https://repo.dairy.foundation/releases`; the dashboard is the team's own build of Dairy's `ftc-dashboard` fork plus the team's dashboard PRs (see Dependencies); Pedro Pathing and its Quickstart come from `github.com/Pedro-Pathing`. The old `6165-MSET-Cuttlefish/Sloth` fork is not consumed; don't add it back.

## Keep this file fresh

Claude maintains this file without being asked. Any change to how the project is built, deployed, structured, tuned, or tested, or to a convention, updates the affected section in the same commit. Before substantive work, check the sections for the area being touched against the tree and fix discrepancies even if unasked. This file states what is true now and what to do about it, for people using the architecture; porter rationale stays out of the repo.

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
- **The Driver Station must be on 12.0.** An 11.x Driver Hub shows a permanent "Driver Station app is obsolete" banner on the RC and fails the DS's Robot Controller Inspection screen, which field inspectors check. OpModes still run, so it is easy to miss. Install `FtcDriverStation-release.apk` from the `FtcRobotController` `v12.0` release; it installs over 11.x.
- **Build JDK.** The Gradle daemon is pinned to JDK 25 in `gradle/gradle-daemon-jvm.properties`; any JDK 17 or newer works with AGP 8.13.2, but whatever is pinned must be installed (`/usr/libexec/java_home -V`). Gradle 9.1.0 and AGP 8.13.2 are the SDK 12.0 baseline (Android Studio Narwhal 3 Feature Drop or newer).
- Pure Java, no Kotlin plugin. There is no test suite; this is a robot controller APK.

## Dependencies

**Single-module app.** The stock `FtcRobotController` module is gone; the SDK is consumed as `org.firstinspires.ftc:*:12.0.0` from Maven Central, and the template's `FtcRobotControllerActivity`, `PermissionValidatorWrapper`, and `FtcOpModeRegister` live under `TeamCode/src/main/java/org/firstinspires/ftc/robotcontroller/internal/`, byte-identical to v12.0. Don't re-add the module. The DS Utility menu is therefore empty; FTC Dashboard's Hardware View covers what `TestHardware` did. `abiFilters` is `arm64-v8a` only (Control Hub); don't add `armeabi-v7a`.

**Sloth + slothboard move together.** `dev.frozenmilk.sinister:Sloth:0.3.2` and the root plugin `dev.frozenmilk.sinister.sloth.load` `0.3.2` in `TeamCode/build.gradle` and the root `build.gradle`. The dashboard is the team's own slothboard build, `com.acmerobotics.slothboard:dashboard:0.3.2+0.6.0-6165.2` (`{sloth}+{dashboard}-6165.{n}`), served from the committed `TeamCode/libs/m2/`: Dairy's `0.3.2+0.6.0` plus the team's dashboard PRs #216, #217, #218, #221, #224, #226, #229, and #230 — keyboard shortcuts, pinned config categories, log view, graph colours and layering, ordered HTML telemetry, telemetry search filter, layout sharing, and non-Mac undo/redo. Bumping it means replacing the six files under `TeamCode/libs/m2/` and the one version string in `TeamCode/build.gradle`; Claude holds the rebuild procedure in its own memory, so ask for it first. slothboard pins its Sloth strictly, so bump all three at once or resolution fails. Keep `configurations.configureEach { exclude group: 'com.acmerobotics.dashboard' }` so upstream acmerobotics dashboard never reaches the classpath. Dashboard is on port 8080, websocket 8000.

**Pedro Pathing 3.0.0**: `com.pedropathing:revhub:3.0.0` (pulls `core`), `com.pedropathing:tuning:1.0.0` for AutoTune, and Ivy, Pedro's command framework, `com.pedropathing.ivy:pedro:1.1.1`. The 2.x `ftc` artifact shares no classes with 3.0; do not add it back. 3.0 rules: `com.pedropathing.math.Pose` is immutable (`x()/y()/heading()`, heading normalized to [0, 2π)); paths come from `com.pedropathing.api.Paths` and must have a heading interpolator or they throw; per-path overrides are `Modifier`s (`ConfigVar.at(...)` via `Path.with(...)`).

**Pedro 3.0.0 bug, unreleased fix:** `Curve.pathCompletion` is inverted on straight lines and multi-leg paths, so `Interpolator.linear`, `Interpolator.piecewise`, `Path.linear(...)`, and `Follower.completion()` run backwards on them. Don't call any of those; use `HeadingInterpolatorBuilder.sweep(start, end)` or the builder for linear headings. The rule stands until a Pedro release after 3.0.0 ships.

## Framework layout

Everything framework-level is under `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/architecture/`.

- `core/` — `EnhancedOpMode` (base OpMode: module discovery, voltage compensation, dual telemetry, dashboard field rendering, command pump), `Robot` (game-agnostic base; subclasses build mechanisms in `initializeGameModules()` and the follower in `createFollower(HardwareMap)`, which runs first), `Module`, `State`, `AllianceColor`, `Context`.
- `command/` — `StateCommands`, Ivy commands that drive Module state machines.
- `auto/` — `FieldConfig.fieldWidthInches` (`@Config`, 141.5); `FieldPose.forAlliance(x, y, heading)`, which mirrors RED geometry to BLUE as `(width - x, y, π - heading)` when `Context.allianceColor` is BLUE (author every pose for RED; don't use Pedro's `PoseFactory.mirrorX`, which negates heading in 3.0.0); `FieldVisualization` (dashboard overlay); `PoseRing` (pose trail); `HeadingInterpolatorBuilder`; `PathCommands`, Ivy commands over a Pedro `Path`.
- `control/` — `PidController`. `hardware/` — `EnhancedMotor`, `EnhancedServo`, `EnhancedCRServo`, `WriteCache`, `BatteryVoltage`, `AbsoluteAnalogEncoder`, `LaserRangefinder`. `input/` — `LayerStack`, `LayeredGamepad`, `LayerGamepad`, `EdgeBooleanSupplier`, `CachedDoubleSupplier`, `InputClock`. `telemetry/` — `DualTelemetry`, `HtmlFormatter`, `FieldMapRenderer`, `LoopProfiler`. `prism/` — vendored goBILDA Prism LED driver. `OptimizationToggles` — framework-wide `@Config` perf toggles.

`Robot`'s constructor sets the follower pose to a placeholder, `(72, fieldWidth - 10, 90°)`, on every init so nothing carries across a Sloth reload. An auto sets its real start with `robot.follower.setPose(FieldPose.forAlliance(...))` in `initialize()` before building paths.

Game code: `biobuzz/` (`BioBuzzRobot` builds the mechanism modules and the follower, `BioBuzzOpMode` is the base OpMode with a typed `robot`), `modules/` (mechanisms; `Drivetrain` needs `withFollower(...)` for field-centric drive, heading lock, and `isBonk()`, and throws if those are used without one), `opmodes/tele/` and `opmodes/test/` (`opmodes/auto/` appears with the first auto). `pedro/` is laid out like the Pedro Quickstart: `Constants.java` (ours; every Pedro config and `create(HardwareMap)`), `Tuning.java` (ours; `@Tuner` registrations), `procedures/` (vendored verbatim; never hand-edit).

Vision detects both game elements — Pollen and red/blue Nectar — as `BallVisionConstants.BallType`; every `BallDetection`/`TrackedBall`/`FieldBall` carries its type, and `BallTracker` never matches a track to a detection of a different type. Vision reports in two frames. `TrackedBall` is camera-relative, rigidly attached to the robot, so a stationary ball reads as moving when the robot moves. `FieldBall` is field-relative with true velocity, produced by `BallFieldTransform` from the robot state at each frame's own capture time (the camera is tens of ms behind) including the `omega x r` term, and only once `Camera.withFollower(...)` has been called. Extrinsics are `@Config("CameraMount")` in `BallFieldTransform`. `BallVisionConstants` is the canonical home for every detection-recipe number (HSV bands per type, radius/Hough/ROI constants, the calibration board, and the shared `H_ARRAY` homography). Its live-tunable numbers are `@Config`-bound nested classes (`Detection`, plus one `PollenHsv`/`RedNectarHsv`/`BlueNectarHsv` per ball type, dashboard categories `BallVision_Detection`/`BallVision_Pollen`/`BallVision_RedNectar`/`BallVision_BlueNectar`) that `BallDetectionPipeline` reads directly every frame — no separate seed step, editing FtcDashboard takes effect immediately. `BallDetectionPipeline`'s own `Tuning` (`@Config("BallVision")`) only holds its display/behaviour toggles (`displayMode`, `drawVelocity`, `velocityArrowSeconds`), since those aren't shared with the sim copy. `OpenCVPipelines/` is a separate EOCV-Sim-facing copy of the detector, used by `PollenDetection`, with its own `PollenDetectionPipeline.java`; EOCV-Sim compiles that file in an isolated per-folder workspace (`eocvsim_workspace.json`) that can't resolve an import into the rest of TeamCode, so its constants are a hand-kept mirror of `BallVisionConstants` (its class javadoc says so) rather than a live reference — changing one requires pasting the same change into the other.

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

- State is an enum implementing `State`; each variant carries its setpoint via `setValue`. `setStates(...)` once with the initial state per state class; `bindTunable(state, supplier)` after it for live-tunable setpoints.
- Tunables are `public static` fields on the `@Config` module class, optionally grouped in plain nested holder classes. A nested class that is itself `@Config` must be named (`@Config("Shooter")`): slothboard keys config classes by simple name, so bare nested `Tuning` classes collide and vanish.
- `read()` reads sensors, `write()` commands hardware, every loop. Telemetry goes in `onTelemetry()` via `log`/`logDS`/`logDashboard` (both sinks, DS only, dashboard only), not in `read()`.
- `init()` runs once after `initStates()`. Don't touch states in the constructor; bindings don't exist yet.
- `stop()` must command hardware directly (`motor.setPower(0)`); `write()` never runs after it.
- `setStartupCommand(command)` arms a command that `start()` schedules once.
- `state.activate()` calls `module.setState(state)`; false only if the state class is unregistered or a guard rejected it.
- `speak` on `robot.telemetry` reaches the Driver Station only; the dashboard adapter ignores it so a combined telemetry still speaks once. `addAction`, `setAutoClear`, and `Func` producers all work.
- Both screens get the same telemetry: group headers and separators are identical HTML, data rows are plain text on the dashboard so its graph view still reads them as numbers, the braille field map is Driver Station only (the dashboard's HTML sanitizer has no `<pre>`), and `telemetry.log()` feeds the dashboard's log pane.

## Commands

Ivy is the only scheduler, and the framework owns it: `Scheduler.execute()` runs once per loop between `gameLoop()` and `writeModules()`, and `Scheduler.reset()` runs at init, `start()`, and `stop()`. Never call either yourself.

**Build commands in `initialize()`, schedule them in `onStart()`.** `start()` resets the scheduler, so anything scheduled during init is dropped. `StateCommands.set(...)` resolves each State's Module at build time and throws if it isn't bound yet, which is also why building must wait for `initialize()`.

**Requirements are the objects a command owns**: `Module` instances, or the `Follower`. Scheduling a command whose requirements collide with a running one of equal priority interrupts the running one (`OVERRIDE`). `StateCommands.set(...)` requires the named states' modules; every `PathCommands` command requires the follower, so a new path command interrupts the path in flight. Groups require the union of their children's requirements, so a whole-auto group holds every module it touches for its entire run, at priority 0: a `StateCommands.set(...)` scheduled from `gameLoop()` on one of those modules interrupts the **whole** auto, which does not resume. For a mid-auto reaction, build it into the sequence, or give the auto `.setPriority(1)` and the reaction `.setBlockedBehavior(BlockedBehavior.QUEUE)` so it runs once the auto is done.

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
2. **`pedro/Constants.java`** for every Pedro value: motor names and directions, Pinpoint offsets and directions, Foresight controllers. These are not dashboard sliders; tune them with AutoTune. Keep the robot still for the first second of init (Pinpoint IMU recalibration).
3. **`OptimizationToggles`** for telemetry/dashboard cadence and the profiler, and **`Robot.telemetryToggles`** (`dsTelemetry`, `dashboardTelemetry`, `voltage`, `current`, `loopProfile`, the switch for the per-section loop-time breakdown). `loopProfileTelemetryByDefault` is read once at class load; flip `telemetryToggles.loopProfile` at runtime instead.

**AutoTune:** deploy, join the robot wifi, open `http://192.168.43.1:10158` (websocket on 12649; over USB forward both and use `localhost`). Run **Mecanum**, **Pinpoint** (it measures directions and offsets itself), **Foresight**, then **Tests**, and paste each emitted block over the matching block in `Constants.java`. The Tests procedure ignores its Distance field and always drives 48 in (upstream bug), so leave it at 48 and clear a 48 × 48 in area. The values in `Constants.java` are the procedures' output from 2026-09-17; re-run them after any drivetrain, wheel, or odometry-pod change.

A `@Tuner` method in `Tuning.java` must be static, take no arguments, and be declared to return exactly `Procedure`; anything else breaks the RC app at boot.

## Testing

**Hub configuration names**: every framework OpMode needs `fl`, `bl`, `fr`, `br` (motors) and `pinpoint` (goBILDA Pinpoint), because `Robot` builds the follower unconditionally. Optional: `floodgate` (analog current sensor) and `nerdDetector` (webcam, only for the camera tests). Selecting the webcam-only `camera.xml` config on the DS breaks every framework OpMode.

- **Mock Architecture Test** (`opmodes/test/MockAuto`): the end-to-end smoke test. Run it first after `installDebug`. It doesn't move unless `Mock Auto → enableDrive` is on, which needs wheels off the ground and verified directions.
- **Camera Module Test** runs `modules/Camera` through the framework, so it also needs the drivetrain and Pinpoint wired up. **Ball Vision** (`opmodes/test/BallVisionTest`) runs `BallDetectionPipeline` directly against the webcam with a bare `LinearOpMode`, needing only `nerdDetector` — the one to run for a webcam-only bench check or to tune Pollen/Nectar HSV live against FtcDashboard's camera stream. **Pollen Detection** (`opmodes/test/PollenDetection`) is the same pattern for the EOCV-Sim-facing `PollenDetectionPipeline` copy. **BioBuzz Tele** is the robot-centric drive smoke test.
- For single-device bench tests use FTC Dashboard's Hardware View. For an isolated module test, extend `OpMode` directly rather than `EnhancedOpMode`.
- SDK sample OpModes are not in this repo; read them on GitHub at the tag matching `TeamCode/build.gradle`.

## Conventions

- `JavaVersion.VERSION_1_8` and `targetSdkVersion 28` are pinned by the SDK; `compileSdk 35`, `minSdk 24`, and `ndkVersion 21.3.6528147` are deliberate. `versionCode`/`versionName` track the SDK version. Don't change any without a reason.
- `libs/ftc.debug.keystore` is the FTC standard keystore. `TeamCode/lib/OpModeAnnotationProcessor.jar` is not wired into the build; add `annotationProcessor files('lib/OpModeAnnotationProcessor.jar')` if the compile-time OpMode checks are wanted.
- **Comments: default to none.** Add one only for a WHY a competent reader would otherwise get wrong: an ordering invariant, a hardware or SDK quirk, a deliberate deviation. Never restate code, add banners, changelog notes, or commented-out code.
- **Vendored code** (`architecture/prism/*`, `architecture/hardware/LaserRangefinder.java`, `pedro/procedures/*`, `robotcontroller/internal/*`) is never hand-edited. Refresh Prism with `scripts/update-prism.sh` (pin in `prism/PRISM_REV`) and the Quickstart procedures with `scripts/update-pedro-quickstart.sh` (pin in `procedures/QUICKSTART_REV`), each as its own commit; if the Quickstart bumped Pedro versions, bump `TeamCode/build.gradle` in the same commit.
- **Fail fast.** Let exceptions propagate; no try/catch to keep a loop alive. Numeric guards and try/finally are fine. Exception: `OpenCVPipelines/WebcamControls.java` deliberately ignores unsupported UVC control calls.
- OpMode `group` is `"Test"` for test OpModes and `"A"` for the competition teleop; the DS treats group names as case-sensitive.
- Commit messages are a single imperative subject line, no body, no trailer. Ask before pushing.
