# CLAUDE.md

Guidance for Claude Code when working in this repository.

## Project context

The 2026–2027 competition-season repository for **FIRST Tech Challenge team 6165 MSET Cuttlefish**, for the **BIOBUZZ™** game (kickoff September 12, 2026). The game elements are **Pollen**, ~2.8" yellow plastic balls, and **Nectar**, ~3.6" red and blue plastic balls; `modules/vision/` detects and tracks them.

Game-specific code (field coordinates, scoring logic, mechanism modules) goes under `biobuzz/`, `modules/`, and `opmodes/`. Nothing game-specific goes under `architecture/`, which stays portable to the next season. Field constants come from the game manual, not guesses.

Upstream: Sloth comes from `github.com/Dairy-Foundation/Sloth` as an artifact on `https://repo.dairy.foundation/releases`; the dashboard is the team's own build of Dairy's `ftc-dashboard` fork plus the team's dashboard PRs (see Dependencies); Pedro Pathing and its Quickstart come from `github.com/Pedro-Pathing`.

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
- **Build JDK.** The Gradle daemon is pinned to JDK 25 in `gradle/gradle-daemon-jvm.properties`; any JDK 17 or newer works, but whatever is pinned must be installed (`/usr/libexec/java_home -V`). Gradle 9.1.0 and AGP 8.13.2 need Android Studio Narwhal 3 Feature Drop or newer.
- Pure Java, no Kotlin plugin. There is no test suite; this is a robot controller APK.

## Dependencies

**Single-module app.** The stock `FtcRobotController` module is gone; the SDK is consumed as `org.firstinspires.ftc:*:12.0.0` from Maven Central, and the template's `FtcRobotControllerActivity`, `PermissionValidatorWrapper`, and `FtcOpModeRegister` live under `TeamCode/src/main/java/org/firstinspires/ftc/robotcontroller/internal/`. The DS Utility menu is therefore empty; FTC Dashboard's Hardware View covers what `TestHardware` did. `abiFilters` is `arm64-v8a` only (Control Hub); don't add `armeabi-v7a`.

**Sloth + slothboard move together.** `dev.frozenmilk.sinister:Sloth:0.3.2` (`TeamCode/build.gradle`), the root plugin `dev.frozenmilk.sinister.sloth.load` `0.3.2`, and the dashboard's `{sloth}` prefix must all match, because slothboard pins its Sloth strictly. The dashboard is the team's own slothboard build, `com.acmerobotics.slothboard:dashboard:0.3.2+0.6.0-6165.2` (`{sloth}+{dashboard}-6165.{n}`), served from the committed `TeamCode/libs/m2/`: Dairy's `0.3.2+0.6.0` plus the team's features (keyboard shortcuts, pinned config categories, log view, graph colours and layering, ordered HTML telemetry, telemetry search filter, layout sharing, non-Mac undo/redo). Bumping it means replacing the six files under `TeamCode/libs/m2/` and the one version string; Claude holds the rebuild procedure, so ask for it first. Keep `configurations.configureEach { exclude group: 'com.acmerobotics.dashboard' }` so the upstream dashboard never reaches the classpath. Dashboard is on port 8080, websocket 8000.

**Pedro Pathing 3.0.1**: `com.pedropathing:revhub:3.0.1` (pulls `core`), `com.pedropathing:tuning:1.0.1` for AutoTune, and Ivy, Pedro's command framework, `com.pedropathing.ivy:pedro:1.1.1`. 3.0 rules: `com.pedropathing.math.Pose` is immutable (`x()/y()/heading()`, heading normalized to [0, 2π)); paths come from `com.pedropathing.api.Paths` and must have a heading interpolator or they throw; per-path overrides are `Modifier`s (`ConfigVar.at(...)` via `Path.with(...)`).

## Framework layout

Everything framework-level is under `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/architecture/`.

- `core/` — `EnhancedOpMode` (the base OpMode), `Robot` (game-agnostic base; subclasses build mechanisms in `initializeGameModules()` and the follower in `createFollower(HardwareMap)`, which runs first), `Module`, `State`, `AllianceColor`, `Context`.
- `command/` — `StateCommands`, Ivy commands that drive Module state machines.
- `auto/` — `FieldConfig.fieldWidthInches` (`@Config`, 141.5); `FieldPose.forAlliance(x, y, heading)`, which mirrors RED geometry to BLUE as `(width - x, y, π - heading)` when `Context.allianceColor` is BLUE (author every pose for RED; don't use Pedro's `PoseFactory.mirrorX`, which negates heading in 3.0.0); `FieldVisualization` (dashboard overlay); `PoseRing` (pose trail); `HeadingInterpolatorBuilder`; `PathCommands`, Ivy commands over a Pedro `Path`.
- `control/` (PID), `hardware/` (cached motor/servo wrappers, voltage, encoders, the Brushland rangefinder), `input/` (gamepad layering and edge detection), `telemetry/` (the DS+dashboard fan-out and the loop profiler), `prism/` (vendored goBILDA Prism LED driver), and `OptimizationToggles` (framework-wide `@Config` perf toggles).

`Robot`'s constructor sets the follower pose to a placeholder, `(72, fieldWidth - 10, 90°)`, on every init so nothing carries across a Sloth reload. An auto sets its real start with `robot.follower.setPose(FieldPose.forAlliance(...))` in `initialize()` before building paths.

Game code: `biobuzz/` (`BioBuzzRobot` builds the mechanism modules and the follower, `BioBuzzOpMode` is the base OpMode with a typed `robot`, `RobotActions` holds the game's Ivy commands and hangs off `robot.actions`), `modules/` (mechanisms; `Drivetrain` needs `withFollower(...)` for field-centric drive, heading lock, and `isBonk()`, and throws if those are used without one), `opmodes/tele/` and `opmodes/test/` (`opmodes/auto/` appears with the first auto). `pedro/` is laid out like the Pedro Quickstart: `Constants.java` (ours; every Pedro config and `create(HardwareMap)`), `Tuning.java` (ours; `@Tuner` registrations), `procedures/` (vendored verbatim; never hand-edit).

Vision detects both game elements — Pollen and red/blue Nectar — as `BallVisionConstants.BallType`; every `BallDetection`/`TrackedBall`/`FieldBall` carries its type, and `BallTracker` never matches a track to a detection of a different type. Vision reports in two frames. `TrackedBall` is camera-relative, rigidly attached to the robot, so a stationary ball reads as moving when the robot moves. `FieldBall` is field-relative with true velocity, produced by `BallFieldTransform` from the robot state at each frame's own capture time (the camera is tens of ms behind) including the `omega x r` term, and only once `Camera.withFollower(...)` has been called. Extrinsics are `@Config("CameraMount")` in `BallFieldTransform`. `FieldBallTracker` sits one layer above that transform (inside `Camera.updateFieldBalls()`) and re-labels each frame's `FieldBall`s with a persistent identity keyed on field position, not camera position, so a ball's ID survives the robot turning away and back — something `BallTracker` can't do on its own, since its tracks are camera-relative and go stale the moment the robot rotates. A known ball missing from the current frame keeps reporting its last position with `visible()` false (not coasted forward — the goal is remembering where a likely-stationary ball was, not extrapolating where it went) until `FieldBallTracking.forgetAfterSeconds` passes with no re-detection. It does no smoothing or transforms of its own and only runs when `Camera`'s guard against re-processing an unchanged camera frame lets a new one through, so the added cost is a handful-of-balls distance check, once per real camera frame. `BallVisionConstants` is the canonical home for every detection-recipe number (HSV bands per type, radius/Hough/ROI constants, the calibration board, and the shared `H_ARRAY` homography). Its live-tunable numbers are `@Config`-bound nested classes (`Detection`, plus one `PollenHsv`/`RedNectarHsv`/`BlueNectarHsv` per ball type, dashboard categories `BallVision_Detection`/`BallVision_Pollen`/`BallVision_RedNectar`/`BallVision_BlueNectar`) that `BallDetectionPipeline` reads directly every frame — no separate seed step, editing FtcDashboard takes effect immediately. Detection is shape-first — the HSV mask seeds ROIs and `HoughCircles` decides what's a ball — with one deliberate exception: Hough's accumulator is a hard threshold, so a ball near it blinks in and out frame to frame even with a steady mask, and a mask blob round enough to fill `Detection.maskCircleMinFill` of its own enclosing circle stands in for the frames Hough misses it. `BallDetectionPipeline`'s own `Tuning` (`@Config("BallVisionDisplay")`) only holds its display/behaviour toggles (`displayMode`, `drawVelocity`, `velocityArrowSeconds`), since those aren't shared with the sim copy; `OVERLAY` draws this frame's raw detections (what Hough found, jitter included — use it to judge detection itself) while `BOX` draws the tracked, smoothed balls with stable ids (what robot code consumes) — it was named plain `BallVision` before the split moved the detection knobs out, and got renamed so a dashboard with the old, wider category pinned or laid out doesn't keep reapplying that stale snapshot over new edits. `OpenCVPipelines/` is a separate EOCV-Sim-facing copy of the detector, currently exercised from EOCV-Sim only (no on-robot test opmode), with its own `PollenDetectionPipeline.java`; EOCV-Sim compiles that file in an isolated per-folder workspace (`eocvsim_workspace.json`) that can't resolve an import into the rest of TeamCode, so its constants are a hand-kept mirror of `BallVisionConstants` (its class javadoc says so) rather than a live reference — changing one requires pasting the same change into the other. `OpenCVPipelines/HomographyCalculationPipeline` is the only way to produce `H_ARRAY` now — `BallDetectionPipeline` has no live chessboard calibration of its own, just a homography fixed at construction from `BallVisionConstants.H_ARRAY`. The tool locks a homography from a chessboard and prints it as a paste-ready `double[][] H_ARRAY`, mapping full-resolution pixels directly to field inches, with a separate, composed transform only for its on-screen warped preview so the printed matrix is never in preview-pixel space by mistake. Corner detection is OpenCV's sector-based `findChessboardCornersSB`, not the classic `findChessboardCorners`, since it degrades gradually on a steep/low camera angle instead of failing the whole board the moment quad-linking can't close — it needs a plain white border around the printed board roughly as wide as one square, or it can detect worse than the classic one did.

`modules/LimelightCamera` is the AprilTag half of vision and shares nothing with the webcam `Camera` above: it owns the Limelight 3A (config name `limelight`) and answers one question — has the watched cell tipped over onto its four tags? Every part of that answer is computed on the Limelight, and so is every setting behind it. Each cell has its own Limelight pipeline holding its own copy of the SnapScript with that cell's four tag ids, the 0.25 s hold, the minimum tag area and require-seen all baked in, so the hub writes nothing over the link at any point: it calls `pipelineSwitch` once in `init()` for the pipeline named by `Context.cell`, then only reads `llpython` (tipped, visible count and mask, hidden seconds, seen-since-start, tag-set checksum, tags in frame, frame counter; the slot layout is declared as constants at the top of `LimelightCamera` and in the scripts' docstrings, and they have to be edited together). Timing the hidden window on the camera is the point of the split — a 40 Hz OpMode loop cannot time a 0.25 s window against a 90 fps camera without smearing it — and moving the tag ids there too is what lets the link go quiet.

Because the pipeline is chosen at init, `Context.cell` has to be set before the OpMode initializes (in `createRobot()`, like the alliance colour); changing it later does nothing. `Context.Cell` holds the four sets and each one's pipeline index (`RED_1` 30-33 on pipeline 0, `RED_2` 34-37 on 1, `BLUE_1` 38-41 on 2, `BLUE_2` 42-45 on 3). The script echoes the checksum of the ids it was built with, and the module discards any verdict whose checksum doesn't match `Context.cell` — that is what catches a pipeline index pointing at the wrong script, and it surfaces as a permanent "NO VERDICT" rather than a wrong answer. `RobotActions.checkTip()` is the auto-facing form, an Ivy command that finishes when the cell tips, and `LimelightCamera.isTipped()` the plain boolean; both read false whenever the answer is stale, absent or about another cell. The command carries its own timeout (`checkTipTimeoutMs`, 10 s; there is a `checkTip(timeoutMs)` overload) so it cannot stall an auto on a cell that never tips or a Limelight that never answers — which means finishing does not mean tipped, and a caller that cares must read `isTipped()` afterwards.

Each pipeline also draws its whole state onto the frame the Limelight's own web UI streams, which is the only telemetry a SnapScript has: every detected tag outlined and labelled with its id (green for one of this cell's, grey for a stranger), a row of the four watched ids marking which are present, a red **TIPPED** banner or an UPRIGHT/visible-count line, the hidden timer against the hold threshold, and the tags-in-frame, seen-since-start and frame counters. `DRAW_OVERLAY` turns the lot off. It is how you tell a cell that is genuinely hidden from a camera that simply cannot resolve the tags.

The four uploaded scripts are generated, not hand-kept: `limelight/cell_tip_snapscript.py` is the template and `scripts/generate-limelight-pipelines.py` stamps it out into `limelight/pipelines/pipeline{n}_{cell}.py`, filling in `TAG_IDS`, `CELL_NAME` and `PIPELINE_INDEX` from the cells it reads straight out of `Context.java` so the two cannot drift. Edit the template, re-run the generator, re-upload all four; never edit a file under `limelight/pipelines/`.

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
- Telemetry goes in `onTelemetry()` via `log`/`logDS`/`logDashboard` (both screens, DS only, dashboard only), never in `read()`. Headers and separators render identically on both screens; data rows stay plain text on the dashboard so its graph view can read them; the braille field map is DS-only; `telemetry.log()` feeds the dashboard's log pane; `speak` reaches the DS only.
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
2. **`pedro/Constants.java`** for every Pedro value: motor names and directions, Pinpoint offsets and directions, Foresight controllers. These are not dashboard sliders; tune them with AutoTune. Keep the robot still for the first second of init: the Pinpoint localizer recalibrates its IMU and blocks for 500 ms while it does.
3. **`OptimizationToggles`** for telemetry/dashboard cadence and the profiler, and **`Robot.telemetryToggles`** (`dsTelemetry`, `dashboardTelemetry`, `voltage`, `current`, `loopProfile`, the switch for the per-section loop-time breakdown). `loopProfileTelemetryByDefault` is read once at class load; flip `telemetryToggles.loopProfile` at runtime instead.
4. **`BallVisionConstants`'s per-type HSV classes** (`PollenHsv`/`RedNectarHsv`/`BlueNectarHsv`) for ball-color thresholds. `docs/hsv-tuning-prompt.md` is a reusable prompt for deriving new defaults from photos of the actual balls — paste its instructions plus photos to a vision-capable AI, then confirm the result live against the real webcam with `Ball Vision`'s `MASK` display mode before trusting it.
5. **`BallFieldTransform.Mount`** (`@Config("CameraMount")`) is the camera's mechanical mount on the robot — the only calibration standing between a homography-correct camera-relative detection and a true absolute field position, since the homography itself only maps pixels to inches in a frame rigidly bolted to the camera. `xIn`/`yIn` (robot-frame position of the camera frame's own origin) needs no robot pose at all: run `Ball Vision`, find where the pipeline's own drawn "(0,0)" crosshair lands physically, mark it, and tape-measure from it to the robot's own pose reference (wherever `pedro/Constants.java`'s Pinpoint offsets are measured from — they must agree). `mirrorY` and `headingDeg` need a robot pose source (`Ball Field Drive` or `Camera Module Test`) with the robot stationary at a known heading: place a ball off to the robot's true left and flip `mirrorY` if reported Y isn't positive, then place a ball on the robot's true centerline at distance and nudge `headingDeg` live until reported Y reads ~0. Verify against a few more measured points spread across the camera's field of view, not just the two calibration points.

6. **Cell-tip detection** is tuned in two places, and only one of them is live. On the hub, `LimelightCamera`'s `maxStalenessMs` and `checkTipTimeoutMs` are ordinary dashboard `@Config` fields. Everything the detection itself does — `TAG_IDS`, `HOLD_SECONDS`, `MIN_TAG_AREA_PX`, `REQUIRE_SEEN`, `DRAW_OVERLAY` — lives in `limelight/cell_tip_snapscript.py` and only changes by editing the template, re-running `scripts/generate-limelight-pipelines.py` and re-uploading; there is deliberately no way to push any of it at run time. Upload from the Limelight web UI at `http://limelight.local:5801`: select pipeline index *n*, Input tab, pipeline type Python, paste `limelight/pipelines/pipeline{n}_*.py`, save — all four, each at the index its filename names, matching `Context.Cell`.

**AutoTune:** deploy, join the robot wifi, open `http://192.168.43.1:10158` (websocket on 12649; over USB forward both and use `localhost`). Run **Mecanum**, **Pinpoint** (it measures directions and offsets itself), **Foresight**, then **Tests**, and paste each emitted block over the matching block in `Constants.java`. The Tests procedure ignores its Distance field and always drives 48 in (upstream bug), so leave it at 48 and clear a 48 × 48 in area. Re-run them after any drivetrain, wheel, or odometry-pod change.

A `@Tuner` method in `Tuning.java` must be static, take no arguments, and be declared to return exactly `Procedure`; anything else breaks the RC app at boot.

## Testing

**Hub configuration names**: every framework OpMode needs `fl`, `bl`, `fr`, `br` (motors) and `pinpoint` (goBILDA Pinpoint), because `Robot` builds the follower unconditionally. Optional: `floodgate` (analog current sensor), `nerdDetector` (webcam, only for the ball-vision tests) and `limelight` (Limelight 3A, only for cell-tip detection — `LimelightCamera` `tryGet`s it, so its absence degrades to "no verdict" rather than breaking unrelated OpModes). Three configs live in `res/xml/`: `biobuzz.xml` is the framework config — the four motors, the Pinpoint, `limelight` and `nerdDetector`, and nothing else — and is what every framework OpMode runs on; `camera.xml` carries only `nerdDetector` and `limelight`, so it suits the bare-`LinearOpMode` tests (**Ball Vision**) and breaks every framework OpMode; `cuttle_decode.xml` is last season's full robot. Ports and buses in `biobuzz.xml` are inherited from `cuttle_decode.xml` and must be re-checked against the actual wiring.

- **Mock Architecture Test** (`opmodes/test/MockAuto`): the end-to-end smoke test. Run it first after `installDebug`. It doesn't move unless `Mock Auto → enableDrive` is on, which needs wheels off the ground and verified directions.
- **Pedro Localization Test** (`opmodes/test/LocalizationTest`): AutoTune's Localization Test rebuilt on the framework so it reaches FtcDashboard — robot marker, pose trail and a start-pose ring on the field view, and pose, forward/lateral offset, path length and total heading in telemetry. Start pose, field-centric and drive power are `@Config("Localization Test")`; gamepad options re-zeros. Use it instead of AutoTune's copy, which is DS-only because `pedro/procedures/` is vendored.
- **Camera Module Test** runs `modules/Camera` through the framework, so it also needs the drivetrain and Pinpoint wired up. **Ball Vision** (`opmodes/test/BallVisionTest`) runs `BallDetectionPipeline` directly against the webcam with a bare `LinearOpMode`, needing only `nerdDetector` — the one to run for a webcam-only bench check or to tune Pollen/Nectar HSV live against FtcDashboard's camera stream; `OpenCVPipelines/PollenDetectionPipeline` has no on-robot test opmode of its own currently — it's exercised from EOCV-Sim. **Ball Field Drive** (`opmodes/test/BallFieldDriveTest`) drives on raw gamepad mecanum and reports ball field positions from a `GoBildaPinpointDriver` read directly — no `Follower`/`Robot`/`EnhancedOpMode`, only the vision subsystems plus the bare `fl`/`bl`/`fr`/`br`/`pinpoint`/`nerdDetector` hardware; its Pinpoint offsets/directions are copied from `pedro/Constants.localizerConfig` and must be kept in sync by hand, and its pose is relative to wherever the robot was at init, not a field-alliance start pose. It runs its own `FieldBallTracker` (mirroring `Camera`'s), so it's also the one to run to check that a ball's ID survives turning the robot away from it and back. **Cell Tip Test** (`opmodes/test/CellTipTest`) shows the selected pipeline's live verdict, visible count and hidden time and runs `RobotActions.checkTip` against it, so the command and its timeout are exercised too — the one to run after uploading or regenerating the scripts. Covering the tags should flip the verdict a quarter second later; uncovering any one of them should clear it at once; a permanent "NO VERDICT" with the Limelight connected means the wrong script is on that pipeline index. `Cell Tip Test → cell` writes `Context.cell` in `createRobot()`, so it only takes effect on the next init. Like **Camera Module Test** it extends `EnhancedOpMode` and holds its own typed `BioBuzzRobot` rather than going through `BioBuzzOpMode`, so it needs the drivetrain and Pinpoint in the config (`biobuzz.xml`) even though nothing in it drives. **BioBuzz Tele** is the robot-centric drive smoke test.
- For single-device bench tests use FTC Dashboard's Hardware View. For an isolated module test, extend `OpMode` directly rather than `EnhancedOpMode`.
- SDK sample OpModes are not in this repo; read them on GitHub at the tag matching `TeamCode/build.gradle`.

## Conventions

- `JavaVersion.VERSION_1_8` and `targetSdkVersion 28` are pinned by the SDK; `compileSdk 35`, `minSdk 24`, and `ndkVersion 21.3.6528147` are deliberate. `versionCode`/`versionName` track the SDK version. Don't change any without a reason.
- `libs/ftc.debug.keystore` is the FTC standard keystore. `TeamCode/lib/OpModeAnnotationProcessor.jar` is not wired into the build; add `annotationProcessor files('lib/OpModeAnnotationProcessor.jar')` if the compile-time OpMode checks are wanted.
- **Comments: default to none.** Add one only for a WHY a competent reader would otherwise get wrong: an ordering invariant, a hardware or SDK quirk, a deliberate deviation. Never restate code, add banners, changelog notes, or commented-out code.
- **Generated code** (`limelight/pipelines/*`) is never hand-edited; regenerate it with `scripts/generate-limelight-pipelines.py` after editing `limelight/cell_tip_snapscript.py`.
- **Vendored code** (`architecture/prism/*`, `architecture/hardware/LaserRangefinder.java`, `pedro/procedures/*`, `robotcontroller/internal/*`) is never hand-edited. Refresh Prism with `scripts/update-prism.sh` (pin in `prism/PRISM_REV`) and the Quickstart procedures with `scripts/update-pedro-quickstart.sh` (pin in `procedures/QUICKSTART_REV`), each as its own commit; if the Quickstart bumped Pedro versions, bump `TeamCode/build.gradle` in the same commit.
- **Fail fast.** Let exceptions propagate; no try/catch to keep a loop alive. Numeric guards and try/finally are fine. Exception: `OpenCVPipelines/WebcamControls.java` deliberately ignores unsupported UVC control calls.
- OpMode `group` is `"Test"` for test OpModes and `"A"` for the competition teleop; the DS treats group names as case-sensitive.
- Commit messages are a single imperative subject line, no body, no trailer. Ask before pushing.
- **Commits are authored as the person running the session**, never as Claude. Set `user.name`/`user.email` to that person's own git identity (on a fresh container `git config --global user.name`/`user.email` start out as Claude) and check `git log -1 --format='%an <%ae>'` before the first commit of a session.
