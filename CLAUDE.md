# CLAUDE.md

## Project context

FTC team 6165 MSET Cuttlefish's 2026–27 **BIOBUZZ™** season repository. The game elements are **Pollen** (~2.8" yellow balls) and **Nectar** (~3.6" red and blue balls).

Game-specific code goes under `biobuzz/`, `modules/` and `opmodes/`; nothing game-specific goes under `architecture/`, which stays portable. Field constants come from the game manual, not guesses.

There is no BioBuzz robot yet: all BioBuzz testing runs on the **Betta bot**, a sister team's DECODE bot, with `pedro/BettaConstants`. `decode/` is last season's code for 6165's **Cuttle bot** only (`cuttle_decode.xml`), driving with `decode/CuttleDrive` on last season's wiring and never following a path; it shares `modules/Drivetrain`, so a Drivetrain change reaches Decode Tele, and nothing else depends on it. Automated ball detection and pathing are Autonomous-only and keep the robot on its own alliance's half.

The top level holds only the Gradle and repo files, `TeamCode/`, `limelight/` (runs on the Limelight), `scripts/` and `docs/`; new Java files go in an existing package. `docs/paths/` holds each auto's path as a visualizer.pedropathing.com `.pp` file named after its OpMode; re-export it when the auto's poses change.

## Keep this file fresh

Claude maintains this file unasked: a change to how the project is built, deployed, structured, tuned or tested, or to a convention, updates it in the same commit, and discrepancies get fixed. It says what is true now: what a thing is, where it lives, how to use it, its footguns. No history, nothing the code already says, no value lists.

## Build & deploy

Deploy from the IDE's run configurations (set up per laptop as in Sloth's README; `.idea/` isn't committed):

- **TeamCode** (the Run button) is the full install, with `:TeamCode:removeSlothRemote` as its first before-launch task. Use it for the first deploy, after wiping the hub or bumping Sloth, and after changing `robotcontroller/internal/`, dependencies, the manifest or resources.
- **deploySloth** hot-reloads `org.firstinspires.ftc.teamcode` in seconds; use it for everything else. Its finishing means the push finished, not the load: wait for the load (RC screen or DS log) before deploying again or pressing INIT.
- With no device attached, the Sloth tasks connect adb to `192.168.43.1`, then run a bare `adb disconnect` that drops every network adb device and `adb forward`. For USB or another address, add `load { address = "..."; autoconnect = dev.frozenmilk.sinister.sloth.AutoConnect.NEVER }` to `TeamCode/build.gradle`.
- If an old hot-load keeps replacing new code, run `removeSlothRemote` on its own (Gradle tool window, `TeamCode` → Tasks → install).
- **The Driver Station must match the SDK version** (an older DS fails the inspection screen): install `FtcDriverStation-release.apk` from the matching `FtcRobotController` release.
- **JDK 17** must be installed (`gradle/gradle-daemon-jvm.properties` pins the daemon to it), and Android Studio must be Narwhal 3 Feature Drop or newer.
- Pure Java, no unit tests. Claude checks its work with `./gradlew :TeamCode:compileDebugJavaWithJavac`, or `:TeamCode:assembleDebug` for the full APK.

## Dependencies

Versions live in `TeamCode/build.gradle` and the root `build.gradle`.

**Single-module app.** The SDK comes from Maven Central as `org.firstinspires.ftc:*`; there is no `FtcRobotController` module, and the template's activity classes live in `robotcontroller/internal/`. `abiFilters` is `arm64-v8a` only (Control Hub).

**Sloth and slothboard move together**: Sloth, the root `dev.frozenmilk.sinister.sloth.load` plugin and the slothboard version's `{sloth}` prefix must match, because slothboard pins Sloth strictly. slothboard comes from the committed `TeamCode/libs/m2/`, built from the same-named tag on the team fork `6165-MSET-Cuttlefish/slothboard`: Dairy's tag, acme's `master`, and only the team's open PRs to `acmerobotics/ftc-dashboard` (a closed PR is dropped at the next build). To bump it, tag a new build there and replace the six files and the version string. Keep `exclude group: 'com.acmerobotics.dashboard'`. Dashboard: port 8080; close its tab when timing loops (it reads every hub's voltage each second).

**Pedro Pathing 3**, with Ivy, its command framework: `com.pedropathing.math.Pose` is immutable (`x()/y()/heading()`, heading in [0, 2π)); paths come from `com.pedropathing.api.Paths` and must have a heading interpolator or they throw; per-path overrides are `Modifier`s (`ConfigVar.at(...)` via `Path.with(...)`). `BezierCurve.length()` (so also `parameter()` and `remainingDistance()`) can under-report S-curves and loops down to the chord: never use it for timeouts or scoring. `PathCommands.followUntilRemaining` relies on it. `follow` and `followThrough` brake at their path's end; legs keep their speed through a join as segments of one `Paths.path(...)`, or across commands when `followUntilRemaining` hands over before braking starts, and a heading jump over 11.25° at a join gives turning priority over driving. `architecture/auto/PathHeadings` gives the headings that avoid one (`tangentAt`, `startTangent`, `endTangent`, `jump`). `Interpolator.piecewise()` has two faults in 3.0.1: a piece gets t rescaled to 0..1, so `Interpolator.tangent` and `facingPoint` there replay the whole curve, and set on a `Paths.path(a, b, ...)` it can throw when the follower starts the path. Use `PathHeadings.linearThenTangent`/`holdThenTangent`/`tangentThenLinear`, or keep a piecewise heading of constant and linear pieces on a single curve.

## Framework layout

Framework code lives in `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/architecture/`: `core/` (`EnhancedOpMode`, `Robot`, `Module`, `State`, `Faults`, `Context`), `command/` (Ivy factories: `StateCommands`, `PathCommands`), `auto/` (field geometry, dashboard drawing, ball routing), and `control/`, `hardware/`, `input/`, `telemetry/`, `prism/` (vendored) and `OptimizationToggles`. A `Robot` subclass builds the follower in `createFollower(HardwareMap)`, then mechanisms in `initializeGameModules()`.

**Poses and alliance.** Author every pose for RED: `FieldPose.forAlliance(x, y, heading)` maps it to BLUE through `FieldConfig.symmetry()` when `Context.allianceColor` is BLUE; a bare radian passed to a path isn't mapped. The `FieldSymmetry` has no default: `EnhancedOpMode` clears it at every INIT, the game's Robot sets it in its constructor with `FieldConfig.setSymmetry(...)`, a bare `LinearOpMode` sets it itself, and `FieldPose` throws until it is set. `Robot` resets the follower pose at every init, so an auto sets its start with `robot.follower.setPose(...)` in `initialize()` before building paths. The static `Context.allianceColor` is set on the dashboard or in `Context.java`, persists until an app restart or Sloth reload, and no OpMode sets it.

**BIOBUZZ field.** `BioBuzzField.SYMMETRY` is `ROTATE_180` (manual TU02); `BioBuzzRobot`, `BallCollectionRobot` and `CloseFlowerLinearAuto` set it. `BioBuzzField.ownHalf(m)` is this alliance's half, `m` in from the walls and centre line, and the ball planner keeps the whole footprint inside it (G402: columns A–C are red's in AUTO): RED x in [m, W/2 − m], BLUE x in [W/2 + m, W − m].

**Ball routing.** The planner drives the intake over the balls, not the robot's centre. `RobotShape` is the frame (length along the heading, width) and the intake (width, and how far ahead of the centre of rotation its middle is); the footprint it keeps clear is the rectangle around both, so an intake may stick out past the frame. `biobuzz/RobotGeometry` (`@Config("Robot Geometry")`) holds the Betta bot's, the test robot until the BIOBUZZ robot exists: 16 in long, 15 in wide, the intake 15 in wide on the front face, 8 in ahead. `RouteOptimizer.findOptimalRoute(...)` tries every visit order of up to `MAX_BALLS` (6) balls and keeps the cheapest: the sampled drive length plus 10 in per radian of turning in place. Each leg aims the intake at the next ball from the previous one, with the robot's centre `intakeOffsetIn` behind it, preferring an approach it can line up on straight from at least 6 in back; where the middle of the intake can't reach a ball (corners, beside the rails) it aims further across, keeping the ball's edge 1 in inside the intake's. It tries a cubic (several handle lengths), then turning in place first, then a detour: straight out (forward, backing up or strafing) to room to turn, through the visibility graph built for the turning circle, then straight in; runs of graph corners become one curve wherever the robot could turn on the spot all along it, so it doesn't stop at each. Cubic handles stay under the chord's length, since a straight cubic with full handles stops dead halfway and Pedro's follower throws there. A leg is used only if the whole footprint, at the heading it actually has, stays inside the area and more than `gapIn` from every `Obstacle` at every sample, turns no tighter than a 6 in radius (`Segment.MIN_TURN_RADIUS_IN`), and brings its ball into the intake within about 37 degrees of head-on. The only obstacles are `BioBuzzField.hiveRails()`: the HIVE frame's two ground rails, from FIRST's field CAD, the same for both alliances; everything else under the HIVE is drivable, though the 18 in robot can't turn there. Every planner user passes them. A ball no heading reaches, one under the robot at the start, or one no drivable order includes goes to `Route.dropped` with a reason; `RouteOptimizer.ballProblem` is the reach check alone, for pre-filtering. Bad input throws, so check a robot-sourced start with `RouteOptimizer.poseProblem` first. It does no alliance mapping. `Plan.steps` cuts the drive wherever the heading or the direction of travel jumps, because Pedro moves on to the next piece of a path early: `RouteRun` drives each piece to a settled stop and turns in place between them (a hold until the heading is within 2 degrees). A drive step gets `clamp(length / minAvgSpeedIps, minLegSec, maxLegSec)` and a turn `minLegSec`; a step running out of time, or `abort(reason)` from OpMode code (never inside a command), ends the route with the follower stopped (`abortReason()` says why). Use `start()` or `command()` in a group, never both.

**Faults** (`core/Faults`) are named, recoverable errors, used only for the Limelight (see Conventions). `Faults.raise(key, message)`, `clear`, `has`, `any` and `active` are thread-safe; Modules and the OpMode have `raiseFault`/`clearFault`/`hasFault`, the OpMode also `activeFaults()`. Keys are global. `EnhancedOpMode` resets faults at every INIT and renders them first: a big red DS line each, a plain-text `FAULT <key>` row atop the dashboard, and red overlay text. A fault raised mid-loop reaches telemetry lines next loop; faults are hidden while `telemetryToggles.dsTelemetry`/`dashboardTelemetry` is off, and a bare `OpMode`/`LinearOpMode` doesn't render them.

**Game code.** `BioBuzzRobot` builds the follower, `drivetrain`, `cellTip` and `camera` and sets the symmetry; `BioBuzzOpMode` gives a typed `robot`; `RobotActions` (`robot.actions`) holds the Ivy commands. Among BioBuzz OpModes, only Ball Collection Auto leaves the Camera enabled, so only it polls the Limelight or can show its fault.

**Drivetrain** needs `withFollower(...)` for field-centric drive, heading lock and `isBonk()`; field-centric BLUE rotates the stick by π. Its `write()` throws while the follower isn't idle (Pedro drives the same motors), so an OpMode that follows paths on a Robot with a Drivetrain calls `robot.drivetrain.setWriteEnabled(false)`. Encoder and current telemetry cost hub reads and are off by default.

**Ball collection.** `RobotActions.collectBalls(settings)` returns a `BallCollection` command requiring the follower and the Drivetrain; build it in `initialize()`, schedule it in `onStart()` inside a timeout. On start it waits up to `visionWaitMs` for a fresh frame, plans the intake over the nearest `maxBalls` visible balls of the enabled types it can reach on `ownHalf(wallGapIn)` within `maxRangeIn`, the footprint at least `wallGapIn` from the walls and centre line and `railGapIn` from the HIVE rails, drives there and back with `RouteRun`, and disables Drivetrain writes meanwhile. With `onField` off (for testing away from a field, with the robot's pose started at (0, 0, 0)) there are no walls, centre line or HIVE rails and the footprint stays within `BallCollection.OFF_FIELD_REACH_IN` (144 in) of (0, 0); `BallCollection.keepIn`/`obstacles`/`configuredPose` give the same choice to OpModes. `status()` ends DONE, ABORTED (leg timeout or interrupted) or SKIPPED (Limelight fault at start, no fresh vision, which includes a START while the INIT sync is still uploading, or a start where the footprint doesn't fit), with `detail()`; a route already driving finishes despite a fault. `skipped()` and `route().dropped` explain every unused ball.

**Vision** (`modules/vision/`). `modules/Camera` reads the Limelight's Pollen and red/blue Nectar detections through `LimelightBallSource`. `TrackedBall` is camera-relative, so a stationary ball reads as moving when the robot moves; `FieldBall` is field-relative, exists only after `Camera.withFollower(...)`, and keeps its ID when the robot turns away and back.

- A frame is stale (`Camera.isFrameStale()`) with no new frame for `LimelightBalls → staleFrameSeconds`, during a Limelight fault, before the first frame, and while Camera is DISABLED. The tracker then resets, and `getBalls()` and the camera-relative getters are empty.
- A field ball out of view, and every one while vision is stale, stays in `getFieldBalls()` and the nearest/fastest/moving getters at its last position with `visible()` false for `forgetAfterSeconds`: check `visible()` before chasing one.
- `Camera.getFrameAgeSeconds()` is infinite before the first frame and while DISABLED (the `getPredicted*Position` getters then throw), and stays finite after a fault or reboot.

**Ball detection** is `limelight/ball_contour_snapscript.py` on pipeline 4 (`LimelightBalls → pipeline`) with the settings in `limelight/ball_pipeline.vpr` (the web UI's pipeline download); its `llpython` layout is in its docstring. The build bakes both files into the generated `LimelightFiles`, so deploySloth carries edits, and swaps the uploaded script's `SCRIPT_ID` for a stamp of both. After the first poll of every INIT (about 2 s), `LimelightSync` parks the Limelight on another pipeline for 1.5 s, uploads both, switches back and waits for the stamp, up to 3 times; the Limelight loads a script only when it switches to its pipeline. With `syncOnInit` off it only switches, and the file's `SCRIPT_ID` or any stamp is accepted too. It drops contacts at or above the horizon or beyond `MAX_RANGE_IN`. SnapScripts get the raw image, so the script undistorts with its own `LENS_*`: paste `eocvsim/homography`'s whole Python block (`UNDISTORT`, `CALIBRATION_SIZE`, `H_ARRAY`) into it, and a recalibrated lens means updating `LENS_*` in both files.

**Limelight faults.** `LimelightBallSource` never throws on a Limelight problem (the hardware lookup aside): it raises the `Limelight` fault, the frame goes stale at once, and recovery clears it. Problems include no data for `noDataFaultMs`, the wrong pipeline for `pipelineGraceMs`, bad script output, a web-editor copy while `syncOnInit` is on, and a failed sync (rejected POSTs, a non-Python pipeline, no stamp after 3 uploads). Read it with `Camera.limelightProblem()` (null while healthy or DISABLED) or `Faults.has("Limelight")`. A DISABLED Camera clears the fault and never polls. The sync runs once per requested pipeline, after the first successful poll, with the frame stale and no sync fault meanwhile (no-data faults still apply); `stop()` cancels it. It isn't retried, so a failed sync, a reboot onto the default pipeline or a web-UI switch keeps the fault up until the next INIT or a `pipeline` change.

**Cell tip.** `modules/CellTipCamera` reports whether this alliance's HIVE cell is tipped (`isTipped()`) or scorable (`isScorable()`) from AprilTags on the `aprilTagDetector` webcam: the cluster right-side up is scorable, upside-down or out of frame tipped. `RobotActions.checkTip()` also finishes after `checkTipTimeoutMs`, so read `isTipped()` afterwards. Mount the webcam image-upright, facing the launch direction; an upright tag reads roll ≈ 0 on the stream overlay. Tag ids come from `Context.allianceColor` at module init, so set the alliance before INIT or in `createRobot()`, not `initialize()`. `CellTipCamera.stop()` is irreversible; only `EnhancedOpMode` calls it.

## OpMode lifecycle

`init()`: `State.clearModuleBindings()`, `Faults.reset()`, `Scheduler.reset()` and `FieldConfig.clearSymmetry()` (statics that outlive an OpMode, Ivy's even a Sloth reload); Lynx hubs to manual bulk caching; `createRobot()`; discover and init modules (`initStates()` binds State→Module and applies initial values, then `init()`); `initialize()`; a second pass for modules created in `initialize()`; snapshot the field map.

User hooks: `createRobot()` (required), `initialize()`, `initializeLoop()`, `onStart()`, `gameLoop()`, `onLoopStart()`, `onEnd()`, `shouldReadDuringInit()` (default true), `shouldWriteDuringInit()` (default false), `telemetry()` for DS/dashboard lines, and `dashboardOverlay(Canvas)`, the only way to draw on the field. Never call `telemetry.update()`: it closes the DS frame and later DS lines are dropped.

`init_loop()` runs the same pipeline with `initializeLoop()` in place of `gameLoop()`, reads gated on `shouldReadDuringInit()` and writes on `shouldWriteDuringInit()` plus a 500 ms grace. The follower and scheduler are not gated, so a follow/hold call or path command before `onStart()` drives the robot in INIT.

```
telemetry.setEnabled        // Robot.telemetryToggles
telemetry.beginLoop         // opens a DS frame if the SDK will send this loop
fault lines                 // every active fault first
clearBulkCaches
InputClock.advance
updateVoltageThrottled
onLoopStart
readModules                 // refreshTunables() then m.read()
robot.follower.update
poseHistory.record
gameLoop                    // initializeLoop during init
Scheduler.execute           // between user code and writes so command state lands in this write pass
writeModules                // if isWriteEnabled
updateTelemetry             // DS lines only in a DS frame
updateDashboard             // every loop: overlay, dashboardOverlay, fault text, one packet
recordLoopTime              // loop() only
```

**Telemetry cadence.** DS lines are built only when the SDK will send, every `OptimizationToggles.dsTransmissionIntervalMs` (250 ms). The dashboard gets a packet every loop, sent every `dashboardTransmissionIntervalMs` (kept low; re-applied each loop because slothboard resets it at INIT). Dashboard telemetry stays as fast as possible and the loop profile stays on: the team tunes on the dashboard.

`start()`: resets timers, caches and loop stats; `Scheduler.reset()`, dropping anything scheduled during init; schedules each Module's startup command; `onStart()`.

`stop()`: `Scheduler.reset()` (no `end()` hooks run), stop the follower and zero its motors, `module.stop()` on every module whose `initStates()` began, then `onEnd()`; each runs even if an earlier one throws, and the first exception is rethrown. If `init()`, `init_loop()`, `start()` or `loop()` throws, the SDK never calls `stop()`, so that hook runs the same pass (without `onEnd()`) before rethrowing. The SDK fail-safes the REV hubs; module `stop()`s cover the rest, like the webcam and Limelight.

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

- `setStates(...)` once per state class, `bindTunable` after it. Don't touch states in the constructor: bindings exist from `initStates()` on, and `init()` runs after that.
- Tunables are `public static` fields on the `@Config` module class, optionally in plain nested holders. A nested class that is itself `@Config` must be named (`@Config("Shooter")`): slothboard keys config classes by simple name, so bare nested `Tuning` classes collide and vanish.
- Telemetry goes in `onTelemetry()` via `log`/`logDS`/`logDashboard`, never in `read()`. DS writes outside a DS frame are dropped, so gate other DS-only work on `getTelemetry().isDSFrame()`. Dashboard rows stay plain text so the graph view can read them.
- `stop()` is abstract and must reach the hardware: use `EnhancedMotor.stop()`, since the write cache can drop `setPower(0)`. It can run mid-OpMode with `write()` resuming afterwards, so keep it reversible (`CellTipCamera` excepted).
- `EnhancedMotor`'s one-argument `setVelocity`/`getVelocity` are in ticks per second, not RPM; `withVoltageCompensation` scales only `setPower`.
- `setStartupCommand(command)` arms a command that `start()` schedules once.

## Commands

Ivy is the only scheduler and the framework runs it (see the lifecycle): never call `Scheduler.execute()` or `Scheduler.reset()` yourself.

**Build commands in `initialize()`, schedule them in `onStart()`.** `start()` resets the scheduler, dropping anything scheduled during init, and `StateCommands.set(...)` resolves each State's Module at build time, throwing if it isn't bound yet.

**Requirements are the objects a command owns**: `Module`s (`StateCommands.set` requires the named states' modules) or the `Follower` (every `PathCommands` command). A new command whose requirements collide with a running one of equal priority interrupts it, and a group requires the union of its children's, so a whole-auto group holds every module it touches for its entire run: a `StateCommands.set(...)` scheduled from `gameLoop()` on one of those modules kills the **whole** auto, which does not resume. For a mid-auto reaction, build it into the sequence, or give the auto `.setPriority(1)` and the reaction `.setBlockedBehavior(BlockedBehavior.QUEUE)`.

- `StateCommands.set(State...)`: instant, activates each state in order. `setLazy(module, supplier)` resolves the state at start.
- `PathCommands.follow(f, path)` finishes once the robot is at the end and Pedro's hold has settled; `followThrough(f, path)` finishes at the end without the hold; `followUntilRemaining(f, path, inches)` finishes `inches` short of the end of the whole path and leaves Pedro driving the tail, so the next command overlaps it. All three stop the follower when interrupted. Also `hold(f)` / `hold(f, pose)`, `stop(f)`, `timeout(command, ms)`, and `remainingBelow(f, inches)` for `waitUntil` triggers (current leg only).
- Compose with Ivy: `sequential`, `parallel` (all finish), `race` (first finishes, rest interrupted), `waitMs`, `waitUntil`, `instant`, `command.until(condition)`.

`end()` hooks do not run on OpMode stop, so a command is never the only thing putting hardware in a safe state. Wrap every auto in a timeout (`PathCommands.timeout`). `opmodes/test/MockAuto` is the reference auto; `opmodes/test/auto/CloseFlowerAuto` is the same pattern on real paths.

## Tuning

1. **Module `@Config` fields** with `bindTunable` for setpoints.
2. **Pedro constants** in `pedro/`, one file per robot. `BettaConstants` is the only one; `DecodeRobot` uses `decode/CuttleDrive` and borrows Betta's Foresight only because `Follower` needs one. Tune with AutoTune first. On the dashboard, Foresight gains apply live and the rest on the next INIT; edits are lost on reload, so copy kept values into the file. Keep the robot still for the first second of init while the Pinpoint calibrates its IMU.
3. **`OptimizationToggles`** for telemetry cost and the profiler; **`Robot.telemetryToggles.loopProfile`** for the per-section loop-time breakdown (`loopProfileTelemetryByDefault` is read only at class load).
4. **Ball colours and detection gates**: the constants at the top of the SnapScript (`BALL_TYPES` HSV bands, `MAX_RANGE_IN`, the horizon margin). `docs/hsv-tuning-prompt.md` derives starting colours from photos. Tune live in the Limelight web editor (`DISPLAY_MODE = "MASK"` for colours) with `LimelightBalls → syncOnInit` off, or the next INIT replaces the edits. Like every dashboard value it turns back on at every deploySloth or app restart, so copy the values back into the file first. Pipeline settings changed there go back by saving the web UI's pipeline download over `limelight/ball_pipeline.vpr`.
5. **Homography** (`eocvsim/homography` → `H_ARRAY`): print `docs/calibration-chessboard-3in-letter.pdf` and set `SQUARE_SIZE_INCHES` to the measured square. Lay it flat and square to the robot, long side away from the camera (sideways never locks); its frame is +X away, +Y left, (0,0) at the near edge's centre. It locks on the first frame showing the board and prints Java and Python blocks; check its 6 in grid with a tape measure. The committed `H_ARRAY`s predate this frame. Set `UNDISTORT` (default on, for the Limelight 3A) false for a camera without a lens calibration.
6. **`BallFieldTransform.Mount`** (`@Config("CameraMount")`): the committed values are the Betta bot's Limelight, worked out from the homography (which gives the camera's position relative to the crosshair) and the Limelight sitting 3.5 in ahead of the robot's centre; check them as follows. With the current homography in the SnapScript, start at `headingDeg` 0 and `mirrorY` false, and set `xIn`/`yIn` to the measured offset from the robot's pose reference (the Pinpoint offsets' origin) to where the overlay's (0,0) crosshair lands, +X forward, +Y left. Run **Camera Module Test** with the robot still, put a ball on the centerline at two measured distances and read the `robot` column: a constant Y error is `yIn`, a Y error growing with distance is `headingDeg`, a ball on the left reading negative Y means flip `mirrorY`, a constant X error is `xIn`, and an X error growing with distance means redo the homography. Then turn in place: a still ball's field position must stay put; a circle means `xIn`/`yIn` is wrong. Copy kept values into the file.
7. **Cell tip**: `CellTipDetection` and `CellTipCamera`; exposure is `WebcamControls`.

**AutoTune:** deploy, join the robot wifi, open `http://192.168.43.1:10158` (websocket on 12649; over USB forward both and use `localhost`). Run **Mecanum**, **Pinpoint**, **Foresight**, then **Tests**, copying each emitted block into the matching `*Settings` fields of `BettaConstants`. Tests always drives 48 in whatever its Distance field says, so leave it at 48 and clear a 48 × 48 in area. Re-run after any drivetrain, wheel or odometry-pod change.

A `@Tuner` method in `Tuning.java` must be static, take no arguments, and be declared to return exactly `Procedure`; anything else breaks the RC app at boot.

## Testing

**Hub configs.** Every framework OpMode needs `fl`, `bl`, `fr`, `br` and `pinpoint`; BioBuzz and Decode OpModes also `floodgate` (Analog Input); BioBuzz OpModes and **Camera Tune** the webcam `aprilTagDetector`; BioBuzz OpModes, **Camera Module Test**, **Ball Field Drive** and **Vision Ball Collection** `limelight`, an `EthernetDevice` whose `serialNumber` starts `EthernetOverUsb:` and which has an `ipAddress`. `res/xml/` ships `cuttle_decode.xml` (the Cuttle bot), `decode_betta.xml` (the Betta bot) and `camera.xml` (webcam only); a config from older copies names the webcam `ballDetector` or `nerdDetector` and must be re-activated or renamed.

- **Mock Architecture Test** (`MockAuto`): the end-to-end smoke test, first after a full install. It drives only with `Mock Auto → enableDrive` on (check motor and odometry directions first) and finishes only on the floor.
- **Close Flower Auto** (a `BioBuzzOpMode`, Camera DISABLED): the CloseFlower route through the framework, `PathCommands.follow` per leg, 30 s timeout, Drivetrain writes disabled, authored for RED and rotated for BLUE. Its `toAudienceWall` leg passes 5.1 in from the centre line, inside the 9 in the ball routes keep, so the robot's side crosses it (G402), and `toHive`'s footprint reaches 0.5 in past it. **Close Flower Linear Auto** is the bare-`LinearOpMode` baseline; its legs end at the parametric end without the hold. Geometry lives only in `CloseFlowerPaths`; re-export `docs/paths/CloseFlowerAuto.pp` after changing it.
- **Camera Module Test** runs `modules/Camera` through the framework (so it needs the drivetrain and Pinpoint) from pose (0,0,0), for the Mount calibration. For the detector alone, watch the Limelight's web UI stream.
- **Ball Field Drive** drives raw mecanum and shows Limelight balls relative to its start pose, stale frames and faults included; turn away and back to check a ball keeps its ID. **Pinpoint Drive Test** drives raw mecanum and shows the Pinpoint pose. Both configure from `BettaConstants`.
- **Cell Tip Test** (on `cuttle_decode.xml`) runs `RobotActions.checkTip` against the live verdict, overlay on the dashboard camera stream.
- **Ball Collection** and **Vision Ball Collection** (Betta bot) are standalone planner tests, independent of `BallCollection`: set balls (or take them live from vision) and the start pose on the dashboard before INIT. By default `onField` is off: the start is (0, 0, 0) wherever the robot stands, positions are relative to it (+x ahead, +y left), and no walls, centre line or HIVE rails apply. With `onField` on, poses are RED field coordinates and a start where the footprint doesn't fit on the own half clear of the rails throws at INIT. The overlay draws the centre's path in blue, the intake's in orange and the footprint where the intake reaches the last ball. While idle they re-plan; `run` to 1 drives exactly the plan shown (refused if the robot moved off it), `run` to 0 aborts. Vision Ball Collection never plans on stale vision or a Limelight fault.
- **Ball Collection Auto** (Betta bot) runs `RobotActions.collectBalls` raced against `timeoutMs`: set the start pose and `collection` settings before INIT (off the field by default, starting at (0, 0, 0); with `collection.onField` on, a RED start whose footprint doesn't fit throws at INIT); it shows the status, unused balls and the plan. Wait for its `Vision` line to read live before START: the Limelight sync takes about 2 s after INIT.
- **BioBuzz Tele** is the robot-centric drive smoke test; cell-tip detection runs while `BioBuzz Tele → cellTip` is set, live, INIT included.
- **Decode Tele**: during INIT only gamepad1's pose resets act, and an INIT press doesn't fire at START. It starts at a placeholder pose, so reset it before trusting the odometry-only turret aim.
- Single-device tests use FTC Dashboard's Hardware View; an isolated module test extends `OpMode`, not `EnhancedOpMode`. SDK samples: GitHub, at the tag matching `TeamCode/build.gradle`.

**Known issues** (unmeasured on a Control Hub, so not fixed yet):

- `WebcamSession.close()` closes synchronously in `stop()`, which may overrun the SDK's stop watchdog.
- INIT during a Sloth load may restart the RC app.
- Nothing recovers when the Limelight comes back on another pipeline.
- The route planner's cost on the hub, and its re-plan hysteresis.

## Conventions

- `JavaVersion.VERSION_1_8` and `targetSdkVersion 28` are pinned by the SDK; `compileSdk`, `minSdk` and `ndkVersion` are deliberate; `versionCode`/`versionName` track the SDK version. Don't change any without a reason.
- `TeamCode/libs/ftc.debug.keystore` is the FTC standard keystore. `TeamCode/libs/OpModeAnnotationProcessor.jar` is not wired into the build.
- **Comments: default to none**, in every language. Add one only for a WHY a competent reader would otherwise get wrong (an ordering invariant, a hardware or SDK quirk, a deliberate deviation), one line where possible. Never restate code, narrate design, or add banners, history or commented-out code; usage guidance belongs in this file.
- **Vendored code** (`architecture/prism/*`, `pedro/procedures/*`, `robotcontroller/internal/*`) is never hand-edited. Refresh Prism with `scripts/update-prism.sh` (pin in `prism/PRISM_REV`) and the Quickstart procedures with `scripts/update-pedro-quickstart.sh` (pin in `procedures/QUICKSTART_REV`), each as its own commit; if the Quickstart bumped Pedro versions, bump `TeamCode/build.gradle` in the same commit.
- **Fail fast.** Fail loudly and early instead of working around a problem: let exceptions propagate, no try/catch to keep a loop alive, no silent fallback when something is missing or wrong. Numeric guards and try/finally are fine. Two deliberate exceptions: `modules/vision/WebcamControls.java` ignores unsupported UVC control calls; and, by the user's decision, Limelight problems never throw: `LimelightBallSource` raises the loud `Limelight` fault, Limelight-dependent actions fall back while everything else keeps running, and the fault clears on recovery. Hardware lookups (`limelight` included), the webcam cell tip and framework invariants stay fail-fast.
- **Limelight rules.** `Limelight3A`'s POSTs (`pipelineSwitch`, `uploadPipeline`, `uploadPython`, `updatePythonInputs`, `updateRobotOrientation`) block the calling thread up to 15 s and return a silent `false` on failure, so never call them from the OpMode thread: use a background sender (one thread, a one-slot mailbox where the newest value wins), send only on change, rate-capped, and switch the pipeline once per OpMode; `LimelightSync` is that sender for the ball pipeline, and its park around each upload is the one extra switch. `LLResult.getPipelineType()` is always empty (use `getStatus()`), and llpython keeps the last script's output on every pipeline. The SDK swallows poll errors, so health comes from `isConnected()`/`getTimeSinceLastUpdate()` and the result's contents. `setPollRateHz` clamps to 1–250 Hz and is ignored while the poller runs, so call `stop()` first; the poller persists across OpModes at whatever rate the last one left.
- OpMode `group` is `"Test"` for test OpModes and `"A"` for the competition teleop; the DS treats group names as case-sensitive.
- Commit messages are a single imperative subject line, no body, no trailer. Ask before pushing.
