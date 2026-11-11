# Pedro Pathing findings

What Pedro Pathing 3.0.2 (`com.pedropathing:revhub` and `core` 3.0.2) and its AutoTune library
(`com.pedropathing:tuning`) do that their API doesn't show, beyond the Pedro rules in [`CLAUDE.md`](../CLAUDE.md).
Read it before building paths or headings by hand, changing the follower setup, or running or bumping AutoTune.

Java paths below are relative to `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`. "Read from source" means
the library sources, or `javap` on the jar where only bytecode ships, read on 2026-11-10. Nothing here was measured on
a hub.

## What to do

1. **Never let a NaN reach the follower.** Pedro drops a NaN or infinite drive power without a word and the motors
   keep their last power, so the robot drives on instead of stopping.
2. **Before the next Foresight run, re-copy the AutoTune procedures and move `tuning` to 1.0.3. Never use 1.0.2.**
   The vendored Foresight tuner has no runaway stop and passes a NaN result on to the robot. Even with the new copy,
   clear 5 ft behind and to the right of each braking start and keep a hand on DS STOP.
3. **For a turn of 180° or more in a set direction, don't use `linear`.** It turns the short way, and at 180° apart
   rounding picks the direction. Use `Interpolator.longLinear` for the long way, or split a half turn in two.
4. **Don't read `follower.isBusy() == false` as "heading arrived".** It goes false 100 ms into the end hold at the
   latest, and `PathCommands.follow` ends on it.
5. **Keep a hand-built Bezier's derivative away from zero.** Pedro can throw mid-drive where it reaches zero.
6. **Pedro's drive power isn't scaled to battery voltage**, and every Pinpoint localizer build blocks for 0.5 s.

## A NaN keeps the robot driving

- `CachedMotor.setPower` returns at once on a NaN or infinite power, so the motor keeps whatever it was last sent.
  `Mecanum` wraps all four drive motors in `CachedMotor`, and every wheel's power uses forward, strafe and turn, so a
  NaN in any one leaves all four wheels at their last power. Nothing in `core` checks for NaN on the way. (Read from
  source: `revhub` `CachedMotor.java:21-22`, `Mecanum.java:33-38, 49-64`; `core` bytecode.)
- Ways to get one: a NaN pose, a NaN tuned constant, a 0/0 in a heading lambda. AutoTune's Foresight tuner ran away
  this way: a NaN heading kP made every braking power NaN and the robot drove off (upstream's note on
  `ForesightTuner.measured`, Quickstart 4f33ef5).
- Check values with `Double.isFinite` before they go into a path, a heading or the follower, and throw if one isn't.

## Paths and headings

**Linear headings take the short way.** `Interpolator.linear(a, b)` and `path.linear(a, b)` normalise both headings
and turn by `Angle.error(a, b)`, which lies in [-180°, 180°): `linear(0°, 270°)` turns -90°.
`Interpolator.longLinear(a, b)` turns the other way, 360° minus the short turn; `Path` has no `longLinear`, so use
`path.heading(Interpolator.longLinear(a, b))`.

A half turn has no short way, and rounding decides it. At exactly 180° apart both `linear` and `longLinear` turn
-180° (clockwise), but headings converted from degrees often land a hair either side: of the 360 whole-degree pairs
180° apart, `linear` turned counterclockwise for 42 to 85, depending on the degree-to-radian conversion, and
`longLinear` flips with it. For a half turn in a set direction, chain two `linear` turns of under 180° each with
`Interpolator.piecewise().until(...)`. (Read from source: `core` `Interpolator.linear`, `longLinear`, `Angle.error`,
`Angle.turnDirection`, `PiecewiseInterpolator.until`; the whole-degree count simulated with the same arithmetic.)

`PathHeadings` (`architecture/auto/PathHeadings.java:49, 65`) and the planner's `Segment`
(`architecture/planner/Segment.java:42, 116`) also turn the short way, so they agree with Pedro.

**Compound paths can skip several short pieces in one update.** A compound path is one `Paths.path(a, b, ...)` of
several curves (pieces). [`CLAUDE.md`](../CLAUDE.md) (Ball routing) says `pathSkip` moves on to the next piece once
the robot would start braking. After that move, `Foresight.calculatePath` (Foresight is Pedro's path-following
algorithm) runs again within the same update, so several short pieces can go by in a single loop. It applies only to
the pieces of one compound path, never across separate `follow` calls, which is why `RouteRun` is safe.
`BettaConstants` keeps Pedro's defaults: `pathSkip` and `brakeAtEnd` on, `parametricTConstraint` 0.025,
`timeoutConstraint` 100 ms. (Read from source: `core` `Foresight.calculatePath`, `PathTracker`, `ForesightConfig`
constructor.)

**`isBusy()` turns false 100 ms into the end hold.** The end hold is the hold Pedro switches to when a path ends with
`holdEnd` on. `Foresight.calculateHold` clears its busy flag once the hold has run `timeoutConstraint` (100 ms), or
once heading, position and velocity are all within tolerance, whichever comes first, and `Follower.isBusy()` returns
that flag. So `PathCommands.follow`, which ends on `atParametricEnd() && !isBusy()`
(`architecture/command/PathCommands.java:32`), can end with a long end turn still going. When the next step needs the
heading, wait for it as `RouteRun` does: heading within 2° and turn rate under 15°/s
(`architecture/planner/RouteRun.java:28-30`). (Read from source: `core` `Foresight.calculateHold`, `timeoutCondition`,
`reset`, `Follower.isBusy`, `Follower.hold`.)

**A Bezier whose derivative reaches zero can throw mid-drive.** `Curve.tangent(t)` normalises the derivative, and
`Vector2D.normalized()` throws `IllegalArgumentException("Cannot normalize 0 vector")` below a length of 1e-9.
`Foresight` and `Follower` call it while following, so the OpMode crashes partway along the path, not when the path
is built.

- A handle is the line from an end point to its neighbouring control point. The classic case is a cubic with control
  points A, B, A, B, so each handle reaches all the way to the other end. Its derivative is proportional to
  (1 − 2t)², which is zero at t = 0.5. Coincident control points do the same at that end.
- For handles that point along the chord toward the other end, keeping each longer than zero and shorter than the
  chord is enough to avoid it (worked through from the cubic's derivative). Check any other hand-built shape, and any
  new curve generator, for points where the derivative vanishes.
- The planner's curves come from `BezierCurve.through`'s control points
  (`architecture/planner/IntakeCurvePlanner.java`) and lines. `Segment` refuses a zero-length line but doesn't
  check a curve's derivative.
- (Read from source: `core` `Curve.tangent`, `Vector2D.normalized`. Seen crashing a desktop run on Pedro 3.0.1,
  2026-10-02.)

## Follower setup

**Building a `PinpointLocalizer` always blocks for 0.5 s.** Its constructor calls `reset()`, which recalibrates the
IMU under the default `ResetMode.RECALIBRATE_IMU` and then runs `Thread.sleep(500)` whatever the mode, so
`ResetMode.NONE` skips the recalibration but not the wait.

- The sleep is most of the 585 ms localizer build measured in [`hub-bench-findings.md`](hub-bench-findings.md).
- `reset()` is public and sleeps on every call: never call it from a loop.
- The only way around it is to drive `GoBildaPinpointDriver` yourself, as `opmodes/test/BettaBareDrive.java:25-31`
  does.
- (Read from source: `revhub` `PinpointLocalizer.java:49, 95-107`, `PinpointConfig.java:25`.)

**Mecanum has no battery-voltage compensation.** `MecanumConfig` holds only motor names and directions,
`manualBrakeMode` and `powerThreshold` (0.01). `Mecanum.applyDrive` writes the computed powers straight to the
motors. Only `SwerveConfig` has `voltageCompensation` and `nominalVoltage`, and `core` has no voltage setting at all.
Our `EnhancedMotor.withVoltageCompensation` doesn't help, because Pedro drives its own motor objects. How much a low
battery changes path speed and end poses is unknown (below). (Read from source: `revhub` `MecanumConfig.java:10-23`,
`Mecanum.java:61-64`, `SwerveConfig.java:14-15`.)

## AutoTune

### Versions

`TeamCode/build.gradle` pins `com.pedropathing:tuning:1.0.1`. Maven Central's newest is 1.0.3 (metadata fetched
2026-11-10).

| version | use | what differs |
|---|---|---|
| 1.0.1 | current pin: replace | `TunerScanner.unload` deregisters every tuner whenever Sloth unloads a class |
| 1.0.2 | never | `TunerScanner.scan` calls `putIfAbsent(loader, new ArrayList<>()).add(...)`; `putIfAbsent` returns null on the first insert, so it throws `NullPointerException` while registering the first procedure |
| 1.0.3 | move to it | `computeIfAbsent` instead; `beforeUnload` removes only the reloaded class loader's tuners, and Sloth 0.3.2 calls it before a hot reload. The POM moves `revhub` from 3.0.0 to 3.0.1, and our 3.0.2 pin wins |

- The server classes (`WebSocketServer`, `Hooks`, `Procedure`, `TuningSession`, `WebServer`, `TuningOpMode`) are
  byte-identical in 1.0.1 and 1.0.3, so the stop behaviour below holds for both.
- **The 1.0.3 page gives up fast.** It must open its websocket within 500 ms, or it shows "The tuning server
  disconnected." at once, because its own close handler cancels the reconnecting socket's retries. 1.0.1 waited
  10 s. On weak robot Wi-Fi, reload the page.
- (Read from source: the 1.0.1, 1.0.2 and 1.0.3 AARs from
  <https://repo1.maven.org/maven2/com/pedropathing/tuning/>, with `javap` and the bundled page script, fetched
  2026-11-10.)

### Re-copy the procedures before the next Foresight run

The vendored procedures (`pedro/procedures/`, `QUICKSTART_REV`: Pedro's Quickstart repo at b75637b) predate
upstream's Foresight tuner fixes. Since then, Quickstart master (at 4f33ef5 when fetched on 2026-11-10) has changed
only `ForesightTuner.java` and its dependency file, which now pins `revhub` 3.0.2 and `tuning` 1.0.3
([compare](https://github.com/Pedro-Pathing/Quickstart/compare/b75637b...4f33ef5), fetched 2026-11-10).

- Run `scripts/update-pedro-quickstart.sh`, set `tuning` to 1.0.3 in `TeamCode/build.gradle` in the same commit
  (CLAUDE.md, Conventions), then do a full install. The script copies master's current head: if it prints a commit
  other than 4f33ef5, read what else changed before running it on the robot.
- What the new `ForesightTuner` adds (read from source, Quickstart 4f33ef5):
  - It stops with a message when a measured value is NaN, infinite, or not positive where it must be, and when a
    run-up calibration has fewer than 2 usable samples or a slope that isn't negative. The vendored copy checks only
    `linReg[1] == 0`, which lets a NaN through (`pedro/procedures/ForesightTuner.java:684, 1142, 1261`).
  - The braking steps stop braking once speed along the test's direction drops to 0.25 in/s, not total speed.
  - The two braking steps get a runaway guard (`OVERRUN` 24 in, `DRIFT` 48 in). The vendored copy has none.

### Space for Foresight braking (new copy)

The guard stops the robot once it is more than Distance + 24 in from the start along the test's direction, or more
than 48 in across it, either way. The steps drive ahead (forward braking) or left (strafe braking) and brake on the
way back at 12 in or 6 in from the start, but the guard is symmetric. With the default braking Distance of 36 in:

| step | a normal run reaches | the guard trips at |
|---|---|---|
| forward braking | about 46 in ahead (36 + about 9.5 in overshoot at 0.7 power) | 60 in ahead or behind, 48 in either side |
| strafe braking | about 48 in left (36 + about 12.5 in at full power) | 60 in left or right, 48 in ahead or behind |

Overshoots are calculated as braking distance = linear × v + quadratic × v², with `BettaConstants`' brake
coefficients and v at 0.7 of `maxAchievableForwardVelocity` (forward) or all of `maxAchievableStrafeVelocity`
(strafe) at the brake point. The on-screen prompts ask for Distance ahead (or to each side) plus unmeasured "tile
space" or "space" around the robot, but a runaway backwards (forward braking) or to the right (strafe braking) goes
5 ft before the guard trips: clear 5 ft behind and to the right as well.

The guard can't catch everything (read from source: Quickstart 4f33ef5 `ForesightTuner.java`; SDK 12.0.0
`GoBildaPinpointDriver` and `TimestampedI2cData`):

- **A Pinpoint that stops giving good data.** The SDK's driver keeps the previous value for a NaN or out-of-range
  read (the pose freezes), and a failed I2C read can read as zeros, putting the pose back at the test's start
  ([`sdk-hub-findings.md`](sdk-hub-findings.md)). Pedro's `PinpointLocalizer.update` doesn't check the status, so the
  guard never trips. Run Pinpoint Localizer Test right before Foresight.
- **The driving steps have no timeout.** With a bad pose a step drives on at its power until stopped: full power in
  the velocity and deceleration steps, which have no guard at all, and up to 0.7 forward or 1.0 strafe in the braking
  steps.

### Stopping a procedure

- **Stops:** DS STOP ends the running step's OpMode. The page header's E-STOP or closing the tab ends the session,
  and `Procedure.runOpMode` then calls `requestOpModeStop`.
- **Doesn't stop:** a laptop that leaves the Wi-Fi or sleeps without closing the connection. The server never pings,
  and `Hooks` starts it with a read timeout of `Integer.MAX_VALUE`, so it never notices. Tests' Hold, Line, Curve and
  Interpolation steps keep looping until DS STOP or Restart Robot. Until the old connection closes or the app
  restarts, a reloaded page only shows "The tuning server disconnected."; the server's reason, "A tuning client is
  already connected.", appears only in the browser console.
- (Read from source: `tuning` 1.0.1 `Hooks.java:28`, `Procedure.java:96-99`, `WebSocketServer.java:196`,
  `TuningSession`, the bundled page script; `pedro/procedures/Tests.java`.)

## Drawing paths in the visualizer

Unverified: in visualizer.pedropathing.com v1.5.0 (the version `docs/paths/CloseFlowerAuto.pp` records), the
playback animation drew mid-turn headings up to 39° off what `PathHeadings` follows on a turn-then-tangent leg, such
as Close Flower's `toFarFlower`. The ghost (onion) drawing matched within 2.5°. The animation appears to advance those
headings by curve parameter t, where `PathHeadings` turns by distance along the path. Judge headings from the ghost
drawing. This rests on one unrecorded comparison on 2026-10-04, and the live site (fetched 2026-11-10) now saves path
files as version 2.0.0, so it may have changed.

## Still unknown

- **Does `pathSkip` skip a short piece entirely at speed, cutting the corner?** Test: on the hub, follow a
  `Paths.path(...)` with a 3–6 in middle piece at full speed and log `follower.pathIndex()` and the pose every loop.
  If the index jumps past the short piece in one loop, it was skipped; the logged poses show whether the corner was
  cut.
- **How far do auto end poses and leg times move with battery charge?** Test: run Close Flower Auto three times at
  about 14 V and three times at about 12.5 V, and compare end poses and leg times. If the spread matters, wrap the
  drivetrain or tune at mid-charge.
- **Does the current visualizer still misdraw?** Test: load `docs/paths/CloseFlowerAuto.pp`, pause the playback
  mid-turn on `toFarFlower`, and compare the drawn heading with `PathHeadings`' at the same distance along the path.
- **AutoTune 1.0.3 and the 4f33ef5 procedures haven't run on a hub.** Test: the first Foresight run after the
  re-copy, with the space above clear and DS STOP in hand; then deploySloth and check the tuners still appear.
