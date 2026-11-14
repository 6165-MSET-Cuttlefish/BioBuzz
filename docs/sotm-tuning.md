# Shooting on the move: tuning guide

Everything needed to take **SOTM Test** from placeholders to scoring while driving. Do the steps in order: each one assumes the ones before it are right.

All SOTM tunables are in the dashboard group **Shot Tuning** (`biobuzz/ShotTuning.java`). Dashboard edits are lost on deploySloth or an app restart, so copy every value you keep into the source as you go.

## What must be true before you tune anything

| Thing | Where | Check |
|---|---|---|
| Hardware config has `frontPollenTurret`, `backPollenTurret`, `nectarTurret`, `pollenFlywheel`, `nectarFlywheel`, `pollenHood`, `nectarHood`, `blocker`, plus everything `BioBuzzRobot` uses | Robot config | SOTM Test INITs with no missing-device error |
| Each flywheel motor is on port 0 or 3 with its Through Bore on the same-numbered encoder port | Wiring | `CLAUDE.md`, "Flywheel RPM" |
| Alliance | Dashboard `Context.allianceColor`, before INIT | Telemetry and field view show the right side |
| Unit tests pass | `./gradlew :TeamCode:testDebugUnitTest` | 0 failures |

## Where to stand

A CELL's opening does not face straight up. It faces the audience wall or the rear wall, tilted 30° upward, and the turrets only reach the robot's rear half. So every shot is taken:

- from **beside** the HIVE (for RED, the audience side for the CELL that starts up and the rear side after a TIP; for BLUE the other way round),
- with the robot's **back toward the HIVE**,
- and never from under the HIVE frame (the solver refuses it).

From straight in front of the HIVE the opening is edge-on and nothing will go in.

## SOTM Test controls

| Control | Does |
|---|---|
| Left stick / right stick | Drive (robot-centric) / turn |
| A / B | Flywheels on / off |
| Hold RT | Fire: opens the hardstop while every enabled shooter is ready |
| X | Switch the target CELL (`STARTS_UP` ↔ `STARTS_DOWN`); only with `cellByFieldHalf` off |
| Y | SOTM on / off (off = aim as if standing still). Off after an app restart or deploySloth (`sotmEnabled`); the telemetry SOTM line shows which |

Dashboard group **SOTM Test**: `startX`, `startY`, `startHeadingDeg` (RED coordinates; put the robot there before INIT), `pollenEnabled`, `nectarEnabled` (whether RT waits for that shooter to be ready; both shooters spin, aim and are fed either way; with both off, RT never fires), `aim` (`LEGACY` = the old `Turret` auto-aim, the shot solved as if standing still), `cellByFieldHalf`, `flipOnCameraTip`.

**Which CELL it aims at.** With `cellByFieldHalf` on (the default), the turrets aim at the CELL facing the half of the field the robot is in: the audience-side CELL from the audience half, the rear one from the rear half. Within **Shot Tuning** `cellSwitchBandIn` (3 in) of the midline it keeps its last choice. It does not know which CELL is up: if the one on your side is tipped down, drive to the other side. With it off, X chooses, and `flipOnCameraTip` flips the choice on each TIP the webcam sees.

Telemetry shows, per shooter: valid or the reason not, target CELL, virtual target and distance, time of flight, turret angle and rate feedforward, flywheel target and droop, hood, and where the ball crosses the opening.

## Step 1: odometry

SOTM is only as good as pose and velocity.

1. Only once the robot's position comes from the OctoQuad and its constants are measured (`CLAUDE.md`, Tuning): in **OctoQuad Localizer Test**, push the robot forward, then left: x, then y, must rise.
2. Check the velocity sign: in SOTM Test with the robot at heading 90°, press Y until telemetry shows SOTM ON, then drive forward. The virtual target must move **toward the audience** (opposite to travel). If it moves with the robot, the velocity sign is wrong: stop and fix that first.
3. Drive a lap and return to the start mark: the pose should come back within about an inch.

## Step 2: turrets (Turret Aim Test)

For each of `Turret.Tuning.nectar` and `pollen`:

1. `leftPosition`, `backPosition`, `rightPosition`: the servo positions that point the turret exactly robot-left, straight back and robot-right (dpad left / down / right).
2. Pollen only (two servos): `offset` so the two servos don't fight. Keep each end at least `offset` inside 0 to 1. Set nectar's `offset` to 0: it still clips the one-servo nectar turret ([`robots-findings.md`](robots-findings.md#known-problems-in-test-opmodes-and-modules)).
3. `x`, `y`: the pivot, forward and left of the robot's centre, in inches.
4. Press A (AUTOAIM) and push the robot around by hand (Turret Aim Test has no drive controls): the turret should stay on the dashboard goal.

Then copy the pivot into **Shot Tuning** for the same shooter: `pivotForwardIn` = `x`, `pivotLeftIn` = `y`. The two must match.

Also measure, per shooter, into `pollenShooter` / `nectarShooter`:

| Field | Meaning |
|---|---|
| `exitHeightIn` | Height of the ball above the TILES as it leaves the shooter |
| `exitLeftIn` | How far left of the turret's centre line the ball leaves (negative = right; 0 if centred) |
| `minAngleDeg`, `maxAngleDeg` | Turret travel in degrees from straight back; stay within ±90 |

## Step 3: flywheels (BioBuzz Shooter Test)

For each flywheel (`Shooter.pollenTuning`, `nectarTuning`):

1. Open loop (LB / RB): step `openLoopPower`, and copy each steady **Feedforward Point** from telemetry into a `feedforward` table point. Skip any marked saturated.
2. Closed loop (A / X): tune `kP`, then `kI`, until it recovers quickly after a shot without overshoot.

Do not go on until both flywheels hold a target within about ±30 RPM.

## Step 4: the shot table, standing still

Each element has its own table (`pollen.table`, `nectar.table`). A point is `distanceIn`, `rpm`, `hoodPosition`, `tofS`; distance 0 means unused. The solver interpolates between distances and will not shoot outside the table's range.

Use the trims as your knobs: `rpmTrim` and `hoodTrim` add straight onto the table's output.

For each of about 5 distances, from the closest you would ever shoot to the farthest:

1. Park, back to the HIVE. Read **Virtual Target → distance** from telemetry.
2. A (flywheels on). Adjust `rpmTrim` and `hoodTrim` until shots drop cleanly into the middle of the opening. Prefer a steeper, slower shot: the ball must be falling as it enters.
3. Read **Flywheel (RPM)** and **hood** from telemetry: those are the values *with* trim. Write them, with the distance, into a table point.
4. Set both trims back to 0 and confirm the shot still goes in from the table alone.

Repeat for the other element. Then do the same from a second angle (not just straight on) to check the table holds.

**Reading the entry check.** `Entry (in): u, w, vz` is where the solver thinks the ball crosses the opening: `u` is across (±10 at the edges), `w` is up the face (0 at the bottom edge, 14 at the top point), `vz` must be negative (falling). If real shots go in but the solver says `RISING` or `MISSES_OPENING`, the time of flight (next step) or `exitHeightIn` is wrong, not the shot.

## Step 5: time of flight

At each table distance, film a shot in slow motion (240 fps). Count frames from the ball leaving the shooter to it crossing the opening. The solver takes `tofS` as the time to the aim point, `aimDepthIn` (3 in) past the opening's centre inside the CELL, which the video can't show, so add the extra time: about 15 ms at 11 ft and 45 ms at 3 ft ([`shot-calculation-findings.md`](shot-calculation-findings.md#leading-a-moving-shot)). `tofS` = frames ÷ frame rate + that extra. Enter it in the same table point.

This is the number that decides how far the turret leads when moving, so measure it; don't guess.

## Step 6: droop points (hood follows flywheel speed)

When you fire quickly, the flywheel sags and the next ball falls short. The fix is extra table points: **same `distanceIn`, lower `rpm`, the `hoodPosition` that still scores at that speed.** The solver reads the measured flywheel speed and slides the hood between the points at each distance. (Their `tofS` is ignored.)

At each distance:

1. Set `rpmTrim` to about −100. Shots should now fall short.
2. Adjust `hoodTrim` until they score again.
3. Add a point: same distance, the lower RPM shown, the hood shown.
4. Optionally repeat at −200 for a second droop point.
5. Zero the trims.

Once droop points exist, widen that shooter's `rpmTolerance` to about the droop you covered (e.g. 100), so the gate lets those shots through. Without droop points the hood ignores flywheel speed, and `rpmTolerance` should stay tight (about 50 to 75).

## Step 7: moving shots

Press Y until telemetry shows SOTM ON, then drive. Change one thing at a time, and test each with the motion that isolates it. "Ahead" means the ball lands past the CELL in the direction the robot is travelling (too little lead); "behind" is the opposite (too much).

| Order | Tunable | Test | Symptom → change |
|---|---|---|---|
| 1 | `kDragPerS` (per element) | Strafe past the CELL at steady speed, near and far | Near shots fine but far shots land **behind** travel (over-led) → raise (try 0.2 to 0.5). Leave 0 if lead is right at all ranges |
| 2 | `phaseDelayS` | Strafe at steady speed | Lands **ahead** of travel by the same amount at every range → raise; behind → lower |
| 3 | `servoLagS` (per shooter) | Spin in place, and strafe close in | Turret trails the target while turning → raise; overshoots or twitches → lower |
| 4 | `maxTurretRateDegPerS` | Spin faster and faster | Set just under the rate where the turret visibly falls behind |
| 5 | `alpha` | Start and stop sharply while shooting | Misses only while speeding up or braking → raise toward 0.5; twitchy with stick movement → lower |
| 6 | `flywheelSpinUpLeadS`, `flywheelSpinDownLeadS`, `hoodLeadS` | Drive straight away from the CELL (spin-up), then toward it (spin-down) | Short while backing away → raise spin-up; long while closing → raise spin-down. Expect these to end up well above the aim lead, often more than the time of flight; spin-down usually more than spin-up |
| 7 | `velocityFilterS` | Watch the turret at a steady drive | Jitters → raise in 0.01 steps (it adds lag, so use as little as possible) |
| 8 | `minSotmSpeedInPerS` | Stand still, then creep | Turret twitches while parked → raise; creeping shots land ahead of travel → lower |
| 9 | `transferTimeS` | — | Leave 0. Only raise it if shots still land ahead of travel after 1 to 3 |

How to tell time-of-flight errors from delay errors: a wrong `tofS` or `kDragPerS` gives a miss that grows with **distance**; a wrong `phaseDelayS` or `servoLagS` gives a miss that grows with **speed** but is the same at every distance.

## Step 8: the gates

These decide when a shot is allowed. Tune them last, by watching the `Valid` line while trying shots that should and shouldn't be taken. The speed, flywheel and turret gates each have a known problem in the code: read [Known problems in the code](#known-problems-in-the-code) first.

| Tunable | Meaning | Start at |
|---|---|---|
| `rimClearanceIn` | Extra room demanded between the ball and the opening's edge | 1; raise if edge hits are allowed through |
| `maxShotSpeedInPerS` | Fastest robot speed at which a shot is allowed | The fastest speed you scored reliably in Step 7 |
| `hiveBadZoneMarginIn` | Grows the no-shoot box around the HIVE frame | 6 |
| `angleToleranceDeg` | How close the turret must be | 2 |
| `rpmTolerance` | How close the flywheel must be | 75, or the droop covered (Step 6) |
| `readyDropoutS` | How long feeding continues through a brief not-ready | 0.1 |
| `minDistanceIn`, `maxDistanceIn` (per element) | Tighter range than the table, if the ends are unreliable | Table ends |
| `keepOuts` | Turret angle pairs where the two turrets or shots collide | Empty until the turrets are mounted; then sweep both by hand and enter any clash in `ShotTuning.java` (the dashboard can't edit them: [`dashboard-findings.md`](dashboard-findings.md#the-rules-behind-it)) |

What each `Valid` reason means:

| Reason | Cause |
|---|---|
| `TOO_CLOSE`, `TOO_FAR` | Outside the table or the min/max distance |
| `TURRET_RANGE` | Target is outside the turret's travel: turn the robot's back to the HIVE |
| `TURRET_RATE` | Turret would have to turn faster than `maxTurretRateDegPerS` |
| `TOO_FAST` | Robot faster than `maxShotSpeedInPerS`; today only with SOTM on and `aim` SOTM ([Known problems in the code](#known-problems-in-the-code)) |
| `EXIT_OFFSET` | `exitLeftIn` is at least the distance to the target: check its value (Step 2) |
| `BEHIND_OPENING` | Robot is on the wrong side of the opening, or between it and the HIVE's centre: move out to the side the CELL faces |
| `RISING` | Ball would still be climbing at the opening: needs a steeper, slower shot there |
| `MISSES_OPENING`, `NO_ENTRY` | Ball would hit the edge or miss: usually shooting from too far off to the side |
| `BAD_ZONE` | Under the HIVE |
| `KEEP_OUT` | Turrets in a configured clash |
| `NOT_FINITE` | The element's table has no point with a distance above 0, or the pose or velocity isn't a number |
| `NOT_CONVERGED` | The virtual target didn't settle: time of flight rises too steeply with distance for the robot's speed toward or away from the CELL; check the table for a mistyped `tofS` |

## Step 9: save it

Copy into `ShotTuning.java`: both tables, `kDragPerS`, every `pollenShooter` / `nectarShooter` field, and the top-level values. Copy turret positions into `Turret.Tuning` and flywheel tuning into `Shooter`. Then run the unit tests again.

## Known problems in the code

None of these is fixed. Each was read from source on 2026-11-11 or 2026-11-12 (paths under `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`); the numbers are computed from the code and its default values. Several do nothing at today's defaults, as each says.

- **`TOO_FAST` never fires with SOTM off or `aim` set to LEGACY** (Step 8). The gate compares `maxShotSpeedInPerS` with the predicted velocity (`biobuzz/ShotSolver.java:150`), which `predict()` sets to zero while `sotmEnabled` is off (`:201-205`), and in LEGACY SOTM Test hands the solver zero velocity (`opmodes/test/SotmTest.java:127-130`). SOTM is off by default, so by default a shot aimed as if standing still is valid at any speed the turret and flywheel gates let through, and neither is a speed limit. Driving toward or away from the CELL hardly turns the turret, so such a shot gets through wherever the flywheel keeps within `rpmTolerance`. Across the line of sight the gate's turret estimate trails the command by at most the bearing rate × `servoLagS` (`opmodes/test/SotmTest.java:166-168`), under `angleToleranceDeg` 2 wherever the aim point is more than 1.43 in away per in/s of speed at the default 0.05 s. With the robot's centre 8 in from the walls, no point on the side a CELL faces is more than about 90 in from its aim point, so at the default 60 in/s limit a pass across the line of sight gets through mostly near a far corner (beyond 86 in at 60 in/s); with `maxShotSpeedInPerS` tuned to 30, a pass 80 in from the CELL at 40 in/s trails at most 1.4°. Holding RT fires these with no lead. `ShotTuning`'s Javadoc for `maxShotSpeedInPerS` says the gate always applies; `ShotSolverTest` checks it only with SOTM on. Until it's fixed, don't hold RT while driving with SOTM off or `aim` LEGACY. Proposed fix: gate on the measured speed taken before `predict()` zeroes it (whether LEGACY gets the gate too is the owner's call).
- **The Turret group's `cellSwitchBandIn` does nothing.** It belongs to `Turret`'s default goal (`biobuzz/modules/Turret.java:23`, `:96-101`), which never runs: SOTM Test and Turret Aim Test, the only OpModes that build a `Turret`, set their own goal in `initialize()` (`opmodes/test/SotmTest.java:66`, `opmodes/test/TurretAimTest.java:27`). SOTM Test's band is **Shot Tuning**'s `cellSwitchBandIn` (`opmodes/test/SotmTest.java:102`). Proposed fix: delete the default goal and its knob.
- **The flywheel gate compares the measured speed with the led target** (Steps 7 and 8). `r.rpm` is the table read `flywheelSpinUpLeadS` or `flywheelSpinDownLeadS` ahead (`biobuzz/ShotSolver.java:297`, `:442-452`), and SOTM Test's gate is `|measured − r.rpm| ≤ rpmTolerance` (`opmodes/test/SotmTest.java:171`), while its turret gate uses the un-led angle (`:170`). A flywheel at exactly the right speed for a ball fired now then reads off by table slope × distance rate × lead: on the placeholder tables (12.5 RPM per inch) at 30 in/s, a 0.2 s lead uses all 75 RPM of `rpmTolerance`, and leads as long as the time of flight (0.55 to 0.9 s on the placeholder tables), which Step 7 row 6 expects or more, give about 210 to 340 RPM. Once the leads are tuned, the gate stays shut whenever the distance closes or opens faster than `rpmTolerance` / (slope × lead): about 11 in/s with a 0.55 s lead on the placeholder tables. Nothing happens today: both leads are 0 and SOTM is off by default. Droop has the same mix-up ([`shot-calculation-findings.md`](shot-calculation-findings.md#our-solver)). Proposed fix: give the solver's `Result` the un-led table RPM plus `rpmTrim` and gate on that.
- **The turret gate doesn't know the turret stops at ±90°** (Step 8). The SOTM turret command is the solved angle plus `angleTrimDeg`, added after the solver's range check, plus the servo lead (`biobuzz/ShotSolver.java:158`, `:307-311`; `opmodes/test/SotmTest.java:143-145`). `Turret` clips it to ±90° (`biobuzz/modules/Turret.java:125-128`), but the gate's turret estimate follows the unclipped command (`opmodes/test/SotmTest.java:166-171`). When the command passes the end by more than `angleToleranceDeg`, the gate opens with the turret stopped short: solved 89° with a 5° trim, the turret stops at 90°, the estimate reaches 94°, and the shot leaves 4° off its trimmed aim. At today's defaults (SOTM off, `rateMethod` ANALYTIC, `angleTrimDeg` 0, `minAngleDeg`/`maxAngleDeg` ±90°) the rate feedforward is 0 (`biobuzz/ShotSolver.java:164`, `:201-205`, `:313-314`) and the command never passes the end. With SOTM on, or `rateMethod` NUMERICAL, the servo lead can, near ±90° while the turret turns. Until it's fixed, keep `angleTrimDeg` within `angleToleranceDeg` for shots near the ends of travel. Proposed fix: clip the command to `minAngleDeg`/`maxAngleDeg` for both the turret and the estimate, and mark a trimmed angle outside them `TURRET_RANGE`.
- **Changing `aim` after START puts the turret and the gate out of step.** SOTM Test sets the turret's state from `aim` only at START (`opmodes/test/SotmTest.java:95`) but reads `aim` every loop for the solver's velocity (`:127-132`). Switched from LEGACY to SOTM mid-run with SOTM on (Y), the solver leads and the gate's turret estimate follows the led command, while the turret stays in AUTOAIM with no lead (`biobuzz/modules/Turret.java:110-115`), so the gate opens on shots aimed with no lead. Nothing happens with SOTM off, the default, or in a run started with `aim` at its default, SOTM (`opmodes/test/SotmTest.java:34`). Until it's fixed, change `aim` only before START. Proposed fix: set the turret's state from `aim` every loop.
- **Driving diagonally or while turning overstates the commanded velocity, so leads come out long** (Step 7, `alpha`). SOTM Test takes each stick axis as reaching its full `maxAchievable*Velocity` on its own (`opmodes/test/SotmTest.java:111-113`, `:122-123`), but `Drivetrain.setTargets` scales all four wheels down whenever |forward| + |strafe| + |turn| passes 1 (`biobuzz/modules/Drivetrain.java:130-139`, `:228-233`). Full forward plus full right drives about (36.5, 30) in/s against a command of (73, 60) (`pedro/BioBuzzConstants.java:60-61`), so at `alpha` 0.2 the solver's velocity, and every lead, is 20% high; full forward plus full turn commands 73 in/s for about 36.5. One stick axis alone is right. Nothing happens with SOTM off, the default. Until it's fixed, tune `alpha` and the leads one axis at a time, and expect long leads on diagonals and while turning. Proposed fix: divide the command by the same sum the drivetrain divides by. To check a fix on the robot: with SOTM on, compare where shots land driving diagonally at full stick and driving straight at the same speed.
- **Every default of the tuning objects (`ShotTuning.pollen`, `nectar`, `pollenShooter` and `nectarShooter`; `Shooter.pollenTuning`, `nectarTuning` and `hardstopTuning`; `Turret.Tuning.nectar` and `pollen`) is written twice** (Step 9): once as the field's initializer and again, with the same value, in a `static` block that runs after the object is built and wins (`biobuzz/ShotTuning.java:29-67` against `:110-173`; `biobuzz/modules/Shooter.java:21-50` against `:56-103`; `biobuzz/modules/Turret.java:46-53` against `:25-43`). Only `ShotTuning`'s placeholder table rows (`:114-118`, `:126-130`) aren't copies. An edit to only an initializer does nothing; today both copies agree, so nothing is wrong yet. Until it's fixed, copy tuned values for those objects into the `static` block lines (`pollenShooter.exitHeightIn = ...`); top-level values such as `ShotTuning.alpha` and `Turret.Tuning.flipBandDeg` are written once, at their declaration. Proposed fix (the owner's call): one copy per value, keeping the initializers, which `ShotSolverTest` builds its tunings from.

## Not done yet

- **Feeding.** SOTM Test only opens the hardstop; the intake and transfer (`Intran`) are not driven.
- **Competition TeleOp and autos.** SOTM lives only in SOTM Test.
- **Automatic CELL switching in auto.** `TargetSelector.onTipVerdict()` exists and `flipOnCameraTip` exercises it, but no auto calls it. `TargetSelector.set(Cell)` and `ShotSolver.result(int)` have no caller yet.
