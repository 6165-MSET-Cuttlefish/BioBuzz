# Shooting on the move: tuning guide

Everything needed to take **SOTM Test** from placeholders to scoring while driving. Do the steps in order: each one assumes the ones before it are right.

All SOTM tunables are in the dashboard group **Shot Tuning** (`biobuzz/ShotTuning.java`). Dashboard edits are lost on deploySloth or an app restart, so copy every value you keep into the source as you go.

## What must be true before you tune anything

| Thing | Where | Check |
|---|---|---|
| Hardware config has `PollenTurret`, `NectarTurretL`, `NectarTurretR`, `PollenShooter`, `NectarShooter`, `PollenHood`, `NectarHood`, `hardstop`, plus everything `BioBuzzRobot` uses | Robot config | SOTM Test INITs with no missing-device error |
| Each flywheel motor is on port 0 or 3 with its Through Bore on the same-numbered encoder port | Wiring | `CLAUDE.md`, "Flywheel RPM" |
| Alliance | Dashboard `Context.allianceColor`, before INIT | Telemetry and field view show the right side |
| Unit tests pass | `./gradlew :TeamCode:testDebugUnitTest` | 0 failures |

## Where to stand

A CELL's opening does not face straight up. It faces the audience wall or the rear wall, tilted 30° upward, and the turrets only reach the robot's rear half. So every shot is taken:

- from **beside** the HIVE (audience side for the CELL that starts up, rear side after a TIP),
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
| Y | SOTM on / off (off = aim as if standing still) |

Dashboard group **SOTM Test**: `startX`, `startY`, `startHeadingDeg` (RED coordinates; put the robot there before INIT), `pollenEnabled`, `nectarEnabled`, `aim` (`LEGACY` = the old `Turret` auto-aim), `cellByFieldHalf`, `flipOnCameraTip`.

**Which CELL it aims at.** With `cellByFieldHalf` on (the default), the turrets aim at the CELL facing the half of the field the robot is in: the audience-side CELL from the audience half, the rear one from the rear half. Within `cellSwitchBandIn` (3 in) of the midline it keeps its last choice. It does not know which CELL is up: if the one on your side is tipped down, drive to the other side. With it off, X chooses, and `flipOnCameraTip` flips the choice on each TIP the webcam sees.

Telemetry shows, per shooter: valid or the reason not, target CELL, virtual target and distance, time of flight, turret angle and rate feedforward, flywheel target and droop, hood, and where the ball crosses the opening.

## Step 1: odometry

SOTM is only as good as pose and velocity.

1. In **Pinpoint Localizer Test**, push the robot forward, then left: x, then y, must rise.
2. Check the velocity sign: in SOTM Test with the robot at heading 90°, drive forward. The virtual target must move **toward the audience** (opposite to travel). If it moves with the robot, the velocity sign is wrong: stop and fix that first.
3. Drive a lap and return to the start mark: the pose should come back within about an inch.

## Step 2: turrets (Turret Aim Test)

For each of `Turret.Tuning.nectar` and `pollen`:

1. `leftPosition`, `backPosition`, `rightPosition`: the servo positions that point the turret exactly robot-left, straight back and robot-right (dpad left / down / right).
2. Nectar only (two servos): `offset` so the two servos don't fight. Keep each end at least `offset` inside 0 to 1.
3. `x`, `y`: the pivot, forward and left of the robot's centre, in inches.
4. Press A (AUTOAIM) and drive around: the turret should stay on the dashboard goal.

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

At each table distance, film a shot in slow motion (240 fps). Count frames from the ball leaving the shooter to it crossing the opening; `tofS` = frames ÷ frame rate. Enter it in the same table point.

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

Now drive. Change one thing at a time, and test each with the motion that isolates it. "Ahead" means the ball lands past the CELL in the direction the robot is travelling (too little lead); "behind" is the opposite (too much).

| Order | Tunable | Test | Symptom → change |
|---|---|---|---|
| 1 | `kDragPerS` (per element) | Strafe past the CELL at steady speed, near and far | Near shots fine but far shots land **behind** travel (over-led) → raise (try 0.2 to 0.5). Leave 0 if lead is right at all ranges |
| 2 | `phaseDelayS` | Strafe at steady speed | Lands **ahead** of travel by the same amount at every range → raise; behind → lower |
| 3 | `servoLagS` (per shooter) | Spin in place, and strafe close in | Turret trails the target while turning → raise; overshoots or twitches → lower |
| 4 | `maxTurretRateDegPerS` | Spin faster and faster | Set just under the rate where the turret visibly falls behind |
| 5 | `alpha` | Start and stop sharply while shooting | Misses only while speeding up or braking → raise toward 0.5; twitchy with stick movement → lower |
| 6 | `flywheelLeadS`, `hoodLeadS` | Drive straight toward, then away from, the CELL | Short while backing away / long while closing → raise |
| 7 | `velocityFilterS` | Watch the turret at a steady drive | Jitters → raise in 0.01 steps (it adds lag, so use as little as possible) |
| 8 | `transferTimeS` | — | Leave 0. Only raise it if shots still land ahead of travel after 1 to 3 |

How to tell time-of-flight errors from delay errors: a wrong `tofS` or `kDragPerS` gives a miss that grows with **distance**; a wrong `phaseDelayS` or `servoLagS` gives a miss that grows with **speed** but is the same at every distance.

## Step 8: the gates

These decide when a shot is allowed. Tune them last, by watching the `Valid` line while trying shots that should and shouldn't be taken.

| Tunable | Meaning | Start at |
|---|---|---|
| `rimClearanceIn` | Extra room demanded between the ball and the opening's edge | 1; raise if edge hits are allowed through |
| `maxShotSpeedInPerS` | Fastest robot speed at which a shot is allowed | The fastest speed you scored reliably in Step 7 |
| `hiveBadZoneMarginIn` | Grows the no-shoot box around the HIVE frame | 6 |
| `angleToleranceDeg` | How close the turret must be | 2 |
| `rpmTolerance` | How close the flywheel must be | 75, or the droop covered (Step 6) |
| `readyDropoutS` | How long feeding continues through a brief not-ready | 0.1 |
| `minDistanceIn`, `maxDistanceIn` (per element) | Tighter range than the table, if the ends are unreliable | Table ends |
| `keepOuts` | Turret angle pairs where the two turrets or shots collide | Empty until the turrets are mounted; then sweep both by hand and enter any clash |

What each `Valid` reason means:

| Reason | Cause |
|---|---|
| `TOO_CLOSE`, `TOO_FAR` | Outside the table or the min/max distance |
| `TURRET_RANGE` | Target is outside the turret's travel: turn the robot's back to the HIVE |
| `TURRET_RATE` | Turret would have to turn faster than `maxTurretRateDegPerS` |
| `TOO_FAST` | Robot faster than `maxShotSpeedInPerS` |
| `BEHIND_OPENING` | Robot is on the wrong side of the opening, or between it and the HIVE's centre: move out to the side the CELL faces |
| `RISING` | Ball would still be climbing at the opening: needs a steeper, slower shot there |
| `MISSES_OPENING`, `NO_ENTRY` | Ball would hit the edge or miss: usually shooting from too far off to the side |
| `BAD_ZONE` | Under the HIVE |
| `KEEP_OUT` | Turrets in a configured clash |
| `NOT_CONVERGED`, `NOT_FINITE` | Table has a gap or bad value: check for empty or out-of-order points |

## Step 9: save it

Copy into `ShotTuning.java`: both tables, `kDragPerS`, every `pollenShooter` / `nectarShooter` field, and the top-level values. Copy turret positions into `Turret.Tuning` and flywheel tuning into `Shooter`. Then run the unit tests again.

## Not done yet

- **Feeding.** SOTM Test only opens the hardstop; the intake and transfer (`Intran`) are not driven.
- **Competition TeleOp and autos.** SOTM lives only in SOTM Test.
- **Automatic CELL switching in auto.** `TargetSelector.onTipVerdict()` exists and `flipOnCameraTip` exercises it, but no auto calls it.
- **Opening position.** Its distance from the HIVE pivot is from the manual's figures and still needs checking against the field CAD (`TODO: verify` in `BioBuzzField`).
