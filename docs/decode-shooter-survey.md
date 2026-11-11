# DECODE teams' shooter code

What 23 FTC teams' public DECODE-season (2025-26) code does to choose a shot and to control the flywheel. Read it for
evidence when changing our shooter; the conclusions drawn from it are in
[`flywheel-control-findings.md`](flywheel-control-findings.md) and
[`shot-calculation-findings.md`](shot-calculation-findings.md). The same 23 repositories' loop time, ball counting,
TeleOp automation and structure are in [`decode-code-survey.md`](decode-code-survey.md).

## What it shows

- Most teams pick flywheel speed from a table keyed by distance (13 of 23) and run feedforward plus P (15 of 23).
  Only one has an integral term that changes the output.
- Feeding seldom waits for the flywheel. Only one team checks speed before each ball; 13 slow the feed or space the
  balls at long range instead.
- Eight teams tried physics-based shot or hood calculation and don't run it. The two that compute shot speed from
  physics multiply it by a correction factor.
- None of this shows what worked: we found no shot-accuracy measurements in the repositories.

## How it was read

Read from source on 2026-11-10, twice and independently, tracing from each team's competition TeleOp and main Auto
into the shooter code they run. Test OpModes and unused classes were left out except where noted as history. Each
row describes the commit named in it, and several of those commits were made after the team's last event. The table
and counts follow the second read. The two reads agreed on how speed is chosen and on battery compensation for every
team, and on the flywheel law for all but one; they classed a few hood and moving-shot rows differently, and those
rows and the flywheel-law question were settled by reading the code again.

## Per team

Terms used in the table:

- **FF + P**: feedforward (kV times the target, often plus a constant kS) plus a proportional term, with no integral
  that changes the output. **Full-power band**: full power while the wheel is more than a set margin below target,
  feedforward otherwise. **Bang-bang**: full power below target, zero or reverse above.
- **Cubic spline**: a smooth curve through the table points. **Fitted curve**: a polynomial or similar formula fitted
  to measured points. **Field-position table**: keyed by the robot's x and y instead of distance.
- **Hood from distance + measured speed**: the hood angle is also corrected when the measured flywheel speed is off
  target.
- **Voltage comp.** (battery compensation): scaling the motor command by battery voltage. **Inactive code**: written
  but returns its input unchanged or is never called. **FF only**: only the feedforward is scaled.
  **Gains scheduled**: the gains are functions of battery voltage.
- **Speed gate**: whether feeding waits for the flywheel to be at speed. **First ball**: once before the first ball of
  each volley, in TeleOp or in every auto volley. **Driver cue**: a rumble or LED only. **None**: no wait in TeleOp or
  in most auto volleys; 6 of these 12 wait once before the preload volley in one or more autos.
- **Moving shots** (shooting while driving). **Iterated**: the aim point is moved by the robot's velocity times the
  time of flight, repeated until that time settles. **One step**: moved once, using the time of flight at the current
  distance. **Fixed lead**: moved by the velocity times a constant time. **Velocity vector**: the robot's velocity is
  subtracted from the ball's launch velocity.

| repository and commit read | speed from | hood | flywheel law | voltage comp. | speed gate | slower feed at range | moving shots |
|---|---|---|---|---|---|---|---|
| [`23521-Desoto-Technix/2025-decode`](https://github.com/23521-Desoto-Technix/2025-decode/tree/0edb2219b6) `0edb221` | distance table, linear | from distance | FF + P | yes | none | yes | fixed lead |
| [`junkjunk123/RevAmped-Decode-V2`](https://github.com/junkjunk123/RevAmped-Decode-V2/tree/38e24bcf0c) `38e24bc` | presets | presets + timed move | three-mode hybrid, Kalman-filtered speed | no (inactive code) | none | no | none |
| [`kleongf/FTC_Decode`](https://github.com/kleongf/FTC_Decode/tree/29dab73e80) `29dab73` | distance table, linear | from distance | full-power band + FF | yes | none | yes | one step |
| [`FTC11329/11329-2026-repo`](https://github.com/FTC11329/11329-2026-repo/tree/ad071b81f1) `ad071b8` | distance table, linear | from distance | FF + P | yes | first ball | no | one step |
| [`Meta-Infinity/DecodePublicRelease`](https://github.com/Meta-Infinity/DecodePublicRelease/tree/7ede9cb53c) `7ede9cb` | distance table, cubic spline | from distance | FF + P, bang-bang while feeding | yes | none | yes | one step |
| [`GoldenDragons772/Decode`](https://github.com/GoldenDragons772/Decode/tree/a5eaa6ad7c) `a5eaa6a` | physics model | from physics | FF + P | no | none | yes | velocity vector |
| [`Dubble-Bubble/HIVEDecode`](https://github.com/Dubble-Bubble/HIVEDecode/tree/fed3ec448a) `fed3ec4` | physics speed, fitted hood | from distance | full-power band + FF | yes | none | no | none |
| [`MOEbo-Sapiens/MOEbo-Sapiens-Decode`](https://github.com/MOEbo-Sapiens/MOEbo-Sapiens-Decode/tree/cd3fc8de4a) `cd3fc8d` | distance table, linear | from distance | FF + P | yes | none | yes | iterated |
| [`DennNuke/ftcpanheya-worlds`](https://github.com/DennNuke/ftcpanheya-worlds/tree/01a991e7c8) `01a991e` | distance table, linear | distance + measured speed | FF + P | no (inactive code) | none | yes | fixed lead |
| [`Andrei0016/CNapSys-22586-Decode`](https://github.com/Andrei0016/CNapSys-22586-Decode/tree/73daa58c7d) `73daa58` | distance table, linear | distance + measured speed | FF + P | no | first ball | no | one step |
| [`GNCE/DECODEv2-Ditto`](https://github.com/GNCE/DECODEv2-Ditto/tree/8e63fc2f40) `8e63fc2` (note 1) | distance table, cubic spline | from distance | FF + P | yes | first ball | no | none |
| [`drwho495/14906Decode`](https://github.com/drwho495/14906Decode/tree/4b28f6ce45) `4b28f6c` (note 2) | fitted curve | from distance | FF + P on the hub's velocity mode | yes | first ball | yes | fixed lead |
| [`Xtendera/OverclockDecode`](https://github.com/Xtendera/OverclockDecode/tree/6267d38bdf) `6267d38` | distance table, cubic spline | from distance | bang-bang | no | first ball | no | iterated |
| [`Skeleton-Army/FtcRobotController-DECODE`](https://github.com/Skeleton-Army/FtcRobotController-DECODE/tree/5b17b2d653) `5b17b2d` | fitted curve | distance + measured speed | FF + P, 5-sample average | yes | first ball | no | velocity vector |
| [`SPS-Robotics/DECODE-V2`](https://github.com/SPS-Robotics/DECODE-V2/tree/536031e2b6) `536031e` | distance table, cubic spline | from distance | FF + P | no | none | no | iterated |
| [`BeepBot99/CodeBloodedDecodeV3`](https://github.com/BeepBot99/CodeBloodedDecodeV3/tree/631b3d737b) `631b3d7` | field-position table | from field position | FF + P | no | none | no | iterated |
| [`ryalisuchir/Decode`](https://github.com/ryalisuchir/Decode/tree/a51e333269) `a51e333` | field-position table | from field position | FF + P | no | none | yes | one step |
| [`dani-Nayal/SolarFlare-Decode`](https://github.com/dani-Nayal/SolarFlare-Decode/tree/0984bad719) `0984bad` | fitted curve | from distance | FF + PID, bang-bang while feeding | FF only | none | yes | iterated |
| [`Andover-Robotics/4410-Decode`](https://github.com/Andover-Robotics/4410-Decode/tree/5d7c93b39e) `5d7c93b` | distance table, linear | from distance | FF + P | no (inactive code) | driver cue | yes | one step |
| [`Scythxrz/TheFBI2025Decode`](https://github.com/Scythxrz/TheFBI2025Decode/tree/460f0aae37) `460f0aa` | distance table, linear | fixed | FF + P | FF only | every ball | yes | none |
| [`BaronClaps/22131-State`](https://github.com/BaronClaps/22131-State/tree/3f9bc800f3) `3f9bc80` | presets | presets | bang-bang | no | first ball | no | none |
| [`OmicronX-30099/FTC-Decode-v2`](https://github.com/OmicronX-30099/FTC-Decode-v2/tree/06eb4afeb6) `06eb4af` | distance table, linear | fixed | FF + P | gains scheduled | driver cue | yes | iterated |
| [`BaronClaps/16010-Worlds`](https://github.com/BaronClaps/16010-Worlds/tree/2c0e6a9354) `2c0e6a9` | fitted curve | fixed | full-power band + FF | no | first ball | yes | fixed lead |

Notes:

1. `8e63fc2` (2026-03-16) was committed after the team's season ended. It switched off the measured-speed hood
   correction and the iterated moving-shot lead, and added battery compensation. From 2026-02-17 to 2026-03-13 both
   were switched on and there was no battery compensation (read from the history).
2. The loop's output goes to motors left in `RUN_USING_ENCODER`, where the SDK turns power into a velocity target for
   the hub's own loop (read from SDK source), so its battery scaling changes that target, not the motor voltage.

## Counts

- **Speed from**: a table by distance in 13 (9 linear, 4 cubic spline), a table by field position in 2, a fitted
  curve in 4, presets in 2, a physics model in 2.
- **Flywheel law**: FF + P in 14; an explicit full-power mode in 8 (a band below target in 3, pure bang-bang in 2,
  bang-bang only while feeding in 2, a three-mode hybrid in 1); FF + P on top of the hub's velocity mode in 1.
- **Integral**: changes the output in 1.
- **Velocity signal**: all 23 control on the velocity the hub reports from a flywheel motor's own encoder; 21 use it
  unfiltered.
- **Battery compensation**: active in 12 (whole output 9, one of them as in note 2; feedforward only 2; scheduled
  gains 1); written but inactive in 3; absent in 8.
- **Speed gate**: none in 12, before the first ball of a volley in 8, a driver cue only in 2, before every ball in 1.
- **Slower feed at long range** (or longer gaps between balls): 13.
- **Hood from measured flywheel speed**: active in 3; in the commit read but switched off or never called in 2; only
  in history or on another branch in 3.
- **Moving shots**: iterated time of flight in 6, one step in 6, a fixed lead time in 4, velocity-vector subtraction
  in 2, none in 5.
- **A physics-based shot or hood calculation in history or in code the competition OpModes don't call**: 8.

## What teams tried and dropped

- **Physics for the shot.** The eight: projectile solvers with drag, closed-form models that solve for a chosen
  entry angle, and hood corrections solved from measured speed. The two teams that compute shot speed from physics
  both multiply it by a correction factor (0.47 for wheel slip in one, 1.075 for losses in the other). Two more use
  physics only for the time of flight of moving shots.
- **Hood from measured flywheel speed.** Of the five that have it but don't run it: one fed measured speed to the
  hood from 2026-04-19 to 2026-04-27 and then removed it; one wrote a physics version on a branch that was never
  merged and whose last commit (2026-03-04) says it doesn't work; one ran a physics version through its season and
  switched it off in a commit after it (note 1); one switched it off and later removed the call; one has it only as
  commented-out code on a branch made after its season. Of the three that run it, two add a single gain times the
  speed error to the hood (one only at long range); the third gets it only through its moving-shot vector math, so
  it does nothing while the robot is still.
- **The hub's `RUN_USING_ENCODER` velocity loop.** Two teams started the season on it. One moved to its own loop with
  the hub's loop off (2026-04-30); the other wrote its own feedforward + P (2026-01-04) but left the motors in that
  mode, so both loops run (note 2).
- **Bang-bang against feedforward + P.** Teams moved both ways. One ran feedforward + PID, then bang-bang
  (2026-01-24), then feedforward + PID again (2026-01-29), and was back on bang-bang for its regional event
  (2026-02-28). Another went from a bang-bang hybrid to feedforward + P (2026-03-08). Another went from
  P + feedforward to pure bang-bang (2026-02-25) and then to a full-power band.
- **Fitted curves against tables.** One repository went from a table to cubic and quadratic fits to two fixed
  presets between 2026-02-11 and 2026-02-16; another replaced a fitted curve with a table (2026-02-08); a third
  switched from a fitted curve to a table for its regional event (2026-03-06) and back to a fitted curve after it,
  on a branch later than the commit read.

## Ideas worth copying

| idea | seen in |
|---|---|
| Keep the flywheel tracking the table whenever a shot could come, so a shot rarely waits for spin-up | `Xtendera/OverclockDecode`, `DennNuke/ftcpanheya-worlds`, `Scythxrz/TheFBI2025Decode` |
| In auto, spin the flywheel (and in some, set the hood and turret) for the planned shooting pose while still driving there | `Xtendera/OverclockDecode`, `kleongf/FTC_Decode`, `dani-Nayal/SolarFlare-Decode` |
| Slow the feeder (or pause between balls) at long range so each ball meets a recovered flywheel; one team stores feed power as a column of the shot table | 13 teams; `Meta-Infinity/DecodePublicRelease` |
| A per-distance band of acceptable speed as the gate, instead of one RPM tolerance | `Skeleton-Army/FtcRobotController-DECODE` |
| Detect each shot from the dip in flywheel speed, armed only once the speed has settled, and re-open the feeder when it recovers or a timeout passes | `Scythxrz/TheFBI2025Decode` |
| Driver trim as an offset to the distance fed into the table, so speed, hood and time of flight move together | `FTC11329/11329-2026-repo` |
| Clear driver trims whenever the pose is re-set, since a trim that covered a pose error is wrong once the pose is fixed | `Meta-Infinity/DecodePublicRelease`, `FTC11329/11329-2026-repo` |
| A table-capture OpMode: set speed and hood live (dashboard or gamepad), drive to a spot, press a button to record the row | `Meta-Infinity/DecodePublicRelease`, `Xtendera/OverclockDecode` |
| Measure turret lag by spinning in place and logging (target − angle) ÷ turn rate, then lead by that | `BeepBot99/CodeBloodedDecodeV3` |
| Judge turret readiness as sideways miss at the goal (distance × sine of the aim error), so the tolerance tightens with range | `FTC11329/11329-2026-repo` |
| Two turret servos on one gear commanded slightly apart, so they load against each other and take out gear lash | `ryalisuchir/Decode`, `DennNuke/ftcpanheya-worlds` |
| Allow capped reverse power (−0.2 in one, −0.7 in the other) only when well over target, so a falling target doesn't just coast | `23521-Desoto-Technix/2025-decode`, `FTC11329/11329-2026-repo` |
| A one-button fallback to a fixed known-good shot distance, for when localization is lost | `SPS-Robotics/DECODE-V2`, `Xtendera/OverclockDecode` |
| Rumble or an LED when the flywheel is inside tolerance | `Andover-Robotics/4410-Decode`, `OmicronX-30099/FTC-Decode-v2` |

## Weaknesses seen in more than one repository

Listed so we don't copy them; the count is how many of the 23 show each.

- No speed check between the balls of a volley (22; the other re-checks before each ball only in its paced fire
  modes).
- No battery compensation that does anything (11).
- No integral that changes the output (22), so the steady speed error is whatever the feedforward gets wrong,
  divided by kP.
- Outside its range, every table read (15) clamps to its end values, extrapolates in a straight line or falls back to
  a default speed, so the robot still shoots. None refuses the shot, as our solver does
  (`TeamCode/src/main/java/org/firstinspires/ftc/teamcode/biobuzz/ShotSolver.java:269-270`, as of 2026-11-10,
  commit 366596c).
- Reverse power up to full strength once the wheel is some margin over target, from a −1 bang-bang branch or a
  feedforward + P output with no lower limit (10, counted in the first read only).
