# Shot calculation findings

How to choose flywheel speed and hood angle for a shot, what our robot's hoods allow, when the hood can make up for a
sagging flywheel, and how much to lead a moving shot. Read it before changing `biobuzz/ShotSolver.java`, the tables in
`biobuzz/ShotTuning.java`, or [`sotm-tuning.md`](sotm-tuning.md).

Researched 2026-11-10 from 23 DECODE teams' code ([`decode-shooter-survey.md`](decode-shooter-survey.md)), public FRC
code, Chief Delphi posts, projectile calculations on our CELL geometry, a generic simulation and a rerun with the
robot's CAD numbers ("The simulations"). **Nothing here has been measured on our shooters.** Exit speed per RPM,
launch angle, ball mass, drag and spin are assumed; the tests at the end replace the assumptions.

Terms: **hood offset** is degrees of hood above its lower stop (launch angle = base + offset); the **schedule** is the
offset to use at each distance. A **window** is how far exit speed (in %), hood angle or distance can be off and the
shot still pass the solver's entry rule. The **exit-speed fraction** is ball exit speed ÷ the flywheel's surface speed
just before the ball touches (the speed the table sets; the f in
[`flywheel-control-findings.md`](flywheel-control-findings.md) divides by the speed as the ball leaves instead); a
ball rolling between the wheel and a fixed hood leaves at no more than half. The **minimum-speed shot** is the
slowest exit speed that reaches the aim point at a distance. **Droop points** are extra table points at the same
distance and a lower RPM, giving the hood to use when the flywheel has sagged (`biobuzz/ShotTuning.java:12-15`).
**Points** are percentage points, of a window's width or of the simulated chance that a shot scores.

## What to do

1. **Start each table from the hood schedule below, then tune only RPM**, keyed by distance alone. Pollen 15°, 8°, 2°,
   then 0° above the stop at 36, 48, 60, and 72 in and beyond; Nectar 11.5°, 4.5°, then 0° from 60 in. In the generic
   simulation, re-picking the hood freely at each distance doubled what a linear table lost.
2. **Calibrate at 6 to 8 distances from 36 to 132 in, two of them at 60 and 72 in.** The schedule bends there, where
   it meets the hood's stop; 3 distances that skip them put the Pollen hood 5.5° off at 60 in. In the generic
   simulation, 4 distances or one 52 in gap lost 8 to 16 points against the best curve.
3. **Fix three solver settings before trusting its `Valid` line** (see "Our solver"): `exitHeightIn` from 14 to about
   13 in; an exit point 1.2 to 2.0 in ahead of the turret axis, which `ShooterGeometry` cannot express yet; and
   `rpmTolerance` as a percentage of target (about 1% at long range), not 75 RPM, the window's edge at 132 in.
4. **Settle the hood's lower stop while the CAD can still change.** It is the binding limit: moving the Pollen stop
   from 57.7° to a 46 to 50° launch angle widens the 132 in speed window from 3.9% to 4.8 to 5.4% and cuts the flight
   from 945 to 717 to 784 ms. The Nectar range is too high and too short. Film the launch angle and spin first.
5. **Measure the exit-speed fraction first on the robot.** Every RPM here assumes 0.45; at 0.35 the 132 in shot needs
   about 5560 to 5670 RPM, beyond the motor's loaded free speed at 12.5 V (simulated,
   [`flywheel-control-findings.md`](flywheel-control-findings.md)).
6. **Don't count on the hood to cover flywheel sag.** On the schedule it covers 0 to 1.5%; more costs window, RPM and
   flight time. Fix bursts at the flywheel and feed ([`flywheel-control-findings.md`](flywheel-control-findings.md)).
   Drive any droop points from the wheel speed expected at the next ball, not a reading just after one, and don't
   shoot on the move with them until the solver uses their `tofS`.
7. **Tune `kDragPerS` before shooting on the move.** At its default of 0 the solver over-leads by about 44 ms for both
   our balls (simulated for a ball with the same drag per unit mass).
8. **Keep measuring time of flight on video, to the aim point.** Computing it from a fitted model is worse.

## The robot as designed

Simulated 2026-11-10 with the CAD model as of 2026-11-10 ([`robots-findings.md`](robots-findings.md)): both turrets,
hood travel (Pollen 0 to 30°, Nectar 0 to 20°), exit points, the 8.571:1 hood drive; checked independently. Unless
stated: nominal drag, `rimClearanceIn` 1 in, a straight-on shot, exit-speed fraction 0.45, and the CAD launch-angle
base of 57.7° (Pollen) and 60.8° (Nectar) at offset 0. A second estimate from the CAD analysis gives 55.9° and 57.4°
(inferred); filming the launch angle at both ends of travel decides. In launch angle the schedule is the same on both
bases at 36 to 60 in. **The Nectar hood was reshaped after these CAD numbers; its rows were not re-derived.**

### Range and the schedule to start from

Both shooters score at every distance from 36 to 132 in inside their hood travel, in every drag case, on both bases,
for spin lift −0.3 to +0.15 and `rimClearanceIn` 0 to 2 in. That is under the solver's entry rule (falling, centre
inside the outline less radius and `rimClearanceIn`, `biobuzz/ShotSolver.java:369-374`), ignoring flywheel limits.

| distance (in) | 36 | 48 | 60 | 72 to 132 |
|---|---|---|---|---|
| Pollen hood offset, CAD base [second base] | 15° [16.5°] | 8° [9.5°] | 2° [4°] | 0 [0] |
| Pollen launch angle | 72.7° | 65.7° | 59.7° | 57.7° [55.9°] |
| Nectar hood offset, CAD base [second base] | 11.5° [15°] | 4.5° [8°] | 0 [3°] | 0 [0] |
| Nectar launch angle | 72.3° | 65.3° | 60.8° | 60.8° [57.4°] |

- The schedule is the widest speed window among shots whose hood window is at least ±2° and distance window at least
  ±3 in. It moves at most 2° (Pollen at 60 in) across thresholds from ±1°/±2 in to ±3°/±4 in, 0.5° across the low to
  high drag cases (1° without drag), 1.5° with topspin. The minimum-speed launch angle falls from 70.8° at 36 in to
  53.4° at 132 in, at 202 to 286 in/s.
- At the SDK's default servo pulse range one unit of `hoodPosition` is about 31.5° of hood, so 1° of offset is about
  0.032 (computed, assuming a linear servo). Which position is the stop, and which way is up, are unknown.
- RPM at fraction 0.45: Pollen 3048 to 4323, Nectar 3047 to 4408, 0.56 to 0.82 of a 5400 RPM free speed (assumed;
  goBILDA's tested figure for this motor is 5800, [`parts-findings.md`](parts-findings.md)). RPM scales as
  1 / fraction: the 132 in shot needs 1.03 to 1.05 of free speed at 0.35, 0.72 to 0.73 at 0.50. Most of a ball's
  speed loss happens in contact, so a fraction measured against the speed before the ball depends on flywheel inertia
  and ball mass: re-measure it after any flywheel change.

### Windows on the schedule

| distance (in) | 36 | 48 | 60 | 72 | 84 | 96 | 108 | 120 | 132 |
|---|---|---|---|---|---|---|---|---|---|
| Pollen hood offset | 15° | 8° | 2° | 0 | 0 | 0 | 0 | 0 | 0 |
| Pollen RPM at 0.45 | 3048 | 3173 | 3356 | 3501 | 3642 | 3803 | 3973 | 4147 | 4323 |
| Pollen speed (%) | −3.2/+5.3 | −2.3/+5.9 | −2.0/+6.1 | −2.1/+5.1 | −2.2/+4.0 | −2.1/+3.3 | −2.0/+2.8 | −1.9/+2.4 | −1.8/+2.1 |
| Pollen hood (°) | −2.4/+3.6 | −2.3/+6.5 | −2.0/+10.4 | −2.9/+9.7 | −4.7/+7.1 | −6.6/+5.4 | −8.4/+4.3 | −10.0/+3.6 | −11.5/+3.1 |
| Pollen distance (in) | −4.0/+5.5 | −3.7/+6.8 | −4.1/+7.9 | −10.9/+7.0 | −20.7/+6.1 | −10.6/+5.7 | −8.4/+5.5 | −7.2/+5.3 | −6.5/+5.2 |
| Pollen flight (ms) | 593 | 559 | 545 | 600 | 681 | 755 | 822 | 885 | 945 |
| Nectar hood offset | 11.5° | 4.5° | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| Nectar RPM at 0.45 | 3047 | 3179 | 3343 | 3486 | 3658 | 3841 | 4029 | 4219 | 4408 |
| Nectar speed (%) | −2.6/+5.0 | −1.8/+5.5 | −1.8/+5.2 | −2.1/+3.9 | −2.1/+3.0 | −1.9/+2.5 | −1.8/+2.1 | −1.7/+1.8 | −1.6/+1.6 |
| Nectar hood (°) | −2.2/+3.6 | −2.1/+6.6 | −2.3/+8.5 | −4.9/+5.6 | −7.7/+4.0 | −10.3/+3.1 | −12.6/+2.6 | −14.6/+2.2 | −3.2/+1.9 |
| Nectar distance (in) | −3.7/+5.1 | −3.3/+6.0 | −6.3/+6.2 | −16.5/+5.3 | −8.7/+4.9 | −6.7/+4.7 | −5.6/+4.6 | −5.0/+4.5 | −4.5/+4.4 |
| Nectar flight (ms) | 585 | 553 | 570 | 665 | 749 | 825 | 895 | 961 | 1024 |

Speed windows are relative to the RPM row; flight time is to the aim point.

- **Speed**: the slow side binds everywhere (Nectar at 132 in is even). Width falls from 8.5% to 3.9% (Pollen) and
  7.6% to 3.2% (Nectar); with an unlimited hood Pollen would still fall to 5.4%, so the stop costs 1.5 points at 132 in.
- **Hood**: the low side binds to 84 in (Pollen) and 72 in (Nectar), the high side beyond (+3.1° and +1.9° at
  132 in). On the stop the low side only matters if the stop is not where we think. At 132 in Nectar has no shot with
  a ±2° hood window.
- **Distance**: the near side (target closer than believed) binds at 36 to 60 in (Pollen) and 36 to 48 in (Nectar),
  set by the descending-entry rule; the far side beyond. The binding tolerance is 3.3 to 7.0 in.
- **Every absolute window scales about ±25% with `rimClearanceIn`** (`biobuzz/ShotTuning.java:100`, a placeholder;
  rim 0 just touches the edge). Other inputs move widths at most 0.32 points (drag), 0.69 (no drag), 0.55 (bases).

  | speed window width | rim 0 in | rim 1 in | rim 2 in |
  |---|---|---|---|
  | Pollen 36 / 84 / 132 in | 10.6 / 7.8 / 4.9% | 8.5 / 6.2 / 3.9% | 6.3 / 4.7 / 2.9% |
  | Nectar 36 / 132 in | 9.8 / 4.1% | 7.6 / 3.2% | 5.5 / 2.3% |

### The short-range limit is the solver's rule, not a proven miss

Below a certain hood offset the ball is still rising at the opening, and `requireDescendingEntry`
(`biobuzz/ShotTuning.java:101`, checked at `biobuzz/ShotSolver.java:369`) refuses the shot as `RISING`. The lowest
offset that passes is 13° (Pollen) and 9.5° (Nectar) at 36 in, 6° and 3° at 48 in (second base: Pollen 14.5° and 8°,
Nectar 13° and 6°). At that limit the hood's low side has only about 0.5°, so the schedule sits 1.5 to 2° above it.
Without the rule the outline test passes 1.5 to 3.5° lower (Pollen 10.5° and 2.5°, Nectar 8° and 0.5°). Whether a
rising ball scores is untested. The rule also sets the near-side distance limit and the hood's low side at 36 to 60 in.

### The hood's lower stop is the binding limit

- The minimum-speed angle drops below the stop beyond 84 in for Pollen (96 in on the second base) and beyond 60 in for
  Nectar (84 in); at 132 in it is 4.3° and 7.3° below. The Pollen stop is about 8 to 10° too high for the long shots;
  less with strong topspin (at lift coefficient −0.6 the minimum-speed shot stays inside travel to 132 in).
- Being on the stop costs little speed (+0.9% Pollen, +2.8% Nectar at 132 in) but costs window and flight time:

  | lowest launch angle | Pollen at 132 in: speed window / flight | Nectar at 132 in |
  |---|---|---|
  | 46° | 5.36% / 717 ms | 4.63% / 739 ms |
  | 48° | 5.11% / 746 ms | 4.56% / 749 ms |
  | 50° | 4.81% / 784 ms | 4.30% / 788 ms |
  | CAD stop (57.7° / 60.8°) | 3.89% / 945 ms | 3.19% / 1024 ms |

  At 108 in a 46° floor gives Pollen 6.19% and 654 ms (CAD: 4.80%, 822 ms). The gain holds for rim 0 to 2 in (+23
  to +40%) and topspin lift down to −0.3.
- Launch angles needed: about 47 to 79° (the most tolerant angles run 46.7 to 72.7°, plus 5 to 8° of sag room at
  36 in), against Pollen 57.7 to 87.7° and Nectar 60.8 to 80.8° in the CAD. Pollen's top above 79° only covers sag
  over 10% at 36 to 48 in. **Servo travel caps the hood at 35° at 8.571:1** (300° at 500 to 2500 µs; 31.5° at the
  SDK's default pulse range): the needed 32° fits with 3° spare, and nothing wider fits at this ratio.
- **Nectar is both too high and too short.** A 48° floor would take the 132 in window from 3.19% to 4.56% and the
  flight from 1024 to 749 ms, at 2.8% less speed. On the reshaped hood the stop's speed cost at 132 in is 1.4 to 4.6%
  for a base of 58.8 to 62.8°. The CAD's physical stop is only about 0.4° below offset 0.

### Calibration distances

`ShotSolver.lookup` interpolates RPM and hood linearly in distance (`biobuzz/ShotSolver.java:456-476`). On the
schedule RPM is within 1.2% (Pollen) and 2.0% (Nectar) of a straight line in distance; 3 to 4% on a schedule 5°
steeper, 5 to 7% at 8°. As interpolation error over a quarter of the speed window, 2 calibration distances fail (1.02,
1.53) and 3 or more pass on every schedule (worst 0.84). But 3 that miss the knee put the Pollen hood at 7.5° instead
of 2° at 60 in: window 6.94% against 8.08%, flight 663 against 545 ms. Put calibration distances at 60 and 72 in.

### Covering sag with the hood

On the schedule the hood covers 0 to 1.5% of sag: the shot sits at the bottom of the speed-against-angle curve or on
the stop. (A Pollen droop point would move it flatter at 36 in, steeper at 48 to 84 in, nowhere from 96 in.) Room
must be bought by tuning steeper than the minimum-speed angle, or than the stop where that angle is below it. Against
the schedule, over both shooters and bases (nominal drag):

| tuned steeper by | sag the hood can then cover | speed window | RPM | flight time |
|---|---|---|---|---|
| 3° | 0.45 to 3.0% | −0.12 to −1.58 points | −0.9 to +3.1% | +33 to +171 ms |
| 5° | 1.3 to 5.6% | −0.44 to −1.93 points | +0.2 to +5.5% | +103 to +207 ms |
| 8° | 3.3 to 10.4% | −0.67 to −2.44 points | +3.2 to +10.9% | +189 to +289 ms |

- The worst losses are at 48 to 84 in (Pollen at 60 in: 8.08% on the schedule, 6.36% at 5°, 5.79% at 8°). At 8° the
  near-side distance tolerance drops to 2.0 to 2.6 in (36 in on both, Nectar at 108 to 132 in) and the hood window
  to about ±1.0 to 1.5° at 120 to 132 in (Nectar −0.5/+0.8° at 132 in): the hood must repeat to about ±0.5°, ±4° at
  the servo.
- To cover 5% on Pollen, tune at offsets of 18.8°, 9.7° and 6.5° at 36, 84 and 132 in: droop points must be per
  distance, as the code has them.

**The hood is a slow trim, not a per-ball correction.**

- It turns about 78°/s at 6 V and 64°/s at 4.8 V with no load (vendor speeds through 8.571:1): 5° takes 64 to 79 ms,
  against 92 ms between balls at the 10.9 balls per second the transfer could pass (inferred).
- It would have to follow the burst spread, not the per-ball dip. A ball takes 13 to 22% (Pollen) or 22 to 37%
  (Nectar) of wheel speed in contact, but the next ball meets only what has not recovered: over 5 balls at 10.9 per
  second into one shooter, Pollen 0 / 0.9 to 2.1 / 3.6 to 11.4% at 60 / 70 / 80% of free speed, Nectar 2.4 to 4.9 /
  5.7 to 16 / 8 to 25%. Alternating the transfer between shooters (5.45 per second each) removes it for Pollen up to
  80%; Nectar 0 / 0 / 4.7%. Simulated at fraction 0.45: the per-ball loss and the alternating case at the inferred
  0.78 × 10⁻⁴ kg·m², which leaves out the rotor and feed roller; the 5-ball spreads for 0.78 to 6 × 10⁻⁴. The as-built
  estimate is in [`flywheel-control-findings.md`](flywheel-control-findings.md).
- The deficit builds in shrinking steps (Pollen at 80% of free speed: 5.0, 3.2, 1.9, 1.3% per ball), so the limits
  are the total cover a reachable schedule allows and the speed measurement ("Our solver"), not the servo. If the
  measured spread is within half the speed window (Pollen about ±3% to 84 in, ±2% at 132 in), no cover is needed.

In a generic 3-ball burst, waiting for the flywheel scored 2.64 of 3; firing through the sag with hood compensation
1.58 to 2.41. Of the 23 teams, 3 adjust the hood from measured flywheel speed, 2 more have it switched off and 3
have it only in history or on another branch; none published results. 13 slow the feeder at long range.

### Time of flight with hood cover, and shooting on the move

- Tuned steeper, covering sag shortens the flight by 17 to 46 ms per 1% of sag (more at 5° than 8°); full cover by
  112 to 159 ms at 5° (1.6 to 3.9% of sag, Pollen) and 190 to 291 ms at 8° (4.1 to 10.2%). On the schedule itself
  (Pollen, CAD base, nominal drag) the hood covers at most 1%, moving the flight −56 to +72 ms.
- The solver leads with the full-speed point's `tofS` and ignores the droop points' (`biobuzz/ShotSolver.java:244`,
  `:487`). The lead error is 0.5 to 1.4 in per 1% of sag at 30 in/s, 1.0 to 2.8 in at 60 in/s; full cover at 8° gives
  11 to 17 in at 60 in/s, against a distance tolerance of 3.3 to 7.0 in. That is for driving along the shot line;
  sideways driving gives a sideways error this vertical-plane study did not compute.
- **Don't shoot on the move with hood cover until the solver uses the droop points' `tofS`.**

## Our solver

Read from the code as of 2026-11-10 (commit `366596c`).

| what | where | effect |
|---|---|---|
| `exitHeightIn` is 14 in, one height for every hood angle | `biobuzz/ShotTuning.java:51`; used at `biobuzz/ShotSolver.java:320` | The CAD exit is 12.5 to 13.8 in, rising with the hood. With the forward offset below, it moves the needed speed by −0.3 to +0.9% on the schedule and +0.2 to +0.4% at the minimum-speed angle, against a 14 in exit at the axis (simulated). A tuned table absorbs that; the entry check behind `RISING` and `MISSES_OPENING` does not. Set about 13. |
| The ball starts at the turret axis | `biobuzz/ShotSolver.java:318-320`; `biobuzz/ShooterGeometry.java:30-38` has `exitLeftIn` and `exitHeightIn`, no forward offset | The CAD exit is 1.2 to 2.0 in ahead of the axis. Effect as above; fixing it needs a new field. |
| `rpmTolerance` is 75 RPM | `biobuzz/ShotTuning.java:73`; `opmodes/test/SotmTest.java:175` | About the whole half-window at 132 in (Pollen ±84, Nectar ±70 RPM at fraction 0.45), leaving nothing for other errors; ±129 at 36 in. Windows in RPM scale as 1 / fraction (Pollen ±108 at 0.35). A gate in percent of target fits better, about 1% at long range. |
| `readyDropoutS` 0.1 s | `biobuzz/ShotTuning.java:88`; `opmodes/test/SotmTest.java:163` | Once firing, the gate stays open 0.1 s after the speed leaves tolerance, about as long as one ball's recovery to within 1% on the CAD flywheels (57 to 139 ms Pollen, 105 to 329 ms Nectar, ideal controller, simulated), so a burst can fire through most of each sag. |
| `angleToleranceDeg` 2 is symmetric | `biobuzz/ShotTuning.java:72`; `opmodes/test/SotmTest.java:174` | Wider than the tight side of the turret window at 40° off the opening's normal from 9 ft, and at 60° at every distance (computed). |
| Droop is measured against the commanded RPM, which includes the flywheel lead | `biobuzz/ShotSolver.java:147`, `:283`, `:434-439` | While the robot drives away from the CELL, a flywheel at the right speed for the current distance reads as sagged and the hood moves for it: 75 RPM at the placeholder 12.5 RPM per inch, 30 in/s and a 0.2 s `flywheelSpinUpLeadS`. Dormant by default: `sotmEnabled` is false and both leads are 0 (`biobuzz/ShotTuning.java:77`, `:64`, `:66`). |
| Droop is floored at 0 | `biobuzz/ShotSolver.java:147` | A flywheel that is too fast is never compensated, and measurement noise only ever pushes the hood one way. |
| Droop points change only the hood | `biobuzz/ShotSolver.java:148`; flight time from the full-speed point at `:244`, `:487` | The lead stays that of the full-speed shot (see "Time of flight with hood cover"). |
| The hood follows a lagging speed reading | `biobuzz/modules/Shooter.java:25`, `:333-343`, `:197-202`; `opmodes/test/SotmTest.java:140-141` | The measured RPM is about 22 ms old (the middle of the 50 ms window, about 45 ms in practice: [`flywheel-control-findings.md`](flywheel-control-findings.md)) and the hood target waits a loop. Worse, a 20 ms (OctoQuad) speed reading ending 10 to 50 ms after a ball shows 7 to 14% droop while the next ball meets a wheel 0.4% slow (Pollen at 70% of free speed, modelled), so the hood re-aims for sag the next ball never sees. Use the speed expected at the next ball, or update only between balls. |

## Which input: distance is enough

The solver aims at one fixed point (the opening's centroid, 3 in inside it, `aimDepthIn`) from a fixed exit height, so
speed and angle depend only on horizontal distance (`biobuzz/ShotSolver.java:233-238`). Computed in 3-D without drag
at 0°, 20°, 40° and 60° off the opening's normal, a distance-only table stays inside the window at every one. The
angle of approach changes the turret instead (computed):

- The allowed turret error becomes one-sided: at 60° off the normal the tight side is 0.3 to 0.9°, under the 2°
  `angleToleranceDeg`; at 40° it drops under 2° from about 9 ft.
- The ball crosses the face off-centre, toward the sloping roof: `aimDepthIn` = 3 leaves about 1 in of room at 60°,
  1.5 would leave about 2.4 in.
- The `BEHIND_OPENING` rule (`biobuzz/ShotSolver.java:331-336`), not the trajectory, refuses close shots from far to
  the side: at 3 ft, anything over 38.5° off the normal. Whether the CELL's structure really blocks them is unknown.

Two of the 23 teams key their tables on field position (x, y). Refit against distance alone, their tables fit to 22
and 34 encoder counts/s RMS (the first inside that team's own 25 counts/s tolerance), and the second's residuals are
only weakly tied to the angle of approach (correlation −0.38): the second input mostly stored hand-tuning scatter
(computed).

## What other teams ship

Of 23 DECODE teams' competition code, read twice and independently, 9 interpolate a distance table linearly and 4
with a cubic spline, 2 key a table on field position, 4 fit a curve (polynomial or line), 2 use fixed presets and 2 a
physics model (or physics for speed with a fitted hood).

- 8 of the 23 have a physics-based shot or hood calculation in their history or unused code. One wrote down why they
  dropped it: "Too complex; Poor real-world correlation; Returned *possible* shots, not *optimal* shots"
  (`FTC11329/11329-2026-repo`, `docs/shooter-interpolation.md`).
- FRC code is mostly tables too: `Mechanical-Advantage/RobotCode2026Public` (10 points), `frc1678/C2022` (21, a
  polynomial option off), `frc971/971-Robot-Code` 2020 (16, no shots outside the table),
  `FRCTeam2910/2026CompetitionRobot-Public` (5). `Team254` is the exception: fitted curves in `FRC-2017-Public` and
  `FRC-2022-Public`, a fitted ballistic solver in `FRC-2024-Public`.
- Plan to retune. `Mechanical-Advantage/RobotCode2026Public` changed its table at least five times in one season (its
  commit history). One BIOBUZZ team reports its RPMs falling about 400 each after lubricating the flywheel and
  changing compression (https://www.chiefdelphi.com/t/524080/32, posted 2026-11-08, read 2026-11-10; not checked).
- None published a measured comparison of table against curve against physics on one robot.

## Which shot

For each distance a family of speed and hood pairs scores. For any shooter (computed, no drag): the minimum-speed shot
tolerates hood error best, and there the hood cannot help a slow flywheel. A steeper entry never needs less exit speed
and flies longer. In the generic simulation the most speed-tolerant shot and one a few degrees steeper scored best; a
much steeper lob, the fastest flat shot and a constant hood scored 8 to 33 points lower. On our robot the
speed-tolerant shot is the schedule: at or slightly flatter than the minimum-speed angle from 48 to 84 in (1.9°
steeper at 36 in, where the descending-entry rule binds), on the stop beyond.

**`sotm-tuning.md` disagrees.** Its step 4 says "Prefer a steeper, slower shot: the ball must be falling as it
enters." For this robot that holds only at 36 to 48 in, where the descending-entry rule sets the hood's lower limit
and the schedule already sits 1.5 to 2° above it. Elsewhere steeper costs and buys only sag room: Pollen at 84 in, 5°
steeper needs 1.6% more RPM, flies 811 against 681 ms and narrows the speed window from 6.2% to 5.2%; at 132 in, 8°
steeper needs 7.3% more RPM, flies 1166 against 945 ms and narrows it from 3.9% to 3.2%. From 72 in (Pollen) and 60 in
(Nectar) the hood is already on its stop, so the only choice left is how much steeper. Its step 6 has the same
problem: a −100 RPM trim is 2.3 to 3.3% at these speeds, more than the hood covers on the schedule, and its suggested
`rpmTolerance` of 100 is wider than the whole window at 132 in.

## Table, curve or physics

Generic simulation, six ball models, a tuner who keeps the hood on a schedule unless it says otherwise.

- **A linear table was never the single best choice; the gap depends on tuning.** A physics-shaped speed curve beat
  it by 8 to 9 points with 4 calibration distances, 4 to 5 with 5, 2 to 3 with 6 and 1.5 to 2 with 8. When the tuner
  re-picked the hood freely at each distance, the gap doubled.
- **Any smooth curve gets most of that gain**: a monotone cubic (the kind SolversLib's `InterpLUT` uses) or a
  least-squares quadratic came within about 2 points of it. The monotone cubic did 1 to 7 points worse than linear
  when the shooter's speed drifted (tuner re-picking the hood).
- **One bad point hurts a point-by-point table most.** A point recorded 150 RPM off dropped the worst-distance scoring
  probability to about 0.2 for the linear table and the monotone cubic, against 0.7 to 0.8 for a fitted quadratic or
  physics curve. Plot RPM against distance after tuning: on our schedule it should lie within 1 to 2% of a straight
  line, with a bend at 60 to 72 in (CAD rerun).
- **Physics constants measured off the robot, with no shots at the goal, are not good enough**: with 3% error in
  measured exit speed and a tuner who re-picks the hood, 0.48 to 0.69, well below linear.
- **Wear**: 5% less exit speed dropped every method to 0.27 to 0.41 (tuner re-picking the hood); one mid-range trim
  restored 0.75 to 0.88. The additive `rpmTrim` covers drift up to about 5%; at 10% a scaling trim is 5 points better.

## Leading a moving shot

Off by default (`sotmEnabled`, `biobuzz/ShotTuning.java:77`): the solver then treats the robot as still
(`biobuzz/ShotSolver.java:187-191`). Y toggles it in SOTM Test.

- **`kDragPerS` = 0 over-leads**: the robot gives the ball sideways speed that drag then eats. Simulated, by 19, 44
  and 125 ms for 45, 25 and 10 g balls of 2.8 in. Both our balls have about the 25 g case's drag per unit mass
  (computed from vendor masses, same assumed drag coefficient): about 44 ms. Tuned from strafing tests near and far it
  leaves 12 to 18 ms RMS, against 32 ms untuned on the same shots.
- **Measured time of flight is good**: frames counted at 240 fps gave 2.5 to 10 ms RMS of error (simulated); a model
  fitted to made shots was 12 to 114 ms off.
- **Measure to the point the solver uses.** `sotm-tuning.md` step 5 times the ball to the opening; the solver treats
  `tofS` as the time to the aim point 3 in deeper: about 15 ms more at 11 ft and 45 ms at 3 ft (computed, no drag).
- One team first measured flight times of 0.26 to 0.54 s and scaled them by 1.6, then 1.8; later it re-measured 0.53
  to 1.00 s and dropped the multiplier (its commit history). Four of the 23 lead by one fixed time (0.4 s in two). The
  lead that works can exceed a quick flight-time measurement; `transferTimeS` (`biobuzz/ShotTuning.java:54-58`) is
  for the extra.

## The simulations

Commands: [`scripts/sim/README.md`](../scripts/sim/README.md). "Computed" means closed-form, `scripts/sim/ballistics/`.

**Generic study** (`scripts/sim/shotcalc/`): six models of a 2.8 in ball (no drag; drag at 10, 25 and 45 g; drag
with backspin lift at 10 and 25 g) flown to the opening as `BioBuzzField` defines it, scoring when the centre crosses
inside the outline less its radius and 1 in, falling. The simulated tuner sets RPM, then hood, at random in the middle
60% of the window. Checked independently to under 0.03 in; other seeds and tuner models showed results depend on the
tuner. Its exit speed per RPM, 14 in exit at the turret axis and 30 to 80° hood were assumed; the CAD rerun replaces
them. No bounce-outs: one BIOBUZZ team with a fixed solid hood reported 24 of 43 shots in and 44% bouncing out,
blaming backspin (https://www.chiefdelphi.com/t/524080/12, posted 2026-09-27, read 2026-11-10; not checked).

**CAD rerun** (`scripts/sim/cad/shotcalc/`; bursts in `scripts/sim/cad/flywheel/`): the same opening and entry rule,
in the vertical plane through the turret axis and the aim point, straight on, no sideways exit offset.

- From the CAD: ball diameters, hood travel, the 57.7° and 60.8° bases, a 72 mm wheel at 1:1 (Nectar's ratio
  assumed). Inferred: exit points (linear in hood angle), the 55.9° and 57.4° bases, flywheel inertia
  0.78 × 10⁻⁴ kg·m² (sag only). Vendor masses via the CAD notes, not re-checked: Pollen 23.5 to 27.2 g, Nectar 39 to
  45 g.
- Assumed: exit-speed fraction 0.35, 0.45 or 0.50; 5400 RPM free speed; quadratic drag at 1.204 kg/m³ with drag
  coefficient 0.4 (heaviest ball), 0.5 (nominal) or 0.6 (lightest); lift coefficient −0.3 to +0.15; the schedule's
  ±2° and ±3 in thresholds; the hood's vendor speed at no load.
- Checked independently: closed form to 0.0004 in/s without drag; a separate solver to 0.02 points, 0.02° and 0.1 in.

## Still unknown

| question | what it changes | test |
|---|---|---|
| Exit-speed fraction | Every RPM; whether 132 in fits the motor | Film shots from the side at 240 fps with a scale in frame, at 3 RPMs and 2 hood angles. Divide exit speed by the wheel's surface speed logged just before the ball touches. Repeat after any flywheel change. |
| Launch angle at both ends of hood travel | Which base is right; the offset numbering; where the minimum-speed shot leaves travel | The same film, with the hood at its stop and at the top of travel. |
| Spin direction and rate | Up to 1.4% of speed and 1.5° of schedule; strong topspin moves the 132 in minimum-speed angle 4.75° up, inside Pollen travel | Film a marked ball at 240 fps. The CAD geometry implies topspin. |
| Ball mass and drag | The 132 in speed by 4.6 to 4.9% between the low and high drag cases; the lead | Weigh 10 to 20 of each. Fit drag from a range sweep at fixed RPM and hood, or from the film. |
| Does a ball entering while still rising score? | The short-range hood limit (1.5 to 3.5° lower if yes) and the near-side distance limit | At 36 and 48 in, shoot with the hood 1 to 3° below the solver's limit; count balls that go in and stay. |
| Hood repeatability and backlash | Whether shots tuned steeper are usable (they need ±0.5°) | Command one hood position from above and from below, 10 times each; measure the hood angle each time. |
| Balls per second delivered to each shooter | The burst spread; whether hood cover is needed at all | Film the transfer through a 5-ball burst, logging flywheel speed at a 5 to 20 ms window, at 3 distances. |
| How much rim clearance shots need | Every window, about ±25% | Ten shots at one setting against a marked board at the opening's distance; count edge hits and bounce-outs at a flat and a steep entry, and tune on "stayed in". |
| Whether droop points help at all | Whether to keep them | Log measured RPM at release and where each ball crosses the opening, with compensation on and off. |
