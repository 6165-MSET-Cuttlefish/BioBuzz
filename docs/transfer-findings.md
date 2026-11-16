# Transfer findings

How fast balls move through the BIOBUZZ robot's transfer, how to measure it, and where rangefinders can see them.
Read it before mounting or moving a transfer sensor, choosing a sensor's timing or sampling rate, writing code that
counts or times balls, or trusting a feed rate in a shooter simulation.

**Nothing on this page has been run on the robot.** Each number says how it is known:

- **(CAD)**: measured on the team's CAD as of the evening of 2026-11-14 (about 23:00 PST), and, for the paddles,
  where balls rest and everything near the intake, as of 13:13 PST on 2026-11-15 with both turrets placed on the
  transfer. The CAD was still changing: the transfer's part shapes are from the evening of 2026-11-14 and lack
  pockets cut on 2026-11-15, the assembly that places the turrets uses an older version of the transfer that nobody
  compared with the current one, and every number near the intake shifts 1:1 with the transfer's front-to-back
  placement, which is known to only a few mm.
- **(vendor)**: from the vendor page or chip datasheet linked under [Sources](#sources), read on 2026-11-15 unless it
  says otherwise.
- **(manual)**: the Competition Manual, read on 2026-11-15 ([Sources](#sources)).
- **(team)**: the team said so; not checked.
- **(inferred)**: worked out from the above; the reasoning is given.
- **(simulated)**: a model of balls, beams and the hub's sampling; its assumptions are listed with its results.

## What to do

- **Spots A to E can't count balls coming in, so don't drill them for the possession count.** While a paddle holds
  balls, the queue stops on or short of them, so they see only the first one or two balls of a load. They suit
  counting balls leaving toward the turrets, and the timing measurement below
  ([Rangefinder spots](#rangefinder-spots)).
- **Count balls coming in with one beam per track, at two spots behind the intake, and treat a beam that stays
  covered as "that track is full".** Pollen: a low beam at (y 66, z 16), 6 mm above the floor plate, through the
  −x side plate and the floor plate's −x cheek. Nectar: a beam at (y 33.0, z 106.6), in front of and above the
  Nectar's path where it climbs the ramp, through the −x side plate. Each sees only its own ball type, and together
  they count every mix of up to 4 balls right, including a 4th ball stopped under the intake (inferred from the CAD's
  geometry; neither spot was checked in the CAD itself and nothing was tested). That holds only while the Nectar
  paddle can't close; if hardware makes it close, the plan must change
  ([The plan](#the-plan)). What could kill it: the cheek may not be solid enough to drill; the lasers read the ball
  at 30 to 35 mm, inside ST's 40 mm minimum; the plan needs the narrow 15° field at a 10 ms budget, and Brushland
  says the narrow field is noisier and needs a longer budget (vendor); and at a 10 ms update a laser counts 4
  touching balls right in only 87 to 92% of runs at 2.3 m/s and 62% at 3.5 m/s, at ST's 20 ms in 0% (Pollen) and
  65% (Nectar) at 2.3 m/s. Break-beams read every 5 ms count all of them (simulated)
  ([Counting balls coming in](#counting-balls-coming-in)).
- **Don't count with the CAD's own spots 1 and 4 as a Pollen and a Nectar counter (spot 1 alone is only a fallback),
  and don't build a two-laser direction pair at the hand-off.** Both spots see both ball types, so read that way they
  count most balls twice (inferred); touching Nectars leave only 1.2 mm between them at spot 4; and spot 1 never sees
  a 4th ball that stops under the intake behind 3 Nectars. No pair of beams 16 to 17 mm apart fits anywhere there
  (CAD).
- **Read Nectar colour with the Color Rangefinder on the +x side, on the guide's flange, at the CAD's spot C1 (s 25)
  or at s 32 to 35.** It sees every Nectar pass and the last Nectar in at rest, though at free speed only 52 to 83% of
  passing Nectars get one whole colour reading (simulated). C1 gets the most passing readings and the earliest colour,
  and the CAD checked it; at s 32 to 35 a passing Pollen stays out of a 15° view, where at C1 it fills up to 15%. A
  bench test with a Pollen in view decides (C1 is CAD, s 32 to 35 inferred)
  ([Colour](#colour-the-color-rangefinder-on-the-x-guide-flange)).
- **Stop the intake at a count of 4, or as soon as either track reads full, and don't reverse it then.** With 3
  Nectars in, a 4th ball of either type stops under the intake, where no sensor sees it and it already counts as
  CONTROLLED (CAD + manual). Reversing the intake would also throw out the 3rd Nectar, which only the intake holds;
  reverse only for a suspected 5th ball.
- **Only the Pollen paddle can close as drawn** (CAD, 2026-11-15). Two brackets stop the Nectar paddle long before
  it reaches a ball. Ask hardware where it is meant to sit
  ([Path, capacity and the blocker](#path-capacity-and-the-blocker)).
- **Take the one measurement below before trusting any speed here.** It tells which part of the transfer sets the
  ball rate, and it needs only one rangefinder and Transfer Thread Count Test.
- **The transfer's continuous servo `melonbotics` is a Melonbotics Super Servo Plus** (CAD). It sits on the lower
  rear Pollen roller shaft. The only Axon MAX in the CAD turns the front sweeper bar's hinge
  ([Parts](parts-findings.md#servos)).
- **Bench-test a Laser Rangefinder at 15 to 45 mm, in SHORT mode at a 10 ms timing budget, before anyone drills a
  hole for one.** Its chip is an ST VL53L1X, and ST's datasheet promises accurate distances only from 40 mm and a
  timing budget no shorter than 20 ms; Brushland lists 10 ms ([The sensor](#the-sensor)). Every spot on this page
  has the ball 15 to 44 mm from the sensor face.
- **Try a 10 ms timing budget (SHORT mode) for any transfer rangefinder, but don't count on it.** At 20 ms no spot
  sees the gap between touching Pollen balls for two readings, even with the transfer motor loaded. At 10 ms, A and B
  see it only while the train is slowed (motor loaded, or the servo roller metering), and maybe not even then (the
  beam is a cone: see [below](#ball-and-gap-times-and-which-sampling-rate-works)). No Nectar spot works at 20 ms.
- **Don't drill spots A and B as drawn: the two sensor bodies would overlap.** See
  [What to tell hardware](#what-to-tell-hardware).
- **Don't trust "the transfer passes 10.9 balls a second".** See [that section](#the-109-balls-per-second-figure).

## The measurement that settles most of it

Feed five touching Pollen balls past one rangefinder and read the time between balls. A touching train passes a
point every D ÷ v, where D is the ball diameter (71.1 mm) and v the speed of the slowest driver behind that point, so:

| ball-to-ball period | what sets the speed (inferred from the speeds below) |
|---|---|
| about 46 ms | the side rollers, at the motor's free speed; the servo roller doesn't matter |
| 53 to 67 ms | the Super Servo Plus roller (1000 down to 800 rpm) |
| about 88 ms | the side rollers with the motor loaded |

How, with Transfer Thread Count Test (usage in `CLAUDE.md`):

1. With the sensor on I2C and `laser_rangefinder_i2c.xml` active, run Configure Laser Rangefinder with
   `distanceMode` SHORT, `timingBudgetMs` 10, `measurementPeriodMs` 0 and `roiSize` 4 (the 15° field of view). Its
   defaults are SHORT, 20, 0 and 4, so only the budget changes. Note its `analogHighMm` (1300 by default). Brushland
   lists SHORT at 10 ms as 100 Hz ranging (vendor).
2. When it says it wrote the analog config, power-cycle the sensor and plug it into Control Hub analog port 1,
   `frontRangefinder` in `biobuzz.xml`.
3. Make `biobuzz.xml` the active config again. The test asks at INIT for `frontRangefinder`, `backRangefinder`,
   `colorRangefinder`, `transfer`, `intake`, `melonbotics`, `fl`, `bl`, `fr` and `br`, and the downstream sensor
   must be on the same hub as the upstream one even with `oneSensor` on. `biobuzz.xml` has all of them, with the
   three analog inputs on the Control Hub.
4. Put the sensor where it sees each ball and the gap behind it: spot B below is the CAD's best, but any spot the
   beam crosses with the ball off its centre works. A beam through the ball's centre sees no gap between touching
   balls.
5. Before INIT: `runThread` on, `oneSensor` on, `upstreamSensorName` `frontRangefinder` (the config name of the port
   the sensor is in), `maxRangeMm` the `analogHighMm` from step 1, `stopAtCount` 5 (or 0). Set
   `oneSensorThresholdMm` halfway between the ball and empty-channel readings.
6. With the channel clear, flip `resetCount`. Set `motor1Power` to the power to test (1 for free speed) and
   `intakePower`, then push five Pollen balls in touching each other.
7. Read `Ball-to-ball periods timed since reset` (should be 4), `Ball-to-ball period, mean (ms)` and the list
   `Last 8 periods, oldest first (ms)`. Also note `motor RPM` and `Motor Current (A)` while the balls pass: they say
   whether the motor was free or loaded.

How good the numbers are:

- The thread polls every `pollMs` (10 ms by default). A rangefinder gives a new distance every 20 ms at its default
  20 ms budget ([`hub-bench-findings.md`](hub-bench-findings.md): the hub refreshes its ADC about every 10 ms,
  measured 2026-10-14 and 2026-10-16), so each time is good to about one sensor update or one poll, whichever is
  longer: ±20 ms at 20 ms, ±10 ms at 10 ms.
- The mean of four periods is the first-to-last time divided by 4, so it is good to about a quarter of that.
- At free speed, at A or B, a ball shows for about 28 to 32 ms and the gap for about 16 to 19 ms, or less (below). At
  a 20 ms budget a gap can fall between two readings, two balls then count as one, and one period reads about
  double. If the count isn't 5, or one period is about twice the others, repeat at a 10 ms budget.
- With `runThread` off the loop times the same events, once per loop, so each time is good to about one loop. A loop
  is longer than `loopMs` (20 ms by default): it waits until `loopMs` has passed, then still adds about 30 telemetry
  lines and reads the motor's encoder and currents. `loop time (ms)` shows most of it.

What the two other times mean, traced from `TransferCounter.Edge` (it arms when the signal is at or below the
threshold minus `hysteresisMm`, and counts when an armed signal rises above the threshold):

- **One sensor** (signal = the sensor's distance): a ball in front reads short, so the counter arms as the ball's
  front edge reaches the beam and counts as its back edge leaves. `Ball in front of the beam` is the time the ball
  blocks the beam; `Gap between balls at the beam` runs from that count to the next ball's front edge. The period is
  ball plus gap, back edge to back edge. Hysteresis shortens the ball time a little and lengthens the gap.
- **Two sensors** (signal = A − B): an empty channel and a ball covering both read about 0 and keep it armed; a ball
  that has left A but still covers B reads large and counts. `Ball's back edge from A to B, or until the next ball
  reaches A` runs from that count until B clears too or the next ball arrives at A, whichever is first. When the gap
  behind the ball is longer than the beam spacing, that is the back edge's travel from A to B, so the ball's speed =
  the two beams' spacing along the path (s, below; 20 mm for A and B, not their 13.4 mm straight-line distance) ÷
  that time.
  `From re-arm until a ball leaves A` is the rest of the period; with spaced balls it includes the wait for the next.
- The first ball after INIT, after `resetCount`, or after a change to `oneSensor`, a threshold or `hysteresisMm`
  gives no period. A spell already armed at INIT or at such a change isn't timed, nor is one already armed when
  `resetCount` is flipped; the timings shown always belong to the current `oneSensor` setting.

## Ball speeds

Formulas, used for every row:

- Surface speed of a wheel = rpm ÷ 60 × 2π × r.
- A ball driven on one side and rolling on a fixed surface on the other moves its centre at half the surface speed.
- On the intake floor the wheel touches the ball off its centre, so centre speed = surface ÷ (1 + r_s ÷ R): R is the
  ball radius and r_s the radius of the ball's slice in the wheel's plane (Pollen 30.9 mm, Nectar 42.3 mm; CAD).
- Time to pass a point = D ÷ v (Pollen D 71.1 mm, Nectar 91.4 mm).

**Free and loaded.** "Free" is the motor at its free speed. "Loaded" is 52.5% of it, inferred from the team's figure
of 4.5 A drawn by the transfer motor with no balls (team, relayed with the CAD study): on a straight line from 0.25 A
at free speed to 9.2 A at stall (vendor), 1 − (4.5 − 0.25) ÷ (9.2 − 0.25) = 0.525. That assumes current rises in a
straight line as the motor slows (a brushed motor at a steady 12 V), and that the 4.5 A was at full power. 1150 rpm
× 0.525 = 604 rpm.

**Drives** (CAD): the transfer motor, goBILDA 5202-2402-0005 (1150 rpm free; vendor), turns every transfer shaft it
drives 1:1 (all pulleys 16T:16T or 24T:24T, bevels 14T:14T). The intake motor, goBILDA 5204-8002-0003 (1620 rpm free;
vendor), turns its roller 16T:16T by belt. The Super Servo Plus is drawn coaxial with the lower rear Pollen roller
shaft, with an adapter, hub and mount plate, but the shaft stops 5.3 mm short of the hub, so as drawn nothing turns
that shaft (CAD).

Speeds in mm/s, free / loaded.

**s**, used everywhere on this page, is mm along each ball's own centre path, 0 where the centre crosses y 46.9,
the side plates' lowest front edge. The floor plate's own front face is at y 49.86 between the cheeks, so s 0 is
about 3 mm ahead of it (CAD, 2026-11-15). Each ball type has its own path, so the same s is a different place for a
Pollen and a Nectar. In the transfer frame ([Where they go](#where-they-go)): on the tile, Pollen s = y − 49.5 and
Nectar s = y − 55.4; on the floor plate, Pollen s = y − 47.0 up to s 40; the Nectar's centre (y, z) is (47.0, 67.8)
at s 0, (61.2, 81.8) at s 20, (73.4, 97.6) at s 40, (83.4, 114.9) at s 60 and (91.1, 133.3) at s 80 (CAD). Beams
near the intake are given as (y, z) as well, since one beam sits at a different s on each path.

| where | what touches the ball | surface | ball centre | contact (s) |
|---|---|---|---|---|
| Intake, on the floor | three 2 in compliant wheels, r 25.4 (CAD), at 1620 rpm | 4309 / no figure | Pollen 2304, Nectar 2238 / no figure (no intake current known) | at the drawn arm pose, Pollen −99.5 to −3.0, Nectar −124.5 to +18.5 (CAD); with the arm up, less ([below](#the-intake-hand-off)) |
| Pollen side rollers V1 and V2 | two vertical 2 in compliant wheels on the −x side, ball pinched against a fixed guide (CAD) | 3058 / 1606 | 1529 / 803 | V1 46.8–57.8, V2 97–110 where they touch at all; 49–55 and 99–108 with at least 0.2 mm of overlap (CAD) |
| Pollen ramp | nothing in the CAD: the servo's 2 in roller misses the ball by 0.9 mm, and the next roller, an unbelted idler, grazes it at s 232 (CAD) | | moves only when pushed | 111–280 |
| Servo roller, if a real compliant wheel touches | Super Servo Plus at 1000 rpm; 833 or 800 rpm if it runs at 5 or 4.8 V (inferred, below) | 2659 | 1329; 1107; 1064 | within 111–280 |
| Pollen spokes | surgical-tube spokes on the two upper rear shafts, 2 lower, 1 upper (team; not in the CAD), tip r 33.2 (team's "about 28 mm" tubes + a 5.2 mm base, assumed) | 3998 / 2100 at the tip | at most 1999 / 1050 (one side against the rear wall) | 280–416 |
| Nectar ramp | leaving the intake | | 1610 free (inferred, 0.374 × the intake surface) | |
| Nectar spokes | tubes on the two front shafts, the lower coaxial with the motor (team), the upper 1:1 by belt (CAD), tip r 42.7 (team's 37.5 mm tubes) | 5142 / 2701 at the tip | 0.363 to 0.5 × tip: 1867–2571 / 980–1350 (inferred: the ball rolls in a trough that fits it) | 29–202 |

- Spokes push in pulses, 4 per turn per spoke: 77 per second free, 40 loaded (inferred).
- The intake arm floats and nothing in the CAD fixes where it rests, so its resting height is unknown. A wedge drawn
  loose under the intake's wheel hubs would stop it about 2° to 3° above the drawn pose, if it is real and sits where it
  is drawn (CAD, 2026-11-14). The CAD gives 49° above the drawn pose as its up-stop, which nobody rechecked. At the
  drawn pose its wheels overlap a Pollen on the tile by 6 to 16 mm and a Nectar by 19 to 29 mm, so the arm lifts for
  every ball. At 15° above it they clear a Pollen by 0.8 to 7.8 mm, only just missing it, and still overlap a Nectar by
  8 to 18 mm (CAD).
- **The servo's speed.** The vendor gives 1000 rpm free with no voltage, next to "6 W at 6 V". 2.3 kgf·cm × 1000 rpm
  ÷ 4 = 5.9 W, the peak power of a motor with that stall torque and free speed, which matches 6 W, so 1000 rpm is
  very likely at 6 V (inferred). 833 rpm at 5 V and 800 rpm at 4.8 V assume speed scales with voltage (inferred). The
  wiring map powers it from servo injector B ([`wiring-findings.md`](wiring-findings.md)), which the CAD runs at
  6 V, so 1000 rpm is the likely case. No loaded speed is known. The vendor's "about 450–700 RPM" is advice on how to
  gear the servo, not a loaded speed.
- **Every part of the path runs at 1.1 to 2.6 m/s free**, so no part of it is much slower than the rest (inferred).

### Hand-offs

Balls arriving touching (inferred from the speeds above):

- **Intake to side rollers (Pollen):** 2304 to 1529, ratio 0.66, so the balls bunch with no gap. From where the
  intake stops touching a Pollen to V1 (s 46.8), 49.8 mm are undriven with the arm at its drawn pose, 63.8 mm with
  it 5° up and 82.8 mm with it 10° up (CAD contacts, inferred lengths). A lone Pollen that stops there can rest
  anywhere in s −0.5 to 46, touched by nothing (CAD).
- **Side rollers to the servo roller:** ratio 1329 ÷ 1529 = 0.87 or less, so if the servo roller touches, it meters
  the train.
- **Into the Pollen spokes:** ratio 1999 ÷ 1529 = 1.31. A faster driver pulls a gap of D × (ratio − 1) = 21.9 mm,
  10.9 ms at 1999 mm/s free and 20.8 ms at 1050 loaded. If the servo meters at 833 rpm: D ÷ 1107 − D ÷ 1999 =
  28.6 ms.
- **Nectar ramp to spokes, free:** ratio 1.16 to 1.60, gap 15 to 55 mm, 8 to 21 ms. Loaded: no gap.
- **Downstream of the slowest driver, ball time + gap time = D ÷ (its speed)**, wherever the beam crosses the ball.

### Time to pass a point

Centre beam, free / loaded, ms (inferred, D ÷ v):

| | intake | side rollers | ramp | spokes | touching train, ball to ball |
|---|---|---|---|---|---|
| Pollen | 30.9 / none | 46.5 / 88.5 | | 35.6 / 67.7 | 46.5 / 88.5; 64 if the servo meters at 833 rpm |
| Nectar | 40.8 / none | | 56.8 | 35.6–49.0 / 67.7–93.3 | 56.8 free |

In balls per second, a touching train is 21.5 free and 11.3 loaded on Pollen, 15.6 with the servo metering at
833 rpm, and 17.6 free on Nectar (1000 ÷ period).

## Path, capacity and the blocker

From the CAD:

- From the intake hand-off to the turret plate: Pollen 422.8 mm, Nectar 202 to 203 mm.
- A Nectar ball blocks the Pollen path while its centre is at s 72.5 or less (CAD, 2026-11-15).
- **Where balls stop** (ball centres in s; CAD, 2026-11-15; each arriving ball stops against the one before it):

| case | Pollen | Nectar |
|---|---|---|
| paddles open | 422.8, 350.6, 278.6, 206.6, 133.9, 62.8; a 7th and 8th at −8.6 and −81.4 | 202.3, 110.5, 17.7; a 4th at −81.0 |
| Pollen paddle closed | 240.55, 168.6, 97.3, 26.3; a 5th and 6th at −46.5 and −117.6 | as with the paddles open: the Nectar paddle can't close (below) |
| behind a Nectar resting at s 17.7 (3 Nectars in) | −58.5 | −81.0 |
| behind a Pollen at about s 61 (the 6th, paddles open) / at about s 26 (the 4th, paddle closed) | | −27.8 / −64.5 |
| pinched under a stopped intake at its drawn pose | −100 (−92 to −93 with the arm 10° to 15° up) | −124.5 (−117.5 with the arm 20° up) |
| alone, nothing driving it | anywhere in −0.5 to 46, 59 to 96 or 111 to 161.5 | not in 18.5 to 29.3, between the intake and the spokes: it rolls back (inferred) |

- The third Nectar at s 17.7 sits below where the front Nectar spokes first touch (s 29.3), so only the intake
  wheels hold it, and it stays at s 17.7 only while the arm rests within about 1° of the drawn pose (their contact
  reaches s 18.3 to 18.5 there). With the arm higher it rolls back to where the wheels' contact ends: about s 15 at
  2° to 3° up, s 12 at 5° and s 3 to 4.5 at 10° (inferred from the CAD's contacts).
- **With the paddles open, only a stopped shooter wheel holds the top ball** (CAD). Pollen: the shooter's side
  plates graze it by 0.9 to 2.0 mm, a 1 in. wheel and a hood each overlap it by about 1.5 mm, and a wheel ring
  overlaps it by 6.5 mm once it is 40 mm past the turret plate.
  Nectar: the Stealth Wheel overlaps it by 1.4 to 6.1 mm, and the flywheel ring by 13.3 mm at 50 mm past. With the
  flywheel spinning the ball is shot; with it stopped, the top ball may rise up to about 40 mm and the whole queue
  with it (inferred).
- A lone Pollen pushed only by the side rollers at loaded speed stalls on the undriven ramp at about s 210 to 220
  and rolls back to s 161.5 or less; at free speed it only just reaches the Pollen spokes (inferred).
- The CAD draws no part named blocker. What it has: each turret tab holds a part named "[LS] Pulley HTD5 20T", a 20T
  pulley with a flat 77 mm paddle on a vertical pivot hanging under the turret plate.
  - **Pollen paddle**, at z 120 to 123 (CAD, 2026-11-15). In its drawn pose (0°) it is clear of the path, so that pose
    reads as open. It closes anticlockwise seen from above, rear end first. It swings freely from 61° clockwise to 116°
    anticlockwise, and the end of a transfer shaft stops it at 62° clockwise. It touches a ball, clear of the walls,
    from 117° to 183° anticlockwise, and the +x plate stops it at 183° to 184°. It closes the path (reaches at least
    21.55 mm into it) only from 138° to 183°, and reaches furthest (35.55 mm) at 156° to 168°. It holds the first ball
    at s 240.5 to 243.5 from 156° to 183°, and at s 255 at 138°; the queue behind it is in the table above. It crosses
    the Pollen path at about s 264 to 294 (CAD, 2026-11-14; not rechecked for the 2026-11-15 placement).
  - **Nectar paddle**, at z 241.35 to 243.85, pivot at x 43.57, y 98.36. Its post sits just outside the +x plate at the
    Nectar column (x 39.6 to 47.6, y 94.9 to 101.9), a sensible place, but **as drawn it can't close** (CAD, 2026-11-15,
    by two independent methods). It swings freely only from 40° clockwise to 57° anticlockwise of its drawn pose: two
    brackets (CAD part 1204-0002-0001), at x 27.9 to 70.9 and z 209.9 to 252.9, one at y 45 to 53 and one at y 165 to
    173, sit in its plane and stop it from 58° anticlockwise and from 41° clockwise, and the +x plate from 89°. It would
    touch a ball only from 123° to 196° anticlockwise, which it can't reach from either side. If it could close it would
    hold balls at s 143.3, 50.6 and −45.1. The transfer's part shapes measured here are from the evening of 2026-11-14
    and lack pockets cut on 2026-11-15, so ask hardware whether those brackets are pocketed or the paddle is meant to
    sit elsewhere.
  - **A goBILDA 2000-0025-0504 five-turn servo** with a hub and a matching 20T pulley sits unmated in the Pollen turret
    tab, far from the paddle. Very probably the blocker's servo, with the belt not drawn (inferred). As parked, its
    output axis is horizontal and the paddle's is vertical, so no belt could join them in that pose.
  - **Code range to angle** (inferred): Basic Tele and Intake Transfer Shooter Test move `blocker` between 0.53
    (closed) and 0.60 (open), and nothing calls `setPwmRange` on it, so the SDK's default 600–2400 µs applies:
    0.07 × 1800 µs = 126 µs, × 0.90°/µs on this servo (vendor: 1800° over 500–2500 µs) = 113°. That is only the size
    of the swing. Measured from the drawn open pose, 113° falls short of the 117° where the paddle first touches a
    ball and of the 138° it needs to close the path. It closes the path only if the servo's open position sits 25° to
    70° anticlockwise of the drawn pose, and the paddle would still be clear there, since it touches nothing up to
    116°. The CAD models neither the servo's zero nor how the paddle is driven, so nothing here says where 0.53 and
    0.60 put it. In Basic Tele the blocker sits at 0.6 (open) only while a flywheel is on and at 0.53 (closed)
    otherwise (`BioBuzzBasicTele.java`).
  - **The code disagrees with itself:** `biobuzz/modules/Shooter.java` calls 0.6 BLOCKED and 0.3 UNBLOCKED
    (`ShooterHardstopState`), where Basic Tele calls 0.6 open. On this servo 0.3 apart is 486° (inferred), far past
    the paddle's swing.

What the docs already said about the blocker: it is "the blocker before the shooters"
([`robots-findings.md`](robots-findings.md)), `blocker` on Expansion Hub servo 4 powered from injector B
([`wiring-findings.md`](wiring-findings.md)). Nothing named its servo or where it acts.

Questions for hardware:

- Which paddles exist on the robot: Pollen, Nectar or both? The code has one `blocker` port for two drawn paddles,
  and as drawn only the Pollen one can close.
- Where is the Nectar paddle meant to sit, and are the two brackets in its swing pocketed?
- Which servo drives each, through what belt, and which `blocker` position is closed?
- Can the turrets turn at all with the paddle posts there? Each post hangs 76 to 81 mm from its turret's axis, so
  most turret angles would run it into the side plates (CAD + inferred).

## The intake hand-off

The two ball types share a rolling surface only on the field tile in front of the side plates, where nothing fixed
can carry a sensor. From the floor plate's front edge they part: the Pollen rolls flat on the floor plate and the
Nectar rides up the plate's two cheeks. So a low beam over the floor plate, far enough back that the Nectar has
climbed clear of it, sees only Pollen, and a beam on the Nectar ramp, in front of and above the Pollen path, sees
only Nectar ([Counting balls coming in](#counting-balls-coming-in)). All of this is CAD, 2026-11-15, unless
marked; every number in it shifts 1:1 with the transfer's front-to-back placement, known to a few mm.

**Surfaces**

- The floor plate's front face is at y 49.86 across x −70 to +5, and slopes forward to y 46.9 only over the outer
  20 mm or so at each side plate. It is a square edge with its top at z 10.05 over a tile at z 0.05, so a 10 mm step;
  the plate is 5 mm thick (z 5.05 to 10.05) with an open 5 mm slot under it.
- A Pollen rolls on the tile until s −24.2 (centre y 25.25), 21.65 mm before y 46.9 and 24.6 mm before the floor
  plate's face (inferred from the CAD's positions). It then pivots on the plate's edge (y 49.9, z 10.05) until about
  s +3, and rolls flat on the plate from there, centre z 45.6.
- A Nectar leaves the tile at s −33.8 (centre y 21.5), 25.4 mm before y 46.9 and 28.4 mm before the face (inferred).
  It is on the −x cheek's front corner by s −25 to −20 and rides both cheek crests from about s 0, where its centre is
  at (y 47.0, z 67.8) and its bottom 22.1 mm up.
- The Nectar leaves the low band early. Its bottom is at z 5.5 at y 30, 8.7 at y 33, 12.6 at y 36 and 15.5 at y 39,
  where the Pollen's is 7.7 at y 37. The Nectar's bottom is above the Pollen's by 0 up to y 22, 2.6 mm at y 25,
  10.6 at y 45, 24.7 at y 60 and 52.6 at y 80. So a fixed low beam at y 30 to 47 sees a Nectar only along a short
  chord, or misses it.
- The cheeks are the floor plate's ramp sides. Their inner faces are at x −70.53 and +5.67 (76.2 mm apart), and from
  y 49.6 they fill the strip inside each side plate: 19.07 and 19.03 mm wide at those faces, 19.94 mm at z 16, where
  the −x inner face is at x −69.66. Whether they are solid or hollow is unknown. Their front corners are uneven: −x
  37.9 mm high, +x 30.6; the +x crest stays flat at about z 31 from y 49.5 to 55, then rises 12 mm by y 57. The
  crests reach z 120.2 at y 113.

**Beside the path**

- Side plates: inner faces at x −89.6 and +24.7 (114.3 mm apart), bottoms at z 5.05. Their front edge is at y 46.9
  below z 49, y 40.9 from z 49 to 69, y 11.9 from z 69 to 182, slopes from z 182 to 202, and is at y −43.1 above
  z 202.
- Both plates are open behind y 69 from z 38 to 92, and behind y 81 from z 13 to 107. So at ball-centre heights the
  only plate material behind the hand-off is a strip 22 to 28 mm long, from y 46.9 (40.9 above z 49) to y 69. The
  side-roller mount (CAD name Part 1 <8>) with V1 fills the −x opening; a fixed guide (Part 1 <15>) fills the +x one.
- The funnels (y 31.4 to 56, z 5 to 53) sit entirely outside the plates (x −92.75 and below, 27.9 and above), so they
  catch only balls wider than the 114.3 mm opening.
- On the tile nothing fixed centres a ball: only the intake wheels touch it, and the centre line falls in the 36 mm
  gap between two of them. Fixed width at the Pollen centre's height: none up to s −20; 187 mm, funnel to funnel, at
  s −10; 114.3 mm from s 0 (21.6 mm of play each side); the cheeks' 76.2 mm only from y 59.7, s 12.7 (2.55 mm each
  side). A Nectar has the plates' 114.3 mm from s 0 (11.45 mm each side), and the two cheek crests centre it: left
  free it settles within about 1 mm of the centre path.
- A ball 20 mm off centre is pushed inward: a Pollen by a cheek's front corner, which it meets with its centre at
  y 19.25 to 20.25 (the contact's sideways component is 0.5), and a Nectar by a plate's front edge at (y 11.9, z 69),
  which it meets with its centre at about y −1 (0.81). Whether that Pollen is turned inward or rides up the corner
  is unknown. From y 58.9 the guide's face at x 2.9 pushes the Pollen onto V1.
- The intake's wheels touch a Pollen from s −99.5 to −3.0 and a Nectar from −124.5 to +18.5 at the drawn arm pose.
  With the arm 5° up a Pollen is touched from −97 to −17, 10° up from −93 to −36; a Nectar from −126.5 to +4.5 at
  10° up and from −117.5 to −38.5 at 20° up. The arm's rest angle is unknown, and it moves every number near the
  intake by 3 to 25 mm.
- The sweeper bar's drawn pose would block balls coming in (35 mm of overlap with a Pollen), so its working pose is
  unknown, and Basic Tele never commands its servo, `ruler` (code). Nothing in the transfer, the funnels or the
  wedges stops a ball at the front.

**Nectar-only stretch**

- From s 19.5 to 29.3 the Nectar's centre runs from (60.9, 81.5) to (67.1, 88.8): above the top of any Pollen
  (z 81.15) and short of where the front Nectar spokes first touch it (s 29.3). A Pollen passes directly underneath.
- A beam across through the Nectar's centre there meets solid side plate on both sides (y 60.5 to 66.8), 57.1 and
  57.2 mm from the centre. The colour spot past s 29.3 is in
  [Colour](#colour-the-color-rangefinder-on-the-x-guide-flange).

**Reversing the transfer motor** (CAD; how the balls move is inferred)

- Every driven surface reverses, all 1:1. Each track is single file, so balls come out last in, first out: the last
  Nectar in leaves first.
- Pollen: V1 (s 46.8 to 57.8) and V2 (s 97 to 110) push it toward the front. On the undriven ramp (s 111 to 280) it
  only rolls back by gravity. Above a closed paddle, where a Pollen gets only after a shot, the reversed Pollen
  spokes trap it on the paddle at s 319 to 325.
  A lone Pollen at s 0 to 46 is touched by nothing.
- Nectar: the spokes push the whole column down. A returning Nectar meets any Pollen centred at s 61 to 65 or less
  and shoves it out ahead of itself.
- The intake has its own motor. If it keeps running forward, the balls stay in: a Pollen bounces between the intake
  (contact to s −3) and V1 (from s 46.8), a Nectar between the intake (to s 18.5) and the front spokes (from 29.3).
  Reversed, it ejects at about 2.3 m/s. Stopped, its FLOAT wheels on a floating arm let balls push out under them.
- In Basic Tele, LT reverses the intake, the transfer motor and `melonbotics` together (`BioBuzzBasicTele.java`).

## What the CAD cannot tell

- Whether the servo's roller touches a ball (it misses by 0.9 mm as drawn), and whether the servo or a belt drives it:
  a 285 mm belt is drawn from that shaft to a motor-driven one. The pulley it cut through on 2026-11-14 was deleted
  the next morning, so the belt looks intended, but its other end still has no pulley drawn (CAD, 2026-11-15): ask
  hardware which of the two drives the roller. If the motor drives that roller, the ball there moves at
  1529 / 803 like the side rollers.
- The servo's voltage and loaded speed. Also: as placed, the servo body reaches through the drivetrain's inner plate
  and about 13 mm into the Control Hub's outline (inferred from the CAD's placement, good to a few mm). Is there a
  cut-out, or is it mounted elsewhere?
- The intake's loaded speed and resting height; how far the tubes bend; how the Nectar ball rolls in its trough.
- Which blocker paddles and servo exist; where the Nectar paddle is meant to sit; the Nectar turret's rotation range.
- Near the intake ([above](#the-intake-hand-off)): the sweeper bar's working pose, whether the floor plate's cheeks
  are solid, how a ball hops on the square 10 mm step at about 2.3 m/s, whether the foam tile sags, and whether a
  Pollen 20 mm off centre is turned inward by a cheek's corner or rides up it. The spokes aren't modelled: their
  42.7 mm tip radius is the team's figure and their planes are assumed.
- Ball mass and rolling resistance, which with the tubes' bending decide which stalls are real and which ball type
  leaves first when the transfer reverses.

The measurement [above](#the-measurement-that-settles-most-of-it) settles the first two, and the free or loaded
state.

## Rangefinder spots

### What to tell hardware

- **Spots A to E count only balls leaving toward the turrets.** For the possession count the sensors go at the
  intake end of each track, at the two spots in [Counting balls coming in](#counting-balls-coming-in): don't drill
  A to E for that count.
- **The case can't be screwed flat behind a plate** (vendor CAD). Its two M4 holes, 8 mm apart, run through the
  14 mm side, parallel to the face with the windows and 6.4 mm below it. The two 2 mm windows sit 2.5 mm off the
  case's centre along its 20 mm side and 3 mm apart along its 14 mm side. The plug (JST PH, side entry) comes out of
  one 14 mm end. With the 14 mm side along the ball path, the screws run along the path, so two cases side by side
  need long M4 screws through both, or a bracket.
- **Which plate.** The CAD study calls the sensor plate "the left plate". Find it by what is outside it: the plate
  with the Super Servo Plus mount bolted to its outer face, on the Control Hub's side (CAD). Go by the servo mount,
  not the word "left": the CAD's own frame puts robot left on the other side, so which side the study meant by
  "left" is unclear (inferred).
- **Pollen: spots A and B as drawn don't fit side by side.** Each body is 20 × 14 mm on the plate; with its 20 mm side
  across the ball's travel, 14 mm runs along it. A's and B's beam points are only 13.4 mm apart in a straight line,
  so with each beam at its body's centre the bodies overlap by about 0.6 mm (inferred). Before drilling, either ask
  CAD to move A earlier along the path (lower s, toward the intake) until the bodies clear, and recheck its cone, or
  use B with C instead (36.5 mm apart, fits; C sees a gap of only about 2 to 4 ms at free speed, so C tells only which
  way a ball moved). Place the hole on the windows (above), not on the case's centre. Use the narrow 15° field of
  view.
- **Bench-test one rangefinder at 15 to 45 mm, in SHORT at a 10 ms budget, first.** If it can't read that close, the
  sensor has to stand back from the plate on spacers. For ST's 40 mm minimum ([below](#the-sensor)) that
  is about 5 mm at A and B, 12 to 14 mm at C and 21 to 22 mm at D and E (40 minus the nearest face-to-ball distance;
  inferred). Each mm of stand-back puts the body 1 mm farther out behind the plate and widens the hole the cone
  needs (below). Behind the plate at A, the body is already only 4 to 5 mm from a standoff of the servo mount, and
  nobody has checked for 21 mm of room behind D and E: check in CAD before drilling.
- **Never in s 266 to 298**: at every beam spot tried there, the Pollen paddle swings through the cone (it crosses
  the path at s 264 to 294).
- **Nectar: spots D and E**, both on the same plate as A and B; the spot at s 150 only once the Nectar turret's
  rotation range is known. No Nectar spot passes at a 20 ms update. D and E are 16 mm apart along the path and
  18.1 mm in a straight line, so their bodies leave only about 2 to 4 mm between them: check the fit. The Nectar turret
  hits E's body if it turns to +108° to +114° (CAD).
- The two cones overlap on the far plate, so the CAD study advises firing the two sensors alternately (an inferred
  crosstalk risk) and never mounting two facing each other across the channel. In analog mode both range on their
  own, and neither Brushland's pages nor ST's datasheet give a way to make them take turns, so the bench test should
  also run two side by side.

### Where they go

Frame (CAD): the transfer's own, in mm. x runs across the channel, y toward the rear, z up. Each beam runs along x.
The sensor face sits on the outer face of its plate (x −92.75), looking through a hole drilled in the plate (x
−92.75 to −89.58, 3.2 mm thick); the far plate's inner face is at x 24.72, 117.5 mm from the sensor face. The channel
between the two plates' inner faces is 114.3 mm wide. Face to ball is the distance from the sensor face to the ball
along the beam (CAD).

To mark a hole on the plate, measure from the plate's front bottom corner: **back** = y − 46.9, how far behind the
side plate's front edge at its bottom (y 46.9, below z 49); **up** = z − 5.0, how far above the side plate's bottom
edge (the plates' bottoms are at z 5.05 and the tile's top at z 0.05; CAD). Higher up the front edge steps forward,
to y 40.9 from z 49 to 69 and y 11.9 from z 69 to 182 ([The intake hand-off](#the-intake-hand-off)), so measuring
from the edge at the hole's own height would put a hole at A to E about 35 mm off. A negative back is in front of
the bottom corner. Back is a straight y distance, not s. Both assume the edges are straight, which nobody checked.

| spot | s | (y, z) | back, up | beam vs ball centre line | face to ball | empty channel reads |
|---|---|---|---|---|---|---|
| A, Pollen | 166 | (206.9, 75.9) | 160.0, 70.9 | 30 mm above | 35–41 | far plate, 97–98% of a 15° cone |
| B, Pollen (best for one sensor) | 186 | (219.4, 80.6) | 172.5, 75.6 | 30 mm above | 35–41 | far plate, 97–98% of a 15° cone |
| C, Pollen partner for B | 226 | (255.5, 85.9) | 208.6, 80.9 | 15 mm above | about 28 | far plate |
| Pollen centre beam | 186 | (231.5, 53.1) | 184.6, 48.1 | through the centre | 24.7 | far plate, 100% |
| D, Nectar | 118 | (119.6, 168.3) | 72.7, 163.3 | 21 mm rear of the column centre | 18.5–19.8 | far plate 67%, a fixed trough at 91 mm 33% |
| E, Nectar | 134 | (120.3, 186.4) | 73.4, 181.4 | 21 mm rear | 18.5–19.8 | same as D |
| Nectar centre beams | 118, 134 | (98.7, 170.4), (99.3, 186.4) | 51.8, 165.4; 52.4, 181.4 | through the centre | 14.6 | far plate, 100% |
| Nectar, wider pair | 150 | (120.3, 202.4), other plate | 73.4, 197.4 | 21 mm rear | 18.5–19.8 | only once the Nectar turret's range is known: turret parts sweep through that body |

**The hole** (inferred): big enough for the sensor's emitter and receiver windows, which the pages read don't
place, plus the cone's spread through the 3.2 mm plate: about 0.8 mm more across at 15°, 1.5 mm at 27°. Standing
the sensor back adds about 0.26 mm across per mm at 15° (0.48 mm at 27°), because the cone is wider where it passes
the plate.

- A and B are 20 mm apart along the path, C 40 mm past B. Straight-line distances between the beam points, computed
  from the coordinates: A–B 13.4 mm, B–C 36.5 mm, D–E 18.1 mm (D and E are 16 mm apart along s). A–B is shorter than
  its 20 mm because the Pollen path bends there and those offset beams sit on the inside of the bend (inferred).
  D–E is longer than its 16 mm, which a straight vertical column can't give: D's offset from its centre beam is
  (20.9, −2.1) where E's is (21.0, 0), so D's z looks about 2 mm low. Recheck D in CAD.
- A and B: nothing moving is in a 15° cone. At the 27° setting a compliant wheel takes 1.6% of A's cone and 3.0% of
  B's, so use 15°. A is 74.5 mm and B 54.5 mm before the ball held by a closed Pollen paddle (s 240.55), 256.8 and
  236.8 mm before the turret. Body clearance: A is 4 to 5 mm from a servo-mount standoff, B 10 to 14 mm from anything.
- **A to E cannot count balls coming in while a paddle holds them** (CAD queue positions; distances from the CAD
  path, inferred). With the Pollen paddle closed the queue's centres are s 240.55, 168.6, 97.3 and 26.3, and a fifth
  ball stops at s −46.5, under the intake. The first ball stops on C (20.3 mm from its beam). The second stops on both
  A and B: its centre is 30.1 mm from A's beam and 33.9 mm from B's, both inside its 35.55 mm radius. The third
  (69.8 mm from A) and fourth never reach A. `TransferCounter` counts only when A clears while B is still covered, so
  the second ball is never counted. The Nectar paddle can't close as drawn, so the only Nectar column is the open
  one, at s 202.3, 110.5 and 17.7 ([above](#path-capacity-and-the-blocker)): the second ball covers D and E (its
  centre 22.2 and 32.4 mm from their beams, inside its 45.7 mm radius) and the third never reaches D (106.7 mm
  away). So spots on the tracks suit counting balls as they leave toward the turrets, not as they come in.
- A and B are on the undriven ramp: a lone Pollen at loaded speed stalls at about s 210 to 220, past both, and rolls
  back to s 161.5 or less, so use the pair to tell direction (inferred).
- D and E: the spokes are 40 mm clear. D is 25.3 mm before where a closed Nectar paddle would hold a ball (s 143.3),
  but that paddle can't close as drawn; the turret is at s 202.
- Rejected (CAD): s 266 to 298 (the Pollen paddle swings through the beam), s 306 (2 mm from the paddle and the tubes),
  s 394, and an earlier pair at s 282, 290 and 306.

### Ball and gap times, and which sampling rate works

On a straight stretch, a beam h mm off the ball's centre line crosses a chord of 2 × √(R² − h²) of each ball, and
the gap it sees between touching balls is D minus that chord. Ball time = chord ÷ v; gap time = (gap + any gap a
hand-off opened) ÷ v. Where the path bends, the CAD path itself gives the numbers (inferred: a touching train moved
along the CAD path, each beam covered while a ball's centre is within its radius of the beam):

- Pollen at A and B (h 30, R 35.55) sit on the inside of the ramp's bend. At A a ball covers the beam for 48.6 mm of
  travel and the gap is 24.3 mm; at B 43.6 mm and 28.4 mm. A straight stretch at h 30 would give 38.1 and 33.0.
- Pollen at C (h 15): a gap of only 3.6 mm (6.6 on a straight stretch): about 2 ms at free speed, so C tells only
  direction.
- A centre beam: chord = D, so touching balls show no gap at all.
- Nectar at D and E (h 21, R 45.7), on the outside of the bend: a gap of 14.6 mm at D and 12.5 mm at E (10.2 on a
  straight stretch), plus the ramp-to-spokes gap when free.

**The beam is a square pyramid, not a line** (inferred; not checked in CAD or on a sensor). ST gives the 15° and 27°
as the diagonal of a square field of view (vendor). At 15° it is 2 × distance × tan 7.5° across the diagonal: about
11 mm where it meets the middle of a Pollen ball at A or B (41 mm out) and 16 mm where the beam meets the ball's
edge (60 mm out, the chord's ends). Across the flats it is about 0.71 of that (about 11 mm at 60 mm). Below, "cone"
means this field. The pages read don't say how much of the cone a ball must fill before
the reading changes, so a ball can look up to about one cone width longer, and the gap that much shorter, than the
chord says. Even with a ball centred on A or B, 14% of the cone misses it (CAD). The "with the cone" column below is
that worst case; the bench test with a ball moved at a known speed shows where the truth lies.

**The rule** (the CAD study's, inferred): a sensor sees a ball reliably when the ball and the gap each last at least
two sensor updates, so one bad reading can't make or hide a count: 40 ms each at a 20 ms update, 20 ms at 10 ms.
So the train must move at no more than gap ÷ (2 × update), on a thin beam: at A 608 mm/s at 20 ms and 1215 mm/s at
10 ms; at B 710 and 1420. (For ball + gap together, the limits are D ÷ 80 ms and D ÷ 40 ms: 889 and 1778 mm/s
Pollen, 1143 and 2285 Nectar.) This rule assumes a new distance every update; the chain from sensor to code
([The sensor](#the-sensor)) makes it optimistic.

| Pollen, A / B | speed (mm/s) | ball / gap (ms) | gap with the cone (ms) | 20 ms updates | 10 ms updates |
|---|---|---|---|---|---|
| side rollers set it, free | 1529 | A 31.8 / 15.9, B 28.5 / 18.6 | A 5–9, B 8–11 | fails | fails |
| motor loaded | 803 | A 60.5 / 30.3, B 54.3 / 35.4 | A 10–17, B 15–22 | fails | passes; with the cone fails at A, at best just passes at B |
| servo meters at 833 rpm | 1107 | A 43.9 / 22.0, B 39.4 / 25.7 | A 8–12, B 11–16 | fails | passes; fails with the cone |
| servo meters at 1000 rpm | 1329 | A 36.6 / 18.3, B 32.8 / 21.4 | A 6–10, B 9–13 | fails | fails at A, just passes at B; fails with the cone |

20 ms passes on a thin beam only if the servo meters at 457 rpm or less at A, 534 at B (gap ÷ 40 ms × 2 ÷
(2π × 25.4) × 60).

| D / E, Nectar | ball / gap (ms) | gap with the cone (ms) | 20 ms | 10 ms |
|---|---|---|---|---|
| free, ball at 0.5 × tip (2571) | D 30.1 / 26.9, E 30.8 / 26.1 | 20–25 | fails | passes; at best just passes with the cone |
| free, ball at 0.363 × tip (1867) | D 41.4 / 15.9, E 42.4 / 14.7 | 6–13 | fails | fails |
| loaded (1350 to 980) | 57 to 81 / 9 to 15 | less | fails | fails |

For Nectar the cone is about 5 mm across at the ball's middle (20 mm out) and 16 mm at the chord's ends (60 mm out).
At 10 ms, Nectar passes on a thin beam only if the ball rolls at 0.41 × the tip speed or more at D (2089 mm/s),
0.42 at E (2146 mm/s).

### The sensor

From Brushland's pages ([Parts](parts-findings.md#brushland-laser-rangefinder)): 20 × 14 × 10 mm, 15° to 27° field
of view, up to 4 m, "up to 100Hz", M4 holes 8 mm apart; SHORT mode at a 10 ms budget gives 100 Hz (vendor).

- **The chip is an ST VL53L1X.** Brushland's own CAD model of the sensor names the part `VL53L1CXV0FY_1` and a
  "VL53L1 Module Outline" (vendor); VL53L1CXV0FY/1 is the VL53L1X's order code in ST's datasheet. That datasheet
  (vendor) says: "Up to 50 Hz ranging frequency"; "20 ms is the minimum timing budget and can be used only in short
  distance mode"; "The minimum ranging distance ensured is 4 cm. Under this minimum distance, the sensor detects a
  target, but the measurement is not accurate"; and the 15° to 27° is a diagonal field of view. So Brushland's 10 ms
  budget at 100 Hz goes beyond what the chip's maker rates, every spot on this page is inside the 4 cm, and both
  hold only if the bench test shows them. Brushland also says smaller regions (the 4 × 4 that gives 15°) are noisier
  and need a longer budget.
- Every spot on this page is 15 to 44 mm from the ball, so bench-test at 15 to 45 mm in SHORT at 10 ms before
  drilling, and leave room for spacers. To try 10 ms: Configure Laser Rangefinder with `timingBudgetMs` 10 (its
  default is 20).
- **No sensor update rate on this page has been measured.** Every bench run so far had the lasers at 20 ms
  ([`hub-bench-findings.md`](hub-bench-findings.md)). "A reading every 10 ms" is the poller's rate, not a new
  distance every 10 ms. Three clocks run one after another, each on its own: the sensor's update, the hub's ADC
  refresh (every 9.7 to 11.1 ms, measured) and the poll (10 ms). So the code can learn of a change up to about
  31 ms after it happens at a 10 ms budget, 41 ms at 20 ms, plus the bulk read and the board's own delay to its
  analog output, which Brushland doesn't publish; and when the ADC refreshes slower than a 10 ms sensor, up to about
  10% of the sensor's updates are never sampled (inferred: 1 − 10 ÷ 11.1).
- **The timing on this page assumes the 10 ms poller thread.** [`wiring-findings.md`](wiring-findings.md)'s default
  reads the rangefinders in the loop instead, at 34 to 37 ms a loop, which is slower than any gap on this page
  (inferred): counting balls needs the poller.
- In analog mode the sensor's range status (no target, weak signal) never reaches the hub; only I2C reads it
  (vendor).
- **The Color Rangefinder's range fits, but its speed and its colour dependence are unknown** (inferred, untested).
  Its distance mode tops out at 100 mm (vendor), the far plate is 117.5 mm from the sensor face, so an empty channel
  reads beyond its range, and at A and B the ball is 35 to 41 mm away. But it measures the strength of reflected
  light, not time of flight, so a yellow, a red and a blue ball may read different distances; its chip repeats a
  distance measurement every 6.25 to 400 ms depending on a setting the vendor doesn't publish, and a colour
  measurement every 25 ms at best ([Parts](parts-findings.md#brushland-color-rangefinder)). By the two-reading rule
  above, a free-running Pollen train at A or B (gap 15.9 to 18.6 ms) needs a new value every 8 to 9 ms: only the
  6.25 ms setting would do. Hue every 25 ms gives a ball passing in 28 to 32 ms one reading at most. Neither sensor's
  rate is settled until it is measured: Brushland's 10 ms for the Laser Rangefinder exceeds its chip's rating.

### Counting balls coming in

**What the team needs** (team, 2026-11-15): the number of balls of either type that have entered the robot, to stay
within the possession limit, and each Nectar ball's colour, to get rid of an opponent's Nectar before it is shot.

**What the rules say** (manual):

- G407: "A ROBOT may not simultaneously CONTROL more than 4 SCORING ELEMENTS." A verbal warning; a major foul and a
  yellow card per match if strategic. A robot that "MOMENTARILY CONTROLS 5 SCORING ELEMENTS which they 'reverse'
  quickly" is likely not strategic; "REPEATED instances" of 5 or more likely are, "even if the ROBOT has a 'keep 4 and
  discard the rest' sorting mechanism". MOMENTARY is "fewer than approximately 3 seconds".
- CONTROL includes a ball "fully supported by or stuck in, on, or under the ROBOT" and herding. So a ball pinched
  under the intake already counts, before any transfer sensor sees it.
- G408: "A ROBOT may not CONTROL the opponent's NECTAR", and repeated instances are likely strategic "even if the
  ROBOT has a sorting mechanism that automatically discards opponent NECTAR". Taking one in and spitting it out is
  the pattern the rule names.

So (inferred): an undercount is the costly error, and an overcount only stops the robot early, so every counter
should err toward counting too many. Stop the intake at a count of 4, or as soon as either track reads full
(below); if a 5th ball may be in, reverse within well under 3 s, which also puts out a 3rd Nectar resting at s 17.7.
With balls arriving touching, a ball can be under the intake before any sensor sees it: with the Pollen paddle
closed a fifth Pollen stops at s −46.5, inside the intake's reach (s −99.5 to −3.0). And once 3 Nectars are in, the
third resting at s 17.7, a 4th ball stops at s −58.5 (Pollen) or −81.0 (Nectar), under the intake and in front of
every beam on this page (CAD). That ball is CONTROLLED, so the count must already read 4 when the third Nectar comes
to rest: stopping at 4 balls seen would stop one ball late. With a ball pinched under a stopped intake, no beam
anywhere in s −100 to +80 is clear of every ball (CAD). Keep opponent Nectar out with the camera and the driver; the
colour sensor is the backstop. Balls that are shot pass no intake sensor, so the code must subtract shots.

#### The plan

One beam per track, each where only its own ball type passes, and the Color Rangefinder for colour: the team's two
Laser Rangefinders and one Color Rangefinder. The spots were worked out for this page from the CAD's paths and
meshes (inferred); the CAD itself checked neither counting spot, and nothing has been tested.

| beam | (y, z) | back, up | covered at s: Pollen / Nectar | what it sees | to make |
|---|---|---|---|---|---|
| P66, Pollen counter | (66, 16) | 19.1, 11.0 | −0.9 to 38.8 / never | every Pollen; never a Nectar (at most 9% of its cone) | a 3.2 mm hole through the −x plate and a tunnel through the −x cheek (about 20 mm; 23.1 mm with the plate), wide enough for the cone all the way; the laser body outside the −x plate, its 14 mm side along y |
| N20, Nectar counter | (33.0, 106.6) | −13.9, 101.6 | never / −10.2 to 50.1 | every Nectar; never a Pollen. The beam is square to the Nectar's path at s 20, 37.5 mm in front of and above its centre | a 3.2 mm hole through the −x plate, solid there (21.1 mm behind its front edge at that height) |
| Color Rangefinder | C1: (64.6, 86.4); or (68.7, 91.1) to (70.5, 93.5) | C1: 17.7, 81.4; or 21.8, 86.1 to 23.6, 88.5 | never / C1: −20.1 to 71.7; s 32 to 35: −13.8 to 80.9 | every Nectar, its beam through the ball's centre at s 25 (C1) or s 32 to 35 ([Colour](#colour-the-color-rangefinder-on-the-x-guide-flange)) | a hole through the +x guide's flange and plate |

"Covered at s" is the stretch of a ball's own path, in s, over which the ball cuts the beam's axis (inferred from the
CAD's paths); a cone stays covered a little longer.

- **Count +1 on each clear-to-covered edge. While the intake runs in reverse with the transfer stopped or reversed,
  count −1 on each covered-to-clear edge instead, and nothing on clear-to-covered.** That takes off a ball that was
  resting in the beam when reversing started, such as the 3rd Nectar, as well as one that passes back through. With
  the intake still forward, a ball bounces between the intake and the transfer's first driven surface, across the
  beam each way ([Reversing the transfer motor](#the-intake-hand-off)), so subtracting then would undercount. A ball
  that leaves from in front of every beam is never taken off, which leaves the count high, the safe error. Basic
  Tele's LT reverses the intake and the transfer together.
- **Treat a beam that stays covered while intaking as "that track is full": count at least 4 and stop the intake.**
  Reverse only for a suspected 5th ball; reversing the intake also puts out the resting 3rd Nectar. The resting 4th
  Pollen (paddle closed, s 26.3) covers P66, and the resting 3rd Nectar (s 17.7) covers N20 and the Color
  Rangefinder. "Stays covered" means longer than a passing ball takes; set it on the bench. Either resting ball closes
  the other track too: the 3rd Nectar blocks the Pollen path, and the 4th Pollen the Nectar path (CAD).
- **In geometry this counts every mix of up to 4 balls right**, in any order, with the Pollen paddle open or closed,
  including 3 Nectars then a 4th ball, which stops under the intake. The one error is reading 4 with only 3 Nectars
  in, when no further ball can reach the transfer anyway (inferred: every arrival order played through the CAD's
  rest positions).
- **That holds only while the Nectar paddle can't close.** If hardware makes it close, it holds the 1st Nectar
  at s 143.3 and the 2nd at s 50.6, just past where it cuts N20's axis (it fills 46% of N20's cone), and the next
  ball stops unseen in front of both beams, under the intake: a Pollen at about s −17.5 or a Nectar at −45.1 (CAD).
  Played through every order, 10 of the 30 mixes of up to 4 balls then read 1 or 2 low (inferred). The plan would
  then have to change, for example by taking "Nectar full" from the Color Rangefinder, whose beam the resting 2nd
  Nectar still cuts, and reading 4 from then on.
- **P66, more.** Touching Pollen leave 31.5 to 33.5 mm of travel clear at the beam (19.3 at spot 1, below). Empty, 82%
  of a 15° cone ends on the floor plate at 46 to 108 mm, 15% on the far plate at 118 mm and 4% on the drivetrain at
  204 mm; only 3% of rays end within 50 mm, against a ball 35.2 mm from the face. That needs the 15° field: at 27°
  the floor plate is in view from 25 mm and 19% of rays end within 50 mm, nearer than the ball, and Brushland says
  the 15° region is noisier and needs a longer budget (vendor). The intake doesn't enter the cone anywhere from 0° to
  49° (inferred: cone cast on the CAD meshes, tunnel drilled wide). Keep the beam at about z 16: at z 18 the laser
  body comes within 1.1 mm of the side-roller mount. The windows sit 2.5 mm off the case's centre along its 20 mm
  side, which is vertical here, so with the body above the beam it is 0.6 mm from that mount, and below it 3.45 mm
  above the tile: check which way it goes and where the plug comes out (inferred, CAD meshes). P66 lies where a lone
  Pollen can stop untouched (s −0.5 to 46); one that stops on the beam reads as "Pollen full" and stops intaking
  early, the safe error.
- **N20, more.** Touching Nectars leave 30 to 35 mm of travel clear. Empty, a 15° cone ends 100% on the far plate at
  117 to 118 mm with the intake at 0°, 15°, 30° and 49° (inferred, cone cast on the CAD meshes). The ball comes
  29.7 mm from the face. The front spokes' disc passes 15.1 mm from the axis, against a 15° cone whose radius is 5.8
  to 10.0 mm at the disc's planes (10.6 to 18.3 mm at 27°, so a 27° field may see the spokes), but the spokes aren't
  modelled. Nobody has checked the sweeper bar against this beam. A second spot at (37.7, 108.3), back −9.2, up 103.3
  (Nectar s 25, 35 mm off centre), also fits, but touching Nectars leave only 20.5 to 25.5 mm there.
- **Speed is the weak point** (simulated). At 2.3 m/s P66 sees a ball for 17.1 ms and the gap for 13.7 ms, N20
  26.3 ms and 13.0 ms, so no gap lasts two 10 ms readings
  ([the rule](#ball-and-gap-times-and-which-sampling-rate-works)). Share of runs that counted 4 touching balls as 4,
  by speed in m/s:

| beam and reader | 1.53 | 2.3 | 2.8 | 3.1 | 3.5 |
|---|---|---|---|---|---|
| P66, laser at a 10 ms update | 100% | 87% | 78% | 75% | 62% |
| P66, laser at a 20 ms update | 100% | 0% | 0% | 0% | 0% |
| P66, break-beam read every 10 ms | 100% | 100% | 100% | 100% | 85% |
| P66, break-beam read every 5 ms | 100% | 100% | 100% | 100% | 100% |
| N20, laser at a 10 ms update | 100% | 92% | 89% | 86% | 62% |
| N20, laser at a 20 ms update | 100% | 65% | 0% | 0% | 0% |
| N20, break-beam read every 10 ms | 100% | 100% | 100% | 93% | 71% |
| N20, break-beam read every 5 ms | 100% | 100% | 100% | 100% | 100% |

- 2.3 m/s is the intake's free speed with the robot still, 3.1 to 3.7 m/s with it driving at 1.85 m/s (the table stops
  at 3.5), and 1.53 m/s a touching Pollen train the side rollers set (speeds inferred, below). Only 3 Nectars fit, so
  the N20 rows are conservative. Model: covered travel from the CAD paths, a part-filled cone counted as covered from
  50%; a laser takes a snapshot every 10 or 20 ms at a random phase, read through the hub's ADC (refreshing every 9.7 to
  11.1 ms, measured) by a 10 ms poll; a break-beam is a thin beam whose state is read every 5 or 10 ms.
- **So use break-beams if they are legal** and a 5 to 10 ms poll reads fresh digital values; otherwise the two
  lasers at a 10 ms budget in the same spots. A break-beam pair needs the same tunnel through the +x cheek and plate
  at P66, and a hole through the +x plate at N20; Adafruit 2167 halves fit on both sides at both spots (inferred). If
  break-beams carry the count, the lasers can still give the "track full" signals.
- **Speeds at the intake** (inferred, no slip assumed): on the tile a pinched ball moves at 2.2 to 2.8 m/s with the
  robot standing still. Driving adds about 0.47 × the robot's speed (0.465 Pollen, 0.481 Nectar, with the wheel above
  the ball): at 1.85 m/s, the only drive speed in the repo (`BioBuzzConstants`, still the Betta bot's value), that is
  about 3.1 to 3.7 m/s. On the plate and the ramp most of that gain is lost. All intake-derived speeds scale with the
  intake's real loaded speed, which is unknown, and with battery voltage, since the intake runs at power 1.0 with no
  voltage compensation. The side rollers' 1529 / 803 mm/s come from the transfer motor and don't change with driving.

#### Spots considered

| spot | (y, z) | back, up | covered at s: Pollen / Nectar | verdict |
|---|---|---|---|---|
| 1, low over the floor plate's edge (the CAD's best counter) | (51, 16) | 4.1, 11.0 | −28.2 to 23.8 / −39.0 to −13.7 | fallback only: sees both types, the resting 4th Pollen fills 39% of its cone, part of its empty view returns at the ball's distance, and it never sees a 4th ball stopped behind 3 Nectars |
| 4, high, just behind the plate's edge | (45, 76) | −1.9, 71.0 | −13.0 to 16.5 / −44.6 to 50.6 | rejected: sees both types; touching Nectars leave 1.2 mm; the resting 3rd Nectar fills its whole cone |
| 5, partner for 4 | (61.5, 76) | 14.6, 71.0 | | rejected: starts inside the side-roller mount |
| 2, low on the tile | y 34.5 | | | rejected: nothing to fix it to (the body cuts the funnel); 34 to 41% of the cone sees the tile and up to 15% leaves the robot |
| 3, low, from the drivetrain's inner plate | y 24 | | | rejected: 39% of the rays leave the robot |
| 6, 66 mm up, in front of the plates | z 66 | | | rejected: the intake is in the cone and touches the body |
| 7, looking down from a new cross bracket | y 10 to 30, face at z 108 | | | rejected: every Nectar runs into the body (by up to 5.1 mm, over s −5.4 to 43.1); moved to y 0 to 20 it clears a Nectar by 2.2 mm, but the intake wheel's rim reaches it at 15° to 25° of lift |
| 8, looking up through the floor plate | | | | rejected: the ball rolls on the sensor's face, and there is no room under the 5 mm plate |
| any pair 16 to 17 mm apart, low or high | | | | none fits |

The verdicts are CAD; back and up are worked out from (y, z), and the covered ranges from the CAD's paths, as in
[The plan](#the-plan).

- **Spot 1** (CAD): the beam runs 6 mm above the plate top. The body sits outside the −x plate behind the funnel's
  flange, at x −106 to −96, y 46 to 60, z 6 to 26, which clears the funnel but, with the windows 3 mm apart about the
  centre of its 14 mm side, puts the beam at about y 53; the figures here are for y 51. The +x mirror still touches the
  other funnel. The beam needs a 26.4 mm tunnel: funnel flange 3.2, plate 3.2 and cheek 20; whether the cheek is solid
  and whether the flange may be drilled are unknown. Its axis is 43.6 mm from a Pollen and 43.8 mm from a Nectar, and
  ends on the far cheek at 100.8 mm. 15.2% of a 15° cone (19.4% at 27°) ends on the plate top or the cheek's fillet at
  36 to 44 mm, the same range as the ball; 25.8% runs past the cheek to the far funnel at 124 to 208 mm. A Pollen covers
  the axis for 51.8 mm of travel and touching Pollen leave 19.3 mm, 8.4 ms at 2.3 m/s; a Nectar only grazes it, 4.5 mm
  deep for 25.4 mm, as it lifts away up the step. The resting 4th Pollen (s 26.3) clears the axis by 1.5 mm but fills
  39% of the 15° cone at 48 to 66 mm, so with 3 Pollen inside it may read "ball" all the time; how the sensor reads a
  part-filled field is unknown. Also in the beam at rest: the 7th Pollen with the paddles open, a Nectar stopped behind
  a Pollen at s 61, any ball under a stopped intake, and a lone Pollen coasting to a stop in s −0.5 to 46 (inferred).
  The intake and the sweeper clear it. As an all-ball counter it got 4 touching Pollen right in 74% of runs at 2.3 m/s
  with a laser at a 10 ms update (simulated).
- **Spot 4** (CAD): a 3.2 mm hole through the −x plate; the body fits. 92% of the 15° cone ends on the far plate at
  117.5 mm. A Nectar passes 15 mm from the face, right through its centre, and covers the beam for 95 mm of travel;
  a Pollen covers it for 29.3 mm, 5 mm deep. That is about 13 ms for a Pollen and 42 to 59 ms for a Nectar at the
  intake's free speed (inferred).
- **Spots 1 and 4 together** see every ball of a 4-ball mix once each, 4 edges apiece, so read as a Pollen and a
  Nectar counter they count most balls twice (inferred).
- **A beam pair 30 mm above the Pollen centre line**, at about z 76, can't count Nectars: touching Nectars leave at
  most 9.1 mm there (chord 82.3 mm), and only 1.2 mm at spot 4. Read as covered or clear by two beams 17 mm apart, a
  touching Nectar train counts as one ball (inferred, and simulated below).

#### Colour: the Color Rangefinder on the +x guide flange

Put it on the +x side, on the guide's flange, its beam through the Nectar's centre, at the CAD's spot C1 or 7 to
10 mm further along the Nectar path with the same mount: C1 is s 25 at (64.6, 86.4), back 17.7, up 81.4; s 32 is at
(68.7, 91.1), back 21.8, up 86.1; s 35 at (70.5, 93.5), back 23.6, up 88.5 (the CAD checked C1 only; s 32 to 35 is
inferred from the CAD). They trade. C1 gets more passing readings (62 and 83% against 55 and 80% at s 32, table
below), the colour sooner (the ball at s 48 to 117 against 54 to 128 for s 35) and a nearer resting 3rd Nectar
(20.1 to 20.9 mm against 21.3 to 23.2 at s 32). But a passing Pollen fills up to 15% of C1's 15° cone from 54.7 mm,
where at s 32 to 35 it fills none. Step 5 under
[What would settle it](#other-counting-options-and-what-would-settle-it) decides.

- **Mount.** The side-roller mount fills the −x side there. On the +x side the case's face sits on the outer face of the
  guide's flange at x 32.9, looking −x: drill the flange (5.0 mm) and the +x plate (3.2 mm) where the plate is there,
  since the plate's opening starts at y 69 below z 92, right at this spot. For a 6 × 8 mm window the hole needs to be
  12.2 mm across for a 15° field, 13.9 mm for 27° and 16.0 mm for 40° (inferred). Nothing comes within 8 mm of the case
  from s 19.5 to 35, either way up; with its 23 mm side along y it spans z 85.5 to 101.5 at s 35, where the flange tops
  out at about z 99.1, so take s 32 if the case must sit wholly on the flange. From s 38, with the 23 mm side vertical,
  the intake motor comes within 6 to 7.5 mm. Nobody has checked that the flange is solid behind the case (inferred, CAD
  meshes at their drawn poses).
- **Distance.** A passing Nectar is 19.65 mm from the face, at the edge of Brushland's "accurate down to 0.1mm
  within the first 20mm" (vendor); the resting 3rd Nectar 21 to 25 mm. Empty, the nearest return is 121.9 to
  122.5 mm for a 15° cone and about 116 mm for 27°, beyond the 100 mm range (vendor), so it should read nothing in
  range; what the output reads with no target is unpublished. At 27°, 2% of rays end 8.4 mm out, just past the
  8.2 mm-deep hole, inside the 100 mm range, and 4 to 6% (C1 to s 35) end on a turning bevel gear's clamping pin at
  119 to 146 mm, beyond it (inferred, CAD meshes).
  Some rays reach robot parts and the field through the plates' windows, so never trust hue without a distance gate.
- **Pollen.** A passing Pollen fills none of a 15° cone at s 32 or 35, 6 to 12% of a 27° one and 16 to 20% of a
  40° one. At C1 it is in up to 15% of a 15° cone (27% at 27°) from 54.7 mm (CAD), since there the beam passes only
  4.4 to 5.2 mm above the Pollen's top, against 9.9 mm at s 32 and 12.3 mm at s 35. Brushland doesn't publish the
  field of view, so these use the Laser Rangefinder's cone angles, and 40°.
- **The last Nectar in, at rest.** The 3rd Nectar (s 17.7) fills 100% of the cone at every angle tried, with nothing
  else in view: a still read of its colour, and a second "Nectar full" signal. With 1 or 2 Nectars in, nothing rests
  in the beam. The 3rd stays at s 17.7 only while the arm rests within about 1° of the drawn pose, and higher it
  settles nearer the intake (about s 12 at 5° up, s 3 to 4.5 at 10°, s −9 at 15°; inferred from the CAD's contacts).
  The beam still sees it fully with its centre anywhere from s −14 for C1, −7 for s 32 or −4 for s 35, so at s 32
  to 35 up to about 10° of arm lift, and at C1 15° as well (inferred).
- **Passing Nectars.** Share of runs with at least one 25 ms colour reading taken wholly while the ball fills 95% or
  more of a 15° cone (simulated, speeds inferred):

| spot | free, ball at 0.5 × spoke tip | free, ball at 0.363 × spoke tip | arm lags (coasting at 1.2 m/s), or loaded |
|---|---|---|---|
| s 32 | 55% | 80% | 100% |
| s 35 | 52% | 79% | 100% |
| C1, s 25 | 62% | 83% | 100% |
| C2, s 60 | 31% | 73% | 80% coasting, 100% loaded |

- At the chip's 100 ms default no reading falls wholly on a passing ball at any spot, so passing reads need the
  25 ms rate, which Brushland's "Color resolution: 16-bit" only suggests
  ([Parts](parts-findings.md#brushland-color-rangefinder)). The code learns the hue after the first whole reading,
  the hub's ADC refresh (up to 11.1 ms), the 10 ms poll and the board's unpublished delay; by then the ball's centre
  is at s 54 to 128 for the s 35 spot, 48 to 117 for C1 and 69 to 153 for C2. So the 1st and 2nd Nectar's colour
  arrives after the ball is in the spokes (inferred). If the intake still drives the ball there (1570 to
  1610 mm/s), it passes C1 in 57 to 58 ms (CAD). If the intake arm can't drop fast enough to follow the ball, it
  coasts at about 1.2 to 1.5 m/s instead (inferred from the arm's length and gravity alone; its mass and any spring
  are unknown). Neither speed is measured.
- **Mixed views.** While a Nectar fills 95% of the cone at s 35, a Pollen covers at most 1% of it (4% at 40°). A
  Pollen alone reads yellow or empty (86° and 93° on the one bench reading of each), outside the red 10 to 50° and
  blue 170 to 210° ranges, and a red or blue Nectar mixed with a Pollen most likely lands between ranges and reads
  "unknown" rather than the wrong colour (inferred, not measured).
- **Its distance output as a Nectar counter.** With hue on pin 0, pin 1's digital "within about 30 mm" marks Nectar
  arrivals: two touching Nectars show 26 to 44 ms with less than half the cone within 30 mm at s 35. That is enough
  at the chip's 6.25 or 12.5 ms proximity rates, marginal at 25 ms and too slow at its 100 ms default; the real rate
  is unpublished. A lone Pollen never comes nearer than 51 mm there. It needs the splitter cable, and it never sees a
  Pollen or a ball under the intake, so it is a cross-check for N20 only (inferred).
- **C2, a third option** (CAD): the −x plate at s 60, beam (83.4, 114.9), back 36.5, up 109.9, through a 3.2 mm
  hole. No Pollen is ever in view; 87 to 89% of rays end on the far plate at 117.5 mm; a passing Nectar is 14.7 mm
  from the face; the spokes clear by 24.5 mm (15°) and 15.5 mm (27°); the body clears its neighbours by 3.9 to
  5.4 mm. But the resting 3rd Nectar cuts its axis by 3.6 mm and fills 74% of the 15° cone, and the 2nd fills another
  15% (inferred), so a still read mixes two Nectars' colours. The axis clears both resting Nectars only at s 63.4 to
  64.8, (85.0, 117.9) to (85.5, 119.2), back 38.1 to 38.6, up 112.9 to 114.2, where the cone still takes 43 to 45%
  of each (inferred) and nobody has checked the body's fit. It also gives the fewest passing reads and the latest
  verdict.

#### Putting a wrong-colour Nectar back out

Shooting is first in, first out, so reading colours anywhere every Nectar passes gives their order. Reversing the
transfer motor also moves the Pollen, since every transfer shaft turns 1:1 off it
([Reversing the transfer motor](#the-intake-hand-off)). Sequences (inferred from the CAD's contacts and the speeds
above; nothing tested):

- **The wrong Nectar is the 3rd, at rest at s 17.7 and read still.** Keep the transfer stopped and reverse only the
  intake, for at least 64 to 89 ms (s 18.5 to −124.5 at 2238 or 1610 mm/s). Only balls under the intake in front of
  it leave with it (a Pollen stopped at s −58.5 or a 4th Nectar at −81.0); nothing in the transfer moves, and no
  Pollen can be at s 0 to 72 then. It needs the intake wheels to reach the ball: at the drawn arm pose they touch a
  Nectar up to s 18.5, with the arm 10° up only to s 4.5. N20 sees the Nectar leave, which takes it off the count
  ([The plan](#the-plan)).
- **It is the 1st or 2nd Nectar and was the last ball in.** Reverse the transfer and the intake together, as LT
  does, stop the transfer as soon as the colour sensor sees that Nectar pass back down, and keep the intake reversed
  for at least another 64 to 89 ms. Down to s 18.5 takes 36 to 49 ms free (68 to 94 loaded) from the 2nd's rest at
  s 110.5, and 71 to 98 ms free (136 to 188 loaded) from the 1st's at 202.3. Any Pollen that V1 or V2 touches is
  driven toward the front: with the paddle closed the 3rd Pollen (s 97.3) likely ends in the untouched stretch at
  s 0 to 46, where the returning Nectar shoves it out ahead of itself.
- **Another Nectar came in after it**, the usual case with touching arrivals: the next arrives 57 ms or more later,
  and the verdict comes later than that. Both come out, the good one first, plus any Pollen above; then take the good
  one in again. To avoid it, pause the intake after each Nectar edge until its hue is known, at a cost of about one
  hue reading per Nectar. A ball coming back passes the sensor again: a second read of one that read "unknown".
- **Rules.** Each ejection takes tenths of a second, well inside G407's MOMENTARY, but G408 has no momentary allowance
  and repeated instances are likely strategic, so ejecting is a rare backstop: alert the driver each time. A Pollen
  that leaves unseen leaves the count too high, the safe error.

#### Other counting options and what would settle it

**Alternatives worth knowing:**

- **Spot 1 as an all-ball counter**, if P66 can't be built: weaker (above), it needs a 20 mm tunnel through the same
  cheek, and it needs the Color Rangefinder's "Nectar full" signal to count a 4th ball stopped behind 3 Nectars.
- **A two-laser direction pair, if used anyway:** read as covered or clear, the spacing must be below both the gap
  and the chord the beams see (the next ball reaching A before the last one leaves B reads as backward), and half the
  smaller of the two makes the shortest step longest. `TransferCounter` is not such a reader: it counts when A clears
  while B is still covered, can't tell direction, and counts a ball going backward as another ball. It does best with
  the beams about as far apart as the gap (34 to 38 mm for a beam 5 to 6 mm up; simulated). No such pair fits at
  z 16: a beam that far in front of spot 1 lands on the tile or in the funnel mouth (spots 2 and 3), and one that far
  behind it is where the resting 4th Pollen covers it, which a beam at z 16 escapes only at y 52.3 or less (inferred).
- **Digital outputs.** The Laser Rangefinder can output a distance threshold as a digital pin (vendor). The Control
  Hub's 8 digital ports are all unused in `biobuzz.xml`, and a digital pin skips the hub's 10 ms ADC refresh
  (inferred). How often the hub samples digital inputs is unmeasured.
- **Break-beam sensors** on Control Hub digital ports: Adafruit's 2167 lists "Response Time: <2 ms", "20mm x 10mm x
  8mm", a sensing distance of "Approx 25cm", "3.3 - 5.5VDC" and an "open collector transistor output" (vendor), so
  it needs a pull-up; whether a hub digital port gives one is unchecked, and so is whether the robot rules allow it.
  A thin on/off beam has no plate-top return and no minimum range. The halves fit at P66 and N20 but hit the funnels
  at spot 1 (inferred). How fresh a digital read is at a 5 ms poll is unmeasured; a 5 ms poller added 6.6 ms to an
  OctoQuad driving loop, against 3.0 ms at 10 ms ([`hub-bench-findings.md`](hub-bench-findings.md)).
- **Occupancy where balls rest** is a cross-check only (inferred). Even six beams at rest positions can't tell 3
  Pollen and a Nectar from 2 Pollen and a Nectar with the paddle closed, and with it open they read the same for 1 to
  3 Pollen; where the 3rd Pollen rests (s 97.3) the plate is open, so no sensor fits there. Take the count as the
  larger of the passing count and what occupancy proves; the two "track full" signals matter most.
- **Motor current can't count**: a touching train is one continuous load, and the transfer draws 4.5 A with no
  balls (team). From the CAD's contacts (inferred): the transfer motor is loaded by the Nectar spokes from 2 Nectars
  and by V2 from 3 Pollen (barely: that ball rests at s 97.3 and V2 reaches from 97); the intake only by a resting
  3rd Nectar (0.8 mm inside its reach) or a ball in the mouth; loads of up to 2 Pollen and at most 1 Nectar load
  nothing. Log it as a jam and full alarm.

**Simulated** (a model written for this page, not in the repo; nothing tested on a sensor). Highest speed at which
99% of runs count right, in m/s:

| beams | reader | sensor update | single ball | touching trains, all kinds | touching Pollen only |
|---|---|---|---|---|---|
| pair 66 mm up, 17 mm apart | covered/clear order | 10 ms | 1.9 | never (Nectar pairs read as one) | 1.6 |
| pair 66 mm up, 17 mm apart | `TransferCounter` | 10 ms | 0.9 | 0.5 | 0.9 |
| pair 6 mm up, 16 mm apart | covered/clear order | 10 ms | 2.0 | 1.6 | 1.6 |
| pair 6 mm up, 16 mm apart | covered/clear order | 20 ms | 1.0 | 0.9 | 0.9 |
| pair 6 mm up, 16 mm apart | `TransferCounter` | 10 ms | 0.9 | 0.8 | 0.8 |

- At 2.3 m/s a single ball is counted wrong 2 to 3% of the time by the order reader and 33 to 37% by
  `TransferCounter` at 16 to 17 mm spacing, and no extra space between balls removes that. No height or spacing
  reached 99% for touching trains above 1.7 m/s.
- These pairs assume balls in line on a flat surface; the one-beam-per-track figures in [The plan](#the-plan) use
  the CAD's paths instead.
- Assumptions: balls in line, at constant speed, touching or as stated, no slip or bounce, on a flat surface; a
  thin beam unless a cone is named; each sensor updating every 10 or 20 ms at a random phase; the hub's ADC refreshing
  every 9.7 to 11.1 ms (measured, [`hub-bench-findings.md`](hub-bench-findings.md)); a 10 ms poll with 0.3 ms
  jitter; at least 2000 runs per case. The order reader takes ±1 per covered/clear step; `TransferCounter` uses its
  defaults (threshold 25 mm, hysteresis 10 mm). Neither pair can be built at the hand-off (above).
- How the sensor reads a ball that half fills its field during one ranging window decides most of this and is
  unknown.

**What would settle it:**

1. Bench, one Laser Rangefinder in SHORT at a 10 ms budget: does it range at all, and does it read a Pollen right at
   15 to 45 mm ([Parts](parts-findings.md#still-unknown)). For P66: a Pollen about 35 mm out with a plate 6 mm below
   the beam, read empty and with the ball, with the 15° and with the 27° field; if the empty reading falls in the
   ball's band, a laser can't use a low beam. P66 needs 15° to range cleanly at 10 ms (at 27° the plate is nearer
   than the ball). For N20: a Nectar about 30 mm out with a plate at 117.5 mm. Run two side by side for crosstalk.
2. Hardware or CAD: is the floor plate's −x cheek solid, and can it take a tunnel at (y 66, z 16)? Is there room for
   the laser bodies outside the −x plate at both spots, and does the sweeper bar clear N20's beam? Where does the
   intake arm rest? Will the Nectar paddle ever close (the plan changes if it does)? Are break-beams legal, and does
   a Control Hub digital port pull their output up?
3. Still: load balls one at a time with the paddle closed and read each sensor at rest. Does P66 read covered with
   the 4th Pollen at rest, and do N20 and the Color Rangefinder (at about 20 mm) with the 3rd Nectar?
4. With temporary brackets at P66 and N20 and Transfer Thread Count Test with `oneSensor` on, run 4 touching Pollen
   20 times and 3 touching Nectars 20 times through the real intake at full power, and record each count. If fewer
   than 99% are right, try break-beams polled every 5 and every 10 ms.
5. Colour, on the bench: the Color Rangefinder's hue and distance for red, blue and Pollen balls at 15, 20, 25, 35,
   50 and 60 mm; with only a plate at 120 mm; through 8.2 mm-deep tubes of 12, 14 and 16 mm bore; a ball slid
   sideways at 20 and 50 mm to map its field of view; and a Nectar at 20 mm with a Pollen part in view at 50 mm. Then
   the drop test ([Parts](parts-findings.md#still-unknown)): Nectars past the window at about 20 mm, dropped from 51,
   73, 130, 270 and 345 mm (1.0, 1.2, 1.6, 2.3 and 2.6 m/s), logging hue and pin 1 at every poll.
6. Colour, on the robot at the spot step 5 picks: 20 runs of 3 mixed-colour Nectars at full intake power, reading the
   1st and 2nd as they pass and the 3rd at rest, with pin 1's arrival count; then the putting-out sequences above,
   counting what else leaves.
7. Log intake and transfer current and encoder speed while feeding 1 to 4 balls with the paddle closed: a step when
   the 2nd Nectar comes to rest or the mouth jams would make a jam and full alarm.
8. A slow-motion video of the intake arm as balls pass: whether the wheels follow a ball or let it coast, and where
   the arm rests.

## The 10.9 balls per second figure

[`robots-findings.md`](robots-findings.md) said the transfer "would pass about 10.9 balls per second", and the
shooter simulations use 92 ms between balls as the transfer's limit
([`flywheel-control-findings.md`](flywheel-control-findings.md),
[`shot-calculation-findings.md`](shot-calculation-findings.md), `scripts/sim/README.md`). Where 10.9 came from is
not recorded. The CAD of 2026-11-14 gives a touching Pollen train 21.5 balls a second free and 11.3 loaded (above),
so 92 ms stays a fair feed interval to simulate, but not a known limit.

## Sources

- Melonbotics Super Servo Plus: <https://docs.melonbotics.com/super-servo-plus> (1000 RPM free, 0.01 s/60°, 6 W at
  6 V, 2.3 kgf·cm and 3.9 A stall, 0.16 A free, 90 g, continuous rotation) and
  <https://www.melonbotics.com/pages/super-servo-plus> (the same, plus about 4.7 A natural stall current, about
  450–700 RPM recommended, CAD file `SuperServoPlus_No_Plate.step`), read 2026-11-15.
- goBILDA 5202-2402-0005: <https://www.gobilda.com/5202-series-yellow-jacket-planetary-gear-motor-5-2-1-ratio-24mm-length-6mm-d-shaft-1150-rpm-36mm-gearbox-3-3-5v-encoder/>,
  read 2026-11-15.
- goBILDA 5204-8002-0003: <https://www.gobilda.com/5204-series-yellow-jacket-planetary-gear-motor-3-7-1-ratio-80mm-length-8mm-rex-shaft-1620-rpm-3-3-5v-encoder/>,
  read 2026-11-15.
- goBILDA 2000-0025-0504: <https://www.gobilda.com/2000-series-5-turn-dual-mode-servo-25-4-super-speed/>, read
  2026-11-15.
- Brushland Laser Rangefinder: see [Parts](parts-findings.md#brushland-laser-rangefinder) for the snapshots read,
  its CAD model and ST's VL53L1X datasheet.
- Competition Manual (G407, G408, CONTROL, MOMENTARY): <https://ftc-resources.firstinspires.org/ftc/game/cm-html>,
  read 2026-11-15.
- Adafruit 2167 break-beam sensor: <https://www.adafruit.com/product/2167>, read 2026-11-15.
- The CAD numbers: the team's CAD as of about 23:00 PST on 2026-11-14, and as of 13:13 PST on 2026-11-15 for the
  paddles, where balls rest and everything near the intake, with both turrets placed on the transfer.
- The code facts: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/tele/BioBuzzBasicTele.java`.
