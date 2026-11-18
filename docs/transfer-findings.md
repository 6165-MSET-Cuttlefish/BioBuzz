# Transfer findings

How fast balls move through the BIOBUZZ robot's transfer, how to measure it, and where rangefinders can see them.
Read it before mounting or moving a transfer sensor, choosing a sensor's timing or sampling rate, writing code that
counts or times balls, or trusting a feed rate in a shooter simulation.

**Nothing on this page has been run on the robot.** Each number says how it is known:

- **(CAD)**: measured on the team's CAD as of the evening of 2026-11-14 (about 23:00 PST), and, for the paddles, where
  balls rest and everything near the intake, as of 13:13 PST on 2026-11-15 with both turrets placed on the transfer.
  The CAD was still changing: the transfer's part shapes are from the evening of 2026-11-14 and lack pockets cut on
  2026-11-15, the assembly that places the turrets uses an older version of the transfer that nobody compared with the
  current one, and every number near the intake shifts 1:1 with the transfer's front-to-back placement, which is known
  to only a few mm. The limit-switch layouts and the mid-height beam are the CAD on the evening of 2026-11-15, and the
  sensor placement at the transfer entrance ([The plan](#the-plan)) is the CAD on 2026-11-16.
- **(vendor)**: from the vendor page or chip datasheet linked under [Sources](#sources), read on 2026-11-15 unless it
  says otherwise.
- **(manual)**: the Competition Manual, read on 2026-11-15 ([Sources](#sources)).
- **(team)**: the team said so; not checked.
- **(inferred)**: worked out from the above; the reasoning is given.
- **(simulated)**: a model of balls, beams and the hub's sampling; its assumptions are listed with its results.

## What to do

- **Count balls coming in with the two Laser Rangefinders drawn at the transfer entrance, lasers A and B (not spots
  A and B further down), and read Nectar colour with the Color Rangefinder drawn just in front of and above them**
  (CAD, 2026-11-16). All three sit on the +x side and look across the channel in −x: laser A's beam at (y 52.49,
  z 68.9), laser B's 22 mm behind it at (y 74.49, z 68.9), and the colour beam at (y 44.0, z 102.28). Each laser sees
  every ball of both types, so count with the rule in [The plan](#the-plan): rising edges on each laser, laser B's
  covered spells split by their length, the larger of the two counts, and the count raised to 4 once laser A stays
  covered for 400 ms. In a simulation with both lasers at a 10 ms timing budget, and assuming the colour sensor flags
  every Nectar, the rule counted balls arriving apart right in 95% or more of runs up to 2.3 m/s. Touching balls
  counted low in 5% or fewer of runs up to 1.5 m/s (85 to 87% exact on the worst train, the rest high) and in up to
  10% at 2.3 m/s. At a 20 ms budget nothing counts touching balls well above 1.0 m/s. Nothing has been tested on a
  sensor.
- **Before anyone drills for the entrance lasers, bench-test one in SHORT mode at a 10 ms timing budget with the 15°
  zone, with balls 15 to 45 mm out.** The 15° zone is `roiSize` 4 in Configure Laser Rangefinder, its default and
  the smallest zone the sensor takes. At the entrance a Nectar passes 20 to 24 mm from the face and a Pollen 39 mm.
  The chip is an ST VL53L1X, and ST's datasheet promises accurate distances only from 40 mm and a timing budget no
  shorter than 20 ms; Brushland lists 10 ms ([The sensor](#the-sensor)). The counting rule needs a new distance
  every 10 ms, so check that too. At the full 27° zone laser B sees the edge of the Nectar's ramp 16 to 24 mm out,
  nearer than any ball (CAD). The sensor stores its zone and timing, and nothing records what the robot's two lasers
  hold, so configure both and note what they read back ([The plan](#the-plan), Detection zone and timing).
- **Keep balls at 1.5 m/s or less as they come in whenever they may arrive touching:** run the intake at roughly 55
  to 70% power, and don't drive into rows of touching balls (inferred; that the ball's speed follows intake power is
  untested). At free speed, 2.3 m/s, touching balls leave the lasers gaps of 0 to about 11 ms, no longer than about
  one 10 ms poll, which no code can fix (simulated; [The plan](#the-plan)).
- **Stop the intake at a count of 4, or as soon as laser A stays covered while intaking**
  ([The plan](#the-plan), When laser A stays covered). With touching Pollen a 5th can already be under the intake
  when the 4th is counted, and with 3 Nectars in, a 4th ball of either type stops under the intake. No sensor sees
  either, and both already count as CONTROLLED (CAD + manual). So when the count reaches 4 with the colour beam
  clear, run the intake alone in reverse briefly to put out a possible 5th ball: the resting 4th Pollen is out of
  the intake's reach (inferred; neither the CAD nor the simulation checked this step). With the colour beam covered,
  3 Nectars are in: don't reverse then, since that also throws out the 3rd Nectar, which only the intake holds.
- **Never add the two lasers' counts, and don't treat the order in which they change as meaningful.** Both see both
  ball types, so a sum counts every ball twice. A Pollen covers a laser for 49 to 54 mm of travel, and the beams are
  17 to 22 mm apart along its path: together about one Pollen (71.1 mm), so between touching Pollen which laser
  changes first is a toss-up. A Nectar covers laser A for 92.5 mm, and the beams are 21.7 mm apart along its path:
  about 114 mm against a 91.4 mm ball. Every reader built on the order of the pair's changes failed in the
  simulation ([Two sensors staggered along the path](#two-sensors-staggered-along-the-path)).
- **The Color Rangefinder reads the last Nectar's colour at rest, but a passing Nectar's late or not at all.** With
  the speed changing along the path as it does at free speed, a Nectar is in its beam for about 40 to 45 ms; its
  colour updates every 25 ms at best, so only about 26 to 42% of passing Nectars get one whole reading. At a constant
  2.3 m/s it is 36 ms and 9% (simulated). The verdict arrives after a touching next ball is already in. Pause the
  intake after each Nectar comes in until its hue is known, and decide what an "unknown" does
  ([Colour](#colour-the-color-rangefinder-on-the-x-side-plate)). A centred Pollen never enters its 15° view, so
  for counting, a colour hit means Nectar.
- **Don't count balls coming in with goBILDA's roller-lever limit switches.** The roller moves at most about 4.3 mm
  before its lever meets the body (vendor STEP), less than the 5.1 mm a Pollen can shift sideways at the cheek spot
  found for a switch body; there a passing Pollen holds it for about 7 ms at 2.3 m/s, so a 10 ms read misses fast
  lone balls (simulated); and no spot is pressed by both ball types within the lever's travel. The CAD on 2026-11-15
  rated every layout it checked red or impossible except a new hinged flap at the floor-plate lip, orange for Pollen
  only: a Pollen holds it for about 18 ms at 2.3 m/s, but as drawn it may not trip, it can't count Nectar, and
  nothing beyond the CAD has looked at it. The one use left is a "Pollen track full" switch where the 4th Pollen
  rests, which the CAD hasn't checked. Don't wire a two-switch pair to a hub encoder port either
  ([Limit switches](#limit-switches)).
- **Break-beams look legal by our reading of the rules; ask the official Q&A before relying on them.** A break-beam is
  a CUSTOM CIRCUIT, which R607 and R613 allow under conditions it can meet, and REV says every hub digital input has
  the pull-up its open-collector output needs ([Break-beams and the rules](#break-beams-and-the-rules)).
- **Only the Pollen paddle can close as drawn** (CAD, 2026-11-15). Two brackets stop the Nectar paddle long before
  it reaches a ball. Ask hardware where it is meant to sit
  ([Path, capacity and the blocker](#path-capacity-and-the-blocker)).
- **Spots A to E can't count balls coming in, so don't drill them for the possession count.** While a paddle holds
  balls, the queue stops on or short of them, so they see only the first one or two balls of a load. They suit
  counting balls leaving toward the turrets, and the timing measurement below
  ([Rangefinder spots](#rangefinder-spots)).
- **Take the one measurement below before trusting any speed here.** It tells which part of the transfer sets the
  ball rate, and it needs only one rangefinder and Transfer Thread Count Test.
- **For timing balls at spots A to E, try a 10 ms budget too, but don't count on it there.** At spots A to E the
  ball is 15 to 44 mm out. At 20 ms no spot sees the gap between touching Pollen balls for two readings, even with
  the transfer motor loaded. At 10 ms, spots A and B see it only while the train is slowed (motor loaded, or the
  servo roller metering), and maybe not even then (the beam is a cone: see
  [below](#ball-and-gap-times-and-which-sampling-rate-works)). No Nectar spot works at 20 ms.
- **Don't drill spots A and B as drawn: the two sensor bodies would overlap.** See
  [What to tell hardware](#what-to-tell-hardware).
- **The transfer's continuous servo `melonbotics` is a Melonbotics Super Servo Plus** (CAD). It sits on the lower
  rear Pollen roller shaft. The only Axon MAX in the CAD turns the front sweeper bar's hinge
  ([Parts](parts-findings.md#servos)).
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
   lists SHORT at 10 ms as 100 Hz ranging (vendor). At INIT the screen shows what the sensor holds now; after START,
   what the sensor reads back after the writes, with `Pin0` and `Pin1` at 1 (analog). A spare read on 2026-11-17:
   [`parts-findings.md`](parts-findings.md#brushland-laser-rangefinder).
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
  the two beams' spacing along the path (s, below; 20 mm for spots A and B, not their 13.4 mm straight-line distance) ÷
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

The two ball types share a rolling surface only on the field tile in front of the side plates, where nothing fixed can
carry a sensor. From the floor plate's front edge they part: the Pollen rolls flat on the floor plate and the Nectar
rides up the plate's two cheeks. So a low beam over the floor plate, far enough back that the Nectar has climbed clear
of it, sees only Pollen, and a beam on the Nectar ramp, in front of and above the Pollen path, sees only Nectar; the
beams finally drawn sit higher and see both types ([The plan](#the-plan)). All of this is CAD, 2026-11-15, unless
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
  is unknown. The guide's face at x 2.9 pushes the Pollen onto V1, but between z 30 and 60, where it would meet a
  Pollen, it starts only at about y 86 to 88; in front of that the Pollen has the cheeks' full 5.1 mm of sideways
  play (inferred from the CAD's meshes).
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
  57.2 mm from the centre. The colour sensor goes elsewhere:
  [Colour](#colour-the-color-rangefinder-on-the-x-side-plate).

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
  transfer entrance, as drawn in [The plan](#the-plan): don't drill A to E for that count.
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
- Every spot on this page has the ball 15 to 44 mm from the sensor face (20 to 39 mm at the entrance lasers), so
  bench-test at 15 to 45 mm in SHORT at 10 ms before drilling, and leave room for spacers. To try 10 ms: Configure
  Laser Rangefinder with `timingBudgetMs` 10 (its default is 20).
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
within the possession limit, and each Nectar ball's colour, to get rid of an opponent's Nectar before it is shot. The
robot code will read the hub's analog inputs from a thread every 10 ms (team).

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
should err toward counting too many. Stop the intake at a count of 4, or as soon as laser A stays covered
(below); if a 5th ball may be in, reverse the intake within well under 3 s, but not with 3 Nectars in, since that
also puts out the 3rd Nectar resting at s 17.7.
With balls arriving touching, a ball can be under the intake before any sensor sees it: with the Pollen paddle
closed a fifth Pollen stops at s −46.5, inside the intake's reach (s −99.5 to −3.0). And once 3 Nectars are in, the
third resting at s 17.7, a 4th ball stops at s −58.5 (Pollen) or −81.0 (Nectar), under the intake and in front of
every beam on this page (CAD). That ball is CONTROLLED, so the count must already read 4 when the third Nectar comes
to rest: stopping at 4 balls seen would stop one ball late. With a ball pinched under a stopped intake, no beam
anywhere in s −100 to +80 is clear of every ball (CAD). Keep opponent Nectar out with the camera and the driver; the
colour sensor is the backstop. Balls that are shot pass no intake sensor, so the code must subtract shots.

#### The plan

Two Laser Rangefinders at the transfer entrance count the balls coming in, and the Color Rangefinder just in front
of and above them reads each Nectar's colour: the team's two lasers and one Color Rangefinder. This is the placement
the CAD draws on 2026-11-16, with real goBILDA parts, in a working copy of the CAD that is not yet in the main model
(CAD, read 2026-11-15 and 2026-11-16). Where each ball covers each beam is worked out from the CAD's ball paths
(inferred; the CAD's own figures agree within 0.2 mm), and the counting rule comes from a simulation. Nothing here
has been tested on a sensor or on the robot. Lasers A and B are not spots A and B in
[Rangefinder spots](#rangefinder-spots).

**Where they are.** All three sit on the +x side, next to the funnel the CAD names Funnel Right and away from the
Super Servo Plus mount (inferred from [Where they go](#where-they-go)), and look across the channel in −x from
windows at x 33.4 to 33.5:

| sensor | beam (y, z) | covered at s: Pollen / Nectar | travel covered: Pollen / Nectar | ball nearest the face | empty channel |
|---|---|---|---|---|---|
| laser A, front (`frontRangefinder`) | (52.49, 68.9) | −16.6 to 32.4 / −42.2 to 50.3 | 49.0 / 92.5 mm | Pollen 39.4 mm, Nectar 20.3 mm | 104 mm or more |
| laser B, rear (`backRangefinder`) | (74.49, 68.9) | 0.8 to 54.4 / −20.5 to 58.9 | 53.6 / 79.4 mm | Pollen 39.4 mm, Nectar 24.2 mm | 104 mm or more |
| Color Rangefinder (`colorRangefinder`) | (44.0, 102.28) | never / −16.3 to 66.1 | 0 / 82.4 mm | Nectar 28.5 mm | beyond its 100 mm range |

- "Covered at s" is where the ball cuts the beam's axis, as elsewhere on this page. The CAD's own figures for the
  travel are 48.9, 92.5, 53.7, 79.4 and 82.3 mm. A ball reads 20 to 66 mm across its width. The nearest empty
  return, 104 mm, is laser B's 15° cone reaching the far cheek; the rest end out to about 123 mm (inferred).
- **Each laser sees every ball of both types.** At 2.3 m/s a Pollen covers laser A for 21.3 ms and laser B for
  23.3 ms, a Nectar 40.2 and 34.5 ms (inferred). A Nectar climbing the cheeks probably moves slower, about 1.6 m/s,
  and covers them longer (inferred). A Pollen reaches laser A's beam while it climbs the floor plate's 10 mm step;
  how it hops there is unknown.
- **The colour beam sees every Nectar and no centred Pollen**
  ([Colour](#colour-the-color-rangefinder-on-the-x-side-plate)).
- If hardware ever makes the Nectar paddle close, recheck all of this: the 2nd Nectar would rest at s 50.6, inside
  laser B's and the colour beam's Nectar stretch, and the next ball would stop unseen just in front of laser A, a
  Pollen at about s −17.5 (0.9 mm short of A's stretch) or a Nectar at −45.1 (inferred).

**Touching balls.** On the beam's axis touching balls do leave a gap, but it is too short to read (inferred from the
CAD's paths):

| touching pair | gap at laser A | gap at laser B |
|---|---|---|
| Pollen, Pollen | 22.85 mm, 9.9 ms at 2.3 m/s | 17.45 mm, 7.6 ms |
| Nectar, Nectar | 2.2 mm, 1.0 ms | 13.3 mm, 5.8 ms |
| Nectar, then Pollen | none | 4.45 mm, 1.9 ms |
| Pollen, then Nectar | 15.0 mm, 6.5 ms | 10.45 mm, 4.5 ms |

- If any part of a 15° cone counts as covered, only 2.75 mm is left, at laser A between two Pollen, and nothing
  anywhere else (inferred). With half the field covered, two Pollen clear laser A for about 23 to 25 mm (up to
  10.8 ms at 2.3 m/s), a Pollen right behind a Nectar for 0.7 to 11.1 mm, and two Nectars for 3.4 to 6.2 mm; with a
  quarter, two Nectars not at all (simulated).
- Beam placement alone can't separate touching balls (CAD). On the axis there are gaps, but none lasts much longer
  than one 10 ms poll at free speed, so touching balls need the speed limit and the covered-time split below.
- **Between two touching Pollen, which laser changes first is a toss-up.** Laser B clears the first ball 0.9 mm of
  travel before laser A is covered by the second (inferred; 1.6 to 2.7 mm with half the field covered, simulated),
  but beams 1 mm lower, or a reading that switches at a quarter of the field, reverse the order. Never rely on both
  lasers reading clear between touching Pollen.

**The counting rule** (simulated). A thread reads both lasers from one bulk read every 10 ms, and the colour
sensor's hue with them.

1. A laser is covered when its distance falls below an on-threshold and clear when it rises above an off-threshold
   about 10 mm higher. A ball reads 20 to 66 mm and the empty channel 104 mm or more, so put both between about 70 and
   100 mm (inferred: any value there gives the stretches above), and set them from bench readings.
2. On each clear-to-covered change of laser A, `nA` += 1; of laser B, `nB` += 1.
3. When laser B goes from covered to clear, split that covered spell, which may hold several touching balls
   (`L`: the travel over which one ball covers laser B; `p`: what each further touching ball adds; both explained
   below):
   - `T` = the time from its first covered poll to its first clear poll, in ms; `v` = the ball's speed in mm/ms
     (below).
   - If the colour sensor saw a Nectar between 15 ms before the spell began and 15 ms after it ended, `L` = 79.0 and
     `p` = 72.0; otherwise `L` = 52.9 and `p` = 71.1.
   - `nB` += min(3, max(0, floor((`T` × `v` − `L`) ÷ `p` + 0.5))): one spell adds at most 3, 4 balls in all.
   - Don't split a spell that hasn't ended, or one during which the intake stopped or step 5 fired: it stays the one
     ball step 2 counted.
4. Balls in = max(`nA`, `nB`). Never add them.
5. If laser A stays covered for 400 ms or more without a break while the intake runs forward, the robot is full:
   raise the count to 4 if it is lower, and stop the intake (below). The raised count stays after the intake stops
   and after laser A clears; only step 6 takes it down.
6. While the intake runs in reverse with the transfer stopped or reversed, freeze `nA` and `nB`, and take 1 off for
   each covered-to-clear change of laser A. That takes off a ball that was resting in the beam when reversing started,
   such as the 3rd Nectar, as well as one that passes back out. With the intake still forward a ball bounces between
   the intake and the transfer's first driven surface, across the beam each way
   ([Reversing the transfer motor](#the-intake-hand-off)), so subtracting then would count low. Take 1 off for each
   ball shot: shots pass no entrance sensor. Neither part was simulated.

Keep the count as one running number: add each rise in max(`nA`, `nB`), raise it in step 5, and take off what step 6
takes off. Without step 5 that is balls in − balls taken off.

- **Why step 3 has a cap and two exceptions** (inferred; not simulated with them): a ball that stops in laser B and
  later moves on makes one long covered spell, which the split would read as many balls. With the split uncapped, a
  lone Pollen that stalled at s 20 for 400 ms with the intake running, then moved on, counted 11 to 16, and after a
  1 s stall 28 to 39 (simulated, 2.3 m/s). The same happens when the resting 4th Pollen (s 26.3) or 3rd Nectar
  (s 17.7) moves on after a shot. With the cap, a ball stalled in laser B alone reads 4 at most, the safe error. They
  change no figure in the table below: no simulated train held more than 4 balls, the intake never stopped, and
  step 5 never fired on a passing train.
- **Why step 5 raises the count rather than holding 4 only while laser A is covered** (inferred): touching Nectars
  rarely clear laser A, and their laser B spell isn't split once step 5 has fired, so `nA` and `nB` can sit well
  below the true number. Held only while laser A is covered, the count would drop below the truth as soon as a shot
  moves the 3rd Nectar on: the costly error. Raised once, 3 Nectars alone read one high until they are shot, the
  safe error.
- `L` is the travel over which one ball covers laser B with half its field covered (Pollen 52.9 mm, Nectar 79.0 mm;
  simulated), and `p` what each further touching ball adds. 71.1 mm is a Pollen's diameter. A touching Nectar adds
  92.5 to 98.2 mm along its curved path (inferred), but 72.0 puts the errors on the high side: with 80 or 90, the
  share of runs counted low at 2.3 m/s on the worst mix rises from 10% to 18% or 34% (simulated).
- The Nectar flag needs only "a Nectar was in view": a hue in the red or blue range. If the colour sensor misses a
  Nectar, its spell is split with the Pollen numbers, which counts 2 or 3 touching Nectars one high, the safe error
  (inferred). A false flag on a Pollen spell swaps in the Nectar numbers, which still count touching Pollen right at
  the true speed but leave less room for speed error, so it can count low; not simulated. CALC, with one Pollen
  covering 52.9 mm and each further one 71.1: 4 touching Pollen cover about 266.2 mm, and read 3 once `v` is about 3%
  low with the Nectar numbers ((0.973 × 266.2 − 79.0) ÷ 72.0 + 0.5 = 3.0), against about 13% low with the Pollen
  numbers ((0.866 × 266.2 − 52.9) ÷ 71.1 + 0.5 = 3.0). With no colour sensor at all, use max(`nA`, `nB`) alone: it
  never counted high in the simulation, but it counts touching trains low. Splitting without the ball type counted
  high in about 50 to 90% of runs.
- **Ball speed `v`.** The simulation assumed an estimate within ±15% of the true speed; ±25% raises the worst share
  counted low at 1.5 m/s from 4% to 12%. Where it comes from is open. The simulation's suggestion: the intake wheels'
  surface speed from the intake motor's encoder, times about 0.52 to 0.53 (the ball centre's share on the tile:
  2304 and 2238 against 4309 mm/s in [Ball speeds](#ball-speeds)), plus about 0.47 × the robot's speed while it
  drives into balls. But the ball is on the floor plate or the cheeks by the time it reaches laser B, where most of
  the driving gain is lost (below), so check the estimate against the entrance speed measured on the robot.
- **Speeds at the intake** (inferred, no slip assumed): on the tile a pinched ball moves at 2.2 to 2.8 m/s with the
  robot standing still. Driving adds about 0.47 × the robot's speed (0.465 Pollen, 0.481 Nectar, with the wheel above
  the ball): at 1.85 m/s, the only drive speed in the repo (`BioBuzzConstants`, still the Betta bot's value), that is
  about 3.1 to 3.7 m/s. On the plate and the ramp most of that gain is lost. All intake-derived speeds scale with the
  intake's real loaded speed, which is unknown, and with battery voltage, since the intake runs at power 1.0 with no
  voltage compensation. The side rollers' 1529 / 803 mm/s come from the transfer motor and don't change with driving.

**How well it counts** (simulated). Share of runs counted exactly / high / low, with both lasers at a 10 ms timing
budget and a colour sensor that flags every Nectar (model below). Each figure is the worst train in its group, so a
row's three figures can come from different trains:

| balls arrive | 1.0 m/s | 1.5 m/s | 2.3 m/s | 3.5 m/s |
|---|---|---|---|---|
| alone or 40 mm apart | 99 / 1 / 0 | 98 / 2 / 0 | 95 / 5 / 0 | 77 / 23 / 5 |
| touching, one type | 94 / 6 / 0 | 85 / 15 / 0 | 68 / 32 / 2 | 49 / 49 / 16 |
| touching, mixed | 94 / 6 / 2 | 87 / 11 / 5 | 73 / 23 / 10 | 62 / 28 / 16 |

- **Keep balls that may arrive touching at 1.5 m/s or less**: there 5% or fewer count low on the worst mix, 2% or
  fewer at 1.0 m/s. Free speed, 2.3 m/s, counts low in up to 10%; 3.5 m/s in 16%. Balls that arrive apart are fine to
  2.3 m/s. Getting to 1.5 m/s means running the intake at roughly 55 to 70% power (inferred; that the ball's speed
  follows intake power is untested) and not driving into rows of touching balls.
- Most of the high counts are 3 touching Nectars read as 4, which is harmless: 3 Nectars already fill the robot. At
  2.3 m/s, 4 touching Pollen count 88 / 9 / 2 and 3 touching Nectars 68 / 32 / 0.
- max(`nA`, `nB`) alone, without the split: touching trains of one type count low in 22% of runs at 1.0 m/s and 70%
  at 2.3 m/s.
- **At a 20 ms timing budget nothing counts touching balls well above 1.0 m/s**: the rule counts touching mixed
  trains low in 3% of runs at 1.0 m/s and 11% at 1.5. Even max(`nA`, `nB`) drops to 82% exact at 2.3 m/s with balls
  arriving apart.
- A sensor that gives a fresh reading every 5 ms, such as a break-beam, would count 97% or more of every group
  exactly up to 2.3 m/s with the same rule.
- Sensitivity at 1.5 m/s, worst share counted low: beam edges off by 0, ±3 or ±6 mm, 4 to 6%; a speed estimate off by
  ±25%, 12%; 1% of polls arriving 20 ms late, 4%.
- Model: the CAD's paths and beam points; each ball's beam edges off by up to ±3 mm; the laser averages how much of
  its 15° field the balls fill over each 10 or 20 ms ranging, back to back at a random phase, with ±0.05 noise; the
  hub's ADC refreshing every 9.7 to 11.1 ms (measured, [`hub-bench-findings.md`](hub-bench-findings.md)); a poll
  every 10 + 0 to 0.9 ms with both lasers from one bulk read; covered from 0.3 to 0.7 of the field, with ±0.05
  hysteresis; a perfect Nectar flag, raised while a Nectar's centre is anywhere in s −42.1 to 58.6, a window wider
  than the colour beam's view (s −16.3 to 66.1), with no hue delay. 21 trains of 1 to 4 balls: P, PP, PPP, PPPP, N,
  NN, NNN, NNNN, PN, NP, PNP, NPN, PNPN, NPNP, PPNN and NNPP touching (half pushed, half with the leaders coasting
  ahead), PPPP and NNN 40 mm apart, and PPPP, NNN and PNPN 150 mm apart (alone); orders such as PPN, NPP or PNNP were
  not run. 1.0 to 3.5 m/s; 1000 runs per case, 600 for the sensitivities.
- **What code can't fix:** gaps of 0 to about 11 ms between touching balls at free speed; a ball stopped under the
  intake in front of both beams, which only step 5 and the 5th-ball reverse (below) cover; and a ball that backs out
  with the intake running forward, which counts one high, or two if it got past laser A's stretch first (simulated),
  the safe error.
- A Pollen arriving right behind a Nectar leaves 0.7 mm at laser A and 7.8 mm at laser B (none with a quarter of the
  field), and only the covered-time split recovers that pair: without it even a sensor read every 5 ms counts it
  right in only 62 to 72% of runs; with it, 99 to 100% on that sensor, and 79 to 100% on the 10 ms laser from 3.5
  down to 1.0 m/s (simulated).
- The laser's real update rate, its output delay, how it reads a part-filled field and how it ranges at 20 to 39 mm
  are all unknown until the bench test.

**When laser A stays covered** (inferred from the CAD's rest positions; step 5 was simulated):

- **What to do: raise the count to 4 and stop the intake, and don't reverse it because the robot is full.**
  Reversing throws out the 3rd Nectar, which only the intake holds. With 3 Nectars in, a 4th ball can stop under the
  intake at s −58.5 (Pollen) or −81.0 (Nectar), outside both lasers' beams, and it already counts as CONTROLLED, so
  the count must read 4 when the 3rd Nectar comes to rest. Step 5 does that: in the simulation it fired in every one
  of these end states and never on a train still passing at 1.0 m/s or more. A 5th ball is another matter: see
  Balls no sensor sees, below.
- With the Pollen paddle closed, the 4th Pollen rests at s 26.3, covering laser A (its axis only 4.4 mm inside the
  ball, so this hangs on the true rest position) and laser B; the colour beam stays 19 mm clear of it.
- The 3rd Nectar rests at s 17.7, covering laser A, laser B and the colour beam, which reads its colour still.
- So all three covered means 3 Nectars in, and both lasers covered with the colour beam clear means 4 Pollen in. With
  the paddles open, the 7th Pollen (s −8.6), or a Nectar at s −27.8 behind a 6th Pollen, covers laser A alone.
- A lone Pollen that stalls untouched at s −0.5 to 32.4 covers laser A and reads as full with one ball in: the safe
  error, but the driver needs a way to override it. Stalled at s 32.4 to 46 it covers only laser B, which step 5
  ignores; once it moves on, step 3's cap makes it read 4 at most (inferred). Whether one stalls there is unknown.
- Touching Nectars passing slower than about 0.7 m/s would also cover laser A for 400 ms (inferred), again stopping
  the intake early.

**Balls no sensor sees** (inferred from the CAD's rest positions). These stop in front of laser A and count as
CONTROLLED all the same: a 5th Pollen at s −46.5 behind 4 with the paddle closed; an 8th at −81.4 with the paddles
open; a 4th ball behind 3 Nectars, a Pollen at −58.5 or a Nectar at −81.0; a Nectar at −64.5 behind the 4th Pollen;
and any ball pinched under a stopped intake (−100 Pollen, −124.5 Nectar). Step 5 counts the 4th ball behind 3
Nectars, but not a 5th ball: it makes the count 4, one low. With touching Pollen the 5th is already under the intake
when the 4th is counted: the 4th covers laser A from s −16.6, and a touching 5th is then 71 to 73 mm behind, near
s −88 to −90, inside the intake's reach (s −99.5 to −3.0). So when the count reaches 4 with the colour beam clear,
stop the intake, let the balls settle, then run the intake alone in reverse for at least 64 to 89 ms, as for
[putting a Nectar back out](#putting-a-wrong-colour-nectar-back-out). That puts out anything under the intake, while
the resting 4th Pollen (s 26.3) is out of its reach; with the transfer stopped, step 6 takes off only a ball that
leaves laser A. With the colour beam covered, 3 Nectars are in and the 3rd is within the intake's Nectar reach (to
s 18.5): don't reverse then. Neither the CAD nor the simulation checked this step (inferred).

**Detection zone and timing.**

- **Set each laser's region of interest to 4 × 4 of its 16 × 16 grid, the smallest allowed, which Brushland gives as
  15°; the full grid is 27° (vendor).** It isn't an angle: code writes the box's corners over I2C (`setROI` in
  `vendor/brushland/LaserRangefinder.java`, which rejects anything under 4 × 4), the sensor stores it, and it takes
  effect at the next power-on (vendor). There is no button or app for it
  ([Parts](parts-findings.md#brushland-laser-rangefinder)).
- Configure Laser Rangefinder already does this: its defaults are SHORT, `timingBudgetMs` 20, `measurementPeriodMs` 0
  and `roiSize` 4, and it centres the 4 × 4 box on the sensor's factory optical centre and shows the box it reads back
  (`opmodes/test/ConfigureLaserRangefinder.java`). Run it on each laser with `timingBudgetMs` 10 before switching it
  to analog, and record the box and timing it reads back: nothing records what the robot's lasers hold, and Brushland
  doesn't say what a new one ships with.
- Why 15°: at 27° laser B's cone reaches the Nectar ramp's edge, the near +x cheek's crest, 16 to 24 mm out (CAD),
  nearer than any ball. Laser A's 10.5 mm hole clears either zone.
- Brushland says smaller regions are noisier and want a longer budget, while counting wants 10 ms (vendor). ST rates
  the chip at 20 ms or more and accurate only from 40 mm, and a Pollen passes at 39 mm, a Nectar at 20 to 24 mm
  ([The sensor](#the-sensor)). Only the bench settles it.
- **Don't mount the lasers lower or tilt them down.** Laser B's 15° cone clears the near cheek's crest by only about
  2 to 3 mm of height: with the beam at z 66.9 the cone still clears the crest; at z 65.9 it meets it 27.5 mm out
  (CAD).

**To build** (CAD, 2026-11-16; part numbers as the CAD names them, not checked against goBILDA's site):

- **Beam L**, goBILDA 1106-0007-0056 (7 holes, 56 mm), stands upright between the lasers at x 35.9 to 43.9, y 59.3 to
  67.3. The two lasers clamp to it with 2 M4 × 40 screws (2800-0004-0040) along y at z 65.25 and 73.25: heads on
  laser A's front face, nuts (2811-0004-0007) on laser B's rear face; the grip is 36 mm, laser 14 + beam 8 + laser 14.
- Beam L fixes with 2 M4 × 22 screws (2800-0004-0022) along x at y 63.3: at z 57.25 through the +x guide and the +x
  side plate with a 3 mm spacer (1502-0006-0030), at z 105.25 through the plate only with an 8 mm spacer
  (1502-0006-0080), nuts inside the channel.
- **Beam C**, 1106-0004-0032, at x 35.9 to 43.9, y 28 to 36, z 82 to 114. The Color Rangefinder clamps to it with 2
  M4 × 30 screws (2800-0004-0030) at z 94 and 102; beam C fixes with 2 M4 × 22 screws and 8 mm spacers at (y 32,
  z 86) and (32, 110), nuts inside the channel. In the CAD the Color Rangefinder's instance is named "Case Bottom <2>".
- Replace the existing screw through the guide and the plate at (y 63.88, z 93.05) with an M4 × 10 (2800-0004-0010),
  head inside the channel and nut on the guide's outer face, fitted before beam L. Not an M4 × 12, and not head
  outside.
- Turn the paired nuts at z 65.25 and 73.25, and at z 94 and 102, flats facing: they sit 8.0 mm apart and measure
  8.08 mm across corners. Fit the nut at z 65.25 first, then 73.25, with the wrench from above. Tools: a 3 mm hex key
  and a slim 7 mm open-end wrench with a head 13.5 mm across or less.
- Hardware in all: 2 M4 × 40, 4 M4 × 22, 2 M4 × 30, 1 M4 × 10, 9 nuts, one 3 mm and three 8 mm spacers.
- **Holes.** In the +x side plate: laser A at (y 52.49, z 68.9), back 5.6, up 63.9 (inferred, measured as in
  [Where they go](#where-they-go)); the colour sensor's 12 mm hole at (44.0, 102.28), back −2.9, up 97.3; 4.3 mm
  holes at (63.3, 57.25), (63.3, 105.25), (32, 86) and (32, 110). In the guide: laser B's 12 mm hole at (74.49,
  68.9), back 27.6, up 63.9, and a 4.3 mm hole at (63.3, 57.25). The cones need 8.1 mm at laser A for 15° (10.0 mm
  for 27°), 7.3 (8.5) at laser B, and 9.5 (11.4) at the colour sensor.
- **Laser A's hole is 10.5 mm**, so that it clears the full 27° cone as well as the 15° one (CAD). An earlier plan
  said 8.5 mm, which the 27° cone would clip 8.8 mm out; use 10.5.
- **Plugs:** the lasers' plugs face down, the colour sensor's up. Laser A's cable comes within 1.66 mm of Funnel
  Right, the colour sensor's within 6.96 mm of the intake motor.
- **Clearances:** the sensors are 0.6 mm from the guide; the intake swings past with at least 13.25 mm to spare. The
  nuts inside the channel stand 2.82 mm into it; with the sideways play a ball really has, a Nectar comes 7.0 mm from
  the M4 × 10's head and 8.9 mm from the nut at z 105.25, and a Pollen 16.5 mm or more from any of them.

**Placements the CAD tried first** (CAD, 2026-11-15 and 2026-11-16):

- A low-front pair on a 1106-0008-0064 beam, lasers at (y 53.81, z 49.35) and (76.19, 72.65) and the colour sensor
  at (77.0, 92.72), didn't fit when drawn: laser A's screw heads hit Funnel Right by 1.86 mm with 0 to 0.6 mm
  of hex-key room, the nuts had 0.07 to 0.56 mm of room, cables crossed screws by 1.3 to 1.9 mm, laser A's beam ran
  through the funnel's flange, and the colour sensor's hole broke the plate's edge.
- A printed carrier for both lasers, with the colour sensor at C1, was drawn only and dropped; it had no fallback
  made of goBILDA parts alone.
- The drawn pair lower: at the low spot Funnel Right blocks the tools, and below about z 66.9 laser B's cone meets the
  cheek. 2 mm lower would add only about 3 mm of Pollen travel (CAD; 3.2 to 3.3 mm, inferred).
- One beam per track, and the spots near the hand-off: below.

#### One beam per track: considered, not chosen

Before the CAD drew the entrance pair, this page planned one laser per track, each where only its own ball type
passes. It wasn't chosen: neither spot was drawn or checked in the CAD (both were worked out from its paths and
meshes), the Pollen beam needs a tunnel about 20 mm long through the −x cheek, whose solidity is unknown, and the
entrance pair has since been drawn and checked with real parts.

| beam | (y, z) | back, up | covered at s: Pollen / Nectar | what it sees |
|---|---|---|---|---|
| P66, Pollen counter | (66, 16) | 19.1, 11.0 | −0.9 to 38.8 / never | every Pollen; never a Nectar (at most 9% of its cone) |
| N20, Nectar counter | (33.0, 106.6) | −13.9, 101.6 | never / −10.2 to 50.1 | every Nectar; never a Pollen |

- **What it does better.** Touching balls leave 31.5 to 35 mm of travel clear at each beam, 1.5 to 3 times the gaps
  at the entrance pair, and it needs neither the colour sensor nor a ball-speed estimate. Share of runs that counted
  4 touching balls as 4, by speed in m/s (simulated):

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

- At 2.3 m/s with a 10 ms laser, the entrance rule counts 4 touching Pollen right in 88% of runs (9% high, 2% low)
  and 3 touching Nectars in 68% (32% high, none low), against 87% and 92% here. The two simulations differ, so compare
  loosely. What the entrance pair gains is one "full" signal, laser A covered, for every end state.
- Model for the table: covered travel from the CAD paths, a part-filled cone counted as covered from 50%; a laser
  takes a snapshot every 10 or 20 ms at a random phase, read through the hub's ADC (refreshing every 9.7 to 11.1 ms,
  measured) by a 10 ms poll; a break-beam is a thin beam whose state is read every 5 or 10 ms. 1.53 m/s is a touching
  Pollen train the side rollers set. Only 3 Nectars fit, so the N20 rows are conservative.
- P66 needed the 15° field (at 27° the floor plate is nearer than the ball, 35.2 mm out), the beam kept at about z 16
  (at z 18 the body comes within 1.1 mm of the side-roller mount), and a tunnel through the −x cheek wide enough for
  the cone. N20's cone ends on the far plate at 117 to 118 mm with the ball 29.7 mm out; the front spokes' disc passes
  15.1 mm from its axis (inferred, from the CAD's meshes).
- A break-beam pair would need the tunnel through the +x cheek and plate at P66 too, and a hole through the +x plate
  at N20; Adafruit 2167 halves fit on both sides at both spots (inferred).
- It held only while the Nectar paddle can't close: if it closes, the 2nd Nectar rests at s 50.6 in N20's cone, the
  next ball stops unseen under the intake, and 10 of the 30 mixes of up to 4 balls read 1 or 2 low (inferred).

#### Spots considered

| spot | (y, z) | back, up | covered at s: Pollen / Nectar | verdict |
|---|---|---|---|---|
| 1, low over the floor plate's edge (the CAD's best counter) | (51, 16) | 4.1, 11.0 | −28.2 to 23.8 / −39.0 to −13.7 | not chosen: sees both types, the resting 4th Pollen fills 39% of its cone, part of its empty view returns at the ball's distance, and it never sees a 4th ball stopped behind 3 Nectars |
| 4, high, just behind the plate's edge | (45, 76) | −1.9, 71.0 | −13.0 to 16.5 / −44.6 to 50.6 | rejected: sees both types; touching Nectars leave 1.2 mm; the resting 3rd Nectar fills its whole cone |
| 5, partner for 4 | (61.5, 76) | 14.6, 71.0 | | rejected: starts inside the side-roller mount |
| 2, low on the tile | y 34.5 | | | rejected: nothing to fix it to (the body cuts the funnel); 34 to 41% of the cone sees the tile and up to 15% leaves the robot |
| 3, low, from the drivetrain's inner plate | y 24 | | | rejected: 39% of the rays leave the robot |
| 6, 66 mm up, in front of the plates | z 66 | | | rejected: the intake is in the cone and touches the body |
| 7, looking down from a new cross bracket | y 10 to 30, face at z 108 | | | rejected: every Nectar runs into the body (by up to 5.1 mm, over s −5.4 to 43.1); moved to y 0 to 20 it clears a Nectar by 2.2 mm, but the intake wheel's rim reaches it at 15° to 25° of lift |
| 8, looking up through the floor plate | | | | rejected: the ball rolls on the sensor's face, and there is no room under the 5 mm plate |
| any pair 16 to 17 mm apart, low or high | | | | none fits at the heights checked then; the CAD later fitted a level pair 22 mm apart at z 68.9 on the +x side ([The plan](#the-plan)) |

The verdicts are CAD; back and up are worked out from (y, z), and the covered ranges from the CAD's paths, as in
[One beam per track](#one-beam-per-track-considered-not-chosen).

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
- **A mid-height beam both types block** (CAD, 2026-11-15, not rechecked): at (y 56, z 46) a
  Pollen covers it for 72 mm of travel and a Nectar for 76 mm, and the parked 4th Pollen and 3rd Nectar both sit in
  it, but touching Pollen show no gap there (72 mm covered against a 71.1 mm ball). The CAD then placed a level
  laser pair 16 mm apart on the +x guide plate; the pair the CAD went on to draw, 22 mm apart at z 68.9, is
  [The plan](#the-plan).

#### Colour: the Color Rangefinder on the +x side plate

Put it where the CAD draws it (CAD, 2026-11-16): on its own goBILDA beam on the +x side, looking −x through a 12 mm
hole in the +x side plate, its beam at (y 44.0, z 102.28), back −2.9, up 97.3 (inferred). Every Nectar crosses the
beam's axis over s −16.3 to 66.1, 82.4 mm of travel (inferred; the CAD gives 82.3), or s −26.3 to 78.0 if any part of
a 15° cone counts. The mount is under To build in [The plan](#the-plan).

- **Distance.** A passing Nectar comes 28.5 mm from the face at its nearest, and the resting 3rd Nectar is about
  29 mm away (inferred). Brushland gives "accurate down to 0.1mm within the first 20mm" (vendor), which doesn't reach
  28.5 mm; how good its colour is there is unknown. Configure Color Rangefinder's `presenceMm` default of 30 sits
  right at that distance: set it from bench readings. Empty, a 15° or 27° view reads beyond the 100 mm range (CAD).
  Some rays reach robot parts and the field through the plates' windows, so trust a hue only while a ball is known
  to be in view (Its distance output, below).
- **Pollen.** A centred Pollen stays out of a 15° cone by 12.2 mm and out of a 27° cone by 4.3 mm, and first enters
  a field about 33.5° across (inferred; other CAD measurements give 12.6 to 13.7 mm for the 15° margin). Brushland
  doesn't publish the field. A Pollen 20 mm off centre that rode up a cheek would fill the whole view over about s −5
  to 43 (simulated, on the CAD's off-centre paths); the cheek's corner pushes such a ball inward, but whether it
  turns or rides up is unknown ([The intake hand-off](#the-intake-hand-off)). So a colour hit means Nectar for a
  centred Pollen, and a Pollen in view would read yellow, not red or blue.
- **The last Nectar in, at rest.** The 3rd Nectar fills the whole 15° view at s 17.7, and still at s 15, 12, 4.5 and
  3, where more arm lift lets it settle; at s −9 it fills 86.5% (simulated). So its colour is read still, and the beam
  gives a second "3 Nectars in" signal. The 1st and 2nd Nectars are read only in passing; the 4th (s −81.0) and any
  ball under the intake are never in view.
- **Passing Nectars.** Time in the beam, on its axis (inferred): 35.8 ms at 2.3 m/s throughout; 51 to 52 ms at the
  ramp's 1.57 to 1.61 m/s; 40 to 45 ms free, with the speed changing along the path; 53 to 63 ms with the spokes
  loaded; 55 to 69 ms if the intake arm can't follow and the ball coasts. Share of passes with at least one whole
  25 ms colour reading taken while the Nectar fills 95% or more of a 15° view (simulated):

| free, ball fast | free, ball slow | spokes loaded | ball coasting |
|---|---|---|---|
| about 26% | about 42% | 64 to 93% | 67 to 100% |

- At the chip's 100 ms default a moving Nectar gets no whole reading. The figures for the other spots below come
  from a different model: compare loosely.
- **So:** the verdict on the 1st and 2nd Nectar comes after the ball has left the view, which ends at s 66 (the
  spokes take the ball from s 29.3), and with touching arrivals the next ball is in by then. Pause the intake after
  each Nectar comes in until its hue is known, or reverse it back past the sensor for a second read
  ([below](#putting-a-wrong-colour-nectar-back-out)), and decide what an "unknown" hue does. Put pin 0 on HUE
  (Configure Color Rangefinder's default is DISTANCE), and latch each Nectar's hue while it is in view rather than
  reading colour when a ball is counted.
- **Mixed views.** A Pollen alone reads yellow or empty (86° and 93° on the one bench reading of each), outside the
  red 10 to 50° and blue 170 to 210° ranges, and a red or blue Nectar mixed with a Pollen most likely lands between
  ranges and reads "unknown" rather than the wrong colour (inferred, not measured).
- **Its distance output.** With hue on pin 0, only pin 1's digital "within `presenceMm`" is left for distance, and
  as wired only pin 0 reaches the hub ([`wiring-findings.md`](wiring-findings.md)); pin 1 would need a digital port
  and a splitter cable. So use the hue with no distance gate, and only inside laser B's spell window (step 3 of the
  counting rule in [The plan](#the-plan)). While a centred Pollen covers laser B the colour view is as empty as with
  no ball, which read 93° on the one bench reading, outside both Nectar ranges, and a false flag only shrinks the
  split's room for speed error on that spell (The plan, the Nectar flag). Wire pin 1 to a free Control Hub digital port as the gate only if the bench shows
  a red or blue hue with no Nectar in view (inferred).

**Colour spots considered and not chosen** (C1 and C2: CAD, 2026-11-15; s 32 to 35: inferred from the CAD). C1 and
s 32 to 35 put the case on the outer face of the +x guide's flange at x 32.9, drilling the flange (5.0 mm) and the
plate (3.2 mm); C2 is on the −x plate. Passing reads below are for free speed, ball fast and slow.

| | drawn spot | C1, s 25 | s 32 to 35 | C2, s 60 |
|---|---|---|---|---|
| beam (y, z) | (44.0, 102.28) | (64.6, 86.4) | (68.7, 91.1) to (70.5, 93.5) | (83.4, 114.9) |
| Nectar on the axis, s | −16.3 to 66.1 | −20.1 to 71.7 | −13.8 to 80.9 | 14.2 to 106.0 |
| Nectar's nearest to the face | 28.5 mm | 19.7 mm | 19.65 mm | 14.7 mm |
| resting 3rd Nectar | fills the view, 29 mm out | fills the view, 20 to 21 mm out | fills the view, 21 to 25 mm out | 74% of the cone, plus 15% of the 2nd |
| passing Pollen, 15° cone | 12.2 mm clear | up to 15% of the cone, from 54.7 mm | none | none |
| passing Nectars with a whole reading | about 26 to 42% | 62 to 83% | 52 to 80% | 31 to 73% |

The drawn spot keeps a centred Pollen out of a 15° view, which C1 doesn't, at the cost of fewer passing reads and a
farther ball. Nobody checked that the guide's flange is solid behind a case at C1 or s 32 to 35, and C2's still read
mixes two Nectars' colours.

#### Putting a wrong-colour Nectar back out

Shooting is first in, first out, so reading colours anywhere every Nectar passes gives their order. Reversing the
transfer motor also moves the Pollen, since every transfer shaft turns 1:1 off it
([Reversing the transfer motor](#the-intake-hand-off)). Sequences (inferred from the CAD's contacts and the speeds
above; nothing tested):

- **The wrong Nectar is the 3rd, at rest at s 17.7 and read still.** Keep the transfer stopped and reverse only the
  intake, for at least 64 to 89 ms (s 18.5 to −124.5 at 2238 or 1610 mm/s). Only balls under the intake in front of it
  leave with it (a Pollen stopped at s −58.5 or a 4th Nectar at −81.0); nothing in the transfer moves, and no Pollen
  can be at s 0 to 72 then. It needs the intake wheels to reach the ball: at the drawn arm pose they touch a Nectar up
  to s 18.5, with the arm 10° up only to s 4.5. Laser A sees the Nectar leave, which takes it off the count ([The
  plan](#the-plan)).
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

- **`TransferCounter` as it stands can't count at the entrance** (inferred, from its code and the CAD's paths). It
  counts when laser A is clear while laser B is covered, a window of only 22.1 mm of Pollen travel (9.6 ms at
  2.3 m/s) and 8.6 mm of Nectar (3.7 ms); touching Nectars never clear laser A, and a ball backing out counts again.
  To use the rule in [The plan](#the-plan) the code needs: a rising-edge count per laser and the larger of the two;
  the covered-time split on laser B with its cap and exceptions; the 400 ms "full" step on laser A, which raises the
  count to 4; the 5th-ball reverse at a count of 4; −1 per covered-to-clear change of laser A while reversing, and −1
  per shot; thresholds set from bench readings at the new distances; the Color Rangefinder's
  pin 0 on HUE, each Nectar's hue latched while it is in view, and `presenceSensorName` moved off analog 3, where
  laser B is; and both lasers configured with `roiSize` 4 at 10 ms, with what they read back recorded. No robot code
  reads these sensors yet.
- **Reading two beams as a covered-or-clear order** works only when their spacing is below both the gap and the chord
  the beams see (the next ball reaching the first beam before the last one leaves the second reads as backward). At
  the entrance the gaps are 0 to 23 mm against 22 mm of spacing, so it can't (inferred).
- **Digital outputs.** The Laser Rangefinder can output a distance threshold as a digital pin (vendor). The Control
  Hub's 8 digital ports are all unused in `biobuzz.xml`, and a digital pin skips the hub's 10 ms ADC refresh
  (inferred). How often the hub samples digital inputs is unmeasured, and a read gives only the level at that
  moment: the SDK has no command that latches a digital input or counts its edges
  ([`sdk-hub-findings.md`](sdk-hub-findings.md#digital-inputs)).
- **Break-beam sensors** on Control Hub digital ports: Adafruit's 2167 lists "Response Time: <2 ms", "20mm x 10mm x
  8mm", a sensing distance of "Approx 25cm", "3.3 - 5.5VDC" and an "open collector transistor output" (vendor), so it
  needs a pull-up, which REV says every hub digital input has; for the rules, see [Break-beams and the
  rules](#break-beams-and-the-rules). A thin on/off beam has no plate-top return and no minimum range. The halves fit
  at P66 and N20 but hit the funnels at spot 1 (inferred); whether a pair fits at the drawn laser beams, with one half
  on the −x side, is unchecked. With a fresh level every 5 ms, the rule in [The plan](#the-plan) would count 97% or
  more of every group of trains exactly up to 2.3 m/s (simulated). How fresh a digital read is at a 5 ms poll is
  unmeasured; a 5 ms poller added 6.6 ms to an OctoQuad driving loop, against 3.0 ms at 10 ms
  ([`hub-bench-findings.md`](hub-bench-findings.md)).
- **Occupancy where balls rest** is a cross-check only (inferred). Even six beams at rest positions can't tell 3
  Pollen and a Nectar from 2 Pollen and a Nectar with the paddle closed, and with it open they read the same for 1 to
  3 Pollen; where the 3rd Pollen rests (s 97.3) the plate is open, so no sensor fits there. Take the count as the
  larger of the passing count and what occupancy proves; the two "track full" signals matter most.
- **Motor current can't count**: a touching train is one continuous load, and the transfer draws 4.5 A with no
  balls (team). From the CAD's contacts (inferred): the transfer motor is loaded by the Nectar spokes from 2 Nectars
  and by V2 from 3 Pollen (barely: that ball rests at s 97.3 and V2 reaches from 97); the intake only by a resting
  3rd Nectar (0.8 mm inside its reach) or a ball in the mouth; loads of up to 2 Pollen and at most 1 Nectar load
  nothing. Log it as a jam and full alarm.
- **goBILDA roller-lever limit switches can't count balls coming in**: at most a "Pollen track full" signal
  ([Limit switches](#limit-switches)).
- **A second sensor staggered along each track** raises the countable speed about 1.5 times if read as a forward-only
  four-step cycle, but only where press plus stagger is shorter than a ball, which at the entrance it isn't ([Two
  sensors staggered along the path](#two-sensors-staggered-along-the-path)).

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
- These pairs assume balls in line on a flat surface; the figures in [The plan](#the-plan) and [One beam per
  track](#one-beam-per-track-considered-not-chosen) use the CAD's paths instead.
- The order reader is ordinary quadrature, which needs every state seen; that is why it is so slow. A forward-only
  reader does far better ([Two sensors staggered along the path](#two-sensors-staggered-along-the-path); inferred).
- Assumptions: balls in line, at constant speed, touching or as stated, no slip or bounce, on a flat surface; a thin
  beam unless a cone is named; each sensor updating every 10 or 20 ms at a random phase; the hub's ADC refreshing
  every 9.7 to 11.1 ms (measured, [`hub-bench-findings.md`](hub-bench-findings.md)); a 10 ms poll with 0.3 ms jitter;
  at least 2000 runs per case. The order reader takes ±1 per covered/clear step; `TransferCounter` uses its defaults
  (threshold 25 mm, hysteresis 10 mm). These are flat-track pairs, not the drawn entrance pair.
- How the sensor reads a ball that half fills its field during one ranging window decides most of this and is
  unknown.

**What would settle it:**

1. Bench, one Laser Rangefinder configured with `roiSize` 4, SHORT and a 10 ms budget: does it give a new distance
   every 10 ms (roll a ball slowly through the beam and log every poll; the trace steps once per update), and does it
   read a Pollen right at 39 mm and a Nectar at 20 to 25 mm, with a plate 104 to 123 mm out behind? Slide a ball
   across the beam and note where the reading crosses the threshold: whether it switches at a quarter, a half or
   three quarters of the field decides which laser changes first between touching Pollen. Run two side by side, 22 mm
   apart, for crosstalk ([Parts](parts-findings.md#still-unknown)).
2. Before switching the robot's two lasers to analog, run Configure Laser Rangefinder on each and record the region,
   timing and mode it reads back.
3. Hardware or CAD: is the sensor mount in the main model yet; where
   does the intake arm rest; does the sweeper bar's working pose enter any beam; will the Nectar paddle ever close
   (recheck [The plan](#the-plan) if it does)? And ask the official Q&A whether break-beams are legal: our reading
   says yes ([Break-beams and the rules](#break-beams-and-the-rules)).
4. Still: load balls one at a time with the paddle closed and read each sensor at rest. Expect lasers A and B covered
   with the 4th Pollen in, and A, B and the Color Rangefinder with the 3rd Nectar.
5. On the robot, log both lasers, the hue and the intake motor's encoder at every 10 ms poll while feeding 4 touching
   Pollen, 3 touching Nectars and mixed trains 20 times each, at full intake power and at 55 to 70%. Play the rule
   over the logs, and from the same logs measure the ball's speed at the entrance against intake power. If more than
   1 in 20 count low at the power chosen, slow the intake further, or try break-beams polled every 5 ms. From the
   same logs, find the Nectar flag's real rates: the share of Nectars that raise a red or blue hue during their
   laser B spell, and any red or blue hue during a Pollen's. Every count figure above assumes every Nectar is flagged
   and no Pollen is.
6. The rule's guards, in simulation and then on the robot: hold a lone Pollen in laser B alone for a second with the
   intake running and let it go (step 3's cap); shoot from a full robot and check the count after each shot (step
   5's raised count); and with 4 Pollen in and a 5th pushed under the intake, run the 5th-ball reverse and check that
   only the 5th leaves. None of these were simulated.
7. Colour, on the bench: the Color Rangefinder's hue and distance for red, blue and Pollen balls at 20, 25, 29, 35,
   50 and 60 mm; with only a plate at 120 mm; through a 12 mm hole; a ball slid sideways at 29 and 50 mm to map its
   field of view; and a Nectar at 29 mm with a Pollen partly in view at 50 mm. Then the drop test
   ([Parts](parts-findings.md#still-unknown)): Nectars past the window at about 29 mm, dropped from 51, 73, 130, 270
   and 345 mm (1.0, 1.2, 1.6, 2.3 and 2.6 m/s), logging the hue at every poll.
8. Colour, on the robot: 20 runs of 3 mixed-colour Nectars at full intake power, reading the 1st and 2nd as they pass
   and the 3rd at rest; then the putting-out sequences above, counting what else leaves.
9. Log intake and transfer current and encoder speed while feeding 1 to 4 balls with the paddle closed: a step when
   the 2nd Nectar comes to rest or the mouth jams would make a jam and full alarm.
10. A slow-motion video of the intake arm as balls pass: whether the wheels follow a ball or let it coast, where the
   arm rests, and whether a lone Pollen stalls in laser A's beam.

#### Limit switches

**Don't count balls coming in with goBILDA's roller-lever micro limit switch** (3103-0001-0001; the part is in
[Parts](parts-findings.md#gobilda-micro-limit-switch-roller-lever)). Keep the two entrance lasers for the count ([The
plan](#the-plan)). A switch earns a place, if at all, only beside them as a "Pollen track full" signal, and only after
the bench tests below. What would change this: a lone ball measured at 1.5 m/s or less where a switch would sit, or 2
mm or more of travel measured after the click. The "track full" use and the encoder-port pair below both hang on the
lever's force, which nobody has measured; neither is established until it is. Nothing here was tried on a switch or on
the robot.

Why:

- **Travel against side play.** The roller moves at most about 4.3 mm up (about 4.4 mm counting its small sideways
  shift) before the lever meets the body, and how much of that comes after the click is unknown: 0 to about 4 mm
  (vendor STEP), or about 0.7 to 2.3 mm with Omron's figures as a stand-in (inferred;
  [Parts](parts-findings.md#gobilda-micro-limit-switch-roller-lever)). The only room found for a switch body beside
  the Pollen's path is inside the hollow +x cheek, at the 4th Pollen's rest (y 73, z 40) (inferred from the CAD's
  meshes; not one of the CAD's checked layouts). A spot on the guide face at y 86 to 100, where V1 pins the ball, was
  not looked at for fit; the resting 4th Pollen never reaches it. At the cheek the Pollen has the cheeks' 5.1 mm of
  sideways play ([The intake hand-off](#the-intake-hand-off), Beside the path), more than the whole travel. A ball on
  the switch side bottoms the lever unless the lever shoves it across within about 20 mm of travel, which takes about
  57 m/s² at 1.5 m/s and 135 m/s² at 2.3 m/s; Omron's operating forces, 0.16 to 0.49 N max (SS-5GL2-F, SS-5GL2), on
  a 24.9 g ball give 6 to 20 m/s² (inferred; neither that force nor that mass is verified).
- **A short press against a 10 ms read.** Set 1.5 mm into the path, the cheek switch is pressed over about 16 mm of a
  passing Pollen's travel, 7.0 ms at 2.3 m/s, with the click assumed 1 mm in (simulated). A 10 ms read then catches a
  lone Pollen 70, 57 and 46% of the time at 2.3, 2.8 and 3.5 m/s, and counts all 4 of 4 lone balls in only 23, 11 and
  4% of runs (simulated): an undercount, the costly error. At 1.5 m/s it catches every one. The SDK has no command
  that latches a digital input or counts its edges ([`sdk-hub-findings.md`](sdk-hub-findings.md#digital-inputs)). The
  entrance lasers see a passing Pollen for 21 to 23 ms at 2.3 m/s, so as a counter of passing balls the cheek switch
  is worse than either laser and worse than a break-beam.
- **No spot both types press within the lever's travel.** After the hand-off the Pollen rolls on the floor plate and
  the Nectar rides the cheek crests, its bottom 10.6 mm higher at y 45 and 24.7 mm at y 60 (CAD). A switch in the +x
  face at y 62 to 66, z 45.6 that a Pollen presses 1.5 to 3 mm is pushed 4.4 to 7.7 mm by a Nectar, at or past the
  lever's travel (simulated).
  Nothing holds a ball against a lever but its own weight and, at the lip, the intake's push, which is part of why
  Nectar is unreliable (CAD).
- **The CAD's ratings** (the CAD on 2026-11-15, each layout designed and then checked again against the meshes):

| layout | rating | why |
|---|---|---|
| hinged flap at the floor-plate lip, switch in a pocket in the −x cheek (a new part) | orange, Pollen only | a Nectar lifts it only when centred; see the flap, below |
| bare roller at the lip | red | the switch body would reach z −5.7; the plate is only 5 mm above the tile |
| roller on the side of a cheek | red | a Pollen presses it anywhere from 0.75 to 5.85 mm, a Nectar 0 to 13 mm |
| one switch per track | red | a crest roller goes from 5.8 mm to 0 with 1 to 2.5 mm of Nectar side shift, and clips passing Pollen |
| pair 17 to 20 mm apart at the one spot both types cross | not possible | the Nectar is off the floor by y 57, and at y 69 the resting 4th Pollen sits on the second switch |
| "Pollen track full" in the +x cheek, y 73, z 40 | not checked | |

**The flap, the one layout the CAD rated better than red** (CAD, 2026-11-15). A new part: a light flap over x −55 to
−10 and y 50.5 to 62, hinged at its rear edge (y 62), its front edge 3.5 mm above the plate, with the switch upside
down in a pocket in the −x cheek (body about x −80.2 to −73.8, y 52.8 to 72.7, z 21.9 to 32.2).

- A Pollen lifts it 3.5 mm over 42.5 mm of travel, about 18 ms at 2.3 m/s, wherever it sits sideways.
- A Nectar lifts it 3.2 mm over 11 mm only when centred, 2.3 mm over under 1 mm when 5 mm off centre, and not at all
  10 mm off; a Nectar can sit up to ±11.4 mm off centre between the plates. So it can't count Nectar.
- With the roller 10 mm behind the hinge the switch sees only about 3.0 mm, which may not trip it. The CAD's check
  would put the roller 13 mm behind the hinge (about 4.0 mm) with a hard stop at flush.
- The resting 4th Pollen and 3rd Nectar leave it untouched; with the paddle open the 7th Pollen (s −8.6) holds it
  down 3.3 mm. A ball rolling back out lifts it again, so the code must count down then.
- It isn't in the plan and doesn't replace the lasers: it is rated orange, it is a new part, as drawn it may not trip,
  it can't count Nectar, and nothing about it has been built or tested beyond the CAD.

**The one use left: "Pollen track full"** (inferred; not checked in the CAD). A switch in the +x cheek face where the
4th Pollen rests with the paddle closed (s 26.3: y about 73, z about 40) should stay pressed while 4 Pollen are in, if
the switch clicks within about 1.5 mm and the lever doesn't push the resting ball across: set 1.5 mm into the path, it
is pressed exactly 1.5 mm by a resting 4th Pollen pushed to the far cheek. A lone Pollen stalled at s 20 to 46 would
press it too, a false "full". Wired REV's way, an open wire reads as pressed, so a broken wire stops the intake early,
the safe direction ([Parts](parts-findings.md#gobilda-micro-limit-switch-roller-lever)). Whether a resting ball holds
it pressed despite the side play is bench test 4 below. It gives no colour, so the Color Rangefinder stays. Lasers A
and B already read covered with the 4th Pollen at rest ([The plan](#the-plan)), so the switch would only be a second
signal.

**Don't build the two-switch pair on an encoder port.** Two switches on a hub encoder port's A and B inputs,
staggered so each ball holds both at once, make the hub count ±4 per ball with direction, and on hardware-counted
ports 0 and 3 no poll can miss an edge; one switch alone counts −1 then +1, net zero (inferred). On this robot:

- Its best settings need more travel than the part has. The best depth for Pollen is about 4.4 mm past the click,
  which, added to the travel before the click, is more than the roller's whole travel. At depths that can be built,
  1 to 2 mm, the switches' edges are 8.7 to 12.2 mm of travel apart: 3.8 to 5.3 ms at 2.3 m/s and 2.5 to 3.5 ms at
  3.5 m/s, against a bounce time nobody knows. In a simulation, with bounce longer than the gap between an A and a B
  edge an edge counter was wrong 213 times in 400, and with bounce as long as that gap one sampled at 1 kHz was
  wrong 220 times in 400.
- The CAD found no spot for a pair (table above): the only surface both ball types ride along is the tile, and the
  one spot both roll over, the floor plate's front lip, has no room for a pair.
- A ball that hops off both levers is lost for good. In a simulation a lone Pollen at 2.2 m/s hopping 5 mm was lost
  at 10 of 41 take-off points, and hopping 10 mm at 14 of 41. A lid over the pair, or a guide that presses balls
  against the switches, would prevent it (inferred).
- The only free hub encoder port, Control Hub encoder 1, counts in software, at a rate and with filtering nobody
  publishes. It is also the front-right drive motor's port: if anything puts `fr` in RUN_USING_ENCODER or calls
  `setVelocity` on it, the hub's speed loop reads ball counts as wheel speed, and a STOP_AND_RESET_ENCODER zeroes the
  count (inferred).
- Wiring a switch across the port's 3.3 V risks the flywheel encoders:
  [`wiring-findings.md`](wiring-findings.md#switches-and-onoff-sensors).

The rules (manual): no rule names limit switches or says what may go on an encoder port. R706 A lets "wires, cables,
and signal lines" connect "via the standard connection points provided on the devices"; R613 A and B allow powering
a device only from the port it plugs into; and a passive switch is not a CUSTOM CIRCUIT, which R607 defines as "any
active electrical item that is not an actuator ... or power regulation device" (the last point is our reading).

**Bench tests, in order, before anyone cuts metal for a switch:**

1. Measure the real ball speed at the hand-off and on the plate:
   [the one-rangefinder measurement](#the-measurement-that-settles-most-of-it) plus slow-motion video. If lone balls
   move at 1.5 m/s or less where a switch would sit, one switch can count passing balls at a 10 ms read (100% in
   the model); otherwise keep switches to "track full".
2. Measure one switch with calipers and a gram scale: the roller's released, click, release and bottom heights, and
   the force at the click and at the bottom. Check that the roller moves at most about 4.3 mm. Under about 1.5 mm
   left after the click rules out a switch fixed in the cheek wall (the side play is 5.1 mm): use a spring-mounted
   switch or none. At 2 mm or more, look at it again.
3. Wire it REV's way to a Control Hub digital port and read its bit from the bulk read `TransferCounter` already
   sends at a 10 ms poll (no code reads a digital pin yet).
   - Pulse the input for 2, 5 and 8 ms (from another hub's digital output or a test rig) and log the share of pulses
     seen. A share above the pulse length ÷ 10 ms means the hub holds or latches the level, and counting passing
     balls gets better.
   - Unplug the switch and confirm it reads pressed.
   - Run about 10,000 presses and log any dropout, for the contacts' sake. One dropout rules the part out.
4. Weigh 5 Pollen and 5 Nectar. Then, on a printed 76.2 mm channel with a mock-up of the crest:
   - Roll Pollen at the measured speeds, hugging the switch wall and then the far wall. Record any bottoming, kicks
     and stalls, and whether a resting ball holds the switch pressed.
   - Push a Nectar against a roller in the crest to see whether the roller lifts it.
   - If the lever bottoms or bends, or a resting ball holds it pressed in fewer than 20 of 20 tries, drop the cheek
     switch. If a Nectar lifts, drop any Nectar switch.
5. In the same rig and the same runs, compare the switch with a break-beam at z 16 and with the entrance lasers. If
   the break-beam counts 4 balls right in 99% or more of runs and the switch doesn't, the break-beam counts and the
   switch, if kept at all, is only "track full".
6. Only if an encoder-port pair is still wanted:
   - Measure the voltage on Control Hub encoder 1's channel A with nothing plugged in.
   - If it shows a pull-up, wire each switch between its input and ground only, never to the 3.3 V pin.
   - Press them by hand in order: expect ±4 per ball.
   - Then run 500 balls at the measured speed on port 1. Anything but 0 errors in 500 abandons it. If the errors drop
     with the pair on the OctoQuad's hardware-counted channel 2, software counting was the cause. The OctoQuad has no
     pull-ups on its A and B inputs (datasheet Rev 3.0C), so there each switch needs a resistor to its own plug's
     3.3 V (R613 B; inferred).

#### Break-beams and the rules

- **Our reading: break-beams are allowed. Ask the official [Q&A](https://game-qa.firstinspires.org/boards/2027/FTC/QA)
  before relying on it.** A break-beam is active, so it is a CUSTOM CIRCUIT: R607 calls "any active electrical item
  that is not an actuator ... or power regulation device" one. R607 lets CUSTOM CIRCUITS be used as long as they
  "shall not provide regulated output voltages exceeding 5V, except if solely used for powering LEDs", and R613 A and
  B require power only from the hub port the device plugs into (manual). Adafruit's 2167 runs on 3.3 to 5.5 V, so the
  digital port's own 3.3 V can power it (inferred).
- REV says every hub digital input has a pull-up resistor, which the 2167's open-collector output needs; REV gives
  no value ([Parts](parts-findings.md#gobilda-micro-limit-switch-roller-lever)).
- A read gives only the level the hub reports at that moment: the SDK has no command that latches a digital input or
  counts its edges, and whether the firmware holds a short pulse is unmeasured
  ([`sdk-hub-findings.md`](sdk-hub-findings.md#digital-inputs)). The break-beam rows in [One beam per
  track](#one-beam-per-track-considered-not-chosen) and the 5 ms figures in [The plan](#the-plan) assume a fresh level
  at every 5 or 10 ms read, which nobody has measured.
- For the low Pollen beam not chosen, a break-beam a little behind P66, at (y 76 to 78, z 16), would need holes
  through the two side plates only, no tunnel through a cheek, and the resting 4th Pollen still blocks it (inferred
  from the CAD's meshes). Room for the sensor bodies outside the plates (the pulley flange from y 83.7, the −x plate's
  opening from y 81) is unchecked.

#### Two sensors staggered along the path

This holds for any two on/off sensors a ball passes one after the other: switches, break-beams, or a laser read
against a threshold. All of it is simulated and derived; nothing was tested. The model reads a fresh level at every
read; a laser's own update period counts as extra gap between reads (inferred).

- **Read the pair as a forward-only four-step cycle.** With sensor A upstream and B a short stagger downstream, the
  pair goes 00 → 10 → 11 → 01 → 00 once per ball. Count each change between two reads as 0 to 3 steps forward,
  and one ball per 4 steps. On touching trains that raises the highest speed at which 99% of 4-ball trains count
  exactly about 1.5 times over one sensor: Pollen from about 2.55 to 3.85 m/s, Nectar from about 3.35 to 5.05 m/s
  (reads 0 to 3 ms late, 1 ms bounce).
- **Why** (derived; the simulation lands on these limits): one sensor must see every press and every release, so a
  ball may move at most half its diameter between reads. The four-step reader loses a ball only when one gap between
  reads spans all four changes, which allows about three quarters of a diameter. Ordinary quadrature, ±1 per step,
  must see every state: a quarter of a diameter, half the speed of one sensor.
- **Press length and stagger.** Make the press (the stretch of travel over which a ball holds the sensor on) about
  half a ball, and the stagger about a quarter of a ball. The stagger must be shorter than the press, and press plus
  stagger shorter than a ball, or the cycle breaks and the count falls to zero. The overlap, press minus stagger, must
  be at least speed × bounce time.
  - Pollen: stagger 18 to 19 mm, never more than 21 mm (from about 23 to 24 mm the count falls to zero). A switch
    pressed about 4.5 mm deep into the ball gives about half a ball of press.
  - Nectar: stagger 23 to 24 mm, safe from about 18 to 30 mm; a switch pressed about 6 mm deep.
  - Those depths are past the click, so the goBILDA switch, whose roller moves at most about 4.3 mm, can't reach
    them (inferred; [Limit switches](#limit-switches)).
  - Depth matters more than stagger. Within ±1 mm on both, Pollen still counts to at least 3.6 m/s at 1 ms bounce;
    within ±2 mm, 3.3. ±3 mm is not safe for Pollen: at 3 ms bounce it falls to 2.2 m/s even at the best mount.
- **What it costs:**
  - Overcounts when a ball goes backward: +1 each time a stopped ball rocks across any of the four edges, and +1 to
    +4 when a ball backs out (one sensor does the same on its two edges). For example, a ball rocking ±1.5 mm at
    20 Hz for 1 s adds 20 with 0.5 mm of differential travel. Counting only while the intake runs forward doesn't
    stop it; only enough differential travel (a switch's hysteresis) does: in the model 1 mm stopped ±1.5 mm rocks
    and 3 mm stopped ±3 mm, because the band it gives along the path depends on depth. A break-beam has none; a laser
    gets it only from hysteresis in the code's threshold (inferred). Whether balls rock or stall while the intake runs
    is unknown.
  - Both pins must come from one bulk read. With bulk caching off, each digital read is its own hub command (SDK
    12.0.0 source), so two pins are read at two moments. For Pollen at 1 ms bounce, 1, 2, 3 and 5 ms between the two
    reads cut the limit from 3.85 to 3.6, 3.4, 3.15 and 2.8 m/s: 2 ms costs about 0.45 m/s, 5 ms most of the gain.
  - The longest gap between reads sets the limit, not the 10 ms average. If 1% of reads come 20 ms late, Pollen at
    2.2 m/s counts exactly in 0.927 of runs with the four-step reader and 0.917 with one sensor; at 3.5 m/s, 0.94
    against 0.20.
  - A second sensor and pin per track.
- If one sensor sticks, the four-step reader falls back to counting like one sensor.
- **Before adding a second sensor, make the one sensor's press about half a ball long**: that alone gives the
  one-sensor row below.
- Whether Pollen at 3.5 m/s counts right depends on bounce and on the longest gap between reads, and neither is
  measured: 3.85 m/s at 1 ms bounce, 3.35 at 3 ms.
- **The laser pair drawn at the entrance fails these conditions** (inferred from the CAD's paths; simulated). Along
  the Pollen path the lasers are 17.4 mm apart where a ball enters them and 22.0 mm where it leaves, and a Pollen
  holds each for 48 to 54 mm, so press plus stagger is 66 to 71 mm against a 71.1 mm ball: on the edge. For a Nectar
  it is 92.5 + 21.7 = 114.2 mm against 91.4. In the simulation the four-step reader never counted 3 touching Nectars
  right in 99% of runs at any speed down to 0.4 m/s (at best about 75% on the 10 ms laser and 91% on a sensor read
  every 5 ms). It reached 99% for 4 touching Pollen only with a fresh reading every 5 to 13 ms and the beams' edges
  exactly where modelled; with the lasers it never did, and with the edges off by ±3 mm no sensor did, even at
  0.4 m/s. The covered-time rule in [The plan](#the-plan) counts there instead. No spot for a switch pair has been found
  ([Limit switches](#limit-switches)).

Highest speed at which 99% of 4-ball touching trains count exactly, at 1 ms / 3 ms bounce (simulated):

| reader | Pollen (m/s) | Nectar (m/s) | error when it fails | what breaks it |
|---|---|---|---|---|
| one sensor | 2.55 / 2.2 | 3.35 / 2.9 | undercount when too fast; overcount when a ball rocks or backs out on its edges | long gaps between reads |
| A or B (either on) | 1.25 / 1.1 | 1.7 / 1.45 | undercount | too little time off between touching balls; on a lone ball or a short press it equals the four-step reader |
| ordinary quadrature | 1.25 / 1.1 | 1.6 / 1.4 | undercount | every state must be read |
| forward four-step | 3.85 / 3.35 | 5.05 / 4.35 | undercount when too fast; +1 per rock on any of the 4 edges, +1 to +4 on a back-out | read skew, long gaps between reads, bounce longer than the time both sensors are on |
| hybrid, −1 to +2 per change | 2.45 / 2.15 | 3.35 / 2.9 | undercount | speed; it is immune to rocking and back-outs |
| larger of two separate counts | 3.15 / 2.5 | 4.05 / 3.25 | undercount | with no errors at all, only about 0.15 m/s above one sensor (2.35 → 2.5 for Pollen) |
| four-step and larger-of-two together | as four-step | as four-step | as four-step | adds nothing measurable |

- Model: rigid trains at constant speed; Pollen with the sensors pressed 5 mm deep and 18 mm apart, Nectar 6 mm and
  23 mm; 1 mm of differential travel; reads every 10 ms plus 0 to 3 ms, never early; a read within the bounce time
  after an edge returns a random value; 2000 runs per case, speeds in 0.05 m/s steps. The A-or-B and ordinary
  quadrature rows come from runs with Pollen 5 mm deep, 0 mm of differential travel and 17.8 mm apart, and Nectar
  6 mm, 1 mm and 22 mm. The depth, stagger and tolerance figures above come from a grid of 600 runs per case in
  0.1 m/s steps, worst case over 0, 1 and 3 mm of differential travel. Not modelled: a ball off centre in the track
  (a Pollen has 5.1 mm of sideways play at the cheek) and the roller's radius.
- Moving steadily forward, no reader overcounted, except the four-step reader with 5 ms between its two pin reads and
  3 ms bounce, and shallow mounts where the overlap is shorter than speed × bounce (there both signs: for example
  0.9% under and 0.7% over at 2.0 m/s).

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
  its CAD model and ST's VL53L1X datasheet. Its region of interest and stored settings: Brushland's full docs text
  <https://docs.brushlandlabs.com/llms-full.txt>, read 2026-11-15, and the live ranging configuration page
  <https://docs.brushlandlabs.com/sensors/laser-rangefinder/ranging-configuration>, read 2026-11-16.
- Competition Manual (G407, G408, CONTROL, MOMENTARY, R607, R613, R706, CUSTOM CIRCUIT):
  <https://ftc-resources.firstinspires.org/ftc/game/cm-html>, read 2026-11-15.
- Adafruit 2167 break-beam sensor: <https://www.adafruit.com/product/2167>, read 2026-11-15.
- goBILDA micro limit switch, roller lever, 3103-0001-0001:
  <https://www.gobilda.com/micro-limit-switch-roller-lever-2-pack/> and its STEP file
  <https://www.gobilda.com/content/step_files/3103-0001-0001.zip>, read 2026-11-15; Omron's SS datasheet as a
  stand-in, <https://omronfs.omron.com/en_US/ecb/products/pdf/en-ss.pdf>. Details: [Parts](parts-findings.md).
- REV, wiring a limit switch to a digital input: <https://docs.revrobotics.com/duo-control/sensors/5v-sensors>, read
  2026-11-15.
- OctoQuad datasheet Rev 3.0C (no pull-ups on A and B):
  <https://github.com/DigitalChickenLabs/OctoQuad/tree/master/documentation>.
- The simulations of switches and staggered pairs: models written on 2026-11-15, not in the repo; their assumptions
  are given with each result. The simulation of the entrance lasers and their counting rule: a model written on
  2026-11-16, not in the repo; its assumptions are given with its results.
- The CAD numbers: the team's CAD as of about 23:00 PST on 2026-11-14, and as of 13:13 PST on 2026-11-15 for the
  paddles, where balls rest and everything near the intake, with both turrets placed on the transfer, and as of the
  evening of 2026-11-15 for the limit-switch layouts and the mid-height beam, and as of 2026-11-16 for the sensor
  placement at the transfer entrance.
- The code facts: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/tele/BioBuzzBasicTele.java`.
