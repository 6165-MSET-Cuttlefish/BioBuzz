# Transfer findings

How fast balls move through the BIOBUZZ robot's transfer, how to measure it, and where rangefinders can see them.
Read it before mounting or moving a transfer sensor, choosing a sensor's timing or sampling rate, writing code that
counts or times balls, or trusting a feed rate in a shooter simulation.

**Nothing on this page has been run on the robot.** Each number says how it is known:

- **(CAD)**: measured on the team's CAD as of the evening of 2026-11-14 (about 23:00 PST). The CAD was still changing.
- **(vendor)**: from the vendor page linked under [Sources](#sources), read on 2026-11-15 unless it says otherwise.
- **(team)**: the team said so; not checked.
- **(inferred)**: worked out from the above; the reasoning is given.

## What to do

- **Take the one measurement below before trusting any speed here.** It tells which part of the transfer sets the
  ball rate, and it needs only one rangefinder and Transfer Thread Count Test.
- **The transfer's continuous servo `melonbotics` is a Melonbotics Super Servo Plus** (CAD). It sits on the lower
  rear Pollen roller shaft. The only Axon MAX in the CAD turns the front sweeper bar's hinge
  ([Parts](parts-findings.md#servos)).
- **Bench-test a Laser Rangefinder at 15 to 40 mm, in SHORT mode at a 10 ms timing budget, before anyone drills a
  hole for one.** No vendor page gives a minimum range, every spot below has the ball 15 to 41 mm from the sensor
  face, and 10 ms is the setting the spots need.
- **Use a 10 ms timing budget (SHORT mode) for any transfer rangefinder.** At the default 20 ms only spots A and B can
  work, only with the transfer motor loaded, and maybe not even then (the beam is a cone: see
  [below](#ball-and-gap-times-and-which-sampling-rate-works)). If the bench test shows the sensor can't range at
  10 ms, no Nectar spot works, and Pollen works only at A or B with the motor loaded, if at all.
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
- At free speed a ball shows for about 25 ms and the gap for about 21 ms, or less (below). At a 20 ms budget a gap
  can fall between two readings, two balls then count as one, and one period reads about double. If the count isn't
  5, or one period is about twice the others, repeat at a 10 ms budget.
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

Speeds in mm/s, free / loaded. s is mm along the ball's path from the front edge of the transfer floor plate.

| where | what touches the ball | surface | ball centre | contact (s) |
|---|---|---|---|---|
| Intake, on the floor | three 2 in compliant wheels, r 25.4 (CAD), at 1620 rpm | 4309 / no figure | Pollen 2304, Nectar 2238 / no figure (no intake current known) | before s 0 |
| Pollen side rollers | two vertical 2 in compliant wheels on the left, ball pinched against a fixed guide (CAD) | 3058 / 1606 | 1529 / 803 | 49–55, 99–108 |
| Pollen ramp | nothing in the CAD: the servo's 2 in roller misses the ball by 0.9 mm, and the next roller, an unbelted idler, grazes it at s 232 (CAD) | | moves only when pushed | 109–280 |
| Servo roller, if a real compliant wheel touches | Super Servo Plus at 1000 rpm; 833 or 800 rpm if it runs at 5 or 4.8 V (inferred, below) | 2659 | 1329; 1107; 1064 | within 109–280 |
| Pollen spokes | surgical-tube spokes on the two upper rear shafts, 2 lower, 1 upper (team; not in the CAD), tip r 33.2 (team's "about 28 mm" tubes + a 5.2 mm base, assumed) | 3998 / 2100 at the tip | at most 1999 / 1050 (one side against the rear wall) | 280–416 |
| Nectar ramp | leaving the intake | | 1610 free (inferred, 0.374 × the intake surface) | |
| Nectar spokes | tubes on the two front shafts, the lower coaxial with the motor (team), the upper 1:1 by belt (CAD), tip r 42.7 (team's 37.5 mm tubes) | 5142 / 2701 at the tip | 0.363 to 0.5 × tip: 1867–2571 / 980–1350 (inferred: the ball rolls in a trough that fits it) | 29–202 |

- Spokes push in pulses, 4 per turn per spoke: 77 per second free, 40 loaded (inferred).
- The intake arm floats and the CAD has no stop for it, so its resting height is unknown. If it rests 15° or more
  above the drawn pose, its wheels miss a Pollen ball completely (CAD + inferred).
- **The servo's speed.** The vendor gives 1000 rpm free with no voltage, next to "6 W at 6 V". 2.3 kgf·cm × 1000 rpm
  ÷ 4 = 5.9 W, the peak power of a motor with that stall torque and free speed, which matches 6 W, so 1000 rpm is
  very likely at 6 V (inferred). 833 rpm at 5 V and 800 rpm at 4.8 V assume speed scales with voltage (inferred). The
  wiring map powers it from servo injector B ([`wiring-findings.md`](wiring-findings.md)), which the CAD runs at
  6 V, so 1000 rpm is the likely case. No loaded speed is known. The vendor's "about 450–700 RPM" is advice on how to
  gear the servo, not a loaded speed.
- **Every part of the path runs at 1.1 to 2.6 m/s free**, so no part of it is much slower than the rest (inferred).

### Hand-offs

Balls arriving touching (inferred from the speeds above):

- **Intake to side rollers (Pollen):** 2304 to 1529, ratio 0.66, so the balls bunch with no gap. 44 to 107 mm are
  undriven between them, depending on the intake arm's height (CAD + inferred).
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
- Touching balls counted back from the turret: Pollen 6, centres at s 423, 350, 277, 205, 132 and 60; Nectar 3, at
  s 202, 110 and 17. A Nectar ball blocks the Pollen path while its centre is at s 72 or less.
- The CAD draws no part named blocker. What it has: each turret tab holds a part named "[LS] Pulley HTD5 20T", a 20T
  pulley with a flat 77 mm paddle on a vertical pivot hanging under the turret plate.
  - **Pollen paddle**, at z 120 to 123: swung 75° to 135° from its drawn pose it crosses the Pollen path at s 264 to
    294 and holds the first ball at s 242; 4 balls then queue, at s 242, 170, 98 and 27. Past about 138° it hits a
    side plate. In its drawn pose (0°) it is clear of the path, so that pose reads as open.
  - **Nectar paddle**, at z 241 to 244: it would hold a ball at s 143 with one more at s 50, but its post is drawn
    through a roller shaft, so as placed it can't be the real part.
  - **A goBILDA 2000-0025-0504 five-turn servo** with a hub and a matching 20T pulley sits unmated in the Pollen turret
    tab, far from the paddle. Very probably the blocker's servo, with the belt not drawn (inferred). As parked, its
    output axis is horizontal and the paddle's is vertical, so no belt could join them in that pose.
  - **Code range to angle** (inferred): Basic Tele and Intake Transfer Shooter Test move `blocker` between 0.53
    (closed) and 0.60 (open), and nothing calls `setPwmRange` on it, so the SDK's default 600–2400 µs applies:
    0.07 × 1800 µs = 126 µs, × 0.90°/µs on this servo (vendor: 1800° over 500–2500 µs) = 113°. That is only the size
    of the swing. If 0.60 put the paddle at its drawn open pose, 113° would land it inside the 75° to 135° that holds
    a ball, but the CAD doesn't model where the servo's zero sits against the paddle, so nothing here says where 0.53
    and 0.60 put it.
  - **The code disagrees with itself:** `biobuzz/modules/Shooter.java` calls 0.6 BLOCKED and 0.3 UNBLOCKED
    (`ShooterHardstopState`), where Basic Tele calls 0.6 open. On this servo 0.3 apart is 486° (inferred), far past
    the paddle's swing.

What the docs already said about the blocker: it is "the blocker before the shooters"
([`robots-findings.md`](robots-findings.md)), `blocker` on Expansion Hub servo 4 powered from injector B
([`wiring-findings.md`](wiring-findings.md)). Nothing named its servo or where it acts.

Questions for hardware:

- Which paddles exist on the robot: Pollen, Nectar or both? The code has one `blocker` port for two drawn paddles.
- Which servo drives each, through what belt, and which `blocker` position is closed?
- Can the turrets turn at all with the paddle posts there? Each post hangs 76 to 81 mm from its turret's axis, so
  most turret angles would run it into the side plates (CAD + inferred).

## What the CAD cannot tell

- Whether the servo's roller touches a ball (it misses by 0.9 mm as drawn), and whether the servo or a belt drives it:
  a 285 mm belt was also drawn that evening from that shaft to a motor-driven one, but the pulley for it is missing
  and it cuts through another pulley, so it looks unfinished. If the motor drives that roller, the ball there moves at
  1529 / 803 like the side rollers.
- The servo's voltage and loaded speed. Also: as placed, the servo body reaches through the drivetrain's inner plate
  and about 13 mm into the Control Hub's outline (inferred from the CAD's placement, good to a few mm). Is there a
  cut-out, or is it mounted elsewhere?
- The intake's loaded speed and resting height; how far the tubes bend; how the Nectar ball rolls in its trough.
- Which blocker paddles and servo exist; the Nectar turret's rotation range.

The measurement [above](#the-measurement-that-settles-most-of-it) settles the first two, and the free or loaded
state.

## Rangefinder spots

### What to tell hardware

- **Which plate.** The CAD study calls the sensor plate "the left plate". Find it by what is outside it: the plate
  with the Super Servo Plus mount bolted to its outer face, on the Control Hub's side (CAD). Go by the servo mount,
  not the word "left": the CAD's own frame puts robot left on the other side, so which side the study meant by
  "left" is unclear (inferred).
- **Pollen: spots A and B as drawn don't fit side by side.** Each body is 20 × 14 mm on the plate; with its 20 mm side
  across the ball's travel, 14 mm runs along it. A's and B's beam points are only 13.4 mm apart in a straight line,
  so with each beam at its body's centre the bodies overlap by about 0.6 mm (inferred). Before drilling, either ask
  CAD to move A earlier along the path (lower s, toward the intake) until the bodies clear, and recheck its cone, or use B with C instead
  (36.5 mm apart, fits; C sees a gap of only about 4 ms at free speed, so C tells only which way a ball moved). Where
  the beam sits on the body isn't on the pages read; take it from Brushland's STEP file. Use the narrow 15° field
  of view.
- **Bench-test one rangefinder at 15 to 40 mm, in SHORT at a 10 ms budget, first.** If it can't read that close, the
  sensor has to stand back from the plate on spacers. For a 40 mm minimum range (the guess [below](#the-sensor)) that
  is about 5 mm at A and B, 12 to 14 mm at C and 21 to 22 mm at D and E (40 minus the nearest face-to-ball distance;
  inferred). Each mm of stand-back puts the body 1 mm farther out behind the plate and widens the hole the cone
  needs (below). Behind the plate at A, the body is already only 4 to 5 mm from a standoff of the servo mount, and
  nobody has checked for 21 mm of room behind D and E: check in CAD before drilling.
- **Never in s 266 to 298**: the Pollen paddle swings through the beam there.
- **Nectar: spots D and E**, both on the same plate as A and B; the spot at s 150 only once the Nectar turret's
  rotation range is known. No Nectar spot passes at a 20 ms update. D and E are 16 mm apart along the path and
  18.1 mm in a straight line, so their bodies leave only about 2 to 4 mm between them: check the fit. The Nectar turret
  hits E's body if it turns to +108° to +114° (CAD).
- The two cones overlap on the far plate, so the CAD study advises firing the two sensors alternately (an inferred
  crosstalk risk) and never mounting two facing each other across the channel. In analog mode both range on their
  own, so the bench test should also run two side by side.

### Where they go

Frame (CAD): the transfer's own, in mm. x runs across the channel, y toward the rear, z up. Each beam runs along x.
The sensor face sits on the outer face of its plate (x −92.75), looking through a hole drilled in the plate (x
−92.75 to −89.58, 3.2 mm thick); the far plate's inner face is at x 24.72, 117.5 mm from the sensor face. The channel
between the two plates' inner faces is 114.3 mm wide. Face to ball is the distance from the sensor face to the ball
along the beam (CAD).

To mark a hole on the plate: **back** = y − 46.9, how far behind the front edge of the transfer floor plate (that
edge is at y 46.9 and is where s = 0); **up** = z − 5.0, how far above the side plate's bottom edge (the plates' bottoms
are at z 5.0, 5.0 mm above the floor; CAD). That assumes the bottom edge is straight, which nobody checked.

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
  B's, so use 15°. A is 76 mm and B 56 mm before the ball held by a closed Pollen paddle (s 242), 257 and 237 mm
  before the turret. Body clearance: A is 4 to 5 mm from a servo-mount standoff, B 10 to 14 mm from anything.
- With the Pollen paddle closed, the second queued ball (s 170) sits on A: count edges, not presence.
- If the Nectar paddle is real and closed, a held Nectar ball (centre s 143) covers s 97 to 188, so D and E both read
  "ball present" the whole time: count edges there too.
- A and B are on the undriven ramp: a lone ball at loaded speed may not climb it and can roll back, so use the pair
  to tell direction (inferred).
- D and E: the spokes are 40 mm clear. D is 25 mm before the doubtful Nectar paddle's hold position (s 143); the
  turret is at s 202.
- Rejected (CAD): s 266 to 298 (the Pollen paddle swings through the beam), s 306 (2 mm from the paddle and the tubes),
  s 394, and an earlier pair at s 282, 290 and 306.

### Ball and gap times, and which sampling rate works

A beam h mm off the ball's centre line crosses a chord of 2 × √(R² − h²) of each ball, and the gap it sees between
touching balls is D minus that chord. Ball time = chord ÷ v; gap time = (gap + any gap a hand-off opened) ÷ v.

- Pollen at A and B (h 30, R 35.55): chord 38.1 mm, gap 33.0 mm.
- Pollen at C (h 15): chord 64.5 mm, gap 6.6 mm: 4.3 ms at free speed, so C tells only direction.
- A centre beam: chord = D, so touching balls show no gap at all.
- Nectar at D and E (h 21, R 45.7): chord 81.2 mm, gap 10.2 mm, plus the ramp-to-spokes gap when free.

**The beam is a cone, not a line** (inferred; not checked in CAD or on a sensor). At 15° it is 2 × distance × tan 7.5°
across: about 11 mm where it meets the middle of a Pollen ball at A or B (41 mm out) and 16 mm where the beam meets
the ball's edge (60 mm out, the chord's ends). The pages read don't say how much of the cone a ball must fill before
the reading changes, so a ball can look up to about one cone width longer, and the gap that much shorter, than the
chord says. Even with a ball centred on A or B, 14% of the cone misses it (CAD). The "with the cone" column below is
that worst case; the bench test with a ball moved at a known speed shows where the truth lies.

**The rule** (the CAD study's, inferred): a sensor sees a ball reliably when the ball and the gap each last at least
two sensor updates, so one bad reading can't make or hide a count: 40 ms each at a 20 ms update, 20 ms at 10 ms.
So the train must move at no more than gap ÷ (2 × update): 824 mm/s at 20 ms, 1648 mm/s at 10 ms for A and B, on a
thin beam. (For ball + gap together, the limits are D ÷ 80 ms and D ÷ 40 ms: 889 and 1778 mm/s Pollen, 1143 and
2285 Nectar.)

| at A and B, Pollen | speed (mm/s) | ball / gap (ms) | gap with the cone (ms) | 20 ms updates | 10 ms updates |
|---|---|---|---|---|---|
| side rollers set it, free | 1529 | 24.9 / 21.5 | 11–14 | fails | passes; fails with the cone |
| motor loaded | 803 | 47.5 / 41.0 | 21–28 | passes; fails with the cone | passes |
| servo meters at 833 rpm | 1107 | 34.4 / 29.8 | 16–20 | fails | passes; at best just passes with the cone |
| servo meters at 1000 rpm | 1329 | 28.7 / 24.8 | 13–17 | fails | passes; fails with the cone |

20 ms also passes on a thin beam if the servo meters at 620 rpm or less (824 mm/s × 2 ÷ (2π × 25.4) × 60). At 10 ms
the free case passes on a thin beam with only 1.5 ms to spare.

| at D and E, Nectar | ball / gap (ms) | gap with the cone (ms) | 20 ms | 10 ms |
|---|---|---|---|---|
| free, ball at 0.5 × tip (2571) | 31.6 / 25.2 | 19–23 | fails | passes; at best just passes with the cone |
| free, ball at 0.363 × tip (1867) | 43.5 / 13.3 | 5–11 | fails | fails |
| loaded (1350 to 980) | 60 / 8 to 83 / 10 | less | fails | fails |

For Nectar the cone is about 5 mm across at the ball's middle (20 mm out) and 16 mm at the chord's ends (60 mm out).
At 10 ms, Nectar passes on a thin beam only if the ball rolls at 0.43 × the tip speed or more (2211 mm/s gives a
20.1 ms gap).

### The sensor

From Brushland's pages ([Parts](parts-findings.md#brushland-laser-rangefinder)): 20 × 14 × 10 mm, 15° to 27° field
of view, up to 4 m, "up to 100Hz", M4 holes 8 mm apart; SHORT mode at a 10 ms budget gives 100 Hz (vendor). None of
the pages read names the chip or gives a minimum range or a timing budget floor.

- The CAD study guessed the chip is an ST VL53L1X (from the 16 × 16 receiver grid, the 27° field of view and the
  940 nm laser) and, from that chip's datasheet, a 20 ms minimum budget and 4 cm guaranteed minimum distance. That is
  unconfirmed: ST's datasheet was not opened for this page, and Brushland's own ranging page lists a 10 ms budget.
  If the guess is right, the sensor can't range at 10 ms, and every spot is inside its minimum range.
- Either way: every spot above is 15 to 41 mm from the ball, so bench-test at 15 to 40 mm in SHORT at 10 ms before
  drilling, and leave room for spacers.
- **The Color Rangefinder's range fits, but its speed and its colour dependence are unknown** (inferred, untested).
  Its distance mode tops out at 100 mm (vendor), the far plate is 117.5 mm from the sensor face, so an empty channel
  reads beyond its range, and at A and B the ball is 35 to 41 mm away. But it measures the strength of reflected
  light, not time of flight, so a yellow, a red and a blue ball may read different distances; its chip repeats a
  distance measurement every 6.25 to 400 ms depending on a setting the vendor doesn't publish, and a colour
  measurement every 25 ms at best ([Parts](parts-findings.md#brushland-color-rangefinder)). Against the table above:
  a new value every 6.25 ms is safe, 12.5 ms is workable, 25 ms or slower misses balls at free speed. Hue at 25 ms
  gives a ball passing in about 25 to 31 ms one reading at most. Until its rate is measured, the Laser Rangefinder at
  a 10 ms budget is the only one of the two with a published rate fast enough.

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
- Brushland Laser Rangefinder: see [Parts](parts-findings.md#brushland-laser-rangefinder) for the snapshots read.
- The CAD numbers: the team's CAD as of about 23:00 PST on 2026-11-14.
