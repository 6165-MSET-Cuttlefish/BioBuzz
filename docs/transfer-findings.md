# Transfer findings

How fast balls move through the BIOBUZZ robot's transfer, how to measure it, and where rangefinders can see them.
Read it before mounting or moving a transfer sensor, choosing a sensor's timing or sampling rate, writing code that
counts or times balls, or trusting a feed rate in a shooter simulation.

**Nothing on this page has been run on the robot.** Each number says how it is known:

- **(CAD)**: measured on the team's CAD as of the evening of 2026-11-14 (about 23:00 PST). The CAD was still changing.
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
- **No spot for counting balls coming in has been checked in the CAD.** Don't build the two-laser direction pair at
  the intake hand-off: the two ball types share a surface only on the field tile in front of the plates, where nothing
  fixed can carry a sensor. The best-supported plan is one Laser Rangefinder per track, counting arrivals, with
  direction taken from the motor's commanded direction, and the Color Rangefinder on the Nectar ramp
  ([Counting balls coming in](#counting-balls-coming-in)).
- **Take the one measurement below before trusting any speed here.** It tells which part of the transfer sets the
  ball rate, and it needs only one rangefinder and Transfer Thread Count Test.
- **The transfer's continuous servo `melonbotics` is a Melonbotics Super Servo Plus** (CAD). It sits on the lower
  rear Pollen roller shaft. The only Axon MAX in the CAD turns the front sweeper bar's hinge
  ([Parts](parts-findings.md#servos)).
- **Bench-test a Laser Rangefinder at 15 to 40 mm, in SHORT mode at a 10 ms timing budget, before anyone drills a
  hole for one.** Its chip is an ST VL53L1X, and ST's datasheet promises accurate distances only from 40 mm and a
  timing budget no shorter than 20 ms; Brushland lists 10 ms ([The sensor](#the-sensor)). Every spot below has the
  ball 15 to 41 mm from the sensor face.
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
  - **Nectar paddle**, at z 241 to 244. In the assembly the team added on 2026-11-15 that places both turrets on
    the transfer, its post sits just outside the right plate at the Nectar column (x 39.6 to 47.6, y 94.9 to 101.9),
    so it is as plausible as the Pollen one (CAD, 2026-11-15). On the 2026-11-14 CAD, where the turret was drawn at
    another angle, it held a ball at s 143 with one more at s 50; those two positions have not been worked out again
    for the new placement, so don't rely on them.
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
  a 285 mm belt is drawn from that shaft to a motor-driven one. The pulley it cut through on 2026-11-14 was deleted
  the next morning, so the belt looks intended, but its other end still has no pulley drawn (CAD, 2026-11-15): ask
  hardware which of the two drives the roller. If the motor drives that roller, the ball there moves at
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

- **Spots A to E count only balls leaving toward the turrets.** For the possession count the sensors go at the
  intake end of each track, where no spot has been checked in CAD yet: don't drill A to E for that count
  ([Counting balls coming in](#counting-balls-coming-in)).
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
- **Bench-test one rangefinder at 15 to 40 mm, in SHORT at a 10 ms budget, first.** If it can't read that close, the
  sensor has to stand back from the plate on spacers. For ST's 40 mm minimum ([below](#the-sensor)) that
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
  own, and neither Brushland's pages nor ST's datasheet give a way to make them take turns, so the bench test should
  also run two side by side.

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
- **A to E cannot count balls coming in while a paddle holds them** (CAD queue positions; distances from the CAD
  path). With the Pollen paddle closed the queue's centres are s 242, 170, 98 and 27, and a fifth ball would stop at
  about s −46, under the intake. The first ball stops on C. The second stops on both A and B: its centre is 30 mm from
  A's beam and 33.5 mm from B's, both inside its 35.55 mm radius. The third (69 mm from A) and fourth never reach A.
  `TransferCounter` counts only when A clears while B is still covered, so the second ball is never counted. For
  Nectar, a ball held by the Nectar paddle (s 143 on the 2026-11-14 CAD) covers both D and E, and a second (s 50)
  never reaches D; the hold position is not yet worked out for the paddle's 2026-11-15 placement
  ([above](#path-capacity-and-the-blocker)). With the paddle open a full column parks at s 202, 110 and 18, and the second ball again covers D and E while the third never reaches D. So
  spots on the tracks suit counting balls as they leave toward the turrets, not as they come in.
- A and B are on the undriven ramp: a lone ball at loaded speed may not climb it and can roll back, so use the pair
  to tell direction (inferred).
- D and E: the spokes are 40 mm clear. D is 25 mm before the Nectar paddle's hold position on the 2026-11-14 CAD (s 143); the
  turret is at s 202.
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
- Every spot above is 15 to 41 mm from the ball, so bench-test at 15 to 40 mm in SHORT at 10 ms before drilling, and
  leave room for spacers. To try 10 ms: Configure Laser Rangefinder with `timingBudgetMs` 10 (its default is 20).
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
should err toward counting too many. Stop intaking at a count of 4 and reverse within well under 3 s, since with
balls arriving touching a fifth can be under the intake before any sensor sees it: with the Pollen paddle closed it
stops at about s −46, inside the intake. Keep opponent Nectar out with the camera and the driver; the colour sensor
is the backstop. Balls that are shot pass no intake sensor, so the code must subtract shots.

**Why not a sensor pair at the hand-off** (CAD):

- Both types roll on one surface only on the field tile under the intake, until about 28 mm in front of the floor
  plate's front edge (Pollen s −29, Nectar s −37, CAD path). By s 0 the Pollen ball sits on the 10 mm floor plate
  (centre 45 mm above the floor) and the Nectar ball rides the cheek edges (centre 68 mm, bottom 22 mm up).
- Nothing fixed can carry a low beam there. The side plates have no material in front of y 11.5 (35 mm before the
  floor plate's edge), only above z 69.5 from there to y 41, and only above z 49.5 from y 42 to the edge at y 46.9.
  From s 0 the floor plate's 18.5 mm solid cheeks stand between a low side beam and the Pollen ball.
- Above the cheeks, between s 0 and about s 11, a beam sees at most 6 to 7 mm between touching balls of both types.
- At the 66 mm height of a beam 30 mm above the Pollen centre line, two touching Nectar balls leave a 9.1 mm gap
  (chord 82.3 mm). Read as covered or clear by two beams 17 mm apart, a touching Nectar train reads as forward, back,
  forward and counts as one ball (inferred, and simulated below).
- On the tile in front of the plate, side play is 29 to 85 mm for a Pollen ball and 27 to 45 mm for a Nectar ball,
  set by the funnels and the intake, not by the 114.3 mm channel. From about s −20 a Nectar ball rides the cheek
  edges within about ±0.75 mm; a Pollen ball is held within about ±9 mm at s −15, narrowing to about ±3 mm from s 5
  (CAD path table).

**The best-supported layout** (inferred; mounting room not checked in CAD):

- **One Laser Rangefinder per track**, where only that type passes, and early enough that every ball of a full load
  crosses its beam. Count +1 each time the beam goes from clear to covered, or −1 instead while that track's motor is
  commanded in reverse. Wrong counts then fall on the high side. Each track carries one type, so the beam height can
  suit it: about 30 mm off the Pollen centre line (gap 33.0 mm on a straight stretch, 35.6 mm at the best offset of
  30.8 mm), 35 to 40 mm off the Nectar centre line (gap 32.6 to 47.2 mm).
- **Nectar:** on the ramp near s 20 to 30, the beam 35 to 40 mm off the centre line toward the front and up, away
  from the Pollen path (62 mm or more from it). The side plate is solid there (CAD). A full column's third ball parks
  at s 18 and its edge still reaches such a beam. The intake arm swings close by: check in CAD.
- **Pollen: no spot found yet.** A beam 30 mm above the Pollen centre line meets the Nectar path until about s 70,
  but with the paddle closed the fourth ball parks at s 27 and its edge reaches no further than about s 46 at that
  height. So the Pollen beam has to run below the centre line, 5 to 10 mm above the floor plate's top (gap 32 to
  22 mm), between s 0 and about s 46: that means a pocket through the floor plate's cheek, and the plate's surface
  would sit inside the sensor's field of view (at 5 mm up it enters about 40 mm out, about where the ball is), which
  may read as a target. Both need a CAD change and a bench check.
- **Color Rangefinder:** through the Nectar's centre on the ramp at s 20 to 28 (transfer y 61 to 66, z 82 to 88).
  It is the only stretch where a centre beam crosses only Nectar: earlier, every Pollen ball crosses it too (hue still
  tells yellow from red and blue). The hole goes through the plate and the outer part fixed to it, 8.2 mm in all; the
  ball is 19.6 mm from the face, and an empty beam ends on the far plate 117.5 to 122.5 mm away, beyond the sensor's
  100 mm range. Pollen balls pass 0.9 to 7.2 mm below the beam's axis, so they may graze its field of view, which
  Brushland doesn't publish.
- Shooting is first in, first out, so reading colours anywhere every Nectar passes gives their order. Reading early
  only means fewer balls move when one is put back out. Reversing the transfer motor to put a Nectar back out also
  moves the Pollen (every transfer shaft turns 1:1 off that motor); reversing the intake alone reaches only a ball
  the intake wheels still touch.
- **Colour timing.** The ball passes the colour beam in about 57 ms if the intake still drives it there (1610 mm/s
  free). The intake arm probably can't drop fast enough to follow it, so it likely coasts at about 1.2 to 1.5 m/s
  (62 to 77 ms; inferred from the arm's length and gravity alone, its mass and any spring unknown). At the chip's
  fastest colour rate, 25 ms, one or two readings fall wholly on the ball (two or three at the slow end), assuming
  each reading averages the whole 25 ms, as the 16-bit colour Brushland lists suggests; at the chip's 100 ms default,
  possibly none. The next touching Nectar arrives 57 ms or more later, so the colour can
  arrive after it.
- **Speeds at the intake** (inferred, no slip assumed): on the tile a pinched ball moves at 2.2 to 2.8 m/s with the
  robot standing still. Driving adds about 0.47 × the robot's speed (0.465 Pollen, 0.481 Nectar, with the wheel above
  the ball): at 1.85 m/s, the only drive speed in the repo (`BioBuzzConstants`, still the Betta bot's value), that is
  about 3.1 to 3.5 m/s. On the plate and the ramp most of that gain is lost. All intake-derived speeds scale with the
  intake's real loaded speed, which is unknown, and with battery voltage, since the intake runs at power 1.0 with no
  voltage compensation. The side rollers' 1529 / 803 mm/s come from the transfer motor and don't change with driving.

**Alternatives worth knowing:**

- **A two-laser direction pair, if used anyway:** read as covered or clear, the spacing must be below both the gap
  and the chord the beams see (the next ball reaching A before the last one leaves B reads as backward), and half the
  smaller of the two makes the shortest step longest. `TransferCounter` is not such a reader: it counts when A clears
  while B is still covered, can't tell direction, and counts a ball going backward as another ball. It does best with
  the beams about as far apart as the gap (34 to 38 mm for a beam 5 to 6 mm up; simulated).
- **Digital outputs.** The Laser Rangefinder can output a distance threshold as a digital pin (vendor). The Control
  Hub's 8 digital ports are all unused in `biobuzz.xml`, and a digital pin skips the hub's 10 ms ADC refresh
  (inferred). How often the hub samples digital inputs is unmeasured.
- **Break-beam sensors** on Control Hub digital ports: Adafruit's 2167 lists "Response Time: <2 ms" (vendor). Not
  checked against the robot rules.
- **Occupancy where balls rest** (a beam at a queue position) gives a cross-check that brings the count back in line
  after a miss. Motor current can't count a touching train: it is one continuous load (inferred).

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
- One beam per track, counting arrivals: 4 touching Pollen at 1529 mm/s were counted right in 75 to 100% of runs,
  4 touching Nectar at 1610 mm/s (beam 40 mm off centre) in 98 to 100%. The low ends are the pessimistic cases for
  the field of view and for how a part-covered reading reads.
- Assumptions: balls in line, at constant speed, touching or as stated, no slip or bounce, on a flat surface; a
  thin beam unless a cone is named; each sensor updating every 10 or 20 ms at a random phase; the hub's ADC refreshing
  every 9.7 to 11.1 ms (measured, [`hub-bench-findings.md`](hub-bench-findings.md)); a 10 ms poll with 0.3 ms
  jitter; at least 2000 runs per case. The order reader takes ±1 per covered/clear step; `TransferCounter` uses its
  defaults (threshold 25 mm, hysteresis 10 mm). The 6 mm pair can't be built at the hand-off (above).
- How the sensor reads a ball that half fills its field during one ranging window decides most of this and is
  unknown.

**What would settle it:**

1. Bench: one Laser Rangefinder at a 10 ms budget: does it range at all, and does it read a Pollen ball correctly at
   15 to 40 mm ([Parts](parts-findings.md#still-unknown)). Run two side by side for crosstalk.
2. Hardware: which paddles exist, and where a per-track sensor for each type can mount.
3. With a temporary bracket at the chosen Pollen spot, the sensor in SHORT at 10 ms and Transfer Thread Count Test
   with `oneSensor` on, run 5 touching Pollen balls 20 times and 3 touching Nectar balls 20 times through the real
   intake at full power, and record the count each run. If fewer than 99% of runs are right, try break-beams.
4. A slow-motion video of the intake arm as balls pass: whether the wheels follow a ball or let it coast.
5. Colour: hue for several balls of each colour at the mount's distance, and with nothing there
   ([Parts](parts-findings.md#still-unknown)).

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
- The CAD numbers: the team's CAD as of about 23:00 PST on 2026-11-14.
