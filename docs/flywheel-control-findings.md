# Flywheel RPM control findings

How fast each shooter can fire as designed, how to tune and control flywheel speed, where to plug the flywheel
encoders, and where our loop (`biobuzz/modules/Shooter.java`) loses accuracy. Read it before changing the flywheel
controller, its RPM measurement, the RPMs in the shot table, or the shooter hardware.

Researched 2026-11-10 from the code of 23 DECODE-season (2025-26) FTC teams
([`decode-shooter-survey.md`](decode-shooter-survey.md)), FRC code and forum posts, the FTC SDK 12.0.0 source, vendor
documents and two simulations, both checked independently: a generic study, and a rerun with the robot's numbers
from the CAD model as of 2026-11-10 ([`robots-findings.md`](robots-findings.md)). **Nothing here is measured on our
shooters, and the robot isn't built.** Code paths are under `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`,
line numbers as of 2026-11-10 (commit 366596c).

## What to do

1. **Tune the RPM table while firing the burst you will use**, not with single shots. Tuned on single shots, ball 1
   sits on the nominal shot and every slower ball after it has only the low side of the scoring window. Tuned on the
   burst, the RPMs come out a constant 2.6 to 3.4% higher (Pollen, 36 to 84 in) and the burst uses the whole window:
   today's loop then feeds Pollen with no waiting (92 ms between balls) out to 84 in, against 120 ms tuned on single
   shots (simulated). **Loop changes are optional**: their simulated gain existed only under single-shot tuning, and
   didn't survive occasional long loops like those measured in [`loop-time-findings.md`](loop-time-findings.md).
2. **Design: one motor runs out of power at long range, and a second motor is the lever.** As designed (one goBILDA
   5000-0002-4008 at 1:1, 72 mm wheel), Pollen can't feed without waiting at 132 in, nor Nectar beyond 36 in, in any
   variant simulated. A second motor plus an 82 mm steel flywheel gives Pollen 92 ms everywhere and Nectar 92 ms out
   to 84 to 108 in (drag-free, simulated). Don't gear down ([below](#the-robot-as-designed)).
3. **Plug the flywheel encoders into Control Hub encoder ports 0 and 3** unless a count-loss test at full speed fails;
   the OctoQuad is quieter, no faster, and costs 3.4 ms a loop ([below](#where-to-plug-the-flywheel-encoders)).
4. **Before adding droop points, make droop compensation predict the next ball's wheel speed**: SOTM Test feeds the
   solver readings that are 7 to 14% low just after a ball (simulated, [below](#where-to-plug-the-flywheel-encoders)).
5. **Measure the exit-speed fraction, the motor, friction, inertia and ball masses before trusting any interval
   here.** The exit-speed fraction alone moves Pollen's no-wait range between 36 and 84 in ([tests](#still-unknown)).
6. **Fix where today's loop loses accuracy** (generic study; these barely move the burst interval): stamp each encoder
   sample at the read that fetched it (keep `rpmWindowMs` at 50 until then), read battery voltage every loop or two
   while a flywheel runs, allow full duty above 13 V, integrate only within 50 to 100 RPM of target, and send the
   power as soon as it is computed ([below](#where-our-loop-loses-accuracy)).
7. **Keep a feedforward table plus P and a zoned integral**, in our own loop, voltage compensated, `kD` at 0.
   **Don't** use pure bang-bang, the hub's velocity mode, a boost pulse timed to the feeder, or SolversLib's
   `getCorrectedVelocity()` ([below](#the-control-law-to-use)).

## The robot as designed

Simulated on 2026-11-10 with the CAD model as of 2026-11-10 and checked independently; where the two disagreed, the
check's numbers are used. Distance is horizontal, shooter to CELL, for the minimum-speed shot
([`shot-calculation-findings.md`](shot-calculation-findings.md)), with the hood on its lower stop where that shot is
flatter. **f** is the ball's exit speed over the wheel's surface speed as the ball leaves: unmeasured, at most 0.50
for a ball rolling on a fixed hood, 0.45 unless stated. No hood angle makes up a speed deficit on that shot, so no
interval here relies on the hood.

| | Pollen | Nectar | how known |
|---|---|---|---|
| Drive | one goBILDA 5000-0002-4008, 1:1 belt, one 72 mm wheel | same, ratio assumed 1:1 | CAD |
| Inertia at the wheel | 1.03 × 10⁻⁴ kg·m²: wheel, hub, shaft and pulleys 0.78, motor rotor 0.05, belted 2 in feed roller 0.20 | 0.83 × 10⁻⁴: no roller modelled | 0.78 inferred from CAD; rotor and roller estimated |
| Ball mass | 24.9 g (23.5 to 27.2) | 41.3 g (39 to 45) | vendor figures in the CAD notes, not re-checked: unverified |
| Loaded free speed | 5533 RPM at 12.5 V (5316 at 12.0 V, 5965 at 13.5 V) | same | simulated |
| Ball contact time | 11 to 15.5 ms | 9.5 to 13 ms | simulated |

**The motor**, from goBILDA's specification sheet for 5000-0002-4008 (PDF, saved 2026-11-10): brushed, 12 V, 8 mm REX
shaft, 254 g, 28-pulse quadrature encoder; no load 6000 RPM theoretical, 5800 RPM tested, 0.25 A; stall 9.2 A. The
sheet prints the stall torque as "1.74kg.cm (20.45 oz-in)", but 20.45 oz-in is 1.47 kg·cm, and goBILDA's product page
gives 1.47 ([`parts-findings.md`](parts-findings.md)). The simulation uses 1.47, the conservative value; 1.74 is within
0.8% of what the other figures imply for a brushed motor, a 23% higher torque constant. Peak output is about 22.5 W
at 12 V (both computed).

### Target RPM

Wheel RPM needed before the ball, including the dip during contact, and its share of the 5533 RPM loaded free speed
(simulated). Over 100% can't be reached. Drag-free unless stated; +14% is the upper drag case.

| shooter, f | 36 in | 60 in | 84 in | 108 in | 132 in |
|---|---|---|---|---|---|
| Pollen, 0.50 | 2977 (54%) | 3236 (58%) | 3513 (63%) | 3783 (68%) | 4058 (73%) |
| Pollen, 0.45 | 3270 (59%) | 3554 (64%) | 3858 (70%) | 4155 (75%) | 4456 (81%) |
| Pollen, 0.45, +14% drag | 3732 (67%) | 4051 (73%) | 4400 (80%) | 4732 (86%) | 5081 (92%) |
| Pollen, 0.35 | 4108 (74%) | 4464 (81%) | 4845 (88%) | 5218 (94%) | 5607 (101%) |
| Nectar, 0.50 | 3335 (60%) | 3629 (66%) | 3937 (71%) | 4279 (77%) | 4603 (83%) |
| Nectar, 0.45 | 3628 (66%) | 3948 (71%) | 4283 (77%) | 4654 (84%) | 5007 (91%) |
| Nectar, 0.45, +14% drag | 4138 (75%) | 4509 (82%) | 4901 (89%) | 5315 (96%) | 5771 (104%) |
| Nectar, 0.35 | 4466 (81%) | 4859 (88%) | 5272 (95%) | 5766 (104%) | 6285 (114%) |

- Exit speeds needed (computed): 196 to 267 in/s on Pollen, 196 to 270 on Nectar. The placeholder table (2500 to
  3700 RPM, `biobuzz/ShotTuning.java:121-125`, `:133-137`) is not a prediction.
- Beyond about 95 in (Pollen) and 70 in (Nectar) the drag-free minimum-speed shot is flatter than the hood's lower
  stop (57.7° and 60.8° launch), costing up to 0.5% and 1.9% more exit speed at 132 in, included above. Reaching it
  at 132 in needs 3.3° and 6.5° more travel below the stop; the CAD stop gives about 0.4°. With drag the shot is
  flatter still and leaves the travel sooner (84 and 60 in:
  [`shot-calculation-findings.md`](shot-calculation-findings.md)).

### What one ball takes

- In the model (a thin-shell ball rolling on the hood) a ball takes about 10% of the wheel's speed on Pollen and 19%
  on Nectar at f 0.45 (8 to 11% and 15.5 to 21% over f 0.35 to 0.50), and 2.1 to 3.1 times its own kinetic energy.
  With a loop that sees the true speed the dip is 270 to 440 RPM (Pollen) and 640 to 950 RPM (Nectar) at 36 to
  132 in, and getting back within 1% takes 57 to 139 ms and 105 to 329 ms (simulated).
- The contact model is unmeasured: the shot-calculation study's other models (solid ball, frictionless hood, a hood
  that drags the ball back) take about 0.85 to 1.45 times the rolling shell's loss (simulated there).

### Fastest feed interval

The shortest gap, from 92, 105, 120, 135, 150, 175, 200, 250 and 300 ms, at which all 10 balls of a burst score in
all 8 random runs; 92 ms (10.9 balls a second) is no waiting. Each cell is **burst-tuned to single-shot-tuned**, for
today's loop on a hub encoder port, f 0.45, 12.5 V (simulated):

| shooter, drag | 36 in | 60 in | 84 in | 108 in | 132 in |
|---|---|---|---|---|---|
| Pollen, none | 92–120 | 92–120 | 92–120 | 105–135 | 135–175 |
| Pollen, +14% | 92–120 | 92–120 | 120–150 | 175–200 | 300 |
| Nectar, none | 105–135 | 120–150 | 175 | 250 | over 300 |
| Nectar, +14% | 135–175 | 175–200 | 300 | over 300 | RPM out of reach |

- "Score" means an exit speed inside the scoring window (burst-tuned: the CAD geometry's, narrower for Nectar and on
  the hood's stop). Neither allows for aim, turret, drag-model or battery-sag error, so both are optimistic; the
  ball-mass spread costs one or two more steps from 84 in (Pollen, burst-tuned: 92/92/105/120/175).
- Burst-tuned, all six real loop and encoder variants and a loop that sees the true speed landed within a step of
  these; occasional long loops cost today's loop a step on Pollen at 108 and 132 in. New random seeds moved one cell
  in 11 by a step single-shot-tuned, one in 40 burst-tuned. As designed, 3-ball bursts match 10-ball ones within a
  step (with more inertia or drag, 2 to 4 steps faster). Nectar reaches 92 ms at 36 in with the 1.74 kg·cm torque or
  a 13.5 V battery, and burst-tuned with f 0.50 or 1.6 × 10⁻⁴ kg·m² or more.

### Sustained rate and ball-mass spread

- **Once the wheel's stored energy is spent, the motor sets the rate**: one motor gives 7 to 20 W above friction at
  these speeds (12.5 V). Per ball, at 36/60/84/108/132 in, with a loop that sees the true speed, no margin, f 0.45,
  drag-free: Pollen 60/73/92/116/153 ms (58/70/86/107/137 with the steel flywheel below), Nectar 108/135/175/241/356 ms
  (98/119/148/191/253) (simulated). So one motor can't keep feeding at 92 ms beyond about 84 in (Pollen) or at any
  distance (Nectar), whatever the controller; inertia only buffers the first balls.
- **Battery state matters as much**: at 132 in Pollen sustains 4.6 balls a second at 12.0 V, 8.1 at 13.5 V, 7.7 with
  today's 13 V cap (computed at target speed, which understates all three).
- **Ball-mass spread**: the vendor's lightest and heaviest balls exit +0.59% and −0.96% off nominal on Pollen, +1.08%
  and −1.69% on Nectar (simulated, f 0.45). No loop can correct it, as it happens within the contact. More inertia
  shrinks it (steel flywheel: −0.40/+0.24% and −0.66/+0.41%); gearing and a second motor don't. Burst-tuned, every
  case fits the scoring window, Nectar at 132 in with 0.16 percentage points to spare.

### What each change buys

Fastest feed interval, burst-tuned, ms at 36/60/84/108/132 in, drag-free ; +14% drag, f 0.45, for a loop that sees
the true speed (today's loop is within a step of it on most rows, two or more steps slower on a few); "–" means the
RPM is out of reach (simulated):

| change | Pollen | Nectar |
|---|---|---|
| none (as designed) | 92/92/92/105/150 ; 92/92/120/175/300 | 105/120/175/250/>300 ; 135/175/300/>300/– |
| 82 mm steel flywheel | 92/92/92/92/120 ; 92/92/105/150/250 | 92/105/135/175/250 ; 120/150/200/300/>300 |
| second motor | 92 everywhere ; 92/92/92/92/120 | 92/92/92/105/150 ; 92/92/120/200/>300 |
| second motor and steel flywheel | 92 everywhere ; 92/92/92/92/105 | 92/92/92/92/120 ; 92/92/105/135/200 |
| 1.25:1 overdrive | 92/92/92/105/120 ; 92/92/120/135/200 | 105/120/150/200/250 ; 135/175/250/300/>300 |
| 0.8:1, geared down | 92/92/105/150/250 ; 92/120/200/–/– | 105/150/250/–/– ; 175/300/–/–/– |

- **Steel flywheel**: goBILDA 3628-0032-0082, listed at 1651 g·cm², 1.65 × 10⁻⁴ kg·m² (goBILDA's page, read
  2026-11-10: [`parts-findings.md`](parts-findings.md)). It cuts the mass error 2.4 times. Its fit on the shaft, hub and hood is
  unchecked: wider than the 72 mm wheel, it must sit outside the ball path, and the listing gives a 32 mm and 48 mm
  hole pattern but no bore. Inertia alone, even 4 × 10⁻⁴, leaves Nectar at 120 to 250 ms from 84 in.
- **A second motor** is what changes long range; that rests on f, friction and the torque constant. **Overdrive**
  (on Pollen a GT2 pulley swap) frees speed headroom against a low f and drag but gains two steps at most; 1.5:1
  matches 1.25:1 on Pollen and is a step worse on Nectar at 36 to 84 in. **Don't gear down.** A 96 mm wheel (about
  2.2 × 10⁻⁴, estimated) acts like the steel flywheel but needs a new hood.

### What holds and what depends

**Holds in every variant simulated** (f 0.35 to 0.50, drag 0 to 14%, every loop including one that sees the true
speed, 12.0 to 13.5 V, both stall torques, half to double the friction, both ways of tuning): Pollen can't feed
without waiting at 132 in on one motor at 1:1 (best 120 ms), and Nectar can't at 60 to 132 in. Also robust, in fewer
variants: at long range the motor's power, not the loop, sets the rate and inertia only buffers; gearing down hurts
and overdrive gains two steps at most; the OctoQuad doesn't shorten the interval; no loop corrects ball-mass spread.

**Where the limits fall, and what a second motor buys, depend on values nobody has measured** ([tests](#still-unknown)):

| value | simulated | what it moves |
|---|---|---|
| f | 0.35 to 0.50 | Pollen's no-wait range, today's loop burst-tuned: 36 in at 0.35, 60 in at 0.40, 84 in at 0.45 and 0.50. At 0.35 Pollen can't reach 132 in, nor Nectar 108 in. |
| Drag | 0 to +14% | Pollen at 84 in: 92–120 to 120–150 ms; Nectar's 132 in out of reach |
| Torque constant | stall 1.47 or 1.74 kg·cm | 1.74: Pollen 132 in 150 to 120 ms, Nectar 36 in 120 to 92 ms (true-speed loop, single-shot-tuned) |
| Friction | 0.004 to 0.016 N·m at 3000 RPM; 0.008 baseline, about 4 times the motor's own | 0.016 loses 1 to 4 steps and puts Nectar's 132 in out of reach |
| Battery | 12.0 to 13.5 V, one shooter alone | Pollen 132 in: 200 to 120 ms. Both shooters and the drivetrain on one battery weren't simulated. |
| Inertia | estimated | Nectar with Pollen's roller (1.03 × 10⁻⁴): same to one step better |
| Ball masses | vendor range | the mass error and Nectar's margin at 132 in |
| Contact model | rolling shell only | each ball's loss, 0.85 to 1.45 times |

## Where to plug the flywheel encoders

Undecided: [`wiring-findings.md`](wiring-findings.md) defaults to Control Hub encoder ports 0 and 3 and leaves the
OctoQuad as the open alternative. Both flywheels read a REV Through Bore at 8192 counts/rev
(`biobuzz/modules/Shooter.java:24`). The hub's own velocity is a signed 16-bit count per second, useless at 8192
counts/rev above 240 RPM ([`sdk-hub-findings.md`](sdk-hub-findings.md), Encoder velocity), so the RPM is computed in
software from position and time (`biobuzz/modules/Shooter.java:333-343`).

| | Control Hub encoder ports 0 and 3 | OctoQuad |
|---|---|---|
| Counting | in hardware on 0 and 3; 1 and 2 count in software ([`sdk-hub-findings.md`](sdk-hub-findings.md), Encoder ports) | rated 1,000,000 counts a second per port, "higher rates may work, but have not been tested" (datasheet Rev 3.0C, 2.3) |
| Count rate needed | 614,400 counts/s at 4500 RPM, 819,200 at 6000 (computed); no maximum published for hub ports, so unverified | 61 to 82% of rated |
| Velocity | from positions in the bulk read, which carries no timestamp ([`sdk-hub-findings.md`](sdk-hub-findings.md), Timestamps); our code times them | on the board's own clock: signed 16-bit counts per interval, 1 to 255 ms per channel, default 50 ms (datasheet 6.5.12 and 8.5.4; SDK 12.0.0 `OctoQuad`) |
| Noise at steady speed | 30 to 48 RPM standard deviation, from ±0.5 ms of timestamp error (simulated) | under 1 RPM (simulated) |
| Age of a reading | about 22 ms: the middle of the 45 ms window | about 27 ms on average at 20 ms: the middle of the interval, up to one interval since it ended, and the read (computed) |
| Loop time | none extra: it rides the Control Hub bulk read the loop already pays ([`wiring-findings.md`](wiring-findings.md)) | +3.4 ms a loop: `readLocalizerDataAndAllEncoderData` 6.58 ms p50 against 3.20 for `readLocalizerData` (measured on our Control Hub, 2026-10-09, OctoQuad MK2 firmware 3.1.0, SDK 12.0.0, [`hub-bench-findings.md`](hub-bench-findings.md)) |

- **OctoQuad interval: 10 to 20 ms, set at every INIT** (`setAllVelocitySampleIntervals`; lost at power-off unless
  `saveParametersToFlash()` is called). Never the 50 ms default: 16-bit counts per 50 ms overflow above 4800 RPM, and
  the longer interval lags (bursts 15 to 150 ms slower, simulated). At 20 ms the limit is 12,000 RPM.
- **OctoQuad on the robot**: keep it on the Control Hub (on the Expansion Hub each command of an I2C read is
  expected to pay the RS485 hop, about 0.9 ms measured on ADC reads, not measured on I2C); its inputs are 3.3 V only
  (datasheet 2.1). Pedro 3.0.2's localizer reads only the localizer block
  (`OctoQuadLocalizer.update()`, revhub 3.0.2 sources), so the encoder block needs our own localizer calling the
  combined read, guarded as [`hub-bench-findings.md`](hub-bench-findings.md) describes and checking `isDataValid()`.
- **The OctoQuad removes noise, not waiting.** Hub ports and the OctoQuad at 10 or 20 ms gave the same fastest
  interval within one step at every distance, both ways of tuning (simulated): the motor's power limits recovery, and
  a 20 ms OctoQuad reading is about as old as the hub's 50 ms window. Low noise helps steady accuracy.
- **Droop compensation must not take a reading from just after a ball as is**, from either input. Read 10 to 50 ms
  after a ball it is 7 to 14% low, while the wheel is only about 0.4% slow when the next ball arrives (Pollen at 70% of
  free speed, OctoQuad at 20 ms; simulated in the shot-calculation study). SOTM Test passes `getPollenRPM()` and
  `getNectarRPM()` to the solver every loop (`opmodes/test/SotmTest.java:140-141`), which sets the hood from the droop
  they imply (`biobuzz/ShotSolver.java:147-148`). Use the wheel speed predicted for the next ball's contact instead.
  The placeholder tables have no droop points, so nothing acts on it today.
- **Untested third option**: if the hub's velocity wraps in multiples of 20 counts/s (community sources only),
  Road Runner's `inverseOverflow` unwraps it into a velocity on the hub's clock
  ([`sdk-hub-findings.md`](sdk-hub-findings.md)); in the generic study, removing timestamp error cut the error at ball
  arrival from about 30 RPM to 1 to 5.

**The decision as it stands:** Control Hub encoder ports 0 and 3 are the lower-loop-time choice, the OctoQuad the
lower-noise choice, and neither feeds faster. The bench test that decides is a count-loss test: run a Through Bore at
full flywheel speed on port 0 or 3 for 10 s and compare the hub's count with an OctoQuad's count of the same shaft;
if they disagree, use the OctoQuad. See [`wiring-findings.md`](wiring-findings.md) for where everything else
plugs in.

## Where our loop loses accuracy

Read from the code on 2026-11-10 (commit 366596c). "Simulated" means the generic study; hub costs are from
[`hub-bench-findings.md`](hub-bench-findings.md) and hub behaviour from [`sdk-hub-findings.md`](sdk-hub-findings.md).

| # | what | where | effect |
|---|---|---|---|
| 1 | The sample is stamped at the midpoint of `getCurrentPosition()`. With MANUAL bulk caching that call returns cached data unless it is the first read on that hub this loop. | `biobuzz/modules/Shooter.java:334-337` | In BioBuzz Shooter Test the pollen read does the bulk read itself, so its stamp falls inside the bulk read (2.2 ms p50, measured); the nectar read, if both flywheels share a hub, is cached and stamped about 1 ms or more late. In SOTM Test, on any loop after the drivetrain was commanded, the drivetrain's `floodgate` current-sensor read does the bulk read first (if it shares the hub), so the offset changes between driving and idle loops. Each 1 ms change in offset is a one-window error of about 67 RPM at 3000 RPM (computed for the 45 ms effective window). |
| 2 | Timestamp error sets the RPM noise, the only measurement noise the simulations model. | same | Simulated at a 15 ms loop: 27 RPM of noise with ±0.5 ms of timestamp error and a 50 ms window, 82 RPM at ±1.5 ms. Shorter windows are worse (63 and 188 RPM at 30 ms): keep `rpmWindowMs` at 50 until finding 1 is fixed. |
| 3 | The window is shorter than its name: `measure()` keeps the oldest sample no older than `rpmWindowMs`. | `biobuzz/modules/Shooter.java:339` | At a 15 ± 2 ms loop, 50 ms is really about 45 ms and 30 ms about 22 ms (simulated). Set the window to a whole number of loops plus a margin. |
| 4 | Battery voltage is refreshed every 50 loops. | `architecture/core/EnhancedOpMode.java:38`, `:407-410` | Simulated, with the hold modelled as 750 ms: a 25 A drivetrain draw on a battery with 0.15 Ω internal resistance (a 3.75 V sag), starting 100 ms before a shot, leaves the wheel 145 to 160 RPM low for the first ball at 1 × 10⁻⁴ kg·m² (32 to 37 RPM at 5 × 10⁻⁴). With a fresh reading: 1 to 6 RPM. A read costs 1.30 ms p50, and the hub's reading changes about every 10 ms (measured). |
| 5 | Output can't reach full duty above 13 V: power is clipped to 1 in 13 V units, then scaled by 13/V. | `biobuzz/modules/Shooter.java:277-279`, `architecture/hardware/WriteCache.java:32-37` | At 13.5 V the motor gets at most 0.963 duty. On a 13.8 V battery that is 10 to 17% less spare torque at 3000 to 4500 RPM (computed from the 5000-0002-4008's figures, friction ignored); at 13.5 V it costs Pollen at 132 in 5% of its sustained rate (CAD rerun). |
| 6 | The integral has no zone and is kept across target changes. It accumulates through every dip while the output isn't pinned. | `architecture/control/PidController.java:85-91`, `biobuzz/modules/Shooter.java:304` | Simulated on a 0.5 × 10⁻⁴ kg·m² flywheel with kP = 0.005 V/RPM and kI = 0.03 V/(RPM·s), about 0.0004 and 0.0023 in our 13 V power units: the wheel was 194 and 372 RPM over target when balls 2 and 3 arrived 150 ms apart. With a 100 RPM zone: 12 RPM under and 2 over. A zone also removes the steady error a wrong feedforward leaves: under 20 RPM with the table 10% off, against 30 to 94 RPM for P alone. |
| 7 | Power is computed in `read()` and sent in `write()`, after the follower update and the other modules' writes. | `biobuzz/modules/Shooter.java:205`, `:345-347` | Estimated 6 to 8 ms later in BioBuzz Shooter Test (more for nectar and on some loops); 12 to 19 ms plus the solver's unmeasured time in SOTM Test while driving. Not in the generic study; the CAD rerun modelled 8 to 12 ms. |
| 8 | Solver setpoints arrive one loop late: `setSotmTargets` goes through `bindTunable`, and the power is computed before `gameLoop`. | `biobuzz/modules/Shooter.java:190-193`, `:197-202` | One loop of lag on RPM and hood targets. |
| 9 | The feedforward table is flat outside its range, and a point with an RPM but power 0 is used as is. | `biobuzz/modules/Shooter.java:246-265` | A target above the last point gets too little feedforward. |
| 10 | A failed voltage read returns 0.0 V. | `biobuzz/modules/Shooter.java:319` | `headroom` becomes 0 and the integral freezes for up to 50 loops. |
| 11 | A failed bulk read returns all-zero encoders without an error. | SDK (`LynxModule.getBulkData`) | One enormous RPM spike in each direction. Reject a sample whose speed is impossible. |
| 12 | Power is floored at 0 and the motor floats. | `biobuzz/modules/Shooter.java:278`, `:299` | A falling target can only coast down. Whether a small positive duty brakes is unknown. |
| 13 | No unit test covers the flywheel loop or the RPM estimate. | `TeamCode/src/test` (only `ShotSolverTest`, `TargetSelectorTest`) | |

Already right: voltage compensation is applied exactly once (the table and PID are in 13 V units,
`biobuzz/modules/Shooter.java:296`, and `WriteCache` scales by 13/V); the integral stops accumulating when the output
is pinned (`architecture/control/PidController.java:89-90`); and the RPM comes from position.

## The control law to use

```
ff       = table(target)          power in 13 V units; extend the last two points in a straight line
error    = target - rpm
power    = ff + kP * error + integral
integral grows only while |error| < zone and the output isn't pinned; held outside     zone: 50 to 100 RPM
optional: power = full while error > band
clip power to 0 .. V/13, V being the battery voltage read in the last few loops, so after scaling it can reach full
```

- **The control law is the smallest lever.** In the generic study (1 × 10⁻⁴ kg·m² and up, balls 250 to 400 ms
  apart), feedforward + P, the same with a zoned integral, a full-power band, a model-based predictor and the hub's
  own velocity loop left the wheel within about 5 to 15 RPM of each other at the next ball, in an order that changed
  with the seed, torque constant and scoring weights. In the CAD rerun, burst-tuned, every loop landed within a step
  of the true-speed loop as designed. One FRC team reported recovery of 0.3 to 0.5 s across control settings, about
  0.4 s with bang-bang ([Chief Delphi](https://www.chiefdelphi.com/t/flywheel-rpm-drop-higher-than-expected/514242),
  read 2026-11-10; one team's account, no published data).
- **Keep kP moderate on a light flywheel.** On 0.5 × 10⁻⁴ kg·m², kP = 0.01 V/RPM (about 0.0008 in our units) left
  the wheel 120 to 150 RPM over target at balls 2 and 3, 150 ms apart, even with the integral off: P acting on a
  measurement that lags by half the 45 ms window. Half that kP did not (simulated).
- **A full-power band below target**, as 3 of the 23 teams run, is optional: it landed within the same 5 to 15 RPM
  (simulated). Its switch point must exceed acceleration times measurement lag: in the CAD rerun, switching 2% low
  overshot by up to about 6% in some cases, and both switch points tried were sensitive to tuning. D had no
  measurable effect (simulated).

Why not the alternatives:

- **Pure bang-bang**, and bang-bang over a feedforward floor, came last in every ranking of the generic study: with
  our delayed measurement they cycle ±50 to 100 RPM and never settle within 50 RPM, except on the heaviest flywheel
  with perfect timestamps. The two teams that ship it use the hub's velocity, which our timestamp error doesn't touch.
- **The hub's velocity mode** (`RUN_USING_ENCODER`) works in 16-bit counts per second
  ([`sdk-hub-findings.md`](sdk-hub-findings.md)), so it would need the motor's 28-count encoder in the Through Bore's
  port. Modelled from community descriptions, it left the wheel 121 RPM low for ball 1 under the sag in finding 4
  (simulated).
- **A boost pulse timed to the feeder** made exit speed 0.6 to 1.0 percentage points less consistent (simulated):
  the pulse lands at a random point of the contact. **SolversLib's `Motor.Encoder.getCorrectedVelocity()`** unwraps
  the hub velocity wrongly (read from the 0.3.6 source: a true 40,000 counts/s returns −25,536).

## What other teams run

Of 23 DECODE teams' competition code, read twice and independently
([`decode-shooter-survey.md`](decode-shooter-survey.md)), 14 run feedforward (kV, often kS) plus a large P with no
working integral; 3 full power below a band and feedforward inside it; 2 pure bang-bang; 2 a normal loop with
bang-bang only while feeding; 1 a three-mode hybrid (spin-up, recover, hold); 1 its own loop on top of the hub's
velocity mode.

- The P is large: in most of the 14, an error of about 30 to 100 ticks/s (65 to 215 RPM on the 28-count encoder)
  already commands full power (computed from their gains), so the loop runs at full power through a dip.
- [`FTC11329/11329-2026-repo`](https://github.com/FTC11329/11329-2026-repo) removed its integral: it was kept across
  target changes and caused "severe overshoot during autonomous shooting" (`docs/shooter-interpolation.md` there).
- 12 of 23 compensate for battery voltage; 3 more have the code but it does nothing. None runs the SDK's
  `RUN_USING_ENCODER` loop on its own. All 23 read the motor's 28-count encoder through the hub's velocity, 21
  unfiltered: steps of about 43 RPM if it comes in multiples of 20 counts/s, against 0.15 RPM per Through Bore count
  over 50 ms.
- Published FRC code on GitHub (`Team254/FRC-2020-Public`, `Team254/FRC-2022-Public`, `frc1678/C2022`,
  `Mechanical-Advantage/RobotCode2022`, `FRCTeam2910/2026CompetitionRobot-Public`) mostly runs feedforward + P on the
  motor controller; `Team254/FRC-2022-Public` adds a small integral within 100 RPM of target, and
  `frc971/971-Robot-Code` feeds timestamped positions to a Kalman filter.

## The simulations

Commands to rerun both: [`scripts/sim/README.md`](../scripts/sim/README.md). Both model a brushed motor on a flywheel,
a battery with internal resistance, a ball taking momentum from the wheel during contact, our position-difference RPM
with timestamp error (the only measurement noise modelled), and a 15 ± 2 ms loop.

**Generic study** (`scripts/sim/flywheel/`): 11 controllers per inertia tuned by grid search, plus the true-speed
loop, scored on spin-up, single shots and 3-ball bursts. Its own assumptions: a goBILDA 5203 1:1 motor (6000 RPM,
1.47 kg·cm, 9.2 A stall; [`parts-findings.md`](parts-findings.md)), balls of 10 to 60 g, a 96 mm wheel at f 0.45,
inertia 0.5 to 5 × 10⁻⁴ kg·m², and the hub's velocity loop from community descriptions. Checked independently: results
reproduced exactly, the plant matched closed forms (momentum within 0.4%, spin-up within 0.1%), and new seeds, a 19%
higher torque constant and other scoring weights kept the conclusions but reordered the better controllers. This
page uses it only for the loop's accuracy and the control law; the CAD rerun replaces its motor, wheel, inertia and
ball numbers.

**CAD rerun** (`scripts/sim/cad/flywheel/`, shot speeds from `scripts/sim/ballistics/`): the plant above, the motor
at 1.47 kg·cm and 5800 RPM, friction 0.008 N·m at 3000 RPM rising to 0.016 at 5800 (pessimistic), a 12.5 V battery
with 0.10 Ω, a thin-shell ball rolling on the hood, and a 50 ms window with ±0.5 ms timestamp error. Today's loop is
modelled from `Shooter.java` and `PidController.java` with 8 to 12 ms from compute to write, voltage every 50 loops
and the 13 V cap, beside two faster loops (full power until a set fraction below target, fresh voltage, sent at
once), the true-speed loop, and the OctoQuad at 10, 20 or 50 ms with a 6.6 ms read. Checked independently (closed
forms to 5 decimals, an independent ODE solver, a semi-analytic burst bound). Most of the differences between
controllers come from tuning on single shots; burst-tuned, every loop lands within a step of the others. Not
modelled: both shooters and the drivetrain on one battery. The loop runs every 15 ± 2 ms;
[`wiring-findings.md`](wiring-findings.md) estimates 19 to 37 ms on this robot, so the numbers for today's loop may be
optimistic at that period (not rerun).

## Still unknown

| question | test |
|---|---|
| f for each shooter: sets every target RPM and interval | Fire single balls at three wheel speeds through a two-gate speed trap, or film from the side at 240 fps with a scale in frame, logging RPM every loop. Divide exit speed by the wheel's surface speed after the ball (0.036 m radius); against the speed before it, the ratio reads low by the dip, about 10% on Pollen and 19% on Nectar. |
| Drag | At fixed RPM and hood, measure landing height against distance and fit quadratic drag ([`shot-calculation-findings.md`](shot-calculation-findings.md)). |
| Motor torque constant (stall 1.47 or 1.74 kg·cm) and friction | With wheel and belt fitted: free speed and current at three battery voltages, a coast-down from 5000 RPM at power 0 (friction against speed), and stall current. |
| Each flywheel's inertia, and whether Nectar has Pollen's belted roller | Log RPM and battery voltage from rest at full power with no ball: the time constant gives the inertia, the settled speed the loaded free speed. Check the CAD for Nectar's roller. |
| Ball masses and their spread | Weigh 20 of each type. The competition manual gives diameters only. |
| The real dip and recovery per ball, and so the contact model | Log RPM (OctoQuad at 10 ms, or the hub), power and battery voltage every loop through 10-ball bursts at two distances. Power pinned at full during recovery means the controller isn't the limit. |
| Battery voltage in a match-like burst | Log it every loop through a 10-ball burst with both shooters spinning, while driving. |
| Whether Control Hub encoder ports 0 and 3 lose counts at flywheel speed | The count-loss test [above](#where-to-plug-the-flywheel-encoders). |
| Whether the 82 mm steel flywheel fits | Check its bore, mounting and clearance to the hood and ball path in the CAD. |
| Whether one burst-tuned table also serves single shots | At each tuned distance, fire single balls and full bursts and count scores. |
| Does the hub's velocity wrap, in multiples of 20? | Log raw `getVelocity()` beside the windowed RPM from 300 to 4000 RPM. |
| Whether to clear the integral when the target steps | Not simulated. Step the target from 2500 to 3500 RPM and back with the zone in place, and log the overshoot. |
| Does a small positive duty brake the wheel? | From a steady speed, command 0.05 and compare the slowdown with power 0, under FLOAT and under BRAKE. |
| How much the sample timestamp moves, and whether the drivetrain's `floodgate` sensor shares the flywheels' hub | Log which read triggers the bulk read each loop, and the time from that read to `measure()`. [`wiring-findings.md`](wiring-findings.md) puts both on the Control Hub; no config in the repo has them yet. |
