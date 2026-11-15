# Parts findings

Specs for the parts the shooter depends on: read before buying, swapping or wiring one, or trusting a simulation of
it. Where each part sits is in [`robots-findings.md`](robots-findings.md). Java paths are under
`TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`, as of 2026-11-10 (commit 366596c). Simulated numbers are
from the CAD rerun in [`flywheel-control-findings.md`](flywheel-control-findings.md); commands to rerun it are in
[`scripts/sim/README.md`](../scripts/sim/README.md).

## What to do

- **The flywheel motor in the CAD model as of 2026-11-10 is the goBILDA 5000-0002-4008**, a bare 6000 RPM motor
  belted 1:1 to each flywheel (Nectar's ratio assumed). Read the part number on the built robot before trusting the
  numbers below.
- **Plan on about 5800 RPM free speed at 12 V, not 6000, and on 1.47 kg·cm stall torque** until the bench test under
  "Still unknown" settles it: goBILDA's page and PDF disagree.
- **Keep the flywheels in `RUN_WITHOUT_ENCODER` reading raw counts.** No SDK motor type fits this motor, so
  `getVelocity(AngleUnit)` and `RUN_USING_ENCODER` are wrong on it. `ticksPerRev` (`biobuzz/modules/Shooter.java:24`)
  is 8192 for a Through Bore (either version, mode switch on A, hub port 0 or 3); 28 for the motor's own encoder.
- **If the flywheel encoders go on the OctoQuad, set their channels' velocity interval to 20 ms at every INIT.** The
  50 ms default overflows above 4800 RPM. Reading encoders with odometry costs about 3.4 ms more per loop than
  odometry alone; Control Hub ports 0 and 3 are the default
  ([`flywheel-control-findings.md`](flywheel-control-findings.md#where-to-plug-the-flywheel-encoders)).
- **Give the hoods the full 500-2500 µs pulse range.** The SDK's default gives 31.5° of hood: 1.5° more than Pollen's
  30° of travel, and less than the about 32° a lowered stop would need
  ([`shot-calculation-findings.md`](shot-calculation-findings.md)). The full range gives 35°.
- **Weigh 20 balls of each type** before trusting any simulated ball-to-ball spread.
- **Don't buy the 82 mm steel flywheel until it fits in CAD.** Its fit on the shaft and next to the hood is unchecked.

## Flywheel motor: goBILDA 5000-0002-4008

goBILDA calls it "the motor that drives the popular Yellow Jacket Planetary Gear Motors", here with an 8 mm REX
shaft. The generic simulation in [`flywheel-control-findings.md`](flywheel-control-findings.md) assumed the 5203 1:1
Yellow Jacket, so both are listed.

| at 12 V | 5000-0002-4008 (in the CAD) | 5203-2402-0001, 1:1 gearmotor |
|---|---|---|
| No-load speed | 6000 RPM theoretical, 5800 tested | 6000 RPM (the 5202 1:1: 5400 tested) |
| No-load / stall current | 0.25 A / 9.2 A | 0.25 A / 9.2 A |
| Stall torque | page: 1.47 kg·cm (20.45 oz-in); PDF: 1.74 kg·cm (20.45 oz-in) | 1.47 kg·cm (20.45 oz-in) |
| Encoder | 28 counts per output revolution, 3.3-5 V | 28 PPR at the output shaft, 3.3-5 V |
| Weight; store on 2026-11-10 | 254 g; $29.99, in stock | 391 g; $54.99, in stock |

From goBILDA's [5000-0002-4008 page][gb5000] and [spec-sheet PDF][gb5000pdf], [5203 page][gb5203] and [5202
page][gb5202] (discontinued, replaced by the 5203), read 2026-11-10. Tested speeds are goBILDA's, not ours.

**The stall torque is uncertain by 18%**, and the documents can't settle which figure is the typo:

- 20.45 oz-in is 1.47 kg·cm (computed), so the page agrees with itself and the PDF doesn't.
- But 1.74 is what the motor's free-speed point predicts. A brushed motor's torque constant equals its back-EMF constant
  in SI units; from 5800 RPM at 0.25 A and the 9.2 A stall that gives 1.75 kg·cm (computed, within 0.8% of 1.74), about
  23% more torque per amp than 1.47.
- The simulations used 1.47, the conservative choice: peak power about 22.5 W at 12 V, against 26.6 W with 1.74
  (computed). With 1.74, simulated rapid fire is one or two feed-interval steps faster at long range.

**No SDK motor type fits.** SDK 12.0.0 (decompiled) has none with 28 counts per revolution or near 6000 RPM. Intake
Transfer Shooter Test's config, `TeamCode/src/main/res/xml/intake_transfer_shooter.xml`, makes its flywheels
(`pollenFlywheel`, `nectarFlywheel`) `goBILDA5202SeriesMotor` (lines 5 and 8): the 99.5:1 gearmotor, 2786 counts per
revolution, 60 RPM, which `getVelocity(AngleUnit)` and `RUN_USING_ENCODER` scale by. `biobuzz/modules/Shooter.java`
avoids both: it sets `RUN_WITHOUT_ENCODER` (line 301) and reads raw counts (line 335). That test reads raw counts too
(`rpm()`, `opmodes/test/IntakeTransferShooterTest.java:199-209`). `biobuzz.xml` declares the same two names
the same way, and `Shooter` asks for them.

## 82 mm steel flywheel (goBILDA 3628-0032-0082)

Considered as added inertia. goBILDA's [page][gbsteel], read 2026-11-10: steel, 82 mm, 152 g, MOI 1651 g·cm²
(1.65 × 10⁻⁴ kg·m²), $15.99, in stock. Four 4 mm holes on a 32 mm square, or eight on a 48 mm square to match the
96 mm Hogback Traction Wheel it was built for. No bore or thickness given.

- 1651 is plausible: a uniform 152 g disc of 41 mm radius is 1278 g·cm², a thin ring 2555 (computed). An older
  listing's address says 840 (from the simulation's notes, not rechecked), below the disc, so unlikely.
- What it adds (simulated, with estimated inertias): Pollen from about 1.03 to 2.68 × 10⁻⁴ kg·m², Nectar from 0.83 to
  2.48. The ball-mass error drops to about 40% of today's, and fire is about one feed-interval step faster at
  108-132 in. It adds no power, so long range still needs a second motor.
- **Fit is unverified.** Wider than the 72 mm flywheel, it must sit outside the ball path; its mounting to the shaft
  or hub and its clearance to the hood are unchecked.

## REV Through Bore Encoder: V1 and V2

The nectar turret has a V1 fitted today; a V2 is on order to replace it (the owner, 2026-11-13).

For the hubs the two are the same: 8192 counts per revolution, up to 10,000 RPM, 3.3 to 5 V, JST-PH 6-pin. The hubs
read only those counts. The absolute output below differs, and the nectar turret's INIT zero reads it on the
OctoQuad, so its pulse limits must match the version fitted
([`wiring-findings.md`](wiring-findings.md#the-nectar-turret-encoder)).

| | V1, REV-11-1271 | V2, REV-11-3174 |
|---|---|---|
| Sensor chip; accuracy | Broadcom AEAT-8800; not stated | MPS MA600; factory calibrated to ±0.5° |
| Absolute output (pulse width gives the angle) | 10-bit, 1025 µs period, 1 to 1024 µs | 12-bit, 1000 µs period, 3.884 to 998.06 µs |
| Case colour, from REV's photos | black | gray |
| On REV's store, 2026-11-10 | discontinued; in stock at $40.80 (was $48) | current; $48; out of stock |

- Tell them apart by the part number moulded under the REV logo. Both kits have the same bore inserts (REV-25-1870)
  and cables (REV-11-1275, REV-11-1817, REV-31-1815), and mounting holes 50.8 mm (2.0 in) apart.
- The case switch marked A and S picks the counting output (A, B, index) or a serial one; on S an encoder port gets
  no counts. REV's V1 page says S is serial; that A counts on V2 too is read from the case marking.
- REV says the FTC hubs take only the counting output, through the motor encoder ports. FTC Docs' [hub tech
  tip](https://ftc-docs.firstinspires.org/tech_tips/tech-tips/tech-tip-hub-tips/tech-tip-hub-tips.html) (read
  2026-11-10) says ports 0 and 3 count in hardware, 1 and 2 in software. `CLAUDE.md`, "Flywheel RPM", has the rule.

From REV's product pages for [V1](https://www.revrobotics.com/rev-11-1271/) and
[V2](https://www.revrobotics.com/rev-11-3174/) and specifications pages for
[V1](https://docs.revrobotics.com/rev-crossover-products/sensors/tbe/v1/specs) and
[V2](https://docs.revrobotics.com/rev-crossover-products/sensors/tbe/specs), read 2026-11-10.

## OctoQuad as an encoder input

The robot's OctoQuad carries the odometry pods and the absolute output of the Nectar turret's Through Bore, read once
at INIT to zero that encoder's count on a Control Hub encoder port
([`wiring-findings.md`](wiring-findings.md#zeroing-the-nectar-turret)). It could also take
the flywheel encoders, though
[`flywheel-control-findings.md`](flywheel-control-findings.md#where-to-plug-the-flywheel-encoders) recommends Control
Hub ports 0 and 3. From DigitalChickenLabs' datasheet Rev 3.0C for firmware v3 ([repository][oqdocs], read
2026-11-10) and SDK 12.0.0's `OctoQuad` (decompiled):

- **Velocity**: signed 16-bit per channel, the net count change over the channel's sampling interval (datasheet
  6.5.12): 1 to 255 ms, default 50 ms, set by `setSingleVelocitySampleInterval` or `setAllVelocitySampleIntervals`.
  Lost at power-off unless `saveParametersToFlash()` is called (datasheet 7.2.6), so set it at every INIT.
- **Rating**: 1 million counts per second per port, "higher rates may work, but have not been tested" (datasheet
  2.3). 3.3 V only, powered from the hub's I2C port. Position is int32: it wraps after 58 minutes at 4500 RPM.

| at 8192 counts/rev (computed) | counts/s | share of rating | per 50 ms | per 20 ms |
|---|---|---|---|---|
| 4500 RPM | 614,400 | 61% | 30,720 | 12,288 |
| 5800 RPM (free speed) | 791,893 | 79% | 39,595, overflows | 15,838 |

The 16-bit limit is 4800 RPM at 50 ms and 12,000 RPM at 20 ms (0.37 RPM per count). A hub port's velocity, int16
counts per second, overflows above 240 RPM.

| read | bytes (SDK 12.0.0 source) | Control Hub p50 / p95 / max ms |
|---|---|---|
| `readLocalizerData`, called each loop by Pedro revhub 3.0.2's `OctoQuadLocalizer` (source line 51) | 15 | 3.20 / 3.77 / 8.22 |
| `readAllEncoderData` | 50 | not measured |
| `readLocalizerDataAndAllEncoderData` | 65 | 6.58 / 8.51 / 17.67 |

- Measured 2026-10-09 (hub firmware 1.8.2, SDK 12.0.0, OctoQuad MK2 firmware 3.1.0, 0 bad reads in 383 loops;
  [`hub-bench-findings.md`](hub-bench-findings.md)). Keep it on the Control Hub: the Expansion Hub added about 0.9 ms
  to each measured ADC read, and an I2C read takes several hub commands.
- The encoder block always holds all 8 channels and a CRC16, so extra channels cost nothing; one transaction for
  odometry and encoders needs our own localizer. On a CRC failure the SDK still fills the block: check `isDataValid()`.

## Servos

| servo | on the robot (CAD) | 6.0 V | 4.8 V | travel |
|---|---|---|---|---|
| goBILDA 2000-0025-0003 (25-3 Speed) | both hoods, through 8.571:1 | 0.09 s/60° | 0.11 s/60° | 300° over 500-2500 µs |
| Axon MINI MK2 | turrets | 0.090 s/60° | 0.110 s/60° | 360° max, 500-2500 µs |
| Axon MAX MK2 | transfer, continuous | 0.115 s/60° | 0.140 s/60° | 360° max, 500-2500 µs |

From the [goBILDA page][gbservo] and Axon's [MINI MK2][axmini] and [MAX MK2][axmax] pages, read 2026-11-10; no load.
The CAD says only "Axon Mini" and "Axon MAX"; Axon's older MINI+ and MAX+ list the same speeds. Hub servo ports
supply 5 V ([REV][revexh]; Axon labels 4.8 V as hub voltage); the CAD runs the servos at 6 V through power injectors.

- **Hood speed** (computed): 77.8° of hood per second at 6 V, so 2° takes 26 ms and 5° takes 64 ms; 63.6°/s at 4.8 V.
- **The SDK's default pulse range cuts hood travel.** SDK 12.0.0's default `PwmRange` is 600-2400 µs
  (`PwmControl.PwmRange.defaultRange`, RobotCore 12.0.0 decompiled): at the servo's 0.15°/µs that is 270°, 31.5° of
  hood (computed). The CAD's hood travel is 30° on Pollen, 20° on Nectar. 500-2500 µs gives 300°, 35.0° of hood.
- To widen it, call `setPwmRange(new PwmRange(500, 2500))`, or declare the hood as the SDK's built-in `ServoFullRange`
  config type (500-2500 µs; read from source, not tried on a hub). Today Intake Transfer Shooter Test's config
  declares its hoods as plain `Servo` (`intake_transfer_shooter.xml` lines 10-11), `biobuzz.xml` declares `Shooter`'s
  `pollenHood`/`nectarHood` as plain `Servo` too, and nothing under `biobuzz/` calls `setPwmRange`.

## Game balls

| | Pollen | Nectar |
|---|---|---|
| Diameter (game manual) | 2.8 in (71.1 mm) | 3.6 in (91.4 mm) |
| Mass used in simulation: nominal (range) | 24.9 g (23.5 to 27.2) | 41.3 g (39 to 45) |

The masses are unverified vendor figures from the CAD notes; the manual gives no mass
([`robots-findings.md`](robots-findings.md)). Every simulated ball-to-ball spread rests on that range: as built, the
lightest and heaviest ball change exit speed by +0.59 / −0.96% on Pollen and +1.08 / −1.69% on Nectar (simulated).

## Brushland Color Rangefinder

Read on 2026-11-14 from Wayback Machine snapshots, since the live site refused connections: [configuration][crfconf]
(snapshot 2025-06-14), [reading data][crfread] (2025-11-12), [getting started][crfstart] (2025-11-12) and
[specifications][crfspec] (2025-11-13). The helper class is vendored as `vendor/brushland/ColorRangefinder.java`;
Configure Color Rangefinder writes it and Transfer Thread Count Test reads its hue (usage in `CLAUDE.md`).

- **Out of the box it is an I2C sensor that emulates a REV Color Sensor V3**: configure it as one (getting started).
  A standard I2C read on the Control Hub takes upwards of 4 ms (getting started).
- **Configuration is written over I2C and persists**; afterwards the sensor no longer sends data over I2C, but listens
  for I2C traffic in the first second after power-up. Reset: INIT and stop the configuring OpMode with the sensor
  unplugged, then plug it in; the LED blinks (configuration). Sensors shipped before Nov. 2024 reset with a button
  instead. A sensor with a side switch keeps it toward the connector while configuring and flips it away for analog
  (configuration).
- **Both pins must be configured in one go**: one pin non-I2C with the other left on I2C reverts the sensor to I2C
  (configuration). On success the LED blinks twice (configuration).
- **Only pin 0 can output analog, one channel at a time**: red, blue, green, alpha, HSV hue or distance (the helper's
  `AnalogMode`), 0 to 3.3 V (configuration). Hue in degrees = volts / 3.3 × 360; distance in mm = volts / 3.3 × 100
  (reading data). Analog is 12-bit (specifications).
- **A digital pin is true when its thresholds are met**: an HSV hue range on a 0 to 255 scale (degrees / 360 × 255),
  several ranges per pin by calling the setter again, optionally with a maximum distance the object must be within; or
  a distance range in mm (configuration). Hue wraps at 360, inside red; the page's fix is to invert the pin's whole hue
  wheel by 180°, and it says only very specific shades of red need it, and its example red samples do not
  (configuration). Configure Color Rangefinder writes a wrapping range as two ranges instead; whether our nectar red
  needs inversion is unmeasured.
- **Pin 0 analog with pin 1 digital works**, but needs custom wiring or a splitter cable to reach both port types
  (configuration).
- **To find a colour's hue**, put pin 0 on analog HSV and read it; the page's example turns a reading of 70° into
  bounds of 50-90° (configuration). Its example ranges are for other game pieces: red 0-50°, yellow 55-90° (55-75° in
  a second snippet), blue 180-250° (configuration).
- **Our balls' hue, one reading each** (the owner, 2026-11-14, Transfer Thread Count Test's `Hue (deg)`; the distance
  to the ball was not recorded): nectar red 30°, nectar blue 190°, pollen 86°, and 93° with nothing in front of the
  sensor. Red and blue are far from everything else. Pollen and the empty reading are 7° apart, so hue alone cannot
  tell a pollen ball from no ball: use presence as well (the rangefinder count, or the sensor's own "within N mm"
  pin), or put a background of some other colour opposite the sensor. Transfer Thread Count Test's defaults are set
  from these: red 10-50°, blue 170-210°, pollen 70-89°; Configure Color Rangefinder's red and blue match.
- **Distance**: 940 nm class 1 VCSEL laser, maximum range 100 mm (specifications). The laser rangefinders' SHORT mode
  reaches 1.3 m (`vendor/brushland/LaserRangefinder.java`'s `DistanceMode`).
- **Power and logic**: 3.3 V in, 3.3 V digital logic (specifications). Bulk reads take every non-I2C port of a hub in
  3-4 ms (reading data). Dust on the window spoils colours: blow it off or wipe it with a moist swab (reading data).
- **The configuration page contradicts itself twice.** Its example's prose says pin 0 is true for red or yellow and the
  colour must be within 30 mm; its code makes pin 0 blue or yellow and requires 20 mm. LED brightness is "0-100"
  in the prose (nonlinear: 50 looks like 100, visibly dimmer below 30) but "between 0-255" in the helper's comment.
  Configure Color Rangefinder writes the LED only when `ledBrightness` is 0 or more.
- **Configurations take effect the next time the sensor powers on** (configuration), so it keeps answering I2C
  between pin writes in one run. That is why Configure Color Rangefinder can read calibration for pin 1's distance
  threshold after writing pin 0; the page's own examples only show that read after a digital pin 0, never after an
  analog one, so the first hub run should confirm the LED blinks twice and pin 1 switches at `presenceMm`.
- **One sensor for colour and counting**, from the facts above: pin 0 can carry hue or distance on analog, not both,
  so with hue on pin 0 the only distance left is pin 1's digital "within N mm", and only within 100 mm. Or leave both
  pins on I2C and read colour and distance from one REV Color Sensor V3 device, at upwards of 4 ms per I2C read
  (getting started).

**Not documented on those pages:** the update rate or response time of the analog, digital and I2C outputs, distance
accuracy or noise, which hub channel of a two-channel analog or digital connector each pin reaches (a pin 1 digital
output on the analog channel would read only about 0 or 3.3 V: an inference), what the analog hue reads with nothing
in front, and how many thresholds a pin holds.

## Still unknown

| question | test |
|---|---|
| Which motor is on each flywheel of the built robot | Read the part number on each motor. |
| Stall torque: 1.47 or 1.74 kg·cm | Run the motor free at 12 V, log speed and current, and meter the winding resistance. Back-EMF constant = (V − I·R) ÷ speed in rad/s; stall torque ≈ that × 8.95 A. Near 1.75 kg·cm means the PDF is right. |
| Ball masses | Weigh 20 of each type to 0.1 g; record the mean and range here. |
| Steel flywheel fit | Place it on the flywheel shaft in CAD: bore and hub, clearance to the ball path and the hood. |
| OctoQuad velocity: refresh timing, jitter, and whether an overflow wraps or saturates (no source says) | Spin a Through Bore steadily on a 20 ms channel and log each value with a timestamp; then pass 4800 RPM on a 50 ms channel. |
| Whether hub ports 0 and 3 count 614,400 counts/s without loss (no rating found) | Spin one Through Bore into a hub port and the OctoQuad at once; compare positions after a long run. |
| How much each ball's hue varies on the Color Rangefinder (one reading each so far), and what the empty chute reads at the real mount | In HUE mode, graph Transfer Thread Count Test's `Hue (deg)` for several balls of each colour at the mount's distance and under event lighting, then with nothing there. |
| Which hub analog channel the Color Rangefinder's pin 0 reaches | In HUE mode on Control Hub analog connector 2-3 with nothing else on it, read `colorRangefinder` (analog 2) and `backRangefinder` (analog 3). |
| The Color Rangefinder's output update rate and distance noise | Log its analog distance every 10 ms against a ball moved at a known speed, and its spread with the ball held still. |

[gb5000]: https://www.gobilda.com/5000-series-12vdc-motor-with-8mm-rex-pinion-shaft/
[gb5000pdf]: https://www.gobilda.com/content/spec_sheets/5000-0002-4008_spec-sheet.pdf
[gb5203]: https://www.gobilda.com/5203-series-yellow-jacket-motor-1-1-ratio-24mm-length-8mm-rex-shaft-6000-rpm-3-3-5v-encoder/
[gb5202]: https://www.gobilda.com/5202-series-yellow-jacket-motor-1-1-ratio-24mm-length-6mm-d-shaft-6000-rpm-3-3-5v-encoder/
[gbsteel]: https://www.gobilda.com/steel-flywheel-82mm-diameter-152g-1651-g-cm/
[gbservo]: https://www.gobilda.com/2000-series-dual-mode-servo-25-3-speed/
[axmini]: https://docs.axon-robotics.com/servos/mini
[axmax]: https://docs.axon-robotics.com/servos/max
[revexh]: https://docs.revrobotics.com/duo-control/control-system-overview/expansion-hub-basics
[oqdocs]: https://github.com/DigitalChickenLabs/OctoQuad/tree/master/documentation
[crfconf]: https://web.archive.org/web/20250614233146/https://docs.brushlandlabs.com/sensors/color-rangefinder/configuration
[crfread]: https://web.archive.org/web/20251112032637/https://docs.brushlandlabs.com/sensors/color-rangefinder/reading-data
[crfstart]: https://web.archive.org/web/20251112041041/https://docs.brushlandlabs.com/sensors/color-rangefinder/getting-started
[crfspec]: https://web.archive.org/web/20251113084727/https://docs.brushlandlabs.com/sensors/color-rangefinder/specifications
