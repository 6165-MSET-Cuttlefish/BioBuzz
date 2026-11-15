# Robot findings

What the code doesn't show about the robots: which robot the team runs, the BIOBUZZ robot as it stands in CAD, the
sizes and masses of the two retired robots, the Cuttle bot (last season's robot) and the Betta bot (the test robot
BioBuzz code ran on before the BIOBUZZ robot), and known problems in the test OpModes and modules that drive them.
Read it before deciding what may run on the BIOBUZZ robot, before writing code for its shooters, turrets or cameras,
before running a shooter or turret test OpMode, or before putting a robot's size into code or a `.pp` path file.

## One robot (decided 2026-11-13)

The team runs one robot, the BIOBUZZ robot. On 2026-11-13, the day it first ran assembled (on BioBuzz Basic Tele),
the team retired the Betta bot and the Cuttle bot and decided that the BIOBUZZ robot uses no Pinpoint this year, for
now: its odometry is the OctoQuad. The hub bench keeps every Pinpoint bench, in case one is added later.

- Removed with them: `decode/` (last season's code) and its gamepad input classes, the Betta bot's test OpModes
  (Pinpoint Localizer Test, Pinpoint Drive Test, Ball Field Drive, Ivy Test, Ivy Path Test), `BettaShooter` and
  `scripts/looptimes/analyze_ivy.py`. `git log --diff-filter=D -- <path>` names the commit that deleted each; its
  parent still has it.
- Deleted on 2026-11-14 with `butterfly.xml`: `betta_decode.xml` and `cuttle_decode.xml`. That shifts the other
  configs' resource IDs, so after the next full install re-activate each hub's config on the Driver Station
  ([`toolchain-findings.md`](toolchain-findings.md#deploying)).
- Kept: `BettaTurret`, which Bench: Turret PWM drives.
- Still the Betta bot's, not measured on the BIOBUZZ robot: every value in `pedro/BioBuzzConstants` (drive directions,
  Pinpoint settings, Foresight gains), `biobuzz/RobotGeometry`'s size and intake, `BallFieldTransform.Mount` and the
  Limelight's `H_ARRAY` ([`limelight-findings.md`](limelight-findings.md)), and the Close Flower start pose.
- So, until the follower reads the OctoQuad, BioBuzz Tele and everything else that builds `BioBuzzConstants`'
  follower (all of `BioBuzzRobot`) stop at INIT on `biobuzz.xml`, which has no Pinpoint
  ([`wiring-findings.md`](wiring-findings.md)), and until those values are measured nothing that follows a path runs
  on the robot.
- Measurements made on the two retired robots stay in these pages, marked "retired 2026-11-13".

## The BIOBUZZ robot, from CAD

From the team's CAD as analysed on 2026-11-10. **Nothing here is measured on a robot, and the CAD is still changing.**
Nothing is wired: the recommended port for every device is in [`wiring-findings.md`](wiring-findings.md). Servo
directions and ranges, turret zero positions and every control constant are undecided: ask
or measure before putting any of these numbers in code. "Inferred" marks a number estimated from the CAD rather than
read from it.

The robot is a double shooter: two turreted flywheel shooters that both aim at the HIVE all match. The POLLEN shooter
is on the rear turret and the NECTAR shooter on the front one, their axes 8.54 in apart on the robot's centreline.
Each turret must cover at least 90° either side of straight back; facing back, NECTAR shoots over POLLEN. Each
shooter has one flywheel motor and a servo hood that sets the launch angle.

| | Pollen shooter | Nectar shooter |
|---|---|---|
| Ball | 2.8 in (71.1 mm), 24.9 g (23.5 to 27.2) | 3.6 in (91.4 mm), 41.3 g (39 to 45) |
| Turret axis from the robot's centre (inferred, ±1 in) | about 5.3 in behind | about 3.3 in ahead |
| Turret drive | Axon Mini, about 1:1 (48T to 20T, then 59T to 140T) | Axon Mini; the turret turns 126/125 = 1.008 of the servo's angle, the same way (48T to 20T, then 63T to 150T, teeth counted) |
| Turret servos | 2 | 1 mated; a second is drawn on the same pinion but fastened to nothing |
| Turret encoder | none | REV Through Bore V1 (REV-11-1271) fitted, a V2 (REV-11-3174) on order (the owner, 2026-11-13; what differs: [`wiring-findings.md`](wiring-findings.md#the-nectar-turret-encoder)), geared 1:2.4 to the turret (team-reported 2026-11-11). The CAD draws a V1 on the pinion shaft, not the turret's axis: 50/21 = 2.381 turns per turret turn, the opposite way; 19,504.76 counts per turret turn, 54.18 per degree ([`wiring-findings.md`](wiring-findings.md#the-nectar-turret-encoder)). Wiring, decided 2026-11-12: its count on Control Hub encoder 2, zeroed once at INIT from its absolute output on the OctoQuad ([`wiring-findings.md`](wiring-findings.md#zeroing-the-nectar-turret)) |
| Turret speed, no load | not derived | 672°/s at 6 V, 550°/s at 4.8 V (the vendor's servo speed × 1.008): 180° in 0.27 s at 6 V |
| Turret travel | not derived | 200° (team-reported 2026-11-11); no limit in the mechanism; 317.5° with the SDK's default 600 to 2400 µs pulse range, 352.8° with 500 to 2500 µs |
| Turret hard stops | none in CAD | none in CAD: the ring gear has teeth all the way round and a sweep of the turret hit no fixed part |
| Flywheel | one 72 mm goBILDA Rhino wheel (3614-0014-0072, 85 g), no added mass | same wheel |
| Flywheel motor | goBILDA 5000-0002-4008, 1:1 by belt (30T:30T) | same motor, ratio assumed 1:1 |
| Launch | one flywheel against a moving hood that shares the wheel's axis, fixed side guides, no second roller at the exit | same |
| Squeeze on the ball (inferred) | about 1.1 mm | about 1.3 mm |
| Hood drive | goBILDA 2000-0025-0003 servo through 8.571:1 | same |
| Hood travel | 0 to 30° | 0 to 20° |
| Launch angle above horizontal | 57.7° plus the hood angle: 57.7 to 87.7° | 60.8° plus the hood angle: 60.8 to 80.8° (hood reshaped since, not re-derived) |
| Exit point, ball centre (inferred) | 12.46 in up and 1.56 in ahead of the turret axis at 0°; 13.83 in and 1.99 in at 30° | 12.53 in and 1.23 in at 0°; 13.57 in and 1.60 in at 20° |
| Camera | Arducam OV9782 on the end of the hood, about 30° up (28 to 34 adjustable); position not analysed | Limelight 3A, 6.32 in behind the turret axis and 16.34 in up, facing opposite the shot, 30° down |

- Ball masses are the vendor's figures as recorded in the CAD notes; we could not re-fetch the vendor page on
  2026-11-10. The manual gives diameters only.
- The flywheel's inertia is not in the CAD. An estimate from the parts is 0.78 × 10⁻⁴ kg·m² (inferred), leaving out
  the motor's rotor and a 2 in feed roller belted to the flywheel shaft. What that means for rapid fire is in
  [`flywheel-control-findings.md`](flywheel-control-findings.md).
- The rest of the robot: four drive motors, an intake motor and a transfer motor (8 motors in all); servos for the
  flower intake, the blocker before the shooters, and a continuous-rotation Axon MAX that drives the transfer's
  compliant wheels (8 servos in all, at 6 V through two goBILDA servo power injectors); two RGB indicator lights on hub
  servo ports; a goBILDA Prism driving LED strips. Battery to a Floodgate switch to an XT30 distribution board to the
  Control Hub and Expansion Hub (RS485). The OctoQuad carries two odometry pods and the nectar turret encoder. Both
  cameras share a USB hub with its own power bank.
- One transfer channel on the centreline carries both ball sizes and exits under each turret. Its speed alone would
  pass about 10.9 balls per second (inferred, and the transfer has changed since).
- Mass 14.6 to 17.6 kg depending on plate material (inferred). The frame is 16.25 in wide over the outer side plates
  and 15.48 in long, 16.74 in over the widest fixed brackets (read from the CAD, 2026-11-11). With the intake arm
  deployed the robot is about 20.9 in long (measured on a render); no retracted pose is modelled.
- Where the two hubs sit, and what blocks their USB ports:
  [`wiring-findings.md`](wiring-findings.md#where-the-hubs-sit-and-the-usb-cable-between-them).
- The nectar turret's gears, encoder position, speed and travel are from saved reads of the CAD's turret document,
  analysed 2026-11-11: its mates as of 2026-11-04 and its placement in the full robot as of 2026-11-08. Later CAD
  changes are not in them. The speeds use Axon's table for the Mini (0.110, 0.090, 0.080 and 0.068 s per 60° at
  4.8, 6.0, 7.4 and 8.4 V) and are unloaded: the real turret is slower by an amount nobody has computed.

What code has to respect:

- **Clamp the hood in software.** The hoods have a lower stop at about −0.4° and no upper stop: past the software
  limit the sector jams against its pinion near 44°.
- **The hood moves about 78° per second** (vendor: 0.09 s per 60° at 6 V, through 8.571:1; no load): 2° in 26 ms, 5°
  in 64 ms. The SDK's default servo pulse range (600 to 2400 µs) gives 270° of this servo's 300°, which is 31.5° of
  hood; call `setPwmRange(new PwmRange(500, 2500))` for the full 35°.
- **Some turret and hood combinations collide or block a shot.** With both turrets on one heading, the pollen hood at
  30° and the nectar hood near 0° come within 0.10 in at headings from about 36° one side of straight ahead to 26° the
  other. At low nectar hood angles, within about 25° of straight back, the pollen shooter's servo is in the nectar
  ball's path. No full map of clashes exists: fill `ShotTuning.keepOuts` from the real robot.
- **Both cameras turn with their turrets**, and neither tilts with a hood as far as the CAD shows. Ball positions
  from the Limelight need the nectar turret's angle; `BallFieldTransform.Mount` assumes a camera fixed to the chassis
  ([`limelight-findings.md`](limelight-findings.md)). The pollen turret has no encoder, so the Arducam's heading is
  only the commanded servo angle.
- **The nectar turret encoder cannot give the turret's angle on its own at power-on.** One encoder turn is 151.2° of
  turret, so its absolute output repeats 2.38 times per turret turn, and there is no hard stop to home against. The
  code needs a rough angle from elsewhere to pick the turn: see
  [`wiring-findings.md`](wiring-findings.md#zeroing-the-nectar-turret).
- **Nothing in the CAD couples the nectar encoder to its shaft.** The shaft is 8 mm REX; the CAD analysis lists the
  encoder's bore as 1/2 in hex with 3/8 in hex, 5 mm hex and 1/4 in round inserts (not re-read from REV here). The
  2.381 ratio assumes an adapter makes the rotor turn with that shaft.
- **The nectar shooter's side plates pass 0.025 mm over the face of the 63-tooth pinion**, at about 32 to 47° and 130
  to 156° either side of the modelled pose. A pinion that sits proud on the real robot becomes a stop there.
- **The pollen shooter may limit the nectar turret to about 290°** (72° one way, 218° the other from the modelled
  pose, before the two come within 2 mm). This is a coarse check with the pollen shooter held at one pose: a flag,
  not a limit.
- **Where the flywheel encoders plug in is undecided**: see
  [`flywheel-control-findings.md`](flywheel-control-findings.md#where-to-plug-the-flywheel-encoders).

## Setting a robot's size or mass

- **`docs/paths/CloseFlowerAuto.pp` draws the Betta bot (retired 2026-11-13, its size still in `RobotGeometry`)
  sideways: set `rWidth` to 16 and `rHeight` to 15.** In a
  `.pp` file of format 1.x (this one is 1.5.0), `rWidth` is the robot's length along its heading (front to back) and
  `rHeight` its width side to side, although the visualizer versions that wrote 1.x labelled them Width and Height.
  The file has `rWidth` 15 and `rHeight` 16, so the robot it draws and collision-checks is 1 in too short and 1 in
  too wide.
  - The current visualizer writes format 2.0.0, where `rLength` is along the heading and `rWidth` side to side,
    labelled Length and Width. It converts a 1.x file on load (`rWidth` to `rLength`, `rHeight` to `rWidth`), so it
    opens this file as Length 15, Width 16 and a re-export keeps the swap: set Length 16 and Width 15 there.
  - Read from the visualizer's source, <https://github.com/Pedro-Pathing/Visualizer> at 97104e0:
    `src/io/migrations.ts` ("The old width ran along the heading and the old height side to side"), `src/types.ts`,
    and `getRobotCorners` in `src/utils/geometry.ts`. The live site, <https://visualizer.pedropathing.com> (fetched
    2026-11-10), has the same migration and labels.
  - A new robot needs its length and width set in every `.pp` file and in `RobotGeometry`.
- **Nothing uses robot mass yet.** No code in the repo reads one, and Pedro core 3.0.2, revhub 3.0.2 and tuning 1.0.1
  have no mass or weight setting (read from their published artifacts, 2026-11-10). Keep the numbers for
  simulations.

| | Betta bot (retired 2026-11-13) | Cuttle bot (retired 2026-11-13) |
|---|---|---|
| mass | 13.5 kg | 16.3 kg |
| frame | 16 in front to back, 15 in side to side | not recorded |
| in code | `biobuzz/RobotGeometry`: `lengthIn` 16, `widthIn` 15 | none |

- Betta bot mass and frame: team-reported 2026-10-03, not re-measured. `RobotGeometry` also sets its intake:
  `intakeWidthIn` 15, its middle `intakeOffsetIn` 8 in ahead of the robot's centre.
- Cuttle bot mass: `mass(16.3)` in last season's Pedro constants, 2026-02-19, since commented out; Pedro 2.1.1's
  source gives no unit, so kg is assumed. Unverified.

## Known problems in test OpModes and modules

None of these is fixed. Each was read from source on 2026-11-11, with SDK 12.0.0's sources from the Gradle cache.

- **Turret Aim Test can't put the nectar turret at a calibrated end of 0 or 1.** `TurretServos.turnTo` keeps every
  turret inside −`offset` to 1 − `offset`, the room a second servo needs (`biobuzz/modules/Turret.java:203`), but
  the nectar turret has one servo (`:79`), and `offset` does nothing else for it (`:207`). With the source's
  `nectar.offset` 0.01 and `rightPosition` 1 (`:31-32`), dpad right commands 0.99 and Nectar Position reads 0.990; a
  negative offset would cut the left end instead. It matters only for a calibrated end within `offset` of 0 or 1.
  Until it's fixed, set `Turret.Tuning.nectar.offset` to 0. Proposed fix: clip by `offset` only on a turret with two
  servos. To check a fix: with `nectar.rightPosition` 1, dpad right must read 1.000 and face exactly robot-right,
  and pollen's second servo must still run `offset` above its first.
- **Intake Transfer Shooter Test's RPM spikes for one window when `reverseFlywheel` or `reverseEncoder` is flipped
  mid-run.** `rpm()` (`opmodes/test/IntakeTransferShooterTest.java:199-210`) divides the change in count across its
  window, and a flip changes the sign of new counts only: the SDK negates `getCurrentPosition()` on a REVERSE motor
  (`DcMotorImpl.adjustPosition`), `reverseEncoder` negates at `:203`, and nothing clears the window. Until the old
  samples leave the window (`rpmWindowMs`, 50 ms) the RPM is off by about 0.29 × the encoder count, some 300,000
  once the wheel has turned 125 times, the kV estimate spikes with it, and with `targetRpm` set and `kP` above 0
  `power()` pins the output at 0 or 1. Until it's fixed, flip them in open loop (`targetRpm` 0) and ignore that
  moment's readings. Proposed fix: do as `Shooter` does, applying `reverseEncoder` to the rate and clearing the window
  when the motor's direction flips (`biobuzz/modules/Shooter.java:308-312`, `:339-340`).
- **Intake Transfer Shooter Test's `stop()` doesn't stop the transfer's continuous-rotation servo.**
  `melonbotics.setPower(0)` (`opmodes/test/IntakeTransferShooterTest.java:183`) is a servo pulse-width write
  (`CRServoImpl.setPower`), which the SDK refuses during STOP without an error
  ([`sdk-hub-findings.md`](sdk-hub-findings.md#opmode-lifecycle)). Whether the servo stops then depends only on the
  hub's fail-safe, which nobody has measured. Until it's fixed, set `transfer.melonboticsPower` to 0 before pressing
  STOP. Proposed fix: `melonbotics.setPwmDisable()` in `stop()`, which the SDK still sends. To check, on
  `intake_transfer_shooter.xml`: set `melonboticsPower` to 0.5, press STOP before and after the fix, and watch
  whether the servo keeps turning; logcat shows a `nack rec'd` line for each refused write.
- **Simple Intake Transfer Shooter Test can't INIT on any hardware config in the repo.** It looks up `flywheel` and
  `hood` (`opmodes/test/SimpleIntakeTransferShooterTest.java:33`, `:36`). `intake_transfer_shooter.xml` names them
  `pollenFlywheel`, `nectarFlywheel`, `pollenHood` and `nectarHood`, no config has a `flywheel`, and none has had a
  `hood` since `betta_decode.xml` and `cuttle_decode.xml` were deleted (2026-11-14); `hardwareMap.get` throws on the
  first missing name (`HardwareMap.get`). Use Intake Transfer Shooter Test, which drives the same rig with both
  shooters; it lacks only the gamepad1 A blocker toggle, and its dashboard `blocker.open` does the same. Proposed fix:
  delete the OpMode, or keep it for a config made on the Driver Station.
- **Butterfly Test has no shipped config since `butterfly.xml` was deleted (2026-11-14):** it needs `fl`, `fr`, `bl`,
  `br`, `left_shifter` and `right_shifter` (`opmodes/test/ButterflyTest.java:30-36`), so a config made on the DS.

## Still unknown

- **Everything in the BIOBUZZ robot section, now that the robot is assembled (2026-11-13).** Test: measure each number
  on the robot and replace it here. First: flywheel inertia (spin-up at full power), the launch angle at each end of hood travel
  (video from the side), the exit height, and how many servos each turret has (the CAD draws a second, unmated
  servo on the nectar pinion).
- **How the nectar turret encoder is coupled to its 8 mm shaft, and its real slew speed under the shooter's
  inertia.** Test: look at the built turret; time a 180° move from the encoder count.
- **The BIOBUZZ robot's mass** (14.6 to 17.6 kg inferred from the CAD). Test: weigh it.
