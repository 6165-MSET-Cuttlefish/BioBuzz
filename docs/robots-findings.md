# Robot findings

What the code doesn't show about the robots: the BIOBUZZ robot as it stands in CAD, what no code holds or has checked
on the Cuttle bot (last season's robot), and the sizes and masses of it and the Betta bot (the robot BioBuzz code
runs on today). Read it before writing code for the BIOBUZZ robot's shooters, turrets or cameras, before running
anything on the Cuttle bot, or before putting a robot's size into code or a `.pp` path file.

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
| Turret encoder | none | REV Through Bore V1 on the pinion shaft, not the turret's axis: 50/21 = 2.381 turns per turret turn, the opposite way; 19,504.76 counts per turret turn, 54.18 per degree. The CAD notes plan it on the OctoQuad, [`wiring-findings.md`](wiring-findings.md) on a hub encoder port |
| Turret speed, no load | not derived | 672°/s at 6 V, 550°/s at 4.8 V (the vendor's servo speed × 1.008): 180° in 0.27 s at 6 V |
| Turret travel | not derived | no limit in the mechanism; 317.5° with the SDK's default 600 to 2400 µs pulse range, 352.8° with 500 to 2500 µs |
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

## Before running anything on the Cuttle bot

`decode/` is the only code for the Cuttle bot ([CLAUDE.md](../CLAUDE.md), Project context); its one OpMode is Decode
Tele, last season's TeleOp on this framework. Nothing in the repo shows it has run on that robot since it was ported,
and nothing below has been checked on it.

- **No code holds the PTOs or the initial lift.** A PTO (power take-off) is a servo-engaged coupling that lets the
  drive motors run the endgame lift; the initial lift is that lift's first stage, on two continuous-rotation servos.
  Before driving, check by hand that both PTOs are disengaged and stay disengaged with their servos unpowered. If they
  don't, hold them in a `decode/` module's `write()`.
  - The devices, in `TeamCode/src/main/res/xml/cuttle_decode.xml`: `leftPto` (Expansion Hub servo port 2, line 33),
    `rightPto` (Expansion Hub servo port 5, line 36), `leftInitial` and `rightInitial` (Control Hub servo ports 3
    and 4, lines 13-14), and the lift's analog encoders `leftInitialEncoder` and `rightInitialEncoder` (lines 40
    and 18). No Java file in `TeamCode/src/main/java` names any of them (read from source, 2026-11-10).
  - Last season's code held the PTOs disengaged at `leftPto` 0.4 and `rightPto` 0.5, and held the initial lift with a
    position loop on those encoders (state `HOLD_BELLYPAN`); `git log -S leftPto` finds it. Whether 0.4 and 0.5 still
    disengage is unchecked.
  - A hold in code covers only START to STOP. Before each OpMode's INIT the SDK turns every servo's pulse off
    (`OpModeManagerImpl.resetHardwareForOpMode()`, which reaches `LynxServoController.floatHardware()`; SDK 12.0.0
    source), and `EnhancedOpMode` writes no module during INIT. Through INIT the PTOs must stay put on their own.
- **Check drive and odometry directions before trusting Decode Tele.** It drives with last season's constants,
  copied into `decode/CuttleDrive.java` and not checked on the robot since (read from source):

  | setting | value | line |
  |---|---|---|
  | motor directions | `fl`, `bl` REVERSE; `fr`, `br` FORWARD | 27-30 |
  | Pinpoint pods | goBILDA 4-bar | 38 |
  | Pinpoint pod offsets | x -5.5 in, y 1.0 in | 39-40 |
  | Pinpoint encoder directions | x and y both REVERSED | 41-42 |

  - Wheels off the ground: drive forward, strafe left and turn left, and check every wheel turns the right way.
  - On the ground, in Decode Tele itself (Pinpoint Localizer Test uses `BettaConstants`): press gamepad1 B, which sets
    the pose facing 0° (the INIT placeholder pose faces 90°, where pushing forward raises Y instead). Push the robot
    forward, then left: `Robot Position` (ROBOT STATUS group) X must rise, then Y. Then turn it in place: X and Y
    should barely move.
  - The Pinpoint is on the Expansion Hub, I2C bus 3 (`cuttle_decode.xml` line 45), where each I2C read costs more
    loop time than on the Control Hub ([hub-bench-findings.md](hub-bench-findings.md)).
- **The hood can stop moving with no warning.** `Shooter.write()` (`decode/modules/Shooter.java` lines 164-167)
  writes the hood only when the computed position is within 0.585-0.905 (`HoodState.RESET` to `RESET + TOP`, lines
  75-84); outside that the servo stays where it was. If the hood stops following, compare the `Hood Position`
  telemetry line with that range. Last season's code did the same.

## Setting a robot's size or mass

- **`docs/paths/CloseFlowerAuto.pp` draws the Betta bot sideways: set `rWidth` to 16 and `rHeight` to 15.** In a
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

| | Betta bot | Cuttle bot |
|---|---|---|
| mass | 13.5 kg | 16.3 kg |
| frame | 16 in front to back, 15 in side to side | not recorded |
| in code | `biobuzz/RobotGeometry`: `lengthIn` 16, `widthIn` 15 | none |

- Betta bot mass and frame: team-reported 2026-10-03, not re-measured. `RobotGeometry` also sets its intake:
  `intakeWidthIn` 15, its middle `intakeOffsetIn` 8 in ahead of the robot's centre.
- Cuttle bot mass: `mass(16.3)` in last season's Pedro constants, 2026-02-19, since commented out; Pedro 2.1.1's
  source gives no unit, so kg is assumed. Unverified.

## Still unknown

- **Everything in the BIOBUZZ robot section, once the robot exists.** Test: measure each number on the robot and
  replace it here. First: flywheel inertia (spin-up at full power), the launch angle at each end of hood travel
  (video from the side), the exit height, and how many servos each turret has (the CAD draws a second, unmated
  servo on the nectar pinion).
- **How the nectar turret encoder is coupled to its 8 mm shaft, and its real slew speed under the shooter's
  inertia.** Test: look at the built turret; time a 180° move from the encoder count.
- **Do `leftPto` 0.4 and `rightPto` 0.5 still disengage?** Test: on the Cuttle bot, set each from the dashboard's
  Hardware view and look at the PTO.
- **What does the initial lift do with `leftInitial` and `rightInitial` unpowered?** Test: run Decode Tele through
  INIT, START and STOP and watch the lift.
- **Are Decode Tele's drive directions, Pinpoint encoder directions and pod offsets right?** Test: the checks above.
- **Both robots' current masses.** Test: weigh each robot as it stands.
- **The Cuttle bot's frame size.** Test: measure it front to back and side to side.
