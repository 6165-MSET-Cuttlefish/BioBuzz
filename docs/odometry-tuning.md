# Odometry tuning

How the BIOBUZZ robot knows where it is, and the checklist that turns its starting values into its own. Read it
before measuring the robot's drive or odometry values, changing how the follower gets its position, or setting
`BioBuzzConstants.measured`.

Java paths below are relative to `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`.

## What to do

1. **Nothing that follows a path runs until the checklist below is done.** Nothing about the pods or the drive is
   measured on this robot. Every value in `pedro/BioBuzzConstants` starts as the retired Betta bot's, apart from the
   pods' channels (0 and 1, from the team's wiring chart) and their counts per millimetre (the goBILDA 4-bar pod's
   own figure): pod directions, offsets, the heading scale, drive directions and Foresight gains are placeholders. Until `BioBuzzConstants.measured`
   is `true`, every OpMode that lets Pedro drive, apart from BioBuzz Tele, stops at INIT with one sentence saying what
   to measure, AutoTune's Tests refuses every test but Pose Test, and AutoTune doesn't list its Foresight Tuner at all
   (step 8). BioBuzz Tele and the other hand-driven OpModes run, but BioBuzz Tele's heading lock (LB) doesn't engage: a press does
   nothing, and the Driver Station says the heading lock waits for the robot's odometry to be measured. Its shot
   waypoint drive (LT) refuses the same way, with three rumble blips and the reason on its `Waypoint Drive` line.
2. **Keep the robot still from INIT until the Driver Station's `OctoQuad` line reads `RUNNING`** (about 2 s). Every
   INIT resets the OctoQuad and recalibrates its IMU.
3. **Do the checklist in order**, and copy each number into `pedro/BioBuzzConstants.java` as you go: dashboard
   edits are lost at the next deploy.

## How the robot gets its position

- **The board.** An OctoQuad MK2, named `octoquad` in `biobuzz.xml`, on Control Hub I2C bus 2. Two goBILDA 4-bar
  odometry pods plug into its channels 0 and 1 ([`wiring-findings.md`](wiring-findings.md)). The heading comes from
  the MK2's own IMU, not the pods and not the Control Hub's IMU. The robot has no Pinpoint this year
  ([`robots-findings.md`](robots-findings.md)).
- **The code.** `BioBuzzConstants.create(opMode)` builds every BIOBUZZ follower (`BioBuzzRobot`,
  `BallCollectionRobot`, `BarePedroAuto`) on `GuardedOctoQuadLocalizer` (`architecture/hardware/`), configured from
  `BioBuzzConstants.octoQuad`. Every value it needs is in that one group, which the dashboard shows under
  BioBuzz Constants.
- **INIT.** Building the localizer writes the settings to the board and calls `resetLocalizerAndCalibrateIMU()`.
  It doesn't wait: the `OctoQuad` status line shows the board's state, and an `OctoQuad` fault appears if it isn't
  `RUNNING` 3 s after the reset or reports `FAULT_NO_IMU` (an MK1). START waits for `RUNNING` up to those 3 s, then
  stops the OpMode with an error naming the problem (`architecture/core/EnhancedOpMode.java`). Forward Back Test,
  which isn't an `EnhancedOpMode`, shows the same status line in INIT and does the same wait at START; STOP during
  that wait ends it at once, as it does in OctoQuad Localizer Test. On the bench the OctoQuad reached `RUNNING`
  1666 ms after a reset ([`hub-bench-findings.md`](hub-bench-findings.md)).
- **The start pose.** A pose set before `RUNNING` (as every auto does in `initialize()`) is held and written to the
  board at the first `RUNNING` read, so the IMU calibration doesn't lose it. The board takes whole millimetres, so a
  pose lands up to 1 mm short of what was asked.
- **Pose and heading.** Inches, heading in radians in [0, 2π) as Pedro normalises it. At heading 0, x is the robot's
  forward and y its left; heading grows counter-clockwise.
- **Velocity.** The board's own velocity (over `velocityIntervalMs`, 25 ms) goes to Pedro unrotated, which assumes it
  is in field axes. Pedro's braking and `Camera`'s field-ball velocities both use it. Nobody has checked which frame
  the board reports in: checklist step 7.
- **Heading lock and field-centric drive** (`biobuzz/modules/Drivetrain.java`) read the follower's heading, now the
  OctoQuad's IMU. The heading lock turns the chassis by itself (P = 1, up to full turn power, over the right stick),
  so a heading that counts the wrong way would make one press spin the robot. Until `measured` is set,
  `Drivetrain.lockHeading` does nothing and BioBuzz Tele shows one line saying so instead of the Heading Lock line.
  Field-centric drive only rotates the left stick by the heading, so it stays allowed, but it will feel wrong (the
  robot drifting off the stick's direction as it turns) until steps 2 and 5 are done; no OpMode turns it on yet.
  BioBuzz Tele's shot waypoint drive (`biobuzz/WaypointDrive.java`) lets Pedro drive a planned path, so until
  `measured` is set it refuses before it builds anything. Nothing else in the hand-driven OpModes turns the pose or
  heading into wheel power: the turrets aim from the pose, the chassis doesn't. Anything new that would (an auto-aim
  that turns the chassis, a hold, a brake to a pose) waits for `measured` the same way.
  `biobuzz/modules/HeadingLockGateTest` checks that a heading lock press leaves the wheels still until then; no test
  covers the waypoint drive's refusal.
- **If the OctoQuad stops answering once running**, more than 250 ms without a valid read stops the OpMode with an
  error naming it.
- **The gate.** `BioBuzzConstants.create` refuses at INIT, before it touches any hardware, unless `measured` is set
  or the OpMode's own class carries `@PedroNeverDrives` (`pedro/PedroNeverDrives.java`). The mark isn't inherited.
  `measured` isn't public, so the dashboard doesn't show it: opening the gate takes a source edit and a deploy.
  Marked: BioBuzz Tele, SOTM Test, Turret Aim Test, Cell Tip Test, BioBuzz Shooter Test and Camera Module Test.
  Gated: Forward Back Test, Close Flower Auto, Ball Collection Auto, Mock Architecture Test, Ball Collection and
  Vision Ball Collection. AutoTune's Tests refuses the same way every test that builds the drivetrain: Hold, Line,
  Curve, Interpolation, Localization, Driving and Odometry. Odometry drives by itself: one click on the AutoTune page
  starts it, and it drives forward, then left, then turns, each at half power for 1 s, with nobody at the sticks.
  Only Pose Test runs: it reads the OctoQuad and never builds the drivetrain. The checklist reads the position in
  OctoQuad Localizer Test. AutoTune doesn't list the Foresight Tuner (`pedro/Tuning.java`).
  `pedro/BioBuzzConstantsTest` checks the gate, the settings reaching the board and the pose conventions above.

How this is known: read from this repository's source and from Pedro `revhub` 3.0.2's `OctoQuadConfig` and
`OctoQuadLocalizer` sources on 2026-11-14. None of it has run on the robot: the checklist is its first run.

## The checklist

Do these at the robot, in order. Each step says which OpMode to run, what to do, what a correct result looks like
and where the number goes. "The dashboard" is `http://192.168.43.1:8080/dash` on the robot's Wi-Fi; the values are
under **BioBuzz Constants**. After every step, copy what you changed into `pedro/BioBuzzConstants.java`.

**Before you start:** a full install (the Run button) of the code with this page, `biobuzz` active on the Driver
Station, a charged battery, and a flat floor with 8 ft of straight space beside a wall or a taped line. Keep the
turrets still throughout: the nectar turret's encoder is on the same OctoQuad.

### 1. Drive motor directions

- **OpMode:** Mecanum (Test).
- **Do:** put the robot on blocks with every wheel off the floor. INIT, START, then push the left stick forward.
- **Correct:** all four wheels turn the way that would drive the robot forward. Left stick right: the front-left
  and back-right wheels forward, the other two backward (strafe right). Right stick right: the left wheels forward,
  the right wheels backward (turn clockwise).
- **If a wheel is wrong:** flip that wheel's direction (`mecanum.frontLeftDirection` and so on) on the dashboard,
  then STOP and INIT again: the directions are read at INIT.
- **Where it goes:** `BioBuzzConstants.MecanumSettings`.
- **AutoTune's Mecanum Tuner** finds the same directions another way: it spins one drive motor at a time at half
  power until STOP, with nobody at the sticks. Run it only with the robot on blocks and every
  wheel off the floor, never on the ground. It is optional: no other step uses it or its results.

### 2. The OctoQuad answers and is an MK2

- **OpMode:** OctoQuad Localizer Test (Test).
- **Do:** robot on the floor and still. INIT and wait. Then START, press B, and turn the robot a quarter turn
  counter-clockwise (seen from above) by hand.
- **Correct:** `Status` turns to `RUNNING` within about 2 s, and `Firmware` shows a version. After the turn,
  heading reads 90 ± 5: counter-clockwise counts up.
- **If not:** `FAULT_NO_IMU`, or START stopping with "needs an OctoQuad MK2", means an MK1: it can't be the robot's
  odometry, so stop and tell the software lead. "isn't answering" means the I2C cable or the port: it must be
  Control Hub I2C bus 2. A heading near 270 after the turn means the heading counts clockwise: stop and tell the
  software lead.
- **Where it goes:** write the firmware version in [`robots-findings.md`](robots-findings.md).

### 3. Which channel is the forward pod, which way each pod counts, and counts per millimetre

- **OpMode:** OctoQuad Deadwheel Calibrator (Test). It is the board maker's own tool
  (`vendor/digitalchickenlabs/DeadwheelCalibrator.java`).
- **Do:** follow its prompts with gamepad 1's right bumper. When it asks for +X, push the robot exactly 8 ft
  straight forward (the robot's front leading) along the wall or tape, without turning it. When it asks for +Y,
  push it exactly 8 ft straight to the robot's left. Step 4 checks the result.
- **Correct:** it names channel 0 for one wheel and 1 for the other, and both ticks/mm read within 1% of 19.89
  (19.69 to 20.09).
- **Where it goes:**
  - `octoQuad.forwardPodChannel`: the X wheel's port. `octoQuad.strafePodChannel`: the Y wheel's port.
  - `octoQuad.forwardPodDirection`: `REVERSE` if "X wheel port should be reversed" says true, else `FORWARD`.
    `octoQuad.strafePodDirection`: the same for Y.
  - `octoQuad.countsPerMm`: leave it at 19.89436789 (the goBILDA 4-bar pod's own figure) when both readings are
    within 1%. If both are off by the same amount, the pods aren't 4-bar pods: use the average of the two and note
    the pod type in [`robots-findings.md`](robots-findings.md). If they differ from each other, redo the pushes:
    the board takes one value for both pods.
- **Note:** the calibrator resets every OctoQuad setting to its default. That's harmless: every INIT writes ours.

### 4. Check the channels and directions

- **OpMode:** OctoQuad Localizer Test.
- **Do:** set step 3's values on the dashboard, INIT, and press A (applies BioBuzz Constants and recalibrates; keep
  still until `RUNNING`). START, press B to zero the pose. Push the robot 48 in straight forward along the tape.
  Press B, then push it 48 in straight to its left.
- **Correct:** forward: x reads 48 ± 0.5 and y stays within ± 0.5. Left: y reads 48 ± 0.5 and x stays within ± 0.5.
  The two pod-count lines rise in the direction their labels say.
- **If not:** a value going down means that pod's direction is flipped. x moving when you push left means the
  channels are swapped. A distance off by more than 1% means redo step 3.

### 5. The IMU's heading scale

- **OpMode:** OctoQuad Heading Scalar Calibrator (Test), the board maker's own tool
  (`vendor/digitalchickenlabs/HeadingScalarCalibrator.java`).
- **Do:** flat floor, one side of the robot against a wall's baseboard. Follow its prompts with the right bumper.
  Don't touch the robot during its 10 s settle. Then turn it 5 full turns counter-clockwise, gently and without
  tipping it, back against the same baseboard, and press the right bumper.
- **Correct:** a scalar between 0.9 and 1.1; the tool refuses anything outside that. Expect within 1% of 1.
- **Don't** press the bumper again to save it to the OctoQuad's flash: our code writes the scale at every INIT.
- **Where it goes:** `octoQuad.headingScalar`.
- **Check:** OctoQuad Localizer Test, A, START, B, then 5 turns counter-clockwise by hand back to the baseboard:
  heading reads within 1° of 0 (0 to 1, or 359 to 360).

### 6. Where the pods sit

- **Measure** on the robot or in the CAD, from the robot's centre (the middle of the drivetrain, halfway between
  the left and right wheels and halfway between the front and back wheels). Use the same point for the Camera
  Mount and the turrets' `x`/`y`.
  - `octoQuad.forwardPodLeftOfCentreIn`: how far the forward pod's wheel is to the left of the centre, in inches.
    To the right is negative.
  - `octoQuad.strafePodForwardOfCentreIn`: how far the strafe pod's wheel is in front of the centre, in inches.
    Behind is negative.
- **Check:** OctoQuad Localizer Test, A, START, B, then turn the robot in place on the right stick, slowly, 2 full
  turns. x and y stay within ± 1 in the whole time.
- **If they trace a circle:** the offsets are off. Run AutoTune's OctoQuad Tuner
  (`http://192.168.43.1:10158`; forward and strafe pod ports from step 3, pod type FOUR_BAR) and use its
  `xPodOffset` for `forwardPodLeftOfCentreIn` and `yPodOffset` for `strafePodForwardOfCentreIn`, then check again.
  Its other results don't replace steps 3 and 5.

### 7. Which frame the velocity is in

- **OpMode:** OctoQuad Localizer Test.
- **Do:** START, B, turn the robot to face its left (heading 90 ± 5), then push it steadily toward its front.
- **Correct:** `vy` grows and `vx` stays near 0. The board reports field-axis velocity, as the code assumes.
- **If `vx` grows instead:** the board reports robot-axis velocity. Stop here: write it in this page, and the code
  must rotate the velocity in `GuardedOctoQuadLocalizer` before anything follows a path.

### 8. Foresight gains

- **First:** AutoTune doesn't list the Foresight Tuner. The vendored copy drives at full power until the pose passes
  Distance, with no STOP check and no runaway stop, so a pod that doesn't count would drive the robot on until the
  app restarts. A software lead re-copies the procedures and moves `tuning` to 1.0.3
  ([`pedro-findings.md`](pedro-findings.md), What to do, 2), then registers it again in `pedro/Tuning.java` with
  `BioBuzzConstants::readyLocalizer` and `BioBuzzConstants::drivetrain`.
- **Then:** AutoTune's Foresight Tuner, with the space [`pedro-findings.md`](pedro-findings.md) asks for and a
  hand on STOP. It reads the position from `BioBuzzConstants`' OctoQuad, so steps 1 to 7 must be done.
- **Where it goes:** `BioBuzzConstants.ForesightSettings`.

### 9. Open the gate and run the first path

- **Do:** set `measured = true` in `pedro/BioBuzzConstants.java`, deploy, and run AutoTune's Tests: Line, Distance
  24, then 48. Line drives out and back until STOP: press STOP when the robot is back at its start after one or two
  laps. Then Forward Back Test, which drives 80 in forward and back once, then holds: clear 7 ft ahead and keep a
  hand on STOP.
- **Correct:** at each stop the robot is within about 1 in and 2° of where it started.
- **Never** set `measured` to get past the INIT message without doing steps 1 to 8: it is the only thing between a
  path, BioBuzz Tele's heading lock or its shot waypoint drive, and the Betta bot's numbers.

## Still unknown

- The robot's OctoQuad firmware, and that it is an MK2 on the robot as well as on the bench: step 2.
- Every pod value: which way each pod counts, counts per millimetre on this robot and where the pods sit: steps 3 to 6.
- The velocity frame: step 7.
- Whether `resetLocalizerAndCalibrateIMU()` at INIT clears the channel-4 pulse-width setting the nectar turret's
  zero will need ([`wiring-findings.md`](wiring-findings.md)). No robot code reads channel 4 yet.
- A cable pull while BioBuzz Tele runs stops the OpMode. Freezing the pose instead, in a TeleOp driven by hand, is
  planned but not in this code.
