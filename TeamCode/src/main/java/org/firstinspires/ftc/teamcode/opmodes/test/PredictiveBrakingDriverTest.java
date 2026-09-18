package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.maxbotix.MaxSonarI2CXL;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.I2cAddr;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Driver-controlled version of {@link PredictiveBrakingUltrasonicTest} — mecanum drive on gamepad 1
 * with the forward axis running through the same {@link PredictiveBraking} clamp. Both opmodes read
 * the same "Braking Law" dashboard block, so anything tuned in the automatic test carries straight
 * over to here.
 *
 * Hardware config: motors "fl", "bl", "fr", "br", and a "MaxSonar I2CXL" named "sonar".
 *
 * Controls:
 *   left stick    — translate (forward/back, strafe)
 *   right stick X — turn
 *   right bumper  — hold to override braking entirely, for A/B-ing against the raw drivetrain
 *   left bumper   — hold for slow mode
 *
 * Only forward motion is clamped, since the sensor faces forward — strafing and turning are always
 * at full authority, and backing away is never restricted. Releasing the stick or pulling back
 * clears the stop latch, so a driver can pull off a wall and re-approach without touching anything.
 */
@TeleOp(name = "Predictive Braking (Driver)", group = "Test")
public class PredictiveBrakingDriverTest extends OpMode {

    @Config("Braking Driver Test")
    public static class Tuning {
        /** True: skip driving, spin every wheel at 20% so you can spot a reversed one. */
        public static boolean wheelTest = false;
        public static double wheelTestPower = 0.2;
        /** Overall scale on driver input. */
        public static double driveSpeed = 1.0;
        /** Scale while the left bumper is held. */
        public static double slowModeSpeed = 0.35;
        /** Stick deflection below this is ignored. */
        public static double stickDeadzone = 0.05;

        public static boolean reverseFrontLeft  = true;
        public static boolean reverseBackLeft   = true;
        public static boolean reverseFrontRight = false;
        public static boolean reverseBackRight  = false;

        /** Ping-to-read delay. 100 ms is the datasheet's full-range figure; lower updates faster. */
        public static int propagationDelayMs = 100;
    }

    /** SDK-side address. Change only if the sensor's EEPROM was rewritten by ConfigureUltrasonicAddress. */
    private static final int I2C_ADDR_8BIT = 0xE0;

    private DcMotorEx frontLeft, backLeft, frontRight, backRight;
    private MaxSonarI2CXL sonar;
    private PredictiveBraking braking;
    private FtcDashboard dashboard;

    @Override
    public void init() {
        frontLeft  = initMotor("fl");
        backLeft   = initMotor("bl");
        frontRight = initMotor("fr");
        backRight  = initMotor("br");

        sonar = hardwareMap.get(MaxSonarI2CXL.class, "sonar");
        sonar.setI2cAddress(I2cAddr.create8bit(I2C_ADDR_8BIT));
        // Before the first ping the sensor answers a read with power-up info bytes, not a range.
        sonar.getDistanceSync(Tuning.propagationDelayMs, DistanceUnit.CM);

        braking = new PredictiveBraking(
                () -> sonar.getDistanceAsync(Tuning.propagationDelayMs, DistanceUnit.CM));
        dashboard = FtcDashboard.getInstance();

        telemetry.addLine("Driver braking test. Left stick drives, right bumper overrides braking.");
        telemetry.update();
    }

    private DcMotorEx initMotor(String name) {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, name);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        return motor;
    }

    @Override
    public void loop() {
        if (Tuning.wheelTest) {
            driveMecanum(Tuning.wheelTestPower, 0, 0);
            telemetry.addLine("WHEEL TEST — every wheel should be driving the robot FORWARD.");
            telemetry.update();
            return;
        }

        braking.read();

        double scale = gamepad1.left_bumper ? Tuning.slowModeSpeed : Tuning.driveSpeed;
        double forward = deadzone(-gamepad1.left_stick_y) * scale;
        double strafe  = deadzone(gamepad1.left_stick_x) * scale;
        double turn    = deadzone(gamepad1.right_stick_x) * scale;

        // Backing off or stopping releases the latch, so the driver re-approaches just by pushing
        // the stick forward again rather than having to restart the opmode.
        if (forward <= 0) {
            braking.resetStop();
        }

        double requestedForward = forward;
        boolean override = gamepad1.right_bumper;
        if (!override) {
            forward = braking.clampApproachPower(forward);
        }

        driveMecanum(forward, strafe, turn);

        double cm = braking.getDistanceCm();

        telemetry.addData("distance cm",  "%.0f", cm);
        telemetry.addData("predicted cm", "%.1f", braking.getPredictedDistanceCm());
        telemetry.addData("closing cm/s", "%.1f", braking.getClosingVelocityCmPerSec());
        telemetry.addData("requested fwd", "%.3f", requestedForward);
        telemetry.addData("clamped fwd",   "%.3f", forward);
        telemetry.addData("override",      override);
        telemetry.addData("stopped",       braking.isStopped());
        telemetry.update();

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("distance_cm",   cm);
        packet.put("predicted_cm",  braking.getPredictedDistanceCm());
        packet.put("closing_cm_s",  braking.getClosingVelocityCmPerSec());
        packet.put("requested_fwd", requestedForward);
        packet.put("clamped_fwd",   forward);
        packet.put("stop_cm",       PredictiveBraking.Tuning.stopDistanceCm);
        dashboard.sendTelemetryPacket(packet);
    }

    private double deadzone(double v) {
        return Math.abs(v) < Tuning.stickDeadzone ? 0 : v;
    }

    private void driveMecanum(double forward, double strafe, double turn) {
        double fl = forward + strafe + turn;
        double bl = forward - strafe + turn;
        double fr = forward - strafe - turn;
        double br = forward + strafe - turn;

        // Scale the whole vector down rather than clipping each wheel, so the commanded heading is
        // preserved when the mix saturates.
        double max = Math.max(1.0, Math.max(Math.max(Math.abs(fl), Math.abs(bl)),
                                            Math.max(Math.abs(fr), Math.abs(br))));

        frontLeft.setPower(fl / max  * (Tuning.reverseFrontLeft  ? -1 : 1));
        backLeft.setPower(bl / max   * (Tuning.reverseBackLeft   ? -1 : 1));
        frontRight.setPower(fr / max * (Tuning.reverseFrontRight ? -1 : 1));
        backRight.setPower(br / max  * (Tuning.reverseBackRight  ? -1 : 1));
    }

    @Override
    public void stop() {
        driveMecanum(0, 0, 0);
    }
}
