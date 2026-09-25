package org.firstinspires.ftc.teamcode.opmodes.test.distance;

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
 * Gamepad mecanum drive with only the forward axis braked on the "sonar" MB1242, using the same
 * "Braking Law" block as {@link PredictiveBrakingUltrasonicTest}. Hold right bumper to bypass braking,
 * left bumper for slow mode.
 */
@TeleOp(name = "Predictive Braking (Driver)", group = "Test")
public class PredictiveBrakingDriverTest extends OpMode {

    @Config("Braking Driver Test")
    public static class Tuning {
        /** Skip driving and spin every wheel at wheelTestPower, to spot a reversed one. */
        public static boolean wheelTest = false;
        public static double wheelTestPower = 0.2;
        public static double driveSpeed = 1.0;
        public static double slowModeSpeed = 0.35;
        public static double stickDeadzone = 0.05;

        public static boolean reverseFrontLeft  = true;
        public static boolean reverseBackLeft   = true;
        public static boolean reverseFrontRight = false;
        public static boolean reverseBackRight  = false;

        /** Ping-to-read delay; 100 ms covers full range, lower updates faster. */
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
