package org.firstinspires.ftc.teamcode.opmodes.test.distance;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.architecture.hardware.LaserRangefinder;

/**
 * Drives straight forward on Play and brakes on the laser ("Laser"), sharing the "Braking Law" block
 * with the ultrasonic test so the sensors compare like for like. Run {@link Tuning#wheelTest} first and
 * flip reverse flags until every wheel drives forward. Run ConfigureLaserRangefinder first for LONG mode.
 */
@TeleOp(name = "Predictive Braking (Laser)", group = "Test")
public class PredictiveBrakingLaserTest extends OpMode {

    @Config("Braking Laser Test")
    public static class Tuning {
        /** Skip braking and spin every wheel at wheelTestPower, to spot a reversed one. */
        public static boolean wheelTest = true;
        public static double wheelTestPower = 0.2;
        public static double driveSpeed = 1.0;

        // Copied from BettaConstants.drivetrainConfig; wheelTest confirms them.
        public static boolean reverseFrontLeft  = true;
        public static boolean reverseBackLeft   = true;
        public static boolean reverseFrontRight = false;
        public static boolean reverseBackRight  = false;
    }

    /** Brushland status: 0 is a good reading, 1-2 usable, anything higher is junk. */
    private static final int MAX_USABLE_STATUS = 2;

    private DcMotorEx frontLeft, backLeft, frontRight, backRight;
    private LaserRangefinder laser;
    private PredictiveBraking braking;
    private FtcDashboard dashboard;

    @Override
    public void init() {
        frontLeft  = initMotor("fl");
        backLeft   = initMotor("bl");
        frontRight = initMotor("fr");
        backRight  = initMotor("br");

        laser = new LaserRangefinder(hardwareMap.get(RevColorSensorV3.class, "Laser"));
        braking = new PredictiveBraking(() -> {
            double cm = laser.getDistance(DistanceUnit.CM);
            // getStatus() describes the read just above; NaN makes PredictiveBraking keep the last good one.
            return laser.getStatus() <= MAX_USABLE_STATUS ? cm : Double.NaN;
        });
        dashboard = FtcDashboard.getInstance();

        telemetry.addLine(Tuning.wheelTest
                ? "WHEEL TEST mode — wheels will spin at 20% on Play."
                : "Braking mode — robot will drive forward at speed on Play.");
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
            drive(Tuning.wheelTestPower);
            telemetry.addLine("WHEEL TEST — every wheel should be driving the robot FORWARD.");
            telemetry.addData("power", "%.2f", Tuning.wheelTestPower);
            telemetry.update();
            return;
        }

        braking.read();
        double power = braking.clampApproachPower(Tuning.driveSpeed);
        drive(power);

        double cm = braking.getDistanceCm();

        telemetry.addData("distance cm", "%.1f", cm);
        telemetry.addData("distance in", "%.2f", DistanceUnit.INCH.fromCm(cm));
        telemetry.addData("status",      laser.getStatus());
        telemetry.addData("power",       "%.3f", power);
        telemetry.addData("ceiling",     "%.3f", braking.getPowerCeiling());
        telemetry.addData("braking",     braking.isBraking());
        telemetry.update();

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("distance_cm", cm);
        packet.put("status",      laser.getStatus());
        packet.put("power",       power);
        packet.put("ceiling",     braking.getPowerCeiling());
        packet.put("stop_cm",     PredictiveBraking.Tuning.stopDistanceCm);
        dashboard.sendTelemetryPacket(packet);
    }

    private void drive(double power) {
        frontLeft.setPower(power  * (Tuning.reverseFrontLeft  ? -1 : 1));
        backLeft.setPower(power   * (Tuning.reverseBackLeft   ? -1 : 1));
        frontRight.setPower(power * (Tuning.reverseFrontRight ? -1 : 1));
        backRight.setPower(power  * (Tuning.reverseBackRight  ? -1 : 1));
    }

    @Override
    public void stop() {
        drive(0);
    }
}
