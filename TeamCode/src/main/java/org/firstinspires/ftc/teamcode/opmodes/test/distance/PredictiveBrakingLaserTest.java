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
 * Drives straight forward at full power and brakes on the Brushland Labs laser rangefinder,
 * settling at {@link PredictiveBraking.Tuning#stopDistanceCm}. The ultrasonic twin of this opmode
 * is {@link PredictiveBrakingUltrasonicTest}; both share one {@link PredictiveBraking.Tuning}
 * block so the two sensors can be compared under identical braking settings.
 *
 * Hardware config:
 *   - Motors "fl", "bl", "fr", "br" on Control Hub motor ports 0-3.
 *   - I2C device of type "REV Color Sensor V3" named "Laser".
 *
 * Run with {@link Tuning#wheelTest} true first (the default). All four wheels spin at 20% and the
 * braking logic is skipped, so you can confirm every wheel drives the robot forward. Flip the
 * matching reverse{Corner} flag on the dashboard for any wheel spinning backwards — the flags are
 * applied live, no restart needed. Once all four agree, set wheelTest false and run for real.
 *
 * Unlike the ultrasonic, this sensor reads well below 20 cm, so stopDistanceCm can go lower here
 * if you want a tighter standoff. Run ConfigureLaserRangefinder first if the sensor hasn't been
 * set to LONG distance mode.
 */
@TeleOp(name = "Predictive Braking (Laser)", group = "Test")
public class PredictiveBrakingLaserTest extends OpMode {

    @Config("Braking Laser Test")
    public static class Tuning {
        /** True: skip braking, spin every wheel at 20% so you can spot a reversed one. */
        public static boolean wheelTest = true;
        public static double wheelTestPower = 0.2;
        /** Forward power the braking logic gets to clamp down from. */
        public static double driveSpeed = 1.0;

        // Starting guess copied from BettaConstants.drivetrainConfig (left side reversed). The wheel test
        // is there because this is a guess — correct it there, then port it back to BettaConstants.
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
            // getStatus() reflects the read we just did, so a bad status discards that sample and
            // PredictiveBraking holds the previous good distance instead.
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

    /** Same power to all four wheels — straight forward, no strafe, no turn. */
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
