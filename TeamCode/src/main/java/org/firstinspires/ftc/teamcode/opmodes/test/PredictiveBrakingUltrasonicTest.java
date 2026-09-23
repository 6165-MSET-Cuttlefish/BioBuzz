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
 * Drives straight forward at full power and brakes on the MaxBotix MB1242 ultrasonic, settling at
 * {@link PredictiveBraking.Tuning#stopDistanceCm}. No gamepad input — it starts driving on Play.
 *
 * Hardware config:
 *   - Motors "fl", "bl", "fr", "br" on Control Hub motor ports 0-3.
 *   - I2C device of type "MaxSonar I2CXL" named "sonar" (put it on bus 1, not 0).
 *
 * Run with {@link Tuning#wheelTest} true first (the default). All four wheels spin at 20% and the
 * braking logic is skipped, so you can confirm every wheel drives the robot forward. Flip the
 * matching reverse{Corner} flag on the dashboard for any wheel spinning backwards — the flags are
 * applied live, no restart needed. Once all four agree, set wheelTest false and run for real.
 *
 * The MB1242 floors at 20 cm — anything closer still reports 20 — so stopDistanceCm must stay above
 * that or the reading won't follow the robot down and it will creep in forever. 25 cm leaves a
 * usable margin.
 *
 * Gamepad A releases the stop latch if you want it to approach again without restarting.
 */
@TeleOp(name = "Predictive Braking (Ultrasonic)", group = "Test")
public class PredictiveBrakingUltrasonicTest extends OpMode {

    @Config("Braking Ultrasonic Test")
    public static class Tuning {
        /** True: skip braking, spin every wheel at 20% so you can spot a reversed one. */
        public static boolean wheelTest = false;
        public static double wheelTestPower = 0.2;
        /** Forward power the braking logic gets to clamp down from. */
        public static double driveSpeed = 1.0;

        // Starting guess copied from BettaConstants.drivetrainConfig (left side reversed). The wheel test
        // is there because this is a guess — correct it there, then port it back to BettaConstants.
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
        // Burn one blocking ping/read here so the braking logic never sees them.
        sonar.getDistanceSync(Tuning.propagationDelayMs, DistanceUnit.CM);

        braking = new PredictiveBraking(
                () -> sonar.getDistanceAsync(Tuning.propagationDelayMs, DistanceUnit.CM));
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

        if (gamepad1.a) {
            braking.resetStop();
        }

        braking.read();
        double power = braking.clampApproachPower(Tuning.driveSpeed);
        drive(power);

        double cm = braking.getDistanceCm();
        double predictedCm = braking.getPredictedDistanceCm();
        double closingCmPerSec = braking.getClosingVelocityCmPerSec();

        telemetry.addData("distance cm",  "%.0f", cm);
        telemetry.addData("model cm",     "%.1f", braking.getModelDistanceCm());
        telemetry.addData("model cm/s",   "%.1f", braking.getModelVelocityCmPerSec());
        telemetry.addData("stop dist cm", "%.1f", braking.getStoppingDistanceCm());
        telemetry.addData("predicted cm", "%.1f", predictedCm);
        telemetry.addData("closing cm/s", "%.1f", closingCmPerSec);
        telemetry.addData("power",        "%.3f", power);
        telemetry.addData("ceiling",      "%.3f", braking.getPowerCeiling());
        telemetry.addData("braking",      braking.isBraking());
        telemetry.addData("rev brake",    braking.isReverseBraking());
        telemetry.addData("stall escape", braking.isStallEscaping());
        telemetry.addData("settling",     braking.isSettling());
        telemetry.addData("stopped",      braking.isStopped());
        telemetry.update();

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("distance_cm",  cm);
        packet.put("current_cm",   braking.getCurrentDistanceCm());
        packet.put("model_cm",     braking.getModelDistanceCm());
        packet.put("model_cm_s",   braking.getModelVelocityCmPerSec());
        packet.put("stop_dist_cm", braking.getStoppingDistanceCm());
        packet.put("predicted_cm", predictedCm);
        packet.put("closing_cm_s", closingCmPerSec);
        packet.put("power",        power);
        packet.put("ceiling",      braking.getPowerCeiling());
        packet.put("stop_cm",      PredictiveBraking.Tuning.stopDistanceCm);
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
