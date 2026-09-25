package org.firstinspires.ftc.teamcode.opmodes.test.distance;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.maxbotix.MaxSonarI2CXL;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Needs a "MaxSonar I2CXL" named "sonar". The sensor pings rather than ranging continuously, so the
 * reading holds flat between pings; that staircase width is its real latency. Gamepad A resets min/max.
 */
@TeleOp(name = "Maxbotix Ultrasonic", group = "Test")
public class MaxbotixUltrasonicTest extends OpMode {

    @Config("Ultrasonic Sensor Test")
    public static class Tuning {
        /** Ping-to-read delay: 100 ms covers full range, ~25 ms a close target; too short reads stale data. */
        public static int propagationDelayMs = 100;
    }

    /** SDK-side address. Change only if the sensor's EEPROM was rewritten by ConfigureUltrasonicAddress. */
    private static final int I2C_ADDR_8BIT = 0xE0;

    /** MB1242 reports anything closer than this as 20 cm — treat readings at the floor as "too close to trust". */
    private static final double MIN_RELIABLE_CM = 20;
    private static final double MAX_RANGE_CM = 765;

    private MaxSonarI2CXL sonar;
    private FtcDashboard dashboard;
    private final ElapsedTime loopTimer = new ElapsedTime();

    private double minCm = Double.POSITIVE_INFINITY;
    private double maxCm = Double.NEGATIVE_INFINITY;

    @Override
    public void init() {
        sonar = hardwareMap.get(MaxSonarI2CXL.class, "sonar");
        sonar.setI2cAddress(I2cAddr.create8bit(I2C_ADDR_8BIT));
        dashboard = FtcDashboard.getInstance();

        // Before the first ping the sensor answers a read with power-up info bytes, not a range.
        sonar.getDistanceSync(Tuning.propagationDelayMs, DistanceUnit.CM);

        telemetry.addLine("MB1242 ready. Press Play.");
        telemetry.update();
    }

    @Override
    public void loop() {
        // A second async call with another DistanceUnit would ping again and mix units in the driver's cache.
        double cm = sonar.getDistanceAsync(Tuning.propagationDelayMs, DistanceUnit.CM);
        double mm = DistanceUnit.MM.fromCm(cm);
        double in = DistanceUnit.INCH.fromCm(cm);
        boolean valid = cm > MIN_RELIABLE_CM && cm <= MAX_RANGE_CM;

        if (gamepad1.a) {
            minCm = Double.POSITIVE_INFINITY;
            maxCm = Double.NEGATIVE_INFINITY;
        }
        if (valid) {
            minCm = Math.min(minCm, cm);
            maxCm = Math.max(maxCm, cm);
        }

        double loopMs = loopTimer.milliseconds();
        loopTimer.reset();

        telemetry.addData("mm",      "%.0f", mm);
        telemetry.addData("cm",      "%.0f", cm);
        telemetry.addData("in",      "%.2f", in);
        telemetry.addData("valid",   valid);
        telemetry.addData("min cm",  "%.0f", minCm);
        telemetry.addData("max cm",  "%.0f", maxCm);
        telemetry.addData("loop ms", "%.1f", loopMs);
        telemetry.update();

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("distance_mm", mm);
        packet.put("distance_cm", cm);
        packet.put("distance_in", in);
        packet.put("valid",       valid ? 1 : 0);
        packet.put("loop_ms",     loopMs);
        dashboard.sendTelemetryPacket(packet);
    }
}
