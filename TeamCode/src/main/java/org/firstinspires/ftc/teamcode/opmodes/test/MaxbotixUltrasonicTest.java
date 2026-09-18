package org.firstinspires.ftc.teamcode.opmodes.test;

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
 * Bench test for the MaxBotix MB1242 (I2CXL-MaxSonar-EZ4) ultrasonic rangefinder.
 *
 * Hardware config: add an "MaxSonar I2CXL" on an I2C port, named "sonar".
 *
 * Unlike the laser rangefinders, this sensor is ping/wait/read rather than continuously ranging:
 * the ranging cycle takes 25 ms (close target) to 100 ms (no target), and the sensor ignores I2C
 * entirely while it's ranging. {@link MaxSonarI2CXL#getDistanceAsync} handles that without
 * blocking the loop — it reads the previous ping's result and re-pings only once
 * {@link Tuning#propagationDelayMs} has elapsed — so the reported distance holds flat between
 * pings and the dashboard plot is a staircase, not a curve. That staircase width is the real
 * sensor latency and is what matters for braking: at 100 ms, the robot has already travelled
 * a couple of inches at speed before the reading updates.
 *
 * Gamepad A resets the min/max hold.
 */
@TeleOp(name = "Maxbotix Ultrasonic", group = "Test")
public class MaxbotixUltrasonicTest extends OpMode {

    @Config("Ultrasonic Sensor Test")
    public static class Tuning {
        /**
         * Delay between commanding a ping and reading the result. The datasheet wants 100 ms
         * between range commands for a full-range cycle; shorter is fine (down to ~25 ms) when
         * the target is close, but too short and the read lands mid-cycle and returns stale data.
         * Turn this down to find the fastest update rate that still tracks a wall approach.
         */
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
        // Burn one blocking ping/read here so loop() never sees them.
        sonar.getDistanceSync(Tuning.propagationDelayMs, DistanceUnit.CM);

        telemetry.addLine("MB1242 ready. Press Play.");
        telemetry.update();
    }

    @Override
    public void loop() {
        // One async call per loop, then convert. Calling it again with a different DistanceUnit
        // would issue a second ping and leave the driver's cache in mixed units.
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
