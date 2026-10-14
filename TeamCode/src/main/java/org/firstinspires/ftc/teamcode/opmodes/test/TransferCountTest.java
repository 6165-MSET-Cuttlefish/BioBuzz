package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "Transfer Count Test", group = "Test")
@Config
public class TransferCountTest extends OpMode {
    public static String upstreamSensorName = "analog0";
    public static String downstreamSensorName = "analog1";
    public static String motor1Name = "transfer1";
    // Must match the analog bounds written by Configure Laser Rangefinder.
    public static double maxRangeMm = 1300;
    public static double emptyAboveMm = 80;
    public static double bothOnBallBelowMm = 50;
    public static double seamAtUpstreamDiffMm = 20;
    public static double seamAtDownstreamDiffMm = 10;
    public static int stopAtCount = 2;
    public static double motor1Power = 0;
    public static boolean resetCount = false;
    public static boolean reset = false;

    private final ElapsedTime runtime = new ElapsedTime();

    AnalogInput upstream;
    AnalogInput downstream;
    DcMotorEx motor1;

    int count;
    boolean ballInZone;
    boolean bridgedBoth;
    boolean seamAtUpstream;

    @Override
    public void init() {
        upstream = hardwareMap.analogInput.get(upstreamSensorName);
        downstream = hardwareMap.analogInput.get(downstreamSensorName);
        motor1 = hardwareMap.get(DcMotorEx.class, motor1Name);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        runtime.reset();
        double mmA = upstream.getVoltage() / 3.3 * maxRangeMm;
        double mmB = downstream.getVoltage() / 3.3 * maxRangeMm;

        if (Math.min(mmA, mmB) < emptyAboveMm) {
            if (!ballInZone) {
                ballInZone = true;
                count++;
                bridgedBoth = false;
                seamAtUpstream = false;
            }
            if (mmA < bothOnBallBelowMm && mmB < bothOnBallBelowMm) {
                bridgedBoth = true;
            }
            if (bridgedBoth && !seamAtUpstream && mmA - mmB > seamAtUpstreamDiffMm) {
                seamAtUpstream = true;
            }
            // The seam crossing from A to B flips the sign of A − B; a lone ball leaving keeps A high, so it never flips.
            if (seamAtUpstream && mmB - mmA > seamAtDownstreamDiffMm) {
                count++;
                seamAtUpstream = false;
            }
        } else {
            ballInZone = false;
            bridgedBoth = false;
            seamAtUpstream = false;
        }

        if (reset && count >= stopAtCount) {
            motor1Power = 0;
        }

        if (resetCount) {
            count = 0;
        }

        motor1.setPower(motor1Power);

        while (runtime.milliseconds() < 20) {

        }

        telemetry.addData("Count", count);
        telemetry.addData("Upstream (mm)", mmA);
        telemetry.addData("Downstream (mm)", mmB);
        telemetry.addData("A - B (mm)", mmA - mmB);
        telemetry.addData("Ball in zone", ballInZone);
        telemetry.addData("Bridged both", bridgedBoth);
        telemetry.addData("Seam at upstream", seamAtUpstream);
        telemetry.addData("motor current", motor1.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("loop time (ms)", runtime.milliseconds());
        telemetry.update();
    }
}
