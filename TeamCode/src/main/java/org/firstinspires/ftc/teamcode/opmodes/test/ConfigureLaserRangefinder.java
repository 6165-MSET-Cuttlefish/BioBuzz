package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Configure Laser Rangefinder", group = "Test")
@Config
public class ConfigureLaserRangefinder extends LinearOpMode {
    public static int analogLowMm = 0;
    public static int analogHighMm = 1300;
    public static LaserRangefinder.DistanceMode distanceMode = LaserRangefinder.DistanceMode.SHORT;
    public static int timingBudgetMs = 20;
    public static int measurementPeriodMs = 0;
    public static int roiSize = 4;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        LaserRangefinder lrf = new LaserRangefinder(hardwareMap.get(RevColorSensorV3.class, "Laser"));
        showConfig(lrf);
        waitForStart();
        if (!lrf.isResponding()) {
            throw new IllegalStateException("Laser rangefinder not answering on I2C: plug it into I2C bus 1 and revert it from analog first");
        }

        // Before the pins go analog: after that the sensor no longer answers on I2C.
        lrf.setDistanceMode(distanceMode);
        lrf.setTiming(timingBudgetMs, measurementPeriodMs);
        int[] center = lrf.getOpticalCenter();
        int topLeftX = center[0] - roiSize / 2;
        int topLeftY = center[1] + roiSize / 2 - 1;
        lrf.setROI(topLeftX, topLeftY, topLeftX + roiSize - 1, topLeftY - roiSize + 1);

        // Both pins at once: one non-I2C pin with the other left on I2C reverts the sensor to I2C.
        lrf.setPin0Analog(analogLowMm, analogHighMm);
        lrf.setPin1Analog(analogLowMm, analogHighMm);

        telemetry.addLine("Wrote analog config; power-cycle the sensor and move it to an analog port");
        showConfig(lrf);
        while (opModeIsActive()) idle();
    }

    private void showConfig(LaserRangefinder lrf) {
        telemetry.addData("Answering on I2C", lrf.isResponding());
        telemetry.addData("Pin0", lrf.getPin0Mode());
        telemetry.addData("Pin1", lrf.getPin1Mode());
        telemetry.addData("Distance Mode (1 SHORT, 2 MEDIUM, 3 LONG)", lrf.getDistanceModeRaw());
        telemetry.addData("Timing [Budget, Period]", java.util.Arrays.toString(lrf.getTiming()));
        telemetry.addData("ROI", java.util.Arrays.toString(lrf.getROI()));
        telemetry.addData("Optical Center", java.util.Arrays.toString(lrf.getOpticalCenter()));
        telemetry.update();
    }
}
