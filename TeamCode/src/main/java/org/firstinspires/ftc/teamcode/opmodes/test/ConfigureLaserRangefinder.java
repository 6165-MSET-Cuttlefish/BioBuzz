package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.architecture.hardware.LaserRangefinder;

import java.util.Arrays;

/**
 * Writes persistent config to a Brushland Laser Rangefinder (a "Rev Color Sensor V3" named "Laser").
 * Config only takes effect after unplugging and replugging the sensor.
 * To recover from a bad digital/analog setting: init with the sensor unplugged, stop, then plug in.
 * https://docs.brushlandlabs.com/sensors/laser-rangefinder/getting-started
 */
@Autonomous(name = "Configure Laser Rangefinder", group = "Test")
public class ConfigureLaserRangefinder extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        LaserRangefinder lrf = new LaserRangefinder(
                hardwareMap.get(RevColorSensorV3.class, "Laser"));

        telemetry.addLine("=== Current sensor config ===");
        telemetry.addData("Pin0 mode",      lrf.getPin0Mode());
        telemetry.addData("Pin1 mode",      lrf.getPin1Mode());
        telemetry.addData("Distance mode",  lrf.getDistanceMode().name());
        telemetry.addData("Timing [budget, period]", Arrays.toString(lrf.getTiming()));
        telemetry.addData("ROI",            Arrays.toString(lrf.getROI()));
        telemetry.addData("Optical center", Arrays.toString(lrf.getOpticalCenter()));
        telemetry.update();

        waitForStart();

        lrf.setDistanceMode(LaserRangefinder.DistanceMode.LONG);
        lrf.setTiming(33, 0);   // 30 Hz, less noisy; period=0 → start next range immediately
        lrf.setROI(0, 15, 15, 0); // full 16×16 FOV (27°)

        telemetry.addLine("Configuration written. Unplug and replug the sensor.");
        telemetry.update();
        sleep(2000);
    }
}
