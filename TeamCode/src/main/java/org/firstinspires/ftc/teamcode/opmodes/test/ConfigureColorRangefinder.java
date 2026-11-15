package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.vendor.brushland.ColorRangefinder;

@TeleOp(name = "Configure Color Rangefinder", group = "Test")
@Config("Configure Color Rangefinder")
public class ConfigureColorRangefinder extends LinearOpMode {
    public enum Pin0 { HUE, DISTANCE, NECTAR }

    public static String deviceName = "Laser";
    public static Pin0 pin0 = Pin0.DISTANCE;
    public static int presenceMm = 30;
    public static double redHueLowDeg = 10;
    public static double redHueHighDeg = 50;
    public static double blueHueLowDeg = 170;
    public static double blueHueHighDeg = 210;
    public static int ledBrightness = -1;

    private static final int MAX_RANGE_MM = 100;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        ColorRangefinder crf = new ColorRangefinder(hardwareMap.get(RevColorSensorV3.class, deviceName));
        while (opModeInInit()) {
            telemetry.addData("Answering on I2C", isResponding(crf));
            telemetry.addData("Distance over I2C (mm)", crf.readDistance());
            telemetry.addData("START writes pin 0", pin0Plan());
            telemetry.addData("START writes pin 1", "digital, true when something is within " + presenceMm + " mm");
            telemetry.addData("START writes LED brightness", ledBrightness < 0 ? "no, left as it is" : ledBrightness);
            telemetry.addLine("A sensor with a side switch: keep it toward the connector while configuring");
            telemetry.update();
            sleep(100);
        }
        if (isStopRequested()) return;

        checkSettings();
        if (!isResponding(crf)) {
            throw new IllegalStateException("Color rangefinder '" + deviceName + "' not answering on I2C: plug it into "
                    + "I2C bus 1, and if it was configured before, INIT and stop this OpMode with it unplugged, then plug it in");
        }

        if (ledBrightness >= 0) crf.setLedBrightness(ledBrightness);
        // Both pins in one run: one non-I2C pin with the other left on I2C reverts the sensor to I2C.
        switch (pin0) {
            case HUE:
                crf.setPin0Analog(ColorRangefinder.AnalogMode.HSV);
                break;
            case DISTANCE:
                crf.setPin0Analog(ColorRangefinder.AnalogMode.DISTANCE);
                break;
            case NECTAR:
                addPin0HueRange(crf, redHueLowDeg, redHueHighDeg);
                addPin0HueRange(crf, blueHueLowDeg, blueHueHighDeg);
                crf.setPin0DigitalMaxDistance(ColorRangefinder.DigitalMode.HSV, presenceMm);
                break;
        }
        crf.setPin1Digital(ColorRangefinder.DigitalMode.DISTANCE, 0, presenceMm);

        telemetry.addData("Wrote pin 0", pin0Plan());
        telemetry.addData("Wrote pin 1", "digital, true when something is within " + presenceMm + " mm");
        telemetry.addLine("The LED should blink twice. Stop this OpMode and unplug the sensor.");
        if (pin0 == Pin0.NECTAR) {
            telemetry.addLine("Both pins are digital: plug it into a digital port and add a Digital Device per pin to the config");
        } else {
            telemetry.addLine("Plug it into Control Hub analog 2-3 (colorRangefinder is analog 2) and flip any side switch away "
                    + "from the connector. Pin 1 reaches a digital port only through a splitter cable.");
            telemetry.addLine(pin0 == Pin0.HUE
                    ? "Then read Hue (deg) in Transfer Thread Count Test"
                    : "Then read it in Laser Rangefinder Test with a sensor name set to colorRangefinder and maxRangeMm 100");
        }
        telemetry.update();
        while (opModeIsActive()) idle();
    }

    private static String pin0Plan() {
        switch (pin0) {
            case HUE:
                return "analog hue, deg = volts / 3.3 * 360";
            case DISTANCE:
                return "analog distance, mm = volts / 3.3 * " + MAX_RANGE_MM;
            default:
                return "digital, true for hue " + redHueLowDeg + "-" + redHueHighDeg + " or " + blueHueLowDeg + "-"
                        + blueHueHighDeg + " deg within " + presenceMm + " mm";
        }
    }

    private static void checkSettings() {
        if (presenceMm < 1 || presenceMm > MAX_RANGE_MM) {
            throw new IllegalArgumentException("presenceMm " + presenceMm + " must be 1 to " + MAX_RANGE_MM);
        }
        if (ledBrightness > 255) throw new IllegalArgumentException("ledBrightness " + ledBrightness + " must be -1 to 255");
        if (pin0 != Pin0.NECTAR) return;
        for (double deg : new double[] {redHueLowDeg, redHueHighDeg, blueHueLowDeg, blueHueHighDeg}) {
            if (deg < 0 || deg > 360) throw new IllegalArgumentException("hue " + deg + " must be 0 to 360 deg");
        }
        if (isEmpty(redHueLowDeg, redHueHighDeg) || isEmpty(blueHueLowDeg, blueHueHighDeg)) {
            throw new IllegalArgumentException("a hue range's low and high must differ, and 360 to 0 holds nothing");
        }
    }

    private static boolean isEmpty(double lowDeg, double highDeg) {
        return lowDeg == highDeg || (lowDeg >= 360 && highDeg <= 0);
    }

    private static void addPin0HueRange(ColorRangefinder crf, double lowDeg, double highDeg) {
        if (lowDeg < highDeg) {
            addPin0Hue(crf, lowDeg, highDeg);
            return;
        }
        // The helper writes equal bounds raw, as a distance requirement, so an empty half is skipped.
        if (lowDeg < 360) addPin0Hue(crf, lowDeg, 360);
        if (highDeg > 0) addPin0Hue(crf, 0, highDeg);
    }

    private static void addPin0Hue(ColorRangefinder crf, double lowDeg, double highDeg) {
        crf.setPin0Digital(ColorRangefinder.DigitalMode.HSV, lowDeg / 360.0 * 255, highDeg / 360.0 * 255);
    }

    // The helper's distance thresholds divide by the first two calibration values; a silent sensor reads zeros.
    private static boolean isResponding(ColorRangefinder crf) {
        float[] calibration = crf.getCalibration();
        return usable(calibration[0]) && usable(calibration[1]);
    }

    private static boolean usable(float v) {
        return v != 0 && Math.abs(v) < Float.POSITIVE_INFINITY;
    }
}
