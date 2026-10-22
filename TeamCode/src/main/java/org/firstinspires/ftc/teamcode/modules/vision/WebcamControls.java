package org.firstinspires.ftc.teamcode.modules.vision;

import com.acmerobotics.dashboard.config.Config;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;
import java.util.function.IntPredicate;
import java.util.function.IntSupplier;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.openftc.easyopencv.OpenCvWebcam;

/**
 * Live, dashboard-tunable webcam exposure/gain/white balance. Robot-only: EOCV-Sim never opens an
 * OpenCvWebcam.
 */
@Config("Webcam Controls")
public class WebcamControls {

    public static boolean manual = true;
    public static int exposureMs = 8;
    // Raw device units, clamped to the camera's range.
    public static int gain = 0;
    public static int whiteBalanceK = 3250;

    private static final int UNSET = Integer.MIN_VALUE;

    private final ExposureControl exposure;
    private final WhiteBalanceControl whiteBalance;
    private final Knob exposureKnob;
    private final Knob gainKnob;
    private final Knob whiteBalanceKnob;

    private boolean lastManual;
    private boolean exposureModeRejected;
    private boolean whiteBalanceModeRejected;
    private String rejected;

    public WebcamControls(OpenCvWebcam webcam) {
        exposure = webcam.getExposureControl();
        whiteBalance = webcam.getWhiteBalanceControl();
        GainControl gainControl = webcam.getGainControl();
        exposureKnob = new Knob(" ms", () -> exposureMs,
                () -> (int) exposure.getMinExposure(TimeUnit.MILLISECONDS),
                () -> (int) exposure.getMaxExposure(TimeUnit.MILLISECONDS),
                ms -> exposure.setExposure(ms, TimeUnit.MILLISECONDS));
        gainKnob = new Knob("", () -> gain, gainControl::getMinGain, gainControl::getMaxGain, gainControl::setGain);
        whiteBalanceKnob = new Knob(" K", () -> whiteBalanceK,
                whiteBalance::getMinWhiteBalanceTemperature, whiteBalance::getMaxWhiteBalanceTemperature,
                whiteBalance::setWhiteBalanceTemperature);
        this.lastManual = !manual; // force a mode apply on the first update()
    }

    // Each write is a blocking USB transfer, so a value is sent once: a rejected one waits for a new value.
    public void update() {
        boolean modeChanged = manual != lastManual;
        lastManual = manual;
        boolean wrote = modeChanged;

        if (!manual) {
            if (modeChanged) setModes(ExposureControl.Mode.ContinuousAuto, WhiteBalanceControl.Mode.AUTO);
        } else {
            if (modeChanged) {
                setModes(ExposureControl.Mode.Manual, WhiteBalanceControl.Mode.MANUAL);
                // The camera resets values on a mode switch.
                exposureKnob.reset();
                gainKnob.reset();
                whiteBalanceKnob.reset();
            }
            if (!exposureModeRejected) wrote |= exposureKnob.update();
            wrote |= gainKnob.update();
            if (!whiteBalanceModeRejected) wrote |= whiteBalanceKnob.update();
        }
        if (wrote) rejected = describeRejected();
    }

    /** What the camera refused, e.g. "gain 100, white balance mode MANUAL"; null when it took everything. */
    public String rejected() {
        return rejected;
    }

    public void addTelemetry(Telemetry telemetry) {
        String mode = lastManual ? "MANUAL" : "AUTO";
        telemetry.addData("Exposure", exposureModeRejected ? mode + " mode REJECTED" : lastManual ? exposureKnob.status() : mode);
        telemetry.addData("Gain", lastManual ? gainKnob.status() : mode);
        telemetry.addData("White balance", whiteBalanceModeRejected ? mode + " mode REJECTED"
                : lastManual ? whiteBalanceKnob.status() : mode);
    }

    private void setModes(ExposureControl.Mode exposureMode, WhiteBalanceControl.Mode whiteBalanceMode) {
        exposureModeRejected = !exposure.setMode(exposureMode);
        whiteBalanceModeRejected = !whiteBalance.setMode(whiteBalanceMode);
    }

    private String describeRejected() {
        String mode = lastManual ? "MANUAL" : "AUTO";
        List<String> parts = new ArrayList<>();
        if (exposureModeRejected) parts.add("exposure mode " + mode);
        else if (lastManual && exposureKnob.rejected) parts.add("exposure " + exposureKnob.value());
        if (lastManual && gainKnob.rejected) parts.add("gain " + gainKnob.value());
        if (whiteBalanceModeRejected) parts.add("white balance mode " + mode);
        else if (lastManual && whiteBalanceKnob.rejected) parts.add("white balance " + whiteBalanceKnob.value());
        return parts.isEmpty() ? null : String.join(", ", parts);
    }

    private static final class Knob {
        private final String unit;
        private final IntSupplier wanted;
        private final IntSupplier min;
        private final IntSupplier max;
        private final IntPredicate set;
        private int last = UNSET;
        private int sent;
        private boolean rejected;

        Knob(String unit, IntSupplier wanted, IntSupplier min, IntSupplier max, IntPredicate set) {
            this.unit = unit;
            this.wanted = wanted;
            this.min = min;
            this.max = max;
            this.set = set;
        }

        void reset() {
            last = UNSET;
            rejected = false;
        }

        boolean update() {
            int want = wanted.getAsInt();
            if (want == last) return false;
            last = want;
            int lo = min.getAsInt();
            int hi = max.getAsInt();
            sent = hi > lo ? Math.max(lo, Math.min(hi, want)) : want;
            rejected = !set.test(sent);
            return true;
        }

        String value() {
            return sent + unit + (sent != last ? " (asked " + last + unit + ")" : "");
        }

        String status() {
            if (last == UNSET) return "not sent";
            return rejected ? value() + " REJECTED" : value();
        }
    }
}
