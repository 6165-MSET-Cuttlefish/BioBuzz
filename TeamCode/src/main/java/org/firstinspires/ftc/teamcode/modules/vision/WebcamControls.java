package org.firstinspires.ftc.teamcode.modules.vision;

import com.acmerobotics.dashboard.config.Config;

import java.util.concurrent.TimeUnit;
import java.util.function.BooleanSupplier;
import java.util.function.IntSupplier;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.openftc.easyopencv.OpenCvWebcam;

/**
 * Live, dashboard-tunable webcam exposure/gain/white balance. Robot-only: EOCV-Sim never opens an
 * OpenCvWebcam.
 */
@Config
public class WebcamControls {

    public static boolean manual = true;
    public static int exposureMs = 8;
    // Raw device units, clamped to the camera's range.
    public static int gain = 0;
    public static int whiteBalanceK = 3250;

    private static final long RETRY_NANOS = TimeUnit.SECONDS.toNanos(1);
    private static final int UNSET = Integer.MIN_VALUE;

    private final OpenCvWebcam webcam;

    private boolean lastManual;
    private final Knob exposureKnob = new Knob(() -> exposureMs, this::applyExposure);
    private final Knob gainKnob = new Knob(() -> gain, this::applyGain);
    private final Knob whiteBalanceKnob = new Knob(() -> whiteBalanceK, this::applyWhiteBalance);

    public WebcamControls(OpenCvWebcam webcam) {
        this.webcam = webcam;
        this.lastManual = !manual; // force a mode apply on the first update()
    }

    // Each write is a blocking USB transfer: push only changed values, and retry a rejected one at most once per RETRY_NANOS.
    public void update() {
        boolean modeChanged = manual != lastManual;
        lastManual = manual;

        if (!manual) {
            if (modeChanged) setModes(ExposureControl.Mode.ContinuousAuto, WhiteBalanceControl.Mode.AUTO);
            return;
        }

        long now = System.nanoTime();
        if (modeChanged) {
            setModes(ExposureControl.Mode.Manual, WhiteBalanceControl.Mode.MANUAL);
            // The camera resets values on a mode switch.
            exposureKnob.reset(now);
            gainKnob.reset(now);
            whiteBalanceKnob.reset(now);
        }
        exposureKnob.update(now);
        gainKnob.update(now);
        whiteBalanceKnob.update(now);
    }

    private static final class Knob {
        private final IntSupplier value;
        private final BooleanSupplier apply;
        private int last = UNSET;
        private long retryAt;

        Knob(IntSupplier value, BooleanSupplier apply) {
            this.value = value;
            this.apply = apply;
        }

        void reset(long now) {
            last = UNSET;
            retryAt = now;
        }

        void update(long now) {
            if (value.getAsInt() != last && now - retryAt >= 0) {
                if (apply.getAsBoolean()) last = value.getAsInt();
                else retryAt = now + RETRY_NANOS;
            }
        }
    }

    private static long clamp(long want, long min, long max) {
        return max > min ? Math.max(min, Math.min(max, want)) : want;
    }

    private void setModes(ExposureControl.Mode expMode, WhiteBalanceControl.Mode wbMode) {
        try {
            ExposureControl exp = webcam.getExposureControl();
            if (exp != null) exp.setMode(expMode);
        } catch (Exception ignored) {}
        try {
            WhiteBalanceControl wb = webcam.getWhiteBalanceControl();
            if (wb != null) wb.setMode(wbMode);
        } catch (Exception ignored) {}
    }

    private boolean applyExposure() {
        try {
            ExposureControl exp = webcam.getExposureControl();
            if (exp == null) return false;
            long min = exp.getMinExposure(TimeUnit.MILLISECONDS);
            long max = exp.getMaxExposure(TimeUnit.MILLISECONDS);
            return exp.setExposure(clamp(exposureMs, min, max), TimeUnit.MILLISECONDS);
        } catch (Exception ignored) {
            return false;
        }
    }

    private boolean applyGain() {
        try {
            GainControl g = webcam.getGainControl();
            if (g == null) return false;
            int min = g.getMinGain();
            int max = g.getMaxGain();
            return g.setGain((int) clamp(gain, min, max));
        } catch (Exception ignored) {
            return false;
        }
    }

    private boolean applyWhiteBalance() {
        try {
            WhiteBalanceControl wb = webcam.getWhiteBalanceControl();
            if (wb == null) return false;
            int min = wb.getMinWhiteBalanceTemperature();
            int max = wb.getMaxWhiteBalanceTemperature();
            return wb.setWhiteBalanceTemperature((int) clamp(whiteBalanceK, min, max));
        } catch (Exception ignored) {
            return false;
        }
    }
}
