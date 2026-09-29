package org.firstinspires.ftc.teamcode.modules.vision;

import com.acmerobotics.dashboard.config.Config;

import java.util.concurrent.TimeUnit;

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
    private int lastExposureMs   = UNSET;
    private int lastGain         = UNSET;
    private int lastWhiteBalance = UNSET;
    private long exposureRetryAt, gainRetryAt, whiteBalanceRetryAt;

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
            lastExposureMs = lastGain = lastWhiteBalance = UNSET;
            exposureRetryAt = gainRetryAt = whiteBalanceRetryAt = now;
        }
        if (exposureMs != lastExposureMs && now - exposureRetryAt >= 0) {
            if (applyExposure()) lastExposureMs = exposureMs;
            else exposureRetryAt = now + RETRY_NANOS;
        }
        if (gain != lastGain && now - gainRetryAt >= 0) {
            if (applyGain()) lastGain = gain;
            else gainRetryAt = now + RETRY_NANOS;
        }
        if (whiteBalanceK != lastWhiteBalance && now - whiteBalanceRetryAt >= 0) {
            if (applyWhiteBalance()) lastWhiteBalance = whiteBalanceK;
            else whiteBalanceRetryAt = now + RETRY_NANOS;
        }
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
            long want = exposureMs;
            if (max > min) want = Math.max(min, Math.min(max, want));
            return exp.setExposure(want, TimeUnit.MILLISECONDS);
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
            int want = gain;
            if (max > min) want = Math.max(min, Math.min(max, want));
            return g.setGain(want);
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
            int want = whiteBalanceK;
            if (max > min) want = Math.max(min, Math.min(max, want));
            return wb.setWhiteBalanceTemperature(want);
        } catch (Exception ignored) {
            return false;
        }
    }
}
