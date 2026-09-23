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
    // BallVisionConstants' HSV bands were tuned at this value.
    public static int whiteBalanceK = 3250;

    private final OpenCvWebcam webcam;

    private boolean lastManual;
    private int lastExposureMs   = Integer.MIN_VALUE;
    private int lastGain         = Integer.MIN_VALUE;
    private int lastWhiteBalance = Integer.MIN_VALUE;

    public WebcamControls(OpenCvWebcam webcam) {
        this.webcam = webcam;
        this.lastManual = !manual; // force a mode apply on the first update()
    }

    // Pushes only changed values (each write is a blocking USB transfer); caches a value only once
    // its set call succeeds, so a write that fails right after a mode switch retries next loop.
    public void update() {
        boolean modeChanged = manual != lastManual;
        lastManual = manual;

        if (!manual) {
            if (modeChanged) setModes(ExposureControl.Mode.ContinuousAuto, WhiteBalanceControl.Mode.AUTO);
            return;
        }

        if (modeChanged) {
            setModes(ExposureControl.Mode.Manual, WhiteBalanceControl.Mode.MANUAL);
            // The camera resets values on a mode switch.
            lastExposureMs = lastGain = lastWhiteBalance = Integer.MIN_VALUE;
        }
        applyExposure();
        applyGain();
        applyWhiteBalance();
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

    private void applyExposure() {
        if (exposureMs == lastExposureMs) return;
        try {
            ExposureControl exp = webcam.getExposureControl();
            if (exp == null) return;
            long min = exp.getMinExposure(TimeUnit.MILLISECONDS);
            long max = exp.getMaxExposure(TimeUnit.MILLISECONDS);
            long want = exposureMs;
            if (max > min) want = Math.max(min, Math.min(max, want));
            if (exp.setExposure(want, TimeUnit.MILLISECONDS)) lastExposureMs = exposureMs;
        } catch (Exception ignored) {}
    }

    private void applyGain() {
        if (gain == lastGain) return;
        try {
            GainControl g = webcam.getGainControl();
            if (g == null) return;
            int min = g.getMinGain();
            int max = g.getMaxGain();
            int want = gain;
            if (max > min) want = Math.max(min, Math.min(max, want));
            if (g.setGain(want)) lastGain = gain;
        } catch (Exception ignored) {}
    }

    private void applyWhiteBalance() {
        if (whiteBalanceK == lastWhiteBalance) return;
        try {
            WhiteBalanceControl wb = webcam.getWhiteBalanceControl();
            if (wb == null) return;
            int min = wb.getMinWhiteBalanceTemperature();
            int max = wb.getMaxWhiteBalanceTemperature();
            int want = whiteBalanceK;
            if (max > min) want = Math.max(min, Math.min(max, want));
            if (wb.setWhiteBalanceTemperature(want)) lastWhiteBalance = whiteBalanceK;
        } catch (Exception ignored) {}
    }
}
