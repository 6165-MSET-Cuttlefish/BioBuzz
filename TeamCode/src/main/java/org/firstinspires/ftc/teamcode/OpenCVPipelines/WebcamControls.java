package org.firstinspires.ftc.teamcode.OpenCVPipelines;

import com.acmerobotics.dashboard.config.Config;

import java.util.concurrent.TimeUnit;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.openftc.easyopencv.OpenCvWebcam;

/**
 * Live, dashboard-tunable exposure / gain / white-balance for an EasyOpenCV webcam — the fix for the
 * washed-out / white look: drop {@link #exposureMs} until the yellow Pollen stops clipping, then
 * trim white balance and gain.
 *
 * <p>Construct it once the camera is open and streaming, then call {@link #update()} every loop. It
 * pushes a value to the camera only when its dashboard field actually changed (each control write is
 * a blocking USB transfer), and it caches a value only once the {@code set*} call reports success, so
 * a write that transiently fails right after a mode switch is retried on the next loop.
 *
 * <p>Robot-only: EOCV-Sim feeds frames straight into the pipeline and never opens an OpenCvWebcam, so
 * these controls do nothing there — sim-tune exposure in the camera driver, use this on the Hub.
 */
@Config
public class WebcamControls {

    // false → hand exposure/white-balance back to the camera's own auto algorithm.
    public static boolean manual = true;
    // Shutter time in milliseconds. Lower = darker; this is the primary knob for the white-out.
    public static int exposureMs = 8;
    // Sensor gain in raw device units (clamped to the camera's range). Keep low to avoid washout.
    public static int gain = 0;
    // White-balance color temperature in Kelvin (clamped to range). This is what the camera
    // assumes the room's light source is, so it can cancel that tint back out — set it LOWER than
    // the room's actual temperature and everything reads too warm/red (the camera under-corrects);
    // set it too high and the compensation over-corrects the other way (too cool/blue). 4300 was
    // tuned warm for one specific venue's fluorescent lighting and reads noticeably red-shifted
    // under cooler LED/daylight; 5500 (daylight-ish) is a more neutral default across venues. Live
    // on FtcDashboard — nudge it to match whatever room you're actually in, no redeploy needed.
    public static int whiteBalanceK = 5500;

    private final OpenCvWebcam webcam;

    private boolean lastManual;
    private int lastExposureMs   = Integer.MIN_VALUE;
    private int lastGain         = Integer.MIN_VALUE;
    private int lastWhiteBalance = Integer.MIN_VALUE;

    public WebcamControls(OpenCvWebcam webcam) {
        this.webcam = webcam;
        this.lastManual = !manual; // force a mode apply on the first update()
    }

    /** Call once per loop, after the camera is streaming. No-op for unchanged values. */
    public void update() {
        if (webcam == null) return;

        boolean modeChanged = manual != lastManual;
        lastManual = manual;

        if (!manual) {
            if (modeChanged) setModes(ExposureControl.Mode.ContinuousAuto, WhiteBalanceControl.Mode.AUTO);
            return;
        }

        if (modeChanged) {
            setModes(ExposureControl.Mode.Manual, WhiteBalanceControl.Mode.MANUAL);
            // The camera resets values on a mode switch — force a re-push of all three.
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
