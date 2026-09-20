package org.firstinspires.ftc.teamcode.OpenCVPipelines;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

/**
 * Opens a Control Hub webcam (MJPEG 640x480), streams the pipeline output to FtcDashboard, and
 * builds live {@link WebcamControls} once the camera is streaming. Shared by the camera test
 * OpModes so the open/cleanup boilerplate lives in one place.
 *
 * <p>The dashboard stream can be toggled off with {@link #setCameraStreamEnabled}. EasyOpenCV's own
 * {@code webcam().getPipelineTimeMs()} / {@code getOverheadTimeMs()} / {@code getTotalFrameTimeMs()}
 * already separate time spent inside the pipeline itself from everything else per frame; a dashboard
 * (or DS) camera stream renders its next bitmap synchronously inside that same per-frame call
 * whenever one is due, so it lands in {@code getOverheadTimeMs()}, not off on some other thread where
 * it wouldn't show up at all. Compare {@code getOverheadTimeMs()} with the stream on vs. off to see
 * how much of it is specifically the dashboard stream.
 */
public final class WebcamSession {
    private static final int WIDTH = 640;
    private static final int HEIGHT = 480;
    private static final int DASHBOARD_FPS = 30;

    private final OpenCvWebcam webcam;
    // Built on the camera thread (onOpened), read on the OpMode thread (update()); volatile for visibility.
    private volatile WebcamControls controls;
    private volatile boolean cameraStreamEnabled = true;

    public WebcamSession(HardwareMap hardwareMap, Telemetry telemetry,
                         String webcamName, OpenCvPipeline pipeline) {
        WebcamName name = hardwareMap.get(WebcamName.class, webcamName);
        int viewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(name, viewId);
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override public void onOpened() {
                // OV9782 only offers 640x480 in MJPEG, not the default uncompressed YUY2.
                webcam.startStreaming(WIDTH, HEIGHT, OpenCvCameraRotation.UPRIGHT,
                        OpenCvWebcam.StreamFormat.MJPEG);
                if (cameraStreamEnabled) FtcDashboard.getInstance().startCameraStream(webcam, DASHBOARD_FPS);
                controls = new WebcamControls(webcam);
            }

            @Override public void onError(int errorCode) {
                telemetry.addData("Camera open error", errorCode);
                telemetry.update();
            }
        });
    }

    public OpenCvWebcam webcam() { return webcam; }

    /** Pump the live controls; no-op until the camera has finished opening. */
    public void update() {
        WebcamControls c = controls;
        if (c != null) c.update();
    }

    /** No-op if already in the requested state, so callers can call this unconditionally every loop. */
    public void setCameraStreamEnabled(boolean enabled) {
        if (enabled == cameraStreamEnabled) return;
        cameraStreamEnabled = enabled;
        if (enabled) FtcDashboard.getInstance().startCameraStream(webcam, DASHBOARD_FPS);
        else FtcDashboard.getInstance().stopCameraStream();
    }

    public boolean isCameraStreamEnabled() { return cameraStreamEnabled; }

    public void close() {
        FtcDashboard.getInstance().stopCameraStream();
        webcam.stopStreaming();
        webcam.closeCameraDevice();
    }
}
