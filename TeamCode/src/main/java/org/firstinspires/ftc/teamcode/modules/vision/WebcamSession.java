package org.firstinspires.ftc.teamcode.modules.vision;

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
 * Opens a webcam, streams the pipeline to FtcDashboard, and pumps {@link WebcamControls} once open.
 */
public final class WebcamSession {
    private static final int WIDTH = 640;
    private static final int HEIGHT = 480;
    private static final int DASHBOARD_FPS = 30;

    private final OpenCvWebcam webcam;
    // Written on the camera thread (onOpened), read on the OpMode thread (update()).
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

    public void update() {
        WebcamControls c = controls;
        if (c != null) c.update();
    }

    /** Stream rendering runs inside each frame callback, so it counts toward getOverheadTimeMs(). */
    public void setCameraStreamEnabled(boolean enabled) {
        if (enabled == cameraStreamEnabled) return;
        cameraStreamEnabled = enabled;
        if (enabled) FtcDashboard.getInstance().startCameraStream(webcam, DASHBOARD_FPS);
        else FtcDashboard.getInstance().stopCameraStream();
    }

    public void close() {
        FtcDashboard.getInstance().stopCameraStream();
        webcam.stopStreaming();
        webcam.closeCameraDevice();
    }
}
