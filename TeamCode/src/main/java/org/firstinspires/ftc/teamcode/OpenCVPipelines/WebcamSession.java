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
 */
public final class WebcamSession {
    private static final int WIDTH = 640;
    private static final int HEIGHT = 480;
    private static final int DASHBOARD_FPS = 30;

    private final OpenCvWebcam webcam;
    // Built on the camera thread (onOpened), read on the OpMode thread (update()); volatile for visibility.
    private volatile WebcamControls controls;

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
                FtcDashboard.getInstance().startCameraStream(webcam, DASHBOARD_FPS);
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

    public void close() {
        FtcDashboard.getInstance().stopCameraStream();
        webcam.stopStreaming();
        webcam.closeCameraDevice();
    }
}
