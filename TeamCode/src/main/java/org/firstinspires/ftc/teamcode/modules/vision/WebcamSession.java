package org.firstinspires.ftc.teamcode.modules.vision;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamServer;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;
import org.openftc.easyopencv.TimestampedOpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

/**
 * Opens a webcam, streams the pipeline to FtcDashboard and the DS preview, and pumps
 * {@link WebcamControls} once open. {@link #update()} throws if the camera failed to open.
 */
public final class WebcamSession {
    private static final int WIDTH = 640;
    private static final int HEIGHT = 480;
    private static final int DASHBOARD_FPS = 30;
    private static final int NO_ERROR = 0;

    private final String webcamName;
    private final OpenCvWebcam webcam;
    private final SharedFrameSource frames;
    // Written on the camera-open thread, read on the OpMode thread.
    private volatile WebcamControls controls;
    private volatile int openError = NO_ERROR;
    private volatile boolean cameraStreamEnabled = true;
    private volatile boolean closed;
    private volatile RuntimeException cameraThreadFailure;

    public WebcamSession(HardwareMap hardwareMap, String webcamName, OpenCvPipeline pipeline) {
        this.webcamName = webcamName;
        WebcamName name = hardwareMap.get(WebcamName.class, webcamName);
        int viewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(name, viewId);
        webcam.setPipeline(new FailureCapture(pipeline));
        frames = new SharedFrameSource(webcam);
        // EasyOpenCV registered the webcam itself as the DS preview source in its constructor.
        CameraStreamServer.getInstance().setSource(frames);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override public void onOpened() {
                // close() ran before the open finished and found nothing to close, so the camera would stay held.
                if (closed) {
                    webcam.closeCameraDevice();
                    return;
                }
                try {
                    // OV9782 only offers 640x480 in MJPEG, not the default uncompressed YUY2.
                    webcam.startStreaming(WIDTH, HEIGHT, OpenCvCameraRotation.UPRIGHT,
                            OpenCvWebcam.StreamFormat.MJPEG);
                    if (cameraStreamEnabled) FtcDashboard.getInstance().startCameraStream(frames, DASHBOARD_FPS);
                    controls = new WebcamControls(webcam);
                } catch (RuntimeException e) {
                    fail(e);
                }
            }

            @Override public void onError(int errorCode) {
                openError = errorCode;
            }
        });
    }

    public OpenCvWebcam webcam() { return webcam; }

    /** Call every loop from the OpMode thread; throws once the camera has reported an open failure. */
    public void update() {
        if (closed) throw new IllegalStateException(webcamName + " was closed; a WebcamSession can't reopen");
        int error = openError;
        if (error != NO_ERROR) {
            throw new IllegalStateException(webcamName + " failed to open (EasyOpenCV error "
                    + error + " " + describe(error) + ")");
        }
        RuntimeException failure = cameraThreadFailure;
        if (failure != null) throw new IllegalStateException(webcamName + " camera thread failed: " + failure, failure);
        WebcamControls c = controls;
        if (c != null) c.update();
    }

    /** Stream rendering runs inside each frame callback, so it counts toward getOverheadTimeMs(). */
    public void setCameraStreamEnabled(boolean enabled) {
        if (enabled == cameraStreamEnabled) return;
        cameraStreamEnabled = enabled;
        if (enabled) FtcDashboard.getInstance().startCameraStream(frames, DASHBOARD_FPS);
        else FtcDashboard.getInstance().stopCameraStream();
    }

    /** Blocks until the camera has stopped streaming, so the pipeline's native resources are free to release. */
    public void close() {
        closed = true;
        try {
            // Stops streaming only if the camera opened; stopStreaming() on an unopened camera throws.
            webcam.closeCameraDevice();
        } finally {
            FtcDashboard.getInstance().stopCameraStream();
        }
    }

    private void fail(RuntimeException e) {
        if (cameraThreadFailure == null) cameraThreadFailure = e;
    }

    private static String describe(int error) {
        switch (error) {
            case OpenCvCamera.CAMERA_OPEN_ERROR_FAILURE_TO_OPEN_CAMERA_DEVICE:
                return "FAILURE_TO_OPEN_CAMERA_DEVICE: check the USB cable, the config name and that no other OpMode holds it";
            case OpenCvCamera.CAMERA_OPEN_ERROR_POSTMORTEM_OPMODE:
                return "POSTMORTEM_OPMODE";
            default:
                return "unknown";
        }
    }

    private final class FailureCapture extends TimestampedOpenCvPipeline {
        private final OpenCvPipeline pipeline;

        FailureCapture(OpenCvPipeline pipeline) {
            this.pipeline = pipeline;
        }

        @Override
        public void init(Mat firstFrame) {
            try {
                pipeline.init(firstFrame);
            } catch (RuntimeException e) {
                fail(e);
            }
        }

        @Override
        public Mat processFrame(Mat input, long captureTimeNanos) {
            if (cameraThreadFailure != null) return input;
            try {
                return pipeline instanceof TimestampedOpenCvPipeline
                        ? ((TimestampedOpenCvPipeline) pipeline).processFrame(input, captureTimeNanos)
                        : pipeline.processFrame(input);
            } catch (RuntimeException e) {
                fail(e);
                return input;
            }
        }

        @Override
        public void onViewportTapped() {
            pipeline.onViewportTapped();
        }

        @Override
        public Object getUserContextForDrawHook() {
            return pipeline.getUserContextForDrawHook();
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx,
                                float scaleCanvasDensity, Object userContext) {
            pipeline.onDrawFrame(canvas, onscreenWidth, onscreenHeight, scaleBmpPxToCanvasPx, scaleCanvasDensity, userContext);
        }
    }

    /**
     * EasyOpenCV holds one pending frame request, so the DS preview and the dashboard stream would
     * each overwrite the other's and leave it waiting forever; this fans one request out to both.
     */
    private static final class SharedFrameSource implements CameraStreamSource {
        private final CameraStreamSource camera;
        private final List<Continuation<? extends Consumer<Bitmap>>> pending = new ArrayList<>();

        SharedFrameSource(CameraStreamSource camera) {
            this.camera = camera;
        }

        @Override
        public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
            boolean request;
            synchronized (pending) {
                request = pending.isEmpty();
                pending.add(continuation);
            }
            if (request) camera.getFrameBitmap(Continuation.createTrivial(new Consumer<Bitmap>() {
                @Override public void accept(Bitmap bitmap) {
                    fanOut(bitmap);
                }
            }));
        }

        private void fanOut(final Bitmap bitmap) {
            List<Continuation<? extends Consumer<Bitmap>>> waiting;
            synchronized (pending) {
                waiting = new ArrayList<>(pending);
                pending.clear();
            }
            // Here, not dispatch(): EasyOpenCV recycles the bitmap as soon as this returns.
            for (Continuation<? extends Consumer<Bitmap>> c : waiting) {
                c.dispatchHere(consumer -> consumer.accept(bitmap));
            }
        }
    }
}
