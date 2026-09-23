package org.firstinspires.ftc.teamcode.eocvsim.homography;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Locks a chessboard homography from full-resolution image pixels to field inches (the convention
 * {@code BallDetectionPipeline} uses) and prints it for {@code BallVisionConstants.H_ARRAY}.
 */
public class HomographyCalculationPipeline extends OpenCvPipeline {

    // Hand-synced with BallVisionConstants: EOCV-Sim compiles this file in isolation and can't import it.
    private static final int   GRID_COLS          = 9;
    private static final int   GRID_ROWS          = 6;
    private static final int   EXPECTED_CORNERS   = GRID_COLS * GRID_ROWS;
    private static final float SQUARE_SIZE_INCHES = 1.0f; // TODO: verify against your physical board
    private static final float OUTPUT_SCALE_PX    = 50.0f;
    private static final float MARGIN_PX          = 250.0f;
    private static final int   FRAMES_TO_CONFIRM  = 5;

    // SB matches corners independently, so foreshortened far squares don't fail the whole board the
    // way classic findChessboardCorners does. It needs a white border about one square wide.
    private static final int CHESSBOARD_FLAGS =
            Calib3d.CALIB_CB_NORMALIZE_IMAGE
                    | Calib3d.CALIB_CB_EXHAUSTIVE
                    | Calib3d.CALIB_CB_ACCURACY;

    // px. Loose because steep or distant boards localize corners more noisily than a close-up one.
    private static final double RANSAC_REPROJ_THRESHOLD_PX = 8.0;

    private static final int OUTPUT_WIDTH_PX  =
            (int) (2 * MARGIN_PX + (GRID_COLS - 1) * SQUARE_SIZE_INCHES * OUTPUT_SCALE_PX);
    private static final int OUTPUT_HEIGHT_PX =
            (int) (2 * MARGIN_PX + (GRID_ROWS - 1) * SQUARE_SIZE_INCHES * OUTPUT_SCALE_PX);
    private static final Size WARP_SIZE = new Size(OUTPUT_WIDTH_PX, OUTPUT_HEIGHT_PX);

    // Inches to preview-canvas pixels, for the on-screen warp only; the exported homography stays in inches.
    private static final Mat PREVIEW_SCALE = buildPreviewScale();

    private static Mat buildPreviewScale() {
        Mat m = Mat.eye(3, 3, CvType.CV_64F);
        m.put(0, 0, (double) OUTPUT_SCALE_PX);
        m.put(1, 1, (double) OUTPUT_SCALE_PX);
        m.put(0, 2, (double) MARGIN_PX);
        m.put(1, 2, (double) MARGIN_PX);
        return m;
    }

    private final AtomicBoolean        homographyLocked        = new AtomicBoolean(false);
    private final AtomicReference<Mat> lockedHomography        = new AtomicReference<>(null);
    private final AtomicReference<Mat> lockedPreviewHomography = new AtomicReference<>(null);
    private final AtomicInteger        confirmCount            = new AtomicInteger(0);
    private final AtomicReference<Mat> candidateHomography     = new AtomicReference<>(null);

    private final AtomicReference<Mat> pendingFrame     = new AtomicReference<>(null);
    private final AtomicBoolean        detectionRunning  = new AtomicBoolean(false);

    private volatile String statusLine    = "Searching for chessboard...";
    private volatile String homographyStr = null;

    private final Telemetry    telemetry;
    private final MatOfPoint2f dstCorners;
    private final Mat          warped = new Mat();
    // Reused: EasyOpenCV copies the returned Mat's pixels but never releases it, so a per-frame new Mat leaks.
    private final Mat          output = new Mat();

    private final Thread detectionThread;

    public HomographyCalculationPipeline(Telemetry telemetry) {
        this.telemetry  = telemetry;
        this.dstCorners = buildCalibrationDstCorners();

        detectionThread = new Thread(this::detectionLoop, "HomographyDetection");
        detectionThread.setDaemon(true);
        detectionThread.start();
    }

    @Override
    public Mat processFrame(Mat input) {

        if (homographyLocked.get()) {
            Imgproc.warpPerspective(input, warped, lockedPreviewHomography.get(), WARP_SIZE);

            Imgproc.resize(warped, output,
                    new Size(input.width(), input.height()),
                    0, 0, Imgproc.INTER_LINEAR);

            telemetry.addLine("HOMOGRAPHY LOCKED");
            telemetry.addLine(homographyStr);
            telemetry.update();
            return output;
        }

        if (!detectionRunning.get() && pendingFrame.get() == null) {
            pendingFrame.set(input.clone());
        }

        telemetry.addLine(statusLine);
        telemetry.addData("Confirmations", confirmCount.get() + " / " + FRAMES_TO_CONFIRM);
        telemetry.update();
        return input;
    }

    private void detectionLoop() {
        final Mat          gray    = new Mat();
        final MatOfPoint2f corners = new MatOfPoint2f();

        while (!Thread.currentThread().isInterrupted() && !homographyLocked.get()) {
            Mat frame = pendingFrame.getAndSet(null);
            if (frame == null) {
                try { Thread.sleep(5); } catch (InterruptedException e) { break; }
                continue;
            }

            detectionRunning.set(true);
            try {
                processDetection(frame, gray, corners);
            } finally {
                frame.release();
                detectionRunning.set(false);
            }
        }

        gray.release();
        corners.release();
    }

    private void processDetection(Mat frame, Mat gray, MatOfPoint2f corners) {

        Imgproc.cvtColor(frame, gray, Imgproc.COLOR_RGB2GRAY);

        boolean found = Calib3d.findChessboardCornersSB(
                gray,
                new Size(GRID_COLS, GRID_ROWS),
                corners,
                CHESSBOARD_FLAGS
        );

        if (!found || corners.rows() != EXPECTED_CORNERS) {
            confirmCount.set(0);
            statusLine = "Searching... (board not visible)";
            return;
        }

        Mat h = Calib3d.findHomography(corners, dstCorners, Calib3d.RANSAC, RANSAC_REPROJ_THRESHOLD_PX);
        if (h == null || h.empty()) {
            statusLine = "Homography failed (RANSAC) — retrying...";
            return;
        }

        // Superseded candidates are deliberately not released: another thread may be reading one.
        candidateHomography.set(h);
        statusLine = "Board found — confirming...";

        if (confirmCount.incrementAndGet() >= FRAMES_TO_CONFIRM) {

            Mat previewHomography = new Mat();
            Core.gemm(PREVIEW_SCALE, h, 1.0, new Mat(), 0.0, previewHomography);

            lockedHomography.set(h);
            lockedPreviewHomography.set(previewHomography);
            homographyStr = buildHomographyString(h);
            homographyLocked.set(true); // must be last
        }
    }

    private static MatOfPoint2f buildCalibrationDstCorners() {
        List<Point> points = new ArrayList<>(GRID_COLS * GRID_ROWS);
        for (int row = 0; row < GRID_ROWS; row++)
            for (int col = 0; col < GRID_COLS; col++)
                points.add(new Point(col * SQUARE_SIZE_INCHES, row * SQUARE_SIZE_INCHES));
        MatOfPoint2f dst = new MatOfPoint2f();
        dst.fromList(points);
        return dst;
    }

    private static String buildHomographyString(Mat h) {
        StringBuilder sb = new StringBuilder("double[][] H_ARRAY = {\n");
        for (int r = 0; r < 3; r++) {
            sb.append("    { ");
            for (int c = 0; c < 3; c++) {
                sb.append(String.format("%.10e", h.get(r, c)[0]));
                if (c < 2) sb.append(", ");
            }
            sb.append(r < 2 ? " },\n" : " }\n");
        }
        return sb.append("};").toString();
    }

    public String getHomographyAsString() {
        if (homographyStr != null) return homographyStr;
        Mat h = candidateHomography.get();
        return (h != null && !h.empty()) ? buildHomographyString(h) : "Homography not available";
    }

    public Mat getHomography() {
        Mat locked = lockedHomography.get();
        return locked != null ? locked : candidateHomography.get();
    }

    /** Call from your op-mode's stop() to shut down the background thread. */
    public void stop() {
        detectionThread.interrupt();
    }
}