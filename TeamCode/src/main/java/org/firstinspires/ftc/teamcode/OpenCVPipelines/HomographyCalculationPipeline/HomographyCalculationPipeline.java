package org.firstinspires.ftc.teamcode.OpenCVPipelines.HomographyCalculationPipeline;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Size;
import org.opencv.core.TermCriteria;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;

public class HomographyCalculationPipeline extends OpenCvPipeline {

    // Chessboard settings
    private static final int   GRID_COLS          = 9;
    private static final int   GRID_ROWS          = 6;
    private static final int   EXPECTED_CORNERS   = GRID_COLS * GRID_ROWS;
    private static final float SQUARE_SIZE_INCHES = 1.0f;
    private static final float OUTPUT_SCALE_PX    = 50.0f;
    private static final float MARGIN_PX          = 250.0f;
    private static final int   FRAMES_TO_CONFIRM  = 5;

    // Detection sensitivity flags for findChessboardCorners. EXHAUSTIVE makes
    // the search much more thorough (slower per-frame, but detection already
    // runs on a background thread) — this is what mainly helps at distance /
    // steep angle, where squares are small and foreshortened and the default
    // search gives up on them too early. FILTER_QUADS rejects bad quad
    // candidates before the grid-matching step, which helps signal-to-noise
    // when squares are tiny.
    private static final int CHESSBOARD_FLAGS =
            Calib3d.CALIB_CB_ADAPTIVE_THRESH
                    | Calib3d.CALIB_CB_NORMALIZE_IMAGE
                    | Calib3d.CALIB_CB_FILTER_QUADS
                    | Calib3d.CALIB_CB_EXHAUSTIVE;

    // cornerSubPix search window (half-size, in pixels — actual window is
    // (2*N+1) x (2*N+1)). At distance, squares are small in pixel terms, so a
    // window that's too large can wander onto a neighboring corner. Shrink
    // this if subpixel refinement looks unstable on far-away boards; grow it
    // if corners are noisy up close. Must stay smaller than half the
    // smallest expected square size in pixels.
    private static final int SUBPIX_WINDOW_PX = 4;

    // RANSAC reprojection error threshold (px) for findHomography. At steep
    // angles / distance, corner localization noise in pixel terms is
    // naturally higher (more foreshortening per pixel), so a threshold tuned
    // for a frame-filling board can start rejecting good corners as
    // outliers. Loosen this (e.g. 8-10) if homography keeps failing only on
    // far/angled boards; tighten it again if the locked homography looks
    // inaccurate.
    private static final double RANSAC_REPROJ_THRESHOLD_PX = 8.0;

    // Fixed output canvas size for the warped image. This is NOT computed
    // from the homography or the input frame — it's just OUTPUT_SCALE_PX and
    // MARGIN_PX applied to the calibration grid's own extents, so the canvas
    // exactly matches the space the chessboard was mapped into. Tune by hand:
    // bump these up if the warped output looks clipped, or down to crop in
    // tighter / save resolution.
    private static final int OUTPUT_WIDTH_PX  =
            (int) (2 * MARGIN_PX + (GRID_COLS - 1) * SQUARE_SIZE_INCHES * OUTPUT_SCALE_PX);
    private static final int OUTPUT_HEIGHT_PX =
            (int) (2 * MARGIN_PX + (GRID_ROWS - 1) * SQUARE_SIZE_INCHES * OUTPUT_SCALE_PX);
    private static final Size WARP_SIZE = new Size(OUTPUT_WIDTH_PX, OUTPUT_HEIGHT_PX);

    // ── Thread-safe state ────────────────────────────────────────────────────
    private final AtomicBoolean        homographyLocked    = new AtomicBoolean(false);
    private final AtomicReference<Mat> lockedHomography    = new AtomicReference<>(null);
    private final AtomicInteger        confirmCount        = new AtomicInteger(0);
    private final AtomicReference<Mat> candidateHomography = new AtomicReference<>(null);

    private final AtomicReference<Mat> pendingFrame     = new AtomicReference<>(null);
    private final AtomicBoolean        detectionRunning  = new AtomicBoolean(false);

    private volatile String statusLine    = "Searching for chessboard...";
    private volatile String homographyStr = null;

    // ── Pipeline-thread-only fields ──────────────────────────────────────────
    private final Telemetry    telemetry;
    private final MatOfPoint2f dstCorners;
    private final Mat          warped = new Mat();
    // Reused output buffer. Was allocated per-frame → a full-resolution native Mat leaked every
    // frame once locked (the EasyOpenCV viewport copies pixels out, it doesn't take ownership).
    private final Mat          output = new Mat();

    private final Thread detectionThread;

    public HomographyCalculationPipeline(Telemetry telemetry) {
        this.telemetry  = telemetry;
        this.dstCorners = buildCalibrationDstCorners();

        detectionThread = new Thread(this::detectionLoop, "HomographyDetection");
        detectionThread.setDaemon(true);
        detectionThread.start();
    }

    // ────────────────────────────────────────────────────────────────────────
    //  Pipeline thread
    // ────────────────────────────────────────────────────────────────────────

    @Override
    public Mat processFrame(Mat input) {

        if (homographyLocked.get()) {
            // Warp into the fixed-size canvas defined by OUTPUT_WIDTH_PX / OUTPUT_HEIGHT_PX
            Imgproc.warpPerspective(input, warped, lockedHomography.get(), WARP_SIZE);

            // Stretch the warped result back to the original camera resolution (reused buffer).
            Imgproc.resize(warped, output,
                    new Size(input.width(), input.height()),
                    0, 0, Imgproc.INTER_LINEAR);

            telemetry.addLine("HOMOGRAPHY LOCKED");
            telemetry.addLine(homographyStr);
            telemetry.update();
            return output;
        }

        // Post a full-res clone to the bg thread only when it is idle
        if (!detectionRunning.get() && pendingFrame.get() == null) {
            pendingFrame.set(input.clone());
        }

        telemetry.addLine(statusLine);
        telemetry.addData("Confirmations", confirmCount.get() + " / " + FRAMES_TO_CONFIRM);
        telemetry.update();
        return input;
    }

    // ────────────────────────────────────────────────────────────────────────
    //  Background detection loop
    // ────────────────────────────────────────────────────────────────────────

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

        // 1. Grayscale
        Imgproc.cvtColor(frame, gray, Imgproc.COLOR_RGB2GRAY);

        // 2. Detect chessboard corners on the full-res frame
        boolean found = Calib3d.findChessboardCorners(
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

        // 3. Sub-pixel refinement
        Imgproc.cornerSubPix(
                gray, corners,
                new Size(SUBPIX_WINDOW_PX, SUBPIX_WINDOW_PX), new Size(-1, -1),
                new TermCriteria(TermCriteria.EPS + TermCriteria.MAX_ITER, 30, 0.01)
        );

        // 4. Compute homography: image corners → flat grid (dstCorners)
        Mat h = Calib3d.findHomography(corners, dstCorners, Calib3d.RANSAC, RANSAC_REPROJ_THRESHOLD_PX);
        if (h == null || h.empty()) {
            statusLine = "Homography failed (RANSAC) — retrying...";
            return;
        }

        // Release the previous candidate we're superseding so it doesn't leak each frame before lock.
        Mat oldCandidate = candidateHomography.getAndSet(h);
        if (oldCandidate != null) oldCandidate.release();
        statusLine = "Board found — confirming...";

        if (confirmCount.incrementAndGet() >= FRAMES_TO_CONFIRM) {

            // No bounding-box fitting / translation correction here — the
            // homography maps straight into the fixed WARP_SIZE canvas
            // (defined by OUTPUT_WIDTH_PX / OUTPUT_HEIGHT_PX above). If the
            // board ends up off-canvas or clipped, tune MARGIN_PX,
            // OUTPUT_SCALE_PX, or the OUTPUT_WIDTH_PX/HEIGHT_PX constants
            // directly rather than relying on auto-sizing.
            lockedHomography.set(h);
            homographyStr = buildHomographyString(h);
            homographyLocked.set(true); // must be last
        }
    }

    // ────────────────────────────────────────────────────────────────────────
    //  Helpers
    // ────────────────────────────────────────────────────────────────────────

    private static MatOfPoint2f buildCalibrationDstCorners() {
        List<Point> points = new ArrayList<>(GRID_COLS * GRID_ROWS);
        for (int row = 0; row < GRID_ROWS; row++)
            for (int col = 0; col < GRID_COLS; col++)
                points.add(new Point(
                        MARGIN_PX + col * SQUARE_SIZE_INCHES * OUTPUT_SCALE_PX,
                        MARGIN_PX + row * SQUARE_SIZE_INCHES * OUTPUT_SCALE_PX));
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

    // ────────────────────────────────────────────────────────────────────────
    //  Public API
    // ────────────────────────────────────────────────────────────────────────

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