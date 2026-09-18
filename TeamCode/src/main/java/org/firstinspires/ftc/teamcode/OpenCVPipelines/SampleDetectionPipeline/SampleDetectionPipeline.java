package org.firstinspires.ftc.teamcode.OpenCVPipelines.SampleDetectionPipeline;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.core.TermCriteria;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

/**
 * ARCHITECTURE NOTE (shape-based detection refactor):
 *
 * DETECTION STRATEGY CHANGED from color-blob segmentation to curve/edge-based
 * circle fitting (Hough Circle Transform). Here's the difference and why it
 * was made:
 *
 * OLD (color-first): threshold HSV for "yellow-ish" pixels -> morphologically
 * CLOSE that mask aggressively to bridge every hole/glare gap into one solid
 * blob per ball -> distance-transform + watershed to split touching balls ->
 * validate the resulting blob's shape (area/circularity/roundness) as a
 * downstream filter. Every step depends on the color mask being close to a
 * solid disc FIRST. If a ball's surface is broken up badly enough by holes +
 * glare + shadow that no amount of closing bridges it into one blob, the ball
 * is invisible to everything downstream — color segmentation was the
 * bottleneck, and "is this actually round" was only ever checked AFTER a
 * blob already existed.
 *
 * NEW (shape-first): cv::HoughCircles searches directly for GRADIENT edges
 * whose curvature is consistent with a circle of some radius, independent of
 * what color or brightness sits inside that boundary. A hole punched in the
 * ball has its own strong edge, but that edge's curvature doesn't match the
 * ball's actual outer radius, so it doesn't vote for the ball's true center.
 * Glare is similarly irrelevant — it's just interior texture; Hough is
 * looking at the ball's outer silhouette against the (differently colored)
 * background, not at what's happening inside that boundary. This means a
 * ball can be found even when its visible surface is mostly broken up by
 * holes/glare/shadow, as long as its outer edge against the background is a
 * clean, mostly-unbroken curve.
 *
 * The yellow-color HSV mask is NOT gone — it's now used only as a coarse,
 * cheap ROI (region-of-interest) filter: find rough clusters of yellow-ish
 * pixels, pad them out generously, and run HoughCircles ONLY inside those
 * small regions. This keeps performance reasonable (HoughCircles over a
 * full 640x480 frame is expensive) while getting the robustness of curve
 * fitting for the actual circle detection.
 *
 * WHAT THIS ELIMINATES: distance transform, watershed, the native
 * marker-building chain, connected-components labeling, and the separate
 * post-hoc area/circularity/roundness validation. HoughCircles solves what
 * all of that machinery existed for — splitting touching balls (each circle
 * is found independently, so touching balls naturally separate) and
 * validating "is this actually round" (built into the algorithm, not a
 * downstream filter) — in one native call per ROI.
 */
public class SampleDetectionPipeline extends OpenCvPipeline {

    // -------------------------------------------------------------------------
    // MODE SWITCH: set to true to skip live homography calibration and use the
    // hardcoded H_ARRAY below instead.
    // -------------------------------------------------------------------------
    private static final boolean USE_PREDETERMINED_HOMOGRAPHY = true;

    // -------------------------------------------------------------------------
    // DISPLAY MODE — all modes draw on the ORIGINAL (unwarped) camera frame,
    // preserving full field of view.
    //
    //   MASK    — the coarse yellow ROI mask (white = candidate region) at
    //             full camera resolution, with circle/center/contact
    //             overlays. Useful for tuning HSV thresholds and seeing
    //             which regions Hough actually searched.
    //
    //   OVERLAY — full-color camera image with circle outline + center dot +
    //             contact dot overlays. Useful for verifying detections
    //             against the real scene.
    //
    //   BOX     — full-color camera image with a clean axis-aligned bounding
    //             box around each ball and a solid label plate above the box
    //             showing the ball number and its calculated field
    //             coordinates. Best for a clean, presentation-style view.
    // -------------------------------------------------------------------------
    public enum DisplayMode { MASK, OVERLAY, BOX }
    private static final DisplayMode DISPLAY_MODE = DisplayMode.MASK; // ← change here

    // -------------------------------------------------------------------------
    // Homography matrix mapping full-resolution image pixels directly to
    // field-coordinate inches. See buildCalibrationDstCorners() for how the
    // calibration destination points are defined.
    // -------------------------------------------------------------------------
    private static final double[][] H_ARRAY = {
            { -1.7797474624e-01, -5.3062009235e-02,  6.0413594965e+01 },
            { -2.0685716542e-02, -3.9378157948e-01,  1.4174826982e+02 },
            { -2.8668090956e-04, -1.2403394999e-02,  1.0000000000e+00 }
    };

    // -------------------------------------------------------------------------
    // Detection downscale factor. Still the single biggest performance lever:
    // ROI-finding (HSV threshold + light morphology) scales with pixel count,
    // and a smaller frame means smaller (cheaper) Hough search regions too.
    // Detected points are scaled back to full resolution before being
    // transformed by the homography, so reported field coordinates are
    // unaffected by this value.
    // -------------------------------------------------------------------------
    private static final double DETECTION_SCALE = 0.5;

    // -------------------------------------------------------------------------
    // Yellow ball HSV range, used ONLY to find coarse candidate ROIs now —
    // NOT required to produce one clean solid blob per ball anymore. This can
    // stay reasonably loose/generous since Hough does the real shape
    // validation; the color mask's only job is "is there probably a ball
    // somewhere around here" so Hough has a small region to search instead
    // of the whole frame.
    // -------------------------------------------------------------------------
    private static final Scalar YELLOW_LOW  = new Scalar(15, 100, 100);
    private static final Scalar YELLOW_HIGH = new Scalar(34, 255, 255);
    private static final Scalar GLARE_LOW   = new Scalar(15,   0, 200);
    private static final Scalar GLARE_HIGH  = new Scalar(34,  90, 255);

    // -------------------------------------------------------------------------
    // ROI-finding morphology. Deliberately LIGHT compared to the old
    // pipeline's aggressive multi-pass closing — this only needs to merge a
    // few nearby fragments into a rough cluster, not bridge an entire ball's
    // hole pattern into one solid disc. Over-closing here just wastes time;
    // Hough doesn't need (or benefit from) a solid blob.
    // -------------------------------------------------------------------------
    private static final Mat ROI_CLOSE_KERNEL = Imgproc.getStructuringElement(
            Imgproc.MORPH_ELLIPSE, new Size(9, 9));

    // How much to pad each color-cluster's bounding box (in scaled pixels)
    // when turning it into a Hough search ROI. Generous padding matters more
    // here than in the old pipeline, because a ball's outer silhouette often
    // extends beyond where the color mask itself lit up (e.g. a thin sliver
    // of yellow near the edge, or a mostly-glare-washed near side).
    private static final int ROI_PAD_PX = 14;

    // Nearby candidate ROIs (within this many scaled pixels of each other)
    // are merged into one larger search region before running Hough, so a
    // single ball whose color mask fragmented into several disconnected
    // blobs still gets ONE combined ROI instead of several overlapping ones.
    private static final int ROI_MERGE_DIST_PX = 20;

    // -------------------------------------------------------------------------
    // Hough Circle Transform parameters. All radius bounds are expressed as a
    // FRACTION of the frame's shorter dimension, keeping them scale-invariant
    // with respect to DETECTION_SCALE and camera resolution — the same
    // scale-invariance philosophy the old pipeline used for area fractions.
    // -------------------------------------------------------------------------

    // Gaussian blur kernel applied before Hough — circle detection is
    // sensitive to pixel-level noise, and holes/glare are exactly the kind
    // of high-frequency noise this needs to smooth over before edge/gradient
    // analysis runs. Applied per-ROI AFTER the upscale below, not to the whole
    // small frame: a 5x5 blur erases the entire edge of a 3-5px-radius ball,
    // which is exactly the size a distant ball occupies at DETECTION_SCALE.
    private static final Size HOUGH_BLUR_KERNEL = new Size(5, 5);

    // dp: inverse ratio of accumulator resolution to image resolution. 1.0 =
    // accumulator has the same resolution as the input ROI (most precise,
    // more compute). Raise toward 1.5–2.0 if Hough becomes a bottleneck.
    private static final double HOUGH_DP = 1.2;

    // Minimum distance between detected circle centers, in pixels within the
    // ROI. Prevents multiple overlapping detections of the same ball's edge.
    // Derived from expected ball radius at runtime (see runDetectionFrame).
    private static final double HOUGH_MIN_DIST_FRACTION = 0.5; // * expected radius

    // Canny high threshold used internally by Hough for edge detection. The
    // low threshold is automatically half of this. Lower = more sensitive to
    // weak edges (catches faint ball outlines against low-contrast
    // backgrounds) but noisier; higher = only very strong edges vote.
    private static final double HOUGH_CANNY_THRESHOLD = 80;

    // Floor for the above once it has been divided down for an upscaled ROI
    // (see ROI_MAX_UPSCALE) — below this, sensor noise starts producing edges.
    private static final double HOUGH_CANNY_MIN_THRESHOLD = 30;

    // Accumulator vote threshold — how many edge-gradient votes a candidate
    // circle needs to be reported. LOWER finds more circles (including
    // false positives on strongly-textured non-ball regions); HIGHER is
    // stricter. Kept fairly low because ROIs are already color-pre-filtered,
    // so false positives here mostly cost a little compute, not accuracy.
    private static final double HOUGH_ACCUMULATOR_THRESHOLD = 22;

    // Upper bound on a searched circle relative to the ROI's own shorter
    // dimension — a circle can't meaningfully exceed the region it was found
    // in. This is only ever the tighter half of a min() against the absolute
    // ceiling below.
    private static final double HOUGH_MAX_RADIUS_FRACTION = 0.60;

    // Ball radius bounds as a fraction of the FRAME's shorter dimension.
    // These are deliberately NOT ROI-relative: ROI size has a hard floor of
    // 2 * ROI_PAD_PX regardless of how small the ball is, so a ROI-relative
    // minimum can never shrink below ~5px and silently excludes every distant
    // ball. Set MIN from the smallest apparent ball at the camera's furthest
    // useful range, MAX from the largest at its closest.
    private static final double MIN_BALL_RADIUS_FRAME_FRACTION = 0.02;
    private static final double MAX_BALL_RADIUS_FRAME_FRACTION = 0.1;

    // A distant ball is only a few pixels across at DETECTION_SCALE, and
    // Hough's accumulator votes scale with circumference — a 4px-radius circle
    // has ~25 edge pixels total to clear HOUGH_ACCUMULATOR_THRESHOLD with,
    // while a near ball clears it trivially. ROIs whose smallest searched
    // radius falls under this are upscaled before the Hough call so tiny
    // circles get a proportionate number of votes.
    private static final double HOUGH_WORKING_MIN_RADIUS_PX = 6.0;

    // Bounds the cost of the above; Hough is O(pixels), so an unbounded
    // upscale on a large ROI would be the whole frame's budget.
    private static final double ROI_MAX_UPSCALE = 4.0;

    // Fraction of a detected circle's own area that must be yellow-mask pixels
    // for it to count as a ball. ROIs are padded and merged, so they always
    // contain non-yellow margin, and HOUGH_GRADIENT votes along the gradient
    // normal in BOTH directions — a real ball's edge therefore also deposits a
    // phantom center about one radius out into the dark background. Without
    // this check nothing downstream distinguishes that phantom from the ball.
    // Kept low: holes, glare and shadow mean a real ball is never fully masked.
    private static final double MIN_YELLOW_FILL_FRACTION = 0.30;

    // Two kept circles must be separated by at least this fraction of the sum
    // of their radii. Hough's own minDist can't do this job: it is one value
    // per call, derived from the SMALLEST searched radius (~3px), so it cannot
    // suppress a second detection on a ball many times that size — and it does
    // nothing at all across separate ROI calls. Balls resting against each
    // other sit at 1.0 (dist == r1 + r2), so this must stay well under that.
    private static final double MIN_CENTER_SEPARATION_FRACTION = 0.7;

    // -------------------------------------------------------------------------
    // Homography calibration settings — GRID_COLS=9, GRID_ROWS=6 matches the
    // physical board (9 inner corners wide, 6 inner corners tall).
    // -------------------------------------------------------------------------
    private static final int   GRID_COLS        = 9;
    private static final int   GRID_ROWS        = 6;
    private static final int   EXPECTED_CORNERS = GRID_COLS * GRID_ROWS;
    private static final float SQUARE_SIZE_INCHES = 1.0f; // TODO: verify against your physical board
    private static final int   DETECTION_FRAME_INTERVAL = 3;
    private static final int   FRAMES_TO_CONFIRM = 5;

    // -------------------------------------------------------------------------
    // Pipeline state
    // -------------------------------------------------------------------------
    private enum Phase { CALIBRATING, DETECTING }

    // volatile: written on the camera thread (processFrame), read on the OpMode thread.
    private volatile Phase phase;
    private volatile Mat   homography   = null;
    private int   confirmCount = 0;
    private int   frameCount   = 0;

    private final Mat gray         = new Mat(); // full-res grayscale, calibration only
    private final Mat small        = new Mat(); // downscaled color frame
    private final Mat smallGray    = new Mat(); // downscaled grayscale, unblurred
    private final Mat roiWork      = new Mat(); // per-ROI upscaled + blurred, fed to Hough
    private final Mat hsv          = new Mat();
    private final Mat yellowMask   = new Mat();
    private final Mat glareMask    = new Mat();
    private final Mat roiMask      = new Mat(); // lightly-closed candidate mask
    private final Mat displayImage = new Mat();
    private final Mat upscaledMask = new Mat();
    // findContours hierarchy output, reused across frames.
    private final Mat contourHierarchy = new Mat();

    private final MatOfPoint2f dstCorners;

    private final Telemetry telemetry;

    private static class BallResult {
        double centerXSmall, centerYSmall;   // Hough circle center, small-image space
        double radiusSmall;                   // Hough circle radius, small-image space
        Point fieldPoint;                     // ground-contact point, transformed to inches
    }

    // =========================================================================
    // Constructor
    // =========================================================================
    public SampleDetectionPipeline(Telemetry telemetry) {
        this.telemetry = telemetry;

        if (USE_PREDETERMINED_HOMOGRAPHY) {
            homography = buildHomographyFromArray(H_ARRAY);
            phase = Phase.DETECTING;
        } else {
            phase = Phase.CALIBRATING;
        }

        dstCorners = buildCalibrationDstCorners();
    }

    // =========================================================================
    // Public accessors
    // =========================================================================

    public Mat getHomography()    { return homography; }
    public boolean isCalibrated() { return phase == Phase.DETECTING; }

    public String getHomographyAsString() {
        if (homography == null || homography.empty()) return "Homography not available";
        StringBuilder sb = new StringBuilder("double[][] H_ARRAY = {\n");
        for (int r = 0; r < 3; r++) {
            sb.append("    { ");
            for (int c = 0; c < 3; c++) {
                sb.append(String.format("%.10e", homography.get(r, c)[0]));
                if (c < 2) sb.append(", ");
            }
            sb.append(" }");
            if (r < 2) sb.append(",");
            sb.append("\n");
        }
        sb.append("};");
        return sb.toString();
    }

    // =========================================================================
    // Main pipeline entry point
    // =========================================================================
    @Override
    public Mat processFrame(Mat input) {
        switch (phase) {
            case CALIBRATING: return runCalibrationFrame(input);
            case DETECTING:   return runDetectionFrame(input);
            default:          return input;
        }
    }

    // =========================================================================
    // PHASE 1 — Homography calibration (unchanged from prior version)
    // =========================================================================

    private Mat runCalibrationFrame(Mat input) {
        frameCount++;

        if (frameCount % DETECTION_FRAME_INTERVAL != 0) {
            telemetry.addLine("[Calibrating] Searching for chessboard...");
            telemetry.addData("Confirmed", confirmCount + " / " + FRAMES_TO_CONFIRM);
            telemetry.update();
            return input;
        }

        MatOfPoint2f imageCorners = detectChessboardCorners(input);

        if (imageCorners == null) {
            confirmCount = 0;
            telemetry.addLine("[Calibrating] Chessboard NOT found");
            telemetry.addData("Tip", "Ensure full board is visible and flat");
            telemetry.update();
            return input;
        }

        Mat h = computeHomography(imageCorners, dstCorners);

        if (h == null) {
            imageCorners.release();
            confirmCount = 0;
            telemetry.addLine("[Calibrating] Homography computation failed");
            telemetry.update();
            return input;
        }

        Mat previousHomography = homography;
        homography = h;
        if (previousHomography != null && previousHomography != h) previousHomography.release();
        confirmCount++;

        if (confirmCount >= FRAMES_TO_CONFIRM) {
            phase = Phase.DETECTING;
        }

        telemetry.addLine(phase == Phase.DETECTING
                ? "[LOCKED] Switching to detection..."
                : "[Calibrating] Confirming...");
        telemetry.addLine("--- Homography (image px -> field inches) ---");
        telemetry.addLine(getHomographyAsString());
        telemetry.update();

        Calib3d.drawChessboardCorners(input, new Size(GRID_COLS, GRID_ROWS), imageCorners, true);
        imageCorners.release();
        return input;
    }

    // =========================================================================
    // PHASE 2 — Ball detection via color-filtered ROIs + Hough circle fitting
    // =========================================================================

    private Mat runDetectionFrame(Mat input) {

        Size smallSize = new Size(input.cols() * DETECTION_SCALE, input.rows() * DETECTION_SCALE);
        Imgproc.resize(input, small, smallSize, 0, 0, Imgproc.INTER_AREA);

        // --- Step 1: coarse color-based ROI candidates -------------------------
        Imgproc.cvtColor(small, hsv, Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsv, YELLOW_LOW, YELLOW_HIGH, yellowMask);
        Core.inRange(hsv, GLARE_LOW, GLARE_HIGH, glareMask);
        Core.bitwise_or(yellowMask, glareMask, yellowMask);

        // Light closing only — just enough to merge nearby fragments of the
        // SAME ball into one rough cluster. Not trying to produce a solid disc.
        Imgproc.morphologyEx(yellowMask, roiMask, Imgproc.MORPH_CLOSE, ROI_CLOSE_KERNEL);

        // --- Step 2: build merged ROI rectangles from color clusters -----------
        List<Rect> rois = findMergedRois(roiMask);

        // --- Step 3: grayscale once for the whole small frame, reused as the
        //             source for every ROI's Hough search -------------------
        Imgproc.cvtColor(small, smallGray, Imgproc.COLOR_RGB2GRAY);

        if (DISPLAY_MODE != DisplayMode.MASK) {
            input.copyTo(displayImage);
        }

        List<BallResult> candidates = new ArrayList<>();
        int rejectedCount = 0;

        double frameShortSide  = Math.min(small.cols(), small.rows());
        double minBallRadiusSmall = frameShortSide * MIN_BALL_RADIUS_FRAME_FRACTION;
        double maxBallRadiusSmall = frameShortSide * MAX_BALL_RADIUS_FRAME_FRACTION;

        // --- Step 4: run HoughCircles within each candidate ROI -----------------
        for (Rect roi : rois) {
            int shortSide = Math.min(roi.width, roi.height);
            if (shortSide < 4) continue; // too small to meaningfully search

            double maxRadius = Math.min(shortSide * HOUGH_MAX_RADIUS_FRACTION, maxBallRadiusSmall);
            double minRadius = Math.min(minBallRadiusSmall, maxRadius * 0.5);
            double minDist   = Math.max(4.0, minRadius * HOUGH_MIN_DIST_FRACTION * 2.0);

            double roiScale = Math.min(ROI_MAX_UPSCALE,
                    Math.max(1.0, HOUGH_WORKING_MIN_RADIUS_PX / minRadius));

            // Interpolation spreads the same intensity step across roiScale
            // pixels, so per-pixel gradient magnitude drops by roughly that
            // factor — a fixed Canny threshold would reject the very edges the
            // upscale exists to recover.
            double cannyThreshold = Math.max(HOUGH_CANNY_MIN_THRESHOLD,
                    HOUGH_CANNY_THRESHOLD / roiScale);

            Mat roiGray = smallGray.submat(roi);
            Mat circles = new Mat();
            try {
                if (roiScale > 1.0) {
                    Imgproc.resize(roiGray, roiWork,
                            new Size(Math.round(roi.width * roiScale),
                                     Math.round(roi.height * roiScale)),
                            0, 0, Imgproc.INTER_LINEAR);
                } else {
                    roiGray.copyTo(roiWork); // submat is a view; don't blur smallGray in place
                }
                Imgproc.GaussianBlur(roiWork, roiWork, HOUGH_BLUR_KERNEL, 0);

                Imgproc.HoughCircles(roiWork, circles, Imgproc.HOUGH_GRADIENT,
                        HOUGH_DP, minDist * roiScale,
                        cannyThreshold, HOUGH_ACCUMULATOR_THRESHOLD,
                        (int) (minRadius * roiScale), (int) (maxRadius * roiScale));

                int cols = circles.cols();
                for (int i = 0; i < cols; i++) {
                    double[] c = circles.get(0, i);
                    // c = { centerX, centerY, radius }, upscaled-ROI-local coordinates
                    BallResult result = new BallResult();
                    result.centerXSmall = c[0] / roiScale + roi.x;
                    result.centerYSmall = c[1] / roiScale + roi.y;
                    result.radiusSmall  = c[2] / roiScale;

                    if (!hasYellowInterior(yellowMask, result)) {
                        rejectedCount++;
                        continue;
                    }

                    candidates.add(result);
                }
            } finally {
                circles.release();
                roiGray.release();
            }
        }

        // --- Step 5: drop overlapping detections, then map survivors to field ---
        List<BallResult> results = suppressOverlaps(candidates);
        int overlapCount = candidates.size() - results.size();

        for (BallResult result : results) {
            // Ground-contact point: analytically the lowest point on the
            // circle, (cx, cy + r) — exact, since Hough gives us a true
            // circle equation rather than a noisy pixel contour to hunt
            // through for the "lowest point" the way the old pipeline did.
            double contactXSmall = result.centerXSmall;
            double contactYSmall = result.centerYSmall + result.radiusSmall;

            double fullResX = contactXSmall / DETECTION_SCALE;
            double fullResY = contactYSmall / DETECTION_SCALE;
            result.fieldPoint = transformPointToField(fullResX, fullResY);
        }

        // --- Step 6: display ----------------------------------------------------
        if (DISPLAY_MODE == DisplayMode.MASK) {
            Imgproc.resize(roiMask, upscaledMask, input.size(), 0, 0, Imgproc.INTER_NEAREST);
            Imgproc.cvtColor(upscaledMask, displayImage, Imgproc.COLOR_GRAY2RGB);
        }

        if (DISPLAY_MODE == DisplayMode.BOX) {
            for (int i = 0; i < results.size(); i++) {
                drawBallBox(displayImage, results.get(i), i);
            }
        } else {
            for (BallResult r : results) {
                drawBallOverlay(displayImage, r);
            }
        }

        drawOriginCrosshair(displayImage);

        // --- Step 7: telemetry ---------------------------------------------------
        telemetry.addLine("[Detecting Yellow Balls — Hough circle fit]");
        telemetry.addData("ROIs searched", rois.size());
        telemetry.addData("Balls Detected", results.size());
        telemetry.addData("Rejected (not yellow)", rejectedCount);
        telemetry.addData("Rejected (overlap)", overlapCount);
        telemetry.addData("Radius search (px)", String.format("%.1f - %.1f",
                minBallRadiusSmall, maxBallRadiusSmall));
        for (int i = 0; i < results.size(); i++) {
            BallResult r = results.get(i);
            telemetry.addLine("--- Ball " + i + " ---");
            telemetry.addData("  Field X (in)", String.format("%.2f", r.fieldPoint.x));
            telemetry.addData("  Field Y (in)", String.format("%.2f", r.fieldPoint.y));
            telemetry.addData("  Radius (px)", String.format("%.1f", r.radiusSmall));
        }
        telemetry.update();

        return displayImage;
    }

    /**
     * Greedy non-maximum suppression over every circle found this frame,
     * largest first. A hole, a glare ring or a shadow inside a ball fits a
     * circle smaller than the ball's own outline, so preferring the larger
     * radius keeps the ball and drops the artifact.
     */
    private static List<BallResult> suppressOverlaps(List<BallResult> candidates) {
        Collections.sort(candidates, new Comparator<BallResult>() {
            @Override public int compare(BallResult a, BallResult b) {
                return Double.compare(b.radiusSmall, a.radiusSmall);
            }
        });

        List<BallResult> kept = new ArrayList<>(candidates.size());
        for (BallResult c : candidates) {
            boolean overlaps = false;
            for (BallResult k : kept) {
                double dx = c.centerXSmall - k.centerXSmall;
                double dy = c.centerYSmall - k.centerYSmall;
                double dist = Math.sqrt(dx * dx + dy * dy);

                // Second test catches an artifact sitting near a ball's rim,
                // whose centre is outside the kept circle but which is far too
                // close to be a separate ball.
                if (dist <= k.radiusSmall
                        || dist < MIN_CENTER_SEPARATION_FRACTION * (k.radiusSmall + c.radiusSmall)) {
                    overlaps = true;
                    break;
                }
            }
            if (!overlaps) kept.add(c);
        }
        return kept;
    }

    /**
     * Confirms a Hough circle actually sits on yellow pixels. Counts mask hits
     * in the circle's bounding box against the area a circle would occupy in
     * that box (pi/4 of it), so a ball clipped by the frame edge is judged on
     * its visible part rather than being penalised for the missing half.
     */
    private boolean hasYellowInterior(Mat mask, BallResult r) {
        int x0 = (int) Math.max(0, Math.round(r.centerXSmall - r.radiusSmall));
        int y0 = (int) Math.max(0, Math.round(r.centerYSmall - r.radiusSmall));
        int x1 = (int) Math.min(mask.cols(), Math.round(r.centerXSmall + r.radiusSmall));
        int y1 = (int) Math.min(mask.rows(), Math.round(r.centerYSmall + r.radiusSmall));
        if (x1 - x0 < 1 || y1 - y0 < 1) return false;

        Mat box = mask.submat(new Rect(x0, y0, x1 - x0, y1 - y0));
        try {
            double circleArea = (Math.PI / 4.0) * box.cols() * box.rows();
            return Core.countNonZero(box) >= MIN_YELLOW_FILL_FRACTION * circleArea;
        } finally {
            box.release();
        }
    }

    /**
     * Finds coarse candidate regions from the (loosely closed) color mask,
     * pads each one out, and merges any that are close together into a
     * single combined ROI. This is intentionally forgiving — its only job is
     * to hand Hough a small region that PROBABLY contains a ball; Hough does
     * the actual shape validation.
     */
    private List<Rect> findMergedRois(Mat mask) {
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(mask, contours, contourHierarchy,
                Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        List<Rect> padded = new ArrayList<>(contours.size());
        for (MatOfPoint c : contours) {
            Rect r = Imgproc.boundingRect(c);
            c.release();
            int x = Math.max(0, r.x - ROI_PAD_PX);
            int y = Math.max(0, r.y - ROI_PAD_PX);
            int w = Math.min(mask.cols() - x, r.width  + ROI_PAD_PX * 2);
            int h = Math.min(mask.rows() - y, r.height + ROI_PAD_PX * 2);
            if (w > 0 && h > 0) padded.add(new Rect(x, y, w, h));
        }

        // Merge ROIs that are close together (fragments of the same ball).
        List<Rect> merged = new ArrayList<>();
        boolean[] consumed = new boolean[padded.size()];
        for (int i = 0; i < padded.size(); i++) {
            if (consumed[i]) continue;
            Rect current = padded.get(i);
            consumed[i] = true;
            boolean growing = true;
            while (growing) {
                growing = false;
                for (int j = 0; j < padded.size(); j++) {
                    if (consumed[j]) continue;
                    if (rectsNear(current, padded.get(j), ROI_MERGE_DIST_PX)) {
                        current = union(current, padded.get(j));
                        consumed[j] = true;
                        growing = true;
                    }
                }
            }
            merged.add(current);
        }
        return merged;
    }

    private static boolean rectsNear(Rect a, Rect b, int dist) {
        Rect expandedA = new Rect(a.x - dist, a.y - dist, a.width + dist * 2, a.height + dist * 2);
        return expandedA.x < b.x + b.width && expandedA.x + expandedA.width > b.x
                && expandedA.y < b.y + b.height && expandedA.y + expandedA.height > b.y;
    }

    private static Rect union(Rect a, Rect b) {
        int x1 = Math.min(a.x, b.x);
        int y1 = Math.min(a.y, b.y);
        int x2 = Math.max(a.x + a.width,  b.x + b.width);
        int y2 = Math.max(a.y + a.height, b.y + b.height);
        return new Rect(x1, y1, x2 - x1, y2 - y1);
    }

    private void drawBallOverlay(Mat displayImage, BallResult r) {
        double scaleUp = 1.0 / DETECTION_SCALE;

        Point centerFull = new Point(r.centerXSmall * scaleUp, r.centerYSmall * scaleUp);
        int radiusFull = (int) (r.radiusSmall * scaleUp);

        Imgproc.circle(displayImage, centerFull, radiusFull, new Scalar(0, 255, 0), 2);
        Imgproc.circle(displayImage, centerFull, 4, new Scalar(255, 255, 0), -1); // center

        Point contactFull = new Point(centerFull.x, centerFull.y + radiusFull);
        Imgproc.circle(displayImage, contactFull, 5, new Scalar(0, 0, 255), -1);  // ground contact

        String label = String.format("(%.1f, %.1f)in", r.fieldPoint.x, r.fieldPoint.y);
        Imgproc.putText(displayImage, label,
                new Point(contactFull.x + 8, contactFull.y),
                Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255, 255, 255), 1);
    }

    /**
     * BOX display mode: draws a clean axis-aligned bounding rectangle around
     * the Hough-detected circle plus a solid label plate showing the ball's
     * index and its calculated field coordinates.
     */
    private void drawBallBox(Mat displayImage, BallResult r, int ballIndex) {
        double scaleUp = 1.0 / DETECTION_SCALE;

        double cxFull = r.centerXSmall * scaleUp;
        double cyFull = r.centerYSmall * scaleUp;
        double radiusFull = r.radiusSmall * scaleUp;

        Rect box = new Rect(
                (int) (cxFull - radiusFull), (int) (cyFull - radiusFull),
                (int) (radiusFull * 2), (int) (radiusFull * 2));

        Scalar boxColor  = new Scalar(0, 255, 0);
        Scalar textColor = new Scalar(255, 255, 255);

        Imgproc.rectangle(displayImage,
                new Point(box.x, box.y),
                new Point(box.x + box.width, box.y + box.height),
                boxColor, 2);

        String label = String.format("#%d (%.1f, %.1f)in", ballIndex, r.fieldPoint.x, r.fieldPoint.y);

        int fontFace = Imgproc.FONT_HERSHEY_SIMPLEX;
        double fontScale = 0.5;
        int thickness = 1;
        int[] baseline = new int[1];
        Size textSize = Imgproc.getTextSize(label, fontFace, fontScale, thickness, baseline);

        int padding = 4;
        int plateWidth  = (int) textSize.width  + padding * 2;
        int plateHeight = (int) textSize.height + baseline[0] + padding * 2;

        int plateDrawWidth = Math.max(plateWidth, box.width);
        int plateX = box.x;
        int plateY = box.y - plateHeight;

        if (plateY < 0) plateY = box.y + box.height + 2;
        if (plateX + plateDrawWidth > displayImage.cols()) {
            plateX = displayImage.cols() - plateDrawWidth;
        }
        if (plateX < 0) plateX = 0;

        Imgproc.rectangle(displayImage,
                new Point(plateX, plateY),
                new Point(plateX + plateDrawWidth, plateY + plateHeight),
                boxColor, -1);

        Point textOrigin = new Point(
                plateX + padding,
                plateY + plateHeight - padding - baseline[0]);
        Imgproc.putText(displayImage, label, textOrigin,
                fontFace, fontScale, textColor, thickness);
    }

    private Point transformPointToField(double x, double y) {
        MatOfPoint2f src = new MatOfPoint2f(new Point(x, y));
        MatOfPoint2f dst = new MatOfPoint2f();
        Core.perspectiveTransform(src, dst, homography);
        return dst.toArray()[0];
    }

    private Point transformFieldToPoint(double fieldX, double fieldY) {
        Mat inverseHomography = homography.inv();
        MatOfPoint2f src = new MatOfPoint2f(new Point(fieldX, fieldY));
        MatOfPoint2f dst = new MatOfPoint2f();
        Core.perspectiveTransform(src, dst, inverseHomography);
        inverseHomography.release();
        return dst.toArray()[0];
    }

    private void drawOriginCrosshair(Mat displayImage) {
        if (homography == null || homography.empty()) return;

        Point originPixel = transformFieldToPoint(0.0, 0.0);

        int size = 10;
        Scalar color = new Scalar(255, 0, 255);

        Imgproc.line(displayImage,
                new Point(originPixel.x - size, originPixel.y),
                new Point(originPixel.x + size, originPixel.y),
                color, 2);
        Imgproc.line(displayImage,
                new Point(originPixel.x, originPixel.y - size),
                new Point(originPixel.x, originPixel.y + size),
                color, 2);
        Imgproc.circle(displayImage, originPixel, 3, color, -1);
        Imgproc.putText(displayImage, "(0,0)",
                new Point(originPixel.x + size + 4, originPixel.y),
                Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, color, 1);
    }

    // =========================================================================
    // Homography calibration helper methods
    // =========================================================================

    private MatOfPoint2f detectChessboardCorners(Mat input) {
        Imgproc.cvtColor(input, gray, Imgproc.COLOR_RGB2GRAY);

        MatOfPoint2f imageCorners = new MatOfPoint2f();
        boolean found = Calib3d.findChessboardCorners(
                gray,
                new Size(GRID_COLS, GRID_ROWS),
                imageCorners,
                Calib3d.CALIB_CB_ADAPTIVE_THRESH |
                        Calib3d.CALIB_CB_NORMALIZE_IMAGE |
                        Calib3d.CALIB_CB_FAST_CHECK
        );

        if (!found || imageCorners.rows() != EXPECTED_CORNERS) return null;

        Imgproc.cornerSubPix(
                gray, imageCorners,
                new Size(5, 5), new Size(-1, -1),
                new TermCriteria(TermCriteria.EPS + TermCriteria.MAX_ITER, 30, 0.01)
        );

        return imageCorners;
    }

    private static Mat computeHomography(MatOfPoint2f srcCorners, MatOfPoint2f dstCorners) {
        Mat h = Calib3d.findHomography(srcCorners, dstCorners, Calib3d.RANSAC, 5.0);
        return (h == null || h.empty()) ? null : h;
    }

    private static MatOfPoint2f buildCalibrationDstCorners() {
        List<Point> dstList = new ArrayList<>();
        for (int row = 0; row < GRID_ROWS; row++) {
            for (int col = 0; col < GRID_COLS; col++) {
                dstList.add(new Point(
                        col * SQUARE_SIZE_INCHES,
                        row * SQUARE_SIZE_INCHES
                ));
            }
        }
        MatOfPoint2f dst = new MatOfPoint2f();
        dst.fromList(dstList);
        return dst;
    }

    private static Mat buildHomographyFromArray(double[][] arr) {
        Mat h = new Mat(3, 3, CvType.CV_64F);
        for (int r = 0; r < 3; r++)
            for (int c = 0; c < 3; c++)
                h.put(r, c, arr[r][c]);
        return h;
    }
}