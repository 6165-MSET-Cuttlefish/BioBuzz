package org.firstinspires.ftc.teamcode.modules.vision;

import com.acmerobotics.dashboard.config.Config;

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
 * Finds yellow balls on the floor and reports where they are — and how they are moving — in field
 * inches. Runs on the camera thread; results are published as an immutable {@link Frame} snapshot
 * that the OpMode thread reads via {@link #latest()}.
 *
 * <p>Detection is shape-first: the HSV yellow gate is only a cheap region-of-interest finder, and
 * {@code HoughCircles} does the real work of deciding what is a ball. Hough votes on gradient
 * curvature, so a ball whose surface is broken up by wiffle holes, glare and shadow is still found
 * from its outer silhouette alone, and two touching balls are found as two independent circles —
 * both things a color-blob segmentation has to bolt on afterwards with watershed splitting and
 * circularity filters.
 *
 * <p>Every detected ball is then fed to a {@link BallTracker}, which is what turns a stream of
 * unlabelled per-frame points into balls with identity and velocity.
 */
public class BallDetectionPipeline extends OpenCvPipeline {

    public enum DisplayMode {
        /** Candidate ROI mask, for tuning the HSV gate. */
        MASK,
        /** Camera image with circle, center and ground-contact overlays. */
        OVERLAY,
        /** Camera image with a bounding box and a field-coordinate label plate per ball. */
        BOX
    }

    @Config("BallVision")
    public static class Tuning {
        public static DisplayMode displayMode = DisplayMode.BOX;
        public static boolean drawVelocity = true;
        /** Lookahead of the drawn velocity arrow, seconds. */
        public static double velocityArrowSeconds = 0.5;

        /** OpenCV HSV: H 0-179, S/V 0-255. Loose on purpose — Hough is the real shape gate. */
        public static int hLow = 15, hHigh = 34;
        public static int sLow = 100, sHigh = 255;
        public static int vLow = 100, vHigh = 255;
        /** Specular highlights on a ball's top: same hue, washed-out saturation, near-max value. */
        public static int glareSHigh = 90, glareVLow = 200;

        public static double houghDp = 1.2;
        /** Canny high threshold; lower catches faint outlines but votes for more noise. */
        public static double houghCanny = 80;
        /** Accumulator votes needed to report a circle. */
        public static double houghAccumulator = 22;
        /** Ball radius bounds as a fraction of the frame's shorter side. */
        public static double minRadiusFrameFraction = 0.02;
        public static double maxRadiusFrameFraction = 0.10;
        /** Fraction of a circle's own area that must be yellow-mask pixels to count as a ball. */
        public static double minYellowFill = 0.30;
        /** Min center separation as a fraction of the two radii summed. */
        public static double minCenterSeparation = 0.7;
    }

    /** Skips live chessboard calibration and uses {@link #H_ARRAY} instead. */
    private static final boolean USE_PREDETERMINED_HOMOGRAPHY = true;

    /** Full-resolution image pixels to field inches. */
    private static final double[][] H_ARRAY = {
            { -1.7797474624e-01, -5.3062009235e-02,  6.0413594965e+01 },
            { -2.0685716542e-02, -3.9378157948e-01,  1.4174826982e+02 },
            { -2.8668090956e-04, -1.2403394999e-02,  1.0000000000e+00 }
    };

    /**
     * Detection runs on a downscaled copy — the single biggest performance lever, since both the
     * HSV pass and the Hough search scale with pixel count. Points are scaled back up before the
     * homography is applied, so field coordinates are unaffected by this value.
     */
    private static final double DETECTION_SCALE = 0.5;

    /**
     * Light close, unlike a color-first pipeline's aggressive multi-pass fill: this only has to
     * merge nearby fragments into a rough cluster, not rebuild a solid disc.
     */
    private static final Mat ROI_CLOSE_KERNEL =
            Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(9, 9));

    /** A ball's silhouette routinely extends past where its color mask lit up, so pad generously. */
    private static final int ROI_PAD_PX = 14;
    /** Fragments of one ball that survived as separate blobs get merged into a single search box. */
    private static final int ROI_MERGE_DIST_PX = 20;

    /**
     * Blur before Hough — holes and glare are exactly the high-frequency noise that wrecks gradient
     * analysis. Applied per-ROI after the upscale below, because a 5x5 blur applied to the small
     * frame erases the entire edge of a 3-5px-radius ball.
     */
    private static final Size HOUGH_BLUR_KERNEL = new Size(5, 5);
    private static final double HOUGH_MIN_DIST_FRACTION = 0.5;
    private static final double HOUGH_CANNY_MIN_THRESHOLD = 30;
    private static final double HOUGH_MAX_RADIUS_FRACTION = 0.60;

    /**
     * Hough's votes scale with circumference, so a distant 4px-radius ball has ~25 edge pixels to
     * clear the accumulator threshold with while a near ball clears it trivially. ROIs searching
     * below this radius are upscaled first so small circles get a proportionate vote count.
     */
    private static final double HOUGH_WORKING_MIN_RADIUS_PX = 6.0;
    /** Bounds the cost of that upscale; Hough is O(pixels). */
    private static final double ROI_MAX_UPSCALE = 4.0;

    private static final int   GRID_COLS = 9;
    private static final int   GRID_ROWS = 6;
    private static final int   EXPECTED_CORNERS = GRID_COLS * GRID_ROWS;
    private static final float SQUARE_SIZE_INCHES = 1.0f;
    private static final int   CALIBRATION_FRAME_INTERVAL = 3;
    private static final int   FRAMES_TO_CONFIRM = 5;

    private static final double FPS_SMOOTHING = 0.1;
    private static final double NANOS_TO_SECONDS = 1e-9;

    private static final Scalar COLOR_CIRCLE   = new Scalar(0, 255, 0);
    private static final Scalar COLOR_CENTER   = new Scalar(255, 255, 0);
    private static final Scalar COLOR_CONTACT  = new Scalar(0, 0, 255);
    private static final Scalar COLOR_LABEL    = new Scalar(255, 255, 255);
    private static final Scalar COLOR_VELOCITY = new Scalar(255, 128, 0);
    private static final Scalar COLOR_ORIGIN   = new Scalar(255, 0, 255);

    private enum Phase { CALIBRATING, DETECTING }

    /** One frame's worth of results, handed from the camera thread to the OpMode thread. */
    public static final class Frame {
        public static final Frame EMPTY = new Frame(
                Collections.<BallDetection>emptyList(), Collections.<TrackedBall>emptyList(),
                0, 0, 0, 0, 0);

        public final List<BallDetection> detections;
        public final List<TrackedBall> balls;
        public final int roiCount;
        public final int rejectedColor;
        public final int rejectedOverlap;
        public final double fps;
        public final double timestampSeconds;

        Frame(List<BallDetection> detections, List<TrackedBall> balls, int roiCount,
              int rejectedColor, int rejectedOverlap, double fps, double timestampSeconds) {
            this.detections = Collections.unmodifiableList(detections);
            this.balls = Collections.unmodifiableList(balls);
            this.roiCount = roiCount;
            this.rejectedColor = rejectedColor;
            this.rejectedOverlap = rejectedOverlap;
            this.fps = fps;
            this.timestampSeconds = timestampSeconds;
        }
    }

    // Written on the camera thread, read on the OpMode thread.
    private volatile Frame latest = Frame.EMPTY;
    private volatile Phase phase;
    private volatile Mat homography;
    private volatile Mat inverseHomography;
    private volatile boolean detectionEnabled = true;
    private volatile boolean trackerResetRequested = false;

    private final BallTracker tracker = new BallTracker();
    private final MatOfPoint2f calibrationDstCorners = buildCalibrationDstCorners();

    private final Mat gray         = new Mat();
    private final Mat small        = new Mat();
    private final Mat smallGray    = new Mat();
    private final Mat hsv          = new Mat();
    private final Mat yellowMask   = new Mat();
    private final Mat glareMask    = new Mat();
    private final Mat roiMask      = new Mat();
    private final Mat roiWork      = new Mat();
    private final Mat upscaledMask = new Mat();
    private final Mat display      = new Mat();
    private final Mat contourHierarchy = new Mat();

    private int confirmCount = 0;
    private int calibrationFrameCount = 0;
    private int rejectedColorCount = 0;
    private int rejectedOverlapCount = 0;
    private double fps = 0;
    private double lastFrameSeconds = Double.NaN;

    public BallDetectionPipeline() {
        if (USE_PREDETERMINED_HOMOGRAPHY) {
            setHomography(buildHomographyFromArray(H_ARRAY));
            phase = Phase.DETECTING;
        } else {
            phase = Phase.CALIBRATING;
        }
    }

    // =========================================================================
    // Master pipeline
    // =========================================================================

    @Override
    public Mat processFrame(Mat input) {
        if (!detectionEnabled) return input;
        return phase == Phase.CALIBRATING ? runCalibration(input) : runDetection(input);
    }

    private Mat runDetection(Mat input) {
        double timestamp = System.nanoTime() * NANOS_TO_SECONDS;
        updateFps(timestamp);
        rejectedColorCount = 0;

        if (trackerResetRequested) {
            trackerResetRequested = false;
            tracker.reset();
        }

        prepareWorkingFrames(input);
        buildColorMask();

        List<Rect> searchRegions = buildSearchRegions();
        List<Candidate> candidates = findCircles(searchRegions);
        List<Candidate> circles = suppressOverlaps(candidates);
        rejectedOverlapCount = candidates.size() - circles.size();

        List<BallDetection> detections = projectToField(circles);
        List<TrackedBall> balls = tracker.update(detections, timestamp);

        latest = new Frame(detections, balls, searchRegions.size(),
                rejectedColorCount, rejectedOverlapCount, fps, timestamp);

        return render(input, detections, balls);
    }

    // =========================================================================
    // Detection stages
    // =========================================================================

    private void prepareWorkingFrames(Mat input) {
        Imgproc.resize(input, small,
                new Size(input.cols() * DETECTION_SCALE, input.rows() * DETECTION_SCALE),
                0, 0, Imgproc.INTER_AREA);
        Imgproc.cvtColor(small, hsv, Imgproc.COLOR_RGB2HSV);
        Imgproc.cvtColor(small, smallGray, Imgproc.COLOR_RGB2GRAY);
    }

    private void buildColorMask() {
        Core.inRange(hsv,
                new Scalar(Tuning.hLow, Tuning.sLow, Tuning.vLow),
                new Scalar(Tuning.hHigh, Tuning.sHigh, Tuning.vHigh), yellowMask);
        Core.inRange(hsv,
                new Scalar(Tuning.hLow, 0, Tuning.glareVLow),
                new Scalar(Tuning.hHigh, Tuning.glareSHigh, 255), glareMask);
        Core.bitwise_or(yellowMask, glareMask, yellowMask);
        Imgproc.morphologyEx(yellowMask, roiMask, Imgproc.MORPH_CLOSE, ROI_CLOSE_KERNEL);
    }

    private List<Rect> buildSearchRegions() {
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(roiMask, contours, contourHierarchy,
                Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        List<Rect> padded = new ArrayList<>(contours.size());
        for (MatOfPoint contour : contours) {
            Rect bounds = Imgproc.boundingRect(contour);
            contour.release();
            int x = Math.max(0, bounds.x - ROI_PAD_PX);
            int y = Math.max(0, bounds.y - ROI_PAD_PX);
            int w = Math.min(roiMask.cols() - x, bounds.width + ROI_PAD_PX * 2);
            int h = Math.min(roiMask.rows() - y, bounds.height + ROI_PAD_PX * 2);
            if (w > 0 && h > 0) padded.add(new Rect(x, y, w, h));
        }
        return mergeNearbyRects(padded);
    }

    private List<Candidate> findCircles(List<Rect> searchRegions) {
        double frameShortSide = Math.min(small.cols(), small.rows());
        double minBallRadius = frameShortSide * Tuning.minRadiusFrameFraction;
        double maxBallRadius = frameShortSide * Tuning.maxRadiusFrameFraction;

        List<Candidate> candidates = new ArrayList<>();
        for (Rect region : searchRegions) {
            int shortSide = Math.min(region.width, region.height);
            if (shortSide < 4) continue;

            double maxRadius = Math.min(shortSide * HOUGH_MAX_RADIUS_FRACTION, maxBallRadius);
            double minRadius = Math.min(minBallRadius, maxRadius * 0.5);
            double minDist = Math.max(4.0, minRadius * HOUGH_MIN_DIST_FRACTION * 2.0);
            double upscale = Math.min(ROI_MAX_UPSCALE,
                    Math.max(1.0, HOUGH_WORKING_MIN_RADIUS_PX / minRadius));

            // Interpolation spreads the same intensity step over `upscale` pixels, so per-pixel
            // gradient magnitude drops by about that factor — a fixed Canny threshold would reject
            // the very edges the upscale exists to recover.
            double canny = Math.max(HOUGH_CANNY_MIN_THRESHOLD, Tuning.houghCanny / upscale);

            Mat regionGray = smallGray.submat(region);
            Mat circles = new Mat();
            try {
                if (upscale > 1.0) {
                    Imgproc.resize(regionGray, roiWork,
                            new Size(Math.round(region.width * upscale),
                                     Math.round(region.height * upscale)),
                            0, 0, Imgproc.INTER_LINEAR);
                } else {
                    regionGray.copyTo(roiWork); // submat is a view; don't blur smallGray in place
                }
                Imgproc.GaussianBlur(roiWork, roiWork, HOUGH_BLUR_KERNEL, 0);

                Imgproc.HoughCircles(roiWork, circles, Imgproc.HOUGH_GRADIENT,
                        Tuning.houghDp, minDist * upscale,
                        canny, Tuning.houghAccumulator,
                        (int) (minRadius * upscale), (int) (maxRadius * upscale));

                for (int i = 0; i < circles.cols(); i++) {
                    double[] circle = circles.get(0, i);
                    Candidate candidate = new Candidate(
                            circle[0] / upscale + region.x,
                            circle[1] / upscale + region.y,
                            circle[2] / upscale);
                    if (hasYellowInterior(candidate)) candidates.add(candidate);
                    else rejectedColorCount++;
                }
            } finally {
                circles.release();
                regionGray.release();
            }
        }
        return candidates;
    }

    /**
     * Rejects circles that don't sit on yellow pixels. ROIs are padded and merged so they always
     * contain non-ball margin, and HOUGH_GRADIENT votes along the gradient normal in both
     * directions — a real ball's edge therefore also deposits a phantom center about one radius out
     * into the background, and nothing else downstream tells that phantom from the ball. Counted
     * over the bounding box so a frame-clipped ball is judged on its visible part.
     */
    private boolean hasYellowInterior(Candidate candidate) {
        int x0 = (int) Math.max(0, Math.round(candidate.x - candidate.radius));
        int y0 = (int) Math.max(0, Math.round(candidate.y - candidate.radius));
        int x1 = (int) Math.min(yellowMask.cols(), Math.round(candidate.x + candidate.radius));
        int y1 = (int) Math.min(yellowMask.rows(), Math.round(candidate.y + candidate.radius));
        if (x1 - x0 < 1 || y1 - y0 < 1) return false;

        Mat box = yellowMask.submat(new Rect(x0, y0, x1 - x0, y1 - y0));
        try {
            double circleArea = (Math.PI / 4.0) * box.cols() * box.rows();
            return Core.countNonZero(box) >= Tuning.minYellowFill * circleArea;
        } finally {
            box.release();
        }
    }

    /**
     * Greedy non-maximum suppression, largest first: a hole, glare ring or shadow inside a ball
     * fits a circle smaller than the ball's own outline, so preferring the larger radius keeps the
     * ball and drops the artifact. Hough's own minDist can't do this — it is one value per call,
     * derived from the smallest searched radius, and does nothing across separate ROI calls.
     */
    private static List<Candidate> suppressOverlaps(List<Candidate> candidates) {
        Collections.sort(candidates, new Comparator<Candidate>() {
            @Override public int compare(Candidate a, Candidate b) {
                return Double.compare(b.radius, a.radius);
            }
        });

        List<Candidate> kept = new ArrayList<>(candidates.size());
        for (Candidate candidate : candidates) {
            boolean overlaps = false;
            for (Candidate k : kept) {
                double distance = Math.hypot(candidate.x - k.x, candidate.y - k.y);
                // Second test catches an artifact on a ball's rim, whose center is outside the kept
                // circle but which is far too close to be a separate ball. Balls resting against
                // each other sit at 1.0, so the fraction must stay well under that.
                if (distance <= k.radius
                        || distance < Tuning.minCenterSeparation * (k.radius + candidate.radius)) {
                    overlaps = true;
                    break;
                }
            }
            if (!overlaps) kept.add(candidate);
        }
        return kept;
    }

    /**
     * Maps each circle's ground-contact point to field inches. Hough gives a true circle equation,
     * so the contact point is analytically (cx, cy + r) rather than a search for the lowest pixel.
     */
    private List<BallDetection> projectToField(List<Candidate> circles) {
        Mat h = homography;
        if (circles.isEmpty() || h == null || h.empty()) {
            return Collections.emptyList();
        }

        List<Point> contacts = new ArrayList<>(circles.size());
        for (Candidate c : circles) {
            contacts.add(new Point(c.x / DETECTION_SCALE, (c.y + c.radius) / DETECTION_SCALE));
        }

        List<Point> field = perspectiveTransform(contacts, h);
        List<BallDetection> detections = new ArrayList<>(circles.size());
        for (int i = 0; i < circles.size(); i++) {
            Candidate c = circles.get(i);
            detections.add(new BallDetection(field.get(i).x, field.get(i).y, c.x, c.y, c.radius));
        }
        return detections;
    }

    // =========================================================================
    // Rendering
    // =========================================================================

    private Mat render(Mat input, List<BallDetection> detections, List<TrackedBall> balls) {
        DisplayMode mode = Tuning.displayMode;

        if (mode == DisplayMode.MASK) {
            Imgproc.resize(roiMask, upscaledMask, input.size(), 0, 0, Imgproc.INTER_NEAREST);
            Imgproc.cvtColor(upscaledMask, display, Imgproc.COLOR_GRAY2RGB);
        } else {
            input.copyTo(display);
        }

        for (int i = 0; i < detections.size(); i++) {
            if (mode == DisplayMode.BOX) drawBallBox(detections.get(i), i);
            else drawBallOverlay(detections.get(i));
        }

        if (Tuning.drawVelocity) drawVelocities(balls);
        drawOriginCrosshair();
        return display;
    }

    private void drawBallOverlay(BallDetection ball) {
        double scaleUp = 1.0 / DETECTION_SCALE;
        Point center = new Point(ball.imageX * scaleUp, ball.imageY * scaleUp);
        int radius = (int) (ball.imageRadius * scaleUp);

        Imgproc.circle(display, center, radius, COLOR_CIRCLE, 2);
        Imgproc.circle(display, center, 4, COLOR_CENTER, -1);

        Point contact = new Point(center.x, center.y + radius);
        Imgproc.circle(display, contact, 5, COLOR_CONTACT, -1);
        Imgproc.putText(display, String.format("(%.1f, %.1f)in", ball.fieldX, ball.fieldY),
                new Point(contact.x + 8, contact.y),
                Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, COLOR_LABEL, 1);
    }

    private void drawBallBox(BallDetection ball, int index) {
        double scaleUp = 1.0 / DETECTION_SCALE;
        double cx = ball.imageX * scaleUp;
        double cy = ball.imageY * scaleUp;
        double radius = ball.imageRadius * scaleUp;

        Rect box = new Rect((int) (cx - radius), (int) (cy - radius),
                (int) (radius * 2), (int) (radius * 2));
        Imgproc.rectangle(display, new Point(box.x, box.y),
                new Point(box.x + box.width, box.y + box.height), COLOR_CIRCLE, 2);

        drawLabelPlate(String.format("#%d (%.1f, %.1f)in", index, ball.fieldX, ball.fieldY), box);
    }

    private void drawLabelPlate(String label, Rect box) {
        int fontFace = Imgproc.FONT_HERSHEY_SIMPLEX;
        double fontScale = 0.5;
        int thickness = 1;
        int[] baseline = new int[1];
        Size textSize = Imgproc.getTextSize(label, fontFace, fontScale, thickness, baseline);

        int padding = 4;
        int plateHeight = (int) textSize.height + baseline[0] + padding * 2;
        int plateWidth = Math.max((int) textSize.width + padding * 2, box.width);

        int plateX = Math.max(0, Math.min(box.x, display.cols() - plateWidth));
        int plateY = box.y - plateHeight;
        if (plateY < 0) plateY = box.y + box.height + 2;

        Imgproc.rectangle(display, new Point(plateX, plateY),
                new Point(plateX + plateWidth, plateY + plateHeight), COLOR_CIRCLE, -1);
        Imgproc.putText(display, label,
                new Point(plateX + padding, plateY + plateHeight - padding - baseline[0]),
                fontFace, fontScale, COLOR_LABEL, thickness);
    }

    private void drawVelocities(List<TrackedBall> balls) {
        Mat inverse = inverseHomography;
        if (inverse == null || balls.isEmpty()) return;

        List<TrackedBall> moving = new ArrayList<>();
        List<Point> fieldPoints = new ArrayList<>();
        for (TrackedBall ball : balls) {
            if (!ball.isMoving()) continue;
            moving.add(ball);
            fieldPoints.add(ball.position());
            fieldPoints.add(ball.predict(Tuning.velocityArrowSeconds));
        }
        if (moving.isEmpty()) return;

        List<Point> pixels = perspectiveTransform(fieldPoints, inverse);
        for (int i = 0; i < moving.size(); i++) {
            Point from = pixels.get(i * 2);
            Point to = pixels.get(i * 2 + 1);
            Imgproc.arrowedLine(display, from, to, COLOR_VELOCITY, 2);
            Imgproc.putText(display,
                    String.format("#%d %.0fin/s", moving.get(i).id, moving.get(i).speed()),
                    new Point(to.x + 6, to.y), Imgproc.FONT_HERSHEY_SIMPLEX, 0.4, COLOR_VELOCITY, 1);
        }
    }

    private void drawOriginCrosshair() {
        Mat inverse = inverseHomography;
        if (inverse == null) return;

        Point origin = perspectiveTransform(
                Collections.singletonList(new Point(0, 0)), inverse).get(0);
        int size = 10;

        Imgproc.line(display, new Point(origin.x - size, origin.y),
                new Point(origin.x + size, origin.y), COLOR_ORIGIN, 2);
        Imgproc.line(display, new Point(origin.x, origin.y - size),
                new Point(origin.x, origin.y + size), COLOR_ORIGIN, 2);
        Imgproc.circle(display, origin, 3, COLOR_ORIGIN, -1);
        Imgproc.putText(display, "(0,0)", new Point(origin.x + size + 4, origin.y),
                Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, COLOR_ORIGIN, 1);
    }

    // =========================================================================
    // Homography calibration
    // =========================================================================

    private Mat runCalibration(Mat input) {
        calibrationFrameCount++;
        if (calibrationFrameCount % CALIBRATION_FRAME_INTERVAL != 0) return input;

        MatOfPoint2f imageCorners = detectChessboardCorners(input);
        if (imageCorners == null) {
            confirmCount = 0;
            return input;
        }

        Mat h = Calib3d.findHomography(imageCorners, calibrationDstCorners, Calib3d.RANSAC, 5.0);
        if (h == null || h.empty()) {
            imageCorners.release();
            confirmCount = 0;
            return input;
        }

        setHomography(h);
        if (++confirmCount >= FRAMES_TO_CONFIRM) phase = Phase.DETECTING;

        Calib3d.drawChessboardCorners(input, new Size(GRID_COLS, GRID_ROWS), imageCorners, true);
        imageCorners.release();
        return input;
    }

    private MatOfPoint2f detectChessboardCorners(Mat input) {
        Imgproc.cvtColor(input, gray, Imgproc.COLOR_RGB2GRAY);

        MatOfPoint2f corners = new MatOfPoint2f();
        boolean found = Calib3d.findChessboardCorners(gray, new Size(GRID_COLS, GRID_ROWS), corners,
                Calib3d.CALIB_CB_ADAPTIVE_THRESH
                        | Calib3d.CALIB_CB_NORMALIZE_IMAGE
                        | Calib3d.CALIB_CB_FAST_CHECK);
        if (!found || corners.rows() != EXPECTED_CORNERS) {
            corners.release();
            return null;
        }

        Imgproc.cornerSubPix(gray, corners, new Size(5, 5), new Size(-1, -1),
                new TermCriteria(TermCriteria.EPS + TermCriteria.MAX_ITER, 30, 0.01));
        return corners;
    }

    private static MatOfPoint2f buildCalibrationDstCorners() {
        List<Point> corners = new ArrayList<>(EXPECTED_CORNERS);
        for (int row = 0; row < GRID_ROWS; row++) {
            for (int col = 0; col < GRID_COLS; col++) {
                corners.add(new Point(col * SQUARE_SIZE_INCHES, row * SQUARE_SIZE_INCHES));
            }
        }
        MatOfPoint2f dst = new MatOfPoint2f();
        dst.fromList(corners);
        return dst;
    }

    private static Mat buildHomographyFromArray(double[][] values) {
        Mat h = new Mat(3, 3, CvType.CV_64F);
        for (int r = 0; r < 3; r++) {
            for (int c = 0; c < 3; c++) h.put(r, c, values[r][c]);
        }
        return h;
    }

    // =========================================================================
    // Public accessors
    // =========================================================================

    /** Latest published results. Never null; {@link Frame#EMPTY} until the first frame lands. */
    public Frame latest() { return latest; }

    public boolean isCalibrated() { return phase == Phase.DETECTING; }

    /** False makes {@link #processFrame} a passthrough, freeing the CPU it would have spent. */
    public void setDetectionEnabled(boolean enabled) {
        if (detectionEnabled == enabled) return;
        detectionEnabled = enabled;
        latest = Frame.EMPTY;
        // Resuming against tracks last seen an arbitrary time ago would associate them to whatever
        // is in frame now and derive a velocity from a gap that was never observed.
        resetTracking();
    }

    public boolean isDetectionEnabled() { return detectionEnabled; }

    /** Drops every track; the next frame starts identities and velocities from scratch. */
    public void resetTracking() { trackerResetRequested = true; }

    public String getHomographyAsString() {
        Mat h = homography;
        if (h == null || h.empty()) return "Homography not available";
        StringBuilder sb = new StringBuilder("double[][] H_ARRAY = {\n");
        for (int r = 0; r < 3; r++) {
            sb.append("    { ");
            for (int c = 0; c < 3; c++) {
                sb.append(String.format("%.10e", h.get(r, c)[0]));
                if (c < 2) sb.append(", ");
            }
            sb.append(" }").append(r < 2 ? "," : "").append("\n");
        }
        return sb.append("};").toString();
    }

    // =========================================================================
    // Helpers
    // =========================================================================

    private void setHomography(Mat h) {
        Mat previous = homography;
        Mat previousInverse = inverseHomography;
        homography = h;
        inverseHomography = (h == null || h.empty()) ? null : h.inv();
        if (previous != null && previous != h) previous.release();
        if (previousInverse != null) previousInverse.release();
    }

    private static List<Point> perspectiveTransform(List<Point> points, Mat transform) {
        MatOfPoint2f src = new MatOfPoint2f();
        src.fromList(points);
        MatOfPoint2f dst = new MatOfPoint2f();
        try {
            Core.perspectiveTransform(src, dst, transform);
            return dst.toList();
        } finally {
            src.release();
            dst.release();
        }
    }

    private static List<Rect> mergeNearbyRects(List<Rect> rects) {
        List<Rect> merged = new ArrayList<>();
        boolean[] consumed = new boolean[rects.size()];
        for (int i = 0; i < rects.size(); i++) {
            if (consumed[i]) continue;
            Rect current = rects.get(i);
            consumed[i] = true;
            boolean growing = true;
            while (growing) {
                growing = false;
                for (int j = 0; j < rects.size(); j++) {
                    if (consumed[j] || !isNear(current, rects.get(j), ROI_MERGE_DIST_PX)) continue;
                    current = union(current, rects.get(j));
                    consumed[j] = true;
                    growing = true;
                }
            }
            merged.add(current);
        }
        return merged;
    }

    private static boolean isNear(Rect a, Rect b, int distance) {
        return a.x - distance < b.x + b.width && a.x + a.width + distance > b.x
                && a.y - distance < b.y + b.height && a.y + a.height + distance > b.y;
    }

    private static Rect union(Rect a, Rect b) {
        int x = Math.min(a.x, b.x);
        int y = Math.min(a.y, b.y);
        return new Rect(x, y,
                Math.max(a.x + a.width, b.x + b.width) - x,
                Math.max(a.y + a.height, b.y + b.height) - y);
    }

    private void updateFps(double timestampSeconds) {
        if (!Double.isNaN(lastFrameSeconds)) {
            double dt = timestampSeconds - lastFrameSeconds;
            if (dt > 0) fps += FPS_SMOOTHING * (1.0 / dt - fps);
        }
        lastFrameSeconds = timestampSeconds;
    }

    private static final class Candidate {
        final double x, y, radius;

        Candidate(double x, double y, double radius) {
            this.x = x;
            this.y = y;
            this.radius = radius;
        }
    }
}
