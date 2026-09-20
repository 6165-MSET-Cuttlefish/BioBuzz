package org.firstinspires.ftc.teamcode.modules.vision;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.modules.vision.BallVisionConstants.BallType;
import org.firstinspires.ftc.teamcode.modules.vision.BallVisionConstants.BlueNectarHsv;
import org.firstinspires.ftc.teamcode.modules.vision.BallVisionConstants.Detection;
import org.firstinspires.ftc.teamcode.modules.vision.BallVisionConstants.PollenHsv;
import org.firstinspires.ftc.teamcode.modules.vision.BallVisionConstants.RedNectarHsv;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

/**
 * Finds Pollen (2.8in yellow) and Nectar (3.6in red/blue) balls on the floor and reports where they
 * are — and how they are moving — in field inches. Runs on the camera thread; results are published
 * as an immutable {@link Frame} snapshot that the OpMode thread reads via {@link #latest()}.
 *
 * <p>Detection is shape-first: each type's HSV gate is only a cheap region-of-interest finder, and
 * {@code HoughCircles} does the real work of deciding what is a ball. Hough votes on gradient
 * curvature, so a ball whose surface is broken up by wiffle holes, glare and shadow is still found
 * from its outer silhouette alone, and two touching balls are found as two independent circles —
 * both things a color-blob segmentation has to bolt on afterwards with watershed splitting and
 * circularity filters. Hough is colour-blind, though — it fits a silhouette regardless of what type
 * it belongs to — so the colour mask that seeded a circle's search region is also what assigns its
 * type (see {@link #colorFillFraction}).
 *
 * <p>Every detected ball is then fed to a {@link BallTracker}, which is what turns a stream of
 * unlabelled per-frame points into balls with identity and velocity; a track only ever matches
 * detections of its own {@link BallType}.
 *
 * <p>Every shared detection-recipe number lives in {@link BallVisionConstants}, the canonical copy
 * that {@code OpenCVPipelines/PollenDetectionPipeline/PollenDetectionPipeline.java} mirrors by hand
 * for EOCV-Sim (see that file's javadoc for why it can't just import this package).
 *
 * <p>The homography is fixed at construction from {@link BallVisionConstants#H_ARRAY} — there is no
 * live chessboard calibration here. Produce {@code H_ARRAY} with
 * {@code OpenCVPipelines/HomographyCalculationPipeline}, which locks a homography from a chessboard
 * and prints it paste-ready, and paste the result into {@code BallVisionConstants}.
 */
public class BallDetectionPipeline extends OpenCvPipeline {

    public enum DisplayMode {
        /** Candidate ROI masks, one colour per ball type, for tuning the HSV gates. */
        MASK,
        /** Camera image with circle, center and ground-contact overlays. */
        OVERLAY,
        /** Camera image with a bounding box and a field-coordinate label plate per ball. */
        BOX
    }

    /**
     * Display/behaviour toggles only — every actual detection-recipe number ({@link
     * BallVisionConstants.Detection} and the per-type HSV classes) lives on {@link
     * BallVisionConstants} itself and is read fresh every frame, so it needs no seeding here.
     *
     * <p>Named {@code BallVisionDisplay} rather than the old bare {@code BallVision}: this class used
     * to hold every detection knob before they moved to {@link BallVisionConstants}, and a dashboard
     * that had that wider {@code BallVision} category pinned or laid out from before the split can
     * keep reapplying its old snapshot over new edits. If a slider here still won't hold a value,
     * unpin/remove any saved {@code BallVision} layout on the dashboard and re-add the fields fresh
     * under this name.
     */
    @Config("BallVisionDisplay")
    public static class Tuning {
        public static DisplayMode displayMode = DisplayMode.BOX;
        public static boolean drawVelocity = true;
        /** Lookahead of the drawn velocity arrow, seconds. */
        public static double velocityArrowSeconds = 0.5;
    }

    private static final double DETECTION_SCALE = BallVisionConstants.DETECTION_SCALE;

    private static final Mat ROI_CLOSE_KERNEL = Imgproc.getStructuringElement(
            Imgproc.MORPH_ELLIPSE, BallVisionConstants.ROI_CLOSE_KERNEL_SIZE);

    private static final int ROI_PAD_PX = BallVisionConstants.ROI_PAD_PX;
    private static final int ROI_MERGE_DIST_PX = BallVisionConstants.ROI_MERGE_DIST_PX;

    private static final Size HOUGH_BLUR_KERNEL = BallVisionConstants.HOUGH_BLUR_KERNEL;
    private static final double HOUGH_MIN_DIST_FRACTION = BallVisionConstants.HOUGH_MIN_DIST_FRACTION;
    private static final double HOUGH_CANNY_MIN_THRESHOLD = BallVisionConstants.HOUGH_CANNY_MIN_THRESHOLD;
    private static final double HOUGH_MAX_RADIUS_FRACTION = BallVisionConstants.HOUGH_MAX_RADIUS_FRACTION;
    private static final double HOUGH_WORKING_MIN_RADIUS_PX = BallVisionConstants.HOUGH_WORKING_MIN_RADIUS_PX;
    private static final double ROI_MAX_UPSCALE = BallVisionConstants.ROI_MAX_UPSCALE;

    private static final double FPS_SMOOTHING = 0.1;
    private static final double NANOS_TO_SECONDS = 1e-9;

    private static final Scalar MASK_CANVAS_CLEAR = new Scalar(0, 0, 0);

    // Enum.values() defensively copies its backing array on every call; this loop runs 3x/frame.
    private static final BallType[] BALL_TYPES = BallType.values();

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
    private volatile Mat homography;
    private volatile Mat inverseHomography;
    private volatile boolean detectionEnabled = true;
    private volatile boolean trackerResetRequested = false;

    private final BallTracker tracker = new BallTracker();

    private final Mat small        = new Mat();
    private final Mat smallGray    = new Mat();
    private final Mat hsv          = new Mat();
    private final Mat rangeMask    = new Mat(); // one HsvRange, OR-ed into colorMask
    private final Mat colorMask    = new Mat(); // current type's raw colour mask
    private final Mat roiMask      = new Mat(); // lightly-closed candidate mask
    private final Mat roiWork      = new Mat();
    private final Mat houghCircles = new Mat(); // reused across every Hough call, all regions/types
    private final Mat maskCanvas   = new Mat(); // MASK mode: per-type masks, colour-coded
    private final Mat contourHierarchy = new Mat();
    // Draws in place onto that frame's input Mat rather than a separate copy — see render().
    private Mat display;

    // Mutated in place by applyRange() instead of allocating a new Scalar per HSV band per frame.
    private final Scalar rangeLow  = new Scalar(0, 0, 0);
    private final Scalar rangeHigh = new Scalar(0, 0, 0);

    // Camera resolution is fixed for the pipeline's lifetime, so this is computed once, not
    // reallocated every frame.
    private Size smallSize;

    private int rejectedColorCount = 0;
    private int rejectedOverlapCount = 0;
    private double fps = 0;
    private double lastFrameSeconds = Double.NaN;

    public BallDetectionPipeline() {
        setHomography(buildHomographyFromArray(BallVisionConstants.H_ARRAY));
    }

    // =========================================================================
    // Master pipeline
    // =========================================================================

    @Override
    public Mat processFrame(Mat input) {
        if (!detectionEnabled) return input;
        return runDetection(input);
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

        List<Rect> searchRegions = new ArrayList<>();
        List<Candidate> candidates = new ArrayList<>();

        if (Tuning.displayMode == DisplayMode.MASK) {
            maskCanvas.create(small.size(), CvType.CV_8UC3);
            maskCanvas.setTo(MASK_CANVAS_CLEAR);
        }

        for (BallType type : BALL_TYPES) {
            buildColorMask(type, colorMask);
            Imgproc.morphologyEx(colorMask, roiMask, Imgproc.MORPH_CLOSE, ROI_CLOSE_KERNEL);

            if (Tuning.displayMode == DisplayMode.MASK) maskCanvas.setTo(type.drawColor, roiMask);

            List<Rect> regions = buildSearchRegions();
            searchRegions.addAll(regions);
            candidates.addAll(findCircles(type, regions));
        }

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
        if (smallSize == null) {
            smallSize = new Size(input.cols() * DETECTION_SCALE, input.rows() * DETECTION_SCALE);
        }
        Imgproc.resize(input, small, smallSize, 0, 0, Imgproc.INTER_AREA);
        Imgproc.cvtColor(small, hsv, Imgproc.COLOR_RGB2HSV);
        Imgproc.cvtColor(small, smallGray, Imgproc.COLOR_RGB2GRAY);
    }

    /**
     * ORs every live-tunable HSV band of one ball type into {@code out}. The first band writes
     * {@code out} directly so the mask is cleared of the previous type's pixels without a separate
     * zeroing pass.
     */
    private void buildColorMask(BallType type, Mat out) {
        switch (type) {
            case POLLEN:
                applyRange(out, true,
                        PollenHsv.hLow, PollenHsv.sLow, PollenHsv.vLow,
                        PollenHsv.hHigh, PollenHsv.sHigh, PollenHsv.vHigh);
                applyRange(out, false,
                        PollenHsv.hLow, 0, PollenHsv.glareVLow,
                        PollenHsv.hHigh, PollenHsv.glareSHigh, 255);
                return;
            case NECTAR_RED:
                applyRange(out, true,
                        RedNectarHsv.hLow1, RedNectarHsv.sLow, RedNectarHsv.vLow,
                        RedNectarHsv.hHigh1, RedNectarHsv.sHigh, RedNectarHsv.vHigh);
                applyRange(out, false,
                        RedNectarHsv.hLow2, RedNectarHsv.sLow, RedNectarHsv.vLow,
                        RedNectarHsv.hHigh2, RedNectarHsv.sHigh, RedNectarHsv.vHigh);
                applyRange(out, false,
                        RedNectarHsv.hLow1, 0, RedNectarHsv.glareVLow,
                        RedNectarHsv.hHigh1, RedNectarHsv.glareSHigh, 255);
                applyRange(out, false,
                        RedNectarHsv.hLow2, 0, RedNectarHsv.glareVLow,
                        RedNectarHsv.hHigh2, RedNectarHsv.glareSHigh, 255);
                return;
            case NECTAR_BLUE:
                applyRange(out, true,
                        BlueNectarHsv.hLow, BlueNectarHsv.sLow, BlueNectarHsv.vLow,
                        BlueNectarHsv.hHigh, BlueNectarHsv.sHigh, BlueNectarHsv.vHigh);
                applyRange(out, false,
                        BlueNectarHsv.hLow, 0, BlueNectarHsv.glareVLow,
                        BlueNectarHsv.hHigh, BlueNectarHsv.glareSHigh, 255);
                return;
        }
        throw new IllegalStateException("Unhandled ball type: " + type);
    }

    private void applyRange(Mat out, boolean first,
                             double h0, double s0, double v0, double h1, double s1, double v1) {
        rangeLow.val[0] = h0;  rangeLow.val[1] = s0;  rangeLow.val[2] = v0;
        rangeHigh.val[0] = h1; rangeHigh.val[1] = s1; rangeHigh.val[2] = v1;
        if (first) {
            Core.inRange(hsv, rangeLow, rangeHigh, out);
        } else {
            Core.inRange(hsv, rangeLow, rangeHigh, rangeMask);
            Core.bitwise_or(out, rangeMask, out);
        }
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

    private List<Candidate> findCircles(BallType type, List<Rect> searchRegions) {
        double frameShortSide = Math.min(small.cols(), small.rows());
        double minBallRadius = frameShortSide * Detection.minRadiusFrameFraction * type.radiusScale;
        double maxBallRadius = frameShortSide * Detection.maxRadiusFrameFraction * type.radiusScale;

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
            double canny = Math.max(HOUGH_CANNY_MIN_THRESHOLD, Detection.houghCanny / upscale);

            Mat regionGray = smallGray.submat(region);
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

                // houghCircles is a reused instance field, not allocated per region/type/frame —
                // HoughCircles resizes its own backing buffer as needed, same as any other Mat output.
                Imgproc.HoughCircles(roiWork, houghCircles, Imgproc.HOUGH_GRADIENT,
                        Detection.houghDp, minDist * upscale,
                        canny, Detection.houghAccumulator,
                        (int) (minRadius * upscale), (int) (maxRadius * upscale));

                for (int i = 0; i < houghCircles.cols(); i++) {
                    double[] circle = houghCircles.get(0, i);
                    Candidate candidate = new Candidate(type,
                            circle[0] / upscale + region.x,
                            circle[1] / upscale + region.y,
                            circle[2] / upscale);
                    candidate.fillFraction = colorFillFraction(colorMask, candidate);
                    if (candidate.fillFraction >= Detection.minColorFill) candidates.add(candidate);
                    else rejectedColorCount++;
                }
            } finally {
                regionGray.release();
            }
        }
        return candidates;
    }

    /**
     * How much of a Hough circle's interior is actually the colour that seeded its ROI. Counted
     * over the circle's bounding box against the area a circle would occupy in that box (pi/4 of
     * it), so a frame-clipped ball is judged on its visible part rather than penalised for the
     * missing half. ROIs are padded and merged so they always contain off-colour margin, and
     * HOUGH_GRADIENT votes along the gradient normal in both directions — a real ball's edge
     * therefore also deposits a phantom center about one radius out into the background, and
     * nothing else downstream tells that phantom from the ball. This is also what assigns the
     * type, since Hough itself is colour-blind: a ball picked up through another colour's glare
     * band fails that colour's fill test.
     */
    private double colorFillFraction(Mat mask, Candidate candidate) {
        int x0 = (int) Math.max(0, Math.round(candidate.x - candidate.radius));
        int y0 = (int) Math.max(0, Math.round(candidate.y - candidate.radius));
        int x1 = (int) Math.min(mask.cols(), Math.round(candidate.x + candidate.radius));
        int y1 = (int) Math.min(mask.rows(), Math.round(candidate.y + candidate.radius));
        if (x1 - x0 < 1 || y1 - y0 < 1) return 0.0;

        Mat box = mask.submat(new Rect(x0, y0, x1 - x0, y1 - y0));
        try {
            double circleArea = (Math.PI / 4.0) * box.cols() * box.rows();
            return Core.countNonZero(box) / circleArea;
        } finally {
            box.release();
        }
    }

    /**
     * Greedy non-maximum suppression, largest first: a hole, glare ring or shadow inside a ball
     * fits a circle smaller than the ball's own outline, so preferring the larger radius keeps the
     * ball and drops the artifact. Hough's own minDist can't do this — it is one value per call,
     * derived from the smallest searched radius, and does nothing across separate ROI calls.
     *
     * <p>Runs over every type at once, not per type: the types' glare bands cover overlapping
     * near-white pixels, so one ball can seed a ROI under more than one colour and be found twice.
     * Colour fill breaks the tie, which is only reached when two circles are the same size — i.e.
     * when they really are the same ball.
     */
    private static List<Candidate> suppressOverlaps(List<Candidate> candidates) {
        Collections.sort(candidates, new Comparator<Candidate>() {
            @Override public int compare(Candidate a, Candidate b) {
                int byRadius = Double.compare(b.radius, a.radius);
                return byRadius != 0 ? byRadius : Double.compare(b.fillFraction, a.fillFraction);
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
                        || distance < Detection.minCenterSeparation * (k.radius + candidate.radius)) {
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
            detections.add(new BallDetection(c.type, field.get(i).x, field.get(i).y, c.x, c.y, c.radius));
        }
        return detections;
    }

    // =========================================================================
    // Rendering
    // =========================================================================

    private Mat render(Mat input, List<BallDetection> detections, List<TrackedBall> balls) {
        DisplayMode mode = Tuning.displayMode;

        // Draw straight onto input instead of a copy: nothing reads input as data past this point
        // (detection already ran on small/hsv/smallGray), and EasyOpenCV is fine getting back the
        // same Mat it handed us. Saves a full-resolution frame copy every loop in OVERLAY/BOX mode.
        display = input;
        if (mode == DisplayMode.MASK) {
            Imgproc.resize(maskCanvas, display, input.size(), 0, 0, Imgproc.INTER_NEAREST);
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

        Imgproc.circle(display, center, radius, ball.type.drawColor, 2);
        Imgproc.circle(display, center, 4, BallVisionConstants.COLOR_CENTER, -1);

        Point contact = new Point(center.x, center.y + radius);
        Imgproc.circle(display, contact, 5, BallVisionConstants.COLOR_CONTACT, -1);
        Imgproc.putText(display,
                String.format("%s (%.1f, %.1f)in", ball.type.label, ball.fieldX, ball.fieldY),
                new Point(contact.x + 8, contact.y),
                Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, ball.type.labelTextColor, 1);
    }

    private void drawBallBox(BallDetection ball, int index) {
        double scaleUp = 1.0 / DETECTION_SCALE;
        double cx = ball.imageX * scaleUp;
        double cy = ball.imageY * scaleUp;
        double radius = ball.imageRadius * scaleUp;

        Rect box = new Rect((int) (cx - radius), (int) (cy - radius),
                (int) (radius * 2), (int) (radius * 2));
        Imgproc.rectangle(display, new Point(box.x, box.y),
                new Point(box.x + box.width, box.y + box.height), ball.type.drawColor, 2);

        drawLabelPlate(String.format("#%d %s (%.1f, %.1f)in",
                index, ball.type.label, ball.fieldX, ball.fieldY), box, ball.type);
    }

    private void drawLabelPlate(String label, Rect box, BallType type) {
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
                new Point(plateX + plateWidth, plateY + plateHeight), type.drawColor, -1);
        Imgproc.putText(display, label,
                new Point(plateX + padding, plateY + plateHeight - padding - baseline[0]),
                fontFace, fontScale, type.labelTextColor, thickness);
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
            Imgproc.arrowedLine(display, from, to, BallVisionConstants.COLOR_VELOCITY, 2);
            Imgproc.putText(display,
                    String.format("#%d %.0fin/s", moving.get(i).id, moving.get(i).speed()),
                    new Point(to.x + 6, to.y), Imgproc.FONT_HERSHEY_SIMPLEX, 0.4,
                    BallVisionConstants.COLOR_VELOCITY, 1);
        }
    }

    private void drawOriginCrosshair() {
        Mat inverse = inverseHomography;
        if (inverse == null) return;

        Point origin = perspectiveTransform(
                Collections.singletonList(new Point(0, 0)), inverse).get(0);
        int size = 10;
        Scalar color = BallVisionConstants.COLOR_ORIGIN;

        Imgproc.line(display, new Point(origin.x - size, origin.y),
                new Point(origin.x + size, origin.y), color, 2);
        Imgproc.line(display, new Point(origin.x, origin.y - size),
                new Point(origin.x, origin.y + size), color, 2);
        Imgproc.circle(display, origin, 3, color, -1);
        Imgproc.putText(display, "(0,0)", new Point(origin.x + size + 4, origin.y),
                Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, color, 1);
    }

    // =========================================================================
    // Homography
    // =========================================================================

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

    /**
     * Always true once constructed — the homography is fixed from {@link
     * BallVisionConstants#H_ARRAY}, not calibrated live. Kept for callers written when this could
     * be false during live calibration.
     */
    public boolean isCalibrated() { return homography != null && !homography.empty(); }

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

    /** Diagnostic round-trip of {@link BallVisionConstants#H_ARRAY} — confirms what's actually
     * loaded, not a live calibration result. */
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
        final BallType type;
        final double x, y, radius;
        double fillFraction;

        Candidate(BallType type, double x, double y, double radius) {
            this.type = type;
            this.x = x;
            this.y = y;
            this.radius = radius;
        }
    }
}
