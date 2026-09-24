package org.firstinspires.ftc.teamcode.modules.vision;

import static org.firstinspires.ftc.teamcode.modules.vision.BallVisionConstants.DETECTION_SCALE;
import static org.firstinspires.ftc.teamcode.modules.vision.BallVisionConstants.HOUGH_BLUR_KERNEL;
import static org.firstinspires.ftc.teamcode.modules.vision.BallVisionConstants.HOUGH_CANNY_MIN_THRESHOLD;
import static org.firstinspires.ftc.teamcode.modules.vision.BallVisionConstants.HOUGH_MAX_RADIUS_FRACTION;
import static org.firstinspires.ftc.teamcode.modules.vision.BallVisionConstants.HOUGH_MIN_DIST_FRACTION;
import static org.firstinspires.ftc.teamcode.modules.vision.BallVisionConstants.HOUGH_WORKING_MIN_RADIUS_PX;
import static org.firstinspires.ftc.teamcode.modules.vision.BallVisionConstants.ROI_MAX_UPSCALE;
import static org.firstinspires.ftc.teamcode.modules.vision.BallVisionConstants.ROI_MERGE_DIST_PX;
import static org.firstinspires.ftc.teamcode.modules.vision.BallVisionConstants.ROI_PAD_PX;

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
import org.openftc.easyopencv.TimestampedOpenCvPipeline;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

/**
 * Finds Pollen (2.8in yellow) and Nectar (3.6in red/blue) balls and reports their field-inch
 * positions and velocities. HSV gates only seed ROIs; HoughCircles decides what is a ball, and the
 * colour mask assigns its type since Hough is colour-blind.
 */
public class BallDetectionPipeline extends TimestampedOpenCvPipeline {

    public enum DisplayMode {
        /** Per-type ROI masks, for tuning the HSV gates. */
        MASK,
        /** This frame's raw Hough detections, jitter and all. */
        OVERLAY,
        /** Tracked balls, smoothed with stable ids — what robot code consumes. */
        BOX
    }

    @Config("BallVisionDisplay")
    public static class Tuning {
        public static DisplayMode displayMode = DisplayMode.MASK;
        public static boolean drawVelocity = true;
        public static double velocityArrowSeconds = 0.5;
    }

    private static final Mat ROI_CLOSE_KERNEL = Imgproc.getStructuringElement(
            Imgproc.MORPH_ELLIPSE, BallVisionConstants.ROI_CLOSE_KERNEL_SIZE);

    private static final double FPS_SMOOTHING = 0.1;
    private static final double NANOS_TO_SECONDS = 1e-9;

    private static final Scalar MASK_CANVAS_CLEAR = new Scalar(0, 0, 0);

    // Enum.values() clones its array on every call.
    private static final BallType[] BALL_TYPES = BallType.values();

    /** Immutable per-frame results, published from the camera thread. */
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
    private volatile boolean detectionEnabled = true;
    private volatile boolean trackerResetRequested = false;

    private final BallTracker tracker = new BallTracker();
    private final Mat homography = buildHomographyFromArray(BallVisionConstants.H_ARRAY);
    private final Mat inverseHomography = homography.inv();

    private final Mat small        = new Mat();
    private final Mat smallGray    = new Mat();
    private final Mat hsv          = new Mat();
    private final Mat rangeMask    = new Mat();
    private final Mat colorMask    = new Mat();
    private final Mat roiMask      = new Mat();
    private final Mat roiWork      = new Mat();
    private final Mat houghCircles = new Mat();
    private final Mat maskCanvas   = new Mat();
    // Never resize into input: it is EasyOpenCV's persistent 4-channel decode buffer, and writing a
    // 3-channel Mat into it reallocates it and corrupts every later frame.
    private final Mat maskDisplay  = new Mat();
    private final Mat contourHierarchy = new Mat();
    private Mat display;

    private final Scalar rangeLow  = new Scalar(0, 0, 0);
    private final Scalar rangeHigh = new Scalar(0, 0, 0);

    private final MatOfPoint2f contourAsFloat = new MatOfPoint2f();
    private final Point maskCircleCenter = new Point();
    private final float[] maskCircleRadius = new float[1];

    private Size smallSize;

    private int rejectedColorCount = 0;
    private int rejectedOverlapCount = 0;
    private double fps = 0;
    private double lastFrameSeconds = Double.NaN;

    /** {@code captureTimeNanos} is on the System.nanoTime() clock, the one RobotStateHistory is stamped on. */
    @Override
    public Mat processFrame(Mat input, long captureTimeNanos) {
        if (!detectionEnabled) return input;
        return runDetection(input, captureTimeNanos * NANOS_TO_SECONDS);
    }

    private Mat runDetection(Mat input, double timestamp) {
        updateFps(timestamp);
        rejectedColorCount = 0;

        if (trackerResetRequested) {
            trackerResetRequested = false;
            tracker.reset();
        }

        prepareWorkingFrames(input);

        List<Rect> searchRegions = new ArrayList<>();
        List<Candidate> candidates = new ArrayList<>();

        // Read once: a mid-frame flip to MASK would have render() resize a maskCanvas this frame never filled.
        DisplayMode mode = Tuning.displayMode;
        if (mode == DisplayMode.MASK) {
            maskCanvas.create(small.size(), CvType.CV_8UC3);
            maskCanvas.setTo(MASK_CANVAS_CLEAR);
        }

        double frameShortSide = Math.min(small.cols(), small.rows());

        for (BallType type : BALL_TYPES) {
            buildColorMask(type, colorMask);
            Imgproc.morphologyEx(colorMask, roiMask, Imgproc.MORPH_CLOSE, ROI_CLOSE_KERNEL);

            if (mode == DisplayMode.MASK) maskCanvas.setTo(type.drawColor, roiMask);

            double minBallRadius = frameShortSide * Detection.minRadiusFrameFraction * type.radiusScale;
            double maxBallRadius = frameShortSide * Detection.maxRadiusFrameFraction * type.radiusScale;

            List<Candidate> maskCircles = new ArrayList<>();
            List<Rect> regions =
                    buildSearchRegions(type, minBallRadius, maxBallRadius, maskCircles);
            searchRegions.addAll(regions);

            List<Candidate> found = findCircles(type, regions, minBallRadius, maxBallRadius);
            candidates.addAll(found);
            candidates.addAll(unclaimedMaskCircles(maskCircles, found));
        }

        List<Candidate> circles = suppressOverlaps(candidates);
        rejectedOverlapCount = candidates.size() - circles.size();

        List<BallDetection> detections = projectToField(circles);
        List<TrackedBall> balls = tracker.update(detections, timestamp);

        latest = new Frame(detections, balls, searchRegions.size(),
                rejectedColorCount, rejectedOverlapCount, fps, timestamp);

        return render(input, detections, balls, mode);
    }

    private void prepareWorkingFrames(Mat input) {
        if (smallSize == null) {
            smallSize = new Size(input.cols() * DETECTION_SCALE, input.rows() * DETECTION_SCALE);
        }
        Imgproc.resize(input, small, smallSize, 0, 0, Imgproc.INTER_AREA);
        Imgproc.cvtColor(small, hsv, Imgproc.COLOR_RGB2HSV);
        Imgproc.cvtColor(small, smallGray, Imgproc.COLOR_RGB2GRAY);
    }

    /**
     * Each type's second band is its glare band: highlights wash out saturation and raise value but
     * keep hue. The first band overwrites {@code out}, which is what clears the previous type's pixels.
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
                        RedNectarHsv.hLow, RedNectarHsv.sLow, RedNectarHsv.vLow,
                        RedNectarHsv.hHigh, RedNectarHsv.sHigh, RedNectarHsv.vHigh);
                applyRange(out, false,
                        RedNectarHsv.hLow, 0, RedNectarHsv.glareVLow,
                        RedNectarHsv.hHigh, RedNectarHsv.glareSHigh, 255);
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

    /** Also collects round-enough mask blobs into {@code maskCircles} as Hough fallbacks. */
    private List<Rect> buildSearchRegions(BallType type, double minBallRadius, double maxBallRadius,
                                          List<Candidate> maskCircles) {
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(roiMask, contours, contourHierarchy,
                Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        List<Rect> padded = new ArrayList<>(contours.size());
        for (MatOfPoint contour : contours) {
            Rect bounds = Imgproc.boundingRect(contour);
            addMaskCircle(type, contour, minBallRadius, maxBallRadius, maskCircles);
            contour.release();
            int x = Math.max(0, bounds.x - ROI_PAD_PX);
            int y = Math.max(0, bounds.y - ROI_PAD_PX);
            int w = Math.min(roiMask.cols() - x, bounds.width + ROI_PAD_PX * 2);
            int h = Math.min(roiMask.rows() - y, bounds.height + ROI_PAD_PX * 2);
            if (w > 0 && h > 0) padded.add(new Rect(x, y, w, h));
        }
        return mergeNearbyRects(padded);
    }

    /**
     * Fallback for frames Hough misses: its accumulator is a hard threshold, so a borderline ball
     * flickers in and out even when its mask is steady.
     */
    private void addMaskCircle(BallType type, MatOfPoint contour,
                               double minBallRadius, double maxBallRadius, List<Candidate> out) {
        contour.convertTo(contourAsFloat, CvType.CV_32F);
        Imgproc.minEnclosingCircle(contourAsFloat, maskCircleCenter, maskCircleRadius);

        double radius = maskCircleRadius[0];
        if (radius < minBallRadius || radius > maxBallRadius) return;
        if (Imgproc.contourArea(contour) < Detection.maskCircleMinFill * Math.PI * radius * radius) {
            return;
        }

        Candidate candidate = new Candidate(type, maskCircleCenter.x, maskCircleCenter.y, radius);
        candidate.fillFraction = colorFillFraction(colorMask, candidate);
        if (candidate.fillFraction >= Detection.minColorFill) out.add(candidate);
    }

    private static List<Candidate> unclaimedMaskCircles(List<Candidate> maskCircles,
                                                        List<Candidate> hough) {
        if (maskCircles.isEmpty() || hough.isEmpty()) return maskCircles;

        List<Candidate> kept = new ArrayList<>(maskCircles.size());
        for (Candidate mask : maskCircles) {
            boolean covered = false;
            for (Candidate found : hough) {
                if (Math.hypot(mask.x - found.x, mask.y - found.y) <= mask.radius) {
                    covered = true;
                    break;
                }
            }
            if (!covered) kept.add(mask);
        }
        return kept;
    }

    private List<Candidate> findCircles(BallType type, List<Rect> searchRegions,
                                        double minBallRadius, double maxBallRadius) {
        // Never derived from the ROI's size: a bounding box wandering a pixel would change upscale
        // and Canny frame to frame and make detections blink.
        double minRadius = minBallRadius;
        double minDist = Math.max(4.0, minRadius * HOUGH_MIN_DIST_FRACTION * 2.0);
        double upscale = Math.min(ROI_MAX_UPSCALE,
                Math.max(1.0, HOUGH_WORKING_MIN_RADIUS_PX / minRadius));

        // Upscaling spreads each edge over `upscale` pixels, cutting gradient magnitude by that factor.
        double canny = Math.max(HOUGH_CANNY_MIN_THRESHOLD, Detection.houghCanny / upscale);

        List<Candidate> candidates = new ArrayList<>();
        for (Rect region : searchRegions) {
            int shortSide = Math.min(region.width, region.height);
            if (shortSide < 4) continue;

            // Only this ceiling may follow the region; the parameters above must not.
            double maxRadius = Math.min(shortSide * HOUGH_MAX_RADIUS_FRACTION, maxBallRadius);
            if (maxRadius <= minRadius) continue;

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
     * Mask fill over the frame-clipped bounding box (a circle covers pi/4 of it). Rejects the phantom
     * centers HOUGH_GRADIENT votes one radius outside real edges, and assigns type.
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
     * Largest-first NMS across all types: holes and glare fit smaller circles inside a ball, and the
     * types' overlapping glare bands can find one ball under two colours.
     */
    private static List<Candidate> suppressOverlaps(List<Candidate> candidates) {
        Collections.sort(candidates, new Comparator<Candidate>() {
            @Override public int compare(Candidate a, Candidate b) {
                // Whole-pixel radii so sub-pixel Hough jitter ties and the steadier colour fill decides;
                // an integer compare stays transitive where a tolerance wouldn't.
                int byRadius = Integer.compare((int) Math.round(b.radius), (int) Math.round(a.radius));
                return byRadius != 0 ? byRadius : Double.compare(b.fillFraction, a.fillFraction);
            }
        });

        List<Candidate> kept = new ArrayList<>(candidates.size());
        for (Candidate candidate : candidates) {
            boolean overlaps = false;
            for (Candidate k : kept) {
                double distance = Math.hypot(candidate.x - k.x, candidate.y - k.y);
                // Second test catches rim artifacts; touching balls sit at 1.0, so
                // minCenterSeparation must stay well under that.
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

    /** Projects each circle's ground-contact point (cx, cy + r) to field inches. */
    private List<BallDetection> projectToField(List<Candidate> circles) {
        if (circles.isEmpty()) return Collections.emptyList();

        List<Point> contacts = new ArrayList<>(circles.size());
        for (Candidate c : circles) {
            contacts.add(new Point(c.x / DETECTION_SCALE, (c.y + c.radius) / DETECTION_SCALE));
        }

        List<Point> field = perspectiveTransform(contacts, homography);
        List<BallDetection> detections = new ArrayList<>(circles.size());
        for (int i = 0; i < circles.size(); i++) {
            Candidate c = circles.get(i);
            detections.add(new BallDetection(c.type, field.get(i).x, field.get(i).y, c.x, c.y, c.radius));
        }
        return detections;
    }

    private Mat render(Mat input, List<BallDetection> detections, List<TrackedBall> balls,
                       DisplayMode mode) {
        if (mode == DisplayMode.MASK) {
            Imgproc.resize(maskCanvas, maskDisplay, input.size(), 0, 0, Imgproc.INTER_NEAREST);
            display = maskDisplay;
        } else {
            display = input;
        }

        if (mode == DisplayMode.BOX) {
            drawTrackedBoxes(balls);
        } else {
            for (int i = 0; i < detections.size(); i++) drawBallOverlay(detections.get(i));
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

    private void drawTrackedBoxes(List<TrackedBall> balls) {
        if (balls.isEmpty()) return;

        List<Point> contacts = new ArrayList<>(balls.size());
        for (TrackedBall ball : balls) contacts.add(ball.position());
        List<Point> pixels = perspectiveTransform(contacts, inverseHomography);

        double scaleUp = 1.0 / DETECTION_SCALE;
        for (int i = 0; i < balls.size(); i++) {
            TrackedBall ball = balls.get(i);
            Point contact = pixels.get(i);
            double radius = ball.radiusPx * scaleUp;

            // Track position is the ground-contact point, so the box sits above it.
            Rect box = new Rect((int) (contact.x - radius), (int) (contact.y - radius * 2),
                    (int) (radius * 2), (int) (radius * 2));
            Imgproc.rectangle(display, new Point(box.x, box.y),
                    new Point(box.x + box.width, box.y + box.height), ball.type.drawColor, 2);

            drawLabelPlate(String.format("#%d %s (%.1f, %.1f)in%s",
                    ball.id, ball.type.label, ball.x, ball.y,
                    ball.visible ? "" : " [coasting]"), box, ball.type);
        }
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
        if (balls.isEmpty()) return;

        List<TrackedBall> moving = new ArrayList<>();
        List<Point> fieldPoints = new ArrayList<>();
        for (TrackedBall ball : balls) {
            if (!ball.isMoving()) continue;
            moving.add(ball);
            fieldPoints.add(ball.position());
            fieldPoints.add(ball.predict(Tuning.velocityArrowSeconds));
        }
        if (moving.isEmpty()) return;

        List<Point> pixels = perspectiveTransform(fieldPoints, inverseHomography);
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
        Point origin = perspectiveTransform(
                Collections.singletonList(new Point(0, 0)), inverseHomography).get(0);
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

    private static Mat buildHomographyFromArray(double[][] values) {
        Mat h = new Mat(3, 3, CvType.CV_64F);
        for (int r = 0; r < 3; r++) {
            for (int c = 0; c < 3; c++) h.put(r, c, values[r][c]);
        }
        return h;
    }

    /** Never null; {@link Frame#EMPTY} until the first frame lands. */
    public Frame latest() { return latest; }

    /** Always true once constructed: the homography is fixed from {@link BallVisionConstants#H_ARRAY}. */
    public boolean isCalibrated() { return !homography.empty(); }

    public void setDetectionEnabled(boolean enabled) {
        if (detectionEnabled == enabled) return;
        detectionEnabled = enabled;
        latest = Frame.EMPTY;
        // Stale tracks would match whatever is in frame now and derive velocity across the unseen gap.
        resetTracking();
    }

    public void resetTracking() { trackerResetRequested = true; }

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
