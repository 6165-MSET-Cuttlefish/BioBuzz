package org.firstinspires.ftc.teamcode.eocvsim.balldetection;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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
 * Shape-first: HSV masks only seed ROIs and pick the ball type; HoughCircles fits the outer
 * silhouette, so holes and glare don't break detection. EOCV-Sim's workspace can't import TeamCode,
 * so its constants are separate from modules.vision.BallVisionConstants.
 */
public class PollenDetectionPipeline extends OpenCvPipeline {

    public enum DisplayMode { MASK, OVERLAY, BOX }
    private static final DisplayMode DISPLAY_MODE = DisplayMode.MASK;

    // Maps full-resolution image pixels to field inches.
    private static final double[][] H_ARRAY = {
            { -1.7797474624e-01, -5.3062009235e-02,  6.0413594965e+01 },
            { -2.0685716542e-02, -3.9378157948e-01,  1.4174826982e+02 },
            { -2.8668090956e-04, -1.2403394999e-02,  1.0000000000e+00 }
    };

    private static final double DETECTION_SCALE = 0.5;

    private static final double REFERENCE_BALL_DIAMETER_INCHES = 2.8;

    private static final class HsvRange {
        final Scalar low, high;

        HsvRange(double h0, double s0, double v0, double h1, double s1, double v1) {
            low  = new Scalar(h0, s0, v0);
            high = new Scalar(h1, s1, v1);
        }
    }

    // Each type's last HsvRange is its glare band: highlights drop S and raise V but keep hue.
    private enum BallType {
        POLLEN("Pollen", 2.8, new Scalar(255, 255, 0), new Scalar(0, 0, 0),
                new HsvRange( 15, 100, 100,  34, 255, 255),
                new HsvRange( 15,   0, 200,  34,  90, 255)),

        NECTAR_RED("Red Nectar", 3.6, new Scalar(255, 40, 40), new Scalar(255, 255, 255),
                new HsvRange(166, 145, 130, 179, 255, 255),
                new HsvRange(166,   0, 210, 179,  95, 255)),

        NECTAR_BLUE("Blue Nectar", 3.6, new Scalar(40, 120, 255), new Scalar(255, 255, 255),
                new HsvRange( 98, 110,  60, 130, 255, 255),
                new HsvRange( 98,   0, 190, 130,  90, 255));

        final String label;
        final Scalar drawColor;
        final Scalar labelTextColor;
        final HsvRange[] ranges;
        final double radiusScale;

        BallType(String label, double diameterInches, Scalar drawColor, Scalar labelTextColor,
                 HsvRange... ranges) {
            this.label = label;
            this.drawColor = drawColor;
            this.labelTextColor = labelTextColor;
            this.ranges = ranges;
            this.radiusScale = diameterInches / REFERENCE_BALL_DIAMETER_INCHES;
        }
    }

    // Light on purpose: Hough fits the outline, so the mask needs rough clusters, not solid discs.
    private static final Mat ROI_CLOSE_KERNEL = Imgproc.getStructuringElement(
            Imgproc.MORPH_ELLIPSE, new Size(9, 9));

    // Detection-scale px; generous since a ball's silhouette often extends past its lit-up mask.
    private static final int ROI_PAD_PX = 14;

    private static final int ROI_MERGE_DIST_PX = 20;

    // Applied per ROI after upscaling: on the small frame it erases a distant 3-5 px ball's edge.
    private static final Size HOUGH_BLUR_KERNEL = new Size(5, 5);

    private static final double HOUGH_DP = 1.2;

    private static final double HOUGH_MIN_DIST_FRACTION = 0.5;

    private static final double HOUGH_CANNY_THRESHOLD = 80;

    // Floor for HOUGH_CANNY_THRESHOLD after dividing by ROI upscale; below it, noise makes edges.
    private static final double HOUGH_CANNY_MIN_THRESHOLD = 30;

    private static final double HOUGH_ACCUMULATOR_THRESHOLD = 22;

    private static final double HOUGH_MAX_RADIUS_FRACTION = 0.60;

    // Frame-relative, for a REFERENCE_BALL_DIAMETER_INCHES ball: ROIs are floored at 2 * ROI_PAD_PX,
    // so an ROI-relative minimum would exclude every distant ball.
    private static final double MIN_BALL_RADIUS_FRAME_FRACTION = 0.02;
    private static final double MAX_BALL_RADIUS_FRAME_FRACTION = 0.1;

    // Hough votes scale with circumference, so ROIs whose min radius is below this are upscaled.
    private static final double HOUGH_WORKING_MIN_RADIUS_PX = 6.0;

    private static final double ROI_MAX_UPSCALE = 4.0;

    // HOUGH_GRADIENT votes both ways, so a real edge leaves a phantom centre ~1 radius outside;
    // this rejects it and assigns the type. Low: holes and glare mean a ball is never fully masked.
    private static final double MIN_COLOR_FILL_FRACTION = 0.30;

    // Of r1 + r2 (touching balls sit at 1.0); Hough's minDist is per call and too small for this.
    private static final double MIN_CENTER_SEPARATION_FRACTION = 0.7;

    private final Mat homography = buildHomographyFromArray(H_ARRAY);
    private final Mat inverseHomography = homography.inv();

    private final Mat small        = new Mat();
    private final Mat smallGray    = new Mat();
    private final Mat roiWork      = new Mat();
    private final Mat hsv          = new Mat();
    private final Mat rangeMask    = new Mat();
    private final Mat colorMask    = new Mat();
    private final Mat roiMask      = new Mat();
    private final Mat displayImage = new Mat();
    private final Mat maskCanvas   = new Mat();
    private final Mat contourHierarchy = new Mat();

    private final Telemetry telemetry;

    private static final Scalar MASK_CANVAS_CLEAR = new Scalar(0, 0, 0);

    private static class BallResult {
        BallType type;
        double centerXSmall, centerYSmall;
        double radiusSmall;
        double fillFraction;
        Point fieldPoint; // ground-contact point, field inches
    }

    public PollenDetectionPipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {
        Size smallSize = new Size(input.cols() * DETECTION_SCALE, input.rows() * DETECTION_SCALE);
        Imgproc.resize(input, small, smallSize, 0, 0, Imgproc.INTER_AREA);

        Imgproc.cvtColor(small, hsv, Imgproc.COLOR_RGB2HSV);
        Imgproc.cvtColor(small, smallGray, Imgproc.COLOR_RGB2GRAY);

        if (DISPLAY_MODE == DisplayMode.MASK) {
            maskCanvas.create(small.size(), CvType.CV_8UC3);
            maskCanvas.setTo(MASK_CANVAS_CLEAR);
        } else {
            input.copyTo(displayImage);
        }

        double frameShortSide = Math.min(small.cols(), small.rows());

        List<BallResult> candidates = new ArrayList<>();
        int rejectedCount = 0;
        int roiCount = 0;

        for (BallType type : BallType.values()) {
            buildColorMask(type, colorMask);

            Imgproc.morphologyEx(colorMask, roiMask, Imgproc.MORPH_CLOSE, ROI_CLOSE_KERNEL);

            if (DISPLAY_MODE == DisplayMode.MASK) maskCanvas.setTo(type.drawColor, roiMask);

            List<Rect> rois = findMergedRois(roiMask);
            roiCount += rois.size();

            double minBallRadiusSmall = minBallRadius(frameShortSide, type);
            double maxBallRadiusSmall = maxBallRadius(frameShortSide, type);

            for (Rect roi : rois) {
                int shortSide = Math.min(roi.width, roi.height);
                if (shortSide < 4) continue;

                double maxRadius = Math.min(shortSide * HOUGH_MAX_RADIUS_FRACTION, maxBallRadiusSmall);
                double minRadius = Math.min(minBallRadiusSmall, maxRadius * 0.5);
                double minDist   = Math.max(4.0, minRadius * HOUGH_MIN_DIST_FRACTION * 2.0);

                double roiScale = Math.min(ROI_MAX_UPSCALE,
                        Math.max(1.0, HOUGH_WORKING_MIN_RADIUS_PX / minRadius));

                // Upscaling spreads each edge over roiScale px, cutting gradient magnitude to match.
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
                        // { centerX, centerY, radius } in upscaled-ROI-local px
                        BallResult result = new BallResult();
                        result.type = type;
                        result.centerXSmall = c[0] / roiScale + roi.x;
                        result.centerYSmall = c[1] / roiScale + roi.y;
                        result.radiusSmall  = c[2] / roiScale;
                        result.fillFraction = colorFillFraction(colorMask, result);

                        if (result.fillFraction < MIN_COLOR_FILL_FRACTION) {
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
        }

        List<BallResult> results = suppressOverlaps(candidates);
        int overlapCount = candidates.size() - results.size();

        for (BallResult result : results) {
            // The homography maps the floor plane, so transform the ground contact, not the centre.
            double contactXSmall = result.centerXSmall;
            double contactYSmall = result.centerYSmall + result.radiusSmall;

            double fullResX = contactXSmall / DETECTION_SCALE;
            double fullResY = contactYSmall / DETECTION_SCALE;
            result.fieldPoint = perspectiveTransform(fullResX, fullResY, homography);
        }

        if (DISPLAY_MODE == DisplayMode.MASK) {
            Imgproc.resize(maskCanvas, displayImage, input.size(), 0, 0, Imgproc.INTER_NEAREST);
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

        telemetry.addLine("[Detecting Balls — Hough circle fit]");
        telemetry.addData("ROIs searched", roiCount);
        telemetry.addData("Balls Detected", results.size());
        for (BallType type : BallType.values()) {
            telemetry.addData("  " + type.label, String.format("%d  (r %.1f - %.1f px)",
                    countOfType(results, type),
                    minBallRadius(frameShortSide, type),
                    maxBallRadius(frameShortSide, type)));
        }
        telemetry.addData("Rejected (colour fill)", rejectedCount);
        telemetry.addData("Rejected (overlap)", overlapCount);
        for (int i = 0; i < results.size(); i++) {
            BallResult r = results.get(i);
            telemetry.addLine("--- Ball " + i + ": " + r.type.label + " ---");
            telemetry.addData("  Field X (in)", String.format("%.2f", r.fieldPoint.x));
            telemetry.addData("  Field Y (in)", String.format("%.2f", r.fieldPoint.y));
            telemetry.addData("  Radius (px)", String.format("%.1f", r.radiusSmall));
            telemetry.addData("  Colour fill", String.format("%.0f%%", r.fillFraction * 100.0));
        }
        telemetry.update();

        return displayImage;
    }

    /** The first band writes {@code out} directly, clearing the previous type's pixels. */
    private void buildColorMask(BallType type, Mat out) {
        for (int i = 0; i < type.ranges.length; i++) {
            HsvRange range = type.ranges[i];
            if (i == 0) {
                Core.inRange(hsv, range.low, range.high, out);
            } else {
                Core.inRange(hsv, range.low, range.high, rangeMask);
                Core.bitwise_or(out, rangeMask, out);
            }
        }
    }

    private static double minBallRadius(double frameShortSide, BallType type) {
        return frameShortSide * MIN_BALL_RADIUS_FRAME_FRACTION * type.radiusScale;
    }

    private static double maxBallRadius(double frameShortSide, BallType type) {
        return frameShortSide * MAX_BALL_RADIUS_FRAME_FRACTION * type.radiusScale;
    }

    private static int countOfType(List<BallResult> results, BallType type) {
        int n = 0;
        for (BallResult r : results) if (r.type == type) n++;
        return n;
    }

    /**
     * Largest first, since holes, glare and shadow fit circles smaller than the ball. Runs across
     * all types because the glare bands overlap, so one ball can be found under several colours.
     */
    private static List<BallResult> suppressOverlaps(List<BallResult> candidates) {
        Collections.sort(candidates, new Comparator<BallResult>() {
            @Override public int compare(BallResult a, BallResult b) {
                int byRadius = Double.compare(b.radiusSmall, a.radiusSmall);
                return byRadius != 0 ? byRadius : Double.compare(b.fillFraction, a.fillFraction);
            }
        });

        List<BallResult> kept = new ArrayList<>(candidates.size());
        for (BallResult c : candidates) {
            boolean overlaps = false;
            for (BallResult k : kept) {
                double dx = c.centerXSmall - k.centerXSmall;
                double dy = c.centerYSmall - k.centerYSmall;
                double dist = Math.sqrt(dx * dx + dy * dy);

                // Second test catches an artifact near a kept ball's rim, centred just outside it.
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

    /** Normalised by the frame-clipped bounding box, so an edge-clipped ball isn't penalised. */
    private double colorFillFraction(Mat mask, BallResult r) {
        int x0 = (int) Math.max(0, Math.round(r.centerXSmall - r.radiusSmall));
        int y0 = (int) Math.max(0, Math.round(r.centerYSmall - r.radiusSmall));
        int x1 = (int) Math.min(mask.cols(), Math.round(r.centerXSmall + r.radiusSmall));
        int y1 = (int) Math.min(mask.rows(), Math.round(r.centerYSmall + r.radiusSmall));
        if (x1 - x0 < 1 || y1 - y0 < 1) return 0.0;

        Mat box = mask.submat(new Rect(x0, y0, x1 - x0, y1 - y0));
        try {
            double circleArea = (Math.PI / 4.0) * box.cols() * box.rows();
            return Core.countNonZero(box) / circleArea;
        } finally {
            box.release();
        }
    }

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

        Imgproc.circle(displayImage, centerFull, radiusFull, r.type.drawColor, 2);
        Imgproc.circle(displayImage, centerFull, 4, new Scalar(255, 255, 255), -1);

        Point contactFull = new Point(centerFull.x, centerFull.y + radiusFull);
        Imgproc.circle(displayImage, contactFull, 5, new Scalar(0, 0, 255), -1);

        String label = String.format("%s (%.1f, %.1f)in",
                r.type.label, r.fieldPoint.x, r.fieldPoint.y);
        Imgproc.putText(displayImage, label,
                new Point(contactFull.x + 8, contactFull.y),
                Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255, 255, 255), 1);
    }

    private void drawBallBox(Mat displayImage, BallResult r, int ballIndex) {
        double scaleUp = 1.0 / DETECTION_SCALE;

        double cxFull = r.centerXSmall * scaleUp;
        double cyFull = r.centerYSmall * scaleUp;
        double radiusFull = r.radiusSmall * scaleUp;

        Rect box = new Rect(
                (int) (cxFull - radiusFull), (int) (cyFull - radiusFull),
                (int) (radiusFull * 2), (int) (radiusFull * 2));

        Scalar boxColor  = r.type.drawColor;
        Scalar textColor = r.type.labelTextColor;

        Imgproc.rectangle(displayImage,
                new Point(box.x, box.y),
                new Point(box.x + box.width, box.y + box.height),
                boxColor, 2);

        String label = String.format("#%d %s (%.1f, %.1f)in",
                ballIndex, r.type.label, r.fieldPoint.x, r.fieldPoint.y);

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

    private static Point perspectiveTransform(double x, double y, Mat transform) {
        MatOfPoint2f src = new MatOfPoint2f(new Point(x, y));
        MatOfPoint2f dst = new MatOfPoint2f();
        try {
            Core.perspectiveTransform(src, dst, transform);
            return dst.toArray()[0];
        } finally {
            src.release();
            dst.release();
        }
    }

    private void drawOriginCrosshair(Mat displayImage) {
        Point originPixel = perspectiveTransform(0.0, 0.0, inverseHomography);

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

    private static Mat buildHomographyFromArray(double[][] arr) {
        Mat h = new Mat(3, 3, CvType.CV_64F);
        for (int r = 0; r < 3; r++)
            for (int c = 0; c < 3; c++)
                h.put(r, c, arr[r][c]);
        return h;
    }
}