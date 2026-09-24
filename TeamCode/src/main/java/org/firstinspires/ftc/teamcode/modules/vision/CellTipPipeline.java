package org.firstinspires.ftc.teamcode.modules.vision;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.easyopencv.TimestampedOpenCvPipeline;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * HIVE-cell tip verdict from the webcam: a cluster of this alliance's tags reading right-side up
 * ({@code |roll| < 90}) is scorable; upside-down or absent is tipped. The verdict starts tipped and
 * flips either way only after the new reading holds for {@link Tuning#holdSeconds}.
 */
public class CellTipPipeline extends TimestampedOpenCvPipeline {

    @Config("CellTipDetection")
    public static class Tuning {
        public static double holdSeconds = 0.25;
        public static double minTagAreaPx = 120;
        public static int minVisibleTags = 1;
        public static double scorableMaxRollDeg = 90;
        public static float decimation = 2;
        public static boolean drawOverlay = true;
    }

    /** Immutable; published from the camera thread. */
    public static final class Verdict {
        public static final Verdict NONE = new Verdict(true, 0);

        public final boolean tipped;
        /** On the System.nanoTime() clock; 0 until the first frame. */
        public final double timestampSeconds;

        Verdict(boolean tipped, double timestampSeconds) {
            this.tipped = tipped;
            this.timestampSeconds = timestampSeconds;
        }
    }

    private static final Scalar GREEN = new Scalar(0, 255, 0, 255);
    private static final Scalar RED = new Scalar(255, 0, 0, 255);
    private static final Scalar AMBER = new Scalar(255, 190, 0, 255);
    private static final Scalar GREY = new Scalar(150, 150, 150, 255);
    private static final Scalar WHITE = new Scalar(255, 255, 255, 255);
    private static final Scalar BLACK = new Scalar(0, 0, 0, 255);
    private static final int FONT = Imgproc.FONT_HERSHEY_SIMPLEX;

    // Only the corners are used; these just feed the detector's unused pose solve.
    private static final double TAG_SIZE_M = 0.1;
    private static final double FX = 578, FY = 578, CX = 320, CY = 240;

    private final String alliance;
    private final String[] clusterLabels;
    private final int[][] clusterIds;

    private final Mat gray = new Mat();
    // Never write into input: it is EasyOpenCV's persistent decode buffer.
    private final Mat display = new Mat();
    private final MatOfPoint quad = new MatOfPoint();
    private long detector;
    private float appliedDecimation;

    private volatile Verdict latest = Verdict.NONE;
    private volatile boolean enabled = true;

    // Camera-thread state: the debounce.
    private boolean tipped = true;
    private double pendingSince = Double.NaN;

    /** Cluster {@code i} is named {@code clusterLabels[i]} and holds the tag ids {@code clusterIds[i]}. */
    public CellTipPipeline(String alliance, String[] clusterLabels, int[][] clusterIds) {
        this.alliance = alliance;
        this.clusterLabels = clusterLabels;
        this.clusterIds = clusterIds;
        appliedDecimation = Tuning.decimation;
        detector = AprilTagDetectorJNI.createApriltagDetector(
                AprilTagDetectorJNI.TagFamily.TAG_36h11.string, appliedDecimation, 3);
    }

    @Override
    public Mat processFrame(Mat input, long captureTimeNanos) {
        if (!enabled) {
            tipped = true;
            pendingSince = Double.NaN;
            return input;
        }
        double now = captureTimeNanos * 1e-9;

        if (Tuning.decimation != appliedDecimation) {
            appliedDecimation = Tuning.decimation;
            AprilTagDetectorJNI.setApriltagDetectorDecimation(detector, appliedDecimation);
        }

        Imgproc.cvtColor(input, gray, Imgproc.COLOR_RGBA2GRAY);
        List<AprilTagDetection> detections = AprilTagDetectorJNI.runAprilTagDetectorSimple(
                detector, gray, TAG_SIZE_M, FX, FY, CX, CY);

        List<List<AprilTagDetection>> found = new ArrayList<>();
        for (int c = 0; c < clusterIds.length; c++) found.add(new ArrayList<AprilTagDetection>());
        for (AprilTagDetection d : detections) {
            int c = clusterOf(d.id);
            if (c < 0) continue;
            quad.fromArray(d.corners);
            if (Math.abs(Imgproc.contourArea(quad)) >= Tuning.minTagAreaPx) found.get(c).add(d);
        }

        double[] rolls = new double[clusterIds.length];
        boolean nowScorable = false;
        for (int c = 0; c < clusterIds.length; c++) {
            rolls[c] = meanRollDeg(found.get(c));
            if (found.get(c).size() >= Tuning.minVisibleTags && scorableRoll(rolls[c])) nowScorable = true;
        }

        if (nowScorable != tipped) {
            pendingSince = Double.NaN;
        } else if (Double.isNaN(pendingSince)) {
            pendingSince = now;
        }
        if (!Double.isNaN(pendingSince) && now - pendingSince >= Tuning.holdSeconds) {
            tipped = !nowScorable;
            pendingSince = Double.NaN;
        }
        latest = new Verdict(tipped, now);

        Imgproc.cvtColor(gray, display, Imgproc.COLOR_GRAY2RGBA);
        if (Tuning.drawOverlay) drawOverlay(found, rolls, nowScorable, now);
        return display;
    }

    /** Never null; {@link Verdict#NONE} until the first frame and while disabled. */
    public Verdict latest() {
        return latest;
    }

    public void setEnabled(boolean enabled) {
        if (this.enabled == enabled) return;
        this.enabled = enabled;
        latest = Verdict.NONE;
    }

    /** Call only once the camera has stopped streaming. */
    public void release() {
        if (detector == 0) return;
        AprilTagDetectorJNI.releaseApriltagDetector(detector);
        detector = 0;
    }

    private int clusterOf(int id) {
        for (int c = 0; c < clusterIds.length; c++) {
            for (int clusterId : clusterIds[c]) if (clusterId == id) return c;
        }
        return -1;
    }

    private static boolean scorableRoll(double roll) {
        return !Double.isNaN(roll) && Math.abs(roll) < Tuning.scorableMaxRollDeg;
    }

    /** Circular mean, so 179 and -179 average to 180, not 0; NaN for no tags. */
    private static double meanRollDeg(List<AprilTagDetection> tags) {
        double x = 0, y = 0;
        for (AprilTagDetection tag : tags) {
            double roll = tagRollRad(tag.corners);
            x += Math.cos(roll);
            y += Math.sin(roll);
        }
        if (x == 0 && y == 0) return Double.NaN;
        return Math.toDegrees(Math.atan2(y, x));
    }

    /** corners[0] to corners[1] is the tag's bottom edge, left to right: ~0 upright, ~180 upside-down. */
    private static double tagRollRad(Point[] corners) {
        return Math.atan2(corners[1].y - corners[0].y, corners[1].x - corners[0].x);
    }

    private void drawOverlay(List<List<AprilTagDetection>> found, double[] rolls, boolean nowScorable,
                             double now) {
        for (int c = 0; c < found.size(); c++) {
            Scalar color = scorableRoll(rolls[c]) ? GREEN : RED;
            for (AprilTagDetection tag : found.get(c)) {
                quad.fromArray(tag.corners);
                Imgproc.polylines(display, Collections.singletonList(quad), true, color, 2);
                label(String.valueOf(tag.id), new Point(tag.center.x - 10, tag.center.y), color, 0.5, 1);
            }
        }

        int y = 18;
        label(alliance + " cell tip", new Point(8, y), WHITE, 0.5, 1);
        for (int c = 0; c < clusterIds.length; c++) {
            y += 20;
            int[] ids = clusterIds[c];
            String range = String.format("%-8s %d-%d", clusterLabels[c], ids[0], ids[ids.length - 1]);
            List<AprilTagDetection> tags = found.get(c);
            if (tags.isEmpty()) {
                label(range + "  0/" + ids.length + "  --", new Point(8, y), GREY, 0.5, 1);
                continue;
            }
            boolean good = scorableRoll(rolls[c]);
            label(String.format("%s  %d/%d  roll %+.1f  %s", range, tags.size(), ids.length, rolls[c],
                    good ? "UP" : "DOWN"), new Point(8, y), good ? GREEN : RED, 0.5, 1);
        }

        String text = tipped ? "TIPPED" : "SCORABLE";
        Size size = Imgproc.getTextSize(text, FONT, 1.1, 3, new int[1]);
        y += 8;
        Imgproc.rectangle(display, new Point(4, y + 8), new Point(12 + size.width, y + 20 + size.height),
                tipped ? RED : GREEN, -1);
        Imgproc.putText(display, text, new Point(8, y + 14 + size.height), FONT, 1.1, tipped ? WHITE : BLACK, 3,
                Imgproc.LINE_AA);
        if (!Double.isNaN(pendingSince)) {
            label(String.format("reads %s for %.2fs / %.2fs", nowScorable ? "SCORABLE" : "TIPPED",
                    now - pendingSince, Tuning.holdSeconds),
                    new Point(8, y + 40 + size.height), AMBER, 0.5, 1);
        }
    }

    private void label(String text, Point origin, Scalar color, double scale, int thickness) {
        int[] baseline = new int[1];
        Size size = Imgproc.getTextSize(text, FONT, scale, thickness, baseline);
        Imgproc.rectangle(display, new Point(origin.x - 2, origin.y - size.height - 2),
                new Point(origin.x + size.width + 2, origin.y + baseline[0]), BLACK, -1);
        Imgproc.putText(display, text, origin, FONT, scale, color, thickness, Imgproc.LINE_AA);
    }
}
