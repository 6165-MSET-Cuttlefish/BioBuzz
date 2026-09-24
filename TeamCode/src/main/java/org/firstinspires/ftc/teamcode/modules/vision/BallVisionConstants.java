package org.firstinspires.ftc.teamcode.modules.vision;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Scalar;
import org.opencv.core.Size;

/**
 * Ball-vision constants; the {@code @Config} classes are read live every frame.
 */
public final class BallVisionConstants {

    private BallVisionConstants() {}

    @Config("BallVision_Detection")
    public static final class Detection {
        private Detection() {}
        public static double houghDp = 1.2;
        public static double houghCanny = 80;
        public static double houghAccumulator = 22;
        /** Fraction of the frame's shorter side, for a Pollen-sized ball. */
        public static double minRadiusFrameFraction = 0.02;
        public static double maxRadiusFrameFraction = 0.10;
        /** Fraction of a circle's own area that must be its type's mask pixels to count as a ball. */
        public static double minColorFill = 0.30;
        /** Fraction of the two radii summed. */
        public static double minCenterSeparation = 0.7;
        /**
         * Min fill of a mask blob's enclosing circle for it to stand in for Hough on frames Hough
         * misses; keeps a ball near the accumulator threshold from flickering.
         */
        public static double maskCircleMinFill = 0.60;
    }

    public static final double REFERENCE_BALL_DIAMETER_INCHES = 2.8;

    @Config("BallVision_Pollen")
    public static final class PollenHsv {
        private PollenHsv() {}
        public static int hLow = 0, hHigh = 30;
        public static int sLow = 50, sHigh = 255;
        public static int vLow = 150, vHigh = 255;
        public static int glareSHigh = 60, glareVLow = 220;
    }

    /** Red sits at the 179 end of this HSV space, not straddling 0, so one band covers it. */
    @Config("BallVision_RedNectar")
    public static final class RedNectarHsv {
        private RedNectarHsv() {}
        public static int hLow = 166, hHigh = 179;
        public static int sLow = 145, sHigh = 255;
        public static int vLow = 130, vHigh = 255;
        public static int glareSHigh = 95, glareVLow = 210;
    }

    @Config("BallVision_BlueNectar")
    public static final class BlueNectarHsv {
        private BlueNectarHsv() {}
        public static int hLow = 105, hHigh = 123;
        public static int sLow = 140, sHigh = 255;
        public static int vLow = 115, vHigh = 255;
        public static int glareSHigh = 100, glareVLow = 200;
    }

    public enum BallType {
        POLLEN("Pollen", 2.8, new Scalar(255, 255, 0), new Scalar(0, 0, 0)),
        NECTAR_RED("Red Nectar", 3.6, new Scalar(255, 40, 40), new Scalar(255, 255, 255)),
        NECTAR_BLUE("Blue Nectar", 3.6, new Scalar(40, 120, 255), new Scalar(255, 255, 255));

        public final String label;
        public final Scalar drawColor;
        public final Scalar labelTextColor;
        public final double radiusScale;

        BallType(String label, double diameterInches, Scalar drawColor, Scalar labelTextColor) {
            this.label = label;
            this.drawColor = drawColor;
            this.labelTextColor = labelTextColor;
            this.radiusScale = diameterInches / REFERENCE_BALL_DIAMETER_INCHES;
        }
    }

    // Full-resolution image pixels to camera-frame ground inches; calibrate at 640x480, the size WebcamSession streams.
    public static final double[][] H_ARRAY = {
            { -6.8658673540e-02, -1.4582606197e-02, 2.5633211718e+01 },
            { -8.0700352292e-04, -2.1452449123e-01, 6.3092382406e+01 },
            { -1.8726593163e-04, -7.7392061899e-03, 1.0000000000e+00 }
    };

    /** Points are scaled back to full resolution before the homography, so ground coordinates don't depend on this. */
    public static final double DETECTION_SCALE = 0.5;

    /** Light on purpose: it only merges nearby fragments into a cluster, it doesn't rebuild a solid disc. */
    public static final Size ROI_CLOSE_KERNEL_SIZE = new Size(9, 9);

    /** A ball's silhouette routinely extends past where its color mask lit up, so pad generously. */
    public static final int ROI_PAD_PX = 14;
    public static final int ROI_MERGE_DIST_PX = 20;

    /** Applied per-ROI after upscaling; on the downscaled frame a 5x5 blur erases a small ball's edge. */
    public static final Size HOUGH_BLUR_KERNEL = new Size(5, 5);

    /** Floor for {@link Detection#houghCanny} once divided down for an upscaled ROI. */
    public static final double HOUGH_CANNY_MIN_THRESHOLD = 30;
    /** Fraction of the ROI's shorter side. */
    public static final double HOUGH_MAX_RADIUS_FRACTION = 0.60;
    /** Fraction of the min searched radius. */
    public static final double HOUGH_MIN_DIST_FRACTION = 0.5;

    /** Hough votes scale with circumference, so ROIs searching below this radius are upscaled first. */
    public static final double HOUGH_WORKING_MIN_RADIUS_PX = 6.0;
    public static final double ROI_MAX_UPSCALE = 4.0;

    public static final Scalar COLOR_CENTER   = new Scalar(255, 255, 255);
    public static final Scalar COLOR_CONTACT  = new Scalar(0, 0, 255);
    public static final Scalar COLOR_VELOCITY = new Scalar(255, 128, 0);
    public static final Scalar COLOR_ORIGIN   = new Scalar(255, 0, 255);
}
