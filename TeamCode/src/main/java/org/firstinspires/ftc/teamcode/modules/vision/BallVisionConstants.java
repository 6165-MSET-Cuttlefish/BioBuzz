package org.firstinspires.ftc.teamcode.modules.vision;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Scalar;
import org.opencv.core.Size;

/**
 * Every detection-recipe constant shared between the on-robot {@link BallDetectionPipeline} and its
 * EOCV-Sim-facing copy, {@code OpenCVPipelines/PollenDetectionPipeline/PollenDetectionPipeline.java}.
 * This is the canonical copy — the sim pipeline mirrors these values by hand (EOCV-Sim's workspace
 * compiles that file in isolation and can't resolve this package's imports), so a change here needs
 * the same change pasted into the sim file's matching constants.
 *
 * <p>The live-tunable numbers ({@link Detection}, {@link PollenHsv}, {@link RedNectarHsv},
 * {@link BlueNectarHsv}) are {@code @Config}-bound and read directly by {@link BallDetectionPipeline}
 * every frame, so editing them on FtcDashboard takes effect immediately with no separate seed/copy
 * step — this file <em>is</em> the dashboard-tunable state, not just its defaults. Everything else
 * here (the {@link BallType} enum, {@link #H_ARRAY}, kernel sizes, and the ROI/Hough constants that
 * were never dashboard knobs even in the single-color pipeline this replaced) stays a plain
 * {@code public static final} constant: dashboard config only reflects on primitive/String/enum
 * fields of a {@code public static} (non-{@code final}) field on an {@code @Config}-annotated class,
 * and most of what else lives here (OpenCV {@code Scalar}/{@code Size}, arrays, the enum itself)
 * either isn't a type dashboard supports or isn't something that should change at runtime.
 */
public final class BallVisionConstants {

    private BallVisionConstants() {}

    /** Detection knobs shared by every ball type. */
    @Config("BallVision_Detection")
    public static final class Detection {
        private Detection() {}
        public static double houghDp = 1.2;
        /** Canny high threshold; lower catches faint outlines but votes for more noise. */
        public static double houghCanny = 80;
        /** Accumulator votes needed to report a circle. */
        public static double houghAccumulator = 22;
        /** Ball radius bounds as a fraction of the frame's shorter side, for a Pollen-sized ball. */
        public static double minRadiusFrameFraction = 0.02;
        public static double maxRadiusFrameFraction = 0.10;
        /** Fraction of a circle's own area that must be its type's mask pixels to count as a ball. */
        public static double minColorFill = 0.30;
        /** Min center separation as a fraction of the two radii summed. */
        public static double minCenterSeparation = 0.7;
    }

    // -------------------------------------------------------------------------
    // Ball types. The field carries 2.8 in yellow Pollen and 3.6 in red and blue
    // Nectar. Each type gets its own HSV mask, scaled radius search window, and
    // display colour; Hough itself is colour-blind, so the mask that seeded a
    // circle's ROI is also what assigns its type (see Detection.minColorFill).
    //
    // Each *Hsv class below lists its colour band(s) first and its glare
    // band(s) last: a specular highlight washes saturation out and drives value
    // up while leaving hue roughly in place. Red straddles the hue origin, so
    // both of its bands are split in two (hLow1/hHigh1, hLow2/hHigh2). These are
    // the live @Config fields BallDetectionPipeline's mask-building reads every
    // frame; BallType.defaultRanges below is a frozen snapshot of them taken at
    // class-load time, wrapped as Scalars for the sim copy and anything else
    // that just wants "the ball-type recipe" without per-field dashboard tuning.
    // -------------------------------------------------------------------------
    public static final double REFERENCE_BALL_DIAMETER_INCHES = 2.8;

    /** OpenCV HSV: H 0-179, S/V 0-255. Loose on purpose — Hough is the real shape gate. */
    @Config("BallVision_Pollen")
    public static final class PollenHsv {
        private PollenHsv() {}
        public static int hLow = 16, hHigh = 30;
        public static int sLow = 115, sHigh = 255;
        public static int vLow = 150, vHigh = 255;
        public static int glareSHigh = 60, glareVLow = 220;
    }

    /** Red straddles the hue origin, so its colour band is split into two (hLow1..hHigh2). */
    @Config("BallVision_RedNectar")
    public static final class RedNectarHsv {
        private RedNectarHsv() {}
        public static int hLow1 = 0, hHigh1 = 8;
        public static int hLow2 = 166, hHigh2 = 179;
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

    public static final class HsvRange {
        public final Scalar low, high;

        public HsvRange(double h0, double s0, double v0, double h1, double s1, double v1) {
            low  = new Scalar(h0, s0, v0);
            high = new Scalar(h1, s1, v1);
        }
    }

    public enum BallType {
        POLLEN("Pollen", 2.8, new Scalar(255, 255, 0), new Scalar(0, 0, 0),
                new HsvRange(PollenHsv.hLow, PollenHsv.sLow, PollenHsv.vLow,
                        PollenHsv.hHigh, PollenHsv.sHigh, PollenHsv.vHigh),
                new HsvRange(PollenHsv.hLow, 0, PollenHsv.glareVLow,
                        PollenHsv.hHigh, PollenHsv.glareSHigh, 255)),

        NECTAR_RED("Red Nectar", 3.6, new Scalar(255, 40, 40), new Scalar(255, 255, 255),
                new HsvRange(RedNectarHsv.hLow1, RedNectarHsv.sLow, RedNectarHsv.vLow,
                        RedNectarHsv.hHigh1, RedNectarHsv.sHigh, RedNectarHsv.vHigh),
                new HsvRange(RedNectarHsv.hLow2, RedNectarHsv.sLow, RedNectarHsv.vLow,
                        RedNectarHsv.hHigh2, RedNectarHsv.sHigh, RedNectarHsv.vHigh),
                new HsvRange(RedNectarHsv.hLow1, 0, RedNectarHsv.glareVLow,
                        RedNectarHsv.hHigh1, RedNectarHsv.glareSHigh, 255),
                new HsvRange(RedNectarHsv.hLow2, 0, RedNectarHsv.glareVLow,
                        RedNectarHsv.hHigh2, RedNectarHsv.glareSHigh, 255)),

        NECTAR_BLUE("Blue Nectar", 3.6, new Scalar(40, 120, 255), new Scalar(255, 255, 255),
                new HsvRange(BlueNectarHsv.hLow, BlueNectarHsv.sLow, BlueNectarHsv.vLow,
                        BlueNectarHsv.hHigh, BlueNectarHsv.sHigh, BlueNectarHsv.vHigh),
                new HsvRange(BlueNectarHsv.hLow, 0, BlueNectarHsv.glareVLow,
                        BlueNectarHsv.hHigh, BlueNectarHsv.glareSHigh, 255));

        public final String label;
        public final Scalar drawColor;
        public final Scalar labelTextColor;
        /** Snapshot of this type's HSV bands at class-load time — see the class javadoc above. */
        public final HsvRange[] defaultRanges;
        public final double radiusScale;

        BallType(String label, double diameterInches, Scalar drawColor, Scalar labelTextColor,
                 HsvRange... defaultRanges) {
            this.label = label;
            this.drawColor = drawColor;
            this.labelTextColor = labelTextColor;
            this.defaultRanges = defaultRanges;
            this.radiusScale = diameterInches / REFERENCE_BALL_DIAMETER_INCHES;
        }
    }

    // -------------------------------------------------------------------------
    // Homography: full-resolution image pixels to field-coordinate inches. Both
    // pipelines calibrate the same physical camera mount, so this is one number
    // shared between them, not per-pipeline tuning.
    // -------------------------------------------------------------------------
    public static final double[][] H_ARRAY = {
            { -6.8658673540e-02, -1.4582606197e-02, 2.5633211718e+01 },
            { -8.0700352292e-04, -2.1452449123e-01, 6.3092382406e+01 },
            { -1.8726593163e-04, -7.7392061899e-03, 1.0000000000e+00 }
    };

    /**
     * Detection downscale factor. Still the single biggest performance lever: ROI-finding scales
     * with pixel count, and a smaller frame means smaller (cheaper) Hough search regions too.
     * Detected points are scaled back to full resolution before the homography is applied, so
     * reported field coordinates are unaffected by this value.
     */
    public static final double DETECTION_SCALE = 0.5;

    /**
     * Light close, unlike a color-first pipeline's aggressive multi-pass fill: this only has to
     * merge nearby fragments into a rough cluster, not rebuild a solid disc.
     */
    public static final Size ROI_CLOSE_KERNEL_SIZE = new Size(9, 9);

    /** A ball's silhouette routinely extends past where its color mask lit up, so pad generously. */
    public static final int ROI_PAD_PX = 14;
    /** Fragments of one ball that survived as separate blobs get merged into a single search box. */
    public static final int ROI_MERGE_DIST_PX = 20;

    /**
     * Blur before Hough — holes and glare are exactly the high-frequency noise that wrecks gradient
     * analysis. Applied per-ROI after the upscale below, because a 5x5 blur applied to the small
     * frame erases the entire edge of a 3-5px-radius ball.
     */
    public static final Size HOUGH_BLUR_KERNEL = new Size(5, 5);

    /** Floor for {@link Detection#houghCanny} once divided down for an upscaled ROI. */
    public static final double HOUGH_CANNY_MIN_THRESHOLD = 30;
    /** Upper bound on a searched circle relative to its own ROI's shorter dimension. */
    public static final double HOUGH_MAX_RADIUS_FRACTION = 0.60;
    /** Min center distance Hough enforces within one call, as a fraction of the min searched radius. */
    public static final double HOUGH_MIN_DIST_FRACTION = 0.5;

    /**
     * Hough's votes scale with circumference, so a distant 4px-radius ball has ~25 edge pixels to
     * clear the accumulator threshold with while a near ball clears it trivially. ROIs searching
     * below this radius are upscaled first so small circles get a proportionate vote count.
     */
    public static final double HOUGH_WORKING_MIN_RADIUS_PX = 6.0;
    /** Bounds the cost of that upscale; Hough is O(pixels). */
    public static final double ROI_MAX_UPSCALE = 4.0;

    // -------------------------------------------------------------------------
    // Homography calibration board. GRID_COLS=9, GRID_ROWS=6 matches the
    // physical board (9 inner corners wide, 6 inner corners tall).
    // -------------------------------------------------------------------------
    public static final int GRID_COLS = 9;
    public static final int GRID_ROWS = 6;
    public static final int EXPECTED_CORNERS = GRID_COLS * GRID_ROWS;
    public static final float SQUARE_SIZE_INCHES = 1.0f;
    public static final int CALIBRATION_FRAME_INTERVAL = 3;
    public static final int FRAMES_TO_CONFIRM = 5;

    // -------------------------------------------------------------------------
    // Overlay colors not already carried per-type by BallType (center dot,
    // ground-contact dot, velocity arrow, origin crosshair).
    // -------------------------------------------------------------------------
    public static final Scalar COLOR_CENTER   = new Scalar(255, 255, 255);
    public static final Scalar COLOR_CONTACT  = new Scalar(0, 0, 255);
    public static final Scalar COLOR_VELOCITY = new Scalar(255, 128, 0);
    public static final Scalar COLOR_ORIGIN   = new Scalar(255, 0, 255);
}
