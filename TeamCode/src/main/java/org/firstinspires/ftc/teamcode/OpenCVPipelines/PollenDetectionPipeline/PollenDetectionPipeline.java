package org.firstinspires.ftc.teamcode.OpenCVPipelines.PollenDetectionPipeline;




import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

/**
 * Detects yellow Pollen balls and reports each ball's GROUND-CONTACT position
 * (where it touches the floor), in inches, in a top-down frame.
 *
 * Why ground contact and not the ball center: the homography only maps points that
 * lie on the ground plane. A ball's center/top are above the plane, so warping the
 * whole ball smears it outward and the smear-center over-reports distance. Instead we
 * detect each ball in the ORIGINAL (un-warped) image, take the bottom of its silhouette
 * (the floor-contact point, which IS on the ground plane), and map only that point
 * through the homography. The warped view is used for display only.
 */

public class PollenDetectionPipeline extends OpenCvPipeline {

    // Set true to skip live calibration and use the hardcoded H_ARRAY below.
    private static final boolean USE_PREDETERMINED_HOMOGRAPHY = false;

    //   MASK    — upscaled detection mask (tune the HSV range against this).
    //   OVERLAY — top-down warped image with a ground-contact marker per ball.
    public enum DisplayMode { MASK, OVERLAY }

    // Predetermined homography (used when USE_PREDETERMINED_HOMOGRAPHY = true).
    private static final double[][] H_ARRAY = {
            { 1.45538818e+00,  4.74651118e-01, -1.39014816e+02 },
            {-1.91056302e-02,  2.35917771e+00, -1.78695993e+02 },
            { 1.00860604e-04,  1.45801043e-03,  1.00000000e+00 }
    };

    // Pixel -> inches in the warped (top-down) view. Calibration places inner corners
    // OUTPUT_SCALE_PX apart and each square is SQUARE_SIZE_INCHES, so 50 px = 1 in.
    private static final float  OUTPUT_SCALE_PX    = 50.0f;
    private static final float  SQUARE_SIZE_INCHES = 1.0f;
    private static final double PIXELS_TO_INCHES   = SQUARE_SIZE_INCHES / OUTPUT_SCALE_PX; // 0.02

    // Detection runs on the ORIGINAL camera image scaled down by this factor (perf).
    // Contact points are scaled back to full-res before the homography is applied.
    private static final double DETECTION_SCALE = 0.25;

    /**
     * Live-tunable yellow HSV gate + shape gate (FtcDashboard). Defaults are measured off real
     * Pollen frames (garage + competition lighting): the ball body spans H≈[12,34], so the hue
     * band is widened well past the old [20,30] that left fragmented blobs. The saturation floor is
     * kept low enough to catch dim/competition balls — which means sunlit warm clutter (cardboard,
     * skin, wood) can also pass the color gate; the roundness gate ({@link #minPeakDist}) and the
     * field-bounds check downstream reject that, and mounting the camera to look AT the field keeps
     * it out of frame. Glare (white specular tops) is low-saturation and excluded here, then filled
     * back in by the morphological close, so each ball still reads as one solid blob.
     * HSV is OpenCV-scaled: H 0-179, S/V 0-255.
     */
    public static class Tuning {
        // Dashboard dropdown: MASK = raw detection mask (tune HSV against it); OVERLAY = top-down view.
        public static DisplayMode displayMode = DisplayMode.OVERLAY;
        public static int hLow = 15, hHigh = 34;
        public static int sLow = 80, sHigh = 255;
        public static int vLow = 80, vHigh = 255;
        // Min inscribed radius (downscaled px) for a blob to count as a ball — the round-shape gate.
        public static double minPeakDist = 3.0;
    }

    // Physical ball radius (1.5 in radius = 3 in ball) in warped px, for the display ring.
    private static final double BALL_RADIUS_PX = (1.5 / PIXELS_TO_INCHES); // 75 px


    // When pinning the ground-contact point on the full-res mask, tolerate vertical gaps up to
    // this many pixels (wiffle-ball holes); a longer run of non-ball pixels means the floor.
    private static final int MAX_HOLE_GAP = 30;

    // Temporal smoothing of detections (anti-flicker), in warped pixels / frames.
    private static final double TRACK_MATCH_PX   = 100.0; // associate a detection to a track within this (~2 in)
    private static final int    TRACK_MAX_MISSES = 6;     // keep a track alive this many frames after it was last seen
    private static final int    TRACK_MIN_HITS   = 2;     // show a track only after it's been seen this many frames
    private static final double TRACK_SMOOTH     = 0.5;   // EMA factor for the smoothed position (higher = snappier)

    private static final Mat OPEN_KERNEL = Imgproc.getStructuringElement(
            Imgproc.MORPH_ELLIPSE, new Size(3, 3));
    // Larger close than before: spans the white specular glare on top of a ball (and the wiffle
    // holes) so the body fills back into one solid, convex blob instead of a ring with a hole.
    private static final Mat FILL_KERNEL = Imgproc.getStructuringElement(
            Imgproc.MORPH_ELLIPSE, new Size(11, 11));
    private static final Mat PEAK_DILATE_KERNEL = Imgproc.getStructuringElement(
            Imgproc.MORPH_ELLIPSE, new Size(3, 3));

    // Homography calibration settings — 7x7 inner-corner chessboard (a standard 8x8
    // checkers/draughts board). Orientation is irrelevant here: we only need the top-down
    // rectification, and any 90/180/mirror of it is still a valid metric warp.
    private static final int GRID_COLS = 7;
    private static final int GRID_ROWS = 7;
    private static final int EXPECTED_CORNERS = GRID_COLS * GRID_ROWS;
    private static final int DETECTION_FRAME_INTERVAL = 3;
    private static final int FRAMES_TO_CONFIRM = 5;

    // Field shown around the board in the warped view. Larger = wider top-down viewport so
    // balls that sit far from the board aren't clipped. Warp canvas = board span + 2*MARGIN_PX
    // (so 500 -> 1450x1300). Raise it further if balls still fall off the edges.
    private static final float MARGIN_PX = 500.0f;

    // Origin = board center in the warped image; reported positions are
    // (warpedContact - ORIGIN) * PIXELS_TO_INCHES.  +X = right, +Y = toward the bottom of
    // the (vertically-corrected) top-down view.
    // Derived from MARGIN_PX/grid so the two can never drift out of sync.
    private static final double ORIGIN_X = MARGIN_PX + (GRID_COLS - 1) / 2.0 * SQUARE_SIZE_INCHES * OUTPUT_SCALE_PX; // 700
    private static final double ORIGIN_Y = MARGIN_PX + (GRID_ROWS - 1) / 2.0 * SQUARE_SIZE_INCHES * OUTPUT_SCALE_PX; // 625

    private enum Phase { CALIBRATING, DETECTING }

    private Phase phase;
    private Mat   homography = null;       // full-res original -> top-down warp
    private final double[] hVals = new double[9]; // cached homography for fast point mapping
    private int   confirmCount = 0;
    private int   frameCount   = 0;
    private Size  warpSize;

    // Pre-allocated Mats reused every frame to avoid native-heap churn.
    private final Mat detSmall     = new Mat(); // downscaled original (detection input)
    private final Mat hsv          = new Mat();
    private final Mat yellowMask   = new Mat();
    private final Mat cleanMask    = new Mat();
    private final Mat filledMask   = new Mat();
    private final Mat distMat      = new Mat(); // distance transform (CV_32F)
    private final Mat localMax     = new Mat(); // dilated distMat, for local-maxima test
    private final Mat distFloor    = new Mat(); // absolute distance-floor mask
    private final Mat peaks        = new Mat(); // ball-center seeds
    private final Mat hsvFull      = new Mat(); // full-res HSV (contact-point refinement)
    private final Mat maskFull     = new Mat(); // full-res yellow mask (contact-point refinement)
    private final Mat contourImage = new Mat(); // returned display frame
    private final Mat contourHierarchy = new Mat();
    private final List<MatOfPoint> peakContours = new ArrayList<>();
    private final byte[] maskPixel = new byte[1]; // reused 1-byte buffer for mask column scans
    private final List<Track> tracks = new ArrayList<>(); // persistent detections (anti-flicker)

    private final Mat          gray         = new Mat();
    private final MatOfPoint2f imageCorners = new MatOfPoint2f();
    private final MatOfPoint2f dstCorners;

    private final Telemetry telemetry;

    public PollenDetectionPipeline(Telemetry telemetry) {
        this.telemetry = telemetry;

        if (USE_PREDETERMINED_HOMOGRAPHY) {
            homography = buildHomographyFromArray(H_ARRAY);
            warpSize   = buildWarpSize(); // the board layout the homography targets, NOT camera res
            cacheHomography();
            phase = Phase.DETECTING;
        } else {
            phase = Phase.CALIBRATING;
        }

        dstCorners = buildCalibrationDstCorners();
    }

    public Mat     getHomography() { return homography; }
    public boolean isCalibrated()  { return phase == Phase.DETECTING; }

    /** Returns the homography as a copy-pasteable Java array string. */
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

    @Override
    public Mat processFrame(Mat input) {
        switch (phase) {
            case CALIBRATING: return runCalibrationFrame(input);
            case DETECTING:   return runDetectionFrame(input);
            default:          return input;
        }
    }

    // =========================================================================
    // PHASE 1 — Homography calibration
    // =========================================================================

    private Mat runCalibrationFrame(Mat input) {
        frameCount++;

        if (frameCount % DETECTION_FRAME_INTERVAL != 0) {
            telemetry.addLine("[Calibrating] Searching for chessboard...");
            telemetry.addData("Confirmed", confirmCount + " / " + FRAMES_TO_CONFIRM);
            telemetry.update();
            return input;
        }

        if (!detectChessboardCorners(input)) {
            confirmCount = 0;
            telemetry.addLine("[Calibrating] Chessboard NOT found");
            telemetry.addData("Tip", "Ensure full board is visible and flat");
            telemetry.update();
            return input;
        }

        Mat h = computeHomography(imageCorners, dstCorners);

        if (h == null) {
            confirmCount = 0;
            telemetry.addLine("[Calibrating] Homography computation failed");
            telemetry.update();
            return input;
        }

        if (homography != null) homography.release(); // free the prior confirm frame's Mat
        homography = h;
        confirmCount++;

        if (confirmCount >= FRAMES_TO_CONFIRM) {
            lockCalibration();
        }

        telemetry.addLine(phase == Phase.DETECTING
                ? "[LOCKED] Switching to detection..."
                : "[Calibrating] Confirming...");
        reportHomographyToTelemetry(homography);
        telemetry.update();

        Calib3d.drawChessboardCorners(input, new Size(GRID_COLS, GRID_ROWS), imageCorners, true);
        return input;
    }

    private void lockCalibration() {
        warpSize = buildWarpSize();
        cacheHomography();
        phase = Phase.DETECTING;
    }

    // =========================================================================
    // PHASE 2 — Pollen detection (raw image) + ground-contact mapping
    // =========================================================================

    private Mat runDetectionFrame(Mat input) {
        Size fullSize = warpSize != null ? warpSize : input.size();

        // 1. Detect on a downscaled copy of the ORIGINAL image — the ground-contact
        //    point must be measured before the ground-plane warp distorts ball height.
        Imgproc.resize(input, detSmall,
                new Size(input.cols() * DETECTION_SCALE, input.rows() * DETECTION_SCALE));
        Scalar yellowLow  = new Scalar(Tuning.hLow,  Tuning.sLow,  Tuning.vLow);
        Scalar yellowHigh = new Scalar(Tuning.hHigh, Tuning.sHigh, Tuning.vHigh);
        Imgproc.cvtColor(detSmall, hsv, Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsv, yellowLow, yellowHigh, yellowMask);

        // 2. Remove specks (open), then fill the wiffle-ball holes (close).
        Imgproc.morphologyEx(yellowMask, cleanMask,  Imgproc.MORPH_OPEN,  OPEN_KERNEL);
        Imgproc.morphologyEx(cleanMask,  filledMask, Imgproc.MORPH_CLOSE, FILL_KERNEL);

        List<float[]> balls = new ArrayList<>(); // each: {warpX, warpY, xIn, yIn}

        if (Core.countNonZero(filledMask) > 0) {
            // 3. Distance transform; ball centers are LOCAL maxima (size-independent, so
            //    smaller/farther balls aren't dropped by a single global threshold).
            Imgproc.distanceTransform(filledMask, distMat, Imgproc.DIST_L2, 3);
            Imgproc.GaussianBlur(distMat, distMat, new Size(3, 3), 0); // steady peak locations frame-to-frame
            Imgproc.dilate(distMat, localMax, PEAK_DILATE_KERNEL);
            Core.compare(distMat, localMax, peaks, Core.CMP_GE); // 255 where pixel == local max

            // Absolute distance floor: drops noise specks AND thin (non-round) objects,
            // whose inscribed radius never reaches Tuning.minPeakDist.
            Imgproc.threshold(distMat, distFloor, Tuning.minPeakDist, 255, Imgproc.THRESH_BINARY);
            distFloor.convertTo(distFloor, CvType.CV_8U);
            Core.bitwise_and(peaks, distFloor, peaks);

            // Collapse each peak plateau into one blob, then take its centroid.
            Imgproc.dilate(peaks, peaks, PEAK_DILATE_KERNEL);
            peakContours.clear();
            Imgproc.findContours(peaks, peakContours, contourHierarchy,
                    Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            // Full-resolution yellow mask, used only to pin each ball's ground-contact point
            // precisely. The 1/4-scale pass above finds and separates balls, but its contact Y
            // is quantized to ~4 full-res px, which the homography magnifies for distant balls.
            Imgproc.cvtColor(input, hsvFull, Imgproc.COLOR_RGB2HSV);
            Core.inRange(hsvFull, yellowLow, yellowHigh, maskFull);

            double[] warpPt = new double[2];
            for (MatOfPoint pc : peakContours) {
                org.opencv.imgproc.Moments m = Imgproc.moments(pc);
                if (m.m00 == 0) continue;

                // Ball center from the 1/4-scale pass, lifted to full-res original pixels.
                int cxF = (int) Math.min(Math.max(m.m10 / m.m00 / DETECTION_SCALE, 0), maskFull.cols() - 1);
                int cyF = (int) Math.min(Math.max(m.m01 / m.m00 / DETECTION_SCALE, 0), maskFull.rows() - 1);

                // Ground contact = the lowest yellow pixel directly below the center in the
                // full-res mask. Tolerate short gaps (wiffle holes); stop at the floor, i.e.
                // once more than MAX_HOLE_GAP consecutive non-ball pixels have passed.
                int bottomYF = cyF, gap = 0;
                for (int y = cyF; y < maskFull.rows(); y++) {
                    maskFull.get(y, cxF, maskPixel);
                    if (maskPixel[0] != 0) { bottomYF = y; gap = 0; }
                    else if (++gap > MAX_HOLE_GAP) break;
                }

                // Map only that ground-plane contact point through the homography.
                if (!mapToWarp(cxF, bottomYF, warpPt)) continue;
                // Background (people, clutter above the wall) isn't on the ground plane, so its
                // contact projects off the top-down field canvas — drop anything outside it.
                if (warpPt[0] < 0 || warpPt[1] < 0
                        || warpPt[0] >= fullSize.width || warpPt[1] >= fullSize.height) continue;
                double xIn = (warpPt[0] - ORIGIN_X) * PIXELS_TO_INCHES;
                double yIn = (warpPt[1] - ORIGIN_Y) * PIXELS_TO_INCHES;
                balls.add(new float[]{ (float) warpPt[0], (float) warpPt[1], (float) xIn, (float) yIn });
            }
            for (MatOfPoint pc : peakContours) pc.release(); // free per-frame native contour Mats
        }

        // Temporal smoothing: fold this frame's raw detections into persistent tracks so a
        // briefly-missed ball (or a 1-frame false positive) doesn't flicker on/off, and the
        // shown positions are EMA-smoothed. `stable` is the debounced output used below.
        updateTracks(balls);
        List<float[]> stable = confirmedTracks();

        // 4. Display.
        if (Tuning.displayMode == DisplayMode.MASK) {
            // Detection mask (original space), upscaled — for tuning the HSV range. Ball markers
            // are in warped coords, so they're omitted here (they wouldn't line up with this view).
            Imgproc.resize(filledMask, yellowMask, fullSize, 0, 0, Imgproc.INTER_NEAREST);
            Imgproc.cvtColor(yellowMask, contourImage, Imgproc.COLOR_GRAY2RGB);
        } else {
            // Top-down warped image — shown every frame, including when no Pollen is present.
            Imgproc.warpPerspective(input, contourImage, homography, fullSize);
            for (float[] ball : stable) {
                Point contact = new Point(ball[0], ball[1]);
                Imgproc.circle(contourImage, contact, (int) BALL_RADIUS_PX, new Scalar(0, 255, 0), 2); // green ball-size ring (RGB)
                Imgproc.circle(contourImage, contact, 5, new Scalar(255, 0, 0), -1);                    // red ground-contact dot (RGB)
            }
            drawOrigin(contourImage);
        }

        // 5. Telemetry.
        telemetry.addLine("[Detecting Pollen]");
        telemetry.addData("Pollen Detected", stable.size());
        for (int i = 0; i < stable.size(); i++) {
            float[] ball = stable.get(i);
            telemetry.addLine("--- Pollen " + i + " ---");
            telemetry.addData("  Ground X (in)", String.format("%.2f", ball[2]));
            telemetry.addData("  Ground Y (in)", String.format("%.2f", ball[3]));
        }
        telemetry.update();

        return contourImage;
    }

    // =========================================================================
    // Helpers
    // =========================================================================

    /** Marks the coordinate origin (ORIGIN_X, ORIGIN_Y) on a warped frame. Magenta reads
     *  the same in RGB or BGR and stands out from the green/red ball markers. */
    private void drawOrigin(Mat img) {
        Point  o       = new Point(ORIGIN_X, ORIGIN_Y);
        Scalar magenta = new Scalar(255, 0, 255);
        Imgproc.line(img, new Point(o.x - 14, o.y), new Point(o.x + 14, o.y), magenta, 2);
        Imgproc.line(img, new Point(o.x, o.y - 14), new Point(o.x, o.y + 14), magenta, 2);
        Imgproc.circle(img, o, 5, magenta, 2);
        Imgproc.putText(img, "(0,0)", new Point(o.x + 8, o.y + 20),
                Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, magenta, 1);
    }

    /** Detects inner chessboard corners into the reused imageCorners field (sub-pixel
     *  refined). Returns false if detection fails or the corner count is wrong. */
    private boolean detectChessboardCorners(Mat input) {
        Imgproc.cvtColor(input, gray, Imgproc.COLOR_RGB2GRAY);

        // Sector-based detector (OpenCV 4): far more robust to blur, glare, and the steep
        // floor-level perspective than the legacy findChessboardCorners, and it returns
        // sub-pixel corners directly (no cornerSubPix pass needed). EXHAUSTIVE trades speed
        // for hit-rate — the right call for a one-time calibration.
        boolean found = Calib3d.findChessboardCornersSB(
                gray,
                new Size(GRID_COLS, GRID_ROWS),
                imageCorners,
                Calib3d.CALIB_CB_NORMALIZE_IMAGE |
                        Calib3d.CALIB_CB_EXHAUSTIVE |
                        Calib3d.CALIB_CB_ACCURACY
        );

        return found && imageCorners.rows() == EXPECTED_CORNERS;
    }

    /** Computes a RANSAC homography mapping srcCorners -> dstCorners; null if empty. */
    private static Mat computeHomography(MatOfPoint2f srcCorners, MatOfPoint2f dstCorners) {
        Mat h = Calib3d.findHomography(srcCorners, dstCorners, Calib3d.RANSAC, 5.0);
        if (h == null || h.empty()) {
            if (h != null) h.release();
            return null;
        }
        return h;
    }

    /** Destination inner-corner grid (inner corners only, OUTPUT_SCALE_PX apart). */
    private static MatOfPoint2f buildCalibrationDstCorners() {
        List<Point> dstList = new ArrayList<>();
        for (int row = 0; row < GRID_ROWS; row++) {
            for (int col = 0; col < GRID_COLS; col++) {
                dstList.add(new Point(
                        MARGIN_PX + col * SQUARE_SIZE_INCHES * OUTPUT_SCALE_PX,
                        // Row order flipped (GRID_ROWS-1-row) so the top-down warp isn't upside down.
                        MARGIN_PX + (GRID_ROWS - 1 - row) * SQUARE_SIZE_INCHES * OUTPUT_SCALE_PX
                ));
            }
        }
        MatOfPoint2f dst = new MatOfPoint2f();
        dst.fromList(dstList);
        return dst;
    }

    /** Warp output size = board layout + 2*MARGIN_PX (1450x1300 with MARGIN_PX=500). */
    private static Size buildWarpSize() {
        int width  = (int)(GRID_COLS * SQUARE_SIZE_INCHES * OUTPUT_SCALE_PX + 2 * MARGIN_PX);
        int height = (int)(GRID_ROWS * SQUARE_SIZE_INCHES * OUTPUT_SCALE_PX + 2 * MARGIN_PX);
        return new Size(width, height);
    }

    /** Builds a 3x3 CV_64F Mat from a raw 3x3 double array (loads H_ARRAY). */
    private static Mat buildHomographyFromArray(double[][] arr) {
        Mat h = new Mat(3, 3, CvType.CV_64F);
        for (int r = 0; r < 3; r++)
            for (int c = 0; c < 3; c++)
                h.put(r, c, arr[r][c]);
        return h;
    }

    /** Snapshots the homography into hVals so per-point mapping is pure-Java (no JNI gets). */
    private void cacheHomography() {
        if (homography == null) return;
        for (int r = 0; r < 3; r++)
            for (int c = 0; c < 3; c++)
                hVals[r * 3 + c] = homography.get(r, c)[0];
    }

    /** Maps an original-image point through the cached homography into warped pixels.
     *  Returns false (leaving out untouched) for points on/behind the homography horizon,
     *  where w -> 0 would produce NaN/Inf. */
    private boolean mapToWarp(double x, double y, double[] out) {
        double wx = hVals[0] * x + hVals[1] * y + hVals[2];
        double wy = hVals[3] * x + hVals[4] * y + hVals[5];
        double w  = hVals[6] * x + hVals[7] * y + hVals[8];
        if (Math.abs(w) < 1e-9) return false;
        out[0] = wx / w;
        out[1] = wy / w;
        return true;
    }

    private void reportHomographyToTelemetry(Mat h) {
        telemetry.addLine("--- Homography Matrix ---");
        for (int r = 0; r < 3; r++) {
            telemetry.addData("Row " + r,
                    String.format("{ %.6e, %.6e, %.6e }",
                            h.get(r, 0)[0],
                            h.get(r, 1)[0],
                            h.get(r, 2)[0]));
        }
        telemetry.addLine("Copy-paste string:");
        telemetry.addLine(getHomographyAsString());
    }

    // =========================================================================
    // Temporal smoothing (anti-flicker)
    // =========================================================================

    private static final class Track {
        double warpX, warpY; // EMA-smoothed warped position
        int hits;            // frames matched (capped at TRACK_MIN_HITS)
        int sinceSeen;       // frames since last matched
    }

    /** Folds this frame's raw detections into the persistent track list (greedy nearest match). */
    private void updateTracks(List<float[]> detections) {
        boolean[] matched = new boolean[tracks.size()];
        List<Track> fresh = new ArrayList<>();
        for (float[] d : detections) {
            int best = -1;
            double bestDist = TRACK_MATCH_PX * TRACK_MATCH_PX;
            for (int i = 0; i < tracks.size(); i++) {
                if (matched[i]) continue;
                Track t = tracks.get(i);
                double dx = t.warpX - d[0], dy = t.warpY - d[1];
                double dd = dx * dx + dy * dy;
                if (dd < bestDist) { bestDist = dd; best = i; }
            }
            if (best >= 0) {
                Track t = tracks.get(best);
                t.warpX += (d[0] - t.warpX) * TRACK_SMOOTH;
                t.warpY += (d[1] - t.warpY) * TRACK_SMOOTH;
                if (t.hits < TRACK_MIN_HITS) t.hits++;
                t.sinceSeen = 0;
                matched[best] = true;
            } else {
                Track t = new Track();
                t.warpX = d[0];
                t.warpY = d[1];
                t.hits = 1;
                t.sinceSeen = 0;
                fresh.add(t);
            }
        }
        // Age unmatched tracks; drop the stale ones.
        for (int i = tracks.size() - 1; i >= 0; i--) {
            if (matched[i]) continue;
            if (++tracks.get(i).sinceSeen > TRACK_MAX_MISSES) tracks.remove(i);
        }
        tracks.addAll(fresh);
    }

    /** Confirmed, smoothed detections {warpX, warpY, xIn, yIn} — tracks seen >= TRACK_MIN_HITS. */
    private List<float[]> confirmedTracks() {
        List<float[]> out = new ArrayList<>();
        for (Track t : tracks) {
            if (t.hits < TRACK_MIN_HITS) continue;
            double xIn = (t.warpX - ORIGIN_X) * PIXELS_TO_INCHES;
            double yIn = (t.warpY - ORIGIN_Y) * PIXELS_TO_INCHES;
            out.add(new float[]{ (float) t.warpX, (float) t.warpY, (float) xIn, (float) yIn });
        }
        return out;
    }
}
