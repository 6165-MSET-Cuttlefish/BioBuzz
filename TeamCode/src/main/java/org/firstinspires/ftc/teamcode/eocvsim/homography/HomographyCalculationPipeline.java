package org.firstinspires.ftc.teamcode.eocvsim.homography;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Locks a chessboard homography from full-resolution image pixels to camera-frame ground inches
 * (+X away from the camera, +Y left, origin at the centre of the board's near edge) and prints it
 * for {@code BallVisionConstants.H_ARRAY} and the Limelight SnapScripts.
 */
public class HomographyCalculationPipeline extends OpenCvPipeline {

    private static final int    GRID_COLS          = 9;
    private static final int    GRID_ROWS          = 6;
    private static final int    EXPECTED_CORNERS   = GRID_COLS * GRID_ROWS;
    private static final double SQUARE_SIZE_INCHES = 1.0; // measure the print: printers rescale

    // SB matches corners independently, so foreshortened far squares don't fail the whole board the
    // way classic findChessboardCorners does. It needs a white border about one square wide.
    private static final int CHESSBOARD_FLAGS =
            Calib3d.CALIB_CB_NORMALIZE_IMAGE
                    | Calib3d.CALIB_CB_EXHAUSTIVE
                    | Calib3d.CALIB_CB_ACCURACY;

    // px. Loose because steep or distant boards localize corners more noisily than a close-up one.
    private static final double RANSAC_REPROJ_THRESHOLD_PX = 8.0;

    // The printed H_ARRAY is for frames of this size, whatever the input's. Only a rescale: the input
    // must show the same view at the same aspect ratio, not a crop of it.
    private static final int OUTPUT_WIDTH_PX  = 640;
    private static final int OUTPUT_HEIGHT_PX = 480;

    // The Limelight 3A's lens calibration (latest.cal). False for the webcam, which has none: the
    // printed H_ARRAY then maps raw pixels, as BallVisionConstants expects.
    private static final boolean  UNDISTORT                  = true;
    private static final double   LENS_CALIBRATION_WIDTH_PX  = 1280;
    private static final double   LENS_CALIBRATION_HEIGHT_PX = 960;
    private static final double   LENS_FX = 1213.9165101673461, LENS_FY = 1215.7599736077275;
    private static final double   LENS_CX = 619.8522235104792,  LENS_CY = 480.21530891255276;
    // k1, k2, p1, p2, k3; unlike the focal lengths and centre, these don't scale with resolution.
    private static final double[] LENS_DIST = {
            0.18211160674758384, -0.5403918861880735, 0.004010036343230011,
            -0.00029924580297112217, 0.4039037424946443 };
    private static final int      UNDISTORT_ITERATIONS = 20;

    private static final double CHECK_GRID_STEP_IN   = 6.0;
    private static final double CHECK_GRID_MAX_X_IN  = 72.0;
    private static final double CHECK_GRID_HALF_Y_IN = 36.0;
    private static final double CHECK_GRID_SAMPLE_IN = 1.0;
    private static final Scalar GRID_COLOR  = new Scalar(0, 255, 0);
    private static final Scalar LABEL_COLOR = new Scalar(255, 255, 0);

    private static final class Calibration {
        final List<MatOfPoint> gridLines;
        final List<Point>      labelPoints;
        final List<String>     labelTexts;
        final String           report;

        Calibration(List<MatOfPoint> gridLines, List<Point> labelPoints, List<String> labelTexts, String report) {
            this.gridLines   = gridLines;
            this.labelPoints = labelPoints;
            this.labelTexts  = labelTexts;
            this.report      = report;
        }
    }

    private volatile Calibration calibration = null;
    private volatile String      statusLine  = "Searching for chessboard...";

    private final Telemetry    telemetry;
    private final Mat          gray    = new Mat();
    private final MatOfPoint2f corners = new MatOfPoint2f();

    public HomographyCalculationPipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    // Synchronous so a still image works: EOCV-Sim runs the pipeline once on an image source, then pauses.
    @Override
    public Mat processFrame(Mat input) {
        if (calibration == null) detect(input);

        Calibration cal = calibration;
        if (cal != null) {
            Imgproc.polylines(input, cal.gridLines, false, GRID_COLOR, 1);
            for (int i = 0; i < cal.labelPoints.size(); i++) {
                Imgproc.putText(input, cal.labelTexts.get(i), cal.labelPoints.get(i),
                        Imgproc.FONT_HERSHEY_SIMPLEX, 0.4, LABEL_COLOR, 1);
            }
            telemetry.addLine("HOMOGRAPHY LOCKED (tap the viewport to recalibrate)");
            telemetry.addLine(cal.report);
            telemetry.update();
            return input;
        }

        telemetry.addLine(statusLine);
        telemetry.addData("Board", "%dx%d inner corners (%dx%d squares), %.2f in squares",
                GRID_COLS, GRID_ROWS, GRID_COLS + 1, GRID_ROWS + 1, SQUARE_SIZE_INCHES);
        telemetry.addData("Input", "%dx%d, H_ARRAY printed for %dx%d",
                input.width(), input.height(), OUTPUT_WIDTH_PX, OUTPUT_HEIGHT_PX);
        telemetry.update();
        return input;
    }

    @Override
    public void onViewportTapped() {
        statusLine = "Searching for chessboard...";
        calibration = null;
    }

    private void detect(Mat input) {
        Imgproc.cvtColor(input, gray, Imgproc.COLOR_RGB2GRAY);

        boolean found = Calib3d.findChessboardCornersSB(
                gray,
                new Size(GRID_COLS, GRID_ROWS),
                corners,
                CHESSBOARD_FLAGS
        );

        if (!found || corners.rows() != EXPECTED_CORNERS) {
            statusLine = "Searching... (board not visible)";
            return;
        }

        int width = input.width();
        int height = input.height();
        if (UNDISTORT && Math.abs((double) width / height
                - LENS_CALIBRATION_WIDTH_PX / LENS_CALIBRATION_HEIGHT_PX) > 0.01) {
            throw new IllegalArgumentException("Input " + width + "x" + height
                    + " doesn't have the lens calibration's aspect ratio, so the lens numbers don't apply");
        }

        Point[] pixels = corners.toArray();
        for (Point p : pixels) Imgproc.circle(input, p, 3, GRID_COLOR, -1);

        Point[] undistorted = new Point[pixels.length];
        for (int i = 0; i < pixels.length; i++) undistorted[i] = undistort(pixels[i], width, height);

        MatOfPoint2f fitPixels = new MatOfPoint2f(undistorted);
        MatOfPoint2f ground = new MatOfPoint2f(groundCorners(pixels));
        Mat h = Calib3d.findHomography(fitPixels, ground, Calib3d.RANSAC, RANSAC_REPROJ_THRESHOLD_PX);
        try {
            if (h == null || h.empty()) {
                statusLine = "Homography failed (RANSAC) — retrying...";
                return;
            }
            Calibration cal = buildCalibration(h, fitPixels, ground, width, height);
            System.out.println(cal.report);
            calibration = cal;
        } finally {
            fitPixels.release();
            ground.release();
            if (h != null) h.release();
        }
    }

    /** fx, fy, cx, cy scaled to a width x height frame. */
    private static double[] intrinsics(int width, int height) {
        double sx = width / LENS_CALIBRATION_WIDTH_PX;
        double sy = height / LENS_CALIBRATION_HEIGHT_PX;
        return new double[] { LENS_FX * sx, LENS_FY * sy, LENS_CX * sx, LENS_CY * sy };
    }

    /** Radial factor and tangential dx, dy at normalized (x, y), OpenCV's 5-coefficient model. */
    private static double[] distortion(double x, double y) {
        double k1 = LENS_DIST[0], k2 = LENS_DIST[1], p1 = LENS_DIST[2], p2 = LENS_DIST[3], k3 = LENS_DIST[4];
        double r2 = x * x + y * y;
        return new double[] {
                1 + r2 * (k1 + r2 * (k2 + r2 * k3)),
                2 * p1 * x * y + p2 * (r2 + 2 * x * x),
                p1 * (r2 + 2 * y * y) + 2 * p2 * x * y };
    }

    // Fixed-point inversion, written out so the Limelight SnapScripts can match it line for line.
    private static Point undistort(Point p, int width, int height) {
        if (!UNDISTORT) return p;
        double[] k = intrinsics(width, height);
        double xd = (p.x - k[2]) / k[0], yd = (p.y - k[3]) / k[1];
        double x = xd, y = yd;
        for (int i = 0; i < UNDISTORT_ITERATIONS; i++) {
            double[] d = distortion(x, y);
            x = (xd - d[1]) / d[0];
            y = (yd - d[2]) / d[0];
        }
        return new Point(x * k[0] + k[2], y * k[1] + k[3]);
    }

    private static Point distort(Point p, int width, int height) {
        if (!UNDISTORT) return p;
        double[] k = intrinsics(width, height);
        double x = (p.x - k[2]) / k[0], y = (p.y - k[3]) / k[1];
        double[] d = distortion(x, y);
        return new Point((x * d[0] + d[1]) * k[0] + k[2], (y * d[0] + d[2]) * k[1] + k[3]);
    }

    /**
     * Ground inches for each detected corner, whichever corner OpenCV numbered first: the board axis
     * pointing furthest up the image is +X, the other points left as +Y. Corners are row-major,
     * GRID_COLS per row.
     */
    private static Point[] groundCorners(Point[] px) {
        Point alongCols = minus(px[GRID_COLS - 1], px[0]);
        Point alongRows = minus(px[(GRID_ROWS - 1) * GRID_COLS], px[0]);
        double colsUp = -alongCols.y / Math.hypot(alongCols.x, alongCols.y);
        double rowsUp = -alongRows.y / Math.hypot(alongRows.x, alongRows.y);

        boolean forwardAlongCols = Math.abs(colsUp) >= Math.abs(rowsUp);
        double forwardSign = Math.signum(forwardAlongCols ? colsUp : rowsUp);
        double leftSign = (forwardAlongCols ? alongRows.x : alongCols.x) < 0 ? 1 : -1;

        double[] x = new double[px.length];
        double[] y = new double[px.length];
        double minX = Double.MAX_VALUE, minY = Double.MAX_VALUE, maxY = -Double.MAX_VALUE;
        for (int i = 0; i < px.length; i++) {
            int col = i % GRID_COLS;
            int row = i / GRID_COLS;
            x[i] = forwardSign * (forwardAlongCols ? col : row) * SQUARE_SIZE_INCHES;
            y[i] = leftSign * (forwardAlongCols ? row : col) * SQUARE_SIZE_INCHES;
            minX = Math.min(minX, x[i]);
            minY = Math.min(minY, y[i]);
            maxY = Math.max(maxY, y[i]);
        }

        Point[] ground = new Point[px.length];
        for (int i = 0; i < px.length; i++) {
            ground[i] = new Point(x[i] - minX, y[i] - (minY + maxY) / 2.0);
        }
        return ground;
    }

    private static Point minus(Point a, Point b) {
        return new Point(a.x - b.x, a.y - b.y);
    }

    private static Calibration buildCalibration(Mat h, MatOfPoint2f pixels, MatOfPoint2f ground,
                                                int width, int height) {
        MatOfPoint2f fitted = new MatOfPoint2f();
        Core.perspectiveTransform(pixels, fitted, h);
        Point[] fit = fitted.toArray();
        Point[] truth = ground.toArray();
        fitted.release();

        double sumSq = 0, maxErr = 0, maxX = 0;
        for (int i = 0; i < fit.length; i++) {
            double err = Math.hypot(fit[i].x - truth[i].x, fit[i].y - truth[i].y);
            sumSq += err * err;
            maxErr = Math.max(maxErr, err);
            maxX = Math.max(maxX, truth[i].x);
        }

        Mat inverse = h.inv();
        List<MatOfPoint> lines = new ArrayList<>();
        List<Point> labelPoints = new ArrayList<>();
        List<String> labelTexts = new ArrayList<>();
        double refSign = Math.signum(homogeneousW(inverse, truth[0].x, truth[0].y));

        for (double gx = 0; gx <= CHECK_GRID_MAX_X_IN; gx += CHECK_GRID_STEP_IN) {
            List<Point> line = new ArrayList<>();
            for (double gy = -CHECK_GRID_HALF_Y_IN; gy <= CHECK_GRID_HALF_Y_IN; gy += CHECK_GRID_SAMPLE_IN) {
                line.add(toPixel(inverse, gx, gy, refSign, width, height));
            }
            addSegments(lines, line);
            Point label = toPixel(inverse, gx, 0, refSign, width, height);
            if (label != null && label.x >= 0 && label.x < width && label.y >= 0 && label.y < height) {
                labelPoints.add(new Point(label.x + 3, label.y - 3));
                labelTexts.add(String.format("%.0f", gx));
            }
        }
        for (double gy = -CHECK_GRID_HALF_Y_IN; gy <= CHECK_GRID_HALF_Y_IN; gy += CHECK_GRID_STEP_IN) {
            List<Point> line = new ArrayList<>();
            for (double gx = 0; gx <= CHECK_GRID_MAX_X_IN; gx += CHECK_GRID_SAMPLE_IN) {
                line.add(toPixel(inverse, gx, gy, refSign, width, height));
            }
            addSegments(lines, line);
        }
        inverse.release();

        String aspectWarning =
                Math.abs((double) width / height - (double) OUTPUT_WIDTH_PX / OUTPUT_HEIGHT_PX) > 0.01
                        ? String.format("WARNING: input %dx%d and output %dx%d differ in aspect ratio%n",
                                width, height, OUTPUT_WIDTH_PX, OUTPUT_HEIGHT_PX)
                        : "";
        String report = String.format(
                "fit error %.3f in RMS, %.3f in max (over ~0.05 means bad corners or a warped print)%n"
                        + "board spans x 0 to %.1f in; expect error to grow well beyond that%n"
                        + "square size assumed %.3f in: measure the print%n",
                Math.sqrt(sumSq / fit.length), maxErr, maxX, SQUARE_SIZE_INCHES)
                + (UNDISTORT ? String.format("lens undistortion ON (Limelight): for the SnapScripts only%n") : "")
                + aspectWarning + String.format("%n")
                + buildHomographyString(h, width, height);
        return new Calibration(Collections.unmodifiableList(lines), labelPoints, labelTexts, report);
    }

    private static double homogeneousW(Mat m, double x, double y) {
        return m.get(2, 0)[0] * x + m.get(2, 1)[0] * y + m.get(2, 2)[0];
    }

    /**
     * Raw-image pixel for a ground point. Null behind the horizon or off-screen, where the projection
     * and the distortion polynomial are both meaningless.
     */
    private static Point toPixel(Mat inverse, double x, double y, double refSign, int width, int height) {
        double w = homogeneousW(inverse, x, y);
        if (Math.signum(w) != refSign || Math.abs(w) < 1e-9) return null;
        double u = (inverse.get(0, 0)[0] * x + inverse.get(0, 1)[0] * y + inverse.get(0, 2)[0]) / w;
        double v = (inverse.get(1, 0)[0] * x + inverse.get(1, 1)[0] * y + inverse.get(1, 2)[0]) / w;
        if (u < -0.1 * width || u > 1.1 * width || v < -0.1 * height || v > 1.1 * height) return null;
        return distort(new Point(u, v), width, height);
    }

    private static void addSegments(List<MatOfPoint> lines, List<Point> samples) {
        List<Point> segment = new ArrayList<>();
        for (Point p : samples) {
            if (p != null) {
                segment.add(p);
                continue;
            }
            if (segment.size() > 1) lines.add(new MatOfPoint(segment.toArray(new Point[0])));
            segment = new ArrayList<>();
        }
        if (segment.size() > 1) lines.add(new MatOfPoint(segment.toArray(new Point[0])));
    }

    /** Rescaled to OUTPUT size: H_out = H_in * diag(inW / outW, inH / outH, 1). */
    private static String buildHomographyString(Mat h, int inputWidth, int inputHeight) {
        double sx = (double) inputWidth / OUTPUT_WIDTH_PX;
        double sy = (double) inputHeight / OUTPUT_HEIGHT_PX;
        StringBuilder java = new StringBuilder("// calibrated at ")
                .append(OUTPUT_WIDTH_PX).append('x').append(OUTPUT_HEIGHT_PX)
                .append(" (from a ").append(inputWidth).append('x').append(inputHeight).append(" input)")
                .append("\ndouble[][] H_ARRAY = {\n");
        StringBuilder python = new StringBuilder("UNDISTORT = ").append(UNDISTORT ? "True" : "False")
                .append("\nCALIBRATION_SIZE = (")
                .append(OUTPUT_WIDTH_PX).append(", ").append(OUTPUT_HEIGHT_PX).append(")\nH_ARRAY = (\n");
        for (int r = 0; r < 3; r++) {
            String row = String.format("%.10e, %.10e, %.10e",
                    h.get(r, 0)[0] * sx, h.get(r, 1)[0] * sy, h.get(r, 2)[0]);
            java.append("    { ").append(row).append(r < 2 ? " },\n" : " }\n");
            python.append("    (").append(row).append("),\n");
        }
        return java.append("};\n\n").append(python).append(")").toString();
    }
}
