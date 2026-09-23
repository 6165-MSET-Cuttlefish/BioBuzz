package org.firstinspires.ftc.teamcode.decode.modules;

import com.pedropathing.math.Pose;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import org.firstinspires.ftc.teamcode.decode.DecodeContext;

public class ShooterInterpolation {

    static final Pose RED_GOAL = new Pose(DecodeContext.redTargetPose.x(), DecodeContext.redTargetPose.y());
    static final Pose BLUE_GOAL = new Pose(DecodeContext.blueTargetPose.x(), DecodeContext.blueTargetPose.y());

    public enum Mode { CLOSE, FAR }

    public static Mode activeMode = Mode.CLOSE;
    public static double farRPMOffset = -30;
    public static double farHoodOffset = -0.04;

    public static class ShooterDataPoint {
        public final double x, y, rpm, hood, tunedDistance;
        public final Mode mode;

        public ShooterDataPoint(double x, double y, double rpm, double hood, Pose goal, Mode mode) {
            this.x = x;
            this.y = y;
            this.rpm = rpm;
            this.hood = hood;
            this.tunedDistance = Math.hypot(goal.x() - x, goal.y() - y);
            this.mode = mode;
        }
    }

    static class PositionIndex {
        final String key;
        final double distance;
        final List<ShooterDataPoint> points;
        final double meanRPM;
        final double maxRPM;
        final double minRPM;
        final double minHood;
        final double maxHood;

        PositionIndex(String key, double distance, List<ShooterDataPoint> points) {
            this.key = key;
            this.distance = distance;
            this.points = points;

            double sum = 0, max = Double.NEGATIVE_INFINITY, min = Double.POSITIVE_INFINITY;
            double minH = Double.POSITIVE_INFINITY, maxH = Double.NEGATIVE_INFINITY;
            for (ShooterDataPoint p : points) {
                sum += p.rpm;
                max = Math.max(max, p.rpm);
                min = Math.min(min, p.rpm);
                minH = Math.min(minH, p.hood);
                maxH = Math.max(maxH, p.hood);
            }
            this.meanRPM = sum / points.size();
            this.maxRPM = max;
            this.minRPM = min;
            this.minHood = minH;
            this.maxHood = maxH;
        }
    }

    static class InterpolationTable {
        final Map<String, PositionIndex> positionIndex = new LinkedHashMap<>();
        final List<PositionIndex> sortedByDistance = new ArrayList<>();
        final Map<String, List<ShooterDataPoint>> allPointsByKey = new LinkedHashMap<>();

        void addPoint(double x, double y, double rpm, double hood, Pose goal, Mode mode) {
            ShooterDataPoint p = new ShooterDataPoint(x, y, rpm, hood, goal, mode);
            String k = key(x, y);
            allPointsByKey.computeIfAbsent(k, kk -> new ArrayList<>()).add(p);
        }

        void buildIndex() {
            for (Map.Entry<String, List<ShooterDataPoint>> e : allPointsByKey.entrySet()) {
                String k = e.getKey();
                List<ShooterDataPoint> pts = e.getValue();
                double dist = pts.get(0).tunedDistance;
                positionIndex.put(k, new PositionIndex(k, dist, pts));
                sortedByDistance.add(positionIndex.get(k));
            }
            sortedByDistance.sort(Comparator.comparingDouble(p -> p.distance));
        }

        // Every tuned shot was fit against this exact neighbour choice (including {before, before}); don't "fix" it.
        PositionIndex[] getNearestTwo(double targetDist) {
            int n = sortedByDistance.size();
            if (n < 2) {
                PositionIndex only = sortedByDistance.get(0);
                return new PositionIndex[]{only, only};
            }

            int idx = binarySearchClosest(targetDist);
            if (idx == 0) {
                return new PositionIndex[]{sortedByDistance.get(0), sortedByDistance.get(1)};
            } else if (idx >= n - 1) {
                return new PositionIndex[]{sortedByDistance.get(n - 2), sortedByDistance.get(n - 1)};
            } else {
                PositionIndex before = sortedByDistance.get(idx - 1);
                PositionIndex at = sortedByDistance.get(idx);
                PositionIndex after = sortedByDistance.get(idx + 1);

                double beforeDiff = Math.abs(before.distance - targetDist);
                double atDiff = Math.abs(at.distance - targetDist);
                double afterDiff = Math.abs(after.distance - targetDist);

                if (atDiff <= beforeDiff && atDiff <= afterDiff) {
                    return new PositionIndex[]{
                            idx > 0 ? sortedByDistance.get(idx - 1) : at,
                            idx < n - 1 ? sortedByDistance.get(idx + 1) : at
                    };
                } else if (beforeDiff <= afterDiff) {
                    return new PositionIndex[]{before, idx > 0 ? sortedByDistance.get(idx - 1) : before};
                } else {
                    return new PositionIndex[]{at, after};
                }
            }
        }

        private int binarySearchClosest(double target) {
            int lo = 0, hi = sortedByDistance.size() - 1;
            while (lo < hi) {
                int mid = (lo + hi + 1) >>> 1;
                if (sortedByDistance.get(mid).distance < target) {
                    lo = mid;
                } else {
                    hi = mid - 1;
                }
            }
            return lo;
        }

        double getInterpolant(double dist, PositionIndex[] near) {
            double dNear = near[0].distance, dFar = near[1].distance;
            if (dFar == dNear) return 0.0;
            return clamp01((dist - dNear) / (dFar - dNear));
        }

        List<ShooterDataPoint> getGroup(String k) {
            List<ShooterDataPoint> pts = allPointsByKey.get(k);
            return pts != null ? pts : Collections.<ShooterDataPoint>emptyList();
        }

        double interpolateGroupHood(String k, double rpm) {
            List<ShooterDataPoint> group = allPointsByKey.get(k);
            if (group == null || group.isEmpty()) return 0;

            List<ShooterDataPoint> sorted = new ArrayList<>(group);
            sorted.sort(Comparator.comparingDouble(p -> p.rpm));

            if (sorted.size() == 1) return sorted.get(0).hood;

            ShooterDataPoint low = sorted.get(0), high = sorted.get(sorted.size() - 1);
            for (int i = 0; i < sorted.size() - 1; i++) {
                if (rpm >= sorted.get(i).rpm && rpm <= sorted.get(i + 1).rpm) {
                    low = sorted.get(i);
                    high = sorted.get(i + 1);
                    break;
                }
            }
            if (high.rpm == low.rpm) return low.hood;
            return lerp(low.hood, high.hood, clamp01((rpm - low.rpm) / (high.rpm - low.rpm)));
        }

        double[] getRange(double dist, PositionIndex[] near, double t) {
            double nearMin = near[0].minRPM;
            double nearMax = near[0].maxRPM;
            double farMin = near[1].minRPM;
            double farMax = near[1].maxRPM;
            return new double[]{lerp(nearMin, farMin, t), lerp(nearMax, farMax, t)};
        }
    }

    private static final InterpolationTable TABLE = new InterpolationTable();

    public static double lastTargetDistance, lastBaseRPM, lastCompensatedRPM, lastSelectedHood;
    public static String lastClosestDistanceKey = null;
    public static int lastPointsCount;

    private static String key(double x, double y) { return x + "," + y; }

    static {
        TABLE.addPoint(62.3, 61.1, 2800, 0.15, RED_GOAL, Mode.CLOSE);
        TABLE.addPoint(62.3, 61.1, 2700, 0.1, RED_GOAL, Mode.CLOSE);

        TABLE.addPoint(70.4, 71.3, 2500, 0.02, RED_GOAL, Mode.CLOSE);
        TABLE.addPoint(70.4, 71.3, 2400, 0.01, RED_GOAL, Mode.CLOSE);

        TABLE.addPoint(81.3, 84.2, 2350, 0.015, RED_GOAL, Mode.CLOSE);
        TABLE.addPoint(81.3, 84.2, 2300, 0.01, RED_GOAL, Mode.CLOSE);

        TABLE.addPoint(93.1, 97.1, 2200, 0.012, RED_GOAL, Mode.CLOSE);
        TABLE.addPoint(93.1, 97.1, 2150, 0.010, RED_GOAL, Mode.CLOSE);

        TABLE.addPoint(100.8, 110.5, 2075, 0.01, RED_GOAL, Mode.CLOSE);

        TABLE.addPoint(141.5 / 2, 24, 3075, 0.2585, RED_GOAL, Mode.FAR);
        TABLE.addPoint(141.5 / 2, 24, 2975, 0.2125, RED_GOAL, Mode.FAR);
        TABLE.addPoint(141.5 / 2, 24, 2875, 0.1825, RED_GOAL, Mode.FAR);

        TABLE.addPoint(141.5 / 2 + 18, 0 + 8, 3025, 0.2635, RED_GOAL, Mode.FAR);
        TABLE.addPoint(141.5 / 2 + 18, 0 + 8, 2975, 0.2525, RED_GOAL, Mode.FAR);
        TABLE.addPoint(141.5 / 2 + 18, 0 + 8, 2895, 0.195, RED_GOAL, Mode.FAR);

        TABLE.addPoint(45, 0 + 8, 3275, 0.2535, RED_GOAL, Mode.FAR);
        TABLE.addPoint(45, 0 + 8, 3175, 0.2320, RED_GOAL, Mode.FAR);
        TABLE.addPoint(45, 0 + 8, 3125, 0.2025, RED_GOAL, Mode.FAR);
        TABLE.addPoint(45, 0 + 8, 3075, 0.1720, RED_GOAL, Mode.FAR);

        TABLE.addPoint(141.5 / 2, 0 + 8, 3055, 0.2530, RED_GOAL, Mode.FAR);
        TABLE.addPoint(141.5 / 2, 0 + 8, 3035, 0.2325, RED_GOAL, Mode.FAR);
        TABLE.addPoint(141.5 / 2, 0 + 8, 2975, 0.1725, RED_GOAL, Mode.FAR);

        TABLE.addPoint(76, 0 + 8, 3125, 0.2635, RED_GOAL, Mode.FAR);
        TABLE.addPoint(76, 0 + 8, 3025, 0.2325, RED_GOAL, Mode.FAR);
        TABLE.addPoint(76, 0 + 8, 2975, 0.1725, RED_GOAL, Mode.FAR);

        TABLE.buildIndex();
    }

    public static double getTargetRPM(double dist) {
        PositionIndex[] near = TABLE.getNearestTwo(dist);
        double t = TABLE.getInterpolant(dist, near);

        lastTargetDistance = dist;

        double rpmNear, rpmFar;
        if (activeMode == Mode.FAR) {
            rpmNear = near[0].maxRPM;
            rpmFar = near[1].maxRPM;
        } else {
            rpmNear = near[0].meanRPM;
            rpmFar = near[1].meanRPM;
        }

        lastBaseRPM = lerp(rpmNear, rpmFar, t);
        lastCompensatedRPM = lastBaseRPM;
        if (activeMode == Mode.FAR) {
            lastCompensatedRPM += farRPMOffset;
        }
        lastClosestDistanceKey = near[0].key;
        lastPointsCount = near[0].points.size() + near[1].points.size();
        return lastCompensatedRPM;
    }

    public static double getHoodPosition(double dist, double currentRPM) {
        PositionIndex[] near = TABLE.getNearestTwo(dist);
        double t = TABLE.getInterpolant(dist, near);
        lastSelectedHood = lerp(
                TABLE.interpolateGroupHood(near[0].key, currentRPM),
                TABLE.interpolateGroupHood(near[1].key, currentRPM), t);
        if (activeMode == Mode.FAR) {
            lastSelectedHood += farHoodOffset;
        }
        return lastSelectedHood;
    }

    public static double[] getRange(double dist) {
        PositionIndex[] near = TABLE.getNearestTwo(dist);
        double t = TABLE.getInterpolant(dist, near);
        return TABLE.getRange(dist, near, t);
    }

    private static double lerp(double a, double b, double t) { return a + (b - a) * t; }
    private static double clamp01(double v) { return Math.max(0, Math.min(1, v)); }
}
