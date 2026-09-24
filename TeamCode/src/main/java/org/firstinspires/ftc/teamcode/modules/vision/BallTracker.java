package org.firstinspires.ftc.teamcode.modules.vision;

import com.acmerobotics.dashboard.config.Config;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

/**
 * Camera-thread only. Tight pass against predicted positions, then a looser pass against last-known
 * positions so a jittery detection reacquires its track instead of forking a duplicate.
 */
public final class BallTracker {

    @Config("BallTracking")
    public static class Tuning {
        public static double matchRadiusIn = 8.0;
        /** Set equal to matchRadiusIn to disable; too wide merges nearby same-type balls. */
        public static double reacquireRadiusIn = 16.0;
        public static int maxMisses = 8;
        public static int minHits = 3;
        /** Alpha-beta gains, 0-1: how hard each detection corrects the predicted position / velocity. */
        public static double positionSmoothing = 0.6;
        public static double velocitySmoothing = 0.35;
        public static double movingSpeedIn = 3.0;
        /** in/s; faster measured speeds are treated as a mis-association and ignored. */
        public static double maxPlausibleSpeedIn = 250.0;
        /** Velocity multiplier per missed frame. */
        public static double coastDecay = 0.8;
    }

    private final List<Track> tracks = new ArrayList<>();
    private int nextId = 1;
    private double lastUpdateSeconds = Double.NaN;

    public List<TrackedBall> update(List<BallDetection> detections, double timestampSeconds) {
        double dt = Double.isNaN(lastUpdateSeconds) ? 0 : timestampSeconds - lastUpdateSeconds;
        lastUpdateSeconds = timestampSeconds;
        if (dt < 0) dt = 0;

        boolean[] detectionUsed = new boolean[detections.size()];
        boolean[] trackUsed = new boolean[tracks.size()];

        applyPairings(buildPairings(detections, detectionUsed, trackUsed, dt,
                Tuning.matchRadiusIn, false), detections, detectionUsed, trackUsed, dt);

        applyPairings(buildPairings(detections, detectionUsed, trackUsed, dt,
                Tuning.reacquireRadiusIn, true), detections, detectionUsed, trackUsed, dt);

        for (int i = tracks.size() - 1; i >= 0; i--) {
            if (trackUsed[i]) continue;
            if (!tracks.get(i).miss(dt)) tracks.remove(i);
        }

        for (int i = 0; i < detections.size(); i++) {
            if (!detectionUsed[i]) tracks.add(new Track(nextId++, detections.get(i)));
        }

        return publish();
    }

    public void reset() {
        tracks.clear();
        lastUpdateSeconds = Double.NaN;
    }

    private void applyPairings(List<Pairing> pairings, List<BallDetection> detections,
                                boolean[] detectionUsed, boolean[] trackUsed, double dt) {
        for (Pairing p : pairings) {
            if (trackUsed[p.trackIndex] || detectionUsed[p.detectionIndex]) continue;
            trackUsed[p.trackIndex] = true;
            detectionUsed[p.detectionIndex] = true;
            tracks.get(p.trackIndex).hit(detections.get(p.detectionIndex), dt);
        }
    }

    private List<Pairing> buildPairings(List<BallDetection> detections, boolean[] detectionUsed,
                                         boolean[] trackUsed, double dt, double radiusIn,
                                         boolean useLastKnownPosition) {
        List<Pairing> pairings = new ArrayList<>();
        for (int t = 0; t < tracks.size(); t++) {
            if (trackUsed[t]) continue;
            Track track = tracks.get(t);
            double px = useLastKnownPosition ? track.x : track.predictedX(dt);
            double py = useLastKnownPosition ? track.y : track.predictedY(dt);
            for (int d = 0; d < detections.size(); d++) {
                if (detectionUsed[d]) continue;
                BallDetection detection = detections.get(d);
                if (detection.type != track.type) continue;
                double distance = detection.distanceTo(px, py);
                if (distance <= radiusIn) pairings.add(new Pairing(t, d, distance));
            }
        }
        Collections.sort(pairings, new Comparator<Pairing>() {
            @Override public int compare(Pairing a, Pairing b) {
                return Double.compare(a.distance, b.distance);
            }
        });
        return pairings;
    }

    private List<TrackedBall> publish() {
        List<TrackedBall> published = new ArrayList<>(tracks.size());
        for (Track t : tracks) {
            if (t.hits >= Tuning.minHits) published.add(t.snapshot());
        }
        return published;
    }

    private static final class Pairing {
        final int trackIndex;
        final int detectionIndex;
        final double distance;

        Pairing(int trackIndex, int detectionIndex, double distance) {
            this.trackIndex = trackIndex;
            this.detectionIndex = detectionIndex;
            this.distance = distance;
        }
    }

    private static final class Track {
        final int id;
        final BallVisionConstants.BallType type;
        double x, y, vx, vy, radiusPx;
        int hits = 1;
        int misses = 0;
        boolean visible = true;

        Track(int id, BallDetection seed) {
            this.id = id;
            this.type = seed.type;
            this.x = seed.cameraX;
            this.y = seed.cameraY;
            this.radiusPx = seed.imageRadius;
        }

        double predictedX(double dt) { return x + vx * dt; }
        double predictedY(double dt) { return y + vy * dt; }

        // Alpha-beta: correct the prediction; differencing against the smoothed x inflates velocity 1/positionSmoothing.
        void hit(BallDetection detection, double dt) {
            double predictedX = predictedX(dt);
            double predictedY = predictedY(dt);
            double residualX = detection.cameraX - predictedX;
            double residualY = detection.cameraY - predictedY;
            if (dt > 0) {
                double measuredSpeed = Math.hypot(detection.cameraX - x, detection.cameraY - y) / dt;
                if (measuredSpeed <= Tuning.maxPlausibleSpeedIn) {
                    vx += Tuning.velocitySmoothing * residualX / dt;
                    vy += Tuning.velocitySmoothing * residualY / dt;
                }
            }
            x = predictedX + Tuning.positionSmoothing * residualX;
            y = predictedY + Tuning.positionSmoothing * residualY;
            radiusPx += Tuning.positionSmoothing * (detection.imageRadius - radiusPx);
            hits++;
            misses = 0;
            visible = true;
        }

        /** Coasts one frame; false once the track has expired. */
        boolean miss(double dt) {
            x += vx * dt;
            y += vy * dt;
            vx *= Tuning.coastDecay;
            vy *= Tuning.coastDecay;
            misses++;
            visible = false;
            return misses <= Tuning.maxMisses;
        }

        TrackedBall snapshot() {
            return new TrackedBall(id, type, x, y, vx, vy, radiusPx, visible);
        }
    }
}
