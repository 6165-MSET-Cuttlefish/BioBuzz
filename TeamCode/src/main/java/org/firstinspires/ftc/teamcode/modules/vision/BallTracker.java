package org.firstinspires.ftc.teamcode.modules.vision;

import com.acmerobotics.dashboard.config.Config;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

/**
 * Associates per-frame {@link BallDetection}s with persistent tracks so each ball keeps an identity,
 * a smoothed field position, and a velocity across frames.
 *
 * <p>Association is greedy nearest-neighbour against each track's <em>predicted</em> position, which
 * is what lets a fast-moving ball stay matched: gating on the last seen position instead would need
 * a radius large enough to also swallow neighbouring balls.
 *
 * <p>Single-threaded — {@link BallDetectionPipeline} owns one instance and only ever calls it from
 * the camera thread.
 */
public final class BallTracker {

    @Config("BallTracking")
    public static class Tuning {
        /** Max field-inches between a prediction and a detection for them to be the same ball. */
        public static double matchRadiusIn = 8.0;
        /** Frames a track survives unseen before it is dropped. */
        public static int maxMisses = 8;
        /** Frames a track must be seen before it is published. */
        public static int minHits = 2;
        /** Position EMA toward the measurement, 0-1; higher is snappier and noisier. */
        public static double positionSmoothing = 0.6;
        /** Velocity EMA toward the frame-to-frame difference, 0-1. */
        public static double velocitySmoothing = 0.35;
        /** Speed at or above which a ball counts as moving, in/s. */
        public static double movingSpeedIn = 3.0;
        /** Measured speeds above this are treated as an association error and ignored, in/s. */
        public static double maxPlausibleSpeedIn = 250.0;
        /** Per-missed-frame velocity decay while coasting, so a lost track stops running away. */
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

        for (Pairing p : buildPairings(detections, dt)) {
            if (trackUsed[p.trackIndex] || detectionUsed[p.detectionIndex]) continue;
            trackUsed[p.trackIndex] = true;
            detectionUsed[p.detectionIndex] = true;
            tracks.get(p.trackIndex).hit(detections.get(p.detectionIndex), dt);
        }

        for (int i = tracks.size() - 1; i >= 0; i--) {
            if (trackUsed[i]) continue;
            if (!tracks.get(i).miss(dt)) tracks.remove(i);
        }

        for (int i = 0; i < detections.size(); i++) {
            if (!detectionUsed[i]) tracks.add(new Track(nextId++, detections.get(i), timestampSeconds));
        }

        return publish(timestampSeconds);
    }

    public void reset() {
        tracks.clear();
        lastUpdateSeconds = Double.NaN;
    }

    private List<Pairing> buildPairings(List<BallDetection> detections, double dt) {
        List<Pairing> pairings = new ArrayList<>();
        for (int t = 0; t < tracks.size(); t++) {
            Track track = tracks.get(t);
            for (int d = 0; d < detections.size(); d++) {
                BallDetection detection = detections.get(d);
                double distance = detection.distanceTo(track.predictedX(dt), track.predictedY(dt));
                if (distance <= Tuning.matchRadiusIn) pairings.add(new Pairing(t, d, distance));
            }
        }
        Collections.sort(pairings, new Comparator<Pairing>() {
            @Override public int compare(Pairing a, Pairing b) {
                return Double.compare(a.distance, b.distance);
            }
        });
        return pairings;
    }

    private List<TrackedBall> publish(double timestampSeconds) {
        List<TrackedBall> published = new ArrayList<>(tracks.size());
        for (Track t : tracks) {
            if (t.hits >= Tuning.minHits) published.add(t.snapshot(timestampSeconds));
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
        final double firstSeenSeconds;
        double x, y, vx, vy, radiusPx;
        int hits = 1;
        int misses = 0;
        boolean visible = true;

        Track(int id, BallDetection seed, double timestampSeconds) {
            this.id = id;
            this.firstSeenSeconds = timestampSeconds;
            this.x = seed.fieldX;
            this.y = seed.fieldY;
            this.radiusPx = seed.imageRadius;
        }

        double predictedX(double dt) { return x + vx * dt; }
        double predictedY(double dt) { return y + vy * dt; }

        void hit(BallDetection detection, double dt) {
            if (dt > 0) {
                double measuredVx = (detection.fieldX - x) / dt;
                double measuredVy = (detection.fieldY - y) / dt;
                if (Math.hypot(measuredVx, measuredVy) <= Tuning.maxPlausibleSpeedIn) {
                    vx += Tuning.velocitySmoothing * (measuredVx - vx);
                    vy += Tuning.velocitySmoothing * (measuredVy - vy);
                }
            }
            x += Tuning.positionSmoothing * (detection.fieldX - x);
            y += Tuning.positionSmoothing * (detection.fieldY - y);
            radiusPx += Tuning.positionSmoothing * (detection.imageRadius - radiusPx);
            hits++;
            misses = 0;
            visible = true;
        }

        /** Coasts one frame on the last known velocity; false once the track has expired. */
        boolean miss(double dt) {
            x += vx * dt;
            y += vy * dt;
            vx *= Tuning.coastDecay;
            vy *= Tuning.coastDecay;
            misses++;
            visible = false;
            return misses <= Tuning.maxMisses;
        }

        TrackedBall snapshot(double timestampSeconds) {
            return new TrackedBall(id, x, y, vx, vy, radiusPx, hits, misses,
                    timestampSeconds - firstSeenSeconds, visible);
        }
    }
}
