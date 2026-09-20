package org.firstinspires.ftc.teamcode.modules.vision;

import com.acmerobotics.dashboard.config.Config;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

/**
 * Gives ball identity persistence across a gap where the camera loses sight of a ball entirely — a
 * robot turn, not just a flickery detection — something {@link BallTracker} can't do on its own: its
 * tracks are camera-relative ("rigidly attached to the robot" — see {@link TrackedBall}), so they go
 * stale the instant the robot rotates, and it drops a track after a handful of missed frames
 * regardless. This runs one layer up, re-labelling the {@link FieldBall} list {@link
 * BallFieldTransform} already computes every frame with a persistent identity, using field position
 * (which a stationary ball keeps regardless of where the camera is pointed) instead of camera
 * position. It does no position smoothing, coordinate transforms, or velocity estimation of its own
 * — all of that already happened upstream — so per ball this is just a distance check.
 *
 * <p>A known ball missing from the current frame is kept, unchanged, reporting its last known
 * position with {@link FieldBall#visible()} false, until {@link Tuning#forgetAfterSeconds} passes
 * with no re-detection. Deliberately not coasted forward on its old velocity: the point is
 * remembering where a (likely now-stationary) ball was, not extrapolating where it went.
 *
 * <p>Runs on the OpMode thread (inside {@code Camera.read()}), never the camera thread. Cost per
 * call is O(known balls x this frame's balls) comparisons over a handful of small game pieces —
 * negligible regardless of camera resolution or detection load.
 */
public final class FieldBallTracker {

    @Config("FieldBallTracking")
    public static class Tuning {
        /** Max field-inches between a known ball's last position and a fresh one to call them the same. */
        public static double matchRadiusIn = 8.0;
        /** A known ball not re-detected within this long is forgotten entirely. */
        public static double forgetAfterSeconds = 10.0;
    }

    private final List<Known> known = new ArrayList<>();
    private int nextId = 1;

    /** Re-labels this frame's FieldBalls with persistent identity. Safe to call every loop. */
    public List<FieldBall> update(List<FieldBall> fresh, double timestampSeconds) {
        for (Known k : known) k.visible = false;

        boolean[] freshUsed = new boolean[fresh.size()];
        boolean[] knownUsed = new boolean[known.size()];
        for (Pairing p : buildPairings(fresh)) {
            if (knownUsed[p.knownIndex] || freshUsed[p.freshIndex]) continue;
            knownUsed[p.knownIndex] = true;
            freshUsed[p.freshIndex] = true;
            known.get(p.knownIndex).absorb(fresh.get(p.freshIndex), timestampSeconds);
        }

        for (int i = known.size() - 1; i >= 0; i--) {
            if (timestampSeconds - known.get(i).lastSeenSeconds > Tuning.forgetAfterSeconds) {
                known.remove(i);
            }
        }

        for (int i = 0; i < fresh.size(); i++) {
            if (!freshUsed[i]) known.add(new Known(nextId++, fresh.get(i), timestampSeconds));
        }

        List<FieldBall> result = new ArrayList<>(known.size());
        for (Known k : known) result.add(k.toFieldBall());
        return result;
    }

    /** Forget every known ball — use when the robot state source itself changes, not after a move. */
    public void reset() {
        known.clear();
    }

    /** Never pairs across {@link BallVisionConstants.BallType}, same reasoning as {@link BallTracker}. */
    private List<Pairing> buildPairings(List<FieldBall> fresh) {
        List<Pairing> pairings = new ArrayList<>();
        for (int k = 0; k < known.size(); k++) {
            Known kb = known.get(k);
            for (int f = 0; f < fresh.size(); f++) {
                FieldBall fb = fresh.get(f);
                if (fb.type != kb.type) continue;
                double distance = fb.distanceTo(kb.x, kb.y);
                if (distance <= Tuning.matchRadiusIn) pairings.add(new Pairing(k, f, distance));
            }
        }
        Collections.sort(pairings, new Comparator<Pairing>() {
            @Override public int compare(Pairing a, Pairing b) {
                return Double.compare(a.distance, b.distance);
            }
        });
        return pairings;
    }

    private static final class Pairing {
        final int knownIndex, freshIndex;
        final double distance;

        Pairing(int knownIndex, int freshIndex, double distance) {
            this.knownIndex = knownIndex;
            this.freshIndex = freshIndex;
            this.distance = distance;
        }
    }

    private static final class Known {
        final int id;
        final BallVisionConstants.BallType type;
        double x, y, vx, vy;
        double lastSeenSeconds;
        TrackedBall source;
        boolean visible;

        Known(int id, FieldBall seed, double timestampSeconds) {
            this.id = id;
            this.type = seed.type;
            absorb(seed, timestampSeconds);
        }

        void absorb(FieldBall fresh, double timestampSeconds) {
            x = fresh.x;
            y = fresh.y;
            vx = fresh.vx;
            vy = fresh.vy;
            source = fresh.source;
            lastSeenSeconds = timestampSeconds;
            visible = true;
        }

        FieldBall toFieldBall() {
            return new FieldBall(id, type, x, y, vx, vy, source, visible);
        }
    }
}
