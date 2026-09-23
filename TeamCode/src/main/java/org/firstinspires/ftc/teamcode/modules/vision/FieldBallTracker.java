package org.firstinspires.ftc.teamcode.modules.vision;

import com.acmerobotics.dashboard.config.Config;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

/**
 * Keeps ball identity by field position so it survives the camera turning away. OpMode thread only.
 * Unseen balls hold their last position, deliberately not coasted, until forgotten.
 */
public final class FieldBallTracker {

    @Config("FieldBallTracking")
    public static class Tuning {
        public static double matchRadiusIn = 8.0;
        public static double forgetAfterSeconds = 10.0;
    }

    private final List<Known> known = new ArrayList<>();
    private int nextId = 1;

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

    public void reset() {
        known.clear();
    }

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
            lastSeenSeconds = timestampSeconds;
            visible = true;
        }

        FieldBall toFieldBall() {
            return new FieldBall(id, type, x, y, vx, vy, visible);
        }
    }
}
