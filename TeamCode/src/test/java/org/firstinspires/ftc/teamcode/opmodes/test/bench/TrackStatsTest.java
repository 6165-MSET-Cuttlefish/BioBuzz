package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.biobuzz.modules.vision.BallDetection;
import org.firstinspires.ftc.teamcode.biobuzz.modules.vision.BallTracker;
import org.firstinspires.ftc.teamcode.biobuzz.modules.vision.BallType;
import org.firstinspires.ftc.teamcode.biobuzz.modules.vision.TrackedBall;
import org.junit.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.Random;

public class TrackStatsTest {
    private static final double RADIUS = 6;
    private static final double FRAME_S = 0.233;

    private static double num(Map<String, Object> ball, String key) {
        return ((Number) ball.get(key)).doubleValue();
    }

    private static void frame(TrackStats s, double[]... tracks) {
        s.beginFrame();
        for (double[] t : tracks) s.add((int) t[0], t[1], t[2], t[3], t[4], t[5] != 0);
        s.endFrame();
    }

    @Test
    public void twoStillBallsKeepTheirIdsAndSpread() {
        TrackStats s = new TrackStats(RADIUS);
        for (int i = 0; i < 20; i++) {
            double j = i % 2 == 0 ? 0.4 : -0.4;
            frame(s, new double[] {1, 24 + j, 0, 0.5, 0, 1}, new double[] {2, 48, j, 0, -1.5, 1});
        }
        List<Map<String, Object>> balls = s.balls();
        assertEquals(20, s.frames());
        assertEquals(2, balls.size());
        assertEquals(24, num(balls.get(0), "meanXIn"), 1e-9);
        assertEquals(48, num(balls.get(1), "meanXIn"), 1e-9);
        for (Map<String, Object> b : balls) {
            assertEquals(0.4, num(b, "maxDeviationIn"), 1e-9);
            assertEquals(0.4, num(b, "sdIn"), 1e-9);
            assertEquals(0.8, num(b, "maxSpreadIn"), 1e-9);
            assertEquals(0, num(b, "idChanges"), 0);
            assertEquals(0, num(b, "framesMissing"), 0);
        }
        assertEquals(0.5, num(balls.get(0), "speedP95Ips"), 1e-9);
        assertEquals(1.5, num(balls.get(1), "speedP95Ips"), 1e-9);
    }

    @Test
    public void aNewIdAtTheSamePlaceIsAnIdChange() {
        TrackStats s = new TrackStats(RADIUS);
        for (int i = 0; i < 10; i++) frame(s, new double[] {i < 5 ? 1 : 5, 24, 0, 0, 0, 1});
        Map<String, Object> b = s.balls().get(0);
        assertEquals(1, num(b, "idChanges"), 0);
        assertEquals(Arrays.asList(1, 5), b.get("ids"));
    }

    @Test
    public void twoTracksOnOneBallInOneFrameCountOnceAsSeenAndAsAnIdChange() {
        TrackStats s = new TrackStats(RADIUS);
        frame(s, new double[] {1, 24, 0, 0, 0, 1});
        frame(s, new double[] {1, 24, 0, 0, 0, 1}, new double[] {2, 25, 0, 0, 0, 1});
        Map<String, Object> b = s.balls().get(0);
        assertEquals(1, s.balls().size());
        assertEquals(2, num(b, "framesSeen"), 0);
        assertEquals(1, num(b, "idChanges"), 0);
    }

    @Test
    public void missingAndCoastingFramesCountAsMissingAndAddNoSample() {
        TrackStats s = new TrackStats(RADIUS);
        frame(s, new double[] {1, 24, 0, 0, 0, 1});
        frame(s);
        frame(s, new double[] {1, 30, 0, 9, 0, 0});
        frame(s, new double[] {1, 24, 0, 0, 0, 1});
        Map<String, Object> b = s.balls().get(0);
        assertEquals(2, num(b, "framesMissing"), 0);
        assertEquals(2, num(b, "samples"), 0);
        assertEquals(0, num(b, "maxDeviationIn"), 1e-9);
        assertEquals(0, num(b, "speedMaxIps"), 1e-9);
    }

    @Test
    public void samplesFartherThanTheRadiusAreAnotherBall() {
        TrackStats s = new TrackStats(RADIUS);
        frame(s, new double[] {1, 24, 0, 0, 0, 1});
        frame(s, new double[] {1, 24 + RADIUS + 0.1, 0, 0, 0, 1});
        assertEquals(2, s.balls().size());
    }

    /** What a two-ball Limelight stream gives through the real tracker: still balls with a little detection noise. */
    private static TrackStats throughTheTracker(int frames, int dropFrom, int dropTo) {
        BallTracker tracker = new BallTracker();
        TrackStats s = new TrackStats(RADIUS);
        Random noise = new Random(6165);
        for (int i = 0; i < frames; i++) {
            List<BallDetection> detections = new ArrayList<>();
            detections.add(new BallDetection(BallType.POLLEN, 24 + noise.nextGaussian() * 0.3, noise.nextGaussian() * 0.3));
            if (i < dropFrom || i >= dropTo) {
                detections.add(new BallDetection(BallType.POLLEN, 48 + noise.nextGaussian() * 0.3,
                        noise.nextGaussian() * 0.3));
            }
            s.beginFrame();
            for (TrackedBall b : tracker.update(detections, i * FRAME_S)) s.add(b.id, b.x, b.y, b.vx, b.vy, b.visible());
            s.endFrame();
        }
        return s;
    }

    @Test
    public void twoStillBallsThroughTheRealTrackerHoldStillAndKeepTheirIds() {
        TrackStats s = throughTheTracker(129, Integer.MAX_VALUE, Integer.MAX_VALUE);
        List<Map<String, Object>> balls = s.balls();
        assertEquals(2, balls.size());
        for (Map<String, Object> b : balls) {
            assertTrue(b.toString(), num(b, "maxDeviationIn") < 2);
            assertTrue(b.toString(), num(b, "speedP95Ips") < 3);
            assertEquals(b.toString(), 0, num(b, "idChanges"), 0);
            assertEquals(b.toString(), BallTracker.Tuning.minHits - 1, num(b, "framesMissing"), 0);
        }
    }

    @Test
    public void aBallGoneLongerThanTheTrackerRemembersComesBackUnderANewId() {
        int gone = BallTracker.Tuning.maxMisses + 2;
        TrackStats s = throughTheTracker(129, 40, 40 + gone);
        Map<String, Object> far = s.balls().get(1);
        assertEquals(far.toString(), 1, num(far, "idChanges"), 0);
        assertTrue(far.toString(), num(far, "framesMissing") >= gone);
        assertEquals(0, num(s.balls().get(0), "idChanges"), 0);
    }
}
