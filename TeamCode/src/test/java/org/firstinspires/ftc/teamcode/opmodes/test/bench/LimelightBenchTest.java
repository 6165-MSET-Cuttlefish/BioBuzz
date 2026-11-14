package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.biobuzz.modules.vision.LimelightSync;
import org.junit.Test;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.net.InetAddress;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;

public class LimelightBenchTest {

    @Test
    public void findsTheSourcesPrivateSyncRequest() {
        Field sync = LimelightBench.syncField();
        assertEquals(LimelightSync.Request.class, sync.getType());
        assertTrue(Modifier.isPrivate(sync.getModifiers()));
    }

    /** Answers every switch at once except the first, which it holds until released, as a slow POST would. */
    private static final class SlowFirstSwitch extends Limelight3A {
        final List<Integer> switches = new ArrayList<>();
        final CountDownLatch firstSent = new CountDownLatch(1);
        final CountDownLatch release = new CountDownLatch(1);
        private volatile int pipeline;

        SlowFirstSwitch() {
            super(null, "limelight", InetAddress.getLoopbackAddress());
        }

        @Override
        public boolean pipelineSwitch(int index) {
            boolean first;
            synchronized (switches) {
                first = switches.isEmpty();
                switches.add(index);
            }
            if (first) {
                firstSent.countDown();
                try {
                    release.await(5, TimeUnit.SECONDS);
                } catch (InterruptedException e) {
                    throw new IllegalStateException(e);
                }
            }
            pipeline = index;
            return true;
        }

        @Override
        public LLStatus getStatus() {
            return new LLStatus() {
                @Override
                public int getPipelineIndex() {
                    return pipeline;
                }

                @Override
                public String getPipelineType() {
                    return "pipe_python";
                }
            };
        }
    }

    @Test
    public void theRestoreSwitchLandsAfterASyncPostAlreadyInFlight() throws InterruptedException {
        SlowFirstSwitch ll = new SlowFirstSwitch();
        LimelightSync.Request sync = LimelightSync.request(ll, 4, false);
        assertTrue("the sync's switch was sent", ll.firstSent.await(5, TimeUnit.SECONDS));
        sync.cancel();
        LimelightSync.Request restore = LimelightBench.restorePipeline(ll, 2);
        Thread.sleep(200);
        ll.release.countDown();
        long deadline = System.nanoTime() + 10_000_000_000L;
        while (!restore.isDone() && System.nanoTime() < deadline) Thread.sleep(10);
        assertTrue("the restore finished", restore.isDone());
        assertEquals(null, restore.problem());
        synchronized (ll.switches) {
            assertEquals(Arrays.asList(4, 2), ll.switches);
        }
    }
}
