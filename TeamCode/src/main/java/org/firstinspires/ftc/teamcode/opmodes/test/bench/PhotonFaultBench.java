package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.architecture.hardware.photon.Photon;
import org.firstinspires.ftc.teamcode.architecture.hardware.photon.PhotonCore;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

@Photon
@TeleOp(name = "Bench: Photon Faults", group = "Test")
public class PhotonFaultBench extends LinearOpMode {
    private static final double WAIT_ABANDON_MS = 800;
    private static final double RECOVER_LIMIT_MS = 2000;
    private static final int LOST_DEPTH_LIMIT = 8;
    private static final long SUPERSEDE_AFTER_MS = 50;
    private static final double SUPERSEDE_SETTLE_MS = 400;

    private BenchReport report;
    private PhotonRig rig;

    @Override
    public void runOpMode() {
        report = new BenchReport("Photon Faults", "photon_faults_" + BenchIO.param(BenchIO.params(), "runId", "1"));
        rig = new PhotonRig(hardwareMap, report.params, "Bench: Photon Faults");
        int trials = (int) BenchIO.param(report.params, "trials", 3);
        int depthReps = (int) BenchIO.param(report.params, "depthReps", 13);
        int supersededReps = (int) BenchIO.param(report.params, "supersededReps", 10);
        report.put("info", rig.info());
        report.save();
        if (!report.holdUntilStart(this)) return;
        List<Map<String, Object>> lost = new ArrayList<>();
        List<Map<String, Object>> overLimit = new ArrayList<>();
        List<Map<String, Object>> throwing = new ArrayList<>();
        List<Map<String, Object>> amongWrites = new ArrayList<>();
        List<Map<String, Object>> superseded = new ArrayList<>();
        Map<String, Object> reset = new LinkedHashMap<>();
        report.put("lostWrite", lost);
        report.put("lostOverLimit", overLimit);
        report.put("writeThrows", throwing);
        report.put("lostAmongWrites", amongWrites);
        report.put("lostThenSuperseded", superseded);
        report.put("resetRecovery", reset);
        try {
            for (LynxModule hub : rig.hubs) {
                if (!PhotonRig.parallelCapable(hub)) continue;
                for (int t = 0; t < trials && opModeIsActive(); t++) {
                    for (PhotonChannel c : new PhotonChannel[] {rig.on(hub).get(t % 4), rig.on(hub).get(4 + t % 2)}) {
                        lost.add(lostWrite(hub, c, t));
                        report.save();
                    }
                }
                if (!opModeIsActive()) break;
                overLimit.add(lostOverLimit(hub));
                throwing.add(writeThrows(hub, rig.on(hub).get(1)));
                report.save();
                for (int depth = 0; depth < LOST_DEPTH_LIMIT && opModeIsActive(); depth++) {
                    for (int r = 0; r < depthReps && opModeIsActive(); r++) amongWrites.add(lostAmongWrites(hub, depth, r));
                    report.save();
                }
                List<PhotonChannel> on = rig.on(hub);
                for (int r = 0; r < supersededReps && opModeIsActive(); r++) {
                    superseded.add(lostThenSuperseded(hub, on.get(r % on.size()), r));
                }
                report.save();
            }
            for (LynxModule hub : rig.hubs) {
                if (!opModeIsActive()) break;
                Map<String, Object> byMode = new LinkedHashMap<>();
                for (int parallel : new int[] {0, 8}) byMode.put(PhotonWorkload.modeName(parallel), resetRecovery(hub, parallel));
                reset.put(rig.key(hub), byMode);
                report.save();
            }
        } finally {
            PhotonCore.disable();
            rig.zeroMotors();
            rig.disableServos();
        }
        report.done("lost writes " + lost.size() + ", over-limit trials " + overLimit.size());
        report.holdUntilStop(this);
    }

    private Map<String, Object> lostWrite(LynxModule hub, PhotonChannel c, int trial) {
        report.phase("lost write " + c.name + " " + trial);
        PhotonWorkload.setMode(8);
        Map<String, Object> r = new LinkedHashMap<>();
        r.put("hub", rig.key(hub));
        r.put("channel", c.name);
        r.put("kind", c.isMotor() ? "motor" : "servo");
        c.write();
        r.put("primeDrainMs", rig.drain(500));
        int primed = c.expected();
        r.put("primedLanded", c.read() == primed);
        LynxModule other = otherHub(hub);
        Map<String, Object> before = PhotonProbe.counters();
        LinkTap tap = LinkTap.install(hub);
        try {
            tap.swallowNext(1, c.isMotor() ? LinkTap.MOTORS : LinkTap.SERVOS, c.port);
            long t0 = System.nanoTime();
            c.write();
            r.put("writeCallMs", (System.nanoTime() - t0) / 1e6);
            int lostValue = c.expected();
            r.put("inFlightAfterWrite", PhotonProbe.inFlight(hub));
            r.put("registerKeptOldValue", c.read() == primed && primed != lostValue);
            Samples otherWrites = new Samples();
            double abandonMs = Double.NaN;
            while ((System.nanoTime() - t0) / 1e6 < WAIT_ABANDON_MS) {
                for (PhotonChannel o : rig.channels) {
                    if (o == c || o.hub != hub) continue;
                    long w = System.nanoTime();
                    o.write();
                    otherWrites.addNanos(System.nanoTime() - w);
                }
                if (other != null) {
                    for (PhotonChannel o : rig.on(other)) {
                        long w = System.nanoTime();
                        o.write();
                        otherWrites.addNanos(System.nanoTime() - w);
                    }
                }
                long abandoned = (Long) PhotonProbe.delta(before, PhotonProbe.counters()).get("abandonedWrites");
                if (abandoned > 0) {
                    abandonMs = (System.nanoTime() - t0) / 1e6;
                    break;
                }
                sleepMs(5);
            }
            r.put("abandonedAtMs", abandonMs);
            r.put("notRespondingRightAfter", hub.isNotResponding());
            r.put("inFlightAfterAbandon", PhotonProbe.inFlight(hub));
            r.put("pendingEntriesAfterAbandon", PhotonProbe.pendingEntries(hub));
            r.put("standInsAfterAbandon", PhotonProbe.standIns(hub));
            r.put("sdkCacheValidAfterAbandon", c.isMotor() ? PhotonProbe.motorCacheValid(c.motor) : PhotonProbe.servoCacheValid(c.servo));
            r.put("tapLog", tap.log());
            tap.disarm();
            r.put("rewriteMsAfterLostWrite", (System.nanoTime() - t0) / 1e6);
            c.set(c.commanded());
            r.put("respondingAgainMs", awaitResponding(hub));
            r.put("rewriteLanded", c.read() == lostValue);
            r.put("otherWriteMs", otherWrites.summary());
            r.put("otherHubAllAt", other == null ? null : allAt(other));
        } finally {
            tap.remove();
        }
        r.put("drainAfterMs", rig.drain(500));
        r.put("counters", PhotonProbe.delta(before, PhotonProbe.counters()));
        r.put("notRespondingAtEnd", hub.isNotResponding());
        PhotonCore.disable();
        rig.zeroMotors();
        return r;
    }

    private Map<String, Object> lostOverLimit(LynxModule hub) {
        report.phase("lost over limit " + rig.key(hub));
        int limit = 4;
        PhotonWorkload.setMode(limit);
        Map<String, Object> r = new LinkedHashMap<>();
        r.put("hub", rig.key(hub));
        r.put("limit", limit);
        List<PhotonChannel> motors = new ArrayList<>();
        List<PhotonChannel> servos = new ArrayList<>();
        for (PhotonChannel c : rig.on(hub)) (c.isMotor() ? motors : servos).add(c);
        for (PhotonChannel c : rig.on(hub)) c.write();
        r.put("primeDrainMs", rig.drain(500));
        int[] primed = new int[motors.size()];
        for (int i = 0; i < motors.size(); i++) primed[i] = motors.get(i).expected();
        Map<String, Object> before = PhotonProbe.counters();
        LinkTap tap = LinkTap.install(hub);
        try {
            int[] ports = new int[motors.size()];
            for (int i = 0; i < ports.length; i++) ports[i] = motors.get(i).port;
            tap.swallowNext(motors.size(), LinkTap.MOTORS, ports);
            long t0 = System.nanoTime();
            for (PhotonChannel c : motors) c.write();
            r.put("lostWritesCallMs", (System.nanoTime() - t0) / 1e6);
            r.put("inFlightWithAllLost", PhotonProbe.inFlight(hub));
            Samples overflow = new Samples();
            for (PhotonChannel s : servos) {
                long t = System.nanoTime();
                s.write();
                overflow.addNanos(System.nanoTime() - t);
            }
            r.put("overflowWriteMs", overflow.summary());
            r.put("slotTimeoutsSoFar", PhotonProbe.delta(before, PhotonProbe.counters()).get("slotTimeouts"));
            boolean kept = true;
            for (int i = 0; i < motors.size(); i++) kept &= motors.get(i).read() == primed[i];
            r.put("registersKeptOldValues", kept);
            double allAbandonedMs = Double.NaN;
            while ((System.nanoTime() - t0) / 1e6 < WAIT_ABANDON_MS) {
                long abandoned = (Long) PhotonProbe.delta(before, PhotonProbe.counters()).get("abandonedWrites");
                if (abandoned >= motors.size()) {
                    allAbandonedMs = (System.nanoTime() - t0) / 1e6;
                    break;
                }
                sleepMs(2);
            }
            r.put("allAbandonedAtMs", allAbandonedMs);
            r.put("inFlightAfterAbandon", PhotonProbe.inFlight(hub));
            r.put("standInsAfterAbandon", PhotonProbe.standIns(hub));
            r.put("tapLog", tap.log());
            tap.disarm();
            for (PhotonChannel c : motors) c.set(c.commanded());
            r.put("respondingAgainMs", awaitResponding(hub));
            boolean landed = true;
            for (PhotonChannel c : motors) landed &= c.read() == c.expected();
            r.put("rewritesLanded", landed);
        } finally {
            tap.remove();
        }
        r.put("drainAfterMs", rig.drain(500));
        r.put("counters", PhotonProbe.delta(before, PhotonProbe.counters()));
        PhotonCore.disable();
        rig.zeroMotors();
        return r;
    }

    private Map<String, Object> lostAmongWrites(LynxModule hub, int depth, int rep) {
        List<PhotonChannel> on = rig.on(hub);
        PhotonChannel c = on.get(rep % on.size());
        List<PhotonChannel> others = new ArrayList<>();
        for (PhotonChannel o : on) if (o != c) others.add(o);
        report.phase("lost write at depth " + depth + " " + rig.key(hub) + " " + rep);
        PhotonWorkload.setMode(LOST_DEPTH_LIMIT);
        Map<String, Object> r = new LinkedHashMap<>();
        r.put("hub", rig.key(hub));
        r.put("channel", c.name);
        r.put("kind", c.isMotor() ? "motor" : "servo");
        r.put("depth", depth);
        c.write();
        for (PhotonChannel o : others) o.write();
        r.put("primeDrainMs", rig.drain(500));
        int primed = c.expected();
        Map<String, Object> before = PhotonProbe.counters();
        LinkTap tap = LinkTap.install(hub);
        try {
            tap.swallowNext(1, c.isMotor() ? LinkTap.MOTORS : LinkTap.SERVOS, c.port);
            for (int i = 0; i < depth; i++) others.get(i % others.size()).write();
            r.put("inFlightBeforeLostWrite", PhotonProbe.inFlight(hub));
            long t0 = System.nanoTime();
            c.write();
            r.put("writeCallMs", (System.nanoTime() - t0) / 1e6);
            int lostValue = c.expected();
            Samples busyWrites = new Samples();
            double abandonMs = Double.NaN;
            while ((System.nanoTime() - t0) / 1e6 < WAIT_ABANDON_MS) {
                for (PhotonChannel o : others) {
                    long w = System.nanoTime();
                    o.write();
                    busyWrites.addNanos(System.nanoTime() - w);
                }
                if ((Long) PhotonProbe.delta(before, PhotonProbe.counters()).get("abandonedWrites") > 0) {
                    abandonMs = (System.nanoTime() - t0) / 1e6;
                    break;
                }
                sleepMs(2);
            }
            tap.disarm();
            r.put("abandonedAtMs", abandonMs);
            r.put("busyWriteMs", busyWrites.summary());
            r.put("tapLog", tap.log());
            r.put("drainAfterAbandonMs", rig.drain(100));
            r.put("leftoversAfterAbandon", PhotonProbe.leftovers(hub));
            r.put("registerKeptOldValue", c.read() == primed && primed != lostValue);
            r.put("rewriteMsAfterLostWrite", (System.nanoTime() - t0) / 1e6);
            c.set(c.commanded());
            r.put("respondingAgainMs", awaitResponding(hub));
            r.put("rewriteLanded", c.read() == lostValue);
            boolean othersAt = true;
            for (PhotonChannel o : others) othersAt &= o.read() == o.expected();
            r.put("othersLanded", othersAt);
        } finally {
            tap.remove();
        }
        r.put("drainAfterMs", rig.drain(500));
        r.put("counters", PhotonProbe.delta(before, PhotonProbe.counters()));
        r.put("notRespondingAtEnd", hub.isNotResponding());
        PhotonCore.disable();
        rig.zeroMotors();
        return r;
    }

    private Map<String, Object> lostThenSuperseded(LynxModule hub, PhotonChannel c, int rep) {
        report.phase("lost write superseded " + rig.key(hub) + " " + c.name + " " + rep);
        PhotonWorkload.setMode(8);
        Map<String, Object> r = new LinkedHashMap<>();
        r.put("hub", rig.key(hub));
        r.put("channel", c.name);
        r.put("kind", c.isMotor() ? "motor" : "servo");
        c.write();
        r.put("primeDrainMs", rig.drain(500));
        int primed = c.expected();
        Map<String, Object> before = PhotonProbe.counters();
        LinkTap tap = LinkTap.install(hub);
        try {
            tap.swallowNext(1, c.isMotor() ? LinkTap.MOTORS : LinkTap.SERVOS, c.port);
            long t0 = System.nanoTime();
            c.write();
            int lostValue = c.expected();
            sleepMs(SUPERSEDE_AFTER_MS);
            r.put("registerKeptOldValue", c.read() == primed && primed != lostValue);
            r.put("newerWriteAtMs", (System.nanoTime() - t0) / 1e6);
            c.write();
            int newer = c.expected();
            while ((System.nanoTime() - t0) / 1e6 < SUPERSEDE_SETTLE_MS) sleepMs(5);
            r.put("leftovers", PhotonProbe.leftovers(hub));
            r.put("tapLog", tap.log());
            tap.disarm();
            r.put("newerLanded", c.read() == newer);
            r.put("notResponding", hub.isNotResponding());
        } finally {
            tap.remove();
        }
        r.put("counters", PhotonProbe.delta(before, PhotonProbe.counters()));
        PhotonCore.disable();
        rig.zeroMotors();
        return r;
    }

    // Enables and targets re-sent first so the SDK's 500 ms cache timers start together with Photon off and on.
    private Map<String, Object> resetRecovery(LynxModule hub, int parallel) {
        report.phase("reset recovery " + rig.key(hub) + " " + PhotonWorkload.modeName(parallel));
        PhotonWorkload.setMode(parallel);
        List<PhotonChannel> held = new ArrayList<>();
        for (PhotonChannel c : rig.on(hub)) held.add(c.port == 0 ? c.throughWriteCache(hardwareMap) : c);
        double[] targets = new double[held.size()];
        for (int i = 0; i < held.size(); i++) targets[i] = held.get(i).isMotor() ? 0.22 : 0.42;
        for (int i = 0; i < held.size(); i++) {
            PhotonChannel c = held.get(i);
            if (c.isMotor()) {
                c.motor.setMotorDisable();
                c.motor.setMotorEnable();
            } else {
                c.servo.setPwmDisable();
                c.servo.setPwmEnable();
            }
            c.set(targets[i] + 0.01);
            c.set(targets[i]);
        }
        long settleEnd = System.nanoTime() + 300_000_000L;
        while (System.nanoTime() < settleEnd) {
            for (int i = 0; i < held.size(); i++) held.get(i).set(targets[i]);
            sleepMs(5);
        }
        Map<String, Object> r = new LinkedHashMap<>();
        r.put("photon", parallel);
        boolean settled = true;
        for (PhotonChannel c : held) settled &= c.read() == c.expected() && c.enabled();
        r.put("settledBeforeReset", settled);
        PhotonChannel rawMotor = held.get(1);
        r.put("sdkCacheValidBeforeReset", PhotonProbe.motorCacheValid(rawMotor.motor));
        Map<String, Object> before = PhotonProbe.counters();
        HubRegisters.rawFailSafe(hub);
        long t0 = System.nanoTime();
        r.put("sdkCacheValidAfterRawFailSafe", PhotonProbe.motorCacheValid(rawMotor.motor));
        r.put("attentionInFailSafeAck", PhotonProbe.attentionRequired(hub));
        double attentionMs = Double.NaN;
        double[] recovered = new double[held.size()];
        for (int i = 0; i < recovered.length; i++) recovered[i] = Double.NaN;
        boolean disabledSeen = false;
        int loops = 0;
        while ((System.nanoTime() - t0) / 1e6 < RECOVER_LIMIT_MS && opModeIsActive()) {
            for (int i = 0; i < held.size(); i++) held.get(i).set(targets[i]);
            if (Double.isNaN(attentionMs) && PhotonProbe.attentionRequired(hub)) attentionMs = (System.nanoTime() - t0) / 1e6;
            boolean all = true;
            for (int i = 0; i < held.size(); i++) {
                PhotonChannel c = held.get(i);
                boolean on = c.enabled();
                if (!on) disabledSeen = true;
                boolean at = on && c.read() == c.expected();
                if (at && Double.isNaN(recovered[i])) recovered[i] = (System.nanoTime() - t0) / 1e6;
                all &= !Double.isNaN(recovered[i]);
            }
            loops++;
            if (all && loops > 3) break;
        }
        r.put("resetSeen", disabledSeen);
        r.put("attentionSeenMs", attentionMs);
        Map<String, Object> per = new LinkedHashMap<>();
        for (int i = 0; i < held.size(); i++) per.put(held.get(i).name + (held.get(i).wrapped ? " (WriteCache)" : ""), recovered[i]);
        r.put("recoveredMs", per);
        r.put("loops", loops);
        r.put("drainAfterMs", rig.drain(500));
        r.put("counters", PhotonProbe.delta(before, PhotonProbe.counters()));
        PhotonCore.disable();
        rig.zeroMotors();
        for (PhotonChannel c : held) if (c.isMotor() && c.wrapped) c.set(0);
        return r;
    }

    private double awaitResponding(LynxModule hub) {
        long t0 = System.nanoTime();
        while (hub.isNotResponding() && (System.nanoTime() - t0) / 1e6 < 500) sleepMs(1);
        if (hub.isNotResponding()) throw new IllegalStateException("REV Hub #" + hub.getModuleAddress() + " still not responding 500 ms after a re-write");
        return (System.nanoTime() - t0) / 1e6;
    }

    private Map<String, Object> writeThrows(LynxModule hub, PhotonChannel c) {
        report.phase("write throws " + rig.key(hub));
        PhotonWorkload.setMode(8);
        Map<String, Object> r = new LinkedHashMap<>();
        r.put("hub", rig.key(hub));
        r.put("channel", c.name);
        c.write();
        rig.drain(500);
        Map<String, Object> before = PhotonProbe.counters();
        LinkTap tap = LinkTap.install(hub);
        try {
            tap.throwOnNext(LinkTap.MOTORS, c.port);
            long t0 = System.nanoTime();
            c.write();
            r.put("writeCallMs", (System.nanoTime() - t0) / 1e6);
            r.put("thrown", tap.swallowedCount());
            tap.disarm();
        } finally {
            tap.remove();
        }
        r.put("drainAfterMs", rig.drain(500));
        r.put("landed", c.read() == c.expected());
        r.put("inFlightAfter", PhotonProbe.inFlight(hub));
        r.put("notResponding", hub.isNotResponding());
        r.put("shutdownReason", PhotonProbe.impl(hub).getShutdownReason().toString());
        r.put("counters", PhotonProbe.delta(before, PhotonProbe.counters()));
        PhotonCore.disable();
        rig.zeroMotors();
        return r;
    }

    private boolean allAt(LynxModule hub) {
        rig.drain(500);
        for (PhotonChannel c : rig.on(hub)) if (c.read() != c.expected()) return false;
        return true;
    }

    private LynxModule otherHub(LynxModule hub) {
        for (LynxModule h : rig.hubs) if (h != hub && PhotonRig.parallelCapable(h)) return h;
        return null;
    }

    private void sleepMs(long ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            throw new IllegalStateException("interrupted in Bench: Photon Faults", e);
        }
    }
}
