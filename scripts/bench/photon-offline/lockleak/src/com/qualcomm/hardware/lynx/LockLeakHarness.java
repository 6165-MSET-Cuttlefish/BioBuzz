package com.qualcomm.hardware.lynx;

import android.util.Log;

import com.qualcomm.hardware.lynx.commands.LynxMessage;
import com.qualcomm.hardware.lynx.commands.core.LynxSetMotorConstantPowerCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxSetServoEnableCommand;
import com.qualcomm.hardware.lynx.commands.standard.LynxFailSafeCommand;
import com.qualcomm.hardware.lynx.commands.standard.LynxGetModuleStatusCommand;
import com.qualcomm.robotcore.hardware.EmbeddedControlHubModule;

import org.firstinspires.ftc.teamcode.architecture.hardware.photon.PhotonCore;
import org.firstinspires.ftc.teamcode.architecture.hardware.photon.PhotonLynxModule;

import java.lang.reflect.Field;
import java.util.concurrent.atomic.AtomicReference;

/**
 * The SDK 12 network-lock leak a LinearOpMode's STOP interrupt can cause, on real SDK classes against PhotonHarness's
 * fake hub. ArmedLock interrupts the calling thread at the top of MessageKeyedLock.release(), which is exactly where
 * LinearOpMode's shutdownNow() lands when it arrives after a command's last interruptible wait and before its release.
 */
public class LockLeakHarness {
    static int passes;
    static int failures;

    static void check(boolean ok, String what) {
        System.out.println((ok ? "PASS " : "FAIL ") + what);
        if (ok) passes++; else failures++;
    }

    static final class ArmedLock extends MessageKeyedLock {
        volatile boolean armed;

        ArmedLock() {
            super("lynx xmit lock", 500);
        }

        @Override
        public void release(LynxMessage message) throws InterruptedException {
            if (armed) {
                armed = false;
                Thread.currentThread().interrupt();
            }
            super.release(message);
        }
    }

    static ArmedLock install(PhotonHarness.Rig rig) throws Exception {
        ArmedLock lock = new ArmedLock();
        PhotonHarness.set(rig.impl, LynxUsbDeviceImpl.class, "networkTransmissionLock", lock);
        return lock;
    }

    static Object owner(MessageKeyedLock lock) throws Exception {
        return PhotonHarness.get(lock, MessageKeyedLock.class, "lockOwner");
    }

    static int abandons() {
        int n = 0;
        for (String l : Log.LINES) if (l.contains("abandoning lock")) n++;
        return n;
    }

    static int nackLines() {
        int n = 0;
        for (String l : Log.LINES) if (l.contains("nack rec'd") && l.contains("CANCELLED_FOR_SAFETY")) n++;
        return n;
    }

    static String lastAbandon() {
        String last = null;
        for (String l : Log.LINES) if (l.contains("abandoning lock")) last = l;
        return last;
    }

    /** Milliseconds another thread waits to get a non-dangerous command through the hub's network lock. */
    static double contend(LynxModule hub) throws Exception {
        AtomicReference<Throwable> err = new AtomicReference<>();
        long t0 = System.nanoTime();
        Thread t = new Thread(() -> {
            try {
                new LynxGetModuleStatusCommand(hub).sendReceive();
            } catch (Throwable e) {
                err.set(e);
            }
        }, "contender");
        t.start();
        t.join(3000);
        if (err.get() != null) throw new IllegalStateException("contender failed", err.get());
        return (System.nanoTime() - t0) / 1e6;
    }

    static Throwable sendServoEnable(LynxModule hub) {
        try {
            new LynxSetServoEnableCommand(hub, 0, true).send();
            return null;
        } catch (Throwable t) {
            return t;
        }
    }

    static Throwable power(LynxModule hub, int channel, int power) {
        try {
            new LynxSetMotorConstantPowerCommand(hub, channel, power).send();
            return null;
        } catch (Throwable t) {
            return t;
        }
    }

    public static void main(String[] args) {
        try {
            run();
        } catch (Throwable t) {
            t.printStackTrace(System.out);
            failures++;
        }
        System.out.println(passes + " passed, " + failures + " failed");
        System.exit(failures == 0 ? 0 : 1);
    }

    static void run() throws Exception {
        Field u = sun.misc.Unsafe.class.getDeclaredField("theUnsafe");
        u.setAccessible(true);
        PhotonHarness.UNSAFE = (sun.misc.Unsafe) u.get(null);
        PhotonHarness.prevent(false);

        PhotonHarness.Rig stockRig = new PhotonHarness.Rig(2);
        LynxModule stock = stockRig.originals.get(0);
        ArmedLock lock = install(stockRig);
        check(!(stock instanceof PhotonLynxModule) && contend(stock) < 100 && abandons() == 0,
                "stock: a plain SDK LynxModule (no Photon class anywhere) takes and frees its network lock");

        PhotonHarness.prevent(true);
        int written = stockRig.hub.written.size();
        lock.armed = true;
        Throwable thrown = sendServoEnable(stock);
        Object leaked = owner(lock);
        boolean flagAfter = Thread.interrupted();
        boolean nothingSent = stockRig.hub.written.size() == written;
        double waited = contend(stock);
        check(thrown instanceof InterruptedException && leaked instanceof LynxSetServoEnableCommand && nothingSent
                        && !flagAfter,
                "stock, STOP gate up: an interrupt landing after the gate's NACK and before the release throws "
                        + "InterruptedException out of send(), leaves the lock owned by the NACKed command, sends nothing, "
                        + "and leaves the flag clear (" + thrown + ")");
        check(waited >= 450 && waited < 800 && abandons() == 1
                        && lastAbandon().contains("abandoning lock: old=LynxSetServoEnableCommand(0)"),
                String.format("stock, STOP gate up: the next command from another thread waits %.0f ms and logs trial "
                        + "018's line: %s", waited, lastAbandon()));
        PhotonHarness.prevent(false);

        lock.armed = true;
        thrown = power(stock, 0, 1000);
        leaked = owner(lock);
        Thread.interrupted();
        waited = contend(stock);
        check(thrown instanceof InterruptedException && leaked instanceof LynxSetMotorConstantPowerCommand
                        && waited >= 450 && abandons() == 2,
                String.format("stock, gate down: an ACKed write leaks the lock the same way (%.0f ms)", waited));

        stockRig.hub.ackDelayMs = 40;
        AtomicReference<Throwable> senderErr = new AtomicReference<>();
        Thread sender = new Thread(() -> senderErr.set(power(stock, 1, 1100)), "interrupted sender");
        sender.start();
        PhotonHarness.sleep(10);
        sender.interrupt();
        sender.join(2000);
        PhotonHarness.sleep(60);
        stockRig.hub.ackDelayMs = 2;
        Object ownerAfter = owner(lock);
        waited = contend(stock);
        check(senderErr.get() instanceof InterruptedException && ownerAfter == null && waited < 100 && abandons() == 2,
                String.format("stock, negative control: an interrupt landing during the ACK wait is consumed by the "
                        + "wait, the release succeeds, nothing leaks (next command %.1f ms)", waited));

        PhotonHarness.prevent(true);
        written = stockRig.hub.written.size();
        int nacksBefore = nackLines();
        Thread.currentThread().interrupt();
        thrown = power(stock, 2, 1200);
        Thread.currentThread().interrupt();
        Throwable thrown2 = sendServoEnable(stock);
        Thread.interrupted();
        check(thrown instanceof InterruptedException && thrown2 instanceof InterruptedException && owner(lock) == null
                        && stockRig.hub.written.size() == written && nackLines() == nacksBefore
                        && contend(stock) < 100 && abandons() == 2,
                "stock, negative control: with the flag already set, as the SDK's controllers leave it for a "
                        + "LinearOpMode's writes after STOP, a write throws at the lock acquisition before reaching "
                        + "the STOP gate (no NACK logged), holds nothing and leaks nothing");
        PhotonHarness.prevent(false);

        PhotonHarness.Rig photonRig = new PhotonHarness.Rig(3);
        EmbeddedControlHubModule.set(photonRig.originals.get(0));
        check(photonRig.takeOver().isEmpty(), "Photon rig taken over");
        PhotonLynxModule photon = photonRig.photon(3);
        ArmedLock plock = install(photonRig);

        PhotonHarness.enablePhoton(false);
        PhotonHarness.prevent(true);
        int before = abandons();
        plock.armed = true;
        thrown = sendServoEnable(photon);
        leaked = owner(plock);
        Thread.interrupted();
        waited = contend(photon);
        check(thrown instanceof InterruptedException && leaked instanceof LynxSetServoEnableCommand && waited >= 450
                        && abandons() == before + 1 && lastAbandon().contains("old=LynxSetServoEnableCommand(0)"),
                String.format("Photon off: the same injected interrupt leaks the lock exactly as the stock SDK does "
                        + "(%.0f ms)", waited));

        PhotonHarness.enablePhoton(true);
        before = abandons();
        plock.armed = true;
        thrown = sendServoEnable(photon);
        leaked = owner(plock);
        Thread.interrupted();
        waited = contend(photon);
        check(thrown instanceof InterruptedException && leaked instanceof LynxSetServoEnableCommand && waited >= 450
                        && abandons() == before + 1,
                String.format("Photon on, gate up: a gated SDK-path command leaks the same way, no worse (%.0f ms)", waited));

        PhotonCore.resetCounters();
        written = photonRig.hub.written.size();
        nacksBefore = nackLines();
        Thread.currentThread().interrupt();
        thrown = power(photon, 0, 1250);
        Thread.interrupted();
        check(thrown instanceof InterruptedException && photonRig.hub.written.size() == written
                        && nackLines() == nacksBefore && PhotonCore.counters().gateFallbacks == 1
                        && PhotonCore.counters().parallelWrites == 0 && owner(plock) == null,
                "Photon on, gate up, flag already set: a late write is routed off the parallel path and then throws at "
                        + "the SDK lock acquisition, so it never reaches the SDK's STOP gate either");
        PhotonHarness.prevent(false);

        PhotonCore.resetCounters();
        written = photonRig.hub.written.size();
        Thread.currentThread().interrupt();
        thrown = power(photon, 0, 1260);
        boolean stillSet = Thread.interrupted();
        PhotonHarness.sleep(20);
        check(thrown instanceof InterruptedException && PhotonCore.counters().parallelWrites == 1
                        && photonRig.hub.written.size() == written + 1,
                "Photon on, gate down, flag already set: unlike the SDK path, the parallel write still goes out before "
                        + "send() throws in its ACK wait (flag left set " + stillSet + ")");

        AtomicReference<Object> ownerDuringWrite = new AtomicReference<>("unset");
        photonRig.hub.onWrite = d -> {
            if (d.getCommandNumber() == new LynxSetMotorConstantPowerCommand(photon, 0, 1).getCommandNumber()) {
                try {
                    ownerDuringWrite.set(owner(plock));
                } catch (Exception e) {
                    throw new RuntimeException(e);
                }
                Thread.currentThread().interrupt();
            }
        };
        PhotonCore.resetCounters();
        before = abandons();
        thrown = power(photon, 0, 1300);
        photonRig.hub.onWrite = d -> {};
        Thread.interrupted();
        PhotonHarness.sleep(20);
        waited = contend(photon);
        check(PhotonCore.counters().parallelWrites == 1 && ownerDuringWrite.get() == null && owner(plock) == null
                        && waited < 100 && abandons() == before,
                String.format("Photon on, gate down: a parallel write never holds the SDK lock, so an interrupt "
                        + "inside it can't leak it (%s; next command %.1f ms)", thrown, waited));

        int motorCmd = new LynxSetMotorConstantPowerCommand(photon, 0, 1).getCommandNumber();
        photonRig.hub.onWrite = d -> {
            if (d.getCommandNumber() == motorCmd && d.getPayloadData()[1] != 0) {
                try {
                    PhotonHarness.prevent(true);
                } catch (Exception e) {
                    throw new RuntimeException(e);
                }
                plock.armed = true;
            }
        };
        PhotonCore.resetCounters();
        before = abandons();
        thrown = power(photon, 1, 1400);
        photonRig.hub.onWrite = d -> {};
        leaked = owner(plock);
        Thread.interrupted();
        waited = contend(photon);
        check(PhotonCore.counters().postStopFailSafes == 1 && thrown instanceof InterruptedException
                        && leaked instanceof LynxFailSafeCommand && waited >= 450 && abandons() == before + 1,
                String.format("Photon on, STOP landing just after a parallel write went out: Photon's own post-STOP "
                        + "fail-safe is an SDK-locked command on the writer's thread, so it is one more place the "
                        + "interrupt can leak the lock (owner %s, %.0f ms)",
                        leaked == null ? null : leaked.getClass().getSimpleName(), waited));
        PhotonHarness.prevent(false);
    }
}
