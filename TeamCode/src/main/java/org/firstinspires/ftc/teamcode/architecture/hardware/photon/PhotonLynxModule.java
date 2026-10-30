package org.firstinspires.ftc.teamcode.architecture.hardware.photon;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.lynx.LynxCommExceptionHandler;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxModuleWarningManager;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.LynxUnsupportedCommandException;
import com.qualcomm.hardware.lynx.LynxUsbDevice;
import com.qualcomm.hardware.lynx.LynxUsbDeviceImpl;
import com.qualcomm.hardware.lynx.Supplier;
import com.qualcomm.hardware.lynx.commands.LynxDatagram;
import com.qualcomm.hardware.lynx.commands.LynxMessage;
import com.qualcomm.hardware.lynx.commands.LynxRespondable;
import com.qualcomm.hardware.lynx.commands.core.LynxSetMotorConstantPowerCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxSetServoPulseWidthCommand;
import com.qualcomm.hardware.lynx.commands.standard.LynxAck;
import com.qualcomm.hardware.lynx.commands.standard.LynxFailSafeCommand;
import com.qualcomm.hardware.lynx.commands.standard.LynxNack;
import com.qualcomm.robotcore.eventloop.SyncdDevice;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.usb.RobotArmingStateNotifier;
import com.qualcomm.robotcore.hardware.usb.RobotUsbDevice;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.usb.exception.RobotUsbException;

import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.Arrays;
import java.util.Collection;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.Locale;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicLongArray;
import java.util.concurrent.locks.LockSupport;

import dev.frozenmilk.sinister.loading.Pinned;

/**
 * A hub {@link PhotonCore} took over at INIT. While Photon is enabled it sends motor-power and servo-pulse writes
 * without waiting for their ACK; every other command, and every command while disabled, takes the SDK's own path.
 */
@Pinned
public final class PhotonLynxModule extends LynxModule {
    private static final int PARALLEL = 1;
    private static final int STOCK_LATE = 2;
    private static final long SLOT_WAIT_NS = 5_000_000L;
    private static final long SLOT_PARK_NS = 50_000L;
    private static final long PING_RESET_NS = 100_000_000L;
    private static final ThreadLocal<Boolean> FAKING_ACK = new ThreadLocal<>();

    private static final Set<String> COPIED = names("lynxUsbDevice", "controllers", "moduleAddress",
            "moduleSerialNumber", "nextMessageNumber", "isParent", "isSystemSynthetic", "isUserModule",
            "systemOperationCounter", "isEngaged", "isOpen", "isNotResponding", "currentSteps", "isVisuallyIdentifying",
            "executor", "pingFuture", "moduleStatusFuture", "attentionRequiredPreviously", "previousModuleStatus",
            "ftdiResetWatchdogActive", "ftdiResetWatchdogActiveWhenEngaged", "bulkCachingMode", "tag");
    private static final Set<String> MERGED = names("commandClasses", "supportedCommands", "interfacesQueried",
            "previousSteps");
    private static final Set<String> OWN = names("addrAndSerialLock", "engagementLock", "startStopLock",
            "unfinishedCommands", "i2cLock", "pingFutureLock", "moduleStatusLock", "bulkCachingLock",
            "bulkCachingHistory", "lastBulkData");

    final PhotonCore core;
    final LynxModule orphan;
    final boolean parallelCapable;
    private final Map<Integer, LynxRespondable> orphanUnfinished;
    private final LynxUsbDeviceImpl impl;
    private final Object engageLock;
    private final Field implEngaged;
    private final Field orphanNotResponding;
    private final Method orphanStopPing;
    private final ConcurrentHashMap<LynxMessage, Integer> routes = new ConcurrentHashMap<>();
    private final ConcurrentHashMap<Integer, ParallelWrite> parallelWrites = new ConcurrentHashMap<>();
    private final AtomicInteger slots = new AtomicInteger();
    private final AtomicBoolean sweeping = new AtomicBoolean();
    private final AtomicBoolean batteryWarned = new AtomicBoolean();
    final WriteProfile profile = new WriteProfile();
    private volatile long lastDatagramNanos;
    private volatile long lastPingResetNanos = System.nanoTime() - PING_RESET_NS;

    @SuppressWarnings("unchecked")
    PhotonLynxModule(PhotonCore core, LynxModule original, LynxUsbDevice usb, PhotonCore.Fields fields)
            throws IllegalAccessException {
        super(usb, original.getModuleAddress(), original.isParent(), original.isUserModule());
        this.core = core;
        orphan = original;
        parallelCapable = original.isParent();
        orphanUnfinished = (Map<Integer, LynxRespondable>) fields.moduleUnfinished.get(original);
        impl = usb.getDelegationTarget();
        engageLock = fields.engageLock.get(impl);
        implEngaged = fields.implEngaged;
        orphanNotResponding = fields.notResponding;
        orphanStopPing = fields.stopPingTimer;
        transplant(original);
    }

    private static Set<String> names(String... names) {
        return new HashSet<>(Arrays.asList(names));
    }

    static String unhandledFields() {
        if (LynxModule.class.getSuperclass() != LynxCommExceptionHandler.class
                || LynxCommExceptionHandler.class.getSuperclass() != Object.class) {
            return "LynxModule's class hierarchy changed; PhotonLynxModule needs reviewing for this SDK";
        }
        for (Class<?> c = LynxModule.class; c != Object.class; c = c.getSuperclass()) {
            for (Field f : c.getDeclaredFields()) {
                if (Modifier.isStatic(f.getModifiers()) || f.isSynthetic()) continue;
                String name = f.getName();
                boolean known = COPIED.contains(name) || MERGED.contains(name) || OWN.contains(name);
                boolean copyable = !COPIED.contains(name) || !Modifier.isFinal(f.getModifiers());
                boolean mergeable = !MERGED.contains(name)
                        || Map.class.isAssignableFrom(f.getType()) || Collection.class.isAssignableFrom(f.getType());
                if (!known || !copyable || !mergeable) {
                    return "LynxModule field " + c.getSimpleName() + "." + name
                            + " is new or changed; PhotonLynxModule needs reviewing for this SDK";
                }
            }
        }
        return null;
    }

    @SuppressWarnings({"unchecked", "rawtypes"})
    private void transplant(LynxModule from) throws IllegalAccessException {
        executor.shutdownNow();
        for (Class<?> c = LynxModule.class; c != Object.class; c = c.getSuperclass()) {
            for (Field f : c.getDeclaredFields()) {
                if (Modifier.isStatic(f.getModifiers()) || f.isSynthetic()) continue;
                f.setAccessible(true);
                if (COPIED.contains(f.getName())) {
                    f.set(this, f.get(from));
                } else if (MERGED.contains(f.getName())) {
                    Object own = f.get(this);
                    Object theirs = f.get(from);
                    if (own instanceof Map) {
                        ((Map) own).clear();
                        ((Map) own).putAll((Map) theirs);
                    } else {
                        ((Collection) own).clear();
                        ((Collection) own).addAll((Collection) theirs);
                    }
                }
            }
        }
    }

    LynxUsbDeviceImpl usbImpl() {
        return impl;
    }

    void restartPingTimer() {
        startPingTimer();
    }

    int inFlight() {
        return slots.get();
    }

    @Override
    public void acquireNetworkTransmissionLock(@NonNull LynxMessage message) throws InterruptedException {
        boolean on = PhotonCore.writeProfiling();
        long t = on ? System.nanoTime() : 0;
        Integer route = routes.get(message);
        if (route != null && route == PARALLEL) return;
        if (route == null && wantsParallel(message)) {
            routes.put(message, PARALLEL);
            lap(on, WriteProfile.ROUTE, t);
            return;
        }
        super.acquireNetworkTransmissionLock(message);
    }

    @Override
    public void sendCommand(LynxMessage command) throws InterruptedException, LynxUnsupportedCommandException {
        Integer route = routes.get(command);
        if (route != null && route == PARALLEL) {
            if (sendParallel((LynxRespondable<?>) command)) return;
            super.acquireNetworkTransmissionLock(command);
            routes.put(command, STOCK_LATE);
        }
        noteStockWrite(command);
        super.sendCommand(command);
    }

    @Override
    public void releaseNetworkTransmissionLock(@NonNull LynxMessage message) throws InterruptedException {
        boolean on = PhotonCore.writeProfiling();
        long t = on ? System.nanoTime() : 0;
        Integer route = routes.remove(message);
        if (route != null && route == PARALLEL) {
            lap(on, WriteProfile.RELEASE, t);
            return;
        }
        super.releaseNetworkTransmissionLock(message);
    }

    @Override
    public void finishedWithMessage(LynxMessage message) {
        if (!(message instanceof ParallelWrite)) {
            super.finishedWithMessage(message);
            return;
        }
        if (LynxUsbDeviceImpl.DEBUG_LOG_DATAGRAMS_FINISH) {
            RobotLog.vv("LynxModule", "finishing mod=%d msg#=%d", message.getModuleAddress(), message.getMessageNumber());
        }
        ParallelWrite write = (ParallelWrite) message;
        if (end(write, ParallelWrite.DONE) && PhotonCore.writeProfiling()) {
            profile.record(WriteProfile.ACK_RTT, System.nanoTime() - write.firstSentNanos);
        }
    }

    @Override
    public void setAttentionRequired(boolean attentionRequired) {
        if (FAKING_ACK.get() != null) return;
        super.setAttentionRequired(attentionRequired);
    }

    @Override
    protected void noteDatagramReceived() {
        lastDatagramNanos = System.nanoTime();
        super.noteDatagramReceived();
        try {
            if (orphanNotResponding.getBoolean(orphan)) orphanNotResponding.setBoolean(orphan, false);
        } catch (IllegalAccessException e) {
            throw new IllegalStateException("LynxModule.isNotResponding is no longer writable", e);
        }
    }

    // The SDK's unresponsive-hub warning keeps whichever module was reported first, which can be the replaced one.
    @Override
    public void noteNotResponding() {
        super.noteNotResponding();
        orphan.noteNotResponding();
    }

    // Any command still sent through the replaced module restarts its keep-alive, and each keep-alive restarts the next.
    void disarmOrphanPing() {
        try {
            orphanStopPing.invoke(orphan, false);
        } catch (IllegalAccessException | InvocationTargetException e) {
            throw new IllegalStateException("couldn't stop the replaced REV Hub #" + getModuleAddress() + "'s keep-alive", e);
        }
    }

    // I2C clients outside the hardware map keep the replaced module, so every I2C transaction on the hub must take its lock.
    @Override
    public <T> T acquireI2cLockWhile(Supplier<T> supplier) throws InterruptedException, RobotCoreException, LynxNackException {
        warnIfClosed();
        return orphan.acquireI2cLockWhile(supplier);
    }

    // The replaced module shares nextMessageNumber, and the SDK's loop only skips numbers pending in this module's map.
    @Override
    protected byte getNewMessageNumber() {
        for (int tries = 0; tries < 256; tries++) {
            byte number = super.getNewMessageNumber();
            if (!orphanUnfinished.containsKey(number & 0xFF)) return number;
        }
        throw new IllegalStateException("REV Hub #" + getModuleAddress() + " has no free message number");
    }

    @Override
    public void onIncomingDatagramReceived(LynxDatagram datagram) {
        int reference = datagram.getReferenceNumber();
        if (orphanUnfinished.containsKey(reference)) {
            noteDatagramReceived();
            // An ACK or NACK names only the reference number, so a reply to a number both modules hold can't be routed.
            if (unfinishedCommands.containsKey(reference)) {
                core.numberClashes.incrementAndGet();
                return;
            }
            core.orphanDatagrams.incrementAndGet();
            orphan.onIncomingDatagramReceived(datagram);
            return;
        }
        if (!PhotonCore.writeProfiling() || !parallelWrites.containsKey(reference)) {
            super.onIncomingDatagramReceived(datagram);
            return;
        }
        long t = System.nanoTime();
        super.onIncomingDatagramReceived(datagram);
        profile.record(WriteProfile.ACK_HANDLING, System.nanoTime() - t);
    }

    private long lap(boolean on, int step, long since) {
        if (!on) return 0;
        long now = System.nanoTime();
        profile.record(step, now - since);
        return now;
    }

    private boolean wantsParallel(LynxMessage message) {
        if (!parallelCapable || !core.parallelAllowed()) return false;
        if (!(message instanceof LynxSetMotorConstantPowerCommand) && !(message instanceof LynxSetServoPulseWidthCommand)) {
            return false;
        }
        if (message.isDangerous() && OpModeManagerImpl.shouldPreventDangerousHardwareAccess()) {
            core.gateFallbacks.incrementAndGet();
            return false;
        }
        return true;
    }

    private boolean sendParallel(LynxRespondable<?> command) throws InterruptedException, LynxUnsupportedCommandException {
        boolean on = PhotonCore.writeProfiling();
        long start = System.nanoTime();
        sweep(start);
        long t = lap(on, WriteProfile.SWEEP, start);
        if (isNotResponding()) {
            core.notRespondingFallbacks.incrementAndGet();
            return false;
        }
        if (!acquireSlot()) {
            core.slotTimeouts.incrementAndGet();
            return false;
        }
        t = lap(on, WriteProfile.SLOT, t);
        ParallelWrite write = null;
        boolean sent = false;
        try {
            command.setMessageNumber(getNewMessageNumber());
            LynxDatagram datagram = new LynxDatagram(command);
            command.setSerialization(datagram);
            t = lap(on, WriteProfile.SERIALIZE, t);
            write = new ParallelWrite(this, command, datagram);
            t = lap(on, WriteProfile.STAND_IN, t);
            supersede(write.key, write);
            unfinishedCommands.put(write.getMessageNumber(), write);
            parallelWrites.put(write.getMessageNumber(), write);
            lap(on, WriteProfile.TRACK, t);
            sent = transmitParallel(write, command, datagram, on);
        } finally {
            if (!sent) {
                if (write == null) slots.decrementAndGet();
                else end(write, ParallelWrite.DISCARDED);
            }
        }
        if (!sent) return false;
        t = on ? System.nanoTime() : 0;
        core.parallelWrites.incrementAndGet();
        boolean stopped = write.dangerous && OpModeManagerImpl.shouldPreventDangerousHardwareAccess();
        if (stopped) {
            // The SDK gate's own NACK: LynxDcMotorController then skips the motor enable it sends after a power write.
            command.onNackReceived(new LynxNack(this, LynxNack.StandardReasonCode.CANCELLED_FOR_SAFETY));
        } else {
            FAKING_ACK.set(Boolean.TRUE);
            try {
                command.onAckReceived(new LynxAck(this, false));
            } finally {
                FAKING_ACK.remove();
            }
        }
        command.forgetSerialization();
        // send()'s await may still NACK-finish this command; msg# 0 is never issued, so that can't drop the stand-in.
        command.setMessageNumber(0);
        lap(on, WriteProfile.FAKE_ACK, t);
        if (stopped) failSafeAfterStop();
        lap(on, WriteProfile.SEND_PARALLEL, start);
        return true;
    }

    private boolean acquireSlot() throws InterruptedException {
        long deadline = System.nanoTime() + SLOT_WAIT_NS;
        while (true) {
            int taken = slots.get();
            if (taken < PhotonCore.maxParallelWrites()) {
                if (slots.compareAndSet(taken, taken + 1)) return true;
                continue;
            }
            if (Thread.interrupted()) throw new InterruptedException("interrupted waiting for a Photon write slot");
            if (System.nanoTime() - deadline >= 0) return false;
            LockSupport.parkNanos(SLOT_PARK_NS);
        }
    }

    private boolean transmitParallel(ParallelWrite write, LynxRespondable<?> command, LynxDatagram datagram, boolean on)
            throws InterruptedException {
        long t = on ? System.nanoTime() : 0;
        Exception failure = null;
        long sentNanos;
        synchronized (engageLock) {
            t = lap(on, WriteProfile.LOCK_WAIT, t);
            if (write.dangerous && OpModeManagerImpl.shouldPreventDangerousHardwareAccess()) {
                core.gateFallbacks.incrementAndGet();
                return false;
            }
            RobotUsbDevice device = impl.getRobotUsbDevice();
            RobotArmingStateNotifier.ARMINGSTATE arming = impl.getArmingState();
            boolean armed = arming == RobotArmingStateNotifier.ARMINGSTATE.ARMED
                    || arming == RobotArmingStateNotifier.ARMINGSTATE.TO_ARMED;
            if (device == null || !armed || impl.getShutdownReason() != SyncdDevice.ShutdownReason.NORMAL || !implEngaged()) {
                core.deviceNotReadyFallbacks.incrementAndGet();
                return false;
            }
            if (LynxUsbDeviceImpl.DEBUG_LOG_DATAGRAMS || LynxUsbDeviceImpl.DEBUG_LOG_MESSAGES) {
                RobotLog.vv("LynxUsb", "xmit'ing: mod=%d cmd=0x%02x(%s) msg#=%d ref#=%d ", getModuleAddress(),
                        write.getCommandNumber(), write.commandName(), write.getMessageNumber(), write.getReferenceNumber());
            }
            t = lap(on, WriteProfile.CHECKS, t);
            try {
                device.write(datagram.toByteArray());
            } catch (RobotUsbException | RuntimeException e) {
                failure = e;
            }
            sentNanos = System.nanoTime();
            if (failure == null) {
                write.setNanotimeLastTransmit(sentNanos);
                write.firstSentNanos = sentNanos;
                command.setNanotimeLastTransmit(sentNanos);
            }
        }
        if (failure != null) {
            core.writeFailures.incrementAndGet();
            RobotLog.ww(PhotonCore.TAG, "parallel write to REV Hub #%d failed (%s); sending it the SDK's way",
                    getModuleAddress(), failure);
            return false;
        }
        if (on) profile.record(WriteProfile.USB_WRITE, sentNanos - t);
        t = on ? System.nanoTime() : 0;
        resetPingTimerIfDue(command, sentNanos);
        lap(on, WriteProfile.PING_RESET, t);
        command.noteHasBeenTransmitted();
        return true;
    }

    // Each reset reschedules the keep-alive 1950 ms out, so skipping resets within 100 ms of the last one only ever pings early.
    private void resetPingTimerIfDue(LynxMessage command, long now) {
        if (now - lastPingResetNanos < PING_RESET_NS) return;
        lastPingResetNanos = now;
        command.resetModulePingTimer();
    }

    private boolean implEngaged() {
        try {
            return implEngaged.getBoolean(impl);
        } catch (IllegalAccessException e) {
            throw new IllegalStateException("LynxUsbDeviceImpl.isEngaged is no longer readable", e);
        }
    }

    private void failSafeAfterStop() throws InterruptedException {
        core.postStopFailSafes.incrementAndGet();
        try {
            failSafe();
        } catch (RobotCoreException | LynxNackException e) {
            throw new IllegalStateException("REV Hub #" + getModuleAddress() + " didn't take its fail-safe after STOP", e);
        }
    }

    private void noteStockWrite(LynxMessage command) {
        if (parallelWrites.isEmpty()) return;
        if (command instanceof LynxFailSafeCommand) {
            for (ParallelWrite w : parallelWrites.values()) w.superseded = true;
        } else if (command instanceof LynxSetMotorConstantPowerCommand || command instanceof LynxSetServoPulseWidthCommand) {
            supersede(ParallelWrite.keyOf(command, command.toPayloadByteArray()), null);
        }
    }

    private void supersede(int key, ParallelWrite newer) {
        for (ParallelWrite w : parallelWrites.values()) {
            if (w != newer && w.key == key) w.superseded = true;
        }
    }

    void sweep(long now) throws InterruptedException {
        if (parallelWrites.isEmpty() || !isOpen() || !sweeping.compareAndSet(false, true)) return;
        try {
            for (ParallelWrite write : parallelWrites.values()) check(write, now);
        } finally {
            sweeping.set(false);
        }
    }

    private void check(ParallelWrite write, long now) throws InterruptedException {
        long sent = write.firstSentNanos;
        if (sent == 0 || write.state.get() != ParallelWrite.PENDING) return;
        if (now - sent >= write.ackTimeoutNanos()) {
            if (write.superseded && lastDatagramNanos - sent > 0) {
                if (end(write, ParallelWrite.SUPERSEDED)) core.supersededWrites.incrementAndGet();
            } else {
                abandon(write);
            }
        } else if (now - write.getNanotimeLastTransmit() >= write.retransmitNanos()) {
            retransmitIfCurrent(write);
        }
    }

    private void retransmitIfCurrent(ParallelWrite write) throws InterruptedException {
        synchronized (engageLock) {
            if (write.state.get() != ParallelWrite.PENDING || write.superseded) return;
            if (write.dangerous && OpModeManagerImpl.shouldPreventDangerousHardwareAccess()) {
                write.superseded = true;
                return;
            }
            retransmit(write);
        }
        core.retransmits.incrementAndGet();
    }

    private void abandon(ParallelWrite write) {
        if (!end(write, ParallelWrite.ABANDONED)) return;
        int address = getModuleAddress();
        RobotLog.ee("LynxModule", "timeout: abandoning waiting %dms for %s: cmd=%s mod=%d msg#=%d", write.msAwait(),
                "ack", write.commandName(), address, write.getMessageNumber());
        RobotLog.ee("LynxModule", "Marking module #%d as unresponsive until we receive some data back", address);
        if (isOpen()) LynxModuleWarningManager.getInstance().reportModuleUnresponsive(this);
        noteNotResponding();
        forgetLastKnown();
        core.abandonedWrites.incrementAndGet();
    }

    void clearParallelWrites() {
        for (ParallelWrite write : parallelWrites.values()) {
            if (end(write, ParallelWrite.CLEARED)) core.clearedAtInit.incrementAndGet();
        }
        batteryWarned.set(false);
    }

    private boolean end(ParallelWrite write, int outcome) {
        int number = write.getMessageNumber();
        unfinishedCommands.remove(number, write);
        parallelWrites.remove(number, write);
        if (!write.state.compareAndSet(ParallelWrite.PENDING, outcome)) return false;
        slots.decrementAndGet();
        return true;
    }

    void onParallelWriteNack(ParallelWrite write, LynxNack nack) {
        LynxNack.ReasonCode reason = nack.getNackReasonCode();
        core.countNack(reason.toString());
        handleException(new LynxNackException(write, "%s: nack received: %s:%d", write.commandName(), reason.toString(),
                reason.getValue()));
        LynxNack.StandardReasonCode code = nack.getNackReasonCodeAsEnum();
        boolean batteryLow = code == LynxNack.StandardReasonCode.BATTERY_TOO_LOW_TO_RUN_MOTOR
                || code == LynxNack.StandardReasonCode.BATTERY_TOO_LOW_TO_RUN_SERVO;
        if (batteryLow && batteryWarned.compareAndSet(false, true)) {
            RobotLog.addGlobalWarningMessage(String.format(Locale.US, "REV Hub #%d refused a %s: %s",
                    getModuleAddress(), write.commandName(), code));
        }
    }

    @Pinned
    static final class ParallelWrite extends LynxRespondable<LynxAck> {
        static final int PENDING = 0;
        static final int DONE = 1;
        static final int ABANDONED = 2;
        static final int SUPERSEDED = 3;
        static final int CLEARED = 4;
        static final int DISCARDED = 5;

        final PhotonLynxModule hub;
        final Class<?> commandClass;
        final int commandNumber;
        final byte[] payload;
        final int key;
        final boolean dangerous;
        final AtomicInteger state = new AtomicInteger(PENDING);
        volatile long firstSentNanos;
        volatile boolean superseded;

        ParallelWrite(PhotonLynxModule hub, LynxRespondable<?> command, LynxDatagram datagram) {
            super(hub);
            this.hub = hub;
            commandClass = command.getClass();
            commandNumber = command.getCommandNumber();
            payload = datagram.getPayloadData();
            key = keyOf(command, payload);
            dangerous = command.isDangerous();
            setMessageNumber(command.getMessageNumber());
            setSerialization(datagram);
        }

        String commandName() {
            return commandClass.getSimpleName();
        }

        static int keyOf(LynxMessage command, byte[] payload) {
            return (command instanceof LynxSetServoPulseWidthCommand ? 0x100 : 0) | (payload[0] & 0xFF);
        }

        long ackTimeoutNanos() {
            return getMsAwaitInterval() * 1_000_000L;
        }

        long retransmitNanos() {
            return getMsRetransmissionInterval() * 1_000_000L;
        }

        int msAwait() {
            return getMsAwaitInterval();
        }

        @Override
        public int getCommandNumber() {
            return commandNumber;
        }

        @Override
        public byte[] toPayloadByteArray() {
            return payload.clone();
        }

        @Override
        public void fromPayloadByteArray(byte[] rgb) {
            throw new UnsupportedOperationException("a parallel write's stand-in is never decoded");
        }

        @Override
        public boolean isDangerous() {
            return dangerous;
        }

        @Override
        public void onNackReceived(LynxNack nack) {
            super.onNackReceived(nack);
            hub.onParallelWriteNack(this, nack);
        }
    }

    @Pinned
    static final class WriteProfile {
        static final String[] STEPS = {"route", "sweep", "slot", "serialize", "standIn", "track", "lockWait", "checks",
                "usbWrite", "pingReset", "fakeAck", "sendParallel", "release", "ackRtt", "ackHandling"};
        static final int ROUTE = 0;
        static final int SWEEP = 1;
        static final int SLOT = 2;
        static final int SERIALIZE = 3;
        static final int STAND_IN = 4;
        static final int TRACK = 5;
        static final int LOCK_WAIT = 6;
        static final int CHECKS = 7;
        static final int USB_WRITE = 8;
        static final int PING_RESET = 9;
        static final int FAKE_ACK = 10;
        static final int SEND_PARALLEL = 11;
        static final int RELEASE = 12;
        static final int ACK_RTT = 13;
        static final int ACK_HANDLING = 14;
        static final long BUCKET_NS = 5_000L;
        static final int BUCKETS = 500;

        private final AtomicLongArray counts = new AtomicLongArray(STEPS.length * (BUCKETS + 1));
        private final AtomicLongArray sums = new AtomicLongArray(STEPS.length);

        void record(int step, long nanos) {
            int bucket = (int) Math.max(0, Math.min(BUCKETS, nanos / BUCKET_NS));
            counts.incrementAndGet(step * (BUCKETS + 1) + bucket);
            sums.addAndGet(step, nanos);
        }

        void reset() {
            for (int i = 0; i < counts.length(); i++) counts.set(i, 0);
            for (int i = 0; i < sums.length(); i++) sums.set(i, 0);
        }

        Map<String, Map<String, Double>> snapshot() {
            Map<String, Map<String, Double>> out = new LinkedHashMap<>();
            for (int step = 0; step < STEPS.length; step++) {
                long[] row = new long[BUCKETS + 1];
                long n = 0;
                for (int b = 0; b <= BUCKETS; b++) {
                    row[b] = counts.get(step * (BUCKETS + 1) + b);
                    n += row[b];
                }
                Map<String, Double> m = new LinkedHashMap<>();
                m.put("count", (double) n);
                if (n > 0) {
                    m.put("meanUs", sums.get(step) / 1e3 / n);
                    m.put("p50Us", percentileUs(row, n, 0.50));
                    m.put("p95Us", percentileUs(row, n, 0.95));
                    m.put("over2500Us", (double) row[BUCKETS]);
                }
                out.put(STEPS[step], m);
            }
            return out;
        }

        private static double percentileUs(long[] row, long n, double p) {
            long rank = (long) Math.ceil(p * n);
            long seen = 0;
            for (int b = 0; b <= BUCKETS; b++) {
                seen += row[b];
                if (seen >= rank) return (b + 0.5) * BUCKET_NS / 1e3;
            }
            return Double.NaN;
        }
    }
}
