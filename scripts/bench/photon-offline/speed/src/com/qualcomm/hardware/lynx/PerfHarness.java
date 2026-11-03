package com.qualcomm.hardware.lynx;

import com.qualcomm.hardware.lynx.commands.LynxDatagram;
import com.qualcomm.hardware.lynx.commands.core.LynxSetMotorConstantPowerCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxSetServoPulseWidthCommand;

import org.firstinspires.ftc.teamcode.architecture.hardware.photon.PhotonCore;
import org.firstinspires.ftc.teamcode.architecture.hardware.photon.PhotonLynxModule;

import java.lang.management.ManagementFactory;
import java.lang.reflect.Field;
import java.lang.reflect.Proxy;
import java.util.Arrays;
import java.util.Map;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;

public class PerfHarness {
    static final int BLOCKS = Integer.getInteger("blocks", 20000);
    static final int WARM = Integer.getInteger("warm", 5000);
    static final com.sun.management.ThreadMXBean MX =
            (com.sun.management.ThreadMXBean) ManagementFactory.getThreadMXBean();

    interface Writer {
        void write(int i) throws Exception;
    }

    static Field f(Class<?> c, String n) throws Exception {
        Field x = c.getDeclaredField(n);
        x.setAccessible(true);
        return x;
    }

    static void drain(Map<?, ?> unfinished) throws InterruptedException {
        long deadline = System.nanoTime() + 1_000_000_000L;
        while (!unfinished.isEmpty()) {
            if (System.nanoTime() > deadline) throw new IllegalStateException("never drained: " + unfinished.size());
            Thread.yield();
        }
    }

    static void bench(String label, Writer w, Map<?, ?> unfinished, PhotonHarness.FakeHub hub) throws Exception {
        long[] perBlock = new long[BLOCKS];
        long tid = Thread.currentThread().getId();
        long execCpu0 = 0;
        for (int b = 0; b < WARM + BLOCKS; b++) {
            drain(unfinished);
            hub.written.clear();
            long a0 = MX.getThreadAllocatedBytes(tid);
            long t0 = System.nanoTime();
            for (int i = 0; i < 6; i++) w.write(b * 6 + i);
            long dt = System.nanoTime() - t0;
            long da = MX.getThreadAllocatedBytes(tid) - a0;
            if (b == WARM - 1) execCpu0 = executorCpuNanos();
            if (b >= WARM) {
                perBlock[b - WARM] = dt;
                allocSum[0] += da;
            }
        }
        long execCpu = executorCpuNanos() - execCpu0;
        Arrays.sort(perBlock);
        System.out.printf("%-34s per write p50 %7.2f us  p10 %7.2f us  min %7.2f us  alloc %6.0f B/write  executor-thread cpu %5.2f us/write%n", label,
                perBlock[BLOCKS / 2] / 6e3, perBlock[BLOCKS / 10] / 6e3, perBlock[0] / 6e3,
                allocSum[0] / (double) BLOCKS / 6, execCpu / 1e3 / BLOCKS / 6);
        allocSum[0] = 0;
    }

    static final long[] allocSum = new long[1];

    static long executorCpuNanos() {
        long sum = 0;
        for (Thread t : Thread.getAllStackTraces().keySet()) {
            if (t.getName().contains("lynx module executor")) sum += Math.max(0, MX.getThreadCpuTime(t.getId()));
        }
        return sum;
    }

    static void send(LynxModule m, int i) throws Exception {
        if (i % 3 == 2) new LynxSetServoPulseWidthCommand(m, i % 2, 1000 + (i & 0xff)).send();
        else new LynxSetMotorConstantPowerCommand(m, i % 4, 1000 + (i & 0xff)).send();
    }

    static ScheduledExecutorService inertExecutor() {
        Object future = Proxy.newProxyInstance(PerfHarness.class.getClassLoader(), new Class<?>[] {ScheduledFuture.class},
                (p, m, a) -> m.getReturnType() == boolean.class ? false : null);
        return (ScheduledExecutorService) Proxy.newProxyInstance(PerfHarness.class.getClassLoader(),
                new Class<?>[] {ScheduledExecutorService.class},
                (p, m, a) -> m.getReturnType().isAssignableFrom(ScheduledFuture.class) ? future : null);
    }

    public static void main(String[] args) throws Exception {
        Field u = sun.misc.Unsafe.class.getDeclaredField("theUnsafe");
        u.setAccessible(true);
        PhotonHarness.UNSAFE = (sun.misc.Unsafe) u.get(null);
        PhotonHarness.prevent(false);
        System.out.println("jvm " + System.getProperty("java.vm.name") + " " + System.getProperty("java.version")
                + " args " + ManagementFactory.getRuntimeMXBean().getInputArguments());

        PhotonHarness.Rig rig = new PhotonHarness.Rig(173);
        if (!rig.takeOver().isEmpty()) throw new IllegalStateException("take-over failed");
        PhotonLynxModule fork = rig.photon(173);
        rig.hub.ackDelayMs = 0;
        PhotonHarness.enablePhoton(true);
        PhotonCore.setMaxParallelWrites(8);
        bench("fork (this worktree)", i -> send(fork, i), fork.unfinishedCommands, rig.hub);

        Field executor = f(LynxModule.class, "executor");
        Object realExecutor = executor.get(fork);
        executor.set(fork, inertExecutor());
        bench("fork, ping-timer reset inert", i -> send(fork, i), fork.unfinishedCommands, rig.hub);
        executor.set(fork, realExecutor);
        PhotonCore.Counters c = PhotonCore.counters();
        System.out.println("fork counters: parallel " + c.parallelWrites + " fallbacks " + c.stockFallbacks()
                + " retransmits " + c.retransmits);

        PhotonHarness.enablePhoton(false);
        if (Boolean.getBoolean("stock")) {
            Class.forName("com.qualcomm.hardware.lynx.StockPhotonPerf").getDeclaredMethod("run").invoke(null);
        } else {
            System.out.println("stock SolversLib 0.3.6: skipped, no org.solverslib:photon:0.3.6 AAR in the Gradle cache");
        }

        PhotonHarness.enablePhoton(false);
        LynxModule sdk = rig.photon(173);
        bench("SDK path (Photon off, ACK at once)", i -> send(sdk, i), sdk.unfinishedCommands, rig.hub);
        System.exit(0);
    }
}
