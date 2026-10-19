package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.internal.usb.EthernetOverUsbSerialNumber;

import java.io.IOException;
import java.lang.reflect.Field;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.UnknownHostException;
import java.util.LinkedHashMap;
import java.util.Map;

final class BenchLimelight {
    private BenchLimelight() {}

    static Limelight3A limelightAt(String address) {
        try {
            return new Limelight3A(EthernetOverUsbSerialNumber.fromIpAddress(address, "bench"), "bench-" + address,
                    InetAddress.getByName(address));
        } catch (UnknownHostException e) {
            throw new IllegalStateException("bad bench Limelight address " + address, e);
        }
    }

    static Limelight3A loopbackLimelight(int port) {
        Limelight3A limelight = limelightAt("127.0.0.1");
        String url = "http://127.0.0.1:" + port;
        try {
            field("baseUrl").set(limelight, url);
        } catch (IllegalAccessException e) {
            throw changed("baseUrl", e);
        }
        if (!url.equals(baseUrl(limelight))) throw new IllegalStateException("Limelight3A.baseUrl didn't take " + url);
        return limelight;
    }

    static String baseUrl(Limelight3A limelight) {
        try {
            return (String) field("baseUrl").get(limelight);
        } catch (IllegalAccessException e) {
            throw changed("baseUrl", e);
        }
    }

    /** A failed request returns a default LLStatus, which has no name and a 0 °C temperature. */
    static boolean answered(LLStatus status) {
        return !status.getName().isEmpty() || status.getTemp() > 0;
    }

    static int unusedLoopbackPort() {
        try (ServerSocket s = new ServerSocket(0, 1, InetAddress.getByName("127.0.0.1"))) {
            return s.getLocalPort();
        } catch (IOException e) {
            throw new IllegalStateException("can't find a free loopback port", e);
        }
    }

    static String connectFailure(int port) {
        try (Socket s = new Socket()) {
            s.connect(new InetSocketAddress(InetAddress.getByName("127.0.0.1"), port), 100);
        } catch (IOException e) {
            return e.getClass().getSimpleName();
        }
        throw new IllegalStateException("127.0.0.1:" + port + " accepted a connection; the refused case needs nothing there");
    }

    /**
     * Starts the SDK poller at {@code hz}. Limelight3A ignores setPollRateHz while running and the device outlives the
     * OpMode, so it is stopped first, and the interval is checked because the SDK clamps to 1-250 Hz silently.
     */
    static void startPolling(Limelight3A limelight, int hz) {
        limelight.stop();
        limelight.setPollRateHz(hz);
        long interval = pollIntervalMs(limelight);
        if (interval != 1000 / hz) {
            throw new IllegalStateException("Limelight3A took " + hz + " Hz as a " + interval + " ms poll interval, not "
                    + 1000 / hz + " ms (the SDK clamps to 1-250 Hz)");
        }
        limelight.start();
        if (!limelight.isRunning()) throw new IllegalStateException("Limelight3A.start() didn't start the poller");
    }

    static long pollIntervalMs(Limelight3A limelight) {
        try {
            return field("pollIntervalMs").getLong(limelight);
        } catch (IllegalAccessException e) {
            throw changed("pollIntervalMs", e);
        }
    }

    static Map<String, Object> timeSwitches(Limelight3A limelight, int count, String what) {
        Samples ms = new Samples();
        int accepted = 0;
        for (int i = 0; i < count; i++) {
            long t0 = System.nanoTime();
            if (limelight.pipelineSwitch(4)) accepted++;
            ms.addNanos(System.nanoTime() - t0);
        }
        Map<String, Object> m = new LinkedHashMap<>();
        m.put("what", what);
        m.put("pipelineSwitchMs", ms.summary());
        m.put("returnedTrue", accepted);
        return m;
    }

    private static Field field(String name) {
        try {
            Field f = Limelight3A.class.getDeclaredField(name);
            f.setAccessible(true);
            return f;
        } catch (NoSuchFieldException e) {
            throw changed(name, e);
        }
    }

    private static IllegalStateException changed(String field, Exception cause) {
        return new IllegalStateException("Limelight3A." + field + " changed in this SDK", cause);
    }
}
