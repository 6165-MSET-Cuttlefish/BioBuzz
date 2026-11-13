package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import android.graphics.Bitmap;
import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.concurrent.atomic.AtomicInteger;

/** Switches the dashboard off the way slothboard's own Enable/Disable OpMode does, then keeps sending to it. */
@TeleOp(name = "Bench: Dashboard Off", group = "Test")
public class DashboardOffBench extends LinearOpMode {
    static final String CONFIRM = "turn the dashboard off";
    static final String DISABLE = "disable";
    private static final double CAMERA_FPS = 10;

    @Override
    public void runOpMode() {
        BenchReport report = new BenchReport("Dashboard Off", "dashboard_off");
        if (!CONFIRM.equals(BenchIO.param(report.params, "confirm", ""))) {
            throw new IllegalStateException("Bench: Dashboard Off keeps the dashboard off until adb turns it back on: "
                    + BenchIO.RUNNER_HINT + ", with --dashboard-off");
        }
        double onSeconds = BenchIO.param(report.params, "onSeconds", 3);
        double graceSeconds = BenchIO.param(report.params, "graceSeconds", 1);
        double offSeconds = BenchIO.param(report.params, "offSeconds", 10);
        long periodMs = (long) BenchIO.param(report.params, "periodMs", 50);
        Method disable = privateMethod(FtcDashboard.class, DISABLE);
        if (!report.holdUntilStart(this)) return;

        FtcDashboard dashboard = FtcDashboard.getInstance();
        TwoFrames frames = new TwoFrames();
        report.phase("on");
        report.put("enabledAtStart", dashboard.isEnabled());
        dashboard.startCameraStream(frames, CAMERA_FPS);
        report.put("onPackets", stream(report, dashboard, onSeconds, periodMs));
        report.put("onFramesServed", frames.served());
        report.save();
        if (!opModeIsActive()) {
            dashboard.stopCameraStream();
            report.done("stopped before the switch: the dashboard stays on");
            return;
        }

        report.phase("off");
        long t0 = System.nanoTime();
        invoke(disable, dashboard);
        report.put("disableMs", (System.nanoTime() - t0) / 1e6);
        report.put("enabledAfterDisable", dashboard.isEnabled());
        BenchIO.log("dashboard off: disable() returned, isEnabled %s", dashboard.isEnabled());
        sleep((long) (graceSeconds * 1000));
        int framesBefore = frames.served();
        dashboard.startCameraStream(frames, CAMERA_FPS);
        report.put("offPackets", stream(report, dashboard, offSeconds, periodMs));
        report.put("offFramesServed", frames.served() - framesBefore);
        report.put("stoppedEarly", !opModeIsActive());
        dashboard.stopCameraStream();
        report.done("isEnabled " + report.root.get("enabledAtStart") + " -> " + dashboard.isEnabled() + ", "
                + report.root.get("offFramesServed") + " frames served after the switch");
    }

    private int stream(BenchReport report, FtcDashboard dashboard, double seconds, long periodMs) {
        long end = System.nanoTime() + (long) (seconds * 1e9);
        int sent = 0;
        while (opModeIsActive() && System.nanoTime() < end) {
            TelemetryPacket packet = new TelemetryPacket(false);
            report.addTo(packet);
            packet.put(BenchReport.KEY_LOOP, ++sent);
            dashboard.sendTelemetryPacket(packet);
            sleep(periodMs);
        }
        return sent;
    }

    /** The dashboard's private toggle; the OpMode looks it up before START so a renamed member fails at INIT. */
    static Method privateMethod(Class<?> owner, String name) {
        try {
            Method m = owner.getDeclaredMethod(name);
            m.setAccessible(true);
            return m;
        } catch (NoSuchMethodException e) {
            throw new IllegalStateException(owner.getName() + " has no " + name + "(): the pinned dashboard changed "
                    + "(scripts/bench/offline/check_dashboard_api.py names what moved)", e);
        }
    }

    private static void invoke(Method method, Object target) {
        try {
            method.invoke(target);
        } catch (IllegalAccessException e) {
            throw new IllegalStateException("can't call " + method, e);
        } catch (InvocationTargetException e) {
            throw new IllegalStateException(method + " threw", e.getCause());
        }
    }

    /** A camera that alternates two solid frames and counts what the dashboard asked it for. */
    private static final class TwoFrames implements CameraStreamSource {
        private final Bitmap[] frames = {solid(Color.YELLOW), solid(Color.BLUE)};
        private final AtomicInteger served = new AtomicInteger();

        @Override
        public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
            Bitmap frame = frames[served.getAndIncrement() % frames.length];
            continuation.dispatchHere(consumer -> consumer.accept(frame));
        }

        int served() {
            return served.get();
        }

        private static Bitmap solid(int color) {
            Bitmap b = Bitmap.createBitmap(64, 48, Bitmap.Config.ARGB_8888);
            b.eraseColor(color);
            return b;
        }
    }
}
