package org.firstinspires.ftc.teamcode.architecture.core;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.architecture.auto.FieldConfig;
import org.firstinspires.ftc.teamcode.architecture.OptimizationToggles;
import org.firstinspires.ftc.teamcode.architecture.telemetry.DualTelemetry;

/** Game-agnostic robot base. Game subclasses wire up mechanisms in {@link #initializeGameModules()}. */
@Config
public abstract class Robot {

    public final DualTelemetry telemetry;
    public final EnhancedOpMode opMode;
    public final Follower follower;

    public static TelemetryToggles telemetryToggles = new TelemetryToggles();

    protected Robot(EnhancedOpMode opMode) throws InterruptedException {
        this.opMode = opMode;
        opMode.telemetry.setMsTransmissionInterval(100);
        telemetry = new DualTelemetry(opMode.telemetry, FtcDashboard.getInstance().getTelemetry());

        follower = createFollower(opMode.hardwareMap);
        // Placeholder pose on every init; nothing carries over.
        follower.setPose(new Pose(72, FieldConfig.fieldWidthInches - 10, Math.toRadians(90)));
        initializeGameModules();
    }

    /** Runs from the constructor, before {@link #initializeGameModules()} — don't touch subclass fields. */
    protected abstract Follower createFollower(HardwareMap hardwareMap);

    protected abstract void initializeGameModules();

    public static class TelemetryToggles {
        public boolean dsTelemetry = true;
        public boolean dashboardTelemetry = true;
        public boolean voltage = true;
        public boolean current = false;
        public boolean loopProfile = OptimizationToggles.loopProfileTelemetryByDefault;
    }
}
