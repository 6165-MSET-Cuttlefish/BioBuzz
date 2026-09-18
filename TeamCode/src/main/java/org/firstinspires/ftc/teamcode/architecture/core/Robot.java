package org.firstinspires.ftc.teamcode.architecture.core;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.architecture.auto.FieldConfig;
import org.firstinspires.ftc.teamcode.architecture.auto.PoseRing;
import org.firstinspires.ftc.teamcode.architecture.OptimizationToggles;
import org.firstinspires.ftc.teamcode.architecture.telemetry.DualTelemetry;

/** Game-agnostic robot base. Game subclasses wire up mechanisms in {@link #initializeGameModules()}. */
@Config
public abstract class Robot {

    public final DualTelemetry telemetry;
    public final EnhancedOpMode opMode;
    public final PoseRing poseHistory = new PoseRing(30);
    public Follower follower;
    public TelemetryPacket packet;

    public static TelemetryToggles telemetryToggles = new TelemetryToggles();

    protected Robot(EnhancedOpMode opMode) throws InterruptedException {
        this.opMode = opMode;
        telemetry = new DualTelemetry(opMode.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setDSTransmissionInterval(100);

        follower = createFollower(opMode.hardwareMap);
        // Always start from the configured pose — no pose-carry across a Sloth reload / opmode swap.
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
