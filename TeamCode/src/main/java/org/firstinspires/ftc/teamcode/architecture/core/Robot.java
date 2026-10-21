package org.firstinspires.ftc.teamcode.architecture.core;

import com.acmerobotics.dashboard.FtcDashboard;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.architecture.auto.FieldConfig;
import org.firstinspires.ftc.teamcode.architecture.telemetry.DualTelemetry;

public abstract class Robot {

    public final DualTelemetry telemetry;
    public final EnhancedOpMode opMode;
    public final Follower follower;

    protected Robot(EnhancedOpMode opMode) {
        this.opMode = opMode;
        telemetry = new DualTelemetry(opMode.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.syncDsTransmissionInterval();

        follower = createFollower(opMode.hardwareMap);
        // Placeholder pose on every init; nothing carries over.
        follower.setPose(new Pose(72, FieldConfig.fieldWidthInches - 10, Math.toRadians(90)));
        initializeGameModules();
    }

    /** Runs from the constructor, before {@link #initializeGameModules()} — don't touch subclass fields. */
    protected abstract Follower createFollower(HardwareMap hardwareMap);

    protected abstract void initializeGameModules();
}
