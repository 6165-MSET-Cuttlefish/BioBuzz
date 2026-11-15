package org.firstinspires.ftc.teamcode.opmodes.test.auto;

import static com.pedropathing.ivy.Scheduler.schedule;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.architecture.hardware.GuardedOctoQuadLocalizer;
import org.firstinspires.ftc.teamcode.pedro.BioBuzzConstants;

public abstract class BarePedroAuto extends LinearOpMode {
    protected Follower follower;

    /** Runs before the follower is built. */
    protected abstract Pose startPose();

    protected abstract Command routine();

    @Override
    public void runOpMode() {
        Scheduler.reset();
        Pose start = startPose();
        follower = BioBuzzConstants.create(this);
        GuardedOctoQuadLocalizer odometry = (GuardedOctoQuadLocalizer) follower.localizer;
        follower.setPose(start);
        Command routine = routine();

        while (opModeInInit()) {
            follower.update();
            telemetry.addData("OctoQuad", odometry.status());
            addFollowerTelemetry(telemetry, follower);
            telemetry.update();
        }
        if (isStopRequested()) return;
        odometry.awaitRunning(this::isStopRequested);
        schedule(routine);

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();
            addFollowerTelemetry(telemetry, follower);
            telemetry.update();
        }
    }

    public static void addFollowerTelemetry(Telemetry telemetry, Follower follower) {
        Pose pose = follower.pose();
        telemetry.addData("X (in)", pose.x());
        telemetry.addData("Y (in)", pose.y());
        telemetry.addData("Heading (deg)", Math.toDegrees(pose.heading()));
        if (follower.currentPath() != null) {
            telemetry.addData("Distance Left On Path (in)", follower.remainingDistance());
        }
    }
}
