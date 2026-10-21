package org.firstinspires.ftc.teamcode.opmodes.test.auto;

import static com.pedropathing.ivy.Scheduler.schedule;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedro.BettaConstants;

public abstract class BarePedroAuto extends LinearOpMode {
    protected Follower follower;

    /** Runs before the follower is built. */
    protected abstract Pose startPose();

    protected abstract Command routine();

    @Override
    public void runOpMode() {
        Scheduler.reset();
        Pose start = startPose();
        follower = BettaConstants.create(hardwareMap);
        follower.setPose(start);
        follower.update();
        Command routine = routine();

        waitForStart();
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
        telemetry.addData("x", pose.x());
        telemetry.addData("y", pose.y());
        telemetry.addData("heading (deg)", Math.toDegrees(pose.heading()));
        if (follower.currentPath() != null) {
            telemetry.addData("Distance left on path (in)", follower.remainingDistance());
        }
    }
}
