package org.firstinspires.ftc.teamcode.opmodes.test.auto;

import static com.pedropathing.api.Paths.*;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.architecture.command.PathCommands;
import org.firstinspires.ftc.teamcode.pedro.BettaConstants;

@Autonomous(name = "ForwardBackTest", group = "Test")
public class ForwardBackTest extends LinearOpMode {

    private static final double ROUTINE_TIMEOUT_MS = 30000;

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56.5205, 19.2527, 90);
    private final Pose forward = poseFactory.of(56.5205, 99.2527, 90);

    private Command autoRoutine() {
        return PathCommands.timeout(sequential(
                PathCommands.follow(follower, forwardPath()),
                PathCommands.follow(follower, backPath())
        ), ROUTINE_TIMEOUT_MS);
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = BettaConstants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
    }

    private Path forwardPath() {
        return line(start, forward).constant(start.heading());
    }

    private Path backPath() {
        return line(forward, start).constant(start.heading());
    }
}
