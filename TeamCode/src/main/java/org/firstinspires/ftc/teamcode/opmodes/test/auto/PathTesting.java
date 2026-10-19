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

@Autonomous(name = "PathTesting", group = "Test")
public class PathTesting extends LinearOpMode {

    private static final double ROUTINE_TIMEOUT_MS = 30000;

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56.5205, 19.2527, 90);
    private final Pose path1 = poseFactory.of(60.6393, 120.2814, 18.4608);
    private final Pose path1Control1 = poseFactory.of(0.112, 47.7363, 0);
    private final Pose path1Control2 = poseFactory.of(2.2459, 101.3197, 0);
    private final Pose point2 = poseFactory.of(95.4475, 19.7897, -45.3374);
    private final Pose point2Control1 = poseFactory.of(64.3497, 50.9262, 0);
    private final Pose point3Start = poseFactory.of(95.4475, 19.7897, -53);
    private final Pose point3 = poseFactory.of(56.5205, 19.6393, 90);

    private Command autoRoutine() {
        return PathCommands.timeout(sequential(
                PathCommands.follow(follower, path1()),
                PathCommands.follow(follower, path2()),
                PathCommands.follow(follower, path3())
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

    private Path path1() {
        return curve(start, path1Control1, path1Control2, path1).tangent();
    }

    private Path path2() {
        return curve(path1, point2Control1, point2).tangent();
    }

    private Path path3() {
        return line(point3Start, point3).linear(point3Start, point3);
    }
}
