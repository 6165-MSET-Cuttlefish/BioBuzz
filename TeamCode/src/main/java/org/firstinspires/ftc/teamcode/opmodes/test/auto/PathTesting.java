package org.firstinspires.ftc.teamcode.opmodes.test.auto;

import static com.pedropathing.api.Paths.*;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedro.BettaConstants;

@Autonomous(name = "PathTesting", group = "Test")
public class PathTesting extends LinearOpMode {

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

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, path1()),
                follow(follower, path2()),
                follow(follower, path3())
        );
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

    public Path path1() {
        return curve(start, path1Control1, path1Control2, path1).tangent();
    }

    public Path path2() {
        return curve(path1, point2Control1, point2).tangent();
    }

    public Path path3() {
        return line(point3Start, point3).linear(point3Start, point3);
    }
}
