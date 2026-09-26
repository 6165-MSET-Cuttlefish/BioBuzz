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

@Autonomous(name = "CloseFlowerTesting", group = "Autonomous")
public class CloseFlowerTesting extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56, 8, 90);
    private final Pose path1 = poseFactory.of(14.9769, 46.9989, 180);
    private final Pose point2 = poseFactory.of(46.9748, 130.0998, 87.3431);
    private final Pose point2Control1 = poseFactory.of(46.0861, 107.8866, 0);
    private final Pose point3 = poseFactory.of(47.0588, 119.9926, 90.4764);
    private final Pose point4 = poseFactory.of(12.8193, 116.458, -126.4199);
    private final Pose point4Control1 = poseFactory.of(32.1145, 132.7542, 0);
    private final Pose point4Control2 = poseFactory.of(24.7069, 132.6712, 0);
    private final Pose point5 = poseFactory.of(12.8172, 99.8613, -90.0073);
    private final Pose point6 = poseFactory.of(20.8141, 11.6828, 175.3979);
    private final Pose point6Control1 = poseFactory.of(30.1901, 90.5672, 0);
    private final Pose point6Control2 = poseFactory.of(31.0646, 65.2041, 0);
    private final Pose point6Control3 = poseFactory.of(38.7353, 9.4086, 0);
    private final Pose point7 = poseFactory.of(10.8466, 11.354, -178.1108);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, path1()),
                follow(follower, path2()),
                follow(follower, path3()),
                follow(follower, path4()),
                follow(follower, path5()),
                follow(follower, path6()),
                follow(follower, path7())
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
        return line(start, path1).linear(start, path1);
    }

    public Path path2() {
        return curve(path1, point2Control1, point2).tangent();
    }

    public Path path3() {
        return line(point2, point3).reverseTangent();
    }

    public Path path4() {
        return curve(point3, point4Control1, point4Control2, point4).tangent();
    }

    public Path path5() {
        return line(point4, point5).tangent();
    }

    public Path path6() {
        return curve(point5, point6Control1, point6Control2, point6Control3, point6).tangent();
    }

    public Path path7() {
        return line(point6, point7).tangent();
    }
}
